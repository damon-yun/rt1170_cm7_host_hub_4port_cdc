/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017,2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_host_config.h"
#include "usb_host.h"
#include "fsl_device_registers.h"
#include "usb_host_cdc.h"
#include "fsl_debug_console.h"
#include "host_cdc.h"
#include "fsl_common.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */
#include "app.h"
#include "fsl_component_serial_manager.h"
#if ((!USB_HOST_CONFIG_KHCI) && (!USB_HOST_CONFIG_EHCI) && (!USB_HOST_CONFIG_OHCI) && (!USB_HOST_CONFIG_IP3516HS))
#error Please enable USB_HOST_CONFIG_KHCI, USB_HOST_CONFIG_EHCI, USB_HOST_CONFIG_OHCI, or USB_HOST_CONFIG_IP3516HS in file usb_host_config.
#endif

#include "usb_phy.h"
#include "kfifo.h"
#include "timers.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USB_CDC_VCOM_KFIFO_SIZE (2048)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void USB_HostClockInit(void);
extern void USB_HostIsrEnable(void);
extern void USB_HostTaskFn(void *param);
void BOARD_InitHardware(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Allocate the memory for the heap. */
#if defined(configAPPLICATION_ALLOCATED_HEAP) && (configAPPLICATION_ALLOCATED_HEAP)
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t ucHeap[configTOTAL_HEAP_SIZE];
#endif

usb_host_handle g_hostHandle;


static uint8_t recvFifoBuffer[4][USB_CDC_VCOM_KFIFO_SIZE];
static uint8_t sendFifoBuffer[4][USB_CDC_VCOM_KFIFO_SIZE];
kfifo_t cdcRecvkfifo;
kfifo_t cdcSendkfifo;
/*******************************************************************************
 * Code
 ******************************************************************************/

void USB_OTG1_IRQHandler(void)
{
    USB_HostEhciIsrFunction(g_hostHandle);
}

void USB_OTG2_IRQHandler(void)
{
    USB_HostEhciIsrFunction(g_hostHandle);
}

void USB_HostClockInit(void)
{
    uint32_t usbClockFreq;
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };
    usbClockFreq = 24000000;
    if (CONTROLLER_ID == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, usbClockFreq);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, usbClockFreq);
    }
    else
    {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, usbClockFreq);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, usbClockFreq);
    }
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
}

void USB_HostIsrEnable(void)
{
    uint8_t irqNumber;

    uint8_t usbHOSTEhciIrq[] = USBHS_IRQS;
    irqNumber                = usbHOSTEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
/* USB_HOST_CONFIG_EHCI */

/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#endif
    EnableIRQ((IRQn_Type)irqNumber);
}

void USB_HostTaskFn(void *param)
{
    USB_HostEhciTaskFunction(param);
}
/*!
 * @brief USB isr function.
 */

/*!
 * @brief host callback function.
 *
 * device attach/detach callback function.
 *
 * @param deviceHandle           device handle.
 * @param configurationHandle attached device's configuration descriptor information.
 * @param event_code           callback event code, please reference to enumeration host_event_t.
 *
 * @retval kStatus_USB_Success              The host is initialized successfully.
 * @retval kStatus_USB_NotSupported         The application don't support the configuration.
 */
usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                           usb_host_configuration_handle configurationHandle,
                           uint32_t event_code)
{
    usb_status_t status;
    status = kStatus_USB_Success;

    switch (event_code & 0x0000FFFFU)
    {
        case kUSB_HostEventAttach:
            status = USB_HostCdcEvent(deviceHandle, configurationHandle, event_code);
            break;
        case kUSB_HostEventNotSupported:
            usb_echo("device not supported.\r\n");
            break;

        case kUSB_HostEventEnumerationDone:
            status = USB_HostCdcEvent(deviceHandle, configurationHandle, event_code);
            break;

        case kUSB_HostEventDetach:
            status = USB_HostCdcEvent(deviceHandle, configurationHandle, event_code);
            break;

        case kUSB_HostEventEnumerationFail:
            usb_echo("enumeration failed\r\n");
            break;

        default:
            break;
    }
    return status;
}

/*!
 * @brief app initialization.
 */
void HOST_UsbInit(void)
{
    status_t status = (status_t)kStatus_SerialManager_Error;

    USB_HostClockInit();

#if ((defined FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    status = USB_HostInit(CONTROLLER_ID, &g_hostHandle, USB_HostEvent);
    if (status != kStatus_USB_Success)
    {
        usb_echo("host init error\r\n");
        return;
    }
    USB_HostIsrEnable();

    usb_echo("host init done\r\n");
    usb_echo("This example requires that the CDC device uses Hardware flow\r\n");
    usb_echo(
        "if the device does't support it, please set USB_HOST_UART_SUPPORT_HW_FLOW to zero and rebuild this "
        "project\r\n");
    usb_echo("Type strings, then the string\r\n");
    usb_echo("will be echoed back from the device\r\n");
}

void usb_host_task(void *hostHandle)
{
    while (1)
    {
        USB_HostTaskFn(hostHandle);
    }
}

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_currRecvBuf[1024];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_currSendBuf[1024];
SemaphoreHandle_t xSemaphore_cdcrecv;
SemaphoreHandle_t xSemaphore_cdcsend;
uint32_t cdc_vcom_recv_idle = 0;
TimerHandle_t xTimer;
/*!
 * @brief host cdc data transfer callback.
 *
 * This function is used as callback function for bulk in transfer .
 *
 * @param param    the host cdc instance pointer.
 * @param data     data buffer pointer.
 * @param dataLength data length.
 * @status         transfer result status.
 */
static void HOST_CdcVcomDataInCallback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status)
{
    cdc_instance_struct_t *cdcInstance = (cdc_instance_struct_t *)param;
#if 0
    if (dataLength)
    {
        usb_echo("%s", s_currRecvBuf);
    }
    if (cdcInstance->bulkInMaxPacketSize == dataLength)
    {
        /* host will prime to receive zero length packet after recvive one maxpacketsize */
        USB_HostCdcDataRecv(cdcInstance->classHandle, NULL, 0, HOST_CdcVcomDataInCallback, cdcInstance);
    } else {
        /* Producer is ready to provide item. */
        xSemaphoreGive(xSemaphore_cdcrecv);
    }
#else
    if (dataLength)
    {
        kfifo_in(&cdcRecvkfifo, data, dataLength);
        cdc_vcom_recv_idle = 0;
        USB_HostCdcDataRecv(cdcInstance->classHandle, data, 512, HOST_CdcVcomDataInCallback, cdcInstance);
    } else {
        cdc_vcom_recv_idle = 1;
    }
#endif
}

/*!
 * @brief host cdc data transfer callback.
 *
 * This function is used as callback function for bulk out transfer .
 *
 * @param param    the host cdc instance pointer.
 * @param data     data buffer pointer.
 * @param dataLength data length.
 * @status         transfer result status.
 */
void HOST_CdcVcomDataOutCallback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status)
{
    cdc_instance_struct_t *cdcInstance = (cdc_instance_struct_t *)param;


}

void HOST_CDCRecvTask(void *param)
{
    cdc_instance_struct_t *cdcInstance = (cdc_instance_struct_t *)param;
    uint8_t buffer[512];
    uint32_t n = 0;
    
    /* Attempt to create a semaphore. */
    xSemaphore_cdcrecv = xSemaphoreCreateBinary();

    if( xSemaphore_cdcrecv == NULL )
    {
        while(1)
            vTaskDelay(5);
    }
    while(1) {
        if (cdcInstance->attachFlag == 1) {
            if (USB_HostCdcDataRecv(cdcInstance->classHandle, (uint8_t *)&s_currRecvBuf[0], 512, HOST_CdcVcomDataInCallback, cdcInstance) != kStatus_USB_Success){

                vTaskDelay(5);
            }
            break;
        } else {
            vTaskDelay(5);
        }
    }
    while(1) {
        if (cdcInstance->attachFlag == 1) {

            memset(buffer, 0, sizeof(buffer));
            n = kfifo_out(&cdcRecvkfifo, buffer, sizeof(buffer));

            if (n) {
                usb_echo("%s", buffer);
            } else {
                taskYIELD();
            }

        } else {
            vTaskDelay(5);
        }
    }
}

void host_cdc_enum_task(void *param)
{
    while (1)
    {
        USB_HostCdcTask(&g_cdc_instance[0]);
        USB_HostCdcTask(&g_cdc_instance[1]);
        USB_HostCdcTask(&g_cdc_instance[2]);
        USB_HostCdcTask(&g_cdc_instance[3]);
    }
}

void vTimerCallback( TimerHandle_t xTimer )
{
    const uint32_t ulMaxExpiryCountBeforeStopping = 10;
    uint32_t ulCount;

    /* Optionally do something if the pxTimer parameter is NULL. */
    configASSERT( xTimer );

    /* The number of times this timer has expired is saved as the
    timer's ID.  Obtain the count. */
    ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );

    if (cdc_vcom_recv_idle == 1) {
        USB_HostCdcDataRecv(g_cdc_instance[1].classHandle, (uint8_t *)&s_currRecvBuf[0], 512, HOST_CdcVcomDataInCallback, &g_cdc_instance[1]);
    }

}

int main(void)
{
    BOARD_ConfigMPU();

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    
    HOST_UsbInit();

    kfifo_init(&cdcRecvkfifo, &(recvFifoBuffer[0][0]), USB_CDC_VCOM_KFIFO_SIZE);
    
    xTimer = xTimerCreate("Recv Timer", 100, pdTRUE, ( void * ) 0, vTimerCallback);

    /* Start the timer.  No block time is specified, and
    even if one was it would be ignored because the RTOS
    scheduler has not yet been started. */
    if( xTimerStart( xTimer, 0 ) != pdPASS )
    {
        /* The timer could not be set into the Active
        state. */
    }

    if (xTaskCreate(usb_host_task, "usb host task", 2000L / sizeof(portSTACK_TYPE), g_hostHandle, 8, NULL) != pdPASS)
    {
        usb_echo("create host task error\r\n");
    }
    
    if (xTaskCreate(host_cdc_enum_task, "host cdc enum task", 2000L / sizeof(portSTACK_TYPE), NULL, 3, NULL) != pdPASS)
    {
        usb_echo("create cdc task error\r\n");
    }    
    
    if (xTaskCreate(HOST_CDCRecvTask, "host cdc recv task", 2048L / sizeof(portSTACK_TYPE), &g_cdc_instance[1], 5, NULL) != pdPASS)
    {
        usb_echo("create host cdc recv task error\r\n");
    }
    

    
    vTaskStartScheduler();
    while (1)
    {
        ;
    }
}

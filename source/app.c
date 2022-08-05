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
}

void usb_host_task(void *hostHandle)
{
    while (1)
    {
        USB_HostTaskFn(hostHandle);
    }
}

extern int DbgConsole_SendDataReliable(uint8_t *ch, size_t size);

void HOST_CDCRecvTask(void *param)
{
    cdc_instance_struct_t *cdcInstance = (cdc_instance_struct_t *)param;
    uint8_t buffer[512];
    uint32_t n = 0;
    
    while(1) {

        memset(buffer, 0, sizeof(buffer));
        n = HOST_CdcVcomRecv(cdcInstance, buffer, sizeof(buffer));

        if (n) {
            DbgConsole_SendDataReliable(buffer, n);
        } else {
            vTaskDelay(10);
        }
    }
}
#define HOST_TASK_SEND_SIZE (32)

void HOST_CDCSendTask(void *param)
{
    cdc_instance_struct_t *cdcInstance = (cdc_instance_struct_t *)param;
    uint8_t buffer[512];
    
    for (int i = 0; i < HOST_TASK_SEND_SIZE; i++) {
        buffer[i] = '0' + i;
    }
    buffer[HOST_TASK_SEND_SIZE - 2] = '\r';
    buffer[HOST_TASK_SEND_SIZE - 1] = '\n';
    
    while(1) {
        HOST_CdcVcomSend(cdcInstance, buffer, HOST_TASK_SEND_SIZE, portMAX_DELAY);
        vTaskDelay(500);
    }
}
void host_cdc_enum_task(void *param)
{
    /* Reset GNSS Board */
    GPIO_WritePinOutput(GPIO5, 11, 0);
    vTaskDelay(500);
    GPIO_WritePinOutput(GPIO5, 11, 1);
    vTaskDelay(500);
    
    while (1)
    {
        USB_HostCdcTask(&g_cdc_instance[0]);
        USB_HostCdcTask(&g_cdc_instance[1]);
        USB_HostCdcTask(&g_cdc_instance[2]);
        USB_HostCdcTask(&g_cdc_instance[3]);
    }
}

int main(void)
{
    BOARD_ConfigMPU();

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    
    HOST_UsbInit();

    memset(g_cdc_instance, 0, sizeof(g_cdc_instance));

    if (xTaskCreate(usb_host_task, "usb host task", 2048L / sizeof(portSTACK_TYPE), g_hostHandle, 4, NULL) != pdPASS)
    {
        usb_echo("create host task error\r\n");
    }
    
    if (xTaskCreate(host_cdc_enum_task, "host cdc enum task", 2048L / sizeof(portSTACK_TYPE), NULL, 2, NULL) != pdPASS)
    {
        usb_echo("create cdc task error\r\n");
    }
    
    if (xTaskCreate(HOST_CDCRecvTask, "host cdc recv task", 2048L / sizeof(portSTACK_TYPE), &g_cdc_instance[0], 5, NULL) != pdPASS)
    {
        usb_echo("create host cdc recv task error\r\n");
    }
    
    if (xTaskCreate(HOST_CDCRecvTask, "host cdc recv task", 2048L / sizeof(portSTACK_TYPE), &g_cdc_instance[1], 5, NULL) != pdPASS)
    {
        usb_echo("create host cdc recv task error\r\n");
    }
    
    if (xTaskCreate(HOST_CDCRecvTask, "host cdc recv task", 2048L / sizeof(portSTACK_TYPE), &g_cdc_instance[2], 5, NULL) != pdPASS)
    {
        usb_echo("create host cdc recv task error\r\n");
    }
    
    if (xTaskCreate(HOST_CDCSendTask, "host cdc send task", 2048L / sizeof(portSTACK_TYPE), &g_cdc_instance[2], 5, NULL) != pdPASS)
    {
        usb_echo("create host cdc recv task error\r\n");
    }    
    
    vTaskStartScheduler();
    while (1)
    {
        ;
    }
}

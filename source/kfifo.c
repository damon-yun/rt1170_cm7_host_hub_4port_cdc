/*
 * A generic kernel FIFO implementation
 */


#include "kfifo.h"


/* is x a power of 2? */
#define is_power_of_2(x)    ((x) != 0 && (((x) & ((x) - 1)) == 0))

#ifndef __cplusplus
	#define max(a,b) (((a) > (b)) ? (a) : (b))
	#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

/*
 * internal helper to calculate the unused elements in a fifo
 */
static inline unsigned int kfifo_unused(struct kfifo *fifo)
{
    return (fifo->mask + 1) - (fifo->in - fifo->out);
}


int kfifo_init(struct kfifo *fifo, void *buffer, unsigned int size)
{
    if (!is_power_of_2(size)) {
        return -1;        
    }
    if (fifo == NULL) {
        return -1;   
    }
    
    fifo->in = 0;
    fifo->out = 0;
    fifo->data = buffer;

    if (size < 2) {
        fifo->mask = 0;
        return -1;
    }
    fifo->mask = size - 1;

    return 0;
}

static void kfifo_copy_in(struct kfifo *fifo, const void *src, unsigned int len, unsigned int off)
{
    unsigned int size = fifo->mask + 1;
    unsigned int l;
    
    off &= fifo->mask;

    l = min(len, size - off);

    memcpy(fifo->data + off, src, l);
    memcpy(fifo->data, src + l, len - l);
}

unsigned int kfifo_in(struct kfifo *fifo, const void *buf, unsigned int len)
{
    unsigned int l;
    
    if (fifo == NULL || buf == NULL) {
        return 0;   
    }
    
    l = kfifo_unused(fifo);
    if (len > l)
        len = l;

    kfifo_copy_in(fifo, buf, len, fifo->in);
    fifo->in += len;
    return len;
}

unsigned int kfifo_focus_in(struct kfifo *fifo, const void *buf, unsigned int len)
{
    unsigned int l;
    
    if (fifo == NULL || buf == NULL) {
        return 0;   
    }
    
    if (len > kfifo_size(fifo))
    	return 0;

    l = kfifo_unused(fifo);
    if (len > l)
        kfifo_skip(fifo, (len - l));

    kfifo_copy_in(fifo, buf, len, fifo->in);
    fifo->in += len;

    return len;
}

static void kfifo_copy_out(struct kfifo *fifo, void *dst, unsigned int len, unsigned int off)
{
    unsigned int size = fifo->mask + 1;
    unsigned int l;

    off &= fifo->mask;
    l = min(len, size - off);

    memcpy(dst, fifo->data + off, l);
    memcpy(dst + l, fifo->data, len - l);
}

unsigned int kfifo_out_peek(struct kfifo *fifo, void *buf, unsigned int len)
{
    unsigned int l;
    
    if (fifo == NULL || buf == NULL) {
        return 0;   
    }
    
    l = fifo->in - fifo->out;
    if (len > l)
        len = l;

    kfifo_copy_out(fifo, buf, len, fifo->out);
    return len;
}

unsigned int kfifo_out(struct kfifo *fifo, void *buf, unsigned int len)
{
    if (fifo == NULL || buf == NULL) {
        return 0;   
    }
    
    len = kfifo_out_peek(fifo, buf, len);
    fifo->out += len;
    return len;
}


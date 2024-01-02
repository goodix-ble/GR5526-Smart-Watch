/*
* Copyright (C) 2017, Shenzhen Goodix Technology Co., Ltd.
* All Rights Reserved.
*/

#ifndef __GR55XX_USB_SERVER_H__
#define __GR55XX_USB_SERVER_H__

#include "gr55xx_usb_composite.h"

/**
 * @defgroup usbd_driver
 * @ingroup Driver
 * @{*/

/* USB FIFO per EP */
#define ENDPOINT_NUM            6
#define GM_USBD_REQS            16
#define USB_EP4_DMA_MODE_EN     1
#define USB_EP5_DMA_MODE_EN     1

#define USB_DEBUG_LOG_ENABLE      0
#define USB_DEBUG_WARRING_ENABLE  1
#define USB_DEBUG_ERROR_ENABLE    1

#if USB_DEBUG_LOG_ENABLE
#define USB_DBG_LOG_LAYER_TAG
#define USB_LOG_BASE(type, format, ...) printf(USB_DBG_LOG_LAYER_TAG format, ##__VA_ARGS__)
#else
#define USB_LOG_BASE(type, format, ...)
#endif
#define USB_LOG(format, ...) USB_LOG_BASE(PRINT_TYPE_UNDEF, format, ##__VA_ARGS__)

#if USB_DEBUG_WARRING_ENABLE
#define USB_DBG_WARRING_LAYER_TAG
#define USB_WARRING_BASE(type, format, ...) printf(USB_DBG_WARRING_LAYER_TAG format, ##__VA_ARGS__)
#else
#define USB_WARRING_BASE(type, format, ...)
#endif
#define USB_WARRING(format, ...) USB_WARRING_BASE(PRINT_TYPE_UNDEF, format, ##__VA_ARGS__)

#if USB_DEBUG_ERROR_ENABLE
#define USB_DBG_ERROR_LAYER_TAG
#define USB_ERROR_BASE(type, format, ...) printf(USB_DBG_ERROR_LAYER_TAG format, ##__VA_ARGS__)
#else
#define USB_ERROR_BASE(type, format, ...)
#endif
#define USB_ERROR(format, ...) USB_ERROR_BASE(PRINT_TYPE_UNDEF, format, ##__VA_ARGS__)


#define min(a,b) ( ((a)>(b)) ? (b):(a) )

#define U16_LOW(x)   ((uint8_t)(x & 0x00FF))
#define U16_HIGH(x)  ((uint8_t)((x & 0xFF00) >>8))

// Masks
#define U32_MASK(OFFSET_, WIDTH_) \
    ((unsigned int)(~(((WIDTH_) < 32 ? 0xFFFFFFFFul : 0x00ul) << (WIDTH_)) \
        << (OFFSET_)))

/* USB EP */
#define USB_EP0                 0
#define USB_EP1                 1
#define USB_EP2                 2
#define USB_EP3                 3
#define USB_EP4                 4
#define USB_EP5                 5

#define USB_EP_OUT              (0u << 7)
#define USB_EP_IN               (1u << 7)

#define USBRM_EP_NUM            U32_MASK(0, 4)
#define USBRM_EP_DIR            U32_MASK(7, 1)
#define USBRM_EP_ADDR           (USBRM_EP_DIR | USBRM_EP_NUM)

/* Define maximum packet size for endpoint 0 */
#define USB_EP0_MAX_PKS         64
#define USB_EP1_MAX_PKS         64
#define USB_EP2_MAX_PKS         64
#define USB_EP3_MAX_PKS         64
#define USB_EP4_MAX_PKS         1023
#define USB_EP5_MAX_PKS         1023

#define USB_TRANSFER_STATUS_SUCCESS 0

/*USB_XFER_LEN_MAX*/
#define USB_XFER_LEN_MAX              (0xFFFF)

/*USB Device status */
#define DEVSTATE_DEFAULT            0
#define DEVSTATE_ADDRESS            1
#define DEVSTATE_CONFIGFS           2
#define DEVSTATE_CONFIGHS           3

enum ep0_state
{
    EP0_IDLE,
    EP0_IN_DATA_PHASE,
    EP0_OUT_DATA_PHASE,
    EP0_END_XFER,
    EP0_STALL,
};

typedef struct _gr55xx_usb_request
{
    struct list_head entry;
    usb_request_t req;
} gr55xx_usb_request_t;

typedef struct _gr55xx_usb_ep
{
    struct usb_ep ep;
    struct list_head queue;
    unsigned char num;
    unsigned char bEndpointAddress;
    int halted;
    gr55xx_usb_request_t *req;
    struct _gr55xx_dev_usb *dev;
    void *driver_data;
} gr55xx_usb_ep_t;

typedef struct _gr55xx_dev_usb
{
    struct usb_gadget gadget;
    struct list_head head;
    struct _gr55xx_usb_ep ep[ENDPOINT_NUM];
    struct usb_gadget_driver *composite_driver;
    int state;
    int config;
    int address;
    int ep0state;

    unsigned req_std : 1;
    unsigned req_config : 1;
    unsigned req_pending : 1;

} gr55xx_dev_usb_t;

#ifdef __cplusplus
extern "C"
{
#endif

struct usb_request *gr55xx_drv_usbd_alloc_request(struct usb_ep *ep);
void gr55xx_drv_usbd_free_request(struct usb_ep *ep, struct usb_request *req);
gr55xx_usb_state_t gr55xx_drv_usbd_enable(struct usb_ep *ep, const struct usb_endpoint_descriptor *desc);
gr55xx_usb_state_t gr55xx_drv_usbd_disable(struct usb_ep *ep);
gr55xx_usb_state_t gr55xx_drv_usbd_queue(struct usb_ep *ep, struct usb_request *req);
gr55xx_usb_state_t gr55xx_drv_usbd_set_halt(struct usb_ep *ep, int value);
gr55xx_usb_state_t gr55xx_drv_usbd_register_gadget_driver(struct usb_gadget_driver *driver);
uint32_t gr55xx_drv_usbd_driver_data(struct usb_ep *ep);
void gr55xx_drv_usbd_init(void);
void gr55xx_drv_usb_get_wakeup_src_status(void);
void gr55xx_drv_usb_cfg_wakeup_from_deepsleepmode1(void);

#ifdef __cplusplus
}
#endif

#endif /*__DRV_USBD_H__*/


/*
 * Copyright (C) 2021, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
 */

#include <string.h>
#include "gr55xx_usb_composite.h"
#include "gr55xx_usb_cdc.h"
#include "gr55xx_usb_server.h"

struct cdc_device
{
    struct usb_function func;

    struct usb_ep       *notify;
    struct usb_request  *notify_req;

    struct usb_ep       *in;
    struct usb_ep       *out;

    struct list_head    write_pool;
    struct list_head    read_pool;

    struct usb_cdc_line_coding code;

    unsigned mode:1;

    uint32_t state;
    int rcved_command;
    int connected;
};

static struct cdc_device g_cdc;
static uint8_t g_cdc_notify_buffer[CDC_BUFFER_SIZE];

static uint8_t g_in_data_buf[RX_BUF_SIZE];
static uint8_t g_out_data_buf[TX_BUF_SIZE];
bool isp_usb_boot_flag = false;


static struct usb_device_descriptor g_cdc_device_desc =
{
    .bLength            =   USB_DT_DEVICE_SIZE,
    .bDescriptorType    =   USB_DT_DEVICE,
    .bcdUSB             =   0x0200,
    .bDeviceClass       =   USB_CLASS_COMM,
    .bDeviceSubClass    =   0x02,
    .idVendor           =   CDC_VENDOR_ID,
    .idProduct          =   CDC_PRODUCT_ID,
    .bcdDevice          =   CDC_DEVICE_BCD,
    .iManufacturer      =   0,
    .iProduct           =   0,
    .iSerialNumber      =   0,
    .bNumConfigurations =   1,
};

static struct usb_interface_descriptor g_cdc_intf_desc_comm =
{
    .bLength            =   USB_DT_INTERFACE_SIZE,
    .bDescriptorType    =   USB_DT_INTERFACE,

    .bNumEndpoints      =   1,
    .bInterfaceClass    =   USB_CDC_CLASS_COMM,
    .bInterfaceSubClass =   USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol =   USB_CDC_PROTOCOL_V25TER,
};


static struct cdc_header_descriptor g_cdc_header =
{
    0x05,
    USB_CDC_CS_INTERFACE,
    USB_CDC_SCS_HEADER,
    0x0110,
};

static struct cdc_call_mgmt_descriptor g_cdc_mgmt =
{
    0x05,
    USB_CDC_CS_INTERFACE,
    USB_CDC_SCS_CALL_MGMT,
    0x00,
    1,
};

static struct cdc_acm_descriptor g_cdc_acm =
{
    0x04,
    USB_CDC_CS_INTERFACE,
    USB_CDC_SCS_ACM,
    0x02,
};

static struct cdc_union_descriptor g_cdc_union =
{
    0x05,
    USB_CDC_CS_INTERFACE,
    USB_CDC_SCS_UNION,
    0,
    1
};

static struct usb_endpoint_descriptor g_cdc_int_desc =
{
    .bLength            =   USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,

    .bEndpointAddress   =   USB_DIR_IN,
    .bmAttributes       =   USB_ENDPOINT_XFER_INT,
    .bInterval          =   32,
    /* wMaxPacketSize set by autoconfiguration */
};

static struct usb_interface_descriptor g_cdc_intf_desc_data =
{
    .bLength            =   USB_DT_INTERFACE_SIZE,
    .bDescriptorType    =   USB_DT_INTERFACE,

    .bNumEndpoints      =   2,
    .bInterfaceClass    =   USB_CLASS_CDC_DATA,
    .bInterfaceSubClass =   0,
    .bInterfaceProtocol =   0,
};

static struct usb_endpoint_descriptor g_cdc_bulk_out_desc =
{
    .bLength            =   USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,

    .bEndpointAddress   =   USB_DIR_OUT,
    .bmAttributes       =   USB_ENDPOINT_XFER_BULK,
    /* wMaxPacketSize set by autoconfiguration */
};

static struct usb_endpoint_descriptor g_cdc_bulk_in_desc =
{
    .bLength            =   USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,

    .bEndpointAddress   =   USB_DIR_IN,
    .bmAttributes       =   USB_ENDPOINT_XFER_BULK,
    /* wMaxPacketSize set by autoconfiguration */
};

static struct usb_descriptor_header *g_cdc_function[] =
{
    (struct usb_descriptor_header *) &g_cdc_intf_desc_comm,
    (struct usb_descriptor_header *) &g_cdc_header,
    (struct usb_descriptor_header *) &g_cdc_mgmt,
    (struct usb_descriptor_header *) &g_cdc_acm,
    (struct usb_descriptor_header *) &g_cdc_union,
    (struct usb_descriptor_header *) &g_cdc_int_desc,
    (struct usb_descriptor_header *) &g_cdc_intf_desc_data,
    (struct usb_descriptor_header *) &g_cdc_bulk_in_desc,
    (struct usb_descriptor_header *) &g_cdc_bulk_out_desc,
    NULL,
};

static char g_cdc_vendor_label[]     = "Goodix";
static char g_cdc_product_label[]    = "Gr552x CDC";
static char g_cdc_config_label[]     = "ACM";

static struct usb_string g_cdc_strings[] =
{
    [STRING_DESCRIPTION_IDX].s  = g_cdc_config_label,
    [STRING_PRODUCT_IDX].s      = g_cdc_product_label,
    [STRING_MANUFACTURER_IDX].s = g_cdc_vendor_label,
};

static struct usb_gadget_strings g_cdc_device_strings =
{
    .language   = 0x0409,   /* en-us */
    .strings    = g_cdc_strings,
};

static struct usb_string g_cdc_en_us_strings[] =
{
    [CDC_STRING_INTERFACE_COMM].s = "CDC Abstract Control Model (ACM)",
    [CDC_STRING_INTERFACE_DATA].s = "CDC ACM Data",
};

static struct usb_gadget_strings g_cdc_function_strings =
{
    .language   = 0x0409,   /* en-us */
    .strings    = g_cdc_en_us_strings,
};

static void cdc_write_complete(struct usb_ep *ep, struct usb_request *req)
{
#if USB_TEST_PRINTF_OPEN
    printf("[%s] %d", __FUNCTION__, req->actual);
#endif

    list_add_tail(&req->list, &g_cdc.write_pool);

    return;
}

static void cdc_read_complete(struct usb_ep *ep, struct usb_request *req)
{
#if USB_TEST_PRINTF_OPEN
   printf("[%s] %d", __FUNCTION__, req->actual);
#endif

    list_add_tail(&req->list, &g_cdc.read_pool);


    return;
}

gr55xx_usb_state_t gr55xx_usb_cdc_open(uint32_t flags)
{
    g_cdc.mode = 0;
    if(flags  == 1)
    {
        g_cdc.mode = 1;
    }

    g_cdc.state |= CDC_STATUS_ACTIVATED;

    return GR_DRV_USB_OK;
}

gr55xx_usb_state_t gr55xx_usb_cdc_map_ioctx(gr_ep_dir_e dir, struct gr_io_ctx_t *ctx)
{
    struct usb_request *req;

    if (!ctx)
    {
        return GR_DRV_USB_ERROR_PARAMETER;
    }

    if ((g_cdc.state & CDC_STATUS_ACTIVATED) != CDC_STATUS_ACTIVATED)
    {
        return GR_DRV_USB_ERROR_NOT_READY;
    }

    switch (dir)
    {
        case GR_EP_DIR_IN:
        {
            req = gr55xx_usb_alloc_request(g_cdc.in);
            if (!req)
            {
                return GR_DRV_USB_ERROR_NOMEM;
            }

            req->buf = ctx->buf;
            req->length = ctx->length;
            req->complete = cdc_write_complete;

            list_add_tail(&req->list, &g_cdc.write_pool);

            break;
        }
        case GR_EP_DIR_OUT:
        {
            req = gr55xx_usb_alloc_request(g_cdc.out);
            if (!req)
            {
                return GR_DRV_USB_ERROR_NOMEM;
            }

            req->buf = ctx->buf;
            req->length = ctx->length;
            req->complete = cdc_read_complete;

            if (g_cdc.state & CDC_STATUS_CONNECTED)
            {
                gr55xx_usb_ep_queue(g_cdc.out, req);
            }
            else
            {
                list_add_tail(&req->list, &g_cdc.read_pool);
            }


            break;
        }
        default:
        {
            USB_ERROR("ERROR: unknown memmap parameter( %d ).", dir);
            return GR_DRV_USB_ERROR;
        }
    }

    g_cdc.state |= CDC_STATUS_MMAPPED;

    return GR_DRV_USB_OK;
}

gr55xx_usb_state_t gr55xx_usb_cdc_dequeue(gr_ep_dir_e dir, struct gr_io_ctx_t **ctx)
{
    struct usb_request *req = NULL;

    if (!ctx)
    {
        return GR_DRV_USB_ERROR_PARAMETER;
    }

    if ((g_cdc.state & CDC_READY_BITS) != CDC_READY_BITS)
    {
        return GR_DRV_USB_ERROR_NOT_READY;
    }

    switch (dir)
    {
        case GR_EP_DIR_IN:
        {
            do {
                if (!list_empty(&g_cdc.write_pool))
                {
                    req= list_entry(g_cdc.write_pool.next, struct usb_request, list);
                }
            } while (!req && !g_cdc.mode);

            *ctx = (struct gr_io_ctx_t *)req;

            break;
        }
        case GR_EP_DIR_OUT:
        {
            do {
                if (!list_empty(&g_cdc.read_pool))
                {
                    req= list_entry(g_cdc.read_pool.next, struct usb_request, list);
                }
            } while (!req && !g_cdc.mode);

            *ctx = (struct gr_io_ctx_t *)req;

            break;
        }
        default:
        {
            USB_ERROR("ERROR: unknown memmap parameter( %d ).", dir);
            return GR_DRV_USB_ERROR;
        }
    }

    return (req ? GR_DRV_USB_OK : GR_DRV_USB_ERROR_RETRY);
}

gr55xx_usb_state_t gr55xx_usb_cdc_enqueue(gr_ep_dir_e dir, struct gr_io_ctx_t *ctx)
{
    gr55xx_usb_state_t ret;

    if (!ctx)
    {
        return GR_DRV_USB_ERROR_PARAMETER;
    }

    if((g_cdc.state & CDC_READY_BITS) != CDC_READY_BITS)
    {
        return GR_DRV_USB_ERROR_NOT_READY;
    }

    switch (dir)
    {
        case GR_EP_DIR_IN:
        {
            ret = gr55xx_usb_ep_queue(g_cdc.in, (struct usb_request *)ctx);
            break;
        }
        case GR_EP_DIR_OUT:
        {
            ret = gr55xx_usb_ep_queue(g_cdc.out, (struct usb_request *)ctx);
            break;
        }
        default:
        {
            USB_ERROR("ERROR: unknown memmap parameter( %d ).", dir);
            return GR_DRV_USB_ERROR;
        }
    }

    return ret;
}

static void cdc_notify_complete(struct usb_ep *ep, struct usb_request *req)
{
    return;
}

static int g_cdc_function_bind(struct usb_configuration *c, struct usb_function *f)
{
    struct usb_ep *ep;
    int ret = GR_DRV_USB_ERROR;

    struct usb_composite_dev *cdev = c->cdev;

    /* Allocate endpoints. */
#if USB_CDC_BULK_IN_EP == USB_EP2
    ep = gr55xx_usb_ep_find(cdev->gadget, "ep3");
#else
    ep = gr55xx_usb_ep_find(cdev->gadget, "ep2");
#endif

    if (!ep)
    {
        USB_ERROR("ERORR: [%s]: Unable to allocate notify EP", __FUNCTION__);
        goto error;
    }
    USB_LOG("NOTIFY EP: %s\r\n",  ep->name);

#if USB_CDC_BULK_IN_EP == USB_EP2
    g_cdc_int_desc.bEndpointAddress |= 3;
#else
    g_cdc_int_desc.bEndpointAddress |= 2;
#endif
    g_cdc_int_desc.wMaxPacketSize = ep->maxpacket;
    g_cdc.notify = ep;
    ep->driver_data = (void *)&g_cdc;

    /* Allocate endpoints. */
    ep = gr55xx_usb_ep_find(cdev->gadget, "ep1");
    if (!ep)
    {
        USB_ERROR("ERROR: Unable to allocate bulk_out_desc EP");
        goto error;
    }
    USB_LOG("BULK OUT EP: %s\r\n",  ep->name);

    g_cdc_bulk_out_desc.bEndpointAddress |= 1;
    g_cdc_bulk_out_desc.wMaxPacketSize = ep->maxpacket;
    g_cdc.out = ep;
    ep->driver_data = (void *)&g_cdc;

    /* Allocate endpoints. */
#if USB_CDC_BULK_IN_EP == USB_EP2
    ep = gr55xx_usb_ep_find(cdev->gadget, "ep2");
#else
    ep = gr55xx_usb_ep_find(cdev->gadget, "ep3");
#endif

    if (!ep)
    {
        USB_ERROR("ERROR: Unable to allocate bulk_in_desc EP");
        goto error;
    }
    USB_LOG("BULK IN EP: %s\r\n",  ep->name);

#if USB_CDC_BULK_IN_EP == USB_EP2
    g_cdc_bulk_in_desc.bEndpointAddress |= 2;
#else
    g_cdc_bulk_in_desc.bEndpointAddress |= 3;
#endif
    g_cdc_bulk_in_desc.wMaxPacketSize = ep->maxpacket;
    g_cdc.in = ep;
    ep->driver_data = (void *)&g_cdc;

    if ((ret = gr55xx_usb_interface_id(c, f)) < 0)
    {
        goto error;
    }
    g_cdc_intf_desc_comm.bInterfaceNumber = ret;
    g_cdc_union.master_interface = ret;

    if ((ret = gr55xx_usb_interface_id(c, f)) < 0)
    {
        goto error;
    }
    g_cdc_intf_desc_data.bInterfaceNumber = ret;
    g_cdc_union.slave_interface0 = ret;
    g_cdc_mgmt.data_interface = ret;

    ret = GR_DRV_USB_OK;
    f->descriptors = gr55xx_usb_copy_descriptors(g_cdc_function);
    if (!f->descriptors)
    {
        ret = GR_DRV_USB_ERROR_NOMEM;
        goto error;
    }

    g_cdc.notify_req = gr55xx_usb_alloc_request(g_cdc.notify);
    if (!g_cdc.notify_req)
    {
        ret = GR_DRV_USB_ERROR_NOMEM;
        goto error;
    }
    g_cdc.notify_req->buf = &g_cdc_notify_buffer[0];
    g_cdc.notify_req->complete = cdc_notify_complete;

error:
    return ret;
}

static void cdc_complete_set_line_coding(struct usb_ep *ep, struct usb_request *req)
{
    struct usb_cdc_line_coding *value = (struct usb_cdc_line_coding *)&req->buf[0];
    g_cdc.code = *value;
    USB_LOG("dwDTERate:%d bCharFormat:%d bParityType:%d bDataBits:%d", value->dwDTERate, value->bCharFormat, value->bParityType, value->bDataBits);
    return;
}

static int g_cdc_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
    struct usb_ep *ep = g_cdc.func.config->cdev->gadget->ep0;
    struct usb_request *req = g_cdc.func.config->cdev->req;

    uint16_t w_length = ctrl->wLength;
    int value = GR_DRV_USB_ERROR_NOTSUPP;

    USB_LOG(" cdc setup request %02x %02x value %04x index %04x %04x",ctrl->bRequestType, ctrl->bRequest, (ctrl->wValue),(ctrl->wIndex), (ctrl->wLength));
    switch(ctrl->bRequest)
    {
        case CDC_SET_LINE_CODING:
            value = w_length;
            req->complete = cdc_complete_set_line_coding;
            printf("***************************CDC_SET_LINE_CODING***************************\r\n");
            break;
        case CDC_GET_LINE_CODING:
            USB_LOG("***************************CDC_GET_LINE_CODING***************************\r\n");
            value = min(w_length, sizeof(struct usb_cdc_line_coding));
            memcpy(req->buf, &g_cdc.code, value);
            break;
        case CDC_SET_CONTROL_LINE_STATE:
            USB_LOG("***************************CDC_SET_CONTROL_LINE_STATE***************************\r\n");
            value = 0;
            break;

        default:
            USB_ERROR("ERROR: unsupport cdc request:%02x %02x %04x %04x %d\r\n",
                    ctrl->bRequestType, ctrl->bRequest,ctrl->wValue, ctrl->wIndex, w_length);
            break;
    }

    /* respond with data transfer or status phase? */
    if (value >= 0) {
        req->zero = 0;
        req->length = value;
        value = gr55xx_usb_ep_queue(ep, req);
        if (value < 0)
        {
            USB_ERROR("ERROR: cdc response error %d!\r\n", value);
        }
    }

    return value;
}

static int g_cdc_function_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
    if (intf == 0)
    {
        gr55xx_usb_enable(g_cdc.notify, &g_cdc_int_desc);
    }
    else if (intf == 1)
    {
        gr55xx_usb_enable(g_cdc.out, &g_cdc_bulk_out_desc);
        gr55xx_usb_enable(g_cdc.in, &g_cdc_bulk_in_desc);

        g_cdc.state |= CDC_STATUS_CONNECTED;
    }

    return GR_DRV_USB_OK;
}

static int g_cdc_function_disable(struct usb_function *f)
{
    return GR_DRV_USB_OK;
}

static int  cdc_config_bind(struct usb_configuration *c)
{
    int ret;

    memset(&g_cdc, 0x00, sizeof(struct cdc_device));

    INIT_LIST_HEAD(&g_cdc.read_pool);
    INIT_LIST_HEAD(&g_cdc.write_pool);

    if ((ret = gr55xx_usb_string_id(c->cdev)) < 0)
    {
        goto error;
    }
    g_cdc_en_us_strings[CDC_STRING_INTERFACE_COMM].id = ret;
    g_cdc_intf_desc_comm.iInterface = ret;

    if ((ret = gr55xx_usb_string_id(c->cdev)) < 0)
    {
        goto error;
    }
    g_cdc_en_us_strings[CDC_STRING_INTERFACE_DATA].id = ret;
    g_cdc_intf_desc_data.iInterface = ret;

    g_cdc.func.name           = "CDC Function";
    g_cdc.func.strings        = &g_cdc_function_strings;
    g_cdc.func.bind           = g_cdc_function_bind;
    g_cdc.func.setup          = g_cdc_function_setup;
    g_cdc.func.disable        = g_cdc_function_disable;
    g_cdc.func.set_alt        = g_cdc_function_set_alt;

    ret = gr55xx_usb_add_function(c, &g_cdc.func);

error:
    return ret;
}

static struct usb_configuration g_cdc_config_driver =
{
    .label                      = "CDC Serial",
    .bind                       = cdc_config_bind,
    .bConfigurationValue        = 1,
    .iConfiguration             = 0, /* dynamic */
    .bmAttributes               = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_WAKEUP,
    .bMaxPower                  = 0x32,
};

static int  cdc_bind(struct usb_composite_dev *cdev)
{
    int ret;

    if ((ret = gr55xx_usb_string_id(cdev)) < 0)
    {
        goto error;
    }
    g_cdc_strings[STRING_MANUFACTURER_IDX].id = ret;
    g_cdc_device_desc.iManufacturer = ret;

    if ((ret = gr55xx_usb_string_id(cdev)) < 0)
    {
        goto error;
    }
    g_cdc_strings[STRING_PRODUCT_IDX].id = ret;
    g_cdc_device_desc.iProduct = ret;
    g_cdc_device_desc.iSerialNumber = 3;

    if ((ret = gr55xx_usb_string_id(cdev)) < 0)
    {
        goto error;
    }
    g_cdc_strings[STRING_DESCRIPTION_IDX].id = ret;
    g_cdc_config_driver.iConfiguration = ret;

    if ((ret = gr55xx_usb_add_config(cdev, &g_cdc_config_driver)) < 0)
    {
        goto error;
    }

    USB_LOG("[%s]: Goodix CDC Gadget", __FUNCTION__);

error:

    return ret;
}



__weak void gr55xx_usb_cdc_read_data(uint8_t *p_data, uint32_t size)
{

}

//cdc read or write data from data ep
static int cdc_ep_callback(struct usb_gadget *gadget,struct usb_ep *ep)
{
    int retval = GR_DRV_USB_ERROR;
    struct usb_request *req;

    if(!list_empty(&g_cdc.read_pool) && (ep->address == USB_EP1))
    {
        req = list_entry(g_cdc.read_pool.next, struct usb_request, list);
        ep->driver_data = req->buf;
        memset(req->buf,0x00,RX_BUF_SIZE);

        retval = gr55xx_drv_usbd_driver_data(ep);
        gr55xx_usb_cdc_read_data(ep->driver_data,retval);
    }
    return retval;
}

static struct usb_composite_driver g_cdc_driver =
{
    .name           = "cdc driver",
    .dev            = &g_cdc_device_desc,
    .strings        = &g_cdc_device_strings,
    .bind           = cdc_bind,
    .ep_callback    = cdc_ep_callback,
};


gr55xx_usb_state_t gr55xx_usb_cdc_init(void)
{
    struct gr_io_ctx_t ctx;
    memset(&g_cdc, 0x0, sizeof(g_cdc));
    gr55xx_usb_composite_register(&g_cdc_driver);

    gr55xx_usb_cdc_open(1);

    ctx.buf =  &g_out_data_buf[0];
    ctx.length = TX_BUF_SIZE;
    gr55xx_usb_cdc_map_ioctx(GR_EP_DIR_IN, &ctx);

    ctx.buf = &g_in_data_buf[0];
    ctx.length = RX_BUF_SIZE;
    gr55xx_usb_cdc_map_ioctx(GR_EP_DIR_OUT, &ctx);

    return GR_DRV_USB_OK;
}

gr_cdc_state_e gr55xx_usb_cdc_state(void)
{
    gr_cdc_state_e cdc_state;

    if ((g_cdc.state & CDC_READY_BITS) != CDC_READY_BITS)
    {
        cdc_state = GR_CDC_UART_NONE;
    }
    else
    {
        cdc_state = GR_CDC_UART_INSERT;
    }

    return cdc_state;
}


void gr55xx_usb_cdc_send_data(uint8_t *p_data, uint32_t size)
{
    uint8_t *p_dataTemp;
    uint32_t size_temp;
    gr55xx_usb_state_t res;
    struct gr_io_ctx_t *txctx;
    uint32_t timeout = 0;

    p_dataTemp = p_data;
    size_temp = size;
    timeout = 0XFFFFFFF;
    do {
        if ((res = gr55xx_usb_cdc_dequeue(GR_EP_DIR_IN, &txctx)) == GR_DRV_USB_OK)
        break;
    } while((res == GR_DRV_USB_ERROR_RETRY) && (timeout--));

    if(size_temp > TX_BUF_SIZE)
    {
        for(;size_temp > TX_BUF_SIZE;)
        {
            memset(&txctx->buf[0],0x00,TX_BUF_SIZE);
            memcpy(&txctx->buf[0],p_dataTemp,TX_BUF_SIZE);
            txctx->length = TX_BUF_SIZE;

            timeout = 0XFFFFFFF;
            do {
                if ((res = gr55xx_usb_cdc_enqueue(GR_EP_DIR_IN, txctx)) == GR_DRV_USB_OK)
                break;
            } while ((res == GR_DRV_USB_ERROR_RETRY) && (timeout--));
            p_dataTemp += TX_BUF_SIZE;
            size_temp -= TX_BUF_SIZE;
        }

        memset(txctx->buf,0x00,TX_BUF_SIZE);
        memcpy(&txctx->buf[0],p_dataTemp,size_temp);
        txctx->length = size_temp;
    }
    else
    {
        memset(txctx->buf,0x00,TX_BUF_SIZE);
        memcpy(&txctx->buf[0],p_dataTemp,size_temp);
        txctx->length = size_temp;
    }

    timeout = 0XFFFFFFF;
    do {
        if ((res = gr55xx_usb_cdc_enqueue(GR_EP_DIR_IN, txctx)) == GR_DRV_USB_OK)
        break;
    } while ((res == GR_DRV_USB_ERROR_RETRY) && (timeout--));
}

void hal_usb_detach_callback(usb_handle_t *p_usb)
{
    if((g_cdc.state & (CDC_STATUS_MMAPPED | CDC_STATUS_ACTIVATED)) == (CDC_STATUS_MMAPPED | CDC_STATUS_ACTIVATED))
    {
        g_cdc.state = CDC_STATUS_ACTIVATED | CDC_STATUS_MMAPPED;
    }
}

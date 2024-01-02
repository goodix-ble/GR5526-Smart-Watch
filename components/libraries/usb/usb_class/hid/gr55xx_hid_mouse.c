/*
 * Copyright (C) 2017, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
 */

#include <string.h>
#include "gr55xx_usb_composite.h"
#include "gr55xx_hid_mouse.h"
#include "gr55xx_usb_server.h"

#define HID_MOUSE_VENDOR_ID                   0x27C6
#define HID_MOUSE_PRODUCT_ID                  0x2100
#define HID_DEVICE_BCD                  0x0100  /* 1.00 */

#define HID_STRING_INTERFACE_MOUSE      0

#define HID_BUFFER_SIZE                 64

#define HID_STATUS_CONNECTED            0x00000001
#define HID_STATUS_ACTIVATED            0x00000002
#define HID_STATUS_MMAPPED              0x00000004

#define HID_MOUSE_HID_DESC_LEN          0x09

#define HID_READY_BITS                  (HID_STATUS_CONNECTED | HID_STATUS_ACTIVATED | HID_STATUS_MMAPPED)

uint8_t ReportDescriptor_mouse[] =
{
    0x05 , 0x01,
    0x09 , 0x02,
    0xA1 , 0x01,
    0x09 , 0x01,
    0xA1 , 0x00,
    0x05 , 0x09,
    0x19 , 0x01,
    0x29 , 0x03,
    0x15 , 0x00,
    0x25 , 0x01,
    0x95 , 0x03,
    0x75 , 0x01,
    0x81 , 0x02,
    0x95 , 0x01,
    0x75 , 0x05,
    0x81 , 0x03,
    0x05 , 0x01,
    0x09 , 0x30,
    0x09 , 0x31,
    0x09 , 0x38,
    0x15 , 0x81,
    0x25 , 0x7F,
    0x75 , 0x08,
    0x95 , 0x03,
    0x81 , 0x06,
    0xC0 ,
    0xC0
};

struct hid_mouse_device
{
    struct usb_function func;

    struct usb_ep       *notify;
    struct usb_request  *notify_req;

    struct usb_ep       *in;
    struct usb_ep       *out;

    struct list_head    write_pool;
    struct list_head    read_pool;

    uint32_t state;
    int rcved_command;
    int connected;
};

struct hid_mouse_device g_hid_mouse;

static struct usb_device_descriptor g_hid_mouse_device_desc =
{
    .bLength            =   USB_DT_DEVICE_SIZE,
    .bDescriptorType    =   USB_DT_DEVICE,
    .bcdUSB             =   0x0200,
    .bDeviceClass       =   USB_HID_CLASS_DEV,
    .bDeviceSubClass    =   USB_HID_SUBCLASS_DEV,
    .idVendor           =   HID_MOUSE_VENDOR_ID,
    .idProduct          =   HID_MOUSE_PRODUCT_ID,
    .bcdDevice          =   HID_DEVICE_BCD,
    .iManufacturer      =   1,
    .iProduct           =   2,
    .iSerialNumber      =   3,
    .bNumConfigurations =   1,
};

static struct usb_interface_descriptor g_hid_mouse_intf_desc_comm =
{
    .bLength            =   USB_DT_INTERFACE_SIZE,
    .bDescriptorType    =   USB_DT_INTERFACE,
    .bInterfaceNumber   =   0x00,
    .bAlternateSetting  =   0x00,
    .bNumEndpoints      =   1,
    .bInterfaceClass    =   USB_HID_CLASS_INF,
    .bInterfaceSubClass =   USB_HID_SUBCLASS_INF_MOUSE,
    .bInterfaceProtocol =   USB_HID_PROTOCOL_INF_MOUSE,
};

static hid_descriptor_t g_hid_mouse_desc =
{
    .raw.bLength                        = HID_MOUSE_HID_DESC_LEN,
    .raw.bDescriptorType                = USB_HID_DESCRIPTOR_HID,
    .raw.bcdHID                         = 0x0110,
    .raw.bCountryCode                   = 0x21,
    .raw.bNumDescriptors                = 0x01,
};


static struct usb_endpoint_descriptor g_hid_mouse_int_desc =
{
    .bLength            =   USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,

    .bEndpointAddress   =   USB_DIR_IN,
    .bmAttributes       =   USB_ENDPOINT_XFER_INT,
    .bInterval          =   0x0A,
    /* wMaxPacketSize set by autoconfiguration */
};

static struct usb_descriptor_header *g_hid_mouse_function[] =
{
    (struct usb_descriptor_header *) &g_hid_mouse_intf_desc_comm,
    (struct usb_descriptor_header *) &g_hid_mouse_desc,
    (struct usb_descriptor_header *) &g_hid_mouse_int_desc,
    NULL,
};

static char g_hid_mouse_vendor_label[]     = "Goodix Technology Co., Ltd.";
static char g_hid_mouse_product_label[]    = "Goodix USB2.0 HID";
static char g_hid_mouse_config_label[]     = "Goodix HID MOUSE";

static struct usb_string g_hid_mouse_strings[] =
{
    [STRING_MANUFACTURER_IDX].s = g_hid_mouse_vendor_label,
    [STRING_PRODUCT_IDX].s      = g_hid_mouse_product_label,
    [STRING_DESCRIPTION_IDX].s  = g_hid_mouse_config_label,
};

static struct usb_gadget_strings g_hid_mouse_device_strings =
{
    .language   = 0x0409,   /* en-us */
    .strings    = g_hid_mouse_strings,
};

static struct usb_string g_hid_mouse_en_us_strings[] =
{
    [HID_STRING_INTERFACE_MOUSE].s = "HID MOUSE Model",
};

static struct usb_gadget_strings g_hid_mouse_function_strings =
{
    .language   = 0x0409,   /* en-us */
    .strings    = g_hid_mouse_en_us_strings,
};


static void hid_mouse_notify_complete(struct usb_ep *ep, struct usb_request *req)
{

}

gr55xx_usb_state_t gr55xx_hid_mouse_send_operation_report(uint8_t *p_op,uint16_t size)
{
    gr55xx_usb_state_t ret;

    if(!p_op)
    {
        return GR_DRV_USB_ERROR_PARAMETER;
    }
    else
    {
        if(HID_IO_POOL_SIZE == size)
        {
            g_hid_mouse.state |= HID_STATUS_MMAPPED;
        }
        else
        {
            return GR_DRV_USB_ERROR_PARAMETER;
        }
    }

    if((g_hid_mouse.state & HID_READY_BITS) != HID_READY_BITS)
    {
        return GR_DRV_USB_ERROR_NOT_READY;
    }

    g_hid_mouse.notify_req->buf = p_op;
    g_hid_mouse.notify_req->length = size;
    g_hid_mouse.notify_req->complete = hid_mouse_notify_complete;

    ret = gr55xx_usb_ep_queue(g_hid_mouse.notify, g_hid_mouse.notify_req);

    return ret;
}


static int hid_mouse_function_bind(struct usb_configuration *c, struct usb_function *f)
{
    struct usb_ep *ep;
    int ret = GR_DRV_USB_ERROR;

    struct usb_composite_dev *cdev = c->cdev;

    /* Allocate endpoints. */
    ep = gr55xx_usb_ep_find(cdev->gadget, "ep2");
    if (!ep)
    {
        USB_LOG("[%s]: Unable to allocate int_in_desc EP", __FUNCTION__);
        goto error;
    }

    USB_LOG(" -----%s------",ep->name);

//    g_hid_mouse_int_desc.bEndpointAddress |= 2;
    g_hid_mouse_int_desc.bEndpointAddress |= 3;
    g_hid_mouse_int_desc.wMaxPacketSize = ep->maxpacket;
    g_hid_mouse.notify = ep;
    ep->driver_data = (void *)&g_hid_mouse;


    if ((ret = gr55xx_usb_interface_id(c, f)) < 0)
    {
        goto error;
    }
    g_hid_mouse_intf_desc_comm.bInterfaceNumber = ret;

    ret = GR_DRV_USB_OK;

    f->descriptors = gr55xx_usb_copy_descriptors(g_hid_mouse_function);
    if (!f->descriptors)
    {
        ret = GR_DRV_USB_ERROR_NOMEM;
        goto error;
    }

    g_hid_mouse.notify_req = gr55xx_usb_alloc_request(g_hid_mouse.notify);
    if (!g_hid_mouse.notify_req)
    {
        ret = GR_DRV_USB_ERROR_NOMEM;
        goto error;
    }
error:
    return ret;
}



static int hid_mouse_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
    struct usb_ep *ep = g_hid_mouse.func.config->cdev->gadget->ep0;
    struct usb_request *req = g_hid_mouse.func.config->cdev->req;

    uint16_t w_index = ctrl->wIndex;
    uint16_t w_value = ctrl->wValue;
    uint16_t w_length = ctrl->wLength;
    int value = GR_DRV_USB_ERROR_NOTSUPP;

    USB_LOG("hid setup request %02x %02x value %04x index %04x %04x \r\n",ctrl->bRequestType, ctrl->bRequest, (ctrl->wValue),(ctrl->wIndex), (ctrl->wLength));

    if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
    {
        switch(ctrl->bRequest)
        {
            case USB_REQ_GET_DESCRIPTOR:
                if(ctrl->wValue == (USB_HID_DESCRIPTOR_REPORT << 8))
                {
                    USB_LOG("USB host required HID report descriptor \r\n");
                    value = sizeof(ReportDescriptor_mouse);
                    memcpy(req->buf, &ReportDescriptor_mouse, value);
                }
                break;
            default:
                USB_ERROR("ERROR: Unsupport hid mouse request:%02x.%02x v%04x i%04x l%d",
                        ctrl->bRequestType, ctrl->bRequest,w_value, w_index, w_length);
                break;
        }
    }

    if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS)
    {
        switch(ctrl->bRequest)
        {
            case USB_HID_REQ_GET_REPORT:
                value = 0;
                break;
            case USB_HID_REQ_GET_IDLE:
                value = 0;
                break;
            case USB_HID_REQ_GET_PROTOCOL:
                value = 0;
                break;
            case USB_HID_REQ_SET_IDLE:
                break;
            default:
                USB_ERROR("ERROR: Unsupport hid mouse request:%02x.%02x v%04x i%04x l%d \r\n",
                        ctrl->bRequestType, ctrl->bRequest,w_value, w_index, w_length);
                break;
        }
    }

    /* respond with data transfer or status phase? */
    if (value >= 0) {
        req->zero = 0;
        req->length = value;
        value = gr55xx_usb_ep_queue(ep, req);
        if (value < 0)
        {
            USB_ERROR("ERROR: hid mouse response error %d!\r\n", value);
        }
    }

    return value;
}

static int hid_mouse_function_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
    gr55xx_usb_enable(g_hid_mouse.notify, &g_hid_mouse_int_desc);

    g_hid_mouse.state |= HID_STATUS_CONNECTED;

    return GR_DRV_USB_OK;
}

static int hid_mouse_function_disable(struct usb_function *f)
{
    return GR_DRV_USB_OK;
}

static int  hid_mouse_config_bind(struct usb_configuration *c)
{
    int ret;

    memset(&g_hid_mouse, 0x00, sizeof(struct hid_mouse_device));

    INIT_LIST_HEAD(&g_hid_mouse.read_pool);
    INIT_LIST_HEAD(&g_hid_mouse.write_pool);

    if ((ret = gr55xx_usb_string_id(c->cdev)) < 0)
    {
        goto error;
    }
    g_hid_mouse_en_us_strings[HID_STRING_INTERFACE_MOUSE].id = ret;
    g_hid_mouse_intf_desc_comm.iInterface = ret;

    g_hid_mouse_desc.raw.reports[0].bDescriptorType = USB_HID_DESCRIPTOR_REPORT;
    g_hid_mouse_desc.raw.reports[0].wDescriptorLength = sizeof(ReportDescriptor_mouse);


    g_hid_mouse.func.name           = "HID MOUSE Function";
    g_hid_mouse.func.strings        = &g_hid_mouse_function_strings;
    g_hid_mouse.func.bind           = hid_mouse_function_bind;
    g_hid_mouse.func.setup          = hid_mouse_function_setup;
    g_hid_mouse.func.disable        = hid_mouse_function_disable;
    g_hid_mouse.func.set_alt        = hid_mouse_function_set_alt;

    ret = gr55xx_usb_add_function(c, &g_hid_mouse.func);

error:
    return ret;
}

static struct usb_configuration g_hid_mouse_config_driver =
{
    .label                      = "HID MOUSE Serial",
    .bind                       = hid_mouse_config_bind,
    .bConfigurationValue        = 1,
    .iConfiguration             = 0, /* dynamic */
    .bmAttributes               = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_WAKEUP,
    .bMaxPower                  = 0x32,
};

static int hid_mouse_bind(struct usb_composite_dev *cdev)
{
    int ret;

    if ((ret = gr55xx_usb_string_id(cdev)) < 0)
    {
        goto error;
    }
    g_hid_mouse_strings[STRING_MANUFACTURER_IDX].id = ret;
    g_hid_mouse_device_desc.iManufacturer = ret;

    if ((ret = gr55xx_usb_string_id(cdev)) < 0)
    {
        goto error;
    }
    g_hid_mouse_strings[STRING_PRODUCT_IDX].id = ret;
    g_hid_mouse_device_desc.iProduct = ret;
    g_hid_mouse_device_desc.iSerialNumber = 3;

    if ((ret = gr55xx_usb_string_id(cdev)) < 0)
    {
        goto error;
    }
    g_hid_mouse_strings[STRING_DESCRIPTION_IDX].id = ret;
    g_hid_mouse_config_driver.iConfiguration = ret;

    if ((ret = gr55xx_usb_add_config(cdev, &g_hid_mouse_config_driver)) < 0)
    {
        goto error;
    }
    USB_LOG("[%s]: Goodix HID MOUSE Gadget", __FUNCTION__);
error:

    return ret;
}


static struct usb_composite_driver g_hid_mouse_driver =
{
    .name       = "cdc driver",
    .dev        = &g_hid_mouse_device_desc,
    .strings    = &g_hid_mouse_device_strings,
    .bind       = hid_mouse_bind,
};

gr55xx_usb_state_t gr55xx_hid_mouse_init(void)
{
    memset(&g_hid_mouse, 0x0, sizeof(g_hid_mouse));
    gr55xx_usb_composite_register(&g_hid_mouse_driver);
    g_hid_mouse.state |= HID_STATUS_ACTIVATED;

    return GR_DRV_USB_OK;
}

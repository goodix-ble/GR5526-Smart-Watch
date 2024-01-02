/*
 * Copyright (C) 2017, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
 */

#include <string.h>
#include "gr55xx_usb_composite.h"
#include "gr55xx_hid_keyboard.h"
#include "gr55xx_usb_server.h"

#define HID_KEYBOARD_VENDOR_ID                   0x27C6
#define HID_KEYBOARD_PRODUCT_ID                  0x2101
#define HID_DEVICE_BCD                           0x0100  /* 1.00 */

#define HID_STRING_INTERFACE_MOUSE      0

#define HID_BUFFER_SIZE                 64
#define HID_KYB_KEY_READ_SIZE           2

#define HID_STATUS_CONNECTED            0x00000001
#define HID_STATUS_ACTIVATED            0x00000002
#define HID_STATUS_MMAPPED              0x00000004

#define HID_MOUSE_HID_DESC_LEN          0x09

#define HID_READY_BITS                  (HID_STATUS_CONNECTED | HID_STATUS_ACTIVATED | HID_STATUS_MMAPPED)

uint8_t g_hid_keyboard_read_buf[HID_KYB_KEY_READ_SIZE];

uint8_t ReportDescriptor_keyboard[] =
{
    0x05 , 0x01,    //USAGE_PAGE(Generic Desktop)
    0x09 , 0x06,    //USAGE(Keyboard)
    0xA1 , 0x01,    //COLLECTION(Application)
    0x05 , 0x07,    //USAGE_PAGE(Keyboard/Keypad)
    0x19 , 0xE0,    //USAGE_MINIMUN(Keyboard LeftControl) Ctrl key
    0x29 , 0xE7,    //USAGE_MAXIMUN(Keyboard Right GUI) WIN key
    0x15 , 0x00,    //LOGICAL_MINIMUN(0)
    0x25 , 0x01,    //LOGICAL_MAXIMUM(1)
    0x95 , 0x08,    //REPORT_COUNT(8)
    0x75 , 0x01,    //REPORT_SIZE(1)
    0x81 , 0x02,    //INPUT(Data,Var.Abs)
    0x95 , 0x01,    //REPORT_COUNT(1)
    0x75 , 0x08,    //REPORT_SIZE(8)
    0x81 , 0x03,    //INPUT(Cnst,Var,Abs)
    0x95 , 0x06,    //REPORT_COUNT(6)
    0x75 , 0x08,    //REPORT_SIZE(8)
    0x15 , 0x00,    //LOGICAL_MINIMUN(0)
    0x25 , 0xFF,    //LOGICAL_MAXIMUN(255)
    0x05 , 0x07,    //USAGE_PAGE(Keyboaed/Keypad)
    0x19 , 0x00,    //USAGE_MINIMUN(Reserved(no event indicated))
    0x29 , 0x65,    //USAGE_MAXIMUN(Keyboard Application)
    0x81 , 0x00,    //INPUT(Data,Ary,Abs)

    0x25 , 0x01,    //LOGICAL_MAXIMUN(1)
    0x95 , 0x05,    //REPORT_COUNT(5)
    0x75 , 0x01,    //REPORT_SIZE(1)
    0x05 , 0x08,    //USAGE_PAGE(LEDs)
    0x19 , 0x01,    //USAGE_MINIMUN(Num Lock)
    0x29 , 0x05,    //UAAGE_MAXIMUN(Kana)
    0x91 , 0x02,    //OUYPUT(Data,Var,Abs)
    0x95 , 0x01,    //REPORT_COUNT(1)
    0x75 , 0x03,    //REPORT_SIZE(3)
    0x91 , 0x03,    //OUTPUT(Cnst,Var,Abs)
    0xC0            //END_COLLECTION
};

struct hid_keyboard_device
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

static struct hid_keyboard_device g_hid_keyboard;

static struct usb_device_descriptor g_hid_keyboard_device_desc =
{
    .bLength            =   USB_DT_DEVICE_SIZE,
    .bDescriptorType    =   USB_DT_DEVICE,
    .bcdUSB             =   0x0200,
    .bDeviceClass       =   USB_HID_CLASS_DEV,
    .bDeviceSubClass    =   USB_HID_SUBCLASS_DEV,
    .idVendor           =   HID_KEYBOARD_VENDOR_ID,
    .idProduct          =   HID_KEYBOARD_PRODUCT_ID,
    .bcdDevice          =   HID_DEVICE_BCD,
    .iManufacturer      =   1,
    .iProduct           =   2,
    .iSerialNumber      =   3,
    .bNumConfigurations =   1,
};

static struct usb_interface_descriptor g_hid_keyboard_intf_desc_comm =
{
    .bLength            =   USB_DT_INTERFACE_SIZE,
    .bDescriptorType    =   USB_DT_INTERFACE,
    .bInterfaceNumber   =   0x00,
    .bAlternateSetting  =   0x00,
    .bNumEndpoints      =   2,
    .bInterfaceClass    =   USB_HID_CLASS_INF,
    .bInterfaceSubClass =   USB_HID_SUBCLASS_INF_KEYBOARD,
    .bInterfaceProtocol =   USB_HID_PROTOCOL_INF_KEYBOARD,
};

static hid_descriptor_t g_hid_keyboard_desc =
{
    .raw.bLength                        = HID_MOUSE_HID_DESC_LEN,
    .raw.bDescriptorType                = USB_HID_DESCRIPTOR_HID,
    .raw.bcdHID                         = 0x0110,
    .raw.bCountryCode                   = 0x21,
    .raw.bNumDescriptors                = 0x01,
};


static struct usb_endpoint_descriptor g_hid_keyboard_in_desc =
{
    .bLength            =   USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,

    .bEndpointAddress   =   USB_DIR_IN,
    .bmAttributes       =   USB_ENDPOINT_XFER_INT,
    .bInterval          =   0x0A,
    /* wMaxPacketSize set by autoconfiguration */
};

static struct usb_endpoint_descriptor g_hid_keyboard_out_desc =
{
    .bLength            =   USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,

    .bEndpointAddress   =   USB_DIR_OUT,
    .bmAttributes       =   USB_ENDPOINT_XFER_INT,
    .bInterval          =   0x0A,
    /* wMaxPacketSize set by autoconfiguration */
};

static struct usb_descriptor_header *g_hid_keyboard_function[] =
{
    (struct usb_descriptor_header *) &g_hid_keyboard_intf_desc_comm,
    (struct usb_descriptor_header *) &g_hid_keyboard_desc,
    (struct usb_descriptor_header *) &g_hid_keyboard_in_desc,
    (struct usb_descriptor_header *) &g_hid_keyboard_out_desc,
    NULL,
};

static char g_hid_keyboard_vendor_label[]     = "Goodix Technology Co., Ltd.";
static char g_hid_keyboard_product_label[]    = "Goodix USB2.0 HID";
static char g_hid_keyboard_config_label[]     = "Goodix HID KEYBOARD";

static struct usb_string g_hid_keyboard_strings[] =
{
    [STRING_MANUFACTURER_IDX].s = g_hid_keyboard_vendor_label,
    [STRING_PRODUCT_IDX].s      = g_hid_keyboard_product_label,
    [STRING_DESCRIPTION_IDX].s  = g_hid_keyboard_config_label,
};

static struct usb_gadget_strings g_hid_keyboard_device_strings =
{
    .language   = 0x0409,   /* en-us */
    .strings    = g_hid_keyboard_strings,
};

static struct usb_string g_hid_keyboard_en_us_strings[] =
{
    [HID_STRING_INTERFACE_MOUSE].s = "HID KEYBOARD Model",
};

static struct usb_gadget_strings g_hid_keyboard_function_strings =
{
    .language   = 0x0409,   /* en-us */
    .strings    = g_hid_keyboard_en_us_strings,
};


static void hid_keyboard_notify_complete(struct usb_ep *ep, struct usb_request *req)
{

}

static void hid_keyboard_read_complete(struct usb_ep *ep, struct usb_request *req)
{

}

gr55xx_usb_state_t gr55xx_hid_send_keyboard_operation_report(uint8_t* p_op,uint16_t size)
{
    gr55xx_usb_state_t ret = GR_DRV_USB_OK;

    if(!p_op)
    {
        return GR_DRV_USB_ERROR_PARAMETER;
    }
    else
    {
        if(HID_KYB_KEY_POOL_SIZE == size)
        {
            g_hid_keyboard.state |= HID_STATUS_MMAPPED;
        }
        else
        {
            return GR_DRV_USB_ERROR_PARAMETER;
        }
    }

    if((g_hid_keyboard.state & HID_READY_BITS) != HID_READY_BITS)
    {
        return GR_DRV_USB_ERROR_NOT_READY;
    }

    g_hid_keyboard.notify_req->buf = p_op;
    g_hid_keyboard.notify_req->length = size;
    g_hid_keyboard.notify_req->complete = hid_keyboard_notify_complete;

    ret = gr55xx_usb_ep_queue(g_hid_keyboard.notify, g_hid_keyboard.notify_req);

    return ret;
}

static int hid_keyboard_function_bind(struct usb_configuration *c, struct usb_function *f)
{
    struct usb_ep *ep;
    int ret = GR_DRV_USB_ERROR;

    struct usb_composite_dev *cdev = c->cdev;

    /* Allocate endpoints. */
    ep = gr55xx_usb_ep_find(cdev->gadget, "ep2");
    if(!ep)
    {
        USB_ERROR("ERROR: [%s]: Unable to allocate int_in_desc EP \r\n", __FUNCTION__);
        goto error;
    }

    USB_LOG(" -----%s------",ep->name);
    g_hid_keyboard_in_desc.bEndpointAddress |= 2;
    g_hid_keyboard_in_desc.wMaxPacketSize = ep->maxpacket;
    g_hid_keyboard.notify = ep;
    ep->driver_data = (void *)&g_hid_keyboard;

    /* Allocate endpoints. */
    ep = gr55xx_usb_ep_find(cdev->gadget, "ep1");
    if (!ep)
    {
       USB_ERROR("ERROR: Unable to allocate bulk_in_desc EP\r\n");
        goto error;
    }

    USB_LOG(" -----%s------\r\n",ep->name);

    g_hid_keyboard_out_desc.bEndpointAddress |= 1;
    g_hid_keyboard_out_desc.wMaxPacketSize = ep->maxpacket;
    g_hid_keyboard.out = ep;
    ep->driver_data = (void *)&g_hid_keyboard;

    if ((ret = gr55xx_usb_interface_id(c, f)) < 0)
    {
        goto error;
    }
    g_hid_keyboard_intf_desc_comm.bInterfaceNumber = ret;

    ret = GR_DRV_USB_OK;

    f->descriptors = gr55xx_usb_copy_descriptors(g_hid_keyboard_function);
    if (!f->descriptors)
    {
        ret = GR_DRV_USB_ERROR_NOMEM;
        goto error;
    }

    g_hid_keyboard.notify_req = gr55xx_usb_alloc_request(g_hid_keyboard.notify);
    if (!g_hid_keyboard.notify_req)
    {
        ret = GR_DRV_USB_ERROR_NOMEM;
        goto error;
    }
error:
    return ret;
}



static int hid_keyboard_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
    struct usb_ep *ep = g_hid_keyboard.func.config->cdev->gadget->ep0;
    struct usb_request *req = g_hid_keyboard.func.config->cdev->req;

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
                    value = sizeof(ReportDescriptor_keyboard);
                    memcpy(req->buf, &ReportDescriptor_keyboard, value);
                }
                break;
            default:
                USB_ERROR("ERROR: Unsupport hid keyboard request:%02x.%02x v%04x i%04x l%d",
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
                USB_ERROR("ERROR: Unsupport hid keyboard request:%02x.%02x v%04x i%04x l%d\r\n",
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
            USB_ERROR("ERROR: hid keyboard response error %d!\r\n", value);
        }
    }

    return value;
}

static int hid_keyboard_function_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
    gr55xx_usb_enable(g_hid_keyboard.notify, &g_hid_keyboard_in_desc);

    g_hid_keyboard.state |= HID_STATUS_CONNECTED;

    return GR_DRV_USB_OK;
}

static int hid_keyboard_function_disable(struct usb_function *f)
{
    return GR_DRV_USB_OK;
}

static int  hid_keyboard_config_bind(struct usb_configuration *c)
{
    int ret;

    memset(&g_hid_keyboard, 0x00, sizeof(struct hid_keyboard_device));

    INIT_LIST_HEAD(&g_hid_keyboard.read_pool);
    INIT_LIST_HEAD(&g_hid_keyboard.write_pool);

    if ((ret = gr55xx_usb_string_id(c->cdev)) < 0)
    {
        goto error;
    }
    g_hid_keyboard_en_us_strings[HID_STRING_INTERFACE_MOUSE].id = ret;
    g_hid_keyboard_intf_desc_comm.iInterface = ret;

    g_hid_keyboard_desc.raw.reports[0].bDescriptorType = USB_HID_DESCRIPTOR_REPORT;
    g_hid_keyboard_desc.raw.reports[0].wDescriptorLength = sizeof(ReportDescriptor_keyboard);

    g_hid_keyboard.func.name           = "HID KEYBOARD Function";
    g_hid_keyboard.func.strings        = &g_hid_keyboard_function_strings;
    g_hid_keyboard.func.bind           = hid_keyboard_function_bind;
    g_hid_keyboard.func.setup          = hid_keyboard_function_setup;
    g_hid_keyboard.func.disable        = hid_keyboard_function_disable;
    g_hid_keyboard.func.set_alt        = hid_keyboard_function_set_alt;

    ret = gr55xx_usb_add_function(c, &g_hid_keyboard.func);

error:
    return ret;
}

static struct usb_configuration g_hid_keyboard_config_driver =
{
    .label                      = "HID KEYBOARD Serial",
    .bind                       = hid_keyboard_config_bind,
    .bConfigurationValue        = 1,
    .iConfiguration             = 0, /* dynamic */
    .bmAttributes               = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_WAKEUP,
    .bMaxPower                  = 0x32,
};

static int hid_keyboard_bind(struct usb_composite_dev *cdev)
{
    int ret;

    if ((ret = gr55xx_usb_string_id(cdev)) < 0)
    {
        goto error;
    }
    g_hid_keyboard_strings[STRING_MANUFACTURER_IDX].id = ret;
    g_hid_keyboard_device_desc.iManufacturer = ret;

    if ((ret = gr55xx_usb_string_id(cdev)) < 0)
    {
        goto error;
    }
    g_hid_keyboard_strings[STRING_PRODUCT_IDX].id = ret;
    g_hid_keyboard_device_desc.iProduct = ret;
    g_hid_keyboard_device_desc.iSerialNumber = 3;

    if ((ret = gr55xx_usb_string_id(cdev)) < 0)
    {
        goto error;
    }
    g_hid_keyboard_strings[STRING_DESCRIPTION_IDX].id = ret;
    g_hid_keyboard_config_driver.iConfiguration = ret;

    if ((ret = gr55xx_usb_add_config(cdev, &g_hid_keyboard_config_driver)) < 0)
    {
        goto error;
    }

    USB_LOG("[%s]: Goodix HID KEYBOARD Gadget", __FUNCTION__);

error:
    return ret;
}
__weak void gr55xx_usb_hid_keyboard_read_data(uint8_t *p_data, uint16_t size)
{

}
static int hid_keyboard_ep_callback(struct usb_gadget *gadget,struct usb_ep *ep)
{
    int retval = GR_DRV_USB_ERROR;
    struct usb_request *req;

    if(!list_empty(&g_hid_keyboard.read_pool) && (ep->address == 1))
    {
        req = list_entry(g_hid_keyboard.read_pool.next, struct usb_request, list);
        ep->driver_data = req->buf;
        memset(req->buf,0x00,HID_KYB_KEY_READ_SIZE);
        retval = gr55xx_drv_usbd_driver_data(ep);
        gr55xx_usb_hid_keyboard_read_data(ep->driver_data,retval);
    }

    return retval;
}

static struct usb_composite_driver g_hid_keyboard_driver =
{
    .name           = "cdc driver",
    .dev            = &g_hid_keyboard_device_desc,
    .strings        = &g_hid_keyboard_device_strings,
    .bind           = hid_keyboard_bind,
    .ep_callback    = hid_keyboard_ep_callback,
};

gr55xx_usb_state_t gr55xx_hid_keyboard_init(void)
{
    struct usb_request *req;

    memset(&g_hid_keyboard, 0x0, sizeof(g_hid_keyboard));
    gr55xx_usb_composite_register(&g_hid_keyboard_driver);

    req = gr55xx_usb_alloc_request(g_hid_keyboard.out);
    if (!req)
    {
        return GR_DRV_USB_ERROR_NOMEM;
    }

    req->buf = g_hid_keyboard_read_buf;
    req->length = HID_KYB_KEY_READ_SIZE;
    req->complete = hid_keyboard_read_complete;

    list_add_tail(&req->list, &g_hid_keyboard.read_pool);

    g_hid_keyboard.state |= HID_STATUS_ACTIVATED;

    return GR_DRV_USB_OK;
}

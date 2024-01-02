/*
 * Copyright (C) 2021, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
 */

#include <string.h>
#include "gr55xx_usb_composite.h"
#include "gr55xx_usb_server.h"

/* big enough to hold our biggest descriptor */
#define USB_COMPOSITE_BUFSZ (USB_DT_INTERFACE_SIZE + MAX_CONFIG_ENDPOINTS*USB_DT_ENDPOINT_AUDIO_SIZE)*MAX_CONFIG_INTERFACES*4

static uint8_t g_composite_buffer[USB_COMPOSITE_BUFSZ];
static uint8_t g_composite_desc_buffer[USB_COMPOSITE_BUFSZ];
static struct usb_composite_dev g_compositedev;
static struct usb_composite_driver *g_composite;

static int usb_descriptor_fillbuf(uint8_t *buf, unsigned buflen,const struct usb_descriptor_header **src)
{
    uint8_t *dest = buf;

    if (!src)
        return GR_DRV_USB_ERROR;

    /* fill buffer from src[] until null descriptor ptr */
    for (; NULL != *src; src++)
    {
        unsigned len = (*src)->bLength;
        if (len > buflen)
            return GR_DRV_USB_ERROR;
        memcpy(dest, *src, len);
        buflen -= len;
        dest += len;
    }

    return dest - buf;
}

static int count_configs(struct usb_composite_dev *cdev, unsigned type)
{
    struct usb_configuration *c;
    unsigned count = 0;

    list_for_each_entry(c, &cdev->configs, list,struct usb_configuration)
    {
        if(c)
        {
            count++;
            break;
        }
    }

    return count;
}

static int config_buf(struct usb_configuration *config, enum usb_device_speed speed, uint8_t *buf, uint8_t type)
{
    struct usb_config_descriptor *c;
    uint8_t *next = buf + USB_DT_CONFIG_SIZE;
    int len = USB_COMPOSITE_BUFSZ - USB_DT_CONFIG_SIZE;
    struct usb_function *f;
    int status;

    /* write the config descriptor */
    c = (struct usb_config_descriptor *)buf;
    c->bLength = USB_DT_CONFIG_SIZE;
    c->bDescriptorType = type;
    /* wTotalLength is written later */
    c->bNumInterfaces = config->next_interface_id;
    c->bConfigurationValue = config->bConfigurationValue;
    c->iConfiguration = config->iConfiguration;
    c->bmAttributes = USB_CONFIG_ATT_ONE | config->bmAttributes;
    c->bMaxPower = config->bMaxPower;

    list_for_each_entry(f, &config->functions, list,struct usb_function)
    {
        struct usb_descriptor_header **descriptors;
        descriptors = f->descriptors;

        if (!descriptors)
            continue;
        status = usb_descriptor_fillbuf(next, len,(const struct usb_descriptor_header **) descriptors);
        if (status < 0)
            return status;
        len -= status;
        next += status;
    }

    len = next - buf;
    c->wTotalLength = len;

    return len;
}

static int ansitouni(const char *s,uint8_t *buf, unsigned len)
{
    unsigned int i;

    for(i=0;i<len;i++)
    {
        buf[i*2+0]=s[i];
        buf[i*2+1]=0x00;
    }

    return len;
}

static int usb_gadget_get_string (struct usb_gadget_strings* table, int id, uint8_t *buf)
{
    struct usb_string *temp_s;
    int len;

    for (temp_s = table->strings; temp_s && temp_s->s; temp_s++)
    {
        if (temp_s->id == id)
            break;
    }
    if (!temp_s || !temp_s->s)
    {
        return GR_DRV_USB_ERROR;
    }
    len = min((size_t)126, strlen(temp_s->s));
    memset (buf + 2, 0, 2 * len);   /* zero all the bytes */
    len = ansitouni(temp_s->s, &buf[2], len);
    if (len < 0)
    {
        return GR_DRV_USB_ERROR;
    }
    buf[0] = (len + 1) * 2;
    buf[1] = USB_DT_STRING;

    return buf [0];
}

static int config_desc(struct usb_composite_dev *cdev, unsigned w_value)
{
    struct usb_configuration *c;
    uint8_t type = w_value >> 8;
    enum usb_device_speed speed = USB_SPEED_UNKNOWN;

    speed = USB_SPEED_HIGH;
    /* This is a lookup by config *INDEX* */
    w_value &= 0xff;

    list_for_each_entry(c, &cdev->configs, list,struct usb_configuration)
    {
        if (w_value == 0)
        {
            return config_buf(c, speed, cdev->req->buf, type);
        }
        w_value--;
    }

    return GR_DRV_USB_ERROR;
}

static int lookup_string(struct usb_gadget_strings *sp, void *buf,uint16_t language,int id)
{
    struct usb_gadget_strings *s;
    int value;

    if (sp)
    {
        s = sp++;
        if (s->language != language)
        {
            return GR_DRV_USB_ERROR;
        }
        value = usb_gadget_get_string(s, id, buf);
        if (value > 0)
        {
            return value;
        }
    }

    return GR_DRV_USB_ERROR;
}

static int get_string(struct usb_composite_dev *cdev,uint8_t *buf, uint16_t language, int id)
{
    struct usb_configuration *c;
    struct usb_function *f;
    int len;

    if (id == 0 && g_composite->strings)
    {
        buf [0] = 4;
        buf [1] = USB_DT_STRING;
        buf [2] = (uint8_t)g_composite->strings->language;
        buf [3] = (uint8_t)(g_composite->strings->language >> 8);
        return 4;
    }

    if (g_composite->strings)
    {
        len = lookup_string(g_composite->strings, buf, language, id);
        if (len > 0)
        {
            return len;
        }
    }
    list_for_each_entry(c, &cdev->configs, list,struct usb_configuration)
    {
        if (c->strings)
        {
            len = lookup_string(c->strings, buf, language, id);
            if (len > 0)
            {
                return len;
            }
        }
        list_for_each_entry(f, &c->functions, list,struct usb_function)
        {
            if (!f->strings)
            {
                continue;
            }
            len = lookup_string(f->strings, buf, language, id);
            if (len > 0)
            {
                return len;
            }
        }
    }

    return 0;
}

static void usb_ep_autoconfig_reset (struct usb_gadget *gadget)
{
    struct usb_ep *ep;

    list_for_each_entry(ep, &gadget->ep_list, ep_list,struct usb_ep)
    {
        ep->driver_data = NULL;
    }
}

int gr55xx_usb_add_config(struct usb_composite_dev *cdev, struct usb_configuration *config)
{
    int status = GR_DRV_USB_ERROR;
    struct usb_configuration *c;

    if (!config->bConfigurationValue || !config->bind)
    {
        goto done;
    }
    /* Prevent duplicate configuration identifiers */
    list_for_each_entry(c, &cdev->configs, list,struct usb_configuration)
    {
        if (c->bConfigurationValue == config->bConfigurationValue)
        {
            status = GR_DRV_USB_ERROR_BUSY;
            goto done;
        }
    }
    config->cdev = cdev;
    list_add_tail(&config->list, &cdev->configs);

    INIT_LIST_HEAD(&config->functions);
    config->next_interface_id = 0;

    status = config->bind(config); //ref to msc_config_bind or uvc_config_bind

    if (status < 0)
    {
        list_del(&config->list);
        config->cdev = NULL;
    }
    else
    {
        unsigned i;
        for (i = 0; i < MAX_CONFIG_INTERFACES; i++)
        {
            struct usb_function  *f = config->interface[i];

            if(!f)
            {
                continue;
            }
            USB_LOG("interface %d = %s \r\n", i, f->name);
        }
    }

    usb_ep_autoconfig_reset(cdev->gadget);

done:
    return status;
}

int gr55xx_usb_add_function(struct usb_configuration *config, struct usb_function *function)
{
    int value = GR_DRV_USB_ERROR;

    if (!function->set_alt || !function->disable)
    {
        goto done;
    }
    function->config = config;
    list_add_tail(&function->list, &config->functions);

    /* REVISIT *require* function->bind? */
    if (function->bind)
    {
        value = function->bind(config, function); //ref to XXX_function_bind
        if (value < 0)
        {
            list_del(&function->list);
            function->config = NULL;
        }
    }
    else
    {
        value = 0;
    }
done:
    return value;
}

int gr55xx_usb_string_id(struct usb_composite_dev *cdev)
{
    if (cdev->next_string_id < 254)
    {
        /* string id 0 is reserved */
        cdev->next_string_id++;
        return cdev->next_string_id;
    }

    return GR_DRV_USB_ERROR_NODEV;
}

int gr55xx_usb_interface_id(struct usb_configuration *config, struct usb_function *function)
{
    unsigned id = config->next_interface_id;

    if (id < MAX_CONFIG_INTERFACES)
    {
        config->interface[id] = function;
        config->next_interface_id = id + 1;
        return id;
    }
    return GR_DRV_USB_ERROR_NODEV;
}

struct usb_ep *gr55xx_usb_ep_find(struct usb_gadget *gadget, const char *name)
{
    struct usb_ep *ep;

    list_for_each_entry (ep, &gadget->ep_list, ep_list,struct usb_ep)
    {
        if (0 == strncmp(ep->name, name, strlen(name)))
        {
            return ep;
        }
    }

    return NULL;
}

struct usb_descriptor_header **gr55xx_usb_copy_descriptors(struct usb_descriptor_header **src)
{
    struct usb_descriptor_header **ret;
    struct usb_descriptor_header **tmp;
    unsigned bytes;
    unsigned n_desc;
    uint8_t *mem = &g_composite_desc_buffer[0];

    /* count descriptors and their sizes; then add vector size */
    for (bytes = 0, n_desc = 0, tmp = src; *tmp; tmp++, n_desc++)
    {
        bytes += (*tmp)->bLength;
    }
    bytes += (n_desc + 1) * sizeof(*tmp);


    /* fill in pointers starting at "tmp",
        * to descriptors copied starting at "mem";
        * and return "ret"
        */
    tmp = (struct usb_descriptor_header **)mem;
    ret = (struct usb_descriptor_header **)mem;
    mem += (n_desc + 1) * sizeof(*tmp);
    while (*src)
    {
        memcpy(mem, *src, (*src)->bLength);
        *tmp = (struct usb_descriptor_header *)mem;
        tmp++;
        mem += (*src)->bLength;
        src++;
    }
    *tmp = NULL;
    return ret;
}

struct usb_request *gr55xx_usb_alloc_request(struct usb_ep *ep)
{
    struct usb_request *req = NULL;

    if (g_composite)
    {
        req = gr55xx_drv_usbd_alloc_request(ep);
    }

    return req;
}



void gr55xx_usb_free_request(struct usb_ep *ep, struct usb_request *req)
{
    if (g_composite)
    {
        gr55xx_drv_usbd_free_request(ep, req);
    }
}

gr55xx_usb_state_t gr55xx_usb_enable(struct usb_ep *ep, const struct usb_endpoint_descriptor *desc)
{
    gr55xx_usb_state_t err = GR_DRV_USB_ERROR;

    if (g_composite)
    {
        err = gr55xx_drv_usbd_enable(ep, desc);
    }

    return err;
}

gr55xx_usb_state_t gr55xx_usb_disable(struct usb_ep *ep)
{
    gr55xx_usb_state_t err = GR_DRV_USB_ERROR;

    if (g_composite)
    {
        err = gr55xx_drv_usbd_disable(ep);
    }

    return err;
}

gr55xx_usb_state_t gr55xx_usb_ep_queue(struct usb_ep *ep, struct usb_request *req)
{
    gr55xx_usb_state_t err = GR_DRV_USB_ERROR;

    if (g_composite)
    {
        err = gr55xx_drv_usbd_queue(ep, req);
    }

    return err;
}

gr55xx_usb_state_t gr55xx_usb_set_halt(struct usb_ep *ep, int value)
{
    gr55xx_usb_state_t err = GR_DRV_USB_ERROR;

    if (g_composite)
    {
        err = gr55xx_drv_usbd_set_halt(ep, value);
    }

    return err;
}

static int set_config(struct usb_composite_dev *cdev,const struct usb_ctrlrequest *ctrl, unsigned number)
{
    struct usb_configuration *c = NULL;
    int result = GR_DRV_USB_ERROR;
    //unsigned power = 100;
    int tmp;

    if (cdev->config)
    {
        cdev->config = NULL;
    }

    if (number)
    {
        list_for_each_entry(c, &cdev->configs, list,struct usb_configuration)
        {
            if (c->bConfigurationValue == number)
            {
                result = 0;
                break;
            }
        }
        if (result < 0)
        {
            goto done;
        }
    }
    else
    {
        result = 0;
    }


    if (!c)
    {
        goto done;
    }

    cdev->config = c;

    /* Initialize all interfaces by setting them to altsetting zero. */
    for (tmp = 0; tmp < MAX_CONFIG_INTERFACES; tmp++)
    {
        struct usb_function  *f = c->interface[tmp];
        struct usb_descriptor_header **descriptors;

        if (!f)
        {
            break;
        }
        descriptors = f->descriptors;

        for (; *descriptors; ++descriptors)
        {
           // struct usb_endpoint_descriptor *ep;
           // int addr;

            if ((*descriptors)->bDescriptorType != USB_DT_ENDPOINT)
            {
                continue;
            }
           // ep = (struct usb_endpoint_descriptor *)*descriptors;
            //addr = ((ep->bEndpointAddress & 0x80) >> 3)|  (ep->bEndpointAddress & 0x0f);
            //set_bit(addr, f->endpoints);
        }

        result = f->set_alt(f, tmp, 0);
        if (result < 0)
        {
            cdev->config = NULL;
            goto done;
        }
    }

    /* when we return, be sure our power usage is valid */
    //power = c->bMaxPower ? (2 * c->bMaxPower) : CONFIG_USB_GADGET_VBUS_DRAW;
done:
    return result;
}

static void composite_setup_complete(struct usb_ep *ep, struct usb_request *req)
{
    USB_LOG("composite_setup_complete req->buf[0]=%d \r\n", req->buf[0]);
    if (req->status || req->actual != req->length)
    {
        USB_LOG("setup complete --> %d, %d/%d", req->status, req->actual, req->length);
    }
}

static int composite_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
    struct usb_function *f = NULL;
    struct usb_composite_dev *cdev = gadget->cdev;
    uint16_t w_index = ctrl->wIndex;
    uint16_t w_value = ctrl->wValue;
    uint16_t w_length = ctrl->wLength;
    uint8_t intf = w_index & 0xFF;
    usb_request_t *req = cdev->req;
    int value = GR_DRV_USB_ERROR_NOTSUPP;

    req->zero = 0;
    req->complete = composite_setup_complete;
    req->length = 0;
    //gadget->ep0->driver_data = cdev;

    switch (ctrl->bRequest)
    {
        case USB_REQ_GET_DESCRIPTOR:
            if (ctrl->bRequestType != USB_DIR_IN)
                goto _unknown;

            switch (w_value >> 8)
            {
                case USB_DT_DEVICE:
                   cdev->desc.bNumConfigurations = count_configs(cdev, USB_DT_DEVICE);
                   value = min(sizeof(cdev->desc), w_length);
                   memcpy(req->buf, &cdev->desc, value);
                   break;
                case USB_DT_CONFIG:
                   value = config_desc(cdev, ctrl->wValue);
                   if (value >= 0)
                       value = min((uint16_t)ctrl->wLength, (uint16_t)value);
                   req->length = value;
                   break;

                case USB_DT_STRING:
                   value = get_string(cdev, req->buf,ctrl->wIndex, ctrl->wValue & 0xff);
                   if (value >= 0)
                       value = min((uint16_t)ctrl->wLength, (uint16_t)value);
                   req->length = value;
                   break;
            }
            break;
        case USB_REQ_SET_CONFIGURATION:
            if (ctrl->bRequestType != 0)
                goto _unknown;
            value = set_config(cdev, ctrl, w_value);
            break;
        case USB_REQ_GET_CONFIGURATION:
            if (ctrl->bRequestType != USB_DIR_IN)
                goto _unknown;
            if (cdev->config)
                *(uint8_t *)req->buf = cdev->config->bConfigurationValue;
            else
                *(uint8_t *)req->buf = 0;
            value = min(w_length, (uint16_t) 1);
            break;
        case USB_REQ_SET_INTERFACE:
            if (ctrl->bRequestType != USB_RECIP_INTERFACE)
                goto _unknown;
            if (!cdev->config || intf >= MAX_CONFIG_INTERFACES)
                break;
            f = cdev->config->interface[intf];
            if (!f)
                break;
            if (w_value && !f->set_alt)
                break;
            value = f->set_alt(f, w_index, w_value);
            break;
        case USB_REQ_GET_INTERFACE:
            if (ctrl->bRequestType != (USB_DIR_IN|USB_RECIP_INTERFACE))
                goto _unknown;
            if (!cdev->config || intf >= MAX_CONFIG_INTERFACES)
                break;
            f = cdev->config->interface[intf];
            if (!f)
                break;
            /* lots of interfaces only need altsetting zero... */
            value = f->get_alt ? f->get_alt(f, w_index) : 0;
            if (value < 0)
                break;
            *((uint8_t *)req->buf) = value;
            value = min(w_length, (uint16_t) 1);
            break;
        default:
_unknown:
            switch (ctrl->bRequestType & USB_RECIP_MASK)
            {
                case USB_RECIP_INTERFACE:
                    if (!cdev->config || intf >= MAX_CONFIG_INTERFACES)
                        break;
                    f = cdev->config->interface[intf];
                    break;
                case USB_RECIP_ENDPOINT:
                    //endp = ((w_index & 0x80) >> 3) | (w_index & 0x0f);
                    list_for_each_entry(f, &cdev->config->functions, list,struct usb_function)
                    {
                        break;
                    }
                    if (&f->list == &cdev->config->functions)
                    {
                        f = NULL;
                    }
                    break;
            }
            if (f && f->setup)
            {
                value = f->setup(f, ctrl);  //ref to xxx_function_setup
            }

            goto done;
    }

    /* respond with data transfer before status phase? */
    if (value >= 0 && value != GR_DRV_USB_ERROR_DELAY) {
        req->length = value;
        req->zero = value < w_length;
        value = gr55xx_usb_ep_queue(gadget->ep0, req);
        if (value < 0) {
            USB_LOG("ep_queue --> %d\n", value);
            req->status = 0;
            composite_setup_complete(gadget->ep0, req);
        }
    } else if (value == GR_DRV_USB_ERROR_DELAY && w_length != 0) {
        USB_WARRING("%s: Delayed status not supported for w_length != 0", __FUNCTION__);
    }

done:
    return value;
}

static int composite_bind(struct usb_gadget *gadget)
{
    int retval = GR_DRV_USB_ERROR;

    memset(&g_compositedev, 0x00, sizeof(g_compositedev));
    INIT_LIST_HEAD(&g_compositedev.configs);

    g_compositedev.gadget = gadget;
    gadget->cdev = &g_compositedev;

    g_compositedev.req = gr55xx_usb_alloc_request(gadget->ep0);
    if (!g_compositedev.req)
    {
        goto fail;
    }

    g_compositedev.req->buf = g_composite_buffer;
    g_compositedev.req->complete = composite_setup_complete;
    g_compositedev.bufsiz = USB_COMPOSITE_BUFSZ;
    g_compositedev.driver = g_composite;

    retval = g_composite->bind(&g_compositedev);

    g_compositedev.desc = *g_composite->dev;
    g_compositedev.desc.bMaxPacketSize0 = gadget->ep0->maxpacket;

fail:
    return retval;
}


static int composite_ep_callback(struct usb_gadget *gadget,struct usb_ep *ep)
{
    int retval = GR_DRV_USB_ERROR;

    if (g_composite )
    {
        g_composite->ep_callback(gadget,ep);
        retval = GR_DRV_USB_OK; 
    }

    return retval;
}

static struct usb_gadget_driver g_composite_driver =
{
    .bind           = composite_bind,
    .setup          = composite_setup,
    .ep_callback    = composite_ep_callback,
};

int gr55xx_usb_composite_register(struct usb_composite_driver *driver)
{
    g_composite = driver;

    return gr55xx_drv_usbd_register_gadget_driver(&g_composite_driver);
}


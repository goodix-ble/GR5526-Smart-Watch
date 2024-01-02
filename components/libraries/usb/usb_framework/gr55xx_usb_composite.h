/*
 * Copyright (C) 2017, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
 */

#ifndef __GR55XX_USB_COMPOSITE_H__
#define __GR55XX_USB_COMPOSITE_H__

#include "list.h"
#include "ch9.h"

#define MAX_CONFIG_INTERFACES       4
#define MAX_CONFIG_ENDPOINTS        4


typedef enum
{
    GR_DRV_USB_OK               = 0x00,
    GR_DRV_USB_ERROR            = -0x01,
    GR_DRV_USB_ERROR_NODEV      = -0x02,
    GR_DRV_USB_ERROR_BUSY       = -0x03,
    GR_DRV_USB_ERROR_NOTSUPP    = -0x04,
    GR_DRV_USB_ERROR_DELAY      = -0x05,
    GR_DRV_USB_ERROR_INPROGRESS = -0x06,
    GR_DRV_USB_ERROR_OVERFLOW   = -0x07,
    GR_DRV_USB_ERROR_HALT       = -0x08,
    GR_DRV_USB_ERROR_NOMEM      = -0x09,
    GR_DRV_USB_ERROR_NOT_READY  = -0x0A,
    GR_DRV_USB_ERROR_PARAMETER  = -0x0B,
    GR_DRV_USB_ERROR_RETRY      = -0xC,
}gr55xx_usb_state_t;


struct usb_string
{
     unsigned char      id;
     const char         *s;
};

struct usb_gadget_strings
{
    unsigned short      language;
    struct usb_string   *strings;
};

struct usb_ep
{
    const char          *name;
    struct list_head    ep_list;
    uint16_t            address;
    uint16_t            maxpacket;
    uint32_t            state;
    const struct usb_endpoint_descriptor    *desc;
    void                *driver_data;
};

typedef struct usb_request
{
    uint8_t             *buf;
    uint32_t            length;
    uint32_t            actual;

    struct list_head    list;

    int                 zero;
    int                 status;
    void (*complete)(struct usb_ep *ep, struct usb_request *req);
} usb_request_t;

struct usb_function
{
    const char  *name;
    struct usb_gadget_strings *strings;
    struct usb_configuration *config;

    int     (*bind)(struct usb_configuration *,struct usb_function *);
    void    (*unbind)(struct usb_configuration *,struct usb_function *);
    int     (*get_alt)(struct usb_function *,unsigned interface);
    int     (*set_alt)(struct usb_function *,unsigned interface, unsigned alt);
    int     (*setup)(struct usb_function *, const struct usb_ctrlrequest *);
    int     (*disable)(struct usb_function *);

    struct list_head list;

    struct usb_descriptor_header **descriptors;
    //struct usb_descriptor_header **hs_descriptors;
};

struct usb_configuration
{
    const char  *label;

    struct list_head list;
    struct list_head functions;

    int (*bind)(struct usb_configuration *);

    /* fields in the config descriptor */
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;

    uint8_t next_interface_id;

    struct usb_composite_dev *cdev;
    struct usb_function *interface[MAX_CONFIG_INTERFACES];
    struct usb_gadget_strings *strings;
};

struct usb_composite_dev
{
    uint32_t                        bufsiz;
    struct usb_gadget               *gadget;
    usb_request_t                   *req;
    struct usb_composite_driver     *driver;

    struct usb_device_descriptor    desc;

    struct usb_configuration        *config;

    struct list_head                configs;
    unsigned char                   next_string_id;
};

struct usb_composite_driver
{
    const char                          *name;
    const struct usb_device_descriptor  *dev;
    struct usb_gadget_strings           *strings;

    int (*bind)(struct usb_composite_dev *);
    int (*ep_callback)(struct usb_gadget *,struct usb_ep *);  //data ep callback
};

struct usb_gadget
{
    const char *name;
    struct usb_ep   *ep0;
    struct list_head    ep_list;
    struct usb_composite_dev *cdev;
};

struct usb_gadget_driver
{
    int     (*bind)(struct usb_gadget *);
    int     (*setup)(struct usb_gadget *,const struct usb_ctrlrequest *);
    int     (*ep_callback)(struct usb_gadget *,struct usb_ep *);  //data ep callback
};

#ifdef  __cplusplus
extern "C"
{
#endif


int gr55xx_usb_add_config(struct usb_composite_dev *cdev, struct usb_configuration *config);
int gr55xx_usb_string_id(struct usb_composite_dev *cdev);
int gr55xx_usb_interface_id(struct usb_configuration *config,  struct usb_function *function);
struct usb_ep *gr55xx_usb_ep_find(struct usb_gadget *gadget, const char *name);
struct usb_descriptor_header **gr55xx_usb_copy_descriptors(struct usb_descriptor_header **src);


struct usb_request *gr55xx_usb_alloc_request(struct usb_ep *ep);
void gr55xx_usb_free_request(struct usb_ep *ep, struct usb_request *req);
gr55xx_usb_state_t gr55xx_usb_enable(struct usb_ep *ep, const struct usb_endpoint_descriptor *desc);
gr55xx_usb_state_t gr55xx_usb_disable(struct usb_ep *ep);
gr55xx_usb_state_t gr55xx_usb_ep_queue(struct usb_ep *ep, struct usb_request *req);
gr55xx_usb_state_t gr55xx_usb_set_halt(struct usb_ep *ep, int value);
int gr55xx_usb_add_function(struct usb_configuration *config, struct usb_function *function);
int gr55xx_usb_composite_register(struct usb_composite_driver *driver);


#ifdef  __cplusplus
}
#endif



#endif /*__GR55XX_USBD_COMPOSITE_H__*/


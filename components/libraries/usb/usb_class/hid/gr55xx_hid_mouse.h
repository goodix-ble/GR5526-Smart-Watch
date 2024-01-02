/*
 * Copyright (C) 2017, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
 */
#ifndef __GR55XX_HID_MOUSE_H__
#define __GR55XX_HID_MOUSE_H__

#include "stdint.h"
#include "gr55xx_usb_composite.h"

#define HID_IO_POOL_SIZE                      4

#define USB_HID_CLASS_INF                     0x03
#define USB_HID_SUBCLASS_INF_MOUSE            0x01
#define USB_HID_PROTOCOL_INF_MOUSE            0x02
#define USB_HID_CLASS_DEV                     0x00
#define USB_HID_SUBCLASS_DEV                  0x00


#define USB_HID_REQ_GET_REPORT                  0x01    /**< REPORT: device -> host (required).                     */
#define USB_HID_REQ_GET_IDLE                    0x02    /**< IDLE: device -> host (not required).                   */
#define USB_HID_REQ_GET_PROTOCOL                0x03    /**< PROTOCOL: device -> host (required for boot protocol). */
#define USB_HID_REQ_SET_REPORT                  0x09    /**< REPORT: host -> device (not required).                 */
#define USB_HID_REQ_SET_IDLE                    0x0A    /**< IDLE: no data stage (required for boot protocol).      */
#define USB_HID_REQ_SET_PROTOCOL                0x0B    /**< PROTOCOL: no data stage(required for boot protocol).   */


#define USB_HID_REPORT_TYPE_INPUT               0x01    /**< INPUT report type   */
#define USB_HID_REPORT_TYPE_OUTPUT              0x02    /**< OUTPUT report type  */
#define USB_HID_REPORT_TYPE_FEATURE             0x03    /**< FEATURE report type */

#define USB_HID_DESCRIPTOR_HID                  0x21    /**< HID descriptor.      */
#define USB_HID_DESCRIPTOR_REPORT               0x22    /**< REPORT descriptor.   */
#define USB_HID_DESCRIPTOR_PHYSICAL             0x23    /**< PHYSICAL descriptor. */


#pragma pack(push, 1)
typedef union{
    struct  {
        uint8_t  bLength;            //!< Length of descriptor.
        uint8_t  bDescriptorType;    //!< Descriptor type @ref APP_USBD_HID_DESCRIPTOR_HID.
        uint16_t bcdHID;             //!< HID release number (BCD format, little endian).
        uint8_t  bCountryCode;       //!< Country code.
        uint8_t  bNumDescriptors;    //!< Number of class descriptors.
        struct {
            uint8_t  bDescriptorType;   //!< Class descriptor type.
            uint16_t wDescriptorLength;  //!< Class descriptor length (little endian).
        } reports[];
    } raw;
}hid_descriptor_t;
#pragma pack(pop)


/* Line Coding Structure from CDC spec 6.2.13 */



#ifdef __cplusplus
extern "C"
{
#endif

//user api

//platfrom api
gr55xx_usb_state_t gr55xx_hid_mouse_init(void);
gr55xx_usb_state_t gr55xx_hid_mouse_send_operation_report(uint8_t *p_op,uint16_t size);

#ifdef __cplusplus
}
#endif


#endif   /*__CDC_H__*/


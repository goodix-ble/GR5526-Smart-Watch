/*
 * Copyright (C) 2017, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
 */
#ifndef __GR55XX_HID_MOUSE_H__
#define __GR55XX_HID_MOUSE_H__

#include "stdint.h"
#include "gr55xx_usb_composite.h"

#define HID_KYB_KEY_POOL_SIZE                 8

#define USB_HID_CLASS_INF                     0x03
#define USB_HID_SUBCLASS_INF_KEYBOARD         0x01
#define USB_HID_PROTOCOL_INF_KEYBOARD         0x01
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

/**
 * @brief HID keyboard codes.
 */
typedef enum {
    USB_HID_KBD_A               = 4,  /**<KBD_A               code*/
    USB_HID_KBD_B               = 5,  /**<KBD_B               code*/
    USB_HID_KBD_C               = 6,  /**<KBD_C               code*/
    USB_HID_KBD_D               = 7,  /**<KBD_D               code*/
    USB_HID_KBD_E               = 8,  /**<KBD_E               code*/
    USB_HID_KBD_F               = 9,  /**<KBD_F               code*/
    USB_HID_KBD_G               = 10, /**<KBD_G               code*/
    USB_HID_KBD_H               = 11, /**<KBD_H               code*/
    USB_HID_KBD_I               = 12, /**<KBD_I               code*/
    USB_HID_KBD_J               = 13, /**<KBD_J               code*/
    USB_HID_KBD_K               = 14, /**<KBD_K               code*/
    USB_HID_KBD_L               = 15, /**<KBD_L               code*/
    USB_HID_KBD_M               = 16, /**<KBD_M               code*/
    USB_HID_KBD_N               = 17, /**<KBD_N               code*/
    USB_HID_KBD_O               = 18, /**<KBD_O               code*/
    USB_HID_KBD_P               = 19, /**<KBD_P               code*/
    USB_HID_KBD_Q               = 20, /**<KBD_Q               code*/
    USB_HID_KBD_R               = 21, /**<KBD_R               code*/
    USB_HID_KBD_S               = 22, /**<KBD_S               code*/
    USB_HID_KBD_T               = 23, /**<KBD_T               code*/
    USB_HID_KBD_U               = 24, /**<KBD_U               code*/
    USB_HID_KBD_V               = 25, /**<KBD_V               code*/
    USB_HID_KBD_W               = 26, /**<KBD_W               code*/
    USB_HID_KBD_X               = 27, /**<KBD_X               code*/
    USB_HID_KBD_Y               = 28, /**<KBD_Y               code*/
    USB_HID_KBD_Z               = 29, /**<KBD_Z               code*/
    USB_HID_KBD_1               = 30, /**<KBD_1               code*/
    USB_HID_KBD_2               = 31, /**<KBD_2               code*/
    USB_HID_KBD_3               = 32, /**<KBD_3               code*/
    USB_HID_KBD_4               = 33, /**<KBD_4               code*/
    USB_HID_KBD_5               = 34, /**<KBD_5               code*/
    USB_HID_KBD_6               = 35, /**<KBD_6               code*/
    USB_HID_KBD_7               = 36, /**<KBD_7               code*/
    USB_HID_KBD_8               = 37, /**<KBD_8               code*/
    USB_HID_KBD_9               = 38, /**<KBD_9               code*/
    USB_HID_KBD_0               = 39, /**<KBD_0               code*/
    USB_HID_KBD_ENTER           = 40, /**<KBD_ENTER           code*/
    USB_HID_KBD_ESCAPE          = 41, /**<KBD_ESCAPE          code*/
    USB_HID_KBD_BACKSPACE       = 42, /**<KBD_BACKSPACE       code*/
    USB_HID_KBD_TAB             = 43, /**<KBD_TAB             code*/
    USB_HID_KBD_SPACEBAR        = 44, /**<KBD_SPACEBAR        code*/
    USB_HID_KBD_UNDERSCORE      = 45, /**<KBD_UNDERSCORE      code*/
    USB_HID_KBD_PLUS            = 46, /**<KBD_PLUS            code*/
    USB_HID_KBD_OPEN_BRACKET    = 47, /**<KBD_OPEN_BRACKET    code*/
    USB_HID_KBD_CLOSE_BRACKET   = 48, /**<KBD_CLOSE_BRACKET   code*/
    USB_HID_KBD_BACKSLASH       = 49, /**<KBD_BACKSLASH       code*/
    USB_HID_KBD_ASH             = 50, /**<KBD_ASH             code*/
    USB_HID_KBD_COLON           = 51, /**<KBD_COLON           code*/
    USB_HID_KBD_QUOTE           = 52, /**<KBD_QUOTE           code*/
    USB_HID_KBD_TILDE           = 53, /**<KBD_TILDE           code*/
    USB_HID_KBD_COMMA           = 54, /**<KBD_COMMA           code*/
    USB_HID_KBD_DOT             = 55, /**<KBD_DOT             code*/
    USB_HID_KBD_SLASH           = 56, /**<KBD_SLASH           code*/
    USB_HID_KBD_CAPS_LOCK       = 57, /**<KBD_CAPS_LOCK       code*/
    USB_HID_KBD_F1              = 58, /**<KBD_F1              code*/
    USB_HID_KBD_F2              = 59, /**<KBD_F2              code*/
    USB_HID_KBD_F3              = 60, /**<KBD_F3              code*/
    USB_HID_KBD_F4              = 61, /**<KBD_F4              code*/
    USB_HID_KBD_F5              = 62, /**<KBD_F5              code*/
    USB_HID_KBD_F6              = 63, /**<KBD_F6              code*/
    USB_HID_KBD_F7              = 64, /**<KBD_F7              code*/
    USB_HID_KBD_F8              = 65, /**<KBD_F8              code*/
    USB_HID_KBD_F9              = 66, /**<KBD_F9              code*/
    USB_HID_KBD_F10             = 67, /**<KBD_F10             code*/
    USB_HID_KBD_F11             = 68, /**<KBD_F11             code*/
    USB_HID_KBD_F12             = 69, /**<KBD_F12             code*/
    USB_HID_KBD_PRINTSCREEN     = 70, /**<KBD_PRINTSCREEN     code*/
    USB_HID_KBD_SCROLL_LOCK     = 71, /**<KBD_SCROLL_LOCK     code*/
    USB_HID_KBD_PAUSE           = 72, /**<KBD_PAUSE           code*/
    USB_HID_KBD_INSERT          = 73, /**<KBD_INSERT          code*/
    USB_HID_KBD_HOME            = 74, /**<KBD_HOME            code*/
    USB_HID_KBD_PAGEUP          = 75, /**<KBD_PAGEUP          code*/
    USB_HID_KBD_DELETE          = 76, /**<KBD_DELETE          code*/
    USB_HID_KBD_END             = 77, /**<KBD_END             code*/
    USB_HID_KBD_PAGEDOWN        = 78, /**<KBD_PAGEDOWN        code*/
    USB_HID_KBD_RIGHT           = 79, /**<KBD_RIGHT           code*/
    USB_HID_KBD_LEFT            = 80, /**<KBD_LEFT            code*/
    USB_HID_KBD_DOWN            = 81, /**<KBD_DOWN            code*/
    USB_HID_KBD_UP              = 82, /**<KBD_UP              code*/
    USB_HID_KBD_KEYPAD_NUM_LOCK = 83, /**<KBD_KEYPAD_NUM_LOCK code*/
    USB_HID_KBD_KEYPAD_DIVIDE   = 84, /**<KBD_KEYPAD_DIVIDE   code*/
    USB_HID_KBD_KEYPAD_AT       = 85, /**<KBD_KEYPAD_AT       code*/
    USB_HID_KBD_KEYPAD_MULTIPLY = 85, /**<KBD_KEYPAD_MULTIPLY code*/
    USB_HID_KBD_KEYPAD_MINUS    = 86, /**<KBD_KEYPAD_MINUS    code*/
    USB_HID_KBD_KEYPAD_PLUS     = 87, /**<KBD_KEYPAD_PLUS     code*/
    USB_HID_KBD_KEYPAD_ENTER    = 88, /**<KBD_KEYPAD_ENTER    code*/
    USB_HID_KBD_KEYPAD_1        = 89, /**<KBD_KEYPAD_1        code*/
    USB_HID_KBD_KEYPAD_2        = 90, /**<KBD_KEYPAD_2        code*/
    USB_HID_KBD_KEYPAD_3        = 91, /**<KBD_KEYPAD_3        code*/
    USB_HID_KBD_KEYPAD_4        = 92, /**<KBD_KEYPAD_4        code*/
    USB_HID_KBD_KEYPAD_5        = 93, /**<KBD_KEYPAD_5        code*/
    USB_HID_KBD_KEYPAD_6        = 94, /**<KBD_KEYPAD_6        code*/
    USB_HID_KBD_KEYPAD_7        = 95, /**<KBD_KEYPAD_7        code*/
    USB_HID_KBD_KEYPAD_8        = 96, /**<KBD_KEYPAD_8        code*/
    USB_HID_KBD_KEYPAD_9        = 97, /**<KBD_KEYPAD_9        code*/
    USB_HID_KBD_KEYPAD_0        = 98, /**<KBD_KEYPAD_0        code*/
} usb_hid_kbd_codes_t;

/**
 * @brief HID keyboard modifier
 */
typedef enum {
    USB_HID_KBD_MODIFIER_NONE          = 0x00,  /**< MODIFIER_NONE        bit*/
    USB_HID_KBD_MODIFIER_LEFT_CTRL     = 0x01,  /**< MODIFIER_LEFT_CTRL   bit*/
    USB_HID_KBD_MODIFIER_LEFT_SHIFT    = 0x02,  /**< MODIFIER_LEFT_SHIFT  bit*/
    USB_HID_KBD_MODIFIER_LEFT_ALT      = 0x04,  /**< MODIFIER_LEFT_ALT    bit*/
    USB_HID_KBD_MODIFIER_LEFT_UI       = 0x08,  /**< MODIFIER_LEFT_UI     bit*/
    USB_HID_KBD_MODIFIER_RIGHT_CTRL    = 0x10,  /**< MODIFIER_RIGHT_CTRL  bit*/
    USB_HID_KBD_MODIFIER_RIGHT_SHIFT   = 0x20,  /**< MODIFIER_RIGHT_SHIFT bit*/
    USB_HID_KBD_MODIFIER_RIGHT_ALT     = 0x40,  /**< MODIFIER_RIGHT_ALT   bit*/
    USB_HID_KBD_MODIFIER_RIGHT_UI      = 0x80,  /**< MODIFIER_RIGHT_UI    bit*/
} usb_hid_kbd_modifier_t;

/**
 * @brief HID keyboard LEDs.
 */
typedef enum {
    USB_HID_KBD_LED_NUM_LOCK     = 0x01,  /**< LED_NUM_LOCK    id*/
    USB_HID_KBD_LED_CAPS_LOCK    = 0x02,  /**< LED_CAPS_LOCK   id*/
    USB_HID_KBD_LED_SCROLL_LOCK  = 0x04,  /**< LED_SCROLL_LOCK id*/
    USB_HID_KBD_LED_COMPOSE      = 0x08,  /**< LED_COMPOSE     id*/
    USB_HID_KBD_LED_KANA         = 0x10,  /**< LED_KANA        id*/
} usb_hid_kbd_led_t;

#pragma pack(push, 1)
typedef union{
    struct  {
        uint8_t  bLength;            //!< Length of descriptor.
        uint8_t  bDescriptorType;    //!< Descriptor type @ref USB_HID_DESCRIPTOR_HID.
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
gr55xx_usb_state_t gr55xx_hid_keyboard_init(void);
gr55xx_usb_state_t gr55xx_hid_send_keyboard_operation_report(uint8_t* p_op,uint16_t size);
void gr55xx_usb_hid_keyboard_read_data(uint8_t *p_data, uint16_t size);

#ifdef __cplusplus
}
#endif


#endif   /*__CDC_H__*/


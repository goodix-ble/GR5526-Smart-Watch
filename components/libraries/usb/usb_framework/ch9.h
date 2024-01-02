/*
 * Copyright (C) 2017, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
 */

#ifndef __USBD_CH9_H__
#define __USBD_CH9_H__

#include <stdint.h>

#define STRING_MANUFACTURER_IDX     2
#define STRING_PRODUCT_IDX          1
#define STRING_DESCRIPTION_IDX      0

#define USB_DIR_OUT                 0           /* to device */
#define USB_DIR_IN                  0x80        /* to host */

#define USB_TYPE_MASK               (0x03 << 5)
#define USB_TYPE_STANDARD           (0x00 << 5)
#define USB_TYPE_CLASS              (0x01 << 5)
#define USB_TYPE_VENDOR             (0x02 << 5)
#define USB_TYPE_RESERVED           (0x03 << 5)

#define USB_RECIP_MASK              0x1f
#define USB_RECIP_DEVICE            0x00
#define USB_RECIP_INTERFACE         0x01
#define USB_RECIP_ENDPOINT          0x02
#define USB_RECIP_OTHER             0x03
/* From Wireless USB 1.0 */
#define USB_RECIP_PORT              0x04
#define USB_RECIP_RPIPE             0x05

#define USB_REQ_GET_STATUS          0x00
#define USB_REQ_CLEAR_FEATURE       0x01
#define USB_REQ_SET_FEATURE         0x03
#define USB_REQ_SET_ADDRESS         0x05
#define USB_REQ_GET_DESCRIPTOR      0x06
#define USB_REQ_SET_DESCRIPTOR      0x07
#define USB_REQ_GET_CONFIGURATION   0x08
#define USB_REQ_SET_CONFIGURATION   0x09
#define USB_REQ_GET_INTERFACE       0x0A
#define USB_REQ_SET_INTERFACE       0x0B
#define USB_REQ_SYNCH_FRAME         0x0C

#define USB_REQ_SET_ENCRYPTION      0x0D    /* Wireless USB */
#define USB_REQ_GET_ENCRYPTION      0x0E
#define USB_REQ_RPIPE_ABORT         0x0E
#define USB_REQ_SET_HANDSHAKE       0x0F
#define USB_REQ_RPIPE_RESET         0x0F
#define USB_REQ_GET_HANDSHAKE       0x10
#define USB_REQ_SET_CONNECTION      0x11
#define USB_REQ_SET_SECURITY_DATA   0x12
#define USB_REQ_GET_SECURITY_DATA   0x13
#define USB_REQ_SET_WUSB_DATA       0x14
#define USB_REQ_LOOPBACK_DATA_WRITE 0x15
#define USB_REQ_LOOPBACK_DATA_READ  0x16
#define USB_REQ_SET_INTERFACE_DS    0x17

#define USB_DEVICE_SELF_POWERED     0   /* (read only) */
#define USB_DEVICE_REMOTE_WAKEUP    1   /* dev may initiate wakeup */
#define USB_DEVICE_TEST_MODE        2   /* (wired high speed only) */
#define USB_DEVICE_BATTERY          2   /* (wireless) */
#define USB_DEVICE_B_HNP_ENABLE     3   /* (otg) dev may initiate HNP */
#define USB_DEVICE_WUSB_DEVICE      3   /* (wireless)*/
#define USB_DEVICE_A_HNP_SUPPORT    4   /* (otg) RH port supports HNP */
#define USB_DEVICE_A_ALT_HNP_SUPPORT    5   /* (otg) other RH port does */
#define USB_DEVICE_DEBUG_MODE       6   /* (special devices only) */

#define USB_ENDPOINT_HALT           0   /* IN/OUT will STALL */

struct usb_ctrlrequest
{
    uint8_t bRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__ ((packed));

#define CONFIG_USB_GADGET_VBUS_DRAW     (500)   /* USB power current, unit: mA */

#define USB_DT_DEVICE                   0x01
#define USB_DT_CONFIG                   0x02
#define USB_DT_STRING                   0x03
#define USB_DT_INTERFACE                0x04
#define USB_DT_ENDPOINT                 0x05
#define USB_DT_DEVICE_QUALIFIER         0x06
#define USB_DT_OTHER_SPEED_CONFIG       0x07
#define USB_DT_INTERFACE_POWER          0x08
/* these are from a minor usb 2.0 revision (ECN) */
#define USB_DT_OTG                      0x09
#define USB_DT_DEBUG                    0x0a
#define USB_DT_INTERFACE_ASSOCIATION    0x0b
/* these are from the Wireless USB spec */
#define USB_DT_SECURITY                 0x0c
#define USB_DT_KEY                      0x0d
#define USB_DT_ENCRYPTION_TYPE          0x0e
#define USB_DT_BOS                      0x0f
#define USB_DT_DEVICE_CAPABILITY        0x10
#define USB_DT_WIRELESS_ENDPOINT_COMP   0x11
#define USB_DT_WIRE_ADAPTER             0x21
#define USB_DT_RPIPE                    0x22
#define USB_DT_CS_RADIO_CONTROL         0x23
/* From the T10 UAS specification */
#define USB_DT_PIPE_USAGE               0x24
/* From the USB 3.0 spec */
#define	USB_DT_SS_ENDPOINT_COMP         0x30


#define USB_DT_CS_DEVICE        (USB_TYPE_CLASS | USB_DT_DEVICE)
#define USB_DT_CS_CONFIG        (USB_TYPE_CLASS | USB_DT_CONFIG)
#define USB_DT_CS_STRING        (USB_TYPE_CLASS | USB_DT_STRING)
#define USB_DT_CS_INTERFACE     (USB_TYPE_CLASS | USB_DT_INTERFACE)
#define USB_DT_CS_ENDPOINT      (USB_TYPE_CLASS | USB_DT_ENDPOINT)

struct usb_descriptor_header
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
} __attribute__ ((packed));

struct usb_device_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} __attribute__ ((packed));

#define USB_DT_DEVICE_SIZE              18

#define USB_CLASS_PER_INTERFACE         0   /* for DeviceClass */
#define USB_CLASS_AUDIO                 1
#define USB_CLASS_COMM                  2
#define USB_CLASS_HID                   3
#define USB_CLASS_PHYSICAL              5
#define USB_CLASS_STILL_IMAGE           6
#define USB_CLASS_PRINTER               7
#define USB_CLASS_MASS_STORAGE          8
#define USB_CLASS_HUB                   9
#define USB_CLASS_CDC_DATA              0x0a
#define USB_CLASS_CSCID                 0x0b    /* chip+ smart card */
#define USB_CLASS_CONTENT_SEC           0x0d    /* content security */
#define USB_CLASS_VIDEO                 0x0e
#define USB_CLASS_WIRELESS_CONTROLLER   0xe0
#define USB_CLASS_MISC                  0xef
#define USB_CLASS_APP_SPEC              0xfe
#define USB_CLASS_VENDOR_SPEC           0xff

#define USB_SUBCLASS_VENDOR_SPEC        0xff


struct usb_config_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
} __attribute__ ((packed));

#define USB_DT_CONFIG_SIZE          9

#define USB_CONFIG_ATT_ONE          (1 << 7)    /* must be set */
#define USB_CONFIG_ATT_SELFPOWER    (1 << 6)    /* self powered */
#define USB_CONFIG_ATT_WAKEUP       (1 << 5)    /* can wakeup */
#define USB_CONFIG_ATT_BATTERY      (1 << 4)    /* battery powered */

struct usb_string_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint16_t wData[1];      /* UTF-16LE encoded */
} __attribute__ ((packed));

struct usb_interface_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
} __attribute__ ((packed));

#define USB_DT_INTERFACE_SIZE       9

struct usb_endpoint_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;

    uint8_t  bRefresh;
    uint8_t  bSynchAddress;
} __attribute__ ((packed));

#define USB_DT_ENDPOINT_SIZE        7
#define USB_DT_ENDPOINT_AUDIO_SIZE  9   /* Audio extension */



#define USB_ENDPOINT_NUMBER_MASK    0x0f    /* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK       0x80

#define USB_ENDPOINT_SYNCTYPE       0x0c
#define USB_ENDPOINT_SYNC_NONE      (0 << 2)
#define USB_ENDPOINT_SYNC_ASYNC     (1 << 2)
#define USB_ENDPOINT_SYNC_ADAPTIVE  (2 << 2)
#define USB_ENDPOINT_SYNC_SYNC      (3 << 2)

#define USB_ENDPOINT_XFERTYPE_MASK  0x03    /* in bmAttributes */
#define USB_ENDPOINT_XFER_CONTROL   0
#define USB_ENDPOINT_XFER_ISOC      1
#define USB_ENDPOINT_XFER_BULK      2
#define USB_ENDPOINT_XFER_INT       3
#define USB_ENDPOINT_MAX_ADJUSTABLE 0x80


struct usb_ss_ep_comp_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  bMaxBurst;
    uint8_t  bmAttributes;
    uint16_t wBytesPerInterval;
} __attribute__ ((packed));

#define USB_DT_SS_EP_COMP_SIZE      6
/* Bits 4:0 of bmAttributes if this is a bulk endpoint */
#define USB_SS_MAX_STREAMS(p)       (1 << (p & 0x1f))

struct usb_qualifier_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint8_t  bNumConfigurations;
    uint8_t  bRESERVED;
} __attribute__ ((packed));

struct usb_otg_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  bmAttributes;  /* support for HNP, SRP, etc */
} __attribute__ ((packed));

#define USB_OTG_SRP     (1 << 0)
#define USB_OTG_HNP     (1 << 1)    /* swap host/device roles */

struct usb_debug_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  bDebugInEndpoint;
    uint8_t  bDebugOutEndpoint;
} __attribute__((packed));

struct usb_interface_assoc_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  bFirstInterface;
    uint8_t  bInterfaceCount;
    uint8_t  bFunctionClass;
    uint8_t  bFunctionSubClass;
    uint8_t  bFunctionProtocol;
    uint8_t  iFunction;
} __attribute__ ((packed));

struct usb_security_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint16_t wTotalLength;
    uint8_t  bNumEncryptionTypes;
} __attribute__ ((packed));


struct usb_key_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  tTKID[3];
    uint8_t  bReserved;
    //uint8_t  bKeyData[0];
} __attribute__ ((packed));


struct usb_encryption_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  bEncryptionType;
#define	USB_ENC_TYPE_UNSECURE       0
#define	USB_ENC_TYPE_WIRED          1   /* non-wireless mode */
#define	USB_ENC_TYPE_CCM_1          2   /* aes128/cbc session */
#define	USB_ENC_TYPE_RSA_1          3   /* rsa3072/sha1 auth */
    uint8_t  bEncryptionValue;          /* use in SET_ENCRYPTION */
    uint8_t  bAuthKeyIndex;
} __attribute__ ((packed));


struct usb_bos_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint16_t wTotalLength;
    uint8_t  bNumDeviceCaps;
} __attribute__ ((packed));

struct usb_dev_cap_header
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDevCapabilityType;
} __attribute__ ((packed));

#define	USB_CAP_TYPE_WIRELESS_USB   1

struct usb_wireless_cap_descriptor /* Ultra Wide Band */
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDevCapabilityType;

    uint8_t  bmAttributes;
    #define USB_WIRELESS_P2P_DRD        (1 << 1)
    #define USB_WIRELESS_BEACON_MASK    (3 << 2)
    #define USB_WIRELESS_BEACON_SELF    (1 << 2)
    #define USB_WIRELESS_BEACON_DIRECTED    (2 << 2)
    #define USB_WIRELESS_BEACON_NONE    (3 << 2)
    uint16_t wPHYRates;                             /* bit rates, Mbps */
    #define USB_WIRELESS_PHY_53         (1 << 0)    /* always set */
    #define USB_WIRELESS_PHY_80         (1 << 1)
    #define USB_WIRELESS_PHY_107        (1 << 2)    /* always set */
    #define USB_WIRELESS_PHY_160        (1 << 3)
    #define USB_WIRELESS_PHY_200        (1 << 4)    /* always set */
    #define USB_WIRELESS_PHY_320        (1 << 5)
    #define USB_WIRELESS_PHY_400        (1 << 6)
    #define USB_WIRELESS_PHY_480        (1 << 7)
    uint8_t  bmTFITXPowerInfo;                      /* TFI power levels */
    uint8_t  bmFFITXPowerInfo;                      /* FFI power levels */
    uint16_t bmBandGroup;
    uint8_t  bReserved;
} __attribute__ ((packed));

#define	USB_CAP_TYPE_EXT    2

struct usb_ext_cap_descriptor /* Link Power Management */
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDevCapabilityType;
    //uint8_t bmAttributes[4];
    uint32_t bmAttributes;
    #define USB_LPM_SUPPORT                 (1 << 1)        /* supports LPM */
    #define USB_BESL_SUPPORT                (1 << 2)        /* supports BESL */
    #define USB_BESL_BASELINE_VALID         (1 << 3)        /* Baseline BESL valid*/
    #define USB_BESL_DEEP_VALID             (1 << 4)        /* Deep BESL valid */
    #define USB_GET_BESL_BASELINE(p)        (((p) & (0xf << 8)) >> 8)
    #define USB_GET_BESL_DEEP(p)            (((p) & (0xf << 12)) >> 12)
} __attribute__ ((packed));

/*
 * SuperSpeed USB Capability descriptor: Defines the set of SuperSpeed USB
 * specific device level capabilities
 */
#define USB_SS_CAP_TYPE         3
struct usb_ss_cap_descriptor /* Link Power Management */
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDevCapabilityType;
    uint8_t  bmAttributes;
    #define USB_LTM_SUPPORT                 (1 << 1)    /* supports LTM */
    uint16_t wSpeedSupported;
    #define USB_LOW_SPEED_OPERATION         (1)         /* Low speed operation */
    #define USB_FULL_SPEED_OPERATION        (1 << 1)    /* Full speed operation */
    #define USB_HIGH_SPEED_OPERATION        (1 << 2)    /* High speed operation */
    #define USB_5GBPS_OPERATION             (1 << 3)    /* Operation at 5Gbps */
    uint8_t  bFunctionalitySupport;
    uint8_t  bU1devExitLat;
    uint16_t bU2DevExitLat;
} __attribute__ ((packed));


struct usb_wireless_ep_comp_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  bMaxBurst;
    uint8_t  bMaxSequence;
    uint16_t wMaxStreamDelay;
    uint16_t wOverTheAirPacketSize;
    uint8_t  bOverTheAirInterval;
    uint8_t  bmCompAttributes;
    #define USB_ENDPOINT_SWITCH_MASK    0x03    /* in bmCompAttributes */
    #define USB_ENDPOINT_SWITCH_NO      0
    #define USB_ENDPOINT_SWITCH_SWITCH  1
    #define USB_ENDPOINT_SWITCH_SCALE   2
} __attribute__ ((packed));

struct usb_handshake
{
    uint8_t bMessageNumber;
    uint8_t bStatus;
    uint8_t tTKID[3];
    uint8_t bReserved;
    uint8_t CDID[16];
    uint8_t nonce[16];
    uint8_t MIC[8];
} __attribute__ ((packed));


struct usb_connection_context
{
    uint8_t CHID[16];       /* persistent host id */
    uint8_t CDID[16];       /* device id (unique w/in host context) */
    uint8_t CK[16];         /* connection key */
} __attribute__ ((packed));


struct usb_dcd_config_params
{
    uint8_t  bU1devExitLat;                     /* U1 Device exit Latency */
    #define USB_DEFAULT_U1_DEV_EXIT_LAT     0x01    /* Less then 1 microsec */
    uint16_t bU2DevExitLat;                     /* U2 Device exit Latency */
    #define USB_DEFAULT_U2_DEV_EXIT_LAT     0x1F4   /* Less then 500 microsec */
} __attribute__ ((packed));


enum usb_device_speed
{
    USB_SPEED_UNKNOWN = 0,              /* enumerating */
    USB_SPEED_LOW, USB_SPEED_FULL,      /* usb 1.1 */
    USB_SPEED_HIGH,                     /* usb 2.0 */
    USB_SPEED_WIRELESS,                 /* wireless (usb 2.5) */
    USB_SPEED_SUPER,                    /* usb 3.0 */
};

enum usb_device_state
{
    USB_STATE_NOTATTACHED = 0,
    USB_STATE_ATTACHED,
    USB_STATE_POWERED,              /* wired */
    USB_STATE_RECONNECTING,         /* auth */
    USB_STATE_UNAUTHENTICATED,      /* auth */
    USB_STATE_DEFAULT,              /* limited function */
    USB_STATE_ADDRESS,
    USB_STATE_CONFIGURED,           /* most functions */
    USB_STATE_SUSPENDED
};




#endif



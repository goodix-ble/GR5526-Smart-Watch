/*
 * Copyright (C) 2021, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
*  ===================================================================
*                        Audio Class Driver Description
*  ===================================================================
*   This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
*   Audio Devices V1.0 Mar 18, 98".
*   This driver implements the following aspects of the specification:
*     - Arch
*              IT --------> FU ---------> OUT
*             (ID1)        (ID2)         (ID3)
*           (USB OUT)                (EP5 Headhone)
*              IT --------> FU ---------> OUT
*             (ID4)        (ID5)         (ID6)
*       (EP4 Mircophone)                (USB IN)
*     - Audio Class-Specific AC Interfaces
*     - Audio Class-Specific AS Interfaces
*     - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
*     - Audio Feature Unit (limited to Mute and volume control)
*     - Audio Synchronization type: Asynchronous
*     - Single fixed audio sampling rate
*     - 4 Interfaces
*          0.Headphone Control Interface
*          1.Headphone Streaming Interface
*          2.Micphone Control Interface
*          3.Micphone Streaming Interface
*   @note
*    The Audio Class 1.0 is based on USB Specification 1.0 and thus supports only
*    Low and Full speed modes and does not allow High Speed transfers.
*    Please refer to "USB Device Class Definition for Audio Devices V1.0 Mar 18, 98"
*    for more details.
 */
#include <string.h>
#include "gr55xx_usb_composite.h"
#include "gr55xx_usb_audio.h"
#include "gr55xx_usb_server.h"
#include "ring_buffer.h"

extern usb_handle_t g_usb_handle;
struct audio_device g_audio;
struct usb_request *g_mic_req, *g_hp_req;
__ALIGNED(4) uint8_t g_mic_buffer[AUDIO_MIC_BUFFER_SIZE] ={0};
__ALIGNED(4) uint8_t g_hp_buffer[AUDIO_HP_BUFFER_SIZE] ={0};

#define DATA_BUFFER_SIZE (AUDIO_HP_BUFFER_SIZE*4)
uint8_t headphone_buffer[DATA_BUFFER_SIZE];
ring_buffer_t g_headphone_fifo;


#define AUDIO_VENDOR_ID                   0x27C6
#define AUDIO_PRODUCT_ID                  0x2200
#define AUDIO_DEVICE_BCD                  0x0100  /* 1.00 */

#define AUDIO_AC_INTERFACE_HEADER_BCD     0x0100  /* 1.00 */

#define AUDIO_STRING_INTERFACE_HP_AC      0
#define AUDIO_STRING_INTERFACE_HP_AS      1
#define AUDIO_STRING_INTERFACE_MIC_AC     2
#define AUDIO_STRING_INTERFACE_MIC_AS     3

#define AUDIO_STATUS_ACTIVATED            0x00000001
#define AUDIO_STATUS_MMAPPED              0x00000002

#define AUDIO_READY_BITS                  (AUDIO_STATUS_ACTIVATED | AUDIO_STATUS_MMAPPED)

#define AUDIO_HP_VOLUME_MIN               0x0000
#define AUDIO_HP_VOLUME_MAX               (256*10)
#define AUDIO_HP_VOLUME_RES               1

#define AUDIO_MIC_VOLUME_MIN              0x0000
#define AUDIO_MIC_VOLUME_MAX              (256*10)
#define AUDIO_MIC_VOLUME_RES              1

void mic_data_init(void)
{
    for(uint32_t i = 0; i < AUDIO_MIC_BUFFER_SIZE;)
    {
        g_mic_buffer[i] = i;
        i++;
    }
}

static struct usb_device_descriptor g_audio_device_desc =
{
    .bLength            =   USB_DT_DEVICE_SIZE,
    .bDescriptorType    =   USB_DT_DEVICE,
    .bcdUSB             =   0x0200,
    .bDeviceClass       =   0x00,
    .bDeviceSubClass    =   0x00,
    .idVendor           =   AUDIO_VENDOR_ID,
    .idProduct          =   AUDIO_PRODUCT_ID,
    .bcdDevice          =   AUDIO_DEVICE_BCD,
    .iManufacturer      =   1,
    .iProduct           =   2,
    .iSerialNumber      =   3,
    .bNumConfigurations =   1,
};


/*********************start of audio Headphone control descriptor******************************/
static struct usb_interface_descriptor g_audio_hp_ac_intf_desc =
{
    .bLength            =   USB_DT_INTERFACE_SIZE,
    .bDescriptorType    =   USB_DT_INTERFACE,
//    .bInterfaceNumber   =   0,
    .bAlternateSetting  =   0,
    .bNumEndpoints      =   0,
    .bInterfaceClass    =   USB_AUDIO_INTERFACE_CLASS,
    .bInterfaceSubClass =   USB_AUDIO_SUBCLASS_AUDIOCONTROL,
    .bInterfaceProtocol =   0,
    .iInterface         =   0,
};


static struct usb_audio_ac_inft_header_desc g_audio_hp_ac_inft_header_desc =
{
    .bDescriptorType    =   USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE,
    .bDescriptorSubType =   USB_AUDIO_AC_INTF_SUBTYPE_HEADER,
    .bcdADC             =   AUDIO_AC_INTERFACE_HEADER_BCD,
    .bInCollection      =   0x01,
};

static struct usb_audio_input_terminal_desc g_audio_hp_input_terminal_desc =
{
    .bLength            =   0x0C,
    .bDescriptorType    =   USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE,
    .bDescriptorSubType =   USB_AUDIO_AC_INTF_SUBTYPE_INPUT_TERMINAL,
    .bTerminalID        =   AUDIO_HP_IT_ID,
    .wTerminalType      =   USB_AUDIO_TERMINAL_USB_STREAMING,
    .bAssocTerminal     =   0,
    .bNrChannels        =   0x02,
    .wChannelConfig     =   0x03, //  Front Left and Front Right
    .iChannelNames      =   0,
    .iTerminal          =   0,
};

struct usb_audio_feature_unit_desc g_audio_hp_feature_unit_desc =
{
    .bDescriptorType    = USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE,
    .bDescriptorSubType = USB_AUDIO_AC_INTF_SUBTYPE_FEATURE_UNIT,
    .bUnitID            = AUDIO_HP_FU_ID,
    .bSourceID          = AUDIO_HP_IT_ID,
    .bControlSize       = 0x02,  //16 bit
    .bmaControls[0]     = USB_AUDIO_FU_BIT_MUTE | USB_AUDIO_FU_BIT_VOLUME, // Master channle
    .bmaControls[1]     = 0x00,
    .bmaControls[2]     = 0x00,// Left channle
    .bmaControls[3]     = 0x00,
    .bmaControls[4]     = 0x00,// Right channle
    .bmaControls[5]     = 0x00,
};


struct usb_audio_output_terminal_desc g_audio_hp_output_terminal_desc =
{
    .bLength            = 0x09,
    .bDescriptorType    = USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE,
    .bDescriptorSubType = USB_AUDIO_AC_INTF_SUBTYPE_OUTPUT_TERNINAL,
    .bTerminalID        = AUDIO_HP_OT_ID,
    .wTerminalType      = USB_AUDIO_TERMINAL_OUT_HEADPHONES,
    .bAssocTerminal     = 0,
    .bSourceID          = AUDIO_HP_FU_ID,
    .iTerminal          = 0,
};
/*********************end of audio Headphone control descriptor********************************/

/*********************start of audio Headphone streaming descriptor****************************/
/* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
/* Interface 1, Alternate Setting 0                                             */
struct usb_interface_descriptor g_audio_hp_as_intf_desc_alt0 =
{
    .bLength            = USB_DT_INTERFACE_SIZE,
    .bDescriptorType    = USB_DT_INTERFACE,
//    .bInterfaceNumber   = 0x01,
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 0,
    .bInterfaceClass    = USB_AUDIO_INTERFACE_CLASS,
    .bInterfaceSubClass = USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
    .bInterfaceProtocol = 0,
    .iInterface         = 0,
};

/* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
/* Interface 1, Alternate Setting 1                                           */
struct usb_interface_descriptor g_audio_hp_as_intf_desc_alt1 =
{
    .bLength            = USB_DT_INTERFACE_SIZE,
    .bDescriptorType    = USB_DT_INTERFACE,
//    .bInterfaceNumber   = 0x01,
    .bAlternateSetting  = 0x01,
    .bNumEndpoints      = 0x01,
    .bInterfaceClass    = USB_AUDIO_INTERFACE_CLASS,
    .bInterfaceSubClass = USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
    .bInterfaceProtocol = 0,
    .iInterface         = 0,
};

struct usb_audio_as_intf_desc g_audio_hp_as_intf_general_desc =
{
    .bLength            = 0x07,
    .bDescriptorType    = USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE,
    .bDescriptorSubType = USB_AUDIO_AS_INTF_SUBTYPE_GENERAL,
    .bTerminalLink      = AUDIO_HP_IT_ID,  // hp input terminal
    .bDelay             = 0,
    .wFormatTag         = USB_AUDIO_AS_IFACE_FORMAT_PCM,
};

struct usb_audio_as_format_type_III_desc g_audio_hp_as_format_type_III_desc =
{
    .bLength            = 0x0B,
    .bDescriptorType    = USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE,
    .bDescriptorSubType = USB_AUDIO_AS_INTF_SUBTYPE_FORMAT_TYPE,
    .bFormatType        = 0x03,
    .bNrChannels        = 0x02,

#if AUDIO_HP_DATA_BIT == AUDIO_SAMPLE_16BIT
    .bSubframeSize      = 0x02,
    .bBitResolution     = 0x10,
#elif AUDIO_HP_DATA_BIT == AUDIO_SAMPLE_24BIT
    .bSubframeSize      = 0x03,
    .bBitResolution     = 0x18,
#elif AUDIO_HP_DATA_BIT == AUDIO_SAMPLE_32BIT
    .bSubframeSize      = 0x04,
    .bBitResolution     = 0x20,
#endif

    .bSamFreqType       = 0x01,

#if AUDIO_HP_DATA_SAMPLE == AUDIO_SAMPLE_96K
    .tSamFreq[0] = 0x00,
    .tSamFreq[1] = 0x77,
    .tSamFreq[2] = 0x01,
#elif AUDIO_HP_DATA_SAMPLE == AUDIO_SAMPLE_48K
    .tSamFreq[0] = 0x80,
    .tSamFreq[1] = 0xBB,
    .tSamFreq[2] = 0x00,
#elif AUDIO_HP_DATA_SAMPLE == AUDIO_SAMPLE_44_1K
    .tSamFreq[0] = 0x44,
    .tSamFreq[1] = 0xAC,
    .tSamFreq[2] = 0x00,
#elif AUDIO_HP_DATA_SAMPLE == AUDIO_SAMPLE_16K
    .tSamFreq[0] = 0x80,
    .tSamFreq[1] = 0x3E,
    .tSamFreq[2] = 0x00,
#elif AUDIO_HP_DATA_SAMPLE == AUDIO_SAMPLE_8K
    .tSamFreq[0] = 0x40,
    .tSamFreq[1] = 0x1F,
    .tSamFreq[2] = 0x00,
#endif
};

struct usb_audio_as_endpoint_desc g_audio_hp_as_endpoint_general_desc =
{
    .bLength            = 0x07,
    .bDescriptorType    = USB_AUDIO_SPECIFIC_DESCRIPTOR_ENDPOINT,
    .bDescriptorSubType = USB_AUDIO_EP_SUBTYPE_GENERAL,
    .bmAttributes       = 0,
    .bLockDelayUnits    = 0,
    .wLockDelay         = 0,
};

static struct usb_endpoint_descriptor g_audio_hp_iso_out_desc =
{
    .bLength            =   USB_DT_ENDPOINT_AUDIO_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,

    .bEndpointAddress   =   USB_DIR_OUT,
    .bmAttributes       =   USB_ENDPOINT_XFER_ISOC,
    .bInterval          =   0x01, //One packet per frame
    /* wMaxPacketSize set by autoconfiguration */
};
/*********************end of audio Headphone streaming descriptor********************************/


/*********************start of audio Microphone control descriptor******************************/
static struct usb_interface_descriptor g_audio_mic_ac_intf_desc =
{
    .bLength            =   USB_DT_INTERFACE_SIZE,
    .bDescriptorType    =   USB_DT_INTERFACE,
//    .bInterfaceNumber   =   0x02,
    .bAlternateSetting  =   0,
    .bNumEndpoints      =   0,
    .bInterfaceClass    =   USB_AUDIO_INTERFACE_CLASS,
    .bInterfaceSubClass =   USB_AUDIO_SUBCLASS_AUDIOCONTROL,
    .bInterfaceProtocol =   0,
    .iInterface         =   0,
};

static struct usb_audio_ac_inft_header_desc g_audio_mic_ac_inft_header_desc =
{
    .bDescriptorType    =   USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE,
    .bDescriptorSubType =   USB_AUDIO_AC_INTF_SUBTYPE_HEADER,
    .bcdADC             =   AUDIO_AC_INTERFACE_HEADER_BCD,
    .bInCollection      =   0x01,
};

static struct usb_audio_input_terminal_desc g_audio_mic_input_terminal_desc =
{
    .bLength            =   0x0C,
    .bDescriptorType    =   USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE,
    .bDescriptorSubType =   USB_AUDIO_AC_INTF_SUBTYPE_INPUT_TERMINAL,
    .bTerminalID        =   AUDIO_MIC_IT_ID,
    .wTerminalType      =   USB_AUDIO_TERMINAL_IN_MICROPHONE,
    .bAssocTerminal     =   0,
    .bNrChannels        =   0x02,
    .wChannelConfig     =   0x03, //  Front Left and Front Right
    .iChannelNames      =   0,
    .iTerminal          =   0,
};

struct usb_audio_feature_unit_desc g_audio_mic_feature_unit_desc =
{
    .bDescriptorType    = USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE,
    .bDescriptorSubType = USB_AUDIO_AC_INTF_SUBTYPE_FEATURE_UNIT,
    .bUnitID            = AUDIO_MIC_FU_ID,
    .bSourceID          = AUDIO_MIC_IT_ID,
    .bControlSize       = 0x02,  //bmaControls size 16bit
    .bmaControls[0]     = USB_AUDIO_FU_BIT_MUTE | USB_AUDIO_FU_BIT_VOLUME,
    .bmaControls[1]     = 0x00,
    .bmaControls[2]     = 0x00,
    .bmaControls[3]     = 0x00,
    .bmaControls[4]     = 0x00,
    .bmaControls[5]     = 0x00,
};

struct usb_audio_output_terminal_desc g_audio_mic_output_terminal_desc =
{
    .bLength            = 0x09,
    .bDescriptorType    = USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE,
    .bDescriptorSubType = USB_AUDIO_AC_INTF_SUBTYPE_OUTPUT_TERNINAL,
    .bTerminalID        = AUDIO_MIC_OT_ID,
    .wTerminalType      = USB_AUDIO_TERMINAL_USB_STREAMING,
    .bAssocTerminal     = 0,
    .bSourceID          = AUDIO_MIC_FU_ID,
    .iTerminal          = 0,
};

/*********************end of audio Microphone control descriptor********************************/


/*********************start of audio Microphone streaming descriptor********************************/
/* USB Microphone Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
/* Interface 3, Alternate Setting 0                                               */
struct usb_interface_descriptor g_audio_mic_as_intf_desc_alt0 =
{
    .bLength            = USB_DT_INTERFACE_SIZE,
    .bDescriptorType    = USB_DT_INTERFACE,
//  .bInterfaceNumber   = 0x03,
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 0,
    .bInterfaceClass    = USB_AUDIO_INTERFACE_CLASS,
    .bInterfaceSubClass = USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
    .bInterfaceProtocol = 0,
    .iInterface         = 0,
};

/* USB Microphone Standard AS Interface Descriptor - Audio Streaming Operational */
/* Interface 3, Alternate Setting 1                                           */
struct usb_interface_descriptor g_audio_mic_as_intf_desc_alt1 =
{
    .bLength            = USB_DT_INTERFACE_SIZE,
    .bDescriptorType    = USB_DT_INTERFACE,
//    .bInterfaceNumber   = 0x03,
    .bAlternateSetting  = 0x01,
    .bNumEndpoints      = 0x01,
    .bInterfaceClass    = USB_AUDIO_INTERFACE_CLASS,
    .bInterfaceSubClass = USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
    .bInterfaceProtocol = 0,
    .iInterface         = 0,
};

struct usb_audio_as_intf_desc g_audio_mic_as_intf_general_desc =
{
    .bLength            = 0x07,
    .bDescriptorType    = USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE,
    .bDescriptorSubType = USB_AUDIO_AS_INTF_SUBTYPE_GENERAL,
    .bTerminalLink      = AUDIO_MIC_OT_ID,
    .bDelay             = 0,
    .wFormatTag         = USB_AUDIO_AS_IFACE_FORMAT_PCM,
};

struct usb_audio_as_format_type_I_desc g_audio_mic_as_format_type_I_desc =
{
    .bLength            = 0x0B,
    .bDescriptorType    = USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE,
    .bDescriptorSubType = USB_AUDIO_AS_INTF_SUBTYPE_FORMAT_TYPE,
    .bFormatType        = 0x01,
    .bNrChannels        = 0x02,

#if AUDIO_MIC_DATA_BIT == AUDIO_SAMPLE_16BIT
    .bSubframeSize      = 0x02,
    .bBitResolution     = 0x10,
#elif AUDIO_MIC_DATA_BIT == AUDIO_SAMPLE_24BIT
    .bSubframeSize      = 0x03,
    .bBitResolution     = 0x18,
#elif AUDIO_MIC_DATA_BIT == AUDIO_SAMPLE_32BIT
    .bSubframeSize      = 0x04,
    .bBitResolution     = 0x20,
#endif

    .bSamFreqType       = 1,


#if AUDIO_MIC_DATA_SAMPLE == AUDIO_SAMPLE_96K
    .tSamFreq[0] = 0x00,
    .tSamFreq[1] = 0x77,
    .tSamFreq[2] = 0x01,

#elif AUDIO_MIC_DATA_SAMPLE == AUDIO_SAMPLE_48K
    .tSamFreq[0] = 0x80,
    .tSamFreq[1] = 0xBB,
    .tSamFreq[2] = 0x00,

#elif AUDIO_MIC_DATA_SAMPLE == AUDIO_SAMPLE_44_1K
    .tSamFreq[0] = 0x44,
    .tSamFreq[1] = 0xAC,
    .tSamFreq[2] = 0x00,

#elif AUDIO_MIC_DATA_SAMPLE == AUDIO_SAMPLE_16K
    .tSamFreq[0] = 0x80,
    .tSamFreq[1] = 0x3E,
    .tSamFreq[2] = 0x00,

#elif AUDIO_MIC_DATA_SAMPLE == AUDIO_SAMPLE_8K
    .tSamFreq[0] = 0x40,
    .tSamFreq[1] = 0x1F,
    .tSamFreq[2] = 0x00,
#endif
};

struct usb_audio_as_endpoint_desc g_audio_mic_as_endpoint_general_desc =
{
    .bLength            = 0x07,
    .bDescriptorType    = USB_AUDIO_SPECIFIC_DESCRIPTOR_ENDPOINT,
    .bDescriptorSubType = USB_AUDIO_EP_SUBTYPE_GENERAL,
    .bmAttributes       = 0,
    .bLockDelayUnits    = 0,
    .wLockDelay         = 0,
};

static struct usb_endpoint_descriptor g_audio_mic_iso_in_desc =
{
    .bLength            =   USB_DT_ENDPOINT_AUDIO_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,

    .bEndpointAddress   =   USB_DIR_IN,
    .bmAttributes       =   USB_ENDPOINT_XFER_ISOC,
    .bInterval          =   0x01,
    /* wMaxPacketSize set by autoconfiguration */
};
/*********************end of audio Microphone streaming descriptor**********************************/



static struct usb_descriptor_header *g_audio_function[] =
{
    /*************************Headphone function******************************/
    (struct usb_descriptor_header *) &g_audio_hp_ac_intf_desc,
    (struct usb_descriptor_header *) &g_audio_hp_ac_inft_header_desc,
    (struct usb_descriptor_header *) &g_audio_hp_input_terminal_desc,
    (struct usb_descriptor_header *) &g_audio_hp_feature_unit_desc,
    (struct usb_descriptor_header *) &g_audio_hp_output_terminal_desc,
    (struct usb_descriptor_header *) &g_audio_hp_as_intf_desc_alt0,
    (struct usb_descriptor_header *) &g_audio_hp_as_intf_desc_alt1,
    (struct usb_descriptor_header *) &g_audio_hp_as_intf_general_desc,
    (struct usb_descriptor_header *) &g_audio_hp_as_format_type_III_desc,
    (struct usb_descriptor_header *) &g_audio_hp_as_endpoint_general_desc,
    (struct usb_descriptor_header *) &g_audio_hp_iso_out_desc,

    /*************************Microphone function******************************/
    (struct usb_descriptor_header *) &g_audio_mic_ac_intf_desc,
    (struct usb_descriptor_header *) &g_audio_mic_ac_inft_header_desc,
    (struct usb_descriptor_header *) &g_audio_mic_input_terminal_desc,
    (struct usb_descriptor_header *) &g_audio_mic_feature_unit_desc,
    (struct usb_descriptor_header *) &g_audio_mic_output_terminal_desc,
    (struct usb_descriptor_header *) &g_audio_mic_as_intf_desc_alt0,
    (struct usb_descriptor_header *) &g_audio_mic_as_intf_desc_alt1,
    (struct usb_descriptor_header *) &g_audio_mic_as_intf_general_desc,
    (struct usb_descriptor_header *) &g_audio_mic_as_format_type_I_desc,
    (struct usb_descriptor_header *) &g_audio_mic_as_endpoint_general_desc,
    (struct usb_descriptor_header *) &g_audio_mic_iso_in_desc,
    NULL,
};

static char g_audio_vendor_label[]     = "Goodix Technology Co., Ltd.";
static char g_audio_product_label[]    = "Gr552x USB2.0 Audio";
static char g_audio_config_label[]     = "Goodix BLE Audio";

static struct usb_string g_audio_strings[] =
{
    [STRING_DESCRIPTION_IDX].s  = g_audio_config_label,
    [STRING_PRODUCT_IDX].s      = g_audio_product_label,
    [STRING_MANUFACTURER_IDX].s = g_audio_vendor_label,
};

static struct usb_gadget_strings g_audio_device_strings =
{
    .language   = 0x0409,   /* en-us */
    .strings    = g_audio_strings,
};

static struct usb_string g_audio_en_us_strings[] =
{
    [AUDIO_STRING_INTERFACE_HP_AC].s  = "GR552x Audio Headphone Control Model",
    [AUDIO_STRING_INTERFACE_HP_AS].s  = "GR552x Audio Headphone streaming",
    [AUDIO_STRING_INTERFACE_MIC_AC].s = "GR552x Audio Microphone Control Model",
    [AUDIO_STRING_INTERFACE_MIC_AS].s = "GR552x Audio Microphone streaming",
};

//uvc_function_strings
static struct usb_gadget_strings g_audio_function_strings =
{
    .language   = 0x0409,   /* en-us */
    .strings    = g_audio_en_us_strings,
};


static int audio_ep_callback(struct usb_gadget *gadget,struct usb_ep *ep);
static int hp_feature_unit_control(const struct usb_ctrlrequest *ctrl, struct usb_request *req);
static int mic_feature_unit_control(const struct usb_ctrlrequest *ctrl, struct usb_request *req);
static int audio_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl);
static int audio_function_set_alt(struct usb_function *f, unsigned interface_id, unsigned alternate_setting);
static void audio_mic_complete(struct usb_ep *ep, struct usb_request *req);
static void audio_hp_complete(struct usb_ep *ep, struct usb_request *req);


static int audio_function_bind(struct usb_configuration *c, struct usb_function *f)
{
    struct usb_ep *ep;
    int ret = GR_DRV_USB_ERROR;

    struct usb_composite_dev *cdev = c->cdev;

    /* Allocate endpoints. */
    ep = gr55xx_usb_ep_find(cdev->gadget, "ep5");
    if (!ep)
    {
        USB_ERROR("ERROR: [%s]: Unable to allocate g_audio_hp_iso_out_desc EP", __FUNCTION__);
        goto error;
    }
    g_audio_hp_iso_out_desc.bEndpointAddress |= USB_EP5;
#if AUDIO_HP_DATA_SAMPLE == AUDIO_SAMPLE_44_1K
    g_audio_hp_iso_out_desc.wMaxPacketSize = AUDIO_HP_FRAME_SIZE_MAX;
#else
    g_audio_hp_iso_out_desc.wMaxPacketSize = AUDIO_HP_FRAME_SIZE;
#endif
    g_audio.as_out = ep;
    ep->driver_data = (void *)&g_audio;

    /* Allocate endpoints. */
    ep = gr55xx_usb_ep_find(cdev->gadget, "ep4");
    if (!ep)
    {
        USB_ERROR("ERROR: Unable to allocate g_audio_mic_iso_in_desc EP");
        goto error;
    }
    g_audio_mic_iso_in_desc.bEndpointAddress |= USB_EP4;
#if AUDIO_MIC_DATA_SAMPLE == AUDIO_SAMPLE_44_1K
    g_audio_mic_iso_in_desc.wMaxPacketSize = AUDIO_MIC_FRAME_SIZE_MAX;
#else
    g_audio_mic_iso_in_desc.wMaxPacketSize = AUDIO_MIC_FRAME_SIZE;
#endif
    g_audio.as_in = ep;
    ep->driver_data = (void *)&g_audio;

    if ((ret = gr55xx_usb_interface_id(c, f)) < 0)
    {
        goto error;
    }
    g_audio_hp_ac_intf_desc.bInterfaceNumber = ret;

    if ((ret = gr55xx_usb_interface_id(c, f)) < 0)
    {
        goto error;
    }
    g_audio_hp_as_intf_desc_alt0.bInterfaceNumber = ret;
    g_audio_hp_as_intf_desc_alt1.bInterfaceNumber = ret;

    if ((ret = gr55xx_usb_interface_id(c, f)) < 0)
    {
        goto error;
    }
    g_audio_mic_ac_intf_desc.bInterfaceNumber = ret;

    if ((ret = gr55xx_usb_interface_id(c, f)) < 0)
    {
        goto error;
    }
    g_audio_mic_as_intf_desc_alt0.bInterfaceNumber = ret;
    g_audio_mic_as_intf_desc_alt1.bInterfaceNumber = ret;

    ret = GR_DRV_USB_OK;

    f->descriptors = gr55xx_usb_copy_descriptors(g_audio_function);
    if (!f->descriptors)
    {
        ret = GR_DRV_USB_ERROR_NOMEM;
        goto error;
    }

    return ret;
error:
    USB_ERROR("ERROR: return error\r\n");
    return ret;
}


static int hp_feature_unit_control(const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
    int value = ctrl->wLength;
    uint8_t ctrl_selector = U16_HIGH(ctrl->wValue);

    switch (ctrl_selector)
    {
    case USB_AUDIO_FEATURE_UNIT_CONTROL_MUTE:
        USB_LOG("Headphone MUTE control channel:%d \r\n", U16_LOW(ctrl->wValue));
        req->complete = audio_hp_set_mute;
        break;
    case USB_AUDIO_FEATURE_UNIT_CONTROL_VOLUME:
        switch (ctrl->bRequest)
        {
        case USB_AUDIO_REQ_SET_CUR:
            req->complete = audio_hp_set_volume;
            break;
        case USB_AUDIO_REQ_GET_CUR:
            USB_LOG("USB_AUDIO_REQ_GET_CUR \r\n");
            req->buf[0] = U16_LOW(g_audio.hp_volume);
            req->buf[1] = U16_HIGH(g_audio.hp_volume);
            break;
        case USB_AUDIO_REQ_GET_MIN:
            USB_LOG("USB_AUDIO_REQ_GET_MIN \r\n");
            req->buf[0] = U16_LOW(AUDIO_HP_VOLUME_MIN);
            req->buf[1] = U16_HIGH(AUDIO_HP_VOLUME_MIN);
            break;
        case USB_AUDIO_REQ_GET_MAX:
            USB_LOG("USB_AUDIO_REQ_GET_MAX \r\n");
            req->buf[0] = U16_LOW(AUDIO_HP_VOLUME_MAX);
            req->buf[1] = U16_HIGH(AUDIO_HP_VOLUME_MAX);
            break;
        case USB_AUDIO_REQ_SET_RES:
            USB_LOG("USB_AUDIO_REQ_SET_RES \r\n");
            break;

        case USB_AUDIO_REQ_GET_RES:
            USB_LOG("USB_AUDIO_REQ_GET_RES \r\n");
            req->buf[0] = U16_LOW(AUDIO_HP_VOLUME_RES);
            req->buf[1] = U16_HIGH(AUDIO_HP_VOLUME_RES);
            break;

        case USB_AUDIO_REQ_GET_MEM:
            USB_LOG("USB_AUDIO_REQ_GET_MEM \r\n");
            break;
        default:
            USB_ERROR("Unsupport bRequest:%d \r\n", ctrl->bRequest);
            value = GR_DRV_USB_ERROR_NOTSUPP;
        }
        break;
    default:
        USB_ERROR("Unsupport ctrl_selector:%d \r\n", ctrl_selector);
        value = GR_DRV_USB_ERROR_NOTSUPP;
    }
    return value;
}

static int mic_feature_unit_control(const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
    int value = ctrl->wLength;;

    uint8_t ctrl_selector = U16_HIGH(ctrl->wValue);

    switch (ctrl_selector)
    {
    case USB_AUDIO_FEATURE_UNIT_CONTROL_MUTE:
        USB_LOG("MIC MUTE control channel:%d \r\n", U16_LOW(ctrl->wValue));
        req->complete = audio_mic_set_mute;
        break;
    case USB_AUDIO_FEATURE_UNIT_CONTROL_VOLUME:
        switch (ctrl->bRequest)
        {
        case USB_AUDIO_REQ_SET_CUR:
            req->complete = audio_mic_set_volume;
            break;
        case USB_AUDIO_REQ_GET_CUR:
            USB_LOG("USB_AUDIO_REQ_GET_CUR mic \r\n");
            req->buf[0] = U16_LOW(g_audio.mic_volume);
            req->buf[1] = U16_HIGH(g_audio.mic_volume);
            break;
        case USB_AUDIO_REQ_GET_MIN:
            USB_LOG("USB_AUDIO_REQ_GET_MIN mic \r\n");
            req->buf[0] = U16_LOW(AUDIO_MIC_VOLUME_MIN);
            req->buf[1] = U16_HIGH(AUDIO_MIC_VOLUME_MIN);
            break;
        case USB_AUDIO_REQ_GET_MAX:
            USB_LOG("USB_AUDIO_REQ_GET_MAX mic \r\n");
            req->buf[0] = U16_LOW(AUDIO_MIC_VOLUME_MAX);
            req->buf[1] = U16_HIGH(AUDIO_MIC_VOLUME_MAX);
            break;
        case USB_AUDIO_REQ_SET_RES:
            USB_LOG("USB_AUDIO_REQ_SET_RES mic \r\n");
            break;
        case USB_AUDIO_REQ_GET_RES:
            USB_LOG("USB_AUDIO_REQ_GET_RES mic \r\n");
            req->buf[0] = U16_LOW(AUDIO_MIC_VOLUME_RES);
            req->buf[1] = U16_HIGH(AUDIO_MIC_VOLUME_RES);
            break;
        case USB_AUDIO_REQ_GET_MEM:
            USB_LOG("USB_AUDIO_REQ_GET_MEM mic \r\n");
            break;
        default:
            USB_ERROR("Unsupport bRequest:%d \r\n", ctrl->bRequest);
            value = GR_DRV_USB_ERROR_NOTSUPP;
        }
        break;
    default:
        USB_ERROR("Unsupport ctrl_selector:%d \r\n", ctrl_selector);
        value = GR_DRV_USB_ERROR_NOTSUPP;
        break;
    }
   return value;
}

static int audio_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
    struct usb_ep *ep = g_audio.func.config->cdev->gadget->ep0;
    struct usb_request *req = g_audio.func.config->cdev->req;

    int value = GR_DRV_USB_ERROR_NOTSUPP;
    USB_LOG("audio setup bRequestType:%02x  bRequest:%02x  wValue:%04x  wIndex:%04x  wLength:%04x \r\n",ctrl->bRequestType, ctrl->bRequest, (ctrl->wValue),(ctrl->wIndex), (ctrl->wLength));

    uint8_t entity_id = U16_HIGH(ctrl->wIndex);
    switch(entity_id)
    {
        case AUDIO_HP_FU_ID:
            value = hp_feature_unit_control(ctrl, req);
           break;
        case AUDIO_MIC_FU_ID:
            value = mic_feature_unit_control(ctrl, req);
           break;
        default:
            USB_ERROR("ERROR: Unsupport entity_id:%d\r\n", entity_id);
    }

    /* respond with data transfer or status phase? */
    if (value >= 0) {
        req->zero = 0;
        req->length = value;
        value = gr55xx_usb_ep_queue(ep, req);
        if (value < 0)
        {
            USB_ERROR("ERROR: audio response error %d!\r\n", value);
        }
    }

    return value;
}

/* Interface control*/
static int audio_function_set_alt(struct usb_function *f, unsigned interface_id, unsigned alternate_setting)
{
    if (interface_id == g_audio_hp_as_intf_desc_alt0.bInterfaceNumber) //Headphone Streaming interface
    {
        if(!alternate_setting) //0x00 is zero bandwidth alternate setting. Reference:audio10 Set Interface Request Values
        {
            g_audio.hp_connect_state = 0;
            gr55xx_usb_disable(g_audio.as_out);
            printf("headphone disable !\n");
        }
        else //0x01 is normal isochronous operation
        {
            ring_buffer_clean(&g_headphone_fifo);
            g_audio.hp_connect_state = 1;
            gr55xx_usb_enable(g_audio.as_out, &g_audio_hp_iso_out_desc);
            printf("headphone enable !\n");
        }
    }
    else if (interface_id == g_audio_mic_as_intf_desc_alt0.bInterfaceNumber) // 0x01 0B 00 00 03 00 00 00 //Microphone Streaming interface
    {
        if(!alternate_setting)
        {
            g_audio.mic_connect_state = 0;
            gr55xx_usb_disable(g_audio.as_in);
            printf("micphone disable !\n");
        }
        else
        {
            g_audio.mic_dma_done = 1;
            g_audio.mic_connect_state = 1;
            gr55xx_usb_enable(g_audio.as_in, &g_audio_mic_iso_in_desc);
            printf("micphone enable !\n");
            ring_buffer_clean(&g_headphone_fifo);
        }
    }
    else
    {
        printf("%s  Unsupport  interface_id:%d alternate_setting:%d\n", __FUNCTION__, interface_id, alternate_setting);
    }

    return GR_DRV_USB_OK;
}

static int audio_function_disable(struct usb_function *f)
{
    printf("%s\r\n",__FUNCTION__);
    return GR_DRV_USB_OK;
}

static int audio_config_bind(struct usb_configuration *c)
{
    int ret;

    memset(&g_audio, 0x00, sizeof(struct audio_device));

    INIT_LIST_HEAD(&g_audio.read_pool);
    INIT_LIST_HEAD(&g_audio.write_pool);

    if ((ret = gr55xx_usb_string_id(c->cdev)) < 0)
    {
        goto error;
    }
    g_audio_en_us_strings[AUDIO_STRING_INTERFACE_HP_AC].id = ret;
    g_audio_hp_ac_intf_desc.iInterface = ret;

    if ((ret = gr55xx_usb_string_id(c->cdev)) < 0)
    {
        goto error;
    }
    g_audio_en_us_strings[AUDIO_STRING_INTERFACE_HP_AS].id = ret;
    g_audio_hp_as_intf_desc_alt0.iInterface = ret;
    g_audio_hp_as_intf_desc_alt1.iInterface = ret;

    if ((ret = gr55xx_usb_string_id(c->cdev)) < 0)
    {
        goto error;
    }
    g_audio_en_us_strings[AUDIO_STRING_INTERFACE_MIC_AC].id = ret;
    g_audio_mic_ac_intf_desc.iInterface = ret;

    if ((ret = gr55xx_usb_string_id(c->cdev)) < 0)
    {
        goto error;
    }
    g_audio_en_us_strings[AUDIO_STRING_INTERFACE_MIC_AS].id = ret;
    g_audio_mic_as_intf_desc_alt0.iInterface = ret;
    g_audio_mic_as_intf_desc_alt1.iInterface = ret;

    g_audio_hp_feature_unit_desc.bLength = sizeof(g_audio_hp_feature_unit_desc);

    g_audio_hp_ac_inft_header_desc.baInterfaceNr[0] = 1; // g_audio_hp_as_intf_desc_alt0.bInterfaceNumber,
    g_audio_hp_ac_inft_header_desc.bLength = sizeof(g_audio_hp_ac_inft_header_desc);
    g_audio_hp_ac_inft_header_desc.wTotalLength = sizeof(g_audio_hp_ac_inft_header_desc) + sizeof(g_audio_hp_input_terminal_desc)\
                                                   + sizeof(g_audio_hp_feature_unit_desc) + sizeof(g_audio_hp_output_terminal_desc);

    g_audio_mic_feature_unit_desc.bLength = sizeof(g_audio_mic_feature_unit_desc);

    g_audio_mic_ac_inft_header_desc.baInterfaceNr[0] = 3; // g_audio_mic_as_intf_desc_alt0.bInterfaceNumber
    g_audio_mic_ac_inft_header_desc.bLength = sizeof(g_audio_mic_ac_inft_header_desc);
    g_audio_mic_ac_inft_header_desc.wTotalLength = sizeof(g_audio_mic_ac_inft_header_desc) + sizeof(g_audio_mic_input_terminal_desc)\
                                                   + sizeof(g_audio_mic_feature_unit_desc) + sizeof(g_audio_mic_output_terminal_desc);

    g_audio.func.name           = "Audio Function";
    g_audio.func.strings        = &g_audio_function_strings;
    g_audio.func.bind           = audio_function_bind;
    g_audio.func.setup          = audio_function_setup;
    g_audio.func.disable        = audio_function_disable;
    g_audio.func.set_alt        = audio_function_set_alt;

    ret = gr55xx_usb_add_function(c, &g_audio.func);

error:
    return ret;
}

static struct usb_configuration g_audio_config_driver =
{
    .label                      = "BLE Audio",
    .bind                       = audio_config_bind,
    .bConfigurationValue        = 1,
    .iConfiguration             = 0, /* dynamic */
    .bmAttributes               = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
    .bMaxPower                  = 0xFA,
};

static int audio_bind(struct usb_composite_dev *cdev)
{
    int ret;

    if ((ret = gr55xx_usb_string_id(cdev)) < 0)
    {
        goto error;
    }
    g_audio_strings[STRING_MANUFACTURER_IDX].id = ret;
    g_audio_device_desc.iManufacturer = ret;

    if ((ret = gr55xx_usb_string_id(cdev)) < 0)
    {
        goto error;
    }
    g_audio_strings[STRING_PRODUCT_IDX].id = ret;
    g_audio_device_desc.iProduct = ret;

    if ((ret = gr55xx_usb_string_id(cdev)) < 0)
    {
        goto error;
    }
    g_audio_strings[STRING_DESCRIPTION_IDX].id = ret;
    g_audio_device_desc.iSerialNumber = ret;

    if ((ret = gr55xx_usb_add_config(cdev, &g_audio_config_driver)) < 0)
    {
        goto error;
    }

    USB_LOG("[%s]: Goodix AUDIO Gadget\r\n", __FUNCTION__);

error:
    return ret;
}

static struct usb_composite_driver g_audio_driver =
{
    .name           = "Audio driver",
    .dev            = &g_audio_device_desc,
    .strings        = &g_audio_device_strings,
    .bind           = audio_bind,
    .ep_callback    = audio_ep_callback,
};

__WEAK void user_audio_ep4_callback(void)
{
}

__WEAK void user_audio_ep5_callback(void)
{
}

static int audio_ep4_callback(void)
{
    int retval = GR_DRV_USB_ERROR;
    g_audio.mic_dma_done = 1;
    g_audio.mic_dma_done_cnt++;
    user_audio_ep4_callback();
    return retval;
}

static int audio_ep5_callback(void)
{
    int retval = GR_DRV_USB_ERROR;
#if USB_EP5_DMA_MODE_EN
    g_audio.hp_dma_done = 1;
    g_audio.hp_dma_done_cnt++;
    ring_buffer_clean(&g_headphone_fifo);
    ring_buffer_write(&g_headphone_fifo, g_hp_req->buf, g_hp_req->actual);
#endif
    user_audio_ep5_callback();
    return retval;
}

static int audio_ep_callback(struct usb_gadget *gadget,struct usb_ep *ep)
{
    int retval = GR_DRV_USB_ERROR;
    switch(ep->address)
    {
        case USB_EP4:
            retval =  audio_ep4_callback();
            break;
        case USB_EP5:
            retval = audio_ep5_callback();
            break;
        default:
            USB_ERROR("Unkonw EP %d \r\n", ep->address);
            break;
    }
    return retval;
}

static void audio_mic_complete(struct usb_ep *ep, struct usb_request *req)
{
    USB_LOG("%s \r\n",__FUNCTION__);
}

static void audio_hp_complete(struct usb_ep *ep, struct usb_request *req)
{
    USB_LOG("%s \r\n",__FUNCTION__);
    list_add_tail(&req->list, &g_audio.read_pool);
}


gr55xx_usb_state_t gr55xx_usb_audio_init(usb_handle_t *p_usb)
{
    ring_buffer_init(&g_headphone_fifo, headphone_buffer, DATA_BUFFER_SIZE);
    memset(&g_audio, 0x0, sizeof(g_audio));
    gr55xx_usb_composite_register(&g_audio_driver);
    g_audio.state |= AUDIO_STATUS_ACTIVATED;

    g_mic_req = gr55xx_usb_alloc_request(g_audio.as_in);
    if (!g_mic_req)
    {
        return GR_DRV_USB_ERROR_NOMEM;
    }

    g_mic_req->buf = g_mic_buffer;
#if USB_EP4_DMA_MODE_EN
    g_mic_req->length = AUDIO_MIC_BUFFER_SIZE;
    __HAL_USB_SET_EP4_BURST_SIZE(&g_usb_handle, AUDIO_MIC_FRAME_SIZE);
    ll_usb_set_ep4_xfer_len(USB, AUDIO_MIC_BUFFER_SIZE);
#else
    g_mic_req->length = AUDIO_MIC_FRAME_SIZE;
#endif
    g_mic_req->complete = audio_mic_complete;

    list_add_tail(&g_mic_req->list, &g_audio.write_pool);

    g_hp_req = gr55xx_usb_alloc_request(g_audio.as_out);
    if (!g_hp_req)
    {
        return GR_DRV_USB_ERROR_NOMEM;
    }

    g_hp_req->buf = g_hp_buffer;
#if USB_EP5_DMA_MODE_EN
    g_hp_req->length = AUDIO_HP_BUFFER_SIZE;
    hal_usb_ep_receive_dma(p_usb,HAL_USB_EP5, g_hp_req->buf, g_hp_req->length);
//    ll_usb_disable_ep5_rx_cnt_no_overwrite(USB);
#else
    g_hp_req->length = 6000;
#endif
    g_hp_req->complete = audio_hp_complete;

    list_add_tail(&g_hp_req->list, &g_audio.read_pool);

    mic_data_init();

    return GR_DRV_USB_OK;
}


/* User API */
__WEAK void audio_hp_set_mute(struct usb_ep *ep, struct usb_request *req)
{
    USB_LOG("SPK_SET_MUTE value = %d \r\n", req->buf[0]);
    g_audio.hp_mute = req->buf[0];
    return;
}

__WEAK void audio_mic_set_mute(struct usb_ep *ep, struct usb_request *req)
{
    USB_LOG("MIC_SET_MUTE value = %d \r\n", req->buf[0]);
    g_audio.mic_mute = req->buf[0] + (req->buf[1]>>8);
    return;
}

__WEAK void audio_hp_set_volume(struct usb_ep *ep, struct usb_request *req)
{
    g_audio.hp_volume = req->buf[0] + (req->buf[1]<<8);
    printf("g_audio.hp_volume = %d \r\n",g_audio.hp_volume);
    return;
}

__WEAK void audio_mic_set_volume(struct usb_ep *ep, struct usb_request *req)
{
    g_audio.mic_volume = req->buf[0] + (req->buf[1]<<8);
    printf("g_audio.mic_volume = %d \r\n",g_audio.mic_volume);
    return;
}

#if AUDIO_MIC_DATA_SAMPLE != AUDIO_SAMPLE_44_1K
void gr55xx_audio_tx_rx_loopback_test(void)
{
    g_audio.state |= AUDIO_STATUS_MMAPPED;
    if((g_audio.state & AUDIO_READY_BITS) != AUDIO_READY_BITS)
    {
        return;
    }

    if (g_audio.mic_connect_state)
    {
        if (g_audio.mic_dma_done)
        {
            if (AUDIO_MIC_BUFFER_SIZE <= ring_buffer_items_count_get(&g_headphone_fifo))
            {
                ring_buffer_read(&g_headphone_fifo, g_mic_req->buf , AUDIO_MIC_BUFFER_SIZE);
            }
            gr55xx_usb_ep_queue(g_audio.as_in, g_mic_req);
            g_audio.mic_dma_done = 0;
        }
    }
}
#else
void gr55xx_audio_tx_rx_loopback_test(void)
{
    g_audio.state |= AUDIO_STATUS_MMAPPED;
    if((g_audio.state & AUDIO_READY_BITS) != AUDIO_READY_BITS)
    {
        return;
    }

    if (g_audio.mic_connect_state)
    {
        if (g_audio.mic_dma_done)
        {
            uint32_t mic_cnt_mod10 = (g_audio.mic_dma_done_cnt % 10);
            if(mic_cnt_mod10 == 0)
            {
                if (AUDIO_MIC_BUFFER_SIZE <= ring_buffer_items_count_get(&g_headphone_fifo))
                {
                    ring_buffer_read(&g_headphone_fifo, g_mic_req->buf , AUDIO_MIC_BUFFER_SIZE);
                }
            }

              if(mic_cnt_mod10 <9 )
              {
                __HAL_USB_SET_EP4_BURST_SIZE(&g_usb_handle, AUDIO_MIC_FRAME_SIZE);
                ll_usb_enable_clr_ep4_fifo(USB);
                ll_usb_set_ep4_xfer_len(USB, AUDIO_MIC_FRAME_SIZE);
                ll_usb_set_ep4_ahb_m_rd_start_addr(USB,(uint32_t)g_mic_req->buf + mic_cnt_mod10*AUDIO_MIC_FRAME_SIZE);
                ll_usb_enable_ep4_ahb_m(USB);
              }
              else
              {
                __HAL_USB_SET_EP4_BURST_SIZE(&g_usb_handle, AUDIO_MIC_FRAME_SIZE_MAX);
                ll_usb_enable_clr_ep4_fifo(USB);
                ll_usb_set_ep4_xfer_len(USB, AUDIO_MIC_FRAME_SIZE_MAX);
                ll_usb_set_ep4_ahb_m_rd_start_addr(USB,(uint32_t)g_mic_req->buf + mic_cnt_mod10*AUDIO_MIC_FRAME_SIZE);
                ll_usb_enable_ep4_ahb_m(USB);
              }
            g_audio.mic_dma_done = 0;
        }
    }
}
#endif

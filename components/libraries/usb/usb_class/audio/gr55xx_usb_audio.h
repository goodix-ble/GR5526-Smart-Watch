/*
 * Copyright (C) 2021, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
 */
#ifndef __GR55XX_USB_AUDIO_H__
#define __GR55XX_USB_AUDIO_H__

#include "stdint.h"
#include <stdbool.h>
#include "gr55xx_usb_composite.h"
#include "grx_hal.h"

#include "usb_audio_user_config.h"

//usb audio paramater
#define AUDIO_SAMPLE_8K          8000
#define AUDIO_SAMPLE_16K         16000
#define AUDIO_SAMPLE_44_1K       44100
#define AUDIO_SAMPLE_48K         48000
#define AUDIO_SAMPLE_96K         96000

#define AUDIO_SAMPLE_16BIT       2
#define AUDIO_SAMPLE_24BIT       3
#define AUDIO_SAMPLE_32BIT       4

#if (AUDIO_MIC_DATA_SAMPLE != AUDIO_SAMPLE_44_1K)
    /* Frame size = channel*Sample_rate*sample_bit */
    #define AUDIO_MIC_FRAME_SIZE     ((2*AUDIO_MIC_DATA_SAMPLE*AUDIO_MIC_DATA_BIT)/1000)
    #define AUDIO_MIC_BUFFER_SIZE    (AUDIO_MIC_FRAME_SIZE*10)
    #define AUDIO_MIC_FRAME_SIZE_MAX (AUDIO_MIC_FRAME_SIZE)
#else
    #if (AUDIO_MIC_DATA_SAMPLE == AUDIO_SAMPLE_44_1K)
    #define AUDIO_MIC_FRAME_SIZE     ((2*AUDIO_SAMPLE_44_1K*AUDIO_MIC_DATA_BIT)/1000)
    #define AUDIO_MIC_BUFFER_SIZE    ((2*AUDIO_SAMPLE_44_1K*AUDIO_MIC_DATA_BIT)/100)
    #define AUDIO_MIC_FRAME_SIZE_MAX (AUDIO_MIC_BUFFER_SIZE - (AUDIO_MIC_FRAME_SIZE*9))
    #endif
#endif

#if (AUDIO_HP_DATA_SAMPLE != AUDIO_SAMPLE_44_1K)
    /* Frame size = channel*Sample_rate*sample_bit */
    #define AUDIO_HP_FRAME_SIZE      ((2*AUDIO_HP_DATA_SAMPLE*AUDIO_HP_DATA_BIT)/1000)
    #define AUDIO_HP_BUFFER_SIZE     (AUDIO_HP_FRAME_SIZE*10)
    #define AUDIO_HP_FRAME_SIZE_MAX  (AUDIO_HP_FRAME_SIZE)
#else
    #if (AUDIO_HP_DATA_SAMPLE == AUDIO_SAMPLE_44_1K)
    #define AUDIO_HP_FRAME_SIZE     ((2*AUDIO_SAMPLE_44_1K*AUDIO_HP_DATA_BIT)/1000)
    #define AUDIO_HP_BUFFER_SIZE    ((2*AUDIO_SAMPLE_44_1K*AUDIO_HP_DATA_BIT)/100)
    #define AUDIO_HP_FRAME_SIZE_MAX (AUDIO_HP_BUFFER_SIZE - (AUDIO_HP_FRAME_SIZE*9))
    #endif
#endif

#define AUDIO_HP_IT_ID           1
#define AUDIO_HP_FU_ID           2
#define AUDIO_HP_OT_ID           3

#define AUDIO_MIC_IT_ID          4
#define AUDIO_MIC_FU_ID          5
#define AUDIO_MIC_OT_ID          6

//usb interface
#define USB_AUDIO_INTERFACE_CLASS                               0x01

//usb interface descriptor subclass
#define USB_AUDIO_SUBCLASS_UNDEFINED                            0x00
#define USB_AUDIO_SUBCLASS_AUDIOCONTROL                         0x01
#define USB_AUDIO_SUBCLASS_AUDIOSTREAMING                       0x02
#define USB_AUDIO_SUBCLASS_MIDISTREAMING                        0x03

//usb audio class specific descriptor types
#define USB_AUDIO_SPECIFIC_DESCRIPTOR_UNDEFINED                 0x20
#define USB_AUDIO_SPECIFIC_DESCRIPTOR_DEVICE                    0x21
#define USB_AUDIO_SPECIFIC_DESCRIPTOR_CONFIGURATION             0x22
#define USB_AUDIO_SPECIFIC_DESCRIPTOR_STRING                    0x23
#define USB_AUDIO_SPECIFIC_DESCRIPTOR_INTERFACE                 0x24
#define USB_AUDIO_SPECIFIC_DESCRIPTOR_ENDPOINT                  0x25

//usb audio class control interface subtype
#define USB_AUDIO_AC_INTF_SUBTYPE_UNDEFINED                     0x00
#define USB_AUDIO_AC_INTF_SUBTYPE_HEADER                        0x01
#define USB_AUDIO_AC_INTF_SUBTYPE_INPUT_TERMINAL                0x02
#define USB_AUDIO_AC_INTF_SUBTYPE_OUTPUT_TERNINAL               0x03
#define USB_AUDIO_AC_INTF_SUBTYPE_MIXER_UNIT                    0x04
#define USB_AUDIO_AC_INTF_SUBTYPE_SELECTOR_UNIT                 0x05
#define USB_AUDIO_AC_INTF_SUBTYPE_FEATURE_UNIT                  0x06
#define USB_AUDIO_AC_INTF_SUBTYPE_PROCESSING_UNIT               0x07
#define USB_AUDIO_AC_INTF_SUBTYPE_EXTENSION_UNIT                0x08

//usb audio class streaming interface subtype
#define USB_AUDIO_AS_INTF_SUBTYPE_UNDEFINED                     0x00
#define USB_AUDIO_AS_INTF_SUBTYPE_GENERAL                       0x01
#define USB_AUDIO_AS_INTF_SUBTYPE_FORMAT_TYPE                   0x02
#define USB_AUDIO_AS_INTF_SUBTYPE_FORMAT_SPECIFIC               0x03

//usb audio class streaming class specific endpoint subtypes
#define USB_AUDIO_EP_SUBTYPE_UNDEFINED                          0x00
#define USB_AUDIO_EP_SUBTYPE_GENERAL                            0x01

//usb audio class specific requests
#define USB_AUDIO_REQ_UNDEFINED                                 0x00
#define USB_AUDIO_REQ_SET_CUR                                   0x01
#define USB_AUDIO_REQ_SET_MIN                                   0x02
#define USB_AUDIO_REQ_SET_MAX                                   0x03
#define USB_AUDIO_REQ_SET_RES                                   0x04
#define USB_AUDIO_REQ_SET_MEM                                   0x05
#define USB_AUDIO_REQ_GET_CUR                                   0x81
#define USB_AUDIO_REQ_GET_MIN                                   0x82
#define USB_AUDIO_REQ_GET_MAX                                   0x83
#define USB_AUDIO_REQ_GET_RES                                   0x84
#define USB_AUDIO_REQ_GET_MEM                                   0x85
#define USB_AUDIO_REQ_GET_STAT                                  0xFF

//usb audio class terminal types
#define USB_AUDIO_TERMINAL_USB_UNDEFINED                        0x0100
#define USB_AUDIO_TERMINAL_USB_STREAMING                        0x0101
#define USB_AUDIO_TERMINAL_USB_VENDOR_SPEC                      0x01FF
#define USB_AUDIO_TERMINAL_IN_UNDEFINED                         0x0200
#define USB_AUDIO_TERMINAL_IN_MICROPHONE                        0x0201
#define USB_AUDIO_TERMINAL_IN_DESKTOP_MIC                       0x0202
#define USB_AUDIO_TERMINAL_IN_PERSONAL_MIC                      0x0203
#define USB_AUDIO_TERMINAL_IN_OM_DIR_MIC                        0x0204
#define USB_AUDIO_TERMINAL_IN_MIC_ARRAY                         0x0205
#define USB_AUDIO_TERMINAL_IN_PROC_MIC_ARRAY                    0x0205
#define USB_AUDIO_TERMINAL_OUT_UNDEFINED                        0x0300
#define USB_AUDIO_TERMINAL_OUT_SPEAKER                          0x0301
#define USB_AUDIO_TERMINAL_OUT_HEADPHONES                       0x0302
#define USB_AUDIO_TERMINAL_OUT_HEAD_AUDIO                       0x0303
#define USB_AUDIO_TERMINAL_OUT_DESKTOP_SPEAKER                  0x0304
#define USB_AUDIO_TERMINAL_OUT_ROOM_SPEAKER                     0x0305
#define USB_AUDIO_TERMINAL_OUT_COMM_SPEAKER                     0x0306
#define USB_AUDIO_TERMINAL_OUT_LOW_FREQ_SPEAKER                 0x0307
#define USB_AUDIO_TERMINAL_OUT_IO_UNDEFINED                     0x0400
#define USB_AUDIO_TERMINAL_OUT_IO_HANDSET                       0x0401
#define USB_AUDIO_TERMINAL_OUT_IO_HEADSET                       0x0402
#define USB_AUDIO_TERMINAL_OUT_IO_SPEAKERPHONE_ECHO_NONE        0x0403
#define USB_AUDIO_TERMINAL_OUT_IO_SPEAKERPHONE_ECHO_SUP         0x0404
#define USB_AUDIO_TERMINAL_OUT_IO_SPEAKERPHONE_ECHO_CAN         0x0405

//usb audio class  feature unit control bit map
#define USB_AUDIO_FU_BIT_MUTE                     (1u << 0)
#define USB_AUDIO_FU_BIT_VOLUME                   (1u << 1)
#define USB_AUDIO_FU_BIT_BASS                     (1u << 2)
#define USB_AUDIO_FU_BIT_MID                      (1u << 3)
#define USB_AUDIO_FU_BIT_TREBLE                   (1u << 4)
#define USB_AUDIO_FU_BIT_GRAPH_EQ                 (1u << 5)
#define USB_AUDIO_FU_BIT_AUTO_GAIN                (1u << 6)
#define USB_AUDIO_FU_BIT_DELAY                    (1u << 7)
#define USB_AUDIO_FU_BIT_BASS_BOOST               (1u << 8)
#define USB_AUDIO_FU_BIT_LOUDNESS                 (1u << 9)

//usb audio class  Feature Unit Control Selectors
#define USB_AUDIO_FEATURE_UNIT_CONTROL_UNDEFINED                0x00
#define USB_AUDIO_FEATURE_UNIT_CONTROL_MUTE                     0x01
#define USB_AUDIO_FEATURE_UNIT_CONTROL_VOLUME                   0x02
#define USB_AUDIO_FEATURE_UNIT_CONTROL_BASS                     0x03
#define USB_AUDIO_FEATURE_UNIT_CONTROL_MID                      0x04
#define USB_AUDIO_FEATURE_UNIT_CONTROL_TREBLE                   0x05
#define USB_AUDIO_FEATURE_UNIT_CONTROL_GRAPH_EQ                 0x06
#define USB_AUDIO_FEATURE_UNIT_CONTROL_AUTO_GAIN                0x07
#define USB_AUDIO_FEATURE_UNIT_CONTROL_DELAY                    0x08
#define USB_AUDIO_FEATURE_UNIT_CONTROL_BASS_BOOST               0x09
#define USB_AUDIO_FEATURE_UNIT_CONTROL_LOUDNESS                 0x0A

//usb format tag in audio streaming interface descriptor
#define USB_AUDIO_AS_IFACE_FORMAT_TYPE_I_UNDEFINED              0x0000
#define USB_AUDIO_AS_IFACE_FORMAT_PCM                           0x0001
#define USB_AUDIO_AS_IFACE_FORMAT_PCM8                          0x0002
#define USB_AUDIO_AS_IFACE_FORMAT_IEEE_FLOAT                    0x0003
#define USB_AUDIO_AS_IFACE_FORMAT_ALAW                          0x0004
#define USB_AUDIO_AS_IFACE_FORMAT_MULAW                         0x0005
#define USB_AUDIO_AS_IFACE_FORMAT_TYPE_II_UNDEFINED             0x1000
#define USB_AUDIO_AS_IFACE_FORMAT_MPEG                          0x1001
#define USB_AUDIO_AS_IFACE_FORMAT_AC3                           0x1002
#define USB_AUDIO_AS_IFACE_FORMAT_III_UNDEFINED                 0x2000
#define USB_AUDIO_AS_IFACE_FORMAT_IEC1937_AC_3                  0x2001
#define USB_AUDIO_AS_IFACE_FORMAT_IEC1937_MPEG_1_LAYER1         0x2002
#define USB_AUDIO_AS_IFACE_FORMAT_IEC1937_MPEG_2_NOEXT          0x2003
#define USB_AUDIO_AS_IFACE_FORMAT_IEC1937_MPEG_2_EXT            0x2004
#define USB_AUDIO_AS_IFACE_FORMAT_IEC1937_MPEG_2_LAYER1_LS      0x2005
#define USB_AUDIO_AS_IFACE_FORMAT_IEC1937_MPEG_23_LAYER1_LS     0x2005

#define USB_AUDIO_FRATURE_UINT_CTRL_ARRAY                       6
#define UAB_AUDIO_SAMPLING_FREQ_TABLE_ARRY                      3
#define UAB_AUDIO_INTF_NUM_LIST                                 1


typedef enum _gr_cdc_state
{
    GR_CDC_UART_NONE          = 0x00,
    GR_CDC_UART_INSERT        = 0x01
}gr_cdc_state_e;


//usb audio class control interface header descriptor
struct usb_audio_ac_inft_header_desc{
    uint8_t bLength;             //  Length of the descriptor
    uint8_t bDescriptorType;     //  Descriptor type
    uint8_t bDescriptorSubType;  //  Descriptor subtype
    uint16_t bcdADC;             //  BCD ADC
    uint16_t wTotalLength;       //  Total interfaces length
    uint8_t bInCollection;       //  Input collection
    uint8_t baInterfaceNr[UAB_AUDIO_INTF_NUM_LIST];     //  Interface number list
}__attribute__ ((packed));

//usb audio class class input terminal descriptor
struct usb_audio_input_terminal_desc{
    uint8_t bLength;             //  Length of the descriptor
    uint8_t bDescriptorType;     //  Descriptor type
    uint8_t bDescriptorSubType;  //  Descriptor subtype
    uint8_t bTerminalID;         //  Terminal ID
    uint16_t wTerminalType;      //  Terminal type
    uint8_t bAssocTerminal;      //  Association terminal
    uint8_t bNrChannels;         //  Number of channels
    uint16_t wChannelConfig;     //  Channel config
    uint8_t iChannelNames;       //  Channel names
    uint8_t iTerminal;           //  Terminal string ID
}__attribute__ ((packed));

//usb audio class output terminal descriptor
struct usb_audio_output_terminal_desc{
    uint8_t bLength;            //  Length of the descriptor
    uint8_t bDescriptorType;    //  Descriptor type
    uint8_t bDescriptorSubType; //  Descriptor subtype
    uint8_t bTerminalID;        //  Terminal ID
    uint16_t wTerminalType;     //  Terminal type
    uint8_t bAssocTerminal;     //  Association terminal
    uint8_t bSourceID;          //  Source ID
    uint8_t iTerminal;          //  Terminal string ID
}__attribute__ ((packed));

//usb audio class feature unit descriptor
struct usb_audio_feature_unit_desc{
    uint8_t bLength;                //  Length of the descriptor
    uint8_t bDescriptorType;        //  Descriptor type
    uint8_t bDescriptorSubType;     //  Descriptor subtype
    uint8_t bUnitID;                //  Unit ID
    uint8_t bSourceID;              //  Source ID, ID of the Unit or Terminal to which this Feature Unit is connected
    uint8_t bControlSize;           //  Size in bytes of an element of the bmaControls() array: n
    uint8_t bmaControls[USB_AUDIO_FRATURE_UINT_CTRL_ARRAY];  //  Controls array. Controls0 is master channel 0 (always present) and Controls1 is logical channel 1.
}__attribute__ ((packed));

//usb audio class audio streaming interface descriptor
struct usb_audio_as_intf_desc{
    uint8_t bLength;                //  Length of the descriptor
    uint8_t bDescriptorType;        //  Descriptor type
    uint8_t bDescriptorSubType;     //  Descriptor subtype
    uint8_t bTerminalLink;          //  Terminal link
    uint8_t bDelay;                 //  Delay
    uint16_t wFormatTag;            //  Format TAG
}__attribute__ ((packed));


//usb audio streaming format type I descriptor
struct usb_audio_as_format_type_I_desc{
    uint8_t bLength;                //  Length of the descriptor
    uint8_t bDescriptorType;        //  Descriptor type
    uint8_t bDescriptorSubType;     //  Descriptor subtype
    uint8_t bFormatType;            //  Format type: fixed value 1
    uint8_t bNrChannels;            //  Number of channels
    uint8_t bSubframeSize;          //  Subframe size
    uint8_t bBitResolution;         //  Bit resolution
    uint8_t bSamFreqType;           //  Number of supported sampling frequencies
    uint8_t tSamFreq[UAB_AUDIO_SAMPLING_FREQ_TABLE_ARRY];             //  Number of supported sampling frequencies table (24 bit entries)
}__attribute__ ((packed));

//usb audio format type II descriptor
struct usb_audio_as_format_type_II_desc{
    uint8_t bLength;                //  Length of the descriptor
    uint8_t bDescriptorType;        //  Descriptor type @ref APP_USBD_AUDIO_DESCRIPTOR_INTERFACE
    uint8_t bDescriptorSubType;     //  Descriptor subtype @ref app_usbd_audio_as_iface_subtype_t
    uint8_t bFormatType;            //  Format type: fixed value 2
    uint16_t wMaxBitRate;           //  Maximum bitrate
    uint16_t wSamplesPerFrame;      //  Samples per frame
    uint8_t bSamFreqType;           //  Number of supported sampling frequencies
    uint8_t tSamFreq[UAB_AUDIO_SAMPLING_FREQ_TABLE_ARRY];             //  Number of supported sampling frequencies table (24 bit entries)
}__attribute__ ((packed));

//usb audio format type III descriptor
struct usb_audio_as_format_type_III_desc{
    uint8_t bLength;                //  Length of the descriptor
    uint8_t bDescriptorType;        //  Descriptor type
    uint8_t bDescriptorSubType;     //  Descriptor subtype
    uint8_t bFormatType;            //  Format type: fixed value 3
    uint8_t bNrChannels;            //  Number of channels
    uint8_t bSubframeSize;          //  Subframe size
    uint8_t bBitResolution;         //  Bit resolution
    uint8_t bSamFreqType;           //  Number of supported sampling frequencies
    uint8_t tSamFreq[UAB_AUDIO_SAMPLING_FREQ_TABLE_ARRY];             //  Number of supported sampling frequencies table (24 bit entries)
}__attribute__ ((packed));

//usb audio streaming endpoint descriptor
struct  usb_audio_as_endpoint_desc{
    uint8_t bLength;                //  Length of the descriptor
    uint8_t bDescriptorType;        //  Descriptor type
    uint8_t bDescriptorSubType;     //  Descriptor subtype
    uint8_t bmAttributes;           //  Audio endpoint attributes
    uint8_t bLockDelayUnits;        //  Lock delay units
    uint16_t wLockDelay;            //  Lock delay value
}__attribute__ ((packed));


struct audio_device
{
    struct usb_function func;

    struct usb_ep       *notify;
    struct usb_request  *notify_req;

    struct usb_ep       *as_in;
    struct usb_ep       *as_out;

    struct list_head    write_pool;
    struct list_head    read_pool;

    uint8_t  mic_mute;
    uint8_t  hp_mute;
    uint16_t mic_volume;
    uint16_t hp_volume;
    uint8_t  mic_connect_state;
    uint8_t  hp_connect_state;
    uint8_t  mic_dma_done;
    uint8_t  hp_dma_done;

    uint32_t  mic_dma_done_cnt;
    uint32_t  hp_dma_done_cnt;

    uint32_t state;
};

#ifdef __cplusplus
extern "C"
{
#endif

//user api
void audio_hp_set_mute(struct usb_ep *ep, struct usb_request *req);
void audio_mic_set_mute(struct usb_ep *ep, struct usb_request *req);
void audio_hp_set_volume(struct usb_ep *ep, struct usb_request *req);
void audio_mic_set_volume(struct usb_ep *ep, struct usb_request *req);
//platfrom api
gr55xx_usb_state_t gr55xx_usb_audio_init(usb_handle_t *p_usb);


#ifdef __cplusplus
}
#endif


#endif   /*__GR55XX_USB_AUDIO_H__*/


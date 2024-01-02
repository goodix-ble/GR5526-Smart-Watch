/*
 * Copyright (C) 2021, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
 */
#ifndef __GR55XX_USB_CDC_H__
#define __GR55XX_USB_CDC_H__

#include "stdint.h"
#include <stdbool.h>
#include "gr55xx_usb_composite.h"
#include "grx_hal.h"

#define USB_CDC_BULK_IN_EP      USB_EP2   //USB_EP2 or USB_EP3
#define USB_CDC_BULK_OUT_EP     USB_EP1   //only USB_EP1

#define USB_CDC_CLASS_COMM              0x02
#define USB_CDC_CLASS_DATA              0x0A

#define USB_CDC_SUBCLASS_DLCM           0x01
#define USB_CDC_SUBCLASS_ACM            0x02
#define USB_CDC_SUBCLASS_TCM            0x03
#define USB_CDC_SUBCLASS_MCCM           0x04
#define USB_CDC_SUBCLASS_CCM            0x05
#define USB_CDC_SUBCLASS_ETH            0x06
#define USB_CDC_SUBCLASS_ATM            0x07

#define USB_CDC_PROTOCOL_V25TER         0x01

#define USB_CDC_PROTOCOL_I430           0x30
#define USB_CDC_PROTOCOL_HDLC           0x31
#define USB_CDC_PROTOCOL_TRANS          0x32
#define USB_CDC_PROTOCOL_Q921M          0x50
#define USB_CDC_PROTOCOL_Q921           0x51
#define USB_CDC_PROTOCOL_Q921TM         0x52
#define USB_CDC_PROTOCOL_V42BIS         0x90
#define USB_CDC_PROTOCOL_Q931           0x91
#define USB_CDC_PROTOCOL_V120           0x92
#define USB_CDC_PROTOCOL_CAPI20         0x93
#define USB_CDC_PROTOCOL_HOST           0xFD
#define USB_CDC_PROTOCOL_PUFD           0xFE
#define USB_CDC_PROTOCOL_VENDOR         0xFF

#define USB_CDC_CS_INTERFACE            0x24
#define USB_CDC_CS_ENDPOINT             0x25

#define USB_CDC_SCS_HEADER              0x00
#define USB_CDC_SCS_CALL_MGMT           0x01
#define USB_CDC_SCS_ACM                 0x02
#define USB_CDC_SCS_UNION               0x06

#define CDC_SEND_ENCAPSULATED_COMMAND   0x00
#define CDC_GET_ENCAPSULATED_RESPONSE   0x01
#define CDC_SET_COMM_FEATURE            0x02
#define CDC_GET_COMM_FEATURE            0x03
#define CDC_CLEAR_COMM_FEATURE          0x04
#define CDC_SET_AUX_LINE_STATE          0x10
#define CDC_SET_HOOK_STATE              0x11
#define CDC_PULSE_SETUP                 0x12
#define CDC_SEND_PULSE                  0x13
#define CDC_SET_PULSE_TIME              0x14
#define CDC_RING_AUX_JACK               0x15
#define CDC_SET_LINE_CODING             0x20
#define CDC_GET_LINE_CODING             0x21
#define CDC_SET_CONTROL_LINE_STATE      0x22
#define CDC_SEND_BREAK                  0x23
#define CDC_SET_RINGER_PARMS            0x30
#define CDC_GET_RINGER_PARMS            0x31
#define CDC_SET_OPERATION_PARMS         0x32
#define CDC_GET_OPERATION_PARMS         0x33
#define CDC_SET_LINE_PARMS              0x34
#define CDC_GET_LINE_PARMS              0x35
#define CDC_DIAL_DIGITS                 0x36
#define CDC_SET_UNIT_PARAMETER          0x37
#define CDC_GET_UNIT_PARAMETER          0x38
#define CDC_CLEAR_UNIT_PARAMETER        0x39
#define CDC_GET_PROFILE                 0x3A
#define CDC_SET_ETH_MULTICAST_FILTERS   0x40
#define CDC_SET_ETH_POWER_MGMT_FILT     0x41
#define CDC_GET_ETH_POWER_MGMT_FILT     0x42
#define CDC_SET_ETH_PACKET_FILTER       0x43
#define CDC_GET_ETH_STATISTIC           0x44
#define CDC_SET_ATM_DATA_FORMAT         0x50
#define CDC_GET_ATM_DEVICE_STATISTICS   0x51
#define CDC_SET_ATM_DEFAULT_VC          0x52
#define CDC_GET_ATM_VC_STATISTICS       0x53

struct gr_io_ctx_t
{
    uint8_t  *buf;
    uint32_t length;
} __attribute__ ((packed));

typedef enum _gr_ep_dir
{
    GR_EP_DIR_IN          = 0x00,
    GR_EP_DIR_OUT         = 0x01
}gr_ep_dir_e;

typedef enum _gr_cdc_state
{
    GR_CDC_UART_NONE          = 0x00,
    GR_CDC_UART_INSERT        = 0x01
}gr_cdc_state_e;

#define CDC_STATUS_NONE                 0x00000000
#define CDC_STATUS_CONNECTED            0x00000001
#define CDC_STATUS_ACTIVATED            0x00000002
#define CDC_STATUS_MMAPPED              0x00000004

#define CDC_READY_BITS                  (CDC_STATUS_CONNECTED | CDC_STATUS_ACTIVATED | CDC_STATUS_MMAPPED)

#define CDC_STRING_INTERFACE_COMM       0
#define CDC_STRING_INTERFACE_DATA       1

#define CDC_BUFFER_SIZE                 64
#define CDC_IO_POOL_SIZE                4

#define RX_BUF_SIZE        64
#if USB_CDC_BULK_IN_EP == USB_EP2
#define TX_BUF_SIZE        63
#else
#define TX_BUF_SIZE        64
#endif

#define CDC_VENDOR_ID                   0x27C6
#define CDC_PRODUCT_ID                  0x2000
#define CDC_DEVICE_BCD                  0x0100  /* 1.00 */


struct cdc_header_descriptor
{
    uint8_t length;
    uint8_t type;
    uint8_t subtype;
    uint16_t bcd;
} __attribute__ ((packed));

struct cdc_acm_descriptor
{
    uint8_t length;
    uint8_t type;
    uint8_t subtype;
    uint8_t capabilties;
} __attribute__ ((packed));

struct cdc_call_mgmt_descriptor
{
    uint8_t length;
    uint8_t type;
    uint8_t subtype;
    uint8_t capabilties;
    uint8_t data_interface;
} __attribute__ ((packed));

struct cdc_union_descriptor
{
    uint8_t length;
    uint8_t type;
    uint8_t subtype;
    uint8_t master_interface;
    uint8_t slave_interface0;
} __attribute__ ((packed));


/* Line Coding Structure from CDC spec 6.2.13 */
struct usb_cdc_line_coding {
    uint32_t    dwDTERate;
    uint8_t bCharFormat;
#define USB_CDC_1_STOP_BITS         0
#define USB_CDC_1_5_STOP_BITS       1
#define USB_CDC_2_STOP_BITS         2

    uint8_t bParityType;
#define USB_CDC_NO_PARITY           0
#define USB_CDC_ODD_PARITY          1
#define USB_CDC_EVEN_PARITY         2
#define USB_CDC_MARK_PARITY         3
#define USB_CDC_SPACE_PARITY        4

    uint8_t bDataBits;
} __attribute__ ((packed));



#ifdef __cplusplus
extern "C"
{
#endif

//user api
gr_cdc_state_e gr55xx_usb_cdc_state(void);
void gr55xx_usb_cdc_read_data(uint8_t *p_data, uint32_t size);
void gr55xx_usb_cdc_send_data(uint8_t *p_data, uint32_t size);

//platfrom api
gr55xx_usb_state_t gr55xx_usb_cdc_init(void);


#ifdef __cplusplus
}
#endif


#endif   /*__CDC_H__*/


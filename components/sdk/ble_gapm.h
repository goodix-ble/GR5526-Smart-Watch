/**
 ****************************************************************************************
 *
 * @file ble_gapm.h
 *
 * @brief BLE GAPM API
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

   /**
 * @addtogroup BLE
 * @{
 * @brief Definitions and prototypes for the BLE SDK interface.
 */
 
  /**
 * @addtogroup BLE_GAP Generic Access Profile (GAP)
 * @{
 * @brief Definitions and prototypes for the GAP interface.
 */

#ifndef __BLE_GAPM_H__
#define __BLE_GAPM_H__
 
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "ble_error.h"
#include "ble_gapc.h"
#include <stdint.h>
#include <stddef.h>


/**
 @addtogroup BLE_GAPM Generic Access Profile (GAP) Management
 @{
 @brief Definitions and prototypes for the GAP Management interface.
 */
 
/** @addtogroup BLE_GAPM_DEFINES Defines
 * @{ */

#define CO_BIT(pos)(1UL<<(pos))                         /**< Bit operation helper. */ 
#define BLE_GAP_MAX_ADV_NUM              5              /**< Maximal advertising instance number. */
#define BLE_GAP_MAX_PER_SYNC_NUM         5              /**< Maximal periodic syncronization instance number. */
#define BLE_GAP_MAX_WL_NUM               10             /**< Maximal white list number. */
#define BLE_GAP_MAX_PRD_ADV_NUM          4              /**< Maximal periodic advertising list number. */
#define BLE_GAP_MAX_KEY_LEN              16             /**< The key length. */
#define BLE_GAP_INVALID_ADV_IDX          0xFF           /**< Invalid adv index. */
#define BLE_GAP_DEVNAME_DEFAULT          "GOODIX_BLE"   /**< Default device name value. */
#define BLE_GAP_DEVNAME_MAX_LEN          248            /**< Maximal length of device name. */

#define BLE_GAP_MAX_ANTENNA_ID_NUM       0x4B           /**< Maximal number of Antenna IDs. */

#define BLE_APPEARANCE_UNKNOWN                                0     /**< Unknown. */
#define BLE_APPEARANCE_GENERIC_PHONE                         64     /**< Generic Phone. */
#define BLE_APPEARANCE_GENERIC_COMPUTER                     128     /**< Generic Computer. */
#define BLE_APPEARANCE_GENERIC_WATCH                        192     /**< Generic Watch. */
#define BLE_APPEARANCE_WATCH_SPORTS_WATCH                   193     /**< Watch: Sports Watch. */
#define BLE_APPEARANCE_GENERIC_CLOCK                        256     /**< Generic Clock. */
#define BLE_APPEARANCE_GENERIC_DISPLAY                      320     /**< Generic Display. */
#define BLE_APPEARANCE_GENERIC_REMOTE_CONTROL               384     /**< Generic Remote Control. */
#define BLE_APPEARANCE_GENERIC_EYE_GLASSES                  448     /**< Generic Eye-glasses. */
#define BLE_APPEARANCE_GENERIC_TAG                          512     /**< Generic Tag. */
#define BLE_APPEARANCE_GENERIC_KEYRING                      576     /**< Generic Keyring. */
#define BLE_APPEARANCE_GENERIC_MEDIA_PLAYER                 640     /**< Generic Media Player. */
#define BLE_APPEARANCE_GENERIC_BARCODE_SCANNER              704     /**< Generic Barcode Scanner. */
#define BLE_APPEARANCE_GENERIC_THERMOMETER                  768     /**< Generic Thermometer. */
#define BLE_APPEARANCE_THERMOMETER_EAR                      769     /**< Thermometer: Ear. */
#define BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR            832     /**< Generic Heart Rate Sensor. */
#define BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT    833     /**< Heart Rate Sensor: Heart Rate Belt. */
#define BLE_APPEARANCE_GENERIC_BLOOD_PRESSURE               896     /**< Generic Blood Pressure. */
#define BLE_APPEARANCE_BLOOD_PRESSURE_ARM                   897     /**< Blood Pressure: Arm. */
#define BLE_APPEARANCE_BLOOD_PRESSURE_WRIST                 898     /**< Blood Pressure: Wrist. */
#define BLE_APPEARANCE_GENERIC_HID                          960     /**< Human Interface Device (HID). */
#define BLE_APPEARANCE_HID_KEYBOARD                         961     /**< Keyboard (HID Subtype). */
#define BLE_APPEARANCE_HID_MOUSE                            962     /**< Mouse (HID Subtype). */
#define BLE_APPEARANCE_HID_JOYSTICK                         963     /**< Joystick (HID Subtype). */
#define BLE_APPEARANCE_HID_GAMEPAD                          964     /**< Gamepad (HID Subtype). */
#define BLE_APPEARANCE_HID_DIGITIZERSUBTYPE                 965     /**< Digitizer Tablet (HID Subtype). */
#define BLE_APPEARANCE_HID_CARD_READER                      966     /**< Card Reader (HID Subtype). */
#define BLE_APPEARANCE_HID_DIGITAL_PEN                      967     /**< Digital Pen (HID Subtype). */
#define BLE_APPEARANCE_HID_BARCODE                          968     /**< Barcode Scanner (HID Subtype). */
#define BLE_APPEARANCE_GENERIC_GLUCOSE_METER               1024     /**< Generic Glucose Meter. */
#define BLE_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR      1088     /**< Generic Running Walking Sensor. */
#define BLE_APPEARANCE_RUNNING_WALKING_SENSOR_IN_SHOE      1089     /**< Running Walking Sensor: In-Shoe. */
#define BLE_APPEARANCE_RUNNING_WALKING_SENSOR_ON_SHOE      1090     /**< Running Walking Sensor: On-Shoe. */
#define BLE_APPEARANCE_RUNNING_WALKING_SENSOR_ON_HIP       1091     /**< Running Walking Sensor: On-Hip. */
#define BLE_APPEARANCE_GENERIC_CYCLING                     1152     /**< Generic Cycling. */
#define BLE_APPEARANCE_CYCLING_CYCLING_COMPUTER            1153     /**< Cycling: Cycling Computer. */
#define BLE_APPEARANCE_CYCLING_SPEED_SENSOR                1154     /**< Cycling: Speed Sensor. */
#define BLE_APPEARANCE_CYCLING_CADENCE_SENSOR              1155     /**< Cycling: Cadence Sensor. */
#define BLE_APPEARANCE_CYCLING_POWER_SENSOR                1156     /**< Cycling: Power Sensor. */
#define BLE_APPEARANCE_CYCLING_SPEED_CADENCE_SENSOR        1157     /**< Cycling: Speed and Cadence Sensor. */
#define BLE_APPEARANCE_GENERIC_PULSE_OXIMETER              3136     /**< Generic Pulse Oximeter. */
#define BLE_APPEARANCE_PULSE_OXIMETER_FINGERTIP            3137     /**< Fingertip (Pulse Oximeter subtype). */
#define BLE_APPEARANCE_PULSE_OXIMETER_WRIST_WORN           3138     /**< Wrist Worn(Pulse Oximeter subtype). */
#define BLE_APPEARANCE_GENERIC_WEIGHT_SCALE                3200     /**< Generic Weight Scale. */
#define BLE_APPEARANCE_GENERIC_OUTDOOR_SPORTS_ACT          5184     /**< Generic Outdoor Sports Activity. */
#define BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_DISP         5185     /**< Location Display Device (Outdoor Sports Activity subtype). */
#define BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_AND_NAV_DISP 5186     /**< Location and Navigation Display Device (Outdoor Sports Activity subtype). */
#define BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_POD          5187     /**< Location Pod (Outdoor Sports Activity subtype). */
#define BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_AND_NAV_POD  5188     /**< Location and Navigation Pod (Outdoor Sports Activity subtype). */

/**@defgroup BLE_GAP_PHYS GAP PHYs (bitmask)
 * @{ */
#define BLE_GAP_PHY_ANY       0x00     /**< No preferred PHY. */
#define BLE_GAP_PHY_LE_1MBPS  (1 << 0) /**< LE 1M PHY preferred for an active link. */
#define BLE_GAP_PHY_LE_2MBPS  (1 << 1) /**< LE 2M PHY preferred for an active link. */
#define BLE_GAP_PHY_LE_CODED  (1 << 2) /**< LE Coded PHY preferred for an active link. */
/**@} */


/**@defgroup BLE_GAP_ADV_CHANNEL GAP ADV CHANNEL (bitmask)
 * @{ */
#define BLE_GAP_ADV_CHANNEL_37              0x01 /**< Advertising Channel 37 (2402MHz). */ 
#define BLE_GAP_ADV_CHANNEL_38              0x02 /**< Advertising Channel 38 (2426MHz). */
#define BLE_GAP_ADV_CHANNEL_39              0x04 /**< Advertising Channel 39 (2480MHz). */
#define BLE_GAP_ADV_CHANNEL_37_38_39        0x07 /**< Advertising Channel 37, 38, 39. */
/**@} */
/**@} */

/** @addtogroup BLE_GAPM_ENUMERATIONS Enumerations
 * @{ */
/**
 * @brief GAP role options
 */
typedef enum
{
    BLE_GAP_ROLE_NONE        = 0x00,                                             /**< No role set yet. */
    BLE_GAP_ROLE_OBSERVER    = 0x01,                                             /**< Observer role. */
    BLE_GAP_ROLE_BROADCASTER = 0x02,                                             /**< Broadcaster role. */
    BLE_GAP_ROLE_CENTRAL     = (0x04 | BLE_GAP_ROLE_OBSERVER),                   /**< Master/Central role. */
    BLE_GAP_ROLE_PERIPHERAL  = (0x08 | BLE_GAP_ROLE_BROADCASTER),                /**< Peripheral/Slave role. */
    BLE_GAP_ROLE_ALL         = (BLE_GAP_ROLE_CENTRAL | BLE_GAP_ROLE_PERIPHERAL), /**< Device has all roles, both peripheral and central. */
} ble_gap_role_t;

/**
 * @brief Own BD address source of the device
 */
typedef enum
{
   BLE_GAP_OWN_ADDR_STATIC = 0,   /**< Public or Private Static Address according to device address configuration. */
   BLE_GAP_OWN_ADDR_GEN_RSLV,     /**< Generated resolvable private random address. */
   BLE_GAP_OWN_ADDR_GEN_NON_RSLV, /**< Generated non-resolvable private random address. */
} ble_gap_own_addr_t;

/**
 * @brief Write permissions of the device name characteristic
 */
typedef enum
{
   BLE_GAP_WRITE_PERM_DISABLE = 0,   /**< Disable write access. */
   BLE_GAP_WRITE_PERM_NOAUTH,        /**< LE Security Mode 1, Level 1. Link does not need to be encrypted or authenticated. */
   BLE_GAP_WRITE_PERM_UNAUTH,        /**< LE Security Mode 1, Level 2. Link needs to be encrypted, but not to be authenticated. */
   BLE_GAP_WRITE_PERM_AUTH,          /**< LE Security Mode 1, Level 3. Link needs to be encrypted and authenticated (MITM). */
   BLE_GAP_WRITE_PERM_SEC_CON,       /**< LE Security Mode 1, Level 4. Link needs to be encrypted and authenticated (secure connections). */
} ble_gap_dev_name_write_perm_t;

/**
 * @brief Advertising data type
 */
typedef enum
{
   BLE_GAP_ADV_DATA_TYPE_DATA = 0, /**< Advertising data. */
   BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, /**< Scan response data. */
   BLE_GAP_ADV_DATA_TYPE_PER_DATA, /**< Periodic advertising data. */
} ble_gap_adv_data_type_t;


/**@brief Get device parameters operation code. */
typedef enum
{
    BLE_GAP_GET_DEV_VERSION = 0,            /**< Get version information for the local Controller. */
    BLE_GAP_GET_DEV_BDADDR,                 /**< Get local device BD Address. */
    //BLE_GAP_GET_DEV_ADV_TX_POWER,           /**< Get the transmit power level used for LE advertising channel packets. */
    BLE_GAP_GET_SUGGESTED_DFLT_LE_DATA_LEN, /**< Get suggested values (SuggestedMaxTxOctets and SuggestedMaxTxTime)
                                                 for the Controller's maximum transmitted number of payload octets
                                                 and maximum packet transmission time to be used for new connections. */
    BLE_GAP_GET_MAX_LE_DATA_LEN,            /**< Get the Controller' maximum supported payload octets and packet duration
                                                 times for transmission and reception (supportedMaxTxOctets and supportedMaxTxTime,
                                                 supportedMaxRxOctets, and supportedMaxRxTime). */
    BLE_GAP_GET_NB_ADV_SETS,                /**< Read the maximum number of advertising sets currently supported by the controller. */
    BLE_GAP_GET_MAX_LE_ADV_DATA_LEN,        /**< Get maximum data length for advertising data. */
    BLE_GAP_GET_DEV_TX_POWER,               /**< Read the minimum and maximum transmit powers supported by the Controller. */
    BLE_GAP_GET_DEV_RF_RF_PATH_COMP,        /**< Get RF path compensation values. */
    BLE_GAP_GET_ANTENNA_INFO,               /**< Get antenna information. */
} ble_gap_dev_info_get_type_t;

/**@brief GAP advertising modes. */
typedef enum
{
    BLE_GAP_ADV_TYPE_ADV_IND = 0,         /**< Undirected connectable and scannable mode. */
    BLE_GAP_ADV_TYPE_ADV_NONCONN_IND,     /**< Non-connectable and non-scanable mode. */
    BLE_GAP_ADV_TYPE_ADV_SCAN_IND,        /**< Undirected scannable mode. */
    BLE_GAP_ADV_TYPE_ADV_HIGH_DIRECT_IND, /**< Directed high duty cycle mode. */
    BLE_GAP_ADV_TYPE_ADV_LOW_DIRECT_IND,  /**< Directed low duty cycle mode. */
} ble_gap_adv_mode_t;

/**@brief GAP discoverability modes. */
typedef enum
{
    BLE_GAP_DISC_MODE_NON_DISCOVERABLE = 0, /**< Non-discoverable mode. */
    BLE_GAP_DISC_MODE_GEN_DISCOVERABLE,     /**< General-discoverable mode. */
    BLE_GAP_DISC_MODE_LIM_DISCOVERABLE,     /**< Limited-discoverable mode. */
    BLE_GAP_DISC_MODE_BROADCASTER,          /**< Broadcaster mode. */
} ble_gap_disc_mode_t;

/**
 * @brief Advertising filter policy
 */
typedef enum
{
    BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY    = 0, /**< Allow both scan and connection requests from anyone. */
    BLE_GAP_ADV_ALLOW_SCAN_WLST_CON_ANY,       /**< Allow scan req from white-list devices only and connection req from anyone. */
    BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_WLST,       /**< Allow scan req from anyone and connection req from white-list devices only. */
    BLE_GAP_ADV_ALLOW_SCAN_WLST_CON_WLST,      /**< Allow scan and connection requests from white-list devices only. */
} ble_gap_adv_filter_policy_t;

/**
 * @brief Specify what PHY the Controller has changed for TX/RX. HCI:7.7.65.12
 */
typedef enum
{
    BLE_GAP_PHY_UNDEF_VALUE = 0, /**< Undefined LE PHY. */
    BLE_GAP_PHY_1MBPS_VALUE = 1, /**< LE 1M PHY. */
    BLE_GAP_PHY_2MBPS_VALUE = 2, /**< LE 2M PHY. */
    BLE_GAP_PHY_CODED_VALUE = 3, /**< LE Coded PHY. */
} ble_gap_le_phy_value_t;

/**
 * @brief Advertising type
 */
typedef enum
{
    BLE_GAP_ADV_TYPE_LEGACY = 0, /**< Legacy advertising. */
    BLE_GAP_ADV_TYPE_EXTENDED,   /**< Extended advertising. */
    BLE_GAP_ADV_TYPE_PERIODIC,   /**< Periodic advertising. */
} ble_gap_adv_type_t;

/**
 * @brief Advertising properties bit field and bit positions
 */
typedef enum
{
    BLE_GAP_ADV_PROP_CONNECTABLE_POS = 0, /**< Indicate that advertising is connectable, reception of CONNECT_REQ or 
                                               UX_CONNECT_REQ PDUs is accepted. Not applicable for periodic advertising. */
    BLE_GAP_ADV_PROP_SCANNABLE_POS,       /**< Indicate that advertising is scannable, reception of SCAN_REQ or AUX_SCAN_REQ PDUs is accepted. */
    BLE_GAP_ADV_PROP_DIRECTED_POS,        /**< Indicate that advertising targets at a specific device. Only applicable in following cases:
                                               - Legacy advertising: if connectable
                                               - Extended advertising: connectable or (non-connectable and non-discoverable). */
    BLE_GAP_ADV_PROP_HDC_POS,             /**< Indicate that High Duty Cycle has to be used for advertising on primary channel,
                                               applicable only if created advertising is not an extended advertising. */
    BLE_GAP_ADV_PROP_USE_LEGACY_PDUS_POS, /**< Use legacy advertising PDUs. */
    BLE_GAP_ADV_PROP_ANONYMOUS_POS,       /**< Enable anonymous mode. Device address will not appear in sending PDUs.
                                               Valid only if the created advertising is an extended advertising. */
    BLE_GAP_ADV_PROP_TX_PWR_POS,          /**< Include TX power in the extended header of the advertising PDU.
                                               Valid only if the created advertising is not a legacy advertising. */
    BLE_GAP_ADV_PROP_PER_TX_PWR_POS,      /**< Include TX power in the periodic advertising PDU.
                                               Valid only if the created advertising is a periodic advertising. */
    BLE_GAP_ADV_PROP_SCAN_REQ_NTF_EN_POS, /**< Indicate if application must be informed about receiving scan request PDUs. */
} ble_gap_adv_prop_pos_t;

/**
 * @brief Advertising properties bit field bit value
 */
typedef enum
{
    BLE_GAP_ADV_PROP_CONNECTABLE_BIT     = CO_BIT(BLE_GAP_ADV_PROP_CONNECTABLE_POS),
    BLE_GAP_ADV_PROP_SCANNABLE_BIT       = CO_BIT(BLE_GAP_ADV_PROP_SCANNABLE_POS),
    BLE_GAP_ADV_PROP_DIRECTED_BIT        = CO_BIT(BLE_GAP_ADV_PROP_DIRECTED_POS),
    BLE_GAP_ADV_PROP_HDC_BIT             = CO_BIT(BLE_GAP_ADV_PROP_HDC_POS),
    BLE_GAP_ADV_PROP_USE_LEGACY_PDUS_BIT = CO_BIT(BLE_GAP_ADV_PROP_USE_LEGACY_PDUS_POS),
    BLE_GAP_ADV_PROP_ANONYMOUS_BIT       = CO_BIT(BLE_GAP_ADV_PROP_ANONYMOUS_POS),
    BLE_GAP_ADV_PROP_TX_PWR_BIT          = CO_BIT(BLE_GAP_ADV_PROP_TX_PWR_POS),
    BLE_GAP_ADV_PROP_PER_TX_PWR_BIT      = CO_BIT(BLE_GAP_ADV_PROP_PER_TX_PWR_POS),
    BLE_GAP_ADV_PROP_SCAN_REQ_NTF_EN_BIT = CO_BIT(BLE_GAP_ADV_PROP_SCAN_REQ_NTF_EN_POS),
} ble_gap_adv_prop_t;

/**
 * @brief Scanning types
 */
typedef enum
{
    BLE_GAP_SCAN_ACTIVE = 0, /**< Active scan type. */
    BLE_GAP_SCAN_PASSIVE,    /**< Passive scan type. */
} ble_gap_scan_type_t;

/**
 * @brief Scanning modes
 */
typedef enum
{
    BLE_GAP_SCAN_GEN_DISC_MODE = 0, /**< General discoverable mode. */
    BLE_GAP_SCAN_LIM_DISC_MODE,     /**< Limited discoverable mode. */
    BLE_GAP_SCAN_OBSERVER_MODE,     /**< Observer mode. */
} ble_gap_scan_mode_t;

/**
 * @brief Duplicate filter policy
 */
typedef enum
{
    BLE_GAP_SCAN_FILT_DUPLIC_DIS  = 0, /**< Disable filtering of duplicate packets. */
    BLE_GAP_SCAN_FILT_DUPLIC_EN,       /**< Enable filtering of duplicate packets. */
} ble_gap_scan_dup_filt_policy_t;

/**
 * @brief Extended scanning types
 */
typedef enum
{
    BLE_GAP_EXT_SCAN_TYPE_GEN_DISC = 0,  /**< General discovery. */
    BLE_GAP_EXT_SCAN_TYPE_LIM_DISC,      /**< Limited discovery. */
    BLE_GAP_EXT_SCAN_TYPE_OBSERVER,      /**< Observer. */
    BLE_GAP_EXT_SCAN_TYPE_SEL_OBSERVER,  /**< Selective observer. */
    BLE_GAP_EXT_SCAN_TYPE_CONN_DISC,     /**< Connectable discovery. */
    BLE_GAP_EXT_SCAN_TYPE_SEL_CONN_DISC, /**< Selective connectable discovery. */
} ble_gap_ext_scan_type_t;

/**
 * @brief Filtering policy for duplicated packets
 */
typedef enum
{
    BLE_GAP_EXT_DUP_FILT_DIS = 0,   /**< Disable filtering of duplicated packets. */
    BLE_GAP_EXT_DUP_FILT_EN,        /**< Enable filtering of duplicated packets. */
    BLE_GAP_EXT_DUP_FILT_EN_PERIOD, /**< Enable filtering of duplicated packets, reset for each scan period. */
} ble_gap_ext_scan_dup_filt_policy_t;

/**
 * @brief Initiating types
 */
typedef enum
{
    BLE_GAP_INIT_TYPE_DIRECT_CONN_EST = 0, /**< Direct connection establishment: establish a connection with an indicated device. */
    BLE_GAP_INIT_TYPE_AUTO_CONN_EST,       /**< Automatic connection establishment: establish a connection with all devices whose address is present in the white list. */
    BLE_GAP_INIT_TYPE_NAME_DISC,           /**< Name discovery: establish a connection with an indicated device in order to read content of its device name characteristic. Connection is closed once this operation is stopped. */
} ble_gap_init_type_t;

/**
 * @brief Initiating properties
 */
typedef enum
{
    BLE_GAP_INIT_PROP_1M_BIT    = (1 << 0), /**< Scan connectable advertisements on the LE 1M PHY. Connection parameters for the LE 1M PHY are provided. */
    BLE_GAP_INIT_PROP_2M_BIT    = (1 << 1), /**< Connection parameters for the LE 2M PHY are provided. */
    BLE_GAP_INIT_PROP_CODED_BIT = (1 << 2), /**< Scan connectable advertisements on the LE Coded PHY. Connection parameters for the LE Coded PHY are provided. */
} ble_gap_init_prop_t;

/**
 * @brief Scanning properties bit field bit value
 */
typedef enum 
{
    BLE_GAP_SCAN_PROP_PHY_1M_BIT       = (1 << 0), /**< Scan advertisement on the LE 1M PHY. */
    BLE_GAP_SCAN_PROP_PHY_CODED_BIT    = (1 << 1), /**< Scan advertisement on the LE Coded PHY. */
    BLE_GAP_SCAN_PROP_ACTIVE_1M_BIT    = (1 << 2), /**< Active scan on LE 1M PHY (scan request PDUs may be sent). */
    BLE_GAP_SCAN_PROP_ACTIVE_CODED_BIT = (1 << 3), /**< Active scan on LE Coded PHY (scan request PDUs may be sent). */
    BLE_GAP_SCAN_PROP_ACCEPT_RPA_BIT   = (1 << 4), /**< Accept directed advertising packets if the used RPA and target address cannot be solved by the controller. */
    BLE_GAP_SCAN_PROP_FILT_TRUNC_BIT   = (1 << 5), /**< Filter truncated advertising or scan response reports. */
} ble_gap_scan_prop_t;

/**
 * @brief Periodic synchronization types
 */
typedef enum 
{
    BLE_GAP_PER_SYNC_TYPE_GENERAL = 0, /**< Do not use periodic advertiser list for synchronization. */
    BLE_GAP_PER_SYNC_TYPE_SELECTIVE,   /**< Use periodic advertiser list for synchronization. */
    BLE_GAP_PER_SYNC_TYPE_PAST,        /**< Use Periodic advertising sync transfer information send through connection for synchronization. */
} ble_gap_per_sync_type_t;

/**
 * @brief Constant Tone Extension sync filtering type
 */
enum gap_sync_cte_type
{
    BLE_GAP_NO_SYNC_IGNORE            = 0,         /**< The presence or absence of a Constant Tone Extension is irrelevant. */
    BLE_GAP_NO_SYNC_WITH_AOA          = (1 << 0),  /**< Do not sync to packets with an AoA Constant Tone Extension. */
    BLE_GAP_NO_SYNC_WITH_AOD_1US_SLOT = (1 << 1),  /**< Do not sync to packets with an AoD Constant Tone Extension with 1us slots. */
    BLE_GAP_NO_SYNC_WITH_AOD_2US_SLOT = (1 << 2),  /**< Do not sync to packets with an AoD Constant Tone Extension with 2us slots. */
    BLE_GAP_NO_SYNC_WITHOUT_CTE       = (1 << 4),  /**< Do not sync to packets without a Constant Tone Extension. */
};

 /**
 * @brief Periodic advertising sync information reception mode
 */
enum gap_per_adv_sync_info_rec_mode
{
    BLE_GAP_SYNC_REP_DIS     = 0x01,  /**< synchronize to the periodic advertising, per adv report events will be disabled. */
    BLE_GAP_SYNC_REP_EN      = 0x02,  /**< synchronize to the periodic advertising, per adv report events will be enabled, duplicate filtering disabled. */
    BLE_GAP_SYNC_REP_EN_DUP  = 0x03,  /**< synchronize to the periodic advertising, per adv report events will be enabled, duplicate filtering enabled. */
};

/**
 * @brief Supported AoA AoD switching sampling rate
 */
enum gap_switch_sampling_rate
{
    BLE_GAP_AOD_TX_1_US          =  (1 << 0),  /**< 1 us switching supported for AoD transmission. */
    BLE_GAP_AOD_RX_1_US          =  (1 << 1),  /**< 1 us switching supported for AoD reception. */
    BLE_GAP_AOA_RX_1_US          =  (1 << 2),  /**< 1 us switching and sampling supported for AoA reception. */
};

 /**
 * @brief Slot duration type for IQ sampling
 */
enum gap_slot_dur_type
{
    BLE_GAP_SLOT_DURA_1US      = 0x01,  /**< Switching and sampling slots are 1 us each. */
    BLE_GAP_SLOT_DURA_2US      = 0x02,  /**< Switching and sampling slots are 2 us each. */
};

/**
 * @brief Security level types
 */
enum gap_sec_lvl_type
{
    BLE_GAP_SEC_LVL_NO_SECURITY = 0, /**< No encryption or auth. */
    BLE_GAP_SEC_LVL_ENC_NO_AUTH,     /**< Encryption with no auth. */
    BLE_GAP_SEC_LVL_LE_ENC_AUTH,     /**< Legacy encryption with auth. */
    BLE_GAP_SEC_LVL_SC_ENC_AUTH,     /**< Security connection encryption with auth. */
};

/**
 * @brief GAP advertising types
 */
typedef enum
{
    BLE_GAP_AD_TYPE_FLAGS                      = 0x01, /**< Flag. */
    BLE_GAP_AD_TYPE_MORE_16_BIT_UUID           = 0x02, /**< Use of more than 16-bit UUID. */
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID  = 0x03, /**< Complete List of 16-bit UUID. */
    BLE_GAP_AD_TYPE_MORE_32_BIT_UUID           = 0x04, /**< Use of more than 32-bit UUD. */
    BLE_GAP_AD_TYPE_COMPLETE_LIST_32_BIT_UUID  = 0x05, /**< Complete List of 32-bit UUID. */
    BLE_GAP_AD_TYPE_MORE_128_BIT_UUID          = 0x06, /**< Use of more than 128-bit UUID. */
    BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID = 0x07, /**< Complete List of 128-bit UUID. */
    BLE_GAP_AD_TYPE_SHORTENED_NAME             = 0x08, /**< Shortened Device name. */
    BLE_GAP_AD_TYPE_COMPLETE_NAME              = 0x09, /**< Complete Device name. */
    BLE_GAP_AD_TYPE_TRANSMIT_POWER             = 0x0A, /**< Transmit power. */
    BLE_GAP_AD_TYPE_CLASS_OF_DEVICE            = 0x0D, /**< Class of device. */
    BLE_GAP_AD_TYPE_SP_HASH_C                  = 0x0E, /**< Simple Pairing Hash C. */
    BLE_GAP_AD_TYPE_SP_RANDOMIZER_R            = 0x0F, /**< Simple Pairing Randomizer. */
    BLE_GAP_AD_TYPE_TK_VALUE                   = 0x10, /**< Temporary key value. */
    BLE_GAP_AD_TYPE_OOB_FLAGS                  = 0x11, /**< Out of Band Flag. */
    BLE_GAP_AD_TYPE_SLAVE_CONN_INT_RANGE       = 0x12, /**< Slave connection interval range. */
    BLE_GAP_AD_TYPE_RQRD_16_BIT_SVC_UUID       = 0x14, /**< Require 16-bit service UUID. */
    BLE_GAP_AD_TYPE_RQRD_32_BIT_SVC_UUID       = 0x1F, /**< Require 32-bit service UUID. */
    BLE_GAP_AD_TYPE_RQRD_128_BIT_SVC_UUID      = 0x15, /**< Require 128-bit service UUID. */
    BLE_GAP_AD_TYPE_SERVICE_16_BIT_DATA        = 0x16, /**< Service data 16-bit UUID. */
    BLE_GAP_AD_TYPE_SERVICE_32_BIT_DATA        = 0x20, /**< Service data 32-bit UUID. */
    BLE_GAP_AD_TYPE_SERVICE_128_BIT_DATA       = 0x21, /**< Service data 128-bit UUID. */
    BLE_GAP_AD_TYPE_PUB_TGT_ADDR               = 0x17, /**< Public Target Address. */
    BLE_GAP_AD_TYPE_RAND_TGT_ADDR              = 0x18, /**< Random Target Address. */
    BLE_GAP_AD_TYPE_APPEARANCE                 = 0x19, /**< Appearance. */
    BLE_GAP_AD_TYPE_ADV_INTV                   = 0x1A, /**< Advertising Interval. */
    BLE_GAP_AD_TYPE_LE_BT_ADDR                 = 0x1B, /**< LE Bluetooth Device Address. */
    BLE_GAP_AD_TYPE_LE_ROLE                    = 0x1C, /**< LE Role. */
    BLE_GAP_AD_TYPE_SPAIR_HASH                 = 0x1D, /**< Simple Pairing Hash C-256. */
    BLE_GAP_AD_TYPE_SPAIR_RAND                 = 0x1E, /**< Simple Pairing Randomizer R-256. */
    BLE_GAP_AD_TYPE_3D_INFO                    = 0x3D, /**< 3D Information Data. */

    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA         = 0xFF, /**< Manufacturer specific data. */
} ble_gap_ad_type_t;

/** @brief Advertising report type. */
typedef enum
{
    BLE_GAP_REPORT_TYPE_ADV_EXT = 0,  /**< Extended advertising report. */
    BLE_GAP_REPORT_TYPE_ADV_LEG,      /**< Legacy advertising report. */
    BLE_GAP_REPORT_TYPE_SCAN_RSP_EXT, /**< Extended scan response report. */
    BLE_GAP_REPORT_TYPE_SCAN_RSP_LEG, /**< Legacy scan response report. */
    BLE_GAP_REPORT_TYPE_PER_ADV,      /**< Periodic advertising report. */
} ble_gap_adv_report_type_t;

/** @brief Advertising report information. */
typedef enum
{
    BLE_GAP_REPORT_INFO_COMPLETE_BIT = (1 << 0), /**< Report is complete. */
    BLE_GAP_REPORT_INFO_CONN_ADV_BIT = (1 << 1), /**< Connectable advertising. */
    BLE_GAP_REPORT_INFO_SCAN_ADV_BIT = (1 << 2), /**< Scannable advertising. */
    BLE_GAP_REPORT_INFO_DIR_ADV_BIT  = (1 << 3), /**< Directed advertising. */
} ble_gap_adv_report_info_t;

/** @brief Stop reason code. */
typedef enum
{
    BLE_GAP_STOPPED_REASON_TIMEOUT = 0,          /**< Stop with timeout. */
    BLE_GAP_STOPPED_REASON_ON_USER,              /**< Stop with user stopping it actively. */
    BLE_GAP_STOPPED_REASON_CONN_EST              /**< Stop with connection established. */
} ble_gap_stopped_reason_t;

/**
 * @brief AD Type Flag - Bit mask.
 */
typedef enum
{
    BLE_GAP_ADV_FLAG_LE_LIMITED_DISC_MODE       = 0x01, /**< Limited discovery flag: AD Flag. */
    BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE       = 0x02, /**< General discovery flag: AD Flag. */
    BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED       = 0x04, /**< Legacy BT not supported: AD Flag. */
    BLE_GAP_ADV_FLAG_SIMUL_BR_EDR_LE_CONTROLLER = 0x08, /**< Dual mode for controller supported (BR/EDR/LE): AD Flag. */
    BLE_GAP_ADV_FLAG_SIMUL_BR_EDR_LE_HOST       = 0x10, /**< Dual mode for host supported (BR/EDR/LE): AD Flag. */
} ble_gap_adv_flags_t;

/**
 * @brief Privacy mode.
 */

typedef enum
{
    BLE_GAP_PRIVACY_MODE_NETWORK= 0x00,      /**< Set to the network privacy mode for the peer device. */
    BLE_GAP_PRIVACY_MODE_DEVICE = 0x01,      /**< Set to the device privacy mode for the peer device. */
} ble_gap_privacy_mode_t;

/**
 * @brief Possible roles of the activity. 
 */
typedef enum
{
    BLE_GAP_ACTIVITY_ROLE_ADV = 0,          /**< Adertise role. */
    BLE_GAP_ACTIVITY_ROLE_CON = 1,          /**< Connect role. */
    BLE_GAP_ACTIVITY_ROLE_SCAN_INIT = 2,    /**< Scann role. */
    BLE_GAP_ACTIVITY_ROLE_UNKNOWN = 0xf,    /**< Unknown role. */
} ble_gap_actv_role_t;

/** @} */

/**@addtogroup BLE_GAPM_STRUCTURES Structures
 * @{ */

/**
 * @brief Advertising parameters for legacy advertising
 */
typedef struct
{
    uint8_t          adv_mode;        /**< Advertising mode (see enum @ref  ble_gap_adv_mode_t). */
    uint8_t          disc_mode;       /**< Discovery mode (see enum @ref ble_gap_disc_mode_t). */
    uint8_t          filter_pol;      /**< Advertising filtering policy (see enum @ref ble_gap_adv_filter_policy_t). */
    ble_gap_bdaddr_t peer_addr;       /**< Peer address configuration (only used in case of directed advertising, or used to locate the IRK list). */
    uint16_t         adv_intv_min;    /**< Minimum advertising interval (in unit of 625 μs). Must be greater than 20 ms. */
    uint16_t         adv_intv_max;    /**< Maximum advertising interval (in unit of 625 μs). Must be greater than 20 ms. */
    uint8_t          chnl_map;        /**< Advertising channel map. See @ref BLE_GAP_ADV_CHANNEL. */
    bool             scan_req_ind_en; /**< Indicate if the application should be informed when receiving a scan request from the scanner. */
    int8_t           max_tx_pwr;      /**< Maximum power level at which the advertising packets have to be transmitted (between -127 dBm and 127 dBm).
                                           For the real value, please refer to GR551x Datasheet. */
} ble_gap_adv_param_t;

/**
 * @brief Configuration for advertising on primary channel
 */
typedef struct
{
    uint32_t                adv_intv_min;       /**< Minimum advertising interval (in unit of 625 μs). Must be greater than 20 ms. */
    uint32_t                adv_intv_max;       /**< Maximum advertising interval (in unit of 625 μs). Must be greater than 20 ms. */
    uint8_t                 chnl_map;           /**< Bit field indicating the channel map. See @ref BLE_GAP_ADV_CHANNEL. */
    ble_gap_le_phy_value_t  phy;                /**< Indicate on which PHY primary advertising has to be performed. See enum @ref ble_gap_le_phy_value_t. 
                                                     Note that LE 2M PHY is not allowed and that legacy advertising only supports LE 1M PHY. */
} ble_gap_adv_prim_cfg_t;

/**
 * @brief Configuration for advertising on secondary channel
 */
typedef struct
{
    uint8_t                 max_skip;       /**< Maximum number of advertising events the controller can skip before sending the AUX_ADV_IND packets.
                                                 The range is 0x00 to 0xFF. 0x00 means that AUX_ADV_IND PDUs shall be sent prior to each advertising events. */
    ble_gap_le_phy_value_t  phy;            /**< Indicate on which PHY secondary advertising has to be performed. See enum @ref ble_gap_le_phy_value_t. */
    uint8_t                 adv_sid;        /**< Advertising SID. Allowed range is 0x00 to 0x0F. */
} ble_gap_adv_second_cfg_t;

/**
 * @brief Configuration for periodic advertising
 */
typedef struct
{
    uint16_t    adv_intv_min;                     /**< Minimum advertising interval (in unit of 1.25 ms). Must be greater than 20 ms. */
    uint16_t    adv_intv_max;                     /**< Maximum advertising interval (in unit of 1.25 ms). Must be greater than 20 ms. */
    bool        inc_adi_flag;                     /**< Flag for include the ADI field in AUX_SYNC_IND PDUs. */
} ble_gap_adv_period_cfg_t;

/**
 * @brief  Advertising parameters for extended advertising and periodic advertising
 */
typedef struct
{
    uint8_t                  type;                      /**< Advertising type (see enum @ref ble_gap_adv_type_t). */
    uint8_t                  disc_mode;                 /**< Discovery mode (see enum @ref ble_gap_disc_mode_t). */
    uint16_t                 prop;                      /**< Bit field value provided by advertising properties. See enum @ref ble_gap_adv_prop_t for bit signification. */
    int8_t                   max_tx_pwr;                /**< Maximum power level at which the advertising packets have to be transmitted (between -127 and 127 dBm). */
    uint8_t                  filter_pol;                /**< Advertising filtering policy (see enum @ref ble_gap_adv_filter_policy_t). */
    ble_gap_bdaddr_t         peer_addr;                 /**< Peer address configuration (only used in case of directed advertising or used to locate the IRK list). */
    ble_gap_adv_prim_cfg_t   prim_cfg;                  /**< Configuration for primary advertising. */
    ble_gap_adv_second_cfg_t second_cfg;                /**< Configuration for secondary advertising (valid only if advertising type is GAP_ADV_TYPE_EXTENDED or GAP_ADV_TYPE_PERIODIC). */
    ble_gap_adv_period_cfg_t period_cfg;                /**< Configuration for periodic advertising (valid only if advertising type is GAP_ADV_TYPE_PERIODIC). */
} ble_gap_ext_adv_param_t;

/**
 * @brief Advertising timing parameter
 */
typedef struct
{
    uint16_t duration;    /**< Advertising duration (in unit of 10ms). 0 means that advertising continues until the host disables it. 
                               If Advertising discovery mode is GAP_DISC_MODE_LIM_DISCOVERABLE (see enum @ref ble_gap_disc_mode_t), 
                               the setting duration range is [1, 18000].
                               If adv mode is high duty, duration time range is [1, 128]. */
    uint8_t  max_adv_evt; /**< Maximum number of extended advertising events. The controller shall attempt to send prior to terminating the extending advertising.
                               The range is [0, 255]. 0 means no maximum number of advertising events. Valid only if the created advertising is an extended advertising. */
} ble_gap_adv_time_param_t;

/**
 * @brief Security key
 */
typedef struct
{
    uint8_t key[BLE_GAP_MAX_KEY_LEN]; /**< Key value MSB -> LSB (MSB followed by LSB). */
} ble_gap_sec_key_t;

/**
 * @brief Parameters for legacy scanning
 */
typedef struct
{
    ble_gap_scan_type_t             scan_type;              /**< Active scanning or passive scanning. */
    ble_gap_scan_mode_t             scan_mode;              /**< Scan mode. */
    ble_gap_scan_dup_filt_policy_t  scan_dup_filt;          /**< Duplicate filter policy. */
    bool                            use_whitelist;          /**< Filter policy. */
    uint16_t                        interval;               /**< Scan interval between 0x0004 and 0x4000 in 0.625 ms (range: 2.5 ms to 10.24s). */
    uint16_t                        window;                 /**< Scan window between 0x0004 and 0x4000 in 0.625 ms (range: 2.5 ms to 10.24s). */
    uint16_t                        timeout;                /**< Scan timeout should be a value between 0x0001 and 0xFFFF(unit: 10 ms). 0x0000 indicates that the timeout has no effect. */
} ble_gap_scan_param_t;

/**
 * @brief Scan Window operation parameters
 */
typedef struct
{
    uint16_t scan_intv; /**< Scan interval between 0x0004 and 0xFFFF in 0.625 ms (range: 2.5 ms to 40.959375s). */
    uint16_t scan_wd;   /**< Scan window between 0x0004 and 0xFFFF in 0.625 ms (range: 2.5 ms to 40.959375s). */
} ble_gap_scan_wd_op_param_t;

/**
 * @brief Parameters for extended scanning
 */
typedef struct
{
    uint8_t                     type;               /**< Type of scanning to be started (see enum @ref ble_gap_ext_scan_type_t). */
    uint8_t                     prop;               /**< Properties for the scan procedure (see enum @ref ble_gap_scan_prop_t for bit signification). */
    uint8_t                     dup_filt_pol;       /**< Duplicate packet filtering policy (see enum @ref ble_gap_ext_scan_dup_filt_policy_t). */
    uint8_t                     rsvd;               /**< Reserved for future use. */
    ble_gap_scan_wd_op_param_t  scan_param_1m;      /**< Scan window opening parameters for LE 1M PHY. */
    ble_gap_scan_wd_op_param_t  scan_param_coded;   /**< Scan window opening parameters for LE Coded PHY. */
    uint16_t duration;                              /**< Scan duration (in unit of 10ms). 0 means that the controller will scan continuously 
                                                         until receiving a stop command from the application (10 ms to 655.35s). */
    uint16_t period;                                /**< Scan period, which is the time interval between two consequent starts of a scan duration by the controller.
                                                         0 means that the scan procedure is not periodic, in unit of 1.28s (1.28s to 83,884.8s). */
} ble_gap_ext_scan_param_t;

/**
 * @brief Periodic advertising information
 */
typedef struct
{
    ble_gap_bdaddr_t bd_addr;       /**< Advertiser address information. */
    uint8_t          adv_sid;       /**< Advertising SID. */
} ble_gap_period_adv_addr_cfg_t;

/**
 * @brief Periodic advertising synchronization parameters
 */
typedef struct
{
    uint16_t                      skip;             /**< Number of periodic advertising that can be skipped after a successful reception. 
                                                         Maximum authorized value is 499. */
    uint16_t                      sync_to;          /**< Synchronization timeout for the periodic advertising (in unit of 10ms between 100ms and 163.84s). */
    uint8_t                       type;             /**< Periodic synchronization type (see enum @ref ble_gap_per_sync_type_t). */
    uint8_t                       rsvd;             /**< Reserved for future use.*/
    ble_gap_period_adv_addr_cfg_t adv_addr;         /**< Address of advertiser with which synchronization has to be established(used only if use_pal is false). */
    bool                          report_disable;   /**< true to disable periodic advertising report, false to enable them by default.*/
    uint8_t                       cte_type;         /**< Type of Constant Tone Extension device should sync on (@see enum gap_sync_cte_type). */
} ble_gap_per_sync_param_t;

/**
 * @brief Set connectionless CTE transmit parameters 
 */
typedef struct
{
    uint8_t     cte_len;                          /**< CTE length (8us unit, range 0x02 to 0x14). */
    uint8_t     cte_type;                         /**< CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us) (@see enum gap_cte_type_t). */
    uint8_t     cte_count;                        /**< CTE count (number of CTEs to transmit in each periodic advertising interval, range 0x01 to 0x10). */
    uint8_t     num_antenna;                      /**< The number of Antenna IDs in the pattern (range 0x02 to 0x4B). */
    uint8_t     *antenna_id;                      /**< List of Antenna IDs in the pattern. */
} gap_connless_cte_trans_param_t;

/**
 * @brief Set connectionless IQ sampling enable parameter
 */
typedef struct
{
    uint8_t  slot_dur;                    /**< The slot duration for IQ sampling, see enum @ref gap_slot_dur_type. */
    uint8_t  max_smaple_cte;              /**< The maximum number of CTE to sample and report in each periodic advertising interval (range 0x00 to 0x10).
                                               0x00: Sample and report all available CTE. */
    uint8_t  antenna_num;                 /**< The number of antenna IDs in the pattern (range 0x02 to 0x4B). */
    uint8_t  *antenna_id;                 /**< List of Antenna IDs in the pattern. */
} ble_gap_connless_iq_sample_param_t;

/**
 * @brief Legacy initiating parameters
 */
typedef struct
{
    uint8_t      type;           /**< Initiating type (see enum @ref ble_gap_init_type_t). */
    ble_gap_bdaddr_t peer_addr;      /**< Peer device address. */
    uint16_t     interval_min;   /**< Minimum value for the connection interval (in unit of 1.25ms). Shall be less than or equal to interval_max value.
                                      Allowed range is 7.5 ms to 4s. */
    uint16_t     interval_max;   /**< Maximum value for the connection interval (in unit of 1.25ms). Shall be greater than or equal to interval_min value.
                                      Allowed range is 7.5 ms to 4s. */
    uint16_t     slave_latency;  /**< Slave latency. Number of events that can be missed by a connected slave device. */
    uint16_t     sup_timeout;    /**< Link supervision timeout (in unit of 10ms). Allowed range is 100 ms to 32s. */
    uint16_t     conn_timeout;   /**< Timeout for connection establishment (in unit of 10ms).
                                      Cancel the procedure if connection has not been connected when the timeout occurs.
                                      0 means there is no timeout. */
} ble_gap_init_param_t;


/**
 * @brief Connection parameters
 */
typedef struct
{
    uint16_t conn_intv_min;  /**< Minimum value for the connection interval (in unit of 1.25ms). Shall be less than or equal to conn_intv_max value.
                                  Allowed range is 7.5 ms to 4s. */
    uint16_t conn_intv_max;  /**< Maximum value for the connection interval (in unit of 1.25ms). Shall be greater than or equal to conn_intv_min value.
                                  Allowed range is 7.5 ms to 4s. */
    uint16_t conn_latency;   /**< Slave latency. Number of events that can be missed by a connected slave device. */
    uint16_t supervision_to; /**< Link supervision timeout (in unit of 10ms). Allowed range is 100 ms to 32s. */
    uint16_t ce_len;         /**< The length of connection event needed for this LE connection.  Range: 0x0002 to 0xFFFF, Unit:0.625 ms, Time Range: 1.25 ms to 40.9s. */
} ble_gap_ext_conn_param_t;

/**
 * @brief Extended initiating parameters
 */
typedef struct
{
    uint8_t                     type;               /**< Initiating type (see enum @ref ble_gap_init_type_t). */
    uint8_t                     prop;               /**< Properties for the initiating procedure (see enum @ref ble_gap_init_prop_t for bit signification). */
    uint16_t                    conn_to;            /**< Timeout for automatic connection establishment (in unit of 10ms).
                                                         Cancel the procedure if connection has not been connected when the timeout occurs.
                                                         0 means there is no timeout. */
    ble_gap_scan_wd_op_param_t  scan_param_1m;      /**< Scan window opening parameters for LE 1M PHY. */
    ble_gap_scan_wd_op_param_t  scan_param_coded;   /**< Scan window opening parameters for LE Coded PHY. */
    ble_gap_ext_conn_param_t    conn_param_1m;      /**< Connection parameters for LE 1M PHY. */
    ble_gap_ext_conn_param_t    conn_param_2m;      /**< Connection parameters for LE 2M PHY. */
    ble_gap_ext_conn_param_t    conn_param_coded;   /**< Connection parameters for LE Coded PHY. */
    ble_gap_bdaddr_t            peer_addr;          /**< Address of peer device in case white list is not used for connection. */
} ble_gap_ext_init_param_t;

/**
 * @brief LE Protocol/Service Multiplexer information
 */
typedef struct
{
    uint16_t    le_psm;     /**< LE Protocol/Service Multiplexer (range: 1 to 255), PSMs should be odd. */
    uint8_t     sec_lvl;    /**< Security level requirement, see @ref gap_sec_lvl_type. */
    bool        mks_flag;   /**< Whether to use maximum-size key (16 bytes) or not. */
} ble_gap_lepsm_register_t;

/**
 * @brief Bonded device list
 */
typedef struct
{
    uint8_t      num;                   /**< Number of bonded device. */
    ble_gap_bdaddr_t *items;                /**< Bonded device addr info. */
} ble_gap_bond_dev_list_t;

/**
 * @brief White list 
 */
typedef struct
{
    uint8_t      num;                   /**< Number of available items. */
    ble_gap_bdaddr_t items[BLE_GAP_MAX_WL_NUM];     /**< Content of each item. */
} ble_gap_white_list_t;

/**
 * @brief Periodic advertising list
 */
typedef struct
{
    uint8_t                         num;                     /**< Number of available items. */
    ble_gap_period_adv_addr_cfg_t   items[BLE_GAP_MAX_PRD_ADV_NUM];     /**< Content of each item. */
} ble_gap_period_adv_list_t;

/**
 * @brief RPA list item info 
 */
typedef struct
{
    ble_gap_bdaddr_t    bd_addr;                    /**< Peer device identity. */
    uint8_t         priv_mode;                  /**< Privacy mode, see enum @ref ble_gap_privacy_mode_t. */
    uint8_t         peer_irk[BLE_GAP_MAX_KEY_LEN];      /**< Peer IRK. */
    uint8_t         local_irk[BLE_GAP_MAX_KEY_LEN];     /**< Local IRK. */
} ble_gap_ral_dev_info_t;

/**
 * @brief RPA list info 
 */
typedef struct
{
    uint8_t                offset;                      /**< offset of RPA list. */
    uint8_t                num;                         /**< Number of RPA list. */
    ble_gap_ral_dev_info_t *items;                      /**< RPA list item info. */
} ble_gap_ral_dev_list_t;

/**
 * @brief Operation code used to read resolvable address.
 */
typedef enum
{
    BLE_GAP_OPCODE_LOCAL_RSLV_ADDR_READ,    /**< Local resolvable address operation. */
    BLE_GAP_OPCODE_PEER_RSLV_ADDR_READ,     /**< Peer resolvable address operation. */
} ble_gap_rslv_addr_read_op_id_t;

/** @brief Get device info event for @ref BLE_GAPM_EVT_DEV_INFO_GOT. */
typedef struct
{
    uint8_t               operation;      /**< Operation code. @ref ble_gap_dev_info_get_type_t. */
    ble_gap_dev_info_t    dev_info;       /**< Device info. */
 } ble_gap_evt_dev_info_get_t;

/** @brief Advertising Stop info. */
typedef struct
{
    ble_gap_stopped_reason_t  reason;       /**< The reason for the advertising stop. */
} ble_gap_evt_adv_stop_t;

/** @brief Scan request info. */
typedef struct
{
    ble_gap_bdaddr_t    peer_addr;          /**< Peer address. */
} ble_gap_evt_scan_req_t;

/** @brief Scan Stop info struct. */
typedef struct
{
    ble_gap_stopped_reason_t    reason;     /**< The reason why the scan stopped. */
} ble_gap_evt_scan_stop_t;

/** @brief Advertising report event for @ref BLE_GAPM_EVT_ADV_REPORT. */
typedef struct
{
    ble_gap_adv_report_type_t      adv_type;              /**< Advertising type. @ref ble_gap_adv_report_type_t. */
    ble_gap_adv_report_info_t      adv_info;              /**< Bit field providing information about the received report. @ref ble_gap_adv_report_info_t. */
    ble_gap_bdaddr_t               broadcaster_addr;      /**< Broadcaster device address. */
    ble_gap_bdaddr_t               direct_addr;           /**< Target address (in case of a directed advertising report). */
    int8_t                         tx_pwr;                /**< TX power (in dBm). */
    int8_t                         rssi;                  /**< RSSI (between -127 and +20 dBm). */
    uint8_t                        phy_prim;              /**< Primary PHY on which advertising report has been received. */
    uint8_t                        phy_second;            /**< Secondary PHY on which advertising report has been received. */
    uint8_t                        adv_sid;               /**< Advertising SID , valid only for periodic advertising report. */
    uint16_t                       period_adv_intv;       /**< Periodic advertising interval (in unit of 1.25ms, min is 7.5ms), valid only for periodic advertising report. */
    uint8_t                        per_sync_idx;          /**< Periodic syncronization index, valid only for periodic advertising report. */
    uint16_t                       length;                /**< Report length. */
    uint8_t                       *data;                  /**< Report data. */
} ble_gap_evt_adv_report_t;

/** @brief Sync established event for @ref BLE_GAPM_EVT_SYNC_ESTABLISH. */
typedef struct
{
    uint8_t         phy;           /**< PHY on which synchronization has been established. */
    uint16_t        intv;          /**< Periodic advertising interval (in unit of 1.25ms, min is 7.5ms). */
    uint8_t         adv_sid;       /**< Advertising SID. */
    uint8_t         clk_acc;       /**< Advertiser clock accuracy. */
    ble_gap_bdaddr_t bd_addr;      /**< Advertiser address. */
    uint16_t        sync_hdl;      /**< Sync handle. */
} ble_gap_evt_sync_established_t;

/** @brief Read resolvable address event for @ref BLE_GAPM_EVT_READ_RSLV_ADDR. */
typedef struct
{
    uint8_t           op_code;        /**< Operation code. @ref ble_gap_rslv_addr_read_op_id_t. */
    ble_gap_addr_t    gap_addr;       /**< Resolvable address info. */
} ble_gap_evt_rslv_addr_read_t;

/**@brief BLE GAPM event structure. */
typedef struct
{
    uint8_t  index;                                             /**< Index of connection or advertising. */
    union
    {
        ble_gap_evt_dev_info_get_t       dev_info;              /**< Device info parameters. */
        ble_gap_evt_adv_stop_t           adv_stop;              /**< Advertising stop parameter. */
        ble_gap_evt_scan_req_t           scan_req;              /**< Scan Request parameter. */
        ble_gap_evt_scan_stop_t          scan_stop;             /**< Scan Stop parameter. */
        ble_gap_evt_adv_report_t         adv_report;            /**< Advertising Report parameter. */
        ble_gap_evt_sync_established_t   sync_established;      /**< Periodic Advertising Synchronization Established parameter. */
        ble_gap_evt_rslv_addr_read_t     rslv_addr;             /**< Read Resolvable Address parameter. */
    } params;                                                   /**< Event Parameters. */
} ble_gapm_evt_t;
/** @} */



/** @addtogroup BLE_GAPM_FUNCTION Functions
 * @{ */

/**
 ****************************************************************************************
 * @brief Decide whether to support the pair feature.
 *
 * @param[in] enable: Support flag.
 ****************************************************************************************
 */
void ble_gap_pair_enable(bool enable);

/**
 ****************************************************************************************
 * @brief Set the device identity address (public or random static).
 *
 * @param[in] p_addr: Provided public or static random address. If addr_type = 0, it means public address,
 *            If addr_type = 1, it means static random address.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_addr_set(ble_gap_bdaddr_t const *p_addr);

/**
 ****************************************************************************************
 * @brief Get the device identity address (public or random static).
 *
 * @param[in] p_addr: The pointer for the returned identity address.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_addr_get(ble_gap_bdaddr_t *p_addr);

/**
 ****************************************************************************************
 * @brief Set the tx power
 *
 * @param[in] role: Select the role to set tx power. @ref ble_gap_actv_role_t for possible roles.
 * @param[in] index: The idx parameter is interpreted on role.
 *              -If role is @ref BLE_GAP_ACTIVITY_ROLE_ADV, it's the index of Advertising.
 *              -If role is @ref BLE_GAP_ACTIVITY_ROLE_CON, it's the index of connection.
 *              -For all other roles, it should be ignored.
 * @param[in] txpwr_dbm: The value of the tx power, Range: -127dbm to 127dbm.
 *
 * @retval SDK_SUCCESS: Operation is Success.
 * @retval SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval SDK_ERR_INVALID_ADV_IDX: Invalid advertising index supplied.
 * @retval SDK_ERR_INVALID_HANDLE: Invalid handle supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_tx_power_set(ble_gap_actv_role_t role, uint8_t index, int8_t txpwr_dbm);

/**
 ****************************************************************************************
 * @brief Get the tx power
 *
 * @param[in] role: Select the role to Get tx power. @ref ble_gap_actv_role_t for possible roles.
 * @param[in] index: The idx parameter is interpreted on role.
 *                   If role is @ref BLE_GAP_ACTIVITY_ROLE_ADV, it's the index of Advertising.
 *                   If role is @ref BLE_GAP_ACTIVITY_ROLE_CON, it's the index of connection.
 *                   For all other roles, it should be ignored.
 * @param[in] txpwr_dbm: The value of the tx power, Range: -127dbm to 128dbm.
 *
 * @retval SDK_SUCCESS: Operation is Success.
 * @retval SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval SDK_ERR_INVALID_ADV_IDX: Invalid advertising index supplied.
 * @retval SDK_ERR_INVALID_HANDLE: Invalid handle supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_tx_power_get(ble_gap_actv_role_t role, uint8_t index, int8_t *txpwr_dbm);

/**
 ****************************************************************************************
 * @brief Set IRK used for resolvable random BD address generation (LSB first).
 *
 * @param[in] p_irk: Device IRK (Identity Resolving Key).
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_irk_set(ble_gap_sec_key_t* p_irk);

/**
 ****************************************************************************************
 * @brief Set privacy related parameters.
 * @param[in] renew_dur: Duration before regenerating a device address when privacy is enabled in seconds. Range: 0x0001 (1s) ~ 0xA1B8 (11.5 hr).
 *            The suggested time is 900s(15 minutes).
 * @param[in] enable_flag: Indicate the controller privacy is enabled or disabled.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_privacy_params_set(uint16_t renew_dur, bool enable_flag);

/**
 ****************************************************************************************
 * @brief Set suggested default LE data length.
 *
 * @param[in] sugg_max_tx_octet: Suggested value for the Controller's maximum transmitted number of payload octets to be used, the range is 27~251.
 * @param[in] sugg_max_tx_time:  Suggested value for the Controller's maximum packet transmission time to be used, the range is 328~2120.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_data_length_set(uint16_t sugg_max_tx_octet,uint16_t sugg_max_tx_time);

/**
 ****************************************************************************************
 * @brief Set L2CAP related parameters.
 *
 * @param[in] max_mtu:     Maximal MTU acceptable for device, the range is 65~512.
 * @param[in] max_mps:     Maximal MPS Packet size acceptable for device (for COC SDU), the range is 65~max_mtu.
 * @param[in] max_nb_lecb: Maximum number of LE Credit based connection that can be established, this range is 0x00~0x20.
 *                         The actual number is decided by resource available.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 *
 * @note If these parameters are not set, the stack will config the default value as (max_mtu = 512, max_mps = 512 and  max_nb_lecb = 10).
 ****************************************************************************************
 */
uint16_t ble_gap_l2cap_params_set(uint16_t max_mtu,uint16_t max_mps,uint8_t max_nb_lecb);

/**
 ****************************************************************************************
 * @brief Set the preferred values for the transmitter PHY and receiver PHY.
 *
 * @param[in] tx_pref_phy: A bit field that indicates the transmitter PHYs that the Host prefers the Controller to use(see @ref BLE_GAP_PHYS).
 * @param[in] rx_pref_phy: A bit field that indicates the receiver PHYs that the Host prefers the Controller to use(see @ref BLE_GAP_PHYS).
 ****************************************************************************************
 */
void ble_gap_pref_phy_set(uint8_t tx_pref_phy, uint8_t rx_pref_phy);

/**
 ****************************************************************************************
 * @brief Set the RF path gain or loss between the RF transceiver and the antenna contributed by intermediate components.
 *
 * @param[in] tx_path_comp: RF TX Path Compensation value (from -128dB to 128dB, unit is 0.1dB).
 * @param[in] rx_path_comp: RF RX Path Compensation value (from -128dB to 128dB, unit is 0.1dB).
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PARAM: The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_gap_path_compensation_set(int16_t tx_path_comp, int16_t rx_path_comp);

/**
 ****************************************************************************************
 * @brief Set GAP Peripheral Preferred Connection Parameters present flag.
 *
 * @param[in] present_flag: Present or not.
 ****************************************************************************************
 */
void ble_gap_ppcp_present_set(bool present_flag);

/**
 ****************************************************************************************
 * @brief Set advertising channel map before advertising starting.
 * @note  This API is asynchronous.
 * @note  Once the operation has completed, the event @ref BLE_GAP_OPCODE_CHNL_MAP_SET will be called.
 *
 * @param[in] p_chnl_map: Bitmask of LE channel map. See enum @ref ble_gap_chnl_map_t for BT Core Spec version <= 4.2.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 ****************************************************************************************
 */
uint16_t ble_gap_chnl_map_set(ble_gap_chnl_map_t* p_chnl_map);

/**
 ****************************************************************************************
 * @brief Get all bonded devices.
 *
 * @param[in] p_bond_list: Pointer to the bond list.
 * @param[in] max_num: max number of bond devices can be gotten.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_SDK_INTERNAL: SDK internal error.
 * @retval SDK_ERR_NVDS_NOT_INIT: NVDS is not initiated.
 * @retval SDK_ERR_LIST_ITEM_NOT_FOUND: Item not found in list.
 * @retval SDK_ERR_LIST_FULL: List is full.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_bond_devs_get(ble_gap_bond_dev_list_t *p_bond_list, uint8_t max_num);

/**
 ****************************************************************************************
 * @brief Clear all bonded devices.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_SDK_INTERNAL: SDK internal error.
 * @retval SDK_ERR_NVDS_NOT_INIT: NVDS is not initiated.
 * @retval SDK_ERR_LIST_ITEM_NOT_FOUND: Item not found in list.
 * @retval SDK_ERR_LIST_FULL: List is full.
 ****************************************************************************************
 */
uint16_t ble_gap_bond_devs_clear(void);

/**
 ****************************************************************************************
 * @brief Delete a bond device with the specified BD address.
 *
 * @param[in] p_peer_addr: Pointer to the BD addrss.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_SDK_INTERNAL: SDK internal error.
 * @retval SDK_ERR_NVDS_NOT_INIT: NVDS is not initiated.
 * @retval SDK_ERR_LIST_ITEM_NOT_FOUND: Item not found in list.
 * @retval SDK_ERR_LIST_FULL: List is full.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_bond_dev_del(const ble_gap_bdaddr_t *p_peer_addr);

/**
 ****************************************************************************************
 * @brief Get the content of the whole white list.
 *
 * @param[in] p_whitelist: Pointer to the output white list.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_whitelist_get(ble_gap_white_list_t *p_whitelist);

/**
 ****************************************************************************************
 * @brief Add the devices into current white list. If white list is full or there are duplicated entries, it will return error.
 * @note  This API is asynchronous.
 * @note  Once the operation has completed, the event @ref BLE_GAP_OPCODE_WHITELIST_SET will be called.
 *
 * @param[in] p_whitelist: Pointer to input white list. 
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_LIST_ITEM_ALREADY_EXISTED: Item already existed in list.
 * @retval SDK_ERR_LIST_FULL: List is full.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 ****************************************************************************************
 */
uint16_t ble_gap_whitelist_add(const ble_gap_white_list_t *p_whitelist);

/**
 ****************************************************************************************
 * @brief Delete the devices out of current white list. If the entries do not exist in the current white list, it will return error.
 * @note  This API is asynchronous.
 * @note  Once the operation has completed, the event @ref BLE_GAP_OPCODE_WHITELIST_SET will be called.
 *
 * @param[in] p_whitelist: Pointer to input white list.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_LIST_ITEM_NOT_FOUND: Item not found in list.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 ****************************************************************************************
 */
uint16_t ble_gap_whitelist_del(const ble_gap_white_list_t *p_whitelist);

/**
 ****************************************************************************************
 * @brief Clear all the entries in the current white list.
 * @note  This API is asynchronous.
 * @note  once the operation has completed, the event @ref BLE_GAP_OPCODE_WHITELIST_SET will be called.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 ****************************************************************************************
 */
uint16_t ble_gap_whitelist_clear(void);

/**
 ****************************************************************************************
 * @brief Get the content of the whole periodic advertising list.
 *
 * @param[in] p_pal_list: Pointer to the output periodic advertising list.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_per_adv_list_get(ble_gap_period_adv_list_t *p_pal_list);

/**
 ****************************************************************************************
 * @brief Add the devices into current periodic advertising list. If periodic advertising list is full or there are
 *        duplicated entries, it will return error.
 * @note  This API is asynchronous.
 * @note  Once the operation has completed, the event @ref BLE_GAP_OPCODE_PER_ADV_LIST_SET will be called.
 *
 * @param[in] p_pal_list: Pointer to input periodic advertising list.
 *

 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_LIST_ITEM_ALREADY_EXISTED: Item already existed in list.
 * @retval SDK_ERR_LIST_FULL: List is full.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 ****************************************************************************************
 */
uint16_t ble_gap_per_adv_list_add(const ble_gap_period_adv_list_t *p_pal_list);

/**
 ****************************************************************************************
 * @brief Delete the devices out of periodic advertising list. If the entries do not exist in the current list, an error will be returned.
 * @note  This API is asynchronous.
 * @note  Once the operation has completed, the event @ref BLE_GAP_OPCODE_PER_ADV_LIST_SET will be called.
 *
 * @param[in] p_pal_list: Pointer to input periodic advertising list.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_LIST_ITEM_NOT_FOUND: Item not found in list.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 ****************************************************************************************
 */
uint16_t ble_gap_per_adv_list_del(const ble_gap_period_adv_list_t *p_pal_list);

/**
 ****************************************************************************************
 * @brief Clear all the entries in the current periodic advertising list.
 * @note This API is asynchronous.
 * @note Once the operation has completed, the event @ref BLE_GAP_OPCODE_PER_ADV_LIST_SET will be called.  
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 ****************************************************************************************
 */
uint16_t ble_gap_per_adv_list_clear(void);

/**
 ****************************************************************************************
 * @brief Get the RPA list info.
 * @param[in] p_rpa_list: Pointer to the output RPA list.
 * @param[in] max_num: Max number of rpa list can be gotten.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_rpa_list_get(ble_gap_ral_dev_list_t *p_rpa_list, uint8_t max_num);

/**
 ****************************************************************************************
 * @brief Get the device information according to param type.
 * @note  This API is asynchronous.
 * @note  Once the requested parameters has been got, the event @ref BLE_GAPM_EVT_DEV_INFO_GOT will be called.
 *
 * @param[in] type: Param type.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 ****************************************************************************************
 */
uint16_t ble_gap_dev_info_get(ble_gap_dev_info_get_type_t type);

/**
 ****************************************************************************************
 * @brief Set parameters for advertising. Note that this function must be called prior to advertising started.
 * @param[in] adv_idx: Advertising index, range is 0 to 4.
 * @param[in] own_addr_type: Own BD address source of the local device.
 * @param[in] p_adv_param:   The advertising parameters.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_INVALID_ADV_IDX: Invalid advertising index supplied.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 * 

 * @note Discovery mode param contains Flags AD type, setting adv data should not set the Flags AD type. 
 * @note This API is for legacy advertsing (BT Core Spec version <= 4.2).
 ****************************************************************************************
 */
uint16_t ble_gap_adv_param_set(uint8_t adv_idx, ble_gap_own_addr_t own_addr_type, ble_gap_adv_param_t* p_adv_param);

/**
 ****************************************************************************************
 * @brief Set extended parameters for advertising. Note that this function must be called prior to advertising.
 * @param[in] adv_idx:       Advertising index, range is 0 to 4.
 * @param[in] own_addr_type: Own BD address source of the local device.
 * @param[in] p_adv_param:   The advertising parameters.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_INVALID_ADV_IDX: Invalid advertising index supplied.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 * @note This API is for extended and periodic advertising (BT Core Spec version >= 5.0).
 ****************************************************************************************
 */
uint16_t ble_gap_ext_adv_param_set(uint8_t adv_idx, ble_gap_own_addr_t own_addr_type, ble_gap_ext_adv_param_t* p_adv_param);

/**
 ****************************************************************************************
 * @brief Set Advertising Data, Scan Response Data and Periodic Advertising Data. See ENUM @ref ble_gap_ad_type_t
          for ADV Type definitions. See ENUM @ref ble_gap_adv_flags_t for ADV flag definitions.
 *
 * @param[in] adv_idx: Advertising index, range is 0 to 4.
 * @param[in] type:    Data type.
 * @param[in] p_data:  The data pointer.
 * @param[in] length:  Data length.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_INVALID_ADV_IDX: Invalid advertising index supplied.
 * @retval SDK_ERR_INVALID_ADV_DATA_TYPE: Invalid advertising data type supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 * @note User should not add extra Flags AD type as the BLE Stack has added the Flags AD type already.
 ****************************************************************************************
 */
uint16_t ble_gap_adv_data_set(uint8_t adv_idx, ble_gap_adv_data_type_t type, const uint8_t* p_data, uint16_t length);

/**
 ****************************************************************************************
 * @brief Update Advertising Data, Scan Response Data and Periodic Advertising Data. See ENUM @ref ble_gap_ad_type_t
 *        for ADV Type definitions. See ENUM @ref ble_gap_adv_flags_t for ADV flag definitions.
 *
 * @param[in] adv_idx: Advertising index, range is 0 to 4.
 * @param[in] type:    Data type.
 * @param[in] p_data:  The data pointer.
 * @param[in] length:  Data length.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_INVALID_ADV_IDX: Invalid advertising index supplied.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 * @note User should not add extra Flags AD type as the BLE Stack has added the Flags AD type already.
 ****************************************************************************************
 */
uint16_t ble_gap_update_adv_data(uint8_t adv_idx, ble_gap_adv_data_type_t type, const uint8_t* p_data, uint16_t length);

/**
 ****************************************************************************************
 * @brief Start advertising.
 * @param[in] adv_idx:   Advertising index, range is 0 to 4.
 * @param[in] p_timeout: Advertising timing parameter.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_INVALID_ADV_IDX: Invalid advertising index supplied.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 * @note If advertising mode is directed high duty cycle mode, duration should be set nonzero and not be greater than 1.28s. 
 * @note This API is asynchronous. 
 * @note If the field scan_req_ind_en in @ref ble_gap_adv_param_t  is TRUE passed in @ref ble_gap_adv_param_set function, or if the 
         GAP_ADV_PROP_SCAN_REQ_NTF_EN_BIT is set in @ref ble_gap_adv_prop_t and passed in @ref ble_gap_ext_adv_param_set function,
         @ref BLE_GAPM_EVT_SCAN_REQUEST event will be triggered once a scan request has been received.
 * @note @ref BLE_GAPM_EVT_ADV_START event will be triggered once the operation has completed.
 ****************************************************************************************
 */
uint16_t ble_gap_adv_start(uint8_t adv_idx, ble_gap_adv_time_param_t* p_timeout);

/**
 ****************************************************************************************
 * @brief Stop advertising.
 * @param[in] adv_idx: Advertising index, range is 0 to 4.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_ADV_IDX: Invalid advertising index supplied.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 *
 * @note This API is asynchronous. 
 * @note event @ref BLE_GAPM_EVT_ADV_STOP will be triggered once the operation has completed or advertising has been stopped.
 ****************************************************************************************
 */
uint16_t ble_gap_adv_stop(uint8_t adv_idx);

/**
 ****************************************************************************************
 * @brief Set parameters for scanning. Note that this function must be called prior to scanning started.
 * @param[in] own_addr_type: Own BD address source of the local device.
 * @param[in] p_scan_param:  The scanning parameters.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 * 
 * @note This API is for legacy scanning (BT Core Spec version <= 4.2).
 ****************************************************************************************
 */
uint16_t ble_gap_scan_param_set(ble_gap_own_addr_t own_addr_type, ble_gap_scan_param_t* p_scan_param);

/**
 ****************************************************************************************
 * @brief Set extended parameters for scanning. Note that this function must be called prior to scanning started.
 * @param[in] own_addr_type: Own BD address source of the local device.
 * @param[in] p_scan_param:  The scanning parameters.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 * @note This API is for extended scanning (BT Core Spec version >= 5.0).
 *       The Scan window in p_scan_param should be enough to recieve one packet. For example.
 *       If you want to recieve packects with 1270 bytes on coded phy(S8), the scan_wd should be greater than 82ms.
 *       If you want to recieve packects with 1270 bytes on coded phy(S2), the scan_wd should be greater than 21ms.
 ****************************************************************************************
 */
uint16_t ble_gap_ext_scan_param_set(ble_gap_own_addr_t own_addr_type, ble_gap_ext_scan_param_t* p_scan_param);

/**
 ****************************************************************************************
 * @brief Start scanning
 * @note  This API is asynchronous. 
 * @note  Once the advertising report has been received, the event @ref BLE_GAPM_EVT_ADV_REPORT will be called.
 * @note  Once the scanning has been stopped, the event @ref BLE_GAPM_EVT_SCAN_STOP will be called.
 * @note  Once the operation has completed, the event @ref BLE_GAPM_EVT_SCAN_START will be called.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 *
 ****************************************************************************************
 */
uint16_t ble_gap_scan_start(void);

/**
 ****************************************************************************************
 * @brief Stop scanning
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 *
 * @note This API is asynchronous. 
 * @note event @ref BLE_GAPM_EVT_SCAN_STOP will be triggered once the operation has completed or the scanning has been stopped.
 ****************************************************************************************
 */
uint16_t ble_gap_scan_stop(void);

/**
 ****************************************************************************************
 * @brief Set the parameters used for periodic sync.
 * @param[in] per_sync_idx: Periodic synchronization index (range is 0 to 4).
 * @param[in] p_per_sync_param: Periodic synchronization parameters.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_INVALID_PER_SYNC_IDX: Invalid periodic syncronization index supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_per_sync_param_set(uint8_t per_sync_idx, ble_gap_per_sync_param_t* p_per_sync_param);

/**
 ****************************************************************************************
 * @brief Start to synchronize with periodic advertising from an advertiser and begin receiving periodic advertising packets.
 * @param[in] per_sync_idx: Periodic synchronization index (range is 0 to 4).
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_INVALID_PER_SYNC_IDX: Invalid periodic syncronization index supplied.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 *
 * @note This API is asynchronous. 
 * @note event @ref BLE_GAPM_EVT_SYNC_ESTABLISH will be triggered once the periodic advertising synchronization has been established.
 ****************************************************************************************
 */
uint16_t ble_gap_per_sync_start(uint8_t per_sync_idx);

/**
 ****************************************************************************************
 * @brief Stop periodic synchronization.
 * @param[in] per_sync_idx: Periodic synchronization index (range is 0 to 4).
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PER_SYNC_IDX: Invalid periodic syncronization index supplied.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 *
 * @note This API is asynchronous. 
 * @note event @ref BLE_GAPM_EVT_SYNC_STOP will be triggered once the operation has completed.
 ****************************************************************************************
 */
uint16_t ble_gap_per_sync_stop(uint8_t per_sync_idx);

/**
 ****************************************************************************************
 * @brief Start a legacy connection to a device.
 * @param[in] own_addr_type: Own BD address source of the local device.
 * @param[in] p_init_param:  Initiate parameters.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 * 
 * @note This API is for legacy connection (BT Core Spec version <= 4.2).
 *
 * @note This API is asynchronous. 
 * @note event @ref BLE_GAPC_EVT_CONNECTED will be triggered once the operation has completed or the connection has been completed. 
 * @note event @ref BLE_GAPC_EVT_PEER_NAME_GOT will be triggered once the peer name has been got for the 
         Name Discovery Procedure. See enum @ref BLE_GAP_INIT_TYPE_NAME_DISC of type @ref ble_gap_init_type_t.
 ****************************************************************************************
 */
uint16_t ble_gap_connect(ble_gap_own_addr_t own_addr_type, ble_gap_init_param_t* p_init_param);

/**
 ****************************************************************************************
 * @brief Start an extended connection to a device.
 * @param[in] own_addr_type: Own BD address source of the local device.
 * @param[in] p_init_param:  Initiate parameters.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 * 
 * @note This API is for extended connection (BT Core Spec version >= 5.0).
 *
 * @note This API is asynchronous. 
 * @note event @ref BLE_GAPC_EVT_CONNECTED will be triggered once the operation has completed or the connection has been completed.
 ****************************************************************************************
 */
uint16_t ble_gap_ext_connect(ble_gap_own_addr_t own_addr_type, ble_gap_ext_init_param_t* p_init_param);

/**
 ****************************************************************************************
 * @brief Cancel an initiated connection.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 *
 * @note This API is asynchronous. 
 * @note event @ref BLE_GAPC_EVT_CONNECT_CANCLE will be triggered once the operation has completed.
 ****************************************************************************************
 */
uint16_t ble_gap_connect_cancel(void);

/**
 ****************************************************************************************
 * @brief Set GAP appearance value.
 * @param[in] appearance: Appearance value.
 ****************************************************************************************
 */
void ble_gap_appearance_set(uint16_t appearance);

/**
 ****************************************************************************************
 * @brief Get GAP appearance value.
 * @param[out] p_appearance:  The pointer to appearance value.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_appearance_get(uint16_t *p_appearance);

/**
 ****************************************************************************************
 * @brief Set GAP Peripheral Preferred Connection Parameters.
 * @param[in] p_conn_params: The pointer to PPCP values.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_ppcp_set(ble_gap_conn_param_t const *p_conn_params);

/**
 ****************************************************************************************
 * @brief Get GAP Peripheral Preferred Connection Parameters.
 * @param[out] p_conn_params: The pointer to PPCP values.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_ppcp_get(ble_gap_conn_param_t *p_conn_params);

/**
 ****************************************************************************************
 * @brief Set GAP device name.
 * @param[in] write_perm: Write permissions of the device name characteristic.
 * @param[in] p_dev_name: The pointer to device name value. If p_dev_name is NULL, this function will only set write permissions of the device name.
 * @param[in] length: Device name length.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_device_name_set(ble_gap_dev_name_write_perm_t write_perm, uint8_t const *p_dev_name, uint16_t length);

/**
 ****************************************************************************************
 * @brief Get GAP device name.
 * @param[out] p_dev_name: The pointer to device name value, set to NULL to obtain the complete device name length.
 * @param[in, out] p_length:   Length of the buffer pointed by p_dev_name, complete device name length on output.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_INVALID_DATA_LENGTH: Invalid data size(s) supplied.
 *
 * @note If device name was not set, the default device name "GOODIX_BLE" will be available.
 * @note If the device name is longer than the size of the supplied buffer,p_len will return the complete device name length,
 *        and not the number of bytes actually returned in p_dev_name.
 *        The application may use this information to allocate a suitable buffer size.
 ****************************************************************************************
 */
uint16_t ble_gap_device_name_get(uint8_t *p_dev_name, uint16_t *p_length);

/**
 ****************************************************************************************
 * @brief Register a LE Protocol/Service Multiplexer.
 * @param[in] p_lepsm: The lepsm information to be registerred.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_INVALID_PSM_NUM: Invalid psm number.
 * @retval SDK_ERR_INVALID_PSM_EXCEEDED_MAX_PSM_NUM: The maximum psm number limit is exceeded.
 * @retval SDK_ERR_INVALID_PSM_ALREADY_REGISTERED: The psm number has been registered.
 *
 * @note This API is asynchronous. 
 *       @ref BLE_GAPM_EVT_LEPSM_REGISTER event with op_id: @ref BLE_GAP_OPCODE_LEPSM_REGISTER will be triggered.
 ****************************************************************************************
 */
uint16_t ble_gap_lepsm_register(ble_gap_lepsm_register_t* p_lepsm);

/**
 ****************************************************************************************
 * @brief Unregister a LE Protocol/Service Multiplexer.
 * @param[in] le_psm:  LE Protocol/Service Multiplexer (1 to 255).
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PSM_NUM: Invalid psm number.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 * @note This API is asynchronous. 
 *       @ref BLE_GAPM_EVT_LEPSM_UNREGISTER event with op_id: @ref BLE_GAP_OPCODE_LEPSM_UNREGISTER will be triggered.  
 ****************************************************************************************
 */
uint16_t ble_gap_lepsm_unregister(uint16_t le_psm);

/**
 ****************************************************************************************
 * @brief Set privacy mode for peer device.
 * @param[in] peer_addr: The peer address.
 * @param[in] mode: Privacy mode (see @ref ble_gap_privacy_mode_t).
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 *
 ****************************************************************************************
 */
uint16_t ble_gap_privacy_mode_set(ble_gap_bdaddr_t peer_addr, ble_gap_privacy_mode_t mode);

/**
 ****************************************************************************************
 * @brief Read peer or local resolvable address.
 * @param[in] op_code: The operation code (see @ref ble_gap_rslv_addr_read_op_id_t).
 * @param[in] peer_iden_addr: The peer identity address.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 *
 ****************************************************************************************
 */
uint16_t ble_gap_rslv_addr_read(ble_gap_rslv_addr_read_op_id_t op_code, ble_gap_bdaddr_t peer_iden_addr);

/**
 ****************************************************************************************
 * @brief Control periodic advertising report.
 * @param[in] per_sync_index: The index of the periodic syncronization instance.
 * @param[in] enable_flag: Per adv report flag.
 *            bit0: Reporting enabled(true report or false not report).
 *            bit1: Duplicate filtering enabled.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_per_adv_report_ctrl(uint8_t per_sync_index, bool enable_flag);

/**
 ****************************************************************************************
 * @brief Set connectionless CTE transmit parameters.
 * @param[in] per_adv_index: The index of the periodic advertising.
 * @param[in] param: Connectionless CTE transmit arameters.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_ADV_IDX: Invalid advertising index supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_connless_cte_trans_param_set(uint8_t per_adv_index, gap_connless_cte_trans_param_t *param);

/**
 ****************************************************************************************
 * @brief Set connectionless CTE transmit enable.
 * @param[in] per_adv_index: The index of the periodic advertising.
 * @param[in] enable_flag: Whether enable to transmit connectionless cte.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_ADV_IDX: Invalid advertising index supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_connless_cte_trans_enable_set(uint8_t per_adv_index, bool enable_flag);

/**
 ****************************************************************************************
 * @brief Set connectionless IQ sampling enable.
 * @param[in] per_sync_index: The index of the periodic syncronization instance.
 * @param[in] enable_flag:    Whether enable connectionless iq sampling. if enable_flag is false, the param shall be NULL.
 * @param[in] param:          Enable connectionless iq sampling parameters (see @ref ble_gap_connless_iq_sample_param_t).
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval SDK_ERR_INVALID_PER_SYNC_IDX: Invalid periodic synchronization index supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_connless_iq_sample_enable_set(uint8_t per_sync_index, bool enable_flag, ble_gap_connless_iq_sample_param_t *param);

/**
 ****************************************************************************************
 * @brief Register connectionless iq report event callback.
 *
 * @param[in] cb: The callback for connectionless IQ report event.
 *
 * @retval  void.
 ****************************************************************************************
 */
void ble_gap_register_conless_iq_report_cb(void (*cb)(uint8_t per_sync_idx, const ble_gap_connless_iq_report_t *iq_report));

/**
 ****************************************************************************************
 * @brief Set ranging parameter.
 *
 * @param[in] base_addr: The base address.
 * @param[in] cte_len: The length for cte.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_set_gdx_ranging_param(uint32_t base_addr, uint8_t cte_len);

/** @} */

#endif
/** @} */

/** @} */
/** @} */

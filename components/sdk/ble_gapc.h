/**
 ****************************************************************************************
 *
 * @file ble_gapc.h
 *
 * @brief BLE GAPC API
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
 
 /**
 * @defgroup BLE_GAPC Generic Access Profile (GAP) Connection Control
 * @{
 * @brief Definitions and prototypes for the GAP Connection Control interface.
 */
#ifndef __BLE_GAPC_H__
#define __BLE_GAPC_H__

#include "ble_error.h"
#include "gr55xx_sys_cfg.h"
#include <stdint.h>         // Standard Integer
#include <string.h>
#include <stdbool.h>

/**
 * @defgroup  BLE_GAPC_DEFINES Defines
 * @{
 */
#define BLE_GAP_CHNL_MAP_LEN         0x05 /**< The length of channel map. */
#define BLE_GAP_FEATS_LEN            0x08 /**< The length of features. */
#define BLE_GAP_ADDR_LEN             0x06 /**< The length of address. */
#define BLE_GAP_INVALID_CONN_INDEX   0xFF /**< Invalid connection index. */

/// CTE length (in number of 8us periods)
#define BLE_GAP_MIN_CTE_LEN          0x02 /**< The minimum CTE length. */
#define BLE_GAP_MAX_CTE_LEN          0x14 /**< The maximum CTE length. */

/// CTE count
#define BLE_GAP_MIN_CTE_CNT          0x01 /**< The minimum CTE count. */
#define BLE_GAP_MAX_CTE_CNT          0x10 /**< The maximum CTE count. */

#define BLE_GAP_MIN_IQ_SAMPLE_NUM    0x09 /**< The minimum IQ sample number. */
#define BLE_GAP_MAX_IQ_SAMPLE_NUM    0x52 /**< The maximum IQ sample number. */

#define BLE_GAP_MAX_GDX_RANGING_CH    20  /**< The maximum GDX Ranging Channel. */

/** @} */

/**
 * @defgroup BLE_SDK_GAPC_ENUM Enumerations
 * @{
 */

/** @brief The identity address type */
typedef enum
{
    BLE_GAP_ADDR_TYPE_PUBLIC = 0,      /**< Public (identity) address.*/
    BLE_GAP_ADDR_TYPE_RANDOM_STATIC,   /**< Random static (identity) address. */
} ble_gap_addr_type_t;

/** @brief The phy options */
typedef enum
{
    BLE_GAP_PHY_OPT_NO_CODING = 0, /**< The Host has no preferred coding when transmitting on the LE Coded PHY. */
    BLE_GAP_PHY_OPT_S2_CODING,     /**< The Host prefers that S=2 coding be used when transmitting on the LE Coded PHY. */
    BLE_GAP_PHY_OPT_S8_CODING,     /**< The Host prefers that S=8 coding be used when transmitting on the LE Coded PHY. */
} ble_gap_phy_options_t;

/** @brief The prefer phy type */
typedef enum
{
    GAP_PHY_ANY       = 0x00,      /**< No preferred PHY. */
    GAP_PHY_LE_1MBPS  = (1 << 0),  /**< LE 1M PHY preferred for an active link. */
    GAP_PHY_LE_2MBPS  = (1 << 1),  /**< LE 2M PHY preferred for an active link. */
    GAP_PHY_LE_CODED  = (1 << 2),  /**< LE Coded PHY preferred for an active link. */
} ble_gap_prefer_phy_t;

/** @brief The operation code used to get connection info */
typedef enum  
{
    BLE_GAP_GET_CON_RSSI = 0,        /**< Get connection RSSI info. */
    BLE_GAP_GET_CON_CHANNEL_MAP,     /**< Get connection channel map. */
    BLE_GAP_GET_PHY,                 /**< Get connection PHY. */
    BLE_GAP_GET_CHAN_SEL_ALGO        /**< Get selection algorithm for connection channel. */
} ble_gap_get_conn_info_op_t;

/**@brief The operation code used to get peer device info. */
typedef enum 
{
    BLE_GAP_GET_PEER_VERSION = 0,    /**< Get peer device version info. */
    BLE_GAP_GET_PEER_FEATURES        /**< Get peer device features info. */
} ble_gap_get_peer_info_op_t;

/** @brief Device role of LL layer type */
typedef enum
{
    BLE_GAP_LL_ROLE_MASTER = 0,                  /**< Master role. */
    BLE_GAP_LL_ROLE_SLAVE  = 1,                  /**< Slave role. */
} ble_gap_ll_role_type_t;

/**
 * @brief Operation code used to set param(s).
 */
typedef enum
{
    BLE_GAP_OPCODE_CHNL_MAP_SET,            /**< Set Channel Map. */
    BLE_GAP_OPCODE_WHITELIST_SET,           /**< Set white list. */
    BLE_GAP_OPCODE_PER_ADV_LIST_SET,        /**< Set periodic advertising list. */
    BLE_GAP_OPCODE_PRIVACY_MODE_SET,        /**< Set privacy mode for peer device. */
} ble_gap_param_set_op_id_t;

/**
 * @brief The specified reason for terminating a connection.
 */
typedef enum
{
    BLE_GAP_HCI_AUTHENTICATION_FAILURE                          = 0x05, /**< Authentication Failure. */
    BLE_GAP_HCI_REMOTE_USER_TERMINATED_CONNECTION               = 0x13, /**< Remote User Terminated Connection. */
    BLE_GAP_HCI_REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES     = 0x14, /**< Remote Device Terminated Connection due to Low Resources. */
    BLE_GAP_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF         = 0x15, /**< Remote Device Terminated Connection due to Power Off. */
    BLE_GAP_HCI_UNSUPPORTED_REMOTE_FEATURE                      = 0x1A, /**< Unsupported Remote Feature. */
    BLE_GAP_HCI_PAIRING_WITH_UNIT_KEY_UNSUPPORTED               = 0X29, /**< Pairing With Unit Key Not Supported. */
    BLE_GAP_HCI_CONN_INTERVAL_UNACCEPTABLE                      = 0x3B, /**< Unacceptable Connection Parameters. */
} ble_gap_disconn_reason_t;

/**
 * @brief Operation code used for LEPSM manager.
 */
typedef enum
{
    BLE_GAP_OPCODE_LEPSM_REGISTER,      /**< LEPSM register operation. */
    BLE_GAP_OPCODE_LEPSM_UNREGISTER,    /**< LEPSM unregister operation. */
} ble_gap_psm_manager_op_id_t;

/** @brief GAP Device inforamtion write indication. */
typedef enum
{
    BLE_GAPC_DEV_NAME,              /* Device name type*/
    BLE_GAPC_DEV_APPEARANCE,        /* Device Appearance Icon type*/
} ble_gap_dev_info_type_t;

/**
 * @brief Type of constant tone extension.
 */
typedef enum
{
    BLE_GAP_CTE_TYPE_AOA         = 0x01 << 0,   /**< Allow AoA Constant Tone Extension Response. */
    BLE_GAP_CTE_TYPE_AOD_1US     = 0x01 << 1,   /**< Allow AoD Constant Tone Extension Response with 1us slots. */
    BLE_GAP_CTE_TYPE_AOD_2US     = 0x01 << 2,   /**< Allow AoD Constant Tone Extension Response with 2us slots. */
} ble_gap_cte_type_t;

/**
 * @brief Type of switching and sampling slots 
 */
typedef enum
{
    BLE_GAP_SLOT_1US = 0x01,     /**< Switching and sampling slots are 1us each. */
    BLE_GAP_SLOT_2US,            /**< Switching and sampling slots are 2us each. */
} ble_gap_switching_sampling_type_t;

/**
 * @brief Status of IQ report packet
 */
typedef enum
{
    BLE_GAP_CRC_OK,                      /**< CRC was correct. */
    BLE_GAP_CRC_ERR1,                    /**< CRC was incorrect and the Length and CTETime fields of the packet were used to determine sampling points. */
    BLE_GAP_CRC_ERR2,                    /**< CRC was incorrect but the Controller has determined the position and length of the Constant Tone Extension in some other way. */
    BLE_GAP_INSUFFI_RESOURCE = 0xFF     /**< Insufficient resources to sample (data_channel_idx, cte_type, and slot_dur invalid). */
} ble_gap_iq_report_status_t;

/**
 * @brief Phy for power control management 
 */
 typedef enum
{
    BLE_GAP_PHY_1M       = 0x01,        /**< LE 1M PHY. */
    BLE_GAP_PHY_2M       = 0x02,        /**< LE 2M PHY. */
    BLE_GAP_PHY_CODED_S8 = 0x03,        /**< LE Coded PHY with S=8 data coding. */
    BLE_GAP_PHY_CODED_S2 = 0x04         /**< LE Coded PHY with S=2 data coding. */
} ble_gap_phy_type_t;

/**
 * @brief Transmit power change reporting reason.
 */
typedef enum
{
    BLE_GAP_PWR_LOCAL_TX_CHG   = 0x00, /**< Local transmit power changed. */
    BLE_GAP_PWR_REMOTE_TX_CHG  = 0x01, /**< Remote transmit power changed. */
} ble_gap_tx_pwr_change_report_reason_t;

/**
 * @brief Transmit Power level flag.
 */
typedef enum
{
    BLE_GAP_PWR_MID_LVL  = 0x00, /**< Transmit power level is between minimum and max level. */
    BLE_GAP_PWR_MIN_LVL  = 0x01, /**< Transmit power level is at minimum level. */
    BLE_GAP_PWR_MAX_LVL  = 0x02  /**< Transmit power level is at maximum level. */
} ble_gap_pwr_lvl_flag_t;

/// Path Loss zones. HCI:7.8.118
typedef enum
{
    BLE_GAP_PATH_LOSS_LOW           = 0x00, /**< Entered Low zone. */
    BLE_GAP_PATH_LOSS_MID           = 0x01, /**< Entered Middle zone. */
    BLE_GAP_PATH_LOSS_HIGH          = 0x02, /**< Entered High zone. */
} ble_gap_path_loss_zone_t;

/** @} */


/**
 * @defgroup BLE_GAPC_STRUCT Structures
 * @{
 */

/** @brief The struct of device version. */
typedef struct
{
    uint8_t  hci_ver;      /**< HCI version. */
    uint8_t  lmp_ver;      /**< LMP version. */
    uint8_t  host_ver;     /**< Host version. */
    uint16_t hci_subver;   /**< HCI subversion. */
    uint16_t lmp_subver;   /**< LMP subversion. */
    uint16_t host_subver;  /**< Host subversion. */
    uint16_t manuf_name;   /**< Manufacturer name. */
} ble_gap_dev_version_ind_t;

/** @brief The struct of address. */
typedef struct
{
    uint8_t  addr[BLE_GAP_ADDR_LEN]; /**< 6-byte array address value. */
} ble_gap_addr_t;

/** @brief The struct of broadcast address with broadcast type. */
typedef struct
{
    ble_gap_addr_t gap_addr;     /**< Device BD Address. */
    uint8_t        addr_type;    /**< Address type of the device: 0=public/1=random. please @ref ble_gap_addr_type_t. */
} ble_gap_bdaddr_t;

/** @brief Get broadcast address struct. */
typedef struct
{
    uint8_t     index;        /**< Advertsing index. The valid range is: 0 - 4. */
    ble_gap_bdaddr_t bd_addr;     /**< BD address. */
} ble_gap_get_bd_addr_t;

/** @brief TX power info struct. */
typedef struct
{
    int8_t     power_lvl;       /**< Advertising channel TX power level. Range: -20 to 10. Unit: dBm. Accuracy: +/-4dB. */
} ble_gap_dev_adv_tx_power_t;

/** @brief TX power info struct. */
typedef struct
{
    int8_t min_tx_pwr;      /**< MIN of TX power. Size: 1 octet (signed integer). Range: -127  to +126. Unit: dBm. */
    int8_t max_tx_pwr;      /**< MAX of TX power. Size: 1 octet (signed integer). Range: -127 to +126. Unit: dBm. */
} ble_gap_dev_tx_power_t;

/** @brief Max data length info struct. */
typedef struct
{
    uint16_t suppted_max_tx_octets; /**< Maximum number of payload octets that the local Controller supports for transmission of a single Link Layer packet on a data connection.
                                         Range: 0x001B-0x00FB (all other values reserved for future use). */
    uint16_t suppted_max_tx_time;   /**< Maximum time, in microseconds, that the local Controller supports for transmission of a single Link Layer packet on a data connection.
                                         Range: 0x0148-0x4290 (all other values reserved for future use). */
    uint16_t suppted_max_rx_octets; /**< Maximum number of payload octets that the local Controller supports for reception of a single Link Layer packet on a data connection.
                                         Range: 0x001B-0x00FB (all other values reserved for future use). */
    uint16_t suppted_max_rx_time;   /**< Maximum time, in microseconds, that the local Controller supports for reception of a single Link Layer packet on a data connection.
                                         Range: 0x0148-0x4290 (all other values reserved for future use). */
} ble_gap_max_data_len_t;

/** @brief Suggested default data length info. */
typedef struct
{
    uint16_t suggted_max_tx_octets; /**< The Host's suggested value for the Controller's maximum transmitted number of payload octets to be used for new connections.
                                         Range: 0x001B-0x00FB (all other values reserved for future use), default: 0x001B */
    uint16_t suggted_max_tx_time;   /**< The Host's suggested value for the Controller's maximum packet transmission time to be used for new connections.
                                         Range: 0x0148-0x4290 (all other values reserved for future use), default: 0x0148*/
} ble_gap_sugg_dflt_data_len_t;

/** @brief Number of available advertising sets info. */
typedef struct
{
    uint8_t nb_adv_sets; /**< Number of available advertising sets. */
} ble_gap_nb_adv_sets_t;

/** @brief Maximum advertising data length info. */
typedef struct
{
    uint16_t length; /**< Maximum advertising data length supported by controller. */
} ble_gap_max_adv_data_len_ind_t;

/** @brief RF path compensation values info. */
typedef struct
{
    uint16_t tx_path_comp; /**< RF TX path compensation. */
    uint16_t rx_path_comp; /**< RF RX path compensation. */
} ble_gap_dev_rf_path_comp_ind_t;

/** @brief antenna information. */
typedef struct
{
    uint8_t     supp_switching_sampl_rates;  /**< Supported switching sampling rates bit field (@see enum gap_switch_sampling_rate). */
    uint8_t     antennae_num;                /**< Number of antennae, range 0x01 to 0x4B. */
    uint8_t     max_switching_pattern_len;   /**< Max length of switching pattern (number of antenna IDs in the pattern), range 0x02 to 0x4B. */
    uint8_t     max_cte_len;                 /**< Max CTE length, range 0x02 to 0x14. */
} ble_gap_antenna_inf_t;

/** @brief Device info. */
typedef union
{
    ble_gap_dev_version_ind_t       dev_version;            /**< Version info. */
    ble_gap_get_bd_addr_t           get_bd_addr;            /**< Device BD address info. */   
    ble_gap_dev_adv_tx_power_t      adv_tx_power;           /**< Advertising TX power info. */
    ble_gap_sugg_dflt_data_len_t    sugg_dflt_data_len;     /**< Suggested default data length info. */
    ble_gap_max_data_len_t          max_data_len;           /**< Suggested  MAX data length info. */
    ble_gap_nb_adv_sets_t           nb_adv_sets;            /**< Number of available advertising sets. */
    ble_gap_max_adv_data_len_ind_t  max_adv_data_len;       /**< Maximum advertising data length info. */
    ble_gap_dev_tx_power_t          dev_tx_power;           /**< Device TX power info. */
    ble_gap_dev_rf_path_comp_ind_t  dev_rf_path_comp;       /**< RF path compensation values. */
    ble_gap_antenna_inf_t           dev_antenna_inf;        /**< Device antenna information. */
} ble_gap_dev_info_t;

/** @brief The parameter of connection. */
typedef  struct
{
     uint16_t interval_min;  /**< Minimum value for the connection interval. This shall be less than or equal to Conn_Interval_Max.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s*/
     uint16_t interval_max;  /**< Maximum value for the connection interval. This shall be greater than or equal to Conn_Interval_Min.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s.*/
     uint16_t slave_latency; /**< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
     uint16_t sup_timeout;   /**< Supervision timeout for the LE link. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
} ble_gap_conn_param_t;


/** @brief The parameter of update connection. */
typedef  struct
{
     uint16_t interval_min;  /**< Minimum value for the connection interval. This shall be less than or equal to Conn_Interval_Max.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s*/
     uint16_t interval_max;  /**< Maximum value for the connection interval. This shall be greater than or equal to Conn_Interval_Min.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s.*/
     uint16_t slave_latency; /**< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
     uint16_t sup_timeout;   /**< Supervision timeout for the LE link. range: 0x000A to 0x0C80, unit: 10 ms, Time range: 100 ms to 32 s. */
     uint16_t ce_len;        /**< The length of connection event needed for this LE connection. Range: 0x0002 to 0xFFFF, unit: 0.625 ms, time Range: 1.25 ms to 40.9 s.
                                  recommended value: 0x0002 for 1M phy, 0x0006 for coded phy*/
} ble_gap_conn_update_param_t;

/** @brief  Channel map structure. */
typedef struct
{
    uint8_t map[BLE_GAP_CHNL_MAP_LEN]; /**< This parameter contains 37 1-bit fields. The nth bit (n is in the range of 0 to 36) contains the value for the link layer channel index n.
                                        Channel n is unused = 0, channel n is used = 1. The most significant bits are reserved for future use.*/
} ble_gap_chnl_map_t;

/** @brief PHY info. */
typedef struct
{
    uint8_t tx_phy; /**< LE PHY for data transmission. @see ble_gap_phy_type_t. */
    uint8_t rx_phy; /**< LE PHY for data reception. @see ble_gap_phy_type_t. */
} ble_gap_le_phy_ind_t;

/** @brief Connection info. */
typedef union
{
    int8_t           rssi;              /**< RSSI. */
    ble_gap_chnl_map_t   chnl_map;      /**< channel map. */
    ble_gap_le_phy_ind_t phy;           /**< PHY indicaiton. */
    uint8_t          chan_sel_algo;     /**< Chanel Selection algorithm, 0x00: LE Channel Selection Algorithm #1 is used.
                                             0x01: LE Channel Selection Algorithm #2 is used.\n 0x02-0xFF: reserved. */
} ble_gap_conn_info_t;

/** @brief Peer version info. */
typedef struct 
{
    uint16_t compid;        /**<Manufacturer name. */
    uint16_t lmp_subvers;   /**< LMP subversion. */
    uint8_t  lmp_vers;      /**< LMP version. */
} ble_gap_peer_version_ind_t;


/** @brief LE features info. */
typedef struct
{
    uint8_t features[BLE_GAP_FEATS_LEN]; /**< 8-byte array for LE features\n 
                                          Feature Setting field's bit mapping to Controller Features (0: not support, 1: support) \n
                                                          |Bit position       | Link Layer Feature|
                                                          |-------------|-----------------|
                                                          |0                    | LE Encryption|
                                                          |1                    |Connection Parameters Request Procedure| 
                                                          |2                    |Extended Reject Indication|
                                                          |3                    | Slave-initiated Features Exchange | 
                                                          |4                    |LE Ping | 
                                                          |5                    |LE Data Packet Length Extension | 
                                                          |6                    |LL Privacy |  
                                                          |7                    |Extended Scanner Filter Policies | 
                                                          |8                    |LE 2M PHY|  
                                                          |9                    | Stable Modulation Index - Transmitter | 
                                                          |10                   | Stable Modulation Index - Receiver |
                                                          |11                   |LE Coded PHY | 
                                                          |12                   |LE Extended Advertising| 
                                                          |13                   | LE Periodic Advertising| 
                                                          |14                   | Channel Selection Algorithm #2| 
                                                          |15                   |LE Power Class 1|
                                                          |16                   |Minimum Number of Used Channels Procedure|
                                                          |17                   |Connection CTE Request|
                                                          |18                   |Connection CTE Response|
                                                          |19                   |Connectionless CTE Transmitter|
                                                          |20                   |Connectionless CTE Receiver|
                                                          |21                   |Antenna Switching During CTE Transmission(AoD)|
                                                          |22                   |Antenna Switching During CTE Reception(AoA)|
                                                          |23                   |Receiving Constant Tone Extensions|
                                                          |24                   |Periodic Advertising Sync Transfer - Sender|
                                                          |25                   |Periodic Advertising Sync Transfer - Recipient|
                                                          |26                   |Sleep Clock Accuracy Updates|
                                                          |27                   |Remote Public Key Validation|
                                                          |33                   |LE Power Control Request|
                                                          |34                   |LE Power Change Indication|
                                                          |35                   |LE Path Loss Monitoring|
                                                          
                                                          |All other values |Reserved for Future Use|*/
} ble_gap_peer_features_ind_t;

/** @brief LE peer info. */
typedef union
{
    ble_gap_peer_version_ind_t  peer_version;   /**< Version info. */
    ble_gap_peer_features_ind_t peer_features;  /**< Features info. */
} ble_gap_peer_info_t;

/**@brief The Structure for BLE Connection Arrangement. */
typedef struct
{
    uint16_t conn_idx;     /**< Connection Index. */
    uint32_t interval;     /**< Connection Interval (in 312.5 us). */
    uint32_t offset;       /**< Connection Offset (in 312.5 us). */
    uint32_t duration;     /**< Connection Duration (in 312.5 us). */
} ble_gap_con_plan_tag_t;

/** @brief Set preference slave event duration */
typedef struct
{
    uint16_t duration; /**< Preferred event duration. */
    uint8_t  single_tx; /**< Slave transmits a single packet per connection event (False/True). */
} ble_gap_set_pref_slave_evt_dur_param_t;

/** @brief GAP Device name struct. */
typedef struct 
{
    uint16_t length;                /**< Device name length. */
    uint8_t value[__ARRAY_EMPTY];   /**< Device name data. */
} ble_gap_dev_name_ind_t;

/** @brief Device information data struct. */
typedef union
{
    ble_gap_dev_name_ind_t dev_name;    /**< Device name. see @ref ble_gap_dev_name_ind_t. */
    uint16_t               appearance;  /**< Device appearance */
} ble_gapc_set_dev_info_t;


/** @brief GAP Device inforamtion write indication. */
typedef struct
{
    ble_gap_dev_info_type_t info_type; /**< Device info type. see @ref ble_gap_dev_info_type_t. */
    ble_gapc_set_dev_info_t info;      /**< Device info data. see @ref ble_gap_cte_type_t. */
} ble_gapc_set_dev_info_ind_t;

/**
 * @brief Default periodic advertising synchronization transfer parameters
 */
typedef struct
{
    uint8_t   mode;                     /**< @see gap_per_adv_sync_info_rec_mode. */
    uint16_t  skip;                     /**< Number of periodic advertising that can be skipped after a successful receive. 
                                             Maximum authorized value is 499. */
    uint16_t  sync_to;                  /**< Synchronization timeout for the periodic advertising (in unit of 10 ms between 100 ms and 163.84s). */
    uint8_t   cte_type;                 /**< Type of Constant Tone Extension device should sync on (@see enum gap_sync_cte_type). */
} ble_gap_per_sync_trans_param_t;

/**
 * @brief Connectionless IQ Report info
 */
typedef struct
{
    uint8_t  channel_idx;                         /**< The index of the channel on which the packet was received, range 0x00 to 0x24. */
    int16_t  rssi;                                /**< RSSI units: 0.1 dBm, range -1270 to +200. */
    uint8_t  rssi_antenna_id;                     /**< RSSI antenna ID. */
    uint8_t  cte_type;                            /**< CTE type (0: GAP_CET_AOA | 1: GAP_CET_AOD_1US | 2: GAP_CET_AOD_2US), @see enum ble_gap_cte_type_t. */
    uint8_t  slot_dur;                            /**< Slot durations (1: GAP_SLOT_1US | 2: GAP_SLOT_2US), see @ref ble_gap_switching_sampling_type_t. */
    uint8_t  pkt_status;                          /**< Packet status, @see enum ble_gap_iq_report_status_t. */
    uint16_t pa_evt_cnt;                          /**< Periodic advertising event counter. */
    uint8_t  nb_samples;                          /**< Number of samples. 0x00: no samples provided (only permitted if pkt_status is 0xFF),
                                                       0x09 to 0x52: total number of sample pairs. */
    int8_t i_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of i samples for the reported PDU. */
    int8_t q_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of q samples for the reported PDU. */
} ble_gap_connless_iq_report_t;

/** @brief Set connection CTE transmit parameters info. */
typedef struct
{
    uint8_t cte_type;     /**< The type of cte, see @ref ble_gap_cte_type_t. */
    uint8_t num_antenna;  /**< The number of Antenna IDs in the pattern, range 0x02 to 0x4B. */
    uint8_t *antenna_id;  /**< List of Antenna IDs in the pattern. */
} ble_gap_set_conn_cte_trans_param_t;

/** @brief Set connection CTE receive parameters info. */
typedef struct
{
    bool    sampling_enable; /**< Wheter to sample IQ from the CTE. */
    uint8_t slot_durations;  /**< The slot for sample IQ from the CTE, see @ref ble_gap_switching_sampling_type_t. */
    uint8_t num_antenna;     /**< The number of Antenna IDs in the pattern, range 0x02 to 0x4B. */
    uint8_t *antenna_id;     /**< List of Antenna IDs in the pattern. */
} ble_gap_set_conn_cte_rcv_param_t;

/** @brief Set connection CTE Request enable info. */
typedef struct
{
    uint16_t cte_req_interval;    /**< Defines whether the cte request procedure is initiated only once or periodically.
                                       0x0000: initiate the Constant Tone Extension Request procedure once.
                                       0x0001 to 0xFFFF: requested interval for initiating the cte request procedure in number of connection events. */
    uint8_t  cte_req_len;         /**< Minimum length of the cte being requested in 8us units, range 0x02 to 0x14. */
    uint8_t  cte_req_type;        /**< The type for requested cte, see @ref ble_gap_cte_type_t. */
} ble_gap_set_conn_cte_req_enable_t;

/** @brief Connection IQ Report info. */
typedef struct
{
    uint8_t  rx_phy;                          /**< Rx PHY (0x01: 1M | 0x02: 2M), see @ref ble_gap_phy_type_t. */
    uint8_t  data_channel_idx;                /**< Data channel index, range 0x00 to 0x24. */
    int16_t  rssi;                            /**< RSSI units: 0.1 dBm, range -1270 to +200. */
    uint8_t  rssi_antenna_id;                 /**< RSSI antenna ID. */
    uint8_t  cte_type;                        /**< CTE type (0: GAP_CET_AOA | 1: GAP_CET_AOD_1US | 2: GAP_CET_AOD_2US), @see enum ble_gap_cte_type_t. */
    uint8_t  slot_dur;                        /**< Slot durations (1: GAP_SLOT_1US | 2: GAP_SLOT_2US), see @ref ble_gap_switching_sampling_type_t. */
    uint8_t  pkt_status;                      /**< Packet status, @see enum ble_gap_iq_report_status_t. */
    uint16_t con_evt_cnt;                     /**< Connection event counter. */
    uint8_t  nb_samples;                      /**< Number of samples. 0x00: no samples provided (only permitted if pkt_status is 0xFF),
                                                   0x09 to 0x52: total number of sample pairs. */
    int8_t i_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of i samples for the reported PDU. */
    int8_t q_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of q samples for the reported PDU. */
} ble_gap_conn_iq_report_t;

/** @brief Set path loss reporting parameter info. */
typedef struct
{
    uint8_t  high_thr;                  /**< High threshold for the path loss (dB). */
    uint8_t  high_hyst;                 /**< Hysteresis value for the high threshold (dB). */
    uint8_t  low_thr;                   /**< Low threshold for the path loss (dB). */
    uint8_t  low_hyst;                  /**< Hysteresis value for the low threshold (dB). */
    uint16_t min_conn_evt_num;          /**< Minimum time in number of connection events to be observed. */
} ble_gap_set_path_loss_report_param_t;

/** @brief Transmit power change reporting info. */
typedef struct
{
    uint8_t  reason;    /**< Reason see @ref ble_gap_tx_pwr_change_report_reason_t. */
    uint8_t  phy;       /**< Phy see @ref ble_gap_phy_type_t. */
    int8_t   tx_pwr;    /**< Transmit Power level (dBm). */
    uint8_t  flags;     /**< Transmit Power level flags, see @ref ble_gap_pwr_lvl_flag_t. */
    int8_t   delta;     /**< Change in transmit power level (dBm). */
} ble_gap_tx_pwr_change_report_t;

/** @brief Path loss threshold reporting info. */
typedef struct
{
    uint8_t curr_path_loss;   /**< Current path loss (dB). */
    uint8_t zone_entered;     /**< Zone entered, see @ref ble_gap_path_loss_zone_t. */
} ble_gap_path_loss_threshold_report_t;

/** @brief Local transmit power read indication info. */
typedef struct
{
    uint8_t phy;              /**< Phy see @ref ble_gap_phy_type_t. */
    int8_t  curr_tx_pwr_lvl;  /**< Current transmit power level (dBm). */
    int8_t  max_tx_pwr_lvl;   /**< Max transmit power level (dBm). */
} ble_gap_local_tx_pwr_read_ind_t;

/** @brief Remote transmit power read indication info. */
typedef struct
{
    uint8_t  phy;       /**< Phy see @ref ble_gap_phy_type_t. */
    int8_t   tx_pwr;    /**< Transmit Power level (dBm). */
    uint8_t  flags;     /**< Transmit Power level flags, see @ref ble_gap_pwr_lvl_flag_t. */
} ble_gap_remote_tx_pwr_read_ind_t;

/** @brief ranging parameter. */
typedef struct
{
    /// ranging channel sequence
    uint8_t  channel_sequence[BLE_GAP_MAX_GDX_RANGING_CH];
    /// Number of channel to be collected
    uint8_t  channel_num;
} ble_gap_ranging_param_t;

/** @brief ranging indication info. */
typedef struct
{
    uint8_t  status;    /**< ranging status. */
} ble_gap_ranging_ind_t;

/** @brief ranging sample report info. */
typedef struct
{
    ///Status of ranging sample proc
    uint8_t  status;
    /// sample number
    uint16_t nb_sample;
    /// sample address
    int32_t  iq_sample_addr;
} ble_gap_ranging_sample_report_ind_t;

/** @brief ranging complete info. */
typedef struct
{
    /// indicate ranging complete status
    uint8_t  status;
} ble_gap_ranging_cmp_ind_t;

/** @brief Subrate change indication. */
typedef struct
{
    uint16_t            subrate_factor;   /**< subrate factor value. */
    uint16_t            con_latency;      /**< Connection latency value. */
    uint16_t            continuation_number; /**< Connection continuation number value. */
    uint16_t            supervision_timeout; /**< Connection supervision timeout value. */
} ble_gap_subrate_chg_ind_t;

/** @brief Default Subrate command param. */
typedef struct
{
    uint16_t  subrate_min; /**< Minimum subrate factor allowed in requests by a Peripheral(Range: 0x0001 - 0x01F4, Default: 0x0001). */
    uint16_t  subrate_max; /**< Maximum subrate factor allowed in requests by a Peripheral(Range: 0x0001 - 0x01F4, Default: 0x0001). */
    uint16_t  max_latency; /**< Maximum Peripheral latency allowed in requests by a Peripheral. */
    uint16_t  continuation_num; /**< Minimum number of underlying connection events to remain active. */
    uint16_t  superv_timeout;  /**< Maximum supervision timeout allowed in requests by a Peripheral. */
} ble_gap_dft_subrate_param_t;

/** @brief Subrate Request command param. */
typedef struct
{
    uint16_t  subrate_min;      /**< Minimum subrate factor allowed in requests by a Peripheral(Range: 0x0001 - 0x01F4, Default: 0x0001). */
    uint16_t  subrate_max;      /**< Maximum subrate factor allowed in requests by a Peripheral(Range: 0x0001 - 0x01F4, Default: 0x0001). */
    uint16_t  max_latency;      /**< Maximum Peripheral latency allowed in requests by a Peripheral. */
    uint16_t  continuation_num; /**< Minimum number of underlying connection events to remain active. */
    uint16_t  superv_timeout;   /**< Maximum supervision timeout allowed in requests by a Peripheral. */
} ble_gap_subrate_req_t;

/** @brief APP receives the extended advertising report indication info struct. */
typedef struct
{
    uint8_t          adv_type;              /**< Advertising type. @see enum ble_gap_adv_report_type_t. */
    uint8_t          adv_info;              /**< Bit field providing information about the received report. @see enum ble_gap_adv_report_info_t. */
    ble_gap_bdaddr_t broadcaster_addr;      /**< Broadcaster device address. */
    ble_gap_bdaddr_t direct_addr;           /**< Target address (in case of a directed advertising report). */
    int8_t           tx_pwr;                /**< TX power (in dBm). */
    int8_t           rssi;                  /**< RSSI (between -127 and +20 dBm). */
    uint8_t          phy_prim;              /**< Primary PHY on which advertising report has been received. */
    uint8_t          phy_second;            /**< Secondary PHY on which advertising report has been received. */
    uint8_t          adv_sid;               /**< Advertising SID , valid only for periodic advertising report. */
    uint16_t         period_adv_intv;       /**< Periodic advertising interval (in unit of 1.25ms, min is 7.5ms), valid only for periodic advertising report. */
    uint8_t          per_sync_idx;          /**< Periodic syncronization index, valid only for periodic advertising report. */
    uint16_t         length;                /**< Report length. */
    uint8_t          data[__ARRAY_EMPTY];   /**< Report. */
} ble_gap_ext_adv_report_ind_t;

/** @brief Sync established indication. */
typedef struct
{
    uint8_t          phy;           /**< PHY on which synchronization has been established. @see ble_gap_phy_type_t. */
    uint16_t         intv;          /**< Periodic advertising interval (in unit of 1.25ms, min is 7.5ms). */
    uint8_t          adv_sid;       /**< Advertising SID. */
    uint8_t          clk_acc;       /**< Advertiser clock accuracy. @see ble_gap_clk_acc_t. */
    ble_gap_bdaddr_t bd_addr;       /**< Advertiser address. */
    uint16_t         sync_hdl;      /**< Sync handle. */
    uint16_t         serv_data;     /**< Service data. */
    bool             report_flag;   /**< Report Flag. */
} ble_gap_sync_established_ind_t;

/**@brief PHY update event for @ref BLE_GAPC_EVT_PHY_UPDATED. */
typedef struct
{
    uint8_t     tx_phy;         /**< LE PHY for data transmission. @ref ble_gap_phy_type_t. */
    uint8_t     rx_phy;         /**< LE PHY for data reception. @ref ble_gap_phy_type_t. */
} ble_gap_evt_phy_update_t;

/** @brief  Connection complete event for @ref BLE_GAPC_EVT_CONNECTED. */
typedef struct
{
    uint16_t                conn_handle;            /**< Connection_Handle. Range: 0x0000-0x0EFF (all other values reserved for future use). */
    uint16_t                conn_interval;          /**< Connection interval. Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s. */
    uint16_t                slave_latency;          /**< Latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
    uint16_t                sup_timeout;            /**< Connection supervision timeout. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
    uint8_t                 clk_accuracy;           /**< Clock accuracy (0x00: 500 ppm, 0x01: 250 ppm, 0x02: 150 ppm, 0x03: 100 ppm, 0x04: 75 ppm, 0x05:50 ppm, 0x06:30 ppm, 0x07:20 ppm, others: reserved for future use). */
    uint8_t                 peer_addr_type;         /**< Peer address type(0x00: Public Device Address, 0x01 : Random Device Address, others: reserved for future use). */
    ble_gap_addr_t          peer_addr;              /**< Peer BT address. */
    ble_gap_ll_role_type_t  ll_role;                /**< Device Role of LL Layer. */
} ble_gap_evt_connected_t; 

/**@brief Disconnection event for @ref BLE_GAPC_EVT_DISCONNECTED. */
typedef struct
{
    uint8_t reason;         /**< Hci error code. */
} ble_gap_evt_disconnected_t;

/** @brief  Name of peer device indication event for @ref BLE_GAPC_EVT_PEER_NAME_GOT. */
typedef struct
{
    ble_gap_addr_t  peer_addr;              /**< Peer device bd address. */
    uint8_t         addr_type;              /**< Peer device address type. */
    uint8_t         name_len;               /**< Peer device name length. */
    uint8_t        *name;                   /**< Peer device name. */
} ble_gap_evt_peer_name_get_t;

/** @brief Get peer info event for @ref BLE_GAPC_EVT_PEER_INFO_GOT. */
typedef struct
{
    uint8_t             opcode;         /**< Operation code. See @ref ble_gap_get_peer_info_op_t. */
    ble_gap_peer_info_t peer_info;      /**< Peer info. */
} ble_gap_evt_peer_info_t;

/** @brief Connection parameter updated event for @ref BLE_GAPC_EVT_CONN_PARAM_UPDATED. */
typedef struct
{
    uint16_t conn_interval;             /**< Connection interval. Range: 0x0006 to 0x0C80. Unit: 1.25 ms. Time range: 7.5 ms to 4 s. */
    uint16_t slave_latency;             /**< Latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
    uint16_t sup_timeout;               /**< Supervision timeout for the LE link. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
} ble_gap_evt_conn_param_updated_t;

/** @brief Connection parameter update request event for @ref BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ. */
typedef  struct
{
     uint16_t interval_min;  /**< Minimum value for the connection interval. This shall be less than or equal to Conn_Interval_Max.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s*/
     uint16_t interval_max;  /**< Maximum value for the connection interval. This shall be greater than or equal to Conn_Interval_Min.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s.*/
     uint16_t slave_latency; /**< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
     uint16_t sup_timeout;   /**< Supervision timeout for the LE link. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
} ble_gap_evt_conn_param_update_req_t;

/** @brief Get Connection info event for @ref BLE_GAPC_EVT_CONN_INFO_GOT. */
typedef struct
{
    uint8_t             opcode;     /**< Operation code. See @ref ble_gap_get_conn_info_op_t. */
    ble_gap_conn_info_t info;       /**< Connection info. */
} ble_gap_evt_conn_info_t;

/** @brief Data Length Updated event for @ref BLE_GAPC_EVT_DATA_LENGTH_UPDATED. */
typedef struct
{
    uint16_t max_tx_octets; /**<  The maximum number of payload octets in TX. */
    uint16_t max_tx_time;   /**<  The maximum time that the local Controller will take to TX. */
    uint16_t max_rx_octets; /**<  The maximum number of payload octets in RX. */
    uint16_t max_rx_time;   /**<  The maximum time that the local Controller will take to RX. */
} ble_gap_evt_data_length_t;

/** @brief Device Information set event for @ref BLE_GAPC_EVT_DEV_INFO_SET. */
typedef struct
{
    ble_gap_dev_info_type_t info_type; /**< Device info type. see @ref ble_gap_dev_info_type_t. */
    ble_gapc_set_dev_info_t info;      /**< Device info data. see @ref ble_gap_cte_type_t. */
} ble_gap_evt_dev_info_set_t;

/** @brief Connection IQ Report info event for @ref BLE_GAPC_EVT_CONNECT_IQ_REPORT. */
typedef struct
{
    uint8_t  rx_phy;                              /**< Rx PHY (0x01: 1M | 0x02: 2M), see @ref BLE_GAP_PHYS. */
    uint8_t  data_channel_idx;                    /**< Data channel index, range 0x00 to 0x24. */
    int16_t  rssi;                                /**< RSSI units: 0.1 dBm, range -1270 to +200. */
    uint8_t  rssi_antenna_id;                     /**< RSSI antenna ID. */
    uint8_t  cte_type;                            /**< CTE type (0: GAP_CET_AOA | 1: GAP_CET_AOD_1US | 2: GAP_CET_AOD_2US), @see enum ble_gap_cte_type_t. */
    uint8_t  slot_dur;                            /**< Slot durations (1: GAP_SLOT_1US | 2: GAP_SLOT_2US), see @ref ble_gap_switching_sampling_type_t. */
    uint8_t  pkt_status;                          /**< Packet status, @see enum ble_gap_iq_report_status_t. */
    uint16_t con_evt_cnt;                         /**< Connection event counter. */
    uint8_t  nb_samples;                          /**< Number of samples. 0x00: no samples provided (only permitted if pkt_status is 0xFF),
                                                       0x09 to 0x52: total number of sample pairs. */
    int8_t i_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of i samples for the reported PDU. */
    int8_t q_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of q samples for the reported PDU. */
} ble_gap_evt_conn_iq_report_t;

/** @brief Connectionless IQ Report info event for @ref BLE_GAPC_EVT_CONNECTLESS_IQ_REPORT. */
typedef struct
{
    uint8_t  channel_idx;                         /**< The index of the channel on which the packet was received, range 0x00 to 0x24. */
    int16_t  rssi;                                /**< RSSI units: 0.1 dBm, range -1270 to +200. */
    uint8_t  rssi_antenna_id;                     /**< RSSI antenna ID. */
    uint8_t  cte_type;                            /**< CTE type (0: GAP_CET_AOA | 1: GAP_CET_AOD_1US | 2: GAP_CET_AOD_2US), @see enum ble_gap_cte_type_t. */
    uint8_t  slot_dur;                            /**< Slot durations (1: GAP_SLOT_1US | 2: GAP_SLOT_2US), see @ref ble_gap_switching_sampling_type_t. */
    uint8_t  pkt_status;                          /**< Packet status, @see enum ble_gap_iq_report_status_t. */
    uint16_t pa_evt_cnt;                          /**< Periodic advertising event counter. */
    uint8_t  nb_samples;                          /**< Number of samples. 0x00: no samples provided (only permitted if pkt_status is 0xFF),
                                                       0x09 to 0x52: total number of sample pairs. */
    int8_t i_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of i samples for the reported PDU. */
    int8_t q_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of q samples for the reported PDU. */
} ble_gap_evt_connless_iq_report_t;

/** @brief Local transmit power read indication info event for @ref BLE_GAPC_EVT_LOCAL_TX_POWER_READ. */
typedef struct
{
    uint8_t phy;              /**< Phy see @ref ble_gap_phy_type_t. */
    int8_t  curr_tx_pwr_lvl;  /**< Current transmit power level (dBm). */
    int8_t  max_tx_pwr_lvl;   /**< Max transmit power level (dBm). */
} ble_gap_evt_local_tx_pwr_read_ind_t;

/** @brief Remote transmit power read indication info event for @ref BLE_GAPC_EVT_REMOTE_TX_POWER_READ. */
typedef struct
{
    uint8_t  phy;       /**< Phy see @ref ble_gap_phy_type_t. */
    int8_t   tx_pwr;    /**< Transmit Power level (dBm). */
    uint8_t  flags;     /**< Transmit Power level flags, see @ref ble_gap_pwr_lvl_flag_t. */
} ble_gap_evt_remote_tx_pwr_read_ind_t;

/** @brief Transmit power change reporting info event for @ref BLE_GAPC_EVT_TX_POWER_CHANGE_REPORT. */
typedef struct
{
    uint8_t  reason;    /**< Reason see @ref ble_gap_tx_pwr_change_report_reason_t. */
    uint8_t  phy;       /**< Phy see @ref ble_gap_phy_type_t. */
    int8_t   tx_pwr;    /**< Transmit Power level (dBm). */
    uint8_t  flags;     /**< Transmit Power level flags, see @ref ble_gap_pwr_lvl_flag_t. */
    int8_t   delta;     /**< Change in transmit power level (dBm). */
} ble_gap_evt_tx_pwr_change_report_t;

/** @brief Path loss threshold reporting info event for @ref BLE_GAPC_EVT_PATH_LOSS_THRESHOLD_REPORT. */
typedef struct
{
    uint8_t curr_path_loss;   /**< Current path loss (dB). */
    uint8_t zone_entered;     /**< Zone entered, see @ref ble_gap_path_loss_zone_t. */
} ble_gap_evt_path_loss_threshold_report_t;

/** @brief Ranging indication event for @ref BLE_GAPC_EVT_RANGING_IND. */
typedef struct
{
    uint8_t  ranging_status;    /**< ranging status. */
} ble_gap_evt_ranging_ind_t;

/** @brief Ranging sample report event for @ref BLE_GAPC_EVT_RANGING_SAMPLE_REPORT. */
typedef struct
{
    uint16_t nb_sample;      /**< Sample number. */
    int32_t  iq_sample_addr; /**< I/Q sample address. */
} ble_gap_evt_ranging_sample_report_ind_t;

/** @brief Ranging complete indication event for @ref BLE_GAPC_EVT_RANGING_CMP_IND. */
typedef struct
{
    uint8_t  ranging_status;    /**< ranging complete status. */
} ble_gap_evt_ranging_cmp_ind_t;

/** @brief Subrate change indication event for @ref BLE_GAPC_EVT_SUBRATE_CHANGE_IND. */
typedef struct
{
    uint16_t            subrate_factor;   /**< subrate factor value. */
    uint16_t            con_latency;      /**< Connection latency value. */
    uint16_t            continuation_number; /**< Connection continuation number value. */
    uint16_t            supervision_timeout; /**< Connection supervision timeout value. */
} ble_gap_evt_subrate_chg_ind_t;

/**@brief BLE GAPC event structure. */
typedef struct
{
    uint8_t  index;                                                     /**< Index of connection. */
    union                                                               /**< union alternative identified by evt_id in enclosing struct. */
    {
        ble_gap_evt_phy_update_t                 phy_update;             /**< PHY update parameters. */
        ble_gap_evt_connected_t                  connected;              /**< Connection parameters. */
        ble_gap_evt_disconnected_t               disconnected;           /**< Disconnection parameters. See @ref BLE_STACK_ERROR_CODES. */
        ble_gap_evt_peer_name_get_t              peer_name;              /**< Peer device name indication parameters. */
        ble_gap_evt_peer_info_t                  peer_info;              /**< Peer info indication parameters. */
        ble_gap_evt_conn_param_updated_t         conn_param_updated;     /**< Connection parameter updated parameters. */
        ble_gap_evt_conn_param_update_req_t      conn_param_update_req;  /**< Connection parameter update request parameters. */
        ble_gap_evt_conn_info_t                  conn_info;              /**< Connection info parameters. */
        ble_gap_evt_data_length_t                data_length;            /**< Data Length Update parameter. */                             
        ble_gap_evt_dev_info_set_t               dev_info_ind;           /**< Device info parameters. */
        ble_gap_evt_conn_iq_report_t             conn_iq_report;         /**< Connection IQ Report info parameters. */
        ble_gap_evt_connless_iq_report_t         connless_iq_report;     /**< Connectionless IQ Report info parameters. */
        ble_gap_evt_local_tx_pwr_read_ind_t      local_tx_pwr_read;      /**< Local transmit power read indication info parameters. */
        ble_gap_evt_remote_tx_pwr_read_ind_t     remote_tx_pwr_read;         /**< Remote transmit power read indication info parameters. */
        ble_gap_evt_tx_pwr_change_report_t       tx_pwr_change_report;       /**< Transmit power change reporting info parameters. */
        ble_gap_evt_path_loss_threshold_report_t path_loss_threshold_reoprt; /**< Path loss threshold reporting info parameters. */
        ble_gap_evt_ranging_ind_t                ranging_ind;
        ble_gap_evt_ranging_sample_report_ind_t  ranging_sample_report;
        ble_gap_evt_ranging_cmp_ind_t            ranging_cmp_ind;
        ble_gap_evt_subrate_chg_ind_t            subrate_chg_ind;
    } params;                                                               /**< Event Parameters. */
} ble_gapc_evt_t;

/** @} */

/**
 * @defgroup BLE_GAPC_FUNCTION Functions
 * @{
 */
/**
 ****************************************************************************************
 * @brief Terminate an existing connection.
 *
 * @param[in] conn_idx: The index of connection.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
  */
uint16_t ble_gap_disconnect(uint8_t conn_idx);

/**
 ****************************************************************************************
 * @brief Terminate an existing connection with a specified reason.
 *
 * @param[in] conn_idx: The index of connection.
 * @param[in] reason: The specified reason.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
  */
uint16_t ble_gap_disconnect_with_reason(uint8_t conn_idx, ble_gap_disconn_reason_t reason);

/**
 ****************************************************************************************
 * @brief Change the Link Layer connection parameters of a connection.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] p_conn_param: The new connection param.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_param_update (uint8_t conn_idx, const ble_gap_conn_update_param_t *p_conn_param);

/**
 *****************************************************************************************
 * @brief  Set the method for updating connection parameter.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] use_l2cap_flag: Preferred to use l2cap to update connection parameter.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 *****************************************************************************************
 */
uint16_t ble_gap_update_conn_param_method_set(uint8_t conn_idx, bool use_l2cap_flag);

/**
 *****************************************************************************************
 * @brief Set connection's Latency.
 * @note  The latency shall be set to X value by LLCP firstly, then uses this API to change the latency in [0, X].
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] latency:      The latency of connection.
 *                               
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 *****************************************************************************************
 */
uint16_t ble_gap_latency_set(uint8_t conn_idx, uint16_t latency);

/**
 *****************************************************************************************
 * @brief Get connection's Latency.
 * @note  This function is used to get connection's Latency.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] latency:      Pointer to the latency of connection.
 *                               
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 *****************************************************************************************
 */
uint16_t ble_gap_latency_get(uint8_t conn_idx, uint16_t *latency);

/**
 *****************************************************************************************
 * @brief Consult BLE connection activity plan situation function.
 * @note  This function should be called when connection established and no periodic advertising exists.
 *
 * @param[out] p_act_num:        Pointer to the number of existing connection activities.
 * @param[out] p_conn_plan_arr:  Pointer to the global array that stores planned connection activities.
 *                               
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 *****************************************************************************************
 */
uint16_t ble_gap_con_plan_consult(uint8_t *p_act_num, ble_gap_con_plan_tag_t **p_conn_plan_arr);

/**
 ****************************************************************************************
 * @brief Connection param update reply to peer device.
 *
 * @param[in] conn_idx:      The index of connection.
 * @param[in] accept: True to accept connection parameters, false to reject.
 *
 * @retval ::SDK_SUCCESS: Operation is success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_param_update_reply(uint8_t conn_idx, bool accept);

/**
 ****************************************************************************************
 * @brief The suggested maximum transmission packet size and maximum packet transmission time to be used for a given connection.
 *
 * @param[in] conn_idx:   The index of connection.
 * @param[in] tx_octects: Preferred maximum number of payload octets that the local Controller should include in a single Link Layer packet on this connection.
 *            Range 0x001B-0x00FB (all other values reserved for future use).
 * @param[in] tx_time:    Preferred maximum number of microseconds that the local Controller should use to transmit a single Link Layer packet on this connection.
 *            Range 0x0148-0x4290 (all other values reserved for future use).
 *
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_data_length_update(uint8_t conn_idx,  uint16_t  tx_octects , uint16_t tx_time);

/**
 ****************************************************************************************
 * @brief Set the PHY preferences for the connection identified by the connection index.
 *
 * @param[in] conn_idx:   The index of connection.
 * @param[in] tx_phys: A bit field that indicates the transmitter PHYs that the Host prefers the Controller to use (see @ref ble_gap_prefer_phy_t).
 * @param[in] rx_phys: A bit field that indicates the receiver PHYs that the Host prefers the Controller to use (see @ref ble_gap_prefer_phy_t).
 * @param[in] phy_opt: A bit field that allows the Host to specify options for PHYs (see @ref ble_gap_phy_options_t).
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_phy_update(uint8_t conn_idx, uint8_t tx_phys , uint8_t rx_phys, uint8_t phy_opt);

/**
 ****************************************************************************************
 * @brief Get the information of the connection.
 *
 * @param[in] conn_idx: The index of connection.
 * @param[in] opcode:   The operation code. See @ref ble_gap_get_conn_info_op_t.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_info_get(uint8_t conn_idx, ble_gap_get_conn_info_op_t opcode);

/**
 ****************************************************************************************
 * @brief Get the information of the peer device.
 *
 * @param[in] conn_idx: The index of connection.
 * @param[in] opcode:   The operation code. See @ref ble_gap_get_peer_info_op_t.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_peer_info_get(uint8_t conn_idx, ble_gap_get_peer_info_op_t opcode);

/**
 ****************************************************************************************
 * @brief Get BD address of the bonded device.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] p_peer_addr:  Pointer to the peer BD addrss
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_bond_dev_addr_get(uint8_t conn_idx, ble_gap_bdaddr_t *p_peer_addr);

/**
 ****************************************************************************************
 * @brief Set the parameters used for periodic sync transfer.
 *
 * @param[in] conn_idx: The index of connection.
 * @param[in] per_sync_idx: Periodic synchronization index (range is 0 to 4).
 * @param[in] p_per_sync_trans_param: Periodic synchronization transfer parameters.
 *
 * @retval ::SDK_SUCCESS: Operation is successful.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_DISALLOWED: Operation is disallowed.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_per_sync_trans_param_set(uint8_t conn_idx, uint8_t per_sync_idx, ble_gap_per_sync_trans_param_t* p_per_sync_trans_param);

/**
 ****************************************************************************************
 * @brief Send synchronization information about the periodic advertising in an advertising set to a connected device.
 *
 * @note Need to get the feature of peer device before invoke this function. 
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] per_adv_idx:  The index of per adv.
 * @param[in] service_data: Identify the periodic advertisement to the peer device.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_per_adv_set_info_trans(uint8_t conn_idx, uint8_t per_adv_idx, uint16_t service_data);

/**
 ****************************************************************************************
 * @brief Send synchronization information about the periodic advertising identified by the sync_hdl parameter to a connected device.
 *
 * @param[in] conn_idx:      The index of connection.
 * @param[in] per_sync_idx:  The index of the periodic syncronization instance.
 * @param[in] service_data:  Identify the periodic advertisement to the peer device.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_per_adv_sync_trans(uint8_t conn_idx, uint8_t per_sync_idx, uint16_t service_data);

/**
 ****************************************************************************************
 * @brief Set connection CTE transmit parameters.
 *
 * @param[in] conn_idx:  The index of connection.
 * @param[in] param:     Set connection CTE transmit parameters info, see @ref ble_gap_set_conn_cte_trans_param_t.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_cte_trans_param_set(uint8_t conn_idx, ble_gap_set_conn_cte_trans_param_t *param);

/**
 ****************************************************************************************
 * @brief Set connection CTE receive parameters.
 *
 * @param[in] conn_idx:  The index of connection.
 * @param[in] param:     Set connection CTE receive parameters info, see @ref ble_gap_set_conn_cte_rcv_param_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_cte_recv_param_set(uint8_t conn_idx, ble_gap_set_conn_cte_rcv_param_t *param);

/**
 ****************************************************************************************
 * @brief Set connection CTE request enable.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] enable_flag:  Wheter to request the cte for the connection. If enable_flag is set to false, the param shall be NULL.
 * @param[in] param:        Set connection CTE request enable info, see @ref ble_gap_set_conn_cte_req_enable_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_cte_req_enable_set(uint8_t conn_idx, bool enable_flag, ble_gap_set_conn_cte_req_enable_t *param);

/**
 ****************************************************************************************
 * @brief Set connection CTE response enable.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] enable_flag:  Wheter to response the cte req for the connection.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_cte_rsp_enable_set(uint8_t conn_idx, bool enable_flag);

/**
 ****************************************************************************************
 * @brief Read the local current and maximum transmit power levels for the connection identified by the conn_idx.
 * @note  This API is asynchronous.
 * @note  Once the local transmit power level has been available, the event @ref BLE_GAPC_EVT_LOCAL_TX_POWER_READ will be called.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] phy:          Read the transmit power levels on which phy, see @ref ble_gap_phy_type_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_local_tx_pwr_level_read(uint8_t conn_idx, ble_gap_phy_type_t phy);

/**
 ****************************************************************************************
 * @brief Read the remote transmit power levels for the connection identified by the conn_idx.
 * @note  This API is asynchronous.
 * @note  Once the remote transmit power level has been available, the event @ref BLE_GAPC_EVT_REMOTE_TX_POWER_READ will be called.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] phy:          Read the transmit power levels on which phy, see @ref ble_gap_phy_type_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_remote_tx_pwr_level_read(uint8_t conn_idx, ble_gap_phy_type_t phy);

/**
 ****************************************************************************************
 * @brief Set the path loss threshold reporting parameters for the connection identified by the conn_idx.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] param:        Set path loss report parameter, see @ref ble_gap_set_path_loss_report_param_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_path_loss_report_parameter_set(uint8_t conn_idx, ble_gap_set_path_loss_report_param_t *param);

/**
 ****************************************************************************************
 * @brief Enable or disable path loss reporting for the connection identified by the conn_idx.
 * @note  This API is asynchronous.
 * @note  Once a path loss threshold crossing, the event @ref BLE_GAPC_EVT_PATH_LOSS_THRESHOLD_REPORT will be called.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] enable_flag:  The enable flag for reporting path loss.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_path_loss_report_enable_set(uint8_t conn_idx, bool enable_flag);

/**
 ****************************************************************************************
 * @brief Enable or disable the reporting of transmit power level changes in the local and remote for the connection identified by the conn_idx.
 * @note  This API is asynchronous.
 * @note  Once the transmit power changes, the event @ref BLE_GAPC_EVT_TX_POWER_CHANGE_REPORT will be called.
 *
 * @param[in] conn_idx:           The index of connection.
 * @param[in] local_enable_flag:  The enable flag for reporting transmit power level changes in the local.
 * @param[in] remote_enable_flag: The enable flag for reporting transmit power level changes in the remote.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_tx_pwr_change_report_enable_set(uint8_t conn_idx, bool local_enable_flag, bool remote_enable_flag);

/**
 ****************************************************************************************
 * @brief start ranging procedure.
 *
 * @param[in] con_idx: The index of connection.
 * @param[in] param:   Ranging parameter, see @ref ble_gap_ranging_param_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
int ble_gap_ranging_start(uint8_t con_idx, ble_gap_ranging_param_t *param);

/**
 ****************************************************************************************
 * @brief Set Subrate feature.
 * @note shall set Subrate feature before subrate request.
 *
 * @param[in] supp_flag: support flag.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 ****************************************************************************************
 */
uint16_t ble_subrate_set_host_feature(bool supp_flag);

/**
 ****************************************************************************************
 * @brief Set Default Subrate command.
 * @note  This API is asynchronous.
 * @note  Once Default Subrate command set completed, the event @ref BLE_GAPC_EVT_DFT_SUBRATE_SET will be called.
 *
 * @param[in] p_subrate_param: Default subrate param, see @ref ble_gap_dft_subrate_param_t.
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_default_subrate_param_set(const ble_gap_dft_subrate_param_t *p_subrate_param);

/**
 ****************************************************************************************
 * @brief Subrate Request command.
 * @note  This API is asynchronous.
 * @note  Once Subrate Request completed, the event @ref BLE_GAPC_EVT_SUBRATE_CHANGE_IND will be called.
 *
 * @param[in] conn_idx:      The index of conncetion.
 * @param[in] p_subrate_req: Subrate request param, see @ref ble_gap_subrate_req_t.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_subrate_request(uint8_t conn_idx, const ble_gap_subrate_req_t *p_subrate_req);

/** @} */
#endif
/** @} */
/** @} */
/** @} */

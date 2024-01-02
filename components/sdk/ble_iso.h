/**
 ****************************************************************************************
 *
 * @file ble_iso.h
 *
 * @brief BLE ISO API
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
 */

 /**
  @addtogroup BLE_ISO Core Isochronous Channels
  @{
  @brief Definitions and prototypes for the ISO interfaces.
 */

#ifndef __BLE_ISO_H__
#define __BLE_ISO_H__

#include "ble_error.h"
#include "ble_gapc.h"
#include <stdint.h>
#include <stdbool.h>

/** @addtogroup BLE_ISO_DEFINES Defines
 * @{ */

#define ISO_ADDR_LEN 6        /**< Length for address. */
#define ENC_CODE_LEN 16       /**< Length for encryption code. */

#define ISO_SYNC0_PULSE (0x1 << 0)   /**< ISO sync0 pulse. */
#define ISO_SYNC1_PULSE (0x1 << 1)   /**< ISO sync1 pulse. */
/** @} */


/**@addtogroup BLE_ISO_ENUMERATIONS Enumerations
 * @{ */

 /**@brief Operation ID. */
typedef enum
{
    ISO_READ_ISO_TX_SYNC,
    ISO_SET_CIG_PARAM,
    ISO_REMOVE_CIG,
    ISO_REJECT_CIS_REQ,
    ISO_SET_DATA_PATH,
    ISO_REMOVE_DATA_PATH,
    ISO_PER_ADV_SYNC_TRAN,
    ISO_SET_PER_ADV_SET_INFO_TRAN,
    ISO_SET_PER_ADV_SYNC_TRAN_PARAM,
    ISO_SET_DEFAULT_PER_ADV_SYNC_TRAN_PARAM,
} iso_op_id_t;

/**@brief Adv type for announcement. */
typedef enum
{
    ISO_ADV_UNDIRECT,         /**< undirect adv. */
    ISO_ADV_DIRECT,             /**< direct adv (Low duty). */
} iso_adv_type_t;

/**@brief Address type. */
typedef enum
{
    PUBLIC = 0,             /**< public type. */
    RANDOM,                 /**< random type. */
} iso_add_type_t;

/**@brief Data path type. */
typedef enum
{
    DATA_PATH_DISABLE   = 0xFF,     /**< Data path for disabled. */
    DATA_PATH_HCI       = 0x00,     /**< Data path for hci. */
    DATA_PATH_GEN       = 0xF2,     /**< Data path for gen. */
    DATA_PATH_GDX       = 0xF6,     /**< Data path for gdx. */
} iso_data_path_type_t;

/**@brief Data path direction type. */
typedef enum
{
    HOST_TO_CONTROLLER = 0x00,                   /**< Host to controller. */
    CONTROLLER_TO_HOST = 0x01,                   /**< Controller to host. */
    DIRECTION_BOTH     = 0x02,                   /**< Host to controller and controller to host, just use for remove data path. */
} iso_data_path_direction_t;

/**@brief SCA type. */
typedef enum
{
    SCA0 = 0x00,                     /**< 251 ppm to 500 ppm. */
    SCA1 = 0x01,                     /**< 151 ppm to 250 ppm. */
    SCA2 = 0x02,                     /**< 101 ppm to 150 ppm. */
    SCA3 = 0x03,                     /**< 76 ppm to 100 ppm. */
    SCA4 = 0x04,                     /**< 51 ppm to 75 ppm. */
    SCA5 = 0x05,                     /**< 31 ppm to 50 ppm. */
    SCA6 = 0x06,                     /**< 21 ppm to 30 ppm. */
    SCA7 = 0x07,                     /**< 0 ppm to 20 ppm. */
} iso_sca_type_t;

/**@brief Packing type. */
typedef enum
{
    PACK_SEQ = 0x00,                 /**< Sequential. */
    PACK_INT = 0x01                  /**< Interleaved. */
} iso_packing_type_t;

/**@brief Framing type. */
typedef enum
{
    UNFRAMED = 0x00,                 /**< Unframed. */
    FRAMED   = 0x01                  /**< Framed. */
} iso_framing_type_t;

/**@brief Encryption type. */
typedef enum
{
    UNENCRYPTED = 0x00,              /**< Unencrypted. */
    ENCRYPTED   = 0x01               /**< Encrypted. */
} iso_encryption_type_t;

/**@brief Sync mode type. */
typedef enum
{
    NO_EVT_NO_REPORT     = 0x00,          /**< No attempt is made to synchronize to the periodic advertising and no
                                               HCI_LE_Periodic_Advertising_Sync_Transfer_Received event is sent to the Host */
    WITH_EVT_NO_REPORT   = 0x01,          /**< An HCI_LE_Periodic_Advertising_Sync_Transfer_Received event is sent to the Host.
                                               HCI_LE_Periodic_Advertising_Report events will be disabled. */
    WITH_EVT_WITH_REPORT = 0x02,          /**< An HCI_LE_Periodic_Advertising_Sync_Transfer_Received event is sent to the Host.
                                               HCI_LE_Periodic_Advertising_Report events will be enabled. */
} iso_sync_mode_t;

/**@brief Cte type. */
typedef enum
{
    NOT_SYNC_WITH_AOA     = 0x01,          /**< Do not sync to packets with an AoA Constant Tone Extension. */
    NOT_SYNC_WITH_AOD_1US = 0x02,          /**< Do not sync to packets with an AoD Constant Tone Extension with 1 us slots. */
    NOT_SYNC_WITH_AOD_2US = 0x04,          /**< Do not sync to packets with an AoD Constant Tone Extension with 2 us slots. */
    NOT_SYNC_WITHOUT_CTE  = 0x10,          /**< Do not sync to packets without a Constant Tone Extension. */
} iso_cte_type_t;

/**@brief Phy type. */
typedef enum
{
    ISO_PHY_1M    = 0x01 << 0,             /**< The transmitter PHY of packets is LE 1M. */
    ISO_PHY_2M    = 0x01 << 1,             /**< The transmitter PHY of packets is LE 2M. */
    ISO_PHY_CODED = 0x01 << 2              /**< The transmitter PHY of packets is LE Coded. */
} iso_phy_type_t;

/**@brief Terminate big/big sync reason. */
typedef enum
{
    TERMINATE_BY_REMOTE = 0x13,            /**< Remote user terminate. */
    TERMINATE_BY_LOCAL  = 0x16,            /**< Local user terminate. */
} iso_termi_big_reason_t;

enum
{
    ISO_GPIO_MASK_0 = 0x1,
    ISO_GPIO_MASK_1 = 0x2,
    ISO_GPIO_MASK_2 = 0x4,
    ISO_GPIO_MASK_3 = 0x8,
    ISO_GPIO_MASK_4 = 0x10,
    ISO_GPIO_MASK_5 = 0x20,
    ISO_GPIO_MASK_6 = 0x40,
    ISO_GPIO_MASK_7 = 0x80
};
/** @} */

/** @addtogroup BLE_ISO_STRUCTURES Structures
 * @{ */

/**
@brief The parameter for starting broadcast announcement.
*/
typedef struct
{
    uint8_t adv_idx;          /**< Adv index, (Range: 0 to 4). */
    uint8_t sid;              /**< Adv set ID, (Range: 0x00 to 0xFF). */
    uint16_t interval;        /**< Adv interval, (Range: 0x0006 to 0xFFFF), Time = N * 1.25 ms,Time Range: 7.5ms to 81.91875 s. */
    uint16_t data_len;        /**< Broadcast audio announcement data length. */
    uint8_t *data;            /**< Broadcast audio announcement data. */
} iso_start_bd_ann_t;

/**
@brief The parameter for starting genneral announcement.
*/
typedef struct
{
    uint8_t adv_idx;          /**< Adv index. */
    uint32_t interval;        /**< Adv interval, (Range: 0x000020 to 0xFFFFFF), Time = N * 0.625. */
    uint16_t data_len;        /**< General audio announcement data length. */
    uint8_t *data;            /**< General audio announcement data. */
} iso_start_gen_ann_t;

/**
@brief The parameter for starting target announcement.
*/
typedef struct
{
    uint8_t adv_idx;                  /**< Adv index. */
    uint32_t interval;                /**< Adv interval, (Range: 0x000020 to 0xFFFFFF), Time = N * 0.625. */
    uint8_t adv_type;                 /**< Adv type, see iso_adv_type_t , the peer addr_type and addr
                                           is invalid for undirect type, and should ignore them. */
    uint8_t peer_add_type;            /**< Peer address type, see iso_add_type_t */
    uint8_t peer_addr[ISO_ADDR_LEN];  /**< Peer address. */
    uint16_t data_len;                /**< Target audio announcement data length. */
    uint8_t *data;                    /**< Target audio announcement data. */
} iso_start_target_ann_t;

/**
@brief The parameter for discovery announcement.
*/
typedef struct
{
    uint16_t interval;                /**< Scan interval between 0x0004 and 0x4000 in 0.625 ms units(range: 2.5 ms to 10.24 s). */
    uint16_t window;                  /**< Scan window between 0x0004 and 0x4000 in 0.625 ms units(range: 2.5 ms to 10.24 s). */
    uint16_t timeout;                 /**< Scan timeout should be a value between 0x0001 and 0xFFFF(unit: 10 ms).
                                           0x0000 indicates that the timeout have no effect. */
} iso_disc_ann_t;

/**
@brief The parameter for sync announcement.
*/
typedef struct
{
    uint8_t sid;                             /**< public type. */
    uint8_t peer_addr_type;                  /**< Peer address type, see iso_add_type_t */
    uint8_t peer_addr[ISO_ADDR_LEN];         /**< Peer address. */
    uint16_t sync_timeout;                   /**< Synchronization timeout for the periodic advertising(in unit of 10ms between 100ms and 163.84s). */
} iso_sync_ann_t;

/**
@brief The parameter for creating big.
*/
typedef struct
{
    uint8_t big_hdl;                            /**< Used to identify the BIG (0x00 to 0xEF). */
    uint8_t per_adv_idx;                        /**< Used to identify the periodic advertising set (0 to 4). */
    uint8_t num_bis;                            /**< Total number of BISs in the BIG(0x01 to 0x1F). */
    uint32_t sdu_interval;                      /**< Time duration between SDUs in microseconds(0x000100 to 0x0FFFFF). */
    uint16_t max_sdu_size;                      /**< Maximum size of a SDU (0x001 to 0xFFF). */
    uint16_t tran_latency;                      /**< Maximum time (in milliseconds) between transmission and reception of a BIS Data PDU (0x0000 to 0x0FA0). */
    uint8_t rtn;                                /**< Number of times every BIS Data PDU should be retransmitted(0x00 to 0x0F). */
    uint8_t phy;                                /**< Indicates the PHY used for transmission of PDUs of BISs in the BIG.see iso_phy_type_t */
    uint8_t packing;                            /**< Sequential or Interleaved, see iso_packing_type_t. */
    uint8_t framing;                            /**< Unframed or Framed, see iso_framing_type_t. */
    uint8_t encryption;                         /**< Unencrypted or Encrypted, see iso_encryption_type_t. */
    uint8_t encryption_code[ENC_CODE_LEN];      /**< The code used for encrypting or decrypting payloads of an encrypted BIS. */
} iso_create_big_t;

/**
@brief The parameter for creating big sync.
*/
typedef struct
{
    uint8_t big_hdl;                 /**< Used to identify the BIG (0x00 to 0xEF). */
    uint16_t per_sync_idx;           /**< Used to identify the periodic advertising set (0 to 4). */
    uint8_t encryption;              /**< Unencrypted or Encrypted, see iso_encryption_type_t. */
    uint8_t enc_code [ENC_CODE_LEN]; /**< The code used for encrypting or decrypting payloads of an encrypted BIS. */
    uint8_t mse;                     /**< Maximum number of subevents that are used to receive data payloads in each isochronous interval (0x01 to 0xFF). */
    uint16_t big_sync_timeout;       /**< Synchronization timeout for the BIS, Range: 0x000A to 0x4000. Time = N*10 ms, Time Range: 100 ms to 163.84 s */
    uint8_t num_bis;                 /**< Total number of BISs to synchronize (0x01 to 0x1F). */
    uint8_t *bis;                    /**< List of BISs in the BIG (0x01 to 0x1F). */
} iso_create_big_sync_t;

/**
@brief The parameter for setting data path.
*/
typedef struct
{
    uint16_t conn_hdl;            /**< Connection handle of a CIS or BIS. */
    uint8_t direction;            /**< Direction for data path, see iso_data_path_direction_t. */
    uint8_t data_path_type;       /**< Type for data path, see iso_data_path_type_t. */
    uint8_t codec_id[5];          /**< Codec ID (octet 0:coding format; octet 1 to 2:company ID; octets 3 to 4:vendor-defined codec ID) */
    uint32_t ctrl_delay;          /**< Controller delay in microseconds (Range: 0x000000 to 0x3D0900). */
    uint8_t codec_cfg_len;        /**< Codec configuration length. */
    uint8_t *codec_cfg;           /**< Codec configuration. */
} iso_set_data_path_t;

/**
@brief The parameter for removing data path.
*/
typedef struct
{
    uint16_t conn_hdl;                        /**< Connection handle of a CIS or BIS. */
    uint8_t direction;                        /**< Specifies the input data transport path, see iso_data_path_direction_t. */
} iso_rm_data_path_t;

/**
@brief The parameter for setting cis.
*/
typedef struct
{
    uint8_t cis_id;                     /**< Used to identify a CIS (0x00 to 0xEF). */
    uint16_t sdu_size_m2s;              /**< Maximum size of a SDU in octets from the master Host (0x000 to 0xFFF). */
    uint16_t sdu_size_s2m;              /**< Maximum size of a SDU in octets from the slave Host (0x000 to 0xFFF). */
    uint8_t phy_m2s;                    /**< Master to slave PHY, see iso_phy_type_t. */
    uint8_t phy_s2m;                    /**< Slave to master PHY, see iso_phy_type_t. */
    uint8_t rtn_m2s;                    /**< Number of times every CIS Data PDU should be retransmitted from the master to slave (0x00 to 0x0F). */
    uint8_t rtn_s2m;                    /**< Number of times every CIS Data PDU should be retransmitted from the slave to master (0x00 to 0x0F). */
} iso_cis_param_t;

/**
@brief The parameter for setting cig.
*/
typedef struct
{
    uint8_t cig_id;                     /**< Used to identify a CIG (0x00 to 0xEF). */
    uint32_t sdu_int_m2s;               /**< The interval, in microseconds, of periodic SDUs.(0x0000FF to 0xFFFFF). */
    uint32_t sdu_int_s2m;               /**< The interval, in microseconds, of periodic SDUs. (0x0000FF to 0xFFFFF). */
    uint8_t sca;                        /**< The worst-case sleep clock accuracy of all the slaves, see iso_sca_type_t. */
    uint8_t packing;                    /**< Sequential or Interleaved, see iso_packing_type_t. */
    uint8_t framing;                    /**< Unframed or Framed, see iso_framing_type_t. */
    uint16_t trans_latency_m2s;         /**< Maximum time, in milliseconds, for an SDU to be transported from the master Controller to slave Controller(0x0005 to 0x0FA0). */
    uint16_t trans_latency_s2m;         /**< Maximum time, in milliseconds, for an SDU to be transported from the slave Controller to master Controller(0x0005 to 0x0FA0). */
    uint8_t cis_cnt;                    /**< Total number of CISs (0x00 to 0x1F). */
    iso_cis_param_t *cis_param;         /**< CIS parameter, see iso_cis_param_t. */
} iso_set_cig_param_t;

/**
@brief The parameter for creating per cis.
*/
typedef struct
{
    uint16_t cis_hdl;                /**< List of connection handles of CISes (Range 0x0000-0x0EFF). */
    uint16_t conn_idx;               /**< Connection index of ACL Link. */
} iso_create_cis_param_t;

/**
@brief The parameter for creating multi cis.
*/
typedef struct
{
    uint8_t num_cis;                     /**< Total number of CISs to be established (0x01 to 0x1F). */
    iso_create_cis_param_t *params;      /**< Create cis param, see iso_create_cis_param_t. */
} iso_create_cis_t;

/**
@brief The parameter for setting adv sync transport parameter.
*/
typedef struct
{
    uint16_t conn_hdl;         /**< Connection handle of the ACL, (Range: 0x0000 to 0x0EFF). */
    uint8_t mode;              /**< the action to be taken when periodic advertising synchronization information is received, see iso_sync_mode_t. */
    uint16_t skip;             /**< The number of periodic advertising packets that can be skipped after a successful receive, (Range: 0x0000 to 0x01F3). */
    uint16_t sync_timeout;     /**< Synchronization timeout for the periodic advertising,(Range: 0x000A to 0x4000). Time = N*10 ms, Time Range: 100 ms to 163.84 s */
    uint8_t cte_type;          /**< specifies whether to only synchronize to periodic advertising with certain types of Constant Tone Extension, see iso_cte_type_t. */
} iso_set_per_adv_sync_tran_param_t;

/**
@brief The parameter for setting default periodic advertising synchronization transport.
*/
typedef struct
{
    uint8_t mode;              /**< the action to be taken when periodic advertising synchronization information is received, see iso_sync_mode_t. */
    uint16_t skip;             /**< The number of periodic advertising packets that can be skipped after a successful receive, (Range: 0x0000 to 0x01F3). */
    uint16_t sync_timeout;     /**< Synchronization timeout for the periodic advertising,(Range: 0x000A to 0x4000). Time = N*10 ms, Time Range: 100 ms to 163.84 s */
    uint8_t cte_type;          /**< specifies whether to only synchronize to periodic advertising with certain types of Constant Tone Extension, see iso_cte_type_t. */
} iso_set_default_per_adv_sync_tran_param_t;

/**
@brief The parameter for setting periodic advertising set information.
*/
typedef struct
{
    uint16_t adv_hdl;           /**< Used to identify an advertising set,(Range: 0x00-0xEF). */
    uint16_t service_data;      /**< The Service_Data parameter is a value provided by the Host to identify
                                     the periodic advertisement to the peer device. It is not used by the Controller.*/
    uint16_t conn_hdl;          /**< Connection_Handle, (Range: 0x0000 to 0x0EFF)*/
} iso_set_per_adv_set_info_tran_t;

/**
@brief The parameter for periodic advertising synchronization transport.
*/
typedef struct
{
    uint16_t sync_hdl;          /**< identifying the periodic advertising, (Range: 0x0000 to 0x0EFF). */
    uint16_t service_data;      /**< The Service_Data parameter is a value provided by the Host to identify
                                     the periodic advertisement to the peer device. It is not used by the Controller.*/
    uint16_t conn_hdl;          /**< Connection_Handle, (Range: 0x0000 to 0x0EFF)*/
} iso_per_adv_sync_tran_t;

/**
@brief The parameter for setting periodic advertising reciving enable.
*/
typedef struct
{
    uint16_t sync_hdl;              /**< identifying the periodic advertising, (Range: 0x0000 to 0x0EFF). */
    uint8_t enable;                 /**< 0x00: Reporting disabled, 0x01: Reporting enabled. */
} iso_set_per_adv_rcv_enable_t;

/**
@brief The parameter for set iso data path trigger.
*/
typedef struct
{
    uint16_t conn_hdl;         /**< Connection handle of CIS/BIS. */
    uint8_t direction;         /**< Specifies the data transport path, see iso_data_path_direction_t. */
    uint8_t enable_flag;       /**< 0x00: disable, 0x01: enable. */
    int32_t trigger_offset;    /**< Trigger offset in microseconds. */
} iso_set_data_path_trigger_t;

/**
@brief The parameter for cis establish event.
*/
typedef struct
{
    uint8_t status;            /**< 0x00: The CIS has been established, 0x01 to 0xFF: The CIS failed to be established. */
    uint16_t cis_hdl;          /**< Connection handle of CIS (Range 0x0000-0x0EFF). */
    uint32_t cig_sync_delay;   /**< The CIG synchronization delay time in microseconds(Range 0x0000EA to 0x7FFFFF). */
    uint32_t cis_sync_delay;   /**< The CIS synchronization delay time in microseconds(Range 0x0000EA to 0x7FFFFF). */
    uint32_t tran_latency_m2s; /**< The maximum time, in microseconds, for transmission of SDUs of all CISes from master to slave(Range 0x0000EA to 0x7FFFFF). */
    uint32_t tran_latency_s2m; /**< The maximum time, in microseconds, for transmission of SDUs of all CISes from slave to master(Range 0x0000EA to 0x7FFFFF). */
    uint8_t phy_m2s;           /**< indicates the PHY selected for packets from the master to slave. see iso_phy_type_t*/
    uint8_t phy_s2m;           /**< indicates the PHY selected for packets from the slave to master. see iso_phy_type_t */
    uint8_t nse;               /**< Maximum number of subevents in each isochronous event (Range 0x01 to 0x1E). */
    uint8_t bn_m2s;            /**< The burst number for master to slave transmission (Range 0x00 to 0x0F). 0x00: no isochronous data from the master to the slave. */
    uint8_t bn_s2m;            /**< The burst number for slave to master transmission (Range 0x00 to 0x0F). 0x00: no isochronous data from the slave to the master. */
    uint8_t ft_m2s;            /**< The flush timeout, in multiples of the ISO_Interval for the CIS, for each payload sent from the master to the slave (Range: 0x01 to 0xFF). */
    uint8_t ft_s2m;            /**< The flush timeout, in multiples of the ISO_Interval for the CIS, for each payload sent from the slave to the master (Range: 0x01 to 0xFF). */
    uint16_t max_pdu_m2s;      /**< Maximum size, in octets, of the payload from master to slave (Range: 0x0000 to 0x00FB). */
    uint16_t max_pdu_s2m;      /**< Maximum size, in octets, of the payload from slave to master (Range: 0x0000 to 0x00FB). */
    uint16_t iso_interval;     /**< The time between two consecutive CIS anchor points (unit:1.25ms, Range:0x0004 to 0x0C80, Time Range:5ms to 4s). */
} iso_cis_est_evt_t;

/**
@brief The parameter for cis request event.
*/
typedef struct
{
    uint16_t conn_idx;         /**< Connection index of the ACL. */
    uint16_t cis_hdl;          /**< Connection handle of CIS (Range 0x0000-0x0EFF). */
    uint8_t cig_id;            /**< Identifier of the CIG (Range 0x00-0xEF). */
    uint8_t cis_id;            /**< Identifier of the CIS (Range 0x00-0xEF). */
} iso_cis_req_evt_t;

/**
@brief The parameter for cis disconnect event.
*/
typedef struct
{
    uint16_t cis_hdl;          /**< Connection handle of CIS (Range 0x0000-0x0EFF). */
    uint8_t reason;            /**< Reason for disconnect CIS. */
} iso_cis_disc_evt_t;

/**
@brief The parameter for creating big complete event.
*/
typedef struct
{
    uint8_t status;                /**< 0x00: Establishment of the BIG has been completed. 0x01 to 0xFF: Establishment of the BIG failed to be completed */
    uint8_t big_hdl;               /**< Used as the identifier of the BIG, (Range: 0x00 to 0xEF) */
    uint32_t big_sync_delay;       /**< Transmission delay time in microseconds of all BISs in the BIG,(Range: 0x0000EA to 0x7FFFFF). */
    uint32_t big_trans_latency;    /**< The maximum delay time, in microseconds, for transmission of SDUs of all BISes,(Range: 0x0000EA to 0x7FFFFF). */
    uint8_t phy;                   /**< PHY used, bit 0: 1Mbps, bit 1: 2Mbps, bit 2: LE-Coded. */
    uint8_t nse;                   /**< The number of subevents in each BIS event in the BIG, range 0x01-0x1E. */
    uint8_t bn;                    /**< TThe number of new payloads in each BIS event, range 0x01-0x07. */
    uint8_t pto;                   /**< Offset used for pre-transmissions, range 0x00-0x0F. */
    uint8_t irc;                   /**< The number of times a payload is transmitted in a BIS event, range 0x01-0x0F. */
    uint16_t max_pdu;              /**< Maximum size of the payload in octets, range 0x00-0xFB. */
    uint16_t iso_interval;         /**< ISO interval (1.25ms unit, range: 5ms to 4s). */
    uint8_t num_bis;               /**< Total number of BISs in the BIG,(Range: 0x01 to 0x1F). */
    uint16_t *bis_hdl;             /**< The connection handles of the BISs, (Range: 0x0000 to 0x0EFF). */
} iso_create_big_cmpl_evt_t;

/**
@brief The parameter for terminating big complete event.
*/
typedef struct
{
    uint8_t reason;        /**< indicate the reason why the BIG was terminated. */
    uint8_t big_hdl;       /**< Used as the identifier of the BIG (Range 0x00 to 0xEF). */
} iso_termi_big_cmpl_evt_t;

/**
@brief The parameter for creating big sync complete event.
*/
typedef struct
{
    uint8_t status;             /**< 0x00: Synchronization to BIG has been completed.0x01 to 0xFF: Synchronization to BIG failed to be completed. */
    uint8_t big_hdl;            /**< Used as the identifier of the BIG (Range 0x00 to 0xEF). */
    uint32_t big_trans_latency; /**< The maximum delay time, in microseconds, for transmission of SDUs of all BISes(Range 0x0000EA to 0x7FFFFF). */
    uint8_t nse;                /**< The number of subevents in each BIS event in the BIG, range 0x01-0x1E. */
    uint8_t bn;                 /**< The number of new payloads in each BIS event, range 0x01-0x07. */
    uint8_t pto;                /**< Offset used for pre-transmissions, range 0x00-0x0F. */
    uint8_t irc;                /**< The number of times a payload is transmitted in a BIS event, range 0x01-0x0F. */
    uint16_t max_pdu;           /**< Maximum size of the payload in octets, range 0x00-0xFB. */
    uint16_t iso_interval;      /**< ISO interval (1.25ms unit, range: 5ms to 4s). */
    uint8_t num_bis;            /**< Total number of BISs in the BIG.(Range 0x01 to 0x1F). */
    uint16_t *bis_hdl;          /**< The connection handles of the BISs, (Range: 0x0000 to 0x0EFF). */
} iso_big_create_sync_est_evt_t;

/**
@brief The parameter for big lost event.
*/
typedef struct
{
    uint16_t big_hdl;        /**< Used as the identifier the big, (Range: 0x0000 to 0x0EFF). */
    uint8_t reason;          /**< indicate the reason why the synchronization was terminated. */
} iso_big_sync_lost_evt_t;

/**
@brief The parameter for big info report event.
*/
typedef struct
{
    uint8_t  per_sync_idx;      /**< Per sync index, (Range: 0x00 to 0x05). */
    uint8_t  num_bis;           /**< Number of BIS (Range 0x01-0x1F). */
    uint8_t  nse;               /**< Value of the NSE (Range 0x01-0x1F). */
    uint16_t iso_interval;      /**< Value of the ISO interval. */
    uint8_t  bn;                /**< Value of the BN (Range 0x00-0x07). */
    uint8_t  pto;               /**< Value of the PTO (Range 0x00-0x0F). */
    uint8_t  irc;               /**< Value of the IRC (Range 0x01-0x0F). */
    uint16_t max_pdu;           /**< Value of the Max_PDU (Range 0x0000-0x00FB). */
    uint32_t sdu_interval;      /**< Value of the SDU_Interval (Range Range 0x0000FF-0x0FFFFF). */
    uint16_t max_sdu;           /**< Value of the Max_SDU (Range 0x0000-0x0FFF). */
    uint8_t  phy;               /**< Value of the PHY (0x01: 1M, 0x02: 2M, 0x03: Coded, All other values: RFU). */
    uint8_t  framing;           /**< Value of the Framing (0x00: Unframed, 0x01: Framed, All other values: RFU). */
    uint8_t  encryption;        /**< Value of the Encryption (0x00: Unencrypted, 0x01: Encrypted, All other values: RFU)). */
} iso_big_info_report_evt_t;

/**
@brief The parameter for request peer sca complete event.
*/
typedef struct
{
    uint8_t status;         /**< 0x00: The SCA parameter received successfully.0x01-0xFF: The reception of SCA parameter failed */
    uint16_t acl_hdl;       /**< Connection handle of the ACL, (Range: 0x0000 to 0x0EFF). */
    uint8_t sca;            /**< See iso_sca_type_t. */
} iso_req_peer_sca_cmpl_evt_t;

/**
@brief The parameter for receiving periodic advertising Synchronization transport event.
*/
typedef struct
{
    uint8_t status;              /**< 0x00: Synchronization to the periodic advertising succeeded, 0x01 to 0xFF: Synchronization to the periodic advertising failed */
    uint16_t conn_hdl;           /**< Connection_Handle, (Range: 0x0000 to 0x0EFF). */
    uint16_t service_data;       /**< A value provided by the peer device. */
    uint8_t adv_sid;             /**< Value of the Advertising SID used to advertise the periodic advertising. */
    uint16_t sync_hdl;           /**< Sync_Handle identifying the periodic advertising, (Range: 0x0000 to 0x0EFF). */
} iso_per_adv_sync_tran_rcv_evt_t;

/**
@brief The parameter for read iso tx sync complete event.
*/
typedef struct
{
    uint8_t status;         /**< 0x00: succeeded. 0x01-0xFF: failed */
    uint16_t con_hdl;       /**< Connection handle of the bis or cis, (Range: 0x0000 to 0x0EFF). */
    uint16_t pkt_seq_num;   /**< The packet sequence number of the ISO_SDU. */
    uint32_t time_stap;     /**< The timestamp of an SDU identified by the seq_num, (Range: 0x00000000 to 0xFFFFFFFF) */
    uint32_t time_offset;   /**< The time offset associated with the ISO_SDU identified by the seq_num, (Range: 0x000000 to 0xFFFFFF)*/
} iso_read_iso_tx_sync_cmpl_evt_t;

/**
@brief The parameter for set cig parameter complete event.
*/
typedef struct
{
    uint8_t status;         /**< 0x00: succeeded. 0x01-0xFF: failed */
    uint8_t cig_id;         /**< Used to identify a CIG, (Range: 0x00 to 0xEF). */
    uint8_t cis_count;      /**< Total number of CISs, (Range: 0x00 to 0x10). */
    uint16_t *cis_hdl;      /**< List of connection handles of CISs in the CIG, (Range: 0x0000 to 0x0EFF). */
} iso_set_cig_param_cmpl_evt_t;

/**
@brief The structure for iso hci data.
*/
typedef struct
{
    uint16_t conn_hdl;            /**< Used to identify a cis or bis. */
    uint32_t time_stamp;          /**< The Bluetooth_TimeStamp (BTS). */
    uint16_t seq_num;             /**< Packet sequence number. */
    uint16_t sdu_len;             /**< Sdu length. */
    uint8_t *sdu;                 /**< Sdu data buffer. */
} iso_hci_data_t;

/**
@brief The parameter for set cig parameter complete event.
*/

/**
@brief The callback for iso.
*/
typedef struct
{
    void (*iso_sdu_rcv_cb)(iso_hci_data_t *data_info); /**< The callback for receving iso sdu. */

    void (*iso_sdu_send_cb)(iso_hci_data_t *data_info); /**< The callback for sending iso sdu. */

    void (*iso_cis_est_evt_cb)(iso_cis_est_evt_t *evt); /**< The callback for receving cis established event. */

    void (*iso_cis_req_evt_cb)(iso_cis_req_evt_t *evt); /**< The callback for receving a request to establish a CIS from the master. */

    void (*iso_cis_disc_ind_cb)(iso_cis_disc_evt_t *evt); /**< The callback for cis disconnect. */

    void (*iso_create_big_cmpl_evt_cb)(iso_create_big_cmpl_evt_t *evt); /**< The callback for receving create big complete event. */

    void (*iso_termi_big_cmpl_evt_cb)(iso_termi_big_cmpl_evt_t *evt); /**< The callback for receving terminate big complete event. */

    void (*iso_big_create_sync_est_evt_cb)(iso_big_create_sync_est_evt_t *evt); /**< The callback for receving big create sync established event. */

    void (*iso_big_sync_lost_evt_cb)(iso_big_sync_lost_evt_t *evt); /**< The callback for receving big lost event. */

    void (*iso_big_info_report_evt_cb)(iso_big_info_report_evt_t *evt); /**< The callback for receving big info report event. */

    void (*iso_req_peer_sca_cmpl_evt_cb)(iso_req_peer_sca_cmpl_evt_t *evt); /**< The callback for receving request peer sca complete event. */

    void (*iso_read_iso_tx_sync_cmpl_evt_cb)(iso_read_iso_tx_sync_cmpl_evt_t *evt); /**< The callback for read iso tx sync complete event. */

    void (*iso_set_cig_param_cmpl_evt_cb)(iso_set_cig_param_cmpl_evt_t *evt); /**< The callback for set cig parameter complete event. */

    void (*iso_rm_cig_cmpl_evt_cb)(uint8_t status, uint8_t cig_id); /**< The callback for remove cig complete event. */

    void (*iso_reject_cis_req_cmpl_evt_cb)(uint8_t status, uint16_t cis_hdl); /**< The callback for reject cis request complete event. */

    void (*iso_set_data_path_cmpl_evt_cb)(uint8_t status, uint16_t con_hdl); /**< The callback for set data path complete event. */

    void (*iso_rm_data_path_cmpl_evt_cb)(uint8_t status, uint16_t con_hdl); /**< The callback for remove data path complete event. */

    void (*iso_temi_big_sync_cmpl_evt_cb)(uint8_t status); /**< The callback for teminate big sync complete event. */

    void (*iso_set_data_path_trigger_cmpl_evt_cb)(uint8_t status); /**< The callback for teminate big sync complete event. */

    void (*iso_cmd_status_evt_cb)(uint16_t op_code, uint8_t status); /**< The callback for cmd status. */
} iso_cb_fun_t;

/**
@brief The callback for gap event which need to be handled by ascp.
*/
typedef struct
{
    void (*gap_adv_report_ind_cb)(const ble_gap_ext_adv_report_ind_t *p_adv_report); /**< The callback for receving adv report event. */

    void (*gap_sync_established_cb)(uint8_t inst_idx, uint8_t status, const ble_gap_sync_established_ind_t *p_sync_est_evt); /**< The callback for receving sync periodic adv success event. */

} ascp_gap_cb_fun_t;

/** @} */


/** @addtogroup BLE_ISO_FUNCTIONS Functions
 * @{ */

/**
 ****************************************************************************************
 * @brief Register callback for iso.
 * @param[in] cb: Pointer to the callback function structure.
 *
 * @retval SDK_SUCCESS: The callback is successfully to register.
 * @retval BLE_SDK_ERR_BAD_PARAM: The parameter is invalid, such as the cb is NULL.
 ****************************************************************************************
 */
uint16_t ble_iso_register_callback(iso_cb_fun_t *cb);

/**
 ****************************************************************************************
 * @brief Register the gap evetn callback for ascp.
 * @param[in] cb: Pointer to the callback function structure.
 ****************************************************************************************
 */
void ble_iso_register_ascp_gap_callback(ascp_gap_cb_fun_t *cb);

/**
 ****************************************************************************************
 * @brief Start broadcast audio announcement by using periodic adv.
 * @param[in] param:            Pointer to the start broadcast announcement structure.
 * @param[in] ext_adv_data:     Pointer to the ext adv data.
 * @param[in] ext_adv_data_len: The length of ext adv data.
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_start_bd_ann(iso_start_bd_ann_t *param, uint8_t *ext_adv_data, uint16_t ext_adv_data_len);

/**
 ****************************************************************************************
 * @brief Start general audio announcement by using extended adv.
 * @param[in] param: Pointer to the start general announcement structure.
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_start_gen_ann(iso_start_gen_ann_t *param);

/**
 ****************************************************************************************
 * @brief Start target audio announcement by using extended adv.
 * @param[in] param: Pointer to the start target announcement structure.
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_start_target_ann(iso_start_target_ann_t *param);

/**
 ****************************************************************************************
 * @brief Stop the audio announcement by using stop adv.
 * @param[in] adv_idx: Advertising Index.
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_stop_ann(uint8_t adv_idx);

/**
 ****************************************************************************************
 * @brief Discovery the audio announcement by using extended scan.
 * @param[in] param: Pointer to the Discovery announcement structure.
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_iso_discover_ann(iso_disc_ann_t *param);

/**
 ****************************************************************************************
 * @brief Stop discovery the audio announcement by stop extended scan.
 *
 * @retval SDK_SUCCESS.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 ****************************************************************************************
 */
uint16_t ble_iso_stop_discover_ann(void);

/**
 ****************************************************************************************
 * @brief Sync the broadcast audio announcement by sync the period adv.
 * @param[in] index: Periodic synchronization index.
 * @param[in] param: Pointer to the sync audio announcement structure.
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 * @retval SDK_ERR_INVALID_PER_SYNC_IDX: Invalid periodic syncronization index supplied.
 * @retval SDK_ERR_NO_RESOURCES: Not enough resources.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 ****************************************************************************************
 */
uint16_t ble_iso_sync_bd_ann(uint8_t index, iso_sync_ann_t *param);

/**
 ****************************************************************************************
 * @brief Stop sync the broadcast audio announcement.
 * @param[in] index: Periodic synchronization index.
 *
 * @retval SDK_SUCCESS: Operation is successful.
 * @retval SDK_ERR_INVALID_PER_SYNC_IDX: Invalid periodic syncronization index supplied.
 * @retval SDK_ERR_DISALLOWED: Operation is disallowed.
 ****************************************************************************************
 */
uint16_t ble_iso_stop_sync_bd_ann(uint8_t index);

/**
 ****************************************************************************************
 * @brief Create the BIG.
 * @param[in] param: Pointer to the create big structure.
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_create_big(iso_create_big_t *param);

/**
 ****************************************************************************************
 * @brief Terminate the BIG.
 * @param[in] big_hdl: Used to identify the BIG (0x00 to 0xEF).
 * @param[in] reason:  The reason for teminate the big.
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_terminate_big(uint8_t big_hdl, uint8_t reason);

/**
 ****************************************************************************************
 * @brief Create the BIG sync.
 * @param[in] param: Pointer to the create big sync structure.
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_create_big_sync(iso_create_big_sync_t *param);

/**
 ****************************************************************************************
 * @brief Terminate the BIG sync.
 * @param[in] big_hdl: Used to identify the BIG (0x00 to 0xEF).
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_terminate_big_sync(uint8_t big_hdl);

/**
 ****************************************************************************************
 * @brief Set CIG paramter.
 * @param[in] param: Pointer to the cig parameter structure.
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_set_cig_param(iso_set_cig_param_t *param);

/**
 ****************************************************************************************
 * @brief Create CIS.
 * @param[in] param: Pointer to the create cis parameter structure.
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_create_cis(iso_create_cis_t *param);

/**
 ****************************************************************************************
 * @brief Remove the cig indicated by the cig_id.
 * @param[in] cig_id: Used to identify a CIG (Range: 0x00 to 0xEF).
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_rm_cig(uint8_t cig_id);

/**
 ****************************************************************************************
 * @brief Slave Host to inform the Controller to accept the request for the CIS that is
 *  identified by the cis_hdl.
 * @param[in] cis_hdl: Connection handle of the CIS. (Range: 0x0000 to 0x0EFF).
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_accept_cis_req(uint16_t cis_hdl);

/**
 ****************************************************************************************
 * @brief slave Host to reject the request for the CIS that is identified by the cis_hdl.
 * @param[in] cis_hdl: Connection handle of the CIS. (Range: 0x0000 to 0x0EFF).
 * @param[in] reason:  Reason the CIS request was rejected.
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_reject_cis_req(uint16_t cis_hdl, uint8_t reason);

/**
 ****************************************************************************************
 * @brief disconnect the CIS that is identified by the cis_hdl.
 * @param[in] cis_hdl: Connection handle of the CIS. (Range: 0x0000 to 0x0EFF).
 *
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_disc_cis(uint16_t cis_hdl);

/**
 ****************************************************************************************
 * @brief Set the ISO data path.
 * @param[in] param: Pointer to set data path parameter structure.
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_set_data_path(iso_set_data_path_t *param);

/**
 ****************************************************************************************
 * @brief Remove the ISO data path.
 * @param[in] param: Pointer to remove data path parameter structure.
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_rm_data_path(iso_rm_data_path_t *param);

/**
 ****************************************************************************************
 * @brief Read the Sleep Clock Accuracy of the peer device.
 * @param[in] conn_idx: Connection of the acl connection.
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_req_peer_sca(uint16_t conn_idx);

/**
 ****************************************************************************************
 * @brief read the Bluetooth_TimeStamp (BTS) of a packet identified by the Packet_Sequence_Number
 *  on a CIS or BIS identified by the Connection_Handle.
 * @param[in] conn_hdl: Connection handle of a CIS or BIS, (Range: 0x0000 to 0x0EFF).
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_read_tx_sync(uint16_t conn_hdl);

/**
 ****************************************************************************************
 * @brief Set iso data path trigger
 * @param[in] param: Pointer to the set data path trigger parameter structure.
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_set_data_path_trigger(iso_set_data_path_trigger_t *param);

/**
 ****************************************************************************************
 * @brief Un-mask the ISO GPIO
 * @param[in] gpio_sel: Bits to un-mask iso_gpio.see enum iso_gpio_mask_t.
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_gpio_enable(uint8_t gpio_sel);

/**
 ****************************************************************************************
 * @brief Set iso sync pulse GPIO trigger
 * @param[in] iso_sync_p_sel: select iso sync0 pulse(ISO_SYNC0_PULSE) or/and iso sync1 pulse(ISO_SYNC1_PULSE).
 * @param[in] sync0_gpio_sel: Bits to select which iso_gpio signals cause an iso sync0 pulse. GPIO_0 is default for iso sync pulse gpio.see enum iso_gpio_mask_t.
 * @param[in] sync1_gpio_sel: Bits to select which iso_gpio signals cause an iso sync1 pulse. GPIO_0 is default for iso sync pulse gpio.see enum iso_gpio_mask_t.
 * @retval SDK_SUCCESS The parameter is valid.
 * @retval BLE_SDK_ERR_BAD_PARAM The parameter is invalid.
 ****************************************************************************************
 */
uint16_t ble_iso_set_iso_sync_pulse(uint8_t iso_sync_p_sel, uint8_t sync0_gpio_sel, uint8_t sync1_gpio_sel);

/** @} */

#endif

/**
  @}
*/
/** @} */


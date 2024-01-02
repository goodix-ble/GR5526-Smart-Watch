/**
 ****************************************************************************************
 *
 * @file ble_event.h
 *
 * @brief BLE event header files
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
 @addtogroup BLE_EVENT BLE Event
 @{
 @brief BLE Event interface.
 */
#ifndef __BLE_EVENT_H__
#define __BLE_EVENT_H__

#include "ble_gapm.h"
#include "ble_gapc.h"
#include "ble_gatts.h"
#include "ble_gattc.h"
#include "ble_sec.h"
#include "ble_l2cap.h"

/** @addtogroup BLE_EVENT_DEFINES Defines
 * @{
*/
/** @defgroup BLE_EVENT_BASE BLE Event Base
 * @{ */
#define BLE_COMMON_EVT_BASE         0x0100      /**< BLE Common Event base. */
#define BLE_GAPM_EVT_BASE           0x0200      /**< BLE GAP Management Event base. */
#define BLE_GAPC_EVT_BASE           0x0300      /**< BLE GAP Connection Control Event base. */
#define BLE_GATTS_EVT_BASE          0x0400      /**< BLE GATTS Event base. */
#define BLE_GATTC_EVT_BASE          0x0500      /**< BLE GATTC Event base. */
#define BLE_GATT_COMMON_EVT_BASE    0x0600      /**< BLE GATT Event base. */
#define BLE_SEC_EVT_BASE            0x0700      /**< BLE Security Event base. */
#define BLE_L2CAP_EVT_BASE          0x0800      /**< BLE L2CAP Event base. */
/** @} */
/** @} */

/**
 * @defgroup BLE_EVENT_ENUM Enumerations
 * @{
 */
/**
 * @brief BLE Common Events.
 */
enum BLE_COMMON_EVTS
{
    BLE_COMMON_EVT_STACK_INIT = BLE_COMMON_EVT_BASE, /**< BLE Stack init complete event. */
    BLE_COMMON_EVT_MAX,
};

/**@brief BLE GAP Managerment Events. */
enum  BLE_GAPM_EVTS
{
    BLE_GAPM_EVT_CH_MAP_SET    = BLE_GAPM_EVT_BASE,   /**< Channel Map Set complete event. */
    BLE_GAPM_EVT_WHITELIST_SET,                       /**< Whitelist Set complete event. */
    BLE_GAPM_EVT_PER_ADV_LIST_SET,                    /**< Periodic Advertising List Set complete event. */
    BLE_GAPM_EVT_PRIVACY_MODE_SET,                    /**< Privacy Mode for Peer Device Set complete event. */
    BLE_GAPM_EVT_LEPSM_REGISTER,                      /**< LEPSM Register complete event. */
    BLE_GAPM_EVT_LEPSM_UNREGISTER,                    /**< LEPSM Unregister complete event. */
    BLE_GAPM_EVT_DEV_INFO_GOT,                        /**< Device Info Get event. */
    BLE_GAPM_EVT_ADV_START,                           /**< Advertising Start complete event. */
    BLE_GAPM_EVT_ADV_STOP,                            /**< Advertising Stop complete event. */
    BLE_GAPM_EVT_SCAN_REQUEST,                        /**< Scan Request event. */
    BLE_GAPM_EVT_ADV_DATA_UPDATE,                     /**< Advertising Data update event. */
    BLE_GAPM_EVT_SCAN_START,                          /**< Scan Start complete event. */
    BLE_GAPM_EVT_SCAN_STOP,                           /**< Scan Stop complete event. */
    BLE_GAPM_EVT_ADV_REPORT,                          /**< Advertising Report event. */
    BLE_GAPM_EVT_SYNC_ESTABLISH,                      /**< Periodic Advertising Synchronization Establish event. */
    BLE_GAPM_EVT_SYNC_STOP,                           /**< Periodic Advertising Synchronization Stop event. */
    BLE_GAPM_EVT_SYNC_LOST,                           /**< Periodic Advertising Synchronization Lost event. */
    BLE_GAPM_EVT_READ_RSLV_ADDR,                      /**< Read Resolvable Address event. */
    BLE_GAPM_EVT_MAX,
};

/**@brief BLE GAP Connection Control Events. */
enum  BLE_GAPC_EVTS
{
    BLE_GAPC_EVT_PHY_UPDATED    = BLE_GAPC_EVT_BASE,   /**< PHY Update event. */
    BLE_GAPC_EVT_CONNECTED,                            /**< Connected event. */
    BLE_GAPC_EVT_DISCONNECTED,                         /**< Disconnected event. */
    BLE_GAPC_EVT_CONNECT_CANCLE,                       /**< Connect Cancle event. */
    BLE_GAPC_EVT_AUTO_CONN_TIMEOUT,                    /**< Auto Connect Timeout event. */
    BLE_GAPC_EVT_CONN_PARAM_UPDATED,                   /**< Connect Parameter Updated event. */
    BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ,                /**< Connect Parameter Request event. */
    BLE_GAPC_EVT_PEER_NAME_GOT,                        /**< peer Name Get event. */
    BLE_GAPC_EVT_CONN_INFO_GOT,                        /**< Connect Info Get event. */
    BLE_GAPC_EVT_PEER_INFO_GOT,                        /**< Peer Info Get event. */
    BLE_GAPC_EVT_DATA_LENGTH_UPDATED,                  /**< Data Length Updated event. */
    BLE_GAPC_EVT_DEV_INFO_SET,                         /**< Device Info Set event. */
    BLE_GAPC_EVT_CONNECT_IQ_REPORT,                    /**< Connection IQ Report info event. */
    BLE_GAPC_EVT_CONNECTLESS_IQ_REPORT,                /**< Connectionless IQ Report info event. */
    BLE_GAPC_EVT_LOCAL_TX_POWER_READ,                  /**< Local transmit power read indication info event. */
    BLE_GAPC_EVT_REMOTE_TX_POWER_READ,                 /**< Remote transmit power read indication info event. */
    BLE_GAPC_EVT_TX_POWER_CHANGE_REPORT,               /**< Transmit power change reporting info event. */
    BLE_GAPC_EVT_PATH_LOSS_THRESHOLD_REPORT,           /**< Path loss threshold reporting info event. */
    BLE_GAPC_EVT_RANGING_IND,                          /**< Ranging indication event. */
    BLE_GAPC_EVT_RANGING_SAMPLE_REPORT,                /**< Ranging sample report event. */
    BLE_GAPC_EVT_RANGING_CMP_IND,                      /**< Ranging complete indication event. */
    BLE_GAPC_EVT_DFT_SUBRATE_SET,                      /**< Default subrate param set complete event. */
    BLE_GAPC_EVT_SUBRATE_CHANGE_IND,                   /**< Subrate change indication event. */
    BLE_GAPC_EVT_MAX,
};

/**@brief BLE GATT Common Events. */
enum
{
    BLE_GATT_COMMON_EVT_MTU_EXCHANGE        = BLE_GATT_COMMON_EVT_BASE,     /**< MTU Exchange event. */
    BLE_GATT_COMMON_EVT_PRF_REGISTER,                                       /**< Service Register event. */
    BLE_GATT_COMMON_EVT_MAX,
};

/**@brief BLE GATTS Events. */
enum BLE_GATTS_EVTS
{
    BLE_GATTS_EVT_READ_REQUEST   = BLE_GATTS_EVT_BASE,   /**< GATTS Read Request event .*/
    BLE_GATTS_EVT_WRITE_REQUEST,                         /**< GATTS Write Request event .*/
    BLE_GATTS_EVT_PREP_WRITE_REQUEST,                    /**< GATTS Prepare Write Request event .*/
    BLE_GATTS_EVT_NTF_IND,                               /**< GATTS Notify or Indicate Complete event .*/
    BLE_GATTS_EVT_CCCD_RECOVERY,                         /**< GATTS CCCD Recovery event .*/
    BLE_GATTS_EVT_MULT_NTF,                              /**< GATTS Multiple Notifications event .*/
    BLE_GATTS_EVT_ENH_READ_REQUEST,                      /**< GATTS Enhance Read Request event .*/
    BLE_GATTS_EVT_ENH_WRITE_REQUEST,                     /**< GATTS Enhance Write Request event .*/
    BLE_GATTS_EVT_ENH_PREP_WRITE_REQUEST,                /**< GATTS Enhance Prepare Write Request event .*/
    BLE_GATTS_EVT_ENH_NTF_IND,                           /**< GATTS Enhance Notify or Indicate Complete event .*/
    BLE_GATTS_EVT_ENH_CCCD_RECOVERY,                     /**< GATTS Enhance CCCD Recovery event .*/
    BLE_GATTS_EVT_ENH_MULT_NTF,                          /**< GATTS Enhance Multiple Notifications event .*/
    BLE_GATTS_EVT_DATABASE_INITED_IND,
    BLE_GATTS_EVT_MAX,
};

/**@brief BLE GATTC Events. */
enum BLE_GATTC_EVTS
{
    BLE_GATTC_EVT_SRVC_BROWSE        = BLE_GATTC_EVT_BASE,         /**< GATTC Service Browse event .*/
    BLE_GATTC_EVT_PRIMARY_SRVC_DISC,                               /**< GATTC Primary Service Discovery event .*/
    BLE_GATTC_EVT_INCLUDE_SRVC_DISC,                               /**< GATTC Include Service Discovery event .*/
    BLE_GATTC_EVT_CHAR_DISC,                                       /**< GATTC Characteristic Discovery event .*/
    BLE_GATTC_EVT_CHAR_DESC_DISC,                                  /**< GATTC Characteristic Descriptor Discovery event .*/
    BLE_GATTC_EVT_READ_RSP,                                        /**< GATTC Read Response event. */
    BLE_GATTC_EVT_WRITE_RSP,                                       /**< GATTC Write Response event. */
    BLE_GATTC_EVT_NTF_IND,                                         /**< GATTC Notify or Indicate Receive event. */
    BLE_GATTC_EVT_CACHE_UPDATE,                                    /**< GATTC Cache Update event. */
    BLE_GATTC_EVT_ENH_SRVC_BROWSE,                                 /**< GATTC Service Browse event .*/
    BLE_GATTC_EVT_ENH_PRIMARY_SRVC_DISC,                           /**< GATTC Primary Service Discovery event .*/
    BLE_GATTC_EVT_ENH_INCLUDE_SRVC_DISC,                           /**< GATTC Include Service Discovery event .*/
    BLE_GATTC_EVT_ENH_CHAR_DISC,                                   /**< GATTC Characteristic Discovery event .*/
    BLE_GATTC_EVT_ENH_CHAR_DESC_DISC,                              /**< GATTC Characteristic Descriptor Discovery event .*/
    BLE_GATTC_EVT_ENH_READ_RSP,                                    /**< GATTC Read Response event. */
    BLE_GATTC_EVT_ENH_WRITE_RSP,                                   /**< GATTC Write Response event. */
    BLE_GATTC_EVT_ENH_NTF_IND,                                     /**< GATTC Notify or Indicate Receive event. */
    BLE_GATTC_EVT_MAX,
};

/**@brief BLE SEC Events. */
enum BLE_SEC_EVTS
{
    BLE_SEC_EVT_LINK_ENC_REQUEST   = BLE_SEC_EVT_BASE,         /**< Link Encrypte Request event. */
    BLE_SEC_EVT_LINK_ENCRYPTED,                                /**< Link Encrypted event. */
    BLE_SEC_EVT_KEY_PRESS_NTF,                                 /**< Key Press event. */
    BLE_SEC_EVT_KEY_MISSING,                                   /**< Key Missing event. */
    BLE_SEC_EVT_MAX,
};

/**@brief BLE L2CAP Events. */
enum  BLE_L2CAP_EVTS
{
    BLE_L2CAP_EVT_CONN_REQ    = BLE_L2CAP_EVT_BASE,          /**< L2cap Connect Request event. */
    BLE_L2CAP_EVT_CONN_IND,                                  /**< L2cap Connected Indicate event. */
    BLE_L2CAP_EVT_ADD_CREDITS_IND,                           /**< L2cap Credits Add Indicate event. */
    BLE_L2CAP_EVT_DISCONNECTED,                              /**< L2cap Disconnected event. */
    BLE_L2CAP_EVT_SDU_RECV,                                  /**< L2cap SDU Receive event. */
    BLE_L2CAP_EVT_SDU_SEND,                                  /**< L2cap SDU Send event. */
    BLE_L2CAP_EVT_ADD_CREDITS_CPLT,                          /**< L2cap Credits Add Completed event. */
    BLE_L2CAP_EVT_ENH_CONN_REQ,                              /**< L2cap Enhanced Connect Request event. */
    BLE_L2CAP_EVT_ENH_CONN_IND,                              /**< L2cap Enhanced Connected Indicate event. */
    BLE_L2CAP_EVT_ENH_RECONFIG_CPLT,                         /**< L2cap Enhanced Reconfig Completed event. */
    BLE_L2CAP_EVT_ENH_RECONFIG_IND,                          /**< L2cap Enhanced Reconfig Indicate event. */
    BLE_L2CAP_EVT_MAX,
};
/** @} */

/** @addtogroup BLE_COMMEN_STRUCTURES Structures
 * @{
 */
/**@brief BLE Event Information. */
typedef struct
{
    uint16_t    evt_id;                            /**< Event ID. */
    uint16_t    evt_status;                        /**< Event Status. */
    union
    {
        ble_gapm_evt_t          gapm_evt;          /**< BLE GAP Management Event. */
        ble_gapc_evt_t          gapc_evt;          /**< BLE GAP Connection Control Event. */
        ble_gatts_evt_t         gatts_evt;         /**< BLE GATT Server Event. */
        ble_gattc_evt_t         gattc_evt;         /**< BLE GATT Client Event. */
        ble_gatt_common_evt_t   gatt_common_evt;   /**< BLE GATT Common Event. */
        ble_sec_evt_t           sec_evt;           /**< BLE Security Event. */
        ble_l2cap_evt_t         l2cap_evt;         /**< BLE L2CAP Event. */
    } evt;                                         /**< BLE Event parameter. */
} ble_evt_t;
/** @} */

/** @addtogroup BLE_EVENT_TYPEDEF Typedefs
 * @{
 */
/**@brief The BLE event handler type. */
typedef void (*ble_evt_handler_t)(const ble_evt_t *p_evt);
/** @} */

#endif
/** @} */
/** @} */





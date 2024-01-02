/**
 *****************************************************************************************
 *
 * @file mlmr_c.h
 *
 * @brief Header file - Multi Link Multi Role Service Client
 *
 *****************************************************************************************
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
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_MLMR_C Multi Link Multi Role Service Client (MLMR_C)
 * @{
 * @brief Multi Link Multi Role Service Client module.
 *
 * @details The Multi Link Multi Role Service Client contains the APIs and types, which can be used by the
 *          application to perform scanning, connection and discover Multi Link Multi Role Service at
 *          peer and interact with it.
 *
 *          The application must provide an event handler, then call \ref mlmr_client_init(). After the
 *          module can send and receive BLE data, application can call \ref mlmr_c_tx_data_send() to
 *          send data to peer, and receive data from peer \ref MLMR_C_EVT_PEER_DATA_RECEIVE,
 *          meanwhile update its received BLE data state \ref mlmr_c_rx_flow_ctrl_set() to peer.
 */

#ifndef __MLMR_C_H__
#define __MLMR_C_H__

#include "ble_prf_types.h"
#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup MLMR_C_MACRO Defines
 * @{
 */
#define MLMR_C_CONNECTION_MAX                10                                      /**< Maximum number of MLMR Client connections. */

/**
 * @defgroup MLMR_C_UUID Service and Characteristics UUID
 * @{
 */
#define MLMR_C_SVC_UUID       {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                               0x0A, 0x46, 0x44, 0xD3, 0x01, 0x02, 0xED, 0xA6}       /**< UUID of MLMR_C Service. */
#define MLMR_C_TX_CHAR_UUID   {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                               0x0A, 0x46, 0x44, 0xD3, 0x02, 0x02, 0xED, 0xA6}       /**< UUID of MLMR_C Tx characterisitc. */
#define MLMR_C_RX_CHAR_UUID   {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                               0x0A, 0x46, 0x44, 0xD3, 0x03, 0x02, 0xED, 0xA6}       /**< UUID of MLMR_C Rx characterisitc. */
#define MLMR_C_FLOW_CTRL_UUID {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                               0x0A, 0x46, 0x44, 0xD3, 0x04, 0x02, 0xED, 0xA6}       /**< UUID of MLMR_C Flow Control characterisitc. */
/** @} */
/** @} */

/**
 * @defgroup MLMR_C_ENUM Enumerations
 * @{
 */
/**@brief Multi Link Multi Role Service Client event type. */
typedef enum
{
    MLMR_C_EVT_INVALID,                      /**< Invalid MLMR Client event. */
    MLMR_C_EVT_DISCOVERY_COMPLETE,           /**< MLMR Client has found service and its characteristics at peer. */
    MLMR_C_EVT_DISCOVERY_FAIL,               /**< MLMR Client found the service failed because of invalid operation or no found at peer. */
    MLMR_C_EVT_TX_NTF_SET_SUCCESS,           /**< MLMR Client has set peer Tx notify. */
    MLMR_C_EVT_FLOW_CTRL_NTF_SET_SUCCESS,    /**< MLMR Client has set peer ble flow control notify. */
    MLMR_C_EVT_PEER_DATA_RECEIVE,            /**< MLMR Client has received something from peer. */
    MLMR_C_EVT_TX_CPLT,                      /**< MLMR Client has sent something to peer successfully. */
    MLMR_C_EVT_TX_FLOW_OFF,                  /**< MLMR Client has received Tx flow off control request from peer. */
    MLMR_C_EVT_TX_FLOW_ON,                   /**< MLMR Client has received Tx flow on control request from peer. */
    MLMR_C_EVT_RX_FLOW_UPDATE_CPLT,          /**< MLMR CLient has updated flow control to peer completely. */
    MLMR_C_EVT_WRITE_OP_ERR,                 /**< Error occured when MLMR Client wrote to peer. */
} mlmr_c_evt_type_t;

/**@brief Flow control state for MLMR Client service. */
enum mlmr_c_flow_ctrl_state
{
  MLMR_C_FLOW_CTRL_STATE_OFF = 0,      /**< Indicate that MLMR Client can not receive data from peer. */
  MLMR_C_FLOW_CTRL_STATE_ON            /**< Indicate that MLMR Client can receive data from peer. */
};
/**@brief Underlying type used for the MLMR Client flow control state. */
typedef uint8_t mlmr_c_flow_ctrl_state_t;
/** @} */

/**
 * @defgroup MLMR_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t mlmr_c_srvc_start_handle;     /**< MLMR_C Service start handle. */
    uint16_t mlmr_c_srvc_end_handle;       /**< MLMR_C Service end handle. */
    uint16_t mlmr_c_tx_handle;             /**< Handle of MLMR_C Tx characteristic as provided by a discovery. */
    uint16_t mlmr_c_tx_cccd_handle;        /**< Handle of CCCD of MLMR_C Tx characteristic as provided by a discovery. */
    uint16_t mlmr_c_rx_handle;             /**< Handle of MLMR_C Rx characteristic as provided by a discovery. */
    uint16_t mlmr_c_flow_ctrl_handle;      /**< Handle of MLMR_C Flow Control characteristic as provided by a discovery. */
    uint16_t mlmr_c_flow_ctrl_cccd_handle; /**< Handle of CCCD of MLMR_C Flow Control characteristic as provided by a discovery. */
} mlmr_c_handles_t;

/**@brief Multi Link Multi Role Service Client event. */
typedef struct
{
    uint8_t            conn_idx;           /**< Connection index. */
    mlmr_c_evt_type_t  evt_type;           /**< MLMR Client event type. */
    uint16_t           length;             /**< Length of event data. */
    uint8_t           *p_data;             /**< Pointer to event data. */
} mlmr_c_evt_t;
/** @} */

/**
 * @defgroup MLMR_C_TYPEDEF Typedefs
 * @{
 */
/**@brief Multi Link Multi Role Service Client event handler type. */
typedef void (* mlmr_c_evt_handler_t)(mlmr_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup MLMR_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register MLMR Client event handler.
 *
 * @param[in] evt_handler: Multi Link Multi Role Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t mlmr_client_init(mlmr_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery MLMR_C on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t mlmr_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer MLMR_C Tx characteristic notify.
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] is_enable: Enable or disable ths Tx notify.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t mlmr_c_tx_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Enable or disable peer device MLMR_C flow control notify.
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] is_enable: Enable or disable ths Tx notify.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t mlmr_c_flow_ctrl_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Send data to the server.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data need sent.
 * @param[in] length:   Length of data need sent.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t mlmr_c_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Send MLMR Client Rx flow control state to peer device
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] flow_ctrl: MLMR client Rx flow control state.
 *
 * @return Result of sending mlmr_c Rx flow control state.
 *****************************************************************************************
 */
sdk_err_t mlmr_c_rx_flow_ctrl_set(uint8_t conn_idx, mlmr_c_flow_ctrl_state_t flow_ctrl);
/** @} */
#endif
/** @} */
/** @} */


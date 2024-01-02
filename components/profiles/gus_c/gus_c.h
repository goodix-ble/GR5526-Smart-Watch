/**
 *****************************************************************************************
 *
 * @file gus_c.h
 *
 * @brief Header file - Goodix UART Service Client
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
 * @defgroup BLE_SDK_GUS_C Goodix UART Service Client (GUS_C)
 * @{
 * @brief Goodix UART Service Client module.
 *
 * @details The Goodix Uart Service Client contains the APIs and types, which can be used by the 
 *          application to perform scanning, connection and discover Goodix Uart Service at 
 *          peer and interact with it.
 *
 *          The application must provide an event handler, then call \ref gus_client_init(). After the
 *          module can send and receive BLE data, application can call \ref gus_c_tx_data_send() to 
 *          send data to peer, and receive data from peer \ref GUS_C_EVT_PEER_DATA_RECEIVE,
 *          meanwhile update its received BLE data state \ref gus_c_rx_flow_ctrl_set() to peer.
 */

#ifndef __GUS_C_H__
#define __GUS_C_H__

#include "ble_prf_types.h"
#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup GUS_C_MACRO Defines
 * @{
 */
#define GUS_C_CONNECTION_MAX                10                                   /**< Maximum number of GUS Client connections. */

/**
 * @defgroup GUS_UUID Service and Characteristics UUID
 * @{
 */
#define GUS_SVC_UUID       {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                            0x0A, 0x46, 0x44, 0xD3, 0x01, 0x02, 0xED, 0xA6}       /**< UUID of GUS Service. */
#define GUS_TX_CHAR_UUID   {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                            0x0A, 0x46, 0x44, 0xD3, 0x02, 0x02, 0xED, 0xA6}       /**< UUID of GUS Tx characterisitc. */
#define GUS_RX_CHAR_UUID   {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                            0x0A, 0x46, 0x44, 0xD3, 0x03, 0x02, 0xED, 0xA6}       /**< UUID of GUS Rx characterisitc. */
#define GUS_FLOW_CTRL_UUID {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                            0x0A, 0x46, 0x44, 0xD3, 0x04, 0x02, 0xED, 0xA6}       /**< UUID  of GUS Flow Control characterisitc. */
/** @} */
/** @} */

/**
 * @defgroup GUS_C_ENUM Enumerations
 * @{
 */
/**@brief Goodix UART Service Client event type. */
typedef enum
{
    GUS_C_EVT_INVALID,                      /**< Invalid GUS Client event. */
    GUS_C_EVT_DISCOVERY_COMPLETE,           /**< GUS Client has found service and its characteristics at peer. */
    GUS_C_EVT_DISCOVERY_FAIL,               /**< GUS Client found THS service failed because of invalid operation or no found at peer. */
    GUS_C_EVT_TX_NTF_SET_SUCCESS,           /**< GUS Client has set peer Tx notify. */
    GUS_C_EVT_FLOW_CTRL_NTF_SET_SUCCESS,    /**< GUS Client has set peer ble flow control notify. */
    GUS_C_EVT_PEER_DATA_RECEIVE,            /**< GUS Client has received something from peer. */
    GUS_C_EVT_TX_CPLT,                      /**< GUS Client has sent something to peer successfully. */
    GUS_C_EVT_TX_FLOW_OFF,                  /**< GUS Client has received Tx flow off control request from peer. */
    GUS_C_EVT_TX_FLOW_ON,                   /**< GUS Client has received Tx flow on control request from peer. */
    GUS_C_EVT_RX_FLOW_UPDATE_CPLT,          /**< GUS CLient has updated flow control to peer completely. */
    GUS_C_EVT_WRITE_OP_ERR,                 /**< Error occured when GUS Client wrote to peer. */
} gus_c_evt_type_t;

/**@brief Flow control state for GUS Client service. */
enum gus_c_flow_ctrl_state
{
  GUS_C_FLOW_CTRL_STATE_OFF = 0,      /**< Indicate that GUS Client can not receive data from peer. */
  GUS_C_FLOW_CTRL_STATE_ON            /**< Indicate that GUS Client can receive data from peer. */
};
/**@brief Underlying type used for the GUS Client flow control state. */
typedef uint8_t gus_c_flow_ctrl_state_t;
/** @} */

/**
 * @defgroup GUS_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t gus_srvc_start_handle;     /**< GUS Service start handle. */
    uint16_t gus_srvc_end_handle;       /**< GUS Service end handle. */
    uint16_t gus_tx_handle;             /**< Handle of GUS Tx characteristic as provided by a discovery. */
    uint16_t gus_tx_cccd_handle;        /**< Handle of CCCD of GUS Tx characteristic as provided by a discovery. */
    uint16_t gus_rx_handle;             /**< Handle of GUS Rx characteristic as provided by a discovery. */
    uint16_t gus_flow_ctrl_handle;      /**< Handle of GUS Flow Control characteristic as provided by a discovery. */
    uint16_t gus_flow_ctrl_cccd_handle; /**< Handle of CCCD of GUS Flow Control characteristic as provided by a discovery. */
} gus_c_handles_t;

/**@brief Goodix UART Service Client event. */
typedef struct
{
    uint8_t           conn_idx;           /**< Connection index. */
    gus_c_evt_type_t  evt_type;           /**< GUS Client event type. */
    uint16_t          length;             /**< Length of event data. */
    uint8_t          *p_data;             /**< Pointer to event data. */
} gus_c_evt_t;
/** @} */

/**
 * @defgroup GUS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief Goodix UART Service Client event handler type. */
typedef void (* gus_c_evt_handler_t)(gus_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup GUS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register GUS Client event handler.
 *
 * @param[in] evt_handler: Goodix UART Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t gus_client_init(gus_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery GUS on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t gus_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer GUS Tx characteristic notify.
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] is_enable: Enable or disable ths Tx notify.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t gus_c_tx_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Enable or disable peer device GUS flow control notify.
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] is_enable: Enable or disable ths Tx notify.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t gus_c_flow_ctrl_notify_set(uint8_t conn_idx, bool is_enable);

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
sdk_err_t gus_c_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Send GUS Client Rx flow control state to peer device
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] flow_ctrl: GUS client Rx flow control state.
 *
 * @return Result of sending gus_c Rx flow control state.
 *****************************************************************************************
 */
sdk_err_t gus_c_rx_flow_ctrl_set(uint8_t conn_idx, gus_c_flow_ctrl_state_t flow_ctrl);
/** @} */
#endif
/** @} */
/** @} */


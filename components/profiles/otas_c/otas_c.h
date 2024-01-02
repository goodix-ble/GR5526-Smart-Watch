/**
 *****************************************************************************************
 *
 * @file otas_c.h
 *
 * @brief Over The Air Service Client API
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
 */

/**
 * @defgroup BLE_SDK_OTAS_C OTA Service Client (OTAS_C)
 * @{
 * @brief OTAS Client Interface module.
 *
 * @details The OTA Service Client contains the APIs and types, which can be used by the
 *          application to perform discovery of OTA Service at peer and interact with it.
 *
 *          The application must provide to register, then call \ref otas_client_init().
 *          The module can send firmware information data to peer.
 */

#ifndef __OTAS_C_H__
#define __OTAS_C_H__

#include "ble_prf_types.h"
#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup OTAS_C_MACRO Defines
 * @{
 */
#define OTAS_C_CONNECTION_MAX     10                                                 /**< Maximum number of OTAS Client connections. */

/**
 * @defgroup OTAS_UUID OTAS UUIDs
 * @{
 * @brief OTAS service and characteristcs UUID.
 */
#define OTAS_SVC_UUID           {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, \
                                 0x0A, 0x46, 0x44, 0xD3, 0x01, 0x04, 0xED, 0xA6}     /**< UUID of OTA Service. */
#define OTAS_TX_CHAR_UUID       {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, \
                                 0x0A, 0x46, 0x44, 0xD3, 0x02, 0x04, 0xED, 0xA6}     /**< UUID of OTA TX Characteristic. */
#define OTAS_RX_CHAR_UUID       {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, \
                                 0x0A, 0x46, 0x44, 0xD3, 0x03, 0x04, 0xED, 0xA6}     /**< UUID of OTA RX Characteristic. */
#define OTAS_CTRL_CHAR_UUID     {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, \
                                 0x0A, 0x46, 0x44, 0xD3, 0x04, 0x04, 0xED, 0xA6}     /**< UUID of OTA Control Characteristic. */
/** @} */
/** @} */

/**
 * @defgroup OTAS_C_ENUM Enumerations
 * @{
 */
/**@brief OTA Service Client event type. */
typedef enum
{
    OTAS_C_EVT_INVALID,                      /**< OTA Client invalid event. */
    OTAS_C_EVT_DISCOVERY_COMPLETE,           /**< OTA Client has found THS service and its characteristics. */
    OTAS_C_EVT_DISCOVERY_FAIL,               /**< OTA Client found THS service failed because of invalid operation or no found at the peer. */
    OTAS_C_EVT_TX_NTF_SET_SUCCESS,           /**< OTA Client has set peer Tx notify. */
    OTAS_C_EVT_CTRL_SUCCESS,                 /**< OTA Client has set control info. */
    OTAS_C_EVT_PEER_DATA_RECEIVE,            /**< OTA Client has received data from peer. */
    OTAS_C_EVT_TX_CPLT,                      /**< OTA Client has sent something to a peer successfully. */
    OTAS_C_EVT_WRITE_OP_ERR,                 /**< Error occured when OTA Client writen to peer. */
} otas_c_evt_type_t;
/** @} */

/**
 * @defgroup OTAS_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t otas_srvc_start_handle;       /**< OTA Service start handle. */
    uint16_t otas_srvc_end_handle;         /**< OTA Service end handle. */
    uint16_t otas_tx_handle;               /**< OTA tx characteristic handle which has been got from peer. */
    uint16_t otas_tx_cccd_handle;          /**< OTA tx characteristic CCCD handle which has been got from peer. */
    uint16_t otas_rx_handle;               /**< OTA rx characteristic handle which has been got from peer. */
    uint16_t otas_ctrl_handle;             /**< OTA control characteristic handle which has been got from peer. */
} otas_c_handles_t;

/**@brief OTA Service Client event. */
typedef struct
{
    uint8_t           conn_idx;            /**< The connection index. */
    otas_c_evt_type_t evt_type;            /**< OTA client event type. */
    uint8_t          *p_data;              /**< Pointer to event data. */
    uint16_t          length;              /**< Length of event data. */
} otas_c_evt_t;
/** @} */

/**
 * @defgroup OTAS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief OTA Service Client event handler type. */
typedef void (*otas_c_evt_handler_t)(otas_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup OTAS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register THS Client event handler.
 *
 * @param[in] evt_handler: OTA Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t otas_client_init(otas_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery OTAS on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t otas_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer OTA tx characteristic notify.
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] is_enable: Enable or disable OTA tx notify.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t otas_c_tx_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Send control data to peer.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] data:     Control data.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t otas_c_ctrl_data_send(uint8_t conn_idx, uint32_t data);

/**
 *****************************************************************************************
 * @brief Send data to peer.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data to be sent.
 * @param[in] length:   Length of data to be sent.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t otas_c_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);
/** @} */
#endif

/** @} */
/** @} */

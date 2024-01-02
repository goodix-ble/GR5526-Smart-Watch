/**
 *****************************************************************************************
 *
 * @file tps_c.h
 *
 * @brief Tx Power Service Client API
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
 * @defgroup BLE_SDK_TPS_C Tx Power Service Client (TPS_C)
 * @{
 * @brief Tx Power Service Client module.
 *
 * @details The Tx Power Service Client contains the APIs and types, which can be used by the
 *          application to discovery of Tx Power Service of peer and interact with it.
 *
 *          The application must provide an event handler to register, then call \ref tps_client_init().
 *          After Tx Power Service Client discoveries peer Tx Power Service, application can call
 *          \ref tps_c_tx_power_level_read() to get tx power data from peer.
 */

#ifndef __TPS_C_H__
#define __TPS_C_H__

#include "gr_includes.h"
#include "ble_prf_types.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup TPS_C_MACRO Defines
 * @{
 */
#define TPS_C_CONNECTION_MAX                10       /**< Maximum number of TPS Client connections. */
/** @} */

/**
 * @defgroup TPS_C_ENUM Enumerations
 * @{
 */
/**@brief Tx Power Service Client event type. */
typedef enum
{
    TPS_C_EVT_INVALID,                      /**< TPS Client invalid event. */
    TPS_C_EVT_DISCOVERY_COMPLETE,           /**< TPS Client has found TPS service and its characteristics. */
    TPS_C_EVT_DISCOVERY_FAIL,               /**< TPS Client found TPS service failed because of invalid operation or no found at the peer. */
    TPS_C_EVT_TX_POWER_LEVEL_RECEIVE,       /**< TPS Client has received Tx Power Level value. */
} tps_c_evt_type_t;
/** @} */

/**
 * @defgroup TPS_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t tps_srvc_start_handle;      /**< TPS Service start handle. */
    uint16_t tps_srvc_end_handle;        /**< TPS Service end handle. */
    uint16_t tps_tx_power_level_handle;  /**< TPS Tx Power Level characteristic Value handle which has been got from peer. */
} tps_c_handles_t;

/**@brief Tx Power Service Client event. */
typedef struct
{
    uint8_t          conn_idx;            /**< The connection index. */
    tps_c_evt_type_t evt_type;            /**< TPS Client event type. */
    int8_t           tx_power_level;      /**< Tx Power level. */
} tps_c_evt_t;
/** @} */

/**
 * @defgroup TPS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief  Tx Power Service Client event handler type. */
typedef void (*tps_c_evt_handler_t)(tps_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup TPS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register TPS Client event handler.
 *
 * @param[in] evt_handler: Tx Power Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t tps_client_init(tps_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery Tx Power Service on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t tps_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Read Tx Power Level characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t tps_c_tx_power_level_read(uint8_t conn_idx);
/** @} */

#endif
/** @} */
/** @} */

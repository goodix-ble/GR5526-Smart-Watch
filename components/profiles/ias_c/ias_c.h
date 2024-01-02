/**
 *****************************************************************************************
 *
 * @file ias_c.h
 *
 * @brief Immediate Alert Service Client API
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
 * @defgroup BLE_SDK_IAS_C Immediate Alert Service Client (IAS_C)
 * @{
 * @brief Immediate Alert Service Client module.
 *
 * @details The Immediate Alert Service Client contains the APIs and types, which can be used by the
 *          application to discovery of Immediate Alert Service of peer and interact with it.
 *
 *          The application must provide an event handler to register, then call \ref ias_client_init().
 *          After Immediate Alert Service Client discoveries peer Immediate Alert Service, application can call
 *          \ref ias_c_alert_level_set() to set alert level on peer.
 */

#ifndef __IAS_C_H__
#define __IAS_C_H__

#include "gr_includes.h"
#include "ble_prf_types.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup IAS_C_MACRO Defines
 * @{
 */
#define IAS_C_CONNECTION_MAX                10                        /**< Maximum number of IAS Client connections. */
/** @} */

/**
 * @defgroup IAS_C_ENUM Enumerations
 * @{
 */
/**@brief Immediate Alert Service Alert levels. */
typedef enum
{
    IAS_C_ALERT_NONE,     /**< No alert. */
    IAS_C_ALERT_MILD,     /**< Mild alert. */
    IAS_C_ALERT_HIGH,     /**< High alert. */
} ias_c_alert_level_t;

/**@brief Immediate Alert Service Client event type. */
typedef enum
{
    IAS_C_EVT_INVALID,                      /**< IAS Client invalid event. */
    IAS_C_EVT_DISCOVERY_COMPLETE,           /**< IAS Client has found BAS service and its characteristics. */
    IAS_C_EVT_DISCOVERY_FAIL,               /**< IAS Client found BAS service failed because of invalid operation or no found at the peer. */
    IAS_C_EVT_ALERT_LEVEL_SET_SUCCESS,      /**< IAS Client has set Alert Level characteristic. */
    IAS_C_EVT_ALERT_LEVEL_SET_ERR,          /**< Error occured when IAS Client set Alert Level characteristic. */
} ias_c_evt_type_t;
/** @} */

/**
 * @defgroup IAS_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t ias_srvc_start_handle;      /**< IAS Service start handle. */
    uint16_t ias_srvc_end_handle;        /**< IAS Service end handle. */
    uint16_t ias_alert_level_handle;     /**< IAS Alert Level characteristic Value handle which has been got from peer. */
} ias_c_handles_t;

/**@brief Immediate Alert Service Client event. */
typedef struct
{
    uint8_t          conn_idx;            /**< The connection index. */
    ias_c_evt_type_t evt_type;            /**< IAS Client event type. */
} ias_c_evt_t;
/** @} */

/**
 * @defgroup IAS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief  Immediate Alert Service Client event handler type. */
typedef void (*ias_c_evt_handler_t)(ias_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup IAS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register IAS Client event handler.
 *
 * @param[in] evt_handler: Immediate Alert Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t ias_client_init(ias_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery Immediate Alert Service on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ias_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer Alert Level characteristic notify.
 *
 * @param[in] conn_idx:    Index of connection.
 * @param[in] alert_level: Alert level.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ias_c_alert_level_set(uint8_t conn_idx, ias_c_alert_level_t alert_level);

/** @} */

#endif
/** @} */
/** @} */

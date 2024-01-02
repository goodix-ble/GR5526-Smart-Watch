/**
 *****************************************************************************************
 *
 * @file lls_c.h
 *
 * @brief Link Loss Service Client API
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
 * @defgroup BLE_SDK_LLS_C Link Loss Service Client (LLS_C)
 * @{
 * @brief Link Loss Service Client module.
 *
 * @details The Link Loss Service Client contains the APIs and types, which can be used by the
 *          application to discovery of Link Loss Service of peer and interact with it.
 *
 *          The application must provide an event handler to register, then call \ref lls_client_init().
 *          After Link Loss Service Client discoveries peer Link Loss Service, application can call
 *          \ref lls_c_alert_level_set() and \ref lls_c_alert_level_read() to get Alert Level from peer.
 */

#ifndef __LLS_C_H__
#define __LLS_C_H__

#include "gr_includes.h"
#include "ble_prf_types.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup LLS_C_MACRO Defines
 * @{
 */
#define LLS_C_CONNECTION_MAX                10                       /**< Maximum number of LLS Client connections. */
/** @} */

/**
 * @defgroup LLS_C_ENUM Enumerations
 * @{
 */
/**@brief  Link Loss Service Client alert levels. */
typedef enum
{
    LLS_C_ALERT_LEVEL_NO_ALERT,       /**< No alert. */
    LLS_C_ALERT_LEVEL_MILD_ALERT,     /**< Mild alert. */
    LLS_C_ALERT_LEVEL_HIGH_ALERT,     /**< High alert. */
} lls_c_alert_level_t;

/**@brief Link Loss Service Client event type. */
typedef enum
{
    LLS_C_EVT_INVALID,                      /**< LLS Client invalid event. */
    LLS_C_EVT_DISCOVERY_COMPLETE,           /**< LLS Client has found LLS service and its characteristics. */
    LLS_C_EVT_DISCOVERY_FAIL,               /**< LLS Client found LLS service failed because of invalid operation or no found at the peer. */
    LLS_C_EVT_ALERT_LEVEL_SET_SUCCESS,      /**< LLS Client has set Alert Level characteristics. */
    LLS_C_EVT_ALERT_LEVEL_SET_ERR,          /**< Error occured when LLS Client set Alert Level characteristics. */
    LLS_C_EVT_ALERT_LEVEL_RECEIVE,          /**< LLS Client has received Alert Level value. */
} lls_c_evt_type_t;
/** @} */

/**
 * @defgroup LLS_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t lls_srvc_start_handle;      /**< LLS Service start handle. */
    uint16_t lls_srvc_end_handle;        /**< LLS Service end handle. */
    uint16_t lls_alert_level_handle;     /**< LLS Alert Level characteristic Value handle which has been got from peer. */
} lls_c_handles_t;

/**@brief Link Loss Service Client event. */
typedef struct
{
    uint8_t                 conn_idx;            /**< The connection index. */
    lls_c_evt_type_t        evt_type;            /**< LLS Client event type. */
    lls_c_alert_level_t     alert_level;         /**< Alert level. */
} lls_c_evt_t;
/** @} */

/**
 * @defgroup LLS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief  Link Loss Service Client event handler type. */
typedef void (*lls_c_evt_handler_t)(lls_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup LLS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register LLS Client event handler.
 *
 * @param[in] evt_handler: Link Loss Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t lls_client_init(lls_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery Link Loss Service on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t lls_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Set peer Alert Level.
 *
 * @param[in] conn_idx:     Index of connection.
 * @param[in] alert_level:  Alert level.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t lls_c_alert_level_set(uint8_t conn_idx, lls_c_alert_level_t alert_level);

/**
 *****************************************************************************************
 * @brief Read Alert Level characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t lls_c_alert_level_read(uint8_t conn_idx);
/** @} */

#endif
/** @} */
/** @} */

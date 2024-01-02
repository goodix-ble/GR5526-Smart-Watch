/**
 *****************************************************************************************
 *
 * @file bas_c.h
 *
 * @brief Battery Service Client API
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
 * @defgroup BLE_SDK_BAS_C Battery Service Client (BAS_C)
 * @{
 * @brief Battery Service Client module.
 *
 * @details The Battery Service Client contains the APIs and types, which can be used by the
 *          application to discovery of Battery Service of peer and interact with it.
 *
 *          The application must provide an event handler to register, then call \ref bas_client_init().
 *          After Battery Service Client discoveries peer Battery Service, application can call
 *          \ref bas_c_bat_level_notify_set() and \ref bas_c_bat_level_read() to get battery
 *          data from peer.
 */

#ifndef __BAS_C_H__
#define __BAS_C_H__

#include "gr_includes.h"
#include "ble_prf_types.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup BAS_C_MACRO Defines
 * @{
 */
#define BAS_C_CONNECTION_MAX                10       /**< Maximum number of BAS Client connections. */
/** @} */

/**
 * @defgroup BAS_C_ENUM Enumerations
 * @{
 */
/**@brief Battery Service Client event type. */
typedef enum
{
    BAS_C_EVT_INVALID,                      /**< BAS Client invalid event. */
    BAS_C_EVT_DISCOVERY_COMPLETE,           /**< BAS Client has found BAS service and its characteristics. */
    BAS_C_EVT_DISCOVERY_FAIL,               /**< BAS Client found BAS service failed because of invalid operation or no found at the peer. */
    BAS_C_EVT_BAT_LEVEL_NTF_SET_SUCCESS,    /**< BAS Client has enabled Notification of Battery Level characteristics. */
    BAS_C_EVT_BAT_LEVEL_NTF_SET_ERR,        /**< Error occured when BAS Client set Notification of Battery Level characteristics. */
    BAS_C_EVT_BAT_LEVE_RECEIVE,             /**< BAS Client has received Battery Level value (Read or Notification from peer). */
} bas_c_evt_type_t;
/** @} */

/**
 * @defgroup BAS_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t bas_srvc_start_handle;      /**< BAS Service start handle. */
    uint16_t bas_srvc_end_handle;        /**< BAS Service end handle. */
    uint16_t bas_bat_level_handle;       /**< BAS Battery Level characteristic Value handle which has been got from peer. */
    uint16_t bas_bat_level_cccd_handle;  /**< BAS CCCD handle of Battery Level characteristic which has been got from peer. */
    uint16_t bas_bat_level_pres_handle;  /**< BAS Presentation Format Descriptor handle of Battery Level characteristic which has been got from peer. */
} bas_c_handles_t;

/**@brief Battery Service Client event. */
typedef struct
{
    uint8_t          conn_idx;            /**< The connection index. */
    bas_c_evt_type_t evt_type;            /**< BAS Client event type. */
    uint8_t          bat_level;           /**< Battery level. */
} bas_c_evt_t;
/** @} */

/**
 * @defgroup BAS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief  Battery Service Client event handler type. */
typedef void (*bas_c_evt_handler_t)(bas_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup BAS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register BAS Client event handler.
 *
 * @param[in] evt_handler: Battery Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t bas_client_init(bas_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery Battery Service on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t bas_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer Battery Level characteristic notify.
 *
 * @param[in] conn_idx:  Index of connection.
 * @param[in] is_enable: true or false.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t bas_c_bat_level_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Read Battery Level characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t bas_c_bat_level_read(uint8_t conn_idx);
/** @} */

#endif
/** @} */
/** @} */

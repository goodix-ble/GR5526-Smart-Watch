/**
 *****************************************************************************************
 *
 * @file pass_c.h
 *
 * @brief Phone Alert Status Service Client API.
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
 * @defgroup BLE_SDK_PASS_C Phone Alert Status Service Client (PASS_C)
 * @{
 * @brief Phone Alert Status Service Client module.
 *
 * @details The Phone Alert Status Service Client contains the APIs and types, which can be used
 *          by the application to discovery of Phone Alert Status Service of peer and interact with it.
 *
 *          The application must provide an event handler to be register, then call \ref pass_client_init().
 *          After Phone Alert Status Service Client discoveries peer Phone Alert Statuse Service,
 *          application can call \ref pass_c_ctrl_point_set() to send ringer control point to peer,
 *          When ringer setting or alert status is changed, the module will receive notification from peer
 *          if notifications of them are enabled, application also can call \ref pass_c_ringer_set_read()
 *          and \ref pass_c_alert_status_read() to get new value of them on peer.
 */

#ifndef __PASS_C_H__
#define __PASS_C_H__

#include "ble_prf_types.h"
#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup PASS_C_MACRO Defines
 * @{
 */
/**
 * @defgroup PASS_C_ALERT_STATUES_BIT Alert Status BIT
 * @{
 * @brief PASS Alert Status bits.
 */
#define PASS_C_CONNECTION_MAX         10                           /**< Maximum number of HRS Client connections. */
#define PASS_C_NO_STATE_ACTIVE        (0x00)                       /**< Bit for no state active. */
#define PASS_C_RINGER_ACTIVE          (0x01 << 0)                  /**< Bit for ringer State active. */
#define PASS_C_VIBRATE_ACTIVE         (0x01 << 1)                  /**< Bit for vibrate State active. */
#define PASS_C_DISPLAY_ALERT_ACTIVE   (0x01 << 2)                  /**< Bit for display Alert Status State active. */
#define PASS_C_ALL_STATE_ACTIVE       (0x07)                       /**< Bit for no state active. */
/** @} */

#define PASS_C_RINGER_CTRL_PT_VAL_LEN 1                            /**< Length of Ringer Control Point value. */

/**
 * @defgroup PASS_C_RINGER_SETTING Ringer Setting
 * @{
 * @brief The Ringer Setting characteristic defines the setting of the ringer.
 */
#define PASS_C_RINGER_SET_SILENT      0                            /**< Ringer Silent. */
#define PASS_C_RINGER_SET_NORMAL      1                            /**< Ringer Normal. */
/** @} */
/** @} */

/**
 * @defgroup PASS_C_ENUM Enumerations
 * @{
 */
/**@brief Phone Alert Status Service Client Ringer Control Point. */
typedef enum
{
    PASS_C_CTRL_PT_SILENT_MODE = 0x01,    /**< Silent Mode. */
    PASS_C_CTRL_PT_MUTE_ONCE,             /**< Mute Once. */
    PASS_C_CTRL_PT_CANCEL_SLIENT_MODE,    /**< Cancel Silent Mode. */
} pass_c_ringer_ctrl_pt_t;

/**@brief Phone Alert Status Service Client event type. */
typedef enum
{
    PASS_C_EVT_INVALID,                         /*<* PASS Client invalid event. */
    PASS_C_EVT_DISCOVERY_COMPLETE,              /**< PASS Client has found PASS service and its characteristics. */
    PASS_C_EVT_DISCOVERY_FAIL,                  /**< PASS Client found PASS service failed because of invalid operation or no found at the peer. */
    PASS_C_EVT_ALERT_STATUS_NTF_SET_SUCCESS,    /**< PASS Client has set Notification of Alert Status characteristic. */
    PASS_C_EVT_RINGER_SET_NTF_SET_SUCCESS,      /**< PASS Client has set Notification of Ringer Setting characteristic. */
    PASS_C_EVT_ALERT_STATUS_RECEIVE,            /**< PASS Client has received Alert Status value (Read or Notification from peer). */
    PASS_C_EVT_RINGER_SET_RECEIVE,              /**< PASS Client has received Ringer Setting Value (Read or Notification from peer). */
    PASS_C_EVT_CTRL_POINT_SET_SUCCESS,          /**< PASS Client has writen Control Point completely. */
    PASS_C_EVT_WRITE_OP_ERR,                    /**< Error occured when PASS Client writen to peer. */
} pass_c_evt_type_t;

/** @} */

/**
 * @defgroup PASS_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t pass_srvc_start_handle;            /**< PASS Service start handle. */
    uint16_t pass_srvc_end_handle;              /**< PASS Service end handle. */
    uint16_t pass_alert_status_handle;          /**< PASS Alert Status characteristic Value handle which has been got from peer. */
    uint16_t pass_alert_status_cccd_handle;     /**< PASS CCCD handle of Alert Status characteristic which has been got from peer. */
    uint16_t pass_ringer_set_handle;            /**< PASS Ringer Setting characteristic Value handle which has been got from peer. */
    uint16_t pass_ringer_set_cccd_handle;       /**< PASS CCCD handle of Ringer Setting characteristic which has been got from peer. */
    uint16_t pass_ringer_ctrl_pt_handle;        /**< PASS Ringer Control Point characteristic Value handle which has been got from peer. */
} pass_c_handles_t;

/**@brief Phone Alert Status Client Service event. */
typedef struct
{
    uint8_t            conn_idx;        /**< The index of the connection. */
    pass_c_evt_type_t  evt_type;        /**< The PASS event type. */
    union
    {
        uint8_t        alert_status;    /**< Alert status received. */
        uint8_t        ringer_set;      /**< Ringer setting received. */
    } value;                            /**< Value received. */
} pass_c_evt_t;
/** @} */

/**
 * @defgroup PASS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief Phone Alert Status Service Client event handler type. */
typedef void (*pass_c_evt_handler_t)(pass_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup PASS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register PASS Client event handler.
 *
 * @param[in] evt_handler: Phone Alert Status Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t pass_client_init(pass_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery Phone Alert Status Service on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t pass_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer Alert Status characteristic notify.
 *
 * @param[in] conn_idx:  Index of connection.
 * @param[in] is_enable: True or false.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t pass_c_alert_status_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Enable or disable peer Ringer Setting characteristic notify.
 *
 * @param[in] conn_idx:  Index of connection.
 * @param[in] is_enable: True or false.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t pass_c_ringer_set_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Read Alert Status characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t pass_c_alert_status_read(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Read Ringer Setting characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t pass_c_ringer_set_read(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Set Control Point characteristic value.
 *
 * @param[in] conn_idx:   Index of connection.
 * @param[in] ctrl_value: Value of control point.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t pass_c_ctrl_point_set(uint8_t conn_idx, uint8_t ctrl_value);
/** @} */

#endif
/** @} */
/** @} */


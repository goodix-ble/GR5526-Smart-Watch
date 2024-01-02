/**
 ****************************************************************************************
 *
 * @file ble_connect.h
 *
 * @brief BLE Connect Module Header
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

#ifndef __BLE_CONNECT_H__
#define __BLE_CONNECT_H__

#include "gr_includes.h"
#include "app_timer.h"


/**
 * @defgroup BLE_CONNECT_MAROC Defines
 * @{
 */
#define RET_VERIFY_SUCCESS(RET_CODE)                 \
do                                                   \
{                                                    \
    if (RET_CODE != SDK_SUCCESS)                     \
    {                                                \
        return RET_CODE;                             \
    }                                                \
} while(0)

#define BLE_CONN_PARAM_MAX_SLAVE_LATENCY_DEVIATION            10    /**< The largest acceptable deviation in slave latency. */
#define BLE_CONN_PARAM_MAX_SUPERVISION_TIMEOUT_DEVIATION      100   /**< The largest acceptable deviation (in 10 ms units) in supervision timeout. */


#ifdef CFG_MAX_CONNECTIONS
    #define BLE_CONN_LINK_CNT_MAX      CFG_MAX_CONNECTIONS
#else
    #define BLE_CONN_LINK_CNT_MAX      10
#endif

/** @} */

/**
 * @defgroup BLE_CONNECT_ENUM Enumerations
 * @{
 */
/**@brief BLE connect state. */
typedef enum
{
    BLE_CONN_STATE_DISCONNECTED,        /**< BLE link disconnected state. */
    BLE_CONN_STATE_CONNECTED,           /**< BLE link connected state. */
} ble_conn_state_t;

/**@brief BLE connect events type. */
typedef enum
{
    BLE_CONN_EVT_INVALID,                /**< Invalid connect event. */
    BLE_CONN_EVT_CONNECTED,              /**< Connected event. */
    BLE_CONN_EVT_DISCONNECTED,           /**< Disconnected event. */
    BLE_CONN_EVT_PATAM_UPDATED,          /**< Connect param updated event. */
    BLE_CONN_EVT_PPCP_GET_FAIL,          /**< PPCP get fail. */
    BLE_CONN_EVT_PARAM_NEGO_FAILED,      /**< Negotiation procedure failed. */
    BLE_CONN_EVT_PARAM_NEGO_SUCCEEDED    /**< Negotiation procedure succeeded. */
} ble_conn_evt_type_t;
/** @} */

/**
 * @defgroup BLE_CONNECT_TYPEDEF Typedefs
 * @{
 */
/**@brief BLE connect role of LL layer. */
typedef ble_gap_ll_role_type_t  ble_conn_role_t;

/**@brief BLE connect parameter. */
typedef ble_gap_evt_conn_param_updated_t  ble_conn_param_t;

/**@brief All connected link need to execute handler type. */
typedef void (*ble_conn_all_link_exec_handler_t)(uint8_t conn_idx);

/**@brief BLE connect error handler type. */
typedef void (*ble_conn_err_handler_t)(uint8_t err_code);
/** @} */

/**
 * @defgroup BLE_CONNECT_STRUCT Structures
 * @{
 */
/**@brief BLE connect link information. */
typedef struct
{
    bool                    is_encrypt;               /**< Is encrypt or not. */
    ble_conn_state_t        conn_state;               /**< BLE connect state. */
    ble_conn_role_t         local_role;               /**< Local BLE LL role. */
    ble_gap_bdaddr_t        peer_addr;                /**< Peer address. */
    ble_conn_param_t        conn_param;               /**< BLE connect parameters. */
    ble_gap_conn_param_t    pref_conn_param;          /**< BLE prefer connect parameters. */
    uint8_t                 param_ok;                 /**< Whether the current connection parameters on this link are acceptable. */
    uint8_t                 update_count;             /**< The number of times the connection parameters have been attempted
                                                           negotiated on this link. */
    app_timer_id_t          timer_id;                 /**< Timer id of connect parameter update. */
} ble_conn_link_info_t;

/**@brief BLE connect event information. */
typedef struct
{
    ble_conn_evt_type_t  evt_type;
    uint8_t              conn_idx;
    union 
    {
        ble_conn_link_info_t *p_link_info;
        uint8_t               disconn_reason;
        ble_conn_param_t     *p_update_param;
    } param;
}ble_conn_evt_t;
/** @} */

/**
 * @defgroup BLE_CONNECT_TYPEDEF Typedefs
 * @{
 */
/**@brief BLE connect event handler type. */
typedef void (*ble_conn_evt_handler_t)(ble_conn_evt_t *p_evt);
/** @} */

/**
 * @defgroup BLE_CONNECT_STRUCT Structures
 * @{
 */
/**@brief BLE connect initialization. */
typedef struct
{
    bool                    conn_param_manage_enable;      /**< Enable or disable connect param auto update. */
    ble_gap_conn_param_t    *p_conn_param;                  /**< Pointer to the connection parameters desired by the application.It will be fetched from host if set to NULL. */
    uint32_t                first_conn_param_update_delay; /**< Time from connect to first time ble_gap_conn_param_update is called (in 1 ms). */
    uint32_t                next_conn_param_update_delay;  /**< Time between each call to ble_gap_conn_param_update after the first (in 1 ms). */
    uint8_t                 max_conn_param_update_count;   /**< Number of attempts before giving up the negotiation. */
    bool                    disconnect_on_fail;            /**< Set to TRUE if a failed connection parameters update shall cause an automatic disconnection. */
    ble_conn_evt_handler_t  evt_handler;                   /**< Event handler to be called for handling events in the Connection. */
    ble_conn_err_handler_t  err_handler;                   /**< Function to be called in case of an error. */
} ble_conn_init_t;
/** @} */


/**
 * @defgroup BLE_CONNECT_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize BLE connect module.
 *
 * @param[in] p_conn_init: Pointer to initialization params.
 * 
 * @return Result of ble connect initialization.
 *****************************************************************************************
 */
sdk_err_t ble_connect_init(ble_conn_init_t *p_conn_init);

/**
 *****************************************************************************************
 * @brief Get count of established BLE connect link.
 * 
 * @return Count of established ble connect link.
 *****************************************************************************************
 */
uint8_t ble_connect_established_cnt_get(void);

/**
 *****************************************************************************************
 * @brief Check BLE connect link is connected or not.
 *
 * @param[in] conn_idx:  Index of connection.
 *
 * @return Result of check.
 *****************************************************************************************
 */
bool ble_connect_is_connected(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Get connect link information.
 *
 * @param[in]     conn_idx:    Index of connection.
 * @param[in\out] p_link_info: Buffer for save information get.
 *
 * @return Result of get.
 *****************************************************************************************
 */
sdk_err_t ble_connect_link_info_get(uint8_t conn_idx, ble_conn_link_info_t *p_link_info);

/**
 *****************************************************************************************
 * @brief Set active connect link.
 *
 * @param[in] conn_idx:    Index of connection.
 *
 * @return Result of set.
 *****************************************************************************************
 */
sdk_err_t ble_connect_active_link_set(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Set active connect link.
 *
 * @param[in] conn_idx:    Index of connection.
 *
 * @return Result of get.
 *****************************************************************************************
 */
uint8_t ble_connect_active_link_get(void);

/**
 *****************************************************************************************
 * @brief All connected link need to execute handler.
 *
 * @param[in] exec_handler:  Execute handler.
 *
 * @return Result of check.
 *****************************************************************************************
 */
sdk_err_t ble_connect_all_to_do(ble_conn_all_link_exec_handler_t exec_handler);

/**
 *****************************************************************************************
 * @brief Get next connected link index.
 *
 * @param[in]  cur_conn_idx:    Index of cur_connection.
 * @param[in]  next_conn_idx:   Pointer to next connected link index.
 *
 * @return Result of get..
 *****************************************************************************************
 */
sdk_err_t ble_connect_next_link_idx_get(uint8_t cur_conn_idx, uint8_t *next_conn_idx);

 /**
 *****************************************************************************************
 * @brief Function for stopping the Connection Parameters manage module.

 * @return Result of stopping the Connection Parameters module.
 *****************************************************************************************
 */
sdk_err_t ble_connect_param_update_stop(void);

/**
 *****************************************************************************************
 * @brief Initialize parameter update request.
 *
 * @param[in] conn_idx: Index of connection.
 * 
 * @param[in] p_new_param: Pointer to the new connect parameter.
 * 
 * @return Result of ble advertising initialization.
 *****************************************************************************************
 */
sdk_err_t ble_connect_param_change(uint8_t conn_idx, ble_gap_conn_param_t *p_new_param);

void ble_connect_evt_on_ble_capture(const ble_evt_t *p_evt);

/** @} */

#endif

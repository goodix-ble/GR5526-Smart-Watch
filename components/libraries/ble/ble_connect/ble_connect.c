/**
 ****************************************************************************************
 *
 * @file ble_connect.c
 *
 * @brief BLE Connect Module API
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "ble_module_config.h"
#if BLE_CONNECT_ENABLE
#include "ble_connect.h"
#include "app_timer.h"
#include "utility.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define BLE_CONN_INTERVAL_MIN    0x0006          /**< Minimum value for the connection interval(7.5 ms). */
#define BLE_CONN_INTERVAL_MAX    0x0C80          /**< Maximum value for the connection interval(4 s). */
#define BLE_CONN_LATENCY_MAX     0x01F3          /**< Maximum value for the slave latency(499). */
#define BLE_CONN_TIMEOUT_MIN     0x000A          /**< Minimum value for the supervision timeout(100ms s). */
#define BLE_CONN_TIMEOUT_MAX     0x0C80          /**< Maximum value for the supervision timeout(32 s). */
#define BLE_TIMER_TIMEOUT_MAX    4000000         /**< Maximum value for the app_timer timeout(4000 s). */

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief BLE Connect Module environment variable. */
typedef struct
{
    uint8_t                act_conn_idx;
    bool                   conn_param_manage_enable;
    ble_conn_link_info_t   link_info[BLE_CONN_LINK_CNT_MAX];
    uint32_t               first_conn_param_update_delay;
    uint32_t               next_conn_param_update_delay;
    uint8_t                max_conn_param_update_count;
    bool                   disconnect_on_fail;
    ble_conn_evt_handler_t evt_handler;
    ble_conn_err_handler_t err_handler;
} ble_conn_env_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_conn_env_t s_conn_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool is_conn_param_ok(ble_gap_conn_param_t const * p_preferred_conn_param,
                             ble_gap_evt_conn_param_updated_t const * p_actual_conn_param,
                             uint16_t                      max_slave_latency_err,
                             uint16_t                      max_sup_timeout_err)
{
    uint32_t max_allowed_sl = p_preferred_conn_param->slave_latency + max_slave_latency_err;
    uint32_t min_allowed_sl = p_preferred_conn_param->slave_latency
                              - MIN(max_slave_latency_err, p_preferred_conn_param->slave_latency);
    uint32_t max_allowed_to = p_preferred_conn_param->sup_timeout + max_sup_timeout_err;
    uint32_t min_allowed_to = p_preferred_conn_param->sup_timeout
                              - MIN(max_sup_timeout_err, p_preferred_conn_param->sup_timeout);

    // Check if interval is within the acceptable range.
    // NOTE: Using max_conn_interval in the received event data because this contains
    //       the client's connection interval.
    if ((p_actual_conn_param->conn_interval < p_preferred_conn_param->interval_min) ||
        (p_actual_conn_param->conn_interval > p_preferred_conn_param->interval_max))
    {
        return false;
    }

    // Check if slave latency is within the acceptable deviation.
    if   ((p_actual_conn_param->slave_latency < min_allowed_sl)
       || (p_actual_conn_param->slave_latency > max_allowed_sl))
    {
        return false;
    }

    // Check if supervision timeout is within the acceptable deviation.
    if ((p_actual_conn_param->sup_timeout < min_allowed_to) || (p_actual_conn_param->sup_timeout > max_allowed_to))
    {
        return false;
    }

    return true;
}

void ble_connect_err_on_ble_capture(uint8_t err_code)
{
    if (s_conn_env.err_handler && err_code)
    {
        s_conn_env.err_handler(err_code);
    }
}

static bool send_update_request(uint8_t conn_idx, ble_gap_conn_param_t * p_new_conn_param)
{
    sdk_err_t error_code;
    
    ble_gap_conn_update_param_t conn_update_param;
    memcpy(&conn_update_param, p_new_conn_param, sizeof(ble_gap_conn_param_t));
    conn_update_param.ce_len = 0;

    error_code = ble_gap_conn_param_update(conn_idx, &conn_update_param);
    if ((error_code != SDK_SUCCESS) && (error_code != SDK_ERR_BUSY)) // SDK_ERR_BUSY means another conn_param_update request is pending. 
    {
        ble_connect_err_on_ble_capture(error_code);
    }

    return (error_code == SDK_SUCCESS);
}

static void update_timeout_handler(void *p_context)
{
    uint32_t conn_idx = (uint32_t)p_context;

    if (BLE_CONN_STATE_CONNECTED == s_conn_env.link_info[conn_idx].conn_state)
    {
        if (s_conn_env.link_info[conn_idx].param_ok)
        {
            return;
        }
        // Check if we have reached the maximum number of attempts
        if (s_conn_env.link_info[conn_idx].update_count < s_conn_env.max_conn_param_update_count)
        {
            bool update_sent = send_update_request(conn_idx, &s_conn_env.link_info[conn_idx].pref_conn_param);
            if (update_sent)
            {
                s_conn_env.link_info[conn_idx].update_count++;
            }
        }
        else
        {
            s_conn_env.link_info[conn_idx].update_count = 0;

            // Negotiation failed, disconnect automatically if this has been configured
            if (s_conn_env.disconnect_on_fail)
            {
                sdk_err_t error_code;

                error_code = ble_gap_disconnect_with_reason(conn_idx, BLE_GAP_HCI_CONN_INTERVAL_UNACCEPTABLE);
                
                ble_connect_err_on_ble_capture(error_code);
            }

            // Notify the application that the procedure has failed
            if (s_conn_env.evt_handler)
            {
                ble_conn_evt_t conn_evt;

                conn_evt.evt_type = BLE_CONN_EVT_PARAM_NEGO_FAILED;
                conn_evt.conn_idx = conn_idx;

                s_conn_env.evt_handler(&conn_evt);
            }
        }
    }
}

static void conn_param_negotiation(uint8_t conn_idx)
{
    // Start negotiation if the received connection parameters are not acceptable
    if (!s_conn_env.link_info[conn_idx].param_ok)
    {
        sdk_err_t error_code;
        uint32_t  timeout_ticks;

        if (0 == s_conn_env.link_info[conn_idx].update_count)
        {
            // First connection parameter update
            timeout_ticks = s_conn_env.first_conn_param_update_delay;
        }
        else
        {
            timeout_ticks = s_conn_env.next_conn_param_update_delay;
        }

        app_timer_stop(s_conn_env.link_info[conn_idx].timer_id);
        error_code = app_timer_start(s_conn_env.link_info[conn_idx].timer_id, timeout_ticks, (void *)(uint32_t)conn_idx);
        ble_connect_err_on_ble_capture(error_code);
    }
    else
    {
        s_conn_env.link_info[conn_idx].update_count = 0;

        // Notify the application that the procedure has succeeded
        if (s_conn_env.evt_handler != NULL)
        {
            ble_conn_evt_t evt;

            evt.evt_type = BLE_CONN_EVT_PARAM_NEGO_SUCCEEDED;
            evt.conn_idx = conn_idx;
            s_conn_env.evt_handler(&evt);
        }
    }
}

static void ble_conn_established(uint8_t conn_idx, const ble_gap_evt_connected_t *p_conn_param)
{
    if (conn_idx >= BLE_CONN_LINK_CNT_MAX)
    {
        return;
    }

    s_conn_env.link_info[conn_idx].conn_state = BLE_CONN_STATE_CONNECTED;
    s_conn_env.link_info[conn_idx].local_role = p_conn_param->ll_role;
    s_conn_env.link_info[conn_idx].conn_param.conn_interval = p_conn_param->conn_interval;
    s_conn_env.link_info[conn_idx].conn_param.slave_latency = p_conn_param->slave_latency;
    s_conn_env.link_info[conn_idx].conn_param.sup_timeout   = p_conn_param->sup_timeout;
    s_conn_env.link_info[conn_idx].peer_addr.addr_type      = p_conn_param->peer_addr_type;
    memcpy(&s_conn_env.link_info[conn_idx].peer_addr.gap_addr, &p_conn_param->peer_addr, sizeof(ble_gap_addr_t));


    if (BLE_GAP_LL_ROLE_SLAVE == s_conn_env.link_info[conn_idx].local_role)
    {
        ble_gap_evt_conn_param_updated_t conn_updated_param;
        memcpy(&conn_updated_param, &p_conn_param->conn_interval, sizeof(conn_updated_param));
        
        s_conn_env.link_info[conn_idx].param_ok = is_conn_param_ok(&s_conn_env.link_info[conn_idx].pref_conn_param,
                                                                   &conn_updated_param,
                                                                   BLE_CONN_PARAM_MAX_SLAVE_LATENCY_DEVIATION,
                                                                   BLE_CONN_PARAM_MAX_SUPERVISION_TIMEOUT_DEVIATION);

        if (s_conn_env.conn_param_manage_enable)
        {
            conn_param_negotiation(conn_idx);
        }
        else if (!s_conn_env.link_info[conn_idx].param_ok && s_conn_env.disconnect_on_fail)
        {
            sdk_err_t error_code;

            error_code = ble_gap_disconnect_with_reason(conn_idx, BLE_GAP_HCI_CONN_INTERVAL_UNACCEPTABLE);
            ble_connect_err_on_ble_capture(error_code);
        }
    }
    if (s_conn_env.evt_handler)
    {
        ble_conn_evt_t conn_evt;

        conn_evt.evt_type = BLE_CONN_EVT_CONNECTED;
        conn_evt.conn_idx = conn_idx;
        conn_evt.param.p_link_info = &s_conn_env.link_info[conn_idx];

        s_conn_env.evt_handler(&conn_evt);
    }
}

static void ble_conn_terminated(uint8_t conn_idx, uint8_t reason)
{
    if (conn_idx >= BLE_CONN_LINK_CNT_MAX)
    {
        return;
    }

    app_timer_id_t       timer_id_temp = s_conn_env.link_info[conn_idx].timer_id;
    ble_gap_conn_param_t pref_conn_param_temp = s_conn_env.link_info[conn_idx].pref_conn_param;
    
    memset(&s_conn_env.link_info[conn_idx], 0, sizeof(ble_conn_link_info_t));
    s_conn_env.link_info[conn_idx].timer_id = timer_id_temp;
    s_conn_env.link_info[conn_idx].conn_state = BLE_CONN_STATE_DISCONNECTED;
    memcpy(&s_conn_env.link_info[conn_idx].pref_conn_param, &pref_conn_param_temp, sizeof(ble_gap_conn_param_t));

    app_timer_stop(s_conn_env.link_info[conn_idx].timer_id);

    if (conn_idx == s_conn_env.act_conn_idx)
    {
        s_conn_env.act_conn_idx = BLE_GAP_INVALID_CONN_INDEX;
    }

    if (s_conn_env.evt_handler)
    {
        ble_conn_evt_t conn_evt;

        conn_evt.evt_type = BLE_CONN_EVT_DISCONNECTED;
        conn_evt.conn_idx = conn_idx;
        conn_evt.param.disconn_reason = reason;

        s_conn_env.evt_handler(&conn_evt);
    }
}

static void ble_conn_param_updated(uint8_t conn_idx, const ble_gap_evt_conn_param_updated_t *p_conn_param_updated_info)
{
    if (conn_idx >= BLE_CONN_LINK_CNT_MAX)
    {
        return;
    }

    memcpy(&s_conn_env.link_info[conn_idx].conn_param, p_conn_param_updated_info, sizeof(ble_conn_param_t));

    if (BLE_GAP_LL_ROLE_SLAVE == s_conn_env.link_info[conn_idx].local_role)
    {
        s_conn_env.link_info[conn_idx].param_ok = is_conn_param_ok(&s_conn_env.link_info[conn_idx].pref_conn_param,
                                                                    p_conn_param_updated_info,
                                                                    BLE_CONN_PARAM_MAX_SLAVE_LATENCY_DEVIATION,
                                                                    BLE_CONN_PARAM_MAX_SUPERVISION_TIMEOUT_DEVIATION);
        if (s_conn_env.conn_param_manage_enable)
        {
            conn_param_negotiation(conn_idx);
        }
        else if (!s_conn_env.link_info[conn_idx].param_ok && s_conn_env.disconnect_on_fail)
        {
            sdk_err_t error_code;

            error_code = ble_gap_disconnect_with_reason(conn_idx, BLE_GAP_HCI_CONN_INTERVAL_UNACCEPTABLE);
            ble_connect_err_on_ble_capture(error_code);
        }
    }

    // Interval = 0x4000 means thats connect param update request is rejected by master.
    if (s_conn_env.evt_handler && p_conn_param_updated_info->conn_interval != 0x4000)
    {
        ble_conn_evt_t conn_evt;

        conn_evt.evt_type = BLE_CONN_EVT_PATAM_UPDATED;
        conn_evt.conn_idx = conn_idx;
        conn_evt.param.p_update_param = (ble_conn_param_t *)p_conn_param_updated_info;

        s_conn_env.evt_handler(&conn_evt);
    }
}

static void ble_conn_link_encrypted(uint8_t conn_idx, uint8_t enc_ind)
{
    if (conn_idx >= BLE_CONN_LINK_CNT_MAX)
    {
        return;
    }

    s_conn_env.link_info[conn_idx].is_encrypt = true;
}

static sdk_err_t conn_init_param_check(ble_conn_init_t *p_conn_init)
{
    if (NULL == p_conn_init)
    {
        return SDK_ERR_POINTER_NULL;
    }
    if (NULL != p_conn_init->p_conn_param)
    {
        ble_gap_conn_param_t *p_conn_param = p_conn_init->p_conn_param;
        if (p_conn_param->interval_max > BLE_CONN_INTERVAL_MAX || p_conn_param->interval_min < BLE_CONN_INTERVAL_MIN ||
            p_conn_param->interval_max < p_conn_param->interval_min || p_conn_param->slave_latency > BLE_CONN_LATENCY_MAX ||
            p_conn_param->sup_timeout > BLE_CONN_TIMEOUT_MAX || p_conn_param->sup_timeout < BLE_CONN_TIMEOUT_MIN)
        {
            return SDK_ERR_INVALID_PARAM;
        }
    }
    if (p_conn_init->first_conn_param_update_delay > BLE_TIMER_TIMEOUT_MAX ||
        p_conn_init->next_conn_param_update_delay > BLE_TIMER_TIMEOUT_MAX)
    {
        return SDK_ERR_INVALID_PARAM;
    }
    return SDK_SUCCESS;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
sdk_err_t ble_connect_init(ble_conn_init_t *p_conn_init)
{
    sdk_err_t error_code;

    error_code = conn_init_param_check(p_conn_init);
    RET_VERIFY_SUCCESS(error_code);

    memset(&s_conn_env, 0, sizeof(s_conn_env));

    s_conn_env.act_conn_idx             = BLE_GAP_INVALID_CONN_INDEX;
    s_conn_env.conn_param_manage_enable = p_conn_init->conn_param_manage_enable;
    if (s_conn_env.conn_param_manage_enable)
    {
        if (p_conn_init->p_conn_param != NULL)
        {
            // Set the connection param in stack.
            error_code = ble_gap_ppcp_set(p_conn_init->p_conn_param);
            RET_VERIFY_SUCCESS(error_code);
        }
        else
        {
            // Get the (default) connection param from stack.
            error_code = ble_gap_ppcp_get(p_conn_init->p_conn_param);
            RET_VERIFY_SUCCESS(error_code);
        }
        s_conn_env.first_conn_param_update_delay = p_conn_init->first_conn_param_update_delay;
        s_conn_env.next_conn_param_update_delay  = p_conn_init->next_conn_param_update_delay;
        s_conn_env.max_conn_param_update_count   = p_conn_init->max_conn_param_update_count;

        for (uint32_t i = 0; i < BLE_CONN_LINK_CNT_MAX; i++)
        {
            memcpy(&s_conn_env.link_info[i].pref_conn_param, p_conn_init->p_conn_param, sizeof(ble_gap_conn_param_t));

            error_code = app_timer_create(&s_conn_env.link_info[i].timer_id, ATIMER_ONE_SHOT,
                                           update_timeout_handler);
            RET_VERIFY_SUCCESS(error_code);
        }
    }

    s_conn_env.disconnect_on_fail = p_conn_init->disconnect_on_fail;
    s_conn_env.evt_handler = p_conn_init->evt_handler;
    s_conn_env.err_handler = p_conn_init->err_handler;

    return SDK_SUCCESS;
}

uint8_t ble_connect_established_cnt_get(void)
{
    uint8_t count = 0;

    for (uint8_t i = 0; i < BLE_CONN_LINK_CNT_MAX; i++)
    {
        if (BLE_CONN_STATE_CONNECTED == s_conn_env.link_info[i].conn_state)
        {
            count++;
        }
    }

    return count;
}

bool ble_connect_is_connected(uint8_t conn_idx)
{
    if (conn_idx >= BLE_CONN_LINK_CNT_MAX)
    {
        return false;
    }

    if (BLE_CONN_STATE_CONNECTED == s_conn_env.link_info[conn_idx].conn_state)
    {
        return true;
    }

    return false;
}

sdk_err_t ble_connect_link_info_get(uint8_t conn_idx, ble_conn_link_info_t *p_link_info)
{
    if (conn_idx >= BLE_CONN_LINK_CNT_MAX || NULL == p_link_info)
    {
        return SDK_ERR_INVALID_PARAM;
    }

    memcpy(p_link_info, &s_conn_env.link_info[conn_idx], sizeof(ble_conn_link_info_t));

    return SDK_SUCCESS;
}

sdk_err_t ble_connect_active_link_set(uint8_t conn_idx)
{
    if (conn_idx >= BLE_CONN_LINK_CNT_MAX || BLE_CONN_STATE_DISCONNECTED == s_conn_env.link_info[conn_idx].conn_state)
    {
        return SDK_ERR_INVALID_CONN_IDX;
    }

    s_conn_env.act_conn_idx = conn_idx;

    return SDK_SUCCESS;
}

uint8_t ble_connect_active_link_get(void)
{
    return s_conn_env.act_conn_idx;
}

sdk_err_t ble_connect_all_to_do(ble_conn_all_link_exec_handler_t exec_handler)
{
    if (NULL == exec_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    for (uint8_t i = 0; i < BLE_CONN_LINK_CNT_MAX; i++)
    {
        if (BLE_CONN_STATE_CONNECTED ==  s_conn_env.link_info[i].conn_state)
        {
            exec_handler(i);
        }
    }

    return SDK_SUCCESS;
}

sdk_err_t ble_connect_next_link_idx_get(uint8_t cur_conn_idx, uint8_t *next_conn_idx)
{
    if (cur_conn_idx >= BLE_CONN_LINK_CNT_MAX || NULL == next_conn_idx ||
        BLE_CONN_STATE_DISCONNECTED == s_conn_env.link_info[cur_conn_idx].conn_state)
    {
        *next_conn_idx = BLE_GAP_INVALID_CONN_INDEX;
        return SDK_ERR_INVALID_CONN_IDX;
    }

    for (uint8_t i = cur_conn_idx + 1; ; i++)
    {
        if (i == BLE_CONN_LINK_CNT_MAX)
        {
            i = 0;
        }

        if (BLE_CONN_STATE_CONNECTED ==  s_conn_env.link_info[i].conn_state)
        {
            *next_conn_idx =  i;
            return SDK_SUCCESS;
        }
    }
}

sdk_err_t ble_connect_param_update_stop(void)
{
    for (uint32_t i = 0; i < BLE_CONN_LINK_CNT_MAX; i++)
    {
        app_timer_stop(s_conn_env.link_info[i].timer_id);
    }
    return SDK_SUCCESS;
}

sdk_err_t ble_connect_param_change(uint8_t conn_idx, ble_gap_conn_param_t *p_new_conn_param)
{
    sdk_err_t error_code = SDK_ERR_INVALID_CONN_IDX;

    if (conn_idx >= BLE_CONN_LINK_CNT_MAX)
    {
        return SDK_ERR_INVALID_CONN_IDX;
    }

    if (p_new_conn_param == NULL)
    {
        p_new_conn_param = &s_conn_env.link_info[conn_idx].pref_conn_param;
    }

    if (BLE_CONN_STATE_CONNECTED == s_conn_env.link_info[conn_idx].conn_state)
    {
        ble_gap_conn_update_param_t conn_update_param;
        memcpy(&conn_update_param, p_new_conn_param, sizeof(ble_gap_conn_param_t));
        conn_update_param.ce_len = 0;
        // Send request to master.
        error_code = ble_gap_conn_param_update(conn_idx, &conn_update_param);
        if (error_code == SDK_SUCCESS)
        {
            s_conn_env.link_info[conn_idx].param_ok             = false;
            s_conn_env.link_info[conn_idx].update_count          = 1;
            s_conn_env.link_info[conn_idx].pref_conn_param = *p_new_conn_param;
        }
    }

    return error_code;
}

void ble_connect_evt_on_ble_capture(const ble_evt_t *p_evt)
{
    switch (p_evt->evt_id)
    {
        case BLE_GAPC_EVT_CONNECTED:
            ble_conn_established(p_evt->evt.gapc_evt.index, &(p_evt->evt.gapc_evt.params.connected));
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            ble_conn_terminated(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATED:
            ble_conn_param_updated(p_evt->evt.gapc_evt.index, &(p_evt->evt.gapc_evt.params.conn_param_updated));
            break;

        case BLE_SEC_EVT_LINK_ENCRYPTED:
            ble_conn_link_encrypted(p_evt->evt.gapc_evt.index, p_evt->evt_status);
            break;
    }
}

#endif



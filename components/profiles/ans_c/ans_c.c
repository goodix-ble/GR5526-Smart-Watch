/**
 ****************************************************************************************
 *
 * @file ans_c.c
 *
 * @brief Alert Notification Service Client implementation.
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
 ****************************************************************************************
 */
#include "ans_c.h"
#include "utility.h"
#include <string.h>

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
/**@brief Alert Notification Service Client environment variable. */
struct ans_c_env_t
{
    ans_c_handles_t      handles;           /**< Handles of ANS characteristics which will be got for peer. */
    ans_c_evt_handler_t  evt_handler;       /**< Handler of ANS Client event handler. */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct ans_c_env_t s_ans_c_env;    /**< Alert Notification Service Client environment variable. */
static uint8_t            s_target_uuid[2] = {LO_U16(BLE_ATT_SVC_ALERT_NTF), HI_U16(BLE_ATT_SVC_ALERT_NTF)};
static ble_uuid_t         s_ans_service_uuid =
{
    .uuid_len = 2,
    .uuid     = s_target_uuid,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Excute Alert Notification Service Client event handler.
 *
 * @param[in] p_evt: Pointer to Alert Notification Service Client event structure.
 *****************************************************************************************
 */
static void ans_c_evt_handler_excute(ans_c_evt_t *p_evt)
{
    if (NULL != s_ans_c_env.evt_handler && ANS_C_EVT_INVALID != p_evt->evt_type)
    {
        s_ans_c_env.evt_handler(p_evt);
    }
}

/**
 *****************************************************************************************
 * @brief Decode for a New Alert.
 *
 * @param[in]  p_data:     Pointer to data to be decoded.
 * @param[out] p_cur_time: Pointer to New Alert.
 *****************************************************************************************
 */
static void ans_c_new_alert_decode(const uint8_t *p_data, uint16_t length, ans_c_new_alert_t *p_new_alert)
{
    if (2 > length)
    {
        return;
    }

    memset(p_new_alert, 0, sizeof(ans_c_new_alert_t));
    p_new_alert->cat_id    = (ans_c_alert_cat_id_t)p_data[0];
    p_new_alert->alert_num = p_data[1];
    p_new_alert->length    = length - 2;

    if (2 < length)
    {
        memcpy(p_new_alert->str_info, & p_data[2], length - 2);
    }
}

/**
 *****************************************************************************************
 * @brief Decode for a Unread Alert.
 *
 * @param[in]  p_data:     Pointer to data to be decoded.
 * @param[out] p_cur_time: Pointer to Unread Alert.
 *****************************************************************************************
 */
static void ans_c_unread_decode(const uint8_t *p_data, uint16_t length, ans_c_unread_alert_t *p_unread_alert)
{
    if (2 != length)
    {
        return;
    }

    p_unread_alert->cat_id      = (ans_c_alert_cat_id_t)p_data[0];
    p_unread_alert->unread_num  = p_data[1];
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving read response.
 *
 * @param[in] conn_idx:   The connection index.
 * @param[in] status:     The status of GATTC operation.
 * @param[in] p_read_rsp: The information of read response.
 *****************************************************************************************
 */
static void ans_c_att_read_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_read_t *p_read_rsp)
{
    ans_c_evt_t ans_c_evt;
    ans_c_evt.conn_idx = conn_idx;
    ans_c_evt.evt_type = ANS_C_EVT_INVALID;

    if (BLE_SUCCESS != status)
    {
        return;
    }

    if (p_read_rsp->value[0].handle == s_ans_c_env.handles.ans_sup_new_alert_cat_handle)
    {
        ans_c_evt.evt_type                    = ANS_C_EVT_SUP_NEW_ALERT_CAT_RECEIV;
        ans_c_evt.value.sup_new_alert_cat_ids = BUILD_U16(p_read_rsp->value[0].p_value[0], p_read_rsp->value[0].p_value[1]);
    }
    else if (p_read_rsp->value[0].handle == s_ans_c_env.handles.ans_sup_unread_alert_cat_handle)
    {
        ans_c_evt.evt_type                       = ANS_C_EVT_SUP_UNREAD_ALERT_CAT_REC;
        ans_c_evt.value.sup_unread_alert_cat_ids = BUILD_U16(p_read_rsp->value[0].p_value[0], p_read_rsp->value[0].p_value[1]);
    }

    ans_c_evt_handler_excute(&ans_c_evt);
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving read response.
 *
 * @param[in] conn_idx:   The connection index.
 * @param[in] status:     The status of GATTC operation.
 * @param[in] handle:     The handle of attribute.
 *****************************************************************************************
 */
static void ans_c_att_write_evt_handler(uint8_t conn_idx, uint8_t status, uint16_t handle)
{
    ans_c_evt_t ans_c_evt;

    ans_c_evt.conn_idx  = conn_idx;
    ans_c_evt.evt_type  = ANS_C_EVT_INVALID;

    if (handle == s_ans_c_env.handles.ans_new_alert_cccd_handle)
    {
        ans_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              ANS_C_EVT_NEW_ALERT_NTF_SET_SUCCESS :
                              ANS_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_ans_c_env.handles.ans_unread_alert_cccd_handle)
    {
        ans_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              ANS_C_EVT_UNREAD_ALERT_STA_NTF_SET_SUCCESS :
                              ANS_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_ans_c_env.handles.ans_ctrl_pt_handle)
    {
        ans_c_evt.evt_type = (BLE_SUCCESS == status) ?
                             ANS_C_EVT_CTRL_POINT_SET_SUCCESS : \
                             ANS_C_EVT_WRITE_OP_ERR;
    }

    ans_c_evt_handler_excute(&ans_c_evt);
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving notification or indication.
 *
 * @param[in] conn_idx:  The connection index.
 * @param[in] status:    The status of GATTC operation.
 * @param[in] p_ntf_ind: The information of notification or indication.
 *****************************************************************************************
 */
static void ans_c_att_ntf_ind_evt_handler(uint8_t conn_idx, const ble_gattc_evt_ntf_ind_t *p_ntf_ind)
{
    ans_c_evt_t ans_c_evt;

    ans_c_evt.conn_idx = conn_idx;
    ans_c_evt.evt_type = ANS_C_EVT_INVALID;

    if (p_ntf_ind->handle == s_ans_c_env.handles.ans_new_alert_handle)
    {
        ans_c_evt.evt_type = ANS_C_EVT_NEW_ALERT_RECEIVE;
        ans_c_new_alert_decode(p_ntf_ind->p_value, p_ntf_ind->length, &ans_c_evt.value.new_alert);
    }
    else if (p_ntf_ind->handle == s_ans_c_env.handles.ans_unread_alert_handle)
    {
        ans_c_evt.evt_type = ANS_C_EVT_UNREAD_ALERT_RECEIVE;
        ans_c_unread_decode(p_ntf_ind->p_value, p_ntf_ind->length, &ans_c_evt.value.unread_alert);
    }

    ans_c_evt_handler_excute(&ans_c_evt);
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving browse service indication.
 *
 * @param[in] conn_idx:      The connection index.
 * @param[in] status:        The status of GATTC operation.
 * @param[in] p_browse_srvc: The information of service browse.
 *****************************************************************************************
 */
static void ans_c_srvc_browse_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_browse_srvc_t *p_browse_srvc)
{
    ans_c_evt_t  ans_c_evt;
    uint16_t     uuid_disc;
    uint16_t     handle_disc;

    ans_c_evt.conn_idx = conn_idx;
    ans_c_evt.evt_type = ANS_C_EVT_DISCOVERY_FAIL;

    if(BLE_GATT_ERR_BROWSE_NO_ANY_MORE == status)
    {
        return;
    }

    if (BLE_SUCCESS == status)
    {
        uuid_disc = p_browse_srvc->uuid[0] | p_browse_srvc->uuid[1] << 8;

        if (BLE_ATT_SVC_ALERT_NTF == uuid_disc)
        {
            s_ans_c_env.handles.ans_srvc_start_handle = p_browse_srvc->start_hdl;
            s_ans_c_env.handles.ans_srvc_end_handle   = p_browse_srvc->end_hdl;

            for (uint32_t i = 0; i < p_browse_srvc->end_hdl - p_browse_srvc->start_hdl; i++)
            {
                if (BLE_GATTC_BROWSE_ATTR_VAL == p_browse_srvc->info[i].attr_type)
                {
                    uuid_disc   = p_browse_srvc->info[i].attr.uuid[0] | p_browse_srvc->info[i].attr.uuid[1] << 8;
                    handle_disc = p_browse_srvc->start_hdl + i + 1;

                    if (BLE_ATT_CHAR_SUP_NEW_ALERT_CAT == uuid_disc)
                    {
                        s_ans_c_env.handles.ans_sup_new_alert_cat_handle = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_NEW_ALERT == uuid_disc)
                    {
                        s_ans_c_env.handles.ans_new_alert_handle      = handle_disc;
                        s_ans_c_env.handles.ans_new_alert_cccd_handle = handle_disc + 1;
                    }
                    else if (BLE_ATT_CHAR_SUP_UNREAD_ALERT_CAT == uuid_disc)
                    {
                        s_ans_c_env.handles.ans_sup_unread_alert_cat_handle = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_UNREAD_ALERT_STATUS == uuid_disc)
                    {
                        s_ans_c_env.handles.ans_unread_alert_handle      = handle_disc;
                        s_ans_c_env.handles.ans_unread_alert_cccd_handle = handle_disc + 1;
                    }
                    else if (BLE_ATT_CHAR_ALERT_NTF_CTNL_PT == uuid_disc)
                    {
                        s_ans_c_env.handles.ans_ctrl_pt_handle = handle_disc;
                    }
                }
                else if (p_browse_srvc->info[i].attr_type == BLE_GATTC_BROWSE_NONE)
                {
                    break;
                }
            }

            ans_c_evt.evt_type = ANS_C_EVT_DISCOVERY_COMPLETE;
        }
    }

    ans_c_evt_handler_excute(&ans_c_evt);
}

static void ans_c_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTC_EVT_SRVC_BROWSE:
            ans_c_srvc_browse_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.srvc_browse);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            ans_c_att_read_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.read_rsp);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            ans_c_att_write_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, p_evt->evt.gattc_evt.params.write_rsp.handle);
            break;

        case BLE_GATTC_EVT_NTF_IND:
            ans_c_att_ntf_ind_evt_handler(p_evt->evt.gattc_evt.index, &p_evt->evt.gattc_evt.params.ntf_ind);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t ans_client_init(ans_c_evt_handler_t evt_handler)
{
    if (NULL == evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_ans_c_env, 0, sizeof(s_ans_c_env));
    s_ans_c_env.evt_handler = evt_handler;

    return ble_gattc_prf_add(&s_ans_service_uuid, ans_c_ble_evt_handler);
}

sdk_err_t ans_c_disc_srvc_start(uint8_t conn_idx)
{
    return ble_gattc_services_browse(conn_idx, &s_ans_service_uuid);
}

sdk_err_t ans_c_new_alert_notify_set(uint8_t conn_idx, bool is_enable)
{
    uint16_t ntf_value = is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_ans_c_env.handles.ans_new_alert_cccd_handle)
    {
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_ans_c_env.handles.ans_new_alert_cccd_handle, 0, 2, (uint8_t *)&ntf_value);
}

sdk_err_t ans_c_unread_alert_notify_set(uint8_t conn_idx, bool is_enable)
{
    uint16_t ntf_value = is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_ans_c_env.handles.ans_unread_alert_cccd_handle)
    {
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_ans_c_env.handles.ans_unread_alert_cccd_handle, 0, 2, (uint8_t *)&ntf_value);
}

sdk_err_t ans_c_sup_new_alert_cat_read(uint8_t conn_idx)
{
    if (BLE_ATT_INVALID_HDL == s_ans_c_env.handles.ans_sup_new_alert_cat_handle)
    {
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    return ble_gattc_read(conn_idx, s_ans_c_env.handles.ans_sup_new_alert_cat_handle, 0);
}

sdk_err_t ans_c_sup_unread_alert_cat_read(uint8_t conn_idx)
{
    if (BLE_ATT_INVALID_HDL == s_ans_c_env.handles.ans_sup_unread_alert_cat_handle)
    {
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    return ble_gattc_read(conn_idx, s_ans_c_env.handles.ans_sup_unread_alert_cat_handle, 0);
}

sdk_err_t ans_c_ctrl_point_set(uint8_t conn_idx, ans_c_ctrl_pt_t *p_ctrl_pt)
{
    uint8_t ctrl_pt_val[ANS_C_ALERT_NTF_CTRL_PT_VAL_LEN];

    ctrl_pt_val[0] = p_ctrl_pt->cmd_id;
    ctrl_pt_val[1] = p_ctrl_pt->cat_id;

    if (BLE_ATT_INVALID_HDL == s_ans_c_env.handles.ans_ctrl_pt_handle)
    {
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write_no_resp(conn_idx, false, s_ans_c_env.handles.ans_ctrl_pt_handle, 2, ctrl_pt_val);
}





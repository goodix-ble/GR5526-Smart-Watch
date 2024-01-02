/**
 *****************************************************************************************
 *
 * @file pass_c.c
 *
 * @brief Phone Alert Status Service Client Implementation.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "pass_c.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include <string.h>

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
/**@brief Phone Alert Status Service Client environment variable. */
struct pass_c_env_t
{
    pass_c_handles_t      handles;           /**< Handles of PASS characteristics which will be got for peer. */
    pass_c_evt_handler_t  evt_handler;       /**< Handler of PASS Client event  */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct pass_c_env_t s_pass_c_env;    /**< Phone Alert Status Service Client environment variable. */
static uint8_t             s_target_uuid[2] = {LO_U16(BLE_ATT_SVC_PHONE_ALERT_STATUS), HI_U16(BLE_ATT_SVC_PHONE_ALERT_STATUS)};
static ble_uuid_t s_pass_service_uuid =
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
 * @brief Excute Phone Alert Status Service Client event handler.
 *
 * @param[in] p_evt: Pointer to Alert Service Client event structure.
 *****************************************************************************************
 */
static void pass_c_evt_handler_excute(pass_c_evt_t *p_evt)
{
    if (NULL != s_pass_c_env.evt_handler && PASS_C_EVT_INVALID != p_evt->evt_type)
    {
        s_pass_c_env.evt_handler(p_evt);
    }
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
static void pass_c_att_read_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_read_t *p_read_rsp)
{
    pass_c_evt_t pass_c_evt;

    pass_c_evt.conn_idx = conn_idx;
    pass_c_evt.evt_type = PASS_C_EVT_INVALID;

    if (BLE_SUCCESS != status)
    {
        return;
    }

    if (p_read_rsp->value[0].handle == s_pass_c_env.handles.pass_alert_status_handle)
    {
        pass_c_evt.evt_type           = PASS_C_EVT_ALERT_STATUS_RECEIVE;
        pass_c_evt.value.alert_status = p_read_rsp->value[0].p_value[0];
    }
    else if (p_read_rsp->value[0].handle == s_pass_c_env.handles.pass_ringer_set_handle)
    {
        pass_c_evt.evt_type         = PASS_C_EVT_RINGER_SET_RECEIVE;
        pass_c_evt.value.ringer_set = p_read_rsp->value[0].p_value[0];
    }

    pass_c_evt_handler_excute(&pass_c_evt);
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
static void pass_c_att_write_evt_handler(uint8_t conn_idx, uint8_t status, uint16_t handle)
{
    pass_c_evt_t pass_c_evt;

    pass_c_evt.conn_idx  = conn_idx;
    pass_c_evt.evt_type  = PASS_C_EVT_INVALID;

    if (handle == s_pass_c_env.handles.pass_alert_status_cccd_handle)
    {
        pass_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                               PASS_C_EVT_ALERT_STATUS_NTF_SET_SUCCESS :
                               PASS_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_pass_c_env.handles.pass_ringer_set_cccd_handle)
    {
        pass_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                               PASS_C_EVT_RINGER_SET_NTF_SET_SUCCESS :
                               PASS_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_pass_c_env.handles.pass_ringer_ctrl_pt_handle)
    {
        pass_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                               PASS_C_EVT_CTRL_POINT_SET_SUCCESS :
                               PASS_C_EVT_WRITE_OP_ERR;
    }

    pass_c_evt_handler_excute(&pass_c_evt);;
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
static void pass_c_att_ntf_ind_evt_handler(uint8_t conn_idx, const ble_gattc_evt_ntf_ind_t *p_ntf_ind)
{
    pass_c_evt_t pass_c_evt;

    pass_c_evt.conn_idx = conn_idx;
    pass_c_evt.evt_type = PASS_C_EVT_INVALID;

    if (p_ntf_ind->handle == s_pass_c_env.handles.pass_alert_status_handle)
    {
        pass_c_evt.evt_type           = PASS_C_EVT_ALERT_STATUS_RECEIVE;
        pass_c_evt.value.alert_status = p_ntf_ind->p_value[0];
    }
    else if (p_ntf_ind->handle == s_pass_c_env.handles.pass_ringer_set_handle)
    {
        pass_c_evt.evt_type         = PASS_C_EVT_RINGER_SET_RECEIVE;
        pass_c_evt.value.ringer_set = p_ntf_ind->p_value[0];
    }

    pass_c_evt_handler_excute(&pass_c_evt);
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
static void pass_c_srvc_browse_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_browse_srvc_t *p_browse_srvc)
{
    pass_c_evt_t  pass_c_evt;
    uint16_t      uuid_disc;
    uint16_t      handle_disc;

    pass_c_evt.conn_idx = conn_idx;
    pass_c_evt.evt_type = PASS_C_EVT_DISCOVERY_FAIL;
 
    if(BLE_GATT_ERR_BROWSE_NO_ANY_MORE == status)
    {
        return;
    }

    if (BLE_SUCCESS == status)
    {
        uuid_disc = p_browse_srvc->uuid[0] | p_browse_srvc->uuid[1] << 8;

        if (BLE_ATT_SVC_PHONE_ALERT_STATUS == uuid_disc)
        {
            s_pass_c_env.handles.pass_srvc_start_handle = p_browse_srvc->start_hdl;
            s_pass_c_env.handles.pass_srvc_end_handle   = p_browse_srvc->end_hdl;

            for (uint32_t i = 0; i < p_browse_srvc->end_hdl - p_browse_srvc->start_hdl; i++)
            {
                if (BLE_GATTC_BROWSE_ATTR_VAL == p_browse_srvc->info[i].attr_type)
                {
                    uuid_disc   = p_browse_srvc->info[i].attr.uuid[0] | p_browse_srvc->info[i].attr.uuid[1] << 8;
                    handle_disc = p_browse_srvc->start_hdl + i + 1;

                    if (BLE_ATT_CHAR_ALERT_STATUS == uuid_disc)
                    {
                        s_pass_c_env.handles.pass_alert_status_handle      = handle_disc;
                        s_pass_c_env.handles.pass_alert_status_cccd_handle = handle_disc + 1;
                    }
                    else if (BLE_ATT_CHAR_RINGER_CNTL_POINT == uuid_disc)
                    {
                        s_pass_c_env.handles.pass_ringer_ctrl_pt_handle = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_RINGER_SETTING == uuid_disc)
                    {
                        s_pass_c_env.handles.pass_ringer_set_handle      = handle_disc;
                        s_pass_c_env.handles.pass_ringer_set_cccd_handle = handle_disc + 1;
                    }
                }
                else if (p_browse_srvc->info[i].attr_type == BLE_GATTC_BROWSE_NONE)
                {
                    break;
                }
            }

            pass_c_evt.evt_type = PASS_C_EVT_DISCOVERY_COMPLETE;
        }
    }

    pass_c_evt_handler_excute(&pass_c_evt);
}

static void pass_c_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTC_EVT_SRVC_BROWSE:
            pass_c_srvc_browse_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.srvc_browse);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            pass_c_att_read_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.read_rsp);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            pass_c_att_write_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, p_evt->evt.gattc_evt.params.write_rsp.handle);
            break;

        case BLE_GATTC_EVT_NTF_IND:
            pass_c_att_ntf_ind_evt_handler(p_evt->evt.gattc_evt.index, &p_evt->evt.gattc_evt.params.ntf_ind);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t pass_client_init(pass_c_evt_handler_t evt_handler)
{
    if (NULL == evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_pass_c_env, 0, sizeof(s_pass_c_env));
    s_pass_c_env.evt_handler = evt_handler;

    return ble_gattc_prf_add(&s_pass_service_uuid, pass_c_ble_evt_handler);
}

sdk_err_t pass_c_disc_srvc_start(uint8_t conn_idx)
{
    return ble_gattc_services_browse(conn_idx, &s_pass_service_uuid);
}

sdk_err_t pass_c_alert_status_notify_set(uint8_t conn_idx, bool is_enable)
{
    uint16_t ntf_value = is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_pass_c_env.handles.pass_alert_status_cccd_handle)
    {
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_pass_c_env.handles.pass_alert_status_cccd_handle, 0, 2, (uint8_t *)&ntf_value);
}

sdk_err_t pass_c_ringer_set_notify_set(uint8_t conn_idx, bool is_enable)
{
    uint16_t ntf_value = is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_pass_c_env.handles.pass_ringer_set_cccd_handle)
    {
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_pass_c_env.handles.pass_ringer_set_cccd_handle, 0, 2, (uint8_t *)&ntf_value);
}

sdk_err_t pass_c_ctrl_point_set(uint8_t conn_idx, uint8_t ctrl_value)
{
    if (BLE_ATT_INVALID_HDL == s_pass_c_env.handles.pass_ringer_ctrl_pt_handle)
    {
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write_no_resp(conn_idx, false, s_pass_c_env.handles.pass_ringer_ctrl_pt_handle, PASS_C_RINGER_CTRL_PT_VAL_LEN, &ctrl_value);
}

sdk_err_t pass_c_alert_status_read(uint8_t conn_idx)
{
    if (BLE_ATT_INVALID_HDL == s_pass_c_env.handles.pass_alert_status_handle)
    {
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    return ble_gattc_read(conn_idx, s_pass_c_env.handles.pass_alert_status_handle, 0);
}

sdk_err_t pass_c_ringer_set_read(uint8_t conn_idx)
{
    if (BLE_ATT_INVALID_HDL == s_pass_c_env.handles.pass_ringer_set_handle)
    {
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    return ble_gattc_read(conn_idx, s_pass_c_env.handles.pass_ringer_set_handle, 0);
}


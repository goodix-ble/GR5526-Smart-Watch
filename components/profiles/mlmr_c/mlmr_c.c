/**
 *****************************************************************************************
 *
 * @file mlmr_c.c
 *
 * @brief Multi Link Multi Role Client Implementation.
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
#include "mlmr_c.h"
#include "user_app.h"
#include "app_log.h"
/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
/**@brief Multi Link Multi Role Service Client environment variable. */
struct mlmr_c_env_t
{
    mlmr_c_handles_t      handles;            /**< Handles of MLMR_C characteristics which will be got for peer. */
    mlmr_c_evt_handler_t  evt_handler;        /**< Handler of MLMR  client event  */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct mlmr_c_env_t s_mlmr_c_env;       /**< MLMR  Client environment variable. */
static uint8_t             s_mlmr_c_uuid[16]                = MLMR_C_SVC_UUID;
static uint8_t             s_mlmr_c_rx_char_uuid[16]        = MLMR_C_RX_CHAR_UUID;
static uint8_t             s_mlmr_c_tx_char_uuid[16]        = MLMR_C_TX_CHAR_UUID;
static uint8_t             s_mlmr_c_flow_ctrl_char_uuid[16] = MLMR_C_FLOW_CTRL_UUID;

static uint16_t            received_data_len[CFG_BOND_DEVS];
static uint16_t            send_data_len[CFG_BOND_DEVS];
static uint8_t             adv_header                    = 0xA0;
static uint8_t             rx_buffer[CFG_BOND_DEVS][516];
static ble_uuid_t          s_mlmr_c_service_uuid =
{
    .uuid_len = 16,
    .uuid     = s_mlmr_c_uuid,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Excute MLMR  Service Client event handler.
 *
 * @param[in] p_evt: Pointer to MLMR  Service Client event structure.
 *****************************************************************************************
 */
static void mlmr_c_evt_handler_excute(mlmr_c_evt_t *p_evt)
{
    if (NULL != s_mlmr_c_env.evt_handler && MLMR_C_EVT_INVALID != p_evt->evt_type)
    {
        s_mlmr_c_env.evt_handler(p_evt);
    }
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
static void mlmr_c_att_write_evt_handler(uint8_t conn_idx, uint8_t status, uint16_t handle)
{
    mlmr_c_evt_t mlmr_c_evt;

    mlmr_c_evt.conn_idx = conn_idx;
    mlmr_c_evt.evt_type = MLMR_C_EVT_INVALID;

    if (handle == s_mlmr_c_env.handles.mlmr_c_tx_cccd_handle)
    {
        mlmr_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              MLMR_C_EVT_TX_NTF_SET_SUCCESS : \
                              MLMR_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_mlmr_c_env.handles.mlmr_c_flow_ctrl_cccd_handle)
    {
        mlmr_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              MLMR_C_EVT_FLOW_CTRL_NTF_SET_SUCCESS : \
                              MLMR_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_mlmr_c_env.handles.mlmr_c_rx_handle)
    {
        mlmr_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              MLMR_C_EVT_TX_CPLT : \
                              MLMR_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_mlmr_c_env.handles.mlmr_c_flow_ctrl_handle)
    {
        mlmr_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              MLMR_C_EVT_RX_FLOW_UPDATE_CPLT : \
                              MLMR_C_EVT_WRITE_OP_ERR;
    }

    mlmr_c_evt_handler_excute(&mlmr_c_evt);
}

void combin_received_packet(uint8_t conn_idx, uint16_t length, uint8_t *p_received_data, uint8_t *p_combin_data)
{
    uint8_t *buffer = p_combin_data;

    if(send_data_len[conn_idx] > received_data_len[conn_idx])
    {
        buffer += received_data_len[conn_idx];
        memcpy(buffer, p_received_data,length);
        received_data_len[conn_idx] += length;
    }
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
static void mlmr_c_att_ntf_ind_evt_handler(uint8_t conn_idx, const ble_gattc_evt_ntf_ind_t *p_ntf_ind)
{
    mlmr_c_evt_t mlmr_c_evt;

    mlmr_c_evt.conn_idx = conn_idx;
    mlmr_c_evt.evt_type = MLMR_C_EVT_INVALID;

    if (p_ntf_ind->handle == s_mlmr_c_env.handles.mlmr_c_flow_ctrl_handle)
    {
        if (MLMR_C_FLOW_CTRL_STATE_ON == p_ntf_ind->p_value[0])
        {
            mlmr_c_evt.evt_type = MLMR_C_EVT_TX_FLOW_ON;
        }
        else if (MLMR_C_FLOW_CTRL_STATE_OFF == p_ntf_ind->p_value[0])
        {
            mlmr_c_evt.evt_type = MLMR_C_EVT_TX_FLOW_OFF;
        }
    }
    else if (p_ntf_ind->handle == s_mlmr_c_env.handles.mlmr_c_tx_handle)
    {
        mlmr_c_evt.evt_type = MLMR_C_EVT_PEER_DATA_RECEIVE;
        if(p_ntf_ind->p_value[0] == adv_header)
        {
            uint16_t data_length;
            memcpy(&data_length, &(((uint8_t *)p_ntf_ind->p_value)[1]), 2);
            received_data_len[conn_idx] = 0;
            memset(rx_buffer[conn_idx], 0, 516);

            if(data_length <= (MAX_MTU_DEFUALT - 3))
            {
                mlmr_c_evt.evt_type = MLMR_C_EVT_PEER_DATA_RECEIVE;
                mlmr_c_evt.p_data   = p_ntf_ind->p_value;
                mlmr_c_evt.length   = p_ntf_ind->length;
                mlmr_c_evt_handler_excute(&mlmr_c_evt);
            }
            else
            {
                send_data_len[conn_idx] = data_length;
                combin_received_packet(conn_idx, p_ntf_ind->length, p_ntf_ind->p_value, rx_buffer[conn_idx]);
                if(received_data_len[conn_idx] == send_data_len[conn_idx])
                {
                    mlmr_c_evt.p_data = rx_buffer[conn_idx];
                    mlmr_c_evt.length = send_data_len[conn_idx];
                    mlmr_c_evt_handler_excute(&mlmr_c_evt);
                    received_data_len[conn_idx] = 0;
                }
            }
        }
        else
        {
            combin_received_packet(conn_idx, p_ntf_ind->length, p_ntf_ind->p_value, rx_buffer[conn_idx]);
            if(received_data_len[conn_idx] == send_data_len[conn_idx])
            {
                mlmr_c_evt.p_data = rx_buffer[conn_idx];
                mlmr_c_evt.length = send_data_len[conn_idx];
                mlmr_c_evt_handler_excute(&mlmr_c_evt);
                received_data_len[conn_idx] = 0;
            }
        }
    }
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
static void mlmr_c_srvc_browse_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_browse_srvc_t *p_browse_srvc)
{
    mlmr_c_evt_t  mlmr_c_evt;
    uint16_t     handle_disc;

    mlmr_c_evt.conn_idx = conn_idx;
    mlmr_c_evt.evt_type = MLMR_C_EVT_DISCOVERY_FAIL;

    if(BLE_GATT_ERR_BROWSE_NO_ANY_MORE == status)
    {
        return;
    }

    if (BLE_SUCCESS == status)
    {
        if (16 == p_browse_srvc->uuid_len && 0 == memcmp(p_browse_srvc->uuid, s_mlmr_c_uuid, 16))
        {
            s_mlmr_c_env.handles.mlmr_c_srvc_start_handle = p_browse_srvc->start_hdl;
            s_mlmr_c_env.handles.mlmr_c_srvc_end_handle   = p_browse_srvc->end_hdl;

            for (uint32_t i = 0; i < (p_browse_srvc->end_hdl - p_browse_srvc->start_hdl); i++)
            {
                handle_disc = p_browse_srvc->start_hdl + i + 1;

                if (BLE_GATTC_BROWSE_ATTR_VAL == p_browse_srvc->info[i].attr_type)
                {
                    if (0 == memcmp(p_browse_srvc->info[i].attr.uuid, s_mlmr_c_rx_char_uuid, 16))
                    {
                        s_mlmr_c_env.handles.mlmr_c_rx_handle = handle_disc;
                    }
                    else if (0 == memcmp(p_browse_srvc->info[i].attr.uuid, s_mlmr_c_tx_char_uuid, 16))
                    {
                        s_mlmr_c_env.handles.mlmr_c_tx_handle      = handle_disc;
                        s_mlmr_c_env.handles.mlmr_c_tx_cccd_handle = handle_disc + 1;
                    }
                    else if (0 == memcmp(p_browse_srvc->info[i].attr.uuid, s_mlmr_c_flow_ctrl_char_uuid, 16))
                    {
                        s_mlmr_c_env.handles.mlmr_c_flow_ctrl_handle      = handle_disc;
                        s_mlmr_c_env.handles.mlmr_c_flow_ctrl_cccd_handle = handle_disc + 1;
                    }
                }
            
                if (p_browse_srvc->info[i].attr_type == BLE_GATTC_BROWSE_NONE)
                {
                    break;
                }
            }

            mlmr_c_evt.evt_type = MLMR_C_EVT_DISCOVERY_COMPLETE;
        }
    }

    mlmr_c_evt_handler_excute(&mlmr_c_evt);
}

static void mlmr_c_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTC_EVT_SRVC_BROWSE:
            mlmr_c_srvc_browse_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.srvc_browse);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            mlmr_c_att_write_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, p_evt->evt.gattc_evt.params.write_rsp.handle);
            break;

        case BLE_GATTC_EVT_NTF_IND:
            mlmr_c_att_ntf_ind_evt_handler(p_evt->evt.gattc_evt.index, &p_evt->evt.gattc_evt.params.ntf_ind);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t mlmr_client_init(mlmr_c_evt_handler_t evt_handler)
{
    if (NULL == evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_mlmr_c_env, 0, sizeof(s_mlmr_c_env));
    s_mlmr_c_env.evt_handler = evt_handler;

    return ble_gattc_prf_add(&s_mlmr_c_service_uuid, mlmr_c_ble_evt_handler);
}

sdk_err_t mlmr_c_disc_srvc_start(uint8_t conn_idx)
{
    return ble_gattc_services_browse(conn_idx, &s_mlmr_c_service_uuid);
}

sdk_err_t mlmr_c_tx_notify_set(uint8_t conn_idx, bool is_enable)
{
    uint16_t ntf_value = is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_mlmr_c_env.handles.mlmr_c_tx_cccd_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_mlmr_c_env.handles.mlmr_c_tx_cccd_handle, 0, 2, (uint8_t *)&ntf_value);
}

sdk_err_t mlmr_c_flow_ctrl_notify_set(uint8_t conn_idx, bool is_enable)
{
    uint16_t ntf_value = is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_mlmr_c_env.handles.mlmr_c_flow_ctrl_cccd_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_mlmr_c_env.handles.mlmr_c_flow_ctrl_cccd_handle, 0, 2, (uint8_t *)&ntf_value);
}

sdk_err_t mlmr_c_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    sdk_err_t             error_code;
    uint8_t               *buffer = p_data;

    if (BLE_ATT_INVALID_HDL == s_mlmr_c_env.handles.mlmr_c_rx_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    if (NULL == p_data)
    {
        return SDK_ERR_POINTER_NULL;
    }

    if(length > (MAX_MTU_DEFUALT - 3))
    {
        while(length > MAX_MTU_DEFUALT - 3)
        {
            error_code = ble_gattc_write_no_resp(conn_idx, false, s_mlmr_c_env.handles.mlmr_c_rx_handle, MAX_MTU_DEFUALT - 3, buffer);
            buffer += (MAX_MTU_DEFUALT - 3);
            length -= (MAX_MTU_DEFUALT - 3);
        }
        
        if(length != 0 && length < (MAX_MTU_DEFUALT - 3))
        {
            error_code = ble_gattc_write_no_resp(conn_idx, false, s_mlmr_c_env.handles.mlmr_c_rx_handle, length, buffer);
        }
    }
    else
    {
         error_code = ble_gattc_write_no_resp(conn_idx, false, s_mlmr_c_env.handles.mlmr_c_rx_handle, length, buffer);
    }

    return error_code;
}

sdk_err_t mlmr_c_rx_flow_ctrl_set(uint8_t conn_idx, mlmr_c_flow_ctrl_state_t flow_ctrl)
{
    if (BLE_ATT_INVALID_HDL == s_mlmr_c_env.handles.mlmr_c_flow_ctrl_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_mlmr_c_env.handles.mlmr_c_flow_ctrl_handle, 0, 1, &flow_ctrl);
}

/**
 *****************************************************************************************
 *
 * @file otas_c.c
 *
 * @brief OTA Client Implementation.
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
#include "otas_c.h"
#include <string.h>

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
/**@brief OTA Service Client environment variable. */
struct otas_c_env_t
{
    otas_c_handles_t        handles;                /**< Handles of OTA characteristics which will be got for peer. */
    otas_c_evt_handler_t    evt_handler;            /**< Handler of OTA client event  */
    uint8_t                 prf_id;                 /**< OTA Client profile id. */
    ble_gatts_create_db_t   otas_c_gatts_db;        /**< OTA Client attributs database. */
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct otas_c_env_t s_otas_c_env;
static uint8_t             s_otas_uuid[16]              = OTAS_SVC_UUID;
static uint8_t             s_otas_tx_char_uuid[16]      = OTAS_TX_CHAR_UUID;
static uint8_t             s_otas_rx_char_uuid[16]      = OTAS_RX_CHAR_UUID;
static uint8_t             s_otas_ctrl_char_uuid[16]    = OTAS_CTRL_CHAR_UUID;
static ble_uuid_t          s_otas_c_service_uuid =
{
    .uuid_len = 16,
    .uuid     = s_otas_uuid,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Execute OTA Service Client event handler.
 *
 * @param[in] p_evt: Pointer to OTA Service Client event structure.
 *****************************************************************************************
 */
static void otas_c_evt_handler_execute(otas_c_evt_t *p_evt)
{
    if (NULL != s_otas_c_env.evt_handler && OTAS_C_EVT_INVALID != p_evt->evt_type)
    {
        s_otas_c_env.evt_handler(p_evt);
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
static void otas_c_att_write_evt_handler(uint8_t conn_idx, uint8_t status, uint16_t handle)
{
    otas_c_evt_t otas_c_evt;

    otas_c_evt.conn_idx = conn_idx;
    otas_c_evt.evt_type = OTAS_C_EVT_INVALID;

    if (handle == s_otas_c_env.handles.otas_tx_cccd_handle)
    {
        otas_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              OTAS_C_EVT_TX_NTF_SET_SUCCESS : \
                              OTAS_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_otas_c_env.handles.otas_ctrl_handle)
    {
        otas_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              OTAS_C_EVT_CTRL_SUCCESS : \
                              OTAS_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_otas_c_env.handles.otas_rx_handle)
    {
        otas_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              OTAS_C_EVT_TX_CPLT : \
                              OTAS_C_EVT_WRITE_OP_ERR;
    }

    otas_c_evt_handler_execute(&otas_c_evt);
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
static void otas_c_att_ntf_ind_evt_handler(uint8_t conn_idx, const ble_gattc_evt_ntf_ind_t *p_ntf_ind)
{
    otas_c_evt_t otas_c_evt;

    otas_c_evt.conn_idx = conn_idx;
    otas_c_evt.evt_type = OTAS_C_EVT_INVALID;
    otas_c_evt.p_data   = p_ntf_ind->p_value;
    otas_c_evt.length   = p_ntf_ind->length;

    if (p_ntf_ind->handle == s_otas_c_env.handles.otas_tx_handle)
    {
        otas_c_evt.evt_type = OTAS_C_EVT_PEER_DATA_RECEIVE;
    }

    otas_c_evt_handler_execute(&otas_c_evt);
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
static void otas_c_srvc_browse_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_browse_srvc_t *p_browse_srvc)
{
    otas_c_evt_t  otas_c_evt;
    uint16_t     handle_disc;

    otas_c_evt.conn_idx = conn_idx;
    otas_c_evt.evt_type = OTAS_C_EVT_DISCOVERY_FAIL;

    if(BLE_GATT_ERR_BROWSE_NO_ANY_MORE == status)
    {
        return;
    }

    if (BLE_SUCCESS == status)
    {
        if (16 == p_browse_srvc->uuid_len && 0 == memcmp(p_browse_srvc->uuid, s_otas_uuid, 16))
        {
            s_otas_c_env.handles.otas_srvc_start_handle = p_browse_srvc->start_hdl;
            s_otas_c_env.handles.otas_srvc_end_handle   = p_browse_srvc->end_hdl;

            for (uint32_t i = 0; i < (p_browse_srvc->end_hdl - p_browse_srvc->start_hdl); i++)
            {
                handle_disc = p_browse_srvc->start_hdl + i + 1;

                if (p_browse_srvc->info[i].attr_type == BLE_GATTC_BROWSE_ATTR_VAL)
                {
                    if (0 == memcmp(p_browse_srvc->info[i].attr.uuid, s_otas_rx_char_uuid, 16))
                    {
                        s_otas_c_env.handles.otas_rx_handle = handle_disc;
                    }
                    else if (0 == (memcmp(p_browse_srvc->info[i].attr.uuid, s_otas_tx_char_uuid, 16)))
                    {
                        s_otas_c_env.handles.otas_tx_handle      = handle_disc;
                        s_otas_c_env.handles.otas_tx_cccd_handle = handle_disc + 1;
                    }  
                    else if (0 == memcmp(p_browse_srvc->info[i].attr.uuid, s_otas_ctrl_char_uuid, 16))
                    {
                        s_otas_c_env.handles.otas_ctrl_handle = handle_disc;
                    }
                }

                if (p_browse_srvc->info[i].attr_type == BLE_GATTC_BROWSE_NONE)
                {
                    break;
                }
            }

            otas_c_evt.evt_type = OTAS_C_EVT_DISCOVERY_COMPLETE;
        }
    }

    otas_c_evt_handler_execute(&otas_c_evt);
}

static void otas_c_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTC_EVT_SRVC_BROWSE:
            otas_c_srvc_browse_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.srvc_browse);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            otas_c_att_write_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, p_evt->evt.gattc_evt.params.write_rsp.handle);
            break;

        case BLE_GATTC_EVT_NTF_IND:
            otas_c_att_ntf_ind_evt_handler(p_evt->evt.gattc_evt.index, &p_evt->evt.gattc_evt.params.ntf_ind);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t otas_client_init(otas_c_evt_handler_t evt_handler)
{
    if (NULL == evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_otas_c_env, 0, sizeof(s_otas_c_env));
    s_otas_c_env.evt_handler = evt_handler;

    return ble_gattc_prf_add(&s_otas_c_service_uuid, otas_c_ble_evt_handler);
}

sdk_err_t otas_c_disc_srvc_start(uint8_t conn_idx)
{
    const ble_uuid_t otas_uuid =
    {
        .uuid_len = 16,
        .uuid     = s_otas_uuid,
    };

    return ble_gattc_services_browse(conn_idx, &otas_uuid);
}

sdk_err_t otas_c_tx_notify_set(uint8_t conn_idx, bool is_enable)
{
    uint16_t ntf_value = is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_otas_c_env.handles.otas_tx_cccd_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_otas_c_env.handles.otas_tx_cccd_handle, 0, 2, (uint8_t *)&ntf_value);
}

sdk_err_t otas_c_ctrl_data_send(uint8_t conn_idx, uint32_t data)
{
    if (BLE_ATT_INVALID_HDL == s_otas_c_env.handles.otas_ctrl_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write_no_resp(conn_idx, false, s_otas_c_env.handles.otas_ctrl_handle, sizeof(uint32_t), (uint8_t*)&data);
}

sdk_err_t otas_c_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    if (BLE_ATT_INVALID_HDL == s_otas_c_env.handles.otas_rx_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write_no_resp(conn_idx, false, s_otas_c_env.handles.otas_rx_handle, length, p_data);
}

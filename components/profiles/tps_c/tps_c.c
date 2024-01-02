/**
 *****************************************************************************************
 *
 * @file tps_c.c
 *
 * @brief Tx Power Service Client Implementation.
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
#include "tps_c.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include <string.h>

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
/**@brief Tx Power Service environment variable. */
struct tps_c_env_t
{
    tps_c_handles_t     handles;            /**< Handles of TPS characteristics which will be got for peer. */
    tps_c_evt_handler_t evt_handler;        /**< Handler of TPS Client event  */
    uint8_t             prf_id;             /**< TPS Client profile id. */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct tps_c_env_t s_tps_c_env;     /**< Tx Power Service Client environment variable. */
static uint8_t      s_target_uuid[2] = {LO_U16(BLE_ATT_SVC_TX_POWER), HI_U16(BLE_ATT_SVC_TX_POWER)};
static ble_uuid_t   s_tps_service_uuid =
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
 * @brief This event handler function will be called when receiving read response.
 *
 * @param[in] conn_idx:   The connection index.
 * @param[in] status:     The status of GATTC operation.
 * @param[in] p_read_rsp: The information of read response.
 *****************************************************************************************
 */
static void tps_c_att_read_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_read_t *p_read_rsp)
{
    tps_c_evt_t tps_c_evt;

    tps_c_evt.conn_idx = conn_idx;
    tps_c_evt.evt_type = TPS_C_EVT_INVALID;

    if (BLE_SUCCESS != status)
    {
        return;
    }

    if (p_read_rsp->value[0].handle == s_tps_c_env.handles.tps_tx_power_level_handle)
    {
        tps_c_evt.evt_type  = TPS_C_EVT_TX_POWER_LEVEL_RECEIVE;
        tps_c_evt.tx_power_level = p_read_rsp->value[0].p_value[0];

        if (s_tps_c_env.evt_handler)
        {
            s_tps_c_env.evt_handler(&tps_c_evt);
        }
    }
}

/**
 *****************************************************************************************
 * @brief This event handler function will be called when receiving browse service indication.
 *
 * @param[in] conn_idx:      The connection index.
 * @param[in] status:        The status of GATTC operation.
 * @param[in] p_browse_srvc: The information of service browse.
 *****************************************************************************************
 */
static void tps_c_srvc_browse_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_browse_srvc_t *p_browse_srvc)
{
    tps_c_evt_t  tps_c_evt;
    uint16_t     uuid_disc;
    uint16_t     handle_disc;

    tps_c_evt.conn_idx = conn_idx;
    tps_c_evt.evt_type = TPS_C_EVT_DISCOVERY_FAIL;

    if(BLE_GATT_ERR_BROWSE_NO_ANY_MORE == status)
    {
        return;
    }

    if (BLE_SUCCESS == status)
    {
        uuid_disc = p_browse_srvc->uuid[0] | p_browse_srvc->uuid[1] << 8;

        if (BLE_ATT_SVC_TX_POWER == uuid_disc)
        {
            s_tps_c_env.handles.tps_srvc_start_handle = p_browse_srvc->start_hdl;
            s_tps_c_env.handles.tps_srvc_end_handle   = p_browse_srvc->end_hdl;

            for (uint32_t i = 0; i < (p_browse_srvc->end_hdl - p_browse_srvc->start_hdl); i++)
            {
                uuid_disc   = p_browse_srvc->info[i].attr.uuid[0] | p_browse_srvc->info[i].attr.uuid[1] << 8;
                handle_disc = p_browse_srvc->start_hdl + i + 1;

                if (BLE_GATTC_BROWSE_ATTR_VAL == p_browse_srvc->info[i].attr_type)
                {
                    if (BLE_ATT_CHAR_TX_POWER_LEVEL == uuid_disc)
                    {
                        s_tps_c_env.handles.tps_tx_power_level_handle = handle_disc;
                    }
                }
                else if (BLE_GATTC_BROWSE_NONE == p_browse_srvc->info[i].attr_type)
                {
                    break;
                }
            }

            tps_c_evt.evt_type = TPS_C_EVT_DISCOVERY_COMPLETE;
        }
    }

    if (s_tps_c_env.evt_handler)
    {
        s_tps_c_env.evt_handler(&tps_c_evt);
    }
}

static void tps_c_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            tps_c_att_read_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.read_rsp);
            break;

        case BLE_GATTC_EVT_SRVC_BROWSE:
            tps_c_srvc_browse_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.srvc_browse);
            break;
    }
}
/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t tps_client_init(tps_c_evt_handler_t evt_handler)
{
    if (NULL == evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_tps_c_env, 0, sizeof(s_tps_c_env));
    s_tps_c_env.evt_handler = evt_handler;

    return ble_gattc_prf_add(&s_tps_service_uuid, tps_c_ble_evt_handler);
}

sdk_err_t tps_c_disc_srvc_start(uint8_t conn_idx)
{
    return ble_gattc_services_browse(conn_idx, &s_tps_service_uuid);
}


sdk_err_t tps_c_tx_power_level_read(uint8_t conn_idx)
{
    if (BLE_ATT_INVALID_HDL == s_tps_c_env.handles.tps_tx_power_level_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return  ble_gattc_read(conn_idx, s_tps_c_env.handles.tps_tx_power_level_handle, 0);
}

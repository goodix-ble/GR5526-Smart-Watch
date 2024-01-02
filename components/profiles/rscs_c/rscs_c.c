/**
 *****************************************************************************************
 *
 * @file rscs_c.c
 *
 * @brief Running Speed and Cadence Service Client Implementation.
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
#include "rscs_c.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include <string.h>

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
/**@brief Running Speed and Cadence Service Client environment variable. */
struct rscs_c_env_t
{
    rscs_c_handles_t     handles;            /**< Handles of RSCS characteristics which will be got for peer. */
    rscs_c_evt_handler_t evt_handler;        /**< Handler of RSCS Client event  */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct rscs_c_env_t s_rscs_c_env;    /**< Running Speed and Cadence Service Client environment variable. */
static uint8_t             s_target_uuid[2]   = {LO_U16(BLE_ATT_SVC_RUNNING_SPEED_CADENCE), HI_U16(BLE_ATT_SVC_RUNNING_SPEED_CADENCE)};
static ble_uuid_t          s_rscs_service_uuid =
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
 * @brief Excute  Running Speed and Cadence Service Client event handler.
 *
 * @param[in] p_evt: Pointer to  Running Speed and Cadence Service Client event structure.
 *****************************************************************************************
 */
static void rscs_c_evt_handler_excute(rscs_c_evt_t *p_evt)
{
    if (NULL != s_rscs_c_env.evt_handler && RSCS_C_EVT_INVALID != p_evt->evt_type)
    {
        s_rscs_c_env.evt_handler(p_evt);
    }
}

/**
 *****************************************************************************************
 * @brief Encode RSC measurement value.
 *
 * @param[in]  p_data:          Pointer to data encode.
 * @param[in]  length:          Length of data encode.
 * @param[out] p_rsc_meas_buff: Pointer to buffer for encode RSC measurement value.
 *****************************************************************************************
 */
static void rscs_c_meas_value_encode(uint8_t *p_data, uint16_t length, rscs_c_meas_val_t *p_rsc_meas_buff)
{
    uint8_t   flags = 0;
    uint8_t   index = 0;
    flags = p_data[index++];
    memset(p_rsc_meas_buff, 0, sizeof(rscs_c_meas_val_t));
    // Instantaneous speed field
    p_rsc_meas_buff->inst_speed = BUILD_U16(p_data[index], p_data[index + 1]);
    index += sizeof(uint16_t);
    // Instantaneous cadence field
    p_rsc_meas_buff->inst_cadence = p_data[index++];

    // Instantaneous stride length field
    if (flags & RSCS_C_MEAS_FLAG_INST_STRIDE_LEN_BIT)
    {
        p_rsc_meas_buff->inst_stride_length_present = true;
        p_rsc_meas_buff->inst_stride_length         = BUILD_U16(p_data[index], p_data[index + 1]);
        index += sizeof(uint16_t);
    }

    // Total distance field
    if (flags & RSCS_C_MEAS_FLAG_TOTAL_DISTANCE_BIT)
    {
        p_rsc_meas_buff->total_distance_present = true;
        p_rsc_meas_buff->total_distance         = BUILD_U32(p_data[index],
                p_data[index + 1],
                p_data[index + 2],
                p_data[index + 3]);
        index += sizeof(uint32_t);
    }

    // Running or Walking field
    if (flags & RSCS_C_MEAS_FLAG_RUNNING_OR_WALKING_BIT)
    {
        p_rsc_meas_buff->is_run_or_walk = true;
    }
    else
    {
        p_rsc_meas_buff->is_run_or_walk = false;
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
static void rscs_c_att_read_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_read_t *p_read_rsp)
{
    rscs_c_evt_t rscs_c_evt;

    rscs_c_evt.conn_idx = conn_idx;
    rscs_c_evt.evt_type = RSCS_C_EVT_INVALID;

    if (BLE_SUCCESS != status)
    {
        return;
    }

    if (p_read_rsp->value[0].handle == s_rscs_c_env.handles.rscs_rsc_feature_handle)
    {
        rscs_c_evt.evt_type          = RSCS_C_EVT_RSC_FEATURE_RECEIVE;
        rscs_c_evt.value.rsc_feature = BUILD_U16(p_read_rsp->value[0].p_value[0], p_read_rsp->value[0].p_value[1]);
    }
    else if (p_read_rsp->value[0].handle == s_rscs_c_env.handles.rscs_sensor_loc_handle)
    {
        rscs_c_evt.evt_type             = RSCS_C_EVT_SENSOR_LOC_RECEIVE;
        rscs_c_evt.value.rsc_sensor_loc = (rscs_c_sensor_loc_t)p_read_rsp->value[0].p_value[0];
    }

    rscs_c_evt_handler_excute(&rscs_c_evt);
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
static void rscs_c_att_write_evt_handler(uint8_t conn_idx, uint8_t status, uint16_t handle)
{
    rscs_c_evt_t rscs_c_evt;

    rscs_c_evt.conn_idx = conn_idx;
    rscs_c_evt.evt_type = RSCS_C_EVT_INVALID;

    if (handle == s_rscs_c_env.handles.rscs_rsc_meas_cccd_handle)
    {
        rscs_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                               RSCS_C_EVT_RSC_MEAS_NTF_SET_SUCCESS :
                               RSCS_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_rscs_c_env.handles.rscs_ctrl_pt_cccd_handle)
    {
        rscs_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                               RSCS_C_EVT_CTRL_PT_IND_SET_SUCCESS :
                               RSCS_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_rscs_c_env.handles.rscs_ctrl_pt_handle)
    {
        rscs_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                               RSCS_C_EVT_CTRL_PT_SET_SUCCESS :
                               RSCS_C_EVT_WRITE_OP_ERR;
    }

    rscs_c_evt_handler_excute(&rscs_c_evt);
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
static void rscs_c_att_ntf_ind_evt_handler(uint8_t conn_idx, const ble_gattc_evt_ntf_ind_t *p_ntf_ind)
{
    rscs_c_evt_t rscs_c_evt;

    rscs_c_evt.conn_idx = conn_idx;
    rscs_c_evt.evt_type = RSCS_C_EVT_INVALID;

    if (p_ntf_ind->handle == s_rscs_c_env.handles.rscs_rsc_meas_handle)
    {
        rscs_c_evt.evt_type = RSCS_C_EVT_RSC_MEAS_VAL_RECEIVE;
        rscs_c_meas_value_encode(p_ntf_ind->p_value, p_ntf_ind->length, &rscs_c_evt.value.rsc_meas_buff);
    }
    else if (p_ntf_ind->handle == s_rscs_c_env.handles.rscs_ctrl_pt_handle)
    {
        rscs_c_evt.evt_type = RSCS_C_EVT_CTRL_PT_RSP_RECEIVE;
        memcpy(rscs_c_evt.value.ctrl_pt_rsp, p_ntf_ind->p_value, p_ntf_ind->length);
    }

    rscs_c_evt_handler_excute(&rscs_c_evt);
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
static void rscs_c_srvc_browse_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_browse_srvc_t *p_browse_srvc)
{
    rscs_c_evt_t  rscs_c_evt;
    uint16_t      uuid_disc;
    uint16_t      handle_disc;

    rscs_c_evt.conn_idx = conn_idx;
    rscs_c_evt.evt_type = RSCS_C_EVT_DISCOVERY_FAIL;

    if(BLE_GATT_ERR_BROWSE_NO_ANY_MORE == status)
    {
        return;
    }

    if (BLE_SUCCESS == status)
    {
        uuid_disc = p_browse_srvc->uuid[0] | p_browse_srvc->uuid[1] << 8;

        if (BLE_ATT_SVC_RUNNING_SPEED_CADENCE == uuid_disc)
        {
            s_rscs_c_env.handles.rscs_srvc_start_handle = p_browse_srvc->start_hdl;
            s_rscs_c_env.handles.rscs_srvc_end_handle   = p_browse_srvc->end_hdl;

            for (uint32_t i = 0; i < (p_browse_srvc->end_hdl - p_browse_srvc->start_hdl); i++)
            {
                    uuid_disc   = p_browse_srvc->info[i].attr.uuid[0] | p_browse_srvc->info[i].attr.uuid[1] << 8;
                    handle_disc = p_browse_srvc->start_hdl + i + 1;

                    if (BLE_GATTC_BROWSE_ATTR_VAL == p_browse_srvc->info[i].attr_type)
                    {
                        if (BLE_ATT_CHAR_RSC_MEAS == uuid_disc)
                        {
                            s_rscs_c_env.handles.rscs_rsc_meas_handle      = handle_disc;
                            s_rscs_c_env.handles.rscs_rsc_meas_cccd_handle = handle_disc + 1;
                        }
                        else if (BLE_ATT_CHAR_RSC_FEAT == uuid_disc)
                        {
                            s_rscs_c_env.handles.rscs_rsc_feature_handle = handle_disc;
                        }
                        else if (BLE_ATT_CHAR_SENSOR_LOC == uuid_disc)
                        {
                            s_rscs_c_env.handles.rscs_sensor_loc_handle = handle_disc;
                        }
                        else if (BLE_ATT_CHAR_SC_CNTL_PT == uuid_disc)
                        {
                            s_rscs_c_env.handles.rscs_ctrl_pt_handle      = handle_disc;
                            s_rscs_c_env.handles.rscs_ctrl_pt_cccd_handle = handle_disc + 1;
                        }
                    }
                    else if (BLE_GATTC_BROWSE_NONE == p_browse_srvc->info[i].attr_type)
                    {
                        break;
                    }
            }

            rscs_c_evt.evt_type = RSCS_C_EVT_DISCOVERY_COMPLETE;
        }
    }

    rscs_c_evt_handler_excute(&rscs_c_evt);
}

static void rscs_c_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTC_EVT_SRVC_BROWSE:
            rscs_c_srvc_browse_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.srvc_browse);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            rscs_c_att_read_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.read_rsp);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            rscs_c_att_write_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, p_evt->evt.gattc_evt.params.write_rsp.handle);
            break;

        case BLE_GATTC_EVT_NTF_IND:
            rscs_c_att_ntf_ind_evt_handler(p_evt->evt.gattc_evt.index, &p_evt->evt.gattc_evt.params.ntf_ind);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t rscs_client_init(rscs_c_evt_handler_t evt_handler)
{
    if (NULL == evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_rscs_c_env, 0, sizeof(s_rscs_c_env)) ;
    s_rscs_c_env.evt_handler = evt_handler;

    return ble_gattc_prf_add(&s_rscs_service_uuid, rscs_c_ble_evt_handler);
}

sdk_err_t rscs_c_disc_srvc_start(uint8_t conn_idx)
{
    return ble_gattc_services_browse(conn_idx, &s_rscs_service_uuid);
}

sdk_err_t rscs_c_rsc_meas_notify_set(uint8_t conn_idx, bool is_enable)
{
    uint16_t ntf_value = is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_rscs_c_env.handles.rscs_rsc_meas_cccd_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_rscs_c_env.handles.rscs_rsc_meas_cccd_handle, 0, 2, (uint8_t *)&ntf_value);
}

sdk_err_t rscs_c_rsc_feature_read(uint8_t conn_idx)
{
    if (BLE_ATT_INVALID_HDL == s_rscs_c_env.handles.rscs_rsc_feature_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return  ble_gattc_read(conn_idx, s_rscs_c_env.handles.rscs_rsc_feature_handle, 0);
}

sdk_err_t rscs_c_sensor_loc_read(uint8_t conn_idx)
{
    if (BLE_ATT_INVALID_HDL == s_rscs_c_env.handles.rscs_sensor_loc_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return  ble_gattc_read(conn_idx, s_rscs_c_env.handles.rscs_sensor_loc_handle, 0);
}

sdk_err_t rscs_c_ctrl_pt_indicate_set(uint8_t conn_idx, bool is_enable)
{
    uint16_t ind_value = is_enable ? PRF_CLI_START_IND : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_rscs_c_env.handles.rscs_ctrl_pt_cccd_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_rscs_c_env.handles.rscs_ctrl_pt_cccd_handle, 0, 2, (uint8_t *)&ind_value);
}

sdk_err_t rscs_c_ctrl_pt_set(uint8_t conn_idx, uint16_t ctrl_value)
{
    if (BLE_ATT_INVALID_HDL == s_rscs_c_env.handles.rscs_ctrl_pt_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_rscs_c_env.handles.rscs_ctrl_pt_handle, 0, 2, (uint8_t *)&ctrl_value);
}

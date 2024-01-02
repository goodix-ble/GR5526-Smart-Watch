/**
 *****************************************************************************************
 *
 * @file dis_c.c
 *
 * @brief Device Information Service Client Implementation.
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
#include "dis_c.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include <string.h>

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
/**@brief Device Information Service environment variable. */
struct dis_c_env_t
{
    dis_c_handles_t     handles;            /**< Handles of DIS characteristics which will be got for peer. */
    dis_c_evt_handler_t evt_handler;        /**< Handler of DIS Client event  */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct dis_c_env_t s_dis_c_env;     /**< Device Information Service Client environment variable. */
static uint8_t            s_target_uuid[2]   = {LO_U16(BLE_ATT_SVC_DEVICE_INFO), HI_U16(BLE_ATT_SVC_DEVICE_INFO)};
static ble_uuid_t         s_dis_service_uuid =
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
 * @brief Get characterisitic read type.
 *
 * @param[in]  handle: Handle of characterisitic read.
 *
 * @return Type of characterisitic read.
 *****************************************************************************************
 */
static dis_c_char_type_t dis_c_char_read_type_get(uint16_t handle)
{
    for (uint8_t i = 0; i < DIS_C_CHARACTER_NB; i++)
    {
        if (handle == s_dis_c_env.handles.dis_char_handle[i])
        {
            return (dis_c_char_type_t)i;
        }
    }

    return  DIS_C_CHARACTER_NB;
}

/**
 *****************************************************************************************
 * @brief Encode characterisitic read response.
 *
 * @param[in]  char_read_type:    Type of characterisitic read.
 * @param[in]  p_data:            Pointer to data encode.
 * @param[in]  length:            Length of data encode.
 * @param[out] p_read_rsp_buffer: Pointer to buffer for encode read response.
 *****************************************************************************************
 */
static void dis_c_char_read_rsp_encode(dis_c_char_type_t char_read_type,
                                       uint8_t *p_data,
                                       uint16_t length,
                                       ble_dis_c_read_rsp_t *p_read_rsp_buffer)
{
    p_read_rsp_buffer->char_type = char_read_type;

    if (DIS_C_SYS_ID == char_read_type)
    {
        p_read_rsp_buffer->encode_rst.sys_id.manufacturer_id[0] = p_data[0];
        p_read_rsp_buffer->encode_rst.sys_id.manufacturer_id[1] = p_data[1];
        p_read_rsp_buffer->encode_rst.sys_id.manufacturer_id[2] = p_data[2];
        p_read_rsp_buffer->encode_rst.sys_id.manufacturer_id[3] = p_data[3];
        p_read_rsp_buffer->encode_rst.sys_id.manufacturer_id[4] = p_data[4];
        p_read_rsp_buffer->encode_rst.sys_id.org_unique_id[0]   = p_data[5];
        p_read_rsp_buffer->encode_rst.sys_id.org_unique_id[1]   = p_data[6];
        p_read_rsp_buffer->encode_rst.sys_id.org_unique_id[2]   = p_data[7];
    }
    else if (DIS_C_CERT_LIST == char_read_type)
    {
        p_read_rsp_buffer->encode_rst.cert_list.p_list      = p_data;
        p_read_rsp_buffer->encode_rst.cert_list.list_length = length;
    }
    else if (DIS_C_PNP_ID == char_read_type)
    {
        p_read_rsp_buffer->encode_rst.pnp_id.vendor_id_source = p_data[0] | p_data[1] << 8;
        p_read_rsp_buffer->encode_rst.pnp_id.vendor_id        = p_data[2] | p_data[3] << 8;
        p_read_rsp_buffer->encode_rst.pnp_id.product_id       = p_data[4] | p_data[5] << 8;
        p_read_rsp_buffer->encode_rst.pnp_id.product_version  = p_data[6] | p_data[7] << 8;
    }
    else
    {
        p_read_rsp_buffer->encode_rst.string_data.p_data = p_data;
        p_read_rsp_buffer->encode_rst.string_data.length = length;
    }
}

/**
 *****************************************************************************************
 * @brief Excute Device Information Service Client event handler.
 *
 * @param[in] p_evt: Pointer to Device Information Service Client event structure.
 *****************************************************************************************
 */
void dis_c_evt_handler_excute(dis_c_evt_t *p_evt)
{
    if (NULL != s_dis_c_env.evt_handler && DIS_C_EVT_INVALID != p_evt->evt_type)
    {
        s_dis_c_env.evt_handler(p_evt);
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
static void dis_c_att_read_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_read_t *p_read_rsp)
{
    dis_c_evt_t       dis_c_evt;
    dis_c_char_type_t char_read_type = DIS_C_CHARACTER_NB;

    dis_c_evt.conn_idx = conn_idx;
    dis_c_evt.evt_type = DIS_C_EVT_INVALID;

    if (BLE_SUCCESS != status)
    {
        return;
    }

    if ((p_read_rsp->value[0].handle >= s_dis_c_env.handles.dis_srvc_start_handle) && \
        (p_read_rsp->value[0].handle <= s_dis_c_env.handles.dis_srvc_end_handle))
    {
        char_read_type = dis_c_char_read_type_get(p_read_rsp->value[0].handle);
        dis_c_char_read_rsp_encode(char_read_type, p_read_rsp->value[0].p_value, p_read_rsp->value[0].length, &dis_c_evt.read_rsp);
        dis_c_evt.evt_type = DIS_C_EVT_DEV_INFORMATION_READ_RSP;
        dis_c_evt_handler_excute(&dis_c_evt);
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
static void dis_c_srvc_browse_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_browse_srvc_t *p_browse_srvc)
{
    dis_c_evt_t  dis_c_evt;
    uint16_t     uuid_disc;
    uint16_t     handle_disc;

    dis_c_evt.conn_idx = conn_idx;
    dis_c_evt.evt_type = DIS_C_EVT_DISCOVERY_FAIL;

    if(BLE_GATT_ERR_BROWSE_NO_ANY_MORE == status)
    {
        return;
    }

    if (BLE_SUCCESS == status)
    {
        uuid_disc = p_browse_srvc->uuid[0] | p_browse_srvc->uuid[1] << 8;

        if (BLE_ATT_SVC_DEVICE_INFO == uuid_disc)
        {
            s_dis_c_env.handles.dis_srvc_start_handle = p_browse_srvc->start_hdl;
            s_dis_c_env.handles.dis_srvc_end_handle   = p_browse_srvc->end_hdl;

            for (uint32_t i = 0; i < (p_browse_srvc->end_hdl - p_browse_srvc->start_hdl); i++)
            {
                uuid_disc   = p_browse_srvc->info[i].attr.uuid[0] | p_browse_srvc->info[i].attr.uuid[1] << 8;
                handle_disc = p_browse_srvc->start_hdl + i + 1;

                if (BLE_GATTC_BROWSE_ATTR_VAL == p_browse_srvc->info[i].attr_type)
                {
                    if (BLE_ATT_CHAR_SYS_ID == uuid_disc)
                    {
                        s_dis_c_env.handles.dis_char_handle[DIS_C_SYS_ID] = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_MODEL_NB == uuid_disc)
                    {
                        s_dis_c_env.handles.dis_char_handle[DIS_C_MODEL_NUM] = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_SERIAL_NB == uuid_disc)
                    {
                        s_dis_c_env.handles.dis_char_handle[DIS_C_SERIAL_NUM] = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_FW_REV == uuid_disc)
                    {
                        s_dis_c_env.handles.dis_char_handle[DIS_C_FW_REV] = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_HW_REV == uuid_disc)
                    {
                        s_dis_c_env.handles.dis_char_handle[DIS_C_HW_REV] = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_SW_REV == uuid_disc)
                    {
                        s_dis_c_env.handles.dis_char_handle[DIS_C_SW_REV] = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_MANUF_NAME == uuid_disc)
                    {
                        s_dis_c_env.handles.dis_char_handle[DIS_C_MANUF_NAME] = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_IEEE_CERTIF == uuid_disc)
                    {
                        s_dis_c_env.handles.dis_char_handle[DIS_C_CERT_LIST] = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_PNP_ID == uuid_disc)
                    {
                        s_dis_c_env.handles.dis_char_handle[DIS_C_PNP_ID] = handle_disc;
                    }
                }
                else if (BLE_GATTC_BROWSE_NONE == p_browse_srvc->info[i].attr_type)
                {
                    break;
                }
            }

            dis_c_evt.evt_type = DIS_C_EVT_DISCOVERY_COMPLETE;
        }
    }

    dis_c_evt_handler_excute(&dis_c_evt);
}

static void dis_c_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTC_EVT_SRVC_BROWSE:
            dis_c_srvc_browse_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.srvc_browse);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            dis_c_att_read_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.read_rsp);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t dis_client_init(dis_c_evt_handler_t evt_handler)
{
    if (NULL == evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_dis_c_env, 0, sizeof(s_dis_c_env));
    s_dis_c_env.evt_handler = evt_handler;

    return ble_gattc_prf_add(&s_dis_service_uuid,  dis_c_ble_evt_handler);
}

sdk_err_t dis_c_disc_srvc_start(uint8_t conn_idx)
{

    return ble_gattc_services_browse(conn_idx, &s_dis_service_uuid);
}

sdk_err_t dis_c_char_value_read(uint8_t conn_idx, dis_c_char_type_t char_read_type)
{
    uint16_t target_handle = BLE_ATT_INVALID_HDL;

    target_handle = s_dis_c_env.handles.dis_char_handle[char_read_type];

    if (BLE_ATT_INVALID_HDL == target_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return  ble_gattc_read(conn_idx, target_handle, 0);
}


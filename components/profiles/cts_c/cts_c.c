/**
 ****************************************************************************************
 *
 * @file cts_c.c
 *
 * @brief Current Time Service Client implementation.
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
#include "cts_c.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include <string.h>
#include "app_log.h"

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
/**@brief Current Time Service Client environment variable. */
struct cts_c_env_t
{
    cts_c_handles_t      handles;            /**< Handles of CTS characteristics which will be got for peer. */
    cts_c_evt_handler_t  evt_handler;        /**< Handler of CTS Client event  */
    uint8_t              prf_id;             /**< CTS Client profile id. */
};
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct cts_c_env_t  s_cts_c_env;         /**< Current Time Service Client environment variable. */
static uint8_t             s_target_uuid[2]   = {LO_U16(BLE_ATT_SVC_CURRENT_TIME), HI_U16(BLE_ATT_SVC_CURRENT_TIME)};
static ble_uuid_t          s_cts_service_uuid =
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
 * @brief Excute Current Time Service Client event handler.
 *
 * @param[in] p_evt: Pointer to Current Time Service Client event structure.
 *****************************************************************************************
 */
static void cts_c_evt_handler_excute(cts_c_evt_t *p_evt)
{
    if (NULL != s_cts_c_env.evt_handler && CTS_C_EVT_INVALID != p_evt->evt_type)
    {
        s_cts_c_env.evt_handler(p_evt);
    }
}

/**
 *****************************************************************************************
 * @brief Encode a Current Time.
 *
 * @param[in]  p_cur_time:     Pointer to Current Time value to be encoded.
 * @param[out] p_encoded_data: Pointer to encoded data will be written.
 *****************************************************************************************
 */
static void cts_c_cur_time_encode(const cts_c_cur_time_t *p_cur_time, uint8_t *p_encoded_data)
{
    prf_pack_date_time(p_encoded_data, &p_cur_time->day_date_time.date_time);

    p_encoded_data[7] = p_cur_time->day_date_time.day_of_week;
    p_encoded_data[8] = p_cur_time->day_date_time.fractions_256;
    p_encoded_data[9] = p_cur_time->adjust_reason;
}

/**
 *****************************************************************************************
 * @brief Decode for a Current Time.
 *
 * @param[in]  p_data:     Pointer to data to be decoded.
 * @param[out] p_cur_time: Pointer to Current Time.
 *****************************************************************************************
 */
static void cts_c_cur_time_decode(const uint8_t *p_data, cts_c_cur_time_t *p_cur_time)
{
    prf_unpack_date_time(p_data, &p_cur_time->day_date_time.date_time);

    p_cur_time->day_date_time.day_of_week   = p_data[7];
    p_cur_time->day_date_time.fractions_256 = p_data[8];
    p_cur_time->adjust_reason               = p_data[9];
}

/**
 *****************************************************************************************
 * @brief Decode for a Reference Time Information.
 *
 * @param[in]  p_data:          Pointer to data to be decoded.
 * @param[out] p_ref_time_info: Pointer to Reference Time Information.
 *****************************************************************************************
 */
static void cts_c_ref_time_info_decode(const uint8_t *p_data, cts_c_ref_time_info_t *p_ref_time_info)
{
    p_ref_time_info->source             = (cts_c_ref_time_source_t)p_data[0];
    p_ref_time_info->accuracy           = p_data[1];
    p_ref_time_info->days_since_update  = p_data[2];
    p_ref_time_info->hours_since_update = p_data[3];
}

/**
 *****************************************************************************************
 * @brief Check Current Time value is valid or not
 *
 * @param[out] p_cur_time: Pointer to Current Time.
 *****************************************************************************************
 */
static bool cts_c_cur_time_valid_check(cts_c_cur_time_t *p_cur_time)
{
    if ((p_cur_time->day_date_time.date_time.year > CTS_C_TIME_YEAR_VALID_VAL_MAX) || \
        ((p_cur_time->day_date_time.date_time.year < CTS_C_TIME_YEAR_VALID_VAL_MIN) && \
         (CTS_C_TIME_Y_M_D_UNKNOWN != p_cur_time->day_date_time.date_time.year)))
    {
        return false;
    }

    if (p_cur_time->day_date_time.date_time.month > 12)
    {
        return false;
    }

    if (p_cur_time->day_date_time.date_time.day > 31)
    {
        return false;
    }

    if (p_cur_time->day_date_time.date_time.hour > 23)
    {
        return false;
    }

    if (p_cur_time->day_date_time.date_time.min > 59)
    {
        return false;
    }

    if (p_cur_time->day_date_time.date_time.sec > 59)
    {
        return false;
    }

    if (p_cur_time->day_date_time.day_of_week > CTS_C_WEEK_SUNDAY)
    {
        return false;
    }

    if (p_cur_time->adjust_reason > 0x0f)
    {
        return false;
    }
    return true;
}

/**
 *****************************************************************************************
 * @brief Check Local Time Information value is valid or not
 *
 * @param[out] p_loc_time_info: Pointer to Local Time Information.
 *****************************************************************************************
 */
static bool cts_c_loc_time_info_valid_check(cts_c_loc_time_info_t *p_loc_time_info)
{
    if ((p_loc_time_info->time_zone < CTS_C_TIME_ZONE_OFFSET_MIN) || \
         (p_loc_time_info->time_zone > CTS_C_TIME_ZONE_OFFSET_MAX))
    {
        return false;
    }

    if ((CTS_C_DST_OFFSET_STANDAR_TIME != p_loc_time_info->dst_offset) && \
        (CTS_C_DST_OFFSET_HALF_HOUR != p_loc_time_info->dst_offset) && \
        (CTS_C_DST_OFFSET_DAYLIGHT_TIME != p_loc_time_info->dst_offset) && \
        (CTS_C_DST_OFFSET_DOUB_DAYLIGHT_TIME != p_loc_time_info->dst_offset))
    {
        return false;
    }

    return true;
}

/**
 *****************************************************************************************
 * @brief Check Reference Time Information value is valid or not
 *
 * @param[out] p_ref_time_info: Pointer to Local Time Information.
 *****************************************************************************************
 */
static bool cts_c_ref_time_info_valid_check(cts_c_ref_time_info_t *p_ref_time_info)
{
    if (p_ref_time_info->source > CTS_C_REF_TIME_SRC_CELLUAR_NET)
    {
        return false;
    }

    return true;
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
static void cts_c_att_read_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_read_t *p_read_rsp)
{
    cts_c_evt_t cts_c_evt;

    cts_c_evt.conn_idx = conn_idx;
    cts_c_evt.evt_type = CTS_C_EVT_INVALID;

    if (BLE_SUCCESS != status)
    {
        return;
    }

    if (p_read_rsp->value[0].handle == s_cts_c_env.handles.cts_cur_time_handle)
    {
        cts_c_cur_time_decode(p_read_rsp->value[0].p_value, &cts_c_evt.value.cur_time);

        if (cts_c_cur_time_valid_check(&cts_c_evt.value.cur_time))
        {
            cts_c_evt.evt_type = CTS_C_EVT_VALID_CUR_TIME_REC;
        }
        else
        {
            cts_c_evt.evt_type = CTS_C_EVT_INVALID_CUR_TIME_REC;
        }
    }
    else if (p_read_rsp->value[0].handle == s_cts_c_env.handles.cts_loc_time_info_handle)
    {
        cts_c_evt.value.loc_time_info.time_zone  = p_read_rsp->value[0].p_value[0];
        cts_c_evt.value.loc_time_info.dst_offset = (cts_c_dst_offset_t)p_read_rsp->value[0].p_value[1];

        if (cts_c_loc_time_info_valid_check(&cts_c_evt.value.loc_time_info))
        {
            cts_c_evt.evt_type = CTS_C_EVT_VALID_LOC_TIME_INFO_REC;
        }
        else
        {
            cts_c_evt.evt_type = CTS_C_EVT_INVALID_LOC_TIME_INFO_REC;
        }
    }
    else if (p_read_rsp->value[0].handle == s_cts_c_env.handles.cts_ref_time_info_handle)
    {
        cts_c_ref_time_info_decode(p_read_rsp->value[0].p_value, &cts_c_evt.value.ref_time_info);

        if (cts_c_ref_time_info_valid_check(&cts_c_evt.value.ref_time_info))
        {
            cts_c_evt.evt_type = CTS_C_EVT_VALID_REF_TIME_INFO_REC;
        }
        else
        {
            cts_c_evt.evt_type = CTS_C_EVT_INVALID_REF_TIME_INFO_REC;
        }
    }

    cts_c_evt_handler_excute(&cts_c_evt);
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
static void cts_c_att_write_evt_handler(uint8_t conn_idx, uint8_t status, uint16_t handle)
{
    cts_c_evt_t cts_c_evt;

    cts_c_evt.conn_idx = conn_idx;
    cts_c_evt.evt_type = CTS_C_EVT_INVALID;

    if (handle == s_cts_c_env.handles.cts_cur_time_cccd_handle)
    {
        cts_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              CTS_C_EVT_CUR_TIME_NTF_SET_SUCCESS :
                              CTS_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_cts_c_env.handles.cts_cur_time_handle)
    {
        cts_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              CTS_C_EVT_CUR_TIME_SET_SUCCESS :
                              CTS_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_cts_c_env.handles.cts_loc_time_info_handle)
    {
        cts_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              CTS_C_EVT_LOC_TIME_INFO_SET_SUCCESS :
                              CTS_C_EVT_WRITE_OP_ERR;
    }

    cts_c_evt_handler_excute(&cts_c_evt);
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
static void cts_c_att_ntf_ind_evt_handler(uint8_t conn_idx, const ble_gattc_evt_ntf_ind_t *p_ntf_ind)
{
    cts_c_evt_t cts_c_evt;

    cts_c_evt.conn_idx = conn_idx;
    cts_c_evt.evt_type = CTS_C_EVT_INVALID;

    if (p_ntf_ind->handle == s_cts_c_env.handles.cts_cur_time_handle)
    {
        cts_c_cur_time_decode(p_ntf_ind->p_value, &cts_c_evt.value.cur_time);

        if (cts_c_cur_time_valid_check(&cts_c_evt.value.cur_time))
        {
            cts_c_evt.evt_type = CTS_C_EVT_VALID_CUR_TIME_REC;
        }
        else
        {
            cts_c_evt.evt_type = CTS_C_EVT_INVALID_CUR_TIME_REC;
        }
    }

    cts_c_evt_handler_excute(&cts_c_evt);
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
static void cts_c_srvc_browse_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_browse_srvc_t *p_browse_srvc)
{
    cts_c_evt_t  cts_c_evt;
    uint16_t     uuid_disc;
    uint16_t     handle_disc;

    cts_c_evt.conn_idx = conn_idx;
    cts_c_evt.evt_type = CTS_C_EVT_DISCOVERY_FAIL;

    if(BLE_GATT_ERR_BROWSE_NO_ANY_MORE == status)
    {
        return;
    }

    if (BLE_SUCCESS == status)
    {
        uuid_disc = p_browse_srvc->uuid[0] | p_browse_srvc->uuid[1] << 8;

        if (BLE_ATT_SVC_CURRENT_TIME == uuid_disc)
        {
            s_cts_c_env.handles.cts_srvc_start_handle = p_browse_srvc->start_hdl;
            s_cts_c_env.handles.cts_srvc_end_handle   = p_browse_srvc->end_hdl;

            for (uint32_t i = 0; i < (p_browse_srvc->end_hdl - p_browse_srvc->start_hdl); i++)
            {
                uuid_disc   = p_browse_srvc->info[i].attr.uuid[0] | p_browse_srvc->info[i].attr.uuid[1] << 8;
                handle_disc = p_browse_srvc->start_hdl + i + 1;

                if (BLE_GATTC_BROWSE_ATTR_VAL == p_browse_srvc->info[i].attr_type)
                {
                    if (BLE_ATT_CHAR_CT_TIME == uuid_disc)
                    {
                        s_cts_c_env.handles.cts_cur_time_handle      = handle_disc;
                        s_cts_c_env.handles.cts_cur_time_cccd_handle = handle_disc + 1;
                    }
                    else if (BLE_ATT_CHAR_LOCAL_TIME_INFO == uuid_disc)
                    {
                        s_cts_c_env.handles.cts_loc_time_info_handle = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_REFERENCE_TIME_INFO == uuid_disc)
                    {
                        s_cts_c_env.handles.cts_ref_time_info_handle = handle_disc;
                    }
                }
                else if (p_browse_srvc->info[i].attr_type == BLE_GATTC_BROWSE_NONE)
                {
                    break;
                }
            }

            cts_c_evt.evt_type = CTS_C_EVT_DISCOVERY_COMPLETE;
        }
    }
    cts_c_evt_handler_excute(&cts_c_evt);
}

static void cts_c_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTC_EVT_SRVC_BROWSE:
            cts_c_srvc_browse_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.srvc_browse);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            cts_c_att_read_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.read_rsp);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            cts_c_att_write_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, p_evt->evt.gattc_evt.params.write_rsp.handle);
            break;

        case BLE_GATTC_EVT_NTF_IND:
            cts_c_att_ntf_ind_evt_handler(p_evt->evt.gattc_evt.index, &p_evt->evt.gattc_evt.params.ntf_ind);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t cts_client_init(cts_c_evt_handler_t evt_handler)
{
    if (NULL == evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_cts_c_env, 0, sizeof(s_cts_c_env));
    s_cts_c_env.evt_handler = evt_handler;

    return ble_gattc_prf_add(&s_cts_service_uuid, cts_c_ble_evt_handler);
}

sdk_err_t cts_c_disc_srvc_start(uint8_t conn_idx)
{
    uint8_t target_uuid[2];

    target_uuid[0] = LO_U16(BLE_ATT_SVC_CURRENT_TIME);
    target_uuid[1] = HI_U16(BLE_ATT_SVC_CURRENT_TIME);

    const ble_uuid_t cts_service_uuid =
    {
        .uuid_len = 2,
        .uuid     = target_uuid,
    };

    return ble_gattc_services_browse(conn_idx, &cts_service_uuid);
}

sdk_err_t cts_c_cur_time_notify_set(uint8_t conn_idx, bool is_enable)
{
    uint16_t ntf_value = is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_cts_c_env.handles.cts_cur_time_cccd_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_cts_c_env.handles.cts_cur_time_cccd_handle, 0, 2, (uint8_t *)&ntf_value);
}

sdk_err_t cts_c_cur_time_read(uint8_t conn_idx)
{
    if (BLE_ATT_INVALID_HDL == s_cts_c_env.handles.cts_cur_time_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_read(conn_idx, s_cts_c_env.handles.cts_cur_time_handle, 0);
}

sdk_err_t cts_c_loc_time_info_read(uint8_t conn_idx)
{
    if (BLE_ATT_INVALID_HDL == s_cts_c_env.handles.cts_loc_time_info_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_read(conn_idx, s_cts_c_env.handles.cts_loc_time_info_handle, 0);
}

sdk_err_t cts_c_ref_time_info_read(uint8_t conn_idx)
{
    if (BLE_ATT_INVALID_HDL == s_cts_c_env.handles.cts_ref_time_info_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_read(conn_idx, s_cts_c_env.handles.cts_ref_time_info_handle, 0);
}

sdk_err_t cts_c_cur_time_set(uint8_t conn_idx, cts_c_cur_time_t *p_cur_time)
{
    uint8_t encoded_buffer[CTS_C_CUR_TIME_VAL_LEN];

    if (BLE_ATT_INVALID_HDL == s_cts_c_env.handles.cts_cur_time_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    cts_c_cur_time_encode(p_cur_time, encoded_buffer);

    return ble_gattc_write(conn_idx, s_cts_c_env.handles.cts_cur_time_handle, 0, CTS_C_CUR_TIME_VAL_LEN, (uint8_t *)&encoded_buffer);
}

sdk_err_t cts_c_loc_time_info_set(uint8_t conn_idx, cts_c_loc_time_info_t *p_loc_time_info)
{
    uint8_t encoded_buffer[CTS_C_LOC_TIME_INFO_VAL_LEN];

    if (BLE_ATT_INVALID_HDL == s_cts_c_env.handles.cts_loc_time_info_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    encoded_buffer[0] = p_loc_time_info->time_zone;
    encoded_buffer[1] = p_loc_time_info->dst_offset;

    return ble_gattc_write(conn_idx, s_cts_c_env.handles.cts_loc_time_info_handle, 0, CTS_C_LOC_TIME_INFO_VAL_LEN, encoded_buffer);
}

void cts_c_data_parse(uint8_t *p_data, uint16_t length)
{
    if(0 == memcmp(p_data, "CR", 2))
    {
       APP_LOG_DEBUG("Read Current Time value.");
       cts_c_cur_time_read(0);
    }
    else if(0 == memcmp(p_data, "LR", 2))
    {
       APP_LOG_DEBUG("Read Local Time Information value.");
       cts_c_loc_time_info_read(0);
    }
    else if(0 == memcmp(p_data, "RR", 2))
    {
       APP_LOG_DEBUG("Read Reference Time Information value.");
       cts_c_ref_time_info_read(0);
    }
    else if(0 == memcmp(p_data, "EN", 2))
    {
      APP_LOG_INFO("Enabled Current Time Notification.");
      cts_c_cur_time_notify_set(0, true);

    }
    else if(0 == memcmp(p_data, "DN", 2))
    {
      APP_LOG_INFO("Disabled Current Time Notification.");
      cts_c_cur_time_notify_set(0, false);
    }
}


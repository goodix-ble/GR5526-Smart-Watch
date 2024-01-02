/**
 ****************************************************************************************
 *
 * @file cts.c
 *
 * @brief Current Time Service implementation.
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
#include "cts.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include "app_log.h"
#include "user_app.h"

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
 /**@brief The time zone adjustment sent by the receiving host. */
enum
{
    CTS_HOST_DATA_ZONE,             /**< Accept zone receipts from the host. */
    CTS_HOST_DATA_OFFST,            /**< Accept offst receipts from the host. */

    CTS_HOST_NB                     /**< The maximum number of data from the host. */
};

 /**@brief Accept receipts from serial ports. */
enum
{
    CTS_SERIAL_DATA_SOURCE,              /**< Source of information. */
    CTS_SERIAL_DATA_ACCURACY,            /**< Drift accuracy. */
    CTS_SERIAL_DAY_SINCE,                /**< Days since update.*/
    CTS_SERIAL_HOURS_SINCE,              /**< hours since update. */

    CTS_SERIAL_NB                        /**< The maximum number of data from the serial port. */
};

 /**@brief Accept receipts from peers. */
enum
{
    CTS_PEER_DATA_YEAR,           /**< Receive data from the peer year. */
    CTS_PEER_DATA_MONTH,          /**< Receive data from the peer month. */
    CTS_PEER_DATA_DAY,            /**< Receive data from the peer day. */
    CTS_PEER_DATA_HOUR,           /**< Receive data from the peer hour */
    CTS_PEER_DATA_MIN,            /**< Receive data from the peer minute */
    CTS_PEER_DATA_SEC,            /**< Receive data from the peer second */
    CTS_PEER_DATA_WEEK,           /**< Receive data from the peer week */
    CTS_PEER_DATA_FRACTION,       /**< Receive data from the peer fracyion */
    CTS_PEER_DATA_REASON,         /**< Receive data from the peer Reason for adjustment */

    CTS_PEER_NB                   /**< The maximum number of data from peer devices */
};
/**@brief Current Time Service Attributes Indexes. */
enum
{
    // Current Time Service
    CTS_IDX_SVC,

    // Current Time
    CTS_IDX_CUR_TIME_CHAR,
    CTS_IDX_CUR_TIME_VAL,
    CTS_IDX_CUR_TIMR_NTF_CFG,

    // Local Time Information
    CTS_IDX_LOC_TIME_INFO_CHAR,
    CTS_IDX_LOC_TIME_INFO_VAL,

    // Reference Time Information
    CTS_IDX_REF_TIME_INFO_CHAR,
    CTS_IDX_REF_TIME_INFO_VAL,

    CTS_IDX_NB
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Current Time Service environment variable. */
struct cts_env_t
{
    cts_init_t            cts_init;                              /**< Current Time Service initialization variables. */
    uint16_t              start_hdl;                             /**< Current Time Service start handle. */
    uint16_t              cur_time_ntf_cfg[CTS_CONNECTION_MAX];  /**< The configuration of Current Time Notification which is configured by the peer devices. */
    ble_gatts_create_db_t cts_gatts_db;                          /**< Current Time Service attributs database. */
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void        cts_cur_time_read_handler(ble_gatts_read_cfm_t *p_cfm, uint8_t *p_encode_buffer);
static void        cts_loc_time_info_read_handler(ble_gatts_read_cfm_t *p_cfm, uint8_t *p_encode_buffer);
static void        cts_ref_time_info_read_handler(ble_gatts_read_cfm_t *p_cfm, uint8_t *p_encode_buffer);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct cts_env_t s_cts_env;
static struct cts_env_t s_cts_record_time;
static cts_adj_info_t   s_cts_ref_info;
static cts_updata_ref_time_info_t s_cts_updata;
static uint8_t    s_cts_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_CURRENT_TIME);

/**@brief Full CTS Database Description - Used to add attributes into the database. */
static const ble_gatts_attm_desc_t cts_attr_tab[CTS_IDX_NB] =
{
    // CTS Service Declaration
    [CTS_IDX_SVC] = {BLE_ATT_DECL_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // Current Time Characteristic Declaration
    [CTS_IDX_CUR_TIME_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Current Time Characteristic Declaration value
    [CTS_IDX_CUR_TIME_VAL]     = {BLE_ATT_CHAR_CT_TIME,
                                  BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_NOTIFY_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                  BLE_GATTS_ATT_VAL_LOC_USER,
                                  TOTAL_CTS_CUR_TIME_VAL_LEN},
    // Current Time Characteristic Declaration  - Client Characteristic Configuration Descriptor
    [CTS_IDX_CUR_TIMR_NTF_CFG] = {BLE_ATT_DESC_CLIENT_CHAR_CFG, BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC, 0, 0},

    // Local Time Information Characteristic Declaration
    [CTS_IDX_LOC_TIME_INFO_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Local Time Information Characteristic Value
    [CTS_IDX_LOC_TIME_INFO_VAL]  = {BLE_ATT_CHAR_LOCAL_TIME_INFO,
                                    BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                    BLE_GATTS_ATT_VAL_LOC_USER,
                                    TOTAL_CTS_LOC_TIME_INFO_VAL_LEN},

    // Reference Time Information Characteristic Declaration
    [CTS_IDX_REF_TIME_INFO_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Reference Time Information Characteristic Value
    [CTS_IDX_REF_TIME_INFO_VAL]  = {BLE_ATT_CHAR_REFERENCE_TIME_INFO,
                                    BLE_GATTS_READ_PERM_UNSEC,
                                    BLE_GATTS_ATT_VAL_LOC_USER,
                                    CTS_REF_TIME_INFO_VAL_LEN},
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_param:  The parameters of the read request.
 *****************************************************************************************
 */
static void cts_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t  cfm;
    uint8_t               handle    = p_param->handle;
    uint8_t               tab_index = prf_find_idx_by_handle(handle,
                                      s_cts_env.start_hdl,
                                      CTS_IDX_NB,
                                      (uint8_t *)&s_cts_env.cts_init.char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case CTS_IDX_CUR_TIME_VAL:
        {
            uint8_t encoded_buffer[CTS_CUR_TIME_VAL_LEN];
            cts_cur_time_read_handler(&cfm, encoded_buffer);
            APP_LOG_INFO("Read current time success.");
            break;
        }

        case CTS_IDX_CUR_TIMR_NTF_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_cts_env.cur_time_ntf_cfg[conn_idx];
            break;

        case CTS_IDX_LOC_TIME_INFO_VAL:
        {
            uint8_t encoded_buffer[CTS_LOC_TIME_INFO_VAL_LEN];
            cts_loc_time_info_read_handler(&cfm, encoded_buffer);
            APP_LOG_INFO("Read local time success.");
            break;
        }

        case CTS_IDX_REF_TIME_INFO_VAL:
        {
            uint8_t encoded_buffer[CTS_REF_TIME_INFO_VAL_LEN];
            cts_ref_time_info_read_handler(&cfm, encoded_buffer);
            APP_LOG_INFO("Read reference time success.");
            break;
        }

        default:
            cfm.length = 0;
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_read_cfm(conn_idx, &cfm);
}

/**
 *****************************************************************************************
 * @brief Handles reception of the write request.
 *
 * @param[in]: conn_idx: Connection index
 * @param[in]: p_param:  The parameters of the write request.
 *****************************************************************************************
 */
static void   cts_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t              handle      = p_param->handle;
    uint16_t              tab_index   = 0;
    uint16_t              cccd_value  = 0;
    cts_evt_t             event;
    ble_gatts_write_cfm_t cfm;

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_cts_env.start_hdl,
                                        CTS_IDX_NB,
                                        (uint8_t *)&s_cts_env.cts_init.char_mask);
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    event.evt_type = CTS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case CTS_IDX_CUR_TIME_VAL:
            event.evt_type = CTS_EVT_CUR_TIME_SET_BY_PEER;
            event.p_data   = p_param->value;
            event.length   = p_param->length;
            event.length   = current_time_universal_decode(&cfm, &event);
            memcpy(&event.cur_time, &s_cts_env.cts_init.cur_time, sizeof(cts_cur_time_t));
            break;

        case CTS_IDX_CUR_TIMR_NTF_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              CTS_EVT_CUR_TIME_NOTIFICATION_ENABLED : \
                              CTS_EVT_CUR_TIME_NOTIFICATION_DISABLED);
            s_cts_env.cur_time_ntf_cfg[conn_idx] = cccd_value;
            break;

        case CTS_IDX_LOC_TIME_INFO_VAL:
            event.evt_type = CTS_EVT_LOC_TIME_INFO_SET_BY_PEER;
            event.p_data   = p_param->value;
            event.length   = p_param->length;
            event.length   = local_time_universal_decode(&cfm, &event);
            memcpy(&event.loc_time_info, &s_cts_env.cts_init.loc_time_info, sizeof(cts_loc_time_info_t));
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_write_cfm(conn_idx, &cfm);

    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && CTS_EVT_INVALID != event.evt_type && s_cts_env.cts_init.evt_handler)
    {
        s_cts_env.cts_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the cccd recover request.
 *
 * @param[in]: conn_idx:   Connection index
 * @param[in]: handle:     The handle of cccd attribute.
 * @param[in]: cccd_value: The value of cccd attribute.
 *****************************************************************************************
 */
static void cts_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint16_t          tab_index   = 0;
    cts_evt_t         event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_cts_env.start_hdl,
                                        CTS_IDX_NB,
                                        (uint8_t *)&s_cts_env.cts_init.char_mask);

    event.evt_type = CTS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case CTS_IDX_CUR_TIMR_NTF_CFG:
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              CTS_EVT_CUR_TIME_NOTIFICATION_ENABLED : \
                              CTS_EVT_CUR_TIME_NOTIFICATION_DISABLED);
            s_cts_env.cur_time_ntf_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }

    if (CTS_EVT_INVALID != event.evt_type && s_cts_env.cts_init.evt_handler)
    {
        s_cts_env.cts_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the complete event.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_param:  Pointer to the parameters of the complete event.
 *****************************************************************************************
 */
static void cts_ntf_ind_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    if (BLE_GATT_NOTIFICATION == p_ntf_ind->type)
    {
        s_cts_env.cts_init.cur_time.adjust_reason = s_cts_env.cts_init.cur_time.adjust_reason;
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
static void cts_cur_time_encode(const cts_cur_time_t *p_cur_time, uint8_t *p_encoded_data)
{
    prf_pack_date_time(p_encoded_data, &p_cur_time->day_date_time.date_time);

    p_encoded_data[7] = p_cur_time->day_date_time.day_of_week;
    p_encoded_data[8] = p_cur_time->day_date_time.fractions_256;
    p_encoded_data[9] = p_cur_time->adjust_reason;
}

/**
 *****************************************************************************************
 * @brief Handle Current Time read event.
 *
 * @param[out] p_cfm:           Pointer to GATT read attribute result description.
 * @param[out] p_encode_buffer: Pointer to encoded data will be written.
 *****************************************************************************************
 */
static void cts_cur_time_read_handler(ble_gatts_read_cfm_t *p_cfm, uint8_t *p_encode_buffer)
{
    cts_cur_time_encode(&s_cts_env.cts_init.cur_time, p_encode_buffer);

    p_cfm->length = CTS_CUR_TIME_VAL_LEN;
    p_cfm->value  = p_encode_buffer;
}

/**
 *****************************************************************************************
 * @brief Handle Local Time Information read event.
 *
 * @param[out] p_cfm:           Pointer to GATT read attribute result description.
 * @param[out] p_encode_buffer: Pointer to encoded data will be written.
 *****************************************************************************************
 */
static void cts_loc_time_info_read_handler(ble_gatts_read_cfm_t *p_cfm, uint8_t *p_encode_buffer)
{
    p_encode_buffer[0] = s_cts_env.cts_init.loc_time_info.time_zone;
    p_encode_buffer[1] = s_cts_env.cts_init.loc_time_info.dst_offset;

    p_cfm->value  = p_encode_buffer;
    p_cfm->length = CTS_LOC_TIME_INFO_VAL_LEN;
}

/**
 *****************************************************************************************
 * @brief Handle Reference Time Information read event.
 *
 * @param[out] p_cfm:           Pointer to GATT read attribute result description.
 * @param[out] p_encode_buffer: Pointer to encoded data will be written.
 *****************************************************************************************
 */
static void cts_ref_time_info_read_handler(ble_gatts_read_cfm_t *p_cfm, uint8_t *p_encode_buffer)
{
    p_encode_buffer[0] = s_cts_env.cts_init.ref_time_info.source;
    p_encode_buffer[1] = s_cts_env.cts_init.ref_time_info.accuracy;
    p_encode_buffer[2] = s_cts_env.cts_init.ref_time_info.days_since_update;
    p_encode_buffer[3] = s_cts_env.cts_init.ref_time_info.hours_since_update;

    p_cfm->value  = p_encode_buffer;
    p_cfm->length = CTS_REF_TIME_INFO_VAL_LEN;
}

static void cts_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            cts_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            cts_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_NTF_IND:
            cts_ntf_ind_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            cts_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t cts_cur_time_send(uint8_t conn_idx, cts_cur_time_t *p_cur_time)
{
    sdk_err_t            error_code = SDK_ERR_NTF_DISABLED;
    uint8_t              encoded_cur_time[CTS_CUR_TIME_VAL_LEN];
    ble_gatts_noti_ind_t ct_ntf;

    cts_cur_time_encode(p_cur_time, encoded_cur_time);

    if (PRF_CLI_START_NTF == s_cts_env.cur_time_ntf_cfg[conn_idx])
    {
        ct_ntf.type   = BLE_GATT_NOTIFICATION;
        ct_ntf.handle = prf_find_handle_by_idx(CTS_IDX_CUR_TIME_VAL,
                                               s_cts_env.start_hdl,
                                               (uint8_t *)&s_cts_env.cts_init.char_mask);
        ct_ntf.length = CTS_CUR_TIME_VAL_LEN;
        ct_ntf.value  = encoded_cur_time;
        error_code    = ble_gatts_noti_ind(conn_idx, &ct_ntf);
    }

    return error_code;
}

void cts_exact_time_get(cts_init_t *p_cts_exact_time)
{
    memcpy(p_cts_exact_time, &s_cts_env.cts_init, sizeof(cts_init_t));
}

void cts_exact_time_update(cts_init_t *p_cts_exact_time)
{
   p_cts_exact_time->cur_time.adjust_reason = s_cts_env.cts_init.cur_time.adjust_reason ;
   memcpy(&s_cts_env.cts_init, p_cts_exact_time, sizeof(cts_init_t));
   memcpy(&s_cts_env.cts_init.ref_time_info,&s_cts_ref_info.ref_time_info, sizeof(cts_ref_time_info_t));

   s_cts_updata.current_time_min  = s_cts_env.cts_init.cur_time.day_date_time.date_time.min;
   s_cts_updata.current_time_sec  = s_cts_env.cts_init.cur_time.day_date_time.date_time.sec;

   if( s_cts_updata.current_time_min > s_cts_updata.expected_time_1min_adapt ||
       s_cts_updata.current_time_min+1 < s_cts_updata.expected_time_1min_adapt ||
       ( s_cts_updata.current_time_min == s_cts_updata.expected_time_1min_adapt && s_cts_updata.current_time_sec > s_cts_updata.before_updata_sec)
     )
    {
      s_cts_updata.updata_time_flag =1;
      s_cts_ref_info.ref_time_info.source  =  s_cts_updata.stage_update_source;
     if(s_cts_updata.stage_update_source   == 1)
     {
       s_cts_env.cts_init.cur_time.adjust_reason =2;
     }
    else if(s_cts_updata.stage_update_source   == 4)
     {
       s_cts_env.cts_init.cur_time.adjust_reason =1;
     }
    }

   else
    {
      s_cts_updata.updata_time_flag =0;
    }

   cts_cur_time_send(0, &s_cts_env.cts_init.cur_time);
}

void cts_cur_time_adjust(cts_adj_info_t *p_adj_info)
{
    if (CTS_AR_MAUAL_TIME_UPDATE & p_adj_info->adjust_reason)
    {
        memcpy(&s_cts_env.cts_init.cur_time.day_date_time, &p_adj_info->day_date_time, sizeof(cts_exact_time_256_t));
    }

    if (CTS_AR_EXT_REF_TIME_UPDATE & p_adj_info->adjust_reason)
    {
        memcpy(&s_cts_env.cts_init.ref_time_info, &p_adj_info->ref_time_info, sizeof(cts_ref_time_info_t));
    }

    if (CTS_AR_TIME_ZONE_CHANGE & p_adj_info->adjust_reason)
    {
        s_cts_env.cts_init.loc_time_info.time_zone = p_adj_info->loc_time_info.time_zone;
    }

    if (CTS_AR_DST_CHANGE & p_adj_info->adjust_reason)
    {
        s_cts_env.cts_init.loc_time_info.dst_offset = p_adj_info->loc_time_info.dst_offset;
    }

    s_cts_env.cts_init.cur_time.adjust_reason = p_adj_info->adjust_reason;

    cts_cur_time_send(0, &s_cts_env.cts_init.cur_time);
}

sdk_err_t cts_service_init(cts_init_t *p_cts_init)
{
    if (NULL == p_cts_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_cts_env, 0, sizeof(s_cts_env));
    memcpy(&s_cts_env.cts_init, p_cts_init, sizeof(cts_init_t));

    memset(&s_cts_env.cts_gatts_db, 0, sizeof(ble_gatts_create_db_t));

    s_cts_env.start_hdl                         = PRF_INVALID_HANDLE; 
    s_cts_env.cts_gatts_db.shdl                 = &s_cts_env.start_hdl; 
    s_cts_env.cts_gatts_db.uuid                 = s_cts_svc_uuid; 
    s_cts_env.cts_gatts_db.attr_tab_cfg         = (uint8_t *)&(s_cts_env.cts_init.char_mask); 
    s_cts_env.cts_gatts_db.max_nb_attr          = CTS_IDX_NB; 
    s_cts_env.cts_gatts_db.srvc_perm            = 0; 
    s_cts_env.cts_gatts_db.attr_tab_type        = BLE_GATTS_SERVICE_TABLE_TYPE_16; 
    s_cts_env.cts_gatts_db.attr_tab.attr_tab_16 = cts_attr_tab; 

    return ble_gatts_prf_add(&s_cts_env.cts_gatts_db, cts_ble_evt_handler);
}

void cts_c_data_parse(uint8_t *p_data, uint16_t length)
{
    if (0 == memcmp(p_data, "RW:", 3))
    {
        reference_time_encode(&p_data[2], length-2);
        return;
    }

    else if(0 == memcmp(p_data, "CW:", 3))
    {
        current_time_encode(&p_data[2], length-2);
        return;
    }

    else if (0 == memcmp(p_data, "LW:", 3))
    {
        local_time_encode(&p_data[2], length-2);
        return;
    }
}

void reference_time_encode(uint8_t *p_data, uint16_t length)
{
    uint16_t data_length         =0;
    uint8_t  time_param_check_idx = 0;
    uint16_t time_param_check[CTS_SERIAL_NB]  = {0};

    while (data_length < length)
    {
        if (p_data[data_length] <= '9' && p_data[data_length] >= '0')
        {
            time_param_check[time_param_check_idx] = time_param_check[time_param_check_idx] * 10 + (p_data[data_length] - '0');
        }
        else if ('-' == p_data[data_length])
        {
            time_param_check_idx++;
        }

        data_length++;
    }

    if( 7>time_param_check[CTS_SERIAL_DATA_SOURCE]   &&
        255>time_param_check[CTS_SERIAL_DAY_SINCE]   &&
        24>time_param_check[CTS_SERIAL_HOURS_SINCE]  &&
        256>time_param_check[CTS_SERIAL_DATA_ACCURACY]
      )
     {
       s_cts_updata.stage_update_source                =  (cts_ref_time_source_t)time_param_check[CTS_SERIAL_DATA_SOURCE];
       s_cts_ref_info.ref_time_info.days_since_update  =  time_param_check[CTS_SERIAL_DAY_SINCE];
       s_cts_ref_info.ref_time_info.hours_since_update = time_param_check[CTS_SERIAL_HOURS_SINCE];
       s_cts_ref_info.ref_time_info.accuracy           = time_param_check[CTS_SERIAL_DATA_ACCURACY];
     }
     else
     {
        APP_LOG_INFO("Invalid set parameter.");
     }

     if(s_cts_updata.stage_update_source  != s_cts_env.cts_init.ref_time_info.source)
     {
       memcpy(&s_cts_record_time.cts_init.cur_time.day_date_time,&s_cts_env.cts_init.cur_time.day_date_time, sizeof(cts_cur_time_t));
       s_cts_updata.before_updata_sec = s_cts_record_time.cts_init.cur_time.day_date_time.date_time.sec;
       s_cts_updata.expected_time_1min_adapt = s_cts_record_time.cts_init.cur_time.day_date_time.date_time.min + 1;
       APP_LOG_INFO("From now on, wait 1 minute for the automatic update.");
       APP_LOG_INFO("The time source before modification:   SOURCE=%d  ",s_cts_ref_info.ref_time_info.source);
       if(s_cts_updata.expected_time_1min_adapt > 59)
       {
         s_cts_updata.expected_time_1min_adapt = 0 ;
       }
     }
    else
    {
       APP_LOG_INFO("The time source has not changed.");
    }
}

void local_time_encode(uint8_t *p_data, uint8_t length)
{
   ble_gatts_write_cfm_t *p_cfm;
   cts_evt_t p_evt;

   p_evt.length = length;
   p_evt.p_data = p_data;
   p_evt.length = local_time_universal_decode(p_cfm,&p_evt);

   if(p_evt.length)
    {
      APP_LOG_DEBUG("UART Change Local Time value\n");
      p_evt.evt_type = CTS_EVT_LOC_TIME_INFO_SET_BY_PEER;
      memcpy(&p_evt.loc_time_info,&s_cts_env.cts_init.loc_time_info, sizeof(cts_loc_time_info_t));
      s_cts_env.cts_init.evt_handler(&p_evt);
    }
}

void current_time_encode(uint8_t *p_data, uint16_t length)
{
   ble_gatts_write_cfm_t *p_cfm;
   cts_evt_t p_evt;

   p_evt.length = length;
   p_evt.p_data = p_data;

   if(s_cts_updata.updata_time_flag)
   {
     p_evt.length = current_time_universal_decode(p_cfm,&p_evt);
     if(p_evt.length)
     {
       p_evt.evt_type = CTS_EVT_CUR_TIME_SET_BY_PEER;
       p_evt.length   = length;
       memcpy(&p_evt.cur_time, &s_cts_env.cts_init.cur_time, sizeof(cts_cur_time_t));
       s_cts_env.cts_init.evt_handler(&p_evt);
     }

     s_cts_updata.expected_time_1min_adapt = 0;
     s_cts_updata.before_updata_sec = 0;
     s_cts_record_time.cts_init.cur_time.day_date_time.date_time.min =0;
     s_cts_record_time.cts_init.cur_time.day_date_time.date_time.sec =0;
   }

   else
   {
     APP_LOG_INFO("It is not updated for more than 1 minute.\n");
     APP_LOG_INFO("Updatable time minutes:  %d seconds:=  %d.",s_cts_updata.expected_time_1min_adapt,s_cts_updata.before_updata_sec);
   }
}

uint8_t current_time_universal_decode(ble_gatts_write_cfm_t *p_cfm, cts_evt_t *p_evt)
{
    uint8_t data_length         =0;
    uint8_t  time_param_check_idx = 0;
    uint16_t time_param_check[CTS_PEER_NB]  = {0};

    if(p_evt->p_data[0]==':')
    {
        data_length=1;
    }
    else
    {
        APP_LOG_INFO("Data format error.\r\n");
        p_cfm->status = CTS_ERROR_FIELDS_IGNORED;
        p_evt->length = 0 ;
        return p_evt->length;
    }

    while (data_length < p_evt->length)
    {
        if (p_evt->p_data[data_length] <= '9' && p_evt->p_data[data_length] >= '0')
        {
            time_param_check[time_param_check_idx] = time_param_check[time_param_check_idx] * 10 + (p_evt->p_data[data_length] - '0');
        }
        else if ('-' == p_evt->p_data[data_length])
        {
            time_param_check_idx++;
        }

        data_length++;
    }

    if( CTS_TIME_YEAR_VALID_VAL_MIN<=time_param_check[CTS_PEER_DATA_YEAR] &&
        CTS_TIME_YEAR_VALID_VAL_MAX>=time_param_check[CTS_PEER_DATA_YEAR] &&
        9>time_param_check[CTS_PEER_DATA_REASON] &&
        32>time_param_check[CTS_PEER_DATA_DAY] &&
        25>time_param_check[CTS_PEER_DATA_HOUR] &&
        60>time_param_check[CTS_PEER_DATA_MIN] &&
        60>time_param_check[CTS_PEER_DATA_SEC] &&
        8>time_param_check[CTS_PEER_DATA_WEEK]  &&
        256>time_param_check[CTS_PEER_DATA_FRACTION] &&
        13>time_param_check[CTS_PEER_DATA_MONTH]
      )
     {
        s_cts_env.cts_init.cur_time.adjust_reason                  = time_param_check[CTS_PEER_DATA_REASON];
        s_cts_env.cts_init.cur_time.day_date_time.day_of_week      = time_param_check[CTS_PEER_DATA_WEEK];
        s_cts_env.cts_init.cur_time.day_date_time.date_time.year   = time_param_check[CTS_PEER_DATA_YEAR];
        s_cts_env.cts_init.cur_time.day_date_time.date_time.month  = time_param_check[CTS_PEER_DATA_MONTH];
        s_cts_env.cts_init.cur_time.day_date_time.date_time.day    = time_param_check[CTS_PEER_DATA_DAY];
        s_cts_env.cts_init.cur_time.day_date_time.date_time.hour   = time_param_check[CTS_PEER_DATA_HOUR];
        s_cts_env.cts_init.cur_time.day_date_time.date_time.min    = time_param_check[CTS_PEER_DATA_MIN];
        s_cts_env.cts_init.cur_time.day_date_time.date_time.sec    = time_param_check[CTS_PEER_DATA_SEC];
        s_cts_env.cts_init.cur_time.day_date_time.fractions_256    = time_param_check[CTS_PEER_DATA_FRACTION];
     }
     else
     {
        APP_LOG_INFO("Invalid set parameter.");
        p_cfm->status = CTS_ERROR_FIELDS_IGNORED;
        p_evt->length = 0;
     }
     return p_evt->length;
}

uint8_t local_time_universal_decode(ble_gatts_write_cfm_t *p_cfm, cts_evt_t *p_evt)
{
    uint16_t  data_length         =0;
    uint8_t   time_param_check_idx = 0;
    int16_t   time_param_check[CTS_HOST_NB] = {0};
    uint8_t   flag=0;                                 //Determine if the start bit is -
    uint8_t   zone_set_same=1;                        //Determines whether the time zone entered is the same

    if(p_evt->p_data[0]==':')
    {
        data_length=1;
    }
    else
    {
        APP_LOG_INFO("Data format error.\r\n");
        p_cfm->status = CTS_ERROR_FIELDS_IGNORED;
        p_evt->length = 0 ;
        return p_evt->length;
    }

    while (data_length < p_evt->length)
    {
        if (p_evt->p_data[data_length] <= '9' && p_evt->p_data[data_length] >= '0')
        {
            time_param_check[time_param_check_idx] = time_param_check[time_param_check_idx] * 10 + (p_evt->p_data[data_length] - '0');
        }
        else if ('-' == p_evt->p_data[data_length] )
        {
            if (data_length<2)
            {
                flag=1;
            }
            else
            {
              time_param_check_idx++;
              if(flag)
                time_param_check[CTS_HOST_DATA_ZONE] = -time_param_check[CTS_HOST_DATA_ZONE];
            }
        }

        data_length++;
    }

    if(s_cts_env.cts_init.loc_time_info.time_zone  != time_param_check[CTS_HOST_DATA_ZONE])
    {
      s_cts_env.cts_init.cur_time.adjust_reason =4;
    }

    if(s_cts_env.cts_init.loc_time_info.time_zone  == (int16_t)time_param_check[CTS_HOST_DATA_ZONE] &&
       s_cts_env.cts_init.loc_time_info.dst_offset == (cts_dst_offset_t)time_param_check[CTS_HOST_DATA_OFFST]
      )
    {
        zone_set_same = 0;
        APP_LOG_INFO("Enter the same time zone.");
    }

    if(time_param_check[CTS_HOST_DATA_ZONE]== -128 )
    {
       s_cts_env.cts_init.loc_time_info.time_zone  = time_param_check[CTS_HOST_DATA_ZONE];
    }

    else if( CTS_TIME_ZONE_OFFSET_MIN <= (int16_t)time_param_check[CTS_HOST_DATA_ZONE] &&
             CTS_TIME_ZONE_OFFSET_MAX >= (int16_t)time_param_check[CTS_HOST_DATA_ZONE] &&
             zone_set_same
            )
    {
       s_cts_env.cts_init.loc_time_info.time_zone  = time_param_check[CTS_HOST_DATA_ZONE];
    }

    else
    {
       APP_LOG_INFO("Invalid set parameter.");
       p_cfm->status = CTS_ERROR_FIELDS_IGNORED;
       p_evt->length = 0 ;
    }

    if(s_cts_env.cts_init.loc_time_info.dst_offset  != (cts_dst_offset_t)time_param_check[CTS_HOST_DATA_OFFST])
    {
      s_cts_env.cts_init.cur_time.adjust_reason =8;
    }

    if( time_param_check[CTS_HOST_DATA_OFFST] == 255 )
    {
       s_cts_env.cts_init.loc_time_info.dst_offset = (cts_dst_offset_t)time_param_check[CTS_HOST_DATA_OFFST];
    }

    else if( CTS_DST_OFFSET_DOUB_DAYLIGHT_TIME >= (cts_dst_offset_t)time_param_check[CTS_HOST_DATA_OFFST]  &&
             time_param_check[CTS_HOST_DATA_OFFST]%2 ==0  &&
             time_param_check[CTS_HOST_DATA_OFFST]  != 6  &&
             zone_set_same
    )
    {
      s_cts_env.cts_init.loc_time_info.dst_offset = (cts_dst_offset_t)time_param_check[CTS_HOST_DATA_OFFST];
    }

    else
    {
      APP_LOG_INFO("Invalid set parameter.\r\n");
      p_cfm->status = CTS_ERROR_FIELDS_IGNORED;
      p_evt->length = 0 ;
    }

    return p_evt->length;
}

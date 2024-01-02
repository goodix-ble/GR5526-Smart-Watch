/**
 ****************************************************************************************
 *
 * @file lns.c
 *
 * @brief Log Notification Service implementation.
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
#include "lns.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include <string.h>

/*
* DEFINES
*****************************************************************************************
*/
#define LNS_DEFAULT_GATT_PAYLOAD                20
#define LNS_LOG_INFO_CHARACTERISTIC_UUID        {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                                 0x0A, 0x46, 0x44, 0xD3, 0x02, 0x08, 0xED, 0xA6}
#define LNS_LOG_CTRL_PT_CHARACTERISTIC_UUID     {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                                 0x0A, 0x46, 0x44, 0xD3, 0x03, 0x08, 0xED, 0xA6}

/**@brief Macros for conversion of 128bit to 16bit UUID. */
#define ATT_128_PRIMARY_SERVICE     BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_PRIMARY_SERVICE)
#define ATT_128_CHARACTERISTIC      BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC)
#define ATT_128_CLIENT_CHAR_CFG     BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DESC_CLIENT_CHAR_CFG)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/**@brief Log Notification Service Attributes Indexes. */
enum
{
    // Log Notification Service
    LNS_IDX_SVC,

    // Log Information
    LNS_IDX_LOG_INFO_CHAR,
    LNS_IDX_LOG_INFO_VAL,
    LNS_IDX_LOG_INFO_NTF_CFG,

    // Log Control Point
    LNS_IDX_LOG_CTRL_PT_CHAR,
    LNS_IDX_LOG_CTRL_PT_VAL,
    LNS_IDX_LOG_CTRL_PT_IND_CFG,

    LNS_IDX_NB
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Log Notification Service environment variable. */
struct lns_env_t
{
    lns_evt_handler_t     evt_handler;                              /**< Log Notification Service event handler. */
    uint16_t              start_hdl;                                /**< Log Notification Service start handle. */
    uint16_t              payload_len;                              /**< Length of gatt payload. */
    uint16_t              log_info_ntf_cfg[LNS_CONNECTION_MAX];     /**< The configuration of Log Information Notification which is configured by the peer devices. */
    uint16_t              log_ctrl_pt_ind_cfg[LNS_CONNECTION_MAX];  /**< The configuration of Log Control Point indication which is configured by the peer devices. */
    ble_gatts_create_db_t lns_gatts_db;                         /**< Log Notification Service attributs database. */
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static sdk_err_t lns_log_info_chunk(uint8_t conn_idx);
static void      lns_evt_handler(lns_evt_t *p_evt);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct lns_env_t s_lns_env;
static lns_log_data_t   s_log_data_ntf;
static uint16_t         s_lns_char_mask  = 0x007f;
static const uint8_t    s_lns_svc_uuid[] = {LNS_SERVICE_UUID};

/**@brief Full LNS Database Description - Used to add attributes into the database. */
static const ble_gatts_attm_desc_128_t lns_attr_tab[LNS_IDX_NB] =
{
    // Log Notification Service Declaration
    [LNS_IDX_SVC]              = {ATT_128_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // Log Information Characteristic - Declaration
    [LNS_IDX_LOG_INFO_CHAR]    = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Log Information Characteristic - Value
    [LNS_IDX_LOG_INFO_VAL]     = {LNS_LOG_INFO_CHARACTERISTIC_UUID,
                                  BLE_GATTS_NOTIFY_PERM_UNSEC,
                                  BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128),
                                  LNS_LOG_INFO_VAL_LEN},
    // Log Information Characteristic - Client Characteristic Configuration Descriptor
    [LNS_IDX_LOG_INFO_NTF_CFG] = {ATT_128_CLIENT_CHAR_CFG, BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC, 0, 0},

    // Log Control Point Characteristic - Declaration
    [LNS_IDX_LOG_CTRL_PT_CHAR]    = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Log Control Point Characteristic - Value
    [LNS_IDX_LOG_CTRL_PT_VAL]     = {LNS_LOG_CTRL_PT_CHARACTERISTIC_UUID,
                                     BLE_GATTS_INDICATE_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                     BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128),
                                     LNS_LOG_CTRL_PT_VAL_LEN},
    // Log Control Point Characteristic - Client Characteristic Configuration Descriptor
    [LNS_IDX_LOG_CTRL_PT_IND_CFG] = {ATT_128_CLIENT_CHAR_CFG, BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC, 0, 0},
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
static void lns_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t  cfm;
    uint8_t           handle    = p_param->handle;
    uint8_t           tab_index = prf_find_idx_by_handle(handle,
                                  s_lns_env.start_hdl,
                                  LNS_IDX_NB,
                                  (uint8_t *)&s_lns_char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case LNS_IDX_LOG_INFO_NTF_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_lns_env.log_info_ntf_cfg[conn_idx];
            break;

        case LNS_IDX_LOG_CTRL_PT_IND_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_lns_env.log_ctrl_pt_ind_cfg[conn_idx];
            break;

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
static void lns_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t          handle      = p_param->handle;
    uint16_t          tab_index   = 0;
    uint16_t          cccd_value  = 0;
    lns_evt_t         event;
    ble_gatts_write_cfm_t cfm;

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_lns_env.start_hdl,
                                        LNS_IDX_NB,
                                        (uint8_t *)&s_lns_char_mask);
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    event.evt_type = LNS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case LNS_IDX_LOG_INFO_NTF_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              LNS_EVT_LOG_INFO_NTF_ENABLE : \
                              LNS_EVT_LOG_INFO_NTF_DISABLE);
            s_lns_env.log_info_ntf_cfg[conn_idx] = cccd_value;
            break;

        case LNS_IDX_LOG_CTRL_PT_IND_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ? \
                              LNS_EVT_CTRL_PT_IND_ENABLE : \
                              LNS_EVT_CTRL_PT_IND_DISABLE);
            s_lns_env.log_ctrl_pt_ind_cfg[conn_idx] = cccd_value;
            break;

        case LNS_IDX_LOG_CTRL_PT_VAL:
        {
            switch (p_param->value[0])
            {
                case LNS_CTRL_PT_TRACE_STATUS_GET:
                    event.evt_type = LNS_EVT_TRACE_STATUS_GET;
                    break;

                case LNS_CTRL_PT_TRACE_INFO_DUMP:
                    event.evt_type = LNS_EVT_TRACE_INFO_DUMP;
                    break;

                case LNS_CTRL_PT_TRACE_INFO_CLEAR:
                    event.evt_type = LNS_EVT_TRACE_INFO_CLEAR;
                    break;

                default:
                    break;
            }
        }
        break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_write_cfm(conn_idx, &cfm);

    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && LNS_EVT_INVALID != event.evt_type)
    {
        lns_evt_handler(&event);
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
static void lns_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint16_t          tab_index   = 0;
    lns_evt_t        event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_lns_env.start_hdl,
                                        LNS_IDX_NB,
                                        (uint8_t *)&s_lns_char_mask);

    event.evt_type = LNS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case LNS_IDX_LOG_INFO_NTF_CFG:
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              LNS_EVT_LOG_INFO_NTF_ENABLE : \
                              LNS_EVT_LOG_INFO_NTF_DISABLE);
            s_lns_env.log_info_ntf_cfg[conn_idx] = cccd_value;
            break;

        case LNS_IDX_LOG_CTRL_PT_IND_CFG:
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ? \
                              LNS_EVT_CTRL_PT_IND_ENABLE : \
                              LNS_EVT_CTRL_PT_IND_DISABLE);
            s_lns_env.log_ctrl_pt_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }

    if (LNS_EVT_INVALID != event.evt_type && s_lns_env.evt_handler)
    {
        s_lns_env.evt_handler(&event);
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
static void lns_ntf_ind_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    uint8_t tab_index;

    tab_index = prf_find_idx_by_handle(p_ntf_ind->handle,
                                       s_lns_env.start_hdl,
                                       LNS_IDX_NB,
                                       (uint8_t *)&s_lns_char_mask);

    if (LNS_IDX_LOG_INFO_VAL == tab_index)
    {
        if (BLE_SUCCESS == status)
        {
            lns_log_info_chunk(conn_idx);
        }
        else
        {
            s_log_data_ntf.length = 0;
            s_log_data_ntf.offset = 0;

            sys_free(s_log_data_ntf.p_data);
        }
    }
}

/**
 *****************************************************************************************
 * @brief Handles lns event.
 *
 * @param[in] p_evt:  Pointer to event.
 *****************************************************************************************
 */
static void lns_evt_handler(lns_evt_t *p_evt)
{
    uint8_t trace_log_num  = 0;

    switch (p_evt->evt_type)
    {
        case LNS_EVT_TRACE_STATUS_GET:
            trace_log_num = fault_db_records_num_get();
            lns_log_status_send(p_evt->conn_idx, trace_log_num);
            break;

        case LNS_EVT_TRACE_INFO_DUMP:
            lns_log_info_send(p_evt->conn_idx);
            break;

        case LNS_EVT_TRACE_INFO_CLEAR:
            fault_db_record_clear();
            break;
    }

    if (LNS_EVT_INVALID != p_evt->evt_type && s_lns_env.evt_handler)
    {
        s_lns_env.evt_handler(p_evt);
    }
}

/**
 *****************************************************************************************
 * @brief Handle Log Information data notify.
 *
 * @param[in] conn_idx: Connection index.
 *
 * @return Result of handle.
 *****************************************************************************************
 */
static sdk_err_t lns_log_info_chunk(uint8_t conn_idx)
{
    uint16_t         chunk_len = 0;
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    ble_gatts_noti_ind_t data_ntf;

    chunk_len = s_log_data_ntf.length - s_log_data_ntf.offset;
    chunk_len = chunk_len > s_lns_env.payload_len  ? s_lns_env.payload_len  : chunk_len;

    if (0 == chunk_len)
    {
        s_log_data_ntf.length = 0;
        s_log_data_ntf.offset = 0;

        sys_free(s_log_data_ntf.p_data);

        return SDK_SUCCESS;
    }

    if (PRF_CLI_START_NTF == s_lns_env.log_info_ntf_cfg[conn_idx])
    {
        data_ntf.type   = BLE_GATT_NOTIFICATION;
        data_ntf.handle = prf_find_handle_by_idx(LNS_IDX_LOG_INFO_VAL,
                                                 s_lns_env.start_hdl,
                                                 (uint8_t *)&s_lns_char_mask);
        data_ntf.length = chunk_len;
        data_ntf.value  = (uint8_t *)s_log_data_ntf.p_data + s_log_data_ntf.offset;

        error_code = ble_gatts_noti_ind(conn_idx, &data_ntf);
    }

    if (SDK_SUCCESS == error_code)
    {
        s_log_data_ntf.offset += chunk_len;
    }

    return error_code;
}

static void lns_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            lns_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            lns_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_NTF_IND:
            lns_ntf_ind_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            lns_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t lns_service_init(lns_evt_handler_t evt_handler)
{
    s_lns_env.evt_handler = evt_handler;
    s_lns_env.payload_len = LNS_DEFAULT_GATT_PAYLOAD;
    s_lns_env.start_hdl   = PRF_INVALID_HANDLE;

    s_lns_env.lns_gatts_db.shdl                  = &s_lns_env.start_hdl;
    s_lns_env.lns_gatts_db.uuid                  = s_lns_svc_uuid;
    s_lns_env.lns_gatts_db.attr_tab_cfg          = (uint8_t *)&s_lns_char_mask;
    s_lns_env.lns_gatts_db.max_nb_attr           = LNS_IDX_NB;
    s_lns_env.lns_gatts_db.srvc_perm             = BLE_GATTS_SRVC_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128); 
    s_lns_env.lns_gatts_db.attr_tab_type         = BLE_GATTS_SERVICE_TABLE_TYPE_128;
    s_lns_env.lns_gatts_db.attr_tab.attr_tab_128 = lns_attr_tab;

    return ble_gatts_prf_add(&s_lns_env.lns_gatts_db, lns_ble_evt_handler);
}

sdk_err_t lns_log_info_send(uint8_t conn_idx)
{
    s_log_data_ntf.offset = 0;
    s_log_data_ntf.length = fault_db_records_total_len_get();
    s_log_data_ntf.p_data = sys_malloc(s_log_data_ntf.length);

    if (s_log_data_ntf.p_data)
    {
        fault_db_records_dump(s_log_data_ntf.p_data, &s_log_data_ntf.length);
    }
    else
    {
        return SDK_ERR_NO_RESOURCES;
    }

    return lns_log_info_chunk(conn_idx);
}

sdk_err_t lns_log_status_send(uint8_t conn_idx, const uint8_t log_num)
{
    sdk_err_t        error_code = SDK_ERR_IND_DISABLED;
    ble_gatts_noti_ind_t status_ind;

    if (PRF_CLI_START_IND == s_lns_env.log_ctrl_pt_ind_cfg[conn_idx])
    {
        status_ind.type   = BLE_GATT_INDICATION;
        status_ind.handle = prf_find_handle_by_idx(LNS_IDX_LOG_CTRL_PT_VAL,
                                                s_lns_env.start_hdl,
                                                (uint8_t *)&s_lns_char_mask);
        status_ind.length = LNS_LOG_CTRL_PT_VAL_LEN;
        status_ind.value  = (uint8_t *)&log_num;

        error_code  = ble_gatts_noti_ind(conn_idx, &status_ind);
    }

    return error_code;
}

sdk_err_t lns_pay_load_update(uint8_t conn_idx, const uint16_t payload_len)
{
    if (s_lns_env.payload_len > LNS_LOG_INFO_VAL_LEN)
    {
        s_lns_env.payload_len = LNS_LOG_INFO_VAL_LEN;
    }
    else
    {
        s_lns_env.payload_len = payload_len;
    }

    return SDK_SUCCESS;
}


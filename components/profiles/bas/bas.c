/**
 *****************************************************************************************
 *
 * @file bas.c
 *
 * @brief Battery Server Implementation.
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
#include "bas.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief Battery Service Attributes Indexes. */
enum bas_attr_idx_t
{
    BAS_IDX_SVC,

    BAS_IDX_BATT_LVL_CHAR,
    BAS_IDX_BATT_LVL_VAL,
    BAS_IDX_BATT_LVL_NTF_CFG,
    BAS_IDX_BATT_LVL_PRES_FMT,

    BAS_IDX_NB,
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Battery Service environment variable. */
struct bas_env_t
{
    bas_init_t              bas_init;                       /**< Battery Service initialization variables. */
    uint16_t                start_hdl;                      /**< Battery Service start handle. */
    uint16_t                ntf_cfg[BAS_CONNECTION_MAX];    /**< The configuration of Battery Level Notification which is configured by the peer devices. */
    prf_char_pres_fmt_t     batt_level_pres_format;         /**< Battery Level Characteristic Presentation Format which should not change during connection. */
    ble_gatts_create_db_t   bas_gatts_db;                   /**< Battery Service attributs database. */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct bas_env_t s_bas_env[BAS_INSTANCE_MAX];  /**< Battery service instance. */
static uint8_t          s_bas_ins_cnt = 0;            /**< Number of Battery Server task instances. */
static const uint8_t    s_bas_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_BATTERY_SERVICE);

/**@brief Full BAS Database Description which is used to add attributes into the ATT database. */
static const ble_gatts_attm_desc_t bas_attr_tab[BAS_IDX_NB] =
{
    // Battery Service Declaration
    [BAS_IDX_SVC] = {BLE_ATT_DECL_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // Battery Level Characteristic - Declaration
    [BAS_IDX_BATT_LVL_CHAR]     = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Battery Level Characteristic - Value
    [BAS_IDX_BATT_LVL_VAL]      = {BLE_ATT_CHAR_BATTERY_LEVEL,
                                   BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_NOTIFY_PERM_UNSEC,
                                   BLE_GATTS_ATT_VAL_LOC_USER,
                                   BAS_LVL_MAX_LEN},
    // Battery Level Characteristic - Client Characteristic Configuration Descriptor
    [BAS_IDX_BATT_LVL_NTF_CFG]  = {BLE_ATT_DESC_CLIENT_CHAR_CFG, 
                                   BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                   BLE_GATTS_ATT_VAL_LOC_USER,
                                   0},
    // Battery Level Characteristic - Characteristic Presentation Format Descriptor
    [BAS_IDX_BATT_LVL_PRES_FMT] = {BLE_ATT_DESC_CHAR_PRES_FORMAT, BLE_GATTS_READ_PERM_UNSEC, BLE_GATTS_ATT_VAL_LOC_USER, 0},
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_param:  Point to the parameters of the read request.
 *****************************************************************************************
 */
static void bas_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    uint8_t              handle = p_param->handle;
    uint8_t              tab_index;
    uint8_t              char_pres_value[PRF_CHAR_PRES_FMT_SIZE];
    uint8_t              i;
    ble_gatts_read_cfm_t cfm;

    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    for (i = 0; i < s_bas_ins_cnt; i++)
    {
        tab_index = prf_find_idx_by_handle(handle,
                                           s_bas_env[i].start_hdl,
                                           BAS_IDX_NB,
                                           &s_bas_env[i].bas_init.char_mask);

        if (tab_index > 0)
        {
            break;
        }
    }

    switch (tab_index)
    {
        case BAS_IDX_BATT_LVL_VAL:
            cfm.length = sizeof(uint8_t);
            cfm.value  = (uint8_t *)(&s_bas_env[i].bas_init.batt_lvl);
            break;

        case BAS_IDX_BATT_LVL_NTF_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)(&(s_bas_env[i].ntf_cfg[conn_idx]));
            break;

        case BAS_IDX_BATT_LVL_PRES_FMT:
            cfm.length = PRF_CHAR_PRES_FMT_SIZE;
            prf_pack_char_pres_fmt(char_pres_value, &(s_bas_env[i].batt_level_pres_format));
            cfm.value = char_pres_value;
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
 * @param[in] conn_idx  Connection index.
 * @param[in] p_param Point to the parameters of the write request.
 *****************************************************************************************
 */
static void bas_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t                handle     = p_param->handle;
    uint16_t                cccd_value = 0;
    uint8_t                 tab_index  = 0;
    uint8_t                 i          = 0;
    bas_evt_t               bas_evt;
    ble_gatts_write_cfm_t   cfm;

    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    for (i = 0; i < s_bas_ins_cnt; i++)
    {
        tab_index = prf_find_idx_by_handle(handle,
                                           s_bas_env[i].start_hdl,
                                           BAS_IDX_NB,
                                           &s_bas_env[i].bas_init.char_mask);

        if (tab_index > 0)
        {
            break;
        }
    }

    switch (tab_index)
    {
        case BAS_IDX_BATT_LVL_NTF_CFG:
            cccd_value = le16toh(&p_param->value[0]);
            s_bas_env[i].ntf_cfg[conn_idx] = cccd_value;
            bas_evt.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                                 BAS_EVT_NOTIFICATION_ENABLED : \
                                 BAS_EVT_NOTIFICATION_DISABLED);
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && \
        BAS_EVT_INVALID != bas_evt.evt_type && \
        s_bas_env[i].bas_init.evt_handler)
    {
        bas_evt.conn_idx = conn_idx;
        s_bas_env[i].bas_init.evt_handler(&bas_evt);
    }

    ble_gatts_write_cfm(conn_idx, &cfm);
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
static void bas_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint8_t           tab_index  = 0;
    uint8_t           i          = 0;
    bas_evt_t         bas_evt;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    for (i = 0; i < s_bas_ins_cnt; i++)
    {
        tab_index = prf_find_idx_by_handle(handle,
                                           s_bas_env[i].start_hdl,
                                           BAS_IDX_NB,
                                           &s_bas_env[i].bas_init.char_mask);

        if (tab_index > 0)
        {
            break;
        }
    }

    switch (tab_index)
    {
        case BAS_IDX_BATT_LVL_NTF_CFG:
            s_bas_env[i].ntf_cfg[conn_idx] = cccd_value;
            bas_evt.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                                 BAS_EVT_NOTIFICATION_ENABLED : \
                                 BAS_EVT_NOTIFICATION_DISABLED);
            break;

        default:
            bas_evt.evt_type = BAS_EVT_INVALID;
            break;
    }

    if (BAS_EVT_INVALID != bas_evt.evt_type && s_bas_env[i].bas_init.evt_handler)
    {
        bas_evt.conn_idx = conn_idx;
        s_bas_env[i].bas_init.evt_handler(&bas_evt);
    }
}

static void bas_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            bas_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            bas_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            bas_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
sdk_err_t bas_batt_lvl_update(uint8_t conn_idx, uint8_t ins_idx, uint8_t batt_lvl)
{
    sdk_err_t               error_code = SDK_ERR_NTF_DISABLED;
    ble_gatts_noti_ind_t    send_cmd;

    if (ins_idx <= s_bas_ins_cnt)
    {
        s_bas_env[ins_idx].bas_init.batt_lvl = batt_lvl;

        if (PRF_CLI_START_NTF == s_bas_env[ins_idx].ntf_cfg[conn_idx])
        {
            // Fill in the parameter structure
            send_cmd.type   = BLE_GATT_NOTIFICATION;
            send_cmd.handle = prf_find_handle_by_idx(BAS_IDX_BATT_LVL_VAL,
                                                     s_bas_env[ins_idx].start_hdl,
                                                     &s_bas_env[ins_idx].bas_init.char_mask);
            // pack measured value in database
            send_cmd.length = sizeof(uint8_t);
            send_cmd.value  = &s_bas_env[ins_idx].bas_init.batt_lvl;

            // send notification to peer device
            error_code = ble_gatts_noti_ind(conn_idx, &send_cmd);
        }
    }

    return error_code;
}

sdk_err_t bas_service_init(bas_init_t *p_bas_init, uint8_t ins_num)
{
    sdk_err_t   error_code = SDK_SUCCESS;

    if (NULL == p_bas_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    if (ins_num > BAS_INSTANCE_MAX)
    {
        return SDK_ERR_INVALID_PARAM;
    }

    for (uint8_t i = 0; i < ins_num; i++)
    {
        memcpy(&s_bas_env[i].bas_init, &p_bas_init[i], sizeof(bas_init_t));
        s_bas_env[i].start_hdl = PRF_INVALID_HANDLE;
        s_bas_env[i].bas_gatts_db.shdl                  = &s_bas_env[i].start_hdl;
        s_bas_env[i].bas_gatts_db.uuid                  = s_bas_svc_uuid;
        s_bas_env[i].bas_gatts_db.attr_tab_cfg          = (uint8_t *)&s_bas_env[i].bas_init.char_mask;
        s_bas_env[i].bas_gatts_db.max_nb_attr           = BAS_IDX_NB;
        s_bas_env[i].bas_gatts_db.srvc_perm             = 0;
        s_bas_env[i].bas_gatts_db.attr_tab_type         = BLE_GATTS_SERVICE_TABLE_TYPE_16;
        s_bas_env[i].bas_gatts_db.attr_tab.attr_tab_16 = bas_attr_tab;
        error_code = ble_gatts_prf_add(&s_bas_env[i].bas_gatts_db, bas_ble_evt_handler);
        if(error_code)
        {
            return error_code;
        }
    }

    s_bas_ins_cnt = ins_num;

    return error_code;
}

uint16_t bas_service_start_handle_get(void)
{
    return s_bas_env[0].start_hdl;
}

/**
 ****************************************************************************************
 *
 * @file pass.c
 *
 * @brief Phone Alert Status Service implementation.
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
#include "pass.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/**@brief Phone Alert Status Service Attributes Indexes. */
enum
{
    // Phone Alert Status Service
    PASS_IDX_SVC,

    // Alert Status
    PASS_IDX_ALERT_STATUS_CHAR,
    PASS_IDX_ALERT_STATUS_VAL,
    PASS_IDX_ALERT_STATUS_NTF_CFG,

    // Ringer Setting
    PASS_IDX_RINGER_SET_CHAR,
    PASS_IDX_RINGER_SET_VAL,
    PASS_IDX_RINGER_SET_NTF_CFG,

    // Ringer Control Point
    PASS_IDX_RINGER_CTRL_PT_CHAR,
    PASS_IDX_RINGER_CTRL_PT_VAL,

    PASS_IDX_NB
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Phone Alert Status Service environment variable. */
struct pass_env_t
{
    pass_init_t             pass_init;                                     /**< Phone Alert Status Service initialization variables. */
    uint16_t                start_hdl;                                     /**< Phone Alert Status Service start handle. */
    uint16_t                alert_status_ntf_cfg[PASS_CONNECTION_MAX];     /**< The configuration of Alert Status Notification which is configured by the peer devices. */
    uint16_t                ringer_setting_ntf_cfg[PASS_CONNECTION_MAX];   /**< The configuration of Ringer Setting Notification which is configured by the peer devices. */
    ble_gatts_create_db_t   pass_gatts_db;                                 /**< Running Speed and Cadence Service attributs database. */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct pass_env_t s_pass_env;
static const uint8_t     s_pass_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_PHONE_ALERT_STATUS);

/**@brief Full PASS Database Description - Used to add attributes into the database. */
static const ble_gatts_attm_desc_t pass_attr_tab[PASS_IDX_NB] =
{
    // Phone Alert Status Service Declaration
    [PASS_IDX_SVC]              = {BLE_ATT_DECL_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // Alert Status Characteristic - Declaration
    [PASS_IDX_ALERT_STATUS_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Alert Status Characteristic - Value
    [PASS_IDX_ALERT_STATUS_VAL]     = {BLE_ATT_CHAR_ALERT_STATUS,
                                       BLE_GATTS_NOTIFY_PERM_UNSEC | BLE_GATTS_READ_PERM_UNSEC,
                                       BLE_GATTS_ATT_VAL_LOC_USER,
                                       PASS_ALERT_STATUS_VAL_LEN},
    // Alert Status Characteristic - Client Characteristic Configuration Descriptor
    [PASS_IDX_ALERT_STATUS_NTF_CFG] = {BLE_ATT_DESC_CLIENT_CHAR_CFG, BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC, 0, 0},

    // Ringer Setting Characteristic - Declaration
    [PASS_IDX_RINGER_SET_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Ringer Setting Characteristic - Value
    [PASS_IDX_RINGER_SET_VAL]     = {BLE_ATT_CHAR_RINGER_SETTING,
                                     BLE_GATTS_NOTIFY_PERM_UNSEC | BLE_GATTS_READ_PERM_UNSEC,
                                     BLE_GATTS_ATT_VAL_LOC_USER,
                                     PASS_RINGER_SET_VAL_LEN},
    // Ringer Setting Characteristic - Client Characteristic Configuration Descriptor
    [PASS_IDX_RINGER_SET_NTF_CFG] = {BLE_ATT_DESC_CLIENT_CHAR_CFG, BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC, 0, 0},

    // Ringer Setting Characteristic - Declaration
    [PASS_IDX_RINGER_CTRL_PT_CHAR]  = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Ringer Setting Characteristic - Value
    [PASS_IDX_RINGER_CTRL_PT_VAL]   = {BLE_ATT_CHAR_RINGER_CNTL_POINT,
                                       BLE_GATTS_WRITE_CMD_PERM_UNSEC,
                                       BLE_GATTS_ATT_VAL_LOC_USER,
                                       PASS_RINGER_CTRL_PT_VAL_LEN},
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
static void pass_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t  cfm;
    uint8_t           handle    = p_param->handle;
    uint8_t           tab_index = prf_find_idx_by_handle(handle,
                                  s_pass_env.start_hdl,
                                  PASS_IDX_NB,
                                  (uint8_t *)&s_pass_env.pass_init.char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case PASS_IDX_ALERT_STATUS_VAL:
            cfm.length = PASS_ALERT_STATUS_VAL_LEN;
            cfm.value  = &s_pass_env.pass_init.alert_status;
            break;

        case PASS_IDX_ALERT_STATUS_NTF_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_pass_env.alert_status_ntf_cfg[conn_idx];
            break;

        case PASS_IDX_RINGER_SET_VAL:
            cfm.length = PASS_RINGER_SET_VAL_LEN;
            cfm.value  = &s_pass_env.pass_init.ringer_setting;
            break;

        case PASS_IDX_RINGER_SET_NTF_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_pass_env.ringer_setting_ntf_cfg[conn_idx];
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
static void pass_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t          handle      = p_param->handle;
    uint16_t          tab_index   = 0;
    uint16_t          cccd_value  = 0;
    pass_evt_t        event;
    ble_gatts_write_cfm_t cfm;

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_pass_env.start_hdl,
                                        PASS_IDX_NB,
                                        (uint8_t *)&s_pass_env.pass_init.char_mask);
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    event.evt_type = PASS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case PASS_IDX_ALERT_STATUS_NTF_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              PASS_EVT_ALERT_STATUS_NTF_ENABLE : \
                              PASS_EVT_ALERT_STATUS_NTF_DISABLE);
            s_pass_env.alert_status_ntf_cfg[conn_idx] = cccd_value;
            break;

        case PASS_IDX_RINGER_SET_NTF_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              PASS_EVT_RINGER_SET_NTF_ENABLE : \
                              PASS_EVT_RINGER_SET_NTF_DISABLE);
            s_pass_env.ringer_setting_ntf_cfg[conn_idx] = cccd_value;
            break;

        case PASS_IDX_RINGER_CTRL_PT_VAL:
        {
            switch (p_param->value[0])
            {
                case PASS_CTRL_PT_SILENT_MODE:
                    if ((PASS_RINGER_SET_NORMAL == s_pass_env.pass_init.ringer_setting) && \
                            (PASS_RINGER_ACTIVE & s_pass_env.pass_init.alert_status))
                    {
                        event.evt_type = PASS_EVT_SILENT_MODE_SET;
                    }
                    break;

                case PASS_CTRL_PT_MUTE_ONCE:
                    event.evt_type = PASS_EVT_MUTE_ONCE_SET;
                    break;

                case PASS_CTRL_PT_CANCEL_SLIENT_MODE:
                    if ((PASS_RINGER_SET_SILENT == s_pass_env.pass_init.ringer_setting) && \
                            (PASS_RINGER_ACTIVE & s_pass_env.pass_init.alert_status))
                    {
                        event.evt_type = PASS_EVT_SILENT_MODE_CANCEL;
                    }
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

    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && PASS_EVT_INVALID != event.evt_type && s_pass_env.pass_init.evt_handler)
    {
        s_pass_env.pass_init.evt_handler(&event);
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
static void pass_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint16_t          tab_index   = 0;
    pass_evt_t        event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_pass_env.start_hdl,
                                        PASS_IDX_NB,
                                        (uint8_t *)&s_pass_env.pass_init.char_mask);

    event.evt_type = PASS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case PASS_IDX_ALERT_STATUS_NTF_CFG:
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              PASS_EVT_ALERT_STATUS_NTF_ENABLE : \
                              PASS_EVT_ALERT_STATUS_NTF_DISABLE);
            s_pass_env.alert_status_ntf_cfg[conn_idx] = cccd_value;
            break;

        case PASS_IDX_RINGER_SET_NTF_CFG:
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              PASS_EVT_RINGER_SET_NTF_ENABLE : \
                              PASS_EVT_RINGER_SET_NTF_DISABLE);
            s_pass_env.ringer_setting_ntf_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }

    if (PASS_EVT_INVALID != event.evt_type && s_pass_env.pass_init.evt_handler)
    {
        s_pass_env.pass_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Notify Alert Status.

 * @param[in] conn_idx: Connnection index.
 *****************************************************************************************
 */
static sdk_err_t pass_alert_status_send(uint8_t conn_idx)
{
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    ble_gatts_noti_ind_t alert_status_ntf;

    if (PRF_CLI_START_NTF == s_pass_env.alert_status_ntf_cfg[conn_idx])
    {
        alert_status_ntf.type   = BLE_GATT_NOTIFICATION;
        alert_status_ntf.handle = prf_find_handle_by_idx(PASS_IDX_ALERT_STATUS_VAL,
                                  s_pass_env.start_hdl,
                                  (uint8_t *)&s_pass_env.pass_init.char_mask);
        alert_status_ntf.length = PASS_ALERT_STATUS_VAL_LEN;
        alert_status_ntf.value  = &s_pass_env.pass_init.alert_status;
        error_code              = ble_gatts_noti_ind(conn_idx, &alert_status_ntf);
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Notify Ringer Setting.
 *
 * @param[in] conn_idx: Connnection index.
 *****************************************************************************************
 */
sdk_err_t pass_ringer_set_send(uint8_t conn_idx)
{
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    ble_gatts_noti_ind_t ringer_set_ntf;

    if (PRF_CLI_START_NTF == s_pass_env.ringer_setting_ntf_cfg[conn_idx])
    {
        ringer_set_ntf.type   = BLE_GATT_NOTIFICATION;
        ringer_set_ntf.handle = prf_find_handle_by_idx(PASS_IDX_RINGER_SET_VAL,
                                s_pass_env.start_hdl,
                                (uint8_t *)&s_pass_env.pass_init.char_mask);
        ringer_set_ntf.length = PASS_ALERT_STATUS_VAL_LEN;
        ringer_set_ntf.value  = &s_pass_env.pass_init.ringer_setting;
        error_code            = ble_gatts_noti_ind(conn_idx, &ringer_set_ntf);
    }

    return error_code;
}

static void pass_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            pass_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            pass_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            pass_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
uint8_t pass_ringer_setting_get(void)
{
    return s_pass_env.pass_init.ringer_setting;
}

void pass_alert_status_set(uint8_t conn_idx, uint8_t new_status)
{
    if (new_status != s_pass_env.pass_init.alert_status)
    {
        s_pass_env.pass_init.alert_status = new_status;
        pass_alert_status_send(conn_idx);
    }
}

void pass_ringer_setting_set(uint8_t conn_idx, uint8_t new_setting)
{
    if (new_setting != s_pass_env.pass_init.ringer_setting)
    {
        s_pass_env.pass_init.ringer_setting = new_setting;
        pass_ringer_set_send(conn_idx);
    }
}

sdk_err_t pass_service_init(pass_init_t *p_pass_init)
{
    if (NULL == p_pass_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memcpy(&s_pass_env.pass_init, p_pass_init, sizeof(pass_init_t));

    s_pass_env.start_hdl  = PRF_INVALID_HANDLE;

    s_pass_env.pass_gatts_db.shdl                 = &s_pass_env.start_hdl;
    s_pass_env.pass_gatts_db.uuid                 = s_pass_svc_uuid;
    s_pass_env.pass_gatts_db.attr_tab_cfg         = (uint8_t *)&(s_pass_env.pass_init.char_mask);
    s_pass_env.pass_gatts_db.max_nb_attr          = PASS_IDX_NB;
    s_pass_env.pass_gatts_db.srvc_perm            = 0; 
    s_pass_env.pass_gatts_db.attr_tab_type        = BLE_GATTS_SERVICE_TABLE_TYPE_16;
    s_pass_env.pass_gatts_db.attr_tab.attr_tab_16 = pass_attr_tab;

    return ble_gatts_prf_add(&s_pass_env.pass_gatts_db, pass_ble_evt_handler);
}


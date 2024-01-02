/**
 *****************************************************************************************
 *
 * @file pcs.c
 *
 * @brief PCS Service Implementation.
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
******************************************************************************************
*/
#include "pcs.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief The UUIDs of PCS characteristics. */
#define PCS_CHARACTERISTIC_TX_UUID      {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x02, 0x05, 0xED, 0xA6}
#define PCS_CHARACTERISTIC_SETTING_UUID {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x03, 0x05, 0xED, 0xA6}

/**@brief Macros for conversion of 128bit to 16bit UUID. */
#define ATT_128_PRIMARY_SERVICE BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_PRIMARY_SERVICE)
#define ATT_128_CHARACTERISTIC  BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC)
#define ATT_128_CLIENT_CHAR_CFG BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DESC_CLIENT_CHAR_CFG)

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief PCS Service Attributes Indexes. */
enum pcs_attr_idx_t
{
    PCS_IDX_SVC,

    PCS_IDX_TX_CHAR,
    PCS_IDX_TX_VAL,
    PCS_IDX_TX_CFG,

    PCS_IDX_SETTING_CHAR,
    PCS_IDX_SETTING_VAL,
    PCS_IDX_SETTING_CFG,

    PCS_IDX_NB,
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief PCS Service environment variable. */
struct pcs_env_t
{
    pcs_init_t pcs_init;                               /**< PCS Service initialization variables. */
    uint16_t   start_hdl;                              /**< Start handle of services */
    uint16_t   tx_ntf_cfg[PCS_CONNECTION_MAX];         /**< TX Characteristic Notification configuration of the peers. */
    uint16_t   setting_ind_cfg[PCS_CONNECTION_MAX];    /**< Setting Characteristic Indication configuration of the peers. */
    ble_gatts_create_db_t   pcs_gatts_db;                            /**< Running Speed and Cadence Service attributs database. */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct pcs_env_t s_pcs_env;
static const uint16_t   s_char_mask = 0xFFFF;
static const uint8_t    s_pcs_svc_uuid[] = {PCS_SERVICE_UUID};

/**@brief Full PCS Database Description which is used to add attributes into the ATT database. */
static const ble_gatts_attm_desc_128_t pcs_attr_tab[PCS_IDX_NB] =
{
    // PCS service
    [PCS_IDX_SVC]            = {ATT_128_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // PCS TX Characteristic Declaration
    [PCS_IDX_TX_CHAR]        = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // PCS TX Characteristic Value
    [PCS_IDX_TX_VAL]         = {PCS_CHARACTERISTIC_TX_UUID, 
                                BLE_GATTS_NOTIFY_PERM_UNSEC,
                                (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                                PCS_MAX_DATA_LEN},
    // PCS TX Characteristic - Client Characteristic Configuration Descriptor
    [PCS_IDX_TX_CFG]         = {ATT_128_CLIENT_CHAR_CFG,
                                BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                0,
                                0},
    
    // PCS settings
    [PCS_IDX_SETTING_CHAR] = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // PCS settings Value
    [PCS_IDX_SETTING_VAL]  = {PCS_CHARACTERISTIC_SETTING_UUID,
                              (BLE_GATTS_WRITE_CMD_PERM_UNSEC | BLE_GATTS_INDICATE_PERM_UNSEC),
                              (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                               PCS_MAX_DATA_LEN},
    // ths settings cfg
    [PCS_IDX_SETTING_CFG]  = {ATT_128_CLIENT_CHAR_CFG,
                              (BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC | BLE_GATTS_WRITE_CMD_PERM_UNSEC),
                              0,
                              0},
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Index of the connection.
 * @param[in] p_param:  Pointer to the parameters of the read request.
 *****************************************************************************************
 */
static void pcs_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t cfm;
    uint16_t         handle    = p_param->handle;
    uint8_t          tab_index = 0;

    tab_index  = prf_find_idx_by_handle(handle, s_pcs_env.start_hdl, PCS_IDX_NB, (uint8_t *)&s_char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case PCS_IDX_TX_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_pcs_env.tx_ntf_cfg[conn_idx];
            cfm.status = BLE_SUCCESS;
            break;

        case PCS_IDX_SETTING_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_pcs_env.setting_ind_cfg[conn_idx];
            cfm.status = BLE_SUCCESS;
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
 * @param[in] conn_idx: Index of the connection.
 * @param[in] p_param:  Point to the parameters of the write request.
 *****************************************************************************************
 */
static void pcs_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint8_t           handle    = p_param->handle;
    uint8_t           tab_index = 0;
    uint16_t          cccd_value;
    pcs_evt_t         event;
    ble_gatts_write_cfm_t cfm;

    tab_index      = prf_find_idx_by_handle(handle, s_pcs_env.start_hdl, PCS_IDX_NB, (uint8_t *)&s_char_mask);
    event.conn_idx = conn_idx;
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    
    switch (tab_index)
    {

        case PCS_IDX_TX_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? PCS_EVT_TX_ENABLE : PCS_EVT_TX_DISABLE;
            s_pcs_env.tx_ntf_cfg[conn_idx] = cccd_value;
            break;

        case PCS_IDX_SETTING_VAL:
            event.evt_type = PCS_EVT_PARAM_SET;
            break;

        case PCS_IDX_SETTING_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = (PRF_CLI_START_IND == cccd_value) ? PCS_EVT_SETTING_ENABLE : PCS_EVT_SETTING_DISABLE;
            s_pcs_env.setting_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && PCS_EVT_INVALID != event.evt_type && s_pcs_env.pcs_init.evt_handler)
    {
        event.conn_idx = conn_idx;
        event.p_data   = (uint8_t *)p_param->value;
        event.length   = p_param->length;

        s_pcs_env.pcs_init.evt_handler(&event);
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
static void pcs_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint8_t           tab_index = 0;
    pcs_evt_t         event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index      = prf_find_idx_by_handle(handle, s_pcs_env.start_hdl, PCS_IDX_NB, (uint8_t *)&s_char_mask);
    event.conn_idx = conn_idx;
    event.evt_type = PCS_EVT_INVALID;

    switch (tab_index)
    {
        case PCS_IDX_SETTING_CFG:
            event.evt_type = (PRF_CLI_START_IND == cccd_value) ? PCS_EVT_SETTING_ENABLE : PCS_EVT_SETTING_DISABLE;
            s_pcs_env.setting_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }

    if (PCS_EVT_INVALID != event.evt_type && s_pcs_env.pcs_init.evt_handler)
    {
        s_pcs_env.pcs_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the disconnected event.
 *
 * @param[in] conn_idx: Index of the connection.
 *****************************************************************************************
 */
static void pcs_disconnect_evt_handler(uint8_t conn_idx, uint8_t disconn_reason)
{
    pcs_evt_t  event = 
    {
        .conn_idx = conn_idx,
        .evt_type = PCS_EVT_DISCONNECTED,
        .p_data   = &disconn_reason,
        .length   = sizeof(uint8_t)
    };

    if (s_pcs_env.pcs_init.evt_handler)
    {
        s_pcs_env.pcs_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the complete event.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] status:  The status of sending notifications or indications.
 * @param[in] p_ntf_ind:  Pointer to the structure of the complete event.
 *****************************************************************************************
 */
static void pcs_ntf_ind_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    if (NULL != s_pcs_env.pcs_init.evt_handler)
    {
        pcs_evt_t event;
        event.conn_idx = conn_idx;

        if (BLE_SUCCESS == status && BLE_GATT_NOTIFICATION == p_ntf_ind->type)
        {
            event.evt_type = PCS_EVT_TX_DATA_SENT;
            s_pcs_env.pcs_init.evt_handler(&event);
        }
    }
}

static void pcs_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            pcs_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            pcs_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_NTF_IND:
            pcs_ntf_ind_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            pcs_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            pcs_disconnect_evt_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t pcs_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    ble_gatts_noti_ind_t send_rsp;

    if (PRF_CLI_START_NTF == s_pcs_env.tx_ntf_cfg[conn_idx])
    {
        // Fill in the parameter structure
        send_rsp.type   = BLE_GATT_NOTIFICATION;
        send_rsp.handle = prf_find_handle_by_idx(PCS_IDX_TX_VAL, s_pcs_env.start_hdl, (uint8_t *)&s_char_mask);

        // Pack measured value in database
        send_rsp.length = length;
        send_rsp.value  = p_data;

        // Send notification to peer device
        error_code = ble_gatts_noti_ind(conn_idx, &send_rsp);
    }

    return error_code;
}

sdk_err_t pcs_setting_reply(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    sdk_err_t        error_code = SDK_ERR_IND_DISABLED;
    ble_gatts_noti_ind_t send_cmd;

    if (PRF_CLI_START_IND == s_pcs_env.setting_ind_cfg[conn_idx])
    {
        // Fill in the parameter structure
        send_cmd.type   = BLE_GATT_INDICATION;
        send_cmd.handle = prf_find_handle_by_idx(PCS_IDX_SETTING_VAL, s_pcs_env.start_hdl, (uint8_t *)&s_char_mask);

        // Pack measured value in database
        send_cmd.length = length;
        send_cmd.value  = p_data;

        // Send indication to peer device
        error_code = ble_gatts_noti_ind(conn_idx, &send_cmd);
    }

    return error_code;
}

sdk_err_t pcs_service_init(pcs_init_t *p_pcs_init)
{
    if (NULL == p_pcs_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memcpy(&s_pcs_env.pcs_init, p_pcs_init, sizeof(pcs_init_t));

    s_pcs_env.start_hdl  = PRF_INVALID_HANDLE;

    s_pcs_env.pcs_gatts_db.shdl                  = &s_pcs_env.start_hdl;
    s_pcs_env.pcs_gatts_db.uuid                  = s_pcs_svc_uuid;
    s_pcs_env.pcs_gatts_db.attr_tab_cfg          = (uint8_t *)&s_char_mask;
    s_pcs_env.pcs_gatts_db.max_nb_attr           = PCS_IDX_NB;
    s_pcs_env.pcs_gatts_db.srvc_perm             = BLE_GATTS_SRVC_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128); 
    s_pcs_env.pcs_gatts_db.attr_tab_type         = BLE_GATTS_SERVICE_TABLE_TYPE_128;
    s_pcs_env.pcs_gatts_db.attr_tab.attr_tab_128 = pcs_attr_tab;

    return ble_gatts_prf_add(&s_pcs_env.pcs_gatts_db, pcs_ble_evt_handler);
}

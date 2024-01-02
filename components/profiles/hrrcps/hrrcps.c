/**
 ****************************************************************************************
 *
 * @file hrrcps.c
 *
 * @brief HRS hrrcps Relay Control Point Service Implementation.
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
#include "hrrcps.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
* DEFINES
*****************************************************************************************
*/
#define HRRCPS_CTRL_PT_CHARACTERISTIC_UUID        {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                                   0x0A,0x46, 0x44, 0xD3, 0x02, 0x06, 0xED, 0xA6}
#define HRRCPS_CTRL_PT_RSP_CHARACTERISTIC_UUID    {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                                   0x0A, 0x46, 0x44, 0xD3, 0x03, 0x06, 0xED, 0xA6}

/**@brief Macros for conversion of 128bit to 16bit UUID. */
#define ATT_128_PRIMARY_SERVICE     BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_PRIMARY_SERVICE)
#define ATT_128_CHARACTERISTIC      BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC)
#define ATT_128_CLIENT_CHAR_CFG     BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DESC_CLIENT_CHAR_CFG)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/**@brief HRS hrrcps Relay Control Point Service Attributes Indexes. */
enum
{
    // HRS hrrcps Relay Control Point Service
    HRRCPS_IDX_SVC,

    // HRR Control Point
    HRRCPS_IDX_HRR_CTRL_PT_CHAR,
    HRRCPS_IDX_HRR_CTRL_PT_VAL,

    // HRR Control Point Response
    HRRCPS_IDX_HRR_CTRL_PT_RSP_CHAR,
    HRRCPS_IDX_HRR_CTRL_PT_RSP_VAL,
    HRRCPS_IDX_HRR_CTRL_PT_RSP_CFG,

    HRRCPS_IDX_NB
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief HRS hrrcps Relay Control Point Service environment variable. */
struct hrrcps_env_t
{
    hrrcps_evt_handler_t   evt_handler;                                 /**< HRS hrrcps Relay Control Point Service event handler. */
    uint16_t               start_hdl;                                   /**< HRS hrrcps Relay Control Point Service start handle. */
    uint16_t               ctrl_pt_rsp_ind_cfg[HRRCPS_CONNECTION_MAX];  /**< The configuration of Control Point Response which is configured by the peer devices. */
    ble_gatts_create_db_t  hrrcps_gatts_db;                            /**< HRS hrrcps Relay Control Point Service attributs database. */
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void        hrrcps_ctrl_pt_handler(uint8_t conn_idx, hrrcps_ctrl_pt_id_t ctrl_pt_id);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct hrrcps_env_t s_hrrcps_env;
static uint16_t            s_hrrcps_char_mask = 0x7f;
static const uint8_t       s_hrrcps_svc_uuid[] = {HRRCPS_SERVICE_UUID};

/**@brief Full HRRCPS Database Description - Used to add attributes into the database. */
static const ble_gatts_attm_desc_128_t hrrcps_attr_tab[HRRCPS_IDX_NB] =
{
    // HRS hrrcps Relay Control Point Service
    [HRRCPS_IDX_SVC] = {ATT_128_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // HRR Control Point Characteristic - Declaration
    [HRRCPS_IDX_HRR_CTRL_PT_CHAR] = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // HRR Control Point Characteristic - Value
    [HRRCPS_IDX_HRR_CTRL_PT_VAL]  = {HRRCPS_CTRL_PT_CHARACTERISTIC_UUID,
                                     BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                     (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                                     HRRCPS_CTRL_PT_VAL_LEN},

    // HRR Control Point Response Characteristic - Declaration
    [HRRCPS_IDX_HRR_CTRL_PT_RSP_CHAR] = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // HRR Control Point Response Characteristic - Value
    [HRRCPS_IDX_HRR_CTRL_PT_RSP_VAL]  = {HRRCPS_CTRL_PT_RSP_CHARACTERISTIC_UUID,
                                         BLE_GATTS_INDICATE_PERM_UNSEC,
                                        (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                                         HRRCPS_CTRL_PT_RSP_VAL_LEN},
    // HRR Control Point Responset Characteristic - Client Characteristic Configuration Descriptor
    [HRRCPS_IDX_HRR_CTRL_PT_RSP_CFG]  = {ATT_128_CLIENT_CHAR_CFG, BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC, 0, 0},
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
 * @param[in] p_param:  Pointer to the parameters of the read request.
 *****************************************************************************************
 */
static void hrrcps_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t cfm;
    uint8_t          handle    = p_param->handle;
    uint8_t          tab_index = 0;

    tab_index  = prf_find_idx_by_handle(handle, s_hrrcps_env.start_hdl, HRRCPS_IDX_NB, (uint8_t *)&s_hrrcps_char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case HRRCPS_IDX_HRR_CTRL_PT_RSP_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_hrrcps_env.ctrl_pt_rsp_ind_cfg[conn_idx];
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
 * @param[in] conn_idx: Connection index.
 * @param[in] p_param:  Pointer to the parameters of the write request.
 *****************************************************************************************
 */
static void hrrcps_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint8_t              handle           = p_param->handle;
    uint8_t              tab_index        = 0;
    uint16_t             cccd_value       = 0;
    bool                 is_ctrl_pt_valid = false;
    hrrcps_evt_t         event;
    ble_gatts_write_cfm_t    cfm;

    tab_index  = prf_find_idx_by_handle(handle, s_hrrcps_env.start_hdl, HRRCPS_IDX_NB, (uint8_t *)&s_hrrcps_char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case HRRCPS_IDX_HRR_CTRL_PT_VAL:
            if ((HRRCPS_CTRL_PT_SCAN_HRS > p_param->value[0]) || \
                (HRRCPS_CTRL_PT_RSCS_DISCONN < p_param->value[0]))
            {
                cfm.status = 0x80;
            }
            else
            {
                is_ctrl_pt_valid = true;
            }
            break;

        case HRRCPS_IDX_HRR_CTRL_PT_RSP_CFG:
            cccd_value = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ? \
                              HRRCPS_EVT_CTRL_PT_IND_ENABLE : \
                              HRRCPS_EVT_CTRL_PT_IND_DISABLE);
            s_hrrcps_env.ctrl_pt_rsp_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    if (is_ctrl_pt_valid)
    {
        hrrcps_ctrl_pt_handler(conn_idx, (hrrcps_ctrl_pt_id_t)p_param->value[0]);
    }
    else if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && HRRCPS_EVT_INVALID != event.evt_type && s_hrrcps_env.evt_handler)
    {
        event.conn_idx = conn_idx;
        s_hrrcps_env.evt_handler(&event);
    }

    ble_gatts_write_cfm(conn_idx, &cfm);
}

/**
 *****************************************************************************************
 * @brief Handle HRR Control Point.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] ctrl_pt_id: HRRCPS Control Point ID
 *
 * @return True or False
 *****************************************************************************************
 */
static void hrrcps_ctrl_pt_handler(uint8_t conn_idx, hrrcps_ctrl_pt_id_t ctrl_pt_id)
{
    hrrcps_evt_t event;

    event.conn_idx = conn_idx;

    switch (ctrl_pt_id)
    {
        case HRRCPS_CTRL_PT_SCAN_HRS:
            event.evt_type = HRRCPS_EVT_SCAN_HRS;
            break;

        case HRRCPS_CTRL_PT_SCAN_RSCS:
            event.evt_type = HRRCPS_EVT_SCAN_RSCS;
            break;

        case HRRCPS_CTRL_PT_HRS_SEN_LOC_READ:
            event.evt_type = HRRCPS_EVT_HRS_SENSOR_LOC_READ;
            break;
        
        case HRRCPS_CTRL_PT_RSCS_SEN_LOC_READ:
            event.evt_type = HRRCPS_EVT_RSCS_SENSOR_LOC_READ;
            break;
            
        case HRRCPS_CTRL_PT_HRS_NTF_ENABLE:
            event.evt_type = HRRCPS_EVT_ENABLE_HRS_NTF;
            break;
        
        case HRRCPS_CTRL_PT_RSCS_NTF_ENABLE:
            event.evt_type = HRRCPS_EVT_ENABLE_RSCS_NTF;
            break;
        
        case HRRCPS_CTRL_PT_HRS_NTF_DISABLE:
            event.evt_type = HRRCPS_EVT_DISABLE_HRS_NTF;
            break;
        
        case HRRCPS_CTRL_PT_RSCS_NTF_DISABLE:
            event.evt_type = HRRCPS_EVT_DISABLE_RSCS_NTF;
            break;

        case HRRCPS_CTRL_PT_HRS_DISCONN:
            event.evt_type = HRRCPS_EVT_DISCONN_HRS_LINK;
            break;

        case HRRCPS_CTRL_PT_RSCS_DISCONN:
            event.evt_type = HRRCPS_EVT_DISCONN_RSCS_LINK;
            break;

        default:
            break;
    }

    if (HRRCPS_EVT_INVALID != event.evt_type && s_hrrcps_env.evt_handler)
    {
        s_hrrcps_env.evt_handler(&event);
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
static void hrrcps_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint8_t              tab_index        = 0;
    hrrcps_evt_t         event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index  = prf_find_idx_by_handle(handle, s_hrrcps_env.start_hdl, HRRCPS_IDX_NB, (uint8_t *)&s_hrrcps_char_mask);

    switch (tab_index)
    {
        case HRRCPS_IDX_HRR_CTRL_PT_RSP_CFG:
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ? \
                              HRRCPS_EVT_CTRL_PT_IND_ENABLE : \
                              HRRCPS_EVT_CTRL_PT_IND_DISABLE);
            s_hrrcps_env.ctrl_pt_rsp_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            event.evt_type = HRRCPS_EVT_INVALID;
            break;
    }

    if (HRRCPS_EVT_INVALID != event.evt_type && s_hrrcps_env.evt_handler)
    {
        event.conn_idx = conn_idx;
        s_hrrcps_env.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Encode HRR Control Point value.
 *
 * @param[in]  p_ctrl_val_pt:  Pointer to Control Point value.
 * @param[out] p_encoded_buff: Pointer to buffer encoded.
 *
 * @return Length of encoded
 *****************************************************************************************
 */
static uint16_t hrrcps_ctrl_pt_rsp_encode(hrrcps_rsp_val_t *p_rsp_val, uint8_t *p_encoded_buff)
{
    uint16_t length = 0;

    p_encoded_buff[length++] = HRRCPS_CTRL_PT_RSP_CODE;
    p_encoded_buff[length++] = p_rsp_val->cmd_id;
    p_encoded_buff[length++] = p_rsp_val->rsp_id;

    if (p_rsp_val->is_inc_prama)
    {
        p_encoded_buff[length++] = p_rsp_val->rsp_param;
    }

    return length;
}

static void hrrcps_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            hrrcps_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            hrrcps_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            hrrcps_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t hrrcps_ctrl_pt_rsp_send(uint8_t conn_idx, hrrcps_rsp_val_t *p_rsp_val)
{
    uint8_t          encoded_ctrl_pt_rsp[HRRCPS_CTRL_PT_RSP_VAL_LEN];
    ble_gatts_noti_ind_t ctrl_pt_rsp_ind;
    uint16_t         encoded_length;

    encoded_length =hrrcps_ctrl_pt_rsp_encode(p_rsp_val, encoded_ctrl_pt_rsp);

    if (PRF_CLI_START_IND == s_hrrcps_env.ctrl_pt_rsp_ind_cfg[conn_idx])
    {
        ctrl_pt_rsp_ind.type    = BLE_GATT_INDICATION;
        ctrl_pt_rsp_ind.handle  = prf_find_handle_by_idx(HRRCPS_IDX_HRR_CTRL_PT_RSP_VAL,
                                                         s_hrrcps_env.start_hdl,
                                                         (uint8_t *)&s_hrrcps_char_mask);
        ctrl_pt_rsp_ind.length  = encoded_length;
        ctrl_pt_rsp_ind.value   = encoded_ctrl_pt_rsp;

        return ble_gatts_noti_ind(conn_idx, &ctrl_pt_rsp_ind);
    }

    return SDK_ERR_IND_DISABLED;
}

sdk_err_t hrrcps_service_init(hrrcps_evt_handler_t evt_handler)
{
    s_hrrcps_env.evt_handler = evt_handler;

    s_hrrcps_env.start_hdl  = PRF_INVALID_HANDLE;

    s_hrrcps_env.hrrcps_gatts_db.shdl                  = &s_hrrcps_env.start_hdl;
    s_hrrcps_env.hrrcps_gatts_db.uuid                  = s_hrrcps_svc_uuid;
    s_hrrcps_env.hrrcps_gatts_db.attr_tab_cfg          = (uint8_t *)&s_hrrcps_char_mask;
    s_hrrcps_env.hrrcps_gatts_db.max_nb_attr           = HRRCPS_IDX_NB;
    s_hrrcps_env.hrrcps_gatts_db.srvc_perm             = BLE_GATTS_SRVC_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128); 
    s_hrrcps_env.hrrcps_gatts_db.attr_tab_type         = BLE_GATTS_SERVICE_TABLE_TYPE_128;
    s_hrrcps_env.hrrcps_gatts_db.attr_tab.attr_tab_128 = hrrcps_attr_tab;

    return ble_gatts_prf_add(&s_hrrcps_env.hrrcps_gatts_db, hrrcps_ble_evt_handler);
}

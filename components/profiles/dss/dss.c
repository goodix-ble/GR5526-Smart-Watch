/**
 *****************************************************************************************
 *
 * @file dss.h
 *
 * @brief Device Synchronize Service Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redsstribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redsstributions of source code must retain the above copyright
    notice, this list of conditions and the following dssclaimer.
  * Redsstributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following dssclaimer in the
    documentation and/or other materials provided with the dsstribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DSSCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
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
#include "dss.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include "app_error.h"
#include "app_log.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief The UUIDs of DSS service and characteristics. */
#define DSS_ROLE_UUID       {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x02, 0x0A, 0xED, 0xA6}
#define DSS_EVT_CNT_UUID    {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x03, 0x0A, 0xED, 0xA6}
#define DSS_EVT_PERIOD_UUID {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x04, 0x0A, 0xED, 0xA6}
#define DSS_STATUS_UUID     {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x05, 0x0A, 0xED, 0xA6}
#define DSS_CTRL_PT_UUID    {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x06, 0x0A, 0xED, 0xA6}

/**@brief Macros for conversion of 128bit to 16bit UUID. */
#define ATT_128_PRIMARY_SERVICE BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_PRIMARY_SERVICE)
#define ATT_128_CHARACTERISTIC  BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC)
#define ATT_128_CLIENT_CHAR_CFG BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DESC_CLIENT_CHAR_CFG)

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief Device Synchronize Service Attributes database index list. */
enum dss_attr_idx_t
{
    DSS_IDX_SVC,

    DSS_IDX_ROLE_CHAR,
    DSS_IDX_ROLE_VAL,

    DSS_IDX_SYNC_CNT_CHAR,
    DSS_IDX_SYNC_CNT_VAL,
    DSS_IDX_SYNC_CNT_CFG,

    DSS_IDX_SYNC_PERIOD_CHAR,
    DSS_IDX_SYNC_PERIOD_VAL,

    DSS_IDX_SYNC_STATUS_CHAR,
    DSS_IDX_SYNC_STATUS_VAL,

    DSS_IDX_CTRL_PT_CHAR,
    DSS_IDX_CTRL_PT_VAL,
    DSS_IDX_CTRL_PT_CFG,

    DSS_IDX_NB,
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Device Synchronize Service environment variable. */
struct dss_env_t
{
    dss_evt_handler_t     evt_handler;
    uint16_t              start_hdl;
    uint16_t              char_mask;
    dss_role_t            dss_role;
    dss_staus_t           sync_status;
    uint16_t              event_period;
    uint32_t              sync_cnt;
    uint8_t               sync_cfg_conn_idx;
    bool                  is_busy_send;
    bool                  is_auto_calib_drift;
    bool                  is_auto_enter_lp;
    bool                  is_in_lp;
    uint32_t              auto_calib_timing;
    uint8_t               sync_device_num;
    uint16_t              evt_cnt_ntf[DSS_CONNECTION_MAX];
    uint16_t              ctrl_pt_ind[DSS_CONNECTION_MAX];
    ble_gatts_create_db_t dss_gatts_db;                            /**< Device Synchronize Service attributs database. */
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void      dss_ctrl_pt_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length);
static sdk_err_t dss_ctrl_pt_rsp_send(uint8_t conn_idx, dss_op_id_t op_id, dss_rsp_id_t rsp_id);
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct dss_env_t s_dss_env;
static const uint8_t    s_dss_svc_uuid[] = {DSS_SERVICE_UUID};

/**@brief Full DSS Database Description which is used to add attributes into the ATT database. */
static const ble_gatts_attm_desc_128_t dss_attr_tab[DSS_IDX_NB] =
{
    [DSS_IDX_SVC]              = {ATT_128_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    [DSS_IDX_ROLE_CHAR]        = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    [DSS_IDX_ROLE_VAL]         = {DSS_ROLE_UUID,
                                  BLE_GATTS_READ_PERM_UNSEC,
                                  BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128),
                                  DSS_ROLE_VALUE_LEN},

    [DSS_IDX_SYNC_CNT_CHAR]     = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    [DSS_IDX_SYNC_CNT_VAL]      = {DSS_EVT_CNT_UUID,
                                  BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_NOTIFY_PERM_UNSEC,
                                  BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128),
                                  DSS_EVT_CNT_VALUE_LEN},
    [DSS_IDX_SYNC_CNT_CFG]      = {ATT_128_CLIENT_CHAR_CFG, BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC, 0, 0},

    [DSS_IDX_SYNC_PERIOD_CHAR]  = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    [DSS_IDX_SYNC_PERIOD_VAL]   = {DSS_EVT_PERIOD_UUID,
                                  BLE_GATTS_READ_PERM_UNSEC,
                                  BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128),
                                  DSS_EVT_PERIOD_VALUE_LEN},

    [DSS_IDX_SYNC_STATUS_CHAR] = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    [DSS_IDX_SYNC_STATUS_VAL]  = {DSS_STATUS_UUID,
                                  BLE_GATTS_READ_PERM_UNSEC,
                                  BLE_GATTS_ATT_VAL_LOC_USER  | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128),
                                  DSS_STATUS_VALUE_LEN},

    [DSS_IDX_CTRL_PT_CHAR]     = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    [DSS_IDX_CTRL_PT_VAL]      = {DSS_CTRL_PT_UUID,
                                  BLE_GATTS_WRITE_REQ_PERM_UNSEC | BLE_GATTS_INDICATE_PERM_UNSEC,
                                  BLE_GATTS_ATT_VAL_LOC_USER  | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128),
                                  DSS_CTRL_PT_VALUE_LEN},
    [DSS_IDX_CTRL_PT_CFG]      = {ATT_128_CLIENT_CHAR_CFG, BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC, 0, 0},
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Device Synchronize Service disconnect callback.
 *****************************************************************************************
 */
static void dss_disconnect_evt_handler(uint8_t conn_idx, uint8_t reason)
{
    s_dss_env.evt_cnt_ntf[conn_idx] = 0x0000;
    s_dss_env.ctrl_pt_ind[conn_idx] = 0x0000;

    if (conn_idx == s_dss_env.sync_cfg_conn_idx)
    {
        s_dss_env.sync_cfg_conn_idx = BLE_GAP_INVALID_CONN_INDEX;
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_param:  Pointer to the parameters of the read request.
 *****************************************************************************************
 */
static void dss_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t cfm;
    uint16_t         handle    = p_param->handle;
    uint8_t          tab_index = prf_find_idx_by_handle(handle,
                                 s_dss_env.start_hdl,
                                 DSS_IDX_NB,
                                 (uint8_t *)&s_dss_env.char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case DSS_IDX_ROLE_VAL:
            cfm.length = DSS_ROLE_VALUE_LEN;
            cfm.value  = (uint8_t *)&s_dss_env.dss_role;
            break;

        case DSS_IDX_SYNC_CNT_VAL:
            cfm.length = DSS_EVT_CNT_VALUE_LEN;
            cfm.value  = (uint8_t *)&s_dss_env.sync_cnt;
            break;

        case DSS_IDX_SYNC_CNT_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_dss_env.evt_cnt_ntf[conn_idx];
            break;

        case DSS_IDX_SYNC_PERIOD_VAL:
            cfm.length = DSS_EVT_PERIOD_VALUE_LEN;
            cfm.value  = (uint8_t *)&s_dss_env.event_period;
            break;

        case DSS_IDX_SYNC_STATUS_VAL:
            cfm.length = DSS_STATUS_VALUE_LEN;
            cfm.value  = (uint8_t *)&s_dss_env.sync_status;
            break;

        case DSS_IDX_CTRL_PT_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_dss_env.ctrl_pt_ind[conn_idx];
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
static void dss_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint8_t           handle    = p_param->handle;
    uint8_t           tab_index = 0;
    ble_gatts_write_cfm_t cfm;
    bool              ctrl_pt_evt = false;

    s_dss_env.sync_cfg_conn_idx = conn_idx;
    tab_index  = prf_find_idx_by_handle(handle,
                                        s_dss_env.start_hdl,
                                        DSS_IDX_NB,
                                        (uint8_t *)&s_dss_env.char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;
    
    switch (tab_index)
    {
        case DSS_IDX_SYNC_CNT_CFG:
            s_dss_env.evt_cnt_ntf[conn_idx] = le16toh(&p_param->value[0]);
            break;

        case DSS_IDX_CTRL_PT_VAL:
            ctrl_pt_evt = true;
            break;

        case DSS_IDX_CTRL_PT_CFG:
            s_dss_env.ctrl_pt_ind[conn_idx] = le16toh(&p_param->value[0]);
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_write_cfm(conn_idx, &cfm);

    if (ctrl_pt_evt)
    {
        dss_ctrl_pt_handler(conn_idx, p_param->value, p_param->length);
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
static void dss_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint8_t tab_index = 0;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index = prf_find_idx_by_handle(handle,
                                       s_dss_env.start_hdl,
                                       DSS_IDX_NB,
                                       (uint8_t *)&s_dss_env.char_mask);

    switch (tab_index)
    {
        case DSS_IDX_SYNC_CNT_CFG:
            s_dss_env.evt_cnt_ntf[conn_idx] = cccd_value;
            break;

        case DSS_IDX_CTRL_PT_CFG:
            s_dss_env.ctrl_pt_ind[conn_idx] = cccd_value;
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the complete event.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] status:     The status of GATTC operation.
 * @param[in] p_ntf_ind:  Pointer to the parameters of the complete event.
 *****************************************************************************************
 */
static void dss_ntf_ind_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    if (BLE_SUCCESS == status && BLE_GATT_NOTIFICATION == p_ntf_ind->type)
    {
        s_dss_env.is_busy_send = false;
    }
    else if (status && BLE_GATT_NOTIFICATION == p_ntf_ind->type)
    {
        s_dss_env.is_busy_send = true;
    }
}


/**
 *****************************************************************************************
 * @brief Set role OP handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void dss_op_role_set_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    dss_rsp_id_t rsp_id = DSS_RSP_ID_SUCCESS;
    dss_evt_t      evt;

    if (length != 2 || (p_data[1] != DSS_ROLE_SYNC_SOURCE && p_data[1] != DSS_ROLE_SYNC_DEVICE))
    {
        rsp_id = DSS_RSP_ID_PARAM_ERR;
    }
    else if (s_dss_env.sync_status != DSS_STATUS_CFG_READY)
    {
        rsp_id = DSS_RSP_ID_STATUS_ERR;
    }
    else
    {
        s_dss_env.dss_role = (dss_role_t)p_data[1];

        evt.evt_type = s_dss_env.dss_role == DSS_ROLE_SYNC_SOURCE ? DSS_EVT_SOURCE_ROLE_SET : DSS_EVT_DEVICE_ROLE_SET;
        s_dss_env.evt_handler(&evt);
    }

    dss_ctrl_pt_rsp_send(conn_idx, DSS_OP_ID_ROLE_SET, rsp_id);
}

/**
 *****************************************************************************************
 * @brief Create sync source OP handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void dss_op_sync_src_create_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    dss_rsp_id_t rsp_id = DSS_RSP_ID_SUCCESS;
    uint16_t     event_period = le16toh(&p_data[1]);
    dss_evt_t    evt;

    if (length != 3 || (320 > le16toh(&p_data[1])) || (3200 < le16toh(&p_data[1])))
    {
        rsp_id = DSS_RSP_ID_PARAM_ERR;
    }
    else if (s_dss_env.sync_status != DSS_STATUS_CFG_READY)
    {
        rsp_id = DSS_RSP_ID_STATUS_ERR;
    }
    else if (s_dss_env.dss_role != DSS_ROLE_SYNC_SOURCE)
    {
        rsp_id = DSS_RSP_ID_ROLE_ERR;
    }
    else
    {
        if (ble_sync_source_create(event_period))
        {
             rsp_id = DSS_RSP_ID_CREATE_SRC_FAIL;
        }
        else
        {
            s_dss_env.sync_status         = DSS_STATUS_CFG_READY;
            s_dss_env.event_period        = event_period;
            
            evt.evt_type = DSS_EVT_SYNC_SRC_CREATE;
            s_dss_env.evt_handler(&evt);
        }
    }

    dss_ctrl_pt_rsp_send(conn_idx, DSS_OP_ID_SYNC_SRC_CREATE, rsp_id);
}


/**
 *****************************************************************************************
 * @brief Synchronize handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void dss_op_sync_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length, dss_op_id_t op_id)
{
    dss_rsp_id_t   rsp_id = DSS_RSP_ID_SUCCESS;
    dss_evt_t      evt;

    if ((op_id != DSS_OP_ID_SYNC) || length != 7 || 0 == p_data[6])
    {
        rsp_id = DSS_RSP_ID_PARAM_ERR;
    }
    else if (s_dss_env.sync_status != DSS_STATUS_CFG_READY)
    {
        rsp_id = DSS_RSP_ID_STATUS_ERR;
    }
    else if (!s_dss_env.event_period && s_dss_env.dss_role == DSS_ROLE_SYNC_SOURCE)
    {
        rsp_id = DSS_RSP_ID_DISALLOWED;
    }
    else if (s_dss_env.dss_role == DSS_ROLE_SYNC_INVALID)
    {
        rsp_id = DSS_RSP_ID_ROLE_ERR;
    }
    else if (!s_dss_env.evt_handler)
    {
        rsp_id = DSS_RSP_ID_NO_HANDLER;
    }
    else
    {
        s_dss_env.is_auto_calib_drift  = (p_data[1] & 0x01) ? true : false;
        s_dss_env.is_auto_enter_lp     = (p_data[1] & 0x02) ? true : false;

        s_dss_env.auto_calib_timing    = BUILD_U32(p_data[2], p_data[3], p_data[4], p_data[5]);
        s_dss_env.sync_device_num      = p_data[6];
        
        if (s_dss_env.is_auto_calib_drift && !s_dss_env.auto_calib_timing)
        {
            s_dss_env.is_auto_calib_drift  = false;
            s_dss_env.is_auto_enter_lp     = false; 
            rsp_id = DSS_RSP_ID_PARAM_ERR;
        }
        else
        {
            s_dss_env.is_in_lp    = false;
            s_dss_env.sync_status = s_dss_env.dss_role == DSS_ROLE_SYNC_SOURCE ? DSS_STATUS_IN_SCAN : DSS_STATUS_IN_ADV;
            
            evt.conn_idx          = conn_idx;
            evt.evt_type          = DSS_EVT_SYNC_SELF_OR_PEER;
            evt.is_enter_lp_mode  = s_dss_env.is_auto_enter_lp;
            evt.sync_dev_num      = s_dss_env.sync_device_num;
            s_dss_env.evt_handler(&evt);
        }
    }

    if (rsp_id)
    {
        dss_ctrl_pt_rsp_send(conn_idx, op_id, rsp_id);
    }
}

/**
 *****************************************************************************************
 * @brief Cancel Synchronization handler.
 *
 * @param[in] conn_idx: Connection index.
 *****************************************************************************************
 */
static void dss_op_cancel_sync_handler(uint8_t conn_idx)
{
    dss_rsp_id_t rsp_id = DSS_RSP_ID_SUCCESS;
    dss_evt_t    evt;

    if (((s_dss_env.sync_status != DSS_STATUS_IN_SCAN) && (DSS_ROLE_SYNC_SOURCE == s_dss_env.dss_role)) ||
        ((s_dss_env.sync_status != DSS_STATUS_IN_ADV) && (DSS_ROLE_SYNC_DEVICE == s_dss_env.dss_role)))
    {
        rsp_id = DSS_RSP_ID_STATUS_ERR;
    }
    else if (NULL == s_dss_env.evt_handler)
    {
        rsp_id = DSS_RSP_ID_NO_HANDLER;
    }
    else
    {
        evt.conn_idx = conn_idx;
        evt.evt_type = DSS_EVT_SYNC_CANCEL;
        s_dss_env.evt_handler(&evt);
    }

    if (rsp_id)
    {
        dss_ctrl_pt_rsp_send(conn_idx, DSS_OP_ID_CANCEL_SYNC, rsp_id);
    }
}

/**
 *****************************************************************************************
 * @brief Enter low power handler.
 *
 * @param[in] conn_idx: Connection index.
 *****************************************************************************************
 */
static void dss_op_lp_enter_handler(uint8_t conn_idx)
{
    dss_rsp_id_t rsp_id = DSS_RSP_ID_SUCCESS;
    dss_evt_t    evt;

    if (s_dss_env.sync_status != DSS_STATUS_CFG_READY)
    {
        rsp_id = DSS_RSP_ID_STATUS_ERR;
    }
    else if (NULL == s_dss_env.evt_handler)
    {
        rsp_id = DSS_RSP_ID_NO_HANDLER;
    }
    else
    {
//        s_dss_env.is_in_lp         = true;
        s_dss_env.is_auto_enter_lp = true;
        s_dss_env.sync_status      = DSS_STATUS_CFG_READY;

        evt.conn_idx               = conn_idx;
        evt.evt_type               = DSS_EVT_LP_ENTER;
        evt.is_enter_lp_mode       = s_dss_env.is_auto_enter_lp;

        dss_ctrl_pt_rsp_send(conn_idx, DSS_OP_ID_LP_ENTER, rsp_id);
        s_dss_env.evt_handler(&evt);
    }

    if (rsp_id)
    {
        dss_ctrl_pt_rsp_send(conn_idx, DSS_OP_ID_LP_ENTER, rsp_id);
    }
}

/**
 *****************************************************************************************
 * @brief Destroy sync.
 *
 * @param[in] conn_idx: Connection index.
 *****************************************************************************************
 */
static void dss_op_sync_destroy_handler(uint8_t conn_idx)
{
    dss_rsp_id_t rsp_id = DSS_RSP_ID_SUCCESS;
    dss_evt_t    evt;

    if (s_dss_env.dss_role == DSS_ROLE_SYNC_INVALID)
    {
        rsp_id = DSS_RSP_ID_ROLE_ERR;
    }
    else if (ble_sync_source_destroy())
    {
        rsp_id = DSS_RSP_ID_DESTROY_SRC_FAIL;
    }
    else
    {
        s_dss_env.sync_status         = DSS_STATUS_CFG_READY;
        s_dss_env.is_in_lp            = false;
        s_dss_env.is_auto_enter_lp    = false;
        s_dss_env.is_auto_calib_drift = false;
        s_dss_env.event_period        = 0;
        s_dss_env.sync_cnt            = 0;
        s_dss_env.sync_device_num     = 0;
        
        evt.conn_idx                  = conn_idx;
        evt.evt_type                  = DSS_EVT_SYNC_DESTROY;
        s_dss_env.evt_handler(&evt);
    }
    
    if (rsp_id)
    {
        dss_ctrl_pt_rsp_send(conn_idx, DSS_OP_ID_SYNC_DESTROY, rsp_id);
    }
}

/**
 *****************************************************************************************
 * @brief Send Control Point Response.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] op_id:    OP ID.
 * @param[in] rsp_id:   Response id.
 *****************************************************************************************
 */
static sdk_err_t dss_ctrl_pt_rsp_send(uint8_t conn_idx, dss_op_id_t op_id, dss_rsp_id_t rsp_id)
{
    sdk_err_t        error_code = SDK_ERR_IND_DISABLED;
    uint8_t          rsp[DSS_CTRL_PT_RSP_VAL_LEN] = {0};
    ble_gatts_noti_ind_t rsp_ind;

    rsp[0] = DSS_OP_ID_RSP;
    rsp[1] = op_id;
    rsp[2] = rsp_id;

    if (s_dss_env.ctrl_pt_ind[conn_idx] == PRF_CLI_START_IND)
    {
        rsp_ind.type   = BLE_GATT_INDICATION;
        rsp_ind.handle = prf_find_handle_by_idx(DSS_IDX_CTRL_PT_VAL, s_dss_env.start_hdl, (uint8_t *)&s_dss_env.char_mask);
        rsp_ind.length = DSS_CTRL_PT_RSP_VAL_LEN;
        rsp_ind.value  = rsp;

        error_code = ble_gatts_noti_ind(conn_idx, &rsp_ind);
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Control Point handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void dss_ctrl_pt_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    switch(p_data[0])
    {
        case DSS_OP_ID_ROLE_SET:
            dss_op_role_set_handler(conn_idx, p_data, length);
            break;

        case DSS_OP_ID_SYNC_SRC_CREATE:
            dss_op_sync_src_create_handler(conn_idx, p_data, length);
            break;

        case DSS_OP_ID_SYNC:
            dss_op_sync_handler(conn_idx, p_data, length, (dss_op_id_t)p_data[0]);
            break;

        case DSS_OP_ID_LP_ENTER:
            dss_op_lp_enter_handler(conn_idx);
            break;

        case DSS_OP_ID_SYNC_DESTROY:
            dss_op_sync_destroy_handler(conn_idx);
            break;
        
        case DSS_OP_ID_CANCEL_SYNC:
            dss_op_cancel_sync_handler(conn_idx);
            break;

        default:
            dss_ctrl_pt_rsp_send(conn_idx, (dss_op_id_t)p_data[0], DSS_RSP_ID_UNSUPPORT);
            break;
    }
}


static void dss_sync_evt_cb(uint32_t sync_cnt, uint16_t event_period)
{
    ble_gatts_noti_ind_t send_ntf;
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    dss_evt_t        evt;

    if (NULL == s_dss_env.evt_handler)
    {
        return;
    }

    evt.evt_type = DSS_EVT_INVALID;

    evt.conn_idx = BLE_GAP_INVALID_CONN_INDEX;
    evt.evt_type = DSS_EVT_SYNC_OCCUR;
    evt.sync_cnt = sync_cnt;

    s_dss_env.evt_handler(&evt);

    if (!s_dss_env.is_in_lp && s_dss_env.is_auto_enter_lp && s_dss_env.evt_handler)
    {
        evt.evt_type         = DSS_EVT_LP_ENTER;
        evt.is_enter_lp_mode = true;

        s_dss_env.evt_handler(&evt);
    }

    if (s_dss_env.is_auto_calib_drift && !(sync_cnt % s_dss_env.auto_calib_timing))
    {
        evt.sync_dev_num      = s_dss_env.sync_device_num; 
        evt.evt_type          = DSS_EVT_SYNC_SELF_OR_PEER;
        evt.is_enter_lp_mode  = s_dss_env.is_auto_enter_lp;

        s_dss_env.sync_status = s_dss_env.dss_role == DSS_ROLE_SYNC_DEVICE ? DSS_STATUS_IN_ADV : DSS_STATUS_IN_SCAN;
        s_dss_env.evt_handler(&evt);
        s_dss_env.is_in_lp = false;
    }
    else
    {
        s_dss_env.sync_cnt     = sync_cnt;
        s_dss_env.event_period = event_period;

        if (!s_dss_env.is_busy_send)
        {
            if (s_dss_env.evt_cnt_ntf[s_dss_env.sync_cfg_conn_idx] == PRF_CLI_START_NTF)
            {
                send_ntf.type   = BLE_GATT_NOTIFICATION;
                send_ntf.handle = prf_find_handle_by_idx(DSS_IDX_SYNC_CNT_VAL, s_dss_env.start_hdl, (uint8_t *)&s_dss_env.char_mask);
                send_ntf.length = DSS_EVT_CNT_VALUE_LEN;
                send_ntf.value  = (uint8_t *)&s_dss_env.sync_cnt;

                error_code = ble_gatts_noti_ind(s_dss_env.sync_cfg_conn_idx, &send_ntf);
                APP_ERROR_CHECK(error_code);
            }
        }
    }
}

static void dss_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            dss_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            dss_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_NTF_IND:
            dss_ntf_ind_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            dss_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            dss_disconnect_evt_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t dss_service_init(dss_evt_handler_t evt_handler)
{
    if (NULL == evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    ble_sync_evt_cb_register(dss_sync_evt_cb);

    s_dss_env.evt_handler         = evt_handler;
    s_dss_env.dss_role            = DSS_ROLE_SYNC_INVALID;
    s_dss_env.sync_status         = DSS_STATUS_CFG_READY;
    s_dss_env.char_mask           = 0x1fff;
    s_dss_env.is_busy_send        = false;
    s_dss_env.is_auto_calib_drift = false;
    s_dss_env.is_auto_enter_lp    = false;
    s_dss_env.is_in_lp            = false;
    s_dss_env.event_period        = 0;
    s_dss_env.auto_calib_timing   = 0;
    s_dss_env.sync_device_num     = 0;
    s_dss_env.sync_cnt            = 0;

    s_dss_env.start_hdl  = PRF_INVALID_HANDLE;

    s_dss_env.dss_gatts_db.shdl                  = &s_dss_env.start_hdl;
    s_dss_env.dss_gatts_db.uuid                  = s_dss_svc_uuid;
    s_dss_env.dss_gatts_db.attr_tab_cfg          = (uint8_t *)&(s_dss_env.char_mask);
    s_dss_env.dss_gatts_db.max_nb_attr           = DSS_IDX_NB;
    s_dss_env.dss_gatts_db.srvc_perm             = BLE_GATTS_SRVC_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128); 
    s_dss_env.dss_gatts_db.attr_tab_type         = BLE_GATTS_SERVICE_TABLE_TYPE_128;
    s_dss_env.dss_gatts_db.attr_tab.attr_tab_128 = dss_attr_tab;

    return ble_gatts_prf_add(&s_dss_env.dss_gatts_db, dss_ble_evt_handler);
}

sdk_err_t dss_sync_op_result_send(uint8_t conn_idx, dss_evt_type_t evt_type,  dss_rsp_id_t rsp_id)
{
    if (evt_type == DSS_EVT_SYNC_SELF_OR_PEER)
    {
        return dss_ctrl_pt_rsp_send(conn_idx, DSS_OP_ID_SYNC, rsp_id);
    }
    else if (evt_type == DSS_EVT_SYNC_CANCEL)
    {
        return dss_ctrl_pt_rsp_send(conn_idx, DSS_OP_ID_CANCEL_SYNC, rsp_id);
    }
    else if (evt_type == DSS_EVT_LP_ENTER)  
    {
        return dss_ctrl_pt_rsp_send(conn_idx, DSS_OP_ID_LP_ENTER, rsp_id);
    }
    else if (evt_type == DSS_EVT_SYNC_DESTROY)
    {
        return dss_ctrl_pt_rsp_send(conn_idx, DSS_OP_ID_SYNC_DESTROY, rsp_id);
    }
    else
    {
        return SDK_ERR_INVALID_PARAM;
    }
}


void dss_set_status(uint8_t conn_idx, dss_staus_t status)
{
    s_dss_env.sync_status = status;
}

void dss_set_sync_params(uint8_t conn_idx, bool is_auto_enter_lp, bool is_auto_calib_drift)
{
    s_dss_env.is_auto_enter_lp    = is_auto_enter_lp;
    s_dss_env.is_auto_calib_drift = is_auto_calib_drift;
}

void dss_set_lp_mode(uint8_t conn_idx, bool is_in_lp_mode)
{
    s_dss_env.is_in_lp = is_in_lp_mode;
}

void dss_sync_src_distribute(uint8_t conn_idx)
{
    dss_op_id_t  op_id;
    dss_rsp_id_t rsp_id = DSS_RSP_ID_SUCCESS;

    if (DSS_ROLE_SYNC_SOURCE == s_dss_env.dss_role && ble_sync_source_distribute(conn_idx))
    {
        rsp_id = DSS_RSP_ID_DISTR_SRC_FAIL;
    }

    op_id = DSS_OP_ID_SYNC;

    s_dss_env.sync_status = DSS_STATUS_CFG_READY;

    dss_ctrl_pt_rsp_send(s_dss_env.sync_cfg_conn_idx, op_id, rsp_id);
}

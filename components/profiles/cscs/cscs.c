/**
 *****************************************************************************************
 *
 * @file cscs.c
 *
 * @brief Cycling Speed and Cadence Profile Sensor implementation.
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
#include "cscs.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief Cycling Speed and Cadence Service Attributes Indexes. */
enum
{
    CSCS_IDX_SVC,

    CSCS_IDX_CSC_MEAS_CHAR,
    CSCS_IDX_CSC_MEAS_VAL,
    CSCS_IDX_CSC_MEAS_NTF_CFG,

    CSCS_IDX_CSC_FEAT_CHAR,
    CSCS_IDX_CSC_FEAT_VAL,

    CSCS_IDX_SENSOR_LOC_CHAR,
    CSCS_IDX_SENSOR_LOC_VAL,

    CSCS_IDX_CTRL_POINT_CHAR,
    CSCS_IDX_CTRL_POINT_VAL,
    CSCS_IDX_CTRL_POINT_IND_CFG,

    CSCS_IDX_NB
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Cycling Speed and Cadence Service environment variable. */
struct cscs_env_t
{
    cscs_init_t  cscs_init;                               /**< Cycling Speed and Cadence Service initialization variables. */
    uint16_t     start_hdl;                               /**< Cycling Speed and Cadence Service start handle. */
    bool         ctrl_pt_op_in_progress;                  /**< A previously triggered SC Control Point operation is still in progress. */
    bool         ctrl_pt_op_rsp_cplt;                     /**< A previously triggered SC Control Point operation response cplt. */
    uint16_t     meas_ntf_cfg[CSCS_CONNECTION_MAX];       /**< The configuration of CSC Measurement Notification which is configured by the peer devices. */
    uint16_t     ctrl_point_ind_cfg[CSCS_CONNECTION_MAX]; /**< The configuration of SC Control Point Notification which is configured by the peer devices. */
    ble_gatts_create_db_t     cscs_serv_db;               /**< Cycling Speed and Cadence Service DataBase. */
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */


static void      cscs_sc_ctrl_pt_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct cscs_env_t s_cscs_env;
static const uint8_t     s_cscs_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_CYCLING_SPEED_CADENCE);

/**@brief Full CSCS Database Description - Used to add attributes into the database. */
static const ble_gatts_attm_desc_t cscs_attr_tab[CSCS_IDX_NB]  =
{
    // Cycling Speed and Cadence Service Declaration
    [CSCS_IDX_SVC]              = {BLE_ATT_DECL_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // CSC Measurement Characteristic - Declaration
    [CSCS_IDX_CSC_MEAS_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // CSC Measurement Characteristic - Value
    [CSCS_IDX_CSC_MEAS_VAL]     = {BLE_ATT_CHAR_CSC_MEAS,
                                   BLE_GATTS_NOTIFY_PERM_UNSEC,
                                   BLE_GATTS_ATT_VAL_LOC_USER,
                                   CSCS_MEAS_VAL_LEN_MAX},
    // CSC Measurement Characteristic - Client Characteristic Configuration Descriptor
    [CSCS_IDX_CSC_MEAS_NTF_CFG] = {BLE_ATT_DESC_CLIENT_CHAR_CFG,
                                   BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                   0,
                                   0},

    // CS Feature Characteristic - Declaration
    [CSCS_IDX_CSC_FEAT_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // CSC Feature Characteristic - Value
    [CSCS_IDX_CSC_FEAT_VAL]     = {BLE_ATT_CHAR_CSC_FEAT,
                                   BLE_GATTS_READ_PERM_UNSEC,
                                   BLE_GATTS_ATT_VAL_LOC_USER,
                                   CSCS_FEAT_VAL_LEN_MAX},

    // Sensor Location Characteristic - Declaration
    [CSCS_IDX_SENSOR_LOC_CHAR]  = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Sensor Location Characteristic - Value
    [CSCS_IDX_SENSOR_LOC_VAL]   = {BLE_ATT_CHAR_SENSOR_LOC,
                                   BLE_GATTS_READ_PERM_UNSEC,
                                   BLE_GATTS_ATT_VAL_LOC_USER,
                                   CSCS_SENSOR_LOC_VAL_LEN_MAX},

    // SC Control Point Characteristic - Declaration
    [CSCS_IDX_CTRL_POINT_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // SC Control Point Characteristic - Value
    [CSCS_IDX_CTRL_POINT_VAL]     = {BLE_ATT_CHAR_SC_CNTL_PT,
                                     BLE_GATTS_WRITE_REQ_PERM_UNSEC | BLE_GATTS_INDICATE_PERM_UNSEC,
                                     BLE_GATTS_ATT_VAL_LOC_USER,
                                     CSCS_CTRL_PT_VAL_LEN_MAX},
    // SC Control Point Characteristic - Client Characteristic Configuration Descriptor
    [CSCS_IDX_CTRL_POINT_IND_CFG] = {BLE_ATT_DESC_CLIENT_CHAR_CFG,
                                     BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                     0,
                                     0},
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize Cycling Speed and Cadence service  create db in att
 *
 * @return Error code to know if profile initialization succeed or not.
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
static void   cscs_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t  cfm;
    uint8_t           handle    = p_param->handle;
    uint8_t           tab_index = prf_find_idx_by_handle(handle, 
                                                         s_cscs_env.start_hdl,
                                                         CSCS_IDX_NB,
                                                        (uint8_t *)&s_cscs_env.cscs_init.char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case CSCS_IDX_CSC_MEAS_NTF_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_cscs_env.meas_ntf_cfg[conn_idx];
            break;

        case CSCS_IDX_CSC_FEAT_VAL:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_cscs_env.cscs_init.feature;
            break;

        case CSCS_IDX_SENSOR_LOC_VAL:
            cfm.length = sizeof(uint8_t);
            cfm.value  = (uint8_t *)&s_cscs_env.cscs_init.sensor_location;
            break;
 
        case CSCS_IDX_CTRL_POINT_IND_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_cscs_env.ctrl_point_ind_cfg[conn_idx];
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
static void   cscs_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t          handle      = p_param->handle;
    uint16_t          tab_index   = 0;
    uint16_t          cccd_value  = 0;
    bool              ctrl_pt_evt = false;
    cscs_evt_t        event;
    ble_gatts_write_cfm_t cfm;

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_cscs_env.start_hdl,
                                        CSCS_IDX_NB,
                                        (uint8_t *)&s_cscs_env.cscs_init.char_mask);
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    event.evt_type = CSCS_EVT_INVALID;
    event.conn_idx = conn_idx;
 
    switch (tab_index)
    {
        case CSCS_IDX_CSC_MEAS_NTF_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ?\
                               CSCS_EVT_CSC_MEAS_NOTIFICATION_ENABLE :\
                               CSCS_EVT_CSC_MEAS_NOTIFICATION_DISABLE);
            s_cscs_env.meas_ntf_cfg[conn_idx] = cccd_value;
            break;

        case CSCS_IDX_CTRL_POINT_VAL:
            if (PRF_CLI_START_IND != s_cscs_env.ctrl_point_ind_cfg[conn_idx])
            {
                cfm.status = CSCS_ERROR_CCCD_INVALID;
                break;
            }
            else if (s_cscs_env.ctrl_pt_op_in_progress)
            {
                cfm.status = CSCS_ERROR_PROC_IN_PROGRESS;
            }
            else if (PRF_CLI_START_IND == s_cscs_env.ctrl_point_ind_cfg[conn_idx])
            {
                s_cscs_env.ctrl_pt_op_in_progress = true;
                ctrl_pt_evt = true;
            }
            break;

        case CSCS_IDX_CTRL_POINT_IND_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ?\
                               CSCS_EVT_CTRL_POINT_INDICATION_ENABLE :\
                               CSCS_EVT_CTRL_POINT_INDICATION_DISABLE);
            s_cscs_env.ctrl_point_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_write_cfm(conn_idx, &cfm);

    if (ctrl_pt_evt)
    {
        cscs_sc_ctrl_pt_handler(conn_idx, p_param->value, p_param->length);
    }
    else if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && CSCS_EVT_INVALID != event.evt_type && s_cscs_env.cscs_init.evt_handler)
    {
        s_cscs_env.cscs_init.evt_handler(&event);
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
static void cscs_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint16_t          tab_index  = 0;
    cscs_evt_t        event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_cscs_env.start_hdl,
                                        CSCS_IDX_NB,
                                        (uint8_t *)&s_cscs_env.cscs_init.char_mask);

    event.evt_type = CSCS_EVT_INVALID;
    event.conn_idx = conn_idx;
 
    switch (tab_index)
    {
        case CSCS_IDX_CSC_MEAS_NTF_CFG:
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ?\
                               CSCS_EVT_CSC_MEAS_NOTIFICATION_ENABLE :\
                               CSCS_EVT_CSC_MEAS_NOTIFICATION_DISABLE);
            s_cscs_env.meas_ntf_cfg[conn_idx] = cccd_value;
            break;

        case CSCS_IDX_CTRL_POINT_IND_CFG:
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ?\
                               CSCS_EVT_CTRL_POINT_INDICATION_ENABLE :\
                               CSCS_EVT_CTRL_POINT_INDICATION_DISABLE);
            s_cscs_env.ctrl_point_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }


    if (CSCS_EVT_INVALID != event.evt_type && s_cscs_env.cscs_init.evt_handler)
    {
        s_cscs_env.cscs_init.evt_handler(&event);
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
static void cscs_ntf_ind_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    cscs_evt_t  event;

    event.evt_type = CSCS_EVT_INVALID;
    event.conn_idx = conn_idx;

    if (s_cscs_env.cscs_init.evt_handler && SDK_SUCCESS == status)
    {
        if (BLE_GATT_NOTIFICATION == p_ntf_ind->type)
        {
            event.evt_type = CSCS_EVT_CSC_MEAS_SEND_CPLT;
        }
        else if (BLE_GATT_INDICATION == p_ntf_ind->type)
        {
            event.evt_type = CSCS_EVT_CTRL_POINT_RSP_CPLT;
            s_cscs_env.ctrl_pt_op_in_progress = false;
        }
        s_cscs_env.cscs_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the disconnection event.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] reason:   Reason of disconnection.
 *****************************************************************************************
 */
static void cscs_disconnect_evt_handler(uint8_t conn_idx, uint8_t reason)
{
    s_cscs_env.ctrl_pt_op_in_progress = false;
}

/**
 *****************************************************************************************
 * @brief SC Control Point Set Cumulative value handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void cscs_op_set_cumulative_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    cscs_evt_t  event;
    uint8_t     rsp[CSCS_CTRL_PT_RSP_LEN_MIN];

    rsp[0] = CSCS_CTRL_PT_OP_RSP_CODE;
    rsp[1] = CSCS_CTRL_PT_OP_SET_CUMUL_VAL;
    rsp[2] = CSCS_CTRL_PT_RSP_FAILED;

    if ((sizeof(uint32_t) == length) && \
        (s_cscs_env.cscs_init.feature & CSCS_FEAT_WHEEL_REVOLUTION_SUP_BIT) && \
        (s_cscs_env.cscs_init.evt_handler))
    {
        event.conn_idx = conn_idx;
        event.evt_type = CSCS_EVT_CUMUL_VAL_SET;
        event.p_data   = p_data;
        event.length   = length;
        s_cscs_env.cscs_init.evt_handler(&event);
    }
    else
    {
        cscs_ctrl_pt_rsp_send(conn_idx, rsp, CSCS_CTRL_PT_RSP_LEN_MIN);
    }
}

/**
 *****************************************************************************************
 * @brief SC Control Point Sensor Location update handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void cscs_op_sensor_loc_update_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    cscs_evt_t  event;
    uint8_t     rsp[CSCS_CTRL_PT_RSP_LEN_MIN];

    rsp[0] = CSCS_CTRL_PT_OP_RSP_CODE;
    rsp[1] = CSCS_CTRL_PT_OP_UPD_LOC;

    if (CSCS_SENSOR_LOC_SUP_NB <= p_data[0] || (sizeof(uint8_t) != length))
    {
        rsp[2] = CSCS_CTRL_PT_RSP_INVALID_PARAM;
        cscs_ctrl_pt_rsp_send(conn_idx, rsp, CSCS_CTRL_PT_RSP_LEN_MIN);
    }
    else if ((s_cscs_env.cscs_init.feature & CSCS_FEAT_MULTIPLE_SENSORS_BIT) && s_cscs_env.cscs_init.evt_handler)
    {
        event.conn_idx = conn_idx;
        event.evt_type = CSCS_EVT_SEBSOR_LOC_UPD;
        event.p_data   = p_data;
        event.length   = length;
        s_cscs_env.cscs_init.evt_handler(&event);
    }
    else
    {
        rsp[2] = CSCS_CTRL_PT_RSP_FAILED;
        cscs_ctrl_pt_rsp_send(conn_idx, rsp, CSCS_CTRL_PT_RSP_LEN_MIN);
    }
}

/**
 *****************************************************************************************
 * @brief SC Control Point Suppoted Sensor Location list request handler.
 *
 * @param[in] conn_idx: Connection index.
 *****************************************************************************************
 */
static void cscs_op_sup_sensor_loc_req_handler(uint8_t conn_idx)
{
    cscs_evt_t  event;
    uint8_t     rsp[CSCS_CTRL_PT_RSP_LEN_MIN + CSCS_SENSOR_LOC_SUP_NB];
    uint8_t     rsp_idx = CSCS_CTRL_PT_RSP_LEN_MIN;

    rsp[0] = CSCS_CTRL_PT_OP_RSP_CODE;
    rsp[1] = CSCS_CTRL_PT_OP_REQ_SUP_LOC;
    rsp[2] = CSCS_CTRL_PT_RSP_SUCCESS;

    if (s_cscs_env.cscs_init.feature & CSCS_FEAT_MULTIPLE_SENSORS_BIT)
    {
        event.conn_idx = conn_idx;
        event.evt_type = CSCS_EVT_SUP_SEBSOR_LOC_REQ;

        if (s_cscs_env.cscs_init.evt_handler)
        {
             s_cscs_env.cscs_init.evt_handler(&event);
        }

        for (uint8_t i = 0; i < CSCS_SENSOR_LOC_SUP_NB; i++)
        {
            rsp[rsp_idx++] = i;
        }
        cscs_ctrl_pt_rsp_send(conn_idx, rsp, CSCS_CTRL_PT_RSP_LEN_MIN + CSCS_SENSOR_LOC_SUP_NB);
    }
    else
    {
        rsp[2] = CSCS_CTRL_PT_RSP_FAILED;
        cscs_ctrl_pt_rsp_send(conn_idx, rsp, CSCS_CTRL_PT_RSP_LEN_MIN);
    }
}

/**
 *****************************************************************************************
 * @brief SC Control Point handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void cscs_sc_ctrl_pt_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    uint8_t rsp[CSCS_CTRL_PT_RSP_LEN_MIN];

    switch(p_data[0])
    {
        case CSCS_CTRL_PT_OP_SET_CUMUL_VAL:
            cscs_op_set_cumulative_handler(conn_idx, &p_data[1], length - 1);
            break;

        case CSCS_CTRL_PT_OP_START_CALIB:
            rsp[0] = CSCS_CTRL_PT_OP_RSP_CODE;
            rsp[1] = p_data[0];
            rsp[2] = CSCS_CTRL_PT_RSP_NOT_SUP;
            cscs_ctrl_pt_rsp_send(conn_idx, rsp, CSCS_CTRL_PT_RSP_LEN_MIN);
            break;

        case CSCS_CTRL_PT_OP_UPD_LOC:
            cscs_op_sensor_loc_update_handler(conn_idx, &p_data[1], length - 1);
            break;

        case CSCS_CTRL_PT_OP_REQ_SUP_LOC:
            cscs_op_sup_sensor_loc_req_handler(conn_idx);
            break;

        default:
            rsp[0] = CSCS_CTRL_PT_OP_RSP_CODE;
            rsp[1] = p_data[0];
            rsp[2] = CSCS_CTRL_PT_RSP_NOT_SUP;
            cscs_ctrl_pt_rsp_send(conn_idx, rsp, CSCS_CTRL_PT_RSP_LEN_MIN);
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Encoding a CSC Measurement.
 *
 * @param[in]  p_meas:           Pointer to CSC measurement value to be encoded.
 * @param[out] p_encoded_buffer: Buffer where the encoded data will be written.
 *
 * @return Length of encoded data.
 *****************************************************************************************
 */
static uint16_t csc_meas_value_encoded(cscs_meas_val_t *p_meas, uint8_t *p_encoded_buffer)
{
    uint8_t  flags  = 0;
    uint16_t length = 1;

    // Cumulative Wheel Revolutions and Last Wheel Event Time Fields
    if (s_cscs_env.cscs_init.feature & CSCS_FEAT_WHEEL_REVOLUTION_SUP_BIT)
    {
        if (p_meas->wheel_rev_data_present)
        {
            p_encoded_buffer[length++] = LO_UINT32_T(p_meas->cumulative_wheel_revs);
            p_encoded_buffer[length++] = L2_UINT32_T(p_meas->cumulative_wheel_revs);
            p_encoded_buffer[length++] = L3_UINT32_T(p_meas->cumulative_wheel_revs);
            p_encoded_buffer[length++] = HI_UINT32_T(p_meas->cumulative_wheel_revs);
            p_encoded_buffer[length++] = LO_U16(p_meas->last_wheel_event_time);
            p_encoded_buffer[length++] = HI_U16(p_meas->last_wheel_event_time);

            // Flags field
            flags |= CSCS_MEAS_FLAG_WHEEL_REVOLUTION_BIT;
        }
    }

    // Cumulative Crank Revolutions and Last Crank Event Time Fields
    if (s_cscs_env.cscs_init.feature & CSCS_FEAT_CRANK_REVOLUTION_SUP_BIT)
    {
        if (p_meas->crank_rev_data_present)
        {
            p_encoded_buffer[length++] = LO_U16(p_meas->cumulative_crank_revs);
            p_encoded_buffer[length++] = HI_U16(p_meas->cumulative_crank_revs);
            p_encoded_buffer[length++] = LO_U16(p_meas->last_crank_event_time);
            p_encoded_buffer[length++] = HI_U16(p_meas->last_crank_event_time);

            // Flags field
            flags |= CSCS_MEAS_FLAG_CRANK_REVOLUTION_BIT;
        }
    }

    p_encoded_buffer[0] = flags;

    return length;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
sdk_err_t cscs_measurement_send(uint8_t conn_idx, cscs_meas_val_t *p_meas)
{
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    uint8_t          encoded_csc_meas[CSCS_MEAS_VAL_LEN_MAX];
    uint16_t         length;
    ble_gatts_noti_ind_t csc_ntf;

    length = csc_meas_value_encoded(p_meas, encoded_csc_meas);

    if (PRF_CLI_START_NTF == s_cscs_env.meas_ntf_cfg[conn_idx])
    {
        csc_ntf.type   = BLE_GATT_NOTIFICATION;
        csc_ntf.handle = prf_find_handle_by_idx(CSCS_IDX_CSC_MEAS_VAL,
                                                s_cscs_env.start_hdl,
                                                (uint8_t *)&s_cscs_env.cscs_init.char_mask);
        csc_ntf.length = length;
        csc_ntf.value  = encoded_csc_meas;
        error_code     = ble_gatts_noti_ind(conn_idx, &csc_ntf);
    }

    return error_code;
}

sdk_err_t cscs_ctrl_pt_rsp_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    sdk_err_t        error_code = SDK_ERR_IND_DISABLED;
    ble_gatts_noti_ind_t ctrl_pt_rsp;

    if (PRF_CLI_START_IND == s_cscs_env.ctrl_point_ind_cfg[conn_idx])
    {
        ctrl_pt_rsp.type   = BLE_GATT_INDICATION;
        ctrl_pt_rsp.handle = prf_find_handle_by_idx(CSCS_IDX_CTRL_POINT_VAL,
                                                    s_cscs_env.start_hdl,
                                                    (uint8_t *)&s_cscs_env.cscs_init.char_mask);
        ctrl_pt_rsp.length = length;
        ctrl_pt_rsp.value  = p_data;
        error_code         = ble_gatts_noti_ind(conn_idx, &ctrl_pt_rsp);
    }

    return error_code;
}

sdk_err_t cscs_sensor_loc_update(cscs_sensor_loc_t sensor_loc)
{
    sdk_err_t   error_code = BLE_SUCCESS;

    if (s_cscs_env.cscs_init.feature & CSCS_FEAT_MULTIPLE_SENSORS_BIT)
    {
        s_cscs_env.cscs_init.sensor_location = sensor_loc;
    }
    else
    {
        error_code = SDK_ERR_DISALLOWED;
    }

    return error_code;
}
static void cscs_ble_evt_handler(const ble_evt_t *p_evt)
{
    if(NULL == p_evt)
    {
        return ;
    }

    switch(p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            cscs_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;
        case BLE_GATTS_EVT_WRITE_REQUEST:
            cscs_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;
        case BLE_GATTS_EVT_NTF_IND:
            cscs_ntf_ind_evt_handler(p_evt->evt.gatts_evt.index,p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;
        case BLE_GATTS_EVT_CCCD_RECOVERY:
            cscs_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;
        case BLE_GAPC_EVT_DISCONNECTED:
            cscs_disconnect_evt_handler(p_evt->evt.gapc_evt.index,p_evt->evt.gapc_evt.params.disconnected.reason);
            break;
        default:
            break;
    }
}
sdk_err_t cscs_service_init(cscs_init_t *p_cscs_init)
{
    if (NULL == p_cscs_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    if (p_cscs_init->feature & CSCS_FEAT_MULTIPLE_SENSORS_BIT)
    {
        p_cscs_init->char_mask |= CSCS_CHAR_SENSOR_LOC_SUP;
    }
    else
    {
        p_cscs_init->char_mask &= ~CSCS_CHAR_SENSOR_LOC_SUP;
    }

    memcpy(&s_cscs_env.cscs_init, p_cscs_init, sizeof(cscs_init_t));

    memset(&s_cscs_env.cscs_serv_db, 0, sizeof(ble_gatts_create_db_t));

    s_cscs_env.start_hdl                         = PRF_INVALID_HANDLE;
    s_cscs_env.cscs_serv_db.shdl                 = &s_cscs_env.start_hdl;
    s_cscs_env.cscs_serv_db.uuid                 = s_cscs_svc_uuid;
    s_cscs_env.cscs_serv_db.attr_tab_cfg         = (uint8_t *)&(s_cscs_env.cscs_init.char_mask);
    s_cscs_env.cscs_serv_db.max_nb_attr          = CSCS_IDX_NB;
    s_cscs_env.cscs_serv_db.srvc_perm            = 0;
    s_cscs_env.cscs_serv_db.attr_tab_type        = BLE_GATTS_SERVICE_TABLE_TYPE_16;
    s_cscs_env.cscs_serv_db.attr_tab.attr_tab_16 = cscs_attr_tab;
    return ble_gatts_prf_add(&s_cscs_env.cscs_serv_db, cscs_ble_evt_handler);
}

/**
 *****************************************************************************************
 *
 * @file gus.c
 *
 * @brief Goodix UART Service Implementation.
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
#include "gus.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief The UUIDs of GUS characteristics. */
#define GUS_SERVER_TX_UUID {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x02, 0x02, 0xED, 0xA6}
#define GUS_SERVER_RX_UUID {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x03, 0x02, 0xED, 0xA6}
#define GUS_FLOW_CTRL_UUID {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x04, 0x02, 0xED, 0xA6}

/**@brief Macros for conversion of 128bit to 16bit UUID. */
#define ATT_128_PRIMARY_SERVICE BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_PRIMARY_SERVICE)
#define ATT_128_CHARACTERISTIC  BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC)
#define ATT_128_CLIENT_CHAR_CFG BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DESC_CLIENT_CHAR_CFG)

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief Goodix UART Service Attributes Indexes. */
enum gus_attr_idx_t
{
    GUS_IDX_SVC,
    
    GUS_IDX_TX_CHAR,
    GUS_IDX_TX_VAL,
    GUS_IDX_TX_CFG,

    GUS_IDX_RX_CHAR,
    GUS_IDX_RX_VAL,

    GUS_IDX_FLOW_CTRL_CHAR,
    GUS_IDX_FLOW_CTRL_VAL,
    GUS_IDX_FLOW_CTRL_CFG,

    GUS_IDX_NB,
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Goodix UART Service environment variable. */
struct gus_env_t
{
    gus_init_t              gus_init;                               /**< Goodix UART Service initialization variables. */
    uint16_t                start_hdl;                              /**< Start handle of services */
    uint16_t                tx_ntf_cfg[GUS_CONNECTION_MAX];         /**< TX Characteristic Notification configuration of the peers. */
    uint16_t                flow_ctrl_ntf_cfg[GUS_CONNECTION_MAX];  /**< Flow Control Characteristic Notification configuration of the peers. */
    ble_gatts_create_db_t   gus_gatts_db;                          /**< Goodix UART Service attributs database. */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct gus_env_t s_gus_env;
static const uint16_t   s_char_mask = 0xFFFF;
static const uint8_t    s_gus_svc_uuid[] = {GUS_SERVICE_UUID};

/**@brief Full GUS Database Description which is used to add attributes into the ATT database. */
static const ble_gatts_attm_desc_128_t gus_attr_tab[GUS_IDX_NB] =
{
    // GUS service
    [GUS_IDX_SVC]            = {ATT_128_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // GUS TX Characteristic Declaration
    [GUS_IDX_TX_CHAR]        = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // GUS TX Characteristic Value
    [GUS_IDX_TX_VAL]         = {GUS_SERVER_TX_UUID, 
                                BLE_GATTS_NOTIFY_PERM_UNSEC,
                                (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                                GUS_MAX_DATA_LEN},
    // GUS TX Characteristic - Client Characteristic Configuration Descriptor
    [GUS_IDX_TX_CFG]         = {ATT_128_CLIENT_CHAR_CFG,
                                BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                0,
                                0},

    // GUS RX Characteristic Declaration
    [GUS_IDX_RX_CHAR]        = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // GUS RX Characteristic Value
    [GUS_IDX_RX_VAL]         = {GUS_SERVER_RX_UUID,
                                BLE_GATTS_WRITE_REQ_PERM_UNSEC | BLE_GATTS_WRITE_CMD_PERM_UNSEC,
                                (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                                GUS_MAX_DATA_LEN},

    // GUS FLOW_CTRL Characteristic Declaration
    [GUS_IDX_FLOW_CTRL_CHAR] = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // GUS FLOW_CTRL Characteristic Value
    [GUS_IDX_FLOW_CTRL_VAL]  = {GUS_FLOW_CTRL_UUID,
                                BLE_GATTS_NOTIFY_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                                GUS_MAX_DATA_LEN},
    // GUS FLOW_CTRL Characteristic - Client Characteristic Configuration Descriptor
    [GUS_IDX_FLOW_CTRL_CFG]  = {ATT_128_CLIENT_CHAR_CFG,
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
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Index of the connection.
 * @param[in] p_param:  Pointer to the parameters of the read request.
 *****************************************************************************************
 */
static void gus_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t cfm;
    uint16_t         handle    = p_param->handle;
    uint8_t          tab_index = 0;

    tab_index  = prf_find_idx_by_handle(handle, s_gus_env.start_hdl, GUS_IDX_NB, (uint8_t *)&s_char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case GUS_IDX_TX_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_gus_env.tx_ntf_cfg[conn_idx];
            cfm.status = BLE_SUCCESS;
            break;

        case GUS_IDX_FLOW_CTRL_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_gus_env.flow_ctrl_ntf_cfg[conn_idx];
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
static void gus_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint8_t           handle    = p_param->handle;
    uint8_t           tab_index = 0;
    uint16_t          flow_ctrl_state;
    uint16_t          cccd_value;
    gus_evt_t         event;
    ble_gatts_write_cfm_t cfm;

    tab_index      = prf_find_idx_by_handle(handle,s_gus_env.start_hdl, GUS_IDX_NB, (uint8_t *)&s_char_mask);
    event.conn_idx = conn_idx;
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    
    switch (tab_index)
    {
        case GUS_IDX_RX_VAL:
            event.evt_type       = GUS_EVT_RX_DATA_RECEIVED;
            event.p_data = (uint8_t *)p_param->value;
            event.length = p_param->length;
            break;

        case GUS_IDX_TX_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? GUS_EVT_TX_PORT_OPENED : GUS_EVT_TX_PORT_CLOSED;
            s_gus_env.tx_ntf_cfg[conn_idx] = cccd_value;
            break;

        case GUS_IDX_FLOW_CTRL_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? GUS_EVT_FLOW_CTRL_ENABLE : GUS_EVT_FLOW_CTRL_DISABLE;
            s_gus_env.flow_ctrl_ntf_cfg[conn_idx] = cccd_value;
            break;

        case GUS_IDX_FLOW_CTRL_VAL:
            flow_ctrl_state = p_param->value[0];

            if (GUS_FLOW_CTRL_STATE_OFF == flow_ctrl_state)
            {
                event.evt_type = GUS_EVT_TX_FLOW_OFF;
            }
            else if (GUS_FLOW_CTRL_STATE_ON == flow_ctrl_state)
            {
                event.evt_type = GUS_EVT_TX_FLOW_ON;
            }
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && GUS_EVT_INVALID != event.evt_type && s_gus_env.gus_init.evt_handler)
    {
        s_gus_env.gus_init.evt_handler(&event);
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
static void gus_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint8_t           tab_index = 0;
    gus_evt_t         event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index      = prf_find_idx_by_handle(handle,s_gus_env.start_hdl, GUS_IDX_NB, (uint8_t *)&s_char_mask);
    event.conn_idx = conn_idx;
    event.evt_type = GUS_EVT_INVALID;

    switch (tab_index)
    {
        case GUS_IDX_TX_CFG:
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? GUS_EVT_TX_PORT_OPENED : GUS_EVT_TX_PORT_CLOSED;
            s_gus_env.tx_ntf_cfg[conn_idx] = cccd_value;
            break;

        case GUS_IDX_FLOW_CTRL_CFG:
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? GUS_EVT_FLOW_CTRL_ENABLE : GUS_EVT_FLOW_CTRL_DISABLE;
            s_gus_env.flow_ctrl_ntf_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }

    if (GUS_EVT_INVALID != event.evt_type && s_gus_env.gus_init.evt_handler)
    {
        s_gus_env.gus_init.evt_handler(&event);
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
static void gus_ntf_ind_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    if (NULL != s_gus_env.gus_init.evt_handler)
    {
        gus_evt_t event;
        event.conn_idx = conn_idx;

        if (BLE_SUCCESS == status && BLE_GATT_NOTIFICATION == p_ntf_ind->type)
        {
            event.evt_type = GUS_EVT_TX_DATA_SENT;
            s_gus_env.gus_init.evt_handler(&event);
        }
    }
}

static void gus_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            gus_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            gus_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_NTF_IND:
            gus_ntf_ind_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            gus_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t gus_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    sdk_err_t            error_code = SDK_ERR_NTF_DISABLED;
    ble_gatts_noti_ind_t send_cmd;

    if (PRF_CLI_START_NTF == s_gus_env.tx_ntf_cfg[conn_idx])
    {
        // Fill in the parameter structure
        send_cmd.type   = BLE_GATT_NOTIFICATION;
        send_cmd.handle = prf_find_handle_by_idx(GUS_IDX_TX_VAL, s_gus_env.start_hdl, (uint8_t *)&s_char_mask);

        // Pack measured value in database
        send_cmd.length = length;
        send_cmd.value  = p_data;

        // Send notification to peer device
        error_code = ble_gatts_noti_ind(conn_idx, &send_cmd);
    }

    return error_code;
}

sdk_err_t gus_rx_flow_ctrl_set(uint8_t conn_idx, gus_flow_ctrl_state_t flow_ctrl)
{
    sdk_err_t            error_code = BLE_SUCCESS;
    ble_gatts_noti_ind_t send_cmd;

    if (PRF_CLI_START_NTF == s_gus_env.flow_ctrl_ntf_cfg[conn_idx])
    {
        // Fill in the parameter structure
        send_cmd.type   = BLE_GATT_NOTIFICATION;
        send_cmd.handle = prf_find_handle_by_idx(GUS_IDX_FLOW_CTRL_VAL, s_gus_env.start_hdl, (uint8_t *)&s_char_mask);

        // Pack measured value in database
        send_cmd.length = 1;
        send_cmd.value  = &flow_ctrl;

        // Send notification to peer device
        error_code      = ble_gatts_noti_ind(conn_idx, &send_cmd);
    }

    return error_code;
}

sdk_err_t gus_service_init(gus_init_t *p_gus_init)
{
    if (NULL == p_gus_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memcpy(&s_gus_env.gus_init, p_gus_init, sizeof(gus_init_t));

    memset(&s_gus_env.gus_gatts_db, 0, sizeof(ble_gatts_create_db_t));

    s_gus_env.start_hdl  = PRF_INVALID_HANDLE;
    s_gus_env.gus_gatts_db.shdl                  = &s_gus_env.start_hdl;
    s_gus_env.gus_gatts_db.uuid                  = s_gus_svc_uuid;
    s_gus_env.gus_gatts_db.attr_tab_cfg          = (uint8_t *)&s_char_mask;
    s_gus_env.gus_gatts_db.max_nb_attr           = GUS_IDX_NB;
    s_gus_env.gus_gatts_db.srvc_perm             = BLE_GATTS_SRVC_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128); 
    s_gus_env.gus_gatts_db.attr_tab_type         = BLE_GATTS_SERVICE_TABLE_TYPE_128;
    s_gus_env.gus_gatts_db.attr_tab.attr_tab_128 = gus_attr_tab;

    return ble_gatts_prf_add(&s_gus_env.gus_gatts_db, gus_ble_evt_handler);
}

uint16_t gus_service_start_handle_get(void)
{
    return s_gus_env.start_hdl;
}

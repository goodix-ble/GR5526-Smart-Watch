/**
 *****************************************************************************************
 *
 * @file hts.c
 *
 * @brief Health Thermometer Profile Sensor implementation.
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
#include "hts.h"
#include "ble_prf_types.h"
#include "utility.h"

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief Health Thermometer Service Attributes Indexes. */
enum
{
    HTS_IDX_SVC,

    HTS_IDX_TEM_MEAS_CHAR,
    HTS_IDX_TEM_MEAS_VAL,
    HTS_IDX_TEM_MEAS_IND_CFG,

    HTS_IDX_TEM_TYPE_CHAR,
    HTS_IDX_TEM_TYPE_VAL,

    HTS_IDX_INTM_TEM_CHAR,
    HTS_IDX_INTM_TEM_VAL,
    HTS_IDX_INTM_TEM_NTF_CFG,

    HTS_IDX_MEAS_INTERVAL_CHAR,
    HTS_IDX_MEAS_INTERVAL_VAL,
    HTS_IDX_MEAS_INTERVAL_IND_CFG,
    HTS_IDX_MEAS_INTERVAL_VRD_CFG,

    HTS_IDX_NB,
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Health Thermometer Service environment variable. */
struct hts_env_t
{
    hts_init_t                hts_init;                                   /**< Health Thermometer Service initialization variables. */
    uint16_t                  start_hdl;                                  /**< Health Thermometer Service start handle. */
    uint16_t                  meas_ind_cfg[HTS_CONNECTION_MAX];           /**< The configuration of Temperature Measurement Indication which is configured by the peer devices. */
    uint16_t                  intm_tem_ntf_cfg[HTS_CONNECTION_MAX];       /**< The configuration of Intermediate Temperature Notification which is configured by the peer devices. */
    uint16_t                  meas_interval_ind_cfg[HTS_CONNECTION_MAX];  /**< The configuration of Measurement Interval Indication which is configured by the peer devices. */
    ble_gatts_create_db_t     hts_serv_db;                                /**< Health Thermometer Service DataBase. */
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct hts_env_t s_hts_env;
static const uint8_t    s_hts_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_HEALTH_THERMOM);

/**@brief Full HTS Database Description - Used to add attributes into the database. */
static const ble_gatts_attm_desc_t hts_attr_tab[HTS_IDX_NB]  =
{
    // Health Thermometer Service Declaration
    [HTS_IDX_SVC]                   =  {BLE_ATT_DECL_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // Temperature Measurement Characteristic - Declaration
    [HTS_IDX_TEM_MEAS_CHAR]         =  {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Temperature Measurement Characteristic - Value
    [HTS_IDX_TEM_MEAS_VAL]          =  {BLE_ATT_CHAR_TEMPERATURE_MEAS,
                                        BLE_GATTS_INDICATE_PERM_UNSEC, 
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        HTS_TEM_MEAS_MAX_LEN},
    // Temperature Measurement Characteristic - Client Characteristic Configuration Descriptor
    [HTS_IDX_TEM_MEAS_IND_CFG]      =  {BLE_ATT_DESC_CLIENT_CHAR_CFG,
                                        BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                        0,
                                        0},

    // Temperature Type Characteristic - Declaration
    [HTS_IDX_TEM_TYPE_CHAR]         =  {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Temperature Measurement Characteristic - Value
    [HTS_IDX_TEM_TYPE_VAL]          =  {BLE_ATT_CHAR_TEMPERATURE_TYPE,
                                        BLE_GATTS_READ_PERM_UNSEC,
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        HTS_TEM_TYPE_MAX_LEN},

    // Intermediate Temperature Characteristic - Declaration
    [HTS_IDX_INTM_TEM_CHAR]         =  {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Intermediate Temperature Characteristic - Value
    [HTS_IDX_INTM_TEM_VAL]          =  {BLE_ATT_CHAR_INTERMED_TEMPERATURE,
                                        BLE_GATTS_NOTIFY_PERM_UNSEC,
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        HTS_INTM_TEM_MAX_LEN},
    // Intermediate Temperature Characteristic - Client Characteristic Configuration Descriptor
    [HTS_IDX_INTM_TEM_NTF_CFG]      =  {BLE_ATT_DESC_CLIENT_CHAR_CFG,
                                        BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                        0,
                                        0},

    // Measurement Interval Characteristic - Declaration
    [HTS_IDX_MEAS_INTERVAL_CHAR]    =  {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Measurement Interval Characteristic - Value
    [HTS_IDX_MEAS_INTERVAL_VAL]     =  {BLE_ATT_CHAR_MEAS_INTERVAL,
                                        BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_AUTH) | BLE_GATTS_INDICATE_PERM_UNSEC,
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        HTS_MEAS_INTERVAL_MAX_LEN},
    // Measurement Interval Characteristic - Client Characteristic Configuration Descriptor
    [HTS_IDX_MEAS_INTERVAL_IND_CFG] =  {BLE_ATT_DESC_CLIENT_CHAR_CFG,
                                        BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                        0,
                                        0},
    // Measurement Interval Characteristic - Valid Range Descriptor
    [HTS_IDX_MEAS_INTERVAL_VRD_CFG] =  {BLE_ATT_DESC_VALID_RANGE,
                                        BLE_GATTS_READ_PERM_UNSEC,
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        0},
};


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize health thermometer service  create db in att
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
static void hts_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t  cfm;
    uint16_t              meas_interval[2];
    uint8_t               handle    = p_param->handle;
    uint8_t               tab_index = prf_find_idx_by_handle(handle, 
                                                             s_hts_env.start_hdl,
                                                             HTS_IDX_NB,
                                                             (uint8_t *)&s_hts_env.hts_init.char_mask);

    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case HTS_IDX_TEM_MEAS_IND_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_hts_env.meas_ind_cfg[conn_idx];
            break;

        case HTS_IDX_TEM_TYPE_VAL:
            if (s_hts_env.hts_init.evt_handler)
            {
                hts_evt_t event;
                hts_read_characteristic_t characteristic = HTS_READ_CHAR_TEMP_TYPE;
                
                event.evt_type = HTS_EVT_READ_CHARACTERISTIC;
                event.p_data   = (uint8_t *)&characteristic;
                event.length   = sizeof(characteristic);
                s_hts_env.hts_init.evt_handler(&event);
            }
            cfm.length = sizeof(uint8_t);
            cfm.value  = (uint8_t *)&s_hts_env.hts_init.temp_type;
            break;
        
        case HTS_IDX_INTM_TEM_NTF_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_hts_env.intm_tem_ntf_cfg[conn_idx];
            break;

        case HTS_IDX_MEAS_INTERVAL_VAL:
            if (s_hts_env.hts_init.evt_handler)
            {
                hts_evt_t event;
                hts_read_characteristic_t characteristic = HTS_READ_CHAR_MEAS_INTL;
                
                event.evt_type = HTS_EVT_READ_CHARACTERISTIC;
                event.p_data   = (uint8_t *)&characteristic;
                event.length   = sizeof(characteristic);
                s_hts_env.hts_init.evt_handler(&event);
            }
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_hts_env.hts_init.meas_interval;
            break;

        case HTS_IDX_MEAS_INTERVAL_IND_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_hts_env.meas_interval_ind_cfg[conn_idx];
            break;

        case HTS_IDX_MEAS_INTERVAL_VRD_CFG:
            meas_interval[0] = s_hts_env.hts_init.min_meas_interval_sup;
            meas_interval[1] = s_hts_env.hts_init.max_meas_interval_sup;
            cfm.length = sizeof(uint32_t);
            cfm.value  = (uint8_t *)meas_interval;
            break;

        default :
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
static void hts_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t          handle     = p_param->handle;
    uint16_t          tab_index  = 0;;
    uint16_t          cccd_value = 0;
    hts_evt_t         event;
    ble_gatts_write_cfm_t cfm;

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_hts_env.start_hdl,
                                        HTS_IDX_NB,
                                        (uint8_t *)&s_hts_env.hts_init.char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case HTS_IDX_TEM_MEAS_IND_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ?\
                               HTS_EVT_TEM_MEAS_INDICATION_ENABLE :\
                               HTS_EVT_TEM_MEAS_INDICATION_DISABLE);
            event.p_data   = (uint8_t *)&s_hts_env.hts_init.meas_interval;
            event.length   = sizeof(uint16_t);
            s_hts_env.meas_ind_cfg[conn_idx] = cccd_value;
            break;
            
        case HTS_IDX_INTM_TEM_NTF_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ?\
                               HTS_EVT_INTM_TEM_NOTIFICATION_ENABLE :\
                               HTS_EVT_INTM_TEM_NOTIFICATION_DISABLE);
            s_hts_env.intm_tem_ntf_cfg[conn_idx] = cccd_value;
            break;
        
        case HTS_IDX_MEAS_INTERVAL_VAL:
            if (((le16toh(p_param->value) >= s_hts_env.hts_init.min_meas_interval_sup) && \
                (le16toh(p_param->value) <= s_hts_env.hts_init.max_meas_interval_sup)) || \
                (0 == le16toh(p_param->value)))
            {
                event.evt_type = HTS_EVT_MEAS_INTERVAL_UPDATE;
                event.p_data   = p_param->value;
                event.length   = sizeof(uint16_t);
                s_hts_env.hts_init.meas_interval = le16toh(p_param->value);
            }
            else
            {
                 cfm.status = 0x80;    // Out of Range
            }
            break;
        
        case HTS_IDX_MEAS_INTERVAL_IND_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ?\
                               HTS_EVT_MEAS_INTREVAL_INDICATION_ENABLE :\
                               HTS_EVT_MEAS_INTERVAL_INDICATION_DISABLE);
            s_hts_env.meas_interval_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && HTS_EVT_INVALID != event.evt_type && s_hts_env.hts_init.evt_handler)
    {
        s_hts_env.hts_init.evt_handler(&event);
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
static void hts_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint16_t          tab_index  = 0;;
    hts_evt_t         event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_hts_env.start_hdl,
                                        HTS_IDX_NB,
                                        (uint8_t *)&s_hts_env.hts_init.char_mask);

    switch (tab_index)
    {
        case HTS_IDX_TEM_MEAS_IND_CFG:
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ?\
                               HTS_EVT_TEM_MEAS_INDICATION_ENABLE :\
                               HTS_EVT_TEM_MEAS_INDICATION_DISABLE);
            s_hts_env.meas_ind_cfg[conn_idx] = cccd_value;
            break;
            
        case HTS_IDX_INTM_TEM_NTF_CFG:
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ?\
                               HTS_EVT_INTM_TEM_NOTIFICATION_ENABLE :\
                               HTS_EVT_INTM_TEM_NOTIFICATION_DISABLE);
            s_hts_env.intm_tem_ntf_cfg[conn_idx] = cccd_value;
            break;
        
        case HTS_IDX_MEAS_INTERVAL_IND_CFG:
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ?\
                               HTS_EVT_MEAS_INTREVAL_INDICATION_ENABLE :\
                               HTS_EVT_MEAS_INTERVAL_INDICATION_DISABLE);
            s_hts_env.meas_interval_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            event.evt_type = HTS_EVT_INVALID;
            break;
    }

    if (HTS_EVT_INVALID != event.evt_type && s_hts_env.hts_init.evt_handler)
    {
        s_hts_env.hts_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Function for encoding a Temperature Measurement.
 *
 * @param[in]  p_meas:           Pointer to temperature measurement value to be encoded.
 * @param[out] p_encoded_buffer: Buffer where the encoded data will be written.
 *
 * @return Length of encoded data.
  *****************************************************************************************
 */
static uint16_t hts_htm_encoded(hts_meas_val_t *p_meas, uint8_t *p_encoded_buffer)
{
    uint8_t flags           = 0;
    uint16_t length         = 1;
    uint32_t encoded_sfloat = 0;

    if (HTS_TEMPERATURE_CELCIUS == s_hts_env.hts_init.temperature_units)
    {
        p_meas->temp_convert_value.exponent = -2;
        p_meas->temp_convert_value.mantissa = p_meas->temp_original_value;
    }
    else
    {
        p_meas->temp_convert_value.exponent = -2;
        p_meas->temp_convert_value.mantissa = (32 * 100) + ((p_meas->temp_original_value * 9) / 5);
        flags |= HTS_MEAS_FLAG_TEM_UINTS_BIT;
    }

    encoded_sfloat             = ((p_meas->temp_convert_value.exponent << 24) & 0xFF000000) |
                                 ((p_meas->temp_convert_value.mantissa << 0)  & 0x00FFFFFF);
    p_encoded_buffer[length++] = LO_UINT32_T(encoded_sfloat);
    p_encoded_buffer[length++] = L2_UINT32_T(encoded_sfloat);
    p_encoded_buffer[length++] = L3_UINT32_T(encoded_sfloat);
    p_encoded_buffer[length++] = HI_UINT32_T(encoded_sfloat);

    if (s_hts_env.hts_init.time_stamp_present)
    {
        flags |= HTS_MEAS_FLAG_TIME_STAMP_BIT;
        p_encoded_buffer[length++] = LO_U16(p_meas->time_stamp.year);
        p_encoded_buffer[length++] = HI_U16(p_meas->time_stamp.year);
        p_encoded_buffer[length++] = p_meas->time_stamp.month;
        p_encoded_buffer[length++] = p_meas->time_stamp.day;
        p_encoded_buffer[length++] = p_meas->time_stamp.hour;
        p_encoded_buffer[length++] = p_meas->time_stamp.min;
        p_encoded_buffer[length++] = p_meas->time_stamp.sec;
    }

    if (0 == (s_hts_env.hts_init.char_mask & HTS_CHAR_TEM_TYPE_SUP))
    {
        flags |= HTS_MEAS_FLAG_TEM_TYPE_BIT;
        p_encoded_buffer[length++] = p_meas->temp_type;
    }

    p_encoded_buffer[0] = flags;
    return length;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
sdk_err_t hts_measurement_send(uint8_t conn_idx, hts_meas_val_t *p_meas)
{
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    uint8_t          encoded_hts_meas[HTS_TEM_MEAS_MAX_LEN];
    uint16_t         length;
    ble_gatts_noti_ind_t hts_ind;

    length = hts_htm_encoded(p_meas, encoded_hts_meas);
    if( HTS_TEMPERATURE_STABLE == p_meas->temp_meas_type)
    {
        if (PRF_CLI_START_IND == (s_hts_env.meas_ind_cfg[conn_idx] & PRF_CLI_START_IND))
        {
            hts_ind.type   = BLE_GATT_INDICATION;
            hts_ind.handle = prf_find_handle_by_idx(HTS_IDX_TEM_MEAS_VAL,
                                                    s_hts_env.start_hdl,
                                                    (uint8_t *)&s_hts_env.hts_init.char_mask);
            hts_ind.length = length;
            hts_ind.value  = encoded_hts_meas;
            error_code     = ble_gatts_noti_ind(conn_idx, &hts_ind);
        }
    }
    else
    {
        if (PRF_CLI_START_NTF == (s_hts_env.intm_tem_ntf_cfg[conn_idx] & PRF_CLI_START_NTF))
        {
            hts_ind.type   = BLE_GATT_NOTIFICATION;
            hts_ind.handle = prf_find_handle_by_idx(HTS_IDX_INTM_TEM_VAL,
                                                    s_hts_env.start_hdl,
                                                    (uint8_t *)&s_hts_env.hts_init.char_mask);
            hts_ind.length = length;
            hts_ind.value  = encoded_hts_meas;
            error_code     = ble_gatts_noti_ind(conn_idx, &hts_ind);
        }
    }
    return error_code;
}

sdk_err_t hts_measurement_interval_send(uint8_t conn_idx)
{
    sdk_err_t        error_code = SDK_ERR_IND_DISABLED;
    ble_gatts_noti_ind_t hts_ind;
    
    if (PRF_CLI_START_IND == (s_hts_env.meas_interval_ind_cfg[conn_idx] & PRF_CLI_START_IND))
    {
        hts_ind.type   = BLE_GATT_INDICATION;
        hts_ind.handle = prf_find_handle_by_idx(HTS_IDX_MEAS_INTERVAL_VAL,
                                                s_hts_env.start_hdl,
                                                (uint8_t *)&s_hts_env.hts_init.char_mask);
        hts_ind.length = sizeof(uint16_t);
        hts_ind.value  = (uint8_t *)&s_hts_env.hts_init.meas_interval;
        error_code     = ble_gatts_noti_ind(conn_idx, &hts_ind);
    }
    return error_code;
}

static void hts_ble_evt_handler(const ble_evt_t *p_evt)
{
    if(NULL == p_evt)
    {
        return ;
    }

    switch(p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            hts_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;
        case BLE_GATTS_EVT_WRITE_REQUEST:
            hts_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;
        case BLE_GATTS_EVT_NTF_IND:

            break;
        case BLE_GATTS_EVT_CCCD_RECOVERY:
            hts_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;
        default:
            break;
    }
}

sdk_err_t hts_service_init(hts_init_t *p_hts_init)
{
    if (NULL == p_hts_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memcpy(&s_hts_env.hts_init, p_hts_init, sizeof(hts_init_t));

    memset(&s_hts_env.hts_serv_db, 0, sizeof(ble_gatts_create_db_t));

    s_hts_env.start_hdl                        = PRF_INVALID_HANDLE;
    s_hts_env.hts_serv_db.shdl                 = &s_hts_env.start_hdl;
    s_hts_env.hts_serv_db.uuid                 = s_hts_svc_uuid;
    s_hts_env.hts_serv_db.attr_tab_cfg         = (uint8_t *)&(s_hts_env.hts_init.char_mask);
    s_hts_env.hts_serv_db.max_nb_attr          = HTS_IDX_NB;
    s_hts_env.hts_serv_db.srvc_perm            = 0;
    s_hts_env.hts_serv_db.attr_tab_type        = BLE_GATTS_SERVICE_TABLE_TYPE_16;
    s_hts_env.hts_serv_db.attr_tab.attr_tab_16 = hts_attr_tab;

    return ble_gatts_prf_add(&s_hts_env.hts_serv_db, hts_ble_evt_handler);
}


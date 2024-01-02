/**
 *******************************************************************************
 *
 * @file hrs.c
 *
 * @brief Heart Rate Profile Sensor implementation.
 *
 *******************************************************************************
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
 *******************************************************************************
 */
#include "hrs.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * DEFINES
 *******************************************************************************
 */
/** Value for control point characteristic. */
#define HRS_CTRL_POINT_ENERGY_EXP           0x01

/*
 * ENUMERATIONS
 *******************************************************************************
 */
/**@brief Heart Rate Service Attributes Indexes. */
enum hrs_attr_idx_t
{
    HRS_IDX_SVC,

    HRS_IDX_HR_MEAS_CHAR,
    HRS_IDX_HR_MEAS_VAL,
    HRS_IDX_HR_MEAS_NTF_CFG,

    HRS_IDX_BODY_SENSOR_LOC_CHAR,
    HRS_IDX_BODY_SENSOR_LOC_VAL,

    HRS_IDX_HR_CTNL_PT_CHAR,
    HRS_IDX_HR_CTNL_PT_VAL,

    HRS_IDX_NB,
};

/**@brief Heart Rate Service Measurement flag bit. */
enum hrs_flag_bit_t
{

    HRS_BIT_RATE_FORMAT              = 0x01,              /**< Heart Rate Value Format bit. */
    HRS_BIT_SENSOR_CONTACT_DETECTED  = 0x02,              /**< Sensor Contact Detected bit. */
    HRS_BIT_SENSOR_CONTACT_SUPPORTED = 0x04,              /**< Sensor Contact Supported bit. */
    HRS_BIT_ENERGY_EXPENDED_STATUS   = 0x08,              /**< Energy Expended Status bit. */
    HRS_BIT_INTERVAL                 = 0x10,              /**< RR-Interval bit. */
};

/*
 * STRUCT DEFINE
 *******************************************************************************
 */
/**@brief Heart Rate Measurement characteristic value structure. */
struct hr_meas_char_val_t
{
    bool     is_sensor_contact_detected;                  /**< True if sensor contact has been detected. */
    uint8_t  hr_meas_value[HRS_MEAS_MAX_LEN];             /**< The buffer for encoded value of Heart Rate Measurement characteristic. */
    uint16_t energy_expended;                             /**< The accumulated energy expended in kilo Joules since the last time it was reset. */
    uint16_t rr_interval[HRS_MAX_BUFFERED_RR_INTERVALS];  /**< The buffer for RR Interval measurements. */
    uint8_t  rr_interval_count;                           /**< The number of RR Interval measurements in the buffer. */
};

/**@brief Heart Rate Service environment variable. */
struct hrs_env_t
{
    hrs_init_t                hrs_init;                   /**< Heart Rate Service Init Value. */ 
    struct hr_meas_char_val_t hr_meas;                    /**< The value of Heart Rate Measurement characteristic. */ 
    uint16_t                  start_hdl;                  /**< Heart Rate Service start handle. */
    uint16_t                  ntf_cfg[HRS_CONNECTION_MAX];/**< The configuration of Heart Rate Measurement Notification which is configured by the peer devices. */
    ble_gatts_create_db_t     hrs_serv_db;                /**< Heart Rate Service DataBase. */
};

 /*
 * LOCAL FUNCTION DECLARATION
 *******************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *******************************************************************************
 */
static struct hrs_env_t s_hrs_env;
static const  uint8_t   s_hrs_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_HEART_RATE);

/**@brief Full HRS Database Description which is used to add attributes into the ATT database. */
static const ble_gatts_attm_desc_t hrs_attr_tab[HRS_IDX_NB] =
{
    // Heart Rate Service Declaration
    [HRS_IDX_SVC] = {BLE_ATT_DECL_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // HR Measurement Characteristic - Declaration
    [HRS_IDX_HR_MEAS_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // HR Measurement Characteristic - Value
    [HRS_IDX_HR_MEAS_VAL]     = {BLE_ATT_CHAR_HEART_RATE_MEAS, BLE_GATTS_NOTIFY_PERM_UNSEC,BLE_GATTS_ATT_VAL_LOC_USER, HRS_MEAS_MAX_LEN},
    // HR Measurement Characteristic - Client Characteristic Configuration Descriptor
    [HRS_IDX_HR_MEAS_NTF_CFG] = {BLE_ATT_DESC_CLIENT_CHAR_CFG, BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC, 0, 0},

    // Body Sensor Location Characteristic - Declaration
    [HRS_IDX_BODY_SENSOR_LOC_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0,            0},
    // Body Sensor Location Characteristic - Value
    [HRS_IDX_BODY_SENSOR_LOC_VAL]  = {BLE_ATT_CHAR_BODY_SENSOR_LOCATION,BLE_GATTS_READ_PERM_UNSEC, BLE_GATTS_ATT_VAL_LOC_USER, sizeof(uint8_t)},

    // Heart Rate Control Point Characteristic - Declaration
    [HRS_IDX_HR_CTNL_PT_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Heart Rate Control Point Characteristic - Value
    [HRS_IDX_HR_CTNL_PT_VAL]  = {BLE_ATT_CHAR_HEART_RATE_CNTL_POINT,BLE_GATTS_WRITE_REQ_PERM_UNSEC,0, sizeof(uint8_t)},
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 */

/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_param:  Pointer to the parameters of the read request.
 *****************************************************************************************
 */
static void hrs_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    uint8_t handle = p_param->handle;
    uint8_t tab_index = prf_find_idx_by_handle(handle, s_hrs_env.start_hdl,
                                               HRS_IDX_NB,
                                               (uint8_t *)&s_hrs_env.hrs_init.char_mask);

    ble_gatts_read_cfm_t cfm;
    hrs_evt_t        evt;

    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index) 
    {
        case HRS_IDX_HR_MEAS_VAL:
            cfm.length = HRS_MEAS_MAX_LEN;
            cfm.value  = s_hrs_env.hr_meas.hr_meas_value;
            break;

        case HRS_IDX_HR_MEAS_NTF_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)(&(s_hrs_env.ntf_cfg[conn_idx]));
            break;

        case HRS_IDX_BODY_SENSOR_LOC_VAL:
            if (s_hrs_env.hrs_init.evt_handler) 
            {
                evt.conn_idx = conn_idx;
                evt.evt_type = HRS_EVT_READ_BODY_SEN_LOCATION;
                s_hrs_env.hrs_init.evt_handler(&evt);
            }
            cfm.length = sizeof(uint8_t);
            cfm.value  = (uint8_t *)(&s_hrs_env.hrs_init.sensor_loc);
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
static void hrs_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t handle = p_param->handle;
    uint8_t tab_index = prf_find_idx_by_handle(handle, 
                                               s_hrs_env.start_hdl,
                                               HRS_IDX_NB,
                                               (uint8_t *)&s_hrs_env.hrs_init.char_mask);
    uint16_t          cccd_value;
    ble_gatts_write_cfm_t cfm;
    hrs_evt_t         evt;
    
    cfm.handle = handle;

    switch (tab_index) 
    {
        case HRS_IDX_HR_MEAS_NTF_CFG:
            cccd_value = le16toh(&p_param->value[0]);
            s_hrs_env.ntf_cfg[conn_idx] = cccd_value;

            if (s_hrs_env.hrs_init.evt_handler) 
            {
                evt.conn_idx = conn_idx;
                evt.evt_type = ((cccd_value == PRF_CLI_START_NTF) ?
                                 HRS_EVT_NOTIFICATION_ENABLED :
                                 HRS_EVT_NOTIFICATION_DISABLED);
                s_hrs_env.hrs_init.evt_handler(&evt);
            }

            cfm.status = BLE_SUCCESS;
            break;

        case HRS_IDX_HR_CTNL_PT_VAL:
            if (p_param->value[0] == HRS_CTRL_POINT_ENERGY_EXP) 
            {
                if (s_hrs_env.hrs_init.evt_handler) 
                {
                    evt.conn_idx = conn_idx;
                    evt.evt_type = HRS_EVT_RESET_ENERGY_EXPENDED;
                    s_hrs_env.hrs_init.evt_handler(&evt);
                }

                cfm.status = BLE_SUCCESS;
            }
            else 
            {
                cfm.status = 0x80;
            }
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
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
static void hrs_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    hrs_evt_t  evt;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    uint8_t   tab_index = prf_find_idx_by_handle(handle, 
                                                 s_hrs_env.start_hdl,
                                                 HRS_IDX_NB,
                                                (uint8_t *)&s_hrs_env.hrs_init.char_mask);

    switch (tab_index) 
    {
        case HRS_IDX_HR_MEAS_NTF_CFG:
            s_hrs_env.ntf_cfg[conn_idx] = cccd_value;

            if (s_hrs_env.hrs_init.evt_handler) 
            {
                evt.conn_idx = conn_idx;
                evt.evt_type = ((cccd_value == PRF_CLI_START_NTF) ?
                                 HRS_EVT_NOTIFICATION_ENABLED :
                                 HRS_EVT_NOTIFICATION_DISABLED);
                s_hrs_env.hrs_init.evt_handler(&evt);
            }
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Encode a Heart Rate Measurement.
 *
 * @param[in]  heart_rate        The heart rate measurement to be encoded.
 * @param[out] p_encoded_buffer  Pointer to the buffer where the encoded data will
 *                               be written.
 * @param[in]  is_energy_updated Indicate whether update energy expended.
 *
 * @return Size of encoded data.
 *****************************************************************************************
 */
static uint8_t hrs_hrm_encode(uint16_t heart_rate, uint8_t *p_encoded_buffer, bool is_energy_updated)
{
    uint8_t len   = 1;
    uint8_t temp  = 0;
    uint8_t flags = 0;
    struct hr_meas_char_val_t *p_meas_char;
    
    // Set sensor contact related flags
    if (s_hrs_env.hrs_init.is_sensor_contact_supported)
    {
        flags |= HRS_BIT_SENSOR_CONTACT_SUPPORTED;
    }
    if (s_hrs_env.hr_meas.is_sensor_contact_detected)
    {
        flags |= HRS_BIT_SENSOR_CONTACT_DETECTED;
    }

    p_meas_char = &s_hrs_env.hr_meas;
    // Encode heart rate measurement
    if (heart_rate > 0xff) 
    {
        flags |= HRS_BIT_RATE_FORMAT;
        p_encoded_buffer[len++] = LO_U16(heart_rate);
        p_encoded_buffer[len++] = HI_U16(heart_rate);
    } 
    else 
    {
        flags &= ~HRS_BIT_RATE_FORMAT;
        p_encoded_buffer[len++] = (uint8_t)heart_rate;
    }

    // Encode heart rate energy
    if ((s_hrs_env.hrs_init.char_mask & HRS_CHAR_ENGY_EXP_SUP) && is_energy_updated) 
    {
        flags |= HRS_BIT_ENERGY_EXPENDED_STATUS;
        p_encoded_buffer[len++] = LO_U16(p_meas_char->energy_expended);
        p_encoded_buffer[len++] = HI_U16(p_meas_char->energy_expended);
    }

    // Encode rr_interval values
    if (p_meas_char->rr_interval_count == 0) 
    {
        flags &= ~(HRS_BIT_INTERVAL);
    } 
    else 
    {
        flags |= HRS_BIT_INTERVAL;
        temp = ((HRS_MEAS_MAX_LEN - len) / 2);
        for (uint8_t i = 0; i < temp; i++) 
        {
            p_encoded_buffer[len++] = LO_U16(p_meas_char->rr_interval[i]);
            p_encoded_buffer[len++] = HI_U16(p_meas_char->rr_interval[i]);
        }
    }

    // Add flags
    p_encoded_buffer[0] = flags;

    return len;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void hrs_rr_interval_add(uint16_t rr_interval)
{
    struct hr_meas_char_val_t *p_meas_char = &s_hrs_env.hr_meas;

    p_meas_char->rr_interval_count  = 0;

    for (uint8_t i = HRS_MAX_BUFFERED_RR_INTERVALS - 1; i > 0; i--) 
    {
        p_meas_char->rr_interval[i] = p_meas_char->rr_interval[i - 1];
    }

    p_meas_char->rr_interval[0] = rr_interval;
    p_meas_char->rr_interval_count++;
    if (p_meas_char->rr_interval_count > HRS_MAX_BUFFERED_RR_INTERVALS) 
    {
        p_meas_char->rr_interval_count = HRS_MAX_BUFFERED_RR_INTERVALS;
    }
}

void hrs_energy_update(uint16_t energy)
{
    struct hr_meas_char_val_t *p_meas_char = &s_hrs_env.hr_meas;

    p_meas_char->energy_expended = energy;
}


void hrs_sensor_contact_detected_update(bool is_sensor_contact_detected)
{
    s_hrs_env.hr_meas.is_sensor_contact_detected = is_sensor_contact_detected;
}

void hrs_sensor_contact_supported_set(bool is_sensor_contact_supported)
{
    s_hrs_env.hrs_init.is_sensor_contact_supported = is_sensor_contact_supported;
}

void hrs_sensor_location_set(hrs_sensor_loc_t hrs_sensor_loc)
{
    s_hrs_env.hrs_init.sensor_loc = hrs_sensor_loc;
}


sdk_err_t hrs_heart_rate_measurement_send(uint8_t conn_idx, uint16_t heart_rate, bool is_energy_updated)
{
    sdk_err_t   error_code = SDK_ERR_NTF_DISABLED;
    struct hr_meas_char_val_t *p_meas_char = &s_hrs_env.hr_meas;
    uint8_t len = hrs_hrm_encode(heart_rate, p_meas_char->hr_meas_value,is_energy_updated);

    if (s_hrs_env.ntf_cfg[conn_idx] == PRF_CLI_START_NTF) 
    {
        ble_gatts_noti_ind_t hr_noti;

        hr_noti.type   = BLE_GATT_NOTIFICATION;
        hr_noti.handle = prf_find_handle_by_idx(HRS_IDX_HR_MEAS_VAL,
                                                s_hrs_env.start_hdl,
                                                (uint8_t *)&s_hrs_env.hrs_init.char_mask);
        hr_noti.length = len;
        hr_noti.value  = p_meas_char->hr_meas_value;

        error_code = ble_gatts_noti_ind(conn_idx, &hr_noti);
    }

    return error_code;
}

static void hrs_ble_evt_handler(const ble_evt_t *p_evt)
{
    if(NULL == p_evt)
    {
        return ;
    }

    switch(p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            hrs_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;
        case BLE_GATTS_EVT_WRITE_REQUEST:
            hrs_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;
        case BLE_GATTS_EVT_NTF_IND:

            break;
        case BLE_GATTS_EVT_CCCD_RECOVERY:
            hrs_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;
        default:
            break;
    }
}

sdk_err_t hrs_service_init(hrs_init_t *p_hrs_init)
{
    if (NULL == p_hrs_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memcpy(&s_hrs_env.hrs_init, p_hrs_init, sizeof(hrs_init_t));

    memset(&s_hrs_env.hrs_serv_db, 0, sizeof(ble_gatts_create_db_t));

    s_hrs_env.start_hdl                        = PRF_INVALID_HANDLE;
    s_hrs_env.hrs_serv_db.shdl                 = &s_hrs_env.start_hdl;
    s_hrs_env.hrs_serv_db.uuid                 = s_hrs_svc_uuid;
    s_hrs_env.hrs_serv_db.attr_tab_cfg         = (uint8_t *)&s_hrs_env.hrs_init.char_mask;
    s_hrs_env.hrs_serv_db.max_nb_attr          = HRS_IDX_NB;
    s_hrs_env.hrs_serv_db.srvc_perm            = 0;
    s_hrs_env.hrs_serv_db.attr_tab_type        = BLE_GATTS_SERVICE_TABLE_TYPE_16;
    s_hrs_env.hrs_serv_db.attr_tab.attr_tab_16 = hrs_attr_tab;

    return ble_gatts_prf_add(&s_hrs_env.hrs_serv_db, hrs_ble_evt_handler);
}

uint16_t hrs_service_start_handle_get(void)
{
    return s_hrs_env.start_hdl;
}

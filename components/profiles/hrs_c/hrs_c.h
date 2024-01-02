/**
 *****************************************************************************************
 *
 * @file hrs_c.h
 *
 * @brief Heart Rate Service Client API
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 */

/**
 * @defgroup BLE_SDK_HRS_C Heart Rate Service Client (HRS_C)
 * @{
 * @brief Heart Rate Service Client module.
 *
 * @details The Heart Rate Service Client contains the APIs and types, which can be used by the
 *          application to discovery of Heart Rate Service of peer and interact with it.
 *
 *          The application must provide an event handler to register, then call \ref hrs_client_init().
 *          After Heart Rate Service Client discoveries peer Heat Rate Service, application can call
 *          \ref hrs_c_heart_rate_meas_notify_set(), then will receive heart rate data from peer,
 *          also can call \ref hrs_c_sensor_loc_read() and \ref hrs_c_ctrl_point_set() to get sensor
 *          location information and reset energy.
 */

#ifndef __HRS_C_H__
#define __HRS_C_H__

#include "gr_includes.h"
#include "ble_prf_types.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup HRS_C_MACRO Defines
 * @{
 */
#define HRS_C_CONNECTION_MAX                10                              /**< Maximum number of HRS Client connections. */
#define HRS_C_RR_INTERVALS_NUM_MAX          9                               /**< Maximum number of RR intervals in HRS notifications.  */
#define HRS_C_CTRL_POINT_ENERGY_EXP         0x01                            /**< Value for control point characteristic. */
/** @} */

/**
 * @defgroup HRS_C_ENUM Enumerations
 * @{
 */
/**@brief Heart Rate Service Client event type. */
typedef enum
{
    HRS_C_EVT_INVALID,                    /**< HRS Client invalid event. */
    HRS_C_EVT_DISCOVERY_COMPLETE,         /**< HRS Client has found HRS service and its characteristics. */
    HRS_C_EVT_DISCOVERY_FAIL,             /**< HRS Client found HRS service failed because of invalid operation or no found at the peer. */
    HRS_C_EVT_HR_MEAS_NTF_SET_SUCCESS,    /**< HRS Client has set Notification of Heart Rate Measure characteristic. */
    HRS_C_EVT_HR_MEAS_VAL_RECEIVE,        /**< HRS Client has received Heart Rate Measure value notification from peer. */
    HRS_C_EVT_SENSOR_LOC_READ_RSP,        /**< HRS Client has received Sensor Location Value read response. */
    HRS_C_EVT_CTRL_POINT_SET,             /**< HRS Client has set Control Point completely. */
    HRS_C_EVT_WRITE_OP_ERR,               /**< Error occured when HRS Client writen to peer. */
} hrs_c_evt_type_t;

/**@brief Heart Rate Service Measurement flag bit. */
typedef enum
{

    HRS_C_BIT_RATE_FORMAT              = 0x01,              /**< Heart Rate Value Format bit. */
    HRS_C_BIT_SENSOR_CONTACT_DETECTED  = 0x02,              /**< Sensor Contact Detected bit. */
    HRS_C_BIT_SENSOR_CONTACT_SUPPORTED = 0x04,              /**< Sensor Contact Supported bit. */
    HRS_C_BIT_ENERGY_EXPENDED_STATUS   = 0x08,              /**< Energy Expended Status bit. */
    HRS_C_BIT_INTERVAL                 = 0x10,              /**< RR-Interval bit. */
} hrs_c_flag_bit_t;

/**@brief Values for sensor location. */
typedef enum
{
    HRS_C_SENS_LOC_OTHER,             /**< The sensor location is other. */
    HRS_C_SENS_LOC_CHEST,             /**< The sensor location is the chest. */
    HRS_C_SENS_LOC_WRIST,             /**< The sensor location is the wrist. */
    HRS_C_SENS_LOC_FINGER,            /**< The sensor location is the finger. */
    HRS_C_SENS_LOC_HAND,              /**< The sensor location is the hand. */
    HRS_C_SENS_LOC_EARLOBE,           /**< The sensor location is the earlobe. */
    HRS_C_SENS_LOC_FOOT,              /**< The sensor location is the foot. */
} hrs_c_sensor_loc_t;
/** @} */

/**
 * @defgroup HRS_C_STRUCT Structures
 * @{
 */
/**@brief Heart Rate Measurement characteristic value structure. */
typedef struct
{
    bool     is_sensor_contact_detected;                  /**< True if sensor contact has been detected. */
    uint16_t hr_value;                                    /**< Heart Rate Value. */
    uint8_t  rr_intervals_num;                            /**< Number of RR intervals. */
    float    rr_intervals[HRS_C_RR_INTERVALS_NUM_MAX];    /**< RR intervals. */
    uint16_t energy_expended;                             /**< The accumulated energy expended in kilo Joules since the last time it was reset. */
} hrs_c_hr_meas_t;

/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t hrs_srvc_start_handle;      /**< HRS Service start handle. */
    uint16_t hrs_srvc_end_handle;        /**< HRS Service end handle. */
    uint16_t hrs_hr_meas_handle;         /**< HRS Heart Rate Measurement characteristic Value handle which has been got from peer. */
    uint16_t hrs_hr_meas_cccd_handle;    /**< HRS CCCD handle of Heart Rate Measurement characteristic which has been got from peer. */
    uint16_t hrs_sensor_loc_handle;      /**< HRS Sensor Location characteristic Value handle which has been got from peer. */
    uint16_t hrs_ctrl_point_handle;      /**< HRS Control Point characteristic Value handle which has been got from peer. */
} hrs_c_handles_t;

/**@brief Heart Rate Service Client event. */
typedef struct
{
    uint8_t                 conn_idx;            /**< The connection index. */
    hrs_c_evt_type_t        evt_type;            /**< HRS Client event type. */
    union
    {
        hrs_c_hr_meas_t     hr_meas_buff;        /**< Buffer of heart rate measurement value. */
        hrs_c_sensor_loc_t  sensor_loc;          /**< Sensor location. */
    } value;                                     /**< Decoded result of value received. */
} hrs_c_evt_t;
/** @} */

/**
 * @defgroup HRS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief Heart Rate Service Client event handler type. */
typedef void (*hrs_c_evt_handler_t)(hrs_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup HRS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register HRS Client event handler.
 *
 * @param[in] evt_handler: Heart Rate Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t hrs_client_init(hrs_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery Heart Rate Service on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t hrs_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer Heart Rate Measurement characteristic notify.
 *
 * @param[in] conn_idx:  Index of connection.
 * @param[in] is_enable: True or false.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t hrs_c_heart_rate_meas_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Read Sensor Location characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t hrs_c_sensor_loc_read(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Set Control Point characteristic value.
 *
 * @param[in] conn_idx:   Index of connection.
 * @param[in] ctrl_value: Value of control point.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t hrs_c_ctrl_point_set(uint8_t conn_idx, uint16_t ctrl_value);
/** @} */

#endif
/** @} */
/** @} */


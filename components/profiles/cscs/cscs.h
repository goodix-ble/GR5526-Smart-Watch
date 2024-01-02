/**
 *****************************************************************************************
 *
 * @file cscs.h
 *
 * @brief Cycling Speed and Cadence Service API.
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
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_CSCS Cycling Speed and Cadence Service (CSCS)
 * @{
 * @brief Definitions and prototypes for the CSCS interface.
 *
 * @details The Cycling Speed and Cadence (CSC) Service exposes speed-related data and/or cadence-related data
 *          while using the Cycling Speed and Cadence sensor(Server). This module implements the Cycling Speed
 *          and Cadence Service with CSC Measurement, CSC Feature, Sensor Location and SC Control Point characteristics.
 *
 *          After \ref cscs_init_t variable is initialized, the application must call \ref cscs_service_init()
 *          to add the Cycling Speed and Cadence Service and CSC Measurement, CSC Feature, Sensor Location and 
 *          SC Control Point characteristics to the BLE Stack database according to \ref cscs_init_t.char_mask.
 */

#ifndef __CSCS_H__
#define __CSCS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup CSCS_MACRO Defines
 * @{
 */
#define CSCS_CONNECTION_MAX              10                             /**< Maximum number of CSCS connections. */
#define CSCS_MEAS_VAL_LEN_MAX            20                                                     /**< Maximum length of CSC Measurment value. */
#define CSCS_FEAT_VAL_LEN_MAX            2                                                      /**< Maximum length of CSC Feature value. */
#define CSCS_SENSOR_LOC_VAL_LEN_MAX      1                                                      /**< Maximum length of Sensor Location value. */
#define CSCS_CTRL_PT_RSP_LEN_MIN         3                                                      /**< Mimimum length of SC Control Point response value. */
#define CSCS_CTRL_PT_VAL_LEN_MAX         (CSCS_CTRL_PT_RSP_LEN_MIN + CSCS_SENSOR_LOC_SUP_NB)    /**< Maximum length of SC Control Point value. */

#define CSCS_ERROR_PROC_IN_PROGRESS       0x80                                                  /**< Error code: A previously triggered SC Control Point operation is still in progress. */
#define CSCS_ERROR_CCCD_INVALID           0x81                                                  /**< Error code: The Client Characteristic Configuration descriptor is not configured. */

/**
 * @defgroup CSCS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref cscs_init_t.char_mask.
 */
#define CSCS_CHAR_MANDATORY               0x003f     /**< Bit mask for mandatory characteristic in CSCS. */
#define CSCS_CHAR_SENSOR_LOC_SUP          0x00c0     /**< Bit mask for Sensor Location characteristic that is optional. */
#define CSCS_CHAR_SC_CTRL_POINT           0x0700     /**< Bit mask for SC Control Point characteristic that is optional. */
#define CSCS_CHAR_FULL                    0x07ff     /**< Bit mask of the full characteristic. */
/** @} */

/**
 * @defgroup CSCS_MEAS_FLAG_BIT Measurement Flag Bits
 * @{
 * @brief Cycling Speed and Cadence Measurement Flags.
 */
#define CSCS_MEAS_FLAG_WHEEL_REVOLUTION_BIT           (0x01 << 0)     /**< Flag bit for Wheel Revolution Data Present. */
#define CSCS_MEAS_FLAG_CRANK_REVOLUTION_BIT           (0x01 << 1)     /**< Flag bit for Crank Revolution Data Present. */
/** @} */

/**
 * @defgroup CSCS_FEAT_BIT Feature Bits
 * @{
 * @brief Cycling Speed and Cadence Service feature bits.
 */
#define CSCS_FEAT_WHEEL_REVOLUTION_SUP_BIT          (0x01 << 0)     /**< Bit for Wheel Revolution Data Supported. */
#define CSCS_FEAT_CRANK_REVOLUTION_SUP_BIT          (0x01 << 1)     /**< Bit for Crank Revolution Data Supported. */
#define CSCS_FEAT_MULTIPLE_SENSORS_BIT              (0x01 << 2)     /**< Bit for Multiple Sensor Locations Supported. */
#define CSCS_FEAR_FULL_BIT                          (0x07)          /**< Bit for all CSC features Supported. */
/** @} */
/** @} */

/**
 * @defgroup CSCS_ENUM Enumerations
 * @{
 */
/**@brief Cycling Speed and Cadence Service Sensor Location. */
typedef enum
{
    CSCS_SENSOR_LOC_OTHER,          /**< Sensor location: other. */
    CSCS_SENSOR_LOC_SHOE_TOP,       /**< Sensor location: top of shoe. */
    CSCS_SENSOR_LOC_SHOE_IN,        /**< Sensor location: inside of shoe. */
    CSCS_SENSOR_LOC_HIP,            /**< Sensor location: hip. */
    CSCS_SENSOR_LOC_FRONT_WHEEL,    /**< Sensor location: front wheel. */
    CSCS_SENSOR_LOC_LEFT_PEDAL,     /**< Sensor location: left pedal. */
    CSCS_SENSOR_LOC_RIGHT_PEDAL,    /**< Sensor location: right pedal. */
    CSCS_SENSOR_LOC_FRONT_HUB,      /**< Sensor location: front hub. */
    CSCS_SENSOR_LOC_SUP_NB          /**< Number of sensor location. */
} cscs_sensor_loc_t;

/**@brief Cycling Speed and Cadence Service Control Point Operation Code.*/
typedef enum
{
    CSCS_CTRL_PT_OP_RESERVED,         /**< Reserved for future use. */
    CSCS_CTRL_PT_OP_SET_CUMUL_VAL,    /**< Set Cumulative value Operation Code.*/
    CSCS_CTRL_PT_OP_START_CALIB,      /**< Start Sensor Calibration Operation Code.*/
    CSCS_CTRL_PT_OP_UPD_LOC,          /**< Update Sensor Location Operation Code.*/
    CSCS_CTRL_PT_OP_REQ_SUP_LOC,      /**< Request Supported Sensor Locations Operation Code.*/
    CSCS_CTRL_PT_OP_RSP_CODE = 0x10,  /**< Response code. */
} cscs_ctrl_pt_op_code_t;

/**@brief Cycling Speed and Cadence Service Control Point Response value.*/
typedef enum
{
    CSCS_CTRL_PT_RSP_RESERVED,        /**< Reserved value. */
    CSCS_CTRL_PT_RSP_SUCCESS,         /**< Operation Success. */
    CSCS_CTRL_PT_RSP_NOT_SUP,         /**< Operation Code Not Supported. */
    CSCS_CTRL_PT_RSP_INVALID_PARAM,   /**< Invalid Parameter. */
    CSCS_CTRL_PT_RSP_FAILED           /**< Operation Failed. */
} cscs_ctrl_pt_rsp_t;

/**@brief Cycling Speed and Cadence Service event type.*/
typedef enum
{
    CSCS_EVT_INVALID,                                  /**< Indicate that invalid event. */
    CSCS_EVT_CSC_MEAS_NOTIFICATION_ENABLE,             /**< Indicate that CSC Measurement notification has been enabled. */
    CSCS_EVT_CSC_MEAS_NOTIFICATION_DISABLE,            /**< Indicate that CSC Measurement notification has been disabled. */
    CSCS_EVT_CTRL_POINT_INDICATION_ENABLE,             /**< Indicate that SC Control Point indication has been enabled. */
    CSCS_EVT_CTRL_POINT_INDICATION_DISABLE,            /**< Indicate that SC Control Point indication has been disabled. */
    CSCS_EVT_CSC_MEAS_SEND_CPLT,                       /**< Indicate that CSC Measurement has been notified. */
    CSCS_EVT_CUMUL_VAL_SET,                            /**< Indicate that Wheel Revolution Data needs to be set. */
    CSCS_EVT_SEBSOR_CALIBRATION,                       /**< Indicate that Sensor calibration procedure should be initiated. */
    CSCS_EVT_SEBSOR_LOC_UPD,                           /**< Indicate that Sensor Location needs to be reset. */
    CSCS_EVT_SUP_SEBSOR_LOC_REQ,                       /**< Indicate that request supported sensor location list. */
    CSCS_EVT_CTRL_POINT_RSP_CPLT                       /**< Indicate that SC Control Point response has been indicated. */
} cscs_evt_type_t;
/** @} */

/**
 * @defgroup CSCS_STRUCT Structures
 * @{
 */
/**@brief Cycling Speed and Cadence Service event. */
typedef struct
{
    cscs_evt_type_t evt_type;     /**< The CSCS event type. */
    uint8_t         conn_idx;     /**< The index of the connection. */
    const uint8_t  *p_data;       /**< Pointer to event data. */
    uint16_t        length;       /**< Length of event data. */
} cscs_evt_t;
/** @} */

/**
 * @defgroup CSCS_TYPEDEF Typedefs
 * @{
 */
/**@brief Cycling Speed and Cadence Service event handler type.*/
typedef void (*cscs_evt_handler_t)(cscs_evt_t *p_evt);
/** @} */

/**
 * @defgroup CSCS_STRUCT Structures
 * @{
 */
/**@brief Cycling Speed and Cadence Measurement Character value structure. */
typedef struct
{
    bool        wheel_rev_data_present;         /**< If Wheel Revolution Data is present. */
    bool        crank_rev_data_present;         /**< If Crank Revolution Data is present. */
    uint32_t    cumulative_wheel_revs;          /**< Cumulative Wheel Revolutions. */
    uint16_t    last_wheel_event_time;          /**< Last Wheel Event Time. */
    uint16_t    cumulative_crank_revs;          /**< Cumulative Crank Revolutions. */
    uint16_t    last_crank_event_time;          /**< Last Crank Event Time. */
} cscs_meas_val_t;

/**@brief Cycling Speed and Cadence Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    cscs_evt_handler_t     evt_handler;                  /**< Cycling Speed and Cadence Service event handler. */
    uint16_t               char_mask;                    /**< Initial mask of supported characteristics, and configured with \ref CSCS_CHAR_MASK. */
    cscs_sensor_loc_t      sensor_location;              /**< Initial sensor location. */
    uint16_t               feature;                      /**< Initial value for features. */
} cscs_init_t;
/** @} */

/**
 * @defgroup RSCS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Cycling Speed and Cadence Service instance and add in the DB.
 *
 * @param[in] p_cscs_init: Pointer to cscs_init_t Service initialization variable
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t cscs_service_init(cscs_init_t *p_cscs_init);

/**
 *****************************************************************************************
 * @brief Send CSC measurement if notification has been enabled.
 *
 * @param[in] conn_idx: Connnection index.
 * @param[in] p_meas:   The pointer to new Cycling Speed and Cadence measurement.
 *
 * @return Result of notify value
 *****************************************************************************************
 */
sdk_err_t cscs_measurement_send(uint8_t conn_idx, cscs_meas_val_t *p_meas);

/**
 *****************************************************************************************
 * @brief Send SC Control Point responce if indication has been enabled.
 *
 * @param[in] conn_idx: Connnection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *
 * @return Result of indicate value
 *****************************************************************************************
 */
sdk_err_t cscs_ctrl_pt_rsp_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Update Sensor Location if Multiple Sensor Locations Supported \ref cscs_init_t.feature.
 *
 * @param[in] sensor_loc: New sensor location.
 *
 * @return Result of update.
 *****************************************************************************************
 */
sdk_err_t cscs_sensor_loc_update(cscs_sensor_loc_t sensor_loc);
/** @} */

#endif
/** @} */
/** @} */


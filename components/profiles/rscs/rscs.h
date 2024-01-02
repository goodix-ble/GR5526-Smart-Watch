/**
 *****************************************************************************************
 *
 * @file rscs.h
 *
 * @brief Running Speed and Cadence Service API.
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
 * @defgroup BLE_SDK_RSCS Running Speed and Cadence Service (RSCS)
 * @{
 * @brief Definitions and prototypes for the RSCS interface.
 *
 * @details The Running Speed and Cadence (RSC) Service exposes speed, cadence and other data related to
 *          fitness applications such as the stride length and the total distance the user has traveled 
 *          while using the Running Speed and Cadence sensor (Server). This module implements the Running
 *          Speed and Cadence Service with RSC Measurement, RSC Feature, Sensor Location and SC Control
 *          Point characteristics.
 *
 *          After \ref rscs_init_t variable is initialized, the application must call \ref rscs_service_init()
 *          to add the Running Speed and Cadence Service and RSC Measurement, RSC Feature, Sensor Location and 
 *          SC Control Point characteristics to the BLE Stack database according to \ref rscs_init_t.char_mask.
 */

#ifndef __RSCS_H__
#define __RSCS_H__

#include "gr_includes.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup RSCS_MACRO Defines
 * @{
 */
#define RSCS_CONNECTION_MAX               10                                                    /**< Maximum number of Running Speed and Cadence Service connections. */
#define RSCS_MEAS_VAL_LEN_MAX             20                                                    /**< Maximum length of RSC measurement value. */
#define RSCS_FEAT_VAL_LEN_MAX             2                                                     /**< Maximum length of RSC Feature value. */
#define RSCS_SENSOR_LOC_VAL_LEN_MAX       1                                                     /**< Maximum length of RSC Sensor Location value. */
#define RSCS_CTRL_PT_RSP_LEN_MIN          3                                                     /**< Mimimum length of SC Control Point responce value. */
#define RSCS_CTRL_PT_VAL_LEN_MAX          (RSCS_CTRL_PT_RSP_LEN_MIN + RSCS_SENSOR_LOC_SUP_NB)   /**< Maximum length of SC Control Point value. */

#define RSCS_ERROR_PROC_IN_PROGRESS       0x80       /**< Error code: A previously triggered SC Control Point operation is still in progress. */
#define RSCS_ERROR_CCCD_INVALID           0x81       /**< Error code: The Client Characteristic Configuration descriptor is not configured. */

/**
 * @defgroup RSCS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref rscs_init_t.char_mask.
 */
#define RSCS_CHAR_MANDATORY               0x003f     /**< Bit mask for mandatory characteristic in RSCS. */
#define RSCS_CHAR_SENSOR_LOC_SUP          0x00c0     /**< Bit mask for Sensor Location characteristic that is optional. */
#define RSCS_CHAR_SC_CTRL_POINT           0x0700     /**< Bit mask for SC Control Point characteristic that is optional. */
#define RSCS_CHAR_FULL                    0x07ff     /**< Bit mask of the full characteristic. */
/** @} */

/**
 * @defgroup RSCS_MEAS_FLAG_BIT Measurement Flag Bits
 * @{
 * @brief Running Speed and Cadence Measurement Flags.
 */
#define RSCS_MEAS_FLAG_INST_STRIDE_LEN_BIT           (0x01 << 0)     /**< Flag bit for Instantaneous Stride Length Measurement. */
#define RSCS_MEAS_FLAG_TOTAL_DISTANCE_BIT            (0x01 << 1)     /**< Flag bit for Total Distance Measurement. */
#define RSCS_MEAS_FLAG_RUNNING_OR_WALKING_BIT        (0x01 << 2)     /**< Flag bit for Running or Walking. */
/** @} */

/**
 * @defgroup RSCS_FEAT_BIT Feature Bits
 * @{
 * @brief Running Speed and Cadence Service feature bits.
 */
#define RSCS_FEAT_INSTANT_STRIDE_LEN_BIT             (0x01 << 0)     /**< Bit for Instantaneous Stride Length Measurement Supported. */
#define RSCS_FEAT_TOTAL_DISTANCE_BIT                 (0x01 << 1)     /**< Bit for Total Distance Measurement Supported. */
#define RSCS_FEAT_RUNNING_OR_WALKING_STATUS_BIT      (0x01 << 2)     /**< Bit for Running or Walking Status Supported. */
#define RSCS_FEAT_CALIBRATION_PROCEDURE_BIT          (0x01 << 3)     /**< Bit for Calibration Procedure Supported. */
#define RSCS_FEAT_MULTIPLE_SENSORS_BIT               (0x01 << 4)     /**< Bit for Multiple Sensor Locations Supported. */
#define RSCS_FEAR_FULL_BIT                           (0x1f)          /**< Bit for all RSC features Supported. */
/** @} */
/** @} */

/**
 * @defgroup RSCS_ENUM Enumerations
 * @{
 */
/**@brief Running Speed and Cadence Service Sensor Location. */
typedef enum
{
    RSCS_SENSOR_LOC_OTHER,          /**< Sensor location: other. */
    RSCS_SENSOR_LOC_SHOE_TOP,       /**< Sensor location: top of shoe. */
    RSCS_SENSOR_LOC_SHOE_IN,        /**< Sensor location: inside of shoe. */
    RSCS_SENSOR_LOC_HIP,            /**< Sensor location: hip. */
    RSCS_SENSOR_LOC_FRONT_WHEEL,    /**< Sensor location: front wheel. */
    RSCS_SENSOR_LOC_LEFT_PEDAL,     /**< Sensor location: left pedal. */
    RSCS_SENSOR_LOC_RIGHT_PEDAL,    /**< Sensor location: right pedal. */
    RSCS_SENSOR_LOC_FRONT_HUB,      /**< Sensor location: front hub. */
    RSCS_SENSOR_LOC_SUP_NB          /**< Number of sensor location. */
} rscs_sensor_loc_t;

/**@brief Running Speed and Cadence Service Control Point Operation Code.*/
typedef enum
{
    RSCS_CTRL_PT_OP_RESERVED,         /**< Reserved for future use. */
    RSCS_CTRL_PT_OP_SET_CUMUL_VAL,    /**< Set Cumulative value Operation Code.*/
    RSCS_CTRL_PT_OP_START_CALIB,      /**< Start Sensor Calibration Operation Code.*/
    RSCS_CTRL_PT_OP_UPD_LOC,          /**< Update Sensor Location Operation Code.*/
    RSCS_CTRL_PT_OP_REQ_SUP_LOC,      /**< Request Supported Sensor Locations Operation Code.*/
    RSCS_CTRL_PT_OP_RSP_CODE = 0x10,  /**< Response code. */
} rscs_ctrl_pt_op_code_t;

/**@brief Running Speed and Cadence Service Control Point Response value.*/
typedef enum
{
    RSCS_CTRL_PT_RSP_RESERVED,        /**< Reserved value. */
    RSCS_CTRL_PT_RSP_SUCCESS,         /**< Operation Succeeded. */
    RSCS_CTRL_PT_RSP_NOT_SUP,         /**< Operation Code Not Supported. */
    RSCS_CTRL_PT_RSP_INVALID_PARAM,   /**< Invalid Parameter. */
    RSCS_CTRL_PT_RSP_FAILED           /**< Operation Failed. */
} rscs_ctrl_pt_rsp_t;

/**@brief Running Speed and Cadence Service event type.*/
typedef enum
{
    RSCS_EVT_INVALID,                                  /**< Indicate that invalid event. */
    RSCS_EVT_RSC_MEAS_NOTIFICATION_ENABLE,             /**< Indicate that RSC Measurement notification has been enabled. */
    RSCS_EVT_RSC_MEAS_NOTIFICATION_DISABLE,            /**< Indicate that RSC Measurement notification has been disabled. */
    RSCS_EVT_CTRL_POINT_INDICATION_ENABLE,             /**< Indicate that SC Control Point indication has been enabled. */
    RSCS_EVT_CTRL_POINT_INDICATION_DISABLE,            /**< Indicate that SC Control Point indication has been disabled. */
    RSCS_EVT_RSC_MEAS_SEND_CPLT,                       /**< Indicate that RSC Measurement has been notified. */
    RSCS_EVT_CUMUL_VAL_SET,                            /**< Indicate that Total Distance value needs to be set. */
    RSCS_EVT_SEBSOR_CALIBRATION,                       /**< Indicate that Sensor calibration procedure should be initiated. */
    RSCS_EVT_SEBSOR_LOC_UPD,                           /**< Indicate that Sensor Location needs to be reset. */
    RSCS_EVT_SUP_SEBSOR_LOC_REQ,                       /**< Indicate that request supported sensor location list. */
    RSCS_EVT_CTRL_POINT_RSP_CPLT                       /**< Indicate that SC Control Point response has been indicated. */
} rscs_evt_type_t;
/** @} */

/**
 * @defgroup RSCS_STRUCT Structures
 * @{
 */
/**@brief Running Speed and Cadence Service event. */
typedef struct
{
    rscs_evt_type_t evt_type;     /**< The RSCS event type. */
    uint8_t         conn_idx;     /**< The index of the connection. */
    const uint8_t  *p_data;       /**< Pointer to event data. */
    uint16_t        length;       /**< Length of event data. */
} rscs_evt_t;

/**@brief Running Speed and Cadence Measurement Character value structure. */
typedef struct
{
    bool        inst_stride_length_present;   /**< If Instantaneous Stride Length is present. */
    bool        total_distance_present;       /**< If Total Distance is present. */
    bool        is_run_or_walk;               /**< True: Running, False: Walking. */
    uint16_t    inst_speed;                   /**< Instantaneous Speed. */
    uint8_t     inst_cadence;                 /**< Instantaneous Cadence. */
    uint16_t    inst_stride_length;           /**< Instantaneous Stride Length. */
    uint32_t    total_distance;               /**< Total Distance. */
} rscs_meas_val_t;
/** @} */

/**
 * @defgroup RSCS_TYPEDEF Typedefs
 * @{
 */
/**@brief Running Speed and Cadence Service event handler type.*/
typedef void (*rscs_evt_handler_t)(rscs_evt_t *p_evt);
/** @} */

/**
 * @defgroup RSCS_STRUCT Structures
 * @{
 */
/**@brief Running Speed and Cadence Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    rscs_evt_handler_t     evt_handler;                  /**< Running Speed and Cadence Service event handler. */
    uint16_t               char_mask;                    /**< Initial mask of supported characteristics, and configured with \ref RSCS_CHAR_MASK. */
    rscs_sensor_loc_t      sensor_location;              /**< Initial sensor location. */
    uint16_t               feature;                      /**< Initial value for features. */
} rscs_init_t;
/** @} */


/**
 * @defgroup RSCS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Running Speed and Cadence Service instance and add in the DB.
 *
 * @param[in] p_rscs_init: Pointer to rscs_init_t Service initialization variable
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t rscs_service_init(rscs_init_t *p_rscs_init);

/**
 *****************************************************************************************
 * @brief Send RSC measurement if notification has been enabled.
 *
 * @param[in] conn_idx: Connnection index.
 * @param[in] p_meas:   The pointer to new Running Speed and Cadence measurement.
 *
 * @return Result of notify value
 *****************************************************************************************
 */
sdk_err_t rscs_measurement_send(uint8_t conn_idx, rscs_meas_val_t *p_meas);

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
sdk_err_t rscs_ctrl_pt_rsp_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Update Sensor Location if Multiple Sensor Locations Supported \ref rscs_init_t.feature.
 *
 * @param[in] sensor_loc: New sensor location.
 *
 * @return Result of update.
 *****************************************************************************************
 */
sdk_err_t rscs_sensor_loc_update(rscs_sensor_loc_t sensor_loc);

/** @} */

#endif
/** @} */

/** @} */


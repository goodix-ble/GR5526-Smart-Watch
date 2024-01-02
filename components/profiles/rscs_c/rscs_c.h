/**
 *****************************************************************************************
 *
 * @file rscs_c.h
 *
 * @brief Running Speed and Cadence Service Client API
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
 * @defgroup BLE_SDK_RSCS_C Running Speed and Cadence Service Client (RSCS_C)
 * @{
 * @brief Running Speed and Cadence Service Client module.
 *
 * @details The Running Speed and Cadence Service Client contains the APIs and types, which can
 *          be used by the application to discovery of Heart Rate Service of peer and interact with it.
 *
 *          The application must provide an event handler to register, then call \ref rscs_client_init().
 *          After Running Speed and Cadence Service Client discoveries peer Running Speed and Cadence Service,
 *          application can call \ref rscs_c_rsc_meas_notify_set(), then will receive Running Speed and Cadence
 *          data from peer, and can call \ref rscs_c_sensor_loc_read() and \ref rscs_c_rsc_feature_read() to get
 *          sensor location information and the supported features of the Server, also can call \ref rscs_c_ctrl_pt_set()
 *          to send control point to peer.
 */

#ifndef __RSCS_C_H__
#define __RSCS_C_H__

#include "ble_prf_types.h"
#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup RSCS_C_MACRO Defines
 * @{
 */
#define RSCS_C_CONNECTION_MAX               10                              /**< Maximum number of HRS Client connections. */
#define RSCS_C_PT_RSP_LEN_MAX               (3 + RSCS_C_SENSOR_LOC_SUP_NB)  /**< Maximum length of SC Control Point response value. */
#define RSCS_C_ERROR_PROC_IN_PROGRESS       0x80                            /**< Error code: A previously triggered SC Control Point operation is still in progress. */
#define RSCS_C_ERROR_CCCD_INVALID           0x81                            /**< Error code: The Client Characteristic Configuration descriptor is not configured. */

/**
 * @defgroup RSCS_C_MEAS_FLAG_BIT Measurement Flag Bits
 * @{
 * @brief Running Speed and Cadence Measurement Flags.
 */
#define RSCS_C_MEAS_FLAG_INST_STRIDE_LEN_BIT           (0x01 << 0)     /**< Flag bit for Instantaneous Stride Length Measurement. */
#define RSCS_C_MEAS_FLAG_TOTAL_DISTANCE_BIT            (0x01 << 1)     /**< Flag bit for Total Distance Measurement. */
#define RSCS_C_MEAS_FLAG_RUNNING_OR_WALKING_BIT        (0x01 << 2)     /**< Flag bit for Running or Walking. */
/** @} */

/**
 * @defgroup RSCS_C_FEAT_BIT Feature Bits
 * @{
 * @brief Running Speed and Cadence Service feature bits.
 */
#define RSCS_C_FEAT_INSTANT_STRIDE_LEN_BIT             (0x01 << 0)     /**< Bit for Instantaneous Stride Length Measurement Supported. */
#define RSCS_C_FEAT_TOTAL_DISTANCE_BIT                 (0x01 << 1)     /**< Bit for Total Distance Measurement Supported. */
#define RSCS_C_FEAT_RUNNING_OR_WALKING_STATUS_BIT      (0x01 << 2)     /**< Bit for Running or Walking Status Supported. */
#define RSCS_C_FEAT_CALIBRATION_PROCEDURE_BIT          (0x01 << 3)     /**< Bit for Calibration Procedure Supported. */
#define RSCS_C_FEAT_MULTIPLE_SENSORS_BIT               (0x01 << 4)     /**< Bit for Multiple Sensor Locations Supported. */
/** @} */
/** @} */

/**
 * @defgroup RSCS_C_ENUM Enumerations
 * @{
 */
/**@brief Running Speed and Cadence Service Client event type. */
typedef enum
{
    RSCS_C_EVT_INVALID,                    /*<* RSCS Client invalid event. */
    RSCS_C_EVT_DISCOVERY_COMPLETE,         /**< RSCS Client has found RSCS service and its characteristics. */
    RSCS_C_EVT_DISCOVERY_FAIL,             /**< RSCS Client found RSCS service failed because of invalid operation or no found at the peer. */
    RSCS_C_EVT_RSC_MEAS_NTF_SET_SUCCESS,   /**< RSCS Client has set Notification of RSC Measure characteristic. */
    RSCS_C_EVT_CTRL_PT_IND_SET_SUCCESS,    /**< RSCS Client has set Indication of Control Point characteristic. */
    RSCS_C_EVT_RSC_MEAS_VAL_RECEIVE,       /**< RSCS Client has received RSC Measurement value notification from peer. */
    RSCS_C_EVT_RSC_FEATURE_RECEIVE,        /**< RSCS Client has received RSC Feature Value read response. */
    RSCS_C_EVT_SENSOR_LOC_RECEIVE,         /**< RSCS Client has received Sensor Location Value read response. */
    RSCS_C_EVT_CTRL_PT_SET_SUCCESS,        /**< RSCS Client has writen Control Point completely. */
    RSCS_C_EVT_CTRL_PT_RSP_RECEIVE,        /**< RSCS Client has received Indication of Control Point characteristic. */
    RSCS_C_EVT_WRITE_OP_ERR,               /**< Error occured when RSCS Client writen to peer. */
} rscs_c_evt_type_t;

/**@brief Running Speed and Cadence Service Sensor Location. */
typedef enum
{
    RSCS_C_SENSOR_LOC_OTHER,          /**< Sensor location: other. */
    RSCS_C_SENSOR_LOC_SHOE_TOP,       /**< Sensor location: top of shoe. */
    RSCS_C_SENSOR_LOC_SHOE_IN,        /**< Sensor location: inside of shoe. */
    RSCS_C_SENSOR_LOC_HIP,            /**< Sensor location: hip. */
    RSCS_C_SENSOR_LOC_FRONT_WHEEL,    /**< Sensor location: front wheel. */
    RSCS_C_SENSOR_LOC_LEFT_PEDAL,     /**< Sensor location: left pedal. */
    RSCS_C_SENSOR_LOC_RIGHT_PEDAL,    /**< Sensor location: right pedal. */
    RSCS_C_SENSOR_LOC_FRONT_HUB,      /**< Sensor location: front hub. */
    RSCS_C_SENSOR_LOC_SUP_NB          /**< Number of sensor location. */
} rscs_c_sensor_loc_t;

/**@brief Running Speed and Cadence Service Control Point Operation Code.*/
typedef enum
{
    RSCS_C_CTRL_PT_OP_RESERVED,         /**< Reserved for future use. */
    RSCS_C_CTRL_PT_OP_SET_CUMUL_VAL,    /**< Set Cumulative value Operation Code.*/
    RSCS_C_CTRL_PT_OP_START_CALIB,      /**< Start Sensor Calibration Operation Code.*/
    RSCS_C_CTRL_PT_OP_UPD_LOC,          /**< Update Sensor Location Operation Code.*/
    RSCS_C_CTRL_PT_OP_REQ_SUP_LOC,      /**< Request Supported Sensor Locations Operation Code.*/
    RSCS_C_CTRL_PT_OP_RSP_CODE = 0x10,  /**< Response code. */
} rscs_c_ctrl_pt_op_code_t;

/**@brief Running Speed and Cadence Service Control Point Response value.*/
typedef enum
{
    RSCS_C_CTRL_PT_RSP_RESERVED,        /**< Reserved value. */
    RSCS_C_CTRL_PT_RSP_SUCCESS,         /**< Operation Succeeded. */
    RSCS_C_CTRL_PT_RSP_NOT_SUP,         /**< Operation Code Not Supported. */
    RSCS_C_CTRL_PT_RSP_INVALID_PARAM,   /**< Invalid Parameter. */
    RSCS_C_CTRL_PT_RSP_FAILED           /**< Operation Failed. */
} rscs_c_ctrl_pt_rsp_t;
/** @} */

/**
 * @defgroup RSCS_C_STRUCT Structures
 * @{
 */
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
} rscs_c_meas_val_t;

/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t rscs_srvc_start_handle;       /**< RSCS Service start handle. */
    uint16_t rscs_srvc_end_handle;         /**< RSCS Service end handle. */
    uint16_t rscs_rsc_meas_handle;         /**< RSCS RSC Measurement characteristic Value handle which has been got from peer. */
    uint16_t rscs_rsc_meas_cccd_handle;    /**< RSCS CCCD handle of RSC Measurement characteristic which has been got from peer. */
    uint16_t rscs_sensor_loc_handle;       /**< RSCS Sensor Location characteristic Value handle which has been got from peer. */
    uint16_t rscs_rsc_feature_handle;      /**< RSCS RSC Feature characteristic Value handle which has been got from peer. */
    uint16_t rscs_ctrl_pt_handle;          /**< RSCS Control Point characteristic Value handle which has been got from peer. */
    uint16_t rscs_ctrl_pt_cccd_handle;     /**< RSCS CCCD handle of Control Point characteristic which has been got from peer. */
} rscs_c_handles_t;

/**@brief Running Speed and Cadence Service Client event. */
typedef struct
{
    uint8_t                  conn_idx;            /**< The connection index. */
    rscs_c_evt_type_t        evt_type;            /**< RSCS Client event type. */
    uint16_t                 handle;              /**< Handle of characteristic. */
    union
    {
        rscs_c_meas_val_t    rsc_meas_buff;                             /**< Buffer of RSC measurement value. */
        uint16_t             rsc_feature;                               /**< RSC feature received. */
        rscs_c_sensor_loc_t  rsc_sensor_loc;                            /**< RSC sensor location received. */
        uint8_t              ctrl_pt_rsp[RSCS_C_PT_RSP_LEN_MAX];        /**< SC Control Point Response. */
    } value;                                                            /**< Decoded result of value received. */

} rscs_c_evt_t;
/** @} */

/**
 * @defgroup RSCS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief Running Speed and Cadence Service Client event handler type. */
typedef void (*rscs_c_evt_handler_t)(rscs_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup RSCS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register RSCS Client event handler.
 *
 * @param[in] evt_handler: Running Speed and Cadence Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t rscs_client_init(rscs_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery Running Speed and Cadence Service on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t rscs_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer RSC Measurement characteristic notify.
 *
 * @param[in] conn_idx: Index of connection.
 * @param[in] is_enable: true or false.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t rscs_c_rsc_meas_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Read RSC Feature characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t rscs_c_rsc_feature_read(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Read Sensor Location characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t rscs_c_sensor_loc_read(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer Control Point characteristic indicate.
 *
 * @param[in] conn_idx:  Index of connection.
 * @param[in] is_enable: True or false.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t rscs_c_ctrl_pt_indicate_set(uint8_t conn_idx, bool is_enable);

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
sdk_err_t rscs_c_ctrl_pt_set(uint8_t conn_idx, uint16_t ctrl_value);
/** @} */
#endif
/** @} */
/** @} */


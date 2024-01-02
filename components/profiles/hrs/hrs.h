/**
 *****************************************************************************************
 *
 * @file hrs.h
 *
 * @brief Heart Rate Service API.
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
 * @defgroup BLE_SDK_HRS Heart Rate Service (HRS)
 * @{
 * @brief Definitions and prototypes for the HRS interface.
 *
 * @details The Heart Rate Service exposes heart rate and other data from a Heart Rate Sensor
 *          intended for fitness applications. This module implements the Heart Rate Service with
 *          the Heart Rate Measurement, Body Sensor Location and Heart Rate Control Point characteristics.
 *
 *          After \ref hrs_init_t variable is initialized, the application must call \ref hrs_service_init()
 *          to add the Heart Rate Service and Heart Rate Measurement characteristic to the BLE Stack database.
 *          However the value of Heart Rate Measurement characteristic is stored in user space.
 *
 *          If a device supports Body Sensor Location, \ref hrs_init_t.char_mask should be
 *          set with the mask \ref HRS_CHAR_BODY_SENSOR_LOC_SUP to expose the Body Sensor Location characteristic. If Energy Expended Field is included
 *          in the Heart Rate Measurement characteristic, \ref hrs_init_t.char_mask must be set with \ref HRS_CHAR_ENGY_EXP_SUP.
 *
 *          If an event handler is provided by the application, the Heart Rate Service will pass
 *          Heart Rate Service events to the application.
 *
 *          If Notify is enabled, the notification of Heart Rate Measurement characteristic will be sent
 *          when hrs_heart_rate_measurement_send() is called.
 */

#ifndef __HRS_H__
#define __HRS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup HRS_MACRO Defines
 * @{
 */

#define HRS_CONNECTION_MAX                  10                                  /**< Maximum number of Heart Rate Service connections. */
#define HRS_MEAS_MAX_LEN                    20                                  /**< Maximum length of heart rate measurement characteristic. */
#define HRS_MAX_BUFFERED_RR_INTERVALS       9                                   /**< Size of RR Interval buffer inside service. */

/**
 * @defgroup HRS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref hrs_init_t.char_mask.
 */

#define HRS_CHAR_MANDATORY                  0x0F         /**< Bit mask of the mandatory characteristics. */
#define HRS_CHAR_BODY_SENSOR_LOC_SUP        0x30         /**< Bit mask of Body Sensor Location Feature Supported. */
#define HRS_CHAR_ENGY_EXP_SUP               0xC0         /**< Bit mask of Energy Expanded Feature Supported. */
/** @} */
/** @} */

/**
 * @defgroup HRS_ENUM Enumerations
 * @{
 */
/**@brief Values for sensor location. */
typedef enum
{
    HRS_SENS_LOC_OTHER,             /**< The sensor location is other. */
    HRS_SENS_LOC_CHEST,             /**< The sensor location is the chest. */
    HRS_SENS_LOC_WRIST,             /**< The sensor location is the wrist. */
    HRS_SENS_LOC_FINGER,            /**< The sensor location is the finger. */
    HRS_SENS_LOC_HAND,              /**< The sensor location is the hand. */
    HRS_SENS_LOC_EARLOBE,           /**< The sensor location is the earlobe. */
    HRS_SENS_LOC_FOOT,              /**< The sensor location is the foot. */
} hrs_sensor_loc_t;

/**@brief Heart Rate Service event types. */
typedef enum
{
    HRS_EVT_NOTIFICATION_ENABLED,    /**< Heart Rate value notification has been enabled. */
    HRS_EVT_NOTIFICATION_DISABLED,   /**< Heart Rate value notification has been disabled. */
    HRS_EVT_RESET_ENERGY_EXPENDED,   /**< The peer device requests to reset Energy Expended. */
    HRS_EVT_READ_BODY_SEN_LOCATION,  /**< The peer device reads Body Sensor Location characteristic. */
} hrs_evt_type_t;
/** @} */

/**
 * @defgroup HRS_STRUCT Structures
 * @{
 */
/**@brief Heart Rate Service event. */
typedef struct
{
    uint8_t         conn_idx;    /**< Index of connection. */
    hrs_evt_type_t  evt_type;    /**< Heart Rate Service event type. */
} hrs_evt_t;
/** @} */

/**
 * @defgroup HRS_TYPEDEF Typedefs
 * @{
 */
/**@brief Heart Rate Service  event handler type. */
typedef void (*hrs_evt_handler_t)(hrs_evt_t *p_evt);
/** @} */

/**
 * @defgroup HRS_STRUCT Structures
 * @{
 */
/**@brief Heart Rate Service Init variable. */
typedef struct
{
    hrs_evt_handler_t  evt_handler;                       /**< Heart Rate Service event handler. */
    bool is_sensor_contact_supported;                     /**< Determine if sensor contact detection is to be supported. */
    uint8_t char_mask;                                    /**< Mask of Supported characteristics, and configured with \ref HRS_CHAR_MASK */
    hrs_sensor_loc_t sensor_loc;                          /**< The value of Body Sensor Location characteristic is static while in a connection. */
} hrs_init_t;
/** @} */

/**
 * @defgroup HRS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Set the state of the Sensor Contact Detected bit.
 *
 * @param[in] is_sensor_contact_detected: True if sensor contact is detected, false otherwise.
 *****************************************************************************************
 */
void hrs_sensor_contact_detected_update(bool is_sensor_contact_detected);

/**
 *****************************************************************************************
 * @brief Set the state of the Sensor Contact Supported bit.
 *
 * @param[in] is_sensor_contact_supported: New state of the Sensor Contact Supported bit.
 *****************************************************************************************
 */
void hrs_sensor_contact_supported_set(bool is_sensor_contact_supported);

/**
 *****************************************************************************************
 * @brief Set the Body Sensor Location.
 *
 * @details Sets a new value of the Body Sensor Location characteristic. The new value will be sent
 *          to the client the next time the client reads the Body Sensor Location characteristic.
 *
 * @param[in] hrs_sensor_loc: New Body Sensor Location.
 *****************************************************************************************
 */
void hrs_sensor_location_set(hrs_sensor_loc_t hrs_sensor_loc);

/**
 *****************************************************************************************
 * @brief Update Energy measurement if Energy Expended is supported.
 *
 * @param[in] energy: New energy measurement.
 *****************************************************************************************
 */
void hrs_energy_update(uint16_t energy);

/**
 *****************************************************************************************
 * @brief Send Heart Rate measurement if Notify has been enabled.
 *
 * @param[in] conn_idx          Connection index.
 * @param[in] heart_rate        New heart rate measurement.
 * @param[in] is_energy_updated Indicate whether update energy expended.
 *
 * @return Result of notify value.
 *****************************************************************************************
 */
sdk_err_t hrs_heart_rate_measurement_send(uint8_t conn_idx, uint16_t heart_rate, bool is_energy_updated);

/**
 *****************************************************************************************
 * @brief Add an RR Interval measurement to the RR Interval buffer.
 *
 * @details All buffered RR Interval measurements will be included in the next
 *          Heart Rate Measurement notification. The maximum number of
 *          RR Interval measurement is \ref HRS_MAX_BUFFERED_RR_INTERVALS. If the
 *          buffer is full, the oldest measurement in the buffer will be deleted.
 *
 * @param[in] rr_interval New RR Interval measurement (will be buffered until
 *                        the next transmission of Heart Rate Measurement).
 *****************************************************************************************
 */
void hrs_rr_interval_add(uint16_t rr_interval);

/**
 *****************************************************************************************
 * @brief Init a Heart Rate Service instance and add in the DB.
 *
 * @param[in] p_hrs_init: Pointer to a Heart Rate Init variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t hrs_service_init(hrs_init_t *p_hrs_init);

/**
 *****************************************************************************************
 * @brief Provide the interface for other modules to obtain the hrs service start handle .
 *
 * @return The hrs service start handle.
 *****************************************************************************************
 */
uint16_t hrs_service_start_handle_get(void);

/** @} */

#endif
/** @} */
/** @} */

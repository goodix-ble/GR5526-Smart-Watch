/**
 *****************************************************************************************
 *
 * @file hts.h
 *
 * @brief Health Thermometer Service API
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
 * @defgroup BLE_SDK_HTS Health Thermometer Service (HTS)
 * @{
 * @brief Definitions and prototypes for the HTS interface.
 *
 * @details The Health Thermometer Service exposes temperature and other data related to a
 *          thermometer used for healthcare applications. This module implements the Health
 *          Thermometer Service with the Temperature Meaturement, Temperature Type, Intermediate
 *          Temperature, and Measurement Interval characteristics. 
 *
 *          After \ref hts_init_t variable is initialized, the application must call \ref hts_service_init()
 *          to optionally add the Health Thermometer Service, Temperature Meaturement, Temperature Type,
 *          Intermediate Temperature, and Measurement Interval characteristics to the BLE stack database
 *          according to \ref hts_init_t.char_mask. However the value of Temperature Type is  stored within 
 *          \ref hts_init_t.temp_type which locates in user space.
 */
 
#ifndef _HTS_H_
#define _HTS_H_

#include "gr_includes.h"
#include "custom_config.h"
#include "ble_prf_utils.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup HTS_MACRO Defines
 * @{
 */
#define HTS_CONNECTION_MAX               10                              /**< Maximum number of Health Thermometer Service connections. */
#define HTS_TEM_MEAS_MAX_LEN             20                              /**< Maximum length of temperature measurement encode. */
#define HTS_TEM_TYPE_MAX_LEN             1                               /**< Maximum length of temperature type value. */
#define HTS_INTM_TEM_MAX_LEN             20                              /**< Maximun length of intermediate temperature encode. */
#define HTS_MEAS_INTERVAL_MAX_LEN        2                               /**< Maximum length of measurement interval value. */
#define HTS_MEAS_INTV_DFLT_MIN           2                               /**< Minimun interval of temperature measurement (in unit of 1s). */
#define HTS_MEAS_INTV_DFLT_MAX           10                              /**< Maximum interval of temperature measurement (in unit of 1s). */

/**
 * @defgroup HTS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref hts_init_t.char_mask.
 */
#define HTS_CHAR_MANDATORY               0x000f     /**< Bit mask for mandatory characteristic in HTS. */
#define HTS_CHAR_TEM_TYPE_SUP            0x003f     /**< Bit mask for temperature type characteristic that is optional. */
#define HTS_CHAR_INTM_TEM_SUP            0x01c0     /**< Bit mask for intermediate temperature characteristic that is optional. */
#define HTS_CHAR_MEAS_INTERVAL_SUP       0x1e00     /**< Bit mask for mearsurement interval characteristic that is optional. */
#define HTS_CHAR_FULL                    0x1fff     /**< Bit mask of the full characteristic. */
/** @} */
/** @} */

/**
 * @defgroup HTS_ENUM Enumerations
 * @{
 */
/**@brief Health Thermometer Service event type.*/
typedef enum
{
    HTS_EVT_INVALID,                                /**< Indicate that invalid event. */
    HTS_EVT_TEM_MEAS_INDICATION_ENABLE,             /**< Indicate that temperature measurement indication has been enabled. */
    HTS_EVT_TEM_MEAS_INDICATION_DISABLE,            /**< Indicate that temperature measurement indication has been disabled. */
    HTS_EVT_INTM_TEM_NOTIFICATION_ENABLE,           /**< Indicate that intermediate temperature notification has been enabled. */
    HTS_EVT_INTM_TEM_NOTIFICATION_DISABLE,          /**< Indicate that intermediate temperature notification has been disbled. */
    HTS_EVT_MEAS_INTREVAL_INDICATION_ENABLE,        /**< Indicate that measurement interval indication has been enabled. */
    HTS_EVT_MEAS_INTERVAL_INDICATION_DISABLE,       /**< Indicate that measurement interval indication has been disabled. */
    HTS_EVT_MEAS_INTERVAL_UPDATE,                   /**< Indicate that measurement interval has been updated. */
    HTS_EVT_READ_CHARACTERISTIC,                    /**< The peer reads the characteristic. */
} hts_evt_type_t;

/**@brief Health Thermometer temperature unit, */
typedef enum
{
    HTS_TEMPERATURE_CELCIUS,                        /**< Indicate that temperature measurement Value Unit is celcius. */
    HTS_TEMPERATURE_FAHRENHEIT,                     /**< Indicate that temperature measurement Value Unit is fahrenheit. */
} hts_temp_unit_t;

/**@brief Health Thermometer temperature measurement type. */
typedef enum
{
    HTS_TEMPERATURE_STABLE,                         /**< Stable type of temperature. */
    HTS_TEMPERATURE_INTERMEDIATE,                   /**< Intermediary type of temperature. */
} hts_temp_meas_type_t;

/**@brief Health Thermometer Measurement flag bits. */
typedef enum
{
    HTS_MEAS_FLAG_TEM_UINTS_BIT  = (0x01<<0),       /**< Bit for temperature uints. */
    HTS_MEAS_FLAG_TIME_STAMP_BIT = (0x01<<1),       /**< Bit for time stamp. */
    HTS_MEAS_FLAG_TEM_TYPE_BIT   = (0x01<<2),       /**< Bit for temperature type. */
} hts_flag_bit_t;

/**@brief Temperature Type measurement locations. */
typedef enum
{
    HTS_TEMP_TYPE_ARMPIT = 0x01,                    /**< Temperature measurement location: armpit. */
    HTS_TEMP_TYPE_BODY,                             /**< Temperature measurement location: body. */
    HTS_TEMP_TYPE_EAR,                              /**< Temperature measurement location: ear. */
    HTS_TEMP_TYPE_FINGER,                           /**< Temperature measurement location: finger. */
    HTS_TEMP_TYPE_GI_TRACT,                         /**< Temperature measurement location: Gi tract. */
    HTS_TEMP_TYPE_MOUTH,                            /**< Temperature measurement location: mouth. */
    HTS_TEMP_TYPE_RECTUM,                           /**< Temperature measurement location: rectum. */
    HTS_TEMP_TYPE_TOE,                              /**< Temperature measurement location: toe. */
    HTS_TEMP_TYPE_EAR_DRUM,                         /**< Temperature measurement location: ear drum. */
} hts_temp_meas_loc_t;

/**@brief The parameters for \ref HTS_EVT_READ_CHARACTERISTIC. */
typedef enum
{
    HTS_READ_CHAR_TEMP_TYPE,    /**< The peer reads the Temperature Type characteristic. */
    HTS_READ_CHAR_MEAS_INTL,    /**< The peer reads the Measurement Interval characteristic. */
} hts_read_characteristic_t;
/** @} */

/**
 * @defgroup HTS_STRUCT Structures
 * @{
 */
/**@brief Health Thermometer Service event. */
typedef struct
{
    hts_evt_type_t evt_type;     /**< The HTS event type. */
    uint8_t        conn_idx;     /**< The index of the connection. */
    const uint8_t *p_data;       /**< Pointer to the event data. */
    uint16_t       length;       /**< Length of the event data. */
} hts_evt_t;
/** @} */

/**
 * @defgroup HTS_TYPEDEF Typedefs
 * @{
 */
/**@brief Health Thermometer Service event handler type.*/
typedef void (*hts_evt_handler_t)(hts_evt_t *p_evt);

/**@brief Health Thermometer Service date time type.*/
typedef prf_date_time_t hts_date_time_t;
/** @} */

/**
 * @defgroup HTS_STRUCT Structures
 * @{
 */
/**@brief SFLOAT format (IEEE-11073 32-bit FLOAT, defined as a 32-bit vlue with 24-bit mantissa and 8-bit exponent. */
typedef struct
{
  int8_t  exponent;            /**< Base 10 exponent, only 8 bits */
  int32_t mantissa;            /**< Mantissa, only 24 bits */
} ieee_float32_t;

/**@brief Health Theromometer Measurement Character value structure. */
typedef struct
{
    hts_temp_meas_type_t temp_meas_type;            /**< Stable or intermediary type of temperature. */
    uint16_t             temp_original_value;       /**< Temperature measurement original value. */
    ieee_float32_t       temp_convert_value;        /**< Temperature  measurement convert value. */
    hts_date_time_t      time_stamp;                /**< Time Stamp. */
    hts_temp_meas_loc_t  temp_type;                 /**< Temperature Type measurement location. */
} hts_meas_val_t;

/**@brief Health Thermometer Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    hts_evt_handler_t   evt_handler;                /**< Health Thermometer Service event handler. */
    uint16_t            char_mask;                  /**< Initial mask of Supported characteristics, and configured with \ref HTS_CHAR_MASK */
    hts_temp_unit_t     temperature_units;          /**< Initial if Temperature is in Fahrenheit unit, Celcius otherwise. */
    bool                time_stamp_present;         /**< Initial if Time Stamp is present. */
    hts_temp_meas_loc_t temp_type;                  /**< Initial temperature type measurement location. */
    uint16_t            meas_interval;              /**< Initial temperature measurement interval. */
    uint16_t            min_meas_interval_sup;      /**< Initial minimum temperature measurement interval support. */
    uint16_t            max_meas_interval_sup;      /**< Initial maximum temperature measurement interval support. */
} hts_init_t;
/** @} */

/**
 * @defgroup HTS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Health Thermometer Service instance and add in the DB.
 *
 * @param[in] p_hts_init: Pointer to Health Thermometer Service initialization variable
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t hts_service_init(hts_init_t *p_hts_init);

/**
 *****************************************************************************************
 * @brief Send temperature measurement if indication or notification has been enabled.
 *
 * @param[in] conn_idx: Connnection index.
 * @param[in] p_meas:   The pointer to new health temperature measurement.
 *
 * @return Result of indicate value
 *****************************************************************************************
 */
sdk_err_t hts_measurement_send(uint8_t conn_idx, hts_meas_val_t *p_meas);

/**
 *****************************************************************************************
 * @brief Send temperature measurement interval if indication has been enabled.
 *
 * @param[in] conn_idx: Connnection index.
 *
 * @return Result of indicate value
 *****************************************************************************************
 */
sdk_err_t hts_measurement_interval_send(uint8_t conn_idx);
/** @} */

#endif
/** @} */
/** @} */

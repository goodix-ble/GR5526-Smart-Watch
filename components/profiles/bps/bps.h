/**
 *****************************************************************************************
 *
 * @file bps.h
 *
 * @brief Blood Pressure Service API.
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
 * @defgroup BLE_SDK_BPS Blood Pressure Service (BPS)
 * @{
 * @brief Definitions and prototypes for the BPS interface.
 *
 * @details The Blood Pressure Service exposes blood pressure and other data from
 *          a blood pressure monitor for use in consumer and profession healthcare
 *          application.
 *
 *          This module implements the Blood Pressure Service with the Blood
 *          Pressure Measurement characteristic. The Intermediate Cuff Pressure
 *          characteristic is optional, and is not supported in this module.
 *
 *          After \ref bps_init_t variable is initialized, the developer shall call
 *          \ref bps_service_init() to add a Blood Pressure Service and a Blood
 *          Pressure Measurement characteristic to the BLE Stack database. Blood
 *          Pressure Measurement characteristic supports Indicate property only,
 *          so this module stores the value of Blood Pressure Measurement
 *          neither in BLE Stack database nor in bps_env_t variable.
 *
 *          If an event handler is provided by the application, the Blood
 *          Pressure Service will pass Blood Pressure Service events to the
 *          application.
 *
 */

#ifndef __BPS_H__
#define __BPS_H__

#include "gr_includes.h"
#include "ble_prf_utils.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup  BPS_MACRO Defines
 * @{
 */
#define BPS_CONNECTION_MAX              10                           /**< Maximum number of BPS connections. */
#define BPS_BP_MEAS_MAX_LEN             20                           /**< Maximum notification length. */

/**
 * @defgroup BPS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of @ref bps_init_t.char_mask.
 */
#define BPS_CHAR_MANDATORY              0x018F   /**< Bit mask of the mandatory characteristics in BPS. */
#define BPS_CHAR_INTM_CUFF_PRESS_SUP    0x0070   /**< Bit mask of the Intermediate Cuff Pressure descriptor. */
#define BPS_CHAR_FULL                   0x01ff   /**< Bit mask of the full characteristic. */
/** @} */
/** @} */

/**
 * @defgroup BPS_ENUM Enumerations
 * @{
 */
/**@brief Blood Pressure Service event type. */
typedef enum
{
    BPS_EVT_INVALID,                        /**< Invalid event. */
    BPS_EVT_BP_MEAS_INDICATION_ENABLED,     /**< The measurement indication has been enabled. */
    BPS_EVT_BP_MEAS_INDICATION_DISABLED,    /**< The measurement indication has been disabled. */
    BPS_EVT_INTM_CUFF_PRESS_NTF_ENABLED,    /**< The Intermediate Cuff Pressure notification has been enabled. */
    BPS_EVT_INTM_CUFF_PRESS_NTF_DISABLED,   /**< The Intermediate Cuff Pressure notification has been disabled. */
    BPS_EVT_READ_BL_PRESSURE_FEATURE,       /**< The peer reads Blood Pressure Feature characteristic. */
} bps_evt_type_t;

/**@brief Blood Pressure Feature bits. */
enum bp_feature_bit
{
    BP_FEATURE_BODY_MOVEMENT_BIT        = (0x01 << 0),    /**< Body Movement Detection Support bit. */
    BP_FEATURE_CUFF_FIT_BIT             = (0x01 << 1),    /**< Cuff Fit Detection Support bit. */
    BP_FEATURE_IRREGULAR_PULSE_BIT      = (0x01 << 2),    /**< Irregular Pulse Detection Support bit. */
    BP_FEATURE_PULSE_RATE_RANGE_BIT     = (0x01 << 3),    /**< Pulse Rate Range Detection Support bit. */
    BP_FEATURE_MEASUREMENT_POSITION_BIT = (0x01 << 4),    /**< Measurement Position Detection Support bit. */
    BP_FEATURE_MULTIPLE_BOND_BIT        = (0x01 << 5),    /**< Multiple Bond Support bit. */
};
/** @} */

/**
 * @defgroup BPS_TYPEDEF Typedefs
 * @{
 */
/**@brief Blood Pressure Service event handler type. */
typedef void (*bps_evt_handler_t)(uint8_t conn_idx, bps_evt_type_t event);
/** @} */

/**
 * @defgroup BPS_STRUCT Structures
 * @{
 */
/**@brief SFLOAT format (IEEE-11073 16-bit FLOAT, defined as a 16-bit value with 12-bit mantissa and 4-bit exponent. */
typedef struct
{
  int8_t  exponent;     /**< Base 10 exponent, only 4 bits */
  int16_t mantissa;     /**< Mantissa, only 12 bits */
} bps_ieee_float16_t;

/**@brief Blood Pressure measurement structure. */
typedef struct
{
    uint8_t             bl_unit_in_kpa;         /**< Blood Pressure Units Flag, 0=mmHg, 1=kPa */
    uint8_t             time_stamp_present;     /**< Time Stamp Flag, 0=not present, 1=present. */
    uint8_t             pulse_rate_present;     /**< Pulse Rate Flag, 0=not present, 1=present. */
    uint8_t             user_id_present;        /**< User ID Flag, 0=not present, 1=present. */
    uint8_t             meas_status_present;    /**< Measurement Status Flag, 0=not present, 1=present. */
    bps_ieee_float16_t  systolic;               /**< Blood Pressure Measurement Compound Value - Systolic. */
    bps_ieee_float16_t  diastolic;              /**< Blood Pressure Measurement Compound Value - Diastolic . */
    bps_ieee_float16_t  mean_arterial_pr;       /**< Blood Pressure Measurement Compound Value - Mean Arterial Pressure. */
    prf_date_time_t     time_stamp;             /**< Time Stamp. */
    bps_ieee_float16_t  pulse_rate;             /**< Pulse Rate. */
    uint8_t             user_id;                /**< User ID. */
    uint16_t            meas_status;            /**< Measurement Status. */
} bps_meas_t;

/**@brief Blood Pressure Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    bps_evt_handler_t evt_handler;                    /**< Blood Pressure Service event handler. */
    uint16_t          char_mask;                      /**< Mask of Supported characteristics, and configured with \ref BPS_CHAR_MASK */
    uint16_t          bp_feature;                     /**< Value of Blood Pressure Feature characteristic. */
} bps_init_t;
/** @} */

/**
 * @defgroup BPS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Blood Pressure Service instance and add in the DB.
 *
 * @param[in] p_bps_init: Pointer to Blood Pressure Service initialization variable
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t bps_service_init(bps_init_t *p_bps_init);

/**
 *****************************************************************************************
 * @brief Send Blood Pressure Measurement indication.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_meas:   Pointer to the Blood Pressure measurement.
 *
 * @return Result of sending indication.
 *****************************************************************************************
 */
sdk_err_t bps_measurement_send(uint8_t conn_idx, bps_meas_t *p_meas);
/** @} */

#endif
/** @} */
/** @} */

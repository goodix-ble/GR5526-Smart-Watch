/**
 *****************************************************************************************
 *
 * @file ias.h
 *
 * @brief Immediate Alarm Service API
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
 * @defgroup BLE_SDK_IAS Immediate Alarm Service (IAS)
 * @{
 * @brief Definitions and prototypes for the IAS interface.
 *
 * @details The Immediate Alert Service exposes a control point to allow a peer device to cause
 *          the device to immediately alert. 
 *
 *          The application must provide an event handler to set \ref ias_init_t.evt_handler. 
 *          After \ref ias_init_t variable is initialized, the application must call \ref ias_service_init()
 *          to add the Immediate Alert Service and Alert Level characteristic to the BLE Stack database,
 *          the service will notify the application when the value of Alert Level characteristic is changed
 *          by the peer device.
 *
 *          This module also provides \ref ias_alert_level_get() function to the
 *          application to poll the current value of Alert Level characteristic.
 *
 */

#ifndef __IAS_H__
#define __IAS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>

/**
 * @defgroup IAS_ENUM Enumerations
 * @{
 */
/**@brief Immediate Alert Service Alert levels. */
enum
{
    IAS_ALERT_NONE,     /**< No alert. */
    IAS_ALERT_MILD,     /**< Mild alert. */
    IAS_ALERT_HIGH,     /**< High alert. */
};

/**@brief Immediate Alert Service event type. */
typedef enum
{
    IAS_EVT_INVALID,              /**< Invalid IAS event. */
    IAS_EVT_ALERT_LEVEL_UPDATED,  /**< Alert Level Updated event. */
}ias_evt_type_t;
/** @} */

/**
 * @defgroup IAS_STRUCT Structures
 * @{
 */
/**@brief Immediate Alert Service event. */
typedef struct
{
    ias_evt_type_t evt_type;    /**< Type of event. */
    uint8_t alert_level;        /**< New value of Alert level. */
}ias_evt_t;
/** @} */

/**
 * @defgroup IAS_TYPEDEF Typedefs
 * @{
 */
/**@brief Immediate Alert Service event handler type. */
typedef void (*ias_evt_handler_t)(ias_evt_t *p_evt);
/** @} */

/**
 * @addtogroup IAS_STRUCT Structures
 * @{
 */
/**@brief Immediate Alert Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    ias_evt_handler_t evt_handler;      /**< Immediate Alert Service event handler. */
} ias_init_t;
/** @} */

/**
 * @defgroup IAS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Immediate Alert Service instance and add in the BLE Stack database.
 *
 * @param[in] p_ias_init: Pointer to Immediate Aler Service initialization variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t ias_service_init(ias_init_t *p_ias_init);

/**
 *****************************************************************************************
 * @brief Get current value of alert level characteristic.
 *
 * @param[out] p_alert_level: Pointer to the current value of alert level.
 *
 * @return BLE_SDK_SUCCESS on success, otherwise an error code.
 *****************************************************************************************
 */
sdk_err_t ias_alert_level_get(uint8_t *p_alert_level);
/** @} */

#endif
/** @} */
/** @} */

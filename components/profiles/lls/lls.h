/**
 *****************************************************************************************
 *
 * @file lls.h
 *
 * @brief Link Loss Service API
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
 * @defgroup BLE_SDK_LLS Link Loss Service (LLS)
 * @{
 * @brief Definitions and prototypes for the LLS interface.
 *
 * @details The Link Loss Service uses the Alert Level characteristic to cause an alert in the
 *          device when the link is lost.
 *
 *          The application must provide an event handler to set \ref lls_init_t.evt_handler. 
 *          After \ref lls_init_t variable is initialized, the application must call \ref lls_service_init()
 *          to add Alert Level Characteristic to the BLE Stack database, the service will notify the application
 *          with the event handler when the link has been lost, and which Alert Level has been set.
 *
 *          This module also provides \ref lls_alert_level_get() function to the
 *          application to poll the current value of Alert Level characteristic.
 *
 */

#ifndef __LLS_H__
#define __LLS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>

/**
 * @defgroup LLS_ENUM Enumerations
 * @{
 */
/** Link Loss Service alert levels. */
typedef enum
{
    LLS_ALERT_LEVEL_NO_ALERT,       /**< No alert. */
    LLS_ALERT_LEVEL_MILD_ALERT,     /**< Mild alert. */
    LLS_ALERT_LEVEL_HIGH_ALERT,     /**< High alert. */
} lls_alert_levels_t;

/**@brief Link Loss Service event type. */
typedef enum
{
    LLS_EVT_INVALID,            /**< Invalid LLS event. */
    LLS_EVT_LINK_LOSS_ALERT     /**< Link loss alert event. */
} lls_evt_type_t;
/** @} */

/**
 * @defgroup LLS_STRUCT Structures
 * @{
 */
/**@brief Link Loss Service event. */
typedef struct
{
    lls_evt_type_t evt_type;            /**< Type of event. */
    lls_alert_levels_t alert_level;     /**< Alert level. */
} lls_evt_t;
/** @} */

/**
 * @defgroup LLS_TYPEDEF Typedefs
 * @{
 */
/**@brief Link Loss Service event handler type. */
typedef void (*lls_evt_handler_t)(lls_evt_t *p_evt);
/** @} */

/**
 * @addtogroup LLS_STRUCT Structures
 * @{
 */
/**@brief Link Loss Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    lls_evt_handler_t  evt_handler;          /**< Link Loss Service event handler. */
    lls_alert_levels_t initial_alert_level;  /**< Initial value of the alert level. */
} lls_init_t;
/** @} */

/**
 * @defgroup LLS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Link Loss Service instance and add in ATT DB
 *
 * @param[in] p_lls_init: Pointer to Link Loss Service Service initialization variable
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t lls_service_init(lls_init_t *p_lls_init);

/**
 *****************************************************************************************
 * @brief Get current value of alert level characteristic.
 *
 * @param[out] p_alert_level: Pointer to the current value of alert level
 *
 * @return BLE_SDK_SUCCESS on success, otherwise an error code
 *****************************************************************************************
 */
sdk_err_t lls_alert_level_get(uint8_t *p_alert_level);
/** @} */

#endif
/** @} */
/** @} */

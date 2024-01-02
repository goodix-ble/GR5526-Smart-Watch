/**
 *****************************************************************************************
 *
 * @file bas.h
 *
 * @brief Battery Service API
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
 * @defgroup BLE_SDK_BAS Battery Service (BAS)
 * @{
 * @brief Definitions and prototypes for the BAS interface.
 *
 * @details The Battery Service exposes the state of a battery within a device.
 *          This module implements the Battery Service with the Battery Level characteristic.
 *
 *          After \ref bas_init_t variable is initialized, the application must call \ref bas_service_init()
 *          to add the Battery Service(s) and Battery Level characteristic(s) to the BLE Stack database.
 *          However the value of Battery Level characteristic is stored within \ref bas_init_t.batt_lvl
 *          which locates in user space.
 *
 *          The module supports more than one instance of the Battery service with \ref bas_service_init().
 *          When the device has more than one instance, each Battery Level characteristic shall include
 *          a Characteristic Presentation Format descriptor to identify the instance. \ref bas_init_t.char_mask
 *          shall be set with the mask \ref BAS_CHAR_FORMAT_SUP to add the descriptor to the BLE stack database.
 *
 *          If \ref bas_init_t.char_mask is set with the mask \ref BAS_CHAR_LVL_NTF_SUP, the module will
 *          support notification of the Battery Level characteristic through the bas_batt_lvl_update() function.
 *          If an event handler is provided by the application, the Battery Service will pass Battery Service
 *          events to the application.
 *
 */

#ifndef __BAS_H__
#define __BAS_H__

#include "gr_includes.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @defgroup BAS_MACRO Defines
 * @{
 */
#define BAS_INSTANCE_MAX        1         /**< Maximum number of Battery Service instances. The value is configurable. */
#define BAS_CONNECTION_MAX      10        /**< Maximum number of BAS connections.The value is configurable. */
#define BAS_LVL_MAX_LEN         1         /**< Maximun length of battery level value. */

/**
 * @defgroup BAS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref bas_init_t.char_mask.
 */
#define BAS_CHAR_MANDATORY      0x07      /**< Bit mask of the mandatory characteristics in BAS. */
#define BAS_CHAR_LVL_NTF_SUP    0x08      /**< Bit mask of Battery Level notification. */
#define BAS_CHAR_FORMAT_SUP     0x10      /**< Bit mask of the Presentation Format descriptor. */
#define BAS_CHAR_FULL           0x1f      /**< Bit mask of the full characteristic. */
/** @} */
/** @} */

/**
 * @defgroup BAS_ENUM Enumerations
 * @{
 */
/**@brief Battery Service event types. */
typedef enum
{
    BAS_EVT_INVALID,                      /**< Indicate that it's an invalid event. */
    BAS_EVT_NOTIFICATION_ENABLED,         /**< Indicate that notification has been enabled. */
    BAS_EVT_NOTIFICATION_DISABLED,        /**< Indicate that notification has been disabled. */
} bas_evt_type_t;
/** @} */

/**
 * @defgroup BAS_STRUCT Structures
 * @{
 */
/**@brief Battery Service event. */
typedef struct
{
    bas_evt_type_t evt_type;     /**< The BAS event type. */
    uint8_t        conn_idx;     /**< The index of the connection. */
} bas_evt_t;
/** @} */

/**
 * @defgroup BAS_TYPEDEF Typedefs
 * @{
 */
/**@brief Battery Service event handler type. */
typedef void (*bas_evt_handler_t)(bas_evt_t *p_evt);
/** @} */

/**
 * @defgroup BAS_STRUCT Structures
 * @{
 */
/**@brief Battery Service init structure. This contains all options and data needed for initialization of the service. */
typedef struct
{
    bas_evt_handler_t evt_handler;    /**< Battery Service event handler. */
    uint8_t           char_mask;      /**< Initial mask of supported characteristics, and configured with \ref BAS_CHAR_MASK */
    uint8_t           batt_lvl;       /**< Initial value of Battery Level characteristic. */
} bas_init_t;
/** @} */

/**
 * @defgroup BAS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize Battery Service instances and add to the DB.
 *
 * @param[in] ins_num:  The number of Battery Service instances.
 * @param[in] bas_init: The array of Battery Service initialization variables.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t bas_service_init(bas_init_t bas_init[],uint8_t ins_num);

/**
 *****************************************************************************************
 * @brief Update a Battery Level value. If notification is enabled, send it.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] ins_idx:  Battery Service instance index.
 * @param[in] batt_lvl: Battery Level value.
 *
 * @return Result of battery level updating.
 *****************************************************************************************
 */
sdk_err_t bas_batt_lvl_update(uint8_t conn_idx, uint8_t ins_idx, uint8_t batt_lvl);

/**
 *****************************************************************************************
 * @brief Provide the interface for other modules to obtain the bas service start handle .
 *
 * @return The bas service start handle.
 *****************************************************************************************
 */
uint16_t bas_service_start_handle_get(void);

/** @} */

#endif
/** @} */
/** @} */

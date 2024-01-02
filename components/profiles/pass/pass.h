/**
 ****************************************************************************************
 *
 * @file pass.h
 *
 * @brief Phone Alert Status Service API.
 *
 ****************************************************************************************
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
 * @defgroup BLE_SDK_PASS Phone Alert Status Service (PASS)
 * @{
 * @brief Phone Alert Status Service module.
 *
 * @details The Phone Alert Status Service shall expose the Alert Status characteristic and the
 *          Ringer Setting characteristic, and may expose the Ringer Control Point characteristic.
 *
 *          After \ref pass_init_t variable is intialized, the application must call \ref pass_service_init()
 *          to add Phone Alert Status Service and Alert Status, Ringer Setting and Ringer Control Point
 *          characteristics to the BLE Stack database according to \ref pass_init_t.char_mask.
 */

#ifndef __PASS_H__
#define __PASS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup PASS_MACRO Defines
 * @{
 */
#define PASS_CONNECTION_MAX         10                           /**< Maximum number of Phone Alert Status Service connections. */
#define PASS_ALERT_STATUS_VAL_LEN   1                            /**< Length of Alert Status value. */
#define PASS_RINGER_SET_VAL_LEN     1                            /**< Length of Ringer Setting value. */
#define PASS_RINGER_CTRL_PT_VAL_LEN 1                            /**< Length of Ringer Control Point value. */

/**
 * @defgroup PASS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref pass_init_t.char_mask.
 */
#define PASS_CHAR_MANDATORY         0x003f                       /**< Bit mask for mandatory characteristic in PASS. */
#define PASS_CHAR_RING_CTRL_PT_SUP  0x0180                       /**< Bit mask for Ringer Control Point characteristic in PASS. */
#define PASS_CHAR_FULL              0x01ff                       /**< Bit mask of the full characteristic. */
/** @} */

/**
 * @defgroup PASS_ALERT_STATUES_BIT Alert Status BIT
 * @{
 * @brief PASS Alert Status bits.
 */
#define PASS_NO_STATE_ACTIVE        (0x00)                       /**< Bit for no state active. */
#define PASS_RINGER_ACTIVE          (0x01 << 0)                  /**< Bit for ringer State active. */
#define PASS_VIBRATE_ACTIVE         (0x01 << 1)                  /**< Bit for vibrate State active. */
#define PASS_DISPLAY_ALERT_ACTIVE   (0x01 << 2)                  /**< Bit for display Alert Status State active. */
#define PASS_ALL_STATE_ACTIVE       (0x07)                       /**< Bit for no state active. */
/** @} */

/**
 * @defgroup PASS_RINGER_SETTING Ringer Setting
 * @{
 * @brief The Ringer Setting characteristic defines the setting of the ringer.
 */
#define PASS_RINGER_SET_SILENT      0                            /**< Ringer Silent. */
#define PASS_RINGER_SET_NORMAL      1                            /**< Ringer Normal. */
/** @} */
/** @} */

/**
 * @defgroup PASS_ENUM Enumerations
 * @{
 */
/**@brief Phone Alert Status Service Ringer Control Point. */
typedef enum
{
    PASS_CTRL_PT_SILENT_MODE = 0x01,    /**< Silent Mode. */
    PASS_CTRL_PT_MUTE_ONCE,             /**< Mute Once. */
    PASS_CTRL_PT_CANCEL_SLIENT_MODE,    /**< Cancel Silent Mode. */
} pass_ringer_ctrl_pt_t;

/**@brief Phone Alert Status Service event type. */
typedef enum
{
    PASS_EVT_INVALID,                  /**< Invalid PASS event type. */
    PASS_EVT_ALERT_STATUS_NTF_ENABLE,  /**< Alert Status notification is enabled. */
    PASS_EVT_ALERT_STATUS_NTF_DISABLE, /**< Alert Status notification is disabled. */
    PASS_EVT_RINGER_SET_NTF_ENABLE,    /**< Ringer Setting notification is enabled. */
    PASS_EVT_RINGER_SET_NTF_DISABLE,   /**< Ringer Setting notification is disabled. */
    PASS_EVT_SILENT_MODE_SET,          /**< Set silent mode. */
    PASS_EVT_MUTE_ONCE_SET,            /**< Set mute once. */
    PASS_EVT_SILENT_MODE_CANCEL,       /**< Cancel silent mode. */
} pass_evt_type_t;
/** @} */

/**
 * @defgroup PASS_STRUCT Structures
 * @{
 */
/**@brief Phone Alert Status Service event. */
typedef struct
{
    uint8_t             conn_idx;        /**< The index of the connection. */
    pass_evt_type_t      evt_type;        /**< The CTS event type. */
} pass_evt_t;
/** @} */

/**
 * @defgroup PASS_TYPEDEF Typedefs
 * @{
 */
/**@brief Phone Alert Status Service event handler type.*/
typedef void (*pass_evt_handler_t)(pass_evt_t *p_evt);
/** @} */

/**
 * @defgroup PASS_STRUCT Structures
 * @{
 */
/**@brief Phone Alert Status Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    pass_evt_handler_t  evt_handler;     /**< Phone Alert Status Service event handler. */
    uint16_t            char_mask;       /**< Initial mask of supported characteristics, and configured with \ref PASS_CHAR_MASK. */
    uint8_t             alert_status;    /**< Initial alert status. */
    uint8_t             ringer_setting;  /**< Initial ringer setting. */
} pass_init_t;
/** @} */

/**
 * @defgroup PASS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Phone Alert Status Service instance and add in the DB.
 *
 * @param[in] p_pass_init: Pointer to PASS Service initialization variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t pass_service_init(pass_init_t *p_pass_init);

/**
 *****************************************************************************************
 * @brief Get Ringer Setting value.
 *
 * @return Current Ringer Setting value.
 *****************************************************************************************
 */
uint8_t pass_ringer_setting_get(void);

/**
 *****************************************************************************************
 * @brief Set Ringer Setting value.
 *
 * @param[in] conn_idx: Connnection index.
 * @param[in] new_setting: New Ringer Setting value.
 *****************************************************************************************
 */
void pass_ringer_setting_set(uint8_t conn_idx, uint8_t new_setting);

/**
 *****************************************************************************************
 * @brief Set Alert Status value.
 *
 * @param[in] conn_idx: Connnection index.
 * @param[in] new_status: New Alert Status value.
 *****************************************************************************************
 */
void pass_alert_status_set(uint8_t conn_idx, uint8_t new_status);
/** @} */

#endif
/** @} */
/** @} */


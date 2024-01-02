/**
 ****************************************************************************************
 *
 * @file App key core.h
 *
 * @brief App Key Core API
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
#ifndef __APP_KEY_CORE_H__
#define __APP_KEY_CORE_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup APP_KEY_CORE_MAROC Defines
 * @{
 */
#define APP_KEY_REG_COUNT_MAX         10   /**< Maximum number of key instance can register, which can be configurable. */
#define APP_KEY_DOUBLE_TIME_COUNT     50   /**< Double click time count(in unit of 10ms), which can be configurable. */
#define APP_KEY_LONG_TIME_COUNT       100  /**< Long click time count(in uint of 10ms), which can be configurable. */
#define APP_KEY_CONTINUE_TIME_COUNT   20   /**< Continue click time count(in uint of 10ms), which can be configurable. */
/** @} */

/**
 * @defgroup APP_KEY_CORE_ENUM Enumerations
 * @{
 */
/**@brief App key polling state. */
typedef enum
{
    APP_KEY_STA_INIT,                /**< Key initialization state for key scan procedure. */
    APP_KEY_STA_DEBOUNCE,            /**< Key debounce state for key scan procedure. */
    APP_KEY_STA_PRESS,               /**< Key press state for key scan procedure. */
    APP_KEY_STA_WAITE_RELEASE,       /**< Waite key release state for key scan procedure. */
    APP_KEY_STA_NO_CLICK,            /**< Key no click state for key read procedure. */
    APP_KEY_STA_SINGLE_CLICK,        /**< Key single click state for key read procedure. */
    APP_KEY_STA_DOUBLE_CLICK,        /**< Key double click state for key read procedure. */
    APP_KEY_STA_LONG_CLICK,          /**< Key long click state for key read procedure. */
    APP_KEY_STA_CONTINUE_CLICK,      /**< Key continue click state for key read procedure. */
    APP_KEY_STA_RELEASE,             /**< key continue click release state. */
} app_key_state_t;

/**@brief App key click event type. */
typedef enum
{
    APP_KEY_NO_CLICK,                /**< Key no click event. */
    APP_KEY_SINGLE_CLICK,            /**< Key single click event. */
    APP_KEY_DOUBLE_CLICK,            /**< Key double click event. */
    APP_KEY_LONG_CLICK,              /**< Key long click event. */
    APP_KEY_CONTINUE_CLICK,          /**< Key continue click event. */
    APP_KEY_CONTINUE_RELEASE         /**< Key continue click release. */
} app_key_click_type_t;
/** @} */

/**
 * @defgroup APP_KEY_CORE_TYPEDEF Typedefs
 * @{
 */
/**@brief APP Key core event callback.*/
typedef void (*app_key_core_evt_cb_t)(uint8_t key_idx, app_key_click_type_t key_click_type);
/** @} */

/**
 * @defgroup APP_KEY_CORE_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register app key click event callback.
 *
 * @param[in] key_core_evt_cb: Key core event callback.
 *
 * @return Result of key core event callback is NULL or not.
 *****************************************************************************************
 */
bool app_key_core_cb_register(app_key_core_evt_cb_t key_core_evt_cb);

/**
 *****************************************************************************************
 * @brief Record app key is pressed down.
 *
 * @param[in] key_idx:    Index of key register.
 * @param[in] is_pressed: True or false
 *****************************************************************************************
 */
void app_key_core_key_pressed_record(uint8_t key_idx, bool is_pressed);

/**
 *****************************************************************************************
 * @brief Notificaiton which key is waiting for polling.
 *
 * @param[in] key_idx: Index of key register.
 *****************************************************************************************
 */
void app_key_core_key_wait_polling_record(uint8_t key_idx);

/**
 *****************************************************************************************
 * @brief Check all keys are released or not.
 *
 * @return Result of checking.
 *****************************************************************************************
 */
bool app_key_core_is_all_release(void);

/**
 *****************************************************************************************
 * @brief App key state polling.
 *****************************************************************************************
 */
void app_key_core_polling_10ms(void);
/** @} */

#endif
/** @} */
/** @} */


/**
 ****************************************************************************************
 *
 * @file app_key.c
 *
 * @brief App Key API
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

#ifndef __APP_KEY_H__
#define __APP_KEY_H__

#include "app_key_core.h"
#include "app_gpiote.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup APP_KEY_STRUCT Structures
 * @{
 */
/**@brief App key gpio initialization variables. */
typedef struct
{
    app_io_type_t  gpio_type;            /**< Key gpio type. */
    uint32_t       gpio_pin;             /**< Key gpio pin. */
    app_io_mode_t  trigger_mode;         /**< Specifies the operating mode for the selected pin. */
    app_io_pull_t  pull;                 /**< Pull mode.*/
    uint8_t        key_id;               /**< Key register ID. */
} app_key_gpio_t;
/** @} */

/**
 * @defgroup APP_KEY_TYPEDEF Typedefs
 * @{
 */
/**@brief APP Key event callback.*/
typedef void (*app_key_evt_cb_t)(uint8_t key_id, app_key_click_type_t key_click_type);
/** @} */

/**
 * @defgroup APP_KEY_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief App key initialize.
 *
 * @param[in] key_inst:     The array of key instance.
 * @param[in] key_num:      The number of key instance.
 * @param[in] key_click_cb: App key click event callback.
 *
 * @return Result of app key inlitialization.
 *****************************************************************************************
 */
bool app_key_init(app_key_gpio_t key_inst[], uint8_t key_num, app_key_evt_cb_t key_click_cb);

/**
 *****************************************************************************************
 * @brief App key pressed down handler.
 *****************************************************************************************
 */
void app_key_pressed_handler(app_key_gpio_t *p_app_key_info);
/** @} */

#endif


/**
 ****************************************************************************************
 *
 * @file    app_gpiote.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of GPIO Interrupt app library.
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
 ****************************************************************************************
 */

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_GPIO_INTERRUPT GPIO INTERRUPT
  * @brief GPIO INTERRUPT APP module driver.
  * @{
  */


#ifndef _APP_GPIOTE_H_
#define _APP_GPIOTE_H_

#include "app_io.h"
#include "app_drv_error.h"

/** @addtogroup APP_GPIO_INTERRUPT_STRUCTURES Structures
  * @{
  */

/**
  * @brief GPIOTE Interrupt parameters structure definition
  */
typedef struct
{
    app_io_type_t           type;           /**< Specifies IO type */
    uint32_t                pin;            /**< Specifies the IO pins to be configured. */
    app_io_mode_t           mode;           /**< Specifies the IO mode for the selected pins. */
    app_io_pull_t           pull;           /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
    app_io_callback_t       io_evt_cb;      /**< IO callback function. */
} app_gpiote_param_t;

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_GPIOTE_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the APP GPIO Interrupt DRIVER according to the specified parameters
 *         in the app_gpiote_param_t and app_gpiote_event_handler_t.
 *
 * @param[in]  p_params: Pointer to app_gpiote_param_t parameter which contains the
 *                       configuration information for the specified GPIO.
 * @param[in]  table_cnt: Used GPIO number.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_gpiote_init(const app_gpiote_param_t *p_params, uint8_t table_cnt);

/**
 ****************************************************************************************
 * @brief  Config the APP GPIO Interrupt DRIVER according to the specified parameters
 *         in the app_gpiote_param_t.
 *
 * @param[in]  p_config: Pointer to app_gpiote_param_t parameter which contains the
 *                       configuration information for the specified GPIO.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_gpiote_config(const app_gpiote_param_t *p_config);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP GPIO Interrupt DRIVER peripheral.
 *
 * @param[in]  type: GPIO type, See app_io_type_t.
 * @param[in]  pin:  The pin want to unregister.
 *
 * @return Result of unregister.
 ****************************************************************************************
 */
uint16_t app_gpiote_deinit(app_io_type_t type, uint32_t pin);

/** @} */

#endif

/** @} */
/** @} */
/** @} */

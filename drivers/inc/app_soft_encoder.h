/**
 ****************************************************************************************
 *
 * @file    app_soft_encoder.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of TIM app library.
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

/** @defgroup APP_SOFT_ENCODER SOFT ENCODER
  * @brief SOFT ENCODER APP module driver.
  * @{
  */


#ifndef _APP_SOFT_ENCODER_H_
#define _APP_SOFT_ENCODER_H_

#include "app_drv_error.h"
#include "app_drv_config.h"
#include "app_io.h"
#include "app_gpiote.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup APP_SOFT_ENCODER_ENUM Enumerations
  * @{
  */

/**
  * @brief SOFT_ENCODER module Enumerations definition
  */
typedef enum
{
    APP_SOFT_ENCODER_SIGNAL_A = 0,
    APP_SOFT_ENCODER_SIGNAL_B,
    APP_SOFT_ENCODER_SINGAL_NUMBER,
} app_soft_encoder_signal_t;

/**
  * @brief SOFT_ENCODER module DIRECTION definition
  */
typedef enum
{
    APP_SOFT_ENCODER_STOP,
    APP_SOFT_ENCODER_POSITIVE,
    APP_SOFT_ENCODER_REVERSE,
} app_soft_encoder_direction_t;

/** @} */

/** @addtogroup APP_SOFT_ENCODER_STRUCTURES Structures
  * @{
  */

/**
  * @brief SOFT_ENCODER io config
  */
typedef struct
{
    app_io_type_t           type;
    uint32_t                pin;
    app_io_mode_t           mode;
    app_io_pull_t           pull;
} app_soft_encoder_io_param_t;

/**
  * @brief SOFT_ENCODER event callback definition
  */
typedef void (*app_soft_encoder_callback)(app_soft_encoder_direction_t direction,int distance);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_SOFT_ENCODER_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP TIM DRIVER according to the specified parameters
 *         in the app_soft_encoder_params_t and app_soft_encoder_evt_handler_t.
 *
 * @param[in]  p_params: Pointer to app_soft_encoder_params_t parameter which contains the
 *                       configuration information for the specified SOFT_ENCODER module.
 * @param[in]  evt_handler: SOFT_ENCODER user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_soft_encoder_init(app_soft_encoder_io_param_t *p_a_params,
                               app_soft_encoder_io_param_t *p_b_params,
                               app_soft_encoder_callback    evt_handler);
/** @} */

#endif

#ifdef __cplusplus
}
#endif

/** @} */
/** @} */
/** @} */

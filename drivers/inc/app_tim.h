/**
 ****************************************************************************************
 *
 * @file    app_tim.h
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

/** @defgroup APP_TIMER TIMER
  * @brief TIMER APP module driver.
  * @{
  */


#ifndef _APP_TIM_H_
#define _APP_TIM_H_

#include "grx_hal.h"
#include "app_drv_error.h"
#include "app_drv_config.h"
#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_TIMER_MODULE_ENABLED

/** @addtogroup APP_TIM_ENUM Enumerations
  * @{
  */

/**
  * @brief TIM module Enumerations definition
  */
typedef enum
{
    APP_TIM_ID_0,                /**< TIMER module 0 */
    APP_TIM_ID_1,                /**< TIMER module 1 */
    APP_TIM_ID_MAX               /**< Only for check parameter, not used as input parameters. */
} app_tim_id_t;

/**
  * @brief TIM event Enumerations definition
  */
typedef enum
{
    APP_TIM_EVT_ERROR,           /**< Error reported by TIMER peripheral. */
    APP_TIM_EVT_DONE,            /**< Count done event by TIMER peripheral. */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    APP_TIM_EVT_CHANNEL0,        /**< Channle0 event by TIMER peripheral. */
    APP_TIM_EVT_CHANNEL1,        /**< Channle1 event by TIMER peripheral. */
    APP_TIM_EVT_CHANNEL2,        /**< Channle2 event by TIMER peripheral. */
    APP_TIM_EVT_CHANNEL3,        /**< Channle3 event by TIMER peripheral. */
#endif
} app_tim_evt_t;
/** @} */

/** @addtogroup APP_TIM_TYPEDEFS Type definitions
  * @{
  */
/**
  * @brief TIM event callback definition
  */
typedef void (*app_tim_evt_handler_t)(app_tim_evt_t *p_evt);

/** @} */

/** @addtogroup APP_TIM_ENUM Enumerations
  * @{
  */
/**@brief App tim state types. */
typedef enum
{
    APP_TIM_INVALID = 0,
    APP_TIM_ACTIVITY,
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    APP_TIM_SLEEP,
#endif
} app_tim_state_t;

/** @} */

/** @addtogroup APP_TIM_STRUCTURES Structures
  * @{
  */
/**
  * @brief TIM device structure definition
  */
typedef struct
{
    app_tim_evt_handler_t   evt_handler;  /**< TIM event callback.      */
    timer_handle_t          handle;       /**< TIMER handle Structure.      */
    app_tim_state_t         tim_state;    /**<  App tim state types.      */
}tim_env_t;

/**
  * @brief TIM parameters structure definition
  */
typedef struct
{
    app_tim_id_t        id;      /**< specified TIMER module ID.      */
    timer_init_t        init;    /**< TIMER Base required parameters. */
    tim_env_t           tim_env;  /**< TIM device structure definition. */
} app_tim_params_t;

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_TIM_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP TIM DRIVER according to the specified parameters
 *         in the app_tim_params_t and app_tim_evt_handler_t.
 *
 * @param[in]  p_params: Pointer to app_tim_params_t parameter which contains the
 *                       configuration information for the specified TIM module.
 * @param[in]  evt_handler: TIM user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_tim_init(app_tim_params_t *p_params, app_tim_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP TIM DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_tim_deinit(app_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Starts the TIM counter in interrupt mode.
 * @param[in]  id: which TIM module want to start.
 *
 * @return Result of initialization.
 *
 ****************************************************************************************
 */
uint16_t app_tim_start(app_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Stops the TIM counter in interrupt mode.
 * @param[in]  id: which TIM module want to stop.
 *
 * @return Result of initialization.
 *
 ****************************************************************************************
 */
uint16_t app_tim_stop(app_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Return the TIM handle.
 *
 * @param[in]  id: TIM Channel ID.
 *
 * @return Pointer to the specified ID's TIM handle.
 ****************************************************************************************
 */
timer_handle_t *app_tim_get_handle(app_tim_id_t id);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
/**
 ****************************************************************************************
 * @brief  Return channel0 value.
 *
 * @param[in]  id: TIM Channel ID.
 *
 * @return Current value when channel0 captured.
 ****************************************************************************************
 */
uint32_t app_tim_get_channel0_val(app_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Return channel1 value.
 *
 * @param[in]  id: TIM Channel ID.
 *
 * @return Current value when channel1 captured.
 ****************************************************************************************
 */
uint32_t app_tim_get_channel1_val(app_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Return channel2 value.
 *
 * @param[in]  id: TIM Channel ID.
 *
 * @return Current value when channel2 captured.
 ****************************************************************************************
 */
uint32_t app_tim_get_channel2_val(app_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Return channel3 value.
 *
 * @param[in]  id: TIM Channel ID.
 *
 * @return Current value when channel3 captured.
 ****************************************************************************************
 */
uint32_t app_tim_get_channel3_val(app_tim_id_t id);
#endif

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */


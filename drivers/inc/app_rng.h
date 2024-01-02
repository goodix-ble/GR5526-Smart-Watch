/**
 ****************************************************************************************
 *
 * @file    app_rng.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of RNG app library.
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

/** @defgroup APP_RNG RNG
  * @brief RNG APP module driver.
  * @{
  */


#ifndef _APP_RNG_H_
#define _APP_RNG_H_

#include "grx_hal.h"
#include "app_drv_error.h"
#include "app_drv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_RNG_MODULE_ENABLED

/** @addtogroup APP_RNG_ENUM Enumerations
  * @{
  */

/**
  * @brief RNG operating mode Enumerations definition
  */
typedef enum
{
    APP_RNG_TYPE_INTERRUPT,        /**< Interrupt operation mode. */
    APP_RNG_TYPE_POLLING,          /**< Polling operation mode. */
    APP_RNG_TYPE_MAX               /**< Only for check parameter, not used as input parameters. */
} app_rng_type_t;

/**
  * @brief RNG event Enumerations definition
  */
typedef enum
{
    APP_RNG_EVT_DONE,             /**< Generated random by UART peripheral. */
    APP_RNG_EVT_ERROR,            /**< Error reported by UART peripheral. */
} app_rng_evt_type_t;
/** @} */

/** @addtogroup HAL_APP_RNG_STRUCTURES Structures
  * @{
  */
/**
  * @brief RNG event structure definition
  */
typedef struct
{
    app_rng_evt_type_t  type;           /**< Type of event. */
    uint32_t            random_data;    /**< Random number. */
} app_rng_evt_t;

/** @} */

/** @addtogroup APP_RNG_TYPEDEFS Type definitions
  * @{
  */
/**
  * @brief RNG event callback definition
  */
typedef void (*app_rng_evt_handler_t)(app_rng_evt_t *p_evt);

/** @} */

/** @addtogroup APP_RNG_ENUM Enumerations
  * @{
  */
/**@brief App rng state types. */
typedef enum
{
    APP_RNG_INVALID = 0,
    APP_RNG_ACTIVITY,
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    APP_RNG_SLEEP,
#endif
} app_rng_state_t;

/** @} */

/** @addtogroup HAL_APP_RNG_STRUCTURES Structures
  * @{
  */
/**
  * @brief RNG device structure definition
  */
typedef struct
{
    app_rng_evt_handler_t   evt_handler;   /**< RNG event callback. */
    rng_handle_t            handle;        /**< RNG handle Structure. */
    app_rng_type_t          use_type;      /**< RNG operating mode. */
    app_rng_state_t         rng_state;     /**< App rng state types. */
} rng_env_t;
/**
  * @brief RNG parameters structure definition
  */
typedef struct
{
    app_rng_type_t      use_type;  /**< Specifies the operation mode of RNG. */
    rng_init_t          init;      /**< RNG required parameters. */
    rng_env_t           rng_env;   /**< RNG device structure definition. */
} app_rng_params_t;

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_RNG_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the APP RNG DRIVER according to the specified parameters
 *         in the app_rng_params_t and app_rng_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_rng_params_t parameter which contains the
 *                       configuration information for the specified RNG module.
 * @param[in]  evt_handler: RNG user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_rng_init(app_rng_params_t *p_params, app_rng_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP RNG DRIVER peripheral.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_rng_deinit(void);

/**
 ****************************************************************************************
 * @brief  Generate a 32-bit random number.
 *
 * @param[in]  p_seed: user configured seeds. the seed is valid when seed_mode member of
 *               rng_init_t is configured as RNG_SEED_USER. If 59-bit random number is
 *               selected, the seed need to provide [0~58] bit spaces. If 128-bit random
 *               number is selected, the seed need to provide [0~127] bit spaces.
 * @param[out] p_random32bit: Pointer to generated random number variable if successful.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_rng_gen_sync(uint16_t *p_seed, uint32_t *p_random32bit);

/**
 ****************************************************************************************
 * @brief  Generate a 32-bit random number in interrupt mode.
 *
 * @param[in]  p_seed: user configured seeds. the seed is valid when seed_mode member of
 *               rng_init_t is configured as RNG_SEED_USER. If 59-bit random number is
 *               selected, the seed need to provide [0~58] bit spaces. If 128-bit random
 *               number is selected, the seed need to provide [0~127] bit spaces.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_rng_gen_async(uint16_t *p_seed);

/**
 ****************************************************************************************
 * @brief  Return the RNG handle.
 *
 * @return Pointer to the RNG handle.
 ****************************************************************************************
 */
rng_handle_t *app_rng_get_handle(void);

#endif
/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */

/** @} */


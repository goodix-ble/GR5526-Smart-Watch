/**
 ****************************************************************************************
 *
 * @file    app_aon_wdt.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of AON_WDT app library.
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

/** @defgroup APP_AON_WDT AON WDT
  * @brief AON WDT APP module driver.
  * @{
  */


#ifndef _APP_AON_WDT_H_
#define _APP_AON_WDT_H_

#include "grx_hal.h"
#include "app_drv_error.h"
#include "app_drv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_AON_WDT_MODULE_ENABLED

/** @addtogroup APP_AON_WDT_TYPEDEFS Type definitions
  * @{
  */

/**
  * @brief AON_WDT event callback definition
  */
typedef void (*app_aon_wdt_evt_handler_t)(void);
/** @} */

/** @addtogroup APP_AON_WDT_ENUMS Enumerations
  * @{
  */
/**@brief App aon_wdt state types. */
typedef enum
{
    APP_AON_WDT_INVALID = 0,
    APP_AON_WDT_ACTIVITY,
} app_aon_wdt_state_t;

/** @} */

/** @addtogroup APP_AON_WDT_STRUCTURES Structures
  * @{
  */

/**
  * @brief AON_WDT device structure definition
  */
typedef struct
{
    app_aon_wdt_evt_handler_t   evt_handler;        /**< AON WDT event callback definition. */
    aon_wdt_handle_t            handle;             /**< AON WDT handle definition. */
    app_aon_wdt_state_t         aon_wdt_state;      /**< AON WDT state types. */
}aon_aon_wdt_env_t;

/**
  * @brief AON_WDT parameters structure definition
  */
typedef struct
{
    aon_wdt_init_t          init;               /**< AON WDT init parameters. */
    aon_aon_wdt_env_t       aon_aon_wdt_env;    /**< AON_WDT device structure definition. */
} app_aon_wdt_params_t;
/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_AON_WDT_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP AON WDT DRIVER.
 *
 * @param[in]  p_params: Pointer to app_aon_wdt_params_t parameter which contains the
 *                       configuration information for the specified AON WDT module.
 * @param[in]  evt_handler: AON WDT user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_aon_wdt_init(app_aon_wdt_params_t *p_params, app_aon_wdt_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief De-initialize the APP AON WDT DRIVER.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_aon_wdt_deinit(void);

/**
 ****************************************************************************************
 * @brief  Refresh the AON WDT.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_aon_wdt_refresh(void);
/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */


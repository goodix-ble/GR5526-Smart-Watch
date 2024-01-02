/**
 ****************************************************************************************
 *
 * @file    app_comp.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of COMP app library.
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

/** @defgroup APP_COMP COMP
  * @brief COMP APP module driver.
  * @{
  */


#ifndef _APP_COMP_PUB_H_
#define _APP_COMP_PUB_H_

#include "grx_hal.h"
#include "app_drv_error.h"
#include "app_drv_config.h"
#include "app_io.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_COMP_MODULE_ENABLED

/** @addtogroup APP_COMP_ENUM Enumerations
  * @{
  */
/**
  * @brief APP COMP event Enumerations definition
  */
typedef enum
{
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
    APP_COMP_EVT_RISING,
    APP_COMP_EVT_FALLING,
#else
    APP_COMP_EVT_DONE,
#endif
    APP_COMP_EVT_ERROR
} app_comp_evt_t;
/** @} */

/** @addtogroup APP_COMP_TYPEDEFS Type definitions
  * @{
  */
/**
  * @brief COMP event callback definition
  */
typedef void (*app_comp_evt_handler_t)(app_comp_evt_t *p_evt);
/** @} */

/** @addtogroup APP_COMP_STRUCTURES Structures
  * @{
  */
/**
  * @brief COMP pins Structures
  */
typedef struct
{
   app_io_type_t        type;         /**< Specifies the type of COMP IO. */
   app_io_mux_t         mux;          /**< Specifies the Peripheral to be connected to the selected pins. */
   uint32_t             pin;          /**< Specifies the IO pins to be configured.
                                           This parameter can be any value of @ref GR5xxx_pins. */
} app_comp_pin_t;

/**
  * @brief COMP pins config Structures
  */
typedef struct
{
    app_comp_pin_t      input;        /**< Set the configuration of input pin. */
    app_comp_pin_t      vref;         /**< Set the configuration of reference pin. */
} app_comp_pin_cfg_t;

/** @} */

/** @addtogroup APP_COMP_ENUM Enumerations
  * @{
  */
/**
  * @brief App COMP state types
  */
typedef enum
{
    APP_COMP_INVALID = 0,
    APP_COMP_ACTIVITY,
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    APP_COMP_SLEEP,
#endif
} app_comp_state_t;

/** @} */

/** @addtogroup APP_COMP_STRUCTURES Structures
  * @{
  */
/**
  * @brief COMP device structure definition
  */
typedef struct
{
    app_comp_evt_handler_t  evt_handler;     /**< COMP event callback definition. */
    comp_handle_t           handle;          /**< COMP handle definition. */
    app_comp_pin_cfg_t      *p_pin_cfg;      /**< COMP pins config Structures. */
    app_comp_state_t        comp_state;      /**< COMP state types. */
}comp_env_t;

/**
  * @brief COMP parameters structure definition
  */
typedef struct
{
    app_comp_pin_cfg_t  pin_cfg;      /**< the pin configuration information for the specified COMP module. */
    comp_init_t         init;         /**< COMP configuration parameters. */
    comp_env_t          comp_env;     /**< COMP device structure definition. */
} app_comp_params_t;


/** @} */



/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_COMP_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP COMP DRIVER according to the specified parameters
 *         in the app_comp_params_t and app_comp_evt_handler_t.
 *
 * @param[in]  p_params: Pointer to app_comp_params_t parameter which contains the
 *                       configuration information for the specified COMP module.
 * @param[in]  evt_handler: COMP user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_comp_init(app_comp_params_t *p_params, app_comp_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP COMP DRIVER peripheral.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_comp_deinit(void);

/**
 ****************************************************************************************
 * @brief  Start the comparator.
 *
 @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_comp_start(void);

/**
 ****************************************************************************************
 * @brief  Stop the comparator.
 *
 @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_comp_stop(void);

/**
 ****************************************************************************************
 * @brief  Return the COMP handle.
 *
 * @return Pointer to the COMP handle.
 ****************************************************************************************
 */
comp_handle_t *app_comp_get_handle(void);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */

/** @} */

/** @} */


/**
 ****************************************************************************************
 *
 * @file    app_dual_tim.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of DUAL TIM app library.
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

/** @defgroup APP_DUAL_TIMER DUAL TIMER
  * @brief DUAL TIMER APP module driver.
  * @{
  */


#ifndef _APP_DUAL_TIM_H_
#define _APP_DUAL_TIM_H_

#include "grx_hal.h"
#include "app_drv_error.h"
#include "app_drv_config.h"

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
#include "app_io.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_DUAL_TIMER_MODULE_ENABLED

/** @addtogroup APP_DUAL_TIM_ENUM Enumerations
  * @{
  */

/**
  * @brief APP_DUAL_TIM module Enumerations definition
  */
typedef enum
{
    APP_DUAL_TIM_ID_0,     /**< DUAL TIMER module 0 */
    APP_DUAL_TIM_ID_1,     /**< DUAL TIMER module 1 */
    APP_DUAL_TIM_ID_MAX    /**< Only for check parameter, not used as input parameters. */
} app_dual_tim_id_t;

/**
  * @brief DUAL_TIM event Enumerations definition
  */
typedef enum
{
    APP_DUAL_TIM_EVT_ERROR,    /**< Error reported by DUAL TIMER peripheral. */
    APP_DUAL_TIM_EVT_DONE,     /**< Interrupt done by DUAL TIMER peripheral. */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    APP_DUAL_TIM_EVT_ACT_START,   /**< Interrupt action start by DUAL TIMER peripheral.       */
    APP_DUAL_TIM_EVT_IOA_ACT_C1,  /**< Interrupt ioa action count 1 by DUAL TIMER peripheral. */
    APP_DUAL_TIM_EVT_IOA_ACT_C2,  /**< Interrupt ioa action count 2 by DUAL TIMER peripheral. */
    APP_DUAL_TIM_EVT_ACT_PERIOD,  /**< Interrupt action period by DUAL TIMER peripheral.      */
    APP_DUAL_TIM_EVT_ACT_STOP,    /**< Interrupt action stop by DUAL TIMER peripheral.        */
    APP_DUAL_TIM_EVT_IOB_ACT_C1,  /**< Interrupt iob action count 1 by DUAL TIMER peripheral. */
    APP_DUAL_TIM_EVT_IOB_ACT_C2,  /**< Interrupt iob action count 2 by DUAL TIMER peripheral. */
    APP_DUAL_TIM_EVT_IOC_ACT_C1,  /**< Interrupt ioc action count 1 by DUAL TIMER peripheral. */
    APP_DUAL_TIM_EVT_IOC_ACT_C2   /**< Interrupt ioc action count 2 by DUAL TIMER peripheral. */
#endif
} app_dual_tim_evt_t;

/**
  * @brief DUAL_TIM state type Enumerations definition
  */
typedef enum
{
    APP_DUAL_TIM_INVALID = 0,
    APP_DUAL_TIM_ACTIVITY,
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    APP_DUAL_TIM_SLEEP,
#endif
} app_dual_tim_state_t;
/** @} */

/** @addtogroup APP_DUAL_TIM_TYPEDEFS Type definitions
  * @{
  */
/**
  * @brief APP_DUAL_TIM event callback definition
  */
typedef void (*app_dual_tim_evt_handler_t)(app_dual_tim_evt_t *p_evt);

/** @} */

/** @addtogroup APP_DUAL_TIM_STRUCTURES Structures
  * @{
  */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
/**
  * @brief DUAL_TIM IO configuration Structures
  */
typedef struct
{
    app_io_type_t        type;     /**< Specifies the type of Dual Timer IO.
                                        This parameter can be any value of @ref app_io_type_t. */
    uint32_t             pin;      /**< Specifies the IO pins to be configured.
                                        This parameter can be any value of @ref GR5xxx_pins. */
} app_dual_tim_pin_t;

/**
  * @brief DUAL_TIM io crtl parameters structure definition
  */
typedef struct
{
    app_dual_tim_pin_t               pin_cfg;      /**< Set the configuration of pin for DUAL_TIM io crtl. */
    dual_timer_io_ctrl_cfg_t         io_crtl_cfg;  /**< DUAL_TIM io crtl required parameters. */
} app_dual_tim_io_crtl_params_t;
#endif

/**
  * @brief DUAL_TIM device structure definition
  */
typedef struct
{
    app_dual_tim_evt_handler_t  evt_handler;      /**< APP_DUAL_TIM event callback definition. */
    dual_timer_handle_t         handle;           /**< DUAL_TIM handle definition. */
    app_dual_tim_state_t        dual_tim_state;   /**< DUAL_TIM state type Enumerations definition. */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    app_dual_tim_pin_t          cha_pin_cfg;      /**< DUAL_TIM CHANNELA IO configuration Structures. */
    app_dual_tim_pin_t          chb_pin_cfg;      /**< DUAL_TIM CHANNELB IO configuration Structures. */
    app_dual_tim_pin_t          chc_pin_cfg;      /**< DUAL_TIM CHANNELC IO configuration Structures. */

    volatile bool               is_cha_enable;
    volatile bool               is_chb_enable;
    volatile bool               is_chc_enable;
#endif
}dual_tim_env_t;

/**
  * @brief DUAL_TIM parameters structure definition
  */
typedef struct
{
    app_dual_tim_id_t   id;    /**< specified DUAL TIMER module ID. */
    dual_timer_init_t   init;  /**< DUAL_TIM Base required parameters. */
    dual_tim_env_t      dual_tim_env;  /**< DUAL_TIM device structure definition. */
} app_dual_tim_params_t;
/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_DUAL_TIM_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP DUAL TIM DRIVER according to the specified parameters
 *         in the app_dual_tim_params_t and app_dual_tim_evt_handler_t.
 *
 * @param[in]  p_params: Pointer to app_dual_tim_params_t parameter which contains the
 *                       configuration information for the specified DUAL TIM module.
 * @param[in]  evt_handler: DUAL TIM user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_dual_tim_init(app_dual_tim_params_t *p_params, app_dual_tim_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP DUAL TIM DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_dual_tim_deinit(app_dual_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Starts the DUAL TIM counter in interrupt mode.
 * @param[in]  id: which DUAL TIM module want to start.
 *
 * @return Result of execution.
 *
 ****************************************************************************************
 */
uint16_t app_dual_tim_start(app_dual_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Stops the DUAL TIM counter in interrupt mode.
 * @param[in]  id: which DUAL TIM module want to stop.
 *
 * @return Result of execution.
 *
 ****************************************************************************************
 */
uint16_t app_dual_tim_stop(app_dual_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Set the APP DUAL TIM according to the specified parameters
 *
 * @param[in]  p_params: Pointer to app_dual_tim_params_t parameter which contains the
 *                       configuration information for the specified DUAL TIM module.
 * @param[in]  id:  A specific timer ID which can be APP_DUAL_TIM_ID_0 or APP_DUAL_TIM_ID_1
 *
 * @return Result of execution.
 ****************************************************************************************
 */
uint16_t app_dual_tim_set_params(app_dual_tim_params_t *p_params, app_dual_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Set the APP DUAL TIM background reload value
 *         The background reload value contains the value from which the counter is to decrement.
 *         This is the value used to reload the counter when Periodic mode is enabled, and the current count reaches 0.
 * @param[in]  id:  A specific timer ID which can be APP_DUAL_TIM_ID_0 or APP_DUAL_TIM_ID_1
 * @param[in]  reload_value: Background reload value
 *
 * @return Result of execution.
 ****************************************************************************************
 */
uint16_t app_dual_tim_set_background_reload(app_dual_tim_id_t id, uint32_t reload_value);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
/**
 ****************************************************************************************
 * @brief  Set the APP DUAL TIM one-time reload value
 *         The one-time reload value contains the value from which the counter is to decrement.
 *         This is the value used to reload the counter when Periodic mode is enabled, and the current count reaches 0.
 *         The difference is that writes to one-time reload value do not cause the counter to immediately restart from the new value.
 *         The one-time reload value only takes effect in the next cycle. It has higher priority than background reload.
 * @param[in]  id:  A specific timer ID which can be APP_DUAL_TIM_ID_0 or APP_DUAL_TIM_ID_1
 * @param[in]  reload_value: One-time reload value
 *
 * @return Result of execution.
 ****************************************************************************************
 */
uint16_t app_dual_tim_set_onetime_reload(app_dual_tim_id_t id, uint32_t reload_value);

/**
 ****************************************************************************************
 * @brief  DUAL TIMER set period count value
 *         The value is valid when the value is greater than 1. This function is only available in Period/Loop mode with IO control.
 *         This function call must be after app_dual_tim_io_crtl_config.
 *         This function will force the stop action interrupt to be turned on.
 *         If the current count value of the dual timer is not the period value, it may cause the first period to be shortened.
 * @param[in]  id:  A specific timer ID which can be APP_DUAL_TIM_ID_0 or APP_DUAL_TIM_ID_1
 * @param[in]  count_value: period count value
 *
 * @return Result of execution.
 ****************************************************************************************
 */
uint16_t app_dual_tim_set_period_count(app_dual_tim_id_t id, uint32_t count_value);

/**
 ****************************************************************************************
 * @brief  Configure the APP DUAL TIM io ctrl according to the specified parameters
 *         This function call must be after app_dual_tim_init. Reconfiguration needs to reinitialize APP DUAL TIM.
 *
 * @param[in]  id:  A specific timer ID which can be APP_DUAL_TIM_ID_0 or APP_DUAL_TIM_ID_1
 * @param[in]  io_crtl_params: Pointer to app_dual_tim_io_crtl_params_t parameter which contains the
 *                             configuration information for the specified io crtl of the DUAL TIM module.
 *
 * @return Result of execution.
 ****************************************************************************************
 */
uint16_t app_dual_tim_io_crtl_config(app_dual_tim_id_t id, app_dual_tim_io_crtl_params_t *io_crtl_params);
#endif

/**
 ****************************************************************************************
 * @brief  Return the DUAL TIM handle.
 *
 * @param[in]  id: DUAL TIM Channel ID.
 *
 * @return Pointer to the specified ID's DUAL TIM handle.
 ****************************************************************************************
 */
dual_timer_handle_t *app_dual_tim_get_handle(app_dual_tim_id_t id);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */


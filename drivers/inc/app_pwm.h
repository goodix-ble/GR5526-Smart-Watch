/**
 ****************************************************************************************
 *
 * @file    app_pwm.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PWM app library.
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

/** @defgroup APP_PWM  PWM
  * @brief PWM APP module driver.
  * @{
  */
#ifndef _APP_PWM_H_
#define _APP_PWM_H_

#include "grx_hal.h"
#include "app_io.h"
#include "app_drv_error.h"
#include "app_drv_config.h"
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
#include "app_dma.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_PWM_MODULE_ENABLED

/** @addtogroup APP_PWM_DEFINE Defines
  * @{
  */

#define APP_PWM_PIN_ENABLE      1    /**< PWM pin enable */
#define APP_PWM_PIN_DISABLE     0    /**< PWM pin disable */

/** @} */

/** @addtogroup APP_PWM_ENUM Enumerations
  * @{
  */

/**
  * @brief PWM module Enumerations definition
  */
typedef enum
{
    APP_PWM_ID_0,                /**< PWM module 0. */
    APP_PWM_ID_1,                /**< PWM module 1. */
    APP_PWM_ID_MAX               /**< Only for check parameter, not used as input parameters. */
} app_pwm_id_t;

/**
  * @brief PWM active channel Enumerations definition
  */
typedef enum
{
    APP_PWM_ACTIVE_CHANNEL_A        = 0x01,    /**< The active channel is A. */
    APP_PWM_ACTIVE_CHANNEL_B        = 0x02,    /**< The active channel is B. */
    APP_PWM_ACTIVE_CHANNEL_C        = 0x04,    /**< The active channel is C. */
    APP_PWM_ACTIVE_CHANNEL_ALL      = 0x07,    /**< The active channels are ALL. */
    APP_PWM_ACTIVE_CHANNEL_CLEARED  = 0x00     /**< All active channels are cleared. */
} app_pwm_active_channel_t;

/**@brief App pwm state types. */
typedef enum
{
    APP_PWM_INVALID = 0,
    APP_PWM_ACTIVITY,
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    APP_PWM_SLEEP,
#endif

} app_pwm_state_t;

/**@brief App pwm module state types. */
typedef enum
{
    APP_PWM_STOP = 0,
    APP_PWM_START,
} app_pwm_module_state_t;

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
/**
  * @brief PWM event Enumerations definition
  */
typedef enum
{
    APP_PWM_CHANNEL_A_ERROR,           /**< Channel A error in coding mode. */
    APP_PWM_CHANNEL_B_ERROR,           /**< Channel B error in coding mode. */
    APP_PWM_CHANNEL_C_ERROR,           /**< Channel C error in coding mode. */
    APP_PWM_CODING_DONE,               /**< Coding done in coding mode. */
    APP_PWM_CODING_LOAD,               /**< Coding load in coding mode. */
} app_pwm_evt_type_t;
#endif

/** @} */

/** @addtogroup APP_PWM_STRUCTURES Structures
  * @{
  */

/**
  * @brief PWM IO configuration Structures
  */
typedef struct
{
    app_io_type_t  type;         /**< Specifies the type of PWM IO. */
    app_io_mux_t   mux;          /**< Specifies the Peripheral to be connected to the selected pins. */
    uint32_t       pin;          /**< Specifies the IO pins to be configured.
                                      This parameter can be any value of @ref GR5xxx_pins. */
    app_io_pull_t  pull;         /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
    uint8_t        enable;       /**< Enable or disable the pin. */
} app_pwm_pin_t;

/**
  * @brief PWM configuration Structures
  */
typedef struct
{
    app_pwm_pin_t channel_a;     /**< Set the configuration of PWM channel A pin. */
    app_pwm_pin_t channel_b;     /**< Set the configuration of PWM channel B pin. */
    app_pwm_pin_t channel_c;     /**< Set the configuration of PWM channel C pin. */
} app_pwm_pin_cfg_t;

/**
  * @brief PWM Channel init Structure definition
  */
typedef struct
{
    uint8_t duty;               /**< Specifies the duty in PWM output mode.
                                     This parameter must be a number between 0 ~ 100.*/

    uint8_t drive_polarity;     /**< Specifies the drive polarity in PWM output mode.
                                     This parameter can be a value of @ref PWM_Drive_Polarity.*/
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
    uint32_t fstoplvl;          /**< Specifies the PWM io level when stop.
                                             This parameter can be a value of @ref PWM_STOP_LVL */
#endif
} app_pwm_channel_init_t;

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
/**
  * @brief PWM operate mode Enumerations definition
  */
typedef struct
{
    dma_regs_t       *pwm_dma_instance;  /**< Specifies the DMA instance.*/
    dma_channel_t    pwm_dma_channel;    /**< Specifies the dma channel of PWM. */
} app_pwm_mode_t;

/**@brief App pwm dma state types. */
typedef enum
{
    APP_PWM_DMA_INVALID = 0,
    APP_PWM_DMA_ACTIVITY,
} app_pwm_dma_state_t;

/**
  * @brief PWM event structure definition
  */
typedef struct
{
    app_pwm_evt_type_t type; /**< Type of event. */
    uint32_t error_code;     /**< Error code */
} app_pwm_evt_t;

/**
  * @brief PWM event callback definition
  */
typedef void (*app_pwm_evt_handler_t)(app_pwm_evt_t *p_evt);
#endif

/**
  * @brief PWM device structure definition
  */
typedef struct
{
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    app_pwm_evt_handler_t  evt_handler;
#endif
    app_pwm_pin_cfg_t      *p_pin_cfg;            /**< PWM configuration Structures. */
    app_pwm_state_t        pwm_state;             /**< App pwm state types. */
    app_pwm_module_state_t pwm_module_state;      /**< App pwm module state types. */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    app_pwm_dma_state_t    dma_state;             /**< App pwm dma state. */
    dma_id_t               dma_id[1];
#endif
    pwm_handle_t           handle;                /**< PWM handle Structure. */
} pwm_env_t;

/**
  * @brief PWM parameters structure definition
  */
typedef struct
{
    app_pwm_id_t             id;              /**< specified PWM module ID. */
    app_pwm_pin_cfg_t        pin_cfg;         /**< the pin configuration information for the specified PWM module. */
    app_pwm_active_channel_t active_channel;  /**< PWM operate mode. */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    app_pwm_mode_t           use_mode;        /**< PWM operate mode. */
#endif
    pwm_init_t               init;            /**< PWM communication parameters. */
    pwm_env_t                pwm_env;         /**< PWM device structure definition. */
} app_pwm_params_t;
/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_PWM_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the APP PWM DRIVER.
 *
 * @param[in]  p_params: Pointer to app_pwm_params_t parameter which contains the
 *                       configuration information for the specified PWM module.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
uint16_t app_pwm_init(app_pwm_params_t *p_params, app_pwm_evt_handler_t evt_handler);
#else
uint16_t app_pwm_init(app_pwm_params_t *p_params);
#endif

/**
 ****************************************************************************************
 * @brief  De-initialize the APP PWM DRIVER.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_pwm_deinit(app_pwm_id_t id);


/**
 ****************************************************************************************
 * @brief  Starts the PWM signal generation on the output.
 *
 * @param[in]  id: which PWM module want to output.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_pwm_start(app_pwm_id_t id);


/**
 ****************************************************************************************
 * @brief  Stops the PWM signal generation on the output.
 *
 * @param[in]  id: which PWM module want to stop output.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_pwm_stop(app_pwm_id_t id);

/**
 ****************************************************************************************
 * @brief  Update the PWM frequency on the output.
 *
 * @param[in]  id: which PWM module want to config.
 * @param[in]  freq: This parameter ranges between min = 0 and max = SystemFreq / 2.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_pwm_update_freq(app_pwm_id_t id, uint32_t freq);

/**
 ****************************************************************************************
 * @brief  Initialize the PWM channels according to the specified parameters.
 *
 * @param[in]  id: which PWM module want to config.
 * @param[in]  channel:  PWM Channels to be configured.
 * @param[in]  p_config: PWM Channels configuration structure.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_pwm_config_channel(app_pwm_id_t id, app_pwm_active_channel_t channel, app_pwm_channel_init_t *p_config);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
/**
 ****************************************************************************************
 * @brief  Resume the PWM signal generation on the output.
 *
 * @param[in]  id: which PWM module want to output.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_pwm_resume(app_pwm_id_t id);

/**
 ****************************************************************************************
 * @brief  Pause the PWM signal generation on the output.
 *
 * @param[in]  id: which PWM module want to stop output.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_pwm_pause(app_pwm_id_t id);
#endif

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
/**
 ****************************************************************************************
 * @brief  Set the specified PWM channel inactive
 *
 * @param[in]  id: PWM ID.
 * @param[in]  channel: PWM Channels to be configured.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_pwm_inactive_channel(app_pwm_id_t id, app_pwm_active_channel_t channel);
#endif

/**
 ****************************************************************************************
 * @brief  Return the PWM handle.
 *
 * @param[in]  id: PWM ID.
 *
 * @return Pointer to the specified ID's PWM handle.
 ****************************************************************************************
 */
pwm_handle_t *app_pwm_get_handle(app_pwm_id_t id);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
/**
 ****************************************************************************************
 * @brief  Set coding data in one channel mode
 *
 * @param[in]  id: which PWM module want to config.
 * @param[in]  coding_data: every bit represent compa0 or compa1 in coding data.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_pwm_set_coding_data_in_one_channel(app_pwm_id_t id, uint32_t coding_data);

/**
 ****************************************************************************************
 * @brief  Set coding data in three channels mode
 *
 * @param[in]  id: which PWM module want to config.
 * @param[in]  coding_data0: every bit represent compa0 or compa1 in coding data0.
 * @param[in]  coding_data1: every bit represent compb0 or compb1 in coding data1.
 * @param[in]  coding_data2: every bit represent compc0 or compc1 in coding data2.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_pwm_set_coding_data_in_three_channels(app_pwm_id_t id, uint32_t coding_data0, uint32_t coding_data1, uint32_t coding_data2);

/**
 ****************************************************************************************
 * @brief  Start generate one channel wave form in Interrupt mode
 *
 * @param[in]  id: which PWM module want to config.
 * @param[in]  coding_data: every bit represent compa0 or compa1 in coding data.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_pwm_start_coding_in_one_channel(app_pwm_id_t id, uint32_t coding_data);

/**
 ****************************************************************************************
 * @brief  Start generate three channels wave form in Interrupt mode
 *
 * @param[in]  id: which PWM module want to config.
 * @param[in]  coding_data0: every bit represent compa0 or compa1 in coding data0.
 * @param[in]  coding_data1: every bit represent compb0 or compb1 in coding data1.
 * @param[in]  coding_data2: every bit represent compc0 or compc1 in coding data2.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_pwm_start_coding_in_three_channels(app_pwm_id_t id, uint32_t coding_data0, uint32_t coding_data1, uint32_t coding_data2);
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


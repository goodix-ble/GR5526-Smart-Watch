/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_pwm.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PWM HAL library.
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

/** @addtogroup HAL_DRIVER HAL Driver
  * @{
  */

/** @defgroup HAL_PWM PWM
  * @brief PWM HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_PWM_H__
#define __GR55xx_HAL_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"
#include "gr55xx_ll_pwm.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_PWM_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_PWM_state HAL PWM state
  * @{
  */

/**
  * @brief HAL PWM State Enumerations definition
  */
typedef enum
{
    HAL_PWM_STATE_RESET             = 0x00,    /**< Peripheral is not initialized or disabled   */
    HAL_PWM_STATE_READY             = 0x01,    /**< Peripheral is initialized and ready for use */
    HAL_PWM_STATE_BUSY              = 0x02,    /**< An internal process is ongoing              */
    HAL_PWM_STATE_ERROR             = 0x04     /**< Reception process is ongoing                */
} hal_pwm_state_t;
/** @} */

/** @defgroup HAL_PWM_active_channel HAL PWM active channel
  * @{
  */

/**
  * @brief HAL PWM active channel Enumerations definition
  */
typedef enum
{
    HAL_PWM_ACTIVE_CHANNEL_A        = 0x01,    /**< The active channel is A     */
    HAL_PWM_ACTIVE_CHANNEL_B        = 0x02,    /**< The active channel is B     */
    HAL_PWM_ACTIVE_CHANNEL_C        = 0x04,    /**< The active channel is C     */
    HAL_PWM_ACTIVE_CHANNEL_ALL      = 0x07,    /**< The active channels are ALL */
    HAL_PWM_ACTIVE_CHANNEL_CLEARED  = 0x00     /**< All active channels are cleared */
} hal_pwm_active_channel_t;
/** @} */

/** @} */

/** @addtogroup HAL_PWM_STRUCTURES Structures
  * @{
  */

/** @defgroup PWM_Configuration PWM Configuration
  * @{
  */

/**
  * @brief PWM Channel init Structure definition
  */
typedef struct
{
    uint8_t duty;               /**< Specifies the duty in PWM output mode.
                                     This parameter must be a number between 0 ~ 100.*/

    uint8_t drive_polarity;     /**< Specifies the drive polarity in PWM output mode.
                                     This parameter can be a value of @ref PWM_Drive_Polarity.*/

    uint32_t fstoplvl;          /**< Specifies the PWM io level when stop.
                                             This parameter can be a value of @ref PWM_STOP_LVL */
} pwm_channel_init_t;

/**
  * @brief PWM init Structure definition
  */
typedef struct
{
    uint32_t mode;                      /**< Specifies the PWM output mode state.
                                             This parameter can be a value of @ref PWM_Mode */

    uint32_t align;                     /**< Specifies the PWM alignment mode with three channels
                                             This parameter can be a value of @ref PWM_Alignment_Mode */

    uint32_t freq;                      /**< Specifies the PWM frequency.
                                             This parameter must be a number between 0 ~ SystemFreq/2 (max = 32Mhz).*/

    uint32_t bperiod;                   /**< Specifies the PWM breath period in breath mode. Unit: ms.
                                             This parameter must be a number between 0 ~ 0xFFFFFFFF/SystemFreq*1000. */

    uint32_t hperiod;                   /**< Specifies the PWM hold period in breath mode. Unit: ms.
                                             This parameter must be a number between 0 ~ 0xFFFFFF/SystemFreq*1000. */

    uint32_t bstoplvl;                   /**< Specifies the PWM io level when stop.
                                             This parameter can be a value of @ref PWM_STOP_LVL */

    pwm_channel_init_t channel_a;       /**< Specifies the configuration parameters of channel A. */

    pwm_channel_init_t channel_b;       /**< Specifies the configuration parameters of channel B. */

    pwm_channel_init_t channel_c;       /**< Specifies the configuration parameters of channel C. */

} pwm_init_t;
/** @} */

/** @defgroup PWM_handle PWM handle
  * @{
  */

/**
  * @brief PWM handle Structure definition
  */
typedef struct
{
    pwm_regs_t                *p_instance;      /**< Register base address        */

    pwm_init_t                init;           /**< Required parameters for PWM Base */

    hal_pwm_active_channel_t  active_channel; /**< Active channel               */

    __IO hal_lock_t           lock;           /**< Lock object                  */

    __IO hal_pwm_state_t      state;          /**< PWM operation state          */

    uint32_t                  retention[12];  /**< PWM important register information. */

} pwm_handle_t;

/** @} */

/** @} */

/** @addtogroup HAL_PWM_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup PWM_Callback PWM Callback
  * @{
  */

/**
  * @brief HAL_PWM Callback function definition
  */

typedef struct _pwm_callback
{
    void (*pwm_msp_init)(pwm_handle_t *p_pwm);      /**< PWM init MSP callback                  */
    void (*pwm_msp_deinit)(pwm_handle_t *p_pwm);    /**< PWM de-init MSP callback               */
} pwm_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_PWM_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PWM_Exported_Constants PWM Exported Constants
  * @{
  */

/** @defgroup PWM_Mode PWM Mode
  * @{
  */
#define PWM_MODE_FLICKER                LL_PWM_FLICKER_MODE     /**< PWM flicker mode */
#define PWM_MODE_BREATH                 LL_PWM_BREATH_MODE      /**< PWM breath mode  */
/** @} */

/** @defgroup PWM_Alignment_Mode PWM Pulses Aligned Mode
  * @{
  */
#define PWM_ALIGNED_EDGE                LL_PWM_EDGE_ALIGNED     /**< PWM edge-aligned */
#define PWM_ALIGNED_CENTER              LL_PWM_CENTER_ALIGNED   /**< PWM center-aligned */
/** @} */

/** @defgroup PWM_STOP_LVL PWM Stop Level
  * @{
  */
#define PWM_STOP_LVL_LOW                LL_PWM_STOP_LVL_LOW     /**< PWM stop in low io level */
#define PWM_STOP_LVL_HIGH               LL_PWM_STOP_LVL_HIGH    /**< PWM stop in high io level */
/** @} */

/** @defgroup PWM_Drive_Polarity PWM Drive Polarity
  * @{
  */
#define PWM_DRIVEPOLARITY_NEGATIVE      LL_PWM_DRIVEPOLARITY_NEGATIVE   /**< PWM led-negative-drive mode */
#define PWM_DRIVEPOLARITY_POSITIVE      LL_PWM_DRIVEPOLARITY_POSITIVE   /**< PWM led-positive-drive mode */
/** @} */
/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup PWM_Exported_Macros PWM Exported Macros
  * @{
  */

/** @brief  Reset PWM handle states.
  * @param  __HANDLE__ PWM handle.
  * @retval None
  */
#define __HAL_PWM_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_PWM_STATE_RESET)

/** @brief  Enable the specified PWM peripheral.
  * @param  __HANDLE__ specifies the PWM Handle.
  * @retval None
  */
#define __HAL_PWM_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->MODE, PWM_MODE_EN)

/** @brief  Disable the specified PWM peripheral.
  * @param  __HANDLE__ specifies the PWM Handle.
  * @retval None
  */
#define __HAL_PWM_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->MODE, PWM_MODE_EN)

/** @brief  Enable PWM breath mode.
  * @param  __HANDLE__ specifies the PWM Handle.
  * @retval None
  */
#define __HAL_PWM_ENABLE_BREATH(__HANDLE__)                    SET_BITS((__HANDLE__)->p_instance->MODE, PWM_MODE_BREATHEN)

/** @brief  Disable PWM breath mode.
  * @param  __HANDLE__ specifies the PWM Handle.
  * @retval None
  */
#define __HAL_PWM_DISABLE_BREATH(__HANDLE__)                   CLEAR_BITS((__HANDLE__)->p_instance->MODE, PWM_MODE_BREATHEN)

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup PWM_Private_Macro PWM Private Macros
  * @{
  */

/**
  * @brief Check if PWM mode is valid.
  * @param __MODE__ PWM mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_PWM_MODE(__MODE__)                                   (((__MODE__) == PWM_MODE_FLICKER) || \
                                                                 ((__MODE__) == PWM_MODE_BREATH))

/**
  * @brief Check if PWM Alignment mode is valid.
  * @param __MODE__ PWM Alignment mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_PWM_ALIGNMENT_MODE(__MODE__)                          (((__MODE__) == PWM_ALIGNED_EDGE) || \
                                                                  ((__MODE__) == PWM_ALIGNED_CENTER))

/**
  * @brief Check if PWM stop level is valid.
  * @param __MODE__ PWM stop level.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_PWM_STOP_LVL(__MODE__)                                (((__MODE__) == PWM_STOP_LVL_LOW) || \
                                                                  ((__MODE__) == PWM_STOP_LVL_HIGH))

/**
  * @brief Check if PWM drive polarity is valid.
  * @param __POLARITY__ PWM drive polarity.
  * @retval SET (__POLARITY__ is valid) or RESET (__POLARITY__ is invalid)
  */
#define IS_PWM_DRIVEPOLARITY(__POLARITY__)                      (((__POLARITY__) == PWM_DRIVEPOLARITY_NEGATIVE) || \
                                                                 ((__POLARITY__) == PWM_DRIVEPOLARITY_POSITIVE))

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_PWM_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup PWM_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and de-initialization functions
  *
  * @verbatim
===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to initialize the PWMx.
      (+) The parameters below can only be configured in breath mode:
        (++) BreathPeriod
        (++) HoldPeriod

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief Initialize the PWM mode according to the specified
 *        parameters in the pwm_init_t and initialize the associated handle.
 * @param[in]  p_pwm: Pointer to a PWM handle that contains the configuration information for the specified PWM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwm_init(pwm_handle_t *p_pwm);

/**
 ****************************************************************************************
 * @brief  De-initialize the PWM peripheral.
 * @param[in]  p_pwm: Pointer to a PWM handle that contains the configuration information for the specified PWM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwm_deinit(pwm_handle_t *p_pwm);

/**
 ****************************************************************************************
 * @brief Initialize the PWM MSP.
 * @note  This function should not be modified. When the callback is needed,
           the hal_pwm_msp_init can be implemented in the user file.
 * @param[in]  p_pwm: Pointer to a PWM handle that contains the configuration information for the specified PWM module.
 ****************************************************************************************
 */
void hal_pwm_msp_init(pwm_handle_t *p_pwm);

/**
 ****************************************************************************************
 * @brief De-initialize the PWM MSP.
 * @note  This function should not be modified. When the callback is needed,
           the hal_pwm_msp_deinit can be implemented in the user file.
 * @param[in]  p_pwm: Pointer to a PWM handle that contains the configuration information for the specified PWM module.
 ****************************************************************************************
 */
void hal_pwm_msp_deinit(pwm_handle_t *p_pwm);

/** @} */

/** @addtogroup PWM_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions
 *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Start the PWM.
    (+) Stop the PWM.
    (+) Configure the specified PWM channel.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Starts the PWM signal generation on the output.
 * @param[in]  p_pwm: Pointer to a PWM handle that contains the configuration information for the specified PWM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwm_start(pwm_handle_t *p_pwm);

/**
 ****************************************************************************************
 * @brief  Stops the PWM signal generation on the output.
 * @param[in]  p_pwm: Pointer to a PWM handle that contains the configuration information for the specified PWM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwm_stop(pwm_handle_t *p_pwm);

/**
 ****************************************************************************************
 * @brief  Update the PWM frequency on the output.
 * @param[in]  p_pwm: Pointer to a PWM handle that contains the configuration information for the specified PWM module.
 * @param[in]  freq: This parameter ranges between min = 0 and max = SystemFreq / 2.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwm_update_freq(pwm_handle_t *p_pwm, uint32_t freq);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to PWM configuration before sleep.
 * @param[in] p_pwm: Pointer to a PWM handle which contains the configuration
 *                 information for the specified PWM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwm_suspend_reg(pwm_handle_t *p_pwm);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to PWM configuration after sleep.
 *         This function must be used in conjunction with the hal_pwm_suspend_reg().
 * @param[in] p_pwm: Pointer to a PWM handle which contains the configuration
 *                 information for the specified PWM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwm_resume_reg(pwm_handle_t *p_pwm);

/**
 ****************************************************************************************
 * @brief  Initialize the PWM  channels according to the specified
 *         parameters in the pwm_init_t.
 * @param[in]  p_pwm: Pointer to a PWM handle that contains the configuration information for the specified PWM module.
 * @param[in]  p_config: PWM Channels configuration structure.
 * @param[in]  channel: PWM Channels to be configured.
 *          This parameter can be one of the following values:
 *            @arg @ref HAL_PWM_ACTIVE_CHANNEL_A        :PWM Channel A is active
 *            @arg @ref HAL_PWM_ACTIVE_CHANNEL_B        :PWM Channel B is active
 *            @arg @ref HAL_PWM_ACTIVE_CHANNEL_C        :PWM Channel C is active
 *            @arg @ref HAL_PWM_ACTIVE_CHANNEL_ALL      :All Channels are active
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwm_config_channel(pwm_handle_t *p_pwm, pwm_channel_init_t *p_config, hal_pwm_active_channel_t channel);

/**
 ****************************************************************************************
 * @brief  Set the specified PWM channel inactive
 * @param[in]  p_pwm: Pointer to a PWM handle that contains the configuration information for the specified PWM module.
 * @param[in]  channel: PWM Channels to be configured.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwm_inactive_channel(pwm_handle_t *p_pwm, hal_pwm_active_channel_t channel);

/** @} */

/** @addtogroup PWM_Exported_Functions_Group3 Peripheral Control and State functions
 *  @brief   PWM Peripheral State functions
 *
@verbatim
  ==============================================================================
            ##### Peripheral Control and State functions #####
  ==============================================================================
    [..]
    This subsection provides functions allowing to :
      (+) Return the PWM handle state.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the PWM handle state.
 * @param[in]  p_pwm: Pointer to a PWM handle that contains the configuration
 *               information for the specified PWM module.
 * @retval ::HAL_PWM_STATE_RESET: Peripheral is not initialized or disabled.
 * @retval ::HAL_PWM_STATE_READY: Peripheral is initialized and ready for use.
 * @retval ::HAL_PWM_STATE_BUSY: An internal process is ongoing.
 * @retval ::HAL_PWM_STATE_ERROR: Reception process is ongoing.
 ****************************************************************************************
 */
hal_pwm_state_t hal_pwm_get_state(pwm_handle_t *p_pwm);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_PWM_H__ */

/** @} */

/** @} */

/** @} */

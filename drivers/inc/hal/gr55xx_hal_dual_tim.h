/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_dual_tim.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of DUAL TIMER HAL library.
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

/** @defgroup HAL_DUAL_TIMER DUAL TIMER
  * @brief DUAL TIM HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_DUAL_TIMER_H__
#define __GR55xx_HAL_DUAL_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"
#include "gr55xx_ll_dual_tim.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_DUAL_TIMER_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_DUAL_TIMER_state HAL DUAL TIM state
  * @{
  */

/**
  * @brief  HAL DUAL TIMER State Enumerations definition
  */
typedef enum
{
    HAL_DUAL_TIMER_STATE_RESET             = 0x00,    /**< Peripheral not yet initialized or disabled  */
    HAL_DUAL_TIMER_STATE_READY             = 0x01,    /**< Peripheral Initialized and ready for use    */
    HAL_DUAL_TIMER_STATE_BUSY              = 0x02,    /**< An internal process is ongoing              */
    HAL_DUAL_TIMER_STATE_ERROR             = 0x04     /**< Reception process is ongoing                */
} hal_dual_timer_state_t;
/** @} */

/** @} */

/** @addtogroup HAL_DUAL_TIMER_STRUCTURES Structures
  * @{
  */

/** @defgroup DUAL_TIMER_Configuration DUAL TIMER Configuration
  * @{
  */

/**
  * @brief DUAL TIMER init Structure definition
  */
typedef struct _dual_timer_init
{
    uint32_t prescaler;     /**< Specifies the prescaler value used to divide the DUAL_TIMER clock.
                                 This parameter can be a value of @ref DUAL_TIMER_Prescaler_Div */

    uint32_t counter_mode;  /**< Specifies the counter mode.
                                 This parameter can be a value of @ref DUAL_TIMER_Counter_Mode */

    uint32_t auto_reload;   /**< Specifies the auto-reload value. */

} dual_timer_init_t;

/** @} */

/** @defgroup DUAL_TIMER_handle DUAL TIMER handle
  * @{
  */

/**
  * @brief DUAL_TIMER handle Structure definition
  */
typedef struct _dual_timer_handle
{
    dual_timer_regs_t              *p_instance;     /**< Register base address               */

    dual_timer_init_t              init;          /**< DUAL_TIMER Base required parameters   */

    __IO hal_lock_t                lock;          /**< Locking object                      */

    __IO hal_dual_timer_state_t    state;         /**< DUAL_TIMER operation state            */

} dual_timer_handle_t;
/** @} */

/** @} */

/** @addtogroup HAL_DUAL_TIMER_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup DUAL_TIMER_Callback DUAL_TIMER Callback
  * @{
  */

/**
  * @brief HAL_DUAL_TIMER Callback function definition
  */
typedef struct _dual_timer_callback
{
    void (*dual_timer_msp_init)(dual_timer_handle_t *p_dual_timer);                 /**< DUAL_TIMER init MSP callback       */
    void (*dual_timer_msp_deinit)(dual_timer_handle_t *p_dual_timer);               /**< DUAL_TIMER de-init MSP callback    */
    void (*dual_timer_period_elapsed_callback)(dual_timer_handle_t *p_dual_timer);  /**< DUAL_TIMER period elapsed callback */
} dual_timer_callback_t;
/** @} */

/** @} */

/**
  * @defgroup  HAL_DUAL_TIMER_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DUAL_TIMER_Exported_Constants DUAL TIMER Exported Constants
  * @{
  */

/** @defgroup DUAL_TIMER_Prescaler_Div DUAL TIMER Prescaler Division
  * @{
  */
#define DUAL_TIMER_PRESCALER_DIV0         LL_DUAL_TIMER_PRESCALER_DIV0      /**< 0 stage of prescale, clock is divided by 1.   */
#define DUAL_TIMER_PRESCALER_DIV16        LL_DUAL_TIMER_PRESCALER_DIV16     /**< 4 stages of prescale, clock is divided by 16.  */
#define DUAL_TIMER_PRESCALER_DIV256       LL_DUAL_TIMER_PRESCALER_DIV256    /**< 8 stages of prescale, clock is divided by 256. */
/** @} */

/** @defgroup DUAL_TIMER_Counter_Mode DUAL TIMER Counter Mode
  * @{
  */
#define DUAL_TIMER_COUNTERMODE_LOOP       0x00000000U                     /**< DUAL TIMER Loop mode.*/
#define DUAL_TIMER_COUNTERMODE_ONESHOT    DUAL_TIMER_CTRL_ONESHOT           /**< DUAL TIMER One-shot mode. */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DUAL_TIMER_Exported_Macros DUAL TIMER Exported Macros
  * @{
  */

/** @brief  Reset DUAL TIMER handle states.
  * @param  __HANDLE__ DUAL TIMER handle.
  * @retval None
  */
#define __HAL_DUAL_TIMER_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_DUAL_TIMER_STATE_RESET)

/** @brief  Enable the specified DUAL TIMER peripheral.
  * @param  __HANDLE__ Specifies the DUAL TIMER Handle.
  * @retval None
  */
#define __HAL_DUAL_TIMER_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->CTRL, DUAL_TIMER_CTRL_EN)

/** @brief  Disable the specified DUAL TIMER peripheral.
  * @param  __HANDLE__ Specifies the DUAL TIMER Handle.
  * @retval None
  */
#define __HAL_DUAL_TIMER_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->CTRL, DUAL_TIMER_CTRL_EN)

/** @brief  Enable the DUAL TIMER interrupt.
  * @param  __HANDLE__ Specifies the DUAL TIM Handle.
  * @retval None
  */
#define __HAL_DUAL_TIMER_ENABLE_IT(__HANDLE__)                        SET_BITS((__HANDLE__)->p_instance->CTRL, DUAL_TIMER_CTRL_INTEN)

/** @brief  Disable the DUAL TIMER interrupt.
  * @param  __HANDLE__ Specifies the DUAL TIM Handle.
  * @retval None
  */
#define __HAL_DUAL_TIMER_DISABLE_IT(__HANDLE__)                       CLEAR_BITS((__HANDLE__)->p_instance->CTRL, DUAL_TIMER_CTRL_INTEN)

/** @brief  Check whether the DUAL TIMER interrupt has occurred or not.
  * @param  __HANDLE__ Specifies the DUAL TIMER Handle.
  * @retval The new state of DUAL TIMER interrupt (SET or RESET).
  */
#define __HAL_DUAL_TIMER_GET_FLAG_IT(__HANDLE__)                      ll_dual_timer_is_active_flag_it(__HANDLE__->p_instance)

/** @brief  Clear the DUAL TIMER interrupt flag.
  * @param  __HANDLE__ Specifies the DUAL TIMER Handle.
  * @retval None.
  */
#define __HAL_DUAL_TIMER_CLEAR_FLAG_IT(__HANDLE__)                    ll_dual_timer_clear_flag_it(__HANDLE__->p_instance)

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup DUAL_TIMER_Private_Macros DUAL TIMER Private Macros
  * @{
  */

/** @brief  Check if DUAL TIMER prescaler is valid.
  * @param  __PRESCALER__ DUAL TIMER prescaler.
  * @retval SET (__PRESCALER__ is valid) or RESET (__PRESCALER__ is invalid)
  */
#define IS_DUAL_TIMER_PRESCALER(__PRESCALER__)                        (((__PRESCALER__) == DUAL_TIMER_PRESCALER_DIV0)  || \
                                                                     ((__PRESCALER__) == DUAL_TIMER_PRESCALER_DIV16) || \
                                                                     ((__PRESCALER__) == DUAL_TIMER_PRESCALER_DIV256))

/** @brief  Check if DUAL TIMER counter mode is valid.
  * @param  __MODE__ DUAL TIMER counter mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_DUAL_TIMER_COUNTERMODE(__MODE__)                           (((__MODE__) == DUAL_TIMER_COUNTERMODE_LOOP)  || \
                                                                     ((__MODE__) == DUAL_TIMER_COUNTERMODE_ONESHOT))
/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_DUAL_TIMER_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup DUAL_TIMER_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and de-initialization functions
  *
  * @verbatim
===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]
        This section provides functions allowing to:
        (+) Initialize and configure the DUAL TIMER.
        (+) De-initialize the DUAL TIMER.
        (+) Start the Timer.
        (+) Stop the Timer.
        (+) Start the Timer and enable interrupt.
        (+) Stop the Timer and disable interrupt.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the DUAL TIMER according to the specified parameters
 *         in the dual_timer_init_t and initialize the associated handle.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIMER handle which contains the configuration information for the specified DUAL TIMER.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_timer_base_init(dual_timer_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  De-initialize the DUAL TIMER peripheral.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIMER.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_timer_base_deinit(dual_timer_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief Initialize the DUAL TIMER MSP.
 *
 * @note  This function should not be modified. When the callback is needed,
 *         the hal_dual_timer_base_msp_init could be implemented in the user file
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIMER handle which contains the configuration information for the specified DUAL TIMER.
 ****************************************************************************************
 */
void hal_dual_timer_base_msp_init(dual_timer_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief De-initialize the DUAL TIMER MSP.
 *
 * @note  This function should not be modified. When the callback is needed,
 *         the hal_dual_timer_base_msp_deinit could be implemented in the user file
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIMER.
 ****************************************************************************************
 */
void hal_dual_timer_base_msp_deinit(dual_timer_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  Starts the DUAL TIMER counter.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIMER handle which contains the configuration information for the specified DUAL TIMER.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_timer_base_start(dual_timer_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  Stops the DUAL TIMER counter.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIMER.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_timer_base_stop(dual_timer_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  Starts the DUAL TIMER counter in interrupt mode.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIMER.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_timer_base_start_it(dual_timer_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  Stops the DUAL TIMER counter in interrupt mode.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIMER handle which contains the configuration information for the specified DUAL TIMER.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_timer_base_stop_it(dual_timer_handle_t *p_dual_timer);

/** @} */

/** @addtogroup DUAL_TIMER_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief Handle DUAL TIMER interrupt request.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIMER handle which contains the configuration information for the specified DUAL TIMER.
 ****************************************************************************************
 */
void hal_dual_timer_irq_handler(dual_timer_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  Period elapsed callback in non-blocking mode.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_dual_timer_period_elapsed_callback can be implemented in the user file.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIMER handle which contains the configuration information for the specified DUAL TIMER.
 ****************************************************************************************
 */
void hal_dual_timer_period_elapsed_callback(dual_timer_handle_t *p_dual_timer);

/** @} */

/** @addtogroup DUAL_TIMER_Exported_Functions_Group2 Peripheral Control and State functions
 *  @brief   DUAL TIMER Peripheral State functions
 *
@verbatim
  ==============================================================================
            ##### Peripheral Control and State functions #####
  ==============================================================================
    [..]
    This subsection provides functions allowing to :
      (+) Return the DUAL TIMER handle state.
      (+) Configure the DUAL TIMER.
      (+) Set background reload value.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the DUAL TIMER handle state.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIMER handle which contains the configuration information for the specified DUAL TIMER.
 *
 * @retval ::HAL_DUAL_TIMER_STATE_RESET: Peripheral not yet initialized or disabled.
 * @retval ::HAL_DUAL_TIMER_STATE_READY: Peripheral Initialized and ready for use.
 * @retval ::HAL_DUAL_TIMER_STATE_BUSY: An internal process is ongoing.
 * @retval ::HAL_DUAL_TIMER_STATE_ERROR: Reception process is ongoing.
 ****************************************************************************************
 */
hal_dual_timer_state_t hal_dual_timer_get_state(dual_timer_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  DUAL TIMER configuration
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIMER handle which contains the configuration information for the specified DUAL TIMER.
 * @param[in]  p_structure: The DUAL TIMER configuration structure
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_timer_set_config(dual_timer_handle_t *p_dual_timer, dual_timer_init_t *p_structure);

/**
 ****************************************************************************************
 * @brief  DUAL TIMER set background reload value
 *         The background reload value contains the value from which the counter is to decrement.
 *         This is the value used to reload the counter when Periodic mode is enabled, and the current count reaches 0.
 *         The difference is that writes to background reload value do not cause the counter to immediately restart from the new value.
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIMER handle which contains the configuration information for the specified DUAL TIMER.
 * @param[in]  reload_value: Background reload value
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 ****************************************************************************************
 */
hal_status_t hal_dual_timer_set_background_reload(dual_timer_handle_t *p_dual_timer, uint32_t reload_value);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_DUAL_TIMER_H__ */

/** @} */

/** @} */

/** @} */

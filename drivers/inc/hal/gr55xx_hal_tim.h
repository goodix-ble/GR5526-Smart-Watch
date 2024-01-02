/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_tim.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of TIMER HAL library.
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

/** @defgroup HAL_TIMER TIMER
  * @brief TIM HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_TIMER_H__
#define __GR55xx_HAL_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"
#include "gr55xx_ll_tim.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_TIMER_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_TIMER_state HAL TIMER state
  * @{
  */

/**
  * @brief  HAL TIMER State Enumerations definition
  */
typedef enum
{
    HAL_TIMER_STATE_RESET             = 0x00,    /**< Peripheral not yet initialized or disabled  */
    HAL_TIMER_STATE_READY             = 0x01,    /**< Peripheral Initialized and ready for use    */
    HAL_TIMER_STATE_BUSY              = 0x02,    /**< An internal process is ongoing              */
    HAL_TIMER_STATE_ERROR             = 0x04     /**< Reception process is ongoing                */
} hal_timer_state_t;
/** @} */

/** @} */

/** @addtogroup HAL_TIMER_STRUCTURES Structures
  * @{
  */

/** @defgroup TIMER_Configuration TIMER Configuration
  * @{
  */

/**
  * @brief TIMER init Structure definition
  */
typedef struct _timer_init
{
    uint32_t auto_reload;                   /**< Specifies the auto-reload value. */

} timer_init_t;

/** @} */

/** @defgroup TIMER_handle TIMER handle
  * @{
  */

/**
  * @brief TIMER handle Structure definition
  */
typedef struct _timer_handle
{
    timer_regs_t               *p_instance;     /**< Register base address        */

    timer_init_t               init;            /**< TIMER Base required parameters */

    __IO hal_lock_t          lock;              /**< Locking object               */

    __IO hal_timer_state_t     state;           /**< TIMER operation state          */

} timer_handle_t;
/** @} */

/** @} */

/** @addtogroup HAL_TIMER_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup TIMER_Callback TIMER Callback
  * @{
  */

/**
  * @brief HAL_TIMER Callback function definition
  */

typedef struct _timer_base_callback
{
    void (*timer_msp_init)(timer_handle_t *p_timer);                /**< TIMER init MSP callback            */
    void (*timer_msp_deinit)(timer_handle_t *p_timer);              /**< TIMER de-init MSP callback         */
    void (*timer_period_elapsed_callback)(timer_handle_t *p_timer); /**< TIMER period elapsed callback      */
} timer_base_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_TIMER_MACRO Defines
  * @{
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup TIMER_Exported_Macros TIMER Exported Macros
  * @{
  */

/** @brief  Reset TIMER handle states.
  * @param  __HANDLE__ TIMER handle.
  * @retval None
  */
#define __HAL_TIMER_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_TIMER_STATE_RESET)

/** @brief  Enable the specified TIMER peripheral.
  * @param  __HANDLE__ Specifies the TIMER Handle.
  * @retval None
  */
#define __HAL_TIMER_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->CTRL, TIMER_CTRL_EN)

/** @brief  Disable the specified TIMER peripheral.
  * @param  __HANDLE__ Specifies the TIMER Handle.
  * @retval None
  */
#define __HAL_TIMER_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->CTRL, TIMER_CTRL_EN)

/** @brief  Enable the TIMER interrupt.
  * @param  __HANDLE__ Specifies the TIMER Handle.
  * @retval None
  */
#define __HAL_TIMER_ENABLE_IT(__HANDLE__)                        SET_BITS((__HANDLE__)->p_instance->CTRL, TIMER_CTRL_INTEN)

/** @brief  Disable the TIMER interrupt.
  * @param  __HANDLE__ Specifies the TIMER Handle.
  * @retval None
  */
#define __HAL_TIMER_DISABLE_IT(__HANDLE__)                       CLEAR_BITS((__HANDLE__)->p_instance->CTRL, TIMER_CTRL_INTEN)

/** @brief  Check whether the TIMER interrupt has occurred or not.
  * @param  __HANDLE__ Specifies the TIMER Handle.
  * @retval The new state of TIMER interrupt (SET or RESET).
  */
#define __HAL_TIMER_GET_FLAG_IT(__HANDLE__)                      ll_timer_is_active_flag_it(__HANDLE__->p_instance)

/** @brief  Clear the TIMER interrupt flag.
  * @param  __HANDLE__ Specifies the TIMER Handle.
  * @retval None
  */
#define __HAL_TIMER_CLEAR_FLAG_IT(__HANDLE__)                    ll_timer_clear_flag_it(__HANDLE__->p_instance)

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_TIMER_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup TIMER_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and de-initialization functions
  *
  * @verbatim
===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]
        This section provides functions allowing to:
        (+) Initialize and configure the TIMER.
        (+) De-initialize the TIMER.
        (+) Start the Timer.
        (+) Stop the Timer.
        (+) Start the Timer and enable interrupt.
        (+) Stop the Timer and disable interrupt.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the TIMER according to the specified parameters
 *         in the timer_init_t and initialize the associated handle.
 * @param[in]  p_timer: Pointer to a TIMER handle which contains the configuration
 *                 information for the specified TIMER module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_timer_base_init(timer_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  De-initialize the TIMER peripheral.
 * @param[in]  p_timer: Pointer to a TIMER handle which contains the configuration
 *                 information for the specified TIMER module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_timer_base_deinit(timer_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  Initialize the TIMER MSP.
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_timer_base_msp_init could be implemented in the user file.
 * @param[in]  p_timer: Pointer to a TIMER handle which contains the configuration
 *                 information for the specified TIMER module.
 ****************************************************************************************
 */
void hal_timer_base_msp_init(timer_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  De-initialize the TIMER MSP.
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_timer_base_msp_deinit could be implemented in the user file.
 * @param[in]  p_timer: Pointer to a TIM handle which contains the configuration
 *                 information for the specified TIMER module.
 ****************************************************************************************
 */
void hal_timer_base_msp_deinit(timer_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  Starts the TIMER counter.
 * @param[in]  p_timer: Pointer to a TIMER handle which contains the configuration
 *                 information for the specified TIMER module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_timer_base_start(timer_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  Stops the TIMER counter.
 * @param[in]  p_timer: Pointer to a TIMER handle which contains the configuration
 *                 information for the specified TIMER module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_timer_base_stop(timer_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  Starts the TIMER counter in interrupt mode.
 * @param[in]  p_timer: Pointer to a TIMER handle which contains the configuration
 *                 information for the specified TIMER module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_timer_base_start_it(timer_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  Stops the TIMER counter in interrupt mode.
 * @param[in]  p_timer: Pointer to a TIMER handle which contains the configuration
 *                 information for the specified TIMER module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_timer_base_stop_it(timer_handle_t *p_timer);

/** @} */

/** @addtogroup TIMER_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief Handle TIMER interrupt request.
 * @param[in] p_timer: TIMER handle.
 ****************************************************************************************
 */
void hal_timer_irq_handler(timer_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  Period elapsed callback in non-blocking mode.
 * @note   This function should not be modified. When the callback is needed,
            the hal_timer_period_elapsed_callback can be implemented in the user file.
 * @param[in]  p_timer: Pointer to a TIMER handle which contains the configuration
 *                 information for the specified TIMER module.
 ****************************************************************************************
 */
void hal_timer_period_elapsed_callback(timer_handle_t *p_timer);

/** @} */

/** @addtogroup TIMER_Exported_Functions_Group2 Peripheral Control and State functions
 *  @brief   TIMER Peripheral State functions
 *
@verbatim
  ==============================================================================
            ##### Peripheral Control and State functions #####
  ==============================================================================
    [..]
    This subsection provides functions allowing to :
      (+) Return the TIMER handle state.
      (+) Configure the TIMER.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the TIMER handle state.
 * @param[in]  p_timer: Pointer to a TIMER handle which contains the configuration
 *                 information for the specified TIMER module.
 * @retval ::HAL_TIMER_STATE_RESET: Peripheral not yet initialized or disabled.
 * @retval ::HAL_TIMER_STATE_READY: Peripheral Initialized and ready for use.
 * @retval ::HAL_TIMER_STATE_BUSY: An internal process is ongoing.
 * @retval ::HAL_TIMER_STATE_ERROR: Reception process is ongoing.
 ****************************************************************************************
 */
hal_timer_state_t hal_timer_get_state(timer_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  TIMER configuration
 * @param[in]  p_timer: Pointer to a TIMER handle which contains the configuration
 *                      information for the specified TIMER module.
 * @param[in]  p_structure: The TIMER configuration structure
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_timer_set_config(timer_handle_t *p_timer, timer_init_t *p_structure);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_TIMER_H__ */

/** @} */

/** @} */

/** @} */

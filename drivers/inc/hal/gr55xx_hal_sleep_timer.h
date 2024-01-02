/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_sleep_timer.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of sleep timer LL library.
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

/** @defgroup HAL_SLP_TIM SLP_TIM
  * @brief SLEEP TIMER HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_SLEEP_TIMER_H__
#define __GR55xx_HAL_SLEEP_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_sleep_timer.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/

/** @addtogroup HAL_SLP_TIM_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup SLP_TIM_Callback SLP_TIM Callback
  * @{
  */

/**
  * @brief HAL_SLP_TIM Elapsed Callback function definition
  */
typedef void (*pwr_slp_elapsed_handler_t)(void);

/**
  * @brief HAL_SLP_TIM Callback function definition
  */
typedef struct _pwr_handler
{
    pwr_slp_elapsed_handler_t       pwr_slp_elapsed_hander;     /**< PWR sleep timer elapsed callback */
} pwr_handler_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_SLP_TIM_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup HAL_SLP_TIM_Exported_Constants SLP TIM Exported Constants
  * @{
  */

/** @defgroup HAL_SLP_TIM_Timer_Mode PWR Sleep Timer Mode
 * @{
 */
#define PWR_SLP_TIMER_MODE_NORMAL       LL_SLEEP_TIMER_SINGLE_MODE_0    /**< Start counting after sleeping and disabled when waked up */
#define PWR_SLP_TIMER_MODE_SINGLE       LL_SLEEP_TIMER_SINGLE_MODE_1    /**< Single mode(keep counting until finished) */
#define PWR_SLP_TIMER_MODE_RELOAD       LL_SLEEP_TIMER_AUTO_MODE        /**< Auto reload  */
/** @} */

/** @defgroup HAL_SLP_TIM_TIMER_CLK_SEL Sleep Timer Clock Source
 * @{
 */
#define SLEEP_TIMER_CLK_SEL_RNG_OSC      LL_SLEEP_TIMER_CLK_SEL_RNG_OSC   /**< Set Sleep Timer Clock Source as RNG. */
#define SLEEP_TIMER_CLK_SEL_XO           LL_SLEEP_TIMER_CLK_SEL_XO        /**< Set Sleep Timer Clock Source as XO. */
#define SLEEP_TIMER_CLK_SEL_RNG2_OSC     LL_SLEEP_TIMER_CLK_SEL_RNG2_OSC  /**< Set Sleep Timer Clock Source as RNG2. */
#define SLEEP_TIMER_CLK_SEL_RTC_OSC      LL_SLEEP_TIMER_CLK_SEL_RTC_OSC   /**< Set Sleep Timer Clock Source as RTC. */
/** @} */

/** @} */
/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_SLP_TIM_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup HAL_SLP_TIM_Exported_Functions_Group1 Low Power mode configuration functions
  * @{
  */

/**
****************************************************************************************
* @brief  Configure the AON Sleep Timer mode, count and start used to wakeup MCU.
* @param[in]  mode: Specifies the sleep timer mode.
*             This parameter can be a combination of the following values:
*             @arg @ref PWR_SLP_TIMER_MODE_NORMAL
*             @arg @ref PWR_SLP_TIMER_MODE_SINGLE
*             @arg @ref PWR_SLP_TIMER_MODE_RELOAD
* @param[in]  value: Count value of the AON Sleep Timer.
* @retval ::HAL_OK: Operation is OK.
* @retval ::HAL_BUSY: Driver is busy.
* @note   The sleep clock of AON Timer is 32 KHz.
****************************************************************************************
*/
hal_status_t hal_sleep_timer_config_and_start(uint8_t mode, uint32_t value);

/**
****************************************************************************************
* @brief  stop Sleep Timer
****************************************************************************************
*/
void hal_sleep_timer_stop(void);

/**
****************************************************************************************
* @brief  Get the reload value of sleep timer
* @retval the reload value of sleep timer
****************************************************************************************
*/
uint32_t hal_sleep_timer_get_reload_value(void);

/**
****************************************************************************************
* @brief  Get the current value of sleep timer
* @retval the current value of sleep timer
****************************************************************************************
*/
uint32_t hal_sleep_timer_get_current_value(void);

/**
****************************************************************************************
* @brief  Get clock frequency of sleep timer
* @retval clock frequency of sleep timer
****************************************************************************************
*/
uint32_t hal_sleep_timer_get_clock_freq(void);


/**
****************************************************************************************
* @brief  Set the clock source of sleep timer
* @param[in]  clock_src: value This parameter can be a one of the following values:
*         @arg @ref SLEEP_TIMER_CLK_SEL_RNG_OSC
*         @arg @ref SLEEP_TIMER_CLK_SEL_XO
*         @arg @ref SLEEP_TIMER_CLK_SEL_RNG2_OSC
*         @arg @ref SLEEP_TIMER_CLK_SEL_RTC_OSC
****************************************************************************************
*/
void hal_sleep_timer_clock_set(uint32_t clock_src);

/**
****************************************************************************************
* @brief  get sleep timer is running or not
* @retval runing state of sleep timer (1 or 0).
****************************************************************************************
*/
uint8_t hal_sleep_timer_status_get(void);
/** @} */

/** @addtogroup PWR_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */
/**
 ****************************************************************************************
 * @brief  Handle PWR Sleep Timer interrupt request.
 * @note   Only available on GR551xx_B2 and later versions.
 ****************************************************************************************
 */
void hal_pwr_sleep_timer_irq_handler(void);

/**
 ****************************************************************************************
 * @brief  PWR Sleep Timer Elapsed callback.
 * @note   Only available on GR551xx_B2 and later versions.
 *         This function should not be modified. When the callback is needed,
 *         the hal_pwr_sleep_timer_elapsed_callback can be implemented in the user file.
 ****************************************************************************************
 */
void hal_pwr_sleep_timer_elapsed_callback(void);

/** @} */
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_SLEEP_TIMER_H__ */

/** @} */

/** @} */

/** @} */

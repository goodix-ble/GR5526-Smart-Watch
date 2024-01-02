/**
 ****************************************************************************************
 *
 * @file gr55xx_hal_pwr.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PWR HAL library.
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

/** @defgroup HAL_PWR PWR
  * @brief PWR HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_PWR_H__
#define __GR55xx_HAL_PWR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_pwr.h"
#include "gr55xx_hal_def.h"


/**
  * @defgroup  HAL_PWR_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PWR_Exported_Constants PWR Exported Constants
  * @{
  */

/** @defgroup PWR_WakeUp_Conditions  PWR Wakeup Condition
  * @{
  */
#define PWR_WKUP_COND_EXT               LL_PWR_WKUP_COND_EXT            /**< External wakeup: AON_GPIO      */
#define PWR_WKUP_COND_TIMER             LL_PWR_WKUP_COND_TIMER          /**< AON Timer wakeup               */
#define PWR_WKUP_COND_BLE               LL_PWR_WKUP_COND_BLE            /**< BLE wakeup                     */
#define PWR_WKUP_COND_CLDR              LL_PWR_WKUP_COND_CLDR           /**< Calendar wakeup                */

#define PWR_WKUP_COND_BOD_FEDGE         LL_PWR_WKUP_COND_BOD_FEDGE      /**< PMU Bod falling edge wakeup    */
#define PWR_WKUP_COND_MSIO_COMP         LL_PWR_WKUP_COND_COMP           /**< Msio comparator wakeup         */
#define PWR_WKUP_COND_AUSB              LL_PWR_WKUP_COND_AUSB           /**< USB attach wakeup event        */
#define PWR_WKUP_COND_DUSB              LL_PWR_WKUP_COND_DUSB           /**< USB detach wakeup event        */
#define PWR_WKUP_COND_BLE_IRQ           LL_PWR_WKUP_COND_BLE_IRQ        /**< BLE IRQ wakeup event           */
#define PWR_WKUP_COND_CLDR_TICK         LL_PWR_WKUP_COND_CLDR_TICK      /**< Calendar Tick wakeup event     */
#define PWR_WKUP_COND_AON_WDT           LL_PWR_WKUP_COND_AON_WDT        /**< AON WDT Alarm wakeup event     */
#define PWR_WKUP_COND_ALL               LL_PWR_WKUP_COND_ALL            /**< All wakeup sources mask        */

/** @} */

/** @defgroup PWR_Timer_Type  PWR Timer Type
 *  @note     Only available on GR551xx_B2 and later versions.
 *  @{
 */

#define PWR_TIMER_TYPE_CAL_TIMER        0                               /**< Calendar timer                 */
#define PWR_TIMER_TYPE_AON_WDT          1                               /**< AON watchdog alarm timer       */
#define PWR_TIMER_TYPE_SLP_TIMER        2                               /**< Sleep timer                    */
#define PWR_TIMER_TYPE_CAL_ALARM        3                               /**< Calendar alarm timer           */
#define PWR_TIMER_TYPE_AON_WDT_TIMER    4                               /**< AON watchdog timer timer(B0)   */

/** @} */


/** @defgroup PWR_Memory_Power_State  Memory Power State
 * @{
 */
#define PWR_MEM_POWER_OFF               LL_PWR_MEM_POWER_OFF            /**< Power off */
#define PWR_MEM_POWER_FULL              LL_PWR_MEM_POWER_FULL           /**< Full power */
#define PWR_MEM_POWER_RETENTION         LL_PWR_MEM_POWER_RETENTION      /**< Power retention, low valtage mode */
/** @} */

/** @defgroup PWR_Timeout_definition PWR Timeout definition
 * @{
  */
#define HAL_PWR_TIMEOUT_DEFAULT_VALUE ((uint32_t)0x000FFFFF)         /**< 0xFFFFF counts */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup  PWR_Private_Macros   PWR Private Macros
  * @{
  */

/**
  * @brief Check if PWR wakeup condition is valid.
  * @param __COND__ PWR wakeup condition.
  * @retval SET (__COND__ is valid) or RESET (__COND__ is invalid)
  */
#define IS_PWR_WAKEUP_CONDITION(__COND__)       ((((__COND__) & PWR_WKUP_COND_ALL) != 0x00U) &&\
                                                 (((__COND__) & ~PWR_WKUP_COND_ALL) == 0x00U))

/**
  * @brief Check if PWR memory block is valid.
  * @param __BLOCK__ PWR memory block.
  * @retval SET (__BLOCK__ is valid) or RESET (__BLOCK__ is invalid)
  */
#define IS_PWR_MEM_BLOCK(__BLOCK__)             ((((__BLOCK__) & PWR_MEM_ALL) != 0x00U) &&\
                                                 (((__BLOCK__) & ~PWR_MEM_ALL) == 0x00U))

/**
  * @brief Check if PWR memory power state is valid.
  * @param __STATE__ PWR memory power state.
  * @retval SET (__STATE__ is valid) or RESET (__STATE__ is invalid)
  */
#define IS_PWR_MEM_POWER_STAT(__STATE__)        (((__STATE__) == PWR_MEM_POWER_OFF)  || \
                                                 ((__STATE__) == PWR_MEM_POWER_FULL) || \
                                                 ((__STATE__) == PWR_MEM_POWER_RETENTION))

/**
  * @brief Check if PWR sleep timer type is valid.
  * @param __TYPE__ PWR sleep timer type.
  * @retval SET (__TYPE__ is valid) or RESET (__TYPE__ is invalid)
  */
#define IS_PWR_PWR_TIMER_TYPE(__TYPE__)         (((__TYPE__) == PWR_TIMER_TYPE_CAL_TIMER) || \
                                                 ((__TYPE__) == PWR_TIMER_TYPE_AON_WDT)   || \
                                                 ((__TYPE__) == PWR_TIMER_TYPE_SLP_TIMER) || \
                                                 ((__TYPE__) == PWR_TIMER_TYPE_CAL_ALARM))

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_PWR_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup PWR_Exported_Functions_Group1 Low Power mode configuration functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief Enters DeepSleep mode.
 * @note  In DeepSleep mode, all I/O pins keep the same state as in Run mode.
 ****************************************************************************************
*/
void hal_pwr_enter_chip_deepsleep(void);

/** @} */

/** @addtogroup PWR_Exported_Functions_Group2 BLE Communication timer and core configuration function
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Get the current value of specified timer.
 * @note   Only available on GR551xx_B2 and later versions.
 * @param[in]  timer_type: This parameter can be one of the following values:
 *         @arg @ref PWR_TIMER_TYPE_CAL_TIMER
 *         @arg @ref PWR_TIMER_TYPE_AON_WDT
 *         @arg @ref PWR_TIMER_TYPE_SLP_TIMER
 *         @arg @ref PWR_TIMER_TYPE_CAL_ALARM
 * @param[out] p_value: Pointer to an integer storing current value
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwr_get_timer_current_value(uint32_t timer_type, uint32_t *p_value);

/** @} */
/** @} */

#ifdef __cplusplus
}
#endif


#endif /* __GR55xx_HAL_PWR_H__ */

/** @} */

/** @} */

/** @} */

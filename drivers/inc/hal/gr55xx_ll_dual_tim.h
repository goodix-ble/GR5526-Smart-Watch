/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_dual_tim.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of DUAL TIMER LL library.
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

/** @addtogroup LL_DRIVER LL Driver
  * @{
  */

/** @defgroup LL_DUAL_TIMER DUAL_TIMER
  * @brief DUAL TIM LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_DUAL_TIMER_H__
#define __GR55XX_LL_DUAL_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (DUAL_TIMER0) || defined (DUAL_TIMER1)

/** @defgroup DUAL_TIMER_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup DUAL_TIMER_LL_ES_INIT DUAL_TIM Exported init structures
  * @{
  */

/**
  * @brief LL DUAL TIMER init Structure definition
  */
typedef struct _ll_dual_timer_init
{
    uint32_t prescaler;         /**< Specifies the prescaler value used to divide the TIMER clock.
                                   This parameter can be a value of @ref DUAL_TIMER_EC_LL_PRESCALER.

                                   This feature can be modified afterwards using unitary function @ref ll_dual_timer_set_prescaler().*/

    uint32_t counter_size;       /**< Specifies the prescaler value used to divide the DUAL_TIMER clock.
                                   This parameter can be a value of @ref DUAL_TIMER_EC_LL_COUNTERSIZE.

                                   This feature can be modified afterwards using unitary function @ref ll_dual_timer_set_counter_size().*/

    uint32_t counter_mode;       /**< Specifies the counter mode.
                                   This parameter can be a value of @ref DUAL_TIMER_LL_EC_COUNTERMODE.

                                   This feature can be modified afterwards using unitary function @ref ll_dual_timer_set_counter_mode().*/

    uint32_t auto_reload;        /**< Specifies the auto reload value to be loaded into the active
                                   Auto-Reload Register at the next update event.
                                   This parameter must be a number between Min_Data=0x00000000 and Max_Data=0xFFFFFFFF.
                                   Some timer instances may support 16 bits counters. In that case this parameter must be a number between 0x0000 and 0xFFFF.

                                   This feature can be modified afterwards using unitary function @ref ll_dual_timer_set_auto_reload().*/
} ll_dual_timer_init_t;

/** @} */

/** @} */

/**
  * @defgroup  DUAL_TIMER_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DUAL_TIMER_LL_Exported_Constants DUAL_TIM Exported Constants
  * @{
  */

/** @defgroup DUAL_TIMER_LL_EC_COUNTERMODE DUAL_TIM counter mode
  * @{
  */
#define LL_DUAL_TIMER_FREERUNNING_MODE        0x00000000U           /**< Free running mode */
#define LL_DUAL_TIMER_PERIODIC_MODE           DUAL_TIMER_CTRL_MODE    /**< Periodic mode */
/** @} */

/** @defgroup DUAL_TIMER_EC_LL_PRESCALER DUAL_TIM prescaler
  * @{
  */
#define LL_DUAL_TIMER_PRESCALER_DIV0          0x00000000U                     /**< 0 stage  of prescale, clock is divided by 1.   */
#define LL_DUAL_TIMER_PRESCALER_DIV16         (1UL << DUAL_TIMER_CTRL_PRE_Pos)  /**< 4 stages of prescale, clock is divided by 16.  */
#define LL_DUAL_TIMER_PRESCALER_DIV256        (2UL << DUAL_TIMER_CTRL_PRE_Pos)  /**< 8 stages of prescale, clock is divided by 256. */
/** @} */

/** @defgroup DUAL_TIMER_EC_LL_COUNTERSIZE DUAL_TIM counter size
  * @{
  */
#define LL_DUAL_TIMER_COUNTERSIZE_16          0x00000000U         /**< Counter size 16 bits */
#define LL_DUAL_TIMER_COUNTERSIZE_32          DUAL_TIMER_CTRL_SIZE  /**< Counter size 32 bits */
/** @} */

/** @defgroup DUAL_TIMER_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */

/**
  * @brief LL DUAL_TIMER InitStrcut default configuartion
  */
#define DUAL_TIMER_DEFAULT_CONFIG                     \
{                                                   \
    .prescaler   = LL_DUAL_TIMER_PRESCALER_DIV0,      \
    .counter_size = LL_DUAL_TIMER_COUNTERSIZE_32,      \
    .counter_mode = LL_DUAL_TIMER_PERIODIC_MODE,       \
    .auto_reload  = SystemCoreClock - 1,             \
}
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DUAL_TIMER_LL_Exported_Macros DUAL_TIM Exported Macros
  * @{
  */

/** @defgroup DUAL_TIMER_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in DUAL_TIMER register
  * @param  __instance__ DUAL_TIMER instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_DUAL_TIMER_WriteReg(__instance__, __REG__, __VALUE__)  WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in DUAL_TIMER register
  * @param  __instance__ DUAL_TIMER instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_DUAL_TIMER_ReadReg(__instance__, __REG__)              READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup DUAL_TIMER_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup DUAL_TIMER_LL_EF_Configuration Configuration functions
  * @{
  */


/**
  * @brief  Enable dual_timer counter.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | EN
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_dual_timer_enable_counter(dual_timer_regs_t *DUAL_TIMERx)
{
    SET_BITS(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_EN);
}

/**
  * @brief  Disable dual_timer counter.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | EN
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_dual_timer_disable_counter(dual_timer_regs_t *DUAL_TIMERx)
{
    CLEAR_BITS(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_EN);
}

/**
  * @brief  Indicate whether the dual_timer counter is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | EN
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dual_timer_is_enabled_counter(dual_timer_regs_t *DUAL_TIMERx)
{
    return (READ_BITS(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_EN) == (DUAL_TIMER_CTRL_EN));
}

/**
  * @brief  Set the counter mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | MODE
  *
  * @param  DUAL_TIMERx Timer instance
  * @param  counter_mode This parameter can be one of the following values:
  *         @arg @ref LL_DUAL_TIMER_FREERUNNING_MODE
  *         @arg @ref LL_DUAL_TIMER_PERIODIC_MODE
  * @retval None
  */
__STATIC_INLINE void ll_dual_timer_set_counter_mode(dual_timer_regs_t *DUAL_TIMERx, uint32_t counter_mode)
{
    MODIFY_REG(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_MODE, counter_mode);
}

/**
  * @brief  Get the counter mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | MODE
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval Return value can be one of the following values:
  *         @arg @ref LL_DUAL_TIMER_FREERUNNING_MODE
  *         @arg @ref LL_DUAL_TIMER_PERIODIC_MODE
  */
__STATIC_INLINE uint32_t ll_dual_timer_get_counter_mode(dual_timer_regs_t *DUAL_TIMERx)
{
    return (READ_BITS(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_MODE));
}

/**
  * @brief  Set the prescaler.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | PRE
  *
  * @param  DUAL_TIMERx Timer instance
  * @param  prescaler This parameter can be one of the following values:
  *         @arg @ref LL_DUAL_TIMER_PRESCALER_DIV0
  *         @arg @ref LL_DUAL_TIMER_PRESCALER_DIV16
  *         @arg @ref LL_DUAL_TIMER_PRESCALER_DIV256
  * @retval None
  */
__STATIC_INLINE void ll_dual_timer_set_prescaler(dual_timer_regs_t *DUAL_TIMERx, uint32_t prescaler)
{
    MODIFY_REG(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_PRE, prescaler);
}

/**
  * @brief  Get the prescaler.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | PRE
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval Return value can be one of the following values:
  *         @arg @ref LL_DUAL_TIMER_PRESCALER_DIV0
  *         @arg @ref LL_DUAL_TIMER_PRESCALER_DIV16
  *         @arg @ref LL_DUAL_TIMER_PRESCALER_DIV256
  */
__STATIC_INLINE uint32_t ll_dual_timer_get_prescaler(dual_timer_regs_t *DUAL_TIMERx)
{
    return (READ_BITS(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_PRE));
}

/**
  * @brief  Set the counter size.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | SIZE
  *
  * @param  DUAL_TIMERx Timer instance
  * @param  counter_size This parameter can be one of the following values:
  *         @arg @ref LL_DUAL_TIMER_COUNTERSIZE_16
  *         @arg @ref LL_DUAL_TIMER_COUNTERSIZE_32
  * @retval None
  */
__STATIC_INLINE void ll_dual_timer_set_counter_size(dual_timer_regs_t *DUAL_TIMERx, uint32_t counter_size)
{
    MODIFY_REG(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_SIZE, counter_size);
}

/**
  * @brief  Get the counter size.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | SIZE
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval Return value can be one of the following values:
  *         @arg @ref LL_DUAL_TIMER_COUNTERSIZE_16
  *         @arg @ref LL_DUAL_TIMER_COUNTERSIZE_32
  */
__STATIC_INLINE uint32_t ll_dual_timer_get_counter_size(dual_timer_regs_t *DUAL_TIMERx)
{
    return (READ_BITS(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_SIZE));
}

/**
  * @brief  Enable one-shot mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | ONESHOT
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_dual_timer_enable_oneshot(dual_timer_regs_t *DUAL_TIMERx)
{
    SET_BITS(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_ONESHOT);
}

/**
  * @brief  Disable one-shot mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | ONESHOT
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_dual_timer_disable_oneshot(dual_timer_regs_t *DUAL_TIMERx)
{
    CLEAR_BITS(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_ONESHOT);
}

/**
  * @brief  Indicate whether the one-shot mode is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | ONESHOT
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dual_timer_is_enabled_oneshot(dual_timer_regs_t *DUAL_TIMERx)
{
    return (READ_BITS(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_ONESHOT) == (DUAL_TIMER_CTRL_ONESHOT));
}

/**
  * @brief  Get the counter value.
  *
  *  Register|BitsName
  *  --------|--------
  *  VALUE | VALUE
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval Counter value (between Min_Data=0 and Max_Data=0xFFFFFFFF)
  */
__STATIC_INLINE uint32_t ll_dual_timer_get_counter(dual_timer_regs_t *DUAL_TIMERx)
{
    return (uint32_t)(READ_REG(DUAL_TIMERx->VALUE));
}

/**
  * @brief  Set the auto-reload value.
  * @note   The counter is blocked while the auto-reload value is null.
  *
  *  Register|BitsName
  *  --------|--------
  *  RELOAD | RELOAD
  *
  * @param  DUAL_TIMERx Timer instance
  * @param  auto_reload between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_dual_timer_set_auto_reload(dual_timer_regs_t *DUAL_TIMERx, uint32_t auto_reload)
{
    WRITE_REG(DUAL_TIMERx->RELOAD, auto_reload);
}

/**
  * @brief  Get the auto-reload value.
  *
  *  Register|BitsName
  *  --------|--------
  *  RELOAD | RELOAD
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval Auto-reload value
  */
__STATIC_INLINE uint32_t ll_dual_timer_get_auto_reload(dual_timer_regs_t *DUAL_TIMERx)
{
    return (uint32_t)(READ_REG(DUAL_TIMERx->RELOAD));
}

/**
  * @brief  Set the backgroud-reload value.
  *
  *  Register|BitsName
  *  --------|--------
  *  BG_LOAD | BG_LOAD
  *
  * @param  DUAL_TIMERx Timer instance
  * @param  background_reload between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_dual_timer_set_background_reload(dual_timer_regs_t *DUAL_TIMERx, uint32_t background_reload)
{
    WRITE_REG(DUAL_TIMERx->BG_LOAD, background_reload);
}

/**
  * @brief  Get the backgroud-reload value.
  *
  *  Register|BitsName
  *  --------|--------
  *  BG_LOAD | BG_LOAD
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval Return value between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_dual_timer_get_background_reload(dual_timer_regs_t *DUAL_TIMERx)
{
    return (uint32_t)(READ_REG(DUAL_TIMERx->BG_LOAD));
}

/** @} */

/** @defgroup DUAL_TIM_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable dual_timer interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | INTEN
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_dual_timer_enable_it(dual_timer_regs_t *DUAL_TIMERx)
{
    SET_BITS(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_INTEN);
}

/**
  * @brief  Disable dual_timer interrput.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | INTEN
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_dual_timer_disable_it(dual_timer_regs_t *DUAL_TIMERx)
{
    CLEAR_BITS(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_INTEN);
}

/**
  * @brief  Indicate whether the dual_timer interrput is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | INTEN
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dual_timer_is_enabled_it(dual_timer_regs_t *DUAL_TIMERx)
{
    return (READ_BITS(DUAL_TIMERx->CTRL, DUAL_TIMER_CTRL_INTEN) == (DUAL_TIMER_CTRL_INTEN));
}

/** @} */

/** @defgroup DUAL_TIMER_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Clear the interrupt flag (INTSTAT).
  *

  *  Register|BitsName
  *  --------|--------
  *  INTCLR | INTCLR
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_dual_timer_clear_flag_it(dual_timer_regs_t *DUAL_TIMERx)
{
    WRITE_REG(DUAL_TIMERx->INTCLR, DUAL_TIMER_INT_CLR);
}

/**
  * @brief  Indicate whether interrupt flag (INTSTAT) is set (interrupt is pending).
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | INTSTAT
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dual_timer_is_active_flag_it(dual_timer_regs_t *DUAL_TIMERx)
{
    return (READ_BITS(DUAL_TIMERx->INTSTAT, DUAL_TIMER_ISR_TI) == (DUAL_TIMER_ISR_TI));
}

/**
  * @brief  Get Dual_timer raw interrupt flags
  *
  *  Register|BitsName
  *  --------|--------
  *  RAW_INTSTAT | RAW_INTSTAT
  *
  * @param  DUAL_TIMERx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dual_timer_get_raw_it_flag(dual_timer_regs_t *DUAL_TIMERx)
{
    return (READ_REG(DUAL_TIMERx->RAW_INTSTAT));
}

/** @} */

/** @defgroup DUAL_TIMER_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize DUAL_TIMER registers (Registers restored to their default values).
  * @param  DUAL_TIMERx DUAL_TIM instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: DUAL_TIMER registers are de-initialized
  *          - ERROR: DUAL_TIMER registers are not de-initialized
  */
error_status_t ll_dual_timer_deinit(dual_timer_regs_t *DUAL_TIMERx);

/**
  * @brief  Initialize DUAL_TIMER registers according to the specified
  *         parameters in p_dual_timer_init.
  * @param  DUAL_TIMERx DUAL_TIMER instance
  * @param  p_dual_timer_init Pointer to a ll_dual_timer_init_t structure that contains the configuration
  *                         information for the specified DUAL_TIMER peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: DUAL_TIMER registers are initialized according to p_dual_timer_init content
  *          - ERROR: Problem occurred during DUAL_TIM Registers initialization
  */
error_status_t ll_dual_timer_init(dual_timer_regs_t *DUAL_TIMERx, ll_dual_timer_init_t *p_dual_timer_init);

/** @} */

/** @} */


#endif /* DUAL_TIMER0 || DUAL_TIMER1 */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_DUAL_TIMER_H__ */

/** @} */

/** @} */

/** @} */

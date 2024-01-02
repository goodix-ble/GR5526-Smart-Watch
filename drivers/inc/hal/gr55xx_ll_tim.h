/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_tim.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of TIMER LL library.
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

/** @defgroup LL_TIMER TIMER
  * @brief TIMER LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_TIMER_H__
#define __GR55XX_LL_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (TIMER0) || defined (TIMER1)

/** @defgroup TIMER_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup TIMER_LL_ES_INIT TIMER Exported init structures
  * @{
  */

/**
  * @brief LL TIMER init Structure definition
  */
typedef struct _ll_timer_init_t
{
    uint32_t auto_reload;        /**< Specifies the auto reload value to be loaded into the active
                                     Auto-Reload Register at the next update event.
                                     This parameter must be a number between Min_Data=0x00000000 and Max_Data=0xFFFFFFFF.
                                     Some timer instances may support 32 bits counters. In that case this parameter must be a number between 0x0000 and 0xFFFFFFFF.

                                     This feature can be modified afterwards using unitary function @ref ll_timer_set_auto_reload().*/
} ll_timer_init_t;
/** @} */

/** @} */

/**
  * @defgroup  TIMER_LL_TIMER_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup TIMER_LL_Exported_Constants TIMER Exported Constants
  * @{
  */

/** @defgroup TIMER_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */
/**
  * @brief LL TIMER InitStrcut default configuartion
  */
#define TIMER_DEFAULT_CONFIG                \
{                                         \
    .auto_reload = SystemCoreClock - 1,    \
}
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup TIMER_LL_Exported_Macros TIMER Exported Macros
  * @{
  */

/** @defgroup TIMER_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in TIMER register
  * @param  __instance__ TIMER instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_TIMER_WriteReg(__instance__, __REG__, __VALUE__)   WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in TIMER register
  * @param  __instance__ TIMER instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_TIMER_ReadReg(__instance__, __REG__)               READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup TIMER_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup TIMER_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable timer counter.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | EN
  *
  * @param  TIMERx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_timer_enable_counter(timer_regs_t *TIMERx)
{
    SET_BITS(TIMERx->CTRL, TIMER_CTRL_EN);
}

/**
  * @brief  Disable timer counter.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | EN
  *
  * @param  TIMERx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_timer_disable_counter(timer_regs_t *TIMERx)
{
    CLEAR_BITS(TIMERx->CTRL, TIMER_CTRL_EN);
}

/**
  * @brief  Indicate whether the timer counter is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | EN
  *
  * @param  TIMERx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_timer_is_enabled_counter(timer_regs_t *TIMERx)
{
    return (READ_BITS(TIMERx->CTRL, TIMER_CTRL_EN) == (TIMER_CTRL_EN));
}

/**
  * @brief  Set the counter value.
  *
  *  Register|BitsName
  *  --------|--------
  *  VALUE | VALUE
  *
  * @param  TIMERx Timer instance
  * @param  counter Counter value (between Min_Data=0 and Max_Data=0xFFFFFFFF)
  * @retval None
  */
__STATIC_INLINE void ll_timer_set_counter(timer_regs_t *TIMERx, uint32_t counter)
{
    WRITE_REG(TIMERx->VALUE, counter);
}

/**
  * @brief  Get the counter value.
  *
  *  Register|BitsName
  *  --------|--------
  *  VALUE | VALUE
  *
  * @param  TIMERx Timer instance
  * @retval Counter value (between Min_Data=0 and Max_Data=0xFFFFFFFF)
  */
__STATIC_INLINE uint32_t ll_timer_get_counter(timer_regs_t *TIMERx)
{
    return (uint32_t)(READ_REG(TIMERx->VALUE));
}

/**
  * @brief  Set the auto-reload value.
  * @note   The counter is blocked while the auto-reload value is null.
  *
  *  Register|BitsName
  *  --------|--------
  *  RELOAD | RELOAD
  *
  * @param  TIMERx Timer instance
  * @param  auto_reload between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_timer_set_auto_reload(timer_regs_t *TIMERx, uint32_t auto_reload)
{
    WRITE_REG(TIMERx->RELOAD, auto_reload);
}

/**
  * @brief  Get the auto-reload value.
  *
  *  Register|BitsName
  *  --------|--------
  *  RELOAD | RELOAD
  *
  * @param  TIMERx Timer instance
  * @retval Auto-reload value
  */
__STATIC_INLINE uint32_t ll_timer_get_auto_reload(timer_regs_t *TIMERx)
{
    return (uint32_t)(READ_REG(TIMERx->RELOAD));
}

/** @} */

/** @defgroup TIM_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable timer interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | INTEN
  *
  * @param  TIMERx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_timer_enable_it(timer_regs_t *TIMERx)
{
    SET_BITS(TIMERx->CTRL, TIMER_CTRL_INTEN);
}

/**
  * @brief  Disable timer interrput.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | INTEN
  *
  * @param  TIMERx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_timer_disable_it(timer_regs_t *TIMERx)
{
    CLEAR_BITS(TIMERx->CTRL, TIMER_CTRL_INTEN);
}

/**
  * @brief  Indicate whether the timer interrput is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | INTEN
  *
  * @param  TIMERx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_timer_is_enabled_it(timer_regs_t *TIMERx)
{
    return (READ_BITS(TIMERx->CTRL, TIMER_CTRL_INTEN) == (TIMER_CTRL_INTEN));
}

/** @} */

/** @defgroup TIM_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Clear the interrupt flag (INTSTAT).
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | INTSTAT
  *
  * @param  TIMERx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_timer_clear_flag_it(timer_regs_t *TIMERx)
{
    WRITE_REG(TIMERx->INTSTAT, TIMER_INT_STAT);
}

/**
  * @brief  Indicate whether interrupt flag (INTSTAT) is set (interrupt is pending).
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | INTSTAT
  *
  * @param  TIMERx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_timer_is_active_flag_it(timer_regs_t *TIMERx)
{
    return (READ_BITS(TIMERx->INTSTAT, TIMER_INT_STAT) == (TIMER_INT_STAT));
}

/** @} */

/** @defgroup TIM_LL_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize TIMER registers (Registers restored to their default values).
  * @param  TIMERx TIMER instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: TIMER registers are de-initialized
  *          - ERROR: TIMER registers are not de-initialized
  */
error_status_t ll_timer_deinit(timer_regs_t *TIMERx);

/**
  * @brief  Initialize TIMER registers according to the specified
  *         parameters in TIMER_InitStruct.
  * @param  TIMERx TIMER instance
  * @param  p_timer_init Pointer to a ll_timer_init_t structure that contains the configuration
  *                        information for the specified TIM peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: TIMER registers are initialized according to p_timer_init content
  *          - ERROR: Problem occurred during TIMER Registers initialization
  */
error_status_t ll_timer_init(timer_regs_t *TIMERx, ll_timer_init_t *p_timer_init);

/**
  * @brief Set each field of a @ref ll_timer_init_t type structure to default value.
  * @param p_timer_init  Pointer to a @ref ll_timer_init_t structure
  *                        whose fields will be set to default values.
  * @retval None
  */
void ll_timer_struct_init(ll_timer_init_t *p_timer_init);

/** @} */

/** @} */

#endif /* TIMER0 || TIMER1 */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_TIMER_H__ */

/** @} */

/** @} */

/** @} */

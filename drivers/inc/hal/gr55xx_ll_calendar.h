/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_calendar.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of CALENDAR LL library.
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

/** @defgroup LL_CALENDAR CALENDAR
  * @brief CALENDAR LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_CALENDAR_H__
#define __GR55XX_LL_CALENDAR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

/**
  * @defgroup CALENDAR_LL_MACRO Defines
  * @{
  */
/* Exported constants --------------------------------------------------------*/
/** @defgroup CALENDAR_LL_Exported_Constants CALENDAR Exported Constants
  * @{
  */
/** @defgroup CALENDAR_LL_EC_CLOCK_DIV Clock divider
  * @{
  */
#define LL_CALENDAR_DIV_NONE                ((uint32_t)0x00U)                                   /**< Select SLP_CLK       */
#define LL_CALENDAR_DIV_2                   ((uint32_t)0x01U << RTC_CFG1_DIV_Pos)               /**< Select 1/32 divider  */
#define LL_CALENDAR_DIV_4                   ((uint32_t)0x02U << RTC_CFG1_DIV_Pos)               /**< Select 1/32 divider  */
#define LL_CALENDAR_DIV_8                   ((uint32_t)0x03U << RTC_CFG1_DIV_Pos)               /**< Select 1/32 divider  */
#define LL_CALENDAR_DIV_16                  ((uint32_t)0x04U << RTC_CFG1_DIV_Pos)               /**< Select 1/32 divider  */
#define LL_CALENDAR_DIV_32                  ((uint32_t)0x05U << RTC_CFG1_DIV_Pos)               /**< Select 1/64 divider  */
#define LL_CALENDAR_DIV_64                  ((uint32_t)0x06U << RTC_CFG1_DIV_Pos)               /**< Select 1/128 divider */
#define LL_CALENDAR_DIV_128                 ((uint32_t)0x07U << RTC_CFG1_DIV_Pos)               /**< Select 1/256 divider */
/** @} */

/** @defgroup CALENDAR_LL_TIMER_CLK_SEL  Calendar Timer clock select defines
 * @{
 */
#define LL_CLDR_TIMER_CLK_SEL_RNG           (0x0U << RTC_CLK_SEL_Pos)                           /**< Select RNG clcok source    */
#define LL_CLDR_TIMER_CLK_SEL_XO            (0x1U << RTC_CLK_SEL_Pos)                           /**< Select XO clcok source     */
#define LL_CLDR_TIMER_CLK_SEL_RNG2          (0x2U << RTC_CLK_SEL_Pos)                           /**< Select RNG2 clcok source   */
#define LL_CLDR_TIMER_CLK_SEL_RTC           (0x3U << RTC_CLK_SEL_Pos)                           /**< Select RTC clcok source    */
/** @} */

/** @defgroup CALENDAR_LL_TIMER_PERIODIC_SEL  Calendar Periodic timer select defines
 * @{
 */
#define LL_CLDR_TIMER_TICK                  (0x0U)                                              /**< Select periodic alarm       */
#define LL_CLDR_TIMER_TICK_TYPE_SINGLE      (0x0U)                                              /**< Select periodic alarm one-time     */
#define LL_CLDR_TIMER_TICK_TYPE_AUTO        (0x1U)                                              /**< Select periodic alarm auto-reload  */
/** @} */

/** @} */

/** @defgroup CALENDAR_LL_Exported_Macros  CALENDAR Exported Macros
 * @{
 */

/**
  * @brief  Calendar Register Read.
  */
#define   CLDR_REG_READ                     (READ_BITS(CALENDAR->CFG0, RTC_CFG0_EN | \
                                                        RTC_CFG0_ALARM_EN | \
                                                        RTC_CFG0_TICK_EN | \
                                                        RTC_CFG0_TICK_MDOE))
/** @} */

/** @} */


/* Exported functions --------------------------------------------------------*/
/** @defgroup CALENDAR_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup CALENDAR_LL_EF_Configuration Configuration functions
  * @{
  */
/**
  * @brief  Set the Calendar Timer clock
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_CLK | cldr_timer_clk_sel

  * @param  value: This parameter can be a one of the following values:
  *         @arg @ref LL_CLDR_TIMER_CLK_SEL_RNG
  *         @arg @ref LL_CLDR_TIMER_CLK_SEL_XO
  *         @arg @ref LL_CLDR_TIMER_CLK_SEL_RNG2
  *         @arg @ref LL_CLDR_TIMER_CLK_SEL_RTC
  * @retval None
  */
__STATIC_INLINE void ll_calendar_timer_set_clk(uint32_t value)
{
    MODIFY_REG(CALENDAR->CLK, RTC_CLK_SEL, value);
}

/**
  * @brief Get the Calendar Timer clock
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_CLK | cldr_timer_clk_sel
  *
  * @retval Calendar Timer clock source,the value can be the one of the following:
  *         @arg @ref LL_CLDR_TIMER_CLK_SEL_RNG
  *         @arg @ref LL_CLDR_TIMER_CLK_SEL_XO
  *         @arg @ref LL_CLDR_TIMER_CLK_SEL_RNG2
  *         @arg @ref LL_CLDR_TIMER_CLK_SEL_RTC
  */
__STATIC_INLINE uint32_t ll_calendar_timer_get_clk(void)
{
    return (READ_BITS(CALENDAR->CLK, RTC_CLK_SEL));
}

/**
  * @brief  Enable calendar counter.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_CFG0 | EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_enable(void)
{
    WRITE_REG(CALENDAR->CFG0, RTC_CFG0_CFG | RTC_CFG0_EN | CLDR_REG_READ);
}

/**
  * @brief  Disable calendar counter.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_CFG0 | EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_disable(void)
{
    MODIFY_REG(CALENDAR->CFG0, 0xFFFFFFFF, RTC_CFG0_CFG);
}

/**
  * @brief  Check if the CALENDAR peripheral is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_CFG0 | EN
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_calendar_is_enabled(void)
{
    return (READ_BITS(CALENDAR->CFG0, RTC_CFG0_EN) == RTC_CFG0_EN);
}

/**
  * @brief  Reloads CALENDAR counter.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_TIMER_W | TIMER_VALUE
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_reload_counter(uint32_t counter)
{
    WRITE_REG(CALENDAR->TIMER_W, counter);
}

/**
  * @brief  Reloads CALENDAR counter and request.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_TIMER_W | TIMER_VALUE
  *  CLDR_CFG0 | VAL_LOAD
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_reload_counter_and_request(uint32_t counter)
{
    WRITE_REG(CALENDAR->TIMER_W, counter);
    WRITE_REG(CALENDAR->CFG0, RTC_CFG0_CFG | RTC_CFG0_TIMER_SET | CLDR_REG_READ);
}

/**
  * @brief  Reloads CALENDAR alarm.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_TIMER_W | ALARM_VAL_LOAD
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_reload_alarm(uint32_t alarm)
{
    WRITE_REG(CALENDAR->ALARM_W, alarm);
}

/**
  * @brief  Reloads CALENDAR alarm and request.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_ALARM_W | ALARM_VAL_LOAD
  *  CLDR_CFG0 | ALARM_VALUE
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_reload_alarm_and_request(uint32_t alarm)
{
    WRITE_REG(CALENDAR->ALARM_W, alarm);
    WRITE_REG(CALENDAR->CFG0, RTC_CFG0_CFG | RTC_CFG0_ALARM_SET | CLDR_REG_READ);
}

/**
  * @brief  Read the CALENDAR counter config value.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_TIMER_W | TIMER_VAL_READ
  *
  * @retval Value for current counter which should ranging between 0 ~ 0xFFFF_FFFF
  */
__STATIC_INLINE uint32_t ll_calendar_get_counter(void)
{
    return (uint32_t)READ_REG(CALENDAR->TIMER_W);
}

/** 
  * @brief  Read the CALENDAR counter current value.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_TIMER_R | TIMER_VAL_READ
  *
  * @retval Value for current counter which should ranging between 0 ~ 0xFFFF_FFFF
  */
__STATIC_INLINE uint32_t ll_calendar_get_read_counter(void)
{
    return (uint32_t)READ_REG(CALENDAR->TIMER_R);
}

/**
  * @brief  Read the CALENDAR counter config alarm value.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_ALARM_W | CAL_ALARM
  *
  * @retval Value for current alarm which should ranging between 0 ~ 0xFFFF_FFFF
  */
__STATIC_INLINE uint32_t ll_calendar_get_alarm(void)
{
    return (uint32_t)READ_REG(CALENDAR->ALARM_W);
}

/**
  * @brief  Read the CALENDAR counter current alarm value.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_ALARM_R | CAL_ALARM
  *
  * @retval Value for current alarm which should ranging between 0 ~ 0xFFFF_FFFF
  */
__STATIC_INLINE uint32_t ll_calendar_get_read_alarm(void)
{
    return (uint32_t)READ_REG(CALENDAR->ALARM_R);
}

/**
  * @brief  Get the CALENDAR wrap-around value.
  * @note   The value should be read multiple times until get the same value in at least two reads.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_STAT | WRAP_CNT
  *
  * @retval Value between Min_Data=0 and Max_Data=0xF
  */
__STATIC_INLINE uint32_t ll_calendar_get_wrapcnt(void)
{
    return (uint32_t)(READ_BITS(CALENDAR->STAT, RTC_STAT_WRAP_CNT) >> RTC_STAT_WRAP_CNT_Pos);
}

/**
  * @brief  The CALENDAR is busy.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_STAT | BUSY
  *
  * @retval Value between Min_Data=0 and Max_Data=0xF
  */
__STATIC_INLINE uint32_t ll_calendar_is_busy(void)
{
    return (uint32_t)(READ_BITS(CALENDAR->STAT, RTC_STAT_BUSY) == RTC_STAT_BUSY);
}

/**
  * @brief  The CALENDAR is running.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_STAT | RUNNING
  *
  * @retval Value between Min_Data=0 and Max_Data=0xF
  */
__STATIC_INLINE uint32_t ll_calendar_is_running(void)
{
    return (uint32_t)(READ_BITS(CALENDAR->STAT, RTC_STAT_STAT) == RTC_STAT_STAT);
}

/**
  * @brief  CLear calendar wrap.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_CFG0 | Wrap counter clear
  *
  * @retval None
  */

__STATIC_INLINE void ll_calendar_clear_wrap(void)
{
    WRITE_REG(CALENDAR->CFG0, RTC_CFG0_CFG | RTC_CFG0_WRAP_CLR | CLDR_REG_READ);
}

/**
  * @brief  Select the CALENDAR clock divider.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_CFG1 | CLK_SEL
  *
  * @param  div This parameter can be one of the following values:
  *         @arg @ref LL_CALENDAR_DIV_NONE
  *         @arg @ref LL_CALENDAR_DIV_2
  *         @arg @ref LL_CALENDAR_DIV_4
  *         @arg @ref LL_CALENDAR_DIV_8
  *         @arg @ref LL_CALENDAR_DIV_16
  *         @arg @ref LL_CALENDAR_DIV_32
  *         @arg @ref LL_CALENDAR_DIV_64
  *         @arg @ref LL_CALENDAR_DIV_128
  * @retval None
  */
__STATIC_INLINE void ll_calendar_set_clock_div(uint32_t div)
{
    MODIFY_REG(CALENDAR->CFG1, RTC_CFG1_DIV, div);
}

/**
  * @brief  Enable calendar alarm.
  *
  *  Register|BitsName
  *  --------|--------
  *  CALENDAR_TIMER_CTL | ALARM_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_enable_alarm(void)
{
    WRITE_REG(CALENDAR->CFG0, RTC_CFG0_CFG | RTC_CFG0_ALARM_EN | CLDR_REG_READ);
}

/**
  * @brief  Enable calendar alarm interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  CALENDAR_TIMER_CTL | ALARM_INT_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_it_enable_alarm(void)
{
    SET_BITS(CALENDAR->INT_EN, RTC_INT_EN_ALARM);
}

/**
  * @brief  Disable calendar alarm.
  *
  *  Register|BitsName
  *  --------|--------
  *  CALENDAR_TIMER_CTL | ALARM_DISBALE
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_disable_alarm(void)
{
    WRITE_REG(CALENDAR->CFG0, (CLDR_REG_READ & (~RTC_CFG0_ALARM_EN)) | RTC_CFG0_CFG);
}

/**
  * @brief  Disable calendar alarm interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_INT_EN | ALARM_INT_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_it_disable_alarm(void)
{
    CLEAR_BITS(CALENDAR->INT_EN, RTC_INT_EN_ALARM);
}

/**
  * @brief  Check if the CALENDAR alarm interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CALENDAR_TIMER_CTL | ALARM_INT_EN
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_calendar_it_is_enabled_alarm(void)
{
    return (uint32_t)((READ_BITS(CALENDAR->CFG0, RTC_CFG0_ALARM_EN) == RTC_CFG0_ALARM_EN) &&
                    (READ_BITS(CALENDAR->INT_EN, RTC_INT_EN_ALARM) == RTC_INT_EN_ALARM));
}

/**
  * @brief  Set calendar tick mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CALENDAR_TIMER_CTL | TICK_EN
  * @param  tick_number: This parameter can be a one of the following values:
  *         @arg @ref LL_CLDR_TIMER_TICK
  *         @arg @ref LL_CLDR_TIMER_TICK
  * @param  tick_mode: This parameter can be a one of the following values:
  *         @arg @ref LL_CLDR_TIMER_TICK_TYPE_SINGLE
  *         @arg @ref LL_CLDR_TIMER_TICK_TYPE_AUTO
  * @retval None
*/
__STATIC_INLINE void ll_calendar_enable_tick(uint32_t tick_number, uint32_t tick_mode)
{
    UNUSED(tick_number);
    WRITE_REG(CALENDAR->CFG0, (RTC_CFG0_CFG | RTC_CFG0_TICK_EN | (tick_mode << RTC_CFG0_TICK_MDOE_Pos) | (CLDR_REG_READ & (~(1 << RTC_CFG0_TICK_MDOE_Pos)))));
}

/**
  * @brief  Enable calendar tick interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_INT_EN | TICK_INT_EN
  * @param  tick_number This parameter can be a one of the following values:
  *         @arg @ref LL_CLDR_TIMER_TICK
  *         @arg @ref LL_CLDR_TIMER_TICK
  * @retval None
  */
__STATIC_INLINE void ll_calendar_it_enable_tick(uint32_t tick_number)
{
    UNUSED(tick_number);
    SET_BITS(CALENDAR->INT_EN, RTC_INT_EN_TICK);
}

/**
  * @brief  Disable calendar tick.
  *
  *  Register|BitsName
  *  --------|--------
  *  CALENDAR_TIMER_CTL | TICK_EN
  * @param  tick_number This parameter can be a one of the following values:
  *         @arg @ref LL_CLDR_TIMER_TICK
  *         @arg @ref LL_CLDR_TIMER_TICK
  * @retval None
  */
__STATIC_INLINE void ll_calendar_disable_tick(uint32_t tick_number)
{
    UNUSED(tick_number);
    WRITE_REG(CALENDAR->CFG0, (CLDR_REG_READ & (~RTC_CFG0_TICK_EN)) | RTC_CFG0_CFG);
}

/**
  * @brief  Disable calendar tick interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_INT_EN | TICK0_INT_EN
  * @param  tick_number This parameter can be a one of the following values:
  *         @arg @ref LL_CLDR_TIMER_TICK
  *         @arg @ref LL_CLDR_TIMER_TICK
  * @retval None
  */
__STATIC_INLINE void ll_calendar_it_disable_tick(uint32_t tick_number)
{
    UNUSED(tick_number);
    CLEAR_BITS(CALENDAR->INT_EN, RTC_INT_EN_TICK);
}

/**
  * @brief  Check if the CALENDAR alarm interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CALENDAR_TIMER_CTL | TICK_EN
  *  CLDR_INT_EN | TICK_INT_EN
  * @param  tick_number This parameter can be a one of the following values:
  *         @arg @ref LL_CLDR_TIMER_TICK
  *         @arg @ref LL_CLDR_TIMER_TICK
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_calendar_it_is_enabled_tick(uint32_t tick_number)
{
    UNUSED(tick_number);
    return (uint32_t)((READ_BITS(CALENDAR->CFG0, RTC_CFG0_TICK_EN) == RTC_CFG0_TICK_EN) &&
                    (READ_BITS(CALENDAR->INT_EN, RTC_INT_EN_TICK) == RTC_INT_EN_TICK));
}

/**
  * @brief  Reloads CALENDAR tick counter and request.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_TIMER_W | TIMER_VALUE
  *  CLDR_CFG0 | VAL_LOAD
  * @param  tick_number: This parameter can be a one of the following values:
  *         @arg @ref LL_CLDR_TIMER_TICK;
  *         @arg @ref LL_CLDR_TIMER_TICK.
  * @param  counter:     Calendar tick counter.
  * @retval None
  */
__STATIC_INLINE void ll_calendar_reload_tick_and_request(uint32_t tick_number , uint32_t counter)
{
    UNUSED(tick_number);
    WRITE_REG(CALENDAR->TICK_W, counter);
    WRITE_REG(CALENDAR->CFG0, RTC_CFG0_CFG | RTC_CFG0_TICK_SET | CLDR_REG_READ);
}

/**
  * @brief  Enable calendar wrap interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  CALENDAR_TIMER_CTL | WRAP_INT_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_it_enable_wrap(void)
{
    SET_BITS(CALENDAR->INT_EN, RTC_INT_EN_WRAP);
}

/**
  * @brief  Disable calendar warp interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  CALENDAR_TIMER_CTL | WRAP_INT_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_it_disable_wrap(void)
{
    CLEAR_BITS(CALENDAR->INT_EN, RTC_INT_EN_WRAP);
}

/**
  * @brief  Check if the CALENDAR wrap interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CALENDAR_TIMER_CTL | WRAP_INT_EN
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_calendar_it_is_enabled_wrap(void)
{
    return (uint32_t)(READ_BITS(CALENDAR->INT_EN, RTC_INT_EN_WRAP) == RTC_INT_EN_WRAP);
}

/** @} */

/** @defgroup CALENDAR_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Indicate if the CALENDAR alarm event flag is set or not.
  * @note   This bit is set by hardware when the counter has reached alarm value.
  *         It can be cleared by writing 0 to this bit.
  *
  *  Register|BitsName
  *  --------|--------
  *  SLP_EVENT | CALENDAR_TIMER_ALARM
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_calendar_is_active_flag_alarm(void)
{
    return (uint32_t)(READ_BITS(CALENDAR->INT_STAT, RTC_INT_STAT_ALARM) == RTC_INT_STAT_ALARM);
}

/**
  * @brief  Indicate if the CALENDAR wrap event flag is set or not.
  * @note   This bit is set by hardware when the counter has overflow.
  *         It can be cleared by writing 0 to this bit.
  *
  *  Register|BitsName
  *  --------|--------
  *  SLP_EVENT | CALENDAR_TIMER_WRAP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_calendar_is_active_flag_wrap(void)
{
    return (uint32_t)(READ_BITS(CALENDAR->INT_STAT, RTC_INT_STAT_WRAP) == RTC_INT_STAT_WRAP);
}

/**
  * @brief  Indicate if the CALENDAR tick event flag is set or not.
  * @note   This bit is set by hardware when the counter has reached to 0.
  *         It can be cleared by writing 1 to this bit.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_INT_STAT | TICK_INT_STAT
  *
  * @param tick_number: This parameter can be a one of the following values:
  *         @arg @ref LL_CLDR_TIMER_TICK
  *         @arg @ref LL_CLDR_TIMER_TICK
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_calendar_is_active_flag_tick(uint32_t tick_number)
{
    UNUSED(tick_number);
    return (uint32_t)(READ_BITS(CALENDAR->INT_STAT, RTC_INT_STAT_TICK) == RTC_INT_STAT_TICK);
}

/**
  * @brief  Clear calendar alarm interrupt flag.
  *
  *  Register|BitsName
  *  --------|--------
  *  SLP_EVENT| CALENDAR_TIMER_ALARM
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_clear_flag_alarm(void)
{
    WRITE_REG(CALENDAR->INT_STAT, RTC_INT_STAT_ALARM);
}

/**
  * @brief  Clear calendar wrap interrupt flag.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLDR_INT_STAT| CALENDAR_TICK
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_clear_flag_wrap(void)
{
    WRITE_REG(CALENDAR->INT_STAT, RTC_INT_STAT_WRAP);
}

/**
  * @brief  Clear calendar tick interrupt flag.
  *
  *  Register|BitsName
  *  --------|--------
  *  SLP_EVENT| CALENDAR_TIMER_WRAP
  *
  * @param tick_number: This parameter can be a one of the following values:
  *         @arg @ref LL_CLDR_TIMER_TICK;
  *         @arg @ref LL_CLDR_TIMER_TICK.
  * @retval None
  */
__STATIC_INLINE void ll_calendar_clear_flag_tick(uint32_t tick_number)
{
    UNUSED(tick_number);
    WRITE_REG(CALENDAR->INT_STAT, RTC_INT_STAT_TICK);
}

/**
  * @brief  Clear calendar interrupt event.
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_clear_it_event(void)
{
    WRITE_REG(AON_CTL->AON_SLP_EVENT, ~AON_CTL_SLP_EVENT_RTC0);
}
/** @} */

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_CALENDAR_H__ */

/** @} */

/** @} */

/** @} */

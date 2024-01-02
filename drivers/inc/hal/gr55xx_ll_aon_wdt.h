/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_aon_wdt.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of AON WDT LL library.
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

/** @defgroup LL_AON_WDT AON_WDT
  * @brief AON_WDT LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_AON_WDT_H__
#define __GR55XX_LL_AON_WDT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

/** @defgroup  AON_WDT_LL_MACRO Defines
  * @{
  */
/** @defgroup  AON_WDT_LL_Exported_Constants Exported Constants
  * @{
  */
/** @defgroup AON_WDT_LL_TIMER_CLK_SEL  Watchdog Timer clock select defines
 * @{
 */
#define LL_AON_WD_TIMER_CLK_SEL_RNG             (0x0U << AON_WDT_CLK_SEL_Pos)               /**< Select RNG clcok source    */
#define LL_AON_WD_TIMER_CLK_SEL_XO              (0x1U << AON_WDT_CLK_SEL_Pos)               /**< Select XO clcok source     */
#define LL_AON_WD_TIMER_CLK_SEL_RNG2            (0x2U << AON_WDT_CLK_SEL_Pos)               /**< Select RNG2 clcok source   */
#define LL_AON_WD_TIMER_CLK_SEL_RTC             (0x3U << AON_WDT_CLK_SEL_Pos)                /**< Select RTC clcok source    */

/** @} */
/** @} */

/** @defgroup  AON_WDT_LL_Exported_Macros Exported Macros
  * @{
  */

/** @defgroup AON_WDT_LL_CFG_READ_REG config read
 * @{
 */
#define AON_WDT_REG_READ                        (READ_BITS(AON_WDT->CFG0, AON_WDT_CFG0_EN | \
                                                                    AON_WDT_CFG0_ALARM_EN))                         /**< read register config    */
/** @} */
/** @} */
/** @} */


/* Exported functions --------------------------------------------------------*/
/** @defgroup AON_WDT_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup AON_WDT_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable AON watchdog counter and interrupt event.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_WDT_CFG0 | WDT_EN
  */
__STATIC_INLINE void ll_aon_wdt_enable(void)
{
    WRITE_REG(AON_WDT->CFG0,AON_WDT_CFG0_CFG | AON_WDT_CFG0_EN | AON_WDT_REG_READ);
}

/**
  * @brief  Disable AON watchdog counter and interrupt event.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_WDT_CFG0 | WDT_EN
  *
  */
__STATIC_INLINE void ll_aon_wdt_disable(void)
{
    MODIFY_REG(AON_WDT->CFG0, 0xFFFFFFFF, AON_WDT_CFG0_CFG);
}

/**
  * @brief  Check if the AON_WDT peripheral is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_WDT_CFG0 | WDT_EN
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_wdt_is_enabled(void)
{
    return (READ_BITS(AON_WDT->CFG0, AON_WDT_CFG0_EN) == (AON_WDT_CFG0_EN));
}

/**
  * @brief  Set Watchdog Timer clock
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_WDT_CLK | wd_timer_clk_sel
  * @param  value: This parameter can be a one of the following values:
  *         @arg @ref LL_AON_WD_TIMER_CLK_SEL_RNG
  *         @arg @ref LL_AON_WD_TIMER_CLK_SEL_XO
  *         @arg @ref LL_AON_WD_TIMER_CLK_SEL_RNG2
  *         @arg @ref LL_AON_WD_TIMER_CLK_SEL_RTC
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_set_clk(uint32_t value)
{
    MODIFY_REG(AON_WDT->CLK, AON_WDT_CLK_SEL, value);
}

/**
  * @brief  Get Watchdog Timer clock
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_WDT_CLK | wd_timer_clk_sel
  *
  * @retval Watchdog Timer clock source,the value can be one of the following:
  *         @arg @ref LL_AON_WD_TIMER_CLK_SEL_RNG
  *         @arg @ref LL_AON_WD_TIMER_CLK_SEL_XO
  *         @arg @ref LL_AON_WD_TIMER_CLK_SEL_RNG2
  *         @arg @ref LL_AON_WD_TIMER_CLK_SEL_RTC
  */
__STATIC_INLINE uint32_t ll_aon_wdt_get_clk(void)
{
    return (READ_BITS(AON_WDT->CLK, AON_WDT_CLK_SEL));
}

/**
  * @brief  Specify the AON WDT down-counter reload value.
  *
  *  Register|BitsName
  *  --------|--------
  *  TIMER_VALUE | TIMER_VALUE
  * @param  counter: Value for reload down-counter which should ranging between 0 ~ 0xFFFF_FFFF
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_set_reload_counter(uint32_t counter)
{
    WRITE_REG(AON_WDT->TIMER_W, counter);
}

/**
  * @brief  Get the AON WDT down-counter reload value.
  *
  *  Register|BitsName
  *  --------|--------
  *  WD_TIMER_VAL_W | TIMER_VALUE
  *
  * @retval  counter Value for reload down-counter which should ranging between 0 ~ 0xFFFF_FFFF.
  */
__STATIC_INLINE uint32_t ll_aon_wdt_get_reload_counter(void)
{
    return (uint32_t)READ_BITS(AON_WDT->TIMER_W, AON_WDT_TIMER_W_VAL);
}

/**
  * @brief  Get the AON WDT down-counter read reload value.
  *
  *  Register|BitsName
  *  --------|--------
  *  WD_TIMER_STS_0 | TIMER_VALUE
  *
  * @retval  counter Value for reload down-counter which should ranging between 0 ~ 0xFFFF_FFFF.
  */
__STATIC_INLINE uint32_t ll_aon_wdt_get_reload_read_counter()
{
    return (uint32_t)READ_BITS(AON_WDT->TIMER_R, AON_WDT_TIMER_R_VAL);
}

/**
  * @brief  Reloads AON WDT counter.
  * @note   The value in TIMER_VALUE register will be reloaded into AON WDT down-counter
  *         after enable this bit, so ll_aon_wdt_set_reload_counter() should be called before
  *         every reload.
  *
  *  Register|BitsName
  *  --------|--------
  *  WD_TIMER_CFG_0 | WDT_RELOAD
  */
__STATIC_INLINE void ll_aon_wdt_reload_counter(void)
{
    WRITE_REG(AON_WDT->CFG0, AON_WDT_CFG0_CFG | AON_WDT_CFG0_TIMER_SET | AON_WDT_REG_READ);
}

/**
  * @brief  Read the AON WDT counter current value.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL1 | AON_WDT_TIMER
  *  TIMER_VAL    | TIMER_VAL_READ
  *
  * @retval Value for current counter which should ranging between 0 ~ 0xFFFF_FFFF
  */
__STATIC_INLINE uint32_t ll_aon_wdt_get_counter(void)
{
    return (uint32_t)READ_BITS(AON_WDT->TIMER_R, AON_WDT_TIMER_R_VAL);
}

/**
  * @brief  Specify the AON_WDT down-counter alarm value
  * @note   AON watchdog will generate an interrupt when it counts down to the
  *         alarm value to alram that it is almost expired.
  *
  *  Register|BitsName
  *  --------|--------
  *  EXT_WKUP_CTL | WDT_ALARM
  *
  * @param  counter: Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_set_alarm_counter(uint32_t counter)
{
    WRITE_REG(AON_WDT->ALARM_W, (counter & AON_WDT_ALARM_W_VAL));
}

/**
  * @brief  Specify the AON_WDT down-counter alarm value and request
  * @note   AON watchdog will generate an interrupt when it counts down to the
  *         alarm value to alram that it is almost expired.
  *
  *  Register|BitsName
  *  --------|--------
  *  EXT_WKUP_CTL | WDT_ALARM
  *
  * @param  counter: Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_set_alarm_counter_and_request(uint32_t counter)
{
    WRITE_REG(AON_WDT->ALARM_W, (counter & AON_WDT_ALARM_W_VAL));
    WRITE_REG(AON_WDT->CFG0, AON_WDT_CFG0_CFG | AON_WDT_CFG0_ALARM_SET | AON_WDT_REG_READ);
}

/**
  * @brief  Get the AON_WDT down-counter alarm value
  * @note   AON watchdog will generate an interrupt when it counts down to the
  *         alarm value to alram that it is almost expired.
  *
  *  Register|BitsName
  *  --------|--------
  *  WD_TIMER_ALARM | WDT_ALARM
  *
  * @retval  Value between Min_Data=0 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t ll_aon_wdt_get_alarm_counter(void)
{
    return (uint32_t)(READ_BITS(AON_WDT->ALARM_W, AON_WDT_ALARM_W_VAL));
}

/**
  * @brief  Get the AON_WDT down-counter alarm value
  *
  *  Register|BitsName
  *  --------|--------
  *  WD_TIMER_STS_1 | WDT_ALARM
  *
  * @retval Value between Min_Data=0 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t ll_aon_wdt_get_alarm_read_counter(void)
{
    return (uint32_t)(READ_BITS(AON_WDT->ALARM_R, AON_WDT_ALARM_R_VAL));
}

/**
  * @brief  AON WDT busy status flag.
  *
  *  Register|BitsName
  *  --------|--------
  *  WD_TIMER_STS| WD_TIMER_STAT_BUSY
  *
  * @retval AON WDT busy status flag.
  */
__STATIC_INLINE uint32_t ll_aon_wdt_is_busy(void)
{
    return (uint32_t)(READ_BITS(AON_WDT->STAT, AON_WDT_STAT_BUSY) == (AON_WDT_STAT_BUSY));
}

/**
  * @brief  Enable aon wdt alarm interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_WDT_CFG0 | ALARM_INT_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_it_enable_alarm(void)
{
    WRITE_REG(AON_WDT->CFG0, AON_WDT_CFG0_CFG | AON_WDT_CFG0_ALARM_EN | AON_WDT_REG_READ);
}

/**
  * @brief  Disable aon wdt alarm interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_WDT_CFG0 | ALARM_INT_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_it_disable_alarm(void)
{
    WRITE_REG(AON_WDT->CFG0, (AON_WDT_REG_READ & (~AON_WDT_CFG0_ALARM_EN)) | AON_WDT_CFG0_CFG);
}

/**
  * @brief  Check if the aon wdt alarm interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_WDT_CFG0 | ALARM_INT_EN
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_wdt_it_is_enabled_alarm(void)
{
    return (uint32_t)(READ_BITS(AON_WDT->CFG0, AON_WDT_CFG0_ALARM_EN) == AON_WDT_CFG0_ALARM_EN);
}

/** @} */

/** @defgroup AON_WDT_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Indicate if the AON Watchdog Running Flag is set or not.
  * @note   This bit can be used to check if AON Watchdog is in running state.
  *
  *  Register|BitsName
  *  --------|--------
  *  WD_TIMER_STS | WDT_RUNNING
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_wdt_is_active_flag_running(void)
{
    return (uint32_t)(READ_BITS(AON_WDT->STAT, AON_WDT_STAT_STAT) == (AON_WDT_STAT_STAT));
}

/**
  * @brief  Indicate if the AON WDT Reboot Event Flag is set or not.
  * @note   This bit is set by hardware when the counter has reached alarm value.
  *         It can be cleared by writing 0 to this bit.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_IRQ | WD_TIMER_REBOOT
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_wdt_is_active_flag_reboot(void)
{
    return (uint32_t)(READ_BITS(AON_CTL->AON_IRQ, AON_CTL_AON_IRQ_AON_WDT) == AON_CTL_AON_IRQ_AON_WDT);
}

/**
  * @brief  Clear Interrupt Status flag.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_IRQ| WD_TIMER_REBOOT
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_clear_flag_reboot(void)
{
    WRITE_REG(AON_CTL->AON_IRQ, ~AON_CTL_AON_IRQ_AON_WDT);
}

/**
  * @brief  Indicate if the AON WDT Alarm Event Flag is set or not.
  * @note   This bit is set by hardware when the counter has reached alarm value.
  *         It can be cleared by writing 0 to this bit.
  *
  *  Register|BitsName
  *  --------|--------
  *  SLP_EVENT | SLP_EVENT_WDT
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_wdt_is_active_flag_alarm(void)
{
    return (uint32_t)(READ_BITS(AON_CTL->AON_SLP_EVENT, AON_CTL_SLP_EVENT_AON_WDT) == AON_CTL_SLP_EVENT_AON_WDT);
}

/**
  * @brief  Clear Interrupt Status flag.
  *
  *  Register|BitsName
  *  --------|--------
  *  SLP_EVENT| SLP_EVENT_WDT
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_clear_flag_alarm(void)
{
    WRITE_REG(AON_CTL->AON_SLP_EVENT, ~AON_CTL_SLP_EVENT_AON_WDT);
}

/**
  * @brief  Enable write access.
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_enable_write_access(void)
{
   WRITE_REG(AON_WDT->LOCK, 0x15CC5A51 << 1);
}

/**
  * @brief  Disable write access.
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_disable_write_access(void)
{
    WRITE_REG(AON_WDT->LOCK, 0 << 1);
}

/** @} */

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_AON_WDT_PATCH_H__ */

/** @} */

/** @} */

/** @} */

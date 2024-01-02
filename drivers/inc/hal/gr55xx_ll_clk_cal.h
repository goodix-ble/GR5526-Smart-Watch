/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_clk_cal.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of CLOCK LL library.
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

/** @defgroup LL_CLK_CAL CLK_CAL
  * @brief CLOCK CALIBRATION LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_CLK_CAL_H_
#define __GR55XX_LL_CLK_CAL_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

/**
  * @defgroup  CLK_CAL_LL_MACRO Defines
  * @{
  */
/** @defgroup CLK_CAL_CLK_SOURCE Clock Calibration Source
  * @{
  */
#define LL_CLK_CAL_CLK_SOURCE_RTC_32K     (0 << CLK_CAL_SL_CLK_SEL_VAL_POS)  /**< slow clock source RTC */
#define LL_CLK_CAL_CLK_SOURCE_RNG_40K     (1 << CLK_CAL_SL_CLK_SEL_VAL_POS)  /**< slow clock source RTC */
#define LL_CLK_CAL_CLK_SOURCE_RNG_32K     (2 << CLK_CAL_SL_CLK_SEL_VAL_POS)  /**< slow clock source RTC */
/** @} */
/** @} */

/** @defgroup CLK_CAL_LL_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
  * @brief  Enable the clock calibration block
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_CTRL | EN
  */
__STATIC_INLINE void ll_sl_clk_cal_enable(void)
{
    SET_BITS(CLK_CAL->SL_CLK_CTRL, CLK_CAL_SL_CLK_CTRL_EN);
}

/**
  * @brief  Disable the clock calibration block
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_CTRL | EN
  *
  */
__STATIC_INLINE void ll_sl_clk_cal_disable(void)
{
    CLEAR_BITS(CLK_CAL->SL_CLK_CTRL, CLK_CAL_SL_CLK_CTRL_EN);
}

/**
  * @brief  Is the clock calibration block
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_CTRL | EN
  *
  */
__STATIC_INLINE uint32_t ll_sl_clk_cal_is_enable(void)
{
    return (READ_BITS(CLK_CAL->SL_CLK_CTRL, CLK_CAL_SL_CLK_CTRL_EN) == CLK_CAL_SL_CLK_CTRL_EN);
}

/**
  * @brief  Set the slow clock calibration count
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_CNT | CNT_COUNT
  * @param  count: 0x0 ~ 0xFFF.
  */
__STATIC_INLINE void ll_sl_clk_cal_set_slow_count(uint32_t count)
{
    MODIFY_REG(CLK_CAL->SL_CLK_CNT, CLK_CAL_SL_CLK_CNT_COUNT, (count << CLK_CAL_SL_CLK_CNT_COUNT_POS));
}

/**
  * @brief  Get the slow clock calibration setting count
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_CNT | CNT_COUNT
  *
  * @retval 0x0 ~ 0xFFF.
  */
__STATIC_INLINE uint32_t ll_sl_clk_cal_get_slow_count(void)
{
    return READ_BITS(CLK_CAL->SL_CLK_CNT, CLK_CAL_SL_CLK_CNT_COUNT);
}

/**
  * @brief  Get the clock calibration done status
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_STAT | READY
  *
  * @retval 0x0 ~ 0x1.
  */
__STATIC_INLINE uint32_t ll_sl_clk_cal_is_done(void)
{
    return (READ_BITS(CLK_CAL->SL_CLK_STAT, CLK_CAL_SL_CLK_STAT_DONE) == CLK_CAL_SL_CLK_STAT_DONE);
}

/**
  * @brief  Get the clock calibration overflow status
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_STAT | OVERFLOW
  *
  * @retval 0x0 ~ 0x1.
  */
__STATIC_INLINE uint32_t ll_sl_clk_cal_is_overflow(void)
{
    return (READ_BITS(CLK_CAL->SL_CLK_STAT, CLK_CAL_SL_CLK_STAT_OVER) == CLK_CAL_SL_CLK_STAT_OVER);
}

/**
  * @brief  Get the fast clock calibration count value
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_CNT0 | CNT0
  *
  * @retval 0x0 ~ 0xFFFFFF.
  */
__STATIC_INLINE uint32_t ll_sl_clk_cal_get_fast_count(void)
{
    return READ_BITS(CLK_CAL->SL_CLK_CNT0, CLK_CAL_SL_CLK_CNT0_VAL);
}

/**
  * @brief  Get the slow clock calibration current count value
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_CNT1 | CNT1
  *
  * @retval 0x0 ~ 0xFFF.
  */
__STATIC_INLINE uint32_t ll_sl_clk_cal_get_slow_cur_count(void)
{
    return READ_BITS(CLK_CAL->SL_CLK_CNT1, CLK_CAL_SL_CLK_CNT1_VAL);
}

/**
  * @brief  Set the clock calibration clock source
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_CNT | CLK_SOURCE
  * @param  clk_source: 0x0 ~ 0x2.
  */
__STATIC_INLINE void ll_sl_clk_cal_set_clk_source(uint32_t clk_source)
{
    MODIFY_REG(CLK_CAL->SL_CLK_SEL, CLK_CAL_SL_CLK_SEL_VAL, clk_source);
}

/**
  * @brief  Get the clock calibration clock source
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_CNT | CLK_SOURCE
  *
  * @retval 0x0 ~ 0x2.
  */
__STATIC_INLINE uint32_t ll_sl_clk_cal_get_clk_source(void)
{
    return READ_BITS(CLK_CAL->SL_CLK_SEL, CLK_CAL_SL_CLK_SEL_VAL);
}

/**
  * @brief  Enable the clock calibration done interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_INT_EN | DONE
  *
  */
__STATIC_INLINE void ll_sl_clk_cal_enable_done_it(void)
{
    SET_BITS(CLK_CAL->SL_CLK_INT_EN, CLK_CAL_SL_CLK_INT_EN_DONE);
}

/**
  * @brief  Disable the clock calibration done interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_INT_EN | DONE
  *
  */
__STATIC_INLINE void ll_sl_clk_cal_disable_done_it(void)
{
    CLEAR_BITS(CLK_CAL->SL_CLK_INT_EN, CLK_CAL_SL_CLK_INT_EN_DONE);
}

/**
  * @brief  Enable the clock calibration overflow interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_INT_EN | OVER
  *
  */
__STATIC_INLINE void ll_sl_clk_cal_enable_over_it(void)
{
    SET_BITS(CLK_CAL->SL_CLK_INT_EN, CLK_CAL_SL_CLK_INT_EN_OVER);
}

/**
  * @brief  Disable the clock calibration overflow interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_INT_EN | OVER
  *
  */
__STATIC_INLINE void ll_sl_clk_cal_disable_over_it(void)
{
    CLEAR_BITS(CLK_CAL->SL_CLK_INT_EN, CLK_CAL_SL_CLK_INT_EN_OVER);
}

/**
  * @brief  Clear the clock calibration done interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_INT_EN | DONE
  *
  */
__STATIC_INLINE void ll_sl_clk_cal_clear_done_it(void)
{
    SET_BITS(CLK_CAL->SL_CLK_INT_CLR, CLK_CAL_SL_CLK_INT_CLR_DONE);
}

/**
  * @brief  Clear the clock calibration over interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_INT_EN | DONE
  *
  */
__STATIC_INLINE void ll_sl_clk_cal_clear_over_it(void)
{
    SET_BITS(CLK_CAL->SL_CLK_INT_CLR, CLK_CAL_SL_CLK_INT_CLR_OVER);
}

/**
  * @brief  Enable the clock calibration block
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_CTRL | EN
  */
__STATIC_INLINE void ll_hs_clk_cal_enable(void)
{
    SET_BITS(CLK_CAL->HS_CLK_CTRL, CLK_CAL_HS_CLK_CTRL_EN);
}

/**
  * @brief  Disable the clock calibration block
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_CTRL | EN
  *
  */
__STATIC_INLINE void ll_hs_clk_cal_disable(void)
{
    CLEAR_BITS(CLK_CAL->HS_CLK_CTRL, CLK_CAL_HS_CLK_CTRL_EN);
}

/**
  * @brief  Is the clock calibration block
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_CTRL | EN
  *
  */

__STATIC_INLINE uint32_t ll_hs_clk_cal_is_enable(void)
{
    return (READ_BITS(CLK_CAL->HS_CLK_CTRL, CLK_CAL_HS_CLK_CTRL_EN) == CLK_CAL_HS_CLK_CTRL_EN);
}

/**
  * @brief  Set the slow clock calibration count
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_CNT | CNT_COUNT
  * @param  count: 0x0 ~ 0xFFF.
  */
__STATIC_INLINE void ll_hs_clk_cal_set_slow_count(uint32_t count)
{
    MODIFY_REG(CLK_CAL->HS_CLK_CNT, CLK_CAL_HS_CLK_CNT_COUNT, (count << CLK_CAL_HS_CLK_CNT_COUNT_POS));
}

/**
  * @brief  Get the slow clock setting count
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_CNT | CNT_COUNT
  *
  * @retval 0x0 ~ 0xFFF.
  */
__STATIC_INLINE uint32_t ll_hs_clk_cal_get_slow_count(void)
{
    return READ_BITS(CLK_CAL->HS_CLK_CNT, CLK_CAL_HS_CLK_CNT_COUNT);
}

/**
  * @brief  Get the clock calibration done status
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_STAT | READY
  *
  * @retval 0x0 ~ 0x1.
  */
__STATIC_INLINE uint32_t ll_hs_clk_cal_is_done(void)
{
    return (READ_BITS(CLK_CAL->HS_CLK_STAT, CLK_CAL_HS_CLK_STAT_DONE) == CLK_CAL_HS_CLK_STAT_DONE);
}

/**
  * @brief  Get the clock calibration overflow status
  *
  *  Register|BitsName
  *  --------|--------
  *  SL_CLK_STAT | OVERFLOW
  *
  * @retval 0x0 ~ 0x1.
  */
__STATIC_INLINE uint32_t ll_hs_clk_cal_is_overflow(void)
{
    return (READ_BITS(CLK_CAL->HS_CLK_STAT, CLK_CAL_HS_CLK_STAT_OVER) == CLK_CAL_HS_CLK_STAT_OVER);
}

/**
  * @brief  Get the fast clock calibration count value
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_CNT0 | CNT0
  *
  * @retval 0x0 ~ 0xFFFFFF.
  */
__STATIC_INLINE uint32_t ll_hs_clk_cal_get_fast_count(void)
{
    return READ_BITS(CLK_CAL->HS_CLK_CNT0, CLK_CAL_HS_CLK_CNT0_VAL);
}

/**
  * @brief  Get the slow clock calibration current count value
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_CNT1 | CNT1
  *
  * @retval 0x0 ~ 0xFFF.
  */
__STATIC_INLINE uint32_t ll_hs_clk_cal_get_slow_cur_count(void)
{
    return READ_BITS(CLK_CAL->HS_CLK_CNT1, CLK_CAL_HS_CLK_CNT1_VAL);
}

/**
  * @brief  Set the clock calibration clock source
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_CNT | CLK_SOURCE
  * @param  clk_source: 0x0 ~ 0x2.
  */
__STATIC_INLINE void ll_hs_clk_cal_set_clk_source(uint32_t clk_source)
{
    MODIFY_REG(CLK_CAL->HS_CLK_SEL, CLK_CAL_HS_CLK_SEL_VAL, clk_source);
}

/**
  * @brief  Get the clock calibration clock source
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_CNT | CLK_SOURCE
  *
  * @retval 0x0 ~ 0x2.
  */
__STATIC_INLINE uint32_t ll_hs_clk_cal_get_clk_source(void)
{
    return READ_BITS(CLK_CAL->HS_CLK_SEL, CLK_CAL_HS_CLK_SEL_VAL);
}

/**
  * @brief  Enable the clock calibration done interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_INT_EN | DONE
  *
  */
__STATIC_INLINE void ll_hs_clk_cal_enable_done_it(void)
{
    SET_BITS(CLK_CAL->HS_CLK_INT_EN, CLK_CAL_HS_CLK_INT_EN_DONE);
}

/**
  * @brief  Disable the clock calibration done interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_INT_EN | DONE
  *
  */
__STATIC_INLINE void ll_hs_clk_cal_disable_done_it(void)
{
    CLEAR_BITS(CLK_CAL->HS_CLK_INT_EN, CLK_CAL_HS_CLK_INT_EN_DONE);
}

/**
  * @brief  Enable the clock calibration overflow interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_INT_EN | OVER
  *
  */
__STATIC_INLINE void ll_hs_clk_cal_enable_over_it(void)
{
    SET_BITS(CLK_CAL->HS_CLK_INT_EN, CLK_CAL_HS_CLK_INT_EN_OVER);
}

/**
  * @brief  Disable the clock calibration overflow interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_INT_EN | OVER
  *
  */
__STATIC_INLINE void ll_hs_clk_cal_disable_over_it(void)
{
    CLEAR_BITS(CLK_CAL->HS_CLK_INT_EN, CLK_CAL_HS_CLK_INT_EN_OVER);
}

/**
  * @brief  Clear the clock calibration done interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_INT_EN | DONE
  *
  */
__STATIC_INLINE void ll_hs_clk_cal_clear_done_it(void)
{
    SET_BITS(CLK_CAL->HS_CLK_INT_CLR, CLK_CAL_HS_CLK_INT_CLR_DONE);
}

/**
  * @brief  Clear the clock calibration over interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  HS_CLK_INT_EN | DONE
  *
  */
__STATIC_INLINE void ll_hs_clk_cal_clear_over_it(void)
{
    SET_BITS(CLK_CAL->HS_CLK_INT_CLR, CLK_CAL_HS_CLK_INT_CLR_OVER);
}
/** @} */

#endif
/** @} */

/** @} */

/** @} */

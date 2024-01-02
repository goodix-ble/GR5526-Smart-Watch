/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_clk.h
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

/** @defgroup LL_CLK CLK
  * @brief CLOCK CALIBRATION LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_CLK_H_
#define __GR55XX_LL_CLK_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

/**
  * @defgroup  LL_CLK_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup LL_CLK_Exported_Constants CLK Exported Constants
  * @{
  */

/** @defgroup CLK_SOURCE Clock source select
  * @{
  */

#define LL_CLK_SEL_SOURCE_CPLL_CLK          (0UL)                                       /**< Select CPLL clk as the source of the 192MHz clock */
#define LL_CLK_SEL_SOURCE_HF_OSC_CLK        (1UL)                                       /**< Select hf osc clk as the source of the 192MHz clock */
#define LL_CLK_SEL_FAST_WAKEUP_CPLL_CLK     (0UL)                                       /**< Select CPLL clk as fast wakeup clk */
#define LL_CLK_SEL_FAST_WAKEUP_HF_OSC_CLK   (AON_CTL_AON_CLK_WAKUP_FAST_CLK_SEL_Msk)    /**< Select hf osc clk as fast wakeup clk */

/** @} */

/** @defgroup CLK_SELECT System clock frequency select
  * @{
  */
#define LL_CLK_CPLL_S96M_CLK            AON_CTL_MCU_CLK_CTRL_SEL_96M     /**< Select PLL/HF_OSC 96MHz clk as system clock */
#define LL_CLK_CPLL_S64M_CLK            AON_CTL_MCU_CLK_CTRL_SEL_64M     /**< Select PLL/HF_OSC 64MHz clk as system clock */
#define LL_CLK_XO_S16M_CLK              AON_CTL_MCU_CLK_CTRL_SEL_XO_16M  /**< Select XO 16MHz clk as system clock */
#define LL_CLK_CPLL_F48M_CLK            AON_CTL_MCU_CLK_CTRL_SEL_48M     /**< Select PLL/HF_OSC 48MHz clk as system clock */
#define LL_CLK_CPLL_T24M_CLK            AON_CTL_MCU_CLK_CTRL_SEL_24M     /**< Select PLL/HF_OSC 24MHz clk as system clock */
#define LL_CLK_CPLL_S16M_CLK            AON_CTL_MCU_CLK_CTRL_SEL_16M     /**< Select PLL/HF_OSC 16MHz clk as system clock */
#define LL_CLK_CPLL_T32M_CLK            AON_CTL_MCU_CLK_CTRL_SEL_32M     /**< Select PLL/HF_OSC 32MHz clk as system clock */

#define LL_CLK_AON_CLK_WAKUP_CLK_EN     (1 << AON_CTL_AON_CLK_WAKUP_CLK_EN_Pos)     /**< wakeup clock enable  */
#define LL_CLK_AON_CLK_WAKUP_CLK_DIS    0                                           /**< wakeup clock disable */
/** @} */

/** @defgroup XO_PLL_STATE XO PLL state
  * @{
  */
#define LL_CLK_XO_PLL_PLL_STAT          (1UL)       /**< XO PLL state, PLL */
#define LL_CLK_XO_PLL_XO_STAT           (2UL)       /**< XO PLL state, XO  */
#define LL_CLK_XO_PLL_HF_STAT           (4UL)       /**< XO PLL state, HF  */
/** @} */

/** @} */
/** @} */

/** @defgroup LL_CLK_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
  * @brief  Get system clock.
  *
  *  Register|BitsName
  *  --------|--------
  *  MCU_CLK_CTRL | CLK_CTRL_SEL
  *
  * @retval None
  *
  */
__STATIC_INLINE uint32_t ll_clk_get_sys_clk(void)
{
    return READ_BITS(AON_CTL->MCU_CLK_CTRL, AON_CTL_MCU_CLK_CTRL_SEL);
}

/**
  * @brief  Set system clock.
  *
  *  Register|BitsName
  *  --------|--------
  *  MCU_CLK_CTRL | CLK_CTRL_SEL
  *
  * @param  clk_sel This parameter can be a combination of the following values:
  *         @arg @ref LL_CLK_CPLL_S96M_CLK
  *         @arg @ref LL_CLK_CPLL_S64M_CLK
  *         @arg @ref LL_CLK_XO_S16M_CLK
  *         @arg @ref LL_CLK_CPLL_F48M_CLK
  *         @arg @ref LL_CLK_CPLL_T24M_CLK
  *         @arg @ref LL_CLK_CPLL_S16M_CLK
  *         @arg @ref LL_CLK_CPLL_T32M_CLK
  *
  */
__STATIC_INLINE void ll_clk_set_sys_clk(uint32_t clk_sel)
{
    // Need twice writing to ensure success for this register
    MODIFY_REG(AON_CTL->MCU_CLK_CTRL, AON_CTL_MCU_CLK_CTRL_SEL, clk_sel);
    MODIFY_REG(AON_CTL->MCU_CLK_CTRL, AON_CTL_MCU_CLK_CTRL_SEL, clk_sel);
}

/**
  * @brief  set AON_CTL_AON_CLK_WAKUP_CLK_EN bit
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_CLK | AON_CTL_AON_CLK_WAKUP_CLK_EN
  *
  * @param  wakeup_clk_en This parameter can be a combination of the following values:
  *         @arg @ref LL_CLK_AON_CLK_WAKUP_CLK_EN
  *         @arg @ref LL_CLK_AON_CLK_WAKUP_CLK_DIS
  *
  */
__STATIC_INLINE void ll_clk_set_aon_clk_wakeup_clk_en(uint32_t wakeup_clk_en)
{
    MODIFY_REG(AON_CTL->AON_CLK, AON_CTL_AON_CLK_WAKUP_CLK_EN, wakeup_clk_en);
}
/**
  * @brief  Select clock source.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_CLK | CAL_FST_CLK
  *
  * @param  src_sel This parameter can be a combination of the following values:
  *         @arg @ref LL_CLK_SEL_SOURCE_CPLL_CLK
  *         @arg @ref LL_CLK_SEL_SOURCE_HF_OSC_CLK
  *
  */
__STATIC_INLINE void ll_clk_select_source(uint32_t src_sel)
{
    // Need twice writing to ensure success for this register
    MODIFY_REG(AON_CTL->AON_CLK, AON_CTL_AON_CLK_CAL_FST_CLK, src_sel);
    MODIFY_REG(AON_CTL->AON_CLK, AON_CTL_AON_CLK_CAL_FST_CLK, src_sel);
}
/**
  * @brief  Select clock source.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_CLK | WAKUP_FAST_CLK_SEL
  *
  * @param  src_sel This parameter can be a combination of the following values:
  *         @arg @ref LL_CLK_SEL_SOURCE_CPLL_CLK
  *         @arg @ref LL_CLK_SEL_SOURCE_HF_OSC_CLK
  *
  */
__STATIC_INLINE void ll_clk_select_fast_wakeup_source(uint32_t src_sel)
{
    MODIFY_REG(AON_CTL->AON_CLK, AON_CTL_AON_CLK_WAKUP_FAST_CLK_SEL, src_sel);
}

/**
  * @brief  start XO and PLL
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PWR | XO_PLL_SET
  *
  * @retval void.
  *
  */
__STATIC_INLINE void ll_clk_start_xo_pll(void)
{
    MODIFY_REG(AON_PWR->XO_PLL_SET, AON_PWR_XO_PLL_SET_PLL_SET_Msk | AON_PWR_XO_PLL_SET_XO_SET_Msk, AON_PWR_XO_PLL_SET_PLL_SET | AON_PWR_XO_PLL_SET_XO_SET);
}

/**
  * @brief  stop XO and PLL
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PWR | XO_PLL_CLR
  *
  * @retval void.
  *
  */
__STATIC_INLINE void ll_clk_stop_xo_pll(void)
{
    MODIFY_REG(AON_PWR->XO_PLL_CLR, AON_PWR_XO_PLL_SET_PLL_SET_Msk | AON_PWR_XO_PLL_SET_XO_SET_Msk, AON_PWR_XO_PLL_SET_PLL_SET | AON_PWR_XO_PLL_SET_XO_SET);
}

/**
  * @brief  Get XO PLL status.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PWR | XO_PLL_STAT
  *
  * @retval xo pll status value.
  *
  */
__STATIC_INLINE uint32_t ll_clk_get_hf_status(void)
{
    return READ_BITS(AON_PWR->XO_PLL_STAT, AON_PWR_XO_PLL_STAT_PLL_STAT |
                                           AON_PWR_XO_PLL_STAT_XO_STAT  |
                                           AON_PWR_XO_PLL_STAT_HF_STAT);
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */

/** @} */

/** @} */

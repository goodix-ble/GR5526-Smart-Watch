/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_ddvs.h
 * @author  BLE RD
 * @brief   Header file containing functions prototypes of DDVS LL library.
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

/** @defgroup LL_DDVS DDVS
  * @brief DDVS LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_DDVS_H_
#define __GR55XX_LL_DDVS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

/**
  * @defgroup  DDVS_LL_MACRO Defines
  * @{
  */
/** @defgroup DDVS_ENABLE DDVS enable
  * @{
  */
#define LL_DDVS_DIS                              (0U)  /**< DDVS Disable(default) */
#define LL_DDVS_EN                               (1U)  /**< DDVS Enable */
/** @} */

/** @defgroup DDVS_MODE Mode defines
  * @{
  */
#define LL_DDVS_AUTO_MODE                        (0U)  /**< DDVS Auto Mode(default) */
#define LL_DDVS_MALNUAL_MODE                     (1U)  /**< DDVS Manual Mode */
/** @} */

/** @defgroup DDVS_RINGO Ringo defines
  * @{
  */
#define LL_DDVS_RINGO_0_EN                       (1<<0U)  /**< DDVS Ringo_0 Enable */
#define LL_DDVS_RINGO_1_EN                       (1<<1U)  /**< DDVS Ringo_1 Enable */
#define LL_DDVS_RINGO_2_EN                       (1<<2U)  /**< DDVS Ringo_2 Enable */
#define LL_DDVS_RINGO_3_EN                       (1<<3U)  /**< DDVS Ringo_3 Enable */
#define LL_DDVS_RINGO_ALL_EN                     (0xFU)   /**< DDVS All Ringos Enable */
#define LL_DDVS_RINGO_ALL_DIS                    (0x0U)   /**< DDVS All Ringos Disable */
/** @} */

/** @defgroup DDVS_DIVIDE_FACTOR Divide factor defines
  * @{
  */
#define LL_DDVS_DIVIDE_FACTOR_8K                 (0U)  /**< DDVS div_factor 8k */
#define LL_DDVS_DIVIDE_FACTOR_4K                 (1U)  /**< DDVS div_factor 4k */
#define LL_DDVS_DIVIDE_FACTOR_16K                (2U)  /**< DDVS div_factor 16k */
/** @} */

/** @defgroup DDVS_CLOCK_SEL Clock Selection defines
  * @{
  */
#define LL_DDVS_CLK_SEL_XO_32M                   (0U)  /**< DDVS Clock Select XO_32M */
#define LL_DDVS_CLK_SEL_XO_16M                   (1U)  /**< DDVS Clock Select XO_16M */
#define LL_DDVS_CLK_SEL_SYS_32M                  (2U)  /**< DDVS Clock Select SYS_32M */
#define LL_DDVS_CLK_SEL_SYS_16M                  (3U)  /**< DDVS Clock Select SYS_16M */
/** @} */

/** @defgroup DDVS_CLOCK_SWITCH Clock defines
  * @{
  */
#define LL_DDVS_CLK_DIS                          (0U)  /**< DDVS Clock Disable(default) */
#define LL_DDVS_CLK_EN                           (1U)  /**< DDVS Clock Enable */
/** @} */

/** @} */

/** @defgroup DDVS_LL_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
  * @brief DDVS enable state set
  *
  *  Register|BitsName
  *  --------|--------
  *  DDVS_EN | CONF_DDVS_EN
  */
__STATIC_INLINE void ll_ddvs_enable_set(uint8_t enable)
{
    MODIFY_REG(DDVS_CTRL->DDVS_EN, DDVS_CTRL_CONF_DDVS_EN, (enable << DDVS_CTRL_CONF_DDVS_EN_POS));
}

/**
  * @brief  DDVS enable state get
  *
  *  Register|BitsName
  *  --------|--------
  *  DDVS_EN | CONF_DDVS_EN
  */
__STATIC_INLINE uint8_t ll_ddvs_enable_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_EN, DDVS_CTRL_CONF_DDVS_EN)) >> DDVS_CTRL_CONF_DDVS_EN_POS);
}

/**
  * @brief  DDVS mode set
  *  0b - DDVS automatic mode (default)
  *  1b - DDVS manual mode
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | CONF_DDVS_MODE
  */
__STATIC_INLINE void ll_ddvs_mode_set(uint8_t mode)
{
    MODIFY_REG(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_DDVS_MODE, (mode << DDVS_CFG_1_CONF_DDVS_MODE_POS));
}

/**
  * @brief  DDVS mode get
  *  0b - DDVS automatic mode (default)
  *  1b - DDVS manual mode
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | CONF_DDVS_MODE
  */
__STATIC_INLINE uint8_t ll_ddvs_mode_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_DDVS_MODE)) >> DDVS_CFG_1_CONF_DDVS_MODE_POS);
}

/**
  * @brief  DDVS Slow threshold(limit for ringo goes bigger) set
  *  This is the limitation of ringo being bigger than target_cnt
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | CONF_THRESHOLD_SLOW
  */
__STATIC_INLINE void ll_ddvs_slow_threshold_set(uint16_t threshold)
{
    MODIFY_REG(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_THRESHOLD_SLOW, (threshold << DDVS_CFG_1_CONF_THRESHOLD_SLOW_POS));
}

/**
  * @brief  DDVS Slow threshold(limit for ringo goes bigger) get
  *  This is the limitation of ringo being bigger than target_cnt
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | CONF_THRESHOLD_SLOW
  */
__STATIC_INLINE uint16_t ll_ddvs_slow_threshold_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_THRESHOLD_SLOW)) >> DDVS_CFG_1_CONF_THRESHOLD_SLOW_POS);
}

/**
  * @brief  DDVS Target count set
  *
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_2 | CONF_TARGET_CNT
  */
__STATIC_INLINE void ll_ddvs_target_cnt_set(uint16_t target_cnt)
{
    MODIFY_REG(DDVS_CTRL->DDVS_CFG_2, DDVS_CFG_2_CONF_TARGET_CNT, (target_cnt << DDVS_CFG_2_CONF_TARGET_CNT_POS));
}

/**
  * @brief  DDVS Target count get
  *
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_2 | CONF_TARGET_CNT
  */
__STATIC_INLINE uint16_t ll_ddvs_target_cnt_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_CFG_2, DDVS_CFG_2_CONF_TARGET_CNT)) >> DDVS_CFG_2_CONF_TARGET_CNT_POS);
}

/**
  * @brief  DDVS Fast threshold set
  *  This is the limitation of ringo being smaller than target_cnt
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_2 | CONF_THRESHOLD_FAST
  */
__STATIC_INLINE void ll_ddvs_fast_threshold_set(uint16_t threshold)
{
    MODIFY_REG(DDVS_CTRL->DDVS_CFG_2, DDVS_CFG_2_CONF_THRESHOLD_FAST, (threshold << DDVS_CFG_2_CONF_THRESHOLD_FAST_POS));
}

/**
  * @brief  DDVS Fast threshold get
  *  This is the limitation of ringo being smaller than target_cnt
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_2 | CONF_THRESHOLD_FAST
  */
__STATIC_INLINE uint16_t ll_ddvs_fast_threshold_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_CFG_2, DDVS_CFG_2_CONF_THRESHOLD_FAST)) >> DDVS_CFG_2_CONF_THRESHOLD_FAST_POS);
}

/**
  * @brief  DDVS Error interrupt enable
  *
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | ERR_INT
  */
__STATIC_INLINE void ll_ddvs_err_int_enable(void)
{
    SET_BITS(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_INT_EN);
}

/**
  * @brief  DDVS Error interrupt disable
  *
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | ERR_INT
  */
__STATIC_INLINE void ll_ddvs_err_int_disable(void)
{
    CLEAR_BITS(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_INT_EN);
}

/**
  * @brief  DDVS Error interrupt clear
  *  From Spec, the ddvs mode shall set to manual mode before clear the interrupt
  *  When the error happens, it means the target_cnt shall be increased
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | ERR_INT
  */
__STATIC_INLINE void ll_ddvs_err_int_clear(void)
{
    MODIFY_REG(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_DDVS_MODE, (LL_DDVS_MALNUAL_MODE << DDVS_CFG_1_CONF_DDVS_MODE_POS));
    CLEAR_BITS(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_ERR_INT);
}

/**
  * @brief  DDVS Manual vref set
  *
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | CONF_VREF_MANUAL
  */
__STATIC_INLINE void ll_ddvs_manual_vref_set(uint8_t vref)
{
    MODIFY_REG(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_VREF_MANUAL, (vref << DDVS_CFG_1_CONF_VREF_MANUAL_POS));
}

/**
  * @brief  DDVS Manual vref get
  *
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | CONF_VREF_MANUAL
  */
__STATIC_INLINE uint8_t ll_ddvs_manual_vref_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_VREF_MANUAL)) >> DDVS_CFG_1_CONF_VREF_MANUAL_POS);
}

/**
  * @brief  DDVS Ringo enable set
  *  0000b - disable all 4 ringos (default)
  *  0001b~1111b - enable corresponding ringos
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | CONF_RINGO_EN
  */
__STATIC_INLINE void ll_ddvs_ringo_en_set(uint8_t ringo_bits)
{
    MODIFY_REG(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_RINGO_EN, (ringo_bits << DDVS_CFG_1_CONF_RINGO_EN_POS));
}

/**
  * @brief  DDVS Ringo enable get
  *  0000b - disable all 4 ringos (default)
  *  0001b~1111b - enable corresponding ringos
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | CONF_RINGO_EN
  */
__STATIC_INLINE uint8_t ll_ddvs_ringo_en_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_RINGO_EN)) >> DDVS_CFG_1_CONF_RINGO_EN_POS);
}

/**
  * @brief  DDVS Ringo frequency divide factor set
  * 00b - 8K (default)
  * 01b - 4K
  * 10b - 16K
  * 11b - Reserved
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | CONF_DIV_FACTOR
  */
__STATIC_INLINE void ll_ddvs_div_factor_set(uint8_t factor)
{
    MODIFY_REG(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_DIV_FACTOR, (factor<<DDVS_CFG_1_CONF_DIV_FACTOR_POS));
}

/**
  * @brief  DDVS Ringo frequency divide factor get
  * 00b - 8K (default)
  * 01b - 4K
  * 10b - 16K
  * 11b - Reserved
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CFG_1 | CONF_DIV_FACTOR
  */
__STATIC_INLINE uint8_t ll_ddvs_div_factor_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_CFG_1, DDVS_CFG_1_CONF_DIV_FACTOR)) >> DDVS_CFG_1_CONF_DIV_FACTOR_POS);
}

/**
  * @brief  DDVS Ringo_0 count get
  *  Register|BitsName
  *  --------|--------
  *  DDVS_RINGO_CNT_01 | RINGO_0_CNT
  */
__STATIC_INLINE uint16_t ll_ddvs_ringo_0_cnt_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_RINGO_CNT_01, DDVS_RINGO_CNT_01_RINGO_0_CNT)) >> DDVS_RINGO_CNT_01_RINGO_0_CNT_POS);
}

/**
  * @brief  DDVS Ringo_1 count get
  *  Register|BitsName
  *  --------|--------
  *  DDVS_RINGO_CNT_01 | RINGO_1_CNT
  */
__STATIC_INLINE uint16_t ll_ddvs_ringo_1_cnt_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_RINGO_CNT_01, DDVS_RINGO_CNT_01_RINGO_1_CNT)) >> DDVS_RINGO_CNT_01_RINGO_1_CNT_POS);
}

/**
  * @brief  DDVS Ringo_2 count get
  *  Register|BitsName
  *  --------|--------
  *  DDVS_RINGO_CNT_23 | RINGO_2_CNT
  */
__STATIC_INLINE uint16_t ll_ddvs_ringo_2_cnt_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_RINGO_CNT_23, DDVS_RINGO_CNT_23_RINGO_2_CNT)) >> DDVS_RINGO_CNT_23_RINGO_2_CNT_POS);
}

/**
  * @brief  DDVS Ringo_3 count get
  *  Register|BitsName
  *  --------|--------
  *  DDVS_RINGO_CNT_23 | RINGO_3_CNT
  */
__STATIC_INLINE uint16_t ll_ddvs_ringo_3_cnt_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_RINGO_CNT_23, DDVS_RINGO_CNT_23_RINGO_3_CNT)) >> DDVS_RINGO_CNT_23_RINGO_3_CNT_POS);
}

/**
  * @brief  DDVS FSM state get
  *  Register|BitsName
  *  --------|--------
  *  DDVS_FSM | STS_FSM_CURR
  */
__STATIC_INLINE uint8_t ll_ddvs_fsm_state_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_FSM, DDVS_FSM_STS_FSM_CURR)) >> DDVS_FSM_STS_FSM_CURR_POS);
}

/**
  * @brief  DDVS Clock enable set
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CLK_CTRL | DDVS_CLK_EN
  */
__STATIC_INLINE void ll_ddvs_clk_enable_set(uint8_t enable)
{
    MODIFY_REG(DDVS_CTRL->DDVS_CLK_CTRL, DDVS_CLK_CTRL_DDVS_CLK_EN, (enable << DDVS_CLK_CTRL_DDVS_CLK_EN_POS));
}

/**
  * @brief  DDVS Clock enable get
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CLK_CTRL | DDVS_CLK_EN
  */
__STATIC_INLINE uint8_t ll_ddvs_clk_enable_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_CLK_CTRL, DDVS_CLK_CTRL_DDVS_CLK_EN)) >> DDVS_CLK_CTRL_DDVS_CLK_EN_POS);
}

/**
  * @brief  DDVS Clock selection set
  *  0: xo_32MHz(default), 1: xo_16MHz, 2: sys_32MHz, 3: sys_16MHz
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CLK_CTRL | CONF_RINGO_EN
  */
__STATIC_INLINE void ll_ddvs_clk_sel_set(uint8_t clk_sel)
{
    MODIFY_REG(DDVS_CTRL->DDVS_CLK_CTRL, DDVS_CLK_CTRL_DDVS_CLK_SEL, (clk_sel<<DDVS_CLK_CTRL_DDVS_CLK_SEL_POS));
}

/**
  * @brief  DDVS Clock selection get
  *  0: xo_32MHz(default), 1: xo_16MHz, 2: sys_32MHz, 3: sys_16MHz
  *  Register|BitsName
  *  --------|--------
  *  DDVS_CLK_CTRL | CONF_RINGO_EN
  */
__STATIC_INLINE uint8_t ll_ddvs_clk_sel_get(void)
{
    return ((READ_BITS(DDVS_CTRL->DDVS_CLK_CTRL, DDVS_CLK_CTRL_DDVS_CLK_SEL)) >> DDVS_CLK_CTRL_DDVS_CLK_SEL_POS);
}

/** @} */

#endif
/** @} */

/** @} */

/** @} */

/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_aon_rf.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of AON RF LL library.
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

/** @defgroup LL_AON_RF AON_RF
  * @brief AON RF LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_AON_RF_H_
#define __GR55XX_LL_AON_RF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

/** @defgroup AON_RF_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
  * @brief  Enable the test mux 0 tri state
  *
  *  Register|BitsName
  *  --------|--------
  *  RF5 | TEST_MUX_EN
  *
  */
__STATIC_INLINE void ll_aon_rf_enable_test_mux(void)
{
    SET_BITS(AON_RF->RF5, AON_RF_RF5_TEST_MUX_EN);
}

/**
  * @brief  Disable the test mux 0 tri state
  *
  *  Register|BitsName
  *  --------|--------
  *  RF5 | TEST_MUX_EN
  *
  */
__STATIC_INLINE void ll_aon_rf_disable_test_mux(void)
{
    CLEAR_BITS(AON_RF->RF5, AON_RF_RF5_TEST_MUX_EN);
}

/**
  * @brief  Enable the cpll reset
  *
  *  Register|BitsName
  *  --------|--------
  *  RF6 | CPLL_CP_EN
  *
  */
__STATIC_INLINE void ll_aon_rf_enable_cpll_cp_reset(void)
{
    SET_BITS(AON_RF->RF6, AON_RF_RF6_CPLL_CP_EN);
}

/**
  * @brief  Disable the cpll reset
  *
  *  Register|BitsName
  *  --------|--------
  *  RF6 | CPLL_CP_EN
  *
  */
__STATIC_INLINE void ll_aon_rf_disable_cpll_cp_reset(void)
{
    CLEAR_BITS(AON_RF->RF6, AON_RF_RF6_CPLL_CP_EN);
}

/**
  * @brief  Enable the cpll drift detection
  *
  *  Register|BitsName
  *  --------|--------
  *  RF6 | PLL_LOCK_DET_EN
  *
  */
__STATIC_INLINE void ll_aon_rf_enable_cpll_drift_detection(void)
{
    SET_BITS(AON_RF->RF6, AON_RF_RF6_PLL_LOCK_DET_EN);
}

/**
  * @brief  Disable the cpll drift detection
  *
  *  Register|BitsName
  *  --------|--------
  *  RF6 | PLL_LOCK_DET_EN
  *
  */
__STATIC_INLINE void ll_aon_rf_disable_cpll_drift_detection(void)
{
    CLEAR_BITS(AON_RF->RF6, AON_RF_RF6_PLL_LOCK_DET_EN);
}

/**
  * @brief  Set the cpll m div,Pre division before CP (00- no division / 11 divide by 16).
  *
  *  Register|BitsName
  *  --------|--------
  *  RF6 | CPLL_M_DIV_CTRL
  *
  * @param value: The cpll m div value.
  *
  */
__STATIC_INLINE void ll_aon_rf_set_cpll_m_div(uint32_t value)
{
    MODIFY_REG(AON_RF->RF6, AON_RF_RF6_CPLL_M_DIV_CTRL, (value << AON_RF_RF6_CPLL_M_DIV_CTRL_Pos));
}

/**
  * @brief  Set the cpll drift detection.
  *
  *  Register|BitsName
  *  --------|--------
  *  RF7 | L_H_THRESHOLD
  *
  * @param l_threshold: L Threshold.
  * @param h_threshold: H Threshold.
  *
  */
__STATIC_INLINE void ll_aon_rf_set_threshold(uint32_t l_threshold, uint32_t h_threshold)
{
    MODIFY_REG(AON_RF->RF7, AON_RF_RF7_L_H_THRESHOLD, (((h_threshold << 2) | l_threshold) << AON_RF_RF7_L_H_THRESHOLD_Pos));
}

/**
  * @brief  Set the cpll drift CPLL VCO KVCO control.
  *
  *  Register|BitsName
  *  --------|--------
  *  RF7 | cpll_kvco_dig_ctrl_2_0
  *
  * @param code
  *
  */
__STATIC_INLINE void ll_aon_rf_set_cpll_kvco_ctrl(uint32_t code)
{
    MODIFY_REG(AON_RF->RF7, AON_RF_RF7_CPLL_KVOC_CTRL, (code << AON_RF_RF7_CPLL_KVOC_CTRL_Pos));
}
/**
  * @brief  Set XO core current programmability.
  *
  *  Register|BitsName
  *  --------|--------
  *  RF8 | XO_IBIAS_CTRL_4_0
  *
  * @param hi_value
  * @param lo_value
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_rf_set_xo_ibias(uint32_t hi_value,uint32_t lo_value)
{
    MODIFY_REG(AON_RF->RF_XO_BIAS_VAL, AON_RF_XO_BIAS_HI, (hi_value << AON_RF_XO_BIAS_HI_Pos));
    MODIFY_REG(AON_RF->RF_XO_BIAS_VAL, AON_RF_XO_BIAS_LO, (lo_value << AON_RF_XO_BIAS_LO_Pos));
}

/**
  * @brief  Set the RF_REG9
  *
  *  Register|BitsName
  *  --------|--------
  *  RF9 | ALL
  *
  * @param value: The RF_REG9 value.
  *
  */
__STATIC_INLINE void ll_aon_rf_set_rf_reg9(uint32_t value)
{
    WRITE_REG(AON_RF->RF9, value);
}

/**
  * @brief  Get the RF_REG9
  *
  *  Register|BitsName
  *  --------|--------
  *  RF9 | ALL
  *
  * @retval The RF_REG9 value.
  *
  */
__STATIC_INLINE uint32_t ll_aon_rf_get_rf_reg9(void)
{
    return READ_REG(AON_RF->RF9);
}


/**
  * @brief  Set the xo cap value,cload programmability from 50fF to 26pF on each side.
  *
  *  Register|BitsName
  *  --------|--------
  *  RF9 | XO_CAP
  *
  * @param hi_value: The xo cap value.
  * @param lo_value: The xo cap value.
  */
__STATIC_INLINE void ll_aon_rf_set_xo_cap(uint32_t hi_value,uint32_t lo_value)
{
    MODIFY_REG(AON_RF->RF_XO_BIAS_VAL, AON_RF_XO_CAP_HI, (hi_value << AON_RF_XO_CAP_HI_Pos));
    MODIFY_REG(AON_RF->RF_XO_BIAS_VAL, AON_RF_XO_CAP_LO, (lo_value << AON_RF_XO_CAP_LO_Pos));
}

/**
  * @brief  Get the cpll crscde
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_RD_REG_0 | CPLL_CRSCDE
  *
  * @retval cpll crscde.
  *
  */
__STATIC_INLINE uint32_t ll_aon_rf_get_cpll_crscde(void)
{
    return (READ_BITS(AON_RF->RF_RD_REG_0, AON_RF_RF_RD_REG_0_CPLL_CRSCDE) >> AON_RF_RF_RD_REG_0_CPLL_CRSCDE_Pos);
}
/**
  * @brief  Set SU enable.
  *
  *  Register|BitsName
  *  --------|--------
  *  RF8     | SU_ENABLE
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_rf_set_su_enable(void)
{
    SET_BITS(AON_RF->RF8, AON_RF_RF8_SU_EN);
}
/**
  * @brief  Set cpll drift irq enable.
  *
  *  Register|BitsName
  *  --------|--------
  *  CPLL_IRQ_CFG     | MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_EN
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_mcu_set_cpll_drift_irq_enable(void)
{
    SET_BITS(MCU_SUB->CPLL_IRQ_CFG, MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_EN);
}

/**
  * @brief  Set cpll drift irq disable.
  *
  *  Register|BitsName
  *  --------|--------
  *  CPLL_IRQ_CFG     | MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_EN
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_mcu_set_cpll_drift_irq_disable(void)
{
    CLEAR_BITS(MCU_SUB->CPLL_IRQ_CFG, MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_EN);
}

/**
  * @brief  clear cpll drift irq.
  *
  *  Register|BitsName
  *  --------|--------
  *  CPLL_IRQ_CFG     | cpll_drift_irq_clr
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_mcu_clear_cpll_drift_irq(void)
{
    SET_BITS(MCU_SUB->CPLL_IRQ_CFG, MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_CLR);
}

/** @} */
#endif

/** @} */
/** @} */
/** @} */


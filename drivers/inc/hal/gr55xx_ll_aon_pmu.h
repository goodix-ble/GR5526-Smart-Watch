/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_aon_pmu.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PMU LL library.
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

/** @defgroup LL_PMU AON_PMU
  * @brief PMU LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_PMU_H_
#define __GR55XX_LL_PMU_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

/** @defgroup LL_PMU_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
  * @brief  Enable the RTC
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_0 | RTC_EN
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_rtc(void)
{
    SET_BITS(AON_PMU->RF_REG_0, AON_PMU_RF_REG_0_RTC_EN);
}
/**
  * @brief  Enable the RTC
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_0 | RTC_EN
  *           | CGM_MODE
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_rtc_cgm(void)
{
    SET_BITS(AON_PMU->RF_REG_0, AON_PMU_RF_REG_0_RTC_EN);
    CLEAR_BITS(AON_PMU->RF_REG_0, AON_PMU_RF_REG_0_RTC_EN_BGM);
}

/**
  * @brief  Disable the RTC
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_0 | RTC_EN
  *
  */
__STATIC_INLINE void ll_aon_pmu_disable_rtc(void)
{
    CLEAR_BITS(AON_PMU->RF_REG_0, AON_PMU_RF_REG_0_RTC_EN);
}

/**
  * @brief  Set RTC GM
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_0 | EN
  *
  * @param value: The rtc gm value.
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_rtc_gm(uint32_t value)
{
    MODIFY_REG(AON_PMU->RF_REG_0, AON_PMU_RF_REG_0_RTC_GM, (value << AON_PMU_RF_REG_0_RTC_GM_Pos));
}

/**
  * @brief  Set lv,default is set to 1.8V,LSB = 8.5mv
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_0 | EN
  *
  * @param value: The io ldo vout value.
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_io_ldo_vout(uint32_t value)
{
    MODIFY_REG(AON_PMU->RF_REG_0, AON_PMU_RF_REG_0_IO_LDO_REG1, (value << AON_PMU_RF_REG_0_IO_LDO_REG1_Pos));
}

/**
  * @brief  Set retention level
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_0 | ctrl_ret
  *
  * @param value: The retention level value.
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_retention_level(uint32_t value)
{
    MODIFY_REG(AON_PMU->RF_REG_0, AON_PMU_RF_REG_0_CTRL_RET, (value << AON_PMU_RF_REG_0_CTRL_RET_Pos));
}

/**
  * @brief  Get retention level
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_0 | ctrl_ret
  *
  * @retval The current retention level.
  *
  */
__STATIC_INLINE uint32_t ll_aon_pmu_get_retention_level(void)
{
    return (READ_BITS(AON_PMU->RF_REG_0, AON_PMU_RF_REG_0_CTRL_RET) >> AON_PMU_RF_REG_0_CTRL_RET_Pos);
}

/**
  * @brief  Set dcdc the ton value
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_1 | TON
  *
  * @param value: The dcdc ton value.
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_dcdc_ton(uint32_t value)
{
    MODIFY_REG(AON_PMU->RF_REG_1, AON_PMU_RF_REG_1_TON, (value << AON_PMU_RF_REG_1_TON_Pos));
}

/**
  * @brief  Get dcdc the ton value
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_1 | TON
  *
  * @retval The dcdc ton value.
  *
  */
__STATIC_INLINE uint32_t ll_aon_pmu_get_dcdc_ton(void)
{
    return (READ_BITS(AON_PMU->RF_REG_1, AON_PMU_RF_REG_1_TON) >> AON_PMU_RF_REG_1_TON_Pos);
}

/**
  * @brief  Set dcdc ref_cntrl_b_lv_3_0,vreg defaulted to 1.1V.
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_4 | AON_PMU_RF_REG_4_DCDC_VREF
  *
  * @param value: the dcdc vreg value.
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_dcdc_vreg(uint32_t value)
{
    MODIFY_REG(AON_PMU->RF_REG_4, AON_PMU_RF_REG_4_DCDC_VREF, (value << AON_PMU_RF_REG_4_DCDC_VREF_Pos));
}

/**
  * @brief  Get dcdc vreg
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_4 | AON_PMU_RF_REG_4_DCDC_VREF
  *
  * @retval The dcdc vreg value.
  *
  */
__STATIC_INLINE uint32_t ll_aon_pmu_get_dcdc_vreg(void)
{
    return (READ_BITS(AON_PMU->RF_REG_4, AON_PMU_RF_REG_4_DCDC_VREF) >> AON_PMU_RF_REG_4_DCDC_VREF_Pos);
}

/**
  * @brief  Set dcdc reg_sel_aon_pmu_dcore_vref, default from AON.
  *
  *  Register|BitsName
  *  --------|--------
  *  PMU_DCORE_VREF | REG_SEL_AON_PMU_DCORE_VREF
  *
  * @param sel: the dcore vref source control.
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_dcore_sel(uint8_t sel)
{
    MODIFY_REG(AON_PMU->PMU_DCORE_VREF, AON_PMU_DCORE_VREF_REG_SEL, (sel << AON_PMU_DCORE_VREF_REG_SEL_Pos));
}


/**
  * @brief  Enable the io ldo bypass
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_3 | BYPASS_EN
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_io_ldo_bypass(void)
{
    SET_BITS(AON_PMU->RF_REG_3, AON_PMU_RF_REG_3_IO_LDO_BYPASS);
}

/**
  * @brief  Disable the io ldo bypass
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_3 | BYPASS_EN
  *
  */
__STATIC_INLINE void ll_aon_pmu_disable_io_ldo_bypass(void)
{
    CLEAR_BITS(AON_PMU->RF_REG_3, AON_PMU_RF_REG_3_IO_LDO_BYPASS);
}


/**
  * @brief  Enable the dig ldo bleed
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_4 | EN
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_bleed(void)
{
    SET_BITS(AON_PMU->RF_REG_4, AON_PMU_RF_REG_4_DIG_LDO_BLEED_EN);
}

/**
  * @brief  Disable the dig ldo bleed
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_4 | EN
  *
  */
__STATIC_INLINE void ll_aon_pmu_disable_bleed(void)
{
    CLEAR_BITS(AON_PMU->RF_REG_4, AON_PMU_RF_REG_4_DIG_LDO_BLEED_EN);
}

/**
  * @brief  Set dig ldo out
  *
  *  Register|BitsName
  *  --------|--------
  *  PMU_DCORE_VREF | DIG_LDO_OUT
  *
  * @param value: The dig ldo out value.
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_dig_ldo_out(uint32_t value)
{
    MODIFY_REG(AON_PMU->PMU_DCORE_VREF, AON_PMU_DCORE_VREF_REG_DIG_OUT, (value << AON_PMU_DCORE_VREF_REG_DIG_OUT_Pos));
}

/**
  * @brief  Get dig ldo out value
  *
  *  Register|BitsName
  *  --------|--------
  *  PMU_DCORE_VREF | DIG_LDO_OUT
  *
  *
  */
__STATIC_INLINE uint32_t ll_aon_pmu_get_dig_ldo_out(void)
{
    return (READ_BITS(AON_PMU->PMU_DCORE_VREF, AON_PMU_DCORE_VREF_REG_DIG_OUT) >> AON_PMU_DCORE_VREF_REG_DIG_OUT_Pos);
}

/**
  * @brief  Enable the dig ldo bypass
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_4 | BYPASS_EN
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_dig_ldo_bypass(void)
{
    SET_BITS(AON_PMU->RF_REG_4, AON_PMU_RF_REG_4_DIG_LDO_BYPASS_EN);
}

/**
  * @brief  Disable the dig ldo bypass
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_4 | BYPASS_EN
  *
  */
__STATIC_INLINE void ll_aon_pmu_disable_dig_ldo_bypass(void)
{
    CLEAR_BITS(AON_PMU->RF_REG_4, AON_PMU_RF_REG_4_DIG_LDO_BYPASS_EN);
}

/**
  * @brief  Set the dig ldo bypass
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_4 | BYPASS_EN
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_dig_ldo_bypass(bool enable)
{
    MODIFY_REG(AON_PMU->RF_REG_4, AON_PMU_RF_REG_4_DIG_LDO_BYPASS_EN, (enable << AON_PMU_RF_REG_4_DIG_LDO_BYPASS_EN_Pos));
}

/**
  * @brief  Get the dig ldo bypass
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_4 | BYPASS_EN
  *
  * @retval The dig ldo bypass enable value.
  *
  */
__STATIC_INLINE uint32_t ll_aon_pmu_get_dig_ldo_bypass(void)
{
    return (READ_BITS(AON_PMU->RF_REG_4, AON_PMU_RF_REG_4_DIG_LDO_BYPASS_EN) >> AON_PMU_RF_REG_4_DIG_LDO_BYPASS_EN_Pos);
}

/**
  * @brief  Set clk period
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_4 | CLK_PERIOD
  *
  * @param value: The clock period value.
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_clk_period(uint32_t value)
{
    MODIFY_REG(AON_PMU->RF_REG_4, AON_PMU_RF_REG_4_CLK_PERIOD, (value << AON_PMU_RF_REG_4_CLK_PERIOD_Pos));
}

/**
  * @brief  Get clk period
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_4 | CLK_PERIOD
  *
  * @retval The clock period value.
  *
  */
__STATIC_INLINE uint32_t ll_aon_pmu_get_clk_period(void)
{
    return (READ_BITS(AON_PMU->RF_REG_4, AON_PMU_RF_REG_4_CLK_PERIOD) >> AON_PMU_RF_REG_4_CLK_PERIOD_Pos);
}

/**
  * @brief Enables clock injection from XO to ring oscillator.
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_1 | EN_INJ_ON
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_clk_inject(void)
{
    SET_BITS(AON_PMU->RF_REG_1, AON_PMU_RF_REG_1_EN_INJ_ON);
}

/**
  * @brief Disables clock injection from XO to ring oscillator.
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_1 | EN_INJ_ON
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_disable_clk_inject(void)
{
    CLEAR_BITS(AON_PMU->RF_REG_1, AON_PMU_RF_REG_1_EN_INJ_ON);
}

/**
  * @brief  Enable the dcdc ton startup
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | TON_STARTUP
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_ton_startup_overide(void)
{
    SET_BITS(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_TON_STARTUP);
}

/**
  * @brief Enable clock detection override
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | CLK_DET_OVR
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_clk_det_ovr(void)
{
    SET_BITS(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_CLK_DET_OVR);
}

/**
  * @brief  Disable clock detection override
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | CLK_DET_OVR
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_disable_clk_det_ovr(void)
{
    CLEAR_BITS(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_CLK_DET_OVR);
}


/**
  * @brief Enable clock detection override source as XO
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | CLK_DET_OVR_SRC
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_clk_det_ovr_src_xo(void)
{
    SET_BITS(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_CLK_DET_OVR_SRC);
}

/**
  * @brief Disable clock detection override source XO ---- means set as RING
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | CLK_DET_OVR_SRC
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_disable_clk_det_ovr_src_xo(void)
{
    CLEAR_BITS(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_CLK_DET_OVR_SRC);
}

/**
  * @brief  Set clock detection override source
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | CLK_DET_OVR_SRC
  *
  * @param value: the clock detection override source value.
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_clk_det_ovr_src(uint32_t value)
{
    MODIFY_REG(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_CLK_DET_OVR_SRC, (value << AON_PMU_DCDC_LDO_REG0_CLK_DET_OVR_SRC_Pos));
}


/**
  * @brief Enable use_xo
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | USE_XO
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_use_xo(void)
{
    SET_BITS(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_USE_XO);
}

/**
  * @brief  Disable use_xo
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | USE_XO
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_disable_use_xo(void)
{
    CLEAR_BITS(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_USE_XO);
}


/**
  * @brief  Enable the digital io ldo.
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | EN_DIG_IO_LDO
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_dig_io_ldo(void)
{
    SET_BITS(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_EN_DIG_IO_LDO);
}

/**
  * @brief  Disable the tristate ldo.
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | TRISTATE_LD
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_disable_tristate_ldo(void)
{
    CLEAR_BITS(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_TRISTATE_LDO);
}

/**
  * @brief  Disable the tristate analog ldo.
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | TRISTATE_ANA_IO_LDO
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_tristate_ana_io_ldo(void)
{
    SET_BITS(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_TRISTATE_ANA_IO_LDO);
}

/**
  * @brief  Set ldo control_override.
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | REG0_LDO_CTRL_OV
  *
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_ldo_control_override(void)
{
    SET_BITS(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_LDO_CTRL_OV);
}

/**
  * @brief  Set boost step.
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | BOOST_STEP
  *
  * @param value: The boost step value.
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_boost_step(uint32_t value)
{
    MODIFY_REG(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_BOOST_STEP, (value << AON_PMU_DCDC_LDO_REG0_BOOST_STEP_Pos));
}

/**
  * @brief  Set digital io ldo divider.
  *
  *  Register|BitsName
  *  --------|--------
  *  DCDC_LDO_REG_0 | CLK_DIV_SEL
  *
  * @param value: The dig ldo div value.
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_dig_ldo_div(uint32_t value)
{
    MODIFY_REG(AON_PMU->DCDC_LDO_REG_0, AON_PMU_DCDC_LDO_REG0_CLK_DIV_SEL, (value << AON_PMU_DCDC_LDO_REG0_CLK_DIV_SEL_Pos));
}

/**
  * @brief  Set the rtc cur cap
  *
  *  Register|BitsName
  *  --------|--------
  *  RC_RTC_REG_0 | RTC_CAP
  *
  * @param value: The rtc current cap value.
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_rtc_cs(uint32_t value)
{
    MODIFY_REG(AON_PMU->RC_RTC_REG_0, AON_PMU_RC_RTC_REG0_RTC_CS, (value << AON_PMU_RC_RTC_REG0_RTC_CS_Pos));
}
/**
  * @brief  Set the rtc on MSIO A6/7 en pad sw
  *
  *  Register|BitsName
  *  --------|--------
  *  RC_RTC_REG_0 | EN_PAD_SW
  *
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_pad_sw(void)
{
    SET_BITS(AON_PMU->RC_RTC_REG_0, AON_PMU_RC_RTC_REG0_EN_PAD_SW);
}


/**
  * @brief  Set clock detection option
  *
  *  Register|BitsName
  *  --------|--------
  *  RC_RTC_REG_0 | CLK_DET_OPT
  *
  * @param value: clock detection option value, 0: use clk_det, 1: use glitch free MUX.
  * @retval None
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_clk_det_opt(uint32_t value)
{
    MODIFY_REG(AON_PMU->RC_RTC_REG_0, AON_PMU_RC_RTC_REG0_CLK_DET_OPT, (value << AON_PMU_RC_RTC_REG0_CLK_DET_OPT_Pos));
}

/**
  * @brief  Set the rtc cur cap
  *
  *  Register|BitsName
  *  --------|--------
  *  RC_RTC_REG_0 | RTC_CAP
  *
  * @param value: The rtc current cap value.
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_rtc_cap(uint32_t value)
{
    MODIFY_REG(AON_PMU->RC_RTC_REG_0, AON_PMU_RC_RTC_REG0_RTC_CAP, (value << AON_PMU_RC_RTC_REG0_RTC_CAP_Pos));
}

/**
  * @brief  Get the rtc cur cap
  *
  *  Register|BitsName
  *  --------|--------
  *  RC_RTC_REG_0 | RTC_CAP
  *
  * @retval The rtc current cap value.
  *
  */
__STATIC_INLINE uint32_t ll_aon_pmu_get_rtc_cap(void)
{
    return (READ_BITS(AON_PMU->RC_RTC_REG_0, AON_PMU_RC_RTC_REG0_RTC_CAP) >> AON_PMU_RC_RTC_REG0_RTC_CAP_Pos);
}

/**
  * @brief  Enable the RCOSC
  *
  *  Register|BitsName
  *  --------|--------
  *  RC_RTC_REG_0 | RCOSC
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_rcosc(void)
{
    SET_BITS(AON_PMU->RC_RTC_REG_0, AON_PMU_RC_RTC_REG0_RCOSC);
}

/**
  * @brief  Disable the RCOSC
  *
  *  Register|BitsName
  *  --------|--------
  *  RC_RTC_REG_0 | RCOSC
  *
  */
__STATIC_INLINE void ll_aon_pmu_disable_rcosc(void)
{
    CLEAR_BITS(AON_PMU->RC_RTC_REG_0, AON_PMU_RC_RTC_REG0_RCOSC);
}

/**
  * @brief  enable the ret ldo
  *
  *  Register|BitsName
  *  --------|--------
  *  RET_LDO_REG | RET_LDO_EN
  *
  */
__STATIC_INLINE void ll_aon_pmu_enable_ret_ldo(void)
{
    SET_BITS(AON_PMU->RET_LDO, AON_PMU_RET_LDO_EN);
}

/**
  * @brief  modify ret ldo ctrl level
  *
  *  Register|BitsName
  *  --------|--------
  *  RET_LDO_REG | RET_LDO_CTRL_5_1
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_ret_ldo_ctrl_lvl(uint32_t value)
{
    MODIFY_REG(AON_PMU->RET_LDO, AON_PMU_RET_LDO_OUT, (value << AON_PMU_RET_LDO_OUT_Pos));
}

/**
  * @brief  modify lpd active
  *
  *  Register|BitsName
  *  --------|--------
  *  PMU_LPD_CFG | LPD_VAON_ACTIVE
  *
  */
__STATIC_FORCEINLINE void ll_aon_pmu_set_lpd_active(uint32_t value)
{
    MODIFY_REG(AON_PMU->PMU_LPD_CFG, AON_PMU_LPD_VAON_ACTIVE, (value << AON_PMU_LPD_VAON_ACTIVE_Pos));
}

/**
  * @brief  Get lpd active value
  *
  *  Register|BitsName
  *  --------|--------
  *  PMU_LPD_CFG | LPD_VAON_ACTIVE
  *
  * @retval The current lpd active value.
  *
  */
__STATIC_INLINE uint32_t ll_aon_pmu_get_lpd_active(void)
{
    return (READ_BITS(AON_PMU->PMU_LPD_CFG, AON_PMU_LPD_VAON_ACTIVE) >> AON_PMU_LPD_VAON_ACTIVE_Pos);
}

/**
  * @brief  modify lpd sleep
  *
  *  Register|BitsName
  *  --------|--------
  *  PMU_LPD_CFG | LPD_VAON_SLEEP
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_lpd_sleep(uint32_t value)
{
    MODIFY_REG(AON_PMU->PMU_LPD_CFG, AON_PMU_LPD_VAON_SLEEP, (value << AON_PMU_LPD_VAON_SLEEP_Pos));
}
/**
  * @brief  modify ton on
  *
  *  Register|BitsName
  *  --------|--------
  *  PMU_TON_CFG | AON_PMU_TON_CTRL_ON
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_tx_ton_val(uint32_t value)
{
    MODIFY_REG(AON_PMU->PMU_TON_CFG, AON_PMU_TON_CTRL_ON, (value << AON_PMU_TON_CTRL_ON_Pos));
}
/**
  * @brief  modify ton off
  *
  *  Register|BitsName
  *  --------|--------
  *  PMU_TON_CFG | AON_PMU_TON_CTRL_OFF
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_non_tx_ton_val(uint32_t value)
{
    MODIFY_REG(AON_PMU->PMU_TON_CFG, AON_PMU_TON_CTRL_OFF, (value << AON_PMU_TON_CTRL_OFF_Pos));
}
/**
  * @brief  set rng freq
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG0 | AON_PMU_RF_REG_0_RNG_FREQ_CONT
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_rng_req(uint32_t value)
{
    MODIFY_REG(AON_PMU->RF_REG_0, AON_PMU_RF_REG_0_RNG_FREQ_CONT, (value << AON_PMU_RF_REG_0_RNG_FREQ_CONT_Pos));
}
/**
  * @brief  set rng freq
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG0 | AON_PMU_RF_REG_0_RNG_FREQ_CONT
  *
  */
__STATIC_INLINE void ll_aon_pmu_set_rng_freq_bump_enable(void)
{
    SET_BITS(AON_PMU->RF_REG_0, AON_PMU_RF_REG_0_RNG_FREQ_BUMP);
}
/**
  * @brief  Enable short aon digcore
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG2 | SHORT_AON_DIGCORE
  *
  * @retval None
  *
  */
__STATIC_FORCEINLINE void ll_aon_pmu_enable_short_aon_digcore(void)
{
    SET_BITS(AON_PMU->RF_REG_2, AON_PMU_RF_REG_2_SHORT_AON_DIGCORE);
}

/**
  * @brief  Disable short aon digcore
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG2 | SHORT_AON_DIGCORE
  *
  * @retval None
  *
  */
__STATIC_FORCEINLINE void ll_aon_pmu_disable_short_aon_digcore(void)
{
    CLEAR_BITS(AON_PMU->RF_REG_2, AON_PMU_RF_REG_2_SHORT_AON_DIGCORE);
}
/** @} */

#endif

/** @} */

/** @} */

/** @} */

/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_pdm.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PDM LL library.
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

/** @defgroup LL_PDM PDM
  * @brief PDM LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_PDM_H__
#define __GR55xx_LL_PDM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (PDM)


/** @defgroup PDM_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup PDM_LL_ES_INIT PDM Exported init structures
  * @{
  */
/**
  * @brief LL PDM init Structure definition
  */
typedef struct _ll_pdm_init
{
    uint32_t mode;              /**< Specifies the sample rate.must be a value of PDM_LL_MODE */

    uint32_t gain_l;            /**< Specifies the gain of left. must be a value between Min 0x0 ~ Max 0x3FFF. */

    uint32_t gain_r;            /**< Specifies the gain of right.must be a value between Min 0x0 ~ Max 0x3FFF. */

    uint32_t sample_rate;        /**< Specifies the sample rate.must be a value of PDM_LL_SAMPLE_RATE */

} ll_pdm_init_t;

/** @} */

/** @} */

/**
  * @defgroup  PDM_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PDM_LL_Exported_Constants PDM Exported Constants
  * @{
  */

/** @defgroup LL_PDM_SAMPLE_RATE PDM Sample Rate
  * @{
  */

/**
 * @brief PDM sample rate.
 */
#define LL_PDM_SAMPLE_RATE_15_625K              PDM_CLK_SAMPLE_RATE_15_625K /**< PDM sample rate 15.625K */
#define LL_PDM_SAMPLE_RATE_16K                  PDM_CLK_SAMPLE_RATE_16K /**< PDM sample rate 16K */
#define LL_PDM_SAMPLE_RATE_8K                   PDM_CLK_SAMPLE_RATE_8K /**< PDM sample rate 8K */
/** @} */

/** @defgroup LL_PDM_MODE PDM Mode Set Defines
  * @{
  */

/**
 * @brief PDM operation mode.
 */
#define LL_PDM_MODE_LEFT                        ((uint32_t)0x00000000U) /**< PDM left mono */
#define LL_PDM_MODE_RIGHT                       ((uint32_t)0x00000001U) /**< PDM right mono */
#define LL_PDM_MODE_STEREO                      ((uint32_t)0x00000002U) /**< PDM stereo mono */
/** @} */

/** @defgroup LL_PDM_CLK_EN PDM CLK Set Defines
  * @{
  */
#define LL_PDM_CLK_ENABLE                       PDM_CLK_EN_ENABLE /**< PDM clk enable */
#define LL_PDM_CLK_DISABLE                      PDM_CLK_EN_DISABLE /**< PDM clk disable*/
/** @} */

/** @defgroup LL_PDM_RX_EN PDM RX Set Defines
  * @{
  */
#define LL_PDM_LEFT_RX_ENABLE                   PDM_EN_L_EN_RX_ENABLE /**< PDM left rx enable*/
#define LL_PDM_LEFT_RX_DISABLE                  PDM_EN_L_EN_RX_DISABLE /**< PDM left rx disable*/
#define LL_PDM_RIGHT_RX_ENABLE                  PDM_EN_R_EN_RX_ENABLE /**< PDM right rx enable*/
#define LL_PDM_RIGHT_RX_DISABLE                 PDM_EN_R_EN_RX_DISABLE /**< PDM right rx disable*/
/** @} */

/** @defgroup LL_PDM_SAMPLE_DMIC_EN PDM Sample Dmic Set Defines
  * @{
  */
#define LL_PDM_LEFT_SAMPLE_DMIC_ENABLE          PDM_EN_L_SMP_DMIC_ENABLE /**< PDM left sample dmic enable*/
#define LL_PDM_LEFT_SAMPLE_DMIC_DISABLE         PDM_EN_L_SMP_DMIC_DISABLE /**< PDM left sample dmic disable*/
#define LL_PDM_RIGHT_SAMPLE_DMIC_ENABLE         PDM_EN_R_SMP_DMIC_ENABLE /**< PDM right sample dmic enable*/
#define LL_PDM_RIGHT_SAMPLE_DMIC_DISABLE        PDM_EN_R_SMP_DMIC_DISABLE /**< PDM right sample dmic disable*/
/** @} */

/** @defgroup LL_PDM_STAGE0_7_EN PDM Stage Set Defines
  * @{
  */
#define LL_PDM_LEFT_STAGE0_ENABLE               PDM_EN_L_EN_STAGE0_ENABLE /**< PDM left stage0 enable*/
#define LL_PDM_LEFT_STAGE0_DISABLE              PDM_EN_L_EN_STAGE0_DISABLE /**< PDM left stage0 disable*/
#define LL_PDM_LEFT_STAGE1_ENABLE               PDM_EN_L_EN_STAGE1_ENABLE /**< PDM left stage1 enable*/
#define LL_PDM_LEFT_STAGE1_DISABLE              PDM_EN_L_EN_STAGE1_DISABLE /**< PDM left stage1 disable*/
#define LL_PDM_LEFT_STAGE2_ENABLE               PDM_EN_L_EN_STAGE2_ENABLE /**< PDM left stage2 enable*/
#define LL_PDM_LEFT_STAGE2_DISABLE              PDM_EN_L_EN_STAGE2_DISABLE /**< PDM left stage2 disable*/
#define LL_PDM_LEFT_STAGE3_ENABLE               PDM_EN_L_EN_STAGE3_ENABLE /**< PDM left stage3 enable*/
#define LL_PDM_LEFT_STAGE3_DISABLE              PDM_EN_L_EN_STAGE3_DISABLE /**< PDM left stage3 disable*/
#define LL_PDM_LEFT_STAGE4_ENABLE               PDM_EN_L_EN_STAGE4_ENABLE /**< PDM left stage4 enable*/
#define LL_PDM_LEFT_STAGE4_DISABLE              PDM_EN_L_EN_STAGE4_DISABLE /**< PDM left stage4 disable*/
#define LL_PDM_LEFT_STAGE5_ENABLE               PDM_EN_L_EN_STAGE5_ENABLE /**< PDM left stage5 enable*/
#define LL_PDM_LEFT_STAGE5_DISABLE              PDM_EN_L_EN_STAGE5_DISABLE /**< PDM left stage5 disable*/
#define LL_PDM_LEFT_STAGE6_ENABLE               PDM_EN_L_EN_STAGE6_ENABLE /**< PDM left stage6 enable*/
#define LL_PDM_LEFT_STAGE6_DISABLE              PDM_EN_L_EN_STAGE6_DISABLE /**< PDM left stage6 disable*/
#define LL_PDM_LEFT_STAGE7_ENABLE               PDM_EN_L_EN_STAGE7_ENABLE /**< PDM left stage7 enable*/
#define LL_PDM_LEFT_STAGE7_DISABLE              PDM_EN_L_EN_STAGE7_DISABLE /**< PDM left stage7 disable*/
#define LL_PDM_RIGHT_STAGE0_ENABLE              PDM_EN_R_EN_STAGE0_ENABLE /**< PDM right stage0 enable*/
#define LL_PDM_RIGHT_STAGE0_DISABLE             PDM_EN_R_EN_STAGE0_DISABLE /**< PDM right stage0 disable*/
#define LL_PDM_RIGHT_STAGE1_ENABLE              PDM_EN_R_EN_STAGE1_ENABLE /**< PDM right stage1 enable*/
#define LL_PDM_RIGHT_STAGE1_DISABLE             PDM_EN_R_EN_STAGE1_DISABLE /**< PDM right stage1 disable*/
#define LL_PDM_RIGHT_STAGE2_ENABLE              PDM_EN_R_EN_STAGE2_ENABLE /**< PDM right stage2 enable*/
#define LL_PDM_RIGHT_STAGE2_DISABLE             PDM_EN_R_EN_STAGE2_DISABLE /**< PDM right stage2 disable*/
#define LL_PDM_RIGHT_STAGE3_ENABLE              PDM_EN_R_EN_STAGE3_ENABLE /**< PDM right stage3 enable*/
#define LL_PDM_RIGHT_STAGE3_DISABLE             PDM_EN_R_EN_STAGE3_DISABLE /**< PDM right stage3 disable*/
#define LL_PDM_RIGHT_STAGE4_ENABLE              PDM_EN_R_EN_STAGE4_ENABLE /**< PDM right stage4 enable*/
#define LL_PDM_RIGHT_STAGE4_DISABLE             PDM_EN_R_EN_STAGE4_DISABLE /**< PDM right stage4 disable*/
#define LL_PDM_RIGHT_STAGE5_ENABLE              PDM_EN_R_EN_STAGE5_ENABLE /**< PDM right stage5 enable*/
#define LL_PDM_RIGHT_STAGE5_DISABLE             PDM_EN_R_EN_STAGE5_DISABLE /**< PDM right stage5 disable*/
#define LL_PDM_RIGHT_STAGE6_ENABLE              PDM_EN_R_EN_STAGE6_ENABLE /**< PDM right stage6 enable*/
#define LL_PDM_RIGHT_STAGE6_DISABLE             PDM_EN_R_EN_STAGE6_DISABLE /**< PDM right stage6 disable*/
#define LL_PDM_RIGHT_STAGE7_ENABLE              PDM_EN_R_EN_STAGE7_ENABLE /**< PDM right stage7 enable*/
#define LL_PDM_RIGHT_STAGE7_DISABLE             PDM_EN_R_EN_STAGE7_DISABLE /**< PDM right stage7 disable*/
/** @} */

/** @defgroup LL_PDM_HPF_EN PDM HPF Set Defines
  * @{
  */
#define LL_PDM_LEFT_HPF_ENABLE                  PDM_EN_L_EN_HPF_ENABLE /**< PDM left hpf enable*/
#define LL_PDM_LEFT_HPF_DISABLE                 PDM_EN_L_EN_HPF_DISABLE /**< PDM left hpf disable*/
#define LL_PDM_RIGHT_HPF_ENABLE                 PDM_EN_R_EN_HPF_ENABLE /**< PDM right hpf enable*/
#define LL_PDM_RIGHT_HPF_DISABLE                PDM_EN_R_EN_HPF_DISABLE /**< PDM right hpf disable*/
/** @} */

/** @defgroup LL_PDM_HPF_BYPASS_EN PDM HPF Bypass Set Defines
  * @{
  */
#define LL_PDM_LEFT_HPF_BYPASS_ENABLE           PDM_HPF_CFG_L_BYPASS_ENABLE /**< PDM left hpf bypass enable*/
#define LL_PDM_LEFT_HPF_BYPASS_DISABLE          PDM_HPF_CFG_L_BYPASS_DISABLE /**< PDM left hpf bypass disable*/
#define LL_PDM_RIGHT_HPF_BYPASS_ENABLE          PDM_HPF_CFG_R_BYPASS_ENABLE /**< PDM right hpf bypass enable*/
#define LL_PDM_RIGHT_HPF_BYPASS_DISABLE         PDM_HPF_CFG_R_BYPASS_DISABLE /**< PDM right hpf bypass disable*/
/** @} */

/** @defgroup LL_PDM_HPF_CORNER PDM HPF Corner
  * @{
  */
#define LL_PDM_LEFT_HPF_CORNER_0_25             PDM_HPF_CFG_L_CORNER_0_25 /**< PDM right hpf corner 0.25*/
#define LL_PDM_LEFT_HPF_CORNER_1                PDM_HPF_CFG_L_CORNER_1 /**< PDM right hpf corner 1*/
#define LL_PDM_LEFT_HPF_CORNER_4                PDM_HPF_CFG_L_CORNER_4 /**< PDM right hpf corner 4*/
#define LL_PDM_LEFT_HPF_CORNER_16               PDM_HPF_CFG_L_CORNER_16 /**< PDM right hpf corner 16*/
#define LL_PDM_RIGHT_HPF_CORNER_0_25            PDM_HPF_CFG_R_CORNER_0_25 /**< PDM right hpf corner 0.25*/
#define LL_PDM_RIGHT_HPF_CORNER_1               PDM_HPF_CFG_R_CORNER_1 /**< PDM right hpf corner 1*/
#define LL_PDM_RIGHT_HPF_CORNER_4               PDM_HPF_CFG_R_CORNER_4 /**< PDM right hpf corner 4*/
#define LL_PDM_RIGHT_HPF_CORNER_16              PDM_HPF_CFG_R_CORNER_16 /**< PDM right hpf corner 16*/
/** @} */

/** @defgroup LL_PDM_HPF_FREEZE_EN PDM HPF Freeze
  * @{
  */
#define LL_PDM_LEFT_HPF_FREEZE_ENABLE           PDM_HPF_CFG_L_FREEZE_ENABLE /**< PDM left hpf freeze enable*/
#define LL_PDM_LEFT_HPF_FREEZE_DISABLE          PDM_HPF_CFG_L_FREEZE_DISABLE /**< PDM left hpf freeze disable*/
#define LL_PDM_RIGHT_HPF_FREEZE_ENABLE          PDM_HPF_CFG_R_FREEZE_ENABLE /**< PDM right hpf freeze enable*/
#define LL_PDM_RIGHT_HPF_FREEZE_DISABLE         PDM_HPF_CFG_R_FREEZE_DISABLE /**< PDM right hpf freeze disable*/
/** @} */

/** @} */
/* Exported macro ------------------------------------------------------------*/
/** @defgroup PDM_LL_Exported_Macros PDM Exported Macros
  * @{
  */

/** @defgroup PDM_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in PDM register
  * @param  __instance__ PDM instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_PDM_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in PDM register
  * @param  __instance__ PDM instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_PDM_ReadReg(__instance__, __REG__) READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PDM_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup PDM_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable PDM Module CLK.
  * @note This function is used to enable the PDM Module CLK.
  *
  *  Register|BitsName
  *  --------|--------
  *  PDM_CLK | EN
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_clk(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->CLK, PDM_CLK_EN, LL_PDM_CLK_ENABLE);
}

/**
  * @brief  Disable PDM Module CLK.
  * @note This function is used to enable the PDM Module CLK.
  *
  *  Register|BitsName
  *  --------|--------
  *  PDM_CLK | EN
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_clk(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->CLK, PDM_CLK_EN, LL_PDM_CLK_DISABLE);
}

/**
  * @brief  Disable PDM Module CLK.
  * @note This function is used to check the PDM Module CLK is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  PDM_CLK | EN
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_clk(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->CLK, PDM_CLK_EN) == LL_PDM_CLK_ENABLE);
}

/**
  * @brief  Set PDM sample rate
  * @note This function is Configures the sample rate for PDM
  *
  *  Register|BitsName
  *  --------|--------
  *  PDM_CLK | SMAPLE_RATE
  *
  * @param  PDMx PDM instance
  * @param  sample_rate This parameter can be one of the following values:
  *         @arg @ref LL_PDM_SAMPLE_RATE_15_625K
  *         @arg @ref LL_PDM_SAMPLE_RATE_16K
  *         @arg @ref LL_PDM_SAMPLE_RATE_8K
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_sample_rate(pdm_regs_t *PDMx, uint32_t sample_rate)
{
    MODIFY_REG(PDMx->CLK, PDM_CLK_SAMPLE_RATE, sample_rate);
}

/**
  * @brief  Get PDM sample rate
  *
  *  Register|BitsName
  *  --------|--------
  *  PDM_CLK | SMAPLE_RATE
  *
  * @param  PDMx PDM instance
  * @retval  one of the following values:
  *         @arg @ref LL_PDM_SAMPLE_RATE_15_625K
  *         @arg @ref LL_PDM_SAMPLE_RATE_16K
  *         @arg @ref LL_PDM_SAMPLE_RATE_8K
  */
__STATIC_INLINE uint32_t ll_pdm_get_sample_rate(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->CLK, PDM_CLK_SAMPLE_RATE));
}

/**
  * @brief  Set PDM posedge en pulse cfg
  * @note This function is Configures how many 16MHz clock cycles after rising edge of PDM_CLK to sample data (rising edge capture)
  *
  *  Register|BitsName
  *  --------|--------
  *  PDM_CLK_DIV | TARGET
  *
  * @param  PDMx PDM instance
  * @param  target This parameter can be one of the following values:
  *         Min_Data = 0 and Max_Data = 0xF
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_posedge_en_pulse_cfg(pdm_regs_t *PDMx, uint32_t target)
{
    MODIFY_REG(PDMx->CLK_DIV, PDM_CLK_DIV_POSEDGE_EN_PULSE_CFG, (target << PDM_CLK_DIV_POSEDGE_EN_PULSE_CFG_POS));
}

/**
  * @brief  Get PDM posedge en pulse cfg
  *
  *  Register|BitsName
  *  --------|--------
  *  PDM_CLK_DIV | TARGET
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_get_posedge_en_pulse_cfg(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->CLK_DIV, PDM_CLK_DIV_POSEDGE_EN_PULSE_CFG) >> PDM_CLK_DIV_POSEDGE_EN_PULSE_CFG_POS);
}

/**
  * @brief  Set PDM negedge en pulse cfg
  * @note This function is Configures how many 16MHz clock cycles after rising edge of PDM_CLK to sample data (negative  edge capture)
  *
  *  Register|BitsName
  *  --------|--------
  *  PDM_CLK_DIV | TARGET
  *
  * @param  PDMx PDM instance
  * @param  target This parameter can be one of the following values:
  *         Min_Data = 0 and Max_Data = 0xF
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_negedge_en_pulse_cfg(pdm_regs_t *PDMx, uint32_t target)
{
    MODIFY_REG(PDMx->CLK_DIV, PDM_CLK_DIV_NEGEDGE_EN_PULSE_CFG, (target << PDM_CLK_DIV_NEGEDGE_EN_PULSE_CFG_POS));
}

/**
  * @brief  Get PDM posedge en pulse cfg
  *
  *  Register|BitsName
  *  --------|--------
  *  PDM_CLK_DIV | TARGET
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_get_negedge_en_pulse_cfg(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->CLK_DIV, PDM_CLK_DIV_NEGEDGE_EN_PULSE_CFG) >> PDM_CLK_DIV_NEGEDGE_EN_PULSE_CFG_POS);
}

/**
  * @brief  PDM Module enable left channel sample dmic.
  * @note This function is used to set PDM left  enable register
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | ENABLE_RX
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_channel(pdm_regs_t *PDMx)
{

    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_RX | PDM_EN_L_EN_STAGE0 | PDM_EN_L_EN_STAGE1 |\
                           PDM_EN_L_EN_STAGE2 | PDM_EN_L_EN_STAGE3 | PDM_EN_L_EN_STAGE4 |\
                           PDM_EN_L_EN_STAGE5 | PDM_EN_L_EN_STAGE6 | PDM_EN_L_EN_STAGE7 |\
                           PDM_EN_L_SMP_DMIC | PDM_EN_L_EN_HPF, \
                           PDM_EN_L_EN_RX_ENABLE | PDM_EN_L_SMP_DMIC_ENABLE | PDM_EN_L_EN_STAGE0_DISABLE |\
                           PDM_EN_L_EN_STAGE1_DISABLE | PDM_EN_L_EN_STAGE2_ENABLE | PDM_EN_L_EN_STAGE3_ENABLE |\
                           PDM_EN_L_EN_STAGE4_ENABLE | PDM_EN_L_EN_STAGE5_ENABLE | PDM_EN_L_EN_STAGE6_ENABLE |\
                           PDM_EN_L_EN_STAGE7_ENABLE | PDM_EN_L_EN_HPF_ENABLE);
}

/**
  * @brief  PDM Module disable left channel sample dmic.
  * @note This function is used to set PDM left  disable register
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | ENABLE_RX
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_left_channel(pdm_regs_t *PDMx)
{

    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_RX | PDM_EN_L_EN_STAGE0 | PDM_EN_L_EN_STAGE1 |\
                           PDM_EN_L_EN_STAGE2 | PDM_EN_L_EN_STAGE3 | PDM_EN_L_EN_STAGE4 |\
                           PDM_EN_L_EN_STAGE5 | PDM_EN_L_EN_STAGE6 | PDM_EN_L_EN_STAGE7 |\
                           PDM_EN_L_SMP_DMIC | PDM_EN_L_EN_HPF, \
                           PDM_EN_L_EN_RX_DISABLE | PDM_EN_L_SMP_DMIC_DISABLE | PDM_EN_L_EN_STAGE0_DISABLE |\
                           PDM_EN_L_EN_STAGE1_DISABLE | PDM_EN_L_EN_STAGE2_DISABLE | PDM_EN_L_EN_STAGE3_DISABLE |\
                           PDM_EN_L_EN_STAGE4_DISABLE | PDM_EN_L_EN_STAGE5_DISABLE | PDM_EN_L_EN_STAGE6_DISABLE |\
                           PDM_EN_L_EN_STAGE7_DISABLE | PDM_EN_L_EN_HPF_DISABLE);
}

/**
  * @brief  PDM Module enable right channel sample dmic..
  * @note This function is used to set PDM right  enable register
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | ENABLE_RX
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_channel(pdm_regs_t *PDMx)
{

    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_RX | PDM_EN_R_EN_STAGE0 | PDM_EN_R_EN_STAGE1 |\
                           PDM_EN_R_EN_STAGE2 | PDM_EN_R_EN_STAGE3 | PDM_EN_R_EN_STAGE4 |\
                           PDM_EN_R_EN_STAGE5 | PDM_EN_R_EN_STAGE6 | PDM_EN_R_EN_STAGE7 |\
                           PDM_EN_R_SMP_DMIC | PDM_EN_R_EN_HPF, \
                           PDM_EN_R_EN_RX_ENABLE | PDM_EN_R_SMP_DMIC_ENABLE | PDM_EN_R_EN_STAGE0_DISABLE |\
                           PDM_EN_R_EN_STAGE1_DISABLE | PDM_EN_R_EN_STAGE2_ENABLE | PDM_EN_R_EN_STAGE3_ENABLE |\
                           PDM_EN_R_EN_STAGE4_ENABLE | PDM_EN_R_EN_STAGE5_ENABLE | PDM_EN_R_EN_STAGE6_ENABLE |\
                           PDM_EN_R_EN_STAGE7_ENABLE | PDM_EN_R_EN_HPF_ENABLE);
}

/**
  * @brief  PDM Module disable right channel sample dmic..
  * @note This function is used to set PDM right  disable register
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | ENABLE_RX
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_right_channel(pdm_regs_t *PDMx)
{

    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_RX | PDM_EN_R_EN_STAGE0 | PDM_EN_R_EN_STAGE1 |\
                           PDM_EN_R_EN_STAGE2 | PDM_EN_R_EN_STAGE3 | PDM_EN_R_EN_STAGE4 |\
                           PDM_EN_R_EN_STAGE5 | PDM_EN_R_EN_STAGE6 | PDM_EN_R_EN_STAGE7 |\
                           PDM_EN_R_SMP_DMIC | PDM_EN_R_EN_HPF, \
                           PDM_EN_R_EN_RX_DISABLE | PDM_EN_R_SMP_DMIC_DISABLE | PDM_EN_R_EN_STAGE0_DISABLE |\
                           PDM_EN_R_EN_STAGE1_DISABLE | PDM_EN_R_EN_STAGE2_DISABLE | PDM_EN_R_EN_STAGE3_DISABLE |\
                           PDM_EN_R_EN_STAGE4_DISABLE | PDM_EN_R_EN_STAGE5_DISABLE | PDM_EN_R_EN_STAGE6_DISABLE |\
                           PDM_EN_R_EN_STAGE7_DISABLE | PDM_EN_R_EN_HPF_DISABLE);
}

/**
  * @brief  Enable PDM Module Left RX.
  * @note This function is used to set PDM left RX enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | ENABLE_RX
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_rx(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_RX, LL_PDM_LEFT_RX_ENABLE);
}

/**
  * @brief  Disable PDM Module Left RX.
  * @note This function is used to set PDM left RX disable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | ENABLE_RX
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_left_rx(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_RX, LL_PDM_LEFT_RX_DISABLE);
}

/**
  * @brief  is PDM enables left rx
  * @note This function is used to check the PDM Module left rx is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | ENABLE_RX
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_rx(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_L, PDM_EN_L_EN_RX) == LL_PDM_LEFT_RX_ENABLE);
}


/**
  * @brief  Enable PDM Module right RX.
  * @note This function is used to set PDM right RX enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | ENABLE_RX
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_rx(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_RX, LL_PDM_RIGHT_RX_ENABLE);
}

/**
  * @brief  Disable PDM Module right RX.
  * @note This function is used to set PDM right RX disable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | ENABLE_RX
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_right_rx(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_RX, LL_PDM_RIGHT_RX_DISABLE);
}

/**
  * @brief  is PDM enables right rx
  * @note This function is used to check the PDM Module right rx is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_rx(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_R, PDM_EN_R_EN_RX) == LL_PDM_RIGHT_RX_ENABLE);
}


/**
  * @brief  Enable PDM Module Left sample dmic.
  * @note This function is used to set PDM Left sample dmic enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | SAMPLE_DMIC
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_sample_dmic(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_SMP_DMIC, LL_PDM_LEFT_SAMPLE_DMIC_ENABLE);
}

/**
  * @brief  Disable PDM Module Left sample dmic.
  * @note This function is used to set PDM Left sample dmic enable

  *  Register|BitsName
  *  --------|--------
  *  EN_L | SAMPLE_DMIC
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_left_sample_dmic(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_SMP_DMIC, LL_PDM_LEFT_SAMPLE_DMIC_DISABLE);
}

/**
  * @brief  is PDM enables left rx
  * @note This function is used to check the PDM Module left rx is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | SAMPLE_DMIC
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_sample_dmic(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_L, PDM_EN_L_SMP_DMIC) == LL_PDM_LEFT_SAMPLE_DMIC_ENABLE);
}

/**
  * @brief  Enable PDM Module right sample dmic.
  * @note This function is used to set PDM right sample dmic enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | SAMPLE_DMIC
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_sample_dmic(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_SMP_DMIC, LL_PDM_RIGHT_SAMPLE_DMIC_ENABLE);
}

/**
  * @brief  Disable PDM Module right sample dmic.
  * @note This function is used to set PDM right sample dmic enable

  *  Register|BitsName
  *  --------|--------
  *  EN_R | SAMPLE_DMIC
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_right_sample_dmic(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_SMP_DMIC, LL_PDM_RIGHT_SAMPLE_DMIC_DISABLE);
}

/**
  * @brief  is PDM enables right rx
  * @note This function is used to check the PDM Module right rx is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | SAMPLE_DMIC
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_sample_dmic(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_R, PDM_EN_R_SMP_DMIC) == LL_PDM_RIGHT_SAMPLE_DMIC_ENABLE);
}

/**
  * @brief  Enable PDM Module Left stage0.
  * @note This function is used to set PDM Left stage0 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE0
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_stage0(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE0, LL_PDM_LEFT_STAGE0_ENABLE);
}

/**
  * @brief  Disable PDM Module Left stage0.
  * @note This function is used to set PDM Left stage0 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE0
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_left_stage0(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE0, LL_PDM_LEFT_STAGE0_DISABLE);
}

/**
  * @brief  is PDM enables left stage0
  * @note This function is used to check the PDM Module left stage0 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE0
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_stage0(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_L, PDM_EN_L_EN_STAGE0) == LL_PDM_LEFT_STAGE0_ENABLE);
}

/**
  * @brief  Enable PDM Module right stage0
  * @note This function is used to set PDM right stage0 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE0
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_stage0(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE0, LL_PDM_RIGHT_STAGE0_ENABLE);
}

/**
  * @brief  Disable PDM Module right stage0
  * @note This function is used to set PDM right stage0 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE0
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_right_stage0(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE0, LL_PDM_RIGHT_STAGE0_DISABLE);
}

/**
  * @brief  is PDM enables right stage0
  * @note This function is used to check the PDM Module right stage0is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE0
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_stage0(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_R, PDM_EN_R_EN_STAGE0) == LL_PDM_RIGHT_STAGE0_ENABLE);
}

/**
  * @brief  Enable PDM Module Left stage1.
  * @note This function is used to set PDM Left stage1 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE1
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_stage1(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE1, LL_PDM_LEFT_STAGE1_ENABLE);
}

/**
  * @brief  Disable PDM Module Left stage1.
  * @note This function is used to set PDM Left stage1 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE1
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_left_stage1(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE1, LL_PDM_LEFT_STAGE1_DISABLE);
}

/**
  * @brief  is PDM enables left stage1
  * @note This function is used to check the PDM Module left stage1 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE1
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_stage1(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_L, PDM_EN_L_EN_STAGE1) == LL_PDM_LEFT_STAGE1_ENABLE);
}

/**
  * @brief  Enable PDM Module right stage1
  * @note This function is used to set PDM right stage1 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE1
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_stage1(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE1, LL_PDM_RIGHT_STAGE1_ENABLE);
}

/**
  * @brief  Disable PDM Module right stage1
  * @note This function is used to set PDM right stage1 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE1
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_right_stage1(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE1, LL_PDM_RIGHT_STAGE1_DISABLE);
}

/**
  * @brief  is PDM enables right stage1
  * @note This function is used to check the PDM Module right stage1is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE1
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_stage1(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_R, PDM_EN_R_EN_STAGE1) == LL_PDM_RIGHT_STAGE1_ENABLE);
}

/**
  * @brief  Enable PDM Module Left stage2.
  * @note This function is used to set PDM Left stage2 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE2
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_stage2(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE2, LL_PDM_LEFT_STAGE2_ENABLE);
}

/**
  * @brief  Disable PDM Module Left stage2.
  * @note This function is used to set PDM Left stage1 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE2
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_left_stage2(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE2, LL_PDM_LEFT_STAGE2_DISABLE);
}

/**
  * @brief  is PDM enables left stage2
  * @note This function is used to check the PDM Module left stage2 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE2
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_stage2(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_L, PDM_EN_L_EN_STAGE2) == LL_PDM_LEFT_STAGE2_ENABLE);
}

/**
  * @brief  Enable PDM Module right stage2
  * @note This function is used to set PDM right stage2 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE2
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_stage2(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE2, LL_PDM_RIGHT_STAGE2_ENABLE);
}

/**
  * @brief  Disable PDM Module right stage2
  * @note This function is used to set PDM right stage2 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE2
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_right_stage2(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE2, LL_PDM_RIGHT_STAGE2_DISABLE);
}

/**
  * @brief  is PDM enables right stage2
  * @note This function is used to check the PDM Module right stage2 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE2
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_stage2(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_R, PDM_EN_R_EN_STAGE2) == LL_PDM_RIGHT_STAGE2_ENABLE);
}

/**
  * @brief  Enable PDM Module Left stage3.
  * @note This function is used to set PDM Left stage3 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE3
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_stage3(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE3, LL_PDM_LEFT_STAGE3_ENABLE);
}

/**
  * @brief  Disable PDM Module Left stage3.
  * @note This function is used to set PDM Left stage3 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE3
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_left_stage3(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE3, LL_PDM_LEFT_STAGE3_DISABLE);
}

/**
  * @brief  is PDM enables left stage3
  * @note This function is used to check the PDM Module left stage3 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE3
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_stage3(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_L, PDM_EN_L_EN_STAGE3) == LL_PDM_LEFT_STAGE3_ENABLE);
}

/**
  * @brief  Enable PDM Module right stage3
  * @note This function is used to set PDM right stage3 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE3
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_stage3(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE3, LL_PDM_RIGHT_STAGE3_ENABLE);
}

/**
  * @brief  Disable PDM Module right stage3
  * @note This function is used to set PDM right stage3 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE3
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_right_stage3(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE3, LL_PDM_RIGHT_STAGE3_DISABLE);
}

/**
  * @brief  is PDM enables right stage3
  * @note This function is used to check the PDM Module right stage3 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE3
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_stage3(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_R, PDM_EN_R_EN_STAGE3) == LL_PDM_RIGHT_STAGE3_ENABLE);
}

/**
  * @brief  Enable PDM Module Left stage4.
  * @note This function is used to set PDM Left stage4 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE4
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_stage4(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE4, LL_PDM_LEFT_STAGE4_ENABLE);
}

/**
  * @brief  Disable PDM Module Left stage4.
  * @note This function is used to set PDM Left stage4 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE4
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_left_stage4(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE4, LL_PDM_LEFT_STAGE4_DISABLE);
}

/**
  * @brief  is PDM enables left stage4
  * @note This function is used to check the PDM Module left stage4 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE4
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_stage4(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_L, PDM_EN_L_EN_STAGE4) == LL_PDM_LEFT_STAGE4_ENABLE);
}

/**
  * @brief  Enable PDM Module right stage4
  * @note This function is used to set PDM right stage4 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE4
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_stage4(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE4, LL_PDM_RIGHT_STAGE4_ENABLE);
}

/**
  * @brief  Disable PDM Module right stage4
  * @note This function is used to set PDM right stage4 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE4
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_right_stage4(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE4, LL_PDM_RIGHT_STAGE4_DISABLE);
}

/**
  * @brief  is PDM enables right stage4
  * @note This function is used to check the PDM Module right stage4 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE4
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_stage4(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_R, PDM_EN_R_EN_STAGE4) == LL_PDM_RIGHT_STAGE4_ENABLE);
}

/**
  * @brief  Enable PDM Module Left stage5.
  * @note This function is used to set PDM Left stage5 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE5
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_stage5(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE5, LL_PDM_LEFT_STAGE5_ENABLE);
}

/**
  * @brief  Disable PDM Module Left stage5.
  * @note This function is used to set PDM Left stage5 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE5
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_left_stage5(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE5, LL_PDM_LEFT_STAGE5_DISABLE);
}

/**
  * @brief  is PDM enables left stage5
  * @note This function is used to check the PDM Module left stage5 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE5
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_stage5(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_L, PDM_EN_L_EN_STAGE5) == LL_PDM_LEFT_STAGE5_ENABLE);
}

/**
  * @brief  Enable PDM Module right stage5
  * @note This function is used to set PDM right stage5 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE5
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_stage5(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE5, LL_PDM_RIGHT_STAGE5_ENABLE);
}

/**
  * @brief  Disable PDM Module right stage5
  * @note This function is used to set PDM right stage5 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE5
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_right_stage5(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE5, LL_PDM_RIGHT_STAGE5_DISABLE);
}

/**
  * @brief  is PDM enables right stage5
  * @note This function is used to check the PDM Module right stage5 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE5
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_stage5(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_R, PDM_EN_R_EN_STAGE5) == LL_PDM_RIGHT_STAGE5_ENABLE);
}

/**
  * @brief  Enable PDM Module Left stage6.
  * @note This function is used to set PDM Left stage6 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE6
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_stage6(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE6, LL_PDM_LEFT_STAGE6_ENABLE);
}

/**
  * @brief  Disable PDM Module Left stage6.
  * @note This function is used to set PDM Left stage6 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE6
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_left_stage6(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE6, LL_PDM_LEFT_STAGE6_DISABLE);
}

/**
  * @brief  is PDM enables left stage6
  * @note This function is used to check the PDM Module left stage6 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE6
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_stage6(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_L, PDM_EN_L_EN_STAGE6) == LL_PDM_LEFT_STAGE6_ENABLE);
}

/**
  * @brief  Enable PDM Module right stage6
  * @note This function is used to set PDM right stage6 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE6
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_stage6(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE6, LL_PDM_RIGHT_STAGE6_ENABLE);
}

/**
  * @brief  Disable PDM Module right stage6
  * @note This function is used to set PDM right stage6 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE6
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_right_stage6(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE6, LL_PDM_RIGHT_STAGE6_DISABLE);
}

/**
  * @brief  is PDM enables right stage6
  * @note This function is used to check the PDM Module right stage6 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE6
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_stage6(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_R, PDM_EN_R_EN_STAGE6) == LL_PDM_RIGHT_STAGE6_ENABLE);
}

/**
  * @brief  Enable PDM Module Left stage7.
  * @note This function is used to set PDM Left stage7 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE7
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_stage7(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE7, LL_PDM_LEFT_STAGE7_ENABLE);
}

/**
  * @brief  Disable PDM Module Left stage7.
  * @note This function is used to set PDM Left stage7 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE7
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_left_stage7(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_STAGE7, LL_PDM_LEFT_STAGE7_DISABLE);
}

/**
  * @brief  is PDM enables left stage7
  * @note This function is used to check the PDM Module left stage7 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_STAGE7
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_stage7(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_L, PDM_EN_L_EN_STAGE7) == LL_PDM_LEFT_STAGE7_ENABLE);
}

/**
  * @brief  Enable PDM Module right stage7
  * @note This function is used to set PDM right stage7 enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE7
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_stage7(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE7, LL_PDM_RIGHT_STAGE7_ENABLE);
}

/**
  * @brief  Disable PDM Module right stage7
  * @note This function is used to set PDM right stage7 enable

  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE7
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_right_stage7(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_STAGE7, LL_PDM_RIGHT_STAGE7_DISABLE);
}

/**
  * @brief  is PDM enables right stage7
  * @note This function is used to check the PDM Module right stage7 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE7
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_stage7(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_R, PDM_EN_R_EN_STAGE7) == LL_PDM_RIGHT_STAGE7_ENABLE);
}

/**
  * @brief  Enable PDM Module Left hpf
  * @note This function is used to set PDM Left hpf enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_HPF
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_hpf(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_HPF, LL_PDM_LEFT_HPF_ENABLE);
}

/**
  * @brief  Disable PDM Module Left hpf.
  * @note This function is used to set PDM Left hpf enable

  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_HPF
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_left_hpf(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_L, PDM_EN_L_EN_HPF, LL_PDM_LEFT_HPF_DISABLE);
}

/**
  * @brief  is PDM enables left hpf
  * @note This function is used to check the PDM Module left hpf is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_L | EN_HPF
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_hpf(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_L, PDM_EN_L_EN_HPF) == LL_PDM_LEFT_HPF_ENABLE);
}

/**
  * @brief  Enable PDM Module right hpf
  * @note This function is used to set PDM right hpf enable
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_HPF
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_hpf(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_HPF, LL_PDM_RIGHT_HPF_ENABLE);
}

/**
  * @brief  Disable PDM Module right hpf.
  * @note This function is used to set PDM right hpf enable

  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_HPF
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_disable_right_hpf(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->EN_R, PDM_EN_R_EN_HPF, LL_PDM_RIGHT_HPF_DISABLE);
}

/**
  * @brief  is PDM enables right hpf
  * @note This function is used to check the PDM Module right hpf is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_HPF
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_hpf(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->EN_R, PDM_EN_R_EN_HPF) == LL_PDM_RIGHT_HPF_ENABLE);
}

/**
  * @brief  Set PDM left rxd upsample
  * @note This function is used to set PDM left rxd upsample
  *
  *  Register|BitsName
  *  --------|--------
  *  INPUT_CFG_L | RXD_UPSAMPLE
  *
  * @param  PDMx PDM instance
  * @param  rxd_upsample This parameter can be one of the following values:
  *         Min_Data = 0 and Max_Data = 0x1
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_left_rxd_upsample(pdm_regs_t *PDMx, uint32_t rxd_upsample)
{
    MODIFY_REG(PDMx->IN_CFG_L, PDM_IN_CFG_L_RX_UPSMP, rxd_upsample << PDM_IN_CFG_L_RX_UPSMP_POS);
}

/**
  * @brief  Get PDM left rxd upsample.
  * @note This function is used to get PDM left rxd upsample
  *
  *  Register|BitsName
  *  --------|--------
  *  INPUT_CFG_L | RXD_UPSAMPLE
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         0x0 or 0x1
  */
__STATIC_INLINE uint32_t ll_pdm_get_left_rxd_upsample(pdm_regs_t *PDMx)
{
    return READ_BITS(PDMx->IN_CFG_L, PDM_IN_CFG_L_RX_UPSMP >> PDM_IN_CFG_L_RX_UPSMP_POS);
}

/**
  * @brief  Set PDM right rxd upsample
  * @note This function is used to set PDM right rxd upsample
  *
  *  Register|BitsName
  *  --------|--------
  *  INPUT_CFG_R | RXD_UPSAMPLE
  *
  * @param  PDMx PDM instance
  * @param  rxd_upsample This parameter can be one of the following values:
  *         Min_Data = 0 and Max_Data = 0x1
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_right_rxd_upsample(pdm_regs_t *PDMx, uint32_t rxd_upsample)
{
    MODIFY_REG(PDMx->IN_CFG_R, PDM_IN_CFG_R_RX_UPSMP, rxd_upsample << PDM_IN_CFG_R_RX_UPSMP_POS);
}

/**
  * @brief  Get PDM right rxd upsample.
  * @note This function is used to get PDM right rxd upsample
  *
  *  Register|BitsName
  *  --------|--------
  *  INPUT_CFG_L | RXD_UPSAMPLE
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         0x0 or 0x1
  */
__STATIC_INLINE uint32_t ll_pdm_get_right_rxd_upsample(pdm_regs_t *PDMx)
{
    return READ_BITS(PDMx->IN_CFG_R, PDM_IN_CFG_R_RX_UPSMP >> PDM_IN_CFG_R_RX_UPSMP_POS);
}

/**
  * @brief  Set PDM left statge init
  * @note This function is used to set PDM left statge init
  *
  *  Register|BitsName
  *  --------|--------
  *  INPUT_CFG_L | STAGE_INIT
  *
  * @param  PDMx PDM instance
  * @param  stage_init This parameter can be one of the following values:
  *         Min_Data = 0 and Max_Data = 0x2
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_left_stage_init(pdm_regs_t *PDMx, uint32_t stage_init)
{
    MODIFY_REG(PDMx->IN_CFG_L, PDM_IN_CFG_L_STAGE_INIT, stage_init << PDM_IN_CFG_L_STAGE_INIT_POS);
}

/**
  * @brief  Get PDM left statge init
  * @note This function is used to get PDM left statge init
  *
  *  Register|BitsName
  *  --------|--------
  *  INPUT_CFG_L | STAGE_INIT
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         0x0 or 0x2
  */
__STATIC_INLINE uint32_t ll_pdm_get_left_stage_init(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->IN_CFG_L, PDM_IN_CFG_L_STAGE_INIT) >> PDM_IN_CFG_L_STAGE_INIT_POS) ;
}

/**
  * @brief  Set PDM right statge init
  * @note This function is used to set PDM right statge init
  *
  *  Register|BitsName
  *  --------|--------
  *  INPUT_CFG_R | STAGE_INIT
  *
  * @param  PDMx PDM instance
  * @param  stage_init This parameter can be one of the following values:
  *         Min_Data = 0 and Max_Data = 0x2
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_right_stage_init(pdm_regs_t *PDMx, uint32_t stage_init)
{
    MODIFY_REG(PDMx->IN_CFG_R, PDM_IN_CFG_R_STAGE_INIT, stage_init << PDM_IN_CFG_R_STAGE_INIT_POS);
}

/**
  * @brief  Get PDM right statge init
  * @note This function is used to get PDM right statge init
  *
  *  Register|BitsName
  *  --------|--------
  *  INPUT_CFG_R | STAGE_INIT
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         0x0 or 0x2
  */
__STATIC_INLINE uint32_t ll_pdm_get_right_stage_init(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->IN_CFG_R, PDM_IN_CFG_R_STAGE_INIT) >> PDM_IN_CFG_R_STAGE_INIT_POS) ;
}

/**
  * @brief  Set PDM left upsample factor
  * @note This function is used to set PDM left upsample factor
  *
  *  Register|BitsName
  *  --------|--------
  *  LPF_CFG_L | UPSAMPLE_FACTOR
  *
  * @param  PDMx PDM instance
  * @param  upsample_factor This parameter can be one of the following values:
  *         Min_Data = 0 and Max_Data = 0x2
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_left_upsample_factor(pdm_regs_t *PDMx, uint32_t upsample_factor)
{
    MODIFY_REG(PDMx->LPF_CFG_L, PDM_LPF_CFG_L_UPSMP_FACTOR, upsample_factor << PDM_LPF_CFG_L_UPSMP_FACTOR_POS);
}

/**
  * @brief  Get PDM left upsample factor
  * @note This function is used to get PDM left upsample factor
  *
  *  Register|BitsName
  *  --------|--------
  *  LPF_CFG_L | UPSAMPLE_FACTOR
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         0x0 or 0x2
  */
__STATIC_INLINE uint32_t ll_pdm_get_left_upsample_factor(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->LPF_CFG_L, PDM_LPF_CFG_L_UPSMP_FACTOR) >> PDM_LPF_CFG_L_UPSMP_FACTOR_POS) ;
}

/**
  * @brief  Set PDM right upsample factor
  * @note This function is used to set PDM right upsample factor
  *
  *  Register|BitsName
  *  --------|--------
  *  LPF_CFG_R | UPSAMPLE_FACTOR
  *
  * @param  PDMx PDM instance
  * @param  upsample_factor This parameter can be one of the following values:
  *         Min_Data = 0 and Max_Data = 0x2
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_right_upsample_factor(pdm_regs_t *PDMx, uint32_t upsample_factor)
{
    MODIFY_REG(PDMx->LPF_CFG_R, PDM_LPF_CFG_R_UPSMP_FACTOR, upsample_factor << PDM_LPF_CFG_R_UPSMP_FACTOR_POS);
}

/**
  * @brief  Get PDM right upsample factor
  * @note This function is used to get PDM right upsample factor
  *
  *  Register|BitsName
  *  --------|--------
  *  LPF_CFG_R | UPSAMPLE_FACTOR
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         0x0 or 0x2
  */
__STATIC_INLINE uint32_t ll_pdm_get_right_upsample_factor(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->LPF_CFG_R, PDM_LPF_CFG_R_UPSMP_FACTOR) >> PDM_LPF_CFG_R_UPSMP_FACTOR_POS) ;
}

/**
  * @brief  Enable PDM left hpf_bypass
  * @note This function is used to set PDM left hpf_bypass
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_L | HPY_BYPASS
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_hpy_bypass(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->HPF_CFG_L, PDM_HPF_CFG_L_BYPASS, LL_PDM_LEFT_HPF_BYPASS_ENABLE);
}

/**
  * @brief  Disable PDM left hpf_bypass
  * @note This function is used to disable PDM left hpf_bypass
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_L | HPY_BYPASS
  *
  * @param  PDMx PDM instance
  */
__STATIC_INLINE void ll_pdm_disable_left_hpy_bypass(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->HPF_CFG_L, PDM_HPF_CFG_L_BYPASS, LL_PDM_LEFT_HPF_BYPASS_DISABLE);
}

/**
  * @brief  is PDM enables right stage7
  * @note This function is used to check the PDM Module right stage7 is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  EN_R | EN_STAGE7
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_hpy_bypass(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->HPF_CFG_L, PDM_HPF_CFG_L_BYPASS) == LL_PDM_LEFT_HPF_BYPASS_ENABLE);
}

/**
  * @brief  Enable PDM right hpf_bypass
  * @note This function is used to set PDM right hpf_bypass
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_R | HPY_BYPASS
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_hpy_bypass(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->HPF_CFG_R, PDM_HPF_CFG_R_BYPASS, LL_PDM_RIGHT_HPF_BYPASS_ENABLE);
}

/**
  * @brief  Disable PDM right hpf_bypass
  * @note This function is used to disable PDM right hpf_bypass
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_R | HPY_BYPASS
  *
  * @param  PDMx PDM instance
  */
__STATIC_INLINE void ll_pdm_disable_right_hpy_bypass(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->HPF_CFG_R, PDM_HPF_CFG_R_BYPASS, LL_PDM_RIGHT_HPF_BYPASS_DISABLE);
}

/**
  * @brief  is PDM enables right hpf_bypass
  * @note This function is used to check the PDM Module right hpf_bypass is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_R | HPY_BYPASS
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_hpy_bypass(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->HPF_CFG_R, PDM_HPF_CFG_R_BYPASS) == LL_PDM_RIGHT_HPF_BYPASS_ENABLE);
}

/**
  * @brief  Set PDM left hpf corner
  * @note This function is used to set PDM left  hpf corner
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_L | HPF_CORNER
  *
  * @param  PDMx PDM instance
  * @param  hpf_corner This parameter can be one of the following values:
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_0_25
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_1
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_4
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_16
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_left_hpf_corner(pdm_regs_t *PDMx, uint32_t hpf_corner)
{
    MODIFY_REG(PDMx->HPF_CFG_L, PDM_HPF_CFG_L_CORNER, hpf_corner);
}

/**
  * @brief  Get PDM left hpf corner
  * @note This function is used to get PDM left hpf corner
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_L | HPF_CORNER
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_0_25
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_1
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_4
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_16
  */
__STATIC_INLINE uint32_t ll_pdm_get_left_hpf_corner(pdm_regs_t *PDMx)
{
    return READ_BITS(PDMx->HPF_CFG_L, PDM_HPF_CFG_L_CORNER);
}

/**
  * @brief  Set PDM right hpf corner
  * @note This function is used to set PDM right  hpf corner
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_R | HPF_CORNER
  *
  * @param  PDMx PDM instance
  * @param  hpf_corner This parameter can be one of the following values:
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_0_25
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_1
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_4
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_16
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_right_hpf_corner(pdm_regs_t *PDMx, uint32_t hpf_corner)
{
    MODIFY_REG(PDMx->HPF_CFG_R, PDM_HPF_CFG_R_CORNER, hpf_corner);
}

/**
  * @brief  Get PDM right hpf corner
  * @note This function is used to get PDM right hpf corner
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_R | HPF_CORNER
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_0_25
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_1
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_4
  *         @arg @ref LL_PDM_LEFT_HPF_CORNER_16
  */
__STATIC_INLINE uint32_t ll_pdm_get_right_hpf_corner(pdm_regs_t *PDMx)
{
    return READ_BITS(PDMx->HPF_CFG_R, PDM_HPF_CFG_R_CORNER);
}

/**
  * @brief  Enable PDM left hpf_freeze_en
  * @note This function is used to set PDM left hpf_freeze_en
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_L | HPY_FREEZE_EN
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_left_hpf_freeze_en(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->HPF_CFG_L, PDM_HPF_CFG_L_FREEZE_EN, LL_PDM_LEFT_HPF_FREEZE_ENABLE);
}

/**
  * @brief  Disable PDM left hpf_freeze_en
  * @note This function is used to disable PDM left hpf_freeze_en
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_L | HPY_FREEZE_EN
  *
  * @param  PDMx PDM instance
  */
__STATIC_INLINE void ll_pdm_disable_left_hpf_freeze_en(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->HPF_CFG_L, PDM_HPF_CFG_L_FREEZE_EN, LL_PDM_LEFT_HPF_FREEZE_DISABLE);
}

/**
  * @brief  is PDM enables left hpf_freeze_en
  * @note This function is used to check the PDM Module left hpf_freeze_en is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_L | HPY_FREEZE_EN
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_left_hpf_freeze_en(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->HPF_CFG_L, PDM_HPF_CFG_L_FREEZE_EN) == LL_PDM_LEFT_HPF_FREEZE_ENABLE);
}

/**
  * @brief  Enable PDM right hpf_freeze_en
  * @note This function is used to set PDM right hpf_freeze_en
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_R | HPY_FREEZE_EN
  *
  * @param  PDMx PDM instance
  * @retval None
  */
__STATIC_INLINE void ll_pdm_enable_right_hpf_freeze_en(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->HPF_CFG_R, PDM_HPF_CFG_R_FREEZE_EN, LL_PDM_RIGHT_HPF_FREEZE_ENABLE);
}

/**
  * @brief  Disable PDM right hpf_freeze_en
  * @note This function is used to disable PDM righthpf_freeze_en
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_R | HPY_FREEZE_EN
  *
  * @param  PDMx PDM instance
  */
__STATIC_INLINE void ll_pdm_disable_right_hpf_freeze_en(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->HPF_CFG_R, PDM_HPF_CFG_R_FREEZE_EN, LL_PDM_RIGHT_HPF_FREEZE_DISABLE);
}

/**
  * @brief  is PDM enables right hpf_freeze_en
  * @note This function is used to check the PDM Module right hpf_freeze_en is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  HPF_CFG_R | HPY_FREEZE_EN
  *
  * @param  PDMx PDM instance.
  * @retval None
  */
__STATIC_INLINE uint32_t ll_pdm_is_enable_right_hpf_freeze_en(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->HPF_CFG_R, PDM_HPF_CFG_R_FREEZE_EN) == LL_PDM_RIGHT_HPF_FREEZE_ENABLE);
}

/**
  * @brief  Set PDM left pga_value
  * @note This function is used to set PDM left pga_value
  *
  *  Register|BitsName
  *  --------|--------
  *  PGA_CFG_L | PGA_VAL
  *
  * @param  PDMx PDM instance
  * @param  pga_val This parameter can be one of the following values:
  *         Min 0x0 Max 0x3FFF
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_left_pga_val(pdm_regs_t *PDMx, uint32_t pga_val)
{
    MODIFY_REG(PDMx->PGA_CFG_L, PDM_PGA_CFG_L_VAL, pga_val << PDM_PGA_CFG_L_VAL_POS);
}

/**
  * @brief  Get PDM left pga_value
  * @note This function is used to get PDM left pga_value
  *
  *  Register|BitsName
  *  --------|--------
  *  PGA_CFG_L | PGA_VAL
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         Min 0x0 Max 0x3FFF
  */
__STATIC_INLINE uint32_t ll_pdm_get_left_pga_val(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->PGA_CFG_L, PDM_PGA_CFG_L_VAL) >> PDM_PGA_CFG_L_VAL_POS);
}

/**
  * @brief  Set PDM right pga_value
  * @note This function is used to set PDM right pga_value
  *
  *  Register|BitsName
  *  --------|--------
  *  PGA_CFG_R | PGA_VAL
  *
  * @param  PDMx PDM instance
  * @param  pga_val This parameter can be one of the following values:
  *         Min 0x0 Max 0x3FFF
  * @retval None
  */
__STATIC_INLINE void ll_pdm_set_right_pga_val(pdm_regs_t *PDMx, uint32_t pga_val)
{
    MODIFY_REG(PDMx->PGA_CFG_R, PDM_PGA_CFG_R_VAL, pga_val << PDM_PGA_CFG_R_VAL_POS);
}

/**
  * @brief  Get PDM right pga_value
  * @note This function is used to get PDM right pga_value
  *
  *  Register|BitsName
  *  --------|--------
  *  PGA_CFG_R | PGA_VAL
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         Min 0x0 Max 0x3FFF
  */
__STATIC_INLINE uint32_t ll_pdm_get_right_pga_val(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->PGA_CFG_R, PDM_PGA_CFG_R_VAL) >> PDM_PGA_CFG_R_VAL_POS);
}

/**
  * @brief  Get PDM left data
  * @note This function is used to get PDM left data
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_L | DATA
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         Min 0x0 Max 0xFFFF
  */
__STATIC_INLINE uint32_t ll_pdm_get_left_data(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->DATA_L, PDM_DATA_L_DATA) >> PDM_DATA_L_DATA_POS);
}

/**
  * @brief  Get PDM right data
  * @note This function is used to get PDM right data
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_R | DATA
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         Min 0x0 Max 0xFFFF
  */
__STATIC_INLINE uint32_t ll_pdm_get_right_data(pdm_regs_t *PDMx)
{
    return (READ_BITS(PDMx->DATA_R, PDM_DATA_R_DATA) >> PDM_DATA_R_DATA_POS);
}

/**
  * @brief  Clean PDM left valid
  * @note This function is used to get PDM right overflow
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_R | OVERFLOW
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         LL_PDM_LEFT_DATA_OVERFLOW
  *         LL_PDM_LEFGT_DATA_NO_OVERFLOW
  */
__STATIC_INLINE void ll_pdm_left_clear_flag_valid(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->DATA_L, PDM_DATA_L_VALID, (1 << PDM_DATA_L_VALID_POS));
}

/**
  * @brief  Get PDM left valid
  * @note This function is used to get PDM left valid
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_L | VALID
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         LL_PDM_LEFT_DATA_VALID
  *         LL_PDM_LEFT_DATA_INVALID
  */
__STATIC_INLINE uint32_t ll_pdm_left_is_active_flag_valid(pdm_regs_t *PDMx)
{
    return ((READ_BITS(PDMx->DATA_L, PDM_DATA_L_VALID) >> PDM_DATA_L_VALID_POS) == 1);
}

/**
  * @brief  Clean PDM right valid
  * @note This function is used to get PDM right overflow
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_R | OVERFLOW
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         LL_PDM_LEFT_DATA_OVERFLOW
  *         LL_PDM_LEFGT_DATA_NO_OVERFLOW
  */
__STATIC_INLINE void ll_pdm_right_clear_flag_valid(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->DATA_R, PDM_DATA_R_VALID, (1 << PDM_DATA_R_VALID_POS));
}

/**
  * @brief  Get PDM right valid
  * @note This function is used to get PDM right valid
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_R | VALID
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         LL_PDM_RIGHT_DATA_VALID
  *         LL_PDM_RIGHT_DATA_INVALID
  */
__STATIC_INLINE uint32_t ll_pdm_right_is_active_flag_valid(pdm_regs_t *PDMx)
{
    return ((READ_BITS(PDMx->DATA_R, PDM_DATA_R_VALID) >> PDM_DATA_R_VALID_POS) == 1);
}

/**
  * @brief  Set PDM left overflow
  * @note This function is used to get PDM right overflow
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_R | OVERFLOW
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         LL_PDM_LEFT_DATA_OVERFLOW
  *         LL_PDM_LEFGT_DATA_NO_OVERFLOW
  */
__STATIC_INLINE void ll_pdm_left_clear_flag_overflow(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->DATA_L, PDM_DATA_L_OVER, (1 << PDM_DATA_L_OVER_POS));
}

/**
  * @brief  Get PDM left overflow
  * @note This function is used to get PDM left overflow
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_L | OVERFLOW
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         LL_PDM_LEFT_DATA_OVERFLOW
  *         LL_PDM_LEFT_DATA_NO_OVERFLOW
  */
__STATIC_INLINE uint32_t ll_pdm_left_is_active_flag_overflow(pdm_regs_t *PDMx)
{
    return ((READ_BITS(PDMx->DATA_L, PDM_DATA_L_OVER) >> PDM_DATA_L_OVER_POS) == 1);
}

/**
  * @brief  Set PDM right overflow
  * @note This function is used to get PDM right overflow
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_R | OVERFLOW
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         LL_PDM_RIGHT_DATA_OVERFLOW
  *         LL_PDM_RIGHT_DATA_NO_OVERFLOW
  */
__STATIC_INLINE void ll_pdm_right_clear_flag_overflow(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->DATA_R, PDM_DATA_R_OVER, (1 << PDM_DATA_R_OVER_POS));
}

/**
  * @brief  Get PDM right overflow
  * @note This function is used to get PDM right overflow
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_R | OVERFLOW
  *
  * @param  PDMx PDM instance
  * @retval can be one of the following values:
  *         LL_PDM_RIGHT_DATA_OVERFLOW
  *         LL_PDM_RIGHT_DATA_NO_OVERFLOW
  */
__STATIC_INLINE uint32_t ll_pdm_right_is_active_flag_overflow(pdm_regs_t *PDMx)
{
    return ((READ_BITS(PDMx->DATA_R, PDM_DATA_R_OVER) >> PDM_DATA_R_OVER_POS) == 1);
}

/**
  * @brief  enable .valid inter
  *
  *  Register|BitsName
  *  --------|--------
  *  INTR_L | VALID_INT_MASK
  *
  * @param  PDMx PDMx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE void ll_pdm_left_enable_it_valid(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->INT_L, PDM_INT_L_VALID_MASK,(0 << PDM_INT_L_VALID_MASK_POS));
}

/**
  * @brief  disable .valid inter
  *
  *  Register|BitsName
  *  --------|--------
  *  INTR_L | VALID_INT_MASK
  *
  * @param  PDMx PDMx instance
  * @retval None.
  */
__STATIC_INLINE void ll_pdm_left_disable_it_valid(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->INT_L, PDM_INT_L_VALID_MASK,(1 << PDM_INT_L_VALID_MASK_POS));
}

/**
  * @brief  Get .valid inter status
  *
  *  Register|BitsName
  *  --------|--------
  *  INTR_L | VALID_INT_MASK
  *
  * @param  PDMx PDMx instance
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_pdm_left_is_enable_it_valid(pdm_regs_t *PDMx)
{
    return ((READ_BITS(PDMx->INT_L, PDM_INT_L_VALID_MASK) >> PDM_INT_L_VALID_MASK_POS) == 0);
}

/**
  * @brief  enable .valid inter
  *
  *  Register|BitsName
  *  --------|--------
  *  INTR_R | VALID_INT_MASK
  *
  * @param  PDMx PDMx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE void ll_pdm_right_enable_it_valid(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->INT_R, PDM_INT_R_VALID_MASK,(0 << PDM_INT_R_VALID_MASK_POS));
}

/**
  * @brief  Clean the status of .valid inter status
  *
  *  Register|BitsName
  *  --------|--------
  *  INTR_R | VALID_INT_MASK
  *
  * @param  PDMx PDMx instance
  * @retval None.
  */
__STATIC_INLINE void ll_pdm_right_disable_it_valid(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->INT_R, PDM_INT_R_VALID_MASK,(1 << PDM_INT_R_VALID_MASK_POS));
}

/**
  * @brief  Get .valid inter status
  *
  *  Register|BitsName
  *  --------|--------
  *  INTR_R | VALID_INT_MASK
  *
  * @param  PDMx PDMx instance
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_pdm_right_is_enable_it_valid(pdm_regs_t *PDMx)
{
    return ((READ_BITS(PDMx->INT_R, PDM_INT_R_VALID_MASK) >> PDM_INT_R_VALID_MASK_POS) == 0);
}

/**
  * @brief  enable .overflow inter
  *
  *  Register|BitsName
  *  --------|--------
  *  INTR_L | OVERFLOW_INT_MASK
  *
  * @param  PDMx PDMx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE void ll_pdm_left_enable_it_overflow(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->INT_L, PDM_INT_L_OVER_MASK,(0 << PDM_INT_L_OVER_MASK_POS));
}

/**
  * @brief  disable .overflow inter
  *
  *  Register|BitsName
  *  --------|--------
  *  INTR_L | OVERFLOW_INT_MASK
  *
  * @param  PDMx PDMx instance
  * @retval None.
  */
__STATIC_INLINE void ll_pdm_left_disable_it_overflow(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->INT_L, PDM_INT_L_OVER_MASK,(1 << PDM_INT_L_OVER_MASK_POS));
}

/**
  * @brief  Get .overflow inter status
  *
  *  Register|BitsName
  *  --------|--------
  *  INTR_L | OVERFLOW_INT_MASK
  *
  * @param  PDMx PDMx instance
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_pdm_left_is_enable_it_overflow(pdm_regs_t *PDMx)
{
    return ((READ_BITS(PDMx->INT_L, PDM_INT_L_OVER_MASK) >> PDM_INT_L_OVER_MASK_POS) == 0);
}

/**
  * @brief  enable .overflow inter
  *
  *  Register|BitsName
  *  --------|--------
  *  INTR_R | OVERFLOW_INT_MASK
  *
  * @param  PDMx PDMx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE void ll_pdm_right_enable_it_overflow(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->INT_R, PDM_INT_R_OVER_MASK,(0 << PDM_INT_R_OVER_MASK_POS));
}

/**
  * @brief  disable .overflow inter
  *
  *  Register|BitsName
  *  --------|--------
  *  INTR_R | OVERFLOW_INT_MASK
  *
  * @param  PDMx PDMx instance
  * @retval None.
  */
__STATIC_INLINE void ll_pdm_right_disable_it_overflow(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->INT_R, PDM_INT_R_OVER_MASK,(1 << PDM_INT_R_OVER_MASK_POS));
}

/**
  * @brief  Get .overflow inter status
  *
  *  Register|BitsName
  *  --------|--------
  *  INTR_R | OVERFLOW_INT_MASK
  *
  * @param  PDMx PDMx instance
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_pdm_right_is_enable_it_overflow(pdm_regs_t *PDMx)
{
    return ((READ_BITS(PDMx->INT_R, PDM_INT_R_OVER_MASK) >> PDM_INT_R_OVER_MASK_POS) == 0);
}

/**
  * @brief   set valid dma mask enable
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_L | VALID_DMA_MASK
  *
  * @param  PDMx PDMx instance
  * @retval None.
  */
__STATIC_INLINE void ll_pdm_left_enable_dma_mask(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->DATA_L, PDM_DATA_L_VALID_DMA_MASK,PDM_DATA_L_VALID_DMA_MASK_ENABLE);
}

/**
  * @brief   set valid dma mask disable
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_L | VALID_DMA_MASK
  *
  * @param  PDMx PDMx instance
  * @retval None.
  */
__STATIC_INLINE void ll_pdm_left_disable_dma_mask(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->DATA_L, PDM_DATA_L_VALID_DMA_MASK,PDM_DATA_L_VALID_DMA_MASK_DISABLE);
}

/**
  * @brief   set valid dma mask enable
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_R | VALID_DMA_MASK
  *
  * @param  PDMx PDMx instance
  * @retval None.
  */
__STATIC_INLINE void ll_pdm_right_enable_dma_mask(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->DATA_R, PDM_DATA_R_VALID_DMA_MASK,PDM_DATA_R_VALID_DMA_MASK_ENABLE);
}

/**
  * @brief   set valid dma mask disable
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_R | VALID_DMA_MASK
  *
  * @param  PDMx PDMx instance
  * @retval None.
  */
__STATIC_INLINE void ll_pdm_right_disable_dma_mask(pdm_regs_t *PDMx)
{
    MODIFY_REG(PDMx->DATA_R, PDM_DATA_R_VALID_DMA_MASK,PDM_DATA_R_VALID_DMA_MASK_DISABLE);
}

/** @} */

/** @defgroup PDM_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize the PDM registers to their default reset values.
  * @param  PDMx instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: PDM registers are de-initialized
  *          - ERROR: PDM registers are not de-initialized
  */
error_status_t ll_pdm_deinit(pdm_regs_t *PDMx);

/**
  * @brief  Initialize the PDM registers according to the specified parameters in p_pdm_init.
  * @param  PDMx instance
  * @param  p_pdm_init pointer to a @ref ll_pdm_init_t structure.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: PDM registers are initialized
  *          - ERROR: Not applicable
  */
error_status_t ll_pdm_init(pdm_regs_t *PDMx, ll_pdm_init_t *p_pdm_init);

/**
  * @brief Set each field of a @ref ll_pdm_init_t type structure to default value.
  * @param p_pdm_init  Pointer to a @ref ll_pdm_init_t structure
  *                        whose fields will be set to default values.
  * @retval None
  */
void ll_pdm_struct_init(ll_pdm_init_t *p_pdm_init);

/** @} */

/** @} */

#endif /* PDM */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_PDM_H__ */

/** @} */

/** @} */

/** @} */

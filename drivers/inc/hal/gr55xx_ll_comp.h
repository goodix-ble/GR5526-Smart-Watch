/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_comp.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of COMP LL library.
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

/** @defgroup LL_COMP COMP
  * @brief COMP LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_COMP_H__
#define __GR55XX_LL_COMP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined(AON_CTL)

/** @defgroup COMP_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup COMP_LL_ES_INIT COMP Exported init structures
  * @{
  */

/**
  * @brief LL COMP init Structure definition
  */
typedef struct _ll_comp_init
{
    uint32_t input_source;  /**< Specifies the input source for the comparator.
                                 This parameter can be any value of @ref COMP_LL_EC_INPUT_SRC.
                                 This parameter can be modified afterwards using unitary function @ref ll_comp_set_input_src(). */

    uint32_t ref_source;    /**< Specifies the reference source for the comparator.
                                 This parameter can be any value of @ref COMP_LL_EC_INPUT_SRC.
                                 This parameter can be modified afterwards using unitary function @ref ll_comp_set_ref_src(). */

    uint32_t ref_value;     /*!< Specifies the value of the COMP buffered reference.
                                 If ref_source select to LL_COMP_REF_SRC_VBAT, this parameter can be a value between: 0 ~ 7.
                                 This parameter can be modified afterwards using unitary function @ref ll_comp_set_vbatt_lvl().
                                 If ref_source select to LL_COMP_REF_SRC_VREF, this parameter can be a value between: 0 ~ 255.
                                 This parameter can be modified afterwards using unitary function @ref ll_comp_set_vref_lvl(). */

    uint32_t hyst;          /**< Specifies the hysteresis for the comparator.
                                 This parameter can be modified afterwards using unitary function ll_comp_positive_hysteresis() and ll_comp_negative_hysteresis(). */

    uint32_t edge;         /**< Specifies the wakeup edge of the comparator.
                                 This parameter can be any value of @ref COMP_LL_EC_WAKEUP_EDGE. */

    uint32_t res_deg;        /**< Specifies the calibration for the comparator.
                                 This parameter can be modified afterwards using unitary function ll_comp_positive_degeneration() and ll_comp_negative_degeneration(). */

} ll_comp_init_t;

/** @} */

/** @} */

/**
  * @defgroup  COMP_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup COMP_LL_Exported_Constants COMP Exported Constants
  * @{
  */

/** @defgroup COMP_LL_EC_INPUT_SRC COMP INPUT SOURCE
  * @{
  */
#define LL_COMP_INPUT_SRC_IO0         (0UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Pos)    /**< Set MSIO_0 as inputs for the comparator */
#define LL_COMP_INPUT_SRC_IO1         (1UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Pos)    /**< Set MSIO_1 as inputs for the comparator */
#define LL_COMP_INPUT_SRC_IO2         (2UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Pos)    /**< Set MSIO_2 as inputs for the comparator */
#define LL_COMP_INPUT_SRC_IO3         (3UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Pos)    /**< Set MSIO_3 as inputs for the comparator */
#define LL_COMP_INPUT_SRC_IO4         (4UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Pos)    /**< Set MSIO_4 as inputs for the comparator */
#define LL_COMP_INPUT_SRC_IO5         (5UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Pos)    /**< Set MSIO_5 as inputs for the comparator */
#define LL_COMP_INPUT_SRC_IO6         (6UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Pos)    /**< Set MSIO_6 as inputs for the comparator */
#define LL_COMP_INPUT_SRC_IO7         (7UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Pos)    /**< Set MSIO_7 as inputs for the comparator */

#define LL_COMP_INPUT_SRC_VBAT          (9UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Pos)    /**< Set VBATT as inputs for the comparator  */
#define LL_COMP_INPUT_SRC_VREF          (10UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Pos)    /**< Set VREF as inputs for the comparator   */

/** @} */

/** @defgroup COMP_LL_EC_REF_SRC COMP REF SOURCE
  * @{
  */
#define LL_COMP_REF_SRC_IO0           (0UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Pos)    /**< Set MSIO_0 as references for the comparator */
#define LL_COMP_REF_SRC_IO1           (1UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Pos)    /**< Set MSIO_1 as references for the comparator */
#define LL_COMP_REF_SRC_IO2           (2UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Pos)    /**< Set MSIO_2 as references for the comparator */
#define LL_COMP_REF_SRC_IO3           (3UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Pos)    /**< Set MSIO_3 as references for the comparator */
#define LL_COMP_REF_SRC_IO4           (4UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Pos)    /**< Set MSIO_4 as references for the comparator */
#define LL_COMP_REF_SRC_IO5           (5UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Pos)    /**< Set MSIO_5 as references for the comparator */
#define LL_COMP_REF_SRC_IO6           (6UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Pos)    /**< Set MSIO_6 as references for the comparator */
#define LL_COMP_REF_SRC_IO7           (7UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Pos)    /**< Set MSIO_7 as references for the comparator */

#define LL_COMP_REF_SRC_VBAT          (9UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Pos)    /**< Set VBATT as references for the comparator  */
#define LL_COMP_REF_SRC_VREF          (10UL << AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Pos)    /**< Set VREF as references for the comparator   */
/** @} */

/** @defgroup COMP_LL_EC_HYST_SRC COMP HYST
  * @{
  */
#define LL_COMP_HYST_POSITIVE           (1UL << AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_HYST_Pos )    /**< Set  positive side of hysteresis for the comparator */
#define LL_COMP_HYST_NEGATIVE           (1UL << AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_HYST_Pos )    /**< Set negative side of hysteresis for the comparator */

/** @} */

/** @defgroup COMP_LL_EC_WAKEUP_EDGE COMP WAKEUP EDGE
  * @{
  */
#define LL_COMP_WAKEUP_EDGE_BOTH             ( 0UL )
#define LL_COMP_WAKEUP_EDGE_FALLING          ( 1UL )
#define LL_COMP_WAKEUP_EDGE_RISING           ( 2UL )

/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup COMP_LL_Exported_Macros COMP Exported Macros
  * @{
  */

/** @defgroup COMP_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in COMP register
  * @param  __instance__ COMP instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_COMP_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG((__instance__)->__REG__, (__VALUE__))

/**
  * @brief  Read a value in COMP register
  * @param  __instance__ COMP instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_COMP_ReadReg(__instance__, __REG__) READ_REG((__instance__)->__REG__)

/** @} */

/** @} */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup COMP_LL_Private_Macros COMP Private Macros
  * @{
  */

/** @defgroup COMP_LL_EC_DEFAULT_CONFIG InitStruct default configuartion
  * @{
  */

/**
  * @brief Default configuartion for initializing structure
  */
#define LL_COMP_DEFAULT_CONFIG                     \
{                                                  \
    .channel_p  = LL_COMP_CHANNEL_IO0,             \
    .channel_n  = LL_COMP_CHANNEL_IO1,             \
}
/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup COMP_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup COMP_LL_EF_Configuration Basic Configuration
  * @{
  */

/**
  * @brief  Enable COMP module.
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP_REG_0 | COMP_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_comp_enable(void)
{
    SET_BITS(AON_PMU->COMP_REG_0, AON_PMU_COMP_REG_0_WAKE_COMP_EN_Msk);
}

/**
  * @brief  Disable COMP module.
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP_REG_0 | COMP_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_comp_disable(void)
{
    CLEAR_BITS(AON_PMU->COMP_REG_0, AON_PMU_COMP_REG_0_WAKE_COMP_EN_Msk);
}

/**
  * @brief  Set channel of COMP input source.
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP_REG_0 | AON_COMP_REG_0_CHANNEL_SEL_P
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_SRC_IO0
  *         @arg @ref LL_COMP_INPUT_SRC_IO1
  *         @arg @ref LL_COMP_INPUT_SRC_IO2
  *         @arg @ref LL_COMP_INPUT_SRC_IO3
  *         @arg @ref LL_COMP_INPUT_SRC_IO4
  *         @arg @ref LL_COMP_INPUT_SRC_IO5
  *         @arg @ref LL_COMP_INPUT_SRC_IO6
  *         @arg @ref LL_COMP_INPUT_SRC_IO7
  *         @arg @ref LL_COMP_INPUT_SRC_VBAT
  *         @arg @ref LL_COMP_INPUT_SRC_VREF
  * @retval None
  */
__STATIC_INLINE void ll_comp_set_input_src(uint32_t source)
{
    MODIFY_REG(AON_PMU->COMP_REG_0, AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Msk, source);
}

/**
  * @brief  Set channel of COMP reference source.
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP_REG_0 | AON_COMP_REG_0_CHANNEL_SEL_N
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_COMP_REF_SRC_IO0
  *         @arg @ref LL_COMP_REF_SRC_IO1
  *         @arg @ref LL_COMP_REF_SRC_IO2
  *         @arg @ref LL_COMP_REF_SRC_IO3
  *         @arg @ref LL_COMP_REF_SRC_IO4
  *         @arg @ref LL_COMP_REF_SRC_IO5
  *         @arg @ref LL_COMP_REF_SRC_IO6
  *         @arg @ref LL_COMP_REF_SRC_IO7
  *         @arg @ref LL_COMP_REF_SRC_VBAT
  *         @arg @ref LL_COMP_REF_SRC_VREF
  * @retval None
  */
__STATIC_INLINE void ll_comp_set_ref_src(uint32_t source)
{
    MODIFY_REG(AON_PMU->COMP_REG_0, AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Msk, source);
}

/**
  * @brief  Set VBATT control level.
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP_REG_0 | BATT_LVL_CTRL_LV
  *
  * @param  level This parameter can be a value between: 0 ~ 7
  *         Vbatt_ref = ((level+1)/10) * VBATT
  * @retval None
  */
__STATIC_INLINE void ll_comp_set_vbatt_lvl(uint32_t level)
{
    SET_BITS(AON_PMU->COMP_REG_1, AON_PMU_COMP_REG_1_COMP_VBAT_EN_Msk);
    MODIFY_REG(AON_PMU->COMP_REG_0, AON_PMU_COMP_REG_0_COMP_BATT_LVL_CTRL_LV_Msk, level << AON_PMU_COMP_REG_0_COMP_BATT_LVL_CTRL_LV_Pos);
}

/**
  * @brief  Set VREF control level.
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP_REG_0 | COMP_REF_CTRL
  *
  * @param  level This parameter can be a value between: 0 ~ 255
  *         Vref = 7.5mv * level
  * @retval None
  */
__STATIC_INLINE void ll_comp_set_vref_lvl(uint32_t level)
{
    SET_BITS(AON_PMU->COMP_REG_1, AON_PMU_COMP_REG_1_COMP_VREF_EN_Msk);
    MODIFY_REG(AON_PMU->COMP_REG_0, AON_PMU_COMP_REG_0_COMP_REF_CTRL_LV_Msk, level << AON_PMU_COMP_REG_0_COMP_REF_CTRL_LV_Pos);
}
/**
  * @brief  set current of comparator.
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP_REG_0 | icomp_ctrl
  *
  * @retval None
  */
__STATIC_INLINE void ll_comp_set_current(uint32_t level)
{
    MODIFY_REG(AON_PMU->COMP_REG_0, AON_PMU_COMP_REG_0_ICOMP_CTRL_LV_Msk, level << AON_PMU_COMP_REG_0_ICOMP_CTRL_LV_Pos);
}

/**
  * @brief  set power of comparator.
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP_REG_0 | cascres_half
  *
  * @retval None
  */
__STATIC_INLINE void ll_comp_cascres_half_high(uint32_t level)
{
    MODIFY_REG(AON_PMU->COMP_REG_1, AON_PMU_COMP_REG_1_COMP_CASCRES_HALF_CTRL_Msk, level <<AON_PMU_COMP_REG_1_COMP_CASCRES_HALF_CTRL_Pos);
}

/**
  * @brief  Set hysteresis comparator
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP_REG_1 | P_HYS_EN
  *
 * @param  hyst This parameter can be the following value:
  *         @arg @ref LL_COMP_HYST_POSITIVE
  *
  * @retval None
  */
__STATIC_INLINE void ll_comp_positive_hysteresis(uint32_t  hyst)
{
    MODIFY_REG(AON_PMU->COMP_REG_1, AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_HYST , hyst);
}

/**
  * @brief  Set hysteresis comparator
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP_REG_1 | N_HYS_EN
  *
  * @param  hyst This parameter can be the following value:
  *         @arg @ref LL_COMP_HYST_NEGATIVE
  *
  * @retval None
  */
__STATIC_INLINE void ll_comp_negative_hysteresis(uint32_t  hyst)
{
    MODIFY_REG(AON_PMU->COMP_REG_1, AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_HYST , hyst);
}

/**
  * @brief  Enable Clocked COMP.
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP_REG_1 | AON_PMU_COMP_REG_1_CLK_COMP_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_clocked_comp_enable(void)
{
    SET_BITS(AON_PMU->COMP_REG_1, AON_PMU_COMP_REG_1_CLK_COMP_EN_Msk);
}

/**
  * @brief  Disable Clocked COMP.
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP_REG_1 | AON_PMU_COMP_REG_1_CLK_COMP_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_clocked_comp_disable(void)
{
    CLEAR_BITS(AON_PMU->COMP_REG_1, AON_PMU_COMP_REG_1_CLK_COMP_EN_Msk);
}

/**
  * @brief Enable Wakeup Interrupt for COMP Rising.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_SLEEP_EVENT | MSIO_COMP
  *
  * @retval None.
  */
__STATIC_INLINE void ll_comp_enable_rising_wakeup(void)
{
    BIT_ADDR((uint32_t)&AON_CTL->MCU_WAKEUP_CTRL, AON_CTL_MCU_WAKEUP_CTRL_COMP_RISE_Pos) = 1;
}

/**
  * @brief Disable Wakeup Interrupt for COMP Rising.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_SLEEP_EVENT | MSIO_COMP
  *
  * @retval None.
  */
__STATIC_INLINE void ll_comp_disable_rising_wakeup(void)
{
    BIT_ADDR((uint32_t)&AON_CTL->MCU_WAKEUP_CTRL, AON_CTL_MCU_WAKEUP_CTRL_COMP_RISE_Pos) = 0;
}

/**
  * @brief Get Wakeup Interrupt for COMP Rising.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_SLEEP_EVENT | MSIO_COMP
  *
  * @retval State of bit (1 or o).
  */
__STATIC_INLINE uint32_t ll_comp_is_enable_rising_wakeup(void)
{
    return (READ_BITS(AON_CTL->MCU_WAKEUP_CTRL, AON_CTL_MCU_WAKEUP_CTRL_COMP_RISE) == AON_CTL_MCU_WAKEUP_CTRL_COMP_RISE);
}

/**
  * @brief Enable Wakeup Interrupt for COMP Falling.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_SLEEP_EVENT | MSIO_COMP
  *
  * @retval None.
  */
__STATIC_INLINE void ll_comp_enable_falling_wakeup(void)
{
    BIT_ADDR((uint32_t)&AON_CTL->MCU_WAKEUP_CTRL, AON_CTL_MCU_WAKEUP_CTRL_COMP_FALL_Pos) = 1;
}

/**
  * @brief Disable Wakeup Interrupt for COMP Falling.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_SLEEP_EVENT | MSIO_COMP
  *
  * @retval None.
  */
__STATIC_INLINE void ll_comp_disable_falling_wakeup(void)
{
    BIT_ADDR((uint32_t)&AON_CTL->MCU_WAKEUP_CTRL, AON_CTL_MCU_WAKEUP_CTRL_COMP_FALL_Pos) = 0;
}

/**
  * @brief Get Wakeup Interrupt for COMP Falling.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_SLEEP_EVENT | MSIO_COMP
  *
  * @retval State of bit (1 or o).
  */
__STATIC_INLINE uint32_t ll_comp_is_enable_falling_wakeup(void)
{
    return (READ_BITS(AON_CTL->MCU_WAKEUP_CTRL, AON_CTL_MCU_WAKEUP_CTRL_COMP_FALL) == AON_CTL_MCU_WAKEUP_CTRL_COMP_FALL);
}

/**
  * @brief  Indicate if the COMP rising_triger Flag is set or not.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_SLEEP_EVENT | MSIO_COMP
  *
  * @retval State of bit (1 or o).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_comp_is_rising_triger_flag_it(void)
{
    return (READ_BITS(AON_CTL->AON_SLP_EVENT, AON_CTL_SLP_EVENT_CMP_RISE) == AON_CTL_SLP_EVENT_CMP_RISE);
}

/**
  * @brief Clear rising_triger flag for COMP.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_SLEEP_EVENT | MSIO_COMP
  *
  * @retval None.
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_comp_clear_rising_triger_flag_it(void)
{
    WRITE_REG(AON_CTL->AON_SLP_EVENT, ~AON_CTL_SLP_EVENT_CMP_RISE);
}

/**
  * @brief  Indicate if the COMP falling_triger Flag is set or not.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_SLEEP_EVENT | MSIO_COMP
  *
  * @retval State of bit (1 or o).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_comp_is_falling_triger_flag_it(void)
{
    return (READ_BITS(AON_CTL->AON_SLP_EVENT, AON_CTL_SLP_EVENT_CMP_FALL) == AON_CTL_SLP_EVENT_CMP_FALL);
}

/**
  * @brief Clear falling_triger flag for COMP.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_SLEEP_EVENT | MSIO_COMP
  *
  * @retval None.
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_comp_clear_falling_triger_flag_it(void)
{
    WRITE_REG(AON_CTL->AON_SLP_EVENT, ~AON_CTL_SLP_EVENT_CMP_FALL);
}

/**
  * @brief Set compator glitch remove cycles
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_CTL_PMU_COMP_GLITCH_REMOVE | AON_CTL_PMU_COMP_GLITCH_REMOVE_CYCLE
  *
  * @retval None.
  */
__STATIC_INLINE void ll_comp_set_remove_cycle(uint32_t cycle)
{
    MODIFY_REG(AON_CTL->PMU_COMP_GLITCH_REMOVE, AON_CTL_PMU_COMP_GLITCH_REMOVE_CYCLE, cycle << AON_CTL_PMU_COMP_GLITCH_REMOVE_CYCLE_Pos);
}

/**
  * @brief get the glitch remove_cycle
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_CTL_PMU_COMP_GLITCH_REMOVE | AON_CTL_PMU_COMP_GLITCH_REMOVE_CYCLE
  *
  * @retval remove_cycles.
  */
__STATIC_INLINE uint32_t ll_comp_get_remove_cycle(void)
{
    return (uint32_t)(READ_BITS(AON_CTL->PMU_COMP_GLITCH_REMOVE, AON_CTL_PMU_COMP_GLITCH_REMOVE_CYCLE_Msk) >> AON_CTL_PMU_COMP_GLITCH_REMOVE_CYCLE_Pos);
}

/** @} */

/** @defgroup COMP_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize COMP registers (Registers restored to their default values).
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: COMP registers are de-initialized
  *          - ERROR: COMP registers are not de-initialized
  */
error_status_t ll_comp_deinit(void);

/**
  * @brief  Initialize COMP registers according to the specified.
  *         parameters in p_comp_init.
  * @param  p_comp_init Pointer to a ll_comp_init_t structure that contains the configuration
  *                             information for the specified COMP peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: COMP registers are initialized according to p_comp_init content
  *          - ERROR: Problem occurred during COMP Registers initialization
  */
error_status_t ll_comp_init(ll_comp_init_t *p_comp_init);

/**
  * @brief Set each field of a @ref ll_comp_init_t type structure to default value.
  * @param p_comp_init  Pointer to a @ref ll_comp_init_t structure
  *                             whose fields will be set to default values.
  * @retval None
  */
void ll_comp_struct_init(ll_comp_init_t *p_comp_init);

/** @} */

/** @} */

#endif /* AON */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_COMP_H__ */

/** @} */

/** @} */

/** @} */

/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_adc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of ADC LL library.
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

/** @defgroup LL_ADC ADC
  * @brief ADC LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_ADC_H__
#define __GR55XX_LL_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined(AON_CTL)

/** @defgroup LL_ADC_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup LL_ADC_ES_INIT ADC Exported init structures
  * @{
  */

/**
  * @brief LL ADC init Structure definition
  */
typedef struct _ll_adc_init
{
    uint32_t channel_p;     /**< Specifies the input source to ADC channel P.
                                 This parameter can be any value of @ref LL_ADC_EC_INPUT_SRC.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_channelp(). */

    uint32_t channel_n;     /**< Specifies the input source to ADC channel N.
                                 This parameter can be any value of @ref LL_ADC_EC_INPUT_SRC.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_channeln(). */

    uint32_t input_mode;    /**< Specifies the operation mode for the ADC sample.
                                 This parameter can be a value of @ref LL_ADC_EC_INPUT_MODE.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_input_mode(). */

    uint32_t ref_source;    /**< Specifies the source of the ADC reference.
                                 This parameter can be a value of @ref LL_ADC_EC_REFERENCE_SRC.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_ref().*/

    uint32_t ref_value;     /*!< Specifies the value of the ADC buffered reference.
                                 This parameter can be a value of @ref LL_ADC_EC_REFERENCE.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_ref_value().*/

    uint32_t clock;         /**< Specifies the clock of ADC.
                                 This parameter can be a value of @ref LL_ADC_EC_CLK.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_clock().*/

} ll_adc_init_t;

/** @} */

/** @} */

/**
  * @defgroup  LL_ADC_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup LL_ADC_Exported_Constants ADC Exported Constants
  * @{
  */

/** @defgroup LL_ADC_EC_CLK ADC CLOCK
  * @{
  */
#if defined(GR552xx)
#define LL_ADC_CLK_16M              (4UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 16 MHz  */
#define LL_ADC_CLK_8M               (5UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 8 MHz  */
#define LL_ADC_CLK_4M               (6UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 4 MHz  */
#define LL_ADC_CLK_1M               (7UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 1 MHz  */
#define LL_ADC_CLK_16K              (1UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 16KHz  */
#define LL_ADC_CLK_8K               (2UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 8KHz  */
#define LL_ADC_CLK_4K               (3UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 4KHz  */
#define LL_ADC_CLK_NONE             (0UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< Close Clock*/
#endif
/** @} */

/** @defgroup LL_ADC_EC_REFERENCE ADC Buffered Internal Reference Value
  * @{
  */
#define LL_ADC_REF_VALUE_0P8        (0x3UL << AON_PMU_SNSADC_CFG_REF_VALUE_Pos)   /**< Reference = 0.85 V */
#define LL_ADC_REF_VALUE_1P2        (0x7UL << AON_PMU_SNSADC_CFG_REF_VALUE_Pos)   /**< Reference = 1.28 V */
#define LL_ADC_REF_VALUE_1P6        (0xAUL << AON_PMU_SNSADC_CFG_REF_VALUE_Pos)   /**< Reference = 1.60 V */
#define LL_ADC_REF_VALUE_2P0        (0xFUL << AON_PMU_SNSADC_CFG_REF_VALUE_Pos)   /**< Reference = 2.00 V */
/** @} */

/** @defgroup LL_ADC_EC_INPUT_MODE ADC Input Mode
  * @{
  */
#define LL_ADC_INPUT_SINGLE         (1UL << AON_PMU_SNSADC_CFG_SINGLE_EN_Pos)     /**< Single ended mode */
#define LL_ADC_INPUT_DIFFERENTIAL   (0x00000000UL)                            /**< Differential mode */
/** @} */

/** @defgroup LL_ADC_EC_INPUT_SRC ADC Input Source
  * @{
  */
#define LL_ADC_INPUT_SRC_IO0        (0UL)  /**< Select MSIO0 as input       */
#define LL_ADC_INPUT_SRC_IO1        (1UL)  /**< Select MSIO1 as input       */
#define LL_ADC_INPUT_SRC_IO2        (2UL)  /**< Select MSIO2 as input       */
#define LL_ADC_INPUT_SRC_IO3        (3UL)  /**< Select MSIO3 as input       */
#define LL_ADC_INPUT_SRC_IO4        (4UL)  /**< Select MSIO4 as input       */
#define LL_ADC_INPUT_SRC_IO5        (5UL)  /**< Select MSIO5 as input       */
#define LL_ADC_INPUT_SRC_IO6        (6UL)  /**< Select MSIO6 as input       */
#define LL_ADC_INPUT_SRC_IO7        (7UL)  /**< Select MSIO7 as input       */
#define LL_ADC_INPUT_SRC_TMP        (13UL)  /**< Select temperature as input */
#define LL_ADC_INPUT_SRC_BAT        (14UL)  /**< Select Vbattery as input    */
#define LL_ADC_INPUT_SRC_REF        (15UL)  /**< Select reference as input   */

/** @} */

/** @defgroup LL_ADC_EC_REFERENCE_SRC ADC Reference Source
  * @{
  */
#if defined(GR552xx)
#define LL_ADC_REF_SRC_BUF_INT      (0x00000000UL)                            /**< Select buffered internal reference as reference   */
#define LL_ADC_REF_SRC_IO0          (3UL << AON_PMU_SNSADC_CFG_REF_SEL_Pos)       /**< Select MSIO0 as reference                         */
#define LL_ADC_REF_SRC_IO1          (4UL << AON_PMU_SNSADC_CFG_REF_SEL_Pos)       /**< Select MSIO1 as reference                         */
#define LL_ADC_REF_SRC_IO2          (5UL << AON_PMU_SNSADC_CFG_REF_SEL_Pos)       /**< Select MSIO2 as reference                         */
#define LL_ADC_REF_SRC_IO3          (6UL << AON_PMU_SNSADC_CFG_REF_SEL_Pos)       /**< Select MSIO3 as reference                         */
#endif
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup LL_ADC_Exported_Macros ADC Exported Macros
  * @{
  */

/** @defgroup LL_ADC_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in ADC register
  * @param  __instance__ ADC instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_ADC_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG((__instance__)->__REG__, (__VALUE__))

/**
  * @brief  Read a value in ADC register
  * @param  __instance__ ADC instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_ADC_ReadReg(__instance__, __REG__) READ_REG((__instance__)->__REG__)

/** @} */

/** @} */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup LL_ADC_Private_Macros ADC Private Macros
  * @{
  */

/** @defgroup LL_ADC_EC_DEFAULT_CONFIG InitStruct default configuartion
  * @{
  */

/**
  * @brief LL ADC InitStrcut default configuartion
  */
#if defined(GR552xx)
#define LL_ADC_DEFAULT_CONFIG                      \
{                                                  \
    .channel_p  = LL_ADC_INPUT_SRC_IO0,            \
    .channel_n  = LL_ADC_INPUT_SRC_IO1,            \
    .input_mode = LL_ADC_INPUT_DIFFERENTIAL,       \
    .ref_source = LL_ADC_REF_SRC_BUF_INT,          \
    .ref_value  = LL_ADC_REF_VALUE_1P2,            \
    .clock      = LL_ADC_CLK_16M                   \
}
#else
#define LL_ADC_DEFAULT_CONFIG                      \
{                                                  \
    .channel_p  = LL_ADC_INPUT_SRC_IO0,            \
    .channel_n  = LL_ADC_INPUT_SRC_IO1,            \
    .input_mode = LL_ADC_INPUT_DIFFERENTIAL,       \
    .ref_source = LL_ADC_REF_SRC_BUF_INT,          \
    .ref_value  = LL_ADC_REF_VALUE_1P2,            \
    .clock      = LL_ADC_CLK_16                    \
}
#endif
/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup LL_ADC_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup LL_ADC_EF_Configuration Basic Configuration
  * @{
  */

/**
  * @brief  Enable ADC module.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable(void)
{
#if defined(GR552xx)
    SET_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_EN_Msk);
#endif
}

/**
  * @brief  Disable ADC module.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable(void)
{
#if defined(GR552xx)
    CLEAR_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_EN_Msk);
#endif
}

/**
  * @brief  Check if ADC module is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled(void)
{
#if defined(GR552xx)
    return (READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_EN_Msk) == (AON_PMU_SNSADC_CFG_EN_Msk));
#endif
}

/**
  * @brief  Disable ADC clock.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_1 | ADC_CLK_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_clock(void)
{
#if defined(GR552xx)
    MODIFY_REG(MCU_SUB->SENSE_ADC_CLK, MCU_SUB_SNSADC_CLK_WR, MCU_SUB_SNSADC_CLK_NONE);
#endif
}

/**
  * @brief  Check if ADC clock is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_1 | ADC_CLK_EN
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_clock(void)
{
#if defined(GR552xx)
    return (READ_BITS(MCU_SUB->SENSE_ADC_CLK, MCU_SUB_SNSADC_CLK_RD) != 0);
#endif
}

/**
  * @brief  Set ADC clock source.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_1 | ADC_CLK_SEL
  *
  * @param  clk This parameter can be one of the following values:
  *         @arg @ref LL_ADC_CLK_16M
  *         @arg @ref LL_ADC_CLK_8M
  *         @arg @ref LL_ADC_CLK_4M
  *         @arg @ref LL_ADC_CLK_1M
  *         @arg @ref LL_ADC_CLK_16K
  *         @arg @ref LL_ADC_CLK_8K
  *         @arg @ref LL_ADC_CLK_4K
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_clock(uint32_t clk)
{
#if defined(GR552xx)
    MODIFY_REG(MCU_SUB->SENSE_ADC_CLK, MCU_SUB_SNSADC_CLK_WR, clk);
#endif
}

/**
  * @brief  Return source for ADC clock.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_1 | ADC_CLK_SEL
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_CLK_16M
  *         @arg @ref LL_ADC_CLK_8M
  *         @arg @ref LL_ADC_CLK_4M
  *         @arg @ref LL_ADC_CLK_1M
  *         @arg @ref LL_ADC_CLK_16K
  *         @arg @ref LL_ADC_CLK_8K
  *         @arg @ref LL_ADC_CLK_4K
  */
__STATIC_INLINE uint32_t ll_adc_get_clock(void)
{
#if defined(GR552xx)
     return (uint32_t)(READ_BITS(MCU_SUB->SENSE_ADC_CLK, MCU_SUB_SNSADC_CLK_RD) >> MCU_SUB_SNSADC_CLK_RD_Pos);
#endif
}

/**
  * @brief  Set ADC bias reference.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG1
  *
  * @param  value This parameter can be one of the following values:
  *         @arg @ref LL_ADC_REF_VALUE_0P8
  *         @arg @ref LL_ADC_REF_VALUE_1P2
  *         @arg @ref LL_ADC_REF_VALUE_1P6
  *         @arg @ref LL_ADC_REF_VALUE_2P0
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_ref_value(uint32_t value)
{
#if defined(GR552xx)
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_REF_VALUE_Msk, value);
#endif
}

/**
  * @brief  Return ADC bias reference.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG1
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_REF_VALUE_0P8
  *         @arg @ref LL_ADC_REF_VALUE_1P2
  *         @arg @ref LL_ADC_REF_VALUE_1P6
  *         @arg @ref LL_ADC_REF_VALUE_2P0
  */
__STATIC_INLINE uint32_t ll_adc_get_ref_value(void)
{
#if defined(GR552xx)
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_REF_VALUE_Msk) >> AON_PMU_SNSADC_CFG_REF_VALUE_Pos);
#endif
}

/**
  * @brief  Enable temperature sensor.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_temp(void)
{
#if defined(GR552xx)
    SET_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_TEMP_EN_Msk);
#endif
}

/**
  * @brief  Disable temperature sensor.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_temp(void)
{
#if defined(GR552xx)
    CLEAR_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_TEMP_EN_Msk);
#endif
}

/**
  * @brief  Check if temperature sensor is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_temp(void)
{
#if defined(GR552xx)
    return (READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_TEMP_EN_Msk) == (AON_PMU_SNSADC_CFG_TEMP_EN_Msk));
#endif
}

/**
  * @brief  Enable Vbattery sensor.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_vbat(void)
{
#if defined(GR552xx)
    SET_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_VBAT_EN_Msk);
#endif
}

/**
  * @brief  Disable Vbattery sensor.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_vbat(void)
{
#if defined(GR552xx)
    CLEAR_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_VBAT_EN_Msk);
#endif
}

/**
  * @brief  Check if Vbattery sensor is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_vbat(void)
{
#if defined(GR552xx)
    return (READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_VBAT_EN_Msk) == (AON_PMU_SNSADC_CFG_VBAT_EN_Msk));
#endif
}

/**
  * @brief  Set ADC input mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SINGLE
  *         @arg @ref LL_ADC_INPUT_DIFFERENTIAL
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_input_mode(uint32_t mode)
{
#if defined(GR552xx)
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_SINGLE_EN_Msk, mode);
#endif
}

/**
  * @brief  Return ADC input mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SINGLE
  *         @arg @ref LL_ADC_INPUT_DIFFERENTIAL
  */
__STATIC_INLINE uint32_t ll_adc_get_input_mode(void)
{
#if defined(GR552xx)
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_SINGLE_EN_Msk) >> AON_PMU_SNSADC_CFG_SINGLE_EN_Pos);
#endif
}

/**
  * @brief  Enable offset calibration.
  * @note   Enable offset calibration, used to swap inputs of comparator for offset
  *         calibration.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_ofs_cal(void)
{
#if defined(GR552xx)
    SET_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_OFS_CAL_EN_Msk);
#endif
}

/**
  * @brief  Disable offset calibration.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_ofs_cal(void)
{
#if defined(GR552xx)
    CLEAR_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_OFS_CAL_EN_Msk);
#endif
}

/**
  * @brief  Check if offset calibration is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_ofs_cal(void)
{
#if defined(GR552xx)
    return (READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_OFS_CAL_EN_Msk) == (AON_PMU_SNSADC_CFG_OFS_CAL_EN_Msk));
#endif
}

/**
  * @brief  Set dynamic rang of ADC.
  * @note   When higher input signal frequencies close to Nyquist rate, you should set 1.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @param  rang This parameter can be a value between: 1 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_dynamic_rang(uint32_t rang)
{
#if defined(GR552xx)
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_DYMAMIC_Msk, (rang & 0x7) << AON_PMU_SNSADC_CFG_DYMAMIC_Pos);
#endif
}

/**
  * @brief  Return ADC dynamic rang.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval Returned value can be a value between: 1 ~ 7
  */
__STATIC_INLINE uint32_t ll_adc_get_dynamic_rang(void)
{
#if defined(GR552xx)
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_DYMAMIC_Msk) >> AON_PMU_SNSADC_CFG_DYMAMIC_Pos);
#endif
}

/**
  * @brief  Set source of ADC input channelP.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG3
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SRC_IO0
  *         @arg @ref LL_ADC_INPUT_SRC_IO1
  *         @arg @ref LL_ADC_INPUT_SRC_IO2
  *         @arg @ref LL_ADC_INPUT_SRC_IO3
  *         @arg @ref LL_ADC_INPUT_SRC_IO4
  *         @arg @ref LL_ADC_INPUT_SRC_TMP
  *         @arg @ref LL_ADC_INPUT_SRC_BAT
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_channelp(uint32_t source)
{
#if defined(GR552xx)
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_CHN_P_Msk, source << AON_PMU_SNSADC_CFG_CHN_P_Pos);
#endif
}

/**
  * @brief  Return source of ADC input channelP.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG3
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SRC_IO0
  *         @arg @ref LL_ADC_INPUT_SRC_IO1
  *         @arg @ref LL_ADC_INPUT_SRC_IO2
  *         @arg @ref LL_ADC_INPUT_SRC_IO3
  *         @arg @ref LL_ADC_INPUT_SRC_IO4
  *         @arg @ref LL_ADC_INPUT_SRC_TMP
  *         @arg @ref LL_ADC_INPUT_SRC_BAT
  */
__STATIC_INLINE uint32_t ll_adc_get_channelp(void)
{
#if defined(GR552xx)
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_CHN_P_Msk) >> AON_PMU_SNSADC_CFG_CHN_P_Pos);
#endif
}

/**
  * @brief  Set source of ADC input channelN.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG3
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SRC_IO0
  *         @arg @ref LL_ADC_INPUT_SRC_IO1
  *         @arg @ref LL_ADC_INPUT_SRC_IO2
  *         @arg @ref LL_ADC_INPUT_SRC_IO3
  *         @arg @ref LL_ADC_INPUT_SRC_IO4
  *         @arg @ref LL_ADC_INPUT_SRC_TMP
  *         @arg @ref LL_ADC_INPUT_SRC_BAT
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_channeln(uint32_t source)
{
#if defined(GR552xx)
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_CHN_N_Msk, source << AON_PMU_SNSADC_CFG_CHN_N_Pos);
#endif
}

/**
  * @brief  Return source of ADC input channelN.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG3
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SRC_IO0
  *         @arg @ref LL_ADC_INPUT_SRC_IO1
  *         @arg @ref LL_ADC_INPUT_SRC_IO2
  *         @arg @ref LL_ADC_INPUT_SRC_IO3
  *         @arg @ref LL_ADC_INPUT_SRC_IO4
  *         @arg @ref LL_ADC_INPUT_SRC_TMP
  *         @arg @ref LL_ADC_INPUT_SRC_BAT
  */
__STATIC_INLINE uint32_t ll_adc_get_channeln(void)
{
#if defined(GR552xx)
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_CHN_N_Msk) >> AON_PMU_SNSADC_CFG_CHN_N_Pos);
#endif
}

/**
  * @brief  Enable ADC MAS_RST.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_mas_rst(void)
{
#if defined(GR552xx)
    SET_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_MAS_RST_Msk);
#endif
}

/**
  * @brief  Disable ADC MAS_RST.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_mas_rst(void)
{
#if defined(GR552xx)
    CLEAR_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_MAS_RST_Msk);
#endif
}

/**
  * @brief  Check if ADC MAS_RST is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_mas_rst(void)
{
#if defined(GR552xx)
    return (READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_MAS_RST_Msk) == (AON_PMU_SNSADC_CFG_MAS_RST_Msk));
#endif
}

/**
  * @brief  Set source of ADC reference.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_ADC_REF_SRC_BUF_INT
  *         @arg @ref LL_ADC_REF_SRC_IO0
  *         @arg @ref LL_ADC_REF_SRC_IO1
  *         @arg @ref LL_ADC_REF_SRC_IO2
  *         @arg @ref LL_ADC_REF_SRC_IO3
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_ref(uint32_t source)
{
#if defined(GR552xx)
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_REF_SEL_Msk, source);
#endif
}

/**
  * @brief  Return source of ADC reference.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_REF_SRC_BUF_INT
  *         @arg @ref LL_ADC_REF_SRC_IO0
  *         @arg @ref LL_ADC_REF_SRC_IO1
  *         @arg @ref LL_ADC_REF_SRC_IO2
  *         @arg @ref LL_ADC_REF_SRC_IO3
  */
__STATIC_INLINE uint32_t ll_adc_get_ref(void)
{
#if defined(GR552xx)
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_REF_SEL_Msk) >> AON_PMU_SNSADC_CFG_REF_SEL_Pos);
#endif
}

/**
  * @brief  Set current of ADC reference circuit.
  * @note   When samples at 100kbps, you should set 0.
  *         When samples at 1mbps, you should set 7.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @param  source This parameter can be a value between: 0 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_ref_current(uint32_t source)
{
#if defined(GR552xx)
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_REF_HP_Msk, (source & 0x7) << AON_PMU_SNSADC_CFG_REF_HP_Pos);
#endif
}

/**
  * @brief  Return current of ADC reference circuit.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval Returned value can be a value between: 0 ~ 7
  */
__STATIC_INLINE uint32_t ll_adc_get_ref_current(void)
{
#if defined(GR552xx)
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_REF_HP_Msk) >> AON_PMU_SNSADC_CFG_REF_HP_Pos);
#endif
}

/** @} */

/** @defgroup LL_ADC_EF_FIFO_Access FIFO Access
  * @{
  */

/**
  * @brief  Return samples value of ADC by reading FIFO.
  * @note   There are two value in the register, both of them is 16bits.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_FIFO | SENSE_ADC_FIFO
  *
  * @retval Smaples value of input
  */
__STATIC_INLINE uint32_t ll_adc_read_fifo(void)
{
    return (uint32_t)(READ_REG(MCU_SUB->SENSE_ADC_FIFO));
}

/**
  * @brief  Set threshold of ADC FIFO.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_FF_THRESH | SENSE_FF_THRESH
  *
  * @param  thresh This parameter can be a value between: 0 ~ 64
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_thresh(uint32_t thresh)
{
    MODIFY_REG(MCU_SUB->SENSE_FF_THRESH, MCU_SUB_SNSADC_FF_THRESH, (thresh & 0x3F) << MCU_SUB_SNSADC_FF_THRESH_Pos);
}

/**
  * @brief  Return threshold of ADC FIFO.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_FF_THRESH | SENSE_FF_THRESH
  *
  * @retval Returned value can be a value between: 0 ~ 64
  */
__STATIC_INLINE uint32_t ll_adc_get_thresh(void)
{
    return (uint32_t)(READ_BITS(MCU_SUB->SENSE_FF_THRESH, MCU_SUB_SNSADC_FF_THRESH) >> MCU_SUB_SNSADC_FF_THRESH_Pos);
}

/**
  * @brief  Enable ADC dma_req.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_FF_THRESH | MCU_SUB_SNSADC_FF_DMA_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_dma_req(void)
{
#if defined(GR552xx)
    SET_BITS(MCU_SUB->SENSE_FF_THRESH, MCU_SUB_SNSADC_FF_DMA_EN_Msk);
#endif
}

/**
  * @brief  Disable ADC dma_req.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_FF_THRESH | MCU_SUB_SNSADC_FF_DMA_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_dma_req(void)
{
#if defined(GR552xx)
    CLEAR_BITS(MCU_SUB->SENSE_FF_THRESH, MCU_SUB_SNSADC_FF_DMA_EN_Msk);
#endif
}


/**
  * @brief  Check if dma_req is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_FF_THRESH | MCU_SUB_SNSADC_FF_DMA_EN
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_dma_req(void)
{
#if defined(GR552xx)
    return (READ_BITS(MCU_SUB->SENSE_FF_THRESH, MCU_SUB_SNSADC_FF_DMA_EN_Msk) == (MCU_SUB_SNSADC_FF_DMA_EN_Msk));
#endif
}

/**
  * @brief  Check if ADC FIFO is not empty.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_STAT | VAL
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_fifo_notempty(void)
{
    return (uint32_t)(READ_BITS(MCU_SUB->SENSE_ADC_STAT, MCU_SUB_SNSADC_STAT_VAL) == MCU_SUB_SNSADC_STAT_VAL);
}

/**
  * @brief  Return count of ADC FIFO.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_STAT | FF_COUNT
  *
  * @retval Returned value can be a value between: 0 ~ 64
  */
__STATIC_INLINE uint32_t ll_adc_get_fifo_count(void)
{
    return (uint32_t)(READ_BITS(MCU_SUB->SENSE_ADC_STAT, MCU_SUB_SNSADC_STAT_FF_COUNT) >> MCU_SUB_SNSADC_STAT_FF_COUNT_Pos);
}

/**
  * @brief  Flush ADC FIFO.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_STAT | FF_FLUSH
  *
  * @retval void
  */
__STATIC_INLINE void ll_adc_flush_fifo(void)
{
    SET_BITS(MCU_SUB->SENSE_ADC_STAT, MCU_SUB_SNSADC_STAT_FLUSH_Msk);
}

/**
  * @brief  Try to lock sw token.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_GET_TKN_SW
  *
  * @retval Returned true if sw lock adc token success; return false if sw lock adc token fail
  */
__STATIC_INLINE uint32_t ll_adc_try_lock_sw_token(void)
{
    return (uint32_t)(READ_REG(MCU_SUB->SENSE_ADC_GET_TKN_SW) == MCU_SUB_SNSADC_TKN_LOCKED_SW);
}

/**
  * @brief  Release sw token.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_RET_TKN_SW | SW_RELEASE_Msk
  *
  * @retval none
  */
__STATIC_INLINE void ll_adc_release_sw_token(void)
{
    CLEAR_BITS(MCU_SUB->SENSE_ADC_RET_TKN_SW, MCU_SUB_SNSADC_RET_TKN_SW_RELEASE_Msk);
}

/**
  * @brief  get adc token state.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_TKN_STS
  *
  * @retval Returned value from SENSE_ADC_TKN_STS reg
  */
__STATIC_INLINE uint32_t ll_adc_get_token_state(void)
{
    return READ_REG(MCU_SUB->SENSE_ADC_TKN_STS);
}

/** @} */

/** @defgroup LL_ADC_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize ADC registers (Registers restored to their default values).
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: ADC registers are de-initialized
  *          - ERROR: ADC registers are not de-initialized
  */
error_status_t ll_adc_deinit(void);

/**
  * @brief  Initialize ADC registers according to the specified.
  *         parameters in p_adc_init.
  * @param  p_adc_init Pointer to a ll_adc_init_t structure that contains the configuration
  *                             information for the specified ADC peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: ADC registers are initialized according to p_adc_init content
  *          - ERROR: Problem occurred during ADC Registers initialization
  */
error_status_t ll_adc_init(ll_adc_init_t *p_adc_init);

/**
  * @brief Set each field of a @ref ll_adc_init_t type structure to default value.
  * @param p_adc_init  Pointer to a @ref ll_adc_init_t structure
  *                             whose fields will be set to default values.
  * @retval None
  */
void ll_adc_struct_init(ll_adc_init_t *p_adc_init);

/** @} */

/** @} */

#endif /* AON */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_ADC_H__ */

/** @} */

/** @} */

/** @} */

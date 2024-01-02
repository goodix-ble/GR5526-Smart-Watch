/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_aon_gpio.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of AON GPIO LL library.
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

/** @defgroup LL_AON_GPIO AON_GPIO
  * @brief AON_GPIO LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_AON_GPIO_H__
#define __GR55XX_LL_AON_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined(AON_IO) || defined(MCU_RET)

/** @defgroup AON_GPIO_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup AON_GPIO_LL_ES_INIT AON_GPIO Exported init structures
  * @{
  */

/**
  * @brief LL AON_GPIO init Structure definition
  */
typedef struct _ll_aon_gpio_init
{
    uint32_t pin;           /**< Specifies the AON_GPIO pins to be AON_GPIO_InitStructured.
                                 This parameter can be any value of @ref AON_GPIO_LL_EC_PIN */

    uint32_t mode;          /**< Specifies the operating mode for the selected pins.
                                 This parameter can be a value of @ref AON_GPIO_LL_EC_MODE.

                                 AON_GPIO HW configuration can be modified afterwards using unitary function 
                                 @ref ll_aon_gpio_set_pin_mode(). */

    uint32_t pull;          /**< Specifies the operating Pull-up/Pull down for the selected pins.
                                 This parameter can be a value of @ref AON_GPIO_LL_EC_PULL.

                                 AON_GPIO HW configuration can be modified afterwards using unitary function 
                                 @ref ll_aon_gpio_set_pin_pull(). */

    uint32_t mux;           /*!< Specifies the Peripheral to be connected to the selected pins.
                                This parameter can be a value of @ref AON_GPIO_LL_EC_MUX.

                                AON_GPIO HW configuration can be modified afterwards using unitary function
                                @ref ll_aon_gpio_set_pin_mux(). */

    uint32_t trigger;       /**< Specifies the trigger signal active edge.
                                 This parameter can be a value of @ref AON_GPIO_LL_EC_TRIGGER. */
} ll_aon_gpio_init_t;

/** @} */

/** @} */

/**
  * @defgroup  AON_GPIO_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup AON_GPIO_LL_Exported_Constants AON_GPIO Exported Constants
  * @{
  */

/** @defgroup AON_GPIO_LL_EC_PIN PIN
  * @{
  */
#define LL_AON_GPIO_PIN_0               ((uint32_t)0x01U) /**< Select pin 0 */
#define LL_AON_GPIO_PIN_1               ((uint32_t)0x02U) /**< Select pin 1 */
#define LL_AON_GPIO_PIN_2               ((uint32_t)0x04U) /**< Select pin 2 */
#define LL_AON_GPIO_PIN_3               ((uint32_t)0x08U) /**< Select pin 3 */
#define LL_AON_GPIO_PIN_4               ((uint32_t)0x10U) /**< Select pin 4 */
#define LL_AON_GPIO_PIN_5               ((uint32_t)0x20U) /**< Select pin 5 */
#define LL_AON_GPIO_PIN_6               ((uint32_t)0x40U) /**< Select pin 6 */
#define LL_AON_GPIO_PIN_7               ((uint32_t)0x80U) /**< Select pin 7 */
#define LL_AON_GPIO_PIN_ALL             ((uint32_t)0xFFU) /**< Select all pins */
/** @} */

/** @defgroup AON_GPIO_LL_EC_MODE Mode
  * @{
  */
#define LL_AON_GPIO_MODE_INPUT          ((uint32_t)0x0U)  /**< Select input mode */
#define LL_AON_GPIO_MODE_OUTPUT         ((uint32_t)0x1U)  /**< Select output mode */
#define LL_AON_GPIO_MODE_MUX            ((uint32_t)0x2U)  /**< Select mux peripheral mode */
/** @} */

/** @defgroup AON_GPIO_LL_EC_PULL Pull Up Pull Down
  * @{
  */
#define LL_AON_GPIO_PULL_NO             LL_AON_GPIO_RE_N /**< Select I/O no pull */
#define LL_AON_GPIO_PULL_UP             LL_AON_GPIO_RTYP /**< Select I/O pull up */
#define LL_AON_GPIO_PULL_DOWN           ((uint32_t)0x0U) /**< Select I/O pull down */
/** @} */

/** @defgroup AON_GPIO_LL_EC_MUX Alternate Function
  * @{
  */
#define LL_AON_GPIO_MUX_0               ((uint32_t)0x0U) /*!< Select alternate function 0 */
#define LL_AON_GPIO_MUX_1               ((uint32_t)0x1U) /*!< Select alternate function 1 */
#define LL_AON_GPIO_MUX_2               ((uint32_t)0x2U) /*!< Select alternate function 2 */
#define LL_AON_GPIO_MUX_3               ((uint32_t)0x3U) /*!< Select alternate function 3 */
#define LL_AON_GPIO_MUX_4               ((uint32_t)0x4U) /*!< Select alternate function 4 */
#define LL_AON_GPIO_MUX_5               ((uint32_t)0x5U) /*!< Select alternate function 5 */
#define LL_AON_GPIO_MUX_6               ((uint32_t)0x6U) /*!< Select alternate function 6 */
#define LL_AON_GPIO_MUX_7               ((uint32_t)0x7U) /*!< Select alternate function 7 */
#define LL_AON_GPIO_MUX_8               ((uint32_t)0x8U) /*!< Select alternate function 8 */
/** @} */


/** @defgroup AON_GPIO_LL_EC_TRIGGER Interrupt Trigger
  * @{
  */
#define LL_AON_GPIO_TRIGGER_NONE        ((uint32_t)0x00U) /**< No Trigger Mode */
#define LL_AON_GPIO_TRIGGER_RISING      ((uint32_t)0x01U) /**< Trigger Rising Mode */
#define LL_AON_GPIO_TRIGGER_FALLING     ((uint32_t)0x02U) /**< Trigger Falling Mode */
#define LL_AON_GPIO_TRIGGER_HIGH        ((uint32_t)0x03U) /**< Trigger High Mode */
#define LL_AON_GPIO_TRIGGER_LOW         ((uint32_t)0x04U) /**< Trigger Low Mode */
#define LL_AON_GPIO_TRIGGER_BOTH_EDGE   ((uint32_t)0x05U) /**< Trigger both edge */
/** @} */


/** @defgroup AON_GPIO_LL_CLOCK_OUT Select clock output via AON_GPIO_4
  * @{
  */
#define LL_CLK_RNG_OSC_32K              ((uint32_t)0x00U) /**< RNG_OSC_32K */
#define LL_CLK_RNG_OSC_2MHZ             ((uint32_t)0x01U) /**< RNG_OSC_2MHz */
#define LL_CLK_RC_OSC_CLK               ((uint32_t)0x02U) /**< RC_OSC_CLK */
#define LL_CLK_RTC_CLK                  ((uint32_t)0x03U) /**< RTC_CLK */
/** @} */


/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup AON_GPIO_LL_Exported_Macros AON_GPIO Exported Macros
  * @{
  */

/** @defgroup AON_GPIO_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in AON_GPIO register
  * @param  __instance__ AON_GPIO instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_AON_GPIO_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in AON_GPIO register
  * @param  __instance__ AON_GPIO instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_AON_GPIO_ReadReg(__instance__, __REG__) READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup AON_GPIO_LL_Private_Macros AON_GPIO Private Macros
  * @{
  */
/** @defgroup AON_GPIO_LL_PM_RESISTOR Resistor Enable
  * @{
  */
#define LL_AON_GPIO_RE_N_Pos            AON_IO_AON_PAD_CTRL0_R_EN_POS           /**< Resistor Enable bits position */
#define LL_AON_GPIO_RE_N_Msk            (0x1U << LL_AON_GPIO_RE_N_Pos)          /**< Resistor Enable bits mask     */
#define LL_AON_GPIO_RE_N                LL_AON_GPIO_RE_N_Msk                    /**< Resistor Enable bits          */
/** @} */

/** @defgroup AON_GPIO_LL_PM_RESISTOR_TYPE Resistor Type
  * @{
  */
#define LL_AON_GPIO_RTYP_Pos            AON_IO_AON_PAD_CTRL0_R_TYPE_POS         /**< Resistor Type bits position */
#define LL_AON_GPIO_RTYP_Msk            (0x1U << LL_AON_GPIO_RTYP_Pos)          /**< Resistor Type bits mask     */
#define LL_AON_GPIO_RTYP                LL_AON_GPIO_RTYP_Msk                    /**< Resistor Type bits          */
/** @} */

/** @defgroup AON_GPIO_LL_EC_DEFAULT_CONFIG InitStruct default configuartion
  * @{
  */

/**
  * @brief LL AON_GPIO InitStrcut default configuartion
  */
#define LL_AON_GPIO_DEFAULT_CONFIG                      \
{                                                       \
    .pin        = LL_AON_GPIO_PIN_ALL,                  \
    .mode       = LL_AON_GPIO_MODE_INPUT,               \
    .pull       = LL_AON_GPIO_PULL_DOWN,                \
    .mux        = LL_AON_GPIO_MUX_7,                    \
    .trigger    = LL_AON_GPIO_TRIGGER_NONE,             \
}
/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup AON_GPIO_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup AON_GPIO_LL_EF_Port_Configuration Port Configuration
  * @{
  */

/**
  * @brief  Set several AON_GPIO pins to input/output mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL1 | AON_GPO_OE_N
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_AON_GPIO_MODE_INPUT
  *         @arg @ref LL_AON_GPIO_MODE_OUTPUT
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_set_pin_mode(uint32_t pin_mask, uint32_t mode)
{
    pin_mask = (pin_mask << AON_IO_AON_PAD_CTRL1_OUT_EN_POS) & AON_IO_AON_PAD_CTRL1_OUT_EN;
    MODIFY_REG(AON_IO->AON_PAD_CTRL1, pin_mask, (mode == LL_AON_GPIO_MODE_INPUT) ? pin_mask : 0);
}

/**
  * @brief  Return gpio mode for a AON_GPIO pin.
  * @note   I/O mode can be Input mode. General purpose output.
  * @note   Warning: only one pin can be passed as parameter.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL1 | AON_GPO_OE_N
  *
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_AON_GPIO_MODE_INPUT
  *         @arg @ref LL_AON_GPIO_MODE_OUTPUT
  */
__STATIC_INLINE uint32_t ll_aon_gpio_get_pin_mode(uint32_t pin)
{
    pin = (pin << AON_IO_AON_PAD_CTRL1_OUT_EN_POS) & AON_IO_AON_PAD_CTRL1_OUT_EN;
    return ((uint32_t)(READ_BITS(AON_IO->AON_PAD_CTRL1, pin) == pin) ?
            LL_AON_GPIO_MODE_INPUT : LL_AON_GPIO_MODE_OUTPUT);
}

/**
  * @brief  Configure gpio pull-up or pull-down for a dedicated AON_GPIO pin.
  * @note   Warning: only one pin can be passed as parameter.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL0 | GPO_RE_N
  *  AON_PAD_CTL0 | GPO_RTYPE
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @param  pull This parameter can be one of the following values:
  *         @arg @ref LL_AON_GPIO_PULL_NO
  *         @arg @ref LL_AON_GPIO_PULL_UP
  *         @arg @ref LL_AON_GPIO_PULL_DOWN
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_set_pin_pull(uint32_t pin_mask, uint32_t pull)
{
    uint32_t RTypeMask = (pin_mask << AON_IO_AON_PAD_CTRL0_R_TYPE_POS) & AON_IO_AON_PAD_CTRL0_R_TYPE;
    uint32_t REnMask = (pin_mask << AON_IO_AON_PAD_CTRL0_R_EN_POS) & AON_IO_AON_PAD_CTRL0_R_EN;
    uint32_t RType = (pull == LL_AON_GPIO_PULL_UP) ? RTypeMask : 0x0000U;
    uint32_t REn = (pull == LL_AON_GPIO_PULL_NO) ? REnMask : 0x0000U;
    MODIFY_REG(AON_IO->AON_PAD_CTRL0, REnMask | RTypeMask, REn | RType);
}

/**
  * @brief  Return gpio pull-up or pull-down for a dedicated AON_GPIO pin.
  * @note   Warning: only one pin can be passed as parameter.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL0 | GPO_RE_N
  *  AON_PAD_CTL0 | GPO_RTYPE
  *
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_AON_GPIO_PULL_NO
  *         @arg @ref LL_AON_GPIO_PULL_UP
  *         @arg @ref LL_AON_GPIO_PULL_DOWN
  */
__STATIC_INLINE uint32_t ll_aon_gpio_get_pin_pull(uint32_t pin)
{
    uint32_t RTypeMask = (pin << AON_IO_AON_PAD_CTRL0_R_TYPE_POS) & AON_IO_AON_PAD_CTRL0_R_TYPE;
    uint32_t REnMask = (pin << AON_IO_AON_PAD_CTRL0_R_EN_POS) & AON_IO_AON_PAD_CTRL0_R_EN;
    return ((READ_BITS(AON_IO->AON_PAD_CTRL0, REnMask) != RESET) ? LL_AON_GPIO_PULL_NO :
            ((READ_BITS(AON_IO->AON_PAD_CTRL0, RTypeMask) != RESET) ? LL_AON_GPIO_PULL_UP : LL_AON_GPIO_PULL_DOWN));
}

/**
  * @brief  Configure gpio pinmux number of a dedicated pin from 0 to 7 for a dedicated port.
  * @note   Possible values are from AF0 to AF15 depending on target.
  * @note   Warning: only one pin can be passed as parameter.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_MUX_CTRL | CTRL0_7
  *  AON_PAD_CTL_0    |  MCU_OVR
  *
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  * @param  mux This parameter can be one of the following values:
  *         @arg @ref LL_AON_GPIO_MUX_0
  *         @arg @ref LL_AON_GPIO_MUX_1
  *         @arg @ref LL_AON_GPIO_MUX_2
  *         @arg @ref LL_AON_GPIO_MUX_3
  *         @arg @ref LL_AON_GPIO_MUX_4
  *         @arg @ref LL_AON_GPIO_MUX_5
  *         @arg @ref LL_AON_GPIO_MUX_6
  *         @arg @ref LL_AON_GPIO_MUX_7
  *         @arg @ref LL_AON_GPIO_MUX_8
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_set_pin_mux(uint32_t pin, uint32_t mux)
{
    uint32_t pos = POSITION_VAL(pin) << 2;
    if(LL_AON_GPIO_MUX_7 == mux)
    {
        CLEAR_BITS(AON_IO->AON_MCU_OVR, pin << AON_IO_AON_MCU_OVR_OVR_POS);
    }
    else
    {
        MODIFY_REG(MCU_RET->AON_PAD_MUX_CTL, 0xF << pos, mux << pos);
        SET_BITS(AON_IO->AON_MCU_OVR, pin << AON_IO_AON_MCU_OVR_OVR_POS);
    }
}

/**
  * @brief  Return gpio alternate function of a dedicated pin from 0 to 7 for a dedicated port.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_MUX_CTRL | CTRL0_7
  *  AON_PAD_CTL_0    |  MCU_OVR
  *
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_AON_GPIO_MUX_0
  *         @arg @ref LL_AON_GPIO_MUX_1
  *         @arg @ref LL_AON_GPIO_MUX_2
  *         @arg @ref LL_AON_GPIO_MUX_3
  *         @arg @ref LL_AON_GPIO_MUX_4
  *         @arg @ref LL_AON_GPIO_MUX_5
  *         @arg @ref LL_AON_GPIO_MUX_6
  *         @arg @ref LL_AON_GPIO_MUX_7
  *         @arg @ref LL_AON_GPIO_MUX_8
  */
__STATIC_INLINE uint32_t ll_aon_gpio_get_pin_mux(uint32_t pin)
{
    if(READ_BITS(AON_IO->AON_MCU_OVR, pin << AON_IO_AON_MCU_OVR_OVR_POS))
    {
        uint32_t pos = POSITION_VAL(pin) << 2;
        return (READ_BITS(MCU_RET->AON_PAD_MUX_CTL, 0xF << pos) >> pos);
    }
    else
    {
        return LL_AON_GPIO_MUX_7;
    }
}

/**
  * @brief  Enable clock output on AON_GPIO_PIN4.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CLK | AON_PAD_CLK
  *
  *  @param  clk_sel This parameter can be the following values:
  *         @arg @ref LL_CLK_RNG_OSC_32K
  *         @arg @ref LL_CLK_RNG_OSC_2MHZ
  *         @arg @ref LL_CLK_RC_OSC_CLK
  *         @arg @ref LL_CLK_RTC_CLK
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio4_enable_clk_output(uint32_t clk_sel)
{
    MODIFY_REG(AON_CTL->AON_PAD_CLK, AON_CTL_AON_PAD_CLK_AON_GPIO4_CLK_SEL,
               AON_CTL_AON_PAD_CLK_AON_GPIO4_OUT_EN | (clk_sel<<AON_CTL_AON_PAD_CLK_AON_GPIO4_CLK_SEL_Pos));
}

/**
  * @brief  Disable clock output on AON_GPIO_PIN4.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CLK | AON_PAD_CLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio4_disable_clk_output(void)
{
    CLEAR_BITS(AON_CTL->AON_PAD_CLK, AON_CTL_AON_PAD_CLK_AON_GPIO4_OUT_EN);
}

/**
  * @brief  Check if clock output on AON_GPIO_PIN4 is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CLK | AON_PAD_CLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_gpio4_is_enabled_clk_output(void)
{
   return (READ_BITS(AON_CTL->AON_PAD_CLK, AON_CTL_AON_PAD_CLK_AON_GPIO4_OUT_EN) == AON_CTL_AON_PAD_CLK_AON_GPIO4_OUT_EN);
}

/**
  * @brief  Enable Xo_2MHz output on AON_GPIO_PIN5.
  *
  *  Register|BitsName
  *  --------|--------
  *  PWR_RET01 | XO_2MHZ_ENA
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_enable_xo_2mhz_output(void)
{
    SET_BITS(AON_CTL->XO_CTRL, AON_CTL_XO_CTRL_2MHZ_OUT);
}

/**
  * @brief  Disable Xo_2MHz output on AON_GPIO_PIN5.
  *
  *  Register|BitsName
  *  --------|--------
  *  PWR_RET01 | XO_2MHZ_ENA
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_disable_xo_2mhz_output(void)
{
    CLEAR_BITS(AON_CTL->XO_CTRL, AON_CTL_XO_CTRL_2MHZ_OUT);
}

/**
  * @brief  Check if Xo_2MHz output on AON_GPIO_PIN5 is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  PWR_RET01 | XO_2MHZ_ENA
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_aon_gpio_is_enabled_xo_2mhz_output(void)
{
    return (uint32_t)(READ_BITS(AON_CTL->XO_CTRL, AON_CTL_XO_CTRL_2MHZ_OUT) == AON_CTL_XO_CTRL_2MHZ_OUT);
}

/** @} */

/** @defgroup AON_GPIO_LL_EF_Data_Access Data Access
  * @{
  */

/**
  * @brief  Return full input data register value of AON_GPIO.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL1 | O_AON_GPI
  *
  * @retval Input data register value of port
  */
__STATIC_INLINE uint32_t ll_aon_gpio_read_input_port(void)
{
    uint32_t pin_mask = (LL_AON_GPIO_PIN_ALL << AON_IO_AON_PAD_CTRL1_IN_VAL_POS) & AON_IO_AON_PAD_CTRL1_IN_VAL;
    return (uint32_t)(READ_BITS(AON_IO->AON_PAD_CTRL1, pin_mask) >> AON_IO_AON_PAD_CTRL1_IN_VAL_POS);
}

/**
  * @brief  Return if input data level of several AON_GPIO pins is high or low.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL1 | O_AON_GPI
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_gpio_read_input_pin(uint32_t pin_mask)
{
    pin_mask = (pin_mask << AON_IO_AON_PAD_CTRL1_IN_VAL_POS) & AON_IO_AON_PAD_CTRL1_IN_VAL;
    return (uint32_t)(READ_BITS(AON_IO->AON_PAD_CTRL1, pin_mask) == pin_mask);
}

/**
  * @brief  Return if output data level of several AON_GPIO pins is high or low.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL1 | AON_GPIO
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_gpio_read_output_pin(uint32_t pin_mask)
{
    pin_mask = (pin_mask << AON_IO_AON_PAD_CTRL1_OUT_VAL_POS) & AON_IO_AON_PAD_CTRL1_OUT_VAL;
    return (uint32_t)(READ_BITS(AON_IO->AON_PAD_CTRL1, pin_mask) == pin_mask);
}
/**
  * @brief  Write output data register of AON_GPIO.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL1 | AON_GPO
  *
  * @param  port_value Level value for each pin of the port
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_write_output_port(uint32_t port_value)
{
    MODIFY_REG(AON_IO->AON_PAD_CTRL1, AON_IO_AON_PAD_CTRL1_OUT_VAL, (port_value << AON_IO_AON_PAD_CTRL1_OUT_VAL_POS) & AON_IO_AON_PAD_CTRL1_OUT_VAL);
}

/**
  * @brief  Return full output data register value of AON_GPIO.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL1 | AON_GPO
  *
  * @retval Output data register value of port
  */
__STATIC_INLINE uint32_t ll_aon_gpio_read_output_port(void)
{
    return (uint32_t)(READ_BITS(AON_IO->AON_PAD_CTRL1, AON_IO_AON_PAD_CTRL1_OUT_VAL) >> AON_IO_AON_PAD_CTRL1_OUT_VAL_POS);
}


/**
  * @brief  Set specified AON_GPIO pins to high level
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL1 | AON_GPO
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_aon_gpio_set_output_pin(uint32_t pin_mask)
{
    SET_BITS(AON_IO->AON_PAD_CTRL1, (pin_mask << AON_IO_AON_PAD_CTRL1_OUT_VAL_POS) & AON_IO_AON_PAD_CTRL1_OUT_VAL);
}

/**
  * @brief  Set specified AON_GPIO pins to low level.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL1 | AON_GPO
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_aon_gpio_reset_output_pin(uint32_t pin_mask)
{
    CLEAR_BITS(AON_IO->AON_PAD_CTRL1, (pin_mask << AON_IO_AON_PAD_CTRL1_OUT_VAL_POS) & AON_IO_AON_PAD_CTRL1_OUT_VAL);
}

/**
  * @brief  Toggle data value of specified AON_GPIO pins.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_PAD_CTL1 | AON_GPO
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_aon_gpio_toggle_pin(uint32_t pin_mask)
{
    WRITE_REG(AON_IO->AON_PAD_CTRL1, (READ_REG(AON_IO->AON_PAD_CTRL1)
        ^ ((pin_mask << AON_IO_AON_PAD_CTRL1_OUT_VAL_POS) & AON_IO_AON_PAD_CTRL1_OUT_VAL)));
}

/** @} */

/** @defgroup AON_GPIO_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable AON_GPIO Falling Edge Trigger of specified AON_GPIO pins.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLCLR | INTPOLCLR
  *  INTTYPESET | INTTYPESET
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_enable_falling_trigger(uint32_t pin_mask)
{
    uint32_t invert = (pin_mask << AON_IO_EXT_WAKEUP_CTRL0_INVERT_POS) & AON_IO_EXT_WAKEUP_CTRL0_INVERT;
    uint32_t edge_en = (pin_mask << AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN_POS) & AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN;
    uint32_t edge_type = (pin_mask << AON_IO_EXT_WAKEUP_CTRL1_EDGE_TYPE_POS) & AON_IO_EXT_WAKEUP_CTRL1_EDGE_TYPE;
    uint32_t both = (pin_mask << AON_IO_EXT_WAKEUP_CTRL1_EDGE_BOTH_POS) & AON_IO_EXT_WAKEUP_CTRL1_EDGE_BOTH;
    CLEAR_BITS(AON_IO->EXT_WAKEUP_CTRL0, invert);
    MODIFY_REG(AON_IO->EXT_WAKEUP_CTRL1, both, edge_en | edge_type);
}

/**
  * @brief  Check if falling edge trigger is enabled of specified AON_GPIO pins.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLCLR | INTPOLCLR
  *  INTTYPESET | INTTYPESET
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t ll_aon_gpio_is_enabled_falling_trigger(uint32_t pin_mask)
{
    uint32_t invert = ((~READ_BITS(AON_IO->EXT_WAKEUP_CTRL0, AON_IO_EXT_WAKEUP_CTRL0_INVERT) >> AON_IO_EXT_WAKEUP_CTRL0_INVERT_POS) & (pin_mask)) == (pin_mask);
    uint32_t edge_en = ((READ_BITS(AON_IO->EXT_WAKEUP_CTRL1, AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN) >> AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN_POS) & (pin_mask)) == (pin_mask);
    uint32_t edge_type = ((READ_BITS(AON_IO->EXT_WAKEUP_CTRL1, AON_IO_EXT_WAKEUP_CTRL1_EDGE_TYPE) >> AON_IO_EXT_WAKEUP_CTRL1_EDGE_TYPE_POS) & (pin_mask)) == (pin_mask);
    return (invert && edge_en && edge_type);
}

/**
  * @brief  Enable AON_GPIO Rising Edge Trigger of specified AON_GPIO pins.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLSET | INTPOLSET
  *  INTTYPESET | INTTYPESET
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_enable_rising_trigger(uint32_t pin_mask)
{
    uint32_t invert = (pin_mask << AON_IO_EXT_WAKEUP_CTRL0_INVERT_POS) & AON_IO_EXT_WAKEUP_CTRL0_INVERT;
    uint32_t edge_en = (pin_mask << AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN_POS) & AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN;
    uint32_t edge_type = (pin_mask << AON_IO_EXT_WAKEUP_CTRL1_EDGE_TYPE_POS) & AON_IO_EXT_WAKEUP_CTRL1_EDGE_TYPE;
    uint32_t both = (pin_mask << AON_IO_EXT_WAKEUP_CTRL1_EDGE_BOTH_POS) & AON_IO_EXT_WAKEUP_CTRL1_EDGE_BOTH;
    CLEAR_BITS(AON_IO->EXT_WAKEUP_CTRL0, invert);
    MODIFY_REG(AON_IO->EXT_WAKEUP_CTRL1, edge_type | both, edge_en);
}

/**
  * @brief  Check if rising edge trigger is enabled of specified AON_GPIO pins.
  * @note   Please check each device line mapping for AON_GPIO Line availability
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLSET | INTPOLSET
  *  INTTYPESET | INTTYPESET
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_gpio_is_enabled_rising_trigger(uint32_t pin_mask)
{
    uint32_t invert = (((~READ_BITS(AON_IO->EXT_WAKEUP_CTRL0, AON_IO_EXT_WAKEUP_CTRL0_INVERT)) >> AON_IO_EXT_WAKEUP_CTRL0_INVERT_POS) & (pin_mask)) == (pin_mask);
    uint32_t edge_en = ((READ_BITS(AON_IO->EXT_WAKEUP_CTRL1, AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN) >> AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN_POS) & (pin_mask)) == (pin_mask);
    uint32_t edge_type = (((~READ_BITS(AON_IO->EXT_WAKEUP_CTRL1, AON_IO_EXT_WAKEUP_CTRL1_EDGE_TYPE)) >> AON_IO_EXT_WAKEUP_CTRL1_EDGE_TYPE_POS) & (pin_mask)) == (pin_mask);
    return (invert && edge_en && edge_type);
}

/**
  * @brief  Enable AON_GPIO High Level Trigger of specified AON_GPIO pins.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLSET | INTPOLSET
  *  INTTYPECLR | INTTYPECLR
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_enable_high_trigger(uint32_t pin_mask)
{
    uint32_t invert = (pin_mask << AON_IO_EXT_WAKEUP_CTRL0_INVERT_POS) & AON_IO_EXT_WAKEUP_CTRL0_INVERT;
    uint32_t edge_en = (pin_mask << AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN_POS) & AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN;
    CLEAR_BITS(AON_IO->EXT_WAKEUP_CTRL0, invert);
    CLEAR_BITS(AON_IO->EXT_WAKEUP_CTRL1, edge_en);
}

/**
  * @brief  Check if high level trigger is enabled of specified AON_GPIO pins.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLSET | INTPOLSET
  *  INTTYPECLR | INTTYPECLR
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_gpio_is_enabled_high_trigger(uint32_t pin_mask)
{
    uint32_t invert = (((~READ_BITS(AON_IO->EXT_WAKEUP_CTRL0, AON_IO_EXT_WAKEUP_CTRL0_INVERT)) >> AON_IO_EXT_WAKEUP_CTRL0_INVERT_POS) & (pin_mask)) == (pin_mask);
    uint32_t edge_en = (((~READ_BITS(AON_IO->EXT_WAKEUP_CTRL1, AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN)) >> AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN_POS) & (pin_mask)) == (pin_mask);
    return (invert && edge_en );
}

/**
  * @brief  Enable AON_GPIO Low Level Trigger of specified AON_GPIO pins.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLCLR | INTPOLCLR
  *  INTTYPECLR | INTTYPECLR
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_enable_low_trigger(uint32_t pin_mask)
{
    uint32_t invert = (pin_mask << AON_IO_EXT_WAKEUP_CTRL0_INVERT_POS) & AON_IO_EXT_WAKEUP_CTRL0_INVERT;
    uint32_t edge_en = (pin_mask << AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN_POS) & AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN;
    SET_BITS(AON_IO->EXT_WAKEUP_CTRL0, invert);
    CLEAR_BITS(AON_IO->EXT_WAKEUP_CTRL1, edge_en);
}

/**
  * @brief  Check if low level trigger is enabled of specified AON_GPIO pins.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLCLR | INTPOLCLR
  *  INTTYPECLR | INTTYPECLR
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_gpio_is_enabled_low_trigger(uint32_t pin_mask)
{
    uint32_t invert = ((READ_BITS(AON_IO->EXT_WAKEUP_CTRL0, AON_IO_EXT_WAKEUP_CTRL0_INVERT) >> AON_IO_EXT_WAKEUP_CTRL0_INVERT_POS) & (pin_mask)) == (pin_mask);
    uint32_t edge_en = (((~READ_BITS(AON_IO->EXT_WAKEUP_CTRL1, AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN)) >> AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN_POS) & (pin_mask)) == (pin_mask);
    return (invert && edge_en);
}

/**
  * @brief  Enable AON_GPIO Low Level Trigger of specified AON_GPIO pins.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLCLR | INTPOLCLR
  *  INTTYPECLR | INTTYPECLR
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_enable_both_trigger(uint32_t pin_mask)
{
    uint32_t edge_en = (pin_mask << AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN_POS) & AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN;
    uint32_t both = (pin_mask << AON_IO_EXT_WAKEUP_CTRL1_EDGE_BOTH_POS) & AON_IO_EXT_WAKEUP_CTRL1_EDGE_BOTH;
    SET_BITS(AON_IO->EXT_WAKEUP_CTRL1, edge_en | both);
}

/**
  * @brief  Check if low level trigger is enabled of specified AON_GPIO pins.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLCLR | INTPOLCLR
  *  INTTYPECLR | INTTYPECLR
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t ll_aon_gpio_is_enabled_both_trigger(uint32_t pin_mask)
{
    uint32_t edge_en = ((READ_BITS(AON_IO->EXT_WAKEUP_CTRL1, AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN) >> AON_IO_EXT_WAKEUP_CTRL1_EDGE_EN_POS) & (pin_mask)) == (pin_mask);
    uint32_t edge_both = ((READ_BITS(AON_IO->EXT_WAKEUP_CTRL1, AON_IO_EXT_WAKEUP_CTRL1_EDGE_TYPE) >> AON_IO_EXT_WAKEUP_CTRL1_EDGE_TYPE_POS) & (pin_mask)) == (pin_mask);
    return ( edge_en && edge_both);
}

/**
  * @brief  Enable AON_GPIO interrupts of specified AON_GPIO pins.
  * @note   @ref AON_GPIO_LL_EC_TRIGGER can be used to specify the interrupt trigger type
  *
  *  Register|BitsName
  *  --------|--------
  *  INTENSET | INTENSET
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_enable_it(uint32_t pin_mask)
{
    SET_BITS(AON_IO->EXT_WAKEUP_CTRL0, pin_mask);
}

/**
  * @brief  Disable AON_GPIO interrupts of specified AON_GPIO pins.
  * @note   @ref AON_GPIO_LL_EC_TRIGGER can be used to specify the interrupt trigger type
  *
  *  Register|BitsName
  *  --------|--------
  *  INTENCLR | INTENCLR
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_disable_it(uint32_t pin_mask)
{
    CLEAR_BITS(AON_IO->EXT_WAKEUP_CTRL0, pin_mask);
}

/**
  * @brief  Check if the Interrupt of specified GPIO pins is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTENSET | INTENSET
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_gpio_is_enabled_it(uint32_t pin_mask)
{
    return ((READ_BITS(AON_IO->EXT_WAKEUP_CTRL0, AON_IO_EXT_WAKEUP_CTRL0_SRC_EN) & (pin_mask)) == (pin_mask));
}

/**
  * @brief Get aon gpio enable pins.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTENSET | INTENSET
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  */
__STATIC_INLINE uint32_t ll_aon_gpio_get_enabled_pin(void)
{
    return (READ_BITS(AON_IO->EXT_WAKEUP_CTRL0, AON_IO_EXT_WAKEUP_CTRL0_SRC_EN));
}

/** @} */

/** @defgroup AON_GPIO_LL_EF_Flag_Management Flag_Management
 * @{
 */

/**
  * @brief  Read AON_GPIO Interrupt Combination Flag of specified AON_GPIO pins.
  * @note   After an interrupt is triggered, the corresponding bit in the INTSTATUS Register is set.
  *         The interrupt status can cleared by writing 1 to corresponding bit in INTCLEAR Register.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTATUS | INTSTATUS
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval Interrupt flag whose bits were set when the selected trigger event arrives on the interrupt
  */
__STATIC_INLINE uint32_t ll_aon_gpio_read_flag_it(uint32_t pin_mask)
{
    return (READ_BITS(AON_IO->EXT_WAKEUP_STAT, AON_IO_EXT_WAKEUP_STAT_STAT) & (pin_mask));
}

/**
  * @brief  Indicate if the AON_GPIO Interrupt Flag is set or not of specified AON_GPIO pins.
  * @note   After an interrupt is triggered, the corresponding bit in the INTSTATUS Register is set.
  *         The interrupt status can cleared by writing 1 to corresponding bit in INTCLEAR Register.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTATUS | INTSTATUS
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_gpio_is_active_flag_it(uint32_t pin_mask)
{
    return ((READ_BITS(AON_IO->EXT_WAKEUP_STAT, AON_IO_EXT_WAKEUP_STAT_STAT) & (pin_mask)) == (pin_mask));
}

/**
  * @brief  Clear Interrupt Status flag of specified AON_GPIO pins.
  * @note   After an interrupt is triggered, the corresponding bit in the INTSTATUS Register is set.
  *         The interrupt status can be cleared by writing 1 to corresponding bit in INTCLEAR Register.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTATUS | INTSTATUS
  *
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_AON_GPIO_PIN_0
  *         @arg @ref LL_AON_GPIO_PIN_1
  *         @arg @ref LL_AON_GPIO_PIN_2
  *         @arg @ref LL_AON_GPIO_PIN_3
  *         @arg @ref LL_AON_GPIO_PIN_4
  *         @arg @ref LL_AON_GPIO_PIN_5
  *         @arg @ref LL_AON_GPIO_PIN_6
  *         @arg @ref LL_AON_GPIO_PIN_7
  *         @arg @ref LL_AON_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_clear_flag_it(uint32_t pin_mask)
{
    CLEAR_BITS(AON_IO->EXT_WAKEUP_STAT, pin_mask);
}

/**
  * @brief  Indicate if the AON GPIO event flag is set or not.
  * @note   This bit is set by hardware when system wakeup by AON GPIO.
  *         It can be cleared by writing 0 to this bit.
  *
  *  Register|BitsName
  *  --------|--------
  *  SLP_EVENT | AON GPIO
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_gpio_read_event_flag_it(void)
{
    return (uint32_t)(READ_BITS(AON_CTL->AON_SLP_EVENT, AON_CTL_SLP_EVENT_EXT) == AON_CTL_SLP_EVENT_EXT);
}

/**
  * @brief  Clear AON GPIO event interrupt flag.
  *
  *  Register|BitsName
  *  --------|--------
  *  SLP_EVENT| AON GPIO
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_gpio_clear_event_flag_it(void)
{
    WRITE_REG(AON_CTL->AON_SLP_EVENT, ~AON_CTL_SLP_EVENT_EXT);
}

/** @} */

/** @defgroup AON_GPIO_LL_EF_Init Initialization and de-initialization functions
  * @{
  */
/**
  * @brief  Initialize AON_GPIO registers according to the specified.
  *         parameters in p_aon_gpio_init.
  * @param  p_aon_gpio_init Pointer to a ll_aon_gpio_init_t structure that contains the configuration
  *                             information for the specified AON_GPIO peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: AON_GPIO registers are initialized according to p_aon_gpio_init content
  *          - ERROR: Problem occurred during AON_GPIO Registers initialization
  */
error_status_t ll_aon_gpio_init(ll_aon_gpio_init_t *p_aon_gpio_init);

/** @} */

/** @} */

#endif /* AON */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_AON_GPIO_H__ */

/** @} */

/** @} */

/** @} */

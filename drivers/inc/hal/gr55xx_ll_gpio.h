/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_gpio.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of GPIO LL library.
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

/** @defgroup LL_GPIO GPIO
  * @brief GPIO LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_GPIO_H__
#define __GR55XX_LL_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (GPIO0) || defined (GPIO1) || defined(MCU_RET)

/** @defgroup GPIO_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup GPIO_LL_ES_INIT GPIO Exported init structures
  * @{
  */

/**
  * @brief LL GPIO init configuration definition
  */
typedef struct _ll_gpio_init
{
    uint32_t pin;           /*!< Specifies the GPIO pins to be GPIO_InitStructured.
                                This parameter can be any value of @ref GPIO_LL_EC_PIN */

    uint32_t mode;          /*!< Specifies the operating mode for the selected pins.
                                This parameter can be a value of @ref GPIO_LL_EC_MODE.

                                GPIO HW GPIO_InitStructuration can be modified afterwards using unitary function @ref ll_gpio_set_pin_mode(). */

    uint32_t pull;          /*!< Specifies the operating Pull-up/Pull down for the selected pins.
                              This parameter can be a value of @ref GPIO_LL_EC_PULL.

                              GPIO HW configuration can be modified afterwards using unitary function @ref ll_gpio_set_pin_pull().*/

    uint32_t mux;           /*!< Specifies the Peripheral to be connected to the selected pins.
                                This parameter can be a value of @ref GPIO_LL_EC_MUX.

                                GPIO HW GPIO_InitStructuration can be modified afterwards using unitary function
                                ll_gpio_set_mux_pin_0_7() and ll_gpio_set_mux_pin_8_15(). */

    uint32_t trigger;        /*!< Specifies the trigger signal active edge.
                                This parameter can be a value of @ref GPIO_LL_EC_TRIGGER. */

} ll_gpio_init_t;

/** @} */

/** @} */

/**
  * @defgroup  GPIO_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup GPIO_LL_Exported_Constants GPIO Exported Constants
  * @{
  */

/** @defgroup GPIO_LL_EC_PIN PIN
  * @{
  */
#define LL_GPIO_PIN_0               ((uint32_t)0x0001U) /*!< Select pin 0 */
#define LL_GPIO_PIN_1               ((uint32_t)0x0002U) /*!< Select pin 1 */
#define LL_GPIO_PIN_2               ((uint32_t)0x0004U) /*!< Select pin 2 */
#define LL_GPIO_PIN_3               ((uint32_t)0x0008U) /*!< Select pin 3 */
#define LL_GPIO_PIN_4               ((uint32_t)0x0010U) /*!< Select pin 4 */
#define LL_GPIO_PIN_5               ((uint32_t)0x0020U) /*!< Select pin 5 */
#define LL_GPIO_PIN_6               ((uint32_t)0x0040U) /*!< Select pin 6 */
#define LL_GPIO_PIN_7               ((uint32_t)0x0080U) /*!< Select pin 7 */
#define LL_GPIO_PIN_8               ((uint32_t)0x0100U) /*!< Select pin 8 */
#define LL_GPIO_PIN_9               ((uint32_t)0x0200U) /*!< Select pin 9 */
#define LL_GPIO_PIN_10              ((uint32_t)0x0400U) /*!< Select pin 10 */
#define LL_GPIO_PIN_11              ((uint32_t)0x0800U) /*!< Select pin 11 */
#define LL_GPIO_PIN_12              ((uint32_t)0x1000U) /*!< Select pin 12 */
#define LL_GPIO_PIN_13              ((uint32_t)0x2000U) /*!< Select pin 13 */
#define LL_GPIO_PIN_14              ((uint32_t)0x4000U) /*!< Select pin 14 */
#define LL_GPIO_PIN_15              ((uint32_t)0x8000U) /*!< Select pin 15 */
#define LL_GPIO_PIN_ALL             ((uint32_t)0xFFFFU) /*!< Select all pins */
/** @} */

/** @defgroup GPIO_LL_EC_MODE Mode
  * @{
  */
#define LL_GPIO_MODE_INPUT          ((uint32_t)0x0U)  /*!< Select input mode */
#define LL_GPIO_MODE_OUTPUT         ((uint32_t)0x1U)  /*!< Select output mode */
#define LL_GPIO_MODE_MUX            ((uint32_t)0x2U)  /*!< Select mux peripheral mode */
/** @} */

/** @defgroup GPIO_LL_EC_PULL Pull Up Pull Down
  * @{
  */
#define LL_GPIO_PULL_NO             LL_GPIO_RE_N      /*!< Select I/O no pull */
#define LL_GPIO_PULL_UP             LL_GPIO_RTYP      /*!< Select I/O pull up */
#define LL_GPIO_PULL_DOWN           ((uint32_t)0x0U)  /*!< Select I/O pull down */
/** @} */

/** @defgroup GPIO_LL_EC_MUX Alternate Function
  * @{
  */
#define LL_GPIO_MUX_0               ((uint32_t)0x0U) /*!< Select alternate function 0 */
#define LL_GPIO_MUX_1               ((uint32_t)0x1U) /*!< Select alternate function 1 */
#define LL_GPIO_MUX_2               ((uint32_t)0x2U) /*!< Select alternate function 2 */
#define LL_GPIO_MUX_3               ((uint32_t)0x3U) /*!< Select alternate function 3 */
#define LL_GPIO_MUX_4               ((uint32_t)0x4U) /*!< Select alternate function 4 */
#define LL_GPIO_MUX_5               ((uint32_t)0x5U) /*!< Select alternate function 5 */
#define LL_GPIO_MUX_6               ((uint32_t)0x6U) /*!< Select alternate function 6 */
#define LL_GPIO_MUX_7               ((uint32_t)0x7U) /*!< Select alternate function 7 */
#define LL_GPIO_MUX_8               ((uint32_t)0x8U) /*!< Select alternate function 8 */
/** @} */

/** @defgroup GPIO_LL_EC_TRIGGER Interrupt Trigger
  * @{
  */
#define LL_GPIO_TRIGGER_NONE        ((uint32_t)0x00U) /*!< No Trigger Mode */
#define LL_GPIO_TRIGGER_RISING      ((uint32_t)0x01U) /*!< Trigger Rising Mode */
#define LL_GPIO_TRIGGER_FALLING     ((uint32_t)0x02U) /*!< Trigger Falling Mode */
#define LL_GPIO_TRIGGER_HIGH        ((uint32_t)0x03U) /*!< Trigger High Mode */
#define LL_GPIO_TRIGGER_LOW         ((uint32_t)0x04U) /*!< Trigger Low Mode */
#define LL_GPIO_TRIGGER_BOTH_EDGE   ((uint32_t)0x05U) /*!< Trigger Both Rising and Falling Mode */

/** @} */

/** @defgroup GPIO_LL_REG_ADDR Address of GPIOx Registers
  * @attention x=0/1/2
  * @{
  */
#define GPIO_DATA_ADDR(x)                (GPIO##x##_BASE+0x00) /*!< GPIOx_DATA_ADDR       */
#define GPIO_DATAOUT_ADDR(x)             (GPIO##x##_BASE+0x04) /*!< GPIOx_DATAOUT_ADDR    */
#define GPIO_OUTENSET_ADDR(x)            (GPIO##x##_BASE+0x10) /*!< GPIOx_OUTENSET_ADDR   */
#define GPIO_OUTENCLR_ADDR(x)            (GPIO##x##_BASE+0x14) /*!< GPIOx_OUTENCLR_ADDR   */
#define GPIO_ALTFUNCSET_ADDR(x)          (GPIO##x##_BASE+0x18) /*!< GPIOx_ALTFUNCSET_ADDR */
#define GPIO_ALTFUNCCLR_ADDR(x)          (GPIO##x##_BASE+0x1c) /*!< GPIOx_ALTFUNCCLR_ADDR */
#define GPIO_INTENSET_ADDR(x)            (GPIO##x##_BASE+0x20) /*!< GPIOx_INTENSET_ADDR   */
#define GPIO_INTENCLR_ADDR(x)            (GPIO##x##_BASE+0x24) /*!< GPIOx_INTENCLR_ADDR   */
#define GPIO_INTTYPESET_ADDR(x)          (GPIO##x##_BASE+0x28) /*!< GPIOx_INTTYPESET_ADDR */
#define GPIO_INTTYPECLR_ADDR(x)          (GPIO##x##_BASE+0x2c) /*!< GPIOx_INTTYPECLR_ADDR */
#define GPIO_INTPOLSET_ADDR(x)           (GPIO##x##_BASE+0x30) /*!< GPIOx_INTPOLSET_ADDR  */
#define GPIO_INTPOLCLR_ADDR(x)           (GPIO##x##_BASE+0x34) /*!< GPIOx_INTPOLCLR_ADDR  */
#define GPIO_INTSTAT_ADDR(x)             (GPIO##x##_BASE+0x38) /*!< GPIOx_INTSTAT_ADDR    */
#define GPIO_INTDBESET_ADDR(x)           (GPIO##x##_BASE+0x40) /*!< GPIOx_INTDBESET_ADDR  */
#define GPIO_INTDBECLR_ADDR(x)           (GPIO##x##_BASE+0x44) /*!< GPIOx_INTDBECLR_ADDR  */
/** @} */

/** @defgroup GPIO_LL_BITBAND_SET Set nth bit for GPIOx register 0 or 1
  * @attention x=0/1/2,0<=n<=31
  * @{
  */
#define SET_GPIO_DATA(x,n)               BIT_ADDR(GPIO_DATA_ADDR(x),n)       /*!< BITBAND GPIOx_DATA_ADDR       */
#define SET_GPIO_DATAOUT(x,n)            BIT_ADDR(GPIO_DATAOUT_ADDR(x),n)    /*!< BITBAND GPIOx_DATAOUT_ADDR    */
#define SET_GPIO_OUTENSET(x,n)           BIT_ADDR(GPIO_OUTENSET_ADDR(x),n)   /*!< BITBAND GPIOx_OUTENSET_ADDR   */
#define SET_GPIO_OUTENCLR(x,n)           BIT_ADDR(GPIO_OUTENCLR_ADDR(x),n)   /*!< BITBAND GPIOx_OUTENCLR_ADDR   */
#define SET_GPIO_ALTFUNCSET(x,n)         BIT_ADDR(GPIO_ALTFUNCSET_ADDR(x),n) /*!< BITBAND GPIOx_ALTFUNCSET_ADDR */
#define SET_GPIO_ALTFUNCCLR(x,n)         BIT_ADDR(GPIO_ALTFUNCCLR_ADDR(x),n) /*!< BITBAND GPIOx_ALTFUNCCLR_ADDR */
#define SET_GPIO_INTENSET(x,n)           BIT_ADDR(GPIO_INTENSET_ADDR(x),n)   /*!< BITBAND GPIOx_INTENSET_ADDR   */
#define SET_GPIO_INTENCLR(x,n)           BIT_ADDR(GPIO_INTENCLR_ADDR(x),n)   /*!< BITBAND GPIOx_INTENCLR_ADDR   */
#define SET_GPIO_INTTYPESET(x,n)         BIT_ADDR(GPIO_INTTYPESET_ADDR(x),n) /*!< BITBAND GPIOx_INTTYPESET_ADDR */
#define SET_GPIO_INTTYPECLR(x,n)         BIT_ADDR(GPIO_INTTYPECLR_ADDR(x),n) /*!< BITBAND GPIOx_INTTYPECLR_ADDR */
#define SET_GPIO_INTPOLSET(x,n)          BIT_ADDR(GPIO_INTPOLSET_ADDR(x),n)  /*!< BITBAND GPIOx_INTPOLSET_ADDR  */
#define SET_GPIO_INTPOLCLR(x,n)          BIT_ADDR(GPIO_INTPOLCLR_ADDR(x),n)  /*!< BITBAND GPIOx_INTPOLCLR_ADDR  */
#define SET_GPIO_INTSTAT(x,n)            BIT_ADDR(GPIO_INTSTAT_ADDR(x),n)    /*!< BITBAND GPIOx_INTSTAT_ADDR    */
#define SET_GPIO_INTDBESET(x,n)          BIT_ADDR(GPIO_INTDBESET_ADDR(x),n)  /*!< BITBAND GPIOx_INTDBESET_ADDR  */
#define SET_GPIO_INTDBECLR(x,n)          BIT_ADDR(GPIO_INTDBECLR_ADDR(x),n)  /*!< BITBAND GPIOx_INTDBECLR_ADDR  */
/** @} */

/** @defgroup GPIO_LL_EC_DEFAULT_CONFIG InitStrcut default configuration
  * @{
  */

/**
  * @brief LL GPIO InitStrcut default configuration
  */
#define LL_GPIO_DEFAULT_CONFIG                      \
{                                                   \
    .pin        = LL_GPIO_PIN_ALL,                  \
    .mode       = LL_GPIO_MODE_INPUT,               \
    .pull       = LL_GPIO_PULL_DOWN,                \
    .mux        = LL_GPIO_MUX_7,                    \
    .trigger    = LL_GPIO_TRIGGER_NONE,             \
}
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup GPIO_LL_Exported_Macros GPIO Exported Macros
  * @{
  */

/** @defgroup GPIO_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in GPIO register
  * @param  __instance__ GPIO instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_GPIO_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in GPIO register
  * @param  __instance__ GPIO instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_GPIO_ReadReg(__instance__, __REG__) READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup GPIO_LL_Private_Macros GPIO Private Macros
  * @{
  */

/** @defgroup GPIO_LL_PM_RESISTOR Resistor Enable
  * @{
  */
#define LL_GPIO_RE_N_Pos            0                           /**< Resistor Enable bits position */
#define LL_GPIO_RE_N_Msk            (0x1U << LL_GPIO_RE_N_Pos)  /**< Resistor Enable bits mask     */
#define LL_GPIO_RE_N                LL_GPIO_RE_N_Msk            /**< Resistor Enable bits          */
/** @} */

/** @defgroup GPIO_LL_PM_RESISTOR_TYPE Resistor Type
  * @{
  */
#define LL_GPIO_RTYP_Pos            1                           /**< Resistor Type bits position */
#define LL_GPIO_RTYP_Msk            (0x1U << LL_GPIO_RTYP_Pos)  /**< Resistor Type bits mask     */
#define LL_GPIO_RTYP                LL_GPIO_RTYP_Msk            /**< Resistor Type bits          */
/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup GPIO_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup GPIO_LL_EF_Port_Configuration Port Configuration
  * @{
  */

/**
  * @brief  Set several pins to input/output mode on dedicated port.
  *
  *  Register|BitsName
  *  --------|--------
  *  OUTENSET | OUTENSET
  *  OUTENCLR | OUTENCLR
  *
  * @param  GPIOx GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_MODE_INPUT
  *         @arg @ref LL_GPIO_MODE_OUTPUT
  *         @arg @ref LL_GPIO_MODE_MUX
  * @retval None
  */
__STATIC_INLINE void ll_gpio_set_pin_mode(gpio_regs_t *GPIOx, uint32_t pin_mask, uint32_t mode)
{
    if (mode == LL_GPIO_MODE_OUTPUT)
    {
        WRITE_REG(GPIOx->OUTENSET, pin_mask);
        if(GPIO2 == GPIOx)
        {
            SET_BITS(MCU_RET->DPAD_IE_N_BUS_1, pin_mask);
        }
        else
        {
            pin_mask <<=  ((GPIOx == GPIO0) ? 0 : 16);
            SET_BITS(MCU_RET->DPAD_IE_N_BUS_0, pin_mask);
        }
    }
    else if((mode == LL_GPIO_MODE_INPUT) || (mode == LL_GPIO_MODE_MUX))
    {
        WRITE_REG(GPIOx->OUTENCLR, pin_mask);
        if(GPIO2 == GPIOx)
        {
            CLEAR_BITS(MCU_RET->DPAD_IE_N_BUS_1, pin_mask);
        }
        else
        {
            pin_mask <<=  ((GPIOx == GPIO0) ? 0 : 16);
            CLEAR_BITS(MCU_RET->DPAD_IE_N_BUS_0, pin_mask);
        }
    }
}

/**
  * @brief  Return gpio mode for a dedicated pin on dedicated port.
  * @note   I/O mode can be Input mode, General purpose output.
  * @note   Warning: only one pin can be passed as parameter.
  *
  *  Register|BitsName
  *  --------|--------
  *  OUTENSET | OUTENSET
  *
  * @param  GPIOx GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_MODE_INPUT
  *         @arg @ref LL_GPIO_MODE_OUTPUT
  */
__STATIC_INLINE uint32_t ll_gpio_get_pin_mode(gpio_regs_t *GPIOx, uint32_t pin)
{
    return (uint32_t)(READ_BITS(GPIOx->OUTENSET, pin) != RESET);
}

/**
  * @brief  Configure gpio pull-up or pull-down for a dedicated pin on a dedicated port.
  *
  *  Register|BitsName
  *  --------|--------
  *  DPAD_RE_N_BUS | RE_N
  *  DPAD_RTYP_BUS | RTYP
  *
  * @param  GPIOx GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @param  pull This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PULL_NO
  *         @arg @ref LL_GPIO_PULL_UP
  *         @arg @ref LL_GPIO_PULL_DOWN
  * @retval None
  */
__STATIC_INLINE void ll_gpio_set_pin_pull(gpio_regs_t *GPIOx, uint32_t pin_mask, uint32_t pull)
{
    /* Get pin mask in resitor related registers, GPIO0:0~15, GPIO1:16~31 */
    if(GPIO2 == GPIOx)
    {
        MODIFY_REG(MCU_RET->DPAD_RTYP_BUS_1, pin_mask, (pull == LL_GPIO_PULL_UP) ? pin_mask : 0x0000U);
        MODIFY_REG(MCU_RET->DPAD_RE_N_BUS_1, pin_mask, (pull == LL_GPIO_PULL_NO) ? pin_mask : 0x0000U);
    }
    else
    {
        pin_mask <<=  ((GPIOx == GPIO0) ? 0 : 16);
        MODIFY_REG(MCU_RET->DPAD_RTYP_BUS_0, pin_mask, (pull == LL_GPIO_PULL_UP) ? pin_mask : 0x0000U);
        MODIFY_REG(MCU_RET->DPAD_RE_N_BUS_0, pin_mask, (pull == LL_GPIO_PULL_NO) ? pin_mask : 0x0000U);
    }
}

/**
  * @brief  Return gpio pull-up or pull-down for a dedicated pin on a dedicated port
  * @note   Warning: only one pin can be passed as parameter.
  *
  *  Register|BitsName
  *  --------|--------
  *  DPAD_RE_N_BUS | RE_N
  *  DPAD_RTYP_BUS | RTYP
  *
  * @param  GPIOx GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_PULL_NO
  *         @arg @ref LL_GPIO_PULL_UP
  *         @arg @ref LL_GPIO_PULL_DOWN
  */
__STATIC_INLINE uint32_t ll_gpio_get_pin_pull(gpio_regs_t *GPIOx, uint32_t pin)
{
    /* Get pin position in resitor related registers, GPIO0:0~15, GPIO1:16~31 */
    if(GPIO2 == GPIOx)
    {
        return ((READ_BITS(MCU_RET->DPAD_RE_N_BUS_1, pin) != RESET) ? LL_GPIO_PULL_NO :
                ((READ_BITS(MCU_RET->DPAD_RTYP_BUS_1, pin) != RESET) ? LL_GPIO_PULL_UP : LL_GPIO_PULL_DOWN));
    }
    else
    {
        pin <<=  ((GPIOx == GPIO0) ? 0 : 16);
        return ((READ_BITS(MCU_RET->DPAD_RE_N_BUS_0, pin) != RESET) ? LL_GPIO_PULL_NO :
                ((READ_BITS(MCU_RET->DPAD_RTYP_BUS_0, pin) != RESET) ? LL_GPIO_PULL_UP : LL_GPIO_PULL_DOWN));
    }
}

/**
  * @brief  Configure gpio pinmux number of a dedicated pin from 0 to 15 for a dedicated port.
  * @note   Possible values are from AF0 to AF15 depending on target.
  * @note   Warning: only one pin can be passed as parameter.
  *
  *  Register|BitsName
  *  --------|--------
  *  DPAD_MUX_CTRL0_7 | CTRL0_7
  *  DPAD_MUX_CTRL16_23 | CTRL16_23
  *
  * @param  GPIOx GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @param  mux This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_MUX_0
  *         @arg @ref LL_GPIO_MUX_1
  *         @arg @ref LL_GPIO_MUX_2
  *         @arg @ref LL_GPIO_MUX_3
  *         @arg @ref LL_GPIO_MUX_4
  *         @arg @ref LL_GPIO_MUX_5
  *         @arg @ref LL_GPIO_MUX_6
  *         @arg @ref LL_GPIO_MUX_7
  *         @arg @ref LL_GPIO_MUX_8
  * @retval None
  */
__STATIC_INLINE void ll_gpio_set_pin_mux(gpio_regs_t *GPIOx, uint32_t pin, uint32_t mux)
{
    volatile uint32_t* GPIO_MUX_REG_BASE = &(MCU_RET->DPAD_MUX_CTL_00_07);
    uint8_t  offset;
    uint32_t pos;

    pin = POSITION_VAL(pin);
    pos = (pin & 7) << 2; /* pos = (pin % 8) * 4 */

    if(GPIO0 == GPIOx)
    {
        offset = pin >> 3; /* offset = pin / 8 */
    }
    else if(GPIO1 == GPIOx)
    {
        offset = (pin >> 3) + 2; /* offset = pin / 8 + 2*/
    }
    else //GPIO2
    {
        offset = 6; /* DPAD_MUX_CTL_32_39 - DPAD_MUX_CTL_00_07 */
    }

    MODIFY_REG(*(GPIO_MUX_REG_BASE + offset), 0xF << pos, mux << pos);
}

/**
  * @brief  Return gpio alternate function of a dedicated pin from 0 to 15 for a dedicated port.
  * @note   Possible values are from AF0 to AF15 depending on target.
  *
  *  Register|BitsName
  *  --------|--------
  *  DPAD_MUX_CTRL8_15 | CTRL8_15
  *  DPAD_MUX_CTRL24_31 | CTRL24_31
  *
  * @param  GPIOx GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_MUX_0
  *         @arg @ref LL_GPIO_MUX_1
  *         @arg @ref LL_GPIO_MUX_2
  *         @arg @ref LL_GPIO_MUX_3
  *         @arg @ref LL_GPIO_MUX_4
  *         @arg @ref LL_GPIO_MUX_5
  *         @arg @ref LL_GPIO_MUX_6
  *         @arg @ref LL_GPIO_MUX_7
  *         @arg @ref LL_GPIO_MUX_8
  */
__STATIC_INLINE uint32_t ll_gpio_get_pin_mux(gpio_regs_t *GPIOx, uint32_t pin)
{
    uint32_t pos = POSITION_VAL(pin) << 2;
    if(GPIO0 == GPIOx)
    {
        if(pin < LL_GPIO_PIN_8)
        {
             return (READ_BITS(MCU_RET->DPAD_MUX_CTL_00_07, 0xF << pos) >> pos);
        }
        else
        {
            pos = POSITION_VAL(pin >> 8) << 2;
            return (READ_BITS(MCU_RET->DPAD_MUX_CTL_08_15, 0xF << pos) >> pos);
        }
    }
    else if(GPIO1 == GPIOx)
    {
        if(pin < LL_GPIO_PIN_8)
        {
            return (READ_BITS(MCU_RET->DPAD_MUX_CTL_16_23, 0xF << pos) >> pos);
        }
        else
        {
            pos = POSITION_VAL(pin >> 8) << 2;
            return (READ_BITS(MCU_RET->DPAD_MUX_CTL_24_31, 0xF << pos) >> pos);
        }
    }
    else
    {
        if(pin < LL_GPIO_PIN_8)
        {
            return (READ_BITS(MCU_RET->DPAD_MUX_CTL_32_39, 0xF << pos) >> pos);
        }
    }
    return 0;
}
/** @} */

/** @defgroup GPIO_LL_EF_Data_Access Data Access
  * @{
  */

/**
  * @brief  Return full input data register value for a dedicated port.
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  GPIOx GPIO Port
  * @retval Input data register value of port
  */
__STATIC_INLINE uint32_t ll_gpio_read_input_port(gpio_regs_t *GPIOx)
{
    return (uint32_t)(READ_REG(GPIOx->DATA));
}

/**
  * @brief  Return if input data level for several pins of dedicated port is high or low.
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  GPIOx GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_gpio_read_input_pin(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    return (READ_BITS(GPIOx->DATA, pin_mask) == (pin_mask));
}

/**
  * @brief  Write output data register for the port.
  *
  *  Register|BitsName
  *  --------|--------
  *  DATAOUT | DATAOUT
  *
  * @param  GPIOx GPIO Port
  * @param  port_value Level value for each pin of the port
  * @retval None
  */
__STATIC_INLINE void ll_gpio_write_output_port(gpio_regs_t *GPIOx, uint32_t port_value)
{
    WRITE_REG(GPIOx->DATAOUT, port_value);
}

/**
  * @brief  Return full output data register value for a dedicated port.
  *
  *  Register|BitsName
  *  --------|--------
  *  DATAOUT | DATAOUT
  *
  * @param  GPIOx GPIO Port
  * @retval Output data register value of port
  */
__STATIC_INLINE uint32_t ll_gpio_read_output_port(gpio_regs_t *GPIOx)
{
    return (uint32_t)(READ_REG(GPIOx->DATAOUT));
}

/**
  * @brief  Return if output data level for several pins of dedicated port is high or low.
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATAOUT
  *
  * @param  GPIOx GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_gpio_read_output_pin(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    return (READ_BITS(GPIOx->DATAOUT, pin_mask) == (pin_mask));
}

/**
  * @brief  Set several pins to high level on dedicated gpio port.
  *
  *  Register|BitsName
  *  --------|--------
  *  DATAOUT | DATAOUT
  *
  * @param  GPIOx GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_gpio_set_output_pin(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
#ifdef USE_GPIO_MASK_REGISTER
    WRITE_REG(GPIOx->MASKLOWBYTE[(uint8_t)pin_mask], pin_mask & GPIO_MASKLOWBYTE_DATA);
    WRITE_REG(GPIOx->MASKHIGHBYTE[(uint8_t)(pin_mask >> GPIO_MASKHIGHBYTE_DATA_Pos)],
              pin_mask & GPIO_MASKHIGHBYTE_DATA);
#else
    SET_BITS(GPIOx->DATAOUT, pin_mask);
#endif
}

/**
  * @brief  Set several pins to low level on dedicated gpio port.
  *
  *  Register|BitsName
  *  --------|--------
  *  DATAOUT | DATAOUT
  *
  * @param  GPIOx GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_gpio_reset_output_pin(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
#ifdef USE_GPIO_MASK_REGISTER
    WRITE_REG(GPIOx->MASKLOWBYTE[(uint8_t)pin_mask], 0x0000U);
    WRITE_REG(GPIOx->MASKHIGHBYTE[(uint8_t)(pin_mask >> 8)], 0x0000U);
#else
    CLEAR_BITS(GPIOx->DATAOUT, pin_mask);
#endif
}

/**
  * @brief  Toggle data value for several pin of dedicated port.
  *
  *  Register|BitsName
  *  --------|--------
  *  DATAOUT | DATAOUT
  *
  * @param  GPIOx GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_gpio_toggle_pin(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    WRITE_REG(GPIOx->DATAOUT, READ_REG(GPIOx->DATAOUT) ^ pin_mask);
}

/** @} */

/** @defgroup GPIO_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable GPIO Falling Edge Trigger for pins in the range of 0 to 15.
  * @note
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLCLR | INTPOLCLR
  *  INTTYPESET | INTTYPESET
  *
  * @param  GPIOx GPIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_gpio_enable_falling_trigger(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    WRITE_REG(GPIOx->INTPOLCLR, pin_mask);
    WRITE_REG(GPIOx->INTTYPESET, pin_mask);
    WRITE_REG(GPIOx->INTDBECLR, pin_mask);
}

/**
  * @brief  Enable GPIO Rising Edge Trigger for pins in the range of 0 to 15.
  * @note
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLSET | INTPOLSET
  *  INTTYPESET | INTTYPESET
  *
  * @param  GPIOx GPIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_gpio_enable_rising_trigger(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    WRITE_REG(GPIOx->INTPOLSET, pin_mask);
    WRITE_REG(GPIOx->INTTYPESET, pin_mask);
    WRITE_REG(GPIOx->INTDBECLR, pin_mask);
}

/**
  * @brief  Enable GPIO High Level Trigger for pins in the range of 0 to 15.
  * @note
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLSET | INTPOLSET
  *  INTTYPECLR | INTTYPECLR
  *
  * @param  GPIOx GPIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_gpio_enable_high_trigger(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    WRITE_REG(GPIOx->INTPOLSET, pin_mask);
    WRITE_REG(GPIOx->INTTYPECLR, pin_mask);
}

/**
  * @brief  Enable GPIO Low Level Trigger for pins in the range of 0 to 15.
  * @note
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLCLR | INTPOLCLR
  *  INTTYPECLR | INTTYPECLR
  *
  * @param  GPIOx GPIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_gpio_enable_low_trigger(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    WRITE_REG(GPIOx->INTPOLCLR, pin_mask);
    WRITE_REG(GPIOx->INTTYPECLR, pin_mask);
}

/**
  * @brief  Enable GPIO both edge Trigger for pins in the range of 0 to 15.
  * @note
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLCLR | INTPOLCLR
  *  INTTYPESET | INTTYPESET
  *  INTTYPESET | INTTYPESET
  *
  * @param  GPIOx GPIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_gpio_enable_both_edge_trigger(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    WRITE_REG(GPIOx->INTPOLCLR, pin_mask);
    WRITE_REG(GPIOx->INTTYPESET, pin_mask);
    WRITE_REG(GPIOx->INTDBESET, pin_mask);
}

/**
  * @brief  Get trigger type for pins in the range of 0 to 15
  * @note
  *
  *  Register|BitsName
  *  --------|--------
  *  INTPOLCLR | INTPOLCLR
  *  INTTYPECLR | INTTYPECLR
  *  INTTYPESET | INTTYPESET
  *
  * @param  GPIOx GPIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_TRIGGER_RISING
  *         @arg @ref LL_GPIO_TRIGGER_FALLING
  *         @arg @ref LL_GPIO_TRIGGER_HIGH
  *         @arg @ref LL_GPIO_TRIGGER_LOW
  *         @arg @ref LL_GPIO_TRIGGER_BOTH_EDGE
  */
__STATIC_INLINE uint32_t ll_gpio_get_trigger_type(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
/*                  INTTYPESET  INTPOLSET   INTDBESET
TRIGGER_FALLING           1          0          0
TRIGGER_RISING            1          1          0
TRIGGER_BOTH_EDGE         1          X          1
TRIGGER_HIGH              0          1          X
TRIGGER_LOW               0          0          X
*/
    #define TYPE_SET ((uint32_t)0x4U) /**< GPIO INTTYPESET is set */
    #define POL_SET  ((uint32_t)0x2U) /**< GPIO INTPOLSET is set  */
    #define DBE_SET  ((uint32_t)0x1U) /**< GPIO INTDBESET is set  */

    uint32_t pos = POSITION_VAL(pin_mask);
    uint32_t type = (READ_BITS(GPIOx->INTTYPESET, pin_mask) >> pos ) << 2;
    uint32_t pol = (READ_BITS(GPIOx->INTPOLSET, pin_mask) >> pos) << 1;
    uint32_t both_egde = (READ_BITS(GPIOx->INTDBESET, pin_mask) >> pos);

    switch (type | pol | both_egde)
    {
    case TYPE_SET:
        return LL_GPIO_TRIGGER_FALLING;
    case TYPE_SET | POL_SET:
        return LL_GPIO_TRIGGER_RISING;
    case TYPE_SET | DBE_SET:
        return LL_GPIO_TRIGGER_BOTH_EDGE;
    case TYPE_SET | POL_SET | DBE_SET:
        return LL_GPIO_TRIGGER_BOTH_EDGE;
    case POL_SET:
        return LL_GPIO_TRIGGER_HIGH;
    default:
        return LL_GPIO_TRIGGER_LOW;
    }
}

/**
  * @brief  Enable GPIO interrupts for pins in the range of 0 to 15.
  * @note   @ref GPIO_LL_EC_TRIGGER can be used to specify the interrupt trigger type
  *
  *  Register|BitsName
  *  --------|--------
  *  INTENSET | INTENSET
  *
  * @param  GPIOx GPIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_gpio_enable_it(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    WRITE_REG(GPIOx->INTENSET, pin_mask);
}

/**
  * @brief  Disable GPIO interrupts for pins in the range of 0 to 15.
  * @note   @ref GPIO_LL_EC_TRIGGER can be used to specify the interrupt trigger type
  *
  *  Register|BitsName
  *  --------|--------
  *  INTENCLR | INTENCLR
  *
  * @param  GPIOx GPIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_gpio_disable_it(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    WRITE_REG(GPIOx->INTENCLR, pin_mask);
}

/**
  * @brief  Check if the Interrupt of specified GPIO pins is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTENSET | INTENSET
  *
  * @param  GPIOx GPIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_gpio_is_enabled_it(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    return (READ_BITS(GPIOx->INTENSET, pin_mask) == (pin_mask));
}

/** @} */

/** @defgroup GPIO_LL_EF_Flag_Management Flag_Management
 * @{
 */

/**
  * @brief  Read GPIO Interrupt Combination Flag for pins in the range of 0 to 15
  * @note   After an interrupt is triggered, the corresponding bit in the INTSTATUS Register is set.
  *         The interrupt status can be cleared by writing 1 to corresponding bit in INTCLEAR Register.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTATUS | INTSTATUS
  *
  * @param  GPIOx GPIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval Interrupt flag whose bits were set when the selected trigger event arrives on the interrupt
  */
__STATIC_INLINE uint32_t ll_gpio_read_flag_it(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    return (uint32_t)(READ_BITS(GPIOx->INTSTAT, pin_mask));
}

/**
  * @brief  Indicates if the GPIO Interrupt Flag is set or not for pins in the range of 0 to 15.
  * @note   After an interrupt is triggered, the corresponding bit in the INTSTATUS Register is set.
  *         The interrupt status can be cleared by writing 1 to corresponding bit in INTCLEAR Register.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTATUS | INTSTATUS
  *
  * @param  GPIOx GPIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_gpio_is_active_flag_it(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    return (READ_BITS(GPIOx->INTSTAT, pin_mask) == pin_mask);
}

/**
  * @brief  Clear Interrupt Status flag for pins in the range of 0 to 15.
  * @note   After an interrupt is triggered, the corresponding bit in the INTSTATUS Register is set.
  *         The interrupt status can be cleared by writing 1 to corresponding bit in INTCLEAR Register.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTATUS | INTSTATUS
  *
  * @param  GPIOx GPIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_gpio_clear_flag_it(gpio_regs_t *GPIOx, uint32_t pin_mask)
{
    WRITE_REG(GPIOx->INTSTAT, pin_mask);
}

/** @} */

/** @defgroup GPIO_LL_EF_Init Initialization and de-initialization functions
  * @{
  */
/**
  * @brief  Initialize GPIO registers according to the specified
  *         parameters in p_gpio_init.
  * @param  GPIOx       GPIO instance.
  * @param  p_gpio_init   Pointer to a ll_gpio_init_t structure that contains the configuration
  *                     information for the specified GPIO peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: GPIO registers are initialized according to p_gpio_init content
  *          - ERROR: Problem occurred during GPIO Registers initialization
  */
error_status_t ll_gpio_init(gpio_regs_t *GPIOx, ll_gpio_init_t *p_gpio_init);
/** @} */

/** @} */

#endif /* defined (GPIO0) || defined (GPIO1) */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_GPIO_H__ */

/** @} */

/** @} */

/** @} */

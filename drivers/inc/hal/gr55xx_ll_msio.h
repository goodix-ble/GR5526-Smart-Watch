/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_msio.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of MSIO LL library.
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

/** @defgroup LL_MSIO MSIO
  * @brief MSIO LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_MSIO_H__
#define __GR55XX_LL_MSIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined(AON_MSIO) || defined(MCU_RET)

/** @defgroup MSIO_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup MSIO_LL_ES_INIT MSIO Exported init structures
  * @{
  */

/**
  * @brief MSIO pad Enumerations definition
  */
typedef enum
{
    MSIOA        = 0x00,    /**< MSIO_A_PAD             */
} msio_pad_t;


/**
  * @brief LL MSIO init Structure definition
  */
typedef struct _ll_msio_init
{
    uint32_t pin;           /**< Specifies the MSIO pins to be MSIO_InitStructured.
                                 This parameter can be any value of @ref MSIO_LL_EC_PIN */

    uint32_t direction;     /**< Specifies the direction for the selected pins.
                                 This parameter can be a value of @ref MSIO_LL_EC_DIRECTION.

                                 MSIO HW MSIO_InitStructuration can be modified afterwards using unitary function @ref ll_msio_set_pin_direction(). */

    uint32_t mode;          /**< Specifies the operating mode for the selected pins.
                                 This parameter can be a value of @ref MSIO_LL_EC_MODE.

                                 MSIO HW MSIO_InitStructuration can be modified afterwards using unitary function @ref ll_msio_set_pin_mode(). */

    uint32_t pull;          /**< Specifies the operating Pull-up/Pull down for the selected pins.
                                 This parameter can be a value of @ref MSIO_LL_EC_PULL.

                                 MSIO HW configuration can be modified afterwards using unitary function @ref ll_msio_set_pin_pull().*/

    uint32_t mux;           /*!< Specifies the Peripheral to be connected to the selected pins.
                                This parameter can be a value of @ref MSIO_LL_EC_MUX.

                                GPIO HW MSIO_InitStructuration can be modified afterwards using unitary function
                                @ref ll_msio_set_pin_mux(). */

} ll_msio_init_t;

/** @} */

/** @} */

/**
  * @defgroup  MSIO_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup MSIO_LL_Exported_Constants MSIO Exported Constants
  * @{
  */

/** @defgroup MSIO_LL_EC_PIN PIN
  * @{
  */
#define LL_MSIO_PIN_0               ((uint32_t)0x01U) /**< Select pin 0    */
#define LL_MSIO_PIN_1               ((uint32_t)0x02U) /**< Select pin 1    */
#define LL_MSIO_PIN_2               ((uint32_t)0x04U) /**< Select pin 2    */
#define LL_MSIO_PIN_3               ((uint32_t)0x08U) /**< Select pin 3    */
#define LL_MSIO_PIN_4               ((uint32_t)0x10U) /**< Select pin 4    */
#define LL_MSIO_PIN_5               ((uint32_t)0x20U) /**< Select pin 5    */
#define LL_MSIO_PIN_6               ((uint32_t)0x40U) /**< Select pin 6    */
#define LL_MSIO_PIN_7               ((uint32_t)0x80U) /**< Select pin 7    */
#define LL_MSIO_PIN_ALL             ((uint32_t)0xFFU) /**< Select all pins */

/** @} */

/** @defgroup MSIO_LL_EC_DIRECTION Direction
  * @{
  */
#define LL_MSIO_DIRECTION_NONE      ((uint32_t)0x0U)  /**< Disable input/output  */
#define LL_MSIO_DIRECTION_INPUT     ((uint32_t)0x1U)  /**< Enable input          */
#define LL_MSIO_DIRECTION_OUTPUT    ((uint32_t)0x2U)  /**< Enable output         */
#define LL_MSIO_DIRECTION_INOUT     ((uint32_t)0x3U)  /**< Enable input&output   */
/** @} */

/** @defgroup MSIO_LL_EC_MODE Mode
  * @{
  */
#define LL_MSIO_MODE_ANALOG         ((uint32_t)0x0U)  /**< Select analog mode   */
#define LL_MSIO_MODE_DIGITAL        ((uint32_t)0x1U)  /**< Enable digital mode  */
/** @} */

/** @defgroup MSIO_LL_EC_PULL Pull Up Pull Down
  * @{
  */
#define LL_MSIO_PULL_NO             ((uint32_t)0x0U)  /**< Select I/O no pull   */
#define LL_MSIO_PULL_UP             ((uint32_t)0x1U)  /**< Select I/O pull up   */
#define LL_MSIO_PULL_DOWN           ((uint32_t)0x2U)  /**< Select I/O pull down */
/** @} */

/** @defgroup MSIO_LL_EC_MUX Alternate Function
  * @{
  */
#define LL_MSIO_MUX_0               ((uint32_t)0x0U)  /*!< Select alternate function 0 */
#define LL_MSIO_MUX_1               ((uint32_t)0x1U)  /*!< Select alternate function 1 */
#define LL_MSIO_MUX_2               ((uint32_t)0x2U)  /*!< Select alternate function 2 */
#define LL_MSIO_MUX_3               ((uint32_t)0x3U)  /*!< Select alternate function 3 */
#define LL_MSIO_MUX_4               ((uint32_t)0x4U)  /*!< Select alternate function 4 */
#define LL_MSIO_MUX_5               ((uint32_t)0x5U)  /*!< Select alternate function 5 */
#define LL_MSIO_MUX_6               ((uint32_t)0x6U)  /*!< Select alternate function 6 */
#define LL_MSIO_MUX_7               ((uint32_t)0x7U)  /*!< Select alternate function 7 */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup MSIO_LL_Exported_Macros MSIO Exported Macros
  * @{
  */

/** @defgroup MSIO_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in MSIO register
  * @param  __instance__ MSIO instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_MSIO_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in MSIO register
  * @param  __instance__ MSIO instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_MSIO_ReadReg(__instance__, __REG__) READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup MSIO_LL_Private_Macros MSIO Private Macros
  * @{
  */

/** @defgroup MSIO_LL_EC_DEFAULT_CONFIG InitStruct default configuration
  * @{
  */

/**
  * @brief LL MSIO InitStrcut default configuration
  */
#define LL_MSIO_DEFAULT_CONFIG                      \
{                                                   \
    .pin        = LL_MSIO_PIN_ALL,                  \
    .direction  = LL_MSIO_DIRECTION_INPUT,          \
    .mode       = LL_MSIO_MODE_DIGITAL,             \
    .pull       = LL_MSIO_PULL_DOWN,                \
    .mux        = LL_MSIO_MUX_7,                    \
}
/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup MSIO_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup MSIO_LL_EF_Port_Configuration Port Configuration
  * @{
  */

/**
  * @brief  Set several MSIO pins to input/output direction.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_0 | OE_N
  *  MSIO_PAD_CFG_0 | IE_N
  *
  * @param  MSIOx MSIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  *         @arg @ref LL_MSIO_PIN_ALL
  * @param  direction This parameter can be one of the following values:
  *         @arg @ref LL_MSIO_DIRECTION_NONE
  *         @arg @ref LL_MSIO_DIRECTION_INPUT
  *         @arg @ref LL_MSIO_DIRECTION_OUTPUT
  *         @arg @ref LL_MSIO_DIRECTION_INOUT
  * @retval None
  */
__STATIC_INLINE void ll_msio_set_pin_direction(msio_pad_t MSIOx, uint32_t pin_mask, uint32_t direction)
{
    uint32_t oe_mask = (pin_mask << AON_MSIO_MSIO_A_PAD_CFG0_OUT_EN_POS) & AON_MSIO_MSIO_A_PAD_CFG0_OUT_EN;
    uint32_t ie_mask = (pin_mask << AON_MSIO_MSIO_A_PAD_CFG1_IN_EN_POS) & AON_MSIO_MSIO_A_PAD_CFG1_IN_EN;

    if (direction != LL_MSIO_DIRECTION_NONE)
    {
        if (direction != LL_MSIO_DIRECTION_INOUT)
        {
            MODIFY_REG(AON_MSIO->MSIO_A_PAD_CFG0, oe_mask, (direction == LL_MSIO_DIRECTION_OUTPUT) ? 0x00 : oe_mask);
            MODIFY_REG(AON_MSIO->MSIO_A_PAD_CFG1, ie_mask, (direction == LL_MSIO_DIRECTION_INPUT) ? 0x00 : ie_mask);
        }
        else
        {
            CLEAR_BITS(AON_MSIO->MSIO_A_PAD_CFG0, oe_mask);
            CLEAR_BITS(AON_MSIO->MSIO_A_PAD_CFG1, ie_mask);
        }
    }
    else
    {
        SET_BITS(AON_MSIO->MSIO_A_PAD_CFG0, oe_mask);
        SET_BITS(AON_MSIO->MSIO_A_PAD_CFG1, ie_mask);
    }
}

/**
  * @brief  Return gpio direction for a MSIO pin.
  * @note   I/O direction can be Input direction, General purpose output.
  * @note   Warning: only one pin can be passed as parameter.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_0 | OE_N
  *  MSIO_PAD_CFG_0 | IE_N
  *
  * @param  MSIOx MSIO instance.
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_MSIO_DIRECTION_NONE
  *         @arg @ref LL_MSIO_DIRECTION_INPUT
  *         @arg @ref LL_MSIO_DIRECTION_OUTPUT
  *         @arg @ref LL_MSIO_DIRECTION_INOUT
  */
__STATIC_INLINE uint32_t ll_msio_get_pin_direction(msio_pad_t MSIOx, uint32_t pin)
{
    uint32_t oe_mask = (pin << AON_MSIO_MSIO_A_PAD_CFG0_OUT_EN_POS) & AON_MSIO_MSIO_A_PAD_CFG0_OUT_EN;
    uint32_t ie_mask = (pin << AON_MSIO_MSIO_A_PAD_CFG1_IN_EN_POS) & AON_MSIO_MSIO_A_PAD_CFG1_IN_EN;
    uint32_t mask = READ_BITS(AON_MSIO->MSIO_A_PAD_CFG0, oe_mask);
    mask |= READ_BITS(AON_MSIO->MSIO_A_PAD_CFG1, ie_mask);
    if (mask == (ie_mask | oe_mask))
        return LL_MSIO_DIRECTION_NONE;
    else
    {
        if (mask == 0)
            return LL_MSIO_DIRECTION_INOUT;
        else
            return ((mask == ie_mask) ? LL_MSIO_DIRECTION_OUTPUT : LL_MSIO_DIRECTION_INPUT);
    }
}

/**
  * @brief  Set several MSIO pins to analog/digital mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_1 | AE_N
  *
  * @param  MSIOx MSIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  *         @arg @ref LL_MSIO_PIN_ALL
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_MSIO_MODE_ANALOG
  *         @arg @ref LL_MSIO_MODE_DIGITAL
  * @retval None
  */
__STATIC_INLINE void ll_msio_set_pin_mode(msio_pad_t MSIOx, uint32_t pin_mask, uint32_t mode)
{
    uint32_t ae_mask = (pin_mask << AON_MSIO_MSIO_A_PAD_CFG1_A_EN_POS) & AON_MSIO_MSIO_A_PAD_CFG1_A_EN;
    uint32_t ae_n = (mode == LL_MSIO_MODE_ANALOG) ? 0U : ae_mask;
    MODIFY_REG(AON_MSIO->MSIO_A_PAD_CFG1, ae_mask, ae_n);
}

/**
  * @brief  Return gpio mode for a MSIO pin.
  * @note   I/O mode can be analog or digital.
  * @note   Warning: only one pin can be passed as parameter.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_1 | AE_N
  *
  * @param  MSIOx MSIO instance.
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_MSIO_MODE_ANALOG
  *         @arg @ref LL_MSIO_MODE_DIGITAL
  */
__STATIC_INLINE uint32_t ll_msio_get_pin_mode(msio_pad_t MSIOx, uint32_t pin)
{
    uint32_t ae_mask = (pin << AON_MSIO_MSIO_A_PAD_CFG1_A_EN_POS) & AON_MSIO_MSIO_A_PAD_CFG1_A_EN;
    return ((READ_BITS(AON_MSIO->MSIO_A_PAD_CFG1, ae_mask) == ae_mask) ? LL_MSIO_MODE_DIGITAL : LL_MSIO_MODE_ANALOG);
}

/**
  * @brief  Configure gpio pull-up or pull-down for a dedicated MSIO pin.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_0 | RE_N
  *  MSIO_PAD_CFG_1 | RTYPE
  *
  * @param  MSIOx MSIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  *         @arg @ref LL_MSIO_PIN_ALL
  * @param  pull This parameter can be one of the following values:
  *         @arg @ref LL_MSIO_PULL_NO
  *         @arg @ref LL_MSIO_PULL_UP
  *         @arg @ref LL_MSIO_PULL_DOWN
  * @retval None
  */
__STATIC_INLINE void ll_msio_set_pin_pull(msio_pad_t MSIOx, uint32_t pin_mask, uint32_t pull)
{
    if (pull != LL_MSIO_PULL_NO)
    {
        uint32_t rtype_mask = (pin_mask << AON_MSIO_MSIO_A_PAD_CFG1_R_TYPE_POS) & AON_MSIO_MSIO_A_PAD_CFG1_R_TYPE;
        uint32_t rtype = (pull != LL_MSIO_PULL_UP) ? 0U : rtype_mask;
        MODIFY_REG(AON_MSIO->MSIO_A_PAD_CFG1, rtype_mask, rtype);
        CLEAR_BITS(AON_MSIO->MSIO_A_PAD_CFG1,
                   (pin_mask << AON_MSIO_MSIO_A_PAD_CFG1_R_EN_POS) & AON_MSIO_MSIO_A_PAD_CFG1_R_EN);
    }
    else
    {
        SET_BITS(AON_MSIO->MSIO_A_PAD_CFG1,
                 (pin_mask << AON_MSIO_MSIO_A_PAD_CFG1_R_EN_POS) & AON_MSIO_MSIO_A_PAD_CFG1_R_EN);
    }
}

/**
  * @brief  Return gpio pull-up or pull-down for a dedicated MSIO pin.
  * @note   Warning: only one pin can be passed as parameter.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_0 | RE_N
  *  MSIO_PAD_CFG_1 | RTYPE
  *
  * @param  MSIOx MSIO instance.
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_MSIO_PULL_NO
  *         @arg @ref LL_MSIO_PULL_UP
  *         @arg @ref LL_MSIO_PULL_DOWN
  */
__STATIC_INLINE uint32_t ll_msio_get_pin_pull(msio_pad_t MSIOx, uint32_t pin)
{
    if (READ_BITS(AON_MSIO->MSIO_A_PAD_CFG1,
                  (pin << AON_MSIO_MSIO_A_PAD_CFG1_R_EN_POS) & AON_MSIO_MSIO_A_PAD_CFG1_R_EN))
    {
        return LL_MSIO_PULL_NO;
    }
    else
    {
        uint32_t rtype_mask = (pin << AON_MSIO_MSIO_A_PAD_CFG1_R_TYPE_POS) & AON_MSIO_MSIO_A_PAD_CFG1_R_TYPE;
        return ((READ_BITS(AON_MSIO->MSIO_A_PAD_CFG1, rtype_mask) != RESET) ? LL_MSIO_PULL_UP : LL_MSIO_PULL_DOWN);
    }
}

/**
  * @brief  Configure gpio pinmux number of a dedicated pin from 0 to 4 for a dedicated port.
  * @note   Possible values are from AF0 to AF7 depending on target.
  * @note   Warning: only one pin can be passed as parameter.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_MUX_CTL | CTL_00_04
  *  MSIO_PAD_CFG_1   | MCU_OVR
  *
  * @param  MSIOx MSIO instance.
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  * @param  mux This parameter can be one of the following values:
  *         @arg @ref LL_MSIO_MUX_0
  *         @arg @ref LL_MSIO_MUX_1
  *         @arg @ref LL_MSIO_MUX_2
  *         @arg @ref LL_MSIO_MUX_3
  *         @arg @ref LL_MSIO_MUX_4
  *         @arg @ref LL_MSIO_MUX_5
  *         @arg @ref LL_MSIO_MUX_6
  *         @arg @ref LL_MSIO_MUX_7
  * @retval None
  */
__STATIC_INLINE void ll_msio_set_pin_mux(msio_pad_t MSIOx, uint32_t pin, uint32_t mux)
{
    uint32_t pos = POSITION_VAL(pin) << 2;
    if (LL_MSIO_MUX_7 == mux)
    {
        CLEAR_BITS(AON_MSIO->MSIO_MCU_OVR, pin << AON_MSIO_MSIO_MCU_OVR_MSIO_OVR_POS);
    }
    else
    {
        MODIFY_REG(MCU_RET->MSIO_A_PAD_MUX_CTL, 0xF << pos, mux << pos);
        SET_BITS(AON_MSIO->MSIO_MCU_OVR, pin << AON_MSIO_MSIO_MCU_OVR_MSIO_OVR_POS);
    }
}

/**
  * @brief  Return gpio alternate function of a dedicated pin from 0 to 4 for a dedicated port.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_MUX_CTL | CTL_00_04
  *  MSIO_PAD_CFG_1   | MCU_OVR
  *
  * @param  MSIOx MSIO instance.
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_MSIO_MUX_0
  *         @arg @ref LL_MSIO_MUX_1
  *         @arg @ref LL_MSIO_MUX_2
  *         @arg @ref LL_MSIO_MUX_3
  *         @arg @ref LL_MSIO_MUX_4
  *         @arg @ref LL_MSIO_MUX_5
  *         @arg @ref LL_MSIO_MUX_6
  *         @arg @ref LL_MSIO_MUX_7
  */
__STATIC_INLINE uint32_t ll_msio_get_pin_mux(msio_pad_t MSIOx, uint32_t pin)
{
    if (READ_BITS(AON_MSIO->MSIO_MCU_OVR, pin << AON_MSIO_MSIO_MCU_OVR_MSIO_OVR_POS))
    {
        uint32_t pos = POSITION_VAL(pin) << 2;
        return (READ_BITS(MCU_RET->MSIO_A_PAD_MUX_CTL, 0xF << pos) >> pos);
    }
    else
    {
        return LL_MSIO_MUX_7;
    }
}

/** @} */

/** @defgroup MSIO_LL_EF_Data_Access Data Access
  * @{
  */

/**
  * @brief  Return full input data register value of MSIO.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_REG0 | MSIO_C
  *
  * @param  MSIOx MSIO instance.
  * @retval Input data register value of port
  */
__STATIC_INLINE uint32_t ll_msio_read_input_port(msio_pad_t MSIOx)
{
    return (uint32_t)(READ_BITS(AON_MSIO->MSIO_A_PAD_CFG0, AON_MSIO_MSIO_A_PAD_CFG0_IN) >> AON_MSIO_MSIO_A_PAD_CFG0_IN_POS);
}

/**
  * @brief  Return if input data level of several MSIO pins is high or low.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_0 | IN
  *
  * @param  MSIOx MSIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  *         @arg @ref LL_MSIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_msio_read_input_pin(msio_pad_t MSIOx, uint32_t pin_mask)
{
    pin_mask = (pin_mask << AON_MSIO_MSIO_A_PAD_CFG0_IN_POS) & AON_MSIO_MSIO_A_PAD_CFG0_IN;
    return (uint32_t)(READ_BITS(AON_MSIO->MSIO_A_PAD_CFG0, pin_mask) == pin_mask);
}

/**
  * @brief  Return if input data level of several MSIO pins is high or low.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_0 | OUT
  *
  * @param  MSIOx MSIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  *         @arg @ref LL_MSIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_msio_read_output_pin(msio_pad_t MSIOx, uint32_t pin_mask)
{
    pin_mask = (pin_mask << AON_MSIO_MSIO_A_PAD_CFG0_OUT_POS) & AON_MSIO_MSIO_A_PAD_CFG0_OUT;
    return (uint32_t)(READ_BITS(AON_MSIO->MSIO_A_PAD_CFG0, pin_mask) == pin_mask);
}

/**
  * @brief  Write output data register of MSIO.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_0 | IN
  *
  * @param  MSIOx MSIO instance.
  * @param  port_value Level value for each pin of the port
  * @retval None
  */
__STATIC_INLINE void ll_msio_write_output_port(msio_pad_t MSIOx, uint32_t port_value)
{
    MODIFY_REG(AON_MSIO->MSIO_A_PAD_CFG0, AON_MSIO_MSIO_A_PAD_CFG0_OUT, (port_value << AON_MSIO_MSIO_A_PAD_CFG0_OUT_POS) & AON_MSIO_MSIO_A_PAD_CFG0_OUT);
}
/**
  * @brief  Return full output data register value of MSIO.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_0 | IN
  *
  * @param  MSIOx MSIO instance.
  * @retval Output data register value of port
  */
__STATIC_INLINE uint32_t ll_msio_read_output_port(msio_pad_t MSIOx)
{
    return (uint32_t)(READ_BITS(AON_MSIO->MSIO_A_PAD_CFG0, AON_MSIO_MSIO_A_PAD_CFG0_OUT) >> AON_MSIO_MSIO_A_PAD_CFG0_OUT_POS);
}

/**
  * @brief  Set specified MSIO pins to high level
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_0 | IN
  *
  * @param  MSIOx MSIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  *         @arg @ref LL_MSIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_msio_set_output_pin(msio_pad_t MSIOx, uint32_t pin_mask)
{
    SET_BITS(AON_MSIO->MSIO_A_PAD_CFG0, (pin_mask << AON_MSIO_MSIO_A_PAD_CFG0_OUT_POS) & AON_MSIO_MSIO_A_PAD_CFG0_OUT);
}

/**
  * @brief  Set specified MSIO pins to low level.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_0 | IN
  *
  * @param  MSIOx MSIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  *         @arg @ref LL_MSIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_msio_reset_output_pin(msio_pad_t MSIOx, uint32_t pin_mask)
{
    CLEAR_BITS(AON_MSIO->MSIO_A_PAD_CFG0, (pin_mask << AON_MSIO_MSIO_A_PAD_CFG0_OUT_POS) & AON_MSIO_MSIO_A_PAD_CFG0_OUT);
}

/**
  * @brief  Toggle data value of specified MSIO pins.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_0 | IN
  *
  * @param  MSIOx MSIO instance.
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_MSIO_PIN_0
  *         @arg @ref LL_MSIO_PIN_1
  *         @arg @ref LL_MSIO_PIN_2
  *         @arg @ref LL_MSIO_PIN_3
  *         @arg @ref LL_MSIO_PIN_4
  *         @arg @ref LL_MSIO_PIN_5
  *         @arg @ref LL_MSIO_PIN_6
  *         @arg @ref LL_MSIO_PIN_7
  *         @arg @ref LL_MSIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void ll_msio_toggle_pin(msio_pad_t MSIOx, uint32_t pin_mask)
{
    WRITE_REG(AON_MSIO->MSIO_A_PAD_CFG0, (READ_REG(AON_MSIO->MSIO_A_PAD_CFG0) ^ ((pin_mask << AON_MSIO_MSIO_A_PAD_CFG0_OUT_POS) & AON_MSIO_MSIO_A_PAD_CFG0_OUT)));
}

/** @} */

/** @defgroup MSIO_LL_EF_Init Initialization and de-initialization functions
  * @{
  */
/**
  * @brief  Initialize MSIO registers according to the specified.
  *         parameters in p_msio_init.
  *
  * @param  MSIOx MSIO instance.
  * @param  p_msio_init Pointer to a ll_msio_init_t structure that contains the configuration
  *                             information for the specified MSIO peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: MSIO registers are initialized according to p_msio_init content
  *          - ERROR: Problem occurred during MSIO Registers initialization
  */
error_status_t ll_msio_init(msio_pad_t MSIOx,ll_msio_init_t *p_msio_init);

/** @} */

/** @} */

#endif /* defined(AON_MSIO) || defined(MCU_RET) */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_MSIO_H__ */

/** @} */

/** @} */

/** @} */

/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_aon_gpio.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of AON GPIO HAL library.
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

/** @addtogroup HAL_DRIVER HAL Driver
  * @{
  */

/** @defgroup HAL_AON_GPIO AON_GPIO
  * @brief AON_GPIO HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_AON_GPIO_H__
#define __GR55xx_HAL_AON_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_aon_gpio.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_AON_GPIO_ENUMERATIONS Enumerations
  * @{
  */

/**
  * @brief  AON_GPIO  Bit SET and Bit RESET enumerations
  */
typedef enum
{
    AON_GPIO_PIN_RESET = 0U,          /**< AON GPIO pin low level.*/
    AON_GPIO_PIN_SET                  /**< AON GPIO pin high level.*/
} aon_gpio_pin_state_t;

/** @} */

/** @addtogroup HAL_AON_GPIO_STRUCTURES Structures
  * @{
  */

/**
  * @brief   AON_GPIO init structure definition
  */
typedef struct _aon_gpio_init
{
    uint32_t pin;       /**< Specifies the AON_GPIO pins to be configured.
                            This parameter can be any value of @ref AON_GPIO_Pins */

    uint32_t mode;      /**< Specifies the operating mode for the selected pins.
                             This parameter can be a value of @ref AON_GPIO_Mode */

    uint32_t pull;      /**< Specifies the Pull-up or Pull-Down activation for the selected pins.
                             This parameter can be a value of @ref AON_GPIO_Pull */

    uint32_t mux;       /**< Specifies the Peripheral to be connected to the selected pins.
                             This parameter can be a value of @ref GPIOEx_Mux_Function_Selection. */
} aon_gpio_init_t;

/** @} */

/** @addtogroup HAL_AON_GPIO_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup AON_GPIO_Callback AON_GPIO Callback
  * @{
  */

/**
  * @brief HAL AON_GPIO Callback function definition
  */
typedef struct _aon_gpio_callback
{
    void (*aon_gpio_callback)(uint16_t aon_gpio_pin);           /**< AON GPIO pin detection callback   */
} aon_gpio_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_AON_GPIO_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup AON_GPIO_Exported_Constants AON_GPIO Exported Constants
  * @{
  */

/** @defgroup AON_GPIO_Pins AON_GPIO pins
  * @{
  */
#define AON_GPIO_PIN_0                 ((uint16_t)0x0001U)  /**< Pin 0 selected    */
#define AON_GPIO_PIN_1                 ((uint16_t)0x0002U)  /**< Pin 1 selected    */
#define AON_GPIO_PIN_2                 ((uint16_t)0x0004U)  /**< Pin 2 selected    */
#define AON_GPIO_PIN_3                 ((uint16_t)0x0008U)  /**< Pin 3 selected    */
#define AON_GPIO_PIN_4                 ((uint16_t)0x0010U)  /**< Pin 4 selected    */
#define AON_GPIO_PIN_5                 ((uint16_t)0x0020U)  /**< Pin 5 selected    */
#define AON_GPIO_PIN_6                 ((uint16_t)0x0040U)  /**< Pin 6 selected    */
#define AON_GPIO_PIN_7                 ((uint16_t)0x0080U)  /**< Pin 7 selected    */

#define AON_GPIO_PIN_ALL               ((uint16_t)0x00FFU)  /**< All pins selected */

#define AON_GPIO_PIN_MASK              (0x000000FFU)        /**< PIN mask for assert test */
/** @} */

/** @defgroup AON_GPIO_Mode AON_GPIO mode
  * @brief AON_GPIO Configuration Mode
  *        Elements values convention: 0x000000YX
  *           - X  : IO Direction mode (Input, Output, Mux)
  *           - Y  : IT trigger detection
  * @{
  */
#define AON_GPIO_MODE_INPUT         (LL_AON_GPIO_MODE_INPUT << 0)        /**< Input Mode                                                     */
#define AON_GPIO_MODE_OUTPUT        (LL_AON_GPIO_MODE_OUTPUT << 0)       /**< Output Mode                                                    */
#define AON_GPIO_MODE_MUX           (LL_GPIO_MODE_MUX << 0)              /**< Mux Mode                                                       */
#define AON_GPIO_MODE_IT_RISING     (LL_AON_GPIO_TRIGGER_RISING << 4)    /**< Interrupt Mode with Rising edge trigger detection              */
#define AON_GPIO_MODE_IT_FALLING    (LL_AON_GPIO_TRIGGER_FALLING << 4)   /**< Interrupt Mode with Falling edge trigger detection             */
#define AON_GPIO_MODE_IT_HIGH       (LL_AON_GPIO_TRIGGER_HIGH << 4)      /**< Interrupt Mode with High-level trigger detection               */
#define AON_GPIO_MODE_IT_LOW        (LL_AON_GPIO_TRIGGER_LOW << 4)       /**< Interrupt Mode with Low-level trigger detection                */
#define AON_GPIO_MODE_IT_BOTH_EDGE  (LL_AON_GPIO_TRIGGER_BOTH_EDGE << 4) /**< Interrupt Mode with Rising and Falling edge trigger detection   */
/** @} */


/** @defgroup AON_GPIO_Pull AON_GPIO pull
  * @brief AON_GPIO Pull-Up or Pull-Down activation
  * @{
  */
#define  AON_GPIO_NOPULL            LL_AON_GPIO_PULL_NO     /**< No Pull-up or Pull-down activation  */
#define  AON_GPIO_PULLUP            LL_AON_GPIO_PULL_UP     /**< Pull-up activation                  */
#define  AON_GPIO_PULLDOWN          LL_AON_GPIO_PULL_DOWN   /**< Pull-down activation                */
/** @} */

/**
  * @brief AON_GPIO_default_config initStruct default configuration
  */
#define AON_GPIO_DEFAULT_CONFIG                      \
{                                                    \
    .pin        = AON_GPIO_PIN_ALL,                  \
    .mode       = AON_GPIO_MODE_INPUT,               \
    .pull       = AON_GPIO_PULLDOWN,                 \
    .mux        = AON_GPIO_MUX_7,                    \
}
/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup AON_GPIO_Exported_Macros AON_GPIO Exported Macros
  * @{
  */

/**
  * @brief  Check whether the specified AON_GPIO pin is asserted or not.
  * @param  __AON_GPIO_PIN__ specifies the AON_GPIO pin to be checked.
  *          This parameter can be AON_GPIO_PIN_x where x can be (0..7).
  * @retval The new state of __AON_GPIO_PIN__ (SET or RESET).
  */
#define __HAL_AON_GPIO_IT_GET_IT(__AON_GPIO_PIN__)         ll_aon_gpio_read_flag_it(__AON_GPIO_PIN__)

/**
  * @brief  Clear the AON_GPIO pin pending bits.
  * @param  __AON_GPIO_PIN__ specifies the AON_GPIO pins to be cleared.
  *          This parameter can be any combination of AON_GPIO_PIN_x where x can be (0..7).
  * @retval None
  */
#define __HAL_AON_GPIO_IT_CLEAR_IT(__AON_GPIO_PIN__)       ll_aon_gpio_clear_flag_it(__AON_GPIO_PIN__)

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup AON_GPIO_Private_Macros AON_GPIO Private Macros
  * @{
  */

/**
  * @brief Check if AON GPIO pin action is valid.
  * @param __ACTION__ AON GPIO pin action.
  * @retval SET (__ACTION__ is valid) or RESET (__ACTION__ is invalid)
  */
#define IS_AON_GPIO_PIN_ACTION(__ACTION__)  (((__ACTION__) == AON_GPIO_PIN_RESET) || ((__ACTION__) == AON_GPIO_PIN_SET))

/**
  * @brief Check if AON GPIO pins are valid.
  * @param __PIN__ AON GPIO pins.
  * @retval SET (__PIN__ is valid) or RESET (__PIN__ is invalid)
  */
#define IS_AON_GPIO_PIN(__PIN__)    ((((__PIN__) & AON_GPIO_PIN_MASK) != 0x00U) &&\
                                     (((__PIN__) & ~AON_GPIO_PIN_MASK) == 0x00U))

/**
  * @brief Check if AON GPIO mode is valid.
  * @param __MODE__ AON GPIO mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_AON_GPIO_MODE(__MODE__)  (((__MODE__) == AON_GPIO_MODE_INPUT)              ||\
                                     ((__MODE__) == AON_GPIO_MODE_OUTPUT)             ||\
                                     ((__MODE__) == AON_GPIO_MODE_MUX)                ||\
                                     ((__MODE__) == AON_GPIO_MODE_IT_RISING)          ||\
                                     ((__MODE__) == AON_GPIO_MODE_IT_FALLING)         ||\
                                     ((__MODE__) == AON_GPIO_MODE_IT_HIGH)            ||\
                                     ((__MODE__) == AON_GPIO_MODE_IT_LOW))

/**
  * @brief Check if AON GPIO pull type is valid.
  * @param __PULL__ AON GPIO pull type.
  * @retval SET (__PULL__ is valid) or RESET (__PULL__ is invalid)
  */
#define IS_AON_GPIO_PULL(__PULL__)  (((__PULL__) == AON_GPIO_NOPULL)   ||\
                                     ((__PULL__) == AON_GPIO_PULLUP)   || \
                                     ((__PULL__) == AON_GPIO_PULLDOWN))

/** @} */

/** @} */

/* Include AON GPIO HAL Extended module */
#include "gr55xx_hal_aon_gpio_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_AON_GPIO_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup AON_GPIO_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the AON_GPIOx peripheral according to the specified parameters in the @ref aon_gpio_init_t.
 *
 * @param[in]  p_aon_gpio_init: Pointer to an @ref aon_gpio_init_t structure that contains
 *                         the configuration information for the specified AON_GPIO peripheral port.
 ****************************************************************************************
 */
void hal_aon_gpio_init(aon_gpio_init_t *p_aon_gpio_init);

/**
 ****************************************************************************************
 * @brief  De-initialize the AON_GPIOx peripheral registers to their default reset values.
 *
 * @param[in]  aon_gpio_pin: Specifies the port bit to be written.
 *         This parameter can be a combination of the following values:
 *         @arg @ref AON_GPIO_PIN_0
 *         @arg @ref AON_GPIO_PIN_1
 *         @arg @ref AON_GPIO_PIN_2
 *         @arg @ref AON_GPIO_PIN_3
 *         @arg @ref AON_GPIO_PIN_4
 *         @arg @ref AON_GPIO_PIN_5
 *         @arg @ref AON_GPIO_PIN_6
 *         @arg @ref AON_GPIO_PIN_7
 *         @arg @ref AON_GPIO_PIN_ALL
 ****************************************************************************************
 */
void hal_aon_gpio_deinit(uint32_t aon_gpio_pin);

/** @} */

/** @addtogroup AON_GPIO_Exported_Functions_Group2 IO operation functions
  *  @brief AON_GPIO Read, Write, Toggle, Lock and EXTI management functions.
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Read the specified input port pin.
 *
 * @param[in]  aon_gpio_pin: Specifies the port bit to be read.
 *         This parameter can be one of the following values:
 *         @arg @ref AON_GPIO_PIN_0
 *         @arg @ref AON_GPIO_PIN_1
 *         @arg @ref AON_GPIO_PIN_2
 *         @arg @ref AON_GPIO_PIN_3
 *         @arg @ref AON_GPIO_PIN_4
 *         @arg @ref AON_GPIO_PIN_5
 *         @arg @ref AON_GPIO_PIN_6
 *         @arg @ref AON_GPIO_PIN_7
 *
 * @return The input port pin value.
 ****************************************************************************************
 */
aon_gpio_pin_state_t hal_aon_gpio_read_pin(uint16_t aon_gpio_pin);

/**
 ****************************************************************************************
 * @brief  Set or clear the selected data port bit.
 *
 * @param[in]  aon_gpio_pin: Specifies the port bit to be written.
 *         This parameter can be a combination of the following values:
 *         @arg @ref AON_GPIO_PIN_0
 *         @arg @ref AON_GPIO_PIN_1
 *         @arg @ref AON_GPIO_PIN_2
 *         @arg @ref AON_GPIO_PIN_3
 *         @arg @ref AON_GPIO_PIN_4
 *         @arg @ref AON_GPIO_PIN_5
 *         @arg @ref AON_GPIO_PIN_6
 *         @arg @ref AON_GPIO_PIN_7
 *         @arg @ref AON_GPIO_PIN_ALL
 * @param[in]  pin_state: Specifies the value to be written to the selected bit.
 *         This parameter can be one of the AON_GPIO_PinState enum values:
 *         @arg AON_GPIO_PIN_RESET: to clear the port pin
 *         @arg AON_GPIO_PIN_SET: to set the port pin
 ****************************************************************************************
 */
void hal_aon_gpio_write_pin(uint16_t aon_gpio_pin, aon_gpio_pin_state_t pin_state);

/**
 ****************************************************************************************
 * @brief  Toggle the specified AON_GPIO pin.
 *
 * @param[in]  aon_gpio_pin: Specifies the pin to be toggled.
 *         This parameter can be a combination of the following values:
 *         @arg @ref AON_GPIO_PIN_0
 *         @arg @ref AON_GPIO_PIN_1
 *         @arg @ref AON_GPIO_PIN_2
 *         @arg @ref AON_GPIO_PIN_3
 *         @arg @ref AON_GPIO_PIN_4
 *         @arg @ref AON_GPIO_PIN_5
 *         @arg @ref AON_GPIO_PIN_6
 *         @arg @ref AON_GPIO_PIN_7
 *         @arg @ref AON_GPIO_PIN_ALL
 ****************************************************************************************
 */
void hal_aon_gpio_toggle_pin(uint16_t aon_gpio_pin);

/** @} */

/** @addtogroup AON_GPIO_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle AON_GPIO interrupt request.
 ****************************************************************************************
 */
void hal_aon_gpio_irq_handler(void);

/**
 ****************************************************************************************
 * @brief  AON GPIO pin detection callback.
 *
 * @note  This function should not be modified. When the callback is needed,
 *        the hal_aon_gpio_callback can be implemented in the user file.
 *
 * @param[in]  aon_gpio_pin: Indicate the port pin whose interrupt was triggered.
 *         This parameter can be a combination of the following values:
 *         @arg @ref AON_GPIO_PIN_0
 *         @arg @ref AON_GPIO_PIN_1
 *         @arg @ref AON_GPIO_PIN_2
 *         @arg @ref AON_GPIO_PIN_3
 *         @arg @ref AON_GPIO_PIN_4
 *         @arg @ref AON_GPIO_PIN_5
 *         @arg @ref AON_GPIO_PIN_6
 *         @arg @ref AON_GPIO_PIN_7
 *         @arg @ref AON_GPIO_PIN_ALL
 ****************************************************************************************
 */
void hal_aon_gpio_callback(uint16_t aon_gpio_pin);

/** @} */

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_AON_GPIO_H__ */

/** @} */

/** @} */

/** @} */

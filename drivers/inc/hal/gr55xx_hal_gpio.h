/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_gpio.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of GPIO HAL library.
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

/** @defgroup HAL_GPIO GPIO
  * @brief GPIO HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_GPIO_H__
#define __GR55xx_HAL_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_gpio.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_GPIO_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_GPIO_Callback Callback
  * @{
  */

/**
  * @brief HAL_GPIO Callback function definition
  */

typedef struct _gpio_callback
{
    void (*gpio_callback)(gpio_regs_t *GPIOx, uint16_t gpio_pin);   /**< GPIO pin detection callback   */
} gpio_callback_t;

/** @} */

/** @} */

/** @addtogroup HAL_GPIO_ENUMERATIONS Enumerations
  * @{
  */

/**
  * @brief  GPIO Bit SET and Bit RESET Enumerations
  */
typedef enum
{
    GPIO_PIN_RESET = 0U,    /**< GPIO pin low level.*/
    GPIO_PIN_SET            /**< GPIO pin high level.*/
} gpio_pin_state_t;

/** @} */


/** @addtogroup HAL_GPIO_STRUCTURES Structures
  * @{
  */

/**
  * @brief   GPIO init structure definition
  */
typedef struct _gpio_init
{
    uint32_t pin;       /**< Specifies the GPIO pins to be configured.
                             This parameter can be any value of @ref GPIO_pins */

    uint32_t mode;      /**< Specifies the operating mode for the selected pins.
                             This parameter can be a value of @ref GPIO_mode */

    uint32_t pull;      /**< Specifies the Pull-up or Pull-Down activation for the selected pins.
                             This parameter can be a value of @ref GPIO_pull */

    uint32_t mux;       /**< Specifies the Peripheral to be connected to the selected pins.
                             This parameter can be a value of @ref GPIOEx_Mux_Function_Selection. */
} gpio_init_t;

/** @} */

/**
  * @defgroup  HAL_GPIO_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup GPIO_Exported_Constants GPIO Exported Constants
  * @{
  */

/** @defgroup GPIO_pins GPIO pins
  * @{
  */
#define GPIO_PIN_0                 ((uint16_t)0x0001U)  /**< Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002U)  /**< Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004U)  /**< Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008U)  /**< Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010U)  /**< Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020U)  /**< Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040U)  /**< Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080U)  /**< Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100U)  /**< Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200U)  /**< Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400U)  /**< Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800U)  /**< Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000U)  /**< Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000U)  /**< Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000U)  /**< Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000U)  /**< Pin 15 selected   */
#define GPIO_PIN_ALL               ((uint16_t)0xFFFFU)  /**< All pins selected */

#define GPIO_PIN_MASK              (0x0000FFFFU)        /**< PIN mask for assert test */
/** @} */

/** @defgroup GPIO_mode GPIO mode
  * @brief GPIO Configuration Mode
  *        Elements values convention: 0x000000YX
  *           - X  : IO Direction mode (Input, Output, Mux)
  *           - Y  : IT trigger detection
  * @{
  */
#define GPIO_MODE_INPUT                (LL_GPIO_MODE_INPUT << 0)           /**< Input Mode                                         */
#define GPIO_MODE_OUTPUT               (LL_GPIO_MODE_OUTPUT << 0)          /**< Output Mode                                        */
#define GPIO_MODE_MUX                  (LL_GPIO_MODE_MUX << 0)             /**< Mux Mode                                           */
#define GPIO_MODE_IT_RISING            (LL_GPIO_TRIGGER_RISING << 4)       /**< Interrupt Mode with Rising edge trigger detection  */
#define GPIO_MODE_IT_FALLING           (LL_GPIO_TRIGGER_FALLING << 4)      /**< Interrupt Mode with Falling edge trigger detection */
#define GPIO_MODE_IT_HIGH              (LL_GPIO_TRIGGER_HIGH << 4)         /**< Interrupt Mode with High-level trigger detection   */
#define GPIO_MODE_IT_LOW               (LL_GPIO_TRIGGER_LOW << 4)          /**< Interrupt Mode with Low-level trigger detection    */
#define GPIO_MODE_IT_BOTH_EDGE         (LL_GPIO_TRIGGER_BOTH_EDGE << 4)    /**< Interrupt Mode with Rising and Falling edge trigger detection  */

/** @} */

/** @defgroup GPIO_pull GPIO pull
  * @brief GPIO Pull-Up or Pull-Down Activation
  * @{
  */
#define  GPIO_NOPULL        LL_GPIO_PULL_NO     /**< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        LL_GPIO_PULL_UP     /**< Pull-up activation                  */
#define  GPIO_PULLDOWN      LL_GPIO_PULL_DOWN   /**< Pull-down activation                */
/** @} */

/**
  * @brief GPIO_default_config InitStruct default configuration
  */
#define GPIO_DEFAULT_CONFIG                      \
{                                                \
    .pin        = GPIO_PIN_ALL,                  \
    .mode       = GPIO_MODE_INPUT,               \
    .pull       = GPIO_PULLDOWN,                 \
    .mux        = GPIO_PIN_MUX_GPIO,             \
}

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup GPIO_Exported_Macros GPIO Exported Macros
  * @{
  */

/**
  * @brief  Check whether the specified GPIO pin is asserted or not.
  * @param  __GPIOX__ Where X can be (0, 1, 2) to select the GPIO peripheral port
  * @param  __GPIO_PIN__ Specifies the GPIO pin to check.
  *          This parameter can be GPIO_PIN_x where x can be (0..15)
  * @retval The new state of __GPIO_PIN__ (SET or RESET).
  */
#define __HAL_GPIO_IT_GET_IT(__GPIOX__, __GPIO_PIN__)         ll_gpio_read_flag_it(__GPIOX__, __GPIO_PIN__)

/**
  * @brief  Clear the GPIO pin pending bits.
  * @param  __GPIOX__ Where X can be (0, 1, 2) to select the GPIO peripheral port
  * @param  __GPIO_PIN__ Specifies the GPIO pins to clear.
  *          This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
#define __HAL_GPIO_IT_CLEAR_IT(__GPIOX__, __GPIO_PIN__)       ll_gpio_clear_flag_it(__GPIOX__, __GPIO_PIN__)

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup GPIO_Private_Macros GPIO Private Macros
  * @{
  */

/**
  * @brief Check if GPIO pin action is valid.
  * @param __ACTION__ GPIO pin action.
  * @retval SET (__ACTION__ is valid) or RESET (__ACTION__ is invalid)
  */
#define IS_GPIO_PIN_ACTION(__ACTION__)  (((__ACTION__) == GPIO_PIN_RESET) || ((__ACTION__) == GPIO_PIN_SET))

/**
  * @brief Check if GPIO pins are valid.
  * @param __PIN__ GPIO pins.
  * @retval SET (__PIN__ is valid) or RESET (__PIN__ is invalid)
  */
#define IS_GPIO_PIN(__PIN__)        ((((__PIN__) & GPIO_PIN_MASK) != 0x00U) &&\
                                     (((__PIN__) & ~GPIO_PIN_MASK) == 0x00U))

/**
  * @brief Check if GPIO mode is valid.
  * @param __MODE__ GPIO mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */

#define IS_GPIO_MODE(__MODE__)      (((__MODE__) == GPIO_MODE_INPUT)              ||\
                                     ((__MODE__) == GPIO_MODE_OUTPUT)             ||\
                                     ((__MODE__) == GPIO_MODE_MUX)                ||\
                                     ((__MODE__) == GPIO_MODE_IT_RISING)          ||\
                                     ((__MODE__) == GPIO_MODE_IT_FALLING)         ||\
                                     ((__MODE__) == GPIO_MODE_IT_HIGH)            ||\
                                     ((__MODE__) == GPIO_MODE_IT_LOW)             ||\
                                     ((__MODE__) == GPIO_MODE_IT_BOTH_EDGE))

/**
  * @brief Check if GPIO pull type is valid.
  * @param __PULL__ GPIO pull type.
  * @retval SET (__PULL__ is valid) or RESET (__PULL__ is invalid)
  */
#define IS_GPIO_PULL(__PULL__)      (((__PULL__) == GPIO_NOPULL)   ||\
                                     ((__PULL__) == GPIO_PULLUP)   || \
                                     ((__PULL__) == GPIO_PULLDOWN))

/** @} */

/** @} */

/* Include GPIO HAL Extended module */
#include "gr55xx_hal_gpio_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_GPIO_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup GPIO_Exported_Functions_Group1 Initialization/de-initialization functions
 *  @brief    Initialization and de-initialization functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Initialize the GPIOx peripheral according to the specified parameters in the p_gpio_init.
 *
 * @param[in]  GPIOx: Where x can be (0, 1, 2) to select the GPIO peripheral port
 * @param[in]  p_gpio_init: Pointer to a gpio_init_t structure that contains the configuration information
 *                          for the specified GPIO peripheral port.
 ****************************************************************************************
 */
void hal_gpio_init(gpio_regs_t *GPIOx, gpio_init_t *p_gpio_init);

/**
 ****************************************************************************************
 * @brief  De-initialize the GPIOx peripheral registers to their default reset values.
 *
 * @param[in]  GPIOx: Where x can be (0, 1, 2) to select the GPIO peripheral port for GR55xx device
 * @param[in]  gpio_pin: Specifies the port bit to be written.
 *         This parameter can be a combination of the following values:
 *         @arg @ref GPIO_PIN_0
 *         @arg @ref GPIO_PIN_1
 *         @arg @ref GPIO_PIN_2
 *         @arg @ref GPIO_PIN_3
 *         @arg @ref GPIO_PIN_4
 *         @arg @ref GPIO_PIN_5
 *         @arg @ref GPIO_PIN_6
 *         @arg @ref GPIO_PIN_7
 *         @arg @ref GPIO_PIN_8
 *         @arg @ref GPIO_PIN_9
 *         @arg @ref GPIO_PIN_10
 *         @arg @ref GPIO_PIN_11
 *         @arg @ref GPIO_PIN_12
 *         @arg @ref GPIO_PIN_13
 *         @arg @ref GPIO_PIN_14
 *         @arg @ref GPIO_PIN_15
 *         @arg @ref GPIO_PIN_ALL
 ****************************************************************************************
 */
void hal_gpio_deinit(gpio_regs_t *GPIOx, uint32_t gpio_pin);

/** @} */

/** @addtogroup GPIO_Exported_Functions_Group2 IO operation functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Read the specified input port pin.
 *
 * @param[in]  GPIOx: Where x can be (0, 1, 2) to select the GPIO peripheral port
 * @param[in]  gpio_pin: Specifies the port bit to be read.
 *         This parameter can be a one of the following values:
 *         @arg @ref GPIO_PIN_0
 *         @arg @ref GPIO_PIN_1
 *         @arg @ref GPIO_PIN_2
 *         @arg @ref GPIO_PIN_3
 *         @arg @ref GPIO_PIN_4
 *         @arg @ref GPIO_PIN_5
 *         @arg @ref GPIO_PIN_6
 *         @arg @ref GPIO_PIN_7
 *         @arg @ref GPIO_PIN_8
 *         @arg @ref GPIO_PIN_9
 *         @arg @ref GPIO_PIN_10
 *         @arg @ref GPIO_PIN_11
 *         @arg @ref GPIO_PIN_12
 *         @arg @ref GPIO_PIN_13
 *         @arg @ref GPIO_PIN_14
 *         @arg @ref GPIO_PIN_15
 *
 * @retval ::GPIO_PIN_RESET: GPIO pin low level.
 * @retval ::GPIO_PIN_SET: GPIO pin high level.
 ****************************************************************************************
 */
gpio_pin_state_t hal_gpio_read_pin(gpio_regs_t *GPIOx, uint16_t gpio_pin);

/**
 ****************************************************************************************
 * @brief  Set or clear the selected data port bit.
 *
 * @param[in]  GPIOx: Where x can be (0, 1, 2) to select the GPIO peripheral port
 * @param[in]  gpio_pin: Specifies the port bit to be written.
 *         This parameter can be a combination of the following values:
 *         @arg @ref GPIO_PIN_0
 *         @arg @ref GPIO_PIN_1
 *         @arg @ref GPIO_PIN_2
 *         @arg @ref GPIO_PIN_3
 *         @arg @ref GPIO_PIN_4
 *         @arg @ref GPIO_PIN_5
 *         @arg @ref GPIO_PIN_6
 *         @arg @ref GPIO_PIN_7
 *         @arg @ref GPIO_PIN_8
 *         @arg @ref GPIO_PIN_9
 *         @arg @ref GPIO_PIN_10
 *         @arg @ref GPIO_PIN_11
 *         @arg @ref GPIO_PIN_12
 *         @arg @ref GPIO_PIN_13
 *         @arg @ref GPIO_PIN_14
 *         @arg @ref GPIO_PIN_15
 *         @arg @ref GPIO_PIN_ALL
 * @param[in]  pin_state: Specifies the value to be written to the selected bit.
 *         This parameter can be one of the GPIO_PinState enum values:
 *            @arg @ref GPIO_PIN_RESET clear the port pin
 *            @arg @ref GPIO_PIN_SET   set the port pin
 ****************************************************************************************
 */
void hal_gpio_write_pin(gpio_regs_t *GPIOx, uint16_t gpio_pin, gpio_pin_state_t pin_state);

/**
 ****************************************************************************************
 * @brief  Toggle the specified GPIO pin.
 *
 * @param[in]  GPIOx: Where x can be (0, 1, 2) to select the GPIO peripheral port
 * @param[in]  gpio_pin: Specifies the pin to be toggled.
 *         This parameter can be a combination of the following values:
 *         @arg @ref GPIO_PIN_0
 *         @arg @ref GPIO_PIN_1
 *         @arg @ref GPIO_PIN_2
 *         @arg @ref GPIO_PIN_3
 *         @arg @ref GPIO_PIN_4
 *         @arg @ref GPIO_PIN_5
 *         @arg @ref GPIO_PIN_6
 *         @arg @ref GPIO_PIN_7
 *         @arg @ref GPIO_PIN_8
 *         @arg @ref GPIO_PIN_9
 *         @arg @ref GPIO_PIN_10
 *         @arg @ref GPIO_PIN_11
 *         @arg @ref GPIO_PIN_12
 *         @arg @ref GPIO_PIN_13
 *         @arg @ref GPIO_PIN_14
 *         @arg @ref GPIO_PIN_15
 *         @arg @ref GPIO_PIN_ALL
 ****************************************************************************************
 */
void hal_gpio_toggle_pin(gpio_regs_t *GPIOx, uint16_t gpio_pin);

/** @} */

/** @addtogroup GPIO_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle GPIO interrupt request.
 *
 * @param[in]  GPIOx: Where x can be (0, 1, 2) to select the GPIO peripheral port
 ****************************************************************************************
 */
void hal_gpio_exti_irq_handler(gpio_regs_t *GPIOx);

/**
 ****************************************************************************************
 * @brief  GPIO pin detection callback.
 *
 * @note  This function should not be modified. When the callback is needed,
 *          the hal_gpio_exti_callback can be implemented in the user file.
 *
 * @param[in]  GPIOx: Where x can be (0, 1, 2) to select the GPIO peripheral port
 * @param[in]  gpio_pin: Indicate the port pin whose interrupt was triggered.
 *         This parameter can be a combination of the following values:
 *         @arg @ref GPIO_PIN_0
 *         @arg @ref GPIO_PIN_1
 *         @arg @ref GPIO_PIN_2
 *         @arg @ref GPIO_PIN_3
 *         @arg @ref GPIO_PIN_4
 *         @arg @ref GPIO_PIN_5
 *         @arg @ref GPIO_PIN_6
 *         @arg @ref GPIO_PIN_7
 *         @arg @ref GPIO_PIN_8
 *         @arg @ref GPIO_PIN_9
 *         @arg @ref GPIO_PIN_10
 *         @arg @ref GPIO_PIN_11
 *         @arg @ref GPIO_PIN_12
 *         @arg @ref GPIO_PIN_13
 *         @arg @ref GPIO_PIN_14
 *         @arg @ref GPIO_PIN_15
 *         @arg @ref GPIO_PIN_ALL
 ****************************************************************************************
 */
void hal_gpio_exti_callback(gpio_regs_t *GPIOx, uint16_t gpio_pin);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_GPIO_H__ */

/** @} */

/** @} */

/** @} */

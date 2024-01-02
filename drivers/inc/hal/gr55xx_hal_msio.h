/**
 ****************************************************************************************
 *
 * @file   gr55xx_hal_msio.h
 * @author BLE Driver Team
 * @brief  Header file containing functions prototypes of MSIO HAL library.
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

/** @defgroup HAL_MSIO MSIO
  * @brief MSIO HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_MSIO_H__
#define __GR55xx_HAL_MSIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_msio.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/

/** @addtogroup HAL_MSIO_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/**
  * @brief HAL_MSIO Callback function definition
  */
typedef struct _msio_callback
{
    void (*msio_callback)(msio_pad_t MSIOx, uint16_t msio_pin);   /**< MSIO pin detection callback   */
} msio_callback_t;

/** @} */

/** @addtogroup HAL_MSIO_ENUMERATIONS Enumerations
  * @{
  */

/**
  * @brief  MSIO Bit SET and Bit RESET enumerations
  */
typedef enum
{
    MSIO_PIN_RESET = 0U,          /**< MSIO pin low level. */
    MSIO_PIN_SET                  /**< MSIO pin high level.*/
} msio_pin_state_t;

/** @} */

/** @addtogroup HAL_MSIO_STRUCTURES Structures
  * @{
  */

/**
  * @brief   MSIO init structure definition
  */
typedef struct _msio_init
{
    uint32_t pin;       /**< Specifies the MSIO pins to be configured.
                            This parameter can be any value of @ref MSIO_pins */

    uint32_t direction; /**< Specifies the direction for the selected pins.
                             This parameter can be a value of @ref MSIO_direction */

    uint32_t mode;      /**< Specifies the operating mode for the selected pins.
                             This parameter can be a value of @ref MSIO_mode */

    uint32_t pull;      /**< Specifies the Pull-up or Pull-Down activation for the selected pins.
                             This parameter can be a value of @ref MSIO_pull */

    uint32_t mux;       /**< Specifies the Peripheral to be connected to the selected pins.
                             This parameter can be a value of @ref MSIOEx_Mux_Function_Selection. */
} msio_init_t;

/** @} */

/**
  * @defgroup  HAL_MSIO_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup MSIO_Exported_Constants MSIO Exported Constants
  * @{
  */

/** @defgroup MSIO_pins MSIO pins
  * @{
  */
#define MSIO_PIN_0                  ((uint16_t)0x0001U)  /**< Pin 0 selected    */
#define MSIO_PIN_1                  ((uint16_t)0x0002U)  /**< Pin 1 selected    */
#define MSIO_PIN_2                  ((uint16_t)0x0004U)  /**< Pin 2 selected    */
#define MSIO_PIN_3                  ((uint16_t)0x0008U)  /**< Pin 3 selected    */
#define MSIO_PIN_4                  ((uint16_t)0x0010U)  /**< Pin 4 selected    */
#define MSIO_PIN_5                  ((uint16_t)0x0020U)  /**< Pin 5 selected    */
#define MSIO_PIN_6                  ((uint16_t)0x0040U)  /**< Pin 6 selected    */
#define MSIO_PIN_7                  ((uint16_t)0x0080U)  /**< Pin 7 selected    */
#define MSIO_PIN_ALL                ((uint16_t)0x00FFU)  /**< All pins selected */
#define MSIO_PIN_MASK               (0x000000FFU)        /**< PIN mask for assert test */

/** @} */

/** @defgroup MSIO_direction MSIO direction
  * @{
  */
#define MSIO_DIRECTION_NONE         LL_MSIO_DIRECTION_NONE      /**< Disable input & output */
#define MSIO_DIRECTION_INPUT        LL_MSIO_DIRECTION_INPUT     /**< Only Input             */
#define MSIO_DIRECTION_OUTPUT       LL_MSIO_DIRECTION_OUTPUT    /**< Only Output            */
#define MSIO_DIRECTION_INOUT        LL_MSIO_DIRECTION_INOUT     /**< Input & Output         */
/** @} */

/** @defgroup MSIO_mode MSIO mode
  * @brief MSIO Analog or Digital mode
  *        Elements values convention: 0x000000YX
  *           - X  : IO Direction mode (Input, Output, Mux)
  *           - Y  : IT trigger detection
  * @{
  */
#define MSIO_MODE_ANALOG            LL_MSIO_MODE_ANALOG                /**< Analog  IO */
#define MSIO_MODE_DIGITAL           LL_MSIO_MODE_DIGITAL               /**< Digital IO */
#define MSIO_MODE_IT_RISING        (LL_MSIO_TRIGGER_IT_RISING << 4)    /**< Interrupt Mode with Rising edge trigger detection  */
#define MSIO_MODE_IT_FALLING       (LL_MSIO_TRIGGER_IT_FALLING << 4)   /**< Interrupt Mode with Falling edge trigger detection */
#define MSIO_MODE_IT_HIGH          (LL_MSIO_TRIGGER_IT_HIGH << 4)      /**< Interrupt Mode with High-level trigger detection   */
#define MSIO_MODE_IT_LOW           (LL_MSIO_TRIGGER_IT_LOW << 4)       /**< Interrupt Mode with Low-level trigger detection    */
#define MSIO_MODE_IT_BOTH_EDGE     (LL_MSIO_TRIGGER_IT_BOTH_EDGE << 4) /**< Interrupt Mode with Rising and Falling edge trigger detection  */
/** @} */

/** @defgroup MSIO_pull MSIO pull
  * @brief MSIO Pull-Up or Pull-Down Activation
  * @{
  */
#define  MSIO_NOPULL                LL_MSIO_PULL_NO             /**< No Pull-up or Pull-down activation  */
#define  MSIO_PULLUP                LL_MSIO_PULL_UP             /**< Pull-up activation                  */
#define  MSIO_PULLDOWN              LL_MSIO_PULL_DOWN           /**< Pull-down activation                */
/** @} */

/**
  * @brief MSIO_default_config initStruct default configuartion
  */
#define MSIO_DEFAULT_CONFIG                      \
{                                                \
    .pin        = MSIO_PIN_ALL,                  \
    .direction  = MSIO_DIRECTION_INPUT,          \
    .mode       = MSIO_MODE_DIGITAL,             \
    .pull       = MSIO_PULLDOWN,                 \
    .mux        = MSIO_MUX_7,                    \
}

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup MSIO_Exported_Macros MSIO Exported Macros
  * @{
  */

/**
  * @brief  Check whether the specified MSIO pin is asserted or not.
  * @param  __MSIOX__ MSIOX: Must select the MSIOA port
  * @param  __MSIO_PIN__ Specifies the MSIO pin to check.
  *          This parameter can be MSIO_PIN_x where x can be (0..7)
  * @retval The new state of __MSIO_PIN__ (SET or RESET).
  */
#define __HAL_MSIO_IT_GET_IT(__MSIOX__, __MSIO_PIN__)         ll_msio_read_flag_it(__MSIOX__, __MSIO_PIN__)

/**
  * @brief  Clear the MSIO pin pending bits.
  * @param  __MSIOX__ MSIOX: Must select the MSIOA port
  * @param  __MSIO_PIN__ Specifies the MSIO pins to clear.
  *          This parameter can be any combination of MSIO_PIN_x where x can be (0..7)
  * @retval None
  */
#define __HAL_MSIO_IT_CLEAR_IT(__MSIOX__, __MSIO_PIN__)       ll_msio_clear_flag_it(__MSIOX__, __MSIO_PIN__)


/** @} */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup MSIO_Private_Macros MSIO Private Macros
  * @{
  */

/**
  * @brief Check if MSIO pin action is valid.
  * @param __ACTION__ MSIO pin action.
  * @retval SET (__ACTION__ is valid) or RESET (__ACTION__ is invalid)
  */
#define IS_MSIO_PIN_ACTION(__ACTION__)  (((__ACTION__) == MSIO_PIN_RESET) || ((__ACTION__) == MSIO_PIN_SET))

/**
  * @brief Check if MSIO pins are valid.
  * @param __PIN__ MSIO pins.
  * @retval SET (__PIN__ is valid) or RESET (__PIN__ is invalid)
  */
#define IS_MSIO_PIN(__PIN__)            ((((__PIN__) &  MSIO_PIN_MASK) != 0x00U) && \
                                         (((__PIN__) & ~MSIO_PIN_MASK) == 0x00U))

/**
  * @brief Check if MSIO direction is valid.
  * @param __DIR__ MSIO direction.
  * @retval SET (__DIR__ is valid) or RESET (__DIR__ is invalid)
  */
#define IS_MSIO_DIRECTION(__DIR__)      (((__DIR__) == MSIO_DIRECTION_NONE)   || \
                                         ((__DIR__) == MSIO_DIRECTION_INPUT)  || \
                                         ((__DIR__) == MSIO_DIRECTION_OUTPUT) || \
                                         ((__DIR__) == MSIO_DIRECTION_INOUT))

/**
  * @brief Check if MSIO mode is valid.
  * @param __MODE__ MSIO mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_MSIO_MODE(__MODE__)          (((__MODE__) == MSIO_MODE_ANALOG) || \
                                         ((__MODE__) == MSIO_MODE_DIGITAL))

/**
  * @brief Check if MSIO pull type is valid.
  * @param __PULL__ MSIO pull type.
  * @retval SET (__PULL__ is valid) or RESET (__PULL__ is invalid)
  */
#define IS_MSIO_PULL(__PULL__)          (((__PULL__) == MSIO_NOPULL)   || \
                                         ((__PULL__) == MSIO_PULLUP)   || \
                                         ((__PULL__) == MSIO_PULLDOWN))

/** @} */

/** @} */

/* Include MSIO HAL Extended module */
#include "gr55xx_hal_msio_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_MSIO_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup MSIO_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the MSIOx peripheral according to the specified parameters in the @ref msio_init_t.
 * @param[in]  MSIOx:       MSIO peripheral port.
 * @param[in]  p_msio_init: Pointer to an @ref msio_init_t structure that contains
 *                          the configuration information for the specified MSIO peripheral port.
 ****************************************************************************************
 */
void hal_msio_init(msio_pad_t MSIOx, msio_init_t *p_msio_init);

/**
 ****************************************************************************************
 * @brief  De-initialize the MSIOx peripheral registers to their default reset values.
 * @param[in]  MSIOx:    MSIO peripheral port.
 * @param[in]  msio_pin: Specifies the port bit to be written.
 *             This parameter can be a combination of the following values:
 *             @arg @ref MSIO_PIN_0
 *             @arg @ref MSIO_PIN_1
 *             @arg @ref MSIO_PIN_2
 *             @arg @ref MSIO_PIN_3
 *             @arg @ref MSIO_PIN_4
 *             @arg @ref MSIO_PIN_5
 *             @arg @ref MSIO_PIN_6
 *             @arg @ref MSIO_PIN_7
 *             @arg @ref MSIO_PIN_ALL
 ****************************************************************************************
 */
void hal_msio_deinit(msio_pad_t MSIOx, uint32_t msio_pin);

/** @} */

/** @addtogroup MSIO_Exported_Functions_Group2 IO operation functions
  *  @brief MSIO Read, Write, and Toggle management functions.
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Read the specified input port pin.
 * @param[in]  MSIOx:    MSIO peripheral port.
 * @param[in]  msio_pin: Specifies the port bit to be read.
 *         This parameter can be one of the following values:
 *         @arg @ref MSIO_PIN_0
 *         @arg @ref MSIO_PIN_1
 *         @arg @ref MSIO_PIN_2
 *         @arg @ref MSIO_PIN_3
 *         @arg @ref MSIO_PIN_4
 *         @arg @ref MSIO_PIN_5
 *         @arg @ref MSIO_PIN_6
 *         @arg @ref MSIO_PIN_7
 * @retval ::MSIO_PIN_RESET: MSIO pin low level.
 * @retval ::MSIO_PIN_SET:   MSIO pin high level.
 ****************************************************************************************
 */
msio_pin_state_t hal_msio_read_pin(msio_pad_t MSIOx, uint16_t msio_pin);

/**
 ****************************************************************************************
 * @brief  Set or clear the selected data port bit.
 * @param[in]  MSIOx:     MSIO peripheral port.
 * @param[in]  msio_pin:  Specifies the port bit to be written.
 *             This parameter can be a combination of the following values:
 *             @arg @ref MSIO_PIN_0
 *             @arg @ref MSIO_PIN_1
 *             @arg @ref MSIO_PIN_2
 *             @arg @ref MSIO_PIN_3
 *             @arg @ref MSIO_PIN_4
 *             @arg @ref MSIO_PIN_5
 *             @arg @ref MSIO_PIN_6
 *             @arg @ref MSIO_PIN_7
 *             @arg @ref MSIO_PIN_ALL
 * @param[in]  pin_state: Specifies the value to be written to the selected bit.
 *             This parameter can be one of the MSIO_PinState enum values:
 *             @arg MSIO_PIN_RESET: to clear the port pin
 *             @arg MSIO_PIN_SET: to set the port pin
 ****************************************************************************************
 */
void hal_msio_write_pin(msio_pad_t MSIOx, uint16_t msio_pin, msio_pin_state_t pin_state);

/**
 ****************************************************************************************
 * @brief  Toggle the specified MSIO pin.
 * @param[in]  MSIOx:    MSIO peripheral port.
 * @param[in]  msio_pin: Specifies the pin to be toggled.
 *         This parameter can be a combination of the following values:
 *         @arg @ref MSIO_PIN_0
 *         @arg @ref MSIO_PIN_1
 *         @arg @ref MSIO_PIN_2
 *         @arg @ref MSIO_PIN_3
 *         @arg @ref MSIO_PIN_4
 *         @arg @ref MSIO_PIN_5
 *         @arg @ref MSIO_PIN_6
 *         @arg @ref MSIO_PIN_7
 *         @arg @ref MSIO_PIN_ALL
 ****************************************************************************************
 */
void hal_msio_toggle_pin(msio_pad_t MSIOx, uint16_t msio_pin);
/** @} */

 /** @addtogroup MSIO_Exported_Functions_Group3 MSIO_A_IRQHandler and Callbacks
  *  @brief    IRQ Handler and Callbacks functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Handle MSIOA interrupt request.
 *
 * @param[in]  MSIOx: Must select the MSIOA port.
 ****************************************************************************************
 */
void hal_msio_exti_irq_handler(msio_pad_t MSIOx);

/**
 ****************************************************************************************
 * @brief  MSIOA pin detection callback.
 *
 * @note  This function should not be modified. When the callback is needed,
 *          the hal_msio_exti_callback can be implemented in the user file.
 *
 * @param[in]  MSIOx:    Must select the MSIOA port.
 * @param[in]  msio_pin: Indicate the port pin whose interrupt was triggered.
 *         This parameter can be a combination of the following values:
 *         @arg @ref MSIO_PIN_0
 *         @arg @ref MSIO_PIN_1
 *         @arg @ref MSIO_PIN_2
 *         @arg @ref MSIO_PIN_3
 *         @arg @ref MSIO_PIN_4
 *         @arg @ref MSIO_PIN_5
 *         @arg @ref MSIO_PIN_6
 *         @arg @ref MSIO_PIN_7
 *         @arg @ref MSIO_PIN_ALL
 ****************************************************************************************
 */
void hal_msio_exti_callback(msio_pad_t MSIOx, uint16_t msio_pin);


/** @} */

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_MSIO_H__ */

/** @} */

/** @} */

/** @} */

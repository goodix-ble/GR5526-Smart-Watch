/**
 ****************************************************************************************
 *
 * @file    app_io.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of GPIO app library.
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

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_GPIO GPIO
  * @brief GPIO APP module driver.
  * @{
  */


#ifndef _APP_IO_H_
#define _APP_IO_H_

#include "app_drv_error.h"
#include "app_drv_config.h"
#include <stdint.h>

/** @addtogroup APP_GPIO_PIN_DEFINES Defines
  * @{
  */

/** @addtogroup GR5xxx_pins Defines
  * @{
  */
/**
  * @brief APP_GPIO_DEFINE IO pins
  */
#define APP_IO_PIN_0                 ((uint32_t)0x00000001U)  /**< Pin 0 selected    */
#define APP_IO_PIN_1                 ((uint32_t)0x00000002U)  /**< Pin 1 selected    */
#define APP_IO_PIN_2                 ((uint32_t)0x00000004U)  /**< Pin 2 selected    */
#define APP_IO_PIN_3                 ((uint32_t)0x00000008U)  /**< Pin 3 selected    */
#define APP_IO_PIN_4                 ((uint32_t)0x00000010U)  /**< Pin 4 selected    */
#define APP_IO_PIN_5                 ((uint32_t)0x00000020U)  /**< Pin 5 selected    */
#define APP_IO_PIN_6                 ((uint32_t)0x00000040U)  /**< Pin 6 selected    */
#define APP_IO_PIN_7                 ((uint32_t)0x00000080U)  /**< Pin 7 selected    */
#define APP_IO_PIN_8                 ((uint32_t)0x00000100U)  /**< Pin 8 selected    */
#define APP_IO_PIN_9                 ((uint32_t)0x00000200U)  /**< Pin 9 selected    */
#define APP_IO_PIN_10                ((uint32_t)0x00000400U)  /**< Pin 10 selected   */
#define APP_IO_PIN_11                ((uint32_t)0x00000800U)  /**< Pin 11 selected   */
#define APP_IO_PIN_12                ((uint32_t)0x00001000U)  /**< Pin 12 selected   */
#define APP_IO_PIN_13                ((uint32_t)0x00002000U)  /**< Pin 13 selected   */
#define APP_IO_PIN_14                ((uint32_t)0x00004000U)  /**< Pin 14 selected   */
#define APP_IO_PIN_15                ((uint32_t)0x00008000U)  /**< Pin 15 selected   */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
#define APP_IO_PIN_16                ((uint32_t)0x00010000U)  /**< Pin 16 selected   */
#define APP_IO_PIN_17                ((uint32_t)0x00020000U)  /**< Pin 17 selected   */
#define APP_IO_PIN_18                ((uint32_t)0x00040000U)  /**< Pin 18 selected   */
#define APP_IO_PIN_19                ((uint32_t)0x00080000U)  /**< Pin 19 selected   */
#define APP_IO_PIN_20                ((uint32_t)0x00100000U)  /**< Pin 20 selected   */
#define APP_IO_PIN_21                ((uint32_t)0x00200000U)  /**< Pin 21 selected   */
#define APP_IO_PIN_22                ((uint32_t)0x00400000U)  /**< Pin 22 selected   */
#define APP_IO_PIN_23                ((uint32_t)0x00800000U)  /**< Pin 23 selected   */
#define APP_IO_PIN_24                ((uint32_t)0x01000000U)  /**< Pin 24 selected   */
#define APP_IO_PIN_25                ((uint32_t)0x02000000U)  /**< Pin 25 selected   */
#define APP_IO_PIN_26                ((uint32_t)0x04000000U)  /**< Pin 26 selected   */
#define APP_IO_PIN_27                ((uint32_t)0x08000000U)  /**< Pin 27 selected   */
#define APP_IO_PIN_28                ((uint32_t)0x10000000U)  /**< Pin 28 selected   */
#define APP_IO_PIN_29                ((uint32_t)0x20000000U)  /**< Pin 29 selected   */
#define APP_IO_PIN_30                ((uint32_t)0x40000000U)  /**< Pin 30 selected   */
#define APP_IO_PIN_31                ((uint32_t)0x80000000U)  /**< Pin 31 selected   */
#endif

#define APP_IO_PINS_0_7              ((uint32_t)0x000000FFU)  /**< 0~7 pins selected  */
#define APP_IO_PINS_0_15             ((uint32_t)0x0000FFFFU)  /**< 0~15 pins selected  */
#define APP_IO_PINS_16_31            ((uint32_t)0xFFFF0000U)  /**< 16~31 pins selected */
#define APP_IO_PIN_ALL               ((uint32_t)0x0000FFFFU)  /**< All pins selected   */
#define APP_AON_IO_PIN_ALL           ((uint32_t)0x000000FFU)  /**< All AON pins selected */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
#define APP_MSIO_IO_PIN_ALL          ((uint32_t)0x000003FFU)  /**< All MISO pins selected */
#else
#define APP_MSIO_IO_PIN_ALL          ((uint32_t)0x000000FFU)  /**< All MISO pins selected */
#endif
#define APP_IO_PIN_MASK              ((uint32_t)0xFFFFFFFFU)  /**< PIN mask for assert test */

/**
  * @brief GR5xxx_APP_GPIO_default_config initStruct default config of APP_GPIOn
  */
#define APP_IO_DEFAULT_CONFIG                      \
{                                                  \
    .pin        = APP_IO_PIN_ALL,                  \
    .mode       = APP_IO_MODE_INPUT,               \
    .pull       = APP_IO_PULLDOWN,                 \
}

/** @} */

/** @} */

/** @addtogroup APP_GPIO_ENUMERATIONS Enumerations
  * @{
  */
/**
  * @brief   GPIO state Enumerations definition
  */
typedef enum
{
    APP_IO_PIN_RESET,        /**< IO pin low level. */
    APP_IO_PIN_SET,          /**< IO pin high level. */
} app_io_pin_state_t;

/**
  * @brief   GPIO type Enumerations definition
  */
typedef enum
{
    APP_IO_TYPE_GPIOA,       /**< General Purpose Input/Output. */
    APP_IO_TYPE_GPIOB,       /**< General Purpose Input/Output. */
    APP_IO_TYPE_GPIOC,       /**< General Purpose Input/Output. */
    APP_IO_TYPE_AON,         /**< Always-on Input/Output. */
    APP_IO_TYPE_MSIO,        /**< Mixed Signal I/O. */
    // #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    APP_IO_TYPE_NORMAL,      /**< General Purpose Input/Output. */
    // #endif
    APP_IO_TYPE_MAX,         /**< Only for check parameter, not used as input parameters. */
} app_io_type_t;

/**
  * @brief   GPIO mode Enumerations definition
  */
typedef enum
{
    APP_IO_MODE_NONE,
    APP_IO_MODE_INPUT,          /**< Input Mode. */
    APP_IO_MODE_OUTPUT,         /**< Output Mode. */
    APP_IO_MODE_MUX,            /**< Mux Mode. */
    APP_IO_MODE_IT_RISING,      /**< Interrupt Mode with Rising edge trigger detection. */
    APP_IO_MODE_IT_FALLING,     /**< Interrupt Mode with Falling edge trigger detection. */
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
    APP_IO_MODE_IT_BOTH_EDGE,   /**< Interrupt Mode with Rising and Falling edge trigger detection */
#endif
    APP_IO_MODE_IT_HIGH,        /**< Interrupt Mode with High-level trigger detection. */
    APP_IO_MODE_IT_LOW,         /**< Interrupt Mode with Low-level trigger detection. */
    APP_IO_MODE_ANALOG,         /**< Analog IO Mode. */
    APP_IO_MODE_MAX,            /**< Only for check parameter, not used as input parameters. */
} app_io_mode_t;

/**
  * @brief   GPIO pull Enumerations definition
  */
typedef enum
{
    APP_IO_NOPULL,              /**< No Pull-up or Pull-down activation. */
    APP_IO_PULLUP,              /**< Pull-up activation. */
    APP_IO_PULLDOWN,            /**< Pull-down activation. */
    APP_IO_PULL_MAX             /**< Only for check parameter, not used as input parameters. */
} app_io_pull_t;

/**
  * @brief   GPIO mux Enumerations definition
  */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
typedef enum
{
    APP_IO_MUX_0,               /**< IO_MUX_GPIO. */
    APP_IO_MUX_1,               /**< IO_MUX_I2C0_SCL. */
    APP_IO_MUX_2,               /**< IO_MUX_I2C0_SDA. */
    APP_IO_MUX_3,               /**< IO_MUX_I2C1_SCL. */
    APP_IO_MUX_4,               /**< IO_MUX_I2C1_SDA. */
    APP_IO_MUX_5,               /**< IO_MUX_UART0_CTS. */
    APP_IO_MUX_6,               /**< IO_MUX_UART0_RTS. */
    APP_IO_MUX_7,               /**< IO_MUX_UART0_TX. */
    APP_IO_MUX_8,               /**< IO_MUX_UART0_RX. */
    APP_IO_MUX_9,               /**< IO_MUX_UART1_CTS. */
    APP_IO_MUX_10,              /**< IO_MUX_UART1_RTS. */
    APP_IO_MUX_11,              /**< IO_MUX_UART1_TX. */
    APP_IO_MUX_12,              /**< IO_MUX_UART1_RX. */
    APP_IO_MUX_13,              /**< IO_MUX_PWM0. */
    APP_IO_MUX_14,              /**< IO_MUX_PWM1. */
    APP_IO_MUX_15,              /**< IO_MUX_PWM2. */
    APP_IO_MUX_16,              /**< IO_MUX_PWM3. */
    APP_IO_MUX_17,              /**< IO_MUX_PWM4. */
    APP_IO_MUX_18,              /**< IO_MUX_PWM5. */
    APP_IO_MUX_19,              /**< IO_MUX_df_ant_sw_0. */
    APP_IO_MUX_20,              /**< IO_MUX_df_ant_sw_1. */
    APP_IO_MUX_21,              /**< IO_MUX_df_ant_sw_2. */
    APP_IO_MUX_22,              /**< IO_MUX_df_ant_sw_3. */
    APP_IO_MUX_23,              /**< IO_MUX_df_ant_sw_4. */
    APP_IO_MUX_24,              /**< IO_MUX_df_ant_sw_5. */
    APP_IO_MUX_25,              /**< IO_MUX_df_ant_sw_6. */
    APP_IO_MUX_26,              /**< IO_MUX_ferp_gpio_trig_0. */
    APP_IO_MUX_27,              /**< IO_MUX_SWO. */
    APP_IO_MUX_28,              /**< IO_MUX_coex_ble_rx. */
    APP_IO_MUX_29,              /**< IO_MUX_coex_ble_tx. */
    APP_IO_MUX_30,              /**< IO_MUX_coex_wlan_rx. */
    APP_IO_MUX_31,              /**< IO_MUX_coex_wlan_tx. */
    APP_IO_MUX_32,              /**< IO_MUX_coex_ble_in_process. */
    APP_IO_MUX_33,              /**< IO_MUX_SWD_CLK. */
    APP_IO_MUX_34,              /**< IO_MUX_SWD_DATA. */
    APP_IO_MUX_35,              /**< IO_MUX_reserve3. */
    APP_IO_MUX_36,              /**< IO_MUX_reserve4. */
    APP_IO_MUX_37,              /**< IO_MUX_reserve5. */
    APP_IO_MUX_38,              /**< IO_MUX_SPI_S_MOSI. */
    APP_IO_MUX_39,              /**< IO_MUX_SPI_S_CS_N. */
    APP_IO_MUX_40,              /**< IO_MUX_SPI_S_CLK. */
    APP_IO_MUX_41,              /**< IO_MUX_SPI_S_MISO. */
    APP_IO_MUX_42,              /**< IO_MUX_SPI_M_CLK. */
    APP_IO_MUX_43,              /**< IO_MUX_SPI_M_CS0_N. */
    APP_IO_MUX_44,              /**< IO_MUX_SPI_M_CS1_N. */
    APP_IO_MUX_45,              /**< IO_MUX_SPI_M_MISO. */
    APP_IO_MUX_46,              /**< IO_MUX_SPI_M_MOSIss. */
    APP_IO_MUX_47,              /**< RESERVED. */
    APP_IO_MUX_48,              /**< RESERVED. */
    APP_IO_MUX_49,              /**< IO_MUX_DUAL_TIMER0_A. */
    APP_IO_MUX_50,              /**< IO_MUX_DUAL_TIMER0_B. */
    APP_IO_MUX_51,              /**< IO_MUX_DUAL_TIMER0_C. */
    APP_IO_MUX_52,              /**< IO_MUX_DUAL_TIMER1_A. */
    APP_IO_MUX_53,              /**< IO_MUX_DUAL_TIMER1_B. */
    APP_IO_MUX_54,              /**< IO_MUX_DUAL_TIMER1_C. */
    APP_IO_MUX_MAX,             /**< Only for check parameter, not used as input parameters. */
} app_io_mux_t;
#else
typedef enum
{
    APP_IO_MUX_0,               /**< IO mux mode 0. */
    APP_IO_MUX_1,               /**< IO mux mode 1. */
    APP_IO_MUX_2,               /**< IO mux mode 2. */
    APP_IO_MUX_3,               /**< IO mux mode 3. */
    APP_IO_MUX_4,               /**< IO mux mode 4. */
    APP_IO_MUX_5,               /**< IO mux mode 5. */
    APP_IO_MUX_6,               /**< IO mux mode 6. */
    APP_IO_MUX_7,               /**< IO mux mode 7. */
    APP_IO_MUX_8,               /**< IO mux mode 8. */
    APP_IO_MUX_MAX,             /**< Only for check parameter, not used as input parameters. */
} app_io_mux_t;
#endif
/** @} */

/** @addtogroup APP_GPIO_PIN_DEFINES Defines
  * @{
  */
/**
* @brief GPIO mux for different APP_DRIVER_CHIP_TYPE
*/
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
#define APP_IO_MUX APP_IO_MUX_0                                    /**< IO mux for GR5332X. */
#elif (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
#define APP_IO_MUX APP_IO_MUX_8                                   /**< IO mux for GR5525X. */
#else
#define APP_IO_MUX APP_IO_MUX_7                                   /**< IO mux for others. */
#endif

/** @} */

/** @addtogroup APP_GPIO_STRUCT Structures
  * @{
  */
/**
  * @brief GPIO parameter structure definition
  */
typedef struct
{
    uint32_t  pin;              /**< Specifies the IO pins to be configured.
                                     This parameter can be any value of @ref GR5xxx_pins */
    app_io_mode_t mode;         /**< Specifies the operating mode for the selected pins. */
    app_io_pull_t pull;         /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
    app_io_mux_t  mux;          /**< Specifies the Peripheral to be connected to the selected pins. */
} app_io_init_t;

/**
  * @brief GPIO Interrupt event Structure definition
  */
typedef struct
{
    app_io_type_t           type;       /**< Type of event. */
    uint32_t                pin;        /**< Specifies the IO pins to be configured. */
    void                   *arg;        /**< User parameters */
} app_io_evt_t;

/** @} */

/** @addtogroup APP_GPIO_ENUMERATIONS Enumerations
  * @{
  */
/**
  * @brief  GPIO Speed Structure definition
  */
typedef enum {
    APP_IO_SPPED_MEDIUM,       /**< Select medium speed. */
    APP_IO_SPPED_HIGH,         /**< Select high speed. */
    APP_IO_SPPED_MAX           /**< Only for check parameter, not used as input parameters. */
} app_io_speed_t;


/**
  * @brief  GPIO Input type Structure definition
  */
typedef enum {
    APP_IO_INPUT_TYPE_CMOS,    /**< Select CMOS input. */
    APP_IO_INPUT_TYPE_SCHMITT, /**< Select Schmitt input. */
    APP_IO_INPUT_TYPE_MAX      /**< Only for check parameter, not used as input parameters. */
} app_io_input_type_t;

/**
  * @brief  GPIO Strength Structure definition
  */
typedef enum {
    APP_IO_STRENGTH_LOW,       /**< Select low output driver strength. */
    APP_IO_STRENGTH_MEDIUM,    /**< Select medium output driver strength. */
    APP_IO_STRENGTH_HIGH,      /**< Select high output driver strength. */
    APP_IO_STRENGTH_ULTRA,     /**< Select high output driver strength. */
    APP_IO_STRENGTH_MAX,       /**< Only for check parameter, not used as input parameters. */
} app_io_strength_t;

/** @} */

/** @addtogroup APP_GPIO_TYPEDEFS Type definitions
  * @{
  */
/**
  * @brief GPIO callback type.
  */
typedef void (*app_io_callback_t)(app_io_evt_t *p_evt);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_GPIO_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP GPIO DRIVER according to the specified parameters
 *         in the app_io_type_t and app_io_init_t.
 *
 * @param[in]  type:   GPIO type.
 * @param[in]  p_init: Pointer to app_io_init_t parameter which contains the
 *                     configuration information for the specified GPIO.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_io_init(app_io_type_t type, app_io_init_t *p_init);

/**
 ****************************************************************************************
 * @brief  De-initialize the GPIOx peripheral.
 *
 * @param[in]  type: GPIO type, See app_io_type_t.
 * @param[in]  pin:  The pin want to De-initialization.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_io_deinit(app_io_type_t type, uint32_t pin);

/**
 ****************************************************************************************
 * @brief  Read the specified input port pin..
 *
 * @param[in]  type: GPIO type, See app_io_type_t.
 * @param[in]  pin:  The pin want to read.
 *
 * @return The GPIO state.
 ****************************************************************************************
 */
app_io_pin_state_t app_io_read_pin(app_io_type_t type, uint32_t pin);

/**
 ****************************************************************************************
 * @brief  Set or clear the selected data port bit.
 *
 * @param[in]  type:      GPIO type, See app_io_type_t.
 * @param[in]  pin:       The pin want to set or clear.
 * @param[in]  pin_state: Specifies the value to be written to the selected bit.
 *
 * @return Result of write.
 ****************************************************************************************
 */
uint16_t app_io_write_pin(app_io_type_t type, uint32_t pin, app_io_pin_state_t pin_state);

/**
 ****************************************************************************************
 * @brief  Toggle the specified GPIO pin.
 *
 * @param[in]  type: GPIO type, See app_io_type_t.
 * @param[in]  pin:  The pin want to toggle.
 *
 * @return Result of toggle.
 ****************************************************************************************
 */
uint16_t app_io_toggle_pin(app_io_type_t type, uint32_t pin);

/**
 ****************************************************************************************
 * @brief  Set the speed of the GPIO.
 *
 * @param[in]  type:  GPIO type, See app_io_type_t.
 * @param[in]  pin:   The pin want to set.
 * @param[in]  speed: GPIO speed type, See app_io_speed_t.
 *
 * @return Result of setting.
 ****************************************************************************************
 */
uint16_t app_io_set_speed(app_io_type_t type, uint32_t pin, app_io_speed_t speed);

/**
 ****************************************************************************************
 * @brief  Set the strength of the GPIO.
 *
 * @param[in]  type:     GPIO type, See app_io_type_t.
 * @param[in]  pin:      The pin want to set.
 * @param[in]  strength: GPIO strength type, See app_io_strength_t.
 *
 * @return Result of setting.
 ****************************************************************************************
 */
uint16_t app_io_set_strength(app_io_type_t type, uint32_t pin, app_io_strength_t strength);

/**
 ****************************************************************************************
 * @brief  Set the input type of the GPIO.
 *
 * @param[in]  type:       GPIO type, See app_io_type_t.
 * @param[in]  pin:        The pin want to toggle.
 * @param[in]  input_type: GPIO input type, See app_io_input_type_t.
 *
 * @return Result of setting.
 ****************************************************************************************
 */
uint16_t app_io_set_intput_type(app_io_type_t type, uint32_t pin, app_io_input_type_t input_type);

/**
 ****************************************************************************************
 * @brief Initialize GPIO to interrupt mode and register interrupt callback function.
 *
 * @param[in]  type:      GPIO type, See app_io_type_t.
 * @param[in]  p_init:    Pointer to app_io_init_t parameter which contains the
 *                        configuration information for the specified GPIO.
 * @param[in]  io_evt_cb: Interrupt callback function.
 * @param[in]  arg:       User parameters.
 *
 * @return Result of register.
 ****************************************************************************************
 */
uint16_t app_io_event_register_cb(app_io_type_t type, app_io_init_t *p_init, app_io_callback_t io_evt_cb, void *arg);

/**
 ****************************************************************************************
 * @brief  Deinitialize GPIO to normal mode and unregister interrupt.
 *
 * @param[in]  type: GPIO type, See app_io_type_t.
 * @param[in]  pin:  The pin want to unregister.
 *
 * @return Result of unregister.
 ****************************************************************************************
 */
uint16_t app_io_event_unregister(app_io_type_t type, uint32_t pin);

/** @} */
#endif

/** @} */

/** @} */

/** @} */


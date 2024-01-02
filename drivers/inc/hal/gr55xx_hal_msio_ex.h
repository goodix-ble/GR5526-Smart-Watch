/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_msio_ex.h
 * @author  BLE Driver Team
 * @brief   Header file containing extended macro of MSIO HAL library.
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

/** @defgroup HAL_MSIOEx MSIOEx
  * @brief MSIOEx HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_MSIO_EX_H__
#define __GR55xx_HAL_MSIO_EX_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"
#include "gr55xx_ll_msio.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @defgroup  HAL_MSIOEX_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup MSIOEx_Exported_Constants MSIOEx Exported Constants
  * @{
  */

/** @defgroup MSIOEx_Mux_Mode MSIOEx Mux Mode definition
  * @{
  */
#define MSIO_MUX_0                      LL_MSIO_MUX_0   /**< MSIO mux mode 0 */
#define MSIO_MUX_1                      LL_MSIO_MUX_1   /**< MSIO mux mode 1 */
#define MSIO_MUX_2                      LL_MSIO_MUX_2   /**< MSIO mux mode 2 */
#define MSIO_MUX_3                      LL_MSIO_MUX_3   /**< MSIO mux mode 3 */
#define MSIO_MUX_4                      LL_MSIO_MUX_4   /**< MSIO mux mode 4 */
#define MSIO_MUX_5                      LL_MSIO_MUX_5   /**< MSIO mux mode 5 */
#define MSIO_MUX_6                      LL_MSIO_MUX_6   /**< MSIO mux mode 6 */
#define MSIO_MUX_7                      LL_MSIO_MUX_7   /**< MSIO mux mode 7 */
/** @} */

/** @defgroup MSIOEx_Mux_Function_Selection MSIOEx Mux function selection
  * @{
  */

#if defined (GR552xx)
/*---------------------------------- GR552xx ------------------------------*/

/** @defgroup MSIOEx_Common_Selection MSIO PIN common MUX selection(Available for all MSIO pins)
  * @{
  */

#define MSIO_PIN_MUX_GPIO               MSIO_MUX_7  /**< MSIO PIN x Mux Select GPIO */

/** @} */

/** @defgroup MSIOEx_PIN0_Mux_Selection MSIOA_PIN0 MUX selection
  * @{
  */
#define MSIOA_PIN0_MUX_PWM0_A         MSIO_MUX_0  /**< MSIOA_PIN0 Mux Select PWM0_A */
#define MSIOA_PIN0_MUX_SIM_CLK        MSIO_MUX_1  /**< MSIOA_PIN0 Mux Select SIM_CLK */
#define MSIOA_PIN0_MUX_UART3_RX       MSIO_MUX_2  /**< MSIOA_PIN0 Mux Select UART3_RX */
#define MSIOA_PIN0_MUX_I2S_SCLK       MSIO_MUX_3  /**< MSIOA_PIN0 Mux Select I2S_SCLK */
#define MSIOA_PIN0_MUX_I2S_S_SCLK     MSIO_MUX_4  /**< MSIOA_PIN0 Mux Select I2S_S_SCLK */
#define MSIOA_PIN0_MUX_PDM_DI         MSIO_MUX_5  /**< MSIOA_PIN0 Mux Select PDM_DI */
/** @} */

/** @defgroup MSIOEx_PIN1_Mux_Selection MSIOA_PIN1 MUX selection
  * @{
  */
#define MSIOA_PIN1_MUX_PWM0_B         MSIO_MUX_0  /**< MSIOA_PIN1 Mux Select PWM0_B */
#define MSIOA_PIN1_MUX_SIM_IO         MSIO_MUX_1  /**< MSIOA_PIN1 Mux Select SIM_IO */
#define MSIOA_PIN1_MUX_UART3_TX       MSIO_MUX_2  /**< MSIOA_PIN1 Mux Select UART3_TX */
#define MSIOA_PIN1_MUX_I2S_RX_SDI     MSIO_MUX_3  /**< MSIOA_PIN1 Mux Select I2S_RX_SDI */
#define MSIOA_PIN1_MUX_I2S_S_RX_SDI   MSIO_MUX_4  /**< MSIOA_PIN1 Mux Select I2S_S_RX_SDI */
#define MSIOA_PIN1_MUX_PDM_CLKO       MSIO_MUX_5  /**< MSIOA_PIN1 Mux Select PDM_CLKO */
/** @} */

/** @defgroup MSIOEx_PIN2_Mux_Selection MSIOA_PIN2 MUX selection
  * @{
  */
#define MSIOA_PIN2_MUX_PWM0_C         MSIO_MUX_0  /**< MSIOA_PIN2 Mux Select PWM0_C */
#define MSIOA_PIN2_MUX_SIM_RST_N      MSIO_MUX_1  /**< MSIOA_PIN2 Mux Select SIM_RST_N */
#define MSIOA_PIN2_MUX_UART3_RTS      MSIO_MUX_2  /**< MSIOA_PIN2 Mux Select UART3_RTS */
#define MSIOA_PIN2_MUX_I2S_TX_SDO     MSIO_MUX_3  /**< MSIOA_PIN2 Mux Select I2S_TX_SDO */
#define MSIOA_PIN2_MUX_I2S_S_TX_SDO   MSIO_MUX_4  /**< MSIOA_PIN2 Mux Select I2S_S_TX_SDO */
#define MSIOA_PIN2_MUX_COEX_WLAN_RX   MSIO_MUX_5  /**< MSIOA_PIN2 Mux Select COEX_WLAN_RX */
/** @} */

/** @defgroup MSIOEx_PIN3_Mux_Selection MSIOA_PIN3 MUX selection
  * @{
  */
#define MSIOA_PIN3_MUX_PWM1_A         MSIO_MUX_0  /**< MSIOA_PIN3 Mux Select PWM1_A */
#define MSIOA_PIN3_MUX_SIM_PRESENCE   MSIO_MUX_1  /**< MSIOA_PIN3 Mux Select SIM_PRESENCE */
#define MSIOA_PIN3_MUX_UART3_CTS      MSIO_MUX_2  /**< MSIOA_PIN3 Mux Select UART3_CTS */
#define MSIOA_PIN3_MUX_I2S_WS         MSIO_MUX_3  /**< MSIOA_PIN3 Mux Select I2S_WS */
#define MSIOA_PIN3_MUX_I2S_S_WS       MSIO_MUX_4  /**< MSIOA_PIN3 Mux Select I2S_S_WS */
#define MSIOA_PIN3_MUX_COEX_WLAN_TX   MSIO_MUX_5  /**< MSIOA_PIN3 Mux Select COEX_WLAN_TX */
/** @} */

/** @defgroup MSIOEx_PIN4_Mux_Selection MSIOA_PIN4 MUX selection
  * @{
  */
#define MSIOA_PIN4_MUX_UART2_RTS      MSIO_MUX_0  /**< MSIOA_PIN4 Mux Select UART2_RTS */
#define MSIOA_PIN4_MUX_PWM1_B         MSIO_MUX_1  /**< MSIOA_PIN4 Mux Select PWM1_B */
#define MSIOA_PIN4_MUX_I2C3_SDA       MSIO_MUX_2  /**< MSIOA_PIN4 Mux Select I2C3_SDA */
#define MSIOA_PIN4_MUX_I2C0_SDA       MSIO_MUX_4  /**< MSIOA_PIN4 Mux Select I2C0_SDA */
#define MSIOA_PIN4_MUX_COEX_BLE_RX    MSIO_MUX_5  /**< MSIOA_PIN4 Mux Select COEX_BLE_RX */
#define MSIOA_PIN4_MUX_ISO_SYNC0_P    MSIO_MUX_6  /**< MSIOA_PIN4 Mux Select ISO_SYNC0_P */
/** @} */

/** @defgroup MSIOEx_PIN5_Mux_Selection MSIOA_PIN5 MUX selection
  * @{
  */
#define MSIOA_PIN5_MUX_UART2_CTS      MSIO_MUX_0  /**< MSIOA_PIN5 Mux Select UART2_CTS */
#define MSIOA_PIN5_MUX_PWM1_C         MSIO_MUX_1  /**< MSIOA_PIN5 Mux Select PWM1_C */
#define MSIOA_PIN5_MUX_I2C3_SCL       MSIO_MUX_2  /**< MSIOA_PIN5 Mux Select I2C3_SCL */
#define MSIOA_PIN5_MUX_I2C0_SCL       MSIO_MUX_4  /**< MSIOA_PIN5 Mux Select I2C0_SCL */
#define MSIOA_PIN5_MUX_COEX_BLE_TX    MSIO_MUX_5  /**< MSIOA_PIN5 Mux Select COEX_BLE_TX */
#define MSIOA_PIN5_MUX_ISO_SYNC1_P    MSIO_MUX_6  /**< MSIOA_PIN5 Mux Select ISO_SYNC1_P */
/** @} */

/** @defgroup MSIOEx_PIN6_Mux_Selection MSIOA_PIN6 MUX selection
  * @{
  */
#define MSIOA_PIN6_MUX_UART2_RX       MSIO_MUX_0  /**< MSIOA_PIN6 Mux Select UART2_RX */
#define MSIOA_PIN6_MUX_PWM1_B         MSIO_MUX_1  /**< MSIOA_PIN6 Mux Select PWM1_B */
#define MSIOA_PIN6_MUX_I2C4_SDA       MSIO_MUX_2  /**< MSIOA_PIN6 Mux Select I2C4_SDA */
#define MSIOA_PIN6_MUX_I2C1_SDA       MSIO_MUX_4  /**< MSIOA_PIN6 Mux Select I2C1_SDA */
#define MSIOA_PIN6_MUX_ISO_SYNC0_P    MSIO_MUX_5  /**< MSIOA_PIN6 Mux Select ISO_SYNC0_P */
/** @} */

/** @defgroup MSIOEx_PIN7_Mux_Selection MSIOA_PIN7 MUX selection
  * @{
  */
#define MSIOA_PIN7_MUX_UART2_TX       MSIO_MUX_0  /**< MSIOA_PIN7 Mux Select UART2_TX */
#define MSIOA_PIN7_MUX_PWM1_C         MSIO_MUX_1  /**< MSIOA_PIN7 Mux Select PWM1_C */
#define MSIOA_PIN7_MUX_I2C4_SCL       MSIO_MUX_2  /**< MSIOA_PIN7 Mux Select I2C4_SCL */
#define MSIOA_PIN7_MUX_I2C1_SCL       MSIO_MUX_4  /**< MSIOA_PIN7 Mux Select I2C1_SCL */
#define MSIOA_PIN7_MUX_ISO_SYNC1_P    MSIO_MUX_5  /**< MSIOA_PIN7 Mux Select ISO_SYNC1_P */
/** @} */

/**
  * @brief Check if MSIO mux mode is valid.
  * @param __MUX__ MSIO mux mode.
  * @retval SET (__ACTION__ is valid) or RESET (__ACTION__ is invalid)
  */
#define IS_MSIO_MUX(__MUX__)        (((__MUX__) <= MSIO_MUX_7))

/*------------------------------------------------------------------------------------------*/
#endif /* GR552xx */

/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_MSIO_EX_H__ */

/** @} */

/** @} */

/** @} */


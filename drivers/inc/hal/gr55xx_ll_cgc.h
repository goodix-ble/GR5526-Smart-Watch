/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_cgc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of CGC LL library.
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

/** @defgroup LL_CGC CGC
  * @brief CGC LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_CGC_H__
#define __GR55XX_LL_CGC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined(MCU_SUB) || defined(MCU_RET)
/**
  * @defgroup  LL_CGC_MACRO Defines
  * @{
  */
/* Exported constants --------------------------------------------------------*/
/** @defgroup LL_CGC_Exported_Constants CGC Exported Constants
  * @{
  */
/** @defgroup LL_CGC_EC_WFI_CLK0 Block0 Clock During WFI 
  * @{
  */
#define LL_CGC_WFI_SECU_HCLK                                        MCU_SUB_WFI_SECU_HCLK                           /**< Hclk for all security blocks               */
#define LL_CGC_WFI_SIM_HCLK                                         MCU_SUB_WFI_SIM_HCLK                            /**< Hclk for sim card interface                */
#define LL_CGC_WFI_HTB_HCLK                                         MCU_SUB_WFI_HTB_HCLK                            /**< Hclk for hopping table                     */
#define LL_CGC_WFI_PWM_HCLK                                         MCU_SUB_WFI_PWM_HCLK                            /**< Hclk for PWM                               */
#define LL_CGC_WFI_ROM_HCLK                                         MCU_SUB_WFI_ROM_HCLK                            /**< Hclk for ROM                               */
#define LL_CGC_WFI_SNSADC_HCLK                                      MCU_SUB_WFI_SNSADC_HCLK                         /**< Hclk for sense ADC                         */
#define LL_CGC_WFI_GPIO_HCLK                                        MCU_SUB_WFI_GPIO_HCLK                           /**< Hclk for GPIOs                             */
#define LL_CGC_WFI_BLE_BRG_HCLK                                     MCU_SUB_WFI_BLE_BRG_HCLK                        /**< Hclk for BLE MCU bridge                    */
#define LL_CGC_WFI_APB_SUB_HCLK                                     MCU_SUB_WFI_APB_SUB_HCLK                        /**< Hclk for APB subsystem                     */
#define LL_CGC_WFI_SERIAL_HCLK                                      MCU_SUB_WFI_SERIAL_HCLK                         /**< Hclk for serial blocks                     */
#define LL_CGC_WFI_ALL_HCLK0                                        ((uint32_t)0x000007FFU)                         /**< All clock group 0                          */

/** @} */

/** @defgroup LL_CGC_EC_WFI_CLK1 Block1 Clock During WFI
  * @{
  */
#define LL_CGC_WFI_AON_MCUSUB_HCLK                                  MCU_SUB_WFI_AON_MCUSUB_HCLK                     /**< Hclk for Always-on register                */
#define LL_CGC_WFI_XF_XQSPI_HCLK                                    MCU_SUB_WFI_XF_XQSPI_HCLK                       /**< Hclk for cache top                         */
#define LL_CGC_WFI_SRAM_HCLK                                        MCU_SUB_WFI_SRAM_HCLK                           /**< Hclk for SRAMs                             */

#define LL_CGC_WFI_ALL_HCLK1                                        ((uint32_t)0x00000007U)                         /**< All clock group 1                          */
/** @} */

/** @defgroup LL_CGC_EC_WFI_CLK2 Block2 Clock During WFI
  * @{
  */
#define LL_CGC_WFI_SECU_DIV4_PCLK                                   MCU_SUB_WFI_SECU_DIV4_PCLK                      /**< Div4 clk for security blocks               */
#define LL_CGC_WFI_XQSPI_DIV4_PCLK                                  MCU_SUB_WFI_XQSPI_DIV4_PCLK                     /**< Div4 clk for xf qspi                       */

#define LL_CGC_WFI_ALL_HCLK2                                        ((uint32_t)0x05000000U)                         /**< All clock group 2                          */
/** @} */


/** @defgroup LL_CGC_EC_FRC_CLK0 Block0 Clock During FRC 
  * @{
  */
#define LL_CGC_FRC_SECU_HCLK                                        MCU_SUB_FORCE_SECU_HCLK                         /**< Hclk for all security blocks               */
#define LL_CGC_FRC_SIM_HCLK                                         MCU_SUB_FORCE_SIM_HCLK                          /**< Hclk for sim card interface                */
#define LL_CGC_FRC_HTB_HCLK                                         MCU_SUB_FORCE_HTB_HCLK                          /**< Hclk for hopping table                     */
#define LL_CGC_FRC_ROM_HCLK                                         MCU_SUB_FORCE_ROM_HCLK                          /**< Hclk for ROM                               */
#define LL_CGC_FRC_SNSADC_HCLK                                      MCU_SUB_FORCE_SNSADC_HCLK                       /**< Hclk for sense ADC                         */
#define LL_CGC_FRC_GPIO_HCLK                                        MCU_SUB_FORCE_GPIO_HCLK                         /**< Hclk for GPIOs                             */
#define LL_CGC_FRC_BLE_BRG_HCLK                                     MCU_SUB_FORCE_BLE_BRG_HCLK                      /**< Hclk for BLE MCU bridge                    */
#define LL_CGC_FRC_APB_SUB_HCLK                                     MCU_SUB_FORCE_APB_SUB_HCLK                      /**< Hclk for APB subsystem                     */
#define LL_CGC_FRC_SERIAL_HCLK                                      MCU_SUB_FORCE_SERIAL_HCLK                       /**< Hclk for serial blocks                     */
#define LL_CGC_FRC_ALL_HCLK0                                        ((uint32_t)0x00000777U)                         /**< All clock group 0                          */
/** @} */

/** @defgroup LL_CGC_EC_FRC_CLK1 Block1 Clock During FRC 
  * @{
  */
#define LL_CGC_FRC_AON_MCUSUB_HCLK                                  MCU_SUB_FORCE_AON_MCUSUB_HCLK                   /**< Hclk for Always-on register                */
#define LL_CGC_FRC_XF_XQSPI_HCLK                                    MCU_SUB_FORCE_XF_XQSPI_HCLK                     /**< Hclk for cache top                         */
#define LL_CGC_FRC_SRAM_HCLK                                        MCU_SUB_FORCE_SRAM_HCLK                         /**< Hclk for SRAMs                             */

#define LL_CGC_FRC_ALL_HCLK1                                        ((uint32_t)0x00070000U)                         /**< All clock group 1                          */
/** @} */

/** @defgroup LL_CGC_EC_FRC_CLK2 Block2 Clock During FRC 
  * @{
  */
#define LL_CGC_FRC_UART0_PCLK                                       MCU_SUB_FORCE_UART0_PCLK                        /**< Pclk for uart0                             */
#define LL_CGC_FRC_UART1_PCLK                                       MCU_SUB_FORCE_UART1_PCLK                        /**< Pclk for uart1                             */
#define LL_CGC_FRC_UART2_PCLK                                       MCU_SUB_FORCE_UART2_PCLK                        /**< Pclk for uart2                             */
#define LL_CGC_FRC_UART3_PCLK                                       MCU_SUB_FORCE_UART3_PCLK                        /**< Pclk for uart3                             */
#define LL_CGC_FRC_UART4_PCLK                                       MCU_SUB_FORCE_UART4_PCLK                        /**< Pclk for uart4                             */
#define LL_CGC_FRC_UART5_PCLK                                       MCU_SUB_FORCE_UART5_PCLK                        /**< Pclk for uart5                             */
#define LL_CGC_FRC_I2C0_PCLK                                        MCU_SUB_FORCE_I2C0_PCLK                         /**< Hclk for i2c0                              */
#define LL_CGC_FRC_I2C1_PCLK                                        MCU_SUB_FORCE_I2C1_PCLK                         /**< Hclk for i2c1                              */
#define LL_CGC_FRC_I2C2_PCLK                                        MCU_SUB_FORCE_I2C2_PCLK                         /**< Hclk for i2c2                              */
#define LL_CGC_FRC_I2C3_PCLK                                        MCU_SUB_FORCE_I2C3_PCLK                         /**< Hclk for i2c3                              */
#define LL_CGC_FRC_I2C4_PCLK                                        MCU_SUB_FORCE_I2C4_PCLK                         /**< Hclk for i2c4                              */
#define LL_CGC_FRC_I2C5_PCLK                                        MCU_SUB_FORCE_I2C5_PCLK                         /**< Hclk for i2c5                              */
#define LL_CGC_FRC_QSPI0_PCLK                                       MCU_SUB_FORCE_QSPI0_PCLK                        /**< Hclk for qspi0                             */
#define LL_CGC_FRC_QSPI1_PCLK                                       MCU_SUB_FORCE_QSPI1_PCLK                        /**< Hclk for qspi1                             */
#define LL_CGC_FRC_QSPI2_PCLK                                       MCU_SUB_FORCE_QSPI2_PCLK                        /**< Hclk for qspi2                             */
#define LL_CGC_FRC_SPI_M_PCLK                                       MCU_SUB_FORCE_SPI_M_PCLK                        /**< Hclk for spim                              */
#define LL_CGC_FRC_SPI_S_PCLK                                       MCU_SUB_FORCE_SPI_S_PCLK                        /**< Hclk for spis                              */
#define LL_CGC_FRC_I2S_HCLK                                         MCU_SUB_FORCE_I2S_PCLK                          /**< Hclk for i2s                               */
#define LL_CGC_FRC_I2S_S_PCLK                                       MCU_SUB_FORCE_I2S_S_PCLK                        /**< Hclk for i2ss                              */
#define LL_CGC_FRC_DSPI_PCLK                                        MCU_SUB_FORCE_DSPI_PCLK                         /**< Hclk for dspi                              */
#define LL_CGC_FRC_PDM_PCLK                                         MCU_SUB_FORCE_PDM_PCLK                          /**< Hclk for pdm                               */
#define LL_CGC_FRC_PWM_0_PCLK                                       MCU_SUB_FORCE_PWM_0_PCLK                        /**< Pclk for PWM0                              */
#define LL_CGC_FRC_PWM_1_PCLK                                       MCU_SUB_FORCE_PWM_1_PCLK                        /**< Pclk for PWM1                              */
#define LL_CGC_FRC_VTTBL_PCLK                                       MCU_SUB_FORCE_VTTBL_PCLK                        /**< Pclk for VTTBL                             */
#define LL_CGC_FRC_SECU_DIV4_PCLK                                   MCU_SUB_FORCE_SECU_DIV4_PCLK                    /**< Div4 clk for security blocksi              */
#define LL_CGC_FRC_XQSPI_DIV4_PCLK                                  MCU_SUB_FORCE_XQSPI_DIV4_PCLK                   /**< Div4 clk for xf qspi                       */

#define LL_CGC_FRC_SERIALS_HCLK2                                    ((uint32_t)0x705E0FFFUL)                        /**< Hclk for serial blocks                     */
#define LL_CGC_FRC_ALL_HCLK2                                        ((uint32_t)0xFF7FCFFFUL)                        /**< All clock group 2                          */
/** @} */

/** @defgroup LL_CGC_PERIPH_CG_LP_EN  Low Power Feature
  * @{
  */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_AHB2APB_EN                       MCU_SUB_PERIPH_CG_LP_AHB2APB_ASYNC_EN           /**< Enable AHB2APB ASYNC low-power feature     */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_AHB2APB_SYNC_EN                  MCU_SUB_PERIPH_CG_LP_AHB2APB_SYNC_EN            /**< Enable AHB2APB SYNC low-power feature      */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_QSPIM_EN                         MCU_SUB_PERIPH_CG_LP_EN_QSPIM_EN                /**< Enable qspim low-power feature             */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_AHB_BUS_LP_EN                    MCU_SUB_PERIPH_CG_LP_EN_AHB_BUS_LP_EN           /**< Enable AHB bus low-power feature           */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_I2C_LP_SCLK_EN                   MCU_SUB_PERIPH_CG_LP_EN_I2C_LP_SCLK_EN          /**< Enable i2c sclk low-power feature          */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_SPIS_LP_SCLK_EN                  MCU_SUB_PERIPH_CG_LP_EN_SPIS_LP_SCLK_EN         /**< Enable spis sclk low-power feature         */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_SPIM_LP_SCLK_EN                  MCU_SUB_PERIPH_CG_LP_EN_SPIM_LP_SCLK_EN         /**< Enable spim sclk low-power feature         */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_I2S_LP_EN                        MCU_SUB_PERIPH_CG_LP_EN_I2S_LP_EN               /**< Enable i2s master low-power feature        */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_UART_LP_PCLK_EN                  MCU_SUB_PERIPH_CG_LP_EN_UART_LP_PCLK_EN         /**< Enable uart pclk low-power feature         */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_UART_LP_SCLK_EN                  MCU_SUB_PERIPH_CG_LP_EN_UART_LP_SCLK_EN         /**< Enable uart sclk low-power feature         */

#define LL_CGC_MCU_PERIPH_CG_LP                                     ((uint32_t)0x00000F3FUL)                        /**< All Low Power Feature                      */
/** @} */

/** @defgroup LL_CGC_SUBSYS_PERI_CLK_SLP_OFF Peripherals Off During WFI/WFE
  * @brief    Turn the peripherals off during WFI/WFE
  * @{
  */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_UART_0_SLP_OFF                   MCU_SUB_PERIPH_CLK_SLP_OFF_UART0                /**< Turn the uart0 off during WFI/WFE          */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_UART_1_SLP_OFF                   MCU_SUB_PERIPH_CLK_SLP_OFF_UART1                /**< Turn the uart1 off during WFI/WFE          */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_UART_2_SLP_OFF                   MCU_SUB_PERIPH_CLK_SLP_OFF_UART2                /**< Turn the uart2 off during WFI/WFE          */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_UART_3_SLP_OFF                   MCU_SUB_PERIPH_CLK_SLP_OFF_UART3                /**< Turn the uart3 off during WFI/WFE          */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_UART_4_SLP_OFF                   MCU_SUB_PERIPH_CLK_SLP_OFF_UART4                /**< Turn the uart4 off during WFI/WFE          */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_UART_5_SLP_OFF                   MCU_SUB_PERIPH_CLK_SLP_OFF_UART5                /**< Turn the uart5 off during WFI/WFE          */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_I2S_M_SLP_OFF                    MCU_SUB_PERIPH_CLK_SLP_OFF_I2SM                 /**< Turn the i2s_m off during WFI/WFE          */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_I2S_S_SLP_OFF                    MCU_SUB_PERIPH_CLK_SLP_OFF_I2SS                 /**< Turn the i2s_s off during WFI/WFE          */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_SPI_M_SLP_OFF                    MCU_SUB_PERIPH_CLK_SLP_OFF_SPIM                 /**< Turn the spi_m off during WFI/WFE          */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_SPI_S_SLP_OFF                    MCU_SUB_PERIPH_CLK_SLP_OFF_SPIS                 /**< Turn the spi_s off during WFI/WFE          */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_PWM_0_SLP_OFF                    MCU_SUB_PERIPH_CLK_SLP_OFF_PWM0                 /**< Turn the pwm0 off during WFI/WFE           */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_PWM_1_SLP_OFF                    MCU_SUB_PERIPH_CLK_SLP_OFF_PWM1                 /**< Turn the pwm1 off during WFI/WFE           */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_QSPIM_0_SLP_OFF                  MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM0               /**< Turn the qspim0 off during WFI/WFE         */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_QSPIM_1_SLP_OFF                  MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM1               /**< Turn the qspim1 off during WFI/WFE         */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_QSPIM_2_SLP_OFF                  MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM2               /**< Turn the qspim2 off during WFI/WFE         */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_DSPI_SLP_OFF                     MCU_SUB_PERIPH_CLK_SLP_OFF_DSPI                 /**< Turn the dspi off during WFI/WFE           */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_PDM_SLP_OFF                      MCU_SUB_PERIPH_CLK_SLP_OFF_PDM                  /**< Turn the pdm off during WFI/WFE            */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_I2C_0_SLP_OFF                    MCU_SUB_PERIPH_CLK_SLP_OFF_I2C0                 /**< Turn the i2c0 off during WFI/WFE           */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_I2C_1_SLP_OFF                    MCU_SUB_PERIPH_CLK_SLP_OFF_I2C1                 /**< Turn the i2c1 off during WFI/WFE           */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_I2C_2_SLP_OFF                    MCU_SUB_PERIPH_CLK_SLP_OFF_I2C2                 /**< Turn the i2c0 off during WFI/WFE           */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_I2C_3_SLP_OFF                    MCU_SUB_PERIPH_CLK_SLP_OFF_I2C3                 /**< Turn the i2c1 off during WFI/WFE           */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_I2C_4_SLP_OFF                    MCU_SUB_PERIPH_CLK_SLP_OFF_I2C4                 /**< Turn the i2c0 off during WFI/WFE           */
#define LL_CGC_MCU_PERIPH_CG_LP_EN_I2C_5_SLP_OFF                    MCU_SUB_PERIPH_CLK_SLP_OFF_I2C5                 /**< Turn the i2c1 off during WFI/WFE           */

#define LL_CGC_MCU_PERIPH_SERIALS_SLP_OFF                           ((uint32_t)0x01FC3FFFUL)                        /**< Serial blocks                              */
#define LL_CGC_MCU_PERIPH_SERIALS_SLP_ALL                           ((uint32_t)0x03FFFF3FUL)                        /**< Serial blocks                              */

/** @} */

/** @defgroup LL_CGC_SUBSYS_SECU_CLK_CTRL  Individual Block's Clock Control
  * @brief    Individual block's clock control inside security system
  * @{
  */
#define LL_CGC_MCU_FRC_AES_HCLK_OFF_EN                              MCU_SUB_SECU_CLK_CTRL_AES_HCLK_FORCE_OFF        /**< Force individual aes's clock control       */
#define LL_CGC_MCU_SLP_AES_HCLK_OFF_EN                              MCU_SUB_SECU_CLK_CTRL_AES_HCLK_SLP_OFF          /**< Individual aes's clock control             */
#define LL_CGC_MCU_FRC_HMAC_HCLK_OFF_EN                             MCU_SUB_SECU_CLK_CTRL_HMAC_HCLK_FORCE_OFF       /**< Force individual hmac's clock control      */
#define LL_CGC_MCU_SLP_HMAC_HCLK_OFF_EN                             MCU_SUB_SECU_CLK_CTRL_HMAC_HCLK_SLP_OFF         /**< Individual hmac's clock control            */
#define LL_CGC_MCU_FRC_PKC_HCLK_OFF_EN                              MCU_SUB_SECU_CLK_CTRL_PKC_HCLK_FORCE_OFF        /**< Force individual pkc's clock control       */
#define LL_CGC_MCU_SLP_PKC_HCLK_OFF_EN                              MCU_SUB_SECU_CLK_CTRL_PKC_HCLK_SLP_OFF          /**< Individual pkc's clock control             */
#define LL_CGC_MCU_FRC_PRESENT_HCLK_OFF_EN                          MCU_SUB_SECU_CLK_CTRL_PRESENT_HCLK_FORCE_OFF    /**< Force individual present's clock control   */
#define LL_CGC_MCU_SLP_PRESENT_HCLK_OFF_EN                          MCU_SUB_SECU_CLK_CTRL_PRESENT_HCLK_SLP_OFF      /**< Individual present's clock control         */
#define LL_CGC_MCU_FRC_RAMKEY_HCLK_OFF_EN                           MCU_SUB_SECU_CLK_CTRL_RAMKEY_HCLK_FORCE_OFF     /**< Force individual ramkey's clock control    */
#define LL_CGC_MCU_SLP_RAMKEY_HCLK_OFF_EN                           MCU_SUB_SECU_CLK_CTRL_RAMKEY_HCLK_SLP_OFF       /**< Individual ramkey's clock control          */
#define LL_CGC_MCU_FRC_RNG_HCLK_OFF_EN                              MCU_SUB_SECU_CLK_CTRL_RNG_HCLK_FORCE_OFF        /**< Force individual rng's clock control       */
#define LL_CGC_MCU_SLP_RNG_HCLK_OFF_EN                              MCU_SUB_SECU_CLK_CTRL_RNG_HCLK_SLP_OFF          /**< Individual rng's clock control             */
#define LL_CGC_MCU_FRC_EFUSE_HCLK_OFF_EN                            MCU_SUB_SECU_CLK_CTRL_EFUSE_HCLK_FORCE_OFF      /**< Force individual efuse's clock control     */
#define LL_CGC_MCU_SLP_EFUSE_HCLK_OFF_EN                            MCU_SUB_SECU_CLK_CTRL_EFUSE_HCLK_SLP_OFF        /**< Individual efuse's clock control           */

#define LL_CGC_MCU_SECU_FRC_OFF_HCLK                                ((uint32_t)0x00001555U)                         /**< Hclk for security clock                    */
#define LL_CGC_MCU_SECU_FRC_OFF_WFI_HCLK                            ((uint32_t)0x00002AAAU)                         /**< Hclk for security clock WFI/WFE            */

#define LL_CGC_MCU_SECU_FRC_OFF_ALL                                 (LL_CGC_MCU_SECU_FRC_OFF_HCLK |\
                                                                        LL_CGC_MCU_SECU_FRC_OFF_WFI_HCLK)           /**< Hclk for security clock                    */

#define LL_CGC_MCU_MISC_CLK_DEFAULT                                 ((uint32_t)0x0000003BU)                         /**< Hclk for msic default clock                */

#define LL_CGC_MCU_MISC_CLK                                         ((uint32_t)0x0000003FU)                         /**< Hclk for msic all clock                    */

#define LL_CGC_MCU_MISC_DMA_CLK                                     ((uint32_t)0x00000038U)                         /**< Hclk for msic dma clock                    */

/** @} */

/** @defgroup LL_CGC_SUBSYS_DEFAULT_CLK  Default System Clock Specify
  * @brief    Specify the default system clock when the system is initialized
  * @{
  */
#define LL_CGC_MCU_SUBSYS_DEFAULT_WFI_CLK0                          (LL_CGC_WFI_SECU_HCLK |\
                                                                        LL_CGC_WFI_SIM_HCLK |\
                                                                        LL_CGC_WFI_PWM_HCLK |\
                                                                        LL_CGC_WFI_SNSADC_HCLK |\
                                                                        LL_CGC_WFI_GPIO_HCLK |\
                                                                        LL_CGC_WFI_BLE_BRG_HCLK |\
                                                                        LL_CGC_WFI_SERIAL_HCLK)                     /**< Hclk0 for the system default clock WFI/WFE */

#define LL_CGC_MCU_SUBSYS_DEFAULT_WFI_CLK1                          (LL_CGC_WFI_AON_MCUSUB_HCLK |\
                                                                        LL_CGC_WFI_XF_XQSPI_HCLK |\
                                                                        LL_CGC_WFI_SRAM_HCLK)                       /**< Hclk1 for the system default clock WFI/WFE */


#define LL_CGC_MCU_SUBSYS_DEFAULT_CLK                               (LL_CGC_FRC_SECU_HCLK |\
                                                                        LL_CGC_FRC_SIM_HCLK |\
                                                                        LL_CGC_FRC_SNSADC_HCLK |\
                                                                        LL_CGC_FRC_SERIAL_HCLK)                     /**< Hclk for the system default clock          */

#define LL_CGC_MCU_SUBSYS_DEFAULT_CLK1                              (MCU_SUB_FORCE_SECU_DIV4_PCLK)                  /**< Hclk for the system default clock          */


#define LL_CGC_MCU_PERIPH_CG_DEFAULT                                (LL_CGC_FRC_UART0_PCLK |\
                                                                        LL_CGC_FRC_UART1_PCLK |\
                                                                        LL_CGC_FRC_UART2_PCLK |\
                                                                        LL_CGC_FRC_UART3_PCLK |\
                                                                        LL_CGC_FRC_UART4_PCLK |\
                                                                        LL_CGC_FRC_UART5_PCLK |\
                                                                        LL_CGC_FRC_I2C0_PCLK |\
                                                                        LL_CGC_FRC_I2C1_PCLK |\
                                                                        LL_CGC_FRC_I2C2_PCLK |\
                                                                        LL_CGC_FRC_I2C3_PCLK |\
                                                                        LL_CGC_FRC_I2C4_PCLK |\
                                                                        LL_CGC_FRC_I2C5_PCLK |\
                                                                        LL_CGC_FRC_QSPI0_PCLK |\
                                                                        LL_CGC_FRC_QSPI1_PCLK |\
                                                                        LL_CGC_FRC_QSPI2_PCLK |\
                                                                        LL_CGC_FRC_SPI_M_PCLK |\
                                                                        LL_CGC_FRC_SPI_S_PCLK |\
                                                                        LL_CGC_FRC_I2S_HCLK |\
                                                                        LL_CGC_FRC_I2S_S_PCLK |\
                                                                        LL_CGC_FRC_DSPI_PCLK |\
                                                                        LL_CGC_FRC_PDM_PCLK |\
                                                                        LL_CGC_FRC_PWM_0_PCLK |\
                                                                        LL_CGC_FRC_PWM_1_PCLK)             /**< pclk for the system default periph clock   */

#define LL_CGC_MCU_PERIPH_SLP_CG_DEFAULT                            (MCU_SUB_PERIPH_CLK_SLP_OFF_UART0 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_UART1 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_UART2 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_UART3 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_UART4 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_UART5 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_I2SM |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_I2SS |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_SPIM |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_SPIS |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_PWM0 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_PWM1 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM0 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM1 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM2 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_DSPI |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_PDM |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_I2C0 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_I2C1 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_I2C2 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_I2C3 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_I2C4 |\
                                                                        MCU_SUB_PERIPH_CLK_SLP_OFF_I2C5)          /**< pclk for the system default periph wfi clock   */

#define CGC_CLOCK_ENABLE                                            (1)                                             /**< Bit segment address enable                 */
#define CGC_CLOCK_DISABLE                                           (0)                                             /**< Bit segment address disable                */

#if defined(BIT_BAND_SUPPORT)

#define BIT_SEGMENT_VALUE                                           BIT_ADDR                                        /**< Bit segment address value manipulation     */

#else

#define BIT_BAND(addr, bitnum)                                      (((addr) & 0xF0000000) + 0x2000000 + (((addr) & 0xFFFFF) << 5) + ((bitnum) << 2)) /**< Bit segment address calculation  */
#define MEMORY_ADDR(addr)                                           (*((volatile uint32_t *)(addr)))                /**< Bit segment address type conversion        */
#define BIT_SEGMENT_VALUE(addr, bitnum)                             MEMORY_ADDR(BIT_BAND(addr, bitnum))             /**< Bit segment address value manipulation     */

#endif

/** @} */

/** @} */

/** @} */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup LL_CGC_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
  * @brief Some peripherals automatic turn off clock during WFI. (Include: Security/SIM/HTB/PWM/
  *        ROM/SNSADC/GPIO/DMA/BLE_BRG/APB_SUB/SERIAL/I2S)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SECU_HCLK
  *  CG_CTRL_0 | SIM_HCLK
  *  CG_CTRL_0 | HTB_HCLK
  *  CG_CTRL_0 | PWM_HCLK
  *  CG_CTRL_0 | ROM_HCLK
  *  CG_CTRL_0 | SNSADC_HCLK
  *  CG_CTRL_0 | GPIO_HCLK
  *  CG_CTRL_0 | BLE_BRG_HCLK
  *  CG_CTRL_0 | APB_SUB_HCLK
  *  CG_CTRL_0 | SERIAL_HCLK
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_WFI_SECU_HCLK
  *         @arg @ref LL_CGC_WFI_SIM_HCLK
  *         @arg @ref LL_CGC_WFI_HTB_HCLK
  *         @arg @ref LL_CGC_WFI_PWM_HCLK
  *         @arg @ref LL_CGC_WFI_ROM_HCLK
  *         @arg @ref LL_CGC_WFI_SNSADC_HCLK
  *         @arg @ref LL_CGC_WFI_GPIO_HCLK
  *         @arg @ref LL_CGC_WFI_BLE_BRG_HCLK
  *         @arg @ref LL_CGC_WFI_APB_SUB_HCLK
  *         @arg @ref LL_CGC_WFI_SERIAL_HCLK
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_wfi_off_hclk_0(uint32_t clk_mask)
{
    MODIFY_REG(MCU_RET->MCU_SUBSYS_CG_CTRL[0], LL_CGC_WFI_ALL_HCLK0, clk_mask);
}

/**
  * @brief  Return to clock blocks that is turned off during WFI.(Include: Security/SIM/HTB/PWM/
  *        ROM/SNSADC/GPIO/DMA/BLE_BRG/APB_SUB/SERIAL/I2S)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SECU_HCLK
  *  CG_CTRL_0 | SIM_HCLK
  *  CG_CTRL_0 | HTB_HCLK
  *  CG_CTRL_0 | PWM_HCLK
  *  CG_CTRL_0 | ROM_HCLK
  *  CG_CTRL_0 | SNSADC_HCLK
  *  CG_CTRL_0 | GPIO_HCLK
  *  CG_CTRL_0 | BLE_BRG_HCLK
  *  CG_CTRL_0 | APB_SUB_HCLK
  *  CG_CTRL_0 | SERIAL_HCLK
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_WFI_SECU_HCLK
  *         @arg @ref LL_CGC_WFI_SIM_HCLK
  *         @arg @ref LL_CGC_WFI_HTB_HCLK
  *         @arg @ref LL_CGC_WFI_PWM_HCLK
  *         @arg @ref LL_CGC_WFI_ROM_HCLK
  *         @arg @ref LL_CGC_WFI_SNSADC_HCLK
  *         @arg @ref LL_CGC_WFI_GPIO_HCLK
  *         @arg @ref LL_CGC_WFI_BLE_BRG_HCLK
  *         @arg @ref LL_CGC_WFI_APB_SUB_HCLK
  *         @arg @ref LL_CGC_WFI_SERIAL_HCLK
  */
__STATIC_INLINE uint32_t ll_cgc_get_wfi_off_hclk_0(void)
{
    return READ_REG(MCU_RET->MCU_SUBSYS_CG_CTRL[0]);
}

/**
  * @brief Some peripherals automatic turn off clock during WFI. (Include: AON_MCUSUB/XF_XQSPI/SRAM)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_WFI_AON_MCUSUB_HCLK
  *         @arg @ref LL_CGC_WFI_XF_XQSPI_HCLK
  *         @arg @ref LL_CGC_WFI_SRAM_HCLK
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_wfi_off_hclk_1(uint32_t clk_mask)
{
    MODIFY_REG(MCU_RET->MCU_SUBSYS_CG_CTRL[2], LL_CGC_WFI_ALL_HCLK1, clk_mask);
}

/**
  * @brief  Return to clock blocks that is turned off during WFI.(Include: AON_MCUSUB/XF_XQSPI/SRAM)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_WFI_AON_MCUSUB_HCLK
  *         @arg @ref LL_CGC_WFI_XF_XQSPI_HCLK
  *         @arg @ref LL_CGC_WFI_SRAM_HCLK
  */
__STATIC_INLINE uint32_t ll_cgc_get_wfi_off_hclk_1(void)
{
    return READ_BITS(MCU_RET->MCU_SUBSYS_CG_CTRL[2], LL_CGC_WFI_ALL_HCLK1);
}

/**
  * @brief Some peripherals automatic turn off clock during WFI. (Include: SECU_DIV4/XQSPI_DIV4)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SECU_DIV4_PCLK
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_WFI_SECU_DIV4_PCLK
  *         @arg @ref LL_CGC_WFI_XQSPI_DIV4_PCLK
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_wfi_off_hclk_2(uint32_t clk_mask)
{
    MODIFY_REG(MCU_RET->MCU_PERIPH_PCLK_OFF, LL_CGC_WFI_ALL_HCLK2, clk_mask);
}

/**
  * @brief  Return to clock blocks that is turned off during WFI.(Include: AON_MCUSUB/XF_XQSPI/SRAM)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SECU_DIV4_PCLK
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_WFI_SECU_DIV4_PCLK
  *         @arg @ref LL_CGC_WFI_XQSPI_DIV4_PCLK
  */
__STATIC_INLINE uint32_t ll_cgc_get_wfi_off_hclk_2(void)
{
    return READ_BITS(MCU_RET->MCU_PERIPH_PCLK_OFF, LL_CGC_WFI_ALL_HCLK2);
}

/**
  * @brief Some peripherals automatic turn off clock during WFI. (Include: UART/DSPI.I2C/QSPI.etc)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART0 - UART5/I2C0 - I2C5
  *  PERIPH_GC | I2SM/I2SS/SPIM/SPIS/PWM0/PWM1//QSPIM0/QSPIM1/QSPIM2/DSPI/PDM
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_UART_0_SLP_OFF
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_UART_1_SLP_OFF
  *         .....
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_I2C_5_SLP_OFF
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_wfi_off_hclk_3(uint32_t clk_mask)
{
    MODIFY_REG(MCU_RET->MCU_PERIPH_CLK_SLP_OFF, LL_CGC_MCU_PERIPH_SERIALS_SLP_ALL, clk_mask);
}

/**
  * @brief  Return to clock blocks that is turned off during WFI.(Include: UART/DSPI.I2C/QSPI.etc)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART0 - UART5/I2C0 - I2C5
  *  PERIPH_GC | I2SM/I2SS/SPIM/SPIS/PWM0/PWM1//QSPIM0/QSPIM1/QSPIM2/DSPI/PDM
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_UART_0_SLP_OFF
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_UART_1_SLP_OFF
  *         .....
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_I2C_5_SLP_OFF
  */
__STATIC_INLINE uint32_t ll_cgc_get_wfi_off_hclk_3(void)
{
    return READ_BITS(MCU_RET->MCU_PERIPH_CLK_SLP_OFF, LL_CGC_MCU_PERIPH_SERIALS_SLP_ALL);
}

/**
  * @brief Some peripherals automatic turn off clock during WFI. (Include: AES/HMAC/PKC/RNG.etc)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | AES/HMAC/PKC/RNG/EFUSE
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_MCU_SLP_AES_HCLK_OFF_EN
  *         .....
  *         @arg @ref LL_CGC_MCU_SLP_EFUSE_HCLK_OFF_EN
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_wfi_off_hclk_4(uint32_t clk_mask)
{
    MODIFY_REG(MCU_RET->SECU_CLK_CTRL, LL_CGC_MCU_SECU_FRC_OFF_WFI_HCLK, clk_mask);
}

/**
  * @brief  Return to clock blocks that is turned off during WFI.(Include: AES/HMAC/PKC/RNG.etc)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | AES/HMAC/PKC/RNG/EFUSE
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_MCU_SLP_AES_HCLK_OFF_EN
  *         .....
  *         @arg @ref LL_CGC_MCU_SLP_EFUSE_HCLK_OFF_EN
  */
__STATIC_INLINE uint32_t ll_cgc_get_wfi_off_hclk_4(void)
{
    return READ_BITS(MCU_RET->SECU_CLK_CTRL, LL_CGC_MCU_SECU_FRC_OFF_WFI_HCLK);
}

/**
  * @brief Some peripherals force turn off clock. (Include: Security/SIM/HTB/PWM/ROM/SNSADC/GPIO/
  *        DMA/BLE_BRG/APB_SUB/SERIAL/I2S)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SECU_HCLK
  *  CG_CTRL_1 | SIM_HCLK
  *  CG_CTRL_1 | HTB_HCLK
  *  CG_CTRL_1 | ROM_HCLK
  *  CG_CTRL_1 | SNSADC_HCLK
  *  CG_CTRL_1 | GPIO_HCLK
  *  CG_CTRL_1 | BLE_BRG_HCLK
  *  CG_CTRL_1 | APB_SUB_HCLK
  *  CG_CTRL_1 | SERIAL_HCLK
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_FRC_SECU_HCLK
  *         @arg @ref LL_CGC_FRC_SIM_HCLK
  *         @arg @ref LL_CGC_FRC_HTB_HCLK
  *         @arg @ref LL_CGC_FRC_ROM_HCLK
  *         @arg @ref LL_CGC_FRC_SNSADC_HCLK
  *         @arg @ref LL_CGC_FRC_GPIO_HCLK
  *         @arg @ref LL_CGC_FRC_BLE_BRG_HCLK
  *         @arg @ref LL_CGC_FRC_APB_SUB_HCLK
  *         @arg @ref LL_CGC_FRC_SERIAL_HCLK
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_force_off_hclk_0(uint32_t clk_mask)
{
    MODIFY_REG(MCU_RET->MCU_SUBSYS_CG_CTRL[1], LL_CGC_FRC_ALL_HCLK0, clk_mask);
}

/**
  * @brief  Return to clock blocks that was forcibly closed.(Include: Security/SIM/HTB/
  *        ROM/SNSADC/GPIO/DMA/BLE_BRG/APB_SUB/SERIAL/I2S)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SECU_HCLK
  *  CG_CTRL_1 | SIM_HCLK
  *  CG_CTRL_1 | HTB_HCLK
  *  CG_CTRL_1 | ROM_HCLK
  *  CG_CTRL_1 | SNSADC_HCLK
  *  CG_CTRL_1 | GPIO_HCLK
  *  CG_CTRL_1 | BLE_BRG_HCLK
  *  CG_CTRL_1 | APB_SUB_HCLK
  *  CG_CTRL_1 | SERIAL_HCLK
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_FRC_SECU_HCLK
  *         @arg @ref LL_CGC_FRC_SIM_HCLK
  *         @arg @ref LL_CGC_FRC_HTB_HCLK
  *         @arg @ref LL_CGC_FRC_ROM_HCLK
  *         @arg @ref LL_CGC_FRC_SNSADC_HCLK
  *         @arg @ref LL_CGC_FRC_GPIO_HCLK
  *         @arg @ref LL_CGC_FRC_BLE_BRG_HCLK
  *         @arg @ref LL_CGC_FRC_APB_SUB_HCLK
  *         @arg @ref LL_CGC_FRC_SERIAL_HCLK
  */
__STATIC_INLINE uint32_t ll_cgc_get_force_off_hclk_0(void)
{
    return READ_REG(MCU_RET->MCU_SUBSYS_CG_CTRL[1]);
}

/**
  * @brief Some peripherals force turn off clock. (Include: AON_MCUSUB/XF_XQSPI/SRAM)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_FRC_AON_MCUSUB_HCLK
  *         @arg @ref LL_CGC_FRC_XF_XQSPI_HCLK
  *         @arg @ref LL_CGC_FRC_SRAM_HCLK
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_force_off_hclk_1(uint32_t clk_mask)
{
    MODIFY_REG(MCU_RET->MCU_SUBSYS_CG_CTRL[2], LL_CGC_FRC_ALL_HCLK1, clk_mask);
}

/**
  * @brief  Return to clock blocks that was forcibly closed.(Include: AON_MCUSUB/XF_XQSPI/SRAM)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_FRC_AON_MCUSUB_HCLK
  *         @arg @ref LL_CGC_FRC_XF_XQSPI_HCLK
  *         @arg @ref LL_CGC_FRC_SRAM_HCLK
  */
__STATIC_INLINE uint32_t ll_cgc_get_force_off_hclk_1(void)
{
    return READ_BITS(MCU_RET->MCU_SUBSYS_CG_CTRL[2], LL_CGC_FRC_ALL_HCLK1);
}

/**
  * @brief Some peripherals force turn off clock. (Include: UART0_HCLK/UART1_HCLK/UART2_HCLK/UART3_HCLK/UART4_HCLK/UART5_HCLK/
  *        I2C0_HCLK/I2C1_HCLK/SPIM_HCLK/SPIS_HCLK/QSPI0_HCLK/QSPI1_HCLK/I2S_HCLK/SECU_DIV4_PCLK/XQSPI_DIV4_PCLK/PWM0/PWM1)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART0_PCLK
  *  PERIPH_GC | UART1_PCLK
  *  PERIPH_GC | UART2_PCLK
  *  PERIPH_GC | UART3_PCLK
  *  PERIPH_GC | UART4_PCLK
  *  PERIPH_GC | UART5_PCLK
  *  PERIPH_GC | I2C0_PCLK
  *  PERIPH_GC | I2C1_PCLK
  *  PERIPH_GC | I2C2_PCLK
  *  PERIPH_GC | I2C3_PCLK
  *  PERIPH_GC | I2C4_PCLK
  *  PERIPH_GC | I2C5_PCLK
  *  PERIPH_GC | QSPI0_PCLK
  *  PERIPH_GC | QSPI1_PCLK
  *  PERIPH_GC | QSPI2_PCLK
  *  PERIPH_GC | SPIM_PCLK
  *  PERIPH_GC | SPIS_PCLK
  *  PERIPH_GC | I2S_HCLK
  *  PERIPH_GC | I2S_S_PCLK
  *  PERIPH_GC | DSPI_PCLK
  *  PERIPH_GC | PDM_PCLK
  *  PERIPH_GC | PWM_0_PCLK
  *  PERIPH_GC | PWM_1_PCLK
  *  PERIPH_GC | VTTBL_PCLK
  *  PERIPH_GC | SECU_DIV4_PCLK
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *  PERIPH_GC | SERIALS_HCLK2
  *                   
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_FRC_UART0_PCLK
  *         @arg @ref LL_CGC_FRC_UART1_PCLK
  *         @arg @ref LL_CGC_FRC_UART2_PCLK
  *         @arg @ref LL_CGC_FRC_UART3_PCLK
  *         @arg @ref LL_CGC_FRC_UART4_PCLK
  *         @arg @ref LL_CGC_FRC_UART5_PCLK
  *         @arg @ref LL_CGC_FRC_I2C0_PCLK
  *         @arg @ref LL_CGC_FRC_I2C1_PCLK
  *         @arg @ref LL_CGC_FRC_I2C2_PCLK
  *         @arg @ref LL_CGC_FRC_I2C3_PCLK
  *         @arg @ref LL_CGC_FRC_I2C4_PCLK
  *         @arg @ref LL_CGC_FRC_I2C5_PCLK
  *         @arg @ref LL_CGC_FRC_QSPI0_PCLK
  *         @arg @ref LL_CGC_FRC_QSPI1_PCLK
  *         @arg @ref LL_CGC_FRC_QSPI2_PCLK
  *         @arg @ref LL_CGC_FRC_SPI_M_PCLK
  *         @arg @ref LL_CGC_FRC_SPI_S_PCLK
  *         @arg @ref LL_CGC_FRC_I2S_HCLK
  *         @arg @ref LL_CGC_FRC_I2S_S_PCLK
  *         @arg @ref LL_CGC_FRC_DSPI_PCLK
  *         @arg @ref LL_CGC_FRC_PDM_PCLK
  *         @arg @ref LL_CGC_FRC_PWM_0_PCLK
  *         @arg @ref LL_CGC_FRC_PWM_1_PCLK
  *         @arg @ref LL_CGC_FRC_VTTBL_PCLK
  *         @arg @ref LL_CGC_FRC_SECU_DIV4_PCLK
  *         @arg @ref LL_CGC_FRC_XQSPI_DIV4_PCLK
  *         @arg @ref LL_CGC_FRC_SERIALS_HCLK2
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_force_off_hclk_2(uint32_t clk_mask)
{
    MODIFY_REG(MCU_RET->MCU_PERIPH_PCLK_OFF, LL_CGC_FRC_ALL_HCLK2, clk_mask);
}


/**
  * @brief  Return to clock blocks that was forcibly closed.(Include: UART0_HCLK/UART1_HCLK/UART2_HCLK/UART3_HCLK/UART4_HCLK/UART5_HCLK/
  *        I2C0_HCLK/I2C1_HCLK/SPIM_HCLK/SPIS_HCLK/QSPI0_HCLK/QSPI1_HCLK/I2S_HCLK/SECU_DIV4_PCLK/XQSPI_DIV4_PCLK/PWM0/PWM1)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART0_PCLK
  *  PERIPH_GC | UART1_PCLK
  *  PERIPH_GC | UART2_PCLK
  *  PERIPH_GC | UART3_PCLK
  *  PERIPH_GC | UART4_PCLK
  *  PERIPH_GC | UART5_PCLK
  *  PERIPH_GC | I2C0_PCLK
  *  PERIPH_GC | I2C1_PCLK
  *  PERIPH_GC | I2C2_PCLK
  *  PERIPH_GC | I2C3_PCLK
  *  PERIPH_GC | I2C4_PCLK
  *  PERIPH_GC | I2C5_PCLK
  *  PERIPH_GC | QSPI0_PCLK
  *  PERIPH_GC | QSPI1_PCLK
  *  PERIPH_GC | QSPI2_PCLK
  *  PERIPH_GC | SPIM_PCLK
  *  PERIPH_GC | SPIS_PCLK
  *  PERIPH_GC | I2S_HCLK
  *  PERIPH_GC | I2S_S_PCLK
  *  PERIPH_GC | DSPI_PCLK
  *  PERIPH_GC | PDM_PCLK
  *  PERIPH_GC | PWM_0_PCLK
  *  PERIPH_GC | PWM_1_PCLK
  *  PERIPH_GC | VTTBL_PCLK
  *  PERIPH_GC | SECU_DIV4_PCLK
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *  PERIPH_GC | SERIALS_HCLK2
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_FRC_UART0_PCLK
  *         @arg @ref LL_CGC_FRC_UART1_PCLK
  *         @arg @ref LL_CGC_FRC_UART2_PCLK
  *         @arg @ref LL_CGC_FRC_UART3_PCLK
  *         @arg @ref LL_CGC_FRC_UART4_PCLK
  *         @arg @ref LL_CGC_FRC_UART5_PCLK
  *         @arg @ref LL_CGC_FRC_I2C0_PCLK
  *         @arg @ref LL_CGC_FRC_I2C1_PCLK
  *         @arg @ref LL_CGC_FRC_I2C2_PCLK
  *         @arg @ref LL_CGC_FRC_I2C3_PCLK
  *         @arg @ref LL_CGC_FRC_I2C4_PCLK
  *         @arg @ref LL_CGC_FRC_I2C5_PCLK
  *         @arg @ref LL_CGC_FRC_QSPI0_PCLK
  *         @arg @ref LL_CGC_FRC_QSPI1_PCLK
  *         @arg @ref LL_CGC_FRC_QSPI2_PCLK
  *         @arg @ref LL_CGC_FRC_SPI_M_PCLK
  *         @arg @ref LL_CGC_FRC_SPI_S_PCLK
  *         @arg @ref LL_CGC_FRC_I2S_HCLK
  *         @arg @ref LL_CGC_FRC_I2S_S_PCLK
  *         @arg @ref LL_CGC_FRC_DSPI_PCLK
  *         @arg @ref LL_CGC_FRC_PDM_PCLK
  *         @arg @ref LL_CGC_FRC_PWM_0_PCLK
  *         @arg @ref LL_CGC_FRC_PWM_1_PCLK
  *         @arg @ref LL_CGC_FRC_VTTBL_PCLK
  *         @arg @ref LL_CGC_FRC_SECU_DIV4_PCLK
  *         @arg @ref LL_CGC_FRC_XQSPI_DIV4_PCLK
  *         @arg @ref LL_CGC_FRC_SERIALS_HCLK2
  */
__STATIC_INLINE uint32_t ll_cgc_get_force_off_hclk_2(void)
{
    return READ_BITS(MCU_RET->MCU_PERIPH_PCLK_OFF, LL_CGC_FRC_ALL_HCLK2);
}

/**
  * @brief Some peripherals automatic turn off clock. (Include: AES/HMAC/PKC/RNG.etc)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | AES/HMAC/PKC/PRESENT/RAMKEY/RNG/EFUSE
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_MCU_FRC_AES_HCLK_OFF_EN
  *         .....
  *         @arg @ref LL_CGC_MCU_SLP_EFUSE_HCLK_OFF_EN
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_force_off_hclk_3(uint32_t clk_mask)
{
    MODIFY_REG(MCU_RET->SECU_CLK_CTRL, LL_CGC_MCU_SECU_FRC_OFF_HCLK, clk_mask);
}

/**
  * @brief  Return to clock blocks that is turned off.(Include: AES/HMAC/PKC/RNG.etc)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | AES/HMAC/PKC/PRESENT/RAMKEY/RNG/EFUSE
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_MCU_FRC_AES_HCLK_OFF_EN
  *         .....
  *         @arg @ref LL_CGC_MCU_SLP_EFUSE_HCLK_OFF_EN
  */
__STATIC_INLINE uint32_t ll_cgc_get_force_off_hclk_3(void)
{
    return READ_BITS(MCU_RET->SECU_CLK_CTRL, LL_CGC_MCU_SECU_FRC_OFF_HCLK);
}


/**
  * @brief  Enable security blocks(including AES, PKC, Present, HMAC) automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SECU_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_secu_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SECU_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable security blocks(including AES, PKC, Present, HMAC) automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SECU_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_secu_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SECU_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the security blocks(including AES, PKC, Present, HMAC) automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SECU_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_secu_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SECU_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable SIM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SIM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_sim_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SIM_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable SIM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SIM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_sim_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SIM_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the SIM automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SIM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_sim_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SIM_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable Hopping Table automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | HTB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_htb_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_HTB_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable Hopping Table automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | HTB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_htb_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_HTB_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the Hopping Table automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | HTB_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_htb_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_HTB_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable PWM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | PWM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_pwm_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_PWM_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable PWM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | PWM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_pwm_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_PWM_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the PWM automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | PWM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_pwm_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_PWM_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable ROM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | ROM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_rom_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_ROM_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable ROM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | ROM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_rom_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_ROM_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the ROM automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | ROM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_rom_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_ROM_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable SNSADC automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SNSADC_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_snsadc_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SNSADC_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable SNSADC automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SNSADC_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_snsadc_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SNSADC_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the SNSADC automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SNSADC_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_snsadc_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SNSADC_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable GPIO automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | GPIO_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_gpio_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_GPIO_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable GPIO automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | GPIO_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_gpio_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_GPIO_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the GPIO automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | GPIO_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_gpio_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_GPIO_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable DMA automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | DMA_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_dma_hclk(void)
{
    // BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_DMA_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable DMA automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | DMA_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_dma_hclk(void)
{
    // BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_DMA_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the DMA automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | DMA_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_dma_hclk(void)
{
    // return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_DMA_HCLK_Pos) == (CGC_CLOCK_ENABLE));
    return 0;
}

/**
  * @brief  Enable BLE Bridge automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | BLE_BRG_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_ble_brg_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_BLE_BRG_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable BLE Bridge automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | BLE_BRG_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_ble_brg_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_BLE_BRG_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the BLE Bridge automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | BLE_BRG_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_ble_brg_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_BLE_BRG_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable APB Subsystem automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | APB_SUB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_apb_sub_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_APB_SUB_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable APB Subsystem automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | APB_SUB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_apb_sub_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_APB_SUB_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the APB Subsystem automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | APB_SUB_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_apb_sub_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_APB_SUB_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable serial blocks(including I2C, UART, QSPI, I2S, SPI) automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SERIAL_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_serial_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SERIAL_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable serial blocks(including I2C, UART, QSPI, I2S, SPI) automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SERIAL_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_serial_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SERIAL_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the serial blocks(including I2C, UART, QSPI, I2S, SPI) automatic turn off
  *         clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SERIAL_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_serial_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SERIAL_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable USB automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | USB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_usb_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_USB_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable USB automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | USB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_usb_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_USB_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the USB automatic turn off
  *         clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | USB_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_usb_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_USB_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable AON_MUCSUB automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_aon_mcusub_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_AON_MCUSUB_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable AON_MUCSUB automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_aon_mcusub_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_AON_MCUSUB_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the AON_MUCSUB automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_aon_mcusub_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_AON_MCUSUB_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable XQSPI automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_xqspi_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_XF_XQSPI_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable XQSPI automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_xqspi_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_XF_XQSPI_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the XQSPI automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_xqspi_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_XF_XQSPI_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable SRAM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_sram_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_SRAM_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable SRAM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_sram_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_SRAM_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the SRAM automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_sram_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_SRAM_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable security blocks automatic turn off div4 clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SECU_DIV4_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_secu_div4_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_WFI_SECU_DIV4_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable security blocks automatic turn off div4 clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SECU_DIV4_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_secu_div4_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_WFI_SECU_DIV4_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the security blocks automatic turn off div4
  *         clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SECU_DIV4_PCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_secu_div4_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_WFI_SECU_DIV4_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable XQSPI automatic turn off div4 clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_xqspi_div4_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_WFI_XQSPI_DIV4_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable XQSPI automatic turn off div4 clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_xqspi_div4_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_WFI_XQSPI_DIV4_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the XQSPI automatic turn off div4 clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_xqspi_div4_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_WFI_XQSPI_DIV4_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for security blocks(including AES, PKC, Present, HMAC).
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SECU_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_secu_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SECU_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for security blocks(including AES, PKC, Present, HMAC).
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SECU_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_secu_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SECU_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for security blocks(including AES, PKC, Present, HMAC) is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SECU_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_secu_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SECU_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for SIM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SIM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_sim_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SIM_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for SIM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SIM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_sim_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SIM_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for SIM is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SIM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_sim_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SIM_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for Hopping Table.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | HTB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_htb_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_HTB_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for Hopping Table.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | HTB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_htb_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_HTB_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for Hopping Table is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | HTB_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_htb_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_HTB_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for ROM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | ROM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_rom_hclk(void)
{
     BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_ROM_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for ROM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | ROM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_rom_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_ROM_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for ROM is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | ROM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_rom_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_ROM_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for SNSADC.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SNSADC_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_snsadc_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SNSADC_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for SNSADC.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SNSADC_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_snsadc_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SNSADC_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for SNSADC is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SNSADC_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_snsadc_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SNSADC_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for GPIO.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | GPIO_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_gpio_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_GPIO_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for GPIO.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | GPIO_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_gpio_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_GPIO_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for GPIO is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | GPIO_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_gpio_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_GPIO_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for BLE Bridge.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | BLE_BRG_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_ble_brg_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_BLE_BRG_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for BLE Bridge.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | BLE_BRG_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_ble_brg_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_BLE_BRG_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for BLE Bridge is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | BLE_BRG_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_ble_brg_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_BLE_BRG_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for APB Subsystem.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | APB_SUB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_apb_sub_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_APB_SUB_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for APB Subsystem.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | APB_SUB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_apb_sub_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_APB_SUB_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for APB Subsystem is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | APB_SUB_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_apb_sub_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_APB_SUB_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for serial blocks(including I2C, UART, QSPI, I2S, SPI).
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SERIAL_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_serial_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SERIAL_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for serial blocks(including I2C, UART, QSPI, I2S, SPI).
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SERIAL_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_serial_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SERIAL_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for serial blocks(including I2C, UART, QSPI, I2S, SPI) is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SERIAL_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_serial_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SERIAL_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for USB.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | USB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_usb_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_USB_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for USB.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | USB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_usb_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_USB_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief Indicate whether the clock for USB is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | USB_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_usb_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_USB_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for AON_MUCSUB.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_aon_mcusub_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_AON_MCUSUB_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for AON_MUCSUB.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_aon_mcusub_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_AON_MCUSUB_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for AON_MUCSUB is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_aon_mcusub_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_AON_MCUSUB_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for XQSPI.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_xqspi_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_XF_XQSPI_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for XQSPI.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_xqspi_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_XF_XQSPI_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for XQSPI is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_xqspi_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_XF_XQSPI_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for SRAM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_sram_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_SRAM_HCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for SRAM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_sram_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_SRAM_HCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for SRAM is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_sram_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_SRAM_HCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for UART0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART0_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_uart0_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART0_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for UART0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART0_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_uart0_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART0_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for UART0 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART0_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_uart0_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART0_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for UART1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART1_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_uart1_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART1_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for UART1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART1_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_uart1_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART1_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for UART1 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART1_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_uart1_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART1_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for UART2.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART2_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_uart2_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART2_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for UART2.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART2_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_uart2_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART2_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for UART2 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART2_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_uart2_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART2_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for UART3.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART3_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_uart3_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART3_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for UART3.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART3_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_uart3_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART3_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for UART3 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART3_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_uart3_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART3_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for UART4.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART4_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_uart4_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART4_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for UART4.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART4_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_uart4_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART4_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for UART4 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART4_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_uart4_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART4_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for UART5.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART5_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_uart5_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART5_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for UART5.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART5_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_uart5_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART5_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for UART5 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART5_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_uart5_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_UART5_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for I2C0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C0_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_i2c0_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C0_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for I2C0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C0_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_i2c0_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C0_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for I2C0 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C0_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_i2c0_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C0_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for I2C1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C1_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_i2c1_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C1_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for I2C1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C1_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_i2c1_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C1_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for I2C1 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C1_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_i2c1_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C1_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for I2C2.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C2_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_i2c2_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C2_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for I2C2.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C2_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_i2c2_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C2_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for I2C2 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C2_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_i2c2_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C2_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for I2C3.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C3_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_i2c3_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C3_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for I2C3.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C3_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_i2c3_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C3_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for I2C3 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C3_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_i2c3_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C3_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for I2C4.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C4_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_i2c4_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C4_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for I2C4.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C4_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_i2c4_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C4_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for I2C4 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C4_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_i2c4_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C4_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for I2C5.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C5_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_i2c5_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C5_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for I2C5.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C5_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_i2c5_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C5_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for I2C5 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C5_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_i2c5_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2C5_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for SPIM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SPIM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_spim_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_SPI_M_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for SPIM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SPIM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_spim_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_SPI_M_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for SPIM is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SPIM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_spim_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_SPI_M_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for SPIS.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SPIS_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_spis_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_SPI_S_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for SPIS.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SPIS_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_spis_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_SPI_S_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for SPIS is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SPIS_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_spis_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_SPI_S_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for QSPI0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI0_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_qspi0_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_QSPI0_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for QSPI0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI0_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_qspi0_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_QSPI0_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for QSPI0 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI0_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_qspi0_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_QSPI0_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for QSPI1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI1_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_qspi1_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_QSPI1_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for QSPI1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI1_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_qspi1_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_QSPI1_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for QSPI1 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI1_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_qspi1_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_QSPI1_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for QSPI2.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI2_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_qspi2_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_QSPI2_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for QSPI2.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI2_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_qspi2_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_QSPI2_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for QSPI2 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI2_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_qspi2_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_QSPI2_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}


/**
  * @brief  Enabling force to turn off the clock for I2S master.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_i2s_m_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2S_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for I2S master.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_i2s_m_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2S_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for I2S master is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_i2s_m_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2S_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for I2S slave.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_S_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_i2s_s_p_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2S_S_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for I2S slave.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_S_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_i2s_s_p_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2S_S_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for I2S slave is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_S_PCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_i2s_s_p_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_I2S_S_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for DSPI slave.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | DSPI_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_dspi_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_DSPI_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for DSPI slave.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | DSPI_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_dspi_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_DSPI_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for DSPI is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | DSPI_PCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_dspi_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_DSPI_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for PDM slave.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | PDM_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_pdm_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_PDM_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for PDM slave.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | PDM_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_pdm_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_PDM_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for PDM is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | PDM_PCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_pdm_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_PDM_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the div4 clock for security blocks.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_secu_div4_pclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_SECU_DIV4_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the div4 clock for security blocks.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_secu_div4_pclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_SECU_DIV4_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the div4 clock for security blocks is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_secu_div4_pclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_SECU_DIV4_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}
/**
  * @brief  Enabling force to turn off the div4 clock for xf qspi blocks.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | XQSPI_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_xf_xqspi_div4_pclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_XQSPI_DIV4_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the div4 clock for xf qspi blocks.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | XQSPI_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_xf_xqspi_div4_pclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_XQSPI_DIV4_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the div4 clock for xf qspi blocks is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | XQSPI_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_xf_xqspi_div4_pclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_XQSPI_DIV4_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for PWM0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | PWM0_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_pwm0_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_PWM_0_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for PWM0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | PWM0_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_pwm0_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_PWM_0_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for PWM0 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | PWM0_PCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_pwm0_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_PWM_0_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for PWM1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | PWM1_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_pwm1_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_PWM_1_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for PWM1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | PWM1_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_pwm1_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_PWM_1_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for PWM1 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | PWM1_PCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_pwm1_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_PWM_1_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for VTTBL.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | VTTBL_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_vttbl_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_VTTBL_PCLK_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for VTTBL.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | VTTBL_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_vttbl_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_VTTBL_PCLK_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for VTTBL is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | VTTBL_PCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_vttbl_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_PCLK_OFF, MCU_SUB_FORCE_VTTBL_PCLK_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief Some peripherals has low power feature. (Include: UART/I2S/SPIM/SPIS/I2C/AHB BUS)
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | UART_LP_SCLK
  *  CG_LP_EN | UART_LP_PCLK
  *  CG_LP_EN | I2S_LP
  *  CG_LP_EN | SPIM_LP_SCLK
  *  CG_LP_EN | SPIS_LP_SCLK
  *  CG_LP_EN | I2C_LP_SCLK
  *  CG_LP_EN | AHB_BUS_LP
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_AHB_BUS_LP_EN
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_I2C_LP_SCLK_EN
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_SPIS_LP_SCLK_EN
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_SPIM_LP_SCLK_EN
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_I2S_LP_EN
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_UART_LP_PCLK_EN
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_UART_LP_SCLK_EN
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_mcu_periph_low_power(uint32_t clk_mask)
{
    WRITE_REG(MCU_RET->MCU_PERIPH_CG_LP_EN, clk_mask);
}

/**
  * @brief  Return to clock blocks that has low power feature. (Include: UART/I2S/SPIM/SPIS/I2C/AHB BUS)
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | UART_LP_SCLK
  *  CG_LP_EN | UART_LP_PCLK
  *  CG_LP_EN | I2S_LP
  *  CG_LP_EN | SPIM_LP_SCLK
  *  CG_LP_EN | SPIS_LP_SCLK
  *  CG_LP_EN | I2C_LP_SCLK
  *  CG_LP_EN | AHB_BUS_LP
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_AHB_BUS_LP_EN
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_I2C_LP_SCLK_EN
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_SPIS_LP_SCLK_EN
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_SPIM_LP_SCLK_EN
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_I2S_LP_EN
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_UART_LP_PCLK_EN
  *         @arg @ref LL_CGC_MCU_PERIPH_CG_LP_EN_UART_LP_SCLK_EN
  */
__STATIC_INLINE uint32_t ll_cgc_get_mcu_periph_low_power(void)
{
    return READ_REG(MCU_RET->MCU_PERIPH_CG_LP_EN);
}

/**
  * @brief  Enable uart sclk low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | UART_LP_SCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_uart_sclk_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_UART_LP_SCLK_EN_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable uart sclk low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | UART_LP_SCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_uart_sclk_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_UART_LP_SCLK_EN_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the uart sclk low-power is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | UART_LP_SCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_uart_sclk_low_power(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_UART_LP_SCLK_EN_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable uart pclk low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | UART_LP_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_uart_pclk_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_UART_LP_PCLK_EN_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable uart pclk low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | UART_LP_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_uart_pclk_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_UART_LP_PCLK_EN_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the uart pclk low-power is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | UART_LP_PCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_uart_pclk_low_power(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_UART_LP_PCLK_EN_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable i2s low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | I2S_LP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_i2s_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_I2S_LP_EN_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable i2s low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | I2S_LP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_i2s_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_I2S_LP_EN_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the i2s low-power is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | I2S_LP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_i2s_low_power(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_I2S_LP_EN_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable spim sclk low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | SPIM_LP_SCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_spim_sclk_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_SPIM_LP_SCLK_EN_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable spim sclk low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | SPIM_LP_SCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_spim_sclk_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_SPIM_LP_SCLK_EN_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the spim sclk low-power is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | SPIM_LP_SCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_spim_sclk_low_power(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_SPIM_LP_SCLK_EN_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable spis sclk low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | SPIS_LP_SCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_spis_sclk_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_SPIS_LP_SCLK_EN_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable spis sclk low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | SPIS_LP_SCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_spis_sclk_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_SPIS_LP_SCLK_EN_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the spis sclk low-power is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | SPIS_LP_SCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_spis_sclk_low_power(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_SPIS_LP_SCLK_EN_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable i2c sclk low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | I2C_LP_SCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_i2c_sclk_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_I2C_LP_SCLK_EN_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable i2c sclk low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | I2C_LP_SCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_i2c_sclk_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_I2C_LP_SCLK_EN_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the i2c sclk low-power is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | I2C_LP_SCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_i2c_sclk_low_power(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_I2C_LP_SCLK_EN_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable ahb bus low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | AHB_BUS_LP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_ahb_bus_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_AHB_BUS_LP_EN_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable ahb bus low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | AHB_BUS_LP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_ahb_bus_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_AHB_BUS_LP_EN_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the ahb bus low-power is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | AHB_BUS_LP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_ahb_bus_low_power(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_AHB_BUS_LP_EN_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable QSPIM low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | QSPIM_LP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_qspim_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_QSPIM_EN_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable QSPIM low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | QSPIM_LP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_qspim_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_QSPIM_EN_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the QSPIM low-power is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | QSPIM_LP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_qspim_low_power(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_EN_QSPIM_EN_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable AHB2APB bus low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | AHB2APB_BUS_LP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_ahb2apb_sync_bus_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_AHB2APB_SYNC_EN_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable AHB2APB bus low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | AHB2APB_BUS_LP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_ahb2apb_sync_bus_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_AHB2APB_SYNC_EN_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the AHB2APB bus low-power is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | AHB2APB_BUS_LP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_ahb2apb_sync_bus_low_power(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_AHB2APB_SYNC_EN_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable ahb bus low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | AHB_BUS_LP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_ahb2apb_async_bus_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_AHB2APB_ASYNC_EN_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable ahb bus low-power feature
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | AHB_BUS_LP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_ahb2apb_async_bus_low_power(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_AHB2APB_ASYNC_EN_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the ahb bus low-power is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CG_LP_EN | AHB_BUS_LP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_ahb2apb_async_bus_low_power(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CG_LP_EN, MCU_SUB_PERIPH_CG_LP_AHB2APB_ASYNC_EN_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn UART0 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART0_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_uart0_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART0_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn UART0 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART0_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_uart0_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART0_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn UART0 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART0_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_uart0_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART0_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn UART1 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART1_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_uart1_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART1_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn UART1 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART1_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_uart1_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART1_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn UART1 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART1_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_uart1_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART1_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn UART2 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART2_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_uart2_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART2_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn UART2 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART2_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_uart2_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART2_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn UART2 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART2_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_uart2_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART2_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn UART3 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART3_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_uart3_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART3_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn UART3 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART3_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_uart3_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART3_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn UART3 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART3_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_uart3_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART3_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn UART4 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART4_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_uart4_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART4_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn UART4 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART4_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_uart4_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART4_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn UART4 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART4_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_uart4_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART4_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn UART5 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART5_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_uart5_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART5_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn UART5 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART5_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_uart5_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART5_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn UART5 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | UART5_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_uart5_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_UART5_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn I2C0 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C0_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_i2c0_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C0_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn I2C0 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C0_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_i2c0_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C0_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn I2C0 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C0_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_i2c0_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C0_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn I2C1 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C1_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_i2c1_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C1_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn I2C1 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C1_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_i2c1_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C1_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn I2C1 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C1_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_i2c1_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C1_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn I2C2 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C2_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_i2c2_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C2_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn I2C2 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C2_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_i2c2_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C2_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn I2C2 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C2_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_i2c2_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C2_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn I2C3 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C3_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_i2c3_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C3_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn I2C3 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C3_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_i2c3_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C3_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn I2C3 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C3_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_i2c3_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C3_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn I2C4 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C4_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_i2c4_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C4_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn I2C4 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C4_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_i2c4_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C4_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn I2C4 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C4_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_i2c4_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C4_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn I2C5 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C5_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_i2c5_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C5_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn I2C5 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C5_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_i2c5_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C5_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn I2C5 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2C5_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_i2c5_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2C5_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn I2S_M off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2SM_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_i2s_m_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2SM_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn I2S_M off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2SM_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_i2s_m_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2SM_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn I2S_M off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2SM_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_i2s_m_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2SM_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn I2S_S off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2SS_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_i2s_s_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2SS_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn I2S_S off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2SS_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_i2s_s_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2SS_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn I2S_S off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | I2SS_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_i2s_s_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_I2SS_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn SPI_M off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | SPIM_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_spi_m_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_SPIM_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn SPI_M off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | SPIM_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_spi_m_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_SPIM_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn SPI_M off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | SPIM_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_spi_m_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_SPIM_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn SPI_S off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | SPIS_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_spi_s_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_SPIS_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn SPI_S off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | SPIS_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_spi_s_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_SPIS_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn SPI_S off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | SPIS_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_spi_s_slp_wfi(void)
{
   return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_SPIS_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn pwm0 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | PWM0_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_pwm0_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_PWM0_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn pwm0 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | PWM0_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_pwm0_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_PWM0_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn pwm0 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | PWM0_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_pwm0_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_PWM0_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn pwm1 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | PWM1_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_pwm1_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_PWM1_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn pwm1 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | PWM1_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_pwm1_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_PWM1_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn pwm1 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | PWM1_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_pwm1_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_PWM1_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn QSPIM0 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | QSPIM0_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_qspim0_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM0_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn QSPIM0 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | QSPIM0_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_qspim0_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM0_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn QSPIM0 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | QSPIM0_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_qspim0_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM0_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn QSPIM1 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | QSPIM1_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_qspim1_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM1_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn QSPIM1 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | QSPIM1_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_qspim1_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM1_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn QSPIM1 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | QSPIM1_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_qspim1_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM1_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn QSPIM2 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | QSPIM2_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_qspim2_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM2_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn QSPIM2 off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | QSPIM2_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_qspim2_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM2_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn QSPIM2 off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | QSPIM2_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_qspim2_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_QSPIM2_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn DSPI off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | DSPI_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_dspi_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_DSPI_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn DSPI off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | DSPI_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_dspi_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_DSPI_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn DSPI off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | DSPI_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_dspi_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_DSPI_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable turn PDM off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | PDM_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_pdm_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_PDM_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable turn PDM off during WFI/WFE
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | PDM_SLP
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_pdm_slp_wfi(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_PDM_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether turn PDM off during WFI/WFE is enabled.
  *
  *  Register | BitsName
  *  ---------|--------
  *  CLK_SLP_OFF | PDM_SLP
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_pdm_slp_wfi(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_PERIPH_CLK_SLP_OFF, MCU_SUB_PERIPH_CLK_SLP_OFF_PDM_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief Individual block's clock control inside security system which was forced to turn off (Include: AES/HMAC/PKC/PRESENT/RAMKAY/RNG/EFUSE)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | AES_HCLK_FRC_OFF
  *  SECU_CLK_CTRL | HMAC_HCLK_FRC_OFF
  *  SECU_CLK_CTRL | PKC_HCLK_FRC_OFF
  *  SECU_CLK_CTRL | PRESENT_HCLK_FRC_OFF
  *  SECU_CLK_CTRL | RAMKEY_HCLK_FRC_OFF
  *  SECU_CLK_CTRL | RNG_HCLK_FRC_OFF
  *  SECU_CLK_CTRL | EFUSE_HCLK_FRC_OFF
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_MCU_FRC_AES_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_FRC_HMAC_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_FRC_PKC_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_FRC_PRESENT_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_FRC_RAMKEY_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_FRC_RNG_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_FRC_EFUSE_HCLK_OFF_EN
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_force_off_hclk_secu(uint32_t clk_mask)
{
    MODIFY_REG(MCU_RET->SECU_CLK_CTRL, LL_CGC_MCU_SECU_FRC_OFF_HCLK, clk_mask);
}

/**
  * @brief  Return to clock blocks that was forcibly closed inside security system.(Include: AES/HMAC/PKC/PRESENT/RAMKAY/RNG/EFUSE)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | AES_HCLK_FRC_OFF
  *  SECU_CLK_CTRL | HMAC_HCLK_FRC_OFF
  *  SECU_CLK_CTRL | PKC_HCLK_FRC_OFF
  *  SECU_CLK_CTRL | PRESENT_HCLK_FRC_OFF
  *  SECU_CLK_CTRL | RAMKEY_HCLK_FRC_OFF
  *  SECU_CLK_CTRL | RNG_HCLK_FRC_OFF
  *  SECU_CLK_CTRL | EFUSE_HCLK_FRC_OFF
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_MCU_FRC_AES_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_FRC_HMAC_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_FRC_PKC_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_FRC_PRESENT_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_FRC_RAMKEY_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_FRC_RNG_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_FRC_EFUSE_HCLK_OFF_EN
  */
__STATIC_INLINE uint32_t ll_cgc_get_force_off_secu(void)
{
    return READ_BITS(MCU_RET->SECU_CLK_CTRL, LL_CGC_MCU_SECU_FRC_OFF_HCLK);
}

/**
  * @brief Some security blocks automatic turn off clock during WFI/WFE. (Include: AES/HMAC/PKC/PRESENT/RAMKAY/RNG/EFUSE)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | AES_HCLK_SLP_OFF
  *  SECU_CLK_CTRL | HMAC_HCLK_SLP_OFF
  *  SECU_CLK_CTRL | PKC_HCLK_SLP_OFF
  *  SECU_CLK_CTRL | PRESENT_HCLK_SLP_OFF
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_MCU_SLP_AES_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_SLP_HMAC_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_SLP_PKC_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_SLP_PRESENT_HCLK_OFF_EN
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_slp_off_hclk_secu(uint32_t clk_mask)
{
    MODIFY_REG(MCU_RET->SECU_CLK_CTRL, LL_CGC_MCU_SECU_FRC_OFF_WFI_HCLK, clk_mask);
}

/**
  * @brief  Return to security clock blocks that is turned off during WFI/WFE.(Include: AES/HMAC/PKC/PRESENT/RAMKAY/RNG/EFUSE)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | AES_HCLK_SLP_OFF
  *  SECU_CLK_CTRL | HMAC_HCLK_SLP_OFF
  *  SECU_CLK_CTRL | PKC_HCLK_SLP_OFF
  *  SECU_CLK_CTRL | PRESENT_HCLK_SLP_OFF
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_MCU_SLP_AES_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_SLP_HMAC_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_SLP_PKC_HCLK_OFF_EN
  *         @arg @ref LL_CGC_MCU_SLP_PRESENT_HCLK_OFF_EN
  */
__STATIC_INLINE uint32_t ll_cgc_get_slp_off_secu(void)
{
    return READ_BITS(MCU_RET->SECU_CLK_CTRL, LL_CGC_MCU_SECU_FRC_OFF_WFI_HCLK);
}

/**
  * @brief  Enabling force to turn off the clock for AES.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | AES_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_aes_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_AES_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for AES.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | AES_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_aes_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_AES_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for AES is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | AES_HCLK_FRC_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_aes_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_AES_HCLK_FORCE_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for HMAC.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | HMAC_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_hmac_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_HMAC_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for HMAC.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | HMAC_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_hmac_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_HMAC_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for HMAC is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | HMAC_HCLK_FRC_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_hmac_hclk(void)
{
   return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_HMAC_HCLK_FORCE_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for PKC.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | PKC_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_pkc_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_PKC_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for PKC.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | PKC_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_pkc_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_PKC_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for PKC is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | PKC_HCLK_FRC_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_pkc_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_PKC_HCLK_FORCE_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for PRESENT.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | PRESENT_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_present_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_PRESENT_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for PRESENT.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | PRESENT_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_present_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_PRESENT_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for PRESENT is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | PRESENT_HCLK_FRC_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_present_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_PRESENT_HCLK_FORCE_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for RAMKEY.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | RAMKEY_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_ramkey_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_RAMKEY_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for RAMKEY.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | RAMKEY_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_ramkey_hclk(void)
{
   BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_RAMKEY_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for RAMKEY is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | RAMKEY_HCLK_FRC_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_ramkey_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_RAMKEY_HCLK_FORCE_OFF_Pos) == (CGC_CLOCK_ENABLE));
}


/**
  * @brief  Enabling force to turn off the clock for RNG.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | RNG_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_rng_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_RNG_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for RNG.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | RNG_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_rng_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_RNG_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for RNG is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | RNG_HCLK_FRC_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_rng_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_RNG_HCLK_FORCE_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enabling force to turn off the clock for EFUSE.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | EFUSE_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_efuse_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_EFUSE_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disabling force to turn off the clock for EFUSE.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | EFUSE_HCLK_FRC_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_efuse_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_EFUSE_HCLK_FORCE_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the clock for EFUSE is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | EFUSE_HCLK_FRC_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_efuse_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_EFUSE_HCLK_FORCE_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable AES automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | AES_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_aes_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_AES_HCLK_SLP_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable AES automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | AES_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_aes_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_AES_HCLK_SLP_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the AES automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | AES_HCLK_SLP_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_aes_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_AES_HCLK_SLP_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable HMAC automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | HMAC_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_hmac_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_HMAC_HCLK_SLP_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable HMAC automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | HMAC_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_hmac_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_HMAC_HCLK_SLP_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the HMAC automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | HMAC_HCLK_SLP_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_hmac_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_HMAC_HCLK_SLP_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable PKC automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | PKC_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_pkc_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_PKC_HCLK_SLP_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable PKC automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | PKC_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_pkc_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_PKC_HCLK_SLP_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the PKC automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | PKC_HCLK_SLP_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_pkc_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_PKC_HCLK_SLP_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable PRESENT automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | PRESENT_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_present_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_PRESENT_HCLK_SLP_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable PRESENT automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | PRESENT_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_present_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_PRESENT_HCLK_SLP_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the PRESENT automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | PRESENT_HCLK_SLP_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_present_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_PRESENT_HCLK_SLP_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable RAMKEY automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | RAMKEY_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_ramkey_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_RAMKEY_HCLK_SLP_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable RAMKEY automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | RAMKEY_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_ramkey_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_RAMKEY_HCLK_SLP_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the RAMKEY automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | RAMKEY_HCLK_SLP_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_ramkey_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_RAMKEY_HCLK_SLP_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable RNG automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | RNG_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_rng_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_RNG_HCLK_SLP_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable RNG automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | RNG_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_rng_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_RNG_HCLK_SLP_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the RNG automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | RNG_HCLK_SLP_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_rng_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_RNG_HCLK_SLP_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable EFUSE automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | EFUSE_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_efuse_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_EFUSE_HCLK_SLP_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable EFUSE automatic turn off clock during WFI/WFE
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | EFUSE_HCLK_SLP_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_efuse_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_EFUSE_HCLK_SLP_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the EFUSE automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  SECU_CLK_CTRL | EFUSE_HCLK_SLP_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_efuse_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->SECU_CLK_CTRL, MCU_SUB_SECU_CLK_CTRL_EFUSE_HCLK_SLP_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief Some MISC_CLK blocks turn off clock. (Include: GPADC/XQSPI/DMA0/DMA1/DMA2)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK | GPADC/XQSPI/DMA0/DMA1/DMA2
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_misc_clk(uint32_t clk_mask)
{
    MODIFY_REG(MCU_RET->MCU_MISC_CLK, LL_CGC_MCU_MISC_CLK, clk_mask);
}

/**
  * @brief  Return to MISC_CLK clock blocks that is turned off.(Include: GPADC/XQSPI/DMA0/DMA1/DMA2)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK | GPADC/XQSPI/DMA0/DMA1/DMA2
  */
__STATIC_INLINE uint32_t ll_cgc_get_misc_clk(void)
{
    return READ_BITS(MCU_RET->MCU_MISC_CLK, LL_CGC_MCU_MISC_CLK);
}

/**
  * @brief  Enable XQSPI SCK CLK turn off
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK |XQSPI SCK CLK_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_xqspi_sck(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_MISC_CLK, MCU_SUB_CLK_CTRL_XQSPI_SCK_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable XQSPI SCK CLK turn off
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK | XQSPI SCK CLK_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_xqspi_sck(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_MISC_CLK, MCU_SUB_CLK_CTRL_XQSPI_SCK_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the XQSPI SCK CLK automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK | XQSPI SCK CLK_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_xqspi_sck(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_MISC_CLK, MCU_SUB_CLK_CTRL_XQSPI_SCK_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable DMA0 turn off
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK |DMA0_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_dma0_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_MISC_CLK, MCU_SUB_CLK_CTRL_DMA0_HCLK_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable DMA0 turn off
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK | DMA0_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_dma0_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_MISC_CLK, MCU_SUB_CLK_CTRL_DMA0_HCLK_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the DMA0 automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK | DMA0_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_dma0_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_MISC_CLK, MCU_SUB_CLK_CTRL_DMA0_HCLK_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable DMA1 turn off
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK |DMA1_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_dma1_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_MISC_CLK, MCU_SUB_CLK_CTRL_DMA1_HCLK_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable DMA1 turn off
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK | DMA1_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_dma1_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_MISC_CLK, MCU_SUB_CLK_CTRL_DMA1_HCLK_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the DMA1 automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK | DMA1_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_dma1_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_MISC_CLK, MCU_SUB_CLK_CTRL_DMA1_HCLK_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/**
  * @brief  Enable DMA2 turn off
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK |DMA2_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_dma2_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_MISC_CLK, MCU_SUB_CLK_CTRL_DMA2_HCLK_OFF_Pos) = CGC_CLOCK_ENABLE;
}

/**
  * @brief  Disable DMA2 turn off
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK | DMA2_OFF
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_dma2_hclk(void)
{
    BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_MISC_CLK, MCU_SUB_CLK_CTRL_DMA2_HCLK_OFF_Pos) = CGC_CLOCK_DISABLE;
}

/**
  * @brief  Indicate whether the DMA2 automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  MCU_MISC_CLK | DMA2_OFF
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_dma2_hclk(void)
{
    return (BIT_SEGMENT_VALUE((uint32_t)&MCU_RET->MCU_MISC_CLK, MCU_SUB_CLK_CTRL_DMA2_HCLK_OFF_Pos) == (CGC_CLOCK_ENABLE));
}

/** @} */

#endif /* CGC */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_CGC_H__ */

/** @} */

/** @} */

/** @} */

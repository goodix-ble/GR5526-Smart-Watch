/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_cgc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of CGC HAL library.
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

/** @defgroup HAL_CGC CGC
  * @brief CGC HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_CGC_H__
#define __GR55xx_HAL_CGC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_cgc.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_CGC_ENUMERATIONS Enumerations
  * @{
  */

/**
  * @brief  CGC Bit Open and Bit Close Enumerations
  */
typedef enum
{
    CGC_CLK_ON  = 0U,    /**< Turn on the clock.*/
    CGC_CLK_OFF = 1U,    /**< Turn off the clock.*/
} cgc_clk_state_t;

/** @} */

/** @addtogroup HAL_CGC_STRUCTURES Structures
  * @{
  */

/**
  * @brief   CGC init structure definition
  */
typedef struct _cgc_init
{
    uint32_t wfi_clk;       /**< Specifies the blocks that automatically closes the clock.
                                 This parameter can be a combination of CGC_LL_EC_WFI_CLK0 */

    uint32_t force_clk;     /**< Specifies the blocks to forcibly turn off the clock.
                                 This parameter can be a combination of CGC_LL_EC_FRC_CLK0 */
} cgc_init_t;

/** @} */


/**
  * @defgroup  HAL_CGC_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup CGC_Exported_Constants CGC Exported Constants
  * @{
  */

/** @defgroup CGC_auto_clk Automatic Turn off clocks
  * @{
  */
#define CGC_WFI_SECU_HCLK                    ((uint32_t)0x00000001U)     /**< Hclk for all security blocks */
#define CGC_WFI_SIM_HCLK                     ((uint32_t)0x00000002U)     /**< Hclk for sim card interface  */
#define CGC_WFI_HTB_HCLK                     ((uint32_t)0x00000004U)     /**< Hclk for hopping table       */
#define CGC_WFI_PWM_HCLK                     ((uint32_t)0x00000008U)     /**< Hclk for PWM                 */
#define CGC_WFI_ROM_HCLK                     ((uint32_t)0x00000010U)     /**< Hclk for ROM                 */
#define CGC_WFI_SNSADC_HCLK                  ((uint32_t)0x00000020U)     /**< Hclk for sense ADC           */
#define CGC_WFI_GPIO_HCLK                    ((uint32_t)0x00000040U)     /**< Hclk for GPIOs               */
#define CGC_WFI_DMA_HCLK                     ((uint32_t)0x00000080U)     /**< Hclk for DMA engine          */
#define CGC_WFI_BLE_BRG_HCLK                 ((uint32_t)0x00000100U)     /**< Hclk for BLE MCU bridge      */
#define CGC_WFI_APB_SUB_HCLK                 ((uint32_t)0x00000200U)     /**< Hclk for APB subsystem       */
#define CGC_WFI_SERIAL_HCLK                  ((uint32_t)0x00000400U)     /**< Hclk for serial blocks       */
#define CGC_WFI_I2S_S_HCLK                   ((uint32_t)0x00000800U)     /**< Hclk for I2S slave           */
#define CGC_WFI_AON_MCUSUB_HCLK              ((uint32_t)0x00001000U)     /**< Hclk for Always-on register  */
#define CGC_WFI_XF_XQSPI_HCLK                ((uint32_t)0x00002000U)     /**< Hclk for cache top           */
#define CGC_WFI_SRAM_HCLK                    ((uint32_t)0x00004000U)     /**< Hclk for SRAMs               */
#define CGC_WFI_SECU_DIV4_PCLK               ((uint32_t)0x00008000U)     /**< Div4 clk for security blocks */
#define CGC_WFI_XQSPI_DIV4_PCLK              ((uint32_t)0x00020000U)     /**< Div4 clk for xf qspi         */

#define CGC_WFI_ALL_CLK                      ((uint32_t)0x0002FFFFU)     /**< All clocks                   */
/** @} */

/** @defgroup CGC_force_clk Force cloks off
  * @{
  */
#define CGC_FRC_SECU_HCLK                    ((uint32_t)0x00000001U)     /**< Hclk for all security blocks */
#define CGC_FRC_SIM_HCLK                     ((uint32_t)0x00000002U)     /**< Hclk for sim card interface  */
#define CGC_FRC_HTB_HCLK                     ((uint32_t)0x00000004U)     /**< Hclk for hopping table       */
#define CGC_FRC_PWM_HCLK                     ((uint32_t)0x00000008U)     /**< Hclk for PWM                 */
#define CGC_FRC_ROM_HCLK                     ((uint32_t)0x00000010U)     /**< Hclk for ROM                 */
#define CGC_FRC_SNSADC_HCLK                  ((uint32_t)0x00000020U)     /**< Hclk for sense ADC           */
#define CGC_FRC_GPIO_HCLK                    ((uint32_t)0x00000040U)     /**< Hclk for GPIOs               */
#define CGC_FRC_DMA_HCLK                     ((uint32_t)0x00000080U)     /**< Hclk for DMA engine          */
#define CGC_FRC_BLE_BRG_HCLK                 ((uint32_t)0x00000100U)     /**< Hclk for BLE MCU bridge      */
#define CGC_FRC_APB_SUB_HCLK                 ((uint32_t)0x00000200U)     /**< Hclk for APB subsystem       */
#define CGC_FRC_SERIAL_HCLK                  ((uint32_t)0x00000400U)     /**< Hclk for serial blocks       */
#define CGC_FRC_I2S_S_HCLK                   ((uint32_t)0x00000800U)     /**< Hclk for I2S slave           */
#define CGC_FRC_AON_MCUSUB_HCLK              ((uint32_t)0x00001000U)     /**< Hclk for Always-on register  */
#define CGC_FRC_XF_XQSPI_HCLK                ((uint32_t)0x00002000U)     /**< Hclk for cache top           */
#define CGC_FRC_SRAM_HCLK                    ((uint32_t)0x00004000U)     /**< Hclk for SRAMs               */
#define CGC_FRC_UART0_HCLK                   ((uint32_t)0x00008000U)     /**< Hclk for uart0               */
#define CGC_FRC_UART1_HCLK                   ((uint32_t)0x00010000U)     /**< Hclk for uart1               */
#define CGC_FRC_I2C0_HCLK                    ((uint32_t)0x00020000U)     /**< Hclk for i2c0                */
#define CGC_FRC_I2C1_HCLK                    ((uint32_t)0x00040000U)     /**< Hclk for i2c1                */
#define CGC_FRC_SPIM_HCLK                    ((uint32_t)0x00080000U)     /**< Hclk for spim                */
#define CGC_FRC_SPIS_HCLK                    ((uint32_t)0x00100000U)     /**< Hclk for spis                */
#define CGC_FRC_QSPI0_HCLK                   ((uint32_t)0x00200000U)     /**< Hclk for qspi0               */
#define CGC_FRC_QSPI1_HCLK                   ((uint32_t)0x00400000U)     /**< Hclk for qspi1               */
#define CGC_FRC_I2S_HCLK                     ((uint32_t)0x00800000U)     /**< Hclk for i2s                 */
#define CGC_FRC_SECU_DIV4_PCLK               ((uint32_t)0x01000000U)     /**< Div4 clk for security blocks */
#define CGC_FRC_XQSPI_DIV4_PCLK              ((uint32_t)0x04000000U)     /**< Div4 clk for xf qspi         */

#define CGC_FRC_ALL_CLK                      ((uint32_t)0x05FFFFFFU)     /**< All clocks                   */
/** @} */


/** @defgroup CGC_default_config init Struct default configuartion
  * @{
  */
#define CGC_DEFAULT_CONFIG                       \
{                                                \
    .wfi_clk    = ~CGC_WFI_ALL_CLK,              \
    .force_clk  = ~CGC_FRC_ALL_CLK,              \
}
/** @} */

/** @} */

/** @} */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_CGC_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup CGC_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the CGC registers according to the specified parameters in the @ref cgc_init_t.
 * @param[in]  p_cgc_init: Pointer to a @ref cgc_init_t structure that contains
 *                         the configuration information for the specified CGC registers.
 ****************************************************************************************
 */
void hal_cgc_init(cgc_init_t *p_cgc_init);

/**
 ****************************************************************************************
 * @brief  De-initialize the CGC registers to their default reset values.
 ****************************************************************************************
 */
void hal_cgc_deinit(void);

/** @} */

/** @addtogroup CGC_Exported_Functions_Group2 Peripheral Control functions.
  *  @brief Clock Gate Open and Close management functions.
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Configure the clock state for a specified block during WFI.
 * @param[in]  blocks: Specifies the peripheral blocks.
 *         This parameter can be a combiantion of the following values:
 *         @arg @ref CGC_WFI_SECU_HCLK
 *         @arg @ref CGC_WFI_SIM_HCLK
 *         @arg @ref CGC_WFI_HTB_HCLK
 *         @arg @ref CGC_WFI_PWM_HCLK
 *         @arg @ref CGC_WFI_ROM_HCLK
 *         @arg @ref CGC_WFI_SNSADC_HCLK
 *         @arg @ref CGC_WFI_GPIO_HCLK
 *         @arg @ref CGC_WFI_DMA_HCLK
 *         @arg @ref CGC_WFI_BLE_BRG_HCLK
 *         @arg @ref CGC_WFI_APB_SUB_HCLK
 *         @arg @ref CGC_WFI_SERIAL_HCLK
 *         @arg @ref CGC_WFI_I2S_S_HCLK
 *         @arg @ref CGC_WFI_AON_MCUSUB_HCLK
 *         @arg @ref CGC_WFI_XF_XQSPI_HCLK
 *         @arg @ref CGC_WFI_SRAM_HCLK
 *         @arg @ref CGC_WFI_SECU_DIV4_PCLK
 *         @arg @ref CGC_WFI_XQSPI_DIV4_PCLK
 * @param[in]  clk_state: Specifies the clock state during WFI.
 *         This parameter can be one of the following values:
 *         @arg @ref CGC_CLK_ON
 *         @arg @ref CGC_CLK_OFF
 ****************************************************************************************
 */
void hal_cgc_config_wfi_clk(uint32_t blocks, cgc_clk_state_t clk_state);

/**
 ****************************************************************************************
 * @brief  Get the clock state for a specified block during WFI.
 * @param[in]  block: Specifies the peripheral blocks.
 *         This parameter can be one of the following values:
 *         @arg @ref CGC_WFI_SECU_HCLK
 *         @arg @ref CGC_WFI_SIM_HCLK
 *         @arg @ref CGC_WFI_HTB_HCLK
 *         @arg @ref CGC_WFI_PWM_HCLK
 *         @arg @ref CGC_WFI_ROM_HCLK
 *         @arg @ref CGC_WFI_SNSADC_HCLK
 *         @arg @ref CGC_WFI_GPIO_HCLK
 *         @arg @ref CGC_WFI_DMA_HCLK
 *         @arg @ref CGC_WFI_BLE_BRG_HCLK
 *         @arg @ref CGC_WFI_APB_SUB_HCLK
 *         @arg @ref CGC_WFI_SERIAL_HCLK
 *         @arg @ref CGC_WFI_I2S_S_HCLK
 *         @arg @ref CGC_WFI_AON_MCUSUB_HCLK
 *         @arg @ref CGC_WFI_XF_XQSPI_HCLK
 *         @arg @ref CGC_WFI_SRAM_HCLK
 *         @arg @ref CGC_WFI_SECU_DIV4_PCLK
 *         @arg @ref CGC_WFI_XQSPI_DIV4_PCLK
 * @retval ::CGC_CLK_ON: Clock On.
 * @retval ::CGC_CLK_OFF: Clock Off.
 ****************************************************************************************
 */
cgc_clk_state_t hal_cgc_get_wfi_clk(uint32_t block);

/**
 ****************************************************************************************
 * @brief  Forced to Configure the clock state for a specified block.
 * @param[in]  blocks: Specifies the peripheral blocks.
 *         This parameter can be a combiantion of the following values:
 *         @arg @ref CGC_FRC_SECU_HCLK
 *         @arg @ref CGC_FRC_SIM_HCLK
 *         @arg @ref CGC_FRC_HTB_HCLK
 *         @arg @ref CGC_FRC_PWM_HCLK
 *         @arg @ref CGC_FRC_ROM_HCLK
 *         @arg @ref CGC_FRC_SNSADC_HCLK
 *         @arg @ref CGC_FRC_GPIO_HCLK
 *         @arg @ref CGC_FRC_DMA_HCLK
 *         @arg @ref CGC_FRC_BLE_BRG_HCLK
 *         @arg @ref CGC_FRC_APB_SUB_HCLK
 *         @arg @ref CGC_FRC_SERIAL_HCLK
 *         @arg @ref CGC_FRC_I2S_S_HCLK
 *         @arg @ref CGC_FRC_AON_MCUSUB_HCLK
 *         @arg @ref CGC_FRC_XF_XQSPI_HCLK
 *         @arg @ref CGC_FRC_SRAM_HCLK
 *         @arg @ref CGC_FRC_UART0_HCLK
 *         @arg @ref CGC_FRC_UART1_HCLK
 *         @arg @ref CGC_FRC_I2C0_HCLK
 *         @arg @ref CGC_FRC_I2C1_HCLK
 *         @arg @ref CGC_FRC_SPIM_HCLK
 *         @arg @ref CGC_FRC_SPIS_HCLK
 *         @arg @ref CGC_FRC_QSPI0_HCLK
 *         @arg @ref CGC_FRC_QSPI1_HCLK
 *         @arg @ref CGC_FRC_I2S_HCLK
 *         @arg @ref CGC_FRC_SECU_DIV4_PCLK
 *         @arg @ref CGC_FRC_XQSPI_DIV4_PCLK
 * @param[in]  clk_state: Specifies the clock state.
 *         This parameter can be one of the following values:
 *         @arg @ref CGC_CLK_ON
 *         @arg @ref CGC_CLK_OFF
 ****************************************************************************************
 */
void hal_cgc_config_force_clk(uint32_t blocks, cgc_clk_state_t clk_state);

/**
 ****************************************************************************************
 * @brief  Get the clock status of the currently specified block.
 * @param[in]  block: Specifies the peripheral blocks.
 *         This parameter can be one of the following values:
 *         @arg @ref CGC_FRC_SECU_HCLK
 *         @arg @ref CGC_FRC_SIM_HCLK
 *         @arg @ref CGC_FRC_HTB_HCLK
 *         @arg @ref CGC_FRC_PWM_HCLK
 *         @arg @ref CGC_FRC_ROM_HCLK
 *         @arg @ref CGC_FRC_SNSADC_HCLK
 *         @arg @ref CGC_FRC_GPIO_HCLK
 *         @arg @ref CGC_FRC_DMA_HCLK
 *         @arg @ref CGC_FRC_BLE_BRG_HCLK
 *         @arg @ref CGC_FRC_APB_SUB_HCLK
 *         @arg @ref CGC_FRC_SERIAL_HCLK
 *         @arg @ref CGC_FRC_I2S_S_HCLK
 *         @arg @ref CGC_FRC_AON_MCUSUB_HCLK
 *         @arg @ref CGC_FRC_XF_XQSPI_HCLK
 *         @arg @ref CGC_FRC_SRAM_HCLK
 *         @arg @ref CGC_FRC_UART0_HCLK
 *         @arg @ref CGC_FRC_UART1_HCLK
 *         @arg @ref CGC_FRC_I2C0_HCLK
 *         @arg @ref CGC_FRC_I2C1_HCLK
 *         @arg @ref CGC_FRC_SPIM_HCLK
 *         @arg @ref CGC_FRC_SPIS_HCLK
 *         @arg @ref CGC_FRC_QSPI0_HCLK
 *         @arg @ref CGC_FRC_QSPI1_HCLK
 *         @arg @ref CGC_FRC_I2S_HCLK
 *         @arg @ref CGC_FRC_SECU_DIV4_PCLK
 *         @arg @ref CGC_FRC_XQSPI_DIV4_PCLK
 * @retval ::CGC_CLK_ON: Clock On.
 * @retval ::CGC_CLK_OFF: Clock Off.
 ****************************************************************************************
 */
cgc_clk_state_t hal_cgc_get_force_clk(uint32_t block);


/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_CGC_H__ */

/** @} */

/** @} */

/** @} */

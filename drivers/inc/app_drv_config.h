/**
 ****************************************************************************************
 *
 * @file    app_drv_config.h
 * @author  BLE Driver Team
 * @brief   Header file of app driver config code.
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

/** @defgroup APP_DRIVER_CONFIG DRIVER CONFIG
  * @brief APP DRIVER CONFIG
  * @{
  */


#ifndef _APP_DRV_CONFIG_H_
#define _APP_DRV_CONFIG_H_

#include "custom_config.h"

#ifdef __cplusplus
extern "C" {
#endif
#define APP_DVR_LOG_LVL_NONE        (0)                   /**< None log level define */
#define APP_DVR_LOG_LVL_ERR         (1)                   /**< Error log level define */
#define APP_DVR_LOG_LVL_WARN        (2)                   /**< Warning log level define */
#define APP_DVR_LOG_LVL_INFO        (3)                   /**< Info log level define */

#ifndef APP_DRV_LOG_LEVEL
#define APP_DRV_LOG_LEVEL           APP_DVR_LOG_LVL_NONE  /**< App driver log level setting */
#endif
#ifndef APP_DRV_LOG_INTERFACE
#define APP_DRV_LOG_INTERFACE       printf                /**< App driver log interface setting */
#endif
#ifndef APP_DRV_ASSERT_ENABLE
#define APP_DRV_ASSERT_ENABLE       0                     /**< App driver assert enable */
#endif

#define APP_DRIVER_GR551X           0x0              /**< APP_DRIVER for GR551X */
#define APP_DRIVER_GR5525X          0x1              /**< APP_DRIVER for GR5525X */
#define APP_DRIVER_GR5526X          0x2              /**< APP_DRIVER for GR5526X */
#define APP_DRIVER_GR5332X          0x3              /**< APP_DRIVER for GR5332X */

#ifdef SOC_GR5515
#define APP_DRIVER_CHIP_TYPE  APP_DRIVER_GR551X      /**< GR5515 chip type*/
#define SOC_GPIO_PINS_MAX     (32)                   /**< GR5515 max gpio pins */
#define SOC_AON_PINS_MAX      (8)                    /**< GR5515 max aon pins */
#elif defined(SOC_GR5525)
#define APP_DRIVER_CHIP_TYPE  APP_DRIVER_GR5525X     /**< GR5525 chip type*/
#define SOC_GPIO_PINS_MAX     (32)                   /**< GR5525 max gpio pins */
#define SOC_AON_PINS_MAX      (8)                    /**< GR5525 max aon pins */
#elif defined(SOC_GR5526)
#define APP_DRIVER_CHIP_TYPE  APP_DRIVER_GR5526X     /**< GR5526 chip type*/
#define SOC_GPIO_PINS_MAX     (34)                   /**< GR5526 max gpio pins */
#define SOC_AON_PINS_MAX      (8)                    /**< GR5526 max aon pins */
#elif defined(SOC_GR5332)
#define APP_DRIVER_CHIP_TYPE  APP_DRIVER_GR5332X     /**< GR5332 chip type*/
#define SOC_GPIO_PINS_MAX     (14)                   /**< GR5332 max gpio pins */
#define SOC_AON_PINS_MAX      (8)                    /**< GR5332 max aon pins */
#endif

/**
 * @defgroup APP_DRV_PERIPHERAL_PRIORITY_DEFINE Defines
 * @{
 */
/**@brief APP driver peripheral priority define. */
#ifndef APP_DRIVER_ADC_WAKEUP_PRIORITY
#define APP_DRIVER_ADC_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH  /**< ADC Wakeup priority High */
#endif

#ifndef APP_DRIVER_AES_WAKEUP_PRIORITY
#define APP_DRIVER_AES_WAKEUP_PRIORITY              WAKEUP_PRIORITY_MID   /**< AES Wakeup priority Mid */
#endif

#ifndef APP_DRIVER_COMP_WAKEUP_PRIORITY
#define APP_DRIVER_COMP_WAKEUP_PRIORITY             WAKEUP_PRIORITY_LOW   /**< COMP Wakeup priority Low */
#endif

#ifndef APP_DRIVER_DUAL_TIM_WAKEUP_PRIORITY
#define APP_DRIVER_DUAL_TIM_WAKEUP_PRIORITY         WAKEUP_PRIORITY_MID   /**< DUAL TIM Wakeup priority Mid */
#endif

#ifndef APP_DRIVER_DMA_WAKEUP_PRIORITY
#define APP_DRIVER_DMA_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH  /**< DMA Wakeup priority High */
#endif

#ifndef APP_DRIVER_UART_WAKEUP_PRIORITY
#define APP_DRIVER_UART_WAKEUP_PRIORITY             WAKEUP_PRIORITY_HIGH  /**< Uart Wakeup priority High */
#endif

#ifndef APP_DRIVER_HMAC_WAKEUP_PRIORITY
#define APP_DRIVER_HMAC_WAKEUP_PRIORITY             WAKEUP_PRIORITY_MID   /**< Hmac Wakeup priority Mid */
#endif

#ifndef APP_DRIVER_I2C_WAKEUP_PRIORITY
#define APP_DRIVER_I2C_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH  /**< I2C Wakeup priority High */
#endif

#ifndef APP_DRIVER_I2S_WAKEUP_PRIORITY
#define APP_DRIVER_I2S_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH  /**< I2S Wakeup priority High */
#endif

#ifndef APP_DRIVER_QSPI_WAKEUP_PRIORITY
#define APP_DRIVER_QSPI_WAKEUP_PRIORITY             WAKEUP_PRIORITY_HIGH  /**< QSPI Wakeup priority High */
#endif

#ifndef APP_DRIVER_RNG_WAKEUP_PRIORITY
#define APP_DRIVER_RNG_WAKEUP_PRIORITY              WAKEUP_PRIORITY_MID   /**< RNG Wakeup priority Mid */
#endif

#ifndef APP_DRIVER_SPI_WAKEUP_PRIORITY
#define APP_DRIVER_SPI_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH  /**< SPI Wakeup priority High */
#endif

#ifndef APP_DRIVER_TIM_WAKEUP_PRIORITY
#define APP_DRIVER_TIM_WAKEUP_PRIORITY              WAKEUP_PRIORITY_MID   /**< TIM Wakeup priority Mid */
#endif

#ifndef APP_DRIVER_PWM_WAKEUP_PRIORITY
#define APP_DRIVER_PWM_WAKEUP_PRIORITY              WAKEUP_PRIORITY_MID   /**< PWM Wakeup priority Mid */
#endif

#ifndef APP_DRIVER_ISO7816_WAKEUP_PRIORITY
#define APP_DRIVER_ISO7816_WAKEUP_PRIORITY          WAKEUP_PRIORITY_HIGH    /**< ISO7816 Wakeup priority High */
#endif

#ifndef APP_DRIVER_PKC_WAKEUP_PRIORITY
#define APP_DRIVER_PKC_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH    /**< PKC Wakeup priority High */
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X || APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
#ifndef APP_DRIVER_DSPI_WAKEUP_PRIORITY
#define APP_DRIVER_DSPI_WAKEUP_PRIORITY             WAKEUP_PRIORITY_HIGH    /**< DSPI Wakeup priority High */
#endif

#ifndef APP_DRIVER_PDM_WAKEUP_PRIORITY
#define APP_DRIVER_PDM_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH    /**< PDM Wakeup priority High */
#endif
#endif

/**@} */


/**@addtogroup APP_DRV_WAKEUP_PRIORITY_ENUM Enumerations
 * @{
 */
/**@brief APP driver peripheral wakeup priority define. */
typedef enum
{
    WAKEUP_PRIORITY_LOW = 1,          /**< Wakeup priority low */
    WAKEUP_PRIORITY_MID,              /**< Wakeup priority mid */
    WAKEUP_PRIORITY_HIGH              /**< Wakeup priority high */
} wakeup_priority_t;
/** @} */

#ifndef APP_DRIVER_WAKEUP_CALL_FUN
//#define APP_DRIVER_WAKEUP_CALL_FUN
#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */


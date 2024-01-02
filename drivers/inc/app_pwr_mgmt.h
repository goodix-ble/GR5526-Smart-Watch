/**
 ****************************************************************************************
 *
 * @file    app_pwr_mgmt.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of UART PWR library.
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

/** @defgroup APP_PWR_MGMT PWR MGMT
  * @brief PWR MGMT APP module driver.
  * @{
  */


#ifndef _APP_PWR_MGMT_H_
#define _APP_PWR_MGMT_H_

#include "grx_sys.h"
#include "app_drv_config.h"
#include <stdint.h>
#include <stdbool.h>

/** @addtogroup APP_PWR_DEFINE Defines
  * @{
  */
/**
  * @brief PWR MAX value for sleep check
  */
#define APP_SLEEP_CB_MAX     PWR_ID_MAX
/** @} */

/** @addtogroup APP_PWR_ENUMERATIONS Enumerations
  * @{
  */
/**
  * @brief PWR ID Enumerations definition
  */
typedef enum
{
    DMA_PWR_ID,
    UART_PWR_ID,
    I2C_PWR_ID,
    SPI_PWR_ID,
    PWM_PWR_ID,
    TIM_PWR_ID,
    DUAL_TIM_PWR_ID,
    ADC_PWR_ID,
    COMP_PWR_ID,
    RNG_PWR_ID,
#if defined SOC_GR5515 || defined SOC_GR5525 || defined SOC_GR5526
    QSPI_PWR_ID,
    I2S_PWR_ID,
    PKC_PWR_ID,
    HMAC_PWR_ID,
    AES_PWR_ID,
#endif
#if defined SOC_GR5515 || defined SOC_GR5526
    ISO7816_PWR_ID,
#endif
#if defined SOC_GR5525 || defined SOC_GR5526
    DSPI_PWR_ID,
    PDM_PWR_ID,
#endif
    PWR_ID_MAX,
} pwr_id_t;
/** @} */

/** @addtogroup APP_PWR_STRUCTURES Structures
  * @{
  */
/**
  * @brief PWR sleep check function Structure
  */
typedef struct
{
    bool (*app_prepare_for_sleep)(void);    /**< Peripherals prepare sleep fuction. */
    void (*app_wake_up_ind)(void);          /**< Resume peripherals when chip wakeup. */
} app_sleep_callbacks_t;

/** @} */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_PWR_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the app power management module
 ****************************************************************************************
 */
void app_pwr_mgmt_init(void);

/**
 ****************************************************************************************
 * @brief    set PWR sleep callback function
 * @param    p_cb : Device check callback function
 * @param    wakeup_priority : Device wakeup priority
 * @param    id : which id want to register
 *
 * @return   ID
 ****************************************************************************************
 */
pwr_id_t pwr_register_sleep_cb(const app_sleep_callbacks_t *p_cb, wakeup_priority_t wakeup_priority, pwr_id_t id);

/**
 ****************************************************************************************
 * @brief    Unregister PWR sleep callback function
 * @param    id : which id want to unregister
 ****************************************************************************************
 */
void pwr_unregister_sleep_cb(pwr_id_t id);

/**
 ****************************************************************************************
 * @brief    Things to do after waking up.
 ****************************************************************************************
 */
void pwr_wake_up_ind(void);

/**
 ****************************************************************************************
 * @brief    Check peripheral status before going to sleep.
 ****************************************************************************************
 */
pwr_mgmt_dev_state_t pwr_enter_sleep_check(void);
/** @} */

#endif

/** @} */
/** @} */
/** @} */

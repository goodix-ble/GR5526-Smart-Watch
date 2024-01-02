/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_pwr_mgmt.h
 * @author  BLE Driver Team
 * @brief   This file contains all the functions prototypes for the HAL
 *          module driver.
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

/** @defgroup HAL_PWR_MGMT PWR_MGMT
  * @brief PWR_MGMT HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_PWR_MGMT_H__
#define __GR55xx_HAL_PWR_MGMT_H__

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"
#include <stdio.h>
#include <stdbool.h>

/**
  * @defgroup  HAL_PWR_MGMT_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PWR_MGMT_Exported_Constants PWR_MGMT Exported Constants
  * @{
  */
#define ALL_DEVICES_SLEEP    (0xFFFFFFFFU)      /**< All peripherals sleep    */
#define ALL_DEVICES_BACKUP   (0x00000000U)      /**< All peripherals backed up*/
/** @} */

/** @} */

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_PWR_MGMT_ENUMERATIONS Enumerations
  * @{
  */

/**
  * @brief  Peripheral Device ID definition
  * NOTE:The order of enumeration is the order of recovery
  */
typedef enum
{
    PERIPH_DEVICE_NUM_DMA0 = 0,
    PERIPH_DEVICE_NUM_DMA1,     //NO.=1
    PERIPH_DEVICE_NUM_ISO7816,  //NO.=2
    PERIPH_DEVICE_NUM_PKC,      //NO.=3
    PERIPH_DEVICE_NUM_QSPI0,    //NO.=4
    PERIPH_DEVICE_NUM_QSPI1,    //NO.=5
    PERIPH_DEVICE_NUM_QSPI2,    //NO.=6
    PERIPH_DEVICE_NUM_SPIM,     //NO.=7
    PERIPH_DEVICE_NUM_SPIS,     //NO.=8
    PERIPH_DEVICE_NUM_DSPI,     //NO.=9
    PERIPH_DEVICE_NUM_I2C0,     //NO.=10
    PERIPH_DEVICE_NUM_I2C1,     //NO.=11
    PERIPH_DEVICE_NUM_I2C2,     //NO.=12
    PERIPH_DEVICE_NUM_I2C3,     //NO.=13
    PERIPH_DEVICE_NUM_I2C4,     //NO.=14
    PERIPH_DEVICE_NUM_I2C5,     //NO.=15
    PERIPH_DEVICE_NUM_UART0,    //NO.=16
    PERIPH_DEVICE_NUM_UART1,    //NO.=17
    PERIPH_DEVICE_NUM_UART2,    //NO.=18
    PERIPH_DEVICE_NUM_UART3,    //NO.=19
    PERIPH_DEVICE_NUM_UART4,    //NO.=20
    PERIPH_DEVICE_NUM_UART5,    //NO.=21
    PERIPH_DEVICE_NUM_I2S_M,    //NO.=22
    PERIPH_DEVICE_NUM_I2S_S,    //NO.=23
    PERIPH_DEVICE_NUM_PDM,      //NO.=24
    PERIPH_DEVICE_NUM_HMAC,     //NO.=25
    PERIPH_DEVICE_NUM_RNG,      //NO.=26
    PERIPH_DEVICE_NUM_AES,      //NO.=27
    PERIPH_DEVICE_NUM_PWM0,     //NO.=28
    PERIPH_DEVICE_NUM_PWM1,     //NO.=29
    PERIPH_DEVICE_NUM_SNSADC,   //NO.=30
    PERIPH_DEVICE_NUM_USB,      //NO.=31
    MAX_PERIPH_DEVICE_NUM
} periph_device_number_t;

/**
  * @brief  Extra Device ID definition
  */
typedef enum
{
    EXTRA_DEVICE_NUM_CLK_CALIB = 0,
    EXTRA_DEVICE_NUM_GPU,          //NO.=1
    EXTRA_DEVICE_NUM_DC,           //NO.=2
    EXTRA_DEVICE_NUM_OSPI,         //NO.=3
    MAX_EXTRA_DEVICE_NUM
} extra_device_number_t;

/**
  * @brief  Peripheral Device State definition
  */
typedef enum
{
    IDLE = 0,
    ACTIVE
} periph_state_t;
/** @} */

/** @addtogroup HAL_PWR_MGMT_TYPEDEFS Type defintions
  * @{
  */

/**
  * @brief Peripheral suspend callback definition
  */
typedef hal_status_t (*p_device_suspend_func)(void *p_dev_handle);

/**
  * @brief Peripheral resume callback definition
  */
typedef hal_status_t (*p_device_resume_func)(void *p_dev_handle);

/** @} */


/* Exported variable --------------------------------------------------------*/
/** @addtogroup HAL_PWR_MGMT_VARIABLE Variables
  * @{
  */

extern volatile uint32_t g_devices_state;  /**< All peripheral status.When all are idle, the system can sleep */
extern volatile uint32_t g_devices_renew;  /**< Peripheral updata flag.If peripherals have been used, they need to be backed up */
extern volatile uint32_t g_devices_sleep;  /**< Peripheral sleep flag.Go to sleep as 0xFFFFFFFF.The corresponding bit is cleared to 0 after wake-up recovery */
extern p_device_suspend_func devices_suspend_cb[MAX_PERIPH_DEVICE_NUM];  /**< Store the suspend function of all peripherals */
extern p_device_resume_func devices_resume_cb[MAX_PERIPH_DEVICE_NUM];    /**< Store the resume function of all peripherals */
extern void* devices_handle[MAX_PERIPH_DEVICE_NUM];                      /**<Store the handle addresses of all peripheral instances */

extern volatile uint32_t g_extra_devices_state;  /**< All extra devices status.When all are idle, the system can sleep */
extern volatile uint32_t g_extra_devices_renew;  /**< Extra devices updata flag.If extra devices have been used, they need to be backed up */
extern p_device_suspend_func extra_devices_suspend_cb[MAX_EXTRA_DEVICE_NUM];  /**< Store the suspend function of all extra devices */
extern p_device_resume_func extra_devices_resume_cb[MAX_EXTRA_DEVICE_NUM];    /**< Store the resume function of all extra devices */
extern void* extra_devices_handle[MAX_EXTRA_DEVICE_NUM];                      /**<Store the handle addresses of all extra devices instances */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_PWR_MGMT_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Set peripheral suspend function
 *
 * @param[in]  dev_num: Peripheral device ID definition.
 * @param[in]  my_device_suspend_function: Peripheral suspend callback function.

 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_device_suspend_register(periph_device_number_t dev_num, p_device_suspend_func my_device_suspend_function);

/**
 ****************************************************************************************
 * @brief  Release peripheral suspend function
 *
 * @param[in]  dev_num: Peripheral device ID definition.

 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_device_suspend_release(periph_device_number_t dev_num);

/**
 ****************************************************************************************
 * @brief  Set peripheral resume function
 *
 * @param[in]  dev_num: Peripheral device ID definition.
 * @param[in]  my_device_resume_function: Peripheral resume callback function.

 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_device_resume_register(periph_device_number_t dev_num, p_device_resume_func my_device_resume_function);

/**
 ****************************************************************************************
 * @brief  Release peripheral resume function
 *
 * @param[in]  dev_num: Peripheral device ID definition.

 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_device_resume_release(periph_device_number_t dev_num);

/**
 ****************************************************************************************
 * @brief  Store the specified peripheral handle
 *
 * @param[in]  dev_num: Peripheral device ID definition.
 * @param[in]  p_dev: Pointer to the specified peripheral handle.

 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_save_device_handle(periph_device_number_t dev_num,void* p_dev);

/**
 ****************************************************************************************
 * @brief  Clear the specified peripheral handle
 *
 * @param[in]  dev_num: Peripheral device ID definition.

 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_clear_device_handle(periph_device_number_t dev_num);

/**
 ****************************************************************************************
 * @brief  Store the specified peripheral state
 *
 * @param[in]  dev_num: Peripheral device ID definition.
 * @param[in]  state: the specified peripheral state.

 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_set_device_state(periph_device_number_t dev_num, periph_state_t state);

/**
 ****************************************************************************************
 * @brief  Store the specified peripheral backup flag
 *
 * @param[in]  dev_num: Peripheral device ID definition.

 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_set_device_renew_flag(periph_device_number_t dev_num);

/**
 ****************************************************************************************
 * @brief  Store the specified peripheral sleep flag
 *
 * @param[in]  dev_num: Peripheral device ID definition.

 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_clear_device_sleep_flag(periph_device_number_t dev_num);

/**
 ****************************************************************************************
 * @brief  Set extra device suspend function
 *
 * @param[in]  dev_num: extra device ID definition.
 * @param[in]  my_device_suspend_function: extra suspend callback function.Set NULL to release
 * @param[in]  p_dev_handle: Pointer to the specified extra device handle.
 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_extra_device_suspend_register(extra_device_number_t dev_num, p_device_suspend_func my_device_suspend_function, void* p_dev_handle);

/**
 ****************************************************************************************
 * @brief  Set extra resume function
 *
 * @param[in]  dev_num: extra device ID definition.
 * @param[in]  my_device_resume_function: extra resume callback function.Set NULL to release
 * @param[in]  p_dev_handle: Pointer to the specified extra device handle.

 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_extra_device_resume_register(extra_device_number_t dev_num, p_device_resume_func my_device_resume_function, void* p_dev_handle);

/**
 ****************************************************************************************
 * @brief  set the specified extra device state
 *
 * @param[in]  dev_num: extra device ID definition.
 * @param[in]  state: the specified extra device state.

 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_set_extra_device_state(extra_device_number_t dev_num, periph_state_t state);

/**
 ****************************************************************************************
 * @brief  set the specified extra device updata flag
 *
 * @param[in]  dev_num: extra device ID definition.

 * @retval  void.
 ****************************************************************************************
 */
void hal_pwr_mgmt_set_extra_device_renew_flag(extra_device_number_t dev_num);

/** @} */

#endif /*__GR55xx_HAL_PWR_MGMT_H__*/

/** @} */

/** @} */

/** @} */

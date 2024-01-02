/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_br.h
 * @author  BLE Driver Team
 * @brief   This file contains all the functions prototypes for the HAL
 *          Bridge module driver.
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

/** @defgroup HAL_BR HAL_BR
  * @brief HAL Bridge module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_BR_H__
#define __GR55xx_HAL_BR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

/** @addtogroup HAL_HAL_BR_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_HAL_BR_Callback Callback
  * @{
  */

/**
  * @brief HAL_HAL Callback function definition
  */

typedef struct _hal_callback
{
    void (*msp_init)(void);          /**< HAL init MSP callback                  */
    void (*msp_deinit)(void);        /**< HAL de-init MSP callback               */
    void (*systick_callback)(void);  /**< HAL systick callback                   */
} hal_callback_t;

/** @} */

/** @} */

/** @addtogroup HAL_HAL_BR_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup HAL_BR_Exported_Functions_Group1 Initialization, De-initialization \
  *             and Callback registration Functions
  *  @brief    Initialization , de-initialization and Callback registration Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  This function configures time base source, NVIC and Low level hardware.
 *
 * @note   This function is called at the beginning of program after reset and before
 *         the clock configuration.
 *         The SysTick configuration is based on AHB clock.
 *         When the time base configuration is done, time base tick starts incrementing.
 *         In the default implementation, SysTick is used as source of time base.
 *         The tick variable is incremented each 1ms in its ISR.
 *         The function will call the hal_init function to initialize the HAL.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_init_ext(void);

/**
 ****************************************************************************************
 * @brief  This function de-initializes common part of the HAL and stops the source
 *         of time base.
 *
 * @note   This function is optional.
 *         The function will call the hal_deinit function to De-initialize the HAL.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_deinit_ext(void);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note   This function needs to be called before hal_init.
 *
 * @param[in]  hal_callback: Pointer to callback structure function. @ref hal_callback_t
 ****************************************************************************************
 */
void hal_register_callback(hal_callback_t *hal_callback);

#ifdef HAL_ADC_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the ADC according to the specified parameters
 *         in the adc_init_t and initialize the associated handle.
 *
 * @param[in]  p_adc: Pointer to an ADC handle which contains the configuration information for
 *                    the specified ADC module.
 *
 * @note The function will call the hal_adc_init function to initialize the HAL ADC.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_adc_init_ext(adc_handle_t *p_adc);

/**
 ****************************************************************************************
 * @brief  De-initialize the ADC peripheral.
 *
 * @param[in]  p_adc: Pointer to an ADC handle which contains the configuration information for
 *                    the specified ADC module.
 *
 * @note The function will call the hal_adc_deinit function to De-initialize the HAL ADC.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_adc_deinit_ext(adc_handle_t *p_adc);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_adc_init.
 *
 * @param[in]  adc_callback: Pointer to callback structure function. @ref adc_callback_t
 ****************************************************************************************
 */
void hal_adc_register_callback(adc_callback_t *adc_callback);

#endif /* HAL_ADC_MODULE_ENABLED */


#ifdef HAL_AES_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the AES according to the specified parameters
 *         in the aes_init_t and initialize the associated handle.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration
 *                    information for the specified AES module.
 *
 * @note The function will call the hal_aes_init function to initialize the HAL AES.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_init_ext(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  De-initialize the AES peripheral.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration
 *                    information for the specified AES module.
 *
 * @note The function will call the hal_aes_denit function to De-initialize the HAL AES.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_deinit_ext(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_aes_init.
 *
 * @param[in]  aes_callback: Pointer to callback structure function. @ref aes_callback_t
 ****************************************************************************************
 */
void hal_aes_register_callback(aes_callback_t *aes_callback);

#endif /* HAL_AES_MODULE_ENABLED */


#ifdef HAL_AON_GPIO_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the AON_GPIOx peripheral according to the specified parameters in the @ref aon_gpio_init_t.
 *
 * @note The function will call the hal_aon_gpio_init function to initialize the HAL AON GPIO.
 *
 * @param[in]  p_aon_gpio_init: Pointer to an @ref aon_gpio_init_t structure that contains
 *                         the configuration information for the specified AON_GPIO peripheral port.
 ****************************************************************************************
 */
void hal_aon_gpio_init_ext(aon_gpio_init_t *p_aon_gpio_init);

/**
 ****************************************************************************************
 * @brief  De-initialize the AON_GPIOx peripheral registers to their default reset values.
 *
 * @note The function will call the hal_aon_gpio_deinit function to De-initialize the HAL AON GPIO.
 *
 * @param[in]  aon_gpio_pin: Specifies the port bit to be written.
 *         This parameter can be a combination of the following values:
 *         @arg @ref AON_GPIO_PIN_0
 *         @arg @ref AON_GPIO_PIN_1
 *         @arg @ref AON_GPIO_PIN_2
 *         @arg @ref AON_GPIO_PIN_3
 *         @arg @ref AON_GPIO_PIN_4
 *         @arg @ref AON_GPIO_PIN_5
 *         @arg @ref AON_GPIO_PIN_ALL
 ****************************************************************************************
 */
void hal_aon_gpio_deinit_ext(uint32_t aon_gpio_pin);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_aon_gpio_init.
 *
 * @param[in]  aon_gpio_callback: Pointer to callback structure function. @ref aon_gpio_callback_t
 ****************************************************************************************
 */
void hal_aon_gpio_register_callback(aon_gpio_callback_t *aon_gpio_callback);

#endif /* HAL_AON_GPIO_MODULE_ENABLED */


#ifdef HAL_AON_WDT_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the AON_WDT according to the specified parameters in the wdt_init_t
 *         of associated handle.
 *
 * @param[in]  p_aon_wdt: Pointer to a AON_WDT handle which contains the configuration
 *                        information for the specified AON_WDT module.
 *
 * @note The function will call the hal_aon_wdt_init function to initialize the HAL AON WDT.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aon_wdt_init_ext(aon_wdt_handle_t *p_aon_wdt);

/**
 ****************************************************************************************
 * @brief De-initialize the AON_WDT peripheral.
 *
 * @param[in]  p_aon_wdt: AON_WDT handle.
 *
 * @note The function will call the hal_aon_wdt_deinit function to De-initialize the HAL AON WDT.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aon_wdt_deinit_ext(aon_wdt_handle_t *p_aon_wdt);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_aon_wdt_init.
 *
 * @param[in]  aon_wdt_callback: Pointer to callback structure function. @ref aon_wdt_callback_t.
 ****************************************************************************************
 */
void hal_aon_wdt_register_callback(aon_wdt_callback_t *aon_wdt_callback);

#endif /* HAL_AON_WDT_MODULE_ENABLED */


#ifdef HAL_CALENDAR_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the CALENDAR according to the specified parameters in the
 *         calendar_init_t of  associated handle.
 *
 * @param[in]  p_calendar: Pointer to a CALENDAR handle which contains the configuration
 *               information for the specified CALENDAR module.
 *
 * @note The function will call the hal_calendar_init function to initialize the HAL CALENDER.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_calendar_init_ext(calendar_handle_t *p_calendar);

/**
 ****************************************************************************************
 * @brief De-initialize the CALENDAR peripheral.
 *
 * @param[in] p_calendar: CALENDAR handle.
 *
 * @note The function will call the hal_calendar_deinit function to De-initialize the HAL CALENDER.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_calendar_deinit_ext(calendar_handle_t *p_calendar);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_calendar_init.
 *
 * @param[in]  calendar_callback: Pointer to callback structure function. @ref calendar_callback_t
 ****************************************************************************************
 */
void hal_calendar_register_callback(calendar_callback_t *calendar_callback);

#endif /* HAL_CALENDAR_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the RTC according to the specified parameters in the
 *         rtc_init_t of  associated handle.counter start after hal_rtc_init.
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @note The function will call the hal_rtc_init function to initialize the HAL RTC.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rtc_init_ext(rtc_handle_t *p_rtc);

/**
 ****************************************************************************************
 * @brief  DeInitialize the RTC according to the specified parameters in the
 *         rtc_init_t of  associated handle.counter stop after hal_rtc_deinit.
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @note The function will call the hal_rtc_deinit function to De-initialize the HAL RTC.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rtc_deinit_ext(rtc_handle_t *p_rtc);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_rtc_init.
 *
 * @param[in]  rtc_callback: Pointer to callback structure function. @ref rtc_callback_t
 ****************************************************************************************
 */
void hal_rtc_register_callback(rtc_callback_t *rtc_callback);

#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_DUAL_TIMER_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the DUAL TIMER according to the specified parameters
 *         in the dual_timer_init_t and initialize the associated handle.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIMER handle which contains the configuration information for the specified DUAL TIMER.
 *
 * @note The function will call the hal_dual_timer_base_init function to initialize the HAL DUAL TIMER.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_timer_base_init_ext(dual_timer_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  De-initialize the DUAL TIMER peripheral.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIMER.
 *
 * @note The function will call the hal_dual_timer_base_deinit function to De-initialize the HAL DUAL TIMER.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_timer_base_deinit_ext(dual_timer_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_dual_timer_base_init.
 *
 * @param[in]  dual_timer_callback: Pointer to callback structure function. @ref dual_timer_callback_t
 ****************************************************************************************
 */
void hal_dual_timer_register_callback(dual_timer_callback_t *dual_timer_callback);

#endif /* HAL_DUAL_TIMER_MODULE_ENABLED */


#ifdef HAL_GPIO_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the GPIOx peripheral according to the specified parameters in the p_gpio_init.
 *
 * @note The function will call the hal_gpio_init function to initialize the HAL GPIO.
 *
 * @param[in]  GPIOx: Where x can be (0, 1) to select the GPIO peripheral port
 * @param[in]  p_gpio_init: Pointer to a gpio_init_t structure that contains the configuration information
 *                          for the specified GPIO peripheral port.
 ****************************************************************************************
 */
void hal_gpio_init_ext(gpio_regs_t *GPIOx, gpio_init_t *p_gpio_init);

/**
 ****************************************************************************************
 * @brief  De-initialize the GPIOx peripheral registers to their default reset values.
 *
 * @note The function will call the hal_gpio_deinit function to De-initialize the HAL GPIO.
 *
 * @param[in]  GPIOx: Where x can be (0, 1) to select the GPIO peripheral port for GR55xx device
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
void hal_gpio_deinit_ext(gpio_regs_t *GPIOx, uint32_t gpio_pin);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_gpio_init.
 *
 * @param[in]  gpio_callback: Pointer to callback structure function. @ref gpio_callback_t
 ****************************************************************************************
 */
void hal_gpio_register_callback(gpio_callback_t *gpio_callback);

#endif /* HAL_GPIO_MODULE_ENABLED */


#ifdef HAL_HMAC_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the HMAC according to the specified parameters
 *         in the hmac_init_t and initialize the associated handle.
 *
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 *
 * @note The function will call the hal_hmac_init function to initialize the HAL HMAC.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_hmac_init_ext(hmac_handle_t *p_hmac);

/**
 ****************************************************************************************
 * @brief  De-initialize the HMAC peripheral.
 *
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 *
 * @note The function will call the hal_hmac_deinit function to De-initialize the HAL HMAC.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_hmac_deinit_ext(hmac_handle_t *p_hmac);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_hmac_init.
 *
 * @param[in]  hmac_callback: Pointer to callback structure function. @ref hmac_callback_t
 ****************************************************************************************
 */
void hal_hmac_register_callback(hmac_callback_t *hmac_callback);

#endif /* HAL_HMAC_MODULE_ENABLED */


#ifdef HAL_I2C_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initializes the I2C according to the specified parameters
 *         in the i2c_init_t and initialize the associated handle.
 *
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration
 *                information for the specified I2C.
 *
 * @note The function will call the hal_i2c_init function to initialize the HAL I2C.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_init_ext(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  De-initialize the I2C peripheral.
 *
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 *
 * @note The function will call the hal_i2c_deinit function to De-initialize the HAL I2C.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_deinit_ext(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_i2c_init.
 *
 * @param[in]  i2c_callback: Pointer to callback structure function. @ref i2c_callback_t
 ****************************************************************************************
 */
void hal_i2c_register_callback(i2c_callback_t *i2c_callback);

#endif /* HAL_I2C_MODULE_ENABLED */


#ifdef HAL_I2S_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the I2S according to the specified parameters
 *         in the i2s_init_t and initialize the associated handle.
 *
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 *
 * @note The function will call the hal_i2s_init function to initialize the HAL I2S.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_init_ext(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  De-initialize the I2S peripheral.
 *
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 *
 * @note The function will call the hal_i2s_deinit function to De-initialize the HAL I2S.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_deinit_ext(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_i2s_init.
 *
 * @param[in]  i2s_callback: Pointer to callback structure function. @ref i2s_callback_t
 ****************************************************************************************
 */
void hal_i2s_register_callback(i2s_callback_t *i2s_callback);

#endif /* HAL_I2S_MODULE_ENABLED */


#ifdef HAL_MSIO_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the MSIOx peripheral according to the specified parameters in the msio_init_t.
 *
 * @note The function will call the hal_msio_init function to initialize the HAL MSIO.
 *
 * @param[in]  MSIOx:       MSIO peripheral port.
 * @param[in]  p_msio_init: Pointer to an @ref msio_init_t structure that contains
 *                          the configuration information for the specified MSIO peripheral port.
 ****************************************************************************************
 */
void hal_msio_init_ext(msio_pad_t MSIOx, msio_init_t *p_msio_init);

/**
 ****************************************************************************************
 * @brief  De-initialize the MSIOx peripheral registers to their default reset values.
 *
 * @note The function will call the hal_msio_deinit function to De-initialize the HAL MSIO.
 *
 * @param[in]  MSIOx:    MSIO peripheral port.
 * @param[in]  msio_pin: Specifies the port bit to be written.
 *         This parameter can be a combination of the following values:
 *         @arg @ref MSIO_PIN_0
 *         @arg @ref MSIO_PIN_1
 *         @arg @ref MSIO_PIN_2
 *         @arg @ref MSIO_PIN_3
 *         @arg @ref MSIO_PIN_4
 *         @arg @ref MSIO_PIN_ALL
 ****************************************************************************************
 */
void hal_msio_deinit_ext(msio_pad_t MSIOx, uint32_t msio_pin);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_msio_init.
 *
 * @param[in]  msio_callback: Pointer to callback structure function. @ref msio_callback_t
 ****************************************************************************************
 */
void hal_msio_register_callback(msio_callback_t *msio_callback);

#endif /* HAL_MSIO_MODULE_ENABLED */


#ifdef HAL_PKC_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the PKC according to the specified parameters
 *         in the pkc_init_t and initialize the associated handle.
 *
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *               information for the specified PKC module.
 *
 * @note The function will call the hal_pkc_init function to initialize the HAL PKC.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_init_ext(pkc_handle_t *p_pkc);

/**
 ****************************************************************************************
 * @brief  De-initialize the PKC peripheral.
 *
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *               information for the specified PKC module.
 *
 * @note The function will call the hal_pkc_deinit function to De-initialize the HAL PKC.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_deinit_ext(pkc_handle_t *p_pkc);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_pkc_init.
 *
 * @param[in]  pkc_callback: Pointer to callback structure function. @ref pkc_callback_t
 ****************************************************************************************
 */
void hal_pkc_register_callback(pkc_callback_t *pkc_callback);

#endif /* HAL_PKC_MODULE_ENABLED */


#ifdef HAL_PWM_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief Initialize the PWM mode according to the specified
 *        parameters in the pwm_init_t and initialize the associated handle.
 *
 * @param[in]  p_pwm: Pointer to a PWM handle that contains the configuration information for the specified PWM module.
 *
 * @note The function will call the hal_pwm_init function to initialize the HAL PWM.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwm_init_ext(pwm_handle_t *p_pwm);

/**
 ****************************************************************************************
 * @brief  De-initialize the PWM peripheral.
 *
 * @param[in]  p_pwm: Pointer to a PWM handle that contains the configuration information for the specified PWM module.
 *
 * @note The function will call the hal_pwm_deinit function to De-initialize the HAL PWM.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwm_deinit_ext(pwm_handle_t *p_pwm);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_pwm_init.
 *
 * @param[in]  pwm_callback: Pointer to callback structure function. @ref pwm_callback_t
 ****************************************************************************************
 */
void hal_pwm_register_callback(pwm_callback_t *pwm_callback);

#endif /* HAL_PWM_MODULE_ENABLED */


#ifdef HAL_PWR_MODULE_ENABLED

#ifdef HAL_SLEEP_TIMER_MODULE_ENABLED

/**
****************************************************************************************
* @brief  Configure the AON Sleep Timer mode, count and start used to wakeup MCU.
*
* @param[in]  mode:  Specifies the sleep timer mode.
*             This parameter can be a combination of the following values:
*             @arg @ref PWR_SLP_TIMER_MODE_NORMAL
*             @arg @ref PWR_SLP_TIMER_MODE_SINGLE
*             @arg @ref PWR_SLP_TIMER_MODE_RELOAD
* @param[in]  value: Count value of the AON Sleep Timer.
*
* @note The function will call the hal_sleep_timer_config_and_start function to initialize the HAL SLEEP TIMER.
*
* @retval ::HAL_OK:   Operation is OK.
* @retval ::HAL_BUSY: Driver is busy.
* @note   The sleep clock of AON Timer is 32 KHz.
****************************************************************************************
*/
hal_status_t hal_pwr_config_timer_wakeup_ext(uint8_t mode, uint32_t value);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @param[in]  pwr_slp_elapsed_hander: Pointer to callback structure function. @ref pwr_slp_elapsed_handler_t
 ****************************************************************************************
 */
void hal_pwr_register_timer_elaspsed_handler(pwr_slp_elapsed_handler_t pwr_slp_elapsed_hander);

#endif /* HAL_SLEEP_TIMER_MODULE_ENABLED */

#endif /* HAL_PWR_MODULE_ENABLED */


#ifdef HAL_QSPI_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the QSPI according to the specified parameters
 *         in the qspi_init_t and initialize the associated handle.
 *
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 *
 * @note The function will call the hal_qspi_init function to initialize the HAL QSPI.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_init_ext(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  De-initialize the QSPI peripheral.
 *
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 *
 * @note The function will call the hal_qspi_deinit function to De-initialize the HAL QSPI.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_deinit_ext(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_qspi_init.
 *
 * @param[in]  qspi_callback: Pointer to callback structure function. @ref qspi_callback_t
 ****************************************************************************************
 */
void hal_qspi_register_callback(qspi_callback_t *qspi_callback);

#endif /* HAL_QSPI_MODULE_ENABLED */


#ifdef HAL_SPI_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the SPI according to the specified parameters
 *         in the spi_init_t and initialize the associated handle.
 *
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 *
 * @note The function will call the hal_spi_init function to initialize the HAL SPI.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_init_ext(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  De-initialize the SPI peripheral.
 *
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 *
 * @note The function will call the hal_spi_deinit function to De-initialize the HAL SPI.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_deinit_ext(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_spi_init.
 *
 * @param[in]  spi_callback: Pointer to callback structure function. @ref spi_callback_t
 ****************************************************************************************
 */
void hal_spi_register_callback(spi_callback_t *spi_callback);

#endif /* HAL_SPI_MODULE_ENABLED */


#ifdef HAL_TIMER_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the TIMER according to the specified parameters
 *         in the timer_init_t and initialize the associated handle.
 *
 * @param[in]  p_timer: Pointer to a TIMER handle which contains the configuration
 *                 information for the specified TIMER module.
 *
 * @note The function will call the hal_timer_base_init function to initialize the HAL TIM.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_timer_base_init_ext(timer_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  De-initialize the TIMER peripheral.
 *
 * @param[in]  p_timer: Pointer to a TIMER handle which contains the configuration
 *                 information for the specified TIMER module.
 *
 * @note The function will call the hal_timer_base_deinit function to De-initialize the HAL TIM.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_timer_base_deinit_ext(timer_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_timer_base_init.
 *
 * @param[in]  timer_base_callback: Pointer to callback structure function. @ref timer_base_callback_t
 ****************************************************************************************
 */
void hal_timer_register_callback(timer_base_callback_t *timer_base_callback);

#endif /* HAL_TIMER_MODULE_ENABLED */


#ifdef HAL_UART_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief Initialize the UART according to the specified
 *        parameters in the uart_init_t and initialize the associated handle.
 *
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 *
 * @note The function will call the hal_uart_init function to initialize the HAL UART.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_init_ext(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief De-initialize the UART peripheral.
 *
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 *
 * @note The function will call the hal_uart_deinit function to De-initialize the HAL UART.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_deinit_ext (uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_uart_init.
 *
 * @param[in]  uart_callback: Pointer to callback structure function. @ref uart_callback_t
 ****************************************************************************************
 */
void hal_uart_register_callback(uart_callback_t *uart_callback);

#endif /* HAL_UART_MODULE_ENABLED */


#ifdef HAL_WDT_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the WDT according to the specified
 *         parameters in the wdt_init_t of  associated handle.
 *
 * @param[in]  p_wdt: Pointer to a WDT handle which contains the configuration
 *               information for the specified WDT module.
 *
 * @note The function will call the hal_wdt_init function to initialize the HAL WDT.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_wdt_init_ext(wdt_handle_t *p_wdt);

/**
 ****************************************************************************************
 * @brief  De-initialize the WDT peripheral.
 *
 * @param[in]  p_wdt: WDT handle.
 *
 * @note The function will call the hal_wdt_deinit function to De-initialize the HAL WDT.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_wdt_deinit_ext(wdt_handle_t *p_wdt);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_wdt_init.
 *
 * @param[in]  wdt_callback: Pointer to callback structure function. @ref wdt_callback_t
 ****************************************************************************************
 */
void hal_wdt_register_callback(wdt_callback_t *wdt_callback);

#endif /* HAL_WDT_MODULE_ENABLED */


#ifdef HAL_XQSPI_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the XQSPI according to the specified parameters
 *         in the xqspi_init_t and initialize the associated handle.
 *
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 *
 * @note The function will call the hal_xqspi_init function to initialize the HAL XQSPI.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_xqspi_init_ext(xqspi_handle_t *p_xqspi);

/**
 ****************************************************************************************
 * @brief  De-initialize the XQSPI peripheral.
 *
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 *
 * @note The function will call the hal_xqspi_deinit function to De-initialize the HAL XQSPI.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_xqspi_deinit_ext(xqspi_handle_t *p_xqspi);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_wdt_init.
 *
 * @param[in]  xqspi_callback: Pointer to callback structure function. @ref xqspi_callback_t
 ****************************************************************************************
 */
void hal_xqspi_register_callback(xqspi_callback_t *xqspi_callback);

#endif /* HAL_XQSPI_MODULE_ENABLED */


#ifdef HAL_EXFLASH_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the exFlash according to the specified parameters
 *         in the exflash_init_t and initialize the associated handle.
 *
 *
 * @note The function will call the hal_exflash_init function to initialize the HAL EXFLASH.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_init_ext(void);

/**
 ****************************************************************************************
 * @brief  De-initialize the exFlash peripheral.
 *
 *
 * @note The function will call the hal_exflash_deinit function to De-initialize the HAL EXFLASH.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_deinit_ext(void);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_exflash_init.
 *
 * @param[in]  exflash_callback: Pointer to callback structure function. @ref exflash_callback_t
 ****************************************************************************************
 */
void hal_exflash_register_callback(exflash_callback_t *exflash_callback);

#endif /* HAL_EXFLASH_MODULE_ENABLED */


#ifdef HAL_EFUSE_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the eFuse according to the specified parameters
 *         in the efuse_init_t and initialize the associated handle.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 *
 * @note The function will call the hal_efuse_init function to initialize the HAL EFUSE.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_efuse_init_ext(efuse_handle_t *p_efuse);

/**
 ****************************************************************************************
 * @brief  De-initialize the eFuse peripheral.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 *
 * @note The function will call the hal_efuse_deinit function to De-initialize the HAL EFUSE.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_efuse_deinit_ext(efuse_handle_t *p_efuse);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_efuse_init.
 *
 * @param[in]  efuse_callback: Pointer to callback structure function. @ref efuse_callback_t
 ****************************************************************************************
 */
void hal_efuse_register_callback(efuse_callback_t *efuse_callback);

#endif /* HAL_EFUSE_MODULE_ENABLED */


#ifdef HAL_RNG_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the RNG according to the specified
 *         parameters in the rng_init_t of  associated handle.
 *
 * @param[in]  p_rng: Pointer to a RNG handle which contains the configuration
 *               information for the specified RNG module.
 *
 * @note The function will call the hal_rng_init function to initialize the HAL RNG.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rng_init_ext(rng_handle_t *p_rng);

/**
 ****************************************************************************************
 * @brief  De-initialize the RNG peripheral.
 *
 * @param[in]  p_rng: RNG handle.
 *
 * @note The function will call the hal_rng_deinit function to De-initialize the HAL RNG.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rng_deinit_ext(rng_handle_t *p_rng);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_rng_init.
 *
 * @param[in]  rng_callback: Pointer to callback structure function. @ref rng_callback_t
 ****************************************************************************************
 */
void hal_rng_register_callback(rng_callback_t *rng_callback);

#endif /* HAL_RNG_MODULE_ENABLED */


#ifdef HAL_COMP_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the COMP according to the specified parameters
 *         in the comp_init_t and initialize the associated handle.
 *
 * @param[in]  p_comp: Pointer to a COMP handle which contains the configuration information for
 *                    the specified COMP module.
 *
 * @note The function will call the hal_comp_init function to initialize the HAL COMP.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_comp_init_ext(comp_handle_t *p_comp);

/**
 ****************************************************************************************
 * @brief  De-initialize the COMP peripheral.
 *
 * @param[in]  p_comp: Pointer to a COMP handle which contains the configuration information for
 *                    the specified COMP module.
 *
 * @note The function will call the hal_comp_deinit function to De-initialize the HAL COMP.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_comp_deinit_ext(comp_handle_t *p_comp);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_comp_init.
 *
 * @param[in]  comp_callback: Pointer to callback structure function. @ref comp_callback_t
 ****************************************************************************************
 */
void hal_comp_register_callback(comp_callback_t *comp_callback);

#endif /* HAL_COMP_MODULE_ENABLED */


#ifdef HAL_ISO7816_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initializes the ISO7816 according to the specified parameters
 *         in the iso7816_init_t and initialize the associated handle.
 *
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration
 *                information for the specified ISO7816.
 *
 * @note The function will call the hal_iso7816_init function to initialize the HAL ISO7816.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_iso7816_init_ext(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  De-initializes the ISO7816 according to the specified parameters
 *         in the iso7816_init_t and initialize the associated handle.
 *
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration
 *                information for the specified ISO7816.
 *
 * @note The function will call the hal_iso7816_deinit function to De-initialize the HAL ISO7816.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_iso7816_deinit_ext(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_iso7816_init.
 *
 * @param[in]  iso7816_callback: Pointer to callback structure function. @ref iso7816_callback_t
 ****************************************************************************************
 */
void hal_iso7816_register_callback(iso7816_callback_t *iso7816_callback);

#endif /* HAL_ISO7816_MODULE_ENABLED */


#ifdef HAL_PDM_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the PDM according to the specified parameters
 *         in the pdm_init_t and initialize the associated handle.
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                    the specified PDM module.
 *
 * @note The function will call the hal_pdm_init function to initialize the HAL PDM.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_init_ext(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  De-initialize the PDM peripheral.
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                    the specified PDM module.
 *
 * @note The function will call the hal_pdm_deinit function to De-initialize the HAL PDM.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_deinit_ext(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_pdm_init.
 *
 * @param[in]  pdm_callback: Pointer to callback structure function. @ref pdm_callback_t
 ****************************************************************************************
 */
void hal_pdm_register_callback(pdm_callback_t *pdm_callback);

#endif /* HAL_PDM_MODULE_ENABLED */


#ifdef HAL_DSPI_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the DSPI according to the specified parameters
 *         in the dspi_init_t and initialize the associated handle.
 *
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 *
 * @note The function will call the hal_dspi_init function to initialize the HAL DSPI.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_init_ext(dspi_handle_t *p_dspi);

/**
 ****************************************************************************************
 * @brief  De-initialize the DSPI peripheral.
 *
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 *
 * @note The function will call the hal_dspi_deinit function to De-initialize the HAL DSPI.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_deinit_ext(dspi_handle_t *p_dspi);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_dspi_init.
 *
 * @param[in]  dspi_callback: Pointer to callback structure function. @ref dspi_callback_t
 ****************************************************************************************
 */
void hal_dspi_register_callback(dspi_callback_t *dspi_callback);

#endif /* HAL_DSPI_MODULE_ENABLED */



#ifdef HAL_GPADC_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the GPADC according to the specified parameters
 *         in the gpadc_init_t and initialize the associated handle.
 *
 * @param[in]  p_gpadc: Pointer to an GPADC handle which contains the configuration information for
 *                    the specified GPADC module.
 *
 * @note The function will call the hal_gpadc_init function to initialize the HAL GPADC.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_gpadc_init_ext(gpadc_handle_t *p_gpadc);

/**
 ****************************************************************************************
 * @brief  De-initialize the GPADC peripheral.
 *
 * @param[in]  p_gpadc: Pointer to an GPADC handle which contains the configuration information for
 *                    the specified GPADC module.
 *
 * @note The function will call the hal_gpadc_deinit function to De-initialize the HAL GPADC.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_gpadc_deinit_ext(gpadc_handle_t *p_gpadc);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_gpadc_init.
 *
 * @param[in]  gpadc_callback: Pointer to callback structure function. @ref gpadc_callback_t
 ****************************************************************************************
 */
void hal_gpadc_register_callback(gpadc_callback_t *gpadc_callback);

#endif /* HAL_GPADC_MODULE_ENABLED */


#ifdef HAL_USB_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the USB according to the specified parameters
 *         in the usb_init_t and initialize the associated handle.
 *
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 *
 * @note The function will call the hal_usb_init function to initialize the HAL USB.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_init_ext(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  De-initialize the USB peripheral.
 *
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 *
 * @note The function will call the hal_usb_deinit function to De-initialize the HAL USB.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_deinit_ext(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_usb_init.
 *
 * @param[in]  usb_callback: Pointer to callback structure function. @ref usb_callback_t
 ****************************************************************************************
 */
void hal_usb_register_callback(usb_callback_t *usb_callback);

#endif /* HAL_USB_MODULE_ENABLED */

#ifdef HAL_BOD_MODULE_ENABLED

/**
 ****************************************************************************************
 * @brief  Initialize the BOD according to the specified parameters
 *         in the bod_init_t and initialize the associated handle.
 *
 * @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
 *                    the specified BOD module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_bod_init_ext(bod_handle_t *p_bod);

/**
 ****************************************************************************************
 * @brief  De-initialize the BOD peripheral.
 *
 * @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
 *                    the specified BOD module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_bod_deinit_ext(bod_handle_t *p_bod);

/**
 ****************************************************************************************
 * @brief  This function registers the callback function to the ROM area.
 *
 * @note This function needs to be called before hal_bod__init.
 *
 * @param[in]  bod_callback: Pointer to callback structure function. @ref bod_callback_t
 ****************************************************************************************
 */
void hal_bod_register_callback(bod_callback_t *bod_callback);
#endif

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_BR_H__ */

/** @} */

/** @} */

/** @} */

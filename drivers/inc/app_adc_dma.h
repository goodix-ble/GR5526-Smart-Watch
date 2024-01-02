/**
 ****************************************************************************************
 *
 * @file    app_adc_dma.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of ADC app library.
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

/** @defgroup APP_ADC ADC
  * @brief ADC APP module driver.
  * @{
  */


#ifndef _APP_ADC_DMA_H_
#define _APP_ADC_DMA_H_

#include "grx_hal.h"
#include "app_io.h"
#include "app_drv_error.h"
#include "app_drv_config.h"
#include "app_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_ADC_MODULE_ENABLED

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_ADC_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the APP ADC DRIVER according to the specified parameters
 *         in the app_adc_params_t and app_adc_evt_handler_t.
 * @note   If DMA mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use DMA mode.
 *
 * @param[in]  p_params: Pointer to app_adc_params_t parameter which contains the
 *                       configuration information for the specified ADC module.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_adc_dma_init(app_adc_params_t *p_params);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP ADC DRIVER peripheral.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_adc_dma_deinit(void);

/**
 ****************************************************************************************
 * @brief  DMA for conversion.
 *
 * @param[in]  p_data: Pointer to data buffer which to storage ADC conversion results.
 * @param[in]  length: Length of data buffer,  ranging between 0 and 4095.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_adc_dma_conversion_async(uint16_t *p_data, uint32_t length);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */

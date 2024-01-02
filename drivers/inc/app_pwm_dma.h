/**
 ****************************************************************************************
 *
 * @file    app_pwm_dma.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PWM app library.
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

#ifndef _APP_PWM_DMA_H_
#define _APP_PWM_DMA_H_

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_PWM  PWM
  * @brief PWM APP module driver.
  * @{
  */


#include "app_io.h"
#include "app_dma.h"
#include "app_pwm.h"
#include "app_drv_error.h"
#include "app_drv_config.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_PWM_MODULE_ENABLED
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_PWM_DMA_DRIVER_FUNCTIONS Functions
 *  @{
 */
/**
 ****************************************************************************************
 * @brief  Initialize the APP PWM DMA DRIVER according to the specified parameters
 *         in the app_pwm_params_t and app_pwm_evt_handler_t.
 *
 * @param[in]  p_params: Pointer to app_pwm_params_t parameter which contains the
 *                       configuration information for the specified PWM module.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_pwm_dma_init(app_pwm_params_t *p_params);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP PWM DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_pwm_dma_deinit(app_pwm_id_t id);

/**
 ****************************************************************************************
 * @brief  Start generate wave form in DMA mode
 *
 * @param[in]  id: which PWM module want to config.
 * @param[in]  p_data: the coding data address.
 * @param[in]  size: coding data size.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_pwm_start_coding_with_dma(app_pwm_id_t id, uint32_t *p_data, uint16_t size);

/** @} */


#endif

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */

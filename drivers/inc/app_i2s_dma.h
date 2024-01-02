/**
 ****************************************************************************************
 *
 * @file    app_i2s_dma.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of I2S app library.
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

/** @defgroup APP_I2S I2S
  * @brief I2S APP module driver.
  * @{
  */


#ifndef _APP_I2S_DMA_PUB_H_
#define _APP_I2S_DMA_PUB_H_

#include "grx_hal.h"
#include "app_io.h"
#include "app_i2s.h"
#include "app_drv_error.h"
#include "app_drv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_I2S_MODULE_ENABLED

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_I2S_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the APP I2S DRIVER according to the specified parameters
 *         in the app_i2s_params_t and app_i2s_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_i2s_params_t parameter which contains the
 *                       configuration information for the specified I2S module.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_i2s_dma_init(app_i2s_params_t *p_params);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP I2S DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_i2s_dma_deinit(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief  Receive in master or slave mode an amount of data in non-blocking mode with Interrupt
 *
 * @param[in]  id: which I2S module want to receive.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_dma_receive_async(app_i2s_id_t id, uint16_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmits in master or slave mode an amount of data in non-blocking mode with Interrupt
 *
 * @param[in]  id: which I2S module want to transmit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_dma_transmit_async(app_i2s_id_t id, uint16_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
 * @param[in]  id: I2S ID
 * @param[in]  p_tx_data: Pointer to transmission data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  length: Amount of data to be sent and received in bytes
 * @retval ::APP_DRV_SUCCESS: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 * @retval ::APP_DRV_ERR_INVALID_PARAM: Parameter error.
 * @retval ::APP_DRV_ERR_BUSY: Driver is busy.
 ****************************************************************************************
 */
uint16_t app_i2s_dma_transmit_receive_async(app_i2s_id_t id,
                                            uint16_t    *p_tx_data,
                                            uint16_t    *p_rx_data,
                                            uint32_t     length);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */



/**
 ****************************************************************************************
 *
 * @file    app_uart_dma.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of UART app library.
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

/** @defgroup APP_UART UART
  * @brief UART APP module driver.
  * @{
  */


#ifndef _APP_UART_DMA_H_
#define _APP_UART_DMA_H_

#include "app_uart.h"
#include "grx_hal.h"
#include "app_drv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_UART_MODULE_ENABLED

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_UART_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize dma mode of the APP UART DRIVER according to the specified parameters
 *         in the app_uart_params_t.
 *
 * @param[in]  p_params: Pointer to app_uart_params_t parameter which contains the
 *                       configuration information for the specified UART module.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_uart_dma_init(app_uart_params_t *p_params);

/**
 ****************************************************************************************
 * @brief  De-initialize dma mode of the APP UART peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_uart_dma_deinit(app_uart_id_t id);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in dma mode.
 *
 * @param[in]  id: which UART module want to receive.
 * @param[in]  p_data: Pointer to data buffer.
 * @param[in]  size: Amount of data to receive.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_uart_dma_receive_async(app_uart_id_t id, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Send an amount of data in dma mode.
 *
 * @param[in]  id: which UART module want to send.
 * @param[in]  p_data: Pointer to data buffer.
 * @param[in]  size: Amount of data to be sent.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_uart_dma_transmit_async(app_uart_id_t id, uint8_t *p_data, uint16_t size);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
/**
 ****************************************************************************************
 * @brief  Send an amount of data in DMA mode with DMA sg and llp function.
 *
 * @param[in]  id: which UART module want to receive.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  sg_llp_config: The config of source and destination's sg and llp fuction.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_uart_transmit_dma_sg_llp(app_uart_id_t id, uint8_t *p_data, uint16_t size, dma_sg_llp_config_t *sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in in dma_sg_llp mode.
 *
 * @param[in]  id: which UART module want to transmit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  sg_llp_config: The config of source and destination's sg and llp fuction.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_uart_receive_dma_sg_llp(app_uart_id_t id, uint8_t *p_data, uint16_t size, dma_sg_llp_config_t *sg_llp_config);
#endif

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */

/** @} */

/** @} */


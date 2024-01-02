/**
 ****************************************************************************************
 *
 * @file    app_dspi_dma.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of DSPI app library.
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

/** @defgroup APP_DSPI DSPI
  * @brief DSPI APP module driver.
  * @{
  */


#ifndef _APP_DSPI_DMA_H_
#define _APP_DSPI_DMA_H_

#include "grx_hal.h"
#include "app_io.h"
#include "app_dspi.h"
#include "app_drv_error.h"
#include "app_drv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_DSPI_MODULE_ENABLED

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_DSPI_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP DSPI DRIVER according to the specified parameters
 *         in the app_dspi_params_t and app_dspi_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_dspi_params_t parameter which contains the
 *                       configuration information for the specified DSPI module.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_dspi_dma_init(app_dspi_params_t *p_params);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP DSPI DRIVER peripheral.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_dspi_dma_deinit(void);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction in non-blocking mode with Interrupt.
 *
 * @param[in]  p_cmd: Pointer to a app_dspi_command_t structure that contains the instruction, length, data_size for data transfer.
 * @param[out] p_data: Pointer to data buffer
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_dma_command_transmit_async(app_dspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Transmit instruction in non-blocking mode with Interrupt.
 *
 * @param[in]  p_cmd: Pointer to a app_dspi_command_t structure that contains the instruction for data transfer.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_dma_command_async(app_dspi_command_t *p_cmd);

/**
 ****************************************************************************************
 * @brief Transmit an amount of data in non-blocking mode with DMA.
 *
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 0 and 4095.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_dma_transmit_async(uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief Transmit an amount of data in non-blocking mode with DMA SG or LLP.
 *
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 0 and 4095.
 * @param[in]  sg_llp_config: The config of source and destination's SG and LLP.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_dma_sg_llp_transmit_async(uint8_t *p_data, uint32_t length, dma_sg_llp_config_t *sg_llp_config);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif
/** @} */

/** @} */

/** @} */

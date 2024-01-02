/**
 ****************************************************************************************
 *
 * @file    app_i2c_dma.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of I2C app library.
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

/** @defgroup APP_IIC IIC
  * @brief IIC APP module driver.
  * @{
  */


#ifndef _APP_I2C_DMA_H_
#define _APP_I2C_DMA_H_

#include "grx_hal.h"
#include "app_io.h"
#include "app_i2c.h"
#include "app_drv_error.h"
#include "app_drv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_I2C_MODULE_ENABLED

/** @addtogroup APP_I2C_ENUM Enumerations
  * @{
  */


/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_I2C_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP I2C DRIVER according to the specified parameters
 *         in the app_i2c_params_t and app_i2c_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_i2c_params_t parameter which contains the
 *                       configuration information for the specified I2C module.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_i2c_dma_init(app_i2c_params_t *p_params);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP I2C DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_i2c_dma_deinit(app_i2c_id_t id);

/**
 ****************************************************************************************
 * @brief  Receive in master or slave mode an amount of data in non-blocking mode with Interrupt/DMA.
 *
 * @param[in]  id: which I2C module want to receive.
 * @param[in]  target_address: Target device address: The device 7 bits address value in datasheet
               must be shifted at right before call interface.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_dma_receive_async(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmits in master or slave mode an amount of data in non-blocking mode with Interrupt/DMA.
 *
 * @param[in]  id: which I2C module want to transmit.
 * @param[in]  target_address: Target device address: The device 7 bits address value in datasheet
               must be shifted at right before call interface.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_dma_transmit_async(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Read an amount of data in non-blocking mode with Interrupt/DMA from a specific memory address
 *
 * @param[in]  id: which I2C module want to read.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_dma_mem_read_async(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Write an amount of data in non-blocking mode with Interrupt/DMA to a specific memory address
 *
 * @param[in]  id: which I2C module want to write.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_dma_mem_write_async(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size);

/** @} */
#endif

#ifdef __cplusplus
}
#endif

#endif
/** @} */

/** @} */

/** @} */


/**
 ****************************************************************************************
 *
 * @file    app_qspi_dma.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of QSPI app library.
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

/** @defgroup APP_QSPI QSPI
  * @brief QSPI APP module driver.
  * @{
  */


#ifndef __APP_QSPI_DMA_H__
#define __APP_QSPI_DMA_H__

#include <stdbool.h>
#include "grx_hal.h"
#include "app_io.h"
#include "app_qspi.h"
#include "app_drv_error.h"
#include "app_drv_config.h"
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
#include "app_qspi_user_config.h"
#endif
#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_QSPI_MODULE_ENABLED

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_QSPI_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP QSPI DRIVER according to the specified parameters
 *         in the app_qspi_params_t and app_qspi_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_qspi_params_t parameter which contains the
 *                       configuration information for the specified QSPI module.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_qspi_dma_init(app_qspi_params_t *p_params);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP QSPI DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_qspi_dma_deinit(app_qspi_id_t id);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction, address and dummy cycles in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  id: which QSPI module want to receive.
 * @param[in]  p_cmd: Pointer to a app_qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_dma_command_receive_async(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction, address and dummy cycles in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  id: which QSPI module want to transmit.
 * @param[in]  p_cmd: Pointer to a app_qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_dma_command_transmit_async(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Transmit instruction in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  id: which QSPI module want to transmit command.
 * @param[in]  p_cmd: Pointer to a app_qspi_command_t structure that contains the instruction and address for data transfer.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_dma_command_async(app_qspi_id_t id, app_qspi_command_t *p_cmd);

/**
 ****************************************************************************************
 * @brief  Transmit data without command, support std/dual/quad mode
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  qspi_mode : @ref QSPI_DATA_MODE_SPI
 *						   @ref QSPI_DATA_MODE_DUALSPI
 *						   @ref QSPI_DATA_MODE_QUADSPI
 * @param[in]  data_width :@ref QSPI_DATASIZE_08_BITS
 *						   @ref QSPI_DATASIZE_16_BITS
 *						   @ref QSPI_DATASIZE_32_BITS
 * @param[in]  p_data : data Pointer to transmit
 * @param[in]  length : byte length of data
 *
 * @return true/false
 ****************************************************************************************
 */
uint16_t app_qspi_dma_transmit_async_ex(app_qspi_id_t id, uint32_t qspi_mode, uint32_t data_width, uint8_t *p_data, uint32_t length);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
/**
 ****************************************************************************************
 * @brief Transmit an amount of data in non-blocking mode at standard SPI with Interrupt.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  id: which QSPI module want to transmit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_dma_transmit_async(app_qspi_id_t id, uint8_t *p_data, uint32_t length);
#endif

/**
 ****************************************************************************************
 * @brief  Receive data without command, support std/dual/quad mode
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  qspi_mode : @ref QSPI_DATA_MODE_SPI
 *                         @ref QSPI_DATA_MODE_DUALSPI
 *                         @ref QSPI_DATA_MODE_QUADSPI
 * @param[in]  data_width :@ref QSPI_DATASIZE_08_BITS
 *                         @ref QSPI_DATASIZE_16_BITS
 *                         @ref QSPI_DATASIZE_32_BITS
 * @param[in]  p_data : data Pointer to transmit
 * @param[in]  length : byte length of data
 *
 * @return true/false
 ****************************************************************************************
 */
uint16_t app_qspi_dma_receive_async_ex(app_qspi_id_t id, uint32_t qspi_mode, uint32_t data_width, uint8_t *p_data, uint32_t length);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
/**
 ****************************************************************************************
 * @brief Receive an amount of data in non-blocking mode at standard SPI with Interrupt.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  id: which QSPI module want to receive.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be received in bytes
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_dma_receive_async(app_qspi_id_t id, uint8_t *p_data, uint32_t length);
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
/**
 ****************************************************************************************
 * @brief Transmit an amount of data in QPI mode (Async Mode).
 * @param[in] id:         Which QSPI module want to Transmit.
 * @param[in] data_width: Just support @ref QSPI_DATASIZE_08_BITS @ref QSPI_DATASIZE_16_BITS @ref QSPI_DATASIZE_32_BITS
 * @param[in] p_data:     Pointer to data buffer
 * @param[in] length:     Amount of data to be transmitted in bytes
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_dma_transmit_in_qpi_async(app_qspi_id_t id, uint32_t data_width, uint8_t *p_data, uint32_t length);
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

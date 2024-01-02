/**
 ****************************************************************************************
 *
 * @file    app_spi_dma.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of SPI app library.
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

/** @defgroup APP_SPI SPI
  * @brief SPI APP module driver.
  * @{
  */


#ifndef _APP_SPI_DMA_H_
#define _APP_SPI_DMA_H_

#include "app_io.h"
#include "app_dma.h"
#include "app_spi.h"
#include "app_drv_error.h"
#include "app_drv_config.h"
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_SPI_MODULE_ENABLED

/** @addtogroup APP_SPI_STRUCTURES Structures
  * @{
  */
/**
  * @brief SPI SCreen Config Structures
  */
typedef struct {
    uint8_t     instruction;            /**< screen instruction, such as 0x02                             */
    uint32_t    leading_address;        /**< control address started from origin, such as 0x002C00        */
    uint32_t    ongoing_address;        /**< control address started from last position, such as 0x003C00 */
    uint32_t    data_xfer_width;        /**< data width, @ref SPI_DATASIZE_8BIT SPI_DATASIZE_16BIT        */
    uint32_t    buff_pixel_stride;      /**< row stride in pixel                                          */
    uint32_t    buff_pixel_width;       /**< show width in pixel, usually buff_pixel_width <= buff_pixel_stride */
    uint32_t    buff_pixel_height;      /**< column height in pixel             */
    uint32_t    buff_pixel_depth;       /**< Just support 2 now (means 16bit).  */
} app_spi_screen_config_t;

/** @} */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_SPI_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP SPI DRIVER according to the specified parameters
 *         in the app_spi_params_t and app_spi_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_spi_params_t parameter which contains the
 *                       configuration information for the specified SPI module.
 *
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_spi_dma_init(app_spi_params_t *p_params);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP SPI DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_spi_dma_deinit(app_spi_id_t id);
/**
 ****************************************************************************************
 * @brief  SPI master transmit with 1-byte inst and 3-byte addr, can use to write flash/display/eeprom, etc
 * @note   DO NOT Support interrupt mode
 * @param[in]  id          : just support APP_SPI_ID_MASTER
 * @param[in]  instruction : 1-byte instruction phase
 * @param[in]  address     : 3-byte address phase
 * @param[in]  p_data      : pointer to transmit buffer
 * @param[in]  data_length : length of buffer, unit in byte
 *
 * @return APP_DRV_* in app_drv_error.h
 ****************************************************************************************
 */
uint16_t app_spim_dma_transmit_with_ia(app_spi_id_t id, uint8_t instruction, uint32_t address, uint8_t * p_data, uint16_t data_length);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
/**
 ****************************************************************************************
 * @brief  SPI master transmit with 1-byte inst and 4-byte addr, can use to write flash/display/eeprom, etc
 * @note   DO NOT Support interrupt mode
 * @param[in]  id          : just support APP_SPI_ID_MASTER
 * @param[in]  instruction : 1-byte instruction phase
 * @param[in]  address     : 3-byte address phase
 * @param[in]  p_data      : pointer to transmit buffer
 * @param[in]  data_length : length of buffer, unit in byte
 *
 * @return APP_DRV_* in app_drv_error.h
 ****************************************************************************************
 */
uint16_t app_spim_dma_transmit_with_ia_32addr(app_spi_id_t id, uint8_t instruction, uint32_t address, uint8_t * p_data, uint16_t data_length);
#endif

/**
 ****************************************************************************************
 * @brief  SPI master receive with 1-byte inst and 3-byte addr and 0~4 dummy Byte, can use to read flash/display/eeprom, etc
 *
 * @param[in]  id          : just support APP_SPI_ID_MASTER
 * @param[in]  instruction : 1-byte instruction phase
 * @param[in]  address     : 3-byte address phase
 * @param[in]  dummy_bytes : dummy bytes, 0 ~ 4
 * @param[in]  p_data      : pointer to transmit buffer
 * @param[in]  data_length : length of buffer, unit in byte
 *
 * @return APP_DRV_* in app_drv_error.h
 ****************************************************************************************
 */
uint16_t app_spim_dma_receive_with_ia(app_spi_id_t id, uint8_t instruction, uint32_t address, uint8_t dummy_bytes, uint8_t * p_data, uint16_t data_length);

/**
 ****************************************************************************************
 * @brief  Receive in master or slave mode an amount of data in non-blocking mode with Interrupt
 *
 * @param[in]  id: which SPI module want to receive.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_dma_receive_async(app_spi_id_t id, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmits in master or slave mode an amount of data in non-blocking mode with Interrupt
 *
 * @param[in]  id: which SPI module want to transmit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_dma_transmit_async(app_spi_id_t id, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmits and receive in master or slave mode an amount of data in non-blocking mode with Interrupt
 *
 * @param[in]  id: which SPI module want to transmit.
 * @param[in]  p_tx_data: Pointer to tx data buffer
 * @param[in]  p_rx_data: Pointer to rx data buffer
 * @param[in]  size: Amount of data to be sent and receive
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_dma_transmit_receive_async(app_spi_id_t id, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t size);

/**
 ****************************************************************************************
 * @brief  Read an amount of data from EEPROM in non-blocking mode with Interrupt.
 *
 * @param[in]  id: which SPI module want to transmit.
 * @param[in]  p_tx_data: Pointer to transmission data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  tx_size: Amount of data to be sent in bytes
 * @param[in]  rx_size: Amount of data to be received in bytes
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_dma_read_eeprom_async(app_spi_id_t id, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t tx_size, uint32_t rx_size);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
/**
 ****************************************************************************************
 * @brief  Transmits in master or slave mode an amount of data in non-blocking mode with DMA
 *
 * @param[in]  id: which SPI module want to transmit.
 * @param[in]  p_cmd_data: Pointer to command data buffer
 * @param[in]  p_tx_data: Pointer to transmission data buffer
 * @param[in]  cmd_size: Amount of command data to be sent in bytes
 * @param[in]  tx_size: Amount of data to be sent in bytes
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_dma_write_memory_async(app_spi_id_t id, uint8_t *p_cmd_data, uint8_t *p_tx_data, uint32_t cmd_size, uint32_t tx_size);

/**
 ****************************************************************************************
 * @brief  Read an amount of data from EEPROM in non-blocking mode with DMA.
 *
 * @param[in]  id: which SPI module want to transmit.
 * @param[in]  p_cmd_data: Pointer to command data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  cmd_size: Amount of command data to be sent in bytes
 * @param[in]  rx_size: Amount of data to be received in bytes
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_dma_read_memory_async(app_spi_id_t id, uint8_t *p_cmd_data, uint8_t *p_rx_data, uint32_t cmd_size, uint32_t rx_size);

/**
 ****************************************************************************************
 * @brief  [High speed] Receive in master or slave mode an amount of data in blocking mode.
 *
 * @param[in]  id: which SPI module want to receive.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_dma_receive_high_speed_sync(app_spi_id_t id, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  [High speed] Transmit in master or slave mode an amount of data in blocking mode.
 *
 * @param[in]  id: which SPI module want to receive.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_dma_transmit_high_speed_sync(app_spi_id_t id, uint8_t *p_data, uint16_t size);
#endif


#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
/**
 ****************************************************************************************
 * @brief  transfer big block data from sram to screen device in dma llp mode. Must override _v_malloc and _v_free in application layer
 *
 * @param[in]  id: which SPI module want to transmit.
 * @param[in]  p_scrn_info: Pointer to screen configure information
 * @param[in]  p_buff: Pointer to data buffer to transfer
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_send_display_frame(app_spi_id_t id, app_spi_screen_config_t * p_scrn_info, void * p_buff) ;
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


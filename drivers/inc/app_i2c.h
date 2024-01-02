/**
 ****************************************************************************************
 *
 * @file    app_i2c.h
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


#ifndef _APP_I2C_H_
#define _APP_I2C_H_

#include "grx_hal.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_drv_error.h"
#include "app_drv_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_I2C_MODULE_ENABLED

/** @addtogroup APP_I2C_ENUM Enumerations
  * @{
  */

/**
  * @brief I2C module Enumerations definition
  */
typedef enum
{
    APP_I2C_ID_0,                   /**< I2C module 0. */
    APP_I2C_ID_1,                   /**< I2C module 1. */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    APP_I2C_ID_2,                   /**< I2C module 2. */
    APP_I2C_ID_3,                   /**< I2C module 3. */
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
    APP_I2C_ID_4,                   /**< I2C module 4. */
    APP_I2C_ID_5,                   /**< I2C module 5. */
#endif
    APP_I2C_ID_MAX                  /**< Only for check parameter, not used as input parameters. */
} app_i2c_id_t;

/**
  * @brief I2C role Enumerations definition
  */
typedef enum
{
    APP_I2C_ROLE_MASTER,           /**< I2C master device. */
    APP_I2C_ROLE_SLAVE,            /**< I2C slave device. */
    APP_I2C_ROLE_MAX,              /**< Only for check parameter, not used as input parameters. */
} app_i2c_role_t;

/**
  * @brief I2C event Enumerations definition
  */
typedef enum
{
    APP_I2C_EVT_ERROR,                   /**< Error reported by I2C peripheral. */
    APP_I2C_EVT_TX_CPLT,                 /**< Requested TX transfer completed. */
    APP_I2C_EVT_RX_DATA,                 /**< Requested RX transfer completed. */
    APP_I2C_ABORT,                       /**< abort reported by I2C peripheral. */
} app_i2c_evt_type_t;
/** @} */

/** @addtogroup APP_I2C_STRUCTURES Structures
  * @{
  */
/**
  * @brief I2C pins Structures
  */
typedef struct
{
    app_io_type_t        type;     /**< Specifies the type of SPI IO. */
    app_io_mux_t         mux;      /**< Specifies the Peripheral to be connected to the selected pins. */
    uint32_t             pin;      /**< Specifies the IO pins to be configured.
                                        This parameter can be any value of @ref GR5xxx_pins. */
    app_io_pull_t        pull;     /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
} app_i2c_pin_t;

/**
  * @brief I2C pins config Structures
  */
typedef struct
{
    app_i2c_pin_t       scl;       /**< Set the configuration of I2C SCL pin. */
    app_i2c_pin_t       sda;       /**< Set the configuration of I2C SDA pin. */
} app_i2c_pin_cfg_t;

/**
  * @brief I2C operate mode Enumerations definition
  */
typedef struct
{
    dma_regs_t          *tx_dma_instance;/**< Specifies the TX DMA instance.*/
    dma_regs_t          *rx_dma_instance;/**< Specifies the RX DMA instance.*/
    dma_channel_t       tx_dma_channel;  /**< Specifies the dma channel of I2C TX. */
    dma_channel_t       rx_dma_channel;  /**< Specifies the dma channel of I2C RX. */
} app_i2c_dma_cfg_t;

/**
  * @brief I2C event structure definition
  */
typedef struct
{
    app_i2c_evt_type_t type;             /**< Type of event. */
    union
    {
        uint32_t error_code;             /**< I2C Error code. */
        uint16_t size;                   /**< I2C transmitted/received counter. */
    } data;                              /**< Data of event. */
    uint16_t slave_addr;                 /**< I2C slave address. */
} app_i2c_evt_t;

/** @} */

/** @addtogroup APP_I2C_ENUM Enumerations
  * @{
  */
/**@brief App i2c state types. */
typedef enum
{
    APP_I2C_INVALID = 0,
    APP_I2C_ACTIVITY,
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    APP_I2C_SLEEP,
#endif
} app_i2c_state_t;

/**@brief App i2c dma state types. */
typedef enum
{
    APP_I2C_DMA_INVALID = 0,
    APP_I2C_DMA_ACTIVITY,
} app_i2c_dma_state_t;

/** @} */

/** @addtogroup APP_I2C_TYPEDEFS Type definitions
  * @{
  */
/**
  * @brief I2C event callback definition
  */
typedef void (*app_i2c_evt_handler_t)(app_i2c_evt_t *p_evt);

/** @} */

/** @addtogroup APP_I2C_STRUCTURES Structures
  * @{
  */
/**
  * @brief I2C device structure definition
  */
typedef struct
{
    app_i2c_evt_handler_t   evt_handler;         /**< I2C event callback definition. */
    i2c_handle_t            handle;              /**< I2C handle definition. */
    app_i2c_role_t          role;                /**< I2C role Enumerations definition. */
    app_i2c_pin_cfg_t       *p_pin_cfg;          /**< I2C pins config Structures. */
    dma_id_t                dma_id[2];           /**< DMA id definition. */
    app_i2c_state_t         i2c_state;           /**< I2C state types. */
    app_i2c_dma_state_t     i2c_dma_state;       /**< I2C dma state types. */
    volatile bool           start_flag;          /**< Start flag definition. */
    uint16_t                slv_dev_addr;        /**< I2C Slave address. */
} i2c_env_t;

/**
  * @brief I2C parameters structure definition
  */
typedef struct
{
    app_i2c_id_t        id;              /**< specified I2C module ID. */
    app_i2c_role_t      role;            /**< specified the role of I2C. */
    app_i2c_pin_cfg_t   pin_cfg;         /**< the pin configuration information for the specified I2C module. */
    app_i2c_dma_cfg_t   dma_cfg;         /**< I2C operate mode. */
    i2c_init_t          init;            /**< I2C communication parameters. */
    i2c_env_t           i2c_dev;         /**< I2C device structure definition. */
} app_i2c_params_t;

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
 * @param[in]  evt_handler: I2C user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_i2c_init(app_i2c_params_t *p_params, app_i2c_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP I2C DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_i2c_deinit(app_i2c_id_t id);

/**
 ****************************************************************************************
 * @brief  Receive in master or slave mode an amount of data in blocking mode.
 *
 * @param[in]  id: which I2C module want to receive.
 * @param[in]  target_address: Target device address: The device 7 bits address value in datasheet
               must be shifted at right before call interface.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_receive_sync(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size, uint32_t timeout);

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
uint16_t app_i2c_receive_async(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmits in master or slave mode an amount of data in blocking mode.
 *
 * @param[in]  id: which I2C module want to transmit.
 * @param[in]  target_address: Target device address: The device 7 bits address value in datasheet
               must be shifted at right before call interface.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_transmit_sync(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size, uint32_t timeout);

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
uint16_t app_i2c_transmit_async(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Read an amount of data in blocking mode from a specific memory address
 *
 * @param[in]  id: which I2C module want to read.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_mem_read_sync(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout);

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
uint16_t app_i2c_mem_read_async(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Write an amount of data in blocking mode to a specific memory address
 *
 * @param[in]  id: which I2C module want to write.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_mem_write_sync(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout);

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
uint16_t app_i2c_mem_write_async(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Return the I2C handle.
 *
 * @param[in]  id: I2C Channel ID.
 *
 * @return Pointer to the specified ID's I2C handle.
 ****************************************************************************************
 */
i2c_handle_t *app_i2c_get_handle(app_i2c_id_t id);

/**
 ****************************************************************************************
 * @brief  Abort a master I2C IT or DMA process communication with Interrupt
 *
 * @param[in]  id: I2C Channel ID.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_master_abort_it(app_i2c_id_t id);

/**
 ****************************************************************************************
 * @brief  Adjust I2C timing value to adapt to real load.
 *
 * @param[in]  id: I2C Channel ID.
 * @param[in]  timing_type: Timing type. See I2C_Timing_type.
 * @param[in]  delta: timing change value(unit: I2C work clock cycles).
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_timing_adjust(app_i2c_id_t id, uint32_t timing_type, int32_t delta);

/**
 ****************************************************************************************
 * @brief  Get I2C timing value.
 *
 * @param[in]  id: I2C Channel ID.
 * @param[in]  timing_type: Timing type. See I2C_Timing_type.
 * @param[in]  p_timing_value: Pointer of I2C timing value(unit: I2C work clock cycles).
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_timing_get(app_i2c_id_t id, uint32_t timing_type, uint32_t *p_timing_value);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
/**
 ****************************************************************************************
 * @brief  Transmits than receives in master mode or receives than transmits in slave mode with only one stop bit.
 *
 * @param[in]  id: which I2C module want to transmit.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet
               must be shifted at right before call interface.
 * @param[in]  p_tdata: Pointer to transmit data buffer
 * @param[in]  tsize: Amount of data to be sent
 * @param[in]  p_rdata: Pointer to received data buffer
 * @param[in]  rsize: Amount of data to be receive
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_transmit_receive_sync(app_i2c_id_t id, uint16_t dev_address, uint8_t *p_tdata, uint16_t tsize, uint8_t *p_rdata, uint16_t rsize, uint32_t timeout);
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


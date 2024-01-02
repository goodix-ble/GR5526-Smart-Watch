/**
 ****************************************************************************************
 *
 * @file    app_i2s.h
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


#ifndef _APP_I2S_H_
#define _APP_I2S_H_

#include <stdbool.h>
#include "grx_hal.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_drv_error.h"
#include "app_drv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_I2S_MODULE_ENABLED

/** @addtogroup APP_I2S_ENUM Enumerations
  * @{
  */

/**
  * @brief I2S module Enumerations definition
  */
typedef enum
{
    APP_I2S_ID_SLAVE,                /**< I2S slave module. */
    APP_I2S_ID_MASTER,               /**< I2S master module. */
    APP_I2S_ID_MAX                   /**< Only for check parameter, not used as input parameters. */
} app_i2s_id_t;

/**
  * @brief I2S event Enumerations definition
  */
typedef enum
{
    APP_I2S_EVT_ERROR,                  /**< Error reported by UART peripheral. */
    APP_I2S_EVT_TX_CPLT,                /**< Requested TX transfer completed. */
    APP_I2S_EVT_RX_DATA,                /**< Requested RX transfer completed. */
    APP_I2S_EVT_TX_RX,                  /**< Requested TX/RX transfer completed. */
} app_i2s_evt_type_t;
/** @} */


/** @addtogroup APP_I2S_STRUCTURES Structures
  * @{
  */

/**
  * @brief I2S pins Structures
  */
typedef struct
{
    app_io_type_t        type;        /**< Specifies the type of I2S IO. */
    app_io_mux_t         mux;         /**< Specifies the Peripheral to be connected to the selected pins. */
    uint32_t             pin;         /**< Specifies the IO pins to be configured.
                                           This parameter can be any value of @ref GR5xxx_pins. */
    app_io_pull_t        pull;        /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
} app_i2s_pin_t;

/**
  * @brief I2S pins config Structures
  */
typedef struct
{
    app_i2s_pin_t       ws;          /**< Set the configuration of I2S WS pin. */
    app_i2s_pin_t       sdo;         /**< Set the configuration of I2S SDO pin. */
    app_i2s_pin_t       sdi;         /**< Set the configuration of I2S SDI pin. */
    app_i2s_pin_t       sclk;        /**< Set the configuration of I2S SCLK pin. */
} app_i2s_pin_cfg_t;

/**
  * @brief I2S operate mode Enumerations definition
  */
typedef struct
{
    dma_regs_t          *tx_dma_instance;/**< Specifies the TX DMA instance.*/
    dma_regs_t          *rx_dma_instance;/**< Specifies the RX DMA instance.*/
    dma_channel_t       tx_dma_channel; /**< Specifies the dma channel of I2S TX. */
    dma_channel_t       rx_dma_channel; /**< Specifies the dma channel of I2S RX. */
} app_i2s_dma_cfg_t;

/**
  * @brief I2S event structure definition
  */
typedef struct
{
    app_i2s_evt_type_t  type;           /**< Type of event. */
    union
    {
        uint32_t error_code;            /**< I2S Error code . */
        uint16_t size;                  /**< I2S transmitted/received counter. */
    } data;                             /**< Data of event. */
} app_i2s_evt_t;

/** @} */

/** @addtogroup APP_I2S_ENUM Enumerations
  * @{
  */
/**@brief App i2s state types. */
typedef enum
{
    APP_I2S_INVALID = 0,
    APP_I2S_ACTIVITY,
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    APP_I2S_SLEEP,
#endif
} app_i2s_state_t;


/**@brief App i2s dma state types. */
typedef enum
{
    APP_I2S_DMA_INVALID = 0,
    APP_I2S_DMA_ACTIVITY,
} app_i2s_dma_state_t;

/** @} */

/** @addtogroup APP_I2S_TYPEDEFS Type definitions
  * @{
  */
/**
  * @brief I2S event callback definition
  */
typedef void (*app_i2s_evt_handler_t)(app_i2s_evt_t *p_evt);

/** @} */

/** @addtogroup APP_I2S_STRUCTURES Structures
  * @{
  */
/**
  * @brief I2S device structure definition
  */
typedef struct
{
    app_i2s_evt_handler_t   evt_handler;       /**< I2S event callback definition. */
    i2s_handle_t            handle;            /**< I2S handle definition. */
    app_i2s_pin_cfg_t       *p_pin_cfg;        /**< I2S pins config Structures. */
    dma_id_t                dma_id[2];         /**< DMA id definition. */
    app_i2s_state_t         i2s_state;         /**< I2S state types. */
    app_i2s_dma_state_t     i2s_dma_state;     /**< I2S dma state types. */
    bool                    start_flag;        /**< Start flag definition. */
} i2s_env_t;

/**
  * @brief I2S parameters structure definition
  */
typedef struct
{
    app_i2s_id_t        id;             /**< specified I2S module ID. */
    app_i2s_pin_cfg_t   pin_cfg;        /**< the pin configuration information for the specified I2S module. */
    app_i2s_dma_cfg_t   dma_cfg;        /**< I2S operate mode Enumerations definition. */
    i2s_init_t          init;           /**< I2S communication parameters. */
    i2s_env_t           i2s_env;        /**< I2S device structure definition. */
} app_i2s_params_t;

/** @} */

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
 * @param[in]  evt_handler: I2S user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_i2s_init(app_i2s_params_t *p_params, app_i2s_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP I2S DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_i2s_deinit(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief  Receive in master or slave mode an amount of data in blocking mode.
 *
 * @param[in]  id: which I2S module want to receive.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_receive_sync(app_i2s_id_t id, uint16_t *p_data, uint16_t size, uint32_t timeout);

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
uint16_t app_i2s_receive_async(app_i2s_id_t id, uint16_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmits in master or slave mode an amount of data in blocking mode.
 *
 * @param[in]  id: which I2S module want to transmit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_transmit_sync(app_i2s_id_t id, uint16_t *p_data, uint16_t size, uint32_t timeout);

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
uint16_t app_i2s_transmit_async(app_i2s_id_t id, uint16_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Enable the I2S module.
 *
 * @param[in]  id: The I2S module id.
 *
 * @return Result of operation.
 ****************************************************************************************
 */

uint16_t app_i2s_enable(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief  Disable the I2S module.
 *
 * @param[in]  id: The I2S module id.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_disable(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief  Enable the master I2S clock.
 *
 * @param[in]  id: The I2S master module id.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_enable_clock(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief  Disable the master I2S clock.
 *
 * @param[in]  id: The I2S master module id.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_disable_clock(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief  Flush the I2S transmitter FIFO.
 *
 * @param[in]  id: which I2S module want to flush.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_flush_tx_fifo(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief  Flush the I2S receiver FIFO.
 *
 * @param[in]  id: which I2S module want to flush.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_flush_rx_fifo(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief  Return the I2S handle.
 *
 * @param[in]  id: I2S Channel ID.
 *
 * @return Pointer to the specified ID's I2S handle.
 ****************************************************************************************
 */
i2s_handle_t *app_i2s_get_handle(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief Abort ongoing transfer (blocking mode).
 * @param id I2S ID
 * @note This procedure is executed in blocking mode: When exiting
 * function, Abort is considered as completed.
 * @retval ::APP_DRV_SUCCESS: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 * @retval ::APP_DRV_ERR_INVALID_PARAM: Parameter error.
 ****************************************************************************************
 */
uint16_t app_i2s_abort(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief Transmit and Receive an amount of data in blocking mode.
 * @param id I2S ID
 * @param p_tx_data Pointer to transmission data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  length: Amount of data to be sent and received in bytes
 * @param[in]  timeout: Timeout duration
 * @retval ::APP_DRV_SUCCESS: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 * @retval ::APP_DRV_ERR_INVALID_PARAM: Parameter error.
 * @retval ::APP_DRV_ERR_BUSY: Driver is busy.
 ****************************************************************************************
 */
uint16_t app_i2s_transmit_receive_sync(app_i2s_id_t id,
                                       uint16_t *p_tx_data,
                                       uint16_t *p_rx_data,
                                       uint32_t length,
                                       uint32_t timeout);
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
uint16_t app_i2s_transmit_receive_async(app_i2s_id_t id,
                                        uint16_t     *p_tx_data,
                                        uint16_t     *p_rx_data,
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



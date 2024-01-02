/**
 ****************************************************************************************
 *
 * @file    app_dspi.h
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
#ifndef _APP_DSPI_H_
#define _APP_DSPI_H_

#include "grx_hal.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_drv_error.h"
#include "app_drv_config.h"

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



#ifdef HAL_DSPI_MODULE_ENABLED

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup APP_DSPI_DEFINE Defines
  * @{
  */

#define APP_DSPI_PIN_ENABLE      1    /**< DSPI pin enable  */
#define APP_DSPI_PIN_DISABLE     0    /**< DSPI pin disable */

/** @} */

/** @addtogroup APP_DSPI_ENUM Enumerations
  * @{
  */

/** @brief DSPI event Enumerations definition
  */
typedef enum
{
    APP_DSPI_EVT_ERROR,          /**< Error reported by DSPI peripheral. */
    APP_DSPI_EVT_TX_CPLT,        /**< Requested TX transfer completed. */
    APP_DSPI_EVT_ABORT,          /**< Requested abort transfer completed.*/
} app_dspi_evt_type_t;
/** @} */

/** @addtogroup APP_DSPI_STRUCTURES Structures
  * @{
  */

/**
  * @brief DSPI IO configuration Structures
  */
typedef struct
{
    app_io_type_t  type;         /**< Specifies the type of DSPI IO. */
    app_io_mux_t   mux;          /**< Specifies the Peripheral to be connected to the selected pins. */
    uint32_t       pin;          /**< Specifies the IO pins to be configured.
                                      This parameter can be any value of @ref GR5xxx_pins. */
    app_io_pull_t  pull;         /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
    uint8_t        enable;       /**< Enable or disable the pin. */
} app_dspi_pin_t;

/**
  * @brief DSPI Transmission IO configuration Structures
  */
typedef struct
{
    app_dspi_pin_t cs;           /**< Set the configuration of DSPI CS pin. */
    app_dspi_pin_t clk;          /**< Set the configuration of DSPI CLK pin. */
    app_dspi_pin_t mosi;         /**< Set the configuration of DSPI MOSI pin. */
    app_dspi_pin_t miso;         /**< Set the configuration of DSPI MISO pin. */
    app_dspi_pin_t dcx;          /**< Set the configuration of DSPI DCX pin. */
} app_dspi_pin_cfg_t;

/**
  * @brief DSPI DMA configuration Structures
  */
typedef struct
{
    dma_channel_t       channel; /**< Specifies the dma channel of DSPI, and it uses DMA1 by default. */
} app_dspi_dma_cfg_t;

/**
  * @brief DSPI event structure definition
  */
typedef struct
{
    app_dspi_evt_type_t type;       /**< DSPI type of event. */
    union
    {
        uint32_t error_code;        /**< DSPI Error code. */
        uint16_t size;              /**< DSPI transmitted counter. */
    } data;                         /**< DSPI Transmission status data. */
} app_dspi_evt_t;

/** @} */

/** @addtogroup APP_DSPI_TYPEDEFS Type Definitions
  * @{
  */
/**
  * @brief DSPI command structure definition
  */
typedef dspi_command_t app_dspi_command_t;

/**
  * @brief DSPI event callback definition
  */
typedef void (*app_dspi_evt_handler_t)(app_dspi_evt_t *p_evt);
/** @} */

/** @addtogroup APP_DSPI_ENUM Enumerations
  * @{
  */
/**@brief App dspi state types. */
typedef enum
{
    APP_DSPI_INVALID = 0,
    APP_DSPI_ACTIVITY,
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    APP_DSPI_SLEEP,
#endif
} app_dspi_state_t;

/**@brief App dspi dma state types. */
typedef enum
{
    APP_DSPI_DMA_INVALID = 0,
    APP_DSPI_DMA_ACTIVITY,
} app_dspi_dma_state_t;

/** @} */

/** @addtogroup APP_DSPI_STRUCTURES Structures
  * @{
  */

typedef struct
{
    app_dspi_evt_handler_t  evt_handler;           /**< DSPI event callback. */
    dspi_handle_t           handle;                /**< DSPI handle Structure. */
    app_dspi_pin_cfg_t      *p_pin_cfg;            /**< DSPI Transmission IO configuration Structures. */
    dma_id_t                dma_id;                /**< DMA id. */
    app_dspi_state_t        dspi_state;            /**< App dspi state types. */
    app_dspi_dma_state_t    dspi_dma_state;        /**< App dspi dma state types. */

    bool                    start_flag;            /**< start flag. */
    volatile bool           is_soft_cs;            /**<   soft cs. */
} dspi_env_t;

/**
  * @brief DSPI parameters structure definition
  */
typedef struct
{
    app_dspi_pin_cfg_t  pin_cfg;        /**< the pin configuration information for the specified DSPI module. */
    app_dspi_dma_cfg_t  dma_cfg;        /**< DSPI DMA configuration. */
    dspi_init_t         init;           /**< DSPI communication parameters. */
    bool                is_soft_cs;     /**< config whether to control CS signal by software. */
    dspi_env_t          dspi_env;       /**< APP_DSPI_STRUCTURES Structures. */
} app_dspi_params_t;

/** @} */

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
 * @param[in]  evt_handler: DSPI user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_dspi_init(app_dspi_params_t *p_params, app_dspi_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP DSPI DRIVER peripheral.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_dspi_deinit(void);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction in blocking mode.
 *
 * @param[in]  p_cmd: Pointer to a app_dspi_command_t structure that contains the instruction, length, data_size for data transfer.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_command_transmit_sync(app_dspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout);

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
uint16_t app_dspi_command_transmit_async(app_dspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Transmit only instruction in blocking mode.
 *
 * @param[in]  p_cmd: Pointer to a app_dspi_command_t structure that contains the instruction for data transfer.
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_command_sync(app_dspi_command_t *p_cmd, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit instruction in non-blocking mode with Interrupt.
 *
 * @param[in]  p_cmd: Pointer to a app_dspi_command_t structure that contains the instruction for data transfer.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_command_async(app_dspi_command_t *p_cmd);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in blocking mode.
 *
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_transmit_sync(uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief Transmit an amount of data in non-blocking mode with Interrupt.
 *
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_transmit_async(uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Return the DSPI handle.
 *
 * @return Pointer to the DSPI handle.
 ****************************************************************************************
 */
dspi_handle_t *app_dspi_get_handle(void);

/**
 ****************************************************************************************
 * @brief  Set the DSPI transmission mode.
 *
 * @param[in]  mode: the DSPI transmission mode.This parameter can be one of the following values:
 *         @arg @ref DSPI_PROT_MODE_3W1L
 *         @arg @ref DSPI_PROT_MODE_4W1L
 *         @arg @ref DSPI_PROT_MODE_4W2L
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_config_mode(uint32_t mode);

/**
 ****************************************************************************************
 * @brief  Set the DSPI transmission data size.
 *
 * @param[in]  data_size: the DSPI transmission data size.This parameter can be one of the following values:
 *         @arg @ref DSPI_DATASIZE_04_BITS
 *         @arg @ref DSPI_DATASIZE_05_BITS
 *         @arg @ref DSPI_DATASIZE_06_BITS
 *         @arg @ref DSPI_DATASIZE_07_BITS
 *         @arg @ref DSPI_DATASIZE_08_BITS
 *         @arg @ref DSPI_DATASIZE_09_BITS
 *         @arg @ref DSPI_DATASIZE_10_BITS
 *         @arg @ref DSPI_DATASIZE_11_BITS
 *         @arg @ref DSPI_DATASIZE_12_BITS
 *         @arg @ref DSPI_DATASIZE_13_BITS
 *         @arg @ref DSPI_DATASIZE_14_BITS
 *         @arg @ref DSPI_DATASIZE_15_BITS
 *         @arg @ref DSPI_DATASIZE_16_BITS
 *         @arg @ref DSPI_DATASIZE_17_BITS
 *         @arg @ref DSPI_DATASIZE_18_BITS
 *         @arg @ref DSPI_DATASIZE_19_BITS
 *         @arg @ref DSPI_DATASIZE_20_BITS
 *         @arg @ref DSPI_DATASIZE_21_BITS
 *         @arg @ref DSPI_DATASIZE_22_BITS
 *         @arg @ref DSPI_DATASIZE_23_BITS
 *         @arg @ref DSPI_DATASIZE_24_BITS
 *         @arg @ref DSPI_DATASIZE_25_BITS
 *         @arg @ref DSPI_DATASIZE_26_BITS
 *         @arg @ref DSPI_DATASIZE_27_BITS
 *         @arg @ref DSPI_DATASIZE_28_BITS
 *         @arg @ref DSPI_DATASIZE_29_BITS
 *         @arg @ref DSPI_DATASIZE_30_BITS
 *         @arg @ref DSPI_DATASIZE_31_BITS
 *         @arg @ref DSPI_DATASIZE_32_BITS
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_config_data_size(uint32_t data_size);

/**
 ****************************************************************************************
 * @brief  Abort the current transmission.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_abort(void);

/**
 ****************************************************************************************
 * @brief  Abort the current transmission (non-blocking function)
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_dspi_abort_it(void);

/** @} */

#ifdef __cplusplus
}
#endif


#endif //END #ifdef HAL_DSPI_MODULE_ENABLED
#endif //END #ifndef _APP_DSPI_H_


/** @} */

/** @} */

/** @} */

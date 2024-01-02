/**
 ****************************************************************************************
 *
 * @file    app_adc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of ADC app library.
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

/** @defgroup APP_ADC ADC
  * @brief ADC APP module driver.
  * @{
  */


#ifndef _APP_ADC_H_
#define _APP_ADC_H_

#include "grx_hal.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_drv_error.h"
#include "app_drv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_ADC_MODULE_ENABLED

/** @addtogroup APP_ADC_ENUM Enumerations
  * @{
  */

/**
  * @brief ADC event Enumerations definition
  */
typedef enum
{
    APP_ADC_EVT_CONV_CPLT,           /**< Conversion completed by ADC peripheral. */
} app_adc_evt_type_t;

/**@brief App adc state types. */
typedef enum
{
    APP_ADC_INVALID = 0,
    APP_ADC_ACTIVITY,
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    APP_ADC_SLEEP,
#endif
} app_adc_state_t;

/**@brief App adc dma state types. */
typedef enum
{
    APP_ADC_DMA_INVALID = 0,
    APP_ADC_DMA_ACTIVITY,
} app_adc_dma_state_t;

/** @} */


/** @addtogroup APP_ADC_STRUCTURES Structures
  * @{
  */

/**
  * @brief ADC pins Structures
  */
typedef struct
{
   app_io_type_t        type;        /**< Specifies the type of ADC IO. */
   app_io_mux_t         mux;         /**< Specifies the Peripheral to be connected to the selected pins. */
   uint32_t             pin;         /**< Specifies the IO pins to be configured.
                                          This parameter can be any value of @ref GR5xxx_pins. */
} app_adc_pin_t;

/**
  * @brief ADC pins config Structures
  */
typedef struct
{
    app_adc_pin_t       channel_p;   /**< Set the configuration of ADC pin. */
    app_adc_pin_t       channel_n;   /**< Set the configuration of ADC pin. */
    app_adc_pin_t       extern_ref;  /**< Set the configuration of extern reference pin. */
} app_adc_pin_cfg_t;

/**
  * @brief ADC DMA configuration Structures
  */
typedef struct
{
    dma_regs_t *        dma_instance; /**< Specifies the dma instance of ADC. */
    dma_channel_t       dma_channel;  /**< Specifies the dma channel of ADC. */
} app_adc_dma_cfg_t;

/**
  * @brief ADC event structure definition
  */
typedef struct
{
    app_adc_evt_type_t  type; /**< Type of event. */
} app_adc_evt_t;

/** @} */

/** @addtogroup APP_ADC_TYPEDEFS Type definitions
  * @{
  */
/**
  * @brief ADC event callback definition
  */
typedef void (*app_adc_evt_handler_t)(app_adc_evt_t *p_evt);

/** @} */

/** @addtogroup APP_ADC_STRUCTURES Structures
  * @{
  */

/**
  * @brief ADC sample-node definition
  */
typedef struct link_node
{
    uint32_t          channel;     /**< Which channel to be sample; This parameter can be any value of ADC_INPUT_SOURCE:ADC_INPUT_SRC_IO0~ADC_INPUT_SRC_REF*/
    uint16_t          *p_buf;     /**< Buffer pointer of current channel sample codes. Note:pointer must be 4-byte aligned as it is filled in datas by DMA */
    uint32_t          len;        /**< Sample len codes on current channel. */
    struct link_node *next;      /**< Point to the next sample node. */
} app_adc_sample_node_t;

/**
  * @brief ADC device structure definition
  */
typedef struct
{
    app_adc_evt_handler_t   evt_handler;               /**< ADC event callback definition. */
    adc_handle_t            handle;                    /**< ADC handle definition. */
    dma_id_t                dma_id;                    /**< DMA id definition . */
    app_adc_state_t         adc_state;                 /**< ADC state types. */
    app_adc_dma_state_t     adc_dma_state;             /**< ADC dma state types. */
    app_adc_sample_node_t *p_current_sample_node;      /**< ADC sample-node definition. */
    uint32_t multi_channel;                            /**< multi channel definition. */
} adc_env_t;

/**
  * @brief ADC parameters structure definition
  */
typedef struct
{
    app_adc_pin_cfg_t   pin_cfg;     /**< the pin configuration information for the specified ADC module. */
    app_adc_dma_cfg_t   dma_cfg;     /**< ADC DMA configuration. */
    adc_init_t          init;        /**< ADC configuration parameters. */
    adc_env_t           adc_env;     /**< ADC device structure definition. */
} app_adc_params_t;

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_ADC_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the APP ADC DRIVER according to the specified parameters
 *         in the app_adc_params_t and app_adc_evt_handler_t.
 * @note   If DMA mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use DMA mode.
 *
 * @param[in]  p_params: Pointer to app_adc_params_t parameter which contains the
 *                       configuration information for the specified ADC module.
 * @param[in]  evt_handler: ADC user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_adc_init(app_adc_params_t *p_params, app_adc_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP ADC DRIVER peripheral.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_adc_deinit(void);

/**
 ****************************************************************************************
 * @brief  Polling for conversion.
 *
 * @param[in]  p_data: Pointer to data buffer which to storage ADC conversion results.
 * @param[in]  length: Length of data buffer.
 * @param[in]  timeout : Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_adc_conversion_sync(uint16_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  DMA for conversion.
 *
 * @param[in]  p_data: Pointer to data buffer which to storage ADC conversion results.
 * @param[in]  length: Length of data buffer,  ranging between 0 and 4095.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_adc_conversion_async(uint16_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Convert the ADC conversion results to a voltage value(internal reference).
 *
 * @param[in]  inbuf: Pointer to data buffer which storage ADC conversion results.
 * @param[out] outbuf: Pointer to data buffer which to storage voltage results.
 * @param[in]  buflen: Length of data buffer.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_adc_voltage_intern(uint16_t *inbuf, double *outbuf, uint32_t buflen);

/**
 ****************************************************************************************
 * @brief  Convert the ADC conversion results to a voltage value(external reference).
 *
 * @param[in]  ref: slope of ADC.
 * @param[in]  inbuf: Pointer to data buffer which storage ADC conversion results.
 * @param[out] outbuf: Pointer to data buffer which to storage voltage results.
 * @param[in]  buflen: Length of data buffer.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_adc_voltage_extern(double ref, uint16_t *inbuf, double *outbuf, uint32_t buflen);

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
/**
 ****************************************************************************************
 * @brief  Convert the ADC conversion results to temperature value.
 *
 * @param[in]  inbuf: Pointer to data buffer which storage ADC conversion results.
 * @param[out] outbuf: Pointer to data buffer which to storage voltage results.
 * @param[in]  buflen: Length of data buffer.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_adc_temperature_conv(uint16_t *inbuf, double *outbuf, uint32_t buflen);

/**
 ****************************************************************************************
 * @brief  Convert the ADC conversion results to battery value.
 *
 * @param[in]  inbuf: Pointer to data buffer which storage ADC codes.
 * @param[out] outbuf: Pointer to data buffer which to storage conversion results.
 * @param[in]  buflen: Length of data buffer.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_adc_vbat_conv(uint16_t *inbuf, double *outbuf, uint32_t buflen);
#endif

/**
 ****************************************************************************************
 * @brief  Return the ADC handle.
 *
 * @return Pointer to the ADC handle.
 ****************************************************************************************
 */
adc_handle_t *app_adc_get_handle(void);

/**
 ****************************************************************************************
 * @brief  DMA for multi channels conversion; evt_handler in app_adc_init will callback when all channels finish.
 *
 * @param[in]  p_begin_node: Pointer to the multi sample channels list node.
 * @param[in]  total_nodes: total sample channels.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_adc_multi_channel_conversion_async(app_adc_sample_node_t *p_begin_node, uint32_t total_nodes);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
/**
 ****************************************************************************************
 * @brief  Clear the ADC FIFO, filter the first two incorrect code values , and start the ADC clock.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_adc_clock_start(void);

/**
 ****************************************************************************************
 * @brief  Get the ADC sampling code by polling, and convert sampling code of the specified length to the average voltage.
 *
 * @param[out] avg_voltage: Pointer to data which to storage average voltage results.
 * @param[in]  length: Length of the ADC sample values to be averaged.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_adc_get_avg_voltage(float *avg_voltage, uint32_t length);

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


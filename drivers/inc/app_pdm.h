/**
 ****************************************************************************************
 *
 * @file    app_pdm.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PDM app library.
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

#ifndef _APP_PDM_H_
#define _APP_PDM_H_

#include "grx_hal.h"
#include "app_drv_error.h"
#include "app_drv_config.h"
#include "app_io.h"
#include "app_dma.h"

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_PDM PDM
  * @brief PDM APP module driver.
  * @{
  */

#ifdef HAL_PDM_MODULE_ENABLED

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup APP_PDM_ENUMS Enumerations
  * @{
  */

/**
  * @brief PDM event Enumerations definition
  */
typedef enum
{
    APP_PDM_EVT_DMA_ERROR,              /**< The event of dma error interrupt. */
    APP_PDM_EVT_DMA_TFR,                /**< The event of transfer complete interrupt. */
    APP_PDM_EVT_DMA_BLK,                /**< The event of dma block interrupt. */
    APP_PDM_EVT_LEFT_OVERFLOW,          /**< The event of left channel overflow. */
    APP_PDM_EVT_RIGHT_OVERFLOW,         /**< The event of right channel overflow. */
} app_pdm_evt_type_t;

/** @} */

/** @addtogroup APP_PDM_STRUCTURES Structures
  * @{
  */

/**
  * @brief PDM IO configuration Structures
  */
typedef struct
{
    app_io_type_t      type;          /**< Specifies the type of PDM IO. */
    app_io_mux_t       mux;           /**< Specifies the Peripheral to be connected to the selected pins. */
    uint32_t           pin;           /**< Specifies the IO pins to be configured. */
    app_io_pull_t      pull;          /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
}app_pdm_pin_t;

/**
  * @brief PDM IO Structures
  */
typedef struct
{
    app_pdm_pin_t      clk;           /**< Set the configuration of PDM CLK pin. */
    app_pdm_pin_t      data;          /**< Set the configuration of PDM DATA pin. */
}app_pdm_pin_cfg_t;

/**
  * @brief PDM operate mode Enumerations definition
  */
typedef struct
{
    dma_regs_t        *dma_instance;  /**< Specifies the DMA instance. */
    dma_channel_t     dma_channel;    /**< Specifies the channel of PDM. */
}app_pdm_dma_t;

/**
  * @brief PDM event structure definition
  */
typedef struct
{
    app_pdm_evt_type_t type;         /**< Type of event. */

    uint32_t error_code;             /**< PDM error code . */
} app_pdm_evt_t;

/** @} */

/** @addtogroup APP_PDM_TYPEDEFS Type definitions
  * @{
  */

/**
  * @brief PDM event callback definition
  */
typedef void (*app_pdm_evt_handler_t)(app_pdm_evt_t *type);

/** @} */

/** @addtogroup APP_PDM_ENUMS Enumerations
  * @{
  */

/**
  * @brief pdm state types.
  */
typedef enum
{
    APP_PDM_INACTIVITY = 0,
    APP_PDM_ACTIVITY,
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    APP_PDM_SLEEP,
#endif
} app_pdm_state_t;

/** @} */

/** @addtogroup APP_PDM_STRUCTURES Structures
  * @{
  */

/**
  * @brief app pdm instance structure.
  */
typedef struct
{
    app_pdm_evt_handler_t  evt_handler;     /**< Pdm event callback. */
    pdm_handle_t           handle;          /**< Pdm handle Structure. */
    app_pdm_pin_cfg_t      *p_pin_cfg;      /**< Pdm IO Structures. */
    dma_id_t               dma_id;          /**< Dma id. */
    app_pdm_state_t        pdm_state;       /**< Pdm state types. */
} pdm_env_t;

/**
  * @brief PDM parameters structure definition
  */
typedef struct
{
    app_pdm_pin_cfg_t   pin_cfg;       /**< The pin configuration information .*/
    app_pdm_dma_t       dma_cfg;       /**< The dma configuration information. */
    pdm_init_t          init;          /**< The pdm configuration information. */
    pdm_env_t           pdm_env;       /**< App pdm instance structure. */
} app_pdm_params_t;

/** @} */
/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_PDM_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP PDM DRIVER according to the specified parameters
 *         in the app_pdm_params_t and app_pdm_evt_handler_t.
 *
 * @param[in]  p_params: Pointer to app_pdm_params_t parameter which contains the
 *                       configuration information for the specified PDM module.
 * @param[in]  evt_handler: PDM user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_pdm_init(app_pdm_params_t *p_params, app_pdm_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP PDM DRIVER peripheral.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_pdm_deinit(void);

/**
 ****************************************************************************************
 * @brief  Start the pdm left channel transfer with dma.
 *
 * @param[in] p_data: Point to the data buffer.
 * @param[in] length: The length of data,ranging between 0 and 4095.
 ****************************************************************************************
 */
uint16_t app_pdm_left_start_dma(uint16_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Start the pdm right channel transfer with dma.
 *
 * @param[in] p_data: Point to the data buffer.
 * @param[in] length: The length of data,ranging between 0 and 4095.
 ****************************************************************************************
 */
uint16_t app_pdm_right_start_dma(uint16_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Start the pdm dual channel transfer with dma.
 *
 * @param[in] p_data: Point to the data buffer.
 * @param[in] length: The length of data,ranging between 0 and 4095.
 ****************************************************************************************
 */
uint16_t app_pdm_stereo_start_dma(uint32_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Start the pdm left channel transfer with dma's sg and llp functions.
 *
 * @param[in] p_data: Point to the data buffer.
 * @param[in] length: The length of data,ranging between 0 and 4095.
 * @param[in]  sg_llp_config: The config of source and destination's sg and llp fuction.
 ****************************************************************************************
 */
uint16_t app_pdm_left_start_dma_sg_llp(uint16_t *p_data, uint32_t length, dma_sg_llp_config_t *sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Start the pdm right channel transfer with dma's sg and llp functions.
 *
 * @param[in] p_data: Point to the data buffer.
 * @param[in] length: The length of data,ranging between 0 and 4095.
 * @param[in]  sg_llp_config: The config of source and destination's sg and llp fuction.
 ****************************************************************************************
 */
uint16_t app_pdm_right_start_dma_sg_llp(uint16_t *p_data, uint32_t length, dma_sg_llp_config_t *sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Start the pdm dual channel transfer with dma's sg and llp functions.
 *
 * @param[in] p_data: Point to the data buffer.
 * @param[in] length: The length of data,ranging between 0 and 4095.
 * @param[in]  sg_llp_config: The config of source and destination's sg and llp fuction.
 ****************************************************************************************
 */
uint16_t app_pdm_stereo_start_dma_sg_llp(uint32_t *p_data, uint32_t length, dma_sg_llp_config_t *sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Abort the pdm transfer.
 *
 * @return Result of abort.
 ****************************************************************************************
 */
uint16_t app_pdm_abort(void);

/**
 ****************************************************************************************
 * @brief  Return the PDM handle.
 *
 * @return Pointer to the specified PDM handle.
 ****************************************************************************************
 */
pdm_handle_t *app_pdm_get_handle(void);

/** @} */



#ifdef __cplusplus
}
#endif


#endif //#ifdef HAL_PDM_MODULE_ENABLED
#endif //#ifndef _APP_PDM_H_

/** @} */

/** @} */

/** @} */

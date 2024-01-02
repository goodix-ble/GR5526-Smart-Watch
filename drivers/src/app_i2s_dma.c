/**
  ****************************************************************************************
  * @file    app_i2s_dma.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_i2s_dma.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include <string.h>

#ifdef HAL_I2S_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
extern void i2s_wake_up(app_i2s_id_t id);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern i2s_env_t *p_i2s_env[APP_I2S_ID_MAX];

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint16_t app_i2s_config_dma_tx(app_i2s_params_t *p_params)
{
    app_dma_params_t tx_dma_params = { 0 };

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    if (p_params->dma_cfg.tx_dma_instance == DMA0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
#endif

    tx_dma_params.p_instance                 = p_params->dma_cfg.tx_dma_instance;
    tx_dma_params.channel_number             = p_params->dma_cfg.tx_dma_channel;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    tx_dma_params.init.src_request           = DMA1_REQUEST_MEM;
    tx_dma_params.init.dst_request           = (p_params->id == APP_I2S_ID_SLAVE) ? DMA1_REQUEST_I2S_S_TX : DMA1_REQUEST_I2S_M_TX;
#else
    tx_dma_params.init.src_request           = DMA0_REQUEST_MEM;
    tx_dma_params.init.dst_request           = (p_params->id == APP_I2S_ID_SLAVE) ? DMA0_REQUEST_I2S_S_TX : DMA0_REQUEST_I2S_M_TX;
#endif

    tx_dma_params.init.direction             = DMA_MEMORY_TO_PERIPH;
    tx_dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    tx_dma_params.init.dst_increment         = DMA_DST_NO_CHANGE;
    if (p_params->init.data_size <= I2S_DATASIZE_16BIT)
    {
        tx_dma_params.init.src_data_alignment = DMA_SDATAALIGN_HALFWORD;
        tx_dma_params.init.dst_data_alignment = DMA_DDATAALIGN_HALFWORD;
    }
    else
    {
        tx_dma_params.init.src_data_alignment = DMA_SDATAALIGN_WORD;
        tx_dma_params.init.dst_data_alignment = DMA_DDATAALIGN_WORD;
    }
    tx_dma_params.init.mode                  = DMA_NORMAL;
    tx_dma_params.init.priority              = DMA_PRIORITY_LOW;

    p_i2s_env[p_params->id]->dma_id[0] = app_dma_init(&tx_dma_params, NULL);
    if (p_i2s_env[p_params->id]->dma_id[0] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    p_i2s_env[p_params->id]->handle.p_dmatx = app_dma_get_handle(p_i2s_env[p_params->id]->dma_id[0]);
    p_i2s_env[p_params->id]->handle.p_dmatx->p_parent = (void*)&p_i2s_env[p_params->id]->handle;

    return APP_DRV_SUCCESS;
}

static uint16_t app_i2s_config_dma_rx(app_i2s_params_t *p_params)
{
    app_dma_params_t rx_dma_params = { 0 };

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    if (p_params->dma_cfg.rx_dma_instance == DMA0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
#endif

    rx_dma_params.p_instance                 = p_params->dma_cfg.rx_dma_instance;
    rx_dma_params.channel_number             = p_params->dma_cfg.rx_dma_channel;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    rx_dma_params.init.src_request           = (p_params->id == APP_I2S_ID_SLAVE) ? DMA1_REQUEST_I2S_S_RX : DMA1_REQUEST_I2S_M_RX;
    rx_dma_params.init.dst_request           = DMA1_REQUEST_MEM;
#else
    rx_dma_params.init.src_request           = (p_params->id == APP_I2S_ID_SLAVE) ? DMA0_REQUEST_I2S_S_RX : DMA0_REQUEST_I2S_M_RX;
    rx_dma_params.init.dst_request           = DMA0_REQUEST_MEM;
#endif
    rx_dma_params.init.direction             = DMA_PERIPH_TO_MEMORY;
    rx_dma_params.init.src_increment         = DMA_SRC_NO_CHANGE;
    rx_dma_params.init.dst_increment         = DMA_DST_INCREMENT;
    if (p_params->init.data_size <= I2S_DATASIZE_16BIT)
    {
        rx_dma_params.init.src_data_alignment = DMA_SDATAALIGN_HALFWORD;
        rx_dma_params.init.dst_data_alignment = DMA_DDATAALIGN_HALFWORD;
    }
    else
    {
        rx_dma_params.init.src_data_alignment = DMA_SDATAALIGN_WORD;
        rx_dma_params.init.dst_data_alignment = DMA_DDATAALIGN_WORD;
    }
    rx_dma_params.init.mode                  = DMA_NORMAL;
    rx_dma_params.init.priority              = DMA_PRIORITY_LOW;

    p_i2s_env[p_params->id]->dma_id[1] = app_dma_init(&rx_dma_params, NULL);
    if (p_i2s_env[p_params->id]->dma_id[1] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    p_i2s_env[p_params->id]->handle.p_dmarx = app_dma_get_handle(p_i2s_env[p_params->id]->dma_id[1]);
    p_i2s_env[p_params->id]->handle.p_dmarx->p_parent = (void*)&p_i2s_env[p_params->id]->handle;

    return APP_DRV_SUCCESS;
}

static uint16_t app_i2s_config_dma(app_i2s_params_t *p_params)
{
    app_drv_err_t app_err_code = APP_DRV_SUCCESS;

    p_i2s_env[p_params->id]->dma_id[0] = -1;
    p_i2s_env[p_params->id]->dma_id[1] = -1;

    if (p_params->dma_cfg.tx_dma_instance == NULL &&
        p_params->dma_cfg.rx_dma_instance == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (p_params->dma_cfg.tx_dma_instance != NULL)
    {
        app_err_code = app_i2s_config_dma_tx(p_params);
        if (app_err_code != APP_DRV_SUCCESS)
        {
            return app_err_code;
        }
    }
    if (p_params->dma_cfg.rx_dma_instance != NULL)
    {
        app_err_code = app_i2s_config_dma_rx(p_params);
        if (app_err_code != APP_DRV_SUCCESS)
        {
            return app_err_code;
        }
    }

    return APP_DRV_SUCCESS;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_i2s_dma_init(app_i2s_params_t *p_params)
{
    app_i2s_id_t id = p_params->id;
    app_drv_err_t app_err_code;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if ((p_i2s_env[id] == NULL) || (p_i2s_env[id]->i2s_state == APP_I2S_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((p_i2s_env[id]->i2s_dma_state != APP_I2S_DMA_INVALID))
    {
        return APP_DRV_ERR_INVALID_INIT;
    }

    GLOBAL_EXCEPTION_DISABLE();
    app_err_code = app_i2s_config_dma(p_params);
    if (app_err_code != APP_DRV_SUCCESS)
    {
        goto __exit;
    }
    p_i2s_env[id]->i2s_dma_state = APP_I2S_DMA_ACTIVITY;
__exit:
    GLOBAL_EXCEPTION_ENABLE();

    return app_err_code;
}

uint16_t app_i2s_dma_deinit(app_i2s_id_t id)
{
    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2s_env[id] == NULL) ||
        (p_i2s_env[id]->i2s_dma_state != APP_I2S_DMA_ACTIVITY))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    app_dma_deinit(p_i2s_env[id]->dma_id[0]);
    app_dma_deinit(p_i2s_env[id]->dma_id[1]);

    p_i2s_env[id]->i2s_dma_state = APP_I2S_DMA_INVALID;

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_dma_receive_async(app_i2s_id_t id, uint16_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2s_env[id] == NULL) ||  (p_i2s_env[id]->i2s_state == APP_I2S_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    if (p_i2s_env[id]->start_flag == false)
    {
        p_i2s_env[id]->start_flag = true;
        err_code = hal_i2s_receive_dma(&p_i2s_env[id]->handle, p_data, size);
        if (err_code != HAL_OK)
        {
            p_i2s_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_dma_transmit_async(app_i2s_id_t id, uint16_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2s_env[id] == NULL) ||  (p_i2s_env[id]->i2s_state == APP_I2S_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    if (p_i2s_env[id]->start_flag == false)
    {
        p_i2s_env[id]->start_flag = true;
        err_code =  hal_i2s_transmit_dma(&p_i2s_env[id]->handle, p_data, size);
        if (err_code != HAL_OK)
        {
            p_i2s_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_dma_transmit_receive_async(app_i2s_id_t id,
                                            uint16_t    *p_tx_data,
                                            uint16_t    *p_rx_data,
                                            uint32_t     length)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2s_env[id] == NULL) ||  (p_i2s_env[id]->i2s_state == APP_I2S_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (NULL == p_tx_data || NULL == p_rx_data || length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    if (false == p_i2s_env[id]->start_flag)
    {
        p_i2s_env[id]->start_flag = true;
        err_code = hal_i2s_transmit_receive_dma(&p_i2s_env[id]->handle, p_tx_data, p_rx_data, length);
        if (err_code != HAL_OK)
        {
            p_i2s_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }
    return APP_DRV_SUCCESS;
}
#endif

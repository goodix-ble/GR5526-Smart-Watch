/**
  ****************************************************************************************
  * @file    app_i2c_dma.c
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
#include "app_i2c_dma.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include <string.h>

#ifdef HAL_I2C_MODULE_ENABLED
/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
struct dma_request {
    uint32_t tx;
    uint32_t rx;
};

typedef struct {
    struct dma_request dma0_request;
    struct dma_request dma1_request;
} i2c_dma_info_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const i2c_dma_info_t s_i2c_dma_info[APP_I2C_ID_MAX] = {
    {   /* I2C 0 */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        .dma0_request = {
            .tx = DMA0_REQUEST_I2C0_TX,
            .rx = DMA0_REQUEST_I2C0_RX,
        },
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        .dma1_request = {
            .tx = DMA1_REQUEST_I2C0_TX,
            .rx = DMA1_REQUEST_I2C0_RX,
        },
#endif
    },
    {   /* I2C 1 */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        .dma0_request = {
            .tx = DMA0_REQUEST_I2C1_TX,
            .rx = DMA0_REQUEST_I2C1_RX,
        },
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        .dma1_request = {
            .tx = DMA1_REQUEST_I2C1_TX,
            .rx = DMA1_REQUEST_I2C1_RX,
        },
#endif
    },
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    {   /* I2C 2 */
        .dma0_request = {
            .tx = DMA0_REQUEST_I2C2_TX,
            .rx = DMA0_REQUEST_I2C2_RX,
        },
    },
    {   /* I2C 3 */
        .dma0_request = {
            .tx = DMA0_REQUEST_I2C3_TX,
            .rx = DMA0_REQUEST_I2C3_RX,
        },
    },
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
    {   /* I2C 4 */
        .dma0_request = {
            .tx = DMA0_REQUEST_I2C4_TX,
            .rx = DMA0_REQUEST_I2C4_RX,
        },
    },
    {   /* I2C 5 */
        .dma0_request = {
            .tx = DMA0_REQUEST_I2C5_TX,
            .rx = DMA0_REQUEST_I2C5_RX,
        },
    },
#endif
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
extern void i2c_wake_up(app_i2c_id_t id);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern i2c_env_t *p_i2c_env[APP_I2C_ID_MAX];

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint16_t app_i2c_config_dma_tx(app_i2c_params_t *p_params)
{
    app_dma_params_t tx_dma_params = { 0 };

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    if (p_params->dma_cfg.tx_dma_instance == DMA0)
    {
        if (p_params->id < APP_I2C_ID_2)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }
    else if (p_params->dma_cfg.tx_dma_instance == DMA1)
    {
        if (p_params->id > APP_I2C_ID_1)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }
#endif

    tx_dma_params.p_instance                 = p_params->dma_cfg.tx_dma_instance;
    tx_dma_params.channel_number             = p_params->dma_cfg.tx_dma_channel;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    tx_dma_params.init.src_request           = DMA0_REQUEST_MEM;
    tx_dma_params.init.dst_request           = s_i2c_dma_info[p_params->id].dma0_request.tx;
#else
    if (tx_dma_params.p_instance == DMA0)
    {
        tx_dma_params.init.src_request = DMA0_REQUEST_MEM;
        tx_dma_params.init.dst_request = s_i2c_dma_info[p_params->id].dma0_request.tx;
    }
    else if (tx_dma_params.p_instance == DMA1)
    {
        tx_dma_params.init.src_request = DMA1_REQUEST_MEM;
        tx_dma_params.init.dst_request = s_i2c_dma_info[p_params->id].dma1_request.tx;
    }
#endif
    tx_dma_params.init.direction             = DMA_MEMORY_TO_PERIPH;
    tx_dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    tx_dma_params.init.dst_increment         = DMA_DST_NO_CHANGE;
    tx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
    tx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    tx_dma_params.init.mode                  = DMA_NORMAL;
#endif
    tx_dma_params.init.priority              = DMA_PRIORITY_LOW;

    p_i2c_env[p_params->id]->dma_id[0] = app_dma_init(&tx_dma_params, NULL);
    if (p_i2c_env[p_params->id]->dma_id[0] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    p_i2c_env[p_params->id]->handle.p_dmatx = app_dma_get_handle(p_i2c_env[p_params->id]->dma_id[0]);
    p_i2c_env[p_params->id]->handle.p_dmatx->p_parent = (void *)&p_i2c_env[p_params->id]->handle;

    return APP_DRV_SUCCESS;
}

static uint16_t app_i2c_config_dma_rx(app_i2c_params_t *p_params)
{
    app_dma_params_t rx_dma_params = { 0 };

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    if (p_params->dma_cfg.rx_dma_instance == DMA0)
    {
        if (p_params->id < APP_I2C_ID_2)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }
    else if (p_params->dma_cfg.rx_dma_instance == DMA1)
    {
        if (p_params->id > APP_I2C_ID_1)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }
#endif

    rx_dma_params.p_instance                 = p_params->dma_cfg.rx_dma_instance;
    rx_dma_params.channel_number             = p_params->dma_cfg.rx_dma_channel;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    rx_dma_params.init.src_request           = s_i2c_dma_info[p_params->id].dma0_request.rx;
    rx_dma_params.init.dst_request           = DMA0_REQUEST_MEM;
#else
    if (rx_dma_params.p_instance == DMA0)
    {
        rx_dma_params.init.src_request = s_i2c_dma_info[p_params->id].dma0_request.rx;
        rx_dma_params.init.dst_request = DMA0_REQUEST_MEM;
    }
    else if (rx_dma_params.p_instance == DMA1)
    {
        rx_dma_params.init.src_request = s_i2c_dma_info[p_params->id].dma1_request.rx;
        rx_dma_params.init.dst_request = DMA1_REQUEST_MEM;
    }
#endif
    rx_dma_params.init.direction             = DMA_PERIPH_TO_MEMORY;
    rx_dma_params.init.src_increment         = DMA_SRC_NO_CHANGE;
    rx_dma_params.init.dst_increment         = DMA_DST_INCREMENT;
    rx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
    rx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    rx_dma_params.init.mode                  = DMA_NORMAL;
#endif
    rx_dma_params.init.priority              = DMA_PRIORITY_LOW;

    p_i2c_env[p_params->id]->dma_id[1] = app_dma_init(&rx_dma_params, NULL);
    if (p_i2c_env[p_params->id]->dma_id[1] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    p_i2c_env[p_params->id]->handle.p_dmarx = app_dma_get_handle(p_i2c_env[p_params->id]->dma_id[1]);
    p_i2c_env[p_params->id]->handle.p_dmarx->p_parent = (void *)&p_i2c_env[p_params->id]->handle;

    return APP_DRV_SUCCESS;
}

static uint16_t app_i2c_config_dma(app_i2c_params_t *p_params)
{
    app_drv_err_t app_err_code = APP_DRV_SUCCESS;

    p_i2c_env[p_params->id]->dma_id[0] = -1;
    p_i2c_env[p_params->id]->dma_id[1] = -1;

    if (p_params->dma_cfg.tx_dma_instance == NULL &&
        p_params->dma_cfg.rx_dma_instance == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (p_params->dma_cfg.tx_dma_instance != NULL)
    {
        app_err_code = app_i2c_config_dma_tx(p_params);
        if (app_err_code != APP_DRV_SUCCESS)
        {
            return app_err_code;
        }
    }
    if (p_params->dma_cfg.rx_dma_instance != NULL)
    {
        app_err_code = app_i2c_config_dma_rx(p_params);
        if (app_err_code != APP_DRV_SUCCESS)
        {
            return app_err_code;
        }
    }

    return app_err_code;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_i2c_dma_init(app_i2c_params_t *p_params)
{
    app_i2c_id_t  id = p_params->id;
    app_drv_err_t app_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_i2c_env[id]->i2c_dma_state != APP_I2C_DMA_INVALID)
    {
        return APP_DRV_ERR_INVALID_INIT;
    }

    GLOBAL_EXCEPTION_DISABLE();
    app_err_code = app_i2c_config_dma(p_params);
    if (app_err_code != APP_DRV_SUCCESS)
    {
        goto __exit;
    }
    p_i2c_env[id]->i2c_dma_state = APP_I2C_DMA_ACTIVITY;

__exit:
    GLOBAL_EXCEPTION_ENABLE();

    return app_err_code;
}

uint16_t app_i2c_dma_deinit(app_i2c_id_t id)
{
    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) ||
        (p_i2c_env[id]->i2c_dma_state != APP_I2C_DMA_ACTIVITY))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    app_dma_deinit(p_i2c_env[id]->dma_id[0]);
    app_dma_deinit(p_i2c_env[id]->dma_id[1]);

    p_i2c_env[id]->i2c_dma_state = APP_I2C_DMA_INVALID;

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_dma_receive_async(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = target_address;

    if(p_i2c_env[id]->start_flag == false)
    {
        p_i2c_env[id]->start_flag = true;
        switch(p_i2c_env[id]->role)
        {
            case APP_I2C_ROLE_MASTER:
                err_code = hal_i2c_master_receive_dma(&p_i2c_env[id]->handle, target_address, p_data, size);
                break;

            case APP_I2C_ROLE_SLAVE:
                err_code = hal_i2c_slave_receive_dma(&p_i2c_env[id]->handle, p_data, size);
                break;

            default:
                break;
        }

        if (err_code != HAL_OK)
        {
            p_i2c_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_dma_transmit_async(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = target_address;

    if(p_i2c_env[id]->start_flag == false)
    {
        p_i2c_env[id]->start_flag = true;
        switch(p_i2c_env[id]->role)
        {
            case APP_I2C_ROLE_MASTER:
                err_code = hal_i2c_master_transmit_dma(&p_i2c_env[id]->handle, target_address, p_data, size);
                break;

            case APP_I2C_ROLE_SLAVE:
                err_code = hal_i2c_slave_transmit_dma(&p_i2c_env[id]->handle, p_data, size);
                break;

            default:
                break;
        }

        if (err_code != HAL_OK)
        {
            p_i2c_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_dma_mem_read_async(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = mem_address;

    if(p_i2c_env[id]->start_flag == false)
    {
        p_i2c_env[id]->start_flag = true;
        err_code = hal_i2c_mem_read_dma(&p_i2c_env[id]->handle, dev_address, mem_address, mem_addr_size, p_data, size);
        if (err_code != HAL_OK)
        {
            p_i2c_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_dma_mem_write_async(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = mem_address;

    if(p_i2c_env[id]->start_flag == false)
    {
        p_i2c_env[id]->start_flag = true;

        err_code = hal_i2c_mem_write_dma(&p_i2c_env[id]->handle, dev_address, mem_address, mem_addr_size, p_data, size);
        if (err_code != HAL_OK)
        {
            p_i2c_env[id]->start_flag = false;
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


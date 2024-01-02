/**
  ****************************************************************************************
  * @file    app_uart_dma.c
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
#include <string.h>
#include "app_uart.h"
#include "app_uart_dma.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define DMA_REQUEST_NULL                    (0x00)

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
} uart_dma_info_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uart_dma_info_t s_dma_info[APP_UART_ID_MAX] = {
    {  /* UART0 */
        .dma0_request = {
            .tx = DMA0_REQUEST_UART0_TX,
            .rx = DMA0_REQUEST_UART0_RX,
        },
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
        .dma1_request = {
            .tx = DMA1_REQUEST_UART0_TX,
            .rx = DMA1_REQUEST_UART0_RX,
        },
#endif
    },
    {  /* UART1 */
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .dma0_request = {
            .tx = DMA0_REQUEST_UART1_TX,
            .rx = DMA0_REQUEST_UART1_RX,
        },
#else
        .dma0_request = {
            .tx = 0,
            .rx = 0,
        },
#endif
    },
#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X))
    { /* UART2 */
        .dma0_request = {
            .tx = DMA0_REQUEST_UART2_TX,
            .rx = DMA0_REQUEST_UART2_RX,
        },
    },
    {  /* UART3 */
        .dma0_request = {
            .tx = DMA0_REQUEST_UART3_TX,
            .rx = DMA0_REQUEST_UART3_RX,
        },
        .dma1_request = {
            .tx = DMA1_REQUEST_UART3_TX,
            .rx = DMA1_REQUEST_UART3_RX,
        },
    },
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
    {  /* UART4 */
        .dma0_request = {
            .tx = DMA0_REQUEST_UART4_TX,
            .rx = DMA0_REQUEST_UART4_RX,
        },
        .dma1_request = {
            .tx = DMA1_REQUEST_UART4_TX,
            .rx = DMA1_REQUEST_UART4_RX,
        },
    },
    {  /* UART5 */
        .dma1_request = {
            .tx = DMA1_REQUEST_UART5_TX,
            .rx = DMA1_REQUEST_UART5_RX,
        },
    },
#endif
};

extern uart_env_t *p_uart_env[APP_UART_ID_MAX];
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
extern void uart_wake_up(app_uart_id_t id);
#endif

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */

static uint16_t app_uart_config_dma_tx(app_uart_params_t *p_params)
{
    app_dma_params_t tx_dma_params = { 0 };

    tx_dma_params.p_instance = p_params->dma_cfg.tx_dma_instance;
    tx_dma_params.channel_number = p_params->dma_cfg.tx_dma_channel;
    if (tx_dma_params.p_instance == DMA0)
    {
        tx_dma_params.init.src_request = DMA0_REQUEST_MEM;
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    else if (tx_dma_params.p_instance == DMA1)
    {
        tx_dma_params.init.src_request = DMA1_REQUEST_MEM;
    }
#endif

    if (tx_dma_params.p_instance == DMA0)
    {
        tx_dma_params.init.dst_request = s_dma_info[p_params->id].dma0_request.tx;
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    else if (tx_dma_params.p_instance == DMA1)
    {
        tx_dma_params.init.dst_request = s_dma_info[p_params->id].dma1_request.tx;
    }
#endif

    if (tx_dma_params.init.dst_request == DMA_REQUEST_NULL)
    {
        return APP_DRV_ERR_INVALID_MODE;
    }

    tx_dma_params.init.direction           = DMA_MEMORY_TO_PERIPH;
    tx_dma_params.init.src_increment       = DMA_SRC_INCREMENT;
    tx_dma_params.init.dst_increment       = DMA_DST_NO_CHANGE;
    tx_dma_params.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    tx_dma_params.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    tx_dma_params.init.mode                = DMA_NORMAL;
#endif
    tx_dma_params.init.priority            = DMA_PRIORITY_LOW;
    p_uart_env[p_params->id]->dma_id[0]    = app_dma_init(&tx_dma_params, NULL);

    if (p_uart_env[p_params->id]->dma_id[0] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    /* Associate the initialized DMA handle to the UART handle */
    p_uart_env[p_params->id]->handle.p_dmatx = app_dma_get_handle(p_uart_env[p_params->id]->dma_id[0]);
    p_uart_env[p_params->id]->handle.p_dmatx->p_parent = (void*)&p_uart_env[p_params->id]->handle;

    return APP_DRV_SUCCESS;
}

static uint16_t app_uart_config_dma_rx(app_uart_params_t *p_params)
{
    app_dma_params_t rx_dma_params = { 0 };

    rx_dma_params.p_instance               = p_params->dma_cfg.rx_dma_instance;
    rx_dma_params.channel_number           = p_params->dma_cfg.rx_dma_channel;
    if (rx_dma_params.p_instance == DMA0)
    {
        rx_dma_params.init.src_request     = s_dma_info[p_params->id].dma0_request.rx;
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    else if (rx_dma_params.p_instance == DMA1)
    {
        rx_dma_params.init.src_request     = s_dma_info[p_params->id].dma1_request.rx;
    }
#endif

    if (rx_dma_params.init.src_request == DMA_REQUEST_NULL)
    {
        return APP_DRV_ERR_INVALID_MODE;
    }

    if (rx_dma_params.p_instance == DMA0)
    {
        rx_dma_params.init.dst_request     = DMA0_REQUEST_MEM;
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    else if (rx_dma_params.p_instance == DMA1)
    {
        rx_dma_params.init.dst_request     = DMA1_REQUEST_MEM;
    }
#endif

    rx_dma_params.init.direction           = DMA_PERIPH_TO_MEMORY;
    rx_dma_params.init.src_increment       = DMA_SRC_NO_CHANGE;
    rx_dma_params.init.dst_increment       = DMA_DST_INCREMENT;
    rx_dma_params.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    rx_dma_params.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    rx_dma_params.init.mode                = DMA_NORMAL;
#endif
    rx_dma_params.init.priority            = DMA_PRIORITY_HIGH;
    p_uart_env[p_params->id]->dma_id[1] = app_dma_init(&rx_dma_params, NULL);

    if (p_uart_env[p_params->id]->dma_id[1] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    /* Associate the initialized DMA handle to the the UART handle */
    p_uart_env[p_params->id]->handle.p_dmarx = app_dma_get_handle(p_uart_env[p_params->id]->dma_id[1]);
    p_uart_env[p_params->id]->handle.p_dmarx->p_parent = (void*)&p_uart_env[p_params->id]->handle;

    return APP_DRV_SUCCESS;
}

static uint16_t app_uart_config_dma(app_uart_params_t *p_params)
{
    app_drv_err_t app_err_code = APP_DRV_SUCCESS;

    p_uart_env[p_params->id]->dma_id[0] = -1;
    p_uart_env[p_params->id]->dma_id[1] = -1;

    if (p_params->dma_cfg.tx_dma_instance == NULL &&
        p_params->dma_cfg.rx_dma_instance == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (p_params->dma_cfg.tx_dma_instance != NULL)
    {
        app_err_code = app_uart_config_dma_tx(p_params);
        if (app_err_code != APP_DRV_SUCCESS)
        {
            return app_err_code;
        }
    }
    if (p_params->dma_cfg.rx_dma_instance != NULL)
    {
        app_err_code = app_uart_config_dma_rx(p_params);
        if (app_err_code != APP_DRV_SUCCESS)
        {
            return app_err_code;
        }
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_dma_init(app_uart_params_t *p_params)
{
    app_uart_id_t  id  = p_params->id;
    app_drv_err_t  err_code;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    if (id == APP_UART_ID_1)
    {
        return APP_DRV_ERR_INVALID_ID;
    }
#endif

    if ((p_uart_env[id] == NULL) || (p_uart_env[id]->uart_state == APP_UART_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_uart_env[id]->uart_dma_state != APP_UART_DMA_INVALID)
    {
        return APP_DRV_ERR_INVALID_INIT;
    }

    GLOBAL_EXCEPTION_DISABLE();
    err_code = app_uart_config_dma(p_params);
    if (err_code != APP_DRV_SUCCESS)
    {
        goto __exit;
    }
    p_uart_env[id]->uart_dma_state = APP_UART_DMA_ACTIVITY;

__exit:
    GLOBAL_EXCEPTION_ENABLE();

    return err_code;
}

uint16_t app_uart_dma_deinit(app_uart_id_t id)
{
    if (id >= APP_UART_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_uart_env[id] == NULL) ||
        (p_uart_env[id]->uart_dma_state != APP_UART_DMA_ACTIVITY))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    app_dma_deinit(p_uart_env[id]->dma_id[0]);
    app_dma_deinit(p_uart_env[id]->dma_id[1]);

    p_uart_env[id]->uart_dma_state = APP_UART_DMA_INVALID;

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_dma_receive_async(app_uart_id_t id, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_OK;

    if (id >= APP_UART_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_uart_env[id] == NULL) || (p_uart_env[id]->uart_state == APP_UART_INVALID))
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    uart_wake_up(id);
#endif

    if (p_uart_env[id]->rx_abort_flag == true)
    {
        p_uart_env[id]->rx_abort_flag = false;
        /* Clear rx_fifo */
        ll_uart_flush_rx_fifo(p_uart_env[id]->handle.p_instance);
        /* Clear TIMEOUT interrupt flag bit */
        ll_uart_receive_data8(p_uart_env[id]->handle.p_instance);
    }

    err_code = hal_uart_receive_dma(&p_uart_env[id]->handle, p_data, size);
    APP_DRV_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_dma_start_transmit_async(app_uart_id_t id)
{
    uint16_t items_count = ring_buffer_items_count_get(&p_uart_env[id]->tx_ring_buffer);
    uint16_t send_size   = items_count;
    hal_status_t err_code;

    if ((items_count == 0) || (p_uart_env[id]->start_flush_flag == true))
    {
        p_uart_env[id]->start_tx_flag = false;
        return APP_DRV_SUCCESS;
    }

    p_uart_env[id]->is_dma_tx_mode = true;
    if (items_count >= TX_ONCE_MAX_SIZE)
    {
        ring_buffer_read(&p_uart_env[id]->tx_ring_buffer, p_uart_env[id]->tx_send_buf, TX_ONCE_MAX_SIZE);
        send_size = TX_ONCE_MAX_SIZE;
    }
    else
    {
        ring_buffer_read(&p_uart_env[id]->tx_ring_buffer, p_uart_env[id]->tx_send_buf, items_count);
    }

    err_code = hal_uart_transmit_dma(&p_uart_env[id]->handle, p_uart_env[id]->tx_send_buf, send_size);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_dma_transmit_async(app_uart_id_t id, uint8_t *p_data, uint16_t size)
{
    uint16_t err_code;

    if (id >= APP_UART_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_uart_env[id] == NULL) || (p_uart_env[id]->uart_state == APP_UART_INVALID))
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    uart_wake_up(id);
#endif

    p_uart_env[id]->tx_abort_flag = false;
    ring_buffer_write(&p_uart_env[id]->tx_ring_buffer, p_data, size);

    if ((p_uart_env[id]->start_tx_flag == false) && (p_uart_env[id]->start_flush_flag == false) &&
        (p_uart_env[id]->uart_state == APP_UART_ACTIVITY))
    {
        p_uart_env[id]->start_tx_flag = true;

        err_code = app_uart_dma_start_transmit_async(id);
        if (err_code != APP_DRV_SUCCESS)
        {
            p_uart_env[id]->start_tx_flag = false;
            return err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
uint16_t app_uart_transmit_dma_sg_llp(app_uart_id_t id, uint8_t *p_data, uint16_t size, dma_sg_llp_config_t *sg_llp_config)
{
    uint16_t err_code;

    if (id >= APP_UART_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_uart_env[id] == NULL) || (p_uart_env[id]->uart_state == APP_UART_INVALID))
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (p_data == NULL || size == 0 || sg_llp_config == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    uart_wake_up(id);
#endif

    p_uart_env[id]->tx_abort_flag = false;
    app_uart_flush(id);

    if ((p_uart_env[id]->start_tx_flag == false) && (p_uart_env[id]->start_flush_flag == false) &&
        (p_uart_env[id]->uart_state == APP_UART_ACTIVITY))
    {
        p_uart_env[id]->start_tx_flag = true;

        err_code = hal_uart_transmit_dma_sg_llp(&p_uart_env[id]->handle, p_data, size, sg_llp_config);
        if (err_code != APP_DRV_SUCCESS)
        {
            return err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_receive_dma_sg_llp(app_uart_id_t id, uint8_t *p_data, uint16_t size, dma_sg_llp_config_t *sg_llp_config)
{
    hal_status_t err_code = HAL_OK;

    if (id >= APP_UART_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_uart_env[id] == NULL) || (p_uart_env[id]->uart_state == APP_UART_INVALID))
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (p_data == NULL || size == 0 || sg_llp_config == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    uart_wake_up(id);
#endif

    if (p_uart_env[id]->rx_abort_flag == true)
    {
        p_uart_env[id]->rx_abort_flag = false;
        /* Clear rx_fifo */
        ll_uart_flush_rx_fifo(p_uart_env[id]->handle.p_instance);
        /* Clear TIMEOUT interrupt flag bit */
        ll_uart_receive_data8(p_uart_env[id]->handle.p_instance);
    }

    err_code = hal_uart_receive_dma_sg_llp(&p_uart_env[id]->handle, p_data, size, sg_llp_config);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

#endif


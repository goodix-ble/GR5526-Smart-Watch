/**
  ****************************************************************************************
  * @file    app_uart.c
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
#include "app_drv.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "gr_soc.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_UART_CALLBACK(id, evt)               \
    do                                           \
    {                                            \
        if (p_uart_env[id]->evt_handler != NULL) \
        {                                        \
            p_uart_env[id]->evt_handler(&evt);   \
        }                                        \
    } while(0)

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
typedef struct {
    app_uart_id_t id;
    IRQn_Type irq;
    uart_regs_t *instance;
} uart_info_t;

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool uart_prepare_for_sleep(void);
static void uart_wake_up_ind(void);
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void UART3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uart_info_t s_uart_info[APP_UART_ID_MAX] = {
    {
        .id  = APP_UART_ID_0,
        .irq = UART0_IRQn,
        .instance = UART0,
    },
    {
        .id  = APP_UART_ID_1,
        .irq = UART1_IRQn,
        .instance = UART1,
    },
#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) | (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X))
    {
        .id  = APP_UART_ID_2,
        .irq = UART2_IRQn,
        .instance = UART2,
    },
    {
        .id  = APP_UART_ID_3,
        .irq = UART3_IRQn,
        .instance = UART3,
    },
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
    {
        .id  = APP_UART_ID_4,
        .irq = UART4_IRQn,
        .instance = UART4,
    },
    {
        .id  = APP_UART_ID_5,
        .irq = UART5_IRQn,
        .instance = UART5,
    },
#endif
};

uart_env_t *p_uart_env[APP_UART_ID_MAX];
static const app_sleep_callbacks_t uart_sleep_cb =
{
    .app_prepare_for_sleep = uart_prepare_for_sleep,
    .app_wake_up_ind       = uart_wake_up_ind,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool uart_prepare_for_sleep(void)
{
    hal_uart_state_t state;

    for (uint32_t i = 0; i < APP_UART_ID_MAX; i++)
    {
        if (p_uart_env[i] == NULL)
        {
            continue;
        }

        if (p_uart_env[i]->uart_state == APP_UART_ACTIVITY)
        {
            state = hal_uart_get_state(&p_uart_env[i]->handle);
            if ((state != HAL_UART_STATE_RESET) && (state != HAL_UART_STATE_READY))
            {
                return false;
            }

            GLOBAL_EXCEPTION_DISABLE();
            hal_uart_suspend_reg(&p_uart_env[i]->handle);
            GLOBAL_EXCEPTION_ENABLE();

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
            p_uart_env[i]->uart_state = APP_UART_SLEEP;
#endif
        }
    }
    return true;
}

SECTION_RAM_CODE static void uart_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    uint32_t i;

    for (i = 0; i < APP_UART_ID_MAX; i++)
    {
        if (p_uart_env[i] == NULL)
        {
            continue;
        }

        if (p_uart_env[i]->uart_state == APP_UART_ACTIVITY)
        {
            GLOBAL_EXCEPTION_DISABLE();
            hal_uart_resume_reg(&p_uart_env[i]->handle);
            GLOBAL_EXCEPTION_ENABLE();
            hal_nvic_clear_pending_irq(s_uart_info[i].irq);
            hal_nvic_enable_irq(s_uart_info[i].irq);
        }
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
void uart_wake_up(app_uart_id_t id)
{
    if (p_uart_env[id]->uart_state == APP_UART_SLEEP)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_uart_resume_reg(&p_uart_env[id]->handle);
        GLOBAL_EXCEPTION_ENABLE();

        hal_nvic_clear_pending_irq(s_uart_info[id].irq);
        hal_nvic_enable_irq(s_uart_info[id].irq);

        p_uart_env[id]->uart_state = APP_UART_ACTIVITY;
    }

    dma_wake_up(p_uart_env[id]->dma_id[0]);
    dma_wake_up(p_uart_env[id]->dma_id[1]);
}
#endif

__WEAK uint16_t app_uart_dma_start_transmit_async(app_uart_id_t id)
{
    return 0;
};

static app_uart_id_t uart_get_id(uart_handle_t *p_uart)
{
    for (uint32_t i = 0; i < APP_UART_ID_MAX; i++)
    {
        if (p_uart->p_instance == s_uart_info[i].instance)
        {
            return s_uart_info[i].id;
        }
    }

    return APP_UART_ID_MAX;
}

static uint16_t uart_gpio_config(uint32_t hw_flow_ctrl, app_uart_pin_cfg_t *p_pin_cfg)
{
    app_io_init_t io_init  = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.pull = p_pin_cfg->tx.pull;
    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin  = p_pin_cfg->tx.pin;
    io_init.mux  = p_pin_cfg->tx.mux;
    err_code = app_io_init(p_pin_cfg->tx.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pull = p_pin_cfg->rx.pull;
    io_init.pin  = p_pin_cfg->rx.pin;
    io_init.mux  = p_pin_cfg->rx.mux;
    err_code = app_io_init(p_pin_cfg->rx.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    if (UART_HWCONTROL_RTS_CTS == hw_flow_ctrl)
    {
        io_init.pull = p_pin_cfg->cts.pull;
        io_init.pin  = p_pin_cfg->cts.pin;
        io_init.mux  = p_pin_cfg->cts.mux;
        err_code = app_io_init(p_pin_cfg->cts.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);

        io_init.pull = p_pin_cfg->rts.pull;
        io_init.pin  = p_pin_cfg->rts.pin;
        io_init.mux  = p_pin_cfg->rts.mux;
        err_code = app_io_init(p_pin_cfg->rts.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    return err_code;
}

static uint16_t app_uart_start_transmit_async(app_uart_id_t id)
{
    uint16_t items_count = ring_buffer_items_count_get(&p_uart_env[id]->tx_ring_buffer);
    uint16_t send_size   = items_count;
    hal_status_t err_code;

    if ((items_count == 0) || (p_uart_env[id]->start_flush_flag == true))
    {
        p_uart_env[id]->start_tx_flag = false;
        return APP_DRV_SUCCESS;
    }

    if (items_count >= TX_ONCE_MAX_SIZE)
    {
        ring_buffer_read(&p_uart_env[id]->tx_ring_buffer, p_uart_env[id]->tx_send_buf, TX_ONCE_MAX_SIZE);
        send_size = TX_ONCE_MAX_SIZE;
    }
    else
    {
        ring_buffer_read(&p_uart_env[id]->tx_ring_buffer, p_uart_env[id]->tx_send_buf, items_count);
    }

    p_uart_env[id]->is_dma_tx_mode = false;
    err_code = hal_uart_transmit_it(&p_uart_env[id]->handle, p_uart_env[id]->tx_send_buf, send_size);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_uart_init(app_uart_params_t *p_params, app_uart_evt_handler_t evt_handler, app_uart_tx_buf_t *tx_buffer)
{
    app_uart_id_t    id = p_params->id;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if ((p_params == NULL) || (tx_buffer == NULL))
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_UART_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    p_uart_env[id] = &(p_params->uart_dev);
    ring_buffer_init(&p_uart_env[id]->tx_ring_buffer, tx_buffer->tx_buf, tx_buffer->tx_buf_size);

    err_code = uart_gpio_config(p_params->init.hw_flow_ctrl, &p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(err_code);

    p_uart_env[id]->p_pin_cfg = &p_params->pin_cfg;
    p_uart_env[id]->evt_handler = evt_handler;

    memcpy(&p_uart_env[id]->handle.init, &p_params->init, sizeof(uart_init_t));
    p_uart_env[id]->handle.p_instance = s_uart_info[id].instance;

    hal_uart_deinit(&p_uart_env[id]->handle);
    hal_uart_init(&p_uart_env[id]->handle);

    pwr_register_sleep_cb(&uart_sleep_cb, APP_DRIVER_UART_WAKEUP_PRIORITY, UART_PWR_ID);

    p_uart_env[id]->uart_state = APP_UART_ACTIVITY;
    p_uart_env[id]->start_tx_flag = false;
    p_uart_env[id]->start_flush_flag = false;
    p_uart_env[id]->tx_abort_flag = false;
    p_uart_env[id]->rx_abort_flag = false;

    soc_register_nvic(UART0_IRQn, (uint32_t)UART0_IRQHandler);
    soc_register_nvic(UART1_IRQn, (uint32_t)UART1_IRQHandler);
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    soc_register_nvic(UART2_IRQn, (uint32_t)UART2_IRQHandler);
    soc_register_nvic(UART3_IRQn, (uint32_t)UART3_IRQHandler);
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
    soc_register_nvic(UART4_IRQn, (uint32_t)UART4_IRQHandler);
    soc_register_nvic(UART5_IRQn, (uint32_t)UART5_IRQHandler);
#endif
    hal_nvic_clear_pending_irq(s_uart_info[id].irq);
    hal_nvic_enable_irq(s_uart_info[id].irq);

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_deinit(app_uart_id_t id)
{
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if (id >= APP_UART_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_uart_env[id] == NULL) || (p_uart_env[id]->uart_state == APP_UART_INVALID))
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    err_code = app_io_deinit(p_uart_env[id]->p_pin_cfg->tx.type, p_uart_env[id]->p_pin_cfg->tx.pin);
    APP_DRV_ERR_CODE_CHECK(err_code);

    err_code = app_io_deinit(p_uart_env[id]->p_pin_cfg->rx.type, p_uart_env[id]->p_pin_cfg->rx.pin);
    APP_DRV_ERR_CODE_CHECK(err_code);

    if (UART_HWCONTROL_RTS_CTS == p_uart_env[id]->handle.init.hw_flow_ctrl)
    {
        err_code = app_io_deinit(p_uart_env[id]->p_pin_cfg->rts.type, p_uart_env[id]->p_pin_cfg->rts.pin);
        APP_DRV_ERR_CODE_CHECK(err_code);

        err_code = app_io_deinit(p_uart_env[id]->p_pin_cfg->cts.type, p_uart_env[id]->p_pin_cfg->cts.pin);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    hal_nvic_disable_irq(s_uart_info[id].irq);

    p_uart_env[id]->uart_state = APP_UART_INVALID;
    p_uart_env[id]->start_tx_flag = false;
    p_uart_env[id]->start_flush_flag = false;
    p_uart_env[id]->tx_abort_flag = false;
    p_uart_env[id]->rx_abort_flag = false;

    GLOBAL_EXCEPTION_DISABLE();
    for (uint32_t i = 0; i < APP_UART_ID_MAX; i++)
    {
        if ((p_uart_env[i]) && ((p_uart_env[i]->uart_state) != APP_UART_INVALID))
        {
            goto __deinit;
        }
    }
    pwr_unregister_sleep_cb(UART_PWR_ID);
__deinit:
    GLOBAL_EXCEPTION_ENABLE();

    hal_uart_deinit(&p_uart_env[id]->handle);

    if (p_uart_env[id]->uart_dma_state == APP_UART_DMA_INVALID)
    {
        p_uart_env[id] = NULL;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_receive_async(app_uart_id_t id, uint8_t *p_data, uint16_t size)
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

    if (p_data == NULL || size == 0 )
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

    err_code = hal_uart_receive_it(&p_uart_env[id]->handle, p_data, size);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_receive_sync(app_uart_id_t id, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_UART_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_uart_env[id] == NULL) || (p_uart_env[id]->uart_state == APP_UART_INVALID))
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (p_data == NULL || size == 0 )
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
    err_code = hal_uart_receive(&p_uart_env[id]->handle, p_data, size, timeout);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_transmit_async(app_uart_id_t id, uint8_t *p_data, uint16_t size)
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

    if (p_data == NULL || size == 0 )
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

        err_code = app_uart_start_transmit_async(id);
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

uint16_t app_uart_transmit_sync(app_uart_id_t id, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_UART_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_uart_env[id] == NULL) || (p_uart_env[id]->uart_state == APP_UART_INVALID))
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (p_data == NULL || size == 0 )
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    uart_wake_up(id);
#endif

    p_uart_env[id]->tx_abort_flag = false;
    err_code = hal_uart_transmit(&p_uart_env[id]->handle, p_data, size, timeout);
    if (err_code != HAL_OK)
    {
        return err_code;
    }

    return APP_DRV_SUCCESS;
}

uart_handle_t *app_uart_get_handle(app_uart_id_t id)
{
    if (id >= APP_UART_ID_MAX)
    {
        return NULL;
    }

    if ((p_uart_env[id] == NULL) || (p_uart_env[id]->uart_state == APP_UART_INVALID))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    uart_wake_up(id);
#endif

    return &p_uart_env[id]->handle;
}

void app_uart_flush(app_uart_id_t id)
{
    uint16_t items_count;
    uart_handle_t *p_uart = &p_uart_env[id]->handle;

    if (APP_UART_ID_MAX <= id || p_uart_env[id]->uart_state == APP_UART_INVALID)
    {
        return;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    uart_wake_up(id);
#endif

    if (p_uart_env[id]->uart_state == APP_UART_ACTIVITY)
    {
        p_uart_env[id]->start_flush_flag = true;

        uint16_t tx_xfer_size = 0;
        uint16_t tx_xfer_count = 0;
        uint32_t tx_wait_count = 0;
        uint32_t data_width = 1 + p_uart_env[id]->handle.init.data_bits + 5
                            + p_uart_env[id]->handle.init.stop_bits + 1
                            + (p_uart_env[id]->handle.init.parity & 1);

        while(!ll_uart_is_active_flag_tfe(p_uart_env[id]->handle.p_instance));

        if (p_uart_env[id]->is_dma_tx_mode == false)
        {
            tx_xfer_size  = p_uart_env[id]->handle.tx_xfer_size;
            tx_xfer_count = p_uart_env[id]->handle.tx_xfer_count;
            hal_uart_abort_transmit_it(&p_uart_env[id]->handle);
            hal_uart_transmit(&p_uart_env[id]->handle,
                              p_uart_env[id]->tx_send_buf + tx_xfer_size - tx_xfer_count,
                              tx_xfer_count,
                              5000);
        }
        else
        {
            do
            {
                tx_wait_count++;
            }
            while (HAL_UART_STATE_READY != hal_uart_get_state(&p_uart_env[id]->handle) &&
                   (tx_wait_count <= data_width * TX_ONCE_MAX_SIZE * (SystemCoreClock/p_uart_env[id]->handle.init.baud_rate)));
        }

        do{
            items_count = ring_buffer_items_count_get(&p_uart_env[id]->tx_ring_buffer);
            while(items_count)
            {
                uint8_t send_char;

                ring_buffer_read(&p_uart_env[id]->tx_ring_buffer, &send_char, 1);

                while(!ll_uart_is_active_flag_tfnf(p_uart_env[id]->handle.p_instance));

                ll_uart_transmit_data8(p_uart_env[id]->handle.p_instance, send_char);

                items_count--;
            }
        } while(ring_buffer_items_count_get(&p_uart_env[id]->tx_ring_buffer));

        while((HAL_IS_BIT_SET(p_uart->p_instance->LSR, LL_UART_LSR_TEMT) ? SET : RESET) == RESET);

        GLOBAL_EXCEPTION_DISABLE();
        if(ring_buffer_items_count_get(&p_uart_env[id]->tx_ring_buffer))
        {
            /* Enable the UART Transmit Data Register Empty Interrupt */
            __HAL_UART_ENABLE_IT(p_uart, UART_IT_THRE);
        }

        p_uart_env[id]->start_flush_flag = false;
        GLOBAL_EXCEPTION_ENABLE();
    }
}

uint16_t app_uart_abort(app_uart_id_t id)
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

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    uart_wake_up(id);
#endif

    ring_buffer_clean(&p_uart_env[id]->tx_ring_buffer);

    err_code = hal_uart_abort_it(&p_uart_env[id]->handle);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    p_uart_env[id]->tx_abort_flag = true;
    p_uart_env[id]->rx_abort_flag = true;

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_abort_transmit(app_uart_id_t id)
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

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    uart_wake_up(id);
#endif

    ring_buffer_clean(&p_uart_env[id]->tx_ring_buffer);

    err_code = hal_uart_abort_transmit_it(&p_uart_env[id]->handle);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    p_uart_env[id]->tx_abort_flag = true;

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_abort_receive(app_uart_id_t id)
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

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    uart_wake_up(id);
#endif

    err_code = hal_uart_abort_receive_it(&p_uart_env[id]->handle);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    p_uart_env[id]->rx_abort_flag = true;

    return APP_DRV_SUCCESS;
}

void hal_uart_tx_cplt_callback(uart_handle_t *p_uart)
{
    app_uart_evt_t uart_evt;

    app_uart_id_t id = uart_get_id(p_uart);

    uart_evt.type = APP_UART_EVT_TX_CPLT;
    uart_evt.data.size = p_uart->tx_xfer_size - p_uart->tx_xfer_count;

    if (p_uart_env[id]->is_dma_tx_mode)
    {
        app_uart_dma_start_transmit_async(id);
    }
    else
    {
        app_uart_start_transmit_async(id);
    }

    if (p_uart_env[id]->start_tx_flag == false)
    {
        APP_UART_CALLBACK(id, uart_evt);
    }
}

void hal_uart_rx_cplt_callback(uart_handle_t *p_uart)
{
    app_uart_evt_t uart_evt;
    app_uart_id_t id = uart_get_id(p_uart);

    uart_evt.type = APP_UART_EVT_RX_DATA;
    uart_evt.data.size = p_uart->rx_xfer_size - p_uart->rx_xfer_count;
    APP_UART_CALLBACK(id, uart_evt);
}

void hal_uart_error_callback(uart_handle_t *p_uart)
{
    app_uart_evt_t uart_evt;
    app_uart_id_t id = uart_get_id(p_uart);

    uart_evt.type = APP_UART_EVT_ERROR;
    uart_evt.data.error_code = p_uart->error_code;
    p_uart_env[id]->start_tx_flag = false;
    APP_UART_CALLBACK(id, uart_evt);
}

void hal_uart_abort_cplt_callback(uart_handle_t *p_uart)
{
    app_uart_evt_t uart_evt;
    app_uart_id_t id = uart_get_id(p_uart);

    uart_evt.type = APP_UART_EVT_ABORT_TXRX;
    p_uart_env[id]->start_tx_flag = false;
    APP_UART_CALLBACK(id, uart_evt);
}

void hal_uart_abort_tx_cplt_callback(uart_handle_t *p_uart)
{
    app_uart_evt_t uart_evt;
    app_uart_id_t id = uart_get_id(p_uart);

    uart_evt.type = APP_UART_EVT_ABORT_TX;
    p_uart_env[id]->start_tx_flag = false;
    APP_UART_CALLBACK(id, uart_evt);
}

void hal_uart_abort_rx_cplt_callback(uart_handle_t *p_uart)
{
    app_uart_evt_t uart_evt;
    app_uart_id_t id = uart_get_id(p_uart);

    uart_evt.type = APP_UART_EVT_ABORT_RX;
    p_uart_env[id]->start_tx_flag = false;
    APP_UART_CALLBACK(id, uart_evt);
}

#define UART_HANDLER(index, val) \
void UART##index##_IRQHandler(void)\
{\
    hal_uart_irq_handler(&p_uart_env[val]->handle);\
}

UART_HANDLER(0, APP_UART_ID_0)
UART_HANDLER(1, APP_UART_ID_1)
#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) | (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X))
UART_HANDLER(2, APP_UART_ID_2)
UART_HANDLER(3, APP_UART_ID_3)
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
UART_HANDLER(4, APP_UART_ID_4)
UART_HANDLER(5, APP_UART_ID_5)
#endif


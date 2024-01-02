
/**
 *****************************************************************************************
 *
 * @file board.c
 *
 * @brief Board Support Package Implementation.
 *
 *****************************************************************************************
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
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "grx_sys.h"
#include "app_log.h"
#include "app_assert.h"
#include "app_io.h"
#include "board_SK.h"
#if (APP_LOG_PORT == 1)
#include "SEGGER_RTT.h"
#endif

/*
    BOARD_SK DEFINE .
*/
#define UART_TX_BUFF_SIZE                  0x400

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t s_uart_tx_buffer[UART_TX_BUFF_SIZE];
static app_uart_params_t uart_param;

void bsp_uart_send(uint8_t *p_data, uint16_t length)
{
    app_uart_transmit_sync(APP_UART_ID, p_data, length, 1000);
}

void bsp_uart_flush(void)
{
    app_uart_flush(APP_UART_ID);
}

__WEAK void app_uart_evt_handler(app_uart_evt_t *p_evt)
{
    UNUSED(p_evt);
}

void bsp_uart_init(void)
{
    app_uart_tx_buf_t uart_buffer;

    uart_buffer.tx_buf       = s_uart_tx_buffer;
    uart_buffer.tx_buf_size  = UART_TX_BUFF_SIZE;

    uart_param.id                   = APP_UART_ID;
    uart_param.init.baud_rate       = APP_UART_BAUDRATE;
    uart_param.init.data_bits       = UART_DATABITS_8;
    uart_param.init.stop_bits       = UART_STOPBITS_1;
    uart_param.init.parity          = UART_PARITY_NONE;
    uart_param.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    uart_param.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;
    uart_param.pin_cfg.rx.type      = APP_UART_RX_IO_TYPE;
    uart_param.pin_cfg.rx.pin       = APP_UART_RX_PIN;
    uart_param.pin_cfg.rx.mux       = APP_UART_RX_PINMUX;
    uart_param.pin_cfg.rx.pull      = APP_UART_RX_PULL;
    uart_param.pin_cfg.tx.type      = APP_UART_TX_IO_TYPE;
    uart_param.pin_cfg.tx.pin       = APP_UART_TX_PIN;
    uart_param.pin_cfg.tx.mux       = APP_UART_TX_PINMUX;
    uart_param.pin_cfg.tx.pull      = APP_UART_TX_PULL;
    uart_param.dma_cfg.tx_dma_instance = DMA0;
    uart_param.dma_cfg.rx_dma_instance = DMA0;
    uart_param.dma_cfg.tx_dma_channel  = DMA_Channel2;
    uart_param.dma_cfg.rx_dma_channel  = DMA_Channel3;

    app_uart_init(&uart_param, app_uart_evt_handler, &uart_buffer);
}

#if (APP_LOG_PORT == 1)
void bsp_segger_rtt_send(uint8_t *p_data, uint16_t length)
{
    SEGGER_RTT_Write(0, (void*)p_data, length);
}
#endif

#if (APP_LOG_PORT == 2)
void bsp_itm_send(uint8_t *p_data, uint16_t length)
{
    for(uint16_t i = 0; i < length; i++)
    {
        ITM_SendChar(p_data[i]);
    }
}
#endif

void bsp_log_init(void)
{
#if APP_LOG_ENABLE

#if (APP_LOG_PORT == 0)
    bsp_uart_init();
#elif (APP_LOG_PORT == 1)
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);
#endif

#if (APP_LOG_PORT <= 2)
    app_log_init_t   log_init; 

    log_init.filter.level                 = APP_LOG_LVL_DEBUG;
    log_init.fmt_set[APP_LOG_LVL_ERROR]   = APP_LOG_FMT_ALL & (~APP_LOG_FMT_TAG);
    log_init.fmt_set[APP_LOG_LVL_WARNING] = APP_LOG_FMT_LVL;
    log_init.fmt_set[APP_LOG_LVL_INFO]    = APP_LOG_FMT_LVL;
    log_init.fmt_set[APP_LOG_LVL_DEBUG]   = APP_LOG_FMT_LVL;

#if (APP_LOG_PORT == 0)
    app_log_init(&log_init, bsp_uart_send, bsp_uart_flush);
#elif (APP_LOG_PORT == 1)
    app_log_init(&log_init, bsp_segger_rtt_send, NULL);
#elif (APP_LOG_PORT == 2)
    app_log_init(&log_init, bsp_itm_send, NULL);
#endif

#endif
    app_assert_init();
#endif
}

__WEAK void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    UNUSED(key_id);
    UNUSED(key_click_type);
}

void bsp_key_init(void)
{
    app_key_gpio_t app_key_inst[2];

    app_key_inst[0].gpio_type    = APP_KEY_UP_IO_TYPE;
    app_key_inst[0].gpio_pin     = APP_KEY_UP_PIN;
    app_key_inst[0].trigger_mode = APP_KEY_TRIGGER_MODE;
    app_key_inst[0].pull         = APP_KEY_PULL_MODE;
    app_key_inst[0].key_id       = BSP_KEY_UP_ID;

    app_key_inst[1].gpio_type    = APP_KEY_OK_IO_TYPE;
    app_key_inst[1].gpio_pin     = APP_KEY_OK_PIN;
    app_key_inst[1].trigger_mode = APP_KEY_TRIGGER_MODE;
    app_key_inst[1].pull         = APP_KEY_PULL_MODE;
    app_key_inst[1].key_id       = BSP_KEY_OK_ID;

    app_key_init(app_key_inst, 2, app_key_evt_handler);
}

void bsp_led_init(void)
{
    app_io_init_t io_init;

    io_init.pin  = APP_LED_NUM_0_IO;
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pull = APP_IO_PULLDOWN;
    io_init.mux  = APP_IO_MUX;
    app_io_init(APP_IO_TYPE_MSIO, &io_init);

    io_init.pin  = APP_LED_NUM_1_IO;
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pull = APP_IO_PULLDOWN;
    io_init.mux  = APP_IO_MUX;
    app_io_init(APP_IO_TYPE_MSIO, &io_init);
}

void bsp_led_open(bsp_led_num_t led_num)
{
    switch (led_num)
    {
        case BSP_LED_NUM_0:
            app_io_write_pin(APP_IO_TYPE_MSIO, APP_LED_NUM_0_IO, APP_IO_PIN_RESET);
            break;

        case BSP_LED_NUM_1:
            app_io_write_pin(APP_IO_TYPE_MSIO, APP_LED_NUM_1_IO, APP_IO_PIN_RESET);
            break;

        default:
            break;
    }
}

void bsp_led_close(bsp_led_num_t led_num)
{
    switch (led_num)
    {
        case BSP_LED_NUM_0:
            app_io_write_pin(APP_IO_TYPE_MSIO, APP_LED_NUM_0_IO, APP_IO_PIN_SET);
            break;

        case BSP_LED_NUM_1:
            app_io_write_pin(APP_IO_TYPE_MSIO, APP_LED_NUM_1_IO, APP_IO_PIN_SET);
            break;

        default:
            break;
    }
}

void board_init(void)
{
    bsp_log_init();
    bsp_key_init();
    bsp_led_init();
}

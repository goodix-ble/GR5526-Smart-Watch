/**
 *****************************************************************************************
 *
 * @file uart_simu_key_init.c
 *
 * @brief  Uart simulation key Init Function Implementation.
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
  POSSIBILITY OF SUCH DAMAGE.	35
 *****************************************************************************************
 */
 
/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <string.h>
#include "user_periph_setup.h"
#include "uart_simu_key_init.h"
#include "grx_sys.h"
#include "custom_config.h"
#include "app_assert.h"
#include "app_log.h"
#include "app_error.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define UART_TX_BUFFER_SIZE      0x400
#define UART_RX_BUFFER_SIZE      244

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
 
static app_uart_tx_buf_t s_uart_buffer;
static app_uart_params_t s_uart_param;

static uint8_t s_uart_tx_buffer[UART_TX_BUFFER_SIZE] = {0};
static uint8_t s_uart_rx_buffer[UART_RX_BUFFER_SIZE] = {0};

/*
 * LOCAL  FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void app_uart_rx_handler(app_uart_evt_t *p_evt)
{
    if (APP_UART_EVT_RX_DATA == p_evt->type)
    {
        uint8_t              key_id        = 0xFF;
        app_key_click_type_t key_click_type;
        uint8_t              rx_buf_offset = 0;

        if (!memcmp(s_uart_rx_buffer, VIR_KEY_UP_CMD, strlen(VIR_KEY_UP_CMD)))
        {
            rx_buf_offset += strlen(VIR_KEY_UP_CMD);
            key_id = VIR_KEY_UP_ID;
        }
        else if (!memcmp(s_uart_rx_buffer, VIR_KEY_DOWN_CMD, strlen(VIR_KEY_DOWN_CMD)))
        {
            rx_buf_offset += strlen(VIR_KEY_DOWN_CMD);
            key_id = VIR_KEY_DOWN_ID;
        }
        else if (!memcmp(s_uart_rx_buffer, VIR_KEY_LEFT_CMD, strlen(VIR_KEY_LEFT_CMD)))
        {
            rx_buf_offset += strlen(VIR_KEY_LEFT_CMD);
            key_id = VIR_KEY_LEFT_ID;
        }
        else if (!memcmp(s_uart_rx_buffer, VIR_KEY_RIGHT_CMD, strlen(VIR_KEY_RIGHT_CMD)))
        {
            rx_buf_offset += strlen(VIR_KEY_RIGHT_CMD);
            key_id = VIR_KEY_RIGHT_ID;
        }
        else if (!memcmp(s_uart_rx_buffer, VIR_KEY_OK_CMD, strlen(VIR_KEY_OK_CMD)))
        {
            rx_buf_offset += strlen(VIR_KEY_OK_CMD);
            key_id = VIR_KEY_OK_ID;
        }

        /* Determine the click type. */
        if (!memcmp(&(s_uart_rx_buffer[rx_buf_offset]), VIR_KEY_DOUBLE_PRESS, strlen(VIR_KEY_DOUBLE_PRESS)))
        {
            if (!memcmp(&(s_uart_rx_buffer[rx_buf_offset]), VIR_KEY_LONG_PRESS, strlen(VIR_KEY_LONG_PRESS)))
            {
                key_click_type = APP_KEY_LONG_CLICK;
            }
            else
            {
                key_click_type = APP_KEY_DOUBLE_CLICK;
            }
        }
        else if (!memcmp(&(s_uart_rx_buffer[rx_buf_offset]), VIR_KEY_CONTINUE_PRESS, strlen(VIR_KEY_CONTINUE_PRESS)))
        {
            if (!memcmp(&(s_uart_rx_buffer[rx_buf_offset]), VIR_KEY_CONTINUE_RELEASE, strlen(VIR_KEY_CONTINUE_RELEASE)))
            {
                key_click_type = APP_KEY_CONTINUE_CLICK;
            }
            else
            {
                key_click_type = APP_KEY_CONTINUE_RELEASE;
            }
        }
        else
        {
            key_click_type = APP_KEY_SINGLE_CLICK;
        }

        memset(s_uart_rx_buffer,0,UART_RX_BUFFER_SIZE);
        app_key_evt_handler(key_id, key_click_type);
    }
}

static void uart_evt_handler(app_uart_evt_t *p_evt)
{
    if (APP_UART_EVT_RX_DATA == p_evt->type)
    {
        app_uart_rx_handler(p_evt);
        app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void uart_simu_key_init(void)
{
    s_uart_buffer.tx_buf       = s_uart_tx_buffer;
    s_uart_buffer.tx_buf_size  = UART_TX_BUFFER_SIZE;

    s_uart_param.id                       = APP_UART_ID;
    s_uart_param.init.baud_rate           = APP_UART_BAUDRATE;
    s_uart_param.init.data_bits           = UART_DATABITS_8;
    s_uart_param.init.stop_bits           = UART_STOPBITS_1;
    s_uart_param.init.parity              = UART_PARITY_NONE;
    s_uart_param.init.hw_flow_ctrl        = UART_HWCONTROL_NONE;
    s_uart_param.init.rx_timeout_mode     = UART_RECEIVER_TIMEOUT_ENABLE;
    s_uart_param.pin_cfg.rx.type          = APP_UART_RX_IO_TYPE;
    s_uart_param.pin_cfg.rx.pin           = APP_UART_RX_PIN;
    s_uart_param.pin_cfg.rx.mux           = APP_UART_RX_PINMUX;
    s_uart_param.pin_cfg.rx.pull          = APP_UART_RX_PULL;
    s_uart_param.pin_cfg.tx.type          = APP_UART_TX_IO_TYPE;
    s_uart_param.pin_cfg.tx.pin           = APP_UART_TX_PIN;
    s_uart_param.pin_cfg.tx.mux           = APP_UART_TX_PINMUX;
    s_uart_param.pin_cfg.tx.pull          = APP_UART_TX_PULL;
    s_uart_param.dma_cfg.tx_dma_instance = DMA0;
    s_uart_param.dma_cfg.rx_dma_instance = DMA0;
    s_uart_param.dma_cfg.tx_dma_channel  = DMA_Channel2;
    s_uart_param.dma_cfg.rx_dma_channel  = DMA_Channel3;

    app_uart_init(&s_uart_param, uart_evt_handler, &s_uart_buffer);
    app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
}


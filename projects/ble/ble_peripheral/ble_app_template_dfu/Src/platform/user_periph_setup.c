/**
 *****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief  User Periph Init Function Implementation.
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
#include "user_periph_setup.h"
#include "gr_includes.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "user_app.h"
#include "app_log.h"
#include "app_assert.h"
#include "dfu_port.h"
#include "board_SK.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x20, 0xaa, 0xcf, 0x3e, 0xcb, 0xea};
static app_uart_params_t dfu_uart_param;

#define DFU_UART_RX_BUFF_SIZE  0x400
#define DFU_UART_TX_BUFF_SIZE  0x400

#ifdef SOC_GR5332
#define DFU_FW_SAVE_ADDR       (FLASH_START_ADDR + 0x40000)
#else
#define DFU_FW_SAVE_ADDR       (FLASH_START_ADDR + 0x60000)
#endif

static uint8_t s_dfu_uart_rx_buffer[DFU_UART_RX_BUFF_SIZE];
static uint8_t s_dfu_uart_tx_buffer[DFU_UART_TX_BUFF_SIZE];

static void dfu_program_start_callback(void);
static void dfu_programing_callback(uint8_t pro);
static void dfu_program_end_callback(uint8_t status);

static dfu_pro_callback_t dfu_pro_call =
{
    .dfu_program_start_callback = dfu_program_start_callback,
    .dfu_programing_callback = dfu_programing_callback,
    .dfu_program_end_callback = dfu_program_end_callback,
};

static void dfu_program_start_callback(void)
{
    APP_LOG_DEBUG("Dfu start program");
}

static void dfu_programing_callback(uint8_t pro)
{
    APP_LOG_DEBUG("Dfu programing---%d%%", pro);
}

static void dfu_program_end_callback(uint8_t status)
{
    APP_LOG_DEBUG("Dfu program end");
    if (0x01 == status)
    {
        APP_LOG_DEBUG("status: successful");
    }
    else
    {
        APP_LOG_DEBUG("status: error");
    }
}

void dfu_uart_evt_handler(app_uart_evt_t * p_evt)
{
    switch(p_evt->type)
    {
        case APP_UART_EVT_TX_CPLT:
            break;

        case APP_UART_EVT_RX_DATA:
            dfu_uart_receive_data_process(s_dfu_uart_rx_buffer, p_evt->data.size);
            app_uart_receive_async(APP_UART1_ID, s_dfu_uart_rx_buffer, DFU_UART_RX_BUFF_SIZE);
            break;

        case APP_UART_EVT_ERROR:
            break;

        default:break;
    }
}

static void dfu_uart_init(void)
{
    app_uart_tx_buf_t uart_buffer;

    uart_buffer.tx_buf       = s_dfu_uart_tx_buffer;
    uart_buffer.tx_buf_size  = DFU_UART_TX_BUFF_SIZE;

    dfu_uart_param.id                   = APP_UART1_ID;
    dfu_uart_param.init.baud_rate       = APP_UART_BAUDRATE;
    dfu_uart_param.init.data_bits       = UART_DATABITS_8;
    dfu_uart_param.init.stop_bits       = UART_STOPBITS_1;
    dfu_uart_param.init.parity          = UART_PARITY_NONE;
    dfu_uart_param.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    dfu_uart_param.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;
    dfu_uart_param.pin_cfg.rx.type      = APP_UART1_RX_IO_TYPE;
    dfu_uart_param.pin_cfg.rx.pin       = APP_UART1_RX_PIN;
    dfu_uart_param.pin_cfg.rx.mux       = APP_UART1_RX_PINMUX;
    dfu_uart_param.pin_cfg.rx.pull      = APP_UART_RX_PULL;
    dfu_uart_param.pin_cfg.tx.type      = APP_UART1_TX_IO_TYPE;
    dfu_uart_param.pin_cfg.tx.pin       = APP_UART1_TX_PIN;
    dfu_uart_param.pin_cfg.tx.mux       = APP_UART1_TX_PINMUX;
    dfu_uart_param.pin_cfg.tx.pull      = APP_UART_TX_PULL;

    app_uart_init(&dfu_uart_param, dfu_uart_evt_handler, &uart_buffer);
    app_uart_receive_async(APP_UART1_ID, s_dfu_uart_rx_buffer, sizeof(s_dfu_uart_rx_buffer));
}

static void uart_send_data(uint8_t *data, uint16_t size)
{
    app_uart_transmit_async(APP_UART1_ID, data, size);
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    dfu_uart_init();
    board_init();
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
    dfu_port_init(uart_send_data, DFU_FW_SAVE_ADDR, &dfu_pro_call);
}



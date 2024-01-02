/**
  ****************************************************************************************
  * @file hci_uart.c
  *
  * @brief HCI uart function Implementation.
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
 ****************************************************************************************
 */
#include "hci_uart.h"
#include "ble.h"
#include "app_uart.h"
#include "board_SK.h"
#include "grx_sys.h"

#define  HCI_CACHE_BUF_LEN    4096
#define  HCI_UART_TX_BUF_LEN  4096
#define  HCI_UART_RX_BUF_LEN  4096

#define  WVT_HCI_UART_BAUDRATE      921600

static void hci_assert_handler(const char *expr, const char *file, int line)
{
    __disable_irq();

    while(1);
}

/**
 * @defgroup APP_ASSERT_MAROC Defines
 * @{
 */
/**@brief Macro for calling hci error handler function if assert check failed. */
#define HCI_ASSERT_CHECK(EXPR)                              \
    do                                                      \
    {                                                       \
        if (!(EXPR))                                        \
        {                                                   \
            hci_assert_handler(#EXPR, __FILE__, __LINE__);  \
        }                                                   \
    } while(0)
/** @} */


static void hci_ad_host_recv_cb(uint8_t *p_data, uint16_t length);

/*
 * LOCAL VARIABLE DEFINITIONS
 *******************************************************************************
 */
static app_uart_params_t params;
static uint8_t s_hci_cache_buffer[HCI_CACHE_BUF_LEN] = {0};
static uint8_t s_uart_tx_buffer[HCI_UART_TX_BUF_LEN] = {0};
static uint8_t s_uart_rx_buffer[HCI_UART_RX_BUF_LEN] = {0};

static app_uart_tx_buf_t s_uart_tx_buf =
{
    .tx_buf      = s_uart_tx_buffer,
    .tx_buf_size = sizeof(s_uart_tx_buffer),
};

static ble_hci_rx_channel_t  s_hci_ad_rx_channel =
{
    .p_channel      = s_hci_cache_buffer,
    .cache_size   = sizeof(s_hci_cache_buffer),
};

/*
 * Local FUNCTION DEFINITIONS
 *****************************************************************************************
 */
#ifdef ATE_TEST_ENABLE
extern uint32_t g_ate_test_mode;
#endif

static void hci_uart_callback(app_uart_evt_t *p_evt)
{
    sdk_err_t err = SDK_SUCCESS;
    if (APP_UART_EVT_RX_DATA == p_evt->type)
    {
        err = ble_hci_host_packet_send(s_uart_rx_buffer, p_evt->data.size);
        HCI_ASSERT_CHECK(err == SDK_SUCCESS);

        memset(s_uart_rx_buffer, 0, HCI_UART_RX_BUF_LEN);
        #ifdef ATE_TEST_ENABLE
        if(g_ate_test_mode != PMR_MGMT_SLEEP_MODE)
        #endif
        {
            app_uart_receive_async(APP_HCI_UART_ID, s_uart_rx_buffer, sizeof(s_uart_rx_buffer));
        }
    }
}

static void hci_ad_host_recv_cb(uint8_t *p_data, uint16_t length)
{
    app_uart_transmit_async(APP_HCI_UART_ID, p_data, length);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void ble_hci_uart_init(void)
{
    memset(&params, 0, sizeof(params));

    params.id = APP_HCI_UART_ID;

    params.pin_cfg.tx.type = APP_HCI_UART_TRN_PORT;
    params.pin_cfg.tx.mux  = APP_HCI_UART_TX_PINMUX;
    params.pin_cfg.tx.pin  = APP_HCI_UART_TX_PIN;
    params.pin_cfg.tx.pull = APP_UART_TX_PULL;
    params.pin_cfg.rx.type = APP_HCI_UART_TRN_PORT;
    params.pin_cfg.rx.mux  = APP_HCI_UART_RX_PINMUX;
    params.pin_cfg.rx.pin  = APP_HCI_UART_RX_PIN;
    params.pin_cfg.rx.pull = APP_UART_RX_PULL;
    #if HCI_UART_FLOW_ON == 1
    params.pin_cfg.cts.type = APP_HCI_UART_FLOW_PORT;
    params.pin_cfg.cts.mux  = APP_HCI_UART_CTS_PINMUX;
    params.pin_cfg.cts.pin  = APP_HCI_UART_CTS_PIN;
    params.pin_cfg.rts.type = APP_HCI_UART_FLOW_PORT;
    params.pin_cfg.rts.mux  = APP_HCI_UART_RTS_PINMUX;
    params.pin_cfg.rts.pin  = APP_HCI_UART_RTS_PIN;
    #endif

    params.init.baud_rate       = APP_HCI_UART_BAUDRATE;
    params.init.data_bits       = UART_DATABITS_8;
    params.init.stop_bits       = UART_STOPBITS_1;
    params.init.parity          = UART_PARITY_NONE;
    #if HCI_UART_FLOW_ON == 1
    params.init.hw_flow_ctrl    = UART_HWCONTROL_RTS_CTS;
    #else
    params.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    #endif //
    params.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;
    app_uart_init(&params, hci_uart_callback, &s_uart_tx_buf);

    ble_hci_init(&s_hci_ad_rx_channel, hci_ad_host_recv_cb);

    app_uart_receive_async(APP_HCI_UART_ID, s_uart_rx_buffer, sizeof(s_uart_rx_buffer));
}

void ble_hci_stack_init_handle(void)
{
    ble_rf_rx_performance_selection(PRIOR_RF_RX_PERFORMANCE);
}


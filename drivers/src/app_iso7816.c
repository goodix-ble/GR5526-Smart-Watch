/**
  ****************************************************************************************
  * @file    app_iso7816.c
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
#include "app_iso7816.h"
#include "app_io.h"
#include "app_drv.h"
#include "app_pwr_mgmt.h"
#include "gr_soc.h"
#include <string.h>

#ifdef HAL_ISO7816_MODULE_ENABLED

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool iso7816_prepare_for_sleep(void);
static void iso7816_wake_up_ind(void);
static uint16_t iso7816_gpio_config(app_iso7816_pin_cfg_t *p_pin_cfg);
void ISO7816_IRQHandler(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint32_t    s_iso7816_instance = ISO7816_BASE;

iso7816_env_t *p_iso7816_env = NULL;

/* sim card command and response data buffer */
#define ISO7816_BUFFER_SIZE 33
static __ALIGNED(4) uint8_t iso7816_buffer[ISO7816_BUFFER_SIZE];
static const app_sleep_callbacks_t iso7816_sleep_cb =
{
    .app_prepare_for_sleep = iso7816_prepare_for_sleep,
    .app_wake_up_ind       = iso7816_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool iso7816_prepare_for_sleep(void)
{
    hal_iso7816_state_t state;

    if (p_iso7816_env->iso7816_state == APP_ISO7816_ACTIVITY)
    {
        state = hal_iso7816_get_state(&p_iso7816_env->handle);
        if ((state != HAL_ISO7816_STATE_READY) && (state != HAL_ISO7816_STATE_RESET))
        {
            return false;
        }

        GLOBAL_EXCEPTION_DISABLE();
        hal_iso7816_suspend_reg(&p_iso7816_env->handle);
        GLOBAL_EXCEPTION_ENABLE();

        #ifdef APP_DRIVER_WAKEUP_CALL_FUN
        p_iso7816_env->iso7816_state = APP_ISO7816_SLEEP;
        #endif
    }
    return true;
}

SECTION_RAM_CODE static void iso7816_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    if (p_iso7816_env->iso7816_state == APP_ISO7816_ACTIVITY)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_iso7816_resume_reg(&p_iso7816_env->handle); //TODO
        GLOBAL_EXCEPTION_ENABLE();

        if(p_iso7816_env->use_mode!= APP_ISO7816_TYPE_POLLING)
        {
            hal_nvic_clear_pending_irq(ISO7816_IRQn);
            hal_nvic_enable_irq(ISO7816_IRQn);
        }
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
static void iso7816_wake_up(void)
{
    if (p_iso7816_env->iso7816_state == APP_ISO7816_SLEEP)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_iso7816_resume_reg(&p_iso7816_env->handle);
        GLOBAL_EXCEPTION_ENABLE();

        if(p_iso7816_env->use_mode != APP_ISO7816_TYPE_POLLING)
        {
            hal_nvic_clear_pending_irq(ISO7816_IRQn);
            hal_nvic_enable_irq(ISO7816_IRQn);
        }
        p_iso7816_env->iso7816_state = APP_ISO7816_ACTIVITY;
    }
}
#endif

static uint16_t iso7816_gpio_config(app_iso7816_pin_cfg_t *p_pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.pull = p_pin_cfg->clk.pull;
    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin  = p_pin_cfg->clk.pin;
    io_init.mux  = p_pin_cfg->clk.mux;
    err_code = app_io_init(p_pin_cfg->clk.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pull = p_pin_cfg->rst.pull;
    io_init.pin  = p_pin_cfg->rst.pin;
    io_init.mux  = p_pin_cfg->rst.mux;
    err_code = app_io_init(p_pin_cfg->rst.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pull = p_pin_cfg->io.pull;
    io_init.pin  = p_pin_cfg->io.pin;
    io_init.mux  = p_pin_cfg->io.mux;
    err_code = app_io_init(p_pin_cfg->io.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pull = p_pin_cfg->presence.pull;
    io_init.pin  = p_pin_cfg->presence.pin;
    io_init.mux  = p_pin_cfg->presence.mux;
    err_code = app_io_init(p_pin_cfg->presence.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    return err_code;
}

static void app_iso7816_event_call(iso7816_handle_t *p_iso7816, app_iso7816_evt_type_t evt_type)
{
    app_iso7816_evt_t iso7816_evt;
    iso7816_evt.type = evt_type;
    if(evt_type == APP_ISO7816_EVT_ERROR)
    {
        iso7816_evt.data.error_code = p_iso7816->error_code;
    }
    else if(evt_type == APP_ISO7816_EVT_TX_CPLT)
    {
        iso7816_evt.data.size = p_iso7816->tx_xfer_size - p_iso7816->rx_xfer_count;
    }
    else if(evt_type == APP_ISO7816_EVT_RX_CPLT)
    {
        iso7816_evt.data.size = p_iso7816->tx_xfer_size - p_iso7816->rx_xfer_count;
    }

    p_iso7816_env->start_flag = false;
    if (p_iso7816_env->evt_handler != NULL)
    {
        p_iso7816_env->evt_handler(&iso7816_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_iso7816_init(app_iso7816_params_t *p_params, app_iso7816_evt_handler_t evt_handler)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }
    p_iso7816_env = &p_params->iso7816_env;
    app_err_code = iso7816_gpio_config(&p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    p_iso7816_env->use_mode = p_params->use_mode;
    p_iso7816_env->p_pin_cfg = &p_params->pin_cfg;
    p_iso7816_env->evt_handler = evt_handler;

    memcpy(&p_iso7816_env->handle.init, &p_params->init, sizeof(iso7816_init_t));
    p_iso7816_env->handle.p_instance = (iso7816_regs_t *)s_iso7816_instance;
    p_iso7816_env->handle.buffer_size = ISO7816_BUFFER_SIZE;
    p_iso7816_env->handle.tx_xfer_size = ISO7816_BUFFER_SIZE;
    p_iso7816_env->handle.rx_xfer_size = ISO7816_BUFFER_SIZE;
    p_iso7816_env->handle.p_tx_rx_buffer = iso7816_buffer;

    hal_err_code = hal_iso7816_init(&p_iso7816_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    pwr_register_sleep_cb(&iso7816_sleep_cb, APP_DRIVER_ISO7816_WAKEUP_PRIORITY, ISO7816_PWR_ID);

    p_iso7816_env->iso7816_state = APP_ISO7816_ACTIVITY;

    if(p_params->use_mode != APP_ISO7816_TYPE_POLLING)
    {
        soc_register_nvic(ISO7816_IRQn, (uint32_t)ISO7816_IRQHandler);
        hal_nvic_clear_pending_irq(ISO7816_IRQn);
        hal_nvic_enable_irq(ISO7816_IRQn);
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_iso7816_deinit(void)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if ((p_iso7816_env == NULL) || (p_iso7816_env->iso7816_state == APP_ISO7816_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    app_err_code = app_io_deinit(p_iso7816_env->p_pin_cfg->clk.type, p_iso7816_env->p_pin_cfg->clk.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(p_iso7816_env->p_pin_cfg->rst.type, p_iso7816_env->p_pin_cfg->rst.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(p_iso7816_env->p_pin_cfg->io.type, p_iso7816_env->p_pin_cfg->io.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(p_iso7816_env->p_pin_cfg->presence.type, p_iso7816_env->p_pin_cfg->presence.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    hal_nvic_disable_irq(ISO7816_IRQn);

    p_iso7816_env->iso7816_state = APP_ISO7816_INVALID;
    p_iso7816_env->start_flag = false;

    pwr_unregister_sleep_cb(ISO7816_PWR_ID);

    hal_err_code = hal_iso7816_deinit(&p_iso7816_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    p_iso7816_env = NULL;

    return APP_DRV_SUCCESS;
}

uint16_t app_iso7816_receive_sync(uint16_t size, uint32_t timeout)
{
    hal_status_t err_code = HAL_OK;

    if ((p_iso7816_env == NULL) || (p_iso7816_env->iso7816_state == APP_ISO7816_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    iso7816_wake_up();
#endif

    err_code = hal_iso7816_receive(&p_iso7816_env->handle, size, timeout);

    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_iso7816_receive_async(uint16_t size)
{
    hal_status_t err_code = HAL_OK;

    if ((p_iso7816_env == NULL) || (p_iso7816_env->iso7816_state == APP_ISO7816_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (p_iso7816_env->use_mode == APP_ISO7816_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_MODE;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    iso7816_wake_up();
#endif

    if(p_iso7816_env->start_flag == false)
    {
        p_iso7816_env->start_flag = true;
        err_code = hal_iso7816_receive_it(&p_iso7816_env->handle, size);
        if (err_code != HAL_OK)
        {
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_iso7816_transmit_sync(uint16_t size, uint32_t timeout)
{
    hal_status_t err_code = HAL_OK;

    if ((p_iso7816_env == NULL) || (p_iso7816_env->iso7816_state == APP_ISO7816_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    iso7816_wake_up();
#endif

     err_code = hal_iso7816_transmit(&p_iso7816_env->handle, size, timeout);

    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_iso7816_transmit_async(uint16_t size)
{
    hal_status_t err_code = HAL_OK;

    if ((p_iso7816_env == NULL) || (p_iso7816_env->iso7816_state == APP_ISO7816_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (p_iso7816_env->use_mode == APP_ISO7816_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_MODE;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    iso7816_wake_up();
#endif

    if(p_iso7816_env->start_flag == false)
    {
        p_iso7816_env->start_flag = true;

        switch(p_iso7816_env->use_mode)
        {
            case APP_ISO7816_TYPE_INTERRUPT:
                err_code = hal_iso7816_transmit_it(&p_iso7816_env->handle, size);
                break;
            case APP_ISO7816_TYPE_POLLING:
                err_code = hal_iso7816_transmit(&p_iso7816_env->handle, size,100);
                break;
            default:
                break;
        }
        if (err_code != HAL_OK)
        {
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_iso7816_transmit_receive_async(uint16_t tx_size, uint16_t rx_size)
{
    hal_status_t err_code = HAL_OK;

    if ((p_iso7816_env == NULL) || (p_iso7816_env->iso7816_state == APP_ISO7816_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (tx_size == 0 || rx_size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (p_iso7816_env->use_mode == APP_ISO7816_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_MODE;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    iso7816_wake_up();
#endif

    if(p_iso7816_env->start_flag == false)
    {
        p_iso7816_env->start_flag = true;

        err_code = hal_iso7816_transmit_receive_it(&p_iso7816_env->handle, tx_size, rx_size);

        if (err_code != HAL_OK)
        {
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_iso7816_transmit_receive_sync(uint16_t tx_size, uint16_t rx_size, uint32_t timeout)
{
    hal_status_t err_code = HAL_OK;

    if ((p_iso7816_env == NULL) || (p_iso7816_env->iso7816_state == APP_ISO7816_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (tx_size == 0 || rx_size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    iso7816_wake_up();
#endif

    err_code = hal_iso7816_transmit_receive(&p_iso7816_env->handle, tx_size, rx_size, timeout);

    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint32_t app_iso7816_get_power_states(void)
{
    uint32_t power_states;

    if ((p_iso7816_env == NULL) || (p_iso7816_env->iso7816_state == APP_ISO7816_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    iso7816_wake_up();
#endif

    power_states = hal_iso7816_get_power_states(&p_iso7816_env->handle);

    return power_states;
}

uint16_t app_iso7816_set_action(uint32_t action)
{
    hal_status_t err_code = HAL_OK;

    if ((p_iso7816_env == NULL) || (p_iso7816_env->iso7816_state == APP_ISO7816_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (action > APP_ISO7816_ACTION_TXRX)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    iso7816_wake_up();
#endif

    err_code = hal_iso7816_set_action(&p_iso7816_env->handle, action);

    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_iso7816_set_etudiv(uint32_t devide)
{
    hal_status_t err_code = HAL_OK;

    if ((p_iso7816_env == NULL) || (p_iso7816_env->iso7816_state == APP_ISO7816_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    iso7816_wake_up();
#endif

    if(p_iso7816_env->start_flag == false)
    {
        p_iso7816_env->start_flag = true;

        err_code = hal_iso7816_set_etudiv(&p_iso7816_env->handle, devide);

        if (err_code != HAL_OK)
        {
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

iso7816_handle_t *app_iso7816_get_handle(void)
{
    if((p_iso7816_env == NULL) || (p_iso7816_env->iso7816_state == APP_ISO7816_INVALID))
    {
        return NULL;
    }
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    iso7816_wake_up();
#endif
    return &p_iso7816_env->handle;
}

void hal_iso7816_error_callback(iso7816_handle_t *p_iso7816)
{
    app_iso7816_event_call(p_iso7816, APP_ISO7816_EVT_ERROR);
}

void hal_iso7816_abort_callback(iso7816_handle_t *p_iso7816)
{
    app_iso7816_event_call(p_iso7816, APP_ISO7816_EVT_ABORT);
}

void hal_iso7816_presence_callback(iso7816_handle_t *p_iso7816)
{
    app_iso7816_event_call(p_iso7816, APP_ISO7816_EVT_PRESENCE);
}

void hal_iso7816_atr_cplt_callback(iso7816_handle_t *p_iso7816)
{
    app_iso7816_event_call(p_iso7816, APP_ISO7816_EVT_ATR_CPLT);
}

void hal_iso7816_tx_cplt_callback(iso7816_handle_t *p_iso7816)
{
    app_iso7816_event_call(p_iso7816, APP_ISO7816_EVT_TX_CPLT);
}

void hal_iso7816_rx_cplt_callback(iso7816_handle_t *p_iso7816)
{
    app_iso7816_event_call(p_iso7816, APP_ISO7816_EVT_RX_CPLT);
}

void hal_iso7816_tx_rx_cplt_callback(iso7816_handle_t *p_iso7816)
{
    app_iso7816_event_call(p_iso7816, APP_ISO7816_EVT_TX_RX_CPLT);
}

void ISO7816_IRQHandler(void)
{
    hal_iso7816_irq_handler(&p_iso7816_env->handle);
}

#endif

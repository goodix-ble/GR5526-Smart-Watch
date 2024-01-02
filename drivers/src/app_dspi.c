/**
  ****************************************************************************************
  * @file    app_dspi.c
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
#include "app_dspi.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "gr_soc.h"
#include <string.h>

#ifdef HAL_DSPI_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

#define DSPI_SMART_CS_LOW()                                                                         \
    do                                                                                              \
    {                                                                                               \
            if((p_dspi_env->p_pin_cfg->cs.enable == APP_DSPI_PIN_ENABLE) && p_dspi_env->is_soft_cs) \
            {                                                                                       \
                app_io_write_pin(p_dspi_env->p_pin_cfg->cs.type,                                    \
                                 p_dspi_env->p_pin_cfg->cs.pin,                                     \
                                 APP_IO_PIN_RESET);                                                 \
            }                                                                                       \
    } while(0)

#define DSPI_SMART_CS_HIGH()                                                                        \
    do                                                                                              \
    {                                                                                               \
            if((p_dspi_env->p_pin_cfg->cs.enable == APP_DSPI_PIN_ENABLE) && p_dspi_env->is_soft_cs) \
            {                                                                                       \
                app_io_write_pin(p_dspi_env->p_pin_cfg->cs.type,                                    \
                                 p_dspi_env->p_pin_cfg->cs.pin,                                     \
                                 APP_IO_PIN_SET);                                                   \
            }                                                                                       \
    } while(0)

#define APP_DSPI_CALLBACK(evt)                  \
    do                                          \
    {   p_dspi_env->start_flag = false;         \
        DSPI_SMART_CS_HIGH();                   \
        if (p_dspi_env->evt_handler != NULL)    \
        {                                       \
            p_dspi_env->evt_handler(&dspi_evt); \
        }                                       \
    } while(0)

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
bool dspi_prepare_for_sleep(void);
void dspi_wake_up_ind(void);
static uint16_t dspi_gpio_config(app_dspi_pin_cfg_t *p_pin_cfg);
void DSPI_IRQHandler(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const IRQn_Type   s_dspi_irq = DSPI_IRQn;
static const uint32_t    s_dspi_instance = DSPI_BASE;

dspi_env_t *p_dspi_env = NULL;

static const app_sleep_callbacks_t dspi_sleep_cb =
{
    .app_prepare_for_sleep = dspi_prepare_for_sleep,
    .app_wake_up_ind       = dspi_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
bool dspi_prepare_for_sleep(void)
{
    hal_dspi_state_t state;

    if (p_dspi_env->dspi_state == APP_DSPI_ACTIVITY)
    {
        state = hal_dspi_get_state(&p_dspi_env->handle);
        if ((state != HAL_DSPI_STATE_RESET) && (state != HAL_DSPI_STATE_READY))
        {
            return false;
        }

        GLOBAL_EXCEPTION_DISABLE();
        hal_dspi_suspend_reg(&p_dspi_env->handle);
        GLOBAL_EXCEPTION_ENABLE();
        #ifdef APP_DRIVER_WAKEUP_CALL_FUN
        p_dspi_env->dspi_state = APP_DSPI_SLEEP;
        #endif
    }

    return true;
}

SECTION_RAM_CODE void dspi_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    if (p_dspi_env->dspi_state == APP_DSPI_ACTIVITY)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_dspi_resume_reg(&p_dspi_env->handle);
        GLOBAL_EXCEPTION_ENABLE();

        hal_nvic_clear_pending_irq(s_dspi_irq);
        hal_nvic_enable_irq(s_dspi_irq);
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
void dspi_wake_up(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    hal_dspi_resume_reg(&p_dspi_env->handle);
    GLOBAL_EXCEPTION_ENABLE();

    hal_nvic_clear_pending_irq(s_dspi_irq);
    hal_nvic_enable_irq(s_dspi_irq);

    p_dspi_env->dspi_state = APP_DSPI_ACTIVITY;
}
#endif

static uint16_t dspi_gpio_config(app_dspi_pin_cfg_t *p_pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if (p_pin_cfg->cs.enable == APP_DSPI_PIN_ENABLE)
    {
        if (p_dspi_env->is_soft_cs)
        {
            io_init.pull = p_pin_cfg->cs.pull;
            io_init.mode = APP_IO_MODE_OUTPUT;
            io_init.pin  = p_pin_cfg->cs.pin;
            io_init.mux  = APP_IO_MUX;
            err_code = app_io_init(p_pin_cfg->cs.type, &io_init);
            APP_DRV_ERR_CODE_CHECK(err_code);
            app_io_write_pin(p_pin_cfg->cs.type, p_pin_cfg->cs.pin, APP_IO_PIN_SET);
        }
        else
        {
            io_init.mode = APP_IO_MODE_MUX;
            io_init.pull = p_pin_cfg->cs.pull;
            io_init.pin  = p_pin_cfg->cs.pin;
            io_init.mux  = p_pin_cfg->cs.mux;
            err_code = app_io_init(p_pin_cfg->cs.type, &io_init);
            APP_DRV_ERR_CODE_CHECK(err_code);
        }
    }
    io_init.mode = APP_IO_MODE_MUX;
    if (p_pin_cfg->clk.enable == APP_DSPI_PIN_ENABLE)
    {
        io_init.pull = p_pin_cfg->clk.pull;
        io_init.pin  = p_pin_cfg->clk.pin;
        io_init.mux  = p_pin_cfg->clk.mux;
        err_code = app_io_init(p_pin_cfg->clk.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }
    if (p_pin_cfg->mosi.enable == APP_DSPI_PIN_ENABLE)
    {
        io_init.pull = p_pin_cfg->mosi.pull;
        io_init.pin  = p_pin_cfg->mosi.pin;
        io_init.mux  = p_pin_cfg->mosi.mux;
        err_code = app_io_init(p_pin_cfg->mosi.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }
    if (p_pin_cfg->miso.enable == APP_DSPI_PIN_ENABLE)
    {
        io_init.pull = p_pin_cfg->miso.pull;
        io_init.pin  = p_pin_cfg->miso.pin;
        io_init.mux  = p_pin_cfg->miso.mux;
        err_code = app_io_init(p_pin_cfg->miso.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }
    if (p_pin_cfg->dcx.enable == APP_DSPI_PIN_ENABLE)
    {
        io_init.pull = p_pin_cfg->dcx.pull;
        io_init.pin  = p_pin_cfg->dcx.pin;
        io_init.mux  = p_pin_cfg->dcx.mux;
        err_code = app_io_init(p_pin_cfg->dcx.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    return err_code;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_dspi_init(app_dspi_params_t *p_params, app_dspi_evt_handler_t evt_handler)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    p_dspi_env = &p_params->dspi_env;

    p_dspi_env->is_soft_cs = p_params->is_soft_cs;
    app_err_code = dspi_gpio_config(&p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    p_dspi_env->p_pin_cfg = &p_params->pin_cfg;
    p_dspi_env->evt_handler = evt_handler;
    memcpy(&p_dspi_env->handle.init, &p_params->init, sizeof(dspi_init_t));
    p_dspi_env->handle.p_instance = (dspi_regs_t *)s_dspi_instance;
    hal_err_code = hal_dspi_deinit(&p_dspi_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_dspi_init(&p_dspi_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    pwr_register_sleep_cb(&dspi_sleep_cb, APP_DRIVER_DSPI_WAKEUP_PRIORITY, DSPI_PWR_ID);

    p_dspi_env->dspi_state = APP_DSPI_ACTIVITY;
    p_dspi_env->start_flag = false;

    soc_register_nvic(DSPI_IRQn, (uint32_t)DSPI_IRQHandler);
    hal_nvic_clear_pending_irq(s_dspi_irq);
    hal_nvic_enable_irq(s_dspi_irq);

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_deinit(void)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_dspi_env->p_pin_cfg->cs.enable == APP_DSPI_PIN_ENABLE)
    {
        app_err_code = app_io_deinit(p_dspi_env->p_pin_cfg->cs.type, p_dspi_env->p_pin_cfg->cs.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }
    if (p_dspi_env->p_pin_cfg->clk.enable == APP_DSPI_PIN_ENABLE)
    {
        app_err_code = app_io_deinit(p_dspi_env->p_pin_cfg->clk.type, p_dspi_env->p_pin_cfg->clk.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }
    if (p_dspi_env->p_pin_cfg->mosi.enable == APP_DSPI_PIN_ENABLE)
    {
        app_err_code = app_io_deinit(p_dspi_env->p_pin_cfg->mosi.type, p_dspi_env->p_pin_cfg->mosi.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }
    if (p_dspi_env->p_pin_cfg->miso.enable == APP_DSPI_PIN_ENABLE)
    {
        app_err_code = app_io_deinit(p_dspi_env->p_pin_cfg->miso.type, p_dspi_env->p_pin_cfg->miso.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }
    if (p_dspi_env->p_pin_cfg->dcx.enable == APP_DSPI_PIN_ENABLE)
    {
        app_err_code = app_io_deinit(p_dspi_env->p_pin_cfg->dcx.type, p_dspi_env->p_pin_cfg->dcx.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }

    hal_nvic_disable_irq(s_dspi_irq);
    p_dspi_env->dspi_state = APP_DSPI_INVALID;
    p_dspi_env->start_flag = false;

    pwr_unregister_sleep_cb(DSPI_PWR_ID);

    hal_err_code = hal_dspi_deinit(&p_dspi_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    if (p_dspi_env->dspi_dma_state == APP_DSPI_DMA_INVALID)
    {
        p_dspi_env = NULL;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_command_transmit_sync(app_dspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout)
{
    hal_status_t err_code;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL || p_data == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    DSPI_SMART_CS_LOW();
    err_code = hal_dspi_command_transmit(&p_dspi_env->handle, p_cmd, p_data, timeout);
    DSPI_SMART_CS_HIGH();
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_command_transmit_async(app_dspi_command_t *p_cmd, uint8_t *p_data)
{
    hal_status_t err_code = HAL_OK;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL || p_data == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    if (p_dspi_env->start_flag == false)
    {
        DSPI_SMART_CS_LOW();
        p_dspi_env->start_flag = true;
        err_code = hal_dspi_command_transmit_it(&p_dspi_env->handle, p_cmd, p_data);
        if (err_code != HAL_OK)
        {
            DSPI_SMART_CS_HIGH();
            p_dspi_env->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_command_sync(app_dspi_command_t *p_cmd, uint32_t timeout)
{
    hal_status_t err_code;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    DSPI_SMART_CS_LOW();
    err_code = hal_dspi_command(&p_dspi_env->handle, p_cmd, timeout);
    DSPI_SMART_CS_HIGH();
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_command_async(app_dspi_command_t *p_cmd)
{
    hal_status_t err_code = HAL_ERROR;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    if (p_dspi_env->start_flag == false)
    {
        DSPI_SMART_CS_LOW();
        p_dspi_env->start_flag = true;
        err_code = hal_dspi_command_it(&p_dspi_env->handle, p_cmd);
        if (err_code != HAL_OK)
        {
            DSPI_SMART_CS_HIGH();
            p_dspi_env->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_transmit_sync(uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t err_code;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    DSPI_SMART_CS_LOW();
    err_code = hal_dspi_transmit(&p_dspi_env->handle, p_data, length, timeout);
    DSPI_SMART_CS_HIGH();
    if (err_code != HAL_OK)
    {
        return err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_transmit_async(uint8_t *p_data, uint32_t length)
{
    hal_status_t err_code = HAL_OK;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    if (p_dspi_env->start_flag == false)
    {
        DSPI_SMART_CS_LOW();
        p_dspi_env->start_flag = true;
        err_code = hal_dspi_transmit_it(&p_dspi_env->handle, p_data, length);
        if (err_code != HAL_OK)
        {
            DSPI_SMART_CS_HIGH();
            p_dspi_env->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

dspi_handle_t *app_dspi_get_handle(void)
{
    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    return &p_dspi_env->handle;
}

uint16_t app_dspi_config_mode(uint32_t mode)
{
    hal_status_t err_code = HAL_OK;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    err_code = hal_dspi_config_mode(&p_dspi_env->handle, mode);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_config_data_size(uint32_t data_size)
{
    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    p_dspi_env->handle.init.data_size = data_size;

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_abort(void)
{
    hal_status_t err_code = HAL_OK;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    err_code = hal_dspi_abort(&p_dspi_env->handle);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    p_dspi_env->start_flag = false;

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_abort_it(void)
{
    hal_status_t err_code = HAL_OK;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    err_code = hal_dspi_abort_it(&p_dspi_env->handle);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

void hal_dspi_error_callback(dspi_handle_t *p_dspi)
{
    app_dspi_evt_t dspi_evt;

    dspi_evt.type = APP_DSPI_EVT_ERROR;
    dspi_evt.data.error_code = p_dspi->error_code;
    APP_DSPI_CALLBACK(dspi_evt);
}

void hal_dspi_tx_cplt_callback(dspi_handle_t *p_dspi)
{
    app_dspi_evt_t dspi_evt;
    dspi_evt.type = APP_DSPI_EVT_TX_CPLT;
    uint32_t tx_xfer_size_cb = p_dspi->tx_xfer_size;
    uint32_t tx_xfer_count_cb = p_dspi->tx_xfer_count;
    dspi_evt.data.size = tx_xfer_size_cb - tx_xfer_count_cb;
    APP_DSPI_CALLBACK(dspi_evt);
}

void hal_dspi_abort_callback(dspi_handle_t *p_dspi)
{
    app_dspi_evt_t dspi_evt;

    dspi_evt.type = APP_DSPI_EVT_ABORT;
    dspi_evt.data.error_code = p_dspi->error_code;
    APP_DSPI_CALLBACK(dspi_evt);
}

void DSPI_IRQHandler(void)
{
    hal_dspi_irq_handler(&p_dspi_env->handle);
}

#endif

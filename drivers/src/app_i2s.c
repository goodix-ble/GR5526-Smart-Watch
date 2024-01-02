/**
  ****************************************************************************************
  * @file    app_i2s.c
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
#include "app_i2s.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "gr_soc.h"
#include <string.h>

#ifdef HAL_I2S_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_I2S_CALLBACK(id, evt)               \
    do                                          \
    {                                           \
        p_i2s_env[id]->start_flag = false;      \
        if (p_i2s_env[id]->evt_handler != NULL) \
        {                                       \
            p_i2s_env[id]->evt_handler(&evt);   \
        }                                       \
    } while(0)

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool     i2s_prepare_for_sleep(void);
static void     i2s_wake_up_ind(void);
static uint16_t i2s_gpio_config(app_i2s_id_t id, app_i2s_pin_cfg_t *p_pin_cfg);
void I2S_M_IRQHandler(void);
void I2S_S_IRQHandler(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const IRQn_Type s_i2s_irq[APP_I2S_ID_MAX] = {I2S_S_IRQn, I2S_M_IRQn};
static const uint32_t  s_i2s_instance[APP_I2S_ID_MAX] = {I2S_S_BASE, I2S_M_BASE};

i2s_env_t *p_i2s_env[APP_I2S_ID_MAX];

const static app_sleep_callbacks_t i2s_sleep_cb =
{
    .app_prepare_for_sleep = i2s_prepare_for_sleep,
    .app_wake_up_ind       = i2s_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool i2s_prepare_for_sleep(void)
{
    hal_i2s_state_t state;
    uint32_t i;

    for (i = 0; i < APP_I2S_ID_MAX; i++)
    {
        if (p_i2s_env[i] == NULL)
        {
            continue;
        }

        if (p_i2s_env[i]->i2s_state == APP_I2S_ACTIVITY)
        {
            state = hal_i2s_get_state(&p_i2s_env[i]->handle);
            if ((state != HAL_I2S_STATE_READY) && (state != HAL_I2S_STATE_RESET))
            {
                return false;
            }

            GLOBAL_EXCEPTION_DISABLE();
            hal_i2s_suspend_reg(&p_i2s_env[i]->handle);
            GLOBAL_EXCEPTION_ENABLE();

            #ifdef APP_DRIVER_WAKEUP_CALL_FUN
            p_i2s_env[i]->i2s_state = APP_I2S_SLEEP;
            #endif
        }
    }

    return true;
}

SECTION_RAM_CODE static void i2s_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    uint32_t i;

    for (i = 0; i < APP_I2S_ID_MAX; i++)
    {
        if (p_i2s_env[i] == NULL)
        {
            continue;
        }

        if (p_i2s_env[i]->i2s_state == APP_I2S_ACTIVITY)
        {
            GLOBAL_EXCEPTION_DISABLE();
            hal_i2s_resume_reg(&p_i2s_env[i]->handle);
            GLOBAL_EXCEPTION_ENABLE();

            hal_nvic_clear_pending_irq(s_i2s_irq[i]);
            hal_nvic_enable_irq(s_i2s_irq[i]);
        }
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
void i2s_wake_up(app_i2s_id_t id)
{
    if (p_i2s_env[id]->i2s_state == APP_I2S_SLEEP)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_i2s_resume_reg(&p_i2s_env[id]->handle);
        GLOBAL_EXCEPTION_ENABLE();

        hal_nvic_clear_pending_irq(s_i2s_irq[id]);
        hal_nvic_enable_irq(s_i2s_irq[id]);

        p_i2s_env[id]->i2s_state = APP_I2S_ACTIVITY;
        dma_wake_up(p_i2s_env[id]->dma_id[0]);
        dma_wake_up(p_i2s_env[id]->dma_id[1]);
    }
}
#endif

static uint16_t i2s_gpio_config(app_i2s_id_t id, app_i2s_pin_cfg_t *p_pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.pull = p_pin_cfg->ws.pull;
    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin  = p_pin_cfg->ws.pin;
    io_init.mux  = p_pin_cfg->ws.mux;
    err_code = app_io_init(p_pin_cfg->ws.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pull = p_pin_cfg->sdo.pull;
    io_init.pin  = p_pin_cfg->sdo.pin;
    io_init.mux  = p_pin_cfg->sdo.mux;
    err_code = app_io_init(p_pin_cfg->sdo.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pull = p_pin_cfg->sdi.pull;
    io_init.pin  = p_pin_cfg->sdi.pin;
    io_init.mux  = p_pin_cfg->sdi.mux;
    err_code = app_io_init(p_pin_cfg->sdi.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pull = p_pin_cfg->sclk.pull;
    io_init.pin  = p_pin_cfg->sclk.pin;
    io_init.mux  = p_pin_cfg->sclk.mux;
    err_code = app_io_init(p_pin_cfg->sclk.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    return err_code;
}

static app_i2s_id_t i2s_get_id(i2s_handle_t *p_i2s)
{
    app_i2s_id_t id = APP_I2S_ID_MAX;

    if (p_i2s->p_instance == I2S_S)
    {
        id = APP_I2S_ID_SLAVE;
    }
    else if (p_i2s->p_instance == I2S_M)
    {
        id = APP_I2S_ID_MASTER;
    }

    return id;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_i2s_init(app_i2s_params_t *p_params, app_i2s_evt_handler_t evt_handler)
{
    app_i2s_id_t id = p_params->id;
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }
    p_i2s_env[id] = &(p_params->i2s_env);
    app_err_code = i2s_gpio_config(p_params->id, &p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    p_i2s_env[id]->p_pin_cfg = &p_params->pin_cfg;
    p_i2s_env[id]->evt_handler = evt_handler;

    memcpy(&p_i2s_env[id]->handle.init, &p_params->init, sizeof(i2s_init_t));
    p_i2s_env[id]->handle.p_instance = (i2s_regs_t *)s_i2s_instance[id];

    hal_err_code = hal_i2s_deinit(&p_i2s_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    hal_err_code = hal_i2s_init(&p_i2s_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    pwr_register_sleep_cb(&i2s_sleep_cb, APP_DRIVER_I2S_WAKEUP_PRIORITY, I2S_PWR_ID);

    p_i2s_env[id]->i2s_state = APP_I2S_ACTIVITY;
    p_i2s_env[id]->start_flag = false;

    soc_register_nvic(I2S_S_IRQn, (uint32_t)I2S_S_IRQHandler);
    soc_register_nvic(I2S_M_IRQn, (uint32_t)I2S_M_IRQHandler);
    hal_nvic_clear_pending_irq(s_i2s_irq[id]);
    hal_nvic_enable_irq(s_i2s_irq[id]);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_deinit(app_i2s_id_t id)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2s_env[id] == NULL) ||  (p_i2s_env[id]->i2s_state == APP_I2S_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    app_err_code = app_io_deinit(p_i2s_env[id]->p_pin_cfg->ws.type, p_i2s_env[id]->p_pin_cfg->ws.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(p_i2s_env[id]->p_pin_cfg->sdo.type, p_i2s_env[id]->p_pin_cfg->sdo.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(p_i2s_env[id]->p_pin_cfg->sdi.type, p_i2s_env[id]->p_pin_cfg->sdi.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(p_i2s_env[id]->p_pin_cfg->sclk.type, p_i2s_env[id]->p_pin_cfg->sclk.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    hal_nvic_disable_irq(s_i2s_irq[id]);

    p_i2s_env[id]->i2s_state = APP_I2S_INVALID;
    p_i2s_env[id]->start_flag = false;

    GLOBAL_EXCEPTION_DISABLE();
    for (uint32_t i = 0; i < APP_I2S_ID_MAX; i++)
    {
        if ((p_i2s_env[i]) && ((p_i2s_env[i]->i2s_state) != APP_I2S_INVALID))
        {
            goto __deinit;
        }
    }
    pwr_unregister_sleep_cb(I2S_PWR_ID);
__deinit:
    GLOBAL_EXCEPTION_ENABLE();

    hal_err_code = hal_i2s_deinit(&p_i2s_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    if (p_i2s_env[id]->i2s_dma_state == APP_I2S_DMA_INVALID)
    {
        p_i2s_env[id] = NULL;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_receive_async(app_i2s_id_t id, uint16_t *p_data, uint16_t size)
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
        err_code = hal_i2s_receive_it(&p_i2s_env[id]->handle, p_data, size);
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

uint16_t app_i2s_receive_sync(app_i2s_id_t id, uint16_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

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

    err_code = hal_i2s_receive(&p_i2s_env[id]->handle, p_data, size, timeout);

    __HAL_I2S_DISABLE_RX_BLOCK(&p_i2s_env[id]->handle);

    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}
uint16_t app_i2s_transmit_async(app_i2s_id_t id, uint16_t *p_data, uint16_t size)
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
        err_code = hal_i2s_transmit_it(&p_i2s_env[id]->handle, p_data, size);
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

uint16_t app_i2s_transmit_receive_sync(app_i2s_id_t id,
                                       uint16_t *p_tx_data,
                                       uint16_t *p_rx_data,
                                       uint32_t length,
                                       uint32_t timeout)
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

    if (p_tx_data == NULL || p_rx_data == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    if (p_i2s_env[id]->start_flag == false)
    {
        p_i2s_env[id]->start_flag = true;

        err_code = hal_i2s_transmit_receive(&p_i2s_env[id]->handle, p_tx_data,  p_rx_data, length, timeout);

        __HAL_I2S_DISABLE_RX_BLOCK(&p_i2s_env[id]->handle);

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

    p_i2s_env[id]->start_flag = false;

    return APP_DRV_SUCCESS;
}


uint16_t app_i2s_transmit_receive_async(app_i2s_id_t id,
                                        uint16_t     *p_tx_data,
                                        uint16_t     *p_rx_data,
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

    if (p_tx_data == NULL || p_rx_data == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    if (false == p_i2s_env[id]->start_flag)
    {
        p_i2s_env[id]->start_flag = true;
        err_code = hal_i2s_transmit_receive_it(&p_i2s_env[id]->handle, p_tx_data, p_rx_data, length);
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

/**
 * @brief Abort ongoing transfer (blocking mode).
 * @param id I2S ID
 * @note This procedure is executed in blocking mode: When exiting
 * function, Abort is considered as completed.
 * @retval ::APP_DRV_SUCCESS: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 */
uint16_t app_i2s_abort(app_i2s_id_t id)
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

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    err_code =  hal_i2s_abort(&p_i2s_env[id]->handle);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    p_i2s_env[id]->start_flag = false;

    return APP_DRV_SUCCESS;
}


uint16_t app_i2s_transmit_sync(app_i2s_id_t id, uint16_t *p_data, uint16_t size, uint32_t timeout)
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

    err_code =  hal_i2s_transmit(&p_i2s_env[id]->handle, p_data, size, timeout);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_enable(app_i2s_id_t id)
{
    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2s_env[id] == NULL) ||  (p_i2s_env[id]->i2s_state == APP_I2S_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    __HAL_I2S_ENABLE(&p_i2s_env[id]->handle);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_disable(app_i2s_id_t id)
{
    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2s_env[id] == NULL) ||  (p_i2s_env[id]->i2s_state == APP_I2S_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    __HAL_I2S_DISABLE(&p_i2s_env[id]->handle);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_enable_clock(app_i2s_id_t id)
{
    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2s_env[id] == NULL) ||  (p_i2s_env[id]->i2s_state == APP_I2S_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    __HAL_I2S_ENABLE_CLOCK(&p_i2s_env[id]->handle);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_disable_clock(app_i2s_id_t id)
{
    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2s_env[id] == NULL) ||  (p_i2s_env[id]->i2s_state == APP_I2S_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    __HAL_I2S_DISABLE_CLOCK(&p_i2s_env[id]->handle);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_flush_tx_fifo(app_i2s_id_t id)
{
    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2s_env[id] == NULL) ||  (p_i2s_env[id]->i2s_state == APP_I2S_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    __HAL_I2S_FLUSH_TX_FIFO(&p_i2s_env[id]->handle);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_flush_rx_fifo(app_i2s_id_t id)
{
    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2s_env[id] == NULL) ||  (p_i2s_env[id]->i2s_state == APP_I2S_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    __HAL_I2S_FLUSH_RX_FIFO(&p_i2s_env[id]->handle);

    return APP_DRV_SUCCESS;
}

i2s_handle_t *app_i2s_get_handle(app_i2s_id_t id)
{
    if (id >= APP_I2S_ID_MAX)
    {
        return NULL;
    }

    if ((p_i2s_env[id] == NULL) ||  (p_i2s_env[id]->i2s_state == APP_I2S_INVALID))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2s_wake_up(id);
#endif

    return &p_i2s_env[id]->handle;
}

void hal_i2s_tx_cplt_callback(i2s_handle_t *p_i2s)
{
    app_i2s_evt_t i2s_evt;
    app_i2s_id_t id = i2s_get_id(p_i2s);

    i2s_evt.type = APP_I2S_EVT_TX_CPLT;
    uint32_t tx_xfer_size_cb = p_i2s->tx_xfer_size;
    uint32_t tx_xfer_count_cb = p_i2s->tx_xfer_count;
    i2s_evt.data.size = tx_xfer_size_cb - tx_xfer_count_cb;
    APP_I2S_CALLBACK(id, i2s_evt);
}

void hal_i2s_rx_cplt_callback(i2s_handle_t *p_i2s)
{
    app_i2s_evt_t i2s_evt;
    app_i2s_id_t id = i2s_get_id(p_i2s);

    i2s_evt.type = APP_I2S_EVT_RX_DATA;
    uint32_t rx_xfer_size_cb = p_i2s->rx_xfer_size;
    uint32_t rx_xfer_count_cb = p_i2s->rx_xfer_count;
    i2s_evt.data.size = rx_xfer_size_cb - rx_xfer_count_cb;
    __HAL_I2S_DISABLE_RX_BLOCK(p_i2s);
    APP_I2S_CALLBACK(id, i2s_evt);
}

void hal_i2s_tx_rx_cplt_callback(i2s_handle_t *p_i2s)
{
    app_i2s_evt_t i2s_evt;
    app_i2s_id_t id = i2s_get_id(p_i2s);

    i2s_evt.type = APP_I2S_EVT_TX_RX;
    uint32_t rx_xfer_size_cb = p_i2s->rx_xfer_size;
    uint32_t rx_xfer_count_cb = p_i2s->rx_xfer_count;
    i2s_evt.data.size = rx_xfer_size_cb - rx_xfer_count_cb;
    __HAL_I2S_DISABLE_RX_BLOCK(p_i2s);
    APP_I2S_CALLBACK(id, i2s_evt);
}

void hal_i2s_error_callback(i2s_handle_t *p_i2s)
{
    app_i2s_evt_t i2s_evt;
    app_i2s_id_t id = i2s_get_id(p_i2s);

    i2s_evt.type = APP_I2S_EVT_ERROR;
    i2s_evt.data.error_code = p_i2s->error_code;
    APP_I2S_CALLBACK(id, i2s_evt);
}

SECTION_RAM_CODE void I2S_S_IRQHandler(void)
{
    hal_i2s_irq_handler(&p_i2s_env[APP_I2S_ID_SLAVE]->handle);
}

SECTION_RAM_CODE void I2S_M_IRQHandler(void)
{
    hal_i2s_irq_handler(&p_i2s_env[APP_I2S_ID_MASTER]->handle);
}

#endif

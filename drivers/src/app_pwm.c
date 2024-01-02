/**
  ****************************************************************************************
  * @file    app_pwm.c
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
#include "app_pwm.h"
#include "app_io.h"
#include "app_pwr_mgmt.h"
#include <string.h>
#include "gr_soc.h"

#ifdef HAL_CALENDAR_MODULE_ENABLED

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool pwm_prepare_for_sleep(void);
static void pwm_wake_up_ind(void);
static uint16_t pwm_gpio_config(app_pwm_pin_cfg_t *p_pin_cfg);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
pwm_env_t *p_pwm_env[APP_PWM_ID_MAX];
static const uint32_t s_pwm_instance[APP_PWM_ID_MAX] = {PWM0_BASE, PWM1_BASE};

static const app_sleep_callbacks_t pwm_sleep_cb =
{
    .app_prepare_for_sleep = pwm_prepare_for_sleep,
    .app_wake_up_ind       = pwm_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
bool pwm_prepare_for_sleep(void)
{
    hal_pwm_state_t state;
    uint32_t i;

    for (i = 0; i < APP_PWM_ID_MAX; i++)
    {
        if (p_pwm_env[i] == NULL)
        {
            continue;
        }

        if (p_pwm_env[i] != NULL && p_pwm_env[i]->pwm_state == APP_PWM_ACTIVITY)
        {
            state = hal_pwm_get_state(&p_pwm_env[i]->handle);
            if ((state != HAL_PWM_STATE_RESET) && (state != HAL_PWM_STATE_READY))
            {
                return false;
            }

            GLOBAL_EXCEPTION_DISABLE();
            hal_pwm_suspend_reg(&p_pwm_env[i]->handle);
            GLOBAL_EXCEPTION_ENABLE();
            #ifdef APP_DRIVER_WAKEUP_CALL_FUN
            p_pwm_env[i]->pwm_state = APP_PWM_SLEEP;
            #endif
        }
    }

    return true;
}

SECTION_RAM_CODE void pwm_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    uint32_t i;

    for (i = 0; i < APP_PWM_ID_MAX; i++)
    {
        if (p_pwm_env[i] == NULL)
        {
            continue;
        }

        if (p_pwm_env[i] != NULL && p_pwm_env[i]->pwm_state == APP_PWM_ACTIVITY)
        {
            GLOBAL_EXCEPTION_DISABLE();
            hal_pwm_resume_reg(&p_pwm_env[i]->handle);
            GLOBAL_EXCEPTION_ENABLE();

            if (p_pwm_env[i]->pwm_module_state == APP_PWM_START)
            {
                hal_pwm_start(&p_pwm_env[i]->handle);
            }
        }
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
void pwm_wake_up(app_pwm_id_t id)
{
    if (p_pwm_env[id] != NULL && p_pwm_env[id]->pwm_state == APP_PWM_SLEEP)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_pwm_resume_reg(&p_pwm_env[id]->handle);
        GLOBAL_EXCEPTION_ENABLE();

        if (p_pwm_env[id]->pwm_module_state == APP_PWM_START)
        {
            hal_pwm_start(&p_pwm_env[id]->handle);
        }
    }
}
#endif

static uint16_t pwm_gpio_config(app_pwm_pin_cfg_t *p_pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.pull = APP_IO_PULLUP;
    io_init.mode = APP_IO_MODE_MUX;

    if (p_pin_cfg->channel_a.enable == APP_PWM_PIN_ENABLE)
    {
        io_init.pin  = p_pin_cfg->channel_a.pin;
        io_init.mux  = p_pin_cfg->channel_a.mux;
        err_code = app_io_init(p_pin_cfg->channel_a.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    if (p_pin_cfg->channel_b.enable == APP_PWM_PIN_ENABLE)
    {
        io_init.pin  = p_pin_cfg->channel_b.pin;
        io_init.mux  = p_pin_cfg->channel_b.mux;
        err_code = app_io_init(p_pin_cfg->channel_b.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    if (p_pin_cfg->channel_c.enable == APP_PWM_PIN_ENABLE)
    {
        io_init.pin  = p_pin_cfg->channel_c.pin;
        io_init.mux  = p_pin_cfg->channel_c.mux;
        err_code = app_io_init(p_pin_cfg->channel_c.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    return err_code;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)

static void app_pwm_event_call(pwm_handle_t *p_pwm, app_pwm_evt_type_t evt_type)
{
    app_pwm_evt_t pwm_evt;
    app_pwm_id_t id = APP_PWM_ID_0;

    pwm_evt.type = evt_type;
    if(pwm_evt.type == APP_PWM_CHANNEL_A_ERROR) {
        pwm_evt.error_code = HAL_PWM_CHANNEL_A_ERROR;
    } else if (pwm_evt.type == APP_PWM_CHANNEL_B_ERROR) {
        pwm_evt.error_code = HAL_PWM_CHANNEL_B_ERROR;
    } else if (pwm_evt.type == APP_PWM_CHANNEL_C_ERROR) {
        pwm_evt.error_code = HAL_PWM_CHANNEL_C_ERROR;
    } else {
        pwm_evt.error_code = HAL_PWM_ERROR_NONE;
    }

    p_pwm_env[id]->evt_handler(&pwm_evt);
}

void PWM0_IRQHandler(void)
{
    hal_pwm_irq_handler(&p_pwm_env[APP_PWM_ID_0]->handle);
}

#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
uint16_t app_pwm_init(app_pwm_params_t *p_params, app_pwm_evt_handler_t evt_handler)
#else
uint16_t app_pwm_init(app_pwm_params_t *p_params)
#endif
{
    app_pwm_id_t id = p_params->id;
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_PWM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    p_pwm_env[id] = &(p_params->pwm_env);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    soc_register_nvic(PWM0_IRQn, (uint32_t)PWM0_IRQHandler);

    NVIC_ClearPendingIRQ(PWM0_IRQn);
    NVIC_EnableIRQ(PWM0_IRQn);
#endif

    app_err_code = pwm_gpio_config(&p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(app_err_code);
    memcpy(&p_pwm_env[id]->handle.init, &p_params->init, sizeof(pwm_init_t));

    p_pwm_env[id]->p_pin_cfg = &p_params->pin_cfg;
    p_pwm_env[id]->handle.active_channel = (hal_pwm_active_channel_t)p_params->active_channel;
    p_pwm_env[id]->handle.p_instance = (pwm_regs_t *)s_pwm_instance[id];
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    p_pwm_env[id]->evt_handler = evt_handler;
#endif

    hal_err_code = hal_pwm_deinit(&p_pwm_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_pwm_init(&p_pwm_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    pwr_register_sleep_cb(&pwm_sleep_cb, APP_DRIVER_PWM_WAKEUP_PRIORITY, PWM_PWR_ID);

    p_pwm_env[id]->pwm_state = APP_PWM_ACTIVITY;
    p_pwm_env[id]->pwm_module_state = APP_PWM_STOP;

    return APP_DRV_SUCCESS;
}

uint16_t app_pwm_deinit(app_pwm_id_t id)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_PWM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    p_pwm_env[id]->pwm_state = APP_PWM_INVALID;
    p_pwm_env[id]->pwm_module_state = APP_PWM_STOP;

    GLOBAL_EXCEPTION_DISABLE();
    for (uint32_t i = 0; i < APP_PWM_ID_MAX; i++)
    {
        if ((p_pwm_env[i]) && ((p_pwm_env[i]->pwm_state) != APP_PWM_INVALID))
        {
            goto __deinit;
        }
    }
    pwr_unregister_sleep_cb(PWM_PWR_ID);
__deinit:
    GLOBAL_EXCEPTION_ENABLE();

    if (p_pwm_env[id]->p_pin_cfg->channel_a.enable == APP_PWM_PIN_ENABLE)
    {
        app_io_deinit(p_pwm_env[id]->p_pin_cfg->channel_a.type, p_pwm_env[id]->p_pin_cfg->channel_a.pin);
    }
    if (p_pwm_env[id]->p_pin_cfg->channel_b.enable == APP_PWM_PIN_ENABLE)
    {
        app_io_deinit(p_pwm_env[id]->p_pin_cfg->channel_b.type, p_pwm_env[id]->p_pin_cfg->channel_b.pin);
    }
    if (p_pwm_env[id]->p_pin_cfg->channel_c.enable == APP_PWM_PIN_ENABLE)
    {
        app_io_deinit(p_pwm_env[id]->p_pin_cfg->channel_c.type, p_pwm_env[id]->p_pin_cfg->channel_c.pin);
    }

    err_code = hal_pwm_deinit(&p_pwm_env[id]->handle);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_pwm_start(app_pwm_id_t id)
{
    hal_status_t err_code;

    if (id >= APP_PWM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    err_code = hal_pwm_start(&p_pwm_env[id]->handle);
    HAL_ERR_CODE_CHECK(err_code);

    p_pwm_env[id]->pwm_module_state = APP_PWM_START;

    return APP_DRV_SUCCESS;
}

uint16_t app_pwm_stop(app_pwm_id_t id)
{
    hal_status_t err_code;

    if (id >= APP_PWM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }


#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    err_code = hal_pwm_stop(&p_pwm_env[id]->handle);
    HAL_ERR_CODE_CHECK(err_code);

    p_pwm_env[id]->pwm_module_state = APP_PWM_STOP;

    return APP_DRV_SUCCESS;
}

uint16_t app_pwm_update_freq(app_pwm_id_t id, uint32_t freq)
{
    hal_status_t err_code;

    if (id >= APP_PWM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    err_code = hal_pwm_update_freq(&p_pwm_env[id]->handle, freq);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_pwm_config_channel(app_pwm_id_t id, app_pwm_active_channel_t channel, app_pwm_channel_init_t *p_config)
{
    hal_status_t err_code;

    hal_pwm_active_channel_t active_channel;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    pwm_none_coding_channel_init_t channel_cfg;
#else
    pwm_channel_init_t channel_cfg;
#endif

    if (id >= APP_PWM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }


#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    active_channel   = (hal_pwm_active_channel_t)channel;
    channel_cfg.duty = p_config->duty;
    channel_cfg.drive_polarity = p_config->drive_polarity;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
    channel_cfg.fstoplvl = p_config->fstoplvl;
#endif

    err_code = hal_pwm_config_channel(&p_pwm_env[id]->handle, &channel_cfg, active_channel);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)

uint16_t app_pwm_resume(app_pwm_id_t id)
{
    hal_status_t err_code;

    if (id >= APP_PWM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }


#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    err_code = hal_pwm_resume(&p_pwm_env[id]->handle);
    HAL_ERR_CODE_CHECK(err_code);

    p_pwm_env[id]->pwm_module_state = APP_PWM_START;

    return APP_DRV_SUCCESS;
}

uint16_t app_pwm_pause(app_pwm_id_t id)
{
    hal_status_t err_code;

    if (id >= APP_PWM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }


#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    err_code = hal_pwm_pause(&p_pwm_env[id]->handle);
    HAL_ERR_CODE_CHECK(err_code);

    p_pwm_env[id]->pwm_module_state = APP_PWM_STOP;

    return APP_DRV_SUCCESS;
}

#endif

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)

uint16_t app_pwm_inactive_channel(app_pwm_id_t id, app_pwm_active_channel_t channel)
{
    hal_status_t err_code;

    hal_pwm_active_channel_t active_channel;

    if (id >= APP_PWM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    active_channel = (hal_pwm_active_channel_t)channel;

    err_code = hal_pwm_inactive_channel(&p_pwm_env[id]->handle, active_channel);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

#endif

pwm_handle_t *app_pwm_get_handle(app_pwm_id_t id)
{
    if (id >= APP_PWM_ID_MAX)
    {
        return NULL;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    return &p_pwm_env[id]->handle;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)

uint16_t app_pwm_set_coding_data_in_one_channel(app_pwm_id_t id, uint32_t coding_data)
{
    hal_status_t err_code;

    if (id != APP_PWM_ID_0)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    err_code = hal_pwm_set_coding_data_in_one_channel(&p_pwm_env[id]->handle, coding_data);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_pwm_set_coding_data_in_three_channels(app_pwm_id_t id, uint32_t coding_data0, uint32_t coding_data1, uint32_t coding_data2)
{
    hal_status_t err_code;

    if (id != APP_PWM_ID_0)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    err_code = hal_pwm_set_coding_data_in_three_channels(&p_pwm_env[id]->handle, coding_data0, coding_data1, coding_data2);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_pwm_start_coding_in_one_channel(app_pwm_id_t id, uint32_t coding_data)
{
    hal_status_t err_code;

    if (id != APP_PWM_ID_0)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    err_code = hal_pwm_start_coding_in_one_channel(&p_pwm_env[id]->handle, coding_data);
    HAL_ERR_CODE_CHECK(err_code);

    p_pwm_env[id]->pwm_module_state = APP_PWM_START;

    return APP_DRV_SUCCESS;
}

uint16_t app_pwm_start_coding_in_three_channels(app_pwm_id_t id, uint32_t coding_data0, uint32_t coding_data1, uint32_t coding_data2)
{
    hal_status_t err_code;

    if (id != APP_PWM_ID_0)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    err_code = hal_pwm_start_coding_in_three_channels(&p_pwm_env[id]->handle, coding_data0, coding_data1, coding_data2);
    HAL_ERR_CODE_CHECK(err_code);

    p_pwm_env[id]->pwm_module_state = APP_PWM_START;

    return APP_DRV_SUCCESS;
}

void hal_pwm_channel_a_error_callback(pwm_handle_t *p_pwm)
{
    app_pwm_event_call(p_pwm, APP_PWM_CHANNEL_A_ERROR);
}

void hal_pwm_channel_b_error_callback(pwm_handle_t *p_pwm)
{
    app_pwm_event_call(p_pwm, APP_PWM_CHANNEL_B_ERROR);
}

void hal_pwm_channel_c_error_callback(pwm_handle_t *p_pwm)
{
    app_pwm_event_call(p_pwm, APP_PWM_CHANNEL_C_ERROR);
}

void hal_pwm_coding_done_callback(pwm_handle_t *p_pwm)
{
    app_pwm_event_call(p_pwm, APP_PWM_CODING_DONE);
}

void hal_pwm_coding_load_callback(pwm_handle_t *p_pwm)
{
    app_pwm_event_call(p_pwm, APP_PWM_CODING_LOAD);
}

#endif

#endif


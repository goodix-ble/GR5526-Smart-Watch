/**
  ****************************************************************************************
  * @file    app_dual_tim.c
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
#include "app_dual_tim.h"
#include "app_drv.h"
#include "string.h"
#include "app_pwr_mgmt.h"
#include "gr_soc.h"

#ifdef HAL_DUAL_TIMER_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool dual_tim_prepare_for_sleep(void);
static void dual_tim_wake_up_ind(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const IRQn_Type   s_dual_tim_irq[APP_DUAL_TIM_ID_MAX] = { DUAL_TIMER_IRQn, DUAL_TIMER_IRQn };
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
static const uint32_t    s_dual_tim_instance[APP_DUAL_TIM_ID_MAX] = { DUAL_TIMER0_BASE, DUAL_TIMER1_BASE };
#else
static const uint32_t    s_dual_tim_instance[APP_DUAL_TIM_ID_MAX] = { DUAL_TIM0_BASE, DUAL_TIM1_BASE };
#endif

static dual_tim_env_t *p_dual_tim_env[APP_DUAL_TIM_ID_MAX];

const static app_sleep_callbacks_t dual_tim_sleep_cb =
{
    .app_prepare_for_sleep = dual_tim_prepare_for_sleep,
    .app_wake_up_ind       = dual_tim_wake_up_ind,
};

void DUAL_TIMER_IRQHandler(void)
{
    if (p_dual_tim_env[0] != NULL)
    {
        if (p_dual_tim_env[0]->dual_tim_state != APP_DUAL_TIM_INVALID)
        {
            hal_dual_timer_irq_handler(&p_dual_tim_env[0]->handle);
        }
    }

    if (p_dual_tim_env[1] != NULL)
    {
        if (p_dual_tim_env[1]->dual_tim_state != APP_DUAL_TIM_INVALID)
        {
            hal_dual_timer_irq_handler(&p_dual_tim_env[1]->handle);
        }
    }
}

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool dual_tim_prepare_for_sleep(void)
{
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    for (uint32_t i = 0; i < APP_DUAL_TIM_ID_MAX; i++)
    {
        if (p_dual_tim_env[i] == NULL)
        {
            continue;
        }
        if (p_dual_tim_env[i]->dual_tim_state == APP_DUAL_TIM_ACTIVITY)
        {
            p_dual_tim_env[i]->dual_tim_state = APP_DUAL_TIM_SLEEP;
        }
    }
#endif

    return true;
}

SECTION_RAM_CODE static void dual_tim_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    for (uint32_t i = 0; i < APP_DUAL_TIM_ID_MAX; i++)
    {
        if (p_dual_tim_env[i] == NULL)
        {
            continue;
        }

        if (p_dual_tim_env[i]->dual_tim_state == APP_DUAL_TIM_ACTIVITY)
        {
            hal_nvic_clear_pending_irq(s_dual_tim_irq[i]);
            hal_nvic_enable_irq(s_dual_tim_irq[i]);

            hal_dual_timer_base_deinit(&p_dual_tim_env[i]->handle);
            hal_dual_timer_base_init(&p_dual_tim_env[i]->handle);
        }
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
static void dual_tim_wake_up(app_dual_tim_id_t id)
{
    if (p_dual_tim_env[id]->dual_tim_state == APP_DUAL_TIM_SLEEP)
    {
        hal_nvic_clear_pending_irq(s_dual_tim_irq[id]);
        hal_nvic_enable_irq(s_dual_tim_irq[id]);

        hal_dual_timer_base_deinit(&p_dual_tim_env[id]->handle);
        hal_dual_timer_base_init(&p_dual_tim_env[id]->handle);
        p_dual_tim_env[id]->dual_tim_state = APP_DUAL_TIM_ACTIVITY;
    }
}
#endif

static void app_dual_tim_event_call(dual_timer_handle_t *p_dual_tim, app_dual_tim_evt_t evt_type)
{
    app_dual_tim_evt_t dual_tim_evt = APP_DUAL_TIM_EVT_ERROR;
    app_dual_tim_id_t id = APP_DUAL_TIM_ID_0;

    if (p_dual_tim->p_instance == DUAL_TIMER0)
    {
        id = APP_DUAL_TIM_ID_0;
    }
    else if (p_dual_tim->p_instance == DUAL_TIMER1)
    {
        id = APP_DUAL_TIM_ID_1;
    }

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    if (evt_type == APP_DUAL_TIM_EVT_DONE)
    {
        dual_tim_evt = APP_DUAL_TIM_EVT_DONE;
    }
    else if(evt_type == APP_DUAL_TIM_EVT_ACT_START)
    {
        dual_tim_evt = APP_DUAL_TIM_EVT_ACT_START;
    }
    else if(evt_type == APP_DUAL_TIM_EVT_IOA_ACT_C1)
    {
        dual_tim_evt = APP_DUAL_TIM_EVT_IOA_ACT_C1;
    }
    else if(evt_type == APP_DUAL_TIM_EVT_IOA_ACT_C2)
    {
        dual_tim_evt = APP_DUAL_TIM_EVT_IOA_ACT_C2;
    }
    else if(evt_type == APP_DUAL_TIM_EVT_ACT_PERIOD)
    {
        dual_tim_evt = APP_DUAL_TIM_EVT_ACT_PERIOD;
    }
    else if(evt_type == APP_DUAL_TIM_EVT_ACT_STOP)
    {
        dual_tim_evt = APP_DUAL_TIM_EVT_ACT_STOP;
    }
    else if(evt_type == APP_DUAL_TIM_EVT_IOB_ACT_C1)
    {
        dual_tim_evt = APP_DUAL_TIM_EVT_IOB_ACT_C1;
    }
    else if(evt_type == APP_DUAL_TIM_EVT_IOB_ACT_C2)
    {
        dual_tim_evt = APP_DUAL_TIM_EVT_IOB_ACT_C2;
    }
    else if(evt_type == APP_DUAL_TIM_EVT_IOC_ACT_C1)
    {
        dual_tim_evt = APP_DUAL_TIM_EVT_IOC_ACT_C1;
    }
    else if(evt_type == APP_DUAL_TIM_EVT_IOC_ACT_C2)
    {
        dual_tim_evt = APP_DUAL_TIM_EVT_IOC_ACT_C2;
    }
#else
    if (evt_type == APP_DUAL_TIM_EVT_DONE)
    {
        dual_tim_evt = APP_DUAL_TIM_EVT_DONE;
    }
#endif

    if (p_dual_tim_env[id]->dual_tim_state == APP_DUAL_TIM_INVALID)
    {
        dual_tim_evt = APP_DUAL_TIM_EVT_ERROR;
    }

    if (p_dual_tim_env[id]->evt_handler != NULL)
    {
        p_dual_tim_env[id]->evt_handler(&dual_tim_evt);
    }
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
static uint16_t dual_timer_gpio_config(dual_timer_io_ctrl_cfg_t *io_crtl_cfg, app_dual_tim_id_t id, app_dual_tim_pin_t *pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if (id == APP_DUAL_TIM_ID_0)
    {
        switch (io_crtl_cfg->channel)
        {
            case HAL_DUAL_TIMER_CHANNEL_A:
                io_init.mux  = APP_IO_MUX_49;
                break;
            case HAL_DUAL_TIMER_CHANNEL_B:
                io_init.mux  = APP_IO_MUX_50;
                break;
            case HAL_DUAL_TIMER_CHANNEL_C:
                io_init.mux  = APP_IO_MUX_51;
                break;
            default:
                return APP_DRV_ERR_INVALID_PARAM;
        }
    }
    else if (id == APP_DUAL_TIM_ID_1)
    {
        switch (io_crtl_cfg->channel)
        {
            case HAL_DUAL_TIMER_CHANNEL_A:
                io_init.mux  = APP_IO_MUX_52;
                break;
            case HAL_DUAL_TIMER_CHANNEL_B:
                io_init.mux  = APP_IO_MUX_53;
                break;
            case HAL_DUAL_TIMER_CHANNEL_C:
                io_init.mux  = APP_IO_MUX_54;
                break;
            default:
                return APP_DRV_ERR_INVALID_PARAM;
        }
    }
    else
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    io_init.pull = APP_IO_NOPULL;
    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin  = pin_cfg->pin;
    err_code = app_io_init(pin_cfg->type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    return err_code;
}
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_dual_tim_init(app_dual_tim_params_t *p_params, app_dual_tim_evt_handler_t evt_handler)
{
    app_dual_tim_id_t id = p_params->id;
    hal_status_t  hal_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_DUAL_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if (0x0 == p_params->init.auto_reload)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    p_dual_tim_env[id] = &(p_params->dual_tim_env);

    p_dual_tim_env[id]->evt_handler = evt_handler;

    memcpy(&p_dual_tim_env[id]->handle.init, &p_params->init, sizeof(dual_timer_init_t));
    p_dual_tim_env[id]->handle.p_instance = (dual_timer_regs_t *)s_dual_tim_instance[id];
    hal_err_code = hal_dual_timer_base_deinit(&p_dual_tim_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_dual_timer_base_init(&p_dual_tim_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    pwr_register_sleep_cb(&dual_tim_sleep_cb, APP_DRIVER_DUAL_TIM_WAKEUP_PRIORITY, DUAL_TIM_PWR_ID);

    p_dual_tim_env[id]->dual_tim_state = APP_DUAL_TIM_ACTIVITY;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    p_dual_tim_env[id]->is_cha_enable = false;
    p_dual_tim_env[id]->is_chb_enable = false;
    p_dual_tim_env[id]->is_chc_enable = false;
#endif

    soc_register_nvic(DUAL_TIMER_IRQn, (uint32_t)DUAL_TIMER_IRQHandler);
    hal_nvic_clear_pending_irq(s_dual_tim_irq[id]);
    hal_nvic_enable_irq(s_dual_tim_irq[id]);

    return APP_DRV_SUCCESS;
}

uint16_t app_dual_tim_deinit(app_dual_tim_id_t id)
{
    hal_status_t  hal_err_code;

    if (id >= APP_DUAL_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_dual_tim_env[id] == NULL) || (p_dual_tim_env[id]->dual_tim_state == APP_DUAL_TIM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ( (p_dual_tim_env[0]->dual_tim_state == APP_DUAL_TIM_INVALID) &&
         (p_dual_tim_env[1]->dual_tim_state == APP_DUAL_TIM_INVALID))
    {
        hal_nvic_disable_irq(s_dual_tim_irq[id]);
    }

    p_dual_tim_env[id]->dual_tim_state = APP_DUAL_TIM_INVALID;

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    if (p_dual_tim_env[id]->is_cha_enable == true)
    {
        app_io_deinit(p_dual_tim_env[id]->cha_pin_cfg.type, p_dual_tim_env[id]->cha_pin_cfg.pin);
    }
    if (p_dual_tim_env[id]->is_chb_enable == true)
    {
        app_io_deinit(p_dual_tim_env[id]->chb_pin_cfg.type, p_dual_tim_env[id]->chb_pin_cfg.pin);
    }
    if (p_dual_tim_env[id]->is_chc_enable == true)
    {
        app_io_deinit(p_dual_tim_env[id]->chc_pin_cfg.type, p_dual_tim_env[id]->chc_pin_cfg.pin);
    }
    p_dual_tim_env[id]->is_cha_enable = false;
    p_dual_tim_env[id]->is_chb_enable = false;
    p_dual_tim_env[id]->is_chc_enable = false;
#endif

    GLOBAL_EXCEPTION_DISABLE();
    for (uint32_t i = 0; i < APP_DUAL_TIM_ID_MAX; i++)
    {
        if ((p_dual_tim_env[i]) && ((p_dual_tim_env[i]->dual_tim_state) != APP_DUAL_TIM_INVALID))
        {
            goto __deinit;
        }
    }
    pwr_unregister_sleep_cb(DUAL_TIM_PWR_ID);
__deinit:
    GLOBAL_EXCEPTION_ENABLE();

    hal_err_code = hal_dual_timer_base_deinit(&p_dual_tim_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    p_dual_tim_env[id] = NULL;

    return APP_DRV_SUCCESS;
}

uint16_t app_dual_tim_start(app_dual_tim_id_t id)
{
    hal_status_t err_code;

    if (id >= APP_DUAL_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_dual_tim_env[id] == NULL) || (p_dual_tim_env[id]->dual_tim_state == APP_DUAL_TIM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dual_tim_wake_up(id);
#endif

    err_code = hal_dual_timer_base_start_it(&p_dual_tim_env[id]->handle);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_dual_tim_stop(app_dual_tim_id_t id)
{
    hal_status_t err_code;

    if (id >= APP_DUAL_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_dual_tim_env[id] == NULL) || (p_dual_tim_env[id]->dual_tim_state == APP_DUAL_TIM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dual_tim_wake_up(id);
#endif

    err_code = hal_dual_timer_base_stop_it(&p_dual_tim_env[id]->handle);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_dual_tim_set_params(app_dual_tim_params_t *p_params, app_dual_tim_id_t id)
{
    hal_status_t err_code;

    if (id >= APP_DUAL_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_dual_tim_env[id] == NULL) || (p_dual_tim_env[id]->dual_tim_state == APP_DUAL_TIM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dual_tim_wake_up(id);
#endif

    err_code = hal_dual_timer_set_config(&p_dual_tim_env[id]->handle, &p_params->init);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_dual_tim_set_background_reload(app_dual_tim_id_t id, uint32_t reload_value)
{
    hal_status_t err_code;

    if (id >= APP_DUAL_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_dual_tim_env[id] == NULL) || (p_dual_tim_env[id]->dual_tim_state == APP_DUAL_TIM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (reload_value == 0x0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dual_tim_wake_up(id);
#endif

    err_code = hal_dual_timer_set_background_reload(&p_dual_tim_env[id]->handle, reload_value);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
uint16_t app_dual_tim_set_onetime_reload(app_dual_tim_id_t id, uint32_t reload_value)
{
    hal_status_t err_code;

    if (id >= APP_DUAL_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_dual_tim_env[id] == NULL) || (p_dual_tim_env[id]->dual_tim_state == APP_DUAL_TIM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (reload_value == 0x0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dual_tim_wake_up(id);
#endif

    err_code = hal_dual_timer_set_onetime_reload(&p_dual_tim_env[id]->handle, reload_value);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_dual_tim_set_period_count(app_dual_tim_id_t id, uint32_t count_value)
{
    hal_status_t err_code;

    if (id >= APP_DUAL_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_dual_tim_env[id] == NULL) || (p_dual_tim_env[id]->dual_tim_state == APP_DUAL_TIM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((p_dual_tim_env[id]->is_cha_enable == false &&
         p_dual_tim_env[id]->is_chb_enable == false &&
         p_dual_tim_env[id]->is_chc_enable == false) ||
        count_value < 0x2 ||
        count_value > 0xFFFF)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dual_tim_wake_up(id);
#endif

    err_code = hal_dual_timer_set_period_count(&p_dual_tim_env[id]->handle, count_value);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_dual_tim_io_crtl_config(app_dual_tim_id_t id, app_dual_tim_io_crtl_params_t *io_crtl_params)
{
    hal_status_t err_code;

    if (id >= APP_DUAL_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_dual_tim_env[id] == NULL) || (p_dual_tim_env[id]->dual_tim_state == APP_DUAL_TIM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (io_crtl_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dual_tim_wake_up(id);
#endif

    switch (io_crtl_params->io_crtl_cfg.channel)
    {
        case HAL_DUAL_TIMER_CHANNEL_A:
            p_dual_tim_env[id]->is_cha_enable = true;
            memcpy(&p_dual_tim_env[id]->cha_pin_cfg, &io_crtl_params->pin_cfg, sizeof(app_dual_tim_pin_t));
            break;
        case HAL_DUAL_TIMER_CHANNEL_B:
            p_dual_tim_env[id]->is_chb_enable = true;
            memcpy(&p_dual_tim_env[id]->chb_pin_cfg, &io_crtl_params->pin_cfg, sizeof(app_dual_tim_pin_t));
            break;
        case HAL_DUAL_TIMER_CHANNEL_C:
            p_dual_tim_env[id]->is_chc_enable = true;
            memcpy(&p_dual_tim_env[id]->chc_pin_cfg, &io_crtl_params->pin_cfg, sizeof(app_dual_tim_pin_t));
            break;
        default:
            return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = (hal_status_t)dual_timer_gpio_config(&io_crtl_params->io_crtl_cfg, id, &io_crtl_params->pin_cfg);
    HAL_ERR_CODE_CHECK(err_code);

    err_code = hal_dual_timer_io_crtl_config(&p_dual_tim_env[id]->handle, &io_crtl_params->io_crtl_cfg);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}
#endif

dual_timer_handle_t *app_dual_tim_get_handle(app_dual_tim_id_t id)
{
    if (id >= APP_DUAL_TIM_ID_MAX)
    {
        return NULL;
    }

    if ((p_dual_tim_env[id] == NULL) || (p_dual_tim_env[id]->dual_tim_state == APP_DUAL_TIM_INVALID))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dual_tim_wake_up(id);
#endif

    return &p_dual_tim_env[id]->handle;
}

void hal_dual_timer_period_elapsed_callback(dual_timer_handle_t *p_dual_timer)
{
    app_dual_tim_event_call(p_dual_timer, APP_DUAL_TIM_EVT_DONE);
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
void hal_dual_timer_act_start_event_callback(dual_timer_handle_t *p_dual_timer)
{
    app_dual_tim_event_call(p_dual_timer, APP_DUAL_TIM_EVT_ACT_START);
}

void hal_dual_timer_ioa_act_c1_event_callback(dual_timer_handle_t *p_dual_timer)
{
    app_dual_tim_event_call(p_dual_timer, APP_DUAL_TIM_EVT_IOA_ACT_C1);
}

void hal_dual_timer_ioa_act_c2_event_callback(dual_timer_handle_t *p_dual_timer)
{
    app_dual_tim_event_call(p_dual_timer, APP_DUAL_TIM_EVT_IOA_ACT_C2);
}

void hal_dual_timer_act_period_event_callback(dual_timer_handle_t *p_dual_timer)
{
    app_dual_tim_event_call(p_dual_timer, APP_DUAL_TIM_EVT_ACT_PERIOD);
}

void hal_dual_timer_act_stop_event_callback(dual_timer_handle_t *p_dual_timer)
{
    app_dual_tim_event_call(p_dual_timer, APP_DUAL_TIM_EVT_ACT_STOP);
}

void hal_dual_timer_iob_act_c1_event_callback(dual_timer_handle_t *p_dual_timer)
{
    app_dual_tim_event_call(p_dual_timer, APP_DUAL_TIM_EVT_IOB_ACT_C1);
}

void hal_dual_timer_iob_act_c2_event_callback(dual_timer_handle_t *p_dual_timer)
{
    app_dual_tim_event_call(p_dual_timer, APP_DUAL_TIM_EVT_IOB_ACT_C2);
}

void hal_dual_timer_ioc_act_c1_event_callback(dual_timer_handle_t *p_dual_timer)
{
    app_dual_tim_event_call(p_dual_timer, APP_DUAL_TIM_EVT_IOC_ACT_C1);
}

void hal_dual_timer_ioc_act_c2_event_callback(dual_timer_handle_t *p_dual_timer)
{
    app_dual_tim_event_call(p_dual_timer, APP_DUAL_TIM_EVT_IOC_ACT_C2);
}

#endif

#endif

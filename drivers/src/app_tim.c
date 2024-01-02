/**
  ****************************************************************************************
  * @file    app_tim.c
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
#include "app_tim.h"
#include "app_drv.h"
#include "string.h"
#include "app_pwr_mgmt.h"
#include "gr_soc.h"

#ifdef HAL_TIMER_MODULE_ENABLED

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool tim_prepare_for_sleep(void);
static void tim_wake_up_ind(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const IRQn_Type   s_tim_irq[APP_TIM_ID_MAX] = { TIMER0_IRQn, TIMER1_IRQn };
static const uint32_t    s_tim_instance[APP_TIM_ID_MAX] = { TIMER0_BASE, TIMER1_BASE };

tim_env_t *p_tim_env[APP_TIM_ID_MAX];

const static app_sleep_callbacks_t tim_sleep_cb =
{
    .app_prepare_for_sleep = tim_prepare_for_sleep,
    .app_wake_up_ind       = tim_wake_up_ind,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool tim_prepare_for_sleep(void)
{
    return false;
}

SECTION_RAM_CODE static void tim_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    uint32_t i;

    for (i = 0; i < APP_TIM_ID_MAX; i++)
    {
        if (p_tim_env[i] == NULL)
        {
            continue;
        }

        if (p_tim_env[i]->tim_state == APP_TIM_ACTIVITY)
        {
            hal_nvic_clear_pending_irq(s_tim_irq[i]);
            hal_nvic_enable_irq(s_tim_irq[i]);

            #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
            hal_timer_deinit(&p_tim_env[i]->handle);
            hal_timer_init(&p_tim_env[i]->handle);
            #else
            hal_timer_base_deinit(&p_tim_env[i]->handle);
            hal_timer_base_init(&p_tim_env[i]->handle);
            #endif
        }
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
static void tim_wake_up(app_tim_id_t id)
{
    if (p_tim_env[id]->tim_state == APP_TIM_SLEEP)
    {
        hal_nvic_clear_pending_irq(s_tim_irq[id]);
        hal_nvic_enable_irq(s_tim_irq[id]);

        hal_timer_base_deinit(&p_tim_env[id]->handle);
        hal_timer_base_init(&p_tim_env[id]->handle);
        p_tim_env[id]->tim_state = APP_TIM_ACTIVITY;
    }
}
#endif

static void app_tim_event_call(timer_handle_t *p_tim, app_tim_evt_t evt_type)
{
    app_tim_evt_t tim_evt = APP_TIM_EVT_ERROR;
    app_tim_id_t id = APP_TIM_ID_0;

    if(p_tim->p_instance == TIMER0)
    {
        id = APP_TIM_ID_0;
    }
    else if(p_tim->p_instance == TIMER1)
    {
        id = APP_TIM_ID_1;
    }

    if (evt_type == APP_TIM_EVT_DONE)
    {
        tim_evt = APP_TIM_EVT_DONE;
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    else if(evt_type == APP_TIM_EVT_CHANNEL0)
    {
        tim_evt = APP_TIM_EVT_CHANNEL0;
    }
    else if(evt_type == APP_TIM_EVT_CHANNEL1)
    {
        tim_evt = APP_TIM_EVT_CHANNEL1;
    }
    else if(evt_type == APP_TIM_EVT_CHANNEL2)
    {
        tim_evt = APP_TIM_EVT_CHANNEL2;
    }
    else if(evt_type == APP_TIM_EVT_CHANNEL3)
    {
        tim_evt = APP_TIM_EVT_CHANNEL3;
    }
#endif

    if (p_tim_env[id]->tim_state == APP_TIM_INVALID)
    {
        tim_evt = APP_TIM_EVT_ERROR;
    }

    if (p_tim_env[id]->evt_handler != NULL)
    {
        p_tim_env[id]->evt_handler(&tim_evt);
    }
}

#define TIMER_HANDLER(index, val) \
SECTION_RAM_CODE void TIMER##index##_IRQHandler(void)\
{\
    hal_timer_irq_handler(&p_tim_env[val]->handle);\
}

TIMER_HANDLER(0, APP_TIM_ID_0)
TIMER_HANDLER(1, APP_TIM_ID_1)

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_tim_init(app_tim_params_t *p_params, app_tim_evt_handler_t evt_handler)
{
    app_tim_id_t id = p_params->id;
    hal_status_t  hal_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }
    p_tim_env[id] = &p_params->tim_env;

    p_tim_env[id]->evt_handler = evt_handler;

    memcpy(&p_tim_env[id]->handle.init, &p_params->init, sizeof(timer_init_t));
    p_tim_env[id]->handle.p_instance = (timer_regs_t *)s_tim_instance[id];
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    hal_err_code = hal_timer_deinit(&p_tim_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    hal_err_code = hal_timer_init(&p_tim_env[id]->handle);
#else
    hal_err_code = hal_timer_base_deinit(&p_tim_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    hal_err_code = hal_timer_base_init(&p_tim_env[id]->handle);
#endif
    HAL_ERR_CODE_CHECK(hal_err_code);

    pwr_register_sleep_cb(&tim_sleep_cb, APP_DRIVER_TIM_WAKEUP_PRIORITY, TIM_PWR_ID);

    p_tim_env[id]->tim_state = APP_TIM_ACTIVITY;

    soc_register_nvic(TIMER0_IRQn, (uint32_t)TIMER0_IRQHandler);
    soc_register_nvic(TIMER1_IRQn, (uint32_t)TIMER1_IRQHandler);
    hal_nvic_clear_pending_irq(s_tim_irq[id]);
    hal_nvic_enable_irq(s_tim_irq[id]);

    return APP_DRV_SUCCESS;
}

uint16_t app_tim_deinit(app_tim_id_t id)
{
    hal_status_t  hal_err_code;

    if (id >= APP_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_tim_env[id] == NULL) || (p_tim_env[id]->tim_state == APP_TIM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    hal_nvic_disable_irq(s_tim_irq[id]);
    p_tim_env[id]->tim_state = APP_TIM_INVALID;

    GLOBAL_EXCEPTION_DISABLE();
    for (uint32_t i = 0; i < APP_TIM_ID_MAX; i++)
    {
        if ((p_tim_env[i]) && ((p_tim_env[i]->tim_state) != APP_TIM_INVALID))
        {
            goto __deinit;
        }
    }
    pwr_unregister_sleep_cb(TIM_PWR_ID);
__deinit:
    GLOBAL_EXCEPTION_ENABLE();

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    hal_err_code = hal_timer_deinit(&p_tim_env[id]->handle);
#else
    hal_err_code = hal_timer_base_deinit(&p_tim_env[id]->handle);
#endif
    HAL_ERR_CODE_CHECK(hal_err_code);
    p_tim_env[id] = NULL;

    return APP_DRV_SUCCESS;
}

uint16_t app_tim_start(app_tim_id_t id)
{
    hal_status_t err_code;

    if (id >= APP_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_tim_env[id] == NULL) || (p_tim_env[id]->tim_state == APP_TIM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    tim_wake_up(id);
#endif

    #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
        err_code = hal_timer_start_it(&p_tim_env[id]->handle);
    #else
        err_code = hal_timer_base_start_it(&p_tim_env[id]->handle);
    #endif
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_tim_stop(app_tim_id_t id)
{
    hal_status_t err_code;

    if (id >= APP_TIM_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_tim_env[id] == NULL) || (p_tim_env[id]->tim_state == APP_TIM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    tim_wake_up(id);
#endif

    #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
        err_code = hal_timer_stop_it(&p_tim_env[id]->handle);
    #else
        err_code = hal_timer_base_stop_it(&p_tim_env[id]->handle);
    #endif
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

timer_handle_t *app_tim_get_handle(app_tim_id_t id)
{
    if (id >= APP_TIM_ID_MAX)
    {
        return NULL;
    }

    if ((p_tim_env[id] == NULL) || (p_tim_env[id]->tim_state == APP_TIM_INVALID))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    tim_wake_up(id);
#endif

    return &p_tim_env[id]->handle;
}

void hal_timer_period_elapsed_callback(timer_handle_t *p_timer)
{
    app_tim_event_call(p_timer, APP_TIM_EVT_DONE);
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
void hal_timer_channel0_event_callback(timer_handle_t *p_timer)
{
    app_tim_event_call(p_timer, APP_TIM_EVT_CHANNEL0);
}

void hal_timer_channel1_event_callback(timer_handle_t *p_timer)
{
    app_tim_event_call(p_timer, APP_TIM_EVT_CHANNEL1);
}

void hal_timer_channel2_event_callback(timer_handle_t *p_timer)
{
    app_tim_event_call(p_timer, APP_TIM_EVT_CHANNEL2);
}

void hal_timer_channel3_event_callback(timer_handle_t *p_timer)
{
    app_tim_event_call(p_timer, APP_TIM_EVT_CHANNEL3);
}

uint32_t app_tim_get_channel0_val(app_tim_id_t id)
{
    if ((id >= APP_TIM_ID_MAX) || (p_tim_env[id] == NULL) || (p_tim_env[id]->tim_state == APP_TIM_INVALID))
    {
        return 0;
    }

    return hal_timer_get_channel0_val(&p_tim_env[id]->handle);
}

uint32_t app_tim_get_channel1_val(app_tim_id_t id)
{
    if ((id >= APP_TIM_ID_MAX) || (p_tim_env[id] == NULL) || (p_tim_env[id]->tim_state == APP_TIM_INVALID))
    {
        return 0;
    }

    return hal_timer_get_channel1_val(&p_tim_env[id]->handle);
}

uint32_t app_tim_get_channel2_val(app_tim_id_t id)
{
    if ((id >= APP_TIM_ID_MAX) || (p_tim_env[id] == NULL) || (p_tim_env[id]->tim_state == APP_TIM_INVALID))
    {
        return 0;
    }

    return hal_timer_get_channel2_val(&p_tim_env[id]->handle);
}

uint32_t app_tim_get_channel3_val(app_tim_id_t id)
{
    if ((id >= APP_TIM_ID_MAX) || (p_tim_env[id] == NULL) || (p_tim_env[id]->tim_state == APP_TIM_INVALID))
    {
        return 0;
    }

    return hal_timer_get_channel3_val(&p_tim_env[id]->handle);
}
#endif

#endif

/**
  ****************************************************************************************
  * @file    app_comp.c
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
#include "app_comp.h"
#include "app_pwr_mgmt.h"
#include "string.h"
#include "gr_soc.h"

#ifdef HAL_COMP_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool comp_prepare_for_sleep(void);
static void comp_wake_up_ind(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
comp_env_t *p_comp_env = NULL;

const static app_sleep_callbacks_t comp_sleep_cb =
{
    .app_prepare_for_sleep = comp_prepare_for_sleep,
    .app_wake_up_ind       = comp_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool comp_prepare_for_sleep(void)
{
    if (p_comp_env->comp_state == APP_COMP_ACTIVITY)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_comp_suspend_reg(&p_comp_env->handle);
        GLOBAL_EXCEPTION_ENABLE();

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
        p_comp_env->comp_state = APP_COMP_SLEEP;
#endif
    }

    return true;
}

SECTION_RAM_CODE static void comp_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    if (p_comp_env->comp_state == APP_COMP_ACTIVITY)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_comp_resume_reg(&p_comp_env->handle);
        GLOBAL_EXCEPTION_ENABLE();

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        hal_nvic_clear_pending_irq(COMP_IRQn);
        hal_nvic_enable_irq(COMP_IRQn);
#endif
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
static void comp_wake_up(void)
{
    if (p_comp_env->comp_state == APP_COMP_SLEEP)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_comp_resume_reg(&p_comp_env->handle);
        GLOBAL_EXCEPTION_ENABLE();

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        hal_nvic_clear_pending_irq(COMP_IRQn);
        hal_nvic_enable_irq(COMP_IRQn);
#else
        hal_nvic_clear_pending_irq(COMP_EXT_IRQn);
        hal_nvic_enable_irq(COMP_EXT_IRQn);
#endif
        p_comp_env->comp_state = APP_COMP_ACTIVITY;
    }
}
#endif

static uint16_t comp_config_gpio(uint32_t ref_source, app_comp_pin_cfg_t pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.mode = APP_IO_MODE_ANALOG;
    io_init.pull   = APP_IO_NOPULL;
    io_init.pin  = pin_cfg.input.pin;
    io_init.mux  = pin_cfg.input.mux;
    err_code = app_io_init(pin_cfg.input.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    if (ref_source != COMP_REF_SRC_VBAT && ref_source != COMP_REF_SRC_VREF)
    {
        io_init.pin  = pin_cfg.vref.pin;
        io_init.mux  = pin_cfg.vref.mux;
        err_code = app_io_init(pin_cfg.vref.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    return err_code;
}

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
void hal_comp_rising_trigger_callback(comp_handle_t *p_comp)
{
    app_comp_evt_t comp_evt = APP_COMP_EVT_RISING;

    if (p_comp_env->evt_handler != NULL)
    {
        p_comp_env->evt_handler(&comp_evt);
    }
}

void hal_comp_falling_trigger_callback(comp_handle_t *p_comp)
{
    app_comp_evt_t comp_evt = APP_COMP_EVT_FALLING;

    if (p_comp_env->evt_handler != NULL)
    {
        p_comp_env->evt_handler(&comp_evt);
    }
}
#else
static void app_comp_event_call(comp_handle_t *p_comp, app_comp_evt_t evt_type)
{
    app_comp_evt_t comp_evt = APP_COMP_EVT_ERROR;

    if (evt_type == APP_COMP_EVT_DONE)
    {
        comp_evt = APP_COMP_EVT_DONE;
    }

    if (p_comp_env->evt_handler != NULL)
    {
        p_comp_env->evt_handler(&comp_evt);
    }
}
void hal_comp_trigger_callback(comp_handle_t *p_comp)
{
    app_comp_event_call(p_comp, APP_COMP_EVT_DONE);
}
#endif

SECTION_RAM_CODE void COMP_IRQHandler(void)
{
    hal_comp_irq_handler(&p_comp_env->handle);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_comp_init(app_comp_params_t *p_params, app_comp_evt_handler_t evt_handler)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }
    p_comp_env = &p_params->comp_env;
    app_err_code = comp_config_gpio(p_params->init.ref_source, p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    p_comp_env->p_pin_cfg = &p_params->pin_cfg;
    p_comp_env->evt_handler = evt_handler;
    memcpy(&p_comp_env->handle.init, &p_params->init, sizeof(comp_init_t));
    hal_err_code = hal_comp_deinit(&p_comp_env->handle);
    APP_DRV_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_comp_init(&p_comp_env->handle);
    APP_DRV_ERR_CODE_CHECK(hal_err_code);

    pwr_register_sleep_cb(&comp_sleep_cb, APP_DRIVER_COMP_WAKEUP_PRIORITY, COMP_PWR_ID);

    p_comp_env->comp_state = APP_COMP_ACTIVITY;

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    soc_register_nvic(COMP_EXT_IRQn, (uint32_t)COMP_IRQHandler);
    hal_nvic_clear_pending_irq(COMP_EXT_IRQn);
    hal_nvic_enable_irq(COMP_EXT_IRQn);
#else
    soc_register_nvic(COMP_IRQn, (uint32_t)COMP_IRQHandler);
    hal_nvic_clear_pending_irq(COMP_IRQn);
    hal_nvic_enable_irq(COMP_IRQn);
#endif

    return 0;
}

uint16_t app_comp_deinit(void)
{
    hal_status_t  hal_err_code;

    if ((p_comp_env == NULL) || (p_comp_env->comp_state == APP_COMP_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
    app_drv_err_t app_err_code;
    app_err_code = app_io_deinit(p_comp_env->p_pin_cfg->input.type, p_comp_env->p_pin_cfg->input.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    if (p_comp_env->handle.init.ref_source != COMP_REF_SRC_VBAT &&
        p_comp_env->handle.init.ref_source != COMP_REF_SRC_VREF)
    {
        app_err_code = app_io_deinit(p_comp_env->p_pin_cfg->vref.type, p_comp_env->p_pin_cfg->vref.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }
#endif

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
    hal_nvic_disable_irq(COMP_IRQn);
#else
    hal_nvic_disable_irq(COMP_EXT_IRQn);
#endif

    p_comp_env->comp_state = APP_COMP_INVALID;

    pwr_unregister_sleep_cb(COMP_PWR_ID);

    hal_err_code = hal_comp_deinit(&p_comp_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    p_comp_env = NULL;

    return APP_DRV_SUCCESS;
}

uint16_t app_comp_start(void)
{
    hal_status_t  hal_err_code = HAL_ERROR;

    if ((p_comp_env == NULL) || (p_comp_env->comp_state == APP_COMP_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    comp_wake_up();
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    pwr_mgmt_wakeup_source_setup(PWR_WKUP_COND_MSIO_COMP);
#endif

    hal_err_code = hal_comp_start(&p_comp_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_comp_stop(void)
{
    hal_status_t  hal_err_code = HAL_ERROR;

    if ((p_comp_env == NULL) || (p_comp_env->comp_state == APP_COMP_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    comp_wake_up();
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    pwr_mgmt_wakeup_source_clear(PWR_WKUP_COND_MSIO_COMP);
#endif

    hal_err_code = hal_comp_stop(&p_comp_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

comp_handle_t *app_comp_get_handle(void)
{
    if ((p_comp_env == NULL) || (p_comp_env->comp_state == APP_COMP_INVALID))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    comp_wake_up();
#endif

    return &p_comp_env->handle;
}

#endif

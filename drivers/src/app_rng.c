/**
  ****************************************************************************************
  * @file    app_rng.c
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
#include "app_rng.h"
#include "app_pwr_mgmt.h"
#include "string.h"
#include "gr_soc.h"

#ifdef HAL_RNG_MODULE_ENABLED

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool rng_prepare_for_sleep(void);
static void rng_wake_up_ind(void);
void RNG_IRQHandler(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
rng_env_t *p_rng_env = NULL;

const static app_sleep_callbacks_t rng_sleep_cb =
{
    .app_prepare_for_sleep = rng_prepare_for_sleep,
    .app_wake_up_ind       = rng_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool rng_prepare_for_sleep(void)
{
    hal_rng_state_t state;

    if (p_rng_env->rng_state == APP_RNG_ACTIVITY)
    {
        state = hal_rng_get_state(&p_rng_env->handle);
        if ((state != HAL_RNG_STATE_READY) && (state != HAL_RNG_STATE_RESET))
        {
            return false;
        }

        GLOBAL_EXCEPTION_DISABLE();
        hal_rng_suspend_reg(&p_rng_env->handle);
        GLOBAL_EXCEPTION_ENABLE();
        #ifdef APP_DRIVER_WAKEUP_CALL_FUN
        p_rng_env->rng_state = APP_RNG_SLEEP;
        #endif
    }

    return true;
}

SECTION_RAM_CODE static void rng_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    if (p_rng_env->rng_state == APP_RNG_ACTIVITY)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_rng_resume_reg(&p_rng_env->handle);
        GLOBAL_EXCEPTION_ENABLE();
        if (p_rng_env->use_type == APP_RNG_TYPE_INTERRUPT)
        {
            hal_nvic_clear_pending_irq(RNG_IRQn);
            hal_nvic_enable_irq(RNG_IRQn);
        }
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
static void rng_wake_up(void)
{
    if (p_rng_env->rng_state == APP_RNG_SLEEP)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_rng_resume_reg(&p_rng_env->handle);
        GLOBAL_EXCEPTION_ENABLE();
        if (p_rng_env->use_type == APP_RNG_TYPE_INTERRUPT)
        {
            hal_nvic_clear_pending_irq(RNG_IRQn);
            hal_nvic_enable_irq(RNG_IRQn);
        }
        p_rng_env->rng_state = APP_RNG_ACTIVITY;
    }
}
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_rng_init(app_rng_params_t *p_params, app_rng_evt_handler_t evt_handler)
{
    hal_status_t  hal_err_code = HAL_OK;
    app_drv_err_t app_err_code = APP_DRV_SUCCESS;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }
    p_rng_env = &p_params->rng_env;

    p_rng_env->use_type = p_params->use_type;
    p_rng_env->evt_handler = evt_handler;

    memcpy(&p_rng_env->handle.init, &p_params->init, sizeof(rng_init_t));
    p_rng_env->handle.p_instance = RNG;
    hal_err_code = hal_rng_deinit(&p_rng_env->handle);
    APP_DRV_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_rng_init(&p_rng_env->handle);
    APP_DRV_ERR_CODE_CHECK(hal_err_code);

    pwr_register_sleep_cb(&rng_sleep_cb, APP_DRIVER_RNG_WAKEUP_PRIORITY, RNG_PWR_ID);

    p_rng_env->rng_state = APP_RNG_ACTIVITY;

    if (p_params->use_type == APP_RNG_TYPE_INTERRUPT)
    {
        soc_register_nvic(RNG_IRQn, (uint32_t)RNG_IRQHandler);
        hal_nvic_clear_pending_irq(RNG_IRQn);
        hal_nvic_enable_irq(RNG_IRQn);
    }

    return app_err_code;
}

uint16_t app_rng_deinit(void)
{
    hal_status_t  hal_err_code;

    if ((p_rng_env == NULL) ||  (p_rng_env->rng_state == APP_RNG_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    hal_nvic_disable_irq(RNG_IRQn);
    p_rng_env->rng_state = APP_RNG_INVALID;

    pwr_unregister_sleep_cb(RNG_PWR_ID);

    hal_err_code = hal_rng_deinit(&p_rng_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    p_rng_env = NULL;

    return APP_DRV_SUCCESS;
}

uint16_t app_rng_gen_sync(uint16_t *p_seed, uint32_t *p_random32bit)
{
    hal_status_t err_code;

    if ((p_rng_env == NULL) ||  (p_rng_env->rng_state == APP_RNG_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((p_random32bit == NULL) || ((p_seed == NULL) && (p_rng_env->handle.init.seed_mode != RNG_SEED_FR0_S0)))
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    rng_wake_up();
#endif

    err_code = hal_rng_generate_random_number(&p_rng_env->handle, p_seed, p_random32bit);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_rng_gen_async(uint16_t *p_seed)
{
    hal_status_t err_code;

    if ((p_rng_env == NULL) ||  (p_rng_env->rng_state == APP_RNG_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_rng_env->use_type == APP_RNG_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_MODE;
    }

    if ((p_seed == NULL) && (p_rng_env->handle.init.seed_mode != RNG_SEED_FR0_S0))
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    rng_wake_up();
#endif

    err_code = hal_rng_generate_random_number_it(&p_rng_env->handle, p_seed);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

rng_handle_t *app_rng_get_handle(void)
{
    if ((p_rng_env == NULL) ||  (p_rng_env->rng_state == APP_RNG_INVALID))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    rng_wake_up();
#endif

    return &p_rng_env->handle;
}

void hal_rng_ready_data_callback(rng_handle_t *p_rng, uint32_t random32bit)
{
    app_rng_evt_t rng_evt;
    rng_evt.type = APP_RNG_EVT_DONE;
    rng_evt.random_data = random32bit;

    if (p_rng_env->evt_handler != NULL)
    {
        p_rng_env->evt_handler(&rng_evt);
    }
}

void RNG_IRQHandler(void)
{
    hal_rng_irq_handler(&p_rng_env->handle);
}

#endif

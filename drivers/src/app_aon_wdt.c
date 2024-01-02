/**
  ****************************************************************************************
  * @file    app_aon_wdt.c
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
#include "app_aon_wdt.h"
#include "app_drv.h"
#include "gr_soc.h"
#include <string.h>

#ifdef HAL_AON_WDT_MODULE_ENABLED

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/*
 * EXTERN VARIABLE
 *****************************************************************************************
 */

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
extern uint32_t SystemSlowClock;
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

aon_aon_wdt_env_t  *p_aon_wdt_env = NULL;

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */

void AON_WDT_IRQHandler(void);

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void app_aon_wdt_event_call(aon_wdt_handle_t *p_aon_wdt)
{
    if(p_aon_wdt_env->evt_handler != NULL)
    {
        p_aon_wdt_env->evt_handler();
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_aon_wdt_init(app_aon_wdt_params_t *p_params, app_aon_wdt_evt_handler_t evt_handler)
{
    hal_status_t err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    p_aon_wdt_env = &p_params->aon_aon_wdt_env;

    memcpy(&p_aon_wdt_env->handle.init, &p_params->init, sizeof(aon_wdt_init_t));
    err_code = hal_aon_wdt_deinit(&p_aon_wdt_env->handle);
    HAL_ERR_CODE_CHECK(err_code);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    p_aon_wdt_env->handle.SystemCoreLowClock = &SystemSlowClock;
#endif

    err_code = hal_aon_wdt_init(&p_aon_wdt_env->handle);
    HAL_ERR_CODE_CHECK(err_code);

    p_aon_wdt_env->evt_handler = evt_handler;
    p_aon_wdt_env->aon_wdt_state = APP_AON_WDT_ACTIVITY;

    soc_register_nvic(AON_WDT_IRQn, (uint32_t)AON_WDT_IRQHandler);
    NVIC_ClearPendingIRQ(AON_WDT_IRQn);
    NVIC_EnableIRQ(AON_WDT_IRQn);

    return APP_DRV_SUCCESS;
}

uint16_t app_aon_wdt_deinit(void)
{
    hal_status_t err_code;

    if ((p_aon_wdt_env == NULL) || (p_aon_wdt_env->aon_wdt_state == APP_AON_WDT_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    err_code = hal_aon_wdt_deinit(&p_aon_wdt_env->handle);
    HAL_ERR_CODE_CHECK(err_code);

    p_aon_wdt_env = NULL;

    return APP_DRV_SUCCESS;
}

uint16_t app_aon_wdt_refresh(void)
{
    hal_status_t err_code;

    if ((p_aon_wdt_env == NULL) || (p_aon_wdt_env->aon_wdt_state == APP_AON_WDT_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    err_code = hal_aon_wdt_refresh(&p_aon_wdt_env->handle);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

void hal_aon_wdt_alarm_callback(aon_wdt_handle_t *p_aon_wdt)
{
    app_aon_wdt_event_call(p_aon_wdt);
}

void AON_WDT_IRQHandler(void)
{
    hal_aon_wdt_irq_handler(&p_aon_wdt_env->handle);
}

#endif


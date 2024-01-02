/**
  ****************************************************************************************
  * @file    app_bod.c
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
#include "app_bod.h"
#include "gr_soc.h"
#include <string.h>

#ifdef HAL_BOD_MODULE_ENABLED

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

bod_env_t  *p_bod_env = NULL;

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */

void BOD_ASSERT_IRQHandler(void);
void BOD_DEASSERT_IRQHandler(void);

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void app_bod_event_call(bod_handle_t *p_calendar, app_bod_evt_type_t evt_type)
{
    app_bod_evt_t bod_evt;

    bod_evt.type = evt_type;
    if(p_bod_env->evt_handler != NULL)
    {
        p_bod_env->evt_handler(&bod_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_bod_init(app_bod_params_t *p_params, app_bod_evt_handler_t evt_handler)
{
    hal_status_t err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }
    p_bod_env = &p_params->bod_env;

    memcpy(&p_bod_env->handle.init, &p_params->init, sizeof(bod_init_t));
    err_code = hal_bod_deinit(&p_bod_env->handle);
    HAL_ERR_CODE_CHECK(err_code);

    err_code = hal_bod_init(&p_bod_env->handle);
    HAL_ERR_CODE_CHECK(err_code);

    p_bod_env->evt_handler = evt_handler;
    p_bod_env->bod_state = APP_BOD_ACTIVITY;


    soc_register_nvic(BOD_ASSERT_IRQn, (uint32_t)BOD_ASSERT_IRQHandler);
    NVIC_ClearPendingIRQ(BOD_ASSERT_IRQn);
    NVIC_EnableIRQ(BOD_ASSERT_IRQn);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) ||(APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
    soc_register_nvic(BOD_DEASSERT_IRQn, (uint32_t)BOD_DEASSERT_IRQHandler);
    NVIC_ClearPendingIRQ(BOD_DEASSERT_IRQn);
    NVIC_EnableIRQ(BOD_DEASSERT_IRQn);
#endif

    return APP_DRV_SUCCESS;
}

uint16_t app_bod_deinit(void)
{
    hal_status_t err_code;

    if ((p_bod_env == NULL) || (p_bod_env->bod_state == APP_BOD_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    err_code = hal_bod_deinit(&p_bod_env->handle);
    HAL_ERR_CODE_CHECK(err_code);

    p_bod_env->bod_state = APP_BOD_INVALID;
    p_bod_env = NULL;

    return APP_DRV_SUCCESS;
}

uint16_t app_bod_enable(uint8_t enable)
{
    hal_status_t err_code;

    if ((p_bod_env == NULL) || (p_bod_env->bod_state == APP_BOD_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    err_code = hal_bod_enable(&p_bod_env->handle, enable);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_bod_event_enable(uint8_t enable)
{
    hal_status_t err_code;

    if ((p_bod_env == NULL) || (p_bod_env->bod_state == APP_BOD_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    err_code = hal_bod2_enable(&p_bod_env->handle, enable);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_bod_event_set_level(uint8_t level)
{
    hal_status_t err_code;

    if ((p_bod_env == NULL) || (p_bod_env->bod_state == APP_BOD_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    err_code = hal_bod2_set_level(&p_bod_env->handle, level);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_bod_static_mode_enable(uint8_t enable)
{
    hal_status_t err_code;

    if ((p_bod_env == NULL) || (p_bod_env->bod_state == APP_BOD_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    err_code = hal_bod_static_mode_enable(&p_bod_env->handle, enable);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
uint16_t app_bod_event_auto_power_bypass_enable(uint8_t enable)
{
    hal_status_t err_code;

    if ((p_bod_env == NULL) || (p_bod_env->bod_state == APP_BOD_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    err_code = hal_bod2_auto_power_bypass_enable(&p_bod_env->handle, enable);

    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

#endif

void hal_bod_fedge_callback(bod_handle_t *p_bod)
{
    app_bod_event_call(p_bod, APP_BOD_EVT_TRIGGERED);
}

void BOD_ASSERT_IRQHandler(void)
{
    hal_bod_fedge_irq_handler(&p_bod_env->handle);
}

/* BOD disappear event */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) ||(APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
void hal_bod_redge_callback(bod_handle_t *p_bod)
{
    app_bod_event_call(p_bod, APP_BOD_EVT_REMOVED);
}

void BOD_DEASSERT_IRQHandler(void)
{
    hal_bod_redge_irq_handler(&p_bod_env->handle);
}
#endif
#endif


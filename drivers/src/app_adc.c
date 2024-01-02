/**
  ****************************************************************************************
  * @file    app_adc.c
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
#include <string.h>
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "grx_sys.h"
#include "app_adc.h"

#ifdef HAL_ADC_MODULE_ENABLED

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
bool adc_prepare_for_sleep(void);
void adc_wake_up_ind(void);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
const uint32_t s_io_to_input_src[ADC_INPUT_SRC_REF + 1] =
{
    MSIO_PIN_0, MSIO_PIN_1, MSIO_PIN_2, MSIO_PIN_3, MSIO_PIN_4, 0, 0, 0
};
#else
const uint32_t s_io_to_input_src[ADC_INPUT_SRC_REF + 1] =
{
    MSIO_PIN_0, MSIO_PIN_1, MSIO_PIN_2, MSIO_PIN_3, MSIO_PIN_4, MSIO_PIN_5, MSIO_PIN_6, MSIO_PIN_7
};
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
adc_env_t *p_adc_env = NULL;

const static app_sleep_callbacks_t adc_sleep_cb =
{
    .app_prepare_for_sleep = adc_prepare_for_sleep,
    .app_wake_up_ind       = adc_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
bool adc_prepare_for_sleep(void)
{
    hal_adc_state_t state;

    if (p_adc_env->adc_state == APP_ADC_ACTIVITY)
    {
        state = hal_adc_get_state(&p_adc_env->handle);
        if ((state != HAL_ADC_STATE_READY) && (state != HAL_ADC_STATE_RESET))
        {
            return false;
        }

        GLOBAL_EXCEPTION_DISABLE();
        hal_adc_suspend_reg(&p_adc_env->handle);
        GLOBAL_EXCEPTION_ENABLE();

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
        p_adc_env.adc_state = APP_ADC_SLEEP;
#endif
    }

    return true;
}

SECTION_RAM_CODE void adc_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    if (p_adc_env->adc_state == APP_ADC_ACTIVITY)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_adc_resume_reg(&p_adc_env->handle);
        GLOBAL_EXCEPTION_ENABLE();
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
void adc_wake_up(void)
{
    if (p_adc_env->adc_state == APP_ADC_SLEEP)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_adc_resume_reg(&p_adc_env->handle);
        GLOBAL_EXCEPTION_ENABLE();

        p_adc_env->adc_state = APP_ADC_ACTIVITY;
    }

    if(p_adc_env->type == APP_ADC_TYPE_DMA)
    {
        dma_wake_up(p_adc_env->dma_id);
    }
}
#endif

static uint16_t adc_config_gpio(uint32_t input_mode, app_adc_pin_cfg_t pin_cfg, uint32_t ref_source)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.mode = APP_IO_MODE_ANALOG;
    io_init.pull   = APP_IO_NOPULL;
    if (input_mode == LL_ADC_INPUT_DIFFERENTIAL)
    {
        io_init.pin  = pin_cfg.channel_p.pin;
        io_init.mux  = pin_cfg.channel_p.mux;
        err_code = app_io_init(pin_cfg.channel_p.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    io_init.pin  = pin_cfg.channel_n.pin;
    io_init.mux  = pin_cfg.channel_n.mux;
    err_code = app_io_init(pin_cfg.channel_n.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    if (ref_source >= LL_ADC_REF_SRC_IO0)
    {
        io_init.pin  = pin_cfg.extern_ref.pin;
        io_init.mux  = pin_cfg.extern_ref.mux;
        err_code = app_io_init(pin_cfg.extern_ref.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    return err_code;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_adc_init(app_adc_params_t *p_params, app_adc_evt_handler_t evt_handler)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }
    p_adc_env = &(p_params->adc_env);

    app_err_code = adc_config_gpio(p_params->init.input_mode, p_params->pin_cfg, p_params->init.ref_source);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    p_adc_env->evt_handler = evt_handler;
    memcpy(&p_adc_env->handle.init, &p_params->init, sizeof(adc_init_t));
    hal_err_code = hal_adc_deinit(&p_adc_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    hal_err_code = hal_adc_init(&p_adc_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    pwr_register_sleep_cb(&adc_sleep_cb, APP_DRIVER_ADC_WAKEUP_PRIORITY, ADC_PWR_ID);
    p_adc_env->adc_state = APP_ADC_ACTIVITY;

    return APP_DRV_SUCCESS;
}

uint16_t app_adc_deinit(void)
{
    hal_status_t  hal_err_code;

    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    p_adc_env->adc_state = APP_ADC_INVALID;
    p_adc_env->p_current_sample_node = NULL;
    p_adc_env->multi_channel = 0;

    pwr_unregister_sleep_cb(ADC_PWR_ID);

    hal_err_code = hal_adc_deinit(&p_adc_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    if (p_adc_env->adc_dma_state == APP_ADC_DMA_INVALID)
    {
        p_adc_env = NULL;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_adc_conversion_sync(uint16_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t err_code;

    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    adc_wake_up();
#endif

    err_code = hal_adc_poll_for_conversion(&p_adc_env->handle, p_data, length);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
uint16_t app_adc_clock_start(void)
{
    hal_status_t err_code;

    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    err_code = hal_adc_clock_start(&p_adc_env->handle);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_adc_get_avg_voltage(float *p_data, uint32_t length)
{
    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if(length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    hal_adc_get_avg_voltage(&p_adc_env->handle, p_data, length);

    return 0;
}
#endif

uint16_t app_adc_conversion_async(uint16_t *p_data, uint32_t length)
{
    hal_status_t err_code;

    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (p_adc_env->adc_dma_state == APP_ADC_DMA_INVALID)
    {
        return APP_DRV_ERR_INVALID_MODE;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    adc_wake_up();
#endif

    err_code = hal_adc_start_dma(&p_adc_env->handle, p_data, length);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_adc_multi_channel_conversion_async(app_adc_sample_node_t *p_begin_node, uint32_t total_nodes)
{
    hal_status_t err_code;
    uint32_t check_node_num;
    app_adc_sample_node_t *p_check_node;

    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_begin_node == NULL || total_nodes == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (p_adc_env->adc_dma_state == APP_ADC_DMA_INVALID)
    {
        return APP_DRV_ERR_INVALID_MODE;
    }

    check_node_num = total_nodes;
    p_check_node = p_begin_node;
    while (check_node_num)//check samle link node
    {
        if( (p_check_node->channel > ADC_INPUT_SRC_REF) || (p_check_node->p_buf == NULL) || ((check_node_num>1)&&(p_check_node->next == NULL)))
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }

        if (--check_node_num)
        {
            p_check_node = p_check_node->next;
        }
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    adc_wake_up();
#endif

    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    io_init.mode = APP_IO_MODE_ANALOG;
    io_init.mux  = APP_IO_MUX;
    io_init.pull = APP_IO_NOPULL;
    check_node_num = total_nodes;
    p_check_node = p_begin_node;
    while (check_node_num)//config all msios
    {
        if (s_io_to_input_src[p_check_node->channel])
        {
            io_init.pin  = s_io_to_input_src[p_check_node->channel];
            app_io_init(APP_IO_TYPE_MSIO, &io_init);
        }

        if (--check_node_num)
        {
            p_check_node = p_check_node->next;
        }
    }

    p_adc_env->handle.init.input_mode = ADC_INPUT_SINGLE;//multi sample must under single mode
    p_adc_env->handle.init.channel_n = p_begin_node->channel;
    err_code = hal_adc_init(&p_adc_env->handle);
    HAL_ERR_CODE_CHECK(err_code);

    p_adc_env->p_current_sample_node = p_begin_node;
    p_adc_env->multi_channel = total_nodes;
    err_code = hal_adc_start_dma(&p_adc_env->handle, p_begin_node->p_buf, p_begin_node->len);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_adc_voltage_intern(uint16_t *inbuf, double *outbuf, uint32_t buflen)
{
    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (inbuf == NULL || outbuf == NULL || buflen == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    hal_adc_voltage_intern(&p_adc_env->handle, inbuf, outbuf, buflen);

    return APP_DRV_SUCCESS;
}

uint16_t app_adc_voltage_extern(double ref, uint16_t *inbuf, double *outbuf, uint32_t buflen)
{
    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (inbuf == NULL || outbuf == NULL || buflen == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    hal_adc_voltage_extern(&p_adc_env->handle, ref, inbuf, outbuf, buflen);

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
uint16_t app_adc_temperature_conv(uint16_t *inbuf, double *outbuf, uint32_t buflen)
{
    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (inbuf == NULL || outbuf == NULL || buflen == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    hal_adc_temperature_conv(&p_adc_env->handle, inbuf, outbuf, buflen);

    return APP_DRV_SUCCESS;
}

uint16_t app_adc_vbat_conv(uint16_t *inbuf, double *outbuf, uint32_t buflen)
{
    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (inbuf == NULL || outbuf == NULL || buflen == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    hal_adc_vbat_conv(&p_adc_env->handle, inbuf, outbuf, buflen);

    return APP_DRV_SUCCESS;
}
#endif

adc_handle_t *app_adc_get_handle(void)
{
    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    adc_wake_up();
#endif

    return &p_adc_env->handle;
}

void hal_adc_conv_cplt_callback(adc_handle_t *p_adc)
{
    app_adc_evt_t evt;

    if(p_adc_env->multi_channel > 0)
    {
        p_adc_env->multi_channel--;
    }

    if(p_adc_env->multi_channel == 0)
    {
        evt.type = APP_ADC_EVT_CONV_CPLT;

        if (p_adc_env->evt_handler != NULL)
        {
            p_adc_env->evt_handler(&evt);
        }
    }
    else
    {
        p_adc_env->p_current_sample_node = p_adc_env->p_current_sample_node->next;
        ll_adc_set_channeln(p_adc_env->p_current_sample_node->channel);
        hal_adc_start_dma(&p_adc_env->handle, p_adc_env->p_current_sample_node->p_buf, p_adc_env->p_current_sample_node->len);
    }
}

uint16_t adc_get_trim_func(adc_trim_info_t *p_adc_trim)
{
    return sys_adc_trim_get(p_adc_trim);
}

#endif

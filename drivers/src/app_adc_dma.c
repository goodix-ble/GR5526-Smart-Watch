/**
  ****************************************************************************************
  * @file    app_adc_dma.c
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
#include "app_adc_dma.h"

#ifdef HAL_ADC_MODULE_ENABLED

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
extern void adc_wake_up(void);
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
extern const uint32_t s_io_to_input_src[ADC_INPUT_SRC_REF + 1];
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern adc_env_t *p_adc_env;

static uint16_t adc_config_dma(app_adc_params_t *p_params)
{
    app_dma_params_t dma_params = { 0 };

    dma_params.p_instance                = p_params->dma_cfg.dma_instance;
    if (dma_params.p_instance != DMA0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    dma_params.channel_number            = p_params->dma_cfg.dma_channel;
    dma_params.init.src_request          = DMA0_REQUEST_SNSADC;
    dma_params.init.direction            = DMA_PERIPH_TO_MEMORY;
    dma_params.init.src_increment        = DMA_SRC_NO_CHANGE;
    dma_params.init.dst_increment        = DMA_DST_INCREMENT;
    dma_params.init.src_data_alignment   = DMA_SDATAALIGN_WORD;
    dma_params.init.dst_data_alignment   = DMA_DDATAALIGN_WORD;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    dma_params.init.mode                 = DMA_NORMAL;
#endif
    dma_params.init.priority             = DMA_PRIORITY_LOW;

    p_adc_env->dma_id = app_dma_init(&dma_params, NULL);
    if (p_adc_env->dma_id < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    p_adc_env->handle.p_dma = app_dma_get_handle(p_adc_env->dma_id);
    p_adc_env->handle.p_dma->p_parent = (void*)&p_adc_env->handle;

    return APP_DRV_SUCCESS;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_adc_dma_init(app_adc_params_t *p_params)
{
    app_drv_err_t app_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_adc_env->adc_dma_state != APP_ADC_DMA_INVALID)
    {
        return APP_DRV_ERR_INVALID_INIT;
    }

    GLOBAL_EXCEPTION_DISABLE();
    app_err_code = adc_config_dma(p_params);
    if (app_err_code != APP_DRV_SUCCESS)
    {
        goto __exit;
    }
    p_adc_env->adc_dma_state = APP_ADC_DMA_ACTIVITY;
__exit:
    GLOBAL_EXCEPTION_ENABLE();

    return app_err_code;
}

uint16_t app_adc_dma_deinit(void)
{
    if ((p_adc_env == NULL) || (p_adc_env->adc_dma_state != APP_ADC_DMA_ACTIVITY))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    app_dma_deinit(p_adc_env->dma_id);

    p_adc_env->adc_dma_state = APP_ADC_DMA_INVALID;

    return APP_DRV_SUCCESS;
}

uint16_t app_adc_dma_conversion_async(uint16_t *p_data, uint32_t length)
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

    err_code = hal_adc_start_dma(&p_adc_env->handle, p_data, length);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
uint16_t app_adc_dma_multi_channel_conversion_async(app_adc_sample_node_t *p_begin_node, uint32_t total_nodes)
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

    check_node_num = total_nodes;
    p_check_node = p_begin_node;
    while (check_node_num) //check samle link node
    {
        if ((p_check_node->channel > ADC_INPUT_SRC_REF) || (p_check_node->p_buf == NULL) || ((check_node_num>1)&&(p_check_node->next == NULL)))
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
    io_init.mux  = APP_IO_MUX_7;
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
#endif

#endif


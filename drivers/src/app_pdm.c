/**
  ****************************************************************************************
  * @file    app_pdm.c
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
#include "app_pdm.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include <string.h>
#include "gr_soc.h"

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool pdm_prepare_for_sleep(void);
static void pdm_wake_up_ind(void);
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
static void pdm_wake_up(void);
#endif
static uint16_t pdm_gpio_config(app_pdm_pin_cfg_t *p_pin_cfg);
static uint16_t pdm_config_dma(app_pdm_params_t *p_params);
static void app_pdm_event_call(pdm_handle_t *p_pdm, app_pdm_evt_type_t evt_type);
void PDM_IRQHandler(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const IRQn_Type s_pdm_irq      = PDM_IRQn;
static const pdm_regs_t *  s_pdm_instance = PDM;
static const app_sleep_callbacks_t pdm_sleep_cb =
{
    .app_prepare_for_sleep = pdm_prepare_for_sleep,
    .app_wake_up_ind       = pdm_wake_up_ind,
};

pdm_env_t *p_pdm_env = NULL;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool pdm_prepare_for_sleep(void)
{
    hal_pdm_state_t state = HAL_PDM_STATE_ERROR;

    if (APP_PDM_ACTIVITY == p_pdm_env->pdm_state)
    {
        state = hal_pdm_get_state(&p_pdm_env->handle);
        if ((state != HAL_PDM_STATE_RESET) && (state != HAL_PDM_STATE_READY))
        {
            return false;
        }
        GLOBAL_EXCEPTION_DISABLE();
        hal_pdm_suspend_reg(&p_pdm_env->handle);
        GLOBAL_EXCEPTION_ENABLE();
        #ifdef APP_DRIVER_WAKEUP_CALL_FUN
        p_pdm_env->pdm_state = APP_PDM_SLEEP;
        #endif
    }
    return true;
}

SECTION_RAM_CODE static void pdm_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    if (APP_PDM_ACTIVITY == p_pdm_env->pdm_state)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_pdm_resume_reg(&p_pdm_env->handle);
        GLOBAL_EXCEPTION_ENABLE();

        hal_nvic_clear_pending_irq(s_pdm_irq);
        hal_nvic_enable_irq(s_pdm_irq);
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
static void pdm_wake_up()
{
    if (APP_PDM_SLEEP == p_pdm_env->pdm_state)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_pdm_resume_reg(&p_pdm_env->handle);
        GLOBAL_EXCEPTION_ENABLE();

        hal_nvic_clear_pending_irq(s_pdm_irq);
        hal_nvic_enable_irq(s_pdm_irq);
        p_pdm_env->pdm_state = APP_PDM_ACTIVITY;
    }
}
#endif

static uint16_t pdm_gpio_config(app_pdm_pin_cfg_t *p_pin_cfg)
{
    app_io_init_t io_init  = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.pull = p_pin_cfg->clk.pull;
    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin  = p_pin_cfg->clk.pin;
    io_init.mux  = p_pin_cfg->clk.mux;
    err_code = app_io_init(p_pin_cfg->clk.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pull = p_pin_cfg->data.pull;
    io_init.pin  = p_pin_cfg->data.pin;
    io_init.mux  = p_pin_cfg->data.mux;
    err_code = app_io_init(p_pin_cfg->data.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    return err_code;
}

static uint16_t pdm_config_dma(app_pdm_params_t *p_params)
{
    app_dma_params_t dma_params = { 0 };

    dma_params.p_instance               = p_params->dma_cfg.dma_instance;
    dma_params.channel_number           = p_params->dma_cfg.dma_channel;
    dma_params.init.src_request         = DMA1_REQUEST_PDM_TX;
    dma_params.init.dst_request         = DMA1_REQUEST_MEM;
    dma_params.init.direction           = DMA_PERIPH_TO_MEMORY;
    dma_params.init.src_increment       = DMA_SRC_NO_CHANGE;
    dma_params.init.dst_increment       = DMA_DST_INCREMENT;
    dma_params.init.mode                = DMA_NORMAL;
    dma_params.init.priority            = DMA_PRIORITY_HIGH;
    if((PDM_MODE_LEFT == p_params->init.mode) || (PDM_MODE_RIGHT == p_params->init.mode))
    {
        dma_params.init.src_data_alignment  = DMA_SDATAALIGN_HALFWORD;
        dma_params.init.dst_data_alignment  = DMA_DDATAALIGN_HALFWORD;
    }
    else if(PDM_MODE_STEREO == p_params->init.mode)
    {
        dma_params.init.src_data_alignment  = DMA_SDATAALIGN_WORD;
        dma_params.init.dst_data_alignment  = DMA_DDATAALIGN_WORD;
    }
    p_pdm_env->dma_id = app_dma_init(&dma_params, NULL);

    if (p_pdm_env->dma_id < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    /* Associate the initialized DMA handle to the PDM handle */
    p_pdm_env->handle.p_dma = app_dma_get_handle(p_pdm_env->dma_id);
    p_pdm_env->handle.p_dma->p_parent = (void*)&p_pdm_env->handle;

    return APP_DRV_SUCCESS;
}

static void app_pdm_event_call(pdm_handle_t *p_pdm, app_pdm_evt_type_t evt_type)
{
    app_pdm_evt_t pdm_evt = { APP_PDM_EVT_DMA_ERROR, 0};

    pdm_evt.type = evt_type;

    if ((APP_PDM_EVT_DMA_ERROR == pdm_evt.type) || (APP_PDM_EVT_LEFT_OVERFLOW == pdm_evt.type) || \
        (APP_PDM_EVT_RIGHT_OVERFLOW == pdm_evt.type))
    {
        pdm_evt.error_code = p_pdm->error_code;
    }

    if (NULL != p_pdm_env->evt_handler)
    {
        p_pdm_env->evt_handler(&pdm_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_pdm_init(app_pdm_params_t *p_params, app_pdm_evt_handler_t evt_handler)
{
    app_drv_err_t err_code = APP_DRV_SUCCESS;
    hal_status_t hal_err_code = HAL_ERROR;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }
    p_pdm_env = &p_params->pdm_env;

    err_code = pdm_gpio_config(&p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(err_code);

    p_pdm_env->dma_id = -1;
    if (p_params->dma_cfg.dma_instance != NULL)
    {
        err_code = pdm_config_dma(p_params);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    p_pdm_env->p_pin_cfg = &p_params->pin_cfg;
    p_pdm_env->evt_handler = evt_handler;
    memcpy(&p_pdm_env->handle.init, &p_params->init, sizeof(pdm_init_t));
    p_pdm_env->handle.p_instance = (pdm_regs_t *)s_pdm_instance;

    hal_err_code = hal_pdm_deinit(&p_pdm_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    hal_err_code = hal_pdm_init(&p_pdm_env->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    pwr_register_sleep_cb(&pdm_sleep_cb, APP_DRIVER_PDM_WAKEUP_PRIORITY, PDM_PWR_ID);

    p_pdm_env->pdm_state = APP_PDM_ACTIVITY;

    soc_register_nvic(s_pdm_irq, (uint32_t)PDM_IRQHandler);
    hal_nvic_clear_pending_irq(s_pdm_irq);
    hal_nvic_enable_irq(s_pdm_irq);

    return APP_DRV_SUCCESS;
}

uint16_t app_pdm_deinit(void)
{
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if ((p_pdm_env == NULL) || (APP_PDM_INACTIVITY == p_pdm_env->pdm_state))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    hal_nvic_disable_irq(s_pdm_irq);

    err_code = app_io_deinit(p_pdm_env->p_pin_cfg->clk.type, p_pdm_env->p_pin_cfg->clk.pin);
    APP_DRV_ERR_CODE_CHECK(err_code);
    err_code = app_io_deinit(p_pdm_env->p_pin_cfg->data.type, p_pdm_env->p_pin_cfg->data.pin);
    APP_DRV_ERR_CODE_CHECK(err_code);

    app_dma_deinit(p_pdm_env->dma_id);

    p_pdm_env->pdm_state = APP_PDM_INACTIVITY;
    pwr_unregister_sleep_cb(PDM_PWR_ID);

    hal_pdm_deinit(&p_pdm_env->handle);
    p_pdm_env = NULL;

    return APP_DRV_SUCCESS;
}

uint16_t app_pdm_left_start_dma(uint16_t *p_data, uint32_t length)
{
    hal_status_t err_code = HAL_ERROR;

    if ((p_pdm_env == NULL) || (APP_PDM_INACTIVITY == p_pdm_env->pdm_state))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((NULL == p_data) || (0 == length))
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pdm_wake_up();
#endif

    err_code = hal_pdm_left_start_dma(&p_pdm_env->handle, p_data, length);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_pdm_right_start_dma(uint16_t *p_data, uint32_t length)
{
    hal_status_t err_code = HAL_ERROR;

    if ((p_pdm_env == NULL) || (APP_PDM_INACTIVITY == p_pdm_env->pdm_state))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((NULL == p_data) || (0 == length))
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pdm_wake_up();
#endif

    err_code = hal_pdm_right_start_dma(&p_pdm_env->handle, p_data, length);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_pdm_stereo_start_dma(uint32_t *p_data, uint32_t length)
{
    hal_status_t err_code = HAL_ERROR;

    if ((p_pdm_env == NULL) || (APP_PDM_INACTIVITY == p_pdm_env->pdm_state))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((NULL == p_data) || (0 == length))
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pdm_wake_up();
#endif

    err_code = hal_pdm_stereo_start_dma(&p_pdm_env->handle, p_data, length);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_pdm_left_start_dma_sg_llp(uint16_t *p_data, uint32_t length, dma_sg_llp_config_t *sg_llp_config)
{
    hal_status_t err_code = HAL_ERROR;

    if ((p_pdm_env == NULL) || (APP_PDM_INACTIVITY == p_pdm_env->pdm_state))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((NULL == p_data) || (0 == length) || (NULL == sg_llp_config))
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pdm_wake_up();
#endif

    err_code = hal_pdm_left_start_dma_sg_llp(&p_pdm_env->handle, p_data, length, sg_llp_config);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_pdm_right_start_dma_sg_llp(uint16_t *p_data, uint32_t length, dma_sg_llp_config_t *sg_llp_config)
{
    hal_status_t err_code = HAL_ERROR;

    if ((p_pdm_env == NULL) || (APP_PDM_INACTIVITY == p_pdm_env->pdm_state))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((NULL == p_data) || (0 == length) || (NULL == sg_llp_config))
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pdm_wake_up();
#endif

    err_code = hal_pdm_right_start_dma_sg_llp(&p_pdm_env->handle, p_data, length, sg_llp_config);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_pdm_stereo_start_dma_sg_llp(uint32_t *p_data, uint32_t length, dma_sg_llp_config_t *sg_llp_config)
{
    hal_status_t err_code = HAL_ERROR;

    if ((p_pdm_env == NULL) || (APP_PDM_INACTIVITY == p_pdm_env->pdm_state))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((NULL == p_data) || (0 == length) || (NULL == sg_llp_config))
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pdm_wake_up();
#endif

    err_code = hal_pdm_stereo_start_dma_sg_llp(&p_pdm_env->handle, p_data, length, sg_llp_config);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_pdm_abort(void)
{
    hal_status_t err_code = HAL_ERROR;

    if ((p_pdm_env == NULL) || (APP_PDM_INACTIVITY == p_pdm_env->pdm_state))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pdm_wake_up();
#endif

    err_code =  hal_pdm_abort(&p_pdm_env->handle);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

pdm_handle_t *app_pdm_get_handle(void)
{
    if ((p_pdm_env == NULL) || (APP_PDM_INACTIVITY == p_pdm_env->pdm_state))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pdm_wake_up();
#endif

    return &p_pdm_env->handle;
}

void hal_pdm_dma_cplt_callback(pdm_handle_t *p_pdm)
{
    app_pdm_event_call(p_pdm, APP_PDM_EVT_DMA_TFR);
}

void hal_pdm_dma_blk_callback(pdm_handle_t *p_pdm)
{
    app_pdm_event_call(p_pdm, APP_PDM_EVT_DMA_BLK);
}

void hal_pdm_dma_error_callback(pdm_handle_t *p_pdm)
{
    app_pdm_event_call(p_pdm, APP_PDM_EVT_DMA_ERROR);
}

void hal_pdm_left_overflow_callback(pdm_handle_t *p_pdm)
{
    app_pdm_event_call(p_pdm, APP_PDM_EVT_LEFT_OVERFLOW);

}

void hal_pdm_right_overflow_callback(pdm_handle_t *p_pdm)
{
    app_pdm_event_call(p_pdm, APP_PDM_EVT_RIGHT_OVERFLOW);

}

void PDM_IRQHandler(void)
{
    hal_pdm_irq_handler(&p_pdm_env->handle);
}



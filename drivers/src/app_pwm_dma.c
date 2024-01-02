/**
  ****************************************************************************************
  * @file    app_pwm_dma.c
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
#include "app_pwm_dma.h"
#include "app_dma.h"
#include "app_drv_config.h"
#include "app_pwr_mgmt.h"
#include <string.h>
#include "gr_soc.h"

#ifdef HAL_PWM_MODULE_ENABLED

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
extern bool pwm_prepare_for_sleep(void);
extern void pwm_sleep_canceled(void);
extern void pwm_wake_up_ind(void);
extern void pwm_wake_up(app_pwm_id_t id);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern pwm_env_t  *p_pwm_env[APP_PWM_ID_MAX];

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint16_t app_pwm_config_dma(app_pwm_params_t *p_params)
{
    app_dma_params_t dma_params;

    dma_params.p_instance     = p_params->use_mode.pwm_dma_instance;
    dma_params.channel_number = p_params->use_mode.pwm_dma_channel;

    if(dma_params.p_instance != DMA0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if(p_params->id >= APP_PWM_ID_1)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    dma_params.init.src_request         = DMA0_REQUEST_MEM;
    dma_params.init.dst_request         = DMA0_REQUEST_PWM0;
    dma_params.init.direction           = DMA_MEMORY_TO_PERIPH;
    dma_params.init.src_increment       = DMA_SRC_INCREMENT;
    dma_params.init.dst_increment       = DMA_DST_NO_CHANGE;
    dma_params.init.src_data_alignment  = DMA_SDATAALIGN_WORD;
    dma_params.init.dst_data_alignment  = DMA_DDATAALIGN_WORD;
    dma_params.init.priority            = DMA_PRIORITY_LOW;
    p_pwm_env[p_params->id]->dma_id[0]   = app_dma_init(&dma_params, NULL);

    if (p_pwm_env[p_params->id]->dma_id[0] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    /* Associate the initialized DMA handle to the UART handle */
    p_pwm_env[p_params->id]->handle.p_dma = app_dma_get_handle(p_pwm_env[p_params->id]->dma_id[0]);
    p_pwm_env[p_params->id]->handle.p_dma->p_parent = (void*)&p_pwm_env[p_params->id]->handle;

    return APP_DRV_SUCCESS;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_pwm_dma_init(app_pwm_params_t *p_params)
{
    app_drv_err_t app_err_code = APP_DRV_SUCCESS;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    uint8_t id = p_params->id;

    if (id != APP_PWM_ID_0)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->pwm_state == APP_PWM_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_pwm_env[id]->dma_state != APP_PWM_DMA_INVALID)
    {
        return APP_DRV_ERR_INVALID_INIT;
    }

    GLOBAL_EXCEPTION_DISABLE();
    app_err_code = app_pwm_config_dma(p_params);
    if (app_err_code != APP_DRV_SUCCESS)
    {
        goto __exit;
    }
    p_pwm_env[id]->dma_state = APP_PWM_DMA_ACTIVITY;
__exit:
    GLOBAL_EXCEPTION_ENABLE();

    return app_err_code;
}

uint16_t app_pwm_dma_deinit(app_pwm_id_t id)
{
    if (id != APP_PWM_ID_0)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) || (p_pwm_env[id]->dma_state != APP_PWM_DMA_ACTIVITY))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    app_dma_deinit(p_pwm_env[id]->dma_id[0]);

    p_pwm_env[id]->dma_id[0] = -1;
    p_pwm_env[id]->handle.p_dma = NULL;
    p_pwm_env[id]->dma_state = APP_PWM_DMA_INVALID;
    return APP_DRV_SUCCESS;
}

uint16_t app_pwm_start_coding_with_dma(app_pwm_id_t id, uint32_t *p_data, uint16_t size)
{
    hal_status_t err_code;

    if (id != APP_PWM_ID_0)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_pwm_env[id] == NULL) ||
        (p_pwm_env[id]->pwm_state == APP_PWM_INVALID) ||
        (p_pwm_env[id]->handle.p_dma == NULL))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    pwm_wake_up(id);
#endif

    err_code = hal_pwm_start_coding_with_dma(&p_pwm_env[id]->handle, p_data, size);
    HAL_ERR_CODE_CHECK(err_code);

    p_pwm_env[id]->pwm_module_state = APP_PWM_START;

    return APP_DRV_SUCCESS;
}

#endif

#endif  /* HAL_PWM_MODULE_ENABLED */


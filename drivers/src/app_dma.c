/**
  ****************************************************************************************
  * @file    app_dma.c
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
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include <string.h>
#include <stdbool.h>
#include "gr_soc.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
#define DMA_HANDLE_MAX            5
#elif (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
#define DMA_HANDLE_MAX            8
#elif (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
#define DMA_HANDLE_MAX            12
#endif

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool dma_prepare_for_sleep(void);
static void dma_wake_up_ind(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
dma_env_t s_dma_env[DMA_HANDLE_MAX];

static const app_sleep_callbacks_t dma_sleep_cb =
{
    .app_prepare_for_sleep  = dma_prepare_for_sleep,
    .app_wake_up_ind        = dma_wake_up_ind,
};

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
SECTION_RAM_CODE void DMA0_IRQHandler(void)
{
    uint32_t i;

    for (i = 0; i < DMA_HANDLE_MAX; i++)
    {
        if ((s_dma_env[i].dma_state == APP_DMA_ACTIVITY) && (s_dma_env[i].handle.p_instance == DMA0))
        {
             hal_dma_irq_handler(&s_dma_env[i].handle);
        }
    }
}
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
SECTION_RAM_CODE void DMA1_IRQHandler(void)
{
    uint32_t i;

    for (i = 0; i < DMA_HANDLE_MAX; i++)
    {
        if ((s_dma_env[i].dma_state == APP_DMA_ACTIVITY) && (s_dma_env[i].handle.p_instance == DMA1))
        {
             hal_dma_irq_handler(&s_dma_env[i].handle);
        }
    }
}
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
SECTION_RAM_CODE void DMA_IRQHandler(void)
{
    uint32_t i;

    for (i = 0; i < DMA_HANDLE_MAX; i++)
    {
        if (s_dma_env[i].dma_state == APP_DMA_ACTIVITY)
        {
             hal_dma_irq_handler(&s_dma_env[i].handle);
        }
    }
}
#endif

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool dma_prepare_for_sleep(void)
{
    hal_dma_state_t state;

    for (uint32_t i = 0; i < DMA_HANDLE_MAX; i++)
    {
        if (s_dma_env[i].dma_state == APP_DMA_ACTIVITY)
        {
            state = hal_dma_get_state(&s_dma_env[i].handle);
            if ((state != HAL_DMA_STATE_RESET) && (state != HAL_DMA_STATE_READY))
            {
                return false;
            }
            hal_dma_suspend_reg(&s_dma_env[i].handle);
            #ifdef APP_DRIVER_WAKEUP_CALL_FUN
            s_dma_env[i].dma_state = APP_DMA_SLEEP;
            #endif
        }
    }

    return true;
}

SECTION_RAM_CODE static void dma_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    bool find = false;
#endif

    for (uint32_t i = 0; i < DMA_HANDLE_MAX; i++)
    {
        if (s_dma_env[i].dma_state == APP_DMA_ACTIVITY)
        {
            hal_dma_resume_reg(&s_dma_env[i].handle);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
            find = true;
#endif

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            if (DMA0 == s_dma_env[i].handle.p_instance)
            {
                if (!NVIC_GetEnableIRQ(DMA0_IRQn))
                {
                    hal_nvic_clear_pending_irq(DMA0_IRQn);
                    hal_nvic_enable_irq(DMA0_IRQn);
                }
            }
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
            else if (DMA1 == s_dma_env[i].handle.p_instance)
            {
                if (!NVIC_GetEnableIRQ(DMA1_IRQn))
                {
                    hal_nvic_clear_pending_irq(DMA1_IRQn);
                    hal_nvic_enable_irq(DMA1_IRQn);
                }
            }
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
            if (find)
            {
                hal_nvic_clear_pending_irq(DMA_IRQn);
                hal_nvic_enable_irq(DMA_IRQn);
            }
#endif
        }
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
void dma_wake_up(dma_id_t id)
{
    if (id < 0 || id >= DMA_HANDLE_MAX)
        return;

    if (s_dma_env[id].dma_state == APP_DMA_SLEEP)
    {
        hal_dma_resume_reg(&s_dma_env[id].handle);
        s_dma_env[id].dma_state = APP_DMA_ACTIVITY;

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        if(DMA0 == s_dma_env[id].handle.p_instance)
        {
            if (!NVIC_GetEnableIRQ(DMA0_IRQn))
            {
                hal_nvic_clear_pending_irq(DMA0_IRQn);
                hal_nvic_enable_irq(DMA0_IRQn);
            }
        }
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        else if(DMA1 == s_dma_env[id].handle.p_instance)
        {
            if (!NVIC_GetEnableIRQ(DMA1_IRQn))
            {
                hal_nvic_clear_pending_irq(DMA1_IRQn);
                hal_nvic_enable_irq(DMA1_IRQn);
            }
        }
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        if (!NVIC_GetEnableIRQ(DMA_IRQn))
        {
            hal_nvic_clear_pending_irq(DMA_IRQn);
            hal_nvic_enable_irq(DMA_IRQn);
        }
#endif
    }
}
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void dma_tfr_callback(struct _dma_handle *hdma)
{
    uint32_t i;

    for (i = 0; i < DMA_HANDLE_MAX; i++)
    {
        if ((s_dma_env[i].dma_state == APP_DMA_ACTIVITY) &&
            (s_dma_env[i].handle.channel == hdma->channel)
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            && (s_dma_env[i].handle.p_instance == hdma->p_instance)
#endif
        )
        {
            if(NULL != s_dma_env[i].evt_handler)
            {
                s_dma_env[i].evt_handler(APP_DMA_EVT_TFR);
            }
            break;
        }
    }
}

void dma_err_callback(struct _dma_handle * hdma)
{
    uint32_t i;

    for (i = 0; i < DMA_HANDLE_MAX; i++)
    {
        if ((s_dma_env[i].dma_state == APP_DMA_ACTIVITY) &&
            (s_dma_env[i].handle.channel == hdma->channel)
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            && (s_dma_env[i].handle.p_instance == hdma->p_instance)
#endif
        )

        {
            if(NULL != s_dma_env[i].evt_handler)
            {
                s_dma_env[i].evt_handler(APP_DMA_EVT_ERROR);
            }
            break;
        }
    }
}

void dma_blk_callback(struct _dma_handle * hdma)
{
    uint32_t i;

    for (i = 0; i < DMA_HANDLE_MAX; i++)
    {
        if ((s_dma_env[i].dma_state == APP_DMA_ACTIVITY) &&
            (s_dma_env[i].handle.channel == hdma->channel)
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            && (s_dma_env[i].handle.p_instance == hdma->p_instance)
#endif
        )
        {
            if(NULL != s_dma_env[i].evt_handler)
            {
                s_dma_env[i].evt_handler(APP_DMA_EVT_BLK);
            }
            break;
        }
    }
}

dma_id_t app_dma_init(app_dma_params_t *p_params, app_dma_evt_handler_t evt_handler)
{
    uint32_t      i  = 0;
    dma_id_t     id = -1;
    hal_status_t status = HAL_ERROR;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    soc_register_nvic(DMA_IRQn, (uint32_t)DMA_IRQHandler);
#else
    soc_register_nvic(DMA0_IRQn, (uint32_t)DMA0_IRQHandler);
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    soc_register_nvic(DMA1_IRQn, (uint32_t)DMA1_IRQHandler);
#endif

    if (NULL != p_params)
    {
        if(!IS_DMA_ALL_INSTANCE(p_params->channel_number))
        {
            return -1;
        }
        GLOBAL_EXCEPTION_DISABLE();
        for (i = 0; i < DMA_HANDLE_MAX; i++)
        {
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
            if (s_dma_env[i].dma_state == APP_DMA_INVALID || \
                 s_dma_env[i].handle.channel == p_params->channel_number)
#else
            if (s_dma_env[i].dma_state == APP_DMA_INVALID || \
                (s_dma_env[i].handle.p_instance == p_params->p_instance && \
                 s_dma_env[i].handle.channel == p_params->channel_number))
#endif
            {
                if(HAL_DMA_STATE_BUSY == s_dma_env[i].handle.state)
                {
                    i = DMA_HANDLE_MAX;
                    break;
                }
                else
                {
                    id = i;
                    s_dma_env[i].dma_state = APP_DMA_ACTIVITY;
                    break;
                }
            }
        }
        GLOBAL_EXCEPTION_ENABLE();

        if (i < DMA_HANDLE_MAX)
        {
            pwr_register_sleep_cb(&dma_sleep_cb, APP_DRIVER_DMA_WAKEUP_PRIORITY, DMA_PWR_ID);
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            s_dma_env[i].handle.p_instance = p_params->p_instance;
#endif
            s_dma_env[i].handle.channel = p_params->channel_number;
            memcpy(&s_dma_env[i].handle.init, &p_params->init, sizeof(dma_init_t));
            s_dma_env[i].handle.xfer_tfr_callback   = dma_tfr_callback;
            s_dma_env[i].handle.xfer_error_callback = dma_err_callback;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            s_dma_env[i].handle.xfer_blk_callback   = dma_blk_callback;
#endif
            s_dma_env[i].handle.xfer_abort_callback = NULL;
            s_dma_env[i].evt_handler = evt_handler;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            if(DMA0 == s_dma_env[i].handle.p_instance)
            {
                if (!NVIC_GetEnableIRQ(DMA0_IRQn))
                {
                    hal_nvic_clear_pending_irq(DMA0_IRQn);
                    hal_nvic_enable_irq(DMA0_IRQn);
                }
            }
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
            else if (DMA1 == s_dma_env[i].handle.p_instance)
            {
                if (!NVIC_GetEnableIRQ(DMA1_IRQn))
                {
                    hal_nvic_clear_pending_irq(DMA1_IRQn);
                    hal_nvic_enable_irq(DMA1_IRQn);
                }
            }
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
            hal_nvic_clear_pending_irq(DMA_IRQn);
            hal_nvic_enable_irq(DMA_IRQn);
#endif

            status = hal_dma_init(&s_dma_env[i].handle);
        }
    }

    if (HAL_OK != status)
    {
        id = -1;
    }

    return id;
}

uint16_t app_dma_deinit(dma_id_t id)
{
    uint32_t i;

    if ((id < 0) || (id >= DMA_HANDLE_MAX))
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if (s_dma_env[id].dma_state == APP_DMA_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    GLOBAL_EXCEPTION_DISABLE();
    hal_dma_deinit(&s_dma_env[id].handle);
    s_dma_env[id].dma_state = APP_DMA_INVALID;
    s_dma_env[id].handle.channel = (dma_channel_t)(-1);

    for (i = 0; i < DMA_HANDLE_MAX; i++)
    {
        if (s_dma_env[i].dma_state == APP_DMA_ACTIVITY)
        {
            break;
        }
    }

    if (i == DMA_HANDLE_MAX)
    {
        pwr_unregister_sleep_cb(DMA_PWR_ID);
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        hal_nvic_disable_irq(DMA0_IRQn);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        hal_nvic_disable_irq(DMA1_IRQn);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        hal_nvic_disable_irq(DMA_IRQn);
#endif
    }
    GLOBAL_EXCEPTION_ENABLE();

    return APP_DRV_SUCCESS;
}

dma_handle_t *app_dma_get_handle(dma_id_t id)
{
    if ((id < 0) || (id >= DMA_HANDLE_MAX))
    {
        return NULL;
    }

    if (s_dma_env[id].dma_state == APP_DMA_INVALID)
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dma_wake_up(id);
#endif

    return &s_dma_env[id].handle;
}

uint16_t app_dma_start(dma_id_t id, uint32_t src_address, uint32_t dst_address, uint32_t data_length)
{
    hal_status_t status = HAL_ERROR;

    if ((id < 0) || (id >= DMA_HANDLE_MAX))
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if (s_dma_env[id].dma_state == APP_DMA_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((data_length < 1) || (data_length > 0xFFF))
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dma_wake_up(id);
#endif

    status = hal_dma_start_it(&s_dma_env[id].handle, src_address, dst_address, data_length);
    if (HAL_OK != status)
    {
        return (uint16_t)status;
    }

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
uint16_t app_dma_start_sg_llp(dma_id_t id, uint32_t src_address, uint32_t dst_address, uint32_t data_length, dma_sg_llp_config_t *sg_llp_config)
{
    hal_status_t status = HAL_ERROR;

    if ((id < 0) || (id >= DMA_HANDLE_MAX))
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if (s_dma_env[id].dma_state == APP_DMA_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((data_length < 1) || (data_length > 0xFFF))
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dma_wake_up(id);
#endif

    status = hal_dma_start_sg_llp_it(&s_dma_env[id].handle, src_address, dst_address, data_length, sg_llp_config);
    if (HAL_OK != status)
    {
        return (uint16_t)status;
    }

    return APP_DRV_SUCCESS;
}
#endif


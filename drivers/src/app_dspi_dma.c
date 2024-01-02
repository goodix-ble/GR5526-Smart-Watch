/**
  ****************************************************************************************
  * @file    app_dspi_dma.c
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
#include "app_dspi.h"
#include "app_dspi_dma.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include <string.h>

#ifdef HAL_DSPI_MODULE_ENABLED
/*
 * DEFINES
 *****************************************************************************************
 */

#define DSPI_SMART_CS_LOW()                                                                         \
    do                                                                                              \
    {                                                                                               \
            if((p_dspi_env->p_pin_cfg->cs.enable == APP_DSPI_PIN_ENABLE) && p_dspi_env->is_soft_cs) \
            {                                                                                       \
                app_io_write_pin(p_dspi_env->p_pin_cfg->cs.type,                                    \
                                p_dspi_env->p_pin_cfg->cs.pin,                                      \
                                APP_IO_PIN_RESET);                                                  \
            }                                                                                       \
    } while(0)

#define DSPI_SMART_CS_HIGH()                                                                        \
    do                                                                                              \
    {                                                                                               \
            if((p_dspi_env->p_pin_cfg->cs.enable == APP_DSPI_PIN_ENABLE) && p_dspi_env->is_soft_cs) \
            {                                                                                       \
                app_io_write_pin(p_dspi_env->p_pin_cfg->cs.type,                                    \
                                 p_dspi_env->p_pin_cfg->cs.pin,                                     \
                                 APP_IO_PIN_SET);                                                   \
            }                                                                                       \
    } while(0)


/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
extern bool dspi_prepare_for_sleep(void);
extern void dspi_wake_up_ind(void);
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
extern void dspi_wake_up(void);
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern dspi_env_t *p_dspi_env;

static uint16_t app_dspi_config_dma(app_dspi_params_t *p_params)
{
    app_dma_params_t dma;

    dma.p_instance                 = DMA1;
    dma.channel_number             = p_params->dma_cfg.channel;
    dma.init.src_request           = DMA1_REQUEST_MEM;
    dma.init.dst_request           = DMA1_REQUEST_DSPIM_TX;
    dma.init.direction             = DMA_MEMORY_TO_PERIPH;
    dma.init.src_increment         = DMA_SRC_INCREMENT;
    dma.init.dst_increment         = DMA_DST_NO_CHANGE;
    if (p_params->init.data_size <= DSPI_DATASIZE_08_BITS)
    {
        dma.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
        dma.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
    }
    else if (p_params->init.data_size <= DSPI_DATASIZE_16_BITS)
    {
        dma.init.src_data_alignment    = DMA_SDATAALIGN_HALFWORD;
        dma.init.dst_data_alignment    = DMA_DDATAALIGN_HALFWORD;
    }
    else
    {
        dma.init.src_data_alignment    = DMA_SDATAALIGN_WORD;
        dma.init.dst_data_alignment    = DMA_DDATAALIGN_WORD;
    }
    dma.init.mode                  = DMA_NORMAL;
    dma.init.priority              = DMA_PRIORITY_LOW;

    p_dspi_env->dma_id = app_dma_init(&dma, NULL);
    if (p_dspi_env->dma_id < 0 || p_dspi_env->dma_id>= 12)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    p_dspi_env->handle.p_dmatx = app_dma_get_handle(p_dspi_env->dma_id);
    p_dspi_env->handle.p_dmatx->p_parent = (void*)&p_dspi_env->handle;

    return APP_DRV_SUCCESS;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_dspi_dma_init(app_dspi_params_t *p_params)
{
    app_drv_err_t app_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((p_dspi_env->dspi_dma_state != APP_DSPI_DMA_INVALID))
    {
        return APP_DRV_ERR_INVALID_INIT;
    }

    GLOBAL_EXCEPTION_DISABLE();
    app_err_code = app_dspi_config_dma(p_params);
    if (app_err_code != APP_DRV_SUCCESS)
    {
        goto __exit;
    }
    p_dspi_env->dspi_dma_state = APP_DSPI_DMA_ACTIVITY;
__exit:
    GLOBAL_EXCEPTION_ENABLE();

    return app_err_code;
}

uint16_t app_dspi_dma_deinit(void)
{
    if ((p_dspi_env == NULL) ||
        (p_dspi_env->dspi_dma_state != APP_DSPI_DMA_ACTIVITY))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    app_dma_deinit(p_dspi_env->dma_id);
    p_dspi_env->dspi_dma_state = APP_DSPI_DMA_INVALID;

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_dma_command_transmit_async(app_dspi_command_t *p_cmd, uint8_t *p_data)
{
    hal_status_t err_code = HAL_OK;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL || p_data == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    if (p_dspi_env->start_flag == false)
    {
        DSPI_SMART_CS_LOW();
        p_dspi_env->start_flag = true;
        err_code = hal_dspi_command_transmit_dma(&p_dspi_env->handle, p_cmd, p_data);
        if (err_code != HAL_OK)
        {
            DSPI_SMART_CS_HIGH();
            p_dspi_env->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_dma_command_async(app_dspi_command_t *p_cmd)
{
    hal_status_t err_code = HAL_ERROR;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    if (p_dspi_env->start_flag == false)
    {
        DSPI_SMART_CS_LOW();
        p_dspi_env->start_flag = true;
        err_code = hal_dspi_command_dma(&p_dspi_env->handle, p_cmd);
        if (err_code != HAL_OK)
        {
            DSPI_SMART_CS_HIGH();
            p_dspi_env->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_dma_transmit_async(uint8_t *p_data, uint32_t length)
{
    hal_status_t err_code = HAL_OK;

    if ((p_dspi_env == NULL) || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    if (p_dspi_env->start_flag == false)
    {
        DSPI_SMART_CS_LOW();
        p_dspi_env->start_flag = true;
        err_code = hal_dspi_transmit_dma(&p_dspi_env->handle, p_data, length);
        if (err_code != HAL_OK)
        {
            DSPI_SMART_CS_HIGH();
            p_dspi_env->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_dspi_dma_sg_llp_transmit_async(uint8_t *p_data, uint32_t length, dma_sg_llp_config_t *sg_llp_config)
{
    hal_status_t err_code = HAL_OK;

    if ((p_dspi_env == NULL) || sg_llp_config == NULL || (p_dspi_env->dspi_state == APP_DSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    dspi_wake_up();
#endif

    if (p_dspi_env->start_flag == false)
    {
        DSPI_SMART_CS_LOW();
        p_dspi_env->start_flag = true;
        err_code = hal_dspi_transmit_dma_sg_llp(&p_dspi_env->handle, p_data, length, sg_llp_config);
        if (err_code != HAL_OK)
        {
            DSPI_SMART_CS_HIGH();
            p_dspi_env->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

#endif

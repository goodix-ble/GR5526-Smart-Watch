/**
  ****************************************************************************************
  * @file    app_i2c.c
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
#include "app_i2c.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include <string.h>
#include "gr_soc.h"
#include "app_drv.h"

#ifdef HAL_I2C_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */


/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
typedef struct {
    app_i2c_id_t id;
    IRQn_Type    irq;
    i2c_regs_t  *instance;
} i2c_info_t;

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool i2c_prepare_for_sleep(void);
static void i2c_wake_up_ind(void);
static uint16_t i2c_gpio_config(app_i2c_pin_cfg_t *p_pin_cfg);
void I2C0_IRQHandler(void);
void I2C1_IRQHandler(void);
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
void I2C2_IRQHandler(void);
void I2C3_IRQHandler(void);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
void I2C4_IRQHandler(void);
void I2C5_IRQHandler(void);
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const i2c_info_t s_i2c_info[APP_I2C_ID_MAX] =
{
    {
        .id  = APP_I2C_ID_0,
        .irq = I2C0_IRQn,
        .instance = I2C0,
    },
    {
        .id  = APP_I2C_ID_1,
        .irq = I2C1_IRQn,
        .instance = I2C1,
    },
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    {
        .id  = APP_I2C_ID_2,
        .irq = I2C2_IRQn,
        .instance = I2C2,
    },
    {
        .id  = APP_I2C_ID_3,
        .irq = I2C3_IRQn,
        .instance = I2C3,
    },
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
    {
        .id  = APP_I2C_ID_4,
        .irq = I2C4_IRQn,
        .instance = I2C4,
    },
    {
        .id  = APP_I2C_ID_5,
        .irq = I2C5_IRQn,
        .instance = I2C5,
    },
#endif
};

i2c_env_t *p_i2c_env[APP_I2C_ID_MAX];
static const app_sleep_callbacks_t i2c_sleep_cb =
{
    .app_prepare_for_sleep = i2c_prepare_for_sleep,
    .app_wake_up_ind       = i2c_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool i2c_prepare_for_sleep(void)
{
    hal_i2c_state_t state;
    for (uint32_t i = 0; i < APP_I2C_ID_MAX; i++)
    {
        if (p_i2c_env[i] == NULL)
        {
            continue;
        }

        if (p_i2c_env[i]->i2c_state == APP_I2C_ACTIVITY)
        {
            state = hal_i2c_get_state(&(p_i2c_env[i]->handle));
            if ((state != HAL_I2C_STATE_READY) && (state != HAL_I2C_STATE_RESET))
            {
                return false;
            }

            GLOBAL_EXCEPTION_DISABLE();
            hal_i2c_suspend_reg(&p_i2c_env[i]->handle);
            GLOBAL_EXCEPTION_ENABLE();

            #ifdef APP_DRIVER_WAKEUP_CALL_FUN
            p_i2c_env[i]->i2c_state = APP_I2C_SLEEP;
            #endif
        }
    }

    return true;
}

SECTION_RAM_CODE static void i2c_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    for (uint32_t i = 0; i < APP_I2C_ID_MAX; i++)
    {
        if (p_i2c_env[i] == NULL)
        {
            continue;
        }

        if (p_i2c_env[i]->i2c_state == APP_I2C_ACTIVITY)
        {
            GLOBAL_EXCEPTION_DISABLE();
            hal_i2c_resume_reg(&p_i2c_env[i]->handle);
            GLOBAL_EXCEPTION_ENABLE();

            hal_nvic_clear_pending_irq(s_i2c_info[i].irq);
            hal_nvic_enable_irq(s_i2c_info[i].irq);
        }
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
void i2c_wake_up(app_i2c_id_t id)
{
    if (p_i2c_env[id]->i2c_state == APP_I2C_SLEEP)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_i2c_resume_reg(&p_i2c_env[id]->handle);
        GLOBAL_EXCEPTION_ENABLE();

        hal_nvic_clear_pending_irq(s_i2c_irq[id]);
        hal_nvic_enable_irq(s_i2c_irq[id]);
        p_i2c_env[id]->i2c_state = APP_I2C_ACTIVITY;

        dma_wake_up(p_i2c_env[id]->dma_id[0]);
        dma_wake_up(p_i2c_env[id]->dma_id[1]);
    }
}
#endif

static uint16_t i2c_gpio_config(app_i2c_pin_cfg_t *p_pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.pull = p_pin_cfg->scl.pull;
    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin  = p_pin_cfg->scl.pin;
    io_init.mux  = p_pin_cfg->scl.mux;
    err_code = app_io_init(p_pin_cfg->scl.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pull = p_pin_cfg->sda.pull;
    io_init.pin  = p_pin_cfg->sda.pin;
    io_init.mux  = p_pin_cfg->sda.mux;
    err_code = app_io_init(p_pin_cfg->sda.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    return err_code;
}

static app_i2c_id_t i2c_get_id(i2c_handle_t *p_i2c)
{
    for (uint32_t i = 0; i < APP_I2C_ID_MAX; i++)
    {
        if (p_i2c->p_instance == s_i2c_info[i].instance)
        {
            return s_i2c_info[i].id;
        }
    }

    return APP_I2C_ID_MAX;
}

static void app_i2c_event_call(i2c_handle_t *p_i2c, app_i2c_evt_type_t evt_type)
{
    app_i2c_evt_t i2c_evt;
    app_i2c_id_t id = i2c_get_id(p_i2c);

    i2c_evt.type = evt_type;
    if (evt_type == APP_I2C_EVT_ERROR)
    {
        i2c_evt.data.error_code = p_i2c->error_code;
    }
    else
    {
        i2c_evt.data.size = p_i2c->xfer_size - p_i2c->xfer_count;
    }

    i2c_evt.slave_addr = p_i2c_env[id]->slv_dev_addr;
    p_i2c_env[id]->start_flag = false;
    if (p_i2c_env[id]->evt_handler != NULL)
    {
        p_i2c_env[id]->evt_handler(&i2c_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_i2c_init(app_i2c_params_t *p_params, app_i2c_evt_handler_t evt_handler)
{

    app_i2c_id_t  id = p_params->id;
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    p_i2c_env[id] = &(p_params->i2c_dev);

    app_err_code = i2c_gpio_config(&p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    p_params->i2c_dev.role = p_params->role;
    p_params->i2c_dev.p_pin_cfg = &p_params->pin_cfg;
    p_params->i2c_dev.evt_handler = evt_handler;

    memcpy(&p_params->i2c_dev.handle.init, &p_params->init, sizeof(i2c_init_t));
    p_params->i2c_dev.handle.p_instance = s_i2c_info[p_params->id].instance;
    hal_err_code = hal_i2c_deinit(&p_params->i2c_dev.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_i2c_init(&p_params->i2c_dev.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    pwr_register_sleep_cb(&i2c_sleep_cb, APP_DRIVER_I2C_WAKEUP_PRIORITY, I2C_PWR_ID);

    p_params->i2c_dev.i2c_state = APP_I2C_ACTIVITY;
    p_params->i2c_dev.start_flag = false;

    soc_register_nvic(I2C0_IRQn, (uint32_t)I2C0_IRQHandler);
    soc_register_nvic(I2C1_IRQn, (uint32_t)I2C1_IRQHandler);
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    soc_register_nvic(I2C2_IRQn, (uint32_t)I2C2_IRQHandler);
    soc_register_nvic(I2C3_IRQn, (uint32_t)I2C3_IRQHandler);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
    soc_register_nvic(I2C4_IRQn, (uint32_t)I2C4_IRQHandler);
    soc_register_nvic(I2C5_IRQn, (uint32_t)I2C5_IRQHandler);
#endif
    hal_nvic_clear_pending_irq(s_i2c_info[p_params->id].irq);
    hal_nvic_enable_irq(s_i2c_info[p_params->id].irq);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_deinit(app_i2c_id_t id)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    hal_nvic_disable_irq(s_i2c_info[id].irq);

    p_i2c_env[id]->i2c_state = APP_I2C_INVALID;
    p_i2c_env[id]->start_flag = false;

    GLOBAL_EXCEPTION_DISABLE();
    for (uint32_t i = 0; i < APP_I2C_ID_MAX; i++)
    {
        if ((p_i2c_env[i]) && ((p_i2c_env[i]->i2c_state) != APP_I2C_INVALID))
        {
            goto __deinit;
        }
    }
    pwr_unregister_sleep_cb(I2C_PWR_ID);
__deinit:
    GLOBAL_EXCEPTION_ENABLE();

    app_err_code = app_io_deinit(p_i2c_env[id]->p_pin_cfg->scl.type, p_i2c_env[id]->p_pin_cfg->scl.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(p_i2c_env[id]->p_pin_cfg->sda.type, p_i2c_env[id]->p_pin_cfg->sda.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    hal_err_code = hal_i2c_deinit(&p_i2c_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    if (p_i2c_env[id]->i2c_dma_state == APP_I2C_DMA_INVALID)
    {
        p_i2c_env[id] = NULL;
    }

    return APP_DRV_SUCCESS;
}

#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X))
uint16_t app_i2c_timing_adjust(app_i2c_id_t id, uint32_t timing_type, int32_t delta)
{
    hal_status_t  hal_err_code;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    hal_err_code = hal_i2c_timing_adjust(&p_i2c_env[id]->handle, timing_type, delta);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_timing_get(app_i2c_id_t id, uint32_t timing_type, uint32_t *p_timing_value)
{
    hal_status_t  hal_err_code;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }
    hal_err_code = hal_i2c_timing_get(&p_i2c_env[id]->handle, timing_type, p_timing_value);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}
#endif /*  ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)) */

uint16_t app_i2c_receive_sync(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = target_address;
    switch(p_i2c_env[id]->role)
    {
        case APP_I2C_ROLE_MASTER:
            err_code = hal_i2c_master_receive(&p_i2c_env[id]->handle, target_address, p_data, size, timeout);
            break;

        case APP_I2C_ROLE_SLAVE:
            err_code = hal_i2c_slave_receive(&p_i2c_env[id]->handle, p_data, size, timeout);
            break;

        default:
            break;
    }

    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_receive_async(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = target_address;

    if(p_i2c_env[id]->start_flag == false)
    {
        p_i2c_env[id]->start_flag = true;
        switch(p_i2c_env[id]->role)
        {
            case APP_I2C_ROLE_MASTER:
                err_code = hal_i2c_master_receive_it(&p_i2c_env[id]->handle, target_address, p_data, size);
                break;

            case APP_I2C_ROLE_SLAVE:
                err_code = hal_i2c_slave_receive_it(&p_i2c_env[id]->handle, p_data, size);
                break;

            default:
                break;
        }

        if (err_code != HAL_OK)
        {
            p_i2c_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_transmit_sync(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = target_address;

    switch(p_i2c_env[id]->role)
    {
        case APP_I2C_ROLE_MASTER:
            err_code = hal_i2c_master_transmit(&p_i2c_env[id]->handle, target_address, p_data, size, timeout);
            break;

        case APP_I2C_ROLE_SLAVE:
            err_code = hal_i2c_slave_transmit(&p_i2c_env[id]->handle, p_data, size, timeout);
            break;

        default:
            break;
    }

    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_transmit_async(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = target_address;

    if (p_i2c_env[id]->start_flag == false)
    {
        p_i2c_env[id]->start_flag = true;
        switch(p_i2c_env[id]->role)
        {
            case APP_I2C_ROLE_MASTER:
                err_code = hal_i2c_master_transmit_it(&p_i2c_env[id]->handle, target_address, p_data, size);
                break;

            case APP_I2C_ROLE_SLAVE:
                err_code = hal_i2c_slave_transmit_it(&p_i2c_env[id]->handle, p_data, size);
                break;

            default:
                break;
        }

        if (err_code != HAL_OK)
        {
            p_i2c_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}


uint16_t app_i2c_mem_read_sync(app_i2c_id_t id,
                               uint16_t     dev_address,
                               uint16_t     mem_address,
                               uint16_t     mem_addr_size,
                               uint8_t     *p_data,
                               uint16_t     size,
                               uint32_t     timeout)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = dev_address;

    err_code = hal_i2c_mem_read(&p_i2c_env[id]->handle, dev_address, mem_address, mem_addr_size, p_data, size, timeout);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_mem_read_async(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = dev_address;

    if(p_i2c_env[id]->start_flag == false)
    {
        p_i2c_env[id]->start_flag = true;
        err_code = hal_i2c_mem_read_it(&p_i2c_env[id]->handle, dev_address, mem_address, mem_addr_size, p_data, size);
        if (err_code != HAL_OK)
        {
            p_i2c_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_mem_write_sync(app_i2c_id_t id,
                                uint16_t     dev_address,
                                uint16_t     mem_address,
                                uint16_t     mem_addr_size,
                                uint8_t     *p_data,
                                uint16_t     size,
                                uint32_t     timeout)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = dev_address;

    err_code = hal_i2c_mem_write(&p_i2c_env[id]->handle, dev_address, mem_address, mem_addr_size, p_data, size, timeout);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_mem_write_async(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = dev_address;

    if(p_i2c_env[id]->start_flag == false)
    {
        p_i2c_env[id]->start_flag = true;
        err_code = hal_i2c_mem_write_it(&p_i2c_env[id]->handle, dev_address, mem_address, mem_addr_size, p_data, size);
        if (err_code != HAL_OK)
        {
            p_i2c_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
uint16_t app_i2c_transmit_receive_sync(app_i2c_id_t id, uint16_t dev_address, uint8_t *p_tdata, uint16_t tsize, uint8_t *p_rdata, uint16_t rsize, uint32_t timeout)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_tdata == NULL || p_rdata == NULL || tsize == 0 || rsize == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    p_i2c_env[id]->slv_dev_addr = dev_address;

    switch(p_i2c_env[id]->role)
    {
        case APP_I2C_ROLE_MASTER:
            err_code = hal_i2c_master_transmit_receive(&p_i2c_env[id]->handle, dev_address, p_tdata, tsize, p_rdata, rsize, timeout);
            break;

        case APP_I2C_ROLE_SLAVE:
            err_code = hal_i2c_slave_receive_transmit(&p_i2c_env[id]->handle, p_tdata, tsize, p_rdata, rsize, timeout);
            break;

        default:
            break;
    }

    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}
#endif

i2c_handle_t *app_i2c_get_handle(app_i2c_id_t id)
{
    if (id >= APP_I2C_ID_MAX)
    {
        return NULL;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    i2c_wake_up(id);
#endif

    return &p_i2c_env[id]->handle;
}

uint16_t app_i2c_master_abort_it(app_i2c_id_t id)
{
    hal_status_t err_code = HAL_OK;

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_i2c_env[id] == NULL) || (p_i2c_env[id]->i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    err_code = hal_i2c_master_abort_it(&(p_i2c_env[id]->handle));
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

void hal_i2c_master_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_TX_CPLT);
}

void hal_i2c_master_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_RX_DATA);
}

void hal_i2c_slave_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_TX_CPLT);
}

void hal_i2c_slave_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_RX_DATA);
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
void hal_i2c_mem_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_TX_CPLT);
}

void hal_i2c_mem_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_RX_DATA);
}
#endif

void hal_i2c_error_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_ERROR);
}

void hal_i2c_abort_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_ABORT);
}

#define I2C_HANDLER(index, val) \
SECTION_RAM_CODE void I2C##index##_IRQHandler(void)\
{\
    hal_i2c_irq_handler(&p_i2c_env[val]->handle);\
}

I2C_HANDLER(0, APP_I2C_ID_0)
I2C_HANDLER(1, APP_I2C_ID_1)
#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) | (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X))
I2C_HANDLER(2, APP_I2C_ID_2)
I2C_HANDLER(3, APP_I2C_ID_3)
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
I2C_HANDLER(4, APP_I2C_ID_4)
I2C_HANDLER(5, APP_I2C_ID_5)
#endif

#endif


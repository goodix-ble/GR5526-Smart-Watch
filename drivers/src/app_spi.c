/**
  ****************************************************************************************
  * @file    app_spi.c
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
#include "app_spi.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "gr_soc.h"
#include <string.h>

#ifdef HAL_SPI_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */
#define DEFAULT_POLLING_WAIT_TIMEOUT_MS     1000u
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
#define BLE_INT_DISABLE()                                                                            \
do {                                                                                                 \
    volatile uint32_t __ble_l_irq_rest = __get_PRIMASK();                                            \
    volatile bool __ble_int_status = NVIC_GetEnableIRQ(BLE_IRQn) || NVIC_GetEnableIRQ(BLESLP_IRQn);  \
    __set_PRIMASK(1);                                                                                \
    if (__ble_int_status)                                                                            \
    {                                                                                                \
        NVIC_DisableIRQ(BLE_IRQn);                                                                   \
        NVIC_DisableIRQ(BLESLP_IRQn);                                                                \
    }                                                                                                \
    __set_PRIMASK(__ble_l_irq_rest);

/** @brief Restore BLE_IRQn and BLESLP_IRQn.
 *  @sa BLE_INT_RESTORE
 */
#define BLE_INT_RESTORE()                                                                            \
    __ble_l_irq_rest = __get_PRIMASK();                                                              \
    __set_PRIMASK(1);                                                                                \
    if (__ble_int_status)                                                                            \
    {                                                                                                \
        NVIC_EnableIRQ(BLE_IRQn);                                                                    \
        NVIC_EnableIRQ(BLESLP_IRQn);                                                                 \
    }                                                                                                \
    __set_PRIMASK(__ble_l_irq_rest);                                                                 \
} while(0)
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
#define SPI_SMART_CS_LOW(id)                                            \
    do {                                                                \
            if((APP_SPI_ID_SLAVE != id) &&                              \
               (p_spi_env[id]->p_pin_cfg->cs.enable == APP_SPI_PIN_ENABLE)) \
            {                                                           \
                app_io_write_pin(p_spi_env[id]->p_pin_cfg->cs.type,     \
                                p_spi_env[id]->p_pin_cfg->cs.pin,       \
                                APP_IO_PIN_RESET);                      \
            }\
        } while(0)

#define SPI_SMART_CS_HIGH(id)                                           \
    do {                                                                \
            if((APP_SPI_ID_SLAVE != id) &&                              \
               (p_spi_env[id]->p_pin_cfg->cs.enable == APP_SPI_PIN_ENABLE)) \
            {                                                           \
                app_io_write_pin(p_spi_env[id]->p_pin_cfg->cs.type,     \
                                 p_spi_env[id]->p_pin_cfg->cs.pin,      \
                                 APP_IO_PIN_SET);                       \
            }                                                           \
    } while(0)
#else
#define SPI_SMART_CS_LOW(id)
#define SPI_SMART_CS_HIGH(id)
#endif

#define APP_SPI_CALLBACK(id, evt)                 \
    do                                            \
    {                                             \
        p_spi_env[id]->start_flag = false;        \
        SPI_SMART_CS_HIGH(id);                    \
        if (p_spi_env[id]->evt_handler != NULL)   \
        {                                         \
            p_spi_env[id]->evt_handler(&evt);     \
        }                                         \
    } while(0)

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool spi_prepare_for_sleep(void);
static void spi_wake_up_ind(void);
static uint16_t spi_gpio_config(app_spi_id_t id, app_spi_pin_cfg_t *p_pin_cfg);
void SPI_S_IRQHandler(void);
void SPI_M_IRQHandler(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const IRQn_Type s_spi_irq[APP_SPI_ID_MAX]      = {SPI_S_IRQn, SPI_M_IRQn};
static const uint32_t  s_spi_instance[APP_SPI_ID_MAX] = {SPIS_BASE, SPIM_BASE};

spi_env_t  *p_spi_env[APP_SPI_ID_MAX];

static const app_sleep_callbacks_t spi_sleep_cb =
{
    .app_prepare_for_sleep = spi_prepare_for_sleep,
    .app_wake_up_ind       = spi_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool spi_prepare_for_sleep(void)
{
    hal_spi_state_t state;
    uint32_t i;

    for (i = 0; i < APP_SPI_ID_MAX; i++)
    {
        if (p_spi_env[i] == NULL)
        {
            continue;
        }

        if (p_spi_env[i]->spi_state == APP_SPI_ACTIVITY)
        {
            state = hal_spi_get_state(&p_spi_env[i]->handle);
            if ((state != HAL_SPI_STATE_READY) && (state != HAL_SPI_STATE_RESET))
            {
                return false;
            }

            GLOBAL_EXCEPTION_DISABLE();
            hal_spi_suspend_reg(&p_spi_env[i]->handle);
            GLOBAL_EXCEPTION_ENABLE();
            #ifdef APP_DRIVER_WAKEUP_CALL_FUN
            p_spi_env[i]->spi_state = APP_SPI_SLEEP;
            #endif
        }
    }

    return true;
}

SECTION_RAM_CODE static void spi_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    uint32_t i;

    for (i = 0; i < APP_SPI_ID_MAX; i++)
    {
        if (p_spi_env[i] == NULL)
        {
            continue;
        }

        if (p_spi_env[i]->spi_state == APP_SPI_ACTIVITY)
        {
            GLOBAL_EXCEPTION_DISABLE();
            hal_spi_resume_reg(&p_spi_env[i]->handle);
            GLOBAL_EXCEPTION_ENABLE();

            hal_nvic_clear_pending_irq(s_spi_irq[i]);
            hal_nvic_enable_irq(s_spi_irq[i]);
        }
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
void spi_wake_up(app_spi_id_t id)
{
    if (p_spi_env[id]->spi_state == APP_SPI_SLEEP)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_spi_resume_reg(&p_spi_env[id]->handle);
        GLOBAL_EXCEPTION_ENABLE();

        if(p_spi_env[id]->use_mode.type == APP_SPI_TYPE_INTERRUPT ||
           p_spi_env[id]->use_mode.type == APP_SPI_TYPE_DMA)
        {
            hal_nvic_clear_pending_irq(s_spi_irq[id]);
            hal_nvic_enable_irq(s_spi_irq[id]);
        }
        p_spi_env[id]->spi_state = APP_SPI_ACTIVITY;
    }

    dma_wake_up(p_spi_env[id]->dma_id[0]);
    dma_wake_up(p_spi_env[id]->dma_id[1]);
}
#endif

static uint16_t spi_gpio_config(app_spi_id_t id, app_spi_pin_cfg_t *p_pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if (p_pin_cfg->cs.enable == APP_SPI_PIN_ENABLE)
    {
        if (id == APP_SPI_ID_SLAVE)
        {
            io_init.pull = p_pin_cfg->cs.pull;
            io_init.mode = APP_IO_MODE_MUX;
            io_init.pin  = p_pin_cfg->cs.pin;
            io_init.mux  = p_pin_cfg->cs.mux;
            err_code = app_io_init(p_pin_cfg->cs.type, &io_init);
            APP_DRV_ERR_CODE_CHECK(err_code);
        }
        else
        {
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
            io_init.pull = p_pin_cfg->cs.pull;
            io_init.mode = APP_IO_MODE_OUTPUT;
            io_init.pin  = p_pin_cfg->cs.pin;
            io_init.mux  = APP_IO_MUX_7;
            err_code = app_io_init(p_pin_cfg->cs.type, &io_init);
            app_io_write_pin(p_pin_cfg->cs.type, p_pin_cfg->cs.pin, APP_IO_PIN_SET);
            APP_DRV_ERR_CODE_CHECK(err_code);
#else
            if(p_spi_env[id]->is_soft_cs) {
                io_init.pull = p_pin_cfg->cs.pull;
                io_init.mode = APP_IO_MODE_OUTPUT;
                io_init.pin  = p_pin_cfg->cs.pin;
                io_init.mux  = APP_IO_MUX;
                err_code = app_io_init(p_pin_cfg->cs.type, &io_init);
                app_io_write_pin(p_pin_cfg->cs.type, p_pin_cfg->cs.pin, APP_IO_PIN_SET);
                APP_DRV_ERR_CODE_CHECK(err_code);
            } else {
                io_init.pull = p_pin_cfg->cs.pull;
                io_init.mode = p_pin_cfg->cs.mode;
                io_init.pin  = p_pin_cfg->cs.pin;
                io_init.mux  = p_pin_cfg->cs.mux;
                err_code = app_io_init(p_pin_cfg->cs.type, &io_init);
                APP_DRV_ERR_CODE_CHECK(err_code);
            }
#endif
        }
    }
    if (p_pin_cfg->clk.enable == APP_SPI_PIN_ENABLE)
    {
        io_init.pull = p_pin_cfg->clk.pull;
        io_init.mode = APP_IO_MODE_MUX;
        io_init.pin  = p_pin_cfg->clk.pin;
        io_init.mux  = p_pin_cfg->clk.mux;
        err_code = app_io_init(p_pin_cfg->clk.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }
    if (p_pin_cfg->mosi.enable == APP_SPI_PIN_ENABLE)
    {
        io_init.pull = p_pin_cfg->mosi.pull;
        io_init.pin  = p_pin_cfg->mosi.pin;
        io_init.mux  = p_pin_cfg->mosi.mux;
        err_code = app_io_init(p_pin_cfg->mosi.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }
    if (p_pin_cfg->miso.enable == APP_SPI_PIN_ENABLE)
    {
        io_init.pull = p_pin_cfg->miso.pull;
        io_init.pin  = p_pin_cfg->miso.pin;
        io_init.mux  = p_pin_cfg->miso.mux;
        err_code = app_io_init(p_pin_cfg->miso.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    return err_code;
}

static inline app_spi_id_t spi_get_id(spi_handle_t *p_spi)
{
    app_spi_id_t id = APP_SPI_ID_MAX;

    if (p_spi->p_instance == SPIS)
    {
        id = APP_SPI_ID_SLAVE;
    }
    else if (p_spi->p_instance == SPIM)
    {
        id = APP_SPI_ID_MASTER;
    }

    return id;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_spi_init(app_spi_params_t *p_params, app_spi_evt_handler_t evt_handler)
{
    app_spi_id_t    id = p_params->id;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }
    p_spi_env[id] = &(p_params->spi_env);

    p_spi_env[id]->is_soft_cs = p_params->is_soft_cs;

    err_code = spi_gpio_config(p_params->id, &p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(err_code);

    p_spi_env[id]->p_pin_cfg = &p_params->pin_cfg;
    p_spi_env[id]->evt_handler = evt_handler;

    memcpy(&p_spi_env[id]->handle.init, &p_params->init, sizeof(spi_init_t));
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    p_spi_env[id]->handle.p_instance = (ssi_regs_t *)s_spi_instance[id];
#else
    p_spi_env[id]->handle.p_instance = (spi_regs_t *)s_spi_instance[id];
#endif

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
    if(p_params->is_soft_cs) {
        p_spi_env[id]->handle.soft_cs_magic = SPI_SOFT_CS_MAGIC_NUMBER;
    } else {
        p_spi_env[id]->handle.soft_cs_magic = 0x00;
    }
#endif

    hal_spi_deinit(&p_spi_env[id]->handle);
    hal_spi_init(&p_spi_env[id]->handle);

    pwr_register_sleep_cb(&spi_sleep_cb, APP_DRIVER_SPI_WAKEUP_PRIORITY, SPI_PWR_ID);

    p_spi_env[id]->spi_state = APP_SPI_ACTIVITY;
    p_spi_env[id]->start_flag = false;

    soc_register_nvic(SPI_S_IRQn, (uint32_t)SPI_S_IRQHandler);
    soc_register_nvic(SPI_M_IRQn, (uint32_t)SPI_M_IRQHandler);
    hal_nvic_clear_pending_irq(s_spi_irq[id]);
    hal_nvic_enable_irq(s_spi_irq[id]);

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_deinit(app_spi_id_t id)
{
    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_spi_env[id]->p_pin_cfg->cs.enable == APP_SPI_PIN_ENABLE)
    {
        app_io_deinit(p_spi_env[id]->p_pin_cfg->cs.type, p_spi_env[id]->p_pin_cfg->cs.pin);
    }
    if (p_spi_env[id]->p_pin_cfg->clk.enable == APP_SPI_PIN_ENABLE)
    {
        app_io_deinit(p_spi_env[id]->p_pin_cfg->clk.type, p_spi_env[id]->p_pin_cfg->clk.pin);
    }
    if (p_spi_env[id]->p_pin_cfg->mosi.enable == APP_SPI_PIN_ENABLE)
    {
        app_io_deinit(p_spi_env[id]->p_pin_cfg->mosi.type, p_spi_env[id]->p_pin_cfg->mosi.pin);
    }
    if (p_spi_env[id]->p_pin_cfg->miso.enable == APP_SPI_PIN_ENABLE)
    {
        app_io_deinit(p_spi_env[id]->p_pin_cfg->miso.type, p_spi_env[id]->p_pin_cfg->miso.pin);
    }

    hal_nvic_disable_irq(s_spi_irq[id]);

    p_spi_env[id]->spi_state = APP_SPI_INVALID;
    p_spi_env[id]->start_flag = false;

    GLOBAL_EXCEPTION_DISABLE();
    for (uint32_t i = 0; i < APP_SPI_ID_MAX; i++)
    {
        if ((p_spi_env[i]) && ((p_spi_env[i]->spi_state) != APP_SPI_INVALID))
        {
            goto __deinit;
        }
    }
    pwr_unregister_sleep_cb(SPI_PWR_ID);
__deinit:
    GLOBAL_EXCEPTION_ENABLE();

    hal_spi_deinit(&p_spi_env[id]->handle);

    if (p_spi_env[id]->spi_dma_state == APP_SPI_DMA_INVALID)
    {
        p_spi_env[id] = NULL;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_spim_transmit_with_ia(app_spi_id_t id, uint8_t instruction, uint32_t address,  uint8_t * p_data, uint16_t data_length) {
    hal_status_t  err_code = HAL_OK;

    if (id != APP_SPI_ID_MASTER)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((p_data == NULL) || (data_length == 0))
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    if (p_spi_env[id]->start_flag == false)
    {
        p_spi_env[id]->start_flag = true;
        SPI_SMART_CS_LOW(id);
        err_code = hal_spi_transmit_with_ia(&p_spi_env[id]->handle, instruction, address, p_data, data_length, DEFAULT_POLLING_WAIT_TIMEOUT_MS);
        p_spi_env[id]->start_flag = false;
        SPI_SMART_CS_HIGH(id);
        if (err_code != HAL_OK)
        {
            p_spi_env[id]->start_flag = false;
            SPI_SMART_CS_HIGH(id);
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }
    return APP_DRV_SUCCESS;
}

uint16_t app_spim_receive_with_ia(app_spi_id_t id, uint8_t instruction, uint32_t address, uint8_t dummy_bytes, uint8_t * p_data, uint16_t data_length) {
    uint8_t ia_data[8];
    uint8_t sent_len = 0;
    hal_status_t  err_code = HAL_OK;

    if (id != APP_SPI_ID_MASTER)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if ((p_data == NULL) ||(data_length == 0))
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    memset(&ia_data[0], 0, 8);

    ia_data[0] = instruction;
    ia_data[1] = (address >> 16) & 0xff;
    ia_data[2] = (address >>  8) & 0xff;
    ia_data[3] = (address >>  0) & 0xff;

    if(dummy_bytes > 4) {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    sent_len = 4 + dummy_bytes;

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    if (p_spi_env[id]->start_flag == false)
    {
        p_spi_env[id]->start_flag = true;
        SPI_SMART_CS_LOW(id);
        err_code = hal_spi_read_eeprom(&p_spi_env[id]->handle, ia_data, p_data, sent_len, data_length, DEFAULT_POLLING_WAIT_TIMEOUT_MS);
        p_spi_env[id]->start_flag = false;
        SPI_SMART_CS_HIGH(id);
        if (err_code != HAL_OK)
        {
            p_spi_env[id]->start_flag = false;
            SPI_SMART_CS_HIGH(id);
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_receive_async(app_spi_id_t id, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    if (p_spi_env[id]->start_flag == false)
    {
        p_spi_env[id]->start_flag = true;
        SPI_SMART_CS_LOW(id);
        err_code = hal_spi_receive_it(&p_spi_env[id]->handle, p_data, size);
        if (err_code != HAL_OK)
        {
            p_spi_env[id]->start_flag = false;
            SPI_SMART_CS_HIGH(id);
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_receive_sync(app_spi_id_t id, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    SPI_SMART_CS_LOW(id);
    err_code = hal_spi_receive(&p_spi_env[id]->handle, p_data, size, timeout);
    SPI_SMART_CS_HIGH(id);

    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
uint16_t app_spi_receive_high_speed_sync(app_spi_id_t id, uint8_t *p_data, uint16_t size)
{
    app_drv_err_t app_err_code = APP_DRV_SUCCESS;
    hal_status_t  hal_err_code = HAL_ERROR;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    BLE_INT_DISABLE();
    p_spi_env[id]->rx_done = 0;

    if (p_spi_env[id]->start_flag == false)
    {
        p_spi_env[id]->start_flag = true;
        SPI_SMART_CS_LOW(id);
        hal_err_code = hal_spi_receive_it(&p_spi_env[id]->handle, p_data, size);
        if (hal_err_code != HAL_OK)
        {
            p_spi_env[id]->start_flag = false;
            SPI_SMART_CS_HIGH(id);
            app_err_code = (uint16_t)hal_err_code;
            goto exit;
        }
    }
    else
    {
        app_err_code = APP_DRV_ERR_BUSY;
        goto exit;
    }

    while(p_spi_env[id]->rx_done == 0);

exit:
    BLE_INT_RESTORE();

    return app_err_code;
}

uint16_t app_spi_transmit_high_speed_sync(app_spi_id_t id, uint8_t *p_data, uint16_t size)
{
    app_drv_err_t app_err_code = APP_DRV_SUCCESS;
    hal_status_t  hal_err_code = HAL_ERROR;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    BLE_INT_DISABLE();
    p_spi_env[id]->tx_done = 0;

    if (p_spi_env[id]->start_flag == false)
    {
        p_spi_env[id]->start_flag = true;
        SPI_SMART_CS_LOW(id);
        hal_err_code = hal_spi_transmit_it(&p_spi_env[id]->handle, p_data, size);
        if (hal_err_code != HAL_OK)
        {
            p_spi_env[id]->start_flag = false;
            SPI_SMART_CS_HIGH(id);
            app_err_code = (uint16_t)hal_err_code;

            goto exit;
        }
    }
    else
    {
        app_err_code = APP_DRV_ERR_BUSY;
        goto exit;
    }

    while(p_spi_env[id]->tx_done == 0);

exit:
    BLE_INT_RESTORE();

    return app_err_code;
}
#endif

uint16_t app_spi_transmit_async(app_spi_id_t id, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    if (p_spi_env[id]->start_flag == false)
    {
        p_spi_env[id]->start_flag = true;
        SPI_SMART_CS_LOW(id);
        err_code = hal_spi_transmit_it(&p_spi_env[id]->handle, p_data, size);
        if (err_code != HAL_OK)
        {
            p_spi_env[id]->start_flag = false;
            SPI_SMART_CS_HIGH(id);
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_transmit_sync(app_spi_id_t id, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    SPI_SMART_CS_LOW(id);
    err_code = hal_spi_transmit(&p_spi_env[id]->handle, p_data, size, timeout);
    SPI_SMART_CS_HIGH(id);

    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_transmit_receive_sync(app_spi_id_t id, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_tx_data == NULL || p_rx_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    SPI_SMART_CS_LOW(id);
    err_code = hal_spi_transmit_receive(&p_spi_env[id]->handle, p_tx_data, p_rx_data, size, timeout);
    SPI_SMART_CS_HIGH(id);

    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_transmit_receive_async(app_spi_id_t id, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_tx_data == NULL || p_rx_data == NULL || size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    if (p_spi_env[id]->start_flag == false)
    {
        p_spi_env[id]->start_flag = true;
        SPI_SMART_CS_LOW(id);
        err_code = hal_spi_transmit_receive_it(&p_spi_env[id]->handle, p_tx_data, p_rx_data, size);
        if (err_code != HAL_OK)
        {
            p_spi_env[id]->start_flag = false;
            SPI_SMART_CS_HIGH(id);
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_read_eeprom_async(app_spi_id_t id, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t tx_size, uint32_t rx_size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_tx_data == NULL || p_rx_data == NULL || tx_size == 0 || rx_size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    if (p_spi_env[id]->start_flag == false)
    {
        p_spi_env[id]->start_flag = true;
        SPI_SMART_CS_LOW(id);
        err_code = hal_spi_read_eeprom_it(&p_spi_env[id]->handle, p_tx_data, p_rx_data, tx_size, rx_size);
        if (err_code != HAL_OK)
        {
            p_spi_env[id]->start_flag = false;
            SPI_SMART_CS_HIGH(id);
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_read_eeprom_sync(app_spi_id_t id, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t tx_size, uint32_t rx_size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_tx_data == NULL || p_rx_data == NULL || tx_size == 0 || rx_size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    SPI_SMART_CS_LOW(id);
    err_code = hal_spi_read_eeprom(&p_spi_env[id]->handle, p_tx_data, p_rx_data, tx_size, rx_size, timeout);
    SPI_SMART_CS_HIGH(id);

    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
uint16_t app_spi_read_memory_async(app_spi_id_t id, uint8_t *p_cmd_data, uint8_t *p_rx_data, uint32_t cmd_size, uint32_t rx_size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd_data == NULL || p_rx_data == NULL || cmd_size == 0 || rx_size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    if (p_spi_env[id]->start_flag == false)
    {
        p_spi_env[id]->start_flag = true;

        SPI_SMART_CS_LOW(id);
        err_code = hal_spi_transmit(&p_spi_env[id]->handle, p_cmd_data, cmd_size, 1000);
        if (err_code != HAL_OK)
        {
            SPI_SMART_CS_HIGH(id);
            p_spi_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
        err_code = hal_spi_receive_it(&p_spi_env[id]->handle, p_rx_data, rx_size);
        if (err_code != HAL_OK)
        {
            p_spi_env[id]->start_flag = false;
            SPI_SMART_CS_HIGH(id);
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_write_memory_async(app_spi_id_t id, uint8_t *p_cmd_data, uint8_t *p_tx_data, uint32_t cmd_size, uint32_t tx_size)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd_data == NULL || p_tx_data == NULL || cmd_size == 0 || tx_size == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    if (p_spi_env[id]->start_flag == false)
    {
        p_spi_env[id]->start_flag = true;

        SPI_SMART_CS_LOW(id);
        err_code = hal_spi_transmit(&p_spi_env[id]->handle, p_cmd_data, cmd_size, 1000);
        if (err_code != HAL_OK)
        {
            SPI_SMART_CS_HIGH(id);
            p_spi_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
        err_code = hal_spi_transmit_it(&p_spi_env[id]->handle, p_tx_data, tx_size);
        if (err_code != HAL_OK)
        {
            p_spi_env[id]->start_flag = false;
            SPI_SMART_CS_HIGH(id);
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

spi_handle_t *app_spi_get_handle(app_spi_id_t id)
{
    if (id >= APP_SPI_ID_MAX)
    {
        return NULL;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    spi_wake_up(id);
#endif

    return &p_spi_env[id]->handle;
}

uint16_t app_spi_abort(app_spi_id_t id)
{
    uint16_t err_code = HAL_ERROR;

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    err_code = hal_spi_abort_it(&p_spi_env[id]->handle);
    if (err_code != HAL_OK)
        return err_code;

    return APP_DRV_SUCCESS;
}

void hal_spi_soft_cs_assert(spi_handle_t *p_spi, uint32_t state)
{
    app_spi_id_t id = spi_get_id(p_spi);
    app_io_write_pin(p_spi_env[id]->p_pin_cfg->cs.type, p_spi_env[id]->p_pin_cfg->cs.pin, APP_IO_PIN_RESET);
}

void hal_spi_soft_cs_deassert(spi_handle_t *p_spi, uint32_t state)
{
    app_spi_id_t id = spi_get_id(p_spi);
    app_io_write_pin(p_spi_env[id]->p_pin_cfg->cs.type, p_spi_env[id]->p_pin_cfg->cs.pin, APP_IO_PIN_SET);
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
__weak void _free_dma_llp_resource(void) {
    // Override this in app_spi_dma.c module when using dma llp to write screen
}
#endif

void hal_spi_tx_cplt_callback(spi_handle_t *p_spi)
{
    app_spi_evt_t spi_evt;
    app_spi_id_t id = spi_get_id(p_spi);

    spi_evt.type = APP_SPI_EVT_TX_CPLT;
    uint32_t tx_xfer_size_cb = p_spi->tx_xfer_size;
    uint32_t tx_xfer_count_cb = p_spi->tx_xfer_count;
    spi_evt.data.size = tx_xfer_size_cb - tx_xfer_count_cb;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
    _free_dma_llp_resource();
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    p_spi_env[id]->tx_done = 1;
#endif
    APP_SPI_CALLBACK(id, spi_evt);
}

void hal_spi_rx_cplt_callback(spi_handle_t *p_spi)
{
    app_spi_evt_t spi_evt;
    app_spi_id_t id = spi_get_id(p_spi);

    spi_evt.type = APP_SPI_EVT_RX_CPLT;
    uint32_t rx_xfer_size_cb = p_spi->rx_xfer_size;
    uint32_t rx_xfer_count_cb = p_spi->rx_xfer_count;
    spi_evt.data.size = rx_xfer_size_cb - rx_xfer_count_cb;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    p_spi_env[id]->rx_done = 1;
#endif
    APP_SPI_CALLBACK(id, spi_evt);
}

void hal_spi_tx_rx_cplt_callback(spi_handle_t *p_spi)
{
    app_spi_evt_t spi_evt;
    app_spi_id_t id = spi_get_id(p_spi);

    spi_evt.type = APP_SPI_EVT_TX_RX_CPLT;
    uint32_t rx_xfer_size_cb = p_spi->rx_xfer_size;
    uint32_t rx_xfer_count_cb = p_spi->rx_xfer_count;
    spi_evt.data.size = rx_xfer_size_cb - rx_xfer_count_cb;
    APP_SPI_CALLBACK(id, spi_evt);
}

void hal_spi_error_callback(spi_handle_t *p_spi)
{
    app_spi_evt_t spi_evt;
    app_spi_id_t id = spi_get_id(p_spi);

    spi_evt.type = APP_SPI_EVT_ERROR;
    spi_evt.data.error_code = p_spi->error_code;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    p_spi_env[id]->rx_done = 1;
    p_spi_env[id]->tx_done = 1;
#endif
    APP_SPI_CALLBACK(id, spi_evt);
}

void hal_spi_abort_cplt_callback(spi_handle_t *p_spi)
{
    app_spi_evt_t spi_evt;
    app_spi_id_t id = spi_get_id(p_spi);
    spi_evt.type = APP_SPI_EVT_ABORT;
    APP_SPI_CALLBACK(id, spi_evt);
}

SECTION_RAM_CODE void SPI_S_IRQHandler(void)
{
    hal_spi_irq_handler(&p_spi_env[APP_SPI_ID_SLAVE]->handle);
}

SECTION_RAM_CODE void SPI_M_IRQHandler(void)
{
    hal_spi_irq_handler(&p_spi_env[APP_SPI_ID_MASTER]->handle);
}

#endif  /* HAL_SPI_MODULE_ENABLED */


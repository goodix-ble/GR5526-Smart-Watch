/**
  ****************************************************************************************
  * @file    app_spi_dma.c
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
#include "app_spi_dma.h"
#include "app_dma.h"
#include "app_drv_config.h"
#include "app_pwr_mgmt.h"
#include <string.h>
#include "gr_soc.h"

#ifdef HAL_SPI_MODULE_ENABLED
/*
 * DEFINES
 *****************************************************************************************
 */
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

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
extern bool spi_prepare_for_sleep(void);
extern void spi_sleep_canceled(void);
extern void spi_wake_up_ind(void);
extern void spi_wake_up(app_spi_id_t id);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern spi_env_t  *p_spi_env[APP_SPI_ID_MAX];

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint16_t app_spi_config_dma_tx(app_spi_params_t *p_params)
{
    app_dma_params_t tx_dma_params = { 0 };

    tx_dma_params.p_instance                 = p_params->dma_cfg.tx_dma_instance;
    tx_dma_params.channel_number             = p_params->dma_cfg.tx_dma_channel;
    tx_dma_params.init.src_request           = DMA0_REQUEST_MEM;
    tx_dma_params.init.dst_request           = (p_params->id == APP_SPI_ID_SLAVE) ? DMA0_REQUEST_SPIS_TX : DMA0_REQUEST_SPIM_TX;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    if (tx_dma_params.p_instance == DMA1)
    {
        if (p_params->id == APP_SPI_ID_SLAVE)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
        tx_dma_params.init.src_request = DMA1_REQUEST_MEM;
        tx_dma_params.init.dst_request = DMA1_REQUEST_SPIM_TX;
    }
#endif
    tx_dma_params.init.direction             = DMA_MEMORY_TO_PERIPH;
    tx_dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    tx_dma_params.init.dst_increment         = DMA_DST_NO_CHANGE;
    if (p_params->init.data_size <= SPI_DATASIZE_8BIT)
    {
        tx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
        tx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
    }
    else if (p_params->init.data_size <= SPI_DATASIZE_16BIT)
    {
        tx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_HALFWORD;
        tx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_HALFWORD;
    }
    else
    {
        tx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_WORD;
        tx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_WORD;
    }
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    tx_dma_params.init.mode                  = DMA_NORMAL;
#endif
    tx_dma_params.init.priority              = DMA_PRIORITY_LOW;

    p_spi_env[p_params->id]->dma_id[0] = app_dma_init(&tx_dma_params, NULL);

    if (p_spi_env[p_params->id]->dma_id[0] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    p_spi_env[p_params->id]->handle.p_dmatx = app_dma_get_handle(p_spi_env[p_params->id]->dma_id[0]);
    p_spi_env[p_params->id]->handle.p_dmatx->p_parent = (void*)&p_spi_env[p_params->id]->handle;

    return APP_DRV_SUCCESS;
}

static uint16_t app_spi_config_dma_rx(app_spi_params_t *p_params)
{
    app_dma_params_t rx_dma_params = { 0 };

    rx_dma_params.p_instance                 = p_params->dma_cfg.rx_dma_instance;
    rx_dma_params.channel_number             = p_params->dma_cfg.rx_dma_channel;
    rx_dma_params.init.dst_request           = DMA0_REQUEST_MEM;
    rx_dma_params.init.src_request           = (p_params->id == APP_SPI_ID_SLAVE) ? DMA0_REQUEST_SPIS_RX : DMA0_REQUEST_SPIM_RX;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    if (rx_dma_params.p_instance == DMA1)
    {
        if (p_params->id == APP_SPI_ID_SLAVE)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
        rx_dma_params.init.dst_request = DMA1_REQUEST_MEM;
        rx_dma_params.init.src_request = DMA1_REQUEST_SPIM_RX;
    }
#endif
    rx_dma_params.init.direction             = DMA_PERIPH_TO_MEMORY;
    rx_dma_params.init.src_increment         = DMA_SRC_NO_CHANGE;
    rx_dma_params.init.dst_increment         = DMA_DST_INCREMENT;
    if (p_params->init.data_size <= SPI_DATASIZE_8BIT)
    {
        rx_dma_params.init.src_data_alignment = DMA_SDATAALIGN_BYTE;
        rx_dma_params.init.dst_data_alignment = DMA_DDATAALIGN_BYTE;
    }
    else if (p_params->init.data_size <= SPI_DATASIZE_16BIT)
    {
        rx_dma_params.init.src_data_alignment = DMA_SDATAALIGN_HALFWORD;
        rx_dma_params.init.dst_data_alignment = DMA_DDATAALIGN_HALFWORD;
    }
    else
    {
        rx_dma_params.init.src_data_alignment = DMA_SDATAALIGN_WORD;
        rx_dma_params.init.dst_data_alignment = DMA_DDATAALIGN_WORD;
    }
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    rx_dma_params.init.mode                   = DMA_NORMAL;
#endif
    rx_dma_params.init.priority               = DMA_PRIORITY_LOW;

    p_spi_env[p_params->id]->dma_id[1] = app_dma_init(&rx_dma_params, NULL);

    if (p_spi_env[p_params->id]->dma_id[1] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    p_spi_env[p_params->id]->handle.p_dmarx = app_dma_get_handle(p_spi_env[p_params->id]->dma_id[1]);
    p_spi_env[p_params->id]->handle.p_dmarx->p_parent = (void*)&p_spi_env[p_params->id]->handle;

    return APP_DRV_SUCCESS;
}

static uint16_t app_spi_config_dma(app_spi_params_t *p_params)
{
    app_drv_err_t app_err_code = APP_DRV_SUCCESS;

    p_spi_env[p_params->id]->dma_id[0] = -1;
    p_spi_env[p_params->id]->dma_id[1] = -1;

    if (p_params->dma_cfg.tx_dma_instance == NULL &&
        p_params->dma_cfg.rx_dma_instance == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (p_params->dma_cfg.tx_dma_instance != NULL)
    {
        app_err_code = app_spi_config_dma_tx(p_params);
        if (app_err_code != APP_DRV_SUCCESS)
        {
            return app_err_code;
        }
    }
    if (p_params->dma_cfg.rx_dma_instance != NULL)
    {
        app_err_code = app_spi_config_dma_rx(p_params);
        if (app_err_code != APP_DRV_SUCCESS)
        {
            return app_err_code;
        }
    }

    return APP_DRV_SUCCESS;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_spi_dma_init(app_spi_params_t *p_params)
{
    app_spi_id_t id = p_params->id;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_spi_env[id]->spi_dma_state != APP_SPI_DMA_INVALID)
    {
        return APP_DRV_ERR_INVALID_INIT;
    }

    GLOBAL_EXCEPTION_DISABLE();
    err_code = app_spi_config_dma(p_params);
    if (err_code != APP_DRV_SUCCESS)
    {
        goto __exit;
    }
    p_spi_env[id]->spi_dma_state = APP_SPI_DMA_ACTIVITY;
__exit:
    GLOBAL_EXCEPTION_ENABLE();

    return err_code;
}

uint16_t app_spi_dma_deinit(app_spi_id_t id)
{
    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_spi_env[id] == NULL) || (p_spi_env[id]->spi_dma_state != APP_SPI_DMA_ACTIVITY))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    app_dma_deinit(p_spi_env[id]->dma_id[0]);
    app_dma_deinit(p_spi_env[id]->dma_id[1]);

    p_spi_env[id]->spi_dma_state = APP_SPI_DMA_INVALID;

    return APP_DRV_SUCCESS;
}

uint16_t app_spim_dma_transmit_with_ia(app_spi_id_t id, uint8_t instruction, uint32_t address,  uint8_t * p_data, uint16_t data_length) {
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
        err_code = hal_spi_transmit_dma_with_ia(&p_spi_env[id]->handle, instruction, address, p_data, data_length);
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

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
uint16_t app_spim_dma_transmit_with_ia_32addr(app_spi_id_t id, uint8_t instruction, uint32_t address,  uint8_t * p_data, uint16_t data_length) {
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
        err_code = hal_spi_transmit_dma_with_ia_32addr(&p_spi_env[id]->handle, instruction, address, p_data, data_length);
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

uint16_t app_spim_dma_receive_with_ia(app_spi_id_t id, uint8_t instruction, uint32_t address, uint8_t dummy_bytes, uint8_t * p_data, uint16_t data_length) {
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

    if ((p_data == NULL) || (data_length == 0))
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
        err_code = hal_spi_read_eeprom_dma(&p_spi_env[id]->handle, ia_data, p_data, sent_len, data_length);
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

uint16_t app_spi_dma_receive_async(app_spi_id_t id, uint8_t *p_data, uint16_t size)
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
        err_code = hal_spi_receive_dma(&p_spi_env[id]->handle, p_data, size);
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

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
uint16_t app_spi_dma_receive_high_speed_sync(app_spi_id_t id, uint8_t *p_data, uint16_t size)
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
        hal_err_code = hal_spi_receive_dma(&p_spi_env[id]->handle, p_data, size);
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

uint16_t app_spi_dma_transmit_high_speed_sync(app_spi_id_t id, uint8_t *p_data, uint16_t size)
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
        hal_err_code = hal_spi_transmit_dma(&p_spi_env[id]->handle, p_data, size);
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

uint16_t app_spi_dma_transmit_async(app_spi_id_t id, uint8_t *p_data, uint16_t size)
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
        err_code = hal_spi_transmit_dma(&p_spi_env[id]->handle, p_data, size);
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

uint16_t app_spi_dma_transmit_receive_async(app_spi_id_t id, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t size)
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

    if (p_tx_data == NULL || p_rx_data == NULL ||size == 0)
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
        err_code = hal_spi_transmit_receive_dma(&p_spi_env[id]->handle, p_tx_data, p_rx_data, size);
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

uint16_t app_spi_dma_read_eeprom_async(app_spi_id_t id, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t tx_size, uint32_t rx_size)
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
        err_code = hal_spi_read_eeprom_dma(&p_spi_env[id]->handle, p_tx_data, p_rx_data, tx_size, rx_size);
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

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
uint16_t app_spi_dma_read_memory_async(app_spi_id_t id, uint8_t *p_cmd_data, uint8_t *p_rx_data, uint32_t cmd_size, uint32_t rx_size)
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
        err_code = hal_spi_receive_dma(&p_spi_env[id]->handle, p_rx_data, rx_size);
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

uint16_t app_spi_dma_write_memory_async(app_spi_id_t id, uint8_t *p_cmd_data, uint8_t *p_tx_data, uint32_t cmd_size, uint32_t tx_size)
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

        err_code = hal_spi_transmit_dma(&p_spi_env[id]->handle, p_tx_data, tx_size);
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



#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)

__weak void * _v_malloc(uint32_t size) {
    // TODO: override this function
    return NULL;
}

__weak void _v_free(void * ptr) {
    // TODO: override this function
    return;
}

static dma_block_config_t * p_llp = NULL;

void _free_dma_llp_resource(void) {
    if (p_llp != NULL)
    {
        dma_block_config_t * p = p_llp;
        dma_block_config_t * q = NULL;
        while (p)
        {
            q = p->p_lli;
            _v_free(p);
            p = q;
        }
    }
    p_llp = NULL;
}


/****************************************************************
 * IF Flush Area <= Frame Buffer :
 *
 *      ---------------------[stride]------------------
 *      |    [p_buff]                                 |
 *      |    ++++++++++++++[width]++++++++++++        |
 *      |    +                               +        |
 *      |    +                               +        |
 *      |    +          Flush Area        [height]    |
 *      |    +                               +        |
 *      |    +                               +        |
 *      |    +++++++++++++++++++++++++++++++++        |
 *      |                                             |
 *      -----------------------------------------------
 *                      Frame Buffer
 ****************************************************************/
uint16_t app_spi_send_display_frame(app_spi_id_t id, app_spi_screen_config_t * p_scrn_info, void * p_buff) {

    static uint8_t head[4];
    uint32_t shift_bit   = 0;
    uint32_t xfer_width  = 0;
    uint32_t i           = 0;
    uint32_t block_count = 0;
    uint32_t block_left  = 0;
    uint32_t block_beat  = 0;
    hal_status_t status  = HAL_ERROR;

    dma_block_config_t * p_llp_block      = NULL;
    dma_block_config_t * p_llp_block_prev = NULL;

    dma_sg_llp_config_t sg_llp_config = {
        .gather_config = {
            .src_gather_en = DMA_SRC_GATHER_DISABLE,
        },
        .scatter_config = {
            .dst_scatter_en = DMA_DST_SCATTER_DISABLE,
        },
        .llp_config = {
            .llp_src_en = DMA_LLP_SRC_ENABLE,
            .llp_dst_en = DMA_LLP_DST_DISABLE,
            .head_lli = NULL,
        },
    };

    head[0] = p_scrn_info->instruction;
    head[1] = (p_scrn_info->leading_address >> 16) & 0xff;
    head[2] = (p_scrn_info->leading_address >>  8) & 0xff;
    head[3] = (p_scrn_info->leading_address >>  0) & 0xff;

    if(p_scrn_info->data_xfer_width == SPI_DATASIZE_8BIT) {
        xfer_width = DMA_SDATAALIGN_BYTE | DMA_DDATAALIGN_BYTE ;
        shift_bit  = 0;
    } else if(p_scrn_info->data_xfer_width == SPI_DATASIZE_16BIT) {
        xfer_width = DMA_SDATAALIGN_HALFWORD | DMA_DDATAALIGN_HALFWORD ;
        shift_bit  = 1;
    }

    uint32_t stride_size = 0;
    const uint32_t total_beats = (p_scrn_info->buff_pixel_width * p_scrn_info->buff_pixel_height * p_scrn_info->buff_pixel_depth) >> shift_bit;

    if(p_scrn_info->buff_pixel_stride == p_scrn_info->buff_pixel_width) {
        block_count = total_beats / 4092;
        block_left  = total_beats % 4092;
        block_beat  = 4092;
        stride_size = block_beat << shift_bit;
    } else {
        block_count = p_scrn_info->buff_pixel_height;
        block_left  = 0;
        block_beat  = (p_scrn_info->buff_pixel_width * p_scrn_info->buff_pixel_depth) >> shift_bit;
        stride_size = p_scrn_info->buff_pixel_stride * p_scrn_info->buff_pixel_depth;
    }

    // Link the head
    {
        p_llp_block = _v_malloc(sizeof(dma_block_config_t));

        p_llp_block->src_address = (uint32_t)&head[0];
        p_llp_block->dst_address = 0;
        p_llp_block->src_status  = 0x00;
        p_llp_block->dst_status  = 0x00;

        /* memset in word mode */
        p_llp_block->CTL_L       = DMA_CTLL_INI_EN
                                    | DMA_SRC_INCREMENT
                                    | DMA_DST_NO_CHANGE
                                    | DMA_SRC_GATHER_DISABLE
                                    | DMA_DST_SCATTER_DISABLE
                                    | DMA_LLP_SRC_ENABLE
                                    | DMA_LLP_DST_DISABLE
                                    | DMA_MEMORY_TO_PERIPH
                                    | DMA_SDATAALIGN_BYTE | DMA_DDATAALIGN_BYTE;

        p_llp_block->CTL_H       = 4;
        p_llp_block->p_lli       = NULL;

        if(p_llp_block_prev == NULL) {
            sg_llp_config.llp_config.head_lli = p_llp_block;
        } else {
            p_llp_block_prev->p_lli = p_llp_block;
        }
        p_llp_block_prev = p_llp_block;
    }

    uint32_t src_addr = (uint32_t) p_buff;

    for(i = 0; i < block_count; i++) {
        p_llp_block = _v_malloc(sizeof(dma_block_config_t));

        p_llp_block->src_address = src_addr;
        p_llp_block->dst_address = 0;
        p_llp_block->src_status  = 0x00;
        p_llp_block->dst_status  = 0x00;

        /* memset in word mode */
        p_llp_block->CTL_L       = DMA_CTLL_INI_EN
                                    | DMA_SRC_INCREMENT
                                    | DMA_DST_NO_CHANGE
                                    | DMA_SRC_GATHER_DISABLE
                                    | DMA_DST_SCATTER_DISABLE
                                    | DMA_LLP_SRC_ENABLE
                                    | DMA_LLP_DST_DISABLE
                                    | DMA_MEMORY_TO_PERIPH
                                    | LL_DMA_SRC_BURST_LENGTH_4 | LL_DMA_DST_BURST_LENGTH_4
                                    | xfer_width;

        p_llp_block->CTL_H       = block_beat;
        p_llp_block->p_lli       = NULL;

        if(p_llp_block_prev == NULL) {
            sg_llp_config.llp_config.head_lli = p_llp_block;
        } else {
            p_llp_block_prev->p_lli = p_llp_block;
        }
        p_llp_block_prev = p_llp_block;

        src_addr += stride_size;
    }

    if(block_left > 0) {
        p_llp_block = _v_malloc(sizeof(dma_block_config_t));

        p_llp_block->src_address = src_addr;
        p_llp_block->dst_address = 0;
        p_llp_block->src_status  = 0x00;
        p_llp_block->dst_status  = 0x00;

        /* memset in word mode */
        p_llp_block->CTL_L       = DMA_CTLL_INI_EN
                                    | DMA_SRC_INCREMENT
                                    | DMA_DST_NO_CHANGE
                                    | DMA_SRC_GATHER_DISABLE
                                    | DMA_DST_SCATTER_DISABLE
                                    | DMA_LLP_SRC_ENABLE
                                    | DMA_LLP_DST_DISABLE
                                    | DMA_MEMORY_TO_PERIPH
                                    | LL_DMA_SRC_BURST_LENGTH_4 | LL_DMA_DST_BURST_LENGTH_4
                                    | xfer_width;

        p_llp_block->CTL_H       = block_left;
        p_llp_block->p_lli       = NULL;

        if(p_llp_block_prev == NULL) {
            sg_llp_config.llp_config.head_lli = p_llp_block;
        } else {
            p_llp_block_prev->p_lli = p_llp_block;
        }
        p_llp_block_prev = p_llp_block;
    }

    p_llp = sg_llp_config.llp_config.head_lli;

    uint32_t data_length = p_scrn_info->buff_pixel_width * p_scrn_info->buff_pixel_height * p_scrn_info->buff_pixel_depth;

    if (p_spi_env[id]->start_flag == false)
    {
        p_spi_env[id]->start_flag = true;
        SPI_SMART_CS_LOW(id);
        status = hal_spi_transmit_dma_llp(&p_spi_env[id]->handle, &sg_llp_config.llp_config, data_length + 4);
        if (status != HAL_OK)
        {
            _free_dma_llp_resource();
            p_spi_env[id]->start_flag = false;
            SPI_SMART_CS_HIGH(id);
            return (uint16_t)status;
        }
    }
    else
    {
        _free_dma_llp_resource();
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

#endif

#endif  /* HAL_SPI_MODULE_ENABLED */


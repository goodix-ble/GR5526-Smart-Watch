/**
  ****************************************************************************************
  * @file    app_qspi_dma.c
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
#include "app_assert.h"
#include "app_qspi.h"
#include "app_qspi_dma.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include <string.h>
#include "platform_sdk.h"
#include "app_drv.h"
#include "gr_soc.h"

#ifdef HAL_QSPI_MODULE_ENABLED
/*
 * DEFINES
 *****************************************************************************************
 */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
#define QSPI_SMART_CS_LOW(id)                                           \
    do {                                                                \
            if(p_qspi_env[id]->p_pin_cfg->cs.enable == APP_QSPI_PIN_ENABLE) \
            {                                                           \
                app_io_write_pin(p_qspi_env[id]->p_pin_cfg->cs.type,    \
                                p_qspi_env[id]->p_pin_cfg->cs.pin,      \
                                APP_IO_PIN_RESET);                      \
            }                                                           \
        } while(0)

#define QSPI_SMART_CS_HIGH(id)                                          \
    do {                                                                \
            if(p_qspi_env[id]->p_pin_cfg->cs.enable == APP_QSPI_PIN_ENABLE) \
            {                                                           \
                app_io_write_pin(p_qspi_env[id]->p_pin_cfg->cs.type,    \
                                 p_qspi_env[id]->p_pin_cfg->cs.pin,     \
                                 APP_IO_PIN_SET);                       \
            }                                                           \
    } while(0)
#else
#define QSPI_SMART_CS_LOW(id)
#define QSPI_SMART_CS_HIGH(id)
#endif

#define APP_QSPI_EXCEPT_DEBUG_EN            1u
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
/********************************************************************
 * QUAD_WRITE_32b_PATCH : just exist in QUAD/DATASIZE_32BITS/DMA scene
 *   if enable, MUST Control the CS By Software.
 */
#define QSPI_QUAD_WRITE_32b_PATCH_EN        0u

/********************************************************************
 * DATA Endian Mode Optional Value :
 *   0 : data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
 *   1 : data[1] | (data[0] << 8) | (data[3] << 16) | (data[2] << 24)
 *   2 : data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24)
 *   3 : data[2] | (data[3] << 8) | (data[0] << 16) | (data[1] << 24)
 */
#define QSPI_QUAD_WRITE_DATA_ENDIAN_MODE    0u
#endif
/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
extern void qspi_wake_up(app_qspi_id_t id);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
volatile bool is_dma_access = false;
static bool         app_qspi_dma_match_check(app_qspi_params_t * p_params);
static bool         app_qspi_switch_dma_mode(app_qspi_id_t id, bool is_m2m_mode);
#endif
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern qspi_env_t *p_qspi_env[APP_QSPI_ID_MAX];

static uint16_t app_qspi_config_dma(app_qspi_params_t *p_params)
{
    app_dma_params_t dma_params = { 0 };

    dma_params.p_instance                 = p_params->dma_cfg.dma_instance;
    dma_params.channel_number             = p_params->dma_cfg.dma_channel;
    dma_params.init.direction             = DMA_MEMORY_TO_PERIPH;
    dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    dma_params.init.dst_increment         = DMA_DST_NO_CHANGE;
    dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
    dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
    dma_params.init.mode                  = DMA_NORMAL;
    dma_params.init.priority              = DMA_PRIORITY_LOW;

    p_qspi_env[p_params->id]->dma_id = app_dma_init(&dma_params, NULL);
    if (p_qspi_env[p_params->id]->dma_id < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    p_qspi_env[p_params->id]->handle.p_dma = app_dma_get_handle(p_qspi_env[p_params->id]->dma_id);
    p_qspi_env[p_params->id]->handle.p_dma->p_parent = (void*)&p_qspi_env[p_params->id]->handle;

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
static void app_qspi_config_dma_qwrite_32b_patch(app_qspi_id_t id, bool enable_patch, uint32_t endian_mode) {
    extern void hal_qspi_config_dma_qwrite_32b_patch(qspi_handle_t *p_qspi, bool enable_patch, uint32_t endian_mode) ;
    hal_qspi_config_dma_qwrite_32b_patch(&p_qspi_env[id]->handle, enable_patch, endian_mode);
}
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_qspi_dma_init(app_qspi_params_t *p_params)
{
    app_qspi_id_t id = p_params->id;
    app_drv_err_t app_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    GLOBAL_EXCEPTION_DISABLE();
    p_qspi_env[p_params->id]->dma_id = -1;
    app_err_code = app_qspi_config_dma(p_params);
    GLOBAL_EXCEPTION_ENABLE();
    APP_DRV_ERR_CODE_CHECK(app_err_code);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    app_err_code = app_qspi_dma_match_check(p_params) ? APP_DRV_SUCCESS : APP_DRV_ERR_INVALID_PARAM;
    APP_DRV_ERR_CODE_CHECK(app_err_code);
    GLOBAL_EXCEPTION_DISABLE();
    p_qspi_env[p_params->id]->dma_id = -1;
    app_err_code = app_qspi_config_dma(p_params);
    GLOBAL_EXCEPTION_ENABLE();
    APP_DRV_ERR_CODE_CHECK(app_err_code);
    p_qspi_env[id]->is_used_dma = true;
    p_qspi_env[id]->is_dma_mode_m2m = false;
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    p_qspi_env[id]->dma_cfg.wait_timeout_ms = p_params->dma_cfg.wait_timeout_ms;
    p_qspi_env[id]->dma_cfg.dma_instance = p_params->dma_cfg.dma_instance;
    p_qspi_env[id]->dma_cfg.dma_channel  = p_params->dma_cfg.dma_channel;
    // p_qspi_env[id]->qspi_dma_state = APP_QSPI_DMA_ACTIVITY;
#endif
    p_qspi_env[id]->qspi_dma_state = APP_QSPI_DMA_ACTIVITY;

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_dma_deinit(app_qspi_id_t id)
{
    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_dma_state != APP_QSPI_DMA_ACTIVITY))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    app_dma_deinit(p_qspi_env[id]->dma_id);

    GLOBAL_EXCEPTION_DISABLE();
    p_qspi_env[id]->qspi_dma_state = APP_QSPI_DMA_INVALID;
    GLOBAL_EXCEPTION_ENABLE();
    if (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID)
    {
        p_qspi_env[id] = NULL;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_dma_command_receive_async(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL || p_data == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    if (p_qspi_env[id]->start_flag == false)
    {
        p_qspi_env[id]->start_flag = true;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        is_dma_access = true;
        APP_ASSERT_CHECK(p_qspi_env[id]->is_used_dma);
        app_qspi_switch_dma_mode(id, false);
#endif
        QSPI_SMART_CS_LOW(id);
        err_code = hal_qspi_command_receive_dma(&p_qspi_env[id]->handle, p_cmd, p_data);
        if (HAL_OK != err_code)
        {
            QSPI_SMART_CS_HIGH(id);
            p_qspi_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_dma_command_transmit_async(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL || p_data == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    if (p_qspi_env[id]->start_flag == false)
    {
        p_qspi_env[id]->start_flag = true;
        QSPI_SMART_CS_LOW(id);
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        APP_ASSERT_CHECK(p_qspi_env[id]->is_used_dma);
        app_qspi_switch_dma_mode(id, false);
        err_code = hal_qspi_command_transmit_dma(&p_qspi_env[id]->handle, p_cmd, p_data);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        app_qspi_config_dma_qwrite_32b_patch(id, QSPI_QUAD_WRITE_32b_PATCH_EN, QSPI_QUAD_WRITE_DATA_ENDIAN_MODE);
        err_code = hal_qspi_command_transmit_dma(&p_qspi_env[id]->handle, p_cmd, p_data);
        app_qspi_config_dma_qwrite_32b_patch(id, 0, QSPI_QUAD_WRITE_DATA_ENDIAN_MODE);
#endif
        if (err_code != HAL_OK)
        {
            QSPI_SMART_CS_HIGH(id);
            p_qspi_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_dma_command_async(app_qspi_id_t id, app_qspi_command_t *p_cmd)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    if (p_qspi_env[id]->start_flag == false)
    {
        p_qspi_env[id]->start_flag = true;
        QSPI_SMART_CS_LOW(id);
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        APP_ASSERT_CHECK(p_qspi_env[id]->is_used_dma);
        app_qspi_switch_dma_mode(id, false);
#endif
        err_code = hal_qspi_command_dma(&p_qspi_env[id]->handle, p_cmd);
        if (err_code != HAL_OK)
        {
            QSPI_SMART_CS_HIGH(id);
            p_qspi_env[id]->start_flag = false;
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
extern hal_status_t hal_qspi_transmit_dma_in_qpi(qspi_handle_t *p_qspi, uint32_t data_size, uint8_t *p_data, uint32_t length);

uint16_t app_qspi_dma_transmit_in_qpi_async(app_qspi_id_t id, uint32_t data_width, uint8_t *p_data, uint32_t length)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif
    QSPI_SMART_CS_LOW(id);
    err_code = hal_qspi_transmit_dma_in_qpi(&p_qspi_env[id]->handle, data_width, p_data, length);
    //QSPI_SMART_CS_HIGH(id);
    if (err_code != HAL_OK)
    {
        return err_code;
    }

    return APP_DRV_SUCCESS;
}
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
uint16_t app_qspi_dma_transmit_async(app_qspi_id_t id, uint8_t *p_data, uint32_t length)
{
    return app_qspi_dma_transmit_async_ex(id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, p_data, length);
}
#endif

uint16_t app_qspi_dma_transmit_async_ex(app_qspi_id_t id, uint32_t qspi_mode, uint32_t data_width, uint8_t *p_data, uint32_t length)
{
    hal_status_t err_code = HAL_ERROR;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    UNUSED(qspi_mode);
    UNUSED(data_width);
#endif

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    if (p_qspi_env[id]->start_flag == false)
    {
        p_qspi_env[id]->start_flag = true;
        QSPI_SMART_CS_LOW(id);
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        err_code = hal_qspi_transmit_dma(&p_qspi_env[id]->handle, p_data, length);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        APP_ASSERT_CHECK(p_qspi_env[id]->is_used_dma);
        app_qspi_switch_dma_mode(id, false);
        err_code = hal_qspi_transmit_dma(&p_qspi_env[id]->handle, qspi_mode, data_width, p_data, length);
#endif
        if (err_code != HAL_OK)
        {
            QSPI_SMART_CS_HIGH(id);
            p_qspi_env[id]->start_flag = false;
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
uint16_t app_qspi_dma_receive_async(app_qspi_id_t id, uint8_t *p_data, uint32_t length)
{
    return app_qspi_dma_receive_async_ex(id, 0, 0, p_data, length);
}
#endif

uint16_t app_qspi_dma_receive_async_ex(app_qspi_id_t id, uint32_t qspi_mode, uint32_t data_width, uint8_t *p_data, uint32_t length)
{
    hal_status_t err_code = HAL_ERROR;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    UNUSED(qspi_mode);
    UNUSED(data_width);
#endif
    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    if (p_qspi_env[id]->start_flag == false)
    {
        p_qspi_env[id]->start_flag = true;
        QSPI_SMART_CS_LOW(id);
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        err_code = hal_qspi_receive_dma(&p_qspi_env[id]->handle, p_data, length);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        is_dma_access = true;
        APP_ASSERT_CHECK(p_qspi_env[id]->is_used_dma);
        app_qspi_switch_dma_mode(id, false);
        err_code = hal_qspi_receive_dma(&p_qspi_env[id]->handle, qspi_mode, data_width, p_data, length);
#endif
        if (err_code != HAL_OK)
        {
            QSPI_SMART_CS_HIGH(id);
            p_qspi_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

static bool app_qspi_dma_match_check(app_qspi_params_t * p_params) {
    if(((p_params->id == APP_QSPI_ID_0) && (p_params->dma_cfg.dma_instance == DMA1)) ||
       ((p_params->id == APP_QSPI_ID_2) && (p_params->dma_cfg.dma_instance == DMA0))
     ) {
        if(APP_QSPI_EXCEPT_DEBUG_EN) printf("+++ ERR: QSPI/DMA DOES NOT MATCH !\r\n");
        return false;
    }

    if((p_params->dma_cfg.dma_channel != DMA_Channel0) && (p_params->dma_cfg.dma_channel != DMA_Channel1)) {
        if(APP_QSPI_EXCEPT_DEBUG_EN) printf("+++ ERR: Please AGGIGN DMA CHANNEL0/1 TO QSPI !\r\n");
        return false;
    }

    return true;
}

static bool app_qspi_switch_dma_mode(app_qspi_id_t id, bool is_m2m_mode) {
    app_dma_params_t dma_params = {0};

    if(p_qspi_env[id]->is_dma_mode_m2m == is_m2m_mode) {
        return true;
    } else {
        if(is_m2m_mode) {
            dma_params.p_instance                 = p_qspi_env[id]->dma_cfg.dma_instance;
            dma_params.channel_number             = p_qspi_env[id]->dma_cfg.dma_channel;
            dma_params.init.direction             = DMA_MEMORY_TO_MEMORY;
            dma_params.init.src_increment         = DMA_SRC_INCREMENT;
            dma_params.init.dst_increment         = DMA_DST_INCREMENT;
            dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
            dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
            dma_params.init.mode                  = DMA_NORMAL;
            dma_params.init.priority              = DMA_PRIORITY_LOW;
        } else {
            dma_params.p_instance                 = p_qspi_env[id]->dma_cfg.dma_instance;
            dma_params.channel_number             = p_qspi_env[id]->dma_cfg.dma_channel;
            dma_params.init.direction             = DMA_MEMORY_TO_PERIPH;
            dma_params.init.src_increment         = DMA_SRC_INCREMENT;
            dma_params.init.dst_increment         = DMA_DST_NO_CHANGE;
            dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
            dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
            dma_params.init.mode                  = DMA_NORMAL;
            dma_params.init.priority              = DMA_PRIORITY_LOW;
        }

        p_qspi_env[id]->dma_id = app_dma_init(&dma_params, NULL);
        if (p_qspi_env[id]->dma_id < 0)
        {
            return false;
        }
        p_qspi_env[id]->handle.p_dma = app_dma_get_handle(p_qspi_env[id]->dma_id);
        p_qspi_env[id]->handle.p_dma->p_parent = (void*)&p_qspi_env[id]->handle;
    }

    p_qspi_env[id]->is_dma_mode_m2m = is_m2m_mode;
    return true;
}

#endif

#endif


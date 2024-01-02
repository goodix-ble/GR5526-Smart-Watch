/**
  ****************************************************************************************
  * @file    app_gpiote.c
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
#include "app_gpiote.h"

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_gpiote_init(const app_gpiote_param_t *p_params, uint8_t table_cnt)
{
    app_io_init_t io_init;
    app_drv_err_t err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    for (uint8_t idx = 0; idx < table_cnt; idx++)
    {
        io_init.pin  = p_params[idx].pin;
        io_init.mode = p_params[idx].mode;
        io_init.pull = p_params[idx].pull;
        io_init.mux  = APP_IO_MUX;
        err_code = app_io_event_register_cb(p_params[idx].type, &io_init, p_params[idx].io_evt_cb, NULL);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_gpiote_config(const app_gpiote_param_t *p_config)
{
    app_io_init_t io_init;
    app_drv_err_t err_code;

    if (NULL == p_config)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    io_init.pin  = p_config->pin;
    io_init.mode = p_config->mode;
    io_init.pull = p_config->pull;
    io_init.mux  = APP_IO_MUX;

    err_code = app_io_event_register_cb(p_config->type, &io_init, p_config->io_evt_cb, NULL);
    APP_DRV_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_gpiote_deinit(app_io_type_t type, uint32_t pin)
{
    return app_io_event_unregister(type, pin);
}

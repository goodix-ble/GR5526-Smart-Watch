/**
  ****************************************************************************************
  * @file    app_soft_encoder.c
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
#include "app_soft_encoder.h"
#include "string.h"
#include "app_pwr_mgmt.h"
#include "gr55xx_delay.h"
#include "app_timer.h"
#include "app_gpiote.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define SOFT_ENCODER_STOP_STATUS          0x00

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_soft_encoder_direction_t s_cur_direction;  /* <The current encoder directoin>*/
static int s_cur_distance;                               /*<The current encoder directoin>*/

static app_gpiote_param_t s_gpiote_info[APP_SOFT_ENCODER_SINGAL_NUMBER];

static uint8_t s_singal_a_level;        /* <The current encoder signal io level>*/
static uint8_t s_singal_b_level;        /* <The current encoder signal io level>*/

static app_soft_encoder_callback s_ext_callback;/**< the soft encoder current singal irq type A\B*/


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void app_soft_encoder_analy_dir(uint8_t singal_a,uint8_t singal_b)
{
    if (singal_a == 1 && singal_b == 0)
    {
        s_cur_direction = APP_SOFT_ENCODER_POSITIVE;
        s_cur_distance++;
    }
    else if (singal_a == 0 && singal_b == 1)
    {
        s_cur_direction = APP_SOFT_ENCODER_REVERSE;
        s_cur_distance--;
    }
    else
    {
        return;
    }

    s_ext_callback(s_cur_direction,s_cur_distance);
}

static void app_soft_encoder_io_a_callback_t(app_io_evt_t *p_evt)
{
    uint8_t a_io_level = 0;

    delay_us(500);

    a_io_level  = app_io_read_pin(s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].type,s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].pin);

    if (APP_IO_PIN_SET == a_io_level && APP_IO_MODE_IT_RISING == s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].mode)
    {
        s_singal_a_level = a_io_level;
        s_singal_b_level = app_io_read_pin(s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].type,s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].pin);
        app_soft_encoder_analy_dir(s_singal_a_level,s_singal_b_level);
    }
    else if (APP_IO_PIN_RESET == a_io_level && APP_IO_MODE_IT_FALLING == s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].mode)
    {
        s_singal_a_level = a_io_level;
        s_singal_b_level = app_io_read_pin(s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].type,s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].pin);
        app_soft_encoder_analy_dir(s_singal_a_level,s_singal_b_level);
    }
}

static void app_soft_encoder_io_b_callback_t(app_io_evt_t *p_evt)
{
    uint8_t b_io_level = 0;

    delay_us(500);

    b_io_level  = app_io_read_pin(s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].type,s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].pin);

    if (APP_IO_PIN_SET == b_io_level && APP_IO_MODE_IT_RISING == s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].mode)
    {
        s_singal_b_level = b_io_level;
        s_singal_a_level = app_io_read_pin(s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].type,s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].pin);
        app_soft_encoder_analy_dir(s_singal_a_level, s_singal_b_level);
    }
    else if (APP_IO_PIN_RESET == b_io_level && APP_IO_MODE_IT_FALLING == s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].mode)
    {
        s_singal_b_level = b_io_level;
        s_singal_a_level = app_io_read_pin(s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].type,s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].pin);
        app_soft_encoder_analy_dir(s_singal_a_level, s_singal_b_level);
    }

}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_soft_encoder_init(app_soft_encoder_io_param_t *p_a_params, app_soft_encoder_io_param_t *p_b_params, app_soft_encoder_callback evt_handler)
{
    uint16_t error_code;

    if (evt_handler == NULL || p_a_params == NULL || p_b_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    s_ext_callback = evt_handler;


    s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].type        = p_a_params->type;
    s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].pin         = p_a_params->pin;
    s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].mode        = p_a_params->mode;
    s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].pull        = p_a_params->pull;
    s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_A].io_evt_cb   = app_soft_encoder_io_a_callback_t;

    s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].type        = p_b_params->type;
    s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].pin         = p_b_params->pin;
    s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].mode        = p_b_params->mode;
    s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].pull        = p_b_params->pull;
    s_gpiote_info[APP_SOFT_ENCODER_SIGNAL_B].io_evt_cb   = app_soft_encoder_io_b_callback_t;

    error_code = app_gpiote_init(s_gpiote_info, APP_SOFT_ENCODER_SINGAL_NUMBER);
    APP_DRV_ERR_CODE_CHECK(error_code);

    s_cur_direction = APP_SOFT_ENCODER_STOP;
    s_cur_distance = 0;


    s_singal_a_level = 0;
    s_singal_b_level = 0;

    return APP_DRV_SUCCESS;
}


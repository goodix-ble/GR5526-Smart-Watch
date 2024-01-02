/**
 *****************************************************************************************
 *
 * @file app_key.c
 *
 * @brief App key Implementation.
 *
 *****************************************************************************************
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
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_key.h"
#include "app_gpiote.h"

#ifdef ENV_USE_FREERTOS
#include "FreeRTOS.h"
#include "timers.h"
#else
#include "app_timer.h"
#endif
/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_KEY_TIMER_INTERVAL   10    /**< App key polling interval interval (in units of 1ms). */
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t            s_app_key_info[APP_KEY_REG_COUNT_MAX];
static app_gpiote_param_t s_app_io_cfg[APP_KEY_REG_COUNT_MAX];
static app_key_evt_cb_t   s_app_key_evt_cb;
static uint8_t            s_app_key_reg_num;
#ifdef ENV_USE_FREERTOS
static TimerHandle_t app_key_timer_handle = NULL;
#else
static app_timer_id_t     s_app_key_timer_id;
#endif
static bool               s_is_timer_enabled;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Polling app key pressed state.
 *****************************************************************************************
 */
static void app_key_press_state_polling(void)
{
    app_io_pin_state_t   pin_state;
    bool                 is_pressed = false;

    for (uint8_t key_idx = 0; key_idx < s_app_key_reg_num; key_idx++)
    {
        is_pressed = false;

        pin_state = app_io_read_pin(s_app_io_cfg[key_idx].type, s_app_io_cfg[key_idx].pin);

        if (APP_IO_PIN_RESET == pin_state && s_app_io_cfg[key_idx].pull == APP_IO_PULLUP)
        {
            is_pressed = true;
        }
        else if (APP_IO_PIN_SET == pin_state && s_app_io_cfg[key_idx].pull == APP_IO_PULLDOWN)
        {
            is_pressed = true;
        }

        app_key_core_key_pressed_record(key_idx, is_pressed);
    }
}

/**
 *****************************************************************************************
 * @brief App key timing timeout handler.
 *****************************************************************************************
 */
static void app_key_timeout_handler(void *p_arg)
{
    app_key_press_state_polling();
    app_key_core_polling_10ms();
}

/**
 *****************************************************************************************
 * @brief Start app key timer.
 *****************************************************************************************
 */
static void app_key_timer_start(void)
{
#if defined(ENV_USE_FREERTOS)
     xTimerStartFromISR(app_key_timer_handle, 0);
#else
    app_timer_create(&s_app_key_timer_id, ATIMER_REPEAT, app_key_timeout_handler);
    app_timer_start(s_app_key_timer_id, APP_KEY_TIMER_INTERVAL, NULL);
#endif
    s_is_timer_enabled = true;
}

/**
 *****************************************************************************************
 * @brief Stop app key timer.
 *****************************************************************************************
 */
void app_key_timer_stop(void)
{
#if defined(ENV_USE_FREERTOS)
    xTimerStopFromISR(app_key_timer_handle,0);
#else
    app_timer_delete(&s_app_key_timer_id);
#endif
    s_is_timer_enabled = false;
}

/**
 *****************************************************************************************
 * @brief App key core event handler.
 *****************************************************************************************
 */
static void app_key_core_evt_handler(uint8_t key_idx, app_key_click_type_t key_click_type)
{
    if (app_key_core_is_all_release())
    {
        app_key_timer_stop();
    }

    s_app_key_evt_cb(s_app_key_info[key_idx], key_click_type);
}

static void app_gpiote_event_handler(app_io_evt_t *p_evt)
{
    if ( NULL == p_evt)
        return;

    for (uint8_t key_idx = 0; key_idx < s_app_key_reg_num; key_idx++)
    {
        if ((s_app_io_cfg[key_idx].type == p_evt->type)&&
            (p_evt->pin & s_app_io_cfg[key_idx].pin))
        {
            if (!s_is_timer_enabled)
            {
                app_key_timer_start();
            }
            app_key_core_key_wait_polling_record(key_idx);
            return;
        }
    }
}



/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
bool app_key_init(app_key_gpio_t key_inst[], uint8_t key_num, app_key_evt_cb_t key_evt_cb)
{
    if (APP_KEY_REG_COUNT_MAX < key_num || NULL == key_evt_cb)
    {
        return false;
    }

    for (uint8_t key_idx = 0; key_idx < key_num; key_idx++)
    {
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        if (key_inst[key_idx].gpio_type != APP_IO_TYPE_NORMAL &&
            key_inst[key_idx].gpio_type != APP_IO_TYPE_AON)
        {
            continue;
        }
#elif (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
        if (key_inst[key_idx].gpio_type != APP_IO_TYPE_GPIOA &&
            key_inst[key_idx].gpio_type != APP_IO_TYPE_GPIOB &&
            key_inst[key_idx].gpio_type != APP_IO_TYPE_GPIOC &&
            key_inst[key_idx].gpio_type != APP_IO_TYPE_AON)
        {
            continue;
        }
#endif
        s_app_key_info[key_idx] = key_inst[key_idx].key_id;
        s_app_io_cfg[key_idx].type = key_inst[key_idx].gpio_type;
        s_app_io_cfg[key_idx].pin  = key_inst[key_idx].gpio_pin;
        s_app_io_cfg[key_idx].mode = key_inst[key_idx].trigger_mode;
        s_app_io_cfg[key_idx].pull = key_inst[key_idx].pull;
        s_app_io_cfg[key_idx].io_evt_cb = app_gpiote_event_handler;
    }

    app_gpiote_init(s_app_io_cfg, key_num);

    s_app_key_reg_num = key_num;
    s_app_key_evt_cb  = key_evt_cb;
    app_key_core_cb_register(app_key_core_evt_handler);

/* If using FreeRTOS, xTimer need to be create and do not create in IRQ handler */
#if defined(ENV_USE_FREERTOS)
    app_key_timer_handle = xTimerCreate(NULL, APP_KEY_TIMER_INTERVAL, pdTRUE, NULL, (TimerCallbackFunction_t)app_key_timeout_handler);
#endif
    return true;
}



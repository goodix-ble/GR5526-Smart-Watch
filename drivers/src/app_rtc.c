/**
  ****************************************************************************************
  * @file    app_rtc.c
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
#include "app_rtc.h"
#include <string.h>
#include "platform_sdk.h"
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
#include "gr55xx_pwr.h"
#endif
#include "gr_soc.h"

#ifdef HAL_CALENDAR_MODULE_ENABLED

#if (CFG_LPCLK_INTERNAL_EN == 0) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/**@brief App rtc state types. */
typedef enum
{
    APP_RTC_INVALID = 0,
    APP_RTC_ACTIVITY,
} app_rtc_state_t;

struct rtc_env_t
{
    app_rtc_evt_handler_t   evt_handler;
    calendar_handle_t       handle;
    app_rtc_state_t         rtc_state;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */

void CALENDAR_IRQHandler(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

struct rtc_env_t  s_rtc_env;

/*
 * EXPORTED VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
extern float SystemRtcSlowClock;
#endif

/*
* NOTE:
* This defined value is only for SOC GR5332
* 1.RTC is fully functional without FreeRTOS
* 2.In FreeRTOS, only init time and get time work.
*/
#if defined(ENV_USE_FREERTOS) && defined(SOC_GR5332)

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_rtc_init(app_rtc_evt_handler_t evt_handler)
{
    s_rtc_env.evt_handler = evt_handler;
    s_rtc_env.handle.clock_freq = SystemRtcSlowClock;

    clock_calib_notify_register(app_rtc_time_sync);

    s_rtc_env.rtc_state = APP_RTC_ACTIVITY;


    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_deinit(void)
{
    if (s_rtc_env.rtc_state == APP_RTC_INVALID)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    clock_calib_notify_register(NULL);

    s_rtc_env.rtc_state = APP_RTC_INVALID;

    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_init_time(app_rtc_time_t *p_time)
{
    hal_status_t err_code;

    if (s_rtc_env.rtc_state == APP_RTC_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_time == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    err_code = hal_calendar_init_time(&s_rtc_env.handle, p_time);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_get_time(app_rtc_time_t *p_time)
{
    hal_status_t err_code;

    if (s_rtc_env.rtc_state == APP_RTC_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_time == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    err_code = hal_calendar_get_time(&s_rtc_env.handle, p_time);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_setup_alarm(app_rtc_alarm_t *p_alarm)
{
    if (s_rtc_env.rtc_state == APP_RTC_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_alarm == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_setup_tick(uint32_t interval)
{
    if (s_rtc_env.rtc_state == APP_RTC_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (interval == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    return APP_DRV_SUCCESS;
}

void app_rtc_time_sync(float SlowClockFreq)
{
    if (s_rtc_env.rtc_state == APP_RTC_INVALID || SlowClockFreq == 0)
    {
        return;
    }

    hal_calendar_sync_time(&s_rtc_env.handle, SlowClockFreq);
}

#else

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void app_rtc_event_call(calendar_handle_t *p_calendar, app_rtc_evt_type_t evt_type)
{
    app_rtc_evt_t rtc_evt;

    rtc_evt.type = evt_type;
    if(s_rtc_env.evt_handler != NULL)
    {
        s_rtc_env.evt_handler(&rtc_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_rtc_init(app_rtc_evt_handler_t evt_handler)
{
    hal_status_t err_code;

    s_rtc_env.evt_handler = evt_handler;

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    s_rtc_env.handle.clock_freq = SystemRtcSlowClock;
#endif

    soc_register_nvic(CALENDAR_IRQn, (uint32_t)CALENDAR_IRQHandler);
    err_code = hal_calendar_deinit(&s_rtc_env.handle);
    HAL_ERR_CODE_CHECK(err_code);
    err_code = hal_calendar_init(&s_rtc_env.handle);
    HAL_ERR_CODE_CHECK(err_code);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    clock_calib_notify_register(app_rtc_time_sync);
#endif

    s_rtc_env.rtc_state = APP_RTC_ACTIVITY;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    pwr_mgmt_wakeup_source_setup(PWR_WKUP_COND_CALENDAR);
#endif

    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_deinit(void)
{
    hal_status_t err_code;

    if (s_rtc_env.rtc_state == APP_RTC_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    hal_calendar_disable_event(&s_rtc_env.handle, CALENDAR_ALARM_DISABLE_ALL);

    err_code = hal_calendar_deinit(&s_rtc_env.handle);
    HAL_ERR_CODE_CHECK(err_code);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    clock_calib_notify_register(NULL);
#endif

    s_rtc_env.rtc_state = APP_RTC_INVALID;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    pwr_mgmt_wakeup_source_clear(PWR_WKUP_COND_CALENDAR);
#endif

    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_init_time(app_rtc_time_t *p_time)
{
    hal_status_t err_code;

    if (s_rtc_env.rtc_state == APP_RTC_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_time == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    err_code = hal_calendar_init_time(&s_rtc_env.handle, p_time);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_get_time(app_rtc_time_t *p_time)
{
    hal_status_t err_code;

    if (s_rtc_env.rtc_state == APP_RTC_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_time == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    err_code = hal_calendar_get_time(&s_rtc_env.handle, p_time);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_setup_alarm(app_rtc_alarm_t *p_alarm)
{
    hal_status_t err_code;

    if (s_rtc_env.rtc_state == APP_RTC_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_alarm == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    err_code = hal_calendar_set_alarm(&s_rtc_env.handle, p_alarm);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_setup_tick(uint32_t interval)
{
    hal_status_t err_code;

    if (s_rtc_env.rtc_state == APP_RTC_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    if (interval < 5)
#else
    if (interval == 0)
#endif
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_calendar_set_tick(&s_rtc_env.handle, interval);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_disable_event(uint32_t disable_mode)
{
    hal_status_t err_code;

    if (s_rtc_env.rtc_state == APP_RTC_INVALID)
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    err_code = hal_calendar_disable_event(&s_rtc_env.handle, disable_mode);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
void app_rtc_time_sync(float SlowClockFreq)
{
    if (s_rtc_env.rtc_state == APP_RTC_INVALID || SlowClockFreq == 0)
    {
        return;
    }

    hal_calendar_sync_time(&s_rtc_env.handle, SlowClockFreq);
}
#endif

void hal_calendar_alarm_callback(calendar_handle_t *p_calendar)
{
    app_rtc_event_call(p_calendar, APP_RTC_EVT_DATE_ALARM);
}

void hal_calendar_tick_callback(calendar_handle_t *p_calendar)
{
    app_rtc_event_call(p_calendar, APP_RTC_EVT_TICK_ALARM);
}

void hal_calendar_overflow_callback(calendar_handle_t *p_calendar)
{
    app_rtc_event_call(p_calendar, APP_RTC_EVT_OVERFLOW);
}

SECTION_RAM_CODE void CALENDAR_IRQHandler(void)
{
    hal_calendar_irq_handler(&s_rtc_env.handle);
}

#endif

#else

#include "ble_time.h"
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint64_t s_sys_sync_hus = 0; // Record the hus from 01.01.2000 00:00 to the time when time sync
static ble_time_t s_ble_sync_time = {0, 0}; // Record the ble time when time sync

/* Caculate seconds from 01.01.2000 00:00 to the time */
void calendar_time2seconds(calendar_time_t *p_time, uint32_t *p_seconds)
{
    uint16_t year;
    uint32_t utc;

    // 10957 is the days between 1970/1/1 and 2000/1/1
    year = (p_time->year + 2000) % 100;
    utc  = 10957;
    utc += (year * 365 + (year + 3) / 4);
    utc += (367 * p_time->mon - 362) / 12 - (p_time->mon <= 2 ? 0 : ((year % 4 == 0) ? 1 : 2));
    utc += (p_time->date - 1);
    utc *= 86400;
    utc += (p_time->hour * 3600 + p_time->min * 60 + p_time->sec);

    *p_seconds = utc;
}

/* Caculate time with seconds from 01.01.2000 00:00 to the p_time */
void seconds2_calendar_time(calendar_time_t *p_time, uint32_t seconds)
{
    const uint16_t days0[] = { 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366 };
    const uint16_t days1[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 };

    calendar_time_t time;
    uint32_t days, secs;
    uint16_t year;
    const uint16_t * dayp;

    // 10957 is the days between 1970/1/1 and 2000/1/1
    if (seconds >= 10957 * 86400)
    {
        days = seconds / 86400 - 10957;
        secs = seconds % 86400;
    }
    else
    {
        days = 0;
        secs = 0;
    }

    time.sec    = secs % 60;    secs /= 60;
    time.min    = secs % 60;    secs /= 60;
    time.hour   = secs;

    year        = 2000;
    time.week   = (days + 6) % 7;

    year        += (days / 1461) * 4; days %= 1461;
    if (days >= 366)
        dayp = days1;
    else
        dayp = days0;
    if (days >= 366)
    {
        year    += (days - 1) / 365;
        days    = (days - 1) % 365;
    }

    time.mon    = days / 31 + 1;
    if (days >= dayp[time.mon])
        time.mon += 1;

    time.date   = days - dayp[time.mon - 1] + 1;
    time.year   = year - 2000;

    memcpy(p_time, &time, sizeof(calendar_time_t));
}

/* The calendar_time_sync shall be called every 23 hours to keep counting continously*/
void calendar_time_sync(void)
{
    // protect the calendar time sync operation
    GLOBAL_EXCEPTION_DISABLE();

    // calculate the sync diff
    ble_time_t current_time = ble_time_get();
    uint64_t diff_hus = (uint64_t)CLK_SUB(current_time.hs, s_ble_sync_time.hs)*HALF_SLOT_SIZE;
    diff_hus = diff_hus +  current_time.hus - s_ble_sync_time.hus;

    // sync the calendar time
    s_sys_sync_hus = s_sys_sync_hus + diff_hus;
    s_ble_sync_time = current_time;

    GLOBAL_EXCEPTION_ENABLE();
}

uint16_t app_rtc_init_time(app_rtc_time_t *p_time)
{
    // calculate the calendar time in seconds
    uint32_t seconds = 0;
    calendar_time2seconds(p_time, &seconds);

    // initialize the calendar time in hus
    s_sys_sync_hus = ((uint64_t)(seconds) * SECOND_IN_HUS) + (TICK_MS_IN_HUS * p_time->ms);
    s_ble_sync_time = ble_time_get();

    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_get_time(app_rtc_time_t *p_time)
{
    // sync the calendar time
    calendar_time_sync();

    // calculate the calendar time in seconds
    uint32_t seconds = s_sys_sync_hus / SECOND_IN_HUS;
    seconds2_calendar_time(p_time, seconds);

    // calculate the calendar time in ms
    uint32_t ms = (s_sys_sync_hus % SECOND_IN_HUS) / TICK_MS_IN_HUS;
    p_time->ms = ms;
    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_init(app_rtc_evt_handler_t evt_handler)
{
    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_deinit(void)
{
    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_setup_alarm(app_rtc_alarm_t *p_alarm)
{
    return APP_DRV_SUCCESS;
}

uint16_t app_rtc_setup_tick(uint32_t interval)
{
    return APP_DRV_SUCCESS;
}

#endif

#endif


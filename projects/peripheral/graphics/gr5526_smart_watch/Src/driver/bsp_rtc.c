/**
 *****************************************************************************************
 *
 * @file bsp_rtc.c
 *
 * @brief  User Periph Init Function Implementation.
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
#include "time.h"
#include "bsp_rtc.h"
#include <string.h>
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * LOCAL  FUNCTION DEFINITIONS
 *****************************************************************************************
 */

//static void user_rtc_evt_handler(app_rtc_evt_t *p_evt)
//{
//    if (p_evt->type == APP_RTC_EVT_TICK_ALARM)
//    {
//    }
//}

static bool s_rtc_time_updated = false;

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void rtc_init(void)
{
    #if GOMORE_ENABLE > 0u
    app_rtc_init(app_rtc_evt_handler);
    #else
    app_rtc_init(NULL);
    #endif

    app_rtc_time_t time;
    time.year = 22;
    time.mon = 10;
    time.date = 28;
    time.hour = 16;
    time.min = 13;
    time.sec = 0;
    time.week = 0;
    time.ms = 0;
    uint16_t ret = app_rtc_init_time(&time);
}

int rtc_sync_time(uint64_t time)
{
    app_rtc_time_t s_time;
    time_t sec_time;
    struct tm tp;
    uint16_t ret;

    s_time.ms = (uint16_t)(time % 1000);
    sec_time = (time_t)(time / 1000) + 28800; // need add 8 hours;

    localtime_r((time_t *)&sec_time, &tp);
    s_time.year = (uint8_t)tp.tm_year + 1900 - 2000;
    s_time.mon = (uint8_t)tp.tm_mon + 1;
    s_time.date = (uint8_t)tp.tm_mday;
    s_time.week = (uint8_t)tp.tm_wday;
    s_time.hour = (uint8_t)tp.tm_hour;
    s_time.min = (uint8_t)tp.tm_min;
    s_time.sec = (uint8_t)tp.tm_sec;
#if 0
    APP_LOG_DEBUG("set time:year: %u, mon: %u, date: %u, weekday: %u, hour: %u,min: %u, second: %u, ms: %u",
                  s_time.year + 2000, s_time.mon, s_time.date, s_time.week,
                  s_time.hour, s_time.min, s_time.sec, s_time.ms);
#endif
    ret = app_rtc_init_time(&s_time);
    if (ret != APP_DRV_SUCCESS)
    {
        return -1;
    }

    s_rtc_time_updated = true;

    return 0;
}

uint64_t rtc_get_timestamp(void)
{
    time_t sec_time;
    struct tm tmp_tm;
    uint64_t timestamp = 0;
    int yday = 0;
    app_rtc_time_t time;

    GLOBAL_EXCEPTION_DISABLE();
    app_rtc_get_time(&time);
    GLOBAL_EXCEPTION_ENABLE();

    switch (time.mon)
    {
    case 1:
        yday = 0;
        break;
    case 2:
        yday = 31;
        break;
    case 3:
        yday = 59;
        break;
    case 4:
        yday = 90;
        break;
    case 5:
        yday = 120;
        break;
    case 6:
        yday = 151;
        break;
    case 7:
        yday = 181;
        break;
    case 8:
        yday = 212;
        break;
    case 9:
        yday = 243;
        break;
    case 10:
        yday = 273;
        break;
    case 11:
        yday = 304;
        break;
    case 12:
        yday = 334;
        break;
    default:
        break;
    }
    yday = yday + time.date;

    /* If it is a leap year and time.mon > 2, add 1 */
    if (((0 == time.year % 400) || ((0 == time.year % 4) &&
                                    (0 != time.year % 100))) &&
        (time.mon > 2))
    {
        yday += 1;
    }

    memset(&tmp_tm, '0', sizeof(tmp_tm));
    tmp_tm.tm_sec = time.sec;
    tmp_tm.tm_min = time.min;
    tmp_tm.tm_hour = (time.hour - 8);
    tmp_tm.tm_mday = time.date;
    tmp_tm.tm_mon = (time.mon - 1);
    tmp_tm.tm_year = (time.year + 2000 - 1900);
    tmp_tm.tm_wday = time.week;
    tmp_tm.tm_yday = yday;
    tmp_tm.tm_isdst = 0;

#if 0
    APP_LOG_DEBUG("tm  time:year: %u, mon: %u, date: %u, weekday: %u, hour: %u,min: %u, second: %u, ms: %u",
           tmp_tm.tm_year, tmp_tm.tm_mon, tmp_tm.tm_mday, tmp_tm.tm_wday,
           tmp_tm.tm_hour, tmp_tm.tm_min, tmp_tm.tm_sec, time.ms);
#endif

    sec_time = mktime(&tmp_tm);
    timestamp = (uint64_t)sec_time * 1000 + (time.ms % 999);

#if 0
    APP_LOG_DEBUG("timestamp = %llu sec_time = %d", timestamp, sec_time);
#endif
    return timestamp;
}

bool rtc_get_updated_flag(void)
{
    return s_rtc_time_updated;
}

void rtc_clear_updated_flag(void)
{
    s_rtc_time_updated = false;
}

void rtc_set_updated_flag(void)
{
    s_rtc_time_updated = true;
}

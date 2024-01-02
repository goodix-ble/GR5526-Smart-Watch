/**
  ****************************************************************************************
  * @file    app_alarm.c
  * @author  BLE Driver Team
  * @brief   APP Alarm Library.
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
#include "grx_hal.h"
#include "app_alarm.h"

#ifdef HAL_CALENDAR_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

#define _LOCAL_APP_ALARM_LOCK()                                  \
    uint32_t __l_irq_rest = __get_BASEPRI();                     \
    __set_BASEPRI(NVIC_GetPriority(BLE_IRQn) +                   \
                 (1 << (NVIC_GetPriorityGrouping() + 1)));

#define _LOCAL_APP_ALARM_UNLOCK()                                \
    __set_BASEPRI(__l_irq_rest);

#define APP_ALARM_BASE_ALARM_MARK               0x9f00
#define APP_ALARM_FIRST_YEAR                    (2000UL)
#define APP_ALARM_SECONDS_PER_HOUR              (3600UL)
#define APP_ALARM_SECONDS_PER_DAY               (24UL * APP_ALARM_SECONDS_PER_HOUR)
#define APP_ALARM_SECONDS_PER_YEAR              (365UL * APP_ALARM_SECONDS_PER_DAY)
#define APP_ALARM_DAYS_PER_MONTH                {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
#define IS_APP_ALARM_LEAP_YEAR(__YEAR__)        ((((__YEAR__) % 4) == 0 && ((__YEAR__) % 100) != 0) || \
                                                ((__YEAR__) % 400) == 0)

#define APP_ALARM_LOCK()                        _LOCAL_APP_ALARM_LOCK() 
#define APP_ALARM_UNLOCK()                      _LOCAL_APP_ALARM_UNLOCK()

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
typedef struct
{
    app_alarm_id_t      alarm_id;
    uint32_t            delay;
    app_alarm_t         alarm;
} app_alarm_node_t;

typedef struct
{
    app_alarm_node_t    alarm_list[MAX_ALARM_SUPPORT];
    app_alarm_node_t    alarm0;
    calendar_handle_t   handle;
    bool                initialized;
    uint8_t             alarm_available;
    NvdsTag_t           alarm_data_tag;
    app_alarm_fun_t     callback;
} app_alarm_info_t;

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static uint32_t time2seconds(const app_time_t *time);
static void update_alarm_delay(const app_time_t *cur_time, const uint32_t cur_seconds, app_alarm_node_t *alarm_node);
static void calendar_alarm_cb(calendar_handle_t *hcalendar);
static void sort_alarm_list(void);
static uint8_t check_alarm_existed_alarm_id(app_alarm_id_t alarm_id);
static uint8_t check_alarm_existed_alarm(const app_alarm_t *alarm);
static uint16_t save_alarm_list(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_alarm_info_t s_app_alarm_info = {0};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_alarm_init(NvdsTag_t tag, app_alarm_fun_t callback)
{
    hal_status_t hal_err_code;
    uint8_t error_code;
    uint16_t len;
    app_alarm_node_t alarm_buf[MAX_ALARM_SUPPORT];

    if (0xFFFF == tag || 0x0000 == tag || NULL == callback)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    /* Clear rtc_alarm globle info */
    memset(&s_app_alarm_info, 0, sizeof(app_alarm_info_t));

    /* Initialize calendar */
    hal_err_code = hal_calendar_init(&s_app_alarm_info.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    /* Copy alarm setting from NVDS to buffer */
    s_app_alarm_info.alarm_data_tag = tag;
    s_app_alarm_info.callback = callback;
    len = sizeof(alarm_buf);
    error_code = nvds_get(s_app_alarm_info.alarm_data_tag, &len, (uint8_t *)alarm_buf);
    if (NVDS_SUCCESS != error_code)
    {
        if (NVDS_TAG_NOT_EXISTED == error_code)
        {
            memset(alarm_buf, 0, sizeof(alarm_buf));
            error_code = nvds_put(s_app_alarm_info.alarm_data_tag, sizeof(alarm_buf), (const uint8_t *)alarm_buf);
            if (NVDS_SUCCESS != error_code)
            {
                return HAL_ERROR;
            }
        }
        else
        {
            return HAL_ERROR;
        }
    }

    /* Base alarm, generate an alarm clock at 0 o'clock every day */
    s_app_alarm_info.alarm0.alarm.hour = 0;
    s_app_alarm_info.alarm0.alarm.min  = 0;
    s_app_alarm_info.alarm0.alarm.alarm_date_week_mask = 0x7F;
    s_app_alarm_info.alarm0.alarm.alarm_sel = CALENDAR_ALARM_SEL_WEEKDAY;
    s_app_alarm_info.alarm0.alarm_id = APP_ALARM_BASE_ALARM_MARK;

    /* Check the number of alarm */
    for (uint8_t i = 0; i < MAX_ALARM_SUPPORT; i++)
    {
        if (alarm_buf[i].alarm.alarm_date_week_mask)
        {
            memcpy(&s_app_alarm_info.alarm_list[i], &alarm_buf[i], sizeof(app_alarm_node_t));
            s_app_alarm_info.alarm_available++;
        }
    }

    s_app_alarm_info.initialized = true;

    return APP_DRV_SUCCESS;
}

uint16_t app_alarm_deinit(void)
{
    hal_status_t  hal_err_code;
    uint8_t error_code;

    if (!s_app_alarm_info.initialized)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    hal_err_code = hal_calendar_disable_event(&s_app_alarm_info.handle, CALENDAR_ALARM_DISABLE_ALL);
    HAL_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_calendar_deinit(&s_app_alarm_info.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    error_code = nvds_del(s_app_alarm_info.alarm_data_tag);
    if (NVDS_SUCCESS != error_code)
    {
        return APP_DRV_ERR_HAL;
    }

    /* Clear rtc_alarm globle info */
    memset(&s_app_alarm_info, 0, sizeof(app_alarm_info_t));

    return APP_DRV_SUCCESS;
}

uint16_t app_alarm_add(const app_alarm_t *alarm, app_alarm_id_t p_alarm_id)
{
    hal_status_t  hal_err_code;
    app_drv_err_t app_err_code;

    if (NULL == alarm ||
        MAX_ALARM_SUPPORT <= s_app_alarm_info.alarm_available ||
        !s_app_alarm_info.initialized ||
        APP_ALARM_BASE > p_alarm_id)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    APP_ALARM_LOCK();
    /* Check the alarm is existed, avoid to load the same alarm */
    if (check_alarm_existed_alarm(alarm) == s_app_alarm_info.alarm_available)
    {
        /* Insert to end */
        memcpy(&s_app_alarm_info.alarm_list[s_app_alarm_info.alarm_available].alarm, alarm, sizeof(app_alarm_t));
        s_app_alarm_info.alarm_list[s_app_alarm_info.alarm_available].alarm_id = p_alarm_id ;
        s_app_alarm_info.alarm_available++;

        /* Sort alarm then set the lastest */
        sort_alarm_list();
        if (s_app_alarm_info.alarm0.delay <= s_app_alarm_info.alarm_list[0].delay)
        {
            hal_err_code = hal_calendar_set_alarm(&s_app_alarm_info.handle, &s_app_alarm_info.alarm0.alarm);
        }
        else
        {
            hal_err_code = hal_calendar_set_alarm(&s_app_alarm_info.handle, &s_app_alarm_info.alarm_list[0].alarm);
        }
        
        if (HAL_OK != hal_err_code)
        {
            APP_ALARM_UNLOCK();
            return hal_err_code;
        }

        app_err_code = save_alarm_list();
        if (APP_DRV_SUCCESS != app_err_code)
        {
            APP_ALARM_UNLOCK();
            return app_err_code;
        }
    }
    APP_ALARM_UNLOCK();

    return APP_DRV_SUCCESS;
}

uint16_t app_alarm_del(app_alarm_id_t alarm_id)
{
    hal_status_t  hal_err_code;
    app_drv_err_t app_err_code;

    if (0 == s_app_alarm_info.alarm_available ||
        !s_app_alarm_info.initialized ||
        APP_ALARM_BASE > alarm_id)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    APP_ALARM_LOCK();
    uint8_t indx = check_alarm_existed_alarm_id(alarm_id);
    if (indx < s_app_alarm_info.alarm_available)
    {
        /* Delete the indx */
        for (uint8_t i = indx; i < s_app_alarm_info.alarm_available - 1; i++)
        {
            memcpy(&s_app_alarm_info.alarm_list[i], &s_app_alarm_info.alarm_list[i + 1], sizeof(app_alarm_node_t));
        }
        s_app_alarm_info.alarm_available--;
        /* Check alarm available, if there is not any alarm then disable alarm */
        if (s_app_alarm_info.alarm_available > 0)
        {
            if (s_app_alarm_info.alarm0.delay <= s_app_alarm_info.alarm_list[0].delay)
            {
                hal_err_code = hal_calendar_set_alarm(&s_app_alarm_info.handle, &s_app_alarm_info.alarm0.alarm);
            }
            else
            {
                hal_err_code = hal_calendar_set_alarm(&s_app_alarm_info.handle, &s_app_alarm_info.alarm_list[0].alarm);
            }
            if (HAL_OK != hal_err_code)
            {
                APP_ALARM_UNLOCK();
                return hal_err_code;
            }
        }
        else
        {
            hal_err_code = hal_calendar_disable_event(&s_app_alarm_info.handle, CALENDAR_ALARM_DISABLE_ALL);
            if (HAL_OK != hal_err_code)
            {
                APP_ALARM_UNLOCK();
                return hal_err_code;
            }
        }

        app_err_code = save_alarm_list();
        if (APP_DRV_SUCCESS != app_err_code)
        {
            APP_ALARM_UNLOCK();
            return app_err_code;
        }
    }
    else
    {
        APP_ALARM_UNLOCK();
        return APP_DRV_ERR_INVALID_PARAM;
    }

    APP_ALARM_UNLOCK();
    return APP_DRV_SUCCESS;
}

uint16_t app_alarm_del_all(void)
{
    hal_status_t  hal_err_code;
    app_drv_err_t app_err_code;

    if (0 == s_app_alarm_info.alarm_available ||
        !s_app_alarm_info.initialized)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    APP_ALARM_LOCK();
    for (uint8_t i = 0; i < s_app_alarm_info.alarm_available; i++)
    {
        memset(&s_app_alarm_info.alarm_list[i], 0, sizeof(app_alarm_node_t));
    }
    s_app_alarm_info.alarm_available = 0;
    hal_err_code = hal_calendar_disable_event(&s_app_alarm_info.handle, CALENDAR_ALARM_DISABLE_ALL);
    if (HAL_OK != hal_err_code)
    {
        APP_ALARM_UNLOCK();
        return hal_err_code;
    }

    app_err_code = save_alarm_list();
    if (APP_DRV_SUCCESS != app_err_code)
    {
        APP_ALARM_UNLOCK();
        return app_err_code;
    }

    APP_ALARM_UNLOCK();

    return APP_DRV_SUCCESS;
}

uint16_t app_alarm_get_time(app_time_t *p_time)
{
    hal_status_t  hal_err_code;

    if (p_time == NULL || !s_app_alarm_info.initialized)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    hal_err_code = hal_calendar_get_time(&s_app_alarm_info.handle, p_time);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_alarm_set_time(app_time_t *p_time)
{
    hal_status_t  hal_err_code;

    if (p_time == NULL || !s_app_alarm_info.initialized)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    hal_err_code = hal_calendar_init_time(&s_app_alarm_info.handle, p_time);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

/* If you reset system time, you should call this function again */
uint16_t app_alarm_reload(void)
{
    hal_status_t hal_err_code;

    if (!s_app_alarm_info.initialized)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (0 < s_app_alarm_info.alarm_available)
    {
        /* Sort alarm then set the lastest */
        sort_alarm_list();
        sys_delay_ms(100);
        if (s_app_alarm_info.alarm0.delay <= s_app_alarm_info.alarm_list[0].delay)
        {
             hal_err_code = hal_calendar_set_alarm(&s_app_alarm_info.handle, &s_app_alarm_info.alarm0.alarm);
        }
        else
        {
            hal_err_code = hal_calendar_set_alarm(&s_app_alarm_info.handle, &s_app_alarm_info.alarm_list[0].alarm);
        }
        HAL_ERR_CODE_CHECK(hal_err_code);
    }

    return APP_DRV_SUCCESS;
}

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/* Caculate seconds from 01.01.2000 00:00 to the input time */
static uint32_t time2seconds(const app_time_t *time)
{
    uint32_t seconds = 0;
    uint32_t mon_days[12] = APP_ALARM_DAYS_PER_MONTH;

    seconds = time->year * APP_ALARM_SECONDS_PER_YEAR + ((time->year + 3) / 4) * APP_ALARM_SECONDS_PER_DAY;

    mon_days[1] = IS_APP_ALARM_LEAP_YEAR((uint32_t)time->year + APP_ALARM_FIRST_YEAR) ? 29 : 28;
    for (uint32_t i = 0; i < time->mon - 1; i++)
    {
        seconds += mon_days[i] * APP_ALARM_SECONDS_PER_DAY;
    }

    seconds += (uint32_t)(time->date - 1) * APP_ALARM_SECONDS_PER_DAY;

    seconds += (uint32_t)time->hour * APP_ALARM_SECONDS_PER_HOUR;

    seconds += (uint32_t)time->min * 60 + (uint32_t)time->sec;

    return seconds;
}

/* To caculate how long it will take to get to the alarm time from now */
static void update_alarm_delay(const app_time_t *cur_time, const uint32_t cur_seconds, app_alarm_node_t *alarm_node)
{
    uint32_t alarm_sec = 0;
    uint32_t mon_days[12] = APP_ALARM_DAYS_PER_MONTH;
    app_time_t time;

    mon_days[1] = IS_APP_ALARM_LEAP_YEAR((uint32_t)cur_time->year + APP_ALARM_FIRST_YEAR) ? 29 : 28;

    memcpy(&time, cur_time, sizeof(app_time_t));
    /* Set current hour, min and sec to alarm time */
    time.sec  = 0;
    time.min  = alarm_node->alarm.min;
    time.hour = alarm_node->alarm.hour;
    if (CALENDAR_ALARM_SEL_DATE == alarm_node->alarm.alarm_sel)
    {
        /* Type of alarm is date every month */
        time.date = alarm_node->alarm.alarm_date_week_mask;
        alarm_sec = time2seconds(&time);
        /* Calculate the latest alarm time in the future */
        if (alarm_sec <= cur_seconds)
        {
            if (++time.mon > 12)
            {
                time.mon = 1;
                time.year++;
            }
            else
            {
                if (time.date > mon_days[time.mon - 1])
                {
                    time.mon++;
                }
            }
            alarm_sec = time2seconds(&time);
        }
    }
    else
    {
        /* Type of alarm is day every week */
        /* Get the latest date */
        uint8_t mask = 1 << time.week;
        do {
            /* Check if the alarm time is over current time */
            if (mask & alarm_node->alarm.alarm_date_week_mask)
            {
                alarm_sec = time2seconds(&time);
                if (alarm_sec > cur_seconds)
                {
                    break;
                }
            }

            /* Get the next latest alarm time */
            for (uint8_t day = 1; day < 8; day++)
            {
                mask <<= 1;
                if (0x80 == mask)
                {
                    mask = CALENDAR_ALARM_WEEKDAY_SUN;
                }

                if (mask & alarm_node->alarm.alarm_date_week_mask)
                {
                    time.date += day;
                    break;
                }
            }
            if (time.date > mon_days[time.mon - 1])
            {
                time.date -= mon_days[time.mon - 1];
                if (++time.mon > 12)
                {
                    time.mon = 1;
                    time.year++;
                }
            }
            alarm_sec = time2seconds(&time);
        } while(0);
    }

    alarm_node->delay = alarm_sec - cur_seconds;
}

static void calendar_alarm_notify(app_alarm_node_t *alarm_node)
{
    app_time_t time;

    app_alarm_get_time(&time);

    if (time.hour == alarm_node->alarm.hour && time.min == alarm_node->alarm.min)
    {
        if (0 == alarm_node->alarm.hour && 0 == alarm_node->alarm.min)
        {
            if (CALENDAR_ALARM_SEL_WEEKDAY == alarm_node->alarm.alarm_sel)
            {
                if (alarm_node->alarm.alarm_date_week_mask & (1UL << time.week))
                {
                    s_app_alarm_info.callback(alarm_node->alarm_id);
                }
            }
            else
            {
                if (alarm_node->alarm.alarm_date_week_mask == time.date)
                {
                    s_app_alarm_info.callback(alarm_node->alarm_id);
                }
            }
        }
        else
        {
            s_app_alarm_info.callback(alarm_node->alarm_id);
        }
    }
}

static void calendar_alarm_cb(calendar_handle_t *hcalendar)
{
    app_alarm_node_t alarm_node;

    /* Check alarm available */
    if (0 < s_app_alarm_info.alarm_available)
    {
        /* Save the current alarm */
        memcpy(&alarm_node, &s_app_alarm_info.alarm_list[0], sizeof(app_alarm_node_t));
        /* Sort for setting next */
        sort_alarm_list();

        if (s_app_alarm_info.alarm0.delay <= s_app_alarm_info.alarm_list[0].delay)
        {
            hal_calendar_set_alarm(&s_app_alarm_info.handle, &s_app_alarm_info.alarm0.alarm);
        }
        else
        {
            hal_calendar_set_alarm(&s_app_alarm_info.handle, &s_app_alarm_info.alarm_list[0].alarm);
        }

        /* User callback */
        calendar_alarm_notify(&alarm_node);
    }
}

static int32_t partition(app_alarm_node_t *alarm_list, int32_t begin, int32_t end)
{
    uint32_t x = alarm_list[end].delay;
    int32_t  i = begin - 1;
    int32_t  j;
    app_alarm_node_t tmp;

    for (j = begin; j < end; j++)
    {
        if (alarm_list[j].delay <= x)
        {
            i++;
            if (j != i)
            {
                memcpy(&tmp, &alarm_list[i], sizeof(app_alarm_node_t));
                memcpy(&alarm_list[i], &alarm_list[j], sizeof(app_alarm_node_t));
                memcpy(&alarm_list[j], &tmp, sizeof(app_alarm_node_t));
            }
        }
    }

    if (++i != end)
    {
        memcpy(&tmp, &alarm_list[i], sizeof(app_alarm_node_t));
        memcpy(&alarm_list[i], &alarm_list[end], sizeof(app_alarm_node_t));
        memcpy(&alarm_list[end], &tmp, sizeof(app_alarm_node_t));
    }

    return i;
}

static void quick_sort(app_alarm_node_t *alarm_list, int32_t begin, int32_t end)
{
    int32_t m = 0;

    if ((NULL == alarm_list) || (begin >= end))
    {
        return;
    }

    m = partition(alarm_list, begin, end);
    quick_sort(alarm_list, begin, m - 1);
    quick_sort(alarm_list, m + 1, end);
}

static void sort_alarm_list(void)
{
    app_time_t cur_time;
    uint32_t cur_sec;

    if (0 < s_app_alarm_info.alarm_available)
    {
        hal_calendar_get_time(&s_app_alarm_info.handle, &cur_time);
        cur_sec = time2seconds(&cur_time);

        /* Update alarm seconds */
        for (uint8_t i = 0; i < s_app_alarm_info.alarm_available; i++)
        {
            update_alarm_delay(&cur_time, cur_sec, &s_app_alarm_info.alarm_list[i]);
        }

        /* sort the seconds */
        quick_sort(s_app_alarm_info.alarm_list, 0, s_app_alarm_info.alarm_available - 1);

        /* update the alarm0 seconds */
        update_alarm_delay(&cur_time, cur_sec, &s_app_alarm_info.alarm0);
    }
}

static uint8_t check_alarm_existed_alarm_id(app_alarm_id_t alarm_id)
{
    uint8_t indx = 0;

    for (; indx < s_app_alarm_info.alarm_available; indx++)
    {
        if (s_app_alarm_info.alarm_list[indx].alarm_id == alarm_id)
        {
            break;
        }
    }

    return indx;
}

static uint8_t check_alarm_existed_alarm(const app_alarm_t *alarm)
{
    uint8_t indx = 0;

    for (; indx < s_app_alarm_info.alarm_available; indx++)
    {
        if (!memcmp(&s_app_alarm_info.alarm_list[indx].alarm, alarm, sizeof(app_alarm_t)))
        {
            break;
        }
    }

    return indx;
}

static uint16_t save_alarm_list(void)
{
    uint8_t  error_code;
    app_alarm_node_t alarm_buf[MAX_ALARM_SUPPORT];

    if (!s_app_alarm_info.initialized)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    memset(alarm_buf, 0, sizeof(alarm_buf));
    for (uint8_t i = 0; i < s_app_alarm_info.alarm_available; i++)
    {
        memcpy(&alarm_buf[i], &s_app_alarm_info.alarm_list[i], sizeof(app_alarm_node_t));
    }

    error_code = nvds_put(s_app_alarm_info.alarm_data_tag, sizeof(alarm_buf), (const uint8_t *)alarm_buf);
    if (NVDS_SUCCESS != error_code)
    {
        return error_code;
    }

    return APP_DRV_SUCCESS;
}

/* Calendar HAL driver alarm callback */
void hal_calendar_alarm_callback(calendar_handle_t *hcalendar)
{
    calendar_alarm_cb(hcalendar);
}

/* Calendar HAL driver overflow callback */
void hal_calendar_overflow_callback(calendar_handle_t *hcalendar)
{
    calendar_alarm_cb(hcalendar);
}

void CALENDAR_IRQHandler(void)
{
    hal_calendar_irq_handler(&s_app_alarm_info.handle);
}

#endif

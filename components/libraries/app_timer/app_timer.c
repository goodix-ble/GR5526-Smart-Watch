/**
 *****************************************************************************************
 *
 * @file app_timer.c
 *
 * @brief app timer function Implementation.
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
#include "app_timer.h"
#include "grx_hal.h"
#include "gr_soc.h"
#if APP_TIMER_ASSERT_ENABLE
#include "app_assert.h"
#else
#define APP_ASSERT_CHECK(x)
#endif


/*
 * GLOBAL VERIABLE DECLARATION
 *****************************************************************************************
 */

/*
 * DEFINES
 *****************************************************************************************
 */
/*
  ------------------------------------------------------------------------------
  | PRIGROUP | BIT INFO  | GROUP PRI BITS | SUBPRI BITS | GROUP PRIS | SUBPRIS |
  ------------------------------------------------------------------------------
  |  0B011   | xxxx.yyyy |      [7:4]     |    [3:0]    |     16     |    16   |
  ------------------------------------------------------------------------------
  Note :
  App timer uses the basepri feature to implement the lock, in which the lock will
  not disable SVC_IRQ, BLE_IRQ, BLE_SLEEP_IRQ to ensure the highest priority of
  Bluetooth services
*/
#define _LOCAL_APP_TIMER_LOCK()                                  \
    uint32_t __l_irq_rest = __get_BASEPRI();                     \
    __set_BASEPRI(NVIC_GetPriority(BLE_IRQn) +                   \
                 (1 << (NVIC_GetPriorityGrouping() + 1)));

#define _LOCAL_APP_TIMER_UNLOCK()                                \
    __set_BASEPRI(__l_irq_rest);

#define APP_TIMER_LOCK()                _LOCAL_APP_TIMER_LOCK()
#define APP_TIMER_UNLOCK()              _LOCAL_APP_TIMER_UNLOCK()

/** @brief app timer clock and some time conversion formula definitions */
#define APP_TIMER_CLK_FREQ_HZ           (hal_sleep_timer_get_clock_freq())
#define APP_TIMER_MS_TO_US(x)           ((x) * 1000UL)
#define APP_TIMER_US_TO_TICKS(x)        (((double)(x) * APP_TIMER_CLK_FREQ_HZ / 1000000.0) + 0.5)
#define APP_TIMER_TICKS_TO_US(x)        ((uint64_t)(x) * 1000000.0f / APP_TIMER_CLK_FREQ_HZ)

/** @brief app timer status return  definitions */
#define APP_TIMER_RET_SUC               (1)
#define APP_TIMER_RET_FAIL              (0)

/** @brief app timer work parameter definitions */
#define APP_TIMER_INTERNAL_TIMEOUT      (1000)
#define APP_TIMER_DELAY_MS_MIN          (1)
#define APP_TIMER_DELAY_MS_MAX          (36 * 3600 * 1000)
#define APP_TIMER_DELAY_US_MAX          ((uint64_t)APP_TIMER_DELAY_MS_MAX * 1000)
#define APP_TIMER_OVERFLOW_VALUE        (0xFFFFFFFF)

#if APP_TIMER_TRIGGER_WINDOW_ENABLE
/** @brief app timer trigger window definitions */
#define APP_TIMER_TRIGGER_WINDOW_US     (0)
#define FALL_WITHIN_TRIGGER_WINDOW(x)   (((x) - s_app_timer_info.apptimer_total_us) <= s_app_timer_info.apptimer_trigger_window_us)
#define WITHIN_TRIGGER_WINDOW_MARK(x)   (x->timer_mark = true)
#define WITHIN_TRIGGER_WINDOW_CLEAR(x)  (x->timer_mark = false)
#define IS_TRIGGER_WINDOW_MARKED(x)     (x->timer_mark == true)
#endif

/** @brief App timer global state variable. */
typedef struct app_timer_struct
{
    uint32_t        cnt_node;
    app_timer_t    *p_curr_timer_node;
    uint64_t        apptimer_total_us;
    app_timer_t     hd_node;
#if APP_TIMER_TRIGGER_WINDOW_ENABLE
    app_timer_t    *p_within_window_one_shot_node_hd;
    uint64_t        apptimer_trigger_window_us;
#endif
}app_timer_info_t;

/** @brief App timer node state variable. */
typedef enum
{
    NOT_INIT = 0xFF,
    STOP     = 0x00,
    RUN      = 0x01,
}app_timer_node_statu_t;

/** @brief App timer global list, all newly added timer nodes will be added to the queue. */
static app_timer_info_t s_app_timer_info =
{
    .cnt_node                           = 0,
    .p_curr_timer_node                  = NULL,
    .apptimer_total_us                  = 0,
#if APP_TIMER_TRIGGER_WINDOW_ENABLE
    .p_within_window_one_shot_node_hd   = NULL,
    .apptimer_trigger_window_us         = APP_TIMER_TRIGGER_WINDOW_US,
#endif
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void            sleep_timer_handler_register(void);
static void            low_level_timer_startup(uint64_t value);
static void            low_level_timer_stop(void);
static uint64_t        low_level_timer_rest_get(void);
static uint8_t         app_timer_running_queue_insert(app_timer_id_t *p_timer_id);
static app_timer_id_t* app_timer_running_queue_remove(app_timer_id_t *p_timer_id);
#if APP_TIMER_TRIGGER_WINDOW_ENABLE
static uint8_t         app_timer_running_queue_trigger_window_mark(void);
static uint8_t         app_timer_running_queue_trigger_window_execute(void);
#endif
static void            app_timer_node_init(app_timer_id_t *p_timer_id, uint64_t delay, void *p_ctx, uint64_t insert_time);
static uint8_t         is_need_insert_front(uint64_t delay_value, uint64_t rest_time);
static uint8_t         is_timer_node_created(app_timer_id_t *p_timer_id);


/**
 ****************************************************************************************
 * @brief  sleep timer Interrupt Handler
 * @retval none
 ****************************************************************************************
 */
void SLPTIMER_IRQHandler(void)
{
    hal_pwr_sleep_timer_irq_handler();
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_pwr_sleep_timer_elapsed_callback(void)
{
    app_timer_t *p_curr_node = app_timer_running_queue_remove(NULL);

    APP_ASSERT_CHECK(p_curr_node);

    s_app_timer_info.apptimer_total_us = p_curr_node->next_shot_time;
    if (s_app_timer_info.apptimer_total_us >= APP_TIMER_DELAY_US_MAX)
    {
        app_timer_t *tmp = (app_timer_id_t *)&s_app_timer_info.hd_node;
        while (tmp->p_next)
        {
            if (tmp->p_next->next_shot_time >= s_app_timer_info.apptimer_total_us)
            {
               tmp->p_next->next_shot_time -= s_app_timer_info.apptimer_total_us;
            }
            else
            {
                /* should never come here */
                APP_ASSERT_CHECK(false);
            }
            tmp = tmp->p_next;
        }

        s_app_timer_info.apptimer_total_us = 0;
    }

    if (p_curr_node->timer_node_mode == ATIMER_REPEAT)
    {
        p_curr_node->next_shot_time = s_app_timer_info.apptimer_total_us + p_curr_node->original_delay;
        app_timer_running_queue_insert(p_curr_node);
    }

#if APP_TIMER_TRIGGER_WINDOW_ENABLE
    app_timer_running_queue_trigger_window_mark();
#endif

    if (s_app_timer_info.p_curr_timer_node)
    {
        APP_ASSERT_CHECK(s_app_timer_info.p_curr_timer_node->next_shot_time >= s_app_timer_info.apptimer_total_us);
        low_level_timer_startup(s_app_timer_info.p_curr_timer_node->next_shot_time - s_app_timer_info.apptimer_total_us);
    }

    if (p_curr_node->timer_node_cb)
        p_curr_node->timer_node_cb(p_curr_node->arg);
    else
        APP_ASSERT_CHECK(false);

#if APP_TIMER_TRIGGER_WINDOW_ENABLE
    app_timer_running_queue_trigger_window_execute();
#endif
}

sdk_err_t app_timer_start_api(app_timer_id_t *p_timer_id, uint32_t delay, void *p_ctx)
{
    uint8_t  trigger_flag = false;
    uint64_t insert_time  = 0;
    uint64_t last_rest_time;

    if ((NULL == p_timer_id) ||
        (delay > APP_TIMER_DELAY_MS_MAX) ||
        (delay < APP_TIMER_DELAY_MS_MIN))
        return SDK_ERR_INVALID_PARAM;

    APP_TIMER_LOCK();

    if ((!is_timer_node_created(p_timer_id)) ||
        (p_timer_id->timer_node_status != STOP))
    {
        APP_TIMER_UNLOCK();
        return SDK_ERR_DISALLOWED;
    }

    last_rest_time = low_level_timer_rest_get();

    if (is_need_insert_front(APP_TIMER_MS_TO_US(delay), last_rest_time))
    {
        if (s_app_timer_info.p_curr_timer_node)
        {
            APP_ASSERT_CHECK(last_rest_time <= (s_app_timer_info.p_curr_timer_node->next_shot_time - s_app_timer_info.apptimer_total_us));
            s_app_timer_info.apptimer_total_us = s_app_timer_info.p_curr_timer_node->next_shot_time - last_rest_time;
            low_level_timer_stop();
        }
        else
        {
            // the node that will be started is the first timer node
            s_app_timer_info.apptimer_total_us = 0;
            last_rest_time = 0;
        }
        trigger_flag = true;
        insert_time  = s_app_timer_info.apptimer_total_us;
    }
    else
    {
        APP_ASSERT_CHECK(last_rest_time <= (s_app_timer_info.p_curr_timer_node->next_shot_time - s_app_timer_info.apptimer_total_us));
        insert_time = s_app_timer_info.p_curr_timer_node->next_shot_time - last_rest_time;
    }

    app_timer_node_init(p_timer_id, delay, p_ctx, insert_time);
    app_timer_running_queue_insert(p_timer_id);

    sleep_timer_handler_register();
    if (!NVIC_GetEnableIRQ(SLPTIMER_IRQn))
    {
        NVIC_ClearPendingIRQ(SLPTIMER_IRQn);
        NVIC_EnableIRQ(SLPTIMER_IRQn);
    }

    if (trigger_flag)
    {
        APP_ASSERT_CHECK(s_app_timer_info.p_curr_timer_node->next_shot_time > s_app_timer_info.apptimer_total_us);
        low_level_timer_startup(s_app_timer_info.p_curr_timer_node->next_shot_time - s_app_timer_info.apptimer_total_us);
    }

    APP_TIMER_UNLOCK();

    return SDK_SUCCESS;
}

void app_timer_stop_api(app_timer_id_t *p_timer_id)
{
    uint8_t  timer_has_stop = false;
    uint64_t last_rest_time = 0;

    if (NULL == p_timer_id)
        return;

    APP_TIMER_LOCK();
    if (p_timer_id->timer_node_status == RUN)
    {
        if (s_app_timer_info.p_curr_timer_node == p_timer_id)
        {
            last_rest_time = low_level_timer_rest_get();
            APP_ASSERT_CHECK(last_rest_time <= (s_app_timer_info.p_curr_timer_node->next_shot_time - s_app_timer_info.apptimer_total_us));
            s_app_timer_info.apptimer_total_us = (p_timer_id->next_shot_time - last_rest_time);

            low_level_timer_stop();
            timer_has_stop = true;
        }

        app_timer_running_queue_remove(p_timer_id);

        if (timer_has_stop && s_app_timer_info.p_curr_timer_node)
        {
            APP_ASSERT_CHECK(s_app_timer_info.p_curr_timer_node->next_shot_time > s_app_timer_info.apptimer_total_us);
            low_level_timer_startup(s_app_timer_info.p_curr_timer_node->next_shot_time - s_app_timer_info.apptimer_total_us);
        }
        else if ((!timer_has_stop) && (NULL == s_app_timer_info.p_curr_timer_node))
        {
            low_level_timer_stop();
            APP_ASSERT_CHECK(false);
        }
    }

    APP_TIMER_UNLOCK();
}

sdk_err_t app_timer_delete(app_timer_id_t *p_timer_id)
{
    if (NULL == p_timer_id)
        return SDK_ERR_INVALID_PARAM;

    app_timer_stop_api(p_timer_id);
    p_timer_id->timer_node_status = NOT_INIT;
    p_timer_id->original_delay    = 0;
    p_timer_id->next_shot_time    = 0;
    p_timer_id->timer_node_cb     = NULL;

    return SDK_SUCCESS;
}

uint8_t app_timer_get_status(void)
{
    uint8_t status = (uint8_t)STOP;

    APP_TIMER_LOCK();

    if (s_app_timer_info.p_curr_timer_node)
        status = (uint8_t)RUN;

    APP_TIMER_UNLOCK();

    return status;
}

#if APP_TIMER_TRIGGER_WINDOW_ENABLE
void app_timer_trigger_window_set(uint64_t window_us)
{
    APP_TIMER_LOCK();
    s_app_timer_info.apptimer_trigger_window_us = window_us;
    APP_TIMER_UNLOCK();
}

uint64_t app_timer_trigger_window_get(void)
{
    uint64_t window_us = APP_TIMER_TRIGGER_WINDOW_US;

    APP_TIMER_LOCK();
    window_us = s_app_timer_info.apptimer_trigger_window_us;
    APP_TIMER_UNLOCK();

    return window_us;
}
#endif

sdk_err_t app_timer_create(app_timer_id_t *p_timer_id, app_timer_type_t mode, app_timer_fun_t callback)
{
    if (!IS_APP_TIMER_MODE(mode))
        return SDK_ERR_INVALID_PARAM;

    if ((NULL == p_timer_id) || (NULL == callback))
    {
        return SDK_ERR_INVALID_PARAM;
    }

    APP_TIMER_LOCK();

    p_timer_id->timer_node_status = STOP;
    p_timer_id->timer_node_mode   = mode;
    p_timer_id->timer_node_cb     = callback;

    APP_TIMER_UNLOCK();

    return SDK_SUCCESS;
}

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void sleep_timer_handler_register(void)
{
    static uint8_t handler_registered = false;

    if (!handler_registered)
    {
        soc_register_nvic(SLPTIMER_IRQn, (uint32_t)SLPTIMER_IRQHandler);
        handler_registered = true;
    }
}
static void low_level_timer_startup(uint64_t value)
{
    uint32_t timeout  = 0;
    uint32_t lpcycles =(uint32_t) APP_TIMER_US_TO_TICKS(value);

    APP_ASSERT_CHECK(value <= APP_TIMER_DELAY_US_MAX);

    hal_sleep_timer_config_and_start(PWR_SLP_TIMER_MODE_SINGLE, lpcycles);

    /* Wait for slp timer current value update */
    while (!hal_sleep_timer_status_get() || hal_sleep_timer_get_current_value() == APP_TIMER_OVERFLOW_VALUE)
    {
        delay_us(1);
        if (timeout++ >= APP_TIMER_INTERNAL_TIMEOUT)
        {
            APP_ASSERT_CHECK(false);
            return;
        }
    }
}

static void low_level_timer_stop(void)
{
    uint32_t timeout = 0;

    hal_sleep_timer_stop();
    while (hal_sleep_timer_status_get())
    {
        delay_us(1);
        if (timeout++ >= APP_TIMER_INTERNAL_TIMEOUT)
        {
            APP_ASSERT_CHECK(false);
            return;
        }
    }
}

static uint64_t low_level_timer_rest_get(void)
{
    uint32_t atimer_curr_ticks = hal_sleep_timer_get_current_value();
    uint64_t curr_rest_us = 0;

    if (atimer_curr_ticks == APP_TIMER_OVERFLOW_VALUE)
    {
        return 0;
    }

    curr_rest_us =(uint64_t) APP_TIMER_TICKS_TO_US(atimer_curr_ticks);
    if (s_app_timer_info.p_curr_timer_node)
    {
        if (curr_rest_us > (s_app_timer_info.p_curr_timer_node->next_shot_time - s_app_timer_info.apptimer_total_us))
            curr_rest_us = s_app_timer_info.p_curr_timer_node->next_shot_time - s_app_timer_info.apptimer_total_us;
    }
    else
    {
        curr_rest_us = 0;
    }

    return curr_rest_us;
}

static uint8_t app_timer_running_queue_insert(app_timer_t *p_timer_id)
{
    if (p_timer_id == NULL)
    {
        return false;
    }

    APP_TIMER_LOCK();

    app_timer_t *tmp = (app_timer_id_t *)&s_app_timer_info.hd_node;
    p_timer_id->p_next = NULL;

    while (tmp->p_next)
    {
        app_timer_t *curr_node = tmp->p_next;
        if (curr_node->next_shot_time > p_timer_id->next_shot_time)
        {
            p_timer_id->p_next = tmp->p_next;
            break;
        }
        tmp = tmp->p_next;
    }

    tmp->p_next = p_timer_id;
    p_timer_id->timer_node_status = RUN;

    s_app_timer_info.cnt_node++;
    s_app_timer_info.p_curr_timer_node = s_app_timer_info.hd_node.p_next;

    APP_TIMER_UNLOCK();
    return true;
}

#if APP_TIMER_TRIGGER_WINDOW_ENABLE
static uint8_t app_timer_running_queue_trigger_window_mark(void)
{
    // todo:
    // 1, pick timer nodes that fall within the trigger window;
    // 2, mark these timers;
    //    1), for repeat node, need mark and adjust next_shot_timer;
    //    2), for one-shot node, chain up these timers in specify list

    APP_TIMER_LOCK();

    app_timer_t *p_triggered_node = NULL;
    app_timer_t *tmp = &s_app_timer_info.hd_node;

    while (tmp->p_next)
    {
        if (FALL_WITHIN_TRIGGER_WINDOW(tmp->p_next->next_shot_time))
        {
            p_triggered_node = tmp->p_next;
            WITHIN_TRIGGER_WINDOW_MARK(p_triggered_node);

            app_timer_running_queue_remove(p_triggered_node);
            if (p_triggered_node->timer_node_mode == ATIMER_REPEAT)
            {
                p_triggered_node->next_shot_time = s_app_timer_info.apptimer_total_us + p_triggered_node->original_delay;
                app_timer_running_queue_insert(p_triggered_node);
            }
            else
            {
                app_timer_t **pp_trigger_window_one_shot_node_tail = &s_app_timer_info.p_within_window_one_shot_node_hd;
                while (*pp_trigger_window_one_shot_node_tail)
                {
                    pp_trigger_window_one_shot_node_tail = &(*pp_trigger_window_one_shot_node_tail)->p_next;
                }
                p_triggered_node->p_next = NULL;
                (*pp_trigger_window_one_shot_node_tail)->p_next = p_triggered_node;
            }
        }
        else
        {
            break;
        }
    }

    APP_TIMER_UNLOCK();
    return true;
}

static uint8_t app_timer_running_queue_trigger_window_execute(void)
{
    // todo:
    // 1, check timer node that marked in the app_timer_running_queue_trigger_window_mark();
    // 2, execute the callback;
    // 3, clear mark;

    APP_TIMER_LOCK();

    app_timer_t *p_triggered_node = NULL;
    app_timer_t *tmp = &s_app_timer_info.hd_node;

    while (tmp->p_next)
    {
        if (IS_TRIGGER_WINDOW_MARKED(tmp->p_next))
        {
            p_triggered_node = tmp->p_next;

            if (p_triggered_node->timer_node_cb)
                p_triggered_node->timer_node_cb(p_triggered_node->arg);
            else
                APP_ASSERT_CHECK(false);

            WITHIN_TRIGGER_WINDOW_CLEAR(p_triggered_node);
        }

        tmp = tmp->p_next;
    }

    while (s_app_timer_info.p_within_window_one_shot_node_hd)
    {
        p_triggered_node = s_app_timer_info.p_within_window_one_shot_node_hd;
        if (p_triggered_node->timer_node_cb)
            p_triggered_node->timer_node_cb(p_triggered_node->arg);
        else
            APP_ASSERT_CHECK(false);

        WITHIN_TRIGGER_WINDOW_CLEAR(p_triggered_node);

        s_app_timer_info.p_within_window_one_shot_node_hd = p_triggered_node->p_next;
        p_triggered_node->p_next = NULL;
    }

    APP_TIMER_UNLOCK();
    return true;
}
#endif

static app_timer_t *app_timer_running_queue_remove(app_timer_id_t *p_timer_id)
{
    app_timer_t *remove_node = NULL;

    APP_TIMER_LOCK();

    if (NULL != s_app_timer_info.hd_node.p_next)
    {
        if (NULL == p_timer_id)
        {
            // the node that will be removed is the first node
            remove_node = s_app_timer_info.hd_node.p_next;

            s_app_timer_info.hd_node.p_next = remove_node->p_next;
            s_app_timer_info.cnt_node--;
        }
        else
        {
            // remove specified node
            app_timer_t *tmp = (app_timer_id_t *)&s_app_timer_info.hd_node;
            while (tmp->p_next)
            {
                if (tmp->p_next == p_timer_id)
                {
                    s_app_timer_info.cnt_node--;
                    tmp->p_next = p_timer_id->p_next;

                    remove_node = p_timer_id;
                    break;
                }
                tmp = tmp->p_next;
            }
        }

        if (remove_node)
        {
            remove_node->p_next = NULL;
            remove_node->timer_node_status = STOP;
            s_app_timer_info.p_curr_timer_node = s_app_timer_info.hd_node.p_next;
        }
    }

    APP_TIMER_UNLOCK();
    return remove_node;
}

static void app_timer_node_init(app_timer_id_t *p_timer_id, uint64_t delay, void *p_ctx, uint64_t insert_time)
{
    p_timer_id->arg            = p_ctx;
    p_timer_id->original_delay = APP_TIMER_MS_TO_US(delay);
    p_timer_id->next_shot_time = insert_time + p_timer_id->original_delay;
    p_timer_id->p_next         = NULL;
}

static uint8_t is_need_insert_front(uint64_t delay_value, uint64_t rest_time)
{
    APP_TIMER_LOCK();
    if (s_app_timer_info.p_curr_timer_node == NULL)
    {
        APP_TIMER_UNLOCK();
        return APP_TIMER_RET_SUC;
    }

    APP_ASSERT_CHECK(rest_time <= s_app_timer_info.p_curr_timer_node->original_delay);
    {
        if (delay_value < rest_time)
        {
            APP_TIMER_UNLOCK();
            return APP_TIMER_RET_SUC;
        }
    }
    APP_TIMER_UNLOCK();
    return APP_TIMER_RET_FAIL;
}

static uint8_t is_timer_node_created(app_timer_id_t *p_timer_id)
{
    if (p_timer_id)
    {
        if ((p_timer_id->timer_node_cb) &&
            (IS_APP_TIMER_MODE(p_timer_id->timer_node_mode)))
            return true;
    }

    return false;
}

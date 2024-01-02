/**
 *****************************************************************************************
 *
 * @file app_scheduler.c
 *
 * @brief App Scheduler Implementation.
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
#include "app_scheduler.h"
#include "grx_hal.h"
#include "app_memory.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_SCHEDULER_LOCK()                  LOCAL_INT_DISABLE(BLE_IRQn)
#define APP_SCHEDULER_UNLOCK()                LOCAL_INT_RESTORE()

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief App scheduler environment variable. */
struct app_scheduler_env_t
{
    app_scheduler_evt_info_t  *p_evt_info_buffer;
    uint16_t                   evt_queue_size;
    volatile uint8_t           queue_start_index;
    volatile uint8_t           queue_end_index;
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct app_scheduler_env_t  s_app_scheduler_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static __INLINE uint8_t next_index_get(uint8_t index)
{
    return (index < s_app_scheduler_env.evt_queue_size) ? (index + 1) : 0;
}

static __INLINE bool is_evt_queue_full()
{
  return next_index_get(s_app_scheduler_env.queue_end_index) == s_app_scheduler_env.queue_start_index;
}

static __INLINE bool is_evt_queue_empty()
{
  return s_app_scheduler_env.queue_end_index == s_app_scheduler_env.queue_start_index;
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t app_scheduler_init(uint16_t queue_size)
{
    if (!queue_size)
    {
        return SDK_ERR_INVALID_PARAM;
    }

    s_app_scheduler_env.p_evt_info_buffer = app_malloc((queue_size + 1) * sizeof(app_scheduler_evt_info_t));

    if (NULL == s_app_scheduler_env.p_evt_info_buffer)
    {
        return SDK_ERR_NO_RESOURCES;
    }

    memset(s_app_scheduler_env.p_evt_info_buffer, 0, (queue_size + 1) * sizeof(app_scheduler_evt_info_t));

    s_app_scheduler_env.evt_queue_size    = queue_size;
    s_app_scheduler_env.queue_start_index = 0;
    s_app_scheduler_env.queue_end_index   = 0;

    return SDK_SUCCESS;
}

sdk_err_t app_scheduler_evt_put(void const *p_evt_data, uint16_t evt_data_size, app_scheduler_evt_handler_t evt_handler)
{
    sdk_err_t  error_code;
    void      *evt_data_ptr;

    APP_SCHEDULER_LOCK();

    if (!is_evt_queue_full())
    {
        s_app_scheduler_env.p_evt_info_buffer[s_app_scheduler_env.queue_end_index].evt_handler   = evt_handler;
        s_app_scheduler_env.p_evt_info_buffer[s_app_scheduler_env.queue_end_index].evt_data_size = evt_data_size;
        if (p_evt_data && evt_data_size)
        {
            evt_data_ptr = app_malloc(evt_data_size);
            if (NULL == evt_data_ptr)
            {
                return SDK_ERR_NO_RESOURCES;
            }

            memcpy(evt_data_ptr, p_evt_data, evt_data_size);
            s_app_scheduler_env.p_evt_info_buffer[s_app_scheduler_env.queue_end_index].p_evt_data = evt_data_ptr;
        }
        s_app_scheduler_env.queue_end_index = next_index_get(s_app_scheduler_env.queue_end_index);
        error_code = SDK_SUCCESS;
    }
    else
    {
        error_code = SDK_ERR_NO_RESOURCES;
    }

    APP_SCHEDULER_UNLOCK();

    return error_code;
}

void app_scheduler_execute(void)
{
    while(!is_evt_queue_empty())
    {
        void                       *p_evt_data;
        uint16_t                    evt_data_size;
        app_scheduler_evt_handler_t evt_handler;
        uint8_t                     evt_index;

        evt_index = s_app_scheduler_env.queue_start_index;

        p_evt_data    = s_app_scheduler_env.p_evt_info_buffer[evt_index].p_evt_data;
        evt_data_size = s_app_scheduler_env.p_evt_info_buffer[evt_index].evt_data_size;
        evt_handler   = s_app_scheduler_env.p_evt_info_buffer[evt_index].evt_handler;

        if (evt_handler)
        {
            evt_handler(p_evt_data, evt_data_size);
        }

        app_free(p_evt_data);

        s_app_scheduler_env.queue_start_index = next_index_get(s_app_scheduler_env.queue_start_index);
    }
}

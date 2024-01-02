/**
 *****************************************************************************************
 *
 * @file pmu_calibration.c
 *
 * @brief auto calibration function Implementation.
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
#include "pmu_calibration.h"
#include "platform_sdk.h"
#include "app_timer.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_timer_id_t s_pmu_calibration_timer_id;
static uint32_t g_pmu_interval_ms = 30 * 1000;
static uint32_t g_pmu_initial_interval_ms = 1000;
static uint32_t g_pmu_timer_cb_count = 0;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void pmu_timer_handler(void* p_arg)
{
    pmu_calibration_handler(p_arg);
    ++g_pmu_timer_cb_count;
    if (g_pmu_timer_cb_count > 1)
    {
        return;
    }

    app_timer_delete(&s_pmu_calibration_timer_id);
    app_timer_create(&s_pmu_calibration_timer_id, ATIMER_REPEAT, pmu_timer_handler);
    app_timer_start(s_pmu_calibration_timer_id, g_pmu_interval_ms, NULL);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void system_pmu_calibration_init(uint32_t interval)
{
    if (interval)
    {
        g_pmu_interval_ms = interval;

        app_timer_delete(&s_pmu_calibration_timer_id);
        app_timer_create(&s_pmu_calibration_timer_id, ATIMER_REPEAT, pmu_timer_handler);
        app_timer_start(s_pmu_calibration_timer_id, g_pmu_initial_interval_ms, NULL);
    }
    return;
}

void system_pmu_calibration_stop(void)
{
    app_timer_delete(&s_pmu_calibration_timer_id);
}

#ifndef __OSAL_INTERNAL_TIMER_H__
#define __OSAL_INTERNAL_TIMER_H__

#include "osal_timer.h"
#include "osal_internal_globaldefs.h"
#include "os_freertos.h"

typedef struct
{
    osal_timer_cb_function_t func;
    void *arg;
    osal_timer_handle_t timer_handle;
} osal_timer_t;

typedef struct
{
    char timer_name[configMAX_TASK_NAME_LEN];
    osal_tick_type_t timer_period; // unit:ticks
    uint8_t auto_reload;
    osal_timer_t timer_id;
} osal_timer_internal_record_t;

int32_t os_timer_create_impl(osal_timer_handle_t *timer_handle, osal_timer_internal_record_t *timer_record);

int32_t os_timer_start_impl(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait);

int32_t os_timer_stop_impl(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait);

int32_t os_timer_period_change_impl(osal_timer_handle_t timer_handle, osal_tick_type_t new_period, osal_tick_type_t ticks_to_wait);

int32_t os_timer_delete_impl(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait);

int32_t os_timer_reset_impl(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait);

osal_tick_type_t os_timer_period_get_impl(osal_timer_handle_t timer_handle);

#endif // __OSAL_INTERNAL_TIMER_H__

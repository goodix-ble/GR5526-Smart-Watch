#include "osal_internal_timer.h"
#include "osal_internal_globaldefs.h"
#include "osal_internal_heap.h"

#include "app_log.h"

int32_t osal_timer_create(osal_timer_handle_t *p_timer_handle, const char *timer_name, osal_tick_type_t timer_period, uint8_t auto_reload, osal_timer_cb_function_t timer_cb, void *arg)
{
    int32_t ret;
    osal_timer_internal_record_t *p_timer_record;
    p_timer_record = os_heap_malloc_impl(sizeof(osal_timer_internal_record_t));

    memcpy(p_timer_record->timer_name, timer_name, strlen(timer_name) + 1);
    p_timer_record->timer_period = timer_period;
    p_timer_record->auto_reload = auto_reload;
    p_timer_record->timer_id.func = timer_cb;
    p_timer_record->timer_id.arg = arg;
    ret = os_timer_create_impl(p_timer_handle, p_timer_record);
    return ret;
}

int32_t osal_timer_start(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait)
{
    int32_t ret;
    ret = os_timer_start_impl(timer_handle, ticks_to_wait);
    return ret;
}

int32_t osal_timer_stop(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait)
{
    int32_t ret;
    ret = os_timer_stop_impl(timer_handle, ticks_to_wait);
    return ret;
}

int32_t osal_timer_period_change(osal_timer_handle_t timer_handle, osal_tick_type_t new_period, osal_tick_type_t ticks_to_wait)
{
    int32_t ret;
    ret = os_timer_period_change_impl(timer_handle, new_period, ticks_to_wait);
    return ret;
}

int32_t osal_timer_delete(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait)
{
    int32_t ret;
    ret = os_timer_delete_impl(timer_handle, ticks_to_wait);
    return ret;
}

int32_t osal_timer_reset(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait)
{
    int32_t ret;
    ret = os_timer_reset_impl(timer_handle, ticks_to_wait);
    return ret;
}

osal_tick_type_t osal_timer_period_get(osal_timer_handle_t timer_handle)
{
    return os_timer_period_get_impl(timer_handle);
}

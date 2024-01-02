#ifndef __OSAL_TIMER_H__
#define __OSAL_TIMER_H__

#include "common_types.h"

typedef void (*osal_timer_cb_function_t)(osal_timer_handle_t timer_handle, void *);

int32_t osal_timer_create(osal_timer_handle_t *timer_handle, const char *timer_name, osal_tick_type_t timer_period, uint8_t auto_reload, osal_timer_cb_function_t timer_cb, void *arg);

int32_t osal_timer_start(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait);

int32_t osal_timer_stop(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait);

int32_t osal_timer_period_change(osal_timer_handle_t timer_handle, osal_tick_type_t new_period, osal_tick_type_t ticks_to_wait);

int32_t osal_timer_delete(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait);

int32_t osal_timer_reset(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait);

osal_tick_type_t osal_timer_period_get(osal_timer_handle_t timer_handle);

#endif // __OSAL_TIMER_H__

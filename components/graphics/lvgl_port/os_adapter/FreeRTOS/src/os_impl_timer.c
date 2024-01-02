#include "osal_internal_timer.h"
#include "osal_internal_globaldefs.h"
#include "os_freertos.h"
#include "app_log.h"

#if (OSAL_RTOS_SUPPORT == FREERTOS_SUPPORT)

static void os_timer_cb(TimerHandle_t xTimer)
{
    osal_timer_t *timer = pvTimerGetTimerID(xTimer);
    timer->func(timer->timer_handle, timer->arg);
}

int32_t os_timer_create_impl(osal_timer_handle_t *p_timer_handle, osal_timer_internal_record_t *timer_record)
{
    int32_t ret = OSAL_SUCCESS;
    TimerHandle_t cur_timer_handle = xTimerCreate(timer_record->timer_name, timer_record->timer_period, timer_record->auto_reload, &timer_record->timer_id, os_timer_cb);
    *p_timer_handle = (osal_timer_handle_t)cur_timer_handle;
    timer_record->timer_id.timer_handle = *p_timer_handle;
    if (NULL == *p_timer_handle)
    {
        ret = OSAL_INVALID_POINTER;
    }
    return ret;
}

int32_t os_timer_start_impl(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait)
{
    int32_t ret = OSAL_SUCCESS;
    BaseType_t status;

    if (OSAL_IS_IN_ISR())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        status = xTimerStartFromISR((TimerHandle_t)timer_handle, &xHigherPriorityTaskWoken);
        if (pdFALSE != xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        status = xTimerStart((TimerHandle_t)timer_handle, OS_MS_TO_TICKS(ticks_to_wait));
    }
    if (pdFAIL == status)
    {
        ret = OSAL_ERROR;
    }
    return ret;
}

int32_t os_timer_stop_impl(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait)
{
    int32_t ret = OSAL_SUCCESS;
    BaseType_t status;

    if (OSAL_IS_IN_ISR())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        status = xTimerStopFromISR((TimerHandle_t)timer_handle, &xHigherPriorityTaskWoken);
        if (pdFALSE != xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        status = xTimerStop((TimerHandle_t)timer_handle, OS_MS_TO_TICKS(ticks_to_wait));
    }
    if (pdFAIL == status)
    {
        ret = OSAL_ERROR;
    }
    return ret;
}

int32_t os_timer_period_change_impl(osal_timer_handle_t timer_handle, osal_tick_type_t new_period, osal_tick_type_t ticks_to_wait)
{
    int32_t ret = OSAL_SUCCESS;
    BaseType_t status;

    if (OSAL_IS_IN_ISR())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        status = xTimerChangePeriodFromISR((TimerHandle_t)timer_handle, new_period, &xHigherPriorityTaskWoken);
        if (pdFALSE != xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        status = xTimerChangePeriod((TimerHandle_t)timer_handle, OS_MS_TO_TICKS(new_period), OS_MS_TO_TICKS(ticks_to_wait));
    }
    if (pdFAIL == status)
    {
        ret = OSAL_ERROR;
    }
    return ret;
}

int32_t os_timer_delete_impl(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait)
{
    int32_t ret = OSAL_SUCCESS;
    BaseType_t status = xTimerDelete((TimerHandle_t)timer_handle, OS_MS_TO_TICKS(ticks_to_wait));
    if (pdFAIL == status)
    {
        ret = OSAL_ERROR;
    }
    return ret;
}

int32_t os_timer_reset_impl(osal_timer_handle_t timer_handle, osal_tick_type_t ticks_to_wait)
{
    int32_t ret = OSAL_SUCCESS;
    BaseType_t status;

    if (OSAL_IS_IN_ISR())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        status = xTimerResetFromISR((TimerHandle_t)timer_handle, &xHigherPriorityTaskWoken);
        if (pdFALSE != xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        status = xTimerReset((TimerHandle_t)timer_handle, OS_MS_TO_TICKS(ticks_to_wait));
    }
    if (pdFAIL == status)
    {
        ret = OSAL_ERROR;
    }
    return ret;
}

osal_tick_type_t os_timer_period_get_impl(osal_timer_handle_t timer_handle)
{
    return xTimerGetPeriod((TimerHandle_t)timer_handle);
}

#endif // OSAL_RTOS_SUPPORT

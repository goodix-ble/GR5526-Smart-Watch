#include "osal_internal_task.h"
#include "osal_internal_globaldefs.h"
#include "osal_internal_heap.h"

#include "app_log.h"


int32_t osal_task_create(const char *task_name, osal_task_entry func_pointer, size_t stack_size,
                         osal_priority_t priority, osal_task_handle_t task_handle)
{
    int32_t ret;
    osal_task_internal_record_t task;

    OSAL_CHECK_POINTER(func_pointer);
    OSAL_CHECK_SIZE(stack_size);

    memset(&task, 0, sizeof(osal_task_internal_record_t));
    strncpy(task.task_name, task_name, strlen(task_name));
    task.task_handle = task_handle;
    task.stack_size = stack_size;
    task.priority = priority;
    task.entry_function_pointer = func_pointer;

    ret = os_task_create_impl(&task);
    return ret;
}

void osal_task_delete(osal_task_handle_t osal_task_handle)
{
    os_task_delete_impl(osal_task_handle);
}

void osal_task_start(void)
{
    os_task_start_impl();
}

void osal_task_suspend(osal_task_handle_t osal_task_handle)
{
    os_task_suspend_impl(osal_task_handle);
}

void osal_task_suspend_all(void)
{
    os_task_suspend_all_impl();
}

void osal_task_resume(osal_task_handle_t osal_task_handle)
{
    os_task_resume_impl(osal_task_handle);
}

void osal_task_delay(int32_t ticks)
{
    os_task_delay_impl(ticks);
}

void osal_task_delay_ms(uint32_t ms)
{
    os_task_delay_ms_impl(ms);
}

void osal_enter_critical(void)
{
    os_enter_critical_impl();
}

void osal_exit_critical(void)
{
    os_exit_critical_impl();
}

int32_t osal_port_yield(void)
{
    int32_t ret = os_port_yield_impl();
    return ret;
}

void osal_task_enable_interrupts(void)
{
    os_task_enable_interrupts_impl();
}

void osal_task_disable_interrupts(void)
{
    os_task_disable_interrupts_impl();
}

osal_tick_type_t osal_task_get_tick_count(void)
{
    osal_tick_type_t osal_ticks = os_task_get_tick_count_impl();
    return osal_ticks;
}

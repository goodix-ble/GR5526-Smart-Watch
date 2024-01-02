#ifndef __OSAL_INTERNAL_TASK_H__
#define __OSAL_INTERNAL_TASK_H__

#include "osal_task.h"
#include "osal_internal_globaldefs.h"
#include "os_freertos.h"

typedef struct
{
    char task_name[configMAX_TASK_NAME_LEN];
    size_t stack_size;
    osal_priority_t priority;
    osal_task_entry entry_function_pointer;
    osal_task_entry delete_hook_pointer;
    void *entry_arg;
    osal_stackptr_t stack_pointer;
    osal_task_handle_t task_handle;
} osal_task_internal_record_t;

int32_t os_task_create_impl(osal_task_internal_record_t *p_task);

void os_task_delete_impl(osal_task_handle_t task_handle);

void os_task_start_impl(void);

void os_task_suspend_impl(osal_task_handle_t task_handle);

void os_task_resume_impl(osal_task_handle_t task_handle);

void os_task_suspend_all_impl(void);

void os_task_delay_impl(uint32_t ticks);

void os_task_delay_ms_impl(uint32_t ms);

void os_enter_critical_impl(void);

void os_exit_critical_impl(void);

void os_task_disable_interrupts_impl(void);

void os_task_enable_interrupts_impl(void);

int32_t os_port_yield_impl(void);

osal_tick_type_t os_task_get_tick_count_impl(void);


#endif // __OSAL_INTERNAL_TASK_H__

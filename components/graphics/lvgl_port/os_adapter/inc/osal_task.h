#ifndef __OSAL_TASK_H__
#define __OSAL_TASK_H__


#include "common_types.h"
#include "osal_config.h"

/**
 * @brief Type to be used for OSAL task priorities.
 *
 */
typedef uint32_t osal_priority_t;

/**
 * @brief Type to be used for OSAL stack pointer.
 *
 */
typedef void *osal_stackptr_t;

// typedef void oasl_task;
typedef void (*osal_task_entry)(void *);

/**
 * @brief Create a task ans starts running it.
 * Create a task and passes back the task id.
 */
int32_t osal_task_create(const char *task_name, osal_task_entry func_pointer, size_t stack_size,
                         osal_priority_t priority, osal_task_handle_t task_handle);

/**
 * @brief Delete the current task.
 *
 */
void osal_task_delete(osal_task_handle_t osal_task_handle);

void osal_task_start(void);

void osal_task_suspend(osal_task_handle_t osal_task_handle);

void osal_task_resume(osal_task_handle_t osal_task_handle);

void osal_task_suspend_all(void);

void osal_task_delay(int32_t ticks);

void osal_task_delay_ms(uint32_t ms);

void osal_enter_critical(void);

void osal_exit_critical(void);

int32_t osal_port_yield(void);

void osal_task_enable_interrupts(void);

void osal_task_disable_interrupts(void);

osal_tick_type_t osal_task_get_tick_count(void);


#endif // __OSAL_TASK_H__

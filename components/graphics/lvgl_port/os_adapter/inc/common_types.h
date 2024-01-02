#ifndef __COMMON_TYPES_H__
#define __COMMON_TYPES_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#if (OSAL_USE_16_BIT_TICKS == 1)
#define OSAL_MAX_DELAY (0xFFFF)
typedef uint16_t osal_tick_type_t;
#else
#define OSAL_MAX_DELAY (0xFFFFFFFF)
typedef uint32_t osal_tick_type_t;
#endif


typedef long osal_base_type_t;
typedef void * osal_task_handle_t;

typedef void * osal_sema_handle_t;
typedef void * osal_mutex_handle_t;
typedef void * osal_queue_handle_t;
typedef void * osal_timer_handle_t;

#define OSAL_TRUE  ( (osal_base_type_t) 1)
#define OSAL_FALSE ( (osal_base_type_t) 0)


#endif // __COMMON_TYPES_H__

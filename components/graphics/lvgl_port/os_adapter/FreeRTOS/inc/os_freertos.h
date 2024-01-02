#ifndef __OS_FREERTOS_H__
#define __OS_FREERTOS_H__

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#define OS_MS_TO_TICKS(osal_time_in_ms) \
    ((osal_time_in_ms == portMAX_DELAY)? (portMAX_DELAY): ((osal_tick_type_t)(((osal_tick_type_t)(osal_time_in_ms) * (osal_tick_type_t)configTICK_RATE_HZ) / (osal_tick_type_t)1000U)))


#endif // __OS_FREERTOS_H__

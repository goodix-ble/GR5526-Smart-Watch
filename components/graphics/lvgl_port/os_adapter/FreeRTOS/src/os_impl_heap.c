#include "osal_internal_heap.h"
#include "os_freertos.h"

#if (OSAL_RTOS_SUPPORT == FREERTOS_SUPPORT)

void *os_heap_malloc_impl(size_t wanted_size)
{
    void *ptr = pvPortMalloc(wanted_size);
    return ptr;
}

void os_heap_free_impl(void *ptr)
{
    vPortFree(ptr);
}

#endif // OSAL_RTOS_SUPPORT

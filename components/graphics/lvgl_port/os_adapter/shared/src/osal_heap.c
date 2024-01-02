#include "osal_internal_heap.h"
#include "osal_internal_globaldefs.h"
#include "app_log.h"



void *osal_heap_malloc(size_t wanted_size)
{
    void *ptr = os_heap_malloc_impl(wanted_size);
    return ptr;
}

void osal_heap_free(void *ptr)
{
    os_heap_free_impl(ptr);
}

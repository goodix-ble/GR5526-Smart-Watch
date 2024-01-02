#include "osal_internal_sema.h"
#include "osal_internal_globaldefs.h"
#include "osal_internal_heap.h"

#include "app_log.h"

int32_t osal_sema_countings_create(osal_sema_handle_t *p_sema_handle, uint32_t max_count, uint32_t init_count)
{
    int32_t ret;
    ret = os_sema_countings_create_impl(p_sema_handle, max_count, init_count);
    return ret;
}

int32_t osal_sema_binary_create(osal_sema_handle_t *p_sema_handle)
{
    int32_t ret;
    ret = os_sema_binary_create_impl(p_sema_handle);
    return ret;
}

void osal_sema_delete(osal_sema_handle_t sema_handle)
{
    os_sema_delete_impl(sema_handle);
}

int32_t osal_sema_give(osal_sema_handle_t sema_handle)
{
    int32_t ret;
    ret = os_sema_give_impl(sema_handle);
    return ret;
}

int32_t osal_sema_take(osal_sema_handle_t sema_handle, osal_tick_type_t timeout)
{
    int32_t ret;
    ret = os_sema_take_impl(sema_handle, timeout);
    return ret;
}

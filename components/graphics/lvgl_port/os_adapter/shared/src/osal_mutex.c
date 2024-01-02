#include "osal_internal_mutex.h"
#include "osal_internal_globaldefs.h"
#include "app_log.h"


int32_t osal_mutex_create(osal_mutex_handle_t *p_mutex_handle)
{
    int32_t ret;
    ret = os_mutex_create_impl(p_mutex_handle);
    return ret;
}

void osal_mutex_delete(osal_mutex_handle_t mutex_handle)
{
    os_mutex_delete_impl(mutex_handle);
}

int32_t osal_mutex_give(osal_mutex_handle_t mutex_handle)
{
    int32_t ret;
    ret = os_mutex_give_impl(mutex_handle);
    return ret;
}

int32_t osal_mutex_take(osal_mutex_handle_t mutex_handle, osal_tick_type_t timeout)
{
    int32_t ret;
    ret = os_mutex_take_impl(mutex_handle, timeout);
    return ret;
}


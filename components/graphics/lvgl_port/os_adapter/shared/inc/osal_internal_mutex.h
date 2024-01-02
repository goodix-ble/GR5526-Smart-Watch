#ifndef __OSAL_INTERNAL_MUTEX_H__
#define __OSAL_INTERNAL_MUTEX_H__

#include "osal_mutex.h"
#include "osal_internal_globaldefs.h"


int32_t os_mutex_create_impl(osal_mutex_handle_t *p_mutex_handle);

void os_mutex_delete_impl(osal_mutex_handle_t mutex_handle);

int32_t os_mutex_give_impl(osal_mutex_handle_t mutex_handle);

int32_t os_mutex_take_impl(osal_mutex_handle_t mutex_handle, osal_tick_type_t timeout);


#endif // __OSAL_INTERNAL_MUTEX_H__

#ifndef __OSAL_INTERNAL_SEMA_H__
#define __OSAL_INTERNAL_SEMA_H__

#include "osal_sema.h"
#include "osal_internal_globaldefs.h"


int32_t os_sema_countings_create_impl(osal_sema_handle_t *p_sema_handle, uint32_t max_count, uint32_t init_count);

int32_t os_sema_binary_create_impl(osal_sema_handle_t *p_sema_handle);

void os_sema_delete_impl(osal_sema_handle_t sema_handle);

int32_t os_sema_give_impl(osal_sema_handle_t sema_handle);

int32_t os_sema_take_impl(osal_sema_handle_t sema_handle, osal_tick_type_t timeout);




#endif // __OSAL_INTERNAL_SEMA_H__

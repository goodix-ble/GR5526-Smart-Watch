#ifndef __OSAL_SEMA_H__
#define __OSAL_SEMA_H__

#include "common_types.h"

int32_t osal_sema_countings_create(osal_sema_handle_t *p_sema_handle, uint32_t max_count, uint32_t init_count);

int32_t osal_sema_binary_create(osal_sema_handle_t *p_sema_handle);

void osal_sema_delete(osal_sema_handle_t sema_handle);

int32_t osal_sema_give(osal_sema_handle_t sema_handle);

int32_t osal_sema_take(osal_sema_handle_t sema_handle, osal_tick_type_t timeout);



#endif // __OSAL_SEMA_H__

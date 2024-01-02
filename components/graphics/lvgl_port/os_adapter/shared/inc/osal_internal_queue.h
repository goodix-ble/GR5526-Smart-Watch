#ifndef __OSAL_INTERNAL_QUEUE_H__
#define __OSAL_INTERNAL_QUEUE_H__

#include "osal_task.h"
#include "osal_internal_globaldefs.h"

int32_t os_queue_create_impl(osal_queue_handle_t *p_queue_handle, size_t queue_depth, size_t data_size);

void os_queue_delete_impl(osal_queue_handle_t queue_handle);

int32_t os_queue_send_impl(osal_queue_handle_t queue_handle, const void *data, osal_tick_type_t timeout);

int32_t os_queue_receive_impl(osal_queue_handle_t queue_handle, void *data, osal_tick_type_t timeout);

int32_t os_queue_msg_waiting_impl(osal_queue_handle_t queue_handle);

#endif // __OSAL_INTERNAL_QUEUE_H__

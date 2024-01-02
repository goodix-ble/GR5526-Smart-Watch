#ifndef __OSAL_QUEUE_H__
#define __OSAL_QUEUE_H__

#include "common_types.h"

int32_t osal_queue_create(osal_queue_handle_t *p_queue_handle, size_t queue_depth, size_t data_size);

int32_t osal_queue_delete(osal_queue_handle_t queue_handle);

int32_t osal_queue_send(osal_queue_handle_t queue_handle, const void *data, osal_tick_type_t timeout);
int32_t osal_queue_receive(osal_queue_handle_t queue_handle, const void *data, osal_tick_type_t timeout);

int32_t osal_queue_peek(osal_queue_handle_t queue_handle);

int32_t osal_queue_msg_waiting(osal_queue_handle_t queue_handle);



#endif // __OSAL_QUEUE_H__

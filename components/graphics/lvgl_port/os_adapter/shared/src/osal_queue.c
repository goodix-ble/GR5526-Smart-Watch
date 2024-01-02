#include "osal_internal_queue.h"
#include "osal_internal_globaldefs.h"
// #include "osal_internal_idmap.h"

#include "app_log.h"

int32_t osal_queue_create(osal_queue_handle_t *p_queue_handle, size_t queue_depth, size_t data_size)
{
    int32_t ret;
    ret = os_queue_create_impl(p_queue_handle, queue_depth, data_size);
    return ret;
}

void osal_queue_delete(osal_queue_handle_t queue_handle)
{
    os_queue_delete_impl(queue_handle);
}

int32_t osal_queue_send(osal_queue_handle_t queue_handle, const void *data, osal_tick_type_t timeout)
{
    int32_t ret;
    ret = os_queue_send_impl(queue_handle, data, timeout);
    return ret;
}

int32_t osal_queue_receive(osal_queue_handle_t queue_handle, void *data, osal_tick_type_t timeout)
{
    int32_t ret;
    ret = os_queue_receive_impl(queue_handle, data, timeout);
    return ret;
}

int32_t osal_queue_peek(osal_queue_handle_t *queue_handle)
{
    int32_t ret = OSAL_SUCCESS;
    return ret;
}

int32_t osal_queue_msg_waiting(osal_queue_handle_t queue_handle)
{
    int32_t num = os_queue_msg_waiting_impl(queue_handle);
    return num;
}

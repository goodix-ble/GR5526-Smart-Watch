#include "osal_internal_queue.h"
#include "os_freertos.h"
#include "app_log.h"

#if (OSAL_RTOS_SUPPORT == FREERTOS_SUPPORT)

int32_t os_queue_create_impl(osal_queue_handle_t *p_queue_handle, size_t queue_depth, size_t data_size)
{
    int32_t ret;
    xQueueHandle cur_queue_handle;
    cur_queue_handle = xQueueCreate(queue_depth, data_size);
    if (cur_queue_handle == NULL)
    {
        ret = OSAL_ERROR;
    }
    else
    {
        *p_queue_handle = (osal_queue_handle_t)cur_queue_handle;
        int32_t num = uxQueueMessagesWaiting(cur_queue_handle);
        ret = OSAL_SUCCESS;
    }
    return ret;
}

void os_queue_delete_impl(osal_queue_handle_t queue_handle)
{
    vQueueDelete((xQueueHandle)queue_handle);
}

int32_t os_queue_send_impl(osal_queue_handle_t queue_handle, const void *data, osal_tick_type_t timeout)
{
    int32_t ret;
    BaseType_t status;
    xQueueHandle handle = (xQueueHandle)queue_handle;

    OSAL_CHECK_POINTER(queue_handle);
    OSAL_CHECK_POINTER(data);

    if (OSAL_IS_IN_ISR())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        status = xQueueSendFromISR(handle, data, &xHigherPriorityTaskWoken);
        if (pdFALSE != xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        status = xQueueSend(handle, data, OS_MS_TO_TICKS(timeout));
    }

    if (status == pdPASS)
    {
        ret = OSAL_SUCCESS;
    }
    else if (errQUEUE_FULL)
    {
        ret = OSAL_ERROR;
    }
    else
    {
        ret = OSAL_ERROR;
    }
    return ret;
}

int32_t os_queue_receive_impl(osal_queue_handle_t queue_handle, void *data, osal_tick_type_t timeout)
{
    int32_t ret;
    BaseType_t status;
    xQueueHandle handle = (xQueueHandle)queue_handle;
    OSAL_CHECK_POINTER(handle);
    OSAL_CHECK_POINTER(data);

    if (OSAL_IS_IN_ISR())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        status = xQueueReceiveFromISR(handle, data, &xHigherPriorityTaskWoken);
        if (pdFALSE != xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        status = xQueueReceive(handle, data, OS_MS_TO_TICKS(timeout));
    }
    if (pdPASS == status)
    {
        ret = OSAL_SUCCESS;
    }
    else
    {
        ret = OSAL_ERROR;
    }
    return ret;
}

int32_t os_queue_msg_waiting_impl(osal_queue_handle_t queue_handle)
{
    int32_t ret;
    xQueueHandle handle = (xQueueHandle)queue_handle;
    if (OSAL_IS_IN_ISR())
    {
        ret = uxQueueMessagesWaitingFromISR(handle);
    }
    else
    {
        ret = uxQueueMessagesWaiting(handle);
    }
    return ret;
}

#endif // OSAL_RTOS_SUPPORT

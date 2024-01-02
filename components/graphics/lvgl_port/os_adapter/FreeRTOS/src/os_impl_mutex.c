#include "osal_internal_mutex.h"
#include "os_freertos.h"

#if (OSAL_RTOS_SUPPORT == FREERTOS_SUPPORT)

int32_t os_mutex_create_impl(osal_mutex_handle_t *p_mutex_handle)
{
    int32_t ret;
    xSemaphoreHandle cur_mutex_handle;
    cur_mutex_handle = xSemaphoreCreateMutex();
    if (cur_mutex_handle == NULL)
    {
        ret = OSAL_ERROR;
    }
    else
    {
        *p_mutex_handle = (osal_mutex_handle_t)cur_mutex_handle;
        ret = OSAL_SUCCESS;
    }
    return ret;
}

void os_mutex_delete_impl(osal_mutex_handle_t mutex_handle)
{
    vSemaphoreDelete((xSemaphoreHandle)mutex_handle);
}

int32_t os_mutex_give_impl(osal_mutex_handle_t mutex_handle)
{
    int32_t ret;
    BaseType_t status;
    xSemaphoreHandle handle = (xSemaphoreHandle)mutex_handle;

    OSAL_CHECK_POINTER(handle);

    if (OSAL_IS_IN_ISR())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        status = xSemaphoreGiveFromISR(handle, &xHigherPriorityTaskWoken);
        if (pdFALSE != xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        status = xSemaphoreGive(handle);
    }

    if (status == pdPASS)
    {
        ret = OSAL_SUCCESS;
    }
    else
    {
        ret = OSAL_ERROR;
    }
    return ret;
}

int32_t os_mutex_take_impl(osal_mutex_handle_t mutex_handle, osal_tick_type_t timeout)
{
    int32_t ret;
    BaseType_t status;
    xSemaphoreHandle handle = (xSemaphoreHandle)mutex_handle;
    OSAL_CHECK_POINTER(handle);

    if (OSAL_IS_IN_ISR())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        status = xSemaphoreTakeFromISR(handle, &xHigherPriorityTaskWoken);
        if (pdFALSE != xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        status = xSemaphoreTake(handle, OS_MS_TO_TICKS(timeout));
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

#endif // OSAL_RTOS_SUPPORT

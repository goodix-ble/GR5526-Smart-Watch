#include "osal_internal_sema.h"
#include "os_freertos.h"
#include "app_log.h"

#if (OSAL_RTOS_SUPPORT == FREERTOS_SUPPORT)

int32_t os_sema_binary_create_impl(osal_sema_handle_t *p_sema_handle)
{
    int32_t ret;
    xSemaphoreHandle cur_sema_handle;
    cur_sema_handle = xSemaphoreCreateBinary();
    if (cur_sema_handle == NULL)
    {
        ret = OSAL_INVALID_POINTER;
    }
    else
    {
        *p_sema_handle = cur_sema_handle;
        ret = OSAL_SUCCESS;
    }
    return ret;
}

int32_t os_sema_countings_create_impl(osal_sema_handle_t *p_sema_handle, uint32_t max_count, uint32_t init_count)
{
    int32_t ret;
    xSemaphoreHandle cur_sema_handle;
    cur_sema_handle = xSemaphoreCreateCounting(max_count, init_count);
    if (cur_sema_handle == NULL)
    {
        ret = OSAL_ERROR;
    }
    else
    {
        *p_sema_handle = cur_sema_handle;
        ret = OSAL_SUCCESS;
    }
    return ret;
}

void os_sema_delete_impl(osal_sema_handle_t sema_handle)
{
    vSemaphoreDelete((xSemaphoreHandle)sema_handle);
}

int32_t os_sema_give_impl(osal_sema_handle_t sema_handle)
{
    int32_t ret;
    BaseType_t status;
    xSemaphoreHandle handle = (xSemaphoreHandle)sema_handle;

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

int32_t os_sema_take_impl(osal_sema_handle_t sema_handle, osal_tick_type_t timeout)
{
    int32_t ret;
    BaseType_t status;
    xSemaphoreHandle handle = (xSemaphoreHandle)sema_handle;
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

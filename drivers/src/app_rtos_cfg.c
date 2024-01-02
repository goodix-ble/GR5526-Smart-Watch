/**
  ****************************************************************************************
  * @file    app_rtos_cfg.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2019 GOODIX
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of GOODIX nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
  ****************************************************************************************
  */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_rtos_cfg.h"
#include "app_drv_error.h"
#include <stdio.h>
#include <string.h>


#ifdef ENV_USE_FREERTOS

#include "FreeRTOS.h"
#include "semphr.h"

uint16_t app_driver_sem_init(sem_t *sem)
{
    SemaphoreHandle_t *xSemaphore = (SemaphoreHandle_t *)sem;

    if (NULL == sem)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    *xSemaphore = xSemaphoreCreateBinary();
    if (NULL == *xSemaphore)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }
    return APP_DRV_SUCCESS;
}

void app_driver_sem_deinit(sem_t sem)
{
    vSemaphoreDelete(sem);
}

uint16_t app_driver_sem_pend(sem_t sem, uint32_t time_out)
{
    BaseType_t ret = pdTRUE;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    SemaphoreHandle_t xSemaphore = (SemaphoreHandle_t)sem;
    TickType_t xTicks = portMAX_DELAY;

    if (NULL == sem)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (time_out != portMAX_DELAY)
    {
        xTicks = pdMS_TO_TICKS(time_out);
    }

    if (__get_IPSR())
    {
        ret = xSemaphoreTakeFromISR(xSemaphore, &xHigherPriorityTaskWoken);
        if(xHigherPriorityTaskWoken != pdFALSE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        ret = xSemaphoreTake(xSemaphore, xTicks);
    }
    if (ret != pdTRUE)
    {
        return APP_DRV_ERR_TIMEOUT;
    }
    return APP_DRV_SUCCESS;
}

uint16_t app_driver_sem_post(sem_t sem)
{
    BaseType_t ret = pdTRUE;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    SemaphoreHandle_t xSemaphore = (SemaphoreHandle_t)sem;

    if (NULL == sem)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    if (__get_IPSR())
    {
        ret = xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
        if(xHigherPriorityTaskWoken != pdFALSE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        ret = xSemaphoreGive(xSemaphore);
    }
    if (ret != pdTRUE)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    return APP_DRV_SUCCESS;
}

uint16_t app_driver_sem_post_from_isr(sem_t sem)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t ret = pdTRUE;
    SemaphoreHandle_t xSemaphore = (SemaphoreHandle_t)sem;

    if (NULL == sem)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    ret = xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken != pdFALSE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    if (ret != pdTRUE)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    return APP_DRV_SUCCESS;
}

uint16_t app_driver_mutex_init(mutex_t *mutex)
{
    SemaphoreHandle_t *xMutex = (SemaphoreHandle_t *)(mutex);

    if (NULL == mutex)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    *xMutex = xSemaphoreCreateMutex();
    if (NULL == *xMutex)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }
    return APP_DRV_SUCCESS;
}

void app_driver_mutex_deinit(mutex_t mutex)
{
    vSemaphoreDelete(mutex);
}

uint16_t app_driver_mutex_pend(mutex_t mutex, uint32_t time_out)
{
    SemaphoreHandle_t xMutex = (SemaphoreHandle_t)mutex;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t ret = pdTRUE;
    TickType_t xTicks = portMAX_DELAY;

    if (NULL == mutex)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (time_out != portMAX_DELAY)
    {
        xTicks = pdMS_TO_TICKS(time_out);
    }

    if (__get_IPSR())
    {
        ret = xSemaphoreTakeFromISR(xMutex, &xHigherPriorityTaskWoken);
        if(xHigherPriorityTaskWoken != pdFALSE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        ret = xSemaphoreTake(xMutex, xTicks);
    }

    if (ret != pdTRUE)
    {
        return APP_DRV_ERR_TIMEOUT;
    }
    return APP_DRV_SUCCESS;
}

uint16_t app_driver_mutex_post(mutex_t mutex)
{
    BaseType_t ret = pdTRUE;
    SemaphoreHandle_t xMutex = (SemaphoreHandle_t)mutex;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (NULL == mutex)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    if (__get_IPSR())
    {
        ret = xSemaphoreGiveFromISR(xMutex, &xHigherPriorityTaskWoken);
        if(xHigherPriorityTaskWoken != pdFALSE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        ret = xSemaphoreGive(xMutex);
    }

    if (ret != pdTRUE)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    return APP_DRV_SUCCESS;
}

#endif

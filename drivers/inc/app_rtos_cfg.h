/**
 ****************************************************************************************
 *
 * @file    app_rtos_cfg.h
 * @author  BLE Driver Team
 * @brief   Header file of app rtos config code.
 *
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

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_RTOS_CONFIG RTOS CONFIG
  * @brief ADC RTOS CONFIG
  * @{
  */


#ifndef __APP_RTOS_ADAPTER_H
#define __APP_RTOS_ADAPTER_H
/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <string.h>
#include <stdint.h>

#ifdef ENV_USE_FREERTOS

#include "FreeRTOS.h"
#include "semphr.h"

/** @addtogroup APP_RTOS_CONFIG_DEFINES Defines
  * @{
  */
#define ENV_USE_RTOS
/** @} */

/**
 * @defgroup APP_RTOS_CONFIG_TYPEDEF Type definitions
 * @{
 */

/**
  * @brief Semaphore type definition
  */
typedef SemaphoreHandle_t             sem_t;

/**
  * @brief mutex type definition
  */
typedef SemaphoreHandle_t             mutex_t;

/** @} */

/** @addtogroup APP_RTOS_CONFIG_DEFINES Defines
  * @{
  */
#define OS_WAIT_FOREVER               portMAX_DELAY     /**< Block forever until get resource */

#define SEM_WAIT_FOREVER              portMAX_DELAY     /**< Wait for the semaphore forever */
#define SEM_NO_WAIT                   (0)               /**< Non-block */

#define MUTEX_WAIT_FOREVER            portMAX_DELAY     /**< Wait for the mutex forever */
#define MUTEX_NO_WAIT                 (0)               /**< Non-block */
/** @} */

#else

/**
 * @defgroup APP_RTOS_CONFIG_TYPEDEF Typedefs
 * @{
 */

/**
  * @brief Semaphore type definition
  */
typedef void *                        sem_t;

/**
  * @brief mutex type definition
  */
typedef void *                        mutex_t;
/** @} */

/** @addtogroup APP_RTOS_CONFIG_DEFINES Defines
  * @{
  */
#define SEM_WAIT_FOREVER              (0xFFFFUL)        /**< Wait for the semaphore forever. */
#define SEM_NO_WAIT                   (0)               /**< Non-block */

#define MUTEX_WAIT_FOREVER            (0xFFFFUL)        /**< Wait for the mutex forever */
#define MUTEX_NO_WAIT                 (0)               /**< Non-block */
/** @} */

#endif

/** @addtogroup APP_RTOS_CONFIG_DEFINES Defines
  * @{
  */
#define APP_DRV_SEM_DECL(sem)          sem_t   sem              /**< Define a semaphore instance */
#define APP_DRV_MUTEX_DECL(mutex)      mutex_t mutex            /**< Define a mutex instance */

#define APP_DRV_SEM_STATIC(sem)        static APP_DRV_SEM_DECL(sem)     /**< Define a static semaphore instance */
#define APP_DRV_MUTEX_STATIC(mutex)    static APP_DRV_MUTEX_DECL(mutex) /**< Define a static mutex instance */
/** @} */

#ifdef ENV_USE_RTOS
/** @addtogroup APP_RTOS_CONFIG_DEFINES Defines
  * @{
  */
#define ENV_RTOS_USE_SEMP    1          /**< Enable semaphore in app driver */
//#define ENV_RTOS_USE_MUTEX   1          /**< Enable mutex in app driver */
/** @} */

/** @addtogroup APP_RTOS_CONFIG_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize a semaphore.
 *
 * @param[in] sem: Pointer to a sem_t parameter which contains the address of the semaphore object.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_driver_sem_init(sem_t *sem);

/**
 ****************************************************************************************
 * @brief  De-initialize a semaphore.
 *
 * @param[in] sem: the semaphore object.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
void app_driver_sem_deinit(sem_t sem);

/**
 ****************************************************************************************
 * @brief  This function will take a semaphore, if the semaphore is unavailable, the
           thread shall wait for a specified time.
 *
 * @param[in] sem: The semaphore object.
 * @param[in] time_out: The waiting time in milliseconds.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_driver_sem_pend(sem_t sem, uint32_t time_out);

/**
 ****************************************************************************************
 * @brief  This function will release a semaphore, if there are threads suspended on
 *         semaphore, it will be waked up.
 *
 * @param[in] sem: The semaphore object.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_driver_sem_post(sem_t sem);

/**
 ****************************************************************************************
 * @brief  This function will release a semaphore, it is used in interrupt service function,
 *         if there are threads suspended on semaphore, it will be waked up.
 *
 * @param[in] sem: The semaphore object.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_driver_sem_post_from_isr(sem_t sem);

/**
 ****************************************************************************************
 * @brief  Initialize a mutex.
 *
 * @param[in] mutex: Pointer to mutex_t parameter which contains the address of the mutex object.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_driver_mutex_init(mutex_t *mutex);

/**
 ****************************************************************************************
 * @brief  De-initialize a mutex.
 *
 * @param[in] mutex: the mutex object.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
void app_driver_mutex_deinit(mutex_t mutex);

/**
 ****************************************************************************************
 * @brief  This function will take a mutex, if the mutex is unavailable, the thread shall
 *         wait for a specified time.
 *
 * @param[in] mutex: The mutex object.
 * @param[in] time_out: The waiting time in milliseconds.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_driver_mutex_pend(mutex_t mutex, uint32_t time_out);

/**
 ****************************************************************************************
 * @brief  This function will release a mutex, if there are threads suspended on mutex,
 *         it will be waked up.
 *
 * @param[in] mutex: The mutex object.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_driver_mutex_post(mutex_t mutex);
/** @} */

#else
/** @addtogroup APP_RTOS_CONFIG_DEFINES Defines
  * @{
  */
#define app_driver_sem_init(x)        (0)   /**< Initialize the semaphore. */
#define app_driver_sem_deinit(x)            /**< Deinitialize the semaphore. */
#define app_driver_sem_pend(x, y)     (0)   /**< Pend the semaphore. */
#define app_driver_sem_post(x)              /**< Post the semaphore. */
#define app_driver_sem_post_from_isr(x)     /**< Post the semaphore from interrupt. */

#define app_driver_mutex_init(x)      (0)   /**< Initialize the mutex. */
#define app_driver_mutex_deinit(x)          /**< Deinitialize the mutex. */
#define app_driver_mutex_pend(x, y)         /**< Pend the mutex. */
#define app_driver_mutex_post(x)            /**< Post the mutex. */
/** @} */

#endif

#endif
/** @} */
/** @} */
/** @} */

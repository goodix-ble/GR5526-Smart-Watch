/**
 ****************************************************************************************
 *
 * @file app_scheduler.h
 *
 * @brief App Scheduler API
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
 *****************************************************************************************
 */

#ifndef __APP_SCHEDULER_H__
#define __APP_SCHEDULER_H__

#include "grx_sys.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup APP_SCHEDULER_TYPEDEF Typedefs
 * @{
 */
/**@brief  APP Scheduler event handler type. */
typedef void (*app_scheduler_evt_handler_t)(void *p_evt_data, uint16_t evt_data_size);
/** @} */

/**
 * @defgroup APP_SCHEDULER_STRUCT Structures
 * @{
 */
/**@brief App scheduler event information. */
typedef struct
{
    app_scheduler_evt_handler_t  evt_handler;        /**< Event handler. */
    void                        *p_evt_data;         /**< Pointer to event data. */
    uint16_t                     evt_data_size;      /**< Size of event data. */
} app_scheduler_evt_info_t;
/** @} */

/**
 * @defgroup APP_SCHEDULER_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize app scheduler module.
 *
 * @param[in] queue_size: Event queue size.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t app_scheduler_init(uint16_t queue_size);

/**
 *****************************************************************************************
 * @brief Put an event into event queue.
 *
 * @param[in] p_evt_data:     Pointer to event data.
 * @param[in] evt_data_size:  Size of event data.
 * @param[in] evt_handler:    Event handler.
 *
 * @return  Result of put.
 *****************************************************************************************
 */
sdk_err_t app_scheduler_evt_put(void const *p_evt_data, uint16_t evt_data_size, app_scheduler_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Executing all events.
 *****************************************************************************************
 */
void app_scheduler_execute(void);
/** @} */

#endif



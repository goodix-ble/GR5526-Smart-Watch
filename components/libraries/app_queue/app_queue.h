 /**
 *****************************************************************************************
 *
 * @file app_queue.h
 *
 * @brief Header file - APP QUEUE APIs
 *
 *****************************************************************************************
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

#ifndef __APP_QUEUE_H__
#define __APP_QUEUE_H__

#include "grx_sys.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup APP_QUEUE_STRUCT Structures
 * @{
 */
/**@brief App queue instance information. */
typedef struct
{
    uint16_t       element_size;  /**< Size of app queue element. */
    uint16_t       queue_size;    /**< Size of app queue buffer. */
    void          *p_buffer;      /**< Pointer to app queue buffer. */
    uint16_t       start_idx;     /**< Start index of app queue. */
    uint16_t       end_idx;       /**< End index of app queue. */
} app_queue_t;
/** @} */

/**
 * @defgroup APP_QUEUE_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize one app queue instance.
 *
 * @param[in] p_queue:      Pointer to app queue instance.
 * @param[in] p_buffer:     Pointer to queue buffer.
 * @param[in] queue_size:   Size of queue buffer(The actual queue allocation size is one more than available).
 * @param[in] element_size: Size of queue element
 *
 * @return Result of initializing app queue.
 *****************************************************************************************
 */
sdk_err_t app_queue_init(app_queue_t *p_queue, void *p_buffer, uint16_t queue_size, uint16_t element_size);

/**
 *****************************************************************************************
 * @brief Push one element to tail of app queue.
 *
 * @param[in] p_queue:    Pointer to app queue instance.
 * @param[in] p_elemment: Pointer to the element that will be stored in the queue.
 *
 * @return Result of element push.
 *****************************************************************************************
 */
sdk_err_t app_queue_push(app_queue_t *p_queue, void const *p_elemment);

/**
 *****************************************************************************************
 * @brief Push some elements to tail of app queue.
 *
 * @param[in] p_queue:    Pointer to app queue instance.
 * @param[in] p_elemment: Pointer to the elements that will be stored in the queue.
 * @param[in] amount:     Amount of the elements that wants be stored in the queue.
 *
 * @return Amount of writen elements.
 *****************************************************************************************
 */
uint16_t app_queue_multi_push(app_queue_t *p_queue, void const *p_elemment, uint16_t amount);

/**
 *****************************************************************************************
 * @brief Peek one element from tail of app queue.
 *
 * @param[in]  p_queue:    Pointer to app queue instance.
 * @param[out] p_elemment: Pointer to where the element will be copied.
 *
 * @return Result of element peek.
 *****************************************************************************************
 */
sdk_err_t app_queue_peek(app_queue_t *p_queue, void *p_elemment);

/**
 *****************************************************************************************
 * @brief Pop one element from tail of app queue.
 *
 * @param[in]  p_queue:    Pointer to app queue instance.
 * @param[out] p_elemment: Pointer to where the element will be copied.
 *
 * @return Result of element pop.
 *****************************************************************************************
 */
sdk_err_t app_queue_pop(app_queue_t *p_queue, void *p_elemment);

/**
 *****************************************************************************************
 * @brief Get next index.
 *
 * @param[in] curr_idx: Current index.
 * @param[in] size:     Size of current app queue.
 *
 * @retval  Next index.
 *****************************************************************************************
 */
__STATIC_FORCEINLINE uint16_t app_queue_next_idx_get(uint16_t curr_idx, uint16_t size)
{
    return (curr_idx < size) ? (curr_idx + 1) : 0;
}

/**
 *****************************************************************************************
 * @brief Check app queue is full or not.
 *
 * @param[in] p_queue: Pointer to app queue instance.
 *
 * @return Result of check.
 *****************************************************************************************
 */
__STATIC_FORCEINLINE bool app_queue_is_full(app_queue_t *p_queue)
{
    uint16_t next_idx;

    next_idx = app_queue_next_idx_get( p_queue->end_idx, p_queue->queue_size);

    return next_idx == p_queue->start_idx;
}

/**
 *****************************************************************************************
 * @brief Check app queue is empty or not.
 *
 * @param[in] p_queue: Pointer to app queue instance.
 *
 * @return Result of check.
 *****************************************************************************************
 */
__STATIC_FORCEINLINE bool app_queue_is_empty(app_queue_t *p_queue)
{
    return (p_queue->start_idx == p_queue->end_idx);
}

/**
 *****************************************************************************************
 * @brief Get surplus space of one app queue.
 *
 * @param[in] p_queue: Pointer to app queue instance.
 *
 * @retval  Size of surplus space.
 *****************************************************************************************
 */
uint16_t app_queue_surplus_space_get(app_queue_t *p_queue);

/**
 *****************************************************************************************
 * @brief Get availdble data from one ring buffer.
 *
 * @param[in] p_queue: Pointer to app queue instance.
 *
 * @retval  Count of availdble items.
 *****************************************************************************************
 */
uint16_t app_queue_items_count_get(app_queue_t *p_queue);

/**
 *****************************************************************************************
 * @brief Clean one app queue.
 *
 * @param[in]  p_queue: Pointer to app queue instance.
 *****************************************************************************************
 */
void app_queue_clean(app_queue_t *p_queue);

/** @} */


#endif

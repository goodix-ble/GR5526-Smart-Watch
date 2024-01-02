/**
 *****************************************************************************************
 *
 * @file app_queue.c
 *
 * @brief APP Queue function Implementation.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_queue.h"
#include "grx_hal.h"
#include <stdio.h>
#include <string.h>

/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_QUEUE_LOCK()    LOCAL_INT_DISABLE(BLE_IRQn)
#define APP_QUEUE_UNLOCK()  LOCAL_INT_RESTORE()

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t app_queue_init(app_queue_t *p_queue, void *p_buffer, uint16_t queue_size, uint16_t element_size)
{
    if (NULL == p_queue || NULL == p_buffer)
    {
        return SDK_ERR_POINTER_NULL;
    }

    p_queue->element_size = element_size;
    p_queue->queue_size   = queue_size;
    p_queue->p_buffer     = p_buffer;
    p_queue->start_idx    = 0;
    p_queue->end_idx      = 0;

    return SDK_SUCCESS;
}

sdk_err_t app_queue_push(app_queue_t *p_queue, void const *p_elemment)
{
    bool       is_full;
    uint16_t   wr_idx;
    void      *p_wr_pos;
    sdk_err_t  error_code = SDK_SUCCESS;

    if (NULL == p_queue || NULL == p_elemment)
    {
        return SDK_ERR_POINTER_NULL;
    }

    APP_QUEUE_LOCK();

    is_full = app_queue_is_full(p_queue);

    if (is_full)
    {
        error_code =  SDK_ERR_NO_RESOURCES;
    }
    else
    {
        wr_idx           = p_queue->end_idx;
        p_wr_pos         = (void *)((size_t)p_queue->p_buffer + wr_idx * p_queue->element_size);
        p_queue->end_idx = app_queue_next_idx_get(p_queue->end_idx, p_queue->queue_size);

        memcpy(p_wr_pos, p_elemment, p_queue->element_size);
    }

    APP_QUEUE_UNLOCK();

    return error_code;
}

uint16_t app_queue_multi_push(app_queue_t *p_queue, void const *p_elemment, uint16_t amount)
{
    uint16_t  stored_num    = 0;
    uint16_t  surplus_space = 0;
    uint16_t  over_flow     = 0;
    uint16_t  wr_idx;
    void     *p_wr_pos;
    void     *p_head_pos;

    if (NULL == p_elemment || 0 == amount)
    {
        stored_num = 0;
    }

    APP_QUEUE_LOCK();

    surplus_space = app_queue_surplus_space_get(p_queue);
    wr_idx        = p_queue->end_idx;
    p_wr_pos      = (void *)((size_t)p_queue->p_buffer + wr_idx * p_queue->element_size);
    p_head_pos    = (void *)((size_t)p_queue->p_buffer);

    stored_num = amount > surplus_space ? surplus_space : amount;

    if (p_queue->start_idx <= p_queue->end_idx)
    {
        if (p_queue->end_idx + amount >= p_queue->queue_size)
        {
            over_flow = p_queue->end_idx + amount - p_queue->queue_size;
        }
    }

    memcpy(p_wr_pos, p_elemment, p_queue->element_size * (stored_num - over_flow));
    memcpy(p_head_pos,
           (void const *)((size_t)p_elemment + p_queue->element_size * (stored_num - over_flow)),
           p_queue->element_size * over_flow);

    wr_idx += stored_num;

    if (wr_idx >= p_queue->queue_size)
    {
        wr_idx -= p_queue->queue_size;
    }

    p_queue->end_idx = wr_idx;

    APP_QUEUE_UNLOCK();

    return stored_num;
}

sdk_err_t app_queue_peek(app_queue_t *p_queue, void *p_elemment)
{
    uint16_t   peek_idx   = 0;
    void      *p_peek_pos = NULL;
    sdk_err_t  error_code = SDK_SUCCESS;

    if (NULL == p_queue || NULL == p_elemment)
    {
        return SDK_ERR_POINTER_NULL;
    }

    APP_QUEUE_LOCK();

    if (app_queue_is_empty(p_queue))
    {
        error_code = SDK_ERR_LIST_ITEM_NOT_FOUND;
    }

    peek_idx   = p_queue->start_idx;
    p_peek_pos = (void *)((size_t)p_queue->p_buffer + peek_idx * p_queue->element_size);

    memcpy(p_elemment, p_peek_pos, p_queue->element_size);

    APP_QUEUE_UNLOCK();

    return error_code;
}

sdk_err_t app_queue_pop(app_queue_t *p_queue, void *p_elemment)
{
    uint16_t   peek_idx   = 0;
    void      *p_peek_pos = NULL;
    sdk_err_t  error_code = SDK_SUCCESS;

    if (NULL == p_queue || NULL == p_elemment)
    {
        return SDK_ERR_POINTER_NULL;
    }

    APP_QUEUE_LOCK();

    if (app_queue_is_empty(p_queue))
    {
        error_code = SDK_ERR_LIST_ITEM_NOT_FOUND;
    }
    else
    {
        peek_idx           = p_queue->start_idx;
        p_peek_pos         = (void *)((size_t)p_queue->p_buffer + peek_idx * p_queue->element_size);
        p_queue->start_idx = app_queue_next_idx_get(p_queue->start_idx, p_queue->queue_size);

        memcpy(p_elemment, p_peek_pos, p_queue->element_size);
    }

    APP_QUEUE_UNLOCK();

    return error_code;
}

uint16_t app_queue_surplus_space_get(app_queue_t *p_queue)
{
    uint16_t start_idx     = p_queue->start_idx;
    uint16_t end_idx       = p_queue->end_idx;
    uint16_t queue_size    = p_queue->queue_size;
    uint16_t surplus_space = 0;

    APP_QUEUE_LOCK();

    if (start_idx <= end_idx)
    {
        surplus_space = queue_size - end_idx + start_idx;
    }
    else
    {
        surplus_space = start_idx - end_idx - 1;
    }

    APP_QUEUE_UNLOCK();

    return surplus_space;
}

uint16_t app_queue_items_count_get(app_queue_t *p_queue)
{
    uint16_t start_idx     = p_queue->start_idx;
    uint16_t end_idx       = p_queue->end_idx;
    uint16_t queue_size    = p_queue->queue_size;
    uint16_t items_count   = 0;

    if (NULL == p_queue)
    {
        return 0;
    }

    APP_QUEUE_LOCK();

    if (start_idx <= end_idx)
    {
        items_count = end_idx - start_idx;
    }
    else
    {
        items_count = queue_size - start_idx + end_idx;
    }

    APP_QUEUE_UNLOCK();

    return items_count;
}

void app_queue_clean(app_queue_t *p_queue)
{
    APP_QUEUE_LOCK();

    p_queue->start_idx = 0;
    p_queue->end_idx   = 0;

    APP_QUEUE_UNLOCK();
}


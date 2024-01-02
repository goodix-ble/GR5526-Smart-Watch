/**
 *****************************************************************************************
 *
 * @file ring_buffer.c
 *
 * @brief Ring buffer function Implementation.
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
#include "ring_buffer.h"
#include "grx_hal.h"
#include "utility.h"
#include <string.h>

/*
 * DEFINES
 *****************************************************************************************
 */
#define RING_BUFFER_LOCK()      GLOBAL_EXCEPTION_DISABLE()
#define RING_BUFFER_UNLOCK()    GLOBAL_EXCEPTION_ENABLE()
#define RING_BUFFER_SIZE_MIN    (1)

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
bool ring_buffer_init(ring_buffer_t *p_ring_buff, uint8_t *p_buff, uint32_t buff_size)
{
    if ((NULL == p_buff) || (NULL == p_ring_buff) || (buff_size < RING_BUFFER_SIZE_MIN))
    {
        return false;
    }
    else
    {
        RING_BUFFER_LOCK();
        p_ring_buff->buffer_size = buff_size;
        p_ring_buff->p_buffer    = p_buff;
        p_ring_buff->write_index = 0;
        p_ring_buff->read_index  = 0;
        RING_BUFFER_UNLOCK();

        return true;
    }
}

uint32_t ring_buffer_write(ring_buffer_t *p_ring_buff, uint8_t const *p_wr_data, uint32_t length)
{
    uint32_t surplus_space = 0;
    uint32_t over_flow     = 0;

    RING_BUFFER_LOCK();

    uint32_t wr_idx = p_ring_buff->write_index;
    uint32_t rd_idx = p_ring_buff->read_index;

    if ((NULL != p_ring_buff) &&
        (NULL != p_ring_buff->p_buffer) &&
        (NULL != p_wr_data))
    {
        if (rd_idx > wr_idx)
        {
            surplus_space = rd_idx - wr_idx - 1;
            length        = (length > surplus_space ? surplus_space : length);
        }
        else
        {
            surplus_space = p_ring_buff->buffer_size - wr_idx + rd_idx - 1;
            length        = (length > surplus_space ? surplus_space : length);

            if (wr_idx + length >= p_ring_buff->buffer_size)
            {
                over_flow = wr_idx + length - p_ring_buff->buffer_size;
            }
        }

        memcpy(p_ring_buff->p_buffer + wr_idx, p_wr_data, length - over_flow);
        memcpy(p_ring_buff->p_buffer, p_wr_data + length - over_flow, over_flow);
        wr_idx += length;

        if (wr_idx >= p_ring_buff->buffer_size)
        {
            wr_idx -= p_ring_buff->buffer_size;
        }

        p_ring_buff->write_index = wr_idx;
    }
    else
    {
        length = 0;
    }

    RING_BUFFER_UNLOCK();

    return length;
}

uint32_t ring_buffer_read(ring_buffer_t *p_ring_buff, uint8_t *p_rd_data, uint32_t length)
{
    uint32_t items_avail = 0;
    uint32_t over_flow   = 0;

    RING_BUFFER_LOCK();

    uint32_t wr_idx = p_ring_buff->write_index;
    uint32_t rd_idx = p_ring_buff->read_index;

    if ((NULL != p_ring_buff) &&
        (NULL != p_ring_buff->p_buffer) &&
        (NULL != p_rd_data))
    {
        if (wr_idx >= rd_idx)
        {
            items_avail = wr_idx - rd_idx;
            length = (length > items_avail ? items_avail : length);
        }
        else
        {
            items_avail = p_ring_buff->buffer_size - rd_idx + wr_idx;
            length = (length > items_avail ? items_avail : length);

            if (rd_idx + length >= p_ring_buff->buffer_size)
            {
                over_flow = length + rd_idx - p_ring_buff->buffer_size;
            }
        }

        memcpy(p_rd_data, p_ring_buff->p_buffer + rd_idx, length - over_flow);
        memcpy(p_rd_data + length - over_flow, p_ring_buff->p_buffer, over_flow);
        rd_idx += length;

        if (rd_idx >= p_ring_buff->buffer_size && rd_idx > wr_idx)
        {
            rd_idx -= p_ring_buff->buffer_size;
        }

        p_ring_buff->read_index = rd_idx;
    }
    else
    {
        length = 0;
    }

    RING_BUFFER_UNLOCK();

    return length;
}

uint32_t ring_buffer_pick(ring_buffer_t *p_ring_buff, uint8_t *p_rd_data, uint32_t length)
{
    uint32_t items_avail = 0;
    uint32_t over_flow   = 0;

    RING_BUFFER_LOCK();

    uint32_t wr_idx = p_ring_buff->write_index;
    uint32_t rd_idx = p_ring_buff->read_index;

    if ((NULL != p_ring_buff) &&
        (NULL != p_ring_buff->p_buffer) &&
        (NULL != p_rd_data))
    {
        if (wr_idx >= rd_idx)
        {
            items_avail = wr_idx - rd_idx;
            length = (length > items_avail ? items_avail : length);
        }
        else
        {
            items_avail = p_ring_buff->buffer_size - rd_idx + wr_idx;
            length = (length > items_avail ? items_avail : length);

            if (rd_idx + length >= p_ring_buff->buffer_size)
            {
                over_flow = length + rd_idx - p_ring_buff->buffer_size;
            }
        }

        memcpy(p_rd_data, p_ring_buff->p_buffer + rd_idx, length - over_flow);
        memcpy(p_rd_data + length - over_flow, p_ring_buff->p_buffer, over_flow);
    }
    else
    {
        length = 0;
    }

    RING_BUFFER_UNLOCK();

    return length;
}

uint32_t ring_buffer_items_count_get(ring_buffer_t *p_ring_buff)
{
    uint32_t count = 0;
    if (NULL == p_ring_buff)
        return 0;

    RING_BUFFER_LOCK();

    uint32_t wr_idx = p_ring_buff->write_index;
    uint32_t rd_idx = p_ring_buff->read_index;

    if (rd_idx <= wr_idx)
    {
        count = wr_idx - rd_idx;
    }
    else
    {
        count = p_ring_buff->buffer_size - rd_idx + wr_idx;
    }

    RING_BUFFER_UNLOCK();

    return count;
}

uint32_t ring_buffer_surplus_space_get(ring_buffer_t *p_ring_buff)
{
    uint32_t surplus_space = 0;

    RING_BUFFER_LOCK();

    uint32_t wr_idx = p_ring_buff->write_index;
    uint32_t rd_idx = p_ring_buff->read_index;

    if (NULL != p_ring_buff)
    {
        if (rd_idx > wr_idx)
        {
            surplus_space = rd_idx - wr_idx - 1;
        }
        else
        {
            surplus_space = p_ring_buff->buffer_size - wr_idx + rd_idx - 1;
        }
    }

    RING_BUFFER_UNLOCK();

    return surplus_space;
}

bool ring_buffer_is_reach_left_threshold(ring_buffer_t *p_ring_buff, uint32_t letf_threshold)
{
    uint32_t surplus_space;

    surplus_space = ring_buffer_surplus_space_get(p_ring_buff);

    if (letf_threshold >= surplus_space)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void ring_buffer_clean(ring_buffer_t *p_ring_buff)
{
    RING_BUFFER_LOCK();

    if (NULL != p_ring_buff)
    {
        p_ring_buff->write_index = 0;
        p_ring_buff->read_index  = 0;
    }

    RING_BUFFER_UNLOCK();
}


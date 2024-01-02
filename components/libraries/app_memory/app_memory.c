/**
 *****************************************************************************************
 *
 * @file app_memory.c
 *
 * @brief App Memory Implementation.
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
#include "app_memory.h"
#include "utility.h"
#include <string.h>

/*
 * DEFINE
 *****************************************************************************************
 */
#define MEM_BLOCK_SIZE          ALIGN_NUM(APP_MEM_ALIGN_NUM, sizeof(app_mem_block_t))
#define BLOCK_ALLOCATED_BIT     ((size_t)1 << (sizeof(size_t) * 8 - 1))
#define BLOCK_ALLOC_SIZE_MIN    (MEM_BLOCK_SIZE + APP_MEM_ALIGN_NUM)

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static __attribute__ ((aligned (APP_MEM_ALIGN_NUM))) uint8_t s_mem_heap[APP_MEM_HEAP_SIZE];

static app_mem_block_t s_start_list_node;
static app_mem_block_t s_end_list_node;
static size_t          s_curr_free_bytes;
static size_t          s_ever_free_bytes_min;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void app_mem_init(void)
{
    app_mem_block_t *p_fir_free_block;

    s_start_list_node.p_next_free_block = (void *)s_mem_heap;
    s_start_list_node.block_size        = 0;


    s_end_list_node.p_next_free_block = NULL;
    s_end_list_node.block_size        = 0;

    p_fir_free_block = (void *)s_mem_heap;

    p_fir_free_block->p_next_free_block = &s_end_list_node;
    p_fir_free_block->block_size        = APP_MEM_HEAP_SIZE;

    s_curr_free_bytes       = p_fir_free_block->block_size;
    s_ever_free_bytes_min = p_fir_free_block->block_size;
}


static void free_block_node_insert(app_mem_block_t *p_insert_node)
{
    app_mem_block_t *p_iterator_node = &s_start_list_node;

    while (p_iterator_node->p_next_free_block < p_insert_node && 
           NULL != p_iterator_node->p_next_free_block->p_next_free_block)
    {
        p_iterator_node = p_iterator_node->p_next_free_block;
    }


    if (((uint8_t *)p_iterator_node + p_iterator_node->block_size) == (uint8_t *)p_insert_node)
    {
        p_iterator_node->block_size += p_insert_node->block_size;
        p_insert_node = p_iterator_node;
    }

    if (((uint8_t *)p_insert_node + p_insert_node->block_size) == (uint8_t *)p_iterator_node->p_next_free_block)
    {
        if (p_iterator_node->p_next_free_block != &s_end_list_node)
        {
            p_insert_node->block_size += p_iterator_node->p_next_free_block->block_size;
            p_insert_node->p_next_free_block = p_iterator_node->p_next_free_block->p_next_free_block;
        }
        else
        {
            p_insert_node->p_next_free_block = &s_end_list_node;
        }
    }
    else
    {
        p_insert_node->p_next_free_block = p_iterator_node->p_next_free_block;
    }

    if (p_iterator_node != p_insert_node)
    {
        p_iterator_node->p_next_free_block = p_insert_node;
    }
}



/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void *app_malloc(size_t size)
{
    app_mem_block_t *p_block;
    app_mem_block_t *p_pre_block;
    app_mem_block_t *p_new_block;
    void            *return_ptr = NULL;

    APP_MEM_LOCK();

    if (NULL == s_start_list_node.p_next_free_block)
    {
        app_mem_init();
    }

    if (0 == (size & BLOCK_ALLOCATED_BIT) && 0 != size)
    {
        size += MEM_BLOCK_SIZE;
        size  = ALIGN_NUM(APP_MEM_ALIGN_NUM, size);
    }
    else
    {
        return return_ptr;
    }

    if (size > 0 && size < s_curr_free_bytes)
    {
        p_pre_block = &s_start_list_node;
        p_block = s_start_list_node.p_next_free_block;

        while ((p_block->block_size < size) && (NULL != p_block->p_next_free_block))
        {
            p_pre_block = p_block;
            p_block     = p_block->p_next_free_block;
        }

        if (p_block != &s_end_list_node)
        {
            return_ptr = (void *)((uint8_t *)p_pre_block->p_next_free_block + MEM_BLOCK_SIZE);
            p_pre_block->p_next_free_block = p_block->p_next_free_block;

            if ((p_block->block_size - size) > BLOCK_ALLOC_SIZE_MIN)
            {
                p_new_block = (void *)((uint8_t *)p_block + size);
                p_new_block->block_size = p_block->block_size - size;
                p_block->block_size = size;

                free_block_node_insert(p_new_block);
            }

            s_curr_free_bytes -= p_block->block_size;

            if (s_curr_free_bytes < s_ever_free_bytes_min)
            {
                s_ever_free_bytes_min = s_curr_free_bytes;
            }

            p_block->block_size |= BLOCK_ALLOCATED_BIT;
            p_block->p_next_free_block = NULL;
        }
    }

    APP_MEM_UNLOCK();

    return return_ptr;
}


void app_free(void *ptr)
{
    app_mem_block_t *p_block;

    if (NULL == ptr)
    {
        return;
    }

    APP_MEM_LOCK();

    p_block = (app_mem_block_t *)((uint8_t *)ptr - MEM_BLOCK_SIZE);


    if (p_block->block_size & BLOCK_ALLOCATED_BIT)
    {
        if (NULL == p_block->p_next_free_block)
        {
            p_block->block_size &= ~BLOCK_ALLOCATED_BIT;

            s_curr_free_bytes += p_block->block_size;

            free_block_node_insert(p_block);
        }
    }

    APP_MEM_UNLOCK();
}

void *app_realloc(void *ptr, size_t size)
{
    app_mem_block_t *p_block;
    void            *p_realloc_ptr;
    size_t           block_size;
    size_t           copy_size;

    p_realloc_ptr = app_malloc(size);
    if (p_realloc_ptr)
    {
        APP_MEM_LOCK();
        p_block = (app_mem_block_t *)((uint8_t *)ptr - MEM_BLOCK_SIZE);
        block_size = (p_block->block_size & ~BLOCK_ALLOCATED_BIT);
        copy_size  = (block_size - MEM_BLOCK_SIZE) > size ? size : (block_size - MEM_BLOCK_SIZE);
        memcpy(p_realloc_ptr, (uint8_t *)ptr, copy_size);
        APP_MEM_UNLOCK();
        app_free(ptr);
        return p_realloc_ptr;
    }

    return NULL;
}

size_t app_mem_curr_free_size_get(void)
{
    return s_curr_free_bytes;
}


size_t app_mem_ever_free_min_size_get(void)
{
    return s_ever_free_bytes_min;
}




/**
 ****************************************************************************************
 *
 * @file app_memory.h
 *
 * @brief App Memory API
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

#ifndef __APP_MEMORY_H__
#define __APP_MEMORY_H__

#include "grx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup APP_MEMORY_MAROC Defines
 * @{
 */
#ifndef APP_MEM_ALIGN_NUM
#define APP_MEM_ALIGN_NUM           4                /**< ALIGN number: 4 byte. */
#endif

#ifndef APP_MEM_HEAP_SIZE
#define APP_MEM_HEAP_SIZE           (32 * 1024)      /**< Total app memory heap size. */
#endif

#define APP_MEM_LOCK()    GLOBAL_EXCEPTION_DISABLE() /**< App memory lock. */
#define APP_MEM_UNLOCK()  GLOBAL_EXCEPTION_ENABLE()  /**< App memory unlock. */
/** @} */


/**
 * @defgroup APP_MEMORY_STRUCT Structures
 * @{
 */
/**@brief App Memory Block Information */
typedef struct mem_block_info
{
    struct  mem_block_info  *p_next_free_block;     /**< Pointer to next free block. */
    size_t                   block_size;            /**< Size of block. */
} app_mem_block_t;

/** @} */

/**
 * @defgroup APP_MEMORY_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Malloc a block memory.
 *
 * @param[in] size: Size of memory needed to be alloced.
 *
 * @return Pointer to alloced memory.
 *****************************************************************************************
 */
void *app_malloc(size_t size);

/**
 *****************************************************************************************
 * @brief Realloc a block memory.
 *
 * @param[in] ptr:  Pointer to memory had been alloced.
 * @param[in] size: Size of memory needed to be realloced.
 *
 * @return Pointer to realloced memory.
 *****************************************************************************************
 */
void *app_realloc(void *ptr, size_t size);

/**
 *****************************************************************************************
 * @brief Free a block memory.
 *
 * @param[in] ptr: Pointer to memory needed to be free.
 *****************************************************************************************
 */
void app_free(void *ptr);

/**
 *****************************************************************************************
 * @brief Get current free size of app memory.
 *****************************************************************************************
 */
size_t app_mem_curr_free_size_get(void);

/**
 *****************************************************************************************
 * @brief Get ever min free size of app memory.
 *****************************************************************************************
 */
size_t app_mem_ever_free_min_size_get(void);
/** @} */

#endif



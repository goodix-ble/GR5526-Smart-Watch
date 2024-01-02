/**
 *****************************************************************************************
 *
 * @file app_graphics_mem.h
 *
 * @brief app graphics memory management API.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2022 GOODIX
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

#ifndef __APP_GRAPHICS_MEM_H__
#define __APP_GRAPHICS_MEM_H__

#include <stdint.h>
#include <stdbool.h>

/**
  * @defgroup  APP_GRAPHICS_MEM_MACRO Defines
  * @{
  */
/**
  * @brief APP_GRAPHIC_MEM maximum memory malloc number
  */
#define GFX_MAX_MEM_MALLOC_NB (100)
#if defined (__GNUC__) && !defined (__CC_ARM)
    #define GFX_MEM_BASE  (uint32_t)&__GFX_MEM_HEAP_BASE
    #define GFX_PSRAM_END (uint32_t)&__GFX_MEM_HEAP_END
    #define GFX_MEM_SIZE (GFX_PSRAM_END - GFX_MEM_BASE)
#elif defined (__CC_ARM)
    #define GFX_MEM_BASE ((uint32_t)&(Image$$GFX_MEM_HEAP$$Base))
    #define GFX_PSRAM_END (0x30880000UL)
    #define GFX_MEM_SIZE (GFX_PSRAM_END - GFX_MEM_BASE)
#endif
/** @} */

/* Exported variable --------------------------------------------------------*/
/** @defgroup APP_GRAPHIC_MEM_VARIABLE Variables
  * @{
  */
#if defined (__GNUC__) && !defined (__CC_ARM)
    extern  uint32_t __GFX_MEM_HEAP_BASE;
    extern  uint32_t __GFX_MEM_HEAP_END;
#elif defined (__CC_ARM)
    extern uint32_t Image$$GFX_MEM_HEAP$$Base; /* Graphics memory base address */
#endif
/** @} */

/**
 * @defgroup APP_GRAPHICS_MEM_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief  Create a graphics heap for mem allocate.
 *
 * @param[in] pool:        Pointer to the heap start address
 * @param[in] size:        Heap size in bytes
 *
 *****************************************************************************************
 */
void app_graphics_mem_init(uint8_t *pool, uint32_t size);

/**
 *****************************************************************************************
 * @brief  Allocate a graphics membuffer in bytes.
 *
 * @param[in] size:  Heap size in bytes
 *
 * @return the pointer of allocated mem buffer
 */
void *app_graphics_mem_malloc(uint32_t size);

/**
 *****************************************************************************************
 * @brief  Get the maximum address of all allocated mem buffer
 *
 * @return the maximum address of all allocated mem buffer
 */
uint32_t app_graphics_mem_max_alloc_addr_get(void);

/**
 *****************************************************************************************
 * @brief  Free an allocated graphics membuffer.
 *
 * @param[in] buf_ptr:  the pointer of allocated mem buffer
 *
 */
void app_graphics_mem_free(void *buf_ptr);

/** @} */

#endif

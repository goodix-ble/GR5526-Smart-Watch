/**
 ****************************************************************************************
 *
 * @file    graphics_sys_defs.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of Graphics library.
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

/** @addtogroup GRAPHICS_SDK Graphics
 *  @{
 */

/** @addtogroup GRAPHICS_PORTING Porting
  * @{
  */

/** @defgroup GRAPHICS_SYS_DEFS GPU Sys Defs
  * @brief GPU system configration defination
  * @{
  */

#ifndef __GRAPHICS_SYS_DEFS_H__
#define __GRAPHICS_SYS_DEFS_H__

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup GRAPHICS_SYS_DEFS_MACRO Defines
 * @{
 */
#define HAL_GFX_RING_BUFFER_SIZE            (10*1024u)                                       /**< The GPU RING BUFFER SIZE. */
#define HAL_GFX_MEM_POOL_ASSETS             0                                           /**< The same to Pool id */
#define HAL_GFX_MEM_POOL_FB                 0                                           /**< Pool id, only set to 0 currently */
#define VMEM_BASEADDR                       ((uint32_t)(&s_graphics_memory_buffer[0]))  /**< the graphics (video) memory base address. */

#ifndef VMEM_SIZE
    #define VMEM_SIZE                       (25*1024u) /**< The GPU max memory size for frame buffer. */
#endif

#ifndef USE_TSI_MALLOC
    #define USE_TSI_MALLOC                              /**< If enable, use the memory management of GPU. */
#endif

#ifndef HAL_GFX_MULTI_MEM_POOLS_CNT
    #define HAL_GFX_MULTI_MEM_POOLS_CNT     1           /**< if HAL_GFX_MULTI_MEM_POOLS is defined, use HAL_GFX_MULTI_MEM_POOLS_CNT pools must be equal or less than 4. */
#endif
/** @} */

#endif /*__GRAPHICS_SYS_DEFS_H__*/
/** @} */
/** @} */
/** @} */


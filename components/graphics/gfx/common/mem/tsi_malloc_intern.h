/**
 ****************************************************************************************
 *
 * @file    tsi_malloc_intern.h
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

/** @addtogroup GRAPHICS_COMMON Common
  * @{
  */

/** @defgroup MALLOC_INTERNAL GPU Internal Malloc
 * @brief graphics malloc. internal used
 * @{
 */

#ifndef TSI_MALLOC_INTERN_H__
#define TSI_MALLOC_INTERN_H__

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @defgroup MALLOC_INTERNAL_MACRO Defines
 * @{
 */

#ifndef MAX_MEM_POOLS
    #define MAX_MEM_POOLS 4                             /**< count of memory pool */
#endif // MAX_MEM_POOLS

#define ALIGNNUM  (16)                                  /**< align bytes */
#define ALIGNMASK (ALIGNNUM-1)                          /**< align mask */
#define ALIGN(s) ((((s)+ALIGNMASK)/ALIGNNUM)*ALIGNNUM)  /**< align address */

#define FLAG_EMPTY    0xf1fa1U                          /**< empty flag */
#define FLAG_NONEMPTY 0xf1fa2U                          /**< non-empty flag */

#define IS_LAST(c) ( (c)->next_offset == 0U )           /**< check is last or not */
#define OFFSET(c) ((uintptr_t)(c) - (uintptr_t)HEAD)    /**< offset from pool head */
/** @} */

/** @addtogroup MALLOC_INTERNAL_STRUCT Structure
  * @{
  */
  
/**
  * @brief  memory cell structure
  */
typedef struct cell {
    int         size;           /**< cell size */
    unsigned    flags;          /**< cell state flag */
    uintptr_t   next_offset;    /**< Next cell offset */
} cell_t;

/**
  * @brief  size of memory cell structure
  */
static const int cell_t_size = (ALIGN((int)sizeof(cell_t)));

/**
  * @brief  memory pool structure
  */
typedef struct pool {
    uintptr_t base_phys;            /**< base physical address for pool */
    uintptr_t base_virt;            /**< base virtual address for pool */
    uintptr_t end_virt;             /**< end virtual address for pool */
    cell_t   *head_of_empty_list;   /**< list pointer */
    int size;                       /**< pool size in bytes */
} pool_t;

/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */


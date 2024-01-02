/**
 ****************************************************************************************
 *
 * @file    tsi_malloc.h
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

/** @defgroup TSI_MALLOC  GPU Memory Interfaces
  * @brief GPU memory manage interfaces
  * @{
  */

#ifndef TSI_MALLOC_H__
#define TSI_MALLOC_H__


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup TSI_MALLOC_MACRO Definations
 * @{
 */

#define tsi_malloc_init(base_virt, base_phys, size, reset) \
        tsi_malloc_init_pool(0, base_virt, base_phys, size, reset)          /**< Tsi_malloc_init define */

#define tsi_malloc(size) tsi_malloc_pool(0, size)                           /**< Tsi_malloc define */

/** @} */

/**
 * @defgroup TSI_MALLOC_FUNCTION Functions
 * @{
 */

/**
 *****************************************************************************************
 * @brief Initial memory pool, command list memory malloc base on pool
 * @param[in] pool: Pool id, the value is fixed -- 0
 * @param[in] base_virt: Virtual addr, equel physical address
 * @param[in] base_phys: Physical address
 * @param[in] size: Pool memory size
 * @param[in] reset: If 0, reset the memory
 * @return Return Negative mean initial failure
 *****************************************************************************************
 */
int tsi_malloc_init_pool(int pool,
                               void *base_virt,
                               uintptr_t base_phys,
                               int size,
                               int reset);

/**
 *****************************************************************************************
 * @brief Malloc memory from pool
 * @param[in] pool: Pool id, the value is fixed -- 0
 * @param[in] size: Malloc size
 * @return Return the malloc memory addr
 *****************************************************************************************
 */
void *tsi_malloc_pool(int pool, int size);

/**
 *****************************************************************************************
 * @brief Free memory
 * @param[in] ptr: Memory addr
 *****************************************************************************************
 */
void  tsi_free(void *ptr);

/**
 *****************************************************************************************
 * @brief Free memory
 * @param[in] addr: Memory virtual addr, equel physical address
 * @return Return memory addr
 *****************************************************************************************
 */
uintptr_t tsi_virt2phys(void *addr);

/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */


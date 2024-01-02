/**
 ****************************************************************************************
 *
 * @file    hal_gfx_hal.h
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

/** @addtogroup HAL_GFX HAL GFX
  * @{
  */

/** @defgroup HAL_GFX_HAL GFX HAL
 * @brief GPU low layer interfaces, adaptation platform MCU.
 * @{
 */
#ifndef HAL_GFX_HAL_H__
#define HAL_GFX_HAL_H__

#include "hal_gfx_sys_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GFX_HAL_MACRO Defines
 * @{
 */
#define MUTEX_RB     0  /**< Mutex for ringbuffer */
#define MUTEX_MALLOC 1  /**< Mutex for malloc */
#define MUTEX_FLUSH  2  /**< Mutex for flush */
#define MUTEX_MAX    2  /**< MAX */
/** @} */

/**
 * @defgroup HAL_GFX_HAL_STRUCT Structures
 * @{
 */
/**@brief The base structure of gpu memory. */
typedef struct hal_gfx_buffer_t_
{
    int       size;                 /**< Size of buffer */
    int       fd;                   /**< File Descriptor of buffer */
    void     *base_virt;            /**< Virtual address of buffer */
    uintptr_t base_phys;            /**< Physical address of buffer */
} hal_gfx_buffer_t;

/**@brief The ringbuffer structure. */
typedef struct hal_gfx_ringbuffer_t_
{
    hal_gfx_buffer_t bo;                    /**< Memory base structure */
    int              offset;                /**< Record ringbuffer usage */
    int              last_submission_id;    /**< Latest command list id */
} hal_gfx_ringbuffer_t;

/** @} */

/**
 * @defgroup HAL_GFX_HAL_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize system. Implementor defined. Called in hal_gfx_init()
 *
 * @return 0 if no errors occurred
 *****************************************************************************************
 */
int32_t hal_gfx_sys_init(void);

/**
 *****************************************************************************************
 * @brief Wait for interrupt from the GPU
 *
 * @return 0 on success
 *****************************************************************************************
 */
int hal_gfx_wait_irq(void);

/**
 *****************************************************************************************
 * @brief Wait for a Command List to finish
 *
 * @param[in] cl_id: cl_id Command List ID
 *
 * @return 0 on success
 *****************************************************************************************
 */
int hal_gfx_wait_irq_cl(int cl_id);

/**
 *****************************************************************************************
 * @brief Wait for a Breakpoint
 *
 * @param[in] brk_id: Breakpoint ID
 *
 * @return 0 on success
 *****************************************************************************************
 */
int hal_gfx_wait_irq_brk(int brk_id);

/**
 *****************************************************************************************
 * @brief Read Hardware register
 *
 * @param[in] reg: Register to read
 *
 * @return Value read from the register
 *****************************************************************************************
 */
uint32_t hal_gfx_reg_read(uint32_t reg);

/**
 *****************************************************************************************
 * @brief Write Hardware Register
 *
 * @param[in] reg:   Register to write
 * @param[in] value: Value to be written
 *****************************************************************************************
 */
void hal_gfx_reg_write(uint32_t reg, uint32_t value);

/**
 *****************************************************************************************
 * @brief Create memory buffer
 *
 * @param[in] size: Size of buffer in bytes
 *
 * @return hal_gfx_buffer_t struct
 *****************************************************************************************
 */
hal_gfx_buffer_t hal_gfx_buffer_create(int size);

/**
 *****************************************************************************************
 * @brief Create memory buffer at a specific pool
 *
 * @param[in] pool: ID of the desired memory pool
 * @param[in] size: of buffer in bytes
 *
 * @return hal_gfx_buffer_t struct
 *****************************************************************************************
 */
hal_gfx_buffer_t hal_gfx_buffer_create_pool(int pool, int size);

/**
 *****************************************************************************************
 * @brief Maps buffer
 *
 * @param[in] bo: Pointer to buffer struct
 *
 * @return Virtual pointer of the buffer (same as in bo->base_virt)
 *****************************************************************************************
 */
void *hal_gfx_buffer_map(hal_gfx_buffer_t *bo);

/**
 *****************************************************************************************
 * @brief Unmaps buffer
 *
* @param[in] bo: Pointer to buffer struct
 *****************************************************************************************
 */
void hal_gfx_buffer_unmap(hal_gfx_buffer_t *bo);

/**
 *****************************************************************************************
 * @brief Destroy/deallocate buffer
 *
 * @param[in] bo: Pointer to buffer struct
 *****************************************************************************************
 */
void hal_gfx_buffer_destroy(hal_gfx_buffer_t *bo);

/**
 *****************************************************************************************
 * @brief Get physical (GPU) base address of a given buffer
 *
 * @param[in] bo: Pointer to buffer struct
 *
 * @return Physical base address of a given buffer
 *****************************************************************************************
 */
uintptr_t hal_gfx_buffer_phys(hal_gfx_buffer_t *bo);

/**
 *****************************************************************************************
 * @brief Write-back buffer from cache to main memory
 *
 * @param[in] bo: Pointer to buffer struct
 *****************************************************************************************
 */
void hal_gfx_buffer_flush(hal_gfx_buffer_t * bo);

/**
 *****************************************************************************************
 * @brief Allocate memory for CPU to use (typically, standard malloc() is called)
 *
 * @param[in] size: Size in bytes
 *
 * @return Pointer to allocated memory (virtual)
 *****************************************************************************************
 */
void *hal_gfx_host_malloc(size_t size);

/**
 *****************************************************************************************
 * @brief Free memory previously allocated with hal_gfx_host_malloc()
 *
 * @param[in] ptr: Pointer to allocated memory (virtual)
 *****************************************************************************************
 */
void  hal_gfx_host_free(void *ptr );

/**
 *****************************************************************************************
 * @brief Initialize Ring Buffer. Should be called from inside hal_gfx_sys_init().
 *   This is a private function, the user should never call it.
 *
 * @param[in] rb:    Pointer to hal_gfx_ring_buffer_t struct
 * @param[in] reset: Resets the Ring Buffer if non-zero
 *
 * @return Negative number on error
 *****************************************************************************************
 */
int hal_gfx_rb_init(hal_gfx_ringbuffer_t *rb, int reset);

/**
 *****************************************************************************************
 * @brief Mutex Lock for multiple processes/threads
 *
 * @param[in] mutex_id: MUTEX_RB or MUTEX_MALLOC
 *****************************************************************************************
 */
int hal_gfx_mutex_lock(int mutex_id);

/**
 *****************************************************************************************
 * @brief Mutex Unlock for multiple processes/threads
 *
 * @param[in] mutex_id: MUTEX_RB or MUTEX_MALLOC
 *****************************************************************************************
 */
int hal_gfx_mutex_unlock(int mutex_id);
/** @} */

#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */


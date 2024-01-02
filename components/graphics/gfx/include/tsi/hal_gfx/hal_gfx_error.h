/**
 ****************************************************************************************
 *
 * @file    hal_gfx_error.h
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

/** @defgroup HAL_GFX_ERROR GFX ERROR
 * @brief GPU error code define.
 * @{
 */
#ifndef HAL_GFX_ERROR_H__
#define HAL_GFX_ERROR_H__

#include "hal_gfx_sys_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GFX_ERROR_MACRO Defines
 * @{
 */
#define  HAL_GFX_ERR_NO_ERROR                     (0x00000000U) /**< No error has occured */
#define  HAL_GFX_ERR_SYS_INIT_FAILURE             (0x00000001U) /**< System initialization failure */
#define  HAL_GFX_ERR_GPU_ABSENT                   (0x00000002U) /**< Nema GPU is absent */
#define  HAL_GFX_ERR_RB_INIT_FAILURE              (0x00000004U) /**< Ring buffer initialization failure */
#define  HAL_GFX_ERR_NON_EXPANDABLE_CL_FULL       (0x00000008U) /**< Non expandable command list is full*/
#define  HAL_GFX_ERR_CL_EXPANSION                 (0x00000010U) /**< Command list expansion error */
#define  HAL_GFX_ERR_OUT_OF_GFX_MEMORY            (0x00000020U) /**< Graphics memory is full */
#define  HAL_GFX_ERR_OUT_OF_HOST_MEMORY           (0x00000040U) /**< Host memory is full */
#define  HAL_GFX_ERR_NO_BOUND_CL                  (0x00000080U) /**< There is no bound command list */
#define  HAL_GFX_ERR_NO_BOUND_FONT                (0x00000100U) /**< There is no bound font */
#define  HAL_GFX_ERR_GFX_MEMORY_INIT              (0x00000200U) /**< Graphics memory initialization failure */
#define  HAL_GFX_ERR_DRIVER_FAILURE               (0x00000400U) /**< Nema GPU Kernel Driver failure*/
#define  HAL_GFX_ERR_MUTEX_INIT                   (0x00000800U) /**< Mutex initialization failure*/
#define  HAL_GFX_ERR_INVALID_BO                   (0x00001000U) /**< Invalid buffer provided*/
#define  HAL_GFX_ERR_INVALID_CL                   (0x00002000U) /**< Invalid CL provided*/

/** @} */

/**
 * @defgroup HAL_GFX_ERROR_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Return Error Id
 *
 * @return 0 if no error exists
 *****************************************************************************************
 */
uint32_t hal_gfx_get_error(void);
/** @} */

#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */


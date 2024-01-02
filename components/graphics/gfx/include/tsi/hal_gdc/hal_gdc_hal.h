/**
 ****************************************************************************************
 *
 * @file    hal_gdc_hal.h
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

/** @addtogroup HAL_DC HAL DC
  * @{
  */

/** @defgroup HAL_GDC_HAL GDC HAL
  * @brief DC basic function definition.
  * @{
  */

#ifndef HAL_GDC_HAL_H__
#define HAL_GDC_HAL_H__

#include "hal_gfx_sys_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GDC_HAL_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize system. Implementor defined. Called in hal_gdc_init().
 *
 * @return 0 if no errors occurred
 *****************************************************************************************
 */
int32_t hal_gdc_sys_init(void);

/**
 *****************************************************************************************
 * @brief Wait for VSYNC.
 *****************************************************************************************
 */
void hal_gdc_wait_vsync(void);

/**
 *****************************************************************************************
 * @brief Read Hardware register.
 *
 * @param[in] reg: Register to read
 *
 * @return Value read from the register
 *****************************************************************************************
 */
uint32_t hal_gdc_reg_read(uint32_t reg);

/**
 *****************************************************************************************
 * @brief Write Hardware Register
 *
 * @param[in] reg: Register to write
 * @param[in] value: value to write
 *****************************************************************************************
 */
void hal_gdc_reg_write(uint32_t reg, uint32_t value);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */


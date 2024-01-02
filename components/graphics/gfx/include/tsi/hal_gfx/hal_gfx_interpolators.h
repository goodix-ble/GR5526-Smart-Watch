/**
 ****************************************************************************************
 *
 * @file    hal_gfx_interpolators.h
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

/** @defgroup HAL_GFX_INTERPOLATORS GFX INTERPOLATORS
 * @brief GPU interpolators.
 * @{
 */
#ifndef _HAL_GFX_INTERPOLATORS_H_
#define _HAL_GFX_INTERPOLATORS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_gfx_sys_defs.h"

/**
 * @defgroup HAL_GFX_INTERPOLATORS_STRUCT Structures
 * @{
 */
/**@brief Color structure. */
typedef struct _color_var_t
{
    float r; /**< Red */
    float g; /**< Green */
    float b; /**< Blue */
    float a; /**< Alpha */
} color_var_t;
/** @} */

/**
 * @defgroup HAL_GFX_INTERPOLATORS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Interpolate color gradient for rectangle
 *
 * @param[in] x0:   x coordinate of the upper left vertex of the rectangle
 * @param[in] y0:   y coordinate at the upper left vertex of the rectangle
 * @param[in] w:    width of the rectangle
 * @param[in] h:    height of the rectangle
 * @param[in] col0: color for the first vertex
 * @param[in] col1: color for the second vertex
 * @param[in] col2: color for the third vertex
 *****************************************************************************************
 */
void hal_gfx_interpolate_rect_colors(int x0, int y0, int w, int h, color_var_t* col0, color_var_t* col1, color_var_t* col2);

/**
 *****************************************************************************************
 * @brief Interpolate color gradient for triangle
 *
 * @param[in] x0:   x coordinate at the first vertex of the triangle
 * @param[in] y0:   y coordinate at the first vertex of the triangle
 * @param[in] x1:   x coordinate at the second vertex of the triangle
 * @param[in] y1:   y coordinate at the second vertex of the triangle
 * @param[in] x2:   x coordinate at the third vertex of the triangle
 * @param[in] y2:   y coordinate at the third vertex of the triangle
 * @param[in] col0: color for the first vertex
 * @param[in] col1: color for the second vertex
 * @param[in] col2: color for the third vertex
 *****************************************************************************************
 */
void hal_gfx_interpolate_tri_colors(float x0, float y0, float x1, float y1, float x2, float y2, color_var_t* col0, color_var_t* col1, color_var_t* col2);

/**
 *****************************************************************************************
 * @brief Interpolate depth buffer values for triangle
 *
 * @param[in] x0: coordinate at the first vertex of the triangle
 * @param[in] y0: coordinate at the first vertex of the triangle
 * @param[in] z0: coordinate at the first vertex of the triangle
 * @param[in] x1: coordinate at the second vertex of the triangle
 * @param[in] y1: coordinate at the second vertex of the triangle
 * @param[in] z1: coordinate at the second vertex of the triangle
 * @param[in] x2: coordinate at the third vertex of the triangle
 * @param[in] y2: coordinate at the third vertex of the triangle
 * @param[in] z2: coordinate at the third vertex of the triangle
 *****************************************************************************************
 */
void hal_gfx_interpolate_tri_depth(float x0, float y0, float z0, float x1, float y1, float z1, float x2, float y2, float z2);

/**
 *****************************************************************************************
 * @brief Interpolate texture values for triangle
 *
 * @param[in] x0:         x coordinate at the first vertex of the triangle
 * @param[in] y0:         y coordinate at the first vertex of the triangle
 * @param[in] w0:         w coordinate at the first vertex of the triangle
 * @param[in] tx0:        x texture coordinate at the first vertex of the triangle
 * @param[in] ty0:        y texture coordinate at the first vertex of the triangle
 * @param[in] x1:         x coordinate at the second vertex of the triangle
 * @param[in] y1:         y coordinate at the second vertex of the triangle
 * @param[in] w1:         w coordinate at the second vertex of the triangle
 * @param[in] tx1:        x texture coordinate at the second vertex of the triangle
 * @param[in] ty1:        y texture coordinate at the second vertex of the triangle
 * @param[in] x2:         x coordinate at the third vertex of the triangle
 * @param[in] y2:         y coordinate at the third vertex of the triangle
 * @param[in] w2:         w coordinate at the third vertex of the triangle
 * @param[in] tx2:        x texture coordinate at the third vertex of the triangle
 * @param[in] ty2:        x texture coordinate at the third vertex of the triangle
 * @param[in] tex_width:  texture width
 * @param[in] tex_height: texture height
 *****************************************************************************************
 */
void hal_gfx_interpolate_tx_ty(float x0, float y0, float w0, float tx0, float ty0,
                            float x1, float y1, float w1, float tx1, float ty1,
                            float x2, float y2, float w2, float tx2, float ty2,
                            int tex_width, int tex_height );
/** @} */
#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */


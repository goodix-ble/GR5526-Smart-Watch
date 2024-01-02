/**
 ****************************************************************************************
 *
 * @file    hal_gfx_raster.h
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

/** @defgroup HAL_GFX_RASTER GFX RASTER
  * @brief GPU raster interfaces.
  * @{
  */

#ifndef HAL_GFX_RASTER_H__
#define HAL_GFX_RASTER_H__

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @defgroup HAL_GFX_RASTER_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Setting the raster color
 *
 * @param[in] rgba8888: Color value
 *****************************************************************************************
 */
void hal_gfx_set_raster_color(uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Raster a pixel
 *
 * @param[in] x: x coordinate of the pixel
 * @param[in] y: x coordinate of the pixel
 *****************************************************************************************
 */
void hal_gfx_raster_pixel(int x, int y);

/**
 *****************************************************************************************
 * @brief Raster a line
 *
 * @param[in] x0: x coordinate at the beginning of the line
 * @param[in] y0: y coordinate at the beginning of the line
 * @param[in] x1: x coordinate at the end of the line
 * @param[in] y1: y coordinate at the end of the line
 *****************************************************************************************
 */
void hal_gfx_raster_line(int x0, int y0, int x1, int y1);

/**
 *****************************************************************************************
 * @brief Raster a triangle with fixed point(16.16)
 *
 * @param[in] x0fx: x coordinate at the first vertex of the triangle
 * @param[in] y0fx: y coordinate at the first vertex of the triangle
 * @param[in] x1fx: x coordinate at the second vertex of the triangle
 * @param[in] y1fx: y coordinate at the second vertex of the triangle
 * @param[in] x2fx: x coordinate at the third vertex of the triangle
 * @param[in] y2fx: y coordinate at the third vertex of the triangle
 *****************************************************************************************
 */
void hal_gfx_raster_triangle_fx(int x0fx, int y0fx, int x1fx, int y1fx, int x2fx, int y2fx);

/**
 *****************************************************************************************
 * @brief Raster a rectangle
 *
 * @param[in] x: x coordinate of the upper left vertex of the rectangle
 * @param[in] y: y coordinate of the upper left vertex of the rectangle
 * @param[in] w: width of the rectangle
 * @param[in] h: height of the rectangle
 *****************************************************************************************
 */
void hal_gfx_raster_rect(int x, int y, int w, int h);

/**
 *****************************************************************************************
 * @brief Raster a rectangle with rounded edges
 *
 * @param[in] x0: x coordinate of the upper left vertex of the rectangle
 * @param[in] y0: y coordinate of the upper left vertex of the rectangle
 * @param[in] w: width of the rectangle
 * @param[in] h: height of the rectangle
 * @param[in] r: corner radius
 *****************************************************************************************
 */
void hal_gfx_raster_rounded_rect(int x0, int y0, int w, int h, int r);

/**
 *****************************************************************************************
 * @brief Raster a quad with fixed point(16.16)
 *
 * @param[in] x0fx: x coordinate at the first vertex of the quadrilateral
 * @param[in] y0fx: y coordinate at the first vertex of the quadrilateral
 * @param[in] x1fx: x coordinate at the second vertex of the quadrilateral
 * @param[in] y1fx: y coordinate at the second vertex of the quadrilateral
 * @param[in] x2fx: x coordinate at the third vertex of the quadrilateral
 * @param[in] y2fx: y coordinate at the third vertex of the quadrilateral
 * @param[in] x3fx: x coordinate at the fourth vertex of the quadrilateral
 * @param[in] y3fx: y coordinate at the fourth vertex of the quadrilateral
 *****************************************************************************************
 */
void hal_gfx_raster_quad_fx(int x0fx, int y0fx, int x1fx, int y1fx, int x2fx, int y2fx, int x3fx, int y3fx);

/**
 *****************************************************************************************
 * @brief Raster a triangle
 *
 * @param[in] x0: x coordinate at the first vertex of the triangle
 * @param[in] y0: y coordinate at the first vertex of the triangle
 * @param[in] x1: x coordinate at the second vertex of the triangle
 * @param[in] y1: y coordinate at the second vertex of the triangle
 * @param[in] x2: x coordinate at the third vertex of the triangle
 * @param[in] y2: y coordinate at the third vertex of the triangle
 *****************************************************************************************
 */
void hal_gfx_raster_triangle (int x0, int y0, int x1, int y1, int x2, int y2);

/**
 *****************************************************************************************
 * @brief Raster a quad
 *
 * @param[in] x0: x coordinate at the first vertex of the quadrilateral
 * @param[in] y0: y coordinate at the first vertex of the quadrilateral
 * @param[in] x1: x coordinate at the second vertex of the quadrilateral
 * @param[in] y1: y coordinate at the second vertex of the quadrilateral
 * @param[in] x2: x coordinate at the third vertex of the quadrilateral
 * @param[in] y2: y coordinate at the third vertex of the quadrilateral
 * @param[in] x3: x coordinate at the fourth vertex of the quadrilateral
 * @param[in] y3: y coordinate at the fourth vertex of the quadrilateral
 *****************************************************************************************
 */
void hal_gfx_raster_quad (int x0,int y0,int x1,int y1,int x2,int y2,int x3,int y3);

/**
 *****************************************************************************************
 * @brief Raster a circle with Anti-Aliasing (if available)
 *
 * @param[in] x: x coordinate of the circle's center
 * @param[in] y: y coordinate of the circle's center
 * @param[in] r: circle's radius
 *****************************************************************************************
 */
void hal_gfx_raster_circle_aa(float x, float y, float r);

/**
 *****************************************************************************************
 * @brief Raster a circle with Anti-Aliasing (if available) and specified width
 *
 * @param[in] x: x coordinate of the circle's center
 * @param[in] y: y coordinate of the circle's center
 * @param[in] r: circle's radius
 * @param[in] w: pencil width
 *****************************************************************************************
 */
void hal_gfx_raster_stroked_circle_aa(float x, float y, float r, float w);

/**
 *****************************************************************************************
 * @brief Raster a rectangle with fixed point(16.16)
 *
 * @param[in] x: x coordinate of the upper left vertex of the rectangle
 * @param[in] y: y coordinate of the upper left vertex of the rectangle
 * @param[in] w: width of the rectangle
 * @param[in] h: height of the rectangle
 *****************************************************************************************
 */
void hal_gfx_raster_rect_fx(int x, int y, int w, int h);

/**
 *****************************************************************************************
 * @brief Raster a rectangle. (float coordinates)
 *
 * @param[in] x: x coordinate of the upper left vertex of the rectangle
 * @param[in] y: y coordinate of the upper left vertex of the rectangle
 * @param[in] w: width of the rectangle
 * @param[in] h: height of the rectangle
 *****************************************************************************************
 */
void hal_gfx_raster_rect_f(float x, float y, float w, float h);

/**
 *****************************************************************************************
 * @brief Raster a triangle. (float coordinates)
 *
 * @param[in] x0: x coordinate at the first vertex of the triangle
 * @param[in] y0: y coordinate at the first vertex of the triangle
 * @param[in] x1: x coordinate at the second vertex of the triangle
 * @param[in] y1: y coordinate at the second vertex of the triangle
 * @param[in] x2: x coordinate at the third vertex of the triangle
 * @param[in] y2: y coordinate at the third vertex of the triangle
 *****************************************************************************************
 */
void hal_gfx_raster_triangle_f(float x0, float y0, float x1, float y1, float x2, float y2);

/**
 *****************************************************************************************
 * @brief Raster the first vertex of the triangle strip. (float coordinates)
 *
 * @param[in] x0: x coordinate at the first vertex of the triangle
 * @param[in] y0: y coordinate at the first vertex of the triangle
 *****************************************************************************************
 */
void hal_gfx_raster_triangle_p0_f(float x0, float y0);

/**
 *****************************************************************************************
 * @brief Raster the second vertex of the triangle strip. (float coordinates)
 *
 * @param[in] x1: x coordinate at the second vertex of the triangle
 * @param[in] y1: y coordinate at the second vertex of the triangle
 *****************************************************************************************
 */
void hal_gfx_raster_triangle_p1_f(float x1, float y1);

/**
 *****************************************************************************************
 * @brief Raster the third vertex of the triangle strip. (float coordinates)
 *
 * @param[in] x2: x coordinate at the third vertex of the triangle
 * @param[in] y2: y coordinate at the third vertex of the triangle
 *****************************************************************************************
 */
void hal_gfx_raster_triangle_p2_f(float x2, float y2);

/**
 *****************************************************************************************
 * @brief Raster a quad. (float coordinates)
 *
 * @param[in] x0: x coordinate at the first vertex of the quadrilateral
 * @param[in] y0: y coordinate at the first vertex of the quadrilateral
 * @param[in] x1: x coordinate at the second vertex of the quadrilateral
 * @param[in] y1: y coordinate at the second vertex of the quadrilateral
 * @param[in] x2: x coordinate at the third vertex of the quadrilateral
 * @param[in] y2: y coordinate at the third vertex of the quadrilateral
 * @param[in] x3: x coordinate at the fourth vertex of the quadrilateral
 * @param[in] y3: y coordinate at the fourth vertex of the quadrilateral
 *****************************************************************************************
 */
void hal_gfx_raster_quad_f(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3);
/** @} */
#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */


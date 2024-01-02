/**
 ****************************************************************************************
 *
 * @file    hal_gfx_matrix3x3.h
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

/** @defgroup HAL_GFX_MATRIX3X3 GFX MATRIX3X3
 * @brief Base matrix3x3 Operation.
 * @{
 */

#ifndef HAL_GFX_MATRIX3X3_H__
#define HAL_GFX_MATRIX3X3_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GFX_MATRIX3X3_TYPEDEF Typedefs
 * @{
 */
typedef float hal_gfx_matrix3x3_t[3][3];   /**< Global define matrix3x3 variable. */
/** @} */

/**
 * @defgroup HAL_GFX_MATRIX3X3_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Load Identity Matrix
 *
 * @param[in] m: Matrix to be loaded
 *****************************************************************************************
 */
void hal_gfx_mat3x3_load_identity(hal_gfx_matrix3x3_t m);

/**
 *****************************************************************************************
 * @brief Apply translate transformation
 *
 * @param[in] m:  Matrix to apply transformation
 * @param[in] tx: X translation factor
 * @param[in] ty: Y translation factor
 *****************************************************************************************
 */
void hal_gfx_mat3x3_translate   (hal_gfx_matrix3x3_t m, float tx, float ty);

/**
 *****************************************************************************************
 * @brief Apply scale transformation
 *
 * @param[in] m:  Matrix to apply transformation
 * @param[in] sx: X scaling factor
 * @param[in] sy: Y scaling factor
 *****************************************************************************************
 */
void hal_gfx_mat3x3_scale       (hal_gfx_matrix3x3_t m, float sx, float sy);

/**
 *****************************************************************************************
 * @brief Apply shear transformation
 *
 * @param[in] m:   Matrix to apply transformation
 * @param[in] shx: X shearing factor
 * @param[in] shy: Y shearing factor
 *****************************************************************************************
 */
void hal_gfx_mat3x3_shear       (hal_gfx_matrix3x3_t m, float shx, float shy);

/**
 *****************************************************************************************
 * @brief Apply mirror transformation
 *
 * @param[in] m:  Matrix to apply transformation
 * @param[in] mx: if non-zero, mirror horizontally
 * @param[in] my: if non-zero, mirror vertically
 *****************************************************************************************
 */
void hal_gfx_mat3x3_mirror      (hal_gfx_matrix3x3_t m, int mx, int my);

/**
 *****************************************************************************************
 * @brief Apply rotation transformation
 *
 * @param[in] m:             Matrix to apply transformation
 * @param[in] angle_degrees: Angle to rotate in degrees
 *****************************************************************************************
 */
void hal_gfx_mat3x3_rotate      (hal_gfx_matrix3x3_t m, float angle_degrees);

/**
 *****************************************************************************************
 * @brief Multiply two 3x3 matrices ( m = m*_m)
 *
 * @param[in] m: left matrix, will be overwritten by the result
 * @param[in] _m: right matrix
 *****************************************************************************************
 */
void hal_gfx_mat3x3_mul(hal_gfx_matrix3x3_t m, hal_gfx_matrix3x3_t _m);

/**
 *****************************************************************************************
 * @brief Multiply vector with matrix
 *
 * @param[in] m: Matrix to multiply with
 * @param[in] x: Vector x coefficient
 * @param[in] y: Vector y coefficient
 *****************************************************************************************
 */
void hal_gfx_mat3x3_mul_vec(hal_gfx_matrix3x3_t m, float *x, float *y);

/**
 *****************************************************************************************
 * @brief Multiply vector with affine matrix
 *
 * @param[in] m: Matrix to multiply with
 * @param[in] x: Vector x coefficient
 * @param[in] y: Vector y coefficient
 *****************************************************************************************
 */
void hal_gfx_mat3x3_mul_vec_affine(hal_gfx_matrix3x3_t m, float *x, float *y);

/**
 *****************************************************************************************
 * @brief Calculate adjoint
 *
 * @param[in] m: Matrix
 *****************************************************************************************
 */
void hal_gfx_mat3x3_adj(hal_gfx_matrix3x3_t m);

/**
 *****************************************************************************************
 * @brief Divide matrix with scalar value
 *
 * @param[in] m: Matrix to divide
 * @param[in] s: scalar value
 *****************************************************************************************
 */
void hal_gfx_mat3x3_div_scalar(hal_gfx_matrix3x3_t m, float s);

/**
 *****************************************************************************************
 * @brief Invert matrix
 *
 * @param[in] m: Matrix to invert
 *****************************************************************************************
 */
int hal_gfx_mat3x3_invert(hal_gfx_matrix3x3_t m);

/**
 *****************************************************************************************
 * @brief Square to quad transformation
 *
 * @param[in] dx0: x coordinate at the first vertex of the quadrilateral
 * @param[in] dy0: y coordinate at the first vertex of the quadrilateral
 * @param[in] dx1: x coordinate at the second vertex of the quadrilateral
 * @param[in] dy1: y coordinate at the second vertex of the quadrilateral
 * @param[in] dx2: x coordinate at the third vertex of the quadrilateral
 * @param[in] dy2: y coordinate at the third vertex of the quadrilateral
 * @param[in] dx3: x coordinate at the fourth vertex of the quadrilateral
 * @param[in] dy3: y coordinate at the fourth vertex of the quadrilateral
 * @param[in] m:   Store converted matrix
 *****************************************************************************************
 */
int hal_gfx_mat3x3_square_to_quad(float dx0, float dy0,
                             float dx1, float dy1,
                             float dx2, float dy2,
                             float dx3, float dy3,
                             hal_gfx_matrix3x3_t m);

/**
 *****************************************************************************************
 * @brief Map rectangle to quadrilateral
 *
 * @param[in] width:  Rectangle width
 * @param[in] height: Rectangle height
 * @param[in] sx0:    x coordinate at the first vertex of the quadrilateral
 * @param[in] sy0:    y coordinate at the first vertex of the quadrilateral
 * @param[in] sx1:    x coordinate at the second vertex of the quadrilateral
 * @param[in] sy1:    y coordinate at the second vertex of the quadrilateral
 * @param[in] sx2:    x coordinate at the third vertex of the quadrilateral
 * @param[in] sy2:    y coordinate at the third vertex of the quadrilateral
 * @param[in] sx3:    x coordinate at the fourth vertex of the quadrilateral
 * @param[in] sy3:    y coordinate at the fourth vertex of the quadrilateral
 * @param[in] m:      Mapping matrix
 *****************************************************************************************
 */
int hal_gfx_mat3x3_quad_to_rect(int width, int height,
                           float sx0, float sy0,
                           float sx1, float sy1,
                           float sx2, float sy2,
                           float sx3, float sy3,
                           hal_gfx_matrix3x3_t m);
/** @} */
#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */


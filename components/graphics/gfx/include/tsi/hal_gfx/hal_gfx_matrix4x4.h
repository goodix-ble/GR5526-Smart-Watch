/**
 ****************************************************************************************
 *
 * @file    hal_gfx_matrix4x4.h
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

/** @defgroup HAL_GFX_MATRIX4X4 GFX MATRIX4X4
 * @brief Base matrix4x4 Operation.
 * @{
 */


#ifndef HAL_GFX_MATRIX4X4_H__
#define HAL_GFX_MATRIX4X4_H__

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @defgroup HAL_GFX_MATRIX4X4_TYPEDEF Typedefs
 * @{
 */
typedef float hal_gfx_matrix4x4_t[4][4];/**< Global define matrix4x4 variable. */
/** @} */

/**
 * @defgroup HAL_GFX_MATRIX4X4_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Load a 4x4 Identity Matrix
 *
 * @param[in] m: Matrix to be loaded
 *
 *****************************************************************************************
 */
void hal_gfx_mat4x4_load_identity(hal_gfx_matrix4x4_t m);

/**
 *****************************************************************************************
 * @brief Multiply two 4x4 matrices
 *
 * @param[in] m:   Result Matrix
 * @param[in] m_l: Left operand
 * @param[in] m_r: Right operand
 *
 *****************************************************************************************
 */
void hal_gfx_mat4x4_mul(hal_gfx_matrix4x4_t  m,
                     hal_gfx_matrix4x4_t  m_l,
                     hal_gfx_matrix4x4_t  m_r);

/**
 *****************************************************************************************
 * @brief Multiply a 4x1 vector with a 4x4 matrix
 *
 * @param[in] m: Matrix to be multiplied
 * @param[in] x: Vector first element
 * @param[in] y: Vector second element
 * @param[in] z: Vector third element
 * @param[in] w: Vector forth element
 *
 *****************************************************************************************
 */
void hal_gfx_mat4x4_mul_vec(hal_gfx_matrix4x4_t m, float *x, float *y, float *z, float *w);

// ------------------------------------------------------------------------------------
// Object Transformation - ModelView Matrix
// Object Coordinates to Eye Coordinates
// ------------------------------------------------------------------------------------
/**
 *****************************************************************************************
 * @brief Apply translate transformation
 *
 * @param[in] m:  Matrix to apply transformation
 * @param[in] tx: X translation factor
 * @param[in] ty: Y translation factor
 * @param[in] tz: Z translation factor
 *
 *****************************************************************************************
 */
void hal_gfx_mat4x4_translate(hal_gfx_matrix4x4_t m, float tx, float ty, float tz);

/**
 *****************************************************************************************
 * @brief Apply scale transformation
 *
 * @param[in] m:  Matrix to apply transformation
 * @param[in] sx: X scaling factor
 * @param[in] sy: Y scaling factor
 * @param[in] sz: Z scaling factor
 *
 *****************************************************************************************
 */
void hal_gfx_mat4x4_scale(hal_gfx_matrix4x4_t m, float sx, float sy, float sz);

/**
 *****************************************************************************************
 * @brief Apply rotate transformation around X axis
 *
 * @param[in] m:             Matrix to apply transformation
 * @param[in] angle_degrees: Angle to rotate in degrees
 *
 *****************************************************************************************
 */
void hal_gfx_mat4x4_rotate_X    (hal_gfx_matrix4x4_t m, float angle_degrees);

/**
 *****************************************************************************************
 * @brief Apply rotate transformation around Y axis
 *
 * @param[in] m:             Matrix to apply transformation
 * @param[in] angle_degrees: Angle to rotate in degrees
 *
 *****************************************************************************************
 */
void hal_gfx_mat4x4_rotate_Y    (hal_gfx_matrix4x4_t m, float angle_degrees);

/**
 *****************************************************************************************
 * @brief Apply rotate transformation around Z axis
 *
 * @param[in] m:             Matrix to apply transformation
 * @param[in] angle_degrees: Angle to rotate in degrees
 *
 *****************************************************************************************
 */
void hal_gfx_mat4x4_rotate_Z    (hal_gfx_matrix4x4_t m, float angle_degrees);

// ------------------------------------------------------------------------------------
// Scene Transformation/Frustum - Projection Matrix
// Eye Coordinates to Clip Coordinates
// ------------------------------------------------------------------------------------
/**
 *****************************************************************************************
 * @brief Set up a perspective projection matrix
 *
 * @param[in] m:            A 4x4 Matrix
 * @param[in] fovy_degrees: Field of View in degrees
 * @param[in] aspect:       Aspect ratio that determines the field of view in the x direction.
 * @param[in] nearVal:      Distance from the viewer to the near clipping plane (always positive)
 * @param[in] farVal:       Distance from the viewer to the far clipping plane (always positive)
 *
 *****************************************************************************************
 */
void hal_gfx_mat4x4_load_perspective(hal_gfx_matrix4x4_t m, float fovy_degrees, float aspect,
                                  float nearVal, float farVal);

/**
 *****************************************************************************************
 * @brief Set up an orthographic projection matrix
 *
 * @param[in] m:       A 4x4 Matrix
 * @param[in] left:    Left vertical clipping plane
 * @param[in] right:   Right vertical clipping plane
 * @param[in] bottom:  bottom horizontal clipping plane
 * @param[in] top:     Top horizontal clipping plane
 * @param[in] nearVal: Distance from the viewer to the near clipping plane (always positive)
 * @param[in] farVal:  Distance from the viewer to the far clipping plane (always positive)
 *
 *****************************************************************************************
 */
void hal_gfx_mat4x4_load_ortho(hal_gfx_matrix4x4_t m,
                            float left,    float right,
                            float bottom,  float top,
                            float nearVal, float farVal);

/**
 *****************************************************************************************
 * @brief Set up a 2D orthographic projection matrix
 *
 * @param[in] m:      A 4x4 Matrix
 * @param[in] left:   Left vertical clipping plane
 * @param[in] right:  Right vertical clipping plane
 * @param[in] bottom: bottom horizontal clipping plane
 * @param[in] top:    Top horizontal clipping plane
 *
 *****************************************************************************************
 */
void hal_gfx_mat4x4_load_ortho_2d(hal_gfx_matrix4x4_t m,
                               float left,   float right,
                               float bottom, float top);

// ------------------------------------------------------------------------------------
// Clip Coordinates to Window Coordinates
// ------------------------------------------------------------------------------------
/**
 *****************************************************************************************
 * @brief Convenience Function to calculate window coordinates from object coordinates
 *
 * @param[in] mvp:     Model, View and Projection Matrix
 * @param[in] x_orig:  Window top left X coordinate
 * @param[in] y_orig:  Window top left Y coordinate
 * @param[in] width:   Window width
 * @param[in] height:  Window height
 * @param[in] nearVal: Distance from the viewer to the near clipping plane (always positive)
 * @param[in] farVal:  Distance from the viewer to the far clipping plane (always positive)
 * @param[in] x:       X object coordinate
 * @param[in] y:       Y object coordinate
 * @param[in] z:       Z object coordinate
 * @param[in] w:       W object coordinate

 * @return 1 if vertex is outside frustum (should be clipped)
 *****************************************************************************************
 */
int hal_gfx_mat4x4_obj_to_win_coords(hal_gfx_matrix4x4_t mvp,
                                   float x_orig,  float y_orig,
                                   int width, int height,
                                   float nearVal, float farVal,
                                   float *x,
                                   float *y,
                                   float *z,
                                   float *w);
/** @} */

#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */


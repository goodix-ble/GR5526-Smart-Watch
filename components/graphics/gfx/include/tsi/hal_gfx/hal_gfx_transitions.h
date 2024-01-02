/**
 ****************************************************************************************
 *
 * @file    hal_gfx_transitions.h
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

/** @defgroup HAL_GFX_TRANSITIONS GFX_TRANSITIONS
 * @brief GPU transitions.
 * @{
 */

#ifndef HAL_GFX_TRANSITIONS_H__
#define HAL_GFX_TRANSITIONS_H__

#include "hal_gfx_blender.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GFX_TRANSITIONS_ENUM Enumerations
 * @{
 */
/**@brief Definition of transition effect. */
typedef enum {
    HAL_GFX_TRANS_LINEAR_H,     /**< Linear transition horizontally. */
    HAL_GFX_TRANS_CUBE_H,       /**< Cubic transition horizontally. */
    HAL_GFX_TRANS_INNERCUBE_H,  /**< Inner Cube transition horizontally. */
    HAL_GFX_TRANS_STACK_H,      /**< Stack transition horizontally. */
    HAL_GFX_TRANS_LINEAR_V,     /**< Linear transition vertically. */
    HAL_GFX_TRANS_CUBE_V,       /**< Cubic transition vertically. */
    HAL_GFX_TRANS_INNERCUBE_V,  /**< Inner Cube transition vertically. */
    HAL_GFX_TRANS_STACK_V,      /**< Stack transition vertically. */
    HAL_GFX_TRANS_FADE,         /**< Fade transition. */
    HAL_GFX_TRANS_FADE_ZOOM,    /**< Fade-zoom transition. */
    HAL_GFX_TRANS_COVER,        /**< Cover transition. */
    HAL_GFX_TRANS_SPIN_H_R2L,   /**< Spin from Right to Left(ClockWise) horizontally */
    HAL_GFX_TRANS_SPIN_H_L2R,   /**< Spin from Left to Right(Counter-ClockWise) horizontally */
    HAL_GFX_TRANS_PUSHPULL_H,   /**< Linear transition with a depth of field effect, horizontal.*/
    HAL_GFX_TRANS_PUSHPULL_V,   /**< Linear transition with a depth of field effect, horizontal.*/
    HAL_GFX_TRANS_MAX,          /**< MAX. */
    HAL_GFX_TRANS_NONE,         /**< NONE. */
} hal_gfx_transition_t;
/** @} */

/**
 * @defgroup HAL_GFX_TRANSITIONS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Transition from 'initial' texture to 'final' texture. The transition is complete when 'step' is 0 or 1
 *
 * @param[in] effect:        Transition effect
 * @param[in] initial:       Initial texture
 * @param[in] final:         Final texture
 * @param[in] blending_mode: Blending mode
 * @param[in] step:          Transition step within [0.f , 1.f] range
 * @param[in] width:         Texture width
 * @param[in] height:        Texture height
 *
 */
void hal_gfx_transition(hal_gfx_transition_t effect, hal_gfx_tex_t initial, hal_gfx_tex_t final,
                    uint32_t blending_mode, float step, int width, int height);

/**
 *****************************************************************************************
 * @brief Linear transition horizontally. When 'step' changes from zero to one, textures move from right to left,
 * otherwise textures move from left to right. The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] left:          Texture on the left side
 * @param[in] right:         Texture on the right side
 * @param[in] blending_mode: Blending mode
 * @param[in] step:          Current step within [0.f , 1.f] range
 * @param[in] width:         Texture width
 *
 *****************************************************************************************
 */
void hal_gfx_transition_linear_hor(hal_gfx_tex_t left, hal_gfx_tex_t right,
                                uint32_t blending_mode, float step, int width);

/**
 *****************************************************************************************
 * @brief Linear transition vertically. When 'step' changes from zero to one, textures move from top to bottom,
 * otherwise textures move from bottom to top. The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] up:            Texture on the top side
 * @param[in] down:          Texture on the bottom side
 * @param[in] blending_mode: Blending mode
 * @param[in] step:          Current step within [0.f , 1.f] range
 * @param[in] height:        Texture height
 *
 *****************************************************************************************
 */
void hal_gfx_transition_linear_ver(hal_gfx_tex_t up, hal_gfx_tex_t down,
                                uint32_t blending_mode, float step, int height);

/**
 *****************************************************************************************
 * @brief Cubic (textures are mapped on the external faces of a cube) transition horizontally. When 'step' changes from zero to one, textures move from left to right,
 * otherwise textures move from right to left. The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] left:          Texture on the left side
 * @param[in] right:         Texture on the right side
 * @param[in] blending_mode; Blending mode
 * @param[in] step:          Current step within [0.f , 1.f] range
 * @param[in] width:         Texture width
 * @param[in] height:        Texture height
 *
 *****************************************************************************************
 */
void hal_gfx_transition_cube_hor(hal_gfx_tex_t left, hal_gfx_tex_t right,
                              uint32_t blending_mode, float step, int width, int height);

/**
 *****************************************************************************************
 * @brief Cube (textures are mapped on the external faces of a cube) transition vertically. When 'step' changes from zero to one, textures move from top to bottom,
 * otherwise textures move from bottom to top. The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] up:            Texture on the top side
 * @param[in] down:          Texture on the bottom side
 * @param[in] blending_mode: Blending mode
 * @param[in] step:          Current step within [0.f , 1.f] range
 * @param[in] width:         Texture width
 * @param[in] height:        Texture height
 *
 *****************************************************************************************
 */
void hal_gfx_transition_cube_ver(hal_gfx_tex_t up, hal_gfx_tex_t down,
                              uint32_t blending_mode, float step, int width, int height);

/**
 *****************************************************************************************
 * @brief Inner Cube (textures are mapped on the internal faces of a cube) transition horizontally. When 'step' changes from zero to one, textures move from left to right,
 * otherwise textures move from right to left. The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] left:          Texture on the left side
 * @param[in] right:         Texture on the right side
 * @param[in] blending_mode: Blending mode
 * @param[in] step:          Current step within [0.f , 1.f] range
 * @param[in] width:         Texture width
 * @param[in] height:        Texture height
 *
 *****************************************************************************************
 */
void hal_gfx_transition_innercube_hor(hal_gfx_tex_t left, hal_gfx_tex_t right,
                                   uint32_t blending_mode, float step, int width, int height);

/**
 *****************************************************************************************
 * @brief Inner Cube (textures are mapped on the internal faces of a cube) transition vertically. When 'step' changes from zero to one, textures move from top to bottom,
 * otherwise textures move from bottom to top. The transition The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] up:            Texture on the top side
 * @param[in] down:          Texture on the bottom side
 * @param[in] blending_mode: Blending mode
 * @param[in] step:          Current step within [0.f , 1.f] range
 * @param[in] width:         Texture width
 * @param[in] height:        Texture height
 *
 *****************************************************************************************
 */
void hal_gfx_transition_innercube_ver(hal_gfx_tex_t up, hal_gfx_tex_t down,
                                   uint32_t blending_mode, float step, int width, int height);

/**
 *****************************************************************************************
 * @brief Stack transition horizontally. When 'step' changes from zero to one, textures move from left to right,
 * otherwise textures move from right to left. The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] left:          Texture on the left side
 * @param[in] right:         Texture on the right side
 * @param[in] step:          Current step within [0.f , 1.f] range
 * @param[in] width:         Texture width
 * @param[in] height:        Texture height
 *
 *****************************************************************************************
 */
void hal_gfx_transition_stack_hor(hal_gfx_tex_t left, hal_gfx_tex_t right, float step,
                                int width, int height);

/**
 *****************************************************************************************
 * @brief Stack transition vertically. When 'step' moves from zero to one, textures move from top to bottom,
 * otherwise textures move from bottom to top. The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] up:            Texture on the top side
 * @param[in] down:          Texture on the bottom side
 * @param[in] step:          Current step within [0.f , 1.f] range
 * @param[in] width:         Texture width
 * @param[in] height:        Texture height
 *
 *****************************************************************************************
 */
void hal_gfx_transition_stack_ver(hal_gfx_tex_t up, hal_gfx_tex_t down, float step,
                                int width, int height);

/**
 *****************************************************************************************
 * @brief Fade transition. Initial texture is being faded out, while final texture is being faded in.
 * The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] initial:       Texture initial
 * @param[in] final:         Texture final
 * @param[in] blending_mode: Blending mode
 * @param[in] step:          Current step within [0.f , 1.f] range
 * @param[in] width:         Texture width
 * @param[in] height:        Texture height
 *
 *****************************************************************************************
 */
void hal_gfx_transition_fade(hal_gfx_tex_t initial, hal_gfx_tex_t final,
                          uint32_t blending_mode, float step, int width, int height);

/**
 *****************************************************************************************
 * @brief Fade-zoom transition. Initial texture is being zoomed and faded out, while final texture is being zoomed and faded in.
 * The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] initial:       Initial texture
 * @param[in] final:         Final texture
 * @param[in] blending_mode: Blending mode
 * @param[in] step:          Current step within [0.f , 1.f] range
 * @param[in] width:         Texture width
 * @param[in] height:        Texture height
 *
 *****************************************************************************************
 */
void hal_gfx_transition_fade_zoom(hal_gfx_tex_t initial, hal_gfx_tex_t final,
                                uint32_t blending_mode, float step, int width, int height);

/**
 *****************************************************************************************
 * @brief Spin around Y-Axis transition .
 * The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] front:         front texture
 * @param[in] back :         back texture
 * @param[in] blending_mode: Blending mode
 * @param[in] step:          Current step within [0.f , 1.f] range, [0, 0.5] - show front texture; (0.5, 1], show back texture
 * @param[in] width:         Texture width
 * @param[in] height:        Texture height
 * @param[in] is_clockwise:  Spin by clockwise or counter-clockwise
 *
 *****************************************************************************************
 */
void hal_gfx_transition_spin_hor(hal_gfx_tex_t front, hal_gfx_tex_t back,
                              uint32_t blending_mode, float step,
                              int width, int height, bool is_clockwise);

/**
 *****************************************************************************************
 * @brief Push-pull transition horizontally. This is a variation of linear transition with some field-of-depth effect.
 * The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] initial:       Initial texture
 * @param[in] final:         Final texture
 * @param[in] blending_mode: Blending mode
 * @param[in] step:          Current step within [0.f , 1.f] range
 * @param[in] width:         Texture width
 * @param[in] height:        Texture height
 *
 *****************************************************************************************
 */
void hal_gfx_transition_pushpull_hor(hal_gfx_tex_t initial, hal_gfx_tex_t final,
                                     uint32_t blending_mode, float step, int width, int height);

/**
 *****************************************************************************************
 * @brief Push-pull transition vertically. This is a variation of linear transition with some field-of-depth effect.
 * The transition is complete when 'step' is 0 or 1.
 *
 * @param[in] initial:       Initial texture
 * @param[in] final:         Final texture
 * @param[in] blending_mode: Blending mode
 * @param[in] step:          Current step within [0.f , 1.f] range
 * @param[in] width:         Texture width
 * @param[in] height:        Texture height
 *
 *****************************************************************************************
 */
void hal_gfx_transition_pushpull_ver(hal_gfx_tex_t initial, hal_gfx_tex_t final,
                                     uint32_t blending_mode, float step, int width, int height);
/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */


/**
 ****************************************************************************************
 *
 * @file    hal_gfx_blender.h
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

/** @defgroup HAL_GFX_BLENDER GFX BLENDER
 * @brief GPU blender interfaces.
 * @{
 */

#ifndef HAL_GFX_BLENDER_H__
#define HAL_GFX_BLENDER_H__

#include "hal_gfx_sys_defs.h"
#include "hal_gfx_graphics.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @defgroup HAL_GFX_BLENDER_MACRO Defines
 * @{
 */
// Blending Factor Selector
//-----------------------------------------------------------------------------------------------------------------------
#define HAL_GFX_BF_ZERO         (0x0U) /**< 0 */
#define HAL_GFX_BF_ONE          (0x1U) /**< 1 */
#define HAL_GFX_BF_SRCCOLOR     (0x2U) /**< Sc */
#define HAL_GFX_BF_INVSRCCOLOR  (0x3U) /**< (1-Sc) */
#define HAL_GFX_BF_SRCALPHA     (0x4U) /**< Sa */
#define HAL_GFX_BF_INVSRCALPHA  (0x5U) /**< (1-Sa) */
#define HAL_GFX_BF_DESTALPHA    (0x6U) /**< Da */
#define HAL_GFX_BF_INVDESTALPHA (0x7U) /**< (1-Da) */
#define HAL_GFX_BF_DESTCOLOR    (0x8U) /**< Dc */
#define HAL_GFX_BF_INVDESTCOLOR (0x9U) /**< (1-Dc) */
#define HAL_GFX_BF_CONSTCOLOR   (0xaU) /**< Cc */
#define HAL_GFX_BF_CONSTALPHA   (0xbU) /**< Ca */

/*                  source factor         destination factor */
#define HAL_GFX_BL_SIMPLE     (  (uint32_t)HAL_GFX_BF_SRCALPHA      |   ((uint32_t)HAL_GFX_BF_INVSRCALPHA  <<8)  )   /**< Sa * Sa + Da * (1 - Sa) */
#define HAL_GFX_BL_CLEAR      (  (uint32_t)HAL_GFX_BF_ZERO        /*|   ((uint32_t)HAL_GFX_BF_ZERO         <<8)*/)   /**< 0 */
#define HAL_GFX_BL_SRC        (  (uint32_t)HAL_GFX_BF_ONE         /*|   ((uint32_t)HAL_GFX_BF_ZERO         <<8)*/)   /**< Sa */
#define HAL_GFX_BL_SRC_OVER   (  (uint32_t)HAL_GFX_BF_ONE           |   ((uint32_t)HAL_GFX_BF_INVSRCALPHA  <<8)  )   /**< Sa + Da * (1 - Sa) */
#define HAL_GFX_BL_DST_OVER   (  (uint32_t)HAL_GFX_BF_INVDESTALPHA  |   ((uint32_t)HAL_GFX_BF_ONE          <<8)  )   /**< Sa * (1 - Da) + Da */
#define HAL_GFX_BL_SRC_IN     (  (uint32_t)HAL_GFX_BF_DESTALPHA   /*|   ((uint32_t)HAL_GFX_BF_ZERO         <<8)*/)   /**< Sa * Da */
#define HAL_GFX_BL_DST_IN     (/*(uint32_t)HAL_GFX_BF_ZERO          |*/ ((uint32_t)HAL_GFX_BF_SRCALPHA     <<8)  )   /**< Da * Sa */
#define HAL_GFX_BL_SRC_OUT    (  (uint32_t)HAL_GFX_BF_INVDESTALPHA/*|   ((uint32_t)HAL_GFX_BF_ZERO         <<8)*/ )   /**< Sa * (1 - Da) */
#define HAL_GFX_BL_DST_OUT    (/*(uint32_t)HAL_GFX_BF_ZERO          |*/ ((uint32_t)HAL_GFX_BF_INVSRCALPHA  <<8)  )   /**< Da * (1 - Sa) */
#define HAL_GFX_BL_SRC_ATOP   (  (uint32_t)HAL_GFX_BF_DESTALPHA     |   ((uint32_t)HAL_GFX_BF_INVSRCALPHA  <<8)  )   /**< Sa * Da + Da * (1 - Sa) */
#define HAL_GFX_BL_DST_ATOP   (  (uint32_t)HAL_GFX_BF_INVDESTALPHA  |   ((uint32_t)HAL_GFX_BF_SRCALPHA     <<8)  )   /**< Sa * (1 - Da) + Da * Sa */
#define HAL_GFX_BL_ADD        (  (uint32_t)HAL_GFX_BF_ONE           |   ((uint32_t)HAL_GFX_BF_ONE          <<8)  )   /**< Sa + Da */
#define HAL_GFX_BL_XOR        (  (uint32_t)HAL_GFX_BF_INVDESTALPHA  |   ((uint32_t)HAL_GFX_BF_INVSRCALPHA  <<8)  )   /**< Sa * (1 - Da) + Da * (1 - Sa) */


#define HAL_GFX_BLOP_NONE         (0U)           /**< No extra blending operation */
#define HAL_GFX_BLOP_STENCIL_TXTY (0x00800000U)  /**< Use TEX3 as mask */
#define HAL_GFX_BLOP_STENCIL_XY   (0x00400000U)  /**< Use TEX3 as mask */
#define HAL_GFX_BLOP_NO_USE_ROPBL (0x01000000U)  /**< Don't use Rop Blender even if present */
#define HAL_GFX_BLOP_DST_CKEY_NEG (0x02000000U)  /**< Apply Inverse Destination Color Keying - draw only when dst color doesn't match colorkey*/
#define HAL_GFX_BLOP_SRC_PREMULT  (0x04000000U)  /**< Premultiply Source Color with Source Alpha (cannot be used with HAL_GFX_BLOP_MODULATE_RGB) */
#define HAL_GFX_BLOP_MODULATE_A   (0x08000000U)  /**< Modulate by Constant Alpha value*/
#define HAL_GFX_BLOP_FORCE_A      (0x10000000U)  /**< Force Constant Alpha value */
#define HAL_GFX_BLOP_MODULATE_RGB (0x20000000U)  /**< Modulate by Constant Color (RGB) values */
#define HAL_GFX_BLOP_SRC_CKEY     (0x40000000U)  /**< Apply Source Color Keying - draw only when src color doesn't match colorkey */
#define HAL_GFX_BLOP_DST_CKEY     (0x80000000U)  /**< Apply Destination Color Keying - draw only when dst color matches colorkey */
#define HAL_GFX_BLOP_MASK         (0xffc00000U)  /**< blending operation mask */
/** @} */

/**
 * @defgroup HAL_GFX_BLENDER_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Return blending mode given source and destination blending factors and additional blending operations
 *
 * @param[in] src_bf: Source Blending Factor
 * @param[in] dst_bf: Destination Blending Factor
 * @param[in] blops: Additional Blending Operations
 *
 * @return Final Blending Mode
 *****************************************************************************************
 */
static inline uint32_t hal_gfx_blending_mode(uint32_t src_bf, uint32_t dst_bf, uint32_t blops) {
    return ( (src_bf) | (dst_bf << 8) | (blops&HAL_GFX_BLOP_MASK) );
}

/**
 *****************************************************************************************
 * @brief Set blending mode
 * @note  Blit mode only supports foreground texture slot blending and foreground with background texture slot blending
 *
 * @param[in] blending_mode: Blending mode to be set
 * @param[in] dst_tex:       Destination Texture
 * @param[in] fg_tex:        Foreground (source) Texture
 * @param[in] bg_tex:        Background (source2) Texture
 *
 *****************************************************************************************
 */
void hal_gfx_set_blend(uint32_t blending_mode, hal_gfx_tex_t dst_tex, hal_gfx_tex_t fg_tex, hal_gfx_tex_t bg_tex);

/**
 *****************************************************************************************
 * @brief Set blending mode for filling
 *
 * @param[in] blending_mode: Blending mode to be set
 *
 *****************************************************************************************
 */
static inline void hal_gfx_set_blend_fill(uint32_t blending_mode) {
    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, HAL_GFX_NOTEX, HAL_GFX_NOTEX);
}

/**
 *****************************************************************************************
 * @brief Set blending mode for filling with composing
 *
 * @param[in] blending_mode: Blending mode to be set
 *
 *****************************************************************************************
 */
static inline void hal_gfx_set_blend_fill_compose(uint32_t blending_mode) {
    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, HAL_GFX_NOTEX, HAL_GFX_TEX2);
}

/**
 *****************************************************************************************
 * @brief Set blending mode for blitting
 *
 * @param[in] blending_mode: Blending mode to be set
 *
 *****************************************************************************************
 */
static inline void hal_gfx_set_blend_blit(uint32_t blending_mode) {
    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, HAL_GFX_TEX1, HAL_GFX_NOTEX);
}

/**
 *****************************************************************************************
 * @brief Set blending mode for blitting with composing
 *
 * @param[in] blending_mode: Blending mode to be set
 *
 *****************************************************************************************
 */
static inline void hal_gfx_set_blend_blit_compose(uint32_t blending_mode) {
    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, HAL_GFX_TEX1, HAL_GFX_TEX2);
}

/**
 *****************************************************************************************
 * @brief Set constant color
 *
 * @param[in] rgba: RGBA color (format: R[0,7] G[8,15] B[16,23] A[24,31])
 *
 *****************************************************************************************
 */
void hal_gfx_set_const_color(uint32_t rgba);

/**
 *****************************************************************************************
 * @brief Set source color key
 *
 * @param[in] rgba: RGBA color key (format: R[0,7] G[8,15] B[16,23] A[24,31])
 *
 *****************************************************************************************
 */
void hal_gfx_set_src_color_key(uint32_t rgba);

/**
 *****************************************************************************************
 * @brief Set destination color key
 *
 * @param[in] rgba: RGBA color key (format: R[0,7] G[8,15] B[16,23] A[24,31])
 *
 *****************************************************************************************
 */
void hal_gfx_set_dst_color_key(uint32_t rgba);

/**
 *****************************************************************************************
 * @brief Enable/disable ovedraw debugging. Disables gradient and texture, forces blending mode to HAL_GFX_BL_ADD
 *
 * @param[in] enable: Enables overdraw debugging if non-zero
 *
 *****************************************************************************************
 */
void hal_gfx_debug_overdraws(uint32_t enable);
/** @} */

#ifdef __cplusplus
}
#endif

#endif // HAL_GFX_BLENDER_H__

/** @} */
/** @} */
/** @} */


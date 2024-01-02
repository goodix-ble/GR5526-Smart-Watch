/**
 ****************************************************************************************
 *
 * @file    hal_gfx_graphics.h
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

/** @defgroup HAL_GFX_GRAPHICS GFX GRAPHICS
  * @brief Draw basic primitive interfaces and blit texture interfaces
  * @{
  */


#ifndef HAL_GFX_GRAPHICS_H__
#define HAL_GFX_GRAPHICS_H__

#include "hal_gfx_sys_defs.h"

#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup HAL_GFX_GRAPHICS_ENUMERATIONS Enumerations
  * @{
  */

/**
  * @brief  Graphics Texture
  */
typedef enum {
    HAL_GFX_NOTEX = -1,    /**< No Texture */
    HAL_GFX_TEX0  =  0,    /**< Texture 0 */
    HAL_GFX_TEX1  =  1,    /**< Texture 1 */
    HAL_GFX_TEX2  =  2,    /**< Texture 2 */
    HAL_GFX_TEX3  =  3     /**< Texture 3 */
} hal_gfx_tex_t;

/**
  * @brief  Graphics Triangle Culling methond
  */
typedef enum {
    HAL_GFX_CULL_NONE = 0      ,                             /**< Disable Triangle/Quadrilateral Culling */
    HAL_GFX_CULL_CW   = (1U<<28),                            /**< Cull clockwise Triangles/Quadrilaterals */
    HAL_GFX_CULL_CCW  = (1U<<29),                            /**< Cull anti-clockwise Triangles/Quadrilaterals */
    HAL_GFX_CULL_ALL  = HAL_GFX_CULL_CW | HAL_GFX_CULL_CCW   /**< Cull all */
} hal_gfx_tri_cull_t;

/** @} */

/**
 * @defgroup HAL_GFX_GRAPHICS_MACRO Defines
 * @{
 */

#define HAL_GFX_RGBX8888   (0x00U)  /**< RGBX8888 */
#define HAL_GFX_RGBA8888   (0x01U)  /**< RGBA8888 */
#define HAL_GFX_XRGB8888   (0x02U)  /**< XRGB8888 */
#define HAL_GFX_ARGB8888   (0x03U)  /**< ARGB8888 */
#define HAL_GFX_RGB565     (0x04U)  /**< RGBA5650 */
#define HAL_GFX_RGBA5650   (0x04U)  /**< RGBA5650 */
#define HAL_GFX_RGBA5551   (0x05U)  /**< RGBA5551 */
#define HAL_GFX_RGBA4444   (0x06U)  /**< RGBA4444 */
#define HAL_GFX_RGBA0800   (0x07U)  /**< RGBA0800 */
#define HAL_GFX_A8         (0x08U)  /**< RGBA0008 */
#define HAL_GFX_RGBA0008   (0x08U)  /**< RGBA0008 */
#define HAL_GFX_L8         (0x09U)  /**< L8       */
#define HAL_GFX_RGBA3320   (0x38U)  /**< RGBA3320 (source only) */
#define HAL_GFX_RGB332     (0x38U)  /**< RGBA3320 (source only) */
#define HAL_GFX_BW1        (0x0CU)  /**< A1      (source only) */
#define HAL_GFX_A1         (0x0CU)  /**< A1      (source only) */
#define HAL_GFX_L1         (0x0BU)  /**< L1      (source only) */
#define HAL_GFX_UYVY       (0x0DU)  /**< UYVY     */
#define HAL_GFX_ABGR8888   (0x0EU)  /**< ABGR8888 */
#define HAL_GFX_BGRA8888   (0x10U)  /**< BGRA     */
#define HAL_GFX_BGRX8888   (0x11U)  /**< BGRX     */
#define HAL_GFX_TSC4       (0x12U)  /**< TSC4     */
#define HAL_GFX_TSC6       (0x16U)  /**< TSC6     */
#define HAL_GFX_TSC6A      (0x17U)  /**< TSC6A    */
#define HAL_GFX_RV         (0x18U)  /**< RV       */
#define HAL_GFX_GU         (0x19U)  /**< GU       */
#define HAL_GFX_BY         (0x1AU)  /**< BY       */
#define HAL_GFX_YUV        (0x1BU)  /**< YUV      */
#define HAL_GFX_Z24_8      (0x1cU)  /**< Z24_8    */
#define HAL_GFX_Z16        (0x1dU)  /**< Z16      */
#define HAL_GFX_UV         (0x1eU)  /**< UV       */
#define HAL_GFX_A1LE       (0x27U)  /**< A1LE     */
#define HAL_GFX_A2LE       (0x28U)  /**< A2LE     */
#define HAL_GFX_A4LE       (0x29U)  /**< A4LE     */
#define HAL_GFX_L1LE       (0x2AU)  /**< L1LE     */
#define HAL_GFX_L2LE       (0x2BU)  /**< L2LE     */
#define HAL_GFX_L4LE       (0x2CU)  /**< L4LE     */
#define HAL_GFX_A2         (0x30U)  /**< A2       */
#define HAL_GFX_A4         (0x34U)  /**< A4       */
#define HAL_GFX_L2         (0x31U)  /**< L2       */
#define HAL_GFX_L4         (0x35U)  /**< L4       */
#define HAL_GFX_BGR24      (0x39U)  /**< BGR24    */
#define HAL_GFX_RGB24      (0x3CU)  /**< RGB24    */
#define HAL_GFX_RV10       (0x3DU)  /**< RV-10bit */
#define HAL_GFX_GU10       (0x3EU) /**< GU-10bit */
#define HAL_GFX_BY10       (0x3FU)  /**< BY-10bit */

#define HAL_GFX_DITHER     (0x80U)  /**< GPU Dithering */
#define HAL_GFX_FORMAT_MASK (0x7FU) /**< Format Mask */

//-----------------------------------------------------------------------------------------------------------------------

// Texture Unit Parameters
//-----------------------------------------------------------------------------------------
// Filtering - 0:0
//----------------------
#define HAL_GFX_FILTER_PS  (0x00U) /**< Point Sampling. */
#define HAL_GFX_FILTER_BL  (0x01U) /**< Bilinear filtering. */

// Wrapping Mode 3:2
//----------------------
#define HAL_GFX_TEX_CLAMP  (0x00U<<2) /**< Clamp */
#define HAL_GFX_TEX_REPEAT (0x01U<<2) /**< Repeat */
#define HAL_GFX_TEX_BORDER (0x02U<<2) /**< Border */
#define HAL_GFX_TEX_MIRROR (0x03U<<2) /**< Mirror */

// Texture Coordinates Ordering 4:4
//----------------------
#define HAL_GFX_TEX_MORTON_ORDER (0x10U)/**< Order */

// Texture Coordinates Format 6:5
//----------------------
#define HAL_GFX_TEX_RANGE_0_1   (0x1U<<5)  /**< Interpolated Coordinates range: 0-1 */
#define HAL_GFX_TEX_LEFT_HANDED (0x1U<<6)  /**< (0,0) is bottom left corner */

// Rotation Modes
//-----------------------------------------------------------------------------------------------------------------------

#define HAL_GFX_ROT_000_CCW  (0x0U)  /**< No rotation */
#define HAL_GFX_ROT_090_CCW  (0x1U)  /**< Rotate  90 degrees counter-clockwise */
#define HAL_GFX_ROT_180_CCW  (0x2U)  /**< Rotate 180 degrees counter-clockwise */
#define HAL_GFX_ROT_270_CCW  (0x3U)  /**< Rotate 270 degrees counter-clockwise */
#define HAL_GFX_ROT_000_CW   (0x0U)  /**< No rotation */
#define HAL_GFX_ROT_270_CW   (0x1U)  /**< Rotate 270 degrees clockwise */
#define HAL_GFX_ROT_180_CW   (0x2U)  /**< Rotate 180 degrees clockwise */
#define HAL_GFX_ROT_090_CW   (0x3U)  /**< Rotate  90 degrees clockwise */
#define HAL_GFX_MIR_VERT     (0x4U)  /**< Mirror Vertically */
#define HAL_GFX_MIR_HOR      (0x8U)  /**< Mirror Horizontally */

/** @} */

/** @defgroup HAL_GFX_GRAPHICS_TYPE Typedef
  * @{
  */
typedef uint8_t hal_gfx_tex_mode_t;             /**< The tex mode type */
typedef uint32_t hal_gfx_tex_format_t;  /**< The tex format type */
/** @} */


/** @addtogroup HAL_GFX_GRAPHICS_FUNCTION Functions
  * @{
  */

/**
 * @defgroup HAL_GFX_GRAPHICS_PUBLIC_FUNCTION Public Functions
 * @{
 */

/**
 *****************************************************************************************
 * @brief Check if a known GPU is present
 * @return Return -1 if no known GPU is present
 *****************************************************************************************
 */
int hal_gfx_checkGPUPresence(void);

/**
 *****************************************************************************************
 * @brief Program a Texture Unit
 * @param[in] texid: Texture unit to be programmed
 * @param[in] addr_gpu: Texture's address as seen by the GPU
 * @param[in] width: Texture's width
 * @param[in] height: Texture's height
 * @param[in] format: Texture's format
 * @param[in] stride: Texture's stride. If stride < 0, it's left to be calculated
 * @param[in] wrap_mode:  Wrap/Repeat mode to be used
 *****************************************************************************************
 */
void hal_gfx_bind_tex(hal_gfx_tex_t texid, uintptr_t addr_gpu,
                   uint32_t width, uint32_t height,
                   hal_gfx_tex_format_t format, int32_t stride, hal_gfx_tex_mode_t wrap_mode);

/**
 *****************************************************************************************
 * @brief Set Texture Mapping default color
 * @param[in] color: default color in 32-bit RGBA format, @ref hal_gfx_rgba()
 *****************************************************************************************
 */
void hal_gfx_set_tex_color(uint32_t color);

/**
 *****************************************************************************************
 * @brief Write a value to a Constant Register of the GPU
 * @param[in] reg: Constant Register to be written
 * @param[in] value: Value to be written
 *****************************************************************************************
 */
void hal_gfx_set_const_reg(int reg, uint32_t value);

/**
 *****************************************************************************************
 * @brief Sets the drawing area's Clipping Rectangle
 * @param[in] x: Clip Window top-left x coordinate
 * @param[in] y: Clip Window minimum y
 * @param[in] w: Clip Window width
 * @param[in] h: Clip Window height
 *****************************************************************************************
 */
void hal_gfx_set_clip(int32_t x, int32_t y, uint32_t w, uint32_t h);

/**
 *****************************************************************************************
 * @brief Gets the drawing area's Clipping Rectangle
 * @param[in] x: pointer to Clip Window top-left x coordinate
 * @param[in] y: pointer to Clip Window minimum y
 * @param[in] w: pointer to Clip Window width
 * @param[in] h: pointer to Clip Window height
 *****************************************************************************************
 */
void hal_gfx_get_clip(int32_t * x, int32_t * y, uint32_t * w, uint32_t * h);

/**
 *****************************************************************************************
 * @brief Enable color gradient
 * @param[in] enable: !0 enable, 0 disable
 *****************************************************************************************
 */
void hal_gfx_enable_gradient(int enable);

/**
 *****************************************************************************************
 * @brief Enable depth
 * @param[in] enable: !0 enable, 0 disable
 *****************************************************************************************
 */
void hal_gfx_enable_depth(int enable);

/**
 *****************************************************************************************
 * @brief Enables MSAA per edge
 * @param[in] e0: Enable MSAA for edge 0 (vertices 0-1)
 * @param[in] e1: Enable MSAA for edge 1 (vertices 1-2)
 * @param[in] e2: Enable MSAA for edge 2 (vertices 2-3)
 * @param[in] e3: Enable MSAA for edge 3 (vertices 3-0)
 * @return previous AA flags (may be ignored)
 *****************************************************************************************
 */
uint32_t hal_gfx_enable_aa(uint8_t e0, uint8_t e1, uint8_t e2, uint8_t e3);

/**
 *****************************************************************************************
 * @brief Returns the bounding rectangle of all the pixels that have been modified since its previous call
 * @param[out] minx: x coordinate of the upper left corner of the dirty region
 * @param[out] miny: y coordinate of the upper left corner of the dirty region
 * @param[out] maxx: x coordinate of the lower right corner of the dirty region
 * @param[out] maxy: y coordinate of the lower right corner of the dirty region
 *****************************************************************************************
 */
void hal_gfx_get_dirty_region(int *minx, int *miny, int *maxx, int *maxy);

/**
 *****************************************************************************************
 * @brief Clear dirty region information - runs via the bound command-list, @ref hal_gfx_get_dirty_region(), @ref hal_gfx_clear_dirty_region()
 * @retval None
 *****************************************************************************************
 */
void hal_gfx_clear_dirty_region(void);

/**
 *****************************************************************************************
 * @brief Clear dirty region information immediately, no command-list involved, @ref hal_gfx_get_dirty_region(), @ref hal_gfx_clear_dirty_region_imm()
 *  @retval None
 *****************************************************************************************
 */
void hal_gfx_clear_dirty_region_imm(void);

/**
 *****************************************************************************************
 * @brief Set triangle/quadrilateral culling mode
 * @param[in] cull:  Culling mode, @ref hal_gfx_tri_cull_t
 *****************************************************************************************
 */
void hal_gfx_tri_cull(hal_gfx_tri_cull_t cull);

/**
 *****************************************************************************************
 * @brief Return pixel size in bytes
 * @param[in] format:  Color format
 * @return Return Pixel size in bytes
 *****************************************************************************************
 */
int hal_gfx_format_size (hal_gfx_tex_format_t format);

/**
 *****************************************************************************************
 * @brief Return stride in bytes
 * @param[in] format:  Color format
 * @param[in] wrap_mode:  Wrap/Repeat mode to be used
 * @param[in] width: Texture color format
 * @return Return stride in bytes
 *****************************************************************************************
 */
int hal_gfx_stride_size(hal_gfx_tex_format_t format, hal_gfx_tex_mode_t wrap_mode, int width);

/**
 *****************************************************************************************
 * @brief Return texture size in bytes
 * @param[in] format: Texture color format
 * @param[in] wrap_mode: Wrap/Repeat mode to be used
 * @param[in] width: Texture width
 * @param[in] height: Texture height
 * @return Return Texture size in bytes
 *****************************************************************************************
 */
int hal_gfx_texture_size(hal_gfx_tex_format_t format, hal_gfx_tex_mode_t wrap_mode, int width, int height);

/**
 *****************************************************************************************
 * @brief Return Nema internal RGBA color
 * @param[in] R: Red component
 * @param[in] G: Green component
 * @param[in] B: Blue component
 * @param[in] A: Alpha component
 * @return Return RGBA value
 *****************************************************************************************
 */
uint32_t hal_gfx_rgba(unsigned char R,
                   unsigned char G,
                   unsigned char B,
                   unsigned char A);

/**
 *****************************************************************************************
 * @brief Premultiply RGB channels with Alpha channel
 * @param[in] rgba: RGBA color
 * @return Premultiplied RGBA color
 *****************************************************************************************
 */
uint32_t hal_gfx_premultiply_rgba(uint32_t rgba);

/**
 *****************************************************************************************
 * @brief Initialize hal_gfx library
 * @return Return negative value on error
 *****************************************************************************************
 */
int hal_gfx_init(void);

/**
 *****************************************************************************************
 * @brief Program Texture Unit with a foreground (source) texture (@ref HAL_GFX_TEX1)
 * @param[in] baseaddr_phys: Address of the source texture, as seen by the GPU
 * @param[in] width: Texture width
 * @param[in] height: Texture hight
 * @param[in] format: Texture format
 * @param[in] stride: Texture stride. If negative, it's calculated internally
 * @param[in] mode: Wrapping and Filtering mode
 *****************************************************************************************
 */
void hal_gfx_bind_src_tex(uintptr_t baseaddr_phys,
                       uint32_t width, uint32_t height,
                       hal_gfx_tex_format_t format, int32_t stride, hal_gfx_tex_mode_t mode);

/**
 *****************************************************************************************
 * @brief Program Texture Unit with a background texture (@ref HAL_GFX_TEX2)
 * @param[in] baseaddr_phys: Address of the source2 texture, as seen by the GPU
 * @param[in] width: Texture width
 * @param[in] height: Texture hight
 * @param[in] format: Texture format
 * @param[in] stride: Texture stride. If negative, it's calculated internally
 * @param[in] mode: Wrapping and Filtering mode
 *****************************************************************************************
 */
void hal_gfx_bind_src2_tex(uintptr_t baseaddr_phys,
                       uint32_t width, uint32_t height,
                       hal_gfx_tex_format_t format, int32_t stride, hal_gfx_tex_mode_t mode);

/**
 *****************************************************************************************
 * @brief Program Texture Unit with a destination texture (@ref HAL_GFX_TEX0)
 * @param[in] baseaddr_phys: Address of the destination texture, as seen by the GPU
 * @param[in] width: Texture width
 * @param[in] height: Texture hight
 * @param[in] format: Texture format
 * @param[in] stride: Texture stride. If negative, it's calculated internally
 *****************************************************************************************
 */
void hal_gfx_bind_dst_tex(uintptr_t baseaddr_phys,
                        uint32_t width, uint32_t height,
                        hal_gfx_tex_format_t format, int32_t stride);

/**
 *****************************************************************************************
 * @brief Bind Depth Buffer
 * @param[in] baseaddr_phys: Address of the depth buffer, as seen by the GPU
 * @param[in] width: Buffer width
 * @param[in] height: Buffer hight
 *****************************************************************************************
 */
void hal_gfx_bind_depth_buffer(uintptr_t baseaddr_phys,
                             uint32_t width, uint32_t height);

/**
 *****************************************************************************************
 * @brief Clear destination texture with color
 * @param[in] rgba8888: 32-bit RGBA color
 *****************************************************************************************
 */
void hal_gfx_clear(uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Clear depth buffer with specified value
 * @param[in] val: Clear value
 *****************************************************************************************
 */
void hal_gfx_clear_depth(uint32_t val);

/**
 *****************************************************************************************
 * @brief Draw a colored line, @ref hal_gfx_rgba()
 * @param[in] x0 x coordinate at the beginning of the line
 * @param[in] y0 y coordinate at the beginning of the line
 * @param[in] x1 x coordinate at the end of the line
 * @param[in] y1 y coordinate at the end of the line
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_draw_line(int x0, int y0, int x1, int y1, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Draw a line with width. Apply AA if available, @ref hal_gfx_rgba(), @ref hal_gfx_draw_line()
 * @param[in] x0 x coordinate at the beginning of the line
 * @param[in] y0 y coordinate at the beginning of the line
 * @param[in] x1 x coordinate at the end of the line
 * @param[in] y1 y coordinate at the end of the line
 * @param[in] w: line width
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void
hal_gfx_draw_line_aa(float x0, float y0, float x1, float y1, float w,
                  uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Draw a colored circle with 1 pixel width, @ref hal_gfx_rgba()
 * @param[in] x: x coordinate of the circle's center
 * @param[in] y: y coordinate of the circle's center
 * @param[in] r: circle's radius
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_draw_circle(int x, int y, int r, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Draw a colored circle with Anti-Aliasing (if available) and specified width, @ref hal_gfx_rgba()
 * @param[in] x: x coordinate of the circle's center
 * @param[in] y: y coordinate of the circle's center
 * @param[in] r: circle's radius
 * @param[in] w: pencil width
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_draw_circle_aa(float x, float y, float r, float w, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Draw a colored rectangle with rounded edges, @ref hal_gfx_rgba()
 * @param[in] x0: x coordinate of the upper left vertex of the rectangle
 * @param[in] y0: y coordinate at the upper left vertex of the rectangle
 * @param[in] w: width of the rectangle
 * @param[in] h: height of the rectangle
 * @param[in] r: corner radius
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_draw_rounded_rect(int x0, int y0, int w, int h, int r, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Draw a colored rectangle, @ref hal_gfx_rgba()
 * @param[in] x: x coordinate of the upper left vertex of the rectangle
 * @param[in] y: y coordinate at the upper left vertex of the rectangle
 * @param[in] w: width of the rectangle
 * @param[in] h: height of the rectangle
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_draw_rect(int x, int y, int w, int h, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Fill a circle with color, @ref hal_gfx_rgba()
 * @param[in] x: x coordinate of the circle's center
 * @param[in] y: y coordinate of the circle's center
 * @param[in] r: circle's radius
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_fill_circle(int x, int y, int r, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Fill a circle with color, use Anti-Aliasing if available, @ref hal_gfx_rgba()
 * @param[in] x: x coordinate of the circle's center
 * @param[in] y: y coordinate of the circle's center
 * @param[in] r: circle's radius
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_fill_circle_aa(float x, float y, float r, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Fill a triangle with color
 * @param[in] x0: x coordinate at the first vertex of the triangle
 * @param[in] y0: y coordinate at the first vertex of the triangle
 * @param[in] x1: x coordinate at the second vertex of the triangle
 * @param[in] y1: y coordinate at the second vertex of the triangle
 * @param[in] x2: x coordinate at the third vertex of the triangle
 * @param[in] y2: y coordinate at the third vertex of the triangle
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_fill_triangle(int x0, int y0, int x1, int y1, int x2, int y2, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Fill a rectangle with rounded edges with color
 * @param[in] x0: x coordinate of the upper left vertex of the rectangle
 * @param[in] y0: y coordinate at the upper left vertex of the rectangle
 * @param[in] w: width of the rectangle
 * @param[in] h: height of the rectangle
 * @param[in] r: corner radius
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_fill_rounded_rect(int x0, int y0, int w, int h, int r, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Fill a rectangle with color
 * @param[in] x: x coordinate of the upper left vertex of the rectangle
 * @param[in] y: y coordinate at the upper left vertex of the rectangle
 * @param[in] w: width of the rectangle
 * @param[in] h: height of the rectangle
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_fill_rect(int x, int y, int w, int h, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Fill a quadrilateral with color
 * @param[in] x0: x coordinate at the first vertex of the quadrilateral
 * @param[in] y0: y coordinate at the first vertex of the quadrilateral
 * @param[in] x1: x coordinate at the second vertex of the quadrilateral
 * @param[in] y1: y coordinate at the second vertex of the quadrilateral
 * @param[in] x2: x coordinate at the third vertex of the quadrilateral
 * @param[in] y2: y coordinate at the third vertex of the quadrilateral
 * @param[in] x3: x coordinate at the fourth vertex of the quadrilateral
 * @param[in] y3: y coordinate at the fourth vertex of the quadrilateral
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_fill_quad(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Fill a rectangle with color (float coordinates)
 * @param[in] x: x coordinate of the upper left vertex of the rectangle
 * @param[in] y: y coordinate at the upper left vertex of the rectangle
 * @param[in] w: width of the rectangle
 * @param[in] h: height of the rectangle
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_fill_rect_f(float x, float y, float w, float h, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Fill a quadrilateral with color (float coordinates)
 * @param[in] x0: x coordinate at the first vertex of the quadrilateral
 * @param[in] y0: y coordinate at the first vertex of the quadrilateral
 * @param[in] x1: x coordinate at the second vertex of the quadrilateral
 * @param[in] y1: y coordinate at the second vertex of the quadrilateral
 * @param[in] x2: x coordinate at the third vertex of the quadrilateral
 * @param[in] y2: y coordinate at the third vertex of the quadrilateral
 * @param[in] x3: x coordinate at the fourth vertex of the quadrilateral
 * @param[in] y3: y coordinate at the fourth vertex of the quadrilateral
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_fill_quad_f(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Fill a triangle with color (float coordinates)
 * @param[in] x0: x coordinate at the first vertex of the triangle
 * @param[in] y0: y coordinate at the first vertex of the triangle
 * @param[in] x1: x coordinate at the second vertex of the triangle
 * @param[in] y1: y coordinate at the second vertex of the triangle
 * @param[in] x2: x coordinate at the third vertex of the triangle
 * @param[in] y2: y coordinate at the third vertex of the triangle
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_fill_triangle_f(float x0, float y0, float x1, float y1, float x2, float y2, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Blit source texture to destination texture
 * @note  The width and height of the destination for the fast blit function depends on
 *        the foreground (source) texture (@ref HAL_GFX_TEX1)
 *        When the background texture (@ref HAL_GFX_TEX2) is in effect, the background texture
 *        is always at (0,0) and will not be offset according to the destination x and y coordinates.
 * @param[in] x: destination x coordinate
 * @param[in] y: destination y coordinate
 *****************************************************************************************
 */
void hal_gfx_blit (int x, int y);

/**
 *****************************************************************************************
 * @brief Blit source texture to destination texture with rounded corners
 * @param[in] x: destination x coordinate
 * @param[in] y: destination y coordinate
 * @param[in] r: destination corner radius
 *****************************************************************************************
 */
void hal_gfx_blit_rounded (int x, int y, int r);

/**
 *****************************************************************************************
 * @brief Blit source texture to destination's specified rectangle (crop or wrap when needed)
 * @param[in] x: destination x coordinate
 * @param[in] y: destination y coordinate
 * @param[in] w: destination width
 * @param[in] h: destination height
 *****************************************************************************************
 */
void hal_gfx_blit_rect (int x, int y, int w, int h);

/**
 *****************************************************************************************
 * @brief Blit part of a source texture to destination's specified rectangle (crop or wrap when needed)
 * @param[in] dst_x: destination x coordinate
 * @param[in] dst_y: destination y coordinate
 * @param[in] w: destination width
 * @param[in] h: destination height
 * @param[in] src_x: source x coordinate
 * @param[in] src_y: source y coordinate
 *****************************************************************************************
 */
void hal_gfx_blit_subrect(int dst_x, int dst_y, int w, int h, int src_x, int src_y);

/**
 *****************************************************************************************
 * @brief Blit source texture to destination. Fit (scale) texture to specified rectangle
 * @param[in] x: destination x coordinate
 * @param[in] y: destination y coordinate
 * @param[in] w: destination width
 * @param[in] h: destination height
 *****************************************************************************************
 */
void hal_gfx_blit_rect_fit(int x, int y, int w, int h);

/**
 *****************************************************************************************
 * @brief Blit part of source texture to destination. Fit (scale) texture to specified rectangle
 * @param[in] dst_x: destination x coordinate
 * @param[in] dst_y: destination y coordinate
 * @param[in] dst_w: destination width
 * @param[in] dst_h: destination height
 * @param[in] src_x: source x coordinate
 * @param[in] src_y: source y coordinate
 * @param[in] src_w: source width
 * @param[in] src_h: source height
 *****************************************************************************************
 */
void hal_gfx_blit_subrect_fit( int dst_x, int dst_y, int dst_w, int dst_h,
                            int src_x, int src_y, int src_w, int src_h);

/**
 *****************************************************************************************
 * @brief Rotate around pivot point and Blit source texture
 * @param[in] cx: destination rotation center x coordinate
 * @param[in] cy: destination rotation center y coordinate
 * @param[in] px: source pivot point x coordinate
 * @param[in] py: source pivot point y coordinate
 * @param[in] degrees_cw: degrees of clockwise rotation in range [0, 360]
 *****************************************************************************************
 */
void hal_gfx_blit_rotate_pivot( float cx, float cy,
                             float px, float py, float degrees_cw );

/**
 *****************************************************************************************
 * @brief Rotate around pivot point and Blit source texture
 * @param[in] cx: destination rotation center x coordinate
 * @param[in] cy: destination rotation center y coordinate
 * @param[in] px: source pivot point x coordinate
 * @param[in] py: source pivot point y coordinate
 * @param[in] degrees_cw: degrees of clockwise rotation in range [0, 360]
 * @param[in] scale: the ratio of zoom in / zoom out, > 1.0 zoom out, < 1.0 zoom in
 *****************************************************************************************
 */
void hal_gfx_blit_rotate_pivot_scale( float cx, float cy, float px, float py, float degrees_cw,  float scale);

/**
 *****************************************************************************************
 * @brief Rotate and Blit source texture to destination
 * @param[in] x: destination x coordinate
 * @param[in] y: destination y coordinate
 * @param[in] rotation: Rotation to be done
 *****************************************************************************************
 */
void hal_gfx_blit_rotate(int x, int y, uint32_t rotation);

/**
 *****************************************************************************************
 * @brief Rotate and Blit partial source texture to destination
 * @param[in] sx: source upper left x coordinate
 * @param[in] sy: source upper left y coordinate
 * @param[in] sw: source width of partial region
 * @param[in] sh: source height of partial region
 * @param[in] x: destination x coordinate
 * @param[in] y: destination y coordinate
 * @param[in] rotation: Rotation to be done
 *****************************************************************************************
 */
void hal_gfx_blit_rotate_partial(int sx, int sy,
                              int sw, int sh,
                              int x,  int y,
                              uint32_t rotation);

/**
 *****************************************************************************************
 * @brief Blit source texture to destination. Fit texture to specified triangle
 * @param[in] dx0 x coordinate at the first vertex of the triangle
 * @param[in] dy0 y coordinate at the first vertex of the triangle
 * @param[in] v0  in [0, 3] indicates the corner of the texture that fits to the first vertex of the triangle
 *                0 _ _ 1
 *                 |_ _|
 *                3     2
 * @param[in] dx1 x coordinate at the second vertex of the triangle
 * @param[in] dy1 y coordinate at the second vertex of the triangle
 * @param[in] v1  in [0, 3] indicates the corner of the texture that fits to the second vertex of the triangle
 * @param[in] dx2 x coordinate at the third vertex of the triangle
 * @param[in] dy2 y coordinate at the third vertex of the triangle
 * @param[in] v2  in [0, 3] indicates the corner of the texture that fits to the third vertex of the triangle
 *****************************************************************************************
 */
void hal_gfx_blit_tri_fit (float dx0, float dy0, int v0,
                        float dx1, float dy1, int v1,
                        float dx2, float dy2, int v2);

/**
 *****************************************************************************************
 * @brief Blit a triangular part of the source tecture to a triangular destination area
 * @param[in] dx0 x coordinate at the first vertex of the destination triangle
 * @param[in] dy0 y coordinate at the first vertex of the destination triangle
 * @param[in] dw0 w coordinate at the first vertex of the destination triangle
 * @param[in] dx1 x coordinate at the second vertex of the destination triangle
 * @param[in] dy1 y coordinate at the second vertex of the destination triangle
 * @param[in] dw1 w coordinate at the second vertex of the destination triangle
 * @param[in] dx2 x coordinate at the third vertex of the destination triangle
 * @param[in] dy2 y coordinate at the third vertex of the destination triangle
 * @param[in] dw2 w coordinate at the third vertex of the destination triangle
 * @param[in] sx0 x coordinate at the first vertex of the source triangle
 * @param[in] sy0 y coordinate at the first vertex of the source triangle
 * @param[in] sx1 x coordinate at the second vertex of the source triangle
 * @param[in] sy1 y coordinate at the second vertex of the source triangle
 * @param[in] sx2 x coordinate at the third vertex of the source triangle
 * @param[in] sy2 y coordinate at the third vertex of the source triangle
 *****************************************************************************************
 */
void hal_gfx_blit_tri_uv  (float dx0, float dy0, float dw0,
                        float dx1, float dy1, float dw1,
                        float dx2, float dy2, float dw2,
                        float sx0, float sy0,
                        float sx1, float sy1,
                        float sx2, float sy2
                        );

/**
 *****************************************************************************************
 * @brief Blit source texture to destination. Fit texture to specified quadrilateral
 * @param[in] dx0 x coordinate at the first vertex of the quadrilateral
 * @param[in] dy0 y coordinate at the first vertex of the quadrilateral
 * @param[in] dx1 x coordinate at the second vertex of the quadrilateral
 * @param[in] dy1 y coordinate at the second vertex of the quadrilateral
 * @param[in] dx2 x coordinate at the third vertex of the quadrilateral
 * @param[in] dy2 y coordinate at the third vertex of the quadrilateral
 * @param[in] dx3 x coordinate at the fourth vertex of the quadrilateral
 * @param[in] dy3 y coordinate at the fourth vertex of the quadrilateral
 *****************************************************************************************
 */
void hal_gfx_blit_quad_fit (float dx0, float dy0,
                         float dx1, float dy1,
                         float dx2, float dy2,
                         float dx3, float dy3);

/**
 *****************************************************************************************
 * @brief Blit source texture to destination. Fit rectangulare area of texture to specified quadrilateral
 * @param[in] dx0: x coordinate at the first vertex of the quadrilateral
 * @param[in] dy0: y coordinate at the first vertex of the quadrilateral
 * @param[in] dx1: x coordinate at the second vertex of the quadrilateral
 * @param[in] dy1: y coordinate at the second vertex of the quadrilateral
 * @param[in] dx2: x coordinate at the third vertex of the quadrilateral
 * @param[in] dy2: y coordinate at the third vertex of the quadrilateral
 * @param[in] dx3: x coordinate at the fourth vertex of the quadrilateral
 * @param[in] dy3: y coordinate at the fourth vertex of the quadrilateral
 * @param[in] sx: x coordinate of the top left corner of the texture's rectangular area to be blitted
 * @param[in] sy: y coordinate of the top left corner of the texture's rectangular area to be blitted
 * @param[in] sw: width of the texture's rectangular area to be blitted
 * @param[in] sh: height of the texture's rectangular area to be blitted
 *****************************************************************************************
 */
void hal_gfx_blit_subrect_quad_fit(float dx0, float dy0,
                                float dx1, float dy1,
                                float dx2, float dy2,
                                float dx3, float dy3,
                                int sx, int sy,
                                int sw, int sh);


/** @} */

/**
 * @defgroup HAL_GFX_GRAPHICS_PRIVATE_FUNCTION Private Functions
 * @{
 */

/**
 *****************************************************************************************
 * @brief private function
 * @param[in] start: TODO
 * @param[in] dx: TODO
 * @param[in] dy: TODO
 *****************************************************************************************
 */
void hal_gfx_set_depth(float start, float dx, float dy);

/**
 *****************************************************************************************
 * @brief private function
 * @param[in] r_init: TODO
 * @param[in] g_init: TODO
 * @param[in] b_init: TODO
 * @param[in] a_init: TODO
 * @param[in] r_dx: TODO
 * @param[in] r_dy: TODO
 * @param[in] g_dx: TODO
 * @param[in] g_dy: TODO
 * @param[in] b_dx: TODO
 * @param[in] b_dy: TODO
 * @param[in] a_dx: TODO
 * @param[in] a_dy: TODO
 *****************************************************************************************
 */
void hal_gfx_set_gradient(float r_init, float g_init, float b_init, float a_init,
                       float r_dx, float r_dy,
                       float g_dx, float g_dy,
                       float b_dx, float b_dy,
                       float a_dx, float a_dy);

/**
 *****************************************************************************************
 * @brief Enable breakpoints
 *  @retval None
 *****************************************************************************************
 */
void hal_gfx_brk_enable(void);

/**
 *****************************************************************************************
 * @brief Disable breakpoints
 *  @retval None
 *****************************************************************************************
 */
void hal_gfx_brk_disable(void);

/**
 *****************************************************************************************
 * @brief Add a breakpoint to the current Command List
 * @return Breakpoint ID
 *****************************************************************************************
 */
int  hal_gfx_brk_add(void);

/**
 *****************************************************************************************
 * @brief Add a breakpoint to the current Command List
 * @param[in] brk_id: Breakpoint ID to wait for. If zero (0), wait until next Breakpoint
 * @return ID of reached Breakpoint
 *****************************************************************************************
 */
int  hal_gfx_brk_wait(int brk_id);

/**
 *****************************************************************************************
 * @brief Instruct the GPU to resume execution
 *  @retval None
 *****************************************************************************************
 */
void hal_gfx_brk_continue(void);

/**
 *****************************************************************************************
 * @brief Enable external hold signals
 * @param[in] hold_id: Hold signals to be enabled [0-3]
 *****************************************************************************************
 */
void hal_gfx_ext_hold_enable(uint32_t hold_id);

/**
 *****************************************************************************************
 * @brief Disable external hold signals
 * @param[in] hold_id: Hold signals to be disabled [0-3]
 *****************************************************************************************
 */
void hal_gfx_ext_hold_disable(uint32_t hold_id);

/**
 *****************************************************************************************
 * @brief Enable Interrupt Request when GPU reaches hold point
 * @param[in] hold_id: Hold signals' IRQ to be enabled [0-3]
 *****************************************************************************************
 */
void hal_gfx_ext_hold_irq_enable(uint32_t hold_id);

/**
 *****************************************************************************************
 * @brief Disable external hold signals
 * @param[in] hold_id: Hold signals' IRQ to be disabled [0-3]
 *****************************************************************************************
 */
void hal_gfx_ext_hold_irq_disable(uint32_t hold_id);

/**
 *****************************************************************************************
 * @brief Assert hold signals internally via a Command List
 * @param[in] hold_id: Hold signal to be asserted
 * @param[in] stop: If not zero, force Command List Processor to wait for FLAG to be deasserted
 *****************************************************************************************
 */
void hal_gfx_ext_hold_assert(uint32_t hold_id, int stop);

/**
 *****************************************************************************************
 * @brief Dessert hold signals internally via a Command List
 * @param[in] hold_id: Hold signal to be deasserted
 *****************************************************************************************
 */
void hal_gfx_ext_hold_deassert(uint32_t hold_id);

/**
 *****************************************************************************************
 * @brief Assert hold signals from the CPU (no Command List)
 * @param[in] hold_id: Hold signal to be asserted
 *****************************************************************************************
 */
void hal_gfx_ext_hold_assert_imm(uint32_t hold_id);

/**
 *****************************************************************************************
 * @brief Dessert hold signals from the CPU (no Command List)
 * @param[in] hold_id: Hold signal to be deasserted
 *****************************************************************************************
 */
void hal_gfx_ext_hold_deassert_imm(uint32_t hold_id);

/** @} */
/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */


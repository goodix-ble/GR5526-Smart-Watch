/**
 ****************************************************************************************
 *
 * @file    hal_gfx_font.h
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

/** @defgroup HAL_GFX_FONT GFX FONT
  * @brief Graphics Font function definition.
  * @{
  */

#ifndef HAL_GFX_FONT_H__
#define HAL_GFX_FONT_H__

#include "hal_gfx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup HAL_GFX_FONT_DEFINITION Defines
  * @{
  */
#define HAL_GFX_ALIGNX_LEFT            (0x00U)         /**< Align horizontally to the left */
#define HAL_GFX_ALIGNX_RIGHT           (0x01U)         /**< Align horizontally to the right */
#define HAL_GFX_ALIGNX_CENTER          (0x02U)         /**< Align horizontally centered */
#define HAL_GFX_ALIGNX_JUSTIFY         (0x03U)         /**< Justify horizontally */
#define HAL_GFX_ALIGNX_MASK            (0x03U)         /**< Horizontal alignment mask */
#define HAL_GFX_ALIGNY_TOP             (0x00U)         /**< Align vertically to the top */
#define HAL_GFX_ALIGNY_BOTTOM          (0x04U)         /**< Align vertically to the bottom */
#define HAL_GFX_ALIGNY_CENTER          (0x08U)         /**< Align vertically centered */
#define HAL_GFX_ALIGNY_JUSTIFY         (0x0cU)         /**< Justify vertically */
#define HAL_GFX_ALIGNY_MASK            (0x0cU)         /**< Vertical alignment mask */
#define HAL_GFX_TEXT_WRAP              (0x10U)         /**< Use text wrapping */
/** @} */

/** @defgroup HAL_GFX_FONT_STRUCT Structures
  * @{
  */

/**
  * @brief Font Kerning setting Structure
  */
typedef struct {
    uint32_t   left;         /**< Neighbor character to the left of the current one (Unicode value) */
    int8_t     x_offset;     /**< Kerning offset (horizontally) */
} hal_gfx_kern_pair_t;

/**
  * @brief Font glyph setting Structure
  */
typedef struct {
    uint32_t bitmapOffset;        /**< glyph bitmap offset address */
    uint8_t  width;                /**< glyph width */
    uint8_t  xAdvance;            /**< glyph advanced setting */
    int8_t   xOffset;            /**< horizontal offset */
    int8_t   yOffset;            /**< vertical offset */
    uint32_t kern_offset;        /**< Kerning offset */
    uint8_t  kern_length;        /**< Kerning length */
} hal_gfx_glyph_t;

/**
  * @brief Font range setting Structure
  */
typedef struct {
    uint32_t  first;                    /**< first font to apply glyphs */
    uint32_t  last;                        /**< last font to apply glyphs */
    const hal_gfx_glyph_t *glyphs;        /**< pointer to glyphs */
} hal_gfx_font_range_t;

/**
  * @brief Font setting Structure
  */
typedef struct {
    hal_gfx_buffer_t           bo;                    /**< the base object */
    const hal_gfx_font_range_t *ranges;            /**< the font range */
    const int                bitmap_size;        /**< bitmap size */
    const uint8_t           *bitmap;            /**< pointer to bitmap */
    uint32_t                 flags;                /**< specify the flag */
    uint8_t                  xAdvance;             /**< horizontal advance setting */
    uint8_t                  yAdvance;            /**< vertical advance setting */
    uint8_t                  max_ascender;        /**< max ascender setting */
    uint8_t                  bpp;                /**< bits per pixel setting */
    const hal_gfx_kern_pair_t  *kern_pairs;        /**< pointer to kern pair */
} hal_gfx_font_t;
/** @} */

/** @defgroup HAL_GFX_FONT_FUNCTION Functions
  * @{
  */


/**
 *****************************************************************************************
 * @brief Bind the font to use in future hal_gfx_print() calls.
 *
 * @param[in] font: Pointer to font
 *****************************************************************************************
 */
void hal_gfx_bind_font(hal_gfx_font_t *font);


/**
 *****************************************************************************************
 * @brief Get the bounding box's width and height of a string.
 *
 * @param[in] str:  Pointer to string
 * @param[in] w:    Pointer to variable where width should be written
 * @param[in] h:    Pointer to variable where height should be written
 * @param[in] max_w: Max allowed width
 * @param[in] wrap: warp mode
 *
 * @return Number of carriage returns
 *****************************************************************************************
 */
int  hal_gfx_string_get_bbox(const char *str, int *w, int *h, int max_w, uint32_t wrap);


/**
 *****************************************************************************************
 * @brief Print pre-formatted text.
 *
 * @param[in] str:  Pointer to string
 * @param[in] x:    X coordinate of text-area's top-left corner
 * @param[in] y:    Y coordinate of text-area's top-left corner
 * @param[in] w:    Width of the text area
 * @param[in] h:    Height of the text area
 * @param[in] fg_col: Foreground color of text
 * @param[in] align: Alignment and wrapping mode
 *
 * @return
 *****************************************************************************************
 */
void hal_gfx_print(const char *str, int x, int y, int w, int h, uint32_t fg_col, uint32_t align);


/**
 *****************************************************************************************
 * @brief Print pre-formatted text.
 *
 * @param[in] str:  Pointer to string
 * @param[in] pos_x: X position of next character to be drawn. Usually initialized to 0 by the user and then updated internally by the library
 * @param[in] pos_y: Y position of next character to be drawn. Usually initialized to 0 by the user and then updated internally by the library
 * @param[in] x:    X coordinate of text-area's top-left corner
 * @param[in] y:    Y coordinate of text-area's top-left corner
 * @param[in] w:    Width of the text area
 * @param[in] h:    Height of the text area
 * @param[in] fg_col: Foreground color of text
 * @param[in] align: Alignment and wrapping mode
 *
 * @return
 *****************************************************************************************
 */
void hal_gfx_print_to_position(const char *str, int *pos_x, int *pos_y, int x, int y, int w, int h, uint32_t fg_col, uint32_t align);

/** @} */

#ifdef __cplusplus
}
#endif

#endif // HAL_GFX_FONT_H__

/** @} */
/** @} */
/** @} */


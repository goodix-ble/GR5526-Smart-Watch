
/** @addtogroup GRAPHICS_SDK Graphics
 *  @{
 */

/** @defgroup HAL_GFX_ENHANCE Hal gfx enhance
  * @brief Graphics enhance interfaces
  * @{
  */


#ifndef __HAL_GFX_GRAPHICS_ENHANCE_H__
#define __HAL_GFX_GRAPHICS_ENHANCE_H__

#include "hal_gfx_graphics.h"
#include "hal_gfx_math.h"
#include "hal_gfx_raster.h"
#include "hal_gfx_programHW.h"
#include "hal_gfx_cmdlist.h"
#include "hal_gfx_rasterizer.h"

#include "math.h"
#include "string.h"
#include "stdio.h"


#define GPU_BEZIER_VAL_MAX      1024    /**< Max time in Bezier functions (not [0..1] to use integers)*/

/**
 * @defgroup RECT_SIDE_ Defines
 * @{
 */
#define RECT_SIDE_BOTTOM        0x01    /**< Drawing Bottom Side of Rectangle */
#define RECT_SIDE_TOP           0x02    /**< Drawing Top Side of Rectangle */
#define RECT_SIDE_LEFT          0x04    /**< Drawing Left Side of Rectangle */
#define RECT_SIDE_RIGHT         0x08    /**< Drawing Right Side of Rectangle */
#define RECT_SIDE_FULL          0x0F    /**< Drawing all Side of Rectangle */
/** @} */

/**
 * @defgroup ROUND_CORNER_ Defines
 * @{
 */
#define ROUND_CORNER_LEFT       0x01    /**< Drawing Left  Side Corner for Rounded Rect */
#define ROUND_CORNER_UP         0x02    /**< Drawing UP    Side Corner for Rounded Rect */
#define ROUND_CORNER_RIGHT      0x03    /**< Drawing Right Side Corner for Rounded Rect */
#define ROUND_CORNER_DOWN       0x04    /**< Drawing Down  Side Corner for Rounded Rect */
#define ROUND_CORNER_ALL        0x05    /**< Drawing All Corners for Rounded Rect */
/** @} */

/**
 * @defgroup FILL_PARTIAL_CIRCLE_ Defines
 * @{
 */
#define FILL_PARTIAL_CIRCLE_ALL                 0x01    /**< Fill the whole circle     */
#define FILL_PARTIAL_CIRCLE_HALF_UP             0x02    /**< Fill the half-up circle   */
#define FILL_PARTIAL_CIRCLE_HALF_DOWN           0x03    /**< Fill the half-down circle */
#define FILL_PARTIAL_CIRCLE_HALF_LEFT           0x04    /**< Fill the half-left circle */
#define FILL_PARTIAL_CIRCLE_HALF_RIGHT          0x05    /**< Fill the half-right circle */
#define FILL_PARTIAL_CIRCLE_TOP_LEFT_QUAD       0x06    /**< Fill the top-left   1/4 circle */
#define FILL_PARTIAL_CIRCLE_TOP_RIGHT_QUAD      0x07    /**< Fill the top-right  1/4 circle */
#define FILL_PARTIAL_CIRCLE_DOWN_LEFT_QUAD      0x08    /**< Fill the down-left  1/4 circle */
#define FILL_PARTIAL_CIRCLE_DOWN_RIGHT_QUAD     0x09    /**< Fill the down-right 1/4 circle */
/** @} */

#define MAGIC_NUMBER_LV_RADIUS_CIRCLE           0x7FFF  /**< Special Magic number for LV_RADIUS_CIRCLE */

/**
 * @defgroup HAL_GFX_GRAPHICS_ENHANCE_STRUCT Struct
 * @{
 */

/**
  * @brief  Bezier ctrl point information, like x, y, z.
  */
typedef struct b_ctrl_pt
{
    uint32_t u0;
    uint32_t u1;
    uint32_t u2;
    uint32_t u3;
    uint32_t u4;
} b_ctrl_pt_t;
/** @} */

/** @addtogroup HAL_GFX_GRAPHICS_ENHANCE_ENUMERATIONS Enumerations
  * @{
  */

/**
  * @brief   only suport power of 3 & 4 Bezier
  */
typedef enum b_ctrl_power
{
    CUBIC_BEZIER = 3,
    QUARTIC_BEZIER = 4
} b_ctrl_pw_t;

/**
  * @brief  Control the  Bezier postion number
  */
typedef enum b_pos_num
{
    P_64 = 64,
    P_128 = 128,
    P_256 = 256,
    P_512 = 512,
    P_1024 = 1024
} b_ctrl_pos_t;

/** @} */


/**
 * @defgroup HAL_GFX_GRAPHICS_ENHANCE Public Functions
 * @{
 */

/**
 *****************************************************************************************
 * @brief Draw an arc ring with color, use Anti-Aliasing if available, @ref hal_gfx_rgba()
 * @param[in] x: x coordinate of the arc's center
 * @param[in] y: y coordinate of the arc's center
 * @param[in] r: arc's radius
 * @param[in] w: width of arc's edge, r is outer radius, r-w is inner radius.
 * @param[in] start_angle: arc's start angle. value range [0, 360], right is 0 degree, bottom is 90 degree
 * @param[in] end_angle: arc's end angle. value range [0, 360], right is 0 degree, bottom is 90 degree
 * @param[in] is_rounded: whether to handle the side edge of arc to round mode
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_draw_arc(float x, float y, float r, float w, uint16_t start_angle, uint16_t end_angle, bool is_rounded, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Draw an dash line
 * @param[in] x0: x coordinate of the start point for dash-line
 * @param[in] y0: y coordinate of the start point for dash-line
 * @param[in] x1: x coordinate of the end point for dash-line
 * @param[in] y1: y coordinate of the end point for dash-line
 * @param[in] w: dash line's width
 * @param[in] dash_width: line's gap
 * @param[in] dash_gap: dash's gap
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_draw_dash_line(float x0, float y0, float x1, float y1, int w, uint16_t dash_width, uint16_t dash_gap, uint32_t rgba8888);

/**
 * @brief Calculate a value of a Cubic or Quartic Bezier function.
 * @param t time in range of @ref b_ctrl_pos_t, For simplified operation
 * @param pw @ref b_ctrl_pw_t
 * @param pt @ref b_ctrl_pt_t
 * @param re_cal recalculate the bezier coefficients
 * @return the value calculated from the given parameters in range of @ref b_ctrl_pos_t
 */
uint32_t hal_gfx_draw_bezier(uint32_t t, b_ctrl_pos_t max_t, b_ctrl_pw_t pw, b_ctrl_pt_t pt, bool re_cal);

/**
 *****************************************************************************************
 * @brief Enhanced I/F to draw rectangle with edge width
 * @param[in] cl: pointer to command list
 * @param[in] x: x coordinate of the start point
 * @param[in] y: y coordinate of the start point
 * @param[in] w: drawing width for rectangle
 * @param[in] h: drawing height for rectangle
 * @param[in] r: radius
 * @param[in] edge_w: edge line's width. (inner width)
 * @param[in] rgba8888: Color to be used
 * @param[in] draw_side: refer to RECT_SIDE_*
 *****************************************************************************************
 */
void hal_gfx_draw_rounded_rect_x(hal_gfx_cmdlist_t * cl, int x, int y, int w, int h, int r, int edge_w, uint32_t rgba8888, uint32_t draw_side);

/**
 *****************************************************************************************
 * @brief Fill a partial circle, supporting whole circle, half circle and 1/4 circle
 * @param[in] x: x coordinate of the circle's center
 * @param[in] y: y coordinate of the circle's center
 * @param[in] r: circle's radius
 * @param[in] rgba8888: Color to be used
 * @param[in] part : refer to @FILL_PARTIAL_CIRCLE_* defines
 *****************************************************************************************
 */
void hal_gfx_fill_partial_circle_aa(float x, float y, float r, uint32_t rgba8888, uint32_t part);

/**
 *****************************************************************************************
 * @brief Fill a rectangle with rounded edges with color in Anti-Aliasing mode
 *        1. if r <= 2. fill in rectangle mode
 *        2. if (r == 0x7FFF) and (w==h && w == 2*r), fill as circle
 * @param[in] x0: x coordinate of the upper left vertex of the rectangle
 * @param[in] y0: y coordinate at the upper left vertex of the rectangle
 * @param[in] w: width of the rectangle
 * @param[in] h: height of the rectangle
 * @param[in] r: corner radius
 * @param[in] rgba8888: Color to be used
 *****************************************************************************************
 */
void hal_gfx_fill_rounded_rect_aa(int x0, int y0, int w, int h, int r, uint32_t rgba8888);

/**
 *****************************************************************************************
 * @brief Fill a rectangle with optional rounded edges with color in Anti-Aliasing mode
 *        1. if r <= 2. fill in rectangle mode
 *        2. if (r == 0x7FFF) and (w==h && w == 2*r), fill as circle
 * @param[in] x0: x coordinate of the upper left vertex of the rectangle
 * @param[in] y0: y coordinate at the upper left vertex of the rectangle
 * @param[in] w: width of the rectangle
 * @param[in] h: height of the rectangle
 * @param[in] r: corner radius
 * @param[in] rgba8888: Color to be used
 * @param[in] round_corner: which side to drsw round rect, refer to @ROUND_CORNER_*
 *****************************************************************************************
 */
void hal_gfx_fill_optional_rounded_rect_aa(int x0, int y0, int w, int h, int r, uint32_t rgba8888, uint32_t round_corner);

/** @} */

#endif /* __HAL_GFX_GRAPHICS_ENHANCE_H__ */

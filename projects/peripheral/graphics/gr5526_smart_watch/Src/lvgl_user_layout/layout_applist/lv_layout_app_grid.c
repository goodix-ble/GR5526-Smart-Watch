#include <stdio.h>

#include "lvgl.h"
#include "lv_font.h"
#include "lv_img_dsc_list.h"
#include "hal_gfx_math.h"
#include "app_graphics_mem.h"
#include "lv_layout_router.h"
#include <math.h>

/*********************
 *      DEFINES
 *********************/
#define ARRAY_SIZE(arr) (sizeof((arr)) / sizeof((arr)[0]))

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static lv_point_t s_last_scroll_offset = {INT16_MAX, INT16_MAX};
static bool s_last_use_triform = false;

/*
 * STATIC METHODS DECLARATION
 *****************************************************************************************
 */
static void app_grid_delete_cb(lv_event_t *event);
static float app_grid_sphere_translate_cb(lv_point_t *icon_pos, const lv_enhanced_menu_item_t *item);
static float app_grid_triform_translate_cb(lv_point_t *icon_pos, const lv_enhanced_menu_item_t *item);

/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */

/*
 * GLOBAL METHODS IMPLEMENT
 *****************************************************************************************
 */
extern void lv_port_res_mode_set(uint8_t mode);

lv_obj_t *lv_layout_app_grid_create(lv_obj_t *parent, lv_enhanced_menu_item_t *items, uint16_t item_count, uint8_t icon_per_row, bool use_triform)
{
    lv_obj_t *grid = lv_enhanced_grid_create(parent);
    lv_enhanced_grid_set_items(grid, items, item_count, icon_per_row);
    if (use_triform)
    {
        lv_enhanced_grid_set_hex_layout(grid, false);
        lv_enhanced_grid_set_scroll_dir(grid, LV_DIR_VER);
        lv_enhanced_grid_set_size_translate_cb(grid, app_grid_triform_translate_cb);
        lv_enhanced_grid_set_cell_size(grid, 100, 120);
        lv_enhanced_grid_set_padding(grid, 77, DISP_VER_RES / 3);
        lv_point_t scroll_offset = {77, (DISP_VER_RES - 100) / 2};
        if (s_last_use_triform && s_last_scroll_offset.y != INT16_MAX)
        {
            scroll_offset.y = s_last_scroll_offset.y;
        }
        s_last_use_triform = true;
        lv_enhanced_grid_set_scroll_offset(grid, scroll_offset, false);
        lv_layout_router_set_scrollbar_target(grid);
    }
    else
    {
        lv_enhanced_grid_set_hex_layout(grid, true);
        lv_enhanced_grid_set_scroll_dir(grid, LV_DIR_ALL);
        lv_enhanced_grid_set_size_translate_cb(grid, app_grid_sphere_translate_cb);
        lv_enhanced_grid_set_cell_size(grid, 150, 150);
        lv_enhanced_grid_set_padding(grid, DISP_HOR_RES / 2 - 75, DISP_VER_RES / 2 - 75);
        if (!s_last_use_triform && s_last_scroll_offset.x != INT16_MAX)
        {
            lv_enhanced_grid_set_scroll_offset(grid, s_last_scroll_offset, false);
        }
        else
        {
            lv_enhanced_grid_scroll_to_center(grid, false);
        }
        s_last_use_triform = false;
    }
    lv_obj_add_event_cb(grid, app_grid_delete_cb, LV_EVENT_DELETE, NULL);
    return grid;
}

/*
 * STATIC METHODS IMPLEMENT
 *****************************************************************************************
 */

static void app_grid_delete_cb(lv_event_t *event)
{
    s_last_scroll_offset = lv_enhanced_grid_get_scroll_offset(event->current_target);
}

static float app_grid_sphere_translate_cb(lv_point_t *icon_pos, const lv_enhanced_menu_item_t *item)
{
#define SQUARE(x)         ((x) * (x))
#define X_CENTER          (DISP_HOR_RES / 2)
#define Y_CENTER          (DISP_VER_RES / 2)

#define ICON_SIZE         72
#define ICON_RADIUS       (ICON_SIZE / 2)
#define R_CORNER          227

#if ICON_RADIUS > R_CORNER
#define EDGE_DET_THRESHOLD ICON_RADIUS
#else
#define EDGE_DET_THRESHOLD R_CORNER
#endif //  ICON_RADIUS > R_CORNER

#define INNER_WIDTH_HALF  (((int)DISP_HOR_RES / 2) - ICON_RADIUS)
#define INNER_HEIGHT_HALF (((int)DISP_HOR_RES / 2) - ICON_RADIUS)

#define SPHERE_RADIUS     700.f
#define DISTORTION_COEFF  1.9f
#define PYTHAG(x, y)      hal_gfx_sqrt(SQUARE(x) + SQUARE(y))
#define PYTHAG3D(x, y, z) hal_gfx_sqrt(SQUARE(x) + SQUARE(y) + SQUARE(z))

    (void)item;

    float scale = 0;

    float cx = icon_pos->x - (lv_coord_t)X_CENTER;
    float cy = icon_pos->y - (lv_coord_t)Y_CENTER;
    float r2d = PYTHAG(cx, cy);

    if (r2d > 1.f)
    {
        float r3d = PYTHAG3D(cx, cy, SPHERE_RADIUS);
        float force = (r3d - SPHERE_RADIUS) * DISTORTION_COEFF;
        icon_pos->x -= force * cx / r3d;
        icon_pos->y -= force * cy / r3d;
    }

    bool on_edge = false;
    float ex = 0;
    float ey = 0;

#if (EDGE_DET_THRESHOLD == X_CENTER) && (EDGE_DET_THRESHOLD == Y_CENTER)
    on_edge = true;
    ex = EDGE_DET_THRESHOLD - icon_pos->x;
    ey = EDGE_DET_THRESHOLD - icon_pos->y;
#else
    if (icon_pos->x < EDGE_DET_THRESHOLD)
    {
        ex = EDGE_DET_THRESHOLD - icon_pos->x;
        on_edge = true;
    }
    else if (icon_pos->x > ((int)DISP_HOR_RES - EDGE_DET_THRESHOLD))
    {
        ex = (int)DISP_HOR_RES - icon_pos->x - EDGE_DET_THRESHOLD;
        on_edge = true;
    }

    if (icon_pos->y < EDGE_DET_THRESHOLD)
    {
        ey = EDGE_DET_THRESHOLD - icon_pos->y;
        on_edge = true;
    }
    else if (icon_pos->y > ((int)DISP_VER_RES - EDGE_DET_THRESHOLD))
    {
        ey = (int)DISP_VER_RES - icon_pos->y - EDGE_DET_THRESHOLD;
        on_edge = true;
    }
#endif // (EDGE_DET_THRESHOLD == X_CENTER) && (EDGE_DET_THRESHOLD == Y_CENTER)

    if (on_edge)
    {
        float rr = PYTHAG(ex, ey);
        float dist2edge = EDGE_DET_THRESHOLD - rr;

        if (dist2edge < (float)ICON_RADIUS)
        {
            scale = dist2edge / ICON_SIZE + 0.5f;
            icon_pos->x += (1.f - scale) * (float)ICON_RADIUS * (ex / rr);
            icon_pos->y += (1.f - scale) * (float)ICON_RADIUS * (ey / rr);
        }
        else
        {
            on_edge = false;
        }
    }

    // This cannot be simplified to if(on_edge){}else{}
    // Since in previous if-block value of on_edge may be changed
    if (!on_edge)
    {
        float scale_x = LV_MAX((INNER_WIDTH_HALF - fabsf(cx)) / INNER_WIDTH_HALF, 0);
        float scale_y = LV_MAX((INNER_HEIGHT_HALF - fabsf(cy)) / INNER_HEIGHT_HALF, 0);
        scale = 1.f + 0.6f * scale_x * scale_y;
        if (scale < 1.f)
        {
            scale = 1.f;
        }
    }

    return scale;
}

static float app_grid_triform_translate_cb(lv_point_t *icon_pos, const lv_enhanced_menu_item_t *item)
{
#define SCREEN_RADIUS 240.f
    float dist = fabsf(icon_pos->y - 227.f);
    float scale = 0;
    if (dist < SCREEN_RADIUS)
    {
        scale = 1.5f * sqrtf(SQUARE(SCREEN_RADIUS) - SQUARE(dist)) / SCREEN_RADIUS;
    }

    float xofs = (scale - 1.f) * item->icon->header.w;
    float yofs = (scale - 1.f) * item->icon->header.h;
    if (icon_pos->x != 227)
    {
        icon_pos->x += xofs * copysignf(1.f, icon_pos->x - 227.f);
    }

    return scale;
}

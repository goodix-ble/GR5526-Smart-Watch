#include <stdio.h>

#include "lvgl.h"
#include "lv_font.h"
#include "lv_img_dsc_list.h"
#include "lv_layout_router.h"

/*********************
 *      DEFINES
 *********************/
#define ARRAY_SIZE(arr) (sizeof((arr)) / sizeof((arr)[0]))

#define LIST_RADIUS     280

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static lv_coord_t   s_last_scroll_offset = INT16_MAX;

/*
 * STATIC METHODS DECLARATION
 *****************************************************************************************
 */
static void app_list_delete_cb(lv_event_t *event);
static lv_coord_t app_list_item_circular_translate_cb(lv_coord_t obj_y, const lv_enhanced_menu_item_t *item);
static lv_coord_t app_list_item_linear_translate_cb(lv_coord_t obj_y, const lv_enhanced_menu_item_t *item);

/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */

/*
 * GLOBAL METHODS IMPLEMENT
 *****************************************************************************************
 */
lv_obj_t *lv_layout_app_list_create(lv_obj_t *parent, lv_enhanced_menu_item_t *items, uint16_t item_count, bool use_circular)
{
    lv_obj_t *list = lv_enhanced_list_create(parent);
    lv_obj_set_style_text_font(list, &lv_font_montserrat_30, 0);
    if (use_circular)
    {
        lv_enhanced_list_set_translate_cb(list, app_list_item_circular_translate_cb);
    }
    else
    {
        lv_enhanced_list_set_translate_cb(list, app_list_item_linear_translate_cb);
    }
    lv_enhanced_list_set_items(list, items, item_count);
    lv_obj_add_event_cb(list, app_list_delete_cb, LV_EVENT_DELETE, NULL);

    if (s_last_scroll_offset != INT16_MAX)
    {
        lv_enhanced_list_set_scroll_offset(list, s_last_scroll_offset, false);
    }

    lv_layout_router_set_scrollbar_target(list);
    return list;
}

/*
 * STATIC METHODS IMPLEMENT
 *****************************************************************************************
 */
static void app_list_delete_cb(lv_event_t *event)
{
    s_last_scroll_offset = lv_enhanced_list_get_scroll_offset(event->current_target);
}

static lv_coord_t app_list_item_circular_translate_cb(lv_coord_t obj_y, const lv_enhanced_menu_item_t *item)
{
    lv_coord_t x_trans;

    lv_coord_t y_center = obj_y + item->icon->header.h / 2;
    lv_coord_t y_diff = y_center - DISP_VER_RES / 2;
    y_diff = LV_ABS(y_diff);

    if (y_diff >= LIST_RADIUS)
    {
        x_trans = LIST_RADIUS;
    }
    else
    {
        uint32_t x_sqr = LIST_RADIUS * LIST_RADIUS - y_diff * y_diff;
        lv_sqrt_res_t res;
        lv_sqrt(x_sqr, &res, 0x8000);
        x_trans = LIST_RADIUS - res.i;
    }

    return x_trans;
}

static lv_coord_t app_list_item_linear_translate_cb(lv_coord_t obj_y, const lv_enhanced_menu_item_t *item)
{
    return 20;
}

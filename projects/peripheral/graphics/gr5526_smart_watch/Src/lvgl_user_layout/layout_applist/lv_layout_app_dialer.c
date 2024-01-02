#include <stdio.h>

#include "lvgl.h"
#include "lv_font.h"
#include "lv_img_dsc_list.h"
#include "lv_layout_router.h"

/*********************
 *      DEFINES
 *********************/
#define ARRAY_SIZE(arr) (sizeof((arr)) / sizeof((arr)[0]))

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static lv_coord_t s_last_scroll_offset = INT16_MAX;

/*
 * STATIC METHODS DECLARATION
 *****************************************************************************************
 */
static void app_dialer_delete_cb(lv_event_t *event);
static void app_dialer_focus_change_cb(lv_event_t *event);

/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */

/*
 * GLOBAL METHODS IMPLEMENT
 *****************************************************************************************
 */
lv_obj_t *lv_layout_app_dialer_create(lv_obj_t *parent, lv_enhanced_menu_item_t *items, uint16_t item_count)
{
    lv_obj_t *list = lv_circular_list_create(parent);
    lv_circular_list_set_items(list, items, item_count);

    lv_obj_t *label = lv_label_create(list);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_30, 0);
    lv_label_set_text_static(label, items[0].caption);
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -100);

    lv_obj_add_event_cb(list, app_dialer_focus_change_cb, LV_EVENT_VALUE_CHANGED, (void *)label);
    lv_obj_add_event_cb(list, app_dialer_delete_cb, LV_EVENT_DELETE, NULL);

    if (s_last_scroll_offset != INT16_MAX)
    {
        lv_circular_list_set_scroll_offset(list, s_last_scroll_offset, false);
    }


    return list;
}

/*
 * STATIC METHODS IMPLEMENT
 *****************************************************************************************
 */
static void app_dialer_delete_cb(lv_event_t *event)
{
    s_last_scroll_offset = lv_circular_list_get_scroll_offset(event->current_target);
}

static void app_dialer_focus_change_cb(lv_event_t *event)
{
    lv_obj_t *label = (lv_obj_t *)event->user_data;
    lv_obj_t *list = event->current_target;

    const lv_enhanced_menu_item_t *item = lv_circular_list_get_focused_item(list);
    if (item)
    {
        lv_label_set_text_static(label, item->caption);
    }
}

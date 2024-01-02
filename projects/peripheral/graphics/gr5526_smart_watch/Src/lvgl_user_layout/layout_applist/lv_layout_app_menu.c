#include <stdio.h>
#include "gr55xx.h"
#include "lvgl.h"
#include "lv_circular_list.h"
#include "lv_img_dsc_list.h"
#include "lv_layout_router.h"
#include "lv_layout_app_menu.h"
#include "app_graphics_mem.h"

#define ARRAY_SIZE(arr) (sizeof((arr)) / sizeof((arr)[0]))

#define PSRAM_ICON_CACHE_ENABLED 1

typedef void (*app_menu_jump_cb_t)(lv_obj_t *);

lv_enhanced_menu_item_t app_list_items[] = {
    {           &wd_img_APPLIST_00_MENU,            "MENU", (void *)WMS_WID_NONE},
    {         &wd_img_APPLIST_01_SPORTS,          "SPORTS", (void *)WMS_WID_NONE},
    {      &wd_img_APPLIST_02_ACTIVITES,       "ACTIVITES", (void *)WMS_WID_NONE},
    {      &wd_img_APPLIST_03_HEARTRATE,       "HEARTRATE", (void *)WMS_WID_NONE},
    {           &wd_img_APPLIST_04_SPO2,           "SPO2 ", (void *)WMS_WID_NONE},
    {         &wd_img_APPLIST_05_STRESS,          "STRESS", (void *)WMS_WID_NONE},
    {          &wd_img_APPLIST_06_SLEEP,           "SLEEP", (void *)WMS_WID_NONE},
    {            &wd_img_APPLIST_16_ECG,             "ECG", (void *)WMS_WID_NONE},
    {&wd_img_APPLIST_07_BREATH_TRAINING, "BREATH TRAINING", (void *)WMS_WID_NONE},
    {        &wd_img_APPLIST_08_WEATHER,         "WEATHER", (void *)WMS_WID_NONE},
    {          &wd_img_APPLIST_09_MUSIC,           "MUSIC", (void *)WMS_WID_NONE},
    {            &wd_img_APPLIST_10_NFC,             "NFC", (void *)WMS_WID_NONE},
    {          &wd_img_APPLIST_11_ALARM,           "ALARM", (void *)WMS_WID_NONE},
    {      &wd_img_APPLIST_12_STOPWATCH,       "STOPWATCH", (void *)WMS_WID_NONE},
    {          &wd_img_APPLIST_13_TIMER,           "TIMER", (void *)WMS_WID_NONE},
    {     &wd_img_APPLIST_14_FIND_PHONE,      "FIND PHONE", (void *)WMS_WID_NONE},
    {       &wd_img_APPLIST_15_SETTINGS,        "SETTINGS", (void *)WMS_WID_SETTING_ENTRY},
};

const uint16_t LIST_ITEM_COUNT = ARRAY_SIZE(app_list_items);

static app_menu_style_t s_menu_style = APP_MENU_STYLE_LINEAR_LIST;

extern lv_obj_t *lv_layout_app_list_create(lv_obj_t *parent, lv_enhanced_menu_item_t *items, uint16_t item_count, bool use_circular);
extern lv_obj_t *lv_layout_app_grid_create(lv_obj_t *parent, lv_enhanced_menu_item_t *items, uint16_t item_count, uint8_t icon_per_row, bool use_triform);
extern lv_obj_t *lv_layout_app_dialer_create(lv_obj_t *parent, lv_enhanced_menu_item_t *items, uint16_t item_count);

#if PSRAM_ICON_CACHE_ENABLED
static void create_icon_psram_cache(void);
#endif // PSRAM_ICON_CACHE_ENABLED
static void app_menu_item_clicked_cb(lv_event_t *event);
static void app_menu_gesture_callback(lv_event_t *event);

lv_obj_t *lv_layout_app_menu_create(lv_obj_t *parent)
{
    lv_obj_t *menu = NULL;
#if PSRAM_ICON_CACHE_ENABLED
    create_icon_psram_cache();
#endif // PSRAM_ICON_CACHE_ENABLED
    switch (s_menu_style)
    {
        case APP_MENU_STYLE_CIRCULAR_LIST:
            menu = lv_layout_app_list_create(parent, app_list_items, LIST_ITEM_COUNT, true);
            break;

        case APP_MENU_STYLE_LINEAR_LIST:
            menu = lv_layout_app_list_create(parent, app_list_items, LIST_ITEM_COUNT, false);
            break;

        case APP_MENU_STYLE_SPHERE_GRID:
            menu = lv_layout_app_grid_create(parent, app_list_items, LIST_ITEM_COUNT, 4, false);
            break;

        case APP_MENU_STYLE_TRIFORM_GRID:
            menu = lv_layout_app_grid_create(parent, app_list_items, LIST_ITEM_COUNT, 3, true);
            break;

        case APP_MENU_STYLE_DIALER:
            menu = lv_layout_app_dialer_create(parent, app_list_items, LIST_ITEM_COUNT);
            break;

        default:
            printf("Invalid App Menu Style: %d\n", s_menu_style);
            while(1);
    }

    lv_obj_add_event_cb(menu, app_menu_item_clicked_cb, LV_EVENT_SHORT_CLICKED, NULL);
    lv_obj_add_event_cb(menu, app_menu_gesture_callback, LV_EVENT_GESTURE, NULL);
    lv_obj_clear_flag(menu, LV_OBJ_FLAG_GESTURE_BUBBLE);
    return menu;
}

app_menu_style_t app_menu_get_style(void)
{
    return s_menu_style;
}

void app_menu_set_style(app_menu_style_t style)
{
    if (style >= APP_MENU_STYLE_MAX)
    {
        style = APP_MENU_STYLE_CIRCULAR_LIST;
    }
    s_menu_style = style;
}

bool app_menu_gesture_available(lv_obj_t *obj, lv_dir_t gesture_dir)
{
    if (s_menu_style == APP_MENU_STYLE_SPHERE_GRID)
    {
        return (lv_enhanced_grid_is_on_edge(obj) & gesture_dir) ? true : false;
    }
    else if (s_menu_style == APP_MENU_STYLE_CIRCULAR_LIST
            || s_menu_style == APP_MENU_STYLE_LINEAR_LIST
            || s_menu_style == APP_MENU_STYLE_TRIFORM_GRID)
    {
        return (gesture_dir & LV_DIR_HOR) ? true : false;
    }
    else if (s_menu_style == APP_MENU_STYLE_DIALER)
    {
        return (lv_circular_list_is_on_edge(obj) & gesture_dir) ? true : false;
    }

    return true;
}

#if PSRAM_ICON_CACHE_ENABLED
static void create_icon_psram_cache()
{
    extern void lv_port_res_mode_set(uint8_t mode);
    lv_port_res_mode_set(2);
    for (uint16_t i = 0; i < LIST_ITEM_COUNT; i++)
    {
        lv_enhanced_menu_item_t *item = &app_list_items[i];
        if ((uint32_t)item->icon->data >= OSPI0_XIP_BASE && (uint32_t)item->icon->data <= (OSPI0_XIP_BASE + 0x03FFFFFF))
        {
            continue;
        }
        lv_img_dsc_t *img_dsc = lv_mem_alloc(sizeof(lv_img_dsc_t));
        memcpy((void *)img_dsc, item->icon, sizeof(lv_img_dsc_t));
        img_dsc->data = app_graphics_mem_malloc(img_dsc->data_size);
        memcpy((void *)img_dsc->data, item->icon->data, img_dsc->data_size);
        item->icon = img_dsc;
    }
}
#endif // PSRAM_ICON_CACHE_ENABLED

static void app_menu_item_clicked_cb(lv_event_t *event)
{
    extern const lv_obj_class_t lv_enhanced_list_class;
    extern const lv_obj_class_t lv_enhanced_grid_class;
    extern const lv_obj_class_t lv_circular_list_class;

    lv_obj_t *menu = event->current_target;
    const lv_enhanced_menu_item_t *clicked_item = NULL;
    if (menu->class_p == &lv_enhanced_list_class)
    {
        clicked_item = lv_enhanced_list_get_clicked_item(menu);
    }
    else if (menu->class_p == &lv_enhanced_grid_class)
    {
        clicked_item = lv_enhanced_grid_get_clicked_item(menu);
    }
    else if (menu->class_p == &lv_circular_list_class)
    {
        clicked_item = lv_circular_list_get_focused_item(menu);
    }

    if (clicked_item)
    {
        printf("\"%s\" is clicked!\n", clicked_item->caption);

        lv_layout_router_goto_isolate_win(menu, (uint32_t) clicked_item->user_data, WMS_DEFAULT_GOTO_EFFECT_POS, WMS_DEFAULT_GOTO_EFFECT);
    }
}

static void app_menu_gesture_callback(lv_event_t *event)
{
    lv_indev_t *indev = lv_event_get_indev(event);
    if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT)
    {
        if (app_menu_gesture_available(event->current_target, LV_DIR_LEFT))
        {
            lv_layout_router_back(event->current_target);
        }
    }
}

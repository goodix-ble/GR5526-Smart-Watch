#ifndef _LV_LAYOUT_ROUTER_H
#define _LV_LAYOUT_ROUTER_H

#include "lvgl.h"

typedef enum {
    WMS_WID_NONE = 0,
    WMS_WID_CARD_WATCHFACE,
    WMS_WID_CARD_STATUS_BAR,
    WMS_WID_CARD_APP_ENTRY,
    WMS_WID_CARD_NOTIFICATION,
    WMS_WID_CARD_DAY_ACTIVITY,
    WMS_WID_CARD_HR,
    WMS_WID_CARD_STRESS,
    WMS_WID_CARD_SLEEP,
    WMS_WID_CARD_WEATHER,

    WMS_WID_APP_LIST,
    WMS_WID_SETTING_ENTRY,
    WMS_WID_SETTING_LIST_STYLE,
    WMS_WID_SETTING_SWITCH_EFFECT,
    WMS_WID_SETTING_CARD_STYLE,

    WMS_WID_POWER_OFF,

    WMS_WID_MAX,
} lv_wms_wid_e;


enum {
    TILEVIEW_MAP_ID_MAIN_SCREEN = 0,
    TILEVIEW_MAP_ID_MAIN_SCREEN_HOR = TILEVIEW_MAP_ID_MAIN_SCREEN,
    TILEVIEW_MAP_ID_MAIN_SCREEN_VER,
    TILEVIEW_MAP_ID_HR_SUB_LAYOUT,
    TILEVIEW_MAP_ID_STRESS_SUB_LAYOUT,
    TILEVIEW_MAP_ID_APP_LIST,

    TILEVIEW_MAP_ID_MAX,
};

#define TILEVIEW_MAP_ISOLATE_FLAG       0x8000
#define TILEVIEW_MAP_ISOLATE_MASK       0x7FFF

#define WMS_DEFAULT_CARD_EFFECT         LV_WMS_TILEVIEW_EFFECT_CUBE
#define WMS_DEFAULT_GOTO_EFFECT         LV_WMS_TILEVIEW_EFFECT_FADE_ZOOM_ALT
#define WMS_DEFAULT_GOTO_EFFECT_POS     LV_WMS_TILEVIEW_POS_AT_RIGHT

void lv_layout_router_init(void);

void lv_layout_router_set_scrollbar_target(lv_obj_t *target);

lv_obj_t * lv_layout_router_get_active_obj(void);

lv_wms_tileview_tile_info_t * lv_layout_router_get_active_tile_info(void);

void lv_layout_router_back(lv_obj_t * from_obj);

void lv_layout_router_goto_isolate_win(lv_obj_t * from_obj, uint32_t win_id, lv_wms_tileview_pos_t birthplace, lv_wms_tileview_transition_effect_t effect);

#endif

#include <stdio.h>
#include "lv_layout_router.h"
#include "lv_layout_app_menu.h"

#define WMS_WIN_POS(M,R,C)          ((((uint32_t)M << 16) & 0xffff0000) | (((uint16_t)R << 8) & 0xFF00) | ((uint16_t)C & 0xff))



typedef struct {
    lv_wms_wid_e    win_id;
    bool            (*gesture_checker)(lv_obj_t * obj, int16_t row, int16_t col);
    lv_obj_t *      (* layout_creator)(lv_obj_t *);
} lv_wms_wid_map_t;


typedef struct{
    int32_t     win_pos;
    uint8_t     effect;
    bool        is_retain;
    bool        is_symmetry_effect;
    lv_obj_t *  (*layout_creator)(lv_obj_t *);
} lv_wms_layout_map_t;

static bool                 is_cardflow_layout_horizontal  = false;
static lv_obj_t *           s_top_scrollbar = NULL;
static lv_wms_tileview_t *  s_root_tileview = NULL;
static lv_obj_t * lv_layout_router_cb(lv_obj_t * obj, const lv_wms_tileview_tile_info_t * p_old_tile, const lv_wms_tileview_tile_info_t * p_new_tile, lv_wms_tileview_tile_cfg_t * const p_new_tile_cfg);
static lv_obj_t * lv_layout_exit_current(lv_obj_t * obj);
static lv_obj_t * lv_layout_none(lv_obj_t * obj);
static bool       lv_default_gesture_checker(lv_obj_t * obj, int16_t row, int16_t col);
static bool       lv_applist_gesture_checker(lv_obj_t * obj, int16_t row, int16_t col);

extern lv_obj_t *lv_card_watchface_vivid_layout_create(lv_obj_t * parent);
extern lv_obj_t *lv_card_day_activity_layout_create(lv_obj_t * parent);
extern lv_obj_t *lv_card_list_entry_layout_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_card_notification_layout_create(lv_obj_t * parent_tv_obj);
extern lv_obj_t *lv_card_status_layout_create(lv_obj_t * parent);
extern lv_obj_t *lv_card_layout_sleep_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_card_layout_weather_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_card_layout_heartrate_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_card_layout_stress_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_app_menu_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_setting_create(lv_obj_t * parent);
extern lv_obj_t *lv_layout_setting_list_style_create(lv_obj_t * parent);
extern lv_obj_t *lv_layout_settings_switch_effect_create(lv_obj_t * parent);
extern lv_obj_t *lv_layout_setting_cardflow_style_create(lv_obj_t * parent);
extern lv_obj_t *lv_layout_pwr_off_layout_create(lv_obj_t * parent);
extern lv_obj_t *lv_layout_heartrate_chart_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_heartrate_statics_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_spo2_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_stress_chart_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_stress_statics_create(lv_obj_t *parent_tv_obj);


const static lv_wms_wid_map_t g_win_id_map[] = {
    {WMS_WID_NONE,                  lv_default_gesture_checker, lv_layout_none},
    {WMS_WID_CARD_WATCHFACE,        lv_default_gesture_checker, lv_card_watchface_vivid_layout_create},
    {WMS_WID_CARD_STATUS_BAR,       lv_default_gesture_checker, lv_card_status_layout_create},
    {WMS_WID_CARD_APP_ENTRY,        lv_default_gesture_checker, lv_card_list_entry_layout_create},
    {WMS_WID_CARD_NOTIFICATION,     lv_default_gesture_checker, lv_card_notification_layout_create},
    {WMS_WID_CARD_DAY_ACTIVITY,     lv_default_gesture_checker, lv_card_day_activity_layout_create},
    {WMS_WID_CARD_HR,               lv_default_gesture_checker, lv_card_layout_heartrate_create},
    {WMS_WID_CARD_STRESS,           lv_default_gesture_checker, lv_card_layout_stress_create},
    {WMS_WID_CARD_SLEEP,            lv_default_gesture_checker, lv_card_layout_sleep_create},
    {WMS_WID_CARD_WEATHER,          lv_default_gesture_checker, lv_card_layout_weather_create},
    {WMS_WID_APP_LIST,              lv_applist_gesture_checker, lv_layout_app_menu_create},
    {WMS_WID_SETTING_ENTRY,         lv_default_gesture_checker, lv_layout_setting_create},
    {WMS_WID_SETTING_LIST_STYLE,    lv_default_gesture_checker, lv_layout_setting_list_style_create},
    {WMS_WID_SETTING_SWITCH_EFFECT, lv_default_gesture_checker, lv_layout_settings_switch_effect_create},
    {WMS_WID_SETTING_CARD_STYLE,    lv_default_gesture_checker, lv_layout_setting_cardflow_style_create},
    {WMS_WID_POWER_OFF,             lv_default_gesture_checker, lv_layout_pwr_off_layout_create},

};

 lv_wms_layout_map_t g_tileview_map[] ={
    /* MAP: HR_SUB_LAYOUT */
    {WMS_WIN_POS(TILEVIEW_MAP_ID_HR_SUB_LAYOUT, 0,  0),           LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_layout_heartrate_chart_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_HR_SUB_LAYOUT, 1,  0),           LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_layout_heartrate_statics_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_HR_SUB_LAYOUT, 0, -1),           LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_layout_exit_current},

    /* MAP: STRESS_SUB_LAYOUT */
    {WMS_WIN_POS(TILEVIEW_MAP_ID_STRESS_SUB_LAYOUT, 0,  0),       LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_layout_stress_chart_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_STRESS_SUB_LAYOUT, 1,  0),       LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_layout_stress_statics_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_STRESS_SUB_LAYOUT, 0, -1),       LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_layout_exit_current},

    /* MAP: MAIN_SCREEN_HOR */
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_HOR, -1,  0),        LV_WMS_TILEVIEW_EFFECT_STACK,   true , true , lv_card_status_layout_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_HOR,  1,  0),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_card_notification_layout_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_HOR,  0, -1),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_card_list_entry_layout_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_HOR,  0,  0),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, true , false, lv_card_watchface_vivid_layout_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_HOR,  0,  1),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_card_day_activity_layout_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_HOR,  0,  2),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_card_layout_heartrate_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_HOR,  0,  3),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_card_layout_stress_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_HOR,  0,  4),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_card_layout_weather_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_HOR,  0,  5),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_card_layout_sleep_create},

    /* MAP: MAIN_SCREEN_VER */
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_VER,  0, -1),        LV_WMS_TILEVIEW_EFFECT_SPIN,    false, true , lv_card_notification_layout_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_VER,  0,  1),        LV_WMS_TILEVIEW_EFFECT_SPIN,    false, true , lv_card_list_entry_layout_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_VER, -1,  0),        LV_WMS_TILEVIEW_EFFECT_STACK,   true , true , lv_card_status_layout_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_VER,  0,  0),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, true , false, lv_card_watchface_vivid_layout_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_VER,  1,  0),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_card_day_activity_layout_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_VER,  2,  0),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_card_layout_heartrate_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_VER,  3,  0),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_card_layout_stress_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_VER,  4,  0),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_card_layout_weather_create},
    {WMS_WIN_POS(TILEVIEW_MAP_ID_MAIN_SCREEN_VER,  5,  0),        LV_WMS_TILEVIEW_EFFECT_DEFAULT, false, false, lv_card_layout_sleep_create},
};


void lv_layout_router_init(void)
{
    lv_obj_t * scr  = lv_scr_act();
    s_root_tileview = lv_wms_tileview_create(scr, lv_layout_router_cb);
    s_root_tileview->root_tile_info =  s_root_tileview->current_tile_info;
    lv_obj_set_size(&s_root_tileview->obj, LV_PCT(100), LV_PCT(100));
    s_root_tileview->default_effect = WMS_DEFAULT_CARD_EFFECT;
    s_top_scrollbar = lv_enhanced_scrollbar_create(lv_disp_get_layer_top(NULL));
}

lv_obj_t * lv_layout_router_get_active_obj(void) {
    if(s_root_tileview != NULL) {
        return s_root_tileview->p_current_tile_obj;
    }

    return NULL;
}

lv_wms_tileview_tile_info_t * lv_layout_router_get_active_tile_info(void) {
    return &(s_root_tileview->current_tile_info);
}

void lv_layout_router_set_slide_effect(uint32_t effect) {
    bool clear_retain = false;
    if(s_root_tileview) {
        switch(effect) {
            case LV_WMS_TILEVIEW_EFFECT_CUBE        :
            case LV_WMS_TILEVIEW_EFFECT_INNERCUBE   :
            case LV_WMS_TILEVIEW_EFFECT_STACK       :
            case LV_WMS_TILEVIEW_EFFECT_FADE        :
            case LV_WMS_TILEVIEW_EFFECT_FADE_ZOOM   :
            case LV_WMS_TILEVIEW_EFFECT_SPIN        :
            case LV_WMS_TILEVIEW_EFFECT_PUSHPULL    :
            case LV_WMS_TILEVIEW_EFFECT_LINEAR      :
            {
                clear_retain = (s_root_tileview->default_effect != effect) ? true : false;
                s_root_tileview->default_effect = effect;
            }
            break;

            default:
            {}
            break;
        }
    }

    if(clear_retain) {
        lv_wms_tileview_clear_retain(s_root_tileview);
    }

    return;
}

uint32_t lv_layout_router_get_slide_effect(void) {
    if(s_root_tileview) {
        return s_root_tileview->default_effect;
    }

    return LV_WMS_TILEVIEW_EFFECT_CUBE;
}


void lv_layout_router_set_scrollbar_target(lv_obj_t *target)
{
    lv_enhanced_scrollbar_set_target(s_top_scrollbar, target);
}

void lv_layout_router_back(lv_obj_t * from_obj)
{
    lv_wms_tileview_pop(from_obj);
}


void lv_layout_router_goto_map(lv_obj_t * from_obj, int map_id, lv_wms_tileview_pos_t birthplace, lv_wms_tileview_transition_effect_t effect)
{
    lv_wms_tileview_push(from_obj, map_id, 0, 0, birthplace, effect, 0, NULL);
}

void lv_layout_router_goto_isolate_win(lv_obj_t * from_obj, uint32_t win_id, lv_wms_tileview_pos_t birthplace, lv_wms_tileview_transition_effect_t effect)
{
    lv_wms_tileview_push(from_obj, TILEVIEW_MAP_ISOLATE_FLAG | win_id, 0, 0, birthplace, effect, 0, NULL);
}


void lv_layout_set_cardflow_style(bool is_hori)
{
    is_cardflow_layout_horizontal = is_hori;
}

bool lv_layout_get_cardflow_style(void)
{
    return is_cardflow_layout_horizontal;
}

/******************************************************************************************
 *                    Static Functions
 ******************************************************************************************/

static lv_obj_t * lv_layout_exit_current(lv_obj_t * obj)
{
    lv_wms_tileview_pop(obj);
    return NULL;
}

static lv_obj_t * lv_layout_router_cb(lv_obj_t * obj, const lv_wms_tileview_tile_info_t * p_old_tile, const lv_wms_tileview_tile_info_t * p_new_tile, lv_wms_tileview_tile_cfg_t * const p_new_tile_cfg)
{
    int16_t map_id = p_new_tile->map_id;
    int16_t row_id = p_new_tile->row;
    int16_t col_id = p_new_tile->col;

    /* special for isolated window */
    if((uint16_t)map_id & TILEVIEW_MAP_ISOLATE_FLAG) {
        uint32_t win_id = map_id & TILEVIEW_MAP_ISOLATE_MASK;
        if((row_id == 0) && (col_id == 0)) {
            if(g_win_id_map[win_id].gesture_checker(obj, row_id, col_id)) {
                return (g_win_id_map[win_id].layout_creator)(obj);
            } else {
                return NULL;
            }
        } else if ((row_id == 0) && (col_id != 0)) {
            /* horizontal gesture, exit */
            if(g_win_id_map[win_id].gesture_checker(obj, row_id, col_id)) {
                return lv_layout_exit_current(obj);
            } else {
                return NULL;
            }
        } else {
            /* vertical geature, ignore */
            return NULL;
        }
    }

    if(TILEVIEW_MAP_ID_MAIN_SCREEN == map_id) {
        if(is_cardflow_layout_horizontal) {
            map_id = TILEVIEW_MAP_ID_MAIN_SCREEN_HOR;
        } else {
            map_id = TILEVIEW_MAP_ID_MAIN_SCREEN_VER;
        }
    }

    uint32_t win_pos = WMS_WIN_POS(map_id, row_id, col_id);

    for(int i = 0; i < (sizeof(g_tileview_map)/sizeof(lv_wms_layout_map_t)); i++) {
        if(g_tileview_map[i].win_pos == win_pos) {
            p_new_tile_cfg->use_same_effect_when_exit = g_tileview_map[i].is_symmetry_effect;
            p_new_tile_cfg->is_retain                 = g_tileview_map[i].is_retain;
            if(g_tileview_map[i].effect == LV_WMS_TILEVIEW_EFFECT_DEFAULT) {
                p_new_tile_cfg->effect                = lv_layout_router_get_slide_effect();
            } else {
                p_new_tile_cfg->effect                = g_tileview_map[i].effect;
            }

            return (g_tileview_map[i].layout_creator)(obj);
        }
    }

    return NULL;
}

static lv_obj_t * lv_layout_none(lv_obj_t * obj) {
    return NULL;
}

static bool lv_default_gesture_checker(lv_obj_t * obj, int16_t row, int16_t col) {

    if((row == 0) && (col == 0)) {
        return true;
    } else if((row == 0) && (col != 0)) {
        return true;        /* allow hor-gesture to exit in default */
    } else {
        return false;       /* not-allow ver-gesture to exit in default */
    }
}

static bool lv_applist_gesture_checker(lv_obj_t * obj, int16_t row, int16_t col) {
    if((row == 0) && (col == 0)) {
        return true;
    } else {
        return false;       /* not-allow ver-gesture to exit in default */
    }
}

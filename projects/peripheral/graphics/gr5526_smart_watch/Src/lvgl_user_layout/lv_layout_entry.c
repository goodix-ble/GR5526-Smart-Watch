#include "lvgl.h"

#define _TV_RowCol_ID(r, c)     (((uint16_t)(r) << 8) | (c))

#define lv_tileview_add_tile2(tv, row_id, col_id, dir)    lv_tileview_add_tile(tv, col_id, row_id, dir)

typedef struct {

    lv_obj_t * _tv_root;

    lv_obj_t * _tv_01;
    lv_obj_t * _tv_10;
    lv_obj_t * _tv_11;
    lv_obj_t * _tv_12;
    lv_obj_t * _tv_21;

} _layout_tv_t;

_layout_tv_t s_tv = {
    NULL,
    NULL,NULL,NULL,NULL,NULL,
};

extern lv_obj_t *lv_layout_tv_top_create(lv_obj_t * parent_tv_obj);
extern lv_obj_t *lv_layout_tv_bottom_create(lv_obj_t * parent_tv_obj);
extern lv_obj_t *lv_layout_tv_left_create(lv_obj_t * parent_tv_obj);
extern lv_obj_t *lv_layout_tv_right_create(lv_obj_t * parent_tv_obj);
extern lv_obj_t *lv_layout_tv_mid_create(lv_obj_t * parent_tv_obj);

extern lv_obj_t *lv_card_day_activity_layout_create(lv_obj_t * parent);
extern lv_obj_t *lv_card_watchface_vivid_layout_create(lv_obj_t * parent);

extern lv_obj_t *lv_layout_heartrate_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_heartrate_chart_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_heartrate_statics_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_stress_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_stress_chart_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_stress_statics_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_sleep_create(lv_obj_t *parent_tv_obj);
extern lv_obj_t *lv_layout_spo2_create(lv_obj_t *parent_tv_obj);


static void lv_layout_tv_load_win(uint8_t row_id, uint8_t col_id, lv_obj_t * tv_obj);
static void lv_layout_tv_scroll_event_handler(lv_event_t * e) ;

void lv_layout_tv_init(void);

void lv_layout_startup(void) {
    lv_layout_tv_init();
}


void lv_layout_tv_init(void) {

    s_tv._tv_root = lv_tileview_create(lv_scr_act());

    lv_obj_set_scrollbar_mode(s_tv._tv_root, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(s_tv._tv_root, LV_OBJ_FLAG_SCROLLABLE);

    /**** Laylout
            ------
            |    |
            |    |
       ----------------
       |    |    |    |
       |    |    |    |
       ----------------
            |    |
            |    |
            ------

     ***/

    s_tv._tv_01 = lv_tileview_add_tile2(s_tv._tv_root, 0, 1, LV_DIR_BOTTOM);
    s_tv._tv_10 = lv_tileview_add_tile2(s_tv._tv_root, 1, 0, LV_DIR_RIGHT);
    s_tv._tv_11 = lv_tileview_add_tile2(s_tv._tv_root, 1, 1, LV_DIR_ALL);
    s_tv._tv_12 = lv_tileview_add_tile2(s_tv._tv_root, 1, 2, LV_DIR_LEFT);
    s_tv._tv_21 = lv_tileview_add_tile2(s_tv._tv_root, 2, 1, LV_DIR_TOP);

    lv_layout_tv_load_win(0, 1, s_tv._tv_01);
    lv_layout_tv_load_win(1, 0, s_tv._tv_10);
    lv_layout_tv_load_win(1, 1, s_tv._tv_11);
    lv_layout_tv_load_win(1, 2, s_tv._tv_12);
    lv_layout_tv_load_win(2, 1, s_tv._tv_21);

    lv_obj_set_tile_id(s_tv._tv_root, 1, 1, LV_ANIM_OFF);
    lv_obj_add_event_cb(s_tv._tv_root, lv_layout_tv_scroll_event_handler, LV_EVENT_ALL, NULL);
}


static void lv_layout_tv_load_win(uint8_t row_id, uint8_t col_id, lv_obj_t * tv_obj) {

    switch(_TV_RowCol_ID(row_id, col_id)) {
        case _TV_RowCol_ID(0, 1):
        {
            lv_layout_tv_top_create(tv_obj);
        }
        break;

        case _TV_RowCol_ID(1, 0):
        {
            lv_layout_tv_left_create(tv_obj);
        }
        break;

        case _TV_RowCol_ID(1, 1):
        {
            lv_card_watchface_vivid_layout_create(tv_obj);
        }
        break;

        case _TV_RowCol_ID(1, 2):
        {
            lv_card_day_activity_layout_create(tv_obj);
        }
        break;

        case _TV_RowCol_ID(2, 1):
        {
            lv_layout_tv_bottom_create(tv_obj);
        }
        break;
    }
}

void lv_layout_tv_unload_win(uint8_t row_id, uint8_t col_id, lv_obj_t * tv_obj) {

}


volatile bool is_lvgl_tileview_scrolling = false;

static void lv_layout_tv_scroll_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    switch(code) {
        case LV_EVENT_SCROLL_BEGIN:
        {
            is_lvgl_tileview_scrolling = true;
        }
        break;

        case LV_EVENT_SCROLL:
        {
        }
        break;

        case LV_EVENT_SCROLL_END:
        {
            is_lvgl_tileview_scrolling = false;
        }
        break;

        default:break;
    }
}

bool lv_is_tileview_scrolling(void) {
    return is_lvgl_tileview_scrolling;
}



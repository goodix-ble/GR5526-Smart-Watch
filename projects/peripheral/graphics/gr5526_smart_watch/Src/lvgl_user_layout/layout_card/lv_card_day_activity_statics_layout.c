#include <stdio.h>
#include "app_key.h"
#include "lvgl.h"
#include "lv_font.h"
#include "lv_user_font.h"
#include "lv_img_dsc_list.h"


enum {
    MEASURE_READY = 0,
    MEASURE_DOING ,
    MEASURE_SUCCESS ,
    MEASURE_FAIL,
};

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t *   _day_walk_step_img                      = NULL;
static lv_obj_t *   _day_walk_step_val                      = NULL;
static lv_obj_t *   _day_walk_step_unit                     = NULL;
static lv_obj_t *   _day_walk_step_process_arc              = NULL;

static lv_obj_t *   _day_sports_time_img                    = NULL;
static lv_obj_t *   _day_sports_time_val                    = NULL;
static lv_obj_t *   _day_sports_time_unit                   = NULL;
static lv_obj_t *   _day_sports_time_process_arc            = NULL;

static lv_obj_t *   _day_standup_time_img                   = NULL;
static lv_obj_t *   _day_standup_time_val                   = NULL;
static lv_obj_t *   _day_standup_time_unit                  = NULL;
static lv_obj_t *   _day_standup_time_process_arc           = NULL;

static lv_obj_t *   _day_activity_start_btn                 = NULL;

/*
 * STATIC METHODs DECLARATION
 *****************************************************************************************
 */
void         lv_card_day_activity_layout_destroy(void);
lv_obj_t *   lv_card_day_activity_layout_create(lv_obj_t * parent);

static void         lv_event_cb(lv_event_t * e);
static void         _set_day_step(uint32_t val, uint32_t max);
static void         _set_day_sports_time(uint32_t val, uint32_t max);
static void         _set_day_standup_time(uint32_t val, uint32_t max);
static void         _lv_card_day_activity_layout_evt_handler(lv_event_t *e);

/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */



/*
 * STATIC METHODs IMPLEMENT
 *****************************************************************************************
 */
void lv_card_day_activity_layout_destroy(void)
{
    _day_walk_step_img                      = NULL;
    _day_walk_step_val                      = NULL;
    _day_walk_step_unit                     = NULL;
    _day_walk_step_process_arc              = NULL;

    _day_sports_time_img                    = NULL;
    _day_sports_time_val                    = NULL;
    _day_sports_time_unit                   = NULL;
    _day_sports_time_process_arc            = NULL;

    _day_standup_time_img                   = NULL;
    _day_standup_time_val                   = NULL;
    _day_standup_time_unit                  = NULL;
    _day_standup_time_process_arc           = NULL;

    _day_activity_start_btn                 = NULL;
}


static void _set_day_step(uint32_t val, uint32_t max) {
    if(val > max)
        val = max;

    lv_label_set_text_fmt(_day_walk_step_val, "%d", val);
    lv_arc_set_range(_day_walk_step_process_arc,  0, max);
    lv_arc_set_value(_day_walk_step_process_arc, val);
}

static void _set_day_sports_time(uint32_t val, uint32_t max) {
    if(val > max)
        val = max;

    lv_label_set_text_fmt(_day_sports_time_val, "%d", val);
    lv_arc_set_range(_day_sports_time_process_arc,  0, max);
    lv_arc_set_value(_day_sports_time_process_arc, val);
}

static void _set_day_standup_time(uint32_t val, uint32_t max) {
    if(val > max)
        val = max;

    lv_label_set_text_fmt(_day_standup_time_val, "%d", val);
    lv_arc_set_range(_day_standup_time_process_arc,  0, max);
    lv_arc_set_value(_day_standup_time_process_arc, val);
}


lv_obj_t *lv_card_day_activity_layout_create(lv_obj_t * parent)
{
    lv_obj_t * g_card_day_activity_win = NULL;

    lv_obj_enable_style_refresh(false);

    g_card_day_activity_win = lv_obj_create(parent);
    lv_obj_set_size(g_card_day_activity_win, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_set_scrollbar_mode(g_card_day_activity_win, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(g_card_day_activity_win, LV_DIR_NONE);
    lv_obj_set_style_bg_color(g_card_day_activity_win, lv_color_black(), 0);

    /* walk step per day */
    _day_walk_step_img = lv_img_create(g_card_day_activity_win);
    lv_img_set_src(_day_walk_step_img, &wd_img_day_walking);
    lv_obj_set_pos(_day_walk_step_img, 130, 90);

    _day_walk_step_val = lv_label_create(g_card_day_activity_win);
    lv_obj_set_style_text_font(_day_walk_step_val, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_day_walk_step_val, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text_static(_day_walk_step_val, "0");
    lv_obj_set_pos(_day_walk_step_val, 180, 90);

    _day_walk_step_unit = lv_label_create(g_card_day_activity_win);
    lv_obj_set_style_text_font(_day_walk_step_unit, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_day_walk_step_unit, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text(_day_walk_step_unit, "Step");
    lv_obj_set_pos(_day_walk_step_unit, 250, 90);

    /* sports time per day */
    _day_sports_time_img = lv_img_create(g_card_day_activity_win);
    lv_img_set_src(_day_sports_time_img, &wd_img_day_sports);
    lv_obj_set_pos(_day_sports_time_img, 130, 190);

    _day_sports_time_val = lv_label_create(g_card_day_activity_win);
    lv_obj_set_style_text_font(_day_sports_time_val, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_day_sports_time_val, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text_static(_day_sports_time_val, "0");
    lv_obj_set_pos(_day_sports_time_val, 180, 190);

    _day_sports_time_unit = lv_label_create(g_card_day_activity_win);
    lv_obj_set_style_text_font(_day_sports_time_unit, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_day_sports_time_unit, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text(_day_sports_time_unit, "Hour");
    lv_obj_set_pos(_day_sports_time_unit, 250, 190);

    /* stand time per day */
    _day_standup_time_img = lv_img_create(g_card_day_activity_win);
    lv_img_set_src(_day_standup_time_img, &wd_img_day_running);
    lv_obj_set_pos(_day_standup_time_img, 130, 290);

    _day_standup_time_val = lv_label_create(g_card_day_activity_win);
    lv_obj_set_style_text_font(_day_standup_time_val, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_day_standup_time_val, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text_static(_day_standup_time_val, "0");
    lv_obj_set_pos(_day_standup_time_val, 180, 290);

    _day_standup_time_unit = lv_label_create(g_card_day_activity_win);
    lv_obj_set_style_text_font(_day_standup_time_unit, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_day_standup_time_unit, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text(_day_standup_time_unit, "Min");
    lv_obj_set_pos(_day_standup_time_unit, 250, 290);


    /* ARC : Step */
    _day_walk_step_process_arc = lv_arc_create(g_card_day_activity_win);

    lv_obj_set_size(_day_walk_step_process_arc, DISP_HOR_RES - 24, DISP_VER_RES - 24);
    lv_obj_set_style_arc_color(_day_walk_step_process_arc, lv_color_make(0x6C, 0x03, 0x2B), LV_PART_MAIN);     /* BG ARC */
    lv_obj_set_style_arc_color(_day_walk_step_process_arc, lv_color_make(0xD9, 0x06, 0x57), LV_PART_INDICATOR);     /* FG ARC */
    lv_obj_remove_style(_day_walk_step_process_arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_clear_flag(_day_walk_step_process_arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    lv_obj_center(_day_walk_step_process_arc);

    lv_arc_set_rotation(_day_walk_step_process_arc, 260);
    lv_arc_set_angles(_day_walk_step_process_arc, 0, 50);
    lv_arc_set_range(_day_walk_step_process_arc,  0, 100);
    lv_arc_set_mode(_day_walk_step_process_arc, LV_ARC_MODE_NORMAL);
    lv_arc_set_bg_angles(_day_walk_step_process_arc, 0, 100);
    lv_arc_set_value(_day_walk_step_process_arc, 80);
    lv_obj_set_style_arc_width(_day_walk_step_process_arc, 24, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(_day_walk_step_process_arc, 24, LV_PART_MAIN);

    /* ARC : Sports */
    _day_sports_time_process_arc = lv_arc_create(g_card_day_activity_win);

    lv_obj_set_size(_day_sports_time_process_arc, DISP_HOR_RES - 24, DISP_VER_RES - 24);
    lv_obj_set_style_arc_color(_day_sports_time_process_arc, lv_color_make(0x00, 0x4D, 0x23),   LV_PART_MAIN);
    lv_obj_set_style_arc_color(_day_sports_time_process_arc, lv_color_make(0x00, 0x9B, 0x47), LV_PART_INDICATOR);
    lv_obj_remove_style(_day_sports_time_process_arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_clear_flag(_day_sports_time_process_arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    lv_obj_center(_day_sports_time_process_arc);

    lv_arc_set_rotation(_day_sports_time_process_arc, 260);
    lv_arc_set_angles(_day_sports_time_process_arc, 120, 180);
    lv_arc_set_range(_day_sports_time_process_arc,  0, 100);
    lv_arc_set_mode(_day_sports_time_process_arc, LV_ARC_MODE_NORMAL);
    lv_arc_set_bg_angles(_day_sports_time_process_arc, 120, 220);
    lv_arc_set_value(_day_sports_time_process_arc, 50);
    lv_obj_set_style_arc_width(_day_sports_time_process_arc, 24, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(_day_sports_time_process_arc, 24, LV_PART_MAIN);

    /* ARC : Standup time */
    _day_standup_time_process_arc = lv_arc_create(g_card_day_activity_win);

    lv_obj_set_size(_day_standup_time_process_arc, DISP_HOR_RES - 24, DISP_VER_RES - 24);
    lv_obj_set_style_arc_color(_day_standup_time_process_arc, lv_color_make(0x0F, 0x33, 0x64), LV_PART_MAIN);
    lv_obj_set_style_arc_color(_day_standup_time_process_arc, lv_color_make(0x1F, 0x67, 0xC9), LV_PART_INDICATOR);
    lv_obj_remove_style(_day_standup_time_process_arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_clear_flag(_day_standup_time_process_arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    lv_obj_center(_day_standup_time_process_arc);

    lv_arc_set_rotation(_day_standup_time_process_arc, 260);
    lv_arc_set_angles(_day_standup_time_process_arc, 240, 280);
    lv_arc_set_range(_day_standup_time_process_arc,  0, 100);
    lv_arc_set_mode(_day_standup_time_process_arc, LV_ARC_MODE_NORMAL);
    lv_arc_set_bg_angles(_day_standup_time_process_arc, 240, 340);
    lv_arc_set_value(_day_standup_time_process_arc, 70);
    lv_obj_set_style_arc_width(_day_standup_time_process_arc, 24, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(_day_standup_time_process_arc, 24, LV_PART_MAIN);

    /* Button to details */
    _day_activity_start_btn = lv_btn_create(g_card_day_activity_win) ;
    lv_obj_set_size(_day_activity_start_btn, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_set_pos(_day_activity_start_btn, 0, 0);
    lv_obj_set_style_opa(_day_activity_start_btn, 0, 0);        /* set opa to 0, as hidden */
    lv_obj_add_event_cb(_day_activity_start_btn, lv_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(g_card_day_activity_win, _lv_card_day_activity_layout_evt_handler, LV_EVENT_ALL, NULL);

    /* init value */
    if(1) {
        _set_day_step(500, 800);
        _set_day_sports_time(60, 120);
        _set_day_standup_time(3, 12);
    }

    lv_obj_enable_style_refresh(true);

    return g_card_day_activity_win;
}


static void lv_event_cb(lv_event_t * e)
{

}



static void _lv_card_day_activity_layout_evt_handler(lv_event_t *e)
{
    switch(e->code) {
        case LV_EVENT_DELETE:
        {
            lv_card_day_activity_layout_destroy();
        }
        break;

        case LV_EVENT_CANCEL:
        {

        }
        break;

        case LV_EVENT_READY:
        {

        }
        break;

        default:
        {}
        break;
    }
}


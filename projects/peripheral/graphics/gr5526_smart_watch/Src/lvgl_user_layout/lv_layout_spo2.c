#include <stdio.h>
#include "lvgl.h"
#include "app_key.h"
#include "app_log.h"
#include "lv_font.h"
#include "lv_user_font.h"
#include "lv_img_dsc_list.h"

#define SPO2_ANIMATION_TIMER_PERIOD (200)

enum {
    MEASURE_READY = 0,
    MEASURE_DOING ,
    MEASURE_SUCCESS ,
    MEASURE_FAIL,
};

#pragma diag_suppress 177

// extern uint8_t g_lead_on_flag;

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t *   _spo2_bg_img                    = NULL;
static lv_obj_t *   _spo2_logo_img                  = NULL;
static lv_obj_t *   _spo2_logo_measure_img          = NULL;
static lv_obj_t *   _spo2_gesture_img               = NULL;
static lv_obj_t *   ready_meas_label  = NULL;
static lv_obj_t *   fail_meas_label  = NULL;
static lv_obj_t *   start_meas_btn         = NULL;
static lv_obj_t *   start_meas_btn_label   = NULL;
static lv_obj_t *   meas_process_arc       = NULL;
static lv_timer_t * _spo2_animation_timer           = NULL;
static uint8_t      _cur_measure_state              = MEASURE_READY;
static float        _s_progress                     = 0;
static int          _s_timer_run_time               = 0;

/*
 * STATIC METHODs DECLARATION
 *****************************************************************************************
 */
static void         lv_event_cb(lv_event_t * e);
static void         _update_measure_ui_state(uint8_t state);
static void         _spo2_timer_callback(lv_timer_t * tmr);
//static bool         _key_event_handler(uint32_t key, uint32_t event);
//static bool         _gesture_event_handler(lv_dir_t dir);

 static const lv_style_const_prop_t WINDOW_STYLE_PROPS[] = {
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_CONST_BG_OPA(LV_OPA_COVER),
    LV_STYLE_CONST_PAD_TOP(0),
    LV_STYLE_CONST_PAD_BOTTOM(0),
    LV_STYLE_CONST_PAD_LEFT(0),
    LV_STYLE_CONST_PAD_RIGHT(0),
    LV_STYLE_CONST_BORDER_WIDTH(0),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t READY_MEAS_LABEL_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_36),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t FAIL_MEAS_LABEL_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_20),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0x00, 0x9B, 0x47)),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t START_MEAS_BTN_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_20),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t MEAS_PROCESS_ARC_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_20),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t START_MEAS_LABEL_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_20),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(SPO2_WINDOW_STYLE, WINDOW_STYLE_PROPS);
LV_STYLE_CONST_INIT(READY_MEAS_LABEL_STYLE, READY_MEAS_LABEL_STYLE_PROPS);
LV_STYLE_CONST_INIT(FAIL_MEAS_LABEL_STYLE, FAIL_MEAS_LABEL_STYLE_PROPS);
LV_STYLE_CONST_INIT(START_MEAS_BTN_STYLE, START_MEAS_BTN_STYLE_PROPS);
LV_STYLE_CONST_INIT(MEAS_PROCESS_ARC_STYLE, MEAS_PROCESS_ARC_STYLE_PROPS);
LV_STYLE_CONST_INIT(START_MEAS_LABEL_STYLE, START_MEAS_LABEL_STYLE_PROPS);


/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */

/*
 * STATIC METHODs IMPLEMENT
 *****************************************************************************************
 */
static void _update_measure_ui_state(uint8_t state)
{
    uint16_t spo2_value = 0;

    switch(state) {
        case MEASURE_READY:
        {
            lv_obj_add_flag(_spo2_gesture_img, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(fail_meas_label,  LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(_spo2_logo_img, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ready_meas_label, LV_OBJ_FLAG_HIDDEN);

            lv_arc_set_value(meas_process_arc, 0);
            // lv_label_set_text(ready_meas_label, ISTR("佩戴手表,保持静止"));
            lv_label_set_text(ready_meas_label, "wear a watch, stay still");
            // lv_label_set_text(start_meas_btn_label, ISTR("测量"));
            lv_label_set_text(start_meas_btn_label, "Measure");
#if ENABLE_AUTO_DISPLAY_OFF > 0u
            extern void lv_hif_allow_sleep(bool allow);
            lv_hif_allow_sleep(true);
#endif
        }
        break;

        case MEASURE_DOING:
        {
            lv_obj_add_flag(_spo2_gesture_img, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(fail_meas_label, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(_spo2_logo_img, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ready_meas_label, LV_OBJ_FLAG_HIDDEN);

            // lv_label_set_text(ready_meas_label, ISTR("测量中,请勿移动"));
            // lv_label_set_text(start_meas_btn_label, ISTR("测量中"));
            lv_label_set_text(ready_meas_label, "During measurement, do not move");
            lv_label_set_text(start_meas_btn_label, "Measuring");
#if ENABLE_AUTO_DISPLAY_OFF > 0u
            extern void lv_hif_allow_sleep(bool allow);
            lv_hif_allow_sleep(false);
#endif
        }
        break;

        case MEASURE_SUCCESS:
        {
            lv_obj_add_flag(_spo2_gesture_img, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(fail_meas_label,  LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(_spo2_logo_img, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ready_meas_label, LV_OBJ_FLAG_HIDDEN);

            lv_arc_set_value(meas_process_arc, 99);
            // lv_label_set_text(ready_meas_label, ISTR("测量完成!"));
            lv_label_set_text(ready_meas_label, "Done!");

            // spo2_value = (uint16_t)spo2_value_get();
            lv_label_set_text_fmt(start_meas_btn_label, "%d", spo2_value);

#if ENABLE_AUTO_DISPLAY_OFF > 0u
            extern void lv_hif_allow_sleep(bool allow);
            lv_hif_allow_sleep(true);
#endif
        }
        break;

        case MEASURE_FAIL:
        {
            lv_obj_add_flag(_spo2_logo_img, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ready_meas_label, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(_spo2_gesture_img, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(fail_meas_label,  LV_OBJ_FLAG_HIDDEN);

            lv_arc_set_value(meas_process_arc, 0);
            // lv_label_set_text(start_meas_btn_label, ISTR("重新测量"));
            lv_label_set_text(start_meas_btn_label, "Remeasuring");
#if ENABLE_AUTO_DISPLAY_OFF > 0u
            extern void lv_hif_allow_sleep(bool allow);
            lv_hif_allow_sleep(true);
#endif
        }
        break;
    }

    _cur_measure_state = state;
    _cur_measure_state = _cur_measure_state;
}


static void _spo2_measure_animation_callback(void)
{
    lv_arc_set_value(meas_process_arc, _s_progress);

    // uint16_t spo2_value = (uint16_t)spo2_value_get();
    uint16_t spo2_value = 99;

    if (_s_progress >= 100)
    {
        lv_timer_pause(_spo2_animation_timer);

        if (spo2_value == 0)
        {
            _update_measure_ui_state(MEASURE_FAIL);
        }
        else
        {
            _update_measure_ui_state(MEASURE_SUCCESS);
        }
        _s_progress = 0;
    }
    _s_progress += 0.33f;
}

static void btn_event_cb(lv_event_t * e)
{
    switch(e->code){
        case LV_EVENT_PRESSED:
        {
            lv_obj_set_style_opa(start_meas_btn, 30, 0);
            break;
        }

        case LV_EVENT_PRESSING:
        {
            lv_obj_set_style_opa(start_meas_btn, 20, 0);
            break;
        }

        case LV_EVENT_RELEASED:
        {
            lv_obj_set_style_opa(start_meas_btn, 0, 0);

            if(_cur_measure_state == MEASURE_READY) {
                _update_measure_ui_state(MEASURE_DOING);
                lv_timer_resume(_spo2_animation_timer);
            } else if (_cur_measure_state == MEASURE_SUCCESS) {
                _update_measure_ui_state(MEASURE_READY);
                _s_progress = 0;
            } else if (_cur_measure_state == MEASURE_FAIL) {
                _update_measure_ui_state(MEASURE_DOING);
                lv_timer_resume(_spo2_animation_timer);
            }
            break;
        }

        default:
            break;
    }
}

static void _spo2_timer_callback(lv_timer_t * tmr)
{
    _spo2_measure_animation_callback();
    _s_timer_run_time++;

    // should wait at least 5s before ADT test.
    // if (_s_timer_run_time * SPO2_ANIMATION_TIMER_PERIOD >= 5000 && g_lead_on_flag == 0)
    // {
    //     _update_measure_ui_state(MEASURE_FAIL);
    //     lv_timer_pause(_spo2_animation_timer);
    //     _s_timer_run_time = 0;
    // }
}

static void spo2_event_cb(lv_event_t * e)
{
    if (e->code == LV_EVENT_DELETE)
    {
        if (_spo2_animation_timer)
        {
            lv_timer_del(_spo2_animation_timer);
            _spo2_animation_timer = NULL;
        }
    }
    else if (e->code == LV_EVENT_READY)
    {
        if (_spo2_animation_timer)
        {
            // _set_heart_beat_animation(true);
            lv_timer_resume(_spo2_animation_timer);

        }
        else
        {
            _spo2_animation_timer = lv_timer_create(_spo2_timer_callback, SPO2_ANIMATION_TIMER_PERIOD, NULL);
        }
    }
    else if (e->code == LV_EVENT_CANCEL)
    {
        if (_spo2_animation_timer)
        {
            lv_timer_pause(_spo2_animation_timer);
        }
    }
}

lv_obj_t *lv_layout_spo2_create(lv_obj_t *parent_tv_obj)
{
    lv_obj_t *p_window = lv_obj_create(parent_tv_obj);
    lv_obj_set_size(p_window, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_add_style(p_window, (lv_style_t *)&SPO2_WINDOW_STYLE, 0);

    _spo2_bg_img = lv_img_create(p_window);
    lv_img_set_src(_spo2_bg_img, &wd_img_sop2_background);
    lv_obj_set_pos(_spo2_bg_img, 0, 0);

    _spo2_logo_img = lv_img_create(p_window);
    lv_img_set_src(_spo2_logo_img, &wd_img_sop2_icon);
    lv_obj_align(_spo2_logo_img, LV_ALIGN_TOP_MID, 0, 60);

    ready_meas_label = lv_label_create(p_window);
    // lv_obj_add_style(ready_meas_label, (lv_style_t *)&READY_MEAS_LABEL_STYLE, 0);
    lv_obj_set_style_text_font(ready_meas_label, &lv_font_msyh_26_gr552x, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ready_meas_label, lv_color_white(), LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ready_meas_label, LV_TEXT_ALIGN_CENTER, LV_STATE_DEFAULT);
    // lv_label_set_text(ready_meas_label, ISTR("佩戴手表,保持静止"));
    lv_label_set_text(ready_meas_label, ("wear a watch, stay still"));
    lv_obj_align(ready_meas_label, LV_ALIGN_TOP_MID, 0, 140);
    lv_obj_set_size(ready_meas_label, 290, 160);

    _spo2_gesture_img = lv_img_create(p_window);
    lv_img_set_src(_spo2_gesture_img, &wd_img_sop2_measure_gesture);
    lv_obj_set_pos(_spo2_gesture_img, 80, 80);

    fail_meas_label = lv_label_create(p_window);
    // lv_obj_add_style(fail_meas_label, (lv_style_t *)&FAIL_MEAS_LABEL_STYLE, 0);
    lv_obj_set_style_text_font(fail_meas_label, &lv_font_msyh_26_gr552x, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(fail_meas_label, lv_color_white(), LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(fail_meas_label, LV_TEXT_ALIGN_CENTER, LV_STATE_DEFAULT);
    // lv_label_set_text(fail_meas_label, ISTR("佩戴手表,保持静止"));
    lv_label_set_text(fail_meas_label, ("wear a watch, stay still"));
    lv_obj_set_size(fail_meas_label, 290, 160);
    lv_obj_align(fail_meas_label, LV_ALIGN_TOP_MID, 0, 190);

    _spo2_logo_measure_img = lv_img_create(p_window);
    lv_img_set_src(_spo2_logo_measure_img, NULL);
    lv_obj_set_pos(_spo2_logo_measure_img, 48, 270);

    start_meas_btn = lv_btn_create(p_window) ;
    // lv_obj_add_style(start_meas_btn, (lv_style_t *)&START_MEAS_BTN_STYLE, 0);
    lv_obj_set_size(start_meas_btn, DISP_HOR_RES, 90);
    lv_obj_set_pos(start_meas_btn, 0, 270);
    lv_obj_set_style_opa(start_meas_btn, 0, 0);        /* set opa to 0, as hidden */
    lv_obj_add_event_cb(start_meas_btn, btn_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(start_meas_btn, btn_event_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(start_meas_btn, btn_event_cb, LV_EVENT_RELEASED, NULL);

    meas_process_arc = lv_arc_create(p_window);
    // lv_obj_add_style(start_meas_btn, (lv_style_t *)&MEAS_PROCESS_ARC_STYLE, 0);
    lv_obj_set_size(meas_process_arc, DISP_HOR_RES, DISP_HOR_RES);
    lv_obj_set_style_arc_color(meas_process_arc, lv_palette_main(LV_PALETTE_RED), LV_PART_INDICATOR);
    lv_obj_remove_style(meas_process_arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_clear_flag(meas_process_arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    lv_obj_center(meas_process_arc);
    lv_arc_set_rotation(meas_process_arc, 270);
    lv_arc_set_angles(meas_process_arc, 0, 360);
    lv_arc_set_range(meas_process_arc,  0, 100);
    lv_arc_set_mode(meas_process_arc, LV_ARC_MODE_NORMAL);
    lv_arc_set_bg_angles(meas_process_arc, 0, 360);

    start_meas_btn_label = lv_label_create(p_window);
    // lv_obj_add_style(start_meas_btn_label, (lv_style_t *)&START_MEAS_LABEL_STYLE, 0);
    lv_obj_set_style_text_font(start_meas_btn_label, &lv_font_msyh_26_gr552x, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(start_meas_btn_label, lv_color_white(), LV_STATE_DEFAULT);
    lv_obj_center(start_meas_btn_label);
    lv_obj_align(start_meas_btn_label, LV_ALIGN_BOTTOM_MID, 0, -48);
    // lv_label_set_text(start_meas_btn_label, ISTR("测量"));
    lv_label_set_text(start_meas_btn_label, ("Measure"));

    lv_obj_add_flag(_spo2_logo_img, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ready_meas_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(_spo2_gesture_img, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(fail_meas_label, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_event_cb(p_window, spo2_event_cb, LV_EVENT_ALL, NULL);
    _update_measure_ui_state(MEASURE_READY);

    return p_window;
}


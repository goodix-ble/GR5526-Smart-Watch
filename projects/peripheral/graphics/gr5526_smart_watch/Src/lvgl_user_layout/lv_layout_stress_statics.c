#include <stdio.h>
#include "lvgl.h"
#include "app_key.h"
#include "app_log.h"
#include "lv_font.h"
#include "lv_user_font.h"
#include "lv_img_dsc_list.h"

#pragma diag_suppress 177

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t *   _stress_state_img            = NULL;
static lv_obj_t *   _stress_state_label          = NULL;
static lv_obj_t *   _stress_state_analysis       = NULL;
static lv_timer_t * _stress_score_update_timer   = NULL;

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

static const lv_style_const_prop_t REALTIME_LABEL_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_36),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t UNIT_LABEL_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_20),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0x00, 0x9B, 0x47)),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(STRESS_STATICS_WINDOW_STYLE, WINDOW_STYLE_PROPS);
LV_STYLE_CONST_INIT(STRESS_STATE_LABEL_STYLE, REALTIME_LABEL_STYLE_PROPS);
LV_STYLE_CONST_INIT(STRESS_STATE_ANALYSIS_LABEL_STYLE, UNIT_LABEL_STYLE_PROPS);
// LV_STYLE_CONST_INIT(MIN_MAX_LABEL_STYLE, MIN_MAX_LABEL_STYLE_PROPS);

/*
 * STATIC METHODs DECLARATION
 *****************************************************************************************
 */
//static void         _set_stress_state(uint16_t val);

/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */


/*
 * STATIC METHODs IMPLEMENT
 *****************************************************************************************
 */
/*
 * High - 紧张
 * Middle - 中度
 * Normal - 正常
 * Low - 放松
 */
static void _set_stress_state(uint16_t val) {
    if(!_stress_state_label || !_stress_state_img)
        return;

    if((val >= 80) && (val <= 100))
    {
        // lv_label_set_text_static(_stress_state_label, ISTR("紧张"));
        lv_label_set_text_static(_stress_state_label, ("Nervous"));
        lv_img_set_src(_stress_state_img, &wd_img_stress_high_50);
        // lv_label_set_text_static(_stress_state_analysis, ISTR("建议注意休息，避免长期处于压力状态，养成规律作息习惯，保持心情愉悦，避免压力过大。"));
        lv_label_set_text_static(_stress_state_analysis, ("建议注意休息，避免长期处于压力状态，养成规律作息习惯，保持心情愉悦，避免压力过大。"));
    }
    else if ((val >= 60) && (val < 80))
    {
        // lv_label_set_text_static(_stress_state_label, ISTR("中度"));
        lv_label_set_text_static(_stress_state_label, ("Medium"));
        lv_img_set_src(_stress_state_img, &wd_img_stress_middle_50);
        // lv_label_set_text_static(_stress_state_analysis, ISTR("合理安排时间，养成规律作息习惯，保持心情愉悦，避免压力过大。"));
        lv_label_set_text_static(_stress_state_analysis, ("合理安排时间，养成规律作息习惯，保持心情愉悦，避免压力过大。"));
    }
    else if ((val >= 30) && (val < 60))
    {
        // lv_label_set_text_static(_stress_state_label, ISTR("正常"));
        lv_label_set_text_static(_stress_state_label, ("Normal"));
        lv_img_set_src(_stress_state_img, &wd_img_stress_normal_50);
        // lv_label_set_text_static(_stress_state_analysis, ISTR("请放松身心，养成规律作息习惯，保持心情愉悦，避免压力过大。"));
        lv_label_set_text_static(_stress_state_analysis, ("请放松身心，养成规律作息习惯，保持心情愉悦，避免压力过大。"));
    }
    else if (val < 30)
    {
        // lv_label_set_text_static(_stress_state_label, ISTR("放松"));
        lv_label_set_text_static(_stress_state_label, ("Easy"));
        lv_img_set_src(_stress_state_img, &wd_img_stress_easy_50);
        // lv_label_set_text_static(_stress_state_analysis, ISTR("放松身心，保持规律的作息习惯，保持心情愉悦，避免压力过大。"));
        lv_label_set_text_static(_stress_state_analysis, ("放松身心，保持规律的作息习惯，保持心情愉悦，避免压力过大。"));
    }
    else
    {
        lv_label_set_text_static(_stress_state_label, "--");
        lv_img_set_src(_stress_state_img, &wd_img_pressure_icon_50);
    }

    return;
}

static void _stress_score_update_timer_callback(lv_timer_t * tmr)
{
    // uint8_t stress_score = (uint16_t)stress_score_get();
    // uint8_t stress_score = 60;
    // _set_stress_state(stress_score);
}

static void stress_statics_event_cb(lv_event_t * e)
{
    if (e->code == LV_EVENT_DELETE)
    {
        if (_stress_score_update_timer)
        {
            lv_timer_del(_stress_score_update_timer);
            _stress_score_update_timer = NULL;
        }
    }
    else if (e->code == LV_EVENT_READY)
    {
        if (_stress_score_update_timer)
        {
            lv_timer_resume(_stress_score_update_timer);
        }
        else
        {
            _stress_score_update_timer = lv_timer_create(_stress_score_update_timer_callback, 350, NULL);
        }
    }
    else if (e->code == LV_EVENT_CANCEL)
    {
        if (_stress_score_update_timer)
        {
            lv_timer_pause(_stress_score_update_timer);
        }
    }
}

lv_obj_t *lv_layout_stress_statics_create(lv_obj_t *parent_tv_obj)
{
    lv_obj_t *p_window = lv_obj_create(parent_tv_obj);
    lv_obj_set_size(p_window, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_add_style(p_window, (lv_style_t *)&STRESS_STATICS_WINDOW_STYLE, 0);

    // uint8_t stress_score = (uint16_t)stress_score_get();
    uint8_t stress_score = 60;

    /* layout quiet heart rate */
    _stress_state_img = lv_img_create(p_window);
    lv_img_set_src(_stress_state_img, &wd_img_pressure_icon_50);
    lv_obj_align(_stress_state_img, LV_ALIGN_TOP_MID, -40, 110);
    // lv_obj_set_pos(_stress_state_img, 155, 50);

    _stress_state_label = lv_label_create(p_window);
    // lv_obj_add_style(_stress_state_label, (lv_style_t *)&STRESS_STATE_LABEL_STYLE, 0);
    lv_obj_set_style_text_font(_stress_state_label, &lv_font_montserrat_20, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_stress_state_label, lv_color_white(), 0);
//    lv_label_set_text_static(_stress_state_label, ISTR("中度"));
    lv_label_set_text_static(_stress_state_label, "Medium");
    lv_obj_align(_stress_state_label, LV_ALIGN_TOP_MID, 40, 110);
    // lv_obj_align(_stress_state_label, LV_ALIGN_TOP_MID, 0, 115);

    _stress_state_analysis = lv_label_create(p_window);
    // lv_obj_add_style(_stress_state_analysis, (lv_style_t *)&STRESS_STATE_ANALYSIS_LABEL_STYLE, 0);
    lv_obj_set_style_text_font(_stress_state_analysis, &lv_font_montserrat_20, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_stress_state_analysis, lv_color_make(0x80, 0x80, 0x80), 0);
    // lv_label_set_text_static(_stress_state_analysis, "压力分为紧张、中度、正常、放松,请放松身心,保持低压力状态!");
    lv_label_set_text_static(_stress_state_analysis, "Stress is divided into tense, moderate, normal, and relaxed, please relax and maintain a low stress state!");
    lv_obj_set_size(_stress_state_analysis, 320, 180);
    lv_obj_align(_stress_state_analysis, LV_ALIGN_CENTER, 0, 20);
    // lv_obj_set_pos(_stress_state_analysis, 20, 160);

    // _set_stress_state(stress_score);
    lv_obj_add_event_cb(p_window, stress_statics_event_cb, LV_EVENT_ALL, NULL);

    return p_window;
}

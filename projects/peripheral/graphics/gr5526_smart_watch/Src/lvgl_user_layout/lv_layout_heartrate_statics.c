#include <stdio.h>
#include "lvgl.h"
#include "lv_font.h"
#include "lv_user_font.h"
#include "app_key.h"
#include "app_log.h"
#include "lv_img_dsc_list.h"


/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t *   _quiet_hr_img       = NULL;
static lv_obj_t *   _quiet_hr_val       = NULL;
static lv_obj_t *   _quiet_hr_unit      = NULL;
static lv_obj_t *   _quiet_hr_tips      = NULL;

//static lv_obj_t *   _walk_hr_img       = NULL;
//static lv_obj_t *   _walk_hr_val       = NULL;
//static lv_obj_t *   _walk_hr_unit      = NULL;
//static lv_obj_t *   _walk_hr_tips      = NULL;

static lv_timer_t * _hr_update_timer   = NULL;


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

static const lv_style_const_prop_t VAL_LABEL_STYLE_PROPS[] = {
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

LV_STYLE_CONST_INIT(HEARTRATE_STATICS_WINDOW_STYLE, WINDOW_STYLE_PROPS);
LV_STYLE_CONST_INIT(QUIET_HR_VAL_LABEL_STYLE, VAL_LABEL_STYLE_PROPS);
LV_STYLE_CONST_INIT(QUIET_HR_UNIT_LABEL_STYLE, UNIT_LABEL_STYLE_PROPS);


/*
 * STATIC METHODs DECLARATION
 *****************************************************************************************
 */
static void         _set_quiet_hr_val(uint16_t val);
// static void         _set_walk_hr_val(uint16_t val);
//static bool         key_event_handler(uint32_t key, uint32_t event);

/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */



/*
 * STATIC METHODs IMPLEMENT
 *****************************************************************************************
 */
static void _set_quiet_hr_val(uint16_t val) {
    if(!_quiet_hr_val)
        return ;

    if(val == 0) {
        lv_label_set_text_static(_quiet_hr_val, "--");
    } else {
        lv_label_set_text_fmt(_quiet_hr_val, "%d", val);
    }
    return;
}

//static void _set_walk_hr_val(uint16_t val) {
//    if(!_walk_hr_val)
//        return ;

//    if(val == 0) {
//        lv_label_set_text_static(_walk_hr_val, "--");
//    } else {
//        lv_label_set_text_fmt(_walk_hr_val, "%d", val);
//    }
//    return;
//}

static void _hr_update_timer_callback(lv_timer_t * tmr)
{
}

static void heartrate_statics_event_cb(lv_event_t *e)
{
    if (e->code == LV_EVENT_DELETE)
    {
        if (_hr_update_timer)
        {
            lv_timer_del(_hr_update_timer);
            _hr_update_timer = NULL;
        }
    }
    else if (e->code == LV_EVENT_READY)
    {
        if (_hr_update_timer)
        {
            lv_timer_resume(_hr_update_timer);
        }
        else
        {
            _hr_update_timer = lv_timer_create(_hr_update_timer_callback, 350, NULL);
        }
    }
    else if (e->code == LV_EVENT_CANCEL)
    {
        if (_hr_update_timer)
        {
            lv_timer_pause(_hr_update_timer);
        }
    }
}

/*
 * GLOBAL METHODS IMPLEMENT
 *****************************************************************************************
 */
lv_obj_t *lv_layout_heartrate_statics_create(lv_obj_t *parent_tv_obj)
{
    lv_obj_t *p_window = lv_obj_create(parent_tv_obj);
    lv_obj_set_size(p_window, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_add_style(p_window, (lv_style_t *)&HEARTRATE_STATICS_WINDOW_STYLE, 0);

    /* layout quiet heart rate */
    _quiet_hr_img = lv_img_create(p_window);
    lv_img_set_src(_quiet_hr_img, &wd_img_heartrate_sleep_56);
    lv_obj_align(_quiet_hr_img, LV_ALIGN_CENTER, -80, 0);
    // lv_obj_set_pos(_quiet_hr_img, 70, 150);

    _quiet_hr_val = lv_label_create(p_window);
    // lv_obj_add_style(_quiet_hr_val, (lv_style_t *)&QUIET_HR_VAL_LABEL_STYLE, 0);
    lv_obj_set_style_text_font(_quiet_hr_val, &lv_font_msyh_26_gr552x, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_quiet_hr_val, lv_color_white(), 0);
    lv_label_set_text_fmt(_quiet_hr_val, "%d", 80);
    lv_obj_align_to(_quiet_hr_val, _quiet_hr_img, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
    // lv_obj_set_pos(_quiet_hr_val, 200, 140);

    _quiet_hr_unit = lv_label_create(p_window);
    // lv_obj_add_style(_quiet_hr_unit, (lv_style_t *)&QUIET_HR_UNIT_LABEL_STYLE, 0);
    lv_obj_set_style_text_font(_quiet_hr_unit, &lv_font_msyh_26_gr552x, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_quiet_hr_unit, lv_color_make(0x80, 0x80, 0x80), 0);
    // lv_label_set_text_static(_quiet_hr_unit, ISTR("次/分"));
    lv_label_set_text_static(_quiet_hr_unit, ("次/分"));
    lv_obj_align_to(_quiet_hr_unit, _quiet_hr_img, LV_ALIGN_OUT_RIGHT_MID, 50, 0);
    // lv_obj_set_pos(_quiet_hr_unit, 250, 150);

    _quiet_hr_tips = lv_label_create(p_window);
    // lv_obj_add_style(_quiet_hr_tips, (lv_style_t *)&QUIET_HR_UNIT_LABEL_STYLE, 0);
    lv_obj_set_style_text_font(_quiet_hr_tips, &lv_font_msyh_26_gr552x, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_quiet_hr_tips, lv_color_make(0x80, 0x80, 0x80), 0);
    // lv_label_set_text_static(_quiet_hr_tips, ISTR("静息心率"));
    lv_label_set_text_static(_quiet_hr_tips, ("Rest Heart Rate"));
    lv_obj_align_to(_quiet_hr_tips, _quiet_hr_img, LV_ALIGN_OUT_RIGHT_MID, 20, 36);
    // lv_obj_set_pos(_quiet_hr_tips, 200, 180);

//    /* layout walk heart rate */
//    _walk_hr_img = lv_img_create(p_window);
//    lv_img_set_src(_walk_hr_img, &wd_img_heartrate_walk_56);
//    lv_obj_set_pos(_walk_hr_img, 70, 200);

//    _walk_hr_val = lv_label_create(p_window);
//    lv_obj_set_style_text_font(_walk_hr_val, &lv_font_msyh_26_gr552x, LV_STATE_DEFAULT);
//    lv_obj_set_style_text_color(_walk_hr_val, lv_color_white(), 0);
//    lv_label_set_text_fmt(_walk_hr_val, "%d", 80);
//    lv_obj_set_pos(_walk_hr_val, 150, 190);

//    _walk_hr_unit = lv_label_create(p_window);
//    lv_obj_set_style_text_font(_walk_hr_unit, &lv_font_msyh_20_gr552x, LV_STATE_DEFAULT);
//    lv_obj_set_style_text_color(_walk_hr_unit, lv_color_make(0x80, 0x80, 0x80), 0);
//    lv_label_set_text_static(_walk_hr_unit, ISTR("次/分"));
//    lv_obj_set_pos(_walk_hr_unit, 200, 200);

//    _walk_hr_tips = lv_label_create(p_window);
//    lv_obj_set_style_text_font(_walk_hr_tips, &lv_font_msyh_20_gr552x, LV_STATE_DEFAULT);
//    lv_obj_set_style_text_color(_walk_hr_tips, lv_color_make(0x80, 0x80, 0x80), 0);
//    lv_label_set_text_static(_walk_hr_tips, "步行心率");
//    lv_obj_set_pos(_walk_hr_tips, 150, 230);

    _set_quiet_hr_val(0);
    // _set_walk_hr_val(120);

    lv_obj_add_event_cb(p_window, heartrate_statics_event_cb, LV_EVENT_ALL, NULL);

    return p_window;
}

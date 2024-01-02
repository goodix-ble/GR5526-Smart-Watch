#include <stdio.h>
#include "lvgl.h"
#include "lv_font.h"
#include "app_log.h"
#include "lv_user_font.h"
#include "lv_img_dsc_list.h"

#pragma diag_suppress 177

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t *   _label_sleep_title            = NULL;
static lv_obj_t *   _label_sleep_hour_val         = NULL;
static lv_obj_t *   _label_sleep_min_val          = NULL;
static lv_obj_t *   _label_sleep_hour_unit        = NULL;
static lv_obj_t *   _label_sleep_min_unit         = NULL;
static lv_obj_t *   _img_sleep_flag               = NULL;
static lv_obj_t *   _img_sleep_bg                 = NULL;

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

static const lv_style_const_prop_t TITLE_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_36),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)),
    // LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t LABEL_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_30),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)),
    // LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};


LV_STYLE_CONST_INIT(SLEEP_WINDOW_STYLE, WINDOW_STYLE_PROPS);
LV_STYLE_CONST_INIT(SLEEP_TITLE_STYLE, TITLE_STYLE_PROPS);
LV_STYLE_CONST_INIT(SLEEP_LABEL_STYLE, LABEL_STYLE_PROPS);

/*
 * STATIC METHODs DECLARATION
 *****************************************************************************************
 */
static void         _set_sleep_hour_val(uint16_t val);
static void         _set_sleep_min_val(uint16_t val);
static void         _clear_and_reset_state(void);

/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */


/*
 * STATIC METHODs IMPLEMENT
 *****************************************************************************************
 */
static void _set_sleep_hour_val(uint16_t val) {
    if(!_label_sleep_hour_val)
        return;

    if(val == 0) {
        lv_label_set_text_static(_label_sleep_hour_val, "--");
    } else {
        lv_label_set_text_fmt(_label_sleep_hour_val, "%d", val);
    }
    return;
}

static void _set_sleep_min_val(uint16_t val) {
    if(!_label_sleep_min_val)
        return;

    if(val == 0) {
        lv_label_set_text_static(_label_sleep_min_val, "--");
    } else {
        lv_label_set_text_fmt(_label_sleep_min_val, "%d", val);
    }
    return;
}


static void _clear_and_reset_state(void) {
    _set_sleep_hour_val(0);
    _set_sleep_min_val(0);
}

static void btn_event_cb(lv_event_t * e) {
    APP_LOG_DEBUG("Sleep cb called !");
}

static void sleep_event_cb(lv_event_t * e)
{
    if (e->code == LV_EVENT_DELETE)
    {
    }
    else if (e->code == LV_EVENT_READY)
    {
    }
    else if (e->code == LV_EVENT_CANCEL)
    {
    }
}

lv_obj_t *lv_card_layout_sleep_create(lv_obj_t *parent_tv_obj)
{
    lv_obj_t *p_window = lv_obj_create(parent_tv_obj);
    lv_obj_set_size(p_window, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_add_style(p_window, (lv_style_t *)&SLEEP_WINDOW_STYLE, 0);

    _img_sleep_bg = lv_img_create(p_window);
    lv_img_set_src(_img_sleep_bg, &wd_img_sleep_background);
    lv_obj_set_pos(_img_sleep_bg, 0, 0);

    _label_sleep_title = lv_label_create(p_window);
    // lv_obj_add_style(_label_sleep_title, (lv_style_t *)&SLEEP_TITLE_STYLE, LV_STATE_DEFAULT);
    lv_label_set_text_static(_label_sleep_title, "SLEEP");
    lv_obj_set_style_text_font(_label_sleep_title, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_obj_align(_label_sleep_title, LV_ALIGN_TOP_MID, 0, 50);

    _label_sleep_hour_val = lv_label_create(p_window);
    // lv_obj_add_style(_label_sleep_hour_val, (lv_style_t *)&SLEEP_LABEL_STYLE, LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(_label_sleep_hour_val, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_label_set_text_fmt(_label_sleep_hour_val, "%d", 8);
    lv_obj_align(_label_sleep_hour_val, LV_ALIGN_CENTER, -100, 0);
    // lv_obj_set_pos(_label_sleep_hour_val, 60, 150);

    _label_sleep_hour_unit = lv_label_create(p_window);
    // lv_obj_add_style(_label_sleep_hour_unit, (lv_style_t *)&SLEEP_LABEL_STYLE, LV_STATE_DEFAULT);
    // lv_label_set_text_static(_label_sleep_hour_unit, ISTR("时"));
    lv_obj_set_style_text_font(_label_sleep_hour_unit, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_label_set_text_static(_label_sleep_hour_unit, ("Hour"));
    // lv_obj_set_pos(_label_sleep_hour_unit, 100, 150);
    lv_obj_align_to(_label_sleep_hour_unit, _label_sleep_hour_val, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    _label_sleep_min_val = lv_label_create(p_window);
    // lv_obj_add_style(_label_sleep_min_val, (lv_style_t *)&SLEEP_LABEL_STYLE, LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(_label_sleep_min_val, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_label_set_text_fmt(_label_sleep_min_val, "%d", 30);
    // lv_obj_set_pos(_label_sleep_min_val, 210, 150);
    lv_obj_align(_label_sleep_min_val, LV_ALIGN_CENTER, 100, 0);

    _label_sleep_min_unit = lv_label_create(p_window);
    // lv_obj_add_style(_label_sleep_min_unit, (lv_style_t *)&SLEEP_LABEL_STYLE, LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(_label_sleep_min_unit, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_label_set_text_static(_label_sleep_min_unit, ("Min"));
    // lv_obj_set_pos(_label_sleep_min_unit, 250, 150);
    lv_obj_align_to(_label_sleep_min_unit, _label_sleep_min_val, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    _img_sleep_flag = lv_img_create(p_window);
    lv_img_set_src(_img_sleep_flag, &wd_img_sleep_icon_big);
    lv_obj_align(_img_sleep_flag, LV_ALIGN_TOP_MID, 0, 260);

    lv_obj_t * btn = lv_btn_create(p_window) ;
    lv_obj_set_size(btn, 120, 120);
    lv_obj_set_pos(btn, 120, 40);
    lv_obj_set_style_opa(btn, 0, 0);        /* set opa to 0, as hidden */
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_PRESSED, NULL);

    // _clear_and_reset_state();
    lv_obj_add_event_cb(p_window, sleep_event_cb, LV_EVENT_ALL, NULL);

    return p_window;
}

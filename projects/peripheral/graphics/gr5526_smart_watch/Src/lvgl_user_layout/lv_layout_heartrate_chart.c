#include "app_key.h"
#include "app_rtc.h"
#include "lvgl.h"
#include "lv_font.h"
#include "lv_user_font.h"
#include "lv_img_dsc_list.h"
#include <stdio.h>

#define CHART_PAD_VAL (20)
#define CHART_SERIES_LINE_WIDTH (3)
#define CHART_INDIC_RADIUS (6)
#define CHART_INDIC_SIZE (6)

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t *_img_hr_animation_big = NULL;
static lv_obj_t *_img_hr_animation_small = NULL;
static lv_obj_t *_label_hr_realtime_val = NULL;
static lv_obj_t *_label_hr_max_val = NULL;
static lv_obj_t *_label_hr_min_val = NULL;
static lv_timer_t *_hb_animation_timer = NULL;
static bool _is_show_big_hb_img = true;
static bool _is_hr_checked = false;

static lv_chart_series_t *_hr_ser_max = NULL;
static lv_chart_series_t *_hr_ser_min = NULL;

static lv_obj_t *_hr_chart = NULL;

static uint8_t _hr_24h_data_min[24] = {
    68, 80, 92, 95, 78, 120,
    79, 113, 68, 94, 80, 88,
    98, 70, 102, 115, 88, 92,
    96, 82, 66, 114, 90, 80};

static uint8_t _hr_24h_data_max[24] = {
    78, 85, 98, 99, 88, 129,
    99, 123, 78, 104, 88, 98,
    106, 78, 122, 125, 98, 99,
    99, 97, 86, 124, 100, 90};

static const lv_style_const_prop_t WINDOW_STYLE_PROPS[] = {
    LV_STYLE_CONST_PAD_TOP(CHART_PAD_VAL),
    LV_STYLE_CONST_PAD_BOTTOM(CHART_PAD_VAL),
    LV_STYLE_CONST_PAD_LEFT(CHART_PAD_VAL),
    LV_STYLE_CONST_PAD_RIGHT(CHART_PAD_VAL),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
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
    LV_STYLE_CONST_TEXT_ALIGN(LV_TEXT_ALIGN_CENTER),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t CHART_STYLE_PROPS[] = {
    LV_STYLE_CONST_PAD_TOP(CHART_PAD_VAL),
    LV_STYLE_CONST_PAD_BOTTOM(CHART_PAD_VAL),
    LV_STYLE_CONST_PAD_LEFT(CHART_PAD_VAL),
    LV_STYLE_CONST_PAD_RIGHT(CHART_PAD_VAL),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t CHART_SERIES_STYLE_PROPS[] = {
    LV_STYLE_CONST_LINE_WIDTH(CHART_SERIES_LINE_WIDTH),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t CHART_INDIC_STYLE_PROPS[] = {
    LV_STYLE_CONST_RADIUS(CHART_INDIC_RADIUS),
    LV_STYLE_CONST_WIDTH(CHART_INDIC_SIZE),
    LV_STYLE_CONST_HEIGHT(CHART_INDIC_SIZE),
    LV_STYLE_CONST_BG_OPA(LV_OPA_COVER),
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(HR_CHART_WINDOW_STYLE, WINDOW_STYLE_PROPS);
LV_STYLE_CONST_INIT(HR_CHART_REALTIME_LABEL_STYLE, REALTIME_LABEL_STYLE_PROPS);
LV_STYLE_CONST_INIT(HR_CHART_UNIT_LABEL_STYLE, UNIT_LABEL_STYLE_PROPS);
LV_STYLE_CONST_INIT(HR_CHART_STYLE, CHART_STYLE_PROPS);
LV_STYLE_CONST_INIT(HR_CHART_SERIES_STYLE, CHART_SERIES_STYLE_PROPS);
LV_STYLE_CONST_INIT(HR_CHART_INDIC_STYLE, CHART_INDIC_STYLE_PROPS);

/*
 * STATIC METHODS DECLARATION
 *****************************************************************************************
 */
static void _set_hr_realtime_val(uint16_t val);
static void _set_hr_min_max_val(lv_obj_t *p_obj, uint16_t val);
static void _set_heart_beat_animation(bool active);
static void _hb_timer_callback(lv_timer_t *tmr);
// static void         _clear_and_reset_state(void);

/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */

/*
 * STATIC METHODS IMPLEMENT
 *****************************************************************************************
 */
static void _prepare_hr_data(uint8_t hours)
{
    lv_chart_set_x_start_point(_hr_chart, _hr_ser_min, 0);
    lv_chart_set_x_start_point(_hr_chart, _hr_ser_max, 0);
    for (uint32_t i = 0; i < hours; i++)
    {
        lv_chart_set_next_value(_hr_chart, _hr_ser_min, _hr_24h_data_min[i]);
        lv_chart_set_next_value(_hr_chart, _hr_ser_max, _hr_24h_data_max[i]);
    }
}

static void _set_hr_realtime_val(uint16_t val)
{

    if (!_label_hr_realtime_val)
        return;

    if (val == 0)
    {
        _is_hr_checked = true;
        lv_label_set_text_static(_label_hr_realtime_val, "--");
    }
    else
    {
        _is_hr_checked = true;
        lv_label_set_text_fmt(_label_hr_realtime_val, "%d", val);
    }
    return;
}

static void _set_hr_min_max_val(lv_obj_t *p_obj, uint16_t val)
{
    if (!_label_hr_min_val)
        return;

    if (val == 0 || val == 0xFF)
    {
        lv_label_set_text_static(p_obj, "--");
    }
    else
    {
        lv_label_set_text_fmt(p_obj, "%d", val);
    }
    return;
}

static void _set_heart_beat_animation(bool active)
{
    if (active)
    {
        // set animation alive
        lv_timer_resume(_hb_animation_timer);
    }
    else
    {
        lv_timer_pause(_hb_animation_timer);
    }
}

static void _show_big_hr_image(bool state)
{
    if (state)
    {
        lv_obj_add_flag(_img_hr_animation_small, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(_img_hr_animation_big, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_clear_flag(_img_hr_animation_small, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(_img_hr_animation_big, LV_OBJ_FLAG_HIDDEN);
    }
}

static void _hb_timer_callback(lv_timer_t *tmr)
{
    _show_big_hr_image(_is_show_big_hb_img);

    if (_is_hr_checked)
    {
        _is_show_big_hb_img = !_is_show_big_hb_img;
    }
    else
    {
        _is_show_big_hb_img = true;
    }

    // Set the simulated heartrate value.
    _set_hr_realtime_val(80);
    _set_hr_min_max_val(_label_hr_max_val, 140);
    _set_hr_min_max_val(_label_hr_min_val, 70);
}

// static void _clear_and_reset_state(void)
//{
//     _set_hr_realtime_val(0);
//     _set_hr_min_max_val(_label_hr_max_val, 0);
//     _set_hr_min_max_val(_label_hr_max_val, 0);
//     _is_show_big_hb_img = true;
//     _show_big_hr_image(true);
//     _set_heart_beat_animation(false);
// }

static void draw_event_cb(lv_event_t *e)
{
   lv_obj_draw_part_dsc_t * dsc = lv_event_get_draw_part_dsc(e);
   if(!lv_obj_draw_part_check_type(dsc, &lv_chart_class, LV_CHART_DRAW_PART_TICK_LABEL)) return;

   if(dsc->id == LV_CHART_AXIS_PRIMARY_X && dsc->text) {
       const char * month[] = {"0", "2", "4", "6", "8", "10", "12", "14", "16", "18", "20", "22", "24"};
       lv_snprintf(dsc->text, dsc->text_length, "%s", month[dsc->value]);
   }
}

static void heartrate_chart_event_cb(lv_event_t *e)
{
    if (e->code == LV_EVENT_DELETE)
    {
        if (_hb_animation_timer)
        {
            lv_timer_del(_hb_animation_timer);
            _hb_animation_timer = NULL;
        }
    }
    else if (e->code == LV_EVENT_READY)
    {
        if (_hb_animation_timer)
        {
            _set_heart_beat_animation(true);
        }
        else
        {
            _hb_animation_timer = lv_timer_create(_hb_timer_callback, 350, NULL);
        }
    }
    else if (e->code == LV_EVENT_CANCEL)
    {
        if (_hb_animation_timer)
        {
            lv_timer_pause(_hb_animation_timer);
        }
    }
}

/*
 * GLOBAL METHODS IMPLEMENT
 *****************************************************************************************
 */
lv_obj_t *lv_layout_heartrate_chart_create(lv_obj_t *parent_tv_obj)
{
    lv_obj_t *p_window = lv_obj_create(parent_tv_obj);
    lv_obj_set_size(p_window, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_add_style(p_window, (lv_style_t *)&HR_CHART_WINDOW_STYLE, 0);

    _img_hr_animation_big = lv_img_create(p_window);
    lv_img_set_src(_img_hr_animation_big, &wd_img_heart_icon_70);
    lv_obj_set_pos(_img_hr_animation_big, 80, 25);

    _img_hr_animation_small = lv_img_create(p_window);
    lv_img_set_src(_img_hr_animation_small, &wd_img_heart_icon_50);
    lv_obj_set_pos(_img_hr_animation_small, 80, 35);

    _label_hr_realtime_val = lv_label_create(p_window);
    lv_obj_add_style(p_window, (lv_style_t *)&HR_CHART_WINDOW_STYLE, 0);
    lv_obj_set_size(_label_hr_realtime_val, 60, 40);
    lv_label_set_text_fmt(_label_hr_realtime_val, "%d", 80);
    lv_obj_add_style(_label_hr_realtime_val, (lv_style_t *)&HR_CHART_REALTIME_LABEL_STYLE, LV_STATE_DEFAULT);
    lv_obj_set_pos(_label_hr_realtime_val, 156, 40);

    lv_obj_t *p_label_hr_unit = lv_label_create(p_window);
    lv_label_set_text_static(p_label_hr_unit, "bpm");
    lv_obj_add_style(p_label_hr_unit, (lv_style_t *)&HR_CHART_UNIT_LABEL_STYLE, LV_STATE_DEFAULT);
    lv_obj_set_pos(p_label_hr_unit, 214, 50);

    { /* Set CHART */
        /*Create a chart*/
        _hr_chart = lv_chart_create(p_window);
        lv_obj_add_style(_hr_chart, (lv_style_t *)&HR_CHART_STYLE, LV_STATE_DEFAULT);
        lv_obj_add_style(_hr_chart, (lv_style_t *)&HR_CHART_SERIES_STYLE, LV_PART_ITEMS);
        lv_obj_add_style(_hr_chart, (lv_style_t *)&HR_CHART_INDIC_STYLE, LV_PART_INDICATOR);
        lv_obj_set_style_text_font(_hr_chart, &lv_font_montserrat_20, LV_STATE_DEFAULT);
        lv_obj_set_size(_hr_chart, 280, 160);
        lv_obj_set_pos(_hr_chart, 40, 124);
        lv_chart_set_type(_hr_chart, LV_CHART_TYPE_LINE);
        lv_chart_set_range(_hr_chart, LV_CHART_AXIS_SECONDARY_Y, 60, 180);
        lv_chart_set_point_count(_hr_chart, 24);
        lv_obj_add_event_cb(_hr_chart, draw_event_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
        lv_obj_set_style_bg_color(_hr_chart, lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_border_width(_hr_chart, 0, LV_PART_MAIN);

        /*Add ticks and label to every axis*/
        lv_chart_set_axis_tick(_hr_chart, LV_CHART_AXIS_PRIMARY_X, 0, 0, 13, 2, true, 40);
        lv_chart_set_axis_tick(_hr_chart, LV_CHART_AXIS_SECONDARY_Y, 0, 0, 6,  2, true, 40);
        lv_chart_set_div_line_count(_hr_chart, 7, 0);
        lv_obj_set_style_pad_column(_hr_chart, 1, LV_PART_MAIN);

        /*Add two data series*/
        _hr_ser_min = lv_chart_add_series(_hr_chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_SECONDARY_Y);
        _hr_ser_max = lv_chart_add_series(_hr_chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_SECONDARY_Y);

        lv_chart_refresh(_hr_chart); /*Required after direct set*/
    }

    _prepare_hr_data(24);

    // _clear_and_reset_state();

    lv_obj_add_event_cb(p_window, heartrate_chart_event_cb, LV_EVENT_ALL, NULL);

    return p_window;
}

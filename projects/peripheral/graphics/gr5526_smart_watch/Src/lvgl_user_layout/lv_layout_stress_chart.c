#include <stdio.h>
#include "lvgl.h"
#include "app_key.h"
#include "app_log.h"
#include "lv_font.h"
#include "lv_user_font.h"
#include "lv_img_dsc_list.h"

#define CHART_PAD_VAL (20)
#define CHART_SERIES_LINE_WIDTH (3)
#define CHART_INDIC_RADIUS (6)
#define CHART_INDIC_SIZE (6)

#pragma diag_suppress 177

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t *   _img_stress_state             = NULL;
static lv_obj_t *   _label_stress_realtime_val    = NULL;
static lv_obj_t *   _stress_chart                 = NULL;
static lv_timer_t *  _stress_score_update_timer   = NULL;

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

LV_STYLE_CONST_INIT(STRESS_CHART_WINDOW_STYLE, WINDOW_STYLE_PROPS);
LV_STYLE_CONST_INIT(STRESS_CHART_REALTIME_LABEL_STYLE, REALTIME_LABEL_STYLE_PROPS);
LV_STYLE_CONST_INIT(STRESS_CHART_STYLE, UNIT_LABEL_STYLE_PROPS);
LV_STYLE_CONST_INIT(STRESS_CHART_SERIES_STYLE, CHART_SERIES_STYLE_PROPS);
LV_STYLE_CONST_INIT(STRESS_CHART_INDIC_STYLE, CHART_INDIC_STYLE_PROPS);

/*
 * STATIC METHODs DECLARATION
 *****************************************************************************************
 */

/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */

//static lv_chart_series_t * _stress_ser_max = NULL;
static lv_chart_series_t * _stress_ser_min = NULL;

static uint8_t _stress_24h_data_min[24] = {\
    38, 40,  52,   55,  38, 80, \
    39, 73,  0,    54,  40, 48, \
    58, 30,  62,   75,  48, 52, \
    56, 42,  0,    0,   0,  0
};

static uint8_t _stress_24h_data_max[24] = {
    38, 45,  58,   59,  48, 89, \
    59, 83,  38,   64,  48, 58, \
    66, 38,  82,   85,  58, 59, \
    59, 57,  46,   84,  0,  0
};


/*
 * STATIC METHODS IMPLEMENT
 *****************************************************************************************
 */

static void _update_stress_state(uint16_t val) {
    if(!_img_stress_state)
        return ;

    if((val >= 80) && (val < 100)) {
        // lv_img_set_src(_img_stress_state, &wd_img_stress_high_50);
        lv_img_set_src(_img_stress_state, NULL);
    } else if ((val >= 60) && (val < 80)) {
        // lv_img_set_src(_img_stress_state, &wd_img_stress_middle_50);
        lv_img_set_src(_img_stress_state, NULL);
    } else if ((val >= 30) && (val < 60)) {
        // lv_img_set_src(_img_stress_state, &wd_img_stress_normal_50);
        lv_img_set_src(_img_stress_state, NULL);
    } else if ((val >= 1) && (val < 30)) {
        // lv_img_set_src(_img_stress_state, &wd_img_stress_easy_50);
        lv_img_set_src(_img_stress_state, NULL);
    } else {
        // lv_img_set_src(_img_stress_state, &wd_img_pressure_icon_50);
        lv_img_set_src(_img_stress_state, NULL);
    }

    if(!_label_stress_realtime_val)
        return;

    if(val > 100) {
        lv_label_set_text_static(_label_stress_realtime_val, "--");
    } else {
        lv_label_set_text_fmt(_label_stress_realtime_val, "%d", val);
    }
    return;
}


static void _prepare_stress_data(uint8_t hours) {
   lv_chart_set_x_start_point(_stress_chart, _stress_ser_min, 0);
   //lv_chart_set_x_start_point(_stress_chart, _stress_ser_max, 0);

   for (uint32_t i = 0; i < hours; i++)
   {
       lv_chart_set_next_value(_stress_chart, _stress_ser_min, _stress_24h_data_min[i]);
       //lv_chart_set_next_value(_stress_chart, _stress_ser_max, _stress_24h_data_max[i]);
   }
}


static void draw_event_cb(lv_event_t * e)
{
    lv_obj_draw_part_dsc_t * dsc = lv_event_get_draw_part_dsc(e);
    if(!lv_obj_draw_part_check_type(dsc, &lv_chart_class, LV_CHART_DRAW_PART_TICK_LABEL)) return;

    if(dsc->id == LV_CHART_AXIS_PRIMARY_X && dsc->text) {
        const char * month[] = {"0", "2", "4", "6", "8", "10", "12", "14", "16", "18", "20", "22", "24"};
        lv_snprintf(dsc->text, dsc->text_length, "%s", month[dsc->value]);
    }
}

static void _stress_score_update_timer_callback(lv_timer_t * tmr)
{
#if GOMORE_ENABLE > 0u
    uint8_t stress_score = (uint16_t)stress_score_get();
    _update_stress_state(stress_score);
#endif
}

static void stress_chart_event_cb(lv_event_t * e)
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

lv_obj_t *lv_layout_stress_chart_create(lv_obj_t *parent_tv_obj)
{
    lv_obj_t *p_window = lv_obj_create(parent_tv_obj);
    lv_obj_set_size(p_window, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_add_style(p_window, (lv_style_t *)&STRESS_CHART_WINDOW_STYLE, 0);

    // uint8_t stress_score = (uint16_t)stress_score_get();
    uint8_t stress_score = 60;

    _img_stress_state = lv_img_create(p_window);
    // _update_stress_state(stress_score);
    lv_img_set_src(_img_stress_state, &wd_img_pressure_icon_50);
    lv_obj_align(_img_stress_state, LV_ALIGN_TOP_MID, -40, 60);
    // lv_obj_set_pos(_img_stress_state, 80, 35);

    _label_stress_realtime_val = lv_label_create(p_window);
    // lv_obj_add_style(_label_stress_realtime_val, (lv_style_t *)&STRESS_CHART_REALTIME_LABEL_STYLE, 0);
    lv_obj_set_style_text_font(_label_stress_realtime_val, &lv_font_msyh_26_gr552x, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_label_stress_realtime_val, lv_color_white(), 0);
    lv_label_set_text_fmt(_label_stress_realtime_val, "%d", stress_score);
    // lv_obj_set_pos(_label_stress_realtime_val, 156, 40);
    lv_obj_align(_label_stress_realtime_val, LV_ALIGN_TOP_MID, 40, 60);

    /* Set CHART */
    /*Create a chart*/
    _stress_chart = lv_chart_create(p_window);
    // lv_obj_add_style(_stress_chart, (lv_style_t *)&STRESS_CHART_STYLE, 0);
    /*Add ticks and label to every axis*/
    lv_chart_set_point_count(_stress_chart, 24);
    lv_chart_set_type(_stress_chart, LV_CHART_TYPE_BAR);
    lv_chart_set_range(_stress_chart, LV_CHART_AXIS_SECONDARY_Y, 0, 100);
    lv_chart_set_update_mode(_stress_chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_axis_tick(_stress_chart, LV_CHART_AXIS_PRIMARY_X,   0, 0, 13, 2, true, 50);
    lv_chart_set_axis_tick(_stress_chart, LV_CHART_AXIS_SECONDARY_Y, 0, 0, 11,  2, true, 50);
    lv_chart_set_div_line_count(_stress_chart, 11, 0);
    lv_obj_set_style_text_font(_stress_chart, &lv_font_montserrat_20, LV_STATE_DEFAULT);
    lv_obj_set_size(_stress_chart, 280, 160);
    // lv_obj_set_pos(_stress_chart, 40, 124);
    lv_obj_align(_stress_chart, LV_ALIGN_CENTER, 0, 30);
    lv_obj_add_event_cb(_stress_chart, draw_event_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
    lv_obj_set_style_bg_color(_stress_chart, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(_stress_chart, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(_stress_chart, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_column(_stress_chart, 2, LV_PART_MAIN);

    /*Add two data series*/
    _stress_ser_min = lv_chart_add_series(_stress_chart, lv_palette_main(LV_PALETTE_ORANGE),   LV_CHART_AXIS_SECONDARY_Y);
    //_stress_ser_max = lv_chart_add_series(_stress_chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_SECONDARY_Y);

    lv_chart_refresh(_stress_chart); /*Required after direct set*/

    _prepare_stress_data(24);

    lv_obj_add_event_cb(p_window, stress_chart_event_cb, LV_EVENT_ALL, NULL);

//    _clear_and_reset_state();
    return p_window;
}

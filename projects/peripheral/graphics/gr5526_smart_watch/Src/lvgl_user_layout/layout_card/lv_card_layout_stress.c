#include <stdio.h>
#include "lvgl.h"
#include "lv_layout_router.h"
#include "lv_font.h"
#include "app_log.h"
#include "lv_user_font.h"
#include "lv_img_dsc_list.h"

#pragma diag_suppress 177

#define STRESS_TITLE_X (184)
#define STRESS_TITLE_Y (30)
#define STRESS_ICON_X (120)
#define STRESS_ICON_Y (72)
#define STRESS_VAL_X (196)
#define STRESS_VAL_Y (76)
#define STRESS_LEVEL_X (268)
#define STRESS_LEVEL_Y (86)
#define STRESS_MEAS_TIME_X (156)
#define STRESS_MEAS_TIME_Y (136)

#define STRESS_CHART_LEFT_PAD (6)
#define STRESS_CHART_BOTTOM_PAD (28)
#define STRESS_CHART_X_SPACING (15)
#define STRESS_CHART_X (40)
#define STRESS_CHART_Y (176)
#define STRESS_CHART_Y_MAX_TICKS_X (396)
#define STRESS_CHART_Y_MAX_TICKS_Y (176)
#define STRESS_CHART_Y_MIN_TICKS_X (396)
#define STRESS_CHART_Y_MIN_TICKS_Y (300)

#define STRESS_MAX_ICON_X (122)
#define STRESS_MAX_ICON_Y (374)
#define STRESS_MAX_VAL_X (146)
#define STRESS_MAX_VAL_Y (366)
#define STRESS_MIN_ICON_X (270)
#define STRESS_MIN_ICON_Y (374)
#define STRESS_MIN_VAL_X (294)
#define STRESS_MIN_VAL_Y (366)


/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */

static lv_obj_t *_stress_meas_time = NULL;
static lv_obj_t *_stress_val = NULL;
static lv_obj_t *_stress_max = NULL;
static lv_obj_t *_stress_min = NULL;
static lv_obj_t *_stress_val_line[24] = {NULL};
static lv_point_t _stress_val_pts[24][2];
static lv_obj_t *_stress_chart = NULL;
static lv_obj_t *_stress_chart_max_ticks = NULL;
static lv_obj_t *_stress_chart_min_ticks = NULL;
static lv_timer_t *_stress_animation_timer = NULL;


static uint8_t stress_data[] =
{40, 30, 50, 87, 60, 70,
 80, 85, 75, 36, 29, 72,
 91, 79, 43, 75, 64, 60,
 54, 37, 46, 18, 26, 10};

/*
 * STATIC METHODs DECLARATION
 *****************************************************************************************
 */

/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */

/*
 * STATIC METHODs IMPLEMENT
 *****************************************************************************************
 */

static uint32_t _stress_val_to_y_pos(uint8_t hr_val)
{
    uint32_t ret = STRESS_CHART_Y + wd_img_table_bg_scaled.header.h - STRESS_CHART_BOTTOM_PAD - hr_val / 220.f * (wd_img_table_bg_scaled.header.h - STRESS_CHART_BOTTOM_PAD);
    return ret;
}

static void _stress_set_max_min_val(uint8_t *p_data)
{
    for (uint8_t i = 0; i < 24; i++)
    {
        if (p_data[i] < 50)
        {
            lv_obj_set_style_line_color(_stress_val_line[i], lv_color_make(0x45,0x55,0xE3), 0);
        }
        else if (p_data[i] >= 50 && p_data[i] < 70)
        {
            lv_obj_set_style_line_color(_stress_val_line[i], lv_color_make(0x00,0xFF,0xF0), 0);
        }
        else
        {
            lv_obj_set_style_line_color(_stress_val_line[i], lv_color_make(0x9E,0x3E,0xEA), 0);
        }

        _stress_val_pts[i][0].x = STRESS_CHART_X + STRESS_CHART_LEFT_PAD + i * STRESS_CHART_X_SPACING;
        _stress_val_pts[i][0].y = _stress_val_to_y_pos(p_data[i]);
        _stress_val_pts[i][1].x = _stress_val_pts[i][0].x;
        _stress_val_pts[i][1].y = _stress_val_to_y_pos(0);
        lv_line_set_points(_stress_val_line[i], _stress_val_pts[i], 2);
    }
}

lv_obj_t *lv_card_layout_stress_create(lv_obj_t *parent_tv_obj)
{
    lv_obj_t *p_window = lv_obj_create(parent_tv_obj);
    lv_obj_set_size(p_window, DISP_HOR_RES, DISP_VER_RES);

    // title
    lv_obj_t *_stress_title = lv_label_create(p_window);
    lv_obj_set_style_text_font(_stress_title, &lv_font_montserrat_30, LV_STATE_DEFAULT); // 30
    lv_label_set_text_static(_stress_title, "Stress");
    lv_obj_set_pos(_stress_title, STRESS_TITLE_X, STRESS_TITLE_Y);

    // stress icon
    lv_obj_t *_stress_icon = lv_img_create(p_window);
    lv_img_set_src(_stress_icon, &wd_img_pressure_icon);
    lv_obj_set_pos(_stress_icon, STRESS_ICON_X, STRESS_ICON_Y);

    // stress value
    _stress_val = lv_label_create(p_window);
    lv_obj_set_style_text_font(_stress_val, &lv_font_montserrat_48_gdx, LV_STATE_DEFAULT); // 60
    lv_obj_set_style_text_color(_stress_val, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(_stress_val, "%d", 56);
    lv_obj_set_pos(_stress_val, STRESS_VAL_X, STRESS_VAL_Y);

    // stress level
    lv_obj_t *_stress_level = lv_label_create(p_window);
    lv_obj_set_style_text_font(_stress_level, &lv_font_montserrat_30, LV_STATE_DEFAULT); // 30
    lv_obj_set_style_text_color(_stress_level, lv_color_make(0xA0, 0xA0, 0xA0), LV_STATE_DEFAULT);
    lv_label_set_text_static(_stress_level, "Normal");
    lv_obj_set_pos(_stress_level, STRESS_LEVEL_X, STRESS_LEVEL_Y);

    // measure time
    _stress_meas_time = lv_label_create(p_window);
    lv_obj_set_style_text_font(_stress_meas_time, &lv_font_montserrat_26, LV_STATE_DEFAULT); // 30
    lv_obj_set_style_text_color(_stress_meas_time, lv_color_make(0xA0, 0xA0, 0xA0), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(_stress_meas_time, "%s", "5 Mins Ago");
    lv_obj_set_pos(_stress_meas_time, STRESS_MEAS_TIME_X, STRESS_MEAS_TIME_Y);

    // chart
    _stress_chart = lv_img_create(p_window);
    lv_img_set_src(_stress_chart, &wd_img_table_bg_scaled);
    lv_obj_set_pos(_stress_chart, STRESS_CHART_X, STRESS_CHART_Y);

    // stress max icon
    lv_obj_t *_stress_max_icon = lv_img_create(p_window);
    lv_img_set_src(_stress_max_icon, &wd_img_hr_max);
    lv_obj_set_pos(_stress_max_icon, STRESS_MAX_ICON_X, STRESS_MAX_ICON_Y);

    // stress max value
    _stress_max = lv_label_create(p_window);
    lv_obj_set_style_text_font(_stress_max, &lv_font_montserrat_26, LV_STATE_DEFAULT); // 30
    lv_label_set_text_fmt(_stress_max, "%d", 100);
    lv_obj_set_pos(_stress_max, STRESS_MAX_VAL_X, STRESS_MAX_VAL_Y);

    // stress min icon
    lv_obj_t *_stress_min_icon = lv_img_create(p_window);
    lv_img_set_src(_stress_min_icon, &wd_img_hr_min);
    lv_obj_set_pos(_stress_min_icon, STRESS_MIN_ICON_X, STRESS_MIN_ICON_Y);

    // stress min value
    _stress_min = lv_label_create(p_window);
    lv_obj_set_style_text_font(_stress_min, &lv_font_montserrat_26, LV_STATE_DEFAULT); // 30
    lv_label_set_text_fmt(_stress_min, "%d", 47);
    lv_obj_set_pos(_stress_min, STRESS_MIN_VAL_X, STRESS_MIN_VAL_Y);

    // stress line
    for (uint8_t i = 0; i < 24; i++)
    {
        _stress_val_line[i] = lv_line_create(p_window);
        lv_obj_set_style_line_rounded(_stress_val_line[i], true, 0);
        lv_obj_set_style_line_width(_stress_val_line[i], 4, 0);
        lv_obj_set_style_line_color(_stress_val_line[i], lv_color_make(255, 45, 54), 0);
        lv_obj_clear_flag(_stress_val_line[i], LV_OBJ_FLAG_CLICKABLE);
    }

    // stress ticks
    _stress_chart_max_ticks = lv_label_create(p_window);
    lv_obj_set_style_text_font(_stress_chart_max_ticks, &lv_font_montserrat_20, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_stress_chart_max_ticks, lv_color_make(0xA0, 0xA0, 0xA0), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(_stress_chart_max_ticks, "200");
    lv_obj_set_pos(_stress_chart_max_ticks, STRESS_CHART_Y_MAX_TICKS_X, STRESS_CHART_Y_MAX_TICKS_Y);

    _stress_chart_min_ticks = lv_label_create(p_window);
    lv_obj_set_style_text_font(_stress_chart_min_ticks, &lv_font_montserrat_20, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_stress_chart_min_ticks, lv_color_make(0xA0, 0xA0, 0xA0), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(_stress_chart_min_ticks, "0");
    lv_obj_set_pos(_stress_chart_min_ticks, STRESS_CHART_Y_MIN_TICKS_X, STRESS_CHART_Y_MIN_TICKS_Y);

    _stress_set_max_min_val(stress_data);

    return p_window;
}

#include "app_key.h"
#include "app_rtc.h"
#include "lvgl.h"
#include "lv_img_dsc_list.h"
#include "lv_user_font.h"
#include "app_log.h"

#include <stdio.h>

#define WEATHER_TITLE_X (160)
#define WEATHER_TITLE_Y (30)
#define WEATHER_ICON_X (120)
#define WEATHER_ICON_Y (84)
#define WEATHER_VAL_X (196)
#define WEATHER_VAL_Y (94)
#define WEATHER_LABEL_X (180)
#define WEATHER_LABEL_Y (180)
#define WEATHER_LINE_X (26)
#define WEATHER_LINE_Y (234)
#define WEATHER_FUTURE_DATE0_X (72)
#define WEATHER_FUTURE_DATE0_Y (250)
#define WEATHER_FUTURE_DATE1_X (194)
#define WEATHER_FUTURE_DATE1_Y (250)
#define WEATHER_FUTURE_DATE2_X (316)
#define WEATHER_FUTURE_DATE2_Y (250)
#define WEATHER_FUTURE_ICON0_X (70)
#define WEATHER_FUTURE_ICON0_Y (284)
#define WEATHER_FUTURE_ICON1_X (192)
#define WEATHER_FUTURE_ICON1_Y (284)
#define WEATHER_FUTURE_ICON2_X (312)
#define WEATHER_FUTURE_ICON2_Y (284)
#define WEATHER_FUTURE_VAL0_X (60)
#define WEATHER_FUTURE_VAL0_Y (352)
#define WEATHER_FUTURE_VAL1_X (182)
#define WEATHER_FUTURE_VAL1_Y (352)
#define WEATHER_FUTURE_VAL2_X (304)
#define WEATHER_FUTURE_VAL2_Y (352)

enum
{
    WEATHER_TYPE_CLEAR = 0,
    WEATHER_TYPE_CLOUDY,
    WEATHER_TYPE_DRIZZLE,
    WEATHER_TYPE_DUST,
    WEATHER_TYPE_GLOOMY,
    WEATHER_TYPE_HEAVY_RAIN,
    WEATHER_TYPE_HEAVY_SNOW,
    WEATHER_TYPE_LIGHT_SNOW,
    WEATHER_TYPE_MODERATE_RAIN,
    WEATHER_TYPE_MODERATE_SNOW,
} weather_type_t;

typedef struct
{
    uint8_t min_tem_val;
    uint8_t max_tem_val;
} future_tem_val_t;

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t *_weather_icon = NULL;
static lv_obj_t *_cur_tem_label = NULL;
static lv_obj_t *_weather_label = NULL;
static lv_obj_t *_future_weather_icon[3] = {NULL};
static lv_obj_t *_future_temp_val[3] = {NULL};
static lv_obj_t *_future_date[3] = {NULL};

static const char *WEATHER_TYPES[] = {"Clear", "Cloudy", "Drizzle", "Dust", "Gloomy", "Heavy Rain", "Heavy Snow", "Light Snow", "Moderate Rain", "Moderate Snow"};
static const lv_img_dsc_t *WEATHER_ICONS[] = {&wd_img_weather_clear,
                                              &wd_img_weather_cloudy,
                                              &wd_img_weather_gloomy,
                                              &wd_img_weather_heavy_rain,
                                              &wd_img_weather_heavy_snow,
                                              &wd_img_weather_light_snow,
                                              &wd_img_weather_moderate_rain,
                                              &wd_img_weather_moderate_snow};


/*
 * STATIC METHODS DECLARATION
 *****************************************************************************************
 */

/*
 * PUBLIC VARS DEFINITIONS
 *****************************************************************************************
 */

/*
 * STATIC METHODS IMPLEMENT
 *****************************************************************************************
 */
#if 0
 static void _weather_icon_update(lv_obj_t *p_obj, uint8_t weather_type)
 {
     switch (weather_type)
     {
     case WEATHER_TYPE_CLEAR:
         lv_img_set_src(p_obj, &wd_img_weather_Clear);
         break;
     case WEATHER_TYPE_CLOUDY:
         lv_img_set_src(p_obj, &wd_img_weather_cloudy);
         break;
     case WEATHER_TYPE_DRIZZLE:
         lv_img_set_src(p_obj, &wd_img_weather_Drizzle);
         break;
     case WEATHER_TYPE_DUST:
         lv_img_set_src(p_obj, &wd_img_weather_dust);
         break;
     case WEATHER_TYPE_GLOOMY:
         lv_img_set_src(p_obj, &wd_img_weather_gloomy);
         break;
     case WEATHER_TYPE_HEAVY_RAIN:
         lv_img_set_src(p_obj, &wd_img_weather_heavy_rain);
         break;
     case WEATHER_TYPE_HEAVY_SNOW:
         lv_img_set_src(p_obj, &wd_img_weather_heavy_snow);
         break;
     case WEATHER_TYPE_LIGHT_SNOW:
         lv_img_set_src(p_obj, &wd_img_weather_light_snow);
         break;
     case WEATHER_TYPE_MODERATE_RAIN:
         lv_img_set_src(p_obj, &wd_img_weather_moderate_rain);
         break;
     case WEATHER_TYPE_MODERATE_SNOW:
         lv_img_set_src(p_obj, &wd_img_weather_moderate_snow);
         break;
     default:
         break;
     }
 }

 static void _weather_label_update(lv_obj_t *p_obj, uint8_t weather_type)
 {
     switch (weather_type)
     {
     case WEATHER_TYPE_CLEAR:
         lv_label_set_text(p_obj, "");
         break;
     case WEATHER_TYPE_CLOUDY:
         lv_label_set_text(p_obj, "Cloudy");
         break;
     case WEATHER_TYPE_DRIZZLE:
         lv_label_set_text(p_obj, &wd_img_weather_Drizzle);
         break;
     case WEATHER_TYPE_DUST:
         lv_label_set_text(p_obj, &wd_img_weather_dust);
         break;
     case WEATHER_TYPE_GLOOMY:
         lv_label_set_text(p_obj, &wd_img_weather_gloomy);
         break;
     case WEATHER_TYPE_HEAVY_RAIN:
         lv_label_set_text(p_obj, &wd_img_weather_heavy_rain);
         break;
     case WEATHER_TYPE_HEAVY_SNOW:
         lv_label_set_text(p_obj, &wd_img_weather_heavy_snow);
         break;
     case WEATHER_TYPE_LIGHT_SNOW:
         lv_label_set_text(p_obj, &wd_img_weather_light_snow);
         break;
     case WEATHER_TYPE_MODERATE_RAIN:
         lv_label_set_text(p_obj, &wd_img_weather_moderate_rain);
         break;
     case WEATHER_TYPE_MODERATE_SNOW:
         lv_label_set_text(p_obj, &wd_img_weather_moderate_snow);
         break;
     default:
         break;
     }
 }
#endif

static void weather_event_cb(lv_event_t *e)
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

/*
 * GLOBAL METHODS IMPLEMENT
 *****************************************************************************************
 */
void weather_layout_cur_weather_update(char *p_loc, int16_t tem_val, uint8_t weather_type)
{
    // Weather icon
    lv_img_set_src(_weather_icon, WEATHER_ICONS[weather_type]);

    // Temperature value
    lv_label_set_text_fmt(_cur_tem_label, "%d", tem_val);

    // Weather label
    lv_label_set_text(_weather_label, WEATHER_TYPES[weather_type]);
}

void weather_layout_future_weather_update(uint8_t *p_weather_type, future_tem_val_t *p_tem_val)
{
    app_rtc_time_t time;
    app_rtc_get_time(&time);

    // Location label
    for (int i = 0; i < 3; i++)
    {
        lv_img_set_src(_future_weather_icon[i], WEATHER_ICONS[p_weather_type[i]]);
        lv_label_set_text_fmt(_future_temp_val[i], "%d/%d°", p_tem_val[i].min_tem_val, p_tem_val[i].max_tem_val);
        lv_label_set_text_fmt(_future_date[i], "%02d/%02d", time.mon, time.date + i + 1);
    }
}



lv_obj_t *lv_card_layout_weather_create(lv_obj_t *parent_tv_obj)
{
    lv_obj_t *p_window = lv_obj_create(parent_tv_obj);
    lv_obj_set_size(p_window, DISP_HOR_RES, DISP_VER_RES);

    // Weather Title
    lv_obj_t *p_title = lv_label_create(p_window);
    lv_label_set_text(p_title, "Weather");
    lv_obj_set_style_text_font(p_title, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(p_title, lv_color_white(), 0);
    lv_obj_set_pos(p_title, WEATHER_TITLE_X, WEATHER_TITLE_Y);

    // Weather Icon
    _weather_icon = lv_img_create(p_window);
    lv_img_set_src(_weather_icon, &wd_img_weather_clear);
    lv_obj_set_pos(_weather_icon, WEATHER_ICON_X, WEATHER_ICON_Y);

    // Temperature Value
    _cur_tem_label = lv_label_create(p_window);
    lv_label_set_text_fmt(_cur_tem_label, "%d°C", 28);
    lv_obj_set_style_text_font(_cur_tem_label, &lv_font_montserrat_48_gdx, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_cur_tem_label, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_set_pos(_cur_tem_label, WEATHER_VAL_X, WEATHER_VAL_Y);

    // Weather Label
    _weather_label = lv_label_create(p_window);
    lv_label_set_text(_weather_label, "Sunny");
    lv_obj_set_style_text_font(_weather_label, &lv_font_montserrat_30, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_weather_label, lv_color_make(0x99, 0x99, 0x99), 0);
    lv_obj_set_pos(_weather_label, WEATHER_LABEL_X, WEATHER_LABEL_Y);

    // Dividing Line
    static lv_point_t line_pts[] = {{0, 0}, {400, 0}};
    lv_obj_t *p_line = lv_line_create(p_window);
    lv_obj_set_style_line_width(p_line, 2, 0);
    lv_obj_set_style_line_color(p_line, lv_color_make(0x4B, 0x4B, 0x4B), 0);
    lv_obj_set_style_line_rounded(p_line, true, 0);
    lv_line_set_points(p_line, line_pts, 2);
    lv_obj_align_to(p_line, _weather_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
    lv_obj_set_pos(p_line, WEATHER_LINE_X, WEATHER_LINE_Y);

    // Tomorrow's weather icon
    _future_date[0] = lv_label_create(p_window);
    lv_label_set_text_fmt(_future_date[0], "%02d/%02d", 12, 7);
    lv_obj_set_style_text_font(_future_date[0], &lv_font_montserrat_26, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_future_date[0], lv_color_make(0xA0, 0xA0, 0xA0), 0);
    lv_obj_set_pos(_future_date[0], WEATHER_FUTURE_DATE0_X, WEATHER_FUTURE_DATE0_Y);

    _future_weather_icon[0] = lv_img_create(p_window);
    lv_img_set_src(_future_weather_icon[0], &wd_img_weather_clear);
    lv_obj_set_pos(_future_weather_icon[0], WEATHER_FUTURE_ICON0_X, WEATHER_FUTURE_ICON0_Y);

    _future_temp_val[0] = lv_label_create(p_window);
    lv_label_set_text_fmt(_future_temp_val[0], "%d°~%d°", 12, 25);
    lv_obj_set_style_text_font(_future_temp_val[0], &lv_font_montserrat_26, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_future_temp_val[0], lv_color_make(0xA0, 0xA0, 0xA0), 0);
    lv_obj_set_pos(_future_temp_val[0], WEATHER_FUTURE_VAL0_X, WEATHER_FUTURE_VAL0_Y);

    // Weather icon of the day after tomorrow
    _future_date[1] = lv_label_create(p_window);
    lv_label_set_text_fmt(_future_date[1], "%02d/%02d", 12, 8);
    lv_obj_set_style_text_font(_future_date[1], &lv_font_montserrat_26, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_future_date[1], lv_color_make(0xA0, 0xA0, 0xA0), 0);
    lv_obj_set_pos(_future_date[1], WEATHER_FUTURE_DATE1_X, WEATHER_FUTURE_DATE1_Y);

    _future_weather_icon[1] = lv_img_create(p_window);
    lv_img_set_src(_future_weather_icon[1], &wd_img_weather_dust);
    lv_obj_set_pos(_future_weather_icon[1], WEATHER_FUTURE_ICON1_X, WEATHER_FUTURE_ICON1_Y);

    _future_temp_val[1] = lv_label_create(p_window);
    lv_label_set_text_fmt(_future_temp_val[1], "%d°~%d°", 12, 25);
    lv_obj_set_style_text_font(_future_temp_val[1], &lv_font_montserrat_26, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_future_temp_val[1], lv_color_make(0xA0, 0xA0, 0xA0), 0);
    lv_obj_set_pos(_future_temp_val[1], WEATHER_FUTURE_VAL1_X, WEATHER_FUTURE_VAL1_Y);

    // Weather icon of three days from now
    _future_date[2] = lv_label_create(p_window);
    lv_label_set_text_fmt(_future_date[2], "%02d/%02d", 12, 9);
    lv_obj_set_style_text_font(_future_date[2], &lv_font_montserrat_26, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_future_date[2], lv_color_make(0xA0, 0xA0, 0xA0), 0);
    lv_obj_set_pos(_future_date[2], WEATHER_FUTURE_DATE2_X, WEATHER_FUTURE_DATE2_Y);

    _future_weather_icon[2] = lv_img_create(p_window);
    lv_img_set_src(_future_weather_icon[2], &wd_img_weather_cloudy);
    lv_obj_set_pos(_future_weather_icon[2], WEATHER_FUTURE_ICON2_X, WEATHER_FUTURE_ICON2_Y);

    _future_temp_val[2] = lv_label_create(p_window);
    lv_label_set_text_fmt(_future_temp_val[2], "%d°~%d°", 12, 25);
    lv_obj_set_style_text_font(_future_temp_val[2], &lv_font_montserrat_26, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_future_temp_val[2], lv_color_make(0xA0, 0xA0, 0xA0), 0);
    lv_obj_set_pos(_future_temp_val[2], WEATHER_FUTURE_VAL2_X, WEATHER_FUTURE_VAL2_Y);

    lv_obj_add_event_cb(p_window, weather_event_cb, LV_EVENT_ALL, NULL);

    return p_window;
}

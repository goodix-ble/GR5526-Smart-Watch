#include <stdio.h>
#include "app_key.h"
#include "lvgl.h"
#include "lv_font.h"
#include "lv_user_font.h"
#include "lv_img_dsc_list.h"
#include "app_graphics_mem.h"
#include "app_rtc.h"
#include "lv_clock_hands_draw.h"

#define CBUF_W      50
#define CBUF_H      20


/****************************************************************************************************
 *                      Static Declarations
 ****************************************************************************************************/
static lv_obj_t*    _s_img_hour         = NULL;
static lv_obj_t*    _s_img_second       = NULL;
static lv_obj_t*    _s_img_minute       = NULL;
static lv_obj_t*    _s_vivid_clock_win  = NULL;

static lv_obj_t*    _s_btn_date         = NULL;
static lv_obj_t*    _s_btn_alarm        = NULL;
static lv_obj_t*    _s_btn_timer        = NULL;
static lv_obj_t*    _s_btn_music        = NULL;

static lv_obj_t*    _s_img_alarm        = NULL;
static lv_obj_t*    _s_img_timer        = NULL;
static lv_obj_t*    _s_img_music        = NULL;

static lv_obj_t*    _s_img_hr           = NULL;
static lv_obj_t*    _s_img_step         = NULL;
static lv_obj_t*    _s_img_batery       = NULL;
static lv_obj_t*    _s_img_energy       = NULL;

static lv_obj_t*    _s_label_date       = NULL;
static lv_obj_t*    _s_label_hr         = NULL;
static lv_obj_t*    _s_label_step       = NULL;
static lv_obj_t*    _s_label_battery    = NULL;
static lv_obj_t*    _s_label_energy     = NULL;

static lv_img_dsc_t  wd_img_hr_font                        = {.data = NULL,};
static lv_img_dsc_t  wd_img_step_font                      = {.data = NULL,};
static lv_img_dsc_t  wd_img_battery_font                   = {.data = NULL,};
static lv_img_dsc_t  wd_img_energy_font                    = {.data = NULL,};
static lv_img_dsc_t  wd_img_vivid_clock_needle_second_copy = {.data = NULL,};
static lv_img_dsc_t  wd_img_vivid_clock_needle_minute_copy = {.data = NULL,};
static lv_img_dsc_t  wd_img_vivid_clock_needle_hour_copy   = {.data = NULL,};
static lv_img_dsc_t  wd_img_vivid_clock_battery_copy       = {.data = NULL,};
static lv_img_dsc_t  wd_img_vivid_clock_hr_copy            = {.data = NULL,};

static uint16_t _s_data_hr          = 0;
static uint16_t _s_data_step        = 1388;
static uint16_t _s_data_battery     = 50;
static uint16_t _s_data_energy      = 478;


static void             _lv_card_watchface_vivid_load_clock_needles_into_psram(void);
static void             _lv_card_watchface_vivid_unload_clock_needles_in_psram(void);
static void             _lv_card_watchface_vivid_layout_evt_handler(lv_event_t *e);
static void             _lv_card_watchface_vivid_layout_destroy(void);
static lv_img_dsc_t     _lv_card_watchface_vivid_draw_val_in_canvas(uint16_t val, lv_point_t * font_size);
static void             go_to_stopwatch(lv_event_t *evt);
static void             go_to_music(lv_event_t *evt);


extern void lv_port_res_mode_set(uint8_t mode);


lv_obj_t * lv_card_watchface_vivid_layout_create(lv_obj_t * parent) {

    _lv_card_watchface_vivid_load_clock_needles_into_psram();

    _s_vivid_clock_win = lv_obj_create(parent);
    lv_obj_set_style_bg_img_src(_s_vivid_clock_win, &wd_img_vivid_clock_bg, 0);
    lv_obj_set_style_bg_img_opa(_s_vivid_clock_win, 255, 0);
    // lv_obj_set_size(_s_vivid_clock_win, DISP_HOR_RES, DISP_VER_RES);
    // lv_obj_set_style_pad_all(_s_vivid_clock_win, 0, 0);

    // lv_obj_t *img_background = lv_img_create(_s_vivid_clock_win);
    // lv_img_set_src(img_background, &wd_img_vivid_clock_bg);
    // // Notice the background image is RGB565 format
    // lv_obj_set_pos(img_background, 0, 0);

    /* UP: Date Layout */
    _s_btn_date   = lv_btn_create(_s_vivid_clock_win);
    lv_obj_set_size(_s_btn_date, 100, 100);
    // lv_obj_set_style_radius(_s_btn_date, 50, 0);      //avoid the corner bg
    // lv_obj_set_style_opa(_s_btn_date, LV_OPA_30, 0);
    lv_obj_set_pos(_s_btn_date, 227 - 50, 126 - 50);

    _s_label_date = lv_label_create(_s_btn_date);
    lv_obj_center(_s_label_date);
    lv_obj_set_style_text_font(_s_label_date, &lv_font_msyh_26_gr552x, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(_s_label_date, lv_color_white(), 0);

    app_rtc_time_t time;
    app_rtc_get_time(&time);
    lv_label_set_text_fmt(_s_label_date, "%02d-%02d", time.mon, time.date );

    /* alarm layout*/
    _s_btn_alarm = lv_btn_create(_s_vivid_clock_win);
    lv_obj_set_size(_s_btn_alarm, 100, 100);
    // lv_obj_set_style_radius(_s_btn_alarm, 50, 0);      //avoid the corner bg
    // lv_obj_set_style_opa(_s_btn_alarm, LV_OPA_30, 0);
    lv_obj_set_pos(_s_btn_alarm, 126 - 50, 227 - 50);

    _s_img_alarm = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_img_alarm, &wd_img_vivid_clock_alarm);
    // lv_obj_set_size(_s_img_alarm, wd_img_vivid_clock_alarm.header.w, wd_img_vivid_clock_alarm.header.h);
    lv_obj_set_pos(_s_img_alarm, 126 - wd_img_vivid_clock_alarm.header.w/2, 227 - wd_img_vivid_clock_alarm.header.h/2);

    /* timer layout*/
    _s_btn_timer = lv_btn_create(_s_vivid_clock_win);
    lv_obj_set_size(_s_btn_timer, 100, 100);
    // lv_obj_set_style_radius(_s_btn_timer, 50, 0);      //avoid the corner bg
    // lv_obj_set_style_opa(_s_btn_timer, LV_OPA_30, 0);
    lv_obj_set_pos(_s_btn_timer, 328 - 50, 227 - 50);
    lv_obj_add_event_cb(_s_btn_timer, go_to_stopwatch, LV_EVENT_CLICKED, NULL);

    _s_img_timer = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_img_timer, &wd_img_vivid_clock_counter);
    // lv_obj_set_size(_s_img_timer, wd_img_vivid_clock_counter.header.w, wd_img_vivid_clock_counter.header.h);
    lv_obj_set_pos(_s_img_timer, 328 - wd_img_vivid_clock_counter.header.w/2, 227 - wd_img_vivid_clock_counter.header.h/2);

    /* music layout*/
    _s_btn_music = lv_btn_create(_s_vivid_clock_win);
    lv_obj_set_size(_s_btn_music, 100, 100);
    // lv_obj_set_style_radius(_s_btn_music, 50, 0);      //avoid the corner bg
    // lv_obj_set_style_opa(_s_btn_music, LV_OPA_30, 0);
    lv_obj_set_pos(_s_btn_music, 227 - 50, 328 - 50);
    lv_obj_add_event_cb(_s_btn_music, go_to_music, LV_EVENT_CLICKED, NULL);

    _s_img_music = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_img_music, &wd_img_vivid_clock_music);
    // lv_obj_set_size(_s_img_music, wd_img_vivid_clock_music.header.w, wd_img_vivid_clock_music.header.h);
    lv_obj_set_pos(_s_img_music, 227 - wd_img_vivid_clock_music.header.w/2, 328 - wd_img_vivid_clock_music.header.h/2);

    /* N*45 degree layout */
    _s_img_hr   = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_img_hr, &wd_img_vivid_clock_hr_copy);
    // lv_obj_set_size(_s_img_hr, wd_img_vivid_clock_hr_copy.header.w, wd_img_vivid_clock_hr_copy.header.h);
    lv_obj_set_pos(_s_img_hr, 126 - wd_img_vivid_clock_hr_copy.header.w/2, 126 - wd_img_vivid_clock_hr_copy.header.h/2);
    lv_img_set_angle(_s_img_hr, 3150);

    _s_img_step   = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_img_step, &wd_img_vivid_clock_step);
    // lv_obj_set_size(_s_img_step, wd_img_vivid_clock_step.header.w, wd_img_vivid_clock_step.header.h);
    lv_obj_set_pos(_s_img_step, 328 - wd_img_vivid_clock_step.header.w/2, 126 - wd_img_vivid_clock_step.header.h/2);
    lv_img_set_angle(_s_img_step, 450);

    _s_img_batery   = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_img_batery, &wd_img_vivid_clock_battery_copy);
    // lv_obj_set_size(_s_img_batery, wd_img_vivid_clock_battery_copy.header.w, wd_img_vivid_clock_battery_copy.header.h);
    lv_obj_set_pos(_s_img_batery, 126 - wd_img_vivid_clock_battery_copy.header.w/2, 328 - wd_img_vivid_clock_battery_copy.header.h/2);
    lv_img_set_angle(_s_img_batery, 2250);

    _s_img_energy   = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_img_energy, &wd_img_vivid_clock_energy);
    // lv_obj_set_size(_s_img_energy, wd_img_vivid_clock_energy.header.w, wd_img_vivid_clock_energy.header.h);
    lv_obj_set_pos(_s_img_energy, 328 - wd_img_vivid_clock_energy.header.w/2, 328 - wd_img_vivid_clock_energy.header.h/2);
    lv_img_set_angle(_s_img_energy, 1350);

    /* N*45 degree label layout */
    // HR
    lv_point_t font_size;
#if 1
    wd_img_hr_font = _lv_card_watchface_vivid_draw_val_in_canvas(_s_data_hr, &font_size);
    _s_label_hr = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_label_hr, &wd_img_hr_font);
    lv_img_set_angle(_s_label_hr, 3150);
    lv_img_set_pivot(_s_label_hr, CBUF_W/2, CBUF_H/2);
    lv_obj_set_pos(_s_label_hr, 69, 82);
    //printf("HR font size: %d.%d\r\n", font_size.x, font_size.y);

    wd_img_step_font = _lv_card_watchface_vivid_draw_val_in_canvas(_s_data_step, &font_size);
    _s_label_step = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_label_step, &wd_img_step_font);
    lv_img_set_angle(_s_label_step, 450);
    lv_img_set_pivot(_s_label_step, CBUF_W/2, CBUF_H/2);
    lv_obj_set_pos(_s_label_step, 328, 88);
    //printf("Step font size: %d.%d\r\n", font_size.x, font_size.y);

    wd_img_battery_font = _lv_card_watchface_vivid_draw_val_in_canvas(_s_data_battery, &font_size);
    _s_label_battery = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_label_battery, &wd_img_battery_font);
    lv_img_set_angle(_s_label_battery, 2250);
    lv_img_set_pivot(_s_label_battery, CBUF_W/2, CBUF_H/2);
    lv_obj_set_pos(_s_label_battery, 69, 346);
    //printf("BAT font size: %d.%d\r\n", font_size.x, font_size.y);

    wd_img_energy_font = _lv_card_watchface_vivid_draw_val_in_canvas(_s_data_energy, &font_size);
    _s_label_energy = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_label_energy, &wd_img_energy_font);
    lv_img_set_angle(_s_label_energy, 1350);
    lv_img_set_pivot(_s_label_energy, CBUF_W/2, CBUF_H/2);
    lv_obj_set_pos(_s_label_energy, 328,346);
    //printf("Energy font size: %d.%d\r\n", font_size.x, font_size.y);
#endif
    /* needle layout */
    _s_img_hour = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_img_hour, &wd_img_vivid_clock_needle_hour_copy);
    lv_obj_set_pos(_s_img_hour, 227, 221);

    _s_img_minute = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_img_minute, &wd_img_vivid_clock_needle_minute_copy);
    lv_obj_set_pos(_s_img_minute, 227, 221);

    _s_img_second = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(_s_img_second, &wd_img_vivid_clock_needle_second_copy);
    lv_obj_set_pos(_s_img_second, 202, 224);

    lv_obj_t *img_center = lv_img_create(_s_vivid_clock_win);
    lv_img_set_src(img_center, &wd_img_vivid_clock_center_point);
    lv_obj_set_pos(img_center, 217, 217);

    lv_img_set_pivot(_s_img_second, 25, 3);
    lv_img_set_pivot(_s_img_minute, 0, 7);
    lv_img_set_pivot(_s_img_hour, 0, 7);

    lv_obj_clear_flag(_s_vivid_clock_win, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_update_layout(_s_vivid_clock_win);

    lv_clk_set_hour_hand(_s_img_hour);
    lv_clk_set_min_hand(_s_img_minute);
    lv_clk_set_sec_hand(_s_img_second);
    lv_clk_set_bg_cb(NULL, NULL);
    lv_clk_hand_start_run();

    lv_obj_add_event_cb(_s_vivid_clock_win, _lv_card_watchface_vivid_layout_evt_handler, LV_EVENT_ALL, NULL);

    return _s_vivid_clock_win;
}

/****************************************************************************************************
 *                      Static Implements
 ****************************************************************************************************/

static void _lv_card_watchface_vivid_layout_destroy(void) {

    lv_clk_hand_stop_run();

    _lv_card_watchface_vivid_unload_clock_needles_in_psram();
    _s_img_hour         = NULL;
    _s_img_second       = NULL;
    _s_img_minute       = NULL;

    _s_btn_alarm         = NULL;
    _s_btn_timer         = NULL;
    _s_btn_music         = NULL;

    _s_img_alarm         = NULL;
    _s_img_timer         = NULL;
    _s_img_music         = NULL;

    _s_img_hr           = NULL;
    _s_img_step         = NULL;
    _s_img_batery       = NULL;
    _s_img_energy       = NULL;

    _s_label_hr           = NULL;
    _s_label_step         = NULL;
    _s_label_battery      = NULL;
    _s_label_energy       = NULL;

    _s_btn_alarm = _s_btn_alarm;
    _s_btn_timer = _s_btn_timer;
    _s_btn_music = _s_btn_music;

    _s_img_hr           = _s_img_hr;
    _s_img_step         = _s_img_step;
    _s_img_batery       = _s_img_batery;
    _s_img_energy       = _s_img_energy;

    _s_img_alarm         = _s_img_alarm;
    _s_img_timer         = _s_img_timer;
    _s_img_music         = _s_img_music;

    _s_label_hr           = _s_label_hr;
    _s_label_step         = _s_label_step;
    _s_label_battery      = _s_label_battery;
    _s_label_energy       = _s_label_energy;

    _s_vivid_clock_win  = NULL;
}

static lv_img_dsc_t _lv_card_watchface_vivid_draw_val_in_canvas(uint16_t val, lv_point_t * font_size) {

    char sbuf[8] = {0,0,0,0,0,0,0,0};
    void * cbuff = app_graphics_mem_malloc(CBUF_W*CBUF_H*2);

    memset(cbuff, 0, CBUF_W*CBUF_H*2);

    printf("Canvas Adr: 0x%08x \r\n", (int)cbuff);
    lv_obj_t * canvas = lv_canvas_create(NULL);
    lv_canvas_set_buffer(canvas, cbuff, CBUF_W, CBUF_H, LV_IMG_CF_GDX_RGB565);
    lv_obj_center(canvas);

    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_color_white();
    label_dsc.font = &lv_font_montserrat_20;

    sprintf(&sbuf[0], "%d", val);

    lv_txt_get_size(font_size, sbuf, label_dsc.font, label_dsc.letter_space, label_dsc.line_space, LV_COORD_MAX, label_dsc.flag);
    lv_canvas_draw_text(canvas, (CBUF_W - font_size->x)/2, (CBUF_H - font_size->y)/2, CBUF_W, &label_dsc, sbuf);

    lv_img_dsc_t  _canvas_image = {
        .header.always_zero = 0,
        .header.w = CBUF_W,
        .header.h = CBUF_H,
        .data_size = CBUF_W*CBUF_H*2,
        .header.cf = LV_IMG_CF_GDX_RGB565,
        .data = (uint8_t*)(cbuff),
        };

    return _canvas_image;
}


static void _lv_card_watchface_vivid_load_clock_needles_into_psram(void) {

    memcpy(&wd_img_vivid_clock_needle_second_copy, &wd_img_vivid_clock_needle_second,   sizeof(lv_img_dsc_t));
    memcpy(&wd_img_vivid_clock_needle_minute_copy, &wd_img_vivid_clock_needle_minute,   sizeof(lv_img_dsc_t));
    memcpy(&wd_img_vivid_clock_needle_hour_copy,   &wd_img_vivid_clock_needle_hour,     sizeof(lv_img_dsc_t));
    memcpy(&wd_img_vivid_clock_hr_copy,            &wd_img_vivid_clock_hr,              sizeof(lv_img_dsc_t));
    memcpy(&wd_img_vivid_clock_battery_copy,       &wd_img_vivid_clock_battery,         sizeof(lv_img_dsc_t));

    if(wd_img_vivid_clock_needle_second_copy.data != NULL) wd_img_vivid_clock_needle_second_copy.data = app_graphics_mem_malloc(wd_img_vivid_clock_needle_second_copy.data_size);
    if(wd_img_vivid_clock_needle_minute_copy.data != NULL) wd_img_vivid_clock_needle_minute_copy.data = app_graphics_mem_malloc(wd_img_vivid_clock_needle_minute_copy.data_size);
    if(wd_img_vivid_clock_needle_hour_copy.data   != NULL) wd_img_vivid_clock_needle_hour_copy.data   = app_graphics_mem_malloc(wd_img_vivid_clock_needle_hour_copy.data_size);
    if(wd_img_vivid_clock_hr_copy.data            != NULL) wd_img_vivid_clock_hr_copy.data            = app_graphics_mem_malloc(wd_img_vivid_clock_hr.data_size);
    if(wd_img_vivid_clock_battery_copy.data       != NULL) wd_img_vivid_clock_battery_copy.data       = app_graphics_mem_malloc(wd_img_vivid_clock_battery.data_size);

    lv_port_res_mode_set(2);
    memcpy((void*)wd_img_vivid_clock_needle_second_copy.data, (void*)wd_img_vivid_clock_needle_second.data, wd_img_vivid_clock_needle_second_copy.data_size);
    memcpy((void*)wd_img_vivid_clock_needle_minute_copy.data, (void*)wd_img_vivid_clock_needle_minute.data, wd_img_vivid_clock_needle_minute_copy.data_size);
    memcpy((void*)wd_img_vivid_clock_needle_hour_copy.data,   (void*)wd_img_vivid_clock_needle_hour.data,   wd_img_vivid_clock_needle_hour_copy.data_size);
    memcpy((void*)wd_img_vivid_clock_hr_copy.data,            (void*)wd_img_vivid_clock_hr.data,            wd_img_vivid_clock_hr_copy.data_size);
    memcpy((void*)wd_img_vivid_clock_battery_copy.data,       (void*)wd_img_vivid_clock_battery.data,       wd_img_vivid_clock_battery_copy.data_size);

    return;
}

static void _lv_card_watchface_vivid_unload_clock_needles_in_psram(void) {
    if(wd_img_vivid_clock_needle_second_copy.data != NULL) {
        app_graphics_mem_free((void*)wd_img_vivid_clock_needle_second_copy.data);
        wd_img_vivid_clock_needle_second_copy.data = NULL;
    }

    if(wd_img_vivid_clock_needle_minute_copy.data != NULL) {
        app_graphics_mem_free((void*)wd_img_vivid_clock_needle_minute_copy.data);
        wd_img_vivid_clock_needle_minute_copy.data = NULL;
    }

    if(wd_img_vivid_clock_needle_hour_copy.data != NULL) {
        app_graphics_mem_free((void*)wd_img_vivid_clock_needle_hour_copy.data);
        wd_img_vivid_clock_needle_hour_copy.data = NULL;
    }

    if(wd_img_vivid_clock_hr_copy.data != NULL) {
        app_graphics_mem_free((void*)wd_img_vivid_clock_hr_copy.data);
        wd_img_vivid_clock_hr_copy.data = NULL;
    }

    if(wd_img_vivid_clock_battery_copy.data != NULL) {
        app_graphics_mem_free((void*)wd_img_vivid_clock_battery_copy.data);
        wd_img_vivid_clock_battery_copy.data = NULL;
    }

    if(wd_img_hr_font.data != NULL) {
        app_graphics_mem_free((void*)wd_img_hr_font.data);
        wd_img_hr_font.data = NULL;
    }

    if(wd_img_step_font.data != NULL) {
        app_graphics_mem_free((void*)wd_img_step_font.data);
        wd_img_step_font.data = NULL;
    }

    if(wd_img_battery_font.data != NULL) {
        app_graphics_mem_free((void*)wd_img_battery_font.data);
        wd_img_battery_font.data = NULL;
    }

    if(wd_img_energy_font.data != NULL) {
        app_graphics_mem_free((void*)wd_img_energy_font.data);
        wd_img_energy_font.data = NULL;
    }
}


static void go_to_stopwatch(lv_event_t *evt)
{

}

static void go_to_music(lv_event_t *evt)
{

}


static void _lv_card_watchface_vivid_layout_evt_handler(lv_event_t *e)
{
    switch(e->code) {
        case LV_EVENT_DELETE:
        {
            _lv_card_watchface_vivid_layout_destroy();
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


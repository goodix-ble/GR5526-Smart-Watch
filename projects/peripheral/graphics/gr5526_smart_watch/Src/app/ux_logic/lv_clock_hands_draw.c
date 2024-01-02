/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "lvgl.h"
#include "gr55xx_hal.h"
#include "app_log.h"
#include "lv_img_dsc_list.h"
#include "app_graphics_mem.h"
#include "lv_clock_hands_draw.h"
#include "bsp_rtc.h"

/*
 * MACROS
 *****************************************************************************************
 */
#define HOUR_HAND_INDEX        2
#define MIN_HAND_INDEX         1
#define SEC_HAND_INDEX         0
#define BG_INDEX               3
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_rtc_time_t rtc_time;
static lv_anim_t clock_anim;
static lv_obj_t* hands_obj[4];
static uint32_t anim_pre_value;
static lv_clk_period_cb_t s_preriod_cb;
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

static void lv_clock_update_angle_cb(lv_anim_t* anim){
    uint32_t sec_angle = rtc_time.sec * 60 + rtc_time.ms * 60/1000 + 2700;
    uint32_t min_angle = rtc_time.min * 60 + 2700;
    uint32_t hour_angle = rtc_time.hour * 300 + rtc_time.min* 300/60 + 2700;

    if(hands_obj[SEC_HAND_INDEX] != NULL) {
        lv_img_set_angle(hands_obj[SEC_HAND_INDEX], sec_angle % 3600);
    }
    if(hands_obj[MIN_HAND_INDEX] != NULL) {
        lv_img_set_angle(hands_obj[MIN_HAND_INDEX], min_angle % 3600);
    }
    if(hands_obj[HOUR_HAND_INDEX] != NULL) {
        lv_img_set_angle(hands_obj[HOUR_HAND_INDEX], hour_angle % 3600);
    }
}

static void lv_clock_timer_step(app_rtc_time_t* time, uint32_t step_ms)
{
    uint8_t step_sec = step_ms/1000;
    step_ms = step_ms%1000;
    time->ms += step_ms;
    step_sec += time->ms/1000;
    time->ms = time->ms%1000;
    uint8_t step_min = (time->sec + step_sec)/60;
    time->sec = (time->sec + step_sec)%60;
    uint8_t step_hour = (time->min + step_min)/60;
    time->min = (time->min + step_min)%60;
    time->hour = (time->hour + step_hour)%24;
}

static void lv_clock_anim_cb(void * obj, int32_t v)
{
    static uint32_t i = 0;
    // static uint8_t round = false;
    lv_clock_timer_step(&rtc_time, (v + 60 * 1000 - anim_pre_value)%(60 * 1000));
    lv_clock_update_angle_cb(NULL);
    i++;
    if(hands_obj[BG_INDEX] != NULL)
        lv_img_set_angle(hands_obj[BG_INDEX], i * 15);
    if (s_preriod_cb != NULL)
        s_preriod_cb();

    if (rtc_get_updated_flag())
    {
        uint16_t _ms = rtc_time.ms;
        app_rtc_get_time(&rtc_time);
        rtc_time.ms = _ms;
        rtc_clear_updated_flag();
    }

    anim_pre_value = v;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void lv_clk_set_hour_hand(lv_obj_t* obj){
    hands_obj[HOUR_HAND_INDEX] = obj;
}

void lv_clk_set_min_hand(lv_obj_t* obj){
    hands_obj[MIN_HAND_INDEX] = obj;
}

void lv_clk_set_sec_hand(lv_obj_t* obj){
    hands_obj[SEC_HAND_INDEX] = obj;
}

void lv_clk_set_bg_cb(lv_obj_t* obj, lv_clk_period_cb_t period_cb){
    hands_obj[BG_INDEX] = obj;
    s_preriod_cb = period_cb;
}

void lv_clk_hand_start_run(void){
    app_rtc_get_time(&rtc_time);
    // millisec in rtc time is no longer available
    rtc_time.ms = 0;
    anim_pre_value = 0;
    lv_clk_hand_init();
    lv_clock_update_angle_cb(NULL);
    lv_anim_start(&clock_anim);
}

void lv_clk_hand_stop_run(void){
    lv_clk_set_hour_hand(NULL);
    lv_clk_set_min_hand(NULL);
    lv_clk_set_sec_hand(NULL);
    lv_clk_set_bg_cb(NULL, NULL);
    bool deleted = lv_anim_del((void *)0x12345678, lv_clock_anim_cb);
    APP_LOG_DEBUG("Clock Anim Deleted: %d", deleted);
}

void lv_clk_hand_init(void){
    lv_anim_init(&clock_anim);
    lv_anim_set_start_cb(&clock_anim, lv_clock_update_angle_cb);
    lv_anim_set_values(&clock_anim, 0, 60*1000);
    lv_anim_set_exec_cb(&clock_anim, lv_clock_anim_cb);
    lv_anim_set_time(&clock_anim, 60 * 1000);
    lv_anim_set_var(&clock_anim, (void *)0x12345678);
    lv_anim_set_repeat_count(&clock_anim, LV_ANIM_REPEAT_INFINITE);
}

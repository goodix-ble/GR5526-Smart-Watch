/*********************
 *      INCLUDES
 *********************/
#include "lv_port_indev.h"
#include "drv_adapter_touchpad.h"
#include <stdio.h>
#include "grx_hal.h"

/*********************
 *      DEFINES
 *********************/
/* 
 * TP read point cache number(most for 300ms TP points cache)
*/
#define TP_CACHE_NB   (20)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *      DECLARATIONS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void indev_drv_init(void);
static void touchpad_cache_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);

lv_indev_t * indev_touchpad;
static lv_indev_data_t s_touchpad_data[TP_CACHE_NB] = {0};
/* For TP cache points number measurement */
static volatile int8_t s_touchpad_max_idx = 0;
static volatile int8_t s_touchpad_save_idx = 0;
static volatile int8_t s_touchpad_read_idx = 0;
static lv_coord_t s_latest_x = 0;
static lv_coord_t s_latest_y = 0;
static lv_indev_state_t s_latest_state = LV_INDEV_STATE_RELEASED;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_indev_init(void)
{
    /*Initialize your touchpad if you have*/
    drv_adapter_touchpad_init();

    /*Register a touchpad input device*/
    indev_drv_init();
}


/*------------------
 * Touchpad
 * -----------------*/

/*Initialize your touchpad*/
static void indev_drv_init(void)
{
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_cache_read;
    indev_touchpad = lv_indev_drv_register(&indev_drv);
}

/*Will be called by the indev task to save the touchpad data*/
bool lv_port_indev_poll(void)
{
    bool ret = true;
    s_touchpad_max_idx = LV_MAX(s_touchpad_save_idx, s_touchpad_max_idx);
    if(s_touchpad_save_idx >= (int)TP_CACHE_NB){
        /* Discard the point because the LVGL engine not fast enough with the indev speed */
        return false;
    }

    lv_coord_t x_point, y_point;
    /*Save the pressed coordinates and the state*/
    if(drv_adapter_touchpad_read_pointer(&x_point, &y_point)) {
        s_latest_x = x_point & 0xfffe;
        s_latest_y = y_point;
        s_latest_state = LV_INDEV_STATE_PRESSED;
        ret = true;
    } else {
        s_latest_state = LV_INDEV_STATE_RELEASED;
        ret = false;
    }

    /*Set the last pressed coordinates*/
    s_touchpad_data[s_touchpad_save_idx].state = s_latest_state;
    s_touchpad_data[s_touchpad_save_idx].point.x = s_latest_x;
    s_touchpad_data[s_touchpad_save_idx].point.y = s_latest_y;
    s_touchpad_save_idx++;
    return ret;
}

/*Will be called by the LVGL indev timer to read the touchpad*/
static void touchpad_cache_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    /* Use the latest point when there is no cache point */
    if(s_touchpad_save_idx == 0){
        data->continue_reading = false;
        data->state = s_latest_state;
        data->point.x = s_latest_x;
        data->point.y = s_latest_y;
        return;
    }
    /* Set the last pressed coordinates */
    memcpy(data, &s_touchpad_data[s_touchpad_read_idx], sizeof(lv_indev_data_t));
    s_touchpad_read_idx++;
    GLOBAL_EXCEPTION_DISABLE();
    if(s_touchpad_read_idx < s_touchpad_save_idx){
        data->continue_reading = true;
    }else{
        data->continue_reading = false;
        s_touchpad_save_idx = 0;
        s_touchpad_read_idx = 0;
    }
    GLOBAL_EXCEPTION_ENABLE();
}

typedef int keep_pedantic_happy;

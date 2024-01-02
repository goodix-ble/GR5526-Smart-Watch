/**
 * @file lv_clock_hands_draw.h
 *
 * @brief LVGL clock hands draw module.
 */

#ifndef __GX_LV_CLOCK_HANDS_H__
#define __GX_LV_CLOCK_HANDS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include "app_rtc.h"

typedef void (*lv_clk_period_cb_t)(void);
/*
 * FUNCTION DECLARATIONS
 *****************************************************************************************
 */
/**
 * set clock hour hand 
 * @param[in] obj      pointer to a obj of hour hand
 */
void lv_clk_set_hour_hand(lv_obj_t* obj);
/**
 * set clock min hand 
 * @param[in] obj      pointer to a obj of min hand
 */
void lv_clk_set_min_hand(lv_obj_t* obj);
/**
 * set clock second hand 
 * @param[in] obj      pointer to a obj of sec hand
 */
void lv_clk_set_sec_hand(lv_obj_t* obj);
/**
 * set bg obj
 * @param[in] obj              pointer to a obj of background
 * @param[in] lv_clk_period_cb pointer to a funciton of rotation period
 */
void lv_clk_set_bg_cb(lv_obj_t* obj, lv_clk_period_cb_t period_cb);
/**
 *  start clock hands running
 */
void lv_clk_hand_start_run(void);
/**
 *  stop clock hands running
 */
void lv_clk_hand_stop_run(void);
/**
 *  init clock hands animal
 */
void lv_clk_hand_init(void);
#endif

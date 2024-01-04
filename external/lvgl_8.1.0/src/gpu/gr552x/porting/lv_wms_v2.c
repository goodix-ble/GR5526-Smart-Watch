/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdio.h>
#include "lvgl.h"
#include "lv_mem.h"
#include "lv_wms.h"
#include "disp_driver.h"
#include "lv_port_disp.h"
#include "app_graphics_mem.h"
#include "app_graphics_dc.h"

#include "layout_scene_manage.h"

#if WMS_VERSION == WMS_VERSION_v2

/*
 * STRUCTURE DEFINITIONS
 *****************************************************************************************
 */
typedef struct _lv_win_manager_t {
    lv_destroy_func_t self_destroy_func; /* destroy the current screen */
    lv_create_func_t left_create_func; /* left scroll target create function */
    lv_create_func_t right_create_func; /* right scroll target create function */
    lv_create_func_t top_create_func; /* top scroll target create function */
    lv_create_func_t bottom_create_func; /* bottom scroll target create function */
    lv_dir_t scroll_set_dir;            /* scroll setting to adapt layout */
    lv_dir_t inside_scroll_dir;         /* the object inside scroll dir setting */

    window_key_callback_func    key_event_handler;

    uint32_t window_id;

    uint32_t window_up_id;
    uint32_t window_down_id;
    uint32_t window_left_id;
    uint32_t window_right_id;
}lv_win_manager_t;

/*
 * LOCAL FUNCTION DECLARATIONS
 *****************************************************************************************
 */
static void lv_wms_setting_update(lv_obj_t* obj);
static void lv_wms_screen_press_event_cb(lv_event_t * e);
static void lv_wms_scroll_end_event_cb(lv_event_t * e);
static void lv_wms_slide_to_target(lv_obj_t* obj);
static void lv_wms_switch_screen_cache(void);
inline static uint32_t lv_wms_get_window_id(lv_obj_t* obj);


lv_obj_t * lv_wms_try_create_neighbor_window(lv_obj_t* obj, uint32_t direction);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static bool _switch_screen_is_cached = false;
static uint8_t _lv_wms_slide_state = LV_WMS_SLIDE_IDLE_STATE;
static lv_point_t _lv_wms_slide_sum = {0, 0};
static lv_point_t _lv_wms_gesture_sum = {0, 0};
static lv_coord_t _lv_wms_anim_step = 120;
static lv_dir_t _lv_wms_gesture_dir = LV_DIR_NONE;
static lv_dir_t _scroll_decide_dir = LV_DIR_NONE;
static lv_obj_t* _lv_wms_next_obj = NULL;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void lv_wms_slide_state_reset(void){
    _lv_wms_slide_state = LV_WMS_SLIDE_IDLE_STATE;
    _lv_wms_slide_sum.x = (lv_coord_t)0;
    _lv_wms_slide_sum.y = (lv_coord_t)0;
    _lv_wms_gesture_sum.x = (lv_coord_t)0;
    _lv_wms_gesture_sum.y = (lv_coord_t)0;
    _lv_wms_gesture_dir = LV_DIR_NONE;
    lv_refresh_enable_set(true);
    lv_display_enable_set(true);
    _switch_screen_is_cached = false;
    lv_wms_transit_mem_free();
}

static void lv_wms_scr_switch_prepare(lv_obj_t* current, lv_obj_t* next){
    /* Preload the next screen for the switch effect composition */
    lv_display_enable_set(false);
    lv_refresh_enable_set(true);
    /* Don't call lv_refr_now to avoid animation disruption */
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    disp->act_scr = next;
    /* Directly update and refresh to improve the efficiency */
    lv_obj_invalidate(next);
    _lv_disp_refr_timer(disp->refr_timer);
    lv_refresh_enable_set(false);
    /* Restore the current screen for the event handling continue */
    disp->act_scr = current;
    _lv_wms_slide_state = LV_WMS_SLIDING_STATE;
}

static void lv_wms_slide_dir_decide(lv_obj_t* obj){
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(LV_DIR_NONE == p_current_manager->scroll_set_dir) return; // never transit when scroll not set
    lv_indev_t * indev_act = lv_indev_get_act();
    _lv_wms_slide_sum.x += indev_act->proc.types.pointer.vect.x;
    _lv_wms_slide_sum.y += indev_act->proc.types.pointer.vect.y;
    if((LV_ABS(indev_act->proc.types.pointer.vect.x) < indev_act->driver->gesture_min_velocity) &&
       (LV_ABS(indev_act->proc.types.pointer.vect.y) < indev_act->driver->gesture_min_velocity)) {
        _lv_wms_gesture_sum.x = 0;
        _lv_wms_gesture_sum.y = 0;
        _lv_wms_gesture_dir = LV_DIR_NONE;
    }

    /*Count the movement by gesture*/
    _lv_wms_gesture_sum.x += indev_act->proc.types.pointer.vect.x;
    _lv_wms_gesture_sum.y += indev_act->proc.types.pointer.vect.y;

    if(LV_WMS_SLIDE_IDLE_STATE == _lv_wms_slide_state){
        /*Decide if it's a horizontal or vertical scroll*/
        bool hor_en = false;
        bool ver_en = false;
        if(LV_ABS(_lv_wms_slide_sum.x) > LV_ABS(_lv_wms_slide_sum.y)){
            hor_en = true;
        }else {
            ver_en = true;
        }

        /*Consider both up-down or left/right scrollable according to the current direction*/
        bool up_en    = ver_en && (p_current_manager->scroll_set_dir & LV_DIR_TOP);
        bool down_en  = ver_en && (p_current_manager->scroll_set_dir & LV_DIR_BOTTOM);
        bool left_en  = hor_en && (p_current_manager->scroll_set_dir & LV_DIR_LEFT);
        bool right_en = hor_en && (p_current_manager->scroll_set_dir & LV_DIR_RIGHT);

        /*If the object really can be scrolled into the current direction the use it.*/
        _lv_wms_next_obj = NULL;
#if 0
        if(left_en && ((lv_coord_t)_lv_wms_slide_sum.x >= (lv_coord_t)LV_INDEV_DEF_SCROLL_LIMIT)){
            if(p_current_manager->left_create_func){
                _lv_wms_next_obj = p_current_manager->left_create_func();
            }
            if(_lv_wms_next_obj){
                lv_wms_scr_switch_prepare(obj, _lv_wms_next_obj);
                _scroll_decide_dir = LV_DIR_LEFT;
            }
        }else if(right_en && ((lv_coord_t)_lv_wms_slide_sum.x <= (lv_coord_t)(-LV_INDEV_DEF_SCROLL_LIMIT))){
            if(p_current_manager->right_create_func){
                _lv_wms_next_obj = p_current_manager->right_create_func();
            }
            if(_lv_wms_next_obj){
                lv_wms_scr_switch_prepare(obj, _lv_wms_next_obj);
                _scroll_decide_dir = LV_DIR_RIGHT;
            }
        }else if(up_en && ((lv_coord_t)_lv_wms_slide_sum.y >= (lv_coord_t)LV_INDEV_DEF_SCROLL_LIMIT)){
            if(p_current_manager->top_create_func){
                _lv_wms_next_obj = p_current_manager->top_create_func();
            }
            if(_lv_wms_next_obj){
                lv_wms_scr_switch_prepare(obj, _lv_wms_next_obj);
                _scroll_decide_dir = LV_DIR_TOP;
            }
        }else if(down_en && ((lv_coord_t)_lv_wms_slide_sum.y <= (lv_coord_t)(-LV_INDEV_DEF_SCROLL_LIMIT))){
            if(p_current_manager->bottom_create_func){
                _lv_wms_next_obj = p_current_manager->bottom_create_func();
            }
            if(_lv_wms_next_obj){
                lv_wms_scr_switch_prepare(obj, _lv_wms_next_obj);
                _scroll_decide_dir = LV_DIR_BOTTOM;
            }
        }else{
            _lv_wms_slide_state = LV_WMS_SLIDE_IDLE_STATE;
        }
#else
        if(left_en && ((lv_coord_t)_lv_wms_slide_sum.x >= (lv_coord_t)LV_INDEV_DEF_SCROLL_LIMIT)){
            _lv_wms_next_obj = lv_wms_try_create_neighbor_window(obj, LV_DIR_LEFT);

            if(_lv_wms_next_obj){
                lv_wms_scr_switch_prepare(obj, _lv_wms_next_obj);
                _scroll_decide_dir = LV_DIR_LEFT;
            }
        }else if(right_en && ((lv_coord_t)_lv_wms_slide_sum.x <= (lv_coord_t)(-LV_INDEV_DEF_SCROLL_LIMIT))){
            _lv_wms_next_obj = lv_wms_try_create_neighbor_window(obj, LV_DIR_RIGHT);

            if(_lv_wms_next_obj){
                lv_wms_scr_switch_prepare(obj, _lv_wms_next_obj);
                _scroll_decide_dir = LV_DIR_RIGHT;
            }
        }else if(up_en && ((lv_coord_t)_lv_wms_slide_sum.y >= (lv_coord_t)LV_INDEV_DEF_SCROLL_LIMIT)){
            _lv_wms_next_obj = lv_wms_try_create_neighbor_window(obj, LV_DIR_TOP);

            if(_lv_wms_next_obj){
                lv_wms_scr_switch_prepare(obj, _lv_wms_next_obj);
                _scroll_decide_dir = LV_DIR_TOP;
            }
        }else if(down_en && ((lv_coord_t)_lv_wms_slide_sum.y <= (lv_coord_t)(-LV_INDEV_DEF_SCROLL_LIMIT))){
            _lv_wms_next_obj = lv_wms_try_create_neighbor_window(obj, LV_DIR_BOTTOM);

            if(_lv_wms_next_obj){
                lv_wms_scr_switch_prepare(obj, _lv_wms_next_obj);
                _scroll_decide_dir = LV_DIR_BOTTOM;
            }
        }else{
            _lv_wms_slide_state = LV_WMS_SLIDE_IDLE_STATE;
        }
#endif
    }else{
        _lv_wms_slide_state = LV_WMS_SLIDING_STATE;
        /*The object scroll shall limit according the decided direction.*/
        if(_scroll_decide_dir == LV_DIR_LEFT){
            if((lv_coord_t)_lv_wms_slide_sum.x < (lv_coord_t)0) _lv_wms_slide_sum.x = 0;
        }else if(_scroll_decide_dir == LV_DIR_RIGHT){
            if((lv_coord_t)_lv_wms_slide_sum.x > (lv_coord_t)0) _lv_wms_slide_sum.x = 0;
        }else if(_scroll_decide_dir == LV_DIR_TOP){
            if((lv_coord_t)_lv_wms_slide_sum.y < (lv_coord_t)0) _lv_wms_slide_sum.y = 0;
        }else if(_scroll_decide_dir == LV_DIR_BOTTOM){
            if((lv_coord_t)_lv_wms_slide_sum.y > (lv_coord_t)0) _lv_wms_slide_sum.y = 0;
        }
    }
}

static void lv_wms_display_mem_switch(void){
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    if(_switch_screen_is_cached && (draw_buf->buf1 && draw_buf->buf2)){
        // Switch the screen buffer to LVGL draw buffer
        if(draw_buf->buf_act == draw_buf->buf1)
            draw_buf->buf_act = draw_buf->buf2;
        else
            draw_buf->buf_act = draw_buf->buf1;
    }else{
        //lv_disp_draw_buf_init(&draw_buf_dsc, buf1, buf2, FRAME_SIZE)
        while(1); // Only support 2 full screen buffer config shown above
    }
}

static void lv_wms_transit_frame_update(lv_win_manager_t* p_current_manager) {
    lv_dir_t scroll_decide_dir          = _scroll_decide_dir;
    lv_point_t scroll_sum               = {_lv_wms_slide_sum.x, _lv_wms_slide_sum.y};
    lv_transit_direct_e trans_direct = LV_TRANSIT_DIRECT_NONE;
    lv_coord_t slide_offset             = (lv_coord_t)0;

    if (scroll_decide_dir & LV_DIR_HOR){
        slide_offset = LV_ABS(scroll_sum.x);
        if(scroll_sum.x  > (lv_coord_t)0){ // Right Scroll
            trans_direct = LV_TRANSIT_DIRECT_RIGHT;
        } else {
            trans_direct = LV_TRANSIT_DIRECT_LEFT;
        }
    } else {
        slide_offset = LV_ABS(scroll_sum.y);
        if(scroll_sum.y  > (lv_coord_t)0){ // Up Scroll
            trans_direct = LV_TRANSIT_DIRECT_UP;
        } else {
            trans_direct = LV_TRANSIT_DIRECT_DOWN;
        }
    }
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    // Render the transit view with slide offset
    lv_transit_frame_t * p_tf = lv_wms_transit_frame_render(draw_buf->buf_act,
                           lv_wms_transit_effect_get(), trans_direct, slide_offset);

    // Flush the frame buffer to screen
    gx_dc_flush_transition(draw_buf->buf_act, p_tf->transit_scrn_res_w,
                           p_tf->transit_scrn_res_h, HAL_GFX_TSC4);

    // Switch the screen flush buffer
    lv_wms_display_mem_switch();
}

static void lv_wms_screen_switch_to_target(lv_obj_t* obj, lv_dir_t dir){
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    // Reset to enable the LVGL refreshing
    lv_wms_slide_state_reset();
    switch(dir){
        case LV_DIR_LEFT:
        case LV_DIR_RIGHT:
        case LV_DIR_TOP:
        case LV_DIR_BOTTOM:{
            // Destroy the current screen
            if(p_current_manager->self_destroy_func){
                p_current_manager->self_destroy_func();
            }
            // Load the next screen
            lv_scr_load(_lv_wms_next_obj);
            break;
        }
        case LV_DIR_NONE:{
            lv_win_manager_t* p_next_manager = (lv_win_manager_t*)_lv_wms_next_obj->wms_data;
            if(p_next_manager && p_next_manager->self_destroy_func){
                p_next_manager->self_destroy_func();
            }
            lv_obj_invalidate(obj);
            break;
        }
    }

    // Refresh the screen directly when screen is changed
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    _lv_disp_refr_timer(disp->refr_timer);
}

void lv_wms_go_to_window(lv_create_func_t next_create_func){
    // It is forbidden to switch screen in window switch state
    if(lv_wms_is_in_busy_state()) return;
    if(NULL != lv_scr_act()){
        lv_win_manager_t* p_current_manager = (lv_win_manager_t*)lv_scr_act()->wms_data;
        if(p_current_manager && p_current_manager->self_destroy_func){
            p_current_manager->self_destroy_func();
        }
    }
    if(next_create_func) next_create_func();
    // Refresh the screen directly when screen is changed
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    _lv_disp_refr_timer(disp->refr_timer);
}

void lv_wms_destroy_current(void){
    // It is forbidden to switch screen in window switch state
    if(lv_wms_is_in_busy_state())
        return;
    if(NULL != lv_scr_act()){
        lv_win_manager_t* p_current_manager = (lv_win_manager_t*)lv_scr_act()->wms_data;
        if(p_current_manager && p_current_manager->self_destroy_func){
            p_current_manager->self_destroy_func();
        }
    }
    return;
}

static void lv_wms_slide_to_target(lv_obj_t* obj){
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(LV_WMS_SLIDING_STATE != _lv_wms_slide_state) return; // do nothing when not scrolling
    // Calculate the gesture direction
    lv_indev_t * indev_act = lv_indev_get_act();
    if(LV_WMS_SLIDING_STATE == _lv_wms_slide_state){
        if((LV_ABS(_lv_wms_gesture_sum.x) > indev_act->driver->gesture_limit) ||
           (LV_ABS(_lv_wms_gesture_sum.y) > indev_act->driver->gesture_limit)) {
            if(LV_ABS(indev_act->proc.types.pointer.gesture_sum.x) > \
               LV_ABS(indev_act->proc.types.pointer.gesture_sum.y)) {
                if(indev_act->proc.types.pointer.gesture_sum.x > (lv_coord_t)0)
                    _lv_wms_gesture_dir = LV_DIR_RIGHT;
                else
                    _lv_wms_gesture_dir = LV_DIR_LEFT;
            }else {
                if(indev_act->proc.types.pointer.gesture_sum.y > (lv_coord_t)0)
                    _lv_wms_gesture_dir = LV_DIR_BOTTOM;
                else
                    _lv_wms_gesture_dir = LV_DIR_TOP;
            }
        }
    }
    // Animate slide the current window to the target window
    switch(_scroll_decide_dir){
        case LV_DIR_LEFT:{
            if((_lv_wms_slide_sum.x  > (lv_coord_t)(DISP_HOR_RES>>1)) ||\
               (LV_DIR_RIGHT == _lv_wms_gesture_dir)){ // Scroll To Left Target
                while(_lv_wms_slide_sum.x < (lv_coord_t)DISP_HOR_RES){
                    _lv_wms_slide_sum.x += _lv_wms_anim_step;
                    if(_lv_wms_slide_sum.x < (lv_coord_t)DISP_HOR_RES)
                        lv_wms_transit_frame_update(p_current_manager);
                    else
                        break;
                }
                _lv_wms_slide_sum.x = DISP_HOR_RES;
                lv_wms_transit_frame_update(p_current_manager);
                lv_wms_screen_switch_to_target(obj, LV_DIR_LEFT);
            }else{ // Scroll To Self
                while(_lv_wms_slide_sum.x > (lv_coord_t)0){
                    _lv_wms_slide_sum.x -= _lv_wms_anim_step;
                    if(_lv_wms_slide_sum.x > (lv_coord_t)0)
                        lv_wms_transit_frame_update(p_current_manager);
                    else
                        break;
                }
                _lv_wms_slide_sum.x = (lv_coord_t)0;
                lv_wms_transit_frame_update(p_current_manager);
                lv_wms_screen_switch_to_target(obj, LV_DIR_NONE);
            }
            break;
        }
        case LV_DIR_RIGHT:{
            if((_lv_wms_slide_sum.x  > (lv_coord_t)(-(DISP_HOR_RES>>1))) &&
               (LV_DIR_LEFT != _lv_wms_gesture_dir)){ // Scroll To Self
                while(_lv_wms_slide_sum.x < (lv_coord_t)0){
                    _lv_wms_slide_sum.x += _lv_wms_anim_step;
                    if(_lv_wms_slide_sum.x < (lv_coord_t)0)
                        lv_wms_transit_frame_update(p_current_manager);
                    else
                        break;
                }
                _lv_wms_slide_sum.x = (lv_coord_t)0;
                lv_wms_transit_frame_update(p_current_manager);
                lv_wms_screen_switch_to_target(obj, LV_DIR_NONE);
            }else{ // Scroll To Right Target
                while(_lv_wms_slide_sum.x > (lv_coord_t)(-DISP_HOR_RES)){
                    _lv_wms_slide_sum.x -= _lv_wms_anim_step;
                    if(_lv_wms_slide_sum.x > (lv_coord_t)(-DISP_HOR_RES))
                        lv_wms_transit_frame_update(p_current_manager);
                    else
                        break;
                }
                _lv_wms_slide_sum.x = (lv_coord_t)(-DISP_HOR_RES);
                lv_wms_transit_frame_update(p_current_manager);
                lv_wms_screen_switch_to_target(obj, LV_DIR_RIGHT);
            }
            break;
        }
        case LV_DIR_TOP:{
            if((_lv_wms_slide_sum.y  > (lv_coord_t)(DISP_VER_RES>>1)) || \
               (LV_DIR_BOTTOM == _lv_wms_gesture_dir)){ // Scroll To Top Target
                while(_lv_wms_slide_sum.y < (lv_coord_t)DISP_VER_RES){
                    _lv_wms_slide_sum.y += _lv_wms_anim_step;
                    if(_lv_wms_slide_sum.y < (lv_coord_t)DISP_VER_RES)
                        lv_wms_transit_frame_update(p_current_manager);
                    else
                        break;
                }
                _lv_wms_slide_sum.y = DISP_VER_RES;
                lv_wms_transit_frame_update(p_current_manager);
                lv_wms_screen_switch_to_target(obj, LV_DIR_TOP);
            }else{ // Scroll To Self
                while(_lv_wms_slide_sum.y > (lv_coord_t)0){
                    _lv_wms_slide_sum.y -= _lv_wms_anim_step;
                    if(_lv_wms_slide_sum.y > (lv_coord_t)0)
                        lv_wms_transit_frame_update(p_current_manager);
                    else
                        break;
                }
                _lv_wms_slide_sum.y = (lv_coord_t)0;
                lv_wms_transit_frame_update(p_current_manager);
                lv_wms_screen_switch_to_target(obj, LV_DIR_NONE);
            }
            break;
        }
        case LV_DIR_BOTTOM:{
            if((_lv_wms_slide_sum.y  > (lv_coord_t)(-DISP_VER_RES>>1)) && \
               (LV_DIR_TOP != _lv_wms_gesture_dir)){ // Scroll To Self
                while(_lv_wms_slide_sum.y < (lv_coord_t)0){
                    _lv_wms_slide_sum.y += _lv_wms_anim_step;
                    if(_lv_wms_slide_sum.y < (lv_coord_t)0)
                        lv_wms_transit_frame_update(p_current_manager);
                    else
                        break;
                }
                _lv_wms_slide_sum.y = (lv_coord_t)0;
                lv_wms_transit_frame_update(p_current_manager);
                lv_wms_screen_switch_to_target(obj, LV_DIR_NONE);
            }else{ // Scroll To Bottom Target
                while(_lv_wms_slide_sum.y > (lv_coord_t)(-DISP_VER_RES)){
                    _lv_wms_slide_sum.y -= _lv_wms_anim_step;
                    if(_lv_wms_slide_sum.y > (lv_coord_t)(-DISP_VER_RES))
                        lv_wms_transit_frame_update(p_current_manager);
                    else
                        break;
                }
                _lv_wms_slide_sum.y = (lv_coord_t)(-DISP_VER_RES);
                lv_wms_transit_frame_update(p_current_manager);
                lv_wms_screen_switch_to_target(obj, LV_DIR_BOTTOM);
            }
            break;
        }
        default:{
            break;
        }
    }
}

static void lv_wms_switch_screen_cache(void){
    // Alloc buffer and cache the surface for screen switch
    lv_wms_transit_mem_alloc();
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    void* src_addr = draw_buf->buf_act;
    void* src2_addr = (draw_buf->buf_act == draw_buf->buf2) ? (draw_buf->buf1) : (draw_buf->buf2);
    lv_wms_transit_screen_cache(LV_TRANSIT_CACHE_SCRN_CURRENT, (uint32_t)src_addr);
    lv_wms_transit_screen_cache(LV_TRANSIT_CACHE_SCRN_NEXT,    (uint32_t)src2_addr);
}

static void lv_wms_screen_press_event_cb(lv_event_t * e)
{
    lv_obj_t* obj = e->current_target;
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(NULL == p_current_manager) return;
    if(LV_DIR_NONE == p_current_manager->scroll_set_dir) return;
    lv_indev_t * indev_act = lv_indev_get_act();
    switch(e->code){
        case LV_EVENT_PRESSED:{
            // Reset the scroll state to idle when pressed
            lv_wms_slide_state_reset();
            break;
        }
        case LV_EVENT_PRESSING:{
            // Decide the scroll direction firstly
            lv_wms_slide_dir_decide(obj);
            if(LV_WMS_SLIDING_STATE == _lv_wms_slide_state){
                // Prepare buffers and generate the switch frame
                if(!_switch_screen_is_cached) {
                    lv_wms_switch_screen_cache();
                    _switch_screen_is_cached = true;
                }
                lv_wms_transit_frame_update(p_current_manager);
            }
            break;
        }
        case LV_EVENT_RELEASED:{
            // Scroll to target screen when press released
            lv_wms_slide_to_target(obj);
            break;
        }
        default:{
            break;
        }
    }
}

static void lv_obj_get_scroll_boundary(lv_obj_t * obj, lv_coord_t* left, lv_coord_t* right,
                                             lv_coord_t* top, lv_coord_t* bottom){
    *left = LV_COORD_MAX; // find min
    *right = LV_COORD_MIN; // find max
    *top = LV_COORD_MAX; // find min
    *bottom = LV_COORD_MIN; // find max
    uint32_t i;
    uint32_t child_cnt = lv_obj_get_child_cnt(obj);
    for(i = 0; i < child_cnt; i++){
        lv_obj_t * child = obj->spec_attr->children[i];
        if(lv_obj_has_flag_any(child,  LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_FLOATING)) continue;
        *left = LV_MIN(*left, child->coords.x1); // find min
        *right = LV_MAX(*right, child->coords.x2); // find max
        *top = LV_MIN(*top, child->coords.y1); // find min
        *bottom = LV_MAX(*bottom, child->coords.y2); // find max
    }
}

static void lv_wms_slide_inside_outside_update(lv_obj_t* obj, lv_dir_t dir){
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(NULL == p_current_manager) return; // never transit when switch manager not set
    lv_coord_t left, right, top, bottom;
    lv_obj_get_scroll_boundary(obj, &left, &right, &top, &bottom);
    if(dir & LV_DIR_VER & p_current_manager->inside_scroll_dir){
        if((top >= obj->coords.y1) &&
           (p_current_manager->top_create_func)){
            p_current_manager->scroll_set_dir |= LV_DIR_TOP;
            lv_obj_set_scroll_dir(obj, lv_obj_get_scroll_dir(obj) & (~LV_DIR_TOP));
        }else{
            p_current_manager->scroll_set_dir &= (~LV_DIR_TOP);
            lv_obj_set_scroll_dir(obj, lv_obj_get_scroll_dir(obj) | (LV_DIR_TOP));
        }
        if((bottom <= obj->coords.y2) &&
           (p_current_manager->bottom_create_func)){
            p_current_manager->scroll_set_dir |= LV_DIR_BOTTOM;
            lv_obj_set_scroll_dir(obj, lv_obj_get_scroll_dir(obj) & (~LV_DIR_BOTTOM));
        }else{
            p_current_manager->scroll_set_dir &= (~LV_DIR_BOTTOM);
            lv_obj_set_scroll_dir(obj, lv_obj_get_scroll_dir(obj) | (LV_DIR_BOTTOM));
        }
    }
    if(dir & LV_DIR_HOR & p_current_manager->inside_scroll_dir){
        if((left >=  obj->coords.x1) &&
           (p_current_manager->left_create_func)){
            p_current_manager->scroll_set_dir |= LV_DIR_LEFT;
            lv_obj_set_scroll_dir(obj, lv_obj_get_scroll_dir(obj) & (~LV_DIR_LEFT));
        }else{
            p_current_manager->scroll_set_dir &= (~LV_DIR_LEFT);
            lv_obj_set_scroll_dir(obj, lv_obj_get_scroll_dir(obj) | (LV_DIR_LEFT));
        }
        if((right <=  obj->coords.x2) &&
            (p_current_manager->right_create_func)){
            p_current_manager->scroll_set_dir |= LV_DIR_RIGHT;
            lv_obj_set_scroll_dir(obj, lv_obj_get_scroll_dir(obj) & (~LV_DIR_RIGHT));
        }else{
            p_current_manager->scroll_set_dir &= (~LV_DIR_RIGHT);
            lv_obj_set_scroll_dir(obj, lv_obj_get_scroll_dir(obj) | (LV_DIR_RIGHT));
        }
    }
}

static void lv_wms_scroll_end_event_cb(lv_event_t * e)
{
    lv_obj_t* obj = e->current_target;
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(NULL == p_current_manager) return; // never transit when switch manager not set
    lv_indev_t * indev_act = lv_indev_get_act();

    switch(e->code){ // update the inside and outside scroll strategy
        case LV_EVENT_SCROLL_END:{
            lv_wms_slide_inside_outside_update(obj, indev_act->proc.types.pointer.scroll_dir);
            break;
        }
        default:{
            break;
        }
    }
}

static void lv_wms_setting_update(lv_obj_t* obj){
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(p_current_manager == NULL) return;
    if(NULL == p_current_manager->left_create_func){
        p_current_manager->scroll_set_dir &= (~LV_DIR_LEFT);
    }else{
        p_current_manager->scroll_set_dir |= LV_DIR_LEFT;
    }
    if(NULL == p_current_manager->right_create_func){
        p_current_manager->scroll_set_dir &= (~LV_DIR_RIGHT);
    }else{
        p_current_manager->scroll_set_dir |= LV_DIR_RIGHT;
    }
    if(NULL == p_current_manager->bottom_create_func){
        p_current_manager->scroll_set_dir &= (~LV_DIR_BOTTOM);
    }else{
        p_current_manager->scroll_set_dir |= LV_DIR_BOTTOM;
    }
    if(NULL == p_current_manager->top_create_func){
        p_current_manager->scroll_set_dir &= (~LV_DIR_TOP);
    }else{
        p_current_manager->scroll_set_dir |= LV_DIR_TOP;
    }

    /* The object is scrollable to a direction if its content overflow in that direction.*/
    lv_wms_slide_inside_outside_update(obj, p_current_manager->scroll_set_dir);
}


inline static uint32_t lv_wms_get_window_id(lv_obj_t* obj) {

    if(NULL == obj)
        return 0;

    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;

    if(NULL == p_current_manager)
        return 0;

    return p_current_manager->window_id;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void lv_wms_init(lv_obj_t* obj){
    if(NULL == obj) return;
    if(NULL != obj->wms_data) return;
    obj->wms_data = lv_mem_alloc(sizeof(lv_win_manager_t));
    lv_memset(obj->wms_data, 0, sizeof(lv_win_manager_t));

    // For scroll behavior implementation based on press event
    lv_obj_add_event_cb(obj, lv_wms_screen_press_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(obj, lv_wms_screen_press_event_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(obj, lv_wms_screen_press_event_cb, LV_EVENT_RELEASED, NULL);

    // For scrollable object dynamic switch scroll inside or outside
    lv_obj_add_event_cb(obj, lv_wms_scroll_end_event_cb, LV_EVENT_SCROLL_END, NULL);

    // Initialize the surface configuration
    lv_transit_config_t t_config = {
        .scrn_res_w = DISP_HOR_RES,
        .scrn_res_h = DISP_VER_RES,
        .fb_format = HAL_GFX_RGB565,
    };
    lv_wms_transit_starup(&t_config);

    // record the original object inside scroll dir
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    p_current_manager->inside_scroll_dir = lv_obj_get_scroll_dir(obj);
}

void lv_wms_deinit(lv_obj_t* obj){
    if(NULL == obj) return;
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(NULL != p_current_manager){
        lv_obj_remove_event_cb(obj, lv_wms_screen_press_event_cb);
        lv_obj_remove_event_cb(obj, lv_wms_scroll_end_event_cb);
        lv_mem_free(obj->wms_data);
    }
}

void lv_wms_inside_scroll_dir_update(lv_obj_t* obj){
    if(NULL == obj) return;
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(NULL == p_current_manager) return;
    p_current_manager->inside_scroll_dir = lv_obj_get_scroll_dir(obj);
}

void lv_wms_self_destroy_func_set(lv_obj_t* obj, lv_destroy_func_t func){
    if(NULL == obj) return;
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(NULL == p_current_manager) return;
    p_current_manager->self_destroy_func = func;
}

void lv_wms_left_create_func_set(lv_obj_t* obj, lv_create_func_t func){
    if(NULL == obj) return;
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(NULL == p_current_manager) return;
    p_current_manager->left_create_func = func;
    lv_wms_setting_update(obj);
    lv_wms_add_flag_recursive_children(obj, LV_OBJ_FLAG_EVENT_BUBBLE);
}

void lv_wms_right_create_func_set(lv_obj_t* obj, lv_create_func_t func){
    if(NULL == obj) return;
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(NULL == p_current_manager) return;
    p_current_manager->right_create_func = func;
    lv_wms_setting_update(obj);
    lv_wms_add_flag_recursive_children(obj, LV_OBJ_FLAG_EVENT_BUBBLE);
}

void lv_wms_bottom_create_func_set(lv_obj_t* obj, lv_create_func_t func){
    if(NULL == obj) return;
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(NULL == p_current_manager) return;
    p_current_manager->bottom_create_func = func;
    lv_wms_setting_update(obj);
    lv_wms_add_flag_recursive_children(obj, LV_OBJ_FLAG_EVENT_BUBBLE);
}

void lv_wms_top_create_func_set(lv_obj_t* obj, lv_create_func_t func){
    if(NULL == obj) return;
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(NULL == p_current_manager) return;
    p_current_manager->top_create_func = func;
    lv_wms_setting_update(obj);
    lv_wms_add_flag_recursive_children(obj, LV_OBJ_FLAG_EVENT_BUBBLE);
}

void lv_wms_key_handler_func_set(lv_obj_t* obj, window_key_callback_func func) {
    if(NULL == obj)
        return;
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(NULL == p_current_manager)
        return;
    p_current_manager->key_event_handler = func;
}

bool lv_wms_key_handler_execute(lv_obj_t* obj, uint32_t key, uint32_t event) {
    bool ret = false;

    if(NULL == obj)
        return false;

    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;

    if(NULL == p_current_manager)
        return false;

    if(p_current_manager->key_event_handler == NULL)
        return false;

    ret = p_current_manager->key_event_handler(key, event);

    return ret;
}


static void lv_wms_update_direction(lv_obj_t* obj){
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(p_current_manager == NULL) return;

    if(0 == p_current_manager->window_up_id){
        p_current_manager->scroll_set_dir &= (~LV_DIR_TOP);
    }else{
        p_current_manager->scroll_set_dir |= LV_DIR_TOP;
    }

    if(0 == p_current_manager->window_down_id){
        p_current_manager->scroll_set_dir &= (~LV_DIR_BOTTOM);
    }else{
        p_current_manager->scroll_set_dir |= LV_DIR_BOTTOM;
    }

    if(0 == p_current_manager->window_left_id){
        p_current_manager->scroll_set_dir &= (~LV_DIR_LEFT);
    }else{
        p_current_manager->scroll_set_dir |= LV_DIR_LEFT;
    }

    if(0 == p_current_manager->window_right_id){
        p_current_manager->scroll_set_dir &= (~LV_DIR_RIGHT);
    }else{
        p_current_manager->scroll_set_dir |= LV_DIR_RIGHT;
    }
}


void lv_wms_update_neighbor_setting(lv_obj_t* obj, uint32_t win_id) {
    if(NULL == obj)
        return;
    lv_win_manager_t* p_current_manager = (lv_win_manager_t*)obj->wms_data;
    if(NULL == p_current_manager)
        return;
    p_current_manager->window_id = win_id;

    lv_wms_win_neighbor_id_t _nid;
    _nid =  lv_wms_scene_find_all_neighbor_window_id(lv_wms_get_cur_scene_id(), win_id);

    p_current_manager->window_up_id     = _nid.win_up_id;
    p_current_manager->window_down_id   = _nid.win_down_id;
    p_current_manager->window_left_id   = _nid.win_left_id;
    p_current_manager->window_right_id  = _nid.win_right_id;

    lv_wms_update_direction(obj);
}

lv_obj_t * lv_wms_try_create_neighbor_window(lv_obj_t* obj, uint32_t direction) {
    uint32_t win_id , neighbor_win_id;
    lv_wms_window_t * neighbor_win = NULL;
    lv_obj_t * ret = NULL;

    win_id = lv_wms_get_window_id(obj);

    if(0 == win_id) {
        return NULL;
    }

    neighbor_win_id =  lv_wms_scene_find_neighbor_window_id(lv_wms_get_cur_scene_id(), win_id, direction);

    if(0 == neighbor_win_id) {
        return NULL;
    } else {
        neighbor_win = lv_wms_window_lookup(neighbor_win_id);

        if(neighbor_win != NULL) {
            ret = neighbor_win->create_func(neighbor_win_id);
            lv_wms_set_cur_win_id(neighbor_win_id);
        }
    }

    return ret;
}

void lv_wms_ani_step_set(uint8_t step){
    _lv_wms_anim_step = step;
}

void lv_wms_add_flag_recursive_children(lv_obj_t* obj, lv_obj_flag_t flag){
    /* Bubble event for scroll event build */
    uint32_t child_cnt = lv_obj_get_child_cnt(obj);
    for(uint32_t i = 0; i < child_cnt; i++) {
        lv_obj_t * child = obj->spec_attr->children[i];
        lv_obj_add_flag(child, flag);
        uint32_t child_child_cnt = lv_obj_get_child_cnt(child);
        if(child_child_cnt > 0){
            lv_wms_add_flag_recursive_children(child, flag);
        }
    }
}

void lv_wms_add_flag_for_direct_child(lv_obj_t* obj, lv_obj_flag_t flag){
    /* Bubble event for scroll event build */
    uint32_t child_cnt = lv_obj_get_child_cnt(obj);
    for(uint32_t i = 0; i < child_cnt; i++) {
        lv_obj_t * child = obj->spec_attr->children[i];
        lv_obj_add_flag(child, flag);
    }
}

void lv_wms_clear_flag_recursive_children(lv_obj_t* obj, lv_obj_flag_t flag){
    /* Bubble event for scroll event build */
    uint32_t child_cnt = lv_obj_get_child_cnt(obj);
    for(uint32_t i = 0; i < child_cnt; i++) {
        lv_obj_t * child = obj->spec_attr->children[i];
        lv_obj_clear_flag(child, flag);
        uint32_t child_child_cnt = lv_obj_get_child_cnt(child);
        if(child_child_cnt > 0){
            lv_wms_clear_flag_recursive_children(child, flag);
        }
    }
}

void lv_wms_clear_flag_for_direct_child(lv_obj_t* obj, lv_obj_flag_t flag){
    /* Bubble event for scroll event build */
    uint32_t child_cnt = lv_obj_get_child_cnt(obj);
    for(uint32_t i = 0; i < child_cnt; i++) {
        lv_obj_t * child = obj->spec_attr->children[i];
        lv_obj_clear_flag(child, flag);
    }
}

bool lv_wms_is_in_busy_state(void){
    return (LV_WMS_SLIDING_STATE == _lv_wms_slide_state);
}

#endif /* #if WMS_VERSION == WMS_VERSION_v2 */

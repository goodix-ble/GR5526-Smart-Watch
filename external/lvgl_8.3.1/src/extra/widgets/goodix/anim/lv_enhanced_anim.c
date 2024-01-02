/**
 * @file lv_port_anim.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_disp.h"
#include "../misc/lv_math.h"
#include "../core/lv_refr.h"

#include "lv_wms_surface_flinger.h"
#include "lv_enhanced_anim.h"
#include "drv_adapter_display.h"
#include "app_graphics_dc.h"
#include "gr55xx_delay.h"


/*************************************************************************************************
 *                                   Static Declaration
 *************************************************************************************************/
static void _lv_port_scr_anim_transform_start(lv_anim_t * a);
static void _lv_port_scr_anim_transform_finish(lv_anim_t * a);
static void _lv_port_scr_anim_transform_delete(lv_anim_t * a);
static void _lv_port_scr_anim_transform(void * obj, int32_t v);
static void _lv_port_scr_anim_frame_update(uint32_t slide_offset, lv_transit_direct_e trans_direct);
static void _lv_port_scr_anim_preload(lv_obj_t* next);
static void _lv_port_scr_anim_cache(void);
static void _lv_port_scr_anim_display_mem_switch(void);
static void _lv_port_scr_load_postpone(lv_timer_t * timer);


static bool                 s_anim_transit_is_progress  = false;
static bool                 s_anim_exception_delete     = false;
static bool                 s_switch_screen_is_cached   = false;
static uint32_t             s_anim_progress_start       = 0;
static lv_transit_effect_e  s_anim_transform_effect     = LV_TRANS_EFFECT_SPIN_H_L2R;
static lv_timer_t *         s_anim_postpone_timer       = NULL;
static lv_obj_t *           s_anim_lastest_scr          = NULL;



//TODO: Implement the SYNC Method
__weak void __wait_flush_done(void){}

/*************************************************************************************************
 *                                   PUBLIC Methods
 *************************************************************************************************/

void lv_scr_load_anim_enhance(lv_obj_t * new_scr, lv_transit_effect_e transit_effect, uint32_t time, uint32_t delay, bool auto_del)
{
    lv_disp_t * d = lv_obj_get_disp(new_scr);
    lv_obj_t * act_scr = lv_scr_act();

    /* if transit anim is progress, postpone to load new screent in direct-load mode */
    if(s_anim_transit_is_progress) {

        if(s_anim_postpone_timer == NULL) {
            s_anim_postpone_timer = lv_timer_create(_lv_port_scr_load_postpone, 1000, NULL);
        }

        /* postpone timer to avoid conflict */
        s_anim_lastest_scr = new_scr;
        lv_timer_reset(s_anim_postpone_timer);
        lv_timer_resume(s_anim_postpone_timer);

        return;
    }

    /* if last transit anim is delete unexpectedly, load new screent in direct-load mode */
    if(s_anim_exception_delete == true) {
        s_anim_exception_delete = false;
        lv_scr_load(new_scr);

        return;
    }

    s_anim_exception_delete = false;

    switch(transit_effect) {
        case LV_TRANS_EFFECT_CUBE:
        case LV_TRANS_EFFECT_INNERCUBE:
        case LV_TRANS_EFFECT_STACK:
        case LV_TRANS_EFFECT_FADE:
        case LV_TRANS_EFFECT_FADE_ZOOM:
        case LV_TRANS_EFFECT_SPIN_H_L2R:
        case LV_TRANS_EFFECT_SPIN_H_R2L:
            s_anim_transform_effect = transit_effect;
            break;

        default:
            s_anim_transform_effect = LV_TRANS_EFFECT_SPIN_H_L2R;
            break;
    }

    lv_wms_display_enabled_set(false);
    /*If an other screen load animation is in progress
     *make target screen loaded immediately. */
    if(d->scr_to_load && act_scr != d->scr_to_load) {
        lv_disp_load_scr(d->scr_to_load);
        lv_anim_del(d->scr_to_load, NULL);
        lv_obj_set_pos(d->scr_to_load, 0, 0);
        lv_obj_remove_local_style_prop(d->scr_to_load, LV_STYLE_OPA, 0);

        if(d->del_prev) {
            lv_obj_del(act_scr);
        }
        act_scr = d->scr_to_load;
    }

    d->scr_to_load = new_scr;

    if(d->prev_scr && d->del_prev) {
        lv_obj_del(d->prev_scr);
        d->prev_scr = NULL;
    }

    d->del_prev = auto_del;

    /*Be sure there is no other animation on the screens*/
    lv_anim_del(new_scr, NULL);
    if(act_scr) {
        lv_anim_del(act_scr, NULL);
    }

    /*Be sure both screens are in a normal position*/
    lv_obj_set_pos(new_scr, 0, 0);
    lv_obj_remove_local_style_prop(new_scr, LV_STYLE_OPA, 0);
    if(act_scr) {
        lv_obj_set_pos(act_scr, 0, 0);
        lv_obj_remove_local_style_prop(act_scr, LV_STYLE_OPA, 0);
    }

    s_anim_progress_start = 0;
    _lv_port_scr_anim_preload(new_scr);
    if(!s_switch_screen_is_cached) {
        _lv_port_scr_anim_cache();
        s_switch_screen_is_cached = true;
    }

    s_anim_transit_is_progress = true;

    lv_anim_t a_new;
    lv_anim_init(&a_new);
    lv_anim_set_var(&a_new, new_scr);
    lv_anim_set_user_data(&a_new,  (void*)0x00000000);
    lv_anim_set_start_cb(&a_new,   _lv_port_scr_anim_transform_start);
    lv_anim_set_exec_cb(&a_new,    _lv_port_scr_anim_transform );
    lv_anim_set_ready_cb(&a_new,   _lv_port_scr_anim_transform_finish);
    lv_anim_set_deleted_cb(&a_new, _lv_port_scr_anim_transform_delete);
    lv_anim_set_time(&a_new, time);
    lv_anim_set_delay(&a_new, delay);

    lv_anim_start(&a_new);

    return;
}


/*************************************************************************************************
 *                                Static Implement
 *************************************************************************************************/

static void _lv_port_scr_anim_transform_start(lv_anim_t * a)
{
    lv_disp_t * d = lv_obj_get_disp(a->var);

    d->prev_scr = lv_scr_act();
    d->act_scr  = a->var;

    lv_event_send(d->act_scr, LV_EVENT_SCREEN_LOAD_START, NULL);

    return;
}


static void _lv_port_scr_anim_transform_finish(lv_anim_t * a)
{
    lv_disp_t * d = lv_obj_get_disp(a->var);
    __wait_flush_done();
    s_switch_screen_is_cached  = false;
    s_anim_progress_start      = 0;
    s_anim_transit_is_progress = false;

    lv_wms_refresh_enabled_set(true);
    lv_scr_load(d->act_scr);
    lv_wms_display_enabled_set(true);

    lv_event_send(d->act_scr,  LV_EVENT_SCREEN_LOADED,   NULL);
    lv_event_send(d->prev_scr, LV_EVENT_SCREEN_UNLOADED, NULL);

    if(d->prev_scr && d->del_prev) {
        lv_obj_del(d->prev_scr);
    }
    d->prev_scr    = NULL;
    d->scr_to_load = NULL;
    lv_obj_remove_local_style_prop(a->var, LV_STYLE_OPA, 0);

    s_anim_transit_is_progress = false;
    s_anim_exception_delete = false;

    lv_anim_set_user_data(a, (void*)0xDEADBEAF);

    return;
}

static void _lv_port_scr_anim_transform_delete(lv_anim_t * a) {
    lv_disp_t * d = lv_obj_get_disp(a->var);
    __wait_flush_done();

    if(0xDEADBEAF == (uint32_t)lv_anim_get_user_data(a)) {
        s_anim_exception_delete   = false;
    } else {
        s_anim_exception_delete   = true;
    }

    s_switch_screen_is_cached = false;
    s_anim_progress_start  = 0;
    s_anim_transit_is_progress = false;

    lv_wms_refresh_enabled_set(true);

    lv_event_send(d->act_scr,  LV_EVENT_SCREEN_LOADED,   NULL);
    lv_event_send(d->prev_scr, LV_EVENT_SCREEN_UNLOADED, NULL);

    if(d->prev_scr && d->del_prev) {
        lv_obj_del(d->prev_scr);
    }
    d->prev_scr    = NULL;
    d->scr_to_load = NULL;
    lv_obj_remove_local_style_prop(a->var, LV_STYLE_OPA, 0);

    lv_wms_display_enabled_set(true);

    return;
}

static void _lv_port_scr_load_postpone(lv_timer_t * timer) {
    if(s_anim_lastest_scr != NULL) {
        lv_scr_load(s_anim_lastest_scr);
        s_anim_lastest_scr = NULL;
    }
    return;
}

static void _lv_port_scr_anim_frame_update(uint32_t slide_offset, lv_transit_direct_e trans_direct) {
    //lv_transit_direct_e trans_direct = LV_TRANSIT_DIRECT_RIGHT;
    lv_disp_t * disp                 = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf    = lv_disp_get_draw_buf(disp);

    // Render the transit view with slide offset
    lv_transit_frame_t * p_tf = lv_wms_transit_frame_render(draw_buf->buf_act, s_anim_transform_effect, trans_direct, slide_offset);
    // Flush the frame buffer to screen
    drv_adapter_disp_wait_to_flush();
    drv_adapter_disp_set_show_area(0, 0, p_tf->transit_scrn_res_w - 1, p_tf->transit_scrn_res_h - 1);
    drv_adapter_disp_flush((void *) draw_buf->buf_act, GDC_DATA_FORMAT_TSC4, p_tf->transit_scrn_res_w, p_tf->transit_scrn_res_h);

    // Switch the screen flush buffer
    _lv_port_scr_anim_display_mem_switch();

    return;
}

static void _lv_port_scr_anim_transform(void * obj, int32_t v){
    const uint32_t CACHE_BUFF_WIDTH  = lv_wms_transit_get_cache_width();
    lv_transit_direct_e trans_direct = LV_TRANSIT_DIRECT_RIGHT;
    uint32_t range                   = 100;
    uint32_t slide_offset            = 0;

    v = v % 101;
    if((s_anim_progress_start == 0) && (v != 0)) {
        s_anim_progress_start = v;
    }
    range = 100 - s_anim_progress_start;

    switch(s_anim_transform_effect) {
        case LV_TRANS_EFFECT_CUBE:
        {
            trans_direct = LV_TRANSIT_DIRECT_RIGHT;
            if(v < 100)
            {
                slide_offset = ((v - s_anim_progress_start) * CACHE_BUFF_WIDTH) / range;
                _lv_port_scr_anim_frame_update(slide_offset, trans_direct);
            }
        }
        break;

        case LV_TRANS_EFFECT_INNERCUBE:
        {
            trans_direct = LV_TRANSIT_DIRECT_LEFT;
            if(v < 100)
            {
                slide_offset = ((v - s_anim_progress_start) * CACHE_BUFF_WIDTH) / range;
                _lv_port_scr_anim_frame_update(slide_offset, trans_direct);
            }
        }
        break;

        case LV_TRANS_EFFECT_STACK:
        {
            trans_direct = LV_TRANSIT_DIRECT_LEFT;
            if(v < 100)
            {
                slide_offset = ((v - s_anim_progress_start) * CACHE_BUFF_WIDTH) / range;
                _lv_port_scr_anim_frame_update(slide_offset, trans_direct);
            }
        }
        break;

        case LV_TRANS_EFFECT_FADE:
        {
            trans_direct = LV_TRANSIT_DIRECT_LEFT;
            if(v < 100)
            {
                slide_offset = ((v - s_anim_progress_start) * CACHE_BUFF_WIDTH) / range;
                _lv_port_scr_anim_frame_update(slide_offset, trans_direct);
            }
        }
        break;

        case LV_TRANS_EFFECT_FADE_ZOOM:
        {
            trans_direct = LV_TRANSIT_DIRECT_LEFT;
            if(v < 100)
            {
                slide_offset = ((v - s_anim_progress_start) * CACHE_BUFF_WIDTH) / range;
                _lv_port_scr_anim_frame_update(slide_offset, trans_direct);
            }
        }
        break;

        case LV_TRANS_EFFECT_COVER:
        {
            trans_direct = LV_TRANSIT_DIRECT_LEFT;
            if(v < 100)
            {
                slide_offset = ((v - s_anim_progress_start) * CACHE_BUFF_WIDTH) / range;
                _lv_port_scr_anim_frame_update(slide_offset, trans_direct);
            }
        }
        break;

        case LV_TRANS_EFFECT_SPIN_H_R2L:
        case LV_TRANS_EFFECT_SPIN_H_L2R:
        default:
        {
            trans_direct = LV_TRANSIT_DIRECT_RIGHT;
            if(v < 100) {
                slide_offset = ((100 - v) * CACHE_BUFF_WIDTH) / range;

                if((slide_offset >= (CACHE_BUFF_WIDTH/2 - 8)) && (slide_offset <= (CACHE_BUFF_WIDTH/2 + 8))) {  //giveup 90 degree nearby
                    break;
                }
                _lv_port_scr_anim_frame_update(slide_offset, trans_direct);
            }
        }
        break;
    }

    return;
}

static void _lv_port_scr_anim_preload(lv_obj_t* next) {
    /* Preload the next screen for the switch effect composition */
    lv_wms_display_enabled_set(false);
    lv_wms_refresh_enabled_set(true);
    /* Don't call lv_refr_now to avoid animation disruption */
    lv_disp_t * disp  = _lv_refr_get_disp_refreshing();
    lv_obj_t* current = disp->act_scr;
    disp->act_scr = next;
    /* Directly update and refresh to improve the efficiency */
    lv_obj_invalidate(next);
    _lv_disp_refr_timer(disp->refr_timer);
    lv_wms_refresh_enabled_set(false);
    /* Restore the current screen for the event handling continue */
    disp->act_scr = current;

    return;
}


static void _lv_port_scr_anim_cache(void) {
    lv_disp_t *          disp     = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    void* src_addr  = draw_buf->buf_act;
    void* src2_addr = (draw_buf->buf_act == draw_buf->buf2) ? (draw_buf->buf1) : (draw_buf->buf2);
    lv_wms_transit_screen_cache(LV_TRANSIT_CACHE_SCRN_CURRENT, (uint32_t)src_addr);
    lv_wms_transit_screen_cache(LV_TRANSIT_CACHE_SCRN_NEXT,    (uint32_t)src2_addr);

    return;
}


static void _lv_port_scr_anim_display_mem_switch(void){
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);
    if(s_switch_screen_is_cached && (draw_buf->buf1 && draw_buf->buf2)){
        // Switch the screen buffer to LVGL draw buffer
        if(draw_buf->buf_act == draw_buf->buf1)
            draw_buf->buf_act = draw_buf->buf2;
        else
            draw_buf->buf_act = draw_buf->buf1;
    }else{
        while(1); // Only support 2 full screen buffer config shown above
    }

    return;
}

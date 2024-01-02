
/*********************
 *      INCLUDES
 *********************/
#include <stdio.h>
#include "lv_port_disp.h"
#include "lv_conf.h"
#include "hal_gfx_hal.h"
#include "hal_gfx_utils.h"
#include "graphics_sys_defs.h"
#include "app_graphics_mem.h"
#include "app_graphics_ospi.h"
#include "app_graphics_gpu.h"
#include "app_graphics_dc.h"
#include "drv_adapter_port.h"


/*
 * FPS Configuration
 */
#define LV_SHOW_FPS_IN_LABEL                        1
#define LV_SHOW_FPS_IN_UART                         0
#define LV_CALC_AVG_FPS                             1
#define LV_FPS_CACHE_NB                             4


/**********************
 *   LOCAL FUNCTIONS AND VARIABLES
 **********************/
static void         disp_init(void);
static void         disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
static void         drv_disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
static void         rounder_cb(lv_disp_drv_t * disp_drv, lv_area_t * area);
static void         lv_port_disp_update_fps(void);
static void         lv_port_disp_draw_fps(lv_disp_drv_t * disp_drv);

static uint8_t      s_fps_save[LV_FPS_CACHE_NB];
static uint32_t     s_refresh_fps = 30;
static bool         s_debug_info_enable = false;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*-----------------------------
     * Create two fixed frame buffers for drawing and never release it
     *----------------------------*/
    static lv_disp_draw_buf_t draw_buf_dsc;

    lv_color_t* _draw_buf1 = app_graphics_mem_malloc(DISP_HOR_RES * DISP_VER_RES * DISP_PIXEL_DEPTH);
    lv_color_t* _draw_buf2 = app_graphics_mem_malloc(DISP_HOR_RES * DISP_VER_RES * DISP_PIXEL_DEPTH);

    uint32_t fb_on_sram = 0;

    if(OSPI0_XIP_BASE >= (uint32_t)_draw_buf1) {
        fb_on_sram = OSPI0_XIP_BASE - (uint32_t)_draw_buf1;
    }

    printf("%dBytes of FrameBuffer 1 is on SRAM (%.2f%%)\n", fb_on_sram, 100.f * fb_on_sram / (DISP_HOR_RES * DISP_VER_RES * DISP_PIXEL_DEPTH));

    lv_disp_draw_buf_init(&draw_buf_dsc, _draw_buf1, _draw_buf2, DISP_HOR_RES * DISP_VER_RES);

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/
    static lv_disp_drv_t disp_drv;                  /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = DISP_HOR_RES;
    disp_drv.ver_res = DISP_HOR_RES;
    disp_drv.full_refresh = 1;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc;

    disp_drv.rounder_cb = &rounder_cb;

    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    lv_disp_set_theme(disp, lv_theme_simplify_init(disp));
}

extern void     lv_port_set_fb_format(uint32_t format);
extern uint32_t lv_port_get_fb_format(void);

void lv_port_disp_flush_transition_fps(lv_disp_drv_t * disp_drv) {
    if(!s_debug_info_enable) {
        return;
    }

    uint32_t format = lv_port_get_fb_format();
    lv_port_set_fb_format(HAL_GFX_TSC4);
    hal_gfx_cmdlist_t cmd = hal_gfx_cl_le_create();
    hal_gfx_cmdlist_t* cl = &cmd;
    hal_gfx_cl_bind_circular(cl);
    lv_port_disp_draw_fps(disp_drv);
    hal_gfx_cl_le_destroy(cl);
    lv_port_set_fb_format(format);

    return;
}


void lv_port_disp_debug_enable(bool enable) {
    s_debug_info_enable = enable;
}


uint32_t lv_port_get_fb_format(void) {
#if LVGL_FRAMEBUFFER_SIZE == 16
    return HAL_GFX_RGB565;
#elif LVGL_FRAMEBUFFER_SIZE == 32
    return HAL_GFX_RGBA8888;
#else
    #error "Not Support Now"
#endif
}


/**********************
 *   STATIC FUNCTIONS
 **********************/
/* Initialize your display and the required peripherals. */
static void disp_init(void)
{
    drv_adapter_disp_init();
}

/* Flush the content of the internal buffer the specific area on the display
 * You can use DMA or any hardware acceleration to do this operation in the background but
 * 'lv_disp_flush_ready()' has to be called when finished. */
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    drv_disp_flush(disp_drv, area, color_p);

    disp_drv->draw_buf->flushing = 0;
    disp_drv->draw_buf->flushing_last = 0;
}

static void drv_disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    uint32_t data_format ;
    lv_disp_t * disp              = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);

    lv_port_disp_update_fps();
    lv_port_disp_draw_fps(disp_drv);

#if DISP_PIXEL_DEPTH == 4
    data_format = HAL_GDC_RGBA8888;
#elif DISP_PIXEL_DEPTH == 2
    data_format = HAL_GDC_RGB565;
#else
    #error "Not Sopport. Please check DISP_PIXEL_DEPTH=" & DISP_PIXEL_DEPTH
#endif

    drv_adapter_disp_wait_to_flush();
    drv_adapter_disp_set_show_area(area->x1, area->y1, area->x2, area->y2);
    drv_adapter_disp_wait_te();
    drv_adapter_disp_flush((void *) draw_buf->buf_act, data_format, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1);

    return;
}

static void rounder_cb(lv_disp_drv_t * disp_drv, lv_area_t * area)
{
  /* Per RM69330 datasheet, start coord and size must be even*/
    area->x1 = area->x1 & ~1;
    area->y1 = area->y1 & ~1;

    if ((area->x2 - area->x1 + 1) & 1) {
        area->x2 = area->x2 + 1;
    }
    if ((area->y2 - area->y1 + 1) & 1) {
        area->y2 = area->y2 + 1;
    }
}

static void lv_port_disp_update_fps(void) {
    static uint32_t s_last_flush_tick_cnt = 0;
    static uint32_t s_cur_flush_tick_cnt = 0;
    s_cur_flush_tick_cnt = lv_tick_get();
    uint32_t render_diff_ms = s_cur_flush_tick_cnt - s_last_flush_tick_cnt;
    s_last_flush_tick_cnt = s_cur_flush_tick_cnt;
    s_refresh_fps = 1000 / render_diff_ms;

#if LV_CALC_AVG_FPS
    for(uint8_t i=0; i < LV_FPS_CACHE_NB-1; i++){
        s_fps_save[i] = s_fps_save[i+1];
    }
    s_fps_save[LV_FPS_CACHE_NB-1] = s_refresh_fps;
    uint32_t fps_sum = 0;
    for(uint8_t i = 0; i < LV_FPS_CACHE_NB; i++){
        fps_sum += s_fps_save[i];
    }
    #if (4 == LV_FPS_CACHE_NB)
        s_refresh_fps = fps_sum >> 2;
    #else
        s_refresh_fps = fps_sum / LV_FPS_CACHE_NB;
    #endif
#endif
}

static void lv_port_disp_draw_fps(lv_disp_drv_t * disp_drv) {
    if(!s_debug_info_enable) {
        return;
    }

    char info[20] = {0};

#if LV_SHOW_FPS_IN_LABEL
    lv_draw_label_dsc_t draw_label_dsc;
    lv_draw_label_dsc_init(&draw_label_dsc);

    draw_label_dsc.color = lv_color_white();
    draw_label_dsc.font  = &lv_font_montserrat_20;
    draw_label_dsc.align = LV_TEXT_ALIGN_CENTER;
    lv_area_t coords     = {.x1 = 0, .y1=30, .x2=453, .y2=453};
    lv_draw_ctx_t *draw_ctx = disp_drv->draw_ctx;
    draw_ctx->clip_area  = &coords;

    lv_draw_label(draw_ctx, &draw_label_dsc, &coords, info, NULL);
#elif LV_SHOW_FPS_IN_UART
    puts(info);
#endif
}


#include "lv_gpu_gr5526.h"

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

#if LV_USE_GPU_GR552x > 0u

void lv_draw_gr5526_ctx_init(lv_disp_drv_t * drv, lv_draw_ctx_t * draw_ctx)
{
    lv_draw_sw_init_ctx(drv, draw_ctx);
    lv_draw_sw_ctx_t * gdx_draw_ctx         = (lv_draw_sw_ctx_t *)draw_ctx;
    gdx_draw_ctx->base_draw.draw_line       = lv_draw_gr5526_line;
    gdx_draw_ctx->base_draw.draw_img        = lv_draw_gr5526_img;
    gdx_draw_ctx->base_draw.draw_arc        = lv_draw_gr5526_arc;
    gdx_draw_ctx->base_draw.draw_rect       = lv_draw_gr5526_rect;
    gdx_draw_ctx->base_draw.draw_bg         = lv_draw_gr5526_bg;
    gdx_draw_ctx->base_draw.draw_polygon    = lv_draw_gr5526_polygon;
    gdx_draw_ctx->base_draw.draw_letter     = lv_draw_gr5526_letter;
    
    //gdx_draw_ctx->base_draw.draw_img_decoded
    
#if GR552X_GPU_FONT_SUPPORT > 0u
    gdx_draw_ctx->base_draw.draw_label      = lv_draw_gr5526_label;
#endif

#if GR552X_GPU_RENDER_SUPPORT > 0u
    gdx_draw_ctx->base_draw.is_transform_buff = false;
#endif
}

void lv_draw_gr5526_ctx_deinit(lv_disp_drv_t * drv, lv_draw_ctx_t * draw_ctx)
{
    lv_draw_sw_deinit_ctx(drv, draw_ctx);
}

#endif

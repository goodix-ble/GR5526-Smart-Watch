#include <stdio.h>
#include "lv_wms_surface_flinger.h"
#include "drv_adapter_display.h"
#include "app_graphics_mem.h"
// #include "app_graphics_dc.h"


/*
 * STRUCTURE DEFINITIONS
 *****************************************************************************************
 */
typedef struct {
    void *      _scrn_cache_1;
    void *      _scrn_cache_2;

    uint32_t    _framebuffer_width;
    uint32_t    _framebuffer_height;
    uint32_t    _framebuffer_format;

    uint32_t    _cachebuffer_width;
    uint32_t    _cachebuffer_height;
    uint32_t    _cachebuffer_size ;

    bool        _started;
} lv_transit_context_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static lv_transit_context_t _trans_env = {
    ._scrn_cache_1       = NULL,
    ._scrn_cache_2       = NULL,
    ._framebuffer_width  = 0,
    ._framebuffer_height = 0,
    ._framebuffer_format = 0,
    ._cachebuffer_width  = 0,
    ._cachebuffer_height = 0,
    ._cachebuffer_size   = 0,
    ._started           = false,
};

static lv_transit_frame_t s_lv_transit_frame = {
    .transit_scrn_addr = 0,
    .transit_scrn_res_w   = 0,
    .transit_scrn_res_h   = 0,
};

static lv_transit_effect_e s_global_transit_effect = LV_TRANS_EFFECT_CUBE ;
static bool s_refresh_enabled = true;
static bool s_display_enabled = true;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void lv_wms_clear_buffer_with_gpu(uint32_t address, uint32_t buff_format, uint16_t w, uint16_t h)
{
    hal_gfx_cmdlist_t cmd = hal_gfx_cl_le_create();
    hal_gfx_cl_bind_circular(&cmd);
    hal_gfx_set_clip(0, 0, w, h);
    hal_gfx_bind_dst_tex(address, w, h, buff_format, -1);

    hal_gfx_clear(0x00);

    hal_gfx_cl_submit(&cmd);
    hal_gfx_cl_wait(&cmd);
    hal_gfx_cl_le_destroy(&cmd);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
bool lv_wms_transit_starup(lv_transit_config_t * p_config) {
    if(_trans_env._started){
        return true; // do nothing when the module is started
    }

    // Only support TSC4 format for simple(TSC4=(size/4)*4; TSC6A=(size/6)*4)
    _trans_env._cachebuffer_width  = (p_config->scrn_res_w/4)*4;
    _trans_env._cachebuffer_height = (p_config->scrn_res_h/4)*4;
    _trans_env._cachebuffer_size   = hal_gfx_texture_size(HAL_GFX_TSC4, HAL_GFX_TEX_CLAMP, _trans_env._cachebuffer_width, _trans_env._cachebuffer_height);

    lv_wms_transit_mem_alloc();

    _trans_env._framebuffer_width  = p_config->scrn_res_w;
    _trans_env._framebuffer_height = p_config->scrn_res_h;
    _trans_env._framebuffer_format = p_config->fb_format;

    _trans_env._started = true;
    return true;
}

void lv_wms_transit_shutdown(void) {
    lv_wms_transit_mem_free();
    memset(&_trans_env, 0, sizeof(lv_transit_context_t));
}

void lv_wms_transit_mem_alloc(void) {
    if(NULL == _trans_env._scrn_cache_1) {
        _trans_env._scrn_cache_1 = app_graphics_mem_malloc(_trans_env._cachebuffer_size);
    }

    if(NULL == _trans_env._scrn_cache_2) {
        _trans_env._scrn_cache_2 = app_graphics_mem_malloc(_trans_env._cachebuffer_size);
    }
}

void lv_wms_transit_mem_free(void) {
    if(NULL != _trans_env._scrn_cache_1){
        app_graphics_mem_free(_trans_env._scrn_cache_1);
        _trans_env._scrn_cache_1 = NULL;

    }
    if(NULL != _trans_env._scrn_cache_2){
        app_graphics_mem_free(_trans_env._scrn_cache_2);
        _trans_env._scrn_cache_2 = NULL;
    }
}

void lv_wms_transit_screen_cache(lv_transit_scrn_e scrn_no, uint32_t scrn_adress) {

    uint32_t dst_address = (scrn_no == LV_TRANSIT_CACHE_SCRN_CURRENT) ? \
            (uint32_t)_trans_env._scrn_cache_1 : (uint32_t)_trans_env._scrn_cache_2;

    hal_gfx_cmdlist_t cmd = hal_gfx_cl_le_create();
    hal_gfx_cl_bind_circular(&cmd);
    hal_gfx_set_clip(0, 0, _trans_env._cachebuffer_width, _trans_env._cachebuffer_height);
    hal_gfx_bind_dst_tex((uint32_t)dst_address, _trans_env._cachebuffer_width, _trans_env._cachebuffer_height,
                    HAL_GFX_TSC4, -1);
    hal_gfx_clear(0xFF000000);
    hal_gfx_bind_src_tex((uint32_t)scrn_adress, _trans_env._framebuffer_width, _trans_env._framebuffer_height,
                   _trans_env._framebuffer_format, -1, HAL_GFX_FILTER_BL);
    hal_gfx_set_blend_blit(HAL_GFX_BL_SRC);
    hal_gfx_blit(0, 0);

    hal_gfx_cl_submit(&cmd);
    hal_gfx_cl_wait(&cmd);
    hal_gfx_cl_le_destroy(&cmd);
}

/*
 * dest_address : address to store the rendering result for transit
 * trans_type   : transit type
 * trans_direct : transit direction
 * slide_offset : slide offset pixel
 */
lv_transit_frame_t * lv_wms_transit_frame_render(void * dest_address, const lv_transit_effect_e trans_type,
    const lv_transit_direct_e trans_direct, const lv_coord_t slide_offset)
{
    float step             = 0.0f;
    uint16_t ver_res       = _trans_env._framebuffer_height;
    uint16_t hor_res       = _trans_env._framebuffer_width;
    uint32_t _cb_address_1 = (uint32_t)_trans_env._scrn_cache_1;
    uint32_t _cb_address_2 = (uint32_t)_trans_env._scrn_cache_2;
    bool slide_direct_hor  = false;
    hal_gfx_transition_t transit ;

    void * p_dest_buff = dest_address;

    if (trans_direct == LV_TRANSIT_DIRECT_LEFT || trans_direct == LV_TRANSIT_DIRECT_RIGHT){
        slide_direct_hor = true;

        step = 1.0f - (LV_ABS(slide_offset) / (float)hor_res);
        if(trans_direct == LV_TRANSIT_DIRECT_RIGHT){
            step = 1.0f - step;
            _cb_address_1 = (uint32_t)_trans_env._scrn_cache_2;
            _cb_address_2 = (uint32_t)_trans_env._scrn_cache_1;
        }else{
            _cb_address_1 = (uint32_t)_trans_env._scrn_cache_1;
            _cb_address_2 = (uint32_t)_trans_env._scrn_cache_2;
        }

        switch(trans_type){
            case LV_TRANS_EFFECT_LINEAR:
                transit = HAL_GFX_TRANS_LINEAR_H;
                break;
            case LV_TRANS_EFFECT_CUBE:
                transit = HAL_GFX_TRANS_CUBE_H;
                break;
            case LV_TRANS_EFFECT_INNERCUBE:
                transit = HAL_GFX_TRANS_INNERCUBE_H;
                break;
            case LV_TRANS_EFFECT_STACK:
                transit = HAL_GFX_TRANS_STACK_H;
                break;
            case LV_TRANS_EFFECT_FADE:
                transit = HAL_GFX_TRANS_FADE;
                break;
            case LV_TRANS_EFFECT_FADE_ZOOM:
                transit = HAL_GFX_TRANS_FADE_ZOOM;
                break;
            case LV_TRANS_EFFECT_COVER:
                transit = HAL_GFX_TRANS_COVER;
                break;
            case LV_TRANS_EFFECT_SPIN:
                step = 1.f - step;
                transit = HAL_GFX_TRANS_SPIN_H_R2L;
                break;
            case LV_TRANS_EFFECT_PUSHPULL:
                transit = HAL_GFX_TRANS_PUSHPULL_H;
                break;
            default:break;
        }
    }else if(trans_direct == LV_TRANSIT_DIRECT_UP || trans_direct == LV_TRANSIT_DIRECT_DOWN){
        step = 1.0f - (LV_ABS(slide_offset) / (float)ver_res);
        if(trans_direct == LV_TRANSIT_DIRECT_UP){
            step = 1.0f - step;
            _cb_address_1 = (uint32_t)_trans_env._scrn_cache_2;
            _cb_address_2 = (uint32_t)_trans_env._scrn_cache_1;
        }else{
            _cb_address_1 = (uint32_t)_trans_env._scrn_cache_1;
            _cb_address_2 = (uint32_t)_trans_env._scrn_cache_2;
        }

        switch(trans_type){
            case LV_TRANS_EFFECT_LINEAR:
                transit = HAL_GFX_TRANS_LINEAR_V;
                break;
            case LV_TRANS_EFFECT_CUBE:
                transit = HAL_GFX_TRANS_CUBE_V;
                break;
            case LV_TRANS_EFFECT_INNERCUBE:
                transit = HAL_GFX_TRANS_INNERCUBE_V;
                break;
            case LV_TRANS_EFFECT_STACK:
                transit = HAL_GFX_TRANS_STACK_V;
                break;
            case LV_TRANS_EFFECT_FADE:
                transit = HAL_GFX_TRANS_FADE;
                break;
            case LV_TRANS_EFFECT_FADE_ZOOM:
                transit = HAL_GFX_TRANS_FADE_ZOOM;
                break;
            case LV_TRANS_EFFECT_COVER:
                transit = HAL_GFX_TRANS_COVER;
                break;
            case LV_TRANS_EFFECT_PUSHPULL:
                transit = HAL_GFX_TRANS_PUSHPULL_V;
                break;
            default:break;
        }
    }

    hal_gfx_cmdlist_t cmd = hal_gfx_cl_le_create();
    hal_gfx_cl_bind_circular(&cmd);
    if(transit == HAL_GFX_TRANS_COVER){
        uint8_t *p_now_screen_cache_buf  = _trans_env._scrn_cache_1;
        uint8_t *p_next_screen_cache_buf = _trans_env._scrn_cache_2;
        uint16_t _slide_offset           = LV_ABS(slide_offset);
        hal_gfx_set_clip(0, 0, _trans_env._cachebuffer_width, _trans_env._cachebuffer_height);
        hal_gfx_bind_dst_tex((uint32_t)p_dest_buff, _trans_env._cachebuffer_width,
                             _trans_env._cachebuffer_height, HAL_GFX_TSC4, -1);
        hal_gfx_set_blend_blit(HAL_GFX_BL_SIMPLE);
        _slide_offset = (_slide_offset/4)*4;
        hal_gfx_bind_src_tex((uint32_t)p_now_screen_cache_buf,  _trans_env._cachebuffer_width,
                        _trans_env._cachebuffer_height, HAL_GFX_TSC4, -1, HAL_GFX_FILTER_BL);
        hal_gfx_blit(0, 0);
        hal_gfx_bind_src_tex((uint32_t)p_next_screen_cache_buf, _trans_env._cachebuffer_width,
                        _trans_env._cachebuffer_height, HAL_GFX_TSC4, -1, HAL_GFX_FILTER_BL);
        if(slide_direct_hor){
            if(trans_direct == LV_TRANSIT_DIRECT_LEFT){
                hal_gfx_blit(_trans_env._cachebuffer_width - _slide_offset, 0);
            }else{
                hal_gfx_blit(_slide_offset - _trans_env._cachebuffer_width, 0);
            }
        }else{
            if(trans_direct == LV_TRANSIT_DIRECT_UP){
                 hal_gfx_blit(0, _slide_offset - _trans_env._cachebuffer_height);
            }else{
                 hal_gfx_blit(0, _trans_env._cachebuffer_height - _slide_offset);
            }
        }
    } else if (trans_type == LV_TRANS_EFFECT_FADE_ZOOM_ALT) {
        // TODO: Maybe put this into lib?
        hal_gfx_set_clip(0, 0, _trans_env._cachebuffer_width, _trans_env._cachebuffer_height);
        hal_gfx_bind_src_tex(_cb_address_2, _trans_env._cachebuffer_width, _trans_env._cachebuffer_height, HAL_GFX_TSC4, -1, (hal_gfx_tex_mode_t)(HAL_GFX_FILTER_BL | HAL_GFX_TEX_BORDER));
        hal_gfx_bind_dst_tex((uint32_t)p_dest_buff, _trans_env._cachebuffer_width, _trans_env._cachebuffer_height, HAL_GFX_TSC4, -1);

        float width = _trans_env._cachebuffer_width;
        float height = _trans_env._cachebuffer_height;
        float w = width * (1.f - step);
        float h = height * (1.f - step);
        float x = (width - w) / 2;
        float y = (height - h) / 2;
        float a = (1.f - step) * 255.f;
        uint32_t alpha = step < 0.f ? (uint32_t)0U : (step > 1.f ? (uint32_t)255U : (uint32_t)(a));

        hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
        hal_gfx_fill_rect(0, 0, (int)width, (int)height, 0);

        // Src
        hal_gfx_set_const_color(alpha << 24);
        hal_gfx_set_blend(HAL_GFX_BL_SIMPLE | HAL_GFX_BLOP_MODULATE_A, HAL_GFX_TEX0, HAL_GFX_TEX1, HAL_GFX_NOTEX);
        hal_gfx_blit_rect_fit((int)x, (int)y, (int)w, (int)h);

        w = width * (2.f - step);
        h = height * (2.f - step);
        x = (width - w) / 2;
        y = (height - h) / 2;
        alpha = 255 - alpha;

        hal_gfx_bind_src_tex(_cb_address_1, _trans_env._cachebuffer_width, _trans_env._cachebuffer_height, HAL_GFX_TSC4, -1, (hal_gfx_tex_mode_t)(HAL_GFX_FILTER_BL | HAL_GFX_TEX_BORDER));

        // Dst
        hal_gfx_set_const_color(alpha << 24);
        hal_gfx_set_blend(HAL_GFX_BL_SIMPLE | HAL_GFX_BLOP_MODULATE_A, HAL_GFX_TEX0, HAL_GFX_TEX1, HAL_GFX_NOTEX);
        hal_gfx_blit_rect_fit((int)x, (int)y, (int)w, (int)h);
    }else{
        hal_gfx_set_clip(0, 0, _trans_env._cachebuffer_width, _trans_env._cachebuffer_height);
        hal_gfx_bind_src_tex (_cb_address_1, _trans_env._cachebuffer_width, _trans_env._cachebuffer_height,
           HAL_GFX_TSC4, -1, (hal_gfx_tex_mode_t)(HAL_GFX_FILTER_BL | HAL_GFX_TEX_BORDER ));
        hal_gfx_bind_src2_tex(_cb_address_2, _trans_env._cachebuffer_width, _trans_env._cachebuffer_height,
           HAL_GFX_TSC4, -1, (hal_gfx_tex_mode_t)(HAL_GFX_FILTER_BL | HAL_GFX_TEX_BORDER ));
        hal_gfx_set_tex_color(0);
        hal_gfx_bind_dst_tex((uint32_t)p_dest_buff, _trans_env._cachebuffer_width, _trans_env._cachebuffer_height,
           HAL_GFX_TSC4, -1);
        hal_gfx_transition(transit, HAL_GFX_TEX1, HAL_GFX_TEX2, HAL_GFX_BL_SRC, step, _trans_env._cachebuffer_width,
           _trans_env._cachebuffer_height);
    }
    hal_gfx_cl_submit(&cmd);
    hal_gfx_cl_wait(&cmd);
    hal_gfx_cl_le_destroy(&cmd);
    s_lv_transit_frame.transit_scrn_addr = p_dest_buff;
    s_lv_transit_frame.transit_scrn_res_w   = _trans_env._cachebuffer_width;
    s_lv_transit_frame.transit_scrn_res_h   = _trans_env._cachebuffer_height;
    return &s_lv_transit_frame;
}

void lv_wms_transit_effect_set(lv_transit_effect_e e) {
    s_global_transit_effect = e;
}

lv_transit_effect_e lv_wms_transit_effect_get(void) {
    return s_global_transit_effect;
}

uint32_t lv_wms_transit_get_cache_width(void) {
    return _trans_env._cachebuffer_width ;
}

void lv_wms_refresh_enabled_set(bool enabled)
{
    s_refresh_enabled = enabled;
}

void lv_wms_display_enabled_set(bool enabled)
{
    s_display_enabled = enabled;
}

bool lv_wms_is_refresh_enabled(void)
{
    return s_refresh_enabled;
}

bool lv_wms_is_display_enabled(void)
{
    return s_display_enabled;
}

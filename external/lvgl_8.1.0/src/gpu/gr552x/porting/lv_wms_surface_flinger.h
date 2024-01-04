#ifndef __LV_WMS_SURFACE_FLINGER_H__
#define __LV_WMS_SURFACE_FLINGER_H__

#include "lvgl.h"
#include "hal_gfx_core.h"
#include "hal_gfx_utils.h"
#include "hal_gfx_font.h"
#include "hal_gfx_graphics.h"
#include "hal_gfx_transitions.h"

/*
 * TYPE DECLARATIONS
 *****************************************************************************************
 */
typedef enum
{
    LV_TRANSIT_DIRECT_NONE  = LV_DIR_NONE,
    LV_TRANSIT_DIRECT_LEFT  = LV_DIR_LEFT,
    LV_TRANSIT_DIRECT_RIGHT = LV_DIR_RIGHT,
    LV_TRANSIT_DIRECT_UP    = LV_DIR_TOP,
    LV_TRANSIT_DIRECT_DOWN  = LV_DIR_BOTTOM,
} lv_transit_direct_e;

typedef enum {
    LV_TRANSIT_CACHE_SCRN_CURRENT = 0,   /* cache as current screen */
    LV_TRANSIT_CACHE_SCRN_NEXT,          /* cache as next screen */
} lv_transit_scrn_e;

typedef enum {
    LV_TRANS_EFFECT_LINEAR = 0x00,      /**< Linear transit. */
    LV_TRANS_EFFECT_CUBE,               /**< Cubic transit . */
    LV_TRANS_EFFECT_INNERCUBE,          /**< Inner Cube transit . */
    LV_TRANS_EFFECT_STACK,              /**< Stack transit . */
    LV_TRANS_EFFECT_FADE,               /**< Fade transit. */
    LV_TRANS_EFFECT_FADE_ZOOM,          /**< Fade-zoom transit. */
    LV_TRANS_EFFECT_COVER,              /**< Cover transit. */
    LV_TRANS_EFFECT_MAX,
} lv_transit_effect_e;


typedef struct {
    uint32_t scrn_res_w;
    uint32_t scrn_res_h;
    uint32_t fb_format;
} lv_transit_config_t;

typedef struct {
    void *      transit_scrn_addr;
    uint32_t    transit_scrn_res_w;
    uint32_t    transit_scrn_res_h;
} lv_transit_frame_t;

/*
 * FUNCTION DECLARATIONS
 *****************************************************************************************
 */

/**
 * Startup the transit module.
 * @param[in] p_config : the screen t
 */
bool lv_wms_transit_starup(lv_transit_config_t * p_config);

/**
 * Shutdown the transit module.
 */
void lv_wms_transit_shutdown(void);

/**
 * Cache the transit screen.
 * @param[in] scrn_no   : select to current or the next screen to cache
 * @param[in] scrn_adress : the screen buffer address
 */
void lv_wms_transit_screen_cache(lv_transit_scrn_e scrn_no, uint32_t scrn_adress);

/*
 * Output the transit frame buffer.
 * @param[in] dest_address : address to store the rendering result for transit
 * @param[in] trans_type   : transit type
 * @param[in] trans_direct : transit direction
 * @param[in] slide_offset : slide offset pixel
 *
 * @return the transit buffer address and size info.
 */
lv_transit_frame_t * lv_wms_transit_frame_render(void * dest_address, const lv_transit_effect_e trans_type,
                                const lv_transit_direct_e trans_direct, const lv_coord_t slide_offset);

/**
 * Set the transit effect.
 * @param[in] the transit effect.
 */
void lv_wms_transit_effect_set(lv_transit_effect_e e);

/**
 * Get the transit effect.
 *
 * @return the transit effect
 */
lv_transit_effect_e lv_wms_transit_effect_get(void);

/**
 * Prepare the cache memory for the transit
 */
void lv_wms_transit_mem_alloc(void);

/**
 * Release the cache memory for the transit
 */
void lv_wms_transit_mem_free(void);

#endif /* __LV_WMS_SURFACE_FLINGER_H__ */

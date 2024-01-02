
/** @addtogroup GRAPHICS_SDK Graphics
 *  @{
 */

/** @defgroup HAL_GFX_SHADERSPECIFIC Hal gfx shaderspecific TODO
 * @brief TODO.
 * @{
 */

#ifndef HAL_GFX_SHADERSPECIFIC_H__
#define HAL_GFX_SHADERSPECIFIC_H__

#include "hal_gfx_sys_defs.h"
#include "hal_gfx_hal.h"
#include "hal_gfx_programHW.h"
#include "hal_gfx_graphics.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GFX_SHADERSPECIFIC_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] src_addr_Y: TODO
 * @param[in] src_addr_U: TODO
 * @param[in] src_addr_V: TODO
 * @param[in] src_xres:   TODO
 * @param[in] src_yres:   TODO
 * @param[in] src_stride: TODO
 * @param[in] dst_x:      TODO
 * @param[in] dst_y:      TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_yuv( uintptr_t src_addr_Y,
                    uintptr_t src_addr_U,
                    uintptr_t src_addr_V,
                    uint32_t src_xres,
                    uint32_t src_yres,
                    int src_stride,
                    int dst_x,
                    int dst_y);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] src_addr_Y: TODO
 * @param[in] src_addr_U: TODO
 * @param[in] src_addr_V: TODO
 * @param[in] src_xres:   TODO
 * @param[in] src_yres:   TODO
 * @param[in] src_stride: TODO
 * @param[in] dst_x:      TODO
 * @param[in] dst_y:      TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_yuv10( uintptr_t src_addr_Y,
                    uintptr_t src_addr_U,
                    uintptr_t src_addr_V,
                    uint32_t src_xres,
                    uint32_t src_yres,
                    int src_stride,
                    int dst_x,
                    int dst_y);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] warpBase:   TODO
 * @param[in] warpW:      TODO
 * @param[in] warpH:      TODO
 * @param[in] warpMode:   TODO
 * @param[in] warpStride: TODO
 * @param[in] x:          TODO
 * @param[in] y:          TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_warp(uintptr_t warpBase, uint32_t warpW, uint32_t warpH, hal_gfx_tex_format_t warpMode,
                    int warpStride, int x, int y);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] matrix: TODO
 * @param[in] x:      TODO
 * @param[in] y:      TODO
 * @param[in] w:      TODO
 * @param[in] h:      TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_blur(unsigned char matrix[3][3], int x, int y, int w, int h);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] matrix: TODO
 * @param[in] x:      TODO
 * @param[in] y:      TODO
 * @param[in] w:      TODO
 * @param[in] h:      TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_edge(unsigned char matrix[3][3], int x, int y, int w, int h);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] x: TODO
 * @param[in] y: TODO
 * @param[in] w: TODO
 * @param[in] h: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_mean(unsigned char matrix[2][3], int x, int y, int w, int h);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] x: TODO
 * @param[in] y: TODO
 * @param[in] w: TODO
 * @param[in] h: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_gauss(unsigned char matrix[3][3], int x, int y, int w, int h);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] x:       TODO
 * @param[in] y:       TODO
 * @param[in] w:       TODO
 * @param[in] h:       TODO
 * @param[in] sharpen: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_sharpen_gauss(int x, int y, int w, int h, float sharpen);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] x:       TODO
 * @param[in] y:       TODO
 * @param[in] w:       TODO
 * @param[in] h:       TODO
 * @param[in] sharpen: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_sharpen_laplace(int x, int y, int w, int h, float sharpen);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] x:   TODO
 * @param[in] y:   TODO
 * @param[in] w:   TODO
 * @param[in] h:   TODO
 * @param[in] min: TODO
 * @param[in] max: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_contrast_linear(int x, int y, int w, int h, uint8_t min, uint8_t max);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] matrix: TODO
 * @param[in] x:      TODO
 * @param[in] y:      TODO
 * @param[in] w:      TODO
 * @param[in] h:      TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_color_correction(uint8_t matrix[3][3], int x, int y, int w, int h);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] w: TODO
 * @param[in] h: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_rgb_to_ycbcr(int w, int h);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] w: TODO
 * @param[in] h: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_median(int w, int h);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] w:         TODO
 * @param[in] h:         TODO
 * @param[in] histogram: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_hist_equalization(int w, int h, uint32_t histogram[256]);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] w:  TODO
 * @param[in] h:  TODO
 * @param[in] bo: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_gamma(int w, int h,hal_gfx_buffer_t *bo);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] x:         TODO
 * @param[in] y:         TODO
 * @param[in] w:         TODO
 * @param[in] h:         TODO
 * @param[in] threshold: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_binary(int x, int y, int w, int h, float threshold);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] w: TODO
 * @param[in] h: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_debayer(int w, int h);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] w: TODO
 * @param[in] h: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_ycbcr_to_rgb(int w, int h);


/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] w: TODO
 * @param[in] h: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_bayer_L8_to_RGB(int w, int h);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] w: TODO
 * @param[in] h: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_3L8_to_RGB(int w, int h);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] w: TODO
 * @param[in] h: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_blit_RGB_to_3L8(int w, int h);

/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */


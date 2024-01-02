/**
 ****************************************************************************************
 *
 * @file    hal_gdc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of Graphics library.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************************
 */

/** @addtogroup GRAPHICS_SDK Graphics
 *  @{
 */

/** @addtogroup HAL_DC HAL DC
  * @{
  */

/** @defgroup HAL_GDC GDC
  * @brief DC HAL module driver.
  * @{
  */

#ifndef HAL_GDC_H__
#define HAL_GDC_H__

#include "hal_gfx_sys_defs.h"
#include "hal_gdc_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @addtogroup HAL_GDC_MACRO Defines
  * @{
  */
/** @defgroup HAL_GDC_REG_CFG The DC register configure defines
  * @{
  */
#define HAL_GDC_CFG_LAYER_EXISTS(i)   (1U << (8 + (i)*4))       /**< Configure layer-existence for specifical DC layer  */
#define HAL_GDC_CFG_LAYER_BLENDER(i)  (1U << (8 + (i)*4 + 1))   /**< Configure blender for specifical DC layer */
#define HAL_GDC_CFG_LAYER_SCALER(i)   (1U << (8 + (i)*4 + 2))   /**< Configure scaler for specifical DC layer  */
#define HAL_GDC_CFG_LAYER_GAMMA(i)    (1U << (8 + (i)*4 + 3))   /**< Configure gamma for specifical DC layer  */

#define HAL_GDC_LAYER_ENABLE          (1U << 31)                 /**< Enable Layer */
#define HAL_GDC_ENABLE                (1U << 31)                 /**< Enable DC */
#define HAL_GDC_CFG_L3_YUVMEM         (1U << 31)                 /**< Cfg L3 YUV */
#define HAL_GDC_EN_L3PIX              (1U << 31)                 /**< Ignore */
#define DC_STATUS_rsrvd_0             (1U << 31)                 /**< Resrved */
#define hal_gdc_clkctrl_cg_l3_bus_clk (1U << 31)                 /**< Ignore */
/** @} */

/** @} */

/** @addtogroup HAL_GDC_ENUM Enumerations
  * @{
  */
/**
  * @brief Layer control definition
  */
typedef enum
{
    HAL_GDC_LAYER_DISABLE     = 0,          /**< Disable Layer */
    HAL_GDC_FORCE_A           = 1U << 30,   /**< Force Alpha */
    HAL_GDC_SCALE_NN          = 1U << 29,   /**< Activate Bilinear Filter */
    HAL_GDC_MODULATE_A        = 1U << 28,   /**< Modulate Alpha */
    HAL_GDC_LAYER_AHBLOCK     = 1U << 27,   /**< Activate HLOCK signal on AHB DMAs */
    HAL_GDC_LAYER_GAMMALUT_EN = 1U << 26,    /**< Enable Gamma Look Up Table */
} hal_gdc_layer_ctrl_t;

/**
  * @brief Layer blending factor definition
  */
typedef enum
{
    HAL_GDC_BF_ZERO           = 0x0,        /**< Black */
    HAL_GDC_BF_ONE            = 0x1,        /**< White */
    HAL_GDC_BF_SRCALPHA       = 0x2,        /**< Alpha Source */
    HAL_GDC_BF_GLBALPHA       = 0x3,        /**< Alpha Global */
    HAL_GDC_BF_SRCGBLALPHA    = 0x4,        /**< Alpha Source And Alpha Global */
    HAL_GDC_BF_INVSRCALPHA    = 0x5,        /**< Inverted Source */
    HAL_GDC_BF_INVGBLALPHA    = 0x6,        /**< Inverted Global */
    HAL_GDC_BF_INVSRCGBLALPHA = 0x7,        /**< Inverted Source And Global */
    HAL_GDC_BF_DSTALPHA       = 0xa,        /**< Alpha Destination */
    HAL_GDC_BF_INVDSTALPHA    = 0xb,         /**< Inverted Destination */
} hal_gdc_blend_factors_t;

/**
  * @brief Layer blending mode definition
  */
typedef enum
{
    HAL_GDC_BL_SIMPLE     = (HAL_GDC_BF_SRCALPHA     | (HAL_GDC_BF_INVSRCALPHA  <<4)),   /**< Sa * Sa + Da * (1 - Sa) */
    HAL_GDC_BL_CLEAR      = (HAL_GDC_BF_ZERO         | (HAL_GDC_BF_ZERO         <<4)),   /**< 0 */
    HAL_GDC_BL_SRC        = (HAL_GDC_BF_ONE          | (HAL_GDC_BF_ZERO         <<4)),   /**< Sa */
    HAL_GDC_BL_SRC_OVER   = (HAL_GDC_BF_ONE          | (HAL_GDC_BF_INVSRCALPHA  <<4)),   /**< Sa + Da * (1 - Sa) */
    HAL_GDC_BL_DST_OVER   = (HAL_GDC_BF_INVDSTALPHA  | (HAL_GDC_BF_ONE          <<4)),   /**< Sa * (1 - Da) + Da */
    HAL_GDC_BL_SRC_IN     = (HAL_GDC_BF_DSTALPHA     | (HAL_GDC_BF_ZERO         <<4)),   /**< Sa * Da */
    HAL_GDC_BL_DST_IN     = (HAL_GDC_BF_ZERO         | (HAL_GDC_BF_SRCALPHA     <<4)),   /**< Da * Sa */
    HAL_GDC_BL_SRC_OUT    = (HAL_GDC_BF_INVDSTALPHA  | (HAL_GDC_BF_ZERO         <<4)),   /**< Sa * (1 - Da) */
    HAL_GDC_BL_DST_OUT    = (HAL_GDC_BF_ZERO         | (HAL_GDC_BF_INVSRCALPHA  <<4)),   /**< Da * (1 - Sa) */
    HAL_GDC_BL_SRC_ATOP   = (HAL_GDC_BF_DSTALPHA     | (HAL_GDC_BF_INVSRCALPHA  <<4)),   /**< Sa * Da + Da * (1 - Sa) */
    HAL_GDC_BL_DST_ATOP   = (HAL_GDC_BF_INVDSTALPHA  | (HAL_GDC_BF_SRCALPHA     <<4)),   /**< Sa * (1 - Da) + Da * Sa */
    HAL_GDC_BL_ADD        = (HAL_GDC_BF_ONE          | (HAL_GDC_BF_ONE          <<4)),   /**< Sa + Da */
    HAL_GDC_BL_XOR        = (HAL_GDC_BF_INVDSTALPHA  | (HAL_GDC_BF_INVSRCALPHA  <<4)),    /**< Sa * (1 - Da) + Da * (1 - Sa) */
} hal_gdc_blend_mode_t;

/**
  * @brief Layer color mode definition
  */
typedef enum
{
    HAL_GDC_RGBA5551  = 0x01,  /**< RGBA5551 */
    HAL_GDC_ABGR8888  = 0x02,  /**< ABGR8888 */
    HAL_GDC_RGB332    = 0x04,  /**< RGB332 */
    HAL_GDC_RGB565    = 0x05,  /**< RGB565 */
    HAL_GDC_BGRA8888  = 0x06,  /**< BGRA8888 */
    HAL_GDC_L8        = 0x07,  /**< L8 */
    HAL_GDC_L1        = 0x08,  /**< L1 */
    HAL_GDC_L4        = 0x09,  /**< L4 */
    HAL_GDC_YUYV      = 0x0a,  /**< YUYV */
    HAL_GDC_RGB24     = 0x0b,  /**< RGB24 */
    HAL_GDC_YUY2      = 0x0c,  /**< YUY2 */
    HAL_GDC_RGBA8888  = 0x0d,  /**< RGBA8888 */
    HAL_GDC_ARGB8888  = 0x0e,  /**< ARGB8888 */
    HAL_GDC_V_YUV420  = 0x10,  /**< V_YUV420 */
    HAL_GDC_TLYUV420  = 0x11,  /**< TLYUV420 */
    HAL_GDC_TSC4      = 0x12,  /**< TSC4 */
    HAL_GDC_TSC6      = 0x13,  /**< TSC6 */
    HAL_GDC_TSC6A     = 0x14,  /**< TSC6A */
    HAL_GDC_RGBA4444  = 0x15,  /**< RGBA4444 */
    HAL_GDC_ARGB4444  = 0x18,   /**< ARGB4444 */
} hal_gdc_format_t;

/**
  * @brief Layer video mode definition
  */
typedef enum
{
    HAL_GDC_DISABLE    =  0,         /**< DISABLE */
    HAL_GDC_CURSOR     =  1U << 30,  /**< CURSOR */
    HAL_GDC_NEG_V      =  1U << 28,  /**< NEG_V */
    HAL_GDC_NEG_H      =  1U << 27,  /**< NEG_H */
    HAL_GDC_NEG_DE     =  1U << 26,  /**< NEG_DE */
    HAL_GDC_DITHER     =  1U << 24,  /**< DITHER 18-bit */
    HAL_GDC_DITHER16   =  2U << 24,  /**< DITHER 16-bit */
    HAL_GDC_DITHER15   =  3U << 24,  /**< DITHER 15-bit */
    HAL_GDC_SINGLEV    =  1U << 23,  /**< SINGLEV */
    HAL_GDC_INVPIXCLK  =  1U << 22,  /**< INVPIXCLK */
    HAL_GDC_PALETTE    =  1U << 20,  /**< PALETTE */
    HAL_GDC_GAMMA      =  1U << 20,  /**< GAMMA */
    HAL_GDC_BLANK      =  1U << 19,  /**< BLANK */
    HAL_GDC_INTERLACE  =  1U << 18,  /**< INTERLACE */
    HAL_GDC_ONE_FRAME  =  1U << 17,  /**< ONE_FRAME */
    HAL_GDC_P_RGB3_18B =  1U << 12,  /**< P_RGB3 */
    HAL_GDC_P_RGB3_18B1=  2U << 12,  /**< P_RGB3 */
    HAL_GDC_P_RGB3_16B =  3U << 12,  /**< P_RGB3 */
    HAL_GDC_P_RGB3_16B1=  4U << 12,  /**< P_RGB3 */
    HAL_GDC_P_RGB3_16B2=  5U << 12,  /**< P_RGB3 */
    HAL_GDC_CLKOUTDIV  =  1U << 11,  /**< CLKOUTDIV */
    HAL_GDC_LVDSPADS   =  1U << 10,  /**< LVDSPADS */
    HAL_GDC_YUVOUT     =  1U << 9,   /**< YUVOUT */
    HAL_GDC_MIPI_OFF   =  1U << 4,   /**< MIPI_OFF */
    HAL_GDC_OUTP_OFF   =  1U << 3,   /**< OUTP_OFF */
    HAL_GDC_LVDS_OFF   =  1U << 2,   /**< LVDS_OFF */
    HAL_GDC_SCANDOUBLE =  1U << 1,   /**< SCANDOUBLE */
    HAL_GDC_TESTMODE   =  1U << 0,   /**< TESTMODE */
    HAL_GDC_P_RGB3     =  0U << 5,   /**< P_RGB3 */
    HAL_GDC_S_RGBX4    =  1U << 5,   /**< S_RGBX4 */
    HAL_GDC_S_RGB3     =  2U << 5,   /**< S_RGB3 */
    HAL_GDC_S_12BIT    =  3U << 5,   /**< S_12BIT */
    HAL_GDC_LVDS_ISP68 =  4U << 5,   /**< LVDS_ISP68 */
    HAL_GDC_LVDS_ISP8  =  5U << 5,   /**< LVDS_ISP8 */
    HAL_GDC_T_16BIT    =  6U << 5,   /**< T_16BIT */
    HAL_GDC_BT656      =  7U << 5,   /**< BT656 */
    HAL_GDC_JDIMIP     =  8U << 5,   /**< JDIMIP */
    HAL_GDC_LUT8       =  1U << 20   /**< LUT8 */
} hal_gdc_videomode_t;

/**
  * @brief Layer configuration definition
  */
typedef enum
{
    HAL_GDC_CFG_PALETTE      = 1U <<  0,  /**< Global Gamma enabled */
    HAL_GDC_CFG_FIXED_CURSOR = 1U <<  1,  /**< Fixed Cursor enabled */
    HAL_GDC_CFG_PROGR_CURSOR = 1U <<  2,  /**< Programmable Cursor enabled */
    HAL_GDC_CFG_DITHERING    = 1U <<  3,  /**< Dithering enabled */
    HAL_GDC_CFG_FORMAT       = 1U <<  4,  /**< Formatting enabled */
    HAL_GDC_CFG_HiQ_YUV      = 1U <<  5,  /**< High Quality YUV converted enabled */
    HAL_GDC_CFG_DBIB         = 1U <<  6,  /**< DBI Type-B interface enabled */
    HAL_GDC_CFG_YUVOUT       = 1U <<  7,  /**< RGB to YUV converted */
    HAL_GDC_CFG_L0_ENABLED   = 1U <<  8,  /**< Layer 0 enabled */
    HAL_GDC_CFG_L0_BLENDER   = 1U <<  9,  /**< Layer 0 has blender */
    HAL_GDC_CFG_L0_SCALER    = 1U << 10,  /**< Layer 0 has scaler */
    HAL_GDC_CFG_L0_GAMMA     = 1U << 11,  /**< Layer 0 has gamma LUT */
    HAL_GDC_CFG_L1_ENABLED   = 1U << 12,  /**< Layer 1 enabled */
    HAL_GDC_CFG_L1_BLENDER   = 1U << 13,  /**< Layer 1 has blender */
    HAL_GDC_CFG_L1_SCALER    = 1U << 14,  /**< Layer 1 has scaler */
    HAL_GDC_CFG_L1_GAMMA     = 1U << 15,  /**< Layer 1 has gamma LUT */
    HAL_GDC_CFG_L2_ENABLED   = 1U << 16,  /**< Layer 2 enabled */
    HAL_GDC_CFG_L2_BLENDER   = 1U << 17,  /**< Layer 2 has blender */
    HAL_GDC_CFG_L2_SCALER    = 1U << 18,  /**< Layer 2 has scaler */
    HAL_GDC_CFG_L2_GAMMA     = 1U << 19,  /**< Layer 2 has gamma LUT */
    HAL_GDC_CFG_L3_ENABLED   = 1U << 20,  /**< Layer 3 enabled */
    HAL_GDC_CFG_L3_BLENDER   = 1U << 21,  /**< Layer 3 has blender */
    HAL_GDC_CFG_L3_SCALER    = 1U << 22,  /**< Layer 3 has scaler */
    HAL_GDC_CFG_L3_GAMMA     = 1U << 23,  /**< Layer 3 has gamma LUT */
    HAL_GDC_CFG_SPI          = 1U << 24,  /**< SPI interface is enabled */
    HAL_GDC_CFG_L0_YUVMEM    = 1U << 28,  /**< layer 0 has YUV Memory */
    HAL_GDC_CFG_L1_YUVMEM    = 1U << 29,  /**< layer 1 has YUV Memory */
    HAL_GDC_CFG_L2_YUVMEM    = 1U << 30,   /**< layer 2 has YUV Memory */
} hal_gdc_config_t;

/**
  * @brief DC status definition
  */
typedef enum
{
    DC_STATUS_rsrvd_1            = (1U<<30),    /**< Reserved bit */
    DC_STATUS_rsrvd_2            = (1U<<29),    /**< Reserved bit */
    DC_STATUS_rsrvd_3            = (1U<<28),    /**< Reserved bit */
    DC_STATUS_rsrvd_4            = (1U<<27),    /**< Reserved bit */
    DC_STATUS_rsrvd_5            = (1U<<26),    /**< Reserved bit */
    DC_STATUS_rsrvd_6            = (1U<<25),    /**< Reserved bit */
    DC_STATUS_rsrvd_7            = (1U<<24),    /**< Reserved bit */
    DC_STATUS_rsrvd_8            = (1U<<23),    /**< Reserved bit */
    DC_STATUS_rsrvd_9            = (1U<<22),    /**< Reserved bit */
    DC_STATUS_rsrvd_10           = (1U<<21),    /**< Reserved bit */
    DC_STATUS_rsrvd_11           = (1U<<20),    /**< Reserved bit */
    DC_STATUS_rsrvd_12           = (1U<<19),    /**< Reserved bit */
    DC_STATUS_rsrvd_13           = (1U<<18),    /**< Reserved bit */
    DC_STATUS_rsrvd_14           = (1U<<17),    /**< Reserved bit */
    DC_STATUS_rsrvd_15           = (1U<<16),    /**< Reserved bit */
    DC_STATUS_dbi_cmd_ready      = (1U<<15),    /**< DBI i/f fifo full */
    DC_STATUS_dbi_cs             = (1U<<14),    /**< DBI/SPI i/f active transaction */
    DC_STATUS_frame_end          = (1U<<13),    /**< End of frame pulse */
    DC_STATUS_dbi_pending_trans  = (1U<<12),    /**< pending command/data transaction */
    DC_STATUS_dbi_pending_cmd    = (1U<<11),    /**< pending command */
    DC_STATUS_dbi_pending_data   = (1U<<10),    /**< pending pixel data */
    DC_STATUS_dbi_busy           =((1U<<16)|(1U<<14)|(1U<<13)|(1U<<12)|(1U<<11)|(1U<<10)),  /**< DBI i/f busy */
    DC_STATUS_mmu_error          = (1U<< 9),    /**< not implemented */
    DC_STATUS_te                 = (1U<< 8),    /**< tearing */
    DC_STATUS_sticky             = (1U<< 7),    /**< underflow flag */
    DC_STATUS_underflow          = (1U<< 6),    /**< underflow signal */
    DC_STATUS_LASTROW            = (1U<< 5),    /**< last scan-row */
    DC_STATUS_DPI_Csync          = (1U<< 4),    /**< DPI C-sync */
    DC_STATUS_vsync_te           = (1U<< 3),    /**< Vsync or Tearing */
    DC_STATUS_hsync              = (1U<< 2),    /**< Hsync */
    DC_STATUS_framegen_busy      = (1U<< 1),    /**< Frame-generation in-progress */
    DC_STATUS_ACTIVE             = (1U<< 0),    /**< active */
} hal_gdc_status_t;

/**
  * @brief DC clock control definition
  */
typedef enum
{
    HAL_GDC_EN_PIXCLK          = (1U<<22),      /**< Resolution X */
    HAL_GDC_EN_CFCLK           = (1U<<23),      /**< RegFile clock-gaters bypass */
    HAL_GDC_EN_L0BUS           = (1U<<24),      /**< layer 0 bus clock clock-gater bypass */
    HAL_GDC_EN_L0PIX           = (1U<<25),      /**< layer 0 pixel clock clock-gater bypass */
    HAL_GDC_EN_L1BUS           = (1U<<26),      /**< layer 1 bus clock clock-gater bypass */
    HAL_GDC_EN_L1PIX           = (1U<<27),      /**< layer 1 pixel clock clock-gater bypass */
    HAL_GDC_EN_L2BUS           = (1U<<28),      /**< layer 2 bus clock clock-gater bypass */
    HAL_GDC_EN_L2PIX           = (1U<<29),      /**< layer 2 pixel clock clock-gater bypass */
    HAL_GDC_EN_L3BUS           = (1U<<30),      /**< layer 3 bus clock clock-gater bypass */
} hal_gdc_clkctrl_t;

/**
  * @brief DC clock cg control definition
  */
typedef enum
{
    hal_gdc_clkctrl_cg_l3_pix_clk =  (1U<<30),  /**< layer 3 bus clock clock-gater bypass */
    hal_gdc_clkctrl_cg_l2_bus_clk =  (1U<<29),  /**< layer 2 bus clock clock-gater bypass */
    hal_gdc_clkctrl_cg_l2_pix_clk =  (1U<<28),  /**< layer 2 pixel clock clock-gater bypass */
    hal_gdc_clkctrl_cg_l1_bus_clk =  (1U<<27),  /**< layer 1 bus clock clock-gater bypass */
    hal_gdc_clkctrl_cg_l1_pix_clk =  (1U<<26),  /**< layer 1 pixel clock clock-gater bypass */
    hal_gdc_clkctrl_cg_l0_bus_clk =  (1U<<25),  /**< layer 0 bus clock clock-gater bypass */
    hal_gdc_clkctrl_cg_l0_pix_clk =  (1U<<24),  /**< layer 0 pixel clock clock-gater bypass */
    hal_gdc_clkctrl_cg_regfil_clk =  (1U<<23),  /**< RegFile clock-gaters bypass */
    hal_gdc_clkctrl_cg_bypass_clk =  (1U<<22),  /**< Clock-gaters bypass */
    hal_gdc_clkctrl_cg_rsrvd_21   =  (1U<<21),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_20   =  (1U<<20),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_19   =  (1U<<19),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_18   =  (1U<<18),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_17   =  (1U<<17),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_16   =  (1U<<16),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_15   =  (1U<<15),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_14   =  (1U<<14),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_13   =  (1U<<13),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_12   =  (1U<<12),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_11   =  (1U<<11),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_10   =  (1U<<10),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_9    =  (1U<< 9),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_8    =  (1U<< 8),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_7    =  (1U<< 7),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_6    =  (1U<< 6),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_5    =  (1U<< 5),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_4    =  (1U<< 4),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_rsrvd_3    =  (1U<< 3),  /**< Reserved bit */
    hal_gdc_clkctrl_cg_clk_swap   =  (1U<< 2),  /**< Pixel generation and format clock swap */
    hal_gdc_clkctrl_cg_clk_inv    =  (1U<< 1),  /**< Invert (ouput) clock polarity */
    hal_gdc_clkctrl_cg_clk_en     =  (1U<< 0),   /**< Enable clock divider */

} hal_gdc_clkctrl_cg_t;

/** @} */

/** @addtogroup HAL_DC_STRUCTURES Structures
  * @{
  */
/**
  * @brief Display parameters definition
  */
typedef struct __hal_gdc_display_t
{
    uint32_t resx ;  /**< Resolution X */
    uint32_t resy ;  /**< Resolution Y */
    uint32_t fpx  ;  /**< Front Porch X */
    uint32_t fpy  ;  /**< Front Porch Y */
    uint32_t bpx  ;  /**< Back Porch X */
    uint32_t bpy  ;  /**< Back Porch Y */
    uint32_t blx  ;  /**< Blanking X */
    uint32_t bly  ;  /**< Blanking Y */
} hal_gdc_display_t;

/**
  * @brief Layer parameters definition
  */
typedef struct __hal_gdc_layer_t
{
    void            *baseaddr_virt ; /**< Virtual Address */
    uintptr_t        baseaddr_phys ; /**< Physical Address */
    uint32_t         resx          ; /**< Resolution X */
    uint32_t         resy          ; /**< Resolution Y */
    int32_t          stride        ; /**< Stride */
    int32_t          startx        ; /**< Start X */
    int32_t          starty        ; /**< Start Y */
    uint32_t         sizex         ; /**< Size X */
    uint32_t         sizey         ; /**< Size Y */
    uint8_t          alpha         ; /**< Alpha */
    uint8_t          blendmode     ; /**< Blending Mode */
    uint8_t          buscfg        ; /**< bugcfg */
    hal_gdc_format_t  format       ; /**< Format */
    uint32_t         mode          ; /**< Mode */
    uint32_t         u_base        ; /**< U Base */
    uint32_t         v_base        ; /**< Y Base */
    uint32_t         u_stride      ; /**< U Stride */
    uint32_t         v_stride      ; /**< V Stride */
} hal_gdc_layer_t;

/** @} */

/**
 * @addtogroup HAL_GDC_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize hal_gdc library.
 *
 * @return -1 on error
 *****************************************************************************************
 */
int hal_gdc_init(void);

/**
 *****************************************************************************************
 * @brief Read Configuration Register.
 *
 * @return Configuration Register Value
 *****************************************************************************************
 */
uint32_t hal_gdc_get_config(void);

/**
 *****************************************************************************************
 * @brief Read CRC Checksum Register.
 *
 * @return CRC checksum value of last frame. For testing purposes
 *****************************************************************************************
 */
uint32_t hal_gdc_get_crc(void);

/**
 *****************************************************************************************
 * @brief Set hal_gdc Background Color.
 *
 * @param[in] rgba: a 32-bit rgba value (0xRRGGBBXX - Red: color[31:24], Green: color[23:16], Blue: color[15:8])
 *****************************************************************************************
 */
void hal_gdc_set_bgcolor(uint32_t rgba);

/**
 *****************************************************************************************
 * @brief Set Display timing parameters.
 *
 * @param[in] resx: Resolution X
 * @param[in] fpx: Front Porch X
 * @param[in] blx: Blanking X
 * @param[in] bpx: Back Porch X
 * @param[in] resy: Resolution Y
 * @param[in] fpy: Front Porch Y
 * @param[in] bly: Blanking Y
 * @param[in] bpy: Back Porch Y
 *****************************************************************************************
 */
void hal_gdc_timing(int resx, int fpx, int blx, int bpx, int resy, int fpy, int bly, int bpy);

/**
 *****************************************************************************************
 * @brief Get stride size in bytes.
 *
 * @param[in] format: Texture color format
 * @param[in] width: Texture width
 *
 * @return Stride in bytes
 *****************************************************************************************
 */
int hal_gdc_stride_size(hal_gdc_format_t format, int width);

/**
 *****************************************************************************************
 * @brief Set the built-in Clock Dividers and DMA Line Prefetch. (See Configuration Register 0x4)
 *
 * @param[in] div: Set Divider 1
 * @param[in] div2: Set Divider 2
 * @param[in] dma_prefetch: Set number of lines for the dma to prefetch
 * @param[in] phase: Clock phase shift
 *****************************************************************************************
 */
void hal_gdc_clkdiv(int div, int div2, int dma_prefetch, int phase);

/**
 *****************************************************************************************
 * @brief Control the clock gaters
 *
 * @param[in] ctrl: struct control
 *****************************************************************************************
 */
void hal_gdc_clkctrl(hal_gdc_clkctrl_t ctrl);

/**
 *****************************************************************************************
 * @brief Set operation mode
 *
 * @param[in] mode: Mode of operation (See Register 0)
 *****************************************************************************************
 */
void hal_gdc_set_mode(int mode);

/**
 *****************************************************************************************
 * @brief Get status from Status Register
 *
 * @return Status of DC
 *****************************************************************************************
 */
uint32_t hal_gdc_get_status (void);

/**
 *****************************************************************************************
 * @brief Request a VSync Interrupt without blocking
 *
 * @return Status of DC
 *****************************************************************************************
 */
void hal_gdc_request_vsync_non_blocking(void);

/**
 *****************************************************************************************
 * @brief Set the Layer Mode. This function can enable a layer and set attributes to it
 *
 * @param[in] layer_no: The layer number
 * @param[in] layer: Attributes struct
 *****************************************************************************************
 */
void hal_gdc_set_layer (int layer_no, hal_gdc_layer_t *layer);

/**
 *****************************************************************************************
 * @brief Set the physical address of a layer.
 *
 * @param[in] layer_no: The layer number
 * @param[in] addr: Layer Physical Address
 *****************************************************************************************
 */
void hal_gdc_set_layer_addr(int layer_no, uintptr_t addr);

/**
 *****************************************************************************************
 * @brief Set the physical address of a layer.
 *
 * @param[in] layer: Layer number
 * @param[in] index: Layer Physical Address
 * @param[in] colour: 32-bit RGBA color value or gamma index
 *****************************************************************************************
 */
void hal_gdc_set_layer_gamma_lut(int layer, int index, int colour);

/**
 *****************************************************************************************
 * @brief Get an entry in the lut8 Palette Gamma table for a layer
 *
 * @param[in] layer: Layer number
 * @param[in] index: Color Index
 *
 * @return Palette index
 *****************************************************************************************
 */
int hal_gdc_get_layer_gamma_lut(int layer, int index);

/**
 *****************************************************************************************
 * @brief Sets an entry in the lut8 Palatte Gamma table.
 *
 * @param[in] index: Color Index
 * @param[in] colour: 32-bit RGBA colour value or Gamma index
 *****************************************************************************************
 */
void hal_gdc_set_palette(uint32_t index, uint32_t colour);

/**
 *****************************************************************************************
 * @brief Reads an entry from the lut8 Palatte Gamma table
 *
 * @param[in] index: Color Index
 *
 * @return Colour for given palette index
 *****************************************************************************************
 */
int hal_gdc_get_palette(uint32_t index);

/**
 *****************************************************************************************
 * @brief Disable layer
 *
 * @param[in] layer_no: Layer Number
 *****************************************************************************************
 */
void hal_gdc_layer_disable(int layer_no);

/**
 *****************************************************************************************
 * @brief Enable layer
 *
 * @param[in] layer_no: Layer Number
 *****************************************************************************************
 */
void hal_gdc_layer_enable(int layer_no);

/**
 *****************************************************************************************
 * @brief Enable or Disable fixed cursor
 *
 * @param[in] enable: 1 for enable or 0 for disable cursor
 *****************************************************************************************
 */
void hal_gdc_cursor_enable(int enable);

/**
 *****************************************************************************************
 * @brief Set the location of the cursor
 *
 * @param[in] x: Cursor X coordinate
 * @param[in] y: Cursor Y coordinate
 *****************************************************************************************
 */
void hal_gdc_cursor_xy(int x, int y);

/**
 *****************************************************************************************
 * @brief Set programmable cursor image (32x32 pixels)
 *
 * @param[in] img: Base address of the 32x32 Cursor Image
 *****************************************************************************************
 */
void hal_gdc_set_cursor_img(unsigned char *img);

/**
 *****************************************************************************************
 * @brief Set a color for the Cursor LUT
 *
 * @param[in] index: Color index
 * @param[in] color: 32-bit RGBA value
 *****************************************************************************************
 */
void hal_gdc_set_cursor_lut(uint32_t index, uint32_t color);

/**
 *****************************************************************************************
 * @brief Check whether hal_gdc supports a specific characteristic
 *
 * @param[in] flag: Flag to query
 *
 * @return True if the characteristic is supported
 *****************************************************************************************
 */
unsigned char hal_gdc_check_config(hal_gdc_config_t flag);

/**
 *****************************************************************************************
 * @brief Read Color Mode Register
 *
 * @return Color mode register
 *****************************************************************************************
 */
uint32_t hal_gdc_get_col_mode(void);

/**
 *****************************************************************************************
 * @brief Get the number of layers available
 *
 * @return Number of layers
 *****************************************************************************************
 */
int hal_gdc_get_layer_count(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */


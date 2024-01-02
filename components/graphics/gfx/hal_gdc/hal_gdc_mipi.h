/**
 ****************************************************************************************
 *
 * @file    hal_gdc_mipi.h
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

/** @defgroup HAL_GDC_MIPI GDC MIPI
  * @brief DC-related MIPI function definition.
  * @{
  */

#ifndef HAL_GDC_MIPI_H__
#define HAL_GDC_MIPI_H__

#include "hal_gfx_sys_defs.h"

#pragma  diag_suppress      61

/**
  * @addtogroup HAL_GDC_MIPI_MACRO Defines
  * @{
  */
/** @defgroup HAL_GDC_MIPI_CFG The DC configuration
  * @{
  */
#define MIPICFG_DBI_EN           (1U<<31U)             /**< Enables MIPI DBI/SPI interface */
#define MIPICFG_FRC_CSX_0        (1U<<30U)             /**< Enables CSX force value */
#define MIPICFG_FRC_CSX_1        ((1U<<30U)|(1U<<29U)) /**< Force CSX to 1 */
#define MIPICFG_SPI_CSX_V        (1U<<29U)             /**< CSX active high/low */
#define MIPICFG_DIS_TE           (1U<<28U)             /**< Disables Input Tearing Signal */
#define MIPICFG_SPIDC_DQSPI      (1U<<27U)             /**< Enables the usage of SPI_DC wire as SPI_SD1 */
#define MIPICFG_RSTN_DBI_SPI     (1U<<26U)             /**< DBI/SPI interfaces clear */
#define MIPICFG_RESX             (1U<<25U)             /**< Controls MIPI DBI Type-B RESX output signal */
#define MIPICFG_DMA              (1U<<24U)             /**< (unused) Enables pixel data from DMA */
#define MIPICFG_SPI3             (1U<<23U)             /**< Enables SPI 3-wire interface */
#define MIPICFG_SPI4             (1U<<22U)             /**< Enables SPI 4-wire interface */
#define MIPICFG_GPI              ((1U<<23U)|(1U<<22U)) /**< Enables Generic Packet Interface */
#define MIPICFG_EN_STALL         (1U<<21U)             /**< Enables back-pressure from dbi_stall_i signal */
#define MIPICFG_SPI_CPHA         (1U<<20U)             /**< Sets SPI Clock Phase */
#define MIPICFG_SPI_CPOL         (1U<<19U)             /**< Sets SPI Clock Polarity */
#define MIPICFG_SPI_JDI          (1U<<18U)             /**< -- reserved -- */
#define MIPICFG_EN_DVALID        (1U<<18U)             /**< Enables read using external data valid signal */
#define MIPICFG_SPI_HOLD         (1U<<17U)             /**< Binds scanline address with pixel data */
#define MIPICFG_INV_ADDR         (1U<<16U)             /**< Inverts scanline address */
#define MIPICFG_SCAN_ADDR        (1U<<15U)             /**< Scan address used as header of each line */
#define MIPICFG_PIXCLK_OUT_EN    (1U<<14U)             /**< Redirects pixel generation clock to the output */
#define MIPICFG_EXT_CTRL         (1U<<13U)             /**< Enables external control signals */
#define MIPICFG_BLANKING_EN      (1U<<12U)             /**< Enables horizontal blanking */
#define MIPICFG_DSPI_SPIX        (1U<<11U)             /**< Enables DSPI sub-pixel transaction */
#define MIPICFG_QSPI             (1U<<10U)             /**< Enables QSPI */
#define MIPICFG_QSPI_DDR         ((1U<<10U)|(1U<<9U))  /**< Enables QSPI DDR */
#define MIPICFG_DSPI             (1U<< 9U)             /**< Enables DSPI */
#define MIPICFG_SPI              (0U<< 9U)             /**< Enables SPI */
#define MIPICFG_NULL             (0x00U)               /**< MIPI CFG NULL */
/** @} */

/** @defgroup HAL_GDC_MIPI_OUTPUT_MODE The DC output mode defines
  * @{
  */
#define MIPI_DCS_RGB111          (1U)                  /**< Color mode RGB111 */
#define MIPI_DCS_RGB332          (2U)                  /**< Color mode RGB332 */
#define MIPI_DCS_RGB444          (3U)                  /**< Color mode RGB444 */
#define MIPI_DCS_RGB565          (5U)                  /**< Color mode RGB565 */
#define MIPI_DCS_RGB666          (6U)                  /**< Color mode RGB666 */
#define MIPI_DCS_RGB888          (7U)                  /**< Color mode RGB888 */

#define MIPICFG_PF_SPI           (3U<<6U)              /**< Interface mode SPI */
#define MIPICFG_PF_DSPI          (4U<<6U)              /**< Interface mode DSPI */
#define MIPICFG_PF_QSPI          (5U<<6U)              /**< Interface mode QSPI */
#define MIPICFG_PF_DBI8          (0U<<6U)              /**< Interface mode DBI8 */
#define MIPICFG_PF_DBI9          (1U<<6U)              /**< Interface mode DBI9 */
#define MIPICFG_PF_DBI16         (2U<<6U)              /**< Interface mode DBI16 */
#define MIPICFG_PF_GPI           (6U<<6U)              /**< Interface mode GPI */

#define MIPICFG_PF_OPT0          (0U<<3U)              /**< Option DBI_CFG:option 0 */
#define MIPICFG_PF_OPT1          (1U<<3U)              /**< Option DBI_CFG:option 1 */
#define MIPICFG_PF_OPT2          (2U<<3U)              /**< Option DBI_CFG:option 2 */
#define MIPICFG_PF_OPT3          (3U<<3U)              /**< Option DBI_CFG:option 3 */
#define MIPICFG_PF_OPT4          (4U<<3U)              /**< Option DBI_CFG:option 4 */

#define MIPICFG_1RGB111_OPT0     (MIPICFG_PF_SPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB111)   /**< 0xc1 */
#define MIPICFG_1RGB111_OPT1     (MIPICFG_PF_SPI|MIPICFG_PF_OPT1|MIPI_DCS_RGB111)   /**< 0xc9 */
#define MIPICFG_1RGB111_OPT2     (MIPICFG_PF_SPI|MIPICFG_PF_OPT2|MIPI_DCS_RGB111)   /**< 0xd1 */
#define MIPICFG_1RGB111_OPT3     (MIPICFG_PF_SPI|MIPICFG_PF_OPT3|MIPI_DCS_RGB111)   /**< 0xd9 */
#define MIPICFG_1RGB111_OPT4     (MIPICFG_PF_SPI|MIPICFG_PF_OPT4|MIPI_DCS_RGB111)   /**< 0xe1 */
#define MIPICFG_1RGB332_OPT0     (MIPICFG_PF_SPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB332)   /**< 0xc2 */
#define MIPICFG_1RGB444_OPT0     (MIPICFG_PF_SPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB444)   /**< 0xc3 */
#define MIPICFG_1RGB565_OPT0     (MIPICFG_PF_SPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB565)   /**< 0xc5 */
#define MIPICFG_1RGB666_OPT0     (MIPICFG_PF_SPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB666)   /**< 0xc6 */
#define MIPICFG_1RGB888_OPT0     (MIPICFG_PF_SPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB888)   /**< 0xc7 */
#define MIPICFG_2RGB332_OPT0     (MIPICFG_PF_DSPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB332)  /**< 0x102 */
#define MIPICFG_2RGB444_OPT0     (MIPICFG_PF_DSPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB444)  /**< 0x103 */
#define MIPICFG_2RGB444_OPT1     (MIPICFG_PF_DSPI|MIPICFG_PF_OPT1|MIPI_DCS_RGB444)  /**< 0x10b */
#define MIPICFG_2RGB565_OPT0     (MIPICFG_PF_DSPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB565)  /**< 0x105 */
#define MIPICFG_2RGB666_OPT0     (MIPICFG_PF_DSPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB666)  /**< 0x106 */
#define MIPICFG_2RGB666_OPT1     (MIPICFG_PF_DSPI|MIPICFG_PF_OPT1|MIPI_DCS_RGB666)  /**< 0x10e */
#define MIPICFG_2RGB888_OPT0     (MIPICFG_PF_DSPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB888)  /**< 0x107 */
#define MIPICFG_2RGB888_OPT1     (MIPICFG_PF_DSPI|MIPICFG_PF_OPT1|MIPI_DCS_RGB888)  /**< 0x10f */
#define MIPICFG_4RGB111_OPT0     (MIPICFG_PF_QSPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB111)  /**< 0x141 */
#define MIPICFG_4RGB332_OPT0     (MIPICFG_PF_QSPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB332)  /**< 0x142 */
#define MIPICFG_4RGB444_OPT0     (MIPICFG_PF_QSPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB444)  /**< 0x143 */
#define MIPICFG_4RGB565_OPT0     (MIPICFG_PF_QSPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB565)  /**< 0x145 */
#define MIPICFG_4RGB666_OPT0     (MIPICFG_PF_QSPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB666)  /**< 0x146 */
#define MIPICFG_4RGB888_OPT0     (MIPICFG_PF_QSPI|MIPICFG_PF_OPT0|MIPI_DCS_RGB888)  /**< 0x147 */

#define MIPICFG_8RGB332_OPT0     (MIPICFG_PF_DBI8|MIPICFG_PF_OPT0|MIPI_DCS_RGB332)  /**< 0x2 */
#define MIPICFG_8RGB444_OPT0     (MIPICFG_PF_DBI8|MIPICFG_PF_OPT0|MIPI_DCS_RGB444)  /**< 0x3 */
#define MIPICFG_8RGB565_OPT0     (MIPICFG_PF_DBI8|MIPICFG_PF_OPT0|MIPI_DCS_RGB565)  /**< 0x5 */
#define MIPICFG_8RGB666_OPT0     (MIPICFG_PF_DBI8|MIPICFG_PF_OPT0|MIPI_DCS_RGB666)  /**< 0x6 */
#define MIPICFG_8RGB888_OPT0     (MIPICFG_PF_DBI8|MIPICFG_PF_OPT0|MIPI_DCS_RGB888)  /**< 0x7 */
#define MIPICFG_16RGB332_OPT0    (MIPICFG_PF_DBI16|MIPICFG_PF_OPT0|MIPI_DCS_RGB332) /**< 0x82 */
#define MIPICFG_16RGB444_OPT0    (MIPICFG_PF_DBI16|MIPICFG_PF_OPT0|MIPI_DCS_RGB444) /**< 0x83 */
#define MIPICFG_16RGB565_OPT0    (MIPICFG_PF_DBI16|MIPICFG_PF_OPT0|MIPI_DCS_RGB565) /**< 0x85 */
#define MIPICFG_16RGB666_OPT0    (MIPICFG_PF_DBI16|MIPICFG_PF_OPT0|MIPI_DCS_RGB666) /**< 0x86 */
#define MIPICFG_16RGB666_OPT1    (MIPICFG_PF_DBI16|MIPICFG_PF_OPT1|MIPI_DCS_RGB666) /**< 0x8e */
#define MIPICFG_16RGB888_OPT0    (MIPICFG_PF_DBI16|MIPICFG_PF_OPT0|MIPI_DCS_RGB888) /**< 0x87 */
#define MIPICFG_16RGB888_OPT1    (MIPICFG_PF_DBI16|MIPICFG_PF_OPT1|MIPI_DCS_RGB888) /**< 0x8f */
#define MIPICFG_9RGB666_OPT0     (MIPICFG_PF_DBI9|MIPICFG_PF_OPT0|MIPI_DCS_RGB666)  /**< 0x46 */
#define MIPICFG_32RGB332_OPT0    (MIPICFG_PF_GPI  |MIPICFG_PF_OPT0|MIPI_DCS_RGB332) /**< 0x182 */
#define MIPICFG_32RGB444_OPT0    (MIPICFG_PF_GPI  |MIPICFG_PF_OPT0|MIPI_DCS_RGB444) /**< 0x183 */
#define MIPICFG_32RGB565_OPT0    (MIPICFG_PF_GPI  |MIPICFG_PF_OPT0|MIPI_DCS_RGB565) /**< 0x185 */
#define MIPICFG_32RGB666_OPT0    (MIPICFG_PF_GPI  |MIPICFG_PF_OPT0|MIPI_DCS_RGB666) /**< 0x186 */
#define MIPICFG_32RGB666_OPT1    (MIPICFG_PF_GPI  |MIPICFG_PF_OPT1|MIPI_DCS_RGB666) /**< 0x18e */
#define MIPICFG_32RGB888_OPT0    (MIPICFG_PF_GPI  |MIPICFG_PF_OPT0|MIPI_DCS_RGB888) /**< 0x187 */
/** @} */

/** @defgroup HAL_GDC_MIPI_SET_MODE The DC set mode
  * @{
  */
#define hal_gdc_MIPI_set_mode    hal_gdc_MIPI_set_pixel_format      /**< for backward compatibility */
/** @} */

/** @} */

/** @addtogroup HAL_GDC_MIPI_ENUM Enumerations
  * @{
  */
/**
  * @brief MIPI Command List definition
  */
typedef enum {
    MIPI_enter_idle_mode       = 0x39,          /**< Enter idle mode. */
    MIPI_enter_invert_mode     = 0x21,          /**< Enter invert mode. */
    MIPI_enter_normal_mode     = 0x13,          /**< Enter normal mode. */
    MIPI_enter_partial_mode    = 0x12,          /**< Enter partial mode. */
    MIPI_enter_sleep_mode      = 0x10,          /**< Enter sleep mode. */
    MIPI_exit_idle_mode        = 0x38,          /**< Exit idle mode. */
    MIPI_exit_invert_mode      = 0x20,          /**< Exit invert mode. */
    MIPI_exit_sleep_mode       = 0x11,          /**< Exit sleep mode. */
    MIPI_get_3D_control        = 0x3f,          /**< Get 3D control. */
    MIPI_get_address_mode      = 0x0b,          /**< Get address mode. */
    MIPI_get_blue_channel      = 0x08,          /**< Get blue channel. */
    MIPI_get_diagnostic_result = 0x0f,          /**< Get diagnostic result. */
    MIPI_get_display_mode      = 0x0d,          /**< Get display mode. */
    MIPI_get_green_channel     = 0x07,          /**< Get green channel. */
    MIPI_get_pixel_format      = 0x0c,          /**< Get pixel format. */
    MIPI_get_power_mode        = 0x0a,          /**< Get power mode. */
    MIPI_get_red_channel       = 0x06,          /**< Get red channel. */
    MIPI_get_scanline          = 0x45,          /**< Get scanline. */
    MIPI_get_signal_mode       = 0x0e,          /**< Get signal mode. */
    MIPI_nop                   = 0x00,          /**< MIPI nop. */
    MIPI_read_DDB_continue     = 0xa8,          /**< Read DDB continue. */
    MIPI_read_DDB_start        = 0xa1,          /**< Read DDB start. */
    MIPI_read_memory_continue  = 0x3e,          /**< Read memory continue. */
    MIPI_read_memory_start     = 0x2e,          /**< Read memory start. */
    MIPI_set_3D_control        = 0x3d,          /**< Set 3D control. */
    MIPI_set_address_mode      = 0x36,          /**< Set address mode. */
    MIPI_set_column_address    = 0x2a,          /**< Set column address. */
    MIPI_set_display_off       = 0x28,          /**< Set display off. */
    MIPI_set_display_on        = 0x29,          /**< Set display on. */
    MIPI_set_gamma_curve       = 0x26,          /**< Set gamma curve. */
    MIPI_set_page_address      = 0x2b,          /**< Set page address. */
    MIPI_set_partial_columns   = 0x31,          /**< Set partial columns. */
    MIPI_set_partial_rows      = 0x30,          /**< Set partial rows. */
    MIPI_set_pixel_format      = 0x3a,          /**< Set pixel format. */
    MIPI_set_scroll_area       = 0x33,          /**< Set scroll area. */
    MIPI_set_scroll_start      = 0x37,          /**< Set scroll start. */
    MIPI_set_tear_off          = 0x34,          /**< Set tear off. */
    MIPI_set_tear_on           = 0x35,          /**< Set tear on. */
    MIPI_set_tear_scanline     = 0x44,          /**< Set tear scanline. */
    MIPI_set_vsync_timing      = 0x40,          /**< Set vsync timing. */
    MIPI_soft_reset            = 0x01,          /**< Soft reset. */
    MIPI_write_LUT             = 0x2d,          /**< Write LUT. */
    MIPI_write_memory_continue = 0x3c,          /**< Write memory continue. */
    MIPI_write_memory_start    = 0x2c,          /**< Write memory start. */
    MIPI_snapshot              = 0xff,          /**< snapshot. */
    MIPI_DBIB_STORE_BASE_ADDR  = (1 <<31),      /**< DBIB store base addr. */
    MIPI_DBIB_CMD              = (1U<<30),      /**< DBIB CMD. */
    MIPI_CMD08                 = (0U<<28),      /**< Set cmd width to 8bit */
    MIPI_CMD16                 = (1U<<28),      /**< Set cmd width to 16bit */
    MIPI_CMD24                 = (1U<<29),      /**< Set cmd width to 24bit */
    MIPI_MASK_QSPI             = (1U<<27),      /**< Qspi is forced to single line */
} hal_gdc_mipi_cmd_t;

/** @} */

/** @addtogroup HAL_GDC_MIPI_FUNCTION Functions
  * @{
  */
/**
 *****************************************************************************************
 * @brief Send command or data to MIPI Interface.
 *
 * @param[in] cmd: command or data to be sent
 *****************************************************************************************
 */
void hal_gdc_MIPI_out(int cmd);

/**
 *****************************************************************************************
 * @brief Configure hal_gdc's serial interace.
 *
 * @param[in] cfg: configuration mode
 *****************************************************************************************
 */
void hal_gdc_MIPI_CFG_out(int cfg);

/**
 *****************************************************************************************
 * @brief Read data from MIPI interface.
 *
 * @return data form MIPI interface
 *****************************************************************************************
 */
int hal_gdc_MIPI_in(void);

/**
 *****************************************************************************************
 * @brief Read MIPI DBI Type-B parameters.
 *
 * @param[in] cmd: MIPI DCS command
 * @param[in] n_params: Number of parameters to read (max: 3)
 *
 * @return The read parameters
 *****************************************************************************************
 */
unsigned hal_gdc_MIPI_read(int cmd, int n_params);

/**
 *****************************************************************************************
 * @brief Send DCS command to display over the physical interface.
 *
 * @param[in] cmd: MIPI DCS command
 *****************************************************************************************
 */
void hal_gdc_MIPI_cmd(int cmd);

/**
 *****************************************************************************************
 * @brief Similar to hal_gdc_MIPI_cmd, with command parameters.
 *
 * @param[in] cmd: MIPI DCS command
 * @param[in] n_params: Number of cmd parameters
 *****************************************************************************************
 */
void hal_gdc_MIPI_cmd_params(int cmd, int n_params,...);

/**
 *****************************************************************************************
 * @brief Does Partial Update in MIPI.
 *
 * @param[in] start_x: start x coordinate
 * @param[in] start_y: start y coordinate
 * @param[in] end_x: end x coordinate
 * @param[in] end_y: end y coordinate
 *
 * @return Always 1, user can ignore the result
 *****************************************************************************************
 */
int hal_gdc_MIPI_updateregion(int start_x, int start_y,int end_x,   int end_y);

/**
 *****************************************************************************************
 * @brief Convenience function. Sends exit_sleep and display_on commands.
 *****************************************************************************************
 */
void hal_gdc_MIPI_enable(void);

/**
 *****************************************************************************************
 * @brief Convenience function. Sends display_off and enter_sleep_mode commands.
 *****************************************************************************************
 */
void hal_gdc_MIPI_disable(void);

/**
 *****************************************************************************************
 * @brief Set the display pixel format. Sends set_pixel_format command to the display.
 *
 * @param[in] pixel_format: pixel format
 *****************************************************************************************
 */
void hal_gdc_MIPI_set_pixel_format(int pixel_format);

/**
 *****************************************************************************************
 * @brief Set the frame position. Sends set_column_address and set_page_address commands.
 *
 * @param[in] minx: frames' minimum x
 * @param[in] miny: frames' minimum y
 * @param[in] maxx: frame's maximum x
 * @param[in] maxy: frame's maximum y
 *****************************************************************************************
 */
void hal_gdc_MIPI_set_position(int minx, int miny, int maxx, int maxy);

/**
 *****************************************************************************************
 * @brief Set the display partial area and enter Partial Display Mode.
 *
 * @param[in] minx: partial areas' minimum x
 * @param[in] miny: partial areas' minimum y
 * @param[in] maxx: partial areas' maximum x
 * @param[in] maxy: partial areas' maximum y
 *****************************************************************************************
 */
void hal_gdc_MIPI_set_partial_mode(int minx, int miny, int maxx, int maxy);

 /**
  *****************************************************************************************
  * @brief Convenience function. Send a write_memory_start command in order to start transfering the frame to the display.
  *****************************************************************************************
  */
void hal_gdc_MIPI_start_frame_transfer(void);

/** @} */

#endif // HAL_GDC_MIPI_H__
/** @} */
/** @} */
/** @} */


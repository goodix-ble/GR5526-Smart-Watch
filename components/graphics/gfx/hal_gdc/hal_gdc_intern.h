/**
 ****************************************************************************************
 *
 * @file    hal_gdc_intern.h
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

/** @defgroup HAL_GDC_INTERN GDC Intern
  * @brief DC-related MIPI command definition.
  * @{
  */

#ifndef HAL_GDC_INTERN_H__
#define HAL_GDC_INTERN_H__

/**
  * @addtogroup HAL_GDC_INTERN_MACRO Defines
  * @{
  */
/** @defgroup HAL_GDC_INTERN_RW_CMD The DC RW CMD
  * @{
  */
#define HAL_GDC_MAGIC           (0x87452365)                 /**< DC ID */
#define HAL_GDC_DBIB_STALL      (1U <<  31)                  /**< Specify MIPI DBI */
#define SPI_WRITE               (2U)                         /**< Set to SPI Write CMD */
#define SPI_READ                (3U)                         /**< Set to SPI Read CMD */
#define SPICMD                  (1U<<5U)                     /**< Special as SPI CMD */
#define QSPICMD                 (0U<<5U)                     /**< Special as QSPI CMD */
#define QSPIDATA                (1U<<4U)                     /**< Special as QSPI DATA */
#define CMD_OFFSET              (8U)                         /**< offset for command */
#define CMD1_DATA1              (SPI_WRITE)                  /**< Send cmd in 1-wire and data in 1-wire */
#define CMD1_DATA4              (SPICMD|QSPIDATA|SPI_WRITE)  /**< Send cmd in 1-wire and data in 4-wire */
#define CMD4_DATA4              (QSPICMD|QSPIDATA|SPI_WRITE) /**< Send cmd in 4-wire and data in 4-wire */
#define CMD1_RDAT1              (SPI_READ)                   /**< Read cmd in 1-wire and data in 1-wire */
/** @} */

/** @} */

/** @addtogroup HAL_GDC_INTERN_ENUM Enumerations
  * @{
  */
/**
  * @brief DC's color mode check definition
  */
typedef enum {
    HAL_GDC_DBIB_STALL_CHK     = 1  <<  31,         /**< DBIB stall check */
    HAL_GDC_OPENLDI_CHK        = 1U <<  30,         /**< Openldi check */
    HAL_GDC_JDIMIP_CHK         = 1U <<  29,         /**< JDIMIP check */
    HAL_GDC_ARGB4444_CHK       = 1U <<  22,         /**< ARGB444 check */
    HAL_GDC_RGBA4444_CHK       = 1U <<  21,         /**< RGBA444 check */
    HAL_GDC_GPI_CHK            = 1U <<  20,         /**< GPI check */
    HAL_GDC_EXTRCTRL_CHK       = 1U <<  19,         /**< EXTRCTRL check */
    HAL_GDC_TSC6_CHK           = 1U <<  18,         /**< TSC6 check */
    HAL_GDC_TSC_CHK            = 1U <<  17,         /**< TSC check */
    HAL_GDC_LUT8_CHK           = 1U <<  16,         /**< LUT8 check */
    HAL_GDC_RGBA5551_CHK       = 1U <<  15,         /**< RGBA5551 check */
    HAL_GDC_ABGR8888_CHK       = 1U <<  14,         /**< ARGB8888 check */
    HAL_GDC_RGB332_CHK         = 1U <<  13,         /**< RGB332 check */
    HAL_GDC_RGB565_CHK         = 1U <<  12,         /**< RGB565 check */
    HAL_GDC_BGRA8888_CHK       = 1U <<  11,         /**< BGRA8888 check */
    HAL_GDC_L8_CHK             = 1U <<  10,         /**< L8 check */
    HAL_GDC_L1_CHK             = 1U <<  9,          /**< L1 check */
    HAL_GDC_L4_CHK             = 1U <<  8,          /**< L4 check */
    HAL_GDC_YUVV_CHK           = 1U <<  7,          /**< YUVV check */
    HAL_GDC_RGB24_CHK          = 1U <<  6,          /**< RGB24 check */
    HAL_GDC_YUY2_CHK           = 1U <<  5,          /**< YUV2 check */
    HAL_GDC_RGBA8888_CHK       = 1U <<  4,          /**< RGBA8888 check */
    HAL_GDC_ARGB8888_CHK       = 1U <<  3,          /**< ARGB8888 check */
    HAL_GDC_V_YUV420_CHK       = 1U <<  2,          /**< V_YUV420 check */
    HAL_GDC_TLYUV420_CHK       = 1U <<  1,          /**< TLYUV420 check */
    HAL_GDC_BLOCK4X4_CHK       = 1U <<  0           /**< BLOCK4X4 check */
} hal_gdc_colormode_check_t;

/**
  * @brief AXI control definition
  */
typedef enum {
    HAL_GDC_AXI_16BEAT   = 0x0,             /**< 16 beat */
    HAL_GDC_AXI_2BEAT    = 0x1,             /**< 2beat */
    HAL_GDC_AXI_4BEAT    = 0x2,             /**< 4 beat */
    HAL_GDC_AXI_8BEAT    = 0x3,             /**< 8 beat */
    HAL_GDC_AXI_32BEAT   = 0x5,             /**< 32 beat */
    HAL_GDC_AXI_64BEAT   = 0x6,             /**< 64 beat */
    HAL_GDC_AXI_128BEAT  = 0x7,             /**< 128 beat */
    HAL_GDC_AXI_FT_HF    = 0x0U << 3,       /**< Ignore */
    HAL_GDC_AXI_FT_2B    = 0x1U << 3,       /**< Ignore */
    HAL_GDC_AXI_FT_4B    = 0x2U << 3,       /**< Ignore */
    HAL_GDC_AXI_FT_8B    = 0x3U << 3        /**< Ignore */
} hal_gdc_AXI_cfg_t;

/**
  * @brief MIPI CMD definition
  */
typedef enum {
    hal_gdc_snapshot              = 0xff,                /**< Snapshot */
    hal_gdc_store_base_addr       = (1 <<31),            /**< Store base addr */
    hal_gdc_DBI_cmd               = (1U<<30),            /**< DBI CMD */
    hal_gdc_wcmd16                = (1U<<28),            /**< Write CMD 16bits */
    hal_gdc_wcmd24                = (1U<<29),            /**< Write CMD 24bits */
    hal_gdc_wcmd32                = (1U<<29)|(1U<<28),   /**< Write CMD 32bits */
    hal_gdc_rcmd16                = (1U<<28),            /**< Read CMD 16bits */
    hal_gdc_rcmd24                = (1U<<29),            /**< Read CMD 24its */
    hal_gdc_rcmd32                = (1U<<29)|(1U<<28),   /**< Read CMD 32bits */
    hal_gdc_mask_qspi             = (1U<<27),            /**< Force QSPI to be single wire */
    hal_gdc_DBI_ge                = (1U<<27),            /**< DBI ge */
    hal_gdc_DBI_read              = (1U<<26),            /**< DBI read */
    hal_gdc_ext_ctrl              = (1U<<25),            /**< Ext control */
    hal_gdc_sline_cmd             = (1U<<24),            /**< Sline CMD */
    hal_gdc_read_byte             = (0U<<30),            /**< Read 1Byte */
    hal_gdc_read_2byte            = (1U<<30),            /**< Read 2Byte */
    hal_gdc_read_3byte            = (2 <<30),            /**< Read 3Byte */
    hal_gdc_read_4byte            = (3 <<30)             /**< Read 4Byte */
} dc_mipi_cmd_t;

/** @} */

#endif

/** @} */
/** @} */
/** @} */

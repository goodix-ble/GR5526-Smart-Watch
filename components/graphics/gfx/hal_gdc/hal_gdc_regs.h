/**
 ****************************************************************************************
 *
 * @file    hal_gdc_regs.h
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

/** @defgroup HAL_GDC_REG GDC REG
  * @brief DC's registers definition
  * @{
  */

#ifndef HAL_GDC_REGS_H__
#define HAL_GDC_REGS_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GDC_REG_MACRO_DEFINITION Defines
 * @{
 */
#define HAL_GDC_REG_MODE             (0x00U)    /**< This register specifies the general control. */
#define HAL_GDC_REG_CLKCTRL          (0x04U)    /**< This register specifies the clock division. */
#define HAL_GDC_REG_PLAY             (0x10U)    /**< This register sequence the trigger the DC. */
#define HAL_GDC_REG_CLKCTRL_CG       (0x1a8U)   /**< This register specifies the clock control of DC.. */
#define HAL_GDC_REG_BGCOLOR          (0x08U)    /**< This register specifies the main background color */
#define HAL_GDC_REG_RESXY            (0x0cU)    /**< This register specifies the main X and Y resolutions. */
#define HAL_GDC_REG_FRONTPORCHXY     (0x14U)    /**< This register specifies the X and Y front porch. */
#define HAL_GDC_REG_BLANKINGXY       (0x18U)    /**< This register specifies the X and Y blanking period. */
#define HAL_GDC_REG_BACKPORCHXY      (0x1cU)    /**< This register specifies the X and Y back porch. */
#define HAL_GDC_REG_CURSORXY         (0x20U)    /**< This register specifies the cursor's start X and Y coordinates */
#define HAL_GDC_REG_STARTXY          (0x24U)    /**< This register specifies the start position of the first frame. */
#define HAL_GDC_REG_DBIB_CFG         (0x28U)    /**< This register specifies the configuration of the interface. */
#define HAL_GDC_REG_GPIO             (0x2cU)    /**< This register specifies the general Purpose */

#define HAL_GDC_REG_LAYER0_MODE      (0x30U)    /**< This register specifies the mode of layer0. */
#define HAL_GDC_REG_LAYER0_STARTXY   (0x34U)    /**< This register specifies the start position of the layer0. */
#define HAL_GDC_REG_LAYER0_SIZEXY    (0x38U)    /**< This register specifies the size of the layer0. */
#define HAL_GDC_REG_LAYER0_BASEADDR  (0x3cU)    /**< This register specifies the start address of the framebuffer. */
#define HAL_GDC_REG_LAYER0_STRIDE    (0x40U)    /**< This register specifies the stride of the layer0. */
#define HAL_GDC_REG_LAYER0_RESXY     (0x44U)    /**< This register specifies the resolution of the layer0. */
#define HAL_GDC_REG_LAYER0_SCALEX    (0x48U)    /**< This register specifies the scale-x of the layer0. */
#define HAL_GDC_REG_LAYER0_SCALEY    (0x4cU)    /**< This register specifies the scale-y of the layer0. */

#define HAL_GDC_REG_LAYER1_MODE      (0x50U)    /**< This register specifies the mode of layer1. */
#define HAL_GDC_REG_LAYER1_STARTXY   (0x54U)    /**< This register specifies the start position of the layer1. */
#define HAL_GDC_REG_LAYER1_SIZEXY    (0x58U)    /**< This register specifies the size of the layer1. */
#define HAL_GDC_REG_LAYER1_BASEADDR  (0x5cU)    /**< This register specifies the start address of the framebuffer. */
#define HAL_GDC_REG_LAYER1_STRIDE    (0x60U)    /**< This register specifies the stride of the layer1. */
#define HAL_GDC_REG_LAYER1_RESXY     (0x64U)    /**< This register specifies the resolution of the layer1. */
#define HAL_GDC_REG_LAYER1_SCALEX    (0x68U)    /**< This register specifies the scale-x of the layer1. */
#define HAL_GDC_REG_LAYER1_SCALEY    (0x6cU)    /**< This register specifies the scale-y of the layer1. */

#define HAL_GDC_REG_LAYER2_MODE      (0x70U)    /**< This register specifies the mode of layer2. */
#define HAL_GDC_REG_LAYER2_STARTXY   (0x74U)    /**< This register specifies the start position of the layer2. */
#define HAL_GDC_REG_LAYER2_SIZEXY    (0x78U)    /**< This register specifies the size of the layer2. */
#define HAL_GDC_REG_LAYER2_BASEADDR  (0x7cU)    /**< This register specifies the start address of the framebuffer. */
#define HAL_GDC_REG_LAYER2_STRIDE    (0x80U)    /**< This register specifies the stride of the layer2. */
#define HAL_GDC_REG_LAYER2_RESXY     (0x84U)    /**< This register specifies the resolution of the layer2. */
#define HAL_GDC_REG_LAYER2_SCALEX    (0x88U)    /**< This register specifies the scale-x of the layer2. */
#define HAL_GDC_REG_LAYER2_SCALEY    (0x8cU)    /**< This register specifies the scale-y of the layer2. */

#define HAL_GDC_REG_LAYER3_MODE      (0x90U)    /**< This register specifies the mode of layer3. */
#define HAL_GDC_REG_LAYER3_STARTXY   (0x94U)    /**< This register specifies the start position of the layer3. */
#define HAL_GDC_REG_LAYER3_SIZEXY    (0x98U)    /**< This register specifies the size of the layer3. */
#define HAL_GDC_REG_LAYER3_BASEADDR  (0x9cU)    /**< This register specifies the start address of the framebuffer. */
#define HAL_GDC_REG_LAYER3_STRIDE    (0xa0U)    /**< This register specifies the stride of the layer3. */
#define HAL_GDC_REG_LAYER3_RESXY     (0xa4U)    /**< This register specifies the resolution of the layer3. */
#define HAL_GDC_REG_LAYER3_SCALEX    (0xa8U)    /**< This register specifies the scale-x of the layer3. */
#define HAL_GDC_REG_LAYER3_SCALEY    (0xacU)    /**< This register specifies the scale-y of the layer3. */

#define HAL_GDC_REG_LAYER0_UBASE     (0xd0U)    /**< This register specifies the start address of the U chroma for layer 0 YUV planar format */
#define HAL_GDC_REG_LAYER0_VBASE     (0xd4U)    /**< This register specifies the start address of the V chroma for layer 0 YUV planar format */
#define HAL_GDC_REG_LAYER0_UVSTRIDE  (0xd8U)    /**< This register specifies the start address of the V chroma for layer 0 YUV planar format */
#define HAL_GDC_REG_LAYER1_UBASE     (0xdcU)    /**< This register specifies the start address of the U chroma for layer 1 YUV planar format */
#define HAL_GDC_REG_LAYER1_VBASE     (0xe0U)    /**< This register specifies the start address of the V chroma for layer 1 YUV planar format */
#define HAL_GDC_REG_LAYER1_UVSTRIDE  (0xe4U)    /**< This register specifies the start address of the V chroma for layer 1 YUV planar format */
#define HAL_GDC_REG_LAYER2_UBASE     (0x188U)   /**< This register specifies the start address of the U chroma for layer 2 YUV planar format */
#define HAL_GDC_REG_LAYER2_VBASE     (0x18cU)   /**< This register specifies the start address of the V chroma for layer 2 YUV planar format */
#define HAL_GDC_REG_LAYER2_UVSTRIDE  (0x190U)   /**< This register specifies the start address of the V chroma for layer 2 YUV planar format */
#define HAL_GDC_REG_LAYER3_UBASE     (0x194U)   /**< This register specifies the start address of the U chroma for layer 3 YUV planar format */
#define HAL_GDC_REG_LAYER3_VBASE     (0x198U)   /**< This register specifies the start address of the V chroma for layer 3 YUV planar format */
#define HAL_GDC_REG_LAYER3_UVSTRIDE  (0x19cU)   /**< This register specifies the start address of the V chroma for layer 3 YUV planar format */

#define HAL_GDC_REG_DBIB_CMD         (0xe8U)    /**< This register specifies the CMD to SPI interface. */
#define HAL_GDC_REG_DBIB_RDAT        (0xecU)    /**< This register specifies the read data by SPI interface. */
#define HAL_GDC_REG_CONFIG           (0xf0U)    /**< This register specifies the configuration of DC. */
#define HAL_GDC_REG_IDREG            (0xf4U)    /**< This register specifies the ID of DC. */
#define HAL_GDC_REG_INTERRUPT        (0xf8U)    /**< This register specifies the interrupt of DC. */
#define HAL_GDC_REG_STATUS           (0xfcU)    /**< This register specifies the status of DC. */
#define HAL_GDC_REG_COLMOD           (0x100U)   /**< This register specifies the color mode of DC. */
#define HAL_GDC_REG_CRC              (0x184U)   /**< This register specifies the CRC check of DC.  */

#define HAL_GDC_REG_FORMAT_CTRL      (0x1a0U)   /**< This register specifies the format control of DC. */
#define HAL_GDC_REG_FORMAT_CTRL2     (0x1a4U)   /**< This register specifies the format control of DC. */
#define HAL_GDC_REG_FORMAT_CTRL3     (0x1acU)   /**< This register specifies the format control of DC. */
#define HAL_GDC_REG_PALETTE          (0x400U)   /**< This register global palette/gamma correction memory region */
#define HAL_GDC_REG_CURSOR_IMAGE     (0x800U)   /**< This register specifies the cursor image. */
#define HAL_GDC_REG_CURSOR_LUT       (0xA00U)   /**< This register specifies the lut of cursor */
#define HAL_GDC_REG_GAMMALUT_0       (0x1000U)  /**< This register specifies the gammalut of layer0. */
#define HAL_GDC_REG_GAMMALUT_1       (0x1400U)  /**< This register specifies the gammalut of layer1. */
#define HAL_GDC_REG_GAMMALUT_2       (0x1800U)  /**< This register specifies the gammalut of layer2. */
#define HAL_GDC_REG_GAMMALUT_3       (0x1c00U)  /**< This register specifies the gammalut of layer3. */

#define HAL_GDC_REG_LAYER_MODE(i)      (0x030 + 0x20*(i))    /**< This register specifies the mode of layeri. */
#define HAL_GDC_REG_LAYER_STARTXY(i)   (0x034 + 0x20*(i))    /**< This register specifies the start position of the layeri. */
#define HAL_GDC_REG_LAYER_SIZEXY(i)    (0x038 + 0x20*(i))    /**< This register specifies the size of the layeri. */
#define HAL_GDC_REG_LAYER_BASEADDR(i)  (0x03c + 0x20*(i))    /**< This register specifies the start address of the framebuffer. */
#define HAL_GDC_REG_LAYER_STRIDE(i)    (0x040 + 0x20*(i))    /**< This register specifies the stride of the layeri. */
#define HAL_GDC_REG_LAYER_RESXY(i)     (0x044 + 0x20*(i))    /**< This register specifies the resolution of the layeri. */
#define HAL_GDC_REG_LAYER_SCALEX(i)    (0x048 + 0x20*(i))    /**< This register specifies the scale-x of the layeri. */
#define HAL_GDC_REG_LAYER_SCALEY(i)    (0x04c + 0x20*(i))    /**< This register specifies the scale-y of the layeri. */
#define HAL_GDC_REG_GAMMALUT(i)        (0x1000+ 0x400*(i))   /**< This register specifies the gammaluti. */
/** @} */

#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */

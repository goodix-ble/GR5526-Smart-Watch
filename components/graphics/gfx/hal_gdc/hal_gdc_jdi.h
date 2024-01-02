// -----------------------------------------------------------------------------
// Copyright (c) 2019 Think Silicon S.A.
// Think Silicon S.A. Confidential Proprietary
// -----------------------------------------------------------------------------
//     All Rights reserved - Unpublished -rights reserved under
//         the Copyright laws of the European Union
//
//  This file includes the Confidential information of Think Silicon S.A.
//  The receiver of this Confidential Information shall not disclose
//  it to any third party and shall protect its confidentiality by
//  using the same degree of care, but not less than a reasonable
//  degree of care, as the receiver uses to protect receiver's own
//  Confidential Information. The entire notice must be reproduced on all
//  authorised copies and copies may only be made to the extent permitted
//  by a licensing agreement from Think Silicon S.A..
//
//  The software is provided 'as is', without warranty of any kind, express or
//  implied, including but not limited to the warranties of merchantability,
//  fitness for a particular purpose and noninfringement. In no event shall
//  Think Silicon S.A. be liable for any claim, damages or other liability, whether
//  in an action of contract, tort or otherwise, arising from, out of or in
//  connection with the software or the use or other dealings in the software.
//
//
//                    Think Silicon S.A.
//                    http://www.think-silicon.com
//                    Patras Science Park
//                    Rion Achaias 26504
//                    Greece
// -----------------------------------------------------------------------------

#ifndef HAL_GFX_DC_JDI_H__
#define HAL_GFX_DC_JDI_H__

#include "hal_gfx_sys_defs.h"

typedef enum {
    JDI_CMD_NOP         = 0x0,
    JDI_CMD_NOUPDATE    = 0x0,
    JDI_CMD_BLINKOFF    = 0x0,
    JDI_CMD_BLINKBLACK  = 0x10,
    JDI_CMD_BLINKWHITE  = 0x18,
    JDI_CMD_BLINKINVERT = 0x14,
    JDI_CMD_CLEAR       = 0x20
} jdi_cmd_t;

typedef enum {
    JDI_DATAMODE_3BIT   = 0x80,
    JDI_DATAMODE_1BIT   = 0x88,
    JDI_DATAMODE_4BIT   = 0x90,
    JDI_DATAMODE_SHARP  = 0xA0
} jdi_data_mode_t;

typedef enum {
    JDI_PHY_SPI4      ,
    JDI_PHY_SPI4_SHARP,
    JDI_PHY_SPI3      ,
} jdi_phy_t;

void hal_gdc_jdi_configure(jdi_phy_t phy, jdi_data_mode_t data_mode, uint32_t extra_flags);
void hal_gdc_jdi_send_one_frame(int starty);
void hal_gdc_jdi_clear(void);
void hal_gdc_jdi_blink_off(void);
void hal_gdc_jdi_blink_inv_colors(void);
void hal_gdc_jdi_blink_white(void);
void hal_gdc_jdi_blink_black(void);
#endif

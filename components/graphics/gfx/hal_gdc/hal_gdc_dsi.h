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
//------------------------------------------------------------------------------
#ifndef HAL_GDC_DSI_H__
#define HAL_GDC_DSI_H__
//------------------------------------------------------------------------------
#include "hal_gfx_sys_defs.h"
//------------------------------------------------------------------------------

//-----------------------------------------------------

typedef enum {
    //----------------------------------------------
    hal_gdc_dt_vsync_start                        = 0x01,
    hal_gdc_dt_vsync_end                          = 0x11,
    hal_gdc_dt_hsync_start                        = 0x21,
    hal_gdc_dt_hsync_end                          = 0x31,
    hal_gdc_dt_cmpr_mode                          = 0x07,
    hal_gdc_dt_end_of_trans                       = 0x08,
    hal_gdc_dt_pic_param                          = 0x0a,
    hal_gdc_dt_cmpr_pix_stream                    = 0x0b,
    hal_gdc_dt_color_mode_off                     = 0x02,
    hal_gdc_dt_color_mode_on                      = 0x12,
    hal_gdc_dt_shut_down_peripheral               = 0x22,
    hal_gdc_dt_turn_on_peripheral                 = 0x32,
    hal_gdc_dt_generic_short_write_param_no       = 0x03,
    hal_gdc_dt_generic_short_write_param_n1       = 0x13,
    hal_gdc_dt_generic_short_write_param_n2       = 0x23,
    hal_gdc_dt_generic_read_param_no              = 0x04,
    hal_gdc_dt_generic_read_param_n1              = 0x14,
    hal_gdc_dt_execute_queue                      = 0x16,
    hal_gdc_dt_generic_read_param_n2              = 0x24,
    hal_gdc_dt_DCS_short_write_param_no           = 0x05,
    hal_gdc_dt_DCS_short_write_param_n1           = 0x15,
    hal_gdc_dt_DCS_read_param_no                  = 0x06,
    hal_gdc_dt_set_max_return_packet_size         = 0x37,
    hal_gdc_dt_blanking_packet                    = 0x19,
    hal_gdc_dt_generic_long_write                 = 0x29,
    hal_gdc_dt_DCS_long_write                     = 0x39,
    hal_gdc_dt_packed_pixel_stream_rgb565         = 0x0e,
    hal_gdc_dt_packed_pixel_stream_rgb666         = 0x1e,
    hal_gdc_dt_loosely_packed_pixel_stream_rgb666 = 0x2e,
    hal_gdc_dt_loosely_packed_pixel_stream_rgb888 = 0x3e
} hal_gdc_dsi_data_types_t;

    //----------------------------------------------
typedef enum {
    hal_gdc_dcs_datacmd                   =      0U,
    hal_gdc_ge_data                       = (1U<<30),
    hal_gdc_ge_cmd                        = (1 <<31),
    hal_gdc_ge_datacmd                    = ((1 <<30)|(1 <<31))
} hal_gdc_dsi_types_t;
    //----------------------------------------------
//-----------------------------------------------------

//------------------------------------------------------------------------------

/** \brief Send scanline command and start memory write
*
*/
void hal_gdc_dsi_start_frame_transfer(void);

/** \brief Send scanline command and start memory write (generic)
*
*/
void hal_gdc_dsi_start_frame_transfer_generic(void);

/** \brief DC DBI interface to DSI
 *
 * \param  data_type  Data (pixel) type
 * \param  cmd_type   Command type
 * \param  type       DSI command type
 */
void hal_gdc_dsi_ct(hal_gdc_dsi_data_types_t data_type, hal_gdc_dsi_data_types_t cmd_type, hal_gdc_dsi_types_t type);


#endif // HAL_GDC_DSI_H__

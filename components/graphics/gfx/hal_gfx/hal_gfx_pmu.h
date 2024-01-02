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

#ifndef HAL_GFX_PMU_H__
#define HAL_GFX_PMU_H__

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------------- PMU ------------------------------------
//--------------------------------------------------
//@function hal_gfx_start_clock
//@brief Start core clock
//--------------------------------------------------
void hal_gfx_start_clock(void);

//--------------------------------------------------
//@function hal_gfx_stop_clock
//@brief Stop core clock
//--------------------------------------------------
void hal_gfx_stop_clock(void);

//--------------------------------------------------
//@function hal_gfx_control_cg
//@brief Disable clockgating
//@param int cg_dis desc: Code that controls which modules' clockgating will be disabled. If the code is 0 clockgating is enabled for all modules. Bits description -> 31: all modules, 30-27: imem(3-0), 26-23: cores(3-0), 22-19: sched(3-0), 18-15: regs(3-0), 14-11: pipes(3-0), 10-7: texs(3-0), 6-3: fbs(3-0), 2: rast, 1: confregs, 0: cmdlist)
//--------------------------------------------------
void hal_gfx_control_cg(int cg_dis);

//--------------------------------------------------
//@function hal_gfx_change_clock
//@brief Change core clock frequency
//@param int freq_code desc: Code that controls core clock frequency (0:synchronous, 1, 2, 3)
//--------------------------------------------------
void hal_gfx_change_clock(int freq_code);

#ifdef __cplusplus
}
#endif

#endif

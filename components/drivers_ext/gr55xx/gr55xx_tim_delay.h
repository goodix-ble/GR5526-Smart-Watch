/**
 ****************************************************************************************
 *
 * @file gr551x_tim_delay.h
 *
 * @brief Header file - GR551x tim delay.
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
 *****************************************************************************************
 */
#ifndef __GR551X_TIM_DELAY_H__
#define __GR551X_TIM_DELAY_H__

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdint.h>
#include "grx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief  Initialize the DUAL TIM according to the specified register
 *         in the dual_timer_regs_t.
 ****************************************************************************************
 */
void tim_delay_init(dual_timer_regs_t *timx);

/**
 *****************************************************************************************
 * @brief Delay the function execution.
 *
 * @param[in] us:  Microsecond.
 *****************************************************************************************
 */
void tim_delay_us(uint32_t us);

/**
 *****************************************************************************************
 * @brief Delay the function execution.
 *
 * @param[in] ms:  Millisecond.
 *****************************************************************************************
 */
void tim_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif // __GR551X_TIM_DELAY_H__

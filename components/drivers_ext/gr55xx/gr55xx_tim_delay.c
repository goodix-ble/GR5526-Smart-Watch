/**
 ****************************************************************************************
 *
 * @file gr551x_tim_delay.c
 *
 * @brief GR551x tim delay.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "gr551x_tim_delay.h"

/*
 * STATIC VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint32_t fus = 0;
static uint32_t fms = 0;
static dual_timer_regs_t *tim_regs;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void tim_delay_init(dual_timer_regs_t *timx)
{
    fus = SystemCoreClock / 1000000;
    fms = SystemCoreClock / 1000;
    tim_regs = timx;
    tim_regs->RELOAD = 0xFFFFFFFF;
    /* Disable tim, period mode, 32-bit counter, one-shot mdoe */
    tim_regs->CTRL   = 0x43;
}

void tim_delay_us(uint32_t us)
{
    uint32_t load = us * fus - 1;
    tim_regs->RELOAD = load;
    /* Enable tim */
    tim_regs->CTRL  |= 0x80;
    while(tim_regs->VALUE != 0);
    tim_regs->CTRL  &= ~0x80;
    /* Clear flag */
    tim_regs->INTCLR = 1;
}

void tim_delay_ms(uint32_t ms)
{
    uint32_t load = ms * fms - 1;
    tim_regs->RELOAD = load;
    /* Enable tim */
    tim_regs->CTRL  |= 0x80;
    while(tim_regs->VALUE != 0);
    tim_regs->CTRL  &= ~0x80;
    /* Clear flag */
    tim_regs->INTCLR = 1;
}

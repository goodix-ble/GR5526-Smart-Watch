/**
 *****************************************************************************************
 *
 * @file sensorsim.c
 *
 * @brief sensor simulator function Implementation.
 *
 *****************************************************************************************
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
#include "sensorsim.h"

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void sensorsim_init(sensorsim_state_t *p_state, const sensorsim_cfg_t *p_cfg)
{
    if (p_cfg->start_at_max)
    {
        p_state->current_val   = p_cfg->max;
        p_state->is_increasing = false;
    }
    else
    {
        p_state->current_val   = p_cfg->min;
        p_state->is_increasing = true;
    }
}

int16_t sensorsim_measure(sensorsim_state_t *p_state, const sensorsim_cfg_t *p_cfg)
{
    if (p_state->is_increasing)
    {
        sensorsim_increment(p_state, p_cfg);
    }
    else
    {
        sensorsim_decrement(p_state, p_cfg);
    }

    return p_state->current_val;
}


void sensorsim_increment(sensorsim_state_t *p_state, const sensorsim_cfg_t *p_cfg)
{
    if (p_cfg->max - p_state->current_val > p_cfg->incr)
    {
        p_state->current_val += p_cfg->incr;
    }
    else
    {
        p_state->current_val   = p_cfg->max;
        p_state->is_increasing = false;
    }
}

void sensorsim_decrement(sensorsim_state_t *p_state, const sensorsim_cfg_t *p_cfg)
{
    if (p_state->current_val - p_cfg->min > p_cfg->incr)
    {
        p_state->current_val -= p_cfg->incr;
    }
    else
    {
        p_state->current_val   = p_cfg->min;
        p_state->is_increasing = true;
    }
}


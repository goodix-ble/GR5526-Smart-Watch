/**
 *****************************************************************************************
 *
 * @file sensorsim.h
 *
 * @brief Header file - sensor simulator
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

#ifndef _SENSORSIM_H__
#define _SENSORSIM_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup SENSORSIM_STRUCT Structures
 * @{
 */
/**@brief Triangular waveform sensor simulator configuration. */
typedef struct
{
    int16_t min;            /**< Minimum simulated value. */
    int16_t max;            /**< Maximum simulated value. */
    int16_t incr;           /**< Increment between each measurement. */
    bool    start_at_max;   /**< TRUE is measurement is to start at the maximum value, FALSE if it is to start at the minimum. */
} sensorsim_cfg_t;

/**@brief Triangular waveform sensor simulator state. */
typedef struct
{
    int16_t current_val;    /**< Current sensor value. */
    bool    is_increasing;  /**< TRUE if the simulator is in increasing state, FALSE otherwise. */
} sensorsim_state_t;
/** @} */


/**
 * @defgroup SENSORSIM_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Function for initializing a triangular waveform sensor simulator.
 *
 * @param[out] p_state: Current state of simulator.
 * @param[in]  p_cfg:   Simulator configuration.
 *****************************************************************************************
 */
void sensorsim_init(sensorsim_state_t *p_state, const sensorsim_cfg_t *p_cfg);

/**
 *****************************************************************************************
 * @brief Function for generating a simulated sensor measurement using a triangular waveform generator.
 *
 * @param[in,out] p_state: Current state of simulator.
 * @param[in]     p_cfg:   Simulator configuration.
 *
 * @retval ::Simulator output.
 *****************************************************************************************
 */
int16_t sensorsim_measure(sensorsim_state_t *p_state, const sensorsim_cfg_t *p_cfg);

/**
 *****************************************************************************************
 * @brief Function for incrementing a simulated sensor measurement value.
 *
 * @param[in,out] p_state: Current state of simulator.
 * @param[in]     p_cfg:   Simulator configuration.
 *
 * @retval         Simulator output.
 *****************************************************************************************
 */
void sensorsim_increment(sensorsim_state_t *p_state, const sensorsim_cfg_t *p_cfg);

/**
 *****************************************************************************************
 * @brief Function for decrementing a simulated sensor measurement value.
 *
 * @param[in,out] p_state: Current state of simulator.
 * @param[in]     p_cfg:   Simulator configuration.
 *
 * @retval         Simulator output.
 *****************************************************************************************
 */
void sensorsim_decrement(sensorsim_state_t *p_state, const sensorsim_cfg_t *p_cfg);
/** @} */

#endif


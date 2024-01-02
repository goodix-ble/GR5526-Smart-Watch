/**
 ****************************************************************************************
 *
 * @file    hal_gfx_easing.h
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

/** @addtogroup HAL_GFX HAL GFX
  * @{
  */

/** @defgroup HAL_GFX_EASING GFX EASING
 * @brief The interfaces of GPU easing configration.
 * @{
 */


#ifndef HAL_GFX_EASING_H__
#define HAL_GFX_EASING_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GFX_EASING_FUNCTION Functions
 * @{
 */

//Linear
// Modeled after the line y = x
/**
 *****************************************************************************************
 * @brief Linear easing, no acceleration
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_linear(float p);

//Quadratic

// Modeled after the parabola y = x^2
/**
 *****************************************************************************************
 * @brief Quadratic easing in, accelerate from zero
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_quad_in(float p);

// Modeled after the parabola y = -x^2 + 2x
/**
 *****************************************************************************************
 * @brief Quadratic easing out, decelerate to zero velocity
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_quad_out(float p);

// Modeled after the piecewise quadratic
// y = (1/2)((2x)^2)             ; [0, 0.5)
// y = -(1/2)((2x-1)*(2x-3) - 1) ; [0.5, 1]
/**
 *****************************************************************************************
 * @brief Quadratic easing in and out, accelerate to halfway, then decelerate
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_quad_in_out(float p);

//Cubic

// Modeled after the cubic y = x^3
/**
 *****************************************************************************************
 * @brief Cubic easing in, accelerate from zero
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_cub_in(float p);

// Modeled after the cubic y = (x - 1)^3 + 1
/**
 *****************************************************************************************
 * @brief Cubic easing out, decelerate to zero velocity
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_cub_out(float p);

// Modeled after the piecewise cubic
// y = (1/2)((2x)^3)       ; [0, 0.5)
// y = (1/2)((2x-2)^3 + 2) ; [0.5, 1]
/**
 *****************************************************************************************
 * @brief Cubic easing in and out, accelerate to halfway, then decelerate
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_cub_in_out(float p);

//Quartic

// Modeled after the quartic x^4
/**
 *****************************************************************************************
 * @brief Quartic easing in, accelerate from zero
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_quar_in(float p);

// Modeled after the quartic y = 1 - (x - 1)^4
/**
 *****************************************************************************************
 * @brief Quartic easing out, decelerate to zero velocity
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_quar_out(float p);

// Modeled after the piecewise quartic
// y = (1/2)((2x)^4)        ; [0, 0.5)
// y = -(1/2)((2x-2)^4 - 2) ; [0.5, 1]
/**
 *****************************************************************************************
 * @brief Quartic easing in and out, accelerate to halfway, then decelerate
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_quar_in_out(float p);

//Quintic

// Modeled after the quintic y = x^5
/**
 *****************************************************************************************
 * @brief Quintic easing in, accelerate from zero
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_quin_in(float p);

// Modeled after the quintic y = (x - 1)^5 + 1
/**
 *****************************************************************************************
 * @brief Quintic easing out, decelerate to zero velocity
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_quin_out(float p);

// Modeled after the piecewise quintic
// y = (1/2)((2x)^5)       ; [0, 0.5)
// y = (1/2)((2x-2)^5 + 2) ; [0.5, 1]
/**
 *****************************************************************************************
 * @brief Quintic easing in and out, accelerate to halfway, then decelerate
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_quin_in_out(float p);

//Sin

// Modeled after quarter-cycle of sine wave
/**
 *****************************************************************************************
 * @brief Sinusoidal easing in, accelerate from zero
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_sin_in(float p);

// Modeled after quarter-cycle of sine wave (different phase)
/**
 *****************************************************************************************
 * @brief Sinusoidal easing out, decelerate to zero velocity
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_sin_out(float p);

// Modeled after half sine wave
/**
 *****************************************************************************************
 * @brief Sinusoidal easing in and out, accelerate to halfway, then decelerate
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_sin_in_out(float p);

//Circular

// Modeled after shifted quadrant IV of unit circle
/**
 *****************************************************************************************
 * @brief Circular easing in, accelerate from zero
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_circ_in(float p);

// Modeled after shifted quadrant II of unit circle
/**
 *****************************************************************************************
 * @brief Circular easing out, decelerate to zero velocity
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_circ_out(float p);

// Modeled after the piecewise circular function
// y = (1/2)(1 - sqrt(1 - 4x^2))           ; [0, 0.5)
// y = (1/2)(sqrt(-(2x - 3)*(2x - 1)) + 1) ; [0.5, 1]
/**
 *****************************************************************************************
 * @brief Circular easing in and out, accelerate to halfway, then decelerate
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_circ_in_out(float p);

//Exponential

// Modeled after the exponential function y = 2^(10(x - 1))
/**
 *****************************************************************************************
 * @brief Exponential easing in, accelerate from zero
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_exp_in(float p);

// Modeled after the exponential function y = -2^(-10x) + 1
/**
 *****************************************************************************************
 * @brief Exponential easing out, decelerate to zero velocity
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_exp_out(float p);

// Modeled after the piecewise exponential
// y = (1/2)2^(10(2x - 1))         ; [0,0.5)
// y = -(1/2)*2^(-10(2x - 1))) + 1 ; [0.5,1]
/**
 *****************************************************************************************
 * @brief Exponential easing in and out, accelerate to halfway, then decelerate
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_exp_in_out(float p);

//Elastic
// Modeled after the damped sine wave y = sin(13pi/2*x)*pow(2, 10 * (x - 1))

/**
 *****************************************************************************************
 * @brief Elastic easing in, accelerate from zero
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_elast_in(float p);

// Modeled after the damped sine wave y = sin(-13pi/2*(x + 1))*pow(2, -10x) + 1
/**
 *****************************************************************************************
 * @brief Elastic easing out, decelerate to zero velocity
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_elast_out(float p);

// Modeled after the piecewise exponentially-damped sine wave:
// y = (1/2)*sin(13pi/2*(2*x))*pow(2, 10 * ((2*x) - 1))      ; [0,0.5)
// y = (1/2)*(sin(-13pi/2*((2x-1)+1))*pow(2,-10(2*x-1)) + 2) ; [0.5, 1]
/**
 *****************************************************************************************
 * @brief Elastic easing in and out, accelerate to halfway, then decelerate
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_elast_in_out(float p);

//Back

// Modeled after the overshooting cubic y = x^3-x*sin(x*pi)
/**
 *****************************************************************************************
 * @brief Overshooting easing in, accelerate from zero
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_back_in(float p);

// Modeled after overshooting cubic y = 1-((1-x)^3-(1-x)*sin((1-x)*pi))
/**
 *****************************************************************************************
 * @brief Overshooting easing out, decelerate to zero velocity
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_back_out(float p);

// Modeled after the piecewise overshooting cubic function:
// y = (1/2)*((2x)^3-(2x)*sin(2*x*pi))           ; [0, 0.5)
// y = (1/2)*(1-((1-x)^3-(1-x)*sin((1-x)*pi))+1) ; [0.5, 1]
/**
 *****************************************************************************************
 * @brief Overshooting easing in and out, accelerate to halfway, then decelerate
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_back_in_out(float p);

//Bounce

/**
 *****************************************************************************************
 * @brief Bouncing easing in, accelerate from zero
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_bounce_out(float p);

/**
 *****************************************************************************************
 * @brief Bouncing easing out, decelerate to zero velocity
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_bounce_in(float p);

/**
 *****************************************************************************************
 * @brief Bouncing easing in and out, accelerate to halfway, then decelerate
 *
 * @param[in] p: Input value, typically within the [0, 1] range
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez_bounce_in_out(float p);

/**
 *****************************************************************************************
 * @brief Convenience function to perform easing between two values given number of steps, current step and easing function
 *
 * @param[in] A:         Initial value within range [0, 1]
 * @param[in] B:         Finale value within range [0, 1]
 * @param[in] steps:     Total number of steps
 * @param[in] cur_step:  Current Step
 * @param[in] ez_func:   pointer to the desired easing function
 *
 * @return Eased value
 *****************************************************************************************
 */
float hal_gfx_ez(float A, float B, float steps, float cur_step, float (*ez_func)(float p));
/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */


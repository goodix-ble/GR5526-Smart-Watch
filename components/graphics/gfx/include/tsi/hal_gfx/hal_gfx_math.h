/**
 ****************************************************************************************
 *
 * @file    hal_gfx_math.h
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

/** @defgroup HAL_GFX_MATH GFX MATH
  * @brief GPU base math interfaces.
  * @{
  */

#ifndef HAL_GFX_MATH_H__
#define HAL_GFX_MATH_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GFX_MATH_MACRO Defines
 * @{
 */
#define HAL_GFX_E          2.71828182845904523536f  /**< e */
#define HAL_GFX_LOG2E      1.44269504088896340736f  /**< log2(e) */
#define HAL_GFX_LOG10E     0.434294481903251827651f /**< log10(e) */
#define HAL_GFX_LN2        0.693147180559945309417f /**< ln(2) */
#define HAL_GFX_LN10       2.30258509299404568402f  /**< ln(10) */
#define HAL_GFX_PI         3.14159265358979323846f  /**< pi */
#define HAL_GFX_PI_2       1.57079632679489661923f  /**< pi/2 */
#define HAL_GFX_PI_4       0.785398163397448309616f /**< pi/4 */
#define HAL_GFX_1_PI       0.318309886183790671538f /**< 1/pi */
#define HAL_GFX_2_PI       0.636619772367581343076f /**< 2/pi */
#define HAL_GFX_2_SQRTPI   1.12837916709551257390f  /**< 2/sqrt(pi) */
#define HAL_GFX_SQRT2      1.41421356237309504880f  /**< sqrt(2) */
#define HAL_GFX_SQRT1_2    0.707106781186547524401f /**< 1/sqrt(2) */

#define hal_gfx_min2(a,b)            (((a)<(b))?( a):(b))               /**< Find the minimum of two values */
#define hal_gfx_max2(a,b)            (((a)>(b))?( a):(b))               /**< Find the maximum of two values */
#define hal_gfx_clamp(val, min, max)  hal_gfx_min2((max), hal_gfx_max2((min), (val))) /**< Clamp value. */
#define hal_gfx_abs(a)               (((a)< 0 )?(-(a)):(a))             /**< Calculate the absolute value of int. */
#define hal_gfx_absf(a)              (((a)< 0.f )?(-(a)):(a))           /**< Calculate the absolute value of float. */
#define hal_gfx_floats_equal(x, y)   (hal_gfx_absf((x) - (y)) <= 0.00001f * hal_gfx_min2(hal_gfx_absf(x), hal_gfx_absf(y))) /**< Compare two floats. */
#define hal_gfx_float_is_zero(x)     (hal_gfx_absf(x) <= 0.00001f)      /**< Checks if value x is zero. */
#define hal_gfx_deg_to_rad(d)        (0.0174532925199f * (d))           /**< Convert degrees to radians. */
#define hal_gfx_rad_to_deg(r)        (57.295779513f * (r))              /**< onvert radians to degries. */
#define hal_gfx_i2fx(a)              ((a)*0x10000)                      /**< Convert integer to 16.16 fixed point. */
#define hal_gfx_floor(f)             ((int)(f) - ( (int)(f) > (f) ))    /**< Floor function. */
#define hal_gfx_ceil(f)              ((int)(f) + ( (int)(f) < (f) ))    /**< Ceiling function. */
/** @} */

/**
 * @defgroup HAL_GFX_MATH_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Fast sine approximation of a given angle
 *
 * @param[in] angle_degrees: Angle in degrees
 *
 * @return return Sine of the given angle
 *****************************************************************************************
 */
float hal_gfx_sin(float angle_degrees);

/**
 *****************************************************************************************
 * @brief Fast cosine approximation of a given angle
 *
 * @param[in] angle_degrees: Angle in degrees
 *
 * @return Cosine of the given angle
 *****************************************************************************************
 */
float hal_gfx_cos(float angle_degrees);

/**
 *****************************************************************************************
 * @brief Fast tangent approximation of a given angle
 *
 * @param[in] angle_degrees: Angle in degrees
 *
 * @return Tangent of the given angle
 *****************************************************************************************
 */
float hal_gfx_tan(float angle_degrees);

/**
 *****************************************************************************************
 * @brief A rough approximation of x raised to the power of y. USE WITH CAUTION!
 *
 * @param[in] x: base value. Must be non negative.
 * @param[in] y: power value
 *
 * @return The result of raising x to the power y
 *****************************************************************************************
 */
float hal_gfx_pow(float x, float y);

/**
 *****************************************************************************************
 * @brief A rough approximation of the square root of x. USE WITH CAUTION!
 *
 * @param[in] x: X value. Must be non negative
 *
 * @return The square root of x
 *****************************************************************************************
 */
float hal_gfx_sqrt(float x);

/**
 *****************************************************************************************
 * @brief A floating-point approximation of the inverse tangent of x
 *
 * @param[in] x: X value
 *
 * @return Inverse tangent (angle) of x in degrees
 *****************************************************************************************
 */
float hal_gfx_atan(float x);

/**
 *****************************************************************************************
 * @brief Convert float to 16.16 fixed point
 *
 * @param[in] f: Value to be converted
 *
 * @return 16.16 fixed point value
 *****************************************************************************************
 */
int hal_gfx_f2fx(float f); 
/** @} */
#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */


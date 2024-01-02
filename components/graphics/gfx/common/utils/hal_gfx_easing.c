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

#include "hal_gfx_math.h"
#include "hal_gfx_easing.h"



float hal_gfx_ez_linear(float p)
{
    return p;
}

float hal_gfx_ez_quad_in(float p)
{
    return p*p;
}

float hal_gfx_ez_cub_in(float p)
{
    return p*p*p;
}

float hal_gfx_ez_quar_in(float p)
{
    return p*p*p*p;
}

float hal_gfx_ez_quin_in(float p)
{
    return p*p*p*p*p;
}


//  Quadratic

float hal_gfx_ez_quad_out(float p)
{
    float f = (p - 1.f);
    return 1.f-(f*f);
}

float hal_gfx_ez_cub_out(float p)
{
    float f = p - 1.f;
    return (f * f * f) + 1.f;
}

float hal_gfx_ez_quar_out(float p)
{
    float f = (p - 1.f);
    return 1.f-(f * f * f * f);
}

float hal_gfx_ez_quin_out(float p)
{
    float f = (p - 1.f);
    return (f * f * f * f * f) + 1.f;
}

float hal_gfx_ez_quad_in_out(float p)
{
    if(p < 0.5f) {
        return 2.f* p * p;
    } else {
        float f = (p - 1.f);
        return 1.f- (2.f* f * f);
    }
}

//Cubic
float hal_gfx_ez_cub_in_out(float p)
{
    if(p < 0.5f) {
        return 4.f* p * p * p;
    } else {
        float f = (p - 1.f);
        return (4.f* f * f * f) + 1.f;
    }
}

float hal_gfx_ez_quar_in_out(float p)
{
    if(p < 0.5f) {
        return 8.f * p * p * p * p;
    } else {
        float f = (p - 1.f);
        return 1.f - (8.f * f * f * f * f);
    }
}

float hal_gfx_ez_quin_in_out(float p)
{
    if(p < 0.5f) {
        return 16.f * p * p * p * p * p;
    } else {
        float f = (p - 1.f);
        return (16.f * f * f * f * f * f) + 1.f;
    }
}


//Sin
float hal_gfx_ez_sin_in(float p)
{
    return hal_gfx_sin(hal_gfx_rad_to_deg((p - 1.f) * HAL_GFX_PI_2)) + 1.f;
}

float hal_gfx_ez_sin_out(float p)
{
    return hal_gfx_sin(hal_gfx_rad_to_deg(p * HAL_GFX_PI_2));
}

float hal_gfx_ez_sin_in_out(float p)
{
    return 0.5f* (1.f- hal_gfx_cos(hal_gfx_rad_to_deg(p * HAL_GFX_PI)));
}

//Circular
float hal_gfx_ez_circ_in(float p)
{
    return 1.f- hal_gfx_sqrt(1.f- (p * p));
}

float hal_gfx_ez_circ_out(float p)
{
    return hal_gfx_sqrt((2.f- p) * p);
}

float hal_gfx_ez_circ_in_out(float p)
{
    if(p < 0.5f) {
        return 0.5f* (1.f- hal_gfx_sqrt(1.f- (4.f* p * p)));
    } else {
        return 0.5f* (hal_gfx_sqrt(-((2.f* p) - 3.f) * ((2.f* p) - 1.f)) + 1.f);
    }
}

//Exponential
float hal_gfx_ez_exp_in(float p)
{
    return (p == 0.f) ? p : hal_gfx_pow(2.f, 10.f* (p - 1.f));
}

float hal_gfx_ez_exp_out(float p)
{
    return (p == 1.f) ? p : (1.f- hal_gfx_pow(2.f, -(10.f* p)));
}

float hal_gfx_ez_exp_in_out(float p)
{
    if( (p == 0.0f) || (p == 1.0f)) { return p; }

    if(p < 0.5f) {
        return  0.5f * hal_gfx_pow(2.f, ( 20.f * p) - 10.f);
    } else {
        return -0.5f * hal_gfx_pow(2.f, (-20.f * p) + 10.f) + 1.f;
    }
}


//Elastic
float hal_gfx_ez_elast_in(float p)
{
    float rad = 13.f* HAL_GFX_PI_2* p;
    float deg = hal_gfx_rad_to_deg(rad);
    float sin = hal_gfx_sin(deg);
    return sin * hal_gfx_pow(2.f, 10.f* (p - 1.f));
}

float hal_gfx_ez_elast_out(float p)
{
    float rad = -13.f* HAL_GFX_PI_2* (p + 1.f);
    float deg = hal_gfx_rad_to_deg(rad);
    float sin = hal_gfx_sin(deg);
    return sin * hal_gfx_pow(2.f, -10.f* p) + 1.f;
}

float hal_gfx_ez_elast_in_out(float p)
{
    if(p < 0.5f) {
        float rad = 13.f* HAL_GFX_PI_2* (2.f* p);
        float deg = hal_gfx_rad_to_deg(rad);
        float sin = hal_gfx_sin(deg);
        return 0.5f* sin * hal_gfx_pow(2.f, 10.f* ((2.f* p) - 1.f));
    } else {
        float rad = -13.f* HAL_GFX_PI_2* (((2.f* p) - 1.f) + 1.f);
        float deg = hal_gfx_rad_to_deg(rad);
        float sin = hal_gfx_sin(deg);
        return 0.5f* (sin * hal_gfx_pow(2.f, -10.f* ((2.f* p) - 1.f)) + 2.f);
    }
}


//Back
float hal_gfx_ez_back_in(float p)
{
    return (p * p * p) - (p * hal_gfx_sin(hal_gfx_rad_to_deg(p * HAL_GFX_PI)));
}

float hal_gfx_ez_back_out(float p)
{
    float f = (1.f- p);
    return 1.f - ((f * f * f) - (f * hal_gfx_sin(hal_gfx_rad_to_deg(f * HAL_GFX_PI))));
}

float hal_gfx_ez_back_in_out(float p)
{
    if(p < 0.5f) {
        float f = 2.f* p;
        return 0.5f* ((f * f * f) - (f * hal_gfx_sin(hal_gfx_rad_to_deg(f * HAL_GFX_PI))));
    } else {
        float f = (1.f- ((2.f*p) - 1.f));
        return (0.5f* (1.f- ((f * f * f) - (f * hal_gfx_sin(hal_gfx_rad_to_deg(f * HAL_GFX_PI)))))) + 0.5f;
    }
}


//Bounce
float hal_gfx_ez_bounce_out(float p)
{
    if(p < (4.f/11.0f)) {
        return (121.f* p * p)/16.0f;
    } else if(p < (8.f/11.0f)) {
        return (363.f/40.0f* p * p) - (99.f/10.0f* p) + (17.f/5.0f);
    } else if(p < (9.f/10.0f)) {
        return (4356.f/361.0f* p * p) - (35442.f/1805.0f* p) + (16061.f/1805.0f);
    } else {
        return (54.f/5.0f* p * p) - (513.f/25.0f* p) + (268.f/25.0f);
    }
}

float hal_gfx_ez_bounce_in(float p)
{
    return 1.f- hal_gfx_ez_bounce_out(1.f- p);
}

float hal_gfx_ez_bounce_in_out(float p)
{
    if(p < 0.5f) {
        return 0.5f* hal_gfx_ez_bounce_in(p*2.f);
    } else {
        return 0.5f* hal_gfx_ez_bounce_out((p * 2.f)- 1.f) + 0.5f;
    }
}


float hal_gfx_ez(float A, float B, float steps, float cur_step, float (*ez_func)(float p))
{
//    if (cur_step >= steps) return B;
//    if (cur_step <= 0.f  ) return A;

    float factor = cur_step/steps;
    factor = ez_func(factor);
    float res = A+(B-A)*factor;

    return res;
}



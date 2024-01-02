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

// A sine approximation via a fourth-order cosine approx.
// @param x   angle (degrees)
// @return     Sine value
float hal_gfx_sin(float angle_degrees)
{
    float xf = (angle_degrees*((float)0x8000/360.f));
    int x = (int)xf;

    int c, y;
    static const unsigned qN= 13U, qA= 12U, B=19900U, C=3516U;

    const unsigned _31_qn_mask0 = ((unsigned)1U<<(qN+1U))-1U;
    const unsigned _31_qn_mask1 = ~_31_qn_mask0;
    const unsigned _31_qn_mask_bit = (unsigned)1U << qN;
    const int _1qN = (int)((unsigned)1U<<qN);

    c  = (int)((unsigned)x<<(30U-qN));  // Semi-circle info into carry.
    x -= _1qN;                          // sine -> cosine calc

    unsigned x1;
    if ( ((unsigned)x & _31_qn_mask_bit ) != 0U ) {
        x1 = (unsigned)x | _31_qn_mask1;
    } else {
        x1 = (unsigned)x & _31_qn_mask0;
    }

    // Mask with PI
    // x  = x<<(31-qN);
    // Note: SIGNED shift! (to qN)f
    // x  = x>>(31-qN);

    x = (int)x1;
    int xx = x*x;
    x  = (int)((unsigned)(xx)>>(2U*qN-14U));   // x=x^2 To Q14

    unsigned xC = (((unsigned)x*C)>>14U);
    y  = (int)B - (int)xC;           // B - x^2*C
    int xy = x*y;
    unsigned y1  = ((unsigned)1U<<qA)-((unsigned)(xy)>>16U);       // A - x^2*(B-x^2*C)
    y = (int)y1;

    return ((c>=0 ? (float)y : -(float)y))/((float)0x1000);
}

// A cosine approximation via a fourth-order cosine approx.
// @param x   angle (degrees)
// @return     Cos value
float hal_gfx_cos(float angle_degrees)
{
    return hal_gfx_sin(angle_degrees+90.f);
}

// A tan approximation via a fourth-order tangent approx.
// @param x   angle (degrees)
// @return     Tan value
float hal_gfx_tan(float angle_degrees)
{
    return hal_gfx_sin(angle_degrees)/hal_gfx_cos(angle_degrees);
}

float hal_gfx_pow(float x, float y)
{
    union {
        float f;
        int i;
    } u;

    u.f = x;

    int val0 = u.i - 1065307417;
    float val1 = y*(float)val0;
    float val2 = val1 + 1065307417.f;
    u.i = (int)val2;

    return u.f;
}

float hal_gfx_sqrt(float x)
{
    union {
        float f;
        unsigned int i;
    } u;

    u.f = x;
    // adjust bias
    // u.i  += 127U << 23;
    u.i  += 0x3F800000U;
    // approximation of square root
    u.i >>= 1;
    return u.f;
 }

// A inverse tangent approximation if floating-point
// @param x   X value
// @return     Inverse tangent value (angle in degrees)
float hal_gfx_atan(float x)
{
    if(-1.f <= x && x <= 1.f) {  // |x| <= 1
        return ((57.2958f*x) / ((0.28f*x*x) + 1.f));
    }
    else if(x > 1.f) {   // x > 1
        return  (90.f - (57.2958f*x) / ((x*x) + 0.28f));
    }
    else {    // x < -1
        return -(90.f + (57.2958f*x) / ((x*x) + 0.28f));
    }
}

int hal_gfx_f2fx(float f) {
    float ffx = (f * (float)0x10000)+0.5f;

    return (int)ffx;
}

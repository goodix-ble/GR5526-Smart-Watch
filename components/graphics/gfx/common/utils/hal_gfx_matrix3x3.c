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

#include "hal_gfx_matrix3x3.h"
#include "hal_gfx_math.h"

// A B C
// D E F
// G H X

#define A m[0][0]
#define B m[0][1]
#define C m[0][2]
#define D m[1][0]
#define E m[1][1]
#define F m[1][2]
#define G m[2][0]
#define H m[2][1]
#define X m[2][2]

#define A_ _m[0][0]
#define B_ _m[0][1]
#define C_ _m[0][2]
#define D_ _m[1][0]
#define E_ _m[1][1]
#define F_ _m[1][2]
#define G_ _m[2][0]
#define H_ _m[2][1]
#define I_ _m[2][2]

void
hal_gfx_mat3x3_load_identity(hal_gfx_matrix3x3_t m)
{
    A = E = X = 1.f;
    B = C = D = F = G = H = 0.f;
}

void
hal_gfx_mat3x3_translate(hal_gfx_matrix3x3_t m, float tx, float ty)
{
    A += tx*G; B += tx*H; C += tx*X;
    D += ty*G; E += ty*H; F += ty*X;
}

void
hal_gfx_mat3x3_scale(hal_gfx_matrix3x3_t m, float sx, float sy)
{
    A *= sx; B *= sx; C *= sx;
    D *= sy; E *= sy; F *= sy;
}

void
hal_gfx_mat3x3_shear(hal_gfx_matrix3x3_t m, float shx, float shy)
{
    float a = A, b = B, c = C;
    A += D*shx; B += E*shx; C += F*shx;
    D += a*shy; E += b*shy; F += c*shy;
}

void
hal_gfx_mat3x3_mirror(hal_gfx_matrix3x3_t m, int mx, int my)
{
    if (mx != 0 || my != 0 ) {
        float sy = mx == 0 ? 1.f : -1.f;
        float sx = my == 0 ? 1.f : -1.f;
        hal_gfx_mat3x3_scale(m, sx, sy);
    }
}

void
hal_gfx_mat3x3_rotate(hal_gfx_matrix3x3_t m, float angle_degrees)
{
    float cosa = hal_gfx_cos(angle_degrees);
    float sina = hal_gfx_sin(angle_degrees);
    float a = A, b = B, c = C;

    A =  cosa*a - sina*D; B =  cosa*b - sina*E; C =  cosa*c - sina*F;
    D =  sina*a + cosa*D; E =  sina*b + cosa*E; F =  sina*c + cosa*F;
}

void
hal_gfx_mat3x3_mul(hal_gfx_matrix3x3_t m,
                hal_gfx_matrix3x3_t _m)
{
    float mA = A*A_ + B*D_ + C*G_;
    float mB = A*B_ + B*E_ + C*H_;
    float mC = A*C_ + B*F_ + C*I_;
    float mD = D*A_ + E*D_ + F*G_;
    float mE = D*B_ + E*E_ + F*H_;
    float mF = D*C_ + E*F_ + F*I_;
    float mG = G*A_ + H*D_ + X*G_;
    float mH = G*B_ + H*E_ + X*H_;
    float mI = G*C_ + H*F_ + X*I_;

    A = mA; B = mB; C = mC;
    D = mD; E = mE; F = mF;
    G = mG; H = mH; X = mI;
}

void
hal_gfx_mat3x3_mul_vec_affine(hal_gfx_matrix3x3_t m,
                           float *x,
                           float *y)
{
    float x0 = *x;
    float y0 = *y;

    *x = (A*x0+B*y0+C);
    *y = (D*x0+E*y0+F);
}

void
hal_gfx_mat3x3_mul_vec(hal_gfx_matrix3x3_t m,
                    float *x,
                    float *y)
{
    float x0 = *x;
    float y0 = *y;
    float divisor = G*x0+H*y0+X;

    *x = (A*x0+B*y0+C)/divisor;
    *y = (D*x0+E*y0+F)/divisor;
}

static float
hal_gfx_mat3x3_det(hal_gfx_matrix3x3_t m)
{
    return A*(X*E-F*H)-
           B*(X*D-F*G)+
           C*(H*D-E*G);
}

void
hal_gfx_mat3x3_adj(hal_gfx_matrix3x3_t m)
{
    float h[9];
    h[0] = E*X - H*F;    h[3] = H*C - B*X;    h[6] = B*F - E*C;
    h[1] = G*F - D*X;    h[4] = A*X - G*C;    h[7] = D*C - A*F;
    h[2] = D*H - G*E;    h[5] = G*B - A*H;    h[8] = A*E - D*B;

    A = h[0];    B = h[3];    C = h[6];
    D = h[1];    E = h[4];    F = h[7];
    G = h[2];    H = h[5];    X = h[8];
}

void
hal_gfx_mat3x3_div_scalar(hal_gfx_matrix3x3_t m, float s)
{
    s = 1.f/s;
    A *= s;  B *= s;  C *= s;
    D *= s;  E *= s;  F *= s;
    G *= s;  H *= s;  X *= s;
}


int
hal_gfx_mat3x3_invert(hal_gfx_matrix3x3_t m)
{
    float det = hal_gfx_mat3x3_det(m);

    if ( hal_gfx_float_is_zero(det) ) {
        return -1;
    }

    hal_gfx_mat3x3_adj(m);
    hal_gfx_mat3x3_div_scalar(m, det);
    return 0;
}

int
hal_gfx_mat3x3_square_to_quad(float dx0, float dy0,
                     float dx1, float dy1,
                     float dx2, float dy2,
                     float dx3, float dy3,
                     hal_gfx_matrix3x3_t m)
{
    float ax  = dx0 - dx1 + dx2 - dx3;
    float ay  = dy0 - dy1 + dy2 - dy3;

    if (hal_gfx_float_is_zero(ax) && hal_gfx_float_is_zero(ay)) {
        /* affine case */
        A = dx1 - dx0;    B = dx2 - dx1;  C = dx0;
        D = dy1 - dy0;    E = dy2 - dy1;  F = dy0;
        G = 0.f;          H = 0.f;        X = 1.f;
    } else {
        float ax1 = dx1 - dx2;
        float ax2 = dx3 - dx2;
        float ay1 = dy1 - dy2;
        float ay2 = dy3 - dy2;

        /* determinants */
        float gtop    =  ax  * ay2 - ax2 * ay;
        float htop    =  ax1 * ay  - ax  * ay1;
        float bottom  =  ax1 * ay2 - ax2 * ay1;

        if ( hal_gfx_float_is_zero(bottom) ) {
            return -1;
        }

        bottom = 1.f/bottom;

        G = gtop * bottom;
        H = htop * bottom;

        A = dx1 - dx0 + G * dx1;
        B = dx3 - dx0 + H * dx3;
        C = dx0;
        D = dy1 - dy0 + G * dy1;
        E = dy3 - dy0 + H * dy3;
        F = dy0;
        X = 1.f;
    }

    return 0;
}

static int
hal_gfx_mat3x3_quad_to_square(float sx0, float sy0,
                      float sx1, float sy1,
                      float sx2, float sy2,
                      float sx3, float sy3,
                      hal_gfx_matrix3x3_t m)
{
    if (hal_gfx_mat3x3_square_to_quad(sx0, sy0,
                                   sx1, sy1,
                                   sx2, sy2,
                                   sx3, sy3,
                                   m) != 0) {
        return -1;
    }

    return hal_gfx_mat3x3_invert(m);
}

// static int
// hal_gfx_mat3x3_quad_to_quad(float dx0, float dy0,
//                          float dx1, float dy1,
//                          float dx2, float dy2,
//                          float dx3, float dy3,
//                          float sx0, float sy0,
//                          float sx1, float sy1,
//                          float sx2, float sy2,
//                          float sx3, float sy3,
//                          hal_gfx_matrix3x3_t m)
// {
//     hal_gfx_matrix3x3_t _m;
//     if (hal_gfx_mat3x3_quad_to_square(sx0, sy0,
//                               sx1, sy1,
//                               sx2, sy2,
//                               sx3, sy3,
//                               _m) != 0)
//     {
//         return -1;
//     }

//     if (hal_gfx_mat3x3_square_to_quad(dx0, dy0,
//                               dx1, dy1,
//                               dx2, dy2,
//                               dx3, dy3,
//                               m) != 0)
//     {
//         return -1;
//     }

//     hal_gfx_mat3x3_mul(m, _m);

//     return 0;
// }

int
hal_gfx_mat3x3_quad_to_rect(int width, int height,
                    float sx0, float sy0,
                    float sx1, float sy1,
                    float sx2, float sy2,
                    float sx3, float sy3,
                    hal_gfx_matrix3x3_t m)
{
    if (hal_gfx_mat3x3_quad_to_square(sx0, sy0,
                              sx1, sy1,
                              sx2, sy2,
                              sx3, sy3,
                              m) != 0)
    {
        return -1;
    }

    //multiply first 2 rows by width and height
    hal_gfx_mat3x3_scale(m, (float)width, (float)height);

    return 0;
}


#undef A
#undef B
#undef C
#undef D
#undef E
#undef F
#undef G
#undef H
#undef X

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

#include "hal_gfx_matrix4x4.h"
#include "hal_gfx_math.h"

// A B C D
// E F G H
// X J K L
// M N O P

#define A m[0][0]
#define B m[0][1]
#define C m[0][2]
#define D m[0][3]
#define E m[1][0]
#define F m[1][1]
#define G m[1][2]
#define H m[1][3]
#define X m[2][0]
#define J m[2][1]
#define K m[2][2]
#define L m[2][3]
#define M m[3][0]
#define N m[3][1]
#define O m[3][2]
#define P m[3][3]

// #define A_ m_[0][0]
// #define B_ m_[0][1]
// #define C_ m_[0][2]
// #define D_ m_[0][3]
// #define E_ m_[1][0]
// #define F_ m_[1][1]
// #define G_ m_[1][2]
// #define H_ m_[1][3]
// #define I_ m_[2][0]
// #define J_ m_[2][1]
// #define K_ m_[2][2]
// #define L_ m_[2][3]
// #define M_ m_[3][0]
// #define N_ m_[3][1]
// #define O_ m_[3][2]
// #define P_ m_[3][3]

void
hal_gfx_mat4x4_load_identity(hal_gfx_matrix4x4_t m)
{
    A = F = K = P = 1.f;
    B = C = D = E = G = H = X = J = L = M = N = O = 0.f;
}

void
hal_gfx_mat4x4_mul(hal_gfx_matrix4x4_t  m,
                hal_gfx_matrix4x4_t  m_l,
                hal_gfx_matrix4x4_t  m_r)
{
    hal_gfx_matrix4x4_t k;
    int y;
    for (y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
            k[y][x] = 0.f;
            for (int z = 0; z < 4; z++) {
                k[y][x] += m_l[y][z]*m_r[z][x];
            }
        }
    }

    for (y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
            m[y][x] = k[y][x];
        }
    }
}

void
hal_gfx_mat4x4_mul_vec(hal_gfx_matrix4x4_t m,
                    float *x,
                    float *y,
                    float *z,
                    float *w)
{
    float _x, _y, _z;

    _x = A * (*x) + B * (*y) + C * (*z) + D * (*w);
    _y = E * (*x) + F * (*y) + G * (*z) + H * (*w);
    _z = X * (*x) + J * (*y) + K * (*z) + L * (*w);
    *w = M * (*x) + N * (*y) + O * (*z) + P * (*w);

    *x = _x; *y = _y; *z = _z;
}

//         | 1  0  0  tx |   | A B C D |
//     m = | 0  1  0  ty | * | E F G H |
//         | 0  0  1  tz |   | X J K L |
//         | 0  0  0  1  |   | M N O P |
void
hal_gfx_mat4x4_translate(hal_gfx_matrix4x4_t m, float tx, float ty, float tz)
{
//    hal_gfx_matrix4x4_t m_;
//    hal_gfx_mat4x4_load_identity(m_);
//    D_ = tx;
//    H_ = ty;
//    L_ = tz;
//
//    hal_gfx_mat4x4_mul(m, m_, m,);

    A += M*tx; B += N*tx; C += O*tx; D += P*tx;
    E += M*ty; F += N*ty; G += O*ty; H += P*ty;
    X += M*tz; J += N*tz; K += O*tz; L += P*tz;
}

//         | sx 0  0  0 |   | A B C D |
//     m = | 0  sy 0  0 | * | E F G H |
//         | 0  0  sz 0 |   | X J K L |
//         | 0  0  0  1 |   | M N O P |
void
hal_gfx_mat4x4_scale(hal_gfx_matrix4x4_t m, float sx, float sy, float sz)
{
//    hal_gfx_matrix4x4_t m_;
//    hal_gfx_mat4x4_load_identity(m_);
//    A_ = sx;
//    F_ = sy;
//    K_ = sz;
//
//    hal_gfx_mat4x4_mul(m, m_, m,);

    A *= sx; B *= sx; C *= sx; D *= sx;
    E *= sy; F *= sy; G *= sy; H *= sy;
    X *= sz; J *= sz; K *= sz; L *= sz;
}

//         |  1  0       0       0 |   | A B C D |
//     m = |  0  cos(A) -sin(A)  0 | * | E F G H |
//         |  0  sin(A)  cos(A)  0 |   | X J K L |
//         |  0  0       0       1 |   | M N O P |
void
hal_gfx_mat4x4_rotate_X(hal_gfx_matrix4x4_t m, float angle_degrees)
{
    float cosa = hal_gfx_cos(angle_degrees);
    float sina = hal_gfx_sin(angle_degrees);

//    hal_gfx_matrix4x4_t m_;
//    hal_gfx_mat4x4_load_identity(m_);
//    F_ = cosa; _G = -sina;
//    J_ = sina; _K =  cosa;
//
//    hal_gfx_mat4x4_mul(m, m_, m);


    float e = E, f = F, g = G, h = H;

    E =  cosa*e - sina*X; F =  cosa*f - sina*J; G =  cosa*g - sina*K;  H =  cosa*h - sina*L;
    X =  sina*e + cosa*X; J =  sina*f + cosa*J; K =  sina*g + cosa*K;  L =  sina*h + cosa*L;
}

//         |  cos(A)  0   sin(A)  0 |   | A B C D |
//     m = |  0       1   0       0 | * | E F G H |
//         | -sin(A)  0   cos(A)  0 |   | X J K L |
//         |  0       0   0       1 |   | M N O P |
void
hal_gfx_mat4x4_rotate_Y(hal_gfx_matrix4x4_t m, float angle_degrees)
{
    float cosa = hal_gfx_cos(angle_degrees);
    float sina = hal_gfx_sin(angle_degrees);

//    hal_gfx_matrix4x4_t m_;
//    hal_gfx_mat4x4_load_identity(m_);
//    A_ = cosa; _C =  sina;
//    I_ =-sina; _K =  cosa;
//
//    hal_gfx_mat4x4_mul(m, m_, m);

    float a = A, b = B, c = C, d = D;

    A =  cosa*a + sina*X; B =  cosa*b + sina*J; C =  cosa*c + sina*K;  D =  cosa*d + sina*L;
    X = -sina*a + cosa*X; J = -sina*b + cosa*J; K = -sina*c + cosa*K;  L = -sina*d + cosa*L;
}

//         |  cos(A)  -sin(A)   0   0 |   | A B C D |
//     m = |  sin(A)   cos(A)   0   0 | * | E F G H |
//         |  0        0        1   0 |   | X J K L |
//         |  0        0        0   1 |   | M N O P |
void
hal_gfx_mat4x4_rotate_Z(hal_gfx_matrix4x4_t m, float angle_degrees)
{
    float cosa = hal_gfx_cos(angle_degrees);
    float sina = hal_gfx_sin(angle_degrees);

//    hal_gfx_matrix4x4_t m_;
//    hal_gfx_mat4x4_load_identity(m_);
//    A_ = cosa; _B = -sina;
//    E_ = sina; _F =  cosa;
//
//    hal_gfx_mat4x4_mul(m, m_, m);

    float a = A, b = B, c = C, d = D;

    A =  cosa*a - sina*E; B =  cosa*b - sina*F; C =  cosa*c - sina*G;  D =  cosa*d - sina*H;
    E =  sina*a + cosa*E; F =  sina*b + cosa*F; G =  sina*c + cosa*G;  H =  sina*d + cosa*H;
}

//www.opengl.org/sdk/docs/man2/xhtml/gluPerspective.xml
//but negative K and O
void
hal_gfx_mat4x4_load_perspective(hal_gfx_matrix4x4_t m, float fovy_degrees, float aspect,
                             float nearVal, float farVal)
{
    B = C = D = E = G = H = X = J = M = N = P = 0.f;

    float fovy_2 = fovy_degrees*0.5f;

    //cot(fovy_degrees/2)
    float cos = hal_gfx_cos(fovy_2);
    float sin = hal_gfx_sin(fovy_2);
    float f = cos/sin;

    A = f/aspect;
    F = f;
    K = (farVal+nearVal)/(farVal-nearVal); //negative compared to OpenGL
    L = (2.f*farVal*nearVal)/(nearVal-farVal);
    O = 1.f; //negative compared to OpenGL
}

//www.opengl.org/sdk/docs/man2/xhtml/glOrtho.xml
void hal_gfx_mat4x4_load_ortho(hal_gfx_matrix4x4_t m,
                       float left,    float right,
                       float bottom,  float top,
                       float nearVal, float farVal)
{
    B = C = E = G = X = J = M = N = O = 0.f;
    P = 1.f;

    A = 2.f/(right-left);
    F = 2.f/(top-bottom);
    K =-2.f/(farVal-nearVal);

    //tx
    D = -(right  + left   )/(right  - left   );
    //ty
    H = -(top    + bottom )/(top    - bottom );
    //tz
    L = -(farVal + nearVal)/(farVal - nearVal);

}

//www.opengl.org/sdk/docs/man2/xhtml/gluOrtho2D.xml
void hal_gfx_mat4x4_load_ortho_2d(hal_gfx_matrix4x4_t m,
                          float left,   float right,
                          float bottom, float top)
{
    hal_gfx_mat4x4_load_ortho(m, left, right, bottom, top, -1.f, 1.f);
}

int
hal_gfx_mat4x4_obj_to_win_coords(hal_gfx_matrix4x4_t mvp,
                              float x_orig,  float y_orig,
                              int width,     int   height,
                              float nearVal, float farVal,
                              float *x,
                              float *y,
                              float *z,
                              float *w)
{
    // float _w = *w;
    *w = 1.f;
    int clip = 0;

    //calculate Clip Coordinates
    hal_gfx_mat4x4_mul_vec(mvp, x, y, z, w);

    float abs_w = hal_gfx_absf(*w);

    if ( hal_gfx_absf(*x) <= abs_w &&
         hal_gfx_absf(*y) <= abs_w &&
         hal_gfx_absf(*z) <= abs_w) {
        clip = 0;
    }
    else {
        clip = 1;
    }

    //calculate NDC (Normalized Device Coordinates)
    (*x) /= (*w);
    (*y) /= (*w);
    (*z) /= (*w);

    //calculate Window Coordinates
    float width_2  = 0.5f*(float)width;
    float height_2 = 0.5f*(float)height;

    *x = width_2 *(*x) + x_orig + width_2;
    *y = height_2*(*y) + y_orig + height_2;
    *z = 0.5f*((farVal-nearVal)*(*z) + farVal + nearVal);
    // *w = _w;

    return clip;
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
#undef J
#undef K
#undef L
#undef M
#undef N
#undef O
#undef P


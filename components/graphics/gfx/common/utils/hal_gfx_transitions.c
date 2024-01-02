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

#include "hal_gfx_graphics.h"
#include "hal_gfx_matrix4x4.h"
#include "hal_gfx_transitions.h"

void hal_gfx_transition_linear_hor(hal_gfx_tex_t left, hal_gfx_tex_t right,
                                uint32_t blending_mode, float step,
                                int width)
{
    float x_offset_f = step*(float)width;
    int x_offset = (int)x_offset_f;

    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, left, HAL_GFX_NOTEX);
    hal_gfx_blit(x_offset-width, 0);

    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, right, HAL_GFX_NOTEX);
    hal_gfx_blit(x_offset, 0);
}

void hal_gfx_transition_cube_hor(hal_gfx_tex_t left, hal_gfx_tex_t right,
                              uint32_t blending_mode, float step,
                              int width, int height)
{
    hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
    hal_gfx_fill_rect(0, 0, width, height, 0);

    float box_size_2 = 0.5f;
    float FoV = 53.1301023542f;

    /*
     *             4
     *            /|
     *    0_____1/ |
     *    |     |  |
     *    |     | /5
     *    |_____|/
     *    3     2
     *
     */
                   //x     y    z
    float v[18] = {-box_size_2, -box_size_2,-box_size_2,   //0
                    box_size_2, -box_size_2,-box_size_2,   //1
                    box_size_2,  box_size_2,-box_size_2,   //2
                   -box_size_2,  box_size_2,-box_size_2,   //3
                    box_size_2, -box_size_2, box_size_2,   //4
                    box_size_2,  box_size_2, box_size_2};  //5

    step  = 1.f - step;
    float angle = step*90.f;

    //projection
    hal_gfx_matrix4x4_t mvp;

    hal_gfx_mat4x4_load_perspective(mvp, FoV, 1.f, 1.f, 100.f);
    mvp[0][0] = mvp[1][1] = 2.f;

    hal_gfx_matrix4x4_t proj;
    hal_gfx_mat4x4_load_identity(proj);

    hal_gfx_mat4x4_rotate_Y(proj, angle);
    hal_gfx_mat4x4_translate(proj, 0.f, 0.f, 1.5f);
    hal_gfx_mat4x4_mul(mvp, mvp, proj);

    float w = 1.f;

    for (int i = 0; i <= 15; i+=3) {
        (void)hal_gfx_mat4x4_obj_to_win_coords(mvp, 0.f, 0.f, width, height,
                                      1.f, 100.f,
                                      &v[i  ], &v[i+1], &v[i+2], &w);
    }

    if (v[3] < v[12]) {
        hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, right, HAL_GFX_NOTEX);
        hal_gfx_blit_quad_fit(v[ 3], v[ 4],
                           v[12], v[13],
                           v[15], v[16],
                           v[ 6], v[ 7]);
    }

    if (v[0] < v[3]) {
        hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, left, HAL_GFX_NOTEX);
        hal_gfx_blit_quad_fit(v[ 0], v[ 1],
                           v[ 3], v[ 4],
                           v[ 6], v[ 7],
                           v[ 9], v[10]);
    }
}




void hal_gfx_transition_cube_ver(hal_gfx_tex_t up, hal_gfx_tex_t down,
                              uint32_t blending_mode, float step,
                              int width, int height)
{
    hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
    hal_gfx_fill_rect(0, 0, width, height, 0);

    float box_size_2 = 0.5f;
    float FoV = 53.1301023542f;


    /*
     *
     *
     *    0_____1
     * 4 _|___5 |
     *  \ |   \ |
     *   \|_____|
     *    3     2
     *
     */
                   //x     y    z
    float v[18] = {-box_size_2, -box_size_2,-box_size_2,   //0
                    box_size_2, -box_size_2,-box_size_2,   //1
                    box_size_2,  box_size_2,-box_size_2,   //2
                   -box_size_2,  box_size_2,-box_size_2,   //3
                   -box_size_2,  box_size_2, box_size_2,   //4
                    box_size_2,  box_size_2, box_size_2};  //5

    step  = 1.f - step;
    float angle = step*90.f;

    //projection
    hal_gfx_matrix4x4_t mvp;

    hal_gfx_mat4x4_load_perspective(mvp, FoV, 1.f, 1.f, 100.f);
    mvp[0][0] = mvp[1][1] = 2.f;

    hal_gfx_matrix4x4_t proj;
    hal_gfx_mat4x4_load_identity(proj);

    hal_gfx_mat4x4_rotate_X(proj, -angle);
    hal_gfx_mat4x4_translate(proj, 0.f, 0.f, 1.5f);
    hal_gfx_mat4x4_mul(mvp, mvp, proj);

    float w = 1.f;

    for (int i = 0; i <= 15; i+=3) {
        (void)hal_gfx_mat4x4_obj_to_win_coords(mvp, 0.f, 0.f, width, height,
                                      1.f, 100.f,
                                      &v[i  ], &v[i+1], &v[i+2], &w);
    }

    if (v[7] < v[13])
    {
        hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, down, HAL_GFX_NOTEX);
        hal_gfx_blit_quad_fit(v[ 9], v[10],  // 3
                           v[ 6], v[ 7],  // 2
                           v[15], v[16],  // 5
                           v[12], v[13]); // 4
    }

    if (v[1] < v[7])
    {
        hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, up, HAL_GFX_NOTEX);
        hal_gfx_blit_quad_fit(v[ 0], v[ 1],  // 0
                           v[ 3], v[ 4],  // 1
                           v[ 6], v[ 7],  // 2
                           v[ 9], v[10]); // 3
    }
}

void hal_gfx_transition_innercube_hor(hal_gfx_tex_t left, hal_gfx_tex_t right,
                                   uint32_t blending_mode, float step,
                                   int width, int height)
{
    float wf = (float)width;
    float hf = (float)height;
    if (step == 1.f) {
        hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, left, HAL_GFX_NOTEX);
        hal_gfx_blit_quad_fit(0.f, 0.f, wf, 0.f, wf, hf, 0.f, hf);
        return;
    } else if (step == 0.f) {
        hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, right, HAL_GFX_NOTEX);
        hal_gfx_blit_quad_fit(0.f, 0.f, wf, 0.f, wf, hf, 0.f, hf);
        return;
    } else {
        //misra
    }

    step = 1.f-step;

    hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
    hal_gfx_fill_rect(0, 0, width, height, 0);

    float box_size_2 = 0.25f;
    float FoV = 28.0724869359f;


    /*
     *
     *
     *    0_____1
     *    |     |\
     *    |     | \ 4
     *    |_____|  |
     *    3     2\ |
     *            \|
     *              5
     *
     */

                   //x     y    z
    float v[18] = {-box_size_2, -box_size_2, box_size_2,   //0
                    box_size_2, -box_size_2, box_size_2,   //1
                    box_size_2,  box_size_2, box_size_2,   //2
                   -box_size_2,  box_size_2, box_size_2,   //3
                    box_size_2, -box_size_2,-box_size_2,   //4
                    box_size_2,  box_size_2,-box_size_2};  //5

    float angle = -step*90.f;

    //projection
    hal_gfx_matrix4x4_t mvp;

    hal_gfx_mat4x4_load_perspective(mvp, FoV, 1.f, 1.f, 100.f);

    hal_gfx_matrix4x4_t proj;
    hal_gfx_mat4x4_load_identity(proj);

    hal_gfx_mat4x4_rotate_Y(proj, angle);
    hal_gfx_mat4x4_translate(proj, 0.f, 0.f, 1.f-box_size_2);
    hal_gfx_mat4x4_mul(mvp, mvp, proj);

    float w = 1.f;

    for (int i = 0; i <= 15; i+=3) {
        (void)hal_gfx_mat4x4_obj_to_win_coords(mvp, 0.f, 0.f, width, height,
                                      1.f, 100.f,
                                      &v[i  ], &v[i+1], &v[i+2], &w);
    }

    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, left, HAL_GFX_NOTEX);
    hal_gfx_blit_quad_fit(v[ 0], v[ 1],
                       v[ 3], v[ 4],
                       v[ 6], v[ 7],
                       v[ 9], v[10]);

    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, right, HAL_GFX_NOTEX);
    hal_gfx_blit_quad_fit(v[ 3], v[ 4],
                       v[12], v[13],
                       v[15], v[16],
                       v[ 6], v[ 7]);
}

void hal_gfx_transition_innercube_ver(hal_gfx_tex_t up, hal_gfx_tex_t down,
                                   uint32_t blending_mode, float step,
                                   int width, int height)
{
    float wf = (float)width;
    float hf = (float)height;
    if (step == 1.f) {
        hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, up, HAL_GFX_NOTEX);
        hal_gfx_blit_quad_fit(0.f, 0.f, wf, 0.f, wf, hf, 0.f, hf);
        return;
    } else if (step == 0.f) {
        hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, down, HAL_GFX_NOTEX);
        hal_gfx_blit_quad_fit(0.f, 0.f, wf, 0.f, wf, hf, 0.f, hf);
        return;
    } else {
        //misra
    }

    step = 1.f-step;

    hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
    hal_gfx_fill_rect(0, 0, width, height, 0);

    float box_size_2 = 0.25f;
    float FoV = 28.0724869359f;


    /*
     *
     *
     *    0_____1
     *    |     |
     *    |  L  |
     *    |_____|2
     *    3\  R  \
     *      \_____\
     *      4      5
     *
     */

                   //x     y    z
    float v[18] = {-box_size_2, -box_size_2, box_size_2,   //0
                    box_size_2, -box_size_2, box_size_2,   //1
                    box_size_2,  box_size_2, box_size_2,   //2
                   -box_size_2,  box_size_2, box_size_2,   //3
                   -box_size_2,  box_size_2,-box_size_2,   //4
                    box_size_2,  box_size_2,-box_size_2};  //5

    float angle = -step*90.f;

    //projection
    hal_gfx_matrix4x4_t mvp;

    hal_gfx_mat4x4_load_perspective(mvp, FoV, 1.f, 1.f, 100.f);

    hal_gfx_matrix4x4_t proj;
    hal_gfx_mat4x4_load_identity(proj);

    hal_gfx_mat4x4_rotate_X(proj, -angle);
    hal_gfx_mat4x4_translate(proj, 0.f, 0.f, 1.f-box_size_2);
    hal_gfx_mat4x4_mul(mvp, mvp, proj);

    float w = 1.f;

    for (int i = 0; i <= 15; i+=3) {
        (void)hal_gfx_mat4x4_obj_to_win_coords(mvp, 0.f, 0.f, width, height,
                                      1.f, 100.f,
                                      &v[i  ], &v[i+1], &v[i+2], &w);
    }

    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, up, HAL_GFX_NOTEX);
    hal_gfx_blit_quad_fit(v[ 0], v[ 1],
                       v[ 3], v[ 4],
                       v[ 6], v[ 7],
                       v[ 9], v[10]);

    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, down, HAL_GFX_NOTEX);
    hal_gfx_blit_quad_fit(v[ 9], v[10],    // 3
                       v[ 6], v[ 7],    // 2
                       v[15], v[16],    // 5
                       v[12], v[13]);   // 4
}

void hal_gfx_transition_stack_hor(hal_gfx_tex_t left, hal_gfx_tex_t right, float step,
                                int width, int height)
{
    hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
    hal_gfx_fill_rect(0, 0, width, height, 0);

    float alpha_f = step*255.f;
    uint32_t left_alpha  = step < 0.f ? (uint32_t)0U : step > 1.f ? (uint32_t)255U : (uint32_t)(alpha_f);
    uint32_t right_alpha = 255U-left_alpha;

    hal_gfx_set_const_color(right_alpha <<24);
    hal_gfx_set_blend(HAL_GFX_BL_SIMPLE | HAL_GFX_BLOP_MODULATE_A, HAL_GFX_TEX0, right, HAL_GFX_NOTEX);

    step  = 1.f - step;

    if (step > 0.f) {
        float w = (step*(float)width + (float)width )*0.5f;
        float h = (step*(float)height+ (float)height)*0.5f;
        float x = ((float)width -w)*0.5f;
        float y = ((float)height-h)*0.5f;

        hal_gfx_blit_rect_fit((int)x, (int)y, (int)w, (int)h);
    }

    if (step < 1.f) {
        hal_gfx_set_const_color(left_alpha <<24);
        hal_gfx_set_blend(HAL_GFX_BL_SIMPLE | HAL_GFX_BLOP_MODULATE_A, HAL_GFX_TEX0, left, HAL_GFX_NOTEX);
        float x_offset_f = (1.f - step)*(float)width;
        int x_offset = (int)x_offset_f;
        hal_gfx_blit(x_offset-width, 0);
    }
}

void hal_gfx_transition_stack_ver(hal_gfx_tex_t up, hal_gfx_tex_t down, float step,
                                int width, int height)
{
    hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
    hal_gfx_fill_rect(0, 0, width, height, 0);

    float alpha_f = step*255.f;
    uint32_t up_alpha  = step < 0.f ? (uint32_t)0U : step > 1.f ? (uint32_t)255U : (uint32_t)(alpha_f);
    uint32_t down_alpha = 255U-up_alpha;

    hal_gfx_set_const_color(down_alpha <<24);
    hal_gfx_set_blend(HAL_GFX_BL_SIMPLE | HAL_GFX_BLOP_MODULATE_A, HAL_GFX_TEX0, down, HAL_GFX_NOTEX);

    step  = 1.f - step;

    if (step > 0.f) {
        float w = (step*(float)width + (float)width)*0.5f ;
        float h = (step*(float)height+ (float)height)*0.5f;
        float x = ((float)width -w)*0.5f;
        float y = ((float)height-h)*0.5f;

        hal_gfx_blit_rect_fit((int)x, (int)y, (int)w, (int)h);
    }

    if (step < 1.f) {
        hal_gfx_set_const_color(up_alpha <<24);
        hal_gfx_set_blend(HAL_GFX_BL_SIMPLE | HAL_GFX_BLOP_MODULATE_A, HAL_GFX_TEX0, up, HAL_GFX_NOTEX);
        float y_offset_f = (1.f-step)*(float)height;
        int y_offset = (int)y_offset_f;
        hal_gfx_blit(0, y_offset - height);
    }
}

void hal_gfx_transition_fade(hal_gfx_tex_t initial, hal_gfx_tex_t final, uint32_t blending_mode, float step,
                            int width, int height)
{
    hal_gfx_set_blend_fill(blending_mode);
    hal_gfx_fill_rect(0, 0, width, height, 0);

    float alpha_f = step*255.f;
    uint32_t initial_alpha  = step < 0.f ? (uint32_t)0U : step > 1.f ? (uint32_t)255U : (uint32_t)(alpha_f);

    hal_gfx_set_const_color(initial_alpha <<24);
    hal_gfx_set_blend(HAL_GFX_BL_SIMPLE | HAL_GFX_BLOP_MODULATE_A, HAL_GFX_TEX0, initial, final);
    hal_gfx_blit_rect_fit(0, 0, width, height);
}

void hal_gfx_transition_linear_ver(hal_gfx_tex_t up, hal_gfx_tex_t down, uint32_t blending_mode, float step,
                                int height)
{
    float y_offset_f = step*(float)height;
    int y_offset = (int)y_offset_f;
    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, up, HAL_GFX_NOTEX);
    hal_gfx_blit(0, y_offset - height);
    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, down, HAL_GFX_NOTEX);
    hal_gfx_blit(0, y_offset);
}

void hal_gfx_transition_fade_zoom(hal_gfx_tex_t initial, hal_gfx_tex_t final, uint32_t blending_mode, float step,
                               int width, int height)
{
    hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
    hal_gfx_fill_rect(0, 0, width, height, 0);

    float alpha_f = step*255.f;
    uint32_t initial_alpha  = step < 0.f ? (uint32_t)0U : step > 1.f ? (uint32_t)255U : (uint32_t)(alpha_f);
    uint32_t final_alpha    = 255U-initial_alpha;

    step = 1.f - step;
    //Render initial tex
    hal_gfx_set_const_color(initial_alpha <<24);
    hal_gfx_set_blend(HAL_GFX_BL_SIMPLE | HAL_GFX_BLOP_MODULATE_A, HAL_GFX_TEX0, initial, HAL_GFX_NOTEX);
    float w = (1.f - step)*(float)width;
    float h = (1.f - step)*(float)height;
    float x = step*(float)width*0.5f;
    float y = step*(float)height*0.5f;
    hal_gfx_blit_rect_fit((int)x, (int)y, (int)w, (int)h);

    //Render final tex
    hal_gfx_set_const_color(final_alpha <<24);
    hal_gfx_set_blend(HAL_GFX_BL_SIMPLE | HAL_GFX_BLOP_MODULATE_A, HAL_GFX_TEX0, final, HAL_GFX_NOTEX);
    w = step*(float)width;
    h = step*(float)height;
    x = (1.f - step)*(float)width*0.5f;
    y = (1.f - step)*(float)height*0.5f;
    hal_gfx_blit_rect_fit((int)x, (int)y, (int)w, (int)h);
}

/* New Added */
void hal_gfx_transition_spin_hor(hal_gfx_tex_t front, hal_gfx_tex_t back,
                              uint32_t blending_mode, float step,
                              int width, int height, bool is_clockwise)
{
    hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
    hal_gfx_fill_rect(0, 0, width, height, 0);

    float box_size_2 = 0.5f;
    float FoV = 53.1301023542f;

    /*************************************************
     *    Coordinate Axis
     *
     *            7 (Z)
     *           *
     *          *
     *         *
     *        *
     *       *
     *      **************************> (X)
     *      *
     *      *
     *      *
     *      *
     *      *
     *      *
     *      *
     *      * (Y)
     *      V
     *
     *************************************************
     *
     *  Flip around Y-Axis
     *
     **** ClickWise Flip around Y-Axis
     *
     *    0_____1          0 /| 1            1_____0
     *    |     |           / |              |     |
     *    |     |  ------> |  |    ------>   |     |
     *    |_____|          |  |              |_____|
     *    3     2          3\ |              2     3
     *                       \| 2
     *
     *
     **** Counter-ClickWise Flip around Y-Axis
     *
     *    0_____1         0|\              1_____0
     *    |     |          | \ 1           |     |
     *    |     | ------>  |  |   ------>  |     |
     *    |_____|          |  |            |_____|
     *    3     2          | / 2           2     3
     *                    3|/
     *
     *
     *************************************************
     */
                   //x     y    z (z = 0)
    float v[12] = {-box_size_2, -box_size_2, 0,   //0
                    box_size_2, -box_size_2, 0,   //1
                    box_size_2,  box_size_2, 0,   //2
                   -box_size_2,  box_size_2, 0,   //3
                  };

    bool is_show_front = (step <= 0.5) ? true : false;

    float angle = step*180.f;

    if(is_clockwise) {
        // Turn clockwise around the Y-axis
        if(angle <= 90.0) {
            angle = angle;
        } else {
            angle = 180 + angle;
        }
    } else {
        // Turn counterclockwise around the Y-axis
        if(angle <= 90.0) {
            angle = 180 - angle;
        } else {
            angle = 360 - angle;
        }
    }

    //projection
    hal_gfx_matrix4x4_t mvp;

    hal_gfx_mat4x4_load_perspective(mvp, FoV, 1.f, 1.f, 100.f);
    mvp[0][0] = mvp[1][1] = 2.f;

    hal_gfx_matrix4x4_t proj;
    hal_gfx_mat4x4_load_identity(proj);

    hal_gfx_mat4x4_rotate_Y(proj, angle);
    hal_gfx_mat4x4_translate(proj, 0.f, 0.f, 1.0f);
    hal_gfx_mat4x4_mul(mvp, mvp, proj);

    float w = 1.f;

    for (int i = 0; i <= 9; i+=3) {
        (void)hal_gfx_mat4x4_obj_to_win_coords(mvp, 0.f, 0.f, width, height,
                                      1.f, 100.f,
                                      &v[i  ], &v[i+1], &v[i+2], &w);
    }

    if (!is_clockwise || (v[0] > v[3])) {  /* is_clockwise must be false */
        if(is_show_front) {
            hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, front, HAL_GFX_NOTEX);
        } else {
            hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, back, HAL_GFX_NOTEX);
        }

        hal_gfx_blit_quad_fit(v[ 3], v[ 4],
                           v[ 0], v[ 1],
                           v[ 9], v[10],
                           v[ 6], v[ 7]
                           );
    } else if (is_clockwise || (v[0] < v[3])) { /* is_clockwise must be true */
        if(is_show_front) {
            hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, front, HAL_GFX_NOTEX);
        } else {
            hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, back, HAL_GFX_NOTEX);
        }
        hal_gfx_blit_quad_fit(v[ 0], v[ 1],
                           v[ 3], v[ 4],
                           v[ 6], v[ 7],
                           v[ 9], v[10]);
    }
}

static float cubic_bezier(float t, float p0, float p1, float p2, float p3)
{
    float u = 1.f - t;
    float tt = t * t;
    float uu = u * u;
    float uuu = uu * u;
    float ttt = tt * t;

    float res = uuu * p0; // first term
    res += 3.f * uu * t * p1; // second term
    res += 3.f * u * tt * p2; // third term
    res += ttt * p3; // fourth term

    return res;
}

void hal_gfx_transition_pushpull_hor(hal_gfx_tex_t initial,
                                     hal_gfx_tex_t final,
                                     uint32_t blending_mode,
                                     float step,
                                     int width,
                                     int height)
{
    hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
    hal_gfx_fill_rect(0, 0, width, height, 0);
    // Use bezier curve to simulate the DoF effect, no actual 3D math happened.
    // initial
    float scale = cubic_bezier(step, 0.5f, 0.25f, 1.f, 1.f);
    int ww = scale * width;
    int hh = scale * height;
    int xofs = (step - 0.5f) * width - ww / 2;
    int yofs = (height - hh) / 2;
    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, initial, HAL_GFX_NOTEX);
    hal_gfx_blit_rect_fit(xofs, yofs, ww, hh);

    // final
    scale = cubic_bezier(1.f - step, 0.5f, 0.25f, 1.f, 1.f);
    ww = scale * width;
    hh = scale * height;
    xofs = (step + 0.5f) * width - ww / 2;
    yofs = (height - hh) / 2;
    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, final, HAL_GFX_NOTEX);
    hal_gfx_blit_rect_fit(xofs, yofs, ww, hh);
}

void hal_gfx_transition_pushpull_ver(hal_gfx_tex_t initial,
                                     hal_gfx_tex_t final,
                                     uint32_t blending_mode,
                                     float step,
                                     int width,
                                     int height)
{
    hal_gfx_set_blend_fill(HAL_GFX_BL_SRC);
    hal_gfx_fill_rect(0, 0, width, height, 0);
    // Use bezier curve to simulate the DoF effect, no actual 3D math happened.
    // initial
    float scale = cubic_bezier(step, 0.5f, 0.25f, 1.f, 1.f);
    int ww = scale * width;
    int hh = scale * height;
    // int xofs = (step - 0.5f) * width - ww / 2;
    int xofs = (width - ww) / 2;
    // int yofs = (height - hh) / 2;
    int yofs = (step - 0.5f) * height - hh / 2;
    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, initial, HAL_GFX_NOTEX);
    hal_gfx_blit_rect_fit(xofs, yofs, ww, hh);

    // final
    scale = cubic_bezier(1.f - step, 0.5f, 0.25f, 1.f, 1.f);
    ww = scale * width;
    hh = scale * height;
    // xofs = (step + 0.5f) * width - ww / 2;
    xofs = (width - ww) / 2;
    // yofs = (height - hh) / 2;
    yofs = (step + 0.5f) * height - hh / 2;
    hal_gfx_set_blend(blending_mode, HAL_GFX_TEX0, final, HAL_GFX_NOTEX);
    hal_gfx_blit_rect_fit(xofs, yofs, ww, hh);
}

void hal_gfx_transition(hal_gfx_transition_t effect,
                     hal_gfx_tex_t initial,
                     hal_gfx_tex_t final,
                     uint32_t blending_mode,
                     float step,
                     int   width,
                     int   height)
{
    switch(effect) {
    case HAL_GFX_TRANS_MAX:
    case HAL_GFX_TRANS_NONE:
        break;
    case HAL_GFX_TRANS_CUBE_H:
        hal_gfx_transition_cube_hor(initial, final, blending_mode, step, width, height);
        break;
    case HAL_GFX_TRANS_CUBE_V:
        hal_gfx_transition_cube_ver(initial, final, blending_mode, step, width, height);
        break;
    case HAL_GFX_TRANS_INNERCUBE_H:
        hal_gfx_transition_innercube_hor(initial, final, blending_mode, step, width, height);
        break;
    case HAL_GFX_TRANS_INNERCUBE_V:
        hal_gfx_transition_innercube_ver(initial, final, blending_mode, step, width, height);
        break;
    case HAL_GFX_TRANS_STACK_H:
        hal_gfx_transition_stack_hor(initial, final, step, width, height);
        break;
    case HAL_GFX_TRANS_STACK_V:
        hal_gfx_transition_stack_ver(initial, final, step, width, height);
        break;
    case HAL_GFX_TRANS_FADE:
        hal_gfx_transition_fade(initial, final, blending_mode, step, width, height);
        break;
    case HAL_GFX_TRANS_LINEAR_V:
        hal_gfx_transition_linear_ver(initial, final, blending_mode, step, height);
        break;
    case HAL_GFX_TRANS_FADE_ZOOM:
        hal_gfx_transition_fade_zoom(initial, final, blending_mode, step, width, height);
        break;
    case HAL_GFX_TRANS_SPIN_H_R2L:
        hal_gfx_transition_spin_hor(initial, final, blending_mode, step, width, height, true);
        break;
    case HAL_GFX_TRANS_SPIN_H_L2R:
        hal_gfx_transition_spin_hor(initial, final, blending_mode, step, width, height, false);
        break;
    case HAL_GFX_TRANS_PUSHPULL_H:
        hal_gfx_transition_pushpull_hor(initial, final, blending_mode, step, width, height);
        break;
    case HAL_GFX_TRANS_PUSHPULL_V:
        hal_gfx_transition_pushpull_ver(initial, final, blending_mode, step, width, height);
        break;
    case HAL_GFX_TRANS_LINEAR_H:
    default:
        hal_gfx_transition_linear_hor(initial, final, blending_mode, step, width);
        break;
    }
}

#ifndef _LV_GPU_DRAW_POLY_LINE_
#define _LV_GPU_DRAW_POLY_LINE_

#include "lvgl.h"

typedef struct { float x,y;} vec_t;
void lv_draw_polyline(vec_t *vector, uint32_t size_of_vector, uint32_t color);

#endif

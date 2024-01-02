#ifndef __LV_GPU_GR5526_H__
#define __LV_GPU_GR5526_H__

#include "lvgl.h"
#include "lv_hal_disp.h"
#include "lv_draw_sw.h"

#if LV_USE_GPU_GR552x > 0u

typedef struct { float x,y;} vec_t;

void lv_draw_gr5526_ctx_init(lv_disp_drv_t * drv, lv_draw_ctx_t * draw_ctx);

void lv_draw_gr5526_ctx_deinit(lv_disp_drv_t * drv, lv_draw_ctx_t * draw_ctx);

void lv_draw_gr5526_line(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_line_dsc_t * dsc, const lv_point_t * point1, const lv_point_t * point2);

lv_res_t lv_draw_gr5526_img(lv_draw_ctx_t * draw_ctx, const lv_draw_img_dsc_t * dsc, const lv_area_t * coords, const void * src);

void lv_draw_gr5526_arc(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_arc_dsc_t * dsc, const lv_point_t * center, uint16_t radius,  uint16_t start_angle, uint16_t end_angle);

void lv_draw_gr5526_rect(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_rect_dsc_t * draw_dsc, const lv_area_t * coords);

void lv_draw_gr5526_bg(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_rect_dsc_t * draw_dsc, const lv_area_t * coords);

void lv_draw_gr5526_polyline(vec_t *vector, uint32_t size_of_vector, uint32_t color);

void lv_draw_gr5526_polygon(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_rect_dsc_t * draw_dsc, const lv_point_t points[], uint16_t point_cnt);

void lv_draw_gr5526_label(lv_draw_ctx_t * draw_ctx, const lv_draw_label_dsc_t * dsc, const lv_area_t * coords, const char * txt, lv_draw_label_hint_t * hint);

void lv_draw_gr5526_letter(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_label_dsc_t * dsc,  const lv_point_t * pos_p, uint32_t letter);

#endif

#endif /* __LV_GPU_GR5526_H__ */

/**
 * @file lv_enhanced_scrollbar.h
 *
 */

#ifndef __LV_ENHANCED_SCROLLBAR_H__
#define __LV_ENHANCED_SCROLLBAR_H__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"
#include <stdint.h>

/*********************
 *      DEFINES
 *********************/

#define SCROLLBAR_TYPE_ARC      0
#define SCROLLBAR_TYPE_VERTICAL 1

/**********************
 *      TYPEDEFS
 **********************/

typedef struct
{
    lv_obj_t obj;
    lv_point_t center;
    lv_coord_t total_height;
    lv_coord_t ind_height;
    lv_coord_t pad_top;
    uint16_t ind_param;
    lv_obj_t *target_obj;
} lv_enhanced_scrollbar_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

lv_obj_t *lv_enhanced_scrollbar_create(lv_obj_t *parent);

void lv_enhanced_scrollbar_set_center(lv_obj_t *obj, lv_coord_t x, lv_coord_t y);

void lv_enhanced_scrollbar_set_target(lv_obj_t *obj, lv_obj_t *target);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __LV_ENHANCED_SCROLLBAR_H__

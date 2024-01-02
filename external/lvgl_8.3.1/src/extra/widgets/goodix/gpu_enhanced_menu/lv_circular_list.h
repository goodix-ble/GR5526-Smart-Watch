#ifndef __LV_CIRCULAR_LIST_H__
#define __LV_CIRCULAR_LIST_H__

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
#include <stdint.h>

#include "lvgl.h"
#include "lv_enhanced_item.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

typedef struct
{
    lv_obj_t obj;
    const lv_enhanced_menu_item_t *items;
    uint16_t item_count;
    lv_img_dsc_t **icons_cache;
    uint16_t focus_idx;

    struct
    {
        lv_coord_t hor_offset;
        lv_coord_t max_offset;
        lv_coord_t momentum;
        bool focused;
        lv_timer_t *scroll_tmr;
    } scroll;
} lv_circular_list_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

lv_obj_t *lv_circular_list_create(lv_obj_t *parent);

void lv_circular_list_set_items(lv_obj_t *obj, const lv_enhanced_menu_item_t items[], uint16_t item_count);

const lv_enhanced_menu_item_t *lv_circular_list_get_focused_item(lv_obj_t *obj);

lv_coord_t lv_circular_list_get_scroll_offset(lv_obj_t *obj);

void lv_circular_list_set_scroll_offset(lv_obj_t *obj, lv_coord_t offset, bool anim);

lv_dir_t lv_circular_list_is_on_edge(lv_obj_t *obj);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __LV_CIRCULAR_LIST_H__

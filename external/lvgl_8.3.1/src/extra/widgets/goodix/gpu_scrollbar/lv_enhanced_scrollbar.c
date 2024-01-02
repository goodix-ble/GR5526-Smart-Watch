/**
 * @file lv_enhanced_scrollbar.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_enhanced_scrollbar.h"
#include "lv_gpu_gr5526.h"
#include "hal_gfx_cmdlist.h"

#include <stdio.h>
/*********************
 *      DEFINES
 *********************/
#define MY_CLASS    &lv_enhanced_scrollbar_class
#define AS_BAR(obj) ((lv_enhanced_scrollbar_t *)obj)

#define AUTO_DETACH_ON_TARGET_DELETE 1

#define SCROLLBAR_TYPE SCROLLBAR_TYPE_ARC

#if SCROLLBAR_TYPE == SCROLLBAR_TYPE_ARC

// ARC scrollbar styles
#define ARC_SCROLLBAR_START_ANGLE 315
#define ARC_SCROLLBAR_END_ANGLE   405
#define ARC_SCROLLBAR_RADIUS      202

#elif SCROLLBAR_TYPE == SCROLLBAR_TYPE_VERTICAL

// Vertical scrollbar styles
#define VERT_SCROLLBAR_POS_X  350
#define VERT_SCROLLBAR_POS_Y  177
#define VERT_SCROLLBAR_LENGTH 100

#endif // SCROLLBAR_TYPE == SCROLLBAR_TYPE_ARC

// General Style
#define SCROLLBAR_IND_COLOR   lv_color_make(192, 192, 192)
#define SCROLLBAR_BG_COLOR    lv_color_make(32, 32, 32)
#define SCROLLBAR_WIDTH       6
/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_enhanced_scrollbar_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_enhanced_scrollbar_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_enhanced_scrollbar_event(const lv_obj_class_t *class_p, lv_event_t *evt);
static void draw_scrollbar(lv_event_t *evt);

#if AUTO_DETACH_ON_TARGET_DELETE
static void target_delete_cb(lv_event_t *event);
#endif // AUTO_DETACH_ON_TARGET_DELETE

/**********************
 *  STATIC VARIABLES
 **********************/
const lv_obj_class_t lv_enhanced_scrollbar_class = {
    .constructor_cb = lv_enhanced_scrollbar_constructor,
    .destructor_cb = lv_enhanced_scrollbar_destructor,
    .event_cb = lv_enhanced_scrollbar_event,
    .width_def = LV_PCT(100),
    .height_def = LV_PCT(100),
    .instance_size = sizeof(lv_enhanced_scrollbar_t),
    .base_class = &lv_obj_class,
};

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
extern const lv_obj_class_t lv_enhanced_list_class;
extern const lv_obj_class_t lv_enhanced_grid_class;

lv_obj_t *lv_enhanced_scrollbar_create(lv_obj_t *parent)
{
    LV_LOG_INFO("begin");
    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    return obj;
}

void lv_enhanced_scrollbar_set_center(lv_obj_t *obj, lv_coord_t x, lv_coord_t y)
{
    AS_BAR(obj)->center.x = x;
    AS_BAR(obj)->center.y = y;
}

void lv_enhanced_scrollbar_set_target(lv_obj_t *obj, lv_obj_t *target)
{
    AS_BAR(obj)->target_obj = target;
    AS_BAR(obj)->ind_height = 0;
#if AUTO_DETACH_ON_TARGET_DELETE
    if (target)
    {
        lv_obj_add_event_cb(target, target_delete_cb, LV_EVENT_DELETE, (void *)obj);
    }
#endif // AUTO_DETACH_ON_TARGET_DELETE
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_enhanced_scrollbar_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
    LV_TRACE_OBJ_CREATE("begin");
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_enhanced_scrollbar_t *bar = AS_BAR(obj);
    bar->center.x = DISP_HOR_RES / 2;
    bar->center.y = DISP_VER_RES / 2;
    bar->target_obj = NULL;
    LV_TRACE_OBJ_CREATE("finished");
}

static void lv_enhanced_scrollbar_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
}

static void lv_enhanced_scrollbar_event(const lv_obj_class_t *class_p, lv_event_t *evt)
{
    LV_UNUSED(class_p);
    lv_enhanced_scrollbar_t *bar = AS_BAR(evt->current_target);
    if (evt->code == LV_EVENT_DRAW_MAIN_BEGIN)
    {
        if (!bar->ind_height)
        {
            if (bar->target_obj)
            {
                if (bar->target_obj->class_p == &lv_enhanced_list_class)
                {
                    bar->ind_height = lv_obj_get_height(bar->target_obj);
                    bar->total_height = ((lv_enhanced_list_t *)(bar->target_obj))->scroll.max_offset_sb;
                    bar->pad_top = 0;
                }
                else if (bar->target_obj->class_p == &lv_enhanced_grid_class)
                {
                    // Only show vertical scroll for enhanced grid
                    bar->ind_height = lv_obj_get_height(bar->target_obj);
                    bar->total_height = -1 * ((lv_enhanced_grid_t *)(bar->target_obj))->scroll.ver_offset_max + DISP_HOR_RES;
                    bar->pad_top = ((lv_enhanced_grid_t *)(bar->target_obj))->scroll.ver_offset_min;
                }
                else
                {
                    bar->pad_top = lv_obj_get_style_pad_top(bar->target_obj, 0);
                    bar->ind_height = lv_obj_get_height(bar->target_obj);
                    bar->total_height = lv_obj_get_scroll_top(bar->target_obj) + lv_obj_get_scroll_bottom(bar->target_obj) - bar->pad_top - lv_obj_get_style_pad_bottom(bar->target_obj, 0) + bar->ind_height;
                }
                float ind_perc = (float)bar->ind_height / (float)bar->total_height;
#if SCROLLBAR_TYPE == SCROLLBAR_TYPE_ARC
                bar->ind_param = ind_perc * (ARC_SCROLLBAR_END_ANGLE - ARC_SCROLLBAR_START_ANGLE);
#elif SCROLLBAR_TYPE == SCROLLBAR_TYPE_VERTICAL
                bar->ind_param = ind_perc * VERT_SCROLLBAR_LENGTH;
#endif // SCROLLBAR_TYPE == SCROLLBAR_TYPE_ARC
            }
        }
    }
    else if (evt->code == LV_EVENT_DRAW_POST)
    {
        draw_scrollbar(evt);
    }
    else if (evt->code == LV_EVENT_COVER_CHECK)
    {
        lv_cover_check_info_t *info = lv_event_get_param(evt);
        info->res = LV_COVER_RES_NOT_COVER;
    }
}

static void draw_scrollbar(lv_event_t *evt)
{
    lv_enhanced_scrollbar_t *bar = AS_BAR(evt->current_target);

    if (!bar->target_obj)
    {
        return;
    }

    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(evt);

    // Calc indicator angle
    int32_t scroll_offset;
    if (bar->target_obj->class_p == &lv_enhanced_list_class)
    {
        scroll_offset = -((lv_enhanced_list_t *)bar->target_obj)->scroll.ver_offset;
    }
    else if (bar->target_obj->class_p == &lv_enhanced_grid_class)
    {
        scroll_offset = -((lv_enhanced_grid_t *)bar->target_obj)->scroll.ver_offset;
    }
    else
    {
        scroll_offset = -bar->target_obj->spec_attr->scroll.y - bar->pad_top;
    }

#if SCROLLBAR_TYPE == SCROLLBAR_TYPE_ARC
    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);
    arc_dsc.width = SCROLLBAR_WIDTH;
    arc_dsc.rounded = 1;

    // Draw background
    arc_dsc.color = SCROLLBAR_BG_COLOR;
    lv_draw_gr5526_arc(draw_ctx, &arc_dsc, &bar->center, ARC_SCROLLBAR_RADIUS, ARC_SCROLLBAR_START_ANGLE, ARC_SCROLLBAR_END_ANGLE);

    uint16_t start_angle = ARC_SCROLLBAR_START_ANGLE + (ARC_SCROLLBAR_END_ANGLE - ARC_SCROLLBAR_START_ANGLE) * scroll_offset / bar->total_height;
    uint16_t end_angle = start_angle + bar->ind_param;

    start_angle = LV_MAX(start_angle, ARC_SCROLLBAR_START_ANGLE);
    end_angle = LV_MIN(end_angle, ARC_SCROLLBAR_END_ANGLE);

    // Only draw indicator if valid
    if (end_angle > start_angle)
    {
        // Draw Indicator
        arc_dsc.color = SCROLLBAR_IND_COLOR;
        lv_draw_gr5526_arc(draw_ctx, &arc_dsc, &bar->center, ARC_SCROLLBAR_RADIUS, start_angle, end_angle);
    }
#elif SCROLLBAR_TYPE == SCROLLBAR_TYPE_VERTICAL
    lv_point_t start = {VERT_SCROLLBAR_POS_X, VERT_SCROLLBAR_POS_Y};
    lv_point_t end = {VERT_SCROLLBAR_POS_X, VERT_SCROLLBAR_POS_Y + VERT_SCROLLBAR_LENGTH};

    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.width = SCROLLBAR_WIDTH;
    line_dsc.round_start = 1;
    line_dsc.round_end = 1;

    // Draw background
    line_dsc.color = SCROLLBAR_BG_COLOR;
    lv_draw_gr5526_line(draw_ctx, &line_dsc, &start, &end);

    uint16_t start_y = VERT_SCROLLBAR_POS_Y + VERT_SCROLLBAR_LENGTH * scroll_offset / bar->total_height;
    uint16_t end_y = start_y + bar->ind_param;

    start_y = LV_MAX(start_y, VERT_SCROLLBAR_POS_Y);
    end_y = LV_MIN(end_y, VERT_SCROLLBAR_POS_Y + VERT_SCROLLBAR_LENGTH);

    if (end_y > start_y)
    {
        line_dsc.color = SCROLLBAR_IND_COLOR;
        start.y = start_y;
        end.y = end_y;
        lv_draw_gr5526_line(draw_ctx, &line_dsc, &start, &end);
    }

#endif // SCROLLBAR_TYPE == SCROLLBAR_TYPE_ARC
}

#if AUTO_DETACH_ON_TARGET_DELETE
static void target_delete_cb(lv_event_t *event)
{
    lv_enhanced_scrollbar_set_target((lv_obj_t *)event->user_data, NULL);
}
#endif // AUTO_DETACH_ON_TARGET_DELETE

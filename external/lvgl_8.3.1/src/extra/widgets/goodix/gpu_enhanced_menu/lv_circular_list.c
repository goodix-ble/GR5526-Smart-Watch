/**
 * @file lv_circular_list.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_circular_list.h"
#include "hal_gfx_blender.h"
#include "hal_gfx_cmdlist.h"
#include "hal_gfx_graphics.h"
#include "hal_gfx_math.h"
#include "app_graphics_mem.h"

#include <stdio.h>
#include <math.h>

/*********************
 *      DEFINES
 *********************/
#define MY_CLASS      &lv_circular_list_class
#define AS_CLIST(obj) ((lv_circular_list_t *)(obj))

#define CLAMP(x, min, max) (x < min ? min : (x > max ? max : x))

// Factors are emperical magic, feel free to modify
#define SCROLL_THRESHOLD    15
#define ANGLE_PER_PIXEL     (90.f / DISP_HOR_RES)
#define FRICTION_FACTOR     0.95f
#define OUT_OF_BOUND_FACTOR 0.6f
#define ELASTIC_FACTOR      0.15f
#define SNAP_FACTOR         0.2f

// Style related params
#define CIRCULAR_LIST_RADIUS   ((int)DISP_HOR_RES / 2)
#define CIRCULAR_LIST_CENTER_X ((int)DISP_HOR_RES / 2)
#define CIRCULAR_LIST_CENTER_Y ((int)DISP_VER_RES - 10)
#define ANGULAR_SPACING        30
#define FOCUS_ZONE_MIN         (90 - 12)
#define FOCUS_ZONE_MAX         (90 + 12)
#define VISIBLE_ZONE_MIN       (90 - 60 - 10)
#define VISIBLE_ZONE_MAX       (90 + 60 + 10)
#define FOCUS_SCALE_INCREMENT  0.7f
#define MINIMUM_OPACITY        LV_OPA_20

// Feature settings
/**
 * @brief Indicates whether to cache all the icons into PSRAM.
 * Enabling this will consume some PSRAM space and will slightly increase the performance.
 */
#define ICON_CACHED_IN_PSRAM_ENABLED 0

/**
 * @brief Indicates whether to rotate icon along the circle.
 * Enabling will have medium impact on preformance.
 * PLEASE MAKE SURE that icons are all stored in PSRAM or enable @ref ICON_CACHED_IN_PSRAM_ENABLED!
 */
#define ICON_ROTATION_ENABLED 0

/**
 * @brief Indicates whether to snap list item to the 12 O'clock position.
 * Enabling this will have no influence on performance.
 */
#define SNAP_ENABLED 1

/**
 * @brief Indicates whether to place all the icons from left to right. By default icons are placed from right to left.
 * Enabling this will have no influnce on performance.
 */
#define ARRANGE_FROM_LEFT_TO_RIGHT 1

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_circular_list_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_circular_list_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_circular_list_event(const lv_obj_class_t *class_p, lv_event_t *evt);
static void draw_main(lv_event_t *evt);

#if ICON_CACHED_IN_PSRAM_ENABLED
static void build_icon_cache(lv_circular_list_t *list);
static void destroy_icon_cache(lv_circular_list_t *list);
#endif // ICON_CACHED_IN_PSRAM_ENABLED
static void calc_render_param(lv_circular_list_t *list);
static void scroll_timer_cb(lv_timer_t *p_timer);
static void handle_focus_change(lv_circular_list_t *list);

static void scroll_anim_exec_cb(void *var, int32_t val);
static void scroll_anim_ready_cb(lv_anim_t *anim);

static inline float regulate_angle(float angle);
static inline uint32_t img_format(const lv_img_dsc_t *img);

/**********************
 *  STATIC VARIABLES
 **********************/
const lv_obj_class_t lv_circular_list_class = {
    .constructor_cb = lv_circular_list_constructor,
    .destructor_cb = lv_circular_list_destructor,
    .event_cb = lv_circular_list_event,
    .width_def = LV_PCT(100),
    .height_def = LV_PCT(100),
    .instance_size = sizeof(lv_circular_list_t),
    .base_class = &lv_obj_class,
};

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
extern hal_gfx_cmdlist_t *lv_port_get_current_cl(void);
extern uint32_t lv_port_get_fb_format(void);
extern void lv_port_res_mode_set(uint8_t mode);

lv_obj_t *lv_circular_list_create(lv_obj_t *parent)
{
    LV_LOG_INFO("begin");
    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    return obj;
}

void lv_circular_list_set_items(lv_obj_t *obj, const lv_enhanced_menu_item_t items[], uint16_t item_count)
{
    AS_CLIST(obj)->items = items;
    AS_CLIST(obj)->item_count = item_count;

#if ICON_CACHED_IN_PSRAM_ENABLED
    if (AS_CLIST(obj)->icons_cache)
    {
        destroy_icon_cache(AS_CLIST(obj));
    }
    build_icon_cache(AS_CLIST(obj));
#endif // ICON_CACHED_IN_PSRAM_ENABLED

    calc_render_param(AS_CLIST(obj));
}

const lv_enhanced_menu_item_t *lv_circular_list_get_focused_item(lv_obj_t *obj)
{
    lv_circular_list_t *list = AS_CLIST(obj);
    if (list->focus_idx < list->item_count)
    {
        return &list->items[list->focus_idx];
    }
    return NULL;
}

lv_coord_t lv_circular_list_get_scroll_offset(lv_obj_t *obj)
{
    return AS_CLIST(obj)->scroll.hor_offset;
}

void lv_circular_list_set_scroll_offset(lv_obj_t *obj, lv_coord_t offset, bool anim)
{
    if (anim)
    {
        lv_anim_t a;
        lv_anim_init(&a);
        a.var = (void *)obj;
        a.path_cb = lv_anim_path_ease_in_out;
        a.start_value = AS_CLIST(obj)->scroll.hor_offset;
        a.end_value = offset;
        a.time = lv_anim_speed_to_time(750, AS_CLIST(obj)->scroll.hor_offset, offset);
        a.exec_cb = scroll_anim_exec_cb;
        a.ready_cb = scroll_anim_ready_cb;
        lv_anim_start(&a);
    }
    else
    {
        AS_CLIST(obj)->scroll.hor_offset = CLAMP(offset, 0, AS_CLIST(obj)->scroll.max_offset);
        handle_focus_change(AS_CLIST(obj));
    }
}

lv_dir_t lv_circular_list_is_on_edge(lv_obj_t *obj)
{
    if (AS_CLIST(obj)->scroll.hor_offset <= 0)
    {
        return LV_DIR_RIGHT;
    }
    else if (AS_CLIST(obj)->scroll.hor_offset >= AS_CLIST(obj)->scroll.max_offset)
    {
        return LV_DIR_LEFT;
    }
    return LV_DIR_NONE;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_circular_list_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
    LV_TRACE_OBJ_CREATE("begin");
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    lv_circular_list_t *list = AS_CLIST(obj);

    list->scroll.scroll_tmr = lv_timer_create(scroll_timer_cb, LV_INDEV_DEF_READ_PERIOD, list);
    lv_timer_pause(list->scroll.scroll_tmr);

    LV_TRACE_OBJ_CREATE("finished");
}

static void lv_circular_list_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
    lv_circular_list_t *list = AS_CLIST(obj);

    if (list->scroll.scroll_tmr)
    {
        lv_timer_del(list->scroll.scroll_tmr);
    }

#if ICON_CACHED_IN_PSRAM_ENABLED
    if (list->icons_cache)
    {
        destroy_icon_cache(list);
    }
#endif // ICON_CACHED_IN_PSRAM_ENABLED
}

static void lv_circular_list_event(const lv_obj_class_t *class_p, lv_event_t *evt)
{
    LV_UNUSED(class_p);

    lv_obj_t *obj = lv_event_get_current_target(evt);
    lv_event_code_t code = lv_event_get_code(evt);
    lv_circular_list_t *list = AS_CLIST(obj);

    if (code == LV_EVENT_PRESSED)
    {
        lv_anim_del((void *)list, scroll_anim_exec_cb);
        lv_timer_pause(list->scroll.scroll_tmr);
        list->scroll.focused = false;
        list->scroll.momentum = 0;
    }
    else if (code == LV_EVENT_PRESSING)
    {
        lv_indev_t *indev = lv_event_get_indev(evt);
        lv_coord_t diff = indev->proc.types.pointer.vect.x;
        if (list->scroll.hor_offset < 0 || list->scroll.hor_offset > list->scroll.max_offset)
        {
            diff = (float)diff * OUT_OF_BOUND_FACTOR;
        }
        list->scroll.hor_offset += diff;

        if (LV_ABS(list->scroll.hor_offset) > SCROLL_THRESHOLD)
        {
            list->scroll.focused = true;
        }

        if (list->scroll.focused)
        {
            handle_focus_change(list);
            lv_obj_invalidate(obj);
        }
        else
        {
            if (abs(indev->proc.types.pointer.vect.x) < abs(indev->proc.types.pointer.vect.y))
            {
                // Not going horizontal, do nothing
                return;
            }

            list->scroll.momentum += diff;
            if (LV_ABS(list->scroll.momentum) > SCROLL_THRESHOLD)
            {
                list->scroll.focused = true;
                indev->proc.types.pointer.scroll_obj = obj; // Prevent low-level scroll process affecting virtual scroll
            }
        }
    }
    else if (code == LV_EVENT_RELEASED)
    {
        lv_indev_t *indev = lv_event_get_indev(evt);
        list->scroll.momentum = indev->proc.types.pointer.vect.x;
        lv_timer_resume(list->scroll.scroll_tmr);
    }
    else if (code == LV_EVENT_CLICKED || code == LV_EVENT_SHORT_CLICKED)
    {
        // Only allow click on focused element
        lv_indev_t *indev = lv_event_get_indev(evt);
        bool hit = false;

        if (!list->scroll.focused)
        {
#if ARRANGE_FROM_LEFT_TO_RIGHT
            uint16_t focus_idx = list->item_count - 1 - list->focus_idx;
#else  // ARRANGE_FROM_LEFT_TO_RIGHT
            uint16_t focus_idx = list->focus_idx;
#endif // ARRANGE_FROM_LEFT_TO_RIGHT

#if ICON_CACHED_IN_PSRAM_ENABLED
            const lv_img_dsc_t *icon = list->icons_cache[focus_idx];
#else
            const lv_img_dsc_t *icon = list->items[focus_idx].icon;
#endif // ICON_CACHED_IN_PSRAM_ENABLED

            float angle_offset = list->scroll.hor_offset * ANGLE_PER_PIXEL;
            float icon_angle = regulate_angle(focus_idx * ANGULAR_SPACING - angle_offset + 90);

            float _center = (FOCUS_ZONE_MAX + FOCUS_ZONE_MIN) / 2;
            float _diff = fabsf(icon_angle - _center);
            float _scale = 1.f + (1 - _diff / (_center - FOCUS_ZONE_MIN)) * FOCUS_SCALE_INCREMENT;

            lv_point_t circle_pos = {
                .x = CIRCULAR_LIST_RADIUS * hal_gfx_cos(icon_angle),
                .y = -1 * CIRCULAR_LIST_RADIUS * hal_gfx_sin(icon_angle),
            };

            lv_point_t scaled_size = {
                .x = icon->header.w * _scale,
                .y = icon->header.h * _scale,
            };

            lv_area_t hitbox;
            hitbox.x1 = circle_pos.x + CIRCULAR_LIST_CENTER_X - scaled_size.x / 2;
            hitbox.y1 = circle_pos.y + CIRCULAR_LIST_CENTER_Y - scaled_size.y / 2;
            hitbox.x2 = hitbox.x1 + scaled_size.x;
            hitbox.y2 = hitbox.y1 + scaled_size.y;

            if (_lv_area_is_point_on(&hitbox, &indev->proc.types.pointer.act_point, 0))
            {
                hit = true;
            }
        }
        if (!hit)
        {
            _lv_event_mark_deleted(evt->current_target);
        }
    }
    else if (code == LV_EVENT_DRAW_MAIN)
    {
        draw_main(evt);
    }
}

static void draw_main(lv_event_t *evt)
{
    lv_obj_t *obj = lv_event_get_current_target(evt);
    lv_circular_list_t *list = AS_CLIST(obj);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(evt);

    if (list->item_count == 0)
    {
        return;
    }

    // prepare render environment
    hal_gfx_cmdlist_t *cl = lv_port_get_current_cl();
    void *disp_buf = draw_ctx->buf;
    const lv_area_t *disp_area = draw_ctx->buf_area;
    const lv_area_t *clip_area = draw_ctx->clip_area;
    hal_gfx_bind_dst_tex((uintptr_t)disp_buf,
                         disp_area->x2 - disp_area->x1 + 1,
                         disp_area->y2 - disp_area->y1 + 1,
                         lv_port_get_fb_format(),
                         -1);

    hal_gfx_set_clip(clip_area->x1, clip_area->y1, clip_area->x2 - clip_area->x1 + 1, clip_area->y2 - clip_area->y1 + 1);
    hal_gfx_clear(0xFF000000);

    // convert scroll offset to angle
    float angle_offset = list->scroll.hor_offset * ANGLE_PER_PIXEL;
    // float space_angle = list->styles.space_angle / 10.f;
    // lv_coord_t radius = list->styles.radius;
    lv_point_t center = {CIRCULAR_LIST_CENTER_X, CIRCULAR_LIST_CENTER_Y};

    // find start icon
    bool render_started = false;
    for (uint16_t i = 0; i < list->item_count; i++)
    {
#if ARRANGE_FROM_LEFT_TO_RIGHT
        uint16_t i_trans = (list->item_count - 1) - i;
#else // ARRANGE_FROM_LEFT_TO_RIGHT
        uint16_t i_trans = i;
#endif // ARRANGE_FROM_LEFT_TO_RIGHT
#if ICON_CACHED_IN_PSRAM_ENABLED
        const lv_img_dsc_t *icon = list->icons_cache[i_trans];
#else
        const lv_img_dsc_t *icon = list->items[i_trans].icon;
#endif // ICON_CACHED_IN_PSRAM_ENABLED
        float icon_angle = i * ANGULAR_SPACING;
        if (LV_ABS(icon_angle - angle_offset) > 180)
        {
            continue;
        }
        icon_angle = regulate_angle(icon_angle - angle_offset + 90);

        if (icon_angle > VISIBLE_ZONE_MAX || icon_angle < VISIBLE_ZONE_MIN)
        {
            if (render_started)
            {
                break;
            }
            else
            {
                continue;
            }
        }

        lv_point_t circle_pos = {
            .x = CIRCULAR_LIST_RADIUS * hal_gfx_cos(icon_angle),
            .y = -1 * CIRCULAR_LIST_RADIUS * hal_gfx_sin(icon_angle),
        };

        // Render
        if (!render_started)
        {
            render_started = true;
        }

        // Bind src tex
        hal_gfx_bind_src_tex((uintptr_t)icon->data, icon->header.w, icon->header.h, img_format(icon), -1, HAL_GFX_FILTER_BL);

        // Check if in focus zone
        if (icon_angle > FOCUS_ZONE_MIN && icon_angle < FOCUS_ZONE_MAX)
        {
            // In focus zone, calculate scale factor
            float _center = (FOCUS_ZONE_MAX + FOCUS_ZONE_MIN) / 2;
            float _diff = fabsf(icon_angle - _center);
            float _scale = 1.f + (1 - _diff / (_center - FOCUS_ZONE_MIN)) * FOCUS_SCALE_INCREMENT;

            hal_gfx_set_blend_blit(HAL_GFX_BL_SIMPLE);
#if ICON_ROTATION_ENABLED
            hal_gfx_blit_rotate_pivot_scale(circle_pos.x + center.x,
                                            circle_pos.y + center.y,
                                            icon->header.w / 2.f,
                                            icon->header.h / 2.f,
                                            regulate_angle(-1 * icon_angle + 90),
                                            _scale);
#else  // ICON_ROTATION_ENABLED
            lv_point_t scaled_size = {
                .x = icon->header.w * _scale,
                .y = icon->header.h * _scale,
            };
            hal_gfx_blit_rect_fit(circle_pos.x + center.x - scaled_size.x / 2,
                                  circle_pos.y + center.y - scaled_size.y / 2,
                                  scaled_size.x,
                                  scaled_size.y);
#endif // ICON_ROTATION_ENABLED
        }
        else
        {
            // No in focus zone, calculate alpha
            // LV_MAX() is right because one of the result will be negative, and the positive one is expected
            float _diff = LV_MAX(FOCUS_ZONE_MIN - icon_angle, icon_angle - FOCUS_ZONE_MAX);
            uint32_t alpha = MINIMUM_OPACITY;
            if (_diff < ANGULAR_SPACING)
            {
                alpha = LV_MAX(255 * (1 - _diff / ANGULAR_SPACING), alpha);
            }
            hal_gfx_set_blend_blit(HAL_GFX_BL_SIMPLE | HAL_GFX_BLOP_MODULATE_A);
            hal_gfx_set_const_color((alpha << 24) & 0xFF000000);
#if ICON_ROTATION_ENABLED
            hal_gfx_blit_rotate_pivot(circle_pos.x + center.x,
                                      circle_pos.y + center.y,
                                      icon->header.w / 2.f,
                                      icon->header.h / 2.f,
                                      regulate_angle(-1 * icon_angle + 90));
#else  // ICON_ROTATION_ENABLED
            hal_gfx_blit(circle_pos.x + center.x - icon->header.w / 2, circle_pos.y + center.y - icon->header.h / 2);
#endif // ICON_ROTATION_ENABLED
        }
    }

    hal_gfx_cl_submit(cl);
    hal_gfx_cl_wait(cl);
}

#if ICON_CACHED_IN_PSRAM_ENABLED
static void build_icon_cache(lv_circular_list_t *list)
{
    if (list->item_count == 0)
    {
        return;
    }

    LV_ASSERT(list->icons_cache == NULL);

    // Create cache list
    lv_img_dsc_t **cache = lv_mem_alloc(list->item_count * sizeof(lv_img_dsc_t *));

    for (uint16_t i = 0; i < list->item_count; i++)
    {
        cache[i] = lv_mem_alloc(sizeof(lv_img_dsc_t));
        memcpy(cache[i], list->items[i].icon, sizeof(lv_img_dsc_t));
        cache[i]->data = (const uint8_t *)app_graphics_mem_malloc(cache[i]->data_size);
        // For switching MMAP endian mode
        img_format(cache[i]);
        memcpy((void *)cache[i]->data, (void *)list->items[i].icon->data, cache[i]->data_size);
    }

    list->icons_cache = cache;
}

static void destroy_icon_cache(lv_circular_list_t *list)
{
    LV_ASSERT(list->icons_cache);
    for (uint16_t i = 0; i < list->item_count; i++)
    {
        app_graphics_mem_free((void *)list->icons_cache[i]->data);
        lv_mem_free((void *)list->icons_cache[i]);
    }

    lv_mem_free(list->icons_cache);
    list->icons_cache = NULL;
}
#endif // ICON_CACHED_IN_PSRAM_ENABLED

static void calc_render_param(lv_circular_list_t *list)
{
    float max_offset_f = ((list->item_count - 1) * ANGULAR_SPACING) / ANGLE_PER_PIXEL;
    list->scroll.max_offset = roundf(max_offset_f);
#if ARRANGE_FROM_LEFT_TO_RIGHT
    list->scroll.hor_offset = list->scroll.max_offset;
#endif // ARRANGE_FROM_LEFT_TO_RIGHT
}

static void scroll_timer_cb(lv_timer_t *p_timer)
{
    lv_obj_t *obj = (lv_obj_t *)p_timer->user_data;
    lv_circular_list_t *list = AS_CLIST(obj);
    float diff = list->scroll.momentum;
    float offset = list->scroll.hor_offset;
    float max_offset = list->scroll.max_offset;
    bool scroll_done = true;

    diff *= FRICTION_FACTOR;
    if (offset < 0 || offset > max_offset)
    {
        scroll_done = false;
        diff *= OUT_OF_BOUND_FACTOR;
        offset += diff;

        float force = LV_MAX(0 - offset, offset - max_offset);
        if (force > 1.f)
        {
            offset += copysignf(force * ELASTIC_FACTOR, 0 - offset) + copysignf(1.f, 0 - offset);
        }
        else
        {
            if (offset < (max_offset - offset))
            {
                offset = 0;
            }
            else
            {
                offset = max_offset;
            }
            scroll_done = true;
        }
    }
    else
    {
        // Snap won't happen if out of bond, so only calc when not oob
        // Caculate Snap
        offset += diff;
#if SNAP_ENABLED
        float space_px = ANGULAR_SPACING / ANGLE_PER_PIXEL;
        uint16_t snap_index = roundf(offset / space_px);
        float snap_offset = snap_index * space_px;

        float force = offset - snap_offset;
        if (fabsf(diff) < fabsf(force))
        {
            force *= SNAP_FACTOR;

            if (fabsf(force) > 1.f)
            {
                offset -= force;
                scroll_done = false;
            }
            else
            {
                offset = snap_offset;
                diff = 0;
                scroll_done = true;
            }
        }
#endif // SNAP_ENABLED
    }

    if (LV_ABS(diff) < 1.f && scroll_done)
    {
        diff = 0;
        lv_timer_pause(p_timer);
    }

    list->scroll.hor_offset = offset;
    list->scroll.momentum = diff;
    handle_focus_change(list);
    lv_obj_invalidate(obj);
}

static void handle_focus_change(lv_circular_list_t *list)
{
    float angle_offset = list->scroll.hor_offset * ANGLE_PER_PIXEL;

    uint16_t focus_idx = 0xFFFF;
    float focus_dist = 360.f;
    for (uint16_t i = 0; i < list->item_count; i++)
    {
        float icon_angle = i * ANGULAR_SPACING - angle_offset + 90;
        if (icon_angle < FOCUS_ZONE_MAX && icon_angle > FOCUS_ZONE_MIN)
        {
            float focus_center = (FOCUS_ZONE_MAX + FOCUS_ZONE_MIN) / 2;
            if (fabsf(icon_angle - focus_center) < focus_dist)
            {
                focus_dist = fabsf(icon_angle - focus_center);
                focus_idx = i;
            }
        }
        else if (focus_dist < 360.f)
        {
            // Found
            break;
        }
    }

#if ARRANGE_FROM_LEFT_TO_RIGHT
    if (focus_idx != 0xFFFF)
    {
        focus_idx = (list->item_count - 1) - focus_idx;
    }
#endif // ARRANGE_FROM_LEFT_TO_RIGHT

    if (focus_idx != list->focus_idx && focus_idx != 0xFFFF)
    {
        list->focus_idx = focus_idx;
        // Pass the same parameter as lv_roller
        lv_event_send(&list->obj, LV_EVENT_VALUE_CHANGED, &list->focus_idx);
    }
}

static void scroll_anim_exec_cb(void *var, int32_t val)
{
    AS_CLIST(var)->scroll.hor_offset = val;
    handle_focus_change(AS_CLIST(var));
}

static void scroll_anim_ready_cb(lv_anim_t *anim)
{
    lv_timer_resume(AS_CLIST(anim->var)->scroll.scroll_tmr);
}

static inline float regulate_angle(float angle)
{
    while (angle > 360.f)
        angle -= 360.f;
    while (angle < 0)
        angle += 360.f;
    return angle;
}

static inline uint32_t img_format(const lv_img_dsc_t *img)
{
    uint32_t fmt = 0;
    switch (img->header.cf)
    {
        case LV_IMG_CF_GDX_RGB565:
        case LV_IMG_CF_GDX_RGB565_CHROMA_KEYED:
#if LV_COLOR_DEPTH == 16
        case LV_IMG_CF_TRUE_COLOR:
        case LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED:
        case LV_IMG_CF_TRUE_COLOR_ALPHA:
#endif // LV_COLOR_DEPTH == 16
            fmt = HAL_GFX_RGB565;
            lv_port_res_mode_set(1);
            break;

        case LV_IMG_CF_GDX_RGBA8888:
        case LV_IMG_CF_GDX_RGBA8888_CHROMA_KEYED:
#if LV_COLOR_DEPTH == 32
        case LV_IMG_CF_TRUE_COLOR:
        case LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED:
        case LV_IMG_CF_TRUE_COLOR_ALPHA:
#endif // LV_COLOR_DEPTH == 32
            fmt = HAL_GFX_RGBA8888;
            lv_port_res_mode_set(2);
            break;

        case LV_IMG_CF_GDX_TSC4:
            fmt = HAL_GFX_TSC4;
            lv_port_res_mode_set(2);
            break;

        case LV_IMG_CF_GDX_TSC6a:
            fmt = HAL_GFX_TSC6A;
            lv_port_res_mode_set(2);
            break;
    }
    return fmt;
}

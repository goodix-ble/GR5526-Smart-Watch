/**
 * @file lv_enhanced_list.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_enhanced_list.h"

#include <math.h>
#include <stdio.h>

#include "app_graphics_mem.h"
#include "hal_gfx_blender.h"
#include "hal_gfx_cmdlist.h"
#include "hal_gfx_graphics.h"

/*********************
 *      DEFINES
 *********************/
#define MY_CLASS          &lv_enhanced_list_class
#define AS_LIST(obj)      ((lv_enhanced_list_t *)(obj))

#define SCALE_TYPE_NONE   0
#define SCALE_TYPE_TOP    1
#define SCALE_TYPE_BOTTOM 2

// Factors are emperical magic, feel free to modify
#define SCROLL_THRESHOLD    15
#define FRICTION_FACTOR     0.95f
#define OUT_OF_BOUND_FACTOR 0.6f
#define ELASIC_FACTOR       0.15f
#define SNAP_FACTOR         0.4f
#define SNAP_VER_POS        ((lv_coord_t)DISP_VER_RES / 2)

// Style related params
#define LABEL_PADDING_LEFT      16
#define ICON_MARGIN_LEFT        16
#define LIST_PADDING_TOP        ((lv_coord_t)DISP_VER_RES / 3 - 40)
#define LIST_PADDING_BOTTOM     ((lv_coord_t)DISP_VER_RES / 3 - 40)
#define LIST_ITEM_LINE_GAP      30
#define LIST_ITEM_CAPTION_WIDTH 270

#define CAPTION_SCROLL_INTVL    50
#define CAPTION_SCROLL_SPEED    2
#define CAPTION_SCROLL_DELAY    50

// Features settings
/**
 * @brief Indicates whether to get full property for caption rendering.
 * Enabling this will have more scalability but will consume more time on caption caching or rendering.
 */
#define USE_FULL_LABEL_STYLES 0

/**
 * @brief Indicates whether to scale list item down when it's getting close to the edge of the screen
 * Enabling thiss will have no influence on performance.
 */
#define SCALE_ON_EDGE_ENABLED 0

/**
 * @brief Indicates whether to snap list item to @ref SNAP_VER_POS.
 * Enabling this will have no influence on performance.
 */
#define SNAP_ENABLED 0

/**
 * @brief Indicates whether to make the caption of item scroll when longer than @ref LIST_ITEM_CAPTION_WIDTH.
 * Enabling this will have no influence on performance.
 */
#define LONG_CAPTION_SCROLL_ENABLED 1

/**
 * @brief Indicates whether to use circular scroll for long caption.
 * Enabling this will have no influence on performance.
 */
#define LONG_CAPTION_SCROLL_CIRCULAR 1

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_enhanced_list_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_enhanced_list_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_enhanced_list_event(const lv_obj_class_t *class_p, lv_event_t *evt);
static void draw_main(lv_event_t *evt);

static inline void gpu_clear_buffer(hal_gfx_cmdlist_t *cl, void *bg, lv_point_t *size, uint32_t color);
static void build_caption_cache(lv_enhanced_list_t *list);
static void destroy_caption_cache(lv_enhanced_list_t *list);
static void calc_render_param(lv_enhanced_list_t *list);
static void scroll_timer_cb(lv_timer_t *p_timer);
#if LONG_CAPTION_SCROLL_ENABLED
static void caption_timer_cb(lv_timer_t *p_timer);
#endif // LONG_CAPTION_SCROLL_ENABLED
static void scroll_anim_exec_cb(void *var, int32_t val);
static void scroll_anim_ready_cb(lv_anim_t *anim);

static inline uint32_t img_format(const lv_img_dsc_t *img);

/**********************
 *  STATIC VARIABLES
 **********************/
const lv_obj_class_t lv_enhanced_list_class = {
    .constructor_cb = lv_enhanced_list_constructor,
    .destructor_cb = lv_enhanced_list_destructor,
    .event_cb = lv_enhanced_list_event,
    .width_def = LV_PCT(100),
    .height_def = LV_PCT(100),
    .instance_size = sizeof(lv_enhanced_list_t),
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

lv_obj_t *lv_enhanced_list_create(lv_obj_t *parent)
{
    LV_LOG_INFO("begin");
    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    return obj;
}

void lv_enhanced_list_set_translate_cb(lv_obj_t *obj, lv_enhanced_list_translate_cb_t translate_cb)
{
    AS_LIST(obj)->translate_cb = translate_cb;
}

void lv_enhanced_list_set_items(lv_obj_t *obj, const lv_enhanced_menu_item_t items[], uint16_t item_count)
{
    AS_LIST(obj)->items = items;
    AS_LIST(obj)->item_count = item_count;

    // Always rebuild caption cache
    if (AS_LIST(obj)->cache_dsc)
    {
        destroy_caption_cache(AS_LIST(obj));
    }
    build_caption_cache(AS_LIST(obj));

    calc_render_param(AS_LIST(obj));
}

const lv_enhanced_menu_item_t *lv_enhanced_list_get_clicked_item(lv_obj_t *obj)
{
    if (AS_LIST(obj)->clicked_index < AS_LIST(obj)->item_count)
    {
        return &AS_LIST(obj)->items[AS_LIST(obj)->clicked_index];
    }
    return NULL;
}

lv_coord_t lv_enhanced_list_get_scroll_offset(lv_obj_t *obj)
{
    return AS_LIST(obj)->scroll.ver_offset;
}

void lv_enhanced_list_set_scroll_offset(lv_obj_t *obj, lv_coord_t offset, bool anim)
{
    if (anim)
    {
        lv_anim_t a;
        lv_anim_init(&a);
        a.var = (void *)obj;
        a.path_cb = lv_anim_path_ease_in_out;
        a.start_value = AS_LIST(obj)->scroll.ver_offset;
        a.end_value = offset;
        a.time = lv_anim_speed_to_time(750, AS_LIST(obj)->scroll.ver_offset, offset);
        a.exec_cb = scroll_anim_exec_cb;
        a.ready_cb = scroll_anim_ready_cb;
        lv_anim_start(&a);
    }
    else
    {
        AS_LIST(obj)->scroll.ver_offset = offset;
    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_enhanced_list_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
    LV_TRACE_OBJ_CREATE("begin");
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    lv_enhanced_list_t *list = (lv_enhanced_list_t *)obj;

    // default settings
    list->scroll.ver_offset = LIST_PADDING_TOP;

    list->scroll.scroll_tmr = lv_timer_create(scroll_timer_cb, LV_INDEV_DEF_READ_PERIOD, list);
    lv_timer_pause(list->scroll.scroll_tmr);
#if LONG_CAPTION_SCROLL_ENABLED
    list->caption_tmr = lv_timer_create(caption_timer_cb, CAPTION_SCROLL_INTVL, list);
    lv_timer_pause(list->caption_tmr);
#endif // LONG_CAPTION_SCROLL_ENABLED
    LV_TRACE_OBJ_CREATE("finished");
}

static void lv_enhanced_list_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
    if (AS_LIST(obj)->cache_dsc)
    {
        destroy_caption_cache(AS_LIST(obj));
    }

    if (AS_LIST(obj)->scroll.scroll_tmr)
    {
        lv_timer_del(AS_LIST(obj)->scroll.scroll_tmr);
    }

#if LONG_CAPTION_SCROLL_ENABLED
    if (AS_LIST(obj)->caption_tmr)
    {
        lv_timer_del(AS_LIST(obj)->caption_tmr);
    }
#endif // LONG_CAPTION_SCROLL_ENABLED
}

static void lv_enhanced_list_event(const lv_obj_class_t *class_p, lv_event_t *evt)
{
    LV_UNUSED(class_p);

    lv_obj_t *obj = lv_event_get_current_target(evt);
    lv_event_code_t code = lv_event_get_code(evt);
    lv_enhanced_list_t *list = AS_LIST(obj);
    lv_indev_t *indev = lv_event_get_indev(evt);

    if (code == LV_EVENT_PRESSED)
    {
        lv_anim_del((void *)list, scroll_anim_exec_cb);
        lv_timer_pause(list->scroll.scroll_tmr);
        list->scroll.focused = false;
        list->scroll.momentum = 0;
    }
    else if (code == LV_EVENT_PRESSING)
    {

        lv_coord_t diff = indev->proc.types.pointer.vect.y;
        if (list->scroll.ver_offset > LIST_PADDING_TOP || list->scroll.ver_offset < list->scroll.max_offset)
        {
            diff = (float)diff * OUT_OF_BOUND_FACTOR;
        }
        list->scroll.ver_offset += diff;

        if (list->scroll.focused)
        {
            lv_obj_invalidate(obj);
        }
        else
        {
            if (abs(indev->proc.types.pointer.vect.y) < abs(indev->proc.types.pointer.vect.x))
            {
                // Not going vertical, do nothing
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
        if (list->scroll.focused)
        {
            lv_indev_t *indev = lv_event_get_indev(evt);
            list->scroll.momentum = indev->proc.types.pointer.vect.y;
            lv_timer_resume(list->scroll.scroll_tmr);
            indev->proc.types.pointer.scroll_obj = NULL;
        }
    }
    else if (code == LV_EVENT_CLICKED || code == LV_EVENT_SHORT_CLICKED)
    {
        // Calc which item has been clicked
        lv_coord_t y_ofs = list->scroll.ver_offset;
        lv_indev_t *indev = lv_event_get_indev(evt);
        lv_point_t click_pos = indev->proc.types.pointer.act_point;
        bool hit = false;

        if (!list->scroll.focused)
        {
            for (uint16_t i = 0; i < list->item_count; i++)
            {
                if (y_ofs > click_pos.y)
                {
                    break;
                }
                // Check if Y-Pos hit
                const lv_img_dsc_t *icon = list->items[i].icon;
                if ((y_ofs + icon->header.h) < click_pos.y)
                {
                    y_ofs += LIST_ITEM_LINE_GAP + icon->header.h;
                    continue;
                }

                // Check if X-Pos hit
                lv_coord_t x_ofs = 0;
                if (list->translate_cb)
                {
                    x_ofs = list->translate_cb(y_ofs, &list->items[i]);
                }
                x_ofs += ICON_MARGIN_LEFT;

                lv_coord_t item_width = icon->header.w + LABEL_PADDING_LEFT + list->cache_dsc[i].width;
                if (click_pos.x > x_ofs && click_pos.x < x_ofs + item_width)
                {
                    list->clicked_index = i;
                    hit = true;
                }

                /**
                 * Break here directly in the loop is by design.
                 * If the code runs here, it means y_ofs is larger than click_pos.y,
                 * which means current item is clicked, or no item is clicked.
                 */
                break;
            }
        }

        if (!hit)
        {
            // Not hit any item, immediately stop processing.
            list->clicked_index = 0xFFFF;
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
    lv_enhanced_list_t *list = AS_LIST(obj);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(evt);

    if (list->item_count == 0)
    {
        return;
    }

    // Prepare render environment
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

    hal_gfx_set_blend_blit(HAL_GFX_BL_SIMPLE);

    lv_coord_t y_ofs = list->scroll.ver_offset;
    for (uint16_t i = 0; i < list->item_count; i++)
    {
        const lv_img_dsc_t *icon = list->items[i].icon;

        if (!icon)
        {
            continue;
        }
        else if (!list->cache_dsc[i].buf)
        {
            continue;
        }

        if ((y_ofs + icon->header.h) < clip_area->y1)
        {
            y_ofs += LIST_ITEM_LINE_GAP + icon->header.h;
            continue;
        }

        lv_coord_t x_ofs = 0;
        if (list->translate_cb)
        {
            x_ofs = list->translate_cb(y_ofs, &list->items[i]);
        }

#if SCALE_ON_EDGE_ENABLED
        float scale = 1.0f;
        uint8_t scale_type = SCALE_TYPE_NONE;

        if (y_ofs < clip_area->y1)
        {
            scale_type = SCALE_TYPE_TOP;
            scale = 1.f - ((clip_area->y1 - y_ofs) / (float)icon->header.h);
        }
        else if (y_ofs + icon->header.h > clip_area->y2)
        {
            scale_type = SCALE_TYPE_BOTTOM;
            scale = 1.f - ((y_ofs + icon->header.h - clip_area->y2) / (float)icon->header.h);
        }

        if (scale_type)
        {
            // Draw scaled icon
            lv_coord_t x_icon = x_ofs + ICON_MARGIN_LEFT + (1.f - scale) * icon->header.w;
            lv_coord_t y_icon;
            if (scale_type == SCALE_TYPE_TOP)
            {
                y_icon = clip_area->y1;
            }
            else
            {
                y_icon = clip_area->y2 - (icon->header.h) * scale;
            }

            lv_point_t icon_size = {
                .x = icon->header.w * scale,
                .y = icon->header.h * scale,
            };
            hal_gfx_bind_src_tex((uintptr_t)icon->data, icon->header.w, icon->header.h, img_format(icon), -1, HAL_GFX_FILTER_BL);
            hal_gfx_blit_rect_fit(x_icon, y_icon, icon_size.x, icon_size.y);
            // Draw scaled caption
            lv_coord_t label_ofs = 0;
            lv_point_t label_size = {
                .x = list->cache_dsc[i].width * scale,
                .y = list->cache_dsc[i].height * scale,
            };
            hal_gfx_bind_src_tex((uintptr_t)list->cache_dsc[i].buf,
                                 list->cache_dsc[i].width,
                                 list->cache_dsc[i].height,
                                 lv_port_get_fb_format(),
                                 -1,
                                 HAL_GFX_FILTER_BL);
            label_ofs = (icon_size.y - label_size.y) / 2;
            lv_coord_t label_inner_ofs = LV_MAX(0, LV_MIN(list->cache_dsc[i].width - LIST_ITEM_CAPTION_WIDTH, list->cache_dsc[i].offset));
            hal_gfx_blit_subrect_fit(x_icon + icon_size.x + LABEL_PADDING_LEFT * scale,
                                     y_icon + label_ofs,
                                     label_size.x,
                                     label_size.y,
                                     label_inner_ofs,
                                     0,
                                     list->cache_dsc[i].width,
                                     list->cache_dsc[i].height);
        }
        else
#endif // SCALE_ON_EDGE_ENABLED
        {
            // Draw icon
            lv_coord_t x_icon = x_ofs + ICON_MARGIN_LEFT;
            lv_coord_t y_icon = y_ofs;
            hal_gfx_bind_src_tex((uintptr_t)icon->data, icon->header.w, icon->header.h, img_format(icon), -1, HAL_GFX_FILTER_BL);
            hal_gfx_blit(x_icon, y_icon);
            // Draw caption
            lv_coord_t label_ofs = 0;
            hal_gfx_bind_src_tex((uintptr_t)list->cache_dsc[i].buf,
                                 list->cache_dsc[i].width,
                                 list->cache_dsc[i].height,
                                 lv_port_get_fb_format(),
                                 -1,
                                 HAL_GFX_FILTER_BL);
            label_ofs = (icon->header.h - list->cache_dsc[i].height) / 2;
            lv_coord_t label_inner_ofs = LV_MAX(0, LV_MIN(list->cache_dsc[i].width - LIST_ITEM_CAPTION_WIDTH, list->cache_dsc[i].offset));
            hal_gfx_blit_subrect(x_icon + icon->header.w + LABEL_PADDING_LEFT,
                                 y_icon + label_ofs,
                                 LIST_ITEM_CAPTION_WIDTH,
                                 list->cache_dsc[i].height,
                                 label_inner_ofs,
                                 0);
        }

        if (y_ofs > clip_area->y2)
        {
            break;
        }

        y_ofs += LIST_ITEM_LINE_GAP + icon->header.h;
    }

    hal_gfx_cl_submit(cl);
    hal_gfx_cl_wait(cl);
}

static inline void gpu_clear_buffer(hal_gfx_cmdlist_t *cl, void *bg, lv_point_t *size, uint32_t color)
{
    hal_gfx_bind_dst_tex((uintptr_t)bg, size->x, size->y, lv_port_get_fb_format(), -1);
    hal_gfx_set_clip(0, 0, size->x, size->y);
    hal_gfx_clear(color);
    hal_gfx_cl_submit(cl);
    hal_gfx_cl_wait(cl);
}

static void build_caption_cache(lv_enhanced_list_t *list)
{
    // WARNING: This function is time consuming
    if (list->item_count == 0)
    {
        return;
    }

    // Create Command List
    hal_gfx_cmdlist_t cmd = hal_gfx_cl_le_create();
    hal_gfx_cmdlist_t *cl = &cmd;
    hal_gfx_cl_bind_circular(cl);

    // Create cache descriptor list
    LV_ASSERT(list->cache_dsc == NULL);
    list->cache_dsc = lv_mem_alloc(list->item_count * sizeof(_txtimg_cache_dsc_t));

    // Render captions into allocated PSRAM area for better performance
    lv_draw_ctx_t *draw_ctx = lv_disp_get_default()->driver->draw_ctx;
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
#if USE_FULL_LABEL_STYLES
    lv_obj_init_draw_label_dsc(&list->obj, LV_PART_MAIN, &label_dsc);
#else
    label_dsc.font = lv_obj_get_style_text_font(&list->obj, LV_PART_MAIN);
    label_dsc.color = lv_color_white();
#endif // USE_FULL_LABEL_STYLES
    lv_point_t textarea;
    bool need_anim = false;
    for (uint16_t i = 0; i < list->item_count; i++)
    {
        if (list->items[i].icon && list->items[i].caption)
        {
            const char *label_str = (char *)list->items[i].caption;
            lv_txt_get_size(&textarea, label_str, label_dsc.font, label_dsc.letter_space, label_dsc.line_space, DISP_HOR_RES, label_dsc.flag);
            list->cache_dsc[i].width = textarea.x;
            list->cache_dsc[i].height = textarea.y;
            list->cache_dsc[i].buf = app_graphics_mem_malloc(hal_gfx_texture_size(lv_port_get_fb_format(), 0, textarea.x, textarea.y));
            gpu_clear_buffer(cl, list->cache_dsc[i].buf, &textarea, 0);

            draw_ctx->buf = list->cache_dsc[i].buf;
            lv_area_t area = {0, 0, textarea.x - 1, textarea.y - 1};
            draw_ctx->clip_area = &area;
            draw_ctx->buf_area = &area;
            lv_draw_label(draw_ctx, &label_dsc, &area, label_str, NULL);

#if LONG_CAPTION_SCROLL_ENABLED
            if (textarea.x > LIST_ITEM_CAPTION_WIDTH)
            {
                list->cache_dsc[i].anim_step = CAPTION_SCROLL_SPEED;
                list->cache_dsc[i].offset = -(CAPTION_SCROLL_DELAY * CAPTION_SCROLL_SPEED);
                need_anim = true;
            }
#endif // LONG_CAPTION_SCROLL_ENABLED
        }
        else
        {
            list->cache_dsc[i].buf = NULL;
        }
    }

#if LONG_CAPTION_SCROLL_ENABLED
    if (need_anim)
    {
        lv_timer_resume(list->caption_tmr);
    }
    else
    {
        lv_timer_pause(list->caption_tmr);
    }
#endif // LONG_CAPTION_SCROLL_ENABLED

    hal_gfx_cl_le_destroy(cl);
}

static void destroy_caption_cache(lv_enhanced_list_t *list)
{
    LV_ASSERT(list->cache_dsc);
    // Destroy all cache buffer
    for (uint16_t i = 0; i < list->item_count; i++)
    {
        if (list->cache_dsc[i].buf)
        {
            app_graphics_mem_free(list->cache_dsc[i].buf);
        }
    }
    lv_mem_free(list->cache_dsc);
}

static void calc_render_param(lv_enhanced_list_t *list)
{
    lv_coord_t max_offset = 0;
    for (uint16_t i = 0; i < list->item_count; i++)
    {
        if (list->items[i].icon)
        {
            max_offset += list->items[i].icon->header.h;
        }
    }
    max_offset += LIST_ITEM_LINE_GAP * (list->item_count - 1);
    if (max_offset > (DISP_VER_RES - LIST_PADDING_TOP - LIST_PADDING_BOTTOM))
    {
        list->scroll.max_offset = -1 * ((max_offset + LIST_PADDING_BOTTOM) - (lv_coord_t)DISP_VER_RES);
    }
    else
    {
        list->scroll.max_offset = LIST_PADDING_TOP;
    }
    list->scroll.max_offset_sb = max_offset;
}

static void scroll_timer_cb(lv_timer_t *p_timer)
{
    lv_obj_t *obj = (lv_obj_t *)p_timer->user_data;
    lv_enhanced_list_t *list = AS_LIST(obj);
    float diff = list->scroll.momentum;
    float offset = list->scroll.ver_offset;
    float max_offset = list->scroll.max_offset;
    bool scroll_done = true;

    diff *= FRICTION_FACTOR;
    if (offset > LIST_PADDING_TOP || offset <= max_offset)
    {
        scroll_done = false;
        diff *= OUT_OF_BOUND_FACTOR;
        offset += diff;

        float force = LV_MAX(offset - LIST_PADDING_TOP, max_offset - offset);
        if (force > 1.f)
        {
            offset += copysignf(force * ELASIC_FACTOR, LIST_PADDING_TOP - offset) + copysignf(1.f, LIST_PADDING_TOP - offset);
        }
        else
        {
            offset = offset > LIST_PADDING_TOP ? LIST_PADDING_TOP : max_offset;
            scroll_done = true;
        }
    }
    else
    {
        // Snap won't happen if out of bond, so only calc when not oob
        // Caculate Snap
        offset += diff;
#if SNAP_ENABLED
        float snap_offset = 0;
        float accu_offset = offset;
        for (uint16_t i = 0; i < list->item_count; i++)
        {
            if ((accu_offset + list->items[i].icon->header.h) > SNAP_VER_POS)
            {
                snap_offset = snap_offset - (list->items[i].icon->header.h / 2) + SNAP_VER_POS;
                break;
            }

            accu_offset += list->items[i].icon->header.h + LIST_ITEM_LINE_GAP;
            snap_offset -= list->items[i].icon->header.h + LIST_ITEM_LINE_GAP;
        }

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

    if (LV_ABS(diff) < SNAP_FACTOR && scroll_done)
    {
        diff = 0;
        lv_timer_pause(p_timer);
    }

    list->scroll.ver_offset = offset;
    list->scroll.momentum = diff;
    lv_obj_invalidate(obj);
}

#if LONG_CAPTION_SCROLL_ENABLED
static void caption_timer_cb(lv_timer_t *p_timer)
{
    lv_enhanced_list_t *list = AS_LIST(p_timer->user_data);
    for (uint16_t i = 0; i < list->item_count; i++)
    {
        _txtimg_cache_dsc_t *cache = &list->cache_dsc[i];
        if (cache->anim_step == 0)
        {
            continue;
        }
        cache->offset += cache->anim_step;
        if (cache->anim_step > 0 && ((cache->offset - cache->width) >= ((CAPTION_SCROLL_DELAY * CAPTION_SCROLL_SPEED) - LIST_ITEM_CAPTION_WIDTH)))
        {
#if LONG_CAPTION_SCROLL_CIRCULAR
            cache->anim_step *= -1;
#else // LONG_CAPTION_SCROLL_CIRCULAR
            cache->offset = -(CAPTION_SCROLL_DELAY * CAPTION_SCROLL_SPEED);
#endif // LONG_CAPTION_SCROLL_CIRCULAR
        }
#if LONG_CAPTION_SCROLL_CIRCULAR
        else if (cache->anim_step < 0 && cache->offset <= -(CAPTION_SCROLL_DELAY * CAPTION_SCROLL_SPEED))
        {
            cache->anim_step *= -1;
        }
#endif // LONG_CAPTION_SCROLL_CIRCULAR
    }

    lv_obj_invalidate(&list->obj);
}
#endif // LONG_CAPTION_SCROLL_ENABLED

static void scroll_anim_exec_cb(void *var, int32_t val)
{
    AS_LIST(var)->scroll.ver_offset = val;
}

static void scroll_anim_ready_cb(lv_anim_t *anim)
{
    lv_timer_resume(AS_LIST(anim->var)->scroll.scroll_tmr);
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

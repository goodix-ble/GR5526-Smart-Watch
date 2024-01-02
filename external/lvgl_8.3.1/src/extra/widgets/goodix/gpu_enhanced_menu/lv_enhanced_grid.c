/**
 * @file lv_enhanced_grid.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_enhanced_grid.h"
#include "app_graphics_mem.h"
#include "hal_gfx_graphics.h"
#include "hal_gfx_blender.h"
#include "hal_gfx_cmdlist.h"
#include "hal_gfx_math.h"

#include <stdio.h>
#include <math.h>

/*********************
 *      DEFINES
 *********************/
#define MY_CLASS                       &lv_enhanced_grid_class
#define AS_GRID(obj)                   ((lv_enhanced_grid_t *)(obj))
#define SCROLL_IN_RANGE(val, max, min) (val > max && val < min)
#define CLAMP(x, min, max)             (x < min ? min : (x > max ? max : x))

// Factors are emperical magic, feel free to modify
#define SCROLL_THRESHOLD    8
#define FRICTION_FACTOR     0.95f
#define OUT_OF_BOUND_FACTOR 0.6f
#define ELASIC_FACTOR       0.15f
#define SNAP_FACTOR         0.4f

// Style related params
#define DEFAULT_CELL_WIDTH        150
#define DEFAULT_CELL_HEIGHT       150

#define DEFAULT_GRID_PADDING_HOR  ((lv_coord_t)(DISP_HOR_RES / 2) - (100 / 2))
#define DEFAULT_GRID_PADDING_VER  ((lv_coord_t)(DISP_VER_RES / 2) - (100 / 2))

#define GRID_RENDER_MARGIN_HOR    50
#define GRID_RENDER_MARGIN_VER    50

#define CLICKABLE_SCALE_THRESHOLD 0.2f

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_enhanced_grid_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_enhanced_grid_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_enhanced_grid_event(const lv_obj_class_t *class_p, lv_event_t *evt);
static void draw_main(lv_event_t *evt);

static void calc_render_param(lv_enhanced_grid_t *grid);
static void scroll_timer_cb(lv_timer_t *p_timer);
static void scroll_hor_anim_exec_cb(void *var, int32_t val);
static void scroll_ver_anim_exec_cb(void *var, int32_t val);
static void scroll_anim_ready_cb(lv_anim_t *anim);

static inline uint32_t img_format(const lv_img_dsc_t *img);

/**********************
 *  STATIC VARIABLES
 **********************/
const lv_obj_class_t lv_enhanced_grid_class = {
    .constructor_cb = lv_enhanced_grid_constructor,
    .destructor_cb = lv_enhanced_grid_destructor,
    .event_cb = lv_enhanced_grid_event,
    .width_def = LV_PCT(100),
    .height_def = LV_PCT(100),
    .instance_size = sizeof(lv_enhanced_grid_t),
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

lv_obj_t *lv_enhanced_grid_create(lv_obj_t *parent)
{
    LV_LOG_INFO("begin");
    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    return obj;
}

void lv_enhanced_grid_set_size_translate_cb(lv_obj_t *obj, lv_enhanced_grid_size_translate_cb_t size_translate_cb)
{
    AS_GRID(obj)->size_translate_cb = size_translate_cb;
}

void lv_enhanced_grid_set_cell_size(lv_obj_t *obj, lv_coord_t cell_width, lv_coord_t cell_height)
{
    AS_GRID(obj)->cell_width = cell_width;
    AS_GRID(obj)->cell_height = cell_height;
    calc_render_param(AS_GRID(obj));
}

void lv_enhanced_grid_set_padding(lv_obj_t *obj, lv_coord_t pad_hor, lv_coord_t pad_ver)
{
    AS_GRID(obj)->grid_padding_hor = pad_hor;
    AS_GRID(obj)->grid_padding_ver = pad_ver;
    calc_render_param(AS_GRID(obj));
}

void lv_enhanced_grid_set_items(lv_obj_t *obj, const lv_enhanced_menu_item_t items[], uint16_t item_count, uint8_t item_per_row)
{
    AS_GRID(obj)->items = items;
    AS_GRID(obj)->item_count = item_count;
    AS_GRID(obj)->item_per_row = item_per_row;
    calc_render_param(AS_GRID(obj));
}

void lv_enhanced_grid_set_scroll_dir(lv_obj_t *obj, lv_dir_t scroll_dir)
{
    AS_GRID(obj)->scroll.scroll_dir = scroll_dir;
}

void lv_enhanced_grid_set_hex_layout(lv_obj_t *obj, bool use_hex_layout)
{
    AS_GRID(obj)->hex_layout = use_hex_layout;
}

lv_point_t lv_enhanced_grid_get_scroll_offset(lv_obj_t *obj)
{
    lv_point_t ret = {
        .x = AS_GRID(obj)->scroll.hor_offset,
        .y = AS_GRID(obj)->scroll.ver_offset,
    };
    return ret;
}

void lv_enhanced_grid_set_scroll_offset(lv_obj_t *obj, lv_point_t offset, bool anim)
{
    if (anim)
    {
        lv_anim_t a_hor;
        lv_anim_init(&a_hor);
        a_hor.var = (void *)obj;
        a_hor.path_cb = lv_anim_path_ease_in_out;
        a_hor.start_value = AS_GRID(obj)->scroll.hor_offset;
        a_hor.end_value = offset.x;
        a_hor.time = lv_anim_speed_to_time(750, AS_GRID(obj)->scroll.hor_offset, offset.x);
        a_hor.exec_cb = scroll_hor_anim_exec_cb;
        a_hor.ready_cb = scroll_anim_ready_cb;
        lv_anim_start(&a_hor);

        lv_anim_t a_ver;
        lv_anim_init(&a_ver);
        a_ver.var = (void *)obj;
        a_ver.path_cb = lv_anim_path_ease_in_out;
        a_ver.start_value = AS_GRID(obj)->scroll.ver_offset;
        a_ver.end_value = offset.x;
        a_ver.time = lv_anim_speed_to_time(750, AS_GRID(obj)->scroll.ver_offset, offset.y);
        a_ver.exec_cb = scroll_ver_anim_exec_cb;
        a_ver.ready_cb = scroll_anim_ready_cb;
        lv_anim_start(&a_ver);
    }
    else
    {
        AS_GRID(obj)->scroll.hor_offset = CLAMP(offset.x, AS_GRID(obj)->scroll.hor_offset_max, AS_GRID(obj)->scroll.hor_offset_min);
        AS_GRID(obj)->scroll.ver_offset = CLAMP(offset.y, AS_GRID(obj)->scroll.ver_offset_max, AS_GRID(obj)->scroll.ver_offset_min);
    }
}

void lv_enhanced_grid_scroll_to_center(lv_obj_t *obj, bool anim)
{
    lv_enhanced_grid_t *grid = AS_GRID(obj);
    lv_point_t center = {
        .x = (grid->scroll.hor_offset_max + grid->scroll.hor_offset_min) / 2,
        .y = (grid->scroll.ver_offset_max + grid->scroll.ver_offset_min) / 2,
    };
    lv_enhanced_grid_set_scroll_offset(obj, center, anim);
}

const lv_enhanced_menu_item_t *lv_enhanced_grid_get_clicked_item(lv_obj_t *obj)
{
    if (AS_GRID(obj)->clicked_index < AS_GRID(obj)->item_count)
    {
        return &AS_GRID(obj)->items[AS_GRID(obj)->clicked_index];
    }
    return NULL;
}

lv_dir_t lv_enhanced_grid_is_on_edge(lv_obj_t *obj)
{
    lv_dir_t ret = LV_DIR_NONE;
    // "MAX" means right/bottom edge and will be negative, offset LOWER than max means on the right/bottom edge
    // "MIN" means left/top edge and will be positive, offset GREATER than min means on the left/top edge
    if (AS_GRID(obj)->scroll.hor_offset <= AS_GRID(obj)->scroll.hor_offset_max)
    {
        ret |= LV_DIR_RIGHT;
    }
    else if (AS_GRID(obj)->scroll.hor_offset >= AS_GRID(obj)->scroll.hor_offset_min)
    {
        ret |= LV_DIR_LEFT;
    }

    if (AS_GRID(obj)->scroll.ver_offset <= AS_GRID(obj)->scroll.ver_offset_max)
    {
        ret |= LV_DIR_BOTTOM;
    }
    else if (AS_GRID(obj)->scroll.ver_offset >= AS_GRID(obj)->scroll.ver_offset_min)
    {
        ret |= LV_DIR_TOP;
    }

    return ret;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_enhanced_grid_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
    LV_TRACE_OBJ_CREATE("begin");
    lv_enhanced_grid_t *grid = AS_GRID(obj);
    grid->hex_layout = false;
    grid->scroll.scroll_dir = LV_DIR_ALL;
    grid->scroll.hor_offset = DEFAULT_GRID_PADDING_HOR;
    grid->scroll.ver_offset = DEFAULT_GRID_PADDING_VER;

    grid->cell_width = DEFAULT_CELL_WIDTH;
    grid->cell_height = DEFAULT_CELL_HEIGHT;
    grid->grid_padding_hor = DEFAULT_GRID_PADDING_HOR;
    grid->grid_padding_ver = DEFAULT_GRID_PADDING_VER;

    grid->scroll.scroll_tmr = lv_timer_create(scroll_timer_cb, LV_INDEV_DEF_READ_PERIOD, grid);
    lv_timer_pause(grid->scroll.scroll_tmr);
    LV_TRACE_OBJ_CREATE("finished");
}

static void lv_enhanced_grid_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
    if (AS_GRID(obj)->scroll.scroll_tmr)
    {
        lv_timer_del(AS_GRID(obj)->scroll.scroll_tmr);
    }
}

static void lv_enhanced_grid_event(const lv_obj_class_t *class_p, lv_event_t *evt)
{
    LV_UNUSED(class_p);

    lv_obj_t *obj = lv_event_get_current_target(evt);
    lv_event_code_t code = lv_event_get_code(evt);
    lv_enhanced_grid_t *grid = AS_GRID(obj);

    if (code == LV_EVENT_PRESSED)
    {
        lv_timer_pause(grid->scroll.scroll_tmr);
        grid->scroll.focused = false;
        grid->scroll.hor_momentum = 0;
        grid->scroll.ver_momentum = 0;
    }
    else if (code == LV_EVENT_PRESSING)
    {
        lv_indev_t *indev = lv_event_get_indev(evt);
        lv_point_t diff = indev->proc.types.pointer.vect;
        if (!SCROLL_IN_RANGE(grid->scroll.hor_offset, grid->scroll.hor_offset_max, grid->scroll.hor_offset_min))
        {
            diff.x = (float)diff.x * OUT_OF_BOUND_FACTOR;
        }

        if (!SCROLL_IN_RANGE(grid->scroll.ver_offset, grid->scroll.ver_offset_max, grid->scroll.ver_offset_min))
        {
            diff.y = (float)diff.y * OUT_OF_BOUND_FACTOR;
        }

        if (grid->scroll.scroll_dir & LV_DIR_HOR)
        {
            grid->scroll.hor_offset += diff.x;
        }
        if (grid->scroll.scroll_dir & LV_DIR_VER)
        {
            grid->scroll.ver_offset += diff.y;
        }

        if (grid->scroll.focused)
        {
            lv_obj_invalidate(obj);
        }
        else
        {
            grid->scroll.ver_momentum += diff.x;
            grid->scroll.hor_momentum += diff.y;
            if (LV_ABS(grid->scroll.ver_momentum) > SCROLL_THRESHOLD || LV_ABS(grid->scroll.hor_momentum) > SCROLL_THRESHOLD)
            {
                grid->scroll.focused = true;
            }
        }
    }
    else if (code == LV_EVENT_RELEASED)
    {
        if (grid->scroll.focused)
        {
            lv_indev_t *indev = lv_event_get_indev(evt);
            grid->scroll.hor_momentum = indev->proc.types.pointer.vect.x;
            grid->scroll.ver_momentum = indev->proc.types.pointer.vect.y;
            lv_timer_resume(grid->scroll.scroll_tmr);
        }
    }
    else if (code == LV_EVENT_CLICKED || code == LV_EVENT_SHORT_CLICKED)
    {
        // Calc which item has been clicked
        lv_indev_t *indev = lv_event_get_indev(evt);
        lv_point_t click_pos = indev->proc.types.pointer.act_point;
        bool hit = false;
        if (!grid->scroll.focused)
        {
            for (uint16_t i = 0; i < grid->item_count; i++)
            {
                const lv_img_dsc_t *icon = grid->items[i].icon;

                lv_point_t grid_pos;

                if (grid->hex_layout)
                {
                    uint16_t group_size = grid->item_per_row * 2 - 1;
                    uint16_t group_idx = i / group_size;
                    uint16_t group_remain = i % group_size;

                    uint16_t yy = group_idx * 2;
                    uint16_t xx = group_remain;
                    if (group_remain >= grid->item_per_row)
                    {
                        yy++;
                        xx -= grid->item_per_row;
                    }

                    grid_pos.x = xx * grid->cell_width;
                    grid_pos.y = yy * (grid->cell_height - 0.14f * grid->cell_width);

                    if (yy % 2 == 1)
                    {
                        grid_pos.x += grid->cell_width / 2;
                    }
                }
                else
                {
                    grid_pos.x = (i % grid->item_per_row) * grid->cell_width;
                    grid_pos.y = (i / grid->item_per_row) * grid->cell_height;
                }

                grid_pos.x += grid->scroll.hor_offset;
                grid_pos.y += grid->scroll.ver_offset;

                grid_pos.x += grid->cell_width / 2;
                grid_pos.y += grid->cell_height / 2;

                if ((grid_pos.x + grid->cell_width) < click_pos.x || (grid_pos.y + grid->cell_height) < click_pos.y)
                {
                    continue;
                }

                float scale = 1.0f;
                if (grid->size_translate_cb)
                {
                    scale = grid->size_translate_cb(&grid_pos, &grid->items[i]);
                }

                if (scale < CLICKABLE_SCALE_THRESHOLD)
                {
                    continue;
                }

                lv_point_t item_size = {
                    .x = icon->header.w * scale,
                    .y = icon->header.h * scale,
                };

                lv_area_t hitbox = {
                    .x1 = grid_pos.x - item_size.x / 2,
                    .x2 = grid_pos.x + item_size.x / 2,
                    .y1 = grid_pos.y - item_size.y / 2,
                    .y2 = grid_pos.y + item_size.y / 2,
                };

                if (click_pos.x > hitbox.x1 && click_pos.x < hitbox.x2 && click_pos.y > hitbox.y1 && click_pos.y < hitbox.y2)
                {
                    grid->clicked_index = i;
                    hit = true;
                }
            }
        }

        if (!hit)
        {
            grid->clicked_index = 0xFFFF;
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
    lv_enhanced_grid_t *grid = AS_GRID(obj);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(evt);

    if (grid->item_count == 0)
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

    lv_coord_t x_ofs = grid->scroll.hor_offset;
    lv_coord_t y_ofs = grid->scroll.ver_offset;
    uint32_t draw_count = 0;

    for (uint16_t i = 0; i < grid->item_count; i++)
    {
        const lv_img_dsc_t *icon = grid->items[i].icon;

        lv_point_t grid_pos;

        if (grid->hex_layout)
        {
            uint16_t group_size = grid->item_per_row * 2 - 1;
            uint16_t group_idx = i / group_size;
            uint16_t group_remain = i % group_size;

            uint16_t yy = group_idx * 2;
            uint16_t xx = group_remain;
            if (group_remain >= grid->item_per_row)
            {
                yy++;
                xx -= grid->item_per_row;
            }

            grid_pos.x = xx * grid->cell_width;
            grid_pos.y = yy * (grid->cell_height - 0.14f * grid->cell_width);

            if (yy % 2 == 1)
            {
                grid_pos.x += grid->cell_width / 2;
            }
        }
        else
        {
            grid_pos.x = (i % grid->item_per_row) * grid->cell_width;
            grid_pos.y = (i / grid->item_per_row) * grid->cell_height;
        }

        grid_pos.x += x_ofs;
        grid_pos.y += y_ofs;

        if ((grid_pos.x + grid->cell_width) < (clip_area->x1 - GRID_RENDER_MARGIN_HOR) || (grid_pos.y + grid->cell_height) < (clip_area->y1 - GRID_RENDER_MARGIN_VER))
        {
            continue;
        }
        else if ((grid_pos.x - grid->cell_width) > (clip_area->x2 + GRID_RENDER_MARGIN_HOR))
        {
            continue;
        }
        else if ((grid_pos.y - grid->cell_height) > (clip_area->y2 + GRID_RENDER_MARGIN_VER))
        {
            break;
        }

        grid_pos.x += grid->cell_width / 2;
        grid_pos.y += grid->cell_height / 2;

        float scale = 1.0f;
        if (grid->size_translate_cb)
        {
            scale = grid->size_translate_cb(&grid_pos, &grid->items[i]);
        }

        if (scale > 0.01f)
        {
            draw_count++;
            // Render item
            hal_gfx_bind_src_tex((uintptr_t)icon->data, icon->header.w, icon->header.h, img_format(icon), -1, HAL_GFX_FILTER_BL);
            if (fabsf(1.f - scale) < 0.02f)
            {
                hal_gfx_blit(grid_pos.x - icon->header.w / 2, grid_pos.y - icon->header.h / 2);
            }
            else
            {
                lv_point_t item_size = {
                    .x = icon->header.w * scale,
                    .y = icon->header.h * scale,
                };

                hal_gfx_blit_rect_fit(grid_pos.x - item_size.x / 2, grid_pos.y - item_size.y / 2, item_size.x, item_size.y);
            }
        }
    }

    hal_gfx_cl_submit(cl);
    hal_gfx_cl_wait(cl);
}

static void calc_render_param(lv_enhanced_grid_t *grid)
{
    if (grid->item_count == 0)
    {
        return;
    }

    // Horizontal
    grid->scroll.hor_offset_min = grid->grid_padding_hor;
    lv_coord_t content_width = grid->item_per_row * grid->cell_width;
    if (content_width > (lv_coord_t)DISP_HOR_RES)
    {
        grid->scroll.hor_offset_max = -1 * (content_width + grid->grid_padding_hor - (lv_coord_t)DISP_HOR_RES);
    }
    else
    {
        grid->scroll.hor_offset_max = grid->grid_padding_hor - content_width;
    }

    // Vertical
    uint16_t maxidx = grid->item_count - 1;
    grid->scroll.ver_offset_min = grid->grid_padding_ver;
    uint16_t total_row;
    lv_coord_t content_height;
    if (grid->hex_layout)
    {
        uint16_t group_size = grid->item_per_row * 2 - 1;
        total_row = (2 * maxidx / group_size) + (maxidx % group_size > grid->item_per_row);
        content_height = total_row * (grid->cell_height - 0.14f * grid->cell_width) + grid->cell_height;
    }
    else
    {
        total_row = maxidx / grid->item_per_row;
        content_height = total_row * grid->cell_height + grid->cell_height;
    }

    if (content_height > (lv_coord_t)DISP_VER_RES)
    {
        grid->scroll.ver_offset_max = -1 * (content_height + grid->grid_padding_ver - (lv_coord_t)DISP_VER_RES);
    }
    else
    {
        grid->scroll.ver_offset_max = grid->grid_padding_ver - content_height;
    }
}

static void scroll_timer_cb(lv_timer_t *p_timer)
{
    lv_obj_t *obj = (lv_obj_t *)p_timer->user_data;
    lv_enhanced_grid_t *grid = AS_GRID(obj);

    float diff_x = grid->scroll.hor_momentum;
    float diff_y = grid->scroll.ver_momentum;

    float offset_x = grid->scroll.hor_offset;
    float offset_y = grid->scroll.ver_offset;

    float max_offset_x = grid->scroll.hor_offset_max;
    float min_offset_x = grid->scroll.hor_offset_min;
    float max_offset_y = grid->scroll.ver_offset_max;
    float min_offset_y = grid->scroll.ver_offset_min;

    bool scroll_done_x = false;
    bool scroll_done_y = false;

    diff_x *= FRICTION_FACTOR;
    diff_y *= FRICTION_FACTOR;

    // Horizontal
    if (grid->scroll.scroll_dir & LV_DIR_HOR)
    {
        if (SCROLL_IN_RANGE(offset_x, max_offset_x, min_offset_x))
        {
            offset_x += diff_x;
            // TODO: Grid Snap
            if (LV_ABS(diff_x) < 1)
            {
                diff_x = 0;
                scroll_done_x = true;
            }
        }
        else
        {
            diff_x *= OUT_OF_BOUND_FACTOR;
            offset_x += diff_x;

            float force = LV_MAX(offset_x - min_offset_x, max_offset_x - offset_x);
            if (force > 1.f)
            {
                offset_x += copysignf(force * ELASIC_FACTOR, min_offset_x - offset_x) + copysignf(1.f, min_offset_x - offset_x);
            }
            else
            {
                if (fabsf(offset_x - min_offset_x) < fabsf(offset_x - max_offset_x))
                {
                    offset_x = min_offset_x;
                }
                else
                {
                    offset_x = max_offset_x;
                }
                diff_x = 0;
                scroll_done_x = true;
            }
        }
    }
    else
    {
        scroll_done_x = true;
    }

    // Vertical
    if (grid->scroll.scroll_dir & LV_DIR_VER)
    {
        if (SCROLL_IN_RANGE(offset_y, max_offset_y, min_offset_y))
        {
            offset_y += diff_y;
            // TODO: Grid Snap
            if (LV_ABS(diff_y) < 1)
            {
                diff_y = 0;
                scroll_done_y = true;
            }
        }
        else
        {
            diff_y *= OUT_OF_BOUND_FACTOR;
            offset_y += diff_y;

            float force = LV_MAX(offset_y - min_offset_y, max_offset_y - offset_y);
            if (force > 1.f)
            {
                offset_y += copysignf(force * ELASIC_FACTOR, min_offset_y - offset_y) + copysignf(1.f, min_offset_y - offset_y);
            }
            else
            {
                if (fabsf(offset_y - min_offset_y) < fabsf(offset_y - max_offset_y))
                {
                    offset_y = min_offset_y;
                }
                else
                {
                    offset_y = max_offset_y;
                }
                diff_y = 0;
                scroll_done_y = true;
            }
        }
    }
    else
    {
        scroll_done_y = true;
    }

    if (scroll_done_x && scroll_done_y)
    {
        lv_timer_pause(p_timer);
    }

    grid->scroll.hor_offset = offset_x;
    grid->scroll.ver_offset = offset_y;
    grid->scroll.hor_momentum = diff_x;
    grid->scroll.ver_momentum = diff_y;
    lv_obj_invalidate(obj);
}

static void scroll_hor_anim_exec_cb(void *var, int32_t val)
{
    AS_GRID(var)->scroll.hor_offset = val;
}

static void scroll_ver_anim_exec_cb(void *var, int32_t val)
{
    AS_GRID(var)->scroll.ver_offset = val;
}

static void scroll_anim_ready_cb(lv_anim_t *anim)
{
    lv_timer_resume(AS_GRID(anim->var)->scroll.scroll_tmr);
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

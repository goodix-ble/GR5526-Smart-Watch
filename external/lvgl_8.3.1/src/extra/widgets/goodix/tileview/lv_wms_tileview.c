/**
 * @file lv_wms_tileview.c
 *
 *
 * 页面的生命周期事件：
 * 1、创建，页面创建回调函数，此时需要设置初步的页面内容，以便在滑动中能展示适当的静态内容。
 * 2、显示，LV_EVENT_READY，此时可以获取资源，并开始实时更新页面中的内容。
 * 3、暂停，LV_EVENT_CANCEL，此时需要释放资源，并停止更新页面中的内容。
 * 4、销毁，LV_EVENT_DELETE，一般不需要处理。
 *
 * 使用技巧：
 * 1、侧滑返回功能：在tile的建造者函数lv_wms_tileview_create_tile_cb中，调用lv_wms_tileview_pop()函数并且return NULL，就能实现该效果。
 *
 * 笔记：
 * 1、一旦布局结束后，对控件的移动就只能通过 lv_obj_move_children_by 来修改。这个涉及了迭代修改的过程。不好直接通过修改coord来实现。
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_wms_tileview.h"
#include "../../../../core/lv_indev.h"
#include "../../../../lvgl.h"
#include "./lv_wms_surface_flinger.h"
#include "app_graphics_mem.h"
#include "hal_gfx_graphics.h"
#include "hal_gfx_transitions.h"
#include "hal_gfx_cmdlist.h"
#include "drv_adapter_port.h"

#if LV_GDX_PATCH_USE_WMS_TILEVIEW
/*********************
 *      DEFINES
 *********************/
#define TRANSLATION_THRESHOLD (DISP_VER_RES / 4)
#define SCROLL_ANIM_TIME_MIN    200    /*ms*/
#define SCROLL_ANIM_TIME_MAX    400    /*ms*/

#define FB_FORMART_SNAPSHOT     HAL_GFX_TSC4
#define FB_FORMART_DRAW_BUF     HAL_GFX_RGB565



/**********************
 *      TYPEDEFS
 **********************/
typedef void (*flush_cb_t)(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

static struct
{
    void *buf1;
    void *buf2;
    void *buf_act;
} s_trans_fb;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_wms_tileview_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj);

static void wms_tileview_evt_cb(const lv_obj_class_t *p_class, lv_event_t * e);

static void _transition_timer_cb(lv_timer_t * timer);

static void _start_transition(lv_wms_tileview_t * tv, lv_wms_tileview_transition_effect_t runing_effect);

static void _start_transition_with_exit_effect(lv_wms_tileview_t * tv);

static void _finish_transition(lv_wms_tileview_t * tv);

static lv_wms_tileview_t * _obtain_wms_tileview(lv_obj_t * from_obj);

static lv_obj_t * _try_create_tile(lv_wms_tileview_t * tv);

static void _lv_wms_tileview_start_translation_animator(lv_wms_tileview_t * tv, uint32_t anim_compensation);

static lv_wms_tileview_transition_effect_t _get_reverse_effect(lv_wms_tileview_transition_effect_t effect);

/**********************
 *  STATIC VARIABLES
 **********************/

const lv_obj_class_t lv_wms_tileview_class = {.constructor_cb = lv_wms_tileview_constructor,
                                          .base_class = &lv_obj_class,
                                          .event_cb = wms_tileview_evt_cb,
                                          .width_def = LV_PCT(100),
                                          .height_def = LV_PCT(100),
                                          .instance_size = sizeof(lv_wms_tileview_t)
                                         };

/**********************
 *      MACROS
 **********************/
#define LV_EVENT_TILE_STARTED LV_EVENT_READY
#define LV_EVENT_TILE_STOPPED LV_EVENT_CANCEL

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_wms_tileview_t * lv_wms_tileview_create_with_default(lv_obj_t * parent, lv_wms_tileview_create_tile_cb creator_cb, int16_t map_id, int16_t row, int16_t col, lv_wms_tileview_transition_effect_t default_effect)
{
    LV_LOG_INFO("begin");
    lv_obj_t * obj = lv_obj_class_create_obj(&lv_wms_tileview_class, parent);
    lv_obj_class_init_obj(obj);
    lv_wms_tileview_t * tv = (lv_wms_tileview_t *)obj;
    tv->default_effect = default_effect;
    tv->runing_effect = default_effect;
    tv->p_tile_creator = creator_cb;

    // create first tile.
    tv->current_tile_info.map_id = map_id;
    tv->current_tile_info.row = row;
    tv->current_tile_info.col = col;
    tv->current_tile_info.cfg.effect = tv->default_effect;
    tv->current_tile_info.cfg.use_same_effect_when_exit = false;
    tv->current_tile_info.birthplace = LV_WMS_TILEVIEW_POS_AT_DOWN;
    tv->current_tile_info.param1 = 0;
    tv->current_tile_info.param2 = NULL;

    tv->next_tile_info = tv->current_tile_info;

    tv->is_creating = true;
    tv->p_current_tile_obj = _try_create_tile(tv);
    tv->is_creating = false;

    if (tv->p_current_tile_obj) {
        lv_obj_set_size(tv->p_current_tile_obj, LV_PCT(100), LV_PCT(100));
        //lv_obj_update_layout(tv->p_current_tile_obj);  /*Be sure the size is correct*/
        lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STARTED, NULL);
    }

    tv->refr_timer = lv_timer_create(_transition_timer_cb, 1, tv); /* flush screen at full speed. */
    if (tv->refr_timer)
    {
        lv_timer_pause(tv->refr_timer);
    }
    return tv;
}

lv_wms_tileview_t * lv_wms_tileview_create(lv_obj_t * parent, lv_wms_tileview_create_tile_cb creator_cb)
{
    return lv_wms_tileview_create_with_default(parent, creator_cb, 0, 0, 0, LV_WMS_TILEVIEW_EFFECT_LINEAR);
}

void lv_wms_tileview_push(lv_obj_t * from_obj, int map_id, int row, int col, lv_wms_tileview_pos_t birthplace, lv_wms_tileview_transition_effect_t effect, uint32_t param1, void * param2)
{
    lv_wms_tileview_t * tv = _obtain_wms_tileview(from_obj);
    if (tv == NULL) return;
    if (tv->is_transition) return;
    if (tv->is_creating) return;

    // 创建下一个视图
    if (tv->p_tile_creator == NULL) return;

    // 确保栈帧有足够的容量
    if (tv->tile_stack.list_size >= tv->tile_stack.list_capacity) {
        size_t new_capacity = tv->tile_stack.list_size + 2; // 一般会显示3层页面，主地图、菜单地图、菜单项详情地图。前2个页面被压到栈中
        tv->tile_stack.p_list = lv_mem_realloc(tv->tile_stack.p_list, new_capacity * sizeof(lv_wms_tileview_tile_info_t));
        if (tv->tile_stack.p_list != NULL) {
            tv->tile_stack.list_capacity = new_capacity;
        }
        else {
            return; // 没有容量
        }
    }

    tv->next_tile_info.map_id = map_id;
    tv->next_tile_info.row = row;
    tv->next_tile_info.col = col;
    tv->next_tile_info.birthplace = birthplace;
    tv->next_tile_info.cfg.effect = effect;
    tv->next_tile_info.param1 = param1;
    tv->next_tile_info.param2 = param2;

    uint32_t trans_start = lv_tick_get();
    // 创建新界面
    tv->is_creating = true;
    lv_obj_t * p_next_tile_obj = _try_create_tile(tv);
    tv->is_creating = false;
    if (p_next_tile_obj == NULL) return;

    tv->p_next_tile_obj = p_next_tile_obj;
    tv->next_tile_info.cfg.effect = effect; // 因为指定的效果，所以必须强制使用。
    lv_obj_set_size(p_next_tile_obj, LV_PCT(100), LV_PCT(100));
    lv_obj_update_layout(p_next_tile_obj);  /*Be sure the size is correct*/

    // 如果新的页面创建成功，就保存当前页面的位置信息到页面栈帧中
    lv_wms_tileview_tile_info_t * p_frame = &tv->tile_stack.p_list[tv->tile_stack.list_size];
    tv->tile_stack.list_size++;
    *p_frame = tv->current_tile_info;

    // 被压栈的页面的恢复时的位置需要重新计算一下
    switch (birthplace)
    {
    case LV_WMS_TILEVIEW_POS_AT_LEFT:
        p_frame->birthplace = LV_WMS_TILEVIEW_POS_AT_RIGHT;
        break;
    case LV_WMS_TILEVIEW_POS_AT_RIGHT:
        p_frame->birthplace = LV_WMS_TILEVIEW_POS_AT_LEFT;
        break;
    case LV_WMS_TILEVIEW_POS_AT_UP:
        p_frame->birthplace = LV_WMS_TILEVIEW_POS_AT_DOWN;
        break;
    case LV_WMS_TILEVIEW_POS_AT_DOWN:
    default:
        p_frame->birthplace = LV_WMS_TILEVIEW_POS_AT_UP;
        break;
    }

    // // 预处理birthplace
    // if (birthplace == LV_WMS_TILEVIEW_POS_AT_ANY) {
    //     // 先判断是否在右边
    //     if (col >= tv->current_col) {
    //         birthplace = LV_WMS_TILEVIEW_POS_AT_RIGHT;
    //     }
    //     else if (row <= tv->current_row) {
    //         // 再判断是否在上边
    //         birthplace = LV_WMS_TILEVIEW_POS_AT_UP;
    //     }
    //     else {
    //         // 否则就按从下边弹出处理
    //         birthplace = LV_WMS_TILEVIEW_POS_AT_DOWN;
    //     }
    // }

    // 确认使用新页面
    tv->is_switched = true;

    // 特殊处理无切换效果的情况
    if (effect == LV_WMS_TILEVIEW_EFFECT_NONE)
    {
        // 通知当前页面即将发生页面切换
        lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STOPPED, NULL);

        _finish_transition(tv);
    }
    else
    {
        _start_transition(tv, effect);
        // 发起动画，使用动画完成最后的滚动
        uint32_t anim_compensation = lv_tick_elaps(trans_start);
        _lv_wms_tileview_start_translation_animator(tv, anim_compensation);
    }
}

void lv_wms_tileview_goto(lv_obj_t * from_obj, int map_id, int row, int col, lv_wms_tileview_pos_t birthplace, lv_wms_tileview_transition_effect_t effect, uint32_t param1, void * param2)
{
    lv_wms_tileview_t * tv = _obtain_wms_tileview(from_obj);
    if (tv == NULL) return;
    if (tv->is_transition) return;
    if (tv->is_creating) return;

    uint32_t trans_start = lv_tick_get();
    // 创建下一个视图
    if (tv->p_tile_creator == NULL) return;

    // 清空页面栈
    tv->tile_stack.list_size = 0;

    tv->next_tile_info.map_id = map_id;
    tv->next_tile_info.row = row;
    tv->next_tile_info.col = col;
    tv->next_tile_info.birthplace = birthplace;
    tv->next_tile_info.cfg.effect = effect;
    tv->next_tile_info.param1 = param1;
    tv->next_tile_info.param2 = param2;

    // 创建新界面
    tv->is_creating = true;
    lv_obj_t * p_next_tile_obj = _try_create_tile(tv);
    tv->is_creating = false;
    if (p_next_tile_obj == NULL) return;

    tv->p_next_tile_obj = p_next_tile_obj;
    tv->next_tile_info.cfg.effect = effect; // 因为指定的效果，所以必须强制使用。
    tv->next_tile_info.cfg.use_same_effect_when_exit = false; // 作为根页面，不应强制使用自己的退场动画

    lv_obj_set_size(p_next_tile_obj, LV_PCT(100), LV_PCT(100));
    lv_obj_update_layout(p_next_tile_obj);  /*Be sure the size is correct*/

    // 确认使用新页面
    tv->is_switched = true;

    if (tv->p_current_tile_obj != NULL)
    {
        // 特殊处理无切换效果的情况
        if (effect == LV_WMS_TILEVIEW_EFFECT_NONE)
        {
            // 通知当前页面即将发生页面切换
            lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STOPPED, NULL);

            _finish_transition(tv);
        }
        else
        {
            _start_transition(tv, effect);
            // 发起动画，使用动画完成最后的滚动
            uint32_t anim_compensation = lv_tick_elaps(trans_start);
            _lv_wms_tileview_start_translation_animator(tv, anim_compensation);
        }
    }
    else
    {
        // 如果当前还没有创建任何页面。就直接作为根界面显示。
        tv->p_current_tile_obj = p_next_tile_obj;
        tv->current_tile_info = tv->next_tile_info;
    }
}

// Return to previous tile.
void lv_wms_tileview_pop(lv_obj_t * from_obj)
{
    lv_wms_tileview_t * tv = _obtain_wms_tileview(from_obj);
    if (tv == NULL) return;
    if (tv->is_transition) return;
    if (tv->is_creating) {
        tv->pending_pop_type = 1;
        return;
    }

    if (tv->p_tile_creator == NULL) return;

    uint32_t trans_start = lv_tick_get();
    for (;tv->tile_stack.list_size != 0;) {
        // 更新页面栈帧
        tv->tile_stack.list_size--;
        lv_wms_tileview_tile_info_t * p_frame = &tv->tile_stack.p_list[tv->tile_stack.list_size];

        // 创建下一个页面
        tv->next_tile_info = *p_frame;
        tv->is_creating = true;
        lv_obj_t * p_next_tile_obj = _try_create_tile(tv);
        tv->is_creating = false;
        if (p_next_tile_obj == NULL) {
            // 如果创建失败，就自动尝试继续弹出
            continue;
        }

        // 初始化新页面的参数
        tv->p_next_tile_obj = p_next_tile_obj;
        lv_obj_set_size(p_next_tile_obj, LV_PCT(100), LV_PCT(100));
        lv_obj_update_layout(p_next_tile_obj);

        // 确认使用新页面
        tv->is_switched = true;

        // 因为是弹出界面，所以用弹出动画的逆序动画
        lv_wms_tileview_transition_effect_t effect = _get_reverse_effect(tv->current_tile_info.cfg.effect);

        // 特殊处理无切换效果的情况
        if (effect == LV_WMS_TILEVIEW_EFFECT_NONE)
        {
            // 通知当前页面即将发生页面切换
            lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STOPPED, NULL);

            _finish_transition(tv);
        }
        else
        {
            _start_transition(tv, effect);
            // 发起动画，使用动画完成最后的滚动
            uint32_t anim_compensation = lv_tick_elaps(trans_start);
            _lv_wms_tileview_start_translation_animator(tv, anim_compensation);
        }
        break;
    }

    return ;
}

// Return to the earliest tile.
void lv_wms_tileview_pop_all(lv_obj_t * from_obj)
{
    lv_wms_tileview_t * tv = _obtain_wms_tileview(from_obj);
    if (tv == NULL) return;
    if (tv->is_animating) return;
    if (tv->is_scrolling) return;
    if (tv->is_creating) {
        tv->pending_pop_type = 2;
        return;
    }

    if (tv->p_tile_creator == NULL) return;

    lv_wms_tileview_tile_info_t * p_frame = NULL;//&tv->tile_stack.p_list[0];

    if (tv->tile_stack.list_size == 0) {
        if((tv->root_tile_info.map_id == tv->current_tile_info.map_id) &&
           (tv->root_tile_info.col    == tv->current_tile_info.col)    &&
           (tv->root_tile_info.row    == tv->current_tile_info.row) ) {
            return;
        }
        p_frame = &tv->root_tile_info;
    } else {
        p_frame = &tv->tile_stack.p_list[0];
    }

    //lv_wms_tileview_tile_info_t * p_frame = &tv->tile_stack.p_list[0];
    tv->tile_stack.list_size = 0;

    uint32_t trans_start = lv_tick_get();
    // 创建下一个页面
    tv->next_tile_info = *p_frame;
    tv->is_creating = true;
    lv_obj_t * p_next_tile_obj = _try_create_tile(tv);
    tv->is_creating = false;
    if (p_next_tile_obj == NULL) return;

    // 初始化新页面的参数
    tv->p_next_tile_obj = p_next_tile_obj;
    lv_obj_set_size(p_next_tile_obj, LV_PCT(100), LV_PCT(100));
    lv_obj_update_layout(p_next_tile_obj);

    // 确认使用新页面
    tv->is_switched = true;

    // 因为是弹出界面，所以用弹出动画的逆序动画
    lv_wms_tileview_transition_effect_t effect = _get_reverse_effect(tv->current_tile_info.cfg.effect);

    // 特殊处理无切换效果的情况
    if (effect == LV_WMS_TILEVIEW_EFFECT_NONE)
    {
        // 通知当前页面即将发生页面切换
        lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STOPPED, NULL);

        _finish_transition(tv);
    }
    else
    {
        _start_transition(tv, effect); // 因为是弹出界面，所以用弹出动画的逆序动画
        // 发起动画，使用动画完成最后的滚动
        uint32_t anim_compensation = lv_tick_elaps(trans_start);
        _lv_wms_tileview_start_translation_animator(tv, anim_compensation);
    }
}

void lv_wms_tileview_clear_retain(lv_wms_tileview_t * tv) {
    if(!tv)
        return;

    while(tv->retain_list.size > 0){
        int index = tv->retain_list.size - 1;
        lv_obj_del(tv->retain_list.p_items[index].p_obj);
        tv->retain_list.p_items[index].p_obj = NULL;
        memset(&(tv->retain_list.p_items[index].info), 0, sizeof(lv_wms_tileview_tile_info_t));

        tv->retain_list.size --;
    }
}

/*======================
 * Add/remove functions
 *=====================*/

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void lv_wms_tileview_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{
    LV_UNUSED(class_p);
    // lv_obj_set_size(obj, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(obj, LV_OBJ_FLAG_SCROLL_CUSTOM);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE); // ensure unscrollable. use special scroll logic instead native scroll logic.
}

#if LV_GDX_PATCH_DISABLE_STYLE_REFRESH
#include "../../../core/lv_disp.h"
static lv_obj_t * _try_create_tile(lv_wms_tileview_t * tv, int new_map_id, int new_row, int new_col, lv_wms_tileview_transition_effect_t * p_effect)
{
    lv_disp_t * disp = lv_obj_get_disp(&tv->obj);

    //lv_disp_enable_invalidation(disp, false); 不关闭刷新，就可以实现在tile_creator函数中嵌套使用pop函数，实现侧滑返回的效果。
    lv_obj_enable_style_refresh(false);

    lv_obj_t * p_tile_obj = tv->p_tile_creator(&tv->obj, new_map_id, new_row, new_col, p_effect);

    //lv_disp_enable_invalidation(disp, true);
    lv_obj_enable_style_refresh(true);

    if (p_tile_obj) {
        lv_obj_refresh_style(p_tile_obj, LV_PART_ANY, LV_STYLE_PROP_ANY);
    }

    return p_tile_obj;
}
#else
static lv_obj_t * _try_create_tile(lv_wms_tileview_t * tv)
{
    lv_obj_t * p_next_tile_obj = NULL;

    // first, lookup existed tile
    if (tv->retain_list.size > 0 && tv->retain_list.p_items != NULL)
    {
        int size = tv->retain_list.size;
        int16_t map_id = tv->next_tile_info.map_id;
        int16_t new_row = tv->next_tile_info.row;
        int16_t new_col = tv->next_tile_info.col;
        for (int i = 0; i < size; i++)
        {
            lv_wms_tileview_tile_info_t * p_info = &tv->retain_list.p_items[i].info;
            if (p_info->map_id == map_id &&
                p_info->row == new_row &&
                p_info->col == new_col)
            {
                p_next_tile_obj = tv->retain_list.p_items[i].p_obj;
                tv->next_tile_info.cfg = tv->retain_list.p_items[i].info.cfg;
                // instead of creating
                lv_obj_clear_flag(p_next_tile_obj, LV_OBJ_FLAG_HIDDEN);
                break;
            }
        }
    }

    // finally, try to create one.
    if (p_next_tile_obj == NULL)
    {
        tv->next_tile_info.cfg.is_retain = false;
        tv->next_tile_info.cfg.use_same_effect_when_exit = false;

        p_next_tile_obj = tv->p_tile_creator(&tv->obj, &tv->current_tile_info, &tv->next_tile_info, &tv->next_tile_info.cfg);

        if (p_next_tile_obj != NULL && tv->next_tile_info.cfg.is_retain)
        {
            // ensure capacity
            if (tv->retain_list.size >= tv->retain_list.capacity)
            {
                uint16_t new_capacity = tv->retain_list.capacity + 2;
                lv_wms_tileview_retain_list_item_t * tmp;
                tmp = lv_mem_realloc(tv->retain_list.p_items, new_capacity * sizeof(lv_wms_tileview_retain_list_item_t));
                if (tmp != NULL)
                {
                    tv->retain_list.p_items = tmp;
                    tv->retain_list.capacity = new_capacity;
                }
            }
            // save
            if (tv->retain_list.size < tv->retain_list.capacity)
            {
                lv_wms_tileview_retain_list_item_t * tmp = &tv->retain_list.p_items[tv->retain_list.size];
                tv->retain_list.size++;
                tmp->info = tv->next_tile_info;
                tmp->p_obj = p_next_tile_obj;
            }
        }
    }

    return p_next_tile_obj;
}
#endif

static void _dummy_flush_cb(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    disp_drv->draw_buf->flushing = 0;
    disp_drv->draw_buf->flushing_last = 0;
}

static lv_wms_tileview_t * _obtain_from_top_to_bottom(lv_obj_t * parent, int lvl)
{
    if (parent == NULL)
    {
        return NULL;
    }
    if (parent->class_p == &lv_wms_tileview_class)
    {
        return (lv_wms_tileview_t *)parent;
    }
    lv_wms_tileview_t * tileview = NULL;
    lvl -= 1;
    if (lvl > 0)
    {
        uint32_t child_cnt = lv_obj_get_child_cnt(parent);
        for (uint32_t i = 0; i < child_cnt; i++)
        {
            lv_obj_t * child = lv_obj_get_child(parent, i);
            tileview = _obtain_from_top_to_bottom(child, lvl);
            if (tileview != NULL)
            {
                break;
            }
        }
    }
    return tileview;
}

static lv_wms_tileview_t * _obtain_wms_tileview(lv_obj_t * from_obj)
{
    /* bottom-up */
    lv_obj_t * cur = from_obj;
    while (cur != NULL)
    {
        if (cur->class_p == &lv_wms_tileview_class) {
            return (lv_wms_tileview_t *)cur;
        }
        cur = cur->parent;
    }

    /* lookup screen and its children */
    return _obtain_from_top_to_bottom(lv_scr_act(), 16);
}

static void _delete_tile(lv_wms_tileview_t * tv, lv_obj_t * obj)
{
    // check if retained
    if (tv->retain_list.p_items != NULL)
    {
        for (int i = 0; i < tv->retain_list.size; i++)
        {
            if (tv->retain_list.p_items[i].p_obj == obj)
            {
                // instead of deleting
                lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
                return;
            }
        }
    }
    lv_obj_del(obj);
}

static void _finish_transition(lv_wms_tileview_t * tv)
{
    // 关闭timer
    tv->is_transition = false;
    if (tv->refr_timer) {
        lv_timer_pause(tv->refr_timer);
        //puts("lv_timer_pause(tv->refr_timer);");
    }
    // 恢复刷新机制
    lv_disp_t * disp = lv_disp_get_default();
    disp->driver->flush_cb = (flush_cb_t)tv->backup_flush_cb;
    lv_disp_enable_invalidation(disp, true);
    if (disp->refr_timer != NULL)
    {
        lv_timer_resume(disp->refr_timer);
        //puts("lv_timer_resume(disp->refr_timer);");
    }

    if (tv->is_switched) {
        // 发送事件告诉新的页面，可以正常显示了
        lv_event_send(tv->p_next_tile_obj, LV_EVENT_TILE_STARTED, NULL);
        // 销毁当前页，并发送事件
        _delete_tile(tv, tv->p_current_tile_obj);
        // 引用新的页面
        tv->p_current_tile_obj = tv->p_next_tile_obj;
        // 更新页面行列信息
        tv->current_tile_info = tv->next_tile_info;
        //puts("switched = true");
    }
    else {
        // 发送事件告诉当前页面，可以恢复显示了
        lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STARTED, NULL);
        // 销毁新的页面，并发送事件
        _delete_tile(tv, tv->p_next_tile_obj);
        //puts("switched = false");
    }
    tv->p_next_tile_obj = NULL;

    disp->driver->draw_buf->buf_act = disp->driver->draw_buf->buf2;

    // 最后整体刷新一次
    lv_obj_invalidate(&tv->obj);
}

static void _scroll_x_anim(void * obj, int32_t v)
{
    lv_wms_tileview_t * tv = (lv_wms_tileview_t *)obj;
    tv->offset_x = (lv_coord_t)v;
}

static void _scroll_y_anim(void * obj, int32_t v)
{
    lv_wms_tileview_t * tv = (lv_wms_tileview_t *)obj;
    tv->offset_y = (lv_coord_t)v;
}

static void _scroll_anim_ready_cb(lv_anim_t * a)
{
    lv_wms_tileview_t * tv = (lv_wms_tileview_t *)(a->var);
    _finish_transition(tv);
    tv->is_animating = false;
    //puts("animating = false\n");
}

static inline void _lv_wms_tileview_animate_translation_to(lv_wms_tileview_t * tv, lv_coord_t final_offset_x, lv_coord_t final_offset_y, uint32_t anim_compensation)
{
    lv_coord_t start_offset_x = tv->offset_x;
    lv_coord_t start_offset_y = tv->offset_y;
    lv_coord_t diff_x = final_offset_x - start_offset_x;
    lv_coord_t diff_y = final_offset_y - start_offset_y;

    lv_disp_t * d = lv_obj_get_disp(&tv->obj);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, tv);
    lv_anim_set_ready_cb(&a, _scroll_anim_ready_cb);

    // 记录下当前的绝对位置，然后不断移动到目标绝对位置上

    if(diff_x) {
        uint32_t t = lv_anim_speed_to_time((lv_disp_get_hor_res(d) * 2) >> 2, start_offset_x, final_offset_x);
        if(t < SCROLL_ANIM_TIME_MIN) t = SCROLL_ANIM_TIME_MIN;
        if(t > SCROLL_ANIM_TIME_MAX) t = SCROLL_ANIM_TIME_MAX;
        lv_anim_set_time(&a, t);
        lv_anim_set_delay(&a, anim_compensation);
        lv_anim_set_values(&a,start_offset_x, final_offset_x);
        lv_anim_set_exec_cb(&a, _scroll_x_anim);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
        lv_anim_start(&a);
        tv->is_animating = true;
        //puts("animating = true\n");
    }
    else if(diff_y) {
        uint32_t t = lv_anim_speed_to_time((lv_disp_get_ver_res(d) * 2) >> 2, start_offset_y, final_offset_y);
        if(t < SCROLL_ANIM_TIME_MIN) t = SCROLL_ANIM_TIME_MIN;
        if(t > SCROLL_ANIM_TIME_MAX) t = SCROLL_ANIM_TIME_MAX;
        lv_anim_set_time(&a, t);
        lv_anim_set_delay(&a, anim_compensation);
        lv_anim_set_values(&a, start_offset_y, final_offset_y);
        lv_anim_set_exec_cb(&a,  _scroll_y_anim);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
        lv_anim_start(&a);
        tv->is_animating = true;
        //puts("animating = true\n");
    }
    else
    {
        // 如果没有启用任何动画，就直接结束。
        _finish_transition(tv);
    }
}

static void _lv_wms_tileview_start_translation_animator(lv_wms_tileview_t * tv, uint32_t anim_compensation)
{
    if (tv->runing_effect == LV_WMS_TILEVIEW_EFFECT_NONE) {
        // show directly
        _finish_transition(tv);
        return;
    }

    // 完成了，新的页面的目标位置就是(0,0)，否则就回归出生位置
    lv_coord_t tv_w = lv_area_get_width(&tv->obj.coords);
    lv_coord_t tv_h = lv_area_get_height(&tv->obj.coords);
    lv_coord_t final_offset_x = 0;
    lv_coord_t final_offset_y = 0;
    if (tv->is_switched)
    {
        switch (tv->next_tile_info.birthplace) {
        case LV_WMS_TILEVIEW_POS_AT_LEFT:
            final_offset_x = tv_w;
            break;
        case LV_WMS_TILEVIEW_POS_AT_RIGHT:
            final_offset_x = -tv_w;
            break;
        case LV_WMS_TILEVIEW_POS_AT_UP:
            final_offset_y = tv_h;
            break;
        case LV_WMS_TILEVIEW_POS_AT_DOWN:
            final_offset_y = -tv_h;
            break;
        default:
            break;
        }
    }
    // 这里的x和y表示的是新页面的的最终坐标。这和GPU的表示方式不同。
    _lv_wms_tileview_animate_translation_to(tv, final_offset_x, final_offset_y, anim_compensation);
}

static void _start_transition_with_exit_effect(lv_wms_tileview_t * tv)
{
    lv_wms_tileview_transition_effect_t runing_effect;

    if (tv->current_tile_info.cfg.use_same_effect_when_exit)
    {
        runing_effect = _get_reverse_effect(tv->current_tile_info.cfg.effect);
    }
    else
    {
        runing_effect = tv->next_tile_info.cfg.effect;
    }

    _start_transition(tv, runing_effect);
}

static void _start_transition(lv_wms_tileview_t * tv, lv_wms_tileview_transition_effect_t runing_effect)
{
    tv->runing_effect = runing_effect;
    tv->is_transition = true;
    tv->offset_x = 0;
    tv->offset_y = 0;

    // 关闭lvgl的绘制能力
    lv_disp_t * disp = lv_disp_get_default();
    lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);

    tv->backup_flush_cb = disp->driver->flush_cb;
    disp->driver->flush_cb = _dummy_flush_cb;

    lv_disp_enable_invalidation(disp, false);

    if (disp->refr_timer != NULL)
    {
        lv_timer_pause(disp->refr_timer);
        //puts("lv_timer_pause(disp->refr_timer);");
    }

    // 压缩保存当前的帧缓存
    lv_transit_config_t config;
    config.fb_format = FB_FORMART_DRAW_BUF;
    config.scrn_res_w = lv_disp_get_hor_res(disp);  // TODO 换为obj的尺寸
    config.scrn_res_h = lv_disp_get_ver_res(disp);  // TODO
    lv_wms_transit_starup(&config);

    lv_wms_transit_screen_cache(LV_TRANSIT_CACHE_SCRN_CURRENT, (uint32_t)(draw_buf->buf_act == draw_buf->buf1 ? draw_buf->buf2 : draw_buf->buf1));

    // 绘制新页面的图像到指定缓存
    hal_gfx_cmdlist_t cmd = hal_gfx_cl_le_create();
    hal_gfx_cmdlist_t* cl = &cmd;

    hal_gfx_cl_bind_circular(cl);

    lv_area_t snapshot_area;

    disp->driver->draw_ctx->clip_area = &snapshot_area;
    disp->driver->draw_ctx->buf_area = &snapshot_area;
    disp->driver->draw_ctx->buf = draw_buf->buf_act; // 使用指定的缓存进行绘制

    lv_obj_get_coords(tv->p_next_tile_obj, &snapshot_area);

    // hal_gfx_set_clip(0, 0, lv_area_get_width(&snapshot_area), lv_area_get_height(&snapshot_area));
    // hal_gfx_bind_dst_tex((uintptr_t)draw_buf->buf_act, lv_area_get_width(&snapshot_area), lv_area_get_height(&snapshot_area), FB_FORMART_DRAW_BUF, -1);
    // hal_gfx_clear(0);
    // hal_gfx_cl_submit(cl);
    // Comment out the following line for performance reason, enable if this leads to any issue.
    // hal_gfx_cl_wait(cl);

    lv_obj_redraw(disp->driver->draw_ctx, tv->p_next_tile_obj);

    hal_gfx_cl_le_destroy(cl);

    // 再压缩
    lv_wms_transit_screen_cache(LV_TRANSIT_CACHE_SCRN_NEXT, (uint32_t)draw_buf->buf_act);

    // 发送生命周期事件给旧页面，让旧页面暂停
    lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STOPPED, NULL);

    // 由于压缩特性，单个FrameBuffer只需要RGB565的一半，故尽可能多的让Transition Buffer在SRAM上（理想情况下两个都在SRAM上）
    s_trans_fb.buf1 = draw_buf->buf1;
    s_trans_fb.buf2 = ((uint8_t *)draw_buf->buf1) + hal_gfx_texture_size(FB_FORMART_SNAPSHOT, 0, config.scrn_res_w, config.scrn_res_h);
    s_trans_fb.buf_act = s_trans_fb.buf1;

    // 开始过渡动画的刷新定时器
    if (tv->refr_timer != NULL)
    {
        lv_timer_resume(tv->refr_timer);
        //puts("lv_timer_resume(tv->refr_timer);");
    }
}

static void wms_tileview_evt_cb(const lv_obj_class_t *p_class, lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    lv_wms_tileview_t * tv = (lv_wms_tileview_t *) obj;

    lv_wms_tileview_class.base_class->event_cb(&lv_wms_tileview_class, e);

    if (code == LV_EVENT_SCROLL_BEGIN || code == LV_EVENT_SCROLL || code == LV_EVENT_SCROLL_END) {
        _lv_indev_proc_t * proc = (_lv_indev_proc_t *)lv_event_get_param(e);
        if (proc) {
            switch (e->code) {
            case LV_EVENT_SCROLL_BEGIN:
            {
                if (tv->is_transition)
                {
                    proc->wait_until_release = 1;
                }
                else
                {
                    // 判断滚动方向，得到下一个要显示的页面的编号（row，col）
                    // 通过回调尝试创建界面，如果创建成果就进入滚动流程，否则丢弃事件。
                    // 发送生命周期事件给旧页面和新页面
                    lv_obj_t * p_next_tile_obj = NULL;
                    if (tv->p_tile_creator)
                    {
                        int16_t next_row = tv->current_tile_info.row;
                        int16_t next_col = tv->current_tile_info.col;
                        lv_wms_tileview_pos_t birthplace_of_next_tile;
                        if (proc->types.pointer.scroll_dir == LV_DIR_HOR) {
                            if (proc->types.pointer.act_point.x > proc->types.pointer.last_point.x) {
                                // swipe to right
                                next_col--;
                                birthplace_of_next_tile = LV_WMS_TILEVIEW_POS_AT_LEFT;
                            }
                            else {
                                // swipe to left
                                next_col++;
                                birthplace_of_next_tile = LV_WMS_TILEVIEW_POS_AT_RIGHT;
                            }
                        }
                        else {
                            if (proc->types.pointer.act_point.y > proc->types.pointer.last_point.y) {
                                // swipe to down
                                next_row--;
                                birthplace_of_next_tile = LV_WMS_TILEVIEW_POS_AT_UP;
                            }
                            else {
                                // swipe to up
                                next_row++;
                                birthplace_of_next_tile = LV_WMS_TILEVIEW_POS_AT_DOWN;
                            }
                        }

                        tv->next_tile_info.map_id = tv->current_tile_info.map_id;
                        tv->next_tile_info.row = next_row;
                        tv->next_tile_info.col = next_col;
                        tv->next_tile_info.birthplace = birthplace_of_next_tile;
                        tv->next_tile_info.cfg.effect = tv->default_effect;
                        tv->next_tile_info.param1 = 0;
                        tv->next_tile_info.param2 = NULL;

                        // 尝试创建新的页面。创建的时候就相当给新页面发送了创建事件。
                        tv->pending_pop_type = 0;
                        tv->is_creating = true;
                        p_next_tile_obj = _try_create_tile(tv);
                        tv->is_creating = false;
                        if (p_next_tile_obj) {
                            tv->is_scrolling = true;
                            //puts("scrolling = true\n");

                            // 准备新页面的位置
                            tv->p_next_tile_obj = p_next_tile_obj;
                            lv_obj_set_size(p_next_tile_obj, LV_PCT(100), LV_PCT(100));
                            lv_obj_update_layout(p_next_tile_obj);  /*Be sure the size is correct*/

                            _start_transition_with_exit_effect(tv);
                        }
                        else {
                            // 如果没有新的页面，就分情况跳过这次滚动。
                            if (tv->pending_pop_type == 0) {
                                // 如果没有进行退栈处理，就放弃滚动
                                proc->types.pointer.scroll_obj = NULL; // 不接受滚动，让给其它控件处理。例如其它控件处理弹性滚动效果。
                            }
                            else {
                                // 如果有退栈处理，表示消费了这次滚动，但后续的滚动相关事件又不需要了
                                proc->wait_until_release = 1;   // 跳过更多的处理逻辑，减低调度耗时。
                                if (tv->pending_pop_type == 1) {
                                    lv_wms_tileview_pop(obj);
                                }
                                else {
                                    lv_wms_tileview_pop_all(obj);
                                }
                            }
                        }
                    }
                }
                break;
            }
            case LV_EVENT_SCROLL:
            {
                if (tv->is_scrolling)
                {
                    if (proc->types.pointer.scroll_dir == LV_DIR_HOR) {
                        //tv->offset_x += proc->types.pointer.vect.x;
                        tv->offset_x = proc->types.pointer.scroll_sum.x;
                    }
                    else {
                        //tv->offset_y += proc->types.pointer.vect.y;
                        tv->offset_y = proc->types.pointer.scroll_sum.y;
                    }
                }
                break;
            }
            case LV_EVENT_SCROLL_END:
            {
                if (tv->is_scrolling)
                {
                    // 启用动画，完成最后的更新。
                    // 动画结束后，发送生命周期事件给旧页面和新页面
                    int delta_distance;
                    bool compare_negtive;
                    if (proc->types.pointer.scroll_dir == LV_DIR_HOR) {
                        delta_distance = (int)proc->types.pointer.scroll_sum.x;
                        compare_negtive = tv->next_tile_info.col > tv->current_tile_info.col;
                    }
                    else {
                        delta_distance = (int)proc->types.pointer.scroll_sum.y;
                        compare_negtive = tv->next_tile_info.row > tv->current_tile_info.row;
                    }
                    if (compare_negtive) {
                        tv->is_switched = delta_distance < -(int)(TRANSLATION_THRESHOLD);
                    }
                    else {
                        tv->is_switched = delta_distance > (int)(TRANSLATION_THRESHOLD);
                    }

                    tv->is_scrolling = false;
                    //puts("scrolling = false\n");

                    // 发起动画，使用动画完成最后的滚动
                    _lv_wms_tileview_start_translation_animator(tv, 0);
                }
                break;
            }
            default:
                break;
            }
        }
    }
    else if(code == LV_EVENT_DELETE)
    {
        if (tv->refr_timer != NULL)
        {
            lv_timer_del(tv->refr_timer);
            tv->refr_timer = NULL;
            //puts("lv_timer_del(tv->refr_timer);");
        }

        tv->tile_stack.list_capacity = 0;
        tv->tile_stack.list_size = 0;
        if (tv->tile_stack.p_list != NULL)
        {
            lv_mem_free(tv->tile_stack.p_list);
            tv->tile_stack.p_list = NULL;
        }

        tv->retain_list.size = 0;
        tv->retain_list.capacity = 0;
        if (tv->retain_list.p_items != NULL)
        {
            lv_mem_free(tv->retain_list.p_items);
            tv->retain_list.p_items = NULL;
        }
    }
}

static void _transition_timer_cb(lv_timer_t * timer)
{
    // 根据效果和偏移距离更新屏幕效果
    lv_wms_tileview_t * tv = (lv_wms_tileview_t *)timer->user_data;
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    // lv_disp_draw_buf_t * draw_buf = lv_disp_get_draw_buf(disp);

    lv_transit_effect_e effect = (lv_transit_effect_e)tv->runing_effect;
    lv_transit_direct_e trans_direct = LV_TRANSIT_DIRECT_NONE;
    lv_coord_t slide_offset             = (lv_coord_t)0;

    switch (tv->runing_effect)
    {
    case LV_WMS_TILEVIEW_EFFECT_LINEAR:
        effect = LV_TRANS_EFFECT_LINEAR;
        break;
    case LV_WMS_TILEVIEW_EFFECT_COVER:
        effect = LV_TRANS_EFFECT_COVER;
        break;
    default:
        break;
    }

    switch (tv->next_tile_info.birthplace) {
    case LV_WMS_TILEVIEW_POS_AT_LEFT:
        trans_direct = LV_TRANSIT_DIRECT_RIGHT;
        slide_offset = tv->offset_x;
        break;
    case LV_WMS_TILEVIEW_POS_AT_RIGHT:
        trans_direct = LV_TRANSIT_DIRECT_LEFT;
        slide_offset = 0 - tv->offset_x;
        break;
    case LV_WMS_TILEVIEW_POS_AT_UP:
        trans_direct = LV_TRANSIT_DIRECT_UP;
        slide_offset = tv->offset_y;
        break;
    case LV_WMS_TILEVIEW_POS_AT_DOWN:
        trans_direct = LV_TRANSIT_DIRECT_DOWN;
        slide_offset = 0 - tv->offset_y;
        break;
    default:
        break;
    }

    // 钳位。如果不钳位，开始向一个方向滑动后，接下来又往方向滑动，界面会依然向初始方向滚动。
    if (slide_offset < 0)
    {
        slide_offset = 0;
    }

    lv_transit_frame_t * p_tf = lv_wms_transit_frame_render(s_trans_fb.buf_act, effect, trans_direct, slide_offset);

    // Flush the frame buffer to screen
    drv_adapter_disp_wait_to_flush();
    drv_adapter_disp_set_show_area(0, 0, p_tf->transit_scrn_res_w - 1, p_tf->transit_scrn_res_h - 1);
    drv_adapter_disp_wait_te();
    drv_adapter_disp_flush((void *) s_trans_fb.buf_act, FB_FORMART_SNAPSHOT, p_tf->transit_scrn_res_w, p_tf->transit_scrn_res_h );

    // Switch the screen flush buffer
    if(s_trans_fb.buf_act == s_trans_fb.buf1)
    {
        s_trans_fb.buf_act = s_trans_fb.buf2;
    }
    else
    {
        s_trans_fb.buf_act = s_trans_fb.buf1;
    }
}

static lv_wms_tileview_transition_effect_t _get_reverse_effect(lv_wms_tileview_transition_effect_t effect)
{
    return effect;
}

#endif /* LV_GDX_PATCH_USE_WMS_TILEVIEW */

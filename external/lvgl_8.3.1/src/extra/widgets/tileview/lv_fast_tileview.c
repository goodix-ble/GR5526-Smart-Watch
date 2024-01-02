/**
 * @file lv_tileview.c
 *
 * 专门用于手表页面切换。
 * 这里将手表的一个界面称为页面，英文用tile表示
 * 在切换tile时，会在内存中临时保留2个视图的对象。切换完成后，只会在内存中保留一个tile的对象。达到节约内存的目的。
 * 增加了生命周期的支持，以便tile中的业务逻辑及时的获取和释放资源。
 *
 * 功能列表：
 * 1、根据指定的动画效果滑动切换页面
 * 2、根据指定的动画效果弹出页面，支持页面压栈
 * 3、内置style刷新节流处理
 *
 * 页面的生命周期事件：
 * 1、创建，页面创建回调函数，此时需要设置初步的页面内容，以便在滑动中能展示适当的静态内容。
 * 2、显示，LV_EVENT_READY，此时可以获取资源，并开始实时更新页面中的内容。
 * 3、暂停，LV_EVENT_CANCEL，此时需要释放资源，并停止更新页面中的内容。
 * 4、销毁，LV_EVENT_DELETE，一般不需要处理。
 *
 * 使用技巧：
 * 1、侧滑返回功能：在tile的建造者函数lv_fast_tileview_create_tile_cb中，调用lv_fast_tileview_pop()函数并且return NULL，就能实现该效果。
 *
 * 笔记：
 * 1、一旦布局结束后，对控件的移动就只能通过 lv_obj_move_children_by 来修改。这个涉及了迭代修改的过程。不好直接通过修改coord来实现。
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_fast_tileview.h"
#include "../../../core/lv_indev.h"
#include "../../../lvgl.h"

#if LV_GDX_PATCH_USE_FAST_TILEVIEW
/*********************
 *      DEFINES
 *********************/
#define TRANSLATION_THRESHOLD (DISP_VER_RES / 4)
#define SCROLL_ANIM_TIME_MIN    200    /*ms*/
#define SCROLL_ANIM_TIME_MAX    400    /*ms*/

//#undef LV_GDX_PATCH_DISABLE_STYLE_REFRESH      // for test
//#define LV_GDX_PATCH_DISABLE_STYLE_REFRESH 0

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_fast_tileview_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj);
static void fast_tileview_evt_cb(const lv_obj_class_t *p_class, lv_event_t * e);
static void _lv_fast_tileview_complete_translation(lv_fast_tileview_t * tv);
static void _move_tile_to(lv_obj_t * obj, lv_coord_t x, lv_coord_t y);
static lv_fast_tileview_t * _obtain_fast_tileview(lv_obj_t * from_obj);
#if LV_GDX_PATCH_DISABLE_STYLE_REFRESH
static lv_obj_t * _try_create_tile(lv_fast_tileview_t * tv, int new_map_id, int new_row, int new_col, lv_fast_tileview_transition_effect_t * p_effect);
#else
#define _try_create_tile(tv, map_id, row, col, p_effect) tv->p_tile_creator(&tv->obj, map_id, row, col, p_effect)
#endif

/**********************
 *  STATIC VARIABLES
 **********************/

const lv_obj_class_t lv_fast_tileview_class = {.constructor_cb = lv_fast_tileview_constructor,
                                          .base_class = &lv_obj_class,
                                          .event_cb = fast_tileview_evt_cb,
                                          .width_def = LV_PCT(100),
                                          .height_def = LV_PCT(100),
                                          .instance_size = sizeof(lv_fast_tileview_t)
                                         };

/**********************
 *      MACROS
 **********************/
#define LV_EVENT_TILE_STARTED LV_EVENT_READY
#define LV_EVENT_TILE_STOPPED LV_EVENT_CANCEL

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_fast_tileview_t * lv_fast_tileview_create(lv_obj_t * parent, lv_fast_tileview_create_tile_cb creator_cb)
{
    LV_LOG_INFO("begin");
    lv_obj_t * obj = lv_obj_class_create_obj(&lv_fast_tileview_class, parent);
    lv_obj_class_init_obj(obj);
    lv_fast_tileview_t * tv = (lv_fast_tileview_t *)obj;
    tv->p_tile_creator = creator_cb;

    // create first tile.
    tv->current_map_id = 0;
    tv->is_creating = true;
    tv->p_current_tile_obj = _try_create_tile(tv, 0, 0, 0, &tv->effect);
    tv->is_creating = false;

    if (tv->p_current_tile_obj) {
        lv_obj_set_size(tv->p_current_tile_obj, LV_PCT(100), LV_PCT(100));
        //lv_obj_update_layout(tv->p_current_tile_obj);  /*Be sure the size is correct*/
        lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STARTED, NULL);
    }
    return tv;
}

void lv_fast_tileview_push(lv_obj_t * from_obj, int map_id, int row, int col/*, lv_fast_tileview_pos_t birthplace, lv_fast_tileview_transition_effect_t effect*/)
{
    lv_fast_tileview_t * tv = _obtain_fast_tileview(from_obj);
    if (tv == NULL) return;
    if (tv->is_animating) return;
    if (tv->is_creating) return;
    if (tv->is_scrolling) return;

    // 创建下一个视图
    if (tv->p_tile_creator == NULL) return;

    // 确保栈帧有足够的容量
    if (tv->tile_stack.list_size >= tv->tile_stack.list_capacity) {
        size_t new_capacity = tv->tile_stack.list_size + 2; // 一般会显示3层页面，主地图、菜单地图、菜单项详情地图。前2个页面被压到栈中
        tv->tile_stack.p_list = lv_mem_realloc(tv->tile_stack.p_list, new_capacity * sizeof(_lv_fast_tileview_tile_stack_frame_t));
        if (tv->tile_stack.p_list != NULL) {
            tv->tile_stack.list_capacity = new_capacity;
        }
        else {
            return; // 没有容量
        }
    }

    // 创建新界面
    tv->is_creating = true;
    lv_obj_t * p_next_tile_obj = _try_create_tile(tv, map_id, row, col, &tv->effect);
    tv->is_creating = false;
    if (p_next_tile_obj == NULL) return;

    // 如果新的页面创建成功，就保存当前页面的位置信息到页面栈帧中
    _lv_fast_tileview_tile_stack_frame_t * p_frame = &tv->tile_stack.p_list[tv->tile_stack.list_size];
    tv->tile_stack.list_size++;
    p_frame->map_id = tv->current_map_id;
    p_frame->row = tv->current_row;
    p_frame->col = tv->current_col;
    // p_frame->effect = tv->effect; 目前不支持使用滚动效果跳转页面
    // p_frame->birthplace = tv->birthplace_of_next_tile; 被压栈的页面的恢复时的位置需要重新计算一下
    // switch (birthplace)
    // {
    // case LV_FAST_TILEVIEW_POS_AT_LEFT:
    //     p_frame->birthplace = LV_FAST_TILEVIEW_POS_AT_RIGHT;
    //     break;
    // case LV_FAST_TILEVIEW_POS_AT_RIGHT:
    //     p_frame->birthplace = LV_FAST_TILEVIEW_POS_AT_LEFT;
    //     break;
    // case LV_FAST_TILEVIEW_POS_AT_UP:
    //     p_frame->birthplace = LV_FAST_TILEVIEW_POS_AT_DOWN;
    //     break;
    // case LV_FAST_TILEVIEW_POS_AT_DOWN:
    // default:
    //     p_frame->birthplace = LV_FAST_TILEVIEW_POS_AT_UP;
    //     break;
    // }

    // // 预处理birthplace
    // if (birthplace == LV_FAST_TILEVIEW_POS_AT_ANY) {
    //     // 先判断是否在右边
    //     if (col >= tv->current_col) {
    //         birthplace = LV_FAST_TILEVIEW_POS_AT_RIGHT;
    //     }
    //     else if (row <= tv->current_row) {
    //         // 再判断是否在上边
    //         birthplace = LV_FAST_TILEVIEW_POS_AT_UP;
    //     }
    //     else {
    //         // 否则就按从下边弹出处理
    //         birthplace = LV_FAST_TILEVIEW_POS_AT_DOWN;
    //     }
    // }

    // 初始化新页面的参数
    tv->p_next_tile_obj = p_next_tile_obj;
    tv->next_map_id = map_id;
    tv->next_row = row;
    tv->next_col = col;
    // tv->birthplace_of_next_tile = birthplace; 目前不支持使用滚动效果跳转页面
    // tv->effect = effect;
    // tv->is_ignore_scroll = true;
    // tv->is_animating = true;
    tv->is_switched = true;
    lv_obj_set_size(p_next_tile_obj, LV_PCT(100), LV_PCT(100));
    lv_obj_update_layout(p_next_tile_obj);  /*Be sure the size is correct*/
    // switch(birthplace) {} 设定新的页面的初始位置

    // 通知当前页面即将发生页面切换
    lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STOPPED, NULL);

    // 特殊处理无切换效果的情况
    // if (effect == LV_FAST_TILEVIEW_EFFECT_NONE) {}
    _move_tile_to(p_next_tile_obj, 0, 0); // 移动页面到指定坐标
    _lv_fast_tileview_complete_translation(tv);

    // 设置动画参数
    // 目前不支持使用滚动效果跳转页面

    // 启动动画
    // 目前不支持使用滚动效果跳转页面
}

// Return to previous tile.
void lv_fast_tileview_pop(lv_obj_t * from_obj)
{
    lv_fast_tileview_t * tv = _obtain_fast_tileview(from_obj);
    if (tv == NULL) return;
    if (tv->is_animating) return;
    if (tv->is_scrolling) return;
    if (tv->is_creating) {
        tv->pending_pop_type = 1;
        return;
    }

    if (tv->p_tile_creator == NULL) return;

    for (;tv->tile_stack.list_size != 0;) {
        // 更新页面栈帧
        tv->tile_stack.list_size--;
        _lv_fast_tileview_tile_stack_frame_t * p_frame = &tv->tile_stack.p_list[tv->tile_stack.list_size];

        // 创建下一个页面
        lv_fast_tileview_transition_effect_t dummy_effect; // 目前不支持使用滚动效果跳转页面，忽略效果参数
        tv->is_creating = true;
        lv_obj_t * p_next_tile_obj = _try_create_tile(tv, p_frame->map_id, p_frame->row, p_frame->col, &dummy_effect);
        tv->is_creating = false;
        if (p_next_tile_obj == NULL) {
            // 如果创建失败，就自动尝试继续弹出
            continue;
        }

        // 初始化新页面的参数
        tv->p_next_tile_obj = p_next_tile_obj;
        tv->next_map_id = p_frame->map_id;
        tv->next_row = p_frame->row;
        tv->next_col = p_frame->col;
        tv->is_switched = true;
        lv_obj_set_size(p_next_tile_obj, LV_PCT(100), LV_PCT(100));
        lv_obj_update_layout(p_next_tile_obj);
        _move_tile_to(p_next_tile_obj, 0, 0); // 移动页面到指定坐标

        // 发送生命周期事件
        // 通知当前页面即将发生页面切换
        lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STOPPED, NULL);

        // 显示新的页面
        _lv_fast_tileview_complete_translation(tv);
        break;
    }
}

// Return to the earliest tile.
void lv_fast_tileview_pop_all(lv_obj_t * from_obj)
{
    lv_fast_tileview_t * tv = _obtain_fast_tileview(from_obj);
    if (tv == NULL) return;
    if (tv->is_animating) return;
    if (tv->is_scrolling) return;
    if (tv->is_creating) {
        tv->pending_pop_type = 2;
        return;
    }

    if (tv->p_tile_creator == NULL) return;
    if (tv->tile_stack.list_size == 0) return;

    _lv_fast_tileview_tile_stack_frame_t * p_frame = &tv->tile_stack.p_list[0];
    tv->tile_stack.list_size = 0;

    // 创建下一个页面
    lv_fast_tileview_transition_effect_t dummy_effect; // 目前不支持使用滚动效果跳转页面，忽略效果参数
    tv->is_creating = true;
    lv_obj_t * p_next_tile_obj = _try_create_tile(tv, p_frame->map_id, p_frame->row, p_frame->col, &dummy_effect);
    tv->is_creating = false;
    if (p_next_tile_obj == NULL) return;

    // 初始化新页面的参数
    tv->p_next_tile_obj = p_next_tile_obj;
    tv->next_map_id = p_frame->map_id;
    tv->next_row = p_frame->row;
    tv->next_col = p_frame->col;
    tv->is_switched = true;
    lv_obj_set_size(p_next_tile_obj, LV_PCT(100), LV_PCT(100));
    lv_obj_update_layout(p_next_tile_obj);
    _move_tile_to(p_next_tile_obj, 0, 0); // 移动页面到指定坐标

    // 发送生命周期事件
    // 通知当前页面即将发生页面切换
    lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STOPPED, NULL);

    // 显示新的页面
    _lv_fast_tileview_complete_translation(tv);
}

/*======================
 * Add/remove functions
 *=====================*/

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void lv_fast_tileview_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{
    LV_UNUSED(class_p);
    // lv_obj_set_size(obj, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(obj, LV_OBJ_FLAG_SCROLL_CUSTOM);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE); // ensure unscrollable. use special scroll logic instead native scroll logic.
}

//static void test_scroll(lv_obj_t * obj, lv_event_t * e)
//{
//    _lv_indev_proc_t * proc = (_lv_indev_proc_t *)lv_event_get_param(e);
//    lv_coord_t diff_x = 0;
//    lv_coord_t diff_y = 0;
//    if (proc->types.pointer.scroll_dir == LV_DIR_HOR) {
//        diff_x = proc->types.pointer.vect.x;
//    }
//    else {
//        diff_y = proc->types.pointer.vect.y;
//    }
//    lv_obj_move_children_by(obj, diff_x, diff_y, true);
//    lv_obj_invalidate(obj);
//}

#if LV_GDX_PATCH_DISABLE_STYLE_REFRESH
#include "../../../core/lv_disp.h"
static lv_obj_t * _try_create_tile(lv_fast_tileview_t * tv, int new_map_id, int new_row, int new_col, lv_fast_tileview_transition_effect_t * p_effect)
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
#endif

static lv_fast_tileview_t * _obtain_fast_tileview(lv_obj_t * from_obj)
{
    lv_obj_t * cur = from_obj;
    while (cur != NULL)
    {
        if (cur->class_p == &lv_fast_tileview_class) {
            return (lv_fast_tileview_t *)cur;
        }
        cur = cur->parent;
    }
    if (cur == NULL)
    {
        // Try get the default one
        if (lv_scr_act()->spec_attr->children[0]->class_p == &lv_fast_tileview_class)
        {
            return (lv_fast_tileview_t *)lv_scr_act()->spec_attr->children[0];
        }
    }
    return NULL;
}

static void _move_tile_by(lv_obj_t * obj, lv_coord_t diff_x, lv_coord_t diff_y)
{
    /*Do nothing if the position is not changed*/
    /*It is very important else recursive positioning can
     *occur without position change*/
    if(diff_x == 0 && diff_y == 0) return;

    /*Invalidate the original area*/
    lv_obj_invalidate(obj);

    obj->coords.x1 += diff_x;
    obj->coords.y1 += diff_y;
    obj->coords.x2 += diff_x;
    obj->coords.y2 += diff_y;

    lv_obj_move_children_by(obj, diff_x, diff_y, false);

    /*Invalidate the new area*/
    lv_obj_invalidate(obj);
}

static void _move_tile_to(lv_obj_t * obj, lv_coord_t x, lv_coord_t y)
{
    /*Convert x and y to absolute coordinates*/
    lv_obj_t * parent = obj->parent;
    lv_coord_t pad_left = lv_obj_get_style_pad_left(parent, LV_PART_MAIN);
    lv_coord_t pad_top = lv_obj_get_style_pad_top(parent, LV_PART_MAIN);
    x += pad_left + parent->coords.x1;
    y += pad_top + parent->coords.y1;
#if LV_GDX_PATCH_REMOVE_BORDER
#else
    lv_coord_t border_width = lv_obj_get_style_border_width(parent, LV_PART_MAIN);
    x += border_width;
    y += border_width;
#endif

    /*Calculate and set the movement*/
    lv_point_t diff;
    diff.x = x - obj->coords.x1;
    diff.y = y - obj->coords.y1;

    _move_tile_by(obj, diff.x, diff.y);
}

static void _translate_tile(lv_fast_tileview_t * tv, lv_coord_t diff_x, lv_coord_t diff_y)
{
    // 根据不同的效果设置子控件的坐标
    // 第2个子控件就是新的tile页面。
    // 根据脏区情况，调用 lv_obj_invalidate(obj);
    switch (tv->effect) {
    case LV_FAST_TILEVIEW_EFFECT_COVER:
    {
        switch (tv->birthplace_of_next_tile) {
        case LV_FAST_TILEVIEW_POS_AT_LEFT:
        case LV_FAST_TILEVIEW_POS_AT_RIGHT:
            _move_tile_by(tv->p_next_tile_obj, diff_x, 0);
            break;
        case LV_FAST_TILEVIEW_POS_AT_UP:
        case LV_FAST_TILEVIEW_POS_AT_DOWN:
            _move_tile_by(tv->p_next_tile_obj, 0, diff_y);
            break;
        default:
            break;
        }
        // 只需要将新的页面标记为脏区
        // lv_obj_invalidate(&tv->obj); _move_tile_by 中已经标记脏区了
        break;
    }
    case LV_FAST_TILEVIEW_EFFECT_TRANSLATION:
    {
        switch (tv->birthplace_of_next_tile) {
        case LV_FAST_TILEVIEW_POS_AT_LEFT:
        case LV_FAST_TILEVIEW_POS_AT_RIGHT:
            _move_tile_by(tv->p_current_tile_obj, diff_x, 0);
            _move_tile_by(tv->p_next_tile_obj, diff_x, 0);
            break;
        case LV_FAST_TILEVIEW_POS_AT_UP:
        case LV_FAST_TILEVIEW_POS_AT_DOWN:
            _move_tile_by(tv->p_current_tile_obj, 0, diff_y);
            _move_tile_by(tv->p_next_tile_obj, 0, diff_y);
            break;
        default:
            break;
        }
        break;
    }
    default: // LV_FAST_TILEVIEW_EFFECT_NONE
        // do nothing
        break;
    }
}

static void _lv_fast_tileview_complete_translation(lv_fast_tileview_t * tv)
{
    if (tv->is_switched) {
        // 发送事件告诉新的页面，可以正常显示了
        lv_event_send(tv->p_next_tile_obj, LV_EVENT_TILE_STARTED, NULL);
        // 销毁当前页，并发送事件
        lv_obj_del(tv->p_current_tile_obj);
        // 引用新的页面
        tv->p_current_tile_obj = tv->p_next_tile_obj;
        // 确保坐标正确
        // _move_tile_to(tv->p_current_tile_obj, 0, 0);
        // 更新页面行列信息
        tv->current_map_id = tv->next_map_id;
        tv->current_row = tv->next_row;
        tv->current_col = tv->next_col;
    }
    else {
        // 发送事件告诉当前页面，可以恢复显示了
        lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STARTED, NULL);
        // 销毁新的页面，并发送事件
        lv_obj_del(tv->p_next_tile_obj);
        // 恢复当前页面的坐标
        // _move_tile_to(tv->p_current_tile_obj, 0, 0);
    }
    tv->p_next_tile_obj = NULL;

    // 最后整体刷新一次
    lv_obj_invalidate(&tv->obj);
}

static void _scroll_x_anim(void * obj, int32_t v)
{
    lv_fast_tileview_t * tv = (lv_fast_tileview_t *)obj;

    // 以新的页面的坐标为准进行动画量的计算
    lv_coord_t start_x = tv->p_next_tile_obj->coords.x1;
    lv_coord_t diff_x = v - start_x;
    _translate_tile(obj, diff_x, 0);
}

static void _scroll_y_anim(void * obj, int32_t v)
{
    lv_fast_tileview_t * tv = (lv_fast_tileview_t *)obj;
    // 以新的页面的坐标为准进行动画量的计算
    lv_coord_t start_y = tv->p_next_tile_obj->coords.y1;
    lv_coord_t diff_y = v - start_y;
    _translate_tile(obj, 0, diff_y);
}

static void _scroll_anim_ready_cb(lv_anim_t * a)
{
    lv_fast_tileview_t * tv = (lv_fast_tileview_t *)(a->var);
    _lv_fast_tileview_complete_translation(tv);
    tv->is_animating = false;
}

static inline void _lv_fast_tileview_animate_translation_to(lv_fast_tileview_t * tv, lv_coord_t x, lv_coord_t y)
{
    /*Convert x and y to absolute coordinates*/
    lv_coord_t pad_left = lv_obj_get_style_pad_left(&tv->obj, LV_PART_MAIN);
    lv_coord_t pad_top = lv_obj_get_style_pad_top(&tv->obj, LV_PART_MAIN);
    x += pad_left + tv->obj.coords.x1;
    y += pad_top + tv->obj.coords.y1;
#if LV_GDX_PATCH_REMOVE_BORDER
#else
    lv_coord_t border_width = lv_obj_get_style_border_width(&tv->obj, LV_PART_MAIN);
    x += border_width;
    y += border_width;
#endif

    // 以新的页面的坐标为准进行动画量的计算
    lv_coord_t start_x = tv->p_next_tile_obj->coords.x1;
    lv_coord_t start_y = tv->p_next_tile_obj->coords.y1;
    lv_coord_t diff_x = x - start_x;
    lv_coord_t diff_y = y - start_y;

    lv_disp_t * d = lv_obj_get_disp(&tv->obj);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, tv);
    lv_anim_set_ready_cb(&a, _scroll_anim_ready_cb);

    // 记录下当前的绝对位置，然后不断移动到目标绝对位置上

    if(diff_x) {
        uint32_t t = lv_anim_speed_to_time((lv_disp_get_hor_res(d) * 2) >> 2, start_x, x);
        if(t < SCROLL_ANIM_TIME_MIN) t = SCROLL_ANIM_TIME_MIN;
        if(t > SCROLL_ANIM_TIME_MAX) t = SCROLL_ANIM_TIME_MAX;
        lv_anim_set_time(&a, t);
        lv_anim_set_values(&a,start_x, x);
        lv_anim_set_exec_cb(&a, _scroll_x_anim);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
        lv_anim_start(&a);
        tv->is_animating = true;
    }
    else if(diff_y) {
        uint32_t t = lv_anim_speed_to_time((lv_disp_get_ver_res(d) * 2) >> 2, start_y, y);
        if(t < SCROLL_ANIM_TIME_MIN) t = SCROLL_ANIM_TIME_MIN;
        if(t > SCROLL_ANIM_TIME_MAX) t = SCROLL_ANIM_TIME_MAX;
        lv_anim_set_time(&a, t);
        lv_anim_set_values(&a, start_y, y);
        lv_anim_set_exec_cb(&a,  _scroll_y_anim);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
        lv_anim_start(&a);
        tv->is_animating = true;
    }
}

static void _lv_fast_tileview_start_translation_animator(lv_fast_tileview_t * tv)
{
    // 只应该为这2种情况处理
    //LV_FAST_TILEVIEW_EFFECT_COVER
    //LV_FAST_TILEVIEW_EFFECT_TRANSLATION
    if (tv->effect == LV_FAST_TILEVIEW_EFFECT_NONE) {
        // 直接显示
        _lv_fast_tileview_complete_translation(tv);
        // 并确保坐标正确
        _move_tile_to(tv->p_current_tile_obj, 0, 0);
        return;
    }

    // 完成了，新的页面的目标位置就是(0,0)，否则就回归出生位置
    lv_coord_t tv_w = lv_area_get_width(&tv->obj.coords);
    lv_coord_t tv_h = lv_area_get_height(&tv->obj.coords);
    lv_coord_t final_x = 0;
    lv_coord_t final_y = 0;
    if (!tv->is_switched)
    {
        switch (tv->birthplace_of_next_tile) {
        case LV_FAST_TILEVIEW_POS_AT_LEFT:
            final_x = -tv_w;
            break;
        case LV_FAST_TILEVIEW_POS_AT_RIGHT:
            final_x = tv_w;
            break;
        case LV_FAST_TILEVIEW_POS_AT_UP:
            final_y = -tv_h;
            break;
        case LV_FAST_TILEVIEW_POS_AT_DOWN:
            final_y = tv_h;
            break;
        default:
            break;
        }
    }
    _lv_fast_tileview_animate_translation_to(tv, final_x, final_y);
}

static void fast_tileview_evt_cb(const lv_obj_class_t *p_class, lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    lv_fast_tileview_t * tv = (lv_fast_tileview_t *) obj;

    // if(code == LV_EVENT_SCROLL_END) {
    // }
    #ifdef WINVER
    //extern const char* EVENT_NAME_TABLE[];
    //printf("EVT: 0x%08X, 0x%02X, %s\n", obj, (int)e->code, EVENT_NAME_TABLE[e->code]);
    #endif
    lv_fast_tileview_class.base_class->event_cb(&lv_fast_tileview_class, e);

    if (code == LV_EVENT_SCROLL_BEGIN || code == LV_EVENT_SCROLL || code == LV_EVENT_SCROLL_END) {
        if (tv->is_animating) {
            tv->is_ignore_scroll = true;
        }
        if (tv->is_ignore_scroll) {
            // 丢弃在动画期间发的一次滚动
            if (code == LV_EVENT_SCROLL_END) {
                tv->is_ignore_scroll = false;
            }
            return;
        }
        _lv_indev_proc_t * proc = (_lv_indev_proc_t *)lv_event_get_param(e);
        if (proc) {
            switch (e->code) {
            case LV_EVENT_SCROLL_BEGIN:
            {
                // 判断滚动方向，得到下一个要显示的页面的编号（row，col）
                // 通过回调尝试创建界面，如果创建成果就进入滚动流程，否则丢弃事件。
                // 发送生命周期事件给旧页面和新页面
                lv_obj_t * p_next_tile_obj = NULL;
                if (tv->p_tile_creator)
                {
                    int16_t next_row = tv->current_row;
                    int16_t next_col = tv->current_col;
                    if (proc->types.pointer.scroll_dir == LV_DIR_HOR) {
                        if (proc->types.pointer.act_point.x > proc->types.pointer.last_point.x) {
                            // swipe to right
                            next_col--;
                            tv->birthplace_of_next_tile = LV_FAST_TILEVIEW_POS_AT_LEFT;
                        }
                        else {
                            // swipe to left
                            next_col++;
                            tv->birthplace_of_next_tile = LV_FAST_TILEVIEW_POS_AT_RIGHT;
                        }
                    }
                    else {
                        if (proc->types.pointer.act_point.y > proc->types.pointer.last_point.y) {
                            // swipe to down
                            next_row--;
                            tv->birthplace_of_next_tile = LV_FAST_TILEVIEW_POS_AT_UP;
                        }
                        else {
                            // swipe to up
                            next_row++;
                            tv->birthplace_of_next_tile = LV_FAST_TILEVIEW_POS_AT_DOWN;
                        }
                    }
                    // 尝试创建新的页面。创建的时候就相当给新页面发送了创建事件。
                    tv->pending_pop_type = 0;
                    tv->is_creating = true;
                    p_next_tile_obj = _try_create_tile(tv, tv->current_map_id, next_row, next_col, &tv->effect);
                    tv->is_creating = false;
                    if (p_next_tile_obj) {
                        tv->is_scrolling = true;
                        tv->p_next_tile_obj = p_next_tile_obj;
                        tv->next_map_id = tv->current_map_id;
                        tv->next_row = next_row;
                        tv->next_col = next_col;
                        lv_obj_set_size(p_next_tile_obj, LV_PCT(100), LV_PCT(100));
                        lv_obj_update_layout(p_next_tile_obj);  /*Be sure the size is correct*/
                        // 修改新的页面的坐标，在滚动期间，不应该更新布局。接下来的动画都是基于obj->coords的。
                        lv_coord_t tv_w = lv_area_get_width(&obj->coords);
                        lv_coord_t tv_h = lv_area_get_height(&obj->coords);
                        switch (tv->birthplace_of_next_tile) {
                        case LV_FAST_TILEVIEW_POS_AT_LEFT:
                            _move_tile_to(p_next_tile_obj, -tv_w, 0);
                            break;
                        case LV_FAST_TILEVIEW_POS_AT_RIGHT:
                            _move_tile_to(p_next_tile_obj, tv_w, 0);
                            break;
                        case LV_FAST_TILEVIEW_POS_AT_UP:
                            _move_tile_to(p_next_tile_obj, 0, -tv_h);
                            break;
                        case LV_FAST_TILEVIEW_POS_AT_DOWN:
                            _move_tile_to(p_next_tile_obj, 0, tv_h);
                            break;
                        default:
                            break;
                        }
                        // 发送生命周期时间给旧页面，让旧页面暂停
                        lv_event_send(tv->p_current_tile_obj, LV_EVENT_TILE_STOPPED, NULL);
                    }
                    else {
                        // 如果没有新的页面，就分情况跳过这次滚动。
                        if (tv->pending_pop_type == 0) {
                            // 如果没有进行退栈处理，就放弃滚动
                            proc->types.pointer.scroll_obj = NULL; // 不接受滚动，让给其它控件处理。例如其它控件处理弹性滚动效果。
                        }
                        else {
                            // 如果有退栈处理，表示消费了这次滚动，但后续的滚动相关事件又不需要了
                            // tv->is_ignore_scroll = true; // 这2种方式都能实现忽略滚动处理的效果。
                            proc->wait_until_release = 1;   // 第2种方式可以跳过更多的处理逻辑，减低调度耗时。
                            if (tv->pending_pop_type == 1) {
                                lv_fast_tileview_pop(obj);
                            }
                            else {
                                lv_fast_tileview_pop_all(obj);
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
                        _translate_tile(tv, proc->types.pointer.vect.x, 0);
                    }
                    else {
                        _translate_tile(tv, 0, proc->types.pointer.vect.y);
                    }
                }
                break;
            }
            case LV_EVENT_SCROLL_END:
            {
                // 启用动画，完成最后的更新。
                // 动画结束后，发送生命周期事件给旧页面和新页面
                int delta_distance;
                bool compare_negtive;
                if (proc->types.pointer.scroll_dir == LV_DIR_HOR) {
                    delta_distance = (int)proc->types.pointer.scroll_sum.x;
                    compare_negtive = tv->next_col > tv->current_col;
                }
                else {
                    delta_distance = (int)proc->types.pointer.scroll_sum.y;
                    compare_negtive = tv->next_row > tv->current_row;
                }
                if (compare_negtive) {
                    tv->is_switched = delta_distance < -(int)(TRANSLATION_THRESHOLD);
                }
                else {
                    tv->is_switched = delta_distance > (int)(TRANSLATION_THRESHOLD);
                }

                tv->is_scrolling = false;

                // 发起动画，使用动画完成最后的滚动
                _lv_fast_tileview_start_translation_animator(tv);
                break;
            }
            default:
                break;
            }
        }
    }
}
#endif /* LV_GDX_PATCH_USE_FAST_TILEVIEW */

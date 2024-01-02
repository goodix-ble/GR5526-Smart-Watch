/**
 * @file lv_fast_tileview.h
 *
 */

#ifndef LV_FAST_TILEVIEW_H
#define LV_FAST_TILEVIEW_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "../../../core/lv_obj.h"

#if LV_GDX_PATCH_USE_FAST_TILEVIEW

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
enum {
    LV_FAST_TILEVIEW_EFFECT_COVER,        // current tile is fixed. Next tile moves and covers current tile.
    LV_FAST_TILEVIEW_EFFECT_TRANSLATION,  // next tile pushes current tile out.
    LV_FAST_TILEVIEW_EFFECT_NONE,         // NO EFFECT, and only display next tile at final moment.
};
typedef uint8_t lv_fast_tileview_transition_effect_t;

enum {
    LV_FAST_TILEVIEW_POS_AT_LEFT,           // next tile is at left.
    LV_FAST_TILEVIEW_POS_AT_RIGHT,          // next tile is at right.
    LV_FAST_TILEVIEW_POS_AT_UP,             // next tile is at up.
    LV_FAST_TILEVIEW_POS_AT_DOWN,           // next tile is at down.

    //LV_FAST_TILEVIEW_POS_AT_ANY = 0xFF,     // calc birthplace automatically
};
typedef uint8_t lv_fast_tileview_pos_t;

typedef lv_obj_t * (*lv_fast_tileview_create_tile_cb)(lv_obj_t * parent, int new_map_id, int new_row, int new_col, lv_fast_tileview_transition_effect_t * p_effect);

typedef struct {
    int16_t map_id;
    int16_t row;
    int16_t col;
    //lv_fast_tileview_pos_t birthplace;
    //lv_fast_tileview_transition_effect_t effect; 跳转页面时，目前不支持使用滚动效果

} _lv_fast_tileview_tile_stack_frame_t;

typedef struct {
    lv_obj_t obj;
    int16_t current_map_id; // determine which table will be used.
    int16_t current_row; // y, position on map
    int16_t current_col; // x, position on map
    int16_t next_map_id;
    int16_t next_row;
    int16_t next_col;
    lv_fast_tileview_transition_effect_t effect;
    lv_fast_tileview_pos_t birthplace_of_next_tile;
    lv_obj_t * p_current_tile_obj;
    lv_obj_t * p_next_tile_obj;
    lv_fast_tileview_create_tile_cb p_tile_creator;
    struct __lv_fast_tileview_tile_stack {
        _lv_fast_tileview_tile_stack_frame_t * p_list;
        uint8_t list_capacity;
        uint8_t list_size;
    } tile_stack;
    bool is_switched;
    bool is_animating;
    bool is_ignore_scroll;
    bool is_creating; // avoid nested lv_fast_tileview_push()/lv_fast_tileview_pop().
    bool is_scrolling; // prevent lv_fast_tileview_push()/lv_fast_tileview_pop() being called when scrolling.
    uint8_t pending_pop_type; // 0-none, 1-pop, 2-pop_all, postpone action
} lv_fast_tileview_t;

extern const lv_obj_class_t lv_fast_tileview_class;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a Tileview object
 * @param parent pointer to an object, it will be the parent of the new tileview
 * @return pointer to the created tileview
 */
lv_fast_tileview_t * lv_fast_tileview_create(lv_obj_t * parent, lv_fast_tileview_create_tile_cb creator_cb);

// show next tile directly and discard current tile. CAN NOT back to current tile automatically.
//void lv_fast_tileview_goto(lv_obj_t * from_obj, int map_id, int row, int col, lv_fast_tileview_transition_effect_t effect);

/**
 * @brief show next tile and save current tile to stack. CAN return to current tile automatically.
 * 
 * @param from_obj  automatically search lv_fast_tileview_t from "from_obj".
 * @param map_id    the coordinate of the tile.
 * @param row       the coordinate of the tile.
 * @param col       the coordinate of the tile.
 */
void lv_fast_tileview_push(lv_obj_t * from_obj, int map_id, int row, int col/*, lv_fast_tileview_pos_t birthplace, lv_fast_tileview_transition_effect_t effect*/);

/**
 * @brief Return to the previous tile. Use this function in p_tile_creator() will action as back button.
 * 
 * @param from_obj automatically search lv_fast_tileview_t from "from_obj".
 */
void lv_fast_tileview_pop(lv_obj_t * from_obj);

/**
 * @brief Return to the earliest tile.
 * 
 * @param from_obj automatically search lv_fast_tileview_t from "from_obj".
 */
void lv_fast_tileview_pop_all(lv_obj_t * from_obj);

/*=====================
 * Other functions
 *====================*/

/**********************
 *      MACROS
 **********************/

#endif /*LV_GDX_PATCH_USE_FAST_TILEVIEW*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_FAST_TILEVIEW_H*/

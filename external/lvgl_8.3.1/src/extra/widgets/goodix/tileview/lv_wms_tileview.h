/**
 * @file lv_wms_tileview.h
 *
 */

#ifndef LV_WMS_TILEVIEW_H
#define LV_WMS_TILEVIEW_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "../../../../core/lv_obj.h"
#include "../../../../../lv_conf.h"

#if LV_GDX_PATCH_USE_WMS_TILEVIEW

#include "lv_wms_surface_flinger.h"

/*********************
 *      DEFINES
 *********************/
// map from surface flinger directly.
#define LV_WMS_TILEVIEW_EFFECT_LINEAR        LV_TRANS_EFFECT_LINEAR
#define LV_WMS_TILEVIEW_EFFECT_COVER         LV_TRANS_EFFECT_COVER
#define LV_WMS_TILEVIEW_EFFECT_CUBE          LV_TRANS_EFFECT_CUBE
#define LV_WMS_TILEVIEW_EFFECT_INNERCUBE     LV_TRANS_EFFECT_INNERCUBE
#define LV_WMS_TILEVIEW_EFFECT_STACK         LV_TRANS_EFFECT_STACK
#define LV_WMS_TILEVIEW_EFFECT_FADE          LV_TRANS_EFFECT_FADE
#define LV_WMS_TILEVIEW_EFFECT_FADE_ZOOM     LV_TRANS_EFFECT_FADE_ZOOM
#define LV_WMS_TILEVIEW_EFFECT_SPIN          LV_TRANS_EFFECT_SPIN
#define LV_WMS_TILEVIEW_EFFECT_PUSHPULL      LV_TRANS_EFFECT_PUSHPULL
#define LV_WMS_TILEVIEW_EFFECT_FADE_ZOOM_ALT LV_TRANS_EFFECT_FADE_ZOOM_ALT

#define LV_WMS_TILEVIEW_EFFECT_NONE         0xFD    /* Special Value to handle no-effect situations */
#define LV_WMS_TILEVIEW_EFFECT_DEFAULT      0xFF    /* Special Value to use default effect */

typedef uint8_t     lv_wms_tileview_transition_effect_t;

/**********************
 *      TYPEDEFS
 **********************/

enum {
    LV_WMS_TILEVIEW_POS_AT_LEFT,           // next tile is at left.
    LV_WMS_TILEVIEW_POS_AT_RIGHT,          // next tile is at right.
    LV_WMS_TILEVIEW_POS_AT_UP,             // next tile is at up.
    LV_WMS_TILEVIEW_POS_AT_DOWN,           // next tile is at down.

    //LV_WMS_TILEVIEW_POS_AT_ANY = 0xFF,     // calc birthplace automatically
};
typedef uint8_t lv_wms_tileview_pos_t;

typedef struct {
    lv_wms_tileview_transition_effect_t effect;
    bool is_retain;
    bool use_same_effect_when_exit;
} lv_wms_tileview_tile_cfg_t;

typedef struct {
    int16_t map_id; // determine which table will be used.
    int16_t row;    // y, position on map
    int16_t col;    // x, position on map
    lv_wms_tileview_pos_t birthplace;

    uint32_t param1;
    void * param2;

    lv_wms_tileview_tile_cfg_t cfg;
} lv_wms_tileview_tile_info_t;

typedef struct _lv_wms_tileview_retain_list_item_t {
    lv_wms_tileview_tile_info_t info;
    lv_obj_t *p_obj;
} lv_wms_tileview_retain_list_item_t;

typedef lv_obj_t * (*lv_wms_tileview_create_tile_cb)(lv_obj_t * parent, const lv_wms_tileview_tile_info_t * p_old_tile, const lv_wms_tileview_tile_info_t * p_new_tile, lv_wms_tileview_tile_cfg_t * const p_new_tile_cfg);

typedef struct {
    lv_obj_t obj;

    lv_wms_tileview_tile_info_t current_tile_info;
    lv_wms_tileview_tile_info_t next_tile_info;
    lv_wms_tileview_tile_info_t root_tile_info;

    lv_obj_t * p_current_tile_obj;
    lv_obj_t * p_next_tile_obj;

    lv_wms_tileview_create_tile_cb p_tile_creator;

    struct __lv_wms_tileview_tile_stack {
        lv_wms_tileview_tile_info_t * p_list;
        uint8_t list_capacity;
        uint8_t list_size;
    } tile_stack;

    struct __lv_wms_tileview_retain_list_t {
        uint16_t capacity;
        uint16_t size;
        lv_wms_tileview_retain_list_item_t * p_items;
    } retain_list;

    void * backup_flush_cb;
    lv_timer_t * refr_timer;
    lv_coord_t offset_x;
    lv_coord_t offset_y;

    bool is_transition; // enter transition progress. refr_timer is running.
    bool is_scrolling;  // prevent lv_wms_tileview_push()/lv_wms_tileview_pop() being called when scrolling. During scroll begin and end.
    bool is_animating;  /* snap is in progress */

    bool is_creating; // avoid nested lv_wms_tileview_push()/lv_wms_tileview_pop().

    bool is_switched;
    uint8_t pending_pop_type; // 0-none, 1-pop, 2-pop_all, postpone action
    lv_wms_tileview_transition_effect_t default_effect;
    lv_wms_tileview_transition_effect_t runing_effect;
} lv_wms_tileview_t;

extern const lv_obj_class_t lv_wms_tileview_class;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a Tileview object
 * @param parent pointer to an object, it will be the parent of the new tileview
 * @return pointer to the created tileview
 */
lv_wms_tileview_t * lv_wms_tileview_create(lv_obj_t * parent, lv_wms_tileview_create_tile_cb creator_cb);

/**
 * Create a Tilveview object and show the specified tile.
 * @param parent        it will be the parent of the new tileview
 * @param creator_cb    every tile is created by this callback function.
 * @param map_id        the coordinate of the tile.
 * @param row           the coordinate of the tile.
 * @param col           the coordinate of the tile.
 * @param default_effect pass it to creator_cb() as default.
 * @return              pointer to the created tileview
 */
lv_wms_tileview_t * lv_wms_tileview_create_with_default(lv_obj_t * parent, lv_wms_tileview_create_tile_cb creator_cb, int16_t map_id, int16_t row, int16_t col, lv_wms_tileview_transition_effect_t default_effect);

/**
 * @brief show new tile directly and discard current tile. CAN NOT back to current tile by `lv_wms_tileview_pop()`.
 * @param from_obj      automatically search lv_wms_tileview_t from "from_obj".
 * @param map_id        the coordinate of the tile.
 * @param row           the coordinate of the tile.
 * @param col           the coordinate of the tile.
 * @param birthplace    initial position of new tile.
 * @param effect        the new tile will replace current tile in specified effect.
 * @param param1        pass it to new tile.
 * @param param2        pass it to new tile.
 */
void lv_wms_tileview_goto(lv_obj_t * from_obj, int map_id, int row, int col, lv_wms_tileview_pos_t birthplace, lv_wms_tileview_transition_effect_t effect, uint32_t param1, void * param2);

/**
 * @brief show next tile and save current tile to stack. CAN return to current tile automatically.
 *
 * @param from_obj  automatically search lv_wms_tileview_t from "from_obj".
 * @param map_id    the coordinate of the tile.
 * @param row       the coordinate of the tile.
 * @param col       the coordinate of the tile.
 */
void lv_wms_tileview_push(lv_obj_t * from_obj, int map_id, int row, int col, lv_wms_tileview_pos_t birthplace, lv_wms_tileview_transition_effect_t effect, uint32_t param1, void * param2);

/**
 * @brief Return to the previous tile. Use this function in p_tile_creator() will action as back button.
 *
 * @param from_obj automatically search lv_wms_tileview_t from "from_obj".
 */
void lv_wms_tileview_pop(lv_obj_t * from_obj);

/**
 * @brief Return to the earliest tile.
 *
 * @param from_obj automatically search lv_wms_tileview_t from "from_obj".
 */
void lv_wms_tileview_pop_all(lv_obj_t * from_obj);

/**
 * @brief clear the retained object and its information
 *
 * @param tv - pointer to tileview instance.
 */
void lv_wms_tileview_clear_retain(lv_wms_tileview_t * tv);

/*=====================
 * Other functions
 *====================*/

/**********************
 *      MACROS
 **********************/

#endif /*LV_GDX_PATCH_USE_WMS_TILEVIEW*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_WMS_TILEVIEW_H*/

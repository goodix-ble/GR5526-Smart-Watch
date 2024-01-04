#ifndef __LV_WMS_SCENE_H__
#define __LV_WMS_SCENE_H__

#include "lvgl.h"
#include "lv_wms.h"

#if WMS_VERSION == WMS_VERSION_v2

#define _INVALID_WIN_ID         0
#define _INVALID_SCENE_ID       0

#define MAX_WIN_STACK_DEPTH     10u

/* Scroll state for Window in Scene */
#define _U                  (1 << 31u)              /* Allow current Window scroll to UP, if Current is TOP, allow scroll to bottom */
#define _D                  (1 << 30u)              /* Allow current Window scroll to DOWM, if Current is Bottom, allow scroll to top */
#define _L                  (1 << 29u)              /* Allow current Window scroll to LEFT, if Current is LEFT, allow scroll to RIGHT */
#define _R                  (1 << 28u)              /* Allow current Window scroll to RIGHT, if Current is RIGHT, allow scroll to LEFT */
#define _H                  (_L | _R)               /* Allow current Window scroll to LEFT/RIGHT,  */
#define _V                  (_U | _D)               /* Allow current Window scroll to UP/DOWM */
#define _A                  (_L | _R | _U | _D)     /* Allow current Window scroll to Any Direction */

#define UUID_WIN(scene_id, win_id)      (((scene_id & 0xffff) << 16) |  (win_id & 0xffff))

enum {
    DIRECTION_TOP    = LV_DIR_TOP,
    DIRECTION_BOTTOM = LV_DIR_BOTTOM,
    DIRECTION_LEFT   = LV_DIR_LEFT,
    DIRECTION_RIGHT  = LV_DIR_RIGHT,
} ;

typedef union {
    struct {
        uint32_t            window_id:16;
        uint32_t            scene_id:16;
    } id;
    uint32_t                uid;
} uuid_t;


typedef struct {
    uint16_t                scene_id;
    uint16_t                main_win_id;
    uint16_t                scene_x_elems;
    uint16_t                scene_y_elems;
    uint32_t *              win_id_table;
} lv_wms_scene_t;


typedef struct {
    uint16_t                win_up_id;
    uint16_t                win_down_id;
    uint16_t                win_left_id;
    uint16_t                win_right_id;
} lv_wms_win_neighbor_id_t;


typedef struct {
    uuid_t                  _cur_id;
    uuid_t                  _root_id;
    uint32_t                _win_stack_pos;
    uuid_t                  _win_stack[MAX_WIN_STACK_DEPTH];
} lv_wms_scene_mangt_t;


typedef struct {
    lv_wms_window_map_t *   p_win_map_table;
    lv_wms_scene_t      *   p_scene_map_table;
    uint16_t                win_amount;
    uint16_t                scene_amount;
    uint16_t                startup_scene_id;
    uint16_t                startup_win_id;
} lv_wms_scene_config_t;

void                        lv_wms_scene_startup(lv_wms_scene_config_t * p_config);

uint32_t                    lv_wms_scene_find_neighbor_window_id(uint32_t scene_id, uint32_t cur_window_id, uint32_t dir);

lv_wms_win_neighbor_id_t    lv_wms_scene_find_all_neighbor_window_id(uint32_t scene_id, uint32_t cur_window_id);

lv_wms_window_t *           lv_wms_scene_find_neighbor_window(uint32_t scene_id, uint32_t cur_window_id, uint32_t dir);

void                        lv_wms_scene_enter_into_win(uint32_t scene_id, uint32_t win_id);

void                        lv_wms_scene_enter_into(uint32_t scene_id);

void                        lv_wms_scene_exit_cur_win(void);

lv_wms_window_t *           lv_wms_window_lookup(uint32_t win_id);

uuid_t                      lv_wms_window_reset_stack(void);

void                        lv_wms_set_cur_scene_id(uint32_t scene_id);

uint32_t                    lv_wms_get_cur_scene_id(void);

void                        lv_wms_set_cur_win_id(uint32_t scene_id);

uint32_t                    lv_wms_get_cur_win_id(void);

#endif /* WMS_VERSION == WMS_VERSION_v2 */

#endif /* __LV_WMS_SCENE_H__ */

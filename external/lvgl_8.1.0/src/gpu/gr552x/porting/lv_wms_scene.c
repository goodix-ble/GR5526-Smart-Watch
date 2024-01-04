#include "stdio.h"
#include "lv_wms.h"
#include "lv_wms_scene.h"

#if WMS_VERSION == WMS_VERSION_v2

/*
 * STATIC METHODS DEFINITIONS
 *****************************************************************************************
 */
static void     lv_wms_register_scenes(lv_wms_scene_t * p_scenes, uint32_t scene_count) ;
static void     lv_wms_register_windows(lv_wms_window_map_t * p_windows, uint32_t window_count);
static bool     lv_wms_is_root_scene(void);

static void     lv_wms_push_win_stack(uint16_t scene_id , uint16_t win_id);
static uuid_t   lv_wms_pop_win_stack(void);
static void     lv_wms_trace_win_stack(void);
/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */

static lv_wms_scene_t *         _p_scene_head = NULL;
static uint32_t                 _s_scene_count = 0;
static lv_wms_window_map_t *    _p_window_head = NULL;
static uint32_t                 _s_window_count = 0;
static lv_wms_scene_mangt_t     _s_wms_scene_manage;


/*
 * PUBLIC METHODS DEFINITIONS
 *****************************************************************************************
 */

void lv_wms_scene_startup(lv_wms_scene_config_t * p_config) {

    uuid_t uuid = {
        .id.scene_id    = p_config->startup_scene_id,
        .id.window_id   = p_config->startup_win_id,
    };

    lv_wms_register_windows(p_config->p_win_map_table,  p_config->win_amount);
    lv_wms_register_scenes(p_config->p_scene_map_table, p_config->scene_amount);

    _s_wms_scene_manage._win_stack_pos   = 0;
    _s_wms_scene_manage._win_stack[0]    = uuid;
    _s_wms_scene_manage._cur_id          = uuid;
    _s_wms_scene_manage._root_id         = uuid;

    lv_wms_window_t * win = lv_wms_window_lookup(_s_wms_scene_manage._cur_id.id.window_id);

    if(win != NULL) {
        win->create_func(_s_wms_scene_manage._cur_id.id.window_id);
    }
}

lv_wms_window_t * lv_wms_window_lookup(uint32_t win_id) {
    uint32_t i = 0;

    for(i = 0; i < _s_window_count; i++) {
        if(_p_window_head[i].win_id == win_id) {
            return _p_window_head[i].win_object;
        }
    }

    return NULL;
}

uint32_t lv_wms_scene_find_neighbor_window_id(uint32_t scene_id, uint32_t cur_window_id, uint32_t dir) {
    uint32_t i = 0,j,k;
    uint32_t goal_id = 0;

    if(_INVALID_WIN_ID == cur_window_id) {
        return _INVALID_WIN_ID;
    }

    if(NULL == _p_scene_head || 0 == _s_scene_count) {
        return _INVALID_WIN_ID;
    }

    for(i = 0 ; i < _s_scene_count; i++) {
        if(_p_scene_head[i].scene_id == scene_id) {
            const uint32_t _X_ELEMS = _p_scene_head[i].scene_x_elems;
            const uint32_t _Y_ELEMS = _p_scene_head[i].scene_y_elems;
            for(j = 0; j < _Y_ELEMS; j++) {
                for(k = 0; k < _X_ELEMS; k++) {
                    if(cur_window_id == ((uint32_t*)(_p_scene_head[i].win_id_table))[j*_X_ELEMS + k]) {
                        switch(dir) {
                            case DIRECTION_TOP:
                            {
                                if(j > 0) {
                                    goal_id = (_p_scene_head[i]).win_id_table[(j-1)*_X_ELEMS + k];
                                } else {
                                    goal_id = 0;
                                }
                            }
                            break;

                            case DIRECTION_BOTTOM:
                            {
                                if((_Y_ELEMS > 1) && (j < _Y_ELEMS - 1)) {
                                    goal_id = (_p_scene_head[i]).win_id_table[(j+1)*_X_ELEMS + k];
                                } else {
                                    goal_id = 0;
                                }
                            }
                            break;

                            case DIRECTION_LEFT:
                            {
                                if(k > 0) {
                                    goal_id = (_p_scene_head[i]).win_id_table[j*_X_ELEMS + k - 1];
                                } else {
                                    goal_id = 0;
                                }
                            }
                            break;

                            case DIRECTION_RIGHT:
                            {
                                if((_X_ELEMS > 1) && (k < _X_ELEMS - 1)) {
                                    goal_id = (_p_scene_head[i]).win_id_table[j*_X_ELEMS + k + 1];
                                } else {
                                    goal_id = 0;
                                }
                            }
                            break;
                        }

                        goto label_found_1;
                    }
                }
            }
        }
    }

label_found_1:

    return goal_id;
}


lv_wms_win_neighbor_id_t lv_wms_scene_find_all_neighbor_window_id(uint32_t scene_id, uint32_t cur_window_id) {
    uint32_t i,j,k;

    lv_wms_win_neighbor_id_t n_id = {_INVALID_WIN_ID, _INVALID_WIN_ID, _INVALID_WIN_ID, _INVALID_WIN_ID};

    if(_INVALID_WIN_ID == cur_window_id) {
        return n_id;
    }

    if(NULL == _p_scene_head || 0 == _s_scene_count) {
        return n_id;
    }

    for(i = 0 ; i < _s_scene_count; i++) {
        if(_p_scene_head[i].scene_id == scene_id) {
            const uint32_t _X_ELEMS = _p_scene_head[i].scene_x_elems;
            const uint32_t _Y_ELEMS = _p_scene_head[i].scene_y_elems;
            for(j = 0; j < _Y_ELEMS; j++) {
                for(k = 0; k < _X_ELEMS; k++) {
                    if(cur_window_id == ((uint32_t*)(_p_scene_head[i].win_id_table))[j*_X_ELEMS + k]) {

                        /* UP */
                        if(j > 0) {
                            n_id.win_up_id = (_p_scene_head[i]).win_id_table[(j-1)*_X_ELEMS + k];
                        }

                        /* DOWN */
                        if((_Y_ELEMS > 1) && (j < _Y_ELEMS - 1)) {
                            n_id.win_down_id = (_p_scene_head[i]).win_id_table[(j+1)*_X_ELEMS + k];
                        }

                        /* LEFT */
                        if(k > 0) {
                            n_id.win_left_id = (_p_scene_head[i]).win_id_table[j*_X_ELEMS + k - 1];
                        }

                        /* RIGHT */
                        if((_X_ELEMS > 1) && (k < _X_ELEMS - 1)) {
                            n_id.win_right_id = (_p_scene_head[i]).win_id_table[j*_X_ELEMS + k + 1];
                        }

                        goto label_found_2;
                    }
                }
            }
        }
    }

label_found_2:

    return n_id;
}


lv_wms_window_t * lv_wms_scene_find_neighbor_window(uint32_t scene_id, uint32_t cur_window_id, uint32_t dir) {

    lv_wms_window_t * win = NULL;
    uint32_t w_id = lv_wms_scene_find_neighbor_window_id( scene_id,  cur_window_id,  dir);

    if(w_id != _INVALID_WIN_ID) {
        win = lv_wms_window_lookup(w_id);
    }

    return win;
}

void lv_wms_scene_enter_into_win(uint32_t scene_id, uint32_t win_id) {

    // It is forbidden to switch screen in window switch state
    if(lv_wms_is_in_busy_state())
        return;

    lv_wms_window_t * win = lv_wms_window_lookup(win_id);

    if(win != NULL) {
        lv_wms_push_win_stack(scene_id, win_id);
        //TODO: enable destroy to save memory, nut now HardFault
        //lv_wms_destroy_current();
        win->create_func(win_id);
    }

    // Refresh the screen directly when screen is changed
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    _lv_disp_refr_timer(disp->refr_timer);
}


void lv_wms_scene_enter_into(uint32_t scene_id) {
    uint16_t win_id = _INVALID_WIN_ID;

    for(uint16_t i = 0 ; i < _s_scene_count; i++) {
        if(_p_scene_head[i].scene_id == scene_id) {
            win_id = _p_scene_head[i].main_win_id; break;
        }
    }

    lv_wms_scene_enter_into_win(scene_id, win_id);

    return;
}


void lv_wms_scene_exit_cur_win(void) {

    // It is forbidden to switch screen in window switch state
    if(lv_wms_is_in_busy_state())
        return;

    if(lv_wms_is_root_scene()) {
        return;
    }

    uuid_t pre_uuid =  lv_wms_pop_win_stack();
    lv_wms_window_t * win = lv_wms_window_lookup(pre_uuid.id.window_id);

    if(win != NULL) {
        //TODO: enable destroy to save memory, nut now HardFault
        //lv_wms_destroy_current();
        win->create_func(pre_uuid.id.window_id);
    } else {
        printf("+++ EXCEPTIONS !!!\r\n");
    }

    // Refresh the screen directly when screen is changed
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    _lv_disp_refr_timer(disp->refr_timer);
}


void lv_wms_set_cur_scene_id(uint32_t scene_id) {
    _s_wms_scene_manage._cur_id.id.scene_id = scene_id;
}

uint32_t lv_wms_get_cur_scene_id(void) {
    return _s_wms_scene_manage._cur_id.id.scene_id;
}

void lv_wms_set_cur_win_id(uint32_t win_id) {
    _s_wms_scene_manage._cur_id.id.window_id = win_id;
}

uint32_t lv_wms_get_cur_win_id(void) {
    return _s_wms_scene_manage._cur_id.id.window_id;
}


uuid_t lv_wms_window_reset_stack(void) {
    int16_t i = 0;
    uuid_t empty_id = {
        .uid = 0,
    };

    for(i = _s_wms_scene_manage._win_stack_pos; i >= 0; i--) {
        _s_wms_scene_manage._win_stack[i] = empty_id;
    }

    _s_wms_scene_manage._win_stack_pos = 0;
    _s_wms_scene_manage._win_stack[0]  = _s_wms_scene_manage._root_id;
    _s_wms_scene_manage._cur_id        = _s_wms_scene_manage._root_id;

    return _s_wms_scene_manage._root_id;
}

/*
 * STATIC METHODS IMPLEMENT
 *****************************************************************************************
 */
static void lv_wms_register_scenes(lv_wms_scene_t * p_scenes, uint32_t scene_count) {
    _p_scene_head  = p_scenes;
    _s_scene_count = scene_count;
}


static void lv_wms_register_windows(lv_wms_window_map_t * p_windows, uint32_t window_count) {
    _p_window_head  = p_windows;
    _s_window_count = window_count;
}

static bool lv_wms_is_root_scene(void) {
    if(_s_wms_scene_manage._win_stack_pos > 0) {
        return false;
    } else {
        return true;
    }
}

static void lv_wms_push_win_stack(uint16_t scene_id , uint16_t win_id) {
    uuid_t uid = {
        .id.window_id = win_id,
        .id.scene_id  = scene_id,
    };

    if(_s_wms_scene_manage._win_stack[_s_wms_scene_manage._win_stack_pos].id.scene_id != scene_id) {
        /* update cur uuid into stack */
        _s_wms_scene_manage._win_stack[_s_wms_scene_manage._win_stack_pos] = _s_wms_scene_manage._cur_id;

        /* push new in stack */
        _s_wms_scene_manage._win_stack_pos ++;
        _s_wms_scene_manage._win_stack[_s_wms_scene_manage._win_stack_pos] = uid;

        /* set new cur uuid */
        _s_wms_scene_manage._cur_id = uid;
    } else {
        printf("+++ Same Scene, NOT Push!\r\n");
    }

    if(_s_wms_scene_manage._win_stack_pos >= MAX_WIN_STACK_DEPTH) {
        printf("+++ DANGEROUS, Stack Overflow !!! \r\n");
    }

    lv_wms_trace_win_stack();
}

static uuid_t lv_wms_pop_win_stack(void) {
    uuid_t empty_id = {
        .uid = 0,
    };

    if(_s_wms_scene_manage._win_stack_pos > 0) {
        _s_wms_scene_manage._win_stack[_s_wms_scene_manage._win_stack_pos] = empty_id;
        _s_wms_scene_manage._win_stack_pos --;
        _s_wms_scene_manage._cur_id = _s_wms_scene_manage._win_stack[_s_wms_scene_manage._win_stack_pos];
    }

    return _s_wms_scene_manage._cur_id;
}

static void lv_wms_trace_win_stack(void) {
    printf("+++ STACK +++ SCENE +++ WIN +++ \r\n");

    for(int i = _s_wms_scene_manage._win_stack_pos; i >=0 ; i--)
        printf("+++ %5d +++ %5d +++ %3d +++ \r\n", i, _s_wms_scene_manage._win_stack[i].id.scene_id, _s_wms_scene_manage._win_stack[i].id.window_id);
}

#endif /* #if WMS_VERSION == WMS_VERSION_v2 */


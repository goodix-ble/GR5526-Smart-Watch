/**
 * @file lv_wms.h
 *
 * @brief LVGL windows management service.
 */

#ifndef __GX_LV_WMS_H__
#define __GX_LV_WMS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include "lv_wms_surface_flinger.h"


#define WMS_VERSION_v1      1
#define WMS_VERSION_v2      2

#define WMS_VERSION         WMS_VERSION_v1


/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */
typedef enum {
    LV_WMS_SLIDE_IDLE_STATE = 0,
    LV_WMS_SLIDING_STATE,
}lv_wms_state_t;

/*
 * STRUCTURE DECLARATIONS
 *****************************************************************************************
 */
typedef lv_obj_t* (*lv_create_func_t)(void);
typedef void(*lv_destroy_func_t)(void);

#if WMS_VERSION == WMS_VERSION_v2

typedef lv_obj_t* (*lv_create_func_ex_t)(uint32_t);

typedef bool (*window_key_callback_func)(uint32_t, uint32_t);

typedef struct {
    lv_create_func_ex_t     create_func;    /* function pointer to create window */
    lv_obj_t *              window;         /* window object */
} lv_wms_window_t;

typedef struct {
    uint32_t                win_id;         /* window id */
    lv_wms_window_t *       win_object;     /* pointer to window object */
} lv_wms_window_map_t;

#endif

/*
 * FUNCTION DECLARATIONS
 *****************************************************************************************
 */
/**
 * Init a window manager for a screen
 * @param[in] obj pointer to a screen
 */
void lv_wms_init(lv_obj_t* obj);

/**
 * Set the destroy action for a screen
 * @param[in] obj pointer to a screen
 * @param[in] func pointer to a function when exit screen
 */
void lv_wms_self_destroy_func_set(lv_obj_t* obj, lv_destroy_func_t func);

/**
 * Set the create action for the left screen
 * @param[in] obj pointer to a screen
 * @param[in] func pointer to a function when enter screen
 */
void lv_wms_left_create_func_set(lv_obj_t* obj, lv_create_func_t func);

/**
 * Set the destroy action for the right screen
 * @param[in] obj pointer to a screen
 * @param[in] func pointer to a function when enter the right screen
 */
void lv_wms_right_create_func_set(lv_obj_t* obj, lv_create_func_t func);

/**
 * Set the create action for the bottom screen
 * @param[in] obj pointer to a screen
 * @param[in] func pointer to a function when enter the bottom screen
 */
void lv_wms_bottom_create_func_set(lv_obj_t* obj, lv_create_func_t func);

/**
 * Set the create action for the top screen
 * @param[in] obj pointer to a screen
 * @param[in] func pointer to a function when enter the top screen
 */
void lv_wms_top_create_func_set(lv_obj_t* obj, lv_create_func_t func);


/**
 * Set a global step for the wms animation
 * @param[in] step for the wms animation
 */
void lv_wms_ani_step_set(uint8_t step);

/**
 * Deinit a window manager for a screen
 * @param[in] obj pointer to a screen
 */
void lv_wms_deinit(lv_obj_t* obj);

/**
 * Update the inside scroll direction for a screen
 * @note needed when the screen children numbers or sizes is changed
 * @note the screen scroll direction will be affected by children
 * @param[in] obj pointer to a screen
 */
void lv_wms_inside_scroll_dir_update(lv_obj_t* obj);

/**
 * Add object flags for children recursively
 * @param[in] obj pointer to a screen
 * @param[in] flag for object
 */
void lv_wms_add_flag_recursive_children(lv_obj_t* obj, lv_obj_flag_t flag);

/**
 * Add object flags for direct children
 * @param[in] obj pointer to a screen
 * @param[in] flag for object
 */
void lv_wms_add_flag_for_direct_child(lv_obj_t* obj, lv_obj_flag_t flag);

/**
 * Clear object flags for children recursively
 * @param[in] obj pointer to a screen
 * @param[in] flag for object
 */
void lv_wms_clear_flag_recursive_children(lv_obj_t* obj, lv_obj_flag_t flag);

/**
 * Clear object flags for direct children
 * @param[in] obj pointer to a screen
 * @param[in] flag for object
 */
void lv_wms_clear_flag_for_direct_child(lv_obj_t* obj, lv_obj_flag_t flag);

/**
 * Check if the WMS is in the view switching state
 * @return true or false
 */
bool lv_wms_is_in_busy_state(void);

/**
 * Direct Go to the target screen from the current screen
 * @param[in] next_create_func for next screen create
 */
void lv_wms_go_to_window(lv_create_func_t next_create_func);

#if WMS_VERSION == WMS_VERSION_v2

/**
 * Set the window handler for key event
 * @param[in] obj  : pointer to a screen
 * @param[in] func : pointer to a key event handler function
 */
void lv_wms_key_handler_func_set(lv_obj_t* obj, window_key_callback_func func);

/**
 * execute the key event handler
 * @param[in] obj : pointer to a screen
 * @param[in] key : key value to identify which key
 * @param[in] event : event value to identify event type, refer to @app_key_click_type_t
 * return : true if handled, else false
 */
bool lv_wms_key_handler_execute(lv_obj_t* obj, uint32_t key, uint32_t event);

/**
 * Update up/down/left/right neighbor window id
 * @param[in] obj : pointer to a screen
 * @param[in] win_id : current window id
 */
void lv_wms_update_neighbor_setting(lv_obj_t* obj, uint32_t win_id) ;

/**
 * destroy current window object
 */
void lv_wms_destroy_current(void);

#endif  /* WMS_VERSION == WMS_VERSION_v2 */

#endif

#ifndef __LV_LAYOUT_APP_MENU_H__
#define __LV_LAYOUT_APP_MENU_H__

#include <stdint.h>
#include <stdbool.h>
#include "lvgl.h"

typedef enum
{
    APP_MENU_STYLE_LINEAR_LIST,
    APP_MENU_STYLE_CIRCULAR_LIST,
    APP_MENU_STYLE_SPHERE_GRID,
    APP_MENU_STYLE_TRIFORM_GRID,
    APP_MENU_STYLE_DIALER,
    APP_MENU_STYLE_MAX,
} app_menu_style_t;

/**
 * @brief Create an application menu layout object
 *
 * This function creates an application menu layout object and attaches it to the specified parent object.
 *
 * @param parent Pointer to the parent object
 * @return Pointer to the created application menu layout object
 */
lv_obj_t *lv_layout_app_menu_create(lv_obj_t *parent);


/**
 * @brief Get the current style of the application menu
 *
 * This function retrieves the current style of the application menu.
 *
 * @return The current style of the application menu
 */
app_menu_style_t app_menu_get_style(void);


/**
 * @brief Set the style of the application menu
 *
 * This function sets the style of the application menu.
 *
 * @param style The style to be set for the application menu
 */
void app_menu_set_style(app_menu_style_t style);


/**
 * @brief Check if gesture navigation is available for the specified direction in the application menu
 *
 * This function checks if gesture navigation is available for the specified direction in the application menu within the specified object.
 *
 * @param obj Pointer to the application menu object
 * @param gesture_dir The direction of the gesture
 * @return true if gesture navigation is available for the specified direction, false otherwise
 */
bool app_menu_gesture_available(lv_obj_t *obj, lv_dir_t gesture_dir);


#endif // __LV_LAYOUT_APP_MENU_H__

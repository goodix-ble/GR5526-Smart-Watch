/**
 * @file lv_enhanced_grid.h
 *
 */

#ifndef __LV_ENHANCED_GRID_H__
#define __LV_ENHANCED_GRID_H__

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"
#include <stdint.h>
#include "lv_enhanced_item.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**
 * @brief Size translation callback type for enhanced grid
 *
 * This callback type is used for translating the size of grid cells from its position in an enhanced grid object.
 * The callback takes the item position and the item itself as input and returns the translated size as a float.
 * Return 1 means no scale-up or scale-down.
 * Any value below 0.01 will be treated as "hidden".
 *
 * @param item_pos Pointer to the position of the item in the grid
 * @param item Pointer to the enhanced menu item
 * @return Translated size of the grid cell as a float
 */
typedef float (*lv_enhanced_grid_size_translate_cb_t)(lv_point_t *item_pos, const lv_enhanced_menu_item_t *item);

typedef struct
{
    lv_obj_t obj;
    uint8_t item_per_row;
    bool hex_layout;
    uint16_t item_count;

    lv_coord_t cell_width;
    lv_coord_t cell_height;
    lv_coord_t grid_padding_hor;
    lv_coord_t grid_padding_ver;

    const lv_enhanced_menu_item_t *items;

    lv_enhanced_grid_size_translate_cb_t size_translate_cb;

    uint16_t clicked_index;

    struct
    {
        bool focused;
        lv_dir_t scroll_dir;

        lv_coord_t hor_offset;
        lv_coord_t hor_offset_min;
        lv_coord_t hor_offset_max;
        lv_coord_t hor_momentum;

        lv_coord_t ver_offset;
        lv_coord_t ver_offset_min;
        lv_coord_t ver_offset_max;
        lv_coord_t ver_momentum;

        lv_timer_t *scroll_tmr;
    } scroll;
} lv_enhanced_grid_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * @brief Create an enhanced grid object
 *
 * This function creates an enhanced grid object and attaches it to the specified parent object.
 *
 * @param parent Pointer to the parent object
 * @return Pointer to the created enhanced grid object
 */
lv_obj_t *lv_enhanced_grid_create(lv_obj_t *parent);

/**
 * @brief Set the size translation callback for the enhanced grid
 *
 * This function sets the size translation callback for the enhanced grid object.
 * The callback is responsible for translating the size of grid cells from its position based on custom logic.
 *
 * @param obj Pointer to the enhanced grid object
 * @param size_translate_cb Size translation callback function
 */
void lv_enhanced_grid_set_size_translate_cb(lv_obj_t *obj, lv_enhanced_grid_size_translate_cb_t size_translate_cb);

/**
 * @brief Set the cell size for the enhanced grid
 *
 * This function sets the cell size for the enhanced grid object.
 * Every icon in the grid is considered as a cell. The size of the cell won't affect the size of the icon.
 *
 * @param obj Pointer to the enhanced grid object
 * @param cell_width Width of cell
 * @param cell_height Height of cell
 */
void lv_enhanced_grid_set_cell_size(lv_obj_t *obj, lv_coord_t cell_width, lv_coord_t cell_height);

/**
 * @brief Set the horizontal and verticall padding for the enhanced grid
 *
 * This function sets the horizontal and vertical padding for the enhanced grid object.
 *
 * @param obj Pointer to the enhanced grid object
 * @param pad_hor Horizontal padding
 * @param pad_ver Vertical padding
 */
void lv_enhanced_grid_set_padding(lv_obj_t *obj, lv_coord_t pad_hor, lv_coord_t pad_ver);

/**
 * @brief Set the items for the enhanced grid
 *
 * This function sets the items for the enhanced grid object. The items are provided as an array
 * and are displayed in a grid layout with the specified number of items per row.
 *
 * @param obj Pointer to the enhanced grid object
 * @param items Array of enhanced menu items
 * @param item_count Number of items in the array
 * @param item_per_row Number of items to display per row in the grid
 */
void lv_enhanced_grid_set_items(lv_obj_t *obj, const lv_enhanced_menu_item_t items[], uint16_t item_count, uint8_t item_per_row);

/**
 * @brief Set the scroll direction for the enhanced grid
 *
 * This function sets the scroll direction for the enhanced grid object.
 *
 * @param obj Pointer to the enhanced grid object
 * @param scroll_dir Scroll direction to set
 */
void lv_enhanced_grid_set_scroll_dir(lv_obj_t *obj, lv_dir_t scroll_dir);

/**
 * @brief Set the hexagonal layout mode for the enhanced grid
 *
 * This function sets the hexagonal layout mode for the enhanced grid object.
 *
 * @param obj Pointer to the enhanced grid object
 * @param use_hex_layout Set to true to enable hexagonal layout, false otherwise
 */
void lv_enhanced_grid_set_hex_layout(lv_obj_t *obj, bool use_hex_layout);

/**
 * @brief Get the scroll offset of the enhanced grid
 *
 * This function returns the current scroll offset of the enhanced grid object.
 *
 * @param obj Pointer to the enhanced grid object
 * @return Scroll offset as a point (x, y)
 */
lv_point_t lv_enhanced_grid_get_scroll_offset(lv_obj_t *obj);

/**
 * @brief Set the scroll offset of the enhanced grid
 *
 * This function sets the scroll offset of the enhanced grid object.
 *
 * @param obj Pointer to the enhanced grid object
 * @param offset Scroll offset as a point (x, y)
 * @param anim Set to true to animate the scrolling, false otherwise
 */
void lv_enhanced_grid_set_scroll_offset(lv_obj_t *obj, lv_point_t offset, bool anim);

/**
 * @brief Scroll the enhanced grid to the center
 *
 * This function scrolls the enhanced grid object to the center position.
 *
 * @param obj Pointer to the enhanced grid object
 * @param anim Set to true to animate the scrolling, false otherwise
 */
void lv_enhanced_grid_scroll_to_center(lv_obj_t *obj, bool anim);

/**
 * @brief Get the clicked item in the enhanced grid
 *
 * This function returns the clicked item in the enhanced grid object.
 *
 * @param obj Pointer to the enhanced grid object
 * @return Pointer to the clicked item, or NULL if no item was clicked
 */
const lv_enhanced_menu_item_t *lv_enhanced_grid_get_clicked_item(lv_obj_t *obj);

/**
 * @brief Check if the enhanced grid is on an edge
 *
 * This function checks if the enhanced grid is currently positioned on an edge.
 *
 * @return The direction of the edge if the grid is on an edge, or LV_DIR_NONE if it is not on an edge
 */
lv_dir_t lv_enhanced_grid_is_on_edge(lv_obj_t *obj);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __LV_ENHANCED_GRID_H__

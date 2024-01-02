#ifndef __LV_ENHANCED_LIST_H__
#define __LV_ENHANCED_LIST_H__

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
#include <stdint.h>

#include "lvgl.h"
#include "lv_enhanced_item.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

typedef struct
{
    uint16_t width;
    uint16_t height;
    lv_coord_t offset;
    lv_coord_t anim_step;
    uint8_t *buf;
} _txtimg_cache_dsc_t;

/**
 * @brief X Translate callback type for enhanced list
 *
 * This callback type is used for translating the x-position from its y-position of list items in an enhanced list object.
 * The callback takes the object's y-coordinate and the item itself as input and returns the translated x-coordinate as an lv_coord_t value.
 *
 * @param obj_y The y-coordinate of the list object
 * @param item Pointer to the enhanced menu item
 * @return Translated x-coordinate of the list item as an lv_coord_t value
 */
typedef lv_coord_t (*lv_enhanced_list_translate_cb_t)(lv_coord_t obj_y, const lv_enhanced_menu_item_t *item);

typedef struct
{
    lv_obj_t obj;
    lv_enhanced_list_translate_cb_t translate_cb;
    const lv_enhanced_menu_item_t *items;
    uint16_t item_count;
    uint16_t clicked_index;
    _txtimg_cache_dsc_t *cache_dsc;
    lv_timer_t *caption_tmr;

    struct
    {
        lv_coord_t ver_offset;
        lv_coord_t max_offset;
        lv_coord_t max_offset_sb;
        lv_coord_t momentum;
        bool focused;
        lv_timer_t *scroll_tmr;
    } scroll;
} lv_enhanced_list_t;

extern const lv_obj_class_t lv_enhanced_list_class;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * @brief Create an enhanced list object
 *
 * This function creates an enhanced list object and attaches it to the specified parent object.
 *
 * @param parent Pointer to the parent object
 * @return Pointer to the created enhanced list object
 */
lv_obj_t *lv_enhanced_list_create(lv_obj_t *parent);

/**
 * Sets the translate callback for an enhanced list object.
 *
 * This function sets the translate callback for an enhanced list object, which
 * is used to translate the y-coordinate of a list item to its corresponding
 * x-offset in the list. The translate callback takes two arguments: the
 * y-coordinate of the list item and a pointer to the list item itself. It
 * returns the x-offset of the list item in the list.
 *
 * @param obj A pointer to the enhanced list object.
 * @param translate_cb The translate callback function to set.
 */
void lv_enhanced_list_set_translate_cb(lv_obj_t *obj, lv_enhanced_list_translate_cb_t translate_cb);

/**
 * Sets the items for an enhanced list object.
 *
 * @param obj Pointer to the enhanced list object.
 * @param items Array of `lv_enhanced_menu_item_t` structures representing
 *              the items to set.
 * @param item_count Number of items in the `items` array.
 */
void lv_enhanced_list_set_items(lv_obj_t *obj, const lv_enhanced_menu_item_t items[], uint16_t item_count);

/**
 * Gets the clicked item for an enhanced list object.
 *
 * This function returns a pointer to the `lv_enhanced_menu_item_t` structure
 * representing the item that was clicked on in the enhanced list object. If no
 * item was clicked, the function returns `NULL`.
 *
 * @param obj A pointer to the enhanced list object.
 * @return A pointer to the clicked `lv_enhanced_menu_item_t` structure, or
 *         `NULL` if no item was clicked.
 */

const lv_enhanced_menu_item_t *lv_enhanced_list_get_clicked_item(lv_obj_t *obj);

/**
 * Get the current scroll offset the list.
 *
 * @param obj Pointer to the enhanced list object.
 * @return The current scroll offset of the list in pixels.
 */
lv_coord_t lv_enhanced_list_get_scroll_offset(lv_obj_t *obj);

/**
 * Sets the scroll position of an enhanced list with optional animation.
 *
 * If the `anim` parameter is set to true, the position will be animated to the new value,
 * otherwise it will be set immediately.
 *
 * @param obj Pointer to the enhanced list object.
 * @param offset The new scroll position to set.
 * @param anim If true, the scroll position will be set with an animation; if false, it will be set immediately.
 */
void lv_enhanced_list_set_scroll_offset(lv_obj_t *obj, lv_coord_t offset, bool anim);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __LV_ENHANCED_LIST_H__

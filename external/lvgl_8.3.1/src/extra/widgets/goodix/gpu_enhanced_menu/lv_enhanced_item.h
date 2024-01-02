#ifndef __LV_ENHANCED_ITEM_H__
#define __LV_ENHANCED_ITEM_H__

#include "lvgl.h"

typedef struct
{
    const lv_img_dsc_t *icon;
    const char *caption;
    void *user_data;
} lv_enhanced_menu_item_t;

#endif // __LV_ENHANCED_ITEM_H__

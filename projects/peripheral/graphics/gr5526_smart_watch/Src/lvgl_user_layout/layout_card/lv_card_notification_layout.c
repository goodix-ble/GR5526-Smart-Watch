#include "lvgl.h"
#include "lv_img_dsc_list.h"
#include "lv_font.h"
#include "lv_layout_router.h"
#include "notification_center.h"
#include "app_graphics_mem.h"

#define NOTIFICATION_ITEM_PAD_ROW  30
#define NOTIFICATION_ITEM_PAD_LEFT ((DISP_HOR_RES - NOTIFICATION_ITEM_WIDTH) / 2)

LV_FONT_DECLARE(lv_font_msyh_26_gr552x);

static const lv_style_const_prop_t TITLE_LABEL_STYLE_PROPS[] = {
    LV_STYLE_CONST_WIDTH(200),
    LV_STYLE_CONST_HEIGHT(40),
    LV_STYLE_CONST_X(127),
    LV_STYLE_CONST_Y(40),
    LV_STYLE_CONST_TEXT_ALIGN(LV_TEXT_ALIGN_CENTER),
    LV_STYLE_CONST_TEXT_FONT(&lv_font_msyh_26_gr552x),
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(TITLE_LABEL_STYLE, TITLE_LABEL_STYLE_PROPS);

lv_obj_t *lv_card_notification_layout_create(lv_obj_t *parent)
{
    lv_obj_t *p_window = lv_obj_create(parent);

    lv_obj_enable_style_refresh(false); // SPEEEEEED UP Creation process

    // Notification Title
    lv_obj_t *p_title = lv_label_create(p_window);
    lv_obj_add_style(p_title, (lv_style_t *)&TITLE_LABEL_STYLE, LV_PART_MAIN);
    lv_label_set_text_static(p_title, "Notification");

    // Notification List
    lv_obj_t *p_list = lv_obj_create(p_window);
    lv_obj_set_scroll_dir(p_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(p_list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_bottom(p_list, 80, LV_PART_MAIN);
    lv_obj_set_size(p_list, DISP_HOR_RES - NOTIFICATION_ITEM_PAD_LEFT, DISP_VER_RES - 80);
    lv_obj_set_pos(p_list, NOTIFICATION_ITEM_PAD_LEFT, 80);

    notification_info_t *p_info = notification_center_get_header();
    uint32_t info_cnt = notification_center_get_info_count();

    for (uint32_t i = 0; i < info_cnt; i++)
    {
        lv_obj_t *item = lv_img_create(p_list);
        lv_img_set_src(p_list->spec_attr->children[i], p_info->thumbnail);
        lv_obj_set_pos(item, 0, i * (NOTIFICATION_ITEM_HEIGHT + NOTIFICATION_ITEM_PAD_ROW));
        p_info = p_info->next;
    }
    lv_obj_enable_style_refresh(true);

    lv_layout_router_set_scrollbar_target(p_list);

    return p_window;
}

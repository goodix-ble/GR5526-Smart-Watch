#include <stdio.h>
#include "lvgl.h"
#include "lv_font.h"
#include "lv_user_font.h"
#include "lv_layout_app_menu.h"

#define LIST_MENU_TITLE_HEIGHT      70
#define ISTR_MARK(TXT)              TXT
#define ISTR(TXT)                   TXT


LV_FONT_DECLARE(lv_font_msyh_48_gr552x);
LV_FONT_DECLARE(lv_font_msyh_26_gr552x);

static void lv_pwr_slider_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = lv_event_get_current_target(evt);
    if (lv_slider_get_value(obj) == 100)
    {
        printf("TODO: Power Off\r\n");
    }
    else
    {
        lv_slider_set_value(obj, 0, LV_ANIM_ON);
    }
}

lv_obj_t *lv_layout_pwr_off_layout_create(lv_obj_t * parent)
{
    lv_obj_t *p_window = lv_obj_create(parent);
    lv_obj_set_size(p_window, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_set_scrollbar_mode(p_window, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(p_window, LV_DIR_NONE);
    lv_obj_clear_flag(p_window, LV_OBJ_FLAG_SCROLLABLE);

    // Slider BG with text
    lv_obj_t *p_bg = lv_label_create(p_window);
    lv_obj_set_style_text_font(p_bg, &lv_font_msyh_26_gr552x, LV_STATE_DEFAULT);
    lv_label_set_text(p_bg, ISTR("Power Off"));
    lv_obj_set_style_bg_color(p_bg, lv_color_make(0x0F, 0x33, 0x64), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(p_bg, LV_OPA_COVER, LV_STATE_DEFAULT);
    lv_obj_set_style_radius(p_bg, LV_RADIUS_CIRCLE, LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(p_bg, LV_TEXT_ALIGN_CENTER, LV_STATE_DEFAULT);
    lv_obj_set_style_pad_ver(p_bg, 25, LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(p_bg, 20, LV_STATE_DEFAULT);

    lv_obj_set_style_text_color(p_bg, lv_palette_darken(LV_PALETTE_GREY, 2), LV_STATE_DEFAULT);
    lv_obj_set_size(p_bg, 280, 80);
    lv_obj_center(p_bg);

    // Slider
    lv_obj_t *p_slider = lv_slider_create(p_window);
    lv_obj_set_size(p_slider, 280, 80);
    lv_obj_center(p_slider);
    lv_obj_add_flag(p_slider, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_set_style_anim_time(p_slider, 100, LV_STATE_DEFAULT);
    lv_obj_add_event_cb(p_slider, lv_pwr_slider_event_cb, LV_EVENT_RELEASED, NULL);

    lv_obj_set_style_bg_opa(p_slider, 0, LV_PART_MAIN);

    lv_obj_set_style_bg_color(p_slider, lv_color_make(0x1F, 0x67, 0xC9), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(p_slider, LV_OPA_90, LV_PART_INDICATOR);

    lv_obj_set_style_pad_all(p_slider, -2, LV_PART_KNOB);
    lv_obj_set_style_bg_color(p_slider, lv_color_white(), LV_PART_KNOB);

    return p_window;
}

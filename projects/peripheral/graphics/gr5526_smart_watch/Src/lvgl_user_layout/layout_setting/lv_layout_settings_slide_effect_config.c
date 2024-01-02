#include <stdio.h>
#include "lvgl.h"
#include "lv_font.h"
#include "lv_user_font.h"

#define LIST_MENU_TITLE_HEIGHT      70
#define ISTR_MARK(TXT)              TXT
#define ISTR(TXT)                   TXT


static const char *SWITCH_EFFECT_OPTIONS = ISTR_MARK("Linear\nCube\nInner Cube\nStack\nFade\nFade Zoom\nSpin\nPush Pull");


static lv_style_t s_roller_style;
static lv_style_t s_roller_selected_style;

extern void     lv_layout_router_set_slide_effect(uint32_t effect);
extern uint32_t lv_layout_router_get_slide_effect(void);

static void roller_event_handler(lv_event_t *evt)
{
    uint16_t selected = lv_roller_get_selected(evt->current_target);
    uint32_t effect   = LV_WMS_TILEVIEW_EFFECT_CUBE;

    switch(selected) {
        case 0: effect = LV_WMS_TILEVIEW_EFFECT_LINEAR;    break;
        case 1: effect = LV_WMS_TILEVIEW_EFFECT_CUBE;           break;
        case 2: effect = LV_WMS_TILEVIEW_EFFECT_INNERCUBE;      break;
        case 3: effect = LV_WMS_TILEVIEW_EFFECT_STACK;          break;
        case 4: effect = LV_WMS_TILEVIEW_EFFECT_FADE;           break;
        case 5: effect = LV_WMS_TILEVIEW_EFFECT_FADE_ZOOM;      break;
        case 6: effect = LV_WMS_TILEVIEW_EFFECT_SPIN;           break;
        case 7: effect = LV_WMS_TILEVIEW_EFFECT_PUSHPULL;       break;
        default: break;
    }
    lv_layout_router_set_slide_effect(effect);
}

static uint32_t _get_transit_effect(void) {
    uint32_t effect = lv_layout_router_get_slide_effect();

    if(effect == LV_WMS_TILEVIEW_EFFECT_LINEAR) {
        return 0;
    } else if (effect == LV_WMS_TILEVIEW_EFFECT_CUBE) {
        return 1;
    } else if (effect == LV_WMS_TILEVIEW_EFFECT_INNERCUBE) {
        return 2;
    } else if (effect == LV_WMS_TILEVIEW_EFFECT_STACK) {
        return 3;
    } else if (effect == LV_WMS_TILEVIEW_EFFECT_FADE) {
        return 4;
    } else if (effect == LV_WMS_TILEVIEW_EFFECT_FADE_ZOOM) {
        return 5;
    } else if (effect == LV_WMS_TILEVIEW_EFFECT_SPIN) {
        return 6;
    } else if (effect == LV_WMS_TILEVIEW_EFFECT_PUSHPULL) {
        return 7;
    }

    return 1;
}


extern lv_obj_t * lv_layout_setting_create_common_title(lv_obj_t * parent, lv_font_t * font, char * text);

lv_obj_t *lv_layout_settings_switch_effect_create(lv_obj_t * parent)
{
    lv_obj_t *p_window = lv_layout_setting_create_common_title(parent, &lv_font_montserrat_36, "Slide Effect");

    // Roller
    lv_obj_t *p_roller = lv_roller_create(p_window);
    lv_obj_set_size(p_roller, DISP_HOR_RES, DISP_VER_RES - LIST_MENU_TITLE_HEIGHT*2);
    lv_obj_set_pos(p_roller, 0, LIST_MENU_TITLE_HEIGHT + 20);
    lv_obj_set_style_bg_color(p_roller, lv_color_black(), 0);
    lv_obj_set_style_pad_top(p_roller, 8, LV_STATE_DEFAULT);
    lv_roller_set_options(p_roller, ISTR(SWITCH_EFFECT_OPTIONS), LV_ROLLER_MODE_NORMAL);
    lv_obj_add_event_cb(p_roller, roller_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
    lv_roller_set_selected(p_roller, _get_transit_effect(), LV_ANIM_OFF);

    // Roller Style
    lv_style_init(&s_roller_style);
    lv_style_set_text_color(&s_roller_style, lv_palette_darken(LV_PALETTE_GREY, 2));
    lv_style_set_text_font(&s_roller_style, &lv_font_montserrat_36);
    lv_style_set_text_line_space(&s_roller_style, 20);
    lv_style_set_border_width(&s_roller_style, 0);
    lv_style_set_pad_top(&s_roller_style, 0);

    // Roller Selected Style
    lv_style_init(&s_roller_selected_style);
    lv_style_set_bg_color(&s_roller_selected_style, lv_palette_darken(LV_PALETTE_GREY, 4));
    lv_style_set_text_color(&s_roller_selected_style, lv_palette_lighten(LV_PALETTE_CYAN, 1));
    lv_style_set_text_font(&s_roller_selected_style, &lv_font_montserrat_36);

    lv_obj_add_style(p_roller, &s_roller_style, LV_PART_MAIN);
    lv_obj_add_style(p_roller, &s_roller_selected_style, LV_PART_SELECTED);

    lv_obj_align(p_roller, LV_ALIGN_TOP_MID, 120, LIST_MENU_TITLE_HEIGHT + 24);

    return p_window;
}

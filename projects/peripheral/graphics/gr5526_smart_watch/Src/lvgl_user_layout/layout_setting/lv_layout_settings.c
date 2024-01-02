#include <stdio.h>
#include "lvgl.h"
#include "lv_font.h"
#include "lv_user_font.h"
#include "lv_img_dsc_list.h"
#include "lv_port_disp.h"
#include "lv_layout_router.h"
#include "hal_gfx_cmdlist.h"
#include "app_graphics_mem.h"
#include "lv_layout_router.h"

#define __DBG_DO_RAM_CACHE          0
#define LIST_MENU_TITLE_HEIGHT      70
#define ISTR_MARK(TXT)              TXT
#define ISTR(TXT)                   TXT


#define ARRAY_SIZE(arr)             (sizeof(arr) / sizeof(arr[0]))

typedef struct
{
    uint16_t start;
    uint16_t end;
} disp_safe_area_t;

extern void                 lv_port_res_mode_set(uint8_t mode);
extern hal_gfx_cmdlist_t *  lv_port_get_current_cl(void);

typedef struct
{
    char *          caption;
    lv_img_dsc_t *  icon_dsc;
    uint32_t        win_id;
} settings_item_t;

static lv_font_t *s_main_font;

static const settings_item_t SETTINGS_ITEM[] = {
    {
        .caption  = ISTR_MARK("Display"),
        .icon_dsc = (lv_img_dsc_t *)&wd_img_SETTINGS_01_DISPLAY,
        .win_id   = WMS_WID_NONE,
    },
    {
        .caption  = ISTR_MARK("List Style"),
        .icon_dsc = (lv_img_dsc_t *)&wd_img_SETTINGS_02_APPLIST_STYLE,
        .win_id   = WMS_WID_SETTING_LIST_STYLE,
    },
    {
        .caption  = ISTR_MARK("Slide Effect"),
        .icon_dsc = (lv_img_dsc_t *)&wd_img_SETTINGS_03_SWITCH_EFFECT,
        .win_id   = WMS_WID_SETTING_SWITCH_EFFECT,
    },
    {
        .caption  = ISTR_MARK("Card Style"),
        .icon_dsc = (lv_img_dsc_t *)&wd_img_SETTINGS_02_APPLIST_STYLE,
        .win_id   = WMS_WID_SETTING_CARD_STYLE,
    },
    {
        .caption  = ISTR_MARK("Language"),
        .icon_dsc = (lv_img_dsc_t *)&wd_img_SETTINGS_04_LANGUAGE,
        .win_id   = WMS_WID_NONE,
    },
    {
        .caption  = ISTR_MARK("System"),
        .icon_dsc = (lv_img_dsc_t *)&wd_img_SETTINGS_05_SYSTEM,
        .win_id   = WMS_WID_NONE,
    },
    {
        .caption  = ISTR_MARK("About"),
        .icon_dsc = (lv_img_dsc_t *)&wd_img_SETTINGS_06_ABOUT,
        .win_id   = WMS_WID_NONE,
    },
};

#if __DBG_DO_RAM_CACHE
static void *s_resource_cache_pool[ARRAY_SIZE(SETTINGS_ITEM) * 2];
#endif // __DBG_DO_RAM_CACHE


static void lv_settings_item_clicked_callback(lv_event_t *evt)
{
    settings_item_t *p_item = (settings_item_t *)evt->user_data;
    lv_layout_router_goto_isolate_win(evt->current_target, p_item->win_id, WMS_DEFAULT_GOTO_EFFECT_POS, WMS_DEFAULT_GOTO_EFFECT);
}

#if __DBG_DO_RAM_CACHE
static void create_settings_item_ram_cache(void)
{
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.font = s_main_font;
    label_dsc.color = lv_color_white();

    lv_disp_t *disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t *orig_drawbuf = disp->driver->draw_buf;

    lv_wms_refresh_enabled_set(false);
    hal_gfx_cmdlist_t cl = hal_gfx_cl_le_create();
    hal_gfx_cl_bind_circular(&cl);

    for (uint32_t i = 0; i < ARRAY_SIZE(SETTINGS_ITEM); i++)
    {
        // Load Image into PSRAM
        lv_img_dsc_t *p_cache = (lv_img_dsc_t *)app_graphics_mem_malloc(sizeof(lv_img_dsc_t) + SETTINGS_ITEM[i].icon_dsc->data_size);
        memcpy(p_cache, SETTINGS_ITEM[i].icon_dsc, sizeof(lv_img_dsc_t));
        p_cache->data = (uint8_t *)&p_cache[1];
        // Change MMAP Endian to Mode 2 before copy any resource from external flash
        lv_port_res_mode_set(2);
        memcpy((void *)p_cache->data, (void *)SETTINGS_ITEM[i].icon_dsc->data, p_cache->data_size);
        s_resource_cache_pool[i] = p_cache;

        // Pre-Render Caption and treat as an image
        lv_point_t textarea;
        lv_txt_get_size(&textarea, ISTR(SETTINGS_ITEM[i].caption), s_main_font, label_dsc.letter_space, label_dsc.line_space, 280, label_dsc.flag);
        lv_img_dsc_t *p_caption_img = (lv_img_dsc_t *)app_graphics_mem_malloc(sizeof(lv_img_dsc_t) + textarea.x * textarea.y * 2);
        p_caption_img->header.always_zero = 0;
        p_caption_img->header.w = textarea.x;
        p_caption_img->header.h = textarea.y;
        p_caption_img->header.cf = LV_IMG_CF_GDX_RGB565;
        p_caption_img->data_size = textarea.x * textarea.y * 2;
        p_caption_img->data = (uint8_t *)&p_caption_img[1];
        memset((void *)p_caption_img->data, 0, p_caption_img->data_size);
        lv_disp_draw_buf_t draw_buf;
        lv_disp_draw_buf_init(&draw_buf, (void *)p_caption_img->data, NULL, textarea.x * textarea.y);
#if 0
        draw_buf.area.x1 = 0;
        draw_buf.area.y1 = 0;
        draw_buf.area.x2 = textarea.x - 1;
        draw_buf.area.y2 = textarea.y - 1;
#endif
        disp->driver->draw_buf = &draw_buf;
        lv_area_t area = {0, 0, textarea.x, textarea.y};
        // Ensure that the rendering is committed to the right command list
        lv_draw_label(disp->driver->draw_ctx, &label_dsc, &area, ISTR(SETTINGS_ITEM[i].caption), NULL);
        s_resource_cache_pool[ARRAY_SIZE(SETTINGS_ITEM) + i] = p_caption_img;
    }

    disp->driver->draw_buf = orig_drawbuf;
    hal_gfx_cl_le_destroy(&cl);
    lv_wms_refresh_enabled_set(true);
}
#endif

lv_obj_t * lv_layout_setting_create_common_title(lv_obj_t * parent, lv_font_t * font, char * text) {

    lv_obj_t *p_window = lv_obj_create(parent);
    lv_obj_set_size(p_window, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_set_scrollbar_mode(p_window, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(p_window, LV_DIR_NONE);

    // Title
    lv_obj_t *p_title = lv_label_create(p_window);
    lv_obj_set_style_text_font(p_title, font, LV_STATE_DEFAULT);
    lv_label_set_text(p_title, ISTR(text));
    lv_obj_set_style_pad_top(p_title, 40, LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(p_title, 20, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(p_title, lv_color_white(), LV_STATE_DEFAULT);
    lv_obj_set_size(p_title, DISP_HOR_RES, LIST_MENU_TITLE_HEIGHT);
    lv_obj_set_pos(p_title, 0, 0);
    lv_obj_set_style_text_align(p_title, LV_TEXT_ALIGN_CENTER, LV_STATE_DEFAULT);

    // Seperate Line
    static lv_point_t sep_line_points[2] = {{0, 0}, {DISP_HOR_RES, 0}};
    lv_obj_t *p_sep_line = lv_line_create(p_window);
    lv_obj_set_style_line_rounded(p_sep_line, true, 0);
    lv_obj_set_style_line_width(p_sep_line, 4, 0);
    lv_obj_set_style_line_color(p_sep_line, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_clear_flag(p_sep_line, LV_OBJ_FLAG_CLICKABLE);
    lv_line_set_points(p_sep_line, sep_line_points, 2);
    lv_obj_align_to(p_sep_line, p_title, LV_ALIGN_OUT_BOTTOM_MID, 0, 24);

    return p_window;
}


lv_obj_t *lv_layout_setting_create(lv_obj_t * parent)
{
    if (1)
    {
        s_main_font = &lv_font_montserrat_36;//lv_font_msyh_32_gr552x;
    }
    else
    {
        s_main_font = &lv_font_montserrat_36;//lv_font_msyh_36_gr552x;
    }

    lv_obj_t *p_window = lv_layout_setting_create_common_title(parent, s_main_font, "Setting");

    // List Area
    lv_obj_t *p_list = lv_obj_create(p_window);
    lv_obj_set_size(p_list, LV_SIZE_CONTENT, DISP_VER_RES - LIST_MENU_TITLE_HEIGHT);
    lv_obj_set_scrollbar_mode(p_list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(p_list, LV_DIR_VER);
    lv_obj_add_flag(p_list, LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_ELASTIC);
    lv_obj_set_style_text_font(p_list, s_main_font, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(p_list, lv_color_white(), LV_PART_ANY);
    lv_obj_set_style_pad_hor(p_list, 0, 0);
    lv_obj_set_style_pad_top(p_list, 0, 0);
    lv_obj_set_style_pad_bottom(p_list, 0, 0);
    lv_obj_set_flex_flow(p_list, LV_FLEX_FLOW_COLUMN);
    //lv_obj_align_to(p_list, p_title, LV_ALIGN_OUT_BOTTOM_MID, 0, 32);

#if __DBG_DO_RAM_CACHE
    create_settings_item_ram_cache();
#endif // __DBG_DO_RAM_CACHE

    // List Item
    for (uint32_t i = 0; i < ARRAY_SIZE(SETTINGS_ITEM); i++)
    {
        lv_obj_t *p_item = lv_obj_create(p_list);
        lv_obj_clear_flag(p_item, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(p_item, LV_DIR_NONE);
        lv_obj_set_size(p_item, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(p_item, lv_color_white(), LV_STATE_DEFAULT);
        lv_obj_set_style_pad_top(p_item, 20, 0);
        lv_obj_add_event_cb(p_item, lv_settings_item_clicked_callback, LV_EVENT_CLICKED, (void *)&SETTINGS_ITEM[i]);

#if __DBG_DO_RAM_CACHE
        lv_obj_t *p_icon = lv_img_create(p_item);
        lv_img_set_src(p_icon, s_resource_cache_pool[i]);
        lv_obj_t *p_caption = lv_img_create(p_item);
        lv_img_set_src(p_caption, s_resource_cache_pool[ARRAY_SIZE(SETTINGS_ITEM) + i]);
#else
        lv_obj_t *p_icon = lv_img_create(p_item);
        lv_img_set_src(p_icon, SETTINGS_ITEM[i].icon_dsc);
        lv_obj_t *p_caption = lv_label_create(p_item);
        lv_label_set_text(p_caption, ISTR(SETTINGS_ITEM[i].caption));
#endif // __DBG_DO_RAM_CACHE

        lv_obj_align_to(p_caption, p_icon, LV_ALIGN_OUT_RIGHT_MID, 12, 0);
    }

    lv_obj_align(p_list, LV_ALIGN_TOP_MID, -12, LIST_MENU_TITLE_HEIGHT + 32);
    lv_obj_set_style_pad_bottom(p_list, lv_obj_get_height(lv_obj_get_child(p_list, -1)), LV_STATE_DEFAULT);

    return p_window;
}

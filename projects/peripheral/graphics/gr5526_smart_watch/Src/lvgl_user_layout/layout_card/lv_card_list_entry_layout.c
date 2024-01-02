#include "lv_layout_router.h"
#include "lv_user_font.h"
#include "lv_img_dsc_list.h"
#include "app_log.h"

#define ICON_COUNT 7 // fixed number.
#define GAP_HORIZONTAL 40
#define GAP_VERTICALLY 40

#define DISP_CENTER_X  ((int)DISP_HOR_RES / 2)
#define DISP_CENTER_Y  ((int)DISP_VER_RES / 2)

typedef struct _lv_card_list_entry_layout_item
{
    lv_obj_t *           p_obj;
    const lv_img_dsc_t * p_icon;
    uint32_t             win_id;
} icon_item_t;

// <NOTICE> DEFINE ICON ITEMS
static icon_item_t g_icon_items[ICON_COUNT] = {
    {NULL, &wd_img_APPLIST_03_HEARTRATE,        WMS_WID_CARD_HR},
    {NULL, &wd_img_APPLIST_04_SPO2,             WMS_WID_NONE},
    {NULL, &wd_img_APPLIST_07_BREATH_TRAINING,  WMS_WID_NONE},
    {NULL, &wd_img_APPLIST_00_MENU,             WMS_WID_APP_LIST},
    {NULL, &wd_img_APPLIST_10_NFC,              WMS_WID_NONE},
    {NULL, &wd_img_APPLIST_14_FIND_PHONE,       WMS_WID_NONE},
    {NULL, &wd_img_APPLIST_09_MUSIC,            WMS_WID_NONE},
};

static void icon_click_event_cb(lv_event_t *e)
{
    icon_item_t *icon_item = (icon_item_t *)lv_event_get_user_data(e);
    lv_layout_router_goto_isolate_win(icon_item->p_obj, icon_item->win_id, WMS_DEFAULT_GOTO_EFFECT_POS, WMS_DEFAULT_GOTO_EFFECT);
}

static void create_icon_lv_obj(lv_obj_t *win, int idx)
{
    const lv_img_dsc_t *p_icon = g_icon_items[idx].p_icon;

    lv_obj_t *p_btn_obj = lv_btn_create(win);
    lv_obj_set_size(p_btn_obj, p_icon->header.w, p_icon->header.h);
    lv_obj_set_style_bg_img_src(p_btn_obj, p_icon, 0);
    lv_obj_add_event_cb(p_btn_obj, icon_click_event_cb, LV_EVENT_CLICKED, &g_icon_items[idx]);

    g_icon_items[idx].p_obj = p_btn_obj;
}

void lv_card_list_entry_layout_init(lv_obj_t *win)
{
    int idx = 3;
    if (g_icon_items[idx].p_icon != NULL)
    {
        create_icon_lv_obj(win, idx);
        lv_obj_set_pos(g_icon_items[idx].p_obj, DISP_CENTER_X - 36, DISP_CENTER_Y - 36);

        // top line
        idx = 0;
        if (g_icon_items[idx].p_icon != NULL)
        {
            create_icon_lv_obj(win, idx);
            lv_obj_set_pos(g_icon_items[idx].p_obj, DISP_CENTER_X - GAP_HORIZONTAL / 2 - 72, DISP_CENTER_Y - 36 - GAP_VERTICALLY - 72);
        }

        idx = 1;
        if (g_icon_items[idx].p_icon != NULL)
        {
            create_icon_lv_obj(win, idx);
            lv_obj_set_pos(g_icon_items[idx].p_obj, DISP_CENTER_X + GAP_HORIZONTAL / 2, DISP_CENTER_Y - 36 - GAP_VERTICALLY - 72);
        }

        // mid line
        idx = 2;
        if (g_icon_items[idx].p_icon != NULL)
        {
            create_icon_lv_obj(win, idx);
            lv_obj_set_pos(g_icon_items[idx].p_obj, DISP_CENTER_X - 36 - GAP_HORIZONTAL - 72, DISP_CENTER_Y - 36);
        }

        idx = 4;
        if (g_icon_items[idx].p_icon != NULL)
        {
            create_icon_lv_obj(win, idx);
            lv_obj_set_pos(g_icon_items[idx].p_obj, DISP_CENTER_X + 36 + GAP_HORIZONTAL, DISP_CENTER_Y - 36);
        }

        // bottom line
        idx = 5;
        if (g_icon_items[idx].p_icon != NULL)
        {
            create_icon_lv_obj(win, idx);
            lv_obj_set_pos(g_icon_items[idx].p_obj, DISP_CENTER_X - GAP_HORIZONTAL / 2 - 72, DISP_CENTER_Y + 36 + GAP_VERTICALLY);
        }

        idx = 6;
        if (g_icon_items[idx].p_icon != NULL)
        {
            create_icon_lv_obj(win, idx);
            lv_obj_set_pos(g_icon_items[idx].p_obj, DISP_CENTER_X + GAP_HORIZONTAL / 2, DISP_CENTER_Y + 36 + GAP_VERTICALLY);
        }
    }
}

lv_obj_t *lv_card_list_entry_layout_create(lv_obj_t *parent_tv_obj)
{
    lv_obj_t *window = lv_obj_create(parent_tv_obj);

    lv_card_list_entry_layout_init(window);

    return window;
}

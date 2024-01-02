#include "lv_layout_router.h"
#include "lv_user_font.h"
#include "app_rtc.h"
#include "lv_img_dsc_list.h"
#include "app_log.h"


#define PANEL_WIDTH (400)
#define PANEL_HEIGHT (240)

#define LINK_ICON_X (190)
#define LINK_ICON_Y (10)
#define TIME_LABEL_X (70)
#define TIME_LABEL_Y (80)
#define DATE_LABEL_X (214)
#define DATE_LABEL_Y (74)
#define WEEK_LABEL_X (214)
#define WEEK_LABEL_Y (98)
#define BAT_ICON_X (306)
#define BAT_ICON_Y (72)
#define BAT_VAL_X (284)
#define BAT_VAL_Y (98)
#define PANEL_X (0)
#define PANEL_Y (46)
#define TABLE_1_X (1)
#define TABLE_1_Y (0)
#define TABLE_2_X (398)
#define TABLE_2_Y (0)

#define ROW1_COL1_X (20)
#define ROW1_COL1_Y (0)
#define ROW1_COL2_X (152)
#define ROW1_COL2_Y (0)
#define ROW1_COL3_X (284)
#define ROW1_COL3_Y (0)
#define ROW2_COL1_X (20)
#define ROW2_COL1_Y (112)
#define ROW2_COL2_X (152)
#define ROW2_COL2_Y (112)
#define ROW2_COL3_X (284)
#define ROW2_COL3_Y (112)


/*
 * SCREEN VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static lv_obj_t *s_link_icon = NULL;
static lv_obj_t *s_time = NULL;
static lv_obj_t *s_date = NULL;
static lv_obj_t *s_week = NULL;
static lv_obj_t *s_bat_icon = NULL;
static lv_obj_t *s_bat_val = NULL;

static lv_obj_t *s_wrist_raise = NULL;
static lv_obj_t *s_bluetooth = NULL;
static lv_obj_t *s_brightness = NULL;
static lv_obj_t *s_anti_disturb = NULL;
static lv_obj_t *s_phone = NULL;
static lv_obj_t *s_settings = NULL;
static lv_obj_t *s_alarm = NULL;
static lv_obj_t *s_info = NULL;
static lv_obj_t *s_find_phone = NULL;
static lv_obj_t *s_flashlight = NULL;
static lv_obj_t *s_qr_code = NULL;

static lv_timer_t *s_refr_timer = NULL;

static const lv_style_const_prop_t PANEL_STYLE_PROPS[] = {
    LV_STYLE_CONST_BG_OPA(LV_OPA_TRANSP),
    LV_STYLE_CONST_ALIGN(LV_ALIGN_CENTER),
    LV_STYLE_CONST_WIDTH(PANEL_WIDTH),
    LV_STYLE_CONST_HEIGHT(PANEL_HEIGHT),
    LV_STYLE_CONST_PAD_LEFT(0),
    LV_STYLE_CONST_PAD_RIGHT(0),
    LV_STYLE_CONST_PAD_TOP(0),
    LV_STYLE_CONST_PAD_BOTTOM(0),
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(PANEL_STYLE, PANEL_STYLE_PROPS);

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void btn_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    lv_state_t state = lv_obj_get_state(obj);

    if (code == LV_EVENT_CLICKED)
    {
        if (state == LV_STATE_CHECKED)
        {
            if (s_wrist_raise == obj)
            {
                lv_img_set_src(obj, &wd_img_quick_tab_taiwan_on);
            }
            else if (s_anti_disturb == obj)
            {
                lv_img_set_src(obj, &wd_img_quick_tab_dont_on);
            }
        }
        else
        {
            if (s_wrist_raise == obj)
            {
                lv_img_set_src(obj, &wd_img_quick_tab_taiwan_off);
            }
            else if (s_anti_disturb == obj)
            {
                lv_img_set_src(obj, &wd_img_quick_tab_dont_off);
            }
        }
    }
}

static void status_bar_refr_status(lv_timer_t *p_timer)
{
    app_rtc_time_t time;
    app_rtc_get_time(&time);
    lv_label_set_text_fmt(s_time, "%02d:%02d", time.hour, time.min);

    {
        // battery_status_t bat_sts = pwr_manager_get_battery_status(-1);
        uint8_t bat_val = 80;
        const lv_img_dsc_t* bat_img;
        if (bat_val >= 95)
        {
            bat_img = &wd_img_elc_100;
        }
        else if (bat_val >= 75)
        {
            bat_img = &wd_img_elc_80;
        }
        else if (bat_val >= 55)
        {
            bat_img = &wd_img_elc_60;
        }
        else if (bat_val >= 35)
        {
            bat_img = &wd_img_elc_40;
        }
        else if (bat_val >= 15)
        {
            bat_img = &wd_img_elc_20;
        }
        else
        {
            bat_img = &wd_img_elc_5;
        }
        lv_img_set_src(s_bat_icon, bat_img);
    }
}

#if 0
// static void status_bar_charging_state_changed_cb(pwr_state_t state)
// {
//     status_bar_refr_status(NULL);
// }
#endif

static void _win_evt_cb(lv_event_t *e)
{
    lv_event_code_t evt_code = lv_event_get_code(e);
    lv_obj_t *win = lv_event_get_target(e);

    if (evt_code == LV_EVENT_READY)
    {
        if (s_refr_timer)
        {
        }
        else
        {
            s_refr_timer = lv_timer_create(status_bar_refr_status, 15 * 1000, NULL);
        }
    }
    else if (evt_code == LV_EVENT_CANCEL)
    {
        if (s_refr_timer != NULL)
            lv_timer_pause(s_refr_timer);
    }
    else if (evt_code == LV_EVENT_DELETE)
    {
        if (s_refr_timer != NULL)
        {
            lv_timer_del(s_refr_timer);
            s_refr_timer = NULL;
        }
    }
}

static lv_obj_t *status_btn_create(lv_obj_t *obj, const void *src)
{
    lv_obj_t *status_btn = lv_img_create(obj);
    lv_img_set_src(status_btn, src);
    lv_obj_add_flag(status_btn, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(status_btn, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_add_event_cb(status_btn, btn_handler, LV_EVENT_CLICKED, NULL);
    return status_btn;
}
static lv_obj_t *panel = NULL;
static lv_obj_t *tab_1;
static lv_obj_t *tab_2;


static void status_panel_init(lv_obj_t *obj)
{
    tab_1 = lv_obj_create(panel);
    lv_obj_set_style_pad_all(tab_1, 0, 0);
    lv_obj_set_size(tab_1, PANEL_WIDTH, PANEL_HEIGHT);
    lv_obj_set_scrollbar_mode(tab_1, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(tab_1, LV_DIR_HOR);
    lv_obj_set_pos(tab_1, TABLE_1_X, TABLE_1_Y);

    // row 1
    s_wrist_raise = status_btn_create(tab_1, &wd_img_quick_tab_taiwan_on);
    lv_obj_set_pos(s_wrist_raise, ROW1_COL1_X, ROW1_COL1_Y);

    s_bluetooth = lv_img_create(tab_1);
    lv_img_set_src(s_bluetooth, &wd_img_quick_tab_bt_linking);
    lv_obj_set_pos(s_bluetooth, ROW1_COL2_X, ROW1_COL2_Y);

    s_brightness = lv_img_create(tab_1);
    lv_img_set_src(s_brightness, &wd_img_quick_tab_brightness);
    lv_obj_set_pos(s_brightness, ROW1_COL3_X, ROW1_COL3_Y);

    s_anti_disturb = status_btn_create(tab_1, &wd_img_quick_tab_dont_off);
    lv_obj_set_pos(s_anti_disturb, ROW2_COL1_X, ROW2_COL1_Y);

    s_phone = lv_img_create(tab_1);
    lv_img_set_src(s_phone, &wd_img_quick_tab_phone);
    lv_obj_set_pos(s_phone, ROW2_COL2_X, ROW2_COL2_Y);

    s_settings = lv_img_create(tab_1);
    lv_img_set_src(s_settings, &wd_img_quick_tab_setting);
    lv_obj_set_pos(s_settings, ROW2_COL3_X, ROW2_COL3_Y);

    // Table 2
    tab_2 = lv_obj_create(panel);
    lv_obj_set_size(tab_2, PANEL_WIDTH, PANEL_HEIGHT);
    lv_obj_set_style_pad_all(tab_2, 0, 0);
    lv_obj_set_scrollbar_mode(tab_2, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(tab_2, LV_DIR_HOR);
    lv_obj_set_pos(tab_2, TABLE_2_X, TABLE_2_Y);

    s_alarm = lv_img_create(tab_2);
    lv_img_set_src(s_alarm, &wd_img_quick_tab_alarm);
    lv_obj_set_pos(s_alarm, ROW1_COL1_X, ROW1_COL1_Y);

    s_info = lv_img_create(tab_2);
    lv_img_set_src(s_info, &wd_img_quick_tab_about);
    lv_obj_set_pos(s_info, ROW1_COL2_X, ROW1_COL2_Y);

    s_find_phone = lv_img_create(tab_2);
    lv_img_set_src(s_find_phone, &wd_img_quick_tab_find);
    lv_obj_set_pos(s_find_phone, ROW1_COL3_X, ROW1_COL3_Y);

    s_flashlight = lv_img_create(tab_2);
    lv_img_set_src(s_flashlight, &wd_img_quick_tab_flashlight);
    lv_obj_set_pos(s_flashlight, ROW2_COL1_X, ROW2_COL1_Y);

    s_qr_code = lv_img_create(tab_2);
    lv_img_set_src(s_qr_code, &wd_img_quick_tab_qr_code);
    lv_obj_set_pos(s_qr_code, ROW2_COL2_X, ROW2_COL2_Y);

    // lv_obj_enable_style_refresh(true);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
lv_obj_t *lv_card_status_layout_create(lv_obj_t *parent)
{
    lv_obj_t *p_window = lv_obj_create(parent);
    lv_obj_set_size(p_window, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_clear_flag(p_window, LV_OBJ_FLAG_SCROLLABLE);

    s_link_icon = lv_img_create(p_window);
    lv_img_set_src(s_link_icon, &wd_img_quick_link_on);
    lv_obj_set_pos(s_link_icon, LINK_ICON_X, LINK_ICON_Y);

    // time
    s_time = lv_label_create(p_window);
    lv_obj_set_style_text_color(s_time, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_time, &lv_font_montserrat_48_gdx, 0);
    lv_label_set_text_fmt(s_time, "%02d:%02d", 00, 00);
    lv_obj_set_pos(s_time, TIME_LABEL_X, TIME_LABEL_Y);

    // date
    s_date = lv_label_create(p_window);
    lv_obj_set_style_text_color(s_date, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_date, &lv_font_montserrat_20, 0);
    lv_label_set_text_fmt(s_date, "%d/%d", 6, 2);
    lv_obj_set_pos(s_date, DATE_LABEL_X, DATE_LABEL_Y);

    // week
    s_week = lv_label_create(p_window);
    lv_obj_set_style_text_color(s_week, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_week, &lv_font_montserrat_30, 0);
    lv_label_set_text_fmt(s_week, "%s", "Fri");
    lv_obj_set_pos(s_week, WEEK_LABEL_X, WEEK_LABEL_Y);

    // battery icon
    s_bat_icon = lv_img_create(p_window);
    lv_img_set_src(s_bat_icon, &wd_img_elc_100);
    lv_obj_set_pos(s_bat_icon, BAT_ICON_X, BAT_ICON_Y);

    // battery val
    s_bat_val = lv_label_create(p_window);
    lv_obj_set_style_text_color(s_bat_val, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_bat_val, &lv_font_montserrat_30, 0);
    lv_label_set_text_fmt(s_bat_val, "%d%%", 100);
    lv_obj_set_pos(s_bat_val, BAT_VAL_X, BAT_VAL_Y);

    // Panel
    panel = lv_obj_create(p_window);
    lv_obj_add_style(panel, (lv_style_t *)&PANEL_STYLE, LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(panel, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(panel, LV_DIR_HOR);
    lv_obj_set_scroll_snap_x(panel, LV_SCROLL_SNAP_START);
    lv_obj_set_pos(panel, PANEL_X, PANEL_Y);

    // initialize the status panel
    status_panel_init(panel);

    // pwr_manager_regsiter_power_state_changed_callback(status_bar_charging_state_changed_cb);
    lv_obj_add_event_cb(p_window, _win_evt_cb, LV_EVENT_ALL, NULL);

    return p_window;
}

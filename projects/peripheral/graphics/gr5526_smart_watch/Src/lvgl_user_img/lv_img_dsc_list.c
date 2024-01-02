#include "lvgl.h"
#include "gr55xx_hal.h"
#include "lv_img_dsc_list.h"
const lv_img_dsc_t  wd_img_black_clock_face = {
    .header.always_zero = 0,
    .header.w = 454,
    .header.h = 454,
    .data_size = 412232, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BLACK_CLOCK_FACE),
    };
const lv_img_dsc_t  wd_img_black_clock_thumbnail = {
    .header.always_zero = 0,
    .header.w = 214,
    .header.h = 214,
    .data_size = 91592, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BLACK_CLOCK_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_day_calorie = {
    .header.always_zero = 0,
    .header.w = 40,
    .header.h = 40,
    .data_size = 3200, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DAY_CALORIE),
    };
const lv_img_dsc_t  wd_img_day_distance = {
    .header.always_zero = 0,
    .header.w = 40,
    .header.h = 40,
    .data_size = 3200, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DAY_DISTANCE),
    };
const lv_img_dsc_t  wd_img_day_running = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 36,
    .data_size = 2592, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DAY_RUNNING),
    };
const lv_img_dsc_t  wd_img_day_sports = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 36,
    .data_size = 2592, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DAY_SPORTS),
    };
const lv_img_dsc_t  wd_img_day_walking = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 36,
    .data_size = 2592, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DAY_WALKING),
    };
const lv_img_dsc_t  wd_img_digital_clock_thumbnail = {
    .header.always_zero = 0,
    .header.w = 214,
    .header.h = 214,
    .data_size = 91592, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL_CLOCK_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_ECG_01_LOGO = {
    .header.always_zero = 0,
    .header.w = 54,
    .header.h = 49,
    .data_size = 5292, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ECG_01_LOGO),
    };
const lv_img_dsc_t  wd_img_elc_100 = {
    .header.always_zero = 0,
    .header.w = 52,
    .header.h = 24,
    .data_size = 2496, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ELC_100),
    };
const lv_img_dsc_t  wd_img_elc_20 = {
    .header.always_zero = 0,
    .header.w = 52,
    .header.h = 24,
    .data_size = 2496, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ELC_20),
    };
const lv_img_dsc_t  wd_img_elc_40 = {
    .header.always_zero = 0,
    .header.w = 52,
    .header.h = 24,
    .data_size = 2496, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ELC_40),
    };
const lv_img_dsc_t  wd_img_elc_5 = {
    .header.always_zero = 0,
    .header.w = 52,
    .header.h = 24,
    .data_size = 2496, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ELC_5),
    };
const lv_img_dsc_t  wd_img_elc_60 = {
    .header.always_zero = 0,
    .header.w = 52,
    .header.h = 24,
    .data_size = 2496, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ELC_60),
    };
const lv_img_dsc_t  wd_img_elc_80 = {
    .header.always_zero = 0,
    .header.w = 52,
    .header.h = 24,
    .data_size = 2496, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_ELC_80),
    };
const lv_img_dsc_t  wd_img_heartrate_background = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 144,
    .data_size = 77760, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE_BACKGROUND),
    };
const lv_img_dsc_t  wd_img_heartrate_lower = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 16,
    .data_size = 640, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE_LOWER),
    };
const lv_img_dsc_t  wd_img_heartrate_sleep_48 = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 4608, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE_SLEEP_48),
    };
const lv_img_dsc_t  wd_img_heartrate_sleep_56 = {
    .header.always_zero = 0,
    .header.w = 56,
    .header.h = 56,
    .data_size = 6272, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE_SLEEP_56),
    };
const lv_img_dsc_t  wd_img_heartrate_sleep_64 = {
    .header.always_zero = 0,
    .header.w = 64,
    .header.h = 64,
    .data_size = 8192, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE_SLEEP_64),
    };
const lv_img_dsc_t  wd_img_heartrate_upper = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 16,
    .data_size = 640, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE_UPPER),
    };
const lv_img_dsc_t  wd_img_heartrate_walk_48 = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 4608, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE_WALK_48),
    };
const lv_img_dsc_t  wd_img_heartrate_walk_56 = {
    .header.always_zero = 0,
    .header.w = 56,
    .header.h = 56,
    .data_size = 6272, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE_WALK_56),
    };
const lv_img_dsc_t  wd_img_heartrate_walk_64 = {
    .header.always_zero = 0,
    .header.w = 64,
    .header.h = 64,
    .data_size = 8192, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEARTRATE_WALK_64),
    };
const lv_img_dsc_t  wd_img_heart_icon_50 = {
    .header.always_zero = 0,
    .header.w = 50,
    .header.h = 50,
    .data_size = 5000, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEART_ICON_50),
    };
const lv_img_dsc_t  wd_img_heart_icon_70 = {
    .header.always_zero = 0,
    .header.w = 70,
    .header.h = 70,
    .data_size = 9800, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HEART_ICON_70),
    };
const lv_img_dsc_t  wd_img_hr_icon = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON),
    };
const lv_img_dsc_t  wd_img_hr_icon_00000 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00000),
    };
const lv_img_dsc_t  wd_img_hr_icon_00001 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00001),
    };
const lv_img_dsc_t  wd_img_hr_icon_00002 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00002),
    };
const lv_img_dsc_t  wd_img_hr_icon_00003 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00003),
    };
const lv_img_dsc_t  wd_img_hr_icon_00004 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00004),
    };
const lv_img_dsc_t  wd_img_hr_icon_00005 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00005),
    };
const lv_img_dsc_t  wd_img_hr_icon_00006 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00006),
    };
const lv_img_dsc_t  wd_img_hr_icon_00007 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00007),
    };
const lv_img_dsc_t  wd_img_hr_icon_00008 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00008),
    };
const lv_img_dsc_t  wd_img_hr_icon_00009 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00009),
    };
const lv_img_dsc_t  wd_img_hr_icon_00010 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00010),
    };
const lv_img_dsc_t  wd_img_hr_icon_00011 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00011),
    };
const lv_img_dsc_t  wd_img_hr_icon_00012 = {
    .header.always_zero = 0,
    .header.w = 68,
    .header.h = 60,
    .data_size = 8160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_ICON_00012),
    };
const lv_img_dsc_t  wd_img_hr_max = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 16,
    .data_size = 640, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_MAX),
    };
const lv_img_dsc_t  wd_img_hr_min = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 16,
    .data_size = 640, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_HR_MIN),
    };
const lv_img_dsc_t  wd_img_live_wallpaer_flower = {
    .header.always_zero = 0,
    .header.w = 360,
    .header.h = 360,
    .data_size = 259200, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_WALLPAER_FLOWER),
    };
const lv_img_dsc_t  wd_img_msg_placeholder_24 = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 1152, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MSG_PLACEHOLDER_24),
    };
const lv_img_dsc_t  wd_img_msg_placeholder_64 = {
    .header.always_zero = 0,
    .header.w = 64,
    .header.h = 64,
    .data_size = 8192, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MSG_PLACEHOLDER_64),
    };
const lv_img_dsc_t  wd_img_MUSIC_01_PLAY = {
    .header.always_zero = 0,
    .header.w = 60,
    .header.h = 60,
    .data_size = 7200, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MUSIC_01_PLAY),
    };
const lv_img_dsc_t  wd_img_MUSIC_02_PAUSE = {
    .header.always_zero = 0,
    .header.w = 60,
    .header.h = 60,
    .data_size = 7200, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MUSIC_02_PAUSE),
    };
const lv_img_dsc_t  wd_img_MUSIC_03_BACKWORD = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 1152, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MUSIC_03_BACKWORD),
    };
const lv_img_dsc_t  wd_img_MUSIC_04_FORWARD = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 1152, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MUSIC_04_FORWARD),
    };
const lv_img_dsc_t  wd_img_MUSIC_05_VOLUME = {
    .header.always_zero = 0,
    .header.w = 24,
    .header.h = 24,
    .data_size = 1152, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MUSIC_05_VOLUME),
    };
const lv_img_dsc_t  wd_img_MUSIC_06_RESERVED = {
    .header.always_zero = 0,
    .header.w = 152,
    .header.h = 152,
    .data_size = 46208, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MUSIC_06_RESERVED),
    };
const lv_img_dsc_t  wd_img_nfc_access_card = {
    .header.always_zero = 0,
    .header.w = 256,
    .header.h = 160,
    .data_size = 81920, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_ACCESS_CARD),
    };
const lv_img_dsc_t  wd_img_nfc_bank_card = {
    .header.always_zero = 0,
    .header.w = 256,
    .header.h = 160,
    .data_size = 81920, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_BANK_CARD),
    };
const lv_img_dsc_t  wd_img_nfc_card_car_key = {
    .header.always_zero = 0,
    .header.w = 256,
    .header.h = 160,
    .data_size = 81920, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_CARD_CAR_KEY),
    };
const lv_img_dsc_t  wd_img_nfc_card_download = {
    .header.always_zero = 0,
    .header.w = 54,
    .header.h = 32,
    .data_size = 3456, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_CARD_DOWNLOAD),
    };
const lv_img_dsc_t  wd_img_nfc_card_package = {
    .header.always_zero = 0,
    .header.w = 240,
    .header.h = 172,
    .data_size = 82560, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_CARD_PACKAGE),
    };
const lv_img_dsc_t  wd_img_nfc_cellphone_download = {
    .header.always_zero = 0,
    .header.w = 82,
    .header.h = 146,
    .data_size = 23944, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_CELLPHONE_DOWNLOAD),
    };
const lv_img_dsc_t  wd_img_nfc_default_card_flag = {
    .header.always_zero = 0,
    .header.w = 82,
    .header.h = 30,
    .data_size = 4920, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_DEFAULT_CARD_FLAG),
    };
const lv_img_dsc_t  wd_img_nfc_icon_jpg = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_ICON_JPG),
    };
const lv_img_dsc_t  wd_img_nfc_icon_png = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_ICON_PNG),
    };
const lv_img_dsc_t  wd_img_nfc_radio_btn_off = {
    .header.always_zero = 0,
    .header.w = 76,
    .header.h = 30,
    .data_size = 4560, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_RADIO_BTN_OFF),
    };
const lv_img_dsc_t  wd_img_nfc_radio_btn_on = {
    .header.always_zero = 0,
    .header.w = 76,
    .header.h = 30,
    .data_size = 4560, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_RADIO_BTN_ON),
    };
const lv_img_dsc_t  wd_img_nfc_transport_card = {
    .header.always_zero = 0,
    .header.w = 256,
    .header.h = 160,
    .data_size = 81920, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_TRANSPORT_CARD),
    };
const lv_img_dsc_t  wd_img_nfc_watch_copy_card = {
    .header.always_zero = 0,
    .header.w = 128,
    .header.h = 110,
    .data_size = 28160, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_WATCH_COPY_CARD),
    };
const lv_img_dsc_t  wd_img_nfc_watch_download_card = {
    .header.always_zero = 0,
    .header.w = 64,
    .header.h = 86,
    .data_size = 11008, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_WATCH_DOWNLOAD_CARD),
    };
const lv_img_dsc_t  wd_img_pressure_background = {
    .header.always_zero = 0,
    .header.w = 270,
    .header.h = 144,
    .data_size = 77760, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_PRESSURE_BACKGROUND),
    };
const lv_img_dsc_t  wd_img_pressure_icon = {
    .header.always_zero = 0,
    .header.w = 60,
    .header.h = 60,
    .data_size = 7200, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_PRESSURE_ICON),
    };
const lv_img_dsc_t  wd_img_pressure_icon_50 = {
    .header.always_zero = 0,
    .header.w = 50,
    .header.h = 50,
    .data_size = 5000, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_PRESSURE_ICON_50),
    };
const lv_img_dsc_t  wd_img_pressure_icon_70 = {
    .header.always_zero = 0,
    .header.w = 70,
    .header.h = 70,
    .data_size = 9800, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_PRESSURE_ICON_70),
    };
const lv_img_dsc_t  wd_img_pressure_lower = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 16,
    .data_size = 640, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_PRESSURE_LOWER),
    };
const lv_img_dsc_t  wd_img_pressure_upper = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 16,
    .data_size = 640, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_PRESSURE_UPPER),
    };
const lv_img_dsc_t  wd_img_quick_handles = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 8,
    .data_size = 512, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_HANDLES),
    };
const lv_img_dsc_t  wd_img_quick_link_off = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 48,
    .data_size = 3456, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_LINK_OFF),
    };
const lv_img_dsc_t  wd_img_quick_link_on = {
    .header.always_zero = 0,
    .header.w = 36,
    .header.h = 48,
    .data_size = 3456, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_LINK_ON),
    };
const lv_img_dsc_t  wd_img_quick_tab_about = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_ABOUT),
    };
const lv_img_dsc_t  wd_img_quick_tab_alarm = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_ALARM),
    };
const lv_img_dsc_t  wd_img_quick_tab_brightness = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_BRIGHTNESS),
    };
const lv_img_dsc_t  wd_img_quick_tab_bt_linking = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_BT_LINKING),
    };
const lv_img_dsc_t  wd_img_quick_tab_bt_off = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_BT_OFF),
    };
const lv_img_dsc_t  wd_img_quick_tab_bt_on = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_BT_ON),
    };
const lv_img_dsc_t  wd_img_quick_tab_dont_off = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_DONT_OFF),
    };
const lv_img_dsc_t  wd_img_quick_tab_dont_on = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_DONT_ON),
    };
const lv_img_dsc_t  wd_img_quick_tab_find = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_FIND),
    };
const lv_img_dsc_t  wd_img_quick_tab_findphone = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_FINDPHONE),
    };
const lv_img_dsc_t  wd_img_quick_tab_flashlight = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_FLASHLIGHT),
    };
const lv_img_dsc_t  wd_img_quick_tab_grid = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_GRID),
    };
const lv_img_dsc_t  wd_img_quick_tab_list = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_LIST),
    };
const lv_img_dsc_t  wd_img_quick_tab_phone = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_PHONE),
    };
const lv_img_dsc_t  wd_img_quick_tab_qr_code = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_QR_CODE),
    };
const lv_img_dsc_t  wd_img_quick_tab_roulette = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_ROULETTE),
    };
const lv_img_dsc_t  wd_img_quick_tab_setting = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_SETTING),
    };
const lv_img_dsc_t  wd_img_quick_tab_taiwan_off = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_TAIWAN_OFF),
    };
const lv_img_dsc_t  wd_img_quick_tab_taiwan_on = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_TAIWAN_ON),
    };
const lv_img_dsc_t  wd_img_quick_tab_volume = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_VOLUME),
    };
const lv_img_dsc_t  wd_img_quick_tab_water_off = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_WATER_OFF),
    };
const lv_img_dsc_t  wd_img_quick_tab_water_on = {
    .header.always_zero = 0,
    .header.w = 92,
    .header.h = 92,
    .data_size = 16928, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_QUICK_TAB_WATER_ON),
    };
const lv_img_dsc_t  wd_img_sleep_background = {
    .header.always_zero = 0,
    .header.w = 454,
    .header.h = 454,
    .data_size = 412232, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SLEEP_BACKGROUND),
    };
const lv_img_dsc_t  wd_img_sleep_icon_big = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 60,
    .data_size = 8640, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SLEEP_ICON_BIG),
    };
const lv_img_dsc_t  wd_img_sleep_icon_small = {
    .header.always_zero = 0,
    .header.w = 60,
    .header.h = 52,
    .data_size = 6240, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SLEEP_ICON_SMALL),
    };
const lv_img_dsc_t  wd_img_sop2_background = {
    .header.always_zero = 0,
    .header.w = 454,
    .header.h = 454,
    .data_size = 412232, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SOP2_BACKGROUND),
    };
const lv_img_dsc_t  wd_img_sop2_icon = {
    .header.always_zero = 0,
    .header.w = 50,
    .header.h = 63,
    .data_size = 6300, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SOP2_ICON),
    };
const lv_img_dsc_t  wd_img_sop2_measure_gesture = {
    .header.always_zero = 0,
    .header.w = 200,
    .header.h = 84,
    .data_size = 33600, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SOP2_MEASURE_GESTURE),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_01_LAP = {
    .header.always_zero = 0,
    .header.w = 60,
    .header.h = 60,
    .data_size = 7200, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_01_LAP),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_02_PAUSE = {
    .header.always_zero = 0,
    .header.w = 60,
    .header.h = 60,
    .data_size = 7200, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_02_PAUSE),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_03_RESET = {
    .header.always_zero = 0,
    .header.w = 60,
    .header.h = 60,
    .data_size = 7200, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_03_RESET),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_04_START = {
    .header.always_zero = 0,
    .header.w = 60,
    .header.h = 60,
    .data_size = 7200, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_04_START),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_05_LOGO = {
    .header.always_zero = 0,
    .header.w = 50,
    .header.h = 50,
    .data_size = 5000, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_05_LOGO),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_06_LAPFLAG = {
    .header.always_zero = 0,
    .header.w = 14,
    .header.h = 20,
    .data_size = 560, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_06_LAPFLAG),
    };
const lv_img_dsc_t  wd_img_stress_easy_50 = {
    .header.always_zero = 0,
    .header.w = 50,
    .header.h = 50,
    .data_size = 5000, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STRESS_EASY_50),
    };
const lv_img_dsc_t  wd_img_stress_easy_70 = {
    .header.always_zero = 0,
    .header.w = 70,
    .header.h = 70,
    .data_size = 9800, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STRESS_EASY_70),
    };
const lv_img_dsc_t  wd_img_stress_high_50 = {
    .header.always_zero = 0,
    .header.w = 50,
    .header.h = 44,
    .data_size = 4400, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STRESS_HIGH_50),
    };
const lv_img_dsc_t  wd_img_stress_high_70 = {
    .header.always_zero = 0,
    .header.w = 70,
    .header.h = 64,
    .data_size = 8960, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STRESS_HIGH_70),
    };
const lv_img_dsc_t  wd_img_stress_middle_50 = {
    .header.always_zero = 0,
    .header.w = 50,
    .header.h = 50,
    .data_size = 5000, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STRESS_MIDDLE_50),
    };
const lv_img_dsc_t  wd_img_stress_middle_70 = {
    .header.always_zero = 0,
    .header.w = 70,
    .header.h = 70,
    .data_size = 9800, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STRESS_MIDDLE_70),
    };
const lv_img_dsc_t  wd_img_stress_normal_50 = {
    .header.always_zero = 0,
    .header.w = 50,
    .header.h = 50,
    .data_size = 5000, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STRESS_NORMAL_50),
    };
const lv_img_dsc_t  wd_img_stress_normal_70 = {
    .header.always_zero = 0,
    .header.w = 70,
    .header.h = 70,
    .data_size = 9800, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STRESS_NORMAL_70),
    };
const lv_img_dsc_t  wd_img_table_bg = {
    .header.always_zero = 0,
    .header.w = 380,
    .header.h = 164,
    .data_size = 124640, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_TABLE_BG),
    };
const lv_img_dsc_t  wd_img_table_bg_scaled = {
    .header.always_zero = 0,
    .header.w = 356,
    .header.h = 164,
    .data_size = 116768, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_TABLE_BG_SCALED),
    };
const lv_img_dsc_t  wd_img_vivid_clock_bg = {
    .header.always_zero = 0,
    .header.w = 454,
    .header.h = 454,
    .data_size = 412232, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_BG),
    };
const lv_img_dsc_t  wd_img_vivid_clock_thumbnail = {
    .header.always_zero = 0,
    .header.w = 214,
    .header.h = 214,
    .data_size = 91592, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_THUMBNAIL),
    };
const lv_img_dsc_t  wd_img_watchface1 = {
    .header.always_zero = 0,
    .header.w = 454,
    .header.h = 454,
    .data_size = 412232, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WATCHFACE1),
    };
const lv_img_dsc_t  wd_img_weather_clear = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_CLEAR),
    };
const lv_img_dsc_t  wd_img_weather_cloudy = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_CLOUDY),
    };
const lv_img_dsc_t  wd_img_weather_drizzle = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_DRIZZLE),
    };
const lv_img_dsc_t  wd_img_weather_dust = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_DUST),
    };
const lv_img_dsc_t  wd_img_weather_fog = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_FOG),
    };
const lv_img_dsc_t  wd_img_weather_gloomy = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_GLOOMY),
    };
const lv_img_dsc_t  wd_img_weather_haze = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_HAZE),
    };
const lv_img_dsc_t  wd_img_weather_heavy_rain = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_HEAVY_RAIN),
    };
const lv_img_dsc_t  wd_img_weather_heavy_snow = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_HEAVY_SNOW),
    };
const lv_img_dsc_t  wd_img_weather_light_snow = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_LIGHT_SNOW),
    };
const lv_img_dsc_t  wd_img_weather_moderate_rain = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_MODERATE_RAIN),
    };
const lv_img_dsc_t  wd_img_weather_moderate_snow = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_MODERATE_SNOW),
    };
const lv_img_dsc_t  wd_img_weather_nodata = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_NODATA),
    };
const lv_img_dsc_t  wd_img_weather_nodate_icon = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_NODATE_ICON),
    };
const lv_img_dsc_t  wd_img_weather_sandstorm = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_SANDSTORM),
    };
const lv_img_dsc_t  wd_img_weather_sleet = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_SLEET),
    };
const lv_img_dsc_t  wd_img_weather_thunderstorm = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_THUNDERSTORM),
    };
const lv_img_dsc_t  wd_img_weather_typhoon = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 10368, 
    .header.cf = LV_IMG_CF_GDX_RGB565,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_WEATHER_TYPHOON),
    };
const lv_img_dsc_t  wd_img_APPLIST_00_MENU = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_00_MENU),
    };
const lv_img_dsc_t  wd_img_APPLIST_01_SPORTS = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_01_SPORTS),
    };
const lv_img_dsc_t  wd_img_APPLIST_02_ACTIVITES = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_02_ACTIVITES),
    };
const lv_img_dsc_t  wd_img_APPLIST_03_HEARTRATE = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_03_HEARTRATE),
    };
const lv_img_dsc_t  wd_img_APPLIST_04_SPO2 = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_04_SPO2),
    };
const lv_img_dsc_t  wd_img_APPLIST_05_STRESS = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_05_STRESS),
    };
const lv_img_dsc_t  wd_img_APPLIST_06_SLEEP = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_06_SLEEP),
    };
const lv_img_dsc_t  wd_img_APPLIST_07_BREATH_TRAINING = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_07_BREATH_TRAINING),
    };
const lv_img_dsc_t  wd_img_APPLIST_08_WEATHER = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_08_WEATHER),
    };
const lv_img_dsc_t  wd_img_APPLIST_09_MUSIC = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_09_MUSIC),
    };
const lv_img_dsc_t  wd_img_APPLIST_10_NFC = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_10_NFC),
    };
const lv_img_dsc_t  wd_img_APPLIST_11_ALARM = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_11_ALARM),
    };
const lv_img_dsc_t  wd_img_APPLIST_12_STOPWATCH = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_12_STOPWATCH),
    };
const lv_img_dsc_t  wd_img_APPLIST_13_TIMER = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_13_TIMER),
    };
const lv_img_dsc_t  wd_img_APPLIST_14_FIND_PHONE = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_14_FIND_PHONE),
    };
const lv_img_dsc_t  wd_img_APPLIST_15_SETTINGS = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_15_SETTINGS),
    };
const lv_img_dsc_t  wd_img_APPLIST_16_ECG = {
    .header.always_zero = 0,
    .header.w = 72,
    .header.h = 72,
    .data_size = 20736, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_APPLIST_16_ECG),
    };
const lv_img_dsc_t  wd_img_BATTERY_01_CHARGING = {
    .header.always_zero = 0,
    .header.w = 27,
    .header.h = 13,
    .data_size = 1404, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BATTERY_01_CHARGING),
    };
const lv_img_dsc_t  wd_img_BATTERY_02_FULL = {
    .header.always_zero = 0,
    .header.w = 27,
    .header.h = 13,
    .data_size = 1404, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BATTERY_02_FULL),
    };
const lv_img_dsc_t  wd_img_BATTERY_03_25 = {
    .header.always_zero = 0,
    .header.w = 28,
    .header.h = 14,
    .data_size = 1568, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BATTERY_03_25),
    };
const lv_img_dsc_t  wd_img_BATTERY_04_50 = {
    .header.always_zero = 0,
    .header.w = 28,
    .header.h = 14,
    .data_size = 1568, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BATTERY_04_50),
    };
const lv_img_dsc_t  wd_img_BATTERY_05_75 = {
    .header.always_zero = 0,
    .header.w = 28,
    .header.h = 14,
    .data_size = 1568, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BATTERY_05_75),
    };
const lv_img_dsc_t  wd_img_BATTERY_06_EMPTY = {
    .header.always_zero = 0,
    .header.w = 27,
    .header.h = 13,
    .data_size = 1404, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BATTERY_06_EMPTY),
    };
const lv_img_dsc_t  wd_img_BATTERY_07_LOGO = {
    .header.always_zero = 0,
    .header.w = 90,
    .header.h = 150,
    .data_size = 54000, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BATTERY_07_LOGO),
    };
const lv_img_dsc_t  wd_img_BATTERY_08_WVAE = {
    .header.always_zero = 0,
    .header.w = 180,
    .header.h = 21,
    .data_size = 15120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BATTERY_08_WVAE),
    };
const lv_img_dsc_t  wd_img_black_clock_center_point = {
    .header.always_zero = 0,
    .header.w = 18,
    .header.h = 18,
    .data_size = 1296, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BLACK_CLOCK_CENTER_POINT),
    };
const lv_img_dsc_t  wd_img_black_clock_needle_hour = {
    .header.always_zero = 0,
    .header.w = 96,
    .header.h = 12,
    .data_size = 4608, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BLACK_CLOCK_NEEDLE_HOUR),
    };
const lv_img_dsc_t  wd_img_black_clock_needle_minute = {
    .header.always_zero = 0,
    .header.w = 128,
    .header.h = 12,
    .data_size = 6144, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BLACK_CLOCK_NEEDLE_MINUTE),
    };
const lv_img_dsc_t  wd_img_black_clock_needle_second = {
    .header.always_zero = 0,
    .header.w = 161,
    .header.h = 5,
    .data_size = 3220, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BLACK_CLOCK_NEEDLE_SECOND),
    };
const lv_img_dsc_t  wd_img_black_clock_step = {
    .header.always_zero = 0,
    .header.w = 26,
    .header.h = 30,
    .data_size = 3120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BLACK_CLOCK_STEP),
    };
const lv_img_dsc_t  wd_img_black_clock_sun = {
    .header.always_zero = 0,
    .header.w = 23,
    .header.h = 23,
    .data_size = 2116, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_BLACK_CLOCK_SUN),
    };
const lv_img_dsc_t  wd_img_CHARGING_01_LOGO = {
    .header.always_zero = 0,
    .header.w = 128,
    .header.h = 128,
    .data_size = 65536, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_CHARGING_01_LOGO),
    };
const lv_img_dsc_t  wd_img_day_step = {
    .header.always_zero = 0,
    .header.w = 26,
    .header.h = 30,
    .data_size = 3120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DAY_STEP),
    };
const lv_img_dsc_t  wd_img_DIGITAL_WF_01_SPORTS = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 20,
    .data_size = 1600, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL_WF_01_SPORTS),
    };
const lv_img_dsc_t  wd_img_DIGITAL_WF_02_STEPS = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 20,
    .data_size = 1600, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL_WF_02_STEPS),
    };
const lv_img_dsc_t  wd_img_DIGITAL_WF_03_CALORIES = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 20,
    .data_size = 1600, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL_WF_03_CALORIES),
    };
const lv_img_dsc_t  wd_img_DIGITAL_WF_04_SHAVED = {
    .header.always_zero = 0,
    .header.w = 60,
    .header.h = 180,
    .data_size = 43200, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_DIGITAL_WF_04_SHAVED),
    };
const lv_img_dsc_t  wd_img_live_flower_center = {
    .header.always_zero = 0,
    .header.w = 17,
    .header.h = 17,
    .data_size = 1156, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_FLOWER_CENTER),
    };
const lv_img_dsc_t  wd_img_live_flower_hour = {
    .header.always_zero = 0,
    .header.w = 95,
    .header.h = 16,
    .data_size = 6080, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_FLOWER_HOUR),
    };
const lv_img_dsc_t  wd_img_live_flower_minute = {
    .header.always_zero = 0,
    .header.w = 136,
    .header.h = 13,
    .data_size = 7072, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_FLOWER_MINUTE),
    };
const lv_img_dsc_t  wd_img_live_flower_second = {
    .header.always_zero = 0,
    .header.w = 160,
    .header.h = 4,
    .data_size = 2560, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_LIVE_FLOWER_SECOND),
    };
const lv_img_dsc_t  wd_img_msg_default = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MSG_DEFAULT),
    };
const lv_img_dsc_t  wd_img_msg_outlook = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MSG_OUTLOOK),
    };
const lv_img_dsc_t  wd_img_msg_phone = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MSG_PHONE),
    };
const lv_img_dsc_t  wd_img_msg_qq = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MSG_QQ),
    };
const lv_img_dsc_t  wd_img_msg_sms = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MSG_SMS),
    };
const lv_img_dsc_t  wd_img_msg_twitter = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MSG_TWITTER),
    };
const lv_img_dsc_t  wd_img_msg_wechat = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_MSG_WECHAT),
    };
const lv_img_dsc_t  wd_img_nfc_add = {
    .header.always_zero = 0,
    .header.w = 34,
    .header.h = 34,
    .data_size = 4624, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_ADD),
    };
const lv_img_dsc_t  wd_img_nfc_charge = {
    .header.always_zero = 0,
    .header.w = 22,
    .header.h = 22,
    .data_size = 1936, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_NFC_CHARGE),
    };
const lv_img_dsc_t  wd_img_SETTINGS_01_DISPLAY = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_01_DISPLAY),
    };
const lv_img_dsc_t  wd_img_SETTINGS_02_APPLIST_STYLE = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_02_APPLIST_STYLE),
    };
const lv_img_dsc_t  wd_img_SETTINGS_03_SWITCH_EFFECT = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_03_SWITCH_EFFECT),
    };
const lv_img_dsc_t  wd_img_SETTINGS_04_LANGUAGE = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_04_LANGUAGE),
    };
const lv_img_dsc_t  wd_img_SETTINGS_05_SYSTEM = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_05_SYSTEM),
    };
const lv_img_dsc_t  wd_img_SETTINGS_06_ABOUT = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_06_ABOUT),
    };
const lv_img_dsc_t  wd_img_SETTINGS_BRIGHTNESS_01_ICON = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_BRIGHTNESS_01_ICON),
    };
const lv_img_dsc_t  wd_img_SETTINGS_DISPLAY_01_BRIGHTNESS = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_DISPLAY_01_BRIGHTNESS),
    };
const lv_img_dsc_t  wd_img_SETTINGS_DISPLAY_02_SCREEN_OFF_TIME = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_DISPLAY_02_SCREEN_OFF_TIME),
    };
const lv_img_dsc_t  wd_img_SETTINGS_DISPLAY_03_WAKE_METHOD = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_DISPLAY_03_WAKE_METHOD),
    };
const lv_img_dsc_t  wd_img_SETTINGS_DISPLAY_04_WATCHFACE = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_DISPLAY_04_WATCHFACE),
    };
const lv_img_dsc_t  wd_img_SETTINGS_OTA_01_LOGO = {
    .header.always_zero = 0,
    .header.w = 128,
    .header.h = 128,
    .data_size = 65536, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_OTA_01_LOGO),
    };
const lv_img_dsc_t  wd_img_SETTINGS_SYSTEM_01_FOTA = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_SYSTEM_01_FOTA),
    };
const lv_img_dsc_t  wd_img_SETTINGS_SYSTEM_02_DEBUG = {
    .header.always_zero = 0,
    .header.w = 48,
    .header.h = 48,
    .data_size = 9216, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SETTINGS_SYSTEM_02_DEBUG),
    };
const lv_img_dsc_t  wd_img_spo2_start_button = {
    .header.always_zero = 0,
    .header.w = 264,
    .header.h = 72,
    .data_size = 76032, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_SPO2_START_BUTTON),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_DIGITS_0 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 40,
    .data_size = 5120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_DIGITS_0),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_DIGITS_1 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 40,
    .data_size = 5120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_DIGITS_1),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_DIGITS_2 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 40,
    .data_size = 5120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_DIGITS_2),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_DIGITS_3 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 40,
    .data_size = 5120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_DIGITS_3),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_DIGITS_4 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 40,
    .data_size = 5120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_DIGITS_4),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_DIGITS_5 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 40,
    .data_size = 5120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_DIGITS_5),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_DIGITS_6 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 40,
    .data_size = 5120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_DIGITS_6),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_DIGITS_7 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 40,
    .data_size = 5120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_DIGITS_7),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_DIGITS_8 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 40,
    .data_size = 5120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_DIGITS_8),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_DIGITS_9 = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 40,
    .data_size = 5120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_DIGITS_9),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_DIGITS_COLON = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 40,
    .data_size = 5120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_DIGITS_COLON),
    };
const lv_img_dsc_t  wd_img_STOPWATCH_DIGITS_DECIMAL = {
    .header.always_zero = 0,
    .header.w = 32,
    .header.h = 40,
    .data_size = 5120, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_STOPWATCH_DIGITS_DECIMAL),
    };
const lv_img_dsc_t  wd_img_vivid_clock_alarm = {
    .header.always_zero = 0,
    .header.w = 31,
    .header.h = 31,
    .data_size = 3844, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_ALARM),
    };
const lv_img_dsc_t  wd_img_vivid_clock_battery = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 20,
    .data_size = 1600, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_BATTERY),
    };
const lv_img_dsc_t  wd_img_vivid_clock_center_point = {
    .header.always_zero = 0,
    .header.w = 18,
    .header.h = 18,
    .data_size = 1296, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_CENTER_POINT),
    };
const lv_img_dsc_t  wd_img_vivid_clock_counter = {
    .header.always_zero = 0,
    .header.w = 34,
    .header.h = 40,
    .data_size = 5440, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_COUNTER),
    };
const lv_img_dsc_t  wd_img_vivid_clock_energy = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 20,
    .data_size = 1600, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_ENERGY),
    };
const lv_img_dsc_t  wd_img_vivid_clock_hr = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 20,
    .data_size = 1600, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_HR),
    };
const lv_img_dsc_t  wd_img_vivid_clock_music = {
    .header.always_zero = 0,
    .header.w = 35,
    .header.h = 41,
    .data_size = 5740, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_MUSIC),
    };
const lv_img_dsc_t  wd_img_vivid_clock_needle_hour = {
    .header.always_zero = 0,
    .header.w = 96,
    .header.h = 12,
    .data_size = 4608, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_NEEDLE_HOUR),
    };
const lv_img_dsc_t  wd_img_vivid_clock_needle_minute = {
    .header.always_zero = 0,
    .header.w = 128,
    .header.h = 12,
    .data_size = 6144, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_NEEDLE_MINUTE),
    };
const lv_img_dsc_t  wd_img_vivid_clock_needle_second = {
    .header.always_zero = 0,
    .header.w = 177,
    .header.h = 5,
    .data_size = 3540, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_NEEDLE_SECOND),
    };
const lv_img_dsc_t  wd_img_vivid_clock_step = {
    .header.always_zero = 0,
    .header.w = 20,
    .header.h = 20,
    .data_size = 1600, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_STEP),
    };
const lv_img_dsc_t  wd_img_vivid_clock_sun = {
    .header.always_zero = 0,
    .header.w = 25,
    .header.h = 25,
    .data_size = 2500, 
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (uint8_t*)(QSPI0_XIP_BASE + ADDR_VIVID_CLOCK_SUN),
    };

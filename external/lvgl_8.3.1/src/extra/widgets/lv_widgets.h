/**
 * @file lv_widgets.h
 *
 */

#ifndef LV_WIDGETS_H
#define LV_WIDGETS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "animimg/lv_animimg.h"
#include "calendar/lv_calendar.h"
#include "calendar/lv_calendar_header_arrow.h"
#include "calendar/lv_calendar_header_dropdown.h"
#include "chart/lv_chart.h"
#include "keyboard/lv_keyboard.h"
#include "list/lv_list.h"
#include "menu/lv_menu.h"
#include "msgbox/lv_msgbox.h"
#include "meter/lv_meter.h"
#include "spinbox/lv_spinbox.h"
#include "spinner/lv_spinner.h"
#include "tabview/lv_tabview.h"
#include "tileview/lv_tileview.h"
#include "win/lv_win.h"
#include "colorwheel/lv_colorwheel.h"
#include "led/lv_led.h"
#include "imgbtn/lv_imgbtn.h"
#include "span/lv_span.h"

#if LV_GDX_PATCH_USE_FAST_TILEVIEW
#include "tileview/lv_fast_tileview.h"
#endif

#if LV_GDX_PATCH_USE_WMS_TILEVIEW
#include "./goodix/tileview/lv_wms_tileview.h"
#endif

#if LV_GDX_PATCH_USE_GPU_ENHANCED_LIST
#include "./goodix/gpu_enhanced_menu/lv_enhanced_list.h"
#endif // LV_GDX_PATCH_USE_GPU_ENHANCED_LIST

#if LV_GDX_PATCH_USE_GPU_ENHANCED_GRID
#include "./goodix/gpu_enhanced_menu/lv_enhanced_grid.h"
#endif // LV_GDX_PATCH_USE_GPU_ENHANCED_GRID

#if LV_GDX_PATCH_USE_GPU_ENHANCED_SCROLLBAR
#include "./goodix/gpu_scrollbar/lv_enhanced_scrollbar.h"
#endif // LV_GDX_PATCH_USE_GPU_ENHANCED_SCROLLBAR

#if LV_GDX_PATCH_USE_GPU_CIRCULAR_LIST
#include "./goodix/gpu_enhanced_menu/lv_circular_list.h"
#endif // LV_GDX_PATCH_USE_GPU_CIRCULAR_LIST

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_WIDGETS_H*/

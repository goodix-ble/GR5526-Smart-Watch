#include "lvgl.h"
#include "lv_wms.h"
#include "lv_wms_scene.h"
#include "stdio.h"

#if WMS_VERSION == WMS_VERSION_v2

void lv_wms_key_event_handler (uint32_t key, uint32_t event) {
    bool handled = false;
    lv_wms_window_t * p_cur_win = NULL;
    uint16_t cur_win_id = lv_wms_get_cur_win_id();

    p_cur_win = lv_wms_window_lookup(cur_win_id);

    if(p_cur_win != NULL)
        handled = lv_wms_key_handler_execute(p_cur_win->window, key, event);

    if(!handled) {
        printf("MAIN Handle: %d - %d \r\n", key, event);
    }
}

#else

void lv_wms_key_event_handler (uint32_t key, uint32_t event){}

#endif

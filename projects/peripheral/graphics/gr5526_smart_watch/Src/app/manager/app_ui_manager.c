#include <stdio.h>
#include "lv_layout_router.h"
#include "lv_layout_app_menu.h"
#include "app_ui_manager.h"

#define ROOT_MAP_ID     TILEVIEW_MAP_ID_MAIN_SCREEN
#define ROOT_ROW_ID     0
#define ROOT_COL_ID     0

extern lv_obj_t *                       lv_layout_router_get_active_obj(void);
extern lv_wms_tileview_tile_info_t *    lv_layout_router_get_active_tile_info(void);

bool app_ui_mgr_is_at_root(void) {

    lv_wms_tileview_tile_info_t * tinfo = lv_layout_router_get_active_tile_info();

    if((tinfo->map_id == ROOT_MAP_ID) &&
       (tinfo->row    == ROOT_ROW_ID) &&
       (tinfo->col    == ROOT_COL_ID) ) {
        return true;
    }

    return false;
}

bool app_ui_mgr_is_at_applist(void) {
    lv_obj_t * cur = lv_layout_router_get_active_obj();

    return lv_obj_check_type(cur, &lv_enhanced_list_class);
}

void app_ui_mgr_goto_root(void) {
    if (app_ui_mgr_is_at_root())
    {
        return;
    }

    lv_wms_tileview_tile_info_t *tile_info = lv_layout_router_get_active_tile_info();
    lv_wms_tileview_transition_effect_t effect;
    lv_wms_tileview_pos_t birthplace;
    if (tile_info->map_id == TILEVIEW_MAP_ID_MAIN_SCREEN)
    {
        // Same map but not same tile, use effect int symmetric direction
        effect = tile_info->cfg.effect;
        if (tile_info->row > 0)
        {
            // Center at top
            birthplace = LV_WMS_TILEVIEW_POS_AT_UP;
        }
        else if (tile_info->row < 0)
        {
            // Center at bottom
            birthplace = LV_WMS_TILEVIEW_POS_AT_DOWN;
        }
        else if (tile_info->col > 0)
        {
            // Centera at left
            birthplace = LV_WMS_TILEVIEW_POS_AT_LEFT;
        }
        else
        {
            // Center at right (or elsewhere, technically this won't happen)
            birthplace = LV_WMS_TILEVIEW_POS_AT_RIGHT;
        }
    }
    else
    {
        // Not same map, use direction-independent effect
        effect = WMS_DEFAULT_GOTO_EFFECT;
        birthplace = LV_WMS_TILEVIEW_POS_AT_LEFT;
    }

    lv_wms_tileview_goto(lv_layout_router_get_active_obj(), ROOT_MAP_ID, ROOT_ROW_ID, ROOT_COL_ID, birthplace, effect, 0, NULL);
}

void app_ui_mgr_goto_applist(void) {
    if(!app_ui_mgr_is_at_applist()) {
        lv_layout_router_goto_isolate_win(lv_layout_router_get_active_obj(), WMS_WID_APP_LIST, WMS_DEFAULT_GOTO_EFFECT_POS, WMS_DEFAULT_GOTO_EFFECT);
    }
}

bool app_ui_mgr_is_aod_off(void) {
    return false;
}

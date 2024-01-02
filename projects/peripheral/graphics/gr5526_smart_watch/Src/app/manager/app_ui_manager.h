#ifndef __APP_UI_MANAGER_H__
#define __APP_UI_MANAGER_H__

#include <stdint.h>
#include <stdbool.h>

bool app_ui_mgr_is_at_root(void) ;

bool app_ui_mgr_is_at_applist(void);

void app_ui_mgr_goto_root(void);

void app_ui_mgr_goto_applist(void);

bool app_ui_mgr_is_aod_off(void);

#endif /* __APP_UI_MANAGER_H__ */

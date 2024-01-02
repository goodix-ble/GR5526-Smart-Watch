#include "stdio.h"
#include "app_sys_manager.h"


sys_task_handle_t g_task_handle = {
    .gui_handle           = NULL,
    .indev_handle         = NULL,
    .gui_evt_handle       = NULL,
    .ota_handle           = NULL,
};


sys_param_t g_sys_running_rec = {
    .running_state          = SYS_STATE_UNSET,
    .running_state_base_ms  = 0,
};


void sys_state_reset_base_time(void) {
    g_sys_running_rec.running_state_base_ms = osal_task_get_tick_count();
    printf("Reset Tick: %d \r\n", g_sys_running_rec.running_state_base_ms);
}

uint32_t sys_state_get_base_time(void) {
    return g_sys_running_rec.running_state_base_ms;
}

uint32_t sys_state_calc_delta_time(void) {
    uint32_t cur_tick = osal_task_get_tick_count();
    if(cur_tick >= g_sys_running_rec.running_state_base_ms) {
        return (cur_tick - g_sys_running_rec.running_state_base_ms);
    } else { /* overflow */
        return (0xffffffff - g_sys_running_rec.running_state_base_ms + cur_tick);
    }
}

void sys_state_switch(sys_running_state_e new_state) {
    g_sys_running_rec.running_state = new_state;
}

sys_running_state_e sys_state_get(void) {
    return g_sys_running_rec.running_state;
}

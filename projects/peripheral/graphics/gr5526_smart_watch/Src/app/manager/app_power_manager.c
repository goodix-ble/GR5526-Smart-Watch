#include "app_power_manager.h"
#include "drv_adapter_norflash.h"
#include "drv_adapter_touchpad.h"
#include "app_graphics_dc.h"


void sys_peripherals_sleep(void) {
    /* power off the peripherals one by one */
    //drv_adapter_touchpad_sleep();
    drv_adapter_norflash_sleep();

    // mark dc to sleep
    app_graphics_dc_set_power_state(GDC_POWER_STATE_SLEEP);
}


void sys_peripherals_resume(void) {
    /* resume the peripherals one by one */

    drv_adapter_norflash_wakeup();
    //drv_adapter_touchpad_wakeup();
}

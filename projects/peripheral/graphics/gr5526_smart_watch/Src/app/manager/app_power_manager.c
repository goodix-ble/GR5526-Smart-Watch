#include "app_power_manager.h"

#include "drv_adapter_norflash.h"
#include "drv_adapter_touchpad.h"


void sys_peripherals_sleep(void) {
    /* power off the peripherals one by one */
    //drv_adapter_touchpad_sleep();
    drv_adapter_norflash_sleep();
}


void sys_peripherals_resume(void) {
    /* resume the peripherals one by one */

    drv_adapter_norflash_wakeup();
    //drv_adapter_touchpad_wakeup();
}

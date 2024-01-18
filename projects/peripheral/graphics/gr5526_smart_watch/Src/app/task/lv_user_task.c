/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "grx_sys.h"
#include "platform_sdk.h"
#include "osal.h"
#include "osal_task.h"
#include "drv_adapter_display.h"
#include "app_sys_manager.h"
#include "app_ui_manager.h"
#include "app_power_manager.h"
#include "lv_port_gr5526.h"
#include "lv_port_indev.h"
#include "lv_layout_router.h"

#include "app_log.h"
#include "mock_data.h"

/*
 * MACRO DEFINITIONS
 *****************************************************************************************
 */


/*
 * GLOBAL PMU FUNCTION DECLARATIONS
 *****************************************************************************************
 */

/*
 * LOCAL MACRO DEFINITIONS
 *****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

static bool lv_env_is_inited = false;
static osal_sema_handle_t s_gui_refresh_sem;
static osal_sema_handle_t s_sleep_mgnt_sem;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void lvgl_env_init(void)
{
    lvgl_port_init();

    lv_env_is_inited = true;
    //app_rtc_init(NULL);
    osal_sema_binary_create(&s_gui_refresh_sem);
    osal_sema_binary_create(&s_sleep_mgnt_sem);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void app_gui_render_task(void *p_arg)
{
    uint32_t delayTime = 0;

    mock_data_init_pre_gui();
    lvgl_env_init();
    lv_layout_router_init();
    mock_data_init_post_gui();

    while(1){
        delayTime = lv_task_handler();

        if(SYS_STATE_SLEEP != sys_state_get()) {
            //osal_task_delay(delayTime);
            osal_sema_take(s_gui_refresh_sem, delayTime);
        } else {
            printf("Suspend GUI Task \r\n");
            osal_sema_take(s_gui_refresh_sem, OSAL_MAX_DELAY);
            printf("Resume GUI Task \r\n");
        }
    }
}


static void _lv_reload_gui(void* user_data) {
    lv_obj_invalidate(lv_scr_act());
}

extern bool app_graphics_gpu_is_idle(void);

void app_indev_read_task(void * args)
{
    bool is_press_evt;

    while(1) {
        switch(sys_state_get()) {

            case SYS_STATE_UNSET:
            {
                if(lv_env_is_inited){
                    sys_state_switch(SYS_STATE_ACTIVE);
                } else {
                    osal_task_delay(100);
                }
            }
            break;

            case SYS_STATE_ACTIVE:
            {
                is_press_evt = lv_port_indev_poll();
                if(is_press_evt) {
                    osal_sema_give(s_gui_refresh_sem);
                    osal_task_delay(LV_INDEV_DEF_READ_PERIOD);
                    sys_state_reset_base_time();
                } else {
                    /* no tp-press event, wait in period-semaphore */
                    osal_sema_take(s_sleep_mgnt_sem, 500);
                    if(sys_state_calc_delta_time() >= SYS_SCREEN_OFF_WAIT_TIME_MS) {

                        bool    is_aod_off = false;
                        uint8_t wait_count = 15;

                        // Stop Refresh firstly.
                        lv_wms_refresh_enabled_set(false);
                        osal_task_delay_ms(100);

                        // wait GPU to idle
                        while(wait_count--) {
                            if(app_graphics_gpu_is_idle()) {
                                is_aod_off = true;
                                break;
                            } else {
                                osal_task_delay_ms(10);
                                printf("+");
                            }
                        }

                        if(is_aod_off) {
                            printf("GOTO SCREEN OFF, tick: %d \r\n", osal_task_get_tick_count());
                            sys_state_switch(SYS_STATE_SCREEN_OFF);
                            lv_wms_display_enabled_set(false);
                            drv_adapter_disp_on(false);
                            sys_state_reset_base_time();
                        } else {
                            printf("Fail to Screen Off, keep Active!\r\n ");
                            lv_wms_refresh_enabled_set(true);
                            sys_state_reset_base_time();
                        }
                    }
                }
            }
            break;

            case SYS_STATE_SCREEN_OFF:
            {
                //clear tp and show off the display
                if(OSAL_SUCCESS != osal_sema_take(s_sleep_mgnt_sem, 500)) {
                    if(sys_state_calc_delta_time() >= SYS_SLEEP_WAIT_TIME_MS) {
                        sys_state_reset_base_time();
                        sys_state_switch(SYS_STATE_SLEEP);
                        printf("GOTO SLEEP, tick: %d \r\n", osal_task_get_tick_count());
                    }
                } else {
                    printf("TP Evt Occurs, Return to Active!\r\n");
                    lv_wms_refresh_enabled_set(true);
                    lv_wms_display_enabled_set(true);
                    drv_adapter_disp_on(true);
                    sys_state_reset_base_time();
                    sys_state_switch(SYS_STATE_ACTIVE);
                }
            }
            break;

            case SYS_STATE_SLEEP:
            {
                printf("GOTO Sleep!\r\n");
                sys_peripherals_sleep();

                osal_sema_take(s_sleep_mgnt_sem, OSAL_MAX_DELAY);

                printf("Resume from Sleep!\r\n");
                sys_peripherals_resume();
                lv_wms_refresh_enabled_set(true);
                lv_wms_display_enabled_set(true);
                drv_adapter_disp_on(true);
                lv_async_call(_lv_reload_gui, NULL);
                osal_sema_give(s_gui_refresh_sem);

                sys_state_reset_base_time();
                sys_state_switch(SYS_STATE_ACTIVE);
                printf("GOTO Active, tick: %d \r\n", osal_task_get_tick_count());
            }
            break;

            default:break;
        }
    }
}

/*
 * Override this function, defined as __weak in file drv_adapter_port_touchpad.c
 */
void _touchpad_drv_irq_notify(void) {
    osal_sema_give(s_sleep_mgnt_sem);
}

void _key_drv_irq_notify(void) {
    if(SYS_STATE_SLEEP == sys_state_get()) {
        osal_sema_give(s_sleep_mgnt_sem);
    }
}

/**
 *****************************************************************************************
 * @brief To create two task, the one is ble-schedule, another is watcher task
 *****************************************************************************************
 */
void lv_user_task_create(void)
{
    osal_task_create("task_indev", app_indev_read_task, TASK_SIZE_TOUCHPAD_READ, TASK_PRIO_TOUCHPAD_READ, &g_task_handle.indev_handle);
    osal_task_create("task_gui",   app_gui_render_task, TASK_SIZE_GUI_RENDER,    TASK_PRIO_GUI_RENDER,    &g_task_handle.gui_handle);

    extern void lv_gui_evt_task_startup(void);
    lv_gui_evt_task_startup();
}

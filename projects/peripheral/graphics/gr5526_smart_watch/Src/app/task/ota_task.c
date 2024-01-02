/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "osal.h"
#include "dfu_port.h"
#include "app_sys_manager.h"

#define OTA_TASK_PRIORITY (6)
/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */
static void dfu_program_start_callback(void);
static void dfu_programing_callback(uint8_t pro);
static void dfu_program_end_callback(uint8_t status);
static void dfu_enter(void);

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
static osal_sema_handle_t s_ota_startup_sem = NULL;

static bool s_ota_running = false;

static dfu_pro_callback_t dfu_pro_call =
{
    .dfu_program_start_callback = dfu_program_start_callback,
    .dfu_programing_callback    = dfu_programing_callback,
    .dfu_program_end_callback   = dfu_program_end_callback,
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void _ota_schedule_task(void *p_arg)
{
    while(1){
        if (s_ota_running)
        {
            dfu_schedule();
        }
        else
        {
            osal_sema_take(s_ota_startup_sem, OSAL_MAX_DELAY);
            fast_dfu_state_machine_reset();
        }
    }
}

/**
 * @brief this function should be called in services_init().
 */
void ota_task_create(void)
{
    if (g_task_handle.ota_handle == NULL)
    {
        dfu_port_init(NULL, 0x00240000, &dfu_pro_call);

        dfu_service_init(dfu_enter);

        osal_mutex_create(&s_ota_startup_sem);
        osal_task_create("ota_task", _ota_schedule_task, TASK_SIZE_OTA, TASK_PRIO_OTA, &g_task_handle.ota_handle);
    }
}

void ota_task_set_enable(bool enable)
{
    if (enable != s_ota_running)
    {
        if (enable)
        {
            s_ota_running = true;
            osal_sema_give(s_ota_startup_sem);
        }
        else
        {
            s_ota_running = false;
            fast_dfu_state_machine_reset();
        }
    }
}

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void dfu_program_start_callback(void)
{
}

static void dfu_programing_callback(uint8_t pro)
{
}

static void dfu_program_end_callback(uint8_t status)
{
}

static void dfu_enter(void)
{
    ota_task_set_enable(true);
}

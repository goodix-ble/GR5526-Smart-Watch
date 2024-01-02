
/*-----------------------------------------------------------
* Implementation of functions defined in portable.h for the ARM CM4F port.
*----------------------------------------------------------*/

/* Scheduler includes. */
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "grx_hal.h"
#include "grx_sys.h"
#include "gr_soc.h"
#include "FreeRTOSConfig.h"

#if( configOVERRIDE_DEFAULT_TICK_CONFIGURATION == 1 )
/*
Interfaces that must be implemented:
    vPortSetupTimerInterrupt()
    systick_sleep_timeout_set()
    systick_compensate_restart()
*/
/*
 * Global VARIABLE DECLARATIONS
 *****************************************************************************************
 */
extern void xPortSysTickHandler(void);
extern uint32_t SystemSlowClock;
extern uint32_t SystemRngClock;

/*
 * LOCAL MACRO DEFINITIONS
 *****************************************************************************************
 */
#define MAX_SYS_SLEEP_TICKS            (100000) /* Unit: RTOS TICK */

#if !CFG_LPCLK_INTERNAL_EN
#define RTC_CLOCK_HZ                   (SystemSlowClock)
#else
/* If CFG_LPCLK_INTERNAL_EN == 1, RTC1 using RNG clock. */
#define RTC_CLOCK_HZ                   (SystemRngClock)
#endif
#define RTC_DIV_NUM                    (RTC_DIV_NONE)
#define RTC_COUNT_PERIOD_TICK          ((uint32_t) ((RTC_CLOCK_HZ/configTICK_RATE_HZ)))
#define RTCCOUNT_TO_RTOSTICK(COUNT)    ((((uint64_t)(COUNT))*configTICK_RATE_HZ)/RTC_CLOCK_HZ)
#define RTOSTICK_TO_RTCCOUNT(TICK)     ((((uint64_t)(TICK))*RTC_CLOCK_HZ)/configTICK_RATE_HZ)

/*
* LOCAL VARIABLE DEFINITIONS
*****************************************************************************************
*/
static rtc_handle_t s_rtc1_handle;
static uint32_t s_expect_sleep_ticks = 0;
static uint32_t s_sys_tick_cnt = 0;
static uint32_t s_wakeup_tick_cnt = 0;
static uint32_t s_rtc_tick_cnt = 0;

/*
* LOCAL FUNCTION IMPLEMENTATIONS
*****************************************************************************************
*/
SECTION_RAM_CODE static uint32_t vPortLocker(void)
{
    uint32_t ret_pri = __get_PRIMASK();
    __set_PRIMASK(1);
    return ret_pri;
}

SECTION_RAM_CODE static void vPortUnLocker(uint32_t set_pri)
{
    __set_PRIMASK(set_pri);
}

SECTION_RAM_CODE static uint32_t vDisableIRQExeptBLE(void)
{
    uint32_t __l_irq_rest=__get_BASEPRI();
    __set_BASEPRI(NVIC_GetPriority(BLE_IRQn) +(1 << (NVIC_GetPriorityGrouping() + 1)));
    return __l_irq_rest;
}

SECTION_RAM_CODE static void vRestoreIRQExeptBLE(uint32_t set_pri)
{
      __set_BASEPRI(set_pri);
}

/*
 * WEAK Implementation to support graphics power management
 *****************************************************************************************
 */
__WEAK void app_graphics_gpu_sleep(void){}
__WEAK void app_graphics_dc_sleep(void){}
__WEAK void app_graphics_ospi_sleep(void){}
__WEAK void app_graphics_ospi_wakeup(void){}

SECTION_RAM_CODE void wfe_func(void)
{
    __WFI();
}

SECTION_RAM_CODE void RTC1_IRQHandler(void)
{
    hal_rtc_irq_handler(&s_rtc1_handle);
}

SECTION_RAM_CODE void hal_rtc_tick_callback(rtc_handle_t *p_rtc)
{
    if((AUTO_RELOAD == p_rtc->tick.mode) && \
    (HAL_RTC_ERROR_NONE == p_rtc->error_code))
    {
        xPortSysTickHandler();
    }
}

void vPortSetupTimerInterrupt( void )
{
    s_rtc1_handle.p_instance = RTC1;
    s_rtc1_handle.init.prescaler_div = RTC_DIV_NUM;
    s_rtc1_handle.init.overflow_det_state = OPENED;
    s_rtc1_handle.init.start_value = 0x0;
    soc_register_nvic(RTC1_IRQn, (uint32_t)RTC1_IRQHandler);
    hal_rtc_deinit(&s_rtc1_handle);
#if CFG_LPCLK_INTERNAL_EN
    ll_rtc_timer_set_clk(RTC1, LL_RTC_TIMER_CLK_SEL_RNG);
#endif
    hal_rtc_init(&s_rtc1_handle);
    NVIC_SetPriority(RTC1_IRQn, configKERNEL_INTERRUPT_PRIORITY);
    hal_rtc_stop_tick(&s_rtc1_handle);
    hal_rtc_set_tick_and_start(&s_rtc1_handle, AUTO_RELOAD, RTC_COUNT_PERIOD_TICK);
    s_rtc_tick_cnt = ll_rtc_get_read_counter(s_rtc1_handle.p_instance);
    s_sys_tick_cnt = xTaskGetTickCount();
}

SECTION_RAM_CODE static void systick_sleep_timeout_set(uint32_t timeout_tick)
{
    hal_rtc_stop_tick(&s_rtc1_handle);
    // Limit the maximum sleep time
    if( timeout_tick > MAX_SYS_SLEEP_TICKS )
    {
        timeout_tick = MAX_SYS_SLEEP_TICKS;
    }
    if( timeout_tick > s_wakeup_tick_cnt)
    {
        timeout_tick -= s_wakeup_tick_cnt;
    }
    uint32_t rtc_count = RTOSTICK_TO_RTCCOUNT(timeout_tick);
    hal_rtc_set_tick_and_start(&s_rtc1_handle, ONE_TIME, rtc_count);
}

SECTION_RAM_CODE static uint32_t system_sleep_tick_compensate(void)
{
    uint32_t cur_rtc_tick_cnt = ll_rtc_get_read_counter(s_rtc1_handle.p_instance);
    uint32_t cur_sys_tick_cnt = xTaskGetTickCount();
    uint32_t rtos_tick_diff = cur_sys_tick_cnt - s_sys_tick_cnt;
    uint32_t rtc_tick_diff = cur_rtc_tick_cnt - s_rtc_tick_cnt;
    uint32_t rtc_2_rtos_tick_diff = RTCCOUNT_TO_RTOSTICK(rtc_tick_diff);
    // No need to compensate when exit sleep within 1 tick
    if( rtc_2_rtos_tick_diff <= rtos_tick_diff )
    {
        return 0;
    }
    uint32_t step_tick = rtc_2_rtos_tick_diff - rtos_tick_diff;
    // Limit the compensate tick within normal range
    if( step_tick > s_expect_sleep_ticks )
    {
        s_wakeup_tick_cnt = step_tick - s_expect_sleep_ticks;
        step_tick = s_expect_sleep_ticks;
    }
    else
    {
        s_wakeup_tick_cnt = 0;
    }
    vTaskStepTick(step_tick);
    // Update the tick reference every 100 seconds to reduce deviation
    // Notice the RTC frequency may change with the temperature change
    if( rtos_tick_diff > MAX_SYS_SLEEP_TICKS )
    {
        s_rtc_tick_cnt = cur_rtc_tick_cnt;
        s_sys_tick_cnt = xTaskGetTickCount();
    }
    return step_tick;
}

SECTION_RAM_CODE static void systick_compensate_restart(void)
{
    system_sleep_tick_compensate();
    hal_rtc_set_tick_and_start(&s_rtc1_handle, AUTO_RELOAD, RTC_COUNT_PERIOD_TICK);
}

SECTION_RAM_CODE static uint8_t pwr_mgmt_system_sleep(void)
{
    app_graphics_gpu_sleep();
    app_graphics_dc_sleep();
    app_graphics_ospi_sleep();
    uint8_t ret = pwr_mgmt_sleep();
    if ( ret != PMR_MGMT_SUCCESS )
    {
        app_graphics_ospi_wakeup();
    }
    return ret;
}

SECTION_RAM_CODE static void pwr_mgmt_enter_sleep_with_cond(TickType_t pwr_mgmt_expected_time)
{
    s_expect_sleep_ticks = pwr_mgmt_expected_time;
    uint32_t pwr_locker = vPortLocker();
    uint32_t ble_unlocker;
    if ( eTaskConfirmSleepModeStatus() == eAbortSleep )
    {
        vPortUnLocker(pwr_locker);
        return ;
    }

    systick_sleep_timeout_set(s_expect_sleep_ticks);
    if ( (s_expect_sleep_ticks < 5) ||
        (PMR_MGMT_SLEEP_MODE != pwr_mgmt_mode_get()) ||
        (DEVICE_BUSY == pwr_mgmt_dev_suspend()) )
    {
        wfe_func();
        //Enable BLE IRQ during systick compensate for ble schedule on time
        ble_unlocker = vDisableIRQExeptBLE();
        vPortUnLocker(pwr_locker);
        systick_compensate_restart();
        vRestoreIRQExeptBLE(ble_unlocker);
        return ;
    }

    uint8_t ret = pwr_mgmt_system_sleep();
    if (ret != PMR_MGMT_SUCCESS)
    {
        wfe_func();
        //Enable BLE IRQ during systick compensate for ble schedule on time
        ble_unlocker = vDisableIRQExeptBLE();
        vPortUnLocker(pwr_locker);
        systick_compensate_restart();
        vRestoreIRQExeptBLE(ble_unlocker);
    }
    else // Wakeup from deep sleep mode
    {
        uint32_t _local_lock = vPortLocker();
        systick_compensate_restart();
        vPortUnLocker(_local_lock);
    }
}
#endif /* configOVERRIDE_DEFAULT_TICK_CONFIGURATION */

/**
*****************************************************************************************
* @brief vPortEnterDeepSleep
*
* @param[in] xExpectedIdleTime: next task resume time to work
*
* @return void
*****************************************************************************************
*/
SECTION_RAM_CODE void vPortEnterDeepSleep( TickType_t xExpectedIdleTime )
{
#if( configOVERRIDE_DEFAULT_TICK_CONFIGURATION == 1 )
    pwr_mgmt_enter_sleep_with_cond(xExpectedIdleTime);
#else
    __WFI();
#endif
}

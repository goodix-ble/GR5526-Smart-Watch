/**
 *****************************************************************************************
 *
 * @file gr55xx_pwr.h
 *
 * @brief GR55XX Platform Power Manager Module API
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/**
 * @addtogroup SYSTEM
 * @{
 */
 
/**
 * @addtogroup PWR Power Manager
 * @{
 * @brief Definitions and prototypes for the Power Manager interface.
 */


#ifndef __GR55XX_PWR_H_
#define __GR55XX_PWR_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "system_gr55xx.h"

/**
 * @defgroup GR55XX_PWR_DEFINE Defines
 * @{
 */

/**@brief PMU error code. */
#define PMR_MGMT_SUCCESS  0x0
#define PMR_MGMT_FAIL     0xFF
#define PWR_MGMT_BB_CHECK_FAIL 0xFE
#define PWR_MGMT_MODE_CHECK_FAIL 0xFD

/** @} */

/**
 * @addtogroup GR55XX_PWR_STRUCTURES Structures
 * @{
 */
/**
 * @brief power manager setting parameter.
 *        Use pwr_mgmt_var_set to transfer the parameters in the structure to PMU, 
 *        and then the pwr_mgmt_mode_set function will use the new parameters for
 *        power management.
 *        Note that this is an advanced API, the wrong setting of parameters may 
 *        lead to abnormal power management, so please use it carefully.
 */
typedef struct
{
    uint32_t  pwr_mgmt_app_timer_thrd;    /**< App timer threshold. */
    uint32_t  pwr_mgmt_ble_core_thrd;     /**< BLE timer threshold. */
    uint32_t  pwr_mgmt_rtc_timer_thrd;    /**< RTC timer threshold. */
    uint32_t  pwr_mgmt_wdt_timer_thrd;    /**< AON WDT threshold. */
} pwr_mgmt_var_box_t;

/** @} */

/**
 * @addtogroup GR55XX_PWR_ENUM Enumerations
 * @{
 */
/**@brief power manager boot type. */
typedef enum 
{
    COLD_BOOT = 0,                    /**< Cold boot state. */
    WARM_BOOT,                        /**< Warm boot state. */
} boot_mode_t;

/**@brief power manager model. */
typedef enum
{
    PMR_MGMT_ACTIVE_MODE = 0x0,       /**< Full speed state. */
    PMR_MGMT_IDLE_MODE,               /**< Idle state. */
    PMR_MGMT_SLEEP_MODE,              /**< Deep sleep state. */
} pwr_mgmt_mode_t;

/**@brief power manager device work state. */
typedef enum
{
    DEVICE_BUSY = 0x0,                /**< Device busy state. */
    DEVICE_IDLE,                      /**< Device idle state. */
} pwr_mgmt_dev_state_t;

/**@brief power manager app timer work state. */
typedef enum
{
    EVENT_APP_TIMER_START = 0,        /**< App-timer start state. */
    EVENT_APP_TIMER_STOP,             /**< App-timer stop state. */
} notify_timer_event_t;

/**@brief  PMU Tracking*/
enum
{
   TRC_PWR_WFE_MODE = 0,           /**< WFE mode. */
   TRC_PWR_DSLEEP_MODE,            /**< Deep sleep mode. */
   TRC_PWR_ACTIVE_MODE,            /**< Active mode. */
   TRC_PWR_BLE_RET_DSLEEP,         /**< BLE return deep sleep. */
   TRC_PWR_APP_TIMER_REFUSE,       /**< App timer refuse. */
   TRC_PWR_APP_TIMER_PASS,         /**< App timer pass. */
   TRC_PWR_BLE_TIMER_REFUSE,       /**< BLE timer refuse. */
   TRC_PWR_BLE_TIMER_PASS,         /**< BLE timer pass. */
   TRC_PWR_RTC_TIMER_REFUSE,       /**< RTC timer refuse. */
   TRC_PWR_RTC_TIMER_PASS,         /**< RTC timer pass. */
   TRC_PWR_RTC1_TIMER_REFUSE,       /**< RTC timer refuse. */
   TRC_PWR_RTC1_TIMER_PASS,         /**< RTC timer pass. */
   TRC_PWR_WDT_TIMER_REFUSE,       /**< AON WDT timer refuse. */
   TRC_PWR_WDT_TIMER_PASS,         /**< AON WDT timer pass. */
};

/** @} */

/**
 * @addtogroup GR55XX_PWR_STRUCTURES Structures
 * @{
 */
/**@brief parameter configuration table. */ 
typedef struct 
{
   uint16_t pwr_dur;             /**< Duration. */
   uint16_t pwr_ext;             /**< External wake-up. */
   uint16_t pwr_osc;             /**< OSC. */
   uint8_t  pwr_delay_hslot;     /**< Delay half slot. */
   uint16_t pwr_delay_hus;       /**< Delay half us. */
   uint16_t pwr_push_hus;        /**< Push half us. */
   uint32_t pwr_timer_ths;       /**< APP timer threshold. */
   uint32_t pwr_ble_ths;         /**< BLE timer threshold. */
} pwr_table_t; 

/** @} */

/**
 * @addtogroup GR55XX_PWR_TYPEDEFS Type definitions
 * @{
 */
/**@brief Trace function type. */ 
typedef void (*trace_func_t)(uint8_t);

/**@brief Peripheral function type. */ 
typedef void (*periph_func_t)(void);

/**@brief Before sleep function type. */ 
typedef void (*pwr_before_sleep_func_t)(void);

/**@brief Device check function type. */ 
typedef pwr_mgmt_dev_state_t (*pwr_dev_check_func_t)(void);

/**@brief mem check process type. */ 
typedef void (*mem_check_proc_t)(void);

/**@brief SRPG before function type. */ 
typedef uint8_t (*srpg_before_func_t)(void);

/** @} */

/**
 * @addtogroup GR55XX_PWR_VARIABLE Variable
 * @{
 */
/**@brief pwr table. */ 
extern pwr_table_t pwr_table[];
 
/** @} */

/** @addtogroup GR55XX_PWR_FUNCTIONS Functions
 * @{ */
/**
 *****************************************************************************************
 * @brief This function allows ARM to enter deep sleep mode, but users should not use this 
 *        function directly.
 *        Note that this function is only available in environments where non-RTOS is used, 
 *             and that users can only execute it while in main.c.
 * @returns Power Management mode
 *****************************************************************************************
 */
pwr_mgmt_mode_t pwr_mgmt_shutdown(void);

/**
 ****************************************************************************************
 * @brief  Get the current boot mode.     
 * @returns  Boot mode.
 ****************************************************************************************
 */
boot_mode_t pwr_mgmt_get_wakeup_flag(void);

/**
 ****************************************************************************************
 * @brief  Mark the mode of next boot, cold boot or warm boot.
 * @param[in] boot_mode : cold boot or warm boot.
 ****************************************************************************************
 */
void pwr_mgmt_set_wakeup_flag(boot_mode_t boot_mode);

/**
 ****************************************************************************************
 * @brief  Set the specified sleep mode. When the setting is completed, the system will
 *         automatically enter the specified sleep mode through the strategy.  
 * @param[in] pm_mode : sleep level
 ****************************************************************************************
 */
void pwr_mgmt_mode_set(pwr_mgmt_mode_t pm_mode);

/**
 ****************************************************************************************
 * @brief       Get the specified sleep mode.   
 * @retval      Power management mode
 ****************************************************************************************
 */
pwr_mgmt_mode_t pwr_mgmt_mode_get(void);

/**
 ****************************************************************************************
 * @brief       Get the power state of baseband.   
 * @retval      Power management mode
 ****************************************************************************************
 */
pwr_mgmt_mode_t pwr_mgmt_baseband_state_get(void);

/**
 ****************************************************************************************
 * @brief       Get the state of extenal timer.   
 * @retval      Power management mode
 ****************************************************************************************
 */
pwr_mgmt_mode_t pwr_mgmt_check_ext_timer(void);



/**
 ****************************************************************************************
 * @brief  Sleep Policy Scheduling Function.
 *         Note that this function is only available in environments where non-RTOS is used, 
           and that users can only execute it while in main.c.
 ****************************************************************************************
 */
void pwr_mgmt_schedule(void);

/**
 ****************************************************************************************
 * @brief       Wake the BLE core via an external request.
 ****************************************************************************************
 */
void pwr_mgmt_ble_wakeup(void);


/**
 ****************************************************************************************
 * @brief  Check whether there are ble events in the queue, and if so, handle them immediately.
 ****************************************************************************************
 */
void pwr_mgmt_check_ble_event(void);


/**
 ****************************************************************************************
 * @brief  This function is used to push startup information in app timer. 
 *         This information will optimize power management strategy. 
 *         Note that this function is an advanced API and users should not use it directly.
 * @param[in] timer_event :  EVENT_APP_TIMER_START or EVENT_APP_TIMER_STOP
 ****************************************************************************************
 */
void pwr_mgmt_notify_timer_event(notify_timer_event_t timer_event);

/**
 ****************************************************************************************
 * @brief  Execution of this function allows ARM to enter the WFE state and exit the WFE 
 *         state when an event or interrupt occurs.
 ****************************************************************************************
 */
void pwr_mgmt_wfe_sleep(void);

/**
 ****************************************************************************************
 * @brief  Execution of this function allows ARM to enter the ultra sleep state and wakeup 
 *         the chip when an event occurs.
 * @param  time_ms : Specifies the wake-up time during ultra sleep. If time_ms is equal to 0,
 *                   then sleep timer will not be enabled.
 *                   This parameter must be a number between min_value = 0 and max_value = 131071
 ****************************************************************************************
 */
void pwr_mgmt_ultra_sleep(uint32_t time_ms);

/**
 ****************************************************************************************
 * @brief    PMU Initialization Function.
 * @param    p_pwr_table : PMU parameter configuration table.
 * @param    sys_clk     : the clock of system
 ****************************************************************************************
 */
void pwr_mgmt_init( pwr_table_t *p_pwr_table, mcu_clock_type_t sys_clk);

/**
 ****************************************************************************************
 * @brief    Peripheral Controller Initialization Register interface.
 * @param    p_periph_init : the pointer of device init function.
 ****************************************************************************************
 */
void pwr_mgmt_dev_init(periph_func_t p_periph_init);

/**
 ****************************************************************************************
 * @brief    Device config resume interface.
 ****************************************************************************************
 */
void pwr_mgmt_dev_resume(void);

/**
 ****************************************************************************************
 * @brief    Device config suspend interface.
 * @return   power manager device work state
 ****************************************************************************************
 */
pwr_mgmt_dev_state_t pwr_mgmt_dev_suspend(void);

/**
 ****************************************************************************************
 * @brief    Mem state control under deep sleep & work state.
 * @param    mem_sleep_state : control in deep sleep.
 * @param    mem_work_state  : control in work state.
 ****************************************************************************************
 */
void pwr_mgmt_mem_ctl_set(uint32_t mem_sleep_state, uint32_t mem_work_state);

/**
 ****************************************************************************************
 * @brief    Set PMU callback function.
 * @param    dev_check_fun    : Device check callback function.
 * @param    before_sleep_fun : Pre-execution callback function for deep sleep.
 ****************************************************************************************
 */
void pwr_mgmt_set_callback(pwr_dev_check_func_t dev_check_fun, pwr_before_sleep_func_t before_sleep_fun);

 /**
 ****************************************************************************************
 * @brief  Save context function.
 ****************************************************************************************
 */
void pwr_mgmt_save_context(void);

 /**
 ****************************************************************************************
 * @brief  Load context function.
 ****************************************************************************************
 */
void pwr_mgmt_load_context(void);

 /**
 ****************************************************************************************
 * @brief  Disable nvic irq.
 ****************************************************************************************
 */
void pwr_mgmt_disable_nvic_irq(void);

/**
 ****************************************************************************************
 * @brief  Enable nvic irq.
 ****************************************************************************************
 */
void pwr_mgmt_enable_nvic_irq(void);

/**
 ****************************************************************************************
 * @brief  Check nvic irq.
 ****************************************************************************************
 */
bool pwr_mgmt_check_pend_irq(void);

/**
 ****************************************************************************************
 * @brief shutdown power in rtos mode
 * @retval ::PMR_MGMT_SUCCESS: wakeup from shutdown mode.
 * @retval ::PMR_MGMT_FAIL: some condition refuse system enter shutdown mode.
 ****************************************************************************************
 */
uint8_t pwr_mgmt_sleep(void);

/**
 ****************************************************************************************
 * @brief  Trace function register.
 * @param[in] status_trace_func: Trace function.
 * @param[in] sched_trace_func: Trace function.
 ****************************************************************************************
 */
void pwr_mgmt_register_trace_func(trace_func_t status_trace_func, trace_func_t sched_trace_func);

 /**
  ****************************************************************************************
  * @brief    Register callback function before srpg sleep
  * @param    srpg_before_fun : Callback function before srpg sleep
  ****************************************************************************************
  */
void pwr_mgmt_register_srpg_before_func(srpg_before_func_t srpg_before_fun);

/** @} */

#endif
/** @} */
/** @} */

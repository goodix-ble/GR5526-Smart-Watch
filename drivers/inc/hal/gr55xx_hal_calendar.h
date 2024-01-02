/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_calendar.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of CALENDAR HAL library.
 *
 ****************************************************************************************
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
 ****************************************************************************************
 */

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup HAL_DRIVER HAL Driver
  * @{
  */

/** @defgroup HAL_CALENDAR CALENDAR
  * @brief CALENDAR HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_CALENDAR_H__
#define __GR55xx_HAL_CALENDAR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_calendar.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_CALENDAR_STRUCTURES Structures
  * @{
  */

/** @defgroup  CALENDAR_Time CALENDAR Time
  * @{
  */

/**
  * @brief  CALENDAR_Time calendar time structure definition
  */
typedef struct _calendar_time
{
    uint8_t sec;                    /**< Specifies the Calendar time seconds.
                                         This parameter must be a number between min_value = 0 and max_value = 59. */

    uint8_t min;                    /**< Specifies the Calendar time minutes.
                                         This parameter must be a number between min_value = 0 and max_value = 59. */

    uint8_t hour;                   /**< Specifies the Calendar time hour.
                                         This parameter must be a number between min_value = 0 and max_value = 23. */

    uint8_t date;                   /**< Specifies the Calendar date.
                                         This parameter must be a number between min_value = 1 and max_value = 31. */

    uint8_t mon;                    /**< Specifies the Calendar month.
                                         This parameter must be a number between min_value = 1 and max_value = 12. */

    uint8_t year;                   /**< Specifies the Calendar year which stars from 2010.
                                         This parameter must be a number between min_value = 0 and max_value = 99. */

    uint8_t week;                   /**< Specifies the Calendar weekday.
                                         This parameter must be a number between min_value = 0 and max_value = 6.  */

    uint16_t ms;                    /**< Specifies the Calendar time milliseconds.
                                        This parameter must be a number between min_value = 0 and max_value = 999. */
} calendar_time_t;

/**
  * @brief  CALENDAR_Alarm calendar alarm structure definition
  */
typedef struct _calendar_alarm
{
    uint8_t min;                    /**< Specifies the alarm time minutes.
                                         This parameter must be a number between min_value = 0 and max_value = 59. */

    uint8_t hour;                   /**< Specifies the alarm time hour.
                                         This parameter must be a number between min_value = 0 and max_value = 23. */

    uint8_t alarm_sel;              /**< Specifies the alarm is on date or weekday.
                                                 This parameter can be a value of @ref CALENDAR_ALARM_SEL. */

    uint8_t alarm_date_week_mask;   /**< Specifies the alarm date/weekday.
                                         If the alarm date is selected, this parameter must be set to a value in the 1 ~ 31 range.
                                         If the alarm weekday is selected, this parameter must be a value of @ref CALENDAR_ALARM_WEEKDAY. */

} calendar_alarm_t;

/** @} */

/** @defgroup CALENDAR_handle CALENDAR handle
  * @{
  */

/**
  * @brief  CALENDAR handle Structure definition
  */
typedef struct _calendar_handle
{
    uint32_t            utc;                    /**< The current UTC time of the system. */

    uint32_t            pre_count;              /**< The last register count value of the system. */

    calendar_alarm_t    alarm;                  /**< Specifies the Calendar date alarm. */

    hal_lock_t          lock;                   /**< Specifies the Calendar locking object. */

    uint32_t            interval;               /**< Specifies the Calendar milliseconds alarm. */

    uint8_t             mode;                   /**< Specifies the Calendar alarm mode. */

    float               clock_freq;             /**< CALENDAR clock frequce. Unit: Hz. */
} calendar_handle_t;

/** @} */

/** @} */

/** @addtogroup HAL_CALENDAR_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup CALENDAR_Callback CALENDAR Callback
  * @{
  */

/**
  * @brief HAL_CALENDAR Callback function definition
  */
typedef struct _calendar_callback
{
    void (*calendar_alarm_callback)(calendar_handle_t *p_calendar);     /**< CALENDAR date count complete callback */
    void (*calendar_tick_callback)(calendar_handle_t *p_calendar);      /**< CALENDAR tick count complete callback */
    void (*calendar_overflow_callback)(calendar_handle_t *p_calendar);      /**< CALENDAR overflow callback */
} calendar_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_CALENDAR_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup CALENDAR_Exported_Constants CALENDAR Exported Constants
  * @{
  */

/** @defgroup CALENDAR_Interrupts CALENDAR Interrupts
  * @{
  */
#define CALENDAR_IT_ALARM                   RTC_INT_EN_ALARM                        /**< Alarm interrupt    */
#define CALENDAR_IT_WARP                    RTC_INT_EN_WRAP                         /**< Warp interrupt     */
#define CALENDAR_IT_TICK                    RTC_INT_EN_TICK                         /**< Tick   interrupt   */
/** @} */

/** @defgroup CALENDAR_Flags CALENDAR Flags
  * @{
  */
#define CALENDAR_FLAG_ALARM                 RTC_INT_STAT_ALARM                      /**< Alarm interrupt flag   */
#define CALENDAR_FLAG_WARP                  RTC_INT_STAT_WRAP                       /**< Warp interrupt flag    */
#define CALENDAR_FLAG_TICK                  RTC_INT_STAT_TICK                       /**< Tick   interrupt flag  */
/** @} */

/** @defgroup CALENDAR_ALARM_SEL CALENDAR Alarm type select
  * @{
  */
#define CALENDAR_ALARM_SEL_DATE             (0UL)       /**< Alarm in date    */
#define CALENDAR_ALARM_SEL_WEEKDAY          (1UL)       /**< Alarm in weekday */
/** @} */

/** @defgroup CALENDAR_ALARM_WEEKDAY CALENDAR Alarm weekday
  * @{
  */
#define CALENDAR_ALARM_WEEKDAY_SUN          (0x01ul)    /**< Alarm weekday mask Sunday    */
#define CALENDAR_ALARM_WEEKDAY_MON          (0x02ul)    /**< Alarm weekday mask Monday    */
#define CALENDAR_ALARM_WEEKDAY_TUE          (0x04ul)    /**< Alarm weekday mask Tuesday   */
#define CALENDAR_ALARM_WEEKDAY_WED          (0x08ul)    /**< Alarm weekday mask Wednesday */
#define CALENDAR_ALARM_WEEKDAY_THU          (0x10ul)    /**< Alarm weekday mask Thursday  */
#define CALENDAR_ALARM_WEEKDAY_FRI          (0x20ul)    /**< Alarm weekday mask Friday    */
#define CALENDAR_ALARM_WEEKDAY_SAT          (0x40ul)    /**< Alarm weekday mask Saturday  */
/** @} */

/** @defgroup CALENDAR_ALARM_DISABLE CALENDAR Alarm mdoe
  * @{
  */
#define CALENDAR_ALARM_DISABLE_DATE         (1UL)       /**< Disable date alarm */
#define CALENDAR_ALARM_DISABLE_TICK         (2UL)       /**< Disable tick 0 alarm */
#define CALENDAR_ALARM_DISABLE_ALL          (CALENDAR_ALARM_DISABLE_DATE | CALENDAR_ALARM_DISABLE_TICK) /**< Disable all alarm */
/** @} */

/** @defgroup CALENDAR_ALARM_DISABLE CALENDAR Alarm mdoe
  * @{
  */
#define CALENDAR_TICK_SINGLE               LL_CLDR_TIMER_TICK_TYPE_SINGLE   /**< Alarm tick reload single   */
#define CALENDAR_TICK_AUTO                 LL_CLDR_TIMER_TICK_TYPE_AUTO     /**< Alarm tick reload auto     */

/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup CALENDAR_Exported_Macros CALENDAR Exported Macros
  * @{
  */

/** @brief  Enable the specified CALENDAR peripheral.
  * @retval None
  */
#define __HAL_CALENDAR_ENABLE()                             WRITE_REG(CALENDAR->CFG0, RTC_CFG0_CFG | RTC_CFG0_EN)

/** @brief  Disable the specified CALENDAR peripheral.
  * @retval None
  */
#define __HAL_CALENDAR_DISABLE()                            MODIFY_REG(CALENDAR->CFG0, 0xFFFFFFFF, RTC_CFG0_CFG);

/** @brief  Enable the specified CALENDAR interrupts.
  * @param  __INTERRUPT__ Specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg @ref CALENDAR_IT_ALARM Alarm Interrupt
  *            @arg @ref CALENDAR_IT_WARP  Warp Interrupt
  *            @arg @ref CALENDAR_IT_TICK Tick Interrupt
  * @retval None
  */
#define __HAL_CALENDAR_ENABLE_IT(__INTERRUPT__)             SET_BITS(CALENDAR->INT_EN, (__INTERRUPT__))

/** @brief  Disable the specified CALENDAR interrupts.
  * @param  __INTERRUPT__ Specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg @ref CALENDAR_IT_ALARM Alarm Interrupt
  *            @arg @ref CALENDAR_IT_WARP  Warp Interrupt
  *            @arg @ref CALENDAR_IT_TICK Tick Interrupt
  * @retval None
  */
#define __HAL_CALENDAR_DISABLE_IT(__INTERRUPT__)            CLEAR_BITS(CALENDAR->INT_EN, (__INTERRUPT__))

/** @brief  Check whether the specified CALENDAR interrupt flag is set or not.
  * @param  __FLAG__ Specifies the interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg @ref CALENDAR_FLAG_ALARM Alarm Interrupt event
  *            @arg @ref CALENDAR_FLAG_WARP  Warp Interrupt event
  *            @arg @ref CALENDAR_FLAG_TICK Tick Interrupt event
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __HAL_CALENDAR_GET_IT_SOURCE(__FLAG__)              (READ_BITS(CALENDAR->INT_STAT, (__FLAG__)) == (__FLAG__))

/** @brief  Clear the specified CALENDAR flag.
  * @param  __FLAG__ Specifies the flag to clear.
  *         This parameter can be one of the following values:
  *            @arg @ref CALENDAR_FLAG_ALARM Alarm Interrupt event
  *            @arg @ref CALENDAR_FLAG_WARP  Warp Interrupt event
  *            @arg @ref CALENDAR_FLAG_TICK Tick Interrupt event
  * @retval None
  */
#define __HAL_CALENDAR_CLEAR_FLAG(__FLAG__)                 WRITE_REG(CALENDAR->INT_STAT, (__FLAG__))

/** @brief  Get the CALENDAR busy flag.
  * @retval The new state of __BUSY__ (TRUE or FALSE).
  */
#define __HAL_CALENDAR_BUSY_FLAG()                          ((READ_BITS(CALENDAR->STAT, RTC_STAT_BUSY) == RTC_STAT_BUSY))

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup CALENDAR_Private_Macro CALENDAR Private Macros
  * @{
  */

/** @brief  Check if CALENDAR Alarm Type is valid.
  * @param  __TYPE__    CALENDAR Alarm Type.
  * @retval SET (__TYPE__ is valid) or RESET (__TYPE__ is invalid)
  */
#define IS_CALENDAR_ALARM_TYPE(__TYPE__)        (((__TYPE__) == CALENDAR_ALARM_SEL_DATE) || \
                                                 ((__TYPE__) == CALENDAR_ALARM_SEL_WEEKDAY))

/** @brief  Check if CALENDAR Date is valid.
  * @param  __DATE__    CALENDAR Date.
  * @retval SET (__DATE__ is valid) or RESET (__DATE__ is invalid)
  */
#define IS_CALENDAR_DATE(__DATE__)              (((__DATE__) >  0) && ((__DATE__) <= 31))

/** @brief  Check if CALENDAR Weekday is valid.
  * @param  __WEEKDAY__    CALENDAR Weekday.
  * @retval SET (__WEEKDAY__ is valid) or RESET (__WEEKDAY__ is invalid)
  */
#define IS_CALENDAR_WEEKDAY(__WEEKDAY__)        (((__WEEKDAY__) >= 0) && ((__WEEKDAY__) <= 6))

/** @brief  Check if CALENDAR year is leap year.
  * @param  __YEAR__    CALENDAR Year.
  * @retval SET (__YEAR__ is leap year) or RESET (__YEAR__ is nonleap year)
  */
#define IS_CALENDAR_LEAP_YEAR(__YEAR__)         ((((__YEAR__) % 4) == 0 && ((__YEAR__) % 100) != 0) || \
                                                 ((__YEAR__) % 400) == 0)

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_CALENDAR_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup CALENDAR_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions.
 *
@verbatim
  ==============================================================================
          ##### Initialization and Configuration functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
      (+) Initialize and start the CALENDAR according to the specified parameters
          in the cslendar_init_t of associated handle.
      (+) Initialize the CALENDAR MSP.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the CALENDAR according to the specified parameters in the
 *         calendar_init_t of  associated handle.
 *
 * @param[in]  p_calendar: Pointer to a CALENDAR handle which contains the configuration
 *               information for the specified CALENDAR module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_calendar_init(calendar_handle_t *p_calendar);

/**
 ****************************************************************************************
 * @brief De-initialize the CALENDAR peripheral.
 *
 * @param[in] p_calendar: CALENDAR handle.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_calendar_deinit(calendar_handle_t *p_calendar);

/** @} */

/** @addtogroup CALENDAR_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions
 *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Init the CALENDAR time.
    (+) Get the CALENDAR time.
    (+) Set the CALENDAR alarm.
    (+) Disable the CALENDAR alarm.
    (+) Handle CALENDAR interrupt request and associated function callback.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the CALENDAR time.
 *
 * @param[in]  p_calendar: Pointer to a CALENDAR handle which contains the configuration
 *               information for the specified CALENDAR module.
 * @param[in]  p_time: Pointer to a CALENDAR time struction.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_calendar_init_time(calendar_handle_t *p_calendar, calendar_time_t *p_time);

/**
 ****************************************************************************************
 * @brief  Get current CALENDAR time.
 *
 * @param[in]  p_calendar: Pointer to a CALENDAR handle which contains the configuration
 *               information for the specified CALENDAR module.
 * @param[in]  p_time: Pointer to a CALENDAR time struction.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_calendar_get_time(calendar_handle_t *p_calendar, calendar_time_t *p_time);

/**
 ****************************************************************************************
 * @brief  Set a CALENDAR date alarm.
 *
 * @param[in]  p_calendar: Pointer to a CALENDAR handle which contains the configuration
 *               information for the specified CALENDAR module.
 * @param[in]  p_alarm: After seconds will generate an date alarm interrupt.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_calendar_set_alarm(calendar_handle_t *p_calendar, calendar_alarm_t *p_alarm);

/**
 ****************************************************************************************
 * @brief  Set a CALENDAR tick 0 alarm.
 *
 * @param[in]  p_calendar: Pointer to a CALENDAR handle which contains the configuration
 *               information for the specified CALENDAR module.
 * @param[in]  interval: After milliseconds will generate an milliseconds alarm interrupt.
 *              The value of interval is greater than or equal to 0ms.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_calendar_set_tick(calendar_handle_t *p_calendar, uint32_t interval);

/**
 ****************************************************************************************
 * @brief  Disable CALENDAR alarm event.
 *
 * @param[in]  p_calendar: Pointer to a CALENDAR handle which contains the configuration
 *               information for the specified CALENDAR module.
 * @param[in]  disable_mode: Disable specified CALENDAR alarm mode.
 *              This parameter can be the following values:
 *              @arg @ref CALENDAR_ALARM_DISABLE_DATE
 *              @arg @ref CALENDAR_ALARM_DISABLE_TICK
 *              @arg @ref CALENDAR_ALARM_DISABLE_ALL
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_calendar_disable_event(calendar_handle_t *p_calendar, uint32_t disable_mode);

/**
 ****************************************************************************************
 * @brief  Sync slow clock to calendar.
 *
 * @param[in]  p_calendar: Pointer to a CALENDAR handle which contains the configuration
 *               information for the specified CALENDAR module.
 * @param[in]  SlowClockFreq: Number of slow clocks after calibration.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_calendar_sync_time(calendar_handle_t *p_calendar, float SlowClockFreq);
/** @} */


/** @addtogroup CALENDAR_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle CALENDAR interrupt request.
 *
 * @note   When alarm is enabled, CALENDAR will generate an interrupt on conter match alarm value.
 *
 * @param[in]  p_calendar: Pointer to a CALENDAR handle which contains the configuration
 *               information for the specified CALENDAR module.
 ****************************************************************************************
 */
void hal_calendar_irq_handler(calendar_handle_t *p_calendar);

/**
 ****************************************************************************************
 * @brief  CALENDAR date count complete (counter reaches to alarm) callback.
 *
 * @note   This function should not be modified. when the callback is needed,
 *         the hal_calendar_alarm_callback can be implemented in the user file.
 *
 * @param[in]  p_calendar: Pointer to a CALENDAR handle which contains the configuration
 *               information for the specified CALENDAR module.
 ****************************************************************************************
 */
void hal_calendar_alarm_callback(calendar_handle_t *p_calendar);

/**
 ****************************************************************************************
 * @brief  CALENDAR milliseconds count complete (counter reaches to 0) callback.
 *
 * @note   This function should not be modified. when the callback is needed,
 *         the hal_calendar_tick_callback can be implemented in the user file.
 *
 * @param[in]  p_calendar: Pointer to a CALENDAR handle which contains the configuration
 *               information for the specified CALENDAR module.
 ****************************************************************************************
 */
void hal_calendar_tick_callback(calendar_handle_t *p_calendar);

/**
 ****************************************************************************************
 * @brief  CALENDAR overflow callback.
 *
 * @note   This function should not be modified. when the callback is needed,
 *         the hal_calendar_overflow_callback can be implemented in the user file.
 *
 * @note   The overflow time is about 36 hours.
 *
 * @param[in]  p_calendar: Pointer to a CALENDAR handle which contains the configuration
 *               information for the specified CALENDAR module.
 ****************************************************************************************
 */
void hal_calendar_overflow_callback(calendar_handle_t *p_calendar);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_CALENDAR_H__ */

/** @} */

/** @} */

/** @} */

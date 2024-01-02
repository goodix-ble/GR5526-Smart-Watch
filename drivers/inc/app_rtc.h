/**
 ****************************************************************************************
 *
 * @file    app_rtc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of RTC app library.
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

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_RTC RTC
  * @brief RTC APP module driver.
  * @{
  */


#ifndef _APP_RTC_H_
#define _APP_RTC_H_

#include "grx_hal.h"
#include "app_drv_error.h"
#include "app_drv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_CALENDAR_MODULE_ENABLED

/** @addtogroup APP_RTC_DEFINE Defines
  * @{
  */

/** @defgroup App rtc time disable macro definition
  * @{
  */
#define APP_RTC_ALARM_DISABLE_DATE          CALENDAR_ALARM_DISABLE_DATE /**< Disable rtc date alarm */
#define APP_RTC_ALARM_DISABLE_TICK          CALENDAR_ALARM_DISABLE_TICK /**< Disable rtc tick alarm */
#define APP_RTC_ALARM_DISABLE_ALL           CALENDAR_ALARM_DISABLE_ALL  /**< Disable rtc all alarm */
/** @} */

/** @} */

/** @addtogroup APP_RTC_ENUM Enumerations
  * @{
  */
/**
  * @brief RTC event Enumerations definition
  */
typedef enum
{
    APP_RTC_EVT_DATE_ALARM,                 /**< Date alarm event. */
    APP_RTC_EVT_TICK_ALARM,                 /**< Tick alarm event. */
    APP_RTC_EVT_OVERFLOW,                   /**< Overflow event. */
} app_rtc_evt_type_t;
/** @} */

/** @addtogroup APP_RTC_STRUCTURES Structures
  * @{
  */
/**
  * @brief   App time structure definition
  */
typedef calendar_time_t         app_rtc_time_t;

/**
  * @brief   App alarm structure definition
  */
typedef calendar_alarm_t        app_rtc_alarm_t;


/**
  * @brief RTC event structure definition
  */
typedef struct
{
    app_rtc_evt_type_t  type; /**< Type of event. */
} app_rtc_evt_t;

/** @} */

/** @addtogroup APP_RTC_TYPEDEFS Type definitions
  * @{
  */
/**
  * @brief RTC event callback definition
  */
typedef void (*app_rtc_evt_handler_t)(app_rtc_evt_t *p_evt);

/** @} */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_RTC_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP RTC DRIVER.
 *
 * @param[in]  evt_handler: RTC user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_rtc_init(app_rtc_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief De-initialize the app rtc.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_rtc_deinit(void);

/**
 ****************************************************************************************
 * @brief  Initialize the rtc time.
 *
 * @param[in]  p_time: Pointer to a app_rtc_time_t time struct.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_rtc_init_time(app_rtc_time_t *p_time);

/**
 ****************************************************************************************
 * @brief  Get current rtc time.
 *
 * @param[in]  p_time: Pointer to a app_rtc_time_t time struct.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_rtc_get_time(app_rtc_time_t *p_time);

/**
 ****************************************************************************************
 * @brief  Set a rtc date alarm.
 *
 * @param[in]  p_alarm: After seconds will generate an date alarm interrupt.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_rtc_setup_alarm(app_rtc_alarm_t *p_alarm);

/**
 ****************************************************************************************
 * @brief  Set a rtc tick alarm.
 *
 * @param[in]  interval: After milliseconds will generate an tick alarm interrupt.
 *              The value of interval is greater than or equal to 10ms.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_rtc_setup_tick(uint32_t interval);

/**
 ****************************************************************************************
 * @brief  Disable rtc alarm event.
 *
 * @param[in]  disable_mode: Disable specified CALENDAR alarm mode.
 *              This parameter can be the following values:
 *              @arg @ref APP_RTC_ALARM_DISABLE_DATE
 *              @arg @ref APP_RTC_ALARM_DISABLE_TICK
 *              @arg @ref APP_RTC_ALARM_DISABLE_ALL
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_rtc_disable_event(uint32_t disable_mode);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
/**
 ****************************************************************************************
 * @brief  Synchronous RTC low speed clock.
 *
 * @param[in]  SlowClockFreq: Number of slow clocks
 *
 * @return None.
 ****************************************************************************************
 */
void app_rtc_time_sync(float SlowClockFreq);
#endif
/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */



/**
 ****************************************************************************************
 *
 * @file    app_alarm.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of APP Alarm APIs library.
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
 
#ifndef __APP_ALARM__
#define __APP_ALARM__

#include "grx_sys.h"
#include "app_drv_error.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_CALENDAR_MODULE_ENABLED

/** @addtogroup APP_ALARM_DEFINES Macro definition
 * @{
  */

/**@brief APP ALARM tag mask for user application.
 */
#define APP_ALARM_BASE          0x8000

/**@brief Get App Alarm tag for user application.
 */
#define APP_ALARM_ID(idx)       (APP_ALARM_BASE | ((idx) & 0x7FFF))

/**@brief The maximum number of alarms.
 */
#define MAX_ALARM_SUPPORT       8

/**@brief App Alarm base year.
 */
#define APP_ALARM_BASE_YEAR     (2000)

/** @} */

/** @defgroup  APP_ALARM Time
  * @{
  */

/**
  * @brief   App time structure definition
  */
typedef calendar_time_t         app_time_t;

/**
  * @brief   App alarm structure definition
  */
typedef calendar_alarm_t        app_alarm_t;

/**
  * @brief   App alarm id definition
  */
typedef uint16_t                app_alarm_id_t;

/** @} */

/** @addtogroup APP_ALARM_STRUCTURES Event definition
  * @{
  */

/**
  *@brief   The alarm node trigger function. 
  */
typedef void (*app_alarm_fun_t)(app_alarm_id_t app_alarm_id);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_ALARM_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP ALARM according to the specified parameters in the 
 *         NvdsTag_t where the alram data is stored.
 *
 * @param[in] tag: Location where the alram data is stored.
 * @param[in] callback: Pointer to alarm expire callback function
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_alarm_init(NvdsTag_t tag, app_alarm_fun_t callback);

/**
 ****************************************************************************************
 * @brief De-initialize the app alarm.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_alarm_deinit(void);

/**
 ****************************************************************************************
 * @brief  Set a App alarm.
 *
 * @note   if your call the app_alarm_set_time function, 
 *              you should call this function after 1s.
 *
 * @param[in]  alarm: After seconds will generate an alarm interrupt.
 * @param[in]  alarm_id: the id of alarm node.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_alarm_add(const calendar_alarm_t *alarm, app_alarm_id_t alarm_id);

/**
 ****************************************************************************************
 * @brief  Delete a App alarm from list.
 *
 * @param[in]  alarm_id: the id of alarm node.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_alarm_del(app_alarm_id_t alarm_id);

/**
 ****************************************************************************************
 * @brief  Get current App time.
 *
 * @param[in]  p_time: Pointer to a app time struction.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_alarm_get_time(calendar_time_t *p_time);

/**
 ****************************************************************************************
 * @brief  Initialize the app time.
 *
 * @param[in]  p_time: Pointer to a app time struction.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_alarm_set_time(calendar_time_t *p_time);

/**
 ****************************************************************************************
 * @brief  Time synchronization function.
 *
 * @note   If you reset system time, you should call this function again after 1s
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_alarm_reload(void);

/**
 ****************************************************************************************
 * @brief  Delete all App alarm from list.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_alarm_del_all(void);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif  /* __APP_ALARM__ */

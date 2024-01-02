/**
 ****************************************************************************************
 *
 * @file ans.h
 *
 * @brief Alert Notification Service API.
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
 *****************************************************************************************
 */
/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_ANS Alert Notification Service (ANS)
 * @{
 * @brief Alert Notification Service module.
 *
 * @details The Alert Notification Service exposes alert information in a device. This information
 *          includes the following: Type of alert occurring in a device, Additional text information
 *          such as caller ID or sender ID, Count of new alerts and Count of unread alert items.
 *
 *          After \ref ans_init_t variable is intialized, the application must call \ref ans_service_init()
 *          to add Alert Notification Service and Supported New Alert Category, New Alert, Supported
 *          Unread Alert Category, Unread Alert Status and Alert Notification Control Point characteristics
 *          to the BLE Stack database.
 */

#ifndef __ANS_H__
#define __ANS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup ANS_MACRO Defines
 * @{
 */
#define ANS_CONNECTION_MAX                  10                            /**< Maximum number of Alert Notification Service connections. */
#define ANS_ERROR_CMD_NOT_SUP               0xa0                          /**< Command not supported. */
#define ANS_UTF_8_STR_LEN_MAX               18                            /**< Maximum length of “UTF-8 string”. */
#define ANS_SUP_NEW_ALERT_CAT_VAL_LEN       2                             /**< Length of Supported New Alert Category value. */
#define ANS_NEWS_ALERT_VAL_LEN              (ANS_UTF_8_STR_LEN_MAX + 2)   /**< Length of New Alert value. */
#define ANS_SUP_UNREAD_ALERT_CAT_VAL_LEN    2                             /**< Length of Supported Unread Alert Category value. */
#define ANS_UNREAD_ALERT_STA_VAL_LEN        2                             /**< Length of Unread Alert Status value. */
#define ANS_ALERT_NTF_CTRL_PT_VAL_LEN       2                             /**< Length of Alert Notification Control Point value. */

/**
 * @defgroup ANS_CAT_ID_BIT_MASK Category ID Bit Masks
 * @{
 * @brief Category ID Bit Masks.
 */
#define ANS_SMPL_ALERT_SUP              (0x01 << 0)                  /**< Bit for Simple Alert Supported. */
#define ANS_EMAIL_SUP                   (0x01 << 1)                  /**< Bit for Email Supported. */
#define ANS_NEWS_SUP                    (0x01 << 2)                  /**< Bit for News Supported. */
#define ANS_CALL_SUP                    (0x01 << 3)                  /**< Bit for Call Supported. */
#define ANS_MISSED_CALL_SUP             (0x01 << 4)                  /**< Bit for Missed Call Supported. */
#define ANS_SMS_MMS_SUP                 (0x01 << 5)                  /**< Bit for SMS/MMS Supported. */
#define ANS_VOICE_MAIL_SUP              (0x01 << 6)                  /**< Bit for Voice Mail Supported. */
#define ANS_SCHEDULE_SUP                (0x01 << 7)                  /**< Bit for Schedule Supported. */
#define ANS_HG_PRIO_ALERT_SUP           (0x01 << 8)                  /**< Bit for High Prioritized Alert Supported. */
#define ANS_INSTANT_MES                 (0x01 << 9)                  /**< Bit for Instant Message Supported. */
#define ANS_ALL_CAT_SUP                 (0x03ff)                     /**< Bit for All Category Supported. */
/** @} */
/** @} */

/**
 * @defgroup ANS_ENUM Enumerations
 * @{
 */
/**@brief Alert Notification Service Categories of alerts/messages. */
typedef enum
{
    ANS_CAT_ID_SMPL_ALERT,            /**< Simple Alert: General text alert or non-text alert. */
    ANS_CAT_ID_EMAIL,                 /**< Email: Alert when Email messages arrive. */
    ANS_CAT_ID_NEWS,                  /**< News: News feeds such as RSS, Atom. */
    ANS_CAT_ID_CALL,                  /**< Call: Incoming call. */
    ANS_CAT_ID_MISSED_CALL,           /**< Missed call: Missed Call. */
    ANS_CAT_ID_SMS_MMS,               /**< SMS/MMS: SMS/MMS message arrives. */
    ANS_CAT_ID_VOICE_MAIL,            /**< Voice mail: Voice mail.*/
    ANS_CAT_ID_SCHEDULE,              /**< Schedule: Alert occurred on calendar, planner. */
    ANS_CAT_ID_HG_PRIO_ALERT,         /**< High Prioritized Alert: Alert that should be handled as high priority. */
    ANS_CAT_ID_INSTANT_MES,           /**< Instant Message: Alert for incoming instant messages. */
    ANS_CAT_ID_NB,                    /**< Number of all Categories of alerts/messages. */
    ANS_CAT_ID_ALL = 0xff,            /**< All Categories of alerts/messages. */
} ans_alert_cat_id_t;

/**@brief Alert Notification Service Control point. */
typedef enum
{
    ANS_CTRL_PT_EN_NEW_INC_ALERT_NTF,       /**< Enable New Incoming Alert Notification. */
    ANS_CTRL_PT_EN_UNREAD_CAT_STA_NTF,      /**< Enable Unread Category Status Notification. */
    ANS_CTRL_PT_DIS_NEW_INC_ALERT_NTF,      /**< Disable New Incoming Alert Notification. */
    ANS_CTRL_PT_DIS_UNREAD_CAT_STA_NTF,     /**< Disable Unread Category Status Notification. */
    ANS_CTRL_PT_NTF_NEW_INC_ALERT_IMME,     /**< Notify New Incoming Alert immediately. */
    ANS_CTRL_PT_NTF_UNREAD_CAT_STA_IMME,    /**< Notify Unread Category Status immediately. */
} ans_ctrl_pt_id_t;

/**@brief Alert Notification Service Event type. */
typedef enum
{
    ANS_EVT_INVALID,                        /**< Invalid ANS event type. */
    ANS_EVT_NEW_ALERT_NTF_ENABLE,           /**< NEW Alert notification is enabled. */
    ANS_EVT_NEW_ALERT_NTF_DISABLE,          /**< NEW Alert notification is disabled. */
    ANS_EVT_UNREAD_ALERT_STA_NTF_ENABLE,    /**< Unread Alert Status notification is enabled. */
    ANS_EVT_UNREAD_ALERT_STA_NTF_DISABLE,   /**< Unread Alert Status notification is disabled. */
    ANS_EVT_NEW_ALERT_IMME_NTF_REQ,         /**< Request: notify the New Alert characteristic to the client immediately. */
    ANS_EVT_Unread_ALERT_IMME_NTF_REQ,      /**< Request: notify the Unread Alert Status characteristic to the client immediately. */
} ans_evt_type_t;
/** @} */

/**
 * @defgroup ANS_STRUCT Structures
 * @{
 */
/**@brief Alert Notification Service New Alert value. */
typedef struct
{
    ans_alert_cat_id_t  cat_id;                            /**< Category ID. */
    uint8_t             alert_num;                         /**< Number of new alert. */
    uint8_t             str_info[ANS_UTF_8_STR_LEN_MAX];   /**< Text String Information. */
    uint8_t             length;                            /**< Length of Text String. */
} ans_new_alert_t;

/**@brief Alert Notification Service Unread Alert Status value. */
typedef struct
{
    ans_alert_cat_id_t  cat_id;         /**< Category ID. */
    uint8_t             unread_num;     /**< Number of unread alert. */
} ans_unread_alert_t;

/**@brief Alert Notification Service Control Point value. */
typedef struct
{
    ans_ctrl_pt_id_t    cmd_id;     /**< Command ID. */
    ans_alert_cat_id_t  cat_id;     /**< Category ID. */
} ans_ctrl_pt_t;

/**@brief Alert Notification Service event. */
typedef struct
{
    uint8_t          conn_idx;        /**< The index of the connection. */
    ans_evt_type_t   evt_type;        /**< The ANS event type. */
    uint16_t         cat_ids;         /**< Category IDs. */
} ans_evt_t;
/** @} */

/**
 * @defgroup ANS_TYPEDEF Typedefs
 * @{
 */
/**@brief Alert Notification Service event handler type.*/
typedef void (*ans_evt_handler_t)(ans_evt_t *p_evt);
/** @} */

/**
 * @defgroup ANS_STRUCT Structures
 * @{
 */
/**@brief Alert Notification Service init stucture. This contains all options and data needed for initialization of the service. */
typedef struct
{
    ans_evt_handler_t   evt_handler;             /**< Phone Alert Status Service event handler. */
    uint16_t            sup_new_alert_cat;       /**< Initial mask of Supported New Alert Category. */
    uint16_t            sup_unread_alert_sta;    /**< Initial mask of Unread Alert Status. */
} ans_init_t;
/** @} */

/**
 * @defgroup ANS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize an Alert Notification Service instance and add to the DB.
 *
 * @param[in] p_ans_init: Pointer to ANS Service initialization variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t ans_service_init(ans_init_t *p_ans_init);

/**
 *****************************************************************************************
 * @brief Send New Alert if its notification has been enabled.
 *
 * @param[in] conn_idx:    Connnection index.
 * @param[in] p_new_alert: Pointer to New Alert information.
 *
 * @return Result of notify value
 *****************************************************************************************
 */
sdk_err_t ans_new_alert_send(uint8_t conn_idx, ans_new_alert_t *p_new_alert);

/**
 *****************************************************************************************
 * @brief Send Unread Alert Status if its notification has been enabled.
 *
 * @param[in] conn_idx:       Connnection index.
 * @param[in] p_unread_alert: Pointer to Unread Alert Status information.
 *
 * @return Result of notify value
 *****************************************************************************************
 */
sdk_err_t ans_unread_alert_send(uint8_t conn_idx, ans_unread_alert_t *p_unread_alert);
/** @} */

#endif
/** @} */
/** @} */


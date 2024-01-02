/**
 ****************************************************************************************
 *
 * @file ans_c.h
 *
 * @brief Alert Notification Service Client API.
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
 */

/**
 * @defgroup BLE_SDK_ANS_C Alert Notification Service Client (ANS_C)
 * @{
 * @brief Alert Notification Service Client module.
 *
 * @details The Alert Notification Service Client contains the APIs and types, which can be used
 *          by the application to discovery of Alert Notification Service of peer and interact with it.
 *
 *          The application must provide an event handler to be register, then call \ref ans_client_init().
 *          After Alert Notification Service Client discoveries peer Alert Notification Service,
 *          application can call \ref ans_c_ctrl_point_set() to send Alert Notification Control Point to peer.
 *          When number of New Alert or Unread Alert changes, the module will receive notification from peer
 *          if notifications of them are enabled.
 */

#ifndef __ANS_C_H__
#define __ANS_C_H__

#include "gr_includes.h"
#include "ble_prf_types.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup ANS_C_MACRO Defines
 * @{
 */
#define ANS_C_CONNECTION_MAX                  10                            /**< Maximum number of HRS Client connections. */
#define ANS_C_ERROR_CMD_NOT_SUP               0xa0                          /**< Command not supported. */
#define ANS_C_UTF_8_STR_LEN_MAX               18                            /**< Maximum length of “UTF-8 string”. */
#define ANS_C_ALERT_NTF_CTRL_PT_VAL_LEN       2                             /**< Length of Alert Notification Control Point value. */

/**
 * @defgroup ANS_C_CAT_ID_BIT_MASK Category ID Bit Masks
 * @{
 * @brief Category ID Bit Masks.
 */
#define ANS_C_SMPL_ALERT_SUP              (0x01 << 0)                  /**< Bit for Simple Alert Supported. */
#define ANS_C_EMAIL_SUP                   (0x01 << 1)                  /**< Bit for Email Supported. */
#define ANS_C_NEWS_SUP                    (0x01 << 2)                  /**< Bit for News Supported. */
#define ANS_C_CALL_SUP                    (0x01 << 3)                  /**< Bit for Call Supported. */
#define ANS_C_MISSED_CALL_SUP             (0x01 << 4)                  /**< Bit for Missed Call Supported. */
#define ANS_C_SMS_MMS_SUP                 (0x01 << 5)                  /**< Bit for SMS/MMS Supported. */
#define ANS_C_VOICE_MAIL_SUP              (0x01 << 6)                  /**< Bit for Voice Mail Supported. */
#define ANS_C_SCHEDULE_SUP                (0x01 << 7)                  /**< Bit for Schedule Supported. */
#define ANS_C_HG_PRIO_ALERT_SUP           (0x01 << 8)                  /**< Bit for High Prioritized Alert Supported. */
#define ANS_C_INSTANT_MES                 (0x01 << 9)                  /**< Bit for Instant Message Supported. */
#define ANS_C_ALL_CAT_SUP                 (0x03ff)                     /**< Bit for All Category Supported. */
/** @} */
/** @} */

/**
 * @defgroup ANS_C_ENUM Enumerations
 * @{
 */
/**@brief Alert Notification Service Categories of alerts/messages. */
typedef enum
{
    ANS_C_CAT_ID_SMPL_ALERT,            /**< Simple Alert: General text alert or non-text alert. */
    ANS_C_CAT_ID_EMAIL,                 /**< Email: Alert when Email messages arrives. */
    ANS_C_CAT_ID_NEWS,                  /**< News: News feeds such as RSS, Atom. */
    ANS_C_CAT_ID_CALL,                  /**< Call: Incoming call. */
    ANS_C_CAT_ID_MISSED_CALL,           /**< Missed call: Missed Call. */
    ANS_C_CAT_ID_SMS_MMS,               /**< SMS/MMS: SMS/MMS message arrives. */
    ANS_C_CAT_ID_VOICE_MAIL,            /**< Voice mail: Voice mail.*/
    ANS_C_CAT_ID_SCHEDULE,              /**< Schedule: Alert occurred on calendar, planner. */
    ANS_C_CAT_ID_HG_PRIO_ALERT,         /**< High Prioritized Alert: Alert that should be handled as high priority. */
    ANS_C_CAT_ID_INSTANT_MES,           /**< Instant Message: Alert for incoming instant messages. */
    AANS_C_CAT_ID_NB,                    /**< Number of all Categories of alerts/messages. */
    ANS_C_CAT_ID_ALL = 0xff,            /**< All Categories of alerts/messages. */
} ans_c_alert_cat_id_t;

/**@brief Alert Notification Service Client Control point ID. */
typedef enum
{
    ANS_C_CTRL_PT_EN_NEW_INC_ALERT_NTF,       /**< Enable New Incoming Alert Notification. */
    ANS_C_CTRL_PT_EN_UNREAD_CAT_STA_NTF,      /**< Enable Unread Category Status Notification. */
    ANS_C_CTRL_PT_DIS_NEW_INC_ALERT_NTF,      /**< Disable New Incoming Alert Notification. */
    ANS_C_CTRL_PT_DIS_UNREAD_CAT_STA_NTF,     /**< Disable Unread Category Status Notification. */
    ANS_C_CTRL_PT_NTF_NEW_INC_ALERT_IMME,     /**< Notify New Incoming Alert immediately. */
    ANS_C_CTRL_PT_NTF_UNREAD_CAT_STA_IMME,    /**< Notify Unread Category Status immediately. */
} ans_c_ctrl_pt_id_t;

/**@brief Alert Notification Service Client Event type. */
typedef enum
{
    ANS_C_EVT_INVALID,                              /**< ANS Client invalid event type. */
    ANS_C_EVT_DISCOVERY_COMPLETE,                   /**< ANS Client has found ANS service and its characteristics. */
    ANS_C_EVT_DISCOVERY_FAIL  ,                     /**< ANS Client found ANS service failed because of invalid operation or no found at the peer. */
    ANS_C_EVT_NEW_ALERT_NTF_SET_SUCCESS,            /**< ANS Client has set NEW Alert notification. */
    ANS_C_EVT_UNREAD_ALERT_STA_NTF_SET_SUCCESS,     /**< ANS Client has set Unread Alert Status notification. */
    ANS_C_EVT_SUP_NEW_ALERT_CAT_RECEIV,             /**< ANS Client has received Supported New Alert Category value (Read from peer). */
    ANS_C_EVT_SUP_UNREAD_ALERT_CAT_REC,             /**< ANS Client has received Supported Unread Alert Category value (Read from peer). */
    ANS_C_EVT_NEW_ALERT_RECEIVE,                    /**< ANS Client has received New Alert value (Notification from peer). */
    ANS_C_EVT_UNREAD_ALERT_RECEIVE,                 /**< ANS Client has received Unread Alert Status value (Notification from peer). */
    ANS_C_EVT_CTRL_POINT_SET_SUCCESS,               /**< ANS Client has written Control Point completely. */
    ANS_C_EVT_WRITE_OP_ERR,                         /**< Error occured when ANS Client wrote to peer. */
} ans_c_evt_type_t;
/** @} */

/**
 * @defgroup ANS_C_STRUCT Structures
 * @{
 */
/**@brief Alert Notification Service Client decoded New Alert value. */
typedef struct
{
    ans_c_alert_cat_id_t  cat_id;                              /**< Category ID. */
    uint8_t               alert_num;                           /**< Number of new alert. */
    uint8_t               str_info[ANS_C_UTF_8_STR_LEN_MAX];   /**< Text String Information. */
    uint8_t               length;                              /**< Length of Text String. */
} ans_c_new_alert_t;

/**@brief Alert Notification Service Client decoded Unread Alert Status value. */
typedef struct
{
    ans_c_alert_cat_id_t  cat_id;         /**< Category ID. */
    uint8_t               unread_num;     /**< Number of unread alert. */
} ans_c_unread_alert_t;

/**@brief Alert Notification Service Client Control Point value. */
typedef struct
{
    ans_c_ctrl_pt_id_t    cmd_id;     /**< Command ID. */
    ans_c_alert_cat_id_t  cat_id;     /**< Category ID. */
} ans_c_ctrl_pt_t;

/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t ans_srvc_start_handle;            /**< ANS Service start handle. */
    uint16_t ans_srvc_end_handle;              /**< ANS Service end handle. */
    uint16_t ans_sup_new_alert_cat_handle;     /**< ANS Supported New Alert Category characteristic Value handle which has been got from peer. */
    uint16_t ans_new_alert_handle;             /**< ANS New Alert characteristic Value handle which has been got from peer. */
    uint16_t ans_new_alert_cccd_handle;        /**< ANS CCCD handle of New Alert characteristic which has been got from peer. */
    uint16_t ans_sup_unread_alert_cat_handle;  /**< ANS Supported Unread Alert Category characteristic Value handle which has been got from peer. */
    uint16_t ans_unread_alert_handle;          /**< ANS Unread Alert characteristic Value handle which has been got from peer. */
    uint16_t ans_unread_alert_cccd_handle;     /**< ANS CCCD handle of Unread Alert characteristic which has been got from peer. */
    uint16_t ans_ctrl_pt_handle;               /**< ANS Control Point characteristic Value handle which has been got from peer. */
} ans_c_handles_t;

/**@brief Alert Notification Service Client event. */
typedef struct
{
    uint8_t          conn_idx;                          /**< The index of the connection. */
    ans_c_evt_type_t evt_type;                          /**< The ANS event type. */
    union
    {
        uint16_t             sup_new_alert_cat_ids;     /**< Alert status received. */
        uint16_t             sup_unread_alert_cat_ids;  /**< Ringer setting received. */
        ans_c_new_alert_t    new_alert;                 /**< New Alert value. */
        ans_c_unread_alert_t unread_alert;              /**< Unread Alert Status value. */
    } value;                                            /**< Value received. */
} ans_c_evt_t;
/** @} */

/**
 * @defgroup ANS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief Alert Notification Service Client event handler type.*/
typedef void (*ans_c_evt_handler_t)(ans_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup ANS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register ANS Client event handler.
 *
 * @param[in] evt_handler: Alert Notification Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t ans_client_init(ans_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discover Alert Notification Service on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ans_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer New Alert characteristic notify.
 *
 * @param[in] conn_idx:  Index of connection.
 * @param[in] is_enable: True or false.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ans_c_new_alert_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Enable or disable peer Unread Alert characteristic notify.
 *
 * @param[in] conn_idx:  Index of connection.
 * @param[in] is_enable: True or false.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ans_c_unread_alert_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Read Supported New Alert Category characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ans_c_sup_new_alert_cat_read(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Read Supported Unread Alert Category characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ans_c_sup_unread_alert_cat_read(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Set Control Point characteristic value.
 *
 * @param[in] conn_idx:   Index of connection.
 * @param[in] p_ctrl_pt:  Pointer to control point.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ans_c_ctrl_point_set(uint8_t conn_idx, ans_c_ctrl_pt_t *p_ctrl_pt);
/** @} */

#endif
/** @} */
/** @} */


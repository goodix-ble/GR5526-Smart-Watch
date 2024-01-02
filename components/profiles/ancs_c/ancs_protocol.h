/**
 *******************************************************************************
 *
 * @file ancs_protocol.h
 *
 * @brief Apple Notification Center Service Protocol API.
 *
 *******************************************************************************
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
#ifndef _ANCS_PROTOCOL_H_
#define _ANCS_PROTOCOL_H_

#include "gr_includes.h"
#include <stdint.h>

/** Maximum allowed value for attribute length */
#ifndef CFG_ANCS_ATTRIBUTE_MAXLEN
#define CFG_ANCS_ATTRIBUTE_MAXLEN 500
#endif

/** Attribute ID element without maximum length */
#define ANCS_ATTR(ID) ((uint32_t) 0x80000000 | ((uint8_t) ID))

/** Attribute ID element with maximum length */
#define ANCS_ATTR_MAXLEN(ID, LEN) ((uint32_t) 0x80000000 | ((uint8_t) ID) | ((uint16_t) LEN << 8))

/**
 * @defgroup ANCS_ENUM Enumerations
 * @{
 */

/**@brief IDs for iOS notification attributes. */
typedef enum 
{
    ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER = 0,     /**< Identify that the attribute data is of an "App Identifier" type. */
    ANCS_NOTIF_ATTR_ID_TITLE,                  /**< Identify that the attribute data is a "Title". */
    ANCS_NOTIF_ATTR_ID_SUBTITLE,               /**< Identify that the attribute data is a "Subtitle". */
    ANCS_NOTIF_ATTR_ID_MESSAGE,                /**< Identify that the attribute data is a "Message". */
    ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE,           /**< Identify that the attribute data is a "Message Size". */
    ANCS_NOTIF_ATTR_ID_DATE,                   /**< Identify that the attribute data is a "Date". */
    ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL,  /**< The notification has a "Positive action" that can be executed associated with it. */
    ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL,  /**< The notification has a "Negative action" that can be executed associated with it. */
} ancs_notification_attr_t;

/**@brief Category IDs for iOS notifications. */
typedef enum
{
    ANCS_CATEGORY_ID_OTHER,                   /**< The iOS notification belongs to the "other" category.  */
    ANCS_CATEGORY_ID_INCOMING_CALL,           /**< The iOS notification belongs to the "Incoming Call" category. */
    ANCS_CATEGORY_ID_MISSED_CALL,             /**< The iOS notification belongs to the "Missed Call" category. */
    ANCS_CATEGORY_ID_VOICE_MAIL,              /**< The iOS notification belongs to the "Voice Mail" category. */
    ANCS_CATEGORY_ID_SOCIAL,                  /**< The iOS notification belongs to the "Social" category. */
    ANCS_CATEGORY_ID_SCHEDULE,                /**< The iOS notification belongs to the "Schedule" category. */
    ANCS_CATEGORY_ID_EMAIL,                   /**< The iOS notification belongs to the "E-mail" category. */
    ANCS_CATEGORY_ID_NEWS,                    /**< The iOS notification belongs to the "News" category. */
    ANCS_CATEGORY_ID_HEALTH_AND_FITNESS,      /**< The iOS notification belongs to the "Health and Fitness" category. */
    ANCS_CATEGORY_ID_BUSINESS_AND_FINANCE,    /**< The iOS notification belongs to the "Buisness and Finance" category. */
    ANCS_CATEGORY_ID_LOCATION,                /**< The iOS notification belongs to the "Location" category. */
    ANCS_CATEGORY_ID_ENTERTAINMENT            /**< The iOS notification belongs to the "Entertainment" category. */
} ancs_category_id_t;

/**@brief Event IDs for iOS notifications. */
typedef enum
{
    ANCS_EVENT_ID_NOTIFICATION_ADDED,         /**< The iOS notification was added. */
    ANCS_EVENT_ID_NOTIFICATION_MODIFIED,      /**< The iOS notification was modified. */
    ANCS_EVENT_ID_NOTIFICATION_REMOVED        /**< The iOS notification was removed. */
} ancs_evt_id_t;

/**@brief ID for actions that can be performed for iOS notifications. */
typedef enum
{
    ACTION_ID_POSITIVE = 0,                   /**< Positive action. */
    ACTION_ID_NEGATIVE                        /**< Negative action. */
} ancs_c_action_id_t;

/**@brief ctrl point command that can be performed for iOS notifications. */
typedef enum
{
    CTRL_POINT_GET_NTF_ATTRIBUTE = 0,              /**< Request attributes to be sent from the NP to the NC for a given notification. */
    CTRL_POINT_GET_APP_ATTRIBUTE,                  /**< Request attributes to be sent from the NP to the NC for a given iOS app. */
    CTRL_POINT_PERFORM_NTF_ACTION,            /**< Request an action to be performed on a given notification, for example, dismiss an alarm. */
} ancs_c_ctrl_point_t;

/** @} */

/**
 * @defgroup ANCS_STRUCT Structures
 * @{
 */

/**@brief notification flags that can be performed for iOS notifications. */
typedef struct
{
    uint8_t silent          : 1;               /**< If this flag is set, the notification has a low priority. */
    uint8_t important       : 1;               /**< If this flag is set, the notification has a high priority. */
    uint8_t pre_existing    : 1;               /**< If this flag is set, the notification is pre-existing. */
    uint8_t positive_action : 1;               /**< If this flag is set, the notification has a positive action that can be taken. */
    uint8_t negative_action : 1;               /**< If this flag is set, the notification has a negative action that can be taken. */
} ancs_ntf_flags_t;

/**@brief iOS notification structure. */
typedef struct 
{
    ancs_evt_id_t event_id;                  /**< Whether the notification was added, removed, or modified. */
    ancs_ntf_flags_t event_flags;            /**< Whether the notification was added, removed, or modified. */
    ancs_category_id_t category_id;          /**< Classification of the notification type, for example, email or location. */
    uint8_t category_count;                    /**< Current number of active notifications for this category ID. */
    uint32_t notification_uid;                 /**< Notification UID. */
} ntf_source_pdu_t;

/** @} */

/**
 * @defgroup ANCS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Get notification attribute
 *
 * @param[in] uid: The UID of notify message
 * @param[in] noti_attr: The notification attribute 
 *
 *****************************************************************************************
 */
void ancs_notify_attr_get(int uid, char noti_attr);

/**
 *****************************************************************************************
 * @brief ancs perform action  
 *
 * @param[in] uid: The UID of notify message
 * @param[in] action: The action status defined by specification 
 *
 *****************************************************************************************
 */
void ancs_action_perform(int uid, int action);

/**
 *****************************************************************************************
 * @brief get ancs phone call UID
 *
 * @return phone call notify message UID
 *****************************************************************************************
 */
int ancs_get_uid(void);

/**
 *****************************************************************************************
 * @brief Decode notification source message.
 *
 * @param[in] p_data: Pointer to the parameters of the read request.
 * @param[in] length: The Length of read value
 *****************************************************************************************
 */
void ancs_decode_notification_source(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Decode data source message.
 *
 * @param[in] p_data: Pointer to the parameters of the read request.
 * @param[in] length: Length of read data
 *****************************************************************************************
 */
void ancs_decode_data_source(uint8_t *p_data, uint16_t length);
/** @} */
#endif



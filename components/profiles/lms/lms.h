/**
 ****************************************************************************************
 *
 * @file lms.h
 *
 * @brief Log Management Service API
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
 * @defgroup BLE_SDK_LMS LMS Service (LMS)
 * @{
 * @brief Definitions and prototypes for the LMS interface.
 *
 * @details The Log Management Service is a customized service with Command and Data
 *          characteristics. This module implements the log storage and sending function.
 *          After @ref lms_init_t variable is initialized, the developer shall call 
 *          @ref lms_service_init() to add the LMS Service and RX, TX, Control characteristic
 *          to the BLE stack database.
 *
 *          This module also provides \ref lms_notify_data() function to the application
 *          to send data to peer. 
 */

#ifndef _LMS_H_
#define _LMS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gr_includes.h"

/**
 * @defgroup LMS_MACRO Defines
 * @{
 */
#define LMS_CONNECTION_MAX     10                                                   /**< Maximum number of LMS Service connections. */
#define LMS_MAX_DATA_LEN       244                                                  /**< Maximum length of LMS characteristic. */
#define LMS_SERVICE_UUID       0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                               0x0A, 0x46, 0x44, 0xD3, 0x01, 0x0B, 0xED, 0xA6       /**< The UUID of LMS Service for setting advertising data. */

#define LMS_PATTERN_VALUE      0x474f4f44                                           /**< The Fast OTA pattern value. */
/** @} */

/**
 * @defgroup LMS_ENUM Enumerations
 * @{
 */
/**@brief LMS Service event type. */
typedef enum
{
    LMS_EVT_INVALID,
    LMS_EVT_CMD_NOTIFICATION_ENABLED,
    LMS_EVT_CMD_NOTIFICATION_DISABLED,
    LMS_EVT_CMD_RECEIVE_DATA,
    LMS_EVT_CMD_NOTIFY_COMPLETE,
    LMS_EVT_DATA_NOTIFICATION_ENABLED,
    LMS_EVT_DATA_NOTIFICATION_DISABLED,
    LMS_EVT_DATA_RECEIVE_DATA,
    LMS_EVT_DATA_NOTIFY_COMPLETE
} lms_evt_type_t;
/** @} */

/**
 * @defgroup LMS_STRUCT Structures
 * @{
 */
/**@brief LMS Service event. */
typedef struct
{
    lms_evt_type_t  evt_type;            /**< The LMS event. */
    uint8_t         conn_idx;           /**< Index of connection. */
    uint8_t         *p_data;             /**< Pointer to data. */
    uint16_t        length;             /**< Length of data. */
} lms_evt_t;
/** @} */

/**
 * @defgroup LMS_TYPEDEF Typedefs
 * @{
 */
/**@brief LMS Service event handler type. */
typedef void (*lms_evt_handler_t)(lms_evt_t *p_evt);

/** @} */

/**
 * @defgroup LMS_STRUCT Structures
 * @{
 */
/**@brief LMS Service initialization variable. */
typedef struct
{
    lms_evt_handler_t evt_handler;    /**< Handler to handle lms event. */
} lms_init_t;
/** @} */


/**
 * @defgroup LMS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Add an LMS Service instance in the DB
 *
 * @param[in] p_lms_init :Pointer to LMS Service environment variable
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t lms_service_init(lms_init_t *p_lms_init);


/**
 *****************************************************************************************
 * @brief Send data to peer device
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_data:   The Pointer of send value
 * @param[in] length:   The Lenth of send value
 *
 * @return Result of notify and indicate value 
 *****************************************************************************************
 */
sdk_err_t lms_notify_cmd(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Send data to peer device
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_data:   The Pointer of send value
 * @param[in] length:   The Lenth of send value
 *
 * @return Result of notify and indicate value 
 *****************************************************************************************
 */
sdk_err_t lms_notify_data(uint8_t conn_idx, uint8_t *p_data,uint16_t length);
/** @} */

#endif
/** @} */
/** @} */

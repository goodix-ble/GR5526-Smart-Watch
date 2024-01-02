/**
 ****************************************************************************************
 *
 * @file lns.h
 *
 * @brief Log Notification Service API.
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
 * @defgroup BLE_SDK_LNS Log Notification Service (LNS)
 * @{
 * @brief Log Notification Service module.
 *
 * @details The Log Notification Service shall expose the Log Info characteristic and the Log Control Point characteristic.
 *
 *          The application must call \ref lns_service_init() to add Log Notification Service and Log Info 
 *          and Log Control Point characteristics to the BLE Stack database.
 */

#ifndef __LNS_H__
#define __LNS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include "fault_trace.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup LNS_MACRO Defines
 * @{
 */
#define LNS_CONNECTION_MAX          10                                                 /**< Maximum number of Log Notification Service connections. */
#define LNS_SERVICE_UUID            0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                    0x0A, 0x46, 0x44, 0xD3, 0x01, 0x08, 0xED, 0xA6     /**< The UUID of Log Notification Service for setting advertising data. */
#define LNS_LOG_INFO_VAL_LEN        244                                                /**< Length of Log Information value. */
#define LNS_LOG_CTRL_PT_VAL_LEN     1                                                  /**< Length of Log Control Point value. */
/** @} */

/**
 * @defgroup LNS_ENUM Enumerations
 * @{
 */
/**@brief Log Notification Service Control Point. */
typedef enum
{
    LNS_CTRL_PT_TRACE_STATUS_GET = 0x01,    /**< Get trace info status. */
    LNS_CTRL_PT_TRACE_INFO_DUMP,            /**< Dump saved trace info. */
    LNS_CTRL_PT_TRACE_INFO_CLEAR,           /**< Clear trace information. */
} lns_ctrl_pt_t;

/**@brief Log Notification Service event type. */
typedef enum
{
    LNS_EVT_INVALID,                    /**< Invalid lns event type. */
    LNS_EVT_LOG_INFO_NTF_ENABLE,        /**< Trace Information notification is enabled. */
    LNS_EVT_LOG_INFO_NTF_DISABLE,       /**< Trace Information notification is disabled. */
    LNS_EVT_CTRL_PT_IND_ENABLE,         /**< Log Control Point indiaction is enabled. */
    LNS_EVT_CTRL_PT_IND_DISABLE,        /**< Log Control Point indiaction is disabled. */
    LNS_EVT_TRACE_STATUS_GET,           /**< Get log status. */
    LNS_EVT_TRACE_INFO_DUMP,            /**< Get trace information. */
    LNS_EVT_TRACE_INFO_CLEAR,           /**< Clear trace information. */
} lns_evt_type_t;
/** @} */

/**
 * @defgroup LNS_STRUCT Structures
 * @{
 */
/**@brief Log Notification Service event. */
typedef struct
{
    uint8_t             conn_idx;        /**< The index of the connection. */
    lns_evt_type_t      evt_type;        /**< The lns event type. */
} lns_evt_t;

/**@brief Log Information data. */
typedef struct 
{
    uint8_t   *p_data;     /**< Pointer to data. */
    uint32_t   length;     /**< Length of data. */
    uint32_t   offset;     /**< Offset of data. */
} lns_log_data_t;
/** @} */

/**
 * @defgroup LNS_TYPEDEF Typedefs
 * @{
 */
/**@brief Log Notification Service event handler type.*/
typedef void (*lns_evt_handler_t)(lns_evt_t *p_evt);
/** @} */

/**
 * @defgroup LNS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Log Notification Service instance and add in the DB.
 *
 * @param[in] evt_handler: Log Notification Service event handler.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t lns_service_init(lns_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Update lns gatt payload length (MTU - 3).
 *
 * @param[in] conn_idx:    Connnection index.
 * @param[in] payload_len:  Length of payload.
 *
 * @return Result of operation.
 *****************************************************************************************
 */
sdk_err_t lns_pay_load_update(uint8_t conn_idx, const uint16_t payload_len);

/**
 *****************************************************************************************
 * @brief Notify saved log information if it`s cccd is enabled.
 *
 * @param[in] conn_idx:   Connnection index.
 *
 * @return Result of operation.
 *****************************************************************************************
 */
sdk_err_t lns_log_info_send(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Send saved log status.
 *
 * @param[in] conn_idx: Connnection index.
 * @param[in] log_num:  NUmber of log.
 *
 * @return Result of operation.
 *****************************************************************************************
 */
sdk_err_t lns_log_status_send(uint8_t conn_idx, const uint8_t log_num);
/** @} */

#endif
/** @} */
/** @} */


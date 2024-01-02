/**
 *****************************************************************************************
 *
 * @file sample_service.h
 *
 * @brief Sample Service API.
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
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_SAMPLES Sample Service (SAMPLES)
 * @{
 * @brief Definitions and prototypes for the GUS interface.
 *
 * @details The Sample Service demonstrates how to add vendor service to BLE Stack database, which includes 
 *          Tx and Rx base charateristic, developer can changes and adds other characteristics.
 *          
 *          After \ref samples_init_t variable is initialized, the application must call \ref samples_service_init()
 *          to add example characteristics to the BLE Stack database, and it provides \ref samples_notify_tx_data().
 */

#ifndef _SAMPLE_PROFILE_H_
#define _SAMPLE_PROFILE_H_

#include "gr_includes.h"
#include "custom_config.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @defgroup SAMPLES_MACRO Defines
 * @{
 */
#define SAMPLES_INSTANCE_MAX       0x01                                             /**< Maximum number of Sample Service instances. The value is configurable. */
#define SAMPLES_CONNECTION_MAX     10                                               /**< Maximum number of Sample Service connections. */
#define SAMPLES_MAX_DATA_LEN       244                                              /**< Maximum length of sample charateristic value. */
#define SAMPLES_SERVICE_UUID       0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                   0x0A, 0x46, 0x44, 0xD3, 0x01, 0x01, 0xED, 0xA6   /**< The UUID of Sample Service for setting advertising data. */
/** @} */

/**
 * @defgroup SAMPLES_ENUM Enumerations
 * @{
 */
/**@brief Sample Service event type. */
typedef enum
{
    SAMPLES_EVT_INVALID,
    SAMPLES_EVT_TX_NOTIFICATION_ENABLED,
    SAMPLES_EVT_TX_NOTIFICATION_DISABLED,
    SAMPLES_EVT_RX_RECEIVE_DATA,
    SAMPLES_EVT_TX_NOTIFY_COMPLETE,
} samples_evt_type_t;
/** @} */

/**
 * @defgroup SAMPLES_STRUCT Structures
 * @{
 */
/**@brief Sample Service event. */
typedef struct
{
    samples_evt_type_t evt_type;   /**< The sample service event. */
    uint8_t            conn_idx;   /**< The connection index. */
    uint8_t           *p_data;     /**< Pointer to event data. */
    uint16_t           length;     /**< Length of event data. */
} samples_evt_t;
/** @} */

/**
 * @addtogroup SAMPLES_TYPEDEF Typedefs
 * @{
 */
/**@brief Sample Service event handler type. */
typedef void (*samples_evt_handler_t)(samples_evt_t *p_evt);
/** @} */

/**
 * @addtogroup SAMPLES_STRUCT Structures
 * @{
 */
/**@brief Sample Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    samples_evt_handler_t evt_handler;      /**<Service event handler. */
} samples_init_t;
/** @} */

/**
 * @defgroup SAMPLES_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize Sample Service instances and add in the DB.
 *
 * @param[in] ins_num:      The number of Sample Service instances.
 * @param[in] samples_init: The array of Sample Service initialization variables.
 * 
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t samples_service_init(samples_init_t samples_init[], uint8_t ins_num);

/**
 *****************************************************************************************
 * @brief Send data to peer device
 *
 * @param[in] conn_idx: Connection index
 * @param[in] ins_idx:  Sample Service Instance Index
 * @param[in] p_data:   The Pointer of sent value
 * @param[in] length:   The Lenth of sent value
 *
 * @return Result of notify and indicate value 
 *****************************************************************************************
 */
sdk_err_t samples_notify_tx_data(uint8_t conn_idx, uint8_t ins_idx, uint8_t *p_data, uint16_t length);
/** @} */

#endif
/** @} */
/** @} */


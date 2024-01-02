/**
 *****************************************************************************************
 *
 * @file pcs.h
 *
 * @brief Power Consumption Service API
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
 * @defgroup BLE_SDK_PCS Power Consumption Service (PCS)
 * @{
 * @brief Definitions and prototypes for the PCS interface.
 *
 * @details The Power Consumption Service is a customized GATT-based service with TX and Setting
 *          characteristics. The application uses the service to notify data to peer and receive
 *          some BLE parameters such as advertising interval, connect interval, PHY and so on,
 *          which can simulate different application scenes for power consumption test.
 *
 *          After \ref pcs_init_t  variable is initialized , the application must call \ref pcs_service_init()
 *          to add the ower Consumption Service and TX, Setting characteristics to the BLE Stack database.
 */

#ifndef __PCS_H__
#define __PCS_H__

#include "gr_includes.h"
#include "custom_config.h"

/**
 * @defgroup PCS_MACRO Defines
 * @{
 */
#define PCS_CONNECTION_MAX      10                                               /**< Maximum number of Power Consumption Service connections. */
#define PCS_MAX_DATA_LEN        244                                              /**< Maximum length of application data packet which is transmitted via PCS. */
#define PCS_SERVICE_UUID        0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                0x0A, 0x46, 0x44, 0xD3, 0x01, 0x05, 0xED, 0xA6   /**< The UUID of Power Consumption Service for setting advertising data. */

#define PCS_SET_PARAM_SUCCESS   0x00               /**< PCS parameters set successfully. */
#define PCS_SET_PARAM_FAIL      0x81               /**< PCS parameters set unsuccessfully. */

#define PCS_SET_ADV_DATA_3B     0x03               /**< Set 3 byte advertising data. */
#define PCS_SET_ADV_DATA_10B    0x0a               /**< Set 10 byte advertising data. */
#define PCS_SET_ADV_DATA_17B    0x11               /**< Set 17 byte advertising data. */
#define PCS_SET_ADV_DATA_24B    0x18               /**< Set 24 byte advertising data. */
#define PCS_SET_ADV_DATA_31B    0x1f               /**< Set 31 byte advertising data. */
/** @} */

/**
 * @defgroup PCS_ENUM Enumerations
 * @{
 */
/**@brief PCS Service event types. */
typedef enum
{
    PCS_EVT_INVALID,                    /**< Invalid PCS event. */
    PCS_EVT_TX_ENABLE,                  /**< TX notify has been enabled. */
    PCS_EVT_TX_DISABLE,                 /**< TX notify has been disabled. */
    PCS_EVT_SETTING_ENABLE,             /**< Setting notify has been enabled. */
    PCS_EVT_SETTING_DISABLE,            /**< Setting notify has been disabled. */
    PCS_EVT_TX_DATA_SENT,               /**< Data has been notitied completely. */
    PCS_EVT_PARAM_SET,                  /**< BLE parameters set. */
    PCS_EVT_DISCONNECTED,               /**< Disconnected. */
} pcs_evt_type_t;

/**@brief PCS Service settings types. */
typedef enum
{
    PCS_SETTING_TYPE_ADV_INTERVAL,       /**< BLE Advertising Interval parameter. */
    PCS_SETTING_TYPE_CONN_PARAM,         /**< BLE Connection parameter. */
    PCS_SETTING_TYPE_PHY,                /**< Radio Phy mode, 1M, 2M, Encoded. */
    PCS_SETTING_TYPE_ADV_DATA,           /**< BLE advertising data. */
    PCS_SETTING_TYPE_TX_POWER,           /**< Tx Power. */
} pcs_setting_type_t;
/** @} */

/**
 * @defgroup PCS_STRUCT Structures
 * @{
 */
/**@brief PCS Service event. */
typedef struct
{
    uint8_t         conn_idx;           /**< The index of the connection. */
    pcs_evt_type_t  evt_type;           /**< The PCS event type. */
    uint8_t        *p_data;             /**< Pointer to data. */
    uint16_t        length;             /**< Length of data. */
} pcs_evt_t;
/** @} */

/**
 * @defgroup pPCS_TYPEDEF Typedefs
 * @{
 */
/**@brief PCS Service event handler type. */
typedef void (*pcs_evt_handler_t)(pcs_evt_t *p_evt);
/** @} */

/**
 * @addtogroup PCS_STRUCT Structures
 * @{
 */
/**@brief PCS Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    pcs_evt_handler_t evt_handler;    /**< PCS Service event handler. */
} pcs_init_t;
/** @} */

/**
 * @defgroup PCS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a PCS Service instance and add in the database.
 *
 * @param[in] p_pcs_init: Pointer to PCS Service initialization variables.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t pcs_service_init(pcs_init_t *p_pcs_init);

/**
 *****************************************************************************************
 * @brief Send data to peer device.
 *
 * @param[in] conn_idx: Index of the connection.
 * @param[in] p_data:   Pointer to sent data.
 * @param[in] length:   Length of sent data.
 *
 * @return Result of sending data.
 *****************************************************************************************
 */
sdk_err_t pcs_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Reply parameters set result.
 *
 * @param[in] conn_idx: Index of the connection.
 * @param[in] p_data:   Pointer to sent data.
 * @param[in] length:   Length of sent data.
 *
 * @return Result of sending data.
 *****************************************************************************************
 */
sdk_err_t pcs_setting_reply(uint8_t conn_idx, uint8_t *p_data, uint16_t length);
/** @} */

#endif
/** @} */
/** @} */

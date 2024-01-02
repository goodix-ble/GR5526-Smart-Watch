/**
 *****************************************************************************************
 *
 * @file ths.h
 *
 * @brief Throughput Service API
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
 * @defgroup BLE_SDK_THS Throughput Service (THS)
 * @{
 * @brief Definitions and prototypes for the THS interface.
 *
 * @details The Throughput Service is a customized GATT-based service with Settings, Toggle,
 *          Tx and Rx characteristics. The developer uses the service to test throughput. 
 *          The data is sent to the peer as Handle Value Notification, and the data received
 *          from the peer is transmitted with GATT Write Command.
 *
 *          The peer writes Toggle characteristic to command the application starting/stopping
 *          throughput. The application calls \ref ths_data_send() to send data to the peer. 
 *          The application handles \ref THS_EVT_DATA_RECEIVED in \ref ths_init_t.evt_handler()
 *          to get the data received from the peer. The application uses ths_settings_notify() to
 *          request the change of the parameters related with throughput, including CI, MTU, PDU and PHY.
 */

#ifndef _THS_H_
#define _THS_H_

#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>

/**
 * @defgroup THS_MACRO Defines
 * @{
 */
#define THS_CONNECTION_MAX      10                                                 /**< Maximum number of Throughput Service connections. */
#define THS_MAX_DATA_LEN        512                                                /**< Maximum length of the value of Rx or Tx characteristic. */
#define THS_SERVICE_UUID        0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                0x0A, 0x46, 0x44, 0xD3, 0x01, 0x03, 0xED, 0xA6     /**< The UUID of Throughput Service for setting advertising data. */
/** @} */

/**
 * @defgroup THS_ENUM Enumerations
 * @{
 */
/**@brief Throughput data transport mode. */
typedef enum
{
    THS_SLAVE_NOTIFY_MODE,          /**< Only allow device notify. */
    THS_MASTER_WRITE_MODE,          /**< Only allow peer writes. */
    THS_DOUBLE_MODE,                /**< Allow device notify and peer writes at the same time. */
} ths_transport_mode_t;

/**@brief Throughput Service event type. */
typedef enum
{
    THS_EVT_INVALID,                /**< Invalid THS event type. */
    THS_EVT_DATA_RECEIVED,          /**< The data from the peer has been received. The application gets the data in \ref ths_evt_t.p_data. */
    THS_EVT_DATA_SENT,              /**< The data from the application has been sent, and the service is ready to accept new data from the application. */
    THS_EVT_SETTINGS_CHANGED,       /**< The settings parameters, like MTU, PHY, have been changed by the peer. */
    THS_EVT_TOGGLE_SET,             /**< The toggle state has been set by the peer. */
} ths_evt_type_t;

/**@brief Throughput toggle state of sending the data. */
enum ths_toggle_state_t
{
    THS_TOGGLE_STATE_OFF,           /**< Sending data is disabled. */
    THS_TOGGLE_STATE_ON,            /**< Sending data is enabled. */
};

/**@brief Throughput service settings types. */
typedef enum
{
    THS_SETTINGS_TYPE_CI,           /**< BLE Connection Interval parameter. */
    THS_SETTINGS_TYPE_MTU,          /**< MTU Size. */
    THS_SETTINGS_TYPE_PDU,          /**< PDU Size. */
    THS_SETTINGS_TYPE_PHY,          /**< Radio Phy mode, 1M, 2M, Encoded. */
    THS_SETTINGS_TYPE_TRANS_MODE,   /**< Data transmission mode. */
    THS_SETTINGS_TYPE_TX_POWER,     /**< Connect Tx power. */
}ths_settings_type_t;
/** @} */

/**
 * @defgroup THS_STRUCT Structures
 * @{
 */
/**@brief Throughput Service event. */
typedef struct
{
    ths_evt_type_t      evt_type;        /**< The THS event type. */
    ths_settings_type_t setting_type;    /**< The THS parameter set type. */
    uint8_t             conn_idx;        /**< The index of the connection for the data transmission. */
    uint8_t            *p_data;          /**< Pointer to the received data. */
    uint16_t            length;          /**< Length of received data. */
} ths_evt_t;
/** @} */

/**
 * @defgroup THS_TYPEDEF Typedefs
 * @{
 */
/**@brief Throughput Service event handler type. */
typedef void (*ths_evt_handler_t)(ths_evt_t *p_evt);
/** @} */

/**
 * @addtogroup THS_STRUCT Structures
 * @{
 */
/**@brief Throughput Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    ths_evt_handler_t    evt_handler;      /**< Throughput Service event handler which must be provided by the application to send and receive the data and the settings change. */
    ths_transport_mode_t transport_mode;   /**< The transport mode of a device. */
} ths_init_t;
/** @} */

/**
 * @defgroup THS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Throughput Service instance and add in the DB.
 *
 * @param[in] p_ths_init: Throughput Service initialization variable
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t ths_service_init(ths_init_t *p_ths_init);

/**
 *****************************************************************************************
 * @brief Send data to peer device
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_data:   The Pointer of sent value
 * @param[in] length:   The Length of sent value
 *
 * @return Result of notify and indicate value
 *****************************************************************************************
 */
sdk_err_t ths_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Notify the peer device of the change of settings.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] p_settings: Pointer to the value of new settings.
 * @param[in] length:     The Length of the value of new settings.
 *
 * @return Result of notify and indicate value
 *****************************************************************************************
 */
sdk_err_t ths_settings_notify(uint8_t conn_idx, uint8_t *p_settings, uint16_t length);

/**
 *****************************************************************************************
 * @brief Get current transport mode of device.
 *
 * @return Current transport mode.
 *****************************************************************************************
 */
ths_transport_mode_t ths_transport_mode_get(void);
/** @} */

#endif
/** @} */
/** @} */

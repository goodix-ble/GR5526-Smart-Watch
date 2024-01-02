/**
 *****************************************************************************************
 *
 * @file mlmr.h
 *
 * @brief Multi Link Multi Role Service API
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
 * @defgroup BLE_SDK_MLMR Multi Link Multi Role Service (MLMR)
 * @{
 * @brief Definitions and prototypes for the MLMR interface.
 *
 * @details The Multi Link Multi Role Service is a customized GATT-based service with Tx, Rx and Flow Control
 *          characteristics. The application uses the service to send and receive data to and
 *          from the peer. The application data is sent to the peer as Handle Value Notification,
 *          and the data received from the peer is transmitted with GATT Write Command.
 *
 *          After \ref mlmr_init_t variable is initialized , the application must call \ref mlmr_service_init()
 *          to add the Goodix Uart Service and Rx, Tx, Flow Control characteristics to the BLE Stack
 *          database. The application can send the data to the peer with \ref mlmr_tx_data_send() after 
 *          \ref MLMR_EVT_TX_PORT_OPENED received. The application should copy the received data to its own buffer
 *          when handling \ref MLMR_EVT_RX_DATA_RECEIVED.
 */

#ifndef __MLMR_H__
#define __MLMR_H__

#include "gr_includes.h"
#include "custom_config.h"

/**
 * @defgroup MLMR_MACRO Defines
 * @{
 */
#define MLMR_CONNECTION_MAX      10                                                 /**< Maximum number of Multi Link Multi Role Service connections. */
#define MLMR_MAX_DATA_LEN        247                                                /**< Maximum length of application data packet which is transmitted via MLMR. */
#define MLMR_FLOW_CTRL_LEN       1                                                  /**< Maximum length of ble flow control data packet which is transmitted via MLMR. */
#define MLMR_SERVICE_UUID        0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                0x0A, 0x46, 0x44, 0xD3, 0x01, 0x02, 0xED, 0xA6     /**< The UUID of Multi Link Multi Role Service for setting advertising data. */
/** @} */

/**
 * @defgroup MLMR_ENUM Enumerations
 * @{
 */
/**@brief Multi Link Multi Role Service event types. */
typedef enum
{
    MLMR_EVT_INVALID,                /**< Invalid MLMR event. */
    MLMR_EVT_RX_DATA_RECEIVED,       /**< The data from the peer has been received. */
    MLMR_EVT_TX_DATA_SENT,           /**< The data from the application has been sent, and the service is ready to accept new data from the application. */
    MLMR_EVT_TX_PORT_OPENED,         /**< Tx port has been opened. */
    MLMR_EVT_TX_PORT_CLOSED,         /**< Tx port has been closed. */
    MLMR_EVT_FLOW_CTRL_ENABLE,       /**< MLMR flow control been enabled. */
    MLMR_EVT_FLOW_CTRL_DISABLE,      /**< MLMR flow control been disabled. */
    MLMR_EVT_TX_FLOW_OFF,            /**< Tx flow off control request. */
    MLMR_EVT_TX_FLOW_ON,             /**< Tx flow on control request. */
} mlmr_evt_type_t;

/**@brief Flow control state for MLMR service. */
enum mlmr_flow_ctrl_state
{
  MLMR_FLOW_CTRL_STATE_OFF = 0,      /**< Indicate that MLMR can not receive data from peer. */
  MLMR_FLOW_CTRL_STATE_ON            /**< Indicate that MLMR can receive data from peer. */
};
/**@brief Underlying type used for the MLMR flow control state. */
typedef uint8_t mlmr_flow_ctrl_state_t;
/** @} */

/**
 * @defgroup MLMR_STRUCT Structures
 * @{
 */
/**@brief Multi Link Multi Role Service event. */
typedef struct
{
    mlmr_evt_type_t  evt_type;   /**< The MLMR event. */
    uint8_t         conn_idx;   /**< The index of the connection for the data transmission. */
    uint8_t        *p_data;     /**< Pointer to the buffer within received data. */
    uint16_t        length;     /**< Length of received data. */
} mlmr_evt_t;
/** @} */

/**
 * @defgroup MLMR_TYPEDEF Typedefs
 * @{
 */
/**@brief Multi Link Multi Role Service event handler type. */
typedef void (*mlmr_evt_handler_t)(mlmr_evt_t *p_evt);
/** @} */

/**
 * @addtogroup MLMR_STRUCT Structures
 * @{
 */
/**@brief Multi Link Multi Role Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    mlmr_evt_handler_t evt_handler;                     /**< Multi Link Multi Role Service event handler which must be provided by the application to send and receive the data. */
} mlmr_init_t;
/** @} */

/**
 * @defgroup MLMR_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Multi Link Multi Role Service instance and add in the database.
 *
 * @param[in] p_mlmr_init: Pointer to Multi Link Multi Role Service initialization variables.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t mlmr_service_init(mlmr_init_t *p_mlmr_init);

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
sdk_err_t mlmr_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Send MLMR Rx flow control state to peer device
 *
 * @param[in] conn_idx:  Index of the connection.
 * @param[in] flow_ctrl: MLMR Rx flow control state
 *
 * @return Result of sending MLMR Rx flow control state.
 *****************************************************************************************
 */
sdk_err_t mlmr_rx_flow_ctrl_set(uint8_t conn_idx, mlmr_flow_ctrl_state_t flow_ctrl);
/** @} */

#endif
/** @} */
/** @} */

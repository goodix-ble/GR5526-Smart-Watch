/**
 *****************************************************************************************
 *
 * @file gus.h
 *
 * @brief Goodix UART Service API
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
 * @defgroup BLE_SDK_GUS Goodix UART Service (GUS)
 * @{
 * @brief Definitions and prototypes for the GUS interface.
 *
 * @details The Goodix UART Service is a customized GATT-based service with Tx, Rx and Flow Control
 *          characteristics. The application uses the service to send and receive data to and
 *          from the peer. The application data is sent to the peer as Handle Value Notification,
 *          and the data received from the peer is transmitted with GATT Write Command.
 *
 *          After \ref gus_init_t variable is initialized , the application must call \ref gus_service_init()
 *          to add the Goodix Uart Service and Rx, Tx, Flow Control characteristics to the BLE Stack
 *          database. The application can send the data to the peer with \ref gus_tx_data_send() after
 *          \ref GUS_EVT_TX_PORT_OPENED received. The application should copy the received data to its own buffer
 *          when handling \ref GUS_EVT_RX_DATA_RECEIVED.
 */

#ifndef __GUS_H__
#define __GUS_H__

#include "gr_includes.h"
#include "custom_config.h"

/**
 * @defgroup GUS_MACRO Defines
 * @{
 */
#define GUS_CONNECTION_MAX      10                                                 /**< Maximum number of Goodix UART Service connections. */
#define GUS_MAX_DATA_LEN        247                                                /**< Maximum length of application data packet which is transmitted via GUS. */
#define GUS_FLOW_CTRL_LEN       1                                                  /**< Maximum length of ble flow control data packet which is transmitted via GUS. */
#define GUS_SERVICE_UUID        0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                0x0A, 0x46, 0x44, 0xD3, 0x01, 0x02, 0xED, 0xA6     /**< The UUID of Goodix UART Service for setting advertising data. */
/** @} */

/**
 * @defgroup GUS_ENUM Enumerations
 * @{
 */
/**@brief Goodix UART Service event types. */
typedef enum
{
    GUS_EVT_INVALID,                /**< Invalid GUS event. */
    GUS_EVT_RX_DATA_RECEIVED,       /**< The data from the peer has been received. */
    GUS_EVT_TX_DATA_SENT,           /**< The data from the application has been sent, and the service is ready to accept new data from the application. */
    GUS_EVT_TX_PORT_OPENED,         /**< Tx port has been opened. */
    GUS_EVT_TX_PORT_CLOSED,         /**< Tx port has been closed. */
    GUS_EVT_FLOW_CTRL_ENABLE,       /**< GUS flow control been enabled. */
    GUS_EVT_FLOW_CTRL_DISABLE,      /**< GUS flow control been disabled. */
    GUS_EVT_TX_FLOW_OFF,            /**< Tx flow off control request. */
    GUS_EVT_TX_FLOW_ON,             /**< Tx flow on control request. */
} gus_evt_type_t;

/**@brief Flow control state for GUS service. */
enum gus_flow_ctrl_state
{
  GUS_FLOW_CTRL_STATE_OFF = 0,      /**< Indicate that GUS can not receive data from peer. */
  GUS_FLOW_CTRL_STATE_ON            /**< Indicate that GUS can receive data from peer. */
};
/**@brief Underlying type used for the GUS flow control state. */
typedef uint8_t gus_flow_ctrl_state_t;
/** @} */

/**
 * @defgroup GUS_STRUCT Structures
 * @{
 */
/**@brief Goodix UART Service event. */
typedef struct
{
    gus_evt_type_t  evt_type;   /**< The GUS event. */
    uint8_t         conn_idx;   /**< The index of the connection for the data transmission. */
    uint8_t        *p_data;     /**< Pointer to the buffer within received data. */
    uint16_t        length;     /**< Length of received data. */
} gus_evt_t;
/** @} */

/**
 * @addtogroup GUS_TYPEDEF Typedefs
 * @{
 */
/**@brief Goodix UART Service event handler type. */
typedef void (*gus_evt_handler_t)(gus_evt_t *p_evt);
/** @} */

/**
 * @addtogroup GUS_STRUCT Structures
 * @{
 */
/**@brief Goodix UART Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    gus_evt_handler_t evt_handler;                     /**< Goodix UART Service event handler which must be provided by the application to send and receive the data. */
} gus_init_t;
/** @} */

/**
 * @defgroup GUS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Goodix UART Service instance and add in the database.
 *
 * @param[in] p_gus_init: Pointer to Goodix UART Service initialization variables.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t gus_service_init(gus_init_t *p_gus_init);

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
sdk_err_t gus_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Send GUS Rx flow control state to peer device
 *
 * @return Result of sending GUS Rx flow control state.
 *****************************************************************************************
 */
sdk_err_t gus_rx_flow_ctrl_set(uint8_t conn_idx, gus_flow_ctrl_state_t flow_ctrl);

/**
 *****************************************************************************************
 * @brief Provide the interface for other modules to obtain the gus service start handle .
 *
 * @return The gus service start handle.
 *****************************************************************************************
 */
uint16_t gus_service_start_handle_get(void);
/** @} */

#endif
/** @} */
/** @} */

/**
 *****************************************************************************************
 *
 * @file ths_c.h
 *
 * @brief Throughput Client API
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
 */
/**
 * @defgroup BLE_SDK_THS_C Throughput Service Client (THS_C)
 * @{
 * @brief THS Client Interface module.
 *
 * @details The Throughput Service Client contains the APIs and types, which can be used by the
 *          application to perform scanning, connection and discovery of Throughput Service at peer and
 *          interact with it.
 *
 *          The application must provide to register, then call \ref ths_client_init(). After the module can
 *          send setting parameters and throughput data to peer, it will notify application. 
 *          It includes only Throughput Service notify data, only Throughput Service Client write data
 *          and both send data test mode \ref ths_c_transport_mode_t.
 */

#ifndef __THS_C_H__
#define __THS_C_H__

#include "ble_prf_types.h"
#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup THS_C_MACRO Defines
 * @{
 */
/**
 * @defgroup THS_UUID THS UUIDs
 * @{
 * @brief THS service and characteristcs UUID.
 */
#define THS_C_CONNECTION_MAX                10                                                    /**< Maximum number of THS Client connections. */
#define THS_SVC_UUID                        {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                             0x0A, 0x46, 0x44, 0xD3, 0x01, 0x03, 0xED, 0xA6}      /**< UUID of THS Service. */
#define THS_TX_CHAR_UUID                    {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                             0x0A, 0x46, 0x44, 0xD3, 0x02, 0x03, 0xED, 0xA6}      /**< UUID of THS Tx Characteristic. */
#define THS_RX_CHAR_UUID                    {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                             0x0A, 0x46, 0x44, 0xD3, 0x03, 0x03, 0xED, 0xA6}      /**< UUID of THS Rx Characteristic. */
#define THS_SETTING_CHAR_UUID               {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                             0x0A, 0x46, 0x44, 0xD3, 0x04, 0x03, 0xED, 0xA6}      /**< UUID of THS Setting Characteristic. */
#define THS_TOGGLE_CHAR_UUID                {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                             0x0A, 0x46, 0x44, 0xD3, 0x05, 0x03, 0xED, 0xA6}      /**< UUID of THS Toggle Characteristic. */
/** @} */
/** @} */

/**
 * @defgroup THS_C_ENUM Enumerations
 * @{
 */
/**@brief Throughput Service Client data transport mode. */
typedef enum
{
    THS_C_SLAVE_NOTIFY_MODE,          /** The device recieves data from the peer via notify. */
    THS_C_MASTER_WRITE_MODE,          /** The device writes data to the peer. */
    THS_C_DOUBLE_MODE,                /** The device recieve data from the peer via notify and write data to the peer at the same time. */
} ths_c_transport_mode_t;

/**@brief Throughput service settings types. */
typedef enum
{
    THS_C_SETTINGS_TYPE_CI,           /**< BLE Connection Interval parameter. */
    THS_C_SETTINGS_TYPE_MTU,          /**< MTU Size. */
    THS_C_SETTINGS_TYPE_PDU,          /**< PDU Size. */
    THS_C_SETTINGS_TYPE_PHY,          /**< Radio Phy mode, 1M, 2M, Encoded. */
    THS_C_SETTINGS_TYPE_TRANS_MODE,   /**< Data transmission mode. */
    THS_C_SETTINGS_TYPE_TX_POWER,     /**< Connect Tx power. */
} ths_c_settings_type_t;

/**@brief Throughput Service Client event type. */
typedef enum
{
    THS_C_EVT_INVALID,                      /**< THS Client invalid event. */
    THS_C_EVT_DISCOVERY_COMPLETE,           /**< THS Client has found THS service and its characteristics. */
    THS_C_EVT_DISCOVERY_FAIL,               /**< THS Client found THS service failed because of invalid operation or no found at the peer. */
    THS_C_EVT_TX_NTF_SET_SUCCESS,           /**< THS Client has set peer Tx notify. */
    THS_C_EVT_SETTING_NTF_SET_SUCCESS,      /**< THS Client has set peer Settings notify. */
    THS_C_EVT_TOGGLE_SET_SUCCESS,           /**< THS Client has set toggle no. */
    THS_C_EVT_SETTING_RSP_RECEIVE,          /**< THS CLient has received response of parameter set. */
    THS_C_EVT_THRP_DATA_RECEIVE,            /**< THS Client has received throughput data from peer. */
    THS_C_EVT_TX_SUCCESS,                   /**< THS Client has sent something to a peer successfully. */
    THS_C_EVT_PARAM_SET_SUCCESS,            /**< THS Client has set parameter for connection and transport mode. */
    THS_C_EVT_WRITE_OP_ERR,                 /**< Error occured when THS Client writen to peer. */
} ths_c_evt_type_t;
/** @} */

/**
 * @defgroup THS_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t ths_srvc_start_handle;       /**< THS Service start handle. */
    uint16_t ths_srvc_end_handle;         /**< THS Service end handle. */
    uint16_t ths_tx_handle;               /**< THS Tx characteristic handle which has been got from peer. */
    uint16_t ths_tx_cccd_handle;          /**< THS Tx characteristic CCCD handle which has been got from peer. */
    uint16_t ths_rx_handle;               /**< THS Rx characteristic handle which has been got from peer. */
    uint16_t ths_setting_handle;          /**< THS setting characteristic handle which has been got from peer. */
    uint16_t ths_setting_cccd_handle;     /**< THS setting characteristic CCCD handle which has been got from peer. */
    uint16_t ths_toggle_handle;           /**< THS toggle characteristic handle which has been got from peer. */
} ths_c_handles_t;

/**@brief Throughput Service Client event. */
typedef struct
{
    uint8_t           conn_idx;            /**< The connection index. */
    ths_c_evt_type_t  evt_type;            /**< THS client event type. */
    uint8_t          *p_data;              /**< Pointer to event data. */
    uint16_t          length;              /**< Length of event data. */
} ths_c_evt_t;
/** @} */

/**
 * @defgroup THS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief  Throughput Service Client event handler type. */
typedef void (*ths_c_evt_handler_t)(ths_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup THS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register THS Client event handler.
 *
 * @param[in] evt_handler: Throughput Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t ths_client_init(ths_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery THS on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ths_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer THS Tx characteristic notify.
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] is_enable: Enable or disable THS Tx notify.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ths_c_tx_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Enable or disable peer THS setting characteristic notify.
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] is_enable: Enable or disable THS setting notify.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ths_c_setting_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Send communication parameter to peer.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to communication parameter data need sent.
 * @param[in] length:   Length of communication parameter data need sent.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ths_c_comm_param_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Set Throughput service toggle state of sending the data.
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] is_enable: Enable or disable toggle.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ths_c_toggle_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Send data to peer.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data need sent.
 * @param[in] length:   Length of data need sent.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ths_c_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);
/** @} */
#endif
/** @} */
/** @} */

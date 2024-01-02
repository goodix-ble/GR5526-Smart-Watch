/**
 *****************************************************************************************
 *
 * @file ancs_c.h
 *
 * @brief Apple Notification Center Service API.
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
 * @defgroup BLE_SDK_ANCS Apple Notification Center Service (ANCS)
 * @{
 * @brief Definitions and prototypes for the ANCS interface.
 
 * @details ANCS provides a way for BLE devices to receive IOS mobile phone notifications. 
 *          The service consists of three eigenvalues, including notification source, 
 *          data source, control point.
 *          
 *          The application needs to call \ref ancs_c_client_init() to initialize, and then 
 *          use \ref ancs_c_discovery_service() to discover ANCS-related services on IOS devices. 
 *          After discovery, ancs_c_on_browse_svc_evt function is called to parse and save 
 *          the handle corresponding to each service. These handles are used for data
 *          transmission.
 *      
 *          Secondly, use \ref ancs_c_ntf_source_notify_set() and \ref ancs_c_data_source_notify_set()
 *          to enable notification source & data source's CCD. The IOS notification is then sent to the 
 *          BLE device side immediately.
 *      
 *          Finally, the application can use \ref ancs_decode_notification_source() and
 *          \ref ancs_decode_data_source() to parse the ANCS message and \ref ancs_c_write_control_point() 
 *          to command the control point. For specific commands, please refer to the ANCS protocol specification.
 *
 */

#ifndef _ANCS_H_
#define _ANCS_H_

#include "gr_includes.h"
#include "ble_prf_types.h"
#include "custom_config.h"
#include <stdint.h>

/**
 * @defgroup ANCS_C_MACRO Defines
 * @{
 */
#define ANCS_C_CONNECTION_MAX                10                                                   /**< Maximum number of ANCS Client connections. */
#define ANCS_SRVC_UUID                       0xd0, 0x00, 0x2d, 0x12, 0x1e, 0x4b, 0x0f, 0xa4,\
                                             0x99,0x4e, 0xce, 0xb5, 0x31, 0xf4, 0x05, 0x79        /**< UUID of Apple notification center service. */
#define ANCS_NTF_SOURCE_UUID                 0xbd, 0x1d, 0xa2, 0x99, 0xe6, 0x25, 0x58, 0x8c,\
                                             0xd9, 0x42, 0x01, 0x63, 0x0d, 0x12, 0xbf, 0x9f       /**< UUID of notification source. */
#define ANCS_CONTROL_POINT_UUID              0xd9, 0xd9, 0xaa, 0xfd, 0xbd, 0x9b, 0x21, 0x98,\
                                             0xa8, 0x49, 0xe1, 0x45, 0xf3, 0xd8, 0xd1, 0x69       /**< UUID of control point. */
#define ANCS_DATA_SOURCE_UUID                0xfb, 0x7b, 0x7c, 0xce, 0x6a, 0xb3, 0x44, 0xbe,\
                                             0xb5, 0x4b, 0xd6, 0x24, 0xe9, 0xc6, 0xea, 0x22       /**< UUID of data source. */

/** @} */

/**
 * @defgroup ANCS_ENUM Enumerations
 * @{
 */
/**@brief Event types that are passed from client to application on an event. */
typedef enum
{
    BLE_ANCS_C_EVT_INVALID,                        /**< ANCS Client invalid event type. */
    BLE_ANCS_C_EVT_DISCOVERY_CPLT,                 /**< ANCS Client has found ANCS service and its characteristics. */
    BLE_ANCS_C_EVT_DISCOVERY_FAILED,               /**< ANCS Client found ANCS service failed because of invalid operation or no found at the peer. */
    BLE_ANCS_C_EVT_NTF_SOURCE_NTF_ENABLED,         /**< ANCS Client has enable notification for notification source. */
    BLE_ANCS_C_EVT_DATA_SOURCE_NTF_ENABLED,        /**< ANCS Client has enable notification for data source. */
    BLE_ANCS_C_EVT_NTF_SOURCE_RECEIVE,             /**< ANCS Client has receive notification from notification source. */
    BLE_ANCS_C_EVT_DATA_SOURCE_RECEIVE,            /**< ANCS Client has receive notification from data source. */
    BLE_ANCS_C_EVT_WRITE_OP_ERR,
} ble_ancs_c_evt_type_t;
/** @} */

/**
 * @defgroup ANCS_STRUCT Structures
 * @{
 */
/**@brief  ancs handle structure. */
typedef struct
{
    uint16_t ancs_service_handle;                  /**< Handle of ancs service as provided by a discovery. */
    uint16_t ancs_ntf_source_handle;               /**< Handle of ancs  notification source characteristic as provided by a discovery. */
    uint16_t ancs_ntf_source_cccd_handle;          /**< Handle of CCCD of ancs  control point characteristic as provided by a discovery. */
    uint16_t ancs_control_point_handle;            /**< Handle of ancs  control point characteristic as provided by a discovery. */
    uint16_t ancs_data_source_handle;              /**< Handle of ancs  data source characteristic as provided by a discovery. */
    uint16_t ancs_data_source_cccd_handle;         /**< Handle of CCCD of ancs data source characteristic as provided by a discovery. */
}ancs_c_att_handles_t;

/**@brief ANCS Client event. */
typedef struct
{
    uint8_t          conn_idx;                     /**< The index of the connection. */
    ble_ancs_c_evt_type_t evt_type;                /**< The ANCS event type. */

} ancs_c_evt_t;
/** @} */

/**
 * @defgroup ANCS_TYPEDEF Typedefs
 * @{
 */
/**@brief Apple Notification Center Service event handler type.*/
typedef void (*ancs_c_evt_handler_t)(ancs_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup ANCS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize ANCS structure of handle.
 *****************************************************************************************
 */
sdk_err_t ancs_c_client_init(ancs_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief To access phone's all services about ANCS.
 *
 * @param[in] conn_idx: Connection index.
 *****************************************************************************************
*/
sdk_err_t ancs_c_discovery_service(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief  enable ancs notification source CCCD.
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] is_enable: Start or stop the notification.  
 * @return    success or not.
 *****************************************************************************************
 */
sdk_err_t ancs_c_ntf_source_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief  enable ancs data source CCCD.
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] is_enable: Start or stop the notification. 
 * @return    success or not.
 *****************************************************************************************
 */
sdk_err_t ancs_c_data_source_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief This function implements writing commands to control points.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to send out data.
 * @param[in] length:   Length of data sent out.
 *****************************************************************************************
 */
sdk_err_t ancs_c_write_control_point(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/** @} */

#endif
/** @} */
/** @} */

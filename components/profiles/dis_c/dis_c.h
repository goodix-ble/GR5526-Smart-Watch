/**
*****************************************************************************************
*
* @file dis_c.h
*
* @brief Device Information Service Client API
*
*****************************************************************************************
*/

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
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
 * @defgroup BLE_SDK_DIS_C Device Information Service Client (DIS_C)
 * @{
 * @brief Device Information Service Client module.
 *
 * @details The Device Information Service Client contains the APIs and types, which can be used by the
 *          application to discover Device Information Service of peer and interact with it.
 *
 *          The application must provide an event handler to register, then call \ref dis_client_init().
 *          After Device Information Service Client discovers peer Device Information Service,
 *          application calls \ref dis_c_char_value_read() and get peer device information.
 */

#ifndef __DIS_C_H__
#define __DIS_C_H__

#include "gr_includes.h"
#include "ble_prf_types.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup DIS_C_MACRO Defines
 * @{
 */
#define DIS_C_CONNECTION_MAX                 10                             /**< Maximum number of DIS Client connections. */
#define DIS_C_STRING_LEN_MAX                 128                            /**< Maximal length for Characteristic values - 128 bytes. */

/**
 * @defgroup DIS_IEEE_11073_BODY IEEE 11073-20601 Authoritative Body Type
 * @{
 */
#define DIS_C_11073_BODY_EMPTY                0              /**< Empty body type. */
#define DIS_C_11073_BODY_IEEE                 1              /**< IEEE body type. */
#define DIS_C_11073_BODY_CONTINUA             2              /**< Continua body type. */
#define DIS_C_11073_BODY_EXP                  254            /**< Exp body type. */
/** @} */

/** @} */

/**
 * @defgroup DIS_C_ENUM Enumerations
 * @{
 */
/**@brief Device Information Service Client event type. */
typedef enum
{
    DIS_C_EVT_INVALID,                      /**< DIS Client invalid event. */
    DIS_C_EVT_DISCOVERY_COMPLETE,           /**< DIS Client has found Device Information Service and its characteristics. */
    DIS_C_EVT_DISCOVERY_FAIL,               /**< DIS Client found DIS service failed because of invalid operation or no found at the peer. */
    DIS_C_EVT_DEV_INFORMATION_READ_RSP,     /**< DIS Client has received device information value read response. */
} dis_c_evt_type_t;

/**@brief Device Information Service Client characteristic type. */
typedef enum
{
    DIS_C_SYS_ID,                /**< System ID characteristic. */
    DIS_C_MODEL_NUM,             /**< Model Number String characteristic. */
    DIS_C_SERIAL_NUM,            /**< Serial Number String characteristic. */
    DIS_C_HW_REV,                /**< Hardware Revision String characteristic. */
    DIS_C_FW_REV,                /**< Firmware Revision String characteristic. */
    DIS_C_SW_REV,                /**< Software Revision String characteristic. */
    DIS_C_MANUF_NAME,            /**< Manufacturer Name String characteristic. */
    DIS_C_CERT_LIST,             /**< IEEE 11073-20601 Regulatory Certification Data List characteristic. */
    DIS_C_PNP_ID,                /**< PnP ID characteristic. */
    DIS_C_CHARACTER_NB           /**< Number of all Device Information Service characteristics. */
} dis_c_char_type_t;
/** @} */

/**
 * @defgroup DIS_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t dis_srvc_start_handle;                 /**< DIS Serivce start handle. */
    uint16_t dis_srvc_end_handle;                   /**< DIS Service end handle. */
    uint16_t dis_char_handle[DIS_C_CHARACTER_NB];   /**< DIS characteristic handle. */
} dis_c_handles_t;

/**@brief Response data for string-based DIS characteristics. */
typedef struct
{
    uint8_t   *p_data;    /**< Pointer to response data. */
    uint16_t   length;    /**< Response data length. */
} dis_c_string_t;

/**@brief Response data for System ID parameters. */
typedef struct
{
    uint8_t manufacturer_id[5];   /**< Manufacturer-defined ID. */
    uint8_t org_unique_id[3];     /**< Organizationally unique ID (OUI) which is issued by IEEE. */
} dis_c_sys_id_t;

/**@brief Response data for IEEE 11073-20601 Regulatory Certification Data List Structure. */
typedef struct
{
    uint8_t  *p_list;               /**< Pointer to the list which contains the encoded opaque structure based on IEEE 11073-20601 specification. */
    uint16_t  list_length;          /**< Length of the list. */
} dis_c_reg_cert_data_list_t;

/**@brief Response data for PnP ID parameters */
typedef struct
{
    uint8_t  vendor_id_source;    /**< Vendor ID Source. */
    uint16_t vendor_id;           /**< Vendor ID. */
    uint16_t product_id;          /**< Product ID. */
    uint16_t product_version;     /**< Product Version. */
} dis_c_pnp_id_t;

/**@brief Device Information Service Client Read Response encode structure. */
typedef struct
{
    dis_c_char_type_t char_type;                   /**< Characteristic type. */
    union
    {
        dis_c_sys_id_t             sys_id;         /**< System ID characteristic response data. */
        dis_c_string_t             string_data;    /**< Model Number, Serial Number, Hardware Revision, Firmware Revision, Software Revision, Manufacturer Name String characteristic response data. */
        dis_c_reg_cert_data_list_t cert_list;      /**< IEEE 11073-20601 Regulatory Certification Data List characteristic response data. */
        dis_c_pnp_id_t             pnp_id;         /**< PnP ID characteristic response data. */
    } encode_rst;                                  /**< Result of encoding. */
} ble_dis_c_read_rsp_t;

/**@brief Device Information Service Client event. */
typedef struct
{
    uint8_t                   conn_idx;            /**< The connection index. */
    dis_c_evt_type_t          evt_type;            /**< DIS Client event type. */
    ble_dis_c_read_rsp_t      read_rsp;            /**< DIS Client characteristic Read Response encode. */
} dis_c_evt_t;
/** @} */

/**
 * @defgroup DIS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief  Device Information Service Client event handler type. */
typedef void (*dis_c_evt_handler_t)(dis_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup DIS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register DIS Client event handler.
 *
 * @param[in] evt_handler: Device Information Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t dis_client_init(dis_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery Device Information Service on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t dis_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Read Device Information Service characteristic value.
 *
 * @param[in] conn_idx:       Index of connection.
 * @param[in] char_read_type: Type of characteristic read.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t dis_c_char_value_read(uint8_t conn_idx, dis_c_char_type_t char_read_type);
/** @} */

#endif

/** @} */
/** @} */


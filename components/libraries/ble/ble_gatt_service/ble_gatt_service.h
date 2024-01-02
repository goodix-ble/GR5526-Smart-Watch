/**
 ****************************************************************************************
 *
 * @file ble_gatt_service.h
 *
 * @brief BLE GATT Service Module Header
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

#ifndef __BLE_GATT_SERVICE_H__
#define __BLE_GATT_SERVICE_H__

#include "ble.h"
#include "app_memory.h"

/**
 * @defgroup BLE_GATT_SERVICE_MAROC Defines
 * @{
 */
#define BLE_GATT_SERV_MALLOC(size)           app_malloc(size)         /**< Malloc adaptor. */
#define BLE_GATT_SERV_FREE(ptr)              app_free(ptr)            /**< Free adaptor. */
#define BLE_GATT_SERV_REALLOC(ptr, size)     app_realloc(ptr, size)   /**< Realloc adaptor. */
/** @} */

/**
 * @defgroup BLE_GATT_SERVICE_ENUM Enumerations
 * @{
 */
/**@brief BLE ATT UUID types. */
typedef enum 
{
    BLE_ATT_UUID_16,                /**< 16-bit UUID type. */
    BLE_ATT_UUID_128                /**< 128-bit UUID type. */
} ble_att_uuid_type_t;

/**@brief BLE ATT properties. */
typedef enum 
{
    BLE_ATT_PROP_BROADCAST     = 0x01 << 0,        /**< Broadcast. */
    BLE_ATT_PROP_READ          = 0x01 << 1,        /**< Read. */
    BLE_ATT_PROP_WRITE_NO_RESP = 0x01 << 2,        /**< Write CMD. */
    BLE_ATT_PROP_WRITE         = 0x01 << 3,        /**< Write with response. */
    BLE_ATT_PROP_NOTIFY        = 0x01 << 4,        /**< Notify. */
    BLE_ATT_PROP_INDICATE      = 0x01 << 5,        /**< Indicate. */
    BLE_ATT_PROP_WRITE_SIGNED  = 0x01 << 6,        /**< Signed write. */
    BLE_ATT_PROP_EXTENDED      = 0x01 << 7,        /**< Extended. */
} ble_att_prop_t;

/**@brief BLE ATT permission. */
typedef enum 
{
    BLE_ATT_NOAUTH,        /**< No need to be encrypted or authenticated. */
    BLE_ATT_UNAUTH,        /**< Need to be encrypted, but not to be authenticated. */
    BLE_ATT_AUTH,          /**< Need to be encrypted and authenticated (MITM). */
    BLE_ATT_SEC_CON,       /**< Need to be encrypted and authenticated (secure connections). */
} ble_att_perm_t;

/**@brief BLE ATT request reply types. */
typedef enum 
{
    BLE_ATT_READ_REPLY,     /**< GATTS read reply. */
    BLE_ATT_WRITE_REPLY     /**< GATTS write reply. */
} ble_att_req_reply_type_t;
/** @} */

/**
 * @defgroup BLE_GATT_SERVICE_TYPEDEF Typedefs
 * @{
 */
/**@brief Read attribute value request handler. */
typedef void (*ble_gatts_read_req_handler_t)(uint8_t conn_idx, uint16_t handle);

/**@brief Write attribute value request handler. */
typedef void (*ble_gatts_write_req_handler_t)(uint8_t conn_idx, uint16_t handle, uint16_t offset, const uint8_t *p_value, uint16_t length);

/**@brief Notification or indication  handler. */
typedef void (*ble_gatts_ntf_ind_handler_t)(uint8_t conn_idx, uint8_t status, gatt_evt_type_t type,  uint16_t h_offset);

/**@brief CCCD value recovery handler. */
typedef void (*ble_gatts_cccd_recovery_handler_t)(uint8_t conn_idx, uint16_t handle, uint16_t cccd_val);

/**@brief Connect handler. */
typedef void (*ble_gatts_on_connect_handler_t)(uint8_t conn_idx);

/**@brief Disconnect handler. */
typedef void (*ble_gatts_on_disconnect_handler_t)(uint8_t conn_idx, uint8_t reason);

/** @} */

/**
 * @defgroup BLE_GATT_SERVICE_STRUCT Structures
 * @{
 */
/**@brief BLE ATT UUID. */
typedef struct 
{
    ble_att_uuid_type_t type;      /**< UUID type. */
    union 
    {
        uint16_t uuid16;          /**< 16-bit UUID. */
        uint8_t  uuid128[16];     /**< 128-bit UUID. */
    }uuid;                        /**< UUID. */
} ble_att_uuid_t;

/**@brief GATT Attribute. */
typedef struct
{
    ble_att_uuid_t  att_uuid;     /**< Pointer to the attribute UUID. */
    uint16_t        att_prop;     /**< ATT propertie bit map. */
    ble_att_perm_t  att_perm;     /**< ATT permission. */
    uint16_t        max_len;      /**< Maximum attribute value length. */
} ble_gatts_att_t;

/**@brief GATT Attribute read/write request reply. */
typedef struct
{
    ble_att_req_reply_type_t type;     /**< Reply type. */
    union
    {
        struct
        {
            uint16_t     handle;     /**< Value handle. */
            uint8_t      status;     /**< Reply status. */
        }wr;                         /**< Write reply. */
        struct
        {
            uint16_t     handle;     /**< Value handle. */
            uint8_t      status;     /**< Reply status. */
            uint8_t     *p_value;    /**< Pointer to read value. */
            uint16_t     length;     /**< Length of read value. */
        }rd;                         /**< Read reply. */
    }param;                          /**< Parameter of reply. */
} ble_gatts_att_req_reply_t;

/**@brief BLE GATTs callbacks. */
typedef struct
{
    ble_gatts_read_req_handler_t       read_req_handler;        /**< Read attribute value request handler. */
    ble_gatts_write_req_handler_t      write_req_handler;       /**< Write attribute value request handler. */
    ble_gatts_ntf_ind_handler_t        ntf_ind_handler;         /**< Notification or indication handler. */
    ble_gatts_cccd_recovery_handler_t  cccd_recovery_handler;   /**< CCCD value recovery handler. */
    ble_gatts_on_connect_handler_t     on_conn_handler;
    ble_gatts_on_disconnect_handler_t  on_disconn_handler;
} ble_gatts_handler_t;
/** @} */

/**
 * @defgroup BLE_GATT_SERVICE_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Add primary/secondary service.
 *
 * @param[in]  is_primary:  Is primary service or not.
 * @param[in]  p_uuid:      Pointer to service UUID.
 * @param[out] p_handle:    Pointer to service handle(will be allocated async after advertising start).
 * 
 * @return Result of gatt service add.
 *****************************************************************************************
 */
sdk_err_t ble_gatts_serv_add(bool is_primary, const ble_att_uuid_t *p_uuid, uint16_t *p_handle);

/**
 *****************************************************************************************
 * @brief Add included service to GATT service.
 *
 * @param[in] p_offset: Pointer to the include service handle.
 * 
 * @return Result of include service add.
 *****************************************************************************************
 */
sdk_err_t ble_gatts_inc_serv_add(uint16_t *p_handle);

/**
 *****************************************************************************************
 * @brief Add characteristic to GATT service.
 *
 * @param[in]  p_att:    Pointer to att info.
 * @param[out] p_handle: Pointer to value handle(will be allocated async after advertising start).
 * 
 * @return Result of gatt characteristic add.
 *****************************************************************************************
 */
sdk_err_t ble_gatts_characteristic_add(ble_gatts_att_t *p_att, uint16_t *p_val_handle);

/**
 *****************************************************************************************
 * @brief Add descriptor to GATT service.
 *
 * @param[in]  p_att:    Pointer to att info.
 * @param[out] p_handle: Pointer to descriptor handle(will be allocated async after advertising start).
 * 
 * @return Result of gatt descriptor add.
 *****************************************************************************************
 */
sdk_err_t ble_gatts_descriptor_add(ble_gatts_att_t *p_att, uint16_t *p_handle);

/**
 *****************************************************************************************
 * @brief Enable service to local DB.
 *
 * @param[in]  p_att_handler: Pointer to attribute request handler.
 *
 * @return Result of publish.
 *****************************************************************************************
 */
sdk_err_t ble_gatts_serv_enable(ble_gatts_handler_t *p_att_handler);

/**
 ****************************************************************************************
 * @brief Reply to an attribute read/write request.
 *
 * @param[in] conn_idx:    Connection index.
 * @param[in] p_att_reply: Pointer to att reply.
 *
 * @return Result of reply.
 ****************************************************************************************
 */
sdk_err_t ble_gatts_att_req_reply(uint8_t conn_idx, ble_gatts_att_req_reply_t *p_att_reply);

/** @} */

#endif

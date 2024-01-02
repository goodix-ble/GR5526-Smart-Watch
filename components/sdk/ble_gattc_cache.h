/**
 ****************************************************************************************
 *
 * @file ble_gattc_cache.h
 *
 * @brief BLE GATTC API
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
 * @addtogroup BLE
 * @{
 */

  /**
 * @addtogroup BLE_GATT Generic Attribute Profile (GATT)
 * @{
 * @brief Definitions and prototypes for the GATT interface.
 */

/**
  @addtogroup BLE_GATTC_CACHE Generic Attribute Profile (GATT) Client Cache
  @{
  @brief Definitions and prototypes for the GATT client cache interfaces.
 */

#ifndef _BLE_GATTC_CACHE_H_
#define _BLE_GATTC_CACHE_H_

#include "ble_error.h"
#include "ble_gatt.h"
#include "ble_att.h"
#include "ble_gapc.h"
#include <stdint.h>
#include <stdbool.h>

/** @addtogroup BLE_GATT_CACHE_DEFINES Defines
 * @{ */
#define MAX_GATT_CACHE_NUM (0x10) /**< Max number of GATT CACHE. */
/** @} */

/** @addtogroup BLE_GATTC_CACHE_ENUMERATIONS Enumerations
 * @{ */
/**
 * @brief GATT Client Cache Attribute type IDs.
 */
typedef enum
{
    BLE_GATTC_CACHE_PRI_SERV,       /**< Primary Service Declaration. */
    BLE_GATTC_CACHE_SEC_SERV,       /**< Secondary Service Declaration. */
    BLE_GATTC_CACHE_INC_SRVC,       /**< Included Service Declaration. */
    BLE_GATTC_CACHE_ATTR_CHAR,      /**< Characteristic Declaration. */
    BLE_GATTC_CACHE_ATTR_DESC,      /**< Characteristic Descriptor. */
} gattc_cache_attr_t;
/** @} */

/** @addtogroup BLE_GATTC_CACHE_STRUCTURES Structures
 * @{ */

/**
 * @brief GATTC Cache service attribute structure.
 */
typedef struct
{
    uint16_t  start_hdl;            /**< Start handle. */
    uint16_t  end_hdl;              /**< End handle. */
    uint8_t   uuid_len;             /**< Service UUID length. */
    uint8_t  *p_uuid;               /**< Service UUID. */
} ble_gattc_cache_service_t;

/**
 * @brief GATTC Cache include attribute structure.
 */
typedef struct
{
    uint16_t  start_hdl;              /**< Start handle. */
    uint16_t  end_hdl;                /**< End handle. */
    uint8_t   uuid_len;               /**< Service UUID length. */
    uint8_t  *p_uuid;                 /**< Service UUID. */
} ble_gattc_cache_include_t;

/**
 * @brief GATTC Cache characteristic attribute structure.
 */
typedef struct
{
    uint8_t   prop;                   /**< Properties. */
    uint16_t  handle_value;           /**< Handle of the Characteristic Value. */
    uint8_t   uuid_len;               /**< Characteristic UUID length. */
    uint8_t  *p_uuid;                 /**< Characteristic UUID. */
} ble_gattc_cache_char_t;

/**
 * @brief GATTC Cache characteristic descriptor attribute structure.
 */
typedef struct
{
    uint8_t   uuid_len;               /**< Descriptor UUID length. */
    uint8_t  *p_uuid;                 /**< Descriptor UUID. */
} ble_gattc_cache_desc_t;

/**
 * @brief GATTC cache attribute information uinon.
 */
union attr_cache_info
{
    ble_gattc_cache_service_t     service_decl;           /**< Service Declaration. */
    ble_gattc_cache_include_t     include_decl;           /**< Include Declaration. */
    ble_gattc_cache_char_t        char_decl;              /**< Characteristic Declaration. */
    ble_gattc_cache_desc_t        char_desc;              /**< Characteristic Descriptor. */
};

/**
 * @brief GATTC cache attribute information structure.
 */
typedef struct
{
    uint16_t    handle;                             /**< Attribute Handle. */
    uint8_t     attr_type;                          /**< Attribute type. See @ref gattc_cache_attr_t. */
    union       attr_cache_info info;               /**< Attribute cache information. */
} attr_cache_info_t;


/**
 * @brief GATT caching list
 */
typedef struct
{
    uint8_t          num;                           /**< Number of bonded device. */
    ble_gap_bdaddr_t items[MAX_GATT_CACHE_NUM];     /**< GATT caching addr info. */
} gatt_cache_list_t;
/** @} */

/** @addtogroup BLE_GATTC_CACHE_FUNCTIONS Functions
 * @{ */

/**
 ****************************************************************************************
 * @brief GATTC cache feature enable.
 *
 * @note Once enabling peer cache feature and caching data finished, the event @ref BLE_GATTC_EVT_CACHE_UPDATE will be called.
 *
 * @param[in] conn_idx:     Current connection index.
 *
 * @retval SDK_SUCCESS: Successfully enable cache feature.
 * @retval BLE_SDK_ERR_BAD_PARAM: Invalid parameter(s) supplied.
 ****************************************************************************************
 */
uint16_t ble_gattc_cache_enable(uint8_t conn_idx);

/**
 ****************************************************************************************
 * @brief GATTC cache feature disable.
 *
 * @note This function will delete caching data saved in nvds and disable cache feature.
 *
 * @param[in] conn_idx:     Current connection index.
 *
 * @retval SDK_SUCCESS: Successfully enable cache feature.
 * @retval BLE_SDK_ERR_BAD_PARAM: Invalid parameter(s) supplied.
 ****************************************************************************************
 */
uint16_t ble_gattc_cache_disable(uint8_t conn_idx);

/**
 ****************************************************************************************
 * @brief GATTC Get cache feature state.
 *
 * @note This function returns cache feature state.
 *
 * @param[in] conn_idx:     Current connection index.
 *
 * @return true if cache feature enable, false otherwise.
 ****************************************************************************************
 */
bool ble_gattc_cache_feat_get(uint8_t conn_idx);

/**
 ****************************************************************************************
 * @brief GATTC cache data get.
 *
 * @note User should get cache data after gattc_cb_fun_t::app_gattc_cache_update_cb called.
 *       Real cache data length will be returned whether user provide enough buf or not.
 *       If cache data is NULL, p_cache_count will return cache data length.
 *
 *
 * @param[in]     conn_idx:      Current connection index.
 * @param[in]     p_cache_data:  The attribute cache buf.
 * @param[out]    p_cache_count: The count of attribute cache buf.
 *
 * @retval SDK_SUCCESS: Successfully get cache attributes info.
 * @retval BLE_SDK_ERR_BAD_PARAM: Invalid parameter(s) supplied.
 * @retval BLE_SDK_ERR_CACHE_NOT_ENABLE: Cache feature is not enabled.
 * @retval BLE_SDK_ERR_BUSY: Caching data operation is not finished.
 * @retval BLE_SDK_ERR_BUF_LEN_NOT_ENOUGH: The cache buf lenth is not enough.
  ****************************************************************************************
 */
uint16_t ble_gattc_cache_get(uint8_t conn_idx, attr_cache_info_t *p_cache_data, uint16_t *p_cache_count);

/**
 ****************************************************************************************
 * @brief Get the content of the whole GATT caching list.
 *
 * @param[in] p_cache_list: Pointer to the output caching list.
 *
 * @retval ::SDK_SUCCESS: Operation is successful.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gattc_cache_list_get(gatt_cache_list_t *p_cache_list);


/**
 ****************************************************************************************
 * @brief GATTC cache date delete on disconnection state.
 *
 * @param[in] p_peer_bd_addr:    Identity address of peer device.
 *
 * @retval SDK_SUCCESS: Successfully delete cache in nvds.
 * @retval BLE_SDK_ERR_BAD_PARAM: Invalid parameter(s) supplied.
 * @retval BLE_SDK_ERR_LIST_ITEM_NOT_FOUND: Item not found in list.
 ****************************************************************************************
 */
uint16_t ble_gattc_cache_delete(ble_gap_bdaddr_t *p_peer_bd_addr);
/** @} */

#endif

/** @} */

/** @} */
/** @} */


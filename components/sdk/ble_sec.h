/**
 ****************************************************************************************
 *
 * @file ble_sec.h
 *
 * @brief BLE SEC API
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
  @addtogroup BLE_SEC Security Manager(SM)
  @{
  @brief Definitions and prototypes for the BLE_SEC interface.
 */

#ifndef __BLE_SEC_H__
#define __BLE_SEC_H__

#include "ble_error.h"
#include <stdbool.h>

/**@addtogroup BLE_SM_DEFINES Defines 
 * @{ 
 */
/**@defgroup BLE_SEC_AUTH_FLAG  SEC Auth Flag
* @{ 
*/
#define BLE_SEC_AUTH_NONE               0                 /**< No auth requirement. */
#define BLE_SEC_AUTH_BOND              (1 << 0)           /**< Bond flag. */
#define BLE_SEC_AUTH_MITM              (1 << 2)           /**< MITM flag. */
#define BLE_SEC_AUTH_SEC_CON           (1 << 3)           /**< Security connection flag. */
#define BLE_SEC_AUTH_KEY_PRESS_NOTIFY  (1 << 4)           /**< Key press notify flag. */
#define BLE_SEC_AUTH_ALL               (AUTH_BOND | AUTH_MITM | AUTH_SEC_CON | AUTH_KEY_PRESS_NOTIFY)  /**< All authentication flags are on. */
/**@} */

/**@defgroup BLE_SEC_KEY_DIST_FLAG  SEC Key Distribution Flag
* @{ 
*/
#define BLE_SEC_KDIST_NONE      0            /**< No key needs to be distributed. */
#define BLE_SEC_KDIST_ENCKEY   (1 << 0)      /**< Distribute encryption and master identification info. */
#define BLE_SEC_KDIST_IDKEY    (1 << 1)      /**< Distribute identity and address info. */
#define BLE_SEC_KDIST_SIGNKEY  (1 << 2)      /**< Distribute signing info. */
#define BLE_SEC_KDIST_ALL      (BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY | BLE_SEC_KDIST_SIGNKEY)  /**< Distribute all info. */

/**@} */
/**@} */

/**@addtogroup BLE_SEC_ENUMERATIONS Enumerations
 * @{ */
/**@brief SEC IO Capability. */
typedef enum
{
    BLE_SEC_IO_DISPLAY_ONLY       = 0x00,       /**< Display only. */
    BLE_SEC_IO_DISPLAY_YES_NO     = 0x01,       /**< Display and input yes or no. */
    BLE_SEC_IO_KEYBOARD_ONLY      = 0x02,       /**< Keyboard only. */
    BLE_SEC_IO_NO_INPUT_NO_OUTPUT = 0x03,       /**< No input and no output. */
    BLE_SEC_IO_KEYBOARD_DISPLAY   = 0x04        /**< Keyboard and display. */
} ble_sec_io_cap_t;

/**@brief SEC Encryption Request Type.
  *@note These types indicate some operations need to interact with app during pair process.
 */
typedef enum
{
    BLE_SEC_PAIR_REQ,       /**< Pair request. Apps need to decide whether to accept this request. */
    BLE_SEC_TK_REQ,         /**< TK request. Apps need to set the TK value. */
    BLE_SEC_OOB_REQ,        /**< OOB request. Apps need to set the OOB value. */
    BLE_SEC_NC_REQ          /**< Number comparison request. Apps need to check if it is the same number displayed in Master and Slave. */
} ble_sec_enc_req_type_t;

/**@brief SEC Key Press Notify.  */
typedef enum
{
    BLE_SEC_KEY_PRESS_STARTED   = 0x00,        /**< Passkey entry started. */
    BLE_SEC_KEY_PRESS_ENTERED   = 0x01,        /**< Passkey digit entered. */
    BLE_SEC_KEY_PRESS_ERASED    = 0x02,        /**< Passkey digit erased. */
    BLE_SEC_KEY_PRESS_CLEARED   = 0x03,        /**< Passkey cleared. */
    BLE_SEC_KEY_PRESS_COMPLETED = 0x04         /**< Passkey entry completed. */
} ble_sec_keypress_notify_t;

/**@brief SEC mode and level.  */
typedef enum
{
    BLE_SEC_MODE1_LEVEL1 = 0x00,    /**< No security is needed. */
    BLE_SEC_MODE1_LEVEL2 = 0x01,    /**< Encrypted link is required. Unnecessary: MITM and SC. */
    BLE_SEC_MODE1_LEVEL3 = 0x02,    /**< Encrypted link is required. Necessary: MITM; unnecessary: SC. */
    BLE_SEC_MODE1_LEVEL4 = 0x03,    /**< Encrypted link is required. Necessary: MITM and SC. */
    BLE_SEC_MODE2_LEVEL1 = 0x04,    /**< Data signing is required. Unnecessary: MITM and SC. */
    BLE_SEC_MODE2_LEVEL2 = 0x05,    /**< Data signing is required. Necessary: MITM; unnecessary: SC. */
} ble_sec_mode_level_t;

/**@brief SEC TK type. */
typedef enum
{
    BLE_SEC_TK_OOB = 0x00,        /**<TK got from OOB (out of band) method. */
    BLE_SEC_TK_DISPLAY,           /**<TK generated and shall be displayed by local device. */
    BLE_SEC_TK_KEY_ENTRY          /**<TK shall be entered by user using device keyboard. */
} ble_sec_tk_type_t;

/**@brief Key missing reason. */
typedef enum
{
    BLE_SEC_BOND_INFO_LOAD_FAILED = 0x00,      /**<Bond information load failed. */
    BLE_SEC_LTK_VALID_MASK_ERR,                /**<LTK valid mask flag is false. */
    BLE_SEC_EDIV_RAND_VALUE_ERR                /**<Ediv and rand value not match. */
} ble_sec_key_missing_reason_t;
/** @} */

/**@addtogroup BLE_SEC_STRUCTURES Structures
 * @{ */
/**@brief SEC Parameter. */
typedef struct
{
    ble_sec_mode_level_t level;         /**< Set the minimum security level of the device, see @ref ble_sec_mode_level_t. */
    ble_sec_io_cap_t     io_cap;        /**< Set the IO capability, see @ref ble_sec_io_cap_t. */
    bool             oob;           /**< Indicate whether OOB is supported. */
    uint8_t          auth;          /**< Set the auth, see @ref BLE_SEC_AUTH_FLAG. */
    uint8_t          key_size;      /**< Indicate the supported maximum LTK size (range: 7-16). */
    uint8_t          ikey_dist;     /**< Set the initial key distribution, see @ref BLE_SEC_KEY_DIST_FLAG. */
    uint8_t          rkey_dist;     /**< Set the response key distribution, see @ref BLE_SEC_KEY_DIST_FLAG. */
} ble_sec_param_t;

/**@brief TK value. */
typedef struct
{
    uint8_t key[16];          /**< TK value. */
} ble_sec_tk_t;

/**@brief SEC OOB value. */
typedef struct
{
    uint8_t conf[16];        /**< Confirm value. */
    uint8_t rand[16];        /**< Random value. */
} ble_sec_oob_t;

/**@brief SEC Confirm encryption data. */
typedef union
{
    ble_sec_tk_t  tk;           /**< TK value, see @ref ble_sec_tk_t. */
    ble_sec_oob_t oob;          /**< OOB value, see @ref ble_sec_oob_t. */
} ble_sec_cfm_enc_data_t;

/**@brief SEC Confirm encryption. */
typedef struct
{
    ble_sec_enc_req_type_t req_type;         /**< Request type, see @ref ble_sec_enc_req_type_t. */
    bool               accept;               /**< Indicate whether to accept the request. */
    ble_sec_cfm_enc_data_t data;             /**< SEC Confirm encryption data, see @ref ble_sec_cfm_enc_data_t. */
} ble_sec_cfm_enc_t;

/**@brief SEC number comparison value. */
typedef struct
{
    uint8_t value[4];       /**< Number comparison value (000000~999999). */
} ble_sec_nc_t;

/**@brief SEC encryption request data. */
typedef union
{
    ble_sec_tk_type_t tk_type;      /**<TK type, see @ref ble_sec_tk_type_t. */
    ble_sec_oob_t     oob_data;     /**<OOB data, see @ref ble_sec_oob_t. */
    ble_sec_nc_t      nc_data;      /**<Number comparison data, see @ref ble_sec_nc_t. */
} ble_sec_enc_req_data_t;

 /**@brief Link Encrypte Request event for @ref BLE_SEC_EVT_LINK_ENC_REQUEST. */
typedef struct
{
    ble_sec_enc_req_type_t req_type;        /**< Indicate the request type, @ref ble_sec_enc_req_type_t. */
    ble_sec_enc_req_data_t data;            /**< SEC encryption request data, @ref ble_sec_enc_req_data_t. */
} ble_sec_evt_enc_req_t;

 /**@brief Link Encrypted event for @ref BLE_SEC_EVT_LINK_ENCRYPTED. */
typedef struct
{
    uint8_t         auth;       /**< Auth type. */
} ble_sec_evt_enc_ind_t;

/**@brief Key Press Notify event for @ref BLE_SEC_EVT_KEY_PRESS_NTF. */
typedef struct
{
    ble_sec_keypress_notify_t  notify_type;    /**< key Press Notify type. */
} ble_sec_evt_keypress_notify_t;

/**@brief Key Missing event for @ref BLE_SEC_EVT_KEY_MISSING. */
typedef struct
{
    ble_sec_key_missing_reason_t  reason;       /**< Keymissing reason. */
} ble_sec_evt_key_missing_t;

/**@brief BLE Security event structure. */
typedef struct
{
    uint8_t  index;                                           /**< Index of connection. */
    union
    {
        ble_sec_evt_enc_req_t           enc_req;              /**< Link Encrypte Request event. */
        ble_sec_evt_enc_ind_t           enc_ind;              /**< Link Encrypted event. */
        ble_sec_evt_keypress_notify_t   keypress_ntf;         /**< Key Press Notify event. */
        ble_sec_evt_key_missing_t       key_missing;          /**< Key Missing event. */
    } params;                                                 /**< The parameters of sec events. */
} ble_sec_evt_t;                                              /**< Event Parameters. */

/** @} */

/** @addtogroup BLE_SEC_FUNCTIONS Functions
 * @{ */
/**
 ****************************************************************************************
 * @brief Set security parameter.
 *
* @param[in] p_sec_param: Pointer to the security parameter structure, @ref ble_sec_param_t.
 *
 * @retval ::SDK_SUCCESS: The security parameter is successfully set to the BLE stack.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 ****************************************************************************************
 */
uint16_t ble_sec_params_set(ble_sec_param_t *p_sec_param);

/**
 ****************************************************************************************
 * @brief Start security encryption, this interface is used by both slave and master
 *
 * @note If the local device role is master, it will check that if the peer device is bonded firstly. If the peer device is bonded, 
 *  the stack will encrypt the link directly, otherwise the stack will send a pair request to the peer device.
 *
 * @note If the local device role is slave, the stack will send a security request to the peer device.
 *
 * @param[in] conn_idx: ACL connection index, the first ACL connection index is 0, and increased one by one.
 *
 * @retval ::SDK_SUCCESS: The security encryption is successfully set to the BLE stack.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_sec_enc_start(uint8_t conn_idx);

/**
 ****************************************************************************************
 * @brief Send the encrypt confirm information
 *
 * @param[in] conn_idx:  ACL connection index, the first ACL connection index is 0, and increased one by one.
 * @param[in] p_cfm_enc: Pointer to the confirm encryption structure, see @ref ble_sec_cfm_enc_t.
 *
 * @retval ::SDK_SUCCESS: The confirm encryption is successfully set to the BLE stack.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_sec_enc_cfm(uint8_t conn_idx, const ble_sec_cfm_enc_t *p_cfm_enc);

/**
 ****************************************************************************************
 * @brief Send key press notify
 *
 * @param[in] conn_idx:    ACL connection index. The first ACL connection index is 0, and the index will be increased one by one.
 * @param[in] notify_type: Key press notify type, see @ref ble_sec_keypress_notify_t.
 *
 * @retval ::SDK_SUCCESS: The key press notify type is successfully set to the BLE stack.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_sec_keypress_notify_send(uint8_t conn_idx, uint8_t notify_type);
/** @} */

#endif

/** @} */
/** @} */

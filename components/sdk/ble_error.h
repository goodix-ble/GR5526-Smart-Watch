/**
 ****************************************************************************************
 *
 * @file ble_error.h
 *
 * @brief File that contains error codes.
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
  @addtogroup BLE_ERROR Error Codes
  @{
  @brief  File that contains error codes.
 */

#ifndef _BLE_SDK_ERROR_H_
#define _BLE_SDK_ERROR_H_

#include <stdint.h>

/**@addtogroup BLE_ERROR_CODES Defines 
 * @{ 
 */

/**@defgroup SDK_ERROR_CODES  SDK Specific Error Codes
* @{ 
*/
#define SDK_SUCCESS                                 0x0000      /**< Successful. */
#define SDK_ERR_INVALID_PARAM                       0x0001      /**< Invalid parameter supplied. */
#define SDK_ERR_POINTER_NULL                        0x0002      /**< Invalid pointer supplied. */
#define SDK_ERR_INVALID_CONN_IDX                    0x0003      /**< Invalid connection index supplied. */
#define SDK_ERR_INVALID_HANDLE                      0x0004      /**< Invalid handle supplied. */
#define SDK_ERR_PROFILE_COUNT                       0x0005      /**< Maximum SDK profile count exceeded. */
#define SDK_ERR_BUSY                                0x0006      /**< SDK is busy internally. */
#define SDK_ERR_TIMER_INSUFFICIENT                  0x0007      /**< Timer is insufficient. */
#define SDK_ERR_NVDS_NOT_INIT                       0x0008      /**< NVDS is not initiated. */
#define SDK_ERR_LIST_ITEM_NOT_FOUND                 0x0009      /**< Item not found in list. */
#define SDK_ERR_LIST_ITEM_ALREADY_EXISTED           0x000a      /**< Item already exists in list. */
#define SDK_ERR_LIST_FULL                           0x000b      /**< List is full. */
#define SDK_ERR_SDK_INTERNAL                        0x000c      /**< Internal SDK error. */
#define SDK_ERR_INVALID_BUFF_LENGTH                 0x000d      /**< The buffer's length is not enough. */
#define SDK_ERR_INVALID_DATA_LENGTH                 0x000e      /**< Invalid data length supplied. */
#define SDK_ERR_DISALLOWED                          0x000f      /**< Operation is disallowed. */
#define SDK_ERR_NO_RESOURCES                        0x0010      /**< Not enough resources for operation. */
#define SDK_ERR_REQ_NOT_SUPPORTED                   0x0011      /**< Request not supported. */
#define SDK_ERR_INVALID_OFFSET                      0x0012      /**< Offset exceeds the current attribute value length. */
#define SDK_ERR_INVALID_ATT_VAL_LEN                 0x0013      /**< Invalid length of the attribute value. */
#define SDK_ERR_INVALID_PERM                        0x0014      /**< Permission set in service/attribute is invalid. */
#define SDK_ERR_INVALID_ADV_IDX                     0x0015      /**< Invalid advertising index supplied. */
#define SDK_ERR_INVALID_ADV_DATA_TYPE               0x0016      /**< Invalid advertising data type supplied. */
#define SDK_ERR_INVALID_PSM_NUM                     0x0017      /**< Invalid PSM number. */
#define SDK_ERR_INVALID_PSM_ALREADY_REGISTERED      0x0018      /**< The PSM number has been registered. */
#define SDK_ERR_INVALID_PSM_EXCEEDED_MAX_PSM_NUM    0x0019      /**< The maximum PSM number limit is exceeded. */
#define SDK_ERR_NTF_DISABLED                        0x001A      /**< Notification is NOT enabled. */
#define SDK_ERR_IND_DISABLED                        0x001B      /**< Indication is NOT enabled. */
#define SDK_ERR_DISCONNECTED                        0x001C      /**< Disconnection occurs. */
#define SDK_ERR_INVALID_ADDRESS                     0x001D      /**< Invalid address supplied. */
#define SDK_ERR_FEATURE_NOT_ENABLE                  0x001E      /**< Feature not enable in ble feature config. */

#define SDK_ERR_INVALID_ADV_INTERVAL                0x001F      /**< Invalid advertising interval supplied. */
#define SDK_ERR_INVALID_DISVCOVERY_MODE             0x0020      /**< Invalid discovery mode supplied. */
#define SDK_ERR_INVALID_ADV_PARAM                   0x0021      /**< Invalid advertising parameters supplied. */
#define SDK_ERR_INVALID_ADV_PEER_ADDR               0x0022      /**< Invalid peer address supplied. */
#define SDK_ERR_ADV_DATA_NOT_SET                    0x0023      /**< Legacy advertising data not set. */
#define SDK_ERR_PER_ADV_DATA_NOT_SET                0x0024      /**< Periodic advertising data not set. */
#define SDK_ERR_EXT_SCAN_RSP_DATA_NOT_SET           0x0025      /**< Extended scan response data not set. */
#define SDK_ERR_INVALID_DURATION_PARAM              0x0026      /**< Invalid duration parameter supplied. */
#define SDK_ERR_INVALID_PER_SYNC_IDX                0x0027      /**< Invalid periodic synchronization index supplied. */
#define SDK_ERR_INVALID_CID                         0x0028      /**< Invalid CID supplied. */
#define SDK_ERR_INVALID_CHL_NUM                     0x0029      /**< Invalid channel number supplied. */
#define SDK_ERR_NOT_ENOUGH_CREDITS                  0x002A      /**< Not enough credits. */
#define SDK_ERR_REPEAT_CID                          0x002B      /**< Invalid repeat CID. */
#define SDK_ERR_CACHE_NOT_ENABLE                    0x002C      /**< Cache feature is not enabled. */
#define SDK_ERR_CACHE_INVALID                       0x002D      /**< Cache data is invalid. */

#define SDK_ERR_APP_ERROR                           0x0080      /**< Application error. */
/**@} */

/**@defgroup BLE_STACK_ERROR_CODES  BLE Stack specific error codes
* @{ 
*/

#define BLE_SUCCESS                                 0x00        /**< Operation is Successful. */
/**@brief ATT Specific Error. */
#define BLE_ATT_ERR_INVALID_HANDLE                  0x01        /**< The given attribute handle was not valid on this server. */
#define BLE_ATT_ERR_READ_NOT_PERMITTED              0x02        /**< The attribute cannot be read. */
#define BLE_ATT_ERR_WRITE_NOT_PERMITTED             0x03        /**< The attribute cannot be written. */
#define BLE_ATT_ERR_INVALID_PDU                     0x04        /**< The attribute PDU was invalid. */
#define BLE_ATT_ERR_INSUFF_AUTHEN                   0x05        /**< The attribute requires authentication before it can be read or written. */
#define BLE_ATT_ERR_REQUEST_NOT_SUPPORTED           0x06        /**< Attribute server does not support the request received from the client. */
#define BLE_ATT_ERR_INVALID_OFFSET                  0x07        /**< Offset specified was past the end of the attribute. */
#define BLE_ATT_ERR_INSUFF_AUTHOR                   0x08        /**< The attribute requires authorization before it can be read or written. */
#define BLE_ATT_ERR_PREPARE_QUEUE_FULL              0x09        /**< Too many prepare writes have been queued. */
#define BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND             0x0A        /**< No attribute found within the given attribute handle range. */
#define BLE_ATT_ERR_ATTRIBUTE_NOT_LONG              0x0B        /**< The attribute cannot be read using the Read Blob Request. */
#define BLE_ATT_ERR_INSUFF_ENC_KEY_SIZE             0x0C        /**< The Encryption Key Size used for encrypting this link is insufficient. */
#define BLE_ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN       0x0D        /**< The attribute value length is invalid for the operation. */
#define BLE_ATT_ERR_UNLIKELY_ERR                    0x0E        /**< The attribute request has encountered an unlikely error, so the request could not be completed as requested. */
#define BLE_ATT_ERR_INSUFF_ENC                      0x0F        /**< The attribute requires encryption before it can be read or written. */
#define BLE_ATT_ERR_UNSUPP_GRP_TYPE                 0x10        /**< The attribute type is not a supported grouping attribute as defined by a higher layer specification. */
#define BLE_ATT_ERR_INSUFF_RESOURCE                 0x11        /**< Insufficient resources to complete the request. */
#define BLE_ATT_ERR_DB_OUT_OF_SYNC                  0x12        /**< The server requests the client to rediscover the database. */
#define BLE_ATT_ERR_VALUE_NOT_ALLOWED               0x13        /**< The attribute parameter value was not allowed. */

/**@brief L2CAP Specific Error. */
#define BLE_L2C_ENH_CB_RECONFIG_INVALID_MTU         0x2C        /**< Reconfiguration failed - reduction in size of MTU not allowed. */
#define BLE_L2C_ENH_CB_RECONFIG_INVALID_MPS         0x2D        /**< Reconfiguration failed - reduction in size of MPS not allowed for more than one channel at a time. */
#define BLE_L2C_ENH_CB_RECONFIG_INVALID_CID         0x2E        /**< Reconfiguration failed - one or more Destination CIDs invalid. */
#define BLE_L2C_ENH_CB_RECONFIG_UNACCEPT_PARAM      0x2F        /**< Reconfiguration failed - other unacceptable parameters. */
#define BLE_L2C_ERR_CONNECTION_LOST                 0x30        /**< Message cannot be sent because connection is lost (disconnected). */
#define BLE_L2C_ERR_INVALID_MTU_EXCEED              0x31        /**< Invalid PDU length exceeds MTU. */
#define BLE_L2C_ERR_INVALID_MPS_EXCEED              0x32        /**< Invalid PDU length exceeds MPS. */
#define BLE_L2C_ERR_INVALID_CID                     0x33        /**< Invalid Channel ID. */
#define BLE_L2C_ERR_INVALID_PDU                     0x34        /**< Invalid PDU. */
#define BLE_L2C_ERR_NO_RES_AVAIL                    0x35        /**< Connection refused because no resources are available. */
#define BLE_L2C_ERR_INSUFF_AUTHEN                   0x36        /**< Connection refused because of insufficient authentication. */
#define BLE_L2C_ERR_INSUFF_AUTHOR                   0x37        /**< Connection refused because of insufficient authorization. */
#define BLE_L2C_ERR_INSUFF_ENC_KEY_SIZE             0x38        /**< Connection refused because of insufficient encryption key size. */
#define BLE_L2C_ERR_INSUFF_ENC                      0x39        /**< Connection refused because of insufficient encryption. */
#define BLE_L2C_ERR_LEPSM_NOT_SUPP                  0x3A        /**< Connection refused because LE_PSM is not supported. */
#define BLE_L2C_ERR_INSUFF_CREDIT                   0x3B        /**< No more credit. */
#define BLE_L2C_ERR_NOT_UNDERSTOOD                  0x3C        /**< Command not understood by peer device. */
#define BLE_L2C_ERR_CREDIT_ERROR                    0x3D        /**< Credit error: invalid number of credit received. */
#define BLE_L2C_ERR_CID_ALREADY_ALLOC               0x3E        /**< Channel identifier already allocated. */
#define BLE_L2C_ERR_UNKNOWN_PDU                     0x3F        /**< Unknown pdu. */

/**@brief GAP Specific Error. */
#define BLE_GAP_ERR_INVALID_PARAM                   0x40        /**< Invalid parameters set. */
#define BLE_GAP_ERR_PROTOCOL_PROBLEM                0x41        /**< Problem with protocol exchange, resulting in unexpected responses. */
#define BLE_GAP_ERR_NOT_SUPPORTED                   0x42        /**< Request not supported by software configuration. */
#define BLE_GAP_ERR_COMMAND_DISALLOWED              0x43        /**< Request not allowed in current state. */
#define BLE_GAP_ERR_CANCELED                        0x44        /**< Requested operation canceled. */
#define BLE_GAP_ERR_TIMEOUT                         0x45        /**< Requested operation timeout. */
#define BLE_GAP_ERR_DISCONNECTED                    0x46        /**< Link connection is lost during operation. */
#define BLE_GAP_ERR_NOT_FOUND                       0x47        /**< Search algorithm finished, but no result found. */
#define BLE_GAP_ERR_REJECTED                        0x48        /**< Request rejected by peer device. */
#define BLE_GAP_ERR_PRIVACY_CFG_PB                  0x49        /**< Problem with privacy configuration. */
#define BLE_GAP_ERR_ADV_DATA_INVALID                0x4A        /**< Duplicate or invalid advertising data. */
#define BLE_GAP_ERR_INSUFF_RESOURCES                0x4B        /**< Insufficient resources. */
#define BLE_GAP_ERR_UNEXPECTED                      0x4C        /**< Unexpected error. */
#define BLE_GAP_ERR_MISMATCH                        0x4D        /**< Feature mismatch. */

/**@brief GATT Specific Error. */
#define BLE_GATT_ERR_INVALID_ATT_LEN                0x50        /**< Problem with ATTC protocol response. */
#define BLE_GATT_ERR_INVALID_TYPE_IN_SVC_SEARCH     0x51        /**< Error in service search. */
#define BLE_GATT_ERR_WRITE                          0x52        /**< Invalid write data. */
#define BLE_GATT_ERR_SIGNED_WRITE                   0x53        /**< Signed write error. */
#define BLE_GATT_ERR_ATTRIBUTE_CLIENT_MISSING       0x54        /**< No attribute client defined. */
#define BLE_GATT_ERR_ATTRIBUTE_SERVER_MISSING       0x55        /**< No attribute server defined. */
#define BLE_GATT_ERR_INVALID_PERM                   0x56        /**< Permission set in service/attribute is invalid. */
#define BLE_GATT_ERR_BROWSE_NO_ANY_MORE             0x57        /**< GATT browses no any more contents. */
#define BLE_GATT_ERR_CACHE_UPDATING                 0x58        /**< GATT Cache in updating process. */
#define BLE_GATT_ERR_CACHE_FINISH                   0x59        /**< GATT Cache is updated or checked. */

/**@brief SEC Specific Error. */
#define BLE_SEC_ERR_PASSKEY_ENTRY_FAIL              0x61        /**< The user input of passkey failed. */
#define BLE_SEC_ERR_OOB_NOT_AVAILBL                 0x62        /**< The OOB data is not available. */
#define BLE_SEC_ERR_AUTH_REQ                        0x63        /**< The pairing procedure cannot be performed as authentication requirements cannot be met due to IO incapability of one or both devices. */
#define BLE_SEC_ERR_CONFIRM_VAL_FAIL                0x64        /**< The confirm value does not match the calculated compare value. */
#define BLE_SEC_ERR_PAIRING_NOT_SUPPORT             0x65        /**< Pairing is not supported by the device. */
#define BLE_SEC_ERR_ENCRPT_KEY_SIZE                 0x66        /**< The resultant encryption key size is insufficient for the security requirements of this device. */
#define BLE_SEC_ERR_COMMAND_NOT_SUPPORT             0x67        /**< The SMP command received is not supported on this device. */
#define BLE_SEC_ERR_UNSPECIFIED                     0x68        /**< Pairing failed due to an unspecified reason. */
#define BLE_SEC_ERR_REPEAT_ATTEMPT                  0x69        /**< Pairing or authentication procedure is disallowed because too little time has elapsed since last pairing request or security request. */
#define BLE_SEC_ERR_INVALID_PARAM                   0x6A        /**< The Invalid Parameters error code indicates that the command length is invalid or that a parameter is outside of the specified range. */
#define BLE_SEC_ERR_DHKEY_CHECK_FAIL                0x6B        /**< Indicate to the remote device that the DHKey Check value received doesn't  match the one calculated by the local device. */
#define BLE_SEC_ERR_NUM_CMP_FAIL                    0x6C        /**< Indicate that the confirm values in the numeric comparison protocol do not match. */
#define BLE_SEC_ERR_BR_EDR_IN_PROGRESS              0x6D        /**< Indicate that the pairing over the LE transport failed due to a Pairing Request sent over the BR/EDR transport in process. */
#define BLE_SEC_ERR_KEY_DRIV_GEN_NOT_ALLOW          0x6E        /**< Indicate that the BR/EDR Link Key generated on the BR/EDR transport cannot be used to derive and distribute keys for the LE transport. */
#define BLE_SEC_ERR_LTK_MISSING                     0x6F        /**< Indicate the LTK of peer devices missing. */

/**@brief LL Specific Error. */
#define BLE_LL_ERR_UNKNOWN_HCI_COMMAND              0x91        /**< Unknown HCI Command. */
#define BLE_LL_ERR_UNKNOWN_CONNECTION_ID            0x92        /**< Unknown Connection Identifier. */
#define BLE_LL_ERR_HARDWARE_FAILURE                 0x93        /**< Hardware Failure. */
#define BLE_LL_ERR_PAGE_TIMEOUT                     0x94        /**< BT Page Timeout. */
#define BLE_LL_ERR_AUTH_FAILURE                     0x95        /**< Authentication failure. */
#define BLE_LL_ERR_PIN_MISSING                      0x96        /**< Pin code missing. */
#define BLE_LL_ERR_MEMORY_CAPA_EXCEED               0x97        /**< Memory capacity exceeded. */
#define BLE_LL_ERR_CON_TIMEOUT                      0x98        /**< Connection Timeout. */
#define BLE_LL_ERR_CON_LIMIT_EXCEED                 0x99        /**< Connection limit Exceed. */
#define BLE_LL_ERR_SYNC_CON_LIMIT_DEV_EXCEED        0x9A        /**< Synchronous Connection limit exceeded. */
#define BLE_LL_ERR_ACL_CON_EXISTS                   0x9B        /**< ACL Connection exits. */
#define BLE_LL_ERR_COMMAND_DISALLOWED               0x9C        /**< Command Disallowed. */
#define BLE_LL_ERR_CONN_REJ_LIMITED_RESOURCES       0x9D        /**< Connection rejected due to limited resources. */
#define BLE_LL_ERR_CONN_REJ_SECURITY_REASONS        0x9E        /**< Connection rejected due to insecurity issues. */
#define BLE_LL_ERR_CONN_REJ_UNACCEPTABLE_BDADDR     0x9F        /**< Connection rejected due to unacceptable BD Addr. */
#define BLE_LL_ERR_CONN_ACCEPT_TIMEOUT_EXCEED       0xA0        /**< Connection rejected due to Accept connection timeout. */
#define BLE_LL_ERR_UNSUPPORTED                      0xA1        /**< Not Supported. */
#define BLE_LL_ERR_INVALID_HCI_PARAM                0xA2        /**< Invalid parameters. */
#define BLE_LL_ERR_REMOTE_USER_TERM_CON             0xA3        /**< Remote user terminates connection. */
#define BLE_LL_ERR_REMOTE_DEV_TERM_LOW_RESOURCES    0xA4        /**< Remote device loses connection due to low resources. */
#define BLE_LL_ERR_REMOTE_DEV_POWER_OFF             0xA5        /**< Remote device loses connection due to power failure. */
#define BLE_LL_ERR_CON_TERM_BY_LOCAL_HOST           0xA6        /**< Connection terminated by local host. */
#define BLE_LL_ERR_REPEATED_ATTEMPTS                0xA7        /**< Repeated attempts. */
#define BLE_LL_ERR_PAIRING_NOT_ALLOWED              0xA8        /**< Pairing not allowed. */
#define BLE_LL_ERR_UNKNOWN_LMP_PDU                  0xA9        /**< Unknown PDU Error. */
#define BLE_LL_ERR_UNSUPPORTED_REMOTE_FEATURE       0xAA        /**< Unsupported remote feature. */
#define BLE_LL_ERR_SCO_OFFSET_REJECTED              0xAB        /**< SCO Offset rejected. */
#define BLE_LL_ERR_SCO_INTERVAL_REJECTED            0xAC        /**< SCO Interval Rejected. */
#define BLE_LL_ERR_SCO_AIR_MODE_REJECTED            0xAD        /**< SCO air mode Rejected. */
#define BLE_LL_ERR_INVALID_LMP_PARAM                0xAE        /**< Invalid LMP parameters. */
#define BLE_LL_ERR_UNSPECIFIED_ERROR                0xAF        /**< Unspecified error. */
#define BLE_LL_ERR_UNSUPPORTED_LMP_PARAM_VALUE      0xB0        /**< Unsupported LMP Parameter value. */
#define BLE_LL_ERR_ROLE_CHANGE_NOT_ALLOWED          0xB1        /**< Role Change not allowed. */
#define BLE_LL_ERR_LMP_RSP_TIMEOUT                  0xB2        /**< LMP Response timeout. */
#define BLE_LL_ERR_LMP_COLLISION                    0xB3        /**< LMP Collision. */
#define BLE_LL_ERR_LMP_PDU_NOT_ALLOWED              0xB4        /**< LMP PDU not allowed. */
#define BLE_LL_ERR_ENC_MODE_NOT_ACCEPT              0xB5        /**< Encryption mode not accepted. */
#define BLE_LL_ERR_LINK_KEY_CANT_CHANGE             0xB6        /**< Link Key cannot be changed. */
#define BLE_LL_ERR_QOS_NOT_SUPPORTED                0xB7        /**< Quality of Service not supported. */
#define BLE_LL_ERR_INSTANT_PASSED                   0xB8        /**< Error, instant passed. */
#define BLE_LL_ERR_PAIRING_WITH_UNIT_KEY_NOT_SUP    0xB9        /**< Pairing with unit key not supported. */
#define BLE_LL_ERR_DIFF_TRANSACTION_COLLISION       0xBA        /**< Transaction collision. */
#define BLE_LL_ERR_QOS_UNACCEPTABLE_PARAM           0xBC        /**< Quality of Service not supported. */
#define BLE_LL_ERR_QOS_REJECTED                     0xBD        /**< Quality of Service rejected. */
#define BLE_LL_ERR_CHANNEL_CLASS_NOT_SUP            0xBE        /**< Channel class not supported. */
#define BLE_LL_ERR_INSUFFICIENT_SECURITY            0xBF        /**< Insufficient security. */
#define BLE_LL_ERR_PARAM_OUT_OF_MAND_RANGE          0xC0        /**< Parameters out of mandatory range. */
#define BLE_LL_ERR_ROLE_SWITCH_PEND                 0xC2        /**< Role switch pending. */
#define BLE_LL_ERR_RESERVED_SLOT_VIOLATION          0xC4        /**< Reserved slot violation. */
#define BLE_LL_ERR_ROLE_SWITCH_FAIL                 0xC5        /**< Role Switch failed. */
#define BLE_LL_ERR_EIR_TOO_LARGE                    0xC6        /**< Error: EIR too large. */
#define BLE_LL_ERR_SP_NOT_SUPPORTED_HOST            0xC7        /**< Simple pairing not supported by host. */
#define BLE_LL_ERR_HOST_BUSY_PAIRING                0xC8        /**< Host pairing is busy. */
#define BLE_LL_ERR_CONTROLLER_BUSY                  0xCA        /**< Controller is busy. */
#define BLE_LL_ERR_UNACCEPTABLE_CONN_INT            0xCB        /**< Unacceptable connection initialization. */
#define BLE_LL_ERR_ADV_TO                           0xCC        /**< Advertising Timeout. */
#define BLE_LL_ERR_TERMINATED_MIC_FAILURE           0xCD        /**< Connection Terminated due to a MIC failure. */
#define BLE_LL_ERR_CONN_FAILED_TO_BE_EST            0xCE        /**< Connection failed to be established. */
/**@} */

/**
 * @defgroup BLE_ERROR_TYPEDEF Typedefs
 * @{
 */
/**@brief Callback function error parameter type. */
typedef uint8_t   ble_err_t;
/**@brief SDK API result type. */
typedef uint16_t  sdk_err_t;
/**@} */
/**@} */


#endif
/** @} */
/** @} */



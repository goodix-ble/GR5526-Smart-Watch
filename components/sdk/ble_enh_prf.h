/**
 ****************************************************************************************
 *
 * @file ble_enh_prf.h
 *
 * @brief BLE PRF ENHANCED API
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
  @addtogroup BLE_PRF Profile
  @{
  @brief  Definitions and prototypes for the profile interface.
 */

#ifndef __BLE_ENH_PRF_H__
#define __BLE_ENH_PRF_H__

#include "ble_error.h"
#include "ble_att.h"
#include "ble_gatts.h"
#include "ble_gattc.h"
#include "ble_gatt.h"
#include "ble_event.h"

/**
  @addtogroup BLE_PRF_SERVER Profile Server
  @{
  @brief  Definitions and prototypes for Profile Server interface.
 */

/** @addtogroup BLE_PRF_SERVER_STRUCTURES Structures
 * @{ */


/** @} */

/** @addtogroup BLE_PRF_FUNCTIONS Functions
* @{ */

/**
 ****************************************************************************************
 * @brief Add a server profile by providing its detailed information, including manager callback functions and GATT server callback functions.
 *        This API should be called in application initialization function.
 *
 * @param[in] p_gatts_db: Pointer to the database info. See @ref ble_gatts_create_db_t.
 * @param[in] evt_handler: Event handler. See @ref ble_evt_handler_t.
 *
 * @note If there are several profiles which need to be added, this function should be called corresponding times. 
 *
 * @retval ::SDK_SUCCESS: The profile info is recorded successfully, and the database will be created in profile initialization callback function.
 * @retval ::SDK_ERR_POINTER_NULL: The parameter prf_info is NULL, or input parameters that prf_info points to are invalid.
 * @retval ::SDK_ERR_NO_RESOURCES: The profile number is up to the maximum number the system can support.
 ****************************************************************************************
 */
uint16_t ble_gatts_enh_prf_add(ble_gatts_create_db_t *p_gatts_db, ble_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief Respond to an attribute read request. It is used in gatts event handle function @ref BLE_GATTS_EVT_ENH_READ_REQUEST
 *        to send attribute value to stack which is saved in user space.
 *
 * @note The status member ble_gatts_read_cfm_t::status should be set to @ref BLE_ATT_ERR_INSUFF_AUTHOR
 *       to control the authorization of particular read operations of a client.
 *
 * @param[in] conn_idx:        Current connection index.
 * @param[in] cid:             The local channel ID.
 * @param[in] p_param:         Pointer to the parameters filled by profile. See @ref ble_gatts_read_cfm_t.
 *
 * @retval ::SDK_SUCCESS: Send read confirm value to stack successfully.
 * @retval ::SDK_ERR_POINTER_NULL:  Param is NULL.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Conidx is invalid.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gatts_enh_read_cfm(uint8_t conn_idx, uint16_t cid, const ble_gatts_read_cfm_t *p_param);

/**
 ****************************************************************************************
 * @brief Respond to an attribute write request. It is used in gatts event handler function
 * @ref BLE_GATTS_EVT_ENH_WRITE_REQUEST to send write operation status to stack. 
 *
 * @note The status member ble_gatts_write_cfm_t::status should be set to @ref BLE_ATT_ERR_INSUFF_AUTHOR
 *           to control the authorization of particular client's write operation.
 *
 * @param[in] conn_idx:        Current connection index.
 * @param[in] cid:             The local channel ID.
 * @param[in] p_param:         Pointer to the parameters filled by profile. see @ref ble_gatts_write_cfm_t.
 *
 * @retval ::SDK_SUCCESS: Send write confirm status to stack successfully.
 * @retval ::SDK_ERR_POINTER_NULL:  Param is NULL.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Conidx is invalid.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gatts_enh_write_cfm(uint8_t conn_idx, uint16_t cid, const ble_gatts_write_cfm_t *p_param);

/**
 ****************************************************************************************
 * @brief Respond to an attribute prepare write request. It is used in gatts event handler
 *        function @ref BLE_GATTS_EVT_ENH_PREP_WRITE_REQUEST to send prepare write operation status to stack. 
 *
 * @note The status member ble_gatts_prep_write_cfm_t::status should be set to @ref BLE_ATT_ERR_INSUFF_AUTHOR
 *           to control the authorization of particular client's write operation.
 *
 * @param[in] conn_idx:        Current connection index.
 * @param[in] cid:             The local channel ID.
 * @param[in] p_param:         Pointer to the parameters filled by profile. see @ref ble_gatts_prep_write_cfm_t.
 *
 * @retval ::SDK_SUCCESS: Send prepare write confirm status to stack successfully.
 * @retval ::SDK_ERR_POINTER_NULL:  Param is NULL.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Conidx is invalid.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gatts_enh_prepare_write_cfm(uint8_t conn_idx, uint16_t cid, const ble_gatts_prep_write_cfm_t *p_param);

/**
 ****************************************************************************************
 * @brief Send out a notification or an indication. The execution status of sending notification or
 *        indication will be retrieved in the event @ref BLE_GATTS_EVT_ENH_NTF_IND.
 *
 * @note Check whether the relevant Client Characteristic Configuration Descriptor is enabled before using this API.
 *
 * @param[in] conn_idx:       Current connection index.
 * @param[in] cid:            The local channel ID.
 * @param[in] p_param:        Pointer to the parameters filled by profile. see @ref ble_gatts_noti_ind_t.
 * 
 * @retval ::SDK_SUCCESS: Send Notification or Indication event to stack successfully.
 * @retval ::SDK_ERR_POINTER_NULL:  Param is NULL.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Conidx is invalid.
 * @retval ::SDK_ERR_INVALID_PARAM: Type is invalid.
 * @retval ::SDK_ERR_INVALID_HANDLE: Handle not exist in the database.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gatts_enh_noti_ind(uint8_t conn_idx, uint16_t cid, const ble_gatts_noti_ind_t *p_param);


/**
 ****************************************************************************************
 * @brief Send out Multiple Variable Length Notifications. The execution status of sending notification or
 *        indication will be retrieved in the event @ref BLE_GATTS_EVT_ENH_MULT_NTF
 *
 * @note Check whether the relevant Client Characteristic Configuration Descriptor is enabled before using this API.
 *
 * @param[in] conn_idx:       Current connection index.
 * @param[in] cid:            The local channel ID.
 * @param[in] p_param:        Pointer to the parameters filled by profile. see @ref ble_gatts_noti_multiple_t.
 * 
 * @retval ::SDK_SUCCESS: Send Notification or Indication event to stack successfully.
 * @retval ::SDK_ERR_POINTER_NULL:  Param is NULL.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Conidx is invalid.
 * @retval ::SDK_ERR_INVALID_PARAM: Type is invalid.
 * @retval ::SDK_ERR_INVALID_HANDLE: Handle not exist in the database.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gatts_enh_mult_noti(uint8_t conn_idx, uint16_t cid, const ble_gatts_noti_multiple_t *p_param);
/** @} */

/**
  @addtogroup BLE_PRF_CLIENT Profile Client
  @{
  @brief  Definitions and prototypes for Profile Client interface.
 */
/**
  @addtogroup BLE_PRF_CLIENT_FUNCTIONS Functions 
  @{
  @brief  Definitions and prototypes for Profile Client interface.
 */
/**
 ****************************************************************************************
 * @brief Add a client profile by providing its detail information, including manager callback functions and GATT client callback functions.
 *        This API should be called in application initialization function.
 *
 * @param[in]  evt_handler: Event handler, see @ref ble_evt_handler_t.
 * @param[out] p_client_prf_id:  Pointer to the client profile id.
 *
 * @note If there are several profiles which need to be added, this function should be called corresponding times. 
 *
 * @retval ::SDK_SUCCESS: The profile info is recorded successfully, and the profile ENV will be initialized in profile initialization callback function.
 * @retval ::SDK_ERR_POINTER_NULL: The parameter p_client_prf_info or p_client_prf_id is NULL, or input parameters that prf_info points to are invalid.
 * @retval ::SDK_ERR_NO_RESOURCES: The profile number is up to the maximum number the system can support.
 ****************************************************************************************
 */
uint16_t ble_gattc_prf_enh_add(uint8_t *p_client_prf_id, ble_evt_handler_t evt_handler);
/**
 ****************************************************************************************
 * @brief Profile client Browse Specific Primary Service information on remote GATT server.
 *
 * @note This discovery automatically searches for Primary Services, Included Services, Characteristics and Descriptors of each service.
 *       To discover one or more services only, use ble_gattc_primary_services_discover() instead.
 *       This discovery is able to search a specific Primary Service.
 *       If srvc_uuid is NULL, all services are returned.
 *
 * @note Event @ref BLE_GATTC_EVT_SRVC_BROWSE will be called for all attributes of each service found.
 *       After completed service handle range registeration for receiving peer device indication/notification will be executed internally.
 *       Because secondary service can't be browsed, so handle range registeration for receiving peer device indication/notification to this client
 *       profile may be necessary. App can call function ble_gattc_prf_evt_handle_register for registeration, it depends on user app.
 *       If user don't call this function, user shall call ble_gattc_prf_evt_handle_register to register handle range for receiving 
 *       peer device indication/notification in specific client profile callback.
 *       
 *
 * @param[in] prf_id:          Profile id.
 * @param[in] conn_idx:        Current connection index.
 * @param[in] cid:             The local channel ID.
 * @param[in] p_srvc_uuid:     Pointer to Service UUID. If it is NULL, all services will be returned.
 *
 * @retval ::SDK_SUCCESS: Successfully start the Browse Service(s) procedure.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gattc_enh_prf_services_browse(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, const ble_uuid_t *p_srvc_uuid);

/**
 ****************************************************************************************
 * @brief Profile client Discover Primary Services on remote GATT server.
 *
 * @note Event @ref BLE_GATTC_EVT_PRIMARY_SRVC_DISC will be called for service(s) found.
 *
 * @param[in] prf_id:          Profile id.
 * @param[in] conn_idx:        Current connection index.
 * @param[in] cid:             The local channel ID.
 * @param[in] p_srvc_uuid:     Pointer to Service UUID. If it is NULL, all Primary Services will be returned.
 *
 * @retval ::SDK_SUCCESS: Successfully start the Primary Service Discovery procedure.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gattc_enh_prf_primary_services_discover(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, const ble_uuid_t *p_srvc_uuid);

/**
 ****************************************************************************************
 * @brief Profile client Discover Included Services on remote GATT server.
 *
 * @note Event @ref BLE_GATTC_EVT_INCLUDE_SRVC_DISC will be called for Included Service(s) found.
 *
 * @param[in] prf_id:       Profile id.
 * @param[in] conn_idx:     Current connection index.
 * @param[in] cid:          The local channel ID.
 * @param[in] start_hdl:    Start handle.
 * @param[in] end_hdl:      End handle.
 *
 * @retval ::SDK_SUCCESS: Successfully start the Relationship Discovery procedure.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_INVALID_HANDLE: Invalid handle supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gattc_enh_prf_included_services_discover(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, uint16_t start_hdl, uint16_t end_hdl);

/**
 ****************************************************************************************
 * @brief Profile client Discover Characteristics on remote GATT server.
 * @note  Event @ref BLE_GATTC_EVT_CHAR_DISC will be called for Characteristic Declaration(s) found.
 *        If p_disc_char is NULL, the invalid pointer error code will be returned immediately.
 *
 * @param[in] prf_id:         Profile id.
 * @param[in] conn_idx:       Current connection index.
 * @param[in] cid:            The local channel ID.
 * @param[in] p_disc_char:    Pointer to discover by characteristic UUID info. If it's p_uuid member is NULL, all characteristics are returned.
 *
 * @retval ::SDK_SUCCESS: Successfully start the Characteristic Discovery procedure.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_HANDLE: Invalid handle supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gattc_enh_prf_char_discover(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, ble_gattc_disc_char_t *p_disc_char);

/**
 ****************************************************************************************
 * @brief Profile client Discover Characteristics Descriptors on remote GATT server.
 *
 * @note Event @ref BLE_GATTC_EVT_CHAR_DESC_DISC will be called for Characteristic Descriptor(s) found.
 * If the last Descriptor has not been reached, this function must be called again with an updated handle range to continue the discovery.
 *
 * @param[in] prf_id:       Profile id.
 * @param[in] conn_idx:     Current connection index.
 * @param[in] cid:          The local channel ID.
 * @param[in] start_hdl:    Start handle.
 * @param[in] end_hdl:      End handle.
 *
 * @retval ::SDK_SUCCESS: Successfully start the Descriptor Discovery procedure.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_INVALID_HANDLE: Invalid handle supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gattc_enh_prf_char_desc_discover(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, uint16_t start_hdl, uint16_t end_hdl);

/**
 ****************************************************************************************
 * @brief Profile client Read Attribute from remote GATT server.
 *
 * @note This uses either the "Read Characteristic Value" procedure or the "Read Characteristic Descriptor"
 *       procedure, depending on the attribute pointed by handle. If offset is non-zero or the
 *       attribute length is larger than the MTU, the "Read Long Characteristic Value" procedure or the
 *       "Read Long Characteristic Descriptor" procedure will be used respectively.
 *
 * @note Event @ref BLE_GATTC_EVT_READ_RSP will be called when Read is finished.
 *
 * @param[in] prf_id:     Profile id.
 * @param[in] conn_idx:   Current connection index.
 * @param[in] cid:        The local channel ID.
 * @param[in] handle:     Attribute handle.
 * @param[in] offset:     Value offset to start with.
 *
 * @retval ::SDK_SUCCESS: Successfully start the Read (Long) procedure.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_INVALID_HANDLE: Invalid handle supplied.
 * @retval ::SDK_ERR_INVALID_OFFSET: Offset exceeds the current attribute value length.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gattc_enh_prf_read(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, uint16_t handle, uint16_t offset);

/**
 ****************************************************************************************
 * @brief Profile client Read Attribute by UUID.
 *
 * @note Event @ref BLE_GATTC_EVT_READ_RSP will be called when Read is finished.
 *
 * @param[in] prf_id:            Profile id.
 * @param[in] conn_idx:          Current connection index.
 * @param[in] cid:               The local channel ID.
 * @param[in] p_read_by_uuid:    Pointer to Read by Characteristic UUID info.
 *
 * @retval ::SDK_SUCCESS: Successfully start the Read Using Characteristic UUID procedure.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_HANDLE: Invalid handle supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gattc_enh_prf_read_by_uuid(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, ble_gattc_read_by_uuid_t *p_read_by_uuid);

/**
 ****************************************************************************************
 * @brief Profile client Initiate a Read Multiple Characteristic Values procedure
 *
 * @note Event @ref BLE_GATTC_EVT_READ_RSP will be called for each handle value which is read.
 *       If set handle len 0, a Read Multiple Variable Length Characteristic Values procedure will be Initiated, this procesure is only
 *       support on version >= 5.2.
 *
 * @param[in] prf_id:      Profile id.
 * @param[in] conn_idx:    Current connection index.
 * @param[in] cid:         The local channel ID.
 * @param[in] p_param:     Pointer to the parameters of the value. 
 *
 * @note Event @ref BLE_GATTC_EVT_READ_RSP will be called when Read is finished.
 *       If set handle len 0, a Read Multiple Variable Length Characteristic Values procedure will be Initiated.
 *
 * @retval ::SDK_SUCCESS: Successfully start the Read Multiple Characteristic Values procedure.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gattc_enh_prf_read_multiple(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, const ble_gattc_read_multiple_t *p_param);


/**
 ****************************************************************************************
 * @brief Profile client Write (Long) Characteristic (Descriptor) Value.
 *
 * @note This uses either the "Write Characteristic Value" procedure or the "Write Characteristic
 *       Descriptor" procedure, depending on the attribute pointed by handle. If offset is non-zero
 *       or the attribute length is larger than the MTU, the "Write Long Characteristic Value" procedure
 *       or the "Write Long Characteristic Descriptor" procedure will be used respectively.
 *
 * @note Once completed event @ref BLE_GATTC_EVT_WRITE_RSP will be called.
 *
 * @param[in] prf_id:               Profile id.
 * @param[in] conn_idx:             Current connection index.
 * @param[in] cid:                  The local channel ID.
 * @param[in] p_write_attr_value:   Pointer to the write attribue value info.
 *
 * @retval ::SDK_SUCCESS: Successfully start the Write procedure.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_HANDLE: Invalid handle supplied.
 * @retval ::SDK_ERR_INVALID_OFFSET: Offset exceeds the current attribute value length.
 * @retval ::SDK_ERR_INVALID_DATA_LENGTH: Invalid data length supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gattc_enh_prf_write(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, ble_gattc_write_attr_value_t *p_write_attr_value);

/**
 ****************************************************************************************
 * @brief Profile client Prepare Long/Reliable Write to remote GATT server.
 * @note Once completed event @ref BLE_GATTC_EVT_WRITE_RSP will be called.
 *
 * @param[in] prf_id:               Profile id.
 * @param[in] conn_idx:             Current connection index.
 * @param[in] cid:                  The local channel ID.
 * @param[in] p_write_attr_value:   Pointer to the write attribue value info.
 *
 * @retval ::SDK_SUCCESS: Successfully start the Write procedure.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_HANDLE: Invalid handle supplied.
 * @retval ::SDK_ERR_INVALID_OFFSET: Offset exceeds the current attribute value length.
 * @retval ::SDK_ERR_INVALID_DATA_LENGTH: Invalid data length supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
 uint16_t ble_gattc_enh_prf_write_prepare(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, ble_gattc_write_attr_value_t *p_write_attr_value);

/**
 ****************************************************************************************
 * @brief Profile client Execute Reliable/Long Write to remote GATT server.
 *
 * @note Once completed event @ref BLE_GATTC_EVT_WRITE_RSP will be called.
 *
 * @param[in] prf_id:       Profile id.
 * @param[in] conn_idx:     Current connection index.
 * @param[in] cid:          The local channel ID.
 * @param[in] execute:      True if data shall be written; False if cancel all prepared writes.
 *
 * @retval ::SDK_SUCCESS: Successfully send an Execute Write request.
 * @retval ::SDK_ERR_INVALID_PARAM:    Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gattc_enh_prf_write_execute(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, bool execute);

/**
 ****************************************************************************************
 * @brief Profile client Write Attribute to remote GATT server (without response).
 *
 * @note Whatever signed_write is set, Write Without Response sub-procedure is always used, because the Signed Write Without Response 
 *       sub-procedure shall only be supported on the LE Fixed Channel Unenhanced ATT bearer.
 *
 * @note Once completed event @ref BLE_GATTC_EVT_WRITE_RSP will be called.
 *
 * @param[in] prf_id:            Profile id.
 * @param[in] conn_idx:          Current connection index.
 * @param[in] cid:               The local channel ID.
 * @param[in] p_write_no_resp:   Pointer to the write without response info.
 *
 * @retval ::SDK_SUCCESS Successfully: start the Write Without Response procedure.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_HANDLE: Invalid handle supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.

 ****************************************************************************************
 */
uint16_t ble_gattc_enh_prf_write_no_resp(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, ble_gattc_write_no_resp_t *p_write_no_resp);

/**
 ****************************************************************************************
 * @brief Profile client Confirm Reception of Indication.
 *
 * @note Confirm indication which has been correctly received from the peer.
 *
 * @param[in] prf_id        Profile id.
 * @param[in] conn_idx:     Current connection index.
 * @param[in] cid:          The local channel ID.
 * @param[in] handle:       Value handle.
 *
 * @retval ::SDK_SUCCESS: Successfully send a Confirm Reception of Indication request.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter(s) supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_CID: Invalid CID supplied.
 * @retval ::SDK_ERR_INVALID_HANDLE: Invalid handle supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gattc_enh_prf_indicate_cfm(uint8_t prf_id, uint8_t conn_idx, uint16_t cid, uint16_t handle);

/** @} */
/** @} */

#endif

/** @} */
/** @} */
/** @} */

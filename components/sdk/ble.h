/**
 ****************************************************************************************
 *
 * @file ble.h
 *
 * @brief include all ble sdk header files
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
 @addtogroup BLE_COMMEN BLE Common
 @{
 @brief BLE Common interface.
 */

#ifndef __BLE_H__
#define __BLE_H__

#include "ble_att.h"
#include "ble_error.h"
#include "ble_gapc.h"
#include "ble_gapm.h"
#include "ble_gatt.h"
#include "ble_gattc.h"
#include "ble_gatts.h"
#include "ble_l2cap.h"
#include "ble_prf.h"
#include "ble_enh_prf.h"
#include "ble_sec.h"
#include "ble_event.h"
#include "ble_gattc_cache.h"

#include <stdio.h> 

/** @addtogroup BLE_COMMEN_ENUM Enumerations
 * @{
 */

/**
 * @brief RF TX mode. 
 */
typedef enum
{
    BLE_RF_TX_MODE_INVALID = 0,
    BLE_RF_TX_MODE_LP_MODE = 1,
    BLE_RF_TX_MODE_ULP_MODE = 2,
} ble_rf_tx_mode_t;

/**
 * @brief The resistance value (ohm) of the RF match circuit. 
 */
typedef enum
{
    BLE_RF_MATCH_CIRCUIT_25OHM = 25,
    BLE_RF_MATCH_CIRCUIT_100OHM = 100,
} ble_rf_match_circuit_t;

/**
 * @brief RF RX performace/power selection. 
 */
typedef enum
{
    PRIOR_RF_RX_PERFORMANCE = 0,
    PRIOR_RF_RX_POWER = 1,
}rf_rx_performance_selection_t;

/**
 ****************************************************************************************
 * @brief Call back definition of the function that can handle result of an AES based algorithm
 *
 * @param[in] status       Execution status
 * @param[in] aes_res      16 bytes block result
 * @param[in] src_info     Information provided by requester
 ****************************************************************************************
 */
typedef void (*aes_result_cb) (uint8_t status, const uint8_t* aes_res, uint32_t src_info);

/**@brief Receive controller pachet callback type. */
typedef void (*ble_hci_host_recv_cb_t)(uint8_t *p_data, uint16_t length);

/** @} */

/** @addtogroup BLE_COMMEN_STRUCTURES Structures
 * @{
 */
/**@brief The table contains the pointers to four arrays which are used
 * as heap memory by BLE stack in ROM. The size of four arrays depends on
 * the number of connections and the number of attributes of profiles. */
typedef struct 
{
    uint32_t  *env_ret;                 /**< Pointer to the array for environment heap */
    uint32_t  *db_ret;                  /**< Pointer to the array for ATT DB heap */
    uint32_t  *msg_ret;                 /**< Pointer to the array for message heap */
    uint32_t  *non_ret;                 /**< Pointer to the array for non-retention heap */
    uint32_t   env_ret_size;            /**< The size of the array for environment heap */
    uint32_t   db_ret_size;             /**< The size of the array for ATT DB heap */
    uint32_t   msg_ret_size;            /**< The size of the array for message heap */
    uint32_t   non_ret_size;            /**< The size of the array for non-retention heap */
    uint8_t   *prf_buf;                 /**< Pointer to the array for profile heap */
    uint32_t   buf_size;                /**< The size of the array for profile heap */
    uint8_t   *bm_buf;                  /**< Pointer to the array for bond manager heap */
    uint32_t   bm_size;                 /**< The size of the array for bond manager heap */
    uint8_t   *conn_buf;                /**< Pointer to the array for connection heap */
    uint32_t   conn_size;               /**< The size of the array for connection heap */

    uint16_t   em_ble_act_num;          /**< The number of ble maxium activities */
    uint16_t   em_ble_ral_num;          /**< The number of ble maxium resolvable address list */
    uint16_t   em_ble_adv_buf_nb_tx;    /**< The number of advertising data buffers */
    uint16_t   em_ble_adv_frag_nb_tx;   /**< The number of advertising or scan response data fragments in extended advertising PDU chain */
    uint16_t   em_common_offset;        /**< Start offset of the common EM part */
} stack_heaps_table_t;

/**@brief BLE HCI RX Channel (Host send packet to controller). */
typedef struct
{
    uint8_t     *p_channel;    /**< Pointer to buffer for controller receive cache. */
    uint16_t     cache_size;   /**< Size of the cache buffer. */
} ble_hci_rx_channel_t;

/**@brief Extended LLCP. */
struct ble_ext_llcp_cb_func_t
{
    void (*llc_ext_llcp_proc_error_cb) (uint8_t link_id, uint8_t error_type, void* param);                           /**< Handler of Extended llcp procedure error. */
    void (*llc_ext_llcp_req_proc_done_cb) (uint8_t link_id, uint8_t status, uint8_t ext_opcode, uint8_t *per_param); /**< Report the Extended llcp procedure done. */
    uint8_t (*llc_ext_llcp_req_handler_cb) (uint8_t link_id, uint8_t *req_param, uint8_t *rsp_param);                /**< Handle the Extended llcp request. */
};

/** @} */

/** @addtogroup BLE_COMMEN_FUNCTIONS Functions
 * @{ */
/**
 *****************************************************************************************
 * @brief Initialize BLE Stack.
 *
 * @param[in] evt_handler:    Pointer to the ble event handler.
 * @param[in] p_heaps_table:  Pointer to the BLE stack heaps table.
 *****************************************************************************************
 */
uint16_t ble_stack_init(ble_evt_handler_t evt_handler, stack_heaps_table_t *p_heaps_table);


/**
 *****************************************************************************************
 * @brief Initialize BLE Stack controller.
 *
 * @param[in] p_heaps_table:  Pointer to the BLE stack heaps table.
 *
 * @note: This function should be called if only as a controller.
 *****************************************************************************************
 */
void ble_stack_controller_init(stack_heaps_table_t *p_heaps_table);

/**
 *****************************************************************************************
 * @brief Initialize ble hci adapter module.
 *
 * @param[in] p_rx_channel: Pointer to hci adapter rx channel
 * @param[in] host_recv_cb: Callback to receive controller packet.
 *
 * @return Result of initializing.
 *****************************************************************************************
 */
sdk_err_t ble_hci_init(ble_hci_rx_channel_t *p_rx_channel, ble_hci_host_recv_cb_t host_recv_cb);

/**
 *****************************************************************************************
 * @brief BLE HCI adapter host send packet.
 *
 * @param[in] p_data: Pointer to packet data.
 * @param[in] length: Length of packet data.
 *
 * @return Result of send.
 *****************************************************************************************
 */
sdk_err_t ble_hci_host_packet_send(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Get surplus space of controller receive channel.
 *
 * @return Result of get.
 *****************************************************************************************
 */
uint16_t ble_hci_rx_channel_surplus_space_get(void);

/**
 *****************************************************************************************
 * @brief Register BLE idle time notification callback function.
 *
 * @param[in] callback:  function pointer of BLE idle time notification function.
 * @note hs: the idle time of BLE in half slot (312.5us).
 *       Callback will be called by BLE ISR to notify the rest idle time if there are some BLE activities.
 *       It should be realized as simlpe as you can.
 *       It's not suitable for ISO activities.
 *****************************************************************************************
 */
void ble_idle_time_notify_cb_register(void (*callback)(uint32_t hs));

/**
 *****************************************************************************************
 * @brief Register BLE activity start notification callback function.
 *
 * @param[in] callback: function pointer of BLE activity start notification function.
 * @note e_role: the role of activity, @ref ble_gap_actv_role_t for possible roles.
 *       index: The index parameter is interpreted by role.
 *       If role is @ref BLE_GAP_ACTIVITY_ROLE_ADV, it's the index of Advertising.
 *       If role is @ref BLE_GAP_ACTIVITY_ROLE_CON, it's the index of Connection.
 *       For all other roles, it should be ignored.
 *       Callback will be called by BLE ISR when the BLE activity starts every time.
 *       It should be realized as simlpe as you can. You'd better to define it in the RAM space
 *       It's not suitable for ISO activities.
 *****************************************************************************************
 */
void ble_activity_start_notify_cb_register(void (*callback)(ble_gap_actv_role_t e_role,  uint8_t index));

/**
 *****************************************************************************************
 * @brief Register BLE activity end notification callback function.
 *
 * @param[in] callback: function pointer of BLE activity end notification function.
 * @note e_role: the role of activity, refer to @ref ble_gap_actv_role_t for possible roles.
 *       index: The index parameter is interpreted by role.
 *       If role is @ref BLE_GAP_ACTIVITY_ROLE_ADV, it's the index of Advertising.
 *       If role is @ref BLE_GAP_ACTIVITY_ROLE_CON, it's the index of Connection.
 *       For all other roles, it should be ignored.
 *       Callback will be called by BLE ISR when the BLE activity ends every time.
 *       It should be realized as simlpe as you can. You'd better to define it in the RAM space
 *       It's not suitable for ISO activities.
 *****************************************************************************************
 */
void ble_activity_end_notify_cb_register(void (*callback)(ble_gap_actv_role_t e_role,  uint8_t index));

/**
 *****************************************************************************************
 * @brief Change the RF TX mode of LP or ULP.
 *
 * @param[in] e_rf_tx_mode: Refer to @ref ble_rf_tx_mode_t.
 *
 * @note This function should be called before BLE stack init.
 *
 * @return SDK_SUCCESS: Successfully set tx mode.
 *         SDK_ERR_DISALLOWED: Failed to set tx mode.
 *****************************************************************************************
 */
uint8_t ble_rf_tx_mode_set(ble_rf_tx_mode_t e_rf_tx_mode);

/**
 *****************************************************************************************
 * @brief Get the RF TX mode of LP or ULP.
 *
 * @return Refer to @ref ble_rf_tx_mode_t.
 *
 *****************************************************************************************
 */
ble_rf_tx_mode_t ble_rf_tx_mode_get(void);

/**
 *****************************************************************************************
 * @brief Add an option to improve rf rx performance, and the current during rx will increase a bit
 *
 * @param[in] selection: The choice of prefer better rf rx performance or better power consumption.
 *
 * @note  This function should be called after BLE stack init.
 *****************************************************************************************
 */
void ble_rf_rx_performance_selection(rf_rx_performance_selection_t selection);
 
/**
 *****************************************************************************************
 * @brief Set the resistance value of the RF match circuit (unit: ohm).
 *
 * @param[in] e_ohm: The resistance value (ohm) of the RF match circuit according to the board, 
 *                   Refer to @ref ble_rf_match_circuit_t.
 *                   BLE_RF_MATCH_CIRCUIT_25OHM: 25 ohm.
 *                   BLE_RF_MATCH_CIRCUIT_100OHM: 100 ohm.
 *                   Others: invalid.
 *
 * @note  This function should be called before BLE stack init.
 *****************************************************************************************
 */
void ble_rf_match_circuit_set(ble_rf_match_circuit_t e_ohm);

/**
 *****************************************************************************************
 * @brief Get the resistance value of the RF match circuit (unit: ohm).
 *
 * @return The resistance value (ohm) of the RF match circuit according to the board (ohm).
 *         BLE_RF_MATCH_CIRCUIT_25OHM: 25 ohm.
 *         BLE_RF_MATCH_CIRCUIT_100OHM: 100 ohm.
 *         Others: invalid.
 *****************************************************************************************
 */
ble_rf_match_circuit_t ble_rf_match_circuit_get(void);

/**
 *****************************************************************************************
 * @brief  Start a extended llcp procedure.
 * @param[in] conn_idx:   function pointer of BLE activity end notification function.
 * @param[in] param:      Pointer to a param which intializer supply.
 * @param[in] ext_opcode: Extended llcp opcode.
 * @return The result of this function.
 *****************************************************************************************
 */
uint8_t ble_ext_llcp_send(uint16_t conn_idx, uint8_t *param, uint8_t ext_opcode);

/**
 *****************************************************************************************
 * @brief Registered the extended llcp procedure callback.
 * @param[in] func: @ref ble_ext_llcp_cb_func_t.
 * @return The result of this function.
 *****************************************************************************************
 */
uint8_t ble_ext_llcp_cb_reg(struct ble_ext_llcp_cb_func_t *func);

/**
 *****************************************************************************************
 * @brief Perform an AES encryption - result within callback
 * @param[in] key      Key used for the encryption
 * @param[in] val      Value to encrypt using AES
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] src_info Information used retrieve requester
 * @return void.
 *****************************************************************************************
 */
void ble_aes_encrypt(const uint8_t * key, const uint8_t * val, aes_result_cb res_cb, uint32_t src_info);

/**
 *****************************************************************************************
 * @brief Perform an AES dencryption - result within callback
 * @param[in] key      Key used for the dencryption
 * @param[in] val      Value to decrypt using AES
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] src_info Information used retrieve requester
 * @return void.
 *****************************************************************************************
 */
void ble_aes_decrypt(const uint8_t * key, const uint8_t * val, aes_result_cb res_cb, uint32_t src_info);

/**
 *****************************************************************************************
 * @brief Perform an AES ecb encryption - result within callback
 * @param[in] key      Key used for the encryption
 * @param[in] val      Value to encrypt using AES
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] src_info Information used retrieve requester
 * @return void.
 *****************************************************************************************
 */
void ble_aes_ecb_encrypt(const uint8_t * key, const uint8_t * val, aes_result_cb res_cb, uint32_t src_info);

/**
 *****************************************************************************************
 * @brief Perform an AES  ecb dencryption - result within callback
 * @param[in] key      Key used for the dencryption
 * @param[in] val      Value to decrypt using AES
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] src_info Information used retrieve requester
 * @return void.
 *****************************************************************************************
 */
void ble_aes_ecb_decrypt(const uint8_t * key, const uint8_t * val, aes_result_cb res_cb, uint32_t src_info);

/**
 *****************************************************************************************
 * @brief Perform an AES cbc encryption - result within callback
 * @param[in] key      Key used for the encryption
 * @param[in] iv        IV used for the cbc encryption.only first block need to be present, continue block shall be null.
 * @param[in] val      Value to encrypt using AES
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] src_info Information used retrieve requester
 * @return void.
 *****************************************************************************************
 */
void ble_aes_cbc_encrypt(const uint8_t * key, const uint8_t * iv, const uint8_t * val, aes_result_cb res_cb, uint32_t src_info);

/**
 *****************************************************************************************
 * @brief Perform an AES cbc dencryption - result within callback
 * @param[in] key      Key used for the decryption
 * @param[in] iv        IV used for the cbc decryption.only first block need to be present, continue block shall be null.
 * @param[in] val      Value to decrypt using AES
 * @param[in] res_cb   Function that will handle the AES based result (16 bytes)
 * @param[in] src_info Information used retrieve requester
 * @return void.
 *****************************************************************************************
 */
void ble_aes_cbc_decrypt(const uint8_t * key, const uint8_t * iv, const uint8_t * val, aes_result_cb res_cb, uint32_t src_info);
/** @} */
#endif
/** @} */
/** @} */


/**
 ****************************************************************************************
 *
 * @file ble_lcp.h
 *
 * @brief LCP SDK API
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
 * @brief Definitions and prototypes for the BLE SDK interface.
 */
 
  /**
 * @addtogroup BLE_LCP Light Communication Protocol (LCP)
 * @{
 * @brief Definitions and prototypes for the LCP interface.
 */

#ifndef _LCP_SDK_H_
#define _LCP_SDK_H_


/**@addtogroup BLE_LCP_TYPEDEFS Typedefs
 * @{ */
/**@brief RX handler callback function. */
typedef uint16_t (*rx_handler_cb_t) (uint8_t header, uint8_t length, uint8_t *p_payload);
/** @} */

/**@addtogroup BLE_LCP_ENUMERATIONS Enumerations
 * @{ */
/**@brief Protocol Mode. */
enum PROTOCOL_MODE
{
    BLE_ADV,       /**< BLE ADV mode. */
    BLE_SCAN,      /**< BLE SCAN mode. */
    LCP_TX,        /**< LCP TX mode. */
    LCP_RX,        /**< LCP RX mode. */
};
/** @} */

/**@addtogroup BLE_LCP_STRUCTURES Structures
 * @{ */
/**@brief LCP Parameter. */
typedef struct
{
    uint8_t   mode;                 /**< Set protocol mode, see @ref PROTOCOL_MODE. */
    int8_t    txpwr_dbm;            /**< The value of the tx power(range: -20-7), uint: dBm. */
    uint8_t   ch_idx;               /**< The value of the channel index(range: 0-39). */
    uint32_t  freq;                 /**< The value of the frequency(range: 2360-2520), uint: MHz. */
    uint32_t  access_address;       /**< The value of the access address. */
    uint32_t  crc_init;             /**< The initial value of the crc. */
    rx_handler_cb_t rx_handler_cb;  /**< The callback function of rx. */
} gdx_lcp_config_t;
/** @} */

/** @addtogroup BLE_LCP_FUNCTIONS Functions
 * @{ */
/**
 ****************************************************************************************
 * @brief Initialize LCP.
 *
 * @param[in] gdx_lcp_config: Configure the parameter of LCP, @ref gdx_lcp_config_t.
 *
 * @retval ::SDK_SUCCESS: The LCP parameter is successfully configured.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 ****************************************************************************************
 */
uint16_t gdx_lcp_init(gdx_lcp_config_t *gdx_lcp_config);

/**
 ****************************************************************************************
 * @brief Deinitialize LCP.
 *
 * @retval ::SDK_SUCCESS: The LCP is successfully Deinitialized.
 ****************************************************************************************
 */
uint16_t gdx_lcp_deinit(void);

/**
 ****************************************************************************************
 * @brief Set the tx power of LCP.
 *
 * @param[in] txpwr_dbm: The value of the tx power, Range: -20dbm to 7dbm.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 ****************************************************************************************
 */
uint16_t gdx_lcp_tx_power_set(int8_t txpwr_dbm);

/**
 ****************************************************************************************
 * @brief Get the tx power of LCP.
 *
 * @param[in] txpwr_dbm: The value of the tx power, Range: -20dbm to 7dbm.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t gdx_lcp_tx_power_get(int8_t *txpwr_dbm);

/**
 ****************************************************************************************
 * @brief Set the channel of LCP.
 *
 * @param[in] freq: The value of the frequency, Range: 2360MHz to 2520MHz.
 * @param[in] ch_idx: The value of the channel index, Range: 0 to 39.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 ****************************************************************************************
 */
uint16_t gdx_lcp_channel_set(uint32_t freq, uint8_t ch_idx);

/**
 ****************************************************************************************
 * @brief Get the channel of LCP.
 *
 * @param[in] freq: The value of the frequency, Range: 2360MHz to 2520MHz.
 * @param[in] ch_idx: The value of the channel index, Range: 0 to 39.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t gdx_lcp_channel_get(uint32_t *freq, uint8_t *ch_idx);

/**
 ****************************************************************************************
 * @brief Transmmit a packet.
 *
 * @param[in] header: The header of the packet.
 * @param[in] length: The length of the packet payload.
 * @param[in] p_payload: The pointer of the packet payload.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 ****************************************************************************************
 */
uint16_t gdx_lcp_data_tx(uint8_t header, uint8_t length, uint8_t *p_payload);

/**
 ****************************************************************************************
 * @brief Start receiving packets
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 ****************************************************************************************
 */
uint16_t gdx_lcp_rx_start(void);

/**
 ****************************************************************************************
 * @brief Stop receiving packets
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 ****************************************************************************************
 */
uint16_t gdx_lcp_rx_stop(void);

/** @} */

#endif

/** @} */
/** @} */

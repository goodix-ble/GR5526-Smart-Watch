/**
 ****************************************************************************************
 *
 * @file ble_advertising.h
 *
 * @brief BLE Advertising Module Header
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

#ifndef __BLE_ADVERTISING_H__
#define __BLE_ADVERTISING_H__

#include "gr_includes.h"

/**
 * @defgroup BLE_ADVERTISING_MAROC Defines
 * @{
 */
#define RET_VERIFY_SUCCESS(RET_CODE)                 \
do                                                   \
{                                                    \
    if (RET_CODE != SDK_SUCCESS)                     \
    {                                                \
        return RET_CODE;                             \
    }                                                \
} while(0)
/** @} */

/**
 * @defgroup BLE_ADVERTISING_ENUM Enumerations
 * @{
 */
/**@brief  Advertising mode type. */
typedef enum
{
    BLE_ADV_MODE_IDLE,                   /**< No advertising activity. */
    BLE_ADV_MODE_DIRECTED_HIGH_DUTY,     /**< High duty directed advertising. */
    BLE_ADV_MODE_DIRECTED_LOW_DUTY,      /**< Low duty directed advertising. */
    BLE_ADV_MODE_FAST,                   /**< Fast advertising mode. */
    BLE_ADV_MODE_SLOW,                   /**< Slow advertising mode. */
} ble_adv_mode_t;

/**@brief Advertising events type. */
typedef enum
{
    BLE_ADV_EVT_INVALID,                /**< Invalid advertising event. */
    BLE_ADV_EVT_DIRECTED_HIGH_DUTY,     /**< High duty directed advertising started. */
    BLE_ADV_EVT_DIRECTED_LOW_DUTY,      /**< Low duty directed advertising started. */
    BLE_ADV_EVT_FAST,                   /**< Fast advertising started. */
    BLE_ADV_EVT_SLOW,                   /**< Slow advertising started. */
    BLE_ADV_EVT_DIR_ADDR_REQUEST,       /**< Request peer address for directed advertising. */
} ble_adv_evt_type_t;
/** @} */

/**
 * @defgroup BLE_ADVERTISING_TYPEDEF Typedefs
 * @{
 */
/**@brief BLE advertising event handler type. */
typedef void (*ble_adv_evt_handler_t)(ble_adv_evt_type_t evt);

/**@brief BLE advertising error handler type. */
typedef void (*ble_adv_err_handler_t)(uint8_t err_code);
/** @} */


/**
 * @defgroup BLE_ADVERTISING_STRUCT Structures
 * @{
 */
/**@brief Data structure. */
typedef struct
{
    const uint8_t *p_data;  /**< Pointer to the data. */
    uint16_t       length;  /**< Length of the data. */
} ble_data_t;

/**@brief  Options for the different advertisement modes.*/
typedef struct
{
    bool                    multi_link_enabled;             /**< Enable or disable multi link. */
    bool                    adv_on_disconnect_enabled;      /**< Enable or disable auto advertising after disconnecting. */
    bool                    adv_directed_high_duty_enabled; /**< Enable or disable high duty direct advertising mode. */
    bool                    adv_directed_low_duty_enabled;  /**< Enable or disable low duty direct advertising mode. */
    bool                    adv_fast_enabled;               /**< Enable or disable fast advertising mode. */
    bool                    adv_slow_enabled;               /**< Enable or disable slow advertising mode. */
    bool                    adv_extended_enabled;           /**< Enable or disable extended advertising. */
    uint16_t                adv_directed_low_duty_interval; /**< Advertising interval for directed advertising. */
    uint16_t                adv_directed_low_duty_timeout;  /**< Time-out (number of tries) for direct advertising. */
    uint16_t                adv_fast_interval;              /**< Advertising interval for fast advertising. */
    uint16_t                adv_fast_timeout;               /**< Time-out (in units of 10ms) for fast advertising. */
    uint16_t                adv_slow_interval;              /**< Advertising interval for slow advertising. */
    uint16_t                adv_slow_timeout;               /**< Time-out (in units of 10ms) for slow advertising. */
    ble_gap_le_phy_value_t  adv_secondary_phy;              /**< PHY for the secondary (extended) advertising. */
    ble_gap_le_phy_value_t  adv_primary_phy;                /**< PHY for the primary advertising. */
} ble_adv_mode_cfg_t;

/**@brief Avertising initialization. */
typedef struct
{
    ble_data_t            adv_data;         /**< Advertising data. */
    ble_data_t            srp_data;         /**< Scan response data. */
    ble_adv_mode_cfg_t    adv_mode_cfg;     /**< Advertising mode config. */
    ble_adv_evt_handler_t evt_handler;      /**< Advertising event handler. */
    ble_adv_err_handler_t err_handler;      /**< Advertising error handler. */
} ble_adv_init_t;
/** @} */

/**
 * @defgroup BLE_ADVERTISING_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize BLE advertising module.
 *
 * @param[in] p_adv_init: Pointer to advertising init params.
 * 
 * @return Result of ble advertising initialization.
 *****************************************************************************************
 */
sdk_err_t ble_advertising_init(ble_adv_init_t *p_adv_init);

/**
 *****************************************************************************************
 * @brief Start advertising.
 *
 * @param[in] adv_mode: Advertising mode.
 * 
 * @return Result of ble advertising start.
 *****************************************************************************************
 */
sdk_err_t ble_advertising_start(ble_adv_mode_t adv_mode);

/**
 *****************************************************************************************
 * @brief Update the advertising data.
 *
 * @param[in] p_adv_data:     Pointer to the new advertising data, or null if advertising data dosen't need to be updated.
 * @param[in] adv_data_len:   Length of the new advertising data. 
 * @param[in] p_srsp_data:    Pointer to the new scan response data, or null if advertising data dosen't need to be updated.
 * @param[in] srsp_data_len:  Length of the new scan response data.
 * 
 * @return Result of adv data update.
 *****************************************************************************************
 */
sdk_err_t ble_advertising_adv_data_update(const uint8_t *p_adv_data, uint16_t adv_data_len, const uint8_t *p_srsp_data, uint16_t srsp_data_len);

/**
 *****************************************************************************************
 * @brief Fill direct advertising peer address.
 *
 * @param[in] p_peer_addr: Pointer to peer address.
 * 
 * @return Result of address check.
 *****************************************************************************************
 */
sdk_err_t ble_advertising_dir_addr_fill(ble_gap_bdaddr_t *p_peer_addr);

void ble_advertising_evt_on_ble_capture(const ble_evt_t *p_evt);

/** @} */

#endif

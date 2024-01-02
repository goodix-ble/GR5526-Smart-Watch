/**
 ******************************************************************************
 *
 * @file  ble_prf_utils.h
 *
 * @brief Profile/Service Utilities API
 *
 ******************************************************************************
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
 * @defgroup BLE_PRF_UTILS Profile/Service Utilities
 * @{
 * @brief Definitions and prototypes for the Profile/Service Utilities interface.
 *
 */

#ifndef __BLE_PRF_UTILS_H__
#define __BLE_PRF_UTILS_H__

#include "ble_prf_types.h"
#include <stdbool.h>

/**
 * @defgroup BLE_PRF_UTILS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Pack Characteristic Presentation Format descriptor value to a buffer.
 *
 * @param[out] p_packed_val:    Pointer to the packed buffer.
 * @param[in]  p_char_pres_fmt: Pointer to the structure of Characteristic Presentation
 *                              Format value. See @ref prf_char_pres_fmt_t.
 *****************************************************************************************
 */
void prf_pack_char_pres_fmt(uint8_t *p_packed_val, const prf_char_pres_fmt_t *p_char_pres_fmt);

/**
 *****************************************************************************************
 * @brief Unpack the data in a buffer to the structure of Characteristic Presentation
 *        Format descriptor value.
 *
 * @param[in]  p_packed_val:    Pointer to the packed buffer.
 * @param[out] p_char_pres_fmt: Pointer to the structure of Characteristic Presentation
 *                              Format value. See @ref prf_char_pres_fmt_t.
 *****************************************************************************************
 */
void prf_unpack_char_pres_fmt(const uint8_t *p_packed_val, prf_char_pres_fmt_t *p_char_pres_fmt);

/**
 *****************************************************************************************
 * @brief Pack the value in date-time structure to a buffer.
 *
 * @param[out] p_packed_val: Pointer to a packed buffer.
 * @param[in]  p_date_time:  Pointer to the date-time structure, see @ref prf_date_time_t.
 *
 * @return The size of packed value.
 *****************************************************************************************
 */
uint8_t prf_pack_date_time(uint8_t *p_packed_val, const prf_date_time_t *p_date_time);

/**
 *****************************************************************************************
 * @brief Unpack the data in buffer to the date-time structure.
 *
 * @param[in]  p_packed_val: Pointer to the packed buffer.
 * @param[out] p_date_time:  Pointer to date-time structure, see @ref prf_date_time_t.
 *
 * @return The size of packed value
 *****************************************************************************************
 */
uint8_t prf_unpack_date_time(const uint8_t *p_packed_val, prf_date_time_t *p_date_time);

/**
 *****************************************************************************************
 * @brief Find the attribute index by handle.
 *
 * @param[in] handle:      The handle of a characteristic in BLE Stack database.
 * @param[in] start_hdl:   The start handle of a service in BLE Stack database.
 * @param[in] char_nb:     The number of the characteristics in a service's attribute table.
 * @param[in] p_char_mask: Pointer to the mask of characteristics which are added
 *                         into BLE Stack database from the service's attribute table.
 *
 * @return The handle's index in the service's attribute table.
 *****************************************************************************************
 */
uint8_t prf_find_idx_by_handle(uint16_t handle,  uint16_t start_hdl,
                               uint8_t  char_nb, uint8_t *p_char_mask);

/**
 *****************************************************************************************
 * @brief Find the attribute handle by index.
 *
 * @param[in] idx:         The attribute index in the service's attribute table.
 * @param[in] start_hdl:   The start handle value in the database.
 * @param[in] p_char_mask: Pointer to the mask of characteristics which are added
 *                         into BLE Stack database from the service's attribute table.
 *
 * @return The handle of the attribute with the index in BLE Stack database.
 *****************************************************************************************
 */
uint16_t prf_find_handle_by_idx(uint8_t idx, uint16_t start_hdl, uint8_t *p_char_mask);

/**
 *****************************************************************************************
 * @brief Check if a CCCD value is valid.
 *
 * @param[in] cccd_value: The CCCD value to be checked.
 *
 * @return True if valid, otherwise false.
 ****************************************************************************************
 */
bool prf_is_cccd_value_valid(uint16_t cccd_value);

/**
 *****************************************************************************************
 * @brief Check if a CCCD value is notification enabled.
 *
 * @param[in] cccd_value: The CCCD value to be checked.
 *
 * @return true if enabled, otherwise false.
 ****************************************************************************************
 */
bool prf_is_notification_enabled(uint16_t cccd_value);

/**
 *****************************************************************************************
 * @brief Check if a CCCD value is indication enabled.
 *
 * @param[in] cccd_value: The CCCD value to be checked
 *
 * @return true if enabled, otherwise false.
 *****************************************************************************************
 */
bool prf_is_indication_enabled(uint16_t cccd_value);
/** @} */

#endif /* _BLE_PRF_UTILS_H_ */
/** @} */
/** @} */


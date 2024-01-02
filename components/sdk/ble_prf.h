/**
 ****************************************************************************************
 *
 * @file ble_prf.h
 *
 * @brief BLE PRF API
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

#ifndef __BLE_PRF_H__
#define __BLE_PRF_H__

#include "ble_error.h"
#include "ble_att.h"
#include "ble_gatts.h"
#include "ble_gattc.h"
#include "ble_gatt.h"
#include "ble_event.h"

/** @addtogroup BLE_PRF_FUNCTIONS Functions
* @{ */

/**
 ****************************************************************************************
 * @brief Add a server profile by providing its detailed information..
 *
 * @param[in] p_gatts_db:     Pointer to the prf_info. See @ref ble_gatts_create_db_t.
 * @param[in] evt_handler:    Pointer to ble events handler. 
 *
 * @retval ::SDK_SUCCESS:           The profile info is recorded successfully, and the database will be created in profile initialization callback function.
 * @retval ::SDK_ERR_POINTER_NULL:  The parameter p_gatts_db or evt_handler is NULL.
 * @retval ::SDK_ERR_NO_RESOURCES:  The profile number is up to the maximum number the system can support.
 ****************************************************************************************
 */
uint16_t ble_gatts_prf_add(ble_gatts_create_db_t *p_gatts_db, ble_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief Add a client profile by providing its detail information.
 *
 * @param[in]  p_uuid:       Pointer to the target service uuid. See @ref ble_uuid_t.
 * @param[out] evt_handler:  Pointer to ble events handler..
 *
 * @retval ::SDK_SUCCESS:          The profile info is recorded successfully, and the profile ENV will be initialized in profile initialization callback function.
 * @retval ::SDK_ERR_POINTER_NULL: The parameter p_uuid or evt_handler is NULL, or input parameters that prf_info points to are invalid.
 * @retval ::SDK_ERR_NO_RESOURCES: The profile number is up to the maximum number the system can support.
 ****************************************************************************************
 */
uint16_t ble_gattc_prf_add(ble_uuid_t *p_uuid, ble_evt_handler_t evt_handler);
/** @} */

#endif

/** @} */
/** @} */

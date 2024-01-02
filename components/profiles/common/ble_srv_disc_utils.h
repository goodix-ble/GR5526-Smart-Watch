/**
 ******************************************************************************
 *
 * @file  ble_srv_disc_utils.h
 *
 * @brief Service Discovery Utilities API
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
 * @defgroup BLE_SRV_DISC_UTILS Service Discovery Utilities
 * @{
 * @brief Definitions and prototypes for the Service Discovery Utilities interface.
 *
 */

#ifndef __BLE_SRV_DISC_UTILS_H__
#define __BLE_SRV_DISC_UTILS_H__

#include <stdint.h>

/**
 * @defgroup DIS_C_MACRO Defines
 * @{
 */
#define BLE_SRV_DISC_PROC_MAX         6            /**< Maximum number of services discovery procedure in one application. */
/** @} */

/**
 * @defgroup BLE_SRV_DISC_UTILS_ENUM Enumerations
 * @{
 */
/**@brief BLE Service Discovery Procedure State. */
typedef enum
{
    BLE_SRV_DISC_NO_IMPLEMENT,    /**< Service discovery procedure has not been implemented. */
    BLE_SRV_DISC_UNDERWAY,        /**< Service discovery procedure is underway. */
    BLE_SRV_DISC_COMPLETELY,      /**< Service discovery procedure has been completed. */
} ble_srv_disc_state_t;

/**@brief BLE Service Discovery Procedure ID. */
typedef enum
{
    BLE_SRV_DISC_PROC_ID_0,       /**< Service discovery procedure ID_0. */
    BLE_SRV_DISC_PROC_ID_1,       /**< Service discovery procedure ID_1. */
    BLE_SRV_DISC_PROC_ID_2,       /**< Service discovery procedure ID_2. */
    BLE_SRV_DISC_PROC_ID_3,       /**< Service discovery procedure ID_3. */
    BLE_SRV_DISC_PROC_ID_4,       /**< Service discovery procedure ID_4. */
    BLE_SRV_DISC_PROC_ID_5,       /**< Service discovery procedure ID_5. */
    BLE_SRV_DISC_PROC_NB          /**< Maximum number of services discovery procedure in one application. */
} ble_srv_disc_proc_t;
/** @} */

/**
 * @defgroup BLE_SRV_DISC_UTILS_FUNCTION Functions
 * @{
 */

/**
 *****************************************************************************************
 * @brief Service discovery procedure state set.
 *
 * @param[in] srv_disc_proc_id: Service discovery procedure.
 * @param[in] srv_disc_state:   State of service discovery procedure.
 *****************************************************************************************
 */
void ble_srv_disc_proc_state_set(uint8_t srv_disc_proc_id, ble_srv_disc_state_t srv_disc_state);

/**
 *****************************************************************************************
 * @brief Get service discovery procedure state.
 *
 * @param[in] srv_disc_proc_id: Service discovery procedure.
 *
 * @return  State of service discovery procedure.
 *****************************************************************************************
 */
ble_srv_disc_state_t ble_srv_disc_proc_state_get(uint8_t srv_disc_proc_id);
/** @} */

#endif
/** @} */
/** @} */


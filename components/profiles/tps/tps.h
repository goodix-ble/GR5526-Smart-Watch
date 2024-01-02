/**
 *****************************************************************************************
 *
 * @file tps.h
 *
 * @brief Tx Power Service API
 *
 *****************************************************************************************
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
 * @defgroup BLE_SDK_TPS Tx Power Service (TPS)
 * @{
 * @brief Definitions and prototypes for the TPS interface.
 *
 * @details The Tx Power Service uses the Tx Power characteristic to expose a device's current
 *          transmit power level when in connection.
 *
 *          After \ref tps_init_t variable is initialized, the application must call \ref tps_service_init()
 *          to add Tx Power Service and Tx Power Level characteristic to the BLE Stack database.
 *
 *          This module also provides \ref tps_tx_power_level_set() function to the
 *          application to update the current value of Tx Power Level characteristic.
 *
 */

#ifndef __TPS_H__
#define __TPS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>

/**
 * @defgroup TPS_STRUCT Structures
 * @{
 */
/**@brief Tx Power Servic init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    int8_t initial_tx_power_level;        /**< Initial value of Tx Power Level characteristic (in dBm) */
} tps_init_t;
/** @} */

/**
 * @defgroup TPS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Tx Power Service instance and add in BLE Stack database.
 *
 * @param[in] p_tps_init: Pointer to a Tx Power Service environment variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t tps_service_init(tps_init_t *p_tps_init);

/**
 *****************************************************************************************
 * @brief Set new value of Tx power level characteristic.
 *
 * @param[in] tx_power_level: New value of Tx power level, range [-100, 20] dBm.
 *
 * @return BLE_SDK_SUCCESS on success, otherwise an error code.
 *****************************************************************************************
 */
sdk_err_t tps_tx_power_level_set(int8_t tx_power_level);

/**
 *****************************************************************************************
 * @brief Provide the interface for other modules to obtain the tps service start handle .
 *
 * @return The tps service start handle.
 *****************************************************************************************
 */
uint16_t tps_service_start_handle_get(void);

/** @} */

#endif
/** @} */
/** @} */

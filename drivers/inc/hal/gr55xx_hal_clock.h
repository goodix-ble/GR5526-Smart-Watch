/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_clock.h
 * @author  BLE Driver Team
 * @brief   This file contains all the functions prototypes for the HAL
 *          module driver.
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
 ****************************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_CLOCK_H__
#define __GR55xx_HAL_CLOCK_H__

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"
#include <stdio.h>
#include <stdbool.h>

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup HAL_DRIVER HAL Driver
  * @{
  */

/** @defgroup HAL_CLK CLOCK
  * @brief CLOCK CALIBRATION HAL module driver.
  * @{
  */

/** @addtogroup HAL_CLK_ENUMERATIONS Enumerations
  * @{
  */
/**
  * @brief  XO-Requested Devices ID definition
  */
/** @{
  */
typedef enum
{
    XO_REQUEST_DEVICE_NUM_BLE = 0,
    XO_REQUEST_DEVICE_NUM_CALIBRATION, //NO.=1
    XO_REQUEST_DEVICE_NUM_USB,         //NO.=2
    XO_REQUEST_DEVICE_NUM_DDVS         //NO.=3
} xo_request_device_number_t;
/** @} */
/** @} */

/** @addtogroup HAL_CLK_FUNCTIONS Functions
 * @brief  xo clock request and release API
 * @{
 */

/**
 ****************************************************************************************
 * @brief get which devices requested xo clock
 *
 * @return xo_clock_requests, each bit corresponds to xo_request_device_number_t; value 1 means requests xo, and value 0 means no need xo.
 ****************************************************************************************
 */
uint32_t hal_clock_get_xo_requests(void);

/**
 ****************************************************************************************
 * @brief check the specified device whether request xo clock or not
 * @param[in] dev_num: specified device number
 * @return value 1 means requests xo, and value 0 means no need xo.
 ****************************************************************************************
 */
uint32_t hal_clock_get_device_xo_request_status(xo_request_device_number_t dev_num);

/**
 ****************************************************************************************
 * @brief the specified device request xo clock resource
 * @param[in] dev_num: specified device number
 * @return void
 ****************************************************************************************
 */
void hal_clock_request_xo_osc(xo_request_device_number_t dev_num);

/**
 ****************************************************************************************
 * @brief the specified device release xo clock resource
 * @param[in] dev_num: specified device number
 * @return void
 ****************************************************************************************
 */
void hal_clock_release_xo_osc(xo_request_device_number_t dev_num);

/**
 ****************************************************************************************
 * @brief release xo clock resource for all devices.
 * @return void
 ****************************************************************************************
 */
void hal_clock_release_xo_osc_all(void);
/** @} */
/** @} */

#endif /*__GR55xx_HAL_CLOCK_H__*/
/** @} */
/** @} */


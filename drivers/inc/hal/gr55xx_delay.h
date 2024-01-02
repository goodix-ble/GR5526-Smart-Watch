/**
 ****************************************************************************************
 *
 * @file gr55xx_delay.h
 *
 * @brief PERIPHERAL API DELAY DRIVER
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

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup HAL_DRIVER HAL Driver
  * @{
  */

/** @defgroup HAL_Delay HAL_Delay
  * @brief CGC HAL module driver.
  * @{
  */

#ifndef __GR55xx_DELAY_H__
#define __GR55xx_DELAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gr55xx.h"

/**
 ****************************************************************************************
 * @brief  Function for delaying execution for number of microseconds. GR55xxx is based on
 *         Cortex-M4, and this function is based on Data Watchpoint and Trace (DWT) unit.
 * @param[in] number_of_us: number of microseconds
 ****************************************************************************************
 */
void delay_us(uint32_t number_of_us);

/**
 ****************************************************************************************
 * @brief  Function for delaying execution for number of milliseconds. GR55xxx is based on
 *         Cortex-M4, and this function is based on Data Watchpoint and Trace (DWT) unit.
 * @param[in] number_of_ms: number of milliseconds
 ****************************************************************************************
 */
void delay_ms(uint32_t number_of_ms);

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_DELAY_H__ */
/** @} */

/** @} */

/** @} */

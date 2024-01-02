/**
 *******************************************************************************
 *
 * @file gr55xx_sys.h
 *
 * @brief GR55XX System API
 *
 *******************************************************************************
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
 @addtogroup SYSTEM
 @{
 */

/**
 * @addtogroup SYS System SDK
 * @{
 * @brief Definitions and prototypes for the system SDK interface.
*/

#ifndef __GR55XX_SYS_H__
#define __GR55XX_SYS_H__

#include "gr55xx_sys_cfg.h"
#include "gr55xx_nvds.h"
#include "gr55xx_pwr.h"
#include "gr5xx_fpb.h"
#include "ble.h"
#include "gr55xx_hal_adc.h"
#include "gr55xx_hal_exflash.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "gr55xx_sys_sdk.h"

/** @addtogroup GR55XX_SYS_DEFINES Defines
 * @{
 */
#define SYS_ROM_VERSION_ADDR          0x45000                  /**< The rom version address. */
/** @} */

/** @addtogroup GR55XX_SYS_FUNCTIONS Functions
 *  @{
 */
/**
 *****************************************************************************************
 * @brief app boot project turn on the encrypt clock.
 *
 *****************************************************************************************
 */
void app_boot_turn_on_encrypt_clock(void);

/**
 *****************************************************************************************
 * @brief app boot project set  the security clock.
 *
 *
 *****************************************************************************************
 */
void app_boot_security_clock_set(void);

/**
 *****************************************************************************************
 * @brief jump to app firmware.
 *
 * @param[in] fw_addr:     Firmware run address
 * @param[in] fw_bin_size: Firmware bin size
 *****************************************************************************************
 */
void sys_firmware_jump(uint32_t fw_addr, uint32_t fw_bin_size);

/** @} */
#endif

/** @} */
/** @} */


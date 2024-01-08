/**
 ****************************************************************************************
 *
 * @file bootloader_config.h
 *
 * @brief bootloader configuration file.
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

#ifndef _BOOTLOADER_CONFIG_H_
#define _BOOTLOADER_CONFIG_H_

#include <stdint.h>
#include "grx_sys.h"

// <o> Whether to enable port firmware verify and jump strategy
// <0=> Disable
// <1=> enable
#define BOOTLOADER_BOOT_PORT_ENABLE             0

// <o> Whether to enable the dfu port ble
// <0=> Disable
// <1=> enable
#define BOOTLOADER_DFU_BLE_ENABLE               1

// <o> Whether to enable the dfu port uart
// <0=> Disable
// <1=> enable
#define BOOTLOADER_DFU_UART_ENABLE              0

// <o> Whether to enable the dfu
// <0=> Disable
// <1=> enable
#define BOOTLOADER_DFU_ENABLE                   (BOOTLOADER_DFU_UART_ENABLE || BOOTLOADER_DFU_BLE_ENABLE)

// <o> Whether to enable the watchdog
// <0=> Disable
// <1=> enable
#define BOOTLOADER_WDT_ENABLE                   0

// <o> Whether to enable the signature verification function
// <0=> Disable
// <1=> enable
#define BOOTLOADER_SIGN_ENABLE                  0

// Application firmware comments definition
// Must match the user app
#define APP_FW_COMMENTS                         "ble_app_temp"

// <o> copy load address
// <i> Default:  0x01040000
#define DFU_FW_SAVE_ADDR                        (FLASH_START_ADDR + 0x40000)

//Hash value of the signed public key
#define BOOTLOADER_PUBLIC_KEY_HASH              0xFE,0xD2,0x4B,0xEF,0x11,0x7C,0xB9,0xF9,0x6A,0x5A,0x6D,0xF7,0xEF,0xE0,0xA4,0xFC

#endif




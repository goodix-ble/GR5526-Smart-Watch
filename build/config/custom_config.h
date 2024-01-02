/**
 ****************************************************************************************
 *
 * @file custom_config.h
 *
 * @brief Custom configuration file for applications.
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

/*
 * DEFINES
 *****************************************************************************************
 */
#ifndef __CUSTOM_CONFIG_H__
#define __CUSTOM_CONFIG_H__

// <<< Use Configuration Wizard in Context Menu >>>

// <h> Basic configuration

// <o> Chip version
#ifndef SOC_GR5526
#define SOC_GR5526
#endif

// <o> Enable system fault trace module
// <0=> DISABLE
// <1=> ENABLE
#ifndef SYS_FAULT_TRACE_ENABLE
#define SYS_FAULT_TRACE_ENABLE  1
#endif

// <o> Enable callstack backtrace function
// <i> Default: 0
#ifndef ENABLE_BACKTRACE_FEA
#define ENABLE_BACKTRACE_FEA    0
#endif

// <o> Enable app driver module
// <0=> DISABLE
// <1=> ENABLE
#ifndef APP_DRIVER_USE_ENABLE
#define APP_DRIVER_USE_ENABLE   1
#endif

// <o> Eanble APP log module
// <0=> DISABLE
// <1=> ENABLE
#ifndef APP_LOG_ENABLE
#define APP_LOG_ENABLE          1
#endif

// <o> APP log port type
// <0=> UART
// <1=> RTT
// <2=> ITM
#ifndef APP_LOG_PORT
#define APP_LOG_PORT            0
#endif

// <o> Eanble APP log store module
// <0=> DISABLE
// <1=> ENABLE
#ifndef APP_LOG_STORE_ENABLE
#define APP_LOG_STORE_ENABLE    0
#endif

// <o> Enable SK GUI module
// <0=> DISABLE
// <1=> ENABLE
#ifndef SK_GUI_ENABLE
#define SK_GUI_ENABLE           0
#endif

// <o> Enable DTM test support
// <0=> DISABLE
// <1=> ENABLE
#ifndef DTM_TEST_ENABLE
#define DTM_TEST_ENABLE         0
#endif

// <o> Enable PMU Calibration
// <0=> DISABLE
// <1=> ENABLE
#ifndef PMU_CALIBRATION_ENABLE
#define PMU_CALIBRATION_ENABLE  1
#endif

// <o> Protection priority level
// <i> Default:  0
#ifndef FLASH_PROTECT_PRIORITY
#define FLASH_PROTECT_PRIORITY  0
#endif

// <o> NVDS Start Address
// <i> Default:  0x010FF000
//#ifndef NVDS_START_ADDR
//#define NVDS_START_ADDR         0x0027E000
//#endif

// <o> The Number of sectors for NVDS
// <i> Default:  1
#ifndef NVDS_NUM_SECTOR
#define NVDS_NUM_SECTOR         1
#endif

// <o> Call Stack Size
// <i> Default: 0x3000
#ifndef SYSTEM_STACK_SIZE
#define SYSTEM_STACK_SIZE       0x3000
#endif

// <o> Call Heap Size
// <i> Default: 0x0
#ifndef SYSTEM_HEAP_SIZE
#define SYSTEM_HEAP_SIZE        0x4000
#endif

// </h>

// <h> Boot info configuration

// <o> Chip version
// <i> Default: 0x00
#ifndef CHIP_VER
#define CHIP_VER                0x5526
#endif

// <o> Code load address
// <0x20006000=> SRAM address
// <0x00202000=> Flash address
// <i> Default:  0x00202000(Flash)
#ifndef APP_CODE_LOAD_ADDR
#define APP_CODE_LOAD_ADDR      0x00202000
#endif

// <o> Code run address
// <0x20006000=> SRAM address
// <0x00202000=> Flash address
// <i> Default:  0x00202000(Flash XIP)
#ifndef APP_CODE_RUN_ADDR
#define APP_CODE_RUN_ADDR       0x00202000
#endif

// <o> Additional RAM area
// <0x1C000000=> EXT_RAM1_STRAT_ADDR
// <0x20080000=> EXT_RAM2_STRAT_ADDR
#define EXT_RAM1_STRAT_ADDR       0x1C000000
#define EXT_RAM2_STRAT_ADDR       0x20080000

// <ol.0..5> System clock
// <0=> 96MHZ
// <1=> 64MHZ
// <3=> 48MHZ
// <2=> 16MHZ-XO
// <4=> 24MHZ
// <5=> 16MHZ
// <6=> 32MHZ-CPLL
#ifndef SYSTEM_CLOCK
#define SYSTEM_CLOCK            0
#endif

// <o> External clock accuracy used in the LL to compute timing  <1-500>
// <i> Range: 1-500
#ifndef CFG_LF_ACCURACY_PPM
#define CFG_LF_ACCURACY_PPM     500
#endif

// <o> Enable internal osc as low power clock
// <0=> Default: Disable internal osc as low power clock
// <1=> Enable internal osc as low power clock and force CFG_LF_ACCURACY_PPM to 500ppm
#ifndef CFG_LPCLK_INTERNAL_EN
#define CFG_LPCLK_INTERNAL_EN   0
#endif

// <o> Delay time for Boot startup
// <0=> Not Delay
// <1=> Delay 500ms
#ifndef BOOT_LONG_TIME
#define BOOT_LONG_TIME          1
#endif

// <o> In xip mode, check image during cold boot startup
// <0=> Not check
// <1=> Check image
#ifndef BOOT_CHECK_IMAGE
#define BOOT_CHECK_IMAGE        1
#endif

// <o> Code version.16bits
#ifndef VERSION
#define VERSION                 1
#endif

// </h>

// <h> BLE resource configuration
// <i> Note: The total number of BLE Activities(CONNECTIONS+ADVS+SCAN) should not exceed the limit 12.

// <o>  Support maximum number of BLE profiles <1-64>
// <i>  Range: 1-64
#ifndef CFG_MAX_PRFS
#define CFG_MAX_PRFS             10
#endif

// <o>  Support maximum number of bonded devices
#ifndef CFG_MAX_BOND_DEVS
#define CFG_MAX_BOND_DEVS        4
#endif

// <o>  Support maximum number of BLE Links <1-10>
// <i>  Range: 1-10
#ifndef CFG_MAX_CONNECTIONS
#define CFG_MAX_CONNECTIONS      5
#endif

// <o>  Support maximum number of BLE Legacy/Extended Advertisings <0-5>
// <i>  Range: 0-5
// <i>  Note: The total number of BLE Legacy/Extended/Periodic Advertisings should not exceed the limit 5.
#ifndef CFG_MAX_ADVS
#define CFG_MAX_ADVS             1
#endif

// <o>  Support maximum number of BLE Scan <0-1>
// <i>  Range: 0-1
#ifndef CFG_MAX_SCAN
#define CFG_MAX_SCAN             1
#endif

// <o>  Support multiple link with the same device
// <0=> NOT SUPPORT
// <1=> SUPPORT
#ifndef CFG_MUL_LINK_WITH_SAME_DEV
#define CFG_MUL_LINK_WITH_SAME_DEV        0
#endif

// <o>  Support BT-BR/EDR
// <0=> NOT SUPPORT
// <1=> SUPPORT
#ifndef CFG_BT_BREDR
#define CFG_BT_BREDR                      0
#endif

// <o>  Support car key needs
// <0=> NOT SUPPORT
// <1=> SUPPORT
#ifndef CFG_CAR_KEY_SUPPORT
#define CFG_CAR_KEY_SUPPORT               0
#endif

// <o>  BLE controller only(use for extern host or HCI uart transport)
// <0=> Support BLE controller and host
// <1=> Support BLE controller only
#ifndef CFG_CONTROLLER_ONLY
#define CFG_CONTROLLER_ONLY       0
#endif

// <o>  Support maximum number of BLE Periodic Advertisings <0-5>
// <i>  Range: 0-5
// <i>  Note: The total number of BLE Legacy/Extended/Periodic Advertisings should not exceed the limit 5.
#ifndef CFG_MAX_PER_ADVS
#define CFG_MAX_PER_ADVS          0
#endif

// <o>  Support maximum number of BLE Periodic Advertising Synchronizations <0-5>
// <i>  Range: 0-5
#ifndef CFG_MAX_SYNCS
#define CFG_MAX_SYNCS             0
#endif

// <o>  Support EATT feature
// <0=> NOT SUPPORT
// <1=> SUPPORT
#ifndef CFG_EATT_SUPPORT
#define CFG_EATT_SUPPORT                0
#endif

// <o>  Support maximum number of BLE EATT Channels <0-10>
// <i>  Range: 0-10
#ifndef CFG_MAX_EATT_CHANNELS
#define CFG_MAX_EATT_CHANNELS           0
#endif

// <o>  Support ISO
// <0=> NOT SUPPORT
// <1=> SUPPORT
#ifndef CFG_ISO_SUPPORT
#define CFG_ISO_SUPPORT                 0
#endif

// </h>
// <h>  LCP support configuration
// <o>  Support LCP
// <0=> NOT SUPPORT
// <1=> SUPPORT
#ifndef CFG_LCP_SUPPORT
#define CFG_LCP_SUPPORT           0
#endif
// </h>

// <h>  Security configuration
// <o>  Algorithm security level
// <0=> Enable algorithm level one
// <1=> Enable algorithm level two
#ifndef SECURITY_CFG_VAL
#define SECURITY_CFG_VAL         0
#endif
// </h>

// <<< end of configuration section >>>
#endif //__CUSTOM_CONFIG_H__

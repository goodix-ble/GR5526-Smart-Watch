/**
 *****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief  User Periph Init Function Implementation.
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
 
/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "gr_includes.h"
#include "custom_config.h"
#include "bootloader_config.h"
#include "otas.h"
#include "app_log.h"
#include "dfu_port.h"
#include "user_periph_setup.h"
#include "board_SK.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#if BOOTLOADER_WDT_ENABLE
static aon_wdt_handle_t     bootloader_wdt_handle;
#endif

#if BOOTLOADER_DFU_ENABLE
static void dfu_program_start_callback(void);
static void dfu_programing_callback(uint8_t pro);
static void dfu_program_end_callback(uint8_t status);

static dfu_pro_callback_t dfu_pro_call =
{
    .dfu_program_start_callback = dfu_program_start_callback,
    .dfu_programing_callback    = dfu_programing_callback,
    .dfu_program_end_callback   = dfu_program_end_callback,
};


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void dfu_program_start_callback(void)
{
    APP_LOG_DEBUG("    Start DFU OTA.");
}

static void dfu_programing_callback(uint8_t pro)
{
    APP_LOG_DEBUG("    DFU OTA.... %d%%", pro);
}

static void dfu_program_end_callback(uint8_t status)
{
    APP_LOG_DEBUG("    DFU OTA complete.");
}
#endif


/**
 *****************************************************************************************
 * @brief Initialize watch dog.
 *****************************************************************************************
 */
static void bootloader_wdt_init(void)
{
#if BOOTLOADER_WDT_ENABLE
    bootloader_wdt_handle.init.counter = 32768 * 20;
    bootloader_wdt_handle.init.alarm_counter = 0;

    hal_aon_wdt_init(&bootloader_wdt_handle);
    
    SystemCoreUpdateClock();
    SysTick_Config(SystemCoreClock/10);
    hal_nvic_enable_irq(SysTick_IRQn);
#endif
}



void app_periph_init(void)
{
#ifndef SOC_GR5332
    // Turn on the clock of encryption module.
    app_boot_turn_on_encrypt_clock();
#endif

    bootloader_wdt_init();

#if APP_LOG_ENABLE
    bsp_log_init();
#endif

    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);

#if BOOTLOADER_DFU_ENABLE
    dfu_port_init(NULL, DFU_FW_SAVE_ADDR, &dfu_pro_call);
#endif
}

void bootloader_wdt_refresh(void)
{
#if BOOTLOADER_WDT_ENABLE
    hal_aon_wdt_refresh(&bootloader_wdt_handle);
#endif
}

void cortex_backtrace_fault_handler(void)
{
    hal_nvic_system_reset();
    while(1);
}

#if BOOTLOADER_WDT_ENABLE
void SysTick_Handler(void)
{
    bootloader_wdt_refresh();
}
#endif





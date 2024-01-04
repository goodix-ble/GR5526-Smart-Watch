/**
 *****************************************************************************************
 *
 * @file bootloader_boot_vendor.c
 *
 * @brief Bootloader Boot Vendor Function Implementation.
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
#include "bootloader_config.h"
#if !BOOTLOADER_BOOT_PORT_ENABLE
#include "bootloader_boot.h"
#include "bootloader_ota.h"

#include "user_periph_setup.h"
#include "hal_flash.h"
#include "app_log.h"
#include "dfu_port.h"
#include "sign_verify.h"
#include "grx_sys.h"
#include "grx_hal.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define SCA_BOOT_INFO_ADDR         (FLASH_START_ADDR + 0x0000UL)
#define SCA_IMG_INFO_ADDR          (SCA_BOOT_INFO_ADDR + 0x40)
#define SCA_IMG_INF_NUM_MAX        10

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static dfu_boot_info_t      s_bootloader_info;
static dfu_info_t           s_dfu_info;
static bool                 s_is_app_fw_valid;
static dfu_image_info_t     s_app_img_info;
static uint8_t              s_flash_read_buff[DFU_FLASH_SECTOR_SIZE];

#ifndef SOC_GR5332
static bool                 s_flash_security_status = false;
#endif

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void fw_boot_info_print(dfu_boot_info_t *p_boot_info)
{
    if (NULL == p_boot_info)
    {
        return;
    }

    APP_LOG_DEBUG("    Load Address = 0x%08x", p_boot_info->load_addr);
    APP_LOG_DEBUG("    Run Address  = 0x%08x", p_boot_info->run_addr);
    APP_LOG_DEBUG("    Bin Size     = 0x%08x", p_boot_info->bin_size);
    APP_LOG_DEBUG("    CheckSum     = 0x%08x", p_boot_info->check_sum);
}

static void security_disable(void)
{
#ifndef SOC_GR5332
    uint32_t sys_security = sys_security_enable_status_check();
    if(sys_security)
    {
        s_flash_security_status = hal_flash_get_security();
        hal_flash_set_security(false);
    }
#endif
}

static void security_state_recovery(void)
{
#ifndef SOC_GR5332
    uint32_t sys_security = sys_security_enable_status_check();
    if(sys_security)
    {
        hal_flash_set_security(s_flash_security_status);
    }
#endif
}

static bool check_image_crc(const uint8_t * p_data, uint32_t len, uint32_t check)
{

#if defined(SOC_GR5526) && (DFU_SUPPORT_EXTERN_FLASH_FOR_GR5526 > 0u)
    uint32_t sum      = 0;
    uint32_t once_len = 0;
    uint32_t cksum    = 0;
    const uint32_t once_max   = sizeof(s_flash_read_buff);
    const uint32_t start_addr = (uint32_t)p_data;

    while(sum < len) {

        once_len = ((len - sum) >= once_max) ? once_max : (len - sum);
        dfu_portable_flash_read(start_addr + sum, s_flash_read_buff, once_len);
        sum += once_len;

        for(int i = 0; i < once_len; i++) {
            cksum += s_flash_read_buff[i];
        }
    }

    if(cksum == check) {
        return true;
    }
    APP_LOG_ERROR("    CheckSum Error, [0x%08x : %d] Expected: 0x%08x, Actual:0x%08x", start_addr, len, check, cksum);

    return false;

#else
    uint32_t cksum=0;

    for (int i = 0; i < len; i++)
    {
        cksum += p_data[i];
    }

    return (check == cksum);
#endif
}

static uint32_t hal_flash_read_judge_security(const uint32_t addr, uint8_t *buf, const uint32_t size)
{
    uint32_t read_bytes = 0;
    security_disable();
    read_bytes = hal_flash_read(addr, buf, size);
    security_state_recovery();
    return read_bytes;
}

static bool bootloader_firmware_verify(uint32_t bin_addr, uint32_t bin_size, uint32_t check_sum_store)
{
    extern bool check_image_crc(const uint8_t * p_data, uint32_t len, uint32_t check);

#ifndef SOC_GR5332
    if (!sys_security_enable_status_check() && !check_image_crc((uint8_t *)bin_addr, bin_size, check_sum_store))
    {
        APP_LOG_DEBUG("    Firmware checksum invalid");
        return false;
    }

    #if BOOTLOADER_SIGN_ENABLE
        uint8_t public_key_hash[] = {BOOTLOADER_PUBLIC_KEY_HASH};
        security_disable();
        if (!sign_verify(bin_addr, bin_size, public_key_hash, sys_security_enable_status_check()))
        {
            security_state_recovery();
            APP_LOG_DEBUG("    Signature verify check fail.");
            return false;
        }
        security_state_recovery();
        APP_LOG_DEBUG("    Signature verify check success.");
    #endif

#else

    if (!check_image_crc((uint8_t *)bin_addr, bin_size, check_sum_store))
    {
        APP_LOG_DEBUG("    Firmware checksum invalid");
        return false;
    }
#endif

    return true;
}

static void bootloader_fw_boot_info_get(void)
{
    if (sizeof(dfu_boot_info_t) != hal_flash_read(SCA_BOOT_INFO_ADDR, (uint8_t *)&s_bootloader_info, sizeof(dfu_boot_info_t)))
    {
        APP_LOG_DEBUG("    Bootloader firmware boot info get fail!");
    }

    fw_boot_info_print(&s_bootloader_info);
}


static bool bootloader_app_dfu_fw_verify(void)
{

    if (s_dfu_info.dfu_img_info.boot_info.load_addr < (s_bootloader_info.load_addr + s_bootloader_info.bin_size))
    {
        APP_LOG_DEBUG("    App dfu firmware load address overload bootloader firmware, 0x%08x, 0x%08x, %d",

        s_dfu_info.dfu_img_info.boot_info.load_addr, s_bootloader_info.load_addr, s_bootloader_info.bin_size);
        return false;
    }

    return bootloader_firmware_verify(s_dfu_info.dfu_fw_save_addr, s_dfu_info.dfu_img_info.boot_info.bin_size, s_dfu_info.dfu_img_info.boot_info.check_sum);
}

static bool bootloader_dfu_info_verify(void)
{
    if (sizeof(s_dfu_info) != hal_flash_read_judge_security(DFU_INFO_START_ADDR, (uint8_t *)&s_dfu_info, sizeof(dfu_info_t)))
    {
        APP_LOG_DEBUG("    Bootloader dfu info get fail!");
        return false;
    }

    if (s_dfu_info.dfu_mode_pattern == DFU_COPY_UPGRADE_MODE_PATTERN)
    {
        APP_LOG_DEBUG("    Double bank, Background DFU Mode");
    }
    else if (s_dfu_info.dfu_mode_pattern == DFU_NON_COPY_UPGRADE_MODE_PATTERN)
    {
        APP_LOG_DEBUG("    Single bank, Non-background DFU Mode");
    }
    else
    {
        return false;
    }

    return true;
}

static void bootloader_dfu_info_clear(void)
{
    hal_flash_erase(DFU_INFO_START_ADDR, DFU_FLASH_SECTOR_SIZE);
}

SECTION_RAM_CODE static void bootloader_dfu_fw_copy(uint32_t dst_addr, uint32_t src_addr, uint32_t size)
{
    uint16_t   copy_page = 0;
    uint16_t   remain    = 0;
    uint32_t   temp_size = 0;
    uint16_t   copy_size = 0;

    temp_size = size + 48;

    APP_LOG_DEBUG("    App dfu firmware start copy.");

#ifndef SOC_GR5332
    if (sys_security_enable_status_check())
    {
        temp_size += 856;
    }
    else
    {
#if BOOTLOADER_SIGN_ENABLE
        temp_size += 856;
#endif
    }
#endif

    copy_page = temp_size / DFU_FLASH_SECTOR_SIZE;
    remain    = temp_size % DFU_FLASH_SECTOR_SIZE;

    if (remain)
    {
        copy_page++;
    }
#if !(defined(SOC_GR5526) && (DFU_SUPPORT_EXTERN_FLASH_FOR_GR5526 > 0u))
    __disable_irq();
#endif
    bootloader_wdt_refresh();

    security_disable();
    for (uint16_t i = 0; i < copy_page; i++)
    {
        if (i == copy_page - 1 && remain)
        {
            copy_size = remain;
        }
        else
        {
            copy_size = DFU_FLASH_SECTOR_SIZE;
        }
        hal_flash_erase(dst_addr + i * DFU_FLASH_SECTOR_SIZE, DFU_FLASH_SECTOR_SIZE);
#if defined(SOC_GR5526) && (DFU_SUPPORT_EXTERN_FLASH_FOR_GR5526 > 0u)
        dfu_portable_flash_read(src_addr + i * DFU_FLASH_SECTOR_SIZE, s_flash_read_buff, copy_size);
#else
        hal_flash_read(src_addr + i * DFU_FLASH_SECTOR_SIZE, s_flash_read_buff, copy_size);
#endif
        hal_flash_write(dst_addr + i * DFU_FLASH_SECTOR_SIZE, s_flash_read_buff, copy_size);
    }
    security_state_recovery();

    bootloader_wdt_refresh();
}

static void bootloader_app_img_info_update(dfu_image_info_t *p_img_info)
{
    security_disable();
    hal_flash_erase(APP_INFO_START_ADDR, DFU_FLASH_SECTOR_SIZE);
    hal_flash_write(APP_INFO_START_ADDR, (uint8_t *)p_img_info, sizeof(dfu_image_info_t));
    security_state_recovery();
}

static bool bootloader_app_fw_verify()
{
    memset((uint8_t*)&s_app_img_info, 0, sizeof(s_app_img_info));

    hal_flash_read_judge_security(APP_INFO_START_ADDR, (uint8_t*)&s_app_img_info, sizeof(s_app_img_info));

    if (0 == memcmp(s_app_img_info.comments, APP_FW_COMMENTS, strlen(APP_FW_COMMENTS)) &&
        bootloader_firmware_verify(s_app_img_info.boot_info.load_addr, s_app_img_info.boot_info.bin_size, s_app_img_info.boot_info.check_sum))
    {
        APP_LOG_DEBUG("    Found app firmware image info in APP INFO AREA");
        return true;
    }

    for (uint8_t i = 0; i < SCA_IMG_INF_NUM_MAX; i++)
    {
        hal_flash_read(i * sizeof(s_app_img_info) + SCA_IMG_INFO_ADDR, (uint8_t *)&s_app_img_info, sizeof(s_app_img_info));

        if (0 == memcmp(s_app_img_info.comments, APP_FW_COMMENTS, strlen(APP_FW_COMMENTS)) &&
            bootloader_firmware_verify(s_app_img_info.boot_info.load_addr, s_app_img_info.boot_info.bin_size, s_app_img_info.boot_info.check_sum))
        {
            bootloader_app_img_info_update(&s_app_img_info);
            APP_LOG_DEBUG("    Found app firmware image info in SYSTEM CONFIG AREA");
            APP_LOG_DEBUG("    Update app firmware image info to APP INFO AREA");
            return true;
        }
    }

    return false;
}

static void app_bootloader_jump(dfu_boot_info_t *p_boot_info)
{
    if(p_boot_info->run_addr != p_boot_info->load_addr)//mirror mode
    {
        memcpy((uint8_t*)p_boot_info->run_addr, (uint8_t*)p_boot_info->load_addr, p_boot_info->bin_size);
    }

    sys_firmware_jump(p_boot_info->run_addr, p_boot_info->bin_size);
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void bootloader_dfu_task(void)
{
    APP_LOG_DEBUG(">>> Get bootloader firmware boot info");
    bootloader_fw_boot_info_get();

    APP_LOG_DEBUG(">>> Verify DFU info");
    if (!bootloader_dfu_info_verify())
    {
        APP_LOG_DEBUG(">>> There is no DFU task need to do");
        return;
    }

    if (s_dfu_info.dfu_mode_pattern == DFU_COPY_UPGRADE_MODE_PATTERN)
    {
        APP_LOG_DEBUG(">>> Verify app dfu firmware");

        if (!bootloader_app_dfu_fw_verify())
        {
            APP_LOG_DEBUG(">>> Clear dfu info");
            bootloader_dfu_info_clear();
            return;
        }

        APP_LOG_DEBUG(">>> Copy app dfu firmware from bank1 to bank0");
        bootloader_dfu_fw_copy(s_dfu_info.dfu_img_info.boot_info.load_addr, s_dfu_info.dfu_fw_save_addr, s_dfu_info.dfu_img_info.boot_info.bin_size);

        APP_LOG_DEBUG(">>> Update app image info");
        bootloader_app_img_info_update(&s_dfu_info.dfu_img_info);

        APP_LOG_DEBUG(">>> Clear DFU info");
        bootloader_dfu_info_clear();

        APP_LOG_DEBUG(">>> Reset Device");
        hal_nvic_system_reset();
    }

    if (s_dfu_info.dfu_mode_pattern == DFU_NON_COPY_UPGRADE_MODE_PATTERN)
    {
        #if BOOTLOADER_DFU_BLE_ENABLE
            APP_LOG_DEBUG(">>> Start OTA task");
            bootloader_ota_task();
        #endif
    }
}

void bootloader_verify_task(void)
{
#if BOOTLOADER_DFU_ENABLE
    if (s_dfu_info.dfu_mode_pattern == DFU_NON_COPY_UPGRADE_MODE_PATTERN)
    {
        return;
    }
#endif

    APP_LOG_DEBUG(">>> Verify app firmware");
    s_is_app_fw_valid = bootloader_app_fw_verify();
}

void bootloader_jump_task(void)
{
#if BOOTLOADER_DFU_ENABLE
    if (s_dfu_info.dfu_mode_pattern == DFU_NON_COPY_UPGRADE_MODE_PATTERN)
    {
        return;
    }
#endif

    if (s_is_app_fw_valid)
    {
        APP_LOG_DEBUG(">>> Jump to app firmware.");
        app_bootloader_jump(&s_app_img_info.boot_info);
    }
    else
    {
        #if BOOTLOADER_DFU_BLE_ENABLE
            APP_LOG_DEBUG(">>> Start OTA task");
            bootloader_ota_task();
        #endif

        #if !BOOTLOADER_DFU_ENABLE
            APP_LOG_DEBUG(">>> No enable DFU task, so enter while loop");
        #endif
    }
}
#endif


/**
  ****************************************************************************************
  * @file    app_qspi.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */

#include "app_assert.h"
#include "app_qspi.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "platform_sdk.h"
#include "app_drv.h"
#include "gr_soc.h"
#include <string.h>

#ifdef HAL_QSPI_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
#define QSPI_SMART_CS_LOW(id)                                           \
    do {                                                                \
            if(p_qspi_env[id]->p_pin_cfg->cs.enable == APP_QSPI_PIN_ENABLE) \
            {                                                           \
                app_io_write_pin(p_qspi_env[id]->p_pin_cfg->cs.type,    \
                                p_qspi_env[id]->p_pin_cfg->cs.pin,      \
                                APP_IO_PIN_RESET);                      \
            }                                                           \
        } while(0)

#define QSPI_SMART_CS_HIGH(id)                                          \
    do {                                                                \
            if(p_qspi_env[id]->p_pin_cfg->cs.enable == APP_QSPI_PIN_ENABLE) \
            {                                                           \
                app_io_write_pin(p_qspi_env[id]->p_pin_cfg->cs.type,    \
                                 p_qspi_env[id]->p_pin_cfg->cs.pin,     \
                                 APP_IO_PIN_SET);                       \
            }                                                           \
    } while(0)
#else
#define QSPI_SMART_CS_LOW(id)
#define QSPI_SMART_CS_HIGH(id)
#endif

#define REG(x)                              (*(volatile uint32_t*)(x))

#define APP_QSPI_EXCEPT_DEBUG_EN            1u

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
/********************************************************************
 * QUAD_WRITE_32b_PATCH : just exist in QUAD/DATASIZE_32BITS/DMA scene
 *   if enable, MUST Control the CS By Software.
 */
#define QSPI_QUAD_WRITE_32b_PATCH_EN        0u

/********************************************************************
 * DATA Endian Mode Optional Value :
 *   0 : data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
 *   1 : data[1] | (data[0] << 8) | (data[3] << 16) | (data[2] << 24)
 *   2 : data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24)
 *   3 : data[2] | (data[3] << 8) | (data[0] << 16) | (data[1] << 24)
 */
#define QSPI_QUAD_WRITE_DATA_ENDIAN_MODE    0u
#endif
/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
bool qspi_prepare_for_sleep(void);
void qspi_wake_up_ind(void);
static uint16_t qspi_gpio_config(app_qspi_pin_cfg_t *p_pin_cfg);
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
void app_qspi_force_cs(app_qspi_id_t screen_id, bool low_level);
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
extern volatile bool is_dma_access;
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
const qspi_memorymapped_t g_flash_typical_mmap_read_cmd[FLASH_MMAP_CMD_READ_MAX] =
{
    /**************************************************************
     * FOLLOWING DEFINES ARE TYPICAL FLASH CMDs,
     * SUPPORTED BY MANY FLASH DEVICES,
     * IF DEFINES ARE NOT MATCHING WITH THE USED FLASH,
     * PLEASE MODIFY CAREFULLY
     **************************************************************/
    [FLASH_MMAP_CMD_DREAD_3BH] = {
        .x_endian_mode                  = QSPI_CONCURRENT_XIP_ENDIAN_MODE_0,
        .x_prefetch_en                  = QSPI_CONCURRENT_XIP_PREFETCH_DISABLE,
        .x_continous_xfer_en            = QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE,
        .x_instruction_en               = QSPI_CONCURRENT_XIP_INST_ENABLE,
        .x_instruction_size             = QSPI_CONCURRENT_XIP_INSTSIZE_8BIT,
        .x_address_size                 = QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT,
        .x_inst_addr_transfer_format    = QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPI,
        .x_mode_bits_en                 = QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE,
        .x_mode_bits_length             = QSPI_CONCURRENT_XIP_MBL_8,
        .x_mode_bits_data               = 0x00,
        .x_dummy_cycles                 = 8,
        .x_continous_xfer_toc           = 0,
        .x_sioo_mode                    = QSPI_CONCURRENT_XIP_INST_SENT_EVERY_ACCESS,
        .x_data_frame_format            = QSPI_CONCURRENT_XIP_FRF_DUAL_SPI,
        .x_instruction                  = 0x3B,
    },
    [FLASH_MMAP_CMD_2READ_BBH] = {
        .x_endian_mode                  = QSPI_CONCURRENT_XIP_ENDIAN_MODE_0,
        .x_prefetch_en                  = QSPI_CONCURRENT_XIP_PREFETCH_DISABLE,
        .x_continous_xfer_en            = QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE,
        .x_instruction_en               = QSPI_CONCURRENT_XIP_INST_ENABLE,
        .x_instruction_size             = QSPI_CONCURRENT_XIP_INSTSIZE_8BIT,
        .x_address_size                 = QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT,
        .x_inst_addr_transfer_format    = QSPI_CONCURRENT_XIP_INST_IN_SPI_ADDR_IN_SPIFRF,
        .x_mode_bits_en                 = QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE,
        .x_mode_bits_length             = QSPI_CONCURRENT_XIP_MBL_8,
        .x_mode_bits_data               = 0x00,
        .x_dummy_cycles                 = 4,
        .x_continous_xfer_toc           = 0,
        .x_sioo_mode                    = QSPI_CONCURRENT_XIP_INST_SENT_EVERY_ACCESS,
        .x_data_frame_format            = QSPI_CONCURRENT_XIP_FRF_DUAL_SPI,
        .x_instruction                  = 0xBB,
    },
    [FLASH_MMAP_CMD_2READ_BBH_SIOO] = {
        .x_endian_mode                  = QSPI_CONCURRENT_XIP_ENDIAN_MODE_0,
        .x_prefetch_en                  = QSPI_CONCURRENT_XIP_PREFETCH_DISABLE,
        .x_continous_xfer_en            = QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE,
        .x_instruction_en               = QSPI_CONCURRENT_XIP_INST_ENABLE,
        .x_instruction_size             = QSPI_CONCURRENT_XIP_INSTSIZE_8BIT,
        .x_address_size                 = QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT,
        .x_inst_addr_transfer_format    = QSPI_CONCURRENT_XIP_INST_IN_SPI_ADDR_IN_SPIFRF,
        .x_mode_bits_en                 = QSPI_CONCURRENT_XIP_MODE_BITS_ENABLE,
        .x_mode_bits_length             = QSPI_CONCURRENT_XIP_MBL_8,
        .x_mode_bits_data               = 0x20,
        .x_dummy_cycles                 = 0,
        .x_continous_xfer_toc           = 0,
        .x_sioo_mode                    = QSPI_CONCURRENT_XIP_INST_SENT_ONLY_FIRST_ACCESS,
        .x_data_frame_format            = QSPI_CONCURRENT_XIP_FRF_DUAL_SPI,
        .x_instruction                  = 0xBB,
    },
    [FLASH_MMAP_CMD_QREAD_6BH] = {
        .x_endian_mode                  = QSPI_CONCURRENT_XIP_ENDIAN_MODE_0,
        .x_prefetch_en                  = QSPI_CONCURRENT_XIP_PREFETCH_DISABLE,
        .x_continous_xfer_en            = QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE,
        .x_instruction_en               = QSPI_CONCURRENT_XIP_INST_ENABLE,
        .x_instruction_size             = QSPI_CONCURRENT_XIP_INSTSIZE_8BIT,
        .x_address_size                 = QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT,
        .x_inst_addr_transfer_format    = QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPI,
        .x_mode_bits_en                 = QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE,
        .x_mode_bits_length             = QSPI_CONCURRENT_XIP_MBL_8,
        .x_mode_bits_data               = 0x20,
        .x_dummy_cycles                 = 8,
        .x_continous_xfer_toc           = 0,
        .x_sioo_mode                    = QSPI_CONCURRENT_XIP_INST_SENT_EVERY_ACCESS,
        .x_data_frame_format            = QSPI_CONCURRENT_XIP_FRF_QUAD_SPI,
        .x_instruction                  = 0x6B,
    },
    [FLASH_MMAP_CMD_4READ_EBH] = {
        .x_endian_mode                  = QSPI_CONCURRENT_XIP_ENDIAN_MODE_0,
        .x_prefetch_en                  = QSPI_CONCURRENT_XIP_PREFETCH_DISABLE,
        .x_continous_xfer_en            = QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE,
        .x_instruction_en               = QSPI_CONCURRENT_XIP_INST_ENABLE,
        .x_instruction_size             = QSPI_CONCURRENT_XIP_INSTSIZE_8BIT,
        .x_address_size                 = QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT,
        .x_inst_addr_transfer_format    = QSPI_CONCURRENT_XIP_INST_IN_SPI_ADDR_IN_SPIFRF,
        .x_mode_bits_en                 = QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE,
        .x_mode_bits_length             = QSPI_CONCURRENT_XIP_MBL_8,
        .x_mode_bits_data               = 0x00,
        .x_dummy_cycles                 = 6,
        .x_continous_xfer_toc           = 0,
        .x_sioo_mode                    = QSPI_CONCURRENT_XIP_INST_SENT_EVERY_ACCESS,
        .x_data_frame_format            = QSPI_CONCURRENT_XIP_FRF_QUAD_SPI,
        .x_instruction                  = 0xEB,
    },
    [FLASH_MMAP_CMD_4READ_EBH_SIOO] = {
        .x_endian_mode                  = QSPI_CONCURRENT_XIP_ENDIAN_MODE_0,
        .x_prefetch_en                  = QSPI_CONCURRENT_XIP_PREFETCH_DISABLE,
        .x_continous_xfer_en            = QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE,
        .x_instruction_en               = QSPI_CONCURRENT_XIP_INST_ENABLE,
        .x_instruction_size             = QSPI_CONCURRENT_XIP_INSTSIZE_8BIT,
        .x_address_size                 = QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT,
        .x_inst_addr_transfer_format    = QSPI_CONCURRENT_XIP_INST_IN_SPI_ADDR_IN_SPIFRF,
        .x_mode_bits_en                 = QSPI_CONCURRENT_XIP_MODE_BITS_ENABLE,
        .x_mode_bits_length             = QSPI_CONCURRENT_XIP_MBL_8,
        .x_mode_bits_data               = 0x20,
        .x_dummy_cycles                 = 4,
        .x_continous_xfer_toc           = 0,
        .x_sioo_mode                    = QSPI_CONCURRENT_XIP_INST_SENT_ONLY_FIRST_ACCESS,
        .x_data_frame_format            = QSPI_CONCURRENT_XIP_FRF_QUAD_SPI,
        .x_instruction                  = 0xEB,
    }
};

const qspi_memorymapped_t g_psram_typical_mmap_qread_cmd[PSRAM_MMAP_CMD_READ_MAX] =
{
    /**************************************************************
     * IF DEFINES ARE NOT MATCHING WITH THE USED PSRAM,
     * PLEASE MODIFY CAREFULLY
     **************************************************************/
    [PSRAM_MMAP_CMD_QREAD_0BH] = {
        .x_endian_mode                  = QSPI_CONCURRENT_XIP_ENDIAN_MODE_0,
        .x_prefetch_en                  = QSPI_CONCURRENT_XIP_PREFETCH_DISABLE,
        .x_continous_xfer_en            = QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE,
        .x_instruction_en               = QSPI_CONCURRENT_XIP_INST_ENABLE,
        .x_instruction_size             = QSPI_CONCURRENT_XIP_INSTSIZE_8BIT,
        .x_address_size                 = QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT,
        .x_inst_addr_transfer_format    = QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF,
        .x_mode_bits_en                 = QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE,
        .x_mode_bits_length             = QSPI_CONCURRENT_XIP_MBL_8,
        .x_mode_bits_data               = 0x00,
        .x_dummy_cycles                 = 4,
        .x_continous_xfer_toc           = 0,
        .x_sioo_mode                    = QSPI_CONCURRENT_XIP_INST_SENT_EVERY_ACCESS,
        .x_data_frame_format            = QSPI_CONCURRENT_XIP_FRF_QUAD_SPI,
        .x_instruction                  = 0x0B,
    },
    [PSRAM_MMAP_CMD_QREAD_EBH] = {
        .x_endian_mode                  = QSPI_CONCURRENT_XIP_ENDIAN_MODE_0,
        .x_prefetch_en                  = QSPI_CONCURRENT_XIP_PREFETCH_DISABLE,
        .x_continous_xfer_en            = QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE,
        .x_instruction_en               = QSPI_CONCURRENT_XIP_INST_ENABLE,
        .x_instruction_size             = QSPI_CONCURRENT_XIP_INSTSIZE_8BIT,
        .x_address_size                 = QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT,
        .x_inst_addr_transfer_format    = QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF,
        .x_mode_bits_en                 = QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE,
        .x_mode_bits_length             = QSPI_CONCURRENT_XIP_MBL_8,
        .x_mode_bits_data               = 0x00,
        .x_dummy_cycles                 = 6,
        .x_continous_xfer_toc           = 0,
        .x_sioo_mode                    = QSPI_CONCURRENT_XIP_INST_SENT_EVERY_ACCESS,
        .x_data_frame_format            = QSPI_CONCURRENT_XIP_FRF_QUAD_SPI,
        .x_instruction                  = 0xEB,
    },
};

const qspi_memorymapped_write_t g_psram_typical_mmap_qwrite_cmd[PSRAM_MMAP_CMD_WRITE_MAX] =
{
    /**************************************************************
     * IF DEFINES ARE NOT MATCHING WITH THE USED PSRAM,
     * PLEASE MODIFY CAREFULLY
     **************************************************************/
    [PSRAM_MMAP_CMD_QWRITE_02H] = {
        .x_instruction                  = 0x02,
        .x_instruction_size             = QSPI_CONCURRENT_XIP_INSTSIZE_8BIT,
        .x_address_size                 = QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT,
        .x_inst_addr_transfer_format    = QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF,
        .x_dummy_cycles                 = 0,
        .x_data_frame_format            = QSPI_CONCURRENT_XIP_FRF_QUAD_SPI,
    },
    [PSRAM_MMAP_CMD_QWRITE_38H] = {
        .x_instruction                  = 0x38,
        .x_instruction_size             = QSPI_CONCURRENT_XIP_INSTSIZE_8BIT,
        .x_address_size                 = QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT,
        .x_inst_addr_transfer_format    = QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF,
        .x_dummy_cycles                 = 0,
        .x_data_frame_format            = QSPI_CONCURRENT_XIP_FRF_QUAD_SPI,
    }
};

#define APP_QSPI_PIN_CONFIG(pin_type, pin_mux, pin_number, pin_mode, pin_pull, pin_enable)  \
            {                                                                               \
                .type	= pin_type,                                                         \
                .mux	= pin_mux,                                                          \
                .pin	= pin_number,                                                       \
                .mode	= pin_mode,                                                         \
                .pull	= pin_pull,                                                         \
                .enable = pin_enable,                                                       \
            }
#define APP_QSPI_IO_PULL_DEFAULT            APP_IO_NOPULL
const app_qspi_pin_cfg_t g_qspi_pin_groups[QSPIx_PIN_GROUP_MAX] = {
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
    /* DON'T MODIFY THE FOLLOWING CONFIGS FOR GR5526 */
    [QSPI0_PIN_GROUP_0] = {
        .cs   = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_10, APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .clk  = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_5,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_0 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_6,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_1 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_7,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_2 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_8,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_3 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_9,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
    },
    [QSPI1_PIN_GROUP_0] = {
        .cs   = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_0, APP_IO_PIN_10,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .clk  = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_0, APP_IO_PIN_15,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_0 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_0, APP_IO_PIN_14,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_1 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_0, APP_IO_PIN_13,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_2 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_0, APP_IO_PIN_12,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_3 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_0, APP_IO_PIN_11,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
    },
    [QSPI2_PIN_GROUP_0] = {
        .cs   = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_11,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .clk  = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_0,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_0 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_1,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_1 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_2,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_2 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_3,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_3 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_4,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
    }
#else
    /* CAN MODIFY THE FOLLOWING CONFIGS FOR GR5525 */
    [QSPI0_PIN_GROUP_0] = {
        .cs   = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_0, APP_IO_PIN_15,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .clk  = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_2,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_0 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_0, APP_IO_PIN_3,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_1 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_0, APP_IO_PIN_14,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_2 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_0, APP_IO_PIN_13,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_3 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_0, APP_IO_PIN_12,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
    },
    [QSPI1_PIN_GROUP_0] = {
        .cs   = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_7, APP_IO_PIN_5,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .clk  = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_7, APP_IO_PIN_6,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_0 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_7, APP_IO_PIN_4,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_1 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOB, APP_IO_MUX_7, APP_IO_PIN_7,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_2 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOC, APP_IO_MUX_7, APP_IO_PIN_0,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_3 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOC, APP_IO_MUX_7, APP_IO_PIN_1,   APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
    },
    [QSPI2_PIN_GROUP_0] = {
        .cs   = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_7, APP_IO_PIN_6,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .clk  = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_7, APP_IO_PIN_3,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_0 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_7, APP_IO_PIN_4,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_1 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_7, APP_IO_PIN_5,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_2 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_7, APP_IO_PIN_2,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
        .io_3 = APP_QSPI_PIN_CONFIG(APP_IO_TYPE_GPIOA, APP_IO_MUX_7, APP_IO_PIN_7,  APP_IO_MODE_MUX, APP_QSPI_IO_PULL_DEFAULT, APP_QSPI_PIN_ENABLE),
    }
#endif
};
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
static const IRQn_Type              s_qspi_irq[APP_QSPI_ID_MAX]         = { QSPI0_IRQn, QSPI1_IRQn, QSPI2_IRQn };
const uint32_t               s_qspi_instance[APP_QSPI_ID_MAX]    = { QSPI0_BASE, QSPI1_BASE, QSPI2_BASE };
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
static const IRQn_Type   s_qspi_irq[APP_QSPI_ID_MAX] = { QSPI0_IRQn, QSPI1_IRQn };
static const uint32_t    s_qspi_instance[APP_QSPI_ID_MAX] = { QSPI0_BASE, QSPI1_BASE };
#endif

qspi_env_t *p_qspi_env[APP_QSPI_ID_MAX];

static const app_sleep_callbacks_t qspi_sleep_cb =
{
    .app_prepare_for_sleep = qspi_prepare_for_sleep,
    .app_wake_up_ind       = qspi_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
bool qspi_prepare_for_sleep(void)
{
    hal_qspi_state_t state;
    uint32_t i;

    for (i = 0; i < APP_QSPI_ID_MAX; i++)
    {
        if (p_qspi_env[i] == NULL)
        {
            continue;
        }

        if (p_qspi_env[i]->qspi_state == APP_QSPI_ACTIVITY)
        {
            state = hal_qspi_get_state(&p_qspi_env[i]->handle);
            if ((state != HAL_QSPI_STATE_RESET) && (state != HAL_QSPI_STATE_READY))
            {
                return false;
            }

            GLOBAL_EXCEPTION_DISABLE();
            hal_qspi_suspend_reg(&p_qspi_env[i]->handle);
            GLOBAL_EXCEPTION_ENABLE();
            #ifdef APP_DRIVER_WAKEUP_CALL_FUN
            p_qspi_env[i]->qspi_state = APP_QSPI_SLEEP;
            #endif
        }
    }

    return true;
}

SECTION_RAM_CODE void qspi_wake_up_ind(void)
{
#ifndef APP_DRIVER_WAKEUP_CALL_FUN
    uint32_t i;

    for (i = 0; i < APP_QSPI_ID_MAX; i++)
    {
        if (p_qspi_env[i] == NULL)
        {
            continue;
        }

        if (p_qspi_env[i]->qspi_state == APP_QSPI_ACTIVITY)
        {
            GLOBAL_EXCEPTION_DISABLE();
            hal_qspi_resume_reg(&p_qspi_env[i]->handle);
            GLOBAL_EXCEPTION_ENABLE();

            hal_nvic_clear_pending_irq(s_qspi_irq[i]);
            hal_nvic_enable_irq(s_qspi_irq[i]);
        }
    }
#endif
}

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
void qspi_wake_up(app_qspi_id_t id)
{
    if (p_qspi_env[id]->qspi_state == APP_QSPI_SLEEP)
    {
        GLOBAL_EXCEPTION_DISABLE();
        hal_qspi_resume_reg(&p_qspi_env[id]->handle);
        GLOBAL_EXCEPTION_ENABLE();

        hal_nvic_clear_pending_irq(s_qspi_irq[id]);
        hal_nvic_enable_irq(s_qspi_irq[id]);
        p_qspi_env[id]->qspi_state = APP_QSPI_ACTIVITY;
    }

    if(p_qspi_env[id]->use_mode.type == APP_QSPI_TYPE_DMA)
    {
        dma_wake_up(p_qspi_env[id]->dma_id);
    }
}
#endif


#define QSPI_HANDLER(index, val) \
SECTION_RAM_CODE void QSPI##index##_IRQHandler(void)\
{\
    hal_qspi_irq_handler(&p_qspi_env[val]->handle);\
}

QSPI_HANDLER(0, APP_QSPI_ID_0)
QSPI_HANDLER(1, APP_QSPI_ID_1)
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
QSPI_HANDLER(2, APP_QSPI_ID_2)
#endif
/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_qspi_init(app_qspi_params_t *p_params, app_qspi_evt_handler_t evt_handler)
{
    app_qspi_id_t id = p_params->id;
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }
    p_qspi_env[id] = &p_params->qspi_env;

    app_err_code = qspi_gpio_config(&p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    p_qspi_env[id]->p_pin_cfg = &p_params->pin_cfg;
    p_qspi_env[id]->evt_handler = evt_handler;

    memcpy(&p_qspi_env[id]->handle.init, &p_params->init, sizeof(qspi_init_t));
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    p_qspi_env[id]->handle.p_instance = (ssi_regs_t *)s_qspi_instance[id];
#else
    p_qspi_env[id]->handle.p_instance = (qspi_regs_t *)s_qspi_instance[id];
#endif
    hal_err_code = hal_qspi_deinit(&p_qspi_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_qspi_init(&p_qspi_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    p_qspi_env[id]->is_xfer_err      = false;
    p_qspi_env[id]->is_tx_done       = false;
    p_qspi_env[id]->is_rx_done       = false;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    p_qspi_env[id]->is_mmap_inited   = false;
    p_qspi_env[id]->is_mmap_prefetch_en = false;
#endif

    pwr_register_sleep_cb(&qspi_sleep_cb, APP_DRIVER_QSPI_WAKEUP_PRIORITY, QSPI_PWR_ID);

    p_qspi_env[id]->qspi_state = APP_QSPI_ACTIVITY;
    p_qspi_env[id]->start_flag = false;

    soc_register_nvic(QSPI0_IRQn, (uint32_t)QSPI0_IRQHandler);
    soc_register_nvic(QSPI1_IRQn, (uint32_t)QSPI1_IRQHandler);
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    soc_register_nvic(QSPI2_IRQn, (uint32_t)QSPI2_IRQHandler);
#endif
    hal_nvic_clear_pending_irq(s_qspi_irq[id]);
    hal_nvic_enable_irq(s_qspi_irq[id]);

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_deinit(app_qspi_id_t id)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_qspi_env[id]->p_pin_cfg->cs.enable == APP_QSPI_PIN_ENABLE)
    {
        app_err_code = app_io_deinit(p_qspi_env[id]->p_pin_cfg->cs.type, p_qspi_env[id]->p_pin_cfg->cs.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }
    if (p_qspi_env[id]->p_pin_cfg->clk.enable == APP_QSPI_PIN_ENABLE)
    {
        app_err_code = app_io_deinit(p_qspi_env[id]->p_pin_cfg->clk.type, p_qspi_env[id]->p_pin_cfg->clk.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }
    if (p_qspi_env[id]->p_pin_cfg->io_0.enable == APP_QSPI_PIN_ENABLE)
    {
        app_err_code = app_io_deinit(p_qspi_env[id]->p_pin_cfg->io_0.type, p_qspi_env[id]->p_pin_cfg->io_0.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }
    if (p_qspi_env[id]->p_pin_cfg->io_1.enable == APP_QSPI_PIN_ENABLE)
    {
        app_err_code = app_io_deinit(p_qspi_env[id]->p_pin_cfg->io_1.type, p_qspi_env[id]->p_pin_cfg->io_1.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }
    if (p_qspi_env[id]->p_pin_cfg->io_2.enable == APP_QSPI_PIN_ENABLE)
    {
        app_err_code = app_io_deinit(p_qspi_env[id]->p_pin_cfg->io_2.type, p_qspi_env[id]->p_pin_cfg->io_2.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }
    if (p_qspi_env[id]->p_pin_cfg->io_3.enable == APP_QSPI_PIN_ENABLE)
    {
        app_err_code = app_io_deinit(p_qspi_env[id]->p_pin_cfg->io_3.type, p_qspi_env[id]->p_pin_cfg->io_3.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }

    hal_nvic_disable_irq(s_qspi_irq[id]);
    p_qspi_env[id]->qspi_state = APP_QSPI_INVALID;
    p_qspi_env[id]->start_flag = false;
    p_qspi_env[id]->is_xfer_err      = false;
    p_qspi_env[id]->is_tx_done       = false;
    p_qspi_env[id]->is_rx_done       = false;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    p_qspi_env[id]->is_mmap_inited   = false;
    p_qspi_env[id]->is_mmap_prefetch_en = false;
#endif
    GLOBAL_EXCEPTION_DISABLE();
    for (uint32_t i = 0; i < APP_QSPI_ID_MAX; i++)
    {
        if ((p_qspi_env[i]) && ((p_qspi_env[i]->qspi_state) != APP_QSPI_INVALID))
        {
            goto __deinit;
        }
    }
    pwr_unregister_sleep_cb(QSPI_PWR_ID);
__deinit:
    GLOBAL_EXCEPTION_ENABLE();

    hal_err_code = hal_qspi_deinit(&p_qspi_env[id]->handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    if (p_qspi_env[id]->qspi_dma_state == APP_QSPI_DMA_INVALID)
    {
        p_qspi_env[id] = NULL;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_abort(app_qspi_id_t id)
{
    uint16_t err_code = HAL_ERROR;

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (!p_qspi_env[id]->start_flag)
    {
        return APP_DRV_SUCCESS;
    }

    err_code = hal_qspi_abort_it(&p_qspi_env[id]->handle);
    if (err_code != HAL_OK)
    {
        return err_code;
    }

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
bool app_qspi_active_memory_mappped(app_qspi_id_t id, bool is_active) {
    hal_status_t status = HAL_OK;
    APP_ASSERT_CHECK(p_qspi_env[id]->is_mmap_inited);

    if(p_qspi_env[id]->start_flag) {
        if(APP_QSPI_EXCEPT_DEBUG_EN) printf("+++ ERR : CAN NOT change(%d), Busy... \r\n", id);
        return false;
    }
    if(is_active) {
        if((p_qspi_env[id]->mounted_mmap_device.dev_type == APP_QSPI_DEVICE_FLASH) &&
           (g_flash_typical_mmap_read_cmd[p_qspi_env[id]->mounted_mmap_device.rd.flash_rd].x_sioo_mode == QSPI_CONCURRENT_XIP_INST_SENT_ONLY_FIRST_ACCESS)) {
                status = hal_qspi_memorymapped_active(&p_qspi_env[id]->handle, true);
           } else {
                status = hal_qspi_memorymapped_active(&p_qspi_env[id]->handle, false);
           }
    } else {
        status = hal_qspi_memorymapped_deactive(&p_qspi_env[id]->handle);
    }

    return (status == HAL_OK) ? true : false;
}

bool app_qspi_config_memory_mappped(app_qspi_id_t id, app_qspi_mmap_device_t dev) {
    bool  rRet = false;
    hal_status_t status ;
    const qspi_memorymapped_t * mmap_rd_cmd = NULL;
    const qspi_memorymapped_write_t * mmap_wr_cmd = NULL;

    if ((id >= APP_QSPI_ID_MAX) || (p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return false;
    }

    switch(dev.dev_type) {
        case APP_QSPI_DEVICE_FLASH:
        {
            if(dev.rd.flash_rd < FLASH_MMAP_CMD_READ_MAX) {
                mmap_rd_cmd = &g_flash_typical_mmap_read_cmd[dev.rd.flash_rd];
            }
        }
        break;

        case APP_QSPI_DEVICE_PSRAM:
        {
            if(dev.rd.psram_rd < PSRAM_MMAP_CMD_READ_MAX) {
                mmap_rd_cmd = &g_psram_typical_mmap_qread_cmd[dev.rd.psram_rd];
            }

            if(dev.psram_wr < PSRAM_MMAP_CMD_WRITE_MAX) {
                mmap_wr_cmd = &g_psram_typical_mmap_qwrite_cmd[dev.psram_wr];
            }
        }
        break;

        case APP_QSPI_DEVICE_UNSET:
        default:
        {}
        break;
    }

    if((mmap_rd_cmd != NULL) || (mmap_wr_cmd != NULL)) {
        status = hal_qspi_memorymapped(&p_qspi_env[id]->handle, (qspi_memorymapped_t *)mmap_rd_cmd, (qspi_memorymapped_write_t *)mmap_wr_cmd);
        if(HAL_OK == status) {
            if(dev.dev_type == APP_QSPI_DEVICE_PSRAM) {
                ll_qspi_enable_xip_dynamic_le(p_qspi_env[id]->handle.p_instance);    /* active dynamicle mode for psram */
            }
            rRet = true;
        }
    }

    if(rRet) {
        memcpy(&p_qspi_env[id]->mounted_mmap_device, &dev, sizeof(app_qspi_mmap_device_t));
        p_qspi_env[id]->is_mmap_inited   = true;
        p_qspi_env[id]->mmap_endian_mode = (app_qspi_mmap_endian_mode_e) mmap_rd_cmd->x_endian_mode;
    }

    return rRet;
}
#endif

uint16_t app_qspi_command_receive_sync(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL || p_data == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    QSPI_SMART_CS_LOW(id);
    err_code = hal_qspi_command_receive(&p_qspi_env[id]->handle, p_cmd, p_data, timeout);
    QSPI_SMART_CS_HIGH(id);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_command_receive_async(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL || p_data == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    if (p_qspi_env[id]->start_flag == false)
    {
        p_qspi_env[id]->start_flag = true;
        QSPI_SMART_CS_LOW(id);
        err_code = hal_qspi_command_receive_it(&p_qspi_env[id]->handle, p_cmd, p_data);
        if (HAL_OK != err_code)
        {
            QSPI_SMART_CS_HIGH(id);
            p_qspi_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_command_transmit_sync(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL || p_data == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    QSPI_SMART_CS_LOW(id);
    err_code = hal_qspi_command_transmit(&p_qspi_env[id]->handle, p_cmd, p_data, timeout);
    QSPI_SMART_CS_HIGH(id);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_command_transmit_async(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL || p_data == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    if (p_qspi_env[id]->start_flag == false)
    {
        p_qspi_env[id]->start_flag = true;
        QSPI_SMART_CS_LOW(id);
        err_code = hal_qspi_command_transmit_it(&p_qspi_env[id]->handle, p_cmd, p_data);
        if (err_code != HAL_OK)
        {
            QSPI_SMART_CS_HIGH(id);
            p_qspi_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}


uint16_t app_qspi_command_sync(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    QSPI_SMART_CS_LOW(id);
    err_code = hal_qspi_command(&p_qspi_env[id]->handle, p_cmd, timeout);
    QSPI_SMART_CS_HIGH(id);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_command_async(app_qspi_id_t id, app_qspi_command_t *p_cmd)
{
    hal_status_t err_code = HAL_ERROR;

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_cmd == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }


#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    if (p_qspi_env[id]->start_flag == false)
    {
        p_qspi_env[id]->start_flag = true;
        QSPI_SMART_CS_LOW(id);
        err_code = hal_qspi_command_it(&p_qspi_env[id]->handle, p_cmd);
        if (err_code != HAL_OK)
        {
            QSPI_SMART_CS_HIGH(id);
            p_qspi_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
uint16_t app_qspi_transmit_sync(app_qspi_id_t id, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    return app_qspi_transmit_sync_ex(id, 0, 0, p_data, length, timeout);
}
#endif

uint16_t app_qspi_transmit_sync_ex(app_qspi_id_t id, uint32_t qspi_mode, uint32_t data_width, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    UNUSED(qspi_mode);//mode and data_width were fixed when init
    UNUSED(data_width);
#endif
    hal_status_t err_code;
    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    QSPI_SMART_CS_LOW(id);
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    err_code = hal_qspi_transmit(&p_qspi_env[id]->handle, p_data, length, timeout);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    err_code = hal_qspi_transmit(&p_qspi_env[id]->handle, qspi_mode, data_width, p_data, length, timeout);
#endif

    QSPI_SMART_CS_HIGH(id);
    if (err_code != HAL_OK)
    {
        return err_code;
    }

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
uint16_t app_qspi_transmit_async(app_qspi_id_t id, uint8_t *p_data, uint32_t length)
{
    return app_qspi_transmit_async_ex(id, 0, 0, p_data, length);
}
#endif

uint16_t app_qspi_transmit_async_ex(app_qspi_id_t id, uint32_t qspi_mode, uint32_t data_width, uint8_t *p_data, uint32_t length)
{
    hal_status_t err_code = HAL_ERROR;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    UNUSED(qspi_mode);//mode and data_width were fixed when init
    UNUSED(data_width);
#endif

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    if (p_qspi_env[id]->start_flag == false)
    {
        p_qspi_env[id]->start_flag = true;
        QSPI_SMART_CS_LOW(id);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        err_code = hal_qspi_transmit_it(&p_qspi_env[id]->handle, p_data, length);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        err_code = hal_qspi_transmit_it(&p_qspi_env[id]->handle, qspi_mode, data_width, p_data, length);
#endif
        if (err_code != HAL_OK)
        {
            QSPI_SMART_CS_HIGH(id);
            p_qspi_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
uint16_t app_qspi_receive_sync(app_qspi_id_t id, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    return app_qspi_receive_sync_ex( id, 0, 0, p_data, length, timeout);
}
#endif

uint16_t app_qspi_receive_sync_ex(app_qspi_id_t id, uint32_t qspi_mode, uint32_t data_width, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t err_code;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    UNUSED(qspi_mode);
    UNUSED(data_width);
#endif

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    QSPI_SMART_CS_LOW(id);
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    err_code = hal_qspi_receive(&p_qspi_env[id]->handle, p_data, length, timeout);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    err_code = hal_qspi_receive(&p_qspi_env[id]->handle, qspi_mode, data_width, p_data, length, timeout);
#endif
    QSPI_SMART_CS_HIGH(id);
    if (err_code != HAL_OK)
    {
        return (uint16_t)err_code;
    }

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
uint16_t app_qspi_receive_async(app_qspi_id_t id, uint8_t *p_data, uint32_t length)
{
    return app_qspi_receive_async_ex(id, 0, 0, p_data, length);
}
#endif

uint16_t app_qspi_receive_async_ex(app_qspi_id_t id, uint32_t qspi_mode, uint32_t data_width, uint8_t *p_data, uint32_t length)
{
    hal_status_t err_code = HAL_ERROR;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    UNUSED(qspi_mode);
    UNUSED(data_width);
#endif

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_data == NULL || length == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    if (p_qspi_env[id]->start_flag == false)
    {
        p_qspi_env[id]->start_flag = true;
        QSPI_SMART_CS_LOW(id);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        err_code = hal_qspi_receive_it(&p_qspi_env[id]->handle, p_data, length);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        err_code = hal_qspi_receive_it(&p_qspi_env[id]->handle, qspi_mode, data_width, p_data, length);
#endif
        if (err_code != HAL_OK)
        {
            QSPI_SMART_CS_HIGH(id);
            p_qspi_env[id]->start_flag = false;
            return (uint16_t)err_code;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
bool app_qspi_mmap_set_endian_mode(app_qspi_id_t id, app_qspi_mmap_endian_mode_e mode) {
    hal_status_t status = HAL_ERROR;

    APP_ASSERT_CHECK(p_qspi_env[id]->is_mmap_inited);

    if(p_qspi_env[id]->mmap_endian_mode == mode) {
        return true;
    }

    qspi_memorymapped_set_t mmap_set = {
        .mmap_key  = QSPI_MMAPED_IDX_EDIAN_MODE,
        .mmap_val  = (mode == APP_QSPI_MMAP_ENDIAN_MODE_0) ? QSPI_CONCURRENT_XIP_ENDIAN_MODE_0 : ((mode == APP_QSPI_MMAP_ENDIAN_MODE_1) ? QSPI_CONCURRENT_XIP_ENDIAN_MODE_1 : QSPI_CONCURRENT_XIP_ENDIAN_MODE_2),
    };

    status = hal_qspi_memorymapped_update(&p_qspi_env[id]->handle, &mmap_set, 1);

    //override the xip mode
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
    if(id == APP_QSPI_ID_0) {
        REG(0xA000E180) = REG(0xA000E180) & 0xFFFFFFFE;
    } else if (id == APP_QSPI_ID_1) {
        REG(0xA000E180) = REG(0xA000E180) & 0xFFFFFFEF;
    } else if (id == APP_QSPI_ID_2) {
        REG(0xA000E180) = REG(0xA000E180) & 0xFFFFFEFF;
    }
#elif (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    if(id == APP_QSPI_ID_0) {
        REG(0x4000E180) = REG(0x4000E180) & 0xFFFFFFFE;
    } else if (id == APP_QSPI_ID_1) {
        REG(0x4000E180) = REG(0x4000E180) & 0xFFFFFFEF;
    } else if (id == APP_QSPI_ID_2) {
        REG(0x4000E180) = REG(0x4000E180) & 0xFFFFFEFF;
    }
#endif

    if(HAL_OK == status) {
        p_qspi_env[id]->mmap_endian_mode = mode;
        return true;
    }

    return false;
}

bool app_qspi_mmap_set_prefetch(app_qspi_id_t id, bool prefetch_en) {
    hal_status_t status = HAL_ERROR;

    APP_ASSERT_CHECK(p_qspi_env[id]->is_mmap_inited);

    if(p_qspi_env[id]->is_mmap_prefetch_en == prefetch_en) {
        return true;
    }

    qspi_memorymapped_set_t mmap_set = {
        .mmap_key  = QSPI_MMAPED_IDX_PREFETCH_EN,
        .mmap_val  = prefetch_en ? QSPI_CONCURRENT_XIP_PREFETCH_ENABLE : QSPI_CONCURRENT_XIP_PREFETCH_DISABLE,
    };

    status = hal_qspi_memorymapped_update(&p_qspi_env[id]->handle, &mmap_set, 1);

    if(HAL_OK == status) {
        p_qspi_env[id]->is_mmap_prefetch_en = prefetch_en;
        return true;
    }

    return false;
}

uint8_t app_qspi_mmap_read_u8(app_qspi_id_t id, uint32_t address) {
    if (id >= APP_QSPI_ID_MAX)
    {
        return 0;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return 0;
    }
    //APP_ASSERT_CHECK(p_qspi_env[id]->is_mmap_inited);
    app_qspi_mmap_set_endian_mode(id, APP_QSPI_MMAP_ENDIAN_MODE_0);
    return *((volatile uint8_t *)(ll_qspi_get_xip_base_address((qspi_regs_t*)s_qspi_instance[id]) + address));
}

uint16_t app_qspi_mmap_read_u16(app_qspi_id_t id, uint32_t address) {
    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }
    //APP_ASSERT_CHECK(p_qspi_env[id]->is_mmap_inited);
    APP_ASSERT_CHECK(!(address & 0x01));    /* U16 aligned */
    app_qspi_mmap_set_endian_mode(id, APP_QSPI_MMAP_ENDIAN_MODE_1);
    return *((volatile uint16_t *)(ll_qspi_get_xip_base_address((qspi_regs_t*)s_qspi_instance[id]) + address));
}

uint32_t app_qspi_mmap_read_u32(app_qspi_id_t id, uint32_t address) {
    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }
    //APP_ASSERT_CHECK(p_qspi_env[id]->is_mmap_inited);
    APP_ASSERT_CHECK(!(address & 0x03));    /* U32 aligned */
    app_qspi_mmap_set_endian_mode(id, APP_QSPI_MMAP_ENDIAN_MODE_2);
    return *((volatile uint32_t *)(ll_qspi_get_xip_base_address((qspi_regs_t*)s_qspi_instance[id]) + address));
}

bool app_qspi_mmap_read_block(app_qspi_id_t id, uint32_t address, uint8_t * buffer, uint32_t length) {
    bool ret = true;

    if ((id >= APP_QSPI_ID_MAX) || (p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return false;
    }
    //APP_ASSERT_CHECK(p_qspi_env[id]->is_mmap_inited);

    app_qspi_mmap_set_endian_mode(id, APP_QSPI_MMAP_ENDIAN_MODE_2);
    app_qspi_mmap_set_prefetch(id, false);
    memcpy(buffer, (void *)(ll_qspi_get_xip_base_address((qspi_regs_t*)s_qspi_instance[id]) + address), length);
    return ret;
}

uint32_t app_qspi_get_xip_base_address(app_qspi_id_t id) {
    return ll_qspi_get_xip_base_address((qspi_regs_t*)s_qspi_instance[id]);
}

#endif
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint16_t qspi_gpio_config(app_qspi_pin_cfg_t *p_pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if (p_pin_cfg->cs.enable == APP_QSPI_PIN_ENABLE)
    {
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        io_init.pull = p_pin_cfg->cs.pull;
        io_init.pin  = p_pin_cfg->cs.pin;
        io_init.mux  = p_pin_cfg->cs.mux;
        io_init.mode = p_pin_cfg->cs.mode;
        err_code = app_io_init(p_pin_cfg->cs.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        io_init.pull = p_pin_cfg->cs.pull;
        io_init.mode = APP_IO_MODE_OUTPUT;
        io_init.pin  = p_pin_cfg->cs.pin;
        io_init.mux  = APP_IO_MUX_7;
        err_code = app_io_init(p_pin_cfg->cs.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
        app_io_write_pin(p_pin_cfg->cs.type, p_pin_cfg->cs.pin, APP_IO_PIN_SET);
#endif
    }
    if (p_pin_cfg->clk.enable == APP_QSPI_PIN_ENABLE)
    {
        io_init.pull = p_pin_cfg->clk.pull;
        io_init.pin  = p_pin_cfg->clk.pin;
        io_init.mux  = p_pin_cfg->clk.mux;
        io_init.mode = p_pin_cfg->clk.mode;
        err_code = app_io_init(p_pin_cfg->clk.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }
    if (p_pin_cfg->io_0.enable == APP_QSPI_PIN_ENABLE)
    {
        io_init.pull = p_pin_cfg->io_0.pull;
        io_init.pin  = p_pin_cfg->io_0.pin;
        io_init.mux  = p_pin_cfg->io_0.mux;
        io_init.mode = p_pin_cfg->io_0.mode;
        err_code = app_io_init(p_pin_cfg->io_0.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }
    if (p_pin_cfg->io_1.enable == APP_QSPI_PIN_ENABLE)
    {
        io_init.pull = p_pin_cfg->io_1.pull;
        io_init.pin  = p_pin_cfg->io_1.pin;
        io_init.mux  = p_pin_cfg->io_1.mux;
        io_init.mode = p_pin_cfg->io_1.mode;
        err_code = app_io_init(p_pin_cfg->io_1.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }
    if (p_pin_cfg->io_2.enable == APP_QSPI_PIN_ENABLE)
    {
        io_init.pull = p_pin_cfg->io_2.pull;
        io_init.pin  = p_pin_cfg->io_2.pin;
        io_init.mux  = p_pin_cfg->io_2.mux;
        io_init.mode = p_pin_cfg->io_2.mode;
        err_code = app_io_init(p_pin_cfg->io_2.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }
    if (p_pin_cfg->io_3.enable == APP_QSPI_PIN_ENABLE)
    {
        io_init.pull = p_pin_cfg->io_3.pull;
        io_init.pin  = p_pin_cfg->io_3.pin;
        io_init.mux  = p_pin_cfg->io_3.mux;
        io_init.mode = p_pin_cfg->io_3.mode;
        err_code = app_io_init(p_pin_cfg->io_3.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    return err_code;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
void app_qspi_force_cs(app_qspi_id_t screen_id, bool low_level) {

    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;

    io_init.pull = p_qspi_env[screen_id]->p_pin_cfg->cs.pull;
    io_init.pin  = p_qspi_env[screen_id]->p_pin_cfg->cs.pin;
    io_init.mux  = APP_IO_MUX;
    io_init.mode = APP_IO_MODE_OUTPUT;
    app_io_init(p_qspi_env[screen_id]->p_pin_cfg->cs.type, &io_init);
    if(low_level) {
        app_io_write_pin(p_qspi_env[screen_id]->p_pin_cfg->cs.type, io_init.pin, APP_IO_PIN_SET);
        delay_us(1);
        app_io_write_pin(p_qspi_env[screen_id]->p_pin_cfg->cs.type, io_init.pin, APP_IO_PIN_RESET);
    } else {
        /* first : release cs */
        app_io_write_pin(p_qspi_env[screen_id]->p_pin_cfg->cs.type, io_init.pin, APP_IO_PIN_SET);

        /* then : restore hard-cs */
        io_init.mux  = p_qspi_env[screen_id]->p_pin_cfg->cs.mux;
        io_init.mode = p_qspi_env[screen_id]->p_pin_cfg->cs.mode;
        app_io_init(p_qspi_env[screen_id]->p_pin_cfg->cs.type, &io_init);
    }

    return;
}
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
#if (QSPI_DMA_LLP_FEATUTE_SUPPORT > 0u)
__weak bool app_graphics_qspi_draw_screen_continue(app_qspi_id_t id, app_qspi_evt_t qspi_evt)
{
    UNUSED(id);
    UNUSED(qspi_evt);
    return false;
}
#endif

__weak void _free_qspi_dma_llp_resource(void) {
    //override this in app_graphics_qspi.c
}
#endif

static void app_qspi_event_call(qspi_handle_t *p_qspi, app_qspi_evt_type_t evt_type)
{
    app_qspi_evt_t qspi_evt;
    app_qspi_id_t id = APP_QSPI_ID_MAX;

    if (p_qspi->p_instance == QSPI0)
    {
        id = APP_QSPI_ID_0;
    }
    else if (p_qspi->p_instance == QSPI1)
    {
        id = APP_QSPI_ID_1;
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    else if (p_qspi->p_instance == QSPI2)
    {
        id = APP_QSPI_ID_2;
    }
#endif
    qspi_evt.type = evt_type;
    if (evt_type == APP_QSPI_EVT_ERROR)
    {
        qspi_evt.data.error_code = p_qspi->error_code;
        p_qspi_env[id]->is_xfer_err = 1;
        p_qspi_env[id]->is_rx_done  = 1;
        p_qspi_env[id]->is_tx_done  = 1;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        if (is_dma_access)
        {
            ((qspi_regs_t *)s_qspi_instance[id])->DMAC = 0;
            ll_dma_disable_channel(p_qspi_env[id]->dma_cfg.dma_instance, p_qspi_env[id]->dma_cfg.dma_channel);
        }
#endif
        if(APP_QSPI_EXCEPT_DEBUG_EN) printf("+++ ERR: QSPI%d xfer Err ...\r\n", id);
    }
    else if (evt_type == APP_QSPI_EVT_TX_CPLT)
    {
        uint32_t tx_xfer_size_cb = p_qspi->tx_xfer_size;
        uint32_t tx_xfer_count_cb = p_qspi->tx_xfer_count;
        qspi_evt.data.size = tx_xfer_size_cb - tx_xfer_count_cb;
        p_qspi_env[id]->is_xfer_err = 0;
        p_qspi_env[id]->is_tx_done  = 1;

#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X))
        _free_qspi_dma_llp_resource();
#endif
    }
    else if (evt_type == APP_QSPI_EVT_RX_DATA)
    {
        uint32_t rx_xfer_size_cb = p_qspi->rx_xfer_size;
        uint32_t rx_xfer_count_cb = p_qspi->rx_xfer_count;
        qspi_evt.data.size = rx_xfer_size_cb - rx_xfer_count_cb;
        p_qspi_env[id]->is_xfer_err = 0;
        p_qspi_env[id]->is_rx_done  = 1;
    }
    else if (evt_type == APP_QSPI_EVT_ABORT)
    {
        p_qspi_env[id]->is_xfer_err = 1;
        p_qspi_env[id]->is_rx_done  = 1;
        p_qspi_env[id]->is_tx_done  = 1;
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
#if (QSPI_DMA_LLP_FEATUTE_SUPPORT > 0u)
    bool ret = app_graphics_qspi_draw_screen_continue(id, qspi_evt);
    if(ret)return;
#endif
#endif

    p_qspi_env[id]->start_flag = false;
    QSPI_SMART_CS_HIGH(id);
    if (p_qspi_env[id]->evt_handler != NULL)
    {
        p_qspi_env[id]->evt_handler(&qspi_evt);
    }
}

qspi_handle_t *app_qspi_get_handle(app_qspi_id_t id)
{
    if (id >= APP_QSPI_ID_MAX)
    {
        return NULL;
    }

    if ((p_qspi_env[id] == NULL) || (p_qspi_env[id]->qspi_state == APP_QSPI_INVALID))
    {
        return NULL;
    }

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    qspi_wake_up(id);
#endif

    return &p_qspi_env[id]->handle;
}

void hal_qspi_error_callback(qspi_handle_t *p_qspi)
{
    app_qspi_event_call(p_qspi, APP_QSPI_EVT_ERROR);
}

void hal_qspi_rx_cplt_callback(qspi_handle_t *p_qspi)
{
    app_qspi_event_call(p_qspi, APP_QSPI_EVT_RX_DATA);
}

void hal_qspi_tx_cplt_callback(qspi_handle_t *p_qspi)
{
    app_qspi_event_call(p_qspi, APP_QSPI_EVT_TX_CPLT);
}

void hal_qspi_abort_cplt_callback(qspi_handle_t *p_qspi)
{
    app_qspi_event_call(p_qspi, APP_QSPI_EVT_ABORT);
}

#endif


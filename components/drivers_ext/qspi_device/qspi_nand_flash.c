/**
 *****************************************************************************************
 *
 * @file qspi_nand_flash.c
 *
 * @brief Function Implementation.
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
#include <stdlib.h>
#include <string.h>
#include "app_log.h"
#include "qspi_nand_flash.h"


#define DEFAULT_MODE_CONFIG                 {DMA0, DMA_Channel0, 1000, 0}
#define DEFAULT_QSPI_CONFIG                 {48, QSPI_CLOCK_MODE_0, 0}
#define DEFAULT_PARAM_CONFIG                {APP_QSPI_ID_0, g_qspi_pin_groups[QSPI0_PIN_GROUP_0], DEFAULT_MODE_CONFIG, DEFAULT_QSPI_CONFIG}

#define NAND_FLASH_QSPI_PORT                APP_IO_TYPE_GPIOA

/*
 * static declaration
 *****************************************************************************************
 */
static void     qspi_nand_flash_reset(app_qspi_id_t qspi_id);
static void     qspi_nand_flash_write_enable(app_qspi_id_t qspi_id);
//static void     qspi_nand_flash_write_disable(app_qspi_id_t qspi_id);
static uint8_t  qspi_nand_flash_get_status(app_qspi_id_t qspi_id, uint32_t address);
static void     qspi_nand_flash_set_status(app_qspi_id_t qspi_id, uint32_t address, uint8_t status);
static bool     is_qspi_nand_flash_busy(app_qspi_id_t qspi_id);
//static bool     is_qspi_nand_flash_writable(app_qspi_id_t qspi_id);
static bool     is_qspi_nand_flash_erase_ok(app_qspi_id_t qspi_id);
static bool     is_qspi_nand_flash_program_ok(app_qspi_id_t qspi_id);
static void     qspi_nand_flash_wait_busy(app_qspi_id_t qspi_id);
static uint8_t  qspi_nand_flash_get_block_lock_status(app_qspi_id_t qspi_id);

static uint32_t             s_nand_flash_type = 0;
static app_qspi_params_t    g_qspi_params;

static volatile uint8_t g_master_tdone = 0;
static volatile uint8_t g_master_rdone = 0;

static void app_qspi_callback(app_qspi_evt_t *p_evt)
{
    if (p_evt->type == APP_QSPI_EVT_TX_CPLT)
    {
        g_master_tdone = 1;
    }
    if (p_evt->type == APP_QSPI_EVT_RX_DATA)
    {
        g_master_rdone = 1;
    }
    if ((p_evt->type == APP_QSPI_EVT_ERROR) || (p_evt->type == APP_QSPI_EVT_ABORT))
    {
        g_master_tdone = 1;
        g_master_rdone = 1;
    }
}

/*
 * public methods
 *****************************************************************************************
 */
uint32_t qspi_nand_flash_init(app_qspi_id_t id, uint32_t clock_prescaler, qspi_pins_group_e pin_group)
{
    uint16_t ret;
    app_qspi_params_t p_params = DEFAULT_PARAM_CONFIG;
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;

    g_qspi_params = p_params;

    g_qspi_params.id                    = id;
    g_qspi_params.pin_cfg               = g_qspi_pin_groups[pin_group];
    g_qspi_params.init.clock_prescaler  = clock_prescaler;
    if(clock_prescaler == 2){
        g_qspi_params.init.rx_sample_delay = 1;
    }  else {
        g_qspi_params.init.rx_sample_delay = 0;
    }

    ret = app_qspi_init(&g_qspi_params, app_qspi_callback);
    if (ret != 0)
    {
        APP_LOG_ERROR("QSPI initial failed! Please check the input paraments.");
        return 1;
    }

    ret = app_qspi_dma_init(&g_qspi_params);
    if (ret != 0)
    {
        APP_LOG_ERROR("QSPI initial dma failed! Please check the input paraments.");
        return 1;
    }

    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin  = g_qspi_params.pin_cfg.io_2.pin;
    io_init.mux  = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(NAND_FLASH_QSPI_PORT, &io_init);

    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin  = g_qspi_params.pin_cfg.io_3.pin;
    io_init.mux  = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(NAND_FLASH_QSPI_PORT, &io_init);

    app_io_write_pin(NAND_FLASH_QSPI_PORT, g_qspi_params.pin_cfg.io_2.pin, APP_IO_PIN_SET);
    app_io_write_pin(NAND_FLASH_QSPI_PORT, g_qspi_params.pin_cfg.io_3.pin, APP_IO_PIN_SET);

    /* Reset flash */
    qspi_nand_flash_reset(id);

    s_nand_flash_type = qspi_nand_flash_read_id(id);

    qspi_nand_flash_unlock_block(id);

    return s_nand_flash_type;
}

void qspi_nand_flash_deinit(void) {
    app_qspi_deinit(g_qspi_params.id);
    memset(&g_qspi_params, 0, sizeof(app_qspi_params_t));
}

void qspi_nand_flash_unlock_block(app_qspi_id_t qspi_id) {
    uint8_t status = qspi_nand_flash_get_block_lock_status(qspi_id);

    status = status & (~0x3F);
    qspi_nand_flash_set_status(qspi_id, 0xA0, status);

}

void qspi_nand_flash_enable_quad(app_qspi_id_t qspi_id)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    uint8_t otp_status = qspi_nand_flash_get_status(qspi_id, 0xB0);

    if(!(otp_status & 0x01)) {
        otp_status |= 0x01;

        qspi_nand_flash_set_status(qspi_id, 0xB0, otp_status);
    }

    io_init.mode = g_qspi_params.pin_cfg.io_2.mode;
    io_init.pin  = g_qspi_params.pin_cfg.io_2.pin;
    io_init.mux  = g_qspi_params.pin_cfg.io_2.mux;
    io_init.pull = g_qspi_params.pin_cfg.io_2.pull;
    app_io_init(g_qspi_params.pin_cfg.io_2.type, &io_init);

    io_init.mode = g_qspi_params.pin_cfg.io_3.mode;
    io_init.pin  = g_qspi_params.pin_cfg.io_3.pin;
    io_init.mux  = g_qspi_params.pin_cfg.io_3.mux;
    io_init.pull = g_qspi_params.pin_cfg.io_3.pull;
    app_io_init(g_qspi_params.pin_cfg.io_3.type, &io_init);
}

void qspi_nand_flash_disable_quad(app_qspi_id_t qspi_id)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    uint8_t otp_status = qspi_nand_flash_get_status(qspi_id, 0xB0);

    if(otp_status & 0x01) {
        otp_status &= ~0x01;

        qspi_nand_flash_set_status(qspi_id, 0xB0, otp_status);
    }

    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin = g_qspi_params.pin_cfg.io_2.pin;
    io_init.mux = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(g_qspi_params.pin_cfg.io_2.type, &io_init);

    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin = g_qspi_params.pin_cfg.io_3.pin;
    io_init.mux = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(g_qspi_params.pin_cfg.io_3.type, &io_init);

    app_io_write_pin(g_qspi_params.pin_cfg.io_2.type, g_qspi_params.pin_cfg.io_2.pin, APP_IO_PIN_SET);
    app_io_write_pin(g_qspi_params.pin_cfg.io_3.type, g_qspi_params.pin_cfg.io_3.pin, APP_IO_PIN_SET);
}


bool qspi_nand_flash_erase_block(app_qspi_id_t qspi_id, uint32_t block_address) {
    const uint32_t blk_addr = (block_address & 0x03FF) << 6 ;
    uint8_t control_frame[4];

    control_frame[0] = NAND_FLASH_CMD_BLK_ERASE;
    control_frame[1] = (blk_addr >> 16) & 0xFF;
    control_frame[2] = (blk_addr >> 8 ) & 0xFF;
    control_frame[3] = (blk_addr >> 0 ) & 0xFF;


    //printf("block lock : %d \r\n", qspi_nand_flash_get_block_lock_status(qspi_id));

    qspi_nand_flash_write_enable(qspi_id);
#if 0
    if(!is_qspi_nand_flash_writable(qspi_id)) {
        printf("not writable ...\r\n");
        return false;
    }
    //printf("writable ...\r\n");
#endif

    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);

    qspi_nand_flash_wait_busy(qspi_id);

    return is_qspi_nand_flash_erase_ok(qspi_id);
}

uint32_t qspi_nand_flash_read_id(app_qspi_id_t qspi_id)
{
    uint8_t data[2];
    app_qspi_command_t command = {
        .instruction      = NAND_FLASH_CMD_RDID,
        .address          = 0x00,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_08_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 2,
        .clock_stretch_en = 1,
    };

    g_master_rdone = 0;
    app_qspi_dma_command_receive_async(qspi_id, &command, data);
    while(g_master_rdone == 0);

    return (((uint32_t)data[0] << 8) + data[1]);
}


bool qspi_nand_flash_std_read(app_qspi_id_t qspi_id, uint32_t block, uint32_t page, uint32_t page_offset, uint8_t * data, uint32_t length) {
    const uint32_t blk_page_addr = (block << 6) | page ;
    const uint32_t page_addr     = (page_offset << 8) & 0x0FFF00;
    uint8_t control_frame[4];
    uint32_t ret = 0;

    if((block >= NAND_MAX_BLOCKS) || (page >= NAND_MAX_PAGES_PER_BLOCK)) {
        return false;
    }

    if((length == 0) || (page_offset + length > NAND_MAX_BYTES_PER_PAGE)) {
        return false;
    }

    control_frame[0] = NAND_FLASH_CMD_PREAD_2_CACHE;
    control_frame[1] = (blk_page_addr >> 16) & 0xFF;
    control_frame[2] = (blk_page_addr >> 8 ) & 0xFF;
    control_frame[3] = (blk_page_addr >> 0 ) & 0xFF;

    g_master_tdone = 0;
    ret = app_qspi_dma_transmit_async_ex(qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);

    if(ret != APP_DRV_SUCCESS) {
        return false;
    }

    qspi_nand_flash_wait_busy(qspi_id);

    app_qspi_command_t command = {
        .instruction      = NAND_FLASH_CMD_READ,
        .address          = page_addr,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = length,
        .clock_stretch_en = 1,
    };

    g_master_rdone = 0;
    ret = app_qspi_dma_command_receive_async(qspi_id, &command, data);
    while(g_master_rdone == 0);

    if(ret != APP_DRV_SUCCESS) {
        return false;
    }

    return true;
}

bool qspi_nand_flash_dual_read(app_qspi_id_t qspi_id, uint32_t block, uint32_t page, uint32_t page_offset, uint8_t * data, uint32_t length) {
    const uint32_t blk_page_addr = (block << 6) | page ;
    const uint32_t page_addr     = (page_offset << 8) & 0x0FFF00;
    uint8_t control_frame[4];
    uint32_t ret = 0;

    if((block >= NAND_MAX_BLOCKS) || (page >= NAND_MAX_PAGES_PER_BLOCK)) {
        return false;
    }

    if((length == 0) || (page_offset + length > NAND_MAX_BYTES_PER_PAGE)) {
        return false;
    }

    control_frame[0] = NAND_FLASH_CMD_PREAD_2_CACHE;
    control_frame[1] = (blk_page_addr >> 16) & 0xFF;
    control_frame[2] = (blk_page_addr >> 8 ) & 0xFF;
    control_frame[3] = (blk_page_addr >> 0 ) & 0xFF;

    g_master_tdone = 0;
    ret = app_qspi_dma_transmit_async_ex(qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);

    if(ret != APP_DRV_SUCCESS) {
        return false;
    }

    qspi_nand_flash_wait_busy(qspi_id);

    app_qspi_command_t command = {
        .instruction      = NAND_FLASH_CMD_DREAD,
        .address          = page_addr,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_DUALSPI,
        .length           = length,
        .clock_stretch_en = 1,
    };

    g_master_rdone = 0;
    ret = app_qspi_dma_command_receive_async(qspi_id, &command, data);
    while(g_master_rdone == 0);

    if(ret != APP_DRV_SUCCESS) {
        return false;
    }

    return true;
}

bool qspi_nand_flash_quad_read(app_qspi_id_t qspi_id, uint32_t block, uint32_t page, uint32_t page_offset, uint8_t * data, uint32_t length) {
    const uint32_t blk_page_addr = (block << 6) | page ;
    const uint32_t page_addr     = (page_offset << 8) & 0x0FFF00;
    uint8_t control_frame[4];
    uint32_t ret = 0;

    if((block >= NAND_MAX_BLOCKS) || (page >= NAND_MAX_PAGES_PER_BLOCK)) {
        return false;
    }

    if((length == 0) || (page_offset + length > NAND_MAX_BYTES_PER_PAGE)) {
        return false;
    }

    control_frame[0] = NAND_FLASH_CMD_PREAD_2_CACHE;
    control_frame[1] = (blk_page_addr >> 16) & 0xFF;
    control_frame[2] = (blk_page_addr >> 8 ) & 0xFF;
    control_frame[3] = (blk_page_addr >> 0 ) & 0xFF;

    g_master_tdone = 0;
    ret = app_qspi_dma_transmit_async_ex(qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);

    if(ret != APP_DRV_SUCCESS) {
        return false;
    }

    qspi_nand_flash_wait_busy(qspi_id);

    app_qspi_command_t command = {
        .instruction      = NAND_FLASH_CMD_QREAD,
        .address          = page_addr,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_32_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = length,
        .clock_stretch_en = 1,
    };

    g_master_rdone = 0;
    ret = app_qspi_dma_command_receive_async(qspi_id, &command, data);
    while(g_master_rdone == 0);

    if(ret != APP_DRV_SUCCESS) {
        return false;
    }

    return true;
}


bool qspi_nand_flash_std_write(app_qspi_id_t qspi_id, uint32_t block, uint32_t page, uint32_t page_offset, uint8_t * data, uint32_t length) {

    const uint32_t blk_page_addr = (block << 6) | page ;
    const uint32_t page_addr     = (page_offset << 0) & 0x0FFF;
    uint8_t control_frame[4];
    uint32_t ret = 0;

    if((block >= NAND_MAX_BLOCKS) || (page >= NAND_MAX_PAGES_PER_BLOCK)) {
        return false;
    }

    if((length == 0) || (page_offset + length > NAND_MAX_BYTES_PER_PAGE)) {
        return false;
    }

    qspi_nand_flash_write_enable(qspi_id);


    app_qspi_command_t command = {
        .instruction      = NAND_FLASH_CMD_PROG_LOAD,
        .address          = page_addr,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_16_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = length,
        .clock_stretch_en = 1,
    };

    g_master_tdone = 0;
    ret = app_qspi_dma_command_transmit_async(qspi_id, &command, data);
    while(g_master_tdone == 0);

    if(ret != APP_DRV_SUCCESS) {
        return false;
    }


    control_frame[0] = NAND_FLASH_CMD_PROG_EXE;
    control_frame[1] = (blk_page_addr >> 16) & 0xFF;
    control_frame[2] = (blk_page_addr >> 8 ) & 0xFF;
    control_frame[3] = (blk_page_addr >> 0 ) & 0xFF;

    g_master_tdone = 0;
    ret = app_qspi_dma_transmit_async_ex(qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);

    if(ret != APP_DRV_SUCCESS) {
        return false;
    }


    qspi_nand_flash_wait_busy(qspi_id);
#if 1
    return is_qspi_nand_flash_program_ok(qspi_id);
#else
    return true;
#endif
}


bool qspi_nand_flash_quad_write(app_qspi_id_t qspi_id, uint32_t block, uint32_t page, uint32_t page_offset, uint8_t * data, uint32_t length) {

    const uint32_t blk_page_addr = (block << 6) | page ;
    const uint32_t page_addr     = (page_offset << 0) & 0x0FFF;
    uint8_t control_frame[4];
    uint32_t ret = 0;

    if((block >= NAND_MAX_BLOCKS) || (page >= NAND_MAX_PAGES_PER_BLOCK)) {
        return false;
    }

    if((length == 0) || (page_offset + length > NAND_MAX_BYTES_PER_PAGE)) {
        return false;
    }

    qspi_nand_flash_write_enable(qspi_id);

    app_qspi_command_t command = {
        .instruction      = NAND_FLASH_CMD_PROG_QLOAD,
        .address          = page_addr,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_16_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_32_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = length,
        .clock_stretch_en = 1,
    };

    g_master_tdone = 0;
    ret = app_qspi_dma_command_transmit_async(qspi_id, &command, data);
    while(g_master_tdone == 0);

    if(ret != APP_DRV_SUCCESS) {
        return false;
    }

    control_frame[0] = NAND_FLASH_CMD_PROG_EXE;
    control_frame[1] = (blk_page_addr >> 16) & 0xFF;
    control_frame[2] = (blk_page_addr >> 8 ) & 0xFF;
    control_frame[3] = (blk_page_addr >> 0 ) & 0xFF;

    g_master_tdone = 0;
    ret = app_qspi_dma_transmit_async_ex(qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);

    if(ret != APP_DRV_SUCCESS) {
        return false;
    }

    qspi_nand_flash_wait_busy(qspi_id);

#if 1
    return is_qspi_nand_flash_program_ok(qspi_id);
#else
    return true;
#endif
}

/*
 * LOCAL Methods
 *****************************************************************************************
 */

static void qspi_nand_flash_reset(app_qspi_id_t qspi_id) {
    uint8_t control_frame[1] = {NAND_FLASH_CMD_RESET};

    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);
}

static void qspi_nand_flash_write_enable(app_qspi_id_t qspi_id)
{
    uint8_t control_frame[1] = {NAND_FLASH_CMD_WREN};

    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);
}

#if 0
static void qspi_nand_flash_write_disable(app_qspi_id_t qspi_id)
{
    uint8_t control_frame[1] = {NAND_FLASH_CMD_WRDI};

    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));

    while(g_master_tdone == 0);
}
#endif

static uint8_t qspi_nand_flash_get_status(app_qspi_id_t qspi_id, uint32_t address) {
    uint8_t status = 0;
    app_qspi_command_t command = {
        .instruction      = NAND_FLASH_CMD_GET_FEAT,
        .address          = address,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_08_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 1,
        .clock_stretch_en = 1,
    };

    g_master_rdone = 0;
    app_qspi_dma_command_receive_async(qspi_id, &command, &status);
    while(g_master_rdone == 0);

    return status;
}

static void qspi_nand_flash_set_status(app_qspi_id_t qspi_id, uint32_t address, uint8_t status) {

    app_qspi_command_t command = {
        .instruction      = NAND_FLASH_CMD_SET_FEAT,
        .address          = address,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_08_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 1,
        .clock_stretch_en = 1,
    };

    g_master_tdone = 0;
    app_qspi_dma_command_transmit_async(qspi_id, &command, &status);
    while(g_master_tdone == 0);

    return ;
}

static bool is_qspi_nand_flash_busy(app_qspi_id_t qspi_id) {
    return (qspi_nand_flash_get_status(qspi_id, 0xC0) & 0x01) ? true : false;
}

#if 0
static bool is_qspi_nand_flash_writable(app_qspi_id_t qspi_id) {
    return (qspi_nand_flash_get_status(qspi_id, 0xC0) & 0x02) ? true : false;
}
#endif

static bool is_qspi_nand_flash_erase_ok(app_qspi_id_t qspi_id) {
    return (qspi_nand_flash_get_status(qspi_id, 0xC0) & 0x04) ? false : true;
}

static bool is_qspi_nand_flash_program_ok(app_qspi_id_t qspi_id) {
    return (qspi_nand_flash_get_status(qspi_id, 0xC0) & 0x08) ? false : true;
}

static void qspi_nand_flash_wait_busy(app_qspi_id_t qspi_id) {
    while(is_qspi_nand_flash_busy(qspi_id));
}

static uint8_t qspi_nand_flash_get_block_lock_status(app_qspi_id_t qspi_id) {
    return qspi_nand_flash_get_status(qspi_id, 0xA0);
}



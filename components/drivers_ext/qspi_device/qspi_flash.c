/**
 *****************************************************************************************
 *
 * @file qspi_flash.c
 *
 * @brief Function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2021 GOODIX
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
#include "qspi_flash.h"
#include "app_qspi_dma.h"


#define DEFAULT_MODE_CONFIG                 {DMA0, DMA_Channel0, 1000, 0}
#define DEFAULT_QSPI_CONFIG                 {2, QSPI_CLOCK_MODE_3, 0}
#define DEFAULT_PARAM_CONFIG                {APP_QSPI_ID_0, g_qspi_pin_groups[QSPI0_PIN_GROUP_0], DEFAULT_MODE_CONFIG, DEFAULT_QSPI_CONFIG}

uint8_t spi_flash_type = 0;

static  app_qspi_params_t g_qspi_params;

static uint32_t spi_flash_read_status(void);
static uint32_t spi_flash_read_config(void);
static void spi_flash_write_status(uint32_t status);

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

uint8_t SPI_FLASH_init(app_qspi_id_t id, uint32_t clock_prescaler, qspi_pins_group_e pin_group)
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
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5525X)
    io_init.mux  = APP_IO_MUX_7;
#else
    io_init.mux  = APP_IO_MUX_8;
#endif
    io_init.pull = APP_IO_PULLUP;
    app_io_init(g_qspi_params.pin_cfg.io_2.type, &io_init);

    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin  = g_qspi_params.pin_cfg.io_3.pin;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5525X)
    io_init.mux  = APP_IO_MUX_7;
#else
    io_init.mux  = APP_IO_MUX_8;
#endif
    io_init.pull = APP_IO_PULLUP;
    app_io_init(g_qspi_params.pin_cfg.io_3.type, &io_init);

    app_io_write_pin(g_qspi_params.pin_cfg.io_2.type, g_qspi_params.pin_cfg.io_2.pin, APP_IO_PIN_SET);
    app_io_write_pin(g_qspi_params.pin_cfg.io_3.type, g_qspi_params.pin_cfg.io_3.pin, APP_IO_PIN_SET);

    /* Reset flash */
    SPI_FLASH_Reset();
    /* Wakeup from deep power down */
    SPI_FLASH_Wakeup();

    uint8_t count = 20;
    do {
        spi_flash_type = (SPI_FLASH_Read_Device_ID() >> 16) & 0xFF;
        if((spi_flash_type!=0x00) && (spi_flash_type!= 0xFF)) {
            break;
        }
    } while(count--);

    SPI_FLASH_Unprotect();

    return spi_flash_type;
}

void SPI_FLASH_deinit(void) 
{
    uint16_t ret;
    ret = app_qspi_dma_deinit(g_qspi_params.id);
    if (ret != 0)
    {
        printf("QSPI deinit dma failed!\r\n");
        return;
    }
    app_qspi_deinit(g_qspi_params.id);
    memset(&g_qspi_params, 0, sizeof(app_qspi_params_t));
}

void SPI_FLASH_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_READ,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = nbytes,
        .clock_stretch_en = 1,
    };

    g_master_rdone = 0;
    app_qspi_dma_command_receive_async(g_qspi_params.id, &command, buffer);
    while(g_master_rdone == 0);
}

void SPI_FLASH_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_FREAD,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 8,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = nbytes,
        .clock_stretch_en = 1,
    };

    g_master_rdone = 0;
    app_qspi_dma_command_receive_async(g_qspi_params.id, &command, buffer);
    while(g_master_rdone == 0);
}

void SPI_FLASH_Dual_Output_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_DOFR,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 8,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_DUALSPI,
        .length           = nbytes,
        .clock_stretch_en = 1,
    };

    g_master_rdone = 0;
    app_qspi_dma_command_receive_async(g_qspi_params.id, &command, buffer);
    while(g_master_rdone == 0);
}

void SPI_FLASH_Dual_IO_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_DIOFR,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 4,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_IN_SPI_ADDR_IN_SPIFRF,
        .data_mode        = QSPI_DATA_MODE_DUALSPI,
        .length           = nbytes,
        .clock_stretch_en = 1,
    };

    g_master_rdone = 0;
    app_qspi_dma_command_receive_async(g_qspi_params.id, &command, buffer);
    while(g_master_rdone == 0);
}

void SPI_FLASH_Quad_Output_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_QOFR,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 8,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = nbytes,
        .clock_stretch_en = 1,
    };

    g_master_rdone = 0;
    app_qspi_dma_command_receive_async(g_qspi_params.id, &command, buffer);
    while(g_master_rdone == 0);
}

void SPI_FLASH_Quad_IO_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_QIOFR,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 6,
        .data_size        = QSPI_DATASIZE_32_BITS,
        .instruction_address_mode = QSPI_INST_IN_SPI_ADDR_IN_SPIFRF,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = nbytes,
        .clock_stretch_en = 1,
    };

    g_master_rdone = 0;
    app_qspi_dma_command_receive_async(g_qspi_params.id, &command, buffer);
    while(g_master_rdone == 0);
}

uint32_t SPI_FLASH_Read_Device_ID(void)
{
    uint8_t data[3];
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_RDID,
        .address          = 0,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_00_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 3,
        .clock_stretch_en = 1,
    };

    g_master_rdone = 0;
    app_qspi_dma_command_receive_async(g_qspi_params.id, &command, data);
    while(g_master_rdone == 0);

    return (((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + data[2]);
}

static void SPI_FLASH_WREN(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_WREN};

    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(g_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);
}

static void SPI_FLASH_Wait_Busy(void)
{
    while (spi_flash_read_status() & 1);
}

void SPI_FLASH_Enable_Quad(void)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    uint32_t reg_status = spi_flash_read_status();
    uint32_t reg_config;

    switch (spi_flash_type)
    {
    case SPI_FLASH_TYE_GD25:
    case SPI_FLASH_TYE_PY25:
    case SPI_FLASH_TYE_XTX:
        if (!(reg_status & 0x0200))
        {
            reg_status |= 0x0200;
            spi_flash_write_status(reg_status);
        }
        break;
    case SPI_FLASH_TYE_MX25:
        if (!(reg_status & 0x40))
        {
            reg_config = spi_flash_read_config();
            reg_status = ((reg_status | 0x40) & 0xFF) | ((reg_config << 8) & 0xFFFF00);
            spi_flash_write_status(reg_status);
        }
        break;
    case SPI_FLASH_TYE_SST26:
        reg_config = spi_flash_read_config();
        if (!(reg_config & 0x02))
        {
            reg_status = (reg_status & 0xFF) | (((reg_config | 0x02) << 8) & 0xFF00);
            spi_flash_write_status(reg_status);
        }
        break;
    }

    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin = g_qspi_params.pin_cfg.io_2.pin;
    io_init.mux = APP_IO_MUX_0;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(g_qspi_params.pin_cfg.io_2.type, &io_init);

    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin = g_qspi_params.pin_cfg.io_3.pin;
    io_init.mux = APP_IO_MUX_0;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(g_qspi_params.pin_cfg.io_3.type, &io_init);
}

void SPI_FLASH_Disable_Quad(void)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    uint32_t reg_status = spi_flash_read_status();
    uint32_t reg_config;

    switch (spi_flash_type)
    {
    case SPI_FLASH_TYE_GD25:
    case SPI_FLASH_TYE_PY25:
    case SPI_FLASH_TYE_XTX:
        if (reg_status & 0x0200)
        {
            reg_status &= ~0x0200;
            spi_flash_write_status(reg_status);
        }
        break;
    case SPI_FLASH_TYE_MX25:
        if (reg_status & 0x40)
        {
            reg_config = spi_flash_read_config();
            reg_status = ((reg_status & ~0x40) & 0xFF) | ((reg_config << 8) & 0xFFFF00);
            spi_flash_write_status(reg_status);
        }
        break;
    case SPI_FLASH_TYE_SST26:
        reg_config = spi_flash_read_config();
        if (reg_config & 0x02)
        {
            reg_status = (reg_status & 0xFF) | (((reg_config & ~0x02) << 8) & 0xFF00);
            spi_flash_write_status(reg_status);
        }
        break;
    }

    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin = g_qspi_params.pin_cfg.io_2.pin;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5525X)
    io_init.mux = APP_IO_MUX_7;
#else
    io_init.mux  = APP_IO_MUX_8;
#endif
    io_init.pull = APP_IO_PULLUP;
    app_io_init(g_qspi_params.pin_cfg.io_2.type, &io_init);

    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin = g_qspi_params.pin_cfg.io_3.pin;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5525X)
    io_init.mux = APP_IO_MUX_7;
#else
    io_init.mux  = APP_IO_MUX_8;
#endif
    io_init.pull = APP_IO_PULLUP;
    app_io_init(g_qspi_params.pin_cfg.io_3.type, &io_init);

    app_io_write_pin(g_qspi_params.pin_cfg.io_2.type, g_qspi_params.pin_cfg.io_2.pin, APP_IO_PIN_SET);
    app_io_write_pin(g_qspi_params.pin_cfg.io_3.type, g_qspi_params.pin_cfg.io_3.pin, APP_IO_PIN_SET);
}

void SPI_FLASH_Unprotect(void)
{
    uint32_t reg_status = spi_flash_read_status();
    uint32_t reg_config;

    switch (spi_flash_type)
    {
    case SPI_FLASH_TYE_GD25:
        reg_status &= ~0x41FC;
        spi_flash_write_status(reg_status);
        break;
    case SPI_FLASH_TYE_PY25:
        reg_status &= ~0x41FC;
        spi_flash_write_status(reg_status);
        break;
    case SPI_FLASH_TYE_MX25:
        reg_config = spi_flash_read_config();
        reg_status = ((reg_status & ~0xFC) & 0xFF) | ((reg_config << 8) & 0xFFFF00);
        spi_flash_write_status(reg_status);
        break;
    }
}

void SPI_FLASH_Sector_Erase(uint32_t Dst)
{
    uint8_t control_frame[4];
    control_frame[0] = SPI_FLASH_CMD_SE;
    control_frame[1] = (Dst >> 16) & 0xFF;
    control_frame[2] = (Dst >> 8) & 0xFF;
    control_frame[3] = Dst & 0xFF;

    SPI_FLASH_WREN();

    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(g_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Block_Erase_32K(uint32_t Dst)
{
    uint8_t control_frame[4];
    control_frame[0] = SPI_FLASH_CMD_BE_32;
    control_frame[1] = (Dst >> 16) & 0xFF;
    control_frame[2] = (Dst >> 8) & 0xFF;
    control_frame[3] = Dst & 0xFF;

    SPI_FLASH_WREN();

    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(g_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Block_Erase_64K(uint32_t Dst)
{
    uint8_t control_frame[4];
    control_frame[0] = SPI_FLASH_CMD_BE_64;
    control_frame[1] = (Dst >> 16) & 0xFF;
    control_frame[2] = (Dst >> 8) & 0xFF;
    control_frame[3] = Dst & 0xFF;

    SPI_FLASH_WREN();

    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(g_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Chip_Erase(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_CE};

    SPI_FLASH_WREN();

    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(g_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Reset(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_RSTEN};
    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(g_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);

    control_frame[0] = SPI_FLASH_CMD_RST;
    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(g_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);
}

void SPI_FLASH_PowerDown(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_DP};

    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(g_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);
}

void SPI_FLASH_Wakeup(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_RDP};
    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(g_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    while(g_master_tdone == 0);
}

void SPI_FLASH_Page_Program(uint32_t Dst, uint8_t *data)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_PP,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_32_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 256,
        .clock_stretch_en = 1,
    };

    SPI_FLASH_WREN();

    g_master_tdone = 0;
    app_qspi_dma_command_transmit_async(g_qspi_params.id, &command, data);
    while(g_master_tdone == 0);

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Page_Program_With_Data_Size(uint32_t Dst, uint8_t *data, qspi_flash_data_size_t data_size)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_PP,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = data_size,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 256,
        .clock_stretch_en = 1,
    };

    SPI_FLASH_WREN();

    g_master_tdone = 0;
    app_qspi_dma_command_transmit_async(g_qspi_params.id, &command, data);
    while(g_master_tdone == 0);

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Dual_Page_Program(uint32_t Dst, uint8_t *data)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_DPP,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_32_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_DUALSPI,
        .length           = 256,
        .clock_stretch_en = 1,
    };

    SPI_FLASH_WREN();

    g_master_tdone = 0;
    app_qspi_dma_command_transmit_async(g_qspi_params.id, &command, data);
    while(g_master_tdone == 0);

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Dual_Page_Program_With_Data_Size(uint32_t Dst, uint8_t *data, qspi_flash_data_size_t data_size)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_DPP,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = data_size,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_DUALSPI,
        .length           = 256,
        .clock_stretch_en = 1,
    };

    SPI_FLASH_WREN();

    g_master_tdone = 0;
    app_qspi_dma_command_transmit_async(g_qspi_params.id, &command, data);
    while(g_master_tdone == 0);

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Quad_Page_Program(uint32_t Dst, uint8_t *data)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_QPP,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_32_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = 256,
        .clock_stretch_en = 1,
    };

    SPI_FLASH_WREN();

    g_master_tdone = 0;
    app_qspi_dma_command_transmit_async(g_qspi_params.id, &command, data);
    while(g_master_tdone == 0);

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Quad_Page_Program_With_Data_Size(uint32_t Dst, uint8_t *data, qspi_flash_data_size_t data_size)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_QPP,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = data_size,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = 256,
        .clock_stretch_en = 1,
    };

    SPI_FLASH_WREN();

    g_master_tdone = 0;
    app_qspi_dma_command_transmit_async(g_qspi_params.id, &command, data);
    while(g_master_tdone == 0);

    SPI_FLASH_Wait_Busy();
}

static uint32_t spi_flash_read_status(void)
{
    uint32_t ret = 0;
    uint8_t *pret = (uint8_t*)&ret;
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_RDSR,
        .address          = 0,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_00_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 1,
        .clock_stretch_en = 1,
    };

    switch (spi_flash_type)
    {
        case SPI_FLASH_TYE_GD25:
        case SPI_FLASH_TYE_PY25:
            g_master_rdone = 0;
            app_qspi_dma_command_receive_async(g_qspi_params.id, &command, pret++);
            while(g_master_rdone == 0);
            command.instruction = 0x35;
            break;
        case SPI_FLASH_TYE_MX25:
            break;
        case SPI_FLASH_TYE_SST26:
            break;
    }

    g_master_rdone = 0;
    app_qspi_dma_command_receive_async(g_qspi_params.id, &command, pret);
    while(g_master_rdone == 0);

    return ret;
}

static uint32_t spi_flash_read_config(void)
{
    uint32_t ret = 0;
    app_qspi_command_t command = {
        .instruction      = 0,
        .address          = 0,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_00_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 1,
        .clock_stretch_en = 1,
    };

    switch (spi_flash_type)
    {
        case SPI_FLASH_TYE_MX25:
            command.instruction = 0x15;
            command.length      = 2;
            break;
        case SPI_FLASH_TYE_SST26:
            command.instruction = 0x35;
            command.length      = 1;
            break;
    }
    g_master_tdone = 0;
    app_qspi_dma_command_transmit_async(g_qspi_params.id, &command, (uint8_t*)&ret);
    while(g_master_tdone == 0);

    return ret;
}

static void spi_flash_write_status(uint32_t status)
{
    uint8_t control_frame[4], length = 3;
    control_frame[0] = SPI_FLASH_CMD_WRSR;
    control_frame[1] = status & 0xFF;
    control_frame[2] = (status >> 8) & 0xFF;
    control_frame[3] = (status >> 16) & 0xFF;

    SPI_FLASH_WREN();

    if (SPI_FLASH_TYE_MX25 == spi_flash_type)
        length = 4;
    else if (SPI_FLASH_TYE_PY25 == spi_flash_type)
    {
#if SPI_FLASH_PY25_WRSR_2BYTE
        length = 3;
#else /* the WRSR command of some Puya FLASH chips is followed by 1 byte parameter. */
        /* use two commands to write status register for Puya Flash. */
        length = 2;

        g_master_tdone = 0;
        app_qspi_dma_transmit_async_ex(g_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, length);
        while(g_master_tdone == 0);

        SPI_FLASH_Wait_Busy();

        SPI_FLASH_WREN();

        control_frame[0] = SPI_FLASH_CMD_WRSR1;
        control_frame[1] = (status >> 8) & 0xFF;
#endif /* SPI_FLASH_PY25_RSDR_2BYTE */
    }

    g_master_tdone = 0;
    app_qspi_dma_transmit_async_ex(g_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, length);
    while(g_master_tdone == 0);

    SPI_FLASH_Wait_Busy();
}

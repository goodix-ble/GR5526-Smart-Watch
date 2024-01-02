/**
 *****************************************************************************************
 *
 * @file qspi_norflash_v2.c
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
#include "qspi_norflash_v2.h"



/********************************************************************************************
 *                       Defines
 ********************************************************************************************/

#define DEFAULT_MODE_CONFIG                 {DMA0, DMA_Channel0, 1000, 0}
#define DEFAULT_QSPI_CONFIG                 {2, QSPI_CLOCK_MODE_3, 0}
#define DEFAULT_PARAM_CONFIG                {APP_QSPI_ID_0, g_qspi_pin_groups[QSPI0_PIN_GROUP_0], DEFAULT_MODE_CONFIG, DEFAULT_QSPI_CONFIG}


/********************************************************************************************
 *                       Static Declarations
 ********************************************************************************************/
static uint32_t                     _qspi_norf_read_status_reg(void);
static uint32_t                     _qspi_norf_read_config_reg(void);
static void                         _qspi_norf_write_status_reg(uint32_t status);
static void                         _qspi_norf_callback_handler(app_qspi_evt_t *p_evt);
static bool                         _qspi_norf_wren(void);
static void                         _qspi_norf_wait_busy(void);
static void                         _qspi_norf_io_reinit(void);
static void                         _qspi_norf_io_deinit(void);

static          app_qspi_params_t   s_qspi_params;
static          uint8_t             s_norflash_type = 0;
static volatile uint8_t             s_qspi_tx_done  = 0;
static volatile uint8_t             s_qspi_rx_done  = 0;


/********************************************************************************************
 *                       Public Functions
 ********************************************************************************************/

uint8_t qspi_norf_init(app_qspi_id_t id, uint32_t clock_prescaler, app_qspi_pin_cfg_t * pin_cfg) {
    uint16_t ret;
    app_qspi_params_t p_params = DEFAULT_PARAM_CONFIG;
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;

    s_qspi_params = p_params;

    s_qspi_params.id                    = id;
    s_qspi_params.init.clock_prescaler  = clock_prescaler;
    if(clock_prescaler == 2) {
        s_qspi_params.init.rx_sample_delay = 1;
    }  else {
        s_qspi_params.init.rx_sample_delay = 0;
    }
    memcpy(&s_qspi_params.pin_cfg, pin_cfg, sizeof(app_qspi_pin_cfg_t));

    ret = app_qspi_init(&s_qspi_params, _qspi_norf_callback_handler);
    if (ret != 0)
    {
        APP_LOG_ERROR("QSPI initial failed! Please check the input paraments.");
        return 1;
    }

    ret = app_qspi_dma_init(&s_qspi_params);
    if (ret != 0)
    {
        APP_LOG_ERROR("QSPI initial dma failed! Please check the input paraments.");
        return 1;
    }

    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin  = s_qspi_params.pin_cfg.io_2.pin;
    io_init.mux  = APP_IO_MUX;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(s_qspi_params.pin_cfg.io_2.type, &io_init);

    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin  = s_qspi_params.pin_cfg.io_3.pin;
    io_init.mux  = APP_IO_MUX;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(s_qspi_params.pin_cfg.io_3.type, &io_init);

    app_io_write_pin(s_qspi_params.pin_cfg.io_2.type, s_qspi_params.pin_cfg.io_2.pin, APP_IO_PIN_SET);
    app_io_write_pin(s_qspi_params.pin_cfg.io_3.type, s_qspi_params.pin_cfg.io_3.pin, APP_IO_PIN_SET);

    /* Reset flash */
    qspi_norf_reset();
    /* Wakeup from deep power down */
    qspi_norf_wakeup();

    uint8_t count = 20;
    do {
        s_norflash_type = (qspi_norf_read_dev_id() >> 16) & 0xFF;
        if((s_norflash_type!=0x00) && (s_norflash_type!= 0xFF)) {
            break;
        }
    } while(count--);

    qspi_norf_unprotect();

    return s_norflash_type;
}


bool qspi_norf_deinit(void)
{
    uint16_t ret;
    ret = app_qspi_dma_deinit(s_qspi_params.id);
    if (ret != 0)
    {
        printf("QSPI deinit dma failed!\r\n");
        return false;
    }
    app_qspi_deinit(s_qspi_params.id);
    memset(&s_qspi_params, 0, sizeof(app_qspi_params_t));

    return true;
}

uint32_t qspi_norf_read_dev_id(void)
{
    uint16_t ret = 0;
    uint8_t data[3];
    app_qspi_command_t command = {
        .instruction      = QSPI_NORF_CMD_RDID,
        .address          = 0,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_00_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 3,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .clock_stretch_en = 1,
#endif
    };

    s_qspi_rx_done = 0;
    ret = app_qspi_dma_command_receive_async(s_qspi_params.id, &command, data);
    if(0 == ret) {
        while(s_qspi_rx_done == 0);
    } else {
        return 0x00;
    }

    return (((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + data[2]);
}

uint32_t qspi_norf_read_dev_density(void) {
    uint16_t ret        = 0;
    uint32_t flash_size = 0;
    uint8_t data[4]     = {0};
    qspi_command_t command = {
        .instruction      = QSPI_NORF_CMD_SFUD,
        .address          = 0x000034,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .dummy_cycles     = 8,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = sizeof(data),
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .clock_stretch_en = 1,
#endif
    };

    s_qspi_rx_done = 0;
    ret = app_qspi_dma_command_receive_async(s_qspi_params.id, &command, &data[0]);
    if(0 == ret) {
        while(s_qspi_rx_done == 0);
    } else {
        return 0;
    }

    if (data[0] != 0 && data[3] < 0xFF)
    {
        flash_size = ((data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0] << 0) + 1) / 8;
    }

    return flash_size;
}


bool qspi_norf_reset(void)
{
    uint16_t ret = 0;
    uint8_t control_frame[1] = {QSPI_NORF_CMD_RSTEN};
    s_qspi_tx_done = 0;
    ret = app_qspi_dma_transmit_async_ex(s_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    if(0 == ret) {
        while(s_qspi_tx_done == 0);
    } else {
        return false;
    }

    control_frame[0] = QSPI_NORF_CMD_RST;
    s_qspi_tx_done = 0;
    ret = app_qspi_dma_transmit_async_ex(s_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    if(0 == ret) {
        while(s_qspi_tx_done == 0);
    } else {
        return false;
    }

    return true;
}

bool qspi_norf_power_down(void)
{
    uint16_t ret = 0;
    uint8_t control_frame[1] = {QSPI_NORF_CMD_DP};

    s_qspi_tx_done = 0;
    ret = app_qspi_dma_transmit_async_ex(s_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    if(0 == ret) {
        while(s_qspi_tx_done == 0);
    } else {
        return false;
    }

    return true;
}

bool qspi_norf_wakeup(void)
{
    uint16_t ret = 0;
    uint8_t control_frame[1] = {QSPI_NORF_CMD_RDP};

    s_qspi_tx_done = 0;
    ret = app_qspi_dma_transmit_async_ex(s_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    if(0 == ret) {
        while(s_qspi_tx_done == 0);
    } else {
        return false;
    }

    return true;
}


void qspi_norf_unprotect(void)
{
    uint32_t reg_status = _qspi_norf_read_status_reg();
    uint32_t reg_config;

    switch (s_norflash_type)
    {
        case QSPI_NORF_TYE_GD25:
        {
            reg_status &= ~0x41FC;
            _qspi_norf_write_status_reg(reg_status);
        }
        break;

        case QSPI_NORF_TYE_PY25:
        {
            reg_status &= ~0x41FC;
            _qspi_norf_write_status_reg(reg_status);
        }
        break;

        case QSPI_NORF_TYE_MX25:
        {
            reg_config = _qspi_norf_read_config_reg();
            reg_status = ((reg_status & ~0xFC) & 0xFF) | ((reg_config << 8) & 0xFFFF00);
            _qspi_norf_write_status_reg(reg_status);
        }
        break;
    }
}

void qspi_norf_enable_quad(void)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    uint32_t reg_status = _qspi_norf_read_status_reg();
    uint32_t reg_config;

    switch (s_norflash_type)
    {
        case QSPI_NORF_TYE_GD25:
        case QSPI_NORF_TYE_PY25:
        case QSPI_NORF_TYE_XTX:
        {
            if (!(reg_status & 0x0200))
            {
                reg_status |= 0x0200;
                _qspi_norf_write_status_reg(reg_status);
            }
        }
        break;

        case QSPI_NORF_TYE_MX25:
        {
            if (!(reg_status & 0x40))
            {
                reg_config = _qspi_norf_read_config_reg();
                reg_status = ((reg_status | 0x40) & 0xFF) | ((reg_config << 8) & 0xFFFF00);
                _qspi_norf_write_status_reg(reg_status);
            }
        }
        break;

        case QSPI_NORF_TYE_SST26:
        {
            reg_config = _qspi_norf_read_config_reg();
            if (!(reg_config & 0x02))
            {
                reg_status = (reg_status & 0xFF) | (((reg_config | 0x02) << 8) & 0xFF00);
                _qspi_norf_write_status_reg(reg_status);
            }
        }
        break;
    }

    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin = s_qspi_params.pin_cfg.io_2.pin;
    io_init.mux = APP_IO_MUX_0;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(s_qspi_params.pin_cfg.io_2.type, &io_init);

    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin = s_qspi_params.pin_cfg.io_3.pin;
    io_init.mux = APP_IO_MUX_0;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(s_qspi_params.pin_cfg.io_3.type, &io_init);

    return;
}

void qspi_norf_disable_quad(void)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    uint32_t reg_status = _qspi_norf_read_status_reg();
    uint32_t reg_config;

    switch (s_norflash_type)
    {
        case QSPI_NORF_TYE_GD25:
        case QSPI_NORF_TYE_PY25:
        case QSPI_NORF_TYE_XTX:
        {
            if (reg_status & 0x0200)
            {
                reg_status &= ~0x0200;
                _qspi_norf_write_status_reg(reg_status);
            }
        }
        break;

        case QSPI_NORF_TYE_MX25:
        {
            if (reg_status & 0x40)
            {
                reg_config = _qspi_norf_read_config_reg();
                reg_status = ((reg_status & ~0x40) & 0xFF) | ((reg_config << 8) & 0xFFFF00);
                _qspi_norf_write_status_reg(reg_status);
            }
        }
        break;

        case QSPI_NORF_TYE_SST26:
        {
            reg_config = _qspi_norf_read_config_reg();
            if (reg_config & 0x02)
            {
                reg_status = (reg_status & 0xFF) | (((reg_config & ~0x02) << 8) & 0xFF00);
                _qspi_norf_write_status_reg(reg_status);
            }
        }
        break;
    }

    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin = s_qspi_params.pin_cfg.io_2.pin;
    io_init.mux = APP_IO_MUX;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(s_qspi_params.pin_cfg.io_2.type, &io_init);

    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin = s_qspi_params.pin_cfg.io_3.pin;
    io_init.mux = APP_IO_MUX;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(s_qspi_params.pin_cfg.io_3.type, &io_init);

    app_io_write_pin(s_qspi_params.pin_cfg.io_2.type, s_qspi_params.pin_cfg.io_2.pin, APP_IO_PIN_SET);
    app_io_write_pin(s_qspi_params.pin_cfg.io_3.type, s_qspi_params.pin_cfg.io_3.pin, APP_IO_PIN_SET);

    return;
}

bool qspi_norf_dev_read(uint32_t addr, uint8_t *buffer, uint32_t nbytes, norf_rmode_e rmode, norf_data_xfer_width_e width) {

    uint16_t ret = 0;
    app_qspi_command_t command =
    {
        .address          = addr,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .data_size        = width,
        .length           = nbytes,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .clock_stretch_en = 1,
#endif
    };

    switch (rmode) {
        case NORF_RMODE_READ :
        {
            command.instruction              = QSPI_NORF_CMD_READ;
            command.dummy_cycles             = 0;
            command.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI;
            command.data_mode                = QSPI_DATA_MODE_SPI;
        }
        break;

        case NORF_RMODE_FAST_READ:
        {
            command.instruction              = QSPI_NORF_CMD_FREAD;
            command.dummy_cycles             = 8;
            command.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI;
            command.data_mode                = QSPI_DATA_MODE_SPI;
        }
        break;

        case NORF_RMODE_DUAL_READ:
        {
            command.instruction              = QSPI_NORF_CMD_DOFR;
            command.dummy_cycles             = 8;
            command.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI;
            command.data_mode                = QSPI_DATA_MODE_DUALSPI;
        }
        break;

        case NORF_RMODE_2xIO_READ:
        {
            command.instruction              = QSPI_NORF_CMD_DIOFR;
            command.dummy_cycles             = 4;
            command.instruction_address_mode = QSPI_INST_IN_SPI_ADDR_IN_SPIFRF;
            command.data_mode                = QSPI_DATA_MODE_DUALSPI;
        }
        break;

        case NORF_RMODE_QUAD_READ:
        {
            command.instruction              = QSPI_NORF_CMD_QOFR;
            command.dummy_cycles             = 8;
            command.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI;
            command.data_mode                = QSPI_DATA_MODE_QUADSPI;
        }
        break;

        case NORF_RMODE_4xIO_READ:
        {
            command.instruction              = QSPI_NORF_CMD_QIOFR;
            command.dummy_cycles             = 6;
            command.instruction_address_mode = QSPI_INST_IN_SPI_ADDR_IN_SPIFRF;
            command.data_mode                = QSPI_DATA_MODE_QUADSPI;
        }
        break;

        default:
        {
            return false;
        }
        //break;
    }

    s_qspi_rx_done = 0;
    ret = app_qspi_dma_command_receive_async(s_qspi_params.id, &command, buffer);
    if(0 == ret) {
        while(s_qspi_rx_done == 0);
    } else {
        return false;
    }

    return true;
}

bool qspi_norf_dev_write(uint32_t addr, uint8_t *buffer, uint32_t nbytes, norf_wmode_e wmode, norf_data_xfer_width_e width) {
    uint16_t ret = 0;

    app_qspi_command_t command = {
        .address          = addr,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = width,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .length           = nbytes,  // MUST <= 256
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .clock_stretch_en = 1,
#endif
    };

    if(nbytes > 256) {
        return false;
    }

    switch(wmode) {
        case NORF_WMODE_PP:
        {
            command.instruction = QSPI_NORF_CMD_PP;
            command.data_mode   = QSPI_DATA_MODE_SPI;
        }
        break;

        case NORF_WMODE_DUAL_PP:
        {
            command.instruction = QSPI_NORF_CMD_DPP;
            command.data_mode   = QSPI_DATA_MODE_DUALSPI;
        }
        break;

        case NORF_WMODE_QUAD_PP:
        {
            command.instruction = QSPI_NORF_CMD_QPP;
            command.data_mode   = QSPI_DATA_MODE_QUADSPI;
        }
        break;

        default:
        {
            return false;
        }
        //break;
    }

    _qspi_norf_wren();

    s_qspi_tx_done = 0;
    ret = app_qspi_dma_command_transmit_async(s_qspi_params.id, &command, buffer);
    if(0 == ret) {
        while(s_qspi_tx_done == 0);
    } else {
        return false;
    }
    _qspi_norf_wait_busy();

    return true;
}


bool qspi_norf_dev_erase(uint32_t addr, norf_emode_e emode) {

    uint16_t ret = 0;
    uint16_t cmd_len = 4;
    uint8_t erase_cmd[4];
    erase_cmd[0] = 0x00;
    erase_cmd[1] = (addr >> 16) & 0xFF;
    erase_cmd[2] = (addr >>  8) & 0xFF;
    erase_cmd[3] = (addr >>  0) & 0xFF;

    switch(emode) {

        case NORF_EMODE_PAGE:
        {
            erase_cmd[0] = QSPI_NORF_CMD_PE;
            cmd_len      = 4;
        }
        break;

        case NORF_EMODE_SECTOR:
        {
            erase_cmd[0] = QSPI_NORF_CMD_SE;
            cmd_len      = 4;
        }
        break;

        case NORF_EMODE_BLOCK_32K:
        {
            erase_cmd[0] = QSPI_NORF_CMD_BE_32;
            cmd_len      = 4;
        }
        break;

        case NORF_EMODE_BLOCK_64K:
        {
            erase_cmd[0] = QSPI_NORF_CMD_BE_64;
            cmd_len      = 4;
        }
        break;

        case NORF_EMODE_CHIP:
        {
            erase_cmd[0] = QSPI_NORF_CMD_CE;
            cmd_len      = 1;
        }
        break;

        default:
        {
            return false;
        }
        //break;
    }

    _qspi_norf_wren();

    s_qspi_tx_done = 0;
    ret = app_qspi_dma_transmit_async_ex(s_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, erase_cmd, cmd_len);

    if(0 == ret) {
        while(s_qspi_tx_done == 0);
    } else {
        return false;
    }

    _qspi_norf_wait_busy();

    return true;
}

void qspi_norf_set_mmap(bool mmap) {
    if(mmap) {
        app_qspi_mmap_device_t q_dev = {
            .dev_type    = APP_QSPI_DEVICE_FLASH,
            .rd.flash_rd = FLASH_MMAP_CMD_4READ_EBH,
        };

        qspi_norf_enable_quad();
        app_qspi_config_memory_mappped(s_qspi_params.id,  q_dev);
        app_qspi_mmap_set_endian_mode(s_qspi_params.id,   APP_QSPI_MMAP_ENDIAN_MODE_1);
    } else {
        app_qspi_active_memory_mappped(s_qspi_params.id, false);
    }

    return;
}

void qspi_norf_set_sleep(bool is_sleep) {
    if(is_sleep) {
        qspi_norf_set_mmap(false);
        qspi_norf_power_down();
        _qspi_norf_io_deinit();
    } else {
        _qspi_norf_io_reinit();
        qspi_norf_wakeup();
        qspi_norf_set_mmap(true);
    }
}

/********************************************************************************************
 *                       Static Functions
 ********************************************************************************************/

static void _qspi_norf_callback_handler(app_qspi_evt_t *p_evt)
{
    if (p_evt->type == APP_QSPI_EVT_TX_CPLT)
    {
        s_qspi_tx_done = 1;
    }
    if (p_evt->type == APP_QSPI_EVT_RX_DATA)
    {
        s_qspi_rx_done = 1;
    }
    if ((p_evt->type == APP_QSPI_EVT_ERROR) || (p_evt->type == APP_QSPI_EVT_ABORT))
    {
        s_qspi_tx_done = 1;
        s_qspi_rx_done = 1;
    }
}

static bool _qspi_norf_wren(void)
{
    uint16_t ret = 0;
    uint8_t control_frame[1] = {QSPI_NORF_CMD_WREN};

    s_qspi_tx_done = 0;
    ret = app_qspi_dma_transmit_async_ex(s_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    if(0 == ret) {
        while(s_qspi_tx_done == 0);
    } else {
        return false;
    }

    return true;
}

static void _qspi_norf_wait_busy(void)
{
    while (_qspi_norf_read_status_reg() & 1);
}

static uint32_t _qspi_norf_read_status_reg(void)
{
    uint32_t ret = 0;
    uint8_t *pret = (uint8_t*)&ret;
    app_qspi_command_t command = {
        .instruction      = QSPI_NORF_CMD_RDSR,
        .address          = 0,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_00_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 1,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .clock_stretch_en = 1,
#endif
    };

    switch (s_norflash_type)
    {
        case QSPI_NORF_TYE_GD25:
        case QSPI_NORF_TYE_PY25:
            s_qspi_rx_done = 0;
            app_qspi_dma_command_receive_async(s_qspi_params.id, &command, pret++);
            while(s_qspi_rx_done == 0);
            command.instruction = 0x35;
            break;
        case QSPI_NORF_TYE_MX25:
            break;
        case QSPI_NORF_TYE_SST26:
            break;
    }

    s_qspi_rx_done = 0;
    app_qspi_dma_command_receive_async(s_qspi_params.id, &command, pret);
    while(s_qspi_rx_done == 0);

    return ret;
}

static uint32_t _qspi_norf_read_config_reg(void)
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
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .clock_stretch_en = 1,
#endif
    };

    switch (s_norflash_type)
    {
        case QSPI_NORF_TYE_MX25:
            command.instruction = 0x15;
            command.length      = 2;
            break;
        case QSPI_NORF_TYE_SST26:
            command.instruction = 0x35;
            command.length      = 1;
            break;
    }
    s_qspi_tx_done = 0;
    app_qspi_dma_command_transmit_async(s_qspi_params.id, &command, (uint8_t*)&ret);
    while(s_qspi_tx_done == 0);

    return ret;
}

static void _qspi_norf_write_status_reg(uint32_t status)
{
    uint8_t control_frame[4], length = 3;
    control_frame[0] = QSPI_NORF_CMD_WRSR;
    control_frame[1] = status & 0xFF;
    control_frame[2] = (status >> 8) & 0xFF;
    control_frame[3] = (status >> 16) & 0xFF;

    _qspi_norf_wren();

    if (QSPI_NORF_TYE_MX25 == s_norflash_type)
        length = 4;
    else if (QSPI_NORF_TYE_PY25 == s_norflash_type)
    {
#if QSPI_NORF_PY25_WRSR_2BYTE
        length = 3;
#else /* the WRSR command of some Puya FLASH chips is followed by 1 byte parameter. */
        /* use two commands to write status register for Puya Flash. */
        length = 2;

        s_qspi_tx_done = 0;
        app_qspi_dma_transmit_async_ex(s_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, length);
        while(s_qspi_tx_done == 0);

        _qspi_norf_wait_busy();

        _qspi_norf_wren();

        control_frame[0] = QSPI_NORF_CMD_WRSR1;
        control_frame[1] = (status >> 8) & 0xFF;
#endif /* QSPI_NORF_PY25_RSDR_2BYTE */
    }

    s_qspi_tx_done = 0;
    app_qspi_dma_transmit_async_ex(s_qspi_params.id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, length);
    while(s_qspi_tx_done == 0);

    _qspi_norf_wait_busy();

    return;
}

static void _qspi_norf_io_reinit(void) {
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;

    io_init.pull = APP_IO_NOPULL;
    io_init.mode = APP_IO_MODE_MUX;

    io_init.pin  = s_qspi_params.pin_cfg.cs.pin;
    io_init.mux  = s_qspi_params.pin_cfg.cs.mux;
    app_io_init(s_qspi_params.pin_cfg.cs.type, &io_init);

    io_init.pin  = s_qspi_params.pin_cfg.clk.pin;
    io_init.mux  = s_qspi_params.pin_cfg.clk.mux;
    app_io_init(s_qspi_params.pin_cfg.clk.type, &io_init);

    io_init.pin  = s_qspi_params.pin_cfg.io_0.pin;
    io_init.mux  = s_qspi_params.pin_cfg.io_0.mux;
    app_io_init(s_qspi_params.pin_cfg.io_0.type, &io_init);

    io_init.pin  = s_qspi_params.pin_cfg.io_1.pin;
    io_init.mux  = s_qspi_params.pin_cfg.io_1.mux;
    app_io_init(s_qspi_params.pin_cfg.io_1.type, &io_init);

    io_init.pin  = s_qspi_params.pin_cfg.io_2.pin;
    io_init.mux  = s_qspi_params.pin_cfg.io_2.mux;
    app_io_init(s_qspi_params.pin_cfg.io_2.type, &io_init);

    io_init.pin  = s_qspi_params.pin_cfg.io_3.pin;
    io_init.mux  = s_qspi_params.pin_cfg.io_3.mux;
    app_io_init(s_qspi_params.pin_cfg.io_3.type, &io_init);
}

static void _qspi_norf_io_deinit(void) {
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;

    io_init.pull = APP_IO_PULLDOWN;
    io_init.mode = APP_IO_MODE_INPUT;
    io_init.mux  = APP_IO_MUX_7;

    io_init.pin  = s_qspi_params.pin_cfg.clk.pin;
    app_io_init(s_qspi_params.pin_cfg.clk.type, &io_init);

    io_init.pin  = s_qspi_params.pin_cfg.io_0.pin;
    app_io_init(s_qspi_params.pin_cfg.io_0.type, &io_init);

    io_init.pin  = s_qspi_params.pin_cfg.io_1.pin;
    app_io_init(s_qspi_params.pin_cfg.io_1.type, &io_init);

    io_init.pin  = s_qspi_params.pin_cfg.io_2.pin;
    app_io_init(s_qspi_params.pin_cfg.io_2.type, &io_init);

    io_init.pin  = s_qspi_params.pin_cfg.io_3.pin;
    app_io_init(s_qspi_params.pin_cfg.io_3.type, &io_init);

    //CS
    io_init.pull = APP_IO_NOPULL;
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin  = s_qspi_params.pin_cfg.cs.pin;
    io_init.mux  = APP_IO_MUX_7;
    app_io_init(s_qspi_params.pin_cfg.cs.type, &io_init);
    app_io_write_pin(s_qspi_params.pin_cfg.cs.type, s_qspi_params.pin_cfg.cs.pin, APP_IO_PIN_SET);
}

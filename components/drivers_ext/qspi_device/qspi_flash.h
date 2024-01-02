/**
 ****************************************************************************************
 *
 * @file qspi_flash.h
 *
 * @brief Header file - spi flash funtion
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
#ifndef __QSPI_FLASH_H__
#define __QSPI_FLASH_H__

#include <stdint.h>
#include "app_qspi.h"

#define SPI_FLASH_CMD_WRSR              0x01
#define SPI_FLASH_CMD_WRSR1             0x31
#define SPI_FLASH_CMD_RDSR              0x05

#define SPI_FLASH_CMD_WREN              0x06
#define SPI_FLASH_CMD_WRDI              0x04

#define SPI_FLASH_CMD_READ              0x03
#define SPI_FLASH_CMD_FREAD             0x0B
#define SPI_FLASH_CMD_DOFR              0x3B
#define SPI_FLASH_CMD_DIOFR             0xBB
#define SPI_FLASH_CMD_QOFR              0x6B
#define SPI_FLASH_CMD_QIOFR             0xEB
#define SPI_FLASH_CMD_READ_RESET        0xFF

#define SPI_FLASH_CMD_PP                0x02
#define SPI_FLASH_CMD_SE                0x20
#define SPI_FLASH_CMD_BE_32             0x52
#define SPI_FLASH_CMD_BE_64             0xD8
#define SPI_FLASH_CMD_CE                0xC7
#define SPI_FLASH_CMD_PES               0x75
#define SPI_FLASH_CMD_PER               0x7A

#define SPI_FLASH_CMD_DPP               0xA2
#define SPI_FLASH_CMD_QPP               0x32

#define SPI_FLASH_CMD_RDI               0xAB
#define SPI_FLASH_CMD_REMS              0x90
#define SPI_FLASH_CMD_RDID              0x9F

#define SPI_FLASH_CMD_RSTEN             0x66
#define SPI_FLASH_CMD_RST               0x99
#define SPI_FLASH_CMD_DP                0xB9
#define SPI_FLASH_CMD_RDP               0xAB

#define DUMMY_BYTE                      0xFF

#define SPI_FLASH_PAGE_SIZE             0x000100
#define SPI_FLASH_SECTOR_SIZE           0x001000
#define SPI_FLASH_BLOCK_SIZE            0x010000
#define SPI_FLASH_ADDRESS_MAX           0x0FFFFF

#define SPI_FLASH_TYE_GD25              0xC8
#define SPI_FLASH_TYE_PY25              0x85
#define SPI_FLASH_TYE_MX25              0xC2
#define SPI_FLASH_TYE_SST26             0xBF
#define SPI_FLASH_TYE_XTX               0x0B

#define SPI_FLASH_PY25_WRSR_2BYTE       0           /* the WRSR command of some Puya FLASH chips is followed by 2 bytes parameter. e.g. P25Q80L, P25Q80H, P2580U... */

#define QSPI_POLLING                    APP_QSPI_TYPE_POLLING
#define QSPI_DMA                        APP_QSPI_TYPE_DMA

/**
  * @brief QSPI flash data size Enumerations definition
  */
typedef enum
{
    QSPI_FLASH_DATA_SIZE_08_BITS = QSPI_DATASIZE_08_BITS,
    QSPI_FLASH_DATA_SIZE_16_BITS = QSPI_DATASIZE_16_BITS,
    QSPI_FLASH_DATA_SIZE_32_BITS = QSPI_DATASIZE_32_BITS
} qspi_flash_data_size_t;

uint8_t SPI_FLASH_init(app_qspi_id_t id, uint32_t clock_prescaler, qspi_pins_group_e pin_group);
void SPI_FLASH_deinit(void);
void SPI_FLASH_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes);
void SPI_FLASH_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes);
void SPI_FLASH_Dual_Output_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes);
void SPI_FLASH_Dual_IO_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes);
void SPI_FLASH_Quad_Output_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes);
void SPI_FLASH_Quad_IO_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes);
uint32_t SPI_FLASH_Read_Device_ID(void);
void SPI_FLASH_Enable_Quad(void);
void SPI_FLASH_Disable_Quad(void);
void SPI_FLASH_Unprotect(void);
void SPI_FLASH_Sector_Erase(uint32_t Dst);
void SPI_FLASH_Block_Erase_32K(uint32_t Dst);
void SPI_FLASH_Block_Erase_64K(uint32_t Dst);
void SPI_FLASH_Chip_Erase(void);
void SPI_FLASH_Reset(void);
void SPI_FLASH_PowerDown(void);
void SPI_FLASH_Wakeup(void);
void SPI_FLASH_Page_Program(uint32_t Dst, uint8_t *data);
void SPI_FLASH_Dual_Page_Program(uint32_t Dst, uint8_t *data);
void SPI_FLASH_Quad_Page_Program(uint32_t Dst, uint8_t *data);
void SPI_FLASH_Page_Program_With_Data_Size(uint32_t Dst, uint8_t *data, qspi_flash_data_size_t data_size);
void SPI_FLASH_Dual_Page_Program_With_Data_Size(uint32_t Dst, uint8_t *data, qspi_flash_data_size_t data_size);
void SPI_FLASH_Quad_Page_Program_With_Data_Size(uint32_t Dst, uint8_t *data, qspi_flash_data_size_t data_size);

#endif /* __QSPI_FLASH_H__ */

/**
 ****************************************************************************************
 *
 * @file qspi_nand_flash.h
 *
 * @brief Header file - qspi nand flash funtion
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
#ifndef __QSPI_NAND_FLASH_H__
#define __QSPI_NAND_FLASH_H__

#include <stdint.h>
#include "app_qspi.h"
#include "app_qspi_dma.h"


/*
 * implement by DOSILICON.DS35
 *****************************************************************************************
 */

#define NAND_FLASH_CMD_RDID              0x9F
#define NAND_FLASH_CMD_RESET             0xFF

#define NAND_FLASH_CMD_GET_FEAT          0x0F       /* set feature */
#define NAND_FLASH_CMD_SET_FEAT          0x1F       /* get feature */

#define NAND_FLASH_CMD_WREN              0x06       /* write enable */
#define NAND_FLASH_CMD_WRDI              0x04       /* write disable */

#define NAND_FLASH_CMD_PREAD_2_CACHE     0x13       /* page read to cache */

#define NAND_FLASH_CMD_READ              0x03       /* Random Single Read */
#define NAND_FLASH_CMD_READ_ALIAS        0x0B       /* same with 0x03 */
#define NAND_FLASH_CMD_DREAD             0x3B       /* Random Dual Read From Cache */
#define NAND_FLASH_CMD_QREAD             0x6B       /* Random Quad Read From Cache */

#define NAND_FLASH_CMD_PROG_LOAD         0x02       /* Load program data with cache reset by Single mode */
#define NAND_FLASH_CMD_PROG_QLOAD        0x32       /* Load program data with cache reset by Quad Mode */
#define NAND_FLASH_CMD_PROG_LOAD_RDM     0x84       /* Load program data without cache reset by Single mode */
#define NAND_FLASH_CMD_PROG_QLOAD_RDM    0x34       /* Load program data without cache reset by Quad Mode */
#define NAND_FLASH_CMD_PROG_EXE          0x10       /* Load program data without cache reset by Quad Mode */

#define NAND_FLASH_CMD_BLK_ERASE         0xD8       /* Block Erase */

#define NAND_MAX_BLOCKS                       1024u
#define NAND_MAX_PAGES_PER_BLOCK              64u
#define NAND_MAX_BYTES_PER_PAGE               2112u

uint32_t    qspi_nand_flash_init(app_qspi_id_t id, uint32_t clock_prescaler, qspi_pins_group_e pin_group);
void        qspi_nand_flash_deinit(void);
void        qspi_nand_flash_unlock_block(app_qspi_id_t qspi_id);
void        qspi_nand_flash_enable_quad(app_qspi_id_t qspi_id);
void        qspi_nand_flash_disable_quad(app_qspi_id_t qspi_id);
bool        qspi_nand_flash_erase_block(app_qspi_id_t qspi_id, uint32_t block_address);
uint32_t    qspi_nand_flash_read_id(app_qspi_id_t qspi_id);

bool qspi_nand_flash_std_read(app_qspi_id_t qspi_id, uint32_t block, uint32_t page, uint32_t page_offset, uint8_t * data, uint32_t length);
bool qspi_nand_flash_dual_read(app_qspi_id_t qspi_id, uint32_t block, uint32_t page, uint32_t page_offset, uint8_t * data, uint32_t length);
bool qspi_nand_flash_quad_read(app_qspi_id_t qspi_id, uint32_t block, uint32_t page, uint32_t page_offset, uint8_t * data, uint32_t length);

bool qspi_nand_flash_std_write(app_qspi_id_t qspi_id, uint32_t block, uint32_t page, uint32_t page_offset, uint8_t * data, uint32_t length);
bool qspi_nand_flash_quad_write(app_qspi_id_t qspi_id, uint32_t block, uint32_t page, uint32_t page_offset, uint8_t * data, uint32_t length);

#endif /* __QSPI_NAND_FLASH_H__ */

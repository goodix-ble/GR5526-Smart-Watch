/**
 ****************************************************************************************
 *
 * @file qspi_norflash_v2.h
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
#ifndef __QSPI_NORFLASH_V2_H__
#define __QSPI_NORFLASH_V2_H__

#include <stdint.h>
#include "app_qspi.h"
#include "app_qspi_dma.h"


#define QSPI_NORF_CMD_WRSR              0x01
#define QSPI_NORF_CMD_WRSR1             0x31
#define QSPI_NORF_CMD_RDSR              0x05

#define QSPI_NORF_CMD_WREN              0x06
#define QSPI_NORF_CMD_WRDI              0x04

#define QSPI_NORF_CMD_READ              0x03
#define QSPI_NORF_CMD_FREAD             0x0B
#define QSPI_NORF_CMD_DOFR              0x3B
#define QSPI_NORF_CMD_DIOFR             0xBB
#define QSPI_NORF_CMD_QOFR              0x6B
#define QSPI_NORF_CMD_QIOFR             0xEB
#define QSPI_NORF_CMD_READ_RESET        0xFF

#define QSPI_NORF_CMD_PP                0x02
#define QSPI_NORF_CMD_PE                0x81
#define QSPI_NORF_CMD_SE                0x20
#define QSPI_NORF_CMD_BE_32             0x52
#define QSPI_NORF_CMD_BE_64             0xD8
#define QSPI_NORF_CMD_CE                0xC7
#define QSPI_NORF_CMD_PES               0x75
#define QSPI_NORF_CMD_PER               0x7A

#define QSPI_NORF_CMD_DPP               0xA2
#define QSPI_NORF_CMD_QPP               0x32

#define QSPI_NORF_CMD_RDI               0xAB
#define QSPI_NORF_CMD_REMS              0x90
#define QSPI_NORF_CMD_RDID              0x9F

#define QSPI_NORF_CMD_RSTEN             0x66
#define QSPI_NORF_CMD_RST               0x99
#define QSPI_NORF_CMD_DP                0xB9
#define QSPI_NORF_CMD_RDP               0xAB

#define QSPI_NORF_CMD_SFUD              0x5A

#define DUMMY_BYTE                      0xFF

#define QSPI_NORF_PAGE_SIZE             0x000100
#define QSPI_NORF_SECTOR_SIZE           0x001000
#define QSPI_NORF_BLOCK_SIZE            0x010000
#define QSPI_NORF_ADDRESS_MAX           0x0FFFFF

#define QSPI_NORF_TYE_GD25              0xC8
#define QSPI_NORF_TYE_PY25              0x85
#define QSPI_NORF_TYE_MX25              0xC2
#define QSPI_NORF_TYE_SST26             0xBF
#define QSPI_NORF_TYE_XTX               0x0B

#define QSPI_NORF_PY25_WRSR_2BYTE       0           /**< the WRSR command of some Puya FLASH chips is followed by 2 bytes parameter. e.g. P25Q80L, P25Q80H, P2580U... */


/**
  * @brief QSPI flash data size Enumerations definition
  */
typedef enum {
    NORF_RMODE_READ = 0,        /**< SPI Mode with Read CMD : 03H */
    NORF_RMODE_FAST_READ,       /**< SPI Mode with Read CMD : 0BH */
    NORF_RMODE_DUAL_READ,       /**< DualSPI Mode with Read CMD : 3BH */
    NORF_RMODE_2xIO_READ,       /**< DualSPI Mode with Read CMD : BBH */
    NORF_RMODE_QUAD_READ,       /**< QuadSPI Mode with Read CMD : 6BH */
    NORF_RMODE_4xIO_READ,       /**< QuadSPI Mode with Read CMD : EBH */
} norf_rmode_e;


typedef enum {
    NORF_WMODE_PP = 0,          /**< Page Program CMD : 02H */
    NORF_WMODE_DUAL_PP,         /**< Dual Page Program CMD : A2H */
    NORF_WMODE_QUAD_PP,         /**< Quad Page Program CMD : 32H */
} norf_wmode_e;

typedef enum {
    NORF_EMODE_PAGE = 0,        /**< Erase Mode in Page */
    NORF_EMODE_SECTOR,          /**< Erase Mode in Sector */
    NORF_EMODE_BLOCK_32K,       /**< Erase Mode in 32K Block */
    NORF_EMODE_BLOCK_64K,       /**< Erase Mode in STD Block */
    NORF_EMODE_CHIP,            /**< Erase Mode in CHIP */
} norf_emode_e;


typedef enum {
    NORF_XFER_WIDTH_BYTE        = QSPI_DATASIZE_08_BITS,
    NORF_XFER_WIDTH_HALFWORD    = QSPI_DATASIZE_16_BITS,
    NORF_XFER_WIDTH_WORD        = QSPI_DATASIZE_32_BITS,
} norf_data_xfer_width_e;


/********************************************************************************************
 *                       Public Declarations
 ********************************************************************************************/
uint8_t     qspi_norf_init(app_qspi_id_t id, uint32_t clock_prescaler, app_qspi_pin_cfg_t * pin_cfg);
bool        qspi_norf_deinit(void) ;
uint32_t    qspi_norf_read_dev_id(void);
uint32_t    qspi_norf_read_dev_density(void);
bool        qspi_norf_reset(void);
bool        qspi_norf_power_down(void);
bool        qspi_norf_wakeup(void);
void        qspi_norf_unprotect(void);
void        qspi_norf_enable_quad(void);
void        qspi_norf_disable_quad(void);
bool        qspi_norf_dev_read(uint32_t addr, uint8_t *buffer, uint32_t nbytes, norf_rmode_e rmode, norf_data_xfer_width_e width);
bool        qspi_norf_dev_write(uint32_t addr, uint8_t *buffer, uint32_t nbytes, norf_wmode_e wmode, norf_data_xfer_width_e width);
bool        qspi_norf_dev_erase(uint32_t addr, norf_emode_e emode);
void        qspi_norf_set_mmap(bool mmap);
void        qspi_norf_set_sleep(bool is_sleep);

#endif /* __QSPI_NORFLASH_V2_H__ */

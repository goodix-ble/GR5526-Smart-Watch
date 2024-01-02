/**
  ******************************************************************************
  * @file    hal_flash.h
  * @author  Engineering Team
  * @brief   This file contains HAL flash header definitions.
  ******************************************************************************
  * @attention
  *
  * Copyright(C) 2016-2017, Shenzhen Huiding Technology Co., Ltd
  * All Rights Reserved
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of Goodix Technology nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for Goodix Technology.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY Goodix Technology AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL Goodix Technology OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/**
 @addtogroup PERIPHERAL
 @{
*/

/**
  @addtogroup PERIPHERAL_API_HAL_FLASH_DRIVER HAL flash Interface
  @{
  @brief Definitions and prototypes for HAL flash Interface.
 */

#ifndef _HAL_FLASH_H
#define _HAL_FLASH_H

#include <stdint.h>
#include <stdbool.h>

/** @addtogroup HAL_FLASH_DRIVER_FUNCTIONS Functions
 * @{ */

/**
 *******************************************************************************
 * @brief Initialize flash access.
 *
 * @retval true             If successful.
 * @retval false            If failure.
 *******************************************************************************
 */
bool hal_flash_init( void );

/**
 *******************************************************************************
 * @brief Read flash Memory.
 *
 * @param[in]       addr    start address in flash to read data.
 * @param[in,out]   buf     buffer to read data to.
 * @param[in]       size    number of bytes to read.
 *
 * @return          number of bytes read
 *******************************************************************************
 */
uint32_t hal_flash_read(const uint32_t addr, uint8_t *buf, const uint32_t size);

/**
 *******************************************************************************
 * @brief Write flash Memory.
 *
 * @param[in]       addr    start address in flash to write data to.
 * @param[in,out]   buf     buffer of data to write.
 * @param[in]       size    number of bytes to write.
 *
 * @return          number of bytes written
 *******************************************************************************
 */
uint32_t hal_flash_write(const uint32_t addr, const uint8_t *buf, const uint32_t size);

/**
 *******************************************************************************
 * @brief Write flash Memory reliably. 
 *
 * @note It's possible that the data was not written into Flash Memory
 *       successfully. This function reads the data from Flash Memory to check
 *       the reliability of programming Flash Memory.
 * @param[in]       addr    start address in flash to write data to.
 * @param[in,out]   buf     buffer of data to write.
 * @param[in]       size    number of bytes to write.
 *
 * @return          number of bytes written
 *******************************************************************************
 */
uint32_t hal_flash_write_r(const uint32_t addr, const uint8_t *buf, const uint32_t size);

/**
 *******************************************************************************
 * @brief Enable encrypted and decrypted in write-read operations.
 *
 * @param[in]       enable  control encrypted and decrypte.
 *
 *******************************************************************************
 */
void hal_flash_set_security(bool enable);

/**
 *******************************************************************************
 * @brief Enable encrypted and decrypted in write-read operations.
 *
 * @retval true             Enable encrypted and decrypted.
 * @retval false            Disable encrypted and decrypted.
 *******************************************************************************
 */
bool hal_flash_get_security(void);

/**
 *******************************************************************************
 * @brief Erase flash region.
 *
 * @note All sectors that have address in range of [addr, addr+len]
 *       will be erased. If addr is not sector aligned, preceding data
 *       on the sector that addr belongs to will also be erased.
 *       If (addr + size) is not sector aligned, the whole sector
 *       will also be erased.
 *
 * @param[in] addr    start address in flash to write data to.
 * @param[in] size    number of bytes to write.
 *
 * @retval true       If successful.
 * @retval false      If failure.
 *******************************************************************************
 */
bool hal_flash_erase(const uint32_t addr, const uint32_t size);

/**
 *******************************************************************************
 * @brief Get Flash information.
 *
 * @param[in,out] id Pointer to flash id.
 * @param[in,out] size Pointer to flash size.
 *
 *******************************************************************************
 */
void hal_flash_get_info(uint32_t *id, uint32_t *size);

/**
 *******************************************************************************
 * @brief Erase flash chip.
 *
 * @retval true       If successful.
 * @retval false      If failure.
 *******************************************************************************
 */
bool hal_flash_erase_chip(void);


/*!
  * @brief Get size of a sector (which is smallest unit that can be erased)
  *
  * @return sector size in units of bytes
  */
/**
 *******************************************************************************
 * @brief Get size of a sector (which is smallest unit that can be erased).
 *
 * @return  sector size in units of bytes.
 *******************************************************************************
 */
uint32_t hal_flash_sector_size(void);

/** @} */

#endif /* _HAL_FLASH_H */

/** @} */
/** @} */

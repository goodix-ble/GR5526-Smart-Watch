/**
 ******************************************************************************
 *
 * @file gr55xx_nvds.h
 *
 * @brief NVDS API
 *
 ******************************************************************************
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

/**
 * @addtogroup SYSTEM
 * @{
 */
 /**
  @addtogroup NVDS Non-Volatile Data Storage
  @{
  @brief Definitions and prototypes for the NVDS interface.
 */

#ifndef __GR55XX_NVDS_H__
#define __GR55XX_NVDS_H__

#include <stdint.h>
#include <stdbool.h>

/** @addtogroup NVDS_DEFINES Defines
 * @{ */
#define NV_TAGCAT_APP       0x4000                                  /**< NVDS tag mask for user application. */
#define NV_TAG_APP(idx)     (NV_TAGCAT_APP | ((idx) & 0x3FFF))      /**< Get NVDS tag for user application.
                                                                         The values of Tag 0x0000 and 0xFFFF are invalid.
                                                                         Idx should not be used as the parameter of NVDS APIs directly.
                                                                         The range of idx is 0x0000~0x3FFF. */
/** @} */

/**@addtogroup NVDS_ENUMERATIONS Enumerations
 * @{ */
/**@brief NVDS Returned Status. */
enum NVDS_STATUS
{
    NVDS_SUCCESS,                       /**< NVDS succeeds. */
    NVDS_GC_COMPLETE,                   /**< NVDS garbage collection complete. */
    NVDS_FAIL,                          /**< NVDS failed. */
    NVDS_NOT_INIT,                      /**< NVDS not initialize. */
    NVDS_TAG_NOT_EXISTED,               /**< NVDS tag does not exist. */
    NVDS_SPACE_NOT_ENOUGH,              /**< NVDS space is not enough. */
    NVDS_LENGTH_OUT_OF_RANGE,           /**< NVDS length out of range. */
    NVDS_INVALID_PARA,                  /**< NVDS invalid params. */
    NVDS_INVALID_SECTORS,               /**< NVDS_INIT input parameter sectors error */
    NVDS_INIT_START_ADDR_NO_ALIGN,      /**< NVDS_INIT input parameter start addr is not 4K aligned. */
    NVDS_INIT_AREA_SLOP_OVER,           /**< NVDS_INIT input parameters cause nvds to out of range. */
    NVDS_INIT_CHECKSUM_FAIL,            /**< When nvds has been initialized and read tag, verification fail. */
    NVDS_INIT_TAG_LEN_ERR,              /**< When nvds has been initialized and read tag, the tag length is 0. */
    NVDS_INIT_READ_TAG_NOT_COMPLETE,    /**< When nvds has been initialized read tag not complate. */
    NVDS_INIT_MGR_ERR,                  /**< When nvds has been initialized and read tag complate, avail size is error. */
    NVDS_COMPACT_FAILED,                /**< NVDS failed to compact sectors. */
    NVDS_STORAGE_ACCESS_FAILED,         /**< NVDS failed to access storage. */
    NVDS_POINTER_NULL                   /**< NVDS or driver function repalce error: NULL. */
};
/** @} */

/**@addtogroup NVDS_TYPEDEFS Type Typedefs
 * @{ */
/**@brief NVDS Item tag. */
typedef uint16_t NvdsTag_t;
/** @} */


/**@addtogroup NVDS_STRUCTURES Structures
 * @{ */
/**@brief NVDS drive function struct. */
typedef struct
{
    uint32_t (*p_nvds_flash_read)(const uint32_t addr, uint8_t *buf, const uint32_t size);            /**< NVDS flash read function. */
    uint32_t (*p_nvds_flash_write_r)(const uint32_t addr, const uint8_t *buf, const uint32_t size);   /**< NVDS flash write_r function. */
    void (*p_nvds_flash_set_security)(bool enable);                                                   /**< NVDS flash security set function. */
    bool (*p_nvds_flash_get_security)(void);                                                          /**< NVDS flash security get function. */
    bool (*p_nvds_flash_erase)(const uint32_t addr, const uint32_t size);                             /**< NVDS flash erase function. */
    void (*p_nvds_flash_get_info)(uint32_t *id, uint32_t *size);                                      /**< NVDS flash information get function. */
}nvds_drv_func_t;

/**@brief NVDS function struct. */
typedef struct
{
    uint8_t  (*p_nvds_init)(uint32_t start_addr, uint8_t sectors);     /**< NVDS init function */
    uint8_t  (*p_nvds_get)(uint16_t tag, uint16_t *p_len, uint8_t *p_buf);                      /**< NVDS get function */
    uint8_t  (*p_nvds_put)(uint16_t tag, uint16_t len, const uint8_t *p_buf);                   /**< NVDS put function */
    uint8_t  (*p_nvds_del)(uint16_t tag);                                                       /**< NVDS delete function */
    uint16_t (*p_nvds_tag_length)(uint16_t tag);                                                /**< NVDS tag length get function */
}nvds_func_t;
/** @} */



/** @addtogroup NVDS_FUNCTIONS Functions
 * @{ */
/**
 ****************************************************************************************
 * @brief Initialize the sectors for NVDS.
 *
 * @note NVDS module will use one more sector flash for garbage collection.
 *
 * @param[in] start_addr:      Start address of NVDS area. If the value does not equal zero, it must be sector-aligned.
                               If the value equals zero, NVDS area will locate in the last two sector(s) in flash memory.
 * @param[in] sectors:         The number of sectors.
 *
 * @return Result of nvds init.
 ****************************************************************************************
 */
uint8_t nvds_init(uint32_t start_addr, uint8_t sectors);

/**
 ****************************************************************************************
 * @brief Read data from NVDS.
 *
 * @param[in]     tag:   Valid NVDS item tag.
 * @param[in,out] p_len: Pointer to the length of data.
 * @param[out]    p_buf: Data is read into the buffer.
 *
 * @return Result of nvds get.
 ****************************************************************************************
 */
uint8_t nvds_get(NvdsTag_t tag, uint16_t *p_len, uint8_t *p_buf);

/**
 ****************************************************************************************
 * @brief Write data to NVDS. If the tag does not exist, create one.
 *
 * @param[in] tag:   Valid NVDS item tag.
 * @param[in] len:   Length of data to be written.
 * @param[in] p_buf: Data to be written.
 *
 * @return Result of nvds put.
 ****************************************************************************************
 */
uint8_t nvds_put(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf);

/**
 ****************************************************************************************
 * @brief Delete a tag in NVDS
 *
 * @param[in] tag: The tag to be deleted.
 *
 * @return Result of nvds delete.
 ****************************************************************************************
 */
uint8_t nvds_del(NvdsTag_t tag);

/**
 ****************************************************************************************
 * @brief Get the length of a tag in NVDS
 *
 * @param[in] tag: The tag to get the length.
 *
 * @return Length of tag data.
 ****************************************************************************************
 */
uint16_t nvds_tag_length(NvdsTag_t tag);

/**
 ****************************************************************************************
 * @brief Replace the drive function.
 *
 * @param[in] p_nvds_drv_func: Pointer to drive functions.
 *
 * @return Result of drive function replace.
 ****************************************************************************************
 */
uint8_t nvds_drv_func_replace(nvds_drv_func_t *p_nvds_drv_func);

/**
 ****************************************************************************************
 * @brief Replace the function in NVDS.
 *
 * @param[in] p_nvds_func: Pointer to nvds functions.
 *
 * @return Result nvds function replace.
 ****************************************************************************************
 */
uint8_t nvds_func_replace(nvds_func_t *p_nvds_func);

/**
 ****************************************************************************************
 * @brief Calculate the space reserved for system.
 *
 * @param[in] bond_dev_num: The number of bond device.
 *
 * @return System retention size.
 ****************************************************************************************
 */
void nvds_retention_size(uint8_t bond_dev_num);
/** @} */
#endif

/** @} */
/** @} */


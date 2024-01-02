/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_efuse.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of eFuse HAL library.
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
 ****************************************************************************************
 */

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup HAL_DRIVER HAL Driver
  * @{
  */

/** @defgroup HAL_EFUSE EFUSE
  * @brief eFuse HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_EFUSE_H__
#define __GR55xx_HAL_EFUSE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"
#include "gr55xx_ll_efuse.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_EFUSE_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_EFUSE_state HAL EFUSE state
  * @{
  */

/**
  * @brief  HAL eFuse State Enumerations definition
  */
typedef enum
{
    HAL_EFUSE_STATE_RESET             = 0x00,    /**< Peripheral not yet initialized or disabled  */
    HAL_EFUSE_STATE_READY             = 0x01,    /**< Peripheral Initialized and ready for use    */
    HAL_EFUSE_STATE_BUSY              = 0x02,    /**< An internal process is ongoing              */
    HAL_EFUSE_STATE_ERROR             = 0x04     /**< Reception process is error                  */
} hal_efuse_state_t;
/** @} */

/** @} */

/** @addtogroup HAL_EFUSE_STRUCTURES Structures
  * @{
  */

/** @defgroup EFUSE_Configuration EFUSE Configuration
  * @{
  */

/**
  * @brief eFuse init Structure definition
  */
typedef struct _efuse_init
{
    uint32_t info_mode;                     /**< Specifies the info mode, enable or disable main or backup info. */

} efuse_init_t;

/** @} */

/** @defgroup EFUSE_handle EFUSE handle
  * @{
  */

/**
  * @brief eFuse handle Structure definition
  */
typedef struct _efuse_handle
{
    efuse_regs_t            *p_instance;    /**< Register base address          */

    efuse_init_t            init;           /**< eFuse base required parameters */

    __IO hal_lock_t         lock;           /**< Locking object                 */

    __IO hal_efuse_state_t  state;          /**< eFuse operation state          */

    __IO uint32_t           error_code;     /**< eFuse Error code               */

} efuse_handle_t;
/** @} */

/** @defgroup EFUSE_Keyram Keyram configuration
  * @{
  */

/**
  * @brief eFuse Keyram Structure definition
  */
typedef struct _keyram_mask
{
    uint32_t                aes_mask;         /**< AES port mask.                 */

    uint32_t                hmac_mask;        /**< HMAC port mask.                */

    uint32_t                present_mcu_mask; /**< Present of MCU port mask.      */

    uint32_t                present_xip_mask; /**< Present of XIP port mask.      */

    uint32_t                efuse_mask;       /**< eFuse port mask.               */

    uint32_t                enc_key_low;      /**< encrypt key low 32bit mask.    */

    uint32_t                enc_key_high;     /**< encrypt key high 32bit mask.   */
} keyram_mask_t;

/** @} */

/** @} */

/** @addtogroup HAL_EFUSE_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_EFUSE_Callback Callback
  * @{
  */

/**
  * @brief HAL_EFUSE Callback function definition
  */

typedef struct _efuse_callback
{
    void (*efuse_msp_init)(efuse_handle_t *p_efuse);        /**< EFUSE init MSP callback            */
    void (*efuse_msp_deinit)(efuse_handle_t *p_efuse);      /**< EFUSE de-init MSP callback         */
} efuse_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_EFUSE_MACRO Defines
  * @{
  */

/* Exported constants------------------------------------------------------------*/
/** @defgroup EFUSE_Exported_Constants EFUSE Exported Constants
  * @{
  */

/** @defgroup EFUSE_Error_Code EFUSE Error Code
  * @{
  */
#define HAL_EFUSE_ERROR_NONE                         ((uint32_t)0x00000000)              /**< No error                 */
#define HAL_EFUSE_ERROR_TIMEOUT                      ((uint32_t)0x00000001)              /**< Timeout error            */
#define HAL_EFUSE_ERROR_INVALID_PARAM                ((uint32_t)0x00000002)              /**< Invalid parameters error */
/** @} */

/** @defgroup EFUSE_Flags EFUSE Flags
  * @{
  */
#define EFUSE_FLAG_WRITE_KEYRAM_BUSY                 LL_EFUSE_WRITE_KEYRAM_BUSY          /**< Write keyram operation is in process  */
#define EFUSE_FLAG_READ_TRIM_DONE                    LL_EFUSE_READ_TRIM_DONE             /**< Read trim from eFuse has done            */
#define EFUSE_FLAG_CRC_CHECK_DONE                    LL_EFUSE_CRC_CHECK_DONE             /**< eFuse CRC check done                     */
#define EFUSE_FLAG_CRC_CHECK_SUCCESS                 LL_EFUSE_CRC_CHECK_SUCCESS          /**< CRC check succeeded                        */
#define EFUSE_FLAG_INIT_CHECK_DONE                   LL_EFUSE_INIT_CHECK_DONE            /**< eFuse initial value check done           */
#define EFUSE_FLAG_INIT_CHECK_SUCCESS                LL_EFUSE_INIT_CHECK_SUCCESS         /**< eFuse initial value check succeeded        */
#define EFUSE_FLAG_WRITE_DONE                        LL_EFUSE_WRITE_DONE                 /**< eFuse one word write done                */
#define EFUSE_FLAG_TEST_DONE                         LL_EFUSE_TEST_DONE                  /**< Read from eFuse has done in test mode    */
/** @} */

/** @defgroup EFUSE_Loyout_Map EFUSE Loyout Map
  * @{
  */
#define EFUSE_OFFSET_USER_DSVD                       (0x0000UL)                          /**< Reserved offset in backup info block             */
#define EFUSE_OFFSET_BBLK_TRIM                       (0x0020UL)                          /**< Triming offset in backup info block              */
#define EFUSE_OFFSET_BBLK_CONFIG                     (0x005CUL)                          /**< Configuration offset in backup info block        */
#define EFUSE_OFFSET_BBLK_SWD                        (0x0060UL)                          /**< SWD Enable offset in backup info block           */
#define EFUSE_OFFSET_BBLK_EncMode                    (0x0062UL)                          /**< Encryption Mode offset in backup info block      */
#define EFUSE_OFFSET_BBLK_CRC32                      (0x0064UL)                          /**< CRC32 offset in backup info block                */
#define EFUSE_OFFSET_BBLK_CHIP_ID                    (0x0068UL)                          /**< Chip ID offset in backup info block              */
#define EFUSE_OFFSET_BBLK_PRODUCT_ID                 (0x006EUL)                          /**< Product ID offset in backup info block           */
#define EFUSE_OFFSET_BBLK_FW_PUBLICKEY               (0x0070UL)                          /**< Firmware public key offset in backup info block */
#define EFUSE_OFFSET_BBLK_ROOT_PUBLICKEY             (0x0080UL)                          /**< Root public key offset in backup info block     */
#define EFUSE_OFFSET_BBLK_ECC_KEY                    (0x0090UL)                          /**< ECC key offset in backup info block              */
#define EFUSE_OFFSET_BBLK_FW_KEY                     (0x00B0UL)                          /**< Firmware key offset in backup info block         */
#define EFUSE_OFFSET_BBLK_HMAC_KEY                   (0x00D0UL)                          /**< HMAC key offset in backup info block             */
#define EFUSE_OFFSET_BBLK_DATA_KEY                   (0x00F0UL)                          /**< Data key offset in backup info block             */
#define EFUSE_OFFSET_MBLK_TRIM                       (0x0110UL)                          /**< Triming offset in main info block                */
#define EFUSE_OFFSET_MBLK_CONFIG                     (0x014CUL)                          /**< Configuration offset in main info block          */
#define EFUSE_OFFSET_MBLK_SWD                        (0x0150UL)                          /**< SWD Enable offset in main info block             */
#define EFUSE_OFFSET_MBLK_EncMode                    (0x0152UL)                          /**< Encryption Mode offset in main info block        */
#define EFUSE_OFFSET_MBLK_CRC32                      (0x0154UL)                          /**< CRC32 offset in main info block                  */
#define EFUSE_OFFSET_MBLK_CHIP_ID                    (0x0158UL)                          /**< Chip ID offset in main info block                */
#define EFUSE_OFFSET_MBLK_PRODUCT_ID                 (0x015EUL)                          /**< Product ID offset in main info block             */
#define EFUSE_OFFSET_MBLK_FW_PUBLICKEY               (0x0160UL)                          /**< Firmware public key offset in main info block   */
#define EFUSE_OFFSET_MBLK_ROOT_PUBLICKEY             (0x0170UL)                          /**< Root public key offset in main info block       */
#define EFUSE_OFFSET_MBLK_ECC_KEY                    (0x0180UL)                          /**< ECC key offset in main info block                */
#define EFUSE_OFFSET_MBLK_FW_KEY                     (0x01A0UL)                          /**< Firmware key offset in main info block           */
#define EFUSE_OFFSET_MBLK_HMAC_KEY                   (0x01C0UL)                          /**< HMAC key offset in main info block               */
#define EFUSE_OFFSET_MBLK_DATA_KEY                   (0x01E0UL)                          /**< Data key offset in main info block               */
#define EFUSE_OFFSET_END                             (0x0200UL)                          /**< eFuse Offset end                                 */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup EFUSE_Exported_Macros EFUSE Exported Macros
  * @{
  */
/** @brief  Enable the eFuse main or backup.
  * @param  __HANDLE__ Specifies the eFuse Handle.
  * @retval None.
  */
#define __HAL_EFUSE_ENABLE_MAIN_BACKUP(__HANDLE__)              ll_efuse_enable_main_backup((__HANDLE__)->p_instance)

/** @brief  Disable the eFuse main or backup.
  * @param  __HANDLE__ Specifies the eFuse Handle.
  * @retval None.
  */
#define __HAL_EFUSE_DISABLE_MAIN_BACKUP(__HANDLE__)             ll_efuse_disable_main_backup((__HANDLE__)->p_instance)

/** @brief  Enable the eFuse PGENB.
  * @param  __HANDLE__ Specifies the eFuse Handle.
  * @retval None.
  */
#define __HAL_EFUSE_ENABLE_PGENB(__HANDLE__)                    ll_efuse_enable_pgenb((__HANDLE__)->p_instance)

/** @brief  Disable the eFuse PGENB.
  * @param  __HANDLE__ Specifies the eFuse Handle.
  * @retval None.
  */
#define __HAL_EFUSE_DISABLE_PGENB(__HANDLE__)                   ll_efuse_disable_pgenb((__HANDLE__)->p_instance)

/** @brief  Check whether the specified eFuse flag is set or not.
  * @param  __HANDLE__ specifies the eFuse Handle.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref EFUSE_FLAG_WRITE_KEYRAM_BUSY  Write keyram operation is in process
  *            @arg @ref EFUSE_FLAG_READ_TRIM_DONE     Read trim from eFuse has done
  *            @arg @ref EFUSE_FLAG_CRC_CHECK_DONE     eFuse CRC check done
  *            @arg @ref EFUSE_FLAG_CRC_CHECK_SUCCESS  CRC check succeeded
  *            @arg @ref EFUSE_FLAG_INIT_CHECK_DONE    eFuse initial value check done
  *            @arg @ref EFUSE_FLAG_INIT_CHECK_SUCCESS eFuse initial value check succeeded
  *            @arg @ref EFUSE_FLAG_WRITE_DONE         eFuse one word write done
  *            @arg @ref EFUSE_FLAG_TEST_DONE          Read from eFuse has done in test mode
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_EFUSE_GET_FLAG(__HANDLE__, __FLAG__)              ((READ_BITS((__HANDLE__)->p_instance->STAT, (__FLAG__)) != 0) ? SET : RESET)
/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_EFUSE_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup EFUSE_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the EFUSEx peripheral:

      (+) User must implement hal_efuse_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_efuse_init() to configure the selected device with
          the selected configuration:
        (++) info_mode

      (+) Call the function hal_efuse_deinit() to restore the default configuration
          of the selected EFUSEx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the eFuse according to the specified parameters
 *         in the efuse_init_t and initialize the associated handle.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_efuse_init(efuse_handle_t *p_efuse);

/**
 ****************************************************************************************
 * @brief  De-initialize the eFuse peripheral.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_efuse_deinit(efuse_handle_t *p_efuse);

/**
 ****************************************************************************************
 * @brief  Initialize the eFuse MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_efuse_msp_init can be implemented in the user file.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 ****************************************************************************************
 */
void hal_efuse_msp_init(efuse_handle_t *p_efuse);

/**
 ****************************************************************************************
 * @brief  De-initialize the eFuse MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_efuse_msp_deinit can be implemented in the user file.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 ****************************************************************************************
 */
void hal_efuse_msp_deinit(efuse_handle_t *p_efuse);

/** @} */

/** @defgroup EFUSE_Exported_Functions_Group2 EFUSE operation functions
 *  @brief    eFuse Data manage functions
 *
 *  @{
 */

/**
 ****************************************************************************************
 * @brief  Write the eFuse memory data.
 *
 * @note   Address should be eFuse memory address.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 * @param[in]  word_offset: eFuse memory offset, unit word, this parament can be a value between: 0x00 ~ 0x80.
 * @param[in]  p_data: Pointer to data buffer for storage eFuse data.
 * @param[in]  nword: Size of data to be write, unit word.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_efuse_write(efuse_handle_t *p_efuse, uint32_t word_offset, uint32_t *p_data, uint32_t nword);

/**
 ****************************************************************************************
 * @brief  Read the eFuse memory data.
 *
 * @note   Address should be eFuse memory address.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 * @param[in]  word_offset: eFuse memory offset, unit word, this parament can be a value between: 0x000 ~ 0x80.
 * @param[in]  p_data: Pointer to data buffer for storage eFuse data.
 * @param[in]  nword: Size of data to be read, unit word.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_efuse_read(efuse_handle_t *p_efuse, uint32_t word_offset, uint32_t *p_data, uint32_t nword);

/**
 ****************************************************************************************
 * @brief  Read the key from eFuse memory and write to keyram.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 * @param[in]  p_mask: Pointer to a KEYRAM handle which contains the configuration information for the specified KEYRAM MASK.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_efuse_write_keyram(efuse_handle_t *p_efuse, keyram_mask_t *p_mask);

/**
 ****************************************************************************************
 * @brief  Check the eFuse memory with 0, if memory are all 0, return HAL_OK, then return HAL_ERROR.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_efuse_initial_value_check(efuse_handle_t *p_efuse);

/**
 ****************************************************************************************
 * @brief  Calculate CRC of the eFuse memory data.
 *
 * @note   Address must be main info eFuse memory address.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 * @param[in]  word_offset: eFuse memory offset, unit word, this parament can be a value between: 0x00 ~ 0x80.
 * @param[in]  nword: Size of data to be read, unit word, this parament can be a value between: 1 ~ 60.
 * @param[in]  p_result: Pointer to result.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_efuse_crc_calculate(efuse_handle_t *p_efuse, uint32_t word_offset, uint32_t nword, uint32_t *p_result);

/**
 ****************************************************************************************
 * @brief  Read trim value from eFuse memory data.
 *
 * @note   Address must be main info eFuse memory address.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 * @param[in]  word_offset: eFuse memory offset, unit word, this parament can be a value between: 0x00 ~ 0x80.
 * @param[in]  p_data: Pointer to data buffer for storage eFuse data.
 * @param[in]  nword: Size of data to be read, unit word, this parament can be a value between: 1 ~ 20.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_efuse_read_trim(efuse_handle_t *p_efuse, uint32_t word_offset, uint32_t *p_data, uint32_t nword);

/**
 ****************************************************************************************
 * @brief  Set the main or backup info of the eFuse memory.
 *
 * @param[in]  p_efuse: Pointer to a eFuse handle which contains the configuration information for the specified eFuse module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_efuse_set_main_backup(efuse_handle_t *p_efuse);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_EFUSE_H__ */

/** @} */

/** @} */

/** @} */

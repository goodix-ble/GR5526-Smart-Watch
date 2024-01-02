/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_exflash.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of EXFLASH HAL library.
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

/** @defgroup HAL_EXFLASH EXFLASH
  * @brief exFlash HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_EXFLASH_H__
#define __GR55xx_HAL_EXFLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_xqspi.h"
#include "gr55xx_hal_xqspi.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_EXFLASH_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_EXFLASH_STATE HAL EXFLASH State
  * @{
  */

/**
  * @brief HAL exFlash State Enumerations definition
  */
typedef enum
{
    HAL_EXFLASH_STATE_RESET             = 0x00,    /**< Peripheral not initialized */
    HAL_EXFLASH_STATE_READY             = 0x01,    /**< Peripheral initialized and ready for use */
    HAL_EXFLASH_STATE_BUSY              = 0x02,    /**< Peripheral in indirect mode and busy */
    HAL_EXFLASH_STATE_BUSY_READ         = 0x12,    /**< Peripheral in indirect mode with reception ongoing */
    HAL_EXFLASH_STATE_BUSY_WRITE        = 0x22,    /**< Peripheral in indirect mode with transmission ongoing */
    HAL_EXFLASH_STATE_BUSY_ERASE        = 0x42,    /**< Peripheral in indirect mode with erase ongoing */
    HAL_EXFLASH_STATE_SUSPEND_WRITE     = 0x21,    /**< Peripheral in suspend mode from transmission */
    HAL_EXFLASH_STATE_SUSPEND_ERASE     = 0x41,    /**< Peripheral in suspend mode from erase */
    HAL_EXFLASH_STATE_ERROR             = 0x04     /**< Peripheral in error */
} hal_exflash_state_t;
/** @} */

/** @defgroup HAL_EXFLASH_Security HAL EXFLASH Security
  * @{
  */

/**
  * @brief HAL exFlash Security Enumerations definition
  */
typedef enum
{
    HAL_EXFLASH_UNENCRYPTED             = 0x00,    /**< Data will not be encrypted and decrypted in write-read operations */
    HAL_EXFLASH_ENCRYPTED               = 0x01,    /**< Data will be encrypted and decrypted in write-read operations */
} hal_exflash_security_t;

/** @} */

/** @} */

/** @addtogroup HAL_EXFLASH_STRUCTURES Structures
  * @{
  */

/** @defgroup EXFLASH_HANDLE EXFLASH handle
  * @{
  */

/**
  * @brief exFlash handle Structure definition
  */
typedef struct _exflash_handle
{
    xqspi_handle_t       *p_xqspi;          /**< exFlash XQSPI Handle parameters       */

    __IO hal_lock_t       lock;             /**< Locking object                        */

    __IO hal_exflash_state_t state;         /**< exFlash communication state           */

    __IO hal_exflash_security_t security;     /**< exFlash data security                 */

    __IO uint32_t         flash_id;         /**< exFlash ID                            */

    __IO uint32_t         flash_size;       /**< exFlash Size                          */

    __IO uint32_t         error_code;       /**< exFlash Error code                    */

    uint32_t              retry;            /**< Repeat times for the exFlash memory access */

} exflash_handle_t;
/** @} */

/** @defgroup EXFLASH_AC EXFLASH AC
  * @{
  */
/**
  * @brief exFlash AC characteristics
  */
typedef struct _exflash_timing_param
{
    uint8_t               flash_tVSL;       /**< VCC(min.) to device operation. Uint: 10us */

    uint8_t               flash_tESL;       /**< Erase suspend latency. Uint: 5us */

    uint8_t               flash_tPSL;       /**< Program suspend latency. Uint: 5us */

    uint8_t               flash_tPRS;       /**< Latency between program resume and next suspend. Uint: 5us */

    uint8_t               flash_tERS;       /**< Latency between erase resume and next suspend. Uint: 5us */

    uint8_t               flash_tDP;        /**< CS# High to Deep Power-down Mode. Uint: 5us */

    uint8_t               flash_tRES2;      /**< CS# High To Standby Mode With Electronic Signature Read. Uint: 5us */

    uint8_t               flash_tRDINT;     /**< Read status register interval when wait busy. Uint: 5us */
} exflash_timing_param_t;
/** @} */

/** @defgroup EXFLASH_HP_Mode EXFLASH HP mode
  * @{
  */

/**
  * @brief   HP Mode structure definition
  */
typedef ll_xqspi_hp_init_t exflash_hp_init_t;
/** @} */

/** @} */

/** @addtogroup HAL_EXFLASH_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_EXFLASH_Callback EXFLASH_Callback
  * @{
  */

/**
  * @brief HAL_EXFLASH Callback function definition
  */

typedef struct _exflash_callback
{
    void (*exflash_msp_init)(void);      /**< EXFLASH init MSP callback            */
    void (*exflash_msp_deinit)(void);    /**< EXFLASH de-init MSP callback         */
} exflash_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_EXFLASH_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup EXFLASH_EXPORTED_CONSTANTS EXFLASH Exported Constants
  * @{
  */

/** @defgroup EXFLASH_ERROR_CODE EXFLASH Error Code
  * @{
  */
#define HAL_EXFLASH_ERROR_NONE              ((uint32_t)0x00000000) /**< No error                 */
#define HAL_EXFLASH_ERROR_TIMEOUT           ((uint32_t)0x00000001) /**< Timeout error            */
#define HAL_EXFLASH_ERROR_TRANSFER          ((uint32_t)0x00000002) /**< Transfer error           */
#define HAL_EXFLASH_ERROR_ID                ((uint32_t)0x00000003) /**< Flash ID error           */
#define HAL_EXFLASH_ERROR_QUAD              ((uint32_t)0x00000004) /**< Quad mode error           */
#define HAL_EXFLASH_ERROR_INVALID_PARAM     ((uint32_t)0x00000008) /**< Invalid parameters error */
/** @} */

/** @defgroup EXFLASH_ERASE_TYPE EXFLASH Erase Type
  * @{
  */
#define EXFLASH_ERASE_SECTOR                0   /**< Sector erase */
#define EXFLASH_ERASE_CHIP                  1   /**< Chip erase   */
/** @} */

/** @defgroup EXFLASH_PAGE_TYPE EXFLASH Page Type
  * @{
  */
#define EXFLASH_SINGLE_PAGE_TYPE            0   /**< single page */
#define EXFLASH_DUAL_PAGE_TYPE              1   /**< dual page   */
#define EXFLASH_QUAD_PAGE_TYPE              2   /**< dual page   */

/** @} */

/** @defgroup EXFLASH_SIZE_INFO EXFLASH Size Information
  * @{
  */
#define EXFLASH_SIZE_PAGE_BYTES             ((uint32_t)256)       /**< Page size in Bytes     */
#define EXFLASH_SIZE_SECTOR_BYTES           ((uint32_t)4096)      /**< Sector size in Bytes   */
#define EXFLASH_SIZE_CHIP_BYTES             ((uint32_t)0x1000000) /**< Chip size in Bytes     */
#define EXFLASH_START_ADDR                  FLASH_BASE            /**< Flash start address    */
#define EXFLASH_SIZE                        GR55XX_FLASH_SIZE     /**< Flash size             */
#define EXFLASH_END_ADDR                    (EXFLASH_START_ADDR + EXFLASH_SIZE) /**< Flash end address    */
#define EXFLASH_ALIAS_OFFSET                (0x02000000UL)       /**< Alias address offset   */
#define EXFLASH_ALIAS_ADDR                  (EXFLASH_START_ADDR + EXFLASH_ALIAS_OFFSET) /**< Alias start address   */
/** @} */

/** @defgroup EXFLASH_LOCK_AREA_TYPE EXFLASH Lock Area Type
  * @{
  */
#define EXFLASH_LOCK_AREA_NONE              0       /**< Offset NONE                */
#define EXFLASH_LOCK_AREA_UPPER_1_8         1       /**< Offset 0x070000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_UPPER_1_4         2       /**< Offset 0x060000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_UPPER_1_2         3       /**< Offset 0x040000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_LOWER_1_8         9       /**< Offset 0x000000 - 0x00FFFF */
#define EXFLASH_LOCK_AREA_LOWER_1_4         10      /**< Offset 0x000000 - 0x01FFFF */
#define EXFLASH_LOCK_AREA_LOWER_1_2         11      /**< Offset 0x000000 - 0x03FFFF */
#define EXFLASH_LOCK_AREA_ALL               12      /**< Offset 0x000000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_TOP_4K            17      /**< Offset 0x07F000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_TOP_8K            18      /**< Offset 0x07E000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_TOP_16K           19      /**< Offset 0x07C000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_TOP_32K           20      /**< Offset 0x078000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_BOTTOM_4K         25      /**< Offset 0x000000 - 0x000FFF */
#define EXFLASH_LOCK_AREA_BOTTOM_8K         26      /**< Offset 0x000000 - 0x001FFF */
#define EXFLASH_LOCK_AREA_BOTTOM_16K        27      /**< Offset 0x000000 - 0x003FFF */
#define EXFLASH_LOCK_AREA_BOTTOM_32K        28      /**< Offset 0x000000 - 0x007FFF */
#define EXFLASH_LOCK_AREA_LOWER_7_8         33      /**< Offset 0x070000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_LOWER_3_4         34      /**< Offset 0x060000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_UPPER_7_8         41      /**< Offset 0x000000 - 0x00FFFF */
#define EXFLASH_LOCK_AREA_UPPER_3_4         42      /**< Offset 0x000000 - 0x01FFFF */
#define EXFLASH_LOCK_AREA_LOWER_127_128     49      /**< Offset 0x000000 - 0x07EFFF */
#define EXFLASH_LOCK_AREA_LOWER_63_64       50      /**< Offset 0x000000 - 0x07DFFF */
#define EXFLASH_LOCK_AREA_LOWER_31_32       51      /**< Offset 0x000000 - 0x07BFFF */
#define EXFLASH_LOCK_AREA_LOWER_15_16       52      /**< Offset 0x000000 - 0x077FFF */
#define EXFLASH_LOCK_AREA_UPPER_127_128     57      /**< Offset 0x001000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_UPPER_63_64       58      /**< Offset 0x002000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_UPPER_31_32       59      /**< Offset 0x004000 - 0x07FFFF */
#define EXFLASH_LOCK_AREA_UPPER_15_16       60      /**< Offset 0x008000 - 0x07FFFF */
/** @} */

/** @defgroup EXFLASH_RETRY_DEFINITION EXFLASH Repeat Times definition
  * @{
  */
#define HAL_EXFLASH_RETRY_DEFAULT_VALUE     ((uint32_t)40000)          /**< 400000 times */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup EXFLASH_EXPORTED_MACROS EXFLASH Exported Macros
  * @{
  */

/** @brief  Reset exFlash handle states.
  * @param  __HANDLE__: exFlash handle.
  * @retval None
  */
#define __HAL_EXFLASH_RESET_HANDLE_STATE(__HANDLE__)                ((__HANDLE__)->state = HAL_EXFLASH_STATE_RESET)

/** @brief  Enable the specified exFlash power.
  * @retval None
  */
#define __HAL_EXFLASH_POWER_ON()                                    ll_xqspi_enable_exflash_power()

/** @brief  Disable the specified exFlash power.
  * @retval None
  */
#define __HAL_EXFLASH_POWER_OFF()                                   ll_xqspi_disable_exflash_power()

/** @brief  Check the specified exFlash power state.
  * @retval None
  */
#define __HAL_EXFLASH_POWER_IS_ON()                                 ll_xqspi_is_enable_exflash_power()

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup EXFLASH_PRIVATE_MACRO EXFLASH Private Macros
  * @{
  */

/**
  * @brief Check if exFlash erase type is valid.
  * @param __TYPE__ exFlash erase type.
  * @retval SET (__TYPE__ is valid)
  * @retval RESET (__TYPE__ is invalid)
  */
#define IS_EXFLASH_ERASE_TYPE(__TYPE__)            (((__TYPE__) == EXFLASH_ERASE_SECTOR) || \
                                                    ((__TYPE__) == EXFLASH_ERASE_CHIP))

/**
  * @brief Check if exFlash lock area type is valid.
  * @param __AREA__ exFlash lock area type.
  * @retval SET (__AREA__ is valid)
  * @retval RESET (__AREA__ is invalid)
  */
#define IS_EXFLASH_LOCK_AREA(__AREA__)             (((__AREA__) == EXFLASH_LOCK_AREA_NONE)          || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_UPPER_1_8)     || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_UPPER_1_4)     || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_UPPER_1_2)     || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_LOWER_1_8)     || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_LOWER_1_4)     || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_LOWER_1_2)     || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_ALL)           || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_TOP_4K)        || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_TOP_8K)        || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_TOP_16K)       || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_TOP_32K)       || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_BOTTOM_4K)     || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_BOTTOM_8K)     || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_BOTTOM_16K)    || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_BOTTOM_32K)    || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_LOWER_7_8)     || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_LOWER_3_4)     || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_UPPER_7_8)     || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_UPPER_3_4)     || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_LOWER_127_128) || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_LOWER_63_64)   || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_LOWER_31_32)   || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_LOWER_15_16)   || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_UPPER_127_128) || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_UPPER_63_64)   || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_UPPER_31_32)   || \
                                                    ((__AREA__) == EXFLASH_LOCK_AREA_UPPER_15_16))
/** @} */

/** @} */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_EXFLASH_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup EXFLASH_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the exFlash peripheral:

      (+) User must implement hal_exflash_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_exflash_deinit() to restore the default configuration
          of the selected exFlash peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the exFlash according to the specified parameters
 *         in the exflash_init_t and initialize the associated handle.
 *
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_init(void);

/**
 ****************************************************************************************
 * @brief  De-initialize the exFlash peripheral.
 *
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_deinit(void);

/**
 ****************************************************************************************
 * @brief  Initialize the exFlash MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_exflash_msp_init can be implemented in the user file.
 * 
 ****************************************************************************************
 */
void hal_exflash_msp_init(void);

/**
 ****************************************************************************************
 * @brief  De-initialize the exFlash MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_exflash_msp_deinit can be implemented in the user file.
 * 
 ****************************************************************************************
 */
void hal_exflash_msp_deinit(void);

/** @} */

/** @defgroup EXFLASH_EXPORTED_FUNCTIONS_GROUP2 EXFLASH operation functions
 *  @brief   Data transfers functions
 *
@verbatim
  ==============================================================================
                      ##### EXFLASH operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the exFlash
    data transfers.

    [..] The exFlash supports XIP and QSPI mode:

    (#) There are only one modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  According to the model of internal flash, configure AC parameters to meet the requirements of flash operation timing.
 *
 * @note  If this function is not called, the flash interface will use default parameters.
 *
 * @param[in]  p_time: Pointer to a timing structure which contains the AC parameters information for the specified exFlash module.
 ****************************************************************************************
 */
void hal_exflash_timing_set(exflash_timing_param_t *p_time);

/**
 ****************************************************************************************
 * @brief  During Flash erase/write operation, Disable external interrupts with a priority less than or equal to base_priority in the system.
 *
 * @param[in]  base_priority: Base Priority value to set.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_operation_protection(uint32_t base_priority);

/**
 ****************************************************************************************
 * @brief  Configure the security mode of flash interface.
 *
 * @note  If the security mode of flash interface is set to encrypted. the plaintext data will be stored as cipertext when the
 *        hal_exflash_write is called. and the cipertext data stored in flash will be read as plaintext data when the hal_exflash_read
 *        is called.
 *
 * @param[in]  mode: Specifies the value to be written to the selected enum.
 *             @arg HAL_EXFLASH_UNENCRYPTED: Data will be not encrypted and decrypted in write-read operations
 *             @arg HAL_EXFLASH_ENCRYPTED: Data will be encrypted and decrypted in write-read operations
 ****************************************************************************************
 */
void hal_exflash_set_security(hal_exflash_security_t mode);

/**
 ****************************************************************************************
 * @brief  Read the security mode of flash interface.
 *
 * @retval ::HAL_EXFLASH_UNENCRYPTED: Secure read / write is not supported
 * @retval ::HAL_EXFLASH_ENCRYPTED: Secure read / write is supported.
 ****************************************************************************************
 */
hal_exflash_security_t hal_exflash_get_security(void);

/**
 ****************************************************************************************
 * @brief Configure page programming buffer size.
 *
 * @note This function provides a 256/512/1024 Byte page programming buffer that can increase programming performance.
 *       However, in encrypted mode, only support 256 Byte page programming.
 *
 * @param[in]  size_type: This parameter can be one of the following values:
 *         @arg @ref EXFLASH_SINGLE_PAGE_TYPE
 *         @arg @ref EXFLASH_DUAL_PAGE_TYPE
 *         @arg @ref EXFLASH_QUAD_PAGE_TYPE
 ****************************************************************************************
 */
hal_status_t hal_exflash_page_configure(uint32_t size_type);

/**
 ****************************************************************************************
 * @brief  Write an amount of data with specified instruction and address to flash.
 *
 * @note   This function is used only in Indirect Write Mode. In secure mode, address alignment requires 4 bytes.
 *
 * @param[in]  addr: Address to write data in flash, start at @ref EXFLASH_START_ADDR.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Size of buffer bytes
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_write(uint32_t addr, uint8_t *p_data, uint32_t size);

/**
 ****************************************************************************************
 * @brief  Read an amount of data with specified instruction and address from flash.
 *
 * @note   This function is used only in non-encrypted Indirect Read Mode.
 *
 * @param[in]  addr: Address to read data in flash, start at @ref EXFLASH_START_ADDR.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  size: Size of buffer bytes
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_read(uint32_t addr, uint8_t *p_data, uint32_t size);

/**
 ****************************************************************************************
 * @brief  Erase flash region.
 *
 * @note   All sectors that have address in range of [addr, addr+len] will be erased. If addr is not sector aligned,
 *         preceding data on the sector that addr belongs to will also be erased. If (addr + size) is not sector
 *         aligned, the whole sector will also be erased. If erase_type is @ref EXFLASH_ERASE_CHIP , all data in flash
 *         will be erased ignored addr and size.
 *
 * @param[in]  erase_type: Erase flash with sector/chip.
 *                    @arg @ref EXFLASH_ERASE_SECTOR
 *                    @arg @ref EXFLASH_ERASE_CHIP
 * @param[in]  addr: Address to erased data in flash, start at @ref EXFLASH_START_ADDR.
 * @param[in]  size: Size of erased bytes.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_erase(uint32_t erase_type, uint32_t addr, uint32_t size);

/**
 ****************************************************************************************
 * @brief  Suspend flash pragram/erase.
 *
 * @note   The Suspend instruction interrupts a Page Program, Sector Erase, or Block Erase operation to allow access
 *         to the memory array. After the program or erase operation has entered the suspended state, the memory
 *         array can be read except for the page being programmed or the sector or block being erased. This function
 *         is only used in XIP mode.
 *
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_suspend(void);

/**
 ****************************************************************************************
 * @brief  Resume flash pragram/erase.
 *
 * @note   The Resume instruction resumes a suspended Page Program, Sector Erase, or Block Erase operation.
 *         Before issuing the Resume instruction to restart a suspended erase operation, make sure that there is no
 *         Page Program operation in progress. This function is only used in XIP mode.
 *
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_resume(void);

/**
 ****************************************************************************************
 * @brief  Lock area of flash to be software protected against Write and Erase operation.
 *
 * @note   This function is used only in Mirror Mode.
 *
 * @param[in]  lock_type Area need to lock
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_lock(uint32_t lock_type);

/**
 ****************************************************************************************
 * @brief  Unlock write/erase protected in flash.
 *
 * @note   This function is used only in Mirror Mode.
 *
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_unlock(void);

/**
 ****************************************************************************************
 * @brief  the exFlash will go to the Deep Power-Down Mode.
 *
 * @note   This function is used only in Mirror Mode.
 * 
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_deepsleep(void);

/**
 ****************************************************************************************
 * @brief  exFlash will be released from Deep Power-Down Mode.
 *
 * @note   This function is used only in Mirror Mode.
 *
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_wakeup(void);

/**
 ****************************************************************************************
 * @brief  Enable Quad mode to allow Quad operation.
 *
 * @note   This function is used only in Mirror Mode.
 * 
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_enable_quad(exflash_hp_init_t *hp_init);

/**
 ****************************************************************************************
 * @brief  This function serves to read UID of flash
 *
 * @param[out]  uid: store 16 Byte flash UID
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_read_uid(uint8_t *uid);

/** @} */

/** @defgroup EXFLASH_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   exFlash control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the exFlash.
     (+) hal_exflash_get_state()API can be helpful to check in run-time the state of the exFlash peripheral.
     (+) hal_exflash_get_error() check in run-time Errors occurring during communication.
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the exFlash handle state.
 *
 *
 * @retval ::HAL_EXFLASH_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_EXFLASH_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_EXFLASH_STATE_BUSY: Peripheral in indirect mode and busy.
 * @retval ::HAL_EXFLASH_STATE_BUSY_WRITE: Peripheral in indirect mode with transmission ongoing.
 * @retval ::HAL_EXFLASH_STATE_BUSY_READ: Peripheral in indirect mode with reception ongoing.
 * @retval ::HAL_EXFLASH_STATE_BUSY_ERASE: Peripheral in indirect mode with erase ongoing.
 * @retval ::HAL_EXFLASH_STATE_ERROR: Peripheral in error.
 ****************************************************************************************
 */
hal_exflash_state_t hal_exflash_get_state(void);

/**
 ****************************************************************************************
 * @brief  Return the exFlash error code.
 *
 *
 * @return exFlash error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_exflash_get_error(void);

/**
 ****************************************************************************************
 * @brief  This function reads the status register of a flash.
 *
 * @note The status register is a 16-bit register that provides information about the flash operation status.
 *
 * @param[in] p_reg_status: Pointer of status register.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_read_status_reg(uint16_t* p_reg_status);

/**
 ****************************************************************************************
 * @brief  This function writes the status register of a flash.
 *
 * @note The status register is a 16-bit register that provides information about the flash operation status.
 *
 * @param reg_status: An integer value representing the content to be written to the flash status register.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_exflash_write_status_reg(uint16_t reg_status);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_EXFLASH_H__ */

/** @} */

/** @} */

/** @} */

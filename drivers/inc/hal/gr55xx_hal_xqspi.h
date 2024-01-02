/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_xqspi.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of XQSPI HAL library.
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

/** @defgroup HAL_XQSPI XQSPI
  * @brief XQSPI HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_XQSPI_H__
#define __GR55xx_HAL_XQSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_xqspi.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_XQSPI_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_XQSPI_state HAL XQSPI state
  * @{
  */

/**
  * @brief HAL XQSPI State Enumerations definition
  */
typedef enum
{
    HAL_XQSPI_STATE_RESET             = 0x00,    /**< Peripheral not initialized                            */
    HAL_XQSPI_STATE_READY             = 0x01,    /**< Peripheral initialized and ready for use              */
    HAL_XQSPI_STATE_BUSY              = 0x02,    /**< Peripheral in indirect mode and busy                  */
    HAL_XQSPI_STATE_BUSY_INDIRECT_TX  = 0x12,    /**< Peripheral in indirect mode with transmission ongoing */
    HAL_XQSPI_STATE_BUSY_INDIRECT_RX  = 0x22,    /**< Peripheral in indirect mode with reception ongoing    */
    HAL_XQSPI_STATE_ABORT             = 0x08,    /**< Peripheral with abort request ongoing                 */
    HAL_XQSPI_STATE_ERROR             = 0x04     /**< Peripheral in error                                   */

} hal_xqspi_state_t;

/** @} */

/** @} */

/** @addtogroup HAL_XQSPI_STRUCTURES Structures
  * @{
  */

/** @defgroup XQSPI_Configuration XQSPI Configuration
  * @{
  */

/**
  * @brief XQSPI init Structure definition
  */
typedef struct _xqspi_init_t
{
    uint32_t work_mode;             /**< Specifies the work mode for XQSPI.
                                     This parameter can be a value of @ref XQSPI_Work_Mode */

    uint32_t cache_mode;            /**< Specifies the cache mode for XIP mode.
                                     This parameter can be a value of @ref XQSPI_Cache_Mode */

    uint32_t read_cmd;              /**< Specifies the read command for transmit in XIP mode.
                                     This parameter can be a value of @ref XQSPI_Read_CMD */

    uint32_t baud_rate;             /**< Specifies the serial clock speed for transmit in both XIP and QSPI mode.
                                     This parameter can be a value of @ref XQSPI_Baud_Rate */

    uint32_t clock_mode;            /**< Specifies the Clock Mode. It indicates the level that clock takes between commands.
                                     This parameter can be a value of @ref XQSPI_Clock_Mode */

    uint32_t cache_direct_map_en;   /**< Specifies the XQSPI Cache work on direct map or 4-way set associative.
                                     This parameter can be a value of @ref XQSPI_Direct_Map_Cache_EN.*/

    uint32_t cache_flush;           /**< Specifies the XQSPI Cache will be flushed or not.
                                     This parameter can be a value of @ref XQSPI_Cache_Flush_EN.*/

    ll_xqspi_hp_init_t hp_init;     /**< Specifies the XQSPI HP mode Configuration.
                                     This structures is defined @ref ll_xqspi_hp_init_t.*/

} xqspi_init_t;
/** @} */

/** @defgroup XQSPI_handle XQSPI handle
  * @{
  */

/**
  * @brief XQSPI handle Structure definition
  */
typedef struct _xqspi_handle_t
{
    xqspi_regs_t          *p_instance;        /**< XQSPI registers base address        */

    xqspi_init_t          init;             /**< XQSPI communication parameters      */

    uint8_t               *p_tx_buffer;       /**< Pointer to XQSPI Tx transfer Buffer */

    __IO uint32_t         tx_xfer_size;     /**< XQSPI Tx Transfer size              */

    __IO uint32_t         tx_xfer_count;    /**< XQSPI Tx Transfer Counter           */

    uint8_t               *p_rx_buffer;       /**< Pointer to XQSPI Rx transfer Buffer */

    __IO uint32_t         rx_xfer_size;     /**< XQSPI Rx Transfer size              */

    __IO uint32_t         rx_xfer_count;    /**< XQSPI Rx Transfer Counter           */

    __IO hal_lock_t       lock;             /**< Locking object                      */

    __IO hal_xqspi_state_t state;           /**< XQSPI communication state           */

    __IO uint32_t         error_code;       /**< XQSPI Error code                    */

    uint32_t              retry;            /**< Retry for the XQSPI flag access     */

} xqspi_handle_t;
/** @} */

/** @defgroup XQSPI_Command XQSPI command
  * @{
  */

/**
  * @brief XQSPI command Structure definition
  */
typedef struct _xqspi_command_t
{
    uint32_t inst;                      /**< Specifies the Instruction to be sent.
                                             This parameter can be a value (8-bit) between 0x00 and 0xFF */

    uint32_t addr;                      /**< Specifies the Address to be sent (Size from 1 to 4 bytes according to AddressSize).
                                             This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */

    uint32_t inst_size;                 /**< Specifies the Instruction Size.
                                             This parameter can be a value of @ref XQSPI_Instruction_Size */

    uint32_t addr_size;                 /**< Specifies the Address Size.
                                             This parameter can be a value of @ref XQSPI_Address_Size */

    uint32_t dummy_cycles;              /**< Specifies the Number of Dummy Cycles.
                                             This parameter can be a number between 0 and 31 */

    uint32_t inst_addr_mode;            /**< Specifies the Instruction and Address Mode.
                                             This parameter can be a value of @ref XQSPI_Inst_Addr_Mode */

    uint32_t data_mode;                 /**< Specifies the Data Mode (used for dummy cycles and data phases).
                                             This parameter can be a value of @ref XQSPI_Data_Mode */

    uint32_t length;                    /**< Specifies the number of data to transfer. (This is the number of bytes).
                                             This parameter can be any value between 0 and 0xFFFFFFFF (0 means undefined length
                                             until end of memory)  */

} xqspi_command_t;
/** @} */

/** @} */

/** @addtogroup HAL_XQSPI_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_XQSPI_Callback Callback
  * @{
  */

/**
  * @brief HAL_XQSPI Callback function definition
  */

typedef struct _xqspi_callback
{
    void (*xqspi_msp_init)(xqspi_handle_t *p_xqspi);    /**< XQSPI init MSP callback        */
    void (*xqspi_msp_deinit)(xqspi_handle_t *p_xqspi);  /**< XQSPI de-init MSP callback     */
} xqspi_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_XQSPI_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup XQSPI_Exported_Constants XQSPI Exported Constants
  * @{
  */

/** @defgroup XQSPI_Error_Code XQSPI Error Code
  * @{
  */
#define HAL_XQSPI_ERROR_NONE             ((uint32_t)0x00000000) /**< No error                 */
#define HAL_XQSPI_ERROR_TIMEOUT          ((uint32_t)0x00000001) /**< Timeout error            */
#define HAL_XQSPI_ERROR_TRANSFER         ((uint32_t)0x00000002) /**< Transfer error           */
#define HAL_XQSPI_ERROR_INVALID_PARAM    ((uint32_t)0x00000008) /**< Invalid parameter error */
/** @} */

/** @defgroup HAL_XQSPI_HP_MODE_EN XQSPI in HP mode
  * @{
  */
#define XQSPI_HP_MODE_DIS             LL_XQSPI_HP_MODE_DIS     /**< Disable XQSPI High Performance mode  */
#define XQSPI_HP_MODE_EN              LL_XQSPI_HP_MODE_EN      /**< Enable  XQSPI High Performance mode  */
/** @} */

/** @defgroup XQSPI_Work_Mode XQSPI Work Mode
  * @{
  */
#define XQSPI_WORK_MODE_QSPI             LL_XQSPI_MODE_QSPI     /**< Work in QSPI mode  */
#define XQSPI_WORK_MODE_XIP              LL_XQSPI_MODE_XIP      /**< Work in XIP mode   */
/** @} */

/** @defgroup XQSPI_Cache_Mode XQSPI Cache Mode in XIP mode
  * @{
  */
#define XQSPI_CACHE_MODE_DIS             LL_XQSPI_CACHE_DIS     /**< Cache off in XIP mode */
#define XQSPI_CACHE_MODE_EN              LL_XQSPI_CACHE_EN      /**< Cache on  in XIP mode */
/** @} */

/** @defgroup XQSPI_Direct_Map_Cache_EN XQSPI in XIP mode
  * @{
  */
#define XQSPI_CACHE_DIRECT_MAP_DIS       LL_XQSPI_CACHE_DIRECT_MAP_DIS     /**< Cache work on 4-Way Set Associative */
#define XQSPI_CACHE_DIRECT_MAP_EN        LL_XQSPI_CACHE_DIRECT_MAP_EN      /**< Cache work on Direct Map */
/** @} */

/** @defgroup XQSPI_Cache_Flush_EN XQSPI Cache flush enable
  * @{
  */
#define XQSPI_CACHE_FLUSH_DIS            LL_XQSPI_CACHE_FLUSH_DIS       /**< Cache Flush Disable */
#define XQSPI_CACHE_FLUSH_EN             LL_XQSPI_CACHE_FLUSH_EN        /**< Cache Flush Enable  */
/** @} */

/** @defgroup XQSPI_Read_CMD XQSPI Read Command in XIP mode
  * @{
  */
#define XQSPI_READ_CMD_READ              LL_XQSPI_XIP_CMD_READ           /**< Read mode                  */
#define XQSPI_READ_CMD_FAST_READ         LL_XQSPI_XIP_CMD_FAST_READ      /**< Fast Read mode             */
#define XQSPI_READ_CMD_DUAL_OUT_READ     LL_XQSPI_XIP_CMD_DUAL_OUT_READ  /**< Dual-Out Fast Read mode     */
#define XQSPI_READ_CMD_DUAL_IO_READ      LL_XQSPI_XIP_CMD_DUAL_IO_READ   /**< Dual-IO Fast Read mode      */
#define XQSPI_READ_CMD_QUAD_OUT_READ     LL_XQSPI_XIP_CMD_QUAD_OUT_READ  /**< Quad-Out Fast Read mode     */
#define XQSPI_READ_CMD_QUAD_IO_READ      LL_XQSPI_XIP_CMD_QUAD_IO_READ   /**< Quad-IO Fast Read mode      */
/** @} */

/** @defgroup XQSPI_Clock_Mode XQSPI Clock Mode
  * @{
  */
#define XQSPI_CLOCK_MODE_0               ((LL_XQSPI_SCPOL_LOW << 1) | LL_XQSPI_SCPHA_1EDGE)   /**< Inactive state of CLK is low,
                                                                                                   CLK toggles at the start of first data bit   */
#define XQSPI_CLOCK_MODE_1               ((LL_XQSPI_SCPOL_LOW << 1) | LL_XQSPI_SCPHA_2EDGE)   /**< Inactive state of CLK is low,
                                                                                                   CLK toggles in the middle of first data bit  */
#define XQSPI_CLOCK_MODE_2               ((LL_XQSPI_SCPOL_HIGH << 1) | LL_XQSPI_SCPHA_1EDGE)  /**< Inactive state of CLK is high,
                                                                                                   CLK toggles at the start of first data bit   */
#define XQSPI_CLOCK_MODE_3               ((LL_XQSPI_SCPOL_HIGH << 1) | LL_XQSPI_SCPHA_2EDGE)  /**< Inactive state of CLK is high,
                                                                                                   CLK toggles in the middle of first data bit  */
/** @} */

/** @defgroup XQSPI_Baud_Rate XQSPI Clock Speed
  * @{
  */
#define XQSPI_BAUD_RATE_64M              LL_XQSPI_BAUD_RATE_64M         /**< Serial clock speed is 64 MHz  */
#define XQSPI_BAUD_RATE_48M              LL_XQSPI_BAUD_RATE_48M         /**< Serial clock speed is 48 MHz  */
#define XQSPI_BAUD_RATE_32M              LL_XQSPI_BAUD_RATE_32M         /**< Serial clock speed is 32 MHz  */
#define XQSPI_BAUD_RATE_24M              LL_XQSPI_BAUD_RATE_24M         /**< Serial clock speed is 24 MHz  */
#define XQSPI_BAUD_RATE_16M              LL_XQSPI_BAUD_RATE_16M         /**< Serial clock speed is 16 MHz  */
/** @} */

/** @defgroup XQSPI_Data_Mode XQSPI Data Mode, only in QSPI mode
  * @{
  */
#define XQSPI_DATA_MODE_SPI              LL_XQSPI_QSPI_FRF_SPI          /**< Standard SPI Frame Format  */
#define XQSPI_DATA_MODE_DUALSPI          LL_XQSPI_QSPI_FRF_DUALSPI      /**< Dual-SPI Frame Format      */
#define XQSPI_DATA_MODE_QUADSPI          LL_XQSPI_QSPI_FRF_QUADSPI      /**< Quad-SPI Frame Format      */
/** @} */

/** @defgroup XQSPI_FIFO_Threshold XQSPI FIFO Threshold, FIFO depth is 64*4bytes, only in QSPI mode
  * @{
  */
#define XQSPI_FIFO_THRESHOLD_1_8         LL_XQSPI_QSPI_FIFO_WATERMARK_1_8  /**< FIFO depth/8      */
#define XQSPI_FIFO_THRESHOLD_1_4         LL_XQSPI_QSPI_FIFO_WATERMARK_1_4  /**< FIFO depth/4      */
#define XQSPI_FIFO_THRESHOLD_1_2         LL_XQSPI_QSPI_FIFO_WATERMARK_1_2  /**< FIFO depth/2      */
#define XQSPI_FIFO_THRESHOLD_3_4         LL_XQSPI_QSPI_FIFO_WATERMARK_3_4  /**< FIFO depth*3/4    */
#define XQSPI_FIFO_DEPTH                 LL_XQSPI_QSPI_FIFO_DEPTH          /**< FIFO full depth   */
/** @} */

/** @defgroup XQSPI_Instruction_Size XQSPI Instruction Size, only in QSPI mode
  * @{
  */
#define XQSPI_INSTSIZE_00_BITS           (0)                            /**< 0-bit  (No Instruction) */
#define XQSPI_INSTSIZE_08_BITS           (1)                            /**< 8-bit  Instruction      */
#define XQSPI_INSTSIZE_16_BITS           (2)                            /**< 16-bit Instruction      */
/** @} */

/** @defgroup XQSPI_Address_Size XQSPI Address Size, only in QSPI mode
  * @{
  */
#define XQSPI_ADDRSIZE_00_BITS           (0)                            /**< 0-bit  (No Address)  */
#define XQSPI_ADDRSIZE_08_BITS           (1)                            /**< 8-bit  Address       */
#define XQSPI_ADDRSIZE_16_BITS           (2)                            /**< 16-bit Address       */
#define XQSPI_ADDRSIZE_24_BITS           (3)                            /**< 24-bit Address       */
#define XQSPI_ADDRSIZE_32_BITS           (4)                            /**< 32-bit Address       */
/** @} */

/** @defgroup XQSPI_Inst_Addr_Mode XQSPI Instruction and Address Mode, only in QSPI mode
  * @{
  */
#define XQSPI_INST_ADDR_ALL_IN_SPI       (0)   /**< Instruction and address are sent in SPI mode */
#define XQSPI_INST_IN_SPI_ADDR_IN_SPIFRF (1)   /**< Instruction is sent in SPI mode, and address is sent in Daul/Quad SPI mode */
#define XQSPI_INST_ADDR_ALL_IN_SPIFRF    (2)   /**< Instruction and address are sent in Daul/Quad SPI mode */
/** @} */

/** @defgroup XQSPI_Flags XQSPI Flags, only in QSPI mode
  * @{
  */
#define XQSPI_FLAG_RFF                   LL_XQSPI_QSPI_STAT_RFF         /**< Rx FIFO full flag          */
#define XQSPI_FLAG_RFTF                  LL_XQSPI_QSPI_STAT_RFTF        /**< Rx FIFO threshold flag     */
#define XQSPI_FLAG_RFE                   LL_XQSPI_QSPI_STAT_RFE         /**< Rx FIFO empty flag         */
#define XQSPI_FLAG_TFF                   LL_XQSPI_QSPI_STAT_TFF         /**< Tx FIFO full flag          */
#define XQSPI_FLAG_TFTF                  LL_XQSPI_QSPI_STAT_TFTF        /**< Tx FIFO threshold flag     */
#define XQSPI_FLAG_TFE                   LL_XQSPI_QSPI_STAT_TFE         /**< Tx FIFO empty flag         */
#define XQSPI_FLAG_BUSY                  LL_XQSPI_QSPI_STAT_BUSY        /**< Busy flag                  */
/** @} */

/** @defgroup XQSPI_Ctrl_Present Control Present Status, only in XIP mode
  * @{
  */
#define XQSPI_DISABLE_PRESENT           LL_XQSPI_DISABLE_PRESENT        /**< Disable Present */
#define XQSPI_ENABLE_PRESENT            LL_XQSPI_ENABLE_PRESENT         /**< Enable Present  */
/** @} */

/** @defgroup  XQSPI_Retry_definition XQSPI Retry definition
  * @{
  */
#define HAL_XQSPI_RETRY_DEFAULT_VALUE ((uint32_t)1000)                  /**< 1000 times */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup XQSPI_Exported_Macros XQSPI Exported Macros
  * @{
  */

/** @brief  Reset XQSPI handle states.
  * @param  __HANDLE__ XQSPI handle.
  * @retval None
  */
#define __HAL_XQSPI_RESET_HANDLE_STATE(__HANDLE__)    ((__HANDLE__)->state = HAL_XQSPI_STATE_RESET)

/** @brief  Enable the specified QSPI peripheral in XQSPI.
  * @param  __HANDLE__ specifies the XQSPI Handle.
  * @retval None
  */
#define __HAL_XQSPI_ENABLE_QSPI(__HANDLE__)    SET_BITS((__HANDLE__)->p_instance->QSPI.SPIEN, XQSPI_QSPI_EN_EN)
/** @brief  Disable the specified QSPI peripheral in XQSPI.
  * @param  __HANDLE__ specifies the XQSPI Handle.
  * @retval None
  */

#define __HAL_XQSPI_DISABLE_QSPI(__HANDLE__)    CLEAR_BITS((__HANDLE__)->p_instance->QSPI.SPIEN, XQSPI_QSPI_EN_EN)

/** @brief  Enable the specified XIP peripheral in XQSPI.
  * @param  __HANDLE__ specifies the XQSPI Handle.
  * @retval None
  */

#define __HAL_XQSPI_ENABLE_XIP(__HANDLE__)    SET_BITS((__HANDLE__)->p_instance->XIP.CTRL3, XQSPI_QSPI_EN_EN); \
                                              while(!ll_xqspi_get_xip_flag(__HANDLE__->p_instance))

/** @brief  Disable the specified XIP peripheral in XQSPI.
  * @param  __HANDLE__ specifies the XQSPI Handle.
  * @retval None
  */

#define __HAL_XQSPI_DISABLE_XIP(__HANDLE__)    CLEAR_BITS((__HANDLE__)->p_instance->XIP.CTRL3, XQSPI_QSPI_EN_EN); \
                                               while(ll_xqspi_get_xip_flag(__HANDLE__->p_instance))

/** @brief  Enable the specified CACHE peripheral in XQSPI.
  * @param  __HANDLE__ specifies the XQSPI Handle.
  * @retval None
  */
#define __HAL_XQSPI_ENABLE_CACHE(__HANDLE__)    CLEAR_BITS((__HANDLE__)->p_instance->CACHE.CTRL0, XQSPI_CACHE_CTRL0_DIS)

/** @brief  Disable the specified CACHE peripheral in XQSPI.
  * @param  __HANDLE__ specifies the XQSPI Handle.
  * @retval None
  */
#define __HAL_XQSPI_DISABLE_CACHE(__HANDLE__)    SET_BITS((__HANDLE__)->p_instance->CACHE.CTRL0, XQSPI_CACHE_CTRL0_DIS)

/** @brief  Check whether the specified XQSPI flag is set or not.
  * @param  __HANDLE__ specifies the XQSPI Handle.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref XQSPI_FLAG_RFF  Rx FIFO full flag
  *            @arg @ref XQSPI_FLAG_RFTF Rx FIFO threshold flag
  *            @arg @ref XQSPI_FLAG_RFE  Rx FIFO empty flag
  *            @arg @ref XQSPI_FLAG_TFF  Tx FIFO full flag
  *            @arg @ref XQSPI_FLAG_TFTF Tx FIFO threshold flag
  *            @arg @ref XQSPI_FLAG_TFE  Tx FIFO empty flag
  *            @arg @ref XQSPI_FLAG_BUSY Busy flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_XQSPI_GET_FLAG(__HANDLE__, __FLAG__)    ((READ_BITS((__HANDLE__)->p_instance->QSPI.STAT, (__FLAG__)) != 0) ? SET : RESET)

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup XQSPI_Private_Macro XQSPI Private Macros
  * @{
  */

/** @brief  Check if XQSPI Work Mode is valid.
  * @param  __MODE__ XQSPI Work Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_XQSPI_WORK_MODE(__MODE__)             (((__MODE__) == XQSPI_WORK_MODE_QSPI) || \
                                                  ((__MODE__) == XQSPI_WORK_MODE_XIP))

/** @brief  Check if XQSPI Cache Mode is valid.
  * @param  __MODE__ XQSPI Cache Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_XQSPI_CACHE_MODE(__MODE__)            (((__MODE__) == XQSPI_CACHE_MODE_DIS) || \
                                                  ((__MODE__) == XQSPI_CACHE_MODE_EN))

/** @brief  Check if XQSPI Cache Direct Map is valid.
  * @param  __MODE__ XQSPI Cache Direct Map Enable Flag.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_XQSPI_CACHE_MAPPING(__MODE__)         (((__MODE__) == XQSPI_CACHE_DIRECT_MAP_DIS) || \
                                                  ((__MODE__) == XQSPI_CACHE_DIRECT_MAP_EN))

/** @brief  Check if XQSPI Cache Flush is valid.
  * @param  __MODE__ XQSPI Cache Flush Enable Flag.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_XQSPI_CACHE_FLUSH(__MODE__)           (((__MODE__) == XQSPI_CACHE_FLUSH_DIS) || \
                                                  ((__MODE__) == XQSPI_CACHE_FLUSH_EN))

/** @brief  Check if XQSPI Read CMD is valid.
  * @param  __CMD__ XQSPI Cache Mode.
  * @retval SET (__CMD__ is valid) or RESET (__CMD__ is invalid)
  */
#define IS_XQSPI_READ_CMD(__CMD__)               (((__CMD__) == XQSPI_READ_CMD_READ         ) || \
                                                  ((__CMD__) == XQSPI_READ_CMD_FAST_READ    ) || \
                                                  ((__CMD__) == XQSPI_READ_CMD_DUAL_OUT_READ) || \
                                                  ((__CMD__) == XQSPI_READ_CMD_DUAL_IO_READ ) || \
                                                  ((__CMD__) == XQSPI_READ_CMD_QUAD_OUT_READ) || \
                                                  ((__CMD__) == XQSPI_READ_CMD_QUAD_IO_READ ))

/** @brief  Check if XQSPI Clock Baud Rate is valid.
  * @param  __BAUD__ XQSPI Clock Baud Rate.
  * @retval SET (__BAUD__ is valid) or RESET (__BAUD__ is invalid)
  */
#define IS_XQSPI_BAUD_RATE(__BAUD__)             (((__BAUD__) == XQSPI_BAUD_RATE_64M) || \
                                                  ((__BAUD__) == XQSPI_BAUD_RATE_48M) || \
                                                  ((__BAUD__) == XQSPI_BAUD_RATE_32M) || \
                                                  ((__BAUD__) == XQSPI_BAUD_RATE_24M) || \
                                                  ((__BAUD__) == XQSPI_BAUD_RATE_16M))

/** @brief  Check if XQSPI Clock Mode is valid.
  * @param  __CLKMODE__ XQSPI Clock Mode.
  * @retval SET (__CLKMODE__ is valid) or RESET (__CLKMODE__ is invalid)
  */
#define IS_XQSPI_CLOCK_MODE(__CLKMODE__)         (((__CLKMODE__) == XQSPI_CLOCK_MODE_0) || \
                                                  ((__CLKMODE__) == XQSPI_CLOCK_MODE_1) || \
                                                  ((__CLKMODE__) == XQSPI_CLOCK_MODE_2) || \
                                                  ((__CLKMODE__) == XQSPI_CLOCK_MODE_3))

/** @brief  Check if XQSPI FIFO Threshold is valid.
  * @param  __THR__ XQSPI FIFO Threshold.
  * @retval SET (__THR__ is valid) or RESET (__THR__ is invalid)
  */
#define IS_XQSPI_FIFO_THRESHOLD(__THR__)         (((__THR__) == XQSPI_FIFO_THRESHOLD_1_8) || \
                                                  ((__THR__) == XQSPI_FIFO_THRESHOLD_1_4) || \
                                                  ((__THR__) == XQSPI_FIFO_THRESHOLD_1_2) || \
                                                  ((__THR__) == XQSPI_FIFO_THRESHOLD_3_4))

/** @brief  Check if XQSPI Instruction Size is valid.
  * @param  __INST_SIZE__ XQSPI Instruction Size.
  * @retval SET (__INST_SIZE__ is valid) or RESET (__INST_SIZE__ is invalid)
  */
#define IS_XQSPI_INSTRUCTION_SIZE(__INST_SIZE__) (((__INST_SIZE__) == XQSPI_INSTSIZE_00_BITS) || \
                                                  ((__INST_SIZE__) == XQSPI_INSTSIZE_08_BITS) || \
                                                  ((__INST_SIZE__) == XQSPI_INSTSIZE_16_BITS))

/** @brief  Check if XQSPI Address Size is valid.
  * @param  __ADDR_SIZE__ XQSPI Address Size .
  * @retval SET (__ADDR_SIZE__ is valid) or RESET (__ADDR_SIZE__ is invalid)
  */
#define IS_XQSPI_ADDRESS_SIZE(__ADDR_SIZE__)     (((__ADDR_SIZE__) == XQSPI_ADDRSIZE_00_BITS) || \
                                                  ((__ADDR_SIZE__) == XQSPI_ADDRSIZE_08_BITS) || \
                                                  ((__ADDR_SIZE__) == XQSPI_ADDRSIZE_16_BITS) || \
                                                  ((__ADDR_SIZE__) == XQSPI_ADDRSIZE_24_BITS) || \
                                                  ((__ADDR_SIZE__) == XQSPI_ADDRSIZE_32_BITS))

/** @brief  Check if XQSPI Instruction and Address Mode is valid.
  * @param  __MODE__ XQSPI Instruction and Address Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_XQSPI_INSTADDR_MODE(__MODE__)         (((__MODE__) == XQSPI_INST_ADDR_ALL_IN_SPI)       || \
                                                  ((__MODE__) == XQSPI_INST_IN_SPI_ADDR_IN_SPIFRF) || \
                                                  ((__MODE__) == XQSPI_INST_ADDR_ALL_IN_SPIFRF))

/** @brief  Check if XQSPI Data Mode is valid.
  * @param  __MODE__ XQSPI Data Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_XQSPI_DATA_MODE(__MODE__)             (((__MODE__) == XQSPI_DATA_MODE_SPI)     || \
                                                  ((__MODE__) == XQSPI_DATA_MODE_DUALSPI) || \
                                                  ((__MODE__) == XQSPI_DATA_MODE_QUADSPI))

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_XQSPI_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup XQSPI_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the XQSPIx peripheral:

      (+) User must implement hal_xqspi_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_xqspi_init() to configure the selected device with
          the selected configuration:
        (++) work_mode
        (++) cache_mode
        (++) read_cmd
        (++) baud_rate
        (++) clock_mode

      (+) Call the function hal_xqspi_deinit() to restore the default configuration
          of the selected XQSPIx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the XQSPI according to the specified parameters
 *         in the xqspi_init_t and initialize the associated handle.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_xqspi_init(xqspi_handle_t *p_xqspi);

/**
 ****************************************************************************************
 * @brief  De-initialize the XQSPI peripheral.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_xqspi_deinit(xqspi_handle_t *p_xqspi);

/**
 ****************************************************************************************
 * @brief  Initialize the XQSPI MSP.
 * @note   This function should not be modified. When the callback is needed,
            the hal_xqspi_msp_init can be implemented in the user file.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 ****************************************************************************************
 */
void hal_xqspi_msp_init(xqspi_handle_t *p_xqspi);

/**
 ****************************************************************************************
 * @brief  De-initialize the XQSPI MSP.
 * @note   This function should not be modified. When the callback is needed,
            the hal_xqspi_msp_deinit can be implemented in the user file.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 ****************************************************************************************
 */
void hal_xqspi_msp_deinit(xqspi_handle_t *p_xqspi);

/** @} */

/** @defgroup XQSPI_Exported_Functions_Group2 XQSPI operation functions
 *  @brief   Data transfers functions
 *
@verbatim
  ==============================================================================
                      ##### XQSPI operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the XQSPI
    data transfers.

    [..] The XQSPI supports master and slave mode:

    (#) There are one modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.

    (#) APIs provided for only one transfer mode (Blocking mode)
        exist for 1Line/2Line/4Line (simplex) modes.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with specified instruction and address in blocking mode.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @param[in]  p_cmd: Pointer to a xqspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  retry: Repeat times
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_xqspi_command_transmit(xqspi_handle_t *p_xqspi, xqspi_command_t *p_cmd, uint8_t *p_data, uint32_t retry);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with specified instruction and address in blocking mode.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @param[in]  p_cmd: Pointer to a xqspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  retry: Repeat times
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_xqspi_command_receive(xqspi_handle_t *p_xqspi, xqspi_command_t *p_cmd, uint8_t *p_data, uint32_t retry);

#if defined RTL_SIM
hal_status_t hal_xqspi_command_receive_rtl(xqspi_handle_t *p_xqspi, xqspi_command_t *p_cmd, uint8_t *p_data, uint32_t retry);
#endif


/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in blocking mode.
 * @note   This function is used only in Indirect Write Mode, only in standard SPI mode.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 * @param[in]  retry: Repeat times
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_xqspi_transmit(xqspi_handle_t *p_xqspi, uint8_t *p_data, uint32_t length, uint32_t retry);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in blocking mode.
 * @note   This function is used only in Indirect Read Mode, only in standard SPI mode.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be received in bytes
 * @param[in]  retry: Repeat times
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_xqspi_receive(xqspi_handle_t *p_xqspi, uint8_t *p_data, uint32_t length, uint32_t retry);

/** @} */

/** @defgroup XQSPI_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   XQSPI control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the XQSPI.
     (+) hal_xqspi_get_state()API can be helpful to check in run-time the state of the XQSPI peripheral.
     (+) hal_xqspi_get_error() check in run-time Errors occurring during communication.
     (+) hal_xqspi_set_retry() set the repeat times during internal process.
     (+) hal_xqspi_set_tx_fifo_threshold() set the TX FIFO Threshold.
     (+) hal_xqspi_set_rx_fifo_threshold() set the RX FIFO Threshold.
     (+) hal_xqspi_get_tx_fifo_threshold() get the TX FIFO Threshold.
     (+) hal_xqspi_get_rx_fifo_threshold() get the RX FIFO Threshold.
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the XQSPI handle state.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @retval ::HAL_XQSPI_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_XQSPI_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_XQSPI_STATE_BUSY: Peripheral in indirect mode and busy.
 * @retval ::HAL_XQSPI_STATE_BUSY_INDIRECT_TX: Peripheral in indirect mode with transmission ongoing.
 * @retval ::HAL_XQSPI_STATE_BUSY_INDIRECT_RX: Peripheral in indirect mode with reception ongoing.
 * @retval ::HAL_XQSPI_STATE_ABORT: Peripheral with abort request ongoing.
 * @retval ::HAL_XQSPI_STATE_ERROR: Peripheral in error.
 ****************************************************************************************
 */
hal_xqspi_state_t hal_xqspi_get_state(xqspi_handle_t *p_xqspi);

/**
 ****************************************************************************************
 * @brief  Return the XQSPI error code.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @return XQSPI error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_xqspi_get_error(xqspi_handle_t *p_xqspi);

/**
 ****************************************************************************************
 * @brief  Set the XQSPI internal process repeat times value.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @param[in]  retry: Internal process repeat times value.
 ****************************************************************************************
 */
void hal_xqspi_set_retry(xqspi_handle_t *p_xqspi, uint32_t retry);

/**
 ****************************************************************************************
 * @brief  Set the TXFIFO threshold.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @param[in]  threshold: TX FIFO threshold can be one of the following values:
 *         @arg @ref XQSPI_FIFO_THRESHOLD_1_8      threshold is 8 bytes
 *         @arg @ref XQSPI_FIFO_THRESHOLD_1_4      threshold is 16 bytes
 *         @arg @ref XQSPI_FIFO_THRESHOLD_1_2      threshold is 32 bytes
 *         @arg @ref XQSPI_FIFO_THRESHOLD_3_4      threshold is 48 bytes
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_xqspi_set_tx_fifo_threshold(xqspi_handle_t *p_xqspi, uint32_t threshold);

/**
 ****************************************************************************************
 * @brief  Set the RXFIFO threshold.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @param[in]  threshold: RX FIFO threshold can be one of the following values:
 *         @arg @ref XQSPI_FIFO_THRESHOLD_1_8      threshold is 8 bytes
 *         @arg @ref XQSPI_FIFO_THRESHOLD_1_4      threshold is 16 bytes
 *         @arg @ref XQSPI_FIFO_THRESHOLD_1_2      threshold is 32 bytes
 *         @arg @ref XQSPI_FIFO_THRESHOLD_3_4      threshold is 48 bytes
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_xqspi_set_rx_fifo_threshold(xqspi_handle_t *p_xqspi, uint32_t threshold);

/**
 ****************************************************************************************
 * @brief  Get the TXFIFO threshold.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @return TX FIFO threshold
 ****************************************************************************************
 */
uint32_t hal_xqspi_get_tx_fifo_threshold(xqspi_handle_t *p_xqspi);

/**
 ****************************************************************************************
 * @brief  Get the RXFIFO threshold.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @return RX FIFO threshold
 ****************************************************************************************
 */
uint32_t hal_xqspi_get_rx_fifo_threshold(xqspi_handle_t *p_xqspi);

/**
 ****************************************************************************************
 * @brief  Turn on/off present module, only in XIP mode.
 * @param[in]  p_xqspi: Pointer to an XQSPI handle which contains the configuration information for the specified XQSPI module.
 * @param[in]  status: Presen status can be one of the following values:
 *         @arg @ref XQSPI_DISABLE_PRESENT      Disable Present
 *         @arg @ref XQSPI_ENABLE_PRESENT       Enable Present

 ****************************************************************************************
 */
void hal_xqspi_set_xip_present_status(xqspi_handle_t *p_xqspi, uint32_t status);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_XQSPI_H__ */

/** @} */

/** @} */

/** @} */

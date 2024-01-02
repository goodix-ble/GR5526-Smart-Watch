/**
 ****************************************************************************************
 *
 * @file gr55xx_hal_qspi.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of QSPI HAL library.
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

/** @defgroup HAL_QSPI QSPI
  * @brief QSPI HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_QSPI_H__
#define __GR55xx_HAL_QSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "gr55xx_ll_qspi.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_QSPI_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_QSPI_state HAL QSPI state
  * @{
  */

/**
  * @brief HAL QSPI State Enumerations definition
  */
typedef enum
{
    HAL_QSPI_STATE_RESET             = 0x00,    /**< Peripheral not initialized                            */
    HAL_QSPI_STATE_READY             = 0x01,    /**< Peripheral initialized and ready for use              */
    HAL_QSPI_STATE_BUSY              = 0x02,    /**< Peripheral in indirect mode and busy                  */
    HAL_QSPI_STATE_BUSY_INDIRECT_TX  = 0x12,    /**< Peripheral in indirect mode with transmission ongoing */
    HAL_QSPI_STATE_BUSY_INDIRECT_RX  = 0x22,    /**< Peripheral in indirect mode with reception ongoing    */
    HAL_QSPI_STATE_BUSY_MEM_MAPPED   = 0x82U,   /*!< Peripheral in memory mapped mode ongoing              */
    HAL_QSPI_STATE_ABORT             = 0x08,    /**< Peripheral with abort request ongoing                 */
    HAL_QSPI_STATE_ERROR             = 0x04     /**< Peripheral in error                                   */

} hal_qspi_state_t;

/**
  *@brief  HAL Status structures Of memorymapped definition
  */
typedef enum
{
    HAL_MMAPPED_STATE_DEACTIVED         = 0x00U,    /**< Deactived. */
    HAL_MMAPPED_STATE_ACTIVED           = 0x01U,    /**< Actived.   */
    HAL_MMAPPED_STATE_ERROR             = 0xFFU,    /**< Actived.   */
} hal_memorymapped_status_t;

/**
  *@brief KEY index enum for memorymapped mode, use to modify any parameter quickly
  */
typedef enum _qspi_memorymapped_idx_e  {
    QSPI_MMAPED_IDX_DFS_HARDCCODE_EN        = 0x00,     /**< paired value: QSPI_CONCURRENT_XIP_DFS_HARDCODE_ENABLE/QSPI_CONCURRENT_XIP_DFS_HARDCODE_DISABLE */
    QSPI_MMAPED_IDX_DFS                     = 0x01,     /**< paired value: @ref QSPI_CONCURRENT_XIP_DFS*/
    QSPI_MMAPED_IDX_INST_EN                 = 0x02,     /**< paired value: QSPI_CONCURRENT_XIP_INST_ENABLE/QSPI_CONCURRENT_XIP_INST_DISABLE */
    QSPI_MMAPED_IDX_INST_SIZE               = 0x03,     /**< paired value: @ref QSPI_CONCURRENT_XIP_INSTSIZE */
    QSPI_MMAPED_IDX_INST_VAL                = 0x04,     /**< paired value: instruction data */
    QSPI_MMAPED_IDX_ADDR_SIZE               = 0x05,     /**< paired value: @ref QSPI_CONCURRENT_XIP_ADDRSIZE */
    QSPI_MMAPED_IDX_INST_ADDR_XFER_FORMAT   = 0x06,     /**< paired value: @ref QSPI_CONCURRENT_XIP_INST_ADDR_TRANSFER_FORMAT */
    QSPI_MMAPED_IDX_MODE_BITS_EN            = 0x07,     /**< paired value: QSPI_CONCURRENT_XIP_MODE_BITS_ENABLE / QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE */
    QSPI_MMAPED_IDX_MODE_BITS_SIZE          = 0x08,     /**< paired value: @ref QSPI_CONCURRENT_XIP_MBL */
    QSPI_MMAPED_IDX_MODE_BITS_VAL           = 0x09,     /**< paired value: mode bits data */
    QSPI_MMAPED_IDX_WAIT_CYCLES             = 0x0A,     /**< paired value: wait cycles, 0 ~ 31 */
    QSPI_MMAPED_IDX_DATA_FRF                = 0x0B,     /**< paired value: @ref QSPI_CONCURRENT_XIP_FRF */
    QSPI_MMAPED_IDX_PREFETCH_EN             = 0x0C,     /**< paired value: QSPI_CONCURRENT_XIP_PREFETCH_ENABLE / QSPI_CONCURRENT_XIP_PREFETCH_DISABLE  */
    QSPI_MMAPED_IDX_CONT_XFER_EN            = 0x0D,     /**< paired value: QSPI_CONCURRENT_XIP_CONT_XFER_ENABLE / QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE*/
    QSPI_MMAPED_IDX_CONT_XFER_TOC           = 0x0E,     /**< paired value: timeout count for the continuous transfer, 0 ~ 255 */
    QSPI_MMAPED_IDX_EDIAN_MODE              = 0x0F,     /**< paired value: @ref QSPI_CONCURRENT_XIP_ENDIAN_MODE */
} qspi_memorymapped_idx_e;

/** @} */

/** @} */

/** @addtogroup HAL_QSPI_STRUCTURES Structures
  * @{
  */

/** @defgroup QSPI_Configuration QSPI Configuration
  * @{
  */

/**
  * @brief QSPI init Structure definition.
  */
typedef struct _qspi_init_t
{
    uint32_t clock_prescaler;   /**< Specifies the prescaler factor for generating clock based on the AHB clock.
                                     This parameter can be a number between 0 and 0xFFFF. */

    uint32_t clock_mode;        /**< Specifies the Clock Mode. It indicates the level that clock takes between commands.
                                     This parameter can be a value of @ref QSPI_Clock_Mode. */

    uint32_t rx_sample_delay;   /**< Specifies the RX sample delay. It is used to delay the sample of the RX input port.
                                     This parameter can be a number between 0 and 0x7. */
} qspi_init_t;
/** @} */

/** @defgroup QSPI_handle QSPI handle
  * @{
  */

/**
  * @brief QSPI handle Structure definition
  */
typedef struct _qspi_handle
{
    qspi_regs_t           *p_instance;               /**< QSPI registers base address.        */

    qspi_init_t           init;                      /**< QSPI communication parameters.      */

    uint8_t               *p_tx_buffer;              /**< Pointer to QSPI Tx transfer Buffer. */

    __IO uint32_t         tx_xfer_size;              /**< QSPI Tx Transfer size.              */

    __IO uint32_t         tx_xfer_count;             /**< QSPI Tx Transfer Counter           */

    uint8_t               *p_rx_buffer;              /**< Pointer to QSPI Rx transfer Buffer */

    __IO uint32_t         rx_xfer_size;              /**< QSPI Rx Transfer size              */

    __IO uint32_t         rx_xfer_count;             /**< QSPI Rx Transfer Counter           */

    void (*write_fifo)(struct _qspi_handle *p_qspi); /**< Pointer to QSPI Tx transfer FIFO write function */

    void (*read_fifo)(struct _qspi_handle *p_qspi);  /**< Pointer to QSPI Rx transfer FIFO read function  */

    dma_handle_t          *p_dma;                    /**< QSPI Rx/Tx DMA Handle parameters   */

    __IO hal_lock_t       lock;                      /**< Locking object                     */

    __IO hal_qspi_state_t state;                     /**< QSPI communication state           */

    __IO uint32_t         error_code;                /**< QSPI Error code                    */

    uint32_t              timeout;                   /**< Timeout for the QSPI memory access */

    uint32_t              retention[22];              /**< Save important register information. */
} qspi_handle_t;
/** @} */

/** @defgroup QSPI_Command QSPI command
  * @{
  */

/**
  * @brief QSPI command Structure definition
  */
typedef struct _qspi_command_t
{
    uint32_t instruction;               /**< Specifies the Instruction to be sent.
                                             This parameter can be a value (8-bit) between 0x00 and 0xFF. */

    uint32_t address;                   /**< Specifies the Address to be sent (Size from 1 to 4 bytes according AddressSize).
                                             This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF. */

    uint32_t instruction_size;          /**< Specifies the Instruction Size.
                                             This parameter can be a value of @ref QSPI_Instruction_Size. */

    uint32_t address_size;              /**< Specifies the Address Size.
                                             This parameter can be a value of @ref QSPI_Address_Size. */

    uint32_t dummy_cycles;              /**< Specifies the Number of Dummy Cycles.
                                             This parameter can be a number between 0 and 31. */

    uint32_t data_size;                /**< Specifies the QSPI address width.
                                             This parameter can be a value of @ref QSPI_Data_Size. */

    uint32_t instruction_address_mode;  /**< Specifies the Instruction and Address Mode.
                                             This parameter can be a value of @ref QSPI_Inst_Addr_Mode. */

    uint32_t data_mode;                 /**< Specifies the Data Mode (used for dummy cycles and data phases).
                                             This parameter can be a value of @ref QSPI_Data_Mode. */

    uint32_t length;                    /**< Specifies the number of data to transfer. (This is the number of bytes).
                                             This parameter can be any value between 0 and 0xFFFFFFFF (0 means undefined length
                                             until end of memory).
                                             when using in DMA LLP xfer, length means total length of all linkedlist block in bytes */

    uint32_t clock_stretch_en;          /**< Specifies whether to enable the clock stretch feature
                                             This parameter can be LL_QSPI_CLK_STRETCH_ENABLE or LL_QSPI_CLK_STRETCH_DISABLE */
} qspi_command_t;
/** @} */

/** @defgroup QSPI_Memory_Map  QSPI memory map
  * @{
  */
/**
  * @brief QSPI memory map Structure definition
  */
typedef struct _qspi_memorymapped_t
{

    uint32_t x_sioo_mode;                 /**< Specifies instruction sent mode in memorymapped(xip) mode.
                                               This parameter can be a value of @ref QSPI_CONCURRENT_XIP_INST_SENT_MODE. */

    uint32_t x_instruction_en;            /**< Specifies whether to enable the instruction phase feature in memorymapped(xip) mode.
                                               This parameter can be a value of @ref QSPI_CONCURRENT_XIP_INST_PHASE. */

    uint32_t x_instruction_size;          /**< Specifies instruction size in memorymapped(xip) mode.
                                               This parameter can be a value of @ref QSPI_CONCURRENT_XIP_INSTSIZE.*/

    uint32_t x_instruction;               /**< Specifies instruction in memorymapped(xip) mode. */


    uint32_t x_address_size;              /**< Specifies instruction size in memorymapped(xip) mode.
                                               This parameter can be a value of @ref QSPI_CONCURRENT_XIP_ADDRSIZE. */

    uint32_t x_inst_addr_transfer_format; /**< Specifies xfer format of inst & addr in memorymapped(xip) mode.
                                               This parameter can be a value of @ref QSPI_CONCURRENT_XIP_INST_ADDR_TRANSFER_FORMAT. */

    uint32_t x_mode_bits_en;              /**< Specifies whether to enable mode bits phase in memorymapped(xip) mode.
                                               This parameter can be a value of @ref QSPI_CONCURRENT_XIP_MODE_BITS_PHASE. */

    uint32_t x_mode_bits_length;          /**< Specifies mode bits length.
                                               This parameter can be a value of @ref QSPI_CONCURRENT_XIP_MBL. */

    uint32_t x_mode_bits_data;            /**< Specifies value of mode bits phase */


    uint32_t x_dummy_cycles;              /**< Specifies wait(dummy) cycles in memorymapped(xip) mode.
                                               value range [0 ~ 31]. */

    uint32_t x_data_frame_format;         /**< Specifies enhanced spi's frame format in memorymapped(xip) mode.
                                               This parameter can be a value of @ref QSPI_CONCURRENT_XIP_FRF. */

    uint32_t x_prefetch_en;               /**< Specifies whether to enable the prefetch feature.
                                             This parameter can be a value of @ref QSPI_CONCURRENT_XIP_PREFETCH. */

    uint32_t x_continous_xfer_en;         /**< Specifies whether to enable the continuous transfer feature in memorymapped(xip) mode.
                                             This parameter can be a value of @ref QSPI_CONCURRENT_XIP_CONT_XFER. */

    uint32_t x_continous_xfer_toc;        /**< Specifies timeout count for the continuous transfer feature in memorymapped(xip) mode.
                                             unit in terms of hclk, range [0, 255]. */

    uint32_t x_endian_mode;              /**< Specifies endian mode in memorymapped(xip) mode.
                                             This parameter can be a value of @ref QSPI_CONCURRENT_XIP_ENDIAN_MODE. */

} qspi_memorymapped_t;
/** @} */

/** @defgroup QSPI_Memory_Map_Write   QSPI memory map write
  * @{
  */
/**
  * @brief QSPI memory map write Structure definition
  */
typedef struct _qspi_memorymapped_write_t
{

    uint32_t x_instruction_size;        /**< Specifies instruction size in memorymapped(xip) mode.
                                             This parameter can be a value of @ref QSPI_CONCURRENT_XIP_INSTSIZE. */

    uint32_t x_instruction;             /**< Specifies instruction in memorymapped(xip) mode. */


    uint32_t x_address_size;            /**< Specifies instruction size in memorymapped(xip) mode.
                                             This parameter can be a value of @ref QSPI_CONCURRENT_XIP_ADDRSIZE. */

    uint32_t x_inst_addr_transfer_format;   /**< Specifies xfer format of inst & addr in memorymapped(xip) mode.
                                                This parameter can be a value of @ref QSPI_CONCURRENT_XIP_INST_ADDR_TRANSFER_FORMAT. */

    uint32_t x_dummy_cycles;            /**< Specifies wait(dummy) cycles in memorymapped(xip) mode.
                                             value range [0 ~ 31]. */

    uint32_t x_data_frame_format;       /**< Specifies enhanced spi's frame format in memorymapped(xip) mode.
                                             This parameter can be a value of @ref QSPI_CONCURRENT_XIP_FRF. */
} qspi_memorymapped_write_t;
/** @} */

/** @defgroup QSPI_Memorymapped Parameter   QSPI memorymapped parameter
  * @{
  */
/**
  *@brief KEY:Value pair to set memorymapped parameter.
  */
typedef struct _qspi_memorymapped_set_t {
    qspi_memorymapped_idx_e mmap_key;   /**< Memorymapped key. */
    uint32_t                mmap_val;   /**< Memorymapped value. */
} qspi_memorymapped_set_t;
/** @} */

/** @defgroup QSPI_Psram_Write_Command   QSPI psram-write command
  * @{
  */
/**
  * @brief QSPI command for psram-write Structure definition.
  */
typedef struct {

    uint32_t data_size;                 /**< Specifies the QSPI data width.
                                            if llp_data_mode is QSPI_PSRAM_LINKED_BLOCK_DATA_MODE_0 : data_size support 8/16/32 bits;
                                            if llp_data_mode is QSPI_PSRAM_LINKED_BLOCK_DATA_MODE_1 : data_size support 8/16 bits.
                                            This parameter can be a value of @ref QSPI_Data_Size. */

    uint32_t data_block_length;         /**< Specifies the number of data in each block, This is the number of bytes.
                                             if llp_data_mode is QSPI_PSRAM_LINKED_BLOCK_DATA_MODE_0 : the length equals each block's length(including the inst & addr length);
                                             if llp_data_mode is QSPI_PSRAM_LINKED_BLOCK_DATA_MODE_1 : the length equals the result of one inst&addr block plus one data block.
                                             This parameter can be any value between 0 and 4095. */

    uint32_t llp_data_mode;             /**< Specifies Linked list data mode. @ref QSPI_PSRAM_DATA_IN_LLP_MODE. */

    uint32_t llp_data_shape;            /**< Specifies Linked list data shape.
                                             Just support @ref QSPI_PSRAM_LINKED_BLOCK_DATA_SHAPE_RECTANGLE now. */

} qspi_psram_command_t;
/** @} */

/** @} */


/** @addtogroup HAL_QSPI_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_QSPI_Callback Callback
  * @{
  */

/**
  * @brief HAL_QSPI Callback function definition
  */

typedef struct _qspi_callback
{
    void (*qspi_msp_init)(qspi_handle_t *p_qspi);                   /**< QSPI init MSP callback                 */
    void (*qspi_msp_deinit)(qspi_handle_t *p_qspi);                 /**< QSPI de-init MSP callback              */
    void (*qspi_error_callback)(qspi_handle_t *p_qspi);             /**< QSPI error callback                    */
    void (*qspi_abort_cplt_callback)(qspi_handle_t *p_qspi);        /**< QSPI abort complete callback           */
    void (*qspi_rx_cplt_callback)(qspi_handle_t *p_qspi);           /**< QSPI rx transfer completed callback    */
    void (*qspi_tx_cplt_callback)(qspi_handle_t *p_qspi);           /**< QSPI tx transfer completed callback    */
} qspi_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_QSPI_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup QSPI_Exported_Constants QSPI Exported Constants
  * @{
  */

/** @defgroup QSPI_DMA_CTRL_REGISTER_CFG Set DMA Control Register
  * @{
  */
/**@brief Set DMA Control Register.
  *@param src_direction : DMA_SRC_INCREMENT, DMA_SRC_DECREMENT, DMA_SRC_NO_CHANGE
  *@param src_tr_width  : DMA_SDATAALIGN_BYTE, DMA_SDATAALIGN_HALFWORD, DMA_SDATAALIGN_WORD
  *@param dst_tr_width  : DMA_DDATAALIGN_BYTE, DMA_DDATAALIGN_HALFWORD, DMA_DDATAALIGN_WORD
  *@param src_msize     : DMA_CTLL_SRC_MSIZE_1/DMA_CTLL_SRC_MSIZE_4/DMA_CTLL_SRC_MSIZE_8/DMA_CTLL_SRC_MSIZE_16
  *@param dst_msize     : DMA_CTLL_DST_MSIZE_1/DMA_CTLL_DST_MSIZE_4/DMA_CTLL_DST_MSIZE_8/DMA_CTLL_DST_MSIZE_16
  *@param en_gather     : DMA_SRC_GATHER_ENABLE / DMA_SRC_GATHER_DISABLE
  */
#define QSPI_DMA_CRTL_LOW_REGISTER_CFG(src_direction, src_tr_width, dst_tr_width, src_msize, dst_msize, en_gather)      \
                              (                                                                                         \
                                DMA_CTLL_INI_EN | DMA_MEMORY_TO_PERIPH | DMA_LLP_SRC_ENABLE | DMA_LLP_DST_DISABLE |     \
                                en_gather | DMA_DST_SCATTER_DISABLE | DMA_DST_NO_CHANGE |                               \
                                src_direction | src_tr_width | dst_tr_width | src_msize | dst_msize                     \
                              )
/** @} */


/** @defgroup HAL_QSPI_FIFO_DEPTH Fifo Depth
  * @{
  */
#define QSPI_MAX_FIFO_DEPTH                 LL_QSPI_MAX_FIFO_DEPTH                   /**< Max FIFO Depth for QSPI Master. */
/** @} */

/** @defgroup QSPI_Error_Code QSPI Error Code
  * @{
  */
#define HAL_QSPI_ERROR_NONE             ((uint32_t)0x00000000) /**< No error.                 */
#define HAL_QSPI_ERROR_TIMEOUT          ((uint32_t)0x00000001) /**< Timeout error            */
#define HAL_QSPI_ERROR_TRANSFER         ((uint32_t)0x00000002) /**< Transfer error           */
#define HAL_QSPI_ERROR_DMA              ((uint32_t)0x00000004) /**< DMA transfer error       */
#define HAL_QSPI_ERROR_INVALID_PARAM    ((uint32_t)0x00000008) /**< Invalid parameter error  */
/** @} */

/** @defgroup QSPI_Clock_Mode QSPI Clock Mode
  * @{
  */

#define QSPI_CLOCK_MODE_0               (LL_QSPI_SCPOL_LOW | LL_QSPI_SCPHA_1EDGE)   /**< Inactive state of CLK is low; CLK toggles at the start of the first data bit   */
#define QSPI_CLOCK_MODE_1               (LL_QSPI_SCPOL_LOW | LL_QSPI_SCPHA_2EDGE)   /**< Inactive state of CLK is low; CLK toggles in the middle of the first data bit  */
#define QSPI_CLOCK_MODE_2               (LL_QSPI_SCPOL_HIGH | LL_QSPI_SCPHA_1EDGE)  /**< Inactive state of CLK is high; CLK toggles at the start of the first data bit   */
#define QSPI_CLOCK_MODE_3               (LL_QSPI_SCPOL_HIGH | LL_QSPI_SCPHA_2EDGE)  /**< Inactive state of CLK is high; CLK toggles in the middle of the first data bit  */

/** @} */

/** @defgroup QSPI_Data_Mode QSPI Data Mode
  * @{
  */

#define QSPI_DATA_MODE_SPI              LL_QSPI_FRF_SPI          /**< Standard SPI Frame Format  */
#define QSPI_DATA_MODE_DUALSPI          LL_QSPI_FRF_DUALSPI      /**< Dual SPI Frame Format      */
#define QSPI_DATA_MODE_QUADSPI          LL_QSPI_FRF_QUADSPI      /**< Quad SPI Frame Format      */

/** @} */

/** @defgroup QSPI_Instruction_Size QSPI Instruction Size
  * @{
  */

#define QSPI_INSTSIZE_00_BITS           LL_QSPI_INSTSIZE_0BIT    /**< 0-bit (No Instruction) */
#define QSPI_INSTSIZE_04_BITS           LL_QSPI_INSTSIZE_4BIT    /**< 4-bit Instruction      */
#define QSPI_INSTSIZE_08_BITS           LL_QSPI_INSTSIZE_8BIT    /**< 8-bit Instruction      */
#define QSPI_INSTSIZE_16_BITS           LL_QSPI_INSTSIZE_16BIT   /**< 16-bit Instruction     */

/** @} */

/** @defgroup QSPI_Address_Size QSPI Address Size
  * @{
  */

#define QSPI_ADDRSIZE_00_BITS           LL_QSPI_ADDRSIZE_0BIT    /**< 0-bit  address */
#define QSPI_ADDRSIZE_04_BITS           LL_QSPI_ADDRSIZE_4BIT    /**< 4-bit  address */
#define QSPI_ADDRSIZE_08_BITS           LL_QSPI_ADDRSIZE_8BIT    /**< 8-bit  address */
#define QSPI_ADDRSIZE_12_BITS           LL_QSPI_ADDRSIZE_12BIT   /**< 12-bit address */
#define QSPI_ADDRSIZE_16_BITS           LL_QSPI_ADDRSIZE_16BIT   /**< 16-bit address */
#define QSPI_ADDRSIZE_20_BITS           LL_QSPI_ADDRSIZE_20BIT   /**< 20-bit address */
#define QSPI_ADDRSIZE_24_BITS           LL_QSPI_ADDRSIZE_24BIT   /**< 24-bit address */
#define QSPI_ADDRSIZE_28_BITS           LL_QSPI_ADDRSIZE_28BIT   /**< 28-bit address */
#define QSPI_ADDRSIZE_32_BITS           LL_QSPI_ADDRSIZE_32BIT   /**< 32-bit address */

/** @} */

/** @defgroup QSPI_Data_Size Data Width
  * @{
  */

#define QSPI_DATASIZE_04_BITS           LL_QSPI_DATASIZE_4BIT    /**< Data length for SPI transfer:  4 bits */
#define QSPI_DATASIZE_05_BITS           LL_QSPI_DATASIZE_5BIT    /**< Data length for SPI transfer:  5 bits */
#define QSPI_DATASIZE_06_BITS           LL_QSPI_DATASIZE_6BIT    /**< Data length for SPI transfer:  6 bits */
#define QSPI_DATASIZE_07_BITS           LL_QSPI_DATASIZE_7BIT    /**< Data length for SPI transfer:  7 bits */
#define QSPI_DATASIZE_08_BITS           LL_QSPI_DATASIZE_8BIT    /**< Data length for SPI transfer:  8 bits */
#define QSPI_DATASIZE_09_BITS           LL_QSPI_DATASIZE_9BIT    /**< Data length for SPI transfer:  9 bits */
#define QSPI_DATASIZE_10_BITS           LL_QSPI_DATASIZE_10BIT   /**< Data length for SPI transfer:  10 bits */
#define QSPI_DATASIZE_11_BITS           LL_QSPI_DATASIZE_11BIT   /**< Data length for SPI transfer:  11 bits */
#define QSPI_DATASIZE_12_BITS           LL_QSPI_DATASIZE_12BIT   /**< Data length for SPI transfer:  12 bits */
#define QSPI_DATASIZE_13_BITS           LL_QSPI_DATASIZE_13BIT   /**< Data length for SPI transfer:  13 bits */
#define QSPI_DATASIZE_14_BITS           LL_QSPI_DATASIZE_14BIT   /**< Data length for SPI transfer:  14 bits */
#define QSPI_DATASIZE_15_BITS           LL_QSPI_DATASIZE_15BIT   /**< Data length for SPI transfer:  15 bits */
#define QSPI_DATASIZE_16_BITS           LL_QSPI_DATASIZE_16BIT   /**< Data length for SPI transfer:  16 bits */
#define QSPI_DATASIZE_17_BITS           LL_QSPI_DATASIZE_17BIT   /**< Data length for SPI transfer:  17 bits */
#define QSPI_DATASIZE_18_BITS           LL_QSPI_DATASIZE_18BIT   /**< Data length for SPI transfer:  18 bits */
#define QSPI_DATASIZE_19_BITS           LL_QSPI_DATASIZE_19BIT   /**< Data length for SPI transfer:  19 bits */
#define QSPI_DATASIZE_20_BITS           LL_QSPI_DATASIZE_20BIT   /**< Data length for SPI transfer:  20 bits */
#define QSPI_DATASIZE_21_BITS           LL_QSPI_DATASIZE_21BIT   /**< Data length for SPI transfer:  21 bits */
#define QSPI_DATASIZE_22_BITS           LL_QSPI_DATASIZE_22BIT   /**< Data length for SPI transfer:  22 bits */
#define QSPI_DATASIZE_23_BITS           LL_QSPI_DATASIZE_23BIT   /**< Data length for SPI transfer:  23 bits */
#define QSPI_DATASIZE_24_BITS           LL_QSPI_DATASIZE_24BIT   /**< Data length for SPI transfer:  24 bits */
#define QSPI_DATASIZE_25_BITS           LL_QSPI_DATASIZE_25BIT   /**< Data length for SPI transfer:  25 bits */
#define QSPI_DATASIZE_26_BITS           LL_QSPI_DATASIZE_26BIT   /**< Data length for SPI transfer:  26 bits */
#define QSPI_DATASIZE_27_BITS           LL_QSPI_DATASIZE_27BIT   /**< Data length for SPI transfer:  27 bits */
#define QSPI_DATASIZE_28_BITS           LL_QSPI_DATASIZE_28BIT   /**< Data length for SPI transfer:  28 bits */
#define QSPI_DATASIZE_29_BITS           LL_QSPI_DATASIZE_29BIT   /**< Data length for SPI transfer:  29 bits */
#define QSPI_DATASIZE_30_BITS           LL_QSPI_DATASIZE_30BIT   /**< Data length for SPI transfer:  30 bits */
#define QSPI_DATASIZE_31_BITS           LL_QSPI_DATASIZE_31BIT   /**< Data length for SPI transfer:  31 bits */
#define QSPI_DATASIZE_32_BITS           LL_QSPI_DATASIZE_32BIT   /**< Data length for SPI transfer:  32 bits */


/** @} */


/** @defgroup QSPI_Inst_Addr_Mode QSPI Instruction and Address Mode
  * @{
  */

#define QSPI_INST_ADDR_ALL_IN_SPI       LL_QSPI_INST_ADDR_ALL_IN_SPI         /**< Instruction and address are sent in SPI mode */
#define QSPI_INST_IN_SPI_ADDR_IN_SPIFRF LL_QSPI_INST_IN_SPI_ADDR_IN_SPIFRF   /**< Instruction is sent in SPI mode, and address is sent in Daul/Quad SPI mode */
#define QSPI_INST_ADDR_ALL_IN_SPIFRF    LL_QSPI_INST_ADDR_ALL_IN_SPIFRF      /**< Instruction and address are sent in Daul/Quad SPI mode */

/** @} */

/** @defgroup QSPI_Flags QSPI Flags
  * @{
  */

#define QSPI_FLAG_DCOL                  LL_QSPI_SR_DCOL          /**< Data collision error flag  */
#define QSPI_FLAG_TXE                   LL_QSPI_SR_TXE           /**< Transmission error flag    */
#define QSPI_FLAG_RFF                   LL_QSPI_SR_RFF           /**< Rx FIFO full flag          */
#define QSPI_FLAG_RFNE                  LL_QSPI_SR_RFNE          /**< Rx FIFO not empty flag     */
#define QSPI_FLAG_TFE                   LL_QSPI_SR_TFE           /**< Tx FIFO empty flag         */
#define QSPI_FLAG_TFNF                  LL_QSPI_SR_TFNF          /**< Tx FIFO not full flag      */
#define QSPI_FLAG_BUSY                  LL_QSPI_SR_BUSY          /**< Busy flag                  */

/** @} */

/** @defgroup QSPI_Interrupts QSPI Interrupts
  * @{
  */

#define QSPI_IT_TXU                     LL_QSPI_IS_TXU           /**< Transmit FIFO Underflow Interrupt flag    */
#define QSPI_IT_XRXO                    LL_QSPI_IS_XRXO          /**< XIP Receive FIFO Overflow Interrupt flag. */
#define QSPI_IT_MST                     LL_QSPI_IS_MST           /**< Multi-Master Contention Interrupt flag.   */
#define QSPI_IT_RXF                     LL_QSPI_IS_RXF           /**< Receive FIFO Full Interrupt flag.         */
#define QSPI_IT_RXO                     LL_QSPI_IS_RXO           /**< Receive FIFO Overflow Interrupt flag.     */
#define QSPI_IT_RXU                     LL_QSPI_IS_RXU           /**< Receive FIFO Underflow Interrupt flag.    */
#define QSPI_IT_TXO                     LL_QSPI_IS_TXO           /**< Transmit FIFO Overflow Interrupt flag.    */
#define QSPI_IT_TXE                     LL_QSPI_IS_TXE           /**< Transmit FIFO Empty Interrupt flag.       */
#define QSPI_IT_ALL                     LL_QSPI_IS_ALL           /**< ALL QSPI Interrupts flag.                 */

/** @} */

/** @defgroup QSPI_Timeout_definition QSPI Timeout_definition
  * @{
  */
#define HAL_QSPI_TIMEOUT_DEFAULT_VALUE ((uint32_t)5000)         /**< 5s. */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_ENDIAN_MODE endian mode for qspi xip
  * @{
  */
#define QSPI_CONCURRENT_XIP_ENDIAN_MODE_0                LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE_0      /**< Default endian order from AHB. */
#define QSPI_CONCURRENT_XIP_ENDIAN_MODE_1                LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE_1      /**< Re-order the read data as [23:16], [31:24], [7:0], [15:8]. */
#define QSPI_CONCURRENT_XIP_ENDIAN_MODE_2                LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE_2      /**< Re-order the read data as [7:0], [15:8], [23:16], [31:24]. */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_SLAVE Which Slave to Enable in XIP
  * @{
  */
#define QSPI_CONCURRENT_XIP_SLAVE0                       LL_QSPI_CONCURRENT_XIP_SLAVE0  /**< Enable Slave0 in XIP. */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_DFS data frame size in xip, take effect when enable DFS_HC
  * @{
  */
#define QSPI_CONCURRENT_XIP_DFS_BYTE                     LL_QSPI_CONCURRENT_XIP_DFS_BYTE      /**< Set data frame size as byte. */
#define QSPI_CONCURRENT_XIP_DFS_HALFWORD                 LL_QSPI_CONCURRENT_XIP_DFS_HALFWORD  /**< Set data frame size as halfword. */
#define QSPI_CONCURRENT_XIP_DFS_WORD                     LL_QSPI_CONCURRENT_XIP_DFS_WORD      /**< Set data frame size as word. */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_MBL mode bits length for xip mode
  * @{
  */
#define QSPI_CONCURRENT_XIP_MBL_2                        LL_QSPI_CONCURRENT_XIP_MBL_2      /**< mode bits length equals to 2 bit. */
#define QSPI_CONCURRENT_XIP_MBL_4                        LL_QSPI_CONCURRENT_XIP_MBL_4      /**< mode bits length equals to 4 bit. */
#define QSPI_CONCURRENT_XIP_MBL_8                        LL_QSPI_CONCURRENT_XIP_MBL_8      /**< mode bits length equals to 8 bit. */
#define QSPI_CONCURRENT_XIP_MBL_16                       LL_QSPI_CONCURRENT_XIP_MBL_16     /**< mode bits length equals to 16 bit. */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_INSTSIZE instruction size for concurrent xip mode
  * @{
  */
#define QSPI_CONCURRENT_XIP_INSTSIZE_0BIT                LL_QSPI_CONCURRENT_XIP_INSTSIZE_0BIT      /**< no instruction */
#define QSPI_CONCURRENT_XIP_INSTSIZE_4BIT                LL_QSPI_CONCURRENT_XIP_INSTSIZE_4BIT      /**< instruction size equals 4bits  */
#define QSPI_CONCURRENT_XIP_INSTSIZE_8BIT                LL_QSPI_CONCURRENT_XIP_INSTSIZE_8BIT      /**< instruction size equals 8bits  */
#define QSPI_CONCURRENT_XIP_INSTSIZE_16BIT               LL_QSPI_CONCURRENT_XIP_INSTSIZE_16BIT     /**< instruction size equals 16bits */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_ADDRSIZE address size for concurrent xip mode
  * @{
  */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_0BIT                LL_QSPI_CONCURRENT_XIP_ADDRSIZE_0BIT      /**< Address length for QSPI XIP transfer:  0 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_4BIT                LL_QSPI_CONCURRENT_XIP_ADDRSIZE_4BIT      /**< Address length for QSPI XIP transfer:  4 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_8BIT                LL_QSPI_CONCURRENT_XIP_ADDRSIZE_8BIT      /**< Address length for QSPI XIP transfer:  8 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_12BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_12BIT     /**< Address length for QSPI XIP transfer: 12 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_16BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_16BIT     /**< Address length for QSPI XIP transfer: 16 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_20BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_20BIT     /**< Address length for QSPI XIP transfer: 20 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT     /**< Address length for QSPI XIP transfer: 24 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_28BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_28BIT     /**< Address length for QSPI XIP transfer: 28 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_32BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_32BIT     /**< Address length for QSPI XIP transfer: 32 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_36BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_36BIT     /**< Address length for QSPI XIP transfer: 36 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_40BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_40BIT     /**< Address length for QSPI XIP transfer: 40 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_44BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_44BIT     /**< Address length for QSPI XIP transfer: 44 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_48BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_48BIT     /**< Address length for QSPI XIP transfer: 48 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_52BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_52BIT     /**< Address length for QSPI XIP transfer: 52 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_56BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_56BIT     /**< Address length for QSPI XIP transfer: 56 bits */
#define QSPI_CONCURRENT_XIP_ADDRSIZE_60BIT               LL_QSPI_CONCURRENT_XIP_ADDRSIZE_60BIT     /**< Address length for QSPI XIP transfer: 60 bits */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_INST_ADDR_TRANSFER_FORMAT transfer of inst & address for concurrent xip mode
  * @{
  */
#define QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPI         LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPI           /**< Instruction and address are sent in SPI mode */
#define QSPI_CONCURRENT_XIP_INST_IN_SPI_ADDR_IN_SPIFRF   LL_QSPI_CONCURRENT_XIP_INST_IN_SPI_ADDR_IN_SPIFRF     /**< Instruction is in sent in SPI mode and address is sent in Daul/Quad SPI mode */
#define QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF      LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF        /**< Instruction and address are sent in Daul/Quad SPI mode */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_FRF frame format for concurrent xip mode
  * @{
  */
#define QSPI_CONCURRENT_XIP_FRF_RSVD                     LL_QSPI_CONCURRENT_XIP_FRF_RSVD          /**< SPI Frame format : Reserved. */
#define QSPI_CONCURRENT_XIP_FRF_DUAL_SPI                 LL_QSPI_CONCURRENT_XIP_FRF_DUAL_SPI      /**< SPI Frame format : DUAL. */
#define QSPI_CONCURRENT_XIP_FRF_QUAD_SPI                 LL_QSPI_CONCURRENT_XIP_FRF_QUAD_SPI      /**< SPI Frame format : QUAD. */
#define QSPI_CONCURRENT_XIP_FRF_OCTAL_SPI                LL_QSPI_CONCURRENT_XIP_FRF_OCTAL_SPI     /**< SPI Frame format : OCTAL. */
/** @} */

/** @defgroup QSPI_XIP_CLK_STRETCH Mode Clock stretch mode
  * @{
  */
#define QSPI_CLK_STRETCH_ENABLE                          LL_QSPI_CLK_STRETCH_ENABLE   /**< Enable Clock stretch. */
#define QSPI_CLK_STRETCH_DISABLE                         LL_QSPI_CLK_STRETCH_DISABLE  /**< Disable Clock stretch. */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_PREFETCH Prefetch for concurrent xip mode
  * @{
  */
#define QSPI_CONCURRENT_XIP_PREFETCH_ENABLE              LL_QSPI_CONCURRENT_XIP_PREFETCH_ENABLE   /**< Enable Prefetch. */
#define QSPI_CONCURRENT_XIP_PREFETCH_DISABLE             LL_QSPI_CONCURRENT_XIP_PREFETCH_DISABLE  /**< Disable Prefetch. */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_CONT_XFER Cont transfer for concurrent xip mode
  * @{
  */
#define QSPI_CONCURRENT_XIP_CONT_XFER_ENABLE             LL_QSPI_CONCURRENT_XIP_CONT_XFER_ENABLE   /**< Enable Cont trasfer. */
#define QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE            LL_QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE  /**< Disable Cont trasfer. */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_INST_PHASE Instruction phase for concurrent xip mode
  * @{
  */
#define QSPI_CONCURRENT_XIP_INST_ENABLE                  LL_QSPI_CONCURRENT_XIP_INST_ENABLE   /**< Enable Instruction phase. */
#define QSPI_CONCURRENT_XIP_INST_DISABLE                 LL_QSPI_CONCURRENT_XIP_INST_DISABLE  /**< Disable Instruction phase. */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_MODE_BITS_PHASE Bits phase for concurrent xip mode
  * @{
  */
#define QSPI_CONCURRENT_XIP_MODE_BITS_ENABLE             LL_QSPI_CONCURRENT_XIP_MODE_BITS_ENABLE   /**< Enable Bits phase. */
#define QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE            LL_QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE  /**< Disable Bits phase. */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_DFS_HARDCODE DFS hardcore for concurrent xip mode
  * @{
  */
#define QSPI_CONCURRENT_XIP_DFS_HARDCODE_ENABLE          LL_QSPI_CONCURRENT_XIP_DFS_HARDCODE_ENABLE   /**< Enable DFS Hardcode. */
#define QSPI_CONCURRENT_XIP_DFS_HARDCODE_DISABLE         LL_QSPI_CONCURRENT_XIP_DFS_HARDCODE_DISABLE  /**< Disable DFS Hardcode. */
/** @} */

/** @defgroup QSPI_CONCURRENT_XIP_INST_SENT_MODE Instruction sent mode for concurrent xip mode
  * @{
  */
#define QSPI_CONCURRENT_XIP_INST_SENT_EVERY_ACCESS       LL_QSPI_CONCURRENT_XIP_INST_SENT_EVERY_ACCESS          /*!< Send instruction for every transaction */
#define QSPI_CONCURRENT_XIP_INST_SENT_ONLY_FIRST_ACCESS  LL_QSPI_CONCURRENT_XIP_INST_SENT_ONLY_FIRST_ACCESS     /*!< Send instruction only for first transaction */
/** @} */


/** @defgroup QSPI_PSRAM_DATA_IN_LLP_MODE BLOCK Mode for LLP
  * @{
  */
#define QSPI_PSRAM_LINKED_BLOCK_DATA_MODE_0                     0u   /**< This is Loop Block Mode, inst & addr in the head of each block,
                                                                        data are following insta & addr in the same block. */
#define QSPI_PSRAM_LINKED_BLOCK_DATA_MODE_1                     1u   /**< This is Crossed Block Mode, inst & addr in the first block,
                                                                        data in the second block, then inst & addr block, then data block again,
                                                                        and repeat till the end. */
/** @} */


/** @defgroup QSPI_PSRAM_DATA_BLOCK_SHAPE BLOCK SHAPE for DMA LLP
  * @{
  */
#define QSPI_PSRAM_LINKED_BLOCK_DATA_SHAPE_RECTANGLE            0u  /**< Data length in every Block are the same */

#define QSPI_PSRAM_LINKED_BLOCK_DATA_SHAPE_NON_RECTANGLE        1u  /**< Data length in every Block are the different */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup QSPI_Exported_Macros QSPI Exported Macros
  * @{
  */

/** @brief  Reset QSPI handle states.
  * @param  __HANDLE__ QSPI handle.
  * @retval None
  */
#define __HAL_QSPI_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_QSPI_STATE_RESET)

/** @brief  Enable the specified QSPI peripheral.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @retval None
  */

#define __HAL_QSPI_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->QSPI_EN, QSPI_SSI_EN)

/** @brief  Disable the specified QSPI peripheral.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @retval None
  */

#define __HAL_QSPI_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->QSPI_EN, QSPI_SSI_EN)

/** @brief  Enable the QSPI DMA TX Request.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @retval None
  */

#define __HAL_QSPI_ENABLE_DMATX(__HANDLE__)                     SET_BITS((__HANDLE__)->p_instance->DMAC, QSPI_DMAC_TDMAE)

/** @brief  Enable the QSPI DMA RX Request.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @retval None
  */
#define __HAL_QSPI_ENABLE_DMARX(__HANDLE__)                     SET_BITS((__HANDLE__)->p_instance->DMAC, QSPI_DMAC_RDMAE)


/** @brief  Disable the QSPI DMA TX Request.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @retval None
  */
#define __HAL_QSPI_DISABLE_DMATX(__HANDLE__)                    CLEAR_BITS((__HANDLE__)->p_instance->DMAC, QSPI_DMAC_TDMAE)


/** @brief  Disable the QSPI DMA RX Request.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @retval None
  */
#define __HAL_QSPI_DISABLE_DMARX(__HANDLE__)                    CLEAR_BITS((__HANDLE__)->p_instance->DMAC, QSPI_DMAC_RDMAE)


/** @brief  Enable the specified QSPI interrupts.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @param  __INTERRUPT__ Specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg @ref QSPI_IT_MST Multi-Master Contention Interrupt enable
  *            @arg @ref QSPI_IT_RXF Receive FIFO Full Interrupt enable
  *            @arg @ref QSPI_IT_RXO Receive FIFO Overflow Interrupt enable
  *            @arg @ref QSPI_IT_RXU Receive FIFO Underflow Interrupt enable
  *            @arg @ref QSPI_IT_TXO Transmit FIFO Overflow Interrupt enable
  *            @arg @ref QSPI_IT_TXE Transmit FIFO Empty Interrupt enable
  * @retval None
  */
#define __HAL_QSPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)         SET_BITS((__HANDLE__)->p_instance->INTMASK, (__INTERRUPT__))


/** @brief  Disable the specified QSPI interrupts.
  * @param  __HANDLE__ Specifies the QSPI handle.
  * @param  __INTERRUPT__ Specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg @ref QSPI_IT_MST Multi-Master Contention Interrupt enable
  *            @arg @ref QSPI_IT_RXF Receive FIFO Full Interrupt enable
  *            @arg @ref QSPI_IT_RXO Receive FIFO Overflow Interrupt enable
  *            @arg @ref QSPI_IT_RXU Receive FIFO Underflow Interrupt enable
  *            @arg @ref QSPI_IT_TXO Transmit FIFO Overflow Interrupt enable
  *            @arg @ref QSPI_IT_TXE Transmit FIFO Empty Interrupt enable
  * @retval None
  */
#define __HAL_QSPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)        CLEAR_BITS((__HANDLE__)->p_instance->INTMASK, (__INTERRUPT__))


/** @brief  Check whether the specified QSPI interrupt source is enabled or not.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @param  __INTERRUPT__ Specifies the interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg @ref QSPI_IT_MST Multi-Master Contention Interrupt enable
  *            @arg @ref QSPI_IT_RXF Receive FIFO Full Interrupt enable
  *            @arg @ref QSPI_IT_RXO Receive FIFO Overflow Interrupt enable
  *            @arg @ref QSPI_IT_RXU Receive FIFO Underflow Interrupt enable
  *            @arg @ref QSPI_IT_TXO Transmit FIFO Overflow Interrupt enable
  *            @arg @ref QSPI_IT_TXE Transmit FIFO Empty Interrupt enable
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __HAL_QSPI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)     (READ_BITS((__HANDLE__)->p_instance->INTSTAT, (__INTERRUPT__)) == (__INTERRUPT__))


/** @brief  Check whether the specified QSPI flag is set or not.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @param  __FLAG__ Specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref QSPI_FLAG_DCOL Data collision error flag
  *            @arg @ref QSPI_FLAG_TXE  Transmission error flag
  *            @arg @ref QSPI_FLAG_RFF  Rx FIFO full flag
  *            @arg @ref QSPI_FLAG_RFNE Rx FIFO not empty flag
  *            @arg @ref QSPI_FLAG_TFE  Tx FIFO empty flag
  *            @arg @ref QSPI_FLAG_TFNF Tx FIFO not full flag
  *            @arg @ref QSPI_FLAG_BUSY Busy flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_QSPI_GET_FLAG(__HANDLE__, __FLAG__)               ((READ_BITS((__HANDLE__)->p_instance->STAT, (__FLAG__)) != 0) ? SET : RESET)


/** @brief  Clear the specified QSPI flag.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @param  __FLAG__ Specifies the flag to clear.
  *         This parameter can be one of the following values:
  *            @arg @ref QSPI_FLAG_DCOL Data collision error flag
  *            @arg @ref QSPI_FLAG_TXE  Transmission error flag
  *            @arg @ref QSPI_FLAG_RFF  Rx FIFO full flag
  *            @arg @ref QSPI_FLAG_RFNE Rx FIFO not empty flag
  *            @arg @ref QSPI_FLAG_TFE  Tx FIFO empty flag
  *            @arg @ref QSPI_FLAG_TFNF Tx FIFO not full flag
  *            @arg @ref QSPI_FLAG_BUSY Busy flag
  * @retval None
  */
#define __HAL_QSPI_CLEAR_FLAG(__HANDLE__, __FLAG__)             READ_BITS((__HANDLE__)->p_instance->STAT, (__FLAG__))

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup QSPI_Private_Macro QSPI Private Macros
  * @{
  */

/** @brief  Check if QSPI Clock Prescaler is valid.
  * @param  __PRESCALER__ QSPI Clock Prescaler.
  * @retval SET (__PRESCALER__ is valid) or RESET (__PRESCALER__ is invalid)
  */
#define IS_QSPI_CLOCK_PRESCALER(__PRESCALER__)  ((__PRESCALER__) <= 0xFFFF)


/** @brief  Check if QSPI FIFO Threshold is valid.
  * @param  __THR__ QSPI FIFO Threshold.
  * @retval SET (__THR__ is valid) or RESET (__THR__ is invalid)
  */
    #define IS_QSPI_FIFO_THRESHOLD(__THR__)         (((__THR__) >= 0) && ((__THR__) <= (QSPI_MAX_FIFO_DEPTH - 1)))

/** @brief  Check if QSPI Clock Mode is valid.
  * @param  __CLKMODE__ QSPI Clock Mode.
  * @retval SET (__CLKMODE__ is valid) or RESET (__CLKMODE__ is invalid)
  */
#define IS_QSPI_CLOCK_MODE(__CLKMODE__)         (((__CLKMODE__) == QSPI_CLOCK_MODE_0) || \
                                                 ((__CLKMODE__) == QSPI_CLOCK_MODE_1) || \
                                                 ((__CLKMODE__) == QSPI_CLOCK_MODE_2) || \
                                                 ((__CLKMODE__) == QSPI_CLOCK_MODE_3))

/** @brief  Check if QSPI RX Sample Delay Value is valid.
  * @param  __DLY__ QSPI RX Sample Delay value
  * @retval SET (__DLY__ is valid) or RESET (__DLY__ is invalid)
  */
#define IS_QSPI_RX_SAMPLE_DLY(__DLY__)          (((__DLY__) >= 0) && ((__DLY__) <= 7))


/** @brief  Check if QSPI Instruction Size is valid.
  * @param  __INST_SIZE__ QSPI Instruction Size.
  * @retval SET (__INST_SIZE__ is valid) or RESET (__INST_SIZE__ is invalid)
  */
#define IS_QSPI_INSTRUCTION_SIZE(__INST_SIZE__) (((__INST_SIZE__) == QSPI_INSTSIZE_00_BITS) || \
                                                 ((__INST_SIZE__) == QSPI_INSTSIZE_04_BITS) || \
                                                 ((__INST_SIZE__) == QSPI_INSTSIZE_08_BITS) || \
                                                 ((__INST_SIZE__) == QSPI_INSTSIZE_16_BITS))

/** @brief  Check if QSPI Address Size is valid.
  * @param  __ADDR_SIZE__ QSPI Address Size .
  * @retval SET (__ADDR_SIZE__ is valid) or RESET (__ADDR_SIZE__ is invalid)
  */
#define IS_QSPI_ADDRESS_SIZE(__ADDR_SIZE__)     (((__ADDR_SIZE__) == QSPI_ADDRSIZE_00_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_04_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_08_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_12_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_16_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_20_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_24_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_28_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_32_BITS))

/** @brief  Check if QSPI Dummy Cycle is valid.
  * @param  __DCY__ QSPI Dummy Cycle.
  * @retval SET (__DCY__ is valid) or RESET (__DCY__ is invalid)
  */
#define IS_QSPI_DUMMY_CYCLES(__DCY__)           ((__DCY__) <= 31)

/** @brief  Check if QSPI Instruction and Address Mode is valid.
  * @param  __MODE__ QSPI Instruction and Address Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_QSPI_INSTADDR_MODE(__MODE__)         (((__MODE__) == QSPI_INST_ADDR_ALL_IN_SPI)       || \
                                                 ((__MODE__) == QSPI_INST_IN_SPI_ADDR_IN_SPIFRF) || \
                                                 ((__MODE__) == QSPI_INST_ADDR_ALL_IN_SPIFRF))

/** @brief  Check if QSPI Data Mode is valid.
  * @param  __MODE__ QSPI Data Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_QSPI_DATA_MODE(__MODE__)             (((__MODE__) == QSPI_DATA_MODE_SPI)     || \
                                                 ((__MODE__) == QSPI_DATA_MODE_DUALSPI) || \
                                                 ((__MODE__) == QSPI_DATA_MODE_QUADSPI))


/** @brief  Check if QSPI Data Size is supported currently.
  * @param  __SIZE__ QSPI Data Size.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_QSPI_SUPPORTED_DATA_SIZE(__SIZE__)   (((__SIZE__) == QSPI_DATASIZE_08_BITS) || \
                                                 ((__SIZE__) == QSPI_DATASIZE_16_BITS) || \
                                                 ((__SIZE__) == QSPI_DATASIZE_32_BITS))

/** @} */



/** @defgroup QSPI_XIP_Private_Macro QSPI-XIP Private Macros
  * @{
  */

/** @brief  Check if QSPI.XIP SIOO Mode is valid.
  * @param  _SIOO_ QSPI.XIP Data Mode.
  * @retval SET (_SIOO_ is valid) or RESET (_SIOO_ is invalid)
  */
#define IS_QSPI_CONC_XIP_SIOO_MODE(_SIOO_)                        ( (QSPI_CONCURRENT_XIP_INST_SENT_EVERY_ACCESS      == (_SIOO_)) || \
                                                                    (QSPI_CONCURRENT_XIP_INST_SENT_ONLY_FIRST_ACCESS == (_SIOO_)) )

/** @brief  Check if QSPI.XIP DFS Value is valid.
  * @param  _DFS_ QSPI.XIP DFS Value
  * @retval SET (_DFS_ is valid) or RESET (_DFS_ is invalid)
  */
#define IS_QSPI_CONC_XIP_DFS(_DFS_)                                ( (QSPI_CONCURRENT_XIP_DFS_BYTE     == (_DFS_)) || \
                                                                     (QSPI_CONCURRENT_XIP_DFS_HALFWORD == (_DFS_)) || \
                                                                     (QSPI_CONCURRENT_XIP_DFS_WORD     == (_DFS_)) )

/** @brief  Check if QSPI.XIP DFS_HC Switch Value is valid.
  * @param  _HC_EN_ QSPI.XIP DFS Hardcode Switch.
  * @retval SET (_HC_EN_ is valid) or RESET (_HC_EN_ is invalid)
  */
#define IS_QSPI_CONC_XIP_DFS_HC_EN(_HC_EN_)                        ( (QSPI_CONCURRENT_XIP_DFS_HARDCODE_ENABLE  == (_HC_EN_)) || \
                                                                     (QSPI_CONCURRENT_XIP_DFS_HARDCODE_DISABLE == (_HC_EN_)) )

/** @brief  Check if QSPI.XIP inst Switch is valid.
  * @param  _INST_EN_ QSPI.XIP inst en/dis.
  * @retval SET (_INST_EN_ is valid) or RESET (_INST_EN_ is invalid)
  */
#define IS_QSPI_CONC_XIP_INST_EN(_INST_EN_)                        ( (QSPI_CONCURRENT_XIP_INST_ENABLE  == (_INST_EN_)) || \
                                                                     (QSPI_CONCURRENT_XIP_INST_DISABLE == (_INST_EN_)) )

/** @brief  Check if QSPI.XIP inst size is valid.
  * @param  _INST_SIZE_ QSPI.XIP inst size.
  * @retval SET (_INST_SIZE_ is valid) or RESET (_INST_SIZE_ is invalid)
  */
#define IS_QSPI_CONC_XIP_INST_SIZE(_INST_SIZE_)                    ( (QSPI_CONCURRENT_XIP_INSTSIZE_0BIT  == (_INST_SIZE_) ) || \
                                                                     (QSPI_CONCURRENT_XIP_INSTSIZE_4BIT  == (_INST_SIZE_) ) || \
                                                                     (QSPI_CONCURRENT_XIP_INSTSIZE_8BIT  == (_INST_SIZE_) ) || \
                                                                     (QSPI_CONCURRENT_XIP_INSTSIZE_16BIT == (_INST_SIZE_) ) )

/** @brief  Check if QSPI.XIP inst is valid.
  * @param  _INST_ QSPI.XIP inst.
  * @retval SET (_INST_ is valid) or RESET (_INST_ is invalid)
  */
#define IS_QSPI_CONC_XIP_INST(_INST_)                             ((_INST_) <= 0xFFFF )

/** @brief  Check if QSPI.XIP Address Size is valid.
  * @param  _ADDR_SIZE_ QSPI.XIP Address Size.
  * @retval SET (_ADDR_SIZE_ is valid) or RESET (_ADDR_SIZE_ is invalid)
  */
#define IS_QSPI_CONC_XIP_ADDR_SIZE(_ADDR_SIZE_)                   ( (QSPI_CONCURRENT_XIP_ADDRSIZE_0BIT  == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_4BIT  == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_8BIT  == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_12BIT == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_16BIT == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_20BIT == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_28BIT == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_32BIT == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_36BIT == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_40BIT == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_44BIT == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_48BIT == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_52BIT == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_56BIT == (_ADDR_SIZE_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_ADDRSIZE_60BIT == (_ADDR_SIZE_) ) )

/** @brief  Check if QSPI.XIP Addr Xfer format is valid.
  * @param  _FORMAT_ QSPI.XIP Addr Xfer format.
  * @retval SET (_FORMAT_ is valid) or RESET (_FORMAT_ is invalid)
  */
#define IS_QSPI_CONC_INST_ADDR_XFER_FORMAT(_FORMAT_)            ( (QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPI       == (_FORMAT_)) || \
                                                                  (QSPI_CONCURRENT_XIP_INST_IN_SPI_ADDR_IN_SPIFRF == (_FORMAT_)) || \
                                                                  (QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF    == (_FORMAT_)) )

/** @brief  Check if QSPI.XIP Mode bits Switch is valid.
  * @param  _MD_EN_ QSPI.XIP Mode bits Switch.
  * @retval SET (_MD_EN_ is valid) or RESET (_MD_EN_ is invalid)
  */
#define IS_QSPI_CONC_XIP_MODE_BITS_EN(_MD_EN_)                    ( (QSPI_CONCURRENT_XIP_MODE_BITS_ENABLE  == (_MD_EN_) ) || \
                                                                    (QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE == (_MD_EN_) ))

/** @brief  Check if QSPI.XIP Mode Bits size is valid.
  * @param  _MD_SIZE_ QSPI.XIP Mode Bits size.
  * @retval SET (_MD_SIZE_ is valid) or RESET (_MD_SIZE_ is invalid)
  */
#define IS_QSPI_CONC_XIP_MODE_BITS_SIZE(_MD_SIZE_)                ( (QSPI_CONCURRENT_XIP_MBL_2  == (_MD_SIZE_)) || \
                                                                    (QSPI_CONCURRENT_XIP_MBL_4  == (_MD_SIZE_)) || \
                                                                    (QSPI_CONCURRENT_XIP_MBL_8  == (_MD_SIZE_)) || \
                                                                    (QSPI_CONCURRENT_XIP_MBL_16 == (_MD_SIZE_)) )

/** @brief  Check if QSPI.XIP Mode Bits is valid.
  * @param  _MD_BITS_ QSPI.XIP Mode Bits.
  * @retval SET (_MD_BITS_ is valid) or RESET (_MD_BITS_ is invalid)
  */
#define IS_QSPI_CONC_XIP_MODE_BITS(_MD_BITS_)                    ( (_MD_BITS_) <= 0xFFFF)

/** @brief  Check if QSPI.XIP dummy cycles is valid.
  * @param  __DCY__ QSPI.XIPdummy cycles.
  * @retval SET (__DCY__ is valid) or RESET (__DCY__ is invalid)
  */
#define IS_QSPI_CONC_XIP_DUMMY_CYCLES(__DCY__)                   ( (__DCY__) <= 31)

/** @brief  Check if QSPI.XIP frame format is valid.
  * @param  _XIP_FRF_ QSPI.XIP frame format.
  * @retval SET (_XIP_FRF_ is valid) or RESET (_XIP_FRF_ is invalid)
  */
#define IS_QSPI_CONC_XIP_DATA_FRF(_XIP_FRF_)                    ( (QSPI_CONCURRENT_XIP_FRF_DUAL_SPI == (_XIP_FRF_)) || \
                                                                  (QSPI_CONCURRENT_XIP_FRF_QUAD_SPI == (_XIP_FRF_)) )

/** @brief  Check if QSPI.XIP prefetch switch is valid.
  * @param  _PREFETCH_EN_ QSPI.XIP prefetch switch.
  * @retval SET (_PREFETCH_EN_ is valid) or RESET (_PREFETCH_EN_ is invalid)
  */
#define IS_QSPI_CONC_XIP_PREFETCH_EN(_PREFETCH_EN_)                ( (QSPI_CONCURRENT_XIP_PREFETCH_ENABLE  == (_PREFETCH_EN_)) || \
                                                                     (QSPI_CONCURRENT_XIP_PREFETCH_DISABLE == (_PREFETCH_EN_)) )

/** @brief  Check if QSPI.XIP cont xfer switch is valid.
  * @param  _CONT_XFER_EN_ QSPI.XIP cont xfer switch.
  * @retval SET (_CONT_XFER_EN_ is valid) or RESET (_CONT_XFER_EN_ is invalid)
  */
#define IS_QSPI_CONC_XIP_CONT_XFER_EN(_CONT_XFER_EN_)            ( (QSPI_CONCURRENT_XIP_CONT_XFER_ENABLE  == (_CONT_XFER_EN_)) || \
                                                                   (QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE == (_CONT_XFER_EN_)) )

/** @brief  Check if QSPI.XIP timeout count of cont xfer is valid.
  * @param  _TOC_ QSPI.XIP timeout count of cont xfer.
  * @retval SET (_TOC_ is valid) or RESET (_TOC_ is invalid)
  */
#define IS_QSPI_CONC_XIP_CONT_XFER_TOC(_TOC_)                    ( (_TOC_) <= 0xFF)

/** @brief  Check if QSPI.XIP Data endian Mode is valid.
  * @param  _MODE_ QSPI.XIP Data endian Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_QSPI_CONC_XIP_ENDIAN_MODE(_MODE_)                    ( (QSPI_CONCURRENT_XIP_ENDIAN_MODE_0 == (_MODE_)) || \
                                                                  (QSPI_CONCURRENT_XIP_ENDIAN_MODE_1 == (_MODE_)) || \
                                                                  (QSPI_CONCURRENT_XIP_ENDIAN_MODE_2 == (_MODE_)) )
/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_QSPI_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup QSPI_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the QSPIx peripheral:

      (+) User must implement hal_qspi_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_qspi_init() to configure the selected device with
          the selected configuration:
        (++) Clock Prescaler
        (++) Clock Mode

      (+) Call the function hal_qspi_deinit() to restore the default configuration
          of the selected QSPIx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the QSPI according to the specified parameters
 *         in the qspi_init_t and initialize the associated handle.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_init(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  De-initialize the QSPI peripheral.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_deinit(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Initialize the QSPI MSP.
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_qspi_msp_init can be implemented in the user file.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_msp_init(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  De-initialize the QSPI MSP.
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_qspi_msp_deinit can be implemented in the user file.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_msp_deinit(qspi_handle_t *p_qspi);

/** @} */

/** @defgroup QSPI_Exported_Functions_Group2 QSPI operation functions
 *  @brief   Data transfers functions
 *
@verbatim
  ==============================================================================
                      ##### QSPI operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the QSPI
    data transfers.

    [..] The QSPI supports master and slave mode:

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts.
            or DMA, These APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated QSPI IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The hal_qspi_tx_cplt_callback(), hal_qspi_rx_cplt_callback() and hal_qspi_txrx_cplt_callback() user callbacks
            will be executed respectively at the end of the transmit or Receive process.
            The hal_qspi_error_callback() user callback will be executed when a communication error is detected

    (#) APIs provided for these 2 transfer modes (Blocking mode or Non blocking mode using either Interrupt or DMA)
        exist for 1 Line (simplex) and 2 Lines (full duplex) modes.

@endverbatim
  * @{
  */
/**
 ****************************************************************************************
  * @brief  Configure the Memory Mapped mode. Called after hal_qspi_init(...)
  * @note   This function is used only in Memory mapped Mode
  * @param[in]  p_qspi:      Pointer to a QSPI handle
  * @param[in]  mmap_cmd:    Structure that contains the memorymapped read command configuration information.
  * @param[in]  mmap_wr_cmd: Structure that contains the memorymapped write command configuration information.
  * @retval hal status
  ****************************************************************************************
  */
hal_status_t hal_qspi_memorymapped(qspi_handle_t *p_qspi, qspi_memorymapped_t * mmap_cmd, qspi_memorymapped_write_t * mmap_wr_cmd);

/**
  * @brief  Active the memory mapped mode from Ready state. users must make sure parameters of mmaped mode hava been set correctly
  * @param  p_qspi: QSPI handle
  * @param  is_sioo_mode (TRUE/FALSE)- specifies SIOO Mode, must match to Slave device's inst
  * @note   1) This function is used only in Memory mapped Mode
  *         2) User must config the registers correcly firstly
  *         3) It requires that slave device's inst supports the SIOO Mode
  * @retval hal status
  */
hal_status_t hal_qspi_memorymapped_active(qspi_handle_t *p_qspi, uint32_t is_sioo_mode);

/**
  * @brief  Deactive the memory mapped mode to Ready state
  *         it's recommended to use with hal_qspi_memorymapped_active to switch mode quickly
  * @param  p_qspi: QSPI handle
  * @note   This function is used only in Memory mapped Mode
  * @retval hal status
  */
hal_status_t hal_qspi_memorymapped_deactive(qspi_handle_t *p_qspi);

/**
  * @brief  Check whether the memory mapped mode is Actived
  * @param  p_qspi: QSPI handle
  * @note   This function is used only in Memory mapped Mode
  * @retval 1 - actived; 0 - not actived
  */
hal_memorymapped_status_t hal_qspi_memorymapped_is_actived(qspi_handle_t *p_qspi);


/**
  * @brief  Used to update memorymapped any parameter quickly
  * @param  p_qspi:   QSPI handle
  * @param  mmap_set: Pointer to k:v pair of qspi_memorymapped_set_t
  * @param  count:    Amount of mmap_set's element
  * @note   This function is used only in Memory mapped Mode
  * @retval hal status
  */
hal_status_t hal_qspi_memorymapped_update(qspi_handle_t *p_qspi, qspi_memorymapped_set_t * mmap_set, uint32_t count);
/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with the specified instruction and address in blocking mode.
 * @note   This function is used only in Indirect Write Mode. Dummy cycles in command will be ignored.
 * @param[in]  p_qspi:  Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd:   Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_data:  Pointer to data buffer
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_transmit(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction, address and dummy cycles in blocking mode.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_receive(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit only instruction in blocking mode.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in blocking mode with standard SPI.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  data_mode : @ref QSPI_DATA_MODE_SPI
 *                              QSPI_DATA_MODE_DUALSPI
 *                              QSPI_DATA_MODE_QUADSPI
 * @param[in]  data_size : @ref QSPI_DATASIZE_08_BITS
 *                              QSPI_DATASIZE_16_BITS
 *                              QSPI_DATASIZE_32_BITS
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  data_length: Amount of data to be sent in bytes
 *             1. if data_mode equals QSPI_DATA_MODE_SPI : length must >= 2 bytes
 *             2. if data_mode equals QSPI_DATA_MODE_DUALSPI : length must >= 4 bytes and be multiple of HALFWORD
 *             3. if data_mode equals QSPI_DATA_MODE_QUADSPI : length must >= 8 bytes and be multiple of WORD
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_transmit(qspi_handle_t *p_qspi, uint32_t data_mode, uint32_t data_size, uint8_t *p_data, uint32_t data_length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in blocking mode with standard SPI.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  data_mode : @ref QSPI_DATA_MODE_SPI
 *                              QSPI_DATA_MODE_DUALSPI
 *                              QSPI_DATA_MODE_QUADSPI
 * @param[in]  data_size : @ref QSPI_DATASIZE_08_BITS
 *                              QSPI_DATASIZE_16_BITS
 *                              QSPI_DATASIZE_32_BITS
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 *             1. if data_mode equals QSPI_DATA_MODE_SPI : length must >= 2 bytes
 *             2. if data_mode equals QSPI_DATA_MODE_DUALSPI : length must >= 4 bytes and be multiple of HALFWORD
 *             3. if data_mode equals QSPI_DATA_MODE_QUADSPI : length must >= 8 bytes and be multiple of WORD
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_receive(qspi_handle_t *p_qspi, uint32_t data_mode, uint32_t data_size, uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with the specified instruction and address in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Write Mode. Dummy cycles in command will be ignored.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_data: Pointer to data buffer
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_transmit_it(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction, address and dummy cycles in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_receive_it(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Transmit instruction in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_it(qspi_handle_t *p_qspi, qspi_command_t *p_cmd);

/**
 ****************************************************************************************
 * @brief Transmit an amount of data in non-blocking mode at standard SPI with Interrupt.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  data_mode : @ref QSPI_DATA_MODE_SPI
 *                              QSPI_DATA_MODE_DUALSPI
 *                              QSPI_DATA_MODE_QUADSPI
 * @param[in]  data_size : @ref QSPI_DATASIZE_08_BITS
 *                              QSPI_DATASIZE_16_BITS
 *                              QSPI_DATASIZE_32_BITS
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  data_length: Amount of data to be sent in bytes
 *             1. if data_mode equals QSPI_DATA_MODE_SPI : length must >= 2 bytes
 *             2. if data_mode equals QSPI_DATA_MODE_DUALSPI : length must >= 4 bytes and be multiple of HALFWORD
 *             3. if data_mode equals QSPI_DATA_MODE_QUADSPI : length must >= 8 bytes and be multiple of WORD
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_transmit_it(qspi_handle_t *p_qspi, uint32_t data_mode, uint32_t data_size, uint8_t *p_data, uint32_t data_length);

/**
 ****************************************************************************************
 * @brief Receive an amount of data in non-blocking mode at standard SPI with Interrupt.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  data_mode : @ref QSPI_DATA_MODE_SPI
 *                              QSPI_DATA_MODE_DUALSPI
 *                              QSPI_DATA_MODE_QUADSPI
 * @param[in]  data_size : @ref QSPI_DATASIZE_08_BITS
 *                              QSPI_DATASIZE_16_BITS
 *                              QSPI_DATASIZE_32_BITS
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 *             1. if data_mode equals QSPI_DATA_MODE_SPI : length must >= 2 bytes
 *             2. if data_mode equals QSPI_DATA_MODE_DUALSPI : length must >= 4 bytes and be multiple of HALFWORD
 *             3. if data_mode equals QSPI_DATA_MODE_QUADSPI : length must >= 8 bytes and be multiple of WORD
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_receive_it(qspi_handle_t *p_qspi, uint32_t data_mode, uint32_t data_size, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with the specified instruction and address in non-blocking mode with DMA .
 * @note   This function is used only in Indirect Write Mode. Dummy cycles in command will be ignored.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_data: Pointer to data buffer
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_transmit_dma(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction, address and dummy cycles in non-blocking mode with DMA .
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_receive_dma(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Transmit instruction in non-blocking mode with DMA.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_dma(qspi_handle_t *p_qspi, qspi_command_t *p_cmd);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode at standard SPI with DMA.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  data_mode : @ref QSPI_DATA_MODE_SPI
 *                              QSPI_DATA_MODE_DUALSPI
 *                              QSPI_DATA_MODE_QUADSPI
 * @param[in]  data_size : @ref QSPI_DATASIZE_08_BITS
 *                              QSPI_DATASIZE_16_BITS
 *                              QSPI_DATASIZE_32_BITS
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 *             1. if data_mode equals QSPI_DATA_MODE_SPI : length must >= 2 bytes
 *             2. if data_mode equals QSPI_DATA_MODE_DUALSPI : length must >= 4 bytes and be multiple of HALFWORD
 *             3. if data_mode equals QSPI_DATA_MODE_QUADSPI : length must >= 8 bytes and be multiple of WORD
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_transmit_dma(qspi_handle_t *p_qspi, uint32_t data_mode, uint32_t data_size, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in non-blocking mode at standard SPI with DMA.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  data_mode : @ref QSPI_DATA_MODE_SPI
 *                              QSPI_DATA_MODE_DUALSPI
 *                              QSPI_DATA_MODE_QUADSPI
 * @param[in]  data_size : @ref QSPI_DATASIZE_08_BITS
 *                              QSPI_DATASIZE_16_BITS
 *                              QSPI_DATASIZE_32_BITS
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 *             1. if data_mode equals QSPI_DATA_MODE_SPI : length must >= 2 bytes
 *             2. if data_mode equals QSPI_DATA_MODE_DUALSPI : length must >= 4 bytes and be multiple of HALFWORD
 *             3. if data_mode equals QSPI_DATA_MODE_QUADSPI : length must >= 8 bytes and be multiple of WORD
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_receive_dma(qspi_handle_t *p_qspi, uint32_t data_mode, uint32_t data_size, uint8_t *p_data, uint32_t length);


/**
 ****************************************************************************************
 * @brief  Transmit Multi-Block of data with the specified instruction and address in non-blocking mode with DMA Linked List Block.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_llp_config: Pointer to Linked List Block
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_transmit_dma_llp(qspi_handle_t *p_qspi, qspi_command_t * p_cmd, dma_llp_config_t * p_llp_config);

/**
 ****************************************************************************************
 * @brief  Transmit Multi-Block of data without instruction and address in non-blocking mode with DMA Linked List Block.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi:               Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_llp_config:         Pointer to Linked List Block
 * @param[in]  data_mode :           @ref QSPI_Data_Mode
 * @param[in]  data_length:          Total data length of all blocks, in terms of bytes.
 *                                   1. if data_mode equals QSPI_DATA_MODE_SPI : data_length must >= 2 bytes
 *                                   2. if data_mode equals QSPI_DATA_MODE_DUALSPI : data_length must >= 4 bytes and be multiple of HALFWORD
 *                                   3. if data_mode equals QSPI_DATA_MODE_QUADSPI : data_length must >= 8 bytes and be multiple of WORD
 * @param[in]  clock_stretch_enable: Whether enable the clock stretch feature
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_transmit_dma_llp(qspi_handle_t *p_qspi, dma_llp_config_t * p_llp_config, uint32_t data_mode, uint32_t data_length, uint32_t clock_stretch_enable);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with the specified instruction and address in non-blocking mode with DMA Gather.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi:          Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd:           Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_gather_config: Pointer to DMA Gather Configure
 * @param[in]  p_data:          Pointer to data buffer to be sent
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_transmit_dma_gather(qspi_handle_t *p_qspi, qspi_command_t * p_cmd, dma_gather_config_t * p_gather_config, uint8_t *p_data);


/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with the specified instruction and address in non-blocking mode with DMA LLP.
 *         And In Each Block, xfered specified by gather, every data block has the same gather configuration
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi:          Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd:           Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_llp_config:    Pointer to DMA LLP Configure
 * @param[in]  p_gather_config: Pointer to DMA Gather Configure
 *
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_transmit_dma_llp_gather(qspi_handle_t *p_qspi, qspi_command_t * p_cmd,  dma_llp_config_t * p_llp_config,  dma_gather_config_t * p_gather_config);


/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction and address in non-blocking mode with DMA Scatter.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_data: Pointer to data buffer to be sent
 * @param[in]  sct_interval: Specifies the destination address increment/decrement in multiples of data_size in qspi_command_t on a scatter boundary
 * @param[in]  sct_count: Specifies the number of contiguous destination transfers of data_size in qspi_command_t between successive scatter intervals
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_receive_dma_scatter(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data,  uint32_t sct_interval, uint32_t sct_count);


/**
 ****************************************************************************************
 * @brief  : Used to write PSRAM in high speed mode with dma LLP
 *
 * @note   : THIS IS A RESTRICTED OR LIMITED USED FUNCTION !
 *
 *      It requires the following usage Conditions :
 *
 *           1. Just USED to WRITE PSRAM, And The Length Of Block MUST be under the control of tCEM of PSRAM
 *           2. This Function can be USED Just When QSPI Clock Frequency is half Of System Frequency. And MUST config the prescaler firstly before calling this
 *           3. This Function Just Supports QUAD WRITE Command Of PSRAM in Quad Mode, such as the 0x02/0x38 QPI WRITE CMD in IPUS or AP-MEMORY
 *           4. Following the third point, The instruction size must be 8-bits, The address size must be 24-bits
 *           5. This Function Just Supports Rectangular data block, Data length in each data block are the same
 *           6. This Function Supports TWO DATA Block Mode : QSPI_PSRAM_LINKED_BLOCK_DATA_MODE_0 (Loop Mode); QSPI_PSRAM_LINKED_BLOCK_DATA_MODE_1 (Crossed Mode)
 *           7. When in QSPI_PSRAM_LINKED_BLOCK_DATA_MODE_0 Mode, it supports the DATA_SIZE as 8/16/32bits;
 *              When in QSPI_PSRAM_LINKED_BLOCK_DATA_MODE_1 Mode, it just supports the DATA_SIZE as 8/16bits, not Support 32bits
 *           8. It Just support Single BURST for SRC&DST when configuring the DMA xfer
 *           9. Attention The Data Order.
 *              When Data_Size is 8  Bits : INST -> ADDR-High -> ADDR-Middle -> ADDR-Low -> Byte 0 -> Byte 1 -> Byte 2 -> ...
 *              When Data_Size is 16 Bits : ADDR-High -> INST -> ADDR-Low -> ADDR-Middle -> Short0.1 -> Short0.0 -> Short1.1 -> Short1.0 -> ...
 *              When Data_Size is 32 Bits : ADDR-Low -> ADDR-Middle -> ADDR-High -> INST -> Word 0.3 -> Word 0.2 -> Word 0.1 -> Word 0.0 -> ...
 *
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd : Pointer to a psram command configuration.
 * @param[in]  p_llp_config: Pointer to Linked List Block
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_psram_transmit_dma_llp_limited(qspi_handle_t *p_qspi, qspi_psram_command_t * p_cmd, dma_llp_config_t * p_llp_config);


/**
 ****************************************************************************************
 * @brief  Abort the current transmission.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_abort(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Abort the current transmission (non-blocking function)
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_abort_it(qspi_handle_t *p_qspi);

/** @} */

/** @addtogroup QSPI_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle QSPI interrupt request.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_irq_handler(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Tx Transfer completed callback.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_tx_cplt_callback(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Rx Transfer completed callback.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_rx_cplt_callback(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  QSPI error callback.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_error_callback(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  QSPI Abort Complete callback.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_abort_cplt_callback(qspi_handle_t *p_qspi);

/** @} */

/** @defgroup QSPI_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   QSPI control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the QSPI.
     (+) hal_qspi_get_state() API can be helpful to check in run-time the state of the QSPI peripheral.
     (+) hal_qspi_get_error() check in run-time Errors occurring during communication.
     (+) hal_qspi_set_timeout() set the timeout during internal process.
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the QSPI handle state.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @retval ::HAL_QSPI_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_QSPI_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_QSPI_STATE_BUSY: Peripheral in indirect mode and busy.
 * @retval ::HAL_QSPI_STATE_BUSY_INDIRECT_TX: Peripheral in indirect mode with transmission ongoing.
 * @retval ::HAL_QSPI_STATE_BUSY_INDIRECT_RX: Peripheral in indirect mode with reception ongoing.
 * @retval ::HAL_QSPI_STATE_ABORT: Peripheral with abort request ongoing.
 * @retval ::HAL_QSPI_STATE_ERROR: Peripheral in error.
 ****************************************************************************************
 */
hal_qspi_state_t hal_qspi_get_state(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Return the QSPI error code.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @return QSPI error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_qspi_get_error(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Set the QSPI cs setup & release time value.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  delay:  delay clocks for cs setup & release, [0, 0x7]
 * @retval :: None.
 ****************************************************************************************
 */
void hal_qspi_set_tcsu(qspi_handle_t *p_qspi, uint32_t delay);

/**
 ****************************************************************************************
 * @brief  Set the QSPI internal process timeout value.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  timeout: Internal process timeout value.
 * @retval :: None.
 ****************************************************************************************
 */
void hal_qspi_set_timeout(qspi_handle_t *p_qspi, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to QSPI configuration before sleep.
 * @param[in] p_qspi: Pointer to a QSPIhandle which contains the configuration
 *                 information for the specified QSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_suspend_reg(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to QSPI configuration after sleep.
 *         This function must be used in conjunction with the hal_qspi_suspend_reg().
 * @param[in] p_qspi: Pointer to a QSPI handle which contains the configuration
 *                 information for the specified QSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_resume_reg(qspi_handle_t *p_qspi);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_QSPI_H__ */

/** @} */

/** @} */

/** @} */

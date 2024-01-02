/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_qspi.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of QSPI LL library.
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

/** @addtogroup LL_DRIVER LL Driver
  * @{
  */

/** @defgroup LL_QSPI QSPI
  * @brief QSPI LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_QSPI_H__
#define __GR55xx_LL_QSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (QSPI0) || defined (QSPI1) || defined (QSPI2)

/** @defgroup LL_QSPI_DRIVER_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup LL_QSPI_ES_INIT QSPI Exported init structure
  * @{
  */

/**
  * @brief  QSPI init structures definition
  */
typedef struct _ll_qspi_init_t
{
    uint32_t transfer_direction;        /**< Specifies the QSPI transfer or receive mode.
                                             This parameter can be a value of @ref LL_QSPI_EC_TRANSFER_MODE.

                                             This feature can be modified afterwards using unitary function @ref ll_qspi_set_transfer_direction().*/

    uint32_t instruction_size;          /**< Specifies the QSPI instruction width.
                                             This parameter can be a value of @ref LL_QSPI_EC_INSTRUCTIONSIZE.

                                             This feature can be modified afterwards using unitary function @ref ll_qspi_set_instruction_size().*/

    uint32_t address_size;              /**< Specifies the QSPI address width.
                                             This parameter can be a value of @ref LL_QSPI_EC_ADDRESSSIZE.

                                             This feature can be modified afterwards using unitary function @ref ll_qspi_set_address_size().*/

    uint32_t inst_addr_transfer_format; /**< Specifies the QSPI instruction and address transfer format.
                                             This parameter can be a value of @ref LL_QSPI_EC_ADDRINSTTRNASFERFORMAT.

                                             This feature can be modified afterwards using unitary function @ref ll_qspi_set_add_inst_transfer_format().*/

    uint32_t wait_cycles;               /**< Specifies the QSPI dummy clock.
                                             This parameter can be one of the following values: 0 ~ 31.

                                             This feature can be modified afterwards using unitary function @ref ll_qspi_set_wait_cycles().*/

    uint32_t data_size;                 /**< Specifies the QSPI data width.
                                             This parameter can be a value of @ref SPI_LL_EC_DATASIZE.

                                             This feature can be modified afterwards using unitary function @ref ll_qspi_set_data_size().*/

    uint32_t clock_polarity;            /**< Specifies the serial clock steady state.
                                             This parameter can be a value of @ref SPI_LL_EC_POLARITY.
                                             This feature can be modified afterwards using unitary function @ref ll_qspi_set_clock_polarity().*/

    uint32_t clock_phase;               /**< Specifies the clock active edge for the bit capture.
                                             This parameter can be a value of @ref SPI_LL_EC_PHASE.

                                             This feature can be modified afterwards using unitary function @ref ll_qspi_set_clock_phase().*/

    uint32_t baud_rate;                 /**< Specifies the BaudRate prescaler value which will be used to configure the transmit and receive SCK clock.
                                             This parameter can be one even value between 2 and 65534, if the value is 0, the SCLK is disable.
                                             @note The communication clock is derived from the master clock. The slave clock does not need to be set.

                                             This feature can be modified afterwards using unitary function @ref ll_qspi_set_baud_rate_prescaler().*/

    uint32_t rx_sample_delay;           /**< Specifies the RX sample delay. It is used to delay the sample of the RX input port.
                                             This parameter can be a number between 0 and 0x7 */

} ll_qspi_init_t;

typedef enum {
    LL_QSPI_MEMORYMAPPED_MODE_READ_ONLY  = 0,   /* Specifies mmap mode to read only */
    LL_QSPI_MEMORYMAPPED_MODE_READ_WRITE = 1,   /* Specifies mmap mode to read and write */
} ll_qspi_memorymapped_mode_e;

typedef struct _ll_qspi_memorymapped_read_init_t
{
    /*
     * memorymapped read setting parameters
     */

    uint32_t x_dfs;                     /**< Specifies the QSPI data frame size in xip mode.
                                             This parameter can be a value of @ref LL_QSPI_CONCURRENT_XIP_DFS. */

    uint32_t x_dfs_hardcode_en;         /**< Specifies whether to enable the HSIZE hardcoded to DFS feature in memorymapped(xip) mode.
                                             @ref LL_QSPI_CONCURRENT_XIP_DFS_HARDCODE_ENABLE
                                             @ref LL_QSPI_CONCURRENT_XIP_DFS_HARDCODE_DISABLE
                                             */

    uint32_t x_instruction_en;          /**< Specifies whether to enable the instruction phase feature in memorymapped(xip) mode.
                                             @ref LL_QSPI_CONCURRENT_XIP_INST_ENABLE
                                             @ref LL_QSPI_CONCURRENT_XIP_INST_DISABLE
                                             */

    uint32_t x_instruction_size;        /**< Specifies instruction size in memorymapped(xip) mode.
                                             This parameter can be a value of @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE.
                                             */

    uint32_t x_instruction;             /**< Specifies instruction in memorymapped(xip) mode. */

    uint32_t x_address_size;            /**< Specifies instruction size in memorymapped(xip) mode.
                                             This parameter can be a value of @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE.
                                             */

    uint32_t x_inst_addr_transfer_format;   /**< Specifies xfer format of inst & addr in memorymapped(xip) mode.
                                                This parameter can be a value of @ref LL_QSPI_CONCURRENT_XIP_INST_ADDR_TRANSFER_FORMAT.
                                                */

    uint32_t x_mode_bits_en;            /**< Specifies whether to enable mode bits phase in memorymapped(xip) mode.
                                             @ref LL_QSPI_CONCURRENT_XIP_MODE_BITS_ENABLE
                                             @ref LL_QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE
                                             */

    uint32_t x_mode_bits_length;        /**< Specifies mode bits length
                                             This parameter can be a value of @ref LL_QSPI_CONCURRENT_XIP_MBL.
                                             */

    uint32_t x_mode_bits_data;          /**< Specifies value of mode bits phase */

    uint32_t x_dummy_cycles;            /**< Specifies wait(dummy) cycles in memorymapped(xip) mode.
                                             value range [0 ~ 31]
                                             */

    uint32_t x_data_frame_format;       /**< Specifies enhanced spi's frame format in memorymapped(xip) mode.
                                             This parameter can be a value of @ref LL_QSPI_CONCURRENT_XIP_FRF
                                             */

    uint32_t x_prefetch_en;             /**< Specifies whether to enable the prefetch feature.
                                             @ref LL_QSPI_CONCURRENT_XIP_PREFETCH_ENABLE
                                             @ref LL_QSPI_CONCURRENT_XIP_PREFETCH_DISABLE
                                             */

    uint32_t x_continous_xfer_en;       /**< Specifies whether to enable the continuous transfer feature in memorymapped(xip) mode.
                                             @ref LL_QSPI_CONCURRENT_XIP_CONT_XFER_ENABLE
                                             @ref LL_QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE
                                             */

    uint32_t x_continous_xfer_toc;      /**< Specifies timeout count for the continuous transfer feature in memorymapped(xip) mode.
                                             unit in terms of hclk, range [0, 255]
                                             */

    uint32_t x_endian_mode;            /**< Specifies endian mode in memorymapped(xip) mode.
                                             This parameter can be a value of @ref LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE
                                             */
} ll_qspi_memorymapped_read_init_t;



typedef struct _ll_qspi_memorymapped_write_init_t
{
    /*
     * memorymapped write setting parameters
     */

    uint32_t x_wr_instruction_size;        /**< Specifies instruction size in memorymapped(xip) write mode.
                                                This parameter can be a value of @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE.
                                                */

    uint32_t x_wr_instruction;             /**< Specifies instruction in memorymapped(xip) write mode. */

    uint32_t x_wr_address_size;            /**< Specifies instruction size in memorymapped(xip) write mode.
                                                This parameter can be a value of @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE.
                                                */

    uint32_t x_wr_inst_addr_transfer_format;   /**< Specifies xfer format of inst & addr in memorymapped(xip) write mode.
                                                    This parameter can be a value of @ref LL_QSPI_CONCURRENT_XIP_INST_ADDR_TRANSFER_FORMAT.
                                                    */

    uint32_t x_wr_dummy_cycles;            /**< Specifies wait(dummy) cycles in memorymapped(xip) write mode.
                                                value range [0 ~ 31]
                                                */

    uint32_t x_wr_data_frame_format;       /**< Specifies enhanced spi's frame format in memorymapped(xip) write mode.
                                                This parameter can be a value of @ref LL_QSPI_CONCURRENT_XIP_FRF
                                                */
} ll_qspi_memorymapped_write_init_t;


typedef struct _ll_qspi_memorymapped_init_t
{
    /*
     * basical setting parameters
     */

    uint32_t clock_polarity;            /**< Specifies the serial clock steady state.
                                             This parameter can be a value of @ref SPI_LL_EC_POLARITY.
                                             This feature can be modified afterwards using unitary function @ref ll_qspi_set_clock_polarity().*/

    uint32_t clock_phase;               /**< Specifies the clock active edge for the bit capture.
                                             This parameter can be a value of @ref SPI_LL_EC_PHASE.
                                             This feature can be modified afterwards using unitary function @ref ll_qspi_set_clock_phase().*/

    uint32_t baud_rate;                 /**< Specifies the BaudRate prescaler value which will be used to configure the transmit and receive SCK clock.
                                             This parameter can be one even value between 2 and 65534, if the value is 0, the SCLK is disable.
                                             @note The communication clock is derived from the master clock. The slave clock does not need to be set.
                                             This feature can be modified afterwards using unitary function @ref ll_qspi_set_baud_rate_prescaler().*/

    uint32_t rx_sample_delay;           /**< Specifies the RX sample delay. It is used to delay the sample of the RX input port.
                                             This parameter can be a number between 0 and 0x7 */

    ll_qspi_memorymapped_mode_e         rw_mode;    /**< Specifies the access mode for memorymapped: readonly or read&write */

    ll_qspi_memorymapped_read_init_t    rd;         /**< Specifies the initized params for read, must set */

    ll_qspi_memorymapped_write_init_t   wr;         /**< Specifies the initized params for write, if rw_mode is readonly, leave un-set */

} ll_qspi_memorymapped_init_t;

/** uint32_t clock_polarity;     @} */

/** @} */

/**
  * @defgroup LL_QSPI_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup LL_QSPI_Exported_Constants QSPI Exported Constants
  * @{
  */

/** @defgroup LL_QSPI_FIFO_DEPTH FIFO Depth Defines
  * @{
  */
#define LL_QSPI_MAX_FIFO_DEPTH                       (32u)                        /**< FIFO Depth for QSPI Master */

#define LL_QSPI0_REG_RX_FIFO_DEPTH                   (16u)                        /**< Receive  FIFO Depth Of Register Mode for QSPI0 Master */
#define LL_QSPI0_REG_TX_FIFO_DEPTH                   (16u)                        /**< Transmit FIFO Depth Of Register Mode for QSPI0 Master */
#define LL_QSPI0_XIP_RX_FIFO_DEPTH                   (32u)                        /**< Receive  FIFO Depth Of XIP Mode for QSPI0 Master */
#define LL_QSPI0_XIP_TX_FIFO_DEPTH                   (16u)                        /**< Transmit FIFO Depth Of XIP Mode for QSPI0 Master */

#define LL_QSPI1_REG_RX_FIFO_DEPTH                   (32u)                        /**< Receive  FIFO Depth Of Register Mode for QSPI1 Master */
#define LL_QSPI1_REG_TX_FIFO_DEPTH                   (32u)                        /**< Transmit FIFO Depth Of Register Mode for QSPI1 Master */
#define LL_QSPI1_XIP_RX_FIFO_DEPTH                   (32u)                        /**< Receive  FIFO Depth Of XIP Mode for QSPI1 Master */
#define LL_QSPI1_XIP_TX_FIFO_DEPTH                   (32u)                        /**< Transmit FIFO Depth Of XIP Mode for QSPI1 Master */

#define LL_QSPI2_REG_RX_FIFO_DEPTH                   (16u)                        /**< Receive  FIFO Depth Of Register Mode for QSPI2 Master */
#define LL_QSPI2_REG_TX_FIFO_DEPTH                   (32u)                        /**< Transmit FIFO Depth Of Register Mode for QSPI2 Master */
#define LL_QSPI2_XIP_RX_FIFO_DEPTH                   (16u)                        /**< Receive  FIFO Depth Of XIP Mode for QSPI2 Master */
#define LL_QSPI2_XIP_TX_FIFO_DEPTH                   (16u)                        /**< Transmit FIFO Depth Of XIP Mode for QSPI2 Master */
/** @} */

/** @defgroup LL_QSPI_EC_GET_FLAG Get Flags Defines
  * @brief    Flags definitions which can be used with LL_QSPI_ReadReg function
  * @{
  */
#define LL_QSPI_SR_DCOL                      QSPI_STAT_DCOL               /**< Data collision error flag      */
#define LL_QSPI_SR_TXE                       QSPI_STAT_TXE                /**< Transmission error flag        */
#define LL_QSPI_SR_RFF                       QSPI_STAT_RFF                /**< Rx FIFO full flag              */
#define LL_QSPI_SR_RFNE                      QSPI_STAT_RFNE               /**< Rx FIFO not empty flag         */
#define LL_QSPI_SR_TFE                       QSPI_STAT_TFE                /**< Tx FIFO empty flag             */
#define LL_QSPI_SR_TFNF                      QSPI_STAT_TFNF               /**< Tx FIFO not full flag          */
#define LL_QSPI_SR_BUSY                      QSPI_STAT_BUSY               /**< Busy flag                      */
/** @} */

/** @defgroup LL_QSPI_EC_IT IT Defines
  * @brief    Interrupt definitions which can be used with @ref LL_QSPI_ReadReg and @ref LL_QSPI_WriteReg functions
  * @{
  */
#define LL_QSPI_IM_SPITE                     QSPI_INTMASK_SPITEIM         /**< SPI Transmit Error Interrupt enable        */
#define LL_QSPI_IM_TXU                       QSPI_INTMASK_TXUIM           /**< Transmit FIFO Underflow Interrupt enable   */
#define LL_QSPI_IM_XRXO                      QSPI_INTMASK_XRXOIM          /**< XIP Receive FIFO Overflow Interrupt enable */
#define LL_QSPI_IM_MST                       QSPI_INTMASK_MSTIM           /**< Multi-Master Contention Interrupt enable   */
#define LL_QSPI_IM_RXF                       QSPI_INTMASK_RXFIM           /**< Receive FIFO Full Interrupt enable         */
#define LL_QSPI_IM_RXO                       QSPI_INTMASK_RXOIM           /**< Receive FIFO Overflow Interrupt  enable    */
#define LL_QSPI_IM_RXU                       QSPI_INTMASK_RXUIM           /**< Receive FIFO Underflow Interrupt  enable   */
#define LL_QSPI_IM_TXO                       QSPI_INTMASK_TXOIM           /**< Transmit FIFO Overflow Interrupt  enable   */
#define LL_QSPI_IM_TXE                       QSPI_INTMASK_TXEIM           /**< Transmit FIFO Empty Interrupt  enable      */
#define LL_QSPI_IM_ALL                       (LL_QSPI_IM_SPITE| \
                                              LL_QSPI_IM_TXU  | \
                                              LL_QSPI_IM_XRXO | \
                                              LL_QSPI_IM_MST  | \
                                              LL_QSPI_IM_RXF  | \
                                              LL_QSPI_IM_RXO  | \
                                              LL_QSPI_IM_RXU  | \
                                              LL_QSPI_IM_TXO  | \
                                              LL_QSPI_IM_TXE)

#define LL_QSPI_IS_SPITE                     QSPI_INTMASK_SPITEIS         /**< SPI Transmit Error Interrupt flag          */
#define LL_QSPI_IS_TXU                       QSPI_INTMASK_TXUIS           /**< Transmit FIFO Underflow Interrupt flag     */
#define LL_QSPI_IS_XRXO                      QSPI_INTSTAT_XRXOIS          /**< XIP Receive FIFO Overflow Interrupt flag   */
#define LL_QSPI_IS_MST                       QSPI_INTSTAT_MSTIS           /**< Multi-Master Contention Interrupt flag     */
#define LL_QSPI_IS_RXF                       QSPI_INTSTAT_RXFIS           /**< Receive FIFO Full Interrupt flag           */
#define LL_QSPI_IS_RXO                       QSPI_INTSTAT_RXOIS           /**< Receive FIFO Overflow Interrupt  flag      */
#define LL_QSPI_IS_RXU                       QSPI_INTSTAT_RXUIS           /**< Receive FIFO Underflow Interrupt  flag     */
#define LL_QSPI_IS_TXO                       QSPI_INTSTAT_TXOIS           /**< Transmit FIFO Overflow Interrupt  flag     */
#define LL_QSPI_IS_TXE                       QSPI_INTSTAT_TXEIS           /**< Transmit FIFO Empty Interrupt  flag        */
#define LL_QSPI_IS_ALL                       (LL_QSPI_IS_SPITE| \
                                              LL_QSPI_IS_TXU  | \
                                              LL_QSPI_IS_XRXO | \
                                              LL_QSPI_IS_MST  | \
                                              LL_QSPI_IS_RXF  | \
                                              LL_QSPI_IS_RXO  | \
                                              LL_QSPI_IS_RXU  | \
                                              LL_QSPI_IS_TXO  | \
                                              LL_QSPI_IS_TXE)

#define LL_QSPI_RIS_SPITE                    QSPI_RAW_INTMASK_SPITEIR     /**< SPI Transmit Error RAW Interrupt flag        */
#define LL_QSPI_RIS_TXU                      QSPI_RAW_INTMASK_TXUIIR      /**< Transmit FIFO Underflow RAW Interrupt flag   */
#define LL_QSPI_RIS_XRXO                     QSPI_RAW_INTSTAT_XRXOIR      /**< XIP Receive FIFO Overflow RAW Interrupt flag */
#define LL_QSPI_RIS_MST                      QSPI_RAW_INTSTAT_MSTIR       /**< Multi-Master Contention RAW Interrupt flag   */
#define LL_QSPI_RIS_RXF                      QSPI_RAW_INTSTAT_RXFIR       /**< Receive FIFO Full RAW Interrupt flag         */
#define LL_QSPI_RIS_RXO                      QSPI_RAW_INTSTAT_RXOIR       /**< Receive FIFO Overflow RAW Interrupt  flag    */
#define LL_QSPI_RIS_RXU                      QSPI_RAW_INTSTAT_RXUIR       /**< Receive FIFO Underflow RAW Interrupt  flag   */
#define LL_QSPI_RIS_TXO                      QSPI_RAW_INTSTAT_TXOIR       /**< Transmit FIFO Overflow RAW Interrupt  flag   */
#define LL_QSPI_RIS_TXE                      QSPI_RAW_INTSTAT_TXEIR       /**< Transmit FIFO Empty RAW Interrupt  flag      */
#define LL_QSPI_RIS_ALL                      (LL_QSPI_RIS_SPITE| \
                                              LL_QSPI_RIS_TXU  | \
                                              LL_QSPI_RIS_XRXO | \
                                              LL_QSPI_RIS_MST  | \
                                              LL_QSPI_RIS_RXF  | \
                                              LL_QSPI_RIS_RXO  | \
                                              LL_QSPI_RIS_RXU  | \
                                              LL_QSPI_RIS_TXO  | \
                                              LL_QSPI_RIS_TXE)

/** @} */

/** @defgroup LL_QSPI_EC_SPIFRAMEFORMAT SPI Frame Format
  * @{
  */
#define LL_QSPI_FRF_SPI                      0x00000000UL                    /**< SPI frame format for transfer      */
#define LL_QSPI_FRF_DUALSPI                  (1UL << QSPI_CTRL0_SPIFRF_Pos)   /**< Dual-SPI frame format for transfer */
#define LL_QSPI_FRF_QUADSPI                  (2UL << QSPI_CTRL0_SPIFRF_Pos)   /**< Quad-SPI frame format for transfer */
/** @} */

/** @defgroup LL_QSPI_EC_DATASIZE Datawidth
  * @{
  */
#define LL_QSPI_DATASIZE_4BIT                (3UL << QSPI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer:  4 bits */
#define LL_QSPI_DATASIZE_5BIT                (4UL << QSPI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer:  5 bits */
#define LL_QSPI_DATASIZE_6BIT                (5UL << QSPI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer:  6 bits */
#define LL_QSPI_DATASIZE_7BIT                (6UL << QSPI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer:  7 bits */
#define LL_QSPI_DATASIZE_8BIT                (7UL << QSPI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer:  8 bits */
#define LL_QSPI_DATASIZE_9BIT                (8UL << QSPI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer:  9 bits */
#define LL_QSPI_DATASIZE_10BIT               (9UL << QSPI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer: 10 bits */
#define LL_QSPI_DATASIZE_11BIT               (10UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 11 bits */
#define LL_QSPI_DATASIZE_12BIT               (11UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 12 bits */
#define LL_QSPI_DATASIZE_13BIT               (12UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 13 bits */
#define LL_QSPI_DATASIZE_14BIT               (13UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 14 bits */
#define LL_QSPI_DATASIZE_15BIT               (14UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 15 bits */
#define LL_QSPI_DATASIZE_16BIT               (15UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 16 bits */
#define LL_QSPI_DATASIZE_17BIT               (16UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 17 bits */
#define LL_QSPI_DATASIZE_18BIT               (17UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 18 bits */
#define LL_QSPI_DATASIZE_19BIT               (18UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 19 bits */
#define LL_QSPI_DATASIZE_20BIT               (19UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 20 bits */
#define LL_QSPI_DATASIZE_21BIT               (20UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 21 bits */
#define LL_QSPI_DATASIZE_22BIT               (21UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 22 bits */
#define LL_QSPI_DATASIZE_23BIT               (22UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 23 bits */
#define LL_QSPI_DATASIZE_24BIT               (23UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 24 bits */
#define LL_QSPI_DATASIZE_25BIT               (24UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 25 bits */
#define LL_QSPI_DATASIZE_26BIT               (25UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 26 bits */
#define LL_QSPI_DATASIZE_27BIT               (26UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 27 bits */
#define LL_QSPI_DATASIZE_28BIT               (27UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 28 bits */
#define LL_QSPI_DATASIZE_29BIT               (28UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 29 bits */
#define LL_QSPI_DATASIZE_30BIT               (29UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 30 bits */
#define LL_QSPI_DATASIZE_31BIT               (30UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 31 bits */
#define LL_QSPI_DATASIZE_32BIT               (31UL << QSPI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 32 bits */
/** @} */

/** @defgroup LL_QSPI_EC_MICROWIRECOMMANDSIZE MicroWire CommandSize
  * @{
  */
#define LL_QSPI_MW_CMDSIZE_1BIT              0x00000000UL                    /**< CMD length for Microwire transfer:  1 bits */
#define LL_QSPI_MW_CMDSIZE_2BIT              (1UL << QSPI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  2 bits */
#define LL_QSPI_MW_CMDSIZE_3BIT              (2UL << QSPI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  3 bits */
#define LL_QSPI_MW_CMDSIZE_4BIT              (3UL << QSPI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  4 bits */
#define LL_QSPI_MW_CMDSIZE_5BIT              (4UL << QSPI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  5 bits */
#define LL_QSPI_MW_CMDSIZE_6BIT              (5UL << QSPI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  6 bits */
#define LL_QSPI_MW_CMDSIZE_7BIT              (6UL << QSPI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  7 bits */
#define LL_QSPI_MW_CMDSIZE_8BIT              (7UL << QSPI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  8 bits */
#define LL_QSPI_MW_CMDSIZE_9BIT              (8UL << QSPI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  9 bits */
#define LL_QSPI_MW_CMDSIZE_10BIT             (9UL << QSPI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer: 10 bits */
#define LL_QSPI_MW_CMDSIZE_11BIT             (10UL << QSPI_CTRL0_CFS_Pos)     /**< CMD length for Microwire transfer: 11 bits */
#define LL_QSPI_MW_CMDSIZE_12BIT             (11UL << QSPI_CTRL0_CFS_Pos)     /**< CMD length for Microwire transfer: 12 bits */
#define LL_QSPI_MW_CMDSIZE_13BIT             (12UL << QSPI_CTRL0_CFS_Pos)     /**< CMD length for Microwire transfer: 13 bits */
#define LL_QSPI_MW_CMDSIZE_14BIT             (13UL << QSPI_CTRL0_CFS_Pos)     /**< CMD length for Microwire transfer: 14 bits */
#define LL_QSPI_MW_CMDSIZE_15BIT             (14UL << QSPI_CTRL0_CFS_Pos)     /**< CMD length for Microwire transfer: 15 bits */
#define LL_QSPI_MW_CMDSIZE_16BIT             (15UL << QSPI_CTRL0_CFS_Pos)     /**< CMD length for Microwire transfer: 16 bits */
/** @} */

/** @defgroup LL_QSPI_EC_TEST_MODE Test Mode
  * @{
  */
#define LL_QSPI_NORMAL_MODE                  0x00000000UL                    /**< Normal mode for SPI transfer                           */
#define LL_QSPI_TEST_MODE                    (1UL << QSPI_CTRL0_SRL_Pos)      /**< Test mode for SPI transfer: Rx and Tx connected inside */
/** @} */

/** @defgroup LL_QSPI_EC_SLAVEOUT_ENABLE Slave Out Enable
  * @{
  */
#define LL_QSPI_SLAVE_OUTDIS                 0x00000000UL                    /**< Output enable for SPI transfer as slave    */
#define LL_QSPI_SLAVE_OUTEN                  (1UL << QSPI_CTRL0_SLVOE_Pos)    /**< Output disable for SPI transfer as slave   */
/** @} */

/** @defgroup LL_QSPI_EC_TRANSFER_MODE Transfer Mode
  * @{
  */
#define LL_QSPI_FULL_DUPLEX                  0x00000000UL                    /**< Full-Duplex mode. Rx and Tx transfer on 2 lines */
#define LL_QSPI_SIMPLEX_TX                   (1UL << QSPI_CTRL0_TMOD_Pos)     /**< Simplex Tx mode.  Tx transfer only on 1 line    */
#define LL_QSPI_SIMPLEX_RX                   (2UL << QSPI_CTRL0_TMOD_Pos)     /**< Simplex Rx mode.  Rx transfer only on 1 line    */
#define LL_QSPI_READ_EEPROM                  (3UL << QSPI_CTRL0_TMOD_Pos)     /**< Read EEPROM mode.  Rx transfer only on 1 line   */
/** @} */

/** @defgroup LL_QSPI_EC_PHASE Clock Phase
  * @{
  */
#define LL_QSPI_SCPHA_1EDGE                  0x00000000UL                    /**< First clock transition is the first data capture edge  */
#define LL_QSPI_SCPHA_2EDGE                  (1UL << QSPI_CTRL0_SCPHA_Pos)    /**< Second clock transition is the first data capture edge */
/** @} */

/** @defgroup LL_QSPI_EC_POLARITY Clock Polarity
  * @{
  */
#define LL_QSPI_SCPOL_LOW                    0x00000000UL                    /**< Clock to 0 when idle */
#define LL_QSPI_SCPOL_HIGH                   (1UL << QSPI_CTRL0_SCPOL_Pos)    /**< Clock to 1 when idle */
/** @} */

/** @defgroup LL_QSPI_EC_PROTOCOL Serial Protocol
  * @{
  */
#define LL_QSPI_PROTOCOL_MOTOROLA            0x00000000UL                /**< Motorola mode. Used as default value */
#define LL_QSPI_PROTOCOL_TI                  (1UL << QSPI_CTRL0_FRF_Pos)  /**< TI mode                              */
#define LL_QSPI_PROTOCOL_MICROWIRE           (2UL << QSPI_CTRL0_FRF_Pos)  /**< Microwire mode                       */
/** @} */

/** @defgroup LL_QSPI_EC_MICROWIRECONTROL MicroWire Control
  * @{
  */
#define LL_QSPI_MICROWIRE_HANDSHAKE_DIS      0x00000000UL                /**< Enable Handshake for Microwire transfer  */
#define LL_QSPI_MICROWIRE_HANDSHAKE_EN       (1UL << QSPI_MWC_MHS_Pos)    /**< Disable Handshake for Microwire transfer */

#define LL_QSPI_MICROWIRE_RX                 0x00000000UL                /**< Rx mode. Rx transfer at Microwire mode */
#define LL_QSPI_MICROWIRE_TX                 (1UL << QSPI_MWC_MDD_Pos)    /**< Tx mode. Tx transfer at Microwire mode */

#define LL_QSPI_MICROWIRE_NON_SEQUENTIAL     0x00000000UL                /**< Non-sequential for Microwire transfer  */
#define LL_QSPI_MICROWIRE_SEQUENTIAL         (1UL << QSPI_MWC_MWMOD_Pos)  /**< Sequential for Microwire transfer      */
/** @} */

/** @defgroup LL_QSPI_EC_SLAVESELECT Slave Select
  * @{
  */
#define LL_QSPI_SLAVE1                       QSPI_SE_SLAVE1               /**< Enable slave1 select pin for SPI transfer  */
#define LL_QSPI_SLAVE0                       QSPI_SE_SLAVE0               /**< Enable slave0 select pin for SPI transfer  */
/** @} */

/** @defgroup LL_QSPI_EC_DMA DMA Defines
  * @{
  */
#define LL_QSPI_DMA_TX_DIS                   0x00000000UL                /**< Disable the transmit FIFO DMA channel */
#define LL_QSPI_DMA_TX_EN                    QSPI_DMAC_TDMAE              /**< Enable the transmit FIFO DMA channel  */

#define LL_QSPI_DMA_RX_DIS                   0x00000000UL                /**< Disable the receive FIFO DMA channel */
#define LL_QSPI_DMA_RX_EN                    QSPI_DMAC_RDMAE              /**< Enable the receive FIFO DMA channel  */
/** @} */

/** @defgroup LL_QSPI_EC_INSTRUCTIONSIZE QSPI Instruction Size
  * @{
  */
#define LL_QSPI_INSTSIZE_0BIT                0x00000000UL                        /**< Instruction length for QSPI transfer:  0 bits */
#define LL_QSPI_INSTSIZE_4BIT                (1UL << QSPI_SCTRL0_INSTL_Pos)       /**< Instructoin length for QSPI transfer:  4 bits */
#define LL_QSPI_INSTSIZE_8BIT                (2UL << QSPI_SCTRL0_INSTL_Pos)       /**< Instructoin length for QSPI transfer:  8 bits */
#define LL_QSPI_INSTSIZE_16BIT               (3UL << QSPI_SCTRL0_INSTL_Pos)       /**< Instructoin length for QSPI transfer: 16 bits */
/** @} */

/** @defgroup LL_QSPI_EC_ADDRESSSIZE QSPI Address Size
  * @{
  */
#define LL_QSPI_ADDRSIZE_0BIT                0x00000000UL                        /**< Address length for QSPI transfer:  0 bits */
#define LL_QSPI_ADDRSIZE_4BIT                (1UL << QSPI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer:  4 bits */
#define LL_QSPI_ADDRSIZE_8BIT                (2UL << QSPI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer:  8 bits */
#define LL_QSPI_ADDRSIZE_12BIT               (3UL << QSPI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 12 bits */
#define LL_QSPI_ADDRSIZE_16BIT               (4UL << QSPI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 16 bits */
#define LL_QSPI_ADDRSIZE_20BIT               (5UL << QSPI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 20 bits */
#define LL_QSPI_ADDRSIZE_24BIT               (6UL << QSPI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 24 bits */
#define LL_QSPI_ADDRSIZE_28BIT               (7UL << QSPI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 28 bits */
#define LL_QSPI_ADDRSIZE_32BIT               (8UL << QSPI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 32 bits */
#define LL_QSPI_ADDRSIZE_36BIT               (9UL << QSPI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 36 bits */
#define LL_QSPI_ADDRSIZE_40BIT               (10UL << QSPI_SCTRL0_ADDRL_Pos)      /**< Address length for QSPI transfer: 40 bits */
#define LL_QSPI_ADDRSIZE_44BIT               (11UL << QSPI_SCTRL0_ADDRL_Pos)      /**< Address length for QSPI transfer: 44 bits */
#define LL_QSPI_ADDRSIZE_48BIT               (12UL << QSPI_SCTRL0_ADDRL_Pos)      /**< Address length for QSPI transfer: 48 bits */
#define LL_QSPI_ADDRSIZE_52BIT               (13UL << QSPI_SCTRL0_ADDRL_Pos)      /**< Address length for QSPI transfer: 52 bits */
#define LL_QSPI_ADDRSIZE_56BIT               (14UL << QSPI_SCTRL0_ADDRL_Pos)      /**< Address length for QSPI transfer: 56 bits */
#define LL_QSPI_ADDRSIZE_60BIT               (15UL << QSPI_SCTRL0_ADDRL_Pos)      /**< Address length for QSPI transfer: 60 bits */
/** @} */

/** @defgroup LL_QSPI_RX_SAMPLE_EDGE QSPI RX SAMPLE EDGE
  * @{
  */
#define LL_QSPI_RX_SAMPLE_POSITIVE_EDGE        (0U)
#define LL_QSPI_RX_SAMPLE_NEGATIVE_EDGE        (1U)
/** @} */

/** @defgroup LL_QSPI_EC_ADDRINSTTRNASFERFORMAT QSPI Address and Instruction Transfer Format
  * @{
  */
#define LL_QSPI_INST_ADDR_ALL_IN_SPI         0x00000000UL                        /**< Instruction and address are sent in SPI mode */
#define LL_QSPI_INST_IN_SPI_ADDR_IN_SPIFRF   (1UL << QSPI_SCTRL0_TRANSTYPE_Pos)   /**< Instruction is in sent in SPI mode and address is sent in Daul/Quad SPI mode */
#define LL_QSPI_INST_ADDR_ALL_IN_SPIFRF      (2UL << QSPI_SCTRL0_TRANSTYPE_Pos)   /**< Instruction and address are sent in Daul/Quad SPI mode */
/** @} */


/** @defgroup LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE Endian mode For XIP
  * @brief    Endian mode for qspi xip
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE_0                0u      /* Default endian order from AHB */
#define LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE_1                1u      /* Re-order the read data as [23:16], [31:24], [7:0], [15:8] */
#define LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE_2                2u      /* Re-order the read data as [7:0], [15:8], [23:16], [31:24] */
/** @} */

/** @defgroup LL_QSPI_CONCURRENT_XIP_SLAVE Slave For XIP
  * @brief    Which Slave to Enable in XIP
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_SLAVE0                        QSPI_XIP_SLAVE0_EN
/** @} */

/** @defgroup LL_QSPI_CONCURRENT_XIP_DFS Data Frame Size For Xip 
  * @brief  Data frame size in xip, take effect when enable DFS_HC
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_DFS_BYTE                     LL_QSPI_DATASIZE_8BIT
#define LL_QSPI_CONCURRENT_XIP_DFS_HALFWORD                 LL_QSPI_DATASIZE_16BIT
#define LL_QSPI_CONCURRENT_XIP_DFS_WORD                     LL_QSPI_DATASIZE_32BIT
/** @} */

/** @defgroup LL_QSPI_CONCURRENT_XIP_MBL Mode Bits Length For Xip  
  * @brief  Mode bits length for xip mode
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_MBL_2                        0x0     /* mode bits length equals to 2 bit  */
#define LL_QSPI_CONCURRENT_XIP_MBL_4                        0x1     /* mode bits length equals to 4 bit  */
#define LL_QSPI_CONCURRENT_XIP_MBL_8                        0x2     /* mode bits length equals to 8 bit  */
#define LL_QSPI_CONCURRENT_XIP_MBL_16                       0x3     /* mode bits length equals to 16 bit */
/** @} */

/** @defgroup LL_QSPI_CONCURRENT_XIP_INSTSIZE Instruction Size For Xip  
  * @brief  Instruction size for concurrent xip mode
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_INSTSIZE_0BIT                0x0     /* no instruction */
#define LL_QSPI_CONCURRENT_XIP_INSTSIZE_4BIT                0x1     /* instruction size equals 4bits  */
#define LL_QSPI_CONCURRENT_XIP_INSTSIZE_8BIT                0x2     /* instruction size equals 8bits  */
#define LL_QSPI_CONCURRENT_XIP_INSTSIZE_16BIT               0x3     /* instruction size equals 16bits */
/** @} */

/** @defgroup LL_QSPI_CONCURRENT_XIP_ADDRSIZE  Address Size For Xip 
  * @brief  Address size for concurrent xip mode
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_0BIT                0x0     /**< Address length for QSPI XIP transfer:  0 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_4BIT                0x1     /**< Address length for QSPI XIP transfer:  4 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_8BIT                0x2     /**< Address length for QSPI XIP transfer:  8 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_12BIT               0x3     /**< Address length for QSPI XIP transfer: 12 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_16BIT               0x4     /**< Address length for QSPI XIP transfer: 16 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_20BIT               0x5     /**< Address length for QSPI XIP transfer: 20 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT               0x6     /**< Address length for QSPI XIP transfer: 24 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_28BIT               0x7     /**< Address length for QSPI XIP transfer: 28 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_32BIT               0x8     /**< Address length for QSPI XIP transfer: 32 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_36BIT               0x9     /**< Address length for QSPI XIP transfer: 36 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_40BIT               0xA     /**< Address length for QSPI XIP transfer: 40 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_44BIT               0xB     /**< Address length for QSPI XIP transfer: 44 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_48BIT               0xC     /**< Address length for QSPI XIP transfer: 48 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_52BIT               0xD     /**< Address length for QSPI XIP transfer: 52 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_56BIT               0xE     /**< Address length for QSPI XIP transfer: 56 bits */
#define LL_QSPI_CONCURRENT_XIP_ADDRSIZE_60BIT               0xF     /**< Address length for QSPI XIP transfer: 60 bits */
/** @} */

/** @defgroup LL_QSPI_CONCURRENT_XIP_INST_ADDR_TRANSFER_FORMAT   Instruction And Address For Xip 
  * @brief  Transfer of inst & address for concurrent xip mode
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPI         0x0     /**< Instruction and address are sent in SPI mode */
#define LL_QSPI_CONCURRENT_XIP_INST_IN_SPI_ADDR_IN_SPIFRF   0x1     /**< Instruction is in sent in SPI mode and address is sent in Daul/Quad SPI mode */
#define LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF      0x2     /**< Instruction and address are sent in Daul/Quad SPI mode */
/** @} */

/** @defgroup LL_QSPI_CONCURRENT_XIP_FRF  Frame Format For Xip 
  * @brief  Frame format for concurrent xip mode
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_FRF_RSVD                     0x0     /**< SPI Frame format : Reserved */
#define LL_QSPI_CONCURRENT_XIP_FRF_DUAL_SPI                 0x1     /**< SPI Frame format : DUAL     */
#define LL_QSPI_CONCURRENT_XIP_FRF_QUAD_SPI                 0x2     /**< SPI Frame format : QUAD     */
#define LL_QSPI_CONCURRENT_XIP_FRF_OCTAL_SPI                0x3     /**< SPI Frame format : OCTAL    */
/** @} */

/** @defgroup LL_QSPI_XIP_CLK_STRETCH Mode Clock stretch mode
  * @{
  */
#define LL_QSPI_CLK_STRETCH_ENABLE                          1u
#define LL_QSPI_CLK_STRETCH_DISABLE                         0u
/** @} */

/** @defgroup LL_QSPI_XIP_PREFETCH  Prefetch Defines
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_PREFETCH_ENABLE              1u
#define LL_QSPI_CONCURRENT_XIP_PREFETCH_DISABLE             0u
/** @} */

/** @defgroup LL_QSPI_XIP_CONT_XFERR  CONT XFERR Defines
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_CONT_XFER_ENABLE             1u
#define LL_QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE            0u
/** @} */

/** @defgroup LL_QSPI_XIP_INST_PHASE  Instruction Defines
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_INST_ENABLE                  1u
#define LL_QSPI_CONCURRENT_XIP_INST_DISABLE                 0u
/** @} */

/** @defgroup LL_QSPI_XIP_MODE_BITS_PHASE  Mode Bits Defines
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_MODE_BITS_ENABLE             1u
#define LL_QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE            0u
/** @} */

/** @defgroup LL_QSPI_XIP_DFS_HC DFS Hardcode Defines
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_DFS_HARDCODE_ENABLE          1u
#define LL_QSPI_CONCURRENT_XIP_DFS_HARDCODE_DISABLE         0u
/** @} */

/** @defgroup LL_QSPI_CONCURRENT_XIP_INST_SENT_MODE  Instruction Sent Mode
  * @{
  */
#define LL_QSPI_CONCURRENT_XIP_INST_SENT_EVERY_ACCESS       0u     /*!< Send instruction for every transaction */
#define LL_QSPI_CONCURRENT_XIP_INST_SENT_ONLY_FIRST_ACCESS  1u     /*!< Send instruction only for first transaction */
/** @} */


/** @defgroup LL_QSPI_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */

/**
  * @brief LL QSPI InitStrcut default configuartion
  */
#define LL_QSPI_DEFAULT_CONFIG                                  \
{                                                               \
    .transfer_direction  = LL_QSPI_SIMPLEX_RX,                  \
    .instruction_size    = LL_QSPI_INSTSIZE_8BIT,               \
    .address_size        = LL_QSPI_ADDRSIZE_24BIT,              \
    .inst_addr_transfer_format = LL_QSPI_INST_ADDR_ALL_IN_SPI,  \
    .wait_cycles         = 0,                                   \
    .data_size           = LL_QSPI_DATASIZE_8BIT,               \
    .clock_polarity      = LL_QSPI_SCPOL_LOW,                   \
    .clock_phase         = LL_QSPI_SCPHA_1EDGE,                 \
    .baud_rate           = SystemCoreClock / 1000000,           \
    .rx_sample_delay     = 0,                                   \
}



#define LL_CONC_QSPI_DEFAULT_CONFIG                             \
{                                                               \
    .baud_rate           = SystemCoreClock / 1000000,           \
    .clock_polarity      = LL_QSPI_SCPOL_LOW,                    \
    .clock_phase         = LL_QSPI_SCPHA_1EDGE,                  \
    .data_size           = LL_QSPI_DATASIZE_8BIT,                \
    .clock_stretch_en    = LL_QSPI_CLK_STRETCH_DISABLE,          \
    .transfer_direction  = LL_QSPI_SIMPLEX_RX,                   \
    .instruction_size    = LL_QSPI_INSTSIZE_8BIT,                \
    .address_size        = LL_QSPI_ADDRSIZE_24BIT,               \
    .inst_addr_transfer_format = LL_QSPI_INST_ADDR_ALL_IN_SPI,   \
    .wait_cycles         = 0,                                   \
    .rx_sample_delay     = 0,                                   \
    .data_beats          = 0,                                   \
    .tx_start_fifo_threshold = 0,                               \
    .tx_fifo_threshold   = 0,                                   \
    .rx_fifo_threshold   = 0,                                   \
    .dma_tx_fifo_level   = 0,                                   \
    .dma_rx_fifo_level   = 0,                                   \
                                                                \
    .x_prefetch_en       = LL_QSPI_CONCURRENT_XIP_PREFETCH_DISABLE,          \
    .x_continous_xfer_en = LL_QSPI_CONCURRENT_XIP_CONT_XFER_DISABLE,         \
    .x_continous_xfer_toc = 0x00,                               \
    .x_dfs_hardcode_en   = LL_QSPI_CONCURRENT_XIP_DFS_HARDCODE_DISABLE,     \
    .x_mode_bits_en      = LL_QSPI_CONCURRENT_XIP_MODE_BITS_DISABLE,        \
    .x_mode_bits_length  = LL_QSPI_CONCURRENT_XIP_MBL_8,                    \
    .x_mode_bits_data    = 0x00,                                \
    .x_instruction_en    = LL_QSPI_CONCURRENT_XIP_INST_DISABLE,             \
    .x_instruction_size  = LL_QSPI_CONCURRENT_XIP_INSTSIZE_8BIT, \
    .x_instruction       = 0x00,                                \
    .x_address_size      = LL_QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT,\
    .x_inst_addr_transfer_format = LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF, \
    .x_dummy_cycles      = 0x00,                                \
    .x_data_frame_format = LL_QSPI_CONCURRENT_XIP_FRF_QUAD_SPI,  \
}

/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup LL_QSPI_Exported_Macros QSPI Exported Macros
  * @{
  */

/** @defgroup LL_QSPI_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in QSPI register
  * @param  __instance__ QSPI instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_QSPI_WriteReg(__instance__, __REG__, __VALUE__)   WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in QSPI register
  * @param  __instance__ QSPI instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_QSPI_ReadReg(__instance__, __REG__)               READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup LL_QSPI_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup LL_QSPI_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable slave select toggle
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SSTEN
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_enable_ss_toggle(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->CTRL0, QSPI_CTRL0_SSTEN);
}

/**
  * @brief  Disable slave select toggle
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SSTEN
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_disable_ss_toggle(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->CTRL0, QSPI_CTRL0_SSTEN);
}

/**
  * @brief  Check if slave select toggle is enabled
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SSTEN
  *
  * @param  QSPIx QSPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_qspi_is_enabled_ss_toggle(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->CTRL0, QSPI_CTRL0_SSTEN) == (QSPI_CTRL0_SSTEN));
}

/**
  * @brief  Set data frame format for transmitting/receiving the data
  * @note   This bit should be written only when QSPI is disabled (QSPI_SSI_EN = 0) for correct operation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SPIFRF
  *
  * @param  QSPIx QSPI instance
  * @param  frf This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_FRF_SPI
  *         @arg @ref LL_QSPI_FRF_DUALSPI
  *         @arg @ref LL_QSPI_FRF_QUADSPI
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_frame_format(qspi_regs_t *QSPIx, uint32_t frf)
{
    MODIFY_REG(QSPIx->CTRL0, QSPI_CTRL0_SPIFRF, frf);
}

/**
  * @brief  Get data frame format for transmitting/receiving the data
  * @note   This bit should be written only when SPI is disabled (QSPI_EN = 0) for correct operation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SPIFRF
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_FRF_SPI
  *         @arg @ref LL_QSPI_FRF_DUALSPI
  *         @arg @ref LL_QSPI_FRF_QUADSPI
  */
__STATIC_INLINE uint32_t ll_qspi_get_frame_format(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->CTRL0, QSPI_CTRL0_SPIFRF));
}

/**
  * @brief  Set frame data size
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | DFS32
  *
  * @param  QSPIx QSPI instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_DATASIZE_4BIT
  *         @arg @ref LL_QSPI_DATASIZE_5BIT
  *         @arg @ref LL_QSPI_DATASIZE_6BIT
  *         @arg @ref LL_QSPI_DATASIZE_7BIT
  *         @arg @ref LL_QSPI_DATASIZE_8BIT
  *         @arg @ref LL_QSPI_DATASIZE_9BIT
  *         @arg @ref LL_QSPI_DATASIZE_10BIT
  *         @arg @ref LL_QSPI_DATASIZE_11BIT
  *         @arg @ref LL_QSPI_DATASIZE_12BIT
  *         @arg @ref LL_QSPI_DATASIZE_13BIT
  *         @arg @ref LL_QSPI_DATASIZE_14BIT
  *         @arg @ref LL_QSPI_DATASIZE_15BIT
  *         @arg @ref LL_QSPI_DATASIZE_16BIT
  *         @arg @ref LL_QSPI_DATASIZE_17BIT
  *         @arg @ref LL_QSPI_DATASIZE_18BIT
  *         @arg @ref LL_QSPI_DATASIZE_19BIT
  *         @arg @ref LL_QSPI_DATASIZE_20BIT
  *         @arg @ref LL_QSPI_DATASIZE_21BIT
  *         @arg @ref LL_QSPI_DATASIZE_22BIT
  *         @arg @ref LL_QSPI_DATASIZE_23BIT
  *         @arg @ref LL_QSPI_DATASIZE_24BIT
  *         @arg @ref LL_QSPI_DATASIZE_25BIT
  *         @arg @ref LL_QSPI_DATASIZE_26BIT
  *         @arg @ref LL_QSPI_DATASIZE_27BIT
  *         @arg @ref LL_QSPI_DATASIZE_28BIT
  *         @arg @ref LL_QSPI_DATASIZE_29BIT
  *         @arg @ref LL_QSPI_DATASIZE_30BIT
  *         @arg @ref LL_QSPI_DATASIZE_31BIT
  *         @arg @ref LL_QSPI_DATASIZE_32BIT
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_data_size(qspi_regs_t *QSPIx, uint32_t size)
{
    MODIFY_REG(QSPIx->CTRL0, QSPI_CTRL0_DFS32, size);
}

/**
  * @brief  Get frame data size
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | DFS32
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_DATASIZE_4BIT
  *         @arg @ref LL_QSPI_DATASIZE_5BIT
  *         @arg @ref LL_QSPI_DATASIZE_6BIT
  *         @arg @ref LL_QSPI_DATASIZE_7BIT
  *         @arg @ref LL_QSPI_DATASIZE_8BIT
  *         @arg @ref LL_QSPI_DATASIZE_9BIT
  *         @arg @ref LL_QSPI_DATASIZE_10BIT
  *         @arg @ref LL_QSPI_DATASIZE_11BIT
  *         @arg @ref LL_QSPI_DATASIZE_12BIT
  *         @arg @ref LL_QSPI_DATASIZE_13BIT
  *         @arg @ref LL_QSPI_DATASIZE_14BIT
  *         @arg @ref LL_QSPI_DATASIZE_15BIT
  *         @arg @ref LL_QSPI_DATASIZE_16BIT
  *         @arg @ref LL_QSPI_DATASIZE_17BIT
  *         @arg @ref LL_QSPI_DATASIZE_18BIT
  *         @arg @ref LL_QSPI_DATASIZE_19BIT
  *         @arg @ref LL_QSPI_DATASIZE_20BIT
  *         @arg @ref LL_QSPI_DATASIZE_21BIT
  *         @arg @ref LL_QSPI_DATASIZE_22BIT
  *         @arg @ref LL_QSPI_DATASIZE_23BIT
  *         @arg @ref LL_QSPI_DATASIZE_24BIT
  *         @arg @ref LL_QSPI_DATASIZE_25BIT
  *         @arg @ref LL_QSPI_DATASIZE_26BIT
  *         @arg @ref LL_QSPI_DATASIZE_27BIT
  *         @arg @ref LL_QSPI_DATASIZE_28BIT
  *         @arg @ref LL_QSPI_DATASIZE_29BIT
  *         @arg @ref LL_QSPI_DATASIZE_30BIT
  *         @arg @ref LL_QSPI_DATASIZE_31BIT
  *         @arg @ref LL_QSPI_DATASIZE_32BIT
  */
__STATIC_INLINE uint32_t ll_qspi_get_data_size(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->CTRL0, QSPI_CTRL0_DFS32));
}

/**
  * @brief  Set the length of the control word for the Microwire frame format
  * @note   This bit should be written only when SPI is disabled (QSPI_EN = 0) for correct operation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | CFS
  *
  * @param  QSPIx QSPI instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_MW_CMDSIZE_1BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_2BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_3BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_4BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_5BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_6BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_7BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_8BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_9BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_10BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_11BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_12BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_13BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_14BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_15BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_16BIT
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_control_frame_size(qspi_regs_t *QSPIx, uint32_t size)
{
    MODIFY_REG(QSPIx->CTRL0, QSPI_CTRL0_CFS, size);
}

/**
  * @brief  Get the length of the control word for the Microwire frame format
  * @note   This bit should be written only when SPI is disabled (QSPI_EN = 0) for correct operation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | CFS
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_MW_CMDSIZE_1BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_2BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_3BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_4BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_5BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_6BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_7BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_8BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_9BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_10BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_11BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_12BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_13BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_14BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_15BIT
  *         @arg @ref LL_QSPI_MW_CMDSIZE_16BIT
  */
__STATIC_INLINE uint32_t ll_qspi_get_control_frame_size(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->CTRL0, QSPI_CTRL0_CFS));
}

/**
  * @brief  Enable SPI test mode
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SRL
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_enable_test_mode(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->CTRL0, QSPI_CTRL0_SRL);
}

/**
  * @brief  Disable SPI test mode
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SRL
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_disable_test_mode(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->CTRL0, QSPI_CTRL0_SRL);
}

/**
  * @brief  Check if SPI test mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SRL
  *
  * @param  QSPIx QSPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_qspi_is_enabled_test_mode(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->CTRL0, QSPI_CTRL0_SRL) == (QSPI_CTRL0_SRL));
}

/**
  * @brief  Enable slave output
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SLVOE
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_enable_slave_out(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->CTRL0, QSPI_CTRL0_SLVOE);
}

/**
  * @brief  Disable slave output
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SLVOE
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_disable_salve_out(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->CTRL0, QSPI_CTRL0_SLVOE);
}

/**
  * @brief  Check if slave output is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SLVOE
  *
  * @param  QSPIx QSPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_qspi_is_enabled_slave_out(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->CTRL0, QSPI_CTRL0_SLVOE) != (QSPI_CTRL0_SLVOE));
}

/**
  * @brief  Set transfer direction mode
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | TMOD
  *
  * @param  QSPIx QSPI instance
  * @param  transfer_direction This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_FULL_DUPLEX
  *         @arg @ref LL_QSPI_SIMPLEX_TX
  *         @arg @ref LL_QSPI_SIMPLEX_RX
  *         @arg @ref LL_QSPI_READ_EEPROM
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_transfer_direction(qspi_regs_t *QSPIx, uint32_t transfer_direction)
{
    MODIFY_REG(QSPIx->CTRL0, QSPI_CTRL0_TMOD, transfer_direction);
}

/**
  * @brief  Get transfer direction mode
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | TMOD
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_FULL_DUPLEX
  *         @arg @ref LL_QSPI_SIMPLEX_TX
  *         @arg @ref LL_QSPI_SIMPLEX_RX
  *         @arg @ref LL_QSPI_READ_EEPROM
  */
__STATIC_INLINE uint32_t ll_qspi_get_transfer_direction(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->CTRL0, QSPI_CTRL0_TMOD));
}

/**
  * @brief  Set clock polarity
  * @note   This bit should not be changed when communication is ongoing.
  *         This bit is not used in SPI TI mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SCPOL
  *
  * @param  QSPIx QSPI instance
  * @param  clock_polarity This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_SCPOL_LOW
  *         @arg @ref LL_QSPI_SCPOL_HIGH
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_clock_polarity(qspi_regs_t *QSPIx, uint32_t clock_polarity)
{
    MODIFY_REG(QSPIx->CTRL0, QSPI_CTRL0_SCPOL, clock_polarity);
}

/**
  * @brief  Get clock polarity
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SCPOL
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_SCPOL_LOW
  *         @arg @ref LL_QSPI_SCPOL_HIGH
  */
__STATIC_INLINE uint32_t ll_qspi_get_clock_polarity(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->CTRL0, QSPI_CTRL0_SCPOL));
}

/**
  * @brief  Set clock phase
  * @note   This bit should not be changed when communication is ongoing.
  *         This bit is not used in SPI TI mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SCPHA
  *
  * @param  QSPIx QSPI instance
  * @param  clock_phase This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_SCPHA_1EDGE
  *         @arg @ref LL_QSPI_SCPHA_2EDGE
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_clock_phase(qspi_regs_t *QSPIx, uint32_t clock_phase)
{
    MODIFY_REG(QSPIx->CTRL0, QSPI_CTRL0_SCPHA, clock_phase);
}

/**
  * @brief  Get clock phase
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SCPHA
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_SCPHA_1EDGE
  *         @arg @ref LL_QSPI_SCPHA_2EDGE
  */
__STATIC_INLINE uint32_t ll_qspi_get_clock_phase(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->CTRL0, QSPI_CTRL0_SCPHA));
}

/**
  * @brief  Set serial protocol used
  * @note   This bit should be written only when SPI is disabled (QSPI_EN = 0) for correct operation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | FRF
  *
  * @param  QSPIx QSPI instance
  * @param  standard This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_PROTOCOL_MOTOROLA
  *         @arg @ref LL_QSPI_PROTOCOL_TI
  *         @arg @ref LL_QSPI_PROTOCOL_MICROWIRE
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_standard(qspi_regs_t *QSPIx, uint32_t standard)
{
    MODIFY_REG(QSPIx->CTRL0, QSPI_CTRL0_FRF, standard);
}

/**
  * @brief  Get serial protocol used
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | FRF
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_PROTOCOL_MOTOROLA
  *         @arg @ref LL_QSPI_PROTOCOL_TI
  *         @arg @ref LL_QSPI_PROTOCOL_MICROWIRE
  */
__STATIC_INLINE uint32_t ll_qspi_get_standard(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->CTRL0, QSPI_CTRL0_FRF));
}

/**
  * @brief  Set the number of data frames to be continuously received
  * @note   These bits should not be changed when communication is ongoing.
            This bits are effect when TMOD = 2b10 or 2b11.
            This bits are not effect in SPIS.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | NDF
  *
  * @param  QSPIx QSPI instance
  * @param  size This parameter can be one of the following values: 0 ~ 65535
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_receive_size(qspi_regs_t *QSPIx, uint32_t size)
{
    MODIFY_REG(QSPIx->CTRL1, QSPI_CTRL1_NDF, size);
}

#define ll_qspi_set_xfer_size      ll_qspi_set_receive_size

/**
  * @brief  Get the number of data frames to be continuously received
  * @note   These bits should not be changed when communication is ongoing.
            This bits are effect when TMOD = 2b10 or 2b11.
            This bits are not effect in SPIS.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | NDF
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values: 0 ~ 65535
  */
__STATIC_INLINE uint32_t ll_qspi_get_receive_size(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->CTRL1, QSPI_CTRL1_NDF));
}

/**
  * @brief  Enable SPI peripheral
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_EN | EN
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_enable(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->QSPI_EN, QSPI_SSI_EN);
}

/**
  * @brief  Disable SPI peripheral
  * @note   When disabling the SPI, follow the procedure described in the Reference Manual.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_EN | EN
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_disable(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->QSPI_EN, QSPI_SSI_EN);
}

/**
  * @brief  Check if SPI peripheral is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_EN | EN
  *
  * @param  QSPIx QSPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_qspi_is_enabled(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->QSPI_EN, QSPI_SSI_EN) == (QSPI_SSI_EN));
}

/**
  * @brief  Enable Handshake in Microwire mode
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MHS
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_enable_micro_handshake(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->MWC, QSPI_MWC_MHS);
}

/**
  * @brief  Disable Handshake in Microwire mode
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MHS
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_disable_micro_handshake(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->MWC, QSPI_MWC_MHS);
}

/**
  * @brief  Check if Handshake in Microwire mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MHS
  *
  * @param  QSPIx QSPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_qspi_is_enabled_micro_handshake(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->MWC, QSPI_MWC_MHS) == (QSPI_MWC_MHS));
}

/**
  * @brief  Set transfer direction mode in Microwire mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MDD
  *
  * @param  QSPIx QSPI instance
  * @param  transfer_direction This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_MICROWIRE_RX
  *         @arg @ref LL_QSPI_MICROWIRE_TX
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_micro_transfer_direction(qspi_regs_t *QSPIx, uint32_t transfer_direction)
{
    MODIFY_REG(QSPIx->MWC, QSPI_MWC_MDD, transfer_direction);
}

/**
  * @brief  Get transfer direction mode in Microwire mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MDD
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_MICROWIRE_RX
  *         @arg @ref LL_QSPI_MICROWIRE_TX
  */
__STATIC_INLINE uint32_t ll_qspi_get_micro_transfer_direction(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->MWC, QSPI_MWC_MDD));
}

/**
  * @brief  Set transfer mode in Microwire mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MWMOD
  *
  * @param  QSPIx QSPI instance
  * @param  transfer_mode This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_MICROWIRE_NON_SEQUENTIAL
  *         @arg @ref LL_QSPI_MICROWIRE_SEQUENTIAL
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_micro_transfer_mode(qspi_regs_t *QSPIx, uint32_t transfer_mode)
{
    MODIFY_REG(QSPIx->MWC, QSPI_MWC_MWMOD, transfer_mode);
}

/**
  * @brief  Get transfer mode in Microwire mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MWMOD
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_MICROWIRE_NON_SEQUENTIAL
  *         @arg @ref LL_QSPI_MICROWIRE_SEQUENTIAL
  */
__STATIC_INLINE uint32_t ll_qspi_get_micro_transfer_mode(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->MWC, QSPI_MWC_MWMOD));
}

/**
  * @brief  Enable slave select
  *
  *  Register|BitsName
  *  --------|--------
  *  SE | SLAVE1
  *  SE | SLAVE0
  *
  * @param  QSPIx QSPI instance
  * @param  ss This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_SLAVE1
  *         @arg @ref LL_QSPI_SLAVE0
  * @retval None
  */
__STATIC_INLINE void ll_qspi_enable_ss(qspi_regs_t *QSPIx, uint32_t ss)
{
    SET_BITS(QSPIx->SE, ss);
}

/**
  * @brief  Disable slave select
  *
  *  Register|BitsName
  *  --------|--------
  *  SE | SLAVE1
  *  SE | SLAVE0
  *
  * @param  QSPIx QSPI instance
  * @param  ss This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_SLAVE1
  *         @arg @ref LL_QSPI_SLAVE0
  * @retval None
  */
__STATIC_INLINE void ll_qspi_disable_ss(qspi_regs_t *QSPIx, uint32_t ss)
{
    CLEAR_BITS(QSPIx->SE, ss);
}

/**
  * @brief  Check if slave select is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  SE | SLAVE1
  *  SE | SLAVE0
  *
  * @param  QSPIx QSPI instance
  * @param  ss This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_SLAVE1
  *         @arg @ref LL_QSPI_SLAVE0
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_qspi_is_enabled_ss(qspi_regs_t *QSPIx, uint32_t ss)
{
    return (READ_BITS(QSPIx->SE, ss) == ss);
}

/**
  * @brief  Set baud rate prescaler
  * @note   These bits should not be changed when communication is ongoing. SPI BaudRate = fPCLK/Prescaler.
  *
  *  Register|BitsName
  *  --------|--------
  *  BAUD | SCKDIV
  *
  * @param  QSPIx QSPI instance
  * @param  baud_rate This parameter can be one even value between 2 and 65534, if the value is 0, the SCLK is disable.
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_baud_rate_prescaler(qspi_regs_t *QSPIx, uint32_t baud_rate)
{
    WRITE_REG(QSPIx->BAUD, baud_rate & QSPI_BAUD_SCKDIV);
}

/**
  * @brief  Get baud rate prescaler
  *
  *  Register|BitsName
  *  --------|--------
  *  BAUD | SCKDIV
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one even value between 2 and 65534.
  */
__STATIC_INLINE uint32_t ll_qspi_get_baud_rate_prescaler(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->BAUD, QSPI_BAUD_SCKDIV));
}

/**
  * @brief  Set threshold of TX transfer start
  *
  *  Register|BitsName
  *  --------|--------
  *  TXFTL | TXFTHR
  *
  * @param  QSPIx QSPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ (FIFO_DEPTH - 1)
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_tx_start_fifo_threshold(qspi_regs_t *QSPIx, uint32_t threshold)
{
    MODIFY_REG(QSPIx->TX_FTL, QSPI_TXFTHR_TFT, threshold << QSPI_TXFTHR_TFT_Pos);
}

/**
  * @brief  Get threshold of TX transfer start
  *
  *  Register|BitsName
  *  --------|--------
  *  TXFTL | TXFTHR
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values: 0 ~ (FIFO_DEPTH - 1)
  */
__STATIC_INLINE uint32_t ll_qspi_get_tx_start_fifo_threshold(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->TX_FTL, QSPI_TXFTHR_TFT) >> QSPI_TXFTHR_TFT_Pos);
}

/**
  * @brief  Set threshold of TXFIFO that triggers an TXE event
  *
  *  Register|BitsName
  *  --------|--------
  *  TXFTL | TFT
  *
  * @param  QSPIx QSPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ (FIFO_DEPTH - 1)
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_tx_fifo_threshold(qspi_regs_t *QSPIx, uint32_t threshold)
{
    MODIFY_REG(QSPIx->TX_FTL, QSPI_TXFTL_TFT, threshold << QSPI_TXFTL_TFT_Pos);
}

/**
  * @brief  Get threshold of TXFIFO that triggers an TXE event
  *
  *  Register|BitsName
  *  --------|--------
  *  TXFTL | TFT
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values: 0 ~ (FIFO_DEPTH - 1)
  */
__STATIC_INLINE uint32_t ll_qspi_get_tx_fifo_threshold(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->TX_FTL, QSPI_TXFTL_TFT) >> QSPI_TXFTL_TFT_Pos);
}

/**
  * @brief  Set threshold of RXFIFO that triggers an RXNE event
  *
  *  Register|BitsName
  *  --------|--------
  *  RXFTL | RFT
  *
  * @param  QSPIx QSPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ (FIFO_DEPTH - 1)
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_rx_fifo_threshold(qspi_regs_t *QSPIx, uint32_t threshold)
{
    WRITE_REG(QSPIx->RX_FTL, threshold);
}

/**
  * @brief  Get threshold of RXFIFO that triggers an RXNE event
  *
  *  Register|BitsName
  *  --------|--------
  *  RXFTL | RFT
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values: 0 ~ (FIFO_DEPTH - 1)
  */
__STATIC_INLINE uint32_t ll_qspi_get_rx_fifo_threshold(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->RX_FTL, QSPI_RXFTL_RFT));
}

/**
  * @brief  Get FIFO Transmission Level
  *
  *  Register|BitsName
  *  --------|--------
  *  TXFL | TXTFL
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values: 0 ~ FIFO_DEPTH
  */
__STATIC_INLINE uint32_t ll_qspi_get_tx_fifo_level(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->TX_FL, QSPI_TXFL_TXTFL));
}

/**
  * @brief  Get FIFO reception Level
  *
  *  Register|BitsName
  *  --------|--------
  *  RXFL | RXTFL
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values: 0 ~ FIFO_DEPTH
  */
__STATIC_INLINE uint32_t ll_qspi_get_rx_fifo_level(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->RX_FL, QSPI_RXFL_RXTFL));
}

/**
  * @brief  Get ID code
  *
  *  Register|BitsName
  *  --------|--------
  *  IDCODE | ID
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value is const.
  */
__STATIC_INLINE uint32_t ll_qspi_get_id_code(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->ID, QSPI_IDCODE_ID));
}

/**
  * @brief  Get IP version
  *
  *  Register|BitsName
  *  --------|--------
  *  COMP | VERSION
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value is const.
  */
__STATIC_INLINE uint32_t ll_qspi_get_version(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->VERSION_ID, QSPI_COMP_VERSION));
}

/** @} */

/** @defgroup LL_QSPI_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable interrupt
  * @note   This bit controls the generation of an interrupt when an event occurs.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTMASK | INTMASK
  *
  * @param  QSPIx QSPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_IM_SPITE
  *         @arg @ref LL_QSPI_IM_TXU
  *         @arg @ref LL_QSPI_IM_XRXO
  *         @arg @ref LL_QSPI_IM_MST(not effect in SPIS)
  *         @arg @ref LL_QSPI_IM_RXF
  *         @arg @ref LL_QSPI_IM_RXO
  *         @arg @ref LL_QSPI_IM_RXU
  *         @arg @ref LL_QSPI_IM_TXO
  *         @arg @ref LL_QSPI_IM_TXE
  * @retval None
  */
__STATIC_INLINE void ll_qspi_enable_it(qspi_regs_t *QSPIx, uint32_t mask)
{
    SET_BITS(QSPIx->INTMASK, mask);
}

/**
  * @brief  Disable interrupt
  * @note   This bit controls the generation of an interrupt when an event occurs.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTMASK | INTMASK
  *
  * @param  QSPIx QSPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_IM_SPITE
  *         @arg @ref LL_QSPI_IM_TXU
  *         @arg @ref LL_QSPI_IM_XRXO
  *         @arg @ref LL_QSPI_IM_MST(not effect in SPIS)
  *         @arg @ref LL_QSPI_IM_RXF
  *         @arg @ref LL_QSPI_IM_RXO
  *         @arg @ref LL_QSPI_IM_RXU
  *         @arg @ref LL_QSPI_IM_TXO
  *         @arg @ref LL_QSPI_IM_TXE
  * @retval None
  */
__STATIC_INLINE void ll_qspi_disable_it(qspi_regs_t *QSPIx, uint32_t mask)
{
    CLEAR_BITS(QSPIx->INTMASK, mask);
}

/**
  * @brief  Check if interrupt is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  INTMASK | INTMASK
  *
  * @param  QSPIx QSPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_IM_SPITE
  *         @arg @ref LL_QSPI_IM_TXU
  *         @arg @ref LL_QSPI_IM_XRXO
  *         @arg @ref LL_QSPI_IM_MST(not effect in SPIS)
  *         @arg @ref LL_QSPI_IM_RXF
  *         @arg @ref LL_QSPI_IM_RXO
  *         @arg @ref LL_QSPI_IM_RXU
  *         @arg @ref LL_QSPI_IM_TXO
  *         @arg @ref LL_QSPI_IM_TXE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_qspi_is_enabled_it(qspi_regs_t *QSPIx, uint32_t mask)
{
    return (READ_BITS(QSPIx->INTMASK, mask) == mask);
}

/** @} */

/** @defgroup LL_QSPI_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Get SPI status
  *
  *  Register|BitsName
  *  --------|--------
  *  STAT | STAT
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_QSPI_SR_DCOL(no effect in SPIS)
  *         @arg @ref LL_QSPI_SR_TXE
  *         @arg @ref LL_QSPI_SR_RFF
  *         @arg @ref LL_QSPI_SR_RFNE
  *         @arg @ref LL_QSPI_SR_TFE
  *         @arg @ref LL_QSPI_SR_TFNF
  *         @arg @ref LL_QSPI_SR_BUSY
  */
__STATIC_INLINE uint32_t ll_qspi_get_status(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_REG(QSPIx->STAT));
}

/**
  * @brief  Check active flag
  *
  *  Register|BitsName
  *  --------|--------
  *  STAT | DCOL
  *  STAT | TXE
  *  STAT | RFF
  *  STAT | RFNE
  *  STAT | TFE
  *  STAT | TFNF
  *  STAT | BUSY
  *
  * @param  QSPIx QSPI instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_SR_DCOL(no effect in SPIS)
  *         @arg @ref LL_QSPI_SR_TXE
  *         @arg @ref LL_QSPI_SR_RFF
  *         @arg @ref LL_QSPI_SR_RFNE
  *         @arg @ref LL_QSPI_SR_TFE
  *         @arg @ref LL_QSPI_SR_TFNF
  *         @arg @ref LL_QSPI_SR_BUSY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_qspi_is_active_flag(qspi_regs_t *QSPIx, uint32_t flag)
{
    return (READ_BITS(QSPIx->STAT, flag) == (flag));
}

/**
  * @brief  Get SPI interrupt flags
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | INTSTAT
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_QSPI_IS_SPITE
  *         @arg @ref LL_QSPI_IS_TXU
  *         @arg @ref LL_QSPI_IS_XRXO
  *         @arg @ref LL_QSPI_IS_MST(no effect in SPIS)
  *         @arg @ref LL_QSPI_IS_RXF
  *         @arg @ref LL_QSPI_IS_RXO
  *         @arg @ref LL_QSPI_IS_RXU
  *         @arg @ref LL_QSPI_IS_TXO
  *         @arg @ref LL_QSPI_IS_TXE
  */
__STATIC_INLINE uint32_t ll_qspi_get_it_flag(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_REG(QSPIx->INTSTAT));
}

/**
  * @brief  Check interrupt flag
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | MSTIS
  *  INTSTAT | RXFIS
  *  INTSTAT | RXOIS
  *  INTSTAT | RXUIS
  *  INTSTAT | TXOIS
  *  INTSTAT | TXEIS
  *
  * @param  QSPIx QSPI instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_IS_SPITE
  *         @arg @ref LL_QSPI_IS_TXU
  *         @arg @ref LL_QSPI_IS_XRXO
  *         @arg @ref LL_QSPI_IS_MST(no effect in SPIS)
  *         @arg @ref LL_QSPI_IS_RXF
  *         @arg @ref LL_QSPI_IS_RXO
  *         @arg @ref LL_QSPI_IS_RXU
  *         @arg @ref LL_QSPI_IS_TXO
  *         @arg @ref LL_QSPI_IS_TXE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_qspi_is_it_flag(qspi_regs_t *QSPIx, uint32_t flag)
{
    return (READ_BITS(QSPIx->INTSTAT, flag) == flag);
}

/**
  * @brief  Get SPI raw interrupt flags
  *
  *  Register|BitsName
  *  --------|--------
  *  RAW_INTSTAT | RAW_INTSTAT
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_QSPI_RIS_SPITE
  *         @arg @ref LL_QSPI_RIS_TXU
  *         @arg @ref LL_QSPI_RIS_XRXO
  *         @arg @ref LL_QSPI_RIS_MST(no effect in SPIS)
  *         @arg @ref LL_QSPI_RIS_RXF
  *         @arg @ref LL_QSPI_RIS_RXO
  *         @arg @ref LL_QSPI_RIS_RXU
  *         @arg @ref LL_QSPI_RIS_TXO
  *         @arg @ref LL_QSPI_RIS_TXE
  */
__STATIC_INLINE uint32_t ll_qspi_get_raw_if_flag(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_REG(QSPIx->RAW_INTSTAT));
}

/**
  * @brief  Clear transmit FIFO overflow error flag
  * @note   Clearing this flag is done by reading TXOIC register
  *
  *  Register|BitsName
  *  --------|--------
  *  TXOIC | TXOIC
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_clear_flag_txo(qspi_regs_t *QSPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = QSPIx->TXOIC;
    (void) tmpreg;
}

/**
  * @brief  Clear receive FIFO overflow error flag
  * @note   Clearing this flag is done by reading RXOIC register
  *
  *  Register|BitsName
  *  --------|--------
  *  RXOIC | RXOIC
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_clear_flag_rxo(qspi_regs_t *QSPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = QSPIx->RXOIC;
    (void) tmpreg;
}

/**
  * @brief  Clear receive FIFO underflow error flag
  * @note   Clearing this flag is done by reading RXUIC register
  *
  *  Register|BitsName
  *  --------|--------
  *  RXUIC | RXUIC
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_clear_flag_rxu(qspi_regs_t *QSPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = QSPIx->RXUIC;
    (void) tmpreg;
}

/**
  * @brief  Clear multi-master error flag
  * @note   Clearing this flag is done by reading MSTIC register
  *
  *  Register|BitsName
  *  --------|--------
  *  MSTIC | MSTIC
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_clear_flag_mst(qspi_regs_t *QSPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = QSPIx->MSTIC;
    (void) tmpreg;
}

/**
  * @brief  Clear XIP receive FIFO overflow flag
  * @note   Clearing this flag is done by reading XRXOIC register
  *
  *  Register|BitsName
  *  --------|--------
  *  XRXOIC | XRXOIC
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_clear_flag_xrxo(qspi_regs_t *QSPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = QSPIx->XIP_RXOICR;
    (void) tmpreg;
}

/**
  * @brief  Clear all error(txo,rxu,rxo,mst) flag
  * @note   Clearing this flag is done by reading INTCLR register
  *
  *  Register|BitsName
  *  --------|--------
  *  INTCLR | INTCLR
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_clear_flag_all(qspi_regs_t *QSPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = QSPIx->INTCLR;
    (void) tmpreg;
}

/** @} */

/** @defgroup LL_QSPI_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Enable DMA Tx
  *
  *  Register|BitsName
  *  --------|--------
  *  DMAC | TDMAE
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_enable_dma_req_tx(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->DMAC, QSPI_DMAC_TDMAE);
}

/**
  * @brief  Disable DMA Tx
  *
  *  Register|BitsName
  *  --------|--------
  *  DMAC | TDMAE
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_disable_dma_req_tx(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->DMAC, QSPI_DMAC_TDMAE);
}

/**
  * @brief  Check if DMA Tx is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  DMAC | TDMAE
  *
  * @param  QSPIx QSPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_qspi_is_enabled_dma_req_tx(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->DMAC, QSPI_DMAC_TDMAE) == (QSPI_DMAC_TDMAE));
}

/**
  * @brief  Enable DMA Rx
  *
  *  Register|BitsName
  *  --------|--------
  *  DMAC | RDMAE
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_enable_dma_req_rx(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->DMAC, QSPI_DMAC_RDMAE);
}

/**
  * @brief  Disable DMA Rx
  *
  *  Register|BitsName
  *  --------|--------
  *  DMAC | RDMAE
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_disable_dma_req_rx(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->DMAC, QSPI_DMAC_RDMAE);
}

/**
  * @brief  Check if DMA Rx is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  DMAC | RDMAE
  *
  * @param  QSPIx QSPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_qspi_is_enabled_dma_req_rx(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->DMAC, QSPI_DMAC_RDMAE) == (QSPI_DMAC_RDMAE));
}

/**
  * @brief  Set threshold of TXFIFO that triggers an DMA Tx request event
  *
  *  Register|BitsName
  *  --------|--------
  *  DMATDL | DMATDL
  *
  * @param  QSPIx QSPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_dma_tx_fifo_threshold(qspi_regs_t *QSPIx, uint32_t threshold)
{
    WRITE_REG(QSPIx->DMA_TDL, threshold);
}

/**
  * @brief  Get threshold of TXFIFO that triggers an DMA Tx request event
  *
  *  Register|BitsName
  *  --------|--------
  *  DMATDL | DMATDL
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values: 0 ~ 7
  */
__STATIC_INLINE uint32_t ll_qspi_get_dma_tx_fifo_threshold(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->DMA_TDL, QSPI_DMATDL_DMATDL));
}

/**
  * @brief  Set threshold of RXFIFO that triggers an DMA Rx request event
  *
  *  Register|BitsName
  *  --------|--------
  *  DMARDL | DMARDL
  *
  * @param  QSPIx QSPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_dma_rx_fifo_threshold(qspi_regs_t *QSPIx, uint32_t threshold)
{
    WRITE_REG(QSPIx->DMA_RDL, threshold);
}

/**
  * @brief  Get threshold of RXFIFO that triggers an DMA Rx request event
  *
  *  Register|BitsName
  *  --------|--------
  *  DMARDL | DMARDL
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values: 0 ~ 7
  */
__STATIC_INLINE uint32_t ll_qspi_get_dma_rx_fifo_threshold(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->DMA_RDL, QSPI_DMARDL_DMARDL));
}

/** @} */

/** @defgroup LL_QSPI_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Write 8-Bits in the data register
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  QSPIx QSPI instance
  * @param  tx_data Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void ll_qspi_transmit_data8(qspi_regs_t *QSPIx, uint8_t tx_data)
{
    *((__IOM uint8_t *)&QSPIx->DATA) = tx_data;
}

/**
  * @brief  Write 16-Bits in the data register
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  QSPIx QSPI instance
  * @param  tx_data Value between Min_Data=0x0000 and Max_Data=0xFFFF
  * @retval None
  */
__STATIC_INLINE void ll_qspi_transmit_data16(qspi_regs_t *QSPIx, uint16_t tx_data)
{
    *((__IOM uint16_t *)&QSPIx->DATA) = tx_data;
}

/**
  * @brief  Write 32-Bits in the data register
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  QSPIx QSPI instance
  * @param  tx_data Value between Min_Data=0x00000000 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_qspi_transmit_data32(qspi_regs_t *QSPIx, uint32_t tx_data)
{
    *((__IOM uint32_t *)&QSPIx->DATA) = tx_data;
}

/**
  * @brief  Read 8-Bits in the data register
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  QSPIx QSPI instance
  * @retval Rerturned Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t ll_qspi_receive_data8(qspi_regs_t *QSPIx)
{
    return (uint8_t)(READ_REG(QSPIx->DATA));
}

/**
  * @brief  Read 16-Bits in the data register
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  QSPIx QSPI instance
  * @retval Returned Value between Min_Data=0x0000 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint16_t ll_qspi_receive_data16(qspi_regs_t *QSPIx)
{
    return (uint16_t)(READ_REG(QSPIx->DATA));
}

/**
  * @brief  Read 32-Bits in the data register
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  QSPIx QSPI instance
  * @retval Returned Value between Min_Data=0x00000000 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_qspi_receive_data32(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_REG(QSPIx->DATA));
}

/**
  * @brief  Set the RX sample edge
  *
  *  Register|BitsName
  *  --------|--------
  *  RX_SAMPLE_DELAY | SE
  *
  * @param  QSPIx QSPI instance
  * @param  edge - @ ref LL_QSPI_RX_SAMPLE_POSITIVE_EDGE
  *                @ ref LL_QSPI_RX_SAMPLE_NEGATIVE_EDGE
  * @retval none
  */
__STATIC_INLINE void ll_qspi_set_rx_sample_edge(qspi_regs_t *QSPIx, uint32_t edge)
{
    MODIFY_REG(QSPIx->RX_SAMPLE_DLY, QSPI_RX_SAMPLE_EDGE, edge << QSPI_RX_SAMPLE_EDGE_Pos);
}

/**
  * @brief  Get the RX sample edge
  *
  *  Register|BitsName
  *  --------|--------
  *  RX_SAMPLE_DELAY | SE
  *
  * @param  QSPIx QSPI instance
  * @retval edge - @ ref LL_QSPI_RX_SAMPLE_POSITIVE_EDGE
  *                @ ref LL_QSPI_RX_SAMPLE_NEGATIVE_EDGE
  */
__STATIC_INLINE uint32_t ll_qspi_get_rx_sample_edge(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->RX_SAMPLE_DLY, QSPI_RX_SAMPLE_EDGE) >> QSPI_RX_SAMPLE_EDGE_Pos);
}

/**
  * @brief  Set Rx sample delay
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  RX_SAMPLEDLY | RX_SAMPLEDLY
  *
  * @param  QSPIx QSPI instance
  * @param  delay This parameter can be one of the following values: 0 ~ 256
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_rx_sample_delay(qspi_regs_t *QSPIx, uint32_t delay)
{
    MODIFY_REG(QSPIx->RX_SAMPLE_DLY, QSPI_RX_SAMPLEDLY, delay);
}

/**
  * @brief  Get Rx sample delay
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  RX_SAMPLEDLY | RX_SAMPLEDLY
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values: 0 ~ 256
  */
__STATIC_INLINE uint32_t ll_qspi_get_rx_sample_delay(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->RX_SAMPLE_DLY, QSPI_RX_SAMPLEDLY));
}

/**
  * @brief  Enable the clock stretch feature for Enhanced SPI
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SCTRL0 | CLK_STRETCH_EN
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_enable_clk_stretch(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->SPI_CTRL0, QSPI_SCTRL0_CLK_STRETCH_EN);
}

/**
  * @brief  Disable the clock stretch feature for Enhanced SPI
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SCTRL0 | CLK_STRETCH_EN
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_disable_clk_stretch(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->SPI_CTRL0, QSPI_SCTRL0_CLK_STRETCH_EN);
}

/**
  * @brief  Check if the clock stretch feature is enabled or not for Enhanced SPI
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SCTRL0 | CLK_STRETCH_EN
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE uint32_t ll_qspi_is_enabled_clk_stretch(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->SPI_CTRL0, QSPI_SCTRL0_CLK_STRETCH_EN) == (QSPI_SCTRL0_CLK_STRETCH_EN));
}

/**
  * @brief  Set number of wait cycles in Dual/Quad SPI mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SCTRL0 | WAITCYCLES
  *
  * @param  QSPIx QSPI instance
  * @param  wait_cycles This parameter can be one of the following values: 0 ~ 31
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_wait_cycles(qspi_regs_t *QSPIx, uint32_t wait_cycles)
{
    MODIFY_REG(QSPIx->SPI_CTRL0, QSPI_SCTRL0_WAITCYCLES, wait_cycles << QSPI_SCTRL0_WAITCYCLES_Pos);
}

/**
  * @brief  Get number of wait cycles in Dual/Quad SPI mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SCTRL0 | WAITCYCLES
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values: 0 ~ 31
  */
__STATIC_INLINE uint32_t ll_qspi_get_wait_cycles(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->SPI_CTRL0, QSPI_SCTRL0_WAITCYCLES) >> QSPI_SCTRL0_WAITCYCLES_Pos);
}

/**
  * @brief  Set Dual/Quad SPI mode instruction length in bits
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SCTRL0 | INSTL
  *
  * @param  QSPIx QSPI instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_INSTSIZE_0BIT
  *         @arg @ref LL_QSPI_INSTSIZE_4BIT
  *         @arg @ref LL_QSPI_INSTSIZE_8BIT
  *         @arg @ref LL_QSPI_INSTSIZE_16BIT
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_instruction_size(qspi_regs_t *QSPIx, uint32_t size)
{
    MODIFY_REG(QSPIx->SPI_CTRL0, QSPI_SCTRL0_INSTL, size);
}

/**
  * @brief  Get Dual/Quad SPI mode instruction length in bits
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SCTRL0 | INSTL
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_INSTSIZE_0BIT
  *         @arg @ref LL_QSPI_INSTSIZE_4BIT
  *         @arg @ref LL_QSPI_INSTSIZE_8BIT
  *         @arg @ref LL_QSPI_INSTSIZE_16BIT
  */
__STATIC_INLINE uint32_t ll_qspi_get_instruction_size(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->SPI_CTRL0, QSPI_SCTRL0_INSTL));
}

/**
  * @brief  Set Dual/Quad SPI mode address length in bits
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SCTRL0 | ADDRL
  *
  * @param  QSPIx QSPI instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_ADDRSIZE_0BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_4BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_8BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_12BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_16BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_20BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_24BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_28BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_32BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_36BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_40BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_44BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_48BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_52BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_56BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_60BIT
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_address_size(qspi_regs_t *QSPIx, uint32_t size)
{
    MODIFY_REG(QSPIx->SPI_CTRL0, QSPI_SCTRL0_ADDRL, size);
}

/**
  * @brief  Get Dual/Quad SPI mode address length in bits
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SCTRL0 | ADDRL
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_ADDRSIZE_0BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_4BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_8BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_12BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_16BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_20BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_24BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_28BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_32BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_36BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_40BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_44BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_48BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_52BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_56BIT
  *         @arg @ref LL_QSPI_ADDRSIZE_60BIT
  */
__STATIC_INLINE uint32_t ll_qspi_get_address_size(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->SPI_CTRL0, QSPI_SCTRL0_ADDRL));
}

/**
  * @brief  Set Dual/Quad SPI mode address and instruction transfer format
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SCTRL0 | TRANSTYPE
  *
  * @param  QSPIx QSPI instance
  * @param  format This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_INST_ADDR_ALL_IN_SPI
  *         @arg @ref LL_QSPI_INST_IN_SPI_ADDR_IN_SPIFRF
  *         @arg @ref LL_QSPI_INST_ADDR_ALL_IN_SPIFRF
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_add_inst_transfer_format(qspi_regs_t *QSPIx, uint32_t format)
{
    MODIFY_REG(QSPIx->SPI_CTRL0, QSPI_SCTRL0_TRANSTYPE, format);
}

/**
  * @brief  Get Dual/Quad SPI mode address and instruction transfer format
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SCTRL0 | TRANSTYPE
  *
  * @param  QSPIx QSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_INST_ADDR_ALL_IN_SPI
  *         @arg @ref LL_QSPI_INST_IN_SPI_ADDR_IN_SPIFRF
  *         @arg @ref LL_QSPI_INST_ADDR_ALL_IN_SPIFRF
  */
__STATIC_INLINE uint32_t ll_qspi_get_addr_inst_transfer_format(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->SPI_CTRL0, QSPI_SCTRL0_TRANSTYPE));
}



/**
  * @brief  Enable the mode bits phase for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | MD_BITS_EN
  *
  * @param  QSPIx - QSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_enable_xip_mode_bits(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_MD_BIT_EN);
}

/**
  * @brief  Disable the mode bits phase for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | MD_BITS_EN
  *
  * @param  QSPIx - QSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_disable_xip_mode_bits(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_MD_BIT_EN);
}

/**
  * @brief  Check if the mode bits phase is enabled or not for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | MD_BITS_EN
  *
  * @param  QSPIx - QSPI instance
  * @retval TRUE/FALSE
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_is_enabled_xip_mode_bits(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_MD_BIT_EN) == (QSPI_XCTRL_MD_BIT_EN));
}

/**
  * @brief  Set the length of mode bits phase for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | XIP_MBL
  *
  * @param  QSPIx - QSPI instance
  * @param  mbl   - @ref LL_QSPI_CONCURRENT_XIP_MBL_2
  *                 @ref LL_QSPI_CONCURRENT_XIP_MBL_4
  *                 @ref LL_QSPI_CONCURRENT_XIP_MBL_8
  *                 @ref LL_QSPI_CONCURRENT_XIP_MBL_16
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_mode_bits_length(qspi_regs_t *QSPIx, uint32_t mbl)
{
    MODIFY_REG(QSPIx->XIP_CTRL, QSPI_XCTRL_XIP_MBL, mbl << QSPI_XCTRL_XIP_MBL_Pos);
}

/**
  * @brief  Get the length of mode bits phase for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | XIP_MBL
  *
  * @param  QSPIx - QSPI instance
  * @retval mbl   - @ref LL_QSPI_CONCURRENT_XIP_MBL_2
  *                 @ref LL_QSPI_CONCURRENT_XIP_MBL_4
  *                 @ref LL_QSPI_CONCURRENT_XIP_MBL_8
  *                 @ref LL_QSPI_CONCURRENT_XIP_MBL_16
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_mode_bits_length(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_XIP_MBL) >> QSPI_XCTRL_XIP_MBL_Pos);
}

/**
  * @brief  set the mode phase (sent after address phase) value in xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_MODE_BITS | XIP_MD_BITS
  *
  * @param  QSPIx - QSPI instance
  * @param  mode  - mode value, [0 ~ 0xFFFF]
  * @retval None
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_mode_bits_data(qspi_regs_t *QSPIx, uint32_t mode)
{
    MODIFY_REG(QSPIx->XIP_MODE_BITS, QSPI_XIP_MODE_BITS, mode << QSPI_XIP_MODE_BITS_Pos);
}

/**
  * @brief  get the mode phase (sent after address phase) value in xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_MODE_BITS | XIP_MD_BITS
  *
  * @param  QSPIx - QSPI instance
  * @retval mode value, [0 ~ 0xFFFF]
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_mode_bits_data(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_MODE_BITS, QSPI_XIP_MODE_BITS) >> QSPI_XIP_MODE_BITS_Pos);
}

/**
  * @brief  set the ahb-incr transfer instruction in xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_INCR_INST | INCR_INST
  *
  * @param  QSPIx - QSPI instance
  * @param  inst  - instruction op-code, [0 ~ 0xFFFF]
  * @retval None
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_incr_inst(qspi_regs_t *QSPIx, uint32_t inst)
{
    MODIFY_REG(QSPIx->XIP_INCR_INST, QSPI_XIP_INCR_INST, inst << QSPI_XIP_INCR_INST_Pos);
}

/**
  * @brief  get the ahb-incr transfer instruction in xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_INCR_INST | INCR_INST
  *
  * @param  QSPIx - QSPI instance
  * @retval inst  - instruction op-code, [0 ~ 0xFFFF]
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_incr_inst(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_INCR_INST, QSPI_XIP_INCR_INST) >> QSPI_XIP_INCR_INST_Pos);
}

/**
  * @brief  set the ahb-wrap transfer instruction in xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WRAP_INST | WRAP_INST
  *
  * @param  QSPIx - QSPI instance
  * @param  inst  - wrap instruction op-code, [0 ~ 0xFFFF]
  * @retval None
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_wrap_inst(qspi_regs_t *QSPIx, uint32_t inst)
{
    MODIFY_REG(QSPIx->XIP_WRAP_INST, QSPI_XIP_WRAP_INST, inst << QSPI_XIP_WRAP_INST_Pos);
}

/**
  * @brief  get the ahb-wrap transfer instruction in xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WRAP_INST | WRAP_INST
  *
  * @param  QSPIx - QSPI instance
  * @retval inst  - instruction op-code, [0 ~ 0xFFFF]
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_wrap_inst(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_WRAP_INST, QSPI_XIP_WRAP_INST) >> QSPI_XIP_WRAP_INST_Pos);
}

/**
  * @brief  Enable the slave in xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_SER | SER
  *
  * @param  QSPIx - QSPI instance
  * @param  ss  - @ref LL_QSPI_CONCURRENT_XIP_SLAVE0
  * @retval None
  */
__STATIC_INLINE void ll_qspi_concurrent_enable_xip_ss(qspi_regs_t *QSPIx, uint32_t ss)
{
    SET_BITS(QSPIx->XIP_SER, ss);
}

/**
  * @brief  Disable the slave in xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_SER | SER
  *
  * @param  QSPIx - QSPI instance
  * @param  ss  - @ref LL_QSPI_CONCURRENT_XIP_SLAVE0
  * @retval None
  */
__STATIC_INLINE void ll_qspi_concurrent_disable_xip_ss(qspi_regs_t *QSPIx, uint32_t ss)
{
    CLEAR_BITS(QSPIx->XIP_SER, ss);
}

/**
  * @brief  Check if the slave is enabled or not for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_SER | SER
  *
  * @param  QSPIx - QSPI instance
  * @param  ss  - @ref LL_QSPI_CONCURRENT_XIP_SLAVE0
  * @retval None
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_is_enabled_xip_ss(qspi_regs_t *QSPIx, uint32_t ss)
{
    return (READ_BITS(QSPIx->XIP_SER, ss) == ss);
}

/**
  * @brief  Set time out count for continuous transfer for xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CNT_TIME_OUT | XTOC
  *
  * @param  QSPIx - QSPI instance
  * @param  xtoc  - time out counter value in terms of hclk [0  ~ 0xFF]
  * @retval None
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_toc(qspi_regs_t *QSPIx, uint32_t xtoc)
{
    MODIFY_REG(QSPIx->XIP_CNT_TIME_OUT, QSPI_XIP_TOCNT, xtoc << QSPI_XIP_TOCNT_Pos);
}

/**
  * @brief  Get time out count for continuous transfer for xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CNT_TIME_OUT | XTOC
  *
  * @param  QSPIx - QSPI instance
  * @retval xtoc  - time out counter value in terms of hclk [0  ~ 0xFF]
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_toc(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_CNT_TIME_OUT, QSPI_XIP_TOCNT) >> QSPI_XIP_TOCNT_Pos);
}

/**
  * @brief  Enable the pre-fetch feature for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | XIP_PREFETCH_EN
  *
  * @param  QSPIx - QSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_enable_xip_prefetch(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_XIP_PREFETCH_EN);
}

/**
  * @brief  Disable the pre-fetch feature for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | XIP_PREFETCH_EN
  *
  * @param  QSPIx - QSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_disable_xip_prefetch(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_XIP_PREFETCH_EN);
}

/**
  * @brief  check if the pre-fetch feature is enabled or not for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | XIP_PREFETCH_EN
  *
  * @param  QSPIx - QSPI instance
  * @retval TRUE/FALSE
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_is_enabled_xip_prefetch(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_XIP_PREFETCH_EN) == (QSPI_XCTRL_XIP_PREFETCH_EN));
}

/**
  * @brief  Enable the continuous transfer feature for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | CONT_XFER_EN
  *
  * @param  QSPIx - QSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_enable_xip_continuous_xfer(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_CONT_XFER_EN);
}

/**
  * @brief  Disable the continuous transfer feature for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | CONT_XFER_EN
  *
  * @param  QSPIx - QSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_disable_xip_continuous_xfer(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_CONT_XFER_EN);
}

/**
  * @brief  Check if the continuous transfer feature is enabled or not for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | CONT_XFER_EN
  *
  * @param  QSPIx - QSPI instance
  * @retval TRUE/FALSE
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_is_enabled_xip_continuous_xfer(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_CONT_XFER_EN) == (QSPI_XCTRL_CONT_XFER_EN));
}

/**
  * @brief  Enable the instruction phase for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | INST_EN
  *
  * @param  QSPIx - QSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_enable_xip_instruction(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_INST_EN);
}

/**
  * @brief  Disable the instruction phase for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | INST_EN
  *
  * @param  QSPIx - QSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_disable_xip_instruction(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_INST_EN);
}

/**
  * @brief  Check if the instruction phase is enabled or not for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | INST_EN
  *
  * @param  QSPIx - QSPI instance
  * @retval TRUE/FALSE
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_is_enabled_xip_instruction(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_INST_EN) == (QSPI_XCTRL_INST_EN));
}

/**
  * @brief  Set the instruction size for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | INST_L
  *
  * @param  QSPIx - QSPI instance
  * @param  inst_size - @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_0BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_4BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_8BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_16BIT
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_instruction_size(qspi_regs_t *QSPIx, uint32_t inst_size)
{
    MODIFY_REG(QSPIx->XIP_CTRL, QSPI_XCTRL_INSTL, inst_size << QSPI_XCTRL_INSTL_Pos);
}

/**
  * @brief  Get the instruction size for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | INST_L
  *
  * @param  QSPIx - QSPI instance
  * @retval inst_size - @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_0BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_4BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_8BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_16BIT
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_instruction_size(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_INSTL) >> QSPI_XCTRL_INSTL_Pos);
}


/**
  * @brief  Enable the hardcoded DFS feature for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | DFS_HC
  *
  * @param  QSPIx - QSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_enable_xip_dfs_hardcode(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_DFS_HC);
}

/**
  * @brief  Disable the hardcoded DFS feature for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | DFS_HC
  *
  * @param  QSPIx - QSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_disable_xip_dfs_hardcode(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_DFS_HC);
}

/**
  * @brief  Check if the hardcoded DFS feature is enabled or not for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | DFS_HC
  *
  * @param  QSPIx - QSPI instance
  * @retval TRUE/FALSE
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_is_enabled_xip_dfs_hardcode(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_DFS_HC) == (QSPI_XCTRL_DFS_HC));
}

/**
  * @brief  Set the wait(also called dummy) cycles for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | WAIT_CYCLES
  *
  * @param  QSPIx - QSPI instance
  * @param  wait_cycles - 0 ~ 31
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_wait_cycles(qspi_regs_t *QSPIx, uint32_t wait_cycles)
{
    MODIFY_REG(QSPIx->XIP_CTRL, QSPI_XCTRL_WAITCYCLES, wait_cycles << QSPI_XCTRL_WAITCYCLES_Pos);
}

/**
  * @brief  Get the wait(also called dummy) cycles for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | WAIT_CYCLES
  *
  * @param  QSPIx - QSPI instance
  * @retval wait_cycles - 0 ~ 31
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_wait_cycles(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_WAITCYCLES) >> QSPI_XCTRL_WAITCYCLES_Pos);
}

/**
  * @brief  Set the address size for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | ADDR_L
  *
  * @param  QSPIx - QSPI instance
  * @param  addr_size - @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_0BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_4BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_8BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_12BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_16BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_20BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_28BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_32BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_36BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_40BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_44BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_48BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_52BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_56BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_60BIT
  *
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_address_size(qspi_regs_t *QSPIx, uint32_t addr_size)
{
    MODIFY_REG(QSPIx->XIP_CTRL, QSPI_XCTRL_ADDRL, addr_size << QSPI_XCTRL_ADDRL_Pos);
}

/**
  * @brief  Get the address size for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | ADDR_L
  *
  * @param  QSPIx - QSPI instance
  * @retval addr_size - @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_0BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_4BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_8BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_12BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_16BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_20BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_28BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_32BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_36BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_40BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_44BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_48BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_52BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_56BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_60BIT
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_address_size(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_ADDRL) >> QSPI_XCTRL_ADDRL_Pos);
}

/**
  * @brief  Set the transfer format of inst & address for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | TRANS_TYPE
  *
  * @param  QSPIx - QSPI instance
  * @param  format - @ref LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPI
  *                  @ref LL_QSPI_CONCURRENT_XIP_INST_IN_SPI_ADDR_IN_SPIFRF
  *                  @ref LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF
  *
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_inst_addr_transfer_format(qspi_regs_t *QSPIx, uint32_t format)
{
    MODIFY_REG(QSPIx->XIP_CTRL, QSPI_XCTRL_TRANSTYPE, format << QSPI_XCTRL_TRANSTYPE_Pos);
}

/**
  * @brief  Get the transfer format of inst & address for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | TRANS_TYPE
  *
  * @param  QSPIx - QSPI instance
  * @retval format - @ref LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPI
  *                  @ref LL_QSPI_CONCURRENT_XIP_INST_IN_SPI_ADDR_IN_SPIFRF
  *                  @ref LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_addr_inst_transfer_format(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_TRANSTYPE) >> QSPI_XCTRL_TRANSTYPE_Pos);
}

/**
  * @brief  Set the QSPI frame format for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | FRF
  *
  * @param  QSPIx - QSPI instance
  * @param  format - @ref LL_QSPI_CONCURRENT_XIP_FRF_RSVD
  *                  @ref LL_QSPI_CONCURRENT_XIP_FRF_DUAL_SPI
  *                  @ref LL_QSPI_CONCURRENT_XIP_FRF_QUAD_SPI
  *                  @ref LL_QSPI_CONCURRENT_XIP_FRF_OCTAL_SPI
  *
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_frame_format(qspi_regs_t *QSPIx, uint32_t format)
{
    MODIFY_REG(QSPIx->XIP_CTRL, QSPI_XCTRL_FRF, format << QSPI_XCTRL_FRF_Pos);
}

/**
  * @brief  Get the QSPI frame format for concurrent xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_CTRL | FRF
  *
  * @param  QSPIx - QSPI instance
  * @retval format - @ref LL_QSPI_CONCURRENT_XIP_FRF_RSVD
  *                  @ref LL_QSPI_CONCURRENT_XIP_FRF_DUAL_SPI
  *                  @ref LL_QSPI_CONCURRENT_XIP_FRF_QUAD_SPI
  *                  @ref LL_QSPI_CONCURRENT_XIP_FRF_OCTAL_SPI
  *
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_frame_format(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_CTRL, QSPI_XCTRL_FRF) >> QSPI_XCTRL_FRF_Pos);
}


/**
  * @brief  Enable qspi xip mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_XIP : M0_XIP_EN | M1_XIP_EN
  *
  * @param  QSPIx QSPI instance
  *
  * @retval None
  */
void ll_qspi_enable_xip(qspi_regs_t * QSPIx);

/**
  * @brief  Disable qspi xip mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_XIP : M0_XIP_EN | M1_XIP_EN
  *
  * @param  QSPIx QSPI instance
  *
  * @retval None
  */
void ll_qspi_disable_xip(qspi_regs_t * QSPIx);

/**
  * @brief  Check if qspi xip mode is enabled
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_XIP : M0_XIP_EN | M1_XIP_EN
  *
  * @param  QSPIx QSPI instance
  *
  * @retval None
  */
uint32_t ll_qspi_is_enabled_xip(qspi_regs_t * QSPIx);

/**
  * @brief  Set xip's endian mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_XIP : QSPI_M_XIP_M0_ENDIAN | QSPI_M_XIP_M1_ENDIAN | QSPI_M_XIP_M2_ENDIAN
  *
  * @param  QSPIx QSPI instance
  *
  * @param  mode - This parameter can be one of the following values:
  *         @arg @ref LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE_0
  *         @arg @ref LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE_1
  *         @arg @ref LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE_2
  *
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_xip_endian_mode(qspi_regs_t * QSPIx, uint32_t mode)
{
    uint32_t which = (QSPIx == QSPI0) ? MCU_SUB_QSPI_M_XIP_M0_ENDIAN_MODE_Pos : ( (QSPIx == QSPI1) ? MCU_SUB_QSPI_M_XIP_M1_ENDIAN_MODE_Pos : MCU_SUB_QSPI_M_XIP_M2_ENDIAN_MODE_Pos) ;

    MODIFY_REG(MCU_SUB->QSPI_M_XIP, MCU_SUB_QSPI_M_XIP_ENDIAN_ORDER & (0x3 << which), mode << which);
}


/**
  * @brief  Get xip's endian mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_XIP : QSPI_M_XIP_M0_ENDIAN | QSPI_M_XIP_M1_ENDIAN | QSPI_M_XIP_M2_ENDIAN
  *
  * @param QSPIx QSPI instance
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE_0
  *         @arg @ref LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE_1
  *         @arg @ref LL_QSPI_CONCURRENT_XIP_ENDIAN_MODE_2
  *
  */
__STATIC_INLINE uint32_t ll_qspi_get_xip_endian_mode(qspi_regs_t * QSPIx)
{
    uint32_t which = (QSPIx == QSPI0) ? MCU_SUB_QSPI_M_XIP_M0_ENDIAN_MODE_Pos : ( (QSPIx == QSPI1) ? MCU_SUB_QSPI_M_XIP_M1_ENDIAN_MODE_Pos : MCU_SUB_QSPI_M_XIP_M2_ENDIAN_MODE_Pos) ;

    return (READ_BITS(MCU_SUB->QSPI_M_XIP, MCU_SUB_QSPI_M_XIP_ENDIAN_ORDER & (0x3 << which)) >> which );
}

/************************* Add Following APIs from gr552xx-b0 *************************************/


/**
  * @brief  Enable dynamic of wait states for QSPI peripheral
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0   | DWS_EN
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_enable_dws(qspi_regs_t *QSPIx)
{
    SET_BITS(QSPIx->CTRL0, QSPI_CTRL0_DWS_EN);
}

/**
  * @brief  Disable dynamic of wait states for QSPI peripheral
  * @note   When disabling the SPI, follow the procedure described in the Reference Manual.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0   | DWS_EN
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_disable_dws(qspi_regs_t *QSPIx)
{
    CLEAR_BITS(QSPIx->CTRL0, QSPI_CTRL0_DWS_EN);
}

/**
  * @brief  Check if dynamic of wait states for QSPI peripheral is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0   | DWS_EN
  *
  * @param  QSPIx QSPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_qspi_is_enabled_dws(qspi_regs_t *QSPIx)
{
    return (READ_BITS(QSPIx->CTRL0, QSPI_CTRL0_DWS_EN) == (QSPI_CTRL0_DWS_EN));
}

/**
  * @brief  Clear QSPI Transmit Error interrupt
  * @note   Clearing this flag is done by reading SPITEIC register
  *
  *  Register|BitsName
  *  --------|--------
  *  SPI_TEIC | SPI_TEIC
  *
  * @param  QSPIx QSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_qspi_clear_flag_spite(qspi_regs_t *QSPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = QSPIx->SPI_TEIC;
    (void) tmpreg;
}

/**
  * @brief  set the max wait cycles per transaction for dynamic wait state
  * @note   This bit should not be changed when QSPI is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SPI_CTRL1 | QSPI_SCTRL1_MAX_WS
  *
  * @param  QSPIx  - QSPI instance
  * @param  max_ws - max wait cycles per transaction [0 ~ 15]
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_max_wait_cycles(qspi_regs_t *QSPIx, uint32_t max_ws)
{
    MODIFY_REG(QSPIx->SPI_CTRL1, QSPI_SCTRL1_MAX_WS, max_ws << QSPI_SCTRL1_MAX_WS_Pos);
}

/**
  * @brief  get the max wait cycles per transaction for dynamic wait state
  * @note   This bit should not be changed when QSPI is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SPI_CTRL1 | QSPI_SCTRL1_MAX_WS
  *
  * @param  QSPIx  - QSPI instance
  * @retval max wait cycles per transaction [0 ~ 15]
  */
__STATIC_INLINE uint32_t ll_qspi_get_max_wait_cycles(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->SPI_CTRL1, QSPI_SCTRL1_MAX_WS) >> QSPI_SCTRL1_MAX_WS_Pos);
}

/**
  * @brief  set the value for dynamic wait state
  * @note   This bit should not be changed when QSPI is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SPI_CTRL1 | QSPI_SCTRL1_DYN_WS
  *
  * @param  QSPIx  - QSPI instance
  * @param  dyn_ws - dynamic wait state [0 ~ 7]
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_dynamic_wait_state(qspi_regs_t *QSPIx, uint32_t dyn_ws)
{
    MODIFY_REG(QSPIx->SPI_CTRL1, QSPI_SCTRL1_DYN_WS, dyn_ws << QSPI_SCTRL1_DYN_WS_Pos);
}

/**
  * @brief  get the value for dynamic wait state
  * @note   This bit should not be changed when QSPI is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  SPI_CTRL1 | QSPI_SCTRL1_DYN_WS
  *
  * @param  QSPIx  - QSPI instance
  * @param  dyn_ws - dynamic wait state [0 ~ 7]
  * @retval dynamic wait state [0 ~ 7]
  */
__STATIC_INLINE uint32_t ll_qspi_get_dynamic_wait_state(qspi_regs_t *QSPIx, uint32_t dyn_ws)
{
    return (uint32_t)(READ_BITS(QSPIx->SPI_CTRL1, QSPI_SCTRL1_DYN_WS) >> QSPI_SCTRL1_DYN_WS_Pos);
}

/**
  * @brief  set the ahb-incr transfer instruction for write in xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_INCR_INST | WR_INCR_INST
  *
  * @param  QSPIx - QSPI instance
  * @param  inst  - instruction op-code, [0 ~ 0xFFFF]
  * @retval None
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_wr_incr_inst(qspi_regs_t *QSPIx, uint32_t inst)
{
    MODIFY_REG(QSPIx->XIP_WR_INCR_INST, QSPI_XIP_WR_INCR_INST, inst << QSPI_XIP_WR_INCR_INST_Pos);
}

/**
  * @brief  get the ahb-incr transfer instruction for write in xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_INCR_INST | WR_INCR_INST
  *
  * @param  QSPIx - QSPI instance
  * @retval inst  - instruction op-code, [0 ~ 0xFFFF]
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_wr_incr_inst(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_WR_INCR_INST, QSPI_XIP_WR_INCR_INST) >> QSPI_XIP_WR_INCR_INST_Pos);
}

/**
  * @brief  set the ahb-wrap transfer instruction for write in xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_WRAP_INST | WR_WRAP_INST
  *
  * @param  QSPIx - QSPI instance
  * @param  inst  - wrap instruction op-code, [0 ~ 0xFFFF]
  * @retval None
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_wr_wrap_inst(qspi_regs_t *QSPIx, uint32_t inst)
{
    MODIFY_REG(QSPIx->XIP_WR_WRAP_INST, QSPI_XIP_WR_WRAP_INST, inst << QSPI_XIP_WR_WRAP_INST_Pos);
}

/**
  * @brief  get the ahb-wrap transfer instruction for write in xip mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_WRAP_INST | WR_WRAP_INST
  *
  * @param  QSPIx - QSPI instance
  * @retval inst  - instruction op-code, [0 ~ 0xFFFF]
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_wr_wrap_inst(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_WR_WRAP_INST, QSPI_XIP_WR_WRAP_INST) >> QSPI_XIP_WR_WRAP_INST_Pos);
}


/**
  * @brief  Set the wait(also called dummy) cycles for concurrent xip write mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_CTRL | WAITCYCLES
  *
  * @param  QSPIx - QSPI instance
  * @param  wait_cycles - 0 ~ 31
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_wr_wait_cycles(qspi_regs_t *QSPIx, uint32_t wait_cycles)
{
    MODIFY_REG(QSPIx->XIP_WR_CTRL, QSPI_XIP_WR_CTRL_WAITCYCLES, wait_cycles << QSPI_XIP_WR_CTRL_WAITCYCLES_Pos);
}

/**
  * @brief  Get the wait(also called dummy) cycles for concurrent xip write mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_CTRL | WAITCYCLES
  *
  * @param  QSPIx - QSPI instance
  * @retval wait_cycles - 0 ~ 31
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_wr_wait_cycles(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_WR_CTRL, QSPI_XIP_WR_CTRL_WAITCYCLES) >> QSPI_XIP_WR_CTRL_WAITCYCLES_Pos);
}


/**
  * @brief  Set the instruction size for concurrent xip write mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_CTRL | INSTL
  *
  * @param  QSPIx - QSPI instance
  * @param  inst_size - @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_0BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_4BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_8BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_16BIT
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_wr_instruction_size(qspi_regs_t *QSPIx, uint32_t inst_size)
{
    MODIFY_REG(QSPIx->XIP_WR_CTRL, QSPI_XIP_WR_CTRL_INSTL, inst_size << QSPI_XIP_WR_CTRL_INSTL_Pos);
}

/**
  * @brief  Get the instruction size for concurrent xip write mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_CTRL | INSTL
  *
  * @param  QSPIx - QSPI instance
  * @retval inst_size - @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_0BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_4BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_8BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_INSTSIZE_16BIT
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_wr_instruction_size(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_WR_CTRL, QSPI_XIP_WR_CTRL_INSTL) >> QSPI_XIP_WR_CTRL_INSTL_Pos);
}


/**
  * @brief  Set the address size for concurrent xip write mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_CTRL | ADDRL
  *
  * @param  QSPIx - QSPI instance
  * @param  addr_size - @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_0BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_4BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_8BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_12BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_16BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_20BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_28BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_32BIT
  *
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_wr_address_size(qspi_regs_t *QSPIx, uint32_t addr_size)
{
    MODIFY_REG(QSPIx->XIP_WR_CTRL, QSPI_XIP_WR_CTRL_ADDRL, addr_size << QSPI_XIP_WR_CTRL_ADDRL_Pos);
}

/**
  * @brief  Get the address size for concurrent xip write mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_CTRL | ADDRL
  *
  * @param  QSPIx - QSPI instance
  * @retval addr_size - @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_0BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_4BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_8BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_12BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_16BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_20BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_24BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_28BIT
  *                     @ref LL_QSPI_CONCURRENT_XIP_ADDRSIZE_32BIT
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_wr_address_size(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_WR_CTRL, QSPI_XIP_WR_CTRL_ADDRL) >> QSPI_XIP_WR_CTRL_ADDRL_Pos);
}

/**
  * @brief  Set the transfer format of inst & address for concurrent xip write mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_CTRL | TRANS_TYPE
  *
  * @param  QSPIx - QSPI instance
  * @param  format - @ref LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPI
  *                  @ref LL_QSPI_CONCURRENT_XIP_INST_IN_SPI_ADDR_IN_SPIFRF
  *                  @ref LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF
  *
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_wr_inst_addr_transfer_format(qspi_regs_t *QSPIx, uint32_t format)
{
    MODIFY_REG(QSPIx->XIP_WR_CTRL, QSPI_XIP_WR_CTRL_TRANSTYPE, format << QSPI_XIP_WR_CTRL_TRANSTYPE_Pos);
}

/**
  * @brief  Get the transfer format of inst & address for concurrent xip write mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_CTRL | TRANS_TYPE
  *
  * @param  QSPIx - QSPI instance
  * @retval format - @ref LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPI
  *                  @ref LL_QSPI_CONCURRENT_XIP_INST_IN_SPI_ADDR_IN_SPIFRF
  *                  @ref LL_QSPI_CONCURRENT_XIP_INST_ADDR_ALL_IN_SPIFRF
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_wr_addr_inst_transfer_format(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_WR_CTRL, QSPI_XIP_WR_CTRL_TRANSTYPE) >> QSPI_XIP_WR_CTRL_TRANSTYPE_Pos);
}

/**
  * @brief  Set the QSPI frame format for concurrent xip write mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_CTRL | FRF
  *
  * @param  QSPIx - QSPI instance
  * @param  format - @ref LL_QSPI_CONCURRENT_XIP_FRF_RSVD
  *                  @ref LL_QSPI_CONCURRENT_XIP_FRF_DUAL_SPI
  *                  @ref LL_QSPI_CONCURRENT_XIP_FRF_QUAD_SPI
  *                  @ref LL_QSPI_CONCURRENT_XIP_FRF_OCTAL_SPI
  *
  * @retval none
  */
__STATIC_INLINE void ll_qspi_concurrent_set_xip_wr_frame_format(qspi_regs_t *QSPIx, uint32_t format)
{
    MODIFY_REG(QSPIx->XIP_WR_CTRL, QSPI_XIP_WR_CTRL_FRF, format << QSPI_XIP_WR_CTRL_FRF_Pos);
}

/**
  * @brief  Get the QSPI frame format for concurrent xip write mode
  * @note   This bit should not be changed when xip is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  XIP_WR_CTRL | FRF
  *
  * @param  QSPIx - QSPI instance
  * @retval format - @ref LL_QSPI_CONCURRENT_XIP_FRF_RSVD
  *                  @ref LL_QSPI_CONCURRENT_XIP_FRF_DUAL_SPI
  *                  @ref LL_QSPI_CONCURRENT_XIP_FRF_QUAD_SPI
  *                  @ref LL_QSPI_CONCURRENT_XIP_FRF_OCTAL_SPI
  *
  */
__STATIC_INLINE uint32_t ll_qspi_concurrent_get_xip_wr_frame_format(qspi_regs_t *QSPIx)
{
    return (uint32_t)(READ_BITS(QSPIx->XIP_WR_CTRL, QSPI_XIP_WR_CTRL_FRF) >> QSPI_XIP_WR_CTRL_FRF_Pos);
}

/**
  * @brief  Enable the AHB Response Error Debug for all QSPI Modules. if violation, hardfault happens
  * @note   Just enable in debug mode, not enable in release code
  *         Set 0 to get hardfault when generate hresp = 1
  *         Set 1 to avoid hardfault and always mask the hresp to 0, 1 is default value after System Reset
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_HRESP_DBG : QSPI_M_HRESP_ERR_MASK
  *
  *
  * @retval none
  *
  */
__STATIC_INLINE void ll_qspi_enable_hresp_err_debug_mode(void)
{
    MODIFY_REG(MCU_SUB->QSPI_M_HRESP_DBG, MCU_SUB_QSPI_M_HRESP_ERR_MASK_EN, 0);
}

/**
  * @brief  Disable the AHB Response Error Debug for all QSPI Modules. if violation, hardfault happens
  * @note   Just enable in debug mode, not enable in release code
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_HRESP_DBG : QSPI_M_HRESP_ERR_MASK
  *
  *
  * @retval none
  *
  */
__STATIC_INLINE void ll_qspi_disable_hresp_err_debug_mode(void)
{
    MODIFY_REG(MCU_SUB->QSPI_M_HRESP_DBG, MCU_SUB_QSPI_M_HRESP_ERR_MASK_EN, 1);
}

/**
  * @brief  Check if the AHB Response Error Debug is enabled for all QSPI Modules.
  * @note
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_HRESP_DBG : QSPI_M_HRESP_ERR_MASK
  *
  *
  * @retval none
  *
  */
__STATIC_INLINE uint32_t ll_qspi_is_hresp_err_debug_mode_enabled(void)
{
    return (READ_BITS(MCU_SUB->QSPI_M_HRESP_DBG, MCU_SUB_QSPI_M_HRESP_ERR_MASK_EN) == (0));
}

/**
  * @brief  Set CS Setup Delay for QSPI
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_CS_SETUP_DLY : MCU_SUB_QSPI_M0_CS_SETUP_DLY | MCU_SUB_QSPI_M1_CS_SETUP_DLY | MCU_SUB_QSPI_M2_CS_SETUP_DLY
  *
  * @param  QSPIx QSPI instance
  *
  * @param  delay - the SLCK count to delay
  *
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_cs_setup_delay(qspi_regs_t * QSPIx, uint32_t delay)
{
    uint32_t which = (QSPIx == QSPI0) ? MCU_SUB_QSPI_M0_CS_SETUP_DLY_Pos : ( (QSPIx == QSPI1) ? MCU_SUB_QSPI_M1_CS_SETUP_DLY_Pos : MCU_SUB_QSPI_M2_CS_SETUP_DLY_Pos) ;
    uint32_t baudrate = ll_qspi_get_baud_rate_prescaler(QSPIx);

    MODIFY_REG(MCU_SUB->QSPI_M_CS_SETUP_DLY, 0xFF << which, (baudrate*delay) << which);
}


/**
  * @brief Get CS Setup Delay for QSPI
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_CS_SETUP_DLY : MCU_SUB_QSPI_M0_CS_SETUP_DLY | MCU_SUB_QSPI_M1_CS_SETUP_DLY | MCU_SUB_QSPI_M2_CS_SETUP_DLY
  *
  * @param  QSPIx QSPI instance
  *
  * @retval the SLCK count to delay
  */
__STATIC_INLINE uint32_t ll_qspi_get_cs_setup_delay(qspi_regs_t * QSPIx)
{
    uint32_t which = (QSPIx == QSPI0) ? MCU_SUB_QSPI_M0_CS_SETUP_DLY_Pos : ( (QSPIx == QSPI1) ? MCU_SUB_QSPI_M1_CS_SETUP_DLY_Pos : MCU_SUB_QSPI_M2_CS_SETUP_DLY_Pos) ;
    uint32_t baudrate = ll_qspi_get_baud_rate_prescaler(QSPIx);

    if(0 == baudrate){
        return 0;
    }

    return (READ_BITS(MCU_SUB->QSPI_M_CS_SETUP_DLY, 0xFF << which) >> which)/baudrate;
}

/**
  * @brief  Set CS Release Delay for QSPI
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_CS_RELEASE_DLY : MCU_SUB_QSPI_M0_CS_RELEASE_DLY | MCU_SUB_QSPI_M1_CS_RELEASE_DLY | MCU_SUB_QSPI_M2_CS_RELEASE_DLY
  *
  * @param  QSPIx QSPI instance
  *
  * @param  delay - the SLCK count to delay
  *
  * @retval None
  */
__STATIC_INLINE void ll_qspi_set_cs_release_delay(qspi_regs_t * QSPIx, uint32_t delay)
{
    uint32_t which = (QSPIx == QSPI0) ? MCU_SUB_QSPI_M0_CS_RELEASE_DLY_Pos : ((QSPIx == QSPI1) ? MCU_SUB_QSPI_M1_CS_RELEASE_DLY_Pos : MCU_SUB_QSPI_M2_CS_RELEASE_DLY_Pos) ;
    uint32_t baudrate = ll_qspi_get_baud_rate_prescaler(QSPIx);

    MODIFY_REG(MCU_SUB->QSPI_M_CS_RELEASE_DLY, 0xFF << which, (baudrate*delay) << which);
}


/**
  * @brief Get CS Release Delay for QSPI
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_CS_RELEASE_DLY : MCU_SUB_QSPI_M0_CS_RELEASE_DLY | MCU_SUB_QSPI_M1_CS_RELEASE_DLY | MCU_SUB_QSPI_M2_CS_RELEASE_DLY
  *
  * @param  QSPIx QSPI instance
  *
  * @retval the SLCK count to delay
  */
__STATIC_INLINE uint32_t ll_qspi_get_cs_release_delay(qspi_regs_t * QSPIx)
{
    uint32_t which = (QSPIx == QSPI0) ? MCU_SUB_QSPI_M0_CS_RELEASE_DLY_Pos : ( (QSPIx == QSPI1) ? MCU_SUB_QSPI_M1_CS_RELEASE_DLY_Pos : MCU_SUB_QSPI_M2_CS_RELEASE_DLY_Pos) ;
    uint32_t baudrate = ll_qspi_get_baud_rate_prescaler(QSPIx);

    if(0 == baudrate){
        return 0;
    }

    return (READ_BITS(MCU_SUB->QSPI_M_CS_RELEASE_DLY, 0xFF << which) >> which)/baudrate;
}

/**
  * @brief  Enable qspi xip dynamic little-endian mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_XIP : MCU_SUB_QSPI_M_XIP_M0_DYNAMIC_LE_EN | MCU_SUB_QSPI_M_XIP_M1_DYNAMIC_LE_EN | MCU_SUB_QSPI_M_XIP_M2_DYNAMIC_LE_EN
  *
  * @param  QSPIx QSPI instance
  *
  * @retval None
  */
__STATIC_INLINE void ll_qspi_enable_xip_dynamic_le(qspi_regs_t * QSPIx)
{
    uint32_t which = (QSPIx == QSPI0) ? MCU_SUB_QSPI_M_XIP_M0_DYNAMIC_LE_EN : ( (QSPIx == QSPI1) ? MCU_SUB_QSPI_M_XIP_M1_DYNAMIC_LE_EN : MCU_SUB_QSPI_M_XIP_M2_DYNAMIC_LE_EN) ;
    SET_BITS(MCU_SUB->QSPI_M_XIP, which);
}

/**
  * @brief  Disable qspi xip dynamic little-endian mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_XIP : MCU_SUB_QSPI_M_XIP_M0_DYNAMIC_LE_EN | MCU_SUB_QSPI_M_XIP_M1_DYNAMIC_LE_EN | MCU_SUB_QSPI_M_XIP_M2_DYNAMIC_LE_EN
  *
  * @param  QSPIx QSPI instance
  *
  * @retval None
  */
__STATIC_INLINE void ll_qspi_disable_xip_dynamic_le(qspi_regs_t * QSPIx)
{
    uint32_t which = (QSPIx == QSPI0) ? MCU_SUB_QSPI_M_XIP_M0_DYNAMIC_LE_EN : ( (QSPIx == QSPI1) ? MCU_SUB_QSPI_M_XIP_M1_DYNAMIC_LE_EN : MCU_SUB_QSPI_M_XIP_M2_DYNAMIC_LE_EN) ;
    CLEAR_BITS(MCU_SUB->QSPI_M_XIP, which);
}

/**
  * @brief  Check if qspi xip dynamic little-endian mode is enabled
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  QSPI_M_XIP : MCU_SUB_QSPI_M_XIP_M0_DYNAMIC_LE_EN | MCU_SUB_QSPI_M_XIP_M1_DYNAMIC_LE_EN | MCU_SUB_QSPI_M_XIP_M2_DYNAMIC_LE_EN
  *
  * @param  QSPIx QSPI instance
  *
  * @retval None
  */
__STATIC_INLINE uint32_t ll_qspi_is_enabled_xip_dynamic_le(qspi_regs_t * QSPIx)
{
    uint32_t which = (QSPIx == QSPI0) ? MCU_SUB_QSPI_M_XIP_M0_DYNAMIC_LE_EN : ( (QSPIx == QSPI1) ? MCU_SUB_QSPI_M_XIP_M1_DYNAMIC_LE_EN : MCU_SUB_QSPI_M_XIP_M2_DYNAMIC_LE_EN) ;
    return (READ_BITS(MCU_SUB->QSPI_M_XIP, which) == (which));
}

/**
  * @brief  Get Receive FIFO Depth Of Register Mode
  * @note
  * @param  QSPIx QSPI instance
  * @retval Receive FIFO Depth corresponded to QSPI instance
  */
__STATIC_INLINE uint32_t ll_qspi_get_reg_mode_rx_fifo_depth(qspi_regs_t *QSPIx)
{
    if(QSPI0 == QSPIx) {
        return LL_QSPI0_REG_RX_FIFO_DEPTH;
    } else if(QSPI1 == QSPIx) {
        return LL_QSPI1_REG_RX_FIFO_DEPTH;
    } else {
        return LL_QSPI2_REG_RX_FIFO_DEPTH;
    }
}

/**
  * @brief  Get Transmit FIFO Depth Of Register Mode
  * @note
  * @param  QSPIx QSPI instance
  * @retval Transmit FIFO Depth corresponded to QSPI instance
  */
__STATIC_INLINE uint32_t ll_qspi_get_reg_mode_tx_fifo_depth(qspi_regs_t *QSPIx)
{
    if(QSPI0 == QSPIx) {
        return LL_QSPI0_REG_TX_FIFO_DEPTH;
    } else if(QSPI1 == QSPIx) {
        return LL_QSPI1_REG_TX_FIFO_DEPTH;
    } else {
        return LL_QSPI2_REG_TX_FIFO_DEPTH;
    }
}

/**
  * @brief  Get Receive FIFO Depth Of XIP Mode
  * @note
  * @param  QSPIx QSPI instance
  * @retval Receive FIFO Depth corresponded to QSPI instance
  */
__STATIC_INLINE uint32_t ll_qspi_get_xip_mode_rx_fifo_depth(qspi_regs_t *QSPIx)
{
    if(QSPI0 == QSPIx) {
        return LL_QSPI0_XIP_RX_FIFO_DEPTH;
    } else if(QSPI1 == QSPIx) {
        return LL_QSPI1_XIP_RX_FIFO_DEPTH;
    } else {
        return LL_QSPI2_XIP_RX_FIFO_DEPTH;
    }
}

/**
  * @brief  Get Transmit FIFO Depth Of XIP Mode
  * @note
  * @param  QSPIx QSPI instance
  * @retval Transmit FIFO Depth corresponded to QSPI instance
  */
__STATIC_INLINE uint32_t ll_qspi_get_xip_mode_tx_fifo_depth(qspi_regs_t *QSPIx)
{
    if(QSPI0 == QSPIx) {
        return LL_QSPI0_XIP_TX_FIFO_DEPTH;
    } else if(QSPI1 == QSPIx) {
        return LL_QSPI1_XIP_TX_FIFO_DEPTH;
    } else {
        return LL_QSPI2_XIP_TX_FIFO_DEPTH;
    }
}

/**
  * @brief  Get Transmit FIFO Depth Of XIP Mode
  * @note
  * @param  QSPIx QSPI instance
  * @retval Transmit FIFO Depth corresponded to QSPI instance
  */
__STATIC_INLINE uint32_t ll_qspi_get_xip_base_address(qspi_regs_t *QSPIx)
{
    if(QSPI0 == QSPIx) {
        return QSPI0_XIP_BASE;
    } else if(QSPI1 == QSPIx) {
        return QSPI1_XIP_BASE;
    } else {
        return QSPI2_XIP_BASE;
    }
}
/** @} */
/** @defgroup LL_QSPI_Init QSPI Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize SSI registers (Registers restored to their default values).
  * @param  QSPIx SSI instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SSI registers are de-initialized
  *          - ERROR: SSI registers are not de-initialized
  */
error_status_t ll_qspi_deinit(qspi_regs_t *QSPIx);

/**
  * @brief  Initialize SSI registers according to the specified
  *         parameters in SPI_InitStruct.
  * @param  QSPIx SSI instance
  * @param  p_spi_init Pointer to a ll_qspi_init_t structure that contains the configuration
  *                         information for the specified QSPI peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SPI registers are initialized according to p_spi_init content
  *          - ERROR: Problem occurred during SPI Registers initialization
  */
error_status_t ll_qspi_init(qspi_regs_t *QSPIx, ll_qspi_init_t *p_spi_init);

/**
  * @brief  Configure the qspi to memorymapped.
  * @param  QSPIx QSPI instance
  * @param  p_qspi_mmap_init pointer to a @ref ll_qspi_memorymapped_init_t structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: not applicable
  */
error_status_t ll_qspi_memorymapped(qspi_regs_t *QSPIx, ll_qspi_memorymapped_init_t * p_qspi_mmap_init);


/**
  * @brief Set each field of a @ref ll_qspi_init_t type structure to default value.
  * @param p_spi_init  Pointer to a @ref ll_qspi_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_qspi_struct_init(ll_qspi_init_t *p_spi_init);

/** @} */

/** @} */

#endif /* QSPI0 || QSPI1 || QSPI2 */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_SPI_H__ */

/** @} */

/** @} */

/** @} */

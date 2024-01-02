/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_spi.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of SPI LL library.
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

/** @defgroup LL_SPI SPI
  * @brief SPI LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_SPI_H__
#define __GR55xx_LL_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (SPIM) || defined (SPIS)

/** @defgroup LL_SPI_DRIVER_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SPI_LL_ES_INIT SPI Exported init structure
  * @{
  */

/**
  * @brief LL SPIM init structures definition
  */
typedef struct _ll_spim_init_t
{
    uint32_t transfer_direction;    /**< Specifies the SPI unidirectional or bidirectional data mode.
                                         This parameter can be a value of @ref SPI_LL_EC_TRANSFER_MODE.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_transfer_direction().*/

    uint32_t data_size;             /**< Specifies the SPI data size.
                                         This parameter can be a value of @ref SPI_LL_EC_DATASIZE.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_data_size().*/

    uint32_t clock_polarity;        /**< Specifies the serial clock steady state.
                                         This parameter can be a value of @ref SPI_LL_EC_POLARITY.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_clock_polarity().*/

    uint32_t clock_phase;           /**< Specifies the clock active edge for the bit capture.
                                         This parameter can be a value of @ref SPI_LL_EC_PHASE.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_clock_phase().*/

    uint32_t slave_select;          /**< Specifies the SPI slave select.
                                         This parameter can be a value of @ref SPI_LL_EC_SLAVESELECT.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_enable_ss().*/

    uint32_t baud_rate;             /**< Specifies the BaudRate prescaler value which will be used to configure the transmit and receive SCK clock.
                                         This parameter can be one even value between 2 and 65534, if the value is 0, the SCLK is disable.
                                         @note The communication clock is derived from the master clock. The slave clock does not need to be set.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_baud_rate_prescaler().*/
    uint32_t rx_sample_delay;           /**< Specifies the RX sample delay. It is used to delay the sample of the RX input port.
                                             This parameter can be a number between 0 and 0x7 */
} ll_spim_init_t;

/**
  * @brief  SPIS init structures definition
  */
typedef struct _ll_spis_init_t
{
    uint32_t data_size;             /**< Specifies the SPI data width.
                                         This parameter can be a value of @ref SPI_LL_EC_DATASIZE.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_data_size().*/

    uint32_t clock_polarity;        /**< Specifies the serial clock steady state.
                                         This parameter can be a value of @ref SPI_LL_EC_POLARITY.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_clock_polarity().*/

    uint32_t clock_phase;           /**< Specifies the clock active edge for the bit capture.
                                         This parameter can be a value of @ref SPI_LL_EC_PHASE.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_clock_phase().*/

} ll_spis_init_t;

/** @} */

/** @} */

/**
  * @defgroup  SPI_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SPI_LL_Exported_Constants SPI Exported Constants
  * @{
  */

/** @defgroup SPI_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags definitions which can be used with LL_SPI_ReadReg function
  * @{
  */
#define LL_SPI_SR_DCOL                      SPI_STAT_DATA_COLN_ERR               /**< Data collision error flag      */
#define LL_SPI_SR_TXE                       SPI_STAT_TX_ERR                      /**< Transmission error flag        */
#define LL_SPI_SR_RFF                       SPI_STAT_RX_FIFO_FULL                /**< Rx FIFO full flag              */
#define LL_SPI_SR_RFNE                      SPI_STAT_RX_FIFO_NE                  /**< Rx FIFO not empty flag         */
#define LL_SPI_SR_TFE                       SPI_STAT_TX_FIFO_EMPTY               /**< Tx FIFO empty flag             */
#define LL_SPI_SR_TFNF                      SPI_STAT_TX_FIFO_NF                  /**< Tx FIFO not full flag          */
#define LL_SPI_SR_BUSY                      SPI_STAT_SSI_BUSY                    /**< Busy flag                      */
/** @} */

/** @defgroup SPI_LL_EC_IT IT Defines
  * @brief    Interrupt definitions which can be used with LL_SPI_ReadReg and  LL_SPI_WriteReg functions
  * @{
  */
#define LL_SPI_IM_MST                       SPI_INT_MASK_MULTI_M_CIM             /**< Multi-Master Contention Interrupt enable   */
#define LL_SPI_IM_RXF                       SPI_INT_MASK_RX_FIFO_FIM             /**< Receive FIFO Full Interrupt enable         */
#define LL_SPI_IM_RXO                       SPI_INT_MASK_RX_FIFO_OIM             /**< Receive FIFO Overflow Interrupt  enable    */
#define LL_SPI_IM_RXU                       SPI_INT_MASK_RX_FIFO_UIM             /**< Receive FIFO Underflow Interrupt  enable   */
#define LL_SPI_IM_TXO                       SPI_INT_MASK_TX_FIFO_OIM             /**< Transmit FIFO Overflow Interrupt  enable   */
#define LL_SPI_IM_TXE                       SPI_INT_MASK_TX_FIFO_EIM             /**< Transmit FIFO Empty Interrupt  enable      */

#define LL_SPI_IS_MST                       SPI_INT_STAT_MULTI_M_CIS             /**< Multi-Master Contention Interrupt flag     */
#define LL_SPI_IS_RXF                       SPI_INT_STAT_RX_FIFO_FIS             /**< Receive FIFO Full Interrupt flag           */
#define LL_SPI_IS_RXO                       SPI_INT_STAT_RX_FIFO_OIS             /**< Receive FIFO Overflow Interrupt  flag      */
#define LL_SPI_IS_RXU                       SPI_INT_STAT_RX_FIFO_UIS             /**< Receive FIFO Underflow Interrupt  flag     */
#define LL_SPI_IS_TXO                       SPI_INT_STAT_TX_FIFO_OIS             /**< Transmit FIFO Overflow Interrupt  flag     */
#define LL_SPI_IS_TXE                       SPI_INT_STAT_TX_FIFO_EIS             /**< Transmit FIFO Empty Interrupt  flag        */

#define LL_SPI_RIS_MST                      SPI_RAW_INT_STAT_MULTI_M_CRIS        /**< Multi-Master Contention RAW Interrupt flag */
#define LL_SPI_RIS_RXF                      SPI_RAW_INT_STAT_RX_FIFO_FRIS        /**< Receive FIFO Full RAW Interrupt flag       */
#define LL_SPI_RIS_RXO                      SPI_RAW_INT_STAT_RX_FIFO_ORIS        /**< Receive FIFO Overflow RAW Interrupt  flag  */
#define LL_SPI_RIS_RXU                      SPI_RAW_INT_STAT_RX_FIFO_URIS        /**< Receive FIFO Underflow RAW Interrupt  flag */
#define LL_SPI_RIS_TXO                      SPI_RAW_INT_STAT_TX_FIFO_ORIS        /**< Transmit FIFO Overflow RAW Interrupt  flag */
#define LL_SPI_RIS_TXE                      SPI_RAW_INT_STAT_TX_FIFO_ERIS        /**< Transmit FIFO Empty RAW Interrupt  flag    */
/** @} */

/** @defgroup SPI_LL_EC_DATASIZE Datawidth
  * @{
  */
#define LL_SPI_DATASIZE_4BIT                (3UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)    /**< Data length for SPI transfer:  4 bits */
#define LL_SPI_DATASIZE_5BIT                (4UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)    /**< Data length for SPI transfer:  5 bits */
#define LL_SPI_DATASIZE_6BIT                (5UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)    /**< Data length for SPI transfer:  6 bits */
#define LL_SPI_DATASIZE_7BIT                (6UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)    /**< Data length for SPI transfer:  7 bits */
#define LL_SPI_DATASIZE_8BIT                (7UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)    /**< Data length for SPI transfer:  8 bits */
#define LL_SPI_DATASIZE_9BIT                (8UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)    /**< Data length for SPI transfer:  9 bits */
#define LL_SPI_DATASIZE_10BIT               (9UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)    /**< Data length for SPI transfer: 10 bits */
#define LL_SPI_DATASIZE_11BIT               (10UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 11 bits */
#define LL_SPI_DATASIZE_12BIT               (11UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 12 bits */
#define LL_SPI_DATASIZE_13BIT               (12UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 13 bits */
#define LL_SPI_DATASIZE_14BIT               (13UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 14 bits */
#define LL_SPI_DATASIZE_15BIT               (14UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 15 bits */
#define LL_SPI_DATASIZE_16BIT               (15UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 16 bits */
#define LL_SPI_DATASIZE_17BIT               (16UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 17 bits */
#define LL_SPI_DATASIZE_18BIT               (17UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 18 bits */
#define LL_SPI_DATASIZE_19BIT               (18UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 19 bits */
#define LL_SPI_DATASIZE_20BIT               (19UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 20 bits */
#define LL_SPI_DATASIZE_21BIT               (20UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 21 bits */
#define LL_SPI_DATASIZE_22BIT               (21UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 22 bits */
#define LL_SPI_DATASIZE_23BIT               (22UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 23 bits */
#define LL_SPI_DATASIZE_24BIT               (23UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 24 bits */
#define LL_SPI_DATASIZE_25BIT               (24UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 25 bits */
#define LL_SPI_DATASIZE_26BIT               (25UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 26 bits */
#define LL_SPI_DATASIZE_27BIT               (26UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 27 bits */
#define LL_SPI_DATASIZE_28BIT               (27UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 28 bits */
#define LL_SPI_DATASIZE_29BIT               (28UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 29 bits */
#define LL_SPI_DATASIZE_30BIT               (29UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 30 bits */
#define LL_SPI_DATASIZE_31BIT               (30UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 31 bits */
#define LL_SPI_DATASIZE_32BIT               (31UL << SPI_CTRL0_DATA_FRAME_SIZE_POS)   /**< Data length for SPI transfer: 32 bits */
/** @} */

/** @defgroup SPI_LL_EC_MICROWIRECOMMANDSIZE MicroWire CommandSize
  * @{
  */
#define LL_SPI_MW_CMDSIZE_1BIT              0x00000000UL                              /**< CMD length for Microwire transfer:  1 bits */
#define LL_SPI_MW_CMDSIZE_2BIT              (1UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)     /**< CMD length for Microwire transfer:  2 bits */
#define LL_SPI_MW_CMDSIZE_3BIT              (2UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)     /**< CMD length for Microwire transfer:  3 bits */
#define LL_SPI_MW_CMDSIZE_4BIT              (3UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)     /**< CMD length for Microwire transfer:  4 bits */
#define LL_SPI_MW_CMDSIZE_5BIT              (4UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)     /**< CMD length for Microwire transfer:  5 bits */
#define LL_SPI_MW_CMDSIZE_6BIT              (5UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)     /**< CMD length for Microwire transfer:  6 bits */
#define LL_SPI_MW_CMDSIZE_7BIT              (6UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)     /**< CMD length for Microwire transfer:  7 bits */
#define LL_SPI_MW_CMDSIZE_8BIT              (7UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)     /**< CMD length for Microwire transfer:  8 bits */
#define LL_SPI_MW_CMDSIZE_9BIT              (8UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)     /**< CMD length for Microwire transfer:  9 bits */
#define LL_SPI_MW_CMDSIZE_10BIT             (9UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)     /**< CMD length for Microwire transfer: 10 bits */
#define LL_SPI_MW_CMDSIZE_11BIT             (10UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)    /**< CMD length for Microwire transfer: 11 bits */
#define LL_SPI_MW_CMDSIZE_12BIT             (11UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)    /**< CMD length for Microwire transfer: 12 bits */
#define LL_SPI_MW_CMDSIZE_13BIT             (12UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)    /**< CMD length for Microwire transfer: 13 bits */
#define LL_SPI_MW_CMDSIZE_14BIT             (13UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)    /**< CMD length for Microwire transfer: 14 bits */
#define LL_SPI_MW_CMDSIZE_15BIT             (14UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)    /**< CMD length for Microwire transfer: 15 bits */
#define LL_SPI_MW_CMDSIZE_16BIT             (15UL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)    /**< CMD length for Microwire transfer: 16 bits */
/** @} */

/** @defgroup SPI_LL_EC_TEST_MODE Test Mode
  * @{
  */
#define LL_SPI_NORMAL_MODE                  0x00000000UL                              /**< Normal mode for SPI transfer                           */
#define LL_SPI_TEST_MODE                    (1UL << SPI_CTRL0_SHIFT_REG_LOOP_POS)      /**< Test mode for SPI transfer: Rx and Tx connected inside */
/** @} */

/** @defgroup SPI_LL_EC_SLAVEOUT_ENABLE Slave Out Enable
  * @{
  */
#define LL_SPI_SLAVE_OUTDIS                 0x00000000UL                       /**< Output enable for SPI transfer as slave    */
#define LL_SPI_SLAVE_OUTEN                  (1UL << SPI_CTRL0_S_OUT_EN_POS)     /**< Output disable for SPI transfer as slave   */
/** @} */

/** @defgroup SPI_LL_EC_TRANSFER_MODE Transfer Mode
  * @{
  */
#define LL_SPI_FULL_DUPLEX                  0x00000000UL                       /**< Full-Duplex mode. Rx and Tx transfer on 2 lines */
#define LL_SPI_SIMPLEX_TX                   (1UL << SPI_CTRL0_XFE_MODE_POS)     /**< Simplex Tx mode.  Tx transfer only on 1 line    */
#define LL_SPI_SIMPLEX_RX                   (2UL << SPI_CTRL0_XFE_MODE_POS)     /**< Simplex Rx mode.  Rx transfer only on 1 line    */
#define LL_SPI_READ_EEPROM                  (3UL << SPI_CTRL0_XFE_MODE_POS)     /**< Read EEPROM mode.  Rx transfer only on 1 line   */
/** @} */

/** @defgroup SPI_LL_EC_PHASE Clock Phase
  * @{
  */
#define LL_SPI_SCPHA_1EDGE                  0x00000000UL                              /**< First clock transition is the first data capture edge  */
#define LL_SPI_SCPHA_2EDGE                  (1UL << SPI_CTRL0_SERIAL_CLK_PHASE_POS)    /**< Second clock transition is the first data capture edge */
/** @} */

/** @defgroup SPI_LL_EC_POLARITY Clock Polarity
  * @{
  */
#define LL_SPI_SCPOL_LOW                    0x00000000UL                            /**< Clock to 0 when idle */
#define LL_SPI_SCPOL_HIGH                   (1UL << SPI_CTRL0_SERIAL_CLK_POL_POS)    /**< Clock to 1 when idle */
/** @} */

/** @defgroup SPI_LL_EC_PROTOCOL Serial Protocol
  * @{
  */
#define LL_SPI_PROTOCOL_MOTOROLA            0x00000000UL                           /**< Motorola mode. Used as default value */
#define LL_SPI_PROTOCOL_TI                  (1UL << SPI_CTRL0_FRAME_FORMAT_POS)    /**< TI mode                              */
#define LL_SPI_PROTOCOL_MICROWIRE           (2UL << SPI_CTRL0_FRAME_FORMAT_POS)    /**< Microwire mode                       */
/** @} */

/** @defgroup SPI_LL_EC_MICROWIRECONTROL MicroWire Control
  * @{
  */
#define LL_SPI_MICROWIRE_HANDSHAKE_DIS      0x00000000UL                           /**< Enable Handshake for Microwire transfer  */
#define LL_SPI_MICROWIRE_HANDSHAKE_EN       (1UL << SPI_MW_CTRL_MW_HSG_POS)        /**< Disable Handshake for Microwire transfer */

#define LL_SPI_MICROWIRE_RX                 0x00000000UL                           /**< Rx mode. Rx transfer at Microwire mode */
#define LL_SPI_MICROWIRE_TX                 (1UL << SPI_MW_CTRL_MW_DIR_DW_POS)     /**< Tx mode. Tx transfer at Microwire mode */

#define LL_SPI_MICROWIRE_NON_SEQUENTIAL     0x00000000UL                           /**< Non-sequential for Microwire transfer  */
#define LL_SPI_MICROWIRE_SEQUENTIAL         (1UL << SPI_MW_CTRL_MW_XFE_MODE_POS)   /**< Sequential for Microwire transfer      */
/** @} */

/** @defgroup SPI_LL_EC_SLAVESELECT Slave Select
  * @{
  */
#define LL_SPI_SLAVE1                       SPI_SLA_S1_SEL_EN            /**< Enable slave1 select pin for SPI transfer  */
#define LL_SPI_SLAVE0                       SPI_SLA_S0_SEL_EN            /**< Enable slave0 select pin for SPI transfer  */
/** @} */

/** @defgroup SPI_LL_EC_DMA DMA Defines
  * @{
  */
#define LL_SPI_DMA_TX_DIS                   0x00000000UL                 /**< Disable the transmit FIFO DMA channel */
#define LL_SPI_DMA_TX_EN                    SPI_DMA_CTRL_TX_DMA_EN       /**< Enable the transmit FIFO DMA channel  */

#define LL_SPI_DMA_RX_DIS                   0x00000000UL                 /**< Disable the receive FIFO DMA channel */
#define LL_SPI_DMA_RX_EN                    SPI_DMA_CTRL_RX_DMA_EN       /**< Enable the receive FIFO DMA channel  */
/** @} */

/** @defgroup LL_SPI_x_FIFO_DEPTH Defines
  * @{
  */
#define LL_SPI_M_FIFO_DEPTH                 (16u)                        /**< FIFO Depth for SPI Master */
#define LL_SPI_S_FIFO_DEPTH                 (16u)                        /**< FIFO Depth for SPI Slave  */
/** @} */

/** @defgroup SPI_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */

/**
  * @brief LL SPIM InitStrcut default configuartion
  */
#define LL_SPIM_DEFAULT_CONFIG                         \
    {                                                      \
        .transfer_direction  = LL_SPI_FULL_DUPLEX,         \
        .data_size           = LL_SPI_DATASIZE_8BIT,       \
        .clock_polarity      = LL_SPI_SCPOL_LOW,           \
        .clock_phase         = LL_SPI_SCPHA_1EDGE,         \
        .slave_select        = LL_SPI_SLAVE0,              \
        .baud_rate           = SystemCoreClock / 2000000,  \
        .rx_sample_delay     = 0,                          \
    }

/**
  * @brief LL SPIS InitStrcut default configuartion
  */
#define LL_SPIS_DEFAULT_CONFIG                       \
{                                                    \
    .data_size           = LL_SPI_DATASIZE_8BIT,     \
    .clock_polarity      = LL_SPI_SCPOL_LOW,         \
    .clock_phase         = LL_SPI_SCPHA_1EDGE,       \
}
/** @} */

/** @} */


/* Exported macro ------------------------------------------------------------*/
/** @defgroup SPI_LL_Exported_Macros SPI Exported Macros
  * @{
  */

/** @defgroup SPI_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in SPI register
  * @param  __instance__ SPI instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_SPI_WriteReg(__instance__, __REG__, __VALUE__)   WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in SPI register
  * @param  __instance__ SPI instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_SPI_ReadReg(__instance__, __REG__)               READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SPI_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup SPI_LL_EF_Configuration Configuration functions
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
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_ss_toggle(spi_regs_t *SPIx)
{
    SET_BITS(SPIx->CTRL0, SPI_CTRL0_S_ST_EN);
}

/**
  * @brief  Disable slave select toggle
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SSTEN
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_ss_toggle(spi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->CTRL0, SPI_CTRL0_S_ST_EN);
}

/**
  * @brief  Check if slave select toggle is enabled
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SSTEN
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_ss_toggle(spi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->CTRL0, SPI_CTRL0_S_ST_EN) == (SPI_CTRL0_S_ST_EN));
}

/**
  * @brief  Set frame data size
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | DFS32
  *
  * @param  SPIx SPI instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_SPI_DATASIZE_4BIT
  *         @arg @ref LL_SPI_DATASIZE_5BIT
  *         @arg @ref LL_SPI_DATASIZE_6BIT
  *         @arg @ref LL_SPI_DATASIZE_7BIT
  *         @arg @ref LL_SPI_DATASIZE_8BIT
  *         @arg @ref LL_SPI_DATASIZE_9BIT
  *         @arg @ref LL_SPI_DATASIZE_10BIT
  *         @arg @ref LL_SPI_DATASIZE_11BIT
  *         @arg @ref LL_SPI_DATASIZE_12BIT
  *         @arg @ref LL_SPI_DATASIZE_13BIT
  *         @arg @ref LL_SPI_DATASIZE_14BIT
  *         @arg @ref LL_SPI_DATASIZE_15BIT
  *         @arg @ref LL_SPI_DATASIZE_16BIT
  *         @arg @ref LL_SPI_DATASIZE_17BIT
  *         @arg @ref LL_SPI_DATASIZE_18BIT
  *         @arg @ref LL_SPI_DATASIZE_19BIT
  *         @arg @ref LL_SPI_DATASIZE_20BIT
  *         @arg @ref LL_SPI_DATASIZE_21BIT
  *         @arg @ref LL_SPI_DATASIZE_22BIT
  *         @arg @ref LL_SPI_DATASIZE_23BIT
  *         @arg @ref LL_SPI_DATASIZE_24BIT
  *         @arg @ref LL_SPI_DATASIZE_25BIT
  *         @arg @ref LL_SPI_DATASIZE_26BIT
  *         @arg @ref LL_SPI_DATASIZE_27BIT
  *         @arg @ref LL_SPI_DATASIZE_28BIT
  *         @arg @ref LL_SPI_DATASIZE_29BIT
  *         @arg @ref LL_SPI_DATASIZE_30BIT
  *         @arg @ref LL_SPI_DATASIZE_31BIT
  *         @arg @ref LL_SPI_DATASIZE_32BIT
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_data_size(spi_regs_t *SPIx, uint32_t size)
{
    MODIFY_REG(SPIx->CTRL0, SPI_CTRL0_DATA_FRAME_SIZE, size);
}

/**
  * @brief  Get frame data size
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | DFS32
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_DATASIZE_4BIT
  *         @arg @ref LL_SPI_DATASIZE_5BIT
  *         @arg @ref LL_SPI_DATASIZE_6BIT
  *         @arg @ref LL_SPI_DATASIZE_7BIT
  *         @arg @ref LL_SPI_DATASIZE_8BIT
  *         @arg @ref LL_SPI_DATASIZE_9BIT
  *         @arg @ref LL_SPI_DATASIZE_10BIT
  *         @arg @ref LL_SPI_DATASIZE_11BIT
  *         @arg @ref LL_SPI_DATASIZE_12BIT
  *         @arg @ref LL_SPI_DATASIZE_13BIT
  *         @arg @ref LL_SPI_DATASIZE_14BIT
  *         @arg @ref LL_SPI_DATASIZE_15BIT
  *         @arg @ref LL_SPI_DATASIZE_16BIT
  *         @arg @ref LL_SPI_DATASIZE_17BIT
  *         @arg @ref LL_SPI_DATASIZE_18BIT
  *         @arg @ref LL_SPI_DATASIZE_19BIT
  *         @arg @ref LL_SPI_DATASIZE_20BIT
  *         @arg @ref LL_SPI_DATASIZE_21BIT
  *         @arg @ref LL_SPI_DATASIZE_22BIT
  *         @arg @ref LL_SPI_DATASIZE_23BIT
  *         @arg @ref LL_SPI_DATASIZE_24BIT
  *         @arg @ref LL_SPI_DATASIZE_25BIT
  *         @arg @ref LL_SPI_DATASIZE_26BIT
  *         @arg @ref LL_SPI_DATASIZE_27BIT
  *         @arg @ref LL_SPI_DATASIZE_28BIT
  *         @arg @ref LL_SPI_DATASIZE_29BIT
  *         @arg @ref LL_SPI_DATASIZE_30BIT
  *         @arg @ref LL_SPI_DATASIZE_31BIT
  *         @arg @ref LL_SPI_DATASIZE_32BIT
  */
__STATIC_INLINE uint32_t ll_spi_get_data_size(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SPI_CTRL0_DATA_FRAME_SIZE));
}

/**
  * @brief  Set the length of the control word for the Microwire frame format
  * @note   This bit should be written only when SPI is disabled (SSI_EN = 0) for correct operation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | CFS
  *
  * @param  SPIx SPI instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_SPI_MW_CMDSIZE_1BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_2BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_3BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_4BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_5BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_6BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_7BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_8BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_9BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_10BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_11BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_12BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_13BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_14BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_15BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_16BIT
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_control_frame_size(spi_regs_t *SPIx, uint32_t size)
{
    MODIFY_REG(SPIx->CTRL0, SPI_CTRL0_CTRL_FRAME_SIZE, size);
}

/**
  * @brief  Get the length of the control word for the Microwire frame format
  * @note   This bit should be written only when SPI is disabled (SSI_EN = 0) for correct operation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | CFS
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_MW_CMDSIZE_1BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_2BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_3BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_4BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_5BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_6BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_7BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_8BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_9BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_10BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_11BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_12BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_13BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_14BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_15BIT
  *         @arg @ref LL_SPI_MW_CMDSIZE_16BIT
  */
__STATIC_INLINE uint32_t ll_spi_get_control_frame_size(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SPI_CTRL0_CTRL_FRAME_SIZE));
}

/**
  * @brief  Enable SPI test mode
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SRL
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_test_mode(spi_regs_t *SPIx)
{
    SET_BITS(SPIx->CTRL0, SPI_CTRL0_SHIFT_REG_LOOP);
}

/**
  * @brief  Disable SPI test mode
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SRL
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_test_mode(spi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->CTRL0, SPI_CTRL0_SHIFT_REG_LOOP);
}

/**
  * @brief  Check if SPI test mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SRL
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_test_mode(spi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->CTRL0, SPI_CTRL0_SHIFT_REG_LOOP) == (SPI_CTRL0_SHIFT_REG_LOOP));
}

/**
  * @brief  Enable slave output
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SLVOE
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_slave_out(spi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->CTRL0, SPI_CTRL0_S_OUT_EN);
}

/**
  * @brief  Disable slave output
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SLVOE
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_salve_out(spi_regs_t *SPIx)
{
    SET_BITS(SPIx->CTRL0, SPI_CTRL0_S_OUT_EN);
}

/**
  * @brief  Check if slave output is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SLVOE
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_slave_out(spi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->CTRL0, SPI_CTRL0_S_OUT_EN) != (SPI_CTRL0_S_OUT_EN));
}

/**
  * @brief  Set transfer direction mode
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | TMOD
  *
  * @param  SPIx SPI instance
  * @param  transfer_direction This parameter can be one of the following values:
  *         @arg @ref LL_SPI_FULL_DUPLEX
  *         @arg @ref LL_SPI_SIMPLEX_TX
  *         @arg @ref LL_SPI_SIMPLEX_RX
  *         @arg @ref LL_SPI_READ_EEPROM
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_transfer_direction(spi_regs_t *SPIx, uint32_t transfer_direction)
{
    MODIFY_REG(SPIx->CTRL0, SPI_CTRL0_XFE_MODE, transfer_direction);
}

/**
  * @brief  Get transfer direction mode
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | TMOD
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_FULL_DUPLEX
  *         @arg @ref LL_SPI_SIMPLEX_TX
  *         @arg @ref LL_SPI_SIMPLEX_RX
  *         @arg @ref LL_SPI_READ_EEPROM
  */
__STATIC_INLINE uint32_t ll_spi_get_transfer_direction(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SPI_CTRL0_XFE_MODE));
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
  * @param  SPIx SPI instance
  * @param  clock_polarity This parameter can be one of the following values:
  *         @arg @ref LL_SPI_SCPOL_LOW
  *         @arg @ref LL_SPI_SCPOL_HIGH
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_clock_polarity(spi_regs_t *SPIx, uint32_t clock_polarity)
{
    MODIFY_REG(SPIx->CTRL0, SPI_CTRL0_SERIAL_CLK_POL, clock_polarity);
}

/**
  * @brief  Get clock polarity
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SCPOL
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_SCPOL_LOW
  *         @arg @ref LL_SPI_SCPOL_HIGH
  */
__STATIC_INLINE uint32_t ll_spi_get_clock_polarity(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SPI_CTRL0_SERIAL_CLK_POL));
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
  * @param  SPIx SPI instance
  * @param  clock_phase This parameter can be one of the following values:
  *         @arg @ref LL_SPI_SCPHA_1EDGE
  *         @arg @ref LL_SPI_SCPHA_2EDGE
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_clock_phase(spi_regs_t *SPIx, uint32_t clock_phase)
{
    MODIFY_REG(SPIx->CTRL0, SPI_CTRL0_SERIAL_CLK_PHASE, clock_phase);
}

/**
  * @brief  Get clock phase
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | SCPHA
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_SCPHA_1EDGE
  *         @arg @ref LL_SPI_SCPHA_2EDGE
  */
__STATIC_INLINE uint32_t ll_spi_get_clock_phase(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SPI_CTRL0_SERIAL_CLK_PHASE));
}

/**
  * @brief  Set serial protocol used
  * @note   This bit should be written only when SPI is disabled (SSI_EN = 0) for correct operation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | FRF
  *
  * @param  SPIx SPI instance
  * @param  standard This parameter can be one of the following values:
  *         @arg @ref LL_SPI_PROTOCOL_MOTOROLA
  *         @arg @ref LL_SPI_PROTOCOL_TI
  *         @arg @ref LL_SPI_PROTOCOL_MICROWIRE
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_standard(spi_regs_t *SPIx, uint32_t standard)
{
    MODIFY_REG(SPIx->CTRL0, SPI_CTRL0_FRAME_FORMAT, standard);
}

/**
  * @brief  Get serial protocol used
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | FRF
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_PROTOCOL_MOTOROLA
  *         @arg @ref LL_SPI_PROTOCOL_TI
  *         @arg @ref LL_SPI_PROTOCOL_MICROWIRE
  */
__STATIC_INLINE uint32_t ll_spi_get_standard(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SPI_CTRL0_FRAME_FORMAT));
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
  * @param  SPIx SPI instance
  * @param  size This parameter can be one of the following values: 0 ~ 65535
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_receive_size(spi_regs_t *SPIx, uint32_t size)
{
    MODIFY_REG(SPIx->CTRL1, SPI_CTRL1_NUM_DATA_FRAME, size);
}

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
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 65535
  */
__STATIC_INLINE uint32_t ll_spi_get_receive_size(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL1, SPI_CTRL1_NUM_DATA_FRAME));
}

/**
  * @brief  Enable SPI peripheral
  *
  *  Register|BitsName
  *  --------|--------
  *  SSI_EN | EN
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable(spi_regs_t *SPIx)
{
    SET_BITS(SPIx->SSI_EN, SPI_SSI_EN);
}

/**
  * @brief  Disable SPI peripheral
  * @note   When disabling the SPI, follow the procedure described in the Reference Manual.
  *
  *  Register|BitsName
  *  --------|--------
  *  SSI_EN | EN
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable(spi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->SSI_EN, SPI_SSI_EN);
}

/**
  * @brief  Check if SPI peripheral is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  SSI_EN | EN
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled(spi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->SSI_EN, SPI_SSI_EN) == (SPI_SSI_EN));
}

/**
  * @brief  Enable Handshake in Microwire mode
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MHS
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_micro_handshake(spi_regs_t *SPIx)
{
    SET_BITS(SPIx->MW_CTRL, SPI_MW_CTRL_MW_HSG);
}

/**
  * @brief  Disable Handshake in Microwire mode
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MHS
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_micro_handshake(spi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->MW_CTRL, SPI_MW_CTRL_MW_HSG);
}

/**
  * @brief  Check if Handshake in Microwire mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MHS
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_micro_handshake(spi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->MW_CTRL, SPI_MW_CTRL_MW_HSG) == (SPI_MW_CTRL_MW_HSG));
}

/**
  * @brief  Set transfer direction mode in Microwire mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MDD
  *
  * @param  SPIx SPI instance
  * @param  transfer_direction This parameter can be one of the following values:
  *         @arg @ref LL_SPI_MICROWIRE_RX
  *         @arg @ref LL_SPI_MICROWIRE_TX
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_micro_transfer_direction(spi_regs_t *SPIx, uint32_t transfer_direction)
{
    MODIFY_REG(SPIx->MW_CTRL, SPI_MW_CTRL_MW_DIR_DW, transfer_direction);
}

/**
  * @brief  Get transfer direction mode in Microwire mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MDD
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_MICROWIRE_RX
  *         @arg @ref LL_SPI_MICROWIRE_TX
  */
__STATIC_INLINE uint32_t ll_spi_get_micro_transfer_direction(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->MW_CTRL, SPI_MW_CTRL_MW_DIR_DW));
}

/**
  * @brief  Set transfer mode in Microwire mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MWMOD
  *
  * @param  SPIx SPI instance
  * @param  transfer_mode This parameter can be one of the following values:
  *         @arg @ref LL_SPI_MICROWIRE_NON_SEQUENTIAL
  *         @arg @ref LL_SPI_MICROWIRE_SEQUENTIAL
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_micro_transfer_mode(spi_regs_t *SPIx, uint32_t transfer_mode)
{
    MODIFY_REG(SPIx->MW_CTRL, SPI_MW_CTRL_MW_XFE_MODE, transfer_mode);
}

/**
  * @brief  Get transfer mode in Microwire mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  MWC | MWMOD
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_MICROWIRE_NON_SEQUENTIAL
  *         @arg @ref LL_SPI_MICROWIRE_SEQUENTIAL
  */
__STATIC_INLINE uint32_t ll_spi_get_micro_transfer_mode(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->MW_CTRL, SPI_MW_CTRL_MW_XFE_MODE));
}

/**
  * @brief  Enable slave select
  *
  *  Register|BitsName
  *  --------|--------
  *  SE | SLAVE1
  *  SE | SLAVE0
  *
  * @param  SPIx SPI instance
  * @param  ss This parameter can be one of the following values:
  *         @arg @ref LL_SPI_SLAVE1
  *         @arg @ref LL_SPI_SLAVE0
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_ss(spi_regs_t *SPIx, uint32_t ss)
{
    SET_BITS(SPIx->S_EN, ss);
}

/**
  * @brief  Disable slave select
  *
  *  Register|BitsName
  *  --------|--------
  *  SE | SLAVE1
  *  SE | SLAVE0
  *
  * @param  SPIx SPI instance
  * @param  ss This parameter can be one of the following values:
  *         @arg @ref LL_SPI_SLAVE1
  *         @arg @ref LL_SPI_SLAVE0
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_ss(spi_regs_t *SPIx, uint32_t ss)
{
    CLEAR_BITS(SPIx->S_EN, ss);
}

/**
  * @brief  Check if slave select is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  SE | SLAVE1
  *  SE | SLAVE0
  *
  * @param  SPIx SPI instance
  * @param  ss This parameter can be one of the following values:
  *         @arg @ref LL_SPI_SLAVE1
  *         @arg @ref LL_SPI_SLAVE0
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_ss(spi_regs_t *SPIx, uint32_t ss)
{
    return (READ_BITS(SPIx->S_EN, ss) == ss);
}

/**
  * @brief  Set baud rate prescaler
  * @note   These bits should not be changed when communication is ongoing. SPI BaudRate = fPCLK/Prescaler.
  *
  *  Register|BitsName
  *  --------|--------
  *  BAUD | SCKDIV
  *
  * @param  SPIx SPI instance
  * @param  baud_rate This parameter can be one even value between 2 and 65534, if the value is 0, the SCLK is disable.
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_baud_rate_prescaler(spi_regs_t *SPIx, uint32_t baud_rate)
{
    WRITE_REG(SPIx->BAUD, baud_rate);
}

/**
  * @brief  Get baud rate prescaler
  *
  *  Register|BitsName
  *  --------|--------
  *  BAUD | SCKDIV
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one even value between 2 and 65534.
  */
__STATIC_INLINE uint32_t ll_spi_get_baud_rate_prescaler(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->BAUD, SPI_BAUD_CLK_DIV));
}

/**
  * @brief  Set threshold of TXFIFO that triggers an TXE event
  *
  *  Register|BitsName
  *  --------|--------
  *  TXFTL | TFT
  *
  * @param  SPIx SPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ (LL_SPI_x_FIFO_DEPTH - 1)
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_tx_fifo_threshold(spi_regs_t *SPIx, uint32_t threshold)
{
    WRITE_REG(SPIx->TX_FIFO_TL, threshold);
}

/**
  * @brief  Get threshold of TXFIFO that triggers an TXE event
  *
  *  Register|BitsName
  *  --------|--------
  *  TXFTL | TFT
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ (LL_SPI_x_FIFO_DEPTH - 1)
  */
__STATIC_INLINE uint32_t ll_spi_get_tx_fifo_threshold(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->TX_FIFO_TL, SPI_TX_FIFO_TL_TX_FIFO_THD));
}

/**
  * @brief  Set threshold of RXFIFO that triggers an RXNE event
  *
  *  Register|BitsName
  *  --------|--------
  *  RXFTL | RFT
  *
  * @param  SPIx SPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ (LL_SPI_x_FIFO_DEPTH - 1)
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_rx_fifo_threshold(spi_regs_t *SPIx, uint32_t threshold)
{
    WRITE_REG(SPIx->RX_FIFO_TL, threshold);
}

/**
  * @brief  Get threshold of RXFIFO that triggers an RXNE event
  *
  *  Register|BitsName
  *  --------|--------
  *  RXFTL | RFT
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ (LL_SPI_x_FIFO_DEPTH - 1)
  */
__STATIC_INLINE uint32_t ll_spi_get_rx_fifo_threshold(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->RX_FIFO_TL, SPI_RX_FIFO_TL_RX_FIFO_THD));
}

/**
  * @brief  Get FIFO Transmission Level
  *
  *  Register|BitsName
  *  --------|--------
  *  TXFL | TXTFL
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ LL_SPI_x_FIFO_DEPTH
  */
__STATIC_INLINE uint32_t ll_spi_get_tx_fifo_level(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->TX_FIFO_LEVEL, SPI_TX_FIFO_LEVEL_TX_FIFO_LEVEL));
}

/**
  * @brief  Get FIFO reception Level
  *
  *  Register|BitsName
  *  --------|--------
  *  RXFL | RXTFL
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ LL_SPI_x_FIFO_DEPTH
  */
__STATIC_INLINE uint32_t ll_spi_get_rx_fifo_level(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->RX_FIFO_LEVEL, SPI_RX_FIFO_LEVEL_RX_FIFO_LEVEL));
}

/** @} */

/** @defgroup SPI_LL_EF_IT_Management IT_Management
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
  * @param  SPIx SPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_SPI_IM_MST(not effect in SPIS)
  *         @arg @ref LL_SPI_IM_RXF
  *         @arg @ref LL_SPI_IM_RXO
  *         @arg @ref LL_SPI_IM_RXU
  *         @arg @ref LL_SPI_IM_TXO
  *         @arg @ref LL_SPI_IM_TXE
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_it(spi_regs_t *SPIx, uint32_t mask)
{
    SET_BITS(SPIx->INT_MASK, mask);
}

/**
  * @brief  Disable interrupt
  * @note   This bit controls the generation of an interrupt when an event occurs.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTMASK | INTMASK
  *
  * @param  SPIx SPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_SPI_IM_MST(not effect in SPIS)
  *         @arg @ref LL_SPI_IM_RXF
  *         @arg @ref LL_SPI_IM_RXO
  *         @arg @ref LL_SPI_IM_RXU
  *         @arg @ref LL_SPI_IM_TXO
  *         @arg @ref LL_SPI_IM_TXE
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_it(spi_regs_t *SPIx, uint32_t mask)
{
    CLEAR_BITS(SPIx->INT_MASK, mask);
}

/**
  * @brief  Check if interrupt is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  INTMASK | INTMASK
  *
  * @param  SPIx SPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_SPI_IM_MST(not effect in SPIS)
  *         @arg @ref LL_SPI_IM_RXF
  *         @arg @ref LL_SPI_IM_RXO
  *         @arg @ref LL_SPI_IM_RXU
  *         @arg @ref LL_SPI_IM_TXO
  *         @arg @ref LL_SPI_IM_TXE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_it(spi_regs_t *SPIx, uint32_t mask)
{
    return (READ_BITS(SPIx->INT_MASK, mask) == mask);
}

/** @} */

/** @defgroup SPI_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Get SPI status
  *
  *  Register|BitsName
  *  --------|--------
  *  STAT | STAT
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_SPI_SR_DCOL(no effect in SPIS)
  *         @arg @ref LL_SPI_SR_TXE
  *         @arg @ref LL_SPI_SR_RFF
  *         @arg @ref LL_SPI_SR_RFNE
  *         @arg @ref LL_SPI_SR_TFE
  *         @arg @ref LL_SPI_SR_TFNF
  *         @arg @ref LL_SPI_SR_BUSY
  */
__STATIC_INLINE uint32_t ll_spi_get_status(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_REG(SPIx->STAT));
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
  * @param  SPIx SPI instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_SPI_SR_DCOL(no effect in SPIS)
  *         @arg @ref LL_SPI_SR_TXE
  *         @arg @ref LL_SPI_SR_RFF
  *         @arg @ref LL_SPI_SR_RFNE
  *         @arg @ref LL_SPI_SR_TFE
  *         @arg @ref LL_SPI_SR_TFNF
  *         @arg @ref LL_SPI_SR_BUSY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_active_flag(spi_regs_t *SPIx, uint32_t flag)
{
    return (READ_BITS(SPIx->STAT, flag) == (flag));
}

/**
  * @brief  Get SPI interrupt flags
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | INTSTAT
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_SPI_IM_MST(not effect in SPIS)
  *         @arg @ref LL_SPI_IM_RXF
  *         @arg @ref LL_SPI_IM_RXO
  *         @arg @ref LL_SPI_IM_RXU
  *         @arg @ref LL_SPI_IM_TXO
  *         @arg @ref LL_SPI_IM_TXE
  */
__STATIC_INLINE uint32_t ll_spi_get_it_flag(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_REG(SPIx->INT_STAT));
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
  * @param  SPIx SPI instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_SPI_IM_MST(not effect in SPIS)
  *         @arg @ref LL_SPI_IM_RXF
  *         @arg @ref LL_SPI_IM_RXO
  *         @arg @ref LL_SPI_IM_RXU
  *         @arg @ref LL_SPI_IM_TXO
  *         @arg @ref LL_SPI_IM_TXE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_it_flag(spi_regs_t *SPIx, uint32_t flag)
{
    return (READ_BITS(SPIx->INT_STAT, flag) == flag);
}

/**
  * @brief  Get SPI raw interrupt flags
  *
  *  Register|BitsName
  *  --------|--------
  *  RAW_INTSTAT | RAW_INTSTAT
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_SPI_IM_MST(not effect in SPIS)
  *         @arg @ref LL_SPI_IM_RXF
  *         @arg @ref LL_SPI_IM_RXO
  *         @arg @ref LL_SPI_IM_RXU
  *         @arg @ref LL_SPI_IM_TXO
  *         @arg @ref LL_SPI_IM_TXE
  */
__STATIC_INLINE uint32_t ll_spi_get_raw_if_flag(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_REG(SPIx->RAW_INT_STAT));
}

/**
  * @brief  Clear transmit FIFO overflow error flag
  * @note   Clearing this flag is done by reading TXOIC register
  *
  *  Register|BitsName
  *  --------|--------
  *  TXOIC | TXOIC
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_flag_txo(spi_regs_t *SPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = SPIx->TX_FIFO_OIC;
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
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_flag_rxo(spi_regs_t *SPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = SPIx->RX_FIFO_OIC;
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
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_flag_rxu(spi_regs_t *SPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = SPIx->RX_FIFO_UIC;
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
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_flag_mst(spi_regs_t *SPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = SPIx->MULTI_M_IC;
    (void) tmpreg;
}

/**
  * @brief  Clear all error flag
  * @note   Clearing this flag is done by reading INTCLR register
  *
  *  Register|BitsName
  *  --------|--------
  *  INTCLR | INTCLR
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_flag_all(spi_regs_t *SPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = SPIx->INT_CLR;
    (void) tmpreg;
}

/** @} */

/** @defgroup SPI_LL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Enable DMA Tx
  *
  *  Register|BitsName
  *  --------|--------
  *  DMAC | TDMAE
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_dma_req_tx(spi_regs_t *SPIx)
{
    SET_BITS(SPIx->DMA_CTRL, SPI_DMA_CTRL_TX_DMA_EN);
}

/**
  * @brief  Disable DMA Tx
  *
  *  Register|BitsName
  *  --------|--------
  *  DMAC | TDMAE
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_dma_req_tx(spi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->DMA_CTRL, SPI_DMA_CTRL_TX_DMA_EN);
}

/**
  * @brief  Check if DMA Tx is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  DMAC | TDMAE
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_dma_req_tx(spi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->DMA_CTRL, SPI_DMA_CTRL_TX_DMA_EN) == (SPI_DMA_CTRL_TX_DMA_EN));
}

/**
  * @brief  Enable DMA Rx
  *
  *  Register|BitsName
  *  --------|--------
  *  DMAC | RDMAE
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_dma_req_rx(spi_regs_t *SPIx)
{
    SET_BITS(SPIx->DMA_CTRL, SPI_DMA_CTRL_RX_DMA_EN);
}

/**
  * @brief  Disable DMA Rx
  *
  *  Register|BitsName
  *  --------|--------
  *  DMAC | RDMAE
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_dma_req_rx(spi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->DMA_CTRL, SPI_DMA_CTRL_RX_DMA_EN);
}

/**
  * @brief  Check if DMA Rx is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  DMAC | RDMAE
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_dma_req_rx(spi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->DMA_CTRL, SPI_DMA_CTRL_RX_DMA_EN) == (SPI_DMA_CTRL_RX_DMA_EN));
}

/**
  * @brief  Set threshold of TXFIFO that triggers an DMA Tx request event
  *
  *  Register|BitsName
  *  --------|--------
  *  DMATDL | DMATDL
  *
  * @param  SPIx SPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_dma_tx_fifo_threshold(spi_regs_t *SPIx, uint32_t threshold)
{
    WRITE_REG(SPIx->DMA_TX_DL, threshold);
}

/**
  * @brief  Get threshold of TXFIFO that triggers an DMA Tx request event
  *
  *  Register|BitsName
  *  --------|--------
  *  DMATDL | DMATDL
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 7
  */
__STATIC_INLINE uint32_t ll_spi_get_dma_tx_fifo_threshold(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->DMA_TX_DL, SPI_DMA_TX_DL_DMA_TX_DL));
}

/**
  * @brief  Set threshold of RXFIFO that triggers an DMA Rx request event
  *
  *  Register|BitsName
  *  --------|--------
  *  DMARDL | DMARDL
  *
  * @param  SPIx SPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_dma_rx_fifo_threshold(spi_regs_t *SPIx, uint32_t threshold)
{
    WRITE_REG(SPIx->DMA_RX_DL, threshold);
}

/**
  * @brief  Get threshold of RXFIFO that triggers an DMA Rx request event
  *
  *  Register|BitsName
  *  --------|--------
  *  DMARDL | DMARDL
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 7
  */
__STATIC_INLINE uint32_t ll_spi_get_dma_rx_fifo_threshold(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->DMA_RX_DL, SPI_DMA_RX_DL_DMA_RX_DL));
}

/** @} */

/** @defgroup SPI_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Write 8-Bits in the data register
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  SPIx SPI instance
  * @param  tx_data Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void ll_spi_transmit_data8(spi_regs_t *SPIx, uint8_t tx_data)
{
    *((__IOM uint8_t *)&SPIx->DATA) = tx_data;
}

/**
  * @brief  Write 16-Bits in the data register
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  SPIx SPI instance
  * @param  tx_data Value between Min_Data=0x0000 and Max_Data=0xFFFF
  * @retval None
  */
__STATIC_INLINE void ll_spi_transmit_data16(spi_regs_t *SPIx, uint16_t tx_data)
{
    *((__IOM uint16_t *)&SPIx->DATA) = tx_data;
}

/**
  * @brief  Write 32-Bits in the data register
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  SPIx SPI instance
  * @param  tx_data Value between Min_Data=0x00000000 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_spi_transmit_data32(spi_regs_t *SPIx, uint32_t tx_data)
{
    *((__IOM uint32_t *)&SPIx->DATA) = tx_data;
}

/**
  * @brief  Read 8-Bits in the data register
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  SPIx SPI instance
  * @retval Rerturned Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t ll_spi_receive_data8(spi_regs_t *SPIx)
{
    return (uint8_t)(READ_REG(SPIx->DATA));
}

/**
  * @brief  Read 16-Bits in the data register
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  SPIx SPI instance
  * @retval Returned Value between Min_Data=0x0000 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint16_t ll_spi_receive_data16(spi_regs_t *SPIx)
{
    return (uint16_t)(READ_REG(SPIx->DATA));
}

/**
  * @brief  Read 32-Bits in the data register
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA | DATA
  *
  * @param  SPIx SPI instance
  * @retval Returned Value between Min_Data=0x00000000 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_spi_receive_data32(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_REG(SPIx->DATA));
}

/**
  * @brief  Set Rx sample delay
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  RX_SAMPLEDLY | RX_SAMPLEDLY
  *
  * @param  SPIx SPI instance
  * @param  delay This parameter can be one of the following values: 0 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_rx_sample_delay(spi_regs_t *SPIx, uint32_t delay)
{
    MODIFY_REG(SPIx->RX_SAMPLE_DLY, SPI_RX_SAMPLEDLY, delay);
}

/**
  * @brief  Get Rx sample delay
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  RX_SAMPLEDLY | RX_SAMPLEDLY
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 7
  */
__STATIC_INLINE uint32_t ll_spi_get_rx_sample_delay(spi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->RX_SAMPLE_DLY, SPI_RX_SAMPLEDLY));
}

/** @} */

/** @defgroup SPI_LL_EF_Init SPIM Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize SSI registers (Registers restored to their default values).
  * @param  SPIx SSI instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SSI registers are de-initialized
  *          - ERROR: SSI registers are not de-initialized
  */
error_status_t ll_spim_deinit(spi_regs_t *SPIx);

/**
  * @brief  Initialize SPIM registers according to the specified
  *         parameters in p_spi_init.
  * @param  SPIx SSI instance
  * @param  p_spi_init Pointer to a ll_spim_init_t structure that contains the configuration
  *                         information for the specified SPIM peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SSI registers are initialized according to p_spi_init content
  *          - ERROR: Problem occurred during SSI Registers initialization
  */
error_status_t ll_spim_init(spi_regs_t *SPIx, ll_spim_init_t *p_spi_init);

/**
  * @brief Set each field of a @ref ll_spim_init_t type structure to default value.
  * @param p_spi_init  Pointer to a @ref ll_spim_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_spim_struct_init(ll_spim_init_t *p_spi_init);

/** @} */

/** @defgroup SPIS_LL_Init SPIS Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize SSI registers (Registers restored to their default values).
  * @param  SPIx SSI instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SSI registers are de-initialized
  *          - ERROR: SSI registers are not de-initialized
  */
error_status_t ll_spis_deinit(spi_regs_t *SPIx);

/**
  * @brief  Initialize SSI registers according to the specified
  *         parameters in p_spi_init.
  * @param  SPIx SSI instance
  * @param  p_spi_init Pointer to a ll_spis_init_t structure that contains the configuration
  *                         information for the specified SPIS peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SSI registers are initialized according to p_spi_init content
  *          - ERROR: Problem occurred during SPI Registers initialization
  */
error_status_t ll_spis_init(spi_regs_t *SPIx, ll_spis_init_t *p_spi_init);

/**
  * @brief Set each field of a @ref ll_spis_init_t type structure to default value.
  * @param p_spi_init  Pointer to a @ref ll_spis_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_spis_struct_init(ll_spis_init_t *p_spi_init);
/** @} */

/** @} */

#endif /* SPIM || SPIS */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_SPI_H__ */

/** @} */

/** @} */

/** @} */

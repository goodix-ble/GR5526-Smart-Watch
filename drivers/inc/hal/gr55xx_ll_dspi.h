/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_dspi.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of DSPI LL library.
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

/** @defgroup LL_DSPI DSPI
  * @brief DSPI LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_DSPI_H__
#define __GR55XX_LL_DSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined(DSPI)

/** @defgroup DSPI_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup DSPI_LL_ES_INIT DSPI Exported init structure
  * @{
  */

/**
  * @brief  DSPI init structures definition
  */
typedef struct _ll_dspi_init_t
{
    uint32_t data_size;             /**< Specifies the DSPI data width.
                                         This parameter can be a value of @ref DSPI_LL_EC_DATASIZE.

                                         This feature can be modified afterwards using unitary function @ref ll_dspi_set_data_size().*/

    uint32_t baud_rate;             /**< Specifies the BaudRate prescaler value which will be used to configure the transmit and receive SCK clock.
                                         This parameter can be one even value between 0 and 7 @ref  DSPI_LL_EC_CLK

                                         This feature can be modified afterwards using unitary function ll_dspi_set_baud_rate_prescaler().*/

    uint32_t transmit_format;       /**< Specifies the DSPI transmission format.
                                         This parameter can be a value of @ref DSPI_LL_EC_TRANSFER_FORMATE.

                                         This feature can be modified afterwards using unitary function ll_dspi_set_transfer_format().*/

    uint32_t dspi_mode;             /**< Specifies the DSPI protocol mode.
                                         This parameter can be a value of @ref DSPI_LL_EC_PROTOCOL_MODE.

                                         This feature can be modified afterwards using unitary function ll_dspi_set_protocol_mode().*/
} ll_dspi_init_t;

/** @} */

/** @} */

/**
  * @defgroup  DSPI_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DSPI_LL_Exported_Constants DSPI Exported Constants
  * @{
  */

/** @defgroup DSPI_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags definitions which can be used with LL_DSPI_StatReg function
  * @{
  */
#define LL_DSPI_SR_FFE                          DSPI_STAT_FRE                       /**< Frame format error flag        */
#define LL_DSPI_SR_BUSY                         DSPI_STAT_BUSY                      /**< Busy flag                      */
#define LL_DSPI_SR_OVR                          DSPI_STAT_OVR                       /**< Overrun flag                   */
#define LL_DSPI_SR_MODF                         DSPI_STAT_MODF                      /**< Mode fault                     */
#define LL_DSPI_SR_TFE                          DSPI_STAT_TXE                       /**< Tx FIFO empty flag             */
#define LL_DSPI_SR_RFNE                         DSPI_STAT_RXNE                      /**< Rx FIFO not empty flag         */
#define LL_DSPI_SR_ALL                          (LL_DSPI_SR_FFE | \
                                                LL_DSPI_SR_BUSY | \
                                                LL_DSPI_SR_OVR  | \
                                                LL_DSPI_SR_MODF | \
                                                LL_DSPI_SR_TFE  | \
                                                LL_DSPI_SR_RFNE )                  /**< All flag         */
/** @} */

/** @defgroup DSPI_LL_EC_IT IT Defines
  * @brief    Interrupt definitions which can be used with LL_DSPI_Ctrl2Reg
  * @{
  */
#define LL_DSPI_IM_TXE                          DSPI_CR2_TXEIE                      /**< Transmit FIFO Empty Interrupt enable    */
#define LL_DSPI_IM_RXNE                         DSPI_CR2_RXNEIE                     /**< Receive FIFO not empty Interrupt enable */
#define LL_DSPI_IM_ER                           DSPI_CR2_ERRIE                      /**< Error interrupt enable, OVR, MODF */
/** @} */

/** @defgroup DSPI_LL_EC_PROTOCOL_MODE DSPI Protocol mode
  * @{
  */
#define LL_DSPI_PROT_MODE_SPI                   (0UL << DSPI_MODE_SPIMODE_Pos)      /**<  DSPI Normal SPI Interface         */
#define LL_DSPI_PROT_MODE_3W1L                  (1UL << DSPI_MODE_SPIMODE_Pos)      /**<  DSPI 3-Wire 1-Lane Interface      */
#define LL_DSPI_PROT_MODE_4W1L                  (2UL << DSPI_MODE_SPIMODE_Pos)      /**<  DSPI 4-Wire 1-Lane Interface      */
#define LL_DSPI_PROT_MODE_4W2L                  (3UL << DSPI_MODE_SPIMODE_Pos)      /**<  DSPI 4-Wire 2-Lane Interface      */
/** @} */

/** @defgroup DSPI_LL_EC_TRANSFER_FORMATE DSPI Transmission Format
  * @{
  */
#define LL_DSPI_TF_MSB_FIRST                    (0x0)                               /**<  DSPI Frame format MSB first      */
#define LL_DSPI_TF_LSB_FIRST                    DSPI_CR1_LSBFIRST                   /**<  DSPI Frame format LSB first      */
/** @} */

/** @defgroup DSPI_LL_EC_DATASIZE Datawidth
  * @{
  */
#define LL_DSPI_DATASIZE_4BIT                   (3UL << DSPI_CR2_DS_Pos)            /**< Data length for DSPI transfer:  4 bits */
#define LL_DSPI_DATASIZE_5BIT                   (4UL << DSPI_CR2_DS_Pos)            /**< Data length for DSPI transfer:  5 bits */
#define LL_DSPI_DATASIZE_6BIT                   (5UL << DSPI_CR2_DS_Pos)            /**< Data length for DSPI transfer:  6 bits */
#define LL_DSPI_DATASIZE_7BIT                   (6UL << DSPI_CR2_DS_Pos)            /**< Data length for DSPI transfer:  7 bits */
#define LL_DSPI_DATASIZE_8BIT                   (7UL << DSPI_CR2_DS_Pos)            /**< Data length for DSPI transfer:  8 bits */
#define LL_DSPI_DATASIZE_9BIT                   (8UL << DSPI_CR2_DS_Pos)            /**< Data length for DSPI transfer:  9 bits */
#define LL_DSPI_DATASIZE_10BIT                  (9UL << DSPI_CR2_DS_Pos)            /**< Data length for DSPI transfer: 10 bits */
#define LL_DSPI_DATASIZE_11BIT                  (10UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 11 bits */
#define LL_DSPI_DATASIZE_12BIT                  (11UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 12 bits */
#define LL_DSPI_DATASIZE_13BIT                  (12UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 13 bits */
#define LL_DSPI_DATASIZE_14BIT                  (13UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 14 bits */
#define LL_DSPI_DATASIZE_15BIT                  (14UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 15 bits */
#define LL_DSPI_DATASIZE_16BIT                  (15UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 16 bits */
#define LL_DSPI_DATASIZE_17BIT                  (16UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 17 bits */
#define LL_DSPI_DATASIZE_18BIT                  (17UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 18 bits */
#define LL_DSPI_DATASIZE_19BIT                  (18UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 19 bits */
#define LL_DSPI_DATASIZE_20BIT                  (19UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 20 bits */
#define LL_DSPI_DATASIZE_21BIT                  (20UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 21 bits */
#define LL_DSPI_DATASIZE_22BIT                  (21UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 22 bits */
#define LL_DSPI_DATASIZE_23BIT                  (22UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 23 bits */
#define LL_DSPI_DATASIZE_24BIT                  (23UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 24 bits */
#define LL_DSPI_DATASIZE_25BIT                  (24UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 25 bits */
#define LL_DSPI_DATASIZE_26BIT                  (25UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 26 bits */
#define LL_DSPI_DATASIZE_27BIT                  (26UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 27 bits */
#define LL_DSPI_DATASIZE_28BIT                  (27UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 28 bits */
#define LL_DSPI_DATASIZE_29BIT                  (28UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 29 bits */
#define LL_DSPI_DATASIZE_30BIT                  (29UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 30 bits */
#define LL_DSPI_DATASIZE_31BIT                  (30UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 31 bits */
#define LL_DSPI_DATASIZE_32BIT                  (31UL << DSPI_CR2_DS_Pos)           /**< Data length for DSPI transfer: 32 bits */
/** @} */

/** @defgroup DSPI_LL_EC_BIDIMODE Bidirectional data mode enable
  * @{
  */
#define LL_DSPI_BIDIMODE_2L_UNBID               0x00000000UL                        /**<  2-line unidirectional data mode selected */
#define LL_DSPI_BIDIMODE_1L_BID                 DSPI_CR1_BIDIDE                     /**<  1-line bidirectional data mode selected   */
/** @} */

/** @defgroup DSPI_LL_EC_2L_TRANSFER_MODE Transfer Mode in 2-Line Unidirectional Data Mode
  * @{
  */
#define LL_DSPI_2L_FULL_DUPLEX                  0x00000000UL                        /**< Full-Duplex mode. Rx and Tx transfer */
#define LL_DSPI_2L_SIMPLEX_RX                   DSPI_CR1_RXONLY                     /**< Simplex Rx mode.  Rx transfer only   */
/** @} */

/** @defgroup DSPI_LL_EC_1L_TRANSFER_MODE Transfer Mode in 1-Line idirectional Data Mode
  * @{
  */
#define LL_DSPI_1L_SIMPLEX_RX                   0x00000000UL                         /**< Simplex Rx mode.  Rx receive only  */
#define LL_DSPI_1L_SIMPLEX_TX                   DSPI_CR1_BIDIOE                      /**< Simplex Tx mode.  Rx transfer only   */
/** @} */

/** @defgroup DSPI_LL_EC_CLK DSPI Baud rate control
  * @{
  */
#define LL_DSPI_BAUD_RATE_2P1PCLK               (0x0 << DSPI_CR1_BAUD_Pos)              /**< DSPI Baud rate fPCLK / 2   */
#define LL_DSPI_BAUD_RATE_4P1PCLK               (0x1 << DSPI_CR1_BAUD_Pos)              /**< DSPI Baud rate fPCLK / 4   */
#define LL_DSPI_BAUD_RATE_8P1PCLK               (0x2 << DSPI_CR1_BAUD_Pos)              /**< DSPI Baud rate fPCLK / 8   */
#define LL_DSPI_BAUD_RATE_16P1PCLK              (0x3 << DSPI_CR1_BAUD_Pos)              /**< DSPI Baud rate fPCLK / 16  */
#define LL_DSPI_BAUD_RATE_32P1PCLK              (0x4 << DSPI_CR1_BAUD_Pos)              /**< DSPI Baud rate fPCLK / 32  */
#define LL_DSPI_BAUD_RATE_64P1PCLK              (0x5 << DSPI_CR1_BAUD_Pos)              /**< DSPI Baud rate fPCLK / 64  */
#define LL_DSPI_BAUD_RATE_128P1PCLK             (0x6 << DSPI_CR1_BAUD_Pos)              /**< DSPI Baud rate fPCLK / 128 */
#define LL_DSPI_BAUD_RATE_256PCLK               (0x7 << DSPI_CR1_BAUD_Pos)              /**< DSPI Baud rate fPCLK / 256 */
/** @} */

/** @defgroup DSPI_LL_EC_PHASE Clock Phase
  * @{
  */
#define LL_DSPI_SCPHA_1EDGE                     0x00000000UL                            /**< First clock transition is the first data capture edge  */
#define LL_DSPI_SCPHA_2EDGE                     (DSPI_CR1_CPHA)                         /**< Second clock transition is the first data capture edge */
/** @} */

/** @defgroup DSPI_LL_EC_POLARITY Clock Polarity
  * @{
  */
#define LL_DSPI_SCPOL_LOW                       0x00000000UL                            /**< Clock to 0 when idle */
#define LL_DSPI_SCPOL_HIGH                      (DSPI_CR1_CPOL)                         /**< Clock to 1 when idle */
/** @} */

/** @defgroup DSPI_LL_EC_DMA DMA Defines
  * @{
  */
#define LL_DSPI_DMA_TX_DIS                      0x00000000UL                        /**< Disable the transmit FIFO DMA channel */
#define LL_DSPI_DMA_TX_EN                       DSPI_CR2_TXDMAEN                    /**< Enable the transmit FIFO DMA channel  */

#define LL_DSPI_DMA_RX_DIS                      0x00000000UL                        /**< Disable the receive FIFO DMA channel */
#define LL_DSPI_DMA_RX_EN                       DSPI_CR2_RXDMAEN                    /**< Enable the receive FIFO DMA channel  */
/** @} */

/** @defgroup DSPI_LL_EC_LAST_DMA_TF  Last DMA transfer or receive
  * @{
  */
#define LL_DSPI_DMA_LTX_EVEN                    0x00000000UL                        /**< Number of data to transfer is even */
#define LL_DSPI_DMA_LTX_ODD                     DSPI_CR2_LDMA_TX                    /**< Number of data to transfer is odd  */

#define LL_DSPI_DMA_LRX_EVEN                    0x00000000UL                        /**< Number of data to transfer is even */
#define LL_DSPI_DMA_LRX_ODD                     DSPI_CR2_LDMA_RX                    /**< Number of data to transfer is odd  */
/** @} */

/** @defgroup DSPI_LL_EC_FIFO_LEVEL FIFO level defines
  * @{
  */
#define LL_DSPI_FRXTH_1P2                       DSPI_CR1_FIFOTH_8                    /**< FIFO level is 1/2  (8 bytes) */
#define LL_DSPI_FRXTH_1P4                       DSPI_CR1_FIFOTH_4                    /**< FIFO level is 1/4  (4 bytes)  */
#define LL_DSPI_FRXTH_1P8                       DSPI_CR1_FIFOTH_2                    /**< FIFO level is 1/8  (2 bytes) */
#define LL_DSPI_FRXTH_1P16                      DSPI_CR1_FIFOTH_1                    /**< FIFO level is 1/16 (1 bytes)  */
/** @} */

/** @defgroup DSPI_LL_EC_DCX Value of Data-Versus-Command information used in Display SPI Protocol Modes.
  * @{
  */
#define LL_DSPI_DCX_CMD                         0x00000000UL                        /**< Data-Versus-Command value to 0(CMD) */
#define LL_DSPI_DCX_DATA                        (DSPI_MODE_DCX)                     /**< Data-Versus-Command value to 1(DATA) */
/** @} */

/** @defgroup DSPI_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */

/**
  * @brief LL DSPI InitStrcut default configuartion
  */
#define LL_DSPI_DEFAULT_CONFIG                          \
{                                                       \
    .transmit_format    = LL_DSPI_TF_MSB_FIRST,         \
    .data_size          = LL_DSPI_DATASIZE_8BIT,        \
    .dspi_mode          = LL_DSPI_PROT_MODE_4W1L,       \
    .baud_rate          = LL_DSPI_BAUD_RATE_8P1PCLK,    \
}

/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DSPI_LL_Exported_Macros DSPI Exported Macros
  * @{
  */

/** @defgroup DSPI_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in DSPI register
  * @param  __instance__ DSPI instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_DSPI_WriteReg(__instance__, __REG__, __VALUE__)   WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in DSPI register
  * @param  __instance__ DSPI instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_DSPI_ReadReg(__instance__, __REG__)               READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup DSPI_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup DSPI_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable DSPI peripheral
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | EN
  *
  * @param  DSPIx DSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_dspi_enable(dspi_regs_t *DSPIx)
{
    SET_BITS(DSPIx->CTRL1, DSPI_CR1_EN);
}

/**
  * @brief  Disable DSPI peripheral
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | EN
  *
  * @param  DSPIx DSPI instance
  * @retval None
  */

__STATIC_INLINE void ll_dspi_disable(dspi_regs_t *DSPIx)
{
    CLEAR_BITS(DSPIx->CTRL1, DSPI_CR1_EN);
}

/**
  * @brief  Check if DSPI peripheral is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | EN
  *
  * @param  DSPIx DSPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dspi_is_enabled(dspi_regs_t *DSPIx)
{
    return (READ_BITS(DSPIx->CTRL1, DSPI_CR1_EN) == (DSPI_CR1_EN));
}

/**
  * @brief  Set clock polarity
  * @note   This bit should not be changed when communication is ongoing.
  *         This bit is not used in DSPI TI mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | SCPOL
  *
  * @param  DSPIx DSPI instance
  * @param  clock_polarity This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_SCPOL_LOW
  *         @arg @ref LL_DSPI_SCPOL_HIGH
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_clock_polarity(dspi_regs_t *DSPIx, uint32_t clock_polarity)
{
    MODIFY_REG(DSPIx->CTRL1, DSPI_CR1_CPOL, clock_polarity);
}

/**
  * @brief  Get clock polarity
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | SCPOL
  *
  * @param  DSPIx DSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DSPI_SCPOL_LOW
  *         @arg @ref LL_DSPI_SCPOL_HIGH
  */
__STATIC_INLINE uint32_t ll_dspi_get_clock_polarity(dspi_regs_t *DSPIx)
{
    return (uint32_t)(READ_BITS(DSPIx->CTRL1, DSPI_CR1_CPOL));
}

/**
  * @brief  Set clock phase
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | SCPHA
  *
  * @param  DSPIx DSPI instance
  * @param  clock_phase This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_SCPHA_1EDGE
  *         @arg @ref LL_DSPI_SCPHA_2EDGE
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_clock_phase(dspi_regs_t *DSPIx, uint32_t clock_phase)
{
    MODIFY_REG(DSPIx->CTRL1, DSPI_CR1_CPHA, clock_phase);
}

/**
  * @brief  Get clock phase
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | SCPHA
  *
  * @param  DSPIx DSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DSPI_SCPHA_1EDGE
  *         @arg @ref LL_DSPI_SCPHA_2EDGE
  */
__STATIC_INLINE uint32_t ll_dspi_get_clock_phase(dspi_regs_t *DSPIx)
{
    return (uint32_t)(READ_BITS(DSPIx->CTRL1, DSPI_CR1_CPHA));
}

/**
  * @brief  Set data frame format for transmitting/receiving the data
  * @note   This bit should be written only when DSPI is disabled (EN = 0) for correct operation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | LSBFIRST
  *
  * @param  DSPIx DSPI instance
  * @param  frf This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_TF_MSB_FIRST
  *         @arg @ref LL_DSPI_TF_LSB_FIRST
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_frame_format(dspi_regs_t *DSPIx, uint32_t frf)
{
    MODIFY_REG(DSPIx->CTRL1, DSPI_CR1_LSBFIRST, frf);
}

/**
  * @brief  Get data frame format for transmitting/receiving the data
  * @note   This bit should be written only when DSPI is disabled (SSI_EN = 0) for correct operation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | LSBFIRST
  *
  * @param  DSPIx DSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DSPI_TF_MSB_FIRST
  *         @arg @ref LL_DSPI_TF_LSB_FIRST
  */
__STATIC_INLINE uint32_t ll_dspi_get_frame_format(dspi_regs_t *DSPIx)
{
    return (uint32_t)(READ_BITS(DSPIx->CTRL1, DSPI_CR1_LSBFIRST));
}

/**
  * @brief  Set frame data size
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | DS
  *
  * @param  DSPIx DSPI instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_DATASIZE_4BIT
  *         @arg @ref LL_DSPI_DATASIZE_5BIT
  *         @arg @ref LL_DSPI_DATASIZE_6BIT
  *         @arg @ref LL_DSPI_DATASIZE_7BIT
  *         @arg @ref LL_DSPI_DATASIZE_8BIT
  *         @arg @ref LL_DSPI_DATASIZE_9BIT
  *         @arg @ref LL_DSPI_DATASIZE_10BIT
  *         @arg @ref LL_DSPI_DATASIZE_11BIT
  *         @arg @ref LL_DSPI_DATASIZE_12BIT
  *         @arg @ref LL_DSPI_DATASIZE_13BIT
  *         @arg @ref LL_DSPI_DATASIZE_14BIT
  *         @arg @ref LL_DSPI_DATASIZE_15BIT
  *         @arg @ref LL_DSPI_DATASIZE_16BIT
  *         @arg @ref LL_DSPI_DATASIZE_17BIT
  *         @arg @ref LL_DSPI_DATASIZE_18BIT
  *         @arg @ref LL_DSPI_DATASIZE_19BIT
  *         @arg @ref LL_DSPI_DATASIZE_20BIT
  *         @arg @ref LL_DSPI_DATASIZE_21BIT
  *         @arg @ref LL_DSPI_DATASIZE_22BIT
  *         @arg @ref LL_DSPI_DATASIZE_23BIT
  *         @arg @ref LL_DSPI_DATASIZE_24BIT
  *         @arg @ref LL_DSPI_DATASIZE_25BIT
  *         @arg @ref LL_DSPI_DATASIZE_26BIT
  *         @arg @ref LL_DSPI_DATASIZE_27BIT
  *         @arg @ref LL_DSPI_DATASIZE_28BIT
  *         @arg @ref LL_DSPI_DATASIZE_29BIT
  *         @arg @ref LL_DSPI_DATASIZE_30BIT
  *         @arg @ref LL_DSPI_DATASIZE_31BIT
  *         @arg @ref LL_DSPI_DATASIZE_32BIT
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_data_size(dspi_regs_t *DSPIx, uint32_t size)
{
    MODIFY_REG(DSPIx->CTRL2, DSPI_CR2_DS, size);
}

/**
  * @brief  Get frame data size
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | DS
  *
  * @param  DSPIx DSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DSPI_DATASIZE_4BIT
  *         @arg @ref LL_DSPI_DATASIZE_5BIT
  *         @arg @ref LL_DSPI_DATASIZE_6BIT
  *         @arg @ref LL_DSPI_DATASIZE_7BIT
  *         @arg @ref LL_DSPI_DATASIZE_8BIT
  *         @arg @ref LL_DSPI_DATASIZE_9BIT
  *         @arg @ref LL_DSPI_DATASIZE_10BIT
  *         @arg @ref LL_DSPI_DATASIZE_11BIT
  *         @arg @ref LL_DSPI_DATASIZE_12BIT
  *         @arg @ref LL_DSPI_DATASIZE_13BIT
  *         @arg @ref LL_DSPI_DATASIZE_14BIT
  *         @arg @ref LL_DSPI_DATASIZE_15BIT
  *         @arg @ref LL_DSPI_DATASIZE_16BIT
  *         @arg @ref LL_DSPI_DATASIZE_17BIT
  *         @arg @ref LL_DSPI_DATASIZE_18BIT
  *         @arg @ref LL_DSPI_DATASIZE_19BIT
  *         @arg @ref LL_DSPI_DATASIZE_20BIT
  *         @arg @ref LL_DSPI_DATASIZE_21BIT
  *         @arg @ref LL_DSPI_DATASIZE_22BIT
  *         @arg @ref LL_DSPI_DATASIZE_23BIT
  *         @arg @ref LL_DSPI_DATASIZE_24BIT
  *         @arg @ref LL_DSPI_DATASIZE_25BIT
  *         @arg @ref LL_DSPI_DATASIZE_26BIT
  *         @arg @ref LL_DSPI_DATASIZE_27BIT
  *         @arg @ref LL_DSPI_DATASIZE_28BIT
  *         @arg @ref LL_DSPI_DATASIZE_29BIT
  *         @arg @ref LL_DSPI_DATASIZE_30BIT
  *         @arg @ref LL_DSPI_DATASIZE_31BIT
  *         @arg @ref LL_DSPI_DATASIZE_32BIT
  */
__STATIC_INLINE uint32_t ll_dspi_get_data_size(dspi_regs_t *DSPIx)
{
    return (uint32_t)(READ_BITS(DSPIx->CTRL2, DSPI_CR2_DS));
}

/**
  * @brief  Set baud rate
  * @note   These bits should not be changed when communication is ongoing.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | BR
  *
  * @param  DSPIx DSPI instance
  * @param  baud_rate This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_BAUD_RATE_2P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_4P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_8P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_16P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_32P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_64P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_128P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_256PCLK
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_baud_rate(dspi_regs_t *DSPIx, uint32_t baud_rate)
{
    MODIFY_REG(DSPIx->CTRL1, DSPI_CR1_BAUD, baud_rate);
}

/**
  * @brief  Get baud rate
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | BR
  *
  * @param  DSPIx DSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DSPI_BAUD_RATE_2P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_4P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_8P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_16P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_32P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_64P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_128P1PCLK
  *         @arg @ref LL_DSPI_BAUD_RATE_256PCLK
  */
__STATIC_INLINE uint32_t ll_dspi_get_baud_rate(dspi_regs_t *DSPIx)
{
    return (uint32_t)(READ_BITS(DSPIx->CTRL1, DSPI_CR1_BAUD));
}

/**
  * @brief  Set bidirectional data mode enable
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | BIDIMODE
  *
  * @param  DSPIx DSPI instance
  * @param  bid_mode This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_BIDIMODE_2L_UNBID
  *         @arg @ref LL_DSPI_BIDIMODE_1L_BID
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_bidirectional_mode(dspi_regs_t *DSPIx, uint32_t bid_mode)
{
    MODIFY_REG(DSPIx->CTRL1, DSPI_CR1_BIDIDE, bid_mode);
}

/**
  * @brief  Get baud rate
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | BIDIMODE
  *
  * @param  DSPIx DSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DSPI_BIDIMODE_2L_UNBID
  *         @arg @ref LL_DSPI_BIDIMODE_1L_BID
  */
__STATIC_INLINE uint32_t ll_dspi_get_bidirectional_mode(dspi_regs_t *DSPIx)
{
    return (uint32_t)(READ_BITS(DSPIx->CTRL1, DSPI_CR1_BIDIDE));
}

/**
  * @brief  Set transfer direction mode in bidirectional mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | BIDIOE
  *
  * @param  DSPIx DSPI instance
  * @param  transfer_direction This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_1L_SIMPLEX_RX
  *         @arg @ref LL_DSPI_1L_SIMPLEX_TX
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_transfer_direction_bidirectional(dspi_regs_t *DSPIx, uint32_t transfer_direction)
{
    MODIFY_REG(DSPIx->CTRL1, DSPI_CR1_BIDIOE, transfer_direction);
}

/**
  * @brief  Get transfer direction mode in bidirectional mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | BIDIOE
  *
  * @param  DSPIx DSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DSPI_1L_SIMPLEX_RX
  *         @arg @ref LL_DSPI_1L_SIMPLEX_TX
  */
__STATIC_INLINE uint32_t ll_dspi_get_transfer_direction_bidirectional(dspi_regs_t *DSPIx)
{
    return (uint32_t)(READ_BITS(DSPIx->CTRL1, DSPI_CR1_BIDIOE));
}

/**
  * @brief  Set transfer direction mode in unbidirectional mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | RXONLY
  *
  * @param  DSPIx DSPI instance
  * @param  transfer_direction This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_2L_FULL_DUPLEX
  *         @arg @ref LL_DSPI_2L_SIMPLEX_RX
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_transfer_direction_unbidirectional(dspi_regs_t *DSPIx, uint32_t transfer_direction)
{
    MODIFY_REG(DSPIx->CTRL1, DSPI_CR1_RXONLY, transfer_direction);
}

/**
  * @brief  Get transfer direction mode in bidirectional mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | RXONLY
  *
  * @param  DSPIx DSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DSPI_2L_FULL_DUPLEX
  *         @arg @ref LL_DSPI_2L_SIMPLEX_RX
  */
__STATIC_INLINE uint32_t ll_dspi_get_transfer_direction_unbidirectional(dspi_regs_t *DSPIx)
{
    return (uint32_t)(READ_BITS(DSPIx->CTRL1, DSPI_CR1_RXONLY));
}

/**
  * @brief  Set threshold of TXFIFO that triggers an TXE event
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | FRXTH
  *
  * @param  DSPIx DSPI instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_FRXTH_1P2
  *         @arg @ref LL_DSPI_FRXTH_1P4
  *         @arg @ref LL_DSPI_FRXTH_1P8
  *         @arg @ref LL_DSPI_FRXTH_1P16
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_rx_fifo_threshold(dspi_regs_t *DSPIx, uint32_t threshold)
{
    MODIFY_REG(DSPIx->CTRL1, DSPI_CR1_FIFOTH, threshold);
}

/**
  * @brief  Get threshold of TXFIFO that triggers an TXE event
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL1 | FRXTH
  *
  * @param  DSPIx DSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DSPI_FRXTH_1P2
  *         @arg @ref LL_DSPI_FRXTH_1P4
  *         @arg @ref LL_DSPI_FRXTH_1P8
  *         @arg @ref LL_DSPI_FRXTH_1P16
  */
__STATIC_INLINE uint32_t ll_dspi_get_rx_fifo_threshold(dspi_regs_t *DSPIx)
{
    return (uint32_t)(READ_BITS(DSPIx->CTRL1, DSPI_CR1_FIFOTH));
}

/**
  * @brief  FIFO Pointer Reset
  *
  *  Register|BitsName
  *  --------|--------
  *  DSPI | FPRST
  *
  * @param  DSPIx DSPI instance
  */
__STATIC_INLINE void ll_dspi_flush_fifo(dspi_regs_t *DSPIx)
{
    SET_BITS(DSPIx->CTRL1, DSPI_CR1_REFIFO);
    CLEAR_BITS(DSPIx->CTRL1, DSPI_CR1_REFIFO);
}

/**
  * @brief  Master selection
  *
  *  Register|BitsName
  *  --------|--------
  *  DSPI | MSTR
  *
  * @param  DSPIx DSPI instance
  */
__STATIC_INLINE void ll_dspi_master_select(dspi_regs_t *DSPIx)
{
    SET_BITS(DSPIx->CTRL1, DSPI_CR1_MSTR);
}

/**
  * @brief  Last DMA transfer for transmission
  * @note This bit is used in data packing mode, to define if the total number of data to transmit by
  * DMA is odd or even. It has significance only if the TXDMAEN bit in the CTRL2 register is set and
  * if packing mode is used (data length =< 8-bit and write access to DATA is 16-bit wide)
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | LDMA_TX
  *
  * @param  DSPIx DSPI instance
  * @param  bid_mode This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_DMA_LTX_EVEN
  *         @arg @ref LL_DSPI_DMA_LTX_ODD
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_last_dma_tx_packing(dspi_regs_t *DSPIx, uint32_t bid_mode)
{
    MODIFY_REG(DSPIx->CTRL1, DSPI_CR2_LDMA_TX, bid_mode);
}

/**
  * @brief  Last DMA transfer for reception
  * @note This bit is used in data packing mode, to define if the total number of data to receive
  * by DMA is odd or even. It has significance only if the RXDMAEN bit in the CTRL2 register
  * is set and if packing mode is used (data length =< 8-bit and write access to DATA is 16-bit wide)
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | LDMA_RX
  *
  * @param  DSPIx DSPI instance
  * @param  bid_mode This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_DMA_LRX_EVEN
  *         @arg @ref LL_DSPI_DMA_LRX_ODD
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_last_dma_rx_packing(dspi_regs_t *DSPIx, uint32_t bid_mode)
{
    MODIFY_REG(DSPIx->CTRL2, DSPI_CR2_LDMA_RX, bid_mode);
}

/**
  * @brief  Enable DMA Tx
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | TXDMAEN
  *
  * @param  DSPIx DSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_dspi_enable_dma_tx(dspi_regs_t *DSPIx)
{
    SET_BITS(DSPIx->CTRL2, DSPI_CR2_TXDMAEN);
}

/**
  * @brief  Disable DMA Tx
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | TXDMAEN
  *
  * @param  DSPIx DSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_dspi_disable_dma_tx(dspi_regs_t *DSPIx)
{
    CLEAR_BITS(DSPIx->CTRL2, DSPI_CR2_TXDMAEN);
}

/**
  * @brief  Check if DMA Tx is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | TXDMAEN
  *
  * @param  DSPIx DSPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dspi_is_enabled_dma_tx(dspi_regs_t *DSPIx)
{
    return (READ_BITS(DSPIx->CTRL2, DSPI_CR2_TXDMAEN) == (DSPI_CR2_TXDMAEN));
}

/**
  * @brief  Enable DMA Rx
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | RXDMAEN
  *
  * @param  DSPIx DSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_dspi_enable_dma_rx(dspi_regs_t *DSPIx)
{
    SET_BITS(DSPIx->CTRL2, DSPI_CR2_RXDMAEN);
}

/**
  * @brief  Disable DMA Rx
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | RXDMAEN
  *
  * @param  DSPIx DSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_dspi_disable_dma_rx(dspi_regs_t *DSPIx)
{
    CLEAR_BITS(DSPIx->CTRL2, DSPI_CR2_RXDMAEN);
}

/**
  * @brief  Check if DMA Rx is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | RXDMAEN
  *
  * @param  DSPIx DSPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dspi_is_enabled_dma_rx(dspi_regs_t *DSPIx)
{
    return (READ_BITS(DSPIx->CTRL2, DSPI_CR2_RXDMAEN) == (DSPI_CR2_RXDMAEN));
}

/**
  * @brief Value of Data-Versus-Command information
  *
  *  Register|BitsName
  *  --------|--------
  *  MODE | DCX
  *
  * @param  DSPIx DSPI instance
  * @param  dcx This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_DCX_CMD
  *         @arg @ref LL_DSPI_DCX_DATA
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_dcx(dspi_regs_t *DSPIx, uint32_t dcx)
{
    MODIFY_REG(DSPIx->MODE, DSPI_MODE_DCX, dcx);
}

/**
  * @brief DSPI Protocol Mode
  *
  *  Register|BitsName
  *  --------|--------
  *  MODE | SPIMODE
  *
  * @param  DSPIx DSPI instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_PROT_MODE_SPI
  *         @arg @ref LL_DSPI_PROT_MODE_3W1L
  *         @arg @ref LL_DSPI_PROT_MODE_4W1L
  *         @arg @ref LL_DSPI_PROT_MODE_4W2L
  * @retval None
  */
__STATIC_INLINE void ll_dspi_set_mode(dspi_regs_t *DSPIx, uint32_t mode)
{
    MODIFY_REG(DSPIx->MODE, DSPI_MODE_SPIMODE, mode);
}

/**
  * @brief Get DSPI Protocol Mode
  *
  *  Register|BitsName
  *  --------|--------
  *  MODE | SPIMODE
  *
  * @param  DSPIx DSPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DSPI_PROT_MODE_SPI
  *         @arg @ref LL_DSPI_PROT_MODE_3W1L
  *         @arg @ref LL_DSPI_PROT_MODE_4W1L
  *         @arg @ref LL_DSPI_PROT_MODE_4W2L
  */
__STATIC_INLINE uint32_t ll_dspi_get_mode(dspi_regs_t *DSPIx)
{
    return (READ_BITS(DSPIx->CTRL2, DSPI_MODE_SPIMODE));
}

/**
  * @brief  SS output enable
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | SSOE
  *
  * @param  DSPIx DSPI instance
  * @retval None
  */
__STATIC_INLINE void ll_dspi_ss_out_enable(dspi_regs_t *DSPIx)
{
    SET_BITS(DSPIx->CTRL2, DSPI_CR2_SSOE);
}

/**
  * @brief  Enable interrupt
  * @note   This bit controls the generation of an interrupt when an event occurs.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | INTMASK
  *
  * @param  DSPIx DSPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_IM_TXE
  *         @arg @ref LL_DSPI_IM_RXNE
  *         @arg @ref LL_DSPI_IM_ER
  * @retval None
  */
__STATIC_INLINE void ll_dspi_enable_it(dspi_regs_t *DSPIx, uint32_t mask)
{
    SET_BITS(DSPIx->CTRL2, mask);
}

/**
  * @brief  Disable interrupt
  * @note   This bit controls the generation of an interrupt when an event occurs.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | INTMASK
  *
  * @param  DSPIx DSPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_IM_TXE
  *         @arg @ref LL_DSPI_IM_RXNE
  *         @arg @ref LL_DSPI_IM_ER
  * @retval None
  */
__STATIC_INLINE void ll_dspi_disable_it(dspi_regs_t *DSPIx, uint32_t mask)
{
    CLEAR_BITS(DSPIx->CTRL2, mask);
}

/**
  * @brief  Check if interrupt is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL2 | INTMASK
  *
  * @param  DSPIx DSPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_IM_TXE
  *         @arg @ref LL_DSPI_IM_RXNE
  *         @arg @ref LL_DSPI_IM_ER
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dspi_is_enabled_it(dspi_regs_t *DSPIx, uint32_t mask)
{
    return (READ_BITS(DSPIx->CTRL2, mask) == mask);
}

/**
  * @brief  Get DSPI status
  *
  *  Register|BitsName
  *  --------|--------
  *  STAT | STAT
  *
  * @param  DSPIx DSPI instance
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_DSPI_SR_FFE
  *         @arg @ref LL_DSPI_SR_BUSY
  *         @arg @ref LL_DSPI_SR_OVR
  *         @arg @ref LL_DSPI_SR_MODF
  *         @arg @ref LL_DSPI_SR_TFE
  *         @arg @ref LL_DSPI_SR_RFNE
  */
__STATIC_INLINE uint32_t ll_dspi_get_status(dspi_regs_t *DSPIx)
{
    return (uint32_t)(READ_REG(DSPIx->STAT) & LL_DSPI_SR_ALL);
}

/**
  * @brief  Check active flag
  *
  *  Register|BitsName
  *  --------|--------
  *  STAT | FFE
  *  STAT | BUSY
  *  STAT | OVR
  *  STAT | MODF
  *  STAT | TFE
  *  STAT | TFNF
  *
  * @param  DSPIx DSPI instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_SR_FFE
  *         @arg @ref LL_DSPI_SR_BUSY
  *         @arg @ref LL_DSPI_SR_OVR
  *         @arg @ref LL_DSPI_SR_MODF
  *         @arg @ref LL_DSPI_SR_TFE
  *         @arg @ref LL_DSPI_SR_RFNE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dspi_is_active_flag(dspi_regs_t *DSPIx, uint32_t flag)
{
    return (READ_BITS(DSPIx->STAT, flag) == (flag));
}

/**
  * @brief  Get DSPI interrupt flags
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | INTSTAT
  *
  * @param  DSPIx DSPI instance
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_DSPI_SR_FFE
  *         @arg @ref LL_DSPI_SR_BUSY
  *         @arg @ref LL_DSPI_SR_OVR
  *         @arg @ref LL_DSPI_SR_MODF
  *         @arg @ref LL_DSPI_SR_TFE
  *         @arg @ref LL_DSPI_SR_RFNE
  */
__STATIC_INLINE uint32_t ll_dspi_get_it_flag(dspi_regs_t *DSPIx)
{
    return (uint32_t)(READ_REG(DSPIx->STAT) & LL_DSPI_SR_ALL);
}

/**
  * @brief  Check interrupt flag
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | FFE
  *  INTSTAT | BUSY
  *  INTSTAT | OVR
  *  INTSTAT | MODF
  *  INTSTAT | TFE
  *  INTSTAT | RFNE
  *
  * @param  DSPIx DSPI instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_DSPI_SR_FFE
  *         @arg @ref LL_DSPI_SR_BUSY
  *         @arg @ref LL_DSPI_SR_OVR
  *         @arg @ref LL_DSPI_SR_MODF
  *         @arg @ref LL_DSPI_SR_TFE
  *         @arg @ref LL_DSPI_SR_RFNE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dspi_is_it_flag(dspi_regs_t *DSPIx, uint32_t flag)
{
    return (READ_BITS(DSPIx->STAT, flag) == flag);
}
/** @} */

/** @defgroup DSPI_LL_EF_Init DSPI Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize DSPI registers (Registers restored to their default values).
  * @param  DSPIx DSPI instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: DSPIx registers are de-initialized
  *          - ERROR: DSPIx registers are not de-initialized
  */
error_status_t ll_dspi_deinit(dspi_regs_t *DSPIx);

/**
  * @brief  Initialize DSPI registers according to the specified
  *         parameters in p_dspi_init.
  * @param  DSPIx DSPI instance
  * @param  p_dspi_init Pointer to a ll_dspi_init_t structure that contains the configuration
  *                         information for the specified DSPI peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SSI registers are initialized according to p_spi_init content
  *          - ERROR: Problem occurred during SSI Registers initialization
  */
error_status_t ll_dspi_init(dspi_regs_t *DSPIx, ll_dspi_init_t *p_dspi_init);

/**
  * @brief Set each field of a @ref ll_dspi_init_t type structure to default value.
  * @param p_dspi_init  Pointer to a @ref ll_dspi_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_dspi_struct_init(ll_dspi_init_t *p_dspi_init);

/** @} */

/** @} */

#endif /* DSPI */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_DSPI_H__ */

/** @} */

/** @} */

/** @} */


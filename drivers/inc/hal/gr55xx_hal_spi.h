/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_spi.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of SPI HAL library.
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

/** @defgroup HAL_SPI SPI
  * @brief SPI HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_SPI_H__
#define __GR55xx_HAL_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_spi.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_SPI_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_SPI_state HAL SPI state
  * @{
  */

/**
  * @brief HAL SPI State Enumerations definition
  */
typedef enum
{
    HAL_SPI_STATE_RESET        = 0x00,    /**< Peripheral not initialized.                          */
    HAL_SPI_STATE_READY        = 0x01,    /**< Peripheral initialized and ready for use.            */
    HAL_SPI_STATE_BUSY         = 0x02,    /**< An internal process is ongoing.                      */
    HAL_SPI_STATE_BUSY_TX      = 0x12,    /**< Data Transmission process is ongoing.                */
    HAL_SPI_STATE_BUSY_RX      = 0x22,    /**< Data Reception process is ongoing.                   */
    HAL_SPI_STATE_BUSY_TX_RX   = 0x32,    /**< Data Transmission and Reception process is ongoing.  */
    HAL_SPI_STATE_ABORT        = 0x08,    /**< Peripheral with abort request ongoing.               */
    HAL_SPI_STATE_ERROR        = 0x04     /**< Peripheral in error.                                 */

} hal_spi_state_t;

/** @} */

/** @} */

/** @addtogroup HAL_SPI_STRUCTURES Structures
  * @{
  */

/** @defgroup SPI_Configuration SPI Configuration
  * @{
  */

/**
  * @brief SPI init Structure definition
  */
typedef struct _spi_init
{
    uint32_t data_size;             /**< Specifies the SPI data size.
                                         This parameter can be a value of @ref SPI_Data_Size. */

    uint32_t clock_polarity;        /**< Specifies the serial clock steady state.
                                         This parameter can be a value of @ref SPI_Clock_Polarity. */

    uint32_t clock_phase;           /**< Specifies the clock active edge for the bit capture.
                                         This parameter can be a value of @ref SPI_Clock_Phase. */

    uint32_t baudrate_prescaler;    /**< Specifies the BaudRate prescaler value which will be
                                         used to configure the transmit and receive SCK clock.
                                         @note The communication clock is derived from the master
                                         clock. The slave clock does not need to be set. */

    uint32_t ti_mode;               /**< Specifies if the TI mode is enabled or not.
                                         This parameter can be a value of @ref SPI_TI_Mode. */

    uint32_t slave_select;          /**< Specifies the slaves to be selected.
                                         This parameter can be a value of @ref SPI_Slave_Select. */

    uint32_t rx_sample_delay;       /**< Specifies the RX sample delay. It is used to delay the sample of the RX input port.
                                         This parameter can be a number between 0 and 0x7. */
} spi_init_t;
/** @} */
/** @} */


/**
  * @defgroup  HAL_SPI_MACRO Defines
  * @{
  */

/** @defgroup SOFT_CS_MAGIC_NUMBER SOFT CS Signal Description
  * @{
  */
#define SPI_SOFT_CS_MAGIC_NUMBER            0xDEADBEAF  /**< CS magic number. */
/** @} */

/** @defgroup CS_STA_STATE State from Software CS Assert
  * @{
  */
/**
  * @brief State from Software CS Assert
  */
#define CS_STA_STATE_POLL_TX                0x01    /**< CS ASSERT when Transferring Starts by Polling */
#define CS_STA_STATE_POLL_RX                0x02    /**< CS ASSERT when Receiving Starts by Polling */
#define CS_STA_STATE_POLL_TX_RX             0x03    /**< CS ASSERT when DUPLEX Starts by Polling */
#define CS_STA_STATE_POLL_EEPREAD           0x04    /**< CS ASSERT when EEPREAD Starts by Polling */
#define CS_STA_STATE_IT_TX                  0x05    /**< CS ASSERT when Transferring Starts by Interrupt */
#define CS_STA_STATE_IT_RX                  0x06    /**< CS ASSERT when Receiving Starts by Interrupt */
#define CS_STA_STATE_IT_TX_RX               0x07    /**< CS ASSERT when DUPLEX Starts by Interrupt */
#define CS_STA_STATE_IT_EEPREAD             0x08    /**< CS ASSERT when EEPREAD Starts by Interrupt */
#define CS_STA_STATE_DMA_TX                 0x09    /**< CS ASSERT when Transferring Starts by DMA */
#define CS_STA_STATE_DMA_RX                 0x0A    /**< CS ASSERT when Receiving Starts by DMA */
#define CS_STA_STATE_DMA_TX_RX              0x0B    /**< CS ASSERT when DUPLEX Starts by DMA */
#define CS_STA_STATE_DMA_EEPREAD            0x0C    /**< CS ASSERT when EEPREAD Starts by DMA */
#define CS_STA_STATE_DMA_LLP_TX             0x0D    /**< CS ASSERT when Transferring Starts by DMA LLP */
#define CS_STA_STATE_DMA_SCATTER_RX         0x0E    /**< CS ASSERT when Receiving Starts by DMA Scatter */
/** @} */

/** @defgroup CS_END_STATE State from Software CS De-Assert
  * @{
  */
/**
  * @brief State from Software CS De-Assert
  */
#define CS_END_STATE_POLL_TX                0x81    /**< CS DE-ASSERT when Transferring Ends by Polling */
#define CS_END_STATE_POLL_RX                0x82    /**< CS DE-ASSERT when Receiving Ends by Polling */
#define CS_END_STATE_POLL_TX_RX             0x83    /**< CS DE-ASSERT when DUPLEX Ends by Polling */
#define CS_END_STATE_POLL_EEPREAD           0x84    /**< CS DE-ASSERT when EEPREAD Ends by Polling */
#define CS_END_STATE_TX_CPLT                0x90    /**< CS DE-ASSERT when Transferring Ends by IT/DMA */
#define CS_END_STATE_RX_CPLT                0x91    /**< CS DE-ASSERT when Receiving Ends by IT/DMA */
#define CS_END_STATE_TX_RX_CPLT             0x92    /**< CS DE-ASSERT when DUPLEX Ends by IT/DMA */
#define CS_END_STATE_XFER_ERR               0x93    /**< CS DE-ASSERT when xfer Error by IT/DMA */
#define CS_END_STATE_TX_ABORT_CPLT          0x94    /**< CS DE-ASSERT when abort The transfer Ends by IT/DMA */
#define CS_END_STATE_RX_ABORT_CPLT          0x95    /**< CS DE-ASSERT when abort the receive Ends by IT/DMA */
#define CS_END_STATE_ABORT_CPLT             0x96    /**< CS DE-ASSERT when abort the tx/rx by IT/DMA */
/** @} */
/** @} */

/** @addtogroup HAL_SPI_STRUCTURES Structures
  * @{
  */
/** @defgroup SPI_handle SPI handle
  * @{
  */

/**
  * @brief SPI handle Structure definition
  */
typedef struct _spi_handle
{
    spi_regs_t              *p_instance;        /**< SPI registers base address        */

    spi_init_t              init;               /**< SPI communication parameters      */

    __IO uint32_t           soft_cs_magic;      /**< if equals to @ref SPI_SOFT_CS_MAGIC_NUMBER, control the CS signal by software */

    uint8_t                 *p_tx_buffer;       /**< Pointer to SPI Tx transfer Buffer */

    __IO uint32_t           tx_xfer_size;       /**< SPI Tx Transfer size              */

    __IO uint32_t           tx_xfer_count;      /**< SPI Tx Transfer Counter           */

    uint8_t                 *p_rx_buffer;       /**< Pointer to SPI Rx transfer Buffer */

    __IO uint32_t           rx_xfer_size;       /**< SPI Rx Transfer size              */

    __IO uint32_t           rx_xfer_count;      /**< SPI Rx Transfer Counter           */

    void (*write_fifo)(struct _spi_handle *p_spi); /**< Pointer to SPI Tx transfer FIFO write function */

    void (*read_fifo)(struct _spi_handle *p_spi);  /**< Pointer to SPI Rx transfer FIFO read function  */

    void (*read_write_fifo)(struct _spi_handle *p_spi);  /**< Pointer to SPI transfer FIFO read and write function  */

    dma_handle_t            *p_dmatx;           /**< SPI Tx DMA Handle parameters      */

    dma_handle_t            *p_dmarx;           /**< SPI Rx DMA Handle parameters      */

    __IO hal_lock_t         lock;               /**< Locking object                    */

    __IO hal_spi_state_t    state;              /**< SPI communication state           */

    __IO uint32_t           error_code;         /**< SPI Error code                    */

    uint32_t                timeout;            /**< Timeout for the SPI memory access */

    uint32_t                retention[9];       /**< SPI important register information. */
} spi_handle_t;
/** @} */

/** @} */

/** @addtogroup HAL_SPI_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_SPI_Callback Callback
  * @{
  */

/**
  * @brief HAL_SPI Callback function definition
  */

typedef struct _spi_callback
{
    void (*spi_msp_init)(spi_handle_t *p_spi);              /**< SPI init MSP callback                      */
    void (*spi_msp_deinit)(spi_handle_t *p_spi);            /**< SPI de-init MSP callback                   */
    void (*spi_error_callback)(spi_handle_t *p_spi);        /**< SPI error callback                         */
    void (*spi_abort_cplt_callback)(spi_handle_t *p_spi);   /**< SPI abort completed callback               */
    void (*spi_rx_cplt_callback)(spi_handle_t *p_spi);      /**< SPI rx transfer completed callback         */
    void (*spi_tx_cplt_callback)(spi_handle_t *p_spi);      /**< SPI tx transfer completed callback         */
    void (*spi_tx_rx_cplt_callback)(spi_handle_t *p_spi);   /**< SPI tx/rx transfer completed callback      */
    void (*spi_soft_cs_assert)(spi_handle_t *p_spi, uint32_t state);   /**< assert the cs signal by software setting */
    void (*spi_soft_cs_deassert)(spi_handle_t *p_spi, uint32_t state);   /**< deassert the cs signal by software setting */
} spi_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_SPI_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SPI_Exported_Constants SPI Exported Constants
  * @{
  */

/** @defgroup SPI_Direction SPI Direction
  * @{
  */

#define SPI_DIRECTION_FULL_DUPLEX       LL_SPI_FULL_DUPLEX      /**< Full Duplex: Transmit & Receive    */
#define SPI_DIRECTION_SIMPLEX_TX        LL_SPI_SIMPLEX_TX       /**< Simplex Tx: Transmit only          */
#define SPI_DIRECTION_SIMPLEX_RX        LL_SPI_SIMPLEX_RX       /**< Simplex Rx: Receive only           */
#define SPI_DIRECTION_READ_EEPROM       LL_SPI_READ_EEPROM      /**< Read EEPROM                        */

/** @} */

/** @defgroup SPI_Error_Code SPI Error Code
  * @{
  */
#define HAL_SPI_ERROR_NONE              ((uint32_t)0x00000000)  /**< No error           */
#define HAL_SPI_ERROR_TIMEOUT           ((uint32_t)0x00000001)  /**< Timeout error      */
#define HAL_SPI_ERROR_TRANSFER          ((uint32_t)0x00000002)  /**< Transfer error     */
#define HAL_SPI_ERROR_DMA               ((uint32_t)0x00000004)  /**< DMA transfer error */
#define HAL_SPI_ERROR_INVALID_PARAM     ((uint32_t)0x00000008)  /**< Invalid parameters error */
/** @} */

/** @defgroup SPI_Data_Size SPI Data Size
  * @{
  */

#define SPI_DATASIZE_4BIT               LL_SPI_DATASIZE_4BIT    /**< 4-bit  serial data transfer   */
#define SPI_DATASIZE_5BIT               LL_SPI_DATASIZE_5BIT    /**< 5-bit  serial data transfer   */
#define SPI_DATASIZE_6BIT               LL_SPI_DATASIZE_6BIT    /**< 6-bit  serial data transfer   */
#define SPI_DATASIZE_7BIT               LL_SPI_DATASIZE_7BIT    /**< 7-bit  serial data transfer   */
#define SPI_DATASIZE_8BIT               LL_SPI_DATASIZE_8BIT    /**< 8-bit  serial data transfer   */
#define SPI_DATASIZE_9BIT               LL_SPI_DATASIZE_9BIT    /**< 9-bit  serial data transfer   */
#define SPI_DATASIZE_10BIT              LL_SPI_DATASIZE_10BIT   /**< 10-bit serial data transfer   */
#define SPI_DATASIZE_11BIT              LL_SPI_DATASIZE_11BIT   /**< 11-bit serial data transfer   */
#define SPI_DATASIZE_12BIT              LL_SPI_DATASIZE_12BIT   /**< 12-bit serial data transfer   */
#define SPI_DATASIZE_13BIT              LL_SPI_DATASIZE_13BIT   /**< 13-bit serial data transfer   */
#define SPI_DATASIZE_14BIT              LL_SPI_DATASIZE_14BIT   /**< 14-bit serial data transfer   */
#define SPI_DATASIZE_15BIT              LL_SPI_DATASIZE_15BIT   /**< 15-bit serial data transfer   */
#define SPI_DATASIZE_16BIT              LL_SPI_DATASIZE_16BIT   /**< 16-bit serial data transfer   */
#define SPI_DATASIZE_17BIT              LL_SPI_DATASIZE_17BIT   /**< 17-bit serial data transfer   */
#define SPI_DATASIZE_18BIT              LL_SPI_DATASIZE_18BIT   /**< 18-bit serial data transfer   */
#define SPI_DATASIZE_19BIT              LL_SPI_DATASIZE_19BIT   /**< 19-bit serial data transfer   */
#define SPI_DATASIZE_20BIT              LL_SPI_DATASIZE_20BIT   /**< 20-bit serial data transfer   */
#define SPI_DATASIZE_21BIT              LL_SPI_DATASIZE_21BIT   /**< 21-bit serial data transfer   */
#define SPI_DATASIZE_22BIT              LL_SPI_DATASIZE_22BIT   /**< 22-bit serial data transfer   */
#define SPI_DATASIZE_23BIT              LL_SPI_DATASIZE_23BIT   /**< 23-bit serial data transfer   */
#define SPI_DATASIZE_24BIT              LL_SPI_DATASIZE_24BIT   /**< 24-bit serial data transfer   */
#define SPI_DATASIZE_25BIT              LL_SPI_DATASIZE_25BIT   /**< 25-bit serial data transfer   */
#define SPI_DATASIZE_26BIT              LL_SPI_DATASIZE_26BIT   /**< 26-bit serial data transfer   */
#define SPI_DATASIZE_27BIT              LL_SPI_DATASIZE_27BIT   /**< 27-bit serial data transfer   */
#define SPI_DATASIZE_28BIT              LL_SPI_DATASIZE_28BIT   /**< 28-bit serial data transfer   */
#define SPI_DATASIZE_29BIT              LL_SPI_DATASIZE_29BIT   /**< 29-bit serial data transfer   */
#define SPI_DATASIZE_30BIT              LL_SPI_DATASIZE_30BIT   /**< 30-bit serial data transfer   */
#define SPI_DATASIZE_31BIT              LL_SPI_DATASIZE_31BIT   /**< 31-bit serial data transfer   */
#define SPI_DATASIZE_32BIT              LL_SPI_DATASIZE_32BIT   /**< 32-bit serial data transfer   */

/** @} */

/** @defgroup SPI_Clock_Polarity SPI Clock Polarity
  * @{
  */

#define SPI_POLARITY_LOW                LL_SPI_SCPOL_LOW        /**< Inactive state of CLK is low  */
#define SPI_POLARITY_HIGH               LL_SPI_SCPOL_HIGH       /**< Inactive state of CLK is high */

/** @} */

/** @defgroup SPI_Clock_Phase SPI Clock Phase
  * @{
  */

#define SPI_PHASE_1EDGE                 LL_SPI_SCPHA_1EDGE      /**< CLK toggles at start of first data bit  */
#define SPI_PHASE_2EDGE                 LL_SPI_SCPHA_2EDGE      /**< CLK toggles in middle of first data bit */

/** @} */

/** @defgroup SPI_TI_Mode SPI TI Mode
  * @{
  */
#define SPI_TIMODE_DISABLE              ((uint32_t)0x00000000)  /**< SPI TI mode disable */

#define SPI_TIMODE_ENABLE               LL_SPI_PROTOCOL_TI      /**< SPI TI mode enable  */

/** @} */

/** @defgroup SPI_Slave_Select SPI Slave Select
  * @{
  */

#define SPI_SLAVE_SELECT_0              LL_SPI_SLAVE0                       /**< SPIM Select Slave 0    */
#define SPI_SLAVE_SELECT_1              LL_SPI_SLAVE1                       /**< SPIM Select Slave 1    */
#define SPI_SLAVE_SELECT_ALL            (LL_SPI_SLAVE0 | LL_SPI_SLAVE1)     /**< SPIM Select All Slave  */

/** @} */

/** @defgroup SPI_FIFO_LEVEL_MAX SPI FIFO Level Max
  * @{
  */
#define SPI_TX_FIFO_LEVEL_MAX           8                       /**< SPI TX FIFO Level Max Value */
#define SPI_RX_FIFO_LEVEL_MAX           8                       /**< SPI RX FIFO Level Max Value */
/** @} */

/** @defgroup SPI_Flags_definition SPI Flags Definition
  * @{
  */

#define SPI_FLAG_DCOL                   LL_SPI_SR_DCOL          /**< Data collision error flag      */
#define SPI_FLAG_TXE                    LL_SPI_SR_TXE           /**< Transmission error flag        */
#define SPI_FLAG_RFF                    LL_SPI_SR_RFF           /**< Rx FIFO full flag              */
#define SPI_FLAG_RFNE                   LL_SPI_SR_RFNE          /**< Rx FIFO not empty flag         */
#define SPI_FLAG_TFE                    LL_SPI_SR_TFE           /**< Tx FIFO empty flag             */
#define SPI_FLAG_TFNF                   LL_SPI_SR_TFNF          /**< Tx FIFO not full flag          */
#define SPI_FLAG_BUSY                   LL_SPI_SR_BUSY          /**< Busy flag                      */

/** @} */

/** @defgroup SPI_Interrupt_definition SPI Interrupt Definition
  * @{
  */

#define SPI_IT_MST                      LL_SPI_IS_MST           /**< Multi-Master Contention Interrupt flag     */
#define SPI_IT_RXF                      LL_SPI_IS_RXF           /**< Receive FIFO Full Interrupt flag           */
#define SPI_IT_RXO                      LL_SPI_IS_RXO           /**< Receive FIFO Overflow Interrupt  flag      */
#define SPI_IT_RXU                      LL_SPI_IS_RXU           /**< Receive FIFO Underflow Interrupt  flag     */
#define SPI_IT_TXO                      LL_SPI_IS_TXO           /**< Transmit FIFO Overflow Interrupt  flag     */
#define SPI_IT_TXE                      LL_SPI_IS_TXE           /**< Transmit FIFO Empty Interrupt  flag        */

/** @} */

/** @defgroup SPI_Timeout_definition SPI Timeout_definition
  * @{
  */
#define HAL_SPI_TIMEOUT_DEFAULT_VALUE   ((uint32_t)5000)        /**< 5s */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup SPI_Exported_Macros SPI Exported Macros
  * @{
  */

/** @brief  Reset SPI handle states.
  * @param  __HANDLE__ SPI handle.
  * @retval None
  */
#define __HAL_SPI_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_SPI_STATE_RESET)


/** @brief  Enable the specified SPI peripheral.
  * @param  __HANDLE__ Specifies the SPI Handle.
  * @retval None
  */
#define __HAL_SPI_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->SSI_EN, SPI_SSI_EN)


/** @brief  Disable the specified SPI peripheral.
  * @param  __HANDLE__ Specifies the SPI Handle.
  * @retval None
  */
#define __HAL_SPI_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->SSI_EN, SPI_SSI_EN)


/** @brief  Enable the SPI DMA TX Request.
  * @param  __HANDLE__ Specifies the SPI Handle.
  * @retval None
  */
#define __HAL_SPI_ENABLE_DMATX(__HANDLE__)                     SET_BITS((__HANDLE__)->p_instance->DMA_CTRL, SPI_DMA_CTRL_TX_DMA_EN)


/** @brief  Enable the SPI DMA RX Request.
  * @param  __HANDLE__ Specifies the SPI Handle.
  * @retval None
  */
#define __HAL_SPI_ENABLE_DMARX(__HANDLE__)                     SET_BITS((__HANDLE__)->p_instance->DMA_CTRL, SPI_DMA_CTRL_RX_DMA_EN)


/** @brief  Disable the SPI DMA TX Request.
  * @param  __HANDLE__ Specifies the SPI Handle.
  * @retval None
  */
#define __HAL_SPI_DISABLE_DMATX(__HANDLE__)                    CLEAR_BITS((__HANDLE__)->p_instance->DMA_CTRL, SPI_DMA_CTRL_TX_DMA_EN)


/** @brief  Disable the SPI DMA RX Request.
  * @param  __HANDLE__ Specifies the SPI Handle.
  * @retval None
  */
#define __HAL_SPI_DISABLE_DMARX(__HANDLE__)                    CLEAR_BITS((__HANDLE__)->p_instance->DMA_CTRL, SPI_DMA_CTRL_RX_DMA_EN)


/** @brief  Enable the specified SPI interrupts.
  * @param  __HANDLE__      Specifies the SPI Handle.
  * @param  __INTERRUPT__   Specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg @ref SPI_IT_MST Multi-Master Contention Interrupt enable
  *            @arg @ref SPI_IT_RXF Receive FIFO Full Interrupt enable
  *            @arg @ref SPI_IT_RXO Receive FIFO Overflow Interrupt enable
  *            @arg @ref SPI_IT_RXU Receive FIFO Underflow Interrupt enable
  *            @arg @ref SPI_IT_TXO Transmit FIFO Overflow Interrupt enable
  *            @arg @ref SPI_IT_TXE Transmit FIFO Empty Interrupt enable
  * @retval None
  */
#define __HAL_SPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)         SET_BITS((__HANDLE__)->p_instance->INT_MASK, (__INTERRUPT__))


/** @brief  Disable the specified SPI interrupts.
  * @param  __HANDLE__      Specifies the SPI handle.
  * @param  __INTERRUPT__   Specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg @ref SPI_IT_MST Multi-Master Contention Interrupt enable
  *            @arg @ref SPI_IT_RXF Receive FIFO Full Interrupt enable
  *            @arg @ref SPI_IT_RXO Receive FIFO Overflow Interrupt enable
  *            @arg @ref SPI_IT_RXU Receive FIFO Underflow Interrupt enable
  *            @arg @ref SPI_IT_TXO Transmit FIFO Overflow Interrupt enable
  *            @arg @ref SPI_IT_TXE Transmit FIFO Empty Interrupt enable
  * @retval None
  */
#define __HAL_SPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)        CLEAR_BITS((__HANDLE__)->p_instance->INT_MASK, (__INTERRUPT__))


/** @brief  Check whether the specified SPI interrupt source is enabled or not.
  * @param  __HANDLE__      Specifies the SPI Handle.
  * @param  __INTERRUPT__   Specifies the interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg @ref SPI_IT_MST Multi-Master Contention Interrupt enable
  *            @arg @ref SPI_IT_RXF Receive FIFO Full Interrupt enable
  *            @arg @ref SPI_IT_RXO Receive FIFO Overflow Interrupt enable
  *            @arg @ref SPI_IT_RXU Receive FIFO Underflow Interrupt enable
  *            @arg @ref SPI_IT_TXO Transmit FIFO Overflow Interrupt enable
  *            @arg @ref SPI_IT_TXE Transmit FIFO Empty Interrupt enable
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __HAL_SPI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)     (READ_BITS((__HANDLE__)->p_instance->INT_STAT, (__INTERRUPT__)) == (__INTERRUPT__))


/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __HANDLE__  Specifies the SPI Handle.
  * @param  __FLAG__    Specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref SPI_FLAG_DCOL Data collision error flag
  *            @arg @ref SPI_FLAG_TXE  Transmission error flag
  *            @arg @ref SPI_FLAG_RFF  Rx FIFO full flag
  *            @arg @ref SPI_FLAG_RFNE Rx FIFO not empty flag
  *            @arg @ref SPI_FLAG_TFE  Tx FIFO empty flag
  *            @arg @ref SPI_FLAG_TFNF Tx FIFO not full flag
  *            @arg @ref SPI_FLAG_BUSY Busy flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_SPI_GET_FLAG(__HANDLE__, __FLAG__)               ((READ_BITS((__HANDLE__)->p_instance->STAT, (__FLAG__)) != 0) ? SET : RESET)


/** @brief  Clear the specified SPI flag.
  * @param  __HANDLE__  Specifies the SPI Handle.
  * @param  __FLAG__    Specifies the flag to clear.
  *         This parameter can be one of the following values:
  *            @arg @ref SPI_FLAG_DCOL Data collision error flag
  *            @arg @ref SPI_FLAG_TXE  Transmission error flag
  *            @arg @ref SPI_FLAG_RFF  Rx FIFO full flag
  *            @arg @ref SPI_FLAG_RFNE Rx FIFO not empty flag
  *            @arg @ref SPI_FLAG_TFE  Tx FIFO empty flag
  *            @arg @ref SPI_FLAG_TFNF Tx FIFO not full flag
  *            @arg @ref SPI_FLAG_BUSY Busy flag
  * @retval None
  */
#define __HAL_SPI_CLEAR_FLAG(__HANDLE__, __FLAG__)             READ_BITS((__HANDLE__)->p_instance->STAT, (__FLAG__))


/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup SPI_Private_Macro SPI Private Macros
  * @{
  */

/** @brief  Check if SPI Direction Mode is valid.
  * @param  __MODE__    SPI Direction Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_SPI_DIRECTION(__MODE__)              (((__MODE__) == SPI_DIRECTION_FULL_DUPLEX) || \
                                                 ((__MODE__) == SPI_DIRECTION_SIMPLEX_TX)  || \
                                                 ((__MODE__) == SPI_DIRECTION_SIMPLEX_RX)  || \
                                                 ((__MODE__) == SPI_DIRECTION_READ_EEPROM))


/** @brief  Check if SPI Data Size is valid.
  * @param  __DATASIZE__    SPI Data Size.
  * @retval SET (__DATASIZE__ is valid) or RESET (__DATASIZE__ is invalid)
  */
#define IS_SPI_DATASIZE(__DATASIZE__)           (((__DATASIZE__) >= SPI_DATASIZE_4BIT) && \
                                                 ((__DATASIZE__) <= SPI_DATASIZE_32BIT))

/** @brief  Check if SPI Clock Polarity is valid.
  * @param  __CPOL__    SPI Clock Polarity.
  * @retval SET (__CPOL__ is valid) or RESET (__CPOL__ is invalid)
  */
#define IS_SPI_CPOL(__CPOL__)                   (((__CPOL__) == SPI_POLARITY_LOW) || \
                                                 ((__CPOL__) == SPI_POLARITY_HIGH))

/** @brief  Check if SPI Clock Phase is valid.
  * @param  __CPHA__    SPI Clock Phase.
  * @retval SET (__CPHA__ is valid) or RESET (__CPHA__ is invalid)
  */
#define IS_SPI_CPHA(__CPHA__)                   (((__CPHA__) == SPI_PHASE_1EDGE) || \
                                                 ((__CPHA__) == SPI_PHASE_2EDGE))

/** @brief  Check if SPI BaudRate Prescaler is valid.
  * @param  __PRESCALER__   SPI BaudRate Prescaler.
  * @retval SET (__PRESCALER__ is valid) or RESET (__PRESCALER__ is invalid)
  */
#define IS_SPI_BAUDRATE_PRESCALER(__PRESCALER__)    ((__PRESCALER__) <= 0xFFFF)

/** @brief  Check if SPI TI Mode is valid.
  * @param  __MODE__ SPI TI Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_SPI_TIMODE(__MODE__)                 (((__MODE__) == SPI_TIMODE_DISABLE) || \
                                                 ((__MODE__) == SPI_TIMODE_ENABLE))

/** @brief  Check if SPI Slave Select is valid.
  * @param  __SLAVE__ SPI Slave Select.
  * @retval SET (__SLAVE__ is valid) or RESET (__SLAVE__ is invalid)
  */
#define IS_SPI_SLAVE(__SLAVE__)                 (((__SLAVE__) == SPI_SLAVE_SELECT_0) || \
                                                 ((__SLAVE__) == SPI_SLAVE_SELECT_1) || \
                                                 ((__SLAVE__) == SPI_SLAVE_SELECT_ALL))

/** @brief  Check if SPI RX Sample Delay Value is valid.
  * @param  __DLY__ SPI RX Sample Delay value.
  * @retval SET (__DLY__ is valid) or RESET (__DLY__ is invalid)
  */
#define IS_SPI_RX_SAMPLE_DLY(__DLY__)          (((__DLY__) >= 0) && ((__DLY__) <= 7))


/** @brief  Check if SPI FIFO Threshold is valid.
  * @param  __THR__ SPI FIFO Threshold.
  * @retval SET (__THR__ is valid) or RESET (__THR__ is invalid)
  */

#define IS_SPI_FIFO_THRESHOLD(__THR__)          (((__THR__) >= 0) && ((__THR__) <= (LL_SPI_M_FIFO_DEPTH - 1)))

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_SPI_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup SPI_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the SPIx peripheral:

      (+) User must implement hal_spi_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_spi_init() to configure the selected device with
          the selected configuration:
        (++) Direction
        (++) Data Size
        (++) Clock Polarity and Phase
        (++) BaudRate Prescaler
        (++) TIMode
        (++) Slave Select

      (+) Call the function hal_spi_deinit() to restore the default configuration
          of the selected SPIx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the SPI according to the specified parameters
 *         in the spi_init_t and initialize the associated handle.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_init(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  De-initialize the SPI peripheral.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_deinit(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Initialize the SPI MSP.
 * @note   This function should not be modified. When the callback is needed,
           the hal_spi_msp_init can be implemented in the user file.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_msp_init(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  De-initialize the SPI MSP.
 * @note   This function should not be modified. When the callback is needed,
           the hal_spi_msp_deinit can be implemented in the user file.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_msp_deinit(spi_handle_t *p_spi);

/** @} */

/** @defgroup SPI_Exported_Functions_Group2 SPI operation functions
 *  @brief   Data transfer functions
 *
@verbatim
  ==============================================================================
                      ##### SPI operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the SPI
    data transfer.

    [..] The SPI supports master and slave mode:

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts
            or DMA, These APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated SPI IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The hal_spi_tx_cplt_callback(), hal_spi_rx_cplt_callback() and hal_spi_txrx_cplt_callback() user callbacks
            will be executed respectively at the end of the transmit or Receive process
            The hal_spi_error_callback() user callback will be executed when a communication error is detected.

    (#) APIs provided for these 2 transfer modes (Blocking mode or Non blocking mode using either Interrupt or DMA)
        exist for 1-Line (simplex) and 2-Line (full duplex) modes.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in blocking mode.
 * @param[in]  p_spi:   Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  p_data:  Pointer to data buffer
 * @param[in]  length:  Amount of data to be sent in bytes
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_transmit(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in blocking mode.
 * @param[in]  p_spi:   Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[out] p_data:  Pointer to data buffer
 * @param[in]  length:  Amount of data to be received in bytes
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_receive(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit and Receive an amount of data in blocking mode.
 * @param[in]  p_spi:     Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  p_tx_data: Pointer to transmission data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  length:    Amount of data to be sent and received in bytes
 * @param[in]  timeout:   Timeout duration
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_transmit_receive(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Read an amount of data from EEPROM in blocking mode.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  p_tx_data: Pointer to transmission data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  tx_number_data: Amount of data to be sent in bytes
 * @param[in]  rx_number_data: Amount of data to be received in bytes
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_read_eeprom(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t tx_number_data, uint32_t rx_number_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_transmit_it(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in non-blocking mode with Interrupt.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_receive_it(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  p_tx_data: Pointer to transmission data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  length: Amount of data to be sent and received in bytes
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_transmit_receive_it(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Read an amount of data from EEPROM in non-blocking mode with Interrupt.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  p_tx_data: Pointer to transmission data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  tx_number_data: Amount of data to be sent in bytes
 * @param[in]  rx_number_data: Amount of data to be received in bytes
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_read_eeprom_it(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t tx_number_data, uint32_t rx_number_data);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode with DMA.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 0 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_transmit_dma(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in non-blocking mode with DMA.
 * @note   In case of MASTER mode and SPI_DIRECTION_2LINES direction, p_dmatx shall be defined.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 0 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_receive_dma(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  p_tx_data: Pointer to transmission data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 0 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_transmit_receive_dma(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Read an amount of data from EEPROM in non-blocking mode with DMA.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  p_tx_data: Pointer to transmission data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  tx_number_data: Amount of data to be sent in bytes
 * @param[in]  rx_number_data: Amount of data to be received in bytes,  ranging between 0 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_read_eeprom_dma(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t tx_number_data, uint32_t rx_number_data);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode with DMA LLP.
 * @param[in] p_spi:        Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in] p_llp_config: Pointer to Linked List Block
 * @param[in] data_length:  Total data in all Blocks to be transmitted, unit in Byte
 * @retval ::HAL_OK:    Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY:  Driver is busy.
 ****************************************************************************************
 */
hal_status_t hal_spi_transmit_dma_llp(spi_handle_t *p_spi, dma_llp_config_t * p_llp_config, uint32_t data_length);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in non-blocking mode with DMA.
 * @note   In case of MASTER mode and SPI_DIRECTION_2LINES direction, p_dmatx shall be defined.
 * @param[in]  p_spi:          Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  p_tx_data:      Pointer to transmit data buffer
 * @param[in]  tx_data_length: Amount of data to be sent in bytes
 * @param[in]  p_rx_data:      Pointer to receive data buffer
 * @param[in]  rx_data_length: Amount of data to be received in bytes
 * @param[in]  sct_interval:   Scatter interval in beat
 * @param[in]  sct_count:      Scatter count in beat
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_read_eeprom_dma_scatter(spi_handle_t *p_spi, uint8_t * p_tx_data, uint32_t tx_data_length, uint8_t * p_rx_data, uint32_t rx_data_length, uint32_t sct_interval, uint32_t sct_count);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode with polling. Support Setting C&A
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  inst: 1 byte instruction
 * @param[in]  addr: 3 bytes address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 0 and 4095.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_transmit_with_ia(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode with DMA. Support Setting C&A
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  inst: 1 byte instruction
 * @param[in]  addr: 3 bytes address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 0 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_transmit_dma_with_ia(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length);
/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode with polling. Support Setting C&A
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  inst: 1 byte instruction
 * @param[in]  addr: 4 bytes address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 0 and 4095.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_transmit_with_ia_32addr(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode with DMA. Support Setting C&A
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  inst: 1 byte instruction
 * @param[in]  addr: 4 bytes address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 0 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_transmit_dma_with_ia_32addr(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length);
/**
 ****************************************************************************************
 * @brief  Abort ongoing transfer (blocking mode).
 * @param[in]  p_spi: SPI handle.
 * @note   This procedure could be used for aborting any ongoing transfer (Tx and Rx),
 *         started in Interrupt or DMA mode.
 *         This procedure performs following operations :
 *           - Disable SPI Interrupts (depending of transfer direction)
 *           - Disable the DMA transfer in the peripheral register (if enabled)
 *           - Abort DMA transfer by calling hal_dma_abort (in case of transfer in DMA mode)
 *           - Set handle State to READY
 * @note   This procedure is executed in blocking mode: when exiting function, Abort is considered as completed.
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_abort(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Abort ongoing transfer (Interrupt mode).
 * @param[in]  p_spi: SPI handle.
 * @note   This procedure could be used for aborting any ongoing transfer (Tx and Rx),
 *         started in Interrupt or DMA mode.
 *         This procedure performs following operations :
 *           - Disable SPI Interrupts (depending of transfer direction)
 *           - Disable the DMA transfer in the peripheral register (if enabled)
 *           - Abort DMA transfer by calling hal_dma_abort_it (in case of transfer in DMA mode)
 *           - Set handle State to READY
 *           - At abort completion, call user abort complete callback
 * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
 *         considered as completed only when user abort complete callback is executed (not when exiting function).
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_abort_it(spi_handle_t *p_spi);

/** @} */

/** @addtogroup SPI_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle SPI interrupt request.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_irq_handler(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Tx Transfer completed callback.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_tx_cplt_callback(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Rx Transfer completed callback.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_rx_cplt_callback(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Tx and Rx Transfer completed callback.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_tx_rx_cplt_callback(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  SPI error callback.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_error_callback(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  SPI Abort Completed callback.
 * @param[in]  p_spi: SPI handle.
 ****************************************************************************************
 */
void hal_spi_abort_cplt_callback(spi_handle_t *p_spi);

/** @} */

/** @defgroup SPI_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   SPI control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the SPI.
     (+) hal_spi_get_state() API can be helpful to check in run-time the state of the SPI peripheral
     (+) hal_spi_get_error() check in run-time Errors occurring during communication
     (+) hal_spi_set_timeout() set the timeout during internal process
     (+) hal_spi_set_tx_fifo_threshold() set the TX FIFO Threshold
     (+) hal_spi_set_rx_fifo_threshold() set the RX FIFO Threshold
     (+) hal_spi_get_tx_fifo_threshold() get the TX FIFO Threshold
     (+) hal_spi_get_rx_fifo_threshold() get the RX FIFO Threshold
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the SPI handle state.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @retval ::HAL_SPI_STATE_RESET:      Peripheral not initialized.
 * @retval ::HAL_SPI_STATE_READY:      Peripheral initialized and ready for use.
 * @retval ::HAL_SPI_STATE_BUSY:       An internal process is ongoing.
 * @retval ::HAL_SPI_STATE_BUSY_TX:    Data Transmission process is ongoing.
 * @retval ::HAL_SPI_STATE_BUSY_RX:    Data Reception process is ongoing.
 * @retval ::HAL_SPI_STATE_BUSY_TX_RX: Data Transmission and Reception process is ongoing.
 * @retval ::HAL_SPI_STATE_ABORT:      Peripheral with abort request ongoing.
 * @retval ::HAL_SPI_STATE_ERROR:      Peripheral in error.
 ****************************************************************************************
 */
hal_spi_state_t hal_spi_get_state(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Return the SPI error code.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @return SPI error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_spi_get_error(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Set the SPI internal process timeout value.
 * @param[in]  p_spi:   Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  timeout: Internal process timeout value.
 ****************************************************************************************
 */
void hal_spi_set_timeout(spi_handle_t *p_spi, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Set the TX FIFO threshold.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  threshold: TX FIFO threshold value ranging bwtween 0x0U ~ 0x7U.
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_set_tx_fifo_threshold(spi_handle_t *p_spi, uint32_t threshold);

/**
 ****************************************************************************************
 * @brief  Set the RX FIFO threshold.
 * @param[in]  p_spi:     Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  threshold: RX FIFO threshold value ranging bwtween 0x0U ~ 0x7U.
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_set_rx_fifo_threshold(spi_handle_t *p_spi, uint32_t threshold);

/**
 ****************************************************************************************
 * @brief  Get the TX FIFO threshold.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @return TX FIFO threshold
 ****************************************************************************************
 */
uint32_t hal_spi_get_tx_fifo_threshold(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Get the RX FIFO threshold.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @return RX FIFO threshold
 ****************************************************************************************
 */
uint32_t hal_spi_get_rx_fifo_threshold(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to SPI configuration before sleep.
 * @param[in] p_spi: Pointer to a SPI handle which contains the configuration
 *                   information for the specified SPI module.
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_suspend_reg(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to SPI configuration after sleep.
 *         This function must be used in conjunction with the hal_spi_suspend_reg().
 * @param[in] p_spi: Pointer to a SPI handle which contains the configuration
 *                   information for the specified SPI module.
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_resume_reg(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Assert the CS Singal line by software (When activate the soft-cs mode)
 * @param[in] p_spi: Pointer to a SPI handle which contains the configuration
 *                   information for the specified SPI module.
 * @param[in] state: assert state, please @ref CS_STA_STATE.
 * @retval :: none
 ****************************************************************************************
 */
void hal_spi_soft_cs_assert(spi_handle_t *p_spi, uint32_t state);

/**
 ****************************************************************************************
 * @brief  De-Assert the CS Singal line by software (When activate the soft-cs mode)
 * @param[in] p_spi: Pointer to a SPI handle which contains the configuration
 *                   information for the specified SPI module.
 * @param[in] state: assert state, please @ref CS_STA_STATE.
 * @retval :: none
 ****************************************************************************************
 */
void hal_spi_soft_cs_deassert(spi_handle_t *p_spi, uint32_t state);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_SPI_H__ */

/** @} */

/** @} */

/** @} */

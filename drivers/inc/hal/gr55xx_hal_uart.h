/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_uart.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of UART HAL library.
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

/** @defgroup HAL_UART UART
  * @brief UART HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_UART_H__
#define __GR55xx_HAL_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_uart.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_UART_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_UART_state HAL UART state
  * @{
  */

/**
  * @brief HAL UART State enumerations definition
  * @note  HAL UART State value is a combination of 2 different substates: gState and RxState.
  */
typedef enum
{
    HAL_UART_STATE_RESET     = 0x00U,  /**< Peripheral is not initialized.
                                            Value is allowed for gState and RxState */

    HAL_UART_STATE_READY     = 0x10U,  /**< Peripheral initialized and ready for use.
                                            Value is allowed for gState and RxState */

    HAL_UART_STATE_BUSY      = 0x14U,  /**< An internal process is ongoing.
                                            Value is allowed for gState only */

    HAL_UART_STATE_BUSY_TX   = 0x11U,  /**< Data Transmission process is ongoing.
                                            Value is allowed for gState only */

    HAL_UART_STATE_BUSY_RX   = 0x12U,  /**< Data Reception process is ongoing.
                                            Value is allowed for RxState only */

    HAL_UART_STATE_BUSY_TXRX = 0x13U,  /**< Data Transmission and Reception process is ongoing.
                                            Value is allowed for gState only */

    HAL_UART_STATE_TIMEOUT   = 0x30U,  /**< Timeout state.
                                            Value is allowed for gState only */

    HAL_UART_STATE_ERROR     = 0x70U   /**< Error.
                                            Value is allowed for gState only */

} hal_uart_state_t;

/** @} */

/** @} */

/** @addtogroup HAL_UART_STRUCTURES Structures
  * @{
  */

/** @defgroup UART_Configuration UART Configuration
  * @{
  */

/**
  * @brief UART init structure definition
  */
typedef struct _uart_init
{
    uint32_t baud_rate;         /**< This member configures the UART communication baud rate. */

    uint32_t data_bits;         /**< Specifies the number of data bits transmitted or received in a frame.
                                     This parameter can be a value of @ref UART_Data_Bits. */

    uint32_t stop_bits;         /**< Specifies the number of stop bits transmitted.
                                     This parameter can be a value of @ref UART_Stop_Bits. */

    uint32_t parity;            /**< Specifies the parity mode.
                                     This parameter can be a value of @ref UART_Parity. */

    uint32_t hw_flow_ctrl;      /**< Specifies whether the hardware flow control mode is enabled or disabled.
                                     This parameter can be a value of @ref UART_Hardware_Flow_Control. */

    uint32_t rx_timeout_mode;   /**< Specifies whether the receive timeout mode is enabled or disabled.
                                     When rx_timeout_mode is enabled, character timeout interrupt will disable
                                     current receive process after the data in RxFIFO is received, and call
                                     hal_uart_rx_cplt_callback(). Note that the rx_timeout_mode only works
                                     in interrupt mode.
                                     This parameter can be a value of @ref UART_Receiver_TimeOut. */

} uart_init_t;
/** @} */

/** @defgroup UART_handle UART handle
  * @{
  */

/**
  * @brief UART handle Structure definition
  */
typedef struct _uart_handle
{
    uart_regs_t           *p_instance;        /**< UART registers base address        */

    uart_init_t           init;             /**< UART communication parameters      */

    uint8_t               *p_tx_buffer;       /**< Pointer to UART Tx transfer Buffer */

    uint16_t              tx_xfer_size;     /**< UART Tx Transfer size              */

    __IO uint16_t         tx_xfer_count;    /**< UART Tx Transfer Counter           */

    uint8_t               *p_rx_buffer;       /**< Pointer to UART Rx transfer Buffer */

    uint16_t              rx_xfer_size;     /**< UART Rx Transfer size              */

    __IO uint16_t         rx_xfer_count;    /**< UART Rx Transfer Counter           */

    dma_handle_t          *p_dmatx;          /**< UART Tx DMA Handle parameters      */

    dma_handle_t          *p_dmarx;          /**< UART Rx DMA Handle parameters      */

    functional_state_t    dma_tx_mode;      /**< UART Tx DMA mode state             */

    functional_state_t    dma_rx_mode;      /**< UART Rx DMA mode state             */

    hal_lock_t            lock;             /**< Locking object                     */

    __IO hal_uart_state_t tx_state;         /**< UART state information related to Tx operations.
                                                 This parameter can be a value of @ref hal_uart_state_t */

    __IO hal_uart_state_t rx_state;         /**< UART state information related to Rx operations.
                                                 This parameter can be a value of @ref hal_uart_state_t */

    __IO uint32_t         error_code;       /**< UART Error code                    */

    uint32_t              retention[8];     /**< UART important register information. */
} uart_handle_t;
/** @} */

/** @} */

/** @addtogroup HAL_UART_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_UART_Callback Callback
  * @{
  */

/**
  * @brief HAL_UART Callback function definition
  */

typedef struct _uart_callback
{
    void (*uart_msp_init)(uart_handle_t *p_uart);               /**< UART init MSP callback                     */
    void (*uart_msp_deinit)(uart_handle_t *p_uart);             /**< UART de-init MSP callback                  */
    void (*uart_tx_cplt_callback)(uart_handle_t *p_uart);       /**< UART tx transfer completed callback        */
    void (*uart_rx_cplt_callback)(uart_handle_t *p_uart);       /**< UART rx transfer completed callback        */
    void (*uart_error_callback)(uart_handle_t *p_uart);         /**< UART error callback                        */
    void (*uart_abort_cplt_callback)(uart_handle_t *p_uart);    /**< UART abort completed callback              */
    void (*uart_abort_tx_cplt_callback)(uart_handle_t *p_uart); /**< UART abort tansmit complete callback       */
    void (*uart_abort_rx_cplt_callback)(uart_handle_t *p_uart); /**< UART abort receive complete callback       */
} uart_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_UART_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup UART_Exported_Constants UART Exported Constants
  * @{
  */

/** @defgroup UART_Fifo_Size UART Fifo Size
  * @{
  */
#define UART_TXFIFO_SIZE                    128             /**< UART tx fifo size   */
#define UART_RXFIFO_SIZE                    128             /**< UART rx fifo size   */
/** @} */

/** @defgroup UART_Error_Code UART Error Code
  * @{
  */
#define HAL_UART_ERROR_NONE                 (0x00000000U)   /**< No error            */
#define HAL_UART_ERROR_PE                   LL_UART_LSR_PE  /**< Parity error        */
#define HAL_UART_ERROR_FE                   LL_UART_LSR_FE  /**< frame error         */
#define HAL_UART_ERROR_OE                   LL_UART_LSR_OE  /**< Overrun error       */
#define HAL_UART_ERROR_BI                   LL_UART_LSR_BI  /**< Break dection error */
#define HAL_UART_ERROR_DMA                  (0x00000100U)   /**< DMA transfer error  */
#define HAL_UART_ERROR_BUSY                 (0x00000200U)   /**< Busy Error          */
#define HAL_UART_ERROR_INVALID_PARAM        (0x00000400U)   /**< Invalid parameter error */
/** @} */

/** @defgroup UART_Data_Bits UART Number of Data Bits
  * @{
  */
#define UART_DATABITS_5                     LL_UART_DATABITS_5B   /**< UART frame with 5 data bits */
#define UART_DATABITS_6                     LL_UART_DATABITS_6B   /**< UART frame with 6 data bits */
#define UART_DATABITS_7                     LL_UART_DATABITS_7B   /**< UART frame with 7 data bits */
#define UART_DATABITS_8                     LL_UART_DATABITS_8B   /**< UART frame with 8 data bits */
/** @} */

/** @defgroup UART_Stop_Bits UART Number of Stop Bits
  * @{
  */
#define UART_STOPBITS_1                     LL_UART_STOPBITS_1    /**< UART frame with 1 stop bit    */
#define UART_STOPBITS_1_5                   LL_UART_STOPBITS_1_5  /**< UART frame with 1.5 stop bits */
#define UART_STOPBITS_2                     LL_UART_STOPBITS_2    /**< UART frame with 2 stop bits   */
/** @} */

/** @defgroup UART_Parity UART Parity
  * @{
  */
#define UART_PARITY_NONE                    LL_UART_PARITY_NONE   /**< No parity   */
#define UART_PARITY_ODD                     LL_UART_PARITY_ODD    /**< Odd parity */
#define UART_PARITY_EVEN                    LL_UART_PARITY_EVEN   /**< Even parity  */
#define UART_PARITY_SP0                     LL_UART_PARITY_SP0    /**< Stick Parity 0  */
#define UART_PARITY_SP1                     LL_UART_PARITY_SP1    /**< Stick Parity 1  */
/** @} */

/** @defgroup UART_Hardware_Flow_Control UART Hardware Flow Control
  * @{
  */
#define UART_HWCONTROL_NONE                 LL_UART_HWCONTROL_NONE      /**< No hardware control       */
#define UART_HWCONTROL_RTS_CTS              LL_UART_HWCONTROL_RTS_CTS   /**< Auto RTS and CTS hardware flow control       */
/** @} */

/** @defgroup UART_Receiver_TimeOut UART Receiver TimeOut
  * @{
  */
#define UART_RECEIVER_TIMEOUT_DISABLE       (0x00000000U)       /**< UART receiver timeout disable */
#define UART_RECEIVER_TIMEOUT_ENABLE        (0x00000001U)       /**< UART receiver timeout enable  */
/** @} */

/** @defgroup UART_Interrupt_definition UART Interrupt_definition
  * @{
  */
#define UART_IT_MS                          LL_UART_IER_MS      /**< Enable Modem Status Interrupt */
#define UART_IT_RLS                         LL_UART_IER_RLS     /**< Enable Receiver Line Status Interrupt */
#define UART_IT_THRE                        LL_UART_IER_THRE    /**< Enable Transmit Holding Register Empty Interrupt */
#define UART_IT_RDA                         LL_UART_IER_RDA     /**< Enable Received Data Available Interrupt and Character Timeout Interrupt */
/** @} */

/** @defgroup UART_Request_Parameters UART Request Parameters
  * @{
  */
#define UART_RXDATA_FLUSH_REQUEST           UART_SRR_RFR        /**< RX FIFO flush Request  */
#define UART_TXDATA_FLUSH_REQUEST           UART_SRR_XFR        /**< TX FIFO flush Request */
#define UART_TXRXDATA_FLUSH_REQUEST         (UART_SRR_XFR | UART_SRR_RFR)   /**< TX FIFO and RX FIFO flush Request  */
/** @} */

/** @defgroup UART_Interrupt_Mask UART Interrupt Flag Mask
  * @{
  */
#define UART_IT_MASK                        (0x008FU)           /**< UART interruptions flags mask */
/** @} */

/** @defgroup UART_Line_Error_Mask UART Line Error Flag Mask
  * @{
  */
#define UART_LINE_ERROR_MASK                (LL_UART_LSR_PE | LL_UART_LSR_OE | LL_UART_LSR_FE | LL_UART_LSR_BI) /**< UART interruptions flags mask */
/** @} */

/** @defgroup UART_Retention_Length UART Retention Register Length
  * @{
  */
#define UART_RETENTION_LENGTH               ((uint32_t)8)         /**< the number of retention registers */
/** @} */

/** @defgroup UART_Timeout_definition UART Timeout_definition
  * @{
  */
#define HAL_UART_TIMEOUT_DEFAULT_VALUE      ((uint32_t)5000)         /**< 5s */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup UART_Exported_Macros UART Exported Macros
  * @{
  */

/** @brief  Reset UART handle states.
  * @param  __HANDLE__ UART handle.
  * @retval None
  */
#define __HAL_UART_RESET_HANDLE_STATE(__HANDLE__)       \
    do{                                                 \
        (__HANDLE__)->g_state = HAL_UART_STATE_RESET;   \
        (__HANDLE__)->rx_state = HAL_UART_STATE_RESET;  \
    } while(0U)

/** @brief  Enable the specified UART interrupt.
  * @param  __HANDLE__    Specifies the UART Handle.
  * @param  __INTERRUPT__ Specifies the UART interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg @ref UART_IT_RDA
  *            @arg @ref UART_IT_THRE
  *            @arg @ref UART_IT_RLS
  *            @arg @ref UART_IT_MS
  * @retval None
  */
#define __HAL_UART_ENABLE_IT(__HANDLE__, __INTERRUPT__)                 \
    do {                                                                \
        GLOBAL_EXCEPTION_DISABLE();                                     \
        ll_uart_enable_it((__HANDLE__)->p_instance, (__INTERRUPT__));   \
        GLOBAL_EXCEPTION_ENABLE();                                      \
    } while(0U)

/** @brief  Disable the specified UART interrupt.
  * @param  __HANDLE__    Specifies the UART Handle.
  * @param  __INTERRUPT__ Specifies the UART interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg @ref UART_IT_RDA
  *            @arg @ref UART_IT_THRE
  *            @arg @ref UART_IT_RLS
  *            @arg @ref UART_IT_MS
  * @retval None
  */
#define __HAL_UART_DISABLE_IT(__HANDLE__, __INTERRUPT__)                \
    do {                                                                \
        GLOBAL_EXCEPTION_DISABLE();                                     \
        ll_uart_disable_it((__HANDLE__)->p_instance, (__INTERRUPT__));  \
        GLOBAL_EXCEPTION_ENABLE();                                      \
    } while(0)

/** @brief  Flush the UART FIFO and treat FIFO as empty.
  * @param  __HANDLE__ Specifies the UART Handle.
  * @param  __REQ__    Specifies the request flag to set
  *          This parameter can be one of the following values:
  *            @arg @ref UART_RXDATA_FLUSH_REQUEST RX FIFO flush Request
  *            @arg @ref UART_TXDATA_FLUSH_REQUEST TX FIFO flush Request
  *            @arg @ref UART_TXRXDATA_FLUSH_REQUEST TX FIFO and RX FIFO flush
  * @retval None
  */
#define __HAL_UART_SEND_REQ(__HANDLE__, __REQ__) ((__HANDLE__)->p_instance->SRR = (__REQ__))

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup UART_Private_Macro UART Private Macros
  * @{
  */

/** @brief  Check if UART Baudrate is valid.
  * @param  __BAUDRATE__ UART Baudrate.
  * @retval SET (__BAUDRATE__ is valid) or RESET (__BAUDRATE__ is invalid)
  */
#define IS_UART_BAUDRATE(__BAUDRATE__) ((__BAUDRATE__) < 921600U)

/**
  * @brief Check if UART frame number of stop bits is valid.
  * @param __STOPBITS__ UART frame number of stop bits.
  * @retval SET (__STOPBITS__ is valid) or RESET (__STOPBITS__ is invalid)
  */
#define IS_UART_STOPBITS(__STOPBITS__) (((__STOPBITS__) == UART_STOPBITS_1)   || \
                                        ((__STOPBITS__) == UART_STOPBITS_1_5) || \
                                        ((__STOPBITS__) == UART_STOPBITS_2))

/**
  * @brief Check if UART frame number of data bits is valid.
  * @param __DATABITS__ UART frame number of data bits.
  * @retval SET (__DATABITS__ is valid) or RESET (__DATABITS__ is invalid)
  */
#define IS_UART_DATABITS(__DATABITS__) (((__DATABITS__) == UART_DATABITS_5) || \
                                        ((__DATABITS__) == UART_DATABITS_6) || \
                                        ((__DATABITS__) == UART_DATABITS_7) || \
                                        ((__DATABITS__) == UART_DATABITS_8))

/**
  * @brief Check if UART frame parity is valid.
  * @param __PARITY__ UART frame parity.
  * @retval SET (__PARITY__ is valid) or RESET (__PARITY__ is invalid)
  */
#define IS_UART_PARITY(__PARITY__) (((__PARITY__) == UART_PARITY_NONE) || \
                                    ((__PARITY__) == UART_PARITY_EVEN) || \
                                    ((__PARITY__) == UART_PARITY_ODD)  || \
                                    ((__PARITY__) == UART_PARITY_SP0)  || \
                                    ((__PARITY__) == UART_PARITY_SP1))

/**
  * @brief Check if UART hardware flow control is valid.
  * @param __CONTROL__ UART hardware flow control.
  * @retval SET (__CONTROL__ is valid) or RESET (__CONTROL__ is invalid)
  */
#define IS_UART_HARDWARE_FLOW_CONTROL(__CONTROL__)\
                                   (((__CONTROL__) == UART_HWCONTROL_NONE) || \
                                    ((__CONTROL__) == UART_HWCONTROL_RTS_CTS)
/** @} */

/**
  * @brief Default configuartion for initializing structure
  */
#define UART_DEFAULT_CONFIG                             \
{                                                       \
    .baud_rate       = 9600,                            \
    .data_bits       = UART_DATABITS_8,                 \
    .stop_bits       = UART_STOPBITS_1,                 \
    .parity          = UART_PARITY_NONE,                \
    .hw_flow_ctrl    = UART_HWCONTROL_NONE,             \
    .rx_timeout_mode = UART_RECEIVER_TIMEOUT_DISABLE,   \
}

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_UART_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup UART_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and de-initialization functions
  *
  * @verbatim
===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to initialize the UARTx.
      (+) For the asynchronous mode the parameters below can be configured:
        (++) Baud Rate
        (++) Data Bit
        (++) Stop Bit
        (++) Parity
        (++) Hardware flow control
    [..]
    The hal_uart_init() API follow the UART asynchronous configuration procedures.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief Initialize the UART according to the specified
 *        parameters in the uart_init_t and initialize the associated handle.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_init(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief De-initialize the UART peripheral.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_deinit (uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief Initialize the UART MSP.
 * @note  This function should not be modified. When the callback is needed,
           the hal_uart_msp_init can be implemented in the user file.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 ****************************************************************************************
 */
void hal_uart_msp_init(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief De-initialize the UART MSP.
 * @note  This function should not be modified. When the callback is needed,
           the hal_uart_msp_deinit can be implemented in the user file.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 ****************************************************************************************
 */
void hal_uart_msp_deinit(uart_handle_t *p_uart);

/** @} */

/** @addtogroup UART_Exported_Functions_Group2 IO operation functions
  * @brief UART Transmit/Receive functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    This subsection provides a set of functions allowing to manage the UART asynchronous
    and Half duplex data transfers.

    (#) There are two mode of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
           The HAL status of all data processing is returned by the same function
           after finishing transfer.
       (++) Non-Blocking mode: The communication is performed using Interrupts
           or DMA, These API's return the HAL status.
           The end of the data processing will be indicated through the
           dedicated UART IRQ when using Interrupt mode or the DMA IRQ when
           using DMA mode.
           The hal_uart_tx_cplt_callback(), hal_uart_rx_cplt_callback() user callbacks
           will be executed respectively at the end of the transmit or Receive process
           The hal_uart_error_callback() user callback will be executed when a
           communication error is detected

    (#) Blocking mode API's are :
        (++) hal_uart_transmit()
        (++) hal_uart_receive()

    (#) Non-Blocking mode API's with Interrupt are :
        (++) hal_uart_transmit_it()
        (++) hal_uart_receive_it()
        (++) hal_uart_irq_handler()

    (#) Non-Blocking mode API's with DMA are :
        (++) hal_uart_transmit_dma()
        (++) hal_uart_receive_dma()
        (++) hal_uart_dma_pause()
        (++) hal_uart_dma_resume()
        (++) hal_uart_dma_stop()

    (#) A set of Transfer Complete Callbacks are provided in Non_Blocking mode:
        (++) hal_uart_tx_cplt_callback()
        (++) hal_uart_rx_cplt_callback()
        (++) hal_uart_error_callback()

    (#) Non-Blocking mode transfers could be aborted using Abort API's :
        (++) hal_uart_abort()
        (++) hal_uart_abort_transmit()
        (++) hal_uart_abort_receive()
        (++) hal_uart_abort_it()
        (++) hal_uart_abort_transmit_it()
        (++) hal_uart_abort_receive_it()

    (#) For Abort services based on interrupts (hal_uart_abort_xxx_it), a set
        of Abort Complete Callbacks are provided:
        (++) hal_uart_abort_cplt_callback()
        (++) hal_uart_abort_tx_cplt_callback()
        (++) hal_uart_abort_rx_cplt_callback()

    (#) In Non-Blocking mode transfers, possible errors are split into 2 categories.
        Errors are handled as follows :
        (++) Error is considered as Recoverable and non blocking. Transfer could go till end, but error severity is
             to be evaluated by user : this concerns Frame Error, Parity Error or Noise Error in Interrupt mode reception .
             Received character is then retrieved and stored in Rx buffer, Error code is set to allow user to identify error type,
             and hal_uart_error_callback() user callback is executed. Transfer is kept ongoing on UART side.
             If user wants to abort it, Abort services should be called by user.
        (++) Error is considered as Blocking : Transfer could not be completed properly and is aborted.
             This concerns Overrun Error In Interrupt mode reception and all errors in DMA mode.
             Error code is set to allow user to identify error type, and hal_uart_error_callback() user callback is executed.

    -@- In the Half duplex communication, it is forbidden to run the transmit
        and receive process in parallel, the UART state hal_uart_state_busy_tx_rx can't be useful.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief Send an amount of data in blocking mode.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @param[in] p_data: Pointer to data buffer.
 * @param[in] size: Amount of data to be sent.
 * @param[in] timeout: Timeout duration.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_transmit(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief Receive an amount of data in blocking mode.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @param[out] p_data: Pointer to data buffer.
 * @param[in] size: Amount of data to be received.
 * @param[in] timeout: Timeout duration.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_receive(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief Send an amount of data in interrupt mode.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @param[in] p_data: Pointer to data buffer.
 * @param[in] size: Amount of data to be sent.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_transmit_it(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief Receive an amount of data in interrupt mode.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @param[out] p_data: Pointer to data buffer.
 * @param[in] size: Amount of data to be received.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_receive_it(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief Send an amount of data in DMA mode.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @param[in] p_data: Pointer to data buffer.
 * @param[in] size: Amount of data to be sent, ranging between 0 ~ 4095.
 * @note  This function starts a DMA transfer in interrupt mode meaning that
 *        DMA half transfer complete, DMA transfer complete and DMA transfer
 *        error interrupts are enabled
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_transmit_dma(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size);


/**
 ****************************************************************************************
 * @brief Send an amount of data in DMA mode.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @param[in] p_data: Pointer to data buffer.
 * @param[in] size: Amount of data to be sent, ranging between 0 ~ 4095.
 * @param[in] sg_llp_config: The config of source and destination's SG and LLP.
 * @note  This function starts a DMA transfer in interrupt mode meaning that
 *        DMA half transfer complete, DMA transfer complete and DMA transfer
 *        error interrupts are enabled
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_transmit_dma_sg_llp(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size, dma_sg_llp_config_t *sg_llp_config);


/**
 ****************************************************************************************
 * @brief Receive an amount of data in DMA mode.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @param[out] p_data: Pointer to data buffer.
 * @param[in] size: Amount of data to be received, ranging between 0 and 4095.
 * @note  When the UART parity is enabled (PCE = 1), the received data contain
 *        the parity bit (MSB position).
 * @note  This function starts a DMA transfer in interrupt mode meaning that
 *        DMA half transfer complete, DMA transfer complete and DMA transfer
 *        error interrupts are enabled
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_receive_dma(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size);


/**
 ****************************************************************************************
 * @brief Receive an amount of data in DMA mode.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @param[out] p_data: Pointer to data buffer.
 * @param[in] size: Amount of data to be received, ranging between 0 and 4095.
 * @param[in] sg_llp_config: The config of source and destination's SG and LLP.
 * @note  When the UART parity is enabled (PCE = 1), the received data contain
 *        the parity bit (MSB position).
 * @note  This function starts a DMA transfer in interrupt mode meaning that
 *        DMA half transfer complete, DMA transfer complete and DMA transfer
 *        error interrupts are enabled
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_receive_dma_sg_llp(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size, dma_sg_llp_config_t *sg_llp_config);


/**
 ****************************************************************************************
 * @brief  Pause the DMA Transfer.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_dma_pause(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Resume the DMA Transfer.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_dma_resume(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Stop the DMA Transfer.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_dma_stop(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Abort ongoing transfers (blocking mode).
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
 *         This procedure performs following operations :
 *           - Disable UART Interrupts (Tx and Rx)
 *           - Disable the DMA transfer in the peripheral register (if enabled)
 *           - Abort DMA transfer by calling hal_dma_abort (in case of transfer in DMA mode)
 *           - Set handle State to READY
 * @note   This procedure is executed in blocking mode: when exiting function, Abort is considered as completed.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_abort(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Abort ongoing Transmit transfer (blocking mode).
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @note   This procedure could be used for aborting any ongoing Tx transfer started in Interrupt or DMA mode.
 *         This procedure performs following operations :
 *           - Disable UART Interrupts (Tx)
 *           - Disable the DMA transfer in the peripheral register (if enabled)
 *           - Abort DMA transfer by calling hal_dma_abort (in case of transfer in DMA mode)
 *           - Set handle State to READY
 * @note   This procedure is executed in blocking mode: when exiting function, Abort is considered as completed.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_abort_transmit(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Abort ongoing Receive transfer (blocking mode).
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @note   This procedure could be used for aborting any ongoing Rx transfer started in Interrupt or DMA mode.
 *         This procedure performs following operations :
 *           - Disable UART Interrupts (Rx)
 *           - Disable the DMA transfer in the peripheral register (if enabled)
 *           - Abort DMA transfer by calling hal_dma_abort (in case of transfer in DMA mode)
 *           - Set handle State to READY
 * @note   This procedure is executed in blocking mode: when exiting function, Abort is considered as completed.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_abort_receive(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Abort ongoing transfers (Interrupt mode).
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
 *         This procedure performs following operations :
 *           - Disable UART Interrupts (Tx and Rx)
 *           - Disable the DMA transfer in the peripheral register (if enabled)
 *           - Abort DMA transfer by calling hal_dma_abort_it (in case of transfer in DMA mode)
 *           - Set handle State to READY
 *           - At abort completion, call user abort complete callback
 * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
 *         considered as completed only when user abort complete callback is executed (not when exiting function).
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_abort_it(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Abort ongoing Transmit transfer (Interrupt mode).
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @note   This procedure could be used for aborting any ongoing Tx transfer started in Interrupt or DMA mode.
 *         This procedure performs following operations :
 *           - Disable UART Interrupts (Tx)
 *           - Disable the DMA transfer in the peripheral register (if enabled)
 *           - Abort DMA transfer by calling hal_dma_abort_it (in case of transfer in DMA mode)
 *           - Set handle State to READY
 *           - At abort completion, call user abort complete callback
 * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
 *         considered as completed only when user abort complete callback is executed (not when exiting function).
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_abort_transmit_it(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Abort ongoing Receive transfer (Interrupt mode).
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @note   This procedure could be used for aborting any ongoing Rx transfer started in Interrupt or DMA mode.
 *         This procedure performs following operations :
 *           - Disable UART Interrupts (Rx)
 *           - Disable the DMA transfer in the peripheral register (if enabled)
 *           - Abort DMA transfer by calling hal_dma_abort_it (in case of transfer in DMA mode)
 *           - Set handle State to READY
 *           - At abort completion, call user abort complete callback
 * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
 *         considered as completed only when user abort complete callback is executed (not when exiting function).
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_abort_receive_it(uart_handle_t *p_uart);

/** @} */

/** @addtogroup UART_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief Handle UART interrupt request.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration information
 *                 for the specified UART module.
 ****************************************************************************************
 */
void hal_uart_irq_handler(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Tx Transfer completed callback.
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_uart_tx_cplt_callback can be implemented in the user file.
 * @param[in]  p_uart: Pointer to a UART handle which contains the configuration
 *                     information for the specified UART module.
 ****************************************************************************************
 */
void hal_uart_tx_cplt_callback(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Rx Transfer completed callback.
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_uart_rx_cplt_callback can be implemented in the user file.
 * @param[in]  p_uart: Pointer to a UART handle which contains the configuration
 *                  information for the specified UART module.
 ****************************************************************************************
 */
void hal_uart_rx_cplt_callback(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  UART error callback.
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_uart_error_callback can be implemented in the user file.
 * @param[in]  p_uart: Pointer to a UART handle which contains the configuration
 *                  information for the specified UART module.
 ****************************************************************************************
 */
void hal_uart_error_callback(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  UART Abort Complete callback.
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_uart_abort_cplt_callback can be implemented in the user file.
 * @param[in]  p_uart: Pointer to a UART handle which contains the configuration
 *                  information for the specified UART module.
 ****************************************************************************************
 */
void hal_uart_abort_cplt_callback (uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  UART Abort Tansmit Complete callback.
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_uart_abort_tx_cplt_callback can be implemented in the user file.
 * @param[in]  p_uart: Pointer to a UART handle which contains the configuration
 *                  information for the specified UART module.
 ****************************************************************************************
 */
void hal_uart_abort_tx_cplt_callback (uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  UART Abort Receive Complete callback.
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_uart_abort_rx_cplt_callback can be implemented in the user file.
 * @param[in]  p_uart: Pointer to a UART handle which contains the configuration
 *                  information for the specified UART module.
 ****************************************************************************************
 */
void hal_uart_abort_rx_cplt_callback (uart_handle_t *p_uart);

/** @} */


/** @addtogroup UART_Exported_Functions_Group3 Peripheral Control and State functions
 *  @brief   UART Peripheral State functions
 *
@verbatim
  ==============================================================================
            ##### Peripheral Control and State functions #####
  ==============================================================================
    [..]
    This subsection provides functions allowing to :
      (+) Return the UART handle state.
      (+) Return the UART handle error code

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the UART handle state.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @retval ::HAL_UART_STATE_RESET: Peripheral is not initialized.
 * @retval ::HAL_UART_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_UART_STATE_BUSY: An internal process is ongoing.
 * @retval ::HAL_UART_STATE_BUSY_TX: Data Transmission process is ongoing.
 * @retval ::HAL_UART_STATE_BUSY_RX: Data Reception process is ongoing.
 * @retval ::HAL_UART_STATE_TIMEOUT: Timeout state.
 * @retval ::HAL_UART_STATE_ERROR: Error.
 ****************************************************************************************
 */
hal_uart_state_t hal_uart_get_state(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Return the UART handle error code.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @return UART Error Code
 ****************************************************************************************
 */
uint32_t hal_uart_get_error(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to UART configuration before sleep.
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_suspend_reg(uart_handle_t *p_uart);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to UART configuration after sleep.
 *         This function must be used in conjunction with the hal_uart_suspend_reg().
 * @param[in] p_uart: Pointer to a UART handle which contains the configuration
 *                 information for the specified UART module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_uart_resume_reg(uart_handle_t *p_uart);


/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_UART_H__ */

/** @} */

/** @} */

/** @} */

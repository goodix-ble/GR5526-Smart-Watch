/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_uart.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of UART LL library.
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

/** @defgroup LL_UART UART
  * @brief UART LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_UART_H__
#define __GR55xx_LL_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (UART0) || defined (UART1) || \
    defined (UART2) || defined (UART3) || \
    defined (UART4) || defined (UART5)

/** @defgroup UART_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup UART_LL_ES_INIT UART Exported init structures
  * @{
  */

/**
  * @brief LL UART init Structure definition
  */
typedef struct _ll_uart_init_t
{
    uint32_t baud_rate;                 /**< This field defines expected Usart communication baud rate.

                                           This feature can be modified afterwards using unitary function @ref ll_uart_set_baud_rate().*/

    uint32_t data_bits;                 /**< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref UART_LL_EC_DATABITS.

                                           This feature can be modified afterwards using unitary function @ref ll_uart_set_data_bits_length().*/

    uint32_t stop_bits;                 /**< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref UART_LL_EC_STOPBITS.

                                           This feature can be modified afterwards using unitary function @ref ll_uart_set_stop_bits_length().*/

    uint32_t parity;                    /**< Specifies the parity mode.
                                           This parameter can be a value of @ref UART_LL_EC_PARITY.

                                           This feature can be modified afterwards using unitary function @ref ll_uart_set_parity().*/

    uint32_t hw_flow_ctrl;              /**< Specifies whether the hardware flow control mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_LL_EC_HWCONTROL.

                                           This feature can be modified afterwards using unitary function @ref ll_uart_set_hw_flow_ctrl().*/
} ll_uart_init_t;

/** @} */

/** @} */

/**
  * @defgroup  UART_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup UART_LL_Exported_Constants UART Exported Constants
  * @{
  */

/** @defgroup UART_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags definitions which can be used with LL_UART_ReadReg function
  * @{
  */
#define LL_UART_LSR_OE                      UART_LSR_OE             /**< Overrun error flag */
#define LL_UART_LSR_PE                      UART_LSR_PE             /**< Parity error flag */
#define LL_UART_LSR_FE                      UART_LSR_FE             /**< Framing error flag */
#define LL_UART_LSR_BI                      UART_LSR_BI             /**< Break detection flag */
#define LL_UART_LSR_THRE                    UART_LSR_THRE           /**< Transmit holding register empty flag */
#define LL_UART_LSR_TEMT                    UART_LSR_TEMT           /**< Transmitter empty flag */
#define LL_UART_LSR_RFE                     UART_LSR_RFE            /**< Rx FIFO error flag */

#define LL_UART_IIR_MS                      UART_IIR_IID_MS         /**< Modem Status flag */
#define LL_UART_IIR_NIP                     UART_IIR_IID_NIP        /**< No Interrupt Pending flag */
#define LL_UART_IIR_THRE                    UART_IIR_IID_THRE       /**< THR Empty flag */
#define LL_UART_IIR_RDA                     UART_IIR_IID_RDA        /**< Received Data Available flag */
#define LL_UART_IIR_RLS                     UART_IIR_IID_RLS        /**< Receiver Line Status flag */
#define LL_UART_IIR_CTO                     UART_IIR_IID_CTO        /**< Character Timeout flag */

#define LL_UART_USR_RFF                     UART_USR_RFF            /**< Rx FIFO Full flag */
#define LL_UART_USR_RFNE                    UART_USR_RFNE           /**< Rx FIFO Not Empty flag */
#define LL_UART_USR_TFE                     UART_USR_TFE            /**< Tx FIFO Empty flag */
#define LL_UART_USR_TFNF                    UART_USR_TFNF           /**< Tx FIFO Not Full flag */
/** @} */

/** @defgroup UART_LL_EC_IT IT Defines
  * @brief    Interrupt definitions which can be used with LL_UART_ReadReg and  LL_UART_WriteReg functions
  * @{
  */
#define LL_UART_IER_MS                      UART_IER_EDSSI                      /**< Enable Modem Status Interrupt */
#define LL_UART_IER_RLS                     UART_IER_ERLS                       /**< Enable Receiver Line Status Interrupt */
#define LL_UART_IER_THRE                    (UART_IER_ETBEI | UART_IER_PTIME)   /**< Enable Transmit Holding Register Empty Interrupt */
#define LL_UART_IER_RDA                     UART_IER_ERBFI                      /**< Enable Received Data Available Interrupt and Character Timeout Interrupt */
/** @} */

/** @defgroup UART_LL_EC_PARITY Parity Control
  * @{
  */
#define LL_UART_PARITY_NONE                 UART_LCR_PARITY_NONE    /**< Parity control disabled */
#define LL_UART_PARITY_ODD                  UART_LCR_PARITY_ODD     /**< Parity control enabled and Odd Parity is selected */
#define LL_UART_PARITY_EVEN                 UART_LCR_PARITY_EVEN    /**< Parity control enabled and Even Parity is selected */
#define LL_UART_PARITY_SP0                  UART_LCR_PARITY_SP0     /**< Parity control enabled and Stick Parity 0 is selected */
#define LL_UART_PARITY_SP1                  UART_LCR_PARITY_SP1     /**< Parity control enabled and Stick  Parity 1 is selected */
/** @} */

/** @defgroup UART_LL_EC_DATABITS Data Bits
  * @{
  */
#define LL_UART_DATABITS_5B                 UART_LCR_DLS_5          /**< 5 bits word length : Start bit, 5 data bits, n stop bits */
#define LL_UART_DATABITS_6B                 UART_LCR_DLS_6          /**< 6 bits word length : Start bit, 6 data bits, n stop bits */
#define LL_UART_DATABITS_7B                 UART_LCR_DLS_7          /**< 7 bits word length : Start bit, 7 data bits, n stop bits */
#define LL_UART_DATABITS_8B                 UART_LCR_DLS_8          /**< 8 bits word length : Start bit, 8 data bits, n stop bits */
/** @} */

/** @defgroup UART_LL_EC_STOPBITS Stop Bits
  * @{
  */
#define LL_UART_STOPBITS_1                  UART_LCR_STOP_1         /**< 1 stop bit */
#define LL_UART_STOPBITS_1_5                UART_LCR_STOP_1_5       /**< 1.5 stop bits */
#define LL_UART_STOPBITS_2                  UART_LCR_STOP_2         /**< 2 stop bits */
/** @} */

/** @defgroup UART_LL_EC_HWCONTROL Hardware Flow Control
  * @{
  */
#define LL_UART_HWCONTROL_NONE              0x00000000U                     /**< CTS and RTS hardware flow control disabled */
#define LL_UART_HWCONTROL_RTS_CTS           (UART_MCR_AFCE | UART_MCR_RTS)  /**< CTS and RTS hardware flow control enabled */
/** @} */

/** @defgroup UART_LL_EC_TX_FIFO_TH TX FIFO Threshold
  * @{
  */
#define LL_UART_TX_FIFO_TH_EMPTY            0x00000000U    /**< TX FIFO empty */
#define LL_UART_TX_FIFO_TH_CHAR_2           0x00000001U    /**< 2 characters in TX FIFO */
#define LL_UART_TX_FIFO_TH_QUARTER_FULL     0x00000002U    /**< TX FIFO 1/4 full */
#define LL_UART_TX_FIFO_TH_HALF_FULL        0x00000003U    /**< TX FIFO 1/2 full */
/** @} */

/** @defgroup UART_LL_EC_RX_FIFO_TH RX FIFO Threshold
  * @{
  */
#define LL_UART_RX_FIFO_TH_CHAR_1           0x00000000U    /**< 1 character in RX FIFO */
#define LL_UART_RX_FIFO_TH_QUARTER_FULL     0x00000001U    /**< RX FIFO 1/4 full */
#define LL_UART_RX_FIFO_TH_HALF_FULL        0x00000002U    /**< RX FIFO 1/2 full */
#define LL_UART_RX_FIFO_TH_FULL_2           0x00000003U    /**< RX FIFO 2 less than full */
/** @} */

/** @defgroup UART_LL_EC_RTSPIN_STATE RTS Pin State
  * @{
  */
#define LL_UART_RTSPIN_STATE_ACTIVE         0x00000001U    /**< RTS pin active(Logic 1) */
#define LL_UART_RTSPIN_STATE_INACTIVE       0x00000000U    /**< RTS pin inactive(Logic 0) */
/** @} */

/** @defgroup UART_LL_EC_CTSPIN_STATE CTS Pin State
  * @{
  */
#define LL_UART_CTSPIN_STATE_ACTIVE         0x00000001U    /**< CTS pin active(Logic 1) */
#define LL_UART_CTSPIN_STATE_INACTIVE       0x00000000U    /**< CTS pin pin inactive(Logic 0) */
/** @} */

/** @defgroup UART_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */

/**
  * @brief LL UART InitStrcut default configuartion
  */
#define LL_UART_DEFAULT_CONFIG                      \
{                                                   \
    .baud_rate            = 9600U,                   \
    .data_bits            = LL_UART_DATABITS_8B,     \
    .stop_bits            = LL_UART_STOPBITS_1,      \
    .parity               = LL_UART_PARITY_NONE,     \
    .hw_flow_ctrl = LL_UART_HWCONTROL_NONE,  \
}
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup UART_LL_Exported_Macros UART Exported Macros
  * @{
  */

/** @defgroup UART_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in UART register
  * @param  __instance__ UART instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_UART_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in UART register
  * @param  __instance__ UART instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_UART_ReadReg(__instance__, __REG__) READ_REG(__instance__->__REG__)

/** @} */

/** @defgroup UART_LL_EM_Exported_Macros_Helper Exported_Macros_Helper
  * @{
  */

/**
  * @brief  Compute UARTDIV value according to Peripheral Clock and
  *         expected Baud Rate (32 bits value of UARTDIV is returned)
  * @param  __PERIPHCLK__ Peripheral Clock frequency used for UART instance
  * @param  __BAUDRATE__ Baud rate value to achieve
  * @retval UARTDIV value to be used for DLL,DLH registers
  */
#define __LL_UART_DIV(__PERIPHCLK__, __BAUDRATE__) ((__PERIPHCLK__) / (__BAUDRATE__) / 16)

/**
  * @brief  Compute UARTDLF value according to Peripheral Clock and
  *         expected Baud Rate (32 bits value of UARTDLF is returned)
  * @param  __PERIPHCLK__ Peripheral Clock frequency used for UART instance
  * @param  __BAUDRATE__ Baud rate value to achieve
  * @retval UARTDLF value to be used for DLL,DLH registers
  */
#define __LL_UART_DLF(__PERIPHCLK__, __BAUDRATE__) ((__PERIPHCLK__) / (__BAUDRATE__) % 16)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup UART_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup UART_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Configure UART DLF and DLH register for achieving expected Baud Rate value.
  * @note   Peripheral clock and Baud rate values provided as function parameters should be valid
  *         (Baud rate value != 0)
  *
  *  Register|BitsName
  *  --------|--------
  *  DLL | DLL
  *  DLH | DLH
  *
  * @param  UARTx UART instance
  * @param  peripheral_clock Peripheral Clock
  * @param  baud_rate Baud Rate
  * @retval None
  */
__STATIC_INLINE void ll_uart_set_baud_rate(uart_regs_t *UARTx, uint32_t peripheral_clock, uint32_t baud_rate)
{
    register uint32_t uartdiv = __LL_UART_DIV(peripheral_clock, baud_rate);

    SET_BITS(UARTx->LCR, UART_LCR_DLAB);
    WRITE_REG(UARTx->RBR_DLL_THR.DLL, uartdiv & UART_DLL_DLL);
    WRITE_REG(UARTx->DLH_IER.DLH, (uartdiv >> 8) & UART_DLH_DLH);
    CLEAR_BITS(UARTx->LCR, UART_LCR_DLAB);
    WRITE_REG(UARTx->DLF, __LL_UART_DLF(peripheral_clock, baud_rate));
}

/**
  * @brief  Return current Baud Rate value
  * @note   In case of non-initialized or invalid value stored in DLL,DLH and DLF register, the value 0 will be returned..
  *
  *  Register|BitsName
  *  --------|--------
  *  DLL | DLL
  *  DLH | DLH
  *
  * @param  UARTx UART instance
  * @param  peripheral_clock Peripheral Clock
  * @retval Baud Rate
  */
__STATIC_INLINE uint32_t ll_uart_get_baud_rate(uart_regs_t *UARTx, uint32_t peripheral_clock)
{
    register uint32_t uartdiv = 0x0U;
    register uint32_t baud = 0x0U;

    SET_BITS(UARTx->LCR, UART_LCR_DLAB);
    uartdiv = UARTx->RBR_DLL_THR.DLL | (UARTx->DLH_IER.DLH << 8);
    CLEAR_BITS(UARTx->LCR, UART_LCR_DLAB);

    if ((uartdiv != 0) && (UARTx->DLF != 0x0U))
    {
        baud = peripheral_clock / (16 * uartdiv + UARTx->DLF);
    }

    return baud;
}

/**
  * @brief  Set the length of the data bits
  *
  *  Register|BitsName
  *  --------|--------
  *  LCR | DLS
  *
  * @param  UARTx UART instance
  * @param  data_bits This parameter can be one of the following values:
  *         @arg @ref LL_UART_DATABITS_5B
  *         @arg @ref LL_UART_DATABITS_6B
  *         @arg @ref LL_UART_DATABITS_7B
  *         @arg @ref LL_UART_DATABITS_8B
  *
  * @retval None
  */
__STATIC_INLINE void ll_uart_set_data_bits_length(uart_regs_t *UARTx, uint32_t data_bits)
{
    MODIFY_REG(UARTx->LCR, UART_LCR_DLS, data_bits);
}

/**
  * @brief  Return the length of the data bits
  *
  *  Register|BitsName
  *  --------|--------
  *  LCR | DLS
  *
  * @param  UARTx UART instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_UART_DATABITS_5B
  *         @arg @ref LL_UART_DATABITS_6B
  *         @arg @ref LL_UART_DATABITS_7B
  *         @arg @ref LL_UART_DATABITS_8B
  */
__STATIC_INLINE uint32_t ll_uart_get_data_bits_length(uart_regs_t *UARTx)
{
    return (uint32_t)(READ_BITS(UARTx->LCR, UART_LCR_DLS));
}

/**
  * @brief  Set the length of the stop bits
  *
  *  Register|BitsName
  *  --------|--------
  *  LCR | STOP
  *
  * @param  UARTx UART instance
  * @param  stop_bits This parameter can be one of the following values:
  *         @arg @ref LL_UART_STOPBITS_1
  *         @arg @ref LL_UART_STOPBITS_1_5 (*)
  *         @arg @ref LL_UART_STOPBITS_2 (*)
  *
  *         (*) STOPBITS_1_5 only valid when DataBits = 5
  *         (*) STOPBITS_2 is invalid when DataBits = 5
  * @retval None
  */
__STATIC_INLINE void ll_uart_set_stop_bits_length(uart_regs_t *UARTx, uint32_t stop_bits)
{
    MODIFY_REG(UARTx->LCR, UART_LCR_STOP, stop_bits);
}

/**
  * @brief  Retrieve the length of the stop bits
  *
  *  Register|BitsName
  *  --------|--------
  *  LCR | STOP
  *
  * @param  UARTx UART instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_UART_STOPBITS_1
  *         @arg @ref LL_UART_STOPBITS_1_5
  *         @arg @ref LL_UART_STOPBITS_2
  */
__STATIC_INLINE uint32_t ll_uart_get_stop_bits_length(uart_regs_t *UARTx)
{
    return (uint32_t)(READ_BITS(UARTx->LCR, UART_LCR_STOP));
}

/**
  * @brief  Configure Parity.
  * @note   This function selects if hardware parity control (generation and detection) is enabled or disabled.
  *         When the parity control is enabled (Odd,Even,0,1), computed parity bit is inserted at the MSB position
  *         and parity is checked on the received data.
  *
  *  Register|BitsName
  *  --------|--------
  *  LCR | SP
  *  LCR | EPS
  *  LCR | PEN
  *
  * @param  UARTx UART instance
  * @param  parity This parameter can be one of the following values:
  *         @arg @ref LL_UART_PARITY_NONE
  *         @arg @ref LL_UART_PARITY_EVEN
  *         @arg @ref LL_UART_PARITY_ODD
  *         @arg @ref LL_UART_PARITY_SP0
  *         @arg @ref LL_UART_PARITY_SP1
  * @retval None
  */
__STATIC_INLINE void ll_uart_set_parity(uart_regs_t *UARTx, uint32_t parity)
{
    MODIFY_REG(UARTx->LCR, UART_LCR_PARITY, parity);
}

/**
  * @brief  Return Parity configuration
  *
  *  Register|BitsName
  *  --------|--------
  *  LCR | SP
  *  LCR | EPS
  *  LCR | PEN
  *
  * @param  UARTx UART instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_UART_PARITY_NONE
  *         @arg @ref LL_UART_PARITY_EVEN
  *         @arg @ref LL_UART_PARITY_ODD
  *         @arg @ref LL_UART_PARITY_SP0
  *         @arg @ref LL_UART_PARITY_SP1
  */
__STATIC_INLINE uint32_t ll_uart_get_parity(uart_regs_t *UARTx)
{
    return (uint32_t)(READ_BITS(UARTx->LCR, UART_LCR_PARITY));
}

/**
  * @brief  Configure Character frame format (Datawidth, Parity control, Stop Bits)
  * @note   This function call is equivalent to the following function call sequence :
  *         - Data Width configuration using @ref ll_uart_set_data_bits_length() function
  *         - Parity Control and mode configuration using @ref ll_uart_set_parity() function
  *         - Stop bits configuration using @ref ll_uart_set_stop_bits_length() function
  *
  *  Register|BitsName
  *  --------|--------
  *  LCR | PS
  *  LCR | EPS
  *  LCR | PEN
  *  LCR | STOP
  *  LCR | DLS
  *
  * @param  UARTx UART instance
  * @param  data_bits This parameter can be one of the following values:
  *         @arg @ref LL_UART_DATABITS_5B
  *         @arg @ref LL_UART_DATABITS_6B
  *         @arg @ref LL_UART_DATABITS_7B
  *         @arg @ref LL_UART_DATABITS_8B
  * @param  parity This parameter can be one of the following values:
  *         @arg @ref LL_UART_PARITY_NONE
  *         @arg @ref LL_UART_PARITY_EVEN
  *         @arg @ref LL_UART_PARITY_ODD
  *         @arg @ref LL_UART_PARITY_SP0
  *         @arg @ref LL_UART_PARITY_SP1
  * @param  stop_bits This parameter can be one of the following values:
  *         @arg @ref LL_UART_STOPBITS_1
  *         @arg @ref LL_UART_STOPBITS_1_5 (*)
  *         @arg @ref LL_UART_STOPBITS_2 (*)
  *
  *         (*) STOPBITS_1_5 only valid when DataBits = 5
  *         (*) STOPBITS_2 is invalid when DataBits = 5
  * @retval None
  */
__STATIC_INLINE void ll_uart_config_character(uart_regs_t *UARTx,
                                              uint32_t     data_bits,
                                              uint32_t     parity,
                                              uint32_t     stop_bits)
{
    MODIFY_REG(UARTx->LCR, UART_LCR_PARITY | UART_LCR_STOP | UART_LCR_DLS, parity | stop_bits | data_bits);
}

/**
  * @brief  Set UART RTS pin state to Active/Inactive
  * @note   The RTS pin is ACTIVE when logic level is low, and INACTIVE when logic level is high.
  *
  *  Register|BitsName
  *  --------|--------
  *  SRTS | SRTS
  *  MCR | RTS
  *
  * @param  UARTx UART instance
  * @param  pin_state This parameter can be one of the following values:
  *         @arg @ref LL_UART_RTSPIN_STATE_ACTIVE
  *         @arg @ref LL_UART_RTSPIN_STATE_INACTIVE
  * @retval None
  */
__STATIC_INLINE void ll_uart_set_rts_pin_state(uart_regs_t *UARTx, uint32_t pin_state)
{
    WRITE_REG(UARTx->SRTS, pin_state);
}

/**
  * @brief  Get UART RTS pin state
  * @note   The RTS pin is ACTIVE when logic level is low, and INACTIVE when logic level is high.
  *
  *  Register|BitsName
  *  --------|--------
  *  SRTS | SRTS
  *  MCR | RTS
  *
  * @param  UARTx UART instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_UART_RTSPIN_STATE_ACTIVE
  *         @arg @ref LL_UART_RTSPIN_STATE_INACTIVE
  */
__STATIC_INLINE uint32_t ll_uart_get_rts_pin_state(uart_regs_t *UARTx)
{
    return (uint32_t)(READ_REG(UARTx->SRTS));
}

/**
  * @brief  Get UART CTS pin state
  * @note   The CTS pin is ACTIVE when logic level is low, and INACTIVE when logic level is high.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSR | CTS
  *
  * @param  UARTx UART instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_UART_CTSPIN_STATE_ACTIVE
  *         @arg @ref LL_UART_CTSPIN_STATE_INACTIVE
  */
__STATIC_INLINE uint32_t ll_uart_get_cts_pin_state(uart_regs_t *UARTx)
{
    return (uint32_t)(READ_BITS(UARTx->MSR, UART_MSR_CTS) >> UART_MSR_CTS_Pos);
}

/**
  * @brief  Indicate if CTS is changed since the last time the MSR was read
  *
  *  Register|BitsName
  *  --------|--------
  *  MSR | DCTS
  *
  * @param  UARTx UART instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_changed_cts(uart_regs_t *UARTx)
{
    return (uint32_t)(READ_BITS(UARTx->MSR, UART_MSR_DCTS) >> UART_MSR_DCTS_Pos);
}

/**
  * @brief  Configure SIR mode enable
  * @note   This function is used to Enable UART SIR mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  MCR | SIRE
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_enable_sir(uart_regs_t *UARTx)
{
    SET_BITS(UARTx->MCR, UART_MCR_SIRE);
}

/**
  * @brief  Configure SIR mode disable
  * @note   This function is used to Disable UART SIR mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  MCR | SIRE
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_disable_sir(uart_regs_t *UARTx)
{
    CLEAR_BITS(UARTx->MCR, UART_MCR_SIRE);
}

/**
  * @brief  Return SIR mode state
  * @note   This function is used to return UART SIR mode state.
  *
  *  Register|BitsName
  *  --------|--------
  *  MCR | SIRE
  *
  * @param  UARTx UART instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_enabled_sir(uart_regs_t *UARTx)
{
    return (READ_BITS(UARTx->MCR, UART_MCR_SIRE) == UART_MCR_SIRE);
}

/**
  * @brief  Configure HW Flow Control mode (None or Both CTS and RTS)
  * @note   This function is used to Enable/Disable UART Auto Flow Control.
  *
  *  Register|BitsName
  *  --------|--------
  *  MCR | AFCE
  *  MCR | RTS
  *
  * @param  UARTx UART instance
  * @param  hw_flow_ctrl This parameter can be one of the following values:
  *         @arg @ref LL_UART_HWCONTROL_NONE
  *         @arg @ref LL_UART_HWCONTROL_RTS_CTS
  * @retval None
  */
__STATIC_INLINE void ll_uart_set_hw_flow_ctrl(uart_regs_t *UARTx, uint32_t hw_flow_ctrl)
{
    MODIFY_REG(UARTx->MCR, UART_MCR_AFCE | UART_MCR_RTS, hw_flow_ctrl);
}

/**
  * @brief  Return HW Flow Control configuration (None or Both CTS and RTS)
  *
  *  Register|BitsName
  *  --------|--------
  *  MCR | AFCE
  *  MCR | RTS
  *
  * @param  UARTx UART instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_UART_HWCONTROL_NONE
  *         @arg @ref LL_UART_HWCONTROL_RTS_CTS
  */
__STATIC_INLINE uint32_t ll_uart_get_hw_flow_ctrl(uart_regs_t *UARTx)
{
    return (uint32_t)(READ_BITS(UARTx->MCR, UART_MCR_AFCE | UART_MCR_RTS));
}

/**
  * @brief  Enable Break sending
  *
  *  Register|BitsName
  *  --------|--------
  *  LCR | BC
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_enable_break_sending(uart_regs_t *UARTx)
{
    WRITE_REG(UARTx->SBCR, 0x1U);
}

/**
  * @brief  Disable Break sending
  *
  *  Register|BitsName
  *  --------|--------
  *  LCR | BC
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_disable_break_sending(uart_regs_t *UARTx)
{
    WRITE_REG(UARTx->SBCR, 0x0U);
}

/**
  * @brief  Indicate if Break sending is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  LCR | BC
  *
  * @param  UARTx UART instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_enabled_break_sending(uart_regs_t *UARTx)
{
    return READ_REG(UARTx->SBCR);
}

/**
  * @brief  Enable TX FIFO and RX FIFO
  *
  *  Register|BitsName
  *  --------|--------
  *  SFE | SFE
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_enable_fifo(uart_regs_t *UARTx)
{
    WRITE_REG(UARTx->SFE, 0x1U);
}

/**
  * @brief  Disable TX FIFO and RX FIFO
  *
  *  Register|BitsName
  *  --------|--------
  *  SFE | SFE
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_disable_fifo(uart_regs_t *UARTx)
{
    WRITE_REG(UARTx->SFE, 0x0U);
}

/**
  * @brief  Indicate if TX FIFO and RX FIFO is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  SFE | SFE
  *
  * @param  UARTx UART instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_enabled_fifo(uart_regs_t *UARTx)
{
    return READ_REG(UARTx->SFE);
}

/**
  * @brief  Set threshold of TX FIFO that triggers an THRE interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  STET | STET
  *
  * @param  UARTx UART instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_UART_TX_FIFO_TH_EMPTY
  *         @arg @ref LL_UART_TX_FIFO_TH_CHAR_2
  *         @arg @ref LL_UART_TX_FIFO_TH_QUARTER_FULL
  *         @arg @ref LL_UART_TX_FIFO_TH_HALF_FULL
  * @retval None
  */
__STATIC_INLINE void ll_uart_set_tx_fifo_threshold(uart_regs_t *UARTx, uint32_t threshold)
{
    WRITE_REG(UARTx->STET, threshold);
}

/**
  * @brief  Get threshold of TX FIFO that triggers an THRE interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  STET | STET
  *
  * @param  UARTx UART instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_UART_TX_FIFO_TH_EMPTY
  *         @arg @ref LL_UART_TX_FIFO_TH_CHAR_2
  *         @arg @ref LL_UART_TX_FIFO_TH_QUARTER_FULL
  *         @arg @ref LL_UART_TX_FIFO_TH_HALF_FULL
  */
__STATIC_INLINE uint32_t ll_uart_get_tx_fifo_threshold(uart_regs_t *UARTx)
{
    return (uint32_t)(READ_REG(UARTx->STET));
}

/**
  * @brief  Set threshold of RX FIFO that triggers an RDA interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  SRT | SRT
  *
  * @param  UARTx UART instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_UART_RX_FIFO_TH_CHAR_1
  *         @arg @ref LL_UART_RX_FIFO_TH_QUARTER_FULL
  *         @arg @ref LL_UART_RX_FIFO_TH_HALF_FULL
  *         @arg @ref LL_UART_RX_FIFO_TH_FULL_2
  * @retval None
  */
__STATIC_INLINE void ll_uart_set_rx_fifo_threshold(uart_regs_t *UARTx, uint32_t threshold)
{
    WRITE_REG(UARTx->SRT, threshold);
}

/**
  * @brief  Get threshold of RX FIFO that triggers an RDA interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  SRT | SRT
  *
  * @param  UARTx UART instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_UART_RX_FIFO_TH_CHAR_1
  *         @arg @ref LL_UART_RX_FIFO_TH_QUARTER_FULL
  *         @arg @ref LL_UART_RX_FIFO_TH_HALF_FULL
  *         @arg @ref LL_UART_RX_FIFO_TH_FULL_2
  */
__STATIC_INLINE uint32_t ll_uart_get_rx_fifo_threshold(uart_regs_t *UARTx)
{
    return (uint32_t)(READ_REG(UARTx->SRT));
}

/**
  * @brief  Get FIFO Transmission Level
  *
  *  Register|BitsName
  *  --------|--------
  *  TFL | TFL
  *
  * @param  UARTx UART instance
  * @retval Returned value can be one of the following values:
  */
__STATIC_INLINE uint32_t ll_uart_get_tx_fifo_level(uart_regs_t *UARTx)
{
    return (uint32_t)(READ_REG(UARTx->TFL));
}

/**
  * @brief  Get FIFO reception Level
  *
  *  Register|BitsName
  *  --------|--------
  *  RFL | RFL
  *
  * @param  UARTx UART instance
  * @retval Returned value can be one of the following values:
  */
__STATIC_INLINE uint32_t ll_uart_get_rx_fifo_level(uart_regs_t *UARTx)
{
    return (uint32_t)(READ_REG(UARTx->RFL));
}

/**
  * @brief  Flush Receive FIFO
  *
  *  Register|BitsName
  *  --------|--------
  *  SRR | RFR
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_flush_rx_fifo(uart_regs_t *UARTx)
{
    WRITE_REG(UARTx->SRR, UART_SRR_RFR);
}

/**
  * @brief  Flush Transmit FIFO
  *
  *  Register|BitsName
  *  --------|--------
  *  SRR | XFR
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_flush_tx_fifo(uart_regs_t *UARTx)
{
    WRITE_REG(UARTx->SRR, UART_SRR_XFR);
}

/**
  * @brief  Reset UART
  * @note   This function asynchronously resets the DW_apb_uart and synchronously
  *         removes the reset assertion. For a two clock implementation, both pclk
  *         and sclk domains will be reset.
  *
  *  Register|BitsName
  *  --------|--------
  *  SRR | UR
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_reset(uart_regs_t *UARTx)
{
    WRITE_REG(UARTx->SRR, UART_SRR_UR);
}

/** @} */

/** @defgroup UART_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable Modem Status Interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | EDSSI
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_enabled_it_ms(uart_regs_t *UARTx)
{
    SET_BITS(UARTx->DLH_IER.IER, UART_IER_EDSSI);
}

/**
  * @brief  Enable Receiver Line Status Interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | RLS
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_enable_it_rls(uart_regs_t *UARTx)
{
    SET_BITS(UARTx->DLH_IER.IER, UART_IER_ERLS);
}

/**
  * @brief  Enable Transmit Holding Register Empty Interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | PTIME
  *  IER | ETBEI
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_enable_it_thre(uart_regs_t *UARTx)
{
    SET_BITS(UARTx->DLH_IER.IER, UART_IER_PTIME | UART_IER_ETBEI);
}

/**
  * @brief  Enable Received Data Available Interrupt and Character Timeout Interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | ERBFI
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_enable_it_rda(uart_regs_t *UARTx)
{
    SET_BITS(UARTx->DLH_IER.IER, UART_IER_ERBFI);
}

/**
  * @brief  Disable Modem Status Interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | EDSSI
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_disable_it_ms(uart_regs_t *UARTx)
{
    CLEAR_BITS(UARTx->DLH_IER.IER, UART_IER_EDSSI);
}

/**
  * @brief  Disable Receiver Line Status Interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | RLS
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_disable_it_rls(uart_regs_t *UARTx)
{
    CLEAR_BITS(UARTx->DLH_IER.IER, UART_IER_ERLS);
}

/**
  * @brief  Disable Transmit Holding Register Empty Interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | PTIME
  *  IER | ETBEI
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_disable_it_thre(uart_regs_t *UARTx)
{
    CLEAR_BITS(UARTx->DLH_IER.IER, UART_IER_PTIME | UART_IER_ETBEI);
}

/**
  * @brief  Disable Received Data Available Interrupt and Character Timeout Interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | ERBFI
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_disable_it_rda(uart_regs_t *UARTx)
{
    CLEAR_BITS(UARTx->DLH_IER.IER, UART_IER_ERBFI);
}

/**
  * @brief  Check if the UART Modem Status Interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | EDSSI
  *
  * @param  UARTx UART instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_enabled_it_ms(uart_regs_t *UARTx)
{
    return (READ_BITS(UARTx->DLH_IER.IER, UART_IER_EDSSI) == (UART_IER_EDSSI));
}

/**
  * @brief  Check if the UART Receiver Line Status Interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | RLS
  *
  * @param  UARTx UART instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_enabled_it_rls(uart_regs_t *UARTx)
{
    return (READ_BITS(UARTx->DLH_IER.IER, UART_IER_ERLS) == (UART_IER_ERLS));
}

/**
  * @brief  Check if the UART Transmit Holding Register Empty Interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | PTIME
  *  IER | ETBEI
  *
  * @param  UARTx UART instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_enabled_it_thre(uart_regs_t *UARTx)
{
    return (READ_BITS(UARTx->DLH_IER.IER, UART_IER_PTIME | UART_IER_ETBEI) == (UART_IER_PTIME | UART_IER_ETBEI));
}

/**
  * @brief  Check if the UART Received Data Available Interrupt and Character Timeout Interrupt
  *         is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | ERBFI
  *
  * @param  UARTx UART instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_enabled_it_rda(uart_regs_t *UARTx)
{
    return (READ_BITS(UARTx->DLH_IER.IER, UART_IER_ERBFI) == (UART_IER_ERBFI));
}

/**
  * @brief  Enable the specified UART Interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | EDSSI
  *  IER | ERLS
  *  IER | PTIME
  *  IER | ETBEI
  *  IER | ERBFI
  *
  * @param  UARTx UART instance
  * @param  mask This parameter can be a combination of the following values:
  *         @arg @ref LL_UART_IER_MS
  *         @arg @ref LL_UART_IER_RLS
  *         @arg @ref LL_UART_IER_THRE
  *         @arg @ref LL_UART_IER_RDA
  * @retval None
  */
__STATIC_INLINE void ll_uart_enable_it(uart_regs_t *UARTx, uint32_t mask)
{
    SET_BITS(UARTx->DLH_IER.IER, mask);
}

/**
  * @brief  Disable the specified UART Interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | EDSSI
  *  IER | ERLS
  *  IER | PTIME
  *  IER | ETBEI
  *  IER | ERBFI
  *
  * @param  UARTx UART instance
  * @param  mask This parameter can be a combination of the following values:
  *         @arg @ref LL_UART_IER_MS
  *         @arg @ref LL_UART_IER_RLS
  *         @arg @ref LL_UART_IER_THRE
  *         @arg @ref LL_UART_IER_RDA
  * @retval None
  */
__STATIC_INLINE void ll_uart_disable_it(uart_regs_t *UARTx, uint32_t mask)
{
    CLEAR_BITS(UARTx->DLH_IER.IER, mask);
}

/**
  * @brief  Check if the specified UART Interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  IER | EDSSI
  *  IER | ERLS
  *  IER | PTIME
  *  IER | ETBEI
  *  IER | ERBFI
  *
  * @param  UARTx UART instance
  * @param  mask This parameter can be a combination of the following values:
  *         @arg @ref LL_UART_IER_MS
  *         @arg @ref LL_UART_IER_RLS
  *         @arg @ref LL_UART_IER_THRE
  *         @arg @ref LL_UART_IER_RDA
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_enabled_it(uart_regs_t *UARTx, uint32_t mask)
{
    return (READ_BITS(UARTx->DLH_IER.IER, mask) == (mask));
}

/** @} */

/** @defgroup UART_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Get UART Receive Line Status Flag
  * @note This function is used to get OE/PE/FE/BI/THRE/TEMT/RFE flags in LSR register.
  *       After LSR register was read, OE/PE/FE/BI/RFE flags will be cleared.
  *
  *  Register|BitsName
  *  --------|--------
  *  LSR | OE
  *  LSR | PE
  *  LSR | FE
  *  LSR | BI
  *  LSR | THRE
  *  LSR | TEMT
  *  LSR | RFE
  *
  * @param  UARTx UART instance
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_UART_LSR_OE
  *         @arg @ref LL_UART_LSR_PE
  *         @arg @ref LL_UART_LSR_FE
  *         @arg @ref LL_UART_LSR_BI
  *         @arg @ref LL_UART_LSR_THRE
  *         @arg @ref LL_UART_LSR_TEMT
  *         @arg @ref LL_UART_LSR_RFE
  */
__STATIC_INLINE uint32_t ll_uart_get_line_status_flag(uart_regs_t *UARTx)
{
    return ((uint32_t)READ_REG(UARTx->LSR));
}

/**
  * @brief  Clear UART Receive Line Status Flag
  * @note   OE/PE/FE/BI/RFE flags can be cleared by reading LSR register.
  *
  *  Register|BitsName
  *  --------|--------
  *  LSR | OE
  *  LSR | PE
  *  LSR | FE
  *  LSR | BI
  *  LSR | RFE
  *
  * @param  UARTx UART instance
  * @retval None
  */
__STATIC_INLINE void ll_uart_clear_line_status_flag(uart_regs_t *UARTx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(UARTx->LSR);
    (void) tmpreg;
}

/**
  * @brief  Check if the UART Receive FIFO Full Flag is set or not
  *
  *  Register|BitsName
  *  --------|--------
  *  USR | RFF
  *
  * @param  UARTx UART instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_active_flag_rff(uart_regs_t *UARTx)
{
    return (READ_BITS(UARTx->USR, UART_USR_RFF) == UART_USR_RFF);
}

/**
  * @brief  Check if the UART Receive FIFO Not Empty Flag is set or not
  *
  *  Register|BitsName
  *  --------|--------
  *  USR | RFNE
  *
  * @param  UARTx UART instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_active_flag_rfne(uart_regs_t *UARTx)
{
    return (READ_BITS(UARTx->USR, UART_USR_RFNE) == UART_USR_RFNE);
}

/**
  * @brief  Check if the UART Transmit FIFO Empty Flag is set or not
  *
  *  Register|BitsName
  *  --------|--------
  *  USR | TFE
  *
  * @param  UARTx UART instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_active_flag_tfe(uart_regs_t *UARTx)
{
    return (READ_BITS(UARTx->USR, UART_USR_TFE) == UART_USR_TFE);
}

/**
  * @brief  Check if the UART Transmit FIFO Not Full Flag is set or not
  *
  *  Register|BitsName
  *  --------|--------
  *  USR | TFNF
  *
  * @param  UARTx UART instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_uart_is_active_flag_tfnf(uart_regs_t *UARTx)
{
    return (READ_BITS(UARTx->USR, UART_USR_TFNF) == UART_USR_TFNF);
}

/**
  * @brief  Get UART interrupt flags
  * @note   The interrupt flags will be cleared after reading IIR.
  *         If interrupt was triggered when reading IIR register, the interrupt will be pended,
  *         and No Interrupt Pending Flag will be RESET, read IIR again can get the pended interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  IIR | IID
  *
  * @param  UARTx UART instance
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_UART_IIR_MS
  *         @arg @ref LL_UART_IIR_NIP
  *         @arg @ref LL_UART_IIR_THRE
  *         @arg @ref LL_UART_IIR_RDA
  *         @arg @ref LL_UART_IIR_RLS
  *         @arg @ref LL_UART_IIR_CTO
  */
__STATIC_INLINE uint32_t ll_uart_get_it_flag(uart_regs_t *UARTx)
{
    return (uint32_t)(READ_BITS(UARTx->FCR_IIR.IIR, UART_IIR_IID));
}

/** @} */

/** @defgroup UART_LL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Get the data register address used for DMA transfer
  * @note   The address of data register RBR is the same as the address of THR.
  *
  *  Register|BitsName
  *  --------|--------
  *  RBR | RBR
  *  THR | THR
  *
  * @param  UARTx UART instance
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t ll_uart_dma_get_register_address(uart_regs_t *UARTx)
{
    return ((uint32_t) &(UARTx->RBR_DLL_THR));
}

/** @} */

/** @defgroup UART_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Read Receiver Data register (Receive Data value, 8 bits)
  *
  *  Register|BitsName
  *  --------|--------
  *  RBR | RBR
  *
  * @param  UARTx UART instance
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t ll_uart_receive_data8(uart_regs_t *UARTx)
{
    return (uint8_t)(READ_REG(UARTx->RBR_DLL_THR.RBR));
}

/**
  * @brief  Write in Transmitter Data Register (Transmit Data value, 8 bits)
  *
  *  Register|BitsName
  *  --------|--------
  *  THR | THR
  *
  * @param  UARTx UART instance
  * @param  value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void ll_uart_transmit_data8(uart_regs_t *UARTx, uint8_t value)
{
    WRITE_REG(UARTx->RBR_DLL_THR.THR, value);
}

/** @} */

/** @defgroup UART_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize UART registers (Registers restored to their default values).
  * @param  UARTx UART instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: UART registers are de-initialized
  *          - ERROR: UART registers are not de-initialized
  */
error_status_t ll_uart_deinit(uart_regs_t *UARTx);

/**
  * @brief  Initialize UART registers according to the specified
  *         parameters in p_uart_init.
  * @param  UARTx UART instance
  * @param  p_uart_init Pointer to a ll_uart_init_t structure that contains the configuration
  *                         information for the specified UART peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: UART registers are initialized according to p_uart_init content
  *          - ERROR: Problem occurred during UART Registers initialization
  */
error_status_t ll_uart_init(uart_regs_t *UARTx, ll_uart_init_t *p_uart_init);

/**
  * @brief Set each field of a @ref ll_uart_init_t type structure to default value.
  * @param p_uart_init  Pointer to a @ref ll_uart_init_t structure
  *                     whose fields will be set to default values.
  * @retval None
  */
void ll_uart_struct_init(ll_uart_init_t *p_uart_init);

/** @} */

/** @} */

#endif /* UART0 || UART1 || UART2 || UART3 || UART4 || UART5*/

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_UART_H__ */

/** @} */

/** @} */

/** @} */

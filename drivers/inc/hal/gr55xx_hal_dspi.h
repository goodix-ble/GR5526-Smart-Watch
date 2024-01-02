/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_dspi.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of DSPI HAL library.
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

/** @defgroup HAL_DSPI DSPI
  * @brief DSPI HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_DSPI_H__
#define __GR55xx_HAL_DSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_dspi.h"
#include "gr55xx_hal_def.h"
#include "gr55xx_hal_pwr_mgmt.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_DSPI_ENUMERATIONS Enumerations
  * @{
  */

/**
  * @brief HAL DSPI State Enumerations definition
  */
typedef enum
{
    HAL_DSPI_STATE_RESET             = 0x00,    /**< Peripheral not initialized                            */
    HAL_DSPI_STATE_READY             = 0x01,    /**< Peripheral initialized and ready for use              */
    HAL_DSPI_STATE_BUSY              = 0x02,    /**< Peripheral in indirect mode and busy                  */
    HAL_DSPI_STATE_BUSY_INDIRECT_TX  = 0x12,    /**< Peripheral in indirect mode with transmission ongoing */
    HAL_DSPI_STATE_ABORT             = 0x08,    /**< Peripheral with abort request ongoing                 */
    HAL_DSPI_STATE_ERROR             = 0x04     /**< Peripheral in error                                   */
} hal_dspi_state_t;

/** @} */

/** @addtogroup HAL_DSPI_STRUCTURES Structures
  * @{
  */

/** @defgroup DSPI_Configuration DSPI Configuration
  * @{
  */

/**
  * @brief QSPI init Structure definition
  */
typedef struct _dspi_init_t
{
    uint32_t data_size;             /**< Specifies the DSPI data width.
                                         This parameter can be a value of @ref QSPI_Data_Size Data Width.*/

    uint32_t baud_rate;             /**< Specifies the BaudRate prescaler value which will be used to configure the transmit and receive SCK clock.
                                         This parameter can be one even value between 0 and 7 @ref DSPI_Clock_Sel Clock Select.*/

    uint32_t dspi_mode;             /**< Specifies the DSPI interface mode.
                                         This parameter can be a value of @ref DSPI_Interface_Mode DSPI Interface Mode.*/
} dspi_init_t;

/** @} */

/** @defgroup DSPI_handle DSPI handle
  * @{
  */

/**
  * @brief DSPI handle Structure definition
  */
typedef struct _dspi_handle
{
    dspi_regs_t           *p_instance;               /**< DSPI registers base address        */

    dspi_init_t           init;                      /**< DSPI communication parameters      */

    uint8_t               *p_tx_buffer;              /**< Pointer to DSPI Tx transfer Buffer */

    __IO uint32_t         tx_xfer_size;              /**< DSPI Tx Transfer size              */

    __IO uint32_t         tx_xfer_count;             /**< DSPI Tx Transfer Counter           */

    void (*write_fifo)(struct _dspi_handle *p_dspi); /**< Pointer to DSPI Tx transfer FIFO write function */

    dma_handle_t          *p_dmatx;                   /**< DSPI Tx DMA Handle parameters   */

    __IO hal_lock_t       lock;                      /**< Locking object                     */

    __IO hal_dspi_state_t state;                     /**< DSPI communication state           */

    __IO uint32_t         error_code;                /**< DSPI Error code                    */

    uint32_t              timeout;                   /**< Timeout for the DSPI memory access */

    periph_device_number_t dspi_number;              /**< DSPI used for Sleep management */

    uint32_t              retention[3];              /**< DSPI important register information. */
} dspi_handle_t;
/** @} */

/** @defgroup DSPI_Command DSPI command
  * @{
  */

/**
  * @brief DSPI command Structure definition
  */
typedef struct _dspi_command_t
{
    uint32_t instruction;               /**< Specifies the Instruction to be sent.
                                             This parameter can be a value (8-bit, 16bit, 32 bit). */

    uint32_t instruction_size;          /**< Specifies the Instruction Size.
                                             This parameter can be a value of @ref DSPI_Instruction_Size. */

    uint32_t data_size;                /**< Specifies the DSPI data width.
                                             This parameter can be a value of @ref QSPI_Data_Size. */

    uint32_t length;                    /**< Specifies the number of data to transfer. (This is the number of bytes).
                                             This parameter can be any value between 0 and 0xFFFFFFFF (0 means undefined length
                                             until end of memory).  */

} dspi_command_t;
/** @} */

/** @} */

/** @addtogroup HAL_DSPI_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/**
  * @brief HAL_DSPI Callback function definition
  */

typedef struct _dspi_callback
{
    void (*dspi_msp_init)(dspi_handle_t *p_dspi);                   /**< DSPI init MSP callback                 */
    void (*dspi_msp_deinit)(dspi_handle_t *p_dspi);                 /**< DSPI de-init MSP callback              */
    void (*dspi_abort_callback)(dspi_handle_t *p_dspi);             /**< DSPI abort callback                    */
    void (*dspi_error_callback)(dspi_handle_t *p_dspi);             /**< DSPI error callback                    */
    void (*dspi_tx_cplt_callback)(dspi_handle_t *p_dspi);           /**< DSPI tx transfer completed callback    */
} dspi_callback_t;

/** @} */

/**
  * @defgroup  HAL_DSPI_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DSPI_Exported_Constants DSPI Exported Constants
  * @{
  */

/** @defgroup DSPI_Error_Code DSPI Error Code
  * @{
  */
#define HAL_DSPI_ERROR_NONE                     ((uint32_t)0x00000000)          /**< No error                 */
#define HAL_DSPI_ERROR_TIMEOUT                  ((uint32_t)0x00000001)          /**< Timeout error            */
#define HAL_DSPI_ERROR_TRANSFER                 ((uint32_t)0x00000002)          /**< Transfer error           */
#define HAL_DSPI_ERROR_DMA                      ((uint32_t)0x00000004)          /**< DMA transfer error       */
#define HAL_DSPI_ERROR_INVALID_PARAM            ((uint32_t)0x00000008)          /**< Invalid parameter error  */
/** @} */

/** @defgroup DSPI_Clock_Sel Clock Select
  * @{
  */
#define DSPI_BAUD_RATE_2P1PCLK                  LL_DSPI_BAUD_RATE_2P1PCLK       /**< DSPI Baud rate fPCLK / 2   */
#define DSPI_BAUD_RATE_4P1PCLK                  LL_DSPI_BAUD_RATE_4P1PCLK       /**< DSPI Baud rate fPCLK / 4   */
#define DSPI_BAUD_RATE_8P1PCLK                  LL_DSPI_BAUD_RATE_8P1PCLK       /**< DSPI Baud rate fPCLK / 8   */
#define DSPI_BAUD_RATE_16P1PCLK                 LL_DSPI_BAUD_RATE_16P1PCLK      /**< DSPI Baud rate fPCLK / 16  */
#define DSPI_BAUD_RATE_32P1PCLK                 LL_DSPI_BAUD_RATE_32P1PCLK      /**< DSPI Baud rate fPCLK / 32  */
#define DSPI_BAUD_RATE_64P1PCLK                 LL_DSPI_BAUD_RATE_64P1PCLK      /**< DSPI Baud rate fPCLK / 64  */
#define DSPI_BAUD_RATE_128P1PCLK                LL_DSPI_BAUD_RATE_128P1PCLK     /**< DSPI Baud rate fPCLK / 128 */
#define DSPI_BAUD_RATE_256P1PCLK                LL_DSPI_BAUD_RATE_256PCLK       /**< DSPI Baud rate fPCLK / 256 */

/** @} */

/** @defgroup DSPI_Interface_Mode DSPI Interface mode
  * @{
  */
#define DSPI_PROT_MODE_3W1L                     LL_DSPI_PROT_MODE_3W1L          /**<  DSPI 3-Wire 1-Lane Interface      */
#define DSPI_PROT_MODE_4W1L                     LL_DSPI_PROT_MODE_4W1L          /**<  DSPI 4-Wire 1-Lane Interface      */
#define DSPI_PROT_MODE_4W2L                     LL_DSPI_PROT_MODE_4W2L          /**<  DSPI 4-Wire 2-Lane Interface      */
/** @} */

/** @defgroup DSPI_Instruction_Size DSPI Instruction Size
  * @{
  */
#define DSPI_INSTSIZE_08_BITS                   ((uint32_t)0x01)                /**<  8-bit Instruction     */
#define DSPI_INSTSIZE_16_BITS                   ((uint32_t)0x02)                /**< 16-bit Instruction     */
#define DSPI_INSTSIZE_32_BITS                   ((uint32_t)0x03)                /**< 32-bit Instruction     */
/** @} */

/** @defgroup DSPI_Data_Size Data Width
  * @{
  */
#define DSPI_DATASIZE_04_BITS                   LL_DSPI_DATASIZE_4BIT           /**< Data length for DSPI transfer:  4 bits */
#define DSPI_DATASIZE_05_BITS                   LL_DSPI_DATASIZE_5BIT           /**< Data length for DSPI transfer:  5 bits */
#define DSPI_DATASIZE_06_BITS                   LL_DSPI_DATASIZE_6BIT           /**< Data length for DSPI transfer:  6 bits */
#define DSPI_DATASIZE_07_BITS                   LL_DSPI_DATASIZE_7BIT           /**< Data length for DSPI transfer:  7 bits */
#define DSPI_DATASIZE_08_BITS                   LL_DSPI_DATASIZE_8BIT           /**< Data length for DSPI transfer:  8 bits */
#define DSPI_DATASIZE_09_BITS                   LL_DSPI_DATASIZE_9BIT           /**< Data length for DSPI transfer:  9 bits */
#define DSPI_DATASIZE_10_BITS                   LL_DSPI_DATASIZE_10BIT          /**< Data length for DSPI transfer:  10 bits */
#define DSPI_DATASIZE_11_BITS                   LL_DSPI_DATASIZE_11BIT          /**< Data length for DSPI transfer:  11 bits */
#define DSPI_DATASIZE_12_BITS                   LL_DSPI_DATASIZE_12BIT          /**< Data length for DSPI transfer:  12 bits */
#define DSPI_DATASIZE_13_BITS                   LL_DSPI_DATASIZE_13BIT          /**< Data length for DSPI transfer:  13 bits */
#define DSPI_DATASIZE_14_BITS                   LL_DSPI_DATASIZE_14BIT          /**< Data length for DSPI transfer:  14 bits */
#define DSPI_DATASIZE_15_BITS                   LL_DSPI_DATASIZE_15BIT          /**< Data length for DSPI transfer:  15 bits */
#define DSPI_DATASIZE_16_BITS                   LL_DSPI_DATASIZE_16BIT          /**< Data length for DSPI transfer:  16 bits */
#define DSPI_DATASIZE_17_BITS                   LL_DSPI_DATASIZE_17BIT          /**< Data length for DSPI transfer:  17 bits */
#define DSPI_DATASIZE_18_BITS                   LL_DSPI_DATASIZE_18BIT          /**< Data length for DSPI transfer:  18 bits */
#define DSPI_DATASIZE_19_BITS                   LL_DSPI_DATASIZE_19BIT          /**< Data length for DSPI transfer:  19 bits */
#define DSPI_DATASIZE_20_BITS                   LL_DSPI_DATASIZE_20BIT          /**< Data length for DSPI transfer:  20 bits */
#define DSPI_DATASIZE_21_BITS                   LL_DSPI_DATASIZE_21BIT          /**< Data length for DSPI transfer:  21 bits */
#define DSPI_DATASIZE_22_BITS                   LL_DSPI_DATASIZE_22BIT          /**< Data length for DSPI transfer:  22 bits */
#define DSPI_DATASIZE_23_BITS                   LL_DSPI_DATASIZE_23BIT          /**< Data length for DSPI transfer:  23 bits */
#define DSPI_DATASIZE_24_BITS                   LL_DSPI_DATASIZE_24BIT          /**< Data length for DSPI transfer:  24 bits */
#define DSPI_DATASIZE_25_BITS                   LL_DSPI_DATASIZE_25BIT          /**< Data length for DSPI transfer:  25 bits */
#define DSPI_DATASIZE_26_BITS                   LL_DSPI_DATASIZE_26BIT          /**< Data length for DSPI transfer:  26 bits */
#define DSPI_DATASIZE_27_BITS                   LL_DSPI_DATASIZE_27BIT          /**< Data length for DSPI transfer:  27 bits */
#define DSPI_DATASIZE_28_BITS                   LL_DSPI_DATASIZE_28BIT          /**< Data length for DSPI transfer:  28 bits */
#define DSPI_DATASIZE_29_BITS                   LL_DSPI_DATASIZE_29BIT          /**< Data length for DSPI transfer:  29 bits */
#define DSPI_DATASIZE_30_BITS                   LL_DSPI_DATASIZE_30BIT          /**< Data length for DSPI transfer:  30 bits */
#define DSPI_DATASIZE_31_BITS                   LL_DSPI_DATASIZE_31BIT          /**< Data length for DSPI transfer:  31 bits */
#define DSPI_DATASIZE_32_BITS                   LL_DSPI_DATASIZE_32BIT          /**< Data length for DSPI transfer:  32 bits */

/** @} */

/** @defgroup DSPI_Flags DSPI Flags
  * @{
  */
#define DSPI_FLAG_FFE                           LL_DSPI_SR_FFE                  /**< Frame format error flag    */
#define DSPI_FLAG_OVR                           LL_DSPI_SR_OVR                  /**< Rx overrun flag            */
#define DSPI_FLAG_MODE                          LL_DSPI_SR_MODF                 /**< Mode fault flag            */
#define DSPI_FLAG_RFNE                          LL_DSPI_SR_RFNE                 /**< Rx FIFO not empty flag     */
#define DSPI_FLAG_TFE                           LL_DSPI_SR_TFE                  /**< Tx FIFO empty flag         */
#define DSPI_FLAG_BUSY                          LL_DSPI_SR_BUSY                 /**< Busy flag                  */
/** @} */

/** @defgroup DSPI_Interrupts DSPI Interrupts
  * @{
  */
#define DSPI_IT_RXNE                            LL_DSPI_IM_RXNE                 /**< Receive FIFO not empty Interrupt       */
#define DSPI_IT_ERR                             LL_DSPI_IM_ER                   /**< Error Interrupt   */
#define DSPI_IT_TXE                             LL_DSPI_IM_TXE                  /**< Transmit FIFO Empty Interrupt          */
/** @} */

/** @defgroup DSPI_RX_FIFO_Threshold DSPI rx FIFO threshold
  * @{
  */
#define DSPI_RX_FIFO_TH_1P2                     LL_DSPI_FRXTH_1P2               /**< FIFO level is 1/2  (8 bytes) */
#define DSPI_RX_FIFO_TH_1P4                     LL_DSPI_FRXTH_1P4               /**< FIFO level is 1/4  (4 bytes)  */
#define DSPI_RX_FIFO_TH_1P8                     LL_DSPI_FRXTH_1P8               /**< FIFO level is 1/8  (2 bytes) */
#define DSPI_RX_FIFO_TH_1P16                    LL_DSPI_FRXTH_1P16              /**< FIFO level is 1/16 (1 bytes)  */
/** @} */


/** @defgroup DSPI_Data_mode DSPI Data mode
  * @{
  */
#define DSPI_TRANSPORT_DATA                     LL_DSPI_DCX_DATA                /**< Transport data */
#define DSPI_TRANSPORT_CMD                      LL_DSPI_DCX_CMD                 /**< Transport cmd  */
/** @} */


/** @defgroup DSPI_Timeout_definition DSPI Timeout_definition
  * @{
  */
#define HAL_DSPI_TIMEOUT_DEFAULT_VALUE ((uint32_t)5000)         /**< 5s */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DSPI_Exported_Macros DSPI Exported Macros
  * @{
  */

/** @brief  Reset DSPI handle states.
  * @param  __HANDLE__ DPI handle.
  * @retval None
  */
#define __HAL_DSPI_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_DSPI_STATE_RESET)

/** @brief  Enable the specified DSPI peripheral.
  * @param  __HANDLE__ Specifies the DSPI Handle.
  * @retval None
  */
#define __HAL_DSPI_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->CTRL1, DSPI_CR1_EN)

/** @brief  Disable the specified DSPI peripheral.
  * @param  __HANDLE__ Specifies the DSPI Handle.
  * @retval None
  */
#define __HAL_DSPI_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->CTRL1, DSPI_CR1_EN)

/** @brief  Enable the DSPI DMA TX Request.
  * @param  __HANDLE__ Specifies the DSPI Handle.
  * @retval None
  */
#define __HAL_DSPI_ENABLE_DMATX(__HANDLE__)                     SET_BITS((__HANDLE__)->p_instance->CTRL2, DSPI_CR2_TXDMAEN)

/** @brief  Enable the DSPI DMA RX Request.
  * @param  __HANDLE__ Specifies the DSPI Handle.
  * @retval None
  */
#define __HAL_DSPI_ENABLE_DMARX(__HANDLE__)                     SET_BITS((__HANDLE__)->p_instance->CTRL2, DSPI_CR2_RXDMAEN)

/** @brief  Disable the DSPI DMA TX Request.
  * @param  __HANDLE__ Specifies the DSPI Handle.
  * @retval None
  */
#define __HAL_DSPI_DISABLE_DMATX(__HANDLE__)                    CLEAR_BITS((__HANDLE__)->p_instance->CTRL2, DSPI_CR2_TXDMAEN)

/** @brief  Disable the DSPI DMA RX Request.
  * @param  __HANDLE__ Specifies the DSPI Handle.
  * @retval None
  */
#define __HAL_DSPI_DISABLE_DMARX(__HANDLE__)                    CLEAR_BITS((__HANDLE__)->p_instance->CTRL2, DSPI_CR2_RXDMAEN)

/** @brief  Enable the specified DSPI interrupts.
  * @param  __HANDLE__ Specifies the DSPI Handle.
  * @param  __INTERRUPT__ Specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg @ref DSPI_IT_RXNE Receive FIFO not empty Interrupt enable
  *            @arg @ref DSPI_IT_TXE  Transmit FIFO Empty Interrupt enable
  *            @arg @ref DSPI_IT_ERR  Error Interrupt Interrupt enable
  * @retval None
  */
#define __HAL_DSPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)         SET_BITS((__HANDLE__)->p_instance->CTRL2, (__INTERRUPT__))

/** @brief  Disable the specified DSPI interrupts.
  * @param  __HANDLE__ Specifies the DSPI handle.
  * @param  __INTERRUPT__ Specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg @ref DSPI_IT_RXNE Receive FIFO not empty Interrupt disable
  *            @arg @ref DSPI_IT_TXE  Transmit FIFO Empty Interrupt disable
  *            @arg @ref DSPI_IT_ERR  Error Interrupt Interrupt disable
  * @retval None
  */
#define __HAL_DSPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)        CLEAR_BITS((__HANDLE__)->p_instance->CTRL2, (__INTERRUPT__))

/** @brief  Check whether the specified DSPI interrupt source is enabled or not.
  * @param  __HANDLE__ Specifies the DSPI Handle.
  * @param  __INTERRUPT__ Specifies the interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg @ref DSPI_IT_RXNE Receive FIFO not empty Interrupt disable
  *            @arg @ref DSPI_IT_TXE  Transmit FIFO Empty Interrupt disable
  *            @arg @ref DSPI_IT_ERR  Error Interrupt Interrupt disable
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __HAL_DSPI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)     (READ_BITS((__HANDLE__)->p_instance->CTRL2, (__INTERRUPT__)) == (__INTERRUPT__))

/** @brief  Check whether the specified DSPI flag is set or not.
  * @param  __HANDLE__ Specifies the DSPI Handle.
  * @param  __FLAG__ Specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref DSPI_FLAG_FFE Frame format error flag
  *            @arg @ref DSPI_FLAG_OVR  Rx overrun flag
  *            @arg @ref DSPI_FLAG_MODE spi mode flag
  *            @arg @ref DSPI_FLAG_RFNE Rx FIFO not empty flag
  *            @arg @ref DSPI_FLAG_TFE  Tx FIFO empty flag
  *            @arg @ref DSPI_FLAG_BUSY Busy flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_DSPI_GET_FLAG(__HANDLE__, __FLAG__)               ((READ_BITS((__HANDLE__)->p_instance->STAT, (__FLAG__)) != 0) ? SET : RESET)

/** @brief  Flush the DSPI receiver FIFO.
  * @param  __HANDLE__ Specifies the DSPI Handle.
  * @retval None
  */
#define __HAL_DSPI_FLUSH_FIFO(__HANDLE__)                       ll_dspi_flush_fifo((__HANDLE__)->p_instance)

/** @brief  Set the DSPI data transport type.
  * @param  __HANDLE__ Specifies the DSPI Handle.
  * @param  __TYPE__   Specifies the DSPI data transport type.
  * @retval None
  */
#define __HAL_DSPI_TRANSPORT_TYPE(__HANDLE__, __TYPE__)         ll_dspi_set_dcx((__HANDLE__)->p_instance, (__TYPE__))

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup DSPI_Private_Macro DSPI Private Macros
  * @{
  */

/** @brief  Check if DSPI Clock Prescaler is valid.
  * @param  __PRESCALER__ DSPI Clock Prescaler.
  * @retval SET (__PRESCALER__ is valid) or RESET (__PRESCALER__ is invalid)
  */
#define IS_DSPI_CLOCK_PRESCALER(__PRESCALER__)  (((__PRESCALER__) == DSPI_BAUD_RATE_2P1PCLK) || \
                                                  ((__PRESCALER__) == DSPI_BAUD_RATE_4P1PCLK) || \
                                                  ((__PRESCALER__) == DSPI_BAUD_RATE_8P1PCLK) || \
                                                  ((__PRESCALER__) == DSPI_BAUD_RATE_16P1PCLK) || \
                                                  ((__PRESCALER__) == DSPI_BAUD_RATE_32P1PCLK) || \
                                                  ((__PRESCALER__) == DSPI_BAUD_RATE_64P1PCLK) || \
                                                  ((__PRESCALER__) == DSPI_BAUD_RATE_128P1PCLK) || \
                                                  ((__PRESCALER__) == DSPI_BAUD_RATE_256P1PCLK))

/** @brief  Check if DSPI Interface Mode is valid.
  * @param  __INTERFACEMODE__ QSPI Interface Mode.
  * @retval SET (__INTERFACEMODE__ is valid) or RESET (__INTERFACEMODE__ is invalid)
  */
#define IS_DSPI_INTERFACE_MODE(__INTERFACEMODE__)   (((__INTERFACEMODE__) == DSPI_PROT_MODE_3W1L) || \
                                                    ((__INTERFACEMODE__) == DSPI_PROT_MODE_4W1L) || \
                                                    ((__INTERFACEMODE__) == DSPI_PROT_MODE_4W2L))

/** @brief  Check if DSPI Instruction Size is valid.
  * @param  __INST_SIZE__ DSPI Instruction Size.
  * @retval SET (__INST_SIZE__ is valid) or RESET (__INST_SIZE__ is invalid)
  */
#define IS_DSPI_INSTRUCTION_SIZE(__INST_SIZE__) (((__INST_SIZE__) == DSPI_INSTSIZE_08_BITS) || \
                                                 ((__INST_SIZE__) == DSPI_INSTSIZE_16_BITS) || \
                                                 ((__INST_SIZE__) == DSPI_INSTSIZE_32_BITS))

/** @} */

/** @} */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_DSPI_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup DSPI_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the DSPIx peripheral:

      (+) User must implement hal_dspi_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_dspi_init() to configure the selected device with
          the selected configuration:
        (++) baudrate
        (++) dspi mode

      (+) Call the function hal_dspi_deinit() to restore the default configuration
          of the selected DSPIx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the DSPI according to the specified parameters
 *         in the dspi_init_t and initialize the associated handle.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_init(dspi_handle_t *p_dspi);

/**
 ****************************************************************************************
 * @brief  De-initialize the DSPI peripheral.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_deinit(dspi_handle_t *p_dspi);

/**
 ****************************************************************************************
 * @brief  Initialize the DSPI MSP.
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_dspi_msp_init can be implemented in the user file.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 ****************************************************************************************
 */
void hal_dspi_msp_init(dspi_handle_t *p_dspi);

/**
 ****************************************************************************************
 * @brief  De-initialize the DSPI MSP.
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_dspi_msp_deinit can be implemented in the user file.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 ****************************************************************************************
 */
void hal_dspi_msp_deinit(dspi_handle_t *p_dspi);

/** @} */

/** @defgroup DSPI_Exported_Functions_Group2 DSPI operation functions
 *  @brief   Data transfers functions
 *
@verbatim
  ==============================================================================
                      ##### DSPI operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the DSPI
    data transfers.

    [..] The DSPI only supports master:

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts.
            or DMA, These APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated DSPI IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The hal_dspi_tx_cplt_callback() user callbacks
            will be executed respectively at the end of the transmit or Receive process.
            The hal_dspi_error_callback() user callback will be executed when a communication error is detected

    (#) APIs provided for these 2 transfer modes (Blocking mode or Non blocking mode using either Interrupt or DMA)
        exist for 1 Line (simplex) and 2 Lines (full duplex) modes.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with the specified instruction in blocking mode.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  p_cmd: Pointer to a dspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_command_transmit(dspi_handle_t *p_dspi, dspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit only instruction in blocking mode.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  p_cmd: Pointer to a dspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_command(dspi_handle_t *p_dspi, dspi_command_t *p_cmd, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in blocking mode.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_transmit(dspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with the specified instruction in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  p_cmd: Pointer to a dspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_data: Pointer to data buffer
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_command_transmit_it(dspi_handle_t *p_dspi, dspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Transmit instruction in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  p_cmd: Pointer to a dspi_command_t structure that contains the instruction and address for data transfer.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_command_it(dspi_handle_t *p_dspi, dspi_command_t *p_cmd);

/**
 ****************************************************************************************
 * @brief Transmit an amount of data in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_transmit_it(dspi_handle_t *p_dspi, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with the specified instruction in non-blocking mode with DMA .
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  p_cmd: Pointer to a dspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_data: Pointer to data buffer
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_command_transmit_dma(dspi_handle_t *p_dspi, dspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with the specified instruction in non-blocking mode with DMA SG or LLP.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  p_cmd: Pointer to a dspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  sg_llp_config: The config of source and destination's SG and LLP.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_command_transmit_dma_sg_llp(dspi_handle_t *p_dspi, dspi_command_t *p_cmd, uint8_t *p_data,
                                                                        dma_sg_llp_config_t *sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Transmit instruction in non-blocking mode with DMA.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  p_cmd: Pointer to a dspi_command_t structure that contains the instruction and address for data transfer.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_command_dma(dspi_handle_t *p_dspi, dspi_command_t *p_cmd);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode with DMA.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 0 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_transmit_dma(dspi_handle_t *p_dspi, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode with DMA SG or LLP.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 0 and 4095.
 * @param[in]  sg_llp_config: The config of source and destination's SG and LLP.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_transmit_dma_sg_llp(dspi_handle_t *p_dspi, uint8_t *p_data, uint32_t length,
                                                            dma_sg_llp_config_t *sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Abort the current transmission.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_abort(dspi_handle_t *p_dspi);

/**
 ****************************************************************************************
 * @brief  Abort the current transmission (non-blocking function)
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_abort_it(dspi_handle_t *p_dspi);

/** @} */

/** @addtogroup DSPI_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle DSPI interrupt request.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 ****************************************************************************************
 */
void hal_dspi_irq_handler(dspi_handle_t *p_dspi);

/**
 ****************************************************************************************
 * @brief  Tx Transfer completed callback.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 ****************************************************************************************
 */
void hal_dspi_tx_cplt_callback(dspi_handle_t *p_dspi);

/**
 ****************************************************************************************
 * @brief  DSPI error callback.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 ****************************************************************************************
 */
void hal_dspi_error_callback(dspi_handle_t *p_dspi);

/**
 ****************************************************************************************
 * @brief  DSPI Abort callback.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 ****************************************************************************************
 */
void hal_dspi_abort_callback(dspi_handle_t *p_dspi);

/** @} */

/** @defgroup DSPI_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   DSPI control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the DSPI.
     (+) hal_dspi_get_state() API can be helpful to check in run-time the state of the DSPI peripheral.
     (+) hal_dspi_get_error() check in run-time Errors occurring during communication.
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the DSPI handle state.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 *
 * @retval ::HAL_DSPI_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_DSPI_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_DSPI_STATE_BUSY: Peripheral in indirect mode and busy.
 * @retval ::HAL_DSPI_STATE_BUSY_INDIRECT_TX: Peripheral in indirect mode with transmission ongoing.
 * @retval ::HAL_DSPI_STATE_ABORT: Peripheral with abort request ongoing.
 * @retval ::HAL_DSPI_STATE_ERROR: Peripheral in error.
 ****************************************************************************************
 */
hal_dspi_state_t hal_dspi_get_state(dspi_handle_t *p_dspi);

/**
 ****************************************************************************************
 * @brief  Return the DSPI error code.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 *
 * @return DSPI error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_dspi_get_error(dspi_handle_t *p_dspi);

/**
 ****************************************************************************************
 * @brief  Set the DSPI internal process timeout value.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  timeout: Internal process timeout value.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
void hal_dspi_set_timeout(dspi_handle_t *p_dspi, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Set the DSPI transmission mode.
 * @param[in]  p_dspi: Pointer to a DSPI handle which contains the configuration information for the specified DSPI module.
 * @param[in]  mode: The DSPI transmission mode. This parameter can be one of the following values:
 *         @arg @ref DSPI_PROT_MODE_3W1L
 *         @arg @ref DSPI_PROT_MODE_4W1L
 *         @arg @ref DSPI_PROT_MODE_4W2L
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_config_mode(dspi_handle_t *p_dspi, uint32_t mode);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to DSPI configuration before sleep.
 * @param[in] p_dspi: Pointer to a DSPI handle which contains the configuration
 *                 information for the specified DSPI module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_suspend_reg(dspi_handle_t *p_dspi);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to DSPI configuration after sleep.
 *         This function must be used in conjunction with the hal_dspi_suspend_reg().
 * @param[in] p_dspi: Pointer to a DSPI handle which contains the configuration
 *                 information for the specified DSPI module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dspi_resume_reg(dspi_handle_t *p_dspi);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_DSPI_H__ */

/** @} */

/** @} */

/** @} */

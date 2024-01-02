/**
 ****************************************************************************************
 *
 * @file   gr55xx_hal_i2s.h
 * @author BLE Driver Team
 * @brief  Header file containing functions prototypes of I2S HAL library.
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

/** @defgroup HAL_I2S I2S
  * @brief I2S HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_I2S_H__
#define __GR55xx_HAL_I2S_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_i2s.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_I2S_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_I2S_state HAL I2S state
  * @{
  */

/**
  * @brief HAL I2S State Enumerations definition
  */
typedef enum
{
    HAL_I2S_STATE_RESET        = 0x00,    /**< Peripheral not initialized                          */
    HAL_I2S_STATE_READY        = 0x01,    /**< Peripheral initialized and ready for use            */
    HAL_I2S_STATE_BUSY         = 0x02,    /**< An internal process is ongoing                      */
    HAL_I2S_STATE_BUSY_TX      = 0x12,    /**< Data Transmission process is ongoing                */
    HAL_I2S_STATE_BUSY_RX      = 0x22,    /**< Data Reception process is ongoing                   */
    HAL_I2S_STATE_BUSY_TX_RX   = 0x32,    /**< Data Transmission and Reception process is ongoing  */
    HAL_I2S_STATE_ABORT        = 0x08,    /**< Peripheral with abort request ongoing               */
    HAL_I2S_STATE_ERROR        = 0x04     /**< Peripheral in error                                 */

} hal_i2s_state_t;

/** @} */

/** @} */

/** @addtogroup HAL_I2S_STRUCTURES Structures
  * @{
  */

/** @defgroup I2S_Configuration I2S Configuration
  * @{
  */

/**
  * @brief I2S init Structure definition
  */
typedef struct _i2s_init
{
    uint32_t data_size;             /**< Specifies the data size for I2S communication.
                                         This parameter can be a value of @ref I2S_Data_Size */

    uint32_t clock_source;          /**< Specifies the source of the I2S clock.
                                         This parameter can be a value of @ref I2S_Clock_Source */

    uint32_t ws_cycles;             /**< Specifies the Word Select Line Cycles.
                                         This parameter can be a value of @ref I2S_WS_CYCLES.*/

    uint32_t audio_freq;            /**< Specifies the frequency selected for the I2S communication.
                                         @note The communication clock is derived from the master
                                         clock. The slave clock does not need to be set. */
#if I2S_CHANNEL_NUM > 1
    uint32_t channel_active;        /**< Specifies the active channels for I2S communication.
                                         This parameter can be one or more value of @ref I2S_Channel */
#endif

} i2s_init_t;
/** @} */

/** @defgroup I2S_handle I2S handle
  * @{
  */

/**
  * @brief I2S handle Structure definition
  */
typedef struct _i2s_handle
{
    i2s_regs_t              *p_instance;        /**< I2S registers base address        */

    i2s_init_t              init;               /**< I2S communication parameters      */

    uint16_t                *p_tx_buffer;       /**< Pointer to I2S TX transfer Buffer */

    __IO uint32_t           tx_xfer_size;       /**< I2S TX Transfer size              */

    __IO uint32_t           tx_xfer_count;      /**< I2S TX Transfer Counter           */

    uint16_t                *p_rx_buffer;       /**< Pointer to I2S RX transfer Buffer */

    __IO uint32_t           rx_xfer_size;       /**< I2S RX Transfer size              */

    __IO uint32_t           rx_xfer_count;      /**< I2S RX Transfer Counter           */

    void (*write_fifo)(struct _i2s_handle *p_i2s); /**< Pointer to I2S Tx transfer FIFO write function */

    void (*read_fifo)(struct _i2s_handle *p_i2s);  /**< Pointer to I2S Rx transfer FIFO read function  */

    dma_handle_t            *p_dmatx;           /**< I2S TX DMA Handle parameters      */

    dma_handle_t            *p_dmarx;           /**< I2S RX DMA Handle parameters      */

    __IO hal_lock_t         lock;               /**< Locking object                    */

    __IO hal_i2s_state_t    state;              /**< I2S communication state           */

    __IO uint32_t           error_code;         /**< I2S Error code                    */

    uint32_t                timeout;            /**< Timeout for the I2S memory access */

    uint32_t                retention[7];     /**< I2S important register information. */
} i2s_handle_t;
/** @} */

/** @} */

/** @addtogroup HAL_I2S_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_I2S_Callback Callback
  * @{
  */

/**
  * @brief HAL_I2S Callback function definition
  */

typedef struct _i2s_callback
{
    void (*i2s_msp_init)(i2s_handle_t *p_i2s);              /**< I2S init MSP callback                  */
    void (*i2s_msp_deinit)(i2s_handle_t *p_i2s);            /**< I2S de-init MSP callback               */
    void (*i2s_error_callback)(i2s_handle_t *p_i2s);        /**< I2S error callback                     */
    void (*i2s_rx_cplt_callback)(i2s_handle_t *p_i2s);      /**< I2S rx transfer completed callback     */
    void (*i2s_tx_cplt_callback)(i2s_handle_t *p_i2s);      /**< I2S tx transfer completed callbac      */
    void (*i2s_tx_rx_cplt_callback)(i2s_handle_t *p_i2s);   /**< I2S tx/rx transfer completed callback  */
} i2s_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_I2S_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup I2S_Exported_Constants I2S Exported Constants
  * @{
  */

/** @defgroup I2S_Direction I2S Direction
  * @{
  */
#define I2S_DIRECTION_FULL_DUPLEX       LL_I2S_FULL_DUPLEX      /**< Full Duplex: Transmit & Receive    */
#define I2S_DIRECTION_SIMPLEX_TX        LL_I2S_SIMPLEX_TX       /**< Simplex TX: Transmit only          */
#define I2S_DIRECTION_SIMPLEX_RX        LL_I2S_SIMPLEX_RX       /**< Simplex RX: Receive only           */
/** @} */

/** @defgroup I2S_Error_Code I2S Error Code
  * @{
  */
#define HAL_I2S_ERROR_NONE              ((uint32_t)0x00000000)  /**< No error                   */
#define HAL_I2S_ERROR_TIMEOUT           ((uint32_t)0x00000001)  /**< Timeout error              */
#define HAL_I2S_ERROR_TRANSFER          ((uint32_t)0x00000002)  /**< Transfer error             */
#define HAL_I2S_ERROR_DMA               ((uint32_t)0x00000004)  /**< DMA transfer error         */
#define HAL_I2S_ERROR_INVALID_PARAM     ((uint32_t)0x00000008)  /**< Invalid parameters error   */
#define HAL_I2S_ERROR_TX_OVERFLOW       ((uint32_t)0x00000010)  /**< Transmit overflow error    */
#define HAL_I2S_ERROR_RX_OVERFLOW       ((uint32_t)0x00000020)  /**< Receive overflow error     */
/** @} */

/** @defgroup I2S_Data_Size I2S Data Size
  * @{
  */
#define I2S_DATASIZE_12BIT              LL_I2S_DATASIZE_12BIT   /**< 12-bit serial data transfer    */
#define I2S_DATASIZE_16BIT              LL_I2S_DATASIZE_16BIT   /**< 16-bit serial data transfer    */
#define I2S_DATASIZE_20BIT              LL_I2S_DATASIZE_20BIT   /**< 20-bit serial data transfer    */
#define I2S_DATASIZE_24BIT              LL_I2S_DATASIZE_24BIT   /**< 24-bit serial data transfer    */
#define I2S_DATASIZE_32BIT              LL_I2S_DATASIZE_32BIT   /**< 32-bit serial data transfer    */
/** @} */

/** @defgroup I2S_Clock_Source I2S Clock Source
  * @{
  */
#define I2S_CLOCK_SRC_96M               LL_I2S_CLOCK_SRC_96M    /**< I2S clock source select: 96M  */
#define I2S_CLOCK_SRC_64M               LL_I2S_CLOCK_SRC_64M    /**< I2S clock source select: 64M  */
#define I2S_CLOCK_SRC_32M               LL_I2S_CLOCK_SRC_32M    /**< I2S clock source select: 32M  */
/** @} */

/** @defgroup I2S_WS_CYCLES I2S Word Select Line Cycles
  * @{
  */
#define I2S_WS_CYCLES_16                LL_I2S_WS_CYCLES_16    /**< 16 SCLK cycles in word select line. */
#define I2S_WS_CYCLES_24                LL_I2S_WS_CYCLES_24    /**< 24 SCLK cycles in word select line. */
#define I2S_WS_CYCLES_32                LL_I2S_WS_CYCLES_32    /**< 32 SCLK cycles in word select line. */
/** @} */

/** @defgroup I2S_FIFO_LEVEL_MAX I2S FIFO Level Max
  * @{
  */
#define I2S_TX_FIFO_LEVEL_MAX           16                      /**< I2S TX FIFO Level Max Value    */
#define I2S_RX_FIFO_LEVEL_MAX           16                      /**< I2S RX FIFO Level Max Value    */
/** @} */

/** @defgroup I2S_Flags_definition I2S Flags Definition
  * @{
  */
#define I2S_FLAG_TXFO                   LL_I2S_STATUS_TXFO      /**< TX FIFO write overflow flag    */
#define I2S_FLAG_TXFE                   LL_I2S_STATUS_TXFE      /**< TX FIFO empty trigger flag     */
#define I2S_FLAG_RXFO                   LL_I2S_STATUS_RXFO      /**< RX FIFO receive overflow flag  */
#define I2S_FLAG_RXDA                   LL_I2S_STATUS_RXDA      /**< RX FIFO data available flag    */
/** @} */

/** @defgroup I2S_Interrupt_definition I2S Interrupt Definition
  * @{
  */
#define I2S_IT_TXFO                     LL_I2S_INT_TXFO         /**< TX FIFO write overflow interrupt   */
#define I2S_IT_TXFE                     LL_I2S_INT_TXFE         /**< TX FIFO empty trigger interrupt    */
#define I2S_IT_RXFO                     LL_I2S_INT_RXFO         /**< RX FIFO receive overflow interrupt */
#define I2S_IT_RXDA                     LL_I2S_INT_RXDA         /**< RX FIFO data available interrupt   */
/** @} */

/** @defgroup I2S_Timeout_definition I2S Timeout_definition
  * @{
  */
#define HAL_I2S_TIMEOUT_DEFAULT_VALUE   ((uint32_t)5000)        /**< 5s */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup I2S_Exported_Macros I2S Exported Macros
  * @{
  */

/** @brief  Reset I2S handle states.
  * @param  __HANDLE__ I2S handle.
  * @retval None
  */
#define __HAL_I2S_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_I2S_STATE_RESET)

/** @brief  Enable the specified I2S peripheral.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->EN, I2S_EN_I2S_EN)
/** @brief  Disable the specified I2S peripheral.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */

#define __HAL_I2S_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->EN, I2S_EN_I2S_EN)

/** @brief  Enable the specified I2S clock.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */

#if !defined (GR551xx)
#define __HAL_I2S_ENABLE_CLOCK(__HANDLE__)                     SET_BITS((__HANDLE__)->p_instance->CLK_EN, I2S_CLK_EN_CLK_EN)
#else
#define __HAL_I2S_ENABLE_CLOCK(__HANDLE__)                     SET_BITS((__HANDLE__)->p_instance->CLKEN, I2S_CLKEN_EN)
#endif

/** @brief  Disable the specified I2S clock.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */

#if !defined (GR551xx)
#define __HAL_I2S_DISABLE_CLOCK(__HANDLE__)                    CLEAR_BITS((__HANDLE__)->p_instance->CLK_EN, I2S_CLK_EN_CLK_EN)
#else
#define __HAL_I2S_DISABLE_CLOCK(__HANDLE__)                    CLEAR_BITS((__HANDLE__)->p_instance->CLKEN, I2S_CLKEN_EN)
#endif
/** @brief  Enable the specified I2S transmitter block.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_ENABLE_TX_BLOCK(__HANDLE__)                  ll_i2s_enable_txblock((__HANDLE__)->p_instance)

/** @brief  Disable the specified I2S transmitter block.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_DISABLE_TX_BLOCK(__HANDLE__)                 ll_i2s_disable_txblock((__HANDLE__)->p_instance)

/** @brief  Enable the specified I2S receiver block.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_ENABLE_RX_BLOCK(__HANDLE__)                  ll_i2s_enable_rxblock((__HANDLE__)->p_instance)

/** @brief  Disable the specified I2S receiver block.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_DISABLE_RX_BLOCK(__HANDLE__)                 ll_i2s_disable_rxblock((__HANDLE__)->p_instance)

/** @brief  Enable the specified I2S transmitter channel.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */

#define __HAL_I2S_ENABLE_TX_CHANNEL(__HANDLE__)        ll_i2s_enable_tx((__HANDLE__)->p_instance)

/** @brief  Disable the specified I2S transmitter channel.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */

#define __HAL_I2S_DISABLE_TX_CHANNEL(__HANDLE__)       ll_i2s_disable_tx((__HANDLE__)->p_instance)

/** @brief  Enable the specified I2S receiver channel.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */

#define __HAL_I2S_ENABLE_RX_CHANNEL(__HANDLE__)        ll_i2s_enable_rx((__HANDLE__)->p_instance)

/** @brief  Disable the specified I2S receiver channel.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */

#define __HAL_I2S_DISABLE_RX_CHANNEL(__HANDLE__)       ll_i2s_disable_rx((__HANDLE__)->p_instance)

/** @brief  Flush the I2S transmitter FIFO.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_FLUSH_TX_FIFO(__HANDLE__)                    ll_i2s_clr_txfifo_all((__HANDLE__)->p_instance)

/** @brief  Flush the I2S receiver FIFO.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_FLUSH_RX_FIFO(__HANDLE__)                    ll_i2s_clr_rxfifo_all((__HANDLE__)->p_instance)

/** @brief  Enable the I2S DMA Request.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_ENABLE_DMA(__HANDLE__)                       ll_i2s_enable_dma(__HANDLE__->p_instance)

/** @brief  Disable the I2S DMA Request.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_DISABLE_DMA(__HANDLE__)                      ll_i2s_disable_dma(__HANDLE__->p_instance)

/** @brief  Reset the I2S TX DMA request to the lowest enabled channel.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */

#define __HAL_I2S_RESET_TXDMA(__HANDLE__)                      WRITE_REG((__HANDLE__)->p_instance->RST_TX_DMA, I2S_RST_TX_DMA_RST_TX_DMA)

/** @brief  Enable the I2S DMA mode.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_ENABLE_DMA_MODE(__HANDLE__)                       ll_i2s_enable_dma_mode(__HANDLE__->p_instance)

/** @brief  Disable the I2S DMA mode.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_DISABLE_DMA_MODE(__HANDLE__)                      ll_i2s_disable_dma_mode(__HANDLE__->p_instance)

/** @brief  Reset the I2S RX DMA request to the lowest enabled channel.
  * @param  __HANDLE__ Specifies the I2S Handle.
  * @retval None
  */

#define __HAL_I2S_RESET_RXDMA(__HANDLE__)                      WRITE_REG((__HANDLE__)->p_instance->RST_RX_DMA, I2S_RST_RX_DMA_RST_RX_DMA)

/** @brief  Enable the specified I2S interrupts.
  * @param  __HANDLE__      Specifies the I2S Handle.
  * @param  __INTERRUPT__   Specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg @ref I2S_IT_TXFO TX FIFO write overflow interrupt
  *            @arg @ref I2S_IT_TXFE TX FIFO empty trigger interrupt
  *            @arg @ref I2S_IT_RXFO RX FIFO receive overflow interrupt
  *            @arg @ref I2S_IT_RXDA RX FIFO data available interrupt
  * @retval None
  */

#define __HAL_I2S_ENABLE_IT(__HANDLE__, __INTERRUPT__)         CLEAR_BITS((__HANDLE__)->p_instance->INT_MASK, (__INTERRUPT__))

/** @brief  Disable the specified I2S interrupts.
  * @param  __HANDLE__      Specifies the I2S handle.
  * @param  __INTERRUPT__   Specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg @ref I2S_IT_TXFO TX FIFO write overflow interrupt
  *            @arg @ref I2S_IT_TXFE TX FIFO empty trigger interrupt
  *            @arg @ref I2S_IT_RXFO RX FIFO receive overflow interrupt
  *            @arg @ref I2S_IT_RXDA RX FIFO data available interrupt
  * @retval None
  */

#define __HAL_I2S_DISABLE_IT(__HANDLE__, __INTERRUPT__)        SET_BITS((__HANDLE__)->p_instance->INT_MASK, (__INTERRUPT__))

/** @brief  Check whether the specified I2S flag is set or not.
  * @param  __HANDLE__  Specifies the I2S Handle.
  * @param  __FLAG__    Specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref I2S_FLAG_TXFO TX FIFO write overflow flag
  *            @arg @ref I2S_FLAG_TXFE TX FIFO empty trigger flag
  *            @arg @ref I2S_FLAG_RXFO RX FIFO receive overflow flag
  *            @arg @ref I2S_FLAG_RXDA RX FIFO data available flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */

#define __HAL_I2S_GET_FLAG(__HANDLE__, __FLAG__)               ((READ_BITS((__HANDLE__)->p_instance->INT_STAT, (__FLAG__)) != 0) ? SET : RESET)

/** @brief  Clear the specified I2S flag.
  * @param  __HANDLE__  Specifies the I2S Handle.
  * @param  __FLAG__    Specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref I2S_FLAG_TXFO TX FIFO write overflow flag
  *            @arg @ref I2S_FLAG_RXFO RX FIFO receive overflow flag
  * @retval None
  */

#define __HAL_I2S_CLEAR_FLAG(__HANDLE__, __FLAG__)             do {                                 \
                                                                   if ((__FLAG__) & I2S_FLAG_RXFO)  \
                                                                   {                                \
                                                                       READ_BITS((__HANDLE__)->p_instance->RX_OVER, I2S_RX_OVER_RX_CLR_FDO);\
                                                                   }                                \
                                                                   if ((__FLAG__) & I2S_FLAG_TXFO)  \
                                                                   {                                \
                                                                       READ_BITS((__HANDLE__)->p_instance->TX_OVER, I2S_TX_OVER_TX_CLR_FDO);\
                                                                   }                                \
                                                               } while(0);

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup I2S_Private_Macro I2S Private Macros
  * @{
  */

/** @brief  Check if I2S Direction Mode is valid.
  * @param  __MODE__    I2S Direction Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_I2S_DIRECTION(__MODE__)              (((__MODE__) == I2S_DIRECTION_FULL_DUPLEX) || \
                                                 ((__MODE__) == I2S_DIRECTION_SIMPLEX_TX)  || \
                                                 ((__MODE__) == I2S_DIRECTION_SIMPLEX_RX))

/** @brief  Check if I2S Data Size is valid.
  * @param  __DATASIZE__    I2S Data Size.
  * @retval SET (__DATASIZE__ is valid) or RESET (__DATASIZE__ is invalid)
  */
#define IS_I2S_DATASIZE(__DATASIZE__)           (((__DATASIZE__) == I2S_DATASIZE_12BIT) || \
                                                 ((__DATASIZE__) == I2S_DATASIZE_16BIT) || \
                                                 ((__DATASIZE__) == I2S_DATASIZE_20BIT) || \
                                                 ((__DATASIZE__) == I2S_DATASIZE_24BIT) || \
                                                 ((__DATASIZE__) == I2S_DATASIZE_32BIT))

/** @brief  Check if I2S Clock Polarity is valid.
  * @param  __CPOL__    I2S Clock Polarity.
  * @retval SET (__CPOL__ is valid) or RESET (__CPOL__ is invalid)
  */
#define IS_I2S_CPOL(__CPOL__)                   (((__CPOL__) == I2S_POLARITY_LOW) || \
                                                 ((__CPOL__) == I2S_POLARITY_HIGH))

/** @brief  Check if I2S Audio Frequency is valid.
  * @param  __FREQUENCY__   I2S Audio Frequency.
  * @retval SET (__FREQUENCY__ is valid) or RESET (__FREQUENCY__ is invalid)
  */
#define IS_I2S_AUDIO_FREQUENCY(__FREQUENCY__)   (((__FREQUENCY__) > 0) && ((__FREQUENCY__) <= 1500000))

/** @brief  Check if I2S FIFO Threshold is valid.
  * @param  __THR__ I2S FIFO Threshold.
  * @retval SET (__THR__ is valid) or RESET (__THR__ is invalid)
  */
#define IS_I2S_FIFO_THRESHOLD(__THR__)          (((__THR__) >= 0) && ((__THR__) <= I2S_TX_FIFO_LEVEL_MAX))

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_I2S_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup I2S_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initializations functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the I2Sx peripheral:

      (+) User must implement hal_i2s_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_i2s_init() to configure the selected device with
          the selected configuration:
        (++) Data Size
        (++) Clock Polarity
        (++) Audio Frequency

      (+) Call the function hal_i2s_deinit() to restore the default configuration
          of the selected I2Sx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the I2S according to the specified parameters
 *         in the i2s_init_t and initialize the associated handle.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_init(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  De-initialize the I2S peripheral.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_deinit(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  Initialize the I2S MSP.
 * @note   This function should not be modified. When the callback is needed,
            the hal_i2s_msp_deinit can be implemented in the user file.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 ****************************************************************************************
 */
void hal_i2s_msp_init(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  De-initialize the I2S MSP.
 * @note   This function should not be modified. When the callback is needed,
            the hal_i2s_msp_deinit can be implemented in the user file.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 ****************************************************************************************
 */
void hal_i2s_msp_deinit(i2s_handle_t *p_i2s);

/** @} */

/** @defgroup I2S_Exported_Functions_Group2 IO operation functions
 *  @brief   Data transfers functions
 *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the I2S
    data transfers.

    [..] The I2S supports master and slave mode:

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts
            or DMA, These APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated I2S IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The hal_i2s_tx_cplt_callback(), hal_i2s_rx_cplt_callback() and hal_i2s_tx_rx_cplt_callback() user callbacks
            will be executed respectively at the end of the transmit or Receive process
            The hal_i2s_error_callback() user callback will be executed when a communication error is detected.

    (#) APIs provided for these 2 transfer modes (Blocking mode or Non blocking mode using either Interrupt or DMA)
        exist for 1-Line (simplex) and 2-Line (full duplex) modes.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in blocking mode.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in halfword, data of a channel.
 *             For example, when 32 bytes of data need to be sent in each of the left and right channels, length = 16.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_transmit(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in blocking mode.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be received in halfword, data of a channel.
 *             For example, when 32 bytes of data need to be sent in each of the left and right channels, length = 16.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_receive(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit and Receive an amount of data in blocking mode.
 * @param[in]  p_i2s: Pointer to a I2S handle which contains the configuration information for the specified I2S module.
 * @param[in]  p_tx_data: Pointer to transmission data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  length: Amount of data to be sent and received in bytes
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_transmit_receive(i2s_handle_t *p_i2s, uint16_t *p_tx_data, uint16_t *p_rx_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in halfword, data of a channel.
 *             For example, when 32 bytes of data need to be sent in each of the left and right channels, length = 16.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_transmit_it(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in non-blocking mode with Interrupt.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in halfword, data of a channel.
 *             For example, when 32 bytes of data need to be sent in each of the left and right channels, length = 16.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_receive_it(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
 * @param[in]  p_i2s: Pointer to a I2S handle which contains the configuration information for the specified SPI module.
 * @param[in]  p_tx_data: Pointer to transmission data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  length: Amount of data to be sent and received in bytes
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_transmit_receive_it(i2s_handle_t *p_i2s, uint16_t *p_tx_data, uint16_t *p_rx_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode with DMA.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in halfword, data of a channel, ranging between 1 and 4095.
 *             For example, when 32 bytes of data need to be sent in each of the left and right channels, length = 16.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_transmit_dma(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in non-blocking mode with DMA.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in halfword, data of a channel, ranging between 1 and 4095.
 *             For example, when 32 bytes of data need to be sent in each of the left and right channels, length = 16.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_receive_dma(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
 * @param[in]  p_i2s: Pointer to a I2S handle which contains the configuration information for the specified I2S module.
 * @param[in]  p_tx_data: Pointer to transmission data buffer
 * @param[out] p_rx_data: Pointer to reception data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 1 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_transmit_receive_dma(i2s_handle_t *p_i2s, uint16_t *p_tx_data, uint16_t *p_rx_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode with DMA.
 * @param[in]  p_i2s:           Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @param[in]  p_data:          Pointer to data buffer
 * @param[in]  length:          Amount of data to be sent in halfword, data of a channel, ranging between 1 and 4095.
 *                              For example, when 32 bytes of data need to be sent in each of the left and right channels, length = 16.
 * @param[in]  p_sg_llp_config: Pointer to the config of source and destination's SG and LLP.
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_transmit_dma_sg_llp(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length, dma_sg_llp_config_t *p_sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in non-blocking mode with DMA.
 * @param[in]  p_i2s:           Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @param[out] p_data:          Pointer to data buffer
 * @param[in]  length:          Amount of data to be sent in halfword, data of a channel, ranging between 1 and 4095.
 *                              For example, when 32 bytes of data need to be sent in each of the left and right channels, length = 16.
 * @param[in]  p_sg_llp_config: Pointer to the config of source and destination's SG and LLP.
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_receive_dma_sg_llp(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length, dma_sg_llp_config_t *p_sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
 * @param[in]  p_i2s: Pointer to a I2S handle which contains the configuration information for the specified I2S module.
 * @param[in]  p_tx_data:       Pointer to transmission data buffer
 * @param[out] p_rx_data:       Pointer to reception data buffer
 * @param[in]  length:          Amount of data to be sent in bytes,  ranging between 1 and 4095.
 * @param[in]  p_sg_llp_config: Pointer to the config of source and destination's SG and LLP.
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_transmit_receive_dma_sg_llp(i2s_handle_t        *p_i2s,
                                                 uint16_t            *p_tx_data,
                                                 uint16_t            *p_rx_data,
                                                 uint32_t             length,
                                                 dma_sg_llp_config_t *p_sg_llp_config);


/**
 ****************************************************************************************
 * @brief  Start the I2S master clock.
 * @note   In case of SLAVE mode, this function will not take effect.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_start_clock(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  Stop the I2S master clock.
 * @note   In case of SLAVE mode, this function will not take effect.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_stop_clock(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  Abort ongoing transfer (blocking mode).
 * @param[in]  p_i2s: I2S handle.
 * @note   This procedure could be used for aborting any ongoing transfer (TX and RX),
 *         started in Interrupt or DMA mode.
 *         This procedure performs following operations :
 *           - Disable I2S Interrupts (depending of transfer direction)
 *           - Disable the DMA transfer in the peripheral register (if enabled)
 *           - Abort DMA transfer by calling hal_dma_abort (in case of transfer in DMA mode)
 *           - Set handle State to READY
 * @note   This procedure is executed in blocking mode: When exiting function, Abort is considered as completed.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_abort(i2s_handle_t *p_i2s);

/** @} */

/** @addtogroup I2S_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle I2S interrupt request.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 ****************************************************************************************
 */
void hal_i2s_irq_handler(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  TX Transfer completed callback.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 ****************************************************************************************
 */
void hal_i2s_tx_cplt_callback(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  RX Transfer completed callback.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 ****************************************************************************************
 */
void hal_i2s_rx_cplt_callback(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  TX/RX Transfer completed callback.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 ****************************************************************************************
 */
void hal_i2s_tx_rx_cplt_callback(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  I2S error callback.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 ****************************************************************************************
 */
void hal_i2s_error_callback(i2s_handle_t *p_i2s);

/** @} */

/** @defgroup I2S_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   I2S control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the I2S.
     (+) hal_i2s_get_state() API can be helpful to check in run-time the state of the I2S peripheral
     (+) hal_i2s_get_error() check in run-time Errors occurring during communication
     (+) hal_i2s_set_timeout() set the timeout during internal process
     (+) hal_i2s_set_tx_fifo_threshold() set the TX FIFO Threshold
     (+) hal_i2s_set_rx_fifo_threshold() set the RX FIFO Threshold
     (+) hal_i2s_get_tx_fifo_threshold() get the TX FIFO Threshold
     (+) hal_i2s_get_rx_fifo_threshold() get the RX FIFO Threshold
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the I2S handle state.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @retval ::HAL_I2S_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_I2S_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_I2S_STATE_BUSY: An internal process is ongoing.
 * @retval ::HAL_I2S_STATE_BUSY_TX: Data Transmii2son process is ongoing.
 * @retval ::HAL_I2S_STATE_BUSY_RX: Data Reception process is ongoing.
 * @retval ::HAL_I2S_STATE_ABORT: Peripheral with abort request ongoing.
 * @retval ::HAL_I2S_STATE_ERROR: Peripheral in error.
 ****************************************************************************************
 */
hal_i2s_state_t hal_i2s_get_state(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  Return the I2S error code.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @return I2S error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_i2s_get_error(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  Set the TX FIFO threshold.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @param[in]  threshold: TX FIFO threshold value ranging bwtween 0x0U ~ 0x7U.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_set_tx_fifo_threshold(i2s_handle_t *p_i2s, uint32_t threshold);

/**
 ****************************************************************************************
 * @brief  Set the RX FIFO threshold.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @param[in]  threshold: RX FIFO threshold value ranging bwtween 0x0U ~ 0x7U.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_set_rx_fifo_threshold(i2s_handle_t *p_i2s, uint32_t threshold);

/**
 ****************************************************************************************
 * @brief  Get the TX FIFO threshold.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @return TX FIFO threshold
 ****************************************************************************************
 */
uint32_t hal_i2s_get_tx_fifo_threshold(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  Get the RX FIFO threshold.
 * @param[in]  p_i2s: Pointer to an I2S handle which contains the configuration information for the specified I2S module.
 * @return RX FIFO threshold
 ****************************************************************************************
 */
uint32_t hal_i2s_get_rx_fifo_threshold(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to I2S configuration before sleep.
 * @param[in] p_i2s: Pointer to a I2S handle which contains the configuration
 *                 information for the specified I2S module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_suspend_reg(i2s_handle_t *p_i2s);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to I2S configuration after sleep.
 *         This function must be used in conjunction with the hal_i2s_suspend_reg().
 * @param[in] p_i2s: Pointer to a I2S handle which contains the configuration
 *                 information for the specified I2S module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2s_resume_reg(i2s_handle_t *p_i2s);


/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_I2S_H__ */

/** @} */

/** @} */

/** @} */

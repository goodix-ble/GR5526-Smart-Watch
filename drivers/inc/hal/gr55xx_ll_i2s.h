/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_i2s.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of I2S LL library.
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

/** @defgroup LL_I2S I2S
  * @brief I2S LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_I2S_H__
#define __GR55xx_LL_I2S_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (I2S_M) || defined (I2S_S)

/** @defgroup LL_I2S_DRIVER_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup I2S_LL_ES_INIT I2S Exported init structure
  * @{
  */

/**
  * @brief LL I2S init structures definition
  */
typedef struct _ll_i2s_init_t
{
    uint32_t rxdata_size;           /**< Specifies the I2S receive data size.
                                         This parameter can be a value of @ref I2S_LL_EC_DATASIZE.

                                         This feature can be modified afterwards using unitary function @ref ll_i2s_set_rxsize().*/

    uint32_t txdata_size;           /**< Specifies the I2S transmit data size.
                                         This parameter can be a value of @ref I2S_LL_EC_DATASIZE.

                                         This feature can be modified afterwards using unitary function @ref ll_i2s_set_txsize().*/

    uint32_t rx_threshold;          /**< Specifies the I2S receive FIFO threshold.
                                         This parameter can be a value of @ref I2S_LL_EC_FIFO_THRESHOLD.

                                         This feature can be modified afterwards using unitary function @ref ll_i2s_set_rx_fifo_threshold().*/

    uint32_t tx_threshold;          /**< Specifies the I2S transmit FIFO threshold.
                                         This parameter can be a value of @ref I2S_LL_EC_FIFO_THRESHOLD.

                                         This feature can be modified afterwards using unitary function @ref ll_i2s_set_tx_fifo_threshold().*/

    uint32_t clock_source;          /**< Specifies the source of the I2S clock.
                                         This parameter can be a value of @ref I2S_LL_EC_CLOCK_SOURCE.

                                         This feature can be modified afterwards using unitary function @ref ll_i2s_set_clock_src().*/

    uint32_t ws_cycles;             /**< Specifies the I2S Word Select Line Cycles.
                                         This parameter can be a value of @ref I2S_LL_EC_WS_CYCLES.
                                         This feature can be modified afterwards using unitary function @ref ll_i2s_set_wss().*/

    uint32_t audio_freq;            /**< Specifies the frequency selected for the I2S communication.

                                         This feature can be modified afterwards using unitary function @ref ll_i2s_set_clock_div().*/

} ll_i2s_init_t;

/** @} */

/** @} */

/**
  * @defgroup  I2S_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup I2S_LL_Exported_Constants I2S Exported Constants
  * @{
  */

/** @defgroup I2S_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags definitions which can be used with LL_I2S_ReadReg function
  * @{
  */
#define LL_I2S_STATUS_TXFO                  I2S_INT_STAT_TX_FIFO_OVER            /**< TX FIFO write overflow flag                 */
#define LL_I2S_STATUS_TXFE                  I2S_INT_STAT_TX_FIFO_EMPTY           /**< TX FIFO threshold level is not reached flag */
#define LL_I2S_STATUS_RXFO                  I2S_INT_STAT_RX_FIFO_OVER            /**< RX FIFO receive overflow flag               */
#define LL_I2S_STATUS_RXDA                  I2S_INT_STAT_RX_DATA_AVL             /**< RX FIFO threshold level is reached flag     */
/** @} */

/** @defgroup I2S_LL_EC_INTERRUPT Interrupt Defines
  * @brief    Interrupt definitions which can be used with LL_I2S_ReadReg and  LL_I2S_WriteReg functions
  * @{
  */
#define LL_I2S_INT_TXFO                     I2S_INT_MASK_TX_FOM            /**< TX FIFO write overflow interrupt                 */
#define LL_I2S_INT_TXFE                     I2S_INT_MASK_TX_FEM            /**< TX FIFO threshold level is not reached interrupt */
#define LL_I2S_INT_RXFO                     I2S_INT_MASK_RX_FOM            /**< RX FIFO receive overflow interrupt               */
#define LL_I2S_INT_RXDA                     I2S_INT_MASK_RX_DAM            /**< RX FIFO threshold level is reached interrupt     */
/** @} */

/** @defgroup I2S_LL_EC_CLOCK_SOURCE I2S Clock Source
  * @{
  */
#define LL_I2S_CLOCK_SRC_96M                (0x00000000UL)              /**< I2S clock source select: 96M            */
#define LL_I2S_CLOCK_SRC_64M                (1UL << 18)                 /**< I2S clock source select: 64M            */
#define LL_I2S_CLOCK_SRC_32M                (2UL << 18)                 /**< I2S clock source select: 32M            */
/** @} */

/** @defgroup I2S_LL_EC_DATASIZE Transfer Data width
  * @{
  */
#define LL_I2S_DATASIZE_IGNORE              (0x00000000UL)                     /**< Data size for I2S transfer: 32 bits */
#define LL_I2S_DATASIZE_12BIT               (1UL << I2S_RX_CFG_WORD_LEN_POS)    /**< Data size for I2S transfer: 12 bits */
#define LL_I2S_DATASIZE_16BIT               (2UL << I2S_RX_CFG_WORD_LEN_POS)    /**< Data size for I2S transfer: 16 bits */
#define LL_I2S_DATASIZE_20BIT               (3UL << I2S_RX_CFG_WORD_LEN_POS)    /**< Data size for I2S transfer: 20 bits */
#define LL_I2S_DATASIZE_24BIT               (4UL << I2S_RX_CFG_WORD_LEN_POS)    /**< Data size for I2S transfer: 24 bits */
#define LL_I2S_DATASIZE_32BIT               (5UL << I2S_RX_CFG_WORD_LEN_POS)    /**< Data size for I2S transfer: 32 bits */
/** @} */

/** @defgroup I2S_LL_EC_TRANSFER_MODE Transfer Mode
  * @{
  */
#define LL_I2S_SIMPLEX_TX                   (1UL)                           /**< Simplex TX mode.     */
#define LL_I2S_SIMPLEX_RX                   (2UL)                           /**< Simplex RX mode.     */
#define LL_I2S_FULL_DUPLEX                  (3UL)                           /**< Full-Duplex mode.    */
/** @} */

/** @defgroup I2S_LL_EC_FIFO_THRESHOLD FIFO Threshold
  * @{
  */
#define LL_I2S_THRESHOLD_1FIFO              (0x00000000UL)                              /**< Trigger level for FIFO: 1 depth.     */
#define LL_I2S_THRESHOLD_2FIFO              (1UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)      /**< Trigger level for FIFO: 2 depth.     */
#define LL_I2S_THRESHOLD_3FIFO              (2UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)      /**< Trigger level for FIFO: 3 depth.     */
#define LL_I2S_THRESHOLD_4FIFO              (3UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)      /**< Trigger level for FIFO: 4 depth.     */
#define LL_I2S_THRESHOLD_5FIFO              (4UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)      /**< Trigger level for FIFO: 5 depth.     */
#define LL_I2S_THRESHOLD_6FIFO              (5UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)      /**< Trigger level for FIFO: 6 depth.     */
#define LL_I2S_THRESHOLD_7FIFO              (6UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)      /**< Trigger level for FIFO: 7 depth.     */
#define LL_I2S_THRESHOLD_8FIFO              (7UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)      /**< Trigger level for FIFO: 8 depth.     */
#define LL_I2S_THRESHOLD_9FIFO              (8UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)      /**< Trigger level for FIFO: 9 depth.     */
#define LL_I2S_THRESHOLD_10FIFO             (9UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)      /**< Trigger level for FIFO: 10 depth.    */
#define LL_I2S_THRESHOLD_11FIFO             (10UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)     /**< Trigger level for FIFO: 11 depth.    */
#define LL_I2S_THRESHOLD_12FIFO             (11UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)     /**< Trigger level for FIFO: 12 depth.    */
#define LL_I2S_THRESHOLD_13FIFO             (12UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)     /**< Trigger level for FIFO: 13 depth.    */
#define LL_I2S_THRESHOLD_14FIFO             (13UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)     /**< Trigger level for FIFO: 14 depth.    */
#define LL_I2S_THRESHOLD_15FIFO             (14UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)     /**< Trigger level for FIFO: 15 depth.    */
#define LL_I2S_THRESHOLD_16FIFO             (15UL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)     /**< Trigger level for FIFO: 16 depth.    */
/** @} */

/** @defgroup I2S_LL_EC_WS_CYCLES Word Select Line Cycles
  * @{
  */
#define LL_I2S_WS_CYCLES_16                 (0x00000000UL)                         /**< 16 SCLK cycles in word select line. */
#define LL_I2S_WS_CYCLES_24                 (0x1UL << I2S_SCLK_CFG_WS_SCLK_POS)    /**< 24 SCLK cycles in word select line. */
#define LL_I2S_WS_CYCLES_32                 (0x2UL << I2S_SCLK_CFG_WS_SCLK_POS)    /**< 32 SCLK cycles in word select line. */
/** @} */

/** @defgroup I2S_LL_EC_SCLK_GATE SCLK Gate
  * @{
  */
#define LL_I2S_SCLKG_NONE                   (0x00000000UL)                        /**< Clock gating is disabled.    */
#define LL_I2S_SCLKG_CYCLES_12              (0x1UL << I2S_SCLK_CFG_SCLK_GAT_POS)  /**< Gating after 12 sclk cycles. */
#define LL_I2S_SCLKG_CYCLES_16              (0x2UL << I2S_SCLK_CFG_SCLK_GAT_POS)  /**< Gating after 16 sclk cycles. */
#define LL_I2S_SCLKG_CYCLES_20              (0x3UL << I2S_SCLK_CFG_SCLK_GAT_POS)  /**< Gating after 20 sclk cycles. */
#define LL_I2S_SCLKG_CYCLES_24              (0x4UL << I2S_SCLK_CFG_SCLK_GAT_POS)  /**< Gating after 24 sclk cycles. */
/** @} */

/** @defgroup I2S_LL_EC_RESOLUTION RX/TX resolution of one channel
  * @{
  */
#define LL_I2S_RESOLUTION_12BIT             (0UL)                       /**< 12 bits resolution. */
#define LL_I2S_RESOLUTION_16BIT             (1UL)                       /**< 16 bits resolution. */
#define LL_I2S_RESOLUTION_20BIT             (2UL)                       /**< 20 bits resolution. */
#define LL_I2S_RESOLUTION_24BIT             (3UL)                       /**< 24 bits resolution. */
#define LL_I2S_RESOLUTION_32BIT             (4UL)                       /**< 32 bits resolution. */
/** @} */

/** @defgroup I2S_LL_EC_CHANNELS the number of RX/TX channels
  * @{
  */
#define LL_I2S_CHANNEL_NUM_1                (0UL)                       /**< 1 channel.  */
#define LL_I2S_CHANNEL_NUM_2                (1UL)                       /**< 2 channels. */
#define LL_I2S_CHANNEL_NUM_3                (2UL)                       /**< 3 channels. */
#define LL_I2S_CHANNEL_NUM_4                (3UL)                       /**< 4 channels. */
/** @} */

/** @defgroup I2S_LL_EC_FIFO_DEPTH RX/TX FIFO depth
  * @{
  */
#define LL_I2S_FIFO_DEPTH_2                 (0UL)                       /**< FIFO depth is 2 . */
#define LL_I2S_FIFO_DEPTH_4                 (1UL)                       /**< FIFO depth is 4 . */
#define LL_I2S_FIFO_DEPTH_8                 (2UL)                       /**< FIFO depth is 8 . */
#define LL_I2S_FIFO_DEPTH_16                (3UL)                       /**< FIFO depth is 16. */
/** @} */

/** @defgroup I2S_LL_EC_APB_WIDTH APB data width
  * @{
  */
#define LL_I2S_APB_WIDTH_8BIT               (0UL)                       /**< 8  bits APB data width. */
#define LL_I2S_APB_WIDTH_16BIT              (1UL)                       /**< 16 bits APB data width. */
#define LL_I2S_APB_WIDTH_32BIT              (2UL)                       /**< 32 bits APB data width. */
/** @} */

/** @defgroup I2S_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */

/**
  * @brief LL I2S InitStrcut default configuartion
  */
#define LL_I2S_DEFAULT_CONFIG                          \
{                                                      \
    .rxdata_size         = LL_I2S_DATASIZE_16BIT,      \
    .txdata_size         = LL_I2S_DATASIZE_16BIT,      \
    .rx_threshold        = LL_I2S_THRESHOLD_1FIFO,     \
    .tx_threshold        = LL_I2S_THRESHOLD_9FIFO,     \
    .clock_source        = LL_I2S_CLOCK_SRC_32M,       \
    .audio_freq          = 48000                       \
}

/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup I2S_LL_Exported_Macros I2S Exported Macros
  * @{
  */

/** @defgroup I2S_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in I2S register
  * @param  __instance__ I2S instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_I2S_WriteReg(__instance__, __REG__, __VALUE__)   WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in I2S register
  * @param  __instance__ I2S instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_I2S_ReadReg(__instance__, __REG__)               READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup I2S_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup I2S_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable I2S
  *
  *  Register|BitsName
  *  --------|--------
  *  ENABLE  | EN
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_enable(i2s_regs_t *I2Sx)
{
    SET_BITS(I2Sx->EN, I2S_EN_I2S_EN);
}

/**
  * @brief  Disable I2S
  *
  *  Register|BitsName
  *  --------|--------
  *  ENABLE  | EN
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_disable(i2s_regs_t *I2Sx)
{
    CLEAR_BITS(I2Sx->EN, I2S_EN_I2S_EN);
}

/**
  * @brief  Check if I2S is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  ENABLE  | EN
  *
  * @param  I2Sx I2S instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_is_enabled(i2s_regs_t *I2Sx)
{
    return (READ_BITS(I2Sx->EN, I2S_EN_I2S_EN) == (I2S_EN_I2S_EN));
}

/**
  * @brief  Enable I2S RX block
  *
  *  Register|BitsName
  *  --------|--------
  *  RBEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_enable_rxblock(i2s_regs_t *I2Sx)
{
    SET_BITS(I2Sx->RX_EN, I2S_RX_EN_RX_EN);
}

/**
  * @brief  Disable I2S RX block
  *
  *  Register|BitsName
  *  --------|--------
  *  RBEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_disable_rxblock(i2s_regs_t *I2Sx)
{
    CLEAR_BITS(I2Sx->RX_EN, I2S_RX_EN_RX_EN);
}

/**
  * @brief  Check if I2S RX block is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  RBEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_is_enabled_rxblock(i2s_regs_t *I2Sx)
{
    return (READ_BITS(I2Sx->RX_EN, I2S_RX_EN_RX_EN) == (I2S_RX_EN_RX_EN));
}

/**
  * @brief  Enable I2S TX block
  *
  *  Register|BitsName
  *  --------|--------
  *  TBEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_enable_txblock(i2s_regs_t *I2Sx)
{
    SET_BITS(I2Sx->TX_EN, I2S_TX_EN_TX_EN);
}

/**
  * @brief  Disable I2S TX block
  *
  *  Register|BitsName
  *  --------|--------
  *  TBEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_disable_txblock(i2s_regs_t *I2Sx)
{
    CLEAR_BITS(I2Sx->TX_EN, I2S_TX_EN_TX_EN);
}

/**
  * @brief  Check if I2S TX block is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  TBEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_is_enabled_txblock(i2s_regs_t *I2Sx)
{
    return (READ_BITS(I2Sx->TX_EN, I2S_TX_EN_TX_EN) == (I2S_TX_EN_TX_EN));
}

/**
  * @brief  Enable I2S clock
  *
  *  Register|BitsName
  *  --------|--------
  *  CLKEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_enable_clock(i2s_regs_t *I2Sx)
{
    SET_BITS(I2Sx->CLK_EN, I2S_CLK_EN_CLK_EN);
}

/**
  * @brief  Disable I2S clock
  *
  *  Register|BitsName
  *  --------|--------
  *  CLKEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_disable_clock(i2s_regs_t *I2Sx)
{
    CLEAR_BITS(I2Sx->CLK_EN, I2S_CLK_EN_CLK_EN);
}

/**
  * @brief  Check if I2S clock is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CLKEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_is_enabled_clock(i2s_regs_t *I2Sx)
{
    return (READ_BITS(I2Sx->CLK_EN, I2S_CLK_EN_CLK_EN) == (I2S_CLK_EN_CLK_EN));
}

/**
  * @brief  Set word select line cycles for left or right sample
  * @note   This bit should be written only when I2S is disabled (I2S_EN = 0) for correct operation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CLKCONFIG   | WSS
  *
  * @param  I2Sx I2S instance
  * @param  cycles This parameter can be one of the following values:
  *         @arg @ref LL_I2S_WS_CYCLES_16
  *         @arg @ref LL_I2S_WS_CYCLES_24
  *         @arg @ref LL_I2S_WS_CYCLES_32
  * @retval None
  */
__STATIC_INLINE void ll_i2s_set_wss(i2s_regs_t *I2Sx, uint32_t cycles)
{
    MODIFY_REG(I2Sx->SCLK_CFG, I2S_SCLK_CFG_WS_SCLK, cycles);
}

/**
  * @brief  Get word select line cycles for left or right sample
  *
  *  Register|BitsName
  *  --------|--------
  *  CLKCONFIG   | WSS
  *
  * @param  I2Sx I2S instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_I2S_WS_CYCLES_16
  *         @arg @ref LL_I2S_WS_CYCLES_24
  *         @arg @ref LL_I2S_WS_CYCLES_32
  */
__STATIC_INLINE uint32_t ll_i2s_get_wss(i2s_regs_t *I2Sx)
{
    return (uint32_t)(READ_BITS(I2Sx->SCLK_CFG, I2S_SCLK_CFG_WS_SCLK));
}

/**
  * @brief  Set the gating of sclk
  *
  *  Register|BitsName
  *  --------|--------
  *  CLKCONFIG   | SCLKG
  *
  * @param  I2Sx I2S instance
  * @param  cycles This parameter can be one of the following values:
  *         @arg @ref LL_I2S_SCLKG_NONE
  *         @arg @ref LL_I2S_SCLKG_CYCLES_12
  *         @arg @ref LL_I2S_SCLKG_CYCLES_16
  *         @arg @ref LL_I2S_SCLKG_CYCLES_20
  *         @arg @ref LL_I2S_SCLKG_CYCLES_24
  * @retval None
  */
__STATIC_INLINE void ll_i2s_set_sclkg(i2s_regs_t *I2Sx, uint32_t cycles)
{
    MODIFY_REG(I2Sx->SCLK_CFG, I2S_SCLK_CFG_SCLK_GAT, cycles);
}

/**
  * @brief  Get the gating of sclk
  *
  *  Register|BitsName
  *  --------|--------
  *  CLKCONFIG   | SCLKG
  *
  * @param  I2Sx I2S instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_I2S_SCLKG_NONE
  *         @arg @ref LL_I2S_SCLKG_CYCLES_12
  *         @arg @ref LL_I2S_SCLKG_CYCLES_16
  *         @arg @ref LL_I2S_SCLKG_CYCLES_20
  *         @arg @ref LL_I2S_SCLKG_CYCLES_24
  */
__STATIC_INLINE uint32_t ll_i2s_get_sclkg(i2s_regs_t *I2Sx)
{
    return (uint32_t)(READ_BITS(I2Sx->SCLK_CFG, I2S_SCLK_CFG_SCLK_GAT));
}

/**
  * @brief  Clear I2S RX FIFO in all channels
  *
  *  Register|BitsName
  *  --------|--------
  *  RXFIFO_RST | RST
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_clr_rxfifo_all(i2s_regs_t *I2Sx)
{
    WRITE_REG(I2Sx->RX_FIFO_RST, I2S_RX_FIFO_RST_RX_FIFO_RST);
}

/**
  * @brief  Clear I2S TX FIFO in all channels
  *
  *  Register|BitsName
  *  --------|--------
  *  TXFIFO_RST | RST
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_clr_txfifo_all(i2s_regs_t *I2Sx)
{
    WRITE_REG(I2Sx->TX_FIFO_RST, I2S_TX_FIFO_RST_TX_FIFO_RST);
}

/**
  * @brief  Set I2S clock divider
  *
  *  Register|BitsName
  *  --------|--------
  *  I2S_CLK_CFG   | DIV
  *
  * @param  div This parameter can between: 0 ~ 0xFFF
  * @retval None
  */
__STATIC_INLINE void ll_i2s_set_clock_div(uint32_t div)
{
    MODIFY_REG(MCU_SUB->I2S_CLK_CFG, MCU_SUB_I2S_CLK_CFG_DIV_CNT, div);
}

/**
  * @brief  Get I2S clock divider
  *
  *  Register|BitsName
  *  --------|--------
  *  I2S_CLK_CFG   | DIV
  *
  * @retval Returned Value can between: 0 ~ 0xFFF
  */
__STATIC_INLINE uint32_t ll_i2s_get_clock_div(void)
{
    return (uint32_t)(READ_BITS(MCU_SUB->I2S_CLK_CFG, MCU_SUB_I2S_CLK_CFG_DIV_CNT));
}

/**
  * @brief  Enable I2S clock divider
  *
  *  Register|BitsName
  *  --------|--------
  *  I2S_CLK_CFG   | DIV_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_i2s_enable_clock_div(void)
{
    SET_BITS(MCU_SUB->I2S_CLK_CFG, MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN);
}

/**
  * @brief  Disable I2S clock divider
  *
  *  Register|BitsName
  *  --------|--------
  *  I2S_CLK_CFG   | DIV_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_i2s_disable_clock_div(void)
{
    CLEAR_BITS(MCU_SUB->I2S_CLK_CFG, MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN);
}

/**
  * @brief  Check if I2S clock divider is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  I2S_CLK_CFG   | DIV_EN
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_is_enabled_clock_div(void)
{
    return (READ_BITS(MCU_SUB->I2S_CLK_CFG, MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN) == (MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN));
}

/**
  * @brief  Set I2S clock source
  *
  *  Register|BitsName
  *  --------|--------
  *  I2S_CLK_CFG   | SRC
  *
  * @param  src This parameter can be one of the following values:
  *         @arg @ref LL_I2S_CLOCK_SRC_96M
  *         @arg @ref LL_I2S_CLOCK_SRC_64M
  *         @arg @ref LL_I2S_CLOCK_SRC_32M
  * @retval None
  */
__STATIC_INLINE void ll_i2s_set_clock_src(uint32_t src)
{
    MODIFY_REG(MCU_SUB->I2S_CLK_CFG, MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL, src);
}

/**
  * @brief  Get I2S clock source
  *
  *  Register|BitsName
  *  --------|--------
  *  I2S_CLK_CFG   | SRC
  *
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_I2S_CLOCK_SRC_96M
  *         @arg @ref LL_I2S_CLOCK_SRC_32M
  */
__STATIC_INLINE uint32_t ll_i2s_get_clock_src(void)
{
    return (uint32_t)(READ_BITS(MCU_SUB->I2S_CLK_CFG, MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL));
}

/**
  * @brief  Enable I2S clock
  *
  *  Register|BitsName
  *  --------|--------
  *  I2S_CLK_CFG   | DIV_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_i2s_enable_src_clock(void)
{
    SET_BITS(MCU_SUB->I2S_CLK_CFG, MCU_SUB_I2S_CLK_CFG_SRC_CLK_EN);
}

/**
  * @brief  Disable I2S clock divider
  *
  *  Register|BitsName
  *  --------|--------
  *  I2S_CLK_CFG   | DIV_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_i2s_disable_src_clock(void)
{
    CLEAR_BITS(MCU_SUB->I2S_CLK_CFG, MCU_SUB_I2S_CLK_CFG_SRC_CLK_EN);
}

/**
  * @brief  Check if I2S clock divider is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  I2S_CLK_CFG   | DIV_EN
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_is_enabled_src_clock(void)
{
    return (READ_BITS(MCU_SUB->I2S_CLK_CFG, MCU_SUB_I2S_CLK_CFG_SRC_CLK_EN) == (MCU_SUB_I2S_CLK_CFG_SRC_CLK_EN));
}

/** @} */

/** @defgroup I2S_LL_EF_Channel Channel Configuration functions
  * @{
  */

/**
  * @brief  Read one data from left RX FIFO in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_L  | DATA
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE uint32_t ll_i2s_receive_ldata(i2s_regs_t *I2Sx)
{
    return (uint32_t)(READ_REG(I2Sx->LEFT_BUF));
}

/**
  * @brief  Read one data from right RX FIFO in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_R  | DATA
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE uint32_t ll_i2s_receive_rdata(i2s_regs_t *I2Sx)
{
    return (uint32_t)(READ_REG(I2Sx->RIGHT_BUF));
}

/**
  * @brief  Write one data to left TX FIFO in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_L  | DATA
  *
  * @param  I2Sx I2S instance
  * @param  data The data to send
  * @retval None
  */
__STATIC_INLINE void ll_i2s_transmit_ldata(i2s_regs_t *I2Sx, uint32_t data)
{
    WRITE_REG(I2Sx->LEFT_BUF, data);
}

/**
  * @brief  Write one data to right TX FIFO in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA_R  | DATA
  *
  * @param  I2Sx I2S instance
  * @param  data The data to send
  * @retval None
  */
__STATIC_INLINE void ll_i2s_transmit_rdata(i2s_regs_t *I2Sx, uint32_t data)
{
    WRITE_REG(I2Sx->RIGHT_BUF, data);
}

/**
  * @brief  Enable RX in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  RXEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_enable_rx(i2s_regs_t *I2Sx)
{
    SET_BITS(I2Sx->RX_CH_EN, I2S_RX_CH_EN_RX_CH_EN);
}

/**
  * @brief  Disable RX in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  RXEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_disable_rx(i2s_regs_t *I2Sx)
{
    CLEAR_BITS(I2Sx->RX_CH_EN, I2S_RX_CH_EN_RX_CH_EN);
}

/**
  * @brief  Check if RX in a channel is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  RXEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_is_enabled_rx(i2s_regs_t *I2Sx)
{
    return (READ_BITS(I2Sx->RX_CH_EN, I2S_RX_CH_EN_RX_CH_EN) == (I2S_RX_CH_EN_RX_CH_EN));
}

/**
  * @brief  Enable TX in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  TXEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_enable_tx(i2s_regs_t *I2Sx)
{
    SET_BITS(I2Sx->TX_CH_EN, I2S_TX_CH_EN_TX_CH_EN);
}

/**
  * @brief  Disable TX in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  TXEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_disable_tx(i2s_regs_t *I2Sx)
{
    CLEAR_BITS(I2Sx->TX_CH_EN, I2S_TX_CH_EN_TX_CH_EN);
}

/**
  * @brief  Check if TX in a channel is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  TXEN    | EN
  *
  * @param  I2Sx I2S instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_is_enabled_tx(i2s_regs_t *I2Sx)
{
    return (READ_BITS(I2Sx->TX_CH_EN, I2S_TX_CH_EN_TX_CH_EN) == (I2S_TX_CH_EN_TX_CH_EN));
}

/**
  * @brief  Set receive data width in a channel
  * @note   These bits should not be changed when channel is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  RXSIZE  | WLEN
  *
  * @param  I2Sx I2S instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_I2S_DATASIZE_IGNORE
  *         @arg @ref LL_I2S_DATASIZE_12BIT
  *         @arg @ref LL_I2S_DATASIZE_16BIT
  *         @arg @ref LL_I2S_DATASIZE_20BIT
  *         @arg @ref LL_I2S_DATASIZE_24BIT
  *         @arg @ref LL_I2S_DATASIZE_32BIT
  * @retval None
  */
__STATIC_INLINE void ll_i2s_set_rxsize(i2s_regs_t *I2Sx, uint32_t size)
{
    MODIFY_REG(I2Sx->RX_CFG, I2S_RX_CFG_WORD_LEN, size);
}

/**
  * @brief  Get receive data width in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  RXSIZE  | WLEN
  *
  * @param  I2Sx I2S instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_I2S_DATASIZE_IGNORE
  *         @arg @ref LL_I2S_DATASIZE_12BIT
  *         @arg @ref LL_I2S_DATASIZE_16BIT
  *         @arg @ref LL_I2S_DATASIZE_20BIT
  *         @arg @ref LL_I2S_DATASIZE_24BIT
  *         @arg @ref LL_I2S_DATASIZE_32BIT
  */
__STATIC_INLINE uint32_t ll_i2s_get_rxsize(i2s_regs_t *I2Sx)
{
    return (uint32_t)(READ_BITS(I2Sx->RX_CFG, I2S_RX_CFG_WORD_LEN));
}

/**
  * @brief  Set transmit data width in a channel
  * @note   These bits should not be changed when channel is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  TXSIZE  | WLEN
  *
  * @param  I2Sx I2S instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_I2S_DATASIZE_IGNORE
  *         @arg @ref LL_I2S_DATASIZE_12BIT
  *         @arg @ref LL_I2S_DATASIZE_16BIT
  *         @arg @ref LL_I2S_DATASIZE_20BIT
  *         @arg @ref LL_I2S_DATASIZE_24BIT
  *         @arg @ref LL_I2S_DATASIZE_32BIT
  * @retval None
  */
__STATIC_INLINE void ll_i2s_set_txsize(i2s_regs_t *I2Sx,uint32_t size)
{
    MODIFY_REG(I2Sx->TX_CFG, I2S_TX_CFG_WORD_LEN, size);
}

/**
  * @brief  Get transmit data width in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  TXSIZE  | WLEN
  *
  * @param  I2Sx I2S instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_I2S_DATASIZE_IGNORE
  *         @arg @ref LL_I2S_DATASIZE_12BIT
  *         @arg @ref LL_I2S_DATASIZE_16BIT
  *         @arg @ref LL_I2S_DATASIZE_20BIT
  *         @arg @ref LL_I2S_DATASIZE_24BIT
  *         @arg @ref LL_I2S_DATASIZE_32BIT
  */
__STATIC_INLINE uint32_t ll_i2s_get_txsize(i2s_regs_t *I2Sx)
{
    return (uint32_t)(READ_BITS(I2Sx->TX_CFG, I2S_TX_CFG_WORD_LEN));
}

/**
  * @brief  Get interrupt flag in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | TXFO
  *  INTSTAT | TXFE
  *  INTSTAT | RXFO
  *  INTSTAT | RXDA
  *
  * @param  I2Sx I2S instance
  * @retval Returned Value can be one or more of the following values:
  *         @arg @ref LL_I2S_STATUS_TXFO
  *         @arg @ref LL_I2S_STATUS_TXFE
  *         @arg @ref LL_I2S_STATUS_RXFO
  *         @arg @ref LL_I2S_STATUS_RXDA
  */
__STATIC_INLINE uint32_t ll_i2s_get_it_flag(i2s_regs_t *I2Sx)
{
    return (uint32_t)(READ_BITS(I2Sx->INT_STAT, I2S_INT_STAT_TX_FIFO_OVER | I2S_INT_STAT_TX_FIFO_EMPTY | \
                                I2S_INT_STAT_RX_FIFO_OVER | I2S_INT_STAT_RX_DATA_AVL));
}

/**
  * @brief  Check interrupt flag in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | TXFO
  *  INTSTAT | TXFE
  *  INTSTAT | RXFO
  *  INTSTAT | RXDA
  *
  * @param  I2Sx I2S instance
  * @param flag This parameter can be one or more of the following values:
  *         @arg @ref LL_I2S_STATUS_TXFO
  *         @arg @ref LL_I2S_STATUS_TXFE
  *         @arg @ref LL_I2S_STATUS_RXFO
  *         @arg @ref LL_I2S_STATUS_RXDA
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_is_active_it_flag(i2s_regs_t *I2Sx, uint32_t flag)
{
    return (uint32_t)(READ_BITS(I2Sx->INT_STAT, flag) == flag);
}

/**
  * @brief  Enable interrupt in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  INTMASK | TXFO
  *  INTMASK | TXFE
  *  INTMASK | RXFO
  *  INTMASK | RXDA
  *
  * @param  I2Sx I2S instance
  * @param  mask This parameter can be one or more of the following values:
  *         @arg @ref LL_I2S_INT_TXFO
  *         @arg @ref LL_I2S_INT_TXFE
  *         @arg @ref LL_I2S_INT_RXFO
  *         @arg @ref LL_I2S_INT_RXDA
  * @retval None
  */
__STATIC_INLINE void ll_i2s_enable_it(i2s_regs_t *I2Sx, uint32_t mask)
{
    CLEAR_BITS(I2Sx->INT_MASK, mask);
}

/**
  * @brief  Disable interrupt in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  INTMASK | TXFO
  *  INTMASK | TXFE
  *  INTMASK | RXFO
  *  INTMASK | RXDA
  *
  * @param  I2Sx I2S instance
  * @param  mask This parameter can be one or more of the following values:
  *         @arg @ref LL_I2S_INT_TXFO
  *         @arg @ref LL_I2S_INT_TXFE
  *         @arg @ref LL_I2S_INT_RXFO
  *         @arg @ref LL_I2S_INT_RXDA
  * @retval None
  */
__STATIC_INLINE void ll_i2s_disable_it(i2s_regs_t *I2Sx, uint32_t mask)
{
    SET_BITS(I2Sx->INT_MASK, mask);
}

/**
  * @brief  Check if interrupt in a channel is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  INTMASK | TXFO
  *  INTMASK | TXFE
  *  INTMASK | RXFO
  *  INTMASK | RXDA
  *
  * @param  I2Sx I2S instance
  * @param  mask This parameter can be one or more of the following values:
  *         @arg @ref LL_I2S_INT_TXFO
  *         @arg @ref LL_I2S_INT_TXFE
  *         @arg @ref LL_I2S_INT_RXFO
  *         @arg @ref LL_I2S_INT_RXDA
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_is_enabled_it(i2s_regs_t *I2Sx, uint32_t mask)
{
    return ((READ_BITS(I2Sx->INT_MASK, mask) ^ (mask)) == (mask));
}

/**
  * @brief  Clear RX FIFO data overrun interrupt flag in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  RXOVR   | RXCHO
  *
  * @param  I2Sx I2S instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_clear_it_rxovr(i2s_regs_t *I2Sx)
{
    return (READ_BITS(I2Sx->RX_OVER, I2S_RX_OVER_RX_CLR_FDO));
}

/**
  * @brief  Clear TX FIFO data overrun interrupt flag in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  TXOVR   | TXCHO
  *
  * @param  I2Sx I2S instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_clear_it_txovr(i2s_regs_t *I2Sx)
{
    return (READ_BITS(I2Sx->TX_OVER, I2S_TX_OVER_TX_CLR_FDO));
}

/**
  * @brief  Set threshold of RXFIFO in a channel that triggers an RXDA event
  *
  *  Register|BitsName
  *  --------|--------
  *  RXFIFO_TL | TL
  *
  * @param  I2Sx I2S instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_I2S_THRESHOLD_1FIFO
  *         @arg @ref LL_I2S_THRESHOLD_2FIFO
  *         @arg @ref LL_I2S_THRESHOLD_3FIFO
  *         @arg @ref LL_I2S_THRESHOLD_4FIFO
  *         @arg @ref LL_I2S_THRESHOLD_5FIFO
  *         @arg @ref LL_I2S_THRESHOLD_6FIFO
  *         @arg @ref LL_I2S_THRESHOLD_7FIFO
  *         @arg @ref LL_I2S_THRESHOLD_8FIFO
  *         @arg @ref LL_I2S_THRESHOLD_9FIFO
  *         @arg @ref LL_I2S_THRESHOLD_10FIFO
  *         @arg @ref LL_I2S_THRESHOLD_11FIFO
  *         @arg @ref LL_I2S_THRESHOLD_12FIFO
  *         @arg @ref LL_I2S_THRESHOLD_13FIFO
  *         @arg @ref LL_I2S_THRESHOLD_14FIFO
  *         @arg @ref LL_I2S_THRESHOLD_15FIFO
  *         @arg @ref LL_I2S_THRESHOLD_16FIFO
  * @retval None
  */
__STATIC_INLINE void ll_i2s_set_rx_fifo_threshold(i2s_regs_t *I2Sx, uint32_t threshold)
{
    WRITE_REG(I2Sx->RX_FIFO_CFG, threshold);
}

/**
  * @brief  Get threshold of RXFIFO in a channel that triggers an RXDA event
  *
  *  Register|BitsName
  *  --------|--------
  *  RXFIFO_TL | TL
  *
  * @param  I2Sx I2S instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_I2S_THRESHOLD_1FIFO
  *         @arg @ref LL_I2S_THRESHOLD_2FIFO
  *         @arg @ref LL_I2S_THRESHOLD_3FIFO
  *         @arg @ref LL_I2S_THRESHOLD_4FIFO
  *         @arg @ref LL_I2S_THRESHOLD_5FIFO
  *         @arg @ref LL_I2S_THRESHOLD_6FIFO
  *         @arg @ref LL_I2S_THRESHOLD_7FIFO
  *         @arg @ref LL_I2S_THRESHOLD_8FIFO
  *         @arg @ref LL_I2S_THRESHOLD_9FIFO
  *         @arg @ref LL_I2S_THRESHOLD_10FIFO
  *         @arg @ref LL_I2S_THRESHOLD_11FIFO
  *         @arg @ref LL_I2S_THRESHOLD_12FIFO
  *         @arg @ref LL_I2S_THRESHOLD_13FIFO
  *         @arg @ref LL_I2S_THRESHOLD_14FIFO
  *         @arg @ref LL_I2S_THRESHOLD_15FIFO
  *         @arg @ref LL_I2S_THRESHOLD_16FIFO
  */
__STATIC_INLINE uint32_t ll_i2s_get_rx_fifo_threshold(i2s_regs_t *I2Sx)
{
    return (uint32_t)(READ_BITS(I2Sx->RX_FIFO_CFG, I2S_RX_FIFO_CFG_RX_FIFO_TL));
}

/**
  * @brief  Set threshold of TXFIFO in a channel that triggers an TXFE event
  *
  *  Register|BitsName
  *  --------|--------
  *  TXFIFO_TL | TL
  *
  * @param  I2Sx I2S instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_I2S_THRESHOLD_1FIFO
  *         @arg @ref LL_I2S_THRESHOLD_2FIFO
  *         @arg @ref LL_I2S_THRESHOLD_3FIFO
  *         @arg @ref LL_I2S_THRESHOLD_4FIFO
  *         @arg @ref LL_I2S_THRESHOLD_5FIFO
  *         @arg @ref LL_I2S_THRESHOLD_6FIFO
  *         @arg @ref LL_I2S_THRESHOLD_7FIFO
  *         @arg @ref LL_I2S_THRESHOLD_8FIFO
  *         @arg @ref LL_I2S_THRESHOLD_9FIFO
  *         @arg @ref LL_I2S_THRESHOLD_10FIFO
  *         @arg @ref LL_I2S_THRESHOLD_11FIFO
  *         @arg @ref LL_I2S_THRESHOLD_12FIFO
  *         @arg @ref LL_I2S_THRESHOLD_13FIFO
  *         @arg @ref LL_I2S_THRESHOLD_14FIFO
  *         @arg @ref LL_I2S_THRESHOLD_15FIFO
  *         @arg @ref LL_I2S_THRESHOLD_16FIFO
  * @retval None
  */
__STATIC_INLINE void ll_i2s_set_tx_fifo_threshold(i2s_regs_t *I2Sx, uint32_t threshold)
{
    WRITE_REG(I2Sx->TX_FIFO_CFG, threshold);
}

/**
  * @brief  Get threshold of TXFIFO in a channel that triggers an TXFE event
  *
  *  Register|BitsName
  *  --------|--------
  *  TXFIFO_TL | TL
  *
  * @param  I2Sx I2S instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_I2S_THRESHOLD_1FIFO
  *         @arg @ref LL_I2S_THRESHOLD_2FIFO
  *         @arg @ref LL_I2S_THRESHOLD_3FIFO
  *         @arg @ref LL_I2S_THRESHOLD_4FIFO
  *         @arg @ref LL_I2S_THRESHOLD_5FIFO
  *         @arg @ref LL_I2S_THRESHOLD_6FIFO
  *         @arg @ref LL_I2S_THRESHOLD_7FIFO
  *         @arg @ref LL_I2S_THRESHOLD_8FIFO
  *         @arg @ref LL_I2S_THRESHOLD_9FIFO
  *         @arg @ref LL_I2S_THRESHOLD_10FIFO
  *         @arg @ref LL_I2S_THRESHOLD_11FIFO
  *         @arg @ref LL_I2S_THRESHOLD_12FIFO
  *         @arg @ref LL_I2S_THRESHOLD_13FIFO
  *         @arg @ref LL_I2S_THRESHOLD_14FIFO
  *         @arg @ref LL_I2S_THRESHOLD_15FIFO
  *         @arg @ref LL_I2S_THRESHOLD_16FIFO
  */
__STATIC_INLINE uint32_t ll_i2s_get_tx_fifo_threshold(i2s_regs_t *I2Sx)
{
    return (uint32_t)(READ_BITS(I2Sx->TX_FIFO_CFG, I2S_TX_FIFO_CFG_TX_FIFO_TL));
}

/**
  * @brief  Clear RX FIFO data in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  RXFIFO_FLUSH | FLUSH
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_clr_rxfifo_channel(i2s_regs_t *I2Sx)
{
    WRITE_REG(I2Sx->RX_FIFO_FLUSH, I2S_RX_FIFO_FLUSH_RX_FIFO_RST);
}

/**
  * @brief  Clear TX FIFO data in a channel
  *
  *  Register|BitsName
  *  --------|--------
  *  TXFIFO_FLUSH | FLUSH
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_clr_txfifo_channel(i2s_regs_t *I2Sx)
{
    WRITE_REG(I2Sx->TX_FIFO_FLUSH, I2S_TX_FIFO_FLUSH_TX_FIFO_RST);
}

/** @} */

/** @defgroup I2S_LL_EF_DMA_Management DMA Management Functions
  * @{
  */

/**
  * @brief  Reset RX block DMA
  * @note   The RX DMA can be reset to the lowest channel via this register.
  *
  *  Register|BitsName
  *  --------|--------
  *  RXDMA_RST | RST
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_rst_rxdma(i2s_regs_t *I2Sx)
{
    WRITE_REG(I2Sx->RST_RX_DMA, I2S_RST_RX_DMA_RST_RX_DMA);
}

/**
  * @brief  Reset TX block DMA
  * @note   The TX DMA can be reset to the lowest channel via this register.
  *
  *  Register|BitsName
  *  --------|--------
  *  TXDMA_RST | RST
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_rst_txdma(i2s_regs_t *I2Sx)
{
    WRITE_REG(I2Sx->RST_TX_DMA, I2S_RST_TX_DMA_RST_TX_DMA);
}


/**
  * @brief  Enable I2S DMA
  *
  *  Register|BitsName
  *  --------|--------
  *  DMA_ACC_SEL | QSPI1_I2S_M_SEL
  *  DMA_ACC_SEL | I2C1_I2S_S_SEL
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_enable_dma(i2s_regs_t *I2Sx)
{
    if (I2S_M == I2Sx)
        SET_BITS(MCU_SUB->I2S_DMA_MODE, MCU_SUB_I2S_DMA_MODE_SET);
    else
        SET_BITS(MCU_SUB->I2S_DMA_MODE, MCU_SUB_I2S_DMA_MODE_S_SET);
}

/**
  * @brief  Disable I2S DMA
  *
  *  Register|BitsName
  *  --------|--------
  *  DMA_ACC_SEL | QSPI1_I2S_M_SEL
  *  DMA_ACC_SEL | I2C1_I2S_S_SEL
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_disable_dma(i2s_regs_t *I2Sx)
{
    if (I2S_M == I2Sx)
        CLEAR_BITS(MCU_SUB->I2S_DMA_MODE, MCU_SUB_I2S_DMA_MODE_SET);
    else
        CLEAR_BITS(MCU_SUB->I2S_DMA_MODE, MCU_SUB_I2S_DMA_MODE_S_SET);
}

/**
  * @brief  Check if I2S DMA is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  DMA_ACC_SEL | QSPI1_I2S_M_SEL
  *  DMA_ACC_SEL | I2C1_I2S_S_SEL
  *
  * @param  I2Sx I2S instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_is_enabled_dma(i2s_regs_t *I2Sx)
{
    if (I2S_M == I2Sx)
        return (READ_BITS(MCU_SUB->I2S_DMA_MODE, MCU_SUB_I2S_DMA_MODE_SET) == MCU_SUB_I2S_DMA_MODE_SET);
    else
        return (READ_BITS(MCU_SUB->I2S_DMA_MODE, MCU_SUB_I2S_DMA_MODE_S_SET) == MCU_SUB_I2S_DMA_MODE_S_SET);
}

/**
  * @brief  Enable I2S DMA mode
  *
  *  Register|BitsName
  *  --------|--------
  *  DMA_ACC_SEL | MCU_SUB_I2S_DMA_MODE_SET
  *  DMA_ACC_SEL | MCU_SUB_I2S_DMA_MODE_S_SET
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_enable_dma_mode(i2s_regs_t *I2Sx)
{
    if (I2S_M == I2Sx)
        SET_BITS(MCU_SUB->I2S_DMA_MODE, MCU_SUB_I2S_DMA_MODE_SET);
    else
        SET_BITS(MCU_SUB->I2S_DMA_MODE, MCU_SUB_I2S_DMA_MODE_S_SET);
}

/**
  * @brief  Disable I2S DMA mode
  *
  *  Register|BitsName
  *  --------|--------
  *  DMA_ACC_SEL | MCU_SUB_I2S_DMA_MODE_SET
  *  DMA_ACC_SEL | MCU_SUB_I2S_DMA_MODE_S_SET
  *
  * @param  I2Sx I2S instance
  * @retval None
  */
__STATIC_INLINE void ll_i2s_disable_dma_mode(i2s_regs_t *I2Sx)
{
    if (I2S_M == I2Sx)
        CLEAR_BITS(MCU_SUB->I2S_DMA_MODE, MCU_SUB_I2S_DMA_MODE_SET);
    else
        CLEAR_BITS(MCU_SUB->I2S_DMA_MODE, MCU_SUB_I2S_DMA_MODE_S_SET);
}


/**
  * @brief  Check if I2S DMA mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  DMA_ACC_SEL | MCU_SUB_I2S_DMA_MODE_SET
  *  DMA_ACC_SEL | I2C1_I2S_S_SEL
  *
  * @param  I2Sx I2S instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2s_is_enabled_dma_mode(i2s_regs_t *I2Sx)
{
    if (I2S_M == I2Sx)
        return (READ_BITS(MCU_SUB->I2S_DMA_MODE, MCU_SUB_I2S_DMA_MODE_SET) == MCU_SUB_I2S_DMA_MODE_SET);
    else
        return (READ_BITS(MCU_SUB->I2S_DMA_MODE, MCU_SUB_I2S_DMA_MODE_S_SET) == MCU_SUB_I2S_DMA_MODE_S_SET);
}

/** @} */

/** @defgroup I2S_LL_EF_Init I2S_M Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize I2S registers (Registers restored to their default values).
  * @param  I2Sx I2S instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: I2S registers are de-initialized
  *          - ERROR: I2S registers are not de-initialized
  */
error_status_t ll_i2s_deinit(i2s_regs_t *I2Sx);

/**
  * @brief  Initialize I2S_M registers according to the specified
  *         parameters in p_i2s_init.
  * @param  I2Sx I2S instance
  * @param  p_i2s_init Pointer to a ll_i2s_init_t structure that contains the configuration
  *                         information for the specified I2S_M peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: I2S registers are initialized according to p_i2s_init content
  *          - ERROR: Problem occurred during I2S Registers initialization
  */
error_status_t ll_i2s_init(i2s_regs_t *I2Sx, ll_i2s_init_t *p_i2s_init);

/**
  * @brief Set each field of a @ref ll_i2s_init_t type structure to default value.
  * @param p_i2s_init  Pointer to a @ref ll_i2s_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_i2s_struct_init(ll_i2s_init_t *p_i2s_init);

/** @} */

/** @} */

#endif /* I2S_M || I2S_S */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_I2S_H__ */

/** @} */

/** @} */

/** @} */

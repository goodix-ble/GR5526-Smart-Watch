/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_hmac.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of HMAC LL library.
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

/** @defgroup LL_HMAC HMAC
  * @brief HMAC LL module driver.
  * @{
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_HMAC_H__
#define __GR55XX_LL_HMAC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (HMAC)

/** @defgroup HMAC_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup HMAC_LL_ES_INIT HMAC Exported Init structures
  * @{
  */

/**
  * @brief LL HMAC Init Structure definition
  */
typedef struct _ll_hmac_init_t
{
    uint32_t *p_key;         /**< Key         */

    uint32_t *p_hash;        /**< HASH value  */

} ll_hmac_init_t;

/** @} */

/** @} */

/**
  * @defgroup  HMAC_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup HMAC_LL_Exported_Constants HMAC Exported Constants
  * @{
  */

/** @defgroup HMAC_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags definitions which can be used with LL_HMAC_ReadReg function
  * @{
  */
#define LL_HMAC_FLAG_DATAREADY_SHA                           HMAC_STAT_HASH_READY      /**< HMAC data ready(SHA mode)   */
#define LL_HMAC_FLAG_DATAREADY_HMAC                          HMAC_STAT_HMAC_READY      /**< HMAC data ready(HAMC mode)  */
#define LL_HMAC_FLAG_DMA_MESSAGEDONE                         HMAC_STAT_DMA_MSG_DONE    /**< HMAC dma message done       */
#define LL_HMAC_FLAG_DMA_DONE                                HMAC_STAT_DMA_TX_DONE     /**< HMAC dma transfer done      */
#define LL_HMAC_FLAG_DMA_ERR                                 HMAC_STAT_DMA_TX_ERR      /**< HMAC dma transfer error     */
#define LL_HMAC_FLAG_KEY_VALID                               HMAC_STAT_KEY_VALID       /**< HMAC has fetched key        */
/** @} */

/** @defgroup HMAC_LL_EC_HASH_MODE Hash Mode
  * @{
  */
#define LL_HMAC_HASH_STANDARD                                0x00000000U                            /**< Standard Mode */
#define LL_HMAC_HASH_USER                                    (1UL << HMAC_CFG_HASH_POS)             /**< User Mode     */
/** @} */

/** @defgroup HMAC_LL_EC_CALCULATE_TYPE Calculate Type
  * @{
  */
#define LL_HMAC_CALCULATETYPE_HMAC                           0x00000000U                            /**< HMAC mode */
#define LL_HMAC_CALCULATETYPE_SHA                            (1UL << HMAC_CFG_CALC_TYPE_POS)        /**< SHA  moe  */
/** @} */

/** @defgroup HMAC_LL_EC_KEY_TYPE Key Type
  * @{
  */
#define LL_HMAC_KEYTYPE_MCU                                  0x00000000U                            /**< MCU        */
#define LL_HMAC_KEYTYPE_AHB                                  (1UL << HMAC_CFG_KEY_TYPE_POS)         /**< AHB master */
#define LL_HMAC_KEYTYPE_KRAM                                 (2UL << HMAC_CFG_KEY_TYPE_POS)         /**< Key Port   */
/** @} */

/** @defgroup HMAC_LL_EC_TRANSFER_SIZE Transfer Size
  * @{
  */
#define LL_HMAC_DMA_TRANSIZE_MIN                             (1)     /**< Min size = 1 block      */
#define LL_HMAC_DMA_TRANSIZE_MAX                             (512)   /**< Min size = 512 blocks   */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup HMAC_LL_Exported_Macros HMAC Exported Macros
  * @{
  */

/** @defgroup HMAC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in HMAC register
  * @param  __INSTANCE__ HMAC Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_HMAC_WriteReg(__INSTANCE__, __REG__, __VALUE__)   WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in HMAC register
  * @param  __INSTANCE__ HMAC Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_HMAC_ReadReg(__INSTANCE__, __REG__)               READ_REG(__INSTANCE__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup HMAC_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup HMAC_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable HMAC.

  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | EN

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CTRL, HMAC_CTRL_EN);
}

/**
  * @brief  Disable HMAC.

  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | EN

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->CTRL, HMAC_CTRL_EN);
}

/**
  * @brief  Indicate whether the HMAC is enabled.

  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | EN

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CTRL, HMAC_CTRL_EN) == (HMAC_CTRL_EN));
}

/**
  * @brief  Enable HMAC DMA mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | DMA_START

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_dma_start(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CTRL, HMAC_CTRL_DMA_START);
}

/**
  * @brief  Disable HMAC DMA mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | DMA_START

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable_dma_start(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->CTRL, HMAC_CTRL_DMA_START);
}

/**
  * @brief  Indicate whether the HMAC DMA mode is enabled.

  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | DMA_START

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled_dma_start(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CTRL, HMAC_CTRL_DMA_START) == (HMAC_CTRL_DMA_START));
}

/**
  * @brief  Enable fetch key through AHB/key port.

  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | KEY_EN

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_read_key(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CTRL, HMAC_CTRL_KEY_EN);
}

/**
  * @brief  Enable last block transfer in MCU/DMA mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | LST_TX

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_last_transfer(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CTRL, HMAC_CTRL_LST_TX);
}

/**
  * @brief  Enable user HASH.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | HASH

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_user_hash(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CFG, HMAC_CFG_HASH);
}

/**
  * @brief  Disable user HASH.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | HASH

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable_user_hash(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->CFG, HMAC_CFG_HASH);
}

/**
  * @brief  Indicate whether the user HASH is enabled.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | HASH

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled_user_hash(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CFG, HMAC_CFG_HASH) == (HMAC_CFG_HASH));
}

/**
  * @brief  Enable HMAC in little endian.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | ENDIAN

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_little_endian(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CFG, HMAC_CFG_ENDIAN);
}

/**
  * @brief  Disable HMAC in little endian.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | ENDIAN

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable_little_endian(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->CFG, HMAC_CFG_ENDIAN);
}

/**
  * @brief  Indicate whether the HMAC is in little endian.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | ENDIAN

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled_little_endian(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CFG, HMAC_CFG_ENDIAN) == (HMAC_CFG_ENDIAN));
}

/**
  * @brief  Set ways to obtain HMAC key.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | KEY_TYPE

  * @param  HMACx HMAC instance
  * @param  type This parameter can be one of the following values:
  *         @arg @ref LL_HMAC_KEYTYPE_MCU
  *         @arg @ref LL_HMAC_KEYTYPE_AHB
  *         @arg @ref LL_HMAC_KEYTYPE_KRAM
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key_type(hmac_regs_t *HMACx, uint32_t type)
{
    MODIFY_REG(HMACx->CFG, HMAC_CFG_KEY_TYPE, type);
}

/**
  * @brief  Get ways to obtain HMAC key.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | KEY_TYPE

  * @param  HMACx HMAC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_HMAC_KEYTYPE_MCU
  *         @arg @ref LL_HMAC_KEYTYPE_AHB
  *         @arg @ref LL_HMAC_KEYTYPE_KRAM
  */
__STATIC_INLINE uint32_t ll_hmac_get_key_type(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CFG, HMAC_CFG_KEY_TYPE));
}

/**
  * @brief  Enable SHA mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | CALC_TYPE

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_sha(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CFG, HMAC_CFG_CALC_TYPE);
}

/**
  * @brief  Disable SHA mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | CALC_TYPE

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable_sha(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->CFG, HMAC_CFG_CALC_TYPE);
}

/**
  * @brief  Indicate whether the SHA mode is enabled.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | CALC_TYPE

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled_sha(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CFG, HMAC_CFG_CALC_TYPE) == (HMAC_CFG_CALC_TYPE));
}

/**
  * @brief  Enable private mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | PRIVT_MODE

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_private(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CFG, HMAC_CFG_PRIVT_MODE);
}

/**
  * @brief  Disable private mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | PRIVT_MODE

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable_private(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->CFG, HMAC_CFG_PRIVT_MODE);
}

/**
  * @brief  Indicate whether the private mode is enabled.

  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | PRIVT_MODE

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled_private(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CFG, HMAC_CFG_PRIVT_MODE) == (HMAC_CFG_PRIVT_MODE));
}

/** @} */

/** @defgroup HMAC_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable the done interrupt for HMAC.

  *  Register|BitsName
  *  ---------------|---------------
  *  INT            | EN

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_it_done(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->INT, HMAC_INT_EN);
}

/**
  * @brief  Disable the done interrupt for HMAC.

  *  Register|BitsName
  *  ---------------|---------------
  *  INT            | EN

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable_it_done(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->INT, HMAC_INT_EN);
}

/**
  * @brief  Indicate whether Done Interrupt is enabled.

  *  Register|BitsName
  *  ---------------|---------------
  *  INT            | EN

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled_it_done(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->INT, HMAC_INT_EN) == (HMAC_INT_EN));
}

/** @} */

/** @defgroup HMAC_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Indicate whether SHA Ready flag is set.

  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | HASH_READY

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_sha_ready(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->STAT, HMAC_STAT_HASH_READY) == HMAC_STAT_HASH_READY);
}

/**
  * @brief  Indicate whether HMAC Ready flag is set.

  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | HMAC_READY

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_hmac_ready(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->STAT, HMAC_STAT_HMAC_READY) == HMAC_STAT_HMAC_READY);
}

/**
  * @brief  Indicate whether DMA Transmit Message Done flag is set.

  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | DMA_MSG_DONE

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_dma_message_done(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->STAT, HMAC_STAT_DMA_MSG_DONE) == HMAC_STAT_DMA_MSG_DONE);
}

/**
  * @brief  Indicate whether DMA Transfer Done flag is set.

  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | DMA_TX_DONE

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_dma_done(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->STAT, HMAC_STAT_DMA_TX_DONE) == HMAC_STAT_DMA_TX_DONE);
}

/**
  * @brief  Indicate whether DMA Transfer Error flag is set.

  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | DMA_TX_ERR

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_dma_error(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->STAT, HMAC_STAT_DMA_TX_ERR) == HMAC_STAT_DMA_TX_ERR);
}

/**
  * @brief  Indicate whether Key Valid flag is set.

  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | KEY_VALID

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_key_valid(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->STAT, HMAC_STAT_KEY_VALID) == HMAC_STAT_KEY_VALID);
}

/**
  * @brief  Indicate whether Done interrupt flag is set.

  *  Register|BitsName
  *  ---------------|---------------
  *  INT            | DONE

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_it_done(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->INT, HMAC_INT_DONE) == HMAC_INT_DONE);
}

/**
  * @brief  Clear Done interrupt flag.

  *  Register|BitsName
  *  ---------------|---------------
  *  INT            | DONE

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_clear_flag_it_done(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->INT, HMAC_INT_DONE);
}

/** @} */

/** @defgroup HMAC_LL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Set HMAC transfer blocks in DMA mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  XFE_SIZE       | SIZE

  * @param  HMACx HMAC instance
  * @param  block This parameter can be one of the following values: 1 ~ 512
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_dma_transfer_block(hmac_regs_t *HMACx, uint32_t block)
{
    MODIFY_REG(HMACx->XFE_SIZE, HMAC_XFE_SIZE_SIZE, (block << 6) - 1);
}

/**
  * @brief  Get HMAC transfer blocks in DMA mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  XFE_SIZE       | SIZE

  * @param  HMACx HMAC instance
  * @retval Return value is between: 1 ~ 512
  */
__STATIC_INLINE uint32_t ll_hmac_get_dma_transfer_block(hmac_regs_t *HMACx)
{
    return ((READ_BITS(HMACx->XFE_SIZE, HMAC_XFE_SIZE_SIZE) + 1) >> 6);
}

/**
  * @brief  Set HMAC read address of RAM in DMA mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  RD_START_ADDR  | ADDR

  * @param  HMACx HMAC instance
  * @param  address This parameter can be one of the address in RAM
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_dma_read_address(hmac_regs_t *HMACx, uint32_t address)
{
    WRITE_REG(HMACx->RD_START_ADDR, address);
}

/**
  * @brief  Get HMAC read address of RAM in DMA mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  RD_START_ADDR  | ADDR

  * @param  HMACx HMAC instance
  * @retval Return value is the address in RAM
  */
__STATIC_INLINE uint32_t ll_hmac_get_dma_read_address(hmac_regs_t *HMACx)
{
    return (READ_REG(HMACx->RD_START_ADDR));
}

/**
  * @brief  Set HMAC write address of RAM in DMA mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  WR_START_ADDR  | ADDR

  * @param  HMACx HMAC instance
  * @param  address This parameter can be one of the address in RAM
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_dma_write_address(hmac_regs_t *HMACx, uint32_t address)
{
    WRITE_REG(HMACx->WR_START_ADDR, address);
}

/**
  * @brief  Get HMAC write address of RAM in DMA mode.

  *  Register|BitsName
  *  ---------------|---------------
  *  WR_START_ADDR  | ADDR

  * @param  HMACx HMAC instance
  * @retval Return value is the address in RAM
  */
__STATIC_INLINE uint32_t ll_hmac_get_dma_write_address(hmac_regs_t *HMACx)
{
    return (READ_REG(HMACx->WR_START_ADDR));
}

/** @} */

/** @defgroup HMAC_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Set user HASH[255:224].

  *  Register|BitsName
  *  ---------------|---------------
  *  USER_HASH_0    | USER_HASH

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_255_224(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH_0, hash);
}

/**
  * @brief  Set user HASH[223:192].

  *  Register|BitsName
  *  ---------------|---------------
  *  USER_HASH_1    | USER_HASH

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_223_192(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH_1, hash);
}

/**
  * @brief  Set user HASH[191:160].

  *  Register|BitsName
  *  ---------------|---------------
  *  USER_HASH_2    | USER_HASH

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_191_160(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH_2, hash);
}

/**
  * @brief  Set user HASH[159:128].

  *  Register|BitsName
  *  ---------------|---------------
  *  USER_HASH_3    | USER_HASH

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_159_128(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH_3, hash);
}

/**
  * @brief  Set user HASH[127:96].

  *  Register|BitsName
  *  ---------------|---------------
  *  USER_HASH_4    | USER_HASH

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_127_96(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH_4, hash);
}

/**
  * @brief  Set user HASH[95:64].

  *  Register|BitsName
  *  ---------------|---------------
  *  USER_HASH_5    | USER_HASH

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_95_64(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH_5, hash);
}

/**
  * @brief  Set user HASH[63:32].

  *  Register|BitsName
  *  ---------------|---------------
  *  USER_HASH_6    | USER_HASH

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_63_32(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH_6, hash);
}

/**
  * @brief  Set user HASH[31:0].

  *  Register|BitsName
  *  ---------------|---------------
  *  USER_HASH_7    | USER_HASH

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_31_0(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH_7, hash);
}

/**
  * @brief  Get abstract from HMAC.

  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_OUT       | DATA

  * @param  HMACx HMAC instance
  * @retval Abstract
  */
__STATIC_INLINE uint32_t ll_hmac_get_data(hmac_regs_t *HMACx)
{
    return (READ_REG(HMACx->DATA_OUT));
}

/**
  * @brief  Send data to calculate.

  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_IN        | DATA

  * @param  HMACx HMAC instance
  * @param  data This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_data(hmac_regs_t *HMACx, uint32_t data)
{
    WRITE_REG(HMACx->DATA_IN, data);
}

/**
  * @brief  Set HMAC key0.

  *  Register|BitsName
  *  ---------------|---------------
  *  KEY0           | KEY

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key0(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY0, key);
}

/**
  * @brief  Set HMAC key1.

  *  Register|BitsName
  *  ---------------|---------------
  *  KEY1           | KEY

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key1(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY1, key);
}

/**
  * @brief  Set HMAC key2.

  *  Register|BitsName
  *  ---------------|---------------
  *  KEY2           | KEY

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key2(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY2, key);
}

/**
  * @brief  Set HMAC key3.

  *  Register|BitsName
  *  ---------------|---------------
  *  KEY3           | KEY

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key3(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY3, key);
}

/**
  * @brief  Set HMAC key4.

  *  Register|BitsName
  *  ---------------|---------------
  *  KEY4           | KEY

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key4(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY4, key);
}

/**
  * @brief  Set HMAC key5.

  *  Register|BitsName
  *  ---------------|---------------
  *  KEY5           | KEY

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key5(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY5, key);
}

/**
  * @brief  Set HMAC key6.

  *  Register|BitsName
  *  ---------------|---------------
  *  KEY6           | KEY

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key6(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY6, key);
}

/**
  * @brief  Set HMAC key7.

  *  Register|BitsName
  *  ---------------|---------------
  *  KEY7           | KEY

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key7(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY7, key);
}

/**
  * @brief  Set HMAC key address in memory.

  *  Register|BitsName
  *  ---------------|---------------
  *  KEY_ADDR       | ADDR

  * @param  HMACx HMAC instance
  * @param  address This parameter can be one of the address in RAM
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key_address(hmac_regs_t *HMACx, uint32_t address)
{
    WRITE_REG(HMACx->KEY_ADDR, address);
}

/**
  * @brief  Get HMAC key address in memory.

  *  Register|BitsName
  *  ---------------|---------------
  *  KEY_ADDR       | ADDR

  * @param  HMACx HMAC instance
  * @retval Return value is the address in RAM
  */
__STATIC_INLINE uint32_t ll_hmac_get_key_address(hmac_regs_t *HMACx)
{
    return (READ_REG(HMACx->KEY_ADDR));
}

/**
  * @brief  Set HMAC fetch key port mask.

  *  Register|BitsName
  *  ---------------|---------------
  *  KEYPORT_MASK   | MASK

  * @param  HMACx HMAC instance
  * @param  mask This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key_port_mask(hmac_regs_t *HMACx, uint32_t mask)
{
    WRITE_REG(HMACx->KEYPORT_MASK, mask);
}

/** @} */

/** @defgroup HMAC_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize HMAC registers (Registers restored to their default values).
  * @param  HMACx       HMAC Instance
  * @retval An error_status_t  enumeration value:
  *          - SUCCESS: HMAC registers are de-initialized
  *          - ERROR: HMAC registers are not de-initialized
  */
error_status_t ll_hmac_deinit(hmac_regs_t *HMACx);

/**
  * @brief  Initialize HMAC registers according to the specified
  *         parameters in p_hmac_init.
  * @param  HMACx       HMAC Instance
  * @param  p_hmac_init   Pointer to a ll_hmac_init_t structure that contains the configuration
  *                     information for the specified HMAC peripheral.
  * @retval An error_status_t  enumeration value:
  *          - SUCCESS: HMAC registers are initialized according to p_hmac_init content
  *          - ERROR: Problem occurred during HMAC Registers initialization
  */
error_status_t ll_hmac_init(hmac_regs_t *HMACx, ll_hmac_init_t *p_hmac_init);

/**
  * @brief Set each field of a @ref ll_hmac_init_t type structure to default value.
  * @param p_hmac_init    Pointer to a @ref ll_hmac_init_t structure
  *                     whose fields will be set to default values.
  * @retval None
  */
void ll_hmac_struct_init(ll_hmac_init_t *p_hmac_init);

/** @} */

/** @} */

#endif /* HMAC */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_HMAC_H__ */

/** @} */

/** @} */

/** @} */

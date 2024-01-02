/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_aes.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of AES LL library.
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

/** @defgroup LL_AES AES
  * @brief AES LL module driver.
  * @{
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_AES_H__
#define __GR55XX_LL_AES_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (AES)

/** @defgroup AES_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup AES_LL_ES_INIT AES Exported Init structures
  * @{
  */

/**
  * @brief LL AES Init Structure definition
  */
typedef struct _ll_aes_init
{
    uint32_t key_size;      /**< 128, 192 or 256-bit key length.
                                 This parameter can be a value of @ref AES_LL_EC_KEY_SIZE */

    uint32_t *p_key;          /**< Encryption/Decryption Key */

    uint32_t *p_init_vector;  /**< Initialization Vector used for CBC modes */

    uint32_t *p_seed;         /**< Random seeds */

} ll_aes_init_t;

/** @} */

/** @} */

/**
  * @defgroup  AES_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup AES_LL_Exported_Constants AES Exported Constants
  * @{
  */

/** @defgroup AES_LL_EC_GET_FLAG Get Flag Defines
  * @brief    Flag definitions which can be used with LL_AES_ReadReg function
  * @{
  */
#define LL_AES_FLAG_DATAREADY                               AES_STAT_READY           /**< AES result data out ready  */
#define LL_AES_FLAG_DMA_DONE                                AES_STAT_DMA_XFE_CPLT    /**< AES dma transfer done      */
#define LL_AES_FLAG_DMA_ERR                                 AES_STAT_DMA_XFE_ERR     /**< AES dma transfer error     */
#define LL_AES_FLAG_KEY_VALID                               AES_STAT_KEY_STAT        /**< AES has fetched key        */
/** @} */

/** @defgroup AES_LL_EC_KEY_SIZE Key Size
  * @{
  */
#define LL_AES_KEY_SIZE_128                                 0x00000000U                     /**< 128 bits */
#define LL_AES_KEY_SIZE_192                                 (1UL << AES_CFG_KEY_MODE_POS)   /**< 192 bits */
#define LL_AES_KEY_SIZE_256                                 (2UL << AES_CFG_KEY_MODE_POS)   /**< 256 bits */
/** @} */

/** @defgroup AES_LL_EC_OPERATION_MODE Operation Mode
  * @{
  */
#define LL_AES_OPERATION_MODE_ECB                           0x00000000U                     /**< Electronic codebook (ECB) mode   */
#define LL_AES_OPERATION_MODE_CBC                           (1UL << AES_CFG_OPT_MODE_POS)   /**< Cipher block chaining (CBC) mode */
/** @} */

/** @defgroup AES_LL_EC_KEY_TYPE Key Type
  * @{
  */
#define LL_AES_KEYTYPE_MCU                                  0x00000000U                     /**< MCU        */
#define LL_AES_KEYTYPE_AHB                                  (1UL << AES_CFG_KEY_TYPE_POS)   /**< AHB master */
#define LL_AES_KEYTYPE_KRAM                                 (2UL << AES_CFG_KEY_TYPE_POS)   /**< Key Port   */
/** @} */

/** @defgroup AES_LL_EC_TRANSFER_SIZE Transfer Size
  * @{
  */
#define LL_AES_DMA_TRANSIZE_MIN                             (1)     /**< Min size = 1 block      */
#define LL_AES_DMA_TRANSIZE_MAX                             (2048)  /**< Max size = 2048 blocks  */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup AES_LL_Exported_Macros AES Exported Macros
  * @{
  */

/** @defgroup AES_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in AES register
  * @param  __INSTANCE__ AES Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_AES_WriteReg(__INSTANCE__, __REG__, __VALUE__)   WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in AES register
  * @param  __INSTANCE__ AES Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_AES_ReadReg(__INSTANCE__, __REG__)               READ_REG(__INSTANCE__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup AES_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup AES_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable AES.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL      | MODULE_EN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_enable(aes_regs_t *AESx)
{
    SET_BITS(AESx->CTRL, AES_CTRL_MODULE_EN);
}

/**
  * @brief  Disable AES.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | MODULE_EN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_disable(aes_regs_t *AESx)
{
    CLEAR_BITS(AESx->CTRL, AES_CTRL_MODULE_EN);
}

/**
  * @brief  Indicate whether the AES is enabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | MODULE_EN
  *
  * @param  AESx AES instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aes_is_enabled(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->CTRL, AES_CTRL_MODULE_EN) == (AES_CTRL_MODULE_EN));
}

/**
  * @brief  Enable AES start in MCU mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | MCU_MODE_EN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_enable_start(aes_regs_t *AESx)
{
    SET_BITS(AESx->CTRL, AES_CTRL_MCU_MODE_EN);
}

/**
  * @brief  Disable AES start in MCU mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | MCU_MODE_EN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_disable_start(aes_regs_t *AESx)
{
    CLEAR_BITS(AESx->CTRL, AES_CTRL_MCU_MODE_EN);
}

/**
  * @brief  Indicate whether the AES start in MCU mode is enabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | MCU_MODE_EN
  *
  * @param  AESx AES instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aes_is_enabled_start(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->CTRL, AES_CTRL_MCU_MODE_EN) == (AES_CTRL_MCU_MODE_EN));
}

/**
  * @brief  Enable AES DMA mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | DMA_MODE_EN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_enable_dma_start(aes_regs_t *AESx)
{
    SET_BITS(AESx->CTRL, AES_CTRL_DMA_MODE_EN);
}

/**
  * @brief  Disable AES DMA mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | DMA_MODE_EN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_disable_dma_start(aes_regs_t *AESx)
{
    CLEAR_BITS(AESx->CTRL, AES_CTRL_DMA_MODE_EN);
}

/**
  * @brief  Indicate whether the AES DMA mode is enabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | DMA_MODE_EN
  *
  * @param  AESx AES instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aes_is_enabled_dma_start(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->CTRL, AES_CTRL_DMA_MODE_EN) == (AES_CTRL_DMA_MODE_EN));
}

/**
  * @brief  Enable fetch key through AHB/key port.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | FKEY_EN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_enable_read_key(aes_regs_t *AESx)
{
    SET_BITS(AESx->CTRL, AES_CTRL_FKEY_EN);
}

/**
  * @brief  Set AES key size.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | KEY_MODE
  *
  * @param  AESx AES instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_AES_KEY_SIZE_128
  *         @arg @ref LL_AES_KEY_SIZE_192
  *         @arg @ref LL_AES_KEY_SIZE_256
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_key_size(aes_regs_t *AESx, uint32_t size)
{
    MODIFY_REG(AESx->CFG, AES_CFG_KEY_MODE, size);
}

/**
  * @brief  Get AES key size.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | KEY_MODE
  *
  * @param  AESx AES instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_AES_KEY_SIZE_128
  *         @arg @ref LL_AES_KEY_SIZE_192
  *         @arg @ref LL_AES_KEY_SIZE_256
  */
__STATIC_INLINE uint32_t ll_aes_get_key_size(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->CFG, AES_CFG_KEY_MODE));
}

/**
  * @brief  Enable AES full mask.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | FULL_MASK_EN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_enable_full_mask(aes_regs_t *AESx)
{
    SET_BITS(AESx->CFG, AES_CFG_FULL_MASK_EN);
}

/**
  * @brief  Disable AES full mask.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | FULL_MASK_EN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_disable_full_mask(aes_regs_t *AESx)
{
    CLEAR_BITS(AESx->CFG, AES_CFG_FULL_MASK_EN);
}

/**
  * @brief  Indicate whether the AES full mask is enabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | FULL_MASK_EN
  *
  * @param  AESx AES instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aes_is_enabled_full_mask(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->CFG, AES_CFG_FULL_MASK_EN) == (AES_CFG_FULL_MASK_EN));
}

/**
  * @brief  Enable AES encryption mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | DEC_ENC_SEL
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_enable_encryption(aes_regs_t *AESx)
{
    SET_BITS(AESx->CFG, AES_CFG_DEC_ENC_SEL);
}

/**
  * @brief  Disable AES encryption mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | DEC_ENC_SEL
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_disable_encryption(aes_regs_t *AESx)
{
    CLEAR_BITS(AESx->CFG, AES_CFG_DEC_ENC_SEL);
}

/**
  * @brief  Indicate whether the AES encryption mode is enabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | DEC_ENC_SEL
  *
  * @param  AESx AES instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aes_is_enabled_encryption(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->CFG, AES_CFG_DEC_ENC_SEL) == (AES_CFG_DEC_ENC_SEL));
}

/**
  * @brief  Set AES to load seed for LFSR.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | LOAD_SEED
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_load_seed(aes_regs_t *AESx)
{
    SET_BITS(AESx->CFG, AES_CFG_LOAD_SEED);
}

/**
  * @brief  Set AES in first block before starting the first block in normal CBC and DMA CBC mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | FIRST_BLK
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_first_block(aes_regs_t *AESx)
{
    SET_BITS(AESx->CFG, AES_CFG_FIRST_BLK);
}

/**
  * @brief  Enable AES in little endian.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | ENDIAN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_enable_little_endian(aes_regs_t *AESx)
{
    SET_BITS(AESx->CFG, AES_CFG_ENDIAN);
}

/**
  * @brief  Disable AES in little endian.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | ENDIAN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_disable_little_endian(aes_regs_t *AESx)
{
    CLEAR_BITS(AESx->CFG, AES_CFG_ENDIAN);
}

/**
  * @brief  Indicate whether the AES is in little endian.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | ENDIAN
  *
  * @param  AESx AES instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aes_is_enabled_little_endian(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->CFG, AES_CFG_ENDIAN) == (AES_CFG_ENDIAN));
}

/**
  * @brief  Set AES operation mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | OPT_MODE
  *
  * @param  AESx AES instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_AES_OPERATION_MODE_ECB
  *         @arg @ref LL_AES_OPERATION_MODE_CBC
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_operation_mode(aes_regs_t *AESx, uint32_t mode)
{
    MODIFY_REG(AESx->CFG, AES_CFG_OPT_MODE, mode);
}

/**
  * @brief  Get AES operation mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | OPT_MODE
  *
  * @param  AESx AES instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_AES_OPERATION_MODE_ECB
  *         @arg @ref LL_AES_OPERATION_MODE_CBC
  */
__STATIC_INLINE uint32_t ll_aes_get_operation_mode(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->CFG, AES_CFG_OPT_MODE));
}

/**
  * @brief  Set ways to obtain AES key.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | KEY_TYPE
  *
  * @param  AESx AES instance
  * @param  Type This parameter can be one of the following values:
  *         @arg @ref LL_AES_KEYTYPE_MCU
  *         @arg @ref LL_AES_KEYTYPE_AHB
  *         @arg @ref LL_AES_KEYTYPE_KRAM
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_key_type(aes_regs_t *AESx, uint32_t Type)
{
    MODIFY_REG(AESx->CFG, AES_CFG_KEY_TYPE, Type);
}

/**
  * @brief  Get ways to obtain AES key.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CFG            | KEY_TYPE
  *
  * @param  AESx AES instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_AES_KEYTYPE_MCU
  *         @arg @ref LL_AES_KEYTYPE_AHB
  *         @arg @ref LL_AES_KEYTYPE_KRAM
  */
__STATIC_INLINE uint32_t ll_aes_get_key_type(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->CFG, AES_CFG_KEY_TYPE));
}

/** @} */

/** @defgroup AES_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable AES the done interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT            | CPLT_INT_EN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_enable_it_done(aes_regs_t *AESx)
{
    SET_BITS(AESx->INT, AES_INT_CPLT_INT_EN);
}

/**
  * @brief  Disable AES the done interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT            | CPLT_INT_EN
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_disable_it_done(aes_regs_t *AESx)
{
    CLEAR_BITS(AESx->INT, AES_INT_CPLT_INT_EN);
}

/**
  * @brief  Indicate whether the done interrupt is enabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT            | CPLT_INT_EN
  *
  * @param  AESx AES instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aes_is_enabled_it_done(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->INT, AES_INT_CPLT_INT_EN) == (AES_INT_CPLT_INT_EN));
}

/** @} */

/** @defgroup AES_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Indicate whether the ready flag is set.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | READY
  *
  * @param  AESx AES instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aes_is_action_flag_ready(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->STAT, AES_STAT_READY) == AES_STAT_READY);
}

/**
  * @brief  Indicate whether the DMA transfer done flag is set.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | DMA_XFE_CPLT
  *
  * @param  AESx AES instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aes_is_action_flag_dma_done(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->STAT, AES_STAT_DMA_XFE_CPLT) == AES_STAT_DMA_XFE_CPLT);
}

/**
  * @brief  Indicate whether the DMA transfer error flag is set.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | DMA_XFE_ERR
  *
  * @param  AESx AES instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aes_is_action_flag_dma_error(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->STAT, AES_STAT_DMA_XFE_ERR) == AES_STAT_DMA_XFE_ERR);
}

/**
  * @brief  Indicate whether the key valid flag is set.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | KEY_STAT
  *
  * @param  AESx AES instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aes_is_action_flag_key_valid(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->STAT, AES_STAT_KEY_STAT) == AES_STAT_KEY_STAT);
}

/**
  * @brief  Indicate whether the done interrupt flag is set.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT            | CPLT_INT_FLAG
  *
  * @param  AESx AES instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aes_is_action_flag_it_done(aes_regs_t *AESx)
{
    return (READ_BITS(AESx->INT, AES_INT_CPLT_INT_FLAG) == AES_INT_CPLT_INT_FLAG);
}

/**
  * @brief  Clear the done interrupt flag.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT            | CPLT_INT_FLAG
  *
  * @param  AESx AES instance
  * @retval None
  */
__STATIC_INLINE void ll_aes_clear_flag_it_done(aes_regs_t *AESx)
{
    SET_BITS(AESx->INT, AES_INT_CPLT_INT_FLAG);
}

/** @} */

/** @defgroup AES_LL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Set AES transfer blocks in DMA mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  XFE_SIZE       | SIZE
  *
  * @param  AESx AES instance
  * @param  block This parameter can be one of the following values: 1 ~ 2048.
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_dma_transfer_block(aes_regs_t *AESx, uint32_t block)
{
    MODIFY_REG(AESx->XFE_SIZE, AES_XFE_SIZE_SIZE, (block << 4) - 1);
}

/**
  * @brief  Get AES transfer blocks in DMA mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  XFE_SIZE       | SIZE
  *
  * @param  AESx AES instance
  * @retval Return value between 1 and 2048.
  */
__STATIC_INLINE uint32_t ll_aes_get_dma_transfer_block(aes_regs_t *AESx)
{
    return ((READ_BITS(AESx->XFE_SIZE, AES_XFE_SIZE_SIZE) + 1) >> 4);
}

/**
  * @brief  Set AES read address of RAM in DMA mode.
  * @note   This read address of RAM requires 4 byte alignment.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RD_START_ADDR  | ADDR
  *
  * @param  AESx AES instance
  * @param  address This parameter can be a address in RAM area (0x30000000 ~ 0x3003FFFF).
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_dma_read_address(aes_regs_t *AESx, uint32_t address)
{
    WRITE_REG(AESx->RD_START_ADDR, address);
}

/**
  * @brief  Get AES read address of RAM in DMA mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RD_START_ADDR  | ADDR
  *
  * @param  AESx AES instance
  * @retval Returned value is the read address in RAM.
  */
__STATIC_INLINE uint32_t ll_aes_get_dma_read_address(aes_regs_t *AESx)
{
    return (READ_REG(AESx->RD_START_ADDR));
}

/**
  * @brief  Set AES write address of RAM in DMA mode.
  * @note   This write address of RAM requires 4 byte alignment.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  WR_START_ADDR  | ADDR
  *
  * @param  AESx AES instance
  * @param  address This parameter can be a address in RAM area (0x30000000 ~ 0x3003FFFF).
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_dma_write_address(aes_regs_t *AESx, uint32_t address)
{
    WRITE_REG(AESx->WR_START_ADDR, address);
}

/**
  * @brief  Get AES write address of RAM in DMA mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  WR_START_ADDR  | ADDR
  *
  * @param  AESx AES instance
  * @retval Returned value is the wrute address in RAM
  */
__STATIC_INLINE uint32_t ll_aes_get_dma_write_address(aes_regs_t *AESx)
{
    return (READ_REG(AESx->WR_START_ADDR));
}

/** @} */

/** @defgroup AES_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Set AES key address in memory.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  KEY_ADDR       | ADDR
  *
  * @param  AESx AES instance
  * @param  address This parameter can be one of the address in RAM
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_key_address(aes_regs_t *AESx, uint32_t address)
{
    WRITE_REG(AESx->KEY_ADDR, address);
}

/**
  * @brief  Get AES key address in memory.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  KEY_ADDR       | ADDR
  *
  * @param  AESx AES instance
  * @retval Returned value is the key address in RAM.
  */
__STATIC_INLINE uint32_t ll_aes_get_key_address(aes_regs_t *AESx)
{
    return (READ_REG(AESx->KEY_ADDR));
}

/**
  * @brief  Get AES output data[127:96].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_OUT0      | DATA
  *
  * @param  AESx AES instance
  * @retval Output Data[127:96]
  */
__STATIC_INLINE uint32_t ll_aes_get_data_127_96(aes_regs_t *AESx)
{
    return (READ_REG(AESx->DATA_OUT0));
}

/**
  * @brief  Get AES output data[95:64].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_OUT1      | DATA
  *
  * @param  AESx AES instance
  * @retval Output Data[95:64]
  */
__STATIC_INLINE uint32_t ll_aes_get_data_95_64(aes_regs_t *AESx)
{
    return (READ_REG(AESx->DATA_OUT1));
}

/**
  * @brief  Get AES output data[63:32].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_OUT2      | DATA
  *
  * @param  AESx AES instance
  * @retval Output Data[63:32]
  */
__STATIC_INLINE uint32_t ll_aes_get_data_63_32(aes_regs_t *AESx)
{
    return (READ_REG(AESx->DATA_OUT2));
}

/**
  * @brief  Get AES output data[31:0].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_OUT3      | DATA
  *
  * @param  AESx AES instance
  * @retval Output Data[31:0]
  */
__STATIC_INLINE uint32_t ll_aes_get_data_31_0(aes_regs_t *AESx)
{
    return (READ_REG(AESx->DATA_OUT3));
}

/**
  * @brief  Set AES key[255:224].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  KEY0           | KEY
  *
  * @param  AESx AES instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_key_255_224(aes_regs_t *AESx, uint32_t key)
{
    WRITE_REG(AESx->KEY0, key);
}

/**
  * @brief  Set AES key[223:192].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  KEY1           | KEY
  *
  * @param  AESx AES instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_key_223_192(aes_regs_t *AESx, uint32_t key)
{
    WRITE_REG(AESx->KEY1, key);
}

/**
  * @brief  Set AES key[191:160].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  KEY2           | KEY
  *
  * @param  AESx AES instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_key_191_160(aes_regs_t *AESx, uint32_t key)
{
    WRITE_REG(AESx->KEY2, key);
}

/**
  * @brief  Set AES key[159:128].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  KEY3           | KEY
  *
  * @param  AESx AES instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_key_159_128(aes_regs_t *AESx, uint32_t key)
{
    WRITE_REG(AESx->KEY3, key);
}

/**
  * @brief  Set AES key[127:96].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  KEY4           | KEY
  *
  * @param  AESx AES instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_key_127_96(aes_regs_t *AESx, uint32_t key)
{
    WRITE_REG(AESx->KEY4, key);
}

/**
  * @brief  Set AES key[95:64].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  KEY5           | KEY
  *
  * @param  AESx AES instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_key_95_64(aes_regs_t *AESx, uint32_t key)
{
    WRITE_REG(AESx->KEY5, key);
}

/**
  * @brief  Set AES key[63:32].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  KEY6           | KEY
  *
  * @param  AESx AES instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_key_63_32(aes_regs_t *AESx, uint32_t key)
{
    WRITE_REG(AESx->KEY6, key);
}

/**
  * @brief  Set AES key[31:0].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  KEY7           | KEY
  *
  * @param  AESx AES instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_key_31_0(aes_regs_t *AESx, uint32_t key)
{
    WRITE_REG(AESx->KEY7, key);
}

/**
  * @brief  Set AES input seed.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INIT_SSI       | SEED_IN
  *
  * @param  AESx AES instance
  * @param  seed This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_seed_in(aes_regs_t *AESx, uint32_t seed)
{
    WRITE_REG(AESx->INIT_SSI, seed);
}

/**
  * @brief  Get AES input seed.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INIT_SSI       | SEED_IN
  *
  * @param  AESx AES instance
  * @retval Returned value is the input seed.
  */
__STATIC_INLINE uint32_t ll_aes_get_seed_in(aes_regs_t *AESx)
{
    return (READ_REG(AESx->INIT_SSI));
}

/**
  * @brief  Set AES output seed.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INIT_SSO       | SEED_OUT
  *
  * @param  AESx AES instance
  * @param  seed This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_seed_out(aes_regs_t *AESx, uint32_t seed)
{
    WRITE_REG(AESx->INIT_SSO, seed);
}

/**
  * @brief  Get AES output seed.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INIT_SSO       | SEED_OUT
  *
  * @param  AESx AES instance
  * @retval Returned value is the output seed.
  */
__STATIC_INLINE uint32_t ll_aes_get_seed_out(aes_regs_t *AESx)
{
    return (READ_REG(AESx->INIT_SSO));
}

/**
  * @brief  Set sbox input data's mask.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  MASK_SSI       | SEED_IMASK
  *
  * @param  AESx AES instance
  * @param  mask This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_seed_Imask(aes_regs_t *AESx, uint32_t mask)
{
    WRITE_REG(AESx->MASK_SSI, mask);
}

/**
  * @brief  Get sbox input data's mask.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  MASK_SSI       | SEED_IMASK
  *
  * @param  AESx AES instance
  * @retval Returned value is the input data's mask.
  */
__STATIC_INLINE uint32_t ll_aes_get_seed_Imask(aes_regs_t *AESx)
{
    return (READ_REG(AESx->MASK_SSI));
}

/**
  * @brief  Set sbox output data's mask.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  MASK_SSO       | SEED_OSBOX
  *
  * @param  AESx AES instance
  * @param  mask This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_seed_Osbox(aes_regs_t *AESx, uint32_t mask)
{
    WRITE_REG(AESx->MASK_SSO, mask);
}

/**
  * @brief  Get sbox output data's mask.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  MASK_SSO       | SEED_OSBOX
  *
  * @param  AESx AES instance
  * @retval Returned value is the output data's mask.
  */
__STATIC_INLINE uint32_t ll_aes_get_seed_Osbox(aes_regs_t *AESx)
{
    return (READ_REG(AESx->MASK_SSO));
}

/**
  * @brief  Set AES initialization vector[127:96].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INIT_V0        | VECTOR_INIT
  *
  * @param  AESx AES instance
  * @param  vector This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_vector_127_96(aes_regs_t *AESx, uint32_t vector)
{
    WRITE_REG(AESx->INIT_V0, vector);
}

/**
  * @brief  Set AES initialization vector[95:64].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INIT_V1        | VECTOR_INIT
  *
  * @param  AESx AES instance
  * @param  vector This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_vector_95_64(aes_regs_t *AESx, uint32_t vector)
{
    WRITE_REG(AESx->INIT_V1, vector);
}

/**
  * @brief  Set AES initialization vector[63:32].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INIT_V2        | VECTOR_INIT
  *
  * @param  AESx AES instance
  * @param  vector This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_vector_63_32(aes_regs_t *AESx, uint32_t vector)
{
    WRITE_REG(AESx->INIT_V2, vector);
}

/**
  * @brief  Set AES initialization vector[31:0].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INIT_V3        | VECTOR_INIT
  *
  * @param  AESx AES instance
  * @param  vector This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_vector_31_0(aes_regs_t *AESx, uint32_t vector)
{
    WRITE_REG(AESx->INIT_V3, vector);
}

/**
  * @brief  Set AES input data[127:96].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_IN0       | DATA_IN
  *
  * @param  AESx AES instance
  * @param  data This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_data_127_96(aes_regs_t *AESx, uint32_t data)
{
    WRITE_REG(AESx->DATA_IN0, data);
}

/**
  * @brief  Set AES input data[95:64].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_IN1       | DATA_IN
  *
  * @param  AESx AES instance
  * @param  data This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_data_95_64(aes_regs_t *AESx, uint32_t data)
{
    WRITE_REG(AESx->DATA_IN1, data);
}

/**
  * @brief  Set AES input data[63:32].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_IN2       | DATA_IN
  *
  * @param  AESx AES instance
  * @param  data This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_data_63_32(aes_regs_t *AESx, uint32_t data)
{
    WRITE_REG(AESx->DATA_IN2, data);
}

/**
  * @brief  Set AES input data[31:0].
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_IN3       | DATA_IN
  *
  * @param  AESx AES instance
  * @param  data This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_data_31_0(aes_regs_t *AESx, uint32_t data)
{
    WRITE_REG(AESx->DATA_IN3, data);
}

/**
  * @brief  Set AES fetch key port mask.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  KEYPORT_MASK   | KPORT_MASK
  *
  * @param  AESx AES instance
  * @param  mask This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_aes_set_key_port_mask(aes_regs_t *AESx, uint32_t mask)
{
    WRITE_REG(AESx->KEYPORT_MASK, mask);
}

/** @} */

/** @defgroup AES_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize AES registers (Registers restored to their default values).
  * @param  AESx        AES Instance
  * @retval An error_status_t  enumeration value:
  *          - SUCCESS: AES registers are de-initialized
  *          - ERROR: AES registers are not de-initialized
  */
error_status_t ll_aes_deinit(aes_regs_t *AESx);

/**
  * @brief  Initialize AES registers according to the specified
  *         parameters in p_aes_init.
  * @param  AESx        AES Instance
  * @param  p_aes_init    Pointer to a ll_aes_init_t structure that contains the configuration
  *                     information for the specified AES peripheral.
  * @retval An error_status_t  enumeration value:
  *          - SUCCESS: AES registers are initialized according to p_aes_init content
  *          - ERROR: Problem occurred during AES Registers initialization
  */
error_status_t ll_aes_init(aes_regs_t *AESx, ll_aes_init_t *p_aes_init);

/**
  * @brief Set each field of a @ref ll_aes_init_t type structure to default value.
  * @param p_aes_init     Pointer to a @ref ll_aes_init_t structure
  *                     whose fields will be set to default values.
  * @retval None
  */
void ll_aes_struct_init(ll_aes_init_t *p_aes_init);

/** @} */

/** @} */

#endif /* AES */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_AES_H__ */

/** @} */

/** @} */

/** @} */

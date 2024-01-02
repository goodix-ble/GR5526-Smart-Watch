/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_rng.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of RNG LL library.
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

/** @defgroup LL_RNG RNG
  * @brief RNG LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_RNG_H__
#define __GR55XX_LL_RNG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (RNG)

/** @defgroup RNG_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup RNG_LL_ES_INIT RNG Exported Init structures
  * @{
  */

/**
  * @brief LL RNG Init Structure definition
  */
typedef struct _ll_rng_init
{
    uint32_t seed;            /**< Specifies the seed source for the LFSR.
                                 This parameter can be a value of @ref RNG_LL_EC_SEED_SOURCE */

    uint32_t lfsr_mode;       /**< Specifies the configuration mode for the LFSR.
                                 This parameter can be a value of @ref RNG_LL_EC_LFSR_MODE */

    uint32_t out_mode;        /**< Specifies the Output mode for the RNG.
                                 This parameter can be a value of @ref RNG_LL_EC_OUTPUT_MODE */

    uint32_t post_mode;       /**< Specifies post-process configuration for the RNG.
                                 This parameter can be a value of @ref RNG_LL_EC_POST_PRO */

    uint32_t interrupt;       /**< Specifies interrupt configuration for the RNG.
                                 This parameter can be a value of @ref RNG_LL_EC_IT */

} ll_rng_init_t;

/** @} */

/** @} */

/**
  * @defgroup  RNG_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RNG_LL_Exported_Constants RNG Exported Constants
  * @{
  */

/** @defgroup RNG_LL_EC_SEED_SOURCE LFSR seed source
  * @{
  */
#define LL_RNG_SEED_FR0_S0                 (4UL << RNG_CONFIG_LFSR_SEED_SEL_Pos)  /**< LFSR seed is from the switching oscillator s0. */
#define LL_RNG_SEED_USER                   (6UL << RNG_CONFIG_LFSR_SEED_SEL_Pos)  /**< LFSR seed is configured by users. */
/** @} */


/** @defgroup RNG_LL_EC_LFSR_MODE LFSR configuration mode
  * @{
  */
#define LL_RNG_LFSR_MODE_59BIT             (0x00000000UL)                         /**< 59 bit LFSR.  */
#define LL_RNG_LFSR_MODE_128BIT            (1UL << RNG_CONFIG_LFSR_MODE_Pos)      /**< 128 bit LFSR. */
/** @} */

/** @defgroup RNG_LL_EC_POST_PRO Post-process mode
  * @{
  */
#define LL_RNG_POST_PRO_NOT                (0x00000000UL)                         /**< No post process. */
#define LL_RNG_POST_PRO_SKIPPING           (1UL << RNG_CONFIG_POST_MODE_Pos)      /**< bit skipping.    */
#define LL_RNG_POST_PRO_COUNTING           (2UL << RNG_CONFIG_POST_MODE_Pos)      /**< bit counting.    */
#define LL_RNG_POST_PRO_NEUMANN            (3UL << RNG_CONFIG_POST_MODE_Pos)      /**< Von-Neumann.     */
/** @} */

/** @defgroup RNG_LL_EC_IT RNG hardware interrupt enable.
  * @{
  */
#define LL_RNG_IT_DISABLE                  (0x00000000UL)                         /**< Disable RNG interrupt.  */
#define LL_RNG_IT_ENABLE                   (1UL << RNG_CONFIG_IRQ_EN_Pos)         /**< Enable RNG interrupt.   */
/** @} */

/** @defgroup RNG_LL_EC_OUTPUT_MODE RNG Output mode
  * @{
  */
#define LL_RNG_OUTPUT_FR0_S0               (4UL << RNG_CONFIG_OUT_MODE_Pos)       /**< Digital RNG direct output, ring oscillator s0. */
#define LL_RNG_OUTPUT_CYCLIC_PARITY        (6UL << RNG_CONFIG_OUT_MODE_Pos)       /**< LFSR and RNG cyclic sampling and parity generation. */
#define LL_RNG_OUTPUT_CYCLIC               (7UL << RNG_CONFIG_OUT_MODE_Pos)       /**< LFSR and RNG cyclic sampling. */
#define LL_RNG_OUTPUT_LFSR_RNG             (8UL << RNG_CONFIG_OUT_MODE_Pos)       /**< LFSR ⊕ RNG. */
#define LL_RNG_OUTPUT_LFSR                 (9UL << RNG_CONFIG_OUT_MODE_Pos)       /**< LFSR direct output. */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup RNG_LL_Exported_Macros RNG Exported Macros
  * @{
  */

/** @defgroup RNG_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in RNG register
  * @param  __instance__ RNG instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None.
  */
#define LL_RNG_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in RNG register
  * @param  __instance__ RNG instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_RNG_ReadReg(__instance__, __REG__) READ_REG(__instance__->__REG__)
/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup RNG_LL_Exported_Functions RNG Exported Functions
  * @{
  */

/** @defgroup RNG_LL_EF_Configuration RNG Configuration functions
  * @{
  */

/**
  * @brief  Enable Random Number Generation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | RNG_RUN
  *
  * @param  RNGx RNG instance.
  * @retval None
  */
__STATIC_INLINE void ll_rng_enable(rng_regs_t *RNGx)
{
    SET_BITS(RNGx->CTRL, RNG_CTRL_RUN_EN);
}

/**
  * @brief  Disable Random Number Generation.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | RNG_RUN
  *
  * @param  RNGx RNG instance.
  * @retval None
  */
__STATIC_INLINE void ll_rng_disable(rng_regs_t *RNGx)
{
    CLEAR_BITS(RNGx->CTRL, RNG_CTRL_RUN_EN);
}

/**
  * @brief  Check if Random Number Generator is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | RNG_RUN
  *
  * @param  RNGx RNG instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_rng_is_enabled(rng_regs_t *RNGx)
{
    return (READ_BITS(RNGx->CTRL, RNG_CTRL_RUN_EN) == (RNG_CTRL_RUN_EN));
}

/**
  * @brief  Enable Ring oscillator TRNG enabled signal.
  *
  *  Register|BitsName
  *  --------|--------
  *  CONFIG  | RNG_FRO_EN
  *
  * @param  RNGx RNG instance.
  * @retval None
  */
__STATIC_INLINE void ll_rng_enable_fro(rng_regs_t *RNGx)
{
    SET_BITS(RNGx->CONFIG, RNG_CONFIG_FRO_EN);
}

/**
  * @brief  Disable Ring oscillator TRNG enabled signal.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | RNG_RUN
  *
  * @param  RNGx RNG instance.
  * @retval None
  */
__STATIC_INLINE void ll_rng_disable_fro(rng_regs_t *RNGx)
{
    CLEAR_BITS(RNGx->CONFIG, RNG_CONFIG_FRO_EN);
}

/**
  * @brief  Check if Ring oscillator TRNG enabled signal is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | RNG_RUN
  *
  * @param  RNGx RNG instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_rng_fro_is_enabled(rng_regs_t *RNGx)
{
    return (READ_BITS(RNGx->CONFIG, RNG_CONFIG_FRO_EN) == (RNG_CONFIG_FRO_EN));
}

/**
  * @brief  Set source of LFSR seed.
  *
  *  Register|BitsName
  *  --------|--------
  *  CONFIG  | RNG_LFSR_SEED_SEL
  *
  * @param  RNGx RNG instance.
  * @param  seed This parameter can be one of the following values:
  *         @arg @ref LL_RNG_SEED_FR0_S0
  *         @arg @ref LL_RNG_SEED_USER
  * @retval None
  */
__STATIC_INLINE void ll_rng_set_lfsr_seed(rng_regs_t *RNGx, uint32_t seed)
{
    MODIFY_REG(RNGx->CONFIG, RNG_CONFIG_LFSR_SEED_SEL, seed);
}

/**
  * @brief  Get source of LFSR seed.
  *
  *  Register|BitsName
  *  --------|--------
  *  CONFIG  | RNG_LFSR_SEED_SEL
  *
  * @param  RNGx RNG instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RNG_SEED_FR0_S0
  *         @arg @ref LL_RNG_SEED_USER
  */
__STATIC_INLINE uint32_t ll_rng_get_lfsr_seed(rng_regs_t *RNGx)
{
    return READ_BITS(RNGx->CONFIG, RNG_CONFIG_LFSR_SEED_SEL);
}

/**
  * @brief  Set LFSR configuration mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CONFIG  | RNG_LFSR_MODE
  *
  * @param  RNGx RNG instance.
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_RNG_LFSR_MODE_59BIT
  *         @arg @ref LL_RNG_LFSR_MODE_128BIT
  * @retval None
  */
__STATIC_INLINE void ll_rng_set_lfsr_mode(rng_regs_t *RNGx, uint32_t mode)
{
    MODIFY_REG(RNGx->CONFIG, RNG_CONFIG_LFSR_MODE, mode);
}

/**
  * @brief  Get LFSR configuration mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CONFIG  | RNG_LFSR_MODE
  *
  * @param  RNGx RNG instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RNG_LFSR_MODE_59BIT
  *         @arg @ref LL_RNG_LFSR_MODE_128BIT
  */
__STATIC_INLINE uint32_t ll_rng_get_lfsr_mode(rng_regs_t *RNGx)
{
    return READ_BITS(RNGx->CONFIG, RNG_CONFIG_LFSR_MODE);
}

/**
  * @brief  Set RNG post-process configuration.
  *
  *  Register|BitsName
  *  --------|--------
  *  CONFIG  | RNG_POST_MODE
  *
  * @param  RNGx RNG instance.
  * @param  post This parameter can be one of the following values:
  *         @arg @ref LL_RNG_POST_PRO_NOT
  *         @arg @ref LL_RNG_POST_PRO_SKIPPING
  *         @arg @ref LL_RNG_POST_PRO_COUNTING
  *         @arg @ref LL_RNG_POST_PRO_NEUMANN
  * @retval None
  */
__STATIC_INLINE void ll_rng_set_post_mode(rng_regs_t *RNGx, uint32_t post)
{
    MODIFY_REG(RNGx->CONFIG, RNG_CONFIG_POST_MODE, post);
}

/**
  * @brief  Get RNG post-process configuration.
  *
  *  Register|BitsName
  *  --------|--------
  *  CONFIG  | RNG_POST_MODE
  *
  * @param  RNGx RNG instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RNG_POST_PRO_NOT
  *         @arg @ref LL_RNG_POST_PRO_SKIPPING
  *         @arg @ref LL_RNG_POST_PRO_COUNTING
  *         @arg @ref LL_RNG_POST_PRO_NEUMANN
  */
__STATIC_INLINE uint32_t ll_rng_get_post_mode(rng_regs_t *RNGx)
{
    return READ_BITS(RNGx->CONFIG, RNG_CONFIG_POST_MODE);
}

/**
  * @brief  set RNG output mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CONFIG  | RNG_OUT_MODE
  *
  * @param  RNGx RNG instance.
   * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_RNG_OUTPUT_FR0_S0
  *         @arg @ref LL_RNG_OUTPUT_CYCLIC_PARITY
  *         @arg @ref LL_RNG_OUTPUT_CYCLIC
  *         @arg @ref LL_RNG_OUTPUT_LFSR_RNG
  *         @arg @ref LL_RNG_OUTPUT_LFSR
  * @retval None
  */
__STATIC_INLINE void ll_rng_set_output_mode(rng_regs_t *RNGx, uint32_t mode)
{
    MODIFY_REG(RNGx->CONFIG, RNG_CONFIG_OUT_MODE, mode);
}

/**
  * @brief  get RNG output mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CONFIG  | RNG_OUT_MODE
  *
  * @param  RNGx RNG instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RNG_OUTPUT_FR0_S0
  *         @arg @ref LL_RNG_OUTPUT_CYCLIC_PARITY
  *         @arg @ref LL_RNG_OUTPUT_CYCLIC
  *         @arg @ref LL_RNG_OUTPUT_LFSR_RNG
  *         @arg @ref LL_RNG_OUTPUT_LFSR
  */
__STATIC_INLINE uint32_t ll_rng_get_output_mode(rng_regs_t *RNGx)
{
    return READ_BITS(RNGx->CONFIG, RNG_CONFIG_OUT_MODE);
}

/**
  * @brief  set the waiting time that RNG input reaches stable.
  *
  *  Register|BitsName
  *  --------|--------
  *  TSCON   | RNG_TRDY_TIME
  *
  * @param  RNGx RNG instance.
  * @param  time range between 0x1 and 0xFF.
  * @retval None
  */
__STATIC_INLINE void ll_rng_set_trdy_time(rng_regs_t *RNGx, uint32_t time)
{
    MODIFY_REG(RNGx->TSCON, RNG_TSCON_TRDY_TIME, time);
}

/**
  * @brief  get the waiting time that RNG input reaches stable.
  *
  *  Register|BitsName
  *  --------|--------
  *  TSCON   | RNG_TRDY_TIME
  *
  * @param  RNGx RNG instance.
  * @retval Between Min_Time = 0 and Max_Time = 0xFF
  */
__STATIC_INLINE uint32_t ll_rng_get_trdy_time(rng_regs_t *RNGx)
{
    return READ_BITS(RNGx->TSCON, RNG_TSCON_TRDY_TIME);
}


/**
  * @brief  set RNG seed configured by user.
  *
  *  Register|BitsName
  *  --------|--------
  *  USER    | RNG_USER_SEED
  *
  * @param  RNGx RNG instance.
  * @param  seed range between 0x1 and 0xFFFF.
  * @retval None
  */
__STATIC_INLINE void ll_rng_set_user_seed(rng_regs_t *RNGx, uint32_t seed)
{
    WRITE_REG(RNGx->USER_SEED, seed);
}

/** @} */

/** @defgroup RNG_LL_EF_FLAG_Management FLAG Management
  * @{
  */

/**
  * @brief  Indicate if the Flag of RNG long run test is set or not.
  *
  *  Register |BitsName
  *  ---------|--------
  *  LR_STATUS| RNG_LR_FLAG
  *
  * @param  RNGx RNG instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_rng_is_active_flag_lr(rng_regs_t *RNGx)
{
    return (READ_BITS(RNGx->LR_STATUS, RNG_LR_STATUS_FLAG) == (RNG_LR_STATUS_FLAG));
}

/**
  * @brief  Indicate if the RNG Status Flag is set or not.
  *
  *  Register|BitsName
  *  --------|--------
  *  STATUS  | RNG_READY
  *
  * @param  RNGx RNG instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_rng_is_active_flag_sts(rng_regs_t *RNGx)
{
    return (READ_BITS(RNGx->STATUS, RNG_STATUS_READY) == (RNG_STATUS_READY));
}

/**
  * @brief  Clear RNG Status flag.
  *
  *  Register|BitsName
  *  --------|--------
  *  STATUS  | RNG_READY
  *
  * @param  RNGx RNG instance.
  * @retval None
  */
__STATIC_INLINE void ll_rng_clear_flag_sts(rng_regs_t *RNGx)
{
    WRITE_REG(RNGx->STATUS, RNG_STATUS_READY);
}

/** @} */

/** @defgroup RNG_LL_EF_IT_Management IT Management
  * @{
  */

/**
  * @brief  Enable Random Number Generator Interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  CONFIG  | RNG_IRQ_EN
  *
  * @param  RNGx RNG instance.
  * @retval None
  */
__STATIC_INLINE void ll_rng_enable_it(rng_regs_t *RNGx)
{
    SET_BITS(RNGx->CONFIG, RNG_CONFIG_IRQ_EN);
}

/**
  * @brief  Disable Random Number Generator Interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  CONFIG  | RNG_IRQ_EN
  *
  * @param  RNGx RNG instance.
  * @retval None
  */
__STATIC_INLINE void ll_rng_disable_it(rng_regs_t *RNGx)
{
    CLEAR_BITS(RNGx->CONFIG, RNG_CONFIG_IRQ_EN);
}

/**
  * @brief  Check if Random Number Generator Interrupt is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CONFIG  | RNG_IRQ_EN
  *
  * @param  RNGx RNG instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_rng_is_enabled_it(rng_regs_t *RNGx)
{
    return (READ_BITS(RNGx->CONFIG, RNG_CONFIG_IRQ_EN) == (RNG_CONFIG_IRQ_EN));
}

/** @} */

/** @defgroup RNG_LL_EF_Data_Management Data Management
  * @{
  */

/**
  * @brief  Return32-bit Random Number value
  *
  *  Register|BitsName
  *  --------|--------
  *  DATA    | RNG_DATA
  *
  * @param  RNGx RNG instance.
  * @retval Generated 32-bit random value
  */
__STATIC_INLINE uint32_t ll_rng_read_random_data32(rng_regs_t *RNGx)
{
    return (uint32_t)(READ_REG(RNGx->DATA));
}

/**
  * @brief  Return8-bit RNG Long Run Test counts.
  *
  *  Register |BitsName
  *  ---------|--------
  *  LR_STATUS| RNG_LR_CNT
  *
  * @param  RNGx RNG instance.
  * @retval Output Data[7:0]
  */
__STATIC_INLINE uint32_t ll_rng_read_lr_count(rng_regs_t *RNGx)
{
    return READ_BITS(RNGx->LR_STATUS, RNG_LR_STATUS_CNT) >> RNG_LR_STATUS_CNT_Pos;
}

/** @} */

/** @defgroup RNG_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize the RNG registers to their default reset values.
  * @param  RNGx RNG instance.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: RNG registers are de-initialized
  *          - ERROR: RNG registers are not de-initialized
  */
error_status_t ll_rng_deinit(rng_regs_t *RNGx);

/**
  * @brief  Initialize RNG registers according to the specified
  *         parameters in p_rng_init.
  * @param  RNGx        RNG Instance
  * @param  p_rng_init    Pointer to a ll_rng_init_t structure that contains the configuration
  *                     information for the specified RNG peripheral.
  * @retval An error_status_t  enumeration value:
  *          - SUCCESS: RNG registers are initialized according to p_rng_init content
  *          - ERROR: Problem occurred during RNG Registers initialization
  */
error_status_t ll_rng_init(rng_regs_t *RNGx, ll_rng_init_t *p_rng_init);

/**
  * @brief Set each field of a @ref ll_rng_init_t type structure to default value.
  * @param p_rng_init     Pointer to a @ref ll_rng_init_t structure
  *                     whose fields will be set to default values.
  * @retval None
  */
void ll_rng_struct_init(ll_rng_init_t *p_rng_init);

/** @} */

/** @} */

#endif /* RNG */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_RNG_H__ */

/** @} */

/** @} */

/** @} */

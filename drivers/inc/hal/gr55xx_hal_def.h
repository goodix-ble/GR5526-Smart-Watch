/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_def.h
 * @author  BLE Driver Team
 * @brief   This file contains HAL common definitions, enumeration, macros and structures definitions.
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

/** @defgroup HAL_DEF HAL DEFINE
  * @brief HAL common definitions.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_DEF__
#define __GR55xx_HAL_DEF__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_ENUMERATIONS Enumerations
 * @{ */
/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
    HAL_OK       = 0x00U,    /**< Operation is OK. */
    HAL_ERROR    = 0x01U,    /**< Parameter error or operation is not supported. */
    HAL_BUSY     = 0x02U,    /**< Driver is busy. */
    HAL_TIMEOUT  = 0x03      /**< Timeout occurred. */
} hal_status_t;

/**
  * @brief  HAL Lock structures definition
  */
typedef enum
{
    HAL_UNLOCKED = 0x00U,    /**< Object is unlocked. */
    HAL_LOCKED   = 0x01      /**< Object is locked. */
} hal_lock_t;
/** @} */

/**
  * @defgroup  HAL_DEF_MACRO Defines
  * @{
  */

/* Exported macro ------------------------------------------------------------*/
/**
  * @brief  HAL never timeout definition.
  */
#define HAL_NEVER_TIMEOUT                   (0xFFFFFFFFU)

/**
  * @brief  HAL max delay definition. Unit is millisecond.
  *         40000ms is the max delay time in 96MHz system clock.
  */
#define HAL_MAX_DELAY                       (40000U)

/**
  * @brief  Timeout module init. This macro must be used in
  *         conjunction with the @ref HAL_TIMEOUT_DEINIT macro
  */
#define HAL_TIMEOUT_INIT()                                               \
    uint32_t _demcr_initial = CoreDebug->DEMCR;                          \
    uint32_t _dwt_ctrl_initial = DWT->CTRL;                              \
do {                                                                     \
    hal_dwt_enable(_demcr_initial, _dwt_ctrl_initial);                   \
} while (0)

/**
  * @brief  Timeout module deinit. This macro must be used in
  *         conjunction with the @ref HAL_TIMEOUT_INIT macro
  */
#define HAL_TIMEOUT_DEINIT()                                             \
do {                                                                     \
    hal_dwt_disable(_demcr_initial, _dwt_ctrl_initial);                  \
} while(0)

/**
  * @brief  Timeout module get current tick.
  * @retval Current tick
  */
#define HAL_TIMEOUT_GET_TICK()             (DWT->CYCCNT)

/**
  * @brief  Check whether the bits of register are set.
  * @param  REG specifies the register.
  * @param  BIT specifies the bits will be checked.
  * @retval SET (BIT is set) or RESET (BIT is not set)
  */
#define HAL_IS_BIT_SET(REG, BIT)            (((REG) & (BIT)) != RESET)
/**
  * @brief  Check whether the bits of register are clear.
  * @param  REG specifies the register.
  * @param  BIT specifies the bits will be checked.
  * @retval SET (BIT is clear) or RESET (BIT is not clear)
  */
#define HAL_IS_BIT_CLR(REG, BIT)            (((REG) & (BIT)) == RESET)

/**
  * @brief  Link DMA handle and peripheral handle.
  * @param  __HANDLE__ specifies the peripheral handle.
  * @param  __PPP_DMA_FIELD_ specifies the DMA pointer in struction of peripheral handle.
  * @param  __DMA_HANDLE_ specifies the DMA handle.
  * @retval None
  */
#define __HAL_LINKDMA(__HANDLE__, __PPP_DMA_FIELD_, __DMA_HANDLE_)                 \
                        do{                                                        \
                              (__HANDLE__)->__PPP_DMA_FIELD_ = &(__DMA_HANDLE_);   \
                              (__DMA_HANDLE_).p_parent = (__HANDLE__);             \
                          } while(0U)

/** @brief Reset the Handle's State field.
  * @param __HANDLE__ specifies the Peripheral Handle.
  * @note  This macro can be used for the following purposes:
  *          - When the Handle is declared as local variable; before passing it as parameter
  *            to hal_ppp_init() for the first time, it is mandatory to use this macro
  *            to set the Handle's "State" field to 0.
  *            Otherwise, "State" field may have any random value and the first time the function
  *            hal_ppp_init() is called, the low level hardware initialization will be missed
  *            (i.e. hal_ppp_msp_init() will not be executed).
  *          - When there is a need to reconfigure the low level hardware: instead of calling
  *            hal_ppp_deinit() then hal_ppp_init(), user can make a call to this macro then hal_ppp_init().
  *            In this later function, when the Handle's "State" field is set to 0, it will execute the function
  *            hal_ppp_msp_init which will reconfigure the low level hardware.
  * @retval None
  */
#define __HAL_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->state = 0U)

#if (USE_RTOS == 1U)
#error " USE_RTOS should be 0 in the current HAL release "
#else
/**
  * @brief  Lock peripheral handle.
  * @param  __HANDLE__ specifies the peripheral handle.
  * @retval HAL_BUSY If handle is locked.
  */
#define __HAL_LOCK(__HANDLE__)                                              \
                                do{                                         \
                                    if((__HANDLE__)->lock == HAL_LOCKED)    \
                                    {                                       \
                                       return HAL_BUSY;                     \
                                    }                                       \
                                    else                                    \
                                    {                                       \
                                       (__HANDLE__)->lock = HAL_LOCKED;     \
                                    }                                       \
                                  }while (0U)

/**
  * @brief  Unlock peripheral handle.
  * @param  __HANDLE__ specifies the peripheral handle.
  * @retval None
  */
#define __HAL_UNLOCK(__HANDLE__)                                            \
                                  do{                                       \
                                      (__HANDLE__)->lock = HAL_UNLOCKED;    \
                                    }while (0U)
#endif /* USE_RTOS */


#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
#ifndef __weak
#define __weak   __attribute__((weak))
#endif /* __weak */
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif /* __packed */
#endif /* __GNUC__ */


/* Macro to get variable aligned on 4-bytes, for __ICCARM__ the directive "#pragma data_alignment=4" must be used instead */
#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
#ifndef __ALIGN_END
#define __ALIGN_END    __attribute__((aligned(4)))
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif /* __ALIGN_BEGIN */
#else
#ifndef __ALIGN_END
#define __ALIGN_END             /**< ALIGN END */
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#if defined(__CC_ARM)      /* ARM Compiler */
#define __ALIGN_BEGIN    __align(4)
#elif defined(__ICCARM__)    /* IAR Compiler */
#define __ALIGN_BEGIN
#endif /* __CC_ARM */
#endif /* __ALIGN_BEGIN */
#endif /* __GNUC__ */

/**
  * @brief  __NOINLINE definition
  */
#if defined(__CC_ARM) || defined(__GNUC__)
/* ARM & GNUCompiler
   ----------------
*/
#define __NOINLINE __attribute__((noinline))

#elif defined(__ICCARM__)
/* ICCARM Compiler
   ---------------
*/
#define __NOINLINE _Pragma("optimize = no_inline")

#endif

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ___GR55xx_HAL_DEF__ */
/** @} */

/** @} */

/** @} */

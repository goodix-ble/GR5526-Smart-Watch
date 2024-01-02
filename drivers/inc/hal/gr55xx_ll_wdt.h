/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_wdt.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of WDT LL library.
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

/** @defgroup LL_WDT WDT
  * @brief WDT LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_WDT_H__
#define __GR55XX_LL_WDT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (WDT)

/**
  * @defgroup  WDT_LL_MACRO Defines
  * @{
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup WDT_LL_Private_Constants WDT Private Constants
  * @{
  */

/** @defgroup WDT_LL_PC_WR_ACCESS Write Access Defines
  * @{
  */
#define LL_WDT_LOCK_WR_ACCESS_ENABLE       0x1ACCE551               /**< WDT LOCK Write Access Enable  */
#define LL_WDT_LOCK_WR_ACCESS_DISABLE      (~0x1ACCE551)            /**< WDT LOCK Write Access Disable */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup WDT_LL_Exported_Macros WDT Exported Macros
  * @{
  */

/** @defgroup WDT_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in WDT register
  * @param  __instance__ WDT instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_WDT_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in WDT register
  * @param  __instance__ WDT instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_WDT_ReadReg(__instance__, __REG__) READ_REG(__instance__->__REG__)
/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup WDT_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup WDT_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable write access to WDT_LOAD, WDT_CTRL and WDT_INTCLR registers.
  *
  *  Register|BitsName
  *  --------|--------
  *  LOCK | ENRW
  *
  * @param  WDTx WDT instance
  * @retval None
  */
__STATIC_INLINE void ll_wdt_enable_write_access(wdt_regs_t *WDTx)
{
    WRITE_REG(WDTx->LOCK, LL_WDT_LOCK_WR_ACCESS_ENABLE);
}

/**
  * @brief  Disable write access to WDT_LOAD, WDT_CTRL and WDT_INTCLR registers.
  *
  *  Register|BitsName
  *  --------|--------
  *  LOCK | ENRW
  *
  * @param  WDTx WDT instance
  * @retval None
  */
__STATIC_INLINE void ll_wdt_disable_write_access(wdt_regs_t *WDTx)
{
    WRITE_REG(WDTx->LOCK, LL_WDT_LOCK_WR_ACCESS_DISABLE);
}

/**
  * @brief  Enable watchdog counter and interrupt event.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | INTEN
  *
  * @param  WDTx WDT instance.
  * @retval None
  */
__STATIC_INLINE void ll_wdt_enable(wdt_regs_t *WDTx)
{
    SET_BITS(WDTx->CTRL, WDT_CTRL_INTEN);
}

/**
  * @brief  Disable watchdog counter and interrupt event.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | INTEN
  *
  * @param  WDTx WDT instance.
  * @retval None
  */
__STATIC_INLINE void ll_wdt_disable(wdt_regs_t *WDTx)
{
    CLEAR_BITS(WDTx->CTRL, WDT_CTRL_INTEN);
}

/**
  * @brief  Check if the WDT peripheral is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | INTEN
  *
  * @param  WDTx WDT instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_wdt_is_enabled(wdt_regs_t *WDTx)
{
    return (READ_BITS(WDTx->CTRL, WDT_CTRL_INTEN) == (WDT_CTRL_INTEN));
}

/**
  * @brief  Enable reset output.
  * @note   RSTEN acts as a mask for the reset output.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | RSTEN
  *
  * @param  WDTx WDT instance.
  * @retval None
  */
__STATIC_INLINE void ll_wdt_enable_reset(wdt_regs_t *WDTx)
{
    SET_BITS(WDTx->CTRL, WDT_CTRL_RSTEN);
}

/**
  * @brief  Disable reset output.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | RSTEN
  *
  * @param  WDTx WDT instance.
  * @retval None
  */
__STATIC_INLINE void ll_wdt_disable_reset(wdt_regs_t *WDTx)
{
    CLEAR_BITS(WDTx->CTRL, WDT_CTRL_RSTEN);
}

/**
  * @brief  Check if the WDT reset is enabled or disabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | RSTEN
  *
  * @param  WDTx WDT instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_wdt_is_enabled_reset(wdt_regs_t *WDTx)
{
    return (READ_BITS(WDTx->CTRL, WDT_CTRL_RSTEN) == (WDT_CTRL_RSTEN));
}

/**
  * @brief  Specify the WDT down-counter reload value.
  *
  *  Register|BitsName
  *  --------|--------
  *  LOAD | LOAD
  *
  * @param  WDTx WDT instance
  * @param  counter Value range between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_wdt_set_counter_load(wdt_regs_t *WDTx, uint32_t counter)
{
    WRITE_REG(WDTx->LOAD, counter);
}

/**
  * @brief  Get the specified WDT down-counter reload value.
  *
  *  Register|BitsName
  *  --------|--------
  *  LOAD | LOAD
  *
  * @param  WDTx WDT instance
  * @retval Value range between Min_Data=0 and Max_Data=0x0FFF
  */
__STATIC_INLINE uint32_t ll_wdt_get_counter_load(wdt_regs_t *WDTx)
{
    return (uint32_t)(READ_REG(WDTx->LOAD));
}

/**
  * @brief  Get current value of the specified WDT decrementing down-counter.
  *
  *  Register|BitsName
  *  --------|--------
  *  VALUE | VALUE
  *
  * @param  WDTx WDT instance
  * @retval Value range between Min_Data=0 and Max_Data=0x0FFF
  */
__STATIC_INLINE uint32_t ll_wdt_get_counter_value(wdt_regs_t *WDTx)
{
    return (uint32_t)(READ_REG(WDTx->VALUE));
}

/**
  * @brief  Reloads WDT counter with value defined in the reload register
  *
  *  Register|BitsName
  *  --------|--------
  *  INTCLR | INTCLR
  *
  * @param  WDTx WDT instance
  * @retval None
  */
__STATIC_INLINE void ll_wdt_reload_counter(wdt_regs_t *WDTx)
{
    WRITE_REG(WDTx->INTCLR, WDT_INTCLR);
}

/** @} */

/** @defgroup WDT_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Indicate if the WDT Interrupt Flag is set or not.
  * @note   This bit is set by hardware when the counter has reached 0. It can
  *         be cleared by software by writing any value to the INTCLR Register.
  *
  *  Register|BitsName
  *  --------|--------
  *  MIS | INTSTAT
  *
  * @param  WDTx WDT instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_wdt_is_active_flag_it(wdt_regs_t *WDTx)
{
    return (READ_BITS(WDTx->MIS, WDT_MIS_INTSTAT) == (WDT_MIS_INTSTAT));
}

/**
  * @brief  Clear Interrupt Status flag.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTCLR| INTCLR
  *
  * @param  WDTx WDT instance.
  * @retval None
  */
__STATIC_INLINE void ll_wdt_clear_flag_it(wdt_regs_t *WDTx)
{
    WRITE_REG(WDTx->INTCLR, WDT_INTCLR);
}

/** @} */

/** @} */

#endif /* WDT */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_WDT_H__ */

/** @} */

/** @} */

/** @} */

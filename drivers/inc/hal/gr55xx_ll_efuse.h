/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_efuse.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of eFuse LL library.
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

/** @defgroup LL_EFUSE EFUSE
  * @brief eFuse LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_EFUSE_H__
#define __GR55xx_LL_EFUSE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (EFUSE)

/**
  * @defgroup  EFUSE_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup EFUSE_LL_Exported_Constants EFUSE Exported Constants
  * @{
  */

/** @defgroup EFUSE_LL_EC_OPERATION EFUSE Operation Defines
  * @brief    Operation defines which can be used with LL_EFUSE_WriteReg function
  * @{
  */
#define LL_EFUSE_WRITE_KEYRAM               EFUSE_OPER_WRITE_KEYRAM         /**< Read fwkay and rootkey from eFuse, and write to keyram  */
#define LL_EFUSE_READ_TRIM                  EFUSE_OPER_READ_TRIM            /**< Read analog trim from eFuse                             */
#define LL_EFUSE_CRC_CHECK                  EFUSE_OPER_CRC_CHECK            /**< Read the special eFuse addr, and calculate CRC value    */
#define LL_EFUSE_INIT_CHECK                 EFUSE_OPER_INIT_CHECK           /**< Read the whole eFuse value, and check this value with 0 */
#define LL_EFUSE_TEST_READ                  EFUSE_OPER_RD_TEST_MODE         /**< Read eFuse test mode from eFuse                         */
/** @} */

/** @defgroup EFUSE_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_EFUSE_ReadReg function
  * @{
  */
#define LL_EFUSE_WRITE_KEYRAM_BUSY          EFUSE_STATUS_WRITE_KEYRAM_BUSY      /**< Write keyram operation is in processing  */
#define LL_EFUSE_READ_TRIM_DONE             EFUSE_STATUS_READ_TRIM_DONE         /**< Read trim from eFuse has done            */
#define LL_EFUSE_CRC_CHECK_DONE             EFUSE_STATUS_CRC_CHECK_DONE         /**< eFuse CRC check done                     */
#define LL_EFUSE_CRC_CHECK_SUCCESS          EFUSE_STATUS_TRIM_CRC_SUCCESS       /**< CRC check success                        */
#define LL_EFUSE_INIT_CHECK_DONE            EFUSE_STATUS_INIT_DONE              /**< eFuse initial value check done           */
#define LL_EFUSE_INIT_CHECK_SUCCESS         EFUSE_STATUS_INIT_SUCCESS           /**< eFuse initial value check success        */
#define LL_EFUSE_WRITE_DONE                 EFUSE_STATUS_WRITE_DONE             /**< eFuse one word write done                */
#define LL_EFUSE_TEST_DONE                  EFUSE_STATUS_TEST_MODE_DONE         /**< Read from eFuse has done in test mode    */
/** @} */

/** @defgroup EFUSE_LL_EC_GET_CTL_FLAG Get Power Controller Flags Defines
  * @brief    Flags defines which can be used with LL_EFUSE_ReadReg function
  * @{
  */
#define LL_EFUSE_PWR_CTL_EN_DONE          MCU_SUB_EFUSE_PWR_CTL0_EN_DONE        /**< eFuse power enable done  */
#define LL_EFUSE_PWR_CTL_DIS_DONE         MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE       /**< eFuse power disable done            */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup EFUSE_LL_Exported_Macros EFUSE Exported Macros
  * @{
  */

/** @defgroup EFUSE_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in eFuse register
  * @param  __instance__ eFuse instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_EFUSE_WriteReg(__instance__, __REG__, __VALUE__)     WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in eFuse register
  * @param  __instance__ eFuse instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_EFUSE_ReadReg(__instance__, __REG__)                 READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup EFUSE_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup EFUSE_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Set eFuse program time
  *
  *  Register|BitsName
  *  --------|--------
  *  TPGM    | TIME
  *
  * @param  EFUSEx eFuse instance
  * @param  time   This parameter can be one of the following values: 0 ~ 0xFFF
  * @retval None
  */
__STATIC_INLINE void ll_efuse_set_tpro(efuse_regs_t *EFUSEx, uint32_t time)
{
    MODIFY_REG(EFUSEx->TPGM, EFUSE_TPGM_TIME, time << EFUSE_TPGM_TIME_Pos);
}

/**
  * @brief  Get eFuse program time
  *
  *  Register|BitsName
  *  --------|--------
  *  TPGM    | TIME
  *
  * @param  EFUSEx eFuse instance
  * @retval Returned value can be one of the following values: 0 ~ 0xFFF
  */
__STATIC_INLINE uint32_t ll_efuse_get_tpro(efuse_regs_t *EFUSEx)
{
    return (uint32_t)(READ_BITS(EFUSEx->TPGM, EFUSE_TPGM_TIME) >> EFUSE_TPGM_TIME_Pos);
}

/**
  * @brief  Enable read address through APB bus be a main address or backup address
  *
  *  Register|BitsName
  *  --------|--------
  *  TPGM    | MAIN_OR_BACKUP
  *
  * @param  EFUSEx eFuse instance
  * @retval None
  */
__STATIC_INLINE void ll_efuse_enable_main_backup(efuse_regs_t *EFUSEx)
{
    SET_BITS(EFUSEx->TPGM, EFUSE_TPGM_MAIN_OR_BACKUP);
}

/**
  * @brief  Disable read address through APB bus be a main address or backup address
  *
  *  Register|BitsName
  *  --------|--------
  *  TPGM    | MAIN_OR_BACKUP
  *
  * @param  EFUSEx eFuse instance
  * @retval None
  */
__STATIC_INLINE void ll_efuse_disable_main_backup(efuse_regs_t *EFUSEx)
{
    CLEAR_BITS(EFUSEx->TPGM, EFUSE_TPGM_MAIN_OR_BACKUP);
}

/**
  * @brief  Check if read address through APB bus be a main address or backup address is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  TPGM    | MAIN_OR_BACKUP
  *
  * @param  EFUSEx eFuse instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_efuse_is_enabled_main_backup(efuse_regs_t *EFUSEx)
{
    return (READ_BITS(EFUSEx->TPGM, EFUSE_TPGM_MAIN_OR_BACKUP) == (EFUSE_TPGM_MAIN_OR_BACKUP));
}

/**
  * @brief  Set CRC check length
  *
  *  Register|BitsName
  *  --------|--------
  *  TPGM    | CRC_CHECK_LEN
  *
  * @param  EFUSEx eFuse instance
  * @param  length This parameter can be one of the following values: 1 ~ 60
  * @retval None
  */
__STATIC_INLINE void ll_efuse_set_crc_check_len(efuse_regs_t *EFUSEx, uint32_t length)
{
    MODIFY_REG(EFUSEx->TPGM, EFUSE_TPGM_CRC_CHECK_LEN, length << EFUSE_TPGM_CRC_CHECK_LEN_Pos);
}

/**
  * @brief  Get CRC check length
  *
  *  Register|BitsName
  *  --------|--------
  *  TPGM    | CRC_CHECK_LEN
  *
  * @param  EFUSEx eFuse instance
  * @retval Returned value can be one of the following values: 1 ~ 60
  */
__STATIC_INLINE uint32_t ll_efuse_get_crc_check_len(efuse_regs_t *EFUSEx)
{
    return (uint32_t)(READ_BITS(EFUSEx->TPGM, EFUSE_TPGM_CRC_CHECK_LEN) >> EFUSE_TPGM_CRC_CHECK_LEN_Pos);
}

/**
  * @brief  Set the interval number of clk cycles between two bit fuse
  *
  *  Register|BitsName
  *  --------|--------
  *  TPGM    | WRITE_INTERVAL
  *
  * @param  EFUSEx eFuse instance
  * @param  interval This parameter can be one of the following values: 0 ~ 0xFF
  * @retval None
  */
__STATIC_INLINE void ll_efuse_set_interval(efuse_regs_t *EFUSEx, uint32_t interval)
{
    MODIFY_REG(EFUSEx->TPGM, EFUSE_TPGM_WRITE_INTERVAL, interval << EFUSE_TPGM_WRITE_INTERVAL_Pos);
}

/**
  * @brief  Get the interval number of clk cycles between two bit fuse
  *
  *  Register|BitsName
  *  --------|--------
  *  TPGM    | WRITE_INTERVAL
  *
  * @param  EFUSEx eFuse instance
  * @retval Returned value can be one of the following values: 0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_efuse_get_interval(efuse_regs_t *EFUSEx)
{
    return (uint32_t)(READ_BITS(EFUSEx->TPGM, EFUSE_TPGM_WRITE_INTERVAL) >> EFUSE_TPGM_WRITE_INTERVAL_Pos);
}

/**
  * @brief  Enable eFuse PGENB sigal
  *
  *  Register|BitsName
  *  --------|--------
  *  PGENB   | PGENB_SIG
  *
  * @param  EFUSEx eFuse instance
  * @retval None
  */
__STATIC_INLINE void ll_efuse_enable_pgenb(efuse_regs_t *EFUSEx)
{
    SET_BITS(EFUSEx->PGENB, EFUSE_PGENB_SIG);
}

/**
  * @brief  Disable eFuse PGENB sigal
  *
  *  Register|BitsName
  *  --------|--------
  *  PGENB   | PGENB_SIG
  *
  * @param  EFUSEx eFuse instance
  * @retval None
  */
__STATIC_INLINE void ll_efuse_disable_pgenb(efuse_regs_t *EFUSEx)
{
    CLEAR_BITS(EFUSEx->PGENB, EFUSE_PGENB_SIG);
}

/**
  * @brief  Check if eFuse PGENB sigal is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  PGENB   | PGENB_SIG
  *
  * @param  EFUSEx eFuse instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_efuse_is_enabled_pgenb(efuse_regs_t *EFUSEx)
{
    return (READ_BITS(EFUSEx->PGENB, EFUSE_PGENB_SIG) == (EFUSE_PGENB_SIG));
}

/**
  * @brief  Get test mode
  * @note   This bit should be read only.
  *
  *  Register|BitsName
  *  --------|--------
  *  TEST_MODE | TEST_MODE
  *
  * @param  EFUSEx eFuse instance
  * @retval Returned value can be one of the following values: 0xFFFF
  */
__STATIC_INLINE uint32_t ll_efuse_get_test_mode(efuse_regs_t *EFUSEx)
{
    return (uint32_t)(READ_BITS(EFUSEx->TEST_MODE, EFUSE_TEST_MODE));
}

/**
  * @brief  Set eFuse operation mode
  *
  *  Register|BitsName
  *  --------|--------
  *  OPERATION | WRITE_KEYRAM
  *  OPERATION | INIT_CHECK
  *  OPERATION | CRC_CHECK
  *  OPERATION | READ_TRIM
  *  OPERATION | RD_TEST_MODE
  *
  * @param  EFUSEx eFuse instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_EFUSE_WRITE_KEYRAM
  *         @arg @ref LL_EFUSE_READ_TRIM
  *         @arg @ref LL_EFUSE_CRC_CHECK
  *         @arg @ref LL_EFUSE_INIT_CHECK
  *         @arg @ref LL_EFUSE_TEST_READ
  * @retval None
  */
__STATIC_INLINE void ll_efuse_set_operation(efuse_regs_t *EFUSEx, uint32_t mode)
{
    WRITE_REG(EFUSEx->OPERATION, mode);
}

/**
  * @brief  Check active flag
  *
  *  Register|BitsName
  *  --------|--------
  *  STAT    | WRITE_KEYRAM_BUSY
  *  STAT    | READ_TRIM_DONE
  *  STAT    | TRIM_CRC_SUCCESS
  *  STAT    | INIT_DONE
  *  STAT    | INIT_SUCCESS
  *  STAT    | CRC_CHECK_DONE
  *  STAT    | WRITE_DONE
  *  STAT    | TEST_MODE_DONE
  *
  * @param  EFUSEx eFuse instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_EFUSE_WRITE_KEYRAM_BUSY
  *         @arg @ref LL_EFUSE_READ_TRIM_DONE
  *         @arg @ref LL_EFUSE_CRC_CHECK_DONE
  *         @arg @ref LL_EFUSE_CRC_CHECK_SUCCESS
  *         @arg @ref LL_EFUSE_INIT_CHECK_DONE
  *         @arg @ref LL_EFUSE_INIT_CHECK_SUCCESS
  *         @arg @ref LL_EFUSE_WRITE_DONE
  *         @arg @ref LL_EFUSE_TEST_DONE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_efuse_is_active_flag(efuse_regs_t *EFUSEx, uint32_t flag)
{
    return (READ_BITS(EFUSEx->STAT, flag) == (flag));
}

/**
  * @brief  Set key mask
  *
  *  Register|BitsName
  *  --------|--------
  *  KEY_MASK | KEY_MASK
  *
  * @param  EFUSEx eFuse instance
  * @param  mask   Key mask
  * @retval None
  */
__STATIC_INLINE void ll_efuse_set_key_mask(efuse_regs_t *EFUSEx, uint32_t mask)
{
    WRITE_REG(EFUSEx->KEY_MASK, mask);
}

/**
  * @brief  Get key mask
  *
  *  Register|BitsName
  *  --------|--------
  *  KEY_MASK | KEY_MASK
  *
  * @param  EFUSEx eFuse instance
  * @retval None
  */
__STATIC_INLINE uint32_t ll_efuse_get_key_mask(efuse_regs_t *EFUSEx)
{
    return (uint32_t)(READ_REG(EFUSEx->KEY_MASK));
}

/**
  * @brief  Set CRC check start address
  * @note   The address must be a main info address.
  *
  *  Register|BitsName
  *  --------|--------
  *  CRC_ADDR | START_CHECK_ADDR
  *
  * @param  EFUSEx eFuse instance
  * @param  address This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_efuse_set_crc_check_addr(efuse_regs_t *EFUSEx, uint32_t address)
{
    WRITE_REG(EFUSEx->CRC_ADDR, address);
}

/**
  * @brief  Get CRC check start address
  *
  *  Register|BitsName
  *  --------|--------
  *  CRC_ADDR | START_CHECK_ADDR
  *
  * @param  EFUSEx eFuse instance
  * @retval Returned value can be one of the following values: 0 ~ 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_efuse_get_crc_check_addr(efuse_regs_t *EFUSEx)
{
    return (uint32_t)(READ_REG(EFUSEx->CRC_ADDR));
}

/**
  * @brief  Get CRC check results
  *
  *  Register|BitsName
  *  --------|--------
  *  CRC_OUTPUT | OUTPUT_VALUE
  *
  * @param  EFUSEx eFuse instance
  * @retval Returned value can be one of the following values: 0 ~ 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_efuse_get_crc_check_result(efuse_regs_t *EFUSEx)
{
    return (uint32_t)(READ_REG(EFUSEx->CRC_OUTPUT));
}

/**
  * @brief  Set read trim start address
  * @note   The address must be a main info address.
  *
  *  Register|BitsName
  *  --------|--------
  *  TRIM_ADDR | START_ADDR
  *
  * @param  EFUSEx eFuse instance
  * @param  address This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_efuse_set_trim_addr(efuse_regs_t *EFUSEx, uint32_t address)
{
    WRITE_REG(EFUSEx->TRIM_ADDR, address);
}

/**
  * @brief  Get read trim start address
  *
  *  Register|BitsName
  *  --------|--------
  *  TRIM_ADDR | START_ADDR
  *
  * @param  EFUSEx eFuse instance
  * @retval Returned value can be one of the following values: 0 ~ 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_efuse_get_trim_addr(efuse_regs_t *EFUSEx)
{
    return (uint32_t)(READ_REG(EFUSEx->TRIM_ADDR));
}

/**
  * @brief  Set read trim length
  *
  *  Register|BitsName
  *  --------|--------
  *  TRIM_LEN | LENGTH
  *
  * @param  EFUSEx eFuse instance
  * @param  length This parameter can be one of the following values: 1 ~ 14
  * @retval None
  */
__STATIC_INLINE void ll_efuse_set_trim_length(efuse_regs_t *EFUSEx, uint32_t length)
{
    WRITE_REG(EFUSEx->TRIM_LEN, length & EFUSE_TRIM_LENGTH);
}

/**
  * @brief  Get read trim length
  *
  *  Register|BitsName
  *  --------|--------
  *  TRIM_LEN | LENGTH
  *
  * @param  EFUSEx eFuse instance
  * @retval Returned value can be one of the following values: 1 ~ 14
  */
__STATIC_INLINE uint32_t ll_efuse_get_trim_length(efuse_regs_t *EFUSEx)
{
    return (uint32_t)(READ_REG(EFUSEx->TRIM_LEN) & EFUSE_TRIM_LENGTH);
}

/**
  * @brief  Get trim value
  *
  *  Register|BitsName
  *  --------|--------
  *  TRIM[n] | TRIM
  *
  * @param  EFUSEx eFuse instance
  * @param  indx   index of trim value registers: 0 ~ 13
  * @retval Returned value can be one of the following values: 0 ~ 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_efuse_get_trim_value(efuse_regs_t *EFUSEx, uint32_t indx)
{
    return (uint32_t)(READ_REG(EFUSEx->TRIM[indx]));
}

/**
  * @brief  eFuse v1.1 power on.
  *
  *  Register|BitsName
  *  --------|--------
  *  TPGM    | CRC_CHECK_LEN
  *
  * @retval None
  */
__STATIC_INLINE void ll_efuse_enable_power(efuse_regs_t *EFUSEx)
{
    SET_BITS(AON_PMU->RF_REG_2, AON_PMU_RF_REG_2_EFUSE_VDD_EN);
}

/**
  * @brief  eFuse v1.1 power off.
  *
  *  Register|BitsName
  *  --------|--------
  *  TPGM    | CRC_CHECK_LEN
  *
  * @retval None
  */
__STATIC_INLINE void ll_efuse_disable_power(efuse_regs_t *EFUSEx)
{
    CLEAR_BITS(AON_PMU->RF_REG_2, AON_PMU_RF_REG_2_EFUSE_VDD_EN | AON_PMU_RF_REG_2_EFUSE_VDDQ_EN | AON_PMU_RF_REG_2_EFUSE_VDDQ_EN_DEL);
}

/**
  * @brief  Set Efulse power controller timing pararmeter.
  *
  *  Register      |BitsName
  *  ----------|--------
  *  PWR_DELTA| PWR_DELTA_0
  *  PWR_DELTA| PWR_DELTA_1
  *  PWR_DELTA| PWR_DELTA_2
  *
  * @retval None
  */
__STATIC_INLINE void ll_efuse_set_controller_power_timing(efuse_regs_t *EFUSEx, uint16_t vddq_0, uint16_t vddq_1, uint16_t vddq_2)
{
    WRITE_REG(MCU_SUB->EFUSE_PWR_DELTA_0, vddq_0 + (vddq_1 << 16));
    WRITE_REG(MCU_SUB->EFUSE_PWR_DELTA_1, vddq_2);
}

/**
  * @brief  Power sequencer begin.
  *
  *  Register        |BitsName
  *  -----------|--------
  *  PWR_CTRL0  | CTRL_ENABLE
  *  PWR_CTRL0  | SEQR_BEGIN
  *
  * @retval None
  */
__STATIC_INLINE void ll_efuse_enable_controller_power_begin(efuse_regs_t *EFUSEx)
{
    SET_BITS(AON_PMU->RF_REG_2, AON_PMU_RF_REG_2_EFUSE_VDDQ_EN | AON_PMU_RF_REG_2_EFUSE_VDDQ_EN_DEL);
    WRITE_REG(MCU_SUB->EFUSE_PWR_CTRL_0, MCU_SUB_EFUSE_PWR_CTL0_BGN | MCU_SUB_EFUSE_PWR_CTL0_EN);
}

/**
  * @brief  Power sequencer begin.
  *
  *  Register        |BitsName
  *  -----------|--------
  *  PWR_CTRL0  | CTRL_ENABLE
  *  PWR_CTRL0  | SEQR_STOP
  *
  * @retval None
  */
__STATIC_INLINE void ll_efuse_enable_controller_power_stop(efuse_regs_t *EFUSEx)
{
    WRITE_REG(MCU_SUB->EFUSE_PWR_CTRL_0, MCU_SUB_EFUSE_PWR_CTL0_STP | MCU_SUB_EFUSE_PWR_CTL0_EN);
    CLEAR_BITS(AON_PMU->RF_REG_2, AON_PMU_RF_REG_2_EFUSE_VDDQ_EN | AON_PMU_RF_REG_2_EFUSE_VDDQ_EN_DEL);
}

/**
  * @brief  Power sequencer begin.
  *
  *  Register        |BitsName
  *  --------- -|--------
  *  PWR_CTRL0  | CTRL_ENABLE
  *  PWR_CTRL0  | SEQR_BEGIN
  *  PWR_CTRL0  | SEQR_STOP
  *
  * @retval None
  */
__STATIC_INLINE void ll_efuse_disable_controller_power(efuse_regs_t *EFUSEx)
{
    WRITE_REG(MCU_SUB->EFUSE_PWR_CTRL_0, 0);
}

/**
  * @brief  Check power controller active flag
  *
  *  Register|BitsName
  *  --------|--------
  *  PWR_CTRL1    | EN_DONE
  *  PWR_CTRL1    | DIS_DONE
  *
  * @param  EFUSEx eFuse instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_EFUSE_PWR_CTL_EN_DONE
  *         @arg @ref LL_EFUSE_PWR_CTL_DIS_DONE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_efuse_is_controller_power_flag(efuse_regs_t *EFUSEx, uint32_t flag)
{
    return (READ_BITS(MCU_SUB->EFUSE_PWR_CTRL_1, flag) == (flag));
}



/** @} */

/** @} */

#endif /* EFUSE */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_EFUSE_H__ */

/** @} */

/** @} */

/** @} */

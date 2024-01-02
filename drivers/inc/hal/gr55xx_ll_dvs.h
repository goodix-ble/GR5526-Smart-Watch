/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_dvs.h
 * @author  BLE RD
 * @brief   Header file containing functions prototypes of dvs LL library.
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

/** @defgroup LL_DVS DVS
  * @brief DVS LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_dvs_H_
#define __GR55XX_LL_dvs_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

/**
  * @defgroup  DVS_LL_MACRO Defines
  * @{
  */
/** @defgroup Analog_Voltage_Scale_enable Analog Voltage Scaling enable state defines
  * @{
  */
#define LL_ANALOG_VOLTAGE_SCALE_DIS                               (0U)  /**< Analog Voltage Scaling Disable(default) */
#define LL_ANALOG_VOLTAGE_SCALE_EN                                (1U)  /**< Analog Voltage Scaling Enable */
/** @} */

/** @defgroup Analog_Voltage_Scale_type Analog Voltage Scaling slop control type defines
  * @{
  */
#define LL_ANALOG_VOLTAGE_SCALE_LOWER_TYPE                        (0U)  /**< Analog Voltage Scaling Lower Type(default) */
#define LL_ANALOG_VOLTAGE_SCALE_HIGHER_TYPE                       (1U)  /**< Analog Voltage Scaling Higher Type */
/** @} */

/** @defgroup Analog_Voltage_Scale_limit_enable Analog Voltage Scaling limiter enable defines
  * @{
  */
#define LL_ANALOG_VOLTAGE_SCALE_LIMIT_DIS                         (0U)  /**< Analog Voltage Scaling Limit Disable(default) */
#define LL_ANALOG_VOLTAGE_SCALE_LIMIT_EN                          (1U)  /**< Analog Voltage Scaling Limit Enable */
/** @} */

/** @} */

/** @defgroup DVS_LL_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
  * @brief  The DVS_DCDC block enable set
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DCDC | EN_VTBIAS
  */
__STATIC_INLINE void ll_dvs_dcdc_enable_set(uint8_t enable)
{
    MODIFY_REG(AON_PMU->DVS_DCDC, AON_PMU_DVS_DCDC_EN_VTBIAS, (enable << AON_PMU_DVS_DCDC_EN_VTBIAS_Pos));
}

/**
  * @brief  The DVS_DCDC block enable get
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DCDC | EN_VTBIAS
  */
__STATIC_INLINE uint8_t ll_dvs_dcdc_enable_get(void)
{
    return ((READ_BITS(AON_PMU->DVS_DCDC, AON_PMU_DVS_DCDC_EN_VTBIAS)) >> AON_PMU_DVS_DCDC_EN_VTBIAS_Pos);
}

/**
  * @brief  The DVS_DCDC's Slop Control type set
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DCDC | VTBIAS_SLOPE_CTRL
  */
__STATIC_INLINE void ll_dvs_dcdc_vtbias_slop_ctrl_set(uint8_t type)
{
    MODIFY_REG(AON_PMU->DVS_DCDC, AON_PMU_DVS_DCDC_VTBIAS_SLOPE_CTRL, (type << AON_PMU_DVS_DCDC_VTBIAS_SLOPE_CTRL_Pos));
}

/**
  * @brief  The DVS_DCDC's Slop Control type get
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DCDC | VTBIAS_SLOPE_CTRL
  */
__STATIC_INLINE uint8_t ll_dvs_dcdc_vtbias_slop_ctrl_get(void)
{
    return ((READ_BITS(AON_PMU->DVS_DCDC, AON_PMU_DVS_DCDC_VTBIAS_SLOPE_CTRL)) >> AON_PMU_DVS_DCDC_VTBIAS_SLOPE_CTRL_Pos);
}

/**
  * @brief  The DVS_DCDC's Lower Limit Control enable set
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DCDC | EN_LIMITER
  */
__STATIC_INLINE void ll_dvs_dcdc_limiter_enable_set(uint8_t enable)
{
    MODIFY_REG(AON_PMU->DVS_DCDC, AON_PMU_DVS_DCDC_EN_LIMITER, (enable << AON_PMU_DVS_DCDC_EN_LIMITER_Pos));
}

/**
  * @brief The DVS_DCDC's Lower Limit Control enable get
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DCDC | EN_LIMITER
  */
__STATIC_INLINE uint8_t ll_dvs_dcdc_limiter_enable_get(void)
{
    return ((READ_BITS(AON_PMU->DVS_DCDC, AON_PMU_DVS_DCDC_EN_LIMITER)) >> AON_PMU_DVS_DCDC_EN_LIMITER_Pos);
}

/**
  * @brief  The DVS_DCDC's default level value of the VT bias set
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DCDC | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE void ll_dvs_dcdc_vtbias_ctrl_vt_set(uint8_t vt)
{
    MODIFY_REG(AON_PMU->DVS_DCDC, AON_PMU_DVS_DCDC_VTBIAS_CTRL_VT_2_0, (vt << AON_PMU_DVS_DCDC_VTBIAS_CTRL_VT_2_0_Pos));
}

/**
  * @brief The DVS_DCDC's default level value of the VT bias get
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DCDC | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE uint8_t ll_dvs_dcdc_vtbias_ctrl_vt_get(void)
{
    return ((READ_BITS(AON_PMU->DVS_DCDC, AON_PMU_DVS_DCDC_VTBIAS_CTRL_VT_2_0)) >> AON_PMU_DVS_DCDC_VTBIAS_CTRL_VT_2_0_Pos);
}

/**
  * @brief  The DVS_DCDC's lower limit for the output voltage set
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DCDC | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE void ll_dvs_dcdc_vtbias_ctrl_lower_limit_set(uint8_t limit)
{
    MODIFY_REG(AON_PMU->DVS_DCDC, AON_PMU_DVS_DCDC_VTBIAS_CTRL_LIMIT_2_0, (limit << AON_PMU_DVS_DCDC_VTBIAS_CTRL_LIMIT_2_0_Pos));
}

/**
  * @brief The DVS_DCDC's lower limit for the output voltage get
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DCDC | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE uint8_t ll_dvs_dcdc_vtbias_ctrl_lower_limit_get(void)
{
    return ((READ_BITS(AON_PMU->DVS_DCDC, AON_PMU_DVS_DCDC_VTBIAS_CTRL_LIMIT_2_0)) >> AON_PMU_DVS_DCDC_VTBIAS_CTRL_LIMIT_2_0_Pos);
}

/**
  * @brief  The DVS_DIGCORE block enable set
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DIGCORE | EN_VTBIAS
  */
__STATIC_INLINE void ll_dvs_digcore_enable_set(uint8_t enable)
{
    MODIFY_REG(AON_PMU->DVS_DIGCORE, AON_PMU_DVS_DIGCORE_EN_VTBIAS, (enable << AON_PMU_DVS_DIGCORE_EN_VTBIAS_Pos));
}

/**
  * @brief  The DVS_DIGCORE block enable get
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DIGCORE | EN_VTBIAS
  */
__STATIC_INLINE uint8_t ll_dvs_digcore_enable_get(void)
{
    return ((READ_BITS(AON_PMU->DVS_DIGCORE, AON_PMU_DVS_DIGCORE_EN_VTBIAS)) >> AON_PMU_DVS_DIGCORE_EN_VTBIAS_Pos);
}

/**
  * @brief  The DVS_DIGCORE's Slop Control type set
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DIGCORE | VTBIAS_SLOPE_CTRL
  */
__STATIC_INLINE void ll_dvs_digcore_vtbias_slop_ctrl_set(uint8_t type)
{
    MODIFY_REG(AON_PMU->DVS_DIGCORE, AON_PMU_DVS_DIGCORE_VTBIAS_SLOPE_CTRL, (type << AON_PMU_DVS_DIGCORE_VTBIAS_SLOPE_CTRL_Pos));
}

/**
  * @brief  The DVS_DIGCORE's Slop Control type get
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DIGCORE | VTBIAS_SLOPE_CTRL
  */
__STATIC_INLINE uint8_t ll_dvs_digcore_vtbias_slop_ctrl_get(void)
{
    return ((READ_BITS(AON_PMU->DVS_DIGCORE, AON_PMU_DVS_DIGCORE_VTBIAS_SLOPE_CTRL)) >> AON_PMU_DVS_DIGCORE_VTBIAS_SLOPE_CTRL_Pos);
}

/**
  * @brief  The DVS_DIGCORE's Lower Limit Control enable set
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DIGCORE | EN_LIMITER
  */
__STATIC_INLINE void ll_dvs_digcore_limiter_enable_set(uint8_t enable)
{
    MODIFY_REG(AON_PMU->DVS_DIGCORE, AON_PMU_DVS_DIGCORE_EN_LIMITER, (enable << AON_PMU_DVS_DIGCORE_EN_LIMITER_Pos));
}

/**
  * @brief The DVS_DIGCORE's Lower Limit Control enable get
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DIGCORE | EN_LIMITER
  */
__STATIC_INLINE uint8_t ll_dvs_digcore_limiter_enable_get(void)
{
    return ((READ_BITS(AON_PMU->DVS_DIGCORE, AON_PMU_DVS_DIGCORE_EN_LIMITER)) >> AON_PMU_DVS_DIGCORE_EN_LIMITER_Pos);
}

/**
  * @brief  The DVS_DIGCORE's default level value of the VT bias set
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DIGCORE | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE void ll_dvs_digcore_vtbias_ctrl_vt_set(uint8_t vt)
{
    MODIFY_REG(AON_PMU->DVS_DIGCORE, AON_PMU_DVS_DIGCORE_VTBIAS_CTRL_VT_2_0, (vt << AON_PMU_DVS_DIGCORE_VTBIAS_CTRL_VT_2_0_Pos));
}

/**
  * @brief The DVS_DIGCORE's default level value of the VT bias get
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DIGCORE | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE uint8_t ll_dvs_digcore_vtbias_ctrl_vt_get(void)
{
    return ((READ_BITS(AON_PMU->DVS_DIGCORE, AON_PMU_DVS_DIGCORE_VTBIAS_CTRL_VT_2_0)) >> AON_PMU_DVS_DIGCORE_VTBIAS_CTRL_VT_2_0_Pos);
}

/**
  * @brief  The DVS_DIGCORE's lower limit for the output voltage set
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DIGCORE | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE void ll_dvs_digcore_vtbias_ctrl_lower_limit_set(uint8_t limit)
{
    MODIFY_REG(AON_PMU->DVS_DIGCORE, AON_PMU_DVS_DIGCORE_VTBIAS_CTRL_LIMIT_2_0, (limit << AON_PMU_DVS_DIGCORE_VTBIAS_CTRL_LIMIT_2_0_Pos));
}

/**
  * @brief The DVS_DIGCORE's lower limit for the output voltage get
  *
  *  Register|BitsName
  *  --------|--------
  *  DVS_DIGCORE | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE uint8_t ll_dvs_digcore_vtbias_ctrl_lower_limit_get(void)
{
    return ((READ_BITS(AON_PMU->DVS_DIGCORE, AON_PMU_DVS_DIGCORE_VTBIAS_CTRL_LIMIT_2_0)) >> AON_PMU_DVS_DIGCORE_VTBIAS_CTRL_LIMIT_2_0_Pos);
}

/** @} */

#endif
/** @} */

/** @} */

/** @} */

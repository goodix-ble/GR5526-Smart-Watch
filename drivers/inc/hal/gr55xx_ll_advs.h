/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_advs.h
 * @author  BLE RD
 * @brief   Header file containing functions prototypes of advs LL library.
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

/** @defgroup LL_ADVS ADVS
  * @brief ADVS LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_ADVS_H_
#define __GR55XX_LL_ADVS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

/**
  * @defgroup  LL_ADVS_MACRO Defines
  * @{
  */
/** @defgroup LL_ADVS_ENABLE Analog Voltage Scaling enable state defines
  * @{
  */
#define LL_ADVS_DIS                               (0U)  /**< Analog Voltage Scaling Disable */
#define LL_ADVS_EN                                (1U)  /**< Analog Voltage Scaling Enable */
/** @} */

/** @defgroup LL_ADVS_TYPE Analog Voltage Scaling slope control type defines
  * @{
  */
#define LL_ADVS_LOWER_TYPE                        (0U)  /**< Analog Voltage Scaling Lower Type(default) */
#define LL_ADVS_HIGHER_TYPE                       (1U)  /**< Analog Voltage Scaling Higher Type */
/** @} */

/** @defgroup LL_ADVS_LIMIT_ENABLE Analog Voltage Scaling limiter enable defines
  * @{
  */
#define LL_ADVS_LIMIT_DIS                         (0U)  /**< Analog Voltage Scaling Limit Disable(default) */
#define LL_ADVS_LIMIT_EN                          (1U)  /**< Analog Voltage Scaling Limit Enable */
/** @} */

/** @} */

/** @defgroup LL_ADVS_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
  * @brief  The ADVS_DCDC block enable set
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DCDC | EN_VTBIAS
  */
__STATIC_INLINE void ll_advs_dcdc_enable_set(uint8_t enable)
{
    MODIFY_REG(AON_PMU->ADVS_DCDC, AON_PMU_ADVS_DCDC_EN_VTBIAS, (enable << AON_PMU_ADVS_DCDC_EN_VTBIAS_Pos));
}

/**
  * @brief  The ADVS_DCDC block enable get
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DCDC | EN_VTBIAS
  */
__STATIC_INLINE uint8_t ll_advs_dcdc_enable_get(void)
{
    return ((READ_BITS(AON_PMU->ADVS_DCDC, AON_PMU_ADVS_DCDC_EN_VTBIAS)) >> AON_PMU_ADVS_DCDC_EN_VTBIAS_Pos);
}

/**
  * @brief  The ADVS_DCDC's Slop Control type set
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DCDC | VTBIAS_SLOPE_CTRL
  */
__STATIC_INLINE void ll_advs_dcdc_vtbias_slop_ctrl_set(uint8_t type)
{
    MODIFY_REG(AON_PMU->ADVS_DCDC, AON_PMU_ADVS_DCDC_VTBIAS_SLOPE_CTRL, (type << AON_PMU_ADVS_DCDC_VTBIAS_SLOPE_CTRL_Pos));
}

/**
  * @brief  The ADVS_DCDC's Slop Control type get
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DCDC | VTBIAS_SLOPE_CTRL
  */
__STATIC_INLINE uint8_t ll_advs_dcdc_vtbias_slop_ctrl_get(void)
{
    return ((READ_BITS(AON_PMU->ADVS_DCDC, AON_PMU_ADVS_DCDC_VTBIAS_SLOPE_CTRL)) >> AON_PMU_ADVS_DCDC_VTBIAS_SLOPE_CTRL_Pos);
}

/**
  * @brief  The ADVS_DCDC's Lower Limit Control enable set
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DCDC | EN_LIMITER
  */
__STATIC_INLINE void ll_advs_dcdc_limiter_enable_set(uint8_t enable)
{
    MODIFY_REG(AON_PMU->ADVS_DCDC, AON_PMU_ADVS_DCDC_EN_LIMITER, (enable << AON_PMU_ADVS_DCDC_EN_LIMITER_Pos));
}

/**
  * @brief The ADVS_DCDC's Lower Limit Control enable get
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DCDC | EN_LIMITER
  */
__STATIC_INLINE uint8_t ll_advs_dcdc_limiter_enable_get(void)
{
    return ((READ_BITS(AON_PMU->ADVS_DCDC, AON_PMU_ADVS_DCDC_EN_LIMITER)) >> AON_PMU_ADVS_DCDC_EN_LIMITER_Pos);
}

/**
  * @brief  The ADVS_DCDC's default level value of the VT bias set
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DCDC | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE void ll_advs_dcdc_vtbias_ctrl_vt_set(uint8_t vt)
{
    MODIFY_REG(AON_PMU->ADVS_DCDC, AON_PMU_ADVS_DCDC_VTBIAS_CTRL_VT_2_0, (vt << AON_PMU_ADVS_DCDC_VTBIAS_CTRL_VT_2_0_Pos));
}

/**
  * @brief The ADVS_DCDC's default level value of the VT bias get
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DCDC | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE uint8_t ll_advs_dcdc_vtbias_ctrl_vt_get(void)
{
    return ((READ_BITS(AON_PMU->ADVS_DCDC, AON_PMU_ADVS_DCDC_VTBIAS_CTRL_VT_2_0)) >> AON_PMU_ADVS_DCDC_VTBIAS_CTRL_VT_2_0_Pos);
}

/**
  * @brief  The ADVS_DCDC's lower limit for the output voltage set
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DCDC | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE void ll_advs_dcdc_vtbias_ctrl_lower_limit_set(uint8_t limit)
{
    MODIFY_REG(AON_PMU->ADVS_DCDC, AON_PMU_ADVS_DCDC_VTBIAS_CTRL_LIMIT_2_0, (limit << AON_PMU_ADVS_DCDC_VTBIAS_CTRL_LIMIT_2_0_Pos));
}

/**
  * @brief The ADVS_DCDC's lower limit for the output voltage get
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DCDC | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE uint8_t ll_advs_dcdc_vtbias_ctrl_lower_limit_get(void)
{
    return ((READ_BITS(AON_PMU->ADVS_DCDC, AON_PMU_ADVS_DCDC_VTBIAS_CTRL_LIMIT_2_0)) >> AON_PMU_ADVS_DCDC_VTBIAS_CTRL_LIMIT_2_0_Pos);
}

/**
  * @brief  The ADVS_DIGCORE block enable set
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DIGCORE | EN_VTBIAS
  */
__STATIC_INLINE void ll_advs_digcore_enable_set(uint8_t enable)
{
    MODIFY_REG(AON_PMU->ADVS_DIGCORE, AON_PMU_ADVS_DIGCORE_EN_VTBIAS, (enable << AON_PMU_ADVS_DIGCORE_EN_VTBIAS_Pos));
}

/**
  * @brief  The ADVS_DIGCORE block enable get
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DIGCORE | EN_VTBIAS
  */
__STATIC_INLINE uint8_t ll_advs_digcore_enable_get(void)
{
    return ((READ_BITS(AON_PMU->ADVS_DIGCORE, AON_PMU_ADVS_DIGCORE_EN_VTBIAS)) >> AON_PMU_ADVS_DIGCORE_EN_VTBIAS_Pos);
}

/**
  * @brief  The ADVS_DIGCORE's Slop Control type set
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DIGCORE | VTBIAS_SLOPE_CTRL
  */
__STATIC_INLINE void ll_advs_digcore_vtbias_slop_ctrl_set(uint8_t type)
{
    MODIFY_REG(AON_PMU->ADVS_DIGCORE, AON_PMU_ADVS_DIGCORE_VTBIAS_SLOPE_CTRL, (type << AON_PMU_ADVS_DIGCORE_VTBIAS_SLOPE_CTRL_Pos));
}

/**
  * @brief  The ADVS_DIGCORE's Slop Control type get
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DIGCORE | VTBIAS_SLOPE_CTRL
  */
__STATIC_INLINE uint8_t ll_advs_digcore_vtbias_slop_ctrl_get(void)
{
    return ((READ_BITS(AON_PMU->ADVS_DIGCORE, AON_PMU_ADVS_DIGCORE_VTBIAS_SLOPE_CTRL)) >> AON_PMU_ADVS_DIGCORE_VTBIAS_SLOPE_CTRL_Pos);
}

/**
  * @brief  The ADVS_DIGCORE's Lower Limit Control enable set
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DIGCORE | EN_LIMITER
  */
__STATIC_INLINE void ll_advs_digcore_limiter_enable_set(uint8_t enable)
{
    MODIFY_REG(AON_PMU->ADVS_DIGCORE, AON_PMU_ADVS_DIGCORE_EN_LIMITER, (enable << AON_PMU_ADVS_DIGCORE_EN_LIMITER_Pos));
}

/**
  * @brief The ADVS_DIGCORE's Lower Limit Control enable get
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DIGCORE | EN_LIMITER
  */
__STATIC_INLINE uint8_t ll_advs_digcore_limiter_enable_get(void)
{
    return ((READ_BITS(AON_PMU->ADVS_DIGCORE, AON_PMU_ADVS_DIGCORE_EN_LIMITER)) >> AON_PMU_ADVS_DIGCORE_EN_LIMITER_Pos);
}

/**
  * @brief  The ADVS_DIGCORE's default level value of the VT bias set
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DIGCORE | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE void ll_advs_digcore_vtbias_ctrl_vt_set(uint8_t vt)
{
    MODIFY_REG(AON_PMU->ADVS_DIGCORE, AON_PMU_ADVS_DIGCORE_VTBIAS_CTRL_VT_2_0, (vt << AON_PMU_ADVS_DIGCORE_VTBIAS_CTRL_VT_2_0_Pos));
}

/**
  * @brief The ADVS_DIGCORE's default level value of the VT bias get
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DIGCORE | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE uint8_t ll_advs_digcore_vtbias_ctrl_vt_get(void)
{
    return ((READ_BITS(AON_PMU->ADVS_DIGCORE, AON_PMU_ADVS_DIGCORE_VTBIAS_CTRL_VT_2_0)) >> AON_PMU_ADVS_DIGCORE_VTBIAS_CTRL_VT_2_0_Pos);
}

/**
  * @brief  The ADVS_DIGCORE's lower limit for the output voltage set
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DIGCORE | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE void ll_advs_digcore_vtbias_ctrl_lower_limit_set(uint8_t limit)
{
    MODIFY_REG(AON_PMU->ADVS_DIGCORE, AON_PMU_ADVS_DIGCORE_VTBIAS_CTRL_LIMIT_2_0, (limit << AON_PMU_ADVS_DIGCORE_VTBIAS_CTRL_LIMIT_2_0_Pos));
}

/**
  * @brief The ADVS_DIGCORE's lower limit for the output voltage get
  *
  *  Register|BitsName
  *  --------|--------
  *  ADVS_DIGCORE | VTBIAS_CTRL_VT_2_0
  */
__STATIC_INLINE uint8_t ll_advs_digcore_vtbias_ctrl_lower_limit_get(void)
{
    return ((READ_BITS(AON_PMU->ADVS_DIGCORE, AON_PMU_ADVS_DIGCORE_VTBIAS_CTRL_LIMIT_2_0)) >> AON_PMU_ADVS_DIGCORE_VTBIAS_CTRL_LIMIT_2_0_Pos);
}

/** @} */

#endif
/** @} */

/** @} */

/** @} */

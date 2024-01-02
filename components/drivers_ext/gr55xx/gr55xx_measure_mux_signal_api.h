/**
 ****************************************************************************************
 *
 * @file gr55xx_measure_mux_signal_api.h
 *
 * @brief Header file - GR55XX MUX module.
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
 *****************************************************************************************
 */
#ifndef __GR55XX_MEASURE_MUX_API_H__
#define __GR55XX_MEASURE_MUX_API_H__

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  gr55xx mux signals definition
  */
typedef enum
{
     /**< MUX signal of LPD block. */
    GR55XX_MUX_SIGNAL_LPD_RESET     = 0x00U,
    GR55XX_MUX_SIGNAL_LPD_POR_RST,
    GR55XX_MUX_SIGNAL_LPD_VDD_RC,
    GR55XX_MUX_SIGNAL_LPD_POC,
    GR55XX_MUX_SIGNAL_LPD_SRPG_BIAS,
    GR55XX_MUX_SIGNAL_LPD_RET_BIAS,
    GR55XX_MUX_SIGNAL_LPD_PMU_BOD,
    GR55XX_MUX_SIGNAL_LPD_VDDC_AON,
    /**< MUX signal of SensADC block. */
    GR55XX_MUX_SIGNAL_SNSADC_VREF,

    /**< MUX signal of VDMs block. */
    GR55XX_MUX_SIGNAL_VDM_0,

    /**< MUX signal of PD cores block. */
    GR55XX_MUX_SIGNAL_PD_SRON,

    /**< MUX signal of EFUSE block. */
    GR55XX_MUX_SIGNAL_EFUSE_25V,

    /**< MUX signal of Clock block. */
    GR55XX_MUX_SIGNAL_CLOCK_FS_750K,

    /**< MUX signal of LDO block. */
    GR55XX_MUX_SIGNAL_LDO_VIO,

    /**< MUX signal of DCDC block. */
    GR55XX_MUX_SIGNAL_DCDC_BATT_CMP,

    GR55XX_MUX_SIGNAL_MAX,
}gr55xx_mux_signal_t;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief  Initialize mux signal measurement modul.
 *
 ****************************************************************************************
 */
void gr55xx_mux_signal_measurement_init(void);

/**
 ****************************************************************************************
 * @brief  Get the average value of mux signal input
 *
 * @return The volatge of mux signal. Unit (volt).
 ****************************************************************************************
 */
double gr55xx_mux_signal_measure_average(gr55xx_mux_signal_t my_mux_signal);

/**
 ****************************************************************************************
 * @brief  Get num points value of mux signal input, num cannot over 4096
 *
 * @return The volatge of num points value of mux signal. Unit (volt).
 ****************************************************************************************
 */
void gr55xx_mux_signal_measure_n_points(gr55xx_mux_signal_t my_mux_signal, uint32_t num, double* result_buf);

#ifdef __cplusplus
}
#endif

#endif // __GR55XX_MEASURE_MUX_API_H__

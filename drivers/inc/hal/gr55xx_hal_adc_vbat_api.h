/**
 ****************************************************************************************
 *
 * @file gr55xx_hal_adc_vbat_api.h
 *
 * @brief Header file - GR55XX battery module.
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
#ifndef __GR55XX_HAL_ADC_VBAT_API_H__
#define __GR55XX_HAL_ADC_VBAT_API_H__

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief  Initialize ADC battery voltage detection.
 *
 ****************************************************************************************
 */
void hal_adc_vbat_init(void);

/**
 ****************************************************************************************
 * @brief  Get the battery voltage.
 *
 * @return The volatge of battery. Unit (volt).
 ****************************************************************************************
 */
double hal_adc_vbat_read(void);

/**
 ****************************************************************************************
 * @brief  Convert the ADC conversion results to battery value.
 *
 * @param[in]  hadc: Pointer to a ADC handle which contains the configuration information for
 *                    the specified ADC module.
 * @param[in]  inbuf: Pointer to data buffer which storage ADC codes.
 * @param[out] outbuf: Pointer to data buffer which to storage conversion results.
 * @param[in]  buflen: Length of data buffer.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
void hal_adc_vbat_conv(adc_handle_t *hadc, uint16_t *inbuf, double *outbuf, uint32_t buflen);

#ifdef __cplusplus
}
#endif

#endif // __GR55XX_HAL_ADC_VBAT_API_H__

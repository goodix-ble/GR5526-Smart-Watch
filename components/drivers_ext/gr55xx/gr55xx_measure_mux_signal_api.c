/**
 ****************************************************************************************
 *
 * @file gr55xx_measure_mux_signal_api.c
 *
 * @brief GR55XX MUX module.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "grx_hal.h"
#include "grx_sys.h"
#include "gr55xx_measure_mux_signal_api.h"

/* Private macros ------------------------------------------------------------*/
#define  ADC_INPUT_SRC_MUX    (0x0D)
#define  MAX_SIMPLE_POINTS    (4096)

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * STATIC VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static adc_handle_t gr55xx_snsadc_handle = {0};
static double adc_offset = 0;
static double adc_slope = 0;

static void gr55xx_set_signal_to_mux(gr55xx_mux_signal_t my_mux_signal)
{

}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void gr55xx_mux_signal_measurement_init(void)
{
    adc_trim_info_t adc_trim = {0};

    gr55xx_snsadc_handle.init.channel_n  = ADC_INPUT_SRC_REF;
    gr55xx_snsadc_handle.init.channel_p  = ADC_INPUT_SRC_MUX;
    gr55xx_snsadc_handle.init.input_mode = ADC_INPUT_DIFFERENTIAL;
    gr55xx_snsadc_handle.init.ref_source = ADC_REF_SRC_BUF_INT;
    gr55xx_snsadc_handle.init.ref_value  = ADC_REF_VALUE_0P8;
#if !defined(GR55XXx)
    gr55xx_snsadc_handle.init.clock      = ADC_CLK_1M;
#else
    gr55xx_snsadc_handle.init.clock      = ADC_CLK_1P6M;
#endif
    hal_adc_init(&gr55xx_snsadc_handle);

    if(SDK_SUCCESS == sys_adc_trim_get(&adc_trim))
    {
        adc_offset = (double)adc_trim.offset_int_0p8;
        adc_slope = (-1) * (double)adc_trim.slope_int_0p8;
    }
    else
    {
        adc_offset = 8362;
        adc_slope = -4754;
    }
    return;
}

double gr55xx_mux_signal_measure_average(gr55xx_mux_signal_t my_mux_signal)
{
    uint16_t conver_buff[16] = {0};
    uint16_t average = 0;
    double test_result;

    /* Set the fixed signal to Mux*/
    gr55xx_set_signal_to_mux(my_mux_signal);

    /* Get the average of mux signal */
    hal_adc_poll_for_conversion(&gr55xx_snsadc_handle, conver_buff, 16);
    for(uint8_t i = 0; i < 8; i++)
    {
        average += conver_buff[8 + i];
    }
    average = average >> 3;
    test_result = (((double)average - adc_offset) / adc_slope);

    test_result = test_result + 0.85;

    return test_result;
}

void gr55xx_mux_signal_measure_n_points(gr55xx_mux_signal_t my_mux_signal, uint32_t num, double* result_buf)
{
    uint16_t conver_buff[MAX_SIMPLE_POINTS] = {0};
    double test_result;

    /* Set the fixed signal to Mux*/
    gr55xx_set_signal_to_mux(my_mux_signal);

    /* Get the average of mux signal */
    memset(&conver_buff[0], 0, 2*MAX_SIMPLE_POINTS);
    hal_adc_poll_for_conversion(&gr55xx_snsadc_handle, conver_buff, num);
    for(uint32_t i=0; i<num; i++)
    {
        test_result = (((double)conver_buff[i] - adc_offset) / adc_slope);
        test_result = test_result + 0.85;
        result_buf[i] = test_result;
    }
}

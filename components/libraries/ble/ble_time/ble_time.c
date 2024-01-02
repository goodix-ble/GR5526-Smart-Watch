/**
 *****************************************************************************************
 *
 * @file ble_time.c
 *
 * @brief Provide method to get and calculate ble time.
 *
 *****************************************************************************************
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
#include "ble_time.h"

typedef struct
{
    uint32_t hs;
    uint16_t hus;
    uint32_t bts;
} rwip_time_t;

/**
 ****************************************************************************************
 * @brief  This function get the ble time.
 *
 * @note   This function is supported only when ble core is powered on
 *
 ****************************************************************************************
 */
extern rwip_time_t rwip_time_get(void);

/**
 ****************************************************************************************
 * @brief  This function gets the ble time.
 *
 * @note   This function is supported only when ble stack is initiated
 *
 ****************************************************************************************
 */
SECTION_RAM_CODE ble_time_t ble_time_get(void)
{
    rwip_time_t ret1;
    ble_time_t ret2;

    pwr_mgmt_ble_wakeup();
    ret1 = rwip_time_get();
    ret2.hs = ret1.hs;
    ret2.hus = ret1.hus;

    return ret2;
}

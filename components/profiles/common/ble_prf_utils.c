/**
 *******************************************************************************
 *
 * @file  ble_prf_utils.c
 *
 * @brief Implementation of Profile Utilities
 *
 *******************************************************************************
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
 *******************************************************************************
 */
#include "ble_prf_utils.h"
#include "utility.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void prf_pack_char_pres_fmt(uint8_t                   *p_packed_val,
                            const prf_char_pres_fmt_t *p_char_pres_fmt)
{
    *p_packed_val       = p_char_pres_fmt->format;
    *(p_packed_val + 1) = p_char_pres_fmt->exponent;
    
    htole16(p_packed_val + 2, p_char_pres_fmt->unit);
    
    *(p_packed_val + 4) = p_char_pres_fmt->name_space;
    
    htole16(p_packed_val + 5, p_char_pres_fmt->description);
}

void prf_unpack_char_pres_fmt(const uint8_t       *p_packed_val,
                              prf_char_pres_fmt_t *p_char_pres_fmt)
{

    p_char_pres_fmt->format      = *p_packed_val;
    p_char_pres_fmt->exponent    = *(p_packed_val + 1);
    p_char_pres_fmt->unit        =  le16toh(p_packed_val + 2);
    p_char_pres_fmt->name_space  = *(p_packed_val + 4);
    p_char_pres_fmt->description =  le16toh(p_packed_val + 5);
}

uint8_t prf_pack_date_time(uint8_t               *p_packed_val,
                           const prf_date_time_t *p_date_time)
{
    htole16(p_packed_val, p_date_time->year);
    *(p_packed_val + 2) = p_date_time->month;
    *(p_packed_val + 3) = p_date_time->day;
    *(p_packed_val + 4) = p_date_time->hour;
    *(p_packed_val + 5) = p_date_time->min;
    *(p_packed_val + 6) = p_date_time->sec;

    return 7;
}

uint8_t prf_unpack_date_time(const uint8_t   *p_packed_val,
                             prf_date_time_t *p_date_time)
{
    p_date_time->year  = le16toh(&(p_packed_val[0]));
    p_date_time->month = p_packed_val[2];
    p_date_time->day   = p_packed_val[3];
    p_date_time->hour  = p_packed_val[4];
    p_date_time->min   = p_packed_val[5];
    p_date_time->sec   = p_packed_val[6];

    return 7;
}

uint8_t prf_find_idx_by_handle(uint16_t handle,  uint16_t start_hdl,
                               uint8_t  char_nb, uint8_t *p_char_mask)
{
    uint16_t cur_hdl = start_hdl + 1;
    uint8_t  index   = 0;
    uint8_t  byte    = 0;
    uint8_t  bit     = 0;

    for (uint8_t i = 1; i < char_nb; i++) {
        byte = i / 8;
        bit  = i % 8;
        if ((p_char_mask[byte] >> bit) & 0x01) {
            // check if value handle correspond to requested handle
            if (cur_hdl == handle) {
                index = i;
                break;
            }
            cur_hdl++;
        }
    }

    return index;
}

uint16_t prf_find_handle_by_idx(uint8_t idx, uint16_t start_hdl,
                                uint8_t *p_char_mask)
{
    uint16_t found_hdl = 0x0000;    // Core spec: Reserved for future use
    uint16_t cur_hdl   = start_hdl;
    uint8_t  byte      = 0;
    uint8_t  bit       = 0;

    if (!idx) {
        found_hdl = start_hdl;
    } else {
        for(uint8_t i = 1; i <= idx; i++) {
            byte = i / 8;
            bit  = i % 8;

            if ((p_char_mask[byte] >> bit) & 0x01) {
                cur_hdl++;

                if (i == idx) {
                    found_hdl = cur_hdl;
                }
            }
        }
    }

    return found_hdl;
}

bool prf_is_cccd_value_valid(uint16_t cccd_value)
{
    if (PRF_CLI_STOP_NTFIND == cccd_value || \
        PRF_CLI_START_NTF == cccd_value || \
        PRF_CLI_START_IND == cccd_value)
    {
        return true;
    }

    return false;
}

bool prf_is_notification_enabled(uint16_t cccd_value)
{
    return ((cccd_value & PRF_CLI_START_NTF) != 0);
}

bool prf_is_indication_enabled(uint16_t cccd_value)
{
    return ((cccd_value & PRF_CLI_START_IND) != 0);
}


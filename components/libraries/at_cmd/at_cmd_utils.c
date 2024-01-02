/**
 ****************************************************************************************
 *
 * @file at_cmd_utils.c
 *
 * @brief AT Command Utilities implementation.
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
 ****************************************************************************************
 */
#include "at_cmd_utils.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
uint8_t at_cmd_printf_bush(uint8_t *p_buff, const char *format, ...)
{
    char    str_temp[AT_CMD_BUFFER_SIZE_MAX];
    va_list ap;

    va_start(ap, format);
    vsprintf(str_temp, format, ap);
    va_end(ap);

    memcpy(p_buff, (uint8_t *)str_temp, strlen(str_temp));

    return (strlen(str_temp));
}

bool at_cmd_decimal_num_check(uint8_t *p_data, uint16_t length, uint32_t *p_num)
{
    if (0 == length)
    {
        return false;
    }

    *p_num = 0;

    for (uint8_t i = 0; i < length; i++)
    {
        if ('0' > p_data[i] || '9' < p_data[i])
        {
            return false;
        }
        else
        {
            *p_num = (*p_num * 10) + (p_data[i] - '0');
        }
    }

    return true;
}

bool at_cmd_hex_num_check(uint8_t *p_data, uint16_t length, uint32_t *p_num)
{
    if (2 >= length)
    {
        return false;
    }

    *p_num = 0;
    if ('0' != p_data[0]||('X' != p_data[1]&&'x' != p_data[1]))
    {
        return false;
    }
    for (uint8_t i = 2; i < length; i++)
    {
        if ('0' <= p_data[i] && '9' >= p_data[i])
        {
            *p_num = (*p_num * 16) + (p_data[i] - '0');
        }
        else if ('a' <= p_data[i] && 'f' >= p_data[i])
        {
            *p_num = (*p_num * 16) + (p_data[i] - 'a' + 10);
        }
        else if ('A' <= p_data[i] && 'F' >= p_data[i])
        {
            *p_num = (*p_num * 16) + (p_data[i] - 'A' + 10);
        }
        else
        {
            return false;
        }
    }

    return true;
}

at_cmd_error_t at_cmd_hal_err_convert(hal_status_t error_code)
{
    switch (error_code)
    {
        case HAL_OK:
            return AT_CMD_ERR_NO_ERROR;

        case HAL_ERROR:
        case HAL_BUSY:
        case HAL_TIMEOUT:
            return AT_CMD_ERR_HAL_ERROR;

        default:
            break;
    }

    return AT_CMD_ERR_NO_ERROR;
}

at_cmd_error_t at_cmd_ble_err_convert(sdk_err_t   error_code)
{
    switch (error_code)
    {
        case BLE_SUCCESS:
            return AT_CMD_ERR_NO_ERROR;

        case SDK_ERR_INVALID_PARAM:
        case BLE_GAP_ERR_INVALID_PARAM:
            return AT_CMD_ERR_INVALID_PARAM;

        default:
            return AT_CMD_ERR_OTHER_ERROR;
    }
}

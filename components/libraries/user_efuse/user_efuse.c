/**
 *****************************************************************************************
 *
 * @file user_efuse.c
 *
 * @brief efuse access function Implementation.
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
#include "user_efuse.h"
#include "string.h"
#include "grx_sys.h"
#include "grx_hal.h"

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint32_t user_efuse_write(uint8_t word_offset, uint32_t * efuse_value, uint8_t size_word)
{
    uint32_t ret = USER_EFUSE_ERROR_NONE;
    efuse_handle_t efuse_handle = {0};

    /*
     * Reserved For Furture : Because the macro USER_EFUSE_BASE_OFFSET equal zero now, it will cause compiler warning.
     * But in the furture, maybe USER_EFUSE_BASE_OFFSET will be another value.
     */
    if( /*(word_offset < USER_EFUSE_BASE_OFFSET) || */(word_offset > (USER_EFUSE_BASE_OFFSET + USER_EFUSE_SIZE)) || (efuse_value == NULL))
    {
        return USER_EFUSE_ERROR_INVALID_PARAM;
    }

    if((word_offset + size_word * 4) > (USER_EFUSE_BASE_OFFSET + USER_EFUSE_SIZE))
    {
        return USER_EFUSE_ERROR_INVALID_PARAM;
    }

    do {
        efuse_handle.p_instance = EFUSE;
        efuse_handle.init.info_mode = DISABLE;
        if(hal_efuse_init(&efuse_handle) != HAL_OK)
        {
            ret = USER_EFUSE_INIT_ERROR;
            break;
        }

        if(HAL_OK != hal_efuse_write(&efuse_handle, word_offset, efuse_value, size_word))
        {
            ret =  USER_EFUSE_WRITE_ERROR;
            break;
        }
    } while(0);
    hal_efuse_deinit(&efuse_handle);
    return ret;
}

uint32_t user_efuse_read(uint8_t word_offset, uint32_t *data, uint8_t size_word)
{
    uint32_t ret = USER_EFUSE_ERROR_NONE;
    efuse_handle_t efuse_handle = {0};

    /*
     * Reserved For Furture : Because the macro USER_EFUSE_BASE_OFFSET equal zero now, it will cause compiler warning.
     * But in the furture, maybe USER_EFUSE_BASE_OFFSET will be another value.
     */
    if( /*(word_offset < USER_EFUSE_BASE_OFFSET) || */(word_offset > (USER_EFUSE_BASE_OFFSET + USER_EFUSE_SIZE)) || (data == NULL))
    {
        return USER_EFUSE_ERROR_INVALID_PARAM;
    }

    if((word_offset + size_word * 4) > (USER_EFUSE_BASE_OFFSET + USER_EFUSE_SIZE))
    {
        return USER_EFUSE_ERROR_INVALID_PARAM;
    }

    do {
        efuse_handle.p_instance = EFUSE;
        efuse_handle.init.info_mode = DISABLE;
        if(hal_efuse_init(&efuse_handle) != HAL_OK)
        {
            ret = USER_EFUSE_INIT_ERROR;
            break;
        }

        if(HAL_OK != hal_efuse_read(&efuse_handle, word_offset, data, size_word))
        {
            return USER_EFUSE_READ_ERROR;
        }
    }while(0);
    hal_efuse_deinit(&efuse_handle);
    return ret;
}


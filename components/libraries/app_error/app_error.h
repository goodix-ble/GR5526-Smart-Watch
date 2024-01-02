/**
 ****************************************************************************************
 *
 * @file app_error.h
 *
 * @brief App Error API
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

#ifndef __APP_ERROR_H__
#define __APP_ERROR_H__

#include "ble_error.h"
#include <stdint.h>

/**
 * @defgroup APP_ERROR_MAROC Defines
 * @{
 */

/**@brief Macro for calling error handler function if supplied error code isn`t GR_SUCCESS. */
#define APP_ERROR_CHECK(ERROR_CODE)                         \
    do                                                      \
    {                                                       \
        if (ERROR_CODE != SDK_SUCCESS)                      \
        {                                                   \
            app_error_info_t error_info =                   \
            {                                               \
                .error_type       = APP_ERROR_API_RET,      \
                .value.error_code = ERROR_CODE,             \
                .file             = __FILE__,               \
                .func             = __FUNCTION__,           \
                .line             = __LINE__,               \
            };                                              \
            app_error_fault_handler(&error_info);           \
        }                                                   \
    } while(0)

/**@brief Macro for calling error handler function if supplied boolean value is false. */
#define APP_BOOL_CHECK(BOOL_VAL)                        \
    do                                                  \
    {                                                   \
        if (!BOOL_VAL)                                  \
        {                                               \
            app_error_info_t error_info =               \
            {                                           \
                .error_type = APP_ERROR_BOOL_COMPARE,   \
                .value.expr = #BOOL_VAL,                \
                .file       = __FILE__,                 \
                .func       = __FUNCTION__,             \
                .line       = __LINE__,                 \
            };                                          \
            app_error_fault_handler(&error_info);       \
        }                                               \
    } while(0)
/** @} */

/**
 * @defgroup APP_ERROR_ENUM Enumerations
 * @{
 */
/**@brief App error check type.*/
typedef enum
{
    APP_ERROR_API_RET,          /**< API return error code check failed. */
    APP_ERROR_BOOL_COMPARE,     /**< Bool value check failed. */
} app_error_type_t;
/** @} */

/**
 * @defgroup APP_ERROR_STRUCT Enumerations
 * @{
 */
/**@brief App error info.*/
typedef struct
{
    app_error_type_t error_type;     /**< Error occurred type. */
    union
    {
        sdk_err_t    error_code;     /**< Error code. */
        char const  *expr;           /**< Error expression. */
    } value;
    char const *file;                /**< The function in which file the error occurred. */
    char const *func;                /**< The function in which function the error occurred. */
    uint32_t    line;                /**< The line number where the error occurred. */
} app_error_info_t;
/** @} */

/**
 * @defgroup APP_ERROR_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief  Callback function for errors, and faults.
 *****************************************************************************************
 */
void app_error_fault_handler(app_error_info_t *p_error_info);
/** @} */


#endif

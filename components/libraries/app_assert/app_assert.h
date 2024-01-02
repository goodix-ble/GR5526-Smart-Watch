/**
 ****************************************************************************************
 *
 * @file app_assert.h
 *
 * @brief App Assert API
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

#ifndef __APP_ASSERT_H__
#define __APP_ASSERT_H__

#include "grx_sys.h"
#include <stdint.h>

/**
 * @defgroup APP_ASSERT_MAROC Defines
 * @{
 */
/**@brief Macro for calling error handler function if assert check failed. */
#define APP_ASSERT_CHECK(EXPR)                              \
    do                                                      \
    {                                                       \
        if (!(EXPR))                                        \
        {                                                   \
            app_assert_handler(#EXPR, __FILE__, __LINE__);  \
        }                                                   \
    } while(0)
/** @} */

/**
 * @defgroup APP_ASSERT_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Init user assert callbacks.
 *****************************************************************************************
 */
void app_assert_init(void);

/**
 *****************************************************************************************
 * @brief App assert handler.
 *
 * @param[in] expr:  Pxpression.
 * @param[in] file:  File name.
 * @param[in] line:  Line number.
 *****************************************************************************************
 */
void app_assert_handler(const char *expr, const char *file, int line);

/**
 *****************************************************************************************
 * @brief App assert warning callback.
 *
 * @param[in] param0:  Parameter0.
 * @param[in] param1:  Parameter1.
 * @param[in] file:    File name.
 * @param[in] line:    Line number.
 *****************************************************************************************
 */
void app_assert_warn_cb(int param0, int param1, const char *file, int line);

/**
 *****************************************************************************************
 * @brief App assert parameter callback.
 *
 * @param[in] param0:  Parameter0.
 * @param[in] param1:  Parameter1.
 * @param[in] file:    File name.
 * @param[in] line:    Line number.
 *****************************************************************************************
 */
void app_assert_param_cb(int param0, int param1, const char *file, int line);

/**
 *****************************************************************************************
 * @brief App assert error callback.
 *
 * @param[in] expr:   Pxpression.
 * @param[in] file:   File name.
 * @param[in] line:   Line number.
 *****************************************************************************************
 */
void app_assert_err_cb(const char *expr, const char *file, int line);


/** @} */

#endif 




/**
 *****************************************************************************************
 *
 * @file app_assert.c
 *
 * @brief App Assert Implementation.
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

#define APP_LOG_TAG "app_assert.c"

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_assert.h"
#include "app_log.h"
#include <cmsis_compiler.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>


/*
 * DEFINITIONS
 *****************************************************************************************
 */
#define APP_ASSERT_FILE_NAME_LEN             64             /**< Length of file name. */
#define APP_ASSERT_EXPR_NAME_LEN             64             /**< Length of expression. */
#define APP_ASSERT_ERR_MAGIC_1               0xAAAAAAAA     /**< App assert error magic 1. */
#define APP_ASSERT_ERR_MAGIC_2               0xBBBBBBBB     /**< App assert error magic 2. */
#define APP_ASSERT_ERR_MAGIC_3               0xCCCCCCCC     /**< App assert error magic 3. */
#define APP_ASSERT_ERR_MAGIC_4               0xDDDDDDDD     /**< App assert error magic 4. */
#define APP_ASSERT_PARAM_MAGIC_1             0x55555555     /**< App assert parameter magic 1. */
#define APP_ASSERT_PARAM_MAGIC_2             0x66666666     /**< App assert parameter magic 2. */
#define APP_ASSERT_PARAM_MAGIC_3             0x77777777     /**< App assert parameter magic 3. */
#define APP_ASSERT_PARAM_MAGIC_4             0x88888888     /**< App assert parameter magic 4. */
#define APP_ASSERT_WARN_MAGIC_1              0x11111111     /**< App assert error magic 1. */
#define APP_ASSERT_WARN_MAGIC_2              0x22222222     /**< App assert error magic 2. */
#define APP_ASSERT_WARN_MAGIC_3              0x33333333     /**< App assert error magic 3. */
#define APP_ASSERT_WARN_MAGIC_4              0x44444444     /**< App assert error magic 4. */

#define APP_ASSERT_ERROR                     0x00           /**< Assert type: Error. */
#define APP_ASSERT_WARNING                   0x01           /**< Assert type: Waring. */
#define APP_ASSERT_PARAM                     0x02           /**< Assert type: Parameter check. */

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Assert information save. */
struct app_asser_info_t
{
    char    file_name[APP_ASSERT_FILE_NAME_LEN];
    int     magic1;
    int     file_line;
    int     magic2;
    int     param0;
    int     magic3;
    int     param1;
    int     magic4;
    char    expr[APP_ASSERT_EXPR_NAME_LEN];
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct app_asser_info_t  s_assert_info;

static sys_assert_cb_t s_assert_cbs =
{
    .assert_err_cb   = app_assert_err_cb ,
    .assert_param_cb = app_assert_param_cb,
    .assert_warn_cb  = app_assert_warn_cb,
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief App assert infomation output.
 *
 * @param[in] assert_type: Assert type.
 *****************************************************************************************
 */
static void app_assert_info_output(uint8_t assert_type)
{
    char  assert_info[1024]  = {0};

    s_assert_info.file_name[APP_ASSERT_FILE_NAME_LEN - 1] = ' ';
    s_assert_info.expr[APP_ASSERT_FILE_NAME_LEN - 1]      = ' ';

    if (APP_ASSERT_ERROR == assert_type)
    {
        sprintf(assert_info,"[ERROR] %s", s_assert_info.expr);
    }
    else if (APP_ASSERT_WARNING == assert_type)
    {
        sprintf(assert_info,"[WARNING] Param0:%d,Param1:%d", s_assert_info.param0, s_assert_info.param1);
    }
    else if (APP_ASSERT_PARAM == assert_type)
    {
        sprintf(assert_info,"[PARAM] Param0:%d,Param1:%d", s_assert_info.param0, s_assert_info.param1);
    }

    app_log_output(APP_LOG_LVL_ERROR,
                   APP_LOG_TAG,
                   s_assert_info.file_name,
                   "",
                   s_assert_info.file_line,
                   "%s",
                   assert_info);

    app_log_flush();
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
__WEAK void app_assert_warn_cb(int param0, int param1, const char *file, int line)
{
    uint32_t file_name_len;

    file_name_len = (APP_ASSERT_FILE_NAME_LEN < strlen(file)) ? APP_ASSERT_FILE_NAME_LEN : strlen(file);

    memset(&s_assert_info, 0, sizeof(s_assert_info));
    memcpy(s_assert_info.file_name, file, file_name_len);

    s_assert_info.magic1    = APP_ASSERT_WARN_MAGIC_1;
    s_assert_info.file_line = line;
    s_assert_info.magic2    = APP_ASSERT_WARN_MAGIC_2;
    s_assert_info.param0    = param0;
    s_assert_info.magic3    = APP_ASSERT_WARN_MAGIC_3;
    s_assert_info.param1    = param1;
    s_assert_info.magic4    = APP_ASSERT_WARN_MAGIC_4;

    // Also can store assert info to flash
    app_assert_info_output(APP_ASSERT_WARNING);
}

__WEAK void app_assert_param_cb(int param0, int param1, const char *file, int line)
{
    __disable_irq(); 

    uint32_t file_name_len;

    file_name_len = (APP_ASSERT_FILE_NAME_LEN < strlen(file)) ? APP_ASSERT_FILE_NAME_LEN : strlen(file);

    memset(&s_assert_info, 0, sizeof(s_assert_info));
    memcpy(s_assert_info.file_name, file, file_name_len);

    s_assert_info.magic1    = (int)APP_ASSERT_PARAM_MAGIC_1;
    s_assert_info.file_line = line;
    s_assert_info.magic2    = (int)APP_ASSERT_PARAM_MAGIC_2;
    s_assert_info.param0    = param0;
    s_assert_info.magic3    = (int)APP_ASSERT_PARAM_MAGIC_3;
    s_assert_info.param1    = param1;
    s_assert_info.magic4    = (int)APP_ASSERT_PARAM_MAGIC_4;

    // Also can store assert info to flash
    app_assert_info_output(APP_ASSERT_PARAM);

    while(1);
}

__WEAK void app_assert_err_cb(const char *expr, const char *file, int line)
{
    __disable_irq(); 

    uint32_t file_name_len;
    uint32_t expre_len;

    file_name_len = (APP_ASSERT_FILE_NAME_LEN < strlen(file)) ? APP_ASSERT_FILE_NAME_LEN : strlen(file);
    expre_len     = (APP_ASSERT_EXPR_NAME_LEN < strlen(expr)) ? APP_ASSERT_EXPR_NAME_LEN : strlen(expr);

    memset(&s_assert_info, 0, sizeof(s_assert_info));
    memcpy(s_assert_info.file_name, file, file_name_len);
    memcpy(s_assert_info.expr, expr, expre_len);

    s_assert_info.magic1    = (int)APP_ASSERT_ERR_MAGIC_1;
    s_assert_info.file_line = line;
    s_assert_info.magic2    = (int)APP_ASSERT_ERR_MAGIC_2;
    s_assert_info.magic3    = (int)APP_ASSERT_ERR_MAGIC_3;
    s_assert_info.magic4    = (int)APP_ASSERT_ERR_MAGIC_4;

    // Also can store assert info to flash
    app_assert_info_output(APP_ASSERT_ERROR);
    while(1);
}

void app_assert_init(void)
{
    sys_assert_cb_register(&s_assert_cbs);
}

void app_assert_handler(const char *expr, const char *file, int line)
{
    if (s_assert_cbs.assert_err_cb)
    {
        s_assert_cbs.assert_err_cb(expr, file, line);
    }
}



/**
 *****************************************************************************************
 *
 * @file fault_trace.c
 *
 * @brief App Log Implementation.
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
#include "fault_trace.h"
#include "app_assert.h"
#include "grx_sys.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#if SYS_FAULT_TRACE_ENABLE

/*
 * DEFINES
 *****************************************************************************************
 */
#define FAULT_INFO_NVDS_TAG_BASE    0xA000
#define FAULT_INFO_DB_ENV_TAG       FAULT_INFO_NVDS_TAG_BASE + 0
#define FAULT_INFO_DB_ENV_LEN       sizeof(fault_info_db_env_t)
#define FAULT_INFO_LEN_MAX          1024
#define ASSERT_FILE_NAME_LEN        64
#define ASSERT_EXPR_NAME_LEN        64

#define ASSERT_ERROR                0x01           /**< Assert type: Error. */
#define ASSERT_WARNING              0x02           /**< Assert type: Waring. */
#define ASSERT_PARAM                0x03           /**< Assert type: Parameter check. */

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Assert information save. */
typedef struct
{
    uint8_t assert_type;
    char    file_name[ASSERT_FILE_NAME_LEN];
    int     file_line;
    int     param0;
    int     param1;
    char    expr[ASSERT_EXPR_NAME_LEN];
} assert_info_t;



/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
static fault_info_db_env_t  s_fault_info_db_env;
static char                 s_fault_info[FAULT_INFO_LEN_MAX] = {0};
static assert_info_t        s_assert_info;



/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Assert info save.
 *
 * @param[in] p_assert_info: Pointer to assert info.
 *****************************************************************************************
 */
SECTION_RAM_CODE static void assert_info_save(assert_info_t *p_assert_info)
{
    memset(s_fault_info, 0, FAULT_INFO_LEN_MAX);

    p_assert_info->file_name[ASSERT_FILE_NAME_LEN - 1] = ' ';
    p_assert_info->expr[ASSERT_EXPR_NAME_LEN - 1]      = ' ';


    if (ASSERT_ERROR == p_assert_info->assert_type)
    {
        sprintf(s_fault_info,
                "(%s: %d) [ERROR] %s\r\n",
                p_assert_info->file_name,
                p_assert_info->file_line,
                p_assert_info->expr);
    }
    else if (ASSERT_WARNING == p_assert_info->assert_type)
    {
        sprintf(s_fault_info,
                "(%s: %d) [WARNING] Param0:%d,Param1:%d\r\n",
                p_assert_info->file_name,
                p_assert_info->file_line,
                p_assert_info->param0,
                p_assert_info->param1);
    }
    else if (ASSERT_PARAM == p_assert_info->assert_type)
    {
        sprintf(s_fault_info,"(%s: %d) [PARAM] Param0:%d,Param1:%d\r\n",
                p_assert_info->file_name,
                p_assert_info->file_line,
                p_assert_info->param0,
                p_assert_info->param1);
    }

    fault_db_record_add((uint8_t *)s_fault_info, strlen(s_fault_info));
}

/**
 *****************************************************************************************
 * @brief Assert warning callback.
 *
 * @param[in] param0: Parameter 0.
 * @param[in] param1: Parameter 1.
 * @param[in] file:   File name.
 * @param[in] line:  Line number.
 *****************************************************************************************
 */
SECTION_RAM_CODE void app_assert_warn_cb(int param0, int param1, const char *file, int line)
{
    uint32_t  file_name_len = 0;

    file_name_len = (ASSERT_FILE_NAME_LEN < strlen(file)) ? ASSERT_FILE_NAME_LEN : strlen(file);

    memset(&s_assert_info, 0, sizeof(assert_info_t));
    memcpy(s_assert_info.file_name, file, file_name_len);

    s_assert_info.assert_type = ASSERT_WARNING;
    s_assert_info.file_line   = line;
    s_assert_info.param0      = param0;
    s_assert_info.param1      = param1;

    assert_info_save(&s_assert_info);
}

/**
 *****************************************************************************************
 * @brief Assert param callback.
 *
 * @param[in] param0: Parameter 0.
 * @param[in] param1: Parameter 1.
 * @param[in] file:   File name.
 * @param[in] line:  Line number.
 *****************************************************************************************
 */
SECTION_RAM_CODE void app_assert_param_cb(int param0, int param1, const char *file, int line)
{
    __disable_irq();

    uint32_t       file_name_len = 0;

    file_name_len = (ASSERT_FILE_NAME_LEN < strlen(file)) ? ASSERT_FILE_NAME_LEN : strlen(file);

    memset(&s_assert_info, 0, sizeof(assert_info_t));
    memcpy(s_assert_info.file_name, file, file_name_len);

    s_assert_info.assert_type = ASSERT_PARAM;
    s_assert_info.file_line   = line;
    s_assert_info.param0      = param0;
    s_assert_info.param1      = param1;

    assert_info_save(&s_assert_info);

    while(1);
}

/**
 *****************************************************************************************
 * @brief Assert error callback.
 *
 * @param[in] expr:   Pxpression.
 * @param[in] file:   File name.
 * @param[in] line:  Line number.
 *****************************************************************************************
 */
SECTION_RAM_CODE void app_assert_err_cb(const char *expr, const char *file, int line)
{
    __disable_irq();

    uint32_t       expre_len     = 0;
    uint32_t       file_name_len = 0;

    file_name_len = (ASSERT_FILE_NAME_LEN < strlen(file)) ? ASSERT_FILE_NAME_LEN : strlen(file);
    expre_len     = (ASSERT_EXPR_NAME_LEN < strlen(expr)) ? ASSERT_EXPR_NAME_LEN : strlen(expr);

    memset(&s_assert_info, 0, sizeof(assert_info_t));
    memcpy(s_assert_info.file_name, file, file_name_len);
    memcpy(s_assert_info.expr, expr, expre_len);

    s_assert_info.assert_type = ASSERT_ERROR;
    s_assert_info.file_line   = line;

    assert_info_save(&s_assert_info);
    while(1);
}


/**
 *****************************************************************************************
 * @brief Update fault info record database environment variables.
 *****************************************************************************************
 */
static sdk_err_t fault_info_db_env_update(void)
{
    if (nvds_put(FAULT_INFO_DB_ENV_TAG, FAULT_INFO_DB_ENV_LEN, (uint8_t *)&s_fault_info_db_env))
    {
        return SDK_ERR_SDK_INTERNAL;
    }
    else
    {
        return SDK_SUCCESS;
    }
}

/**
 *****************************************************************************************
 * @brief Database is already exist or not.
 *
 * @return True or False.
 *****************************************************************************************
 */
static bool fault_info_db_is_available(void)
{
    fault_info_db_env_t  db_env;
    uint16_t             length = FAULT_INFO_DB_ENV_LEN;

    return  (NVDS_TAG_NOT_EXISTED == nvds_get(FAULT_INFO_DB_ENV_TAG, &length, (uint8_t *)&db_env)) ? false : true;
}

/**
 *****************************************************************************************
 * @brief Get fault info record database environment variables.
 *
 * @return Result of get.
 *****************************************************************************************
 */
static sdk_err_t fault_info_db_get(void)
{
    fault_info_db_env_t  db_env;
    uint16_t             length = FAULT_INFO_DB_ENV_LEN;

    if (nvds_get(FAULT_INFO_DB_ENV_TAG, &length, (uint8_t *)&db_env))
    {
        return SDK_ERR_SDK_INTERNAL;
    }
    else
    {
        memcpy(&s_fault_info_db_env, &db_env, FAULT_INFO_DB_ENV_LEN);
        return SDK_SUCCESS;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
sdk_err_t fault_trace_db_init(void)
{
    app_assert_init();

    if (fault_info_db_is_available())
    {
        return fault_info_db_get();
    }
    else
    {
        for (uint8_t i = 0; i < FAULT_INFO_RECORDS_MAX; i++)
        {
            s_fault_info_db_env.rec_tag[i] = 0xFFFF;
        }

        s_fault_info_db_env.rec_num    = 0;
        s_fault_info_db_env.usable_tag = FAULT_INFO_DB_ENV_TAG + 1;

        return fault_info_db_env_update();
    }
}

sdk_err_t fault_db_record_add(const uint8_t *p_data, uint16_t length)
{
    sdk_err_t error_code;

    if (FAULT_INFO_RECORDS_MAX <= s_fault_info_db_env.rec_num)
    {
        s_fault_info_db_env.usable_tag = s_fault_info_db_env.rec_tag[0];
        s_fault_info_db_env.rec_tag[FAULT_INFO_RECORDS_MAX - 1] = 0xFFFF;
        s_fault_info_db_env.rec_num--;
        memcpy(&s_fault_info_db_env.rec_tag[0],
               &s_fault_info_db_env.rec_tag[1],
               (FAULT_INFO_RECORDS_MAX - 1) * sizeof(NvdsTag_t));
    }

    if (nvds_put(s_fault_info_db_env.usable_tag, length, p_data))
    {
        return SDK_ERR_SDK_INTERNAL;
    }

    if (FAULT_INFO_RECORDS_MAX <= s_fault_info_db_env.rec_num)
    {
        s_fault_info_db_env.rec_num++;
        s_fault_info_db_env.rec_tag[FAULT_INFO_RECORDS_MAX - 1] = s_fault_info_db_env.usable_tag;
    }
    else
    {
        s_fault_info_db_env.rec_tag[s_fault_info_db_env.rec_num] = s_fault_info_db_env.usable_tag;
        s_fault_info_db_env.rec_num++;
        s_fault_info_db_env.usable_tag++;
    }

    error_code = fault_info_db_env_update();

    if (error_code)
    {
        s_fault_info_db_env.rec_num--;
        s_fault_info_db_env.usable_tag--;
        s_fault_info_db_env.rec_tag[s_fault_info_db_env.rec_num] = 0xffff;
    }

    return error_code;
}

uint8_t fault_db_records_num_get(void)
{
    return s_fault_info_db_env.rec_num;
}

uint32_t fault_db_records_total_len_get(void)
{
    uint32_t total_len = 0;

    for (uint8_t i = 0; i <= s_fault_info_db_env.rec_num; i++)
    {
        total_len += nvds_tag_length(s_fault_info_db_env.rec_tag[i]);
    }

    return total_len;
}


sdk_err_t fault_db_records_dump(uint8_t *p_buffer, uint32_t *p_length)
{
    uint16_t dump_len = 0;
    uint16_t resi_len = 0;
    uint32_t buff_len = *p_length;

    if (buff_len < fault_db_records_total_len_get())
    {
        return SDK_ERR_INVALID_BUFF_LENGTH;
    }

    for (uint8_t i = 0; i < s_fault_info_db_env.rec_num; i++)
    {
        resi_len = ((buff_len - dump_len) >= FAULT_INFO_LEN_MAX) ? FAULT_INFO_LEN_MAX : buff_len - dump_len;

        if (nvds_get(s_fault_info_db_env.rec_tag[i], &resi_len, &p_buffer[dump_len]))
        {
            *p_length = dump_len;

            return SDK_ERR_SDK_INTERNAL;
        }
        else
        {
            dump_len += resi_len;
        }
    }

    *p_length = dump_len;

    return SDK_SUCCESS;
}

sdk_err_t fault_db_record_clear(void)
{
    for (uint8_t i = 0; i < s_fault_info_db_env.rec_num; i++)
    {
        nvds_del(s_fault_info_db_env.rec_tag[i]);
        s_fault_info_db_env.rec_tag[i] = 0xFFFF;
    }

    s_fault_info_db_env.rec_num    = 0;
    s_fault_info_db_env.usable_tag = FAULT_INFO_DB_ENV_TAG + 1;

    return fault_info_db_env_update();
}

#if defined ( __CC_ARM )
void hardfault_trace_handler(unsigned int sp)
{
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;

    stacked_r0  = ((unsigned long *)sp)[0];
    stacked_r1  = ((unsigned long *)sp)[1];
    stacked_r2  = ((unsigned long *)sp)[2];
    stacked_r3  = ((unsigned long *)sp)[3];
    stacked_r12 = ((unsigned long *)sp)[4];
    stacked_lr  = ((unsigned long *)sp)[5];
    stacked_pc  = ((unsigned long *)sp)[6];
    stacked_psr = ((unsigned long *)sp)[7];

    memset(s_fault_info, 0, FAULT_INFO_LEN_MAX);

    sprintf(s_fault_info, "HARDFAULT CALLSTACK INFO: R0-%08X R1-%08X R2-%08X R3-%08X R12-%08X LR-%08X PC-%08X XPSR-%08X\r\n",
            stacked_r0, stacked_r1, stacked_r2, stacked_r3, stacked_r12, stacked_lr, stacked_pc, stacked_psr);

    fault_db_record_add((uint8_t *)s_fault_info, strlen(s_fault_info));
}

#elif defined ( __GNUC__ )

void hardfault_trace_handler(unsigned int *args)
{
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;

    stacked_r0  = ((unsigned long)args[0]);
    stacked_r1  = ((unsigned long)args[1]);
    stacked_r2  = ((unsigned long)args[2]);
    stacked_r3  = ((unsigned long)args[3]);

    stacked_r12 = ((unsigned long)args[4]);
    stacked_lr  = ((unsigned long)args[5]);
    stacked_pc  = ((unsigned long)args[6]);
    stacked_psr = ((unsigned long)args[7]);

    memset(s_fault_info, 0, FAULT_INFO_LEN_MAX);

    sprintf(s_fault_info, "HARDFAULT CALLSTACK INFO: R0-%08X R1-%08X R2-%08X R3-%08X R12-%08X LR-%08X PC-%08X XPSR-%08X\r\n",
            stacked_r0, stacked_r1, stacked_r2, stacked_r3, stacked_r12, stacked_lr, stacked_pc, stacked_psr);

    fault_db_record_add((uint8_t *)s_fault_info, strlen(s_fault_info));
}
#endif

#endif


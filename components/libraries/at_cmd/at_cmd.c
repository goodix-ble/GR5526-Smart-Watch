/**
 ****************************************************************************************
 *
 * @file AT_CMD.c
 *
 * @brief AT Command implementation.
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
#include "at_cmd.h"
#include "at_cmd_utils.h"
#include "cmsis_compiler.h"
#include <string.h>

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief AT Command environment variable. */
struct at_cmd_env_t
{
    uint8_t                  cmd_num;
    at_cmd_src_t             cmd_src;
    at_cmd_attr_t           *p_cmd_attr;
    at_cmd_time_callback_t   cmd_time_cb;
    at_cmd_cplt_callback_t   cmd_cplt_cb;
    at_cmd_state_t           cmd_state;
};

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
static struct at_cmd_env_t   s_at_cmd_env;
static        at_cmd_parse_t s_parse_rlt;

static uint8_t at_cmd_inp_buff[AT_CMD_BUFFER_SIZE_MAX];
static uint8_t at_cmd_rsp_buff[AT_CMD_BUFFER_SIZE_MAX];

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Check AT CMD input integrity.
 *
 * @param[in]  p_data:      Pointer to input data.
 * @param[in]  length:      Length of input data.
 * @param[out] p_parse_rlt: Pointer to parse result.
 *
 * @return Result of check.
 *****************************************************************************************
 */
static bool at_cmd_integrity_check(const uint8_t *p_data, uint16_t length, at_cmd_parse_t *p_parse_rlt)
{
    if (0 != memcmp(p_data, "AT:", 3))
    {
        return false;
    }
    for (uint8_t i = 1; i < length; i++)
    {
        if (0x0a == p_data[i] && 0x0d == p_data[i - 1])
        {
            p_parse_rlt->buff_length = i + 1;
            memcpy(p_parse_rlt->p_buff, p_data, p_parse_rlt->buff_length);
            return true;
        }
    }

    return false;
}

/**
 *****************************************************************************************
 * @brief Get args of AT CMD.
 *
 * @param[out] p_parse_rlt: Pointer to parse result.
 *****************************************************************************************
 */
static void at_cmd_args_get(at_cmd_parse_t *p_parse_rlt)
{
    p_parse_rlt->arg_count = 0;

    uint16_t first_arg_idx = 0;

    p_parse_rlt->cmd_tag_idx = 3;

    for (uint16_t i = p_parse_rlt->cmd_tag_idx; i <= (p_parse_rlt->buff_length - 2); i++)
    {
        if (i == (p_parse_rlt->buff_length - 2))
        {
            p_parse_rlt->cmd_tag_length = i - p_parse_rlt->cmd_tag_idx;
            p_parse_rlt->arg_count      = 0;
            return;
        }

        if ('=' == p_parse_rlt->p_buff[i])
        {
            p_parse_rlt->cmd_tag_length = i - p_parse_rlt->cmd_tag_idx + 1;
            first_arg_idx = i + 1;
            break;
        }
    }

    p_parse_rlt->arg_idx[0] = first_arg_idx;

    for (uint16_t i = first_arg_idx; i <= (p_parse_rlt->buff_length - 2); i++)
    {
        if ((':' == p_parse_rlt->p_buff[i]) || (',' == p_parse_rlt->p_buff[i]))
        {
            p_parse_rlt->arg_length[p_parse_rlt->arg_count] = i - p_parse_rlt->arg_idx[p_parse_rlt->arg_count];
            p_parse_rlt->arg_count++;
            p_parse_rlt->arg_idx[p_parse_rlt->arg_count] = i + 1;
        }
        else if (i == (p_parse_rlt->buff_length - 2))
        {
            p_parse_rlt->arg_length[p_parse_rlt->arg_count] = i - p_parse_rlt->arg_idx[p_parse_rlt->arg_count];
            p_parse_rlt->arg_count++;
        }
    }
}

/**
 *****************************************************************************************
 * @brief Get ID of AT CMD.
 *
 * @param[out] p_parse_rlt: Pointer to parse result.
 *****************************************************************************************
 */
static void at_cmd_id_get(at_cmd_parse_t *p_parse_rlt)
{
    p_parse_rlt->cmd_id = AT_CMD_INVALID;

    for (uint8_t i = 0; i < s_at_cmd_env.cmd_num; i++)
    {
        if (p_parse_rlt->cmd_tag_length == s_at_cmd_env.p_cmd_attr[i].cmd_tag_length)
        {
            if (0 == memcmp(&p_parse_rlt->p_buff[p_parse_rlt->cmd_tag_idx],
                            s_at_cmd_env.p_cmd_attr[i].cmd_tag_str,
                            p_parse_rlt->cmd_tag_length))
            {
                p_parse_rlt->cmd_id  = s_at_cmd_env.p_cmd_attr[i].cmd_id;
                p_parse_rlt->cmd_idx = i;
                break;
            }
        }
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void at_cmd_init(at_cmd_init_t *p_cmd_init)
{
    memset(&s_at_cmd_env, 0, sizeof(s_at_cmd_env));
    memset(&s_parse_rlt, 0, sizeof(at_cmd_parse_t));

    s_at_cmd_env.cmd_num     = p_cmd_init->cmd_num;
    s_at_cmd_env.p_cmd_attr  = p_cmd_init->p_cmd_attr;
    s_at_cmd_env.cmd_time_cb = p_cmd_init->cmd_time_cb;
    s_at_cmd_env.cmd_cplt_cb = p_cmd_init->cmd_cplt_cb;
    s_at_cmd_env.cmd_state   = AT_CMD_IN_READY_PARSE;
    s_at_cmd_env.cmd_src     = AT_CMD_SRC_UART;

    s_parse_rlt.p_buff      = at_cmd_inp_buff;
    s_parse_rlt.buff_length = 0;
}

void at_cmd_parse(at_cmd_src_t cmd_src, const uint8_t *p_data, uint16_t length)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    bool reset = false;
    static at_cmd_parse_t pre_parse_rlt;

    s_at_cmd_env.cmd_src = cmd_src;
    if (pre_parse_rlt.cmd_id == AT_CMD_CONN_INIT)
    {
        s_at_cmd_env.cmd_state = AT_CMD_IN_READY_PARSE;
        reset = true;
    }
    
    // Check parse cmd is allowed or not
    if (AT_CMD_IN_READY_PARSE != s_at_cmd_env.cmd_state)
    {
        cmd_rsp.error_code = AT_CMD_ERR_PARSE_NOT_ALLOWED;
        at_cmd_execute_cplt(&cmd_rsp);
        return;
    }
    else
    {
        s_at_cmd_env.cmd_state = AT_CMD_IN_PARSING;
    }

    // Check cmd input is integrity or not
    if (!at_cmd_integrity_check(p_data, length, &s_parse_rlt))
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_INPUT;
        at_cmd_execute_cplt(&cmd_rsp);
        return;
    }

    // Get cmd parameters
    at_cmd_args_get(&s_parse_rlt);

    // Get cmd Id
    at_cmd_id_get(&s_parse_rlt);
    
    if (reset && s_parse_rlt.cmd_id == AT_CMD_CONN_CANCEL)
    {
        cmd_rsp.error_code = AT_CMD_ERR_NO_ERROR;
        cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "start cancel connect...");
        at_cmd_execute_cplt(&cmd_rsp);
    }

    // Check cmd id is valid or not
    if (AT_CMD_INVALID == s_parse_rlt.cmd_id)
    {
        cmd_rsp.error_code = AT_CMD_ERR_UNSUPPORTED_CMD;
        at_cmd_execute_cplt(&cmd_rsp);
        return;
    }

    s_at_cmd_env.cmd_state = AT_CMD_IN_WAITE_EXECUTE;
    pre_parse_rlt = s_parse_rlt;
}

void at_cmd_execute_timing_process(void)
{
    AT_CMD_RSP_DEF(cmd_rsp);

    if (AT_CMD_IN_READY_PARSE != s_at_cmd_env.cmd_state)
    {
        s_at_cmd_env.cmd_state = AT_CMD_IN_READY_PARSE;

        cmd_rsp.error_code = AT_CMD_ERR_TIMEOUT;
        at_cmd_execute_cplt(&cmd_rsp);
    }
}

void at_cmd_execute_cplt(at_cmd_rsp_t *p_cmd_rsp)
{
    uint8_t length = 0;

    if (AT_CMD_ERR_NO_ERROR != p_cmd_rsp->error_code)
    {
        switch(p_cmd_rsp->error_code)
        {
            case AT_CMD_ERR_INVALID_INPUT:
                length = at_cmd_printf_bush(at_cmd_rsp_buff, "ERR: Invalid input.");
                break;
            case AT_CMD_ERR_UNSUPPORTED_CMD:
                length = at_cmd_printf_bush(at_cmd_rsp_buff, "ERR: Unsupported AT CMD.");
                break;
            case AT_CMD_ERR_PARSE_NOT_ALLOWED:
                length = at_cmd_printf_bush(at_cmd_rsp_buff, "ERR: No allowed parse state.");
                break;
            case AT_CMD_ERR_CMD_REQ_ALLOWED:
                length = at_cmd_printf_bush(at_cmd_rsp_buff, "ERR: Command request is not allowed.");
                break;
            case AT_CMD_ERR_NO_CMD_HANDLER:
                length = at_cmd_printf_bush(at_cmd_rsp_buff, "ERR: No AT CMD handler.");
                break;
            case AT_CMD_ERR_INVALID_PARAM:
                length = at_cmd_printf_bush(at_cmd_rsp_buff, "ERR: Invalid parameters.");
                break;
            case AT_CMD_ERR_HAL_ERROR:
                length = at_cmd_printf_bush(at_cmd_rsp_buff, "ERR: Hal error.");
                break;
            case AT_CMD_ERR_TIMEOUT:
                length = at_cmd_printf_bush(at_cmd_rsp_buff, "ERR: AT CMD execute timeout.");
                break;
            case AT_CMD_ERR_OTHER_ERROR:
                length = at_cmd_printf_bush(at_cmd_rsp_buff, "ERR: Other error code.");
                break;
            default:
                break;
        }
    }
    else
    {
        memcpy(at_cmd_rsp_buff, p_cmd_rsp->data, p_cmd_rsp->length);
        length = p_cmd_rsp->length;
    }

    at_cmd_rsp_buff[length]     = 0x0d;
    at_cmd_rsp_buff[length + 1] = 0x0a;

    if (s_at_cmd_env.cmd_cplt_cb)
    {
        if (AT_CMD_SRC_UART == s_at_cmd_env.cmd_src)
        {
            s_at_cmd_env.cmd_cplt_cb(AT_CMD_RSP_DEST_UART, at_cmd_rsp_buff, length + 2);
        }
        else if (AT_CMD_SRC_BLE == s_at_cmd_env.cmd_src)
        {
            s_at_cmd_env.cmd_cplt_cb(AT_CMD_RSP_DEST_BLE, at_cmd_rsp_buff, length + 2);
        }
    }

    s_at_cmd_env.cmd_state = AT_CMD_IN_READY_PARSE;
    if (AT_CMD_ERR_TIMEOUT == p_cmd_rsp->error_code &&
        s_at_cmd_env.p_cmd_attr[s_parse_rlt.cmd_idx].cmd_timeout_str)
    {
        char *cmd_timeout_str = s_at_cmd_env.p_cmd_attr[s_parse_rlt.cmd_idx].cmd_timeout_str;
        uint16_t length = strlen(cmd_timeout_str);
        at_cmd_parse(s_at_cmd_env.cmd_src, (const uint8_t *)cmd_timeout_str, length);
    }
}

void at_cmd_schedule(void)
{
    if (AT_CMD_IN_WAITE_EXECUTE == s_at_cmd_env.cmd_state)
    {
        s_at_cmd_env.cmd_state = AT_CMD_IN_EXECUTING;

        if (s_at_cmd_env.p_cmd_attr[s_parse_rlt.cmd_idx].cmd_handler)
        {
            if (s_at_cmd_env.cmd_time_cb)
            {
                s_at_cmd_env.cmd_time_cb();
            }

            s_at_cmd_env.p_cmd_attr[s_parse_rlt.cmd_idx].cmd_handler(&s_parse_rlt);
        }
        else
        {
            AT_CMD_RSP_DEF(cmd_rsp);
            cmd_rsp.error_code = AT_CMD_ERR_NO_CMD_HANDLER;
            at_cmd_execute_cplt(&cmd_rsp);
        }
    }
}

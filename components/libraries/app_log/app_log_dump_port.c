/**
 *****************************************************************************************
 *
 * @file app_log_store_dump_port.c
 *
 * @brief App Log store dump port Implementation.
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

#include "app_log_dump_port.h"
#include "app_log_store.h"
#include "lms.h"
#include "app_error.h"
#include "custom_config.h"

#if APP_LOG_STORE_ENABLE
/*
 * DEFINE
 *****************************************************************************************
 */


/*
 * STRUCTURES
 *****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static void lms_log_length_send(uint32_t log_len);
static void lms_data_send(uint8_t *p_data, uint16_t length);
static void lms_data_send_finish_cb(void);
static app_log_dump_cbs_t s_dump_cbs = 
{
    .dump_start_cb   = lms_log_length_send,
    .dump_process_cb = lms_data_send,
    .dump_finish_cb  = lms_data_send_finish_cb
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint8_t *s_send_data_addr = NULL;
static uint16_t s_send_block_size = 0;
static uint32_t s_sended_len = 0;
static uint16_t s_once_size = 20;
static uint32_t s_checksum = 0;

static void lms_log_length_send(uint32_t log_len)
{
    sdk_err_t error_code;
    uint8_t arr[10] = {0};
    arr[0] = 0x81;
    arr[1] = 0;
    arr[2] = log_len;
    arr[3] = log_len >> 8;
    arr[4] = log_len >> 16;
    arr[5] = log_len >> 24;
    error_code = lms_notify_cmd(0, arr, 6);
    APP_ERROR_CHECK(error_code);
}

static void lms_data_send(uint8_t *p_data, uint16_t length)
{
    for(uint16_t i = 0; i < length; i++)
        s_checksum += p_data[i];
    
    sdk_err_t error_code;
    s_send_block_size = length;
    s_send_data_addr = p_data;
    
    if (s_send_block_size > s_once_size)
    {
        s_sended_len += s_once_size;
        error_code = lms_notify_data(0, s_send_data_addr, s_once_size);
        APP_ERROR_CHECK(error_code);
    }
    else
    {
        s_sended_len += s_send_block_size;
        error_code = lms_notify_data(0, s_send_data_addr, s_send_block_size);
        APP_ERROR_CHECK(error_code);
    }
}
static void lms_data_continue_send(void)
{
    if (NULL == s_send_data_addr)
    {
        return;
    }
    sdk_err_t error_code;
    uint16_t remain = s_send_block_size - s_sended_len;
    uint16_t offset = 0;

    if (remain > s_once_size)
    {
        offset = s_sended_len;
        s_sended_len += s_once_size;

        error_code = lms_notify_data(0, &s_send_data_addr[offset], s_once_size);
        APP_ERROR_CHECK(error_code);
    }
    else if(remain > 0)
    {
        offset = s_sended_len;
        s_sended_len += remain;
        error_code = lms_notify_data(0, &s_send_data_addr[offset],remain);
        APP_ERROR_CHECK(error_code);
    }
    else
    {
        //vPortFree(s_send_data_addr);
        s_send_data_addr = NULL;
        s_send_block_size = 0;
        s_sended_len = 0;
        app_log_dump_continue();
    }
}

static void lms_data_send_finish_cb(void)
{
    uint8_t arr[5] = {0};
    arr[0] = 0xc1;
    arr[1] = s_checksum;
    arr[2] = s_checksum >> 8;
    arr[3] = s_checksum >> 16;
    arr[4] = s_checksum >> 24;
    lms_notify_cmd(0, arr, 5);
    s_checksum = 0;
}

static void lms_service_process_event(lms_evt_t *p_evt)
{
    uint8_t arr[10] = {0};
    sdk_err_t error_code;
    switch (p_evt->evt_type)
    {
        case LMS_EVT_CMD_RECEIVE_DATA:
            if (1 == p_evt->p_data[0])
            {
                error_code = app_log_store_dump(&s_dump_cbs);
                if (error_code)
                {
                    arr[0] = 0x81;
                    arr[1] = error_code;
                    error_code = lms_notify_cmd(0, arr, 6);
                    APP_ERROR_CHECK(error_code);
                }
            }
            else if(2 == p_evt->p_data[0])
            {
                error_code = app_log_store_clear();
                APP_ERROR_CHECK(error_code);
                arr[0] = 0x82;
                arr[1] = 0;
                error_code = lms_notify_cmd(0, arr, 2);
                APP_ERROR_CHECK(error_code);
            }
            break;

        case LMS_EVT_CMD_NOTIFY_COMPLETE:
            break;

        case LMS_EVT_DATA_RECEIVE_DATA:
            break;

        case LMS_EVT_DATA_NOTIFY_COMPLETE:
            lms_data_continue_send();
            break;


        default:
            break;
    }
}

void lms_update_mtu_size(uint16_t new_mtu)
{
    s_once_size = new_mtu - 3;
}

void app_log_dump_service_init(void)
{
    sdk_err_t error_code;
    lms_init_t lms_init;
    
    lms_init.evt_handler = lms_service_process_event;
    
    error_code = lms_service_init(&lms_init);
    APP_ERROR_CHECK(error_code);
}

#endif


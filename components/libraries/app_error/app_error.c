/**
 *****************************************************************************************
 *
 * @file app_error.c
 *
 * @brief App Error Implementation.
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

#define APP_LOG_TAG "app_error.c"

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_error.h"
#include "app_error_cfg.h"
#include "app_log.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/*
 * DEFINITIONS
 *****************************************************************************************
 */
#define APP_ERROR_INFO_LEN          512
#define APP_ERROR_CODE_NB           46

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief SDK error code information. */
typedef struct 
{
    sdk_err_t  error_code;
    char      *error_info;
} error_code_info_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static char s_error_print_info[APP_ERROR_INFO_LEN] = { 0 };

static const error_code_info_t s_error_code_info[APP_ERROR_CODE_NB] =
{
    {SDK_SUCCESS,                              "Successful."},
    {SDK_ERR_INVALID_PARAM,                    "Invalid parameter supplied."},
    {SDK_ERR_POINTER_NULL,                     "Invalid pointer supplied."},
    {SDK_ERR_INVALID_CONN_IDX,                 "Invalid connection index supplied."},
    {SDK_ERR_INVALID_HANDLE,                   "Invalid handle supplied."},
    {SDK_ERR_PROFILE_COUNT,                    "The value exceeds the maximum SDK profile count."},
    {SDK_ERR_BUSY,                             "SDK is busy internally."},
    {SDK_ERR_TIMER_INSUFFICIENT,               "Timer is insufficient."},
    {SDK_ERR_NVDS_NOT_INIT,                    "NVDS is not initialized."},
    {SDK_ERR_LIST_ITEM_NOT_FOUND,              "Item is not found in the list."},
    {SDK_ERR_LIST_ITEM_ALREADY_EXISTED,        "Item already exists in the list."},
    {SDK_ERR_LIST_FULL,                        "List is full."},
    {SDK_ERR_SDK_INTERNAL,                     "An internal error of SDK occurs."},
    {SDK_ERR_INVALID_BUFF_LENGTH,              "The buffer length is not enough."},
    {SDK_ERR_INVALID_DATA_LENGTH,              "Invalid data length supplied."},
    {SDK_ERR_DISALLOWED,                       "Operation is disallowed."},
    {SDK_ERR_NO_RESOURCES,                     "No enough BLE resources for operation."},
    {SDK_ERR_REQ_NOT_SUPPORTED,                "Request not supported."},
    {SDK_ERR_INVALID_OFFSET,                   "Offset exceeds current attribute value length."},
    {SDK_ERR_INVALID_ATT_VAL_LEN,              "Invalid length of the attribute value."},
    {SDK_ERR_INVALID_PERM,                     "Invalid permission set in service/attribute."},
    {SDK_ERR_INVALID_ADV_IDX,                  "Invalid advertising index supplied."},
    {SDK_ERR_INVALID_ADV_DATA_TYPE,            "Invalid advertising data type supplied."},
    {SDK_ERR_INVALID_PSM_NUM,                  "Invalid PSM number."},
    {SDK_ERR_INVALID_PSM_ALREADY_REGISTERED,   "The PSM number has been registered."},
    {SDK_ERR_INVALID_PSM_EXCEEDED_MAX_PSM_NUM, "The value is beyond the PSM number range."},
    {SDK_ERR_NTF_DISABLED,                     "Notification not enabled."},
    {SDK_ERR_IND_DISABLED,                     "Indication not enabled."},
    {SDK_ERR_DISCONNECTED,                     "Disconnection occurs."},
    {SDK_ERR_APP_ERROR,                        "Application error."},
    {SDK_ERR_INVALID_ADDRESS,                  "Invalid address supplied."},
    {SDK_ERR_INVALID_ADV_INTERVAL,             "Invalid advertising interval supplied."},
    {SDK_ERR_INVALID_DISVCOVERY_MODE,          "Invalid discovery mode supplied."},
    {SDK_ERR_INVALID_ADV_PARAM,                "Invalid advertising parameters supplied."},
    {SDK_ERR_INVALID_ADV_PEER_ADDR,            "Invalid peer address supplied."},
    {SDK_ERR_ADV_DATA_NOT_SET,                 "Legacy advertising data not set."},
    {SDK_ERR_PER_ADV_DATA_NOT_SET,             "Periodic advertising data not set."},
    {SDK_ERR_EXT_SCAN_RSP_DATA_NOT_SET,        "Extended scan response data not set."},
    {SDK_ERR_INVALID_DURATION_PARAM,           "Invalid duration parameter supplied."},
    {SDK_ERR_INVALID_PER_SYNC_IDX,             "Invalid periodic synchronization index supplied."},
    {SDK_ERR_INVALID_CID,                      "Invalid CID supplied."},
    {SDK_ERR_INVALID_CHL_NUM,                  "Invalid channel number supplied."},
    {SDK_ERR_NOT_ENOUGH_CREDITS,               "Not enough credits."},
    {SDK_ERR_REPEAT_CID,                       "Invalid repeat CID."},
    {SDK_ERR_CACHE_NOT_ENABLE,                 "Cache feature is not enabled."},
    {SDK_ERR_CACHE_INVALID,                    "Cache data is invalid."},
};


/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
__WEAK void app_error_fault_handler(app_error_info_t *p_error_info)
{
#if APP_ERROR_INFO_PRINT_ENABLE
    memset(s_error_print_info, 0, APP_ERROR_INFO_LEN);

    if (APP_ERROR_API_RET == p_error_info->error_type)
    {
        for (uint8_t i = 0; ; i++)
        {
            if (p_error_info->value.error_code == s_error_code_info[i].error_code)
            {
                sprintf(s_error_print_info,
                        "Error code 0x%04X: %s",
                        p_error_info->value.error_code,
                        s_error_code_info[i].error_info);
                break;
            }
            else if (APP_ERROR_CODE_NB == i)
            {
                sprintf(s_error_print_info, "Error code 0x%04X: No found information.", p_error_info->value.error_code);
                break;
            }
        }
    }
    else if (APP_ERROR_BOOL_COMPARE == p_error_info->error_type)
    {
        sprintf(s_error_print_info,
                "(%s) is not established.",
                p_error_info->value.expr);
    }

    app_log_output(APP_LOG_LVL_ERROR,
                   APP_LOG_TAG,
                   p_error_info->file,
                   p_error_info->func,
                   p_error_info->line,
                   "%s",
                   s_error_print_info);

    app_log_flush();
#endif
}

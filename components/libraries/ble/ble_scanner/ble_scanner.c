/**
 *****************************************************************************************
 *
 * @file ble_scanner.c
 *
 * @brief BLE Scanner Module implementation.
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
******************************************************************************************
*/
#include "ble_scanner.h"

#if BLE_SCANNER_ENABLE

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief BLE Scanner Module environment variable. */
struct ble_scanner_env_t
{
    bool                      connect_auto;
    bool                      is_matched;
    ble_gap_scan_param_t      scan_param;
    ble_gap_init_param_t      conn_param;
    ble_scanner_evt_handler_t evt_handler;
    ble_scanner_err_handler_t err_handler;
    bool                      filter_enable;
    uint8_t                   filter_type;
    ble_scanner_filter_mode_t filter_mode;
    ble_scanner_filter_data_t target_data;
};

/*
 * LOCAL VARIABLE DECLARATION
 *****************************************************************************************
 */
static struct ble_scanner_env_t s_scanner_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
//static void on_scanner_stoped(void *arg)
static void on_scanner_stoped(const ble_gap_stopped_reason_t reason)
{
    ble_scanner_evt_t   event;
    uint8_t             error_code;

    event.evt_type = BLE_SCANNER_EVT_INVALID;

    if (BLE_GAP_STOPPED_REASON_TIMEOUT == reason)
    {
        event.evt_type = s_scanner_env.filter_type ? BLE_SCANNER_EVT_FILTER_NO_MATCH : BLE_SCANNER_EVT_TIMEOUT;
    }

    if (BLE_GAP_STOPPED_REASON_ON_USER == reason && s_scanner_env.is_matched)
    {
        s_scanner_env.is_matched = false;
        error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &s_scanner_env.conn_param);
        if (error_code)
        {
            ble_scanner_err_on_ble_capture(error_code);
        }
    }

    if (event.evt_type && s_scanner_env.evt_handler)
    {
        s_scanner_env.evt_handler(&event);
    }
}

static void on_scanner_connected(uint8_t conn_idx)
{
    ble_scanner_evt_t event;

    event.evt_type       = BLE_SCANNER_EVT_CONNECTED;
    event.param.conn_idx = conn_idx;

    if (s_scanner_env.evt_handler)
    {
        s_scanner_env.evt_handler(&event);
    }
}

static void ble_scanner_uuid_encode(const uint8_t *p_uuid_data, uint16_t length, uint8_t uuid_type, ble_scanner_uuid_t *p_uuid_buff)
{
    if (NULL == p_uuid_data || NULL == p_uuid_buff)
    {
        return;
    }

    uint8_t current_offset = 0;

    while (current_offset < length)
    {
        switch (uuid_type)
        {
            case UUID_16_BIT_BYTES:
                if (p_uuid_buff->uuid_16_bit_count < UUID_16_BIT_NUM_MAX)
                {
                    memcpy(&p_uuid_buff->uuid_16_bit[p_uuid_buff->uuid_16_bit_count++], &p_uuid_data[current_offset], UUID_16_BIT_BYTES);
                    current_offset += UUID_16_BIT_BYTES;
                }
                break;

            case UUID_32_BIT_BYTES:
                if (p_uuid_buff->uuid_32_bit_count < UUID_32_BIT_NUM_MAX)
                {
                    memcpy(&p_uuid_buff->uuid_16_bit[p_uuid_buff->uuid_32_bit_count++], &p_uuid_data[current_offset], UUID_32_BIT_BYTES);
                    current_offset += UUID_32_BIT_BYTES;
                }
                break;

            case UUID_128_BIT_BYTES:
                if (p_uuid_buff->uuid_128_bit_count < UUID_128_BIT_NUM_MAX)
                {
                    memcpy(&p_uuid_buff->uuid_128_bit[p_uuid_buff->uuid_128_bit_count++][0], &p_uuid_data[current_offset], UUID_128_BIT_BYTES);
                    current_offset += UUID_128_BIT_BYTES;
                }
                break;

            default:
                return;
        }
    }
}

static void ble_scanner_parse_record(uint8_t ad_type, const uint8_t *p_data, uint16_t length, ble_scanner_parse_rec_t *p_parse_rec)
{
    uint8_t rec_idx = p_parse_rec->type_count;

    p_parse_rec->single_rec[rec_idx].ad_type = ad_type;
    p_parse_rec->single_rec[rec_idx].type_data.p_data = p_data;
    p_parse_rec->single_rec[rec_idx].type_data.length = length;
    p_parse_rec->type_count++;
}

static void ble_scanner_data_parse(const ble_gap_evt_adv_report_t *p_adv_report, ble_scanner_parse_report_t *p_parse_report)
{
    if (NULL == p_adv_report || NULL == p_parse_report)
    {
        return;
    }

    memset(p_parse_report, 0, sizeof(ble_scanner_parse_report_t));

    p_parse_report->adv_report_type = (ble_gap_adv_report_type_t)p_adv_report->adv_type;
    p_parse_report->rssi            = p_adv_report->rssi;
    memcpy(&p_parse_report->peer_addr, &p_adv_report->broadcaster_addr, sizeof(ble_gap_bdaddr_t));
    memcpy(&s_scanner_env.conn_param.peer_addr, &p_adv_report->broadcaster_addr, sizeof(ble_gap_bdaddr_t));

    uint8_t  parse_offset         = 0;
    uint8_t *adv_data             = p_adv_report->data;
    uint8_t  adv_data_len         = p_adv_report->length;
    uint8_t  fragment_ad_type     = 0;
    uint8_t  fragment_length      = 0;
    uint8_t  data_length          = 0;

    while (parse_offset < adv_data_len)
    {
        fragment_length = adv_data[parse_offset++];

        if (0 == fragment_length)
        {
            break;
        }

        data_length      = fragment_length - 1;
        fragment_ad_type = adv_data[parse_offset++];

        switch (fragment_ad_type)
        {
            case BLE_GAP_AD_TYPE_FLAGS:
                p_parse_report->flag = adv_data[parse_offset];
                p_parse_report->adv_type_parsed.flag = true;
                break;

            case BLE_GAP_AD_TYPE_APPEARANCE:
                p_parse_report->appearance = adv_data[parse_offset] | adv_data[parse_offset + 1] << 8;
                p_parse_report->adv_type_parsed.appearance = true;
                break;

            case BLE_GAP_AD_TYPE_SHORTENED_NAME:
            case BLE_GAP_AD_TYPE_COMPLETE_NAME:
                p_parse_report->local_name.p_data = &adv_data[parse_offset];
                p_parse_report->local_name.length = data_length;
                p_parse_report->adv_type_parsed.local_name = true;
                break;

            case BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA:
                p_parse_report->manufacture_data.p_data = &adv_data[parse_offset];
                p_parse_report->manufacture_data.length = data_length;
                p_parse_report->adv_type_parsed.manufacture_data = true;
                break;

            case BLE_GAP_AD_TYPE_MORE_16_BIT_UUID:
            case BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID:
                ble_scanner_uuid_encode(&adv_data[parse_offset], data_length, UUID_16_BIT_BYTES, &p_parse_report->uuid_list);
                p_parse_report->adv_type_parsed.uuid = true;
                break;

            case BLE_GAP_AD_TYPE_MORE_32_BIT_UUID:
            case BLE_GAP_AD_TYPE_COMPLETE_LIST_32_BIT_UUID:
                ble_scanner_uuid_encode(&adv_data[parse_offset], data_length, UUID_32_BIT_BYTES, &p_parse_report->uuid_list);
                p_parse_report->adv_type_parsed.uuid = true;
                break;

            case BLE_GAP_AD_TYPE_MORE_128_BIT_UUID:
            case BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID:
                ble_scanner_uuid_encode(&adv_data[parse_offset], data_length, UUID_128_BIT_BYTES, &p_parse_report->uuid_list);
                p_parse_report->adv_type_parsed.uuid = true;
                break;

            default:
                ble_scanner_parse_record(fragment_ad_type, &adv_data[parse_offset], data_length, &p_parse_report->other_parse_rec);
                break;
        }

        parse_offset += data_length;
    }
}

static bool ble_scanner_filter_uuid_macth(ble_data_t *p_uuid, ble_scanner_uuid_t *p_uuid_list)
{
    if (p_uuid->length == UUID_16_BIT_BYTES)
    {
        for (uint8_t i = 0; i < p_uuid_list->uuid_16_bit_count; i++)
        {
            if (0 == memcmp(p_uuid->p_data, (uint8_t *)&p_uuid_list->uuid_16_bit[i], UUID_16_BIT_BYTES))
            {
                return true;
            }
        }
    }

    if (p_uuid->length == UUID_32_BIT_BYTES)
    {
        for (uint8_t i = 0; i < p_uuid_list->uuid_32_bit_count; i++)
        {
            if (0 == memcmp(p_uuid->p_data, (uint8_t *)&p_uuid_list->uuid_32_bit[i], UUID_32_BIT_BYTES))
            {
                return true;
            }
        }
    }

    if (p_uuid->length == UUID_128_BIT_BYTES)
    {
        for (uint8_t i = 0; i < p_uuid_list->uuid_128_bit_count; i++)
        {
            if (0 == memcmp(p_uuid->p_data, p_uuid_list->uuid_128_bit[i], UUID_128_BIT_BYTES))
            {
                return true;
            }
        }
    }

    return false;
}

static bool ble_scanner_filter_match_check(ble_scanner_parse_report_t *p_parse_report, ble_scanner_filter_match_t *p_match_result)
{
    memset(p_match_result, 0, sizeof(ble_scanner_filter_match_t));

    if (s_scanner_env.filter_type & BLE_SCANNER_NAME_FILTER)
    {
        if (p_parse_report->local_name.length == s_scanner_env.target_data.dev_name.length &&
            0 == memcmp(p_parse_report->local_name.p_data,
                        s_scanner_env.target_data.dev_name.p_data,
                        s_scanner_env.target_data.dev_name.length))
        {
            p_match_result->dev_name_match = true;
        }
    }

    if (s_scanner_env.filter_type & BLE_SCANNER_APPEARANCE_FILTER)
    {
        if (p_parse_report->appearance ==  s_scanner_env.target_data.appearance)
        {
            p_match_result->appearance_match = true;
        }
    }

    if (s_scanner_env.filter_type & BLE_SCANNER_UUID_FILTER)
    {
        if (ble_scanner_filter_uuid_macth(&s_scanner_env.target_data.svr_uuid, &p_parse_report->uuid_list))
        {
            p_match_result->uuid_match = true;
        }
    }

    if (s_scanner_env.filter_type & BLE_SCANNER_ADDR_FILTER)
    {
        if (0 == memcmp(&s_scanner_env.target_data.target_addr, &p_parse_report->peer_addr, sizeof(ble_gap_bdaddr_t)))
        {
            p_match_result->addr_match = true;
        }
    }

    if (BLE_SCANNER_FILTER_ANYONE_MATCH == s_scanner_env.filter_mode)
    {
        if (p_match_result->dev_name_match ||
            p_match_result->appearance_match ||
            p_match_result->uuid_match ||
            p_match_result->addr_match)
        {
            return true;
        }

        return false;
    }
    else
    {
        if (s_scanner_env.filter_type & BLE_SCANNER_NAME_FILTER && !p_match_result->dev_name_match)
        {
            return false;
        }

        if (s_scanner_env.filter_type & BLE_SCANNER_APPEARANCE_FILTER && !p_match_result->appearance_match)
        {
            return false;
        }

        if (s_scanner_env.filter_type & BLE_SCANNER_UUID_FILTER && !p_match_result->uuid_match)
        {
            return false;
        } 

        if (s_scanner_env.filter_type & BLE_SCANNER_ADDR_FILTER && !p_match_result->addr_match)
        {
            return false;
        }
    }

    return true;
}

static void on_scanner_adv_report(const ble_gap_evt_adv_report_t *p_adv_report)
{
    ble_scanner_parse_report_t   parse_report;
    ble_scanner_evt_t            event;
    ble_scanner_filter_match_t   match_result;

    memset(&event, 0, sizeof(event));
    memset(&parse_report, 0, sizeof(parse_report));
    memset(&match_result, 0, sizeof(match_result));

    if(!s_scanner_env.is_matched)
    {
        ble_scanner_data_parse(p_adv_report, &parse_report);
    }

    event.evt_type = BLE_SCANNER_EVT_ADV_REPORT_PARSE;
    memcpy(&event.param.parse_record, &parse_report, sizeof(ble_scanner_parse_report_t));

    if (s_scanner_env.evt_handler)
    {
        s_scanner_env.evt_handler(&event);
    }

    if (s_scanner_env.filter_enable)
    {
        if (ble_scanner_filter_match_check(&parse_report, &match_result))
        {
            event.evt_type = BLE_SCANNER_EVT_FILTER_MATCH;
            memcpy(&event.param.match_result, &match_result, sizeof(ble_scanner_filter_match_t));
     
            if (s_scanner_env.evt_handler)
            {
                s_scanner_env.evt_handler(&event);
            }

            if (s_scanner_env.connect_auto)
            {
                s_scanner_env.is_matched = true;
                ble_scanner_stop();
            }
        }
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t ble_scanner_init(ble_scanner_init_t *p_scan_init)
{
    if (NULL == p_scan_init)
    {
        return SDK_ERR_INVALID_PARAM;
    }

    memcpy(&s_scanner_env, p_scan_init, sizeof(ble_scanner_init_t));

    return ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &s_scanner_env.scan_param);
}

sdk_err_t ble_scanner_filter_set(uint8_t filter_type, ble_scanner_filter_data_t *p_filter_data)
{
    sdk_err_t   error_code = SDK_SUCCESS;

    if (NULL == p_filter_data)
    {
        return SDK_ERR_INVALID_PARAM;
    }

    s_scanner_env.filter_type = filter_type;
    memcpy(&s_scanner_env.target_data, p_filter_data, sizeof(ble_scanner_filter_data_t));

    return error_code;
}

void ble_scanner_filter_disable(void)
{
    s_scanner_env.filter_enable = false;
}

void ble_scanner_filter_enable(ble_scanner_filter_mode_t filter_mode)
{
    s_scanner_env.filter_mode = filter_mode;

    s_scanner_env.filter_enable = true;
}

sdk_err_t ble_scanner_start(void)
{
    ble_scanner_evt_t event;

    if (s_scanner_env.scan_param.use_whitelist)
    {
        event.evt_type = BLE_SCANNER_EVT_WHITELIST_REQUEST;

        if (s_scanner_env.evt_handler)
        {
            s_scanner_env.evt_handler(&event);
        }
    }

    return ble_gap_scan_start();
}

sdk_err_t ble_scanner_stop(void)
{
    return ble_gap_scan_stop();
}


void ble_scanner_err_on_ble_capture(uint8_t err_code)
{
    if (s_scanner_env.err_handler && err_code)
    {
        s_scanner_env.err_handler(err_code);
    }
}

void ble_scanner_evt_on_ble_capture(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }
    
    switch (p_evt->evt_id)
    {
        case BLE_GAPM_EVT_SCAN_START:
            break;

        case BLE_GAPM_EVT_SCAN_STOP:
            on_scanner_stoped(p_evt->evt.gapm_evt.params.scan_stop.reason);
            break;

        case BLE_GAPM_EVT_ADV_REPORT:
            on_scanner_adv_report(&p_evt->evt.gapm_evt.params.adv_report);
            break;

        case BLE_GAPC_EVT_CONNECTED:
            on_scanner_connected(p_evt->evt.gapc_evt.index);
            break;

        default:
            break;
    }
}
#endif



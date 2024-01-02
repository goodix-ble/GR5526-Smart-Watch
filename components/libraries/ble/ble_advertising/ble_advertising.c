/**
 ****************************************************************************************
 *
 * @file ble_advertising.c
 *
 * @brief BLE Advertising Module API
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
 *****************************************************************************************
 */
#include "ble_module_config.h"
#if BLE_ADVERTISING_ENABLE
#include "ble_advertising.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define BLE_ADV_HIGH_DUTY_DIR_DURATION        128
#define BLE_ADV_INTERVAL_MIN                  32
#define BLE_ADV_DURATION_MAX                  18000

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief BLE Advertising Module environment variable. */
struct ble_adv_env_t
{
    bool                        initialized;
    bool                        adv_act_exist;
    ble_adv_mode_t              cur_adv_mode;
    ble_adv_mode_cfg_t          adv_mode_cfg;
    ble_adv_evt_handler_t       evt_handler;
    ble_adv_err_handler_t       err_handler;
    ble_gap_ext_adv_param_t     adv_param;
    ble_gap_adv_time_param_t    adv_time_param;
    bool                        peer_addr_exist;
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct ble_adv_env_t adv_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static ble_adv_mode_t adv_mode_next_get(ble_adv_mode_t adv_mode)
{
    return (ble_adv_mode_t)((adv_mode + 1) % (BLE_ADV_MODE_SLOW + 2));
}

static ble_adv_mode_t adv_mode_next_avail_get(ble_adv_mode_cfg_t *p_adv_mode_cfg, ble_adv_mode_t adv_mode)
{
    switch (adv_mode)
    {
        case BLE_ADV_MODE_DIRECTED_HIGH_DUTY:
            if ((p_adv_mode_cfg->adv_directed_high_duty_enabled) && (!p_adv_mode_cfg->adv_extended_enabled) &&
                 adv_env.peer_addr_exist)
            {
                return BLE_ADV_MODE_DIRECTED_HIGH_DUTY;
            }

        case BLE_ADV_MODE_DIRECTED_LOW_DUTY:
            if ((p_adv_mode_cfg->adv_directed_low_duty_enabled && adv_env.peer_addr_exist))
            {
                return BLE_ADV_MODE_DIRECTED_LOW_DUTY;
            }

        case BLE_ADV_MODE_FAST:
            if (p_adv_mode_cfg->adv_fast_enabled)
            {
                return BLE_ADV_MODE_FAST;
            }

        case BLE_ADV_MODE_SLOW:
            if (p_adv_mode_cfg->adv_slow_enabled)
            {
                return BLE_ADV_MODE_SLOW;
            }

        default:
            return BLE_ADV_MODE_IDLE;
    }
}

static sdk_err_t directed_high_duty_adv_param_set(struct ble_adv_env_t *p_adv_env)
{
    p_adv_env->adv_param.type = BLE_GAP_ADV_TYPE_LEGACY;
    p_adv_env->adv_param.prop = BLE_GAP_ADV_PROP_CONNECTABLE_BIT |
                                BLE_GAP_ADV_PROP_DIRECTED_BIT |
                                BLE_GAP_ADV_PROP_HDC_BIT;
    p_adv_env->adv_param.disc_mode = BLE_GAP_DISC_MODE_NON_DISCOVERABLE;

    p_adv_env->adv_time_param.max_adv_evt = 0;
    
    p_adv_env->adv_time_param.duration    = BLE_ADV_HIGH_DUTY_DIR_DURATION;

    return ble_gap_ext_adv_param_set(0, BLE_GAP_OWN_ADDR_GEN_RSLV, &p_adv_env->adv_param);
}

static sdk_err_t directed_low_duty_adv_param_set(struct ble_adv_env_t *p_adv_env)
{
    if (p_adv_env->adv_mode_cfg.adv_extended_enabled)
    {
        p_adv_env->adv_param.type = BLE_GAP_ADV_TYPE_EXTENDED;
        p_adv_env->adv_param.prop = BLE_GAP_ADV_PROP_DIRECTED_BIT |
                                    BLE_GAP_ADV_PROP_CONNECTABLE_BIT;
    }
    else
    {
        p_adv_env->adv_param.type = BLE_GAP_ADV_TYPE_LEGACY;
        p_adv_env->adv_param.prop = BLE_GAP_ADV_PROP_DIRECTED_BIT |
                                    BLE_GAP_ADV_PROP_CONNECTABLE_BIT;
    }
    p_adv_env->adv_param.prim_cfg.adv_intv_min = p_adv_env->adv_mode_cfg.adv_directed_low_duty_interval;
    p_adv_env->adv_param.prim_cfg.adv_intv_max = p_adv_env->adv_mode_cfg.adv_directed_low_duty_interval;
    p_adv_env->adv_param.disc_mode = BLE_GAP_DISC_MODE_NON_DISCOVERABLE;

    p_adv_env->adv_time_param.max_adv_evt = 0;
    p_adv_env->adv_time_param.duration    = p_adv_env->adv_mode_cfg.adv_directed_low_duty_timeout;
    return ble_gap_ext_adv_param_set(0, BLE_GAP_OWN_ADDR_GEN_RSLV, &p_adv_env->adv_param);
}

static sdk_err_t fast_adv_param_set(struct ble_adv_env_t *p_adv_env)
{
    if (p_adv_env->adv_mode_cfg.adv_extended_enabled)
    {
        p_adv_env->adv_param.type = BLE_GAP_ADV_TYPE_EXTENDED;
        p_adv_env->adv_param.prop = BLE_GAP_ADV_PROP_CONNECTABLE_BIT;
    }
    else
    {
        p_adv_env->adv_param.type = BLE_GAP_ADV_TYPE_LEGACY;
        p_adv_env->adv_param.prop = BLE_GAP_ADV_PROP_SCANNABLE_BIT |
                                     BLE_GAP_ADV_PROP_CONNECTABLE_BIT;
    }
    p_adv_env->adv_param.prim_cfg.adv_intv_min = p_adv_env->adv_mode_cfg.adv_fast_interval;
    p_adv_env->adv_param.prim_cfg.adv_intv_max = p_adv_env->adv_mode_cfg.adv_fast_interval;
    p_adv_env->adv_param.disc_mode = (p_adv_env->adv_param.filter_pol == BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY)?
                                       BLE_GAP_DISC_MODE_GEN_DISCOVERABLE : BLE_GAP_DISC_MODE_NON_DISCOVERABLE;

    p_adv_env->adv_time_param.max_adv_evt = 0;
    p_adv_env->adv_time_param.duration    = p_adv_env->adv_mode_cfg.adv_fast_timeout;

    return ble_gap_ext_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &p_adv_env->adv_param);
}

static sdk_err_t slow_adv_param_set(struct ble_adv_env_t *p_adv_env)
{
    if (p_adv_env->adv_mode_cfg.adv_extended_enabled)
    {
        p_adv_env->adv_param.type = BLE_GAP_ADV_TYPE_EXTENDED;
        p_adv_env->adv_param.prop = BLE_GAP_ADV_PROP_CONNECTABLE_BIT;
    }
    else
    {
        p_adv_env->adv_param.type = BLE_GAP_ADV_TYPE_LEGACY;
        p_adv_env->adv_param.prop = BLE_GAP_ADV_PROP_SCANNABLE_BIT |
                                     BLE_GAP_ADV_PROP_CONNECTABLE_BIT;
    }
    p_adv_env->adv_param.prim_cfg.adv_intv_min = p_adv_env->adv_mode_cfg.adv_slow_interval;
    p_adv_env->adv_param.prim_cfg.adv_intv_max = p_adv_env->adv_mode_cfg.adv_slow_interval;
    p_adv_env->adv_param.disc_mode = (p_adv_env->adv_param.filter_pol == BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY)?
                                       BLE_GAP_DISC_MODE_GEN_DISCOVERABLE : BLE_GAP_DISC_MODE_NON_DISCOVERABLE;

    p_adv_env->adv_time_param.max_adv_evt = 0;
    p_adv_env->adv_time_param.duration    = p_adv_env->adv_mode_cfg.adv_slow_timeout;

    return ble_gap_ext_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &p_adv_env->adv_param);
}

static void ble_adv_started(void)
{
    ble_adv_evt_type_t evt = BLE_ADV_EVT_INVALID;

    switch(adv_env.cur_adv_mode)
    {
        case BLE_ADV_MODE_DIRECTED_HIGH_DUTY:
            evt = BLE_ADV_EVT_DIRECTED_HIGH_DUTY;
            break;

        case BLE_ADV_MODE_DIRECTED_LOW_DUTY:
            evt = BLE_ADV_EVT_DIRECTED_LOW_DUTY;
            break;

        case BLE_ADV_MODE_FAST:
            evt = BLE_ADV_EVT_FAST;
            break;

        case BLE_ADV_MODE_SLOW:
            evt = BLE_ADV_EVT_SLOW;
            break;

        default:
            break;
    }

    adv_env.adv_act_exist = true;

    if (adv_env.evt_handler && evt)
    {
        adv_env.evt_handler(evt);
    }
}

static void ble_advertising_err_on_ble_capture(uint8_t err_code)
{
    if (adv_env.err_handler && err_code)
    {
        adv_env.err_handler(err_code);
    }
}

static void ble_adv_stopped(uint8_t stop_reason)
{
    sdk_err_t err_code;

    adv_env.adv_act_exist = false;

    if (BLE_GAP_STOPPED_REASON_TIMEOUT == stop_reason)
    {
        err_code = ble_advertising_start(adv_mode_next_get(adv_env.cur_adv_mode));
        ble_advertising_err_on_ble_capture(err_code);
    }
    else if (BLE_GAP_STOPPED_REASON_CONN_EST == stop_reason)
    {
        if (adv_env.adv_mode_cfg.multi_link_enabled && !adv_env.adv_act_exist)
        {
            ble_advertising_start(BLE_ADV_MODE_DIRECTED_HIGH_DUTY);
        }
    }
}

static void ble_adv_disconnected(void)
{
    if (adv_env.adv_mode_cfg.adv_on_disconnect_enabled && !adv_env.adv_act_exist)
    {
        ble_advertising_start(BLE_ADV_MODE_DIRECTED_HIGH_DUTY);
    }
}

static sdk_err_t adv_init_param_check(ble_adv_init_t *p_adv_init)
{
    if (NULL == p_adv_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    ble_adv_mode_cfg_t *p_cfg = &p_adv_init->adv_mode_cfg;

    if (p_cfg->adv_directed_low_duty_enabled &&
        (p_cfg->adv_directed_low_duty_interval < BLE_ADV_INTERVAL_MIN || p_cfg->adv_directed_low_duty_timeout > BLE_ADV_DURATION_MAX))
    {
        return SDK_ERR_INVALID_PARAM;
    }

    if (p_cfg->adv_fast_enabled &&
        (p_cfg->adv_fast_interval < BLE_ADV_INTERVAL_MIN || p_cfg->adv_fast_interval > BLE_ADV_DURATION_MAX))
    {
        return SDK_ERR_INVALID_PARAM;
    }
    if (p_cfg->adv_slow_enabled &&
        (p_cfg->adv_slow_interval < BLE_ADV_INTERVAL_MIN || p_cfg->adv_slow_interval > BLE_ADV_DURATION_MAX))
    {
        return SDK_ERR_INVALID_PARAM;
    }

    return SDK_SUCCESS;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
sdk_err_t ble_advertising_init(ble_adv_init_t *p_adv_init)
{
    sdk_err_t error_code;

    if (adv_env.initialized)
    {
        return SDK_ERR_DISALLOWED;
    }

    error_code = adv_init_param_check(p_adv_init);
    RET_VERIFY_SUCCESS(error_code);

    adv_env.cur_adv_mode = BLE_ADV_MODE_IDLE;
    adv_env.evt_handler  = p_adv_init->evt_handler;
    adv_env.err_handler  = p_adv_init->err_handler;
 
    memcpy(&adv_env.adv_mode_cfg, &p_adv_init->adv_mode_cfg, sizeof(ble_adv_mode_cfg_t));

    error_code = ble_gap_adv_data_set(0,
                                      BLE_GAP_ADV_DATA_TYPE_DATA,
                                      p_adv_init->adv_data.p_data, 
                                      p_adv_init->adv_data.length);

    if (p_adv_init->srp_data.p_data && p_adv_init->srp_data.length && !p_adv_init->adv_mode_cfg.adv_extended_enabled)
    {
        error_code = ble_gap_adv_data_set(0,
                                          BLE_GAP_ADV_DATA_TYPE_SCAN_RSP,
                                          p_adv_init->srp_data.p_data,
                                          p_adv_init->srp_data.length);
        RET_VERIFY_SUCCESS(error_code);
    }

    RET_VERIFY_SUCCESS(error_code);

    adv_env.initialized = true;

    return error_code;
}

sdk_err_t ble_advertising_start(ble_adv_mode_t adv_mode)
{
    sdk_err_t error_code;

    if (!adv_env.initialized)
    {
        return SDK_ERR_DISALLOWED;
    }

    adv_env.cur_adv_mode = adv_mode;
    
    //Set general parameters
    adv_env.adv_param.disc_mode         = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    adv_env.adv_param.max_tx_pwr        = 0;
    adv_env.adv_param.prim_cfg.chnl_map = BLE_GAP_ADV_CHANNEL_37_38_39;
    adv_env.adv_param.filter_pol        = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
//    adv_env.adv_param.filter_pol = (adv_env.adv_mode_cfg.ble_adv_whitelist_enabled)?
//                                        BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_WLST : BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
    adv_env.adv_param.prim_cfg.phy      = BLE_GAP_PHY_1MBPS_VALUE;
    
    if (adv_env.adv_mode_cfg.adv_extended_enabled)
    {
        if (adv_env.adv_mode_cfg.adv_primary_phy)
        {
            adv_env.adv_param.prim_cfg.phy = adv_env.adv_mode_cfg.adv_primary_phy;
        }
        
        if (adv_env.adv_mode_cfg.adv_secondary_phy)
        {
            adv_env.adv_param.second_cfg.phy = adv_env.adv_mode_cfg.adv_secondary_phy;
        }
        else
        {
            adv_env.adv_param.second_cfg.phy = (ble_gap_le_phy_value_t)BLE_GAP_PHY_LE_1MBPS;
        }
    }

    if (((adv_env.adv_mode_cfg.adv_directed_high_duty_enabled) && (adv_env.cur_adv_mode == BLE_ADV_MODE_DIRECTED_HIGH_DUTY)) || \
        ((adv_env.adv_mode_cfg.adv_directed_low_duty_enabled)  && (adv_env.cur_adv_mode == BLE_ADV_MODE_DIRECTED_HIGH_DUTY)) || \
        ((adv_env.adv_mode_cfg.adv_directed_low_duty_enabled)  && (adv_env.cur_adv_mode == BLE_ADV_MODE_DIRECTED_LOW_DUTY)))
    {
         if (adv_env.evt_handler)
         {
             adv_env.evt_handler(BLE_ADV_EVT_DIR_ADDR_REQUEST);
         }
    }
    else
    {
        memset(&adv_env.adv_param.peer_addr, 0, sizeof(ble_gap_bdaddr_t));
    }

    adv_env.cur_adv_mode = adv_mode_next_avail_get(&adv_env.adv_mode_cfg, adv_mode);

    if (adv_env.adv_mode_cfg.adv_extended_enabled)
    {
        if (adv_env.adv_mode_cfg.adv_primary_phy)
        {
            adv_env.adv_param.prim_cfg.phy = adv_env.adv_mode_cfg.adv_primary_phy;
        }

        if (adv_env.adv_mode_cfg.adv_secondary_phy)
        {
            adv_env.adv_param.second_cfg.phy = adv_env.adv_mode_cfg.adv_secondary_phy;
        }
    }

    switch (adv_env.cur_adv_mode)
    {
        case BLE_ADV_MODE_DIRECTED_HIGH_DUTY:
            error_code = directed_high_duty_adv_param_set(&adv_env);
            RET_VERIFY_SUCCESS(error_code);
            break;

        case BLE_ADV_MODE_DIRECTED_LOW_DUTY:
            error_code = directed_low_duty_adv_param_set(&adv_env);
            RET_VERIFY_SUCCESS(error_code);
            break;

        case BLE_ADV_MODE_FAST:
            error_code = fast_adv_param_set(&adv_env);
            RET_VERIFY_SUCCESS(error_code);
            break;

        case BLE_ADV_MODE_SLOW:
            error_code = slow_adv_param_set(&adv_env);
            RET_VERIFY_SUCCESS(error_code);
            break;

        default:
            break;
    }

    return ble_gap_adv_start(0, &adv_env.adv_time_param);
}

sdk_err_t ble_advertising_adv_data_update(const uint8_t *p_adv_data, uint16_t adv_data_len, const uint8_t *p_srsp_data, uint16_t srsp_data_len)
{
    sdk_err_t  error_code;

    if ((p_adv_data == NULL) && (p_srsp_data == NULL) && (adv_data_len <= 0) && (srsp_data_len <= 0))
    {
        return SDK_ERR_POINTER_NULL;
    }
    
    if (adv_env.initialized == false)
    {
        return SDK_ERR_DISALLOWED;
    }
    else
    {
        if (p_adv_data && adv_data_len)
        {
            error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, p_adv_data, adv_data_len);
            RET_VERIFY_SUCCESS(error_code); 
        }

        if (p_srsp_data && srsp_data_len)
        {
            error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, p_srsp_data, srsp_data_len);
            RET_VERIFY_SUCCESS(error_code);
        } 
    }
    return error_code;
}

sdk_err_t ble_advertising_dir_addr_fill(ble_gap_bdaddr_t *p_peer_addr)
{
    for (uint8_t i = 0; i < BLE_GAP_ADDR_LEN; i++)
    {
        if (p_peer_addr->gap_addr.addr[i] != 0)
        {
            break;
        }
        return SDK_ERR_INVALID_PARAM;
    }

    memcpy(&adv_env.adv_param.peer_addr, p_peer_addr, sizeof(ble_gap_bdaddr_t));
    adv_env.peer_addr_exist = true;

    return SDK_SUCCESS;
}

void ble_advertising_evt_on_ble_capture(const ble_evt_t *p_evt)
{
    switch (p_evt->evt_id)
    {
        case BLE_GAPM_EVT_ADV_START:
            ble_adv_started();
            break;

        case BLE_GAPM_EVT_ADV_STOP:
            ble_adv_stopped(p_evt->evt.gapm_evt.params.adv_stop.reason);
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            ble_adv_disconnected();
            break;

        default:
            break;
    }
}
#endif


/**
 *****************************************************************************************
 *
 * @file ias.c
 *
 * @brief Tx Power Server Implementation.
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
#include "tps.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief Tx Power Service attributes index. */
enum tps_attr_idx_tag
{
    TPS_IDX_SVC,

    TPS_IDX_TX_POWER_LVL_CHAR,
    TPS_IDX_TX_POWER_LVL_VAL,

    TPS_IDX_NB,
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Tx Power Service environment variable. */
struct tps_env_t
{
    tps_init_t tps_init;                    /**< Tx Power Service initialization variables. */
    uint16_t   start_hdl;                   /**< Tx Power Service start handle. */
    ble_gatts_create_db_t     tps_serv_db;  /**< Tx Power Service DataBase. */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct tps_env_t s_tps_env;   /**< Tx Power Service initialization variables. */
static uint8_t s_char_mask = 0x07;   /**< Features added into ATT database.
                                      *   bit0 - Tx Power Service Declaration
                                      *   bit1 - Tx Power Level Characteristic Declaration
                                      *   bit2 - Tx Power Level Characteristic Value
                                      */
static const uint8_t s_tps_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_TX_POWER);

/**@brief TPS Database Description - Used to add attributes into the database. */
static const ble_gatts_attm_desc_t tps_attr_tab[TPS_IDX_NB] =
{
    // Tx Power Service Declaration
    [TPS_IDX_SVC]               = {BLE_ATT_DECL_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Tx Power Level Characteristic Declaration
    [TPS_IDX_TX_POWER_LVL_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC,  BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Tx Power Level Characteristic Value
    [TPS_IDX_TX_POWER_LVL_VAL]  = {BLE_ATT_CHAR_TX_POWER_LEVEL,  BLE_GATTS_READ_PERM_UNSEC, 0, sizeof(int8_t)},
};

/*
 * LOCAL FUNCTION DECLARATIONS
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void tps_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    uint16_t  handle;

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            break;

        case BLE_GATTS_EVT_NTF_IND:
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            break;

        case BLE_GATT_COMMON_EVT_PRF_REGISTER:
            handle = prf_find_handle_by_idx(TPS_IDX_TX_POWER_LVL_VAL, s_tps_env.start_hdl, &s_char_mask);
            ble_gatts_value_set(handle, sizeof(uint8_t), 0, (uint8_t *)&s_tps_env.tps_init.initial_tx_power_level);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t tps_service_init(tps_init_t *p_tps_init)
{
    if (NULL == p_tps_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    s_tps_env.tps_init.initial_tx_power_level = p_tps_init->initial_tx_power_level;

    memset(&s_tps_env.tps_serv_db, 0, sizeof(ble_gatts_create_db_t));

    s_tps_env.start_hdl                        = PRF_INVALID_HANDLE;
    s_tps_env.tps_serv_db.shdl                 = &s_tps_env.start_hdl;
    s_tps_env.tps_serv_db.uuid                 = s_tps_svc_uuid;
    s_tps_env.tps_serv_db.attr_tab_cfg         = &s_char_mask;
    s_tps_env.tps_serv_db.max_nb_attr          = TPS_IDX_NB;
    s_tps_env.tps_serv_db.srvc_perm            = 0;
    s_tps_env.tps_serv_db.attr_tab_type        = BLE_GATTS_SERVICE_TABLE_TYPE_16;
    s_tps_env.tps_serv_db.attr_tab.attr_tab_16 = tps_attr_tab;

    return ble_gatts_prf_add(&s_tps_env.tps_serv_db, tps_ble_evt_handler);
}

sdk_err_t tps_tx_power_level_set(int8_t tx_power_level)
{
    if (tx_power_level < -100 || tx_power_level > 20)
    {
        return SDK_ERR_INVALID_PARAM;
    }
    else
    {
        uint16_t handle = prf_find_handle_by_idx(TPS_IDX_TX_POWER_LVL_VAL, s_tps_env.start_hdl, &s_char_mask);

        return ble_gatts_value_set(handle, sizeof(int8_t), 0, (uint8_t *)&tx_power_level);
    }
}

uint16_t tps_service_start_handle_get(void)
{
    return s_tps_env.start_hdl;
}

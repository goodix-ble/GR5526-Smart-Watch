/**
 *****************************************************************************************
 *
 * @file lls.c
 *
 * @brief Link Loss Server Implementation.
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
#include "lls.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief LLS Attributes database index list. */
enum lls_attr_idx_t
{
    LLS_IDX_SVC,

    LLS_IDX_ALERT_LVL_CHAR,
    LLS_IDX_ALERT_LVL_VAL,

    LLS_IDX_NB,
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Link Loss Service environment variable. */
struct lls_env_t
{
    lls_init_t            lls_init;                    /**< Link Loss Service initialization variables. */
    uint16_t              start_hdl;                   /**< Link Loss Service start handle. */
    ble_gatts_create_db_t lls_serv_db;  /**< Link Loss Service DataBase. */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/** Pointer to Link Loss Service environment variable. */
static struct lls_env_t s_lls_env;            /**< Link Loss Service instance. */
static uint8_t          s_char_mask = 0x07;   /**< Features added into ATT database.
                                               * bit0 - Link Loss Service Declaration
                                               * bit1 - Alert Level Characteristic Declaration
                                               * bit2 - Alert Level Characteristic Value
                                               */
static const uint8_t s_lls_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_LINK_LOSS);

/**@brief Full LLS Database Description - Used to add attributes into the
 *        database.
 */
static const ble_gatts_attm_desc_t lls_attr_tab[LLS_IDX_NB] =
{
    // Link Loss Service Declaration
    [LLS_IDX_SVC]            = {BLE_ATT_DECL_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Alert Level Characteristic Declaration
    [LLS_IDX_ALERT_LVL_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC,  BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Alert Level Characteristic Value
    [LLS_IDX_ALERT_LVL_VAL]  = {BLE_ATT_CHAR_ALERT_LEVEL,     BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC, 0, sizeof(uint8_t)},
};

/*
 * LOCAL FUNCTION DECLARATIONS
 *******************************************************************************
 */
static void        lls_on_connect(uint8_t conn_idx);
static void        lls_on_disconnect(uint8_t conn_idx, uint8_t reason);

/**
 *****************************************************************************************
 * @brief Handle the connect event.
 *
 * @param[in] conn_idx: Connection index.
 *****************************************************************************************
 */
static void lls_on_connect(uint8_t conn_idx)
{
    lls_evt_t   evt;
    sdk_err_t   ret;

    if (s_lls_env.lls_init.evt_handler)
    {
        evt.evt_type = LLS_EVT_LINK_LOSS_ALERT;

        ret = lls_alert_level_get((unsigned char *)(&evt.alert_level));

        if (SDK_SUCCESS == ret && s_lls_env.lls_init.evt_handler)
        {
            /* Inform Application the link is (re)connected */
            s_lls_env.lls_init.evt_handler(&evt);
        }
    }
}

/**
 *****************************************************************************************
 * @brief Handle the disconnect event.
 *
 * @param[in] conn_idx:Connect index.
 * @param[in] reason:  The reason of disconnect.
 *****************************************************************************************
 */
static void lls_on_disconnect(uint8_t conn_idx, uint8_t reason)
{
    /* The reason is HCI Connection Timeout */
    if (BLE_LL_ERR_CON_TIMEOUT == reason || BLE_LL_ERR_INSTANT_PASSED == reason)
    {
        /* Link loss detected, inform application */
        lls_evt_t evt;
        sdk_err_t ret;

        evt.evt_type = LLS_EVT_LINK_LOSS_ALERT;

        ret = lls_alert_level_get((unsigned char *)(&evt.alert_level));

        if (SDK_SUCCESS == ret)
        {
            if (s_lls_env.lls_init.evt_handler)
            {
                s_lls_env.lls_init.evt_handler(&evt);
            }
        }
    }

    return;
}

/**
 *****************************************************************************************
 * @brief Handles reception of the write request.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_param:  Pointer to the parameters of the write request.
 *****************************************************************************************
 */
static void lls_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t handle = prf_find_handle_by_idx(LLS_IDX_ALERT_LVL_VAL, s_lls_env.start_hdl, &s_char_mask);
    ble_gatts_write_cfm_t cfm;

    cfm.handle = p_param->handle;

    if (handle != p_param->handle)
    {
        cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
    }
    else
    {
        if (p_param->length != sizeof(uint8_t))
        {
            cfm.status = SDK_ERR_INVALID_ATT_VAL_LEN;
        }
        else
        {
            //Send write response
            uint8_t value = p_param->value[0];

            cfm.status = (uint8_t)ble_gatts_value_set(cfm.handle, sizeof(uint8_t), 0, &value);
        }
    }

    ble_gatts_write_cfm(conn_idx, &cfm);
}

static void lls_ble_evt_handler(const ble_evt_t *p_evt)
{
    if(NULL == p_evt)
    {
        return ;
    }

    uint16_t  handle;

    switch(p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            break;
        case BLE_GATTS_EVT_WRITE_REQUEST:
            lls_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;
        case BLE_GATTS_EVT_NTF_IND:
            break;
        case BLE_GATTS_EVT_CCCD_RECOVERY:
            break;
        case BLE_GAPC_EVT_CONNECTED:
            lls_on_connect(p_evt->evt.gapc_evt.index);
            break;
        case BLE_GAPC_EVT_DISCONNECTED:
            lls_on_disconnect(p_evt->evt.gapc_evt.index,p_evt->evt.gapc_evt.params.disconnected.reason);
            break;
        case BLE_GATT_COMMON_EVT_PRF_REGISTER:
            handle = prf_find_handle_by_idx(LLS_IDX_ALERT_LVL_VAL, s_lls_env.start_hdl, &s_char_mask);
            ble_gatts_value_set(handle, sizeof(uint8_t), 0, (uint8_t *)&s_lls_env.lls_init.initial_alert_level);
            break;
        default:
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t lls_service_init(lls_init_t *p_lls_init)
{
    if (NULL == p_lls_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    s_lls_env.lls_init.evt_handler         = p_lls_init->evt_handler;
    s_lls_env.lls_init.initial_alert_level = p_lls_init->initial_alert_level;

    memset(&s_lls_env.lls_serv_db, 0, sizeof(ble_gatts_create_db_t));

    s_lls_env.start_hdl                        = PRF_INVALID_HANDLE;
    s_lls_env.lls_serv_db.shdl                 = &s_lls_env.start_hdl;
    s_lls_env.lls_serv_db.uuid                 = s_lls_svc_uuid;
    s_lls_env.lls_serv_db.attr_tab_cfg         = &s_char_mask;
    s_lls_env.lls_serv_db.max_nb_attr          = LLS_IDX_NB;
    s_lls_env.lls_serv_db.srvc_perm            = 0;
    s_lls_env.lls_serv_db.attr_tab_type        = BLE_GATTS_SERVICE_TABLE_TYPE_16;
    s_lls_env.lls_serv_db.attr_tab.attr_tab_16 = lls_attr_tab;

    return ble_gatts_prf_add(&s_lls_env.lls_serv_db, lls_ble_evt_handler);
}

sdk_err_t lls_alert_level_get(uint8_t *p_alert_level)
{
    uint16_t handle = prf_find_handle_by_idx(LLS_IDX_ALERT_LVL_VAL, s_lls_env.start_hdl, (uint8_t *)&s_char_mask);
    uint16_t length = sizeof(uint8_t);

    return ble_gatts_value_get(handle, &length, p_alert_level);
}


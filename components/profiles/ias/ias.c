/**
 *****************************************************************************************
 *
 * @file ias.c
 *
 * @brief Immediate Alert Server Implementation.
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
#include "ias.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * DEFINITIONS
 *****************************************************************************************
 */
#define INITIAL_ALERT_LEVEL IAS_ALERT_NONE

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief IAS Attributes database index list. */
enum ias_attr_idx_t
{
    IAS_IDX_SVC,

    IAS_IDX_ALERT_LVL_CHAR,
    IAS_IDX_ALERT_LVL_VAL,

    IAS_IDX_NB,
};

/**@brief Immediate Alert Service environment variable. */
struct ias_env_t
{
    ias_init_t ias_init;                    /**< Immediate Alert Service initialization variables. */
    uint16_t   start_hdl;                   /**< Immediate Alert Service start handle. */
    ble_gatts_create_db_t     ias_serv_db;  /**< Immediate Alert Service DataBase. */
};
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct ias_env_t s_ias_env;   /**< Immediate Alert Service instance. */
static uint8_t s_char_mask = 0x07;   /**< Features added into ATT database.
                                      *   bit0 - Immediate Alert Service Declaration
                                      *   bit1 - Alert Level Characteristic Declaration
                                      *   bit2 - Alert Level Characteristic Value
                                      */
static const uint8_t s_ias_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_IMMEDIATE_ALERT);

/**@brief IAS Database Description - Used to add attributes into the database. */ 
static const ble_gatts_attm_desc_t ias_attr_tab[IAS_IDX_NB] =
{
    // Immediate Alert Service Declaration
    [IAS_IDX_SVC]            = {BLE_ATT_DECL_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Alert Level Characteristic Declaration
    [IAS_IDX_ALERT_LVL_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC,  BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Alert Level Characteristic Value
    [IAS_IDX_ALERT_LVL_VAL]  = {BLE_ATT_CHAR_ALERT_LEVEL,     BLE_GATTS_WRITE_CMD_PERM_UNSEC, 0, sizeof(uint8_t)},
};

/*
 * LOCAL FUNCTION DECLARATIONS
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize Immediate Alert Service and create DB in ATT.
 *
 * @return Error code to know if service initialization succeed or not.
 *****************************************************************************************
 */

/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_param:  The parameters of the read request.
 *****************************************************************************************
 */
static void ias_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t handle = prf_find_handle_by_idx(IAS_IDX_ALERT_LVL_VAL, s_ias_env.start_hdl, &s_char_mask);
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
            if (BLE_SUCCESS == cfm.status)
            {
                /* Alert level updated by the peer, notify app the event. */
                if (s_ias_env.ias_init.evt_handler)
                {
                    ias_evt_t evt;

                    evt.evt_type    = IAS_EVT_ALERT_LEVEL_UPDATED;
                    evt.alert_level = value;
                    s_ias_env.ias_init.evt_handler(&evt);
                }
            }
        }
    }

    ble_gatts_write_cfm(conn_idx, &cfm);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t ias_alert_level_get(uint8_t *p_alert_level)
{
    uint16_t handle = prf_find_handle_by_idx(IAS_IDX_ALERT_LVL_VAL, s_ias_env.start_hdl, (uint8_t *)&s_char_mask);
    uint16_t length = sizeof(uint8_t);

    return ble_gatts_value_get(handle, &length, p_alert_level);
}

static void ias_ble_evt_handler(const ble_evt_t *p_evt)
{
    if(NULL == p_evt)
    {
        return ;
    }

    switch(p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            break;
        case BLE_GATTS_EVT_WRITE_REQUEST:
            ias_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;
        case BLE_GATTS_EVT_NTF_IND:
            break;
        case BLE_GATTS_EVT_CCCD_RECOVERY:
            break;
        default:
            break;
    }
}
sdk_err_t ias_service_init(ias_init_t *p_ias_init)
{
    if (NULL == p_ias_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    s_ias_env.ias_init.evt_handler = p_ias_init->evt_handler;

    memset(&s_ias_env.ias_serv_db, 0, sizeof(ble_gatts_create_db_t));

    s_ias_env.start_hdl           = PRF_INVALID_HANDLE;
    s_ias_env.ias_serv_db.shdl                 = &s_ias_env.start_hdl;
    s_ias_env.ias_serv_db.uuid                 = s_ias_svc_uuid;
    s_ias_env.ias_serv_db.attr_tab_cfg         = &s_char_mask;
    s_ias_env.ias_serv_db.max_nb_attr          = IAS_IDX_NB;
    s_ias_env.ias_serv_db.srvc_perm            = 0;
    s_ias_env.ias_serv_db.attr_tab_type        = BLE_GATTS_SERVICE_TABLE_TYPE_16;
    s_ias_env.ias_serv_db.attr_tab.attr_tab_16 = ias_attr_tab;

    return ble_gatts_prf_add(&s_ias_env.ias_serv_db, ias_ble_evt_handler);
}

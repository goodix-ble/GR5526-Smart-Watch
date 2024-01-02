/**
 ****************************************************************************************
 *
 * @file ble_gatt_service.c
 *
 * @brief BLE GATT Service Module API
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
#include "ble_gatt_service.h"
#include "ble_prf_types.h"
#include "custom_config.h"


/*
 * DEFINES
 *****************************************************************************************
 */
#define BLE_GATTS_ATT_ELEMENT_SIZE    sizeof(attm_desc_128_t)
#define BLE_GATTS_HADNLE_PTR_SIZE     sizeof(uint16_t *)
#define BLE_GATTS_INVALID_SERV_IDX    0xff

#define BLE_GATT_REALLOC_VERIFY(ptr, offset, size)  \
do                                                  \
{                                                   \
    if (NULL == ptr)                                \
    {                                               \
        return SDK_ERR_NO_RESOURCES;                \
    }                                               \
    memset(&ptr[offset], 0, size);                  \
} while(0);

#define BLE_GATT_MALLOC_VERIFY(ptr, size)   \
do                                          \
{                                           \
    if (NULL == ptr)                        \
    {                                       \
        return SDK_ERR_NO_RESOURCES;        \
    }                                       \
    memset(ptr, 0, size);                   \
} while(0);

/*
 * STRUCTURES
 *****************************************************************************************
 */
typedef struct
{
    uint16_t              start_hdl;         /**< Start handle. */
    uint16_t              end_hdl;           /**< End handle. */
    ble_gatts_handler_t   handlers;          /**< Att handlers. */
} ble_gatts_serv_info_t;

typedef struct
{
    
    uint8_t          inc_serv_num;      /**< Number of include services. */
    uint16_t        **p_inc_hdl_tab;    /**< Pointer to include service handle pointer table. */
    uint16_t        **p_att_hdl_tab;    /**< Pointer to attributehandle pointer table. */
    attm_desc_128_t  *p_db_tab;         /**< Pointer to db table. */
    ble_att_uuid_t   uuid;              /**< Service UUID. */
    uint8_t          att_num;           /**< Number of attributes. */
} ble_gatts_db_info_t;

typedef struct
{
    uint8_t                   serv_num;
    uint16_t                  start_hdl;
    ble_gatts_serv_info_t    *p_serv_info;
} ble_gatts_serv_env_t;

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static ble_err_t ble_gatts_serv_load(void);
static void      ble_gatts_read_cb(uint8_t conn_idx, const gatts_read_req_cb_t *p_read_req);
static void      ble_gatts_write_cb(uint8_t conn_idx, const gatts_write_req_cb_t *p_write_req);
static void      ble_gatts_prep_write_cb(uint8_t conn_idx, const gatts_prep_write_req_cb_t *p_prep_write_req);
static void      ble_gatts_ntf_ind_cplt_cb(uint8_t conn_idx, uint8_t status, const ble_gatts_ntf_ind_t *p_ntf_ind);
static void      ble_gatts_cccd_recovery_cb(uint8_t conidx, uint16_t handle, uint16_t cccd_val);
static void      ble_gatts_on_connect_cb(uint8_t conn_idx);
static void      ble_gatts_on_disconnect_cb(uint8_t conn_idx, uint8_t reason);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_prf_manager_cbs_t s_mgr_cbs =
{
    ble_gatts_serv_load,
    ble_gatts_on_connect_cb,
    ble_gatts_on_disconnect_cb
};

static gatts_prf_cbs_t s_gatts_cbs =
{
    ble_gatts_read_cb,
    ble_gatts_write_cb,
    ble_gatts_prep_write_cb,
    ble_gatts_ntf_ind_cplt_cb,
    ble_gatts_cccd_recovery_cb
};

static const prf_server_info_t s_prf_info =
{
    .max_connection_nb = CFG_MAX_CONNECTIONS,
    .manager_cbs       = &s_mgr_cbs,
    .gatts_prf_cbs     = &s_gatts_cbs
};

static uint8_t                s_create_serv_idx;
static uint8_t                s_load_serv_idx;
static ble_gatts_db_info_t   *s_p_db_info_tab;
static ble_gatts_serv_env_t   s_gatts_serv_env;


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint8_t ble_gatts_serv_indx_find(uint16_t handle)
{
    for (uint8_t i = 0; i < s_gatts_serv_env.serv_num; i++)
    {
        if (handle >= s_gatts_serv_env.p_serv_info[i].start_hdl &&
            handle <= s_gatts_serv_env.p_serv_info[i].end_hdl)
        {
            return i;
        }
    }

    return BLE_GATTS_INVALID_SERV_IDX;
}

extern uint8_t fpb_save_state(void); 
extern void fpb_load_state(uint8_t state);

static ble_err_t ble_gatts_serv_load(void)
{
    fpb_load_state(0);
    sdk_err_t         error_code;
    gatts_create_db_t gatts_db;
    uint8_t           uuid[] = BLE_ATT_16_TO_128_ARRAY(s_p_db_info_tab[s_load_serv_idx].uuid.uuid.uuid16);

    
    memset(&gatts_db, 0, sizeof(gatts_create_db_t));

    s_gatts_serv_env.p_serv_info[s_load_serv_idx].start_hdl = PRF_INVALID_HANDLE;

    if (s_p_db_info_tab[s_load_serv_idx].uuid.type == BLE_ATT_UUID_128)
    {
        gatts_db.uuid      = s_p_db_info_tab[s_load_serv_idx].uuid.uuid.uuid128;
        gatts_db.srvc_perm = SRVC_UUID_TYPE_SET(UUID_TYPE_128);
    }
    else
    {
        gatts_db.uuid      = uuid;
        gatts_db.srvc_perm = 0;
    }

    gatts_db.shdl            = &s_gatts_serv_env.p_serv_info[s_load_serv_idx].start_hdl;
    gatts_db.attr_tab_cfg    = NULL;
    gatts_db.max_nb_attr     = s_p_db_info_tab[s_load_serv_idx].att_num;
    gatts_db.attr_tab_type   = SERVICE_TABLE_TYPE_128;
    gatts_db.attr_tab.attr_tab_128 = s_p_db_info_tab[s_load_serv_idx].p_db_tab;
    gatts_db.inc_srvc_num    = s_p_db_info_tab[s_load_serv_idx].inc_serv_num;

    for (uint8_t i = 0; i < gatts_db.inc_srvc_num; i++)
    {
        gatts_db.inc_srvc_handle[i] = s_p_db_info_tab[s_load_serv_idx].p_inc_hdl_tab[i];
    }

    error_code = ble_gatts_srvc_db_create(&gatts_db);

    if (SDK_SUCCESS == error_code)
    {
        s_gatts_serv_env.p_serv_info[s_load_serv_idx].end_hdl = s_gatts_serv_env.p_serv_info[s_load_serv_idx].start_hdl + \
                                                                s_p_db_info_tab[s_load_serv_idx].att_num - 1;
    }

    for (uint8_t i = 0; i < gatts_db.max_nb_attr; i++)
    {
        *(s_p_db_info_tab[s_load_serv_idx].p_att_hdl_tab[i]) = *(gatts_db.shdl) + i;
    }

    if (s_p_db_info_tab[s_load_serv_idx].p_inc_hdl_tab)
    {
        BLE_GATT_SERV_FREE(s_p_db_info_tab[s_load_serv_idx].p_inc_hdl_tab);
    }


    BLE_GATT_SERV_FREE(s_p_db_info_tab[s_load_serv_idx].p_att_hdl_tab);
    BLE_GATT_SERV_FREE(s_p_db_info_tab[s_load_serv_idx].p_db_tab);


    s_load_serv_idx++;

    if (s_load_serv_idx == s_gatts_serv_env.serv_num)
    {
        BLE_GATT_SERV_FREE(s_p_db_info_tab);
    }

    fpb_save_state();
    return error_code;
}

static void ble_gatts_read_cb(uint8_t conn_idx, const gatts_read_req_cb_t *p_read_req)
{
    uint8_t  sevr_idx = ble_gatts_serv_indx_find(p_read_req->handle);

    if (sevr_idx != BLE_GATTS_INVALID_SERV_IDX &&
        s_gatts_serv_env.p_serv_info[sevr_idx].handlers.read_req_handler)
    {
        s_gatts_serv_env.p_serv_info[sevr_idx].handlers.read_req_handler(conn_idx, p_read_req->handle);
    }
}

static void ble_gatts_write_cb(uint8_t conn_idx, const gatts_write_req_cb_t *p_write_req)
{
    uint8_t  sevr_idx = ble_gatts_serv_indx_find(p_write_req->handle);

    gatts_write_cfm_t cfm;

    cfm.handle = p_write_req->handle;
    cfm.status = sevr_idx == BLE_GATTS_INVALID_SERV_IDX ? BLE_ATT_ERR_INVALID_HANDLE : BLE_SUCCESS;

    ble_gatts_write_cfm(conn_idx, &cfm);

    if (sevr_idx != BLE_GATTS_INVALID_SERV_IDX &&
        s_gatts_serv_env.p_serv_info[sevr_idx].handlers.write_req_handler)
    {
        s_gatts_serv_env.p_serv_info[sevr_idx].handlers.write_req_handler(conn_idx,
                                                                             p_write_req->handle,
                                                                             p_write_req->offset,
                                                                             p_write_req->value,
                                                                             p_write_req->length);
    }
}

static void ble_gatts_prep_write_cb(uint8_t conn_idx, const gatts_prep_write_req_cb_t *p_prep_write_req)
{
    uint8_t  sevr_idx = ble_gatts_serv_indx_find(p_prep_write_req->handle);

    gatts_prep_write_cfm_t cfm;

    cfm.handle = p_prep_write_req->handle;
    cfm.status = sevr_idx == BLE_GATTS_INVALID_SERV_IDX ? BLE_ATT_ERR_INVALID_HANDLE : BLE_SUCCESS;

    ble_gatts_prepare_write_cfm(conn_idx, &cfm);
}

static void ble_gatts_ntf_ind_cplt_cb(uint8_t conn_idx, uint8_t status, const ble_gatts_ntf_ind_t *p_ntf_ind)
{
    uint8_t  sevr_idx = ble_gatts_serv_indx_find(p_ntf_ind->handle);
    uint16_t h_offset = p_ntf_ind->handle - s_gatts_serv_env.start_hdl;

    if (sevr_idx != BLE_GATTS_INVALID_SERV_IDX &&
        s_gatts_serv_env.p_serv_info[sevr_idx].handlers.ntf_ind_handler)
    {
        s_gatts_serv_env.p_serv_info[sevr_idx].handlers.ntf_ind_handler(conn_idx, status, p_ntf_ind->type, h_offset);
    }
}

static void ble_gatts_cccd_recovery_cb(uint8_t conn_idx, uint16_t handle, uint16_t cccd_val)
{
    uint8_t  sevr_idx = ble_gatts_serv_indx_find(handle);

    if (sevr_idx != BLE_GATTS_INVALID_SERV_IDX &&
        s_gatts_serv_env.p_serv_info[sevr_idx].handlers.cccd_recovery_handler)
    {
        s_gatts_serv_env.p_serv_info[sevr_idx].handlers.cccd_recovery_handler(conn_idx, handle, cccd_val);
    }
}

static void  ble_gatts_on_connect_cb(uint8_t conn_idx)
{
    fpb_load_state(0);

    for (uint8_t i = 0; i < s_gatts_serv_env.serv_num; i++)
    {
        if (s_gatts_serv_env.p_serv_info[i].handlers.on_conn_handler)
        {
            s_gatts_serv_env.p_serv_info[i].handlers.on_conn_handler(conn_idx);
        }
    }
    fpb_save_state();
}

static void  ble_gatts_on_disconnect_cb(uint8_t conn_idx, uint8_t reason)
{
    for (uint8_t i = 0; i < s_gatts_serv_env.serv_num; i++)
    {
        if (s_gatts_serv_env.p_serv_info[i].handlers.on_disconn_handler)
        {
            s_gatts_serv_env.p_serv_info[i].handlers.on_disconn_handler(conn_idx, reason);
        }
    }
}

static void ble_gatts_att_perm_set(uint16_t *p_att_perm, uint16_t prop, ble_att_perm_t perm)
{
    if (NULL == p_att_perm || 0 == prop)
    {
        return;
    }

    *p_att_perm = 0;

    if (prop & BLE_ATT_PROP_BROADCAST)
    {
        *p_att_perm |= (BROADCAST << 8);
    }

    if (prop & BLE_ATT_PROP_READ)
    {
        *p_att_perm |= (READ << 8 | (((perm) & SEC_LEVEL_MASK) << READ_POS));
    }

    if (prop & BLE_ATT_PROP_WRITE_NO_RESP)
    {
        *p_att_perm  |= (WRITE_CMD << 8 | (((perm) & SEC_LEVEL_MASK) << WRITE_POS));
    }

    if (prop & BLE_ATT_PROP_WRITE)
    {
        *p_att_perm  |= (WRITE_REQ << 8 | (((perm) & SEC_LEVEL_MASK) << WRITE_POS));
    }

    if (prop & BLE_ATT_PROP_NOTIFY)
    {
        *p_att_perm  |= (NOTIFY << 8 | (((perm) & SEC_LEVEL_MASK) << NOTIFY_POS));
    }

    if (prop & BLE_ATT_PROP_INDICATE)
    {
        *p_att_perm  |= (INDICATE << 8 | (((perm) & SEC_LEVEL_MASK) << INDICATE_POS));
    }

    if (prop & BLE_ATT_PROP_WRITE_SIGNED)
    {
        *p_att_perm  |= (WRITE_SIGNED << 8 | (((perm) & SEC_LEVEL_MASK) << WRITE_POS));
    }

    if (prop & BLE_ATT_PROP_EXTENDED)
    {
        *p_att_perm  |= (EXT_PROP << 8);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
sdk_err_t ble_gatts_serv_add(bool is_primary, const ble_att_uuid_t *p_uuid, uint16_t *p_handle)
{
    size_t    re_size;
    void     *mal_ptr;
    uint8_t   att_idx;

    if (NULL == p_uuid)
    {
        return SDK_ERR_POINTER_NULL;
    }

    // re-malloc s_p_db_info_tab
    mal_ptr = s_p_db_info_tab;
    re_size = sizeof(ble_gatts_db_info_t) * (s_create_serv_idx + 1);

    s_p_db_info_tab = BLE_GATT_SERV_REALLOC(mal_ptr, re_size);
    BLE_GATT_REALLOC_VERIFY(s_p_db_info_tab, s_create_serv_idx, sizeof(ble_gatts_db_info_t));

    // re-malloc s_gatts_serv_env.p_serv_info
    mal_ptr = s_gatts_serv_env.p_serv_info;
    re_size = sizeof(ble_gatts_serv_info_t) * (s_gatts_serv_env.serv_num + 1);

    s_gatts_serv_env.p_serv_info = BLE_GATT_SERV_REALLOC(mal_ptr, re_size);
    BLE_GATT_REALLOC_VERIFY(s_gatts_serv_env.p_serv_info, s_gatts_serv_env.serv_num, sizeof(ble_gatts_serv_info_t));

    // malloc att handle ptr table
    s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab = BLE_GATT_SERV_MALLOC(BLE_GATTS_HADNLE_PTR_SIZE);
    BLE_GATT_MALLOC_VERIFY(s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab, BLE_GATTS_HADNLE_PTR_SIZE);

    att_idx = s_p_db_info_tab[s_create_serv_idx].att_num;
    s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab[att_idx] = p_handle;

    // re-malloc att table
    s_p_db_info_tab[s_create_serv_idx].p_db_tab = BLE_GATT_SERV_MALLOC(BLE_GATTS_ATT_ELEMENT_SIZE);
    BLE_GATT_MALLOC_VERIFY(s_p_db_info_tab[s_create_serv_idx].p_db_tab, BLE_GATTS_ATT_ELEMENT_SIZE);

    // record service uuid
    memcpy(&s_p_db_info_tab[s_create_serv_idx].uuid, p_uuid, sizeof(ble_att_uuid_t));

    // ->uuid
    if (is_primary)
    {
        uint8_t uuid[16] = BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_PRIMARY_SERVICE);
        memcpy(s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].uuid, uuid, 16);
    }
    else
    {
        uint8_t uuid[16] = BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_SECONDARY_SERVICE);
        memcpy(s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].uuid, uuid, 16);
    }
    // ->perm
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].perm = READ_PERM_UNSEC;
    // ->ext_perm
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].ext_perm = 0;
    // ->max_size
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].max_size = 0;

    s_gatts_serv_env.serv_num++;
    s_p_db_info_tab[s_create_serv_idx].att_num++;

    return SDK_SUCCESS;
}

sdk_err_t ble_gatts_inc_serv_add(uint16_t *p_handle)
{
    size_t   re_size;
    void    *mal_ptr;
    uint8_t  inc_serv_idx;
    uint8_t  att_idx;

    if (NULL == p_handle)
    {
        return SDK_ERR_POINTER_NULL;
    }

    att_idx = s_p_db_info_tab[s_create_serv_idx].att_num;

    // re-malloc att handle ptr table
    mal_ptr = s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab;
    re_size = BLE_GATTS_HADNLE_PTR_SIZE * (s_p_db_info_tab[s_create_serv_idx].att_num + 1);

    s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab = BLE_GATT_SERV_REALLOC(mal_ptr, re_size);
    BLE_GATT_REALLOC_VERIFY(s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab, att_idx, BLE_GATTS_HADNLE_PTR_SIZE);

    // re-malloc att table
    mal_ptr = s_p_db_info_tab[s_create_serv_idx].p_db_tab;
    re_size = BLE_GATTS_ATT_ELEMENT_SIZE * (att_idx + 1);

    s_p_db_info_tab[s_create_serv_idx].p_db_tab = BLE_GATT_SERV_REALLOC(mal_ptr, re_size);
    BLE_GATT_REALLOC_VERIFY(s_p_db_info_tab[s_create_serv_idx].p_db_tab, att_idx, BLE_GATTS_ATT_ELEMENT_SIZE);

    // ->uuid
    uint8_t uuid[16] = BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_INCLUDE);
    memcpy(s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].uuid, uuid, 16);
    // ->perm
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].perm = READ_PERM_UNSEC;
    // ->ext_perm
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].ext_perm = 0;
    // ->max_size
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].max_size = 0;

    inc_serv_idx = s_p_db_info_tab[s_create_serv_idx].inc_serv_num;

    // re-malloc p_inc_hdl_table
    mal_ptr = s_p_db_info_tab[s_create_serv_idx].p_inc_hdl_tab;
    re_size = BLE_GATTS_HADNLE_PTR_SIZE * (s_p_db_info_tab[s_create_serv_idx].inc_serv_num + 1);
    s_p_db_info_tab[s_create_serv_idx].p_inc_hdl_tab = BLE_GATT_SERV_REALLOC(mal_ptr, re_size);
    BLE_GATT_REALLOC_VERIFY(s_p_db_info_tab[s_create_serv_idx].p_inc_hdl_tab, inc_serv_idx, BLE_GATTS_HADNLE_PTR_SIZE);

    s_p_db_info_tab[s_create_serv_idx].p_inc_hdl_tab = mal_ptr;
    s_p_db_info_tab[s_create_serv_idx].p_inc_hdl_tab[inc_serv_idx] = p_handle;
    s_p_db_info_tab[s_create_serv_idx].inc_serv_num++;

    s_p_db_info_tab[s_create_serv_idx].att_num++;

    return SDK_SUCCESS;
}

sdk_err_t ble_gatts_characteristic_add(ble_gatts_att_t *p_att, uint16_t *p_val_handle)
{
    size_t   re_size;
    void    *mal_ptr;
    uint8_t  att_idx;

    if (NULL == p_att)
    {
        return SDK_ERR_POINTER_NULL;
    }

    att_idx = s_p_db_info_tab[s_create_serv_idx].att_num;

    // re-malloc att handle ptr table
    mal_ptr = s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab;
    re_size = BLE_GATTS_HADNLE_PTR_SIZE * (s_p_db_info_tab[s_create_serv_idx].att_num + 2);

    s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab = BLE_GATT_SERV_REALLOC(mal_ptr, re_size);
    BLE_GATT_REALLOC_VERIFY(s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab, att_idx, BLE_GATTS_HADNLE_PTR_SIZE * 2);

    // re-malloc att table
    mal_ptr = s_p_db_info_tab[s_create_serv_idx].p_db_tab;
    re_size = BLE_GATTS_ATT_ELEMENT_SIZE * (att_idx + 2);

    s_p_db_info_tab[s_create_serv_idx].p_db_tab = BLE_GATT_SERV_REALLOC(mal_ptr, re_size);
    BLE_GATT_REALLOC_VERIFY(s_p_db_info_tab[s_create_serv_idx].p_db_tab, att_idx, BLE_GATTS_ATT_ELEMENT_SIZE * 2);

    // Character dec
    // ->uuid
    uint8_t uuid[16] = BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC);
    memcpy(s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].uuid, uuid, 16);
    // ->perm
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].perm = READ_PERM_UNSEC;
    // ->ext_perm
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].ext_perm = 0;
    // ->max_size
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].max_size = 0;

    s_p_db_info_tab[s_create_serv_idx].att_num++;
    att_idx++;

    // Character val
    // ->uuid
    if (p_att->att_uuid.type == BLE_ATT_UUID_128)
    {
        memcpy(s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].uuid, p_att->att_uuid.uuid.uuid128, 16);
    }
    else
    {
        uint8_t uuid[16] = BLE_ATT_16_TO_128_ARRAY(p_att->att_uuid.uuid.uuid16);
        memcpy(s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].uuid, uuid, 16);
    }
    // ->perm
    ble_gatts_att_perm_set(&s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].perm, p_att->att_prop, p_att->att_perm);
    // ->ext_perm
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].ext_perm = ATT_VAL_LOC_USER;
    if (p_att->att_uuid.type == BLE_ATT_UUID_128)
    {
        s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].ext_perm |= SERVICE_TABLE_TYPE_128;
    }
    // ->max_size
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].max_size = p_att->max_len;

    s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab[att_idx] = p_val_handle;

    s_p_db_info_tab[s_create_serv_idx].att_num++;

    return SDK_SUCCESS;
}

sdk_err_t ble_gatts_descriptor_add(ble_gatts_att_t *p_att, uint16_t *p_handle)
{
    size_t   re_size;
    void    *mal_ptr;
    uint8_t  att_idx;

    if (NULL == p_att)
    {
        return SDK_ERR_POINTER_NULL;
    }

    att_idx = s_p_db_info_tab[s_create_serv_idx].att_num;

    // re-malloc att handle ptr table
    mal_ptr = s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab;
    re_size = BLE_GATTS_HADNLE_PTR_SIZE * (s_p_db_info_tab[s_create_serv_idx].att_num + 1);

    s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab = BLE_GATT_SERV_REALLOC(mal_ptr, re_size);
    BLE_GATT_REALLOC_VERIFY(s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab, att_idx, BLE_GATTS_HADNLE_PTR_SIZE);

    // re-malloc att table
    mal_ptr = s_p_db_info_tab[s_create_serv_idx].p_db_tab;
    re_size = BLE_GATTS_ATT_ELEMENT_SIZE * (att_idx + 1);

    s_p_db_info_tab[s_create_serv_idx].p_db_tab = BLE_GATT_SERV_REALLOC(mal_ptr, re_size);
    BLE_GATT_REALLOC_VERIFY(s_p_db_info_tab[s_create_serv_idx].p_db_tab, att_idx, BLE_GATTS_ATT_ELEMENT_SIZE);

    // ->uuid
    if (p_att->att_uuid.type == BLE_ATT_UUID_128)
    {
        memcpy(s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].uuid, p_att->att_uuid.uuid.uuid128, 16);
    }
    else
    {
        uint8_t uuid[16] = BLE_ATT_16_TO_128_ARRAY(p_att->att_uuid.uuid.uuid16);
        memcpy(s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].uuid, uuid, 16);
    }

    // ->perm
    ble_gatts_att_perm_set(&s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].perm, p_att->att_prop, p_att->att_perm);
    // ->ext_perm
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].ext_perm = ATT_VAL_LOC_USER;
    if (p_att->att_uuid.type == BLE_ATT_UUID_128)
    {
        s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].ext_perm |= SERVICE_TABLE_TYPE_128;
    }
    // ->max_size
    s_p_db_info_tab[s_create_serv_idx].p_db_tab[att_idx].max_size = p_att->max_len;

    s_p_db_info_tab[s_create_serv_idx].p_att_hdl_tab[att_idx] = p_handle;

    s_p_db_info_tab[s_create_serv_idx].att_num++;

    return SDK_SUCCESS;
}

sdk_err_t ble_gatts_serv_enable(ble_gatts_handler_t *p_att_handler)
{
    if (NULL == p_att_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memcpy(&s_gatts_serv_env.p_serv_info[s_create_serv_idx].handlers, p_att_handler, sizeof(ble_gatts_handler_t));

    return ble_server_prf_add(&s_prf_info);
}

sdk_err_t ble_gatts_att_req_reply(uint8_t conn_idx, ble_gatts_att_req_reply_t *p_att_reply)
{
    if (NULL == p_att_reply)
    {
        return SDK_ERR_POINTER_NULL;
    }

    if (BLE_ATT_READ_REPLY == p_att_reply->type)
    {
        gatts_read_cfm_t cfm;

        cfm.handle = p_att_reply->param.rd.handle;
        cfm.status = p_att_reply->param.rd.status;
        cfm.value  = p_att_reply->param.rd.p_value;
        cfm.length = p_att_reply->param.rd.length;

        return ble_gatts_read_cfm(conn_idx, &cfm);
    }
    else if (BLE_ATT_WRITE_REPLY == p_att_reply->type)
    {
        gatts_write_cfm_t cfm;

        cfm.handle = p_att_reply->param.wr.handle;
        cfm.status = p_att_reply->param.wr.status;

        return ble_gatts_write_cfm(conn_idx, &cfm);
    }
    else
    {
        return SDK_ERR_INVALID_PARAM;
    }
}


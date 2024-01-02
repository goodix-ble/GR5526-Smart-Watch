/**
 *****************************************************************************************
 *
 * @file dss.h
 *
 * @brief Device Synchronize Service API.
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_DSS Device Synchronize Service (DSS)
 * @{
 * @brief Definitions and prototypes for the DSS interface.
 *
 * @details The Device Synchronize Service is set for timing synchronization of device group,
 *          which can accept commands such as device role and timing synchronization. In addition,
 *          related information of device timing synchronization can be got through DSS.
 *
 *          After dss_init_t variable is initialized, the application should call
 *          \ref dss_service_init() to add a Device Synchronize Service and the characteristics
 *          to the BLE Stack database.
 *
 */

#ifndef __DSS_H__
#define __DSS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>

/**
 * @defgroup DSS_MACRO Defines
 * @{
 */
#define DSS_SERVICE_UUID   0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, \
                           0x0A, 0x46, 0x44, 0xD3, 0x01, 0x0A, 0xED, 0xA6    /**< DSS service UUID. */

#define DSS_CONNECTION_MAX                  10                              /**< Maximum number of DSS connections. */
#define DSS_ROLE_VALUE_LEN                  1                               /**< Length of Role characteristic value. */
#define DSS_EVT_CNT_VALUE_LEN               4                               /**< Length of Event Count characteristic value. */
#define DSS_EVT_PERIOD_VALUE_LEN            2                               /**< Length of Event Period characteristic value. */
#define DSS_STATUS_VALUE_LEN                1                               /**< Length of Status characteristic value. */
#define DSS_CTRL_PT_VALUE_LEN               7                               /**< Length of Control Point characteristic value. */
#define DSS_CTRL_PT_RSP_VAL_LEN             3                               /**< Length of Control Point Response characteristic value. */

#define DSS_SYNC_DEV_MAX_NUM                5                               /**< Maximun num of Source Sync Device. */
#define DSS_CFG_ADV_IDX                     0                               /**< DSS Config Advertising Index. */
#define DSS_SYNC_ADV_IDX                    1                               /**< DSS Sync Advertising Index. */
/** @} */

/**
 * @defgroup DSS_ENUM Enumerations
 * @{
 */
/**@brief Device Synchronize Service roles. */
typedef enum
{
    DSS_ROLE_SYNC_INVALID,        /**< Device synchronize invalid role. */
    DSS_ROLE_SYNC_SOURCE,         /**< Device synchronize source role (Create synchronize source and distribute). */
    DSS_ROLE_SYNC_DEVICE,         /**< Device synchronize deivce role. */
} dss_role_t;

/**@brief Device Synchronize Service status. */
typedef enum
{
    DSS_STATUS_CFG_READY,             /**< Device is ready for config, */
    DSS_STATUS_IN_ADV,                /**< Device is in advertising. */
    DSS_STATUS_IN_SCAN,               /**< Device is in scanning. */
    DSS_STATUS_IN_INITIATING          /**< Device is in initiating. */
} dss_staus_t;

/**@brief Device Synchronize Service control point OP IDs. */
typedef enum
{
    DSS_OP_ID_INVALID,             /**< Invalid op id. */
    DSS_OP_ID_ROLE_SET,            /**< Set role op id.*/
    DSS_OP_ID_SYNC_SRC_CREATE,     /**< Create synchronize source op id. */
    DSS_OP_ID_SYNC,                /**< Synchronize self or peer op id. */
    DSS_OP_ID_CANCEL_SYNC,         /**< Cancel Synchronization op id. */
    DSS_OP_ID_LP_ENTER,            /**< Enter low power mode(Stop all ble activity). */
    DSS_OP_ID_SYNC_DESTROY,        /**< Destroy sync. */
    DSS_OP_ID_RSP          = 0xff  /**< Response op id. */
} dss_op_id_t;

/**@brief Device Synchronize Service control point response IDs. */
typedef enum
{
    DSS_RSP_ID_SUCCESS,            /**< Success. */
    DSS_RSP_ID_UNSUPPORT,          /**< Unsupport op. */
    DSS_RSP_ID_DISALLOWED,         /**< Disallowed op. */
    DSS_RSP_ID_STATUS_ERR,         /**< Status error. */
    DSS_RSP_ID_PARAM_ERR,          /**< Parameter error. */
    DSS_RSP_ID_ROLE_ERR,           /**< Role error. */
    DSS_RSP_ID_NO_HANDLER,         /**< No handler for op. */
    DSS_RSP_ID_ADV_START_FAIL,     /**< Advertising start fail. */
    DSS_RSP_ID_ADV_TIMEOUT,        /**< Advertising start timeout. */
    DSS_RSP_ID_SCAN_START_FAIL,    /**< Scan start fail. */
    DSS_RSP_ID_SCAN_TIMEOUT,       /**< Scan start timeout. */
    DSS_RSP_ID_CONN_EST_FAIL,      /**< Connection establish fail. */
    DSS_RSP_ID_CREATE_SRC_FAIL,    /**< Create source fail. */
    DSS_RSP_ID_DISTR_SRC_FAIL,     /**< Distribute source fail. */
    DSS_RSP_ID_DESTROY_SRC_FAIL,   /**< Destroy source fail. */
    DSS_RSP_ID_ENTER_LP_FAIL,      /**< Enter Low Power Mode fail. */
    DSS_RSP_ID_CANCEL_SYNC_FAIL,   /**< Cancel Synchronization fail. */
} dss_rsp_id_t;


/**@brief Device Synchronize Service event types. */
typedef enum
{
    DSS_EVT_INVALID,              /**< Invalid event. */
    DSS_EVT_SOURCE_ROLE_SET,      /**< Source Role set event. */
    DSS_EVT_DEVICE_ROLE_SET,      /**< Device Role set event. */
    DSS_EVT_SYNC_SRC_CREATE,      /**< Sync source create event. */
    DSS_EVT_SYNC_DESTROY,         /**< Destroy sync event. */
    DSS_EVT_SYNC_OCCUR,           /**< Sync occur event. */
    DSS_EVT_SYNC_SELF_OR_PEER,    /**< Synchronize self or peer event. */
    DSS_EVT_SYNC_CANCEL,          /**< Cancel Synchronization event. */
    DSS_EVT_LP_ENTER,             /**< Enter low power event. */
} dss_evt_type_t;
/** @} */

/**
 * @defgroup DSS_STRUCTURES Structures
 * @{
 */
/**@brief Device Synchronize Service Synchronize event. */
typedef struct
{
    dss_evt_type_t evt_type;          /**< Event type. */
    uint8_t        conn_idx;          /**< Connect index. */
    uint32_t       sync_cnt;          /**< Synchronize count. */
    uint8_t        sync_dev_num;      /**< Synchronize Device num. */
    bool           is_enter_lp_mode;  /**< In Low Power Mode flag. */
} dss_evt_t;
/** @} */

/**
 * @defgroup DSS_TYPEDEFS Typedefs
 * @{
 */
/**@brief Device Synchronize Service event handler type. */
typedef void (*dss_evt_handler_t)(dss_evt_t *p_evt);
/** @} */


/**
 * @defgroup DIS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Device Synchronize Service instance and add in the database.
 *
 * @param[in] evt_handler: DSS event handler.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t dss_service_init(dss_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Send Control Point Response.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] evt_type: Event type.
 * @param[in] rsp_id:   Response ID.
 *
 * @return Result of send.
 *****************************************************************************************
 */
sdk_err_t dss_sync_op_result_send(uint8_t conn_idx, dss_evt_type_t evt_type, dss_rsp_id_t rsp_id);

/**
 *****************************************************************************************
 * @brief Distribute sync source to peer.
 *
 * @param[in] conn_idx: Connection index.
 *****************************************************************************************
 */
void dss_sync_src_distribute(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Set dss status.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] status:   Dss status.
 *****************************************************************************************
 */
void dss_set_status(uint8_t conn_idx, dss_staus_t status);

/**
 *****************************************************************************************
 * @brief Set Sync params.
 *
 * @param[in] conn_idx:            Connection index.
 * @param[in] is_auto_enter_lp:    Auto enter low power mode flag.
 * @param[in] is_auto_calib_drift: Auto calibration drift flag. 
 *****************************************************************************************
 */
void dss_set_sync_params(uint8_t conn_idx, bool is_auto_enter_lp, bool is_auto_calib_drift);

/**
 *****************************************************************************************
 * @brief Set Device whether in low power mode.
 *
 * @param[in] conn_idx:            Connection index.
 * @param[in] is_in_lp_mode:       Is Device in low power mode. 
 *****************************************************************************************
 */
void dss_set_lp_mode(uint8_t conn_idx, bool is_in_lp_mode);
/** @} */

#endif
/** @} */
/** @} */

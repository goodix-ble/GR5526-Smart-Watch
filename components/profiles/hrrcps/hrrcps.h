/**
 ****************************************************************************************
 *
 * @file hrrcps.h
 *
 * @brief HRS RSCS Relay Control Point Service API.
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_HRRCPS HRS RSCS Relay Control Point Service (HRRCPS)
 * @{
 * @brief HRS RSCS Relay Control Point Service module.
 *
 * @details The HRS RSCS Relay Control Point Service provides Scan Device, Read Characteristic Value,
 *          Enable Notification and Query Connection State for application.
 *
 *          The application must provide an event handler to register, then call \ref hrrcps_service_init()
 *          to add HRS RSCS Relay Control Point Service and HRR Control Point, HRR Control Point Response
 *          characteristics to the BLE Stack database.
 */

#ifndef __HRRCPS_H__
#define __HRRCPS_H__

#include "ble_prf_types.h"
#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup HRRCPS_MACRO Defines
 * @{
 */
#define HRRCPS_CONNECTION_MAX       10                                                  /**< Maximum number of HRS RSCS Relay Control Point Service connections. */
#define HRRCPS_CTRL_PT_VAL_LEN      2                                                   /**< Length of the value of Control Point characteristic. */
#define HRRCPS_CTRL_PT_RSP_VAL_LEN  4                                                   /**< Length of the value of Control Point Response characteristic. */
#define HRRCPS_SERVICE_UUID         0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                    0x0A, 0x46, 0x44, 0xD3, 0x01, 0x06, 0xED, 0xA6      /**< The UUID of HRS RSCS Relay Control Point Service for setting advertising data. */
/** @} */

/**
 * @defgroup HRRCPS_ENUM Enumerations
 * @{
 */
/**@brief HRS RSCS Relay Control Point Service Control Point IDs. */
typedef enum
{
    HRRCPS_CTRL_PT_SCAN_HRS = 0x01,        /**< Scan HRS device. */
    HRRCPS_CTRL_PT_SCAN_RSCS,              /**< Scan RSCS device. */
    HRRCPS_CTRL_PT_HRS_SEN_LOC_READ,       /**< Read HRS sensor location. */
    HRRCPS_CTRL_PT_RSCS_SEN_LOC_READ,      /**< Read RSCS sensor location. */
    HRRCPS_CTRL_PT_HRS_NTF_ENABLE,         /**< Enable HRS notification. */
    HRRCPS_CTRL_PT_HRS_NTF_DISABLE,        /**< Disable HRS notification. */
    HRRCPS_CTRL_PT_RSCS_NTF_ENABLE,        /**< Enable RSCS notification. */
    HRRCPS_CTRL_PT_RSCS_NTF_DISABLE,       /**< Disable RSCS notification. */
    HRRCPS_CTRL_PT_HRS_CONN_STA_REPORT,    /**< Report HRS connection state. */
    HRRCPS_CTRL_PT_RSCS_CONN_STA_REPORT,   /**< Report RSCS connecntion state. */
    HRRCPS_CTRL_PT_HRS_DISCONN,            /**< Disconnect HRS link. */
    HRRCPS_CTRL_PT_RSCS_DISCONN,           /**< Disconnect RSCS link. */
    HRRCPS_CTRL_PT_RSP_CODE = 0xff,        /**< Response code. */
} hrrcps_ctrl_pt_id_t;

/**@brief HRS RSCS Relay Control Point Service Response IDs of Control Point. */
typedef enum
{
    HRRCPS_RSP_ID_OK = 0x01,               /**< Success. */
    HRRCPS_RSP_ID_ERROR,                   /**< Fail. */
} hrrcps_rsp_id_t;

/**@brief HRS RSCS Relay Control Point Service event type. */
typedef enum
{
    HRRCPS_EVT_INVALID,                     /**< Invalid HRRCPS event type. */
    HRRCPS_EVT_CTRL_PT_IND_ENABLE,          /**< HRR Control Point indicaiton is enabled. */
    HRRCPS_EVT_CTRL_PT_IND_DISABLE,         /**< HRR Control Point indicaiton is disabled. */
    HRRCPS_EVT_SCAN_HRS,                    /**< Scan HRS device. */
    HRRCPS_EVT_SCAN_RSCS,                   /**< Scan RSCS device. */
    HRRCPS_EVT_ENABLE_HRS_NTF,              /**< Enable HRS notification. */
    HRRCPS_EVT_DISABLE_HRS_NTF,             /**< Disable HRS notification. */
    HRRCPS_EVT_ENABLE_RSCS_NTF,             /**< Enable RSCS notificaiton. */
    HRRCPS_EVT_DISABLE_RSCS_NTF,            /**< Disable RSCS notificaiton. */
    HRRCPS_EVT_HRS_SENSOR_LOC_READ,         /**< Read HRS sensor location. */
    HRRCPS_EVT_RSCS_SENSOR_LOC_READ,        /**< Read RSCS sensor location. */
    HRRCPS_EVT_DISCONN_HRS_LINK,            /**< Disconnect HRS link. */
    HRRCPS_EVT_DISCONN_RSCS_LINK,           /**< Disconnect RSCS link. */
} hrrcps_evt_type_t;
/** @} */

/**
 * @defgroup HRRCPS_STRUCT Structures
 * @{
 */
/**@brief HRS RSCS Relay Control Point Response value. */
typedef struct
{
    hrrcps_ctrl_pt_id_t  cmd_id;        /**< Control Point ID. */
    hrrcps_rsp_id_t      rsp_id;        /**< Response ID. */
    bool                 is_inc_prama;  /**< Parameter is included or not. */
    uint8_t              rsp_param;     /**< Response parameters. */
} hrrcps_rsp_val_t;

/**@brief HRS RSCS Relay Control Point Service event. */
typedef struct
{
    uint8_t           conn_idx;        /**< The index of the connection. */
    hrrcps_evt_type_t evt_type;        /**< The HRRCPS event type. */
} hrrcps_evt_t;
/** @} */

/**
 * @defgroup HRRCPS_TYPEDEF Typedefs
 * @{
 */
/**@brief HRS RSCS Relay Control Point Service event handler type.*/
typedef void (*hrrcps_evt_handler_t)(hrrcps_evt_t *p_evt);
/** @} */

/**
 * @defgroup ANS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize an Alert Notification Service instance and add in the DB.
 *
 * @param[in] evt_handler: HRS RSCS Relay Control Point Service event handler.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t hrrcps_service_init(hrrcps_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Send Control Point Response if its indicaiton has been enabled.
 *
 * @param[in] conn_idx:  Connnection index.
 * @param[in] p_rsp_val: Pointer to Response value.
 *
 * @return Result of indicate value
 *****************************************************************************************
 */
sdk_err_t hrrcps_ctrl_pt_rsp_send(uint8_t conn_idx, hrrcps_rsp_val_t *p_rsp_val);
/** @} */

#endif

/** @} */
/** @} */

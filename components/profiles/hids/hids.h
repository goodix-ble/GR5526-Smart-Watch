/**
 *******************************************************************************
 *
 * @file hids.h
 *
 * @brief Human Interface Device Service API
 *
 *******************************************************************************
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
 * @defgroup BLE_SDK_HIDS Human Input Device Service (HIDS)
 * @{
 * @brief Definitions and prototypes for the HIDS interface.
 *
 * @details The HID Service exposes data and associated formatting for HID Devices
 *          and HID Hosts. This module implements the HID Service with HID Information
 *          characteristic, HID Control Point characteristic, Report Map characteristic,
 *          Input/Output/Feature Report characteristics, Boot Keyboard Input characteristic,
 *          Boot Keyboard Output characteristic, Boot Mouse Input Report characteristic.
 *
 *          After \ref hids_init_t variable is initialized, the application must call \ref hids_service_init()
 *          to add the HID Service and the characteristics to the BLE Stack database. However
 *          the array of Report map locates in user space, application must make sure the
 *          array is available.
 *
 *          If Notify is enabled, the value of Input Report characteristic is sent to the
 *          peer when application calls \ref hids_input_rep_send() function. The application is reponsible
 *          for encoding Input Report data as “USB HID Spec”. If an event hanlder is provided by the application,
 *          HID Service will pass HIDS events to the application, e.g. Output Report characteristic is written.
 */

#ifndef __HIDS_H__
#define __HIDS_H__

#include "ble_prf_utils.h"
#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>

/**
 * @defgroup HIDS_MACRO Defines
 * @{
 */

#define HIDS_CONNECTION_MAX                      10      /**< Maximum number of Heart Rate Service connections. */

#define HIDS_REPORT_MAX_SIZE                     20      /**< Maximum length of report. */
#define HIDS_REPORT_MAP_MAX_SIZE                 512     /**< Limitation of length, as per Section 2.6.1 in HIDS Spec, version 1.0 */

/**
 * @defgroup HIDS_REPORT_TYPE Report Type values
 * @{
 * @brief HIDS Report Type values define.
 */
#define HIDS_REP_TYPE_INPUT                      1        /**< The input report type. */
#define HIDS_REP_TYPE_OUTPUT                     2        /**< The output report type. */
#define HIDS_REP_TYPE_FEATURE                    3        /**< The feature report type. */
/** @} */

/**
 * @defgroup HIDS_INFOR_FLAGS Information Flags
 * @{
 * @brief HIDS Information Flags define.
 */
#define HID_INFO_FLAG_REMOTE_WAKE_MSK            0x01      /**< Bit mask of Remote Wake flag in HIDS information. */
#define HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK   0x02      /**< Bit mask of Normally Connectable flag in HIDS information. */
/** @} */
/** @} */


/**
 * @defgroup HIDS_ENUM Enumerations
 * @{
 */

/**@brief HID Service event type. */
typedef enum
{
    HIDS_EVT_INVALID,                       /**< Invalid event. */
    HIDS_EVT_IN_REP_NOTIFY_ENABLED,         /**< Input report notification enabled event. */
    HIDS_EVT_IN_REP_NOTIFY_DISABLED,        /**< Input report notification disabled event. */
    HIDS_EVT_HOST_SUSP,                     /**< Suspend command received. */
    HIDS_EVT_HOST_EXIT_SUSP,                /**< Exit suspend command received. */
    HIDS_EVT_BOOT_MODE_ENTERED,             /**< Boot mode entered */
    HIDS_EVT_REPORT_MODE_ENTERED,           /**< Report mode entered */
    HIDS_EVT_REP_CHAR_WRITE,                /**< New value has been written to a report characteristic */
} hids_evt_type_t;


/**@brief HID Service write report type. */
typedef enum
{
    HIDS_REPORT_TYPE_RESERVED,              /**< The reserved report type. */
    HIDS_REPORT_TYPE_IN1,                   /**< The input report1 type. */
    HIDS_REPORT_TYPE_IN2,                   /**< The input report2 type. */
    HIDS_REPORT_TYPE_IN3,                   /**< The input report3 type. */
    HIDS_REPORT_TYPE_OUT,                   /**< The output report type. */
    HIDS_REPORT_TYPE_FEATURE,               /**< The feature report type. */
    HIDS_REPORT_TYPE_KB_IN,                 /**< The boot keyboard input report type. */
    HIDS_REPORT_TYPE_KB_OUT,                /**< The boot keyboard output report type. */
    HIDS_REPORT_TYPE_MOUSE_IN,              /**< The boot mouse inputreport type. */
} hids_report_type_t;
/** @} */


/**
 * @defgroup HIDS_TYPEDEF Typedefs
 * @{
 */

/**@brief HID Service event. */
typedef struct
{
    hids_evt_type_t evt_type;               /**< Type of event. */
    uint8_t conn_idx;                       /**< Connect index. */
    hids_report_type_t report_type;         /**< Type of report, see @ref hids_report_type_t. */
    uint16_t           offset;              /**< Offset for the write operation. */
    uint16_t           len;                 /**< Length of the incoming data. */
    uint8_t    const * data;                /**< Incoming data, variable length */
}hids_evt_t;

/**@brief HID Information characteristic value. */
typedef struct
{
    uint16_t                      bcd_hid;          /**< 16-bit unsigned integer representing version number of base USB HID Specification implemented by HID Device */
    uint8_t                       b_country_code;   /**< Identifies which country the hardware is localized for. Most hardware is not localized and thus this value would be zero (0). */
    uint8_t                       flags;            /**< See http://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.hid_information.xml */
}hids_hid_info_t;


/**@brief Value of a Report Reference descriptor.
 *
 * @details This is mapping information that maps the parent characteristic to the Report ID(s) and
 *          Report Type(s) defined within a Report Map characteristic.
 */
typedef struct
{
    uint8_t report_id;                              /**< Non-zero value if there is more than one instance of the same Report Type */
    uint8_t report_type;                            /**< Type of Report characteristic (see @ref HIDS_REPORT_TYPE) */
} hids_report_ref_t;


/**@brief HID Service Report characteristic define. */
typedef struct
{
    uint16_t value_len;                            /**< Length of characteristic value. */
    hids_report_ref_t  ref;                       /**<  Value of a Report Reference descriptor, see @ref hids_report_ref_t. */
} hids_report_int_t;


/**@brief HID Service Report Map characteristic value. */
typedef struct
{
    uint8_t *p_map;             /**< Pointer to the report map. */
    uint16_t len;               /**< The length of report map. */
} hids_report_map_t;


/**@brief HID Service event handler type.
 *
 * @param[in] p_evt Pointer to a HID Service event variable.
 */
typedef void (*hids_evt_handler_t)(hids_evt_t *p_evt);


/**@brief HID Service initialization variable. */
typedef struct
{
    hids_evt_handler_t  evt_handler;                        /**< Handle events in HID Service. */
    bool is_kb;                                             /**< TRUE if device is operating as a keyboard, FALSE if it is not. */
    bool is_mouse;                                          /**< TRUE if device is operating as a mouse, FALSE if it is not. */
    hids_hid_info_t     hid_info;                           /**< Value of HID information characteristic. */
    hids_report_map_t   report_map;                         /**< HID Service Report Map characteristic value. */
    uint8_t             input_report_count;                 /**< Number of Input Report characteristics. */
    hids_report_int_t   input_report_array[3];              /**< HID input Report Reference value. */
    bool                out_report_sup;                     /**< TRUE if output Report characteristic suport, FALSE if it is nonsupport. */
    hids_report_int_t   output_report;                      /**< HID output Report Reference value. */
    bool                feature_report_sup;                 /**< TRUE if feature Report characteristic suport, FALSE if it is nonsupport. */
    hids_report_int_t   feature_report;                     /**< HID feature Report Reference value. */
} hids_init_t;
/** @} */

/**
 * @defgroup HIDS_FUNCTION Functions
 * @{
 */

/**
 *****************************************************************************************
 * @brief Initialize a HID Service instance in ATT DB.
 *
 * @param[in] p_hids_init: Pointer to a HID Service initialization variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t hids_service_init(hids_init_t *p_hids_init);

/**
 *****************************************************************************************
 * @brief Send an input report.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] rep_idx: Input report inedx.
 * @param[in] p_data: Pointer to data to be sent.
 * @param[in] length: Length of data to be sent.
 *
 * @return BLE_SDK_SUCCESS on success, otherwise an error code.
 *****************************************************************************************
 */
sdk_err_t hids_input_rep_send(uint8_t conn_idx, uint8_t rep_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Send boot keyboard input report.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data: Pointer to data to be sent.
 * @param[in] length: Length of data to be sent.
 *
 * @return BLE_SDK_SUCCESS on success, otherwise an error code.
 *****************************************************************************************
 */
sdk_err_t hids_boot_kb_in_rep_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Send boot mouse input report.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data: Pointer to data to be sent.
 * @param[in] length: Length of data to be sent.
 *
 * @return BLE_SDK_SUCCESS on success, otherwise an error code.
 *****************************************************************************************
 */
sdk_err_t hids_boot_mouse_in_rep_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Provide the interface for other modules to obtain the hids service start handle .
 *
 * @return The hids service start handle.
 *****************************************************************************************
 */
uint16_t hids_service_start_handle_get(void);

/** @} */

#endif
/** @} */
/** @} */


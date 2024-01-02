/**
 *****************************************************************************************
 *
 * @file ble_scanner.h
 *
 * @brief BLE Scanner Module API
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


#ifndef __BLE_SCANNER__
#define __BLE_SCANNER__

#include "ble_module_config.h"
#include "ble_event.h"
#include "ble_gapm.h"

/**
 * @defgroup BLE_SCANNER_MAROC Defines
 * @{
 */
/**@brief Capture scanner event. */
#define BLE_SCANNER_EVT_CAPTURE(err, evt_id, arg)    \
do                                                   \
{                                                    \
    if (err != BLE_SUCCESS)                          \
    {                                                \
        ble_scanner_err_on_ble_capture(err);         \
    }                                                \
    else                                             \
    {                                                \
        ble_scanner_evt_on_ble_capture(evt_id, arg); \
    }                                                \
} while(0)
/** @} */

#define AD_TYPE_NUM_MAX                     10             /**< Maximum number of ad type can be parsed in one advertising report packet. */
#define UUID_16_BIT_NUM_MAX                 14             /**< Maximum number of 16 bit uuid service can be parsed in one advertising report packet. */
#define UUID_32_BIT_NUM_MAX                 7              /**< Maximum number of 32 bit uuid service can be parsed in one advertising report packet. */
#define UUID_128_BIT_NUM_MAX                1              /**< Maximum number of 128 bit uuid service can be parsed in one advertising report packet. */

#define UUID_16_BIT_BYTES                   2              /**< Length of bytes for 16 bit UUID. */
#define UUID_32_BIT_BYTES                   4              /**< Length of bytes for 32 bit UUID. */
#define UUID_128_BIT_BYTES                  16             /**< Length of bytes for 128 bit UUID. */
/** @} */

/**
 * @defgroup BLE_SCANNER_ENUM Enumerations
 * @{
 */
/**@brief BLE GAP event ids. */
enum
{
    BLE_GAP_EVT_ID_ADV_START   = BLE_COMMON_EVT_BASE,
    BLE_GAP_EVT_ID_ADV_STOP,
    BLE_GAP_EVT_ID_ADV_REPORT,
    BLE_GAP_EVT_ID_CONNECTED,
    BLE_GAP_EVT_ID_DISCONNECTED,
    BLE_GAP_EVT_ID_CONNECTED_CANCLE,
    BLE_GAP_EVT_ID_CONN_PARAM_UPDATED,
    BLE_GAP_EVT_ID_PHY_UPDATED,
    BLE_GAP_EVT_ID_DEV_INFO_GOT,
    BLE_GAP_EVT_ID_SCAN_START,
    BLE_GAP_EVT_ID_SCAN_STOP,
    BLE_GAP_EVT_ID_SCAN_REQUEST,
    BLE_GAP_EVT_ID_SYNC_ESTABLISH,
    BLE_GAP_EVT_ID_PEER_NAME_GOT,
    BLE_GAP_EVT_ID_CONN_PARAM_UPDATE_REQUEST,
    BLE_GAP_EVT_ID_CONN_INFO_GOT,
    BLE_GAP_EVT_ID_PEER_DEV_INFO_GOT,
    BLE_GAP_EVT_ID_DATA_LENGTH_UPDATED,
    BLE_GAP_EVT_ID_READ_RSLV_ADDR
};

/**@brief BLE Scanner filter match mode. */
typedef enum
{
    BLE_SCANNER_FILTER_ALL_MATCH,                /**< Only when all type match. */
    BLE_SCANNER_FILTER_ANYONE_MATCH,             /**< Just match any type . */
} ble_scanner_filter_mode_t;

/**@brief BLE Scanner filter types. */
typedef enum
{
    BLE_SCANNER_NAME_FILTER       = (0x01 << 0),          /**< Filter device base on target device name. */
    BLE_SCANNER_APPEARANCE_FILTER = (0x01 << 1),          /**< Filter device base on target appearance. */
    BLE_SCANNER_UUID_FILTER       = (0x01 << 2),          /**< Filter device base on tagert service UUID. */
    BLE_SCANNER_ADDR_FILTER       = (0x01 << 3),          /**< Filter device base on target address. */
} ble_scanner_filter_type_t;

/**@brief BLE Scanner event type. */
typedef enum
{
    BLE_SCANNER_EVT_INVALID,                /**< Invalid event. */
    BLE_SCANNER_EVT_WHITELIST_REQUEST,      /**< Request user to add whitelist. */
    BLE_SCANNER_EVT_TIMEOUT,                /**< Scan tiemout. */
    BLE_SCANNER_EVT_ADV_REPORT_PARSE,       /**< Advertising report parsed. */
    BLE_SCANNER_EVT_FILTER_MATCH,           /**< Filter match from adv report. */
    BLE_SCANNER_EVT_FILTER_NO_MATCH,        /**< Filter no match from adv report. */
    BLE_SCANNER_EVT_CONNECTED,              /**< Connected. */
} ble_scanner_evt_type_t;
/** @} */

/**
 * @defgroup BLE_SCANNER_STRUCT Structures
 * @{
 */
/**@brief Data structure. */
typedef struct
{
    const uint8_t *p_data;  /**< Pointer to the data. */
    uint16_t       length;  /**< Length of the data. */
} ble_data_t;

/**@brief Target data provided to filter. */
typedef struct
{
    ble_data_t              dev_name;           /**< Target device name. */
    uint16_t                appearance;         /**< Target appearance. */
    ble_data_t              svr_uuid;           /**< Target service UUID. */
    ble_gap_bdaddr_t            target_addr;        /**< Target device address. */
} ble_scanner_filter_data_t;

/**@brief UUID data parsed from advertising report. */
typedef struct
{
    uint8_t  uuid_16_bit_count;                                           /**< Count of 16 bit uuid parsed. */
    uint8_t  uuid_32_bit_count;                                           /**< Count of 32 bit uuid parsed. */
    uint8_t  uuid_128_bit_count;                                          /**< Count of 128 bit uuid parsed. */
    uint16_t uuid_16_bit[UUID_16_BIT_NUM_MAX];                            /**< All 16 bit uuid data parsed. */
    uint32_t uuid_32_bit[UUID_32_BIT_NUM_MAX];                            /**< All 32 bit uuid data parsed. */
    uint8_t  uuid_128_bit[UUID_128_BIT_NUM_MAX][UUID_128_BIT_BYTES];      /**< All 128 bit uuid data parsed. */
} ble_scanner_uuid_t;

/**@brief Parsed adv types */
typedef struct
{
    bool  flag;                /**< True if flag existed, false or not. */
    bool  appearance;          /**< True if appearance existed, false or not. */
    bool  local_name;          /**< True if service UUID existed, false or not. */
    bool  manufacture_data;    /**< True if manufacture data existed, false or not. */
    bool  uuid;                /**< True if UUID existed, false or not. */
} ble_scanner_adv_type_t;

/**@brief Single advertising type parsed from advertising report. */
typedef struct
{
    uint8_t     ad_type;        /**< GAP advertising data type. */
    ble_data_t  type_data;      /**< Adv type raw data. */
} ble_scanner_single_rec_t;

/**@brief All GAP advertising type parsed from advertising report. */
typedef struct
{
    uint8_t                  type_count;                   /**< Count of advertising type parsed. */
    ble_scanner_single_rec_t single_rec[AD_TYPE_NUM_MAX];  /**< Information of parsed. */
} ble_scanner_parse_rec_t;

/**@brief All parsed result from advertising report. */
typedef struct
{
    ble_gap_adv_report_type_t  adv_report_type;   /**< Advertising report type. */
    ble_scanner_adv_type_t     adv_type_parsed;   /**< Parsed adv types. */
    ble_gap_bdaddr_t           peer_addr;         /**< Address of device from which advertising. */
    int8_t                     rssi;              /**< RSSI (between -127 and +20 dBm). */
    uint8_t                    flag;              /**< Flag, shall not appear more than once in a block. */
    uint16_t                   appearance;        /**< Appearance, shall not appear more than once in a block. */
    ble_data_t                 local_name;        /**< Local name, shall not appear more than once in a block. */
    ble_data_t                 manufacture_data;  /**< Manufacturer specific data. */
    ble_scanner_uuid_t         uuid_list;         /**< Service uuid list. */
    ble_scanner_parse_rec_t    other_parse_rec;   /**< Other data parse record. */
} ble_scanner_parse_report_t;

/**@brief Matched with which target data. */
typedef struct
{
    bool  dev_name_match;            /**< True if device name matched, false or not. */
    bool  appearance_match;          /**< True if appearance matched, false or not. */
    bool  uuid_match;                /**< True if service UUID matched, false or not. */
    bool  addr_match;                /**< True if address matched, false or not. */
} ble_scanner_filter_match_t;

/**@brief BLE Scanner Module event. */
typedef struct
{ 
    ble_scanner_evt_type_t            evt_type;          /**< BLE Scanner event type. */
    union
    {
        ble_scanner_filter_match_t    match_result;      /**< Filter match result. */
        ble_scanner_parse_report_t    parse_record;      /**< Advertising report parse result record. */
        uint8_t                       conn_idx;          /**< Connect index. */
    } param;
} ble_scanner_evt_t;
/** @} */

/**
 * @defgroup BLE_SCANNER_TYPEDEF Typedefs
 * @{
 */
/**@brief BLE scanner event handler type. */
typedef void (*ble_scanner_evt_handler_t)(ble_scanner_evt_t *p_evt);

/**@brief BLE scanner error handler type. */
typedef void (*ble_scanner_err_handler_t)(uint8_t err_code);
/** @} */

/**
 * @defgroup BLE_SCANNER_STRUCT Structures
 * @{
 */
typedef struct
{
    bool                      connect_auto;      /**< Ture: Connect to target device when found, False: Notify to application when device when found. */
    ble_gap_scan_param_t      scan_param;        /**< Scan parameters. */
    ble_gap_init_param_t      conn_param;        /**< Connect parameters which must be initialized when @see connect_auto to be set Ture. */
    ble_scanner_evt_handler_t evt_handler;       /**< Handler to handle scan events for application. */
    ble_scanner_err_handler_t err_handler;       /**< Handler to handle scan errors for application. */
} ble_scanner_init_t;
/** @} */

/**
 * @defgroup BLE_SCANNER_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief BLE Scanner Module initialization.
 *
 * @param[in]  p_scanner_init: Pointer to BLE Scanner Module initialization parameters
 *
 * @return Result of init operation.
 *****************************************************************************************
 */
sdk_err_t ble_scanner_init(ble_scanner_init_t *p_scanner_init);

/**
 *****************************************************************************************
 * @brief BLE Scanner Module filter set(can override set).
 *
 * @param[in] filter_type:   Filter type.
 * @param[in] p_filter_data: Pointer to data for filtering.
 *
 * @return Result of set operation.
 *****************************************************************************************
 */
sdk_err_t ble_scanner_filter_set(uint8_t filter_type, ble_scanner_filter_data_t *p_filter_data);

/**
 *****************************************************************************************
 * @brief BLE Scanner Module filter diasble.
 *****************************************************************************************
 */
void ble_scanner_filter_disable(void);

/**
 *****************************************************************************************
 * @brief BLE Scan Module filter enable.
 *
 * @param[in] filter_mode:   Filter mode.
 *****************************************************************************************
 */
void ble_scanner_filter_enable(ble_scanner_filter_mode_t filter_mode);

/**
 *****************************************************************************************
 * @brief BLE Scanner Module starts scanning device.
 *
 * @return Result of start scanning.
 *****************************************************************************************
 */
sdk_err_t ble_scanner_start(void);

/**
 *****************************************************************************************
 * @brief BLE Scanner Module stop scanning device.
 *
 * @return Result of stop scanning.
 *****************************************************************************************
 */
sdk_err_t ble_scanner_stop(void);

/**
 *****************************************************************************************
 * @brief Capture scanner events on BLE.
 *
 * @param[in] evt_id:   Event ID on BLE.
 * @param[in] arg:      Arguments.
 *****************************************************************************************
 */
void ble_scanner_evt_on_ble_capture(const ble_evt_t *p_evt);

/**
 *****************************************************************************************
 * @brief Capture scanner error.
 *
 * @param[in] err_code: Error code.
 *****************************************************************************************
 */
void ble_scanner_err_on_ble_capture(uint8_t err_code);
/** @} */
#endif



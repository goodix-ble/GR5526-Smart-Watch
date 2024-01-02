/**
 ****************************************************************************************
 *
 * @file cts.h
 *
 * @brief Current Time Service API.
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
 * @defgroup BLE_SDK_CTS Current Time Service (CTS)
 * @{
 * @brief Current Time Service module.
 *
 * @details The Current Time Service exposes Current Time characteristic. It optionally exposes
 *          Local Time Information characteristic and Reference Time Information characteristic.
 *
 *          After \ref cts_init_t variable is intialized, the application must call \ref cts_service_init()
 *          to add Current Time Service and Current Time, Local Time Information and Reference Time
 *          Information characteristics to the BLE Stack database according to \ref cts_init_t.char_mask.
 */

#ifndef __CTS_H__
#define __CTS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include "ble_prf_utils.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup CTS_MACRO Defines
 * @{
 */
#define CTS_CONNECTION_MAX                10                           /**< Maximum number of CTS connections. */
#define CTS_CUR_TIME_VAL_LEN              10                           /**< Length of current time value. */
#define TOTAL_CTS_CUR_TIME_VAL_LEN        27                           /**< TOTAL length of current time value. */
#define CTS_LOC_TIME_INFO_VAL_LEN         2                            /**< Length of local time information value. */
#define TOTAL_CTS_LOC_TIME_INFO_VAL_LEN   6                            /**< TOTAL length local time information value. */
#define CTS_REF_TIME_INFO_VAL_LEN         4                            /**< Length of reference time information value. */
#define CTS_TIME_YEAR_VALID_VAL_MIN       1582                         /**< Minimum value of valid year. */
#define CTS_TIME_YEAR_VALID_VAL_MAX       9999                         /**< Maximum value of valid year. */
#define CTS_TIME_ZONE_OFFSET_MIN          -48                           /**< Minimum Value of Offset from UTC. */
#define CTS_TIME_ZONE_OFFSET_MAX          56                           /**< Maximum Value of Offset from UTC. */
#define CTS_TIME_ACCURACY_OUT_RANGE       254                          /**< Accuracy out of range. */
#define CTS_TIME_ACCURACT_UNKNOWN         255                          /**< Accuracy Unknown. */
#define CTS_ERROR_FIELDS_IGNORED          0x80                         /**< The server ignored one or more fields. */

/**
 * @defgroup CTS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref cts_init_t.char_mask.
 */
#define CTS_CHAR_MANDATORY              0x0f     /**< Bit mask for mandatory characteristic in CTS. */
#define CTS_CHAR_LOC_TIME_INFO_SUP      0x30     /**< Bit mask for Local Time Information characteristic that is optional. */
#define CTS_CHAR_REF_TIME_INFO_SUP      0xc0     /**< Bit mask for Reference Time Information characteristic that is optional. */
#define CTS_CHAR_FULL                   0xff     /**< Bit mask of the full characteristic. */
/** @} */

/**
 * @defgroup CTS_ADJ_REASON Current Time Adjust Reason
 * @{
 * @brief Current Time Service Adjust Reason.
 */
#define CTS_AR_NO_CHANGE              (0x00 << 0)           /**< No change for current time. */
#define CTS_AR_MAUAL_TIME_UPDATE      (0x01 << 0)           /**< Manual time update. */
#define CTS_AR_EXT_REF_TIME_UPDATE    (0x01 << 1)           /**< External reference time update. */
#define CTS_AR_TIME_ZONE_CHANGE       (0x01 << 2)           /**< Change of time zone. */
#define CTS_AR_DST_CHANGE             (0x01 << 3)           /**< Change of DST (daylight savings time). */

/** @} */
/** @} */

/**
 * @defgroup CTS_ENUM Enumerations
 * @{
 */
/**@brief Current Time Day of week. */
typedef enum
{
    CTS_WEEK_UNKNOWN_DAY,             /**< Day of week is not known. */
    CTS_WEEK_MONDAY,                  /**< Monday. */
    CTS_WEEK_TUSEDAY,                 /**< Tuesday. */
    CTS_WEEK_WEDNESDAY,               /**< Wednesday. */
    CTS_WEEK_THURSDAT,                /**< Thursday. */
    CTS_WEEK_FRIDAY,                  /**< Friday. */
    CTS_WEEK_SATURDAY,                /**< Saturday. */
    CTS_WEEK_SUNDAY                   /**< Sunday. */
} cts_week_day_t;

/**@brief Local time information:Daylight Saving Time Offset. */
typedef enum
{
    CTS_DST_OFFSET_STANDAR_TIME       = 0x00,   /**< Standard Time. */
    CTS_DST_OFFSET_HALF_HOUR          = 0x02,   /**< Half An Hour Daylight Time (+0.5h). */
    CTS_DST_OFFSET_DAYLIGHT_TIME      = 0x04,   /**< Daylight Time (+1h). */
    CTS_DST_OFFSET_DOUB_DAYLIGHT_TIME = 0x08,   /**< Double Daylight Time (+2h). */
    CTS_DST_OFFSET_DOUB_UNKNOWED_TIME = 0xff,   /**< Unknown time. */
} cts_dst_offset_t;

/**@brief Reference time information:Time Source. */
typedef enum
{
    CTS_REF_TIME_SRC_UNKNOWN,               /**< Unknown. */
    CTS_REF_TIME_SRC_NET_TIME_PROTOCOL,     /**< Network Time Protocol. */
    CTS_REF_TIME_SRC_GPS,                   /**< GPS. */
    CTS_REF_TIME_SRC_RADIO_TIME_SIGNAL,     /**< Radio Time Signal. */
    CTS_REF_TIME_SRC_MANUAL,                /**< Manual. */
    CTS_REF_TIME_SRC_ATOMIC_CLOCK,          /**< Atomic Clock. */
    CTS_REF_TIME_SRC_CELLUAR_NET,           /**< Cellular Network. */
} cts_ref_time_source_t;

/**@brief Current Time Service event type. */
typedef enum
{
    CTS_EVT_INVALID = 0x00,                  /**< Invalid event. */
    CTS_EVT_CUR_TIME_NOTIFICATION_ENABLED,   /**< Current Time Notification is enabled. */
    CTS_EVT_CUR_TIME_NOTIFICATION_DISABLED,  /**< Current Time Notification is disabled. */
    CTS_EVT_CUR_TIME_SET_BY_PEER,            /**< Current Time has been set by peer. */
    CTS_EVT_LOC_TIME_INFO_SET_BY_PEER,       /**< Local Time information has been set by peer. */
} cts_evt_type_t;
/** @} */

/**
 * @defgroup CTS_STRUCT Structures
 * @{
 */
/**@brief CTS Exact Time 256. */
typedef struct
{
    prf_date_time_t date_time;       /**< Date Time. */
    uint8_t         day_of_week;     /**< Day of Week. */
    uint8_t         fractions_256;   /**< 1/256th of a second. */
} cts_exact_time_256_t;

/**@brief CTS Current Time value. */
typedef struct
{
    cts_exact_time_256_t day_date_time;   /**< Exact Time 256. */
    uint8_t              adjust_reason;   /**< Adjust Reason. */
} cts_cur_time_t;

/**@brief CTS Local Time Information. */
typedef struct
{
    int16_t           time_zone;     /**< Time Zone, Offset from UTC in number of 15-minute increments. */
    cts_dst_offset_t dst_offset;    /**< Daylight Saving Time Offset. */
} cts_loc_time_info_t;

/**@brief CTS Reference Time Information. */
typedef struct
{
    cts_ref_time_source_t   source;             /**< Time Source. */
    uint8_t                 accuracy;           /**< Accuracy of time information. */
    uint8_t                 days_since_update;  /**< Days Since Update. */
    uint8_t                 hours_since_update; /**< Hours  Since Update. */
} cts_ref_time_info_t;

/**@brief CTS Reference Time Updata Information. */
typedef struct
{
    uint8_t before_updata_sec ;                     /**< The second before the update. */
    uint8_t current_time_min;                       /**< Minutes of the current time. */
    uint8_t current_time_sec;                       /**< The second of the current time. */
    uint8_t expected_time_1min_adapt;               /**< Minutes of the expected update time. */
    cts_ref_time_source_t stage_update_source;      /**< Minutes of the expected update time. */
    uint8_t updata_time_flag;                       /**< 1 minute time flag bit. */
} cts_updata_ref_time_info_t;

/**@brief CTS Adjust information. */
typedef struct
{
    uint8_t              adjust_reason;   /**< Adjust Reason. */
    cts_exact_time_256_t day_date_time;   /**< Exact Time 256. */
    cts_loc_time_info_t  loc_time_info;    /**< Local Time information. */
    cts_ref_time_info_t  ref_time_info;    /**< Reference Time information. */
} cts_adj_info_t;

/**@brief Current Time Service event. */
typedef struct
{
    uint8_t             conn_idx;        /**< The index of the connection. */
    cts_evt_type_t      evt_type;        /**< The CTS event type. */
    const uint8_t      *p_data;          /**< Pointer to event data. */
    uint16_t            length;          /**< Length of event data. */
    cts_cur_time_t      cur_time;        /**< Curren time set by peer. */
    cts_loc_time_info_t loc_time_info;   /**< Local time information set by peer. */
} cts_evt_t;
/** @} */

/**
 * @defgroup CTS_TYPEDEF Typedefs
 * @{
 */
/**@brief Current Time Service event handler type. */
typedef void (*cts_evt_handler_t)(cts_evt_t *p_evt);
/** @} */

/**
 * @defgroup CTS_STRUCT Structures
 * @{
 */
/**@brief Current Time Service init structure. This contains all option and data needed for initialization of the service. */
typedef struct
{
    cts_evt_handler_t   evt_handler;        /**< Current Time Service event handler. */
    uint16_t            char_mask;          /**< Initial mask of supported characteristics, and configured with \ref CTS_CHAR_MASK. */
    cts_cur_time_t      cur_time;           /**< Current Time. */
    cts_loc_time_info_t loc_time_info;      /**< Local Time information. */
    cts_ref_time_info_t ref_time_info;      /**< Reference Time information. */
} cts_init_t;
/** @} */

/**
 * @defgroup CTS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Current Time Service instance and add in the DB.
 *
 * @param[in] p_cts_init: Pointer to CTS Service initialization variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t cts_service_init(cts_init_t *p_cts_init);

/**
 *****************************************************************************************
 * @brief Get exact time for user.
 *
 * @param[out] p_exact_time: Pointer to exact time.
 *****************************************************************************************
 */
void cts_exact_time_get(cts_init_t *p_exact_time);

/**
 *****************************************************************************************
 * @brief Update exact time.
 *
 * @param[in] p_cts_exact_time: Pointer to exact time.
 *****************************************************************************************
 */
void cts_exact_time_update(cts_init_t *p_cts_exact_time) ;

/**
 *****************************************************************************************
 * @brief Adjust current time.
 *
 * @param[in] p_adj_info:  Pointer to adjust information.
 *****************************************************************************************
 */
void cts_cur_time_adjust(cts_adj_info_t *p_adj_info);

/**
 *****************************************************************************************
 * @brief Send Current Time if its notification has been enabled.
 *
 * @param[in] conn_idx:   Connnection index.
 * @param[in] p_cur_time: Pointer to current time.
 *
 * @return Result of notify value
 *****************************************************************************************
 */
sdk_err_t cts_cur_time_send(uint8_t conn_idx, cts_cur_time_t *p_cur_time);
/** @} */

/**
 *****************************************************************************************
 * @brief Data accepts data and processing functions.
 *
 * @param[in] p_data:   Serial port data.
 * @param[in] length:   Data length.
 *****************************************************************************************
 */
void cts_c_data_parse(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Serial port data is converted into reference time.
 *
 * @param[in] p_data:   Serial port data.
 * @param[in] length:   Data length.
 *****************************************************************************************
 */
void reference_time_encode(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Serial port data is converted into local time.
 *
 * @param[in] p_data:   Serial port data.
 * @param[in] length:   Data length.
 *****************************************************************************************
 */
void local_time_encode(uint8_t *p_data, uint8_t length);

/**
 *****************************************************************************************
 * @brief Serial port data is converted into current time.
 *
 * @param[in] p_data:   Serial port data.
 * @param[in] length:   Data length.
 *****************************************************************************************
 */
void current_time_encode(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Handle Local Time Information conversion.
 *
 * @param[in] p_cfm: Pointer to GATT write attribute result description.
 * @param[in]  p_evt: Pointer to CTS event.
 *
 * @return Result of data lenth.
 *****************************************************************************************
 */
uint8_t local_time_universal_decode(ble_gatts_write_cfm_t *p_cfm, cts_evt_t *p_evt) ;

/**
 *****************************************************************************************
 * @brief Decode for a Current Time.
 *
 * @param[in] p_cfm: Pointer to GATT write attribute result description.
 * @param[in]  p_evt: Pointer to CTS event.
 *
 * @return Result of data lenth.
 *****************************************************************************************
 */
uint8_t current_time_universal_decode(ble_gatts_write_cfm_t *p_cfm, cts_evt_t *p_evt);

#endif
/** @} */
/** @} */


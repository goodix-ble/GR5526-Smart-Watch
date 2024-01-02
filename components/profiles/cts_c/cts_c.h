/**
 *****************************************************************************************
 *
 * @file cts_c.h
 *
 * @brief Current Time Service Client API
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
 */

/**
 * @defgroup BLE_SDK_CTS_C Current Time Service Client (CTS_C)
 * @{
 * @brief Current Time Service Client module.
 *
 * @details The Current Time Service Client contains the APIs and types, which can be used
 *          by the application to discover Current Time Service of peer and interact with it.
 *
 *          The application must provide an event handler to be register, then call \ref cts_client_init().
 *          After Current Time Service Client discovers peer Current Time Service, application can
 *          receive current time value notification from peer, and also call \ref cts_c_cur_time_read()
 *          to get current time from peer. This module provides functions: \ref cts_c_loc_time_info_read()
 *          and \ref cts_c_ref_time_info_read() to read local time information and reference time informarion
 *          on peer.
 */

#ifndef __CTS_C_H__
#define __CTS_C_H__

#include "gr_includes.h"
#include "custom_config.h"
#include "ble_prf_utils.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup CTS_C_MACRO Defines
 * @{
 */
#define CTS_C_CONNECTION_MAX             10                                 /**< Maximum number of HRS Client connections. */
#define CTS_C_CUR_TIME_VAL_LEN           10                                 /**< Length of current time value. */
#define CTS_C_LOC_TIME_INFO_VAL_LEN      2                                  /**< Length of local time information value. */
#define CTS_C_TIME_Y_M_D_UNKNOWN         0                                  /**< Year or Month or Day is not known. */
#define CTS_C_TIME_YEAR_VALID_VAL_MIN    1582                               /**< Minimum value of valid year. */
#define CTS_C_TIME_YEAR_VALID_VAL_MAX    9999                               /**< Maximum value of valid year. */
#define CTS_C_TIME_ZONE_OFFSET_MIN      -48                                 /**< Minimum Value of Offset from UTC. */
#define CTS_C_TIME_ZONE_OFFSET_MAX       56                                 /**< Maximum Value of Offset from UTC. */
#define CTS_C_TIME_ACCURACY_OUT_RANGE    254                                /**< Accuracy out of range. */
#define CTS_C_TIME_ACCURACT_UNKNOWN      255                                /**< Accuracy Unknown. */
#define CTS_C_ERROR_FIELDS_IGNORED       0x80                               /**< The server ignored one or more fields. */

/**
 * @defgroup CTS_C_ADJ_REASON Current Time Adjust Reason
 * @{
 * @brief Current Time Service Adjust Reason.
 */
#define CTS_C_AR_NO_CHANGE              (0x00 << 0)           /**< No change for current time. */
#define CTS_C_AR_MAUAL_TIME_UPDATE      (0x01 << 0)           /**< Manual time update. */
#define CTS_C_AR_EXT_REF_TIME_UPDATE    (0x01 << 1)           /**< External reference time update. */
#define CTS_C_AR_TIME_ZONE_CHANGE       (0x01 << 2)           /**< Change of time zone. */
#define CTS_C_AR_DST_CHANGE             (0x01 << 3)           /**< Change of DST (daylight savings time). */
/** @} */
/** @} */

/**
 * @defgroup CTS_C_ENUM Enumerations
 * @{
 */
/**@brief Current Time Day of week. */
typedef enum
{
    CTS_C_WEEK_UNKNOWN_DAY,             /**< Day of week is not known. */
    CTS_C_WEEK_MONDAY,                  /**< Monday. */
    CTS_C_WEEK_TUSEDAY,                 /**< Tuesday. */
    CTS_C_WEEK_WEDNESDAY,               /**< Wednesday. */
    CTS_C_WEEK_THURSDAT,                /**< Thursday. */
    CTS_C_WEEK_FRIDAY,                  /**< Friday. */
    CTS_C_WEEK_SATURDAY,                /**< Saturday. */
    CTS_C_WEEK_SUNDAY                   /**< Sunday. */
} cts_c_week_day_t;

/**@brief Local time information:Daylight Saving Time Offset. */
typedef enum
{
    CTS_C_DST_OFFSET_STANDAR_TIME       = 0x00,   /**< Standard Time. */
    CTS_C_DST_OFFSET_HALF_HOUR          = 0x02,   /**< Half An Hour Daylight Time (+0.5h). */
    CTS_C_DST_OFFSET_DAYLIGHT_TIME      = 0x04,   /**< Daylight Time (+1h). */
    CTS_C_DST_OFFSET_DOUB_DAYLIGHT_TIME = 0x08,   /**< Double Daylight Time (+2h). */
} cts_c_dst_offset_t;

/**@brief Reference time information:Time Source. */
typedef enum
{
    CTS_C_REF_TIME_SRC_UNKNOWN,               /**< Unknown. */
    CTS_C_REF_TIME_SRC_NET_TIME_PROTOCOL,     /**< Network Time Protocol. */
    CTS_C_REF_TIME_SRC_GPS,                   /**< GPS. */
    CTS_C_REF_TIME_SRC_RADIO_TIME_SIGNAL,     /**< Radio Time Signal. */
    CTS_C_REF_TIME_SRC_MANUAL,                /**< Manual. */
    CTS_C_REF_TIME_SRC_ATOMIC_CLOCK,          /**< Atomic Clock. */
    CTS_C_REF_TIME_SRC_CELLUAR_NET,           /**< Cellular Network. */
} cts_c_ref_time_source_t;

/**@brief Current Time Service Client event type. */
typedef enum
{
    CTS_C_EVT_INVALID,                    /**< CTS Client invalid event. */
    CTS_C_EVT_DISCOVERY_COMPLETE,         /**< CTS Client has found CTS service and its characteristics. */
    CTS_C_EVT_DISCOVERY_FAIL,             /**< CTS Client found CTS service failed because of invalid operation or no found at the peer. */
    CTS_C_EVT_CUR_TIME_NTF_SET_SUCCESS,   /**< CTS Client has set Notification of Current Time characteristic. */
    CTS_C_EVT_VALID_CUR_TIME_REC,         /**< CTS Client has received valid Current Time value (Read or Notification from peer). */
    CTS_C_EVT_INVALID_CUR_TIME_REC,       /**< CTS Client has received invalid Current Time value (Read or Notification from peer). */
    CTS_C_EVT_VALID_LOC_TIME_INFO_REC,    /**< CTS Client has received valid Local Time Information value (Read from peer). */
    CTS_C_EVT_INVALID_LOC_TIME_INFO_REC,  /**< CTS Client has received invalid Local Time Information value (Read from peer). */
    CTS_C_EVT_VALID_REF_TIME_INFO_REC,    /**< CTS Client has received valid Reference Time Information Value (Read from peer). */
    CTS_C_EVT_INVALID_REF_TIME_INFO_REC,  /**< CTS Client has received invalid Reference Time Information Value (Read from peer). */
    CTS_C_EVT_CUR_TIME_SET_SUCCESS,       /**< CTS Client has writen Current Time completely. */
    CTS_C_EVT_LOC_TIME_INFO_SET_SUCCESS,  /**< CTS Client has writen Local Time Information completely. */
    CTS_C_EVT_WRITE_OP_ERR,               /**< Error occured when CTS Client writen to peer. */
} cts_c_evt_type_t;
/** @} */

/**
 * @defgroup CTS_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t cts_srvc_start_handle;            /**< CTS Service start handle. */
    uint16_t cts_srvc_end_handle;              /**< CTS Service end handle. */
    uint16_t cts_cur_time_handle;              /**< CTS Current Time characteristic Value handle which has been got from peer. */
    uint16_t cts_cur_time_cccd_handle;         /**< CTS CCCD handle of Current Time characteristic which has been got from peer. */
    uint16_t cts_loc_time_info_handle;         /**< CTS Local Time Information characteristic Value handle which has been got from peer. */
    uint16_t cts_ref_time_info_handle;         /**< CTS Reference Time Information characteristic Value handle which has been got from peer. */
} cts_c_handles_t;

/**@brief Exact Time 256. */
typedef struct
{
    prf_date_time_t date_time;       /**< Date Time. */
    uint8_t         day_of_week;     /**< Day of Week. */
    uint8_t         fractions_256;   /**< 1/256th of a second. */
} cts_c_exact_time_256_t;

/**@brief Current Time value. */
typedef struct
{
    cts_c_exact_time_256_t day_date_time;   /**< Exact Time 256. */
    uint8_t                adjust_reason;   /**< Adjust Reason. */
} cts_c_cur_time_t;

/**@brief Local Time Information. */
typedef struct
{
    int8_t             time_zone;     /**< Time Zone, Offset from UTC in number of 15 minutes increments. */
    cts_c_dst_offset_t dst_offset;    /**< Daylight Saving Time Offset. */
} cts_c_loc_time_info_t;

/**@brief Reference Time Information. */
typedef struct
{
    cts_c_ref_time_source_t   source;             /**< Time Source. */
    uint8_t                   accuracy;           /**< Accuracy of time information. */
    uint8_t                   days_since_update;  /**< Days Since Update. */
    uint8_t                   hours_since_update; /**< Hours  Since Update. */
} cts_c_ref_time_info_t;

/**@brief Current Time Service Client event. */
typedef struct
{
    uint8_t                   conn_idx;        /**< The index of the connection. */
    cts_c_evt_type_t          evt_type;        /**< The CTS client event type. */
    union
    {
        cts_c_cur_time_t      cur_time;        /**< Curren time received. */
        cts_c_loc_time_info_t loc_time_info;   /**< Local time information received. */
        cts_c_ref_time_info_t ref_time_info;   /**< Referen time information received. */
    } value;                                   /**< Decoded result of value received. */
} cts_c_evt_t;
/** @} */

/**
 * @defgroup CTS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief Current Time Service Client event handler type. */
typedef void (*cts_c_evt_handler_t)(cts_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup CTS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register CTS Client event handler.
 *
 * @param[in] evt_handler: Current Time Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t cts_client_init(cts_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery Current Time Service on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t cts_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer Current Time characteristic notify.
 *
 * @param[in] conn_idx:  Index of connection.
 * @param[in] is_enable: True or false.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t cts_c_cur_time_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Read Current Time characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t cts_c_cur_time_read(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Read Local Time Information characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t cts_c_loc_time_info_read(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Read Reference Time Information characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t cts_c_ref_time_info_read(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Set Current Time characteristic value.
 *
 * @param[in] conn_idx:   Index of connection.
 * @param[in] p_cur_time: Pointer to current time value set.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t cts_c_cur_time_set(uint8_t conn_idx, cts_c_cur_time_t *p_cur_time);

/**
 *****************************************************************************************
 * @brief Set Local Time Information characteristic value.
 *
 * @param[in] conn_idx:        Index of connection.
 * @param[in] p_loc_time_info: Pointer to local time information value set.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t cts_c_loc_time_info_set(uint8_t conn_idx, cts_c_loc_time_info_t *p_loc_time_info);

/**
 *****************************************************************************************
 * @brief Data accepts data and processing functions.
 *
 * @param[in] p_data:   Serial port data.
 * @param[in] length:   Data length.
 *****************************************************************************************
 */
void cts_c_data_parse(uint8_t *p_data, uint16_t length);
/** @} */

#endif
/** @} */
/** @} */


/**
 ****************************************************************************************
 *
 * @file ndcs.h
 *
 * @brief Next DST Change Service API.
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
 * @defgroup BLE_SDK_NDCS Next DST Change Service (NDCS)
 * @{
 * @brief Next DST Change Service module.
 *
 * @details The Next DST Change Service exposes the Time with DST characteristic. This module
 *          implements the Next DST Change Service with Time with DST characteristics.
 *
 *          The application must call \ref ndcs_service_init() to add Next DST Change Service 
 *          and Time with DST characteristic to the BLE Stack database.
 */

#ifndef __NDCS_H__
#define __NDCS_H__

#include "gr_includes.h"
#include "ble_prf_types.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup NDCS_MACRO Defines
 * @{
 */
#define NDCS_CONNECTION_MAX                10                             /**< Maximum number of NDCS connections. */
#define NDCS_TIME_WITH_DST_VAL_LEN         8                              /**< Length of Time with DST value. */
#define NDCS_CHAR_FULL                     0x07                           /**< Bit mask for mandatory characteristic in NDCS. */
/** @} */

/**
 * @defgroup NDCS_ENUM Enumerations
 * @{
 */
/**@brief Daylight Saving Time Offset. */
typedef enum
{
    NDCS_DST_OFFSET_STANDAR_TIME,         /**< Standard Time. */
    NDCS_DST_OFFSET_HALF_HOUR,            /**< Half An Hour Daylight Time (+0.5h). */
    NDCS_DST_OFFSET_DAYLIGHT_TIME,        /**< Daylight Time (+1h). */
    NDCS_DST_OFFSET_DOUB_DAYLIGHT_TIME    /**< Double Daylight Time (+2h). */
} ndcs_dst_offset_t;
/** @} */

/**
 * @defgroup NDCS_STRUCT Structures
 * @{
 */
/**@brief Time with DST. */
typedef struct
{
    prf_date_time_t   date_time;       /**< Date Time. */
    ndcs_dst_offset_t dst_offset;    /**< Daylight Saving Time Offset. */
} ndcs_time_dst_t;
/** @} */

/**
 * @defgroup NDCS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize an NDCS instance and add in the DB.
 *
 * @param[in] char_mask: Mask for mandatory characteristic in NDCS.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t ndcs_service_init(uint8_t char_mask);

/**
 *****************************************************************************************
 * @brief Update day time.
 *
 * @param[in] p_day_time: Pointer to day time.
 *****************************************************************************************
 */
void ndcs_day_time_update(prf_date_time_t *p_day_time);

/**
 *****************************************************************************************
 * @brief Update DST offset.
 *
 * @param[in] dst_offset: DST offset.
 *****************************************************************************************
 */
void ndcs_dst_offset_update(ndcs_dst_offset_t dst_offset);
/** @} */

#endif
/** @} */
/** @} */


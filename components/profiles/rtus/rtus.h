/**
 ****************************************************************************************
 *
 * @file rtus.h
 *
 * @brief Reference Time Update Service API.
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
 * @defgroup BLE_SDK_RTUS Reference Time Update Service (RTUS)
 * @{
 * @brief Reference Time Update Service module.
 *
 * @details The Reference Time Update Service shall expose the Time Update Control Point
 *          characteristic and the Time Update State characteristic.
 *
 *          After \ref rtus_init_t variable is intialized, the application must call \ref rtus_service_init()
 *          to add Reference Time Update Service and Time Update Control Point and Time Update State
 *          characteristics to the BLE Stack database.
 */

#ifndef __RTUS_H__
#define __RTUS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup RTUS_MACRO Defines
 * @{
 */
#define RTUS_CONNECTION_MAX               10                              /**< Maximum number of RTUS connections. */
#define RTUS_CTRL_PT_VAL_LEN               1                              /**< Length of Time Update Control Point value. */
#define RTUS_UPDATE_STATE_VAL_LEN          2                              /**< Length of Time Update State value. */
#define RTUS_CHAR_FULL                     0x1f                           /**< Bit mask for mandatory characteristic in RTUS. */
/** @} */

/**
 * @defgroup RTUS_ENUM Enumerations
 * @{
 */
/**@brief RTUS Time Update Control Point. */
typedef enum
{
    RTUS_CTRL_PT_GET_UPDATE = 0x01,        /**< Get reference update. */
    RTUS_CTRL_PT_CANCEL_UPDATE,            /**< Cancel reference update. */
} rtus_ctrl_pt_t;

/**@brief RTUS Current State. */
typedef enum
{
    RTUS_CUR_STATE_IDLE,         /**< Idle update state. */
    RTUS_CUR_STATE_PENDING,      /**< Update pending state. */
} rtus_cur_state_t;

/**@brief RTUS Time Update Result. */
typedef enum
{
    RTUS_UPDATE_RESULT_SCCESSFUL,          /**< Time update successful. */
    RTUS_UPDATE_RESULT_CANCELED,           /**< Time update canceled. */
    RTUS_UPDATE_RESULT_NO_CONN_TO_REF,     /**< No Connection To Reference. */
    RTUS_UPDATE_RESULT_REP_ERROR,          /**< Reference responded with an error. */
    RTUS_UPDATE_RESULT_TIMEOUT,            /**< Update timeout. */
    RTUS_UPDATE_RESULT_NO_ATTEMPTED,       /**< Update not attempted after reset. */
} rtus_update_result_t;

/**@brief RTUS Event type. */
typedef enum
{
    RTUS_EVT_INVALID,                /**< Invalid event. */
    RTUS_EVT_GET_UPDATE,             /**< Get reference update. */
    RTUS_EVT_CANCEL_UPDATE,          /**< Cancel reference update. */
} rtus_evt_type_t;
/** @} */

/**
 * @defgroup RTUS_STRUCT Structures
 * @{
 */
/**@brief RTUS Time Update State. */
typedef struct
{
    rtus_cur_state_t      cur_state;     /**< RTUS Current State. */
    rtus_update_result_t  update_result; /**< Time Update Result. */
} rtus_update_state_t;

/**@brief RTUS Event data. */
typedef struct
{
    uint8_t         conn_idx;        /**< The index of the connection. */
    rtus_evt_type_t evt_type;        /**< RTUS event type. */
} rtus_evt_t;
/** @} */

/**
 * @defgroup RTUS_TYPEDEF Typedefs
 * @{
 */
/**@brief Reference Time Update Service event handler type. */
typedef void (*rtus_evt_handler_t)(rtus_evt_t *p_evt);
/** @} */

/**
 * @defgroup RTUS_STRUCT Structures
 * @{
 */
/**@brief Reference Time Update Service init structure. This contains all option and data needed for initialization of the service. */
typedef struct
{
    rtus_evt_handler_t   evt_handler;        /**< Reference Time Update Service event handler. */
    uint16_t             char_mask;          /**< Initial mask of supported characteristics. */
} rtus_init_t;
/** @} */

/**
 * @defgroup RTUS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a RTUS instance and add in the DB.
 *
 * @param[in] p_rtus_init: Pointer to RTUS Service initialization variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t rtus_service_init(rtus_init_t *p_rtus_init);

/**
 *****************************************************************************************
 * @brief Set state of reference time update .
 *
 * @param[in] cur_state: Current state of the reference time update .
 *****************************************************************************************
 */
void rtus_current_state_set(rtus_cur_state_t cur_state);

/**
 *****************************************************************************************
 * @brief Set result of reference time update .
 *
 * @param[in] update_result: Resultof the reference time update .
 *****************************************************************************************
 */
void rtus_update_result_set(rtus_update_result_t update_result);
/** @} */

#endif
/** @} */
/** @} */


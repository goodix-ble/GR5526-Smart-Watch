/**
 ****************************************************************************************
 *
 * @file fault_trace.h
 *
 * @brief System Fault Trace API
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

#ifndef __FAULT_TRACE_H__
#define __FAULT_TRACE_H__

#include "custom_config.h"
#include "grx_sys.h"
#include <stdint.h>
#include <stdbool.h>

#if SYS_FAULT_TRACE_ENABLE

/**
 * @defgroup FAULT_TRACE_MAROC Defines
 * @{
 */
#define FAULT_INFO_RECORDS_MAX      0x10          /**< Maximum number of fault info records. */
/** @} */

/**
 * @defgroup FAULT_TRACE_STRUCT Structures
 * @{
 */
/**@brief System fault info database environment variable. */
typedef struct 
{
    NvdsTag_t     rec_tag[FAULT_INFO_RECORDS_MAX];   /**< System fault info record tags. */
    uint16_t      rec_num;                           /**< Number of all fault info records in database. */
    NvdsTag_t     usable_tag;                        /**< Current usable NVDS tag. */
} fault_info_db_env_t;
/** @} */

/**
 * @defgroup FAULT_TRACE_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize system fault info record database.
 *
 * @return Result of initialize.
 *****************************************************************************************
 */
sdk_err_t fault_trace_db_init(void);

/**
 *****************************************************************************************
 * @brief Add a record at the end of the database.
 *
 * @param[in] p_data: Pointer to log data.
 * @param[in] length: Length of data.
 *
 * @return Result of add.
 *****************************************************************************************
 */
sdk_err_t fault_db_record_add(const uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Get the number of records in the database.
 *
 * @return Number of records in the database.
 *****************************************************************************************
 */
uint8_t fault_db_records_num_get(void);

/**
 *****************************************************************************************
 * @brief Get the length of all records info in the database.
 *
 * @return Length of all records info in the database.
 *****************************************************************************************
 */
uint32_t fault_db_records_total_len_get(void);

/**
 *****************************************************************************************
 * @brief Dump records from the database.
 *
 * @param[in,out] p_buffer: Pointer to buffer.
 * @param[in,out] p_length: Pointer to buffer length
 *****************************************************************************************
 */
sdk_err_t fault_db_records_dump(uint8_t *p_buffer, uint32_t *p_length);

/**
 *****************************************************************************************
 * @brief Clear database.
 *****************************************************************************************
 */
sdk_err_t fault_db_record_clear(void);

#if defined ( __CC_ARM )
/**
 *****************************************************************************************
 * @brief Hardfault trace handler.
 *
 * @param[in] sp: Stack Pointer.
 *****************************************************************************************
 */
void hardfault_trace_handler(unsigned int sp);

#elif defined ( __GNUC__ )

/**
 *****************************************************************************************
 * @brief Hardfault trace handler.
 *
 * @param[in] args: Hardfault args.
 *****************************************************************************************
 */
void hardfault_trace_handler(unsigned int *args);
#endif
/** @} */
#endif
#endif



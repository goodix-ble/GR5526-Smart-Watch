/**
 ****************************************************************************************
 *
 * @file app_log_store.h
 *
 * @brief App Log Store API
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

#ifndef __APP_LOG_STORE_H__
#define __APP_LOG_STORE_H__

#include "custom_config.h"
#if APP_LOG_STORE_ENABLE
#include "grx_sys.h"
#include "ring_buffer.h"
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup APP_LOG_STORE_MAROC Defines
 * @{
 */
#define APP_LOG_STORE_RUN_ON_OS  0          /**< Is run on OS. */
#define APP_LOG_STORE_LINE_SIZE  280        /**< Size for every line's log. */
#define APP_LOG_STORE_CACHE_NUM  10         /**< Number of log lines cache. */
/** @} */

/**
 * @defgroup APP_LOG_STORE_TYPEDEF Typedefs
 * @{
 */
/**@brief APP LOG Store flash data dump callback type. */
typedef void (*app_log_store_dump_process_cb_t)(uint8_t *p_data, uint16_t len);
typedef void (*app_log_store_dump_start_cb_t)(uint32_t len);
typedef void (*app_log_store_dump_finish_cb_t)(void);
typedef void (*app_log_sem_process)(void);
/** @} */

/**
 * @defgroup APP_LOG_STORE_STRUCT Structures
 * @{
 */
 /**@brief The date and time structure. The packed size is 7 bytes. */
typedef struct
{
    uint8_t  year;              /**< Year time element. */
    uint8_t  month;             /**< Month time element. */
    uint8_t  day;               /**< Day time element. */
    uint8_t  hour;              /**< Hour time element. */
    uint8_t  min;               /**< Minute time element. */
    uint8_t  sec;               /**< Second time element. */
    uint16_t msec;              /**< Millisecond time element. */
} app_log_store_time_t;

/**@brief App log store operation functions. */
typedef struct
{
    bool     (*flash_init)(void);                                                          /**< Flash init. */
    bool     (*flash_erase)(const uint32_t addr, const uint32_t size);                     /**< Flash erase. */
    uint32_t (*flash_read)(const uint32_t addr, uint8_t *buf, const uint32_t size);        /**< Flash read. */
    uint32_t (*flash_write)(const uint32_t addr, const uint8_t *buf, const uint32_t size); /**< Flash write. */
    void     (*time_get)(app_log_store_time_t *p_time);                                    /**< Get real time. */
    app_log_sem_process sem_give;
    app_log_sem_process sem_take;
} app_log_store_op_t;

/**@brief App log store init stucture. */
typedef struct
{
    uint16_t   nv_tag;        /**< NVDS Tag for app log store env. */
    uint32_t   db_addr;       /**< Start address of app log db flash. */
    uint32_t   db_size;       /**< Size of app log db flash. */
    uint16_t   blk_size;      /**< Block size in the flash for erase minimum granularity */
} app_log_store_info_t;
/** @} */

/**@brief App log store dump stucture. */
typedef struct
{
    app_log_store_dump_process_cb_t dump_process_cb;   /**< App log store dump callback. */
    app_log_store_dump_start_cb_t   dump_start_cb;     /**< App log store dump start callback. */
    app_log_store_dump_finish_cb_t  dump_finish_cb;    /**< App log store dump finish callback. */
} app_log_dump_cbs_t;
/** @} */

/**
 * @defgroup APP_LOG_STORE_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize app log store module.
 *
 * @param[in] p_info:    Pointer to log store information.
 * @param[in] p_op_func: Pointer to log store opteration functions.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
uint16_t app_log_store_init(app_log_store_info_t *p_info, app_log_store_op_t *p_op_func);

/**
 *****************************************************************************************
 * @brief Save app log data to flash.
 *
 * @param[in] p_data: Pointer to app log data.
 * @param[in] length: Length of app log data.
 *
 * @return Result of save.
 *****************************************************************************************
 */
uint16_t app_log_store_save(const uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Dump app log data from flash.
 *
 * @param[in] dump_cb: Dump callback.
 *
 * @return Result of dump.
 *****************************************************************************************
 */
uint16_t app_log_store_dump(app_log_dump_cbs_t *p_dump_cbs);


/**
 *****************************************************************************************
 * @brief App log store clear.
 *****************************************************************************************
 */
uint16_t app_log_store_clear(void);

/**
 *****************************************************************************************
 * @brief App log store dump is ongoing or not.
 *****************************************************************************************
 */
bool app_log_store_dump_ongoing(void);

/**
 *****************************************************************************************
 * @brief Continue log dump.
 *****************************************************************************************
 */
void app_log_dump_continue(void);
/**
 *****************************************************************************************
 * @brief App log store schedule for save and dump.
 *****************************************************************************************
 */
void app_log_store_schedule(void);

/** @} */
#endif
#endif



/**
 *****************************************************************************************
 *
 * @file app_log_store.c
 *
 * @brief App Log tore Implementation.
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


/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_log_store.h"
#if APP_LOG_STORE_ENABLE
#include "utility.h"

/*
 * DEFINE
 *****************************************************************************************
 */
#define APP_LOG_STORE_MAGIC              0x47444442   /**< Magic for app log store: "GDDB". */
#define APP_LOG_STORE_TIME_SIZE          26           /**< [00000000.000] */
#define APP_LOG_STORE_TIME_DEFAULT       "[1970/01/01 00:00:00:000] "
#define APP_LOG_STORE_CACHE_SIZE         ((APP_LOG_STORE_LINE_SIZE) * (APP_LOG_STORE_CACHE_NUM))
#define APP_LOG_STORE_ONECE_OP_SIZE      1024
#define APP_LOG_STORE_CLEAR_BIT          (0x01 << 0)
#define APP_LOG_STORE_SAVE_BIT           (0x01 << 1)
#define APP_LOG_STORE_DUMP_BIT           (0x01 << 2)
#define APP_LOG_STORE_DUMP_READY_BIT     (0x01 << 3)
#define APP_LOG_STORE_DUMP_START_BIT     (0x01 << 4)

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief App log store head info. */
typedef struct
{
    uint32_t magic;         /**< Magic for app log store. */
    uint32_t db_addr;       /**< Start address of app log db flash. */
    uint32_t db_size;       /**< Size of app log db flash. */
    uint32_t offset;        /**< Offset of end log data. */
    uint16_t flip_over;     /**< Is flip over. */
    uint16_t check_sum;     /**< Check sum for head info. */
} log_store_head_t;

/**@brief App log store environment variable. */
struct log_store_env_t
{
    bool              initialized;
    uint8_t           store_status;
    log_store_head_t  store_head;
    uint16_t          head_nv_tag;
    uint16_t          blk_size;
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct log_store_env_t  s_log_store_env;
static app_log_store_op_t      s_log_store_ops;
static app_log_dump_cbs_t     *s_log_dump_cbs;
static uint32_t                s_log_store_dump_offset;
static ring_buffer_t           s_log_store_rbuf __attribute__((section("RAM_CODE"))) = {0};
static uint8_t                 s_log_store_cache[APP_LOG_STORE_CACHE_SIZE] __attribute__((section("RAM_CODE"))) = {0};
static uint8_t                 s_read_dump_buffer[APP_LOG_STORE_ONECE_OP_SIZE];

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint32_t log_store_check_sum_calc(uint8_t *p_data, uint32_t len)
{
    uint32_t   check_sum = 0;

    if (p_data && len)
    {
        for (uint8_t i = 0; i < len; i++)
        {
            check_sum += p_data[i];
        }
    }

    return check_sum;
}

static bool log_store_head_check(log_store_head_t *p_head, uint32_t db_addr, uint32_t db_size)
{
    uint16_t  head_len  = sizeof(log_store_head_t);
    uint8_t  *head_data = (uint8_t *)p_head;

    if (p_head->magic != APP_LOG_STORE_MAGIC ||
        p_head->db_addr != db_addr           ||
        p_head->db_size != db_size           ||
        p_head->offset >  db_size)
    {
        return false;
    }

    if (p_head->check_sum != log_store_check_sum_calc(head_data, head_len - 2))
    {
        return false;
    }

    return true;
}

static bool log_store_head_update(uint16_t nv_tag, log_store_head_t *p_head)
{
    uint16_t  head_len  = sizeof(log_store_head_t);
    uint8_t  *head_data = (uint8_t *)p_head;

    p_head->check_sum = log_store_check_sum_calc(head_data, head_len - 2);

    if (nvds_put(nv_tag, head_len, (uint8_t *)p_head))
    {
        return false;
    }

    return true;
}

static bool log_store_time_stamp_encode(uint8_t *p_buffer, uint8_t buffer_size)
{
    if (APP_LOG_STORE_TIME_SIZE != buffer_size)
    {
        return false;
    }

    app_log_store_time_t rtc_time = {0};

    s_log_store_ops.time_get(&rtc_time);

    if (APP_LOG_STORE_TIME_SIZE == snprintf((char *)p_buffer,
                                            APP_LOG_STORE_TIME_SIZE,
                                            "[%04d/%02d/%02d %02d:%02d:%02d:%03d] ", 
                                            rtc_time.year, rtc_time.month, rtc_time.day,
                                            rtc_time.hour, rtc_time.min, rtc_time.sec, rtc_time.msec))
    {
        return true;
    }

    return false;
}

static void log_store_data_flash_write(void)
{
    uint32_t align_num = 0;
    uint32_t read_len;
    uint8_t  *read_buff = s_read_dump_buffer;

    if (0 == (s_log_store_env.store_head.offset % s_log_store_env.blk_size))
    {
        if (s_log_store_ops.flash_erase)
        {
            s_log_store_ops.flash_erase(s_log_store_env.store_head.db_addr + s_log_store_env.store_head.offset,
                                        s_log_store_env.blk_size);
        }
    }

    align_num = ALIGN_NUM(APP_LOG_STORE_ONECE_OP_SIZE, s_log_store_env.store_head.offset);

    if (align_num != s_log_store_env.store_head.offset)
    {
        read_len = ring_buffer_read(&s_log_store_rbuf, read_buff, align_num - s_log_store_env.store_head.offset);
    }
    else
    {
        read_len = ring_buffer_read(&s_log_store_rbuf, read_buff, APP_LOG_STORE_ONECE_OP_SIZE);
    }

    if (s_log_store_ops.flash_write && read_len)
    {
        s_log_store_ops.flash_write(s_log_store_env.store_head.db_addr + s_log_store_env.store_head.offset, read_buff, read_len);
        s_log_store_env.store_head.offset += read_len;
    }

    if (s_log_store_env.store_head.offset >= s_log_store_env.store_head.db_size)
    {
        s_log_store_env.store_head.offset    = 0;
        s_log_store_env.store_head.flip_over = 1;
    }

    log_store_head_update(s_log_store_env.head_nv_tag, &s_log_store_env.store_head);

}

static void log_store_to_flash(void)
{
    log_store_data_flash_write();

    s_log_store_env.store_status &= ~APP_LOG_STORE_SAVE_BIT;
}

static void log_dump_from_flash(void)
{
    uint8_t *dump_buffer = s_read_dump_buffer;
    uint16_t dump_len;
    uint32_t need_dump_size = s_log_store_env.store_head.flip_over ? 
                              s_log_store_env.store_head.db_size : s_log_store_env.store_head.offset;


    if (s_log_store_ops.flash_read && need_dump_size)
    {
        uint32_t align_num = ALIGN_NUM(APP_LOG_STORE_ONECE_OP_SIZE, need_dump_size);

        if (align_num != need_dump_size && 
            (s_log_store_dump_offset + APP_LOG_STORE_ONECE_OP_SIZE) == align_num)
        {
            dump_len = (need_dump_size + APP_LOG_STORE_ONECE_OP_SIZE - align_num);
        }
        else
        {
            dump_len = APP_LOG_STORE_ONECE_OP_SIZE;
        }

        s_log_store_ops.flash_read(s_log_store_dump_offset + s_log_store_env.store_head.db_addr, dump_buffer, dump_len);

        s_log_store_env.store_status &= ~APP_LOG_STORE_DUMP_READY_BIT;
        
        s_log_store_dump_offset += dump_len;
        if (need_dump_size == s_log_store_dump_offset)
        {
            s_log_store_dump_offset = 0;
            s_log_store_env.store_status &= ~APP_LOG_STORE_DUMP_BIT;
        }
        
        if (s_log_dump_cbs->dump_process_cb)
        {
            s_log_dump_cbs->dump_process_cb(dump_buffer, dump_len);
        }

    }
    else
    {
        s_log_store_env.store_status &= ~APP_LOG_STORE_DUMP_BIT;
        s_log_store_env.store_status |= APP_LOG_STORE_DUMP_READY_BIT;
    }
}

static void log_store_flush(void)
{
    uint32_t items_count = 0;

    if (!s_log_store_env.initialized)
    {
        return;
    }
    do
    {
        items_count = ring_buffer_items_count_get(&s_log_store_rbuf);

        if (items_count)
        {
            log_store_data_flash_write();
        }
    } while (items_count >= APP_LOG_STORE_ONECE_OP_SIZE);
}

static void log_dump_ready(void)
{
    uint32_t log_length;
    log_store_flush();

    // get length and send
    if (s_log_dump_cbs->dump_start_cb)
    {
        log_length = s_log_store_env.store_head.flip_over ? 
                     s_log_store_env.store_head.db_size : s_log_store_env.store_head.offset;
        s_log_dump_cbs->dump_start_cb(log_length);
    }
}

static void log_store_clear(void)
{
    s_log_store_env.store_head.offset    = 0;
    s_log_store_env.store_head.flip_over = 0;
    ring_buffer_clean(&s_log_store_rbuf);

    log_store_head_update(s_log_store_env.head_nv_tag, &s_log_store_env.store_head);
    
    s_log_store_dump_offset = 0;
    s_log_store_env.store_status = APP_LOG_STORE_DUMP_READY_BIT;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
uint16_t app_log_store_init(app_log_store_info_t *p_info, app_log_store_op_t *p_op_func)
{
    uint16_t head_len = sizeof(log_store_head_t);

    if (s_log_store_env.initialized)
    {
        return SDK_ERR_DISALLOWED;
    }

    if (NULL == p_info
        || NULL == p_op_func
        || NULL == p_op_func->flash_init
        || NULL == p_op_func->flash_read
        || NULL == p_op_func->flash_write
        || NULL == p_op_func->flash_erase
        || 0 == p_info->db_size
        || 0 == p_info->blk_size
        || 0 != (p_info->db_addr % p_info->blk_size))
    {
        return SDK_ERR_INVALID_PARAM;
    }

    nvds_get(p_info->nv_tag, &head_len, (uint8_t *)&s_log_store_env.store_head);

    if (!log_store_head_check(&s_log_store_env.store_head, p_info->db_addr, p_info->db_size))
    {
        s_log_store_env.store_head.magic     = APP_LOG_STORE_MAGIC;
        s_log_store_env.store_head.db_addr   = p_info->db_addr;
        s_log_store_env.store_head.db_size   = p_info->db_size;
        s_log_store_env.store_head.offset    = 0;
        s_log_store_env.store_head.flip_over = 0;

        if (!log_store_head_update(p_info->nv_tag, &s_log_store_env.store_head))
        {
            return SDK_ERR_SDK_INTERNAL;
        }
    }

    memcpy(&s_log_store_ops, p_op_func, sizeof(s_log_store_ops));

    s_log_store_env.head_nv_tag = p_info->nv_tag;
    s_log_store_env.blk_size    = p_info->blk_size;
    s_log_store_env.initialized = true;
    s_log_store_env.store_status |= APP_LOG_STORE_DUMP_READY_BIT;

    p_op_func->flash_init();

    if (APP_LOG_STORE_CACHE_SIZE != s_log_store_rbuf.buffer_size)
    {
        ring_buffer_init(&s_log_store_rbuf, s_log_store_cache, APP_LOG_STORE_CACHE_SIZE);
    }
    else
    {
        if (s_log_store_ops.sem_give)
        {
            s_log_store_ops.sem_give();
        }
    }

    return SDK_SUCCESS;
}


uint16_t app_log_store_save(const uint8_t *p_data, const uint16_t length)
{
    uint8_t  time_encode[APP_LOG_STORE_TIME_SIZE] = APP_LOG_STORE_TIME_DEFAULT;

    if (!s_log_store_env.initialized)
    {
        return SDK_ERR_DISALLOWED;
    }

    if (s_log_store_ops.time_get)
    {
        log_store_time_stamp_encode(time_encode, APP_LOG_STORE_TIME_SIZE);
        time_encode[APP_LOG_STORE_TIME_SIZE - 1] = ' ';
    }

    ring_buffer_write(&s_log_store_rbuf, time_encode, APP_LOG_STORE_TIME_SIZE);
    ring_buffer_write(&s_log_store_rbuf, p_data, length);

    if ((APP_LOG_STORE_ONECE_OP_SIZE <= ring_buffer_items_count_get(&s_log_store_rbuf)) && 
        !(s_log_store_env.store_status & APP_LOG_STORE_DUMP_BIT))
    {
        s_log_store_env.store_status |= APP_LOG_STORE_SAVE_BIT;
        if (s_log_store_ops.sem_give)
        {
            s_log_store_ops.sem_give();
        }
    }

    return SDK_SUCCESS;
}

uint16_t app_log_store_dump(app_log_dump_cbs_t * p_dump_cbs)
{
    if (!s_log_store_env.initialized)
    {
        return SDK_ERR_DISALLOWED;
    }

    if (s_log_store_env.store_status & APP_LOG_STORE_DUMP_BIT)
    {
        return SDK_ERR_BUSY;
    }
    s_log_dump_cbs = p_dump_cbs;
    s_log_store_env.store_status |= APP_LOG_STORE_DUMP_START_BIT;
    s_log_store_env.store_status |= APP_LOG_STORE_DUMP_BIT;
    if (s_log_store_ops.sem_give)
    {
        s_log_store_ops.sem_give();
    }

    return SDK_SUCCESS;
}

uint16_t app_log_store_clear(void)
{
    if (!s_log_store_env.initialized)
    {
        return SDK_ERR_DISALLOWED;
    }
    s_log_store_env.store_status |= APP_LOG_STORE_CLEAR_BIT;
    if (s_log_store_ops.sem_give)
    {
        s_log_store_ops.sem_give();
    }
    
    return SDK_SUCCESS;
}

void app_log_status_reset(void)
{
    s_log_store_dump_offset = 0;
    s_log_store_env.store_status = APP_LOG_STORE_DUMP_READY_BIT;
}

bool app_log_store_dump_ongoing(void)
{
    return (s_log_store_env.store_status & APP_LOG_STORE_DUMP_BIT) ? true : false;
}

void app_log_dump_continue(void)
{
    s_log_store_env.store_status |= APP_LOG_STORE_DUMP_READY_BIT;
    
    if (s_log_store_env.store_status & APP_LOG_STORE_DUMP_BIT)
    {
        if (s_log_store_ops.sem_give)
        {
            s_log_store_ops.sem_give();
        }
    }
    else
    {
        if (s_log_dump_cbs->dump_finish_cb)
            s_log_dump_cbs->dump_finish_cb();
    }
}

void app_log_store_schedule(void)
{
    if (s_log_store_ops.sem_take)
    {
        s_log_store_ops.sem_take();
    }

    if (s_log_store_env.store_status & APP_LOG_STORE_CLEAR_BIT)
    {
        log_store_clear();
    }
    
    if (s_log_store_env.store_status & APP_LOG_STORE_SAVE_BIT)
    {
        log_store_to_flash();
    }

    if ((s_log_store_env.store_status & APP_LOG_STORE_DUMP_BIT) && (s_log_store_env.store_status & APP_LOG_STORE_DUMP_READY_BIT))
    {
        if (s_log_store_env.store_status & APP_LOG_STORE_DUMP_START_BIT)
        {
            log_dump_ready();
            s_log_store_env.store_status &= ~APP_LOG_STORE_DUMP_START_BIT;
        }

        log_dump_from_flash();
    }
}

#endif


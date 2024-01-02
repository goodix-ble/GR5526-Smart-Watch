/**
 ****************************************************************************************
 *
 * @file app_error_cfg.h
 *
 * @brief App Error Config API
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

#ifndef __APP_ERROR_CFG_H__
#define __APP_ERROR_CFG_H__

#include "grx_sys.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/**
 * @defgroup APP_ERROR_CFG_MAROC Defines
 * @{
 */
#define APP_IS_USING_FREEROTS               false                                 /**< Is using FREEROTS or not. */
#define APP_ERROR_DUMP_STACK_INFO_ENABLE    1                                     /**< Enable dump stack information. */
#define APP_ERROR_INFO_PRINT_ENABLE         1                                     /**< Enable error information prinf. */
#define APP_ERROR_CALL_STACK_DEPTH_MAX      16                                    /**< Supported function call stack max depth, default is 16. */

#if APP_ERROR_INFO_PRINT_ENABLE
    #define APP_ERROR_INFO_PRINT(...)           printf(__VA_ARGS__);printf("\r\n");/**< Print line. */
#else
    #define APP_ERROR_INFO_PRINT(...)
#endif

#if APP_IS_USING_FREEROTS
    #include "FreeRTOS.h"
    extern uint32_t *vTaskStackAddr(void);
    extern uint32_t  vTaskStackSize(void);
    extern char     *vTaskName(void);
#endif

/** @} */

#endif


/**
 ******************************************************************************
 *
 * @file gr5xx_fpb.h
 *
 ******************************************************************************
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
 * @addtogroup SYSTEM
 * @{
 */
 
/**
 * @addtogroup FPB
 * @{
 * @brief Definitions and prototypes for FPB interface.
 */

#ifndef __GR5XX_FPB_H_
#define __GR5XX_FPB_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "grx_hal.h"

/**
 *@addtogroup GR55XX_FPB_ENUMERATIONS Enumerations
 * @{
 */
/**@brief FPB mode. */
typedef enum
{
    FPB_MODE_PATCH_ONLY = 0,                /**< FPB MODE ENABLE FOR PATCH ONLY*/
    FPB_MODE_DEBUG_ONLY,                    /**< FPB MODE ENABLE FOR DEBUG ONLY*/
    FPB_MODE_PATCH_AND_DEBUG,               /**< FPB MODE ENABLE FOR PATCH AND DEBUG*/
    FPB_MODE_TEST=0x88,
} fpb_mode_t ;

typedef enum
{
  FPB_PATCH_OFF=0,
  FPB_PATCH_ON,
}fpb_t;

typedef void(*fun_t)(void);

fpb_t fpb_save_state(void);
void fpb_load_state(fpb_t state);
void gr5xx_fpb_init(fpb_mode_t mode);
int32_t gr5xx_fpb_func_register(uint32_t bug_func, uint32_t new_func);
void gr5xx_svc_process(void);
void svc_func_register(uint8_t svc_num, uint32_t user_func);


#define FPB_SAVE()       fpb_t __fpb_state_local_var=fpb_save_state()
#define FPB_LOAD()       fpb_load_state(__fpb_state_local_var)
#define FPB_PATCH_ON()   fpb_save_state();
#define FPB_PATCH_OFF()  fpb_load_state(FPB_PATCH_OFF)
#define FPB_MGMT_LOCK    GLOBAL_EXCEPTION_DISABLE
#define FPB_MGMT_UNLOCK  GLOBAL_EXCEPTION_ENABLE
#define FPB_LOG(...)     //printf(__VA_ARGS__)


/** @} */
#endif
/** @} */
/** @} */


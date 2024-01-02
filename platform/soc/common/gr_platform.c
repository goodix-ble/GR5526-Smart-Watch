/**
 *******************************************************************************
 *
 * @file   gr_platform.c
 *
 * @brief  Platform Initialization Routines.
 *
 *******************************************************************************

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
 *******************************************************************************
 */

/*
 * INCLUDE FILES
 *******************************************************************************
 */
#include "grx_sys.h"
#include "gr_soc.h"
#include "gr_plat.h"

#if defined (__CC_ARM )
const APP_INFO_t BUILD_IN_APP_INFO __attribute__((at(APP_INFO_ADDR))) =
#elif defined (__ICCARM__)
__root const APP_INFO_t BUILD_IN_APP_INFO @ (APP_INFO_ADDR) =
#else
const APP_INFO_t BUILD_IN_APP_INFO __attribute__((section(".app_info"))) =
#endif
{
    .app_pattern      = APP_INFO_PATTERN_VALUE,
    .app_info_version = APP_INFO_VERSION,
    .chip_ver         = CHIP_VER,
    .load_addr        = APP_CODE_LOAD_ADDR,
    .run_addr         = APP_CODE_RUN_ADDR,
    .app_info_sum     = CHECK_SUM,
    .check_img        = BOOT_CHECK_IMAGE,
    .boot_delay       = BOOT_LONG_TIME,
    .sec_cfg          = SECURITY_CFG_VAL,
#ifdef APP_INFO_COMMENTS
    .comments         = APP_INFO_COMMENTS,
#endif
};

void C_CONSTRUCTOR system_platform_init(void)
{
    vector_table_init();
    sdk_init();
    soc_init();
    return;
}

#if defined ( __ICCARM__ )
extern void __iar_program_start(void);
void __main(void)
{
    __iar_program_start();
}

extern void __iar_data_init3(void);
int __low_level_init(void)
{
    __iar_data_init3();
    system_platform_init();
    return 0;
}
#elif defined ( __GNUC__ ) && !defined ( __CC_ARM ) 
extern int main(void);
void __main(void)
{
    __asm("ldr    r1, =__etext\n");
    __asm("ldr    r2, =__data_start__\n");
    __asm("ldr    r3, =__data_end__\n");
    __asm(".L_loop1:\n");
    __asm("cmp    r2, r3\n");
    __asm("ittt   lt\n");
    __asm("ldrlt  r0, [r1], #4\n");
    __asm("strlt  r0, [r2], #4\n");
    __asm("blt    .L_loop1\n");
    __asm("ldr    r1, =__bss_start__\n");
    __asm("ldr    r2, =__bss_end__\n");
    __asm("movs   r0, 0\n");
    __asm(".L_loop3:\n");
    __asm("cmp    r1, r2\n");
    __asm("itt    lt\n");
    __asm("strlt  r0, [r1], #4\n");
    __asm("blt    .L_loop3\n");
    system_platform_init();
    main();
}
#endif

void main_init(void)
{
#if defined ( SOC_GR5332 )
    uint32_t boot_flag = pwr_mgmt_get_wakeup_flag();
#else
    uint32_t boot_flag = get_wakeup_flag();
 #endif
    if(COLD_BOOT == boot_flag)
    {
        extern void __main(void);
        __main();
    }
    else
    {
        warm_boot_process();
        while (1);
    }
}


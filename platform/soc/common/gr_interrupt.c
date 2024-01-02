/**
 *****************************************************************************************
 *
 * @file   gr_interrupt.c
 *
 * @brief  Interrupt Service Routines.
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
#include "custom_config.h"
#include "grx_sys.h"

/******************************************************************************/
/*           Cortex-M4F Processor Interruption and Exception Handlers         */
/*           Add here the Interrupt Handler for the BLE peripheral(s)         */
/******************************************************************************/

/**
 ****************************************************************************************
 * @brief  hardfault Interrupt Handler
 * @retval void
 ****************************************************************************************
 */
#if SYS_FAULT_TRACE_ENABLE

__WEAK void app_log_flush(void)
{
}

#if defined ( __CC_ARM )
uint32_t R4_R11_REG[8];
SECTION_RAM_CODE __WEAK void hardfault_trace_handler(uint32_t sp)
{
    printf("HARDFAULT CALLSTACK INFO:\r\n");
    printf("================================\r\n");
    printf("  r0: %08x     r1: %08x\r\n",   ((uint32_t *)sp)[0], ((uint32_t *)sp)[1]);
    printf("  r2: %08x     r3: %08x\r\n",   ((uint32_t *)sp)[2], ((uint32_t *)sp)[3]);
    printf("  r4: %08x     r5: %08x\r\n",   R4_R11_REG[0],  R4_R11_REG[1] );
    printf("  r6: %08x     r7: %08x\r\n",   R4_R11_REG[2],  R4_R11_REG[3] );
    printf("  r8: %08x     r9: %08x\r\n",   R4_R11_REG[4],  R4_R11_REG[5] );
    printf("  r10:%08x     r11:%08x\r\n",   R4_R11_REG[6],  R4_R11_REG[7] );
    printf("  r12:%08x     lr: %08x\r\n",   ((uint32_t *)sp)[4], ((uint32_t *)sp)[5]);
    printf("  pc: %08x     xpsr: %08x\r\n", ((uint32_t *)sp)[6], ((uint32_t *)sp)[7]);
    printf("================================\r\n");
    app_log_flush();
    while (1);
}

SECTION_RAM_CODE __WEAK void cortex_backtrace_fault_handler(uint32_t fault_handler_lr, uint32_t fault_handler_sp)
{
}

SECTION_RAM_CODE __asm void HardFault_Handler (void)
{
#if (ENABLE_BACKTRACE_FEA == 0)//use fault trace module
    PRESERVE8
    IMPORT  hardfault_trace_handler
    IMPORT  R4_R11_REG
    LDR R0,=R4_R11_REG
    STMIA R0!,{R4-R11}
    MOV R0,SP
    BL  hardfault_trace_handler
#elif (ENABLE_BACKTRACE_FEA == 1)//use cortex_backtrace module
    PRESERVE8
    IMPORT  cortex_backtrace_fault_handler
    MOV     r0, lr
    MOV     r1, sp
    BL      cortex_backtrace_fault_handler
#endif

Fault_Loop
    BL      Fault_Loop
    ALIGN
}

#elif defined ( __GNUC__ )

__WEAK void hardfault_trace_handler(unsigned int *hardfault_args)
{
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;

    stacked_r0 = ((unsigned long) hardfault_args[0]);
    stacked_r1 = ((unsigned long) hardfault_args[1]);
    stacked_r2 = ((unsigned long) hardfault_args[2]);
    stacked_r3 = ((unsigned long) hardfault_args[3]);

    stacked_r12 = ((unsigned long) hardfault_args[4]);
    stacked_lr = ((unsigned long) hardfault_args[5]);
    stacked_pc = ((unsigned long) hardfault_args[6]);
    stacked_psr = ((unsigned long) hardfault_args[7]);
    printf("HARDFAULT CALLSTACK INFO:\r\n");
    printf("================================\r\n");
    printf("R0 = %x\r\n", stacked_r0);
    printf("R1 = %x\r\n", stacked_r1);
    printf("R2 = %x\r\n", stacked_r2);
    printf("R3 = %x\r\n", stacked_r3);
    printf("R12 = %x\r\n", stacked_r12);
    printf("LR [R14] = %x  subroutine call return address\r\n", stacked_lr);
    printf("PC [R15] = %x  program counter\r\n", stacked_pc);
    printf("PSR = %x\r\n", stacked_psr);
    printf("BFAR = %x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
    printf("CFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
    printf("HFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
    printf("DFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
    printf("AFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
    printf("SCB_SHCSR = %x\r\n", SCB->SHCSR);
    printf("================================\r\n");
    app_log_flush();

    while (1);
}

SECTION_RAM_CODE __WEAK void cortex_backtrace_fault_handler(uint32_t fault_handler_lr, uint32_t fault_handler_sp)
{
}

SECTION_RAM_CODE void HardFault_Handler (void)
{
#if (ENABLE_BACKTRACE_FEA == 0)//use fault trace module
    __asm("TST     LR,#4\n");
    __asm("ITE     EQ\n");
    __asm("MRSEQ   R0,MSP\n");
    __asm("MRSNE   R0,PSP\n");
    __asm("BL      hardfault_trace_handler\n");
#elif (ENABLE_BACKTRACE_FEA == 1)//use cortex_backtrace module
    __asm("MOV     r0, lr");
    __asm("MOV     r1, sp");
    __asm("BL      cortex_backtrace_fault_handler");
#endif

    while (1);
}

#else

void HardFault_Handler (void)
{
   while (1);
}

#endif

#else /*SYS_FAULT_TRACE_ENABLE*/

SECTION_RAM_CODE void HardFault_Handler (void)
{
    while (1);
}

#endif  /*SYS_FAULT_TRACE_ENABLE*/

/**
 ****************************************************************************************
 * @brief  MemManage fault Interrupt Handler
 * @retval  void
 ****************************************************************************************
 */
void MemManage_Handler(void)
{
    while (1);
}

/**
 ****************************************************************************************
 * @brief  Bus Fault Interrupt Handler
 * @retval  void
 ****************************************************************************************
 */
void BusFault_Handler(void)
{
    while (1);
}

/**
 ****************************************************************************************
 * @brief  UsageFault Interrupt Handler
 * @retval  void
 ****************************************************************************************
 */
void UsageFault_Handler(void)
{
    while (1);
}

#if defined ( __CC_ARM )
SECTION_RAM_CODE __asm void SVC_Handler(void)
{
    PRESERVE8
    IMPORT gr5xx_svc_process
    LDR    R0, =gr5xx_svc_process
    BX     R0
    ALIGN
}
#elif !defined ( __CC_ARM ) && defined ( __GNUC__ )
SECTION_RAM_CODE void __attribute__((naked))SVC_Handler(void)
{
    extern void gr5xx_svc_process(void);
    __asm("LDR    R0, =gr5xx_svc_process\n"
          "BX     R0");
}
#elif defined (__ICCARM__)
SECTION_RAM_CODE void __attribute__((naked))SVC_Handler (void)
{
    extern void gr5xx_svc_process(void);
    asm volatile ("LDR    R0, =gr5xx_svc_process \n\t"
                  "BX     R0");
}
#endif

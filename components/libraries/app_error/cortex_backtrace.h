/**
 ****************************************************************************************
 *
 * @file cortex_backtrace.h
 *
 * @brief Cortex Backtrace API
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

#ifndef __CORTEX_BACKTRACE_H__
#define __CORTEX_BACKTRACE_H__

#include "app_error_cfg.h"
#include <stdint.h>

/**
 * @defgroup CORTEX_BACKTRACE_MAROC Defines
 * @{
 */
#define CB_CPU_ARM_CORTEX_M0          (0x00U)      /**< Cortex-M0 Core */
#define CB_CPU_ARM_CORTEX_M3          (0x03U)      /**< Cortex-M3 Core */
#define CB_CPU_ARM_CORTEX_M4          (0x04U)      /**< Cortex-M4 Core */
#define CB_CPU_ARM_CORTEX_M7          (0x07U)      /**< Cortex-M7 Core */

#define CB_SYSHND_CTRL                (*(volatile unsigned int*)  (0xE000ED24u))    /**< System Handler Control and State Register. */
#define CB_NVIC_MFSR                  (*(volatile unsigned char*) (0xE000ED28u))    /**< Memory management fault State register. */
#define CB_NVIC_BFSR                  (*(volatile unsigned char*) (0xE000ED29u))    /**< Bus fault State register. */
#define CB_NVIC_UFSR                  (*(volatile unsigned short*)(0xE000ED2Au))    /**< Usage fault State register. */
#define CB_NVIC_HFSR                  (*(volatile unsigned int*)  (0xE000ED2Cu))    /**< Hard fault State register. */
#define CB_NVIC_DFSR                  (*(volatile unsigned short*)(0xE000ED30u))    /**< Debug fault State register. */
#define CB_NVIC_MMAR                  (*(volatile unsigned int*)  (0xE000ED34u))    /**< Memory management fault address register. */
#define CB_NVIC_BFAR                  (*(volatile unsigned int*)  (0xE000ED38u))    /**< Bus fault manage address register. */
#define CB_NVIC_AFSR                  (*(volatile unsigned short*)(0xE000ED3Cu))    /**< Auxiliary fault State register. */

/**@brief ELF(Executable and Linking Format) file extension name for each compiler. */
#if defined(__CC_ARM)
    #define CB_ELF_FILE_EXTENSION_NAME          ".axf"
#elif defined(__ICCARM__)
    #define CB_ELF_FILE_EXTENSION_NAME          ".out"
#elif defined(__GNUC__)
    #define CB_ELF_FILE_EXTENSION_NAME          ".elf"
#else
    #error "not supported compiler"
#endif
/** @} */

/**
 * @defgroup CORTEX_BACKTRACE_STRUCT Structures
 * @{
 */
/**@brief Cortex-M fault registers. */
typedef struct
{
  struct 
  {
    unsigned int r0;                     /**< Register R0. */
    unsigned int r1;                     /**< Register R1. */
    unsigned int r2;                     /**< Register R2. */
    unsigned int r3;                     /**< Register R3. */
    unsigned int r12;                    /**< Register R12. */
    unsigned int lr;                     /**< Link register. */
    unsigned int pc;                     /**< Program counter. */
    union 
    {
      unsigned int value;
      struct 
      {
        unsigned int IPSR : 8;           /**< Interrupt Program Status register (IPSR). */
        unsigned int EPSR : 19;          /**< Execution Program Status register (EPSR). */
        unsigned int APSR : 5;           /**< Application Program Status register (APSR). */
      } bits;
    } psr;                               /**< Program status register. */
  } saved;

  union
  {
    unsigned int value;
    struct
    {
      unsigned int MEMFAULTACT    : 1;   /**< Read as 1 if memory management fault is active. */
      unsigned int BUSFAULTACT    : 1;   /**< Read as 1 if bus fault exception is active. */
      unsigned int UnusedBits1    : 1;   /**< Unused Bits 1. */
      unsigned int USGFAULTACT    : 1;   /**< Read as 1 if usage fault exception is active. */
      unsigned int UnusedBits2    : 3;   /**< Unused Bits 2. */
      unsigned int SVCALLACT      : 1;   /**< Read as 1 if SVC exception is active. */
      unsigned int MONITORACT     : 1;   /**< Read as 1 if debug monitor exception is active. */
      unsigned int UnusedBits3    : 1;   /**< Unused Bits 3. */
      unsigned int PENDSVACT      : 1;   /**< Read as 1 if PendSV exception is active. */
      unsigned int SYSTICKACT     : 1;   /**< Read as 1 if SYSTICK exception is active. */
      unsigned int USGFAULTPENDED : 1;   /**< Usage fault pended; usage fault started but was replaced by a higher-priority exception. */
      unsigned int MEMFAULTPENDED : 1;   /**< Memory management fault pended; memory management fault started but was replaced by a higher-priority exception. */
      unsigned int BUSFAULTPENDED : 1;   /**< Bus fault pended; bus fault handler was started but was replaced by a higher-priority exception. */
      unsigned int SVCALLPENDED   : 1;   /**< SVC pended; SVC was started but was replaced by a higher-priority exception. */
      unsigned int MEMFAULTENA    : 1;   /**< Memory management fault handler enable. */
      unsigned int BUSFAULTENA    : 1;   /**< Bus fault handler enable. */
      unsigned int USGFAULTENA    : 1;   /**< Usage fault handler enable. */
    } bits;
  } syshndctrl;                          /**< System Handler Control and State Register (0xE000ED24). */

  union
  {
    unsigned char value;
    struct
    {
      unsigned char IACCVIOL    : 1;     /**< Instruction access violation. */
      unsigned char DACCVIOL    : 1;     /**< Data access violation. */
      unsigned char UnusedBits  : 1;     /**< Unused Bits 1. */
      unsigned char MUNSTKERR   : 1;     /**< Unstacking error. */
      unsigned char MSTKERR     : 1;     /**< Stacking error. */
      unsigned char MLSPERR     : 1;     /**< Floating-point lazy state preservation (M4/M7). */
      unsigned char UnusedBits2 : 1;     /**< Unused Bits 2. */
      unsigned char MMARVALID   : 1;     /**< Indicates the MMAR is valid. */
    } bits;
  } mfsr;                                /**< Memory Management Fault Status Register (0xE000ED28). */
  unsigned int mmar;                     /**< Memory Management Fault Address Register (0xE000ED34). */

  union
  {
    unsigned char value;
    struct
    {
      unsigned char IBUSERR    : 1;      /**< Instruction access violation. */
      unsigned char PRECISERR  : 1;      /**< Precise data access violation. */
      unsigned char IMPREISERR : 1;      /**< Imprecise data access violation. */
      unsigned char UNSTKERR   : 1;      /**< Unstacking error. */
      unsigned char STKERR     : 1;      /**< Stacking error. */
      unsigned char LSPERR     : 1;      /**<  Floating-point lazy state preservation (M4/M7). */
      unsigned char UnusedBits : 1;      /**< Unused Bits 1. */
      unsigned char BFARVALID  : 1;      /**< Indicates BFAR is valid. */
    } bits;
  } bfsr;                                /**< Bus Fault Status Register (0xE000ED29). */
  unsigned int bfar;                     /**< Bus Fault Manage Address Register (0xE000ED38). */

  union
  {
    unsigned short value;
    struct
    {
      unsigned short UNDEFINSTR : 1;     /**< Attempts to execute an undefined instruction. */
      unsigned short INVSTATE   : 1;     /**< Attempts to switch to an invalid state (e.g., ARM). */
      unsigned short INVPC      : 1;     /**< Attempts to do an exception with a bad value in the EXC_RETURN number. */
      unsigned short NOCP       : 1;     /**< Attempts to execute a coprocessor instruction. */
      unsigned short UnusedBits : 4;     /**< Unused Bits 1. */
      unsigned short UNALIGNED  : 1;     /**< Indicates that an unaligned access fault has taken place. */
      unsigned short DIVBYZERO0 : 1;     /**< Indicates a divide by zero has taken place (can be set only if DIV_0_TRP is set). */
    } bits;
  } ufsr;                                /**< Usage Fault Status Register (0xE000ED2A). */

  union
  {
    unsigned int value;
    struct
    {
      unsigned int UnusedBits  : 1;
      unsigned int VECTBL      : 1;      /**< Indicates hard fault is caused by failed vector fetch. */
      unsigned int UnusedBits2 : 28;     /**< Unused Bits 1. */
      unsigned int FORCED      : 1;      /**< Indicates hard fault is taken because of bus fault/memory management fault/usage fault. */
      unsigned int DEBUGEVT    : 1;      /**< Indicates hard fault is triggered by debug event. */
    } bits;
  } hfsr;                                /**< Hard Fault Status Register (0xE000ED2C). */

  union
  {
    unsigned int value;
    struct
    {
      unsigned int HALTED   : 1;         /**< Halt requested in NVIC. */
      unsigned int BKPT     : 1;         /**< BKPT instruction executed. */
      unsigned int DWTTRAP  : 1;         /**< DWT match occurred. */
      unsigned int VCATCH   : 1;         /**< Vector fetch occurred. */
      unsigned int EXTERNAL : 1;         /**< EDBGRQ signal asserted. */
    } bits;                              /**< Unused Bits 1. */
  } dfsr;                                /**< Debug Fault Status Register (0xE000ED30). */

  unsigned int afsr;                     /**< Auxiliary Fault Status Register (0xE000ED3C), Vendor controlled (optional). */
} cb_hard_fault_regs_t;
/** @} */

/**
 * @defgroup CORTEX_BACKTRACE_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Cortex backtrace handler for fault.
 *
 * @param[in] fault_handler_lr: LR register value on fault handler.
 * @param[in] fault_handler_sp: Stack pointer on fault handler.
 *****************************************************************************************
 */
void cortex_backtrace_fault_handler(uint32_t fault_handler_lr, uint32_t fault_handler_sp);
/** @} */


#endif

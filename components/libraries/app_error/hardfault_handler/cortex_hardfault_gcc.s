;/**
; *****************************************************************************************
; *
; * @file app_hardfault_kei.s
; *
; * @brief App HardFault Handler Function Implementation for GCC.
; *
; * Copyright(C) 2016-2018, Shenzhen Goodix Technology Co., Ltd
; * All Rights Reserved
; *
; *****************************************************************************************
; */

.syntax unified
.thumb
.text

/* NOTE: If use this file's HardFault_Handler, please comments the HardFault_Handler code on other file. */

.global HardFault_Handler
.type HardFault_Handler, %function
HardFault_Handler:
    MOV     r0, lr                              /* get lr */
    MOV     r1, sp                              /* get stack pointer (current is MSP) */
    BL      cortex_backtrace_fault_handler

Fault_Loop:
    BL      Fault_Loop              /* while(1) */

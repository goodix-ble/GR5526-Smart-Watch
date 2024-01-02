;/**
; *****************************************************************************************
; *
; * @file app_hardfault_kei.s
; *
; * @brief App HardFault Handler Function Implementation for IAR.
; *
; * Copyright(C) 2016-2018, Shenzhen Goodix Technology Co., Ltd
; * All Rights Reserved
; *
; *****************************************************************************************
; */

    SECTION    .text:CODE(2)
    THUMB
    REQUIRE8
    PRESERVE8

; NOTE: If use this file's HardFault_Handler, please comments the HardFault_Handler code on other file.
    IMPORT cortex_backtrace_fault_handler
    EXPORT HardFault_Handler

HardFault_Handler:
    MOV     r0, lr                          ; get lr
    MOV     r1, sp                          ; get stack pointer (current is MSP)
    BL      cortex_backtrace_fault_handler

Fault_Loop
    BL      Fault_Loop                      ;while(1)

    END

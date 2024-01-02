;/**************************************************************************//**
; * @file     startup_ARMCM4.s
; * @brief    CMSIS Core Device Startup File for
; *           ARMCM4 Device Series
; * @version  V5.00
; * @date     08. March 2016
; ******************************************************************************/
;/*
; * Copyright (c) 2009-2016 ARM Limited. All rights reserved.
; *
; * SPDX-License-Identifier: Apache-2.0
; *
; * Licensed under the Apache License, Version 2.0 (the License); you may
; * not use this file except in compliance with the License.
; * You may obtain a copy of the License at
; *
; * www.apache.org/licenses/LICENSE-2.0
; *
; * Unless required by applicable law or agreed to in writing, software
; * distributed under the License is distributed on an AS IS BASIS, WITHOUT
; * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; * See the License for the specific language governing permissions and
; * limitations under the License.
; */

;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        EXTERN  BLE_IRQHandler
        EXTERN  BLE_SDK_Handler
        EXTERN  main_init

        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     MemManage_Handler
        DCD     BusFault_Handler
        DCD     UsageFault_Handler
__vector_table_0x1c
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     DebugMon_Handler
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD     WDT_IRQHandler            ;  0: Watchdog Timer
        DCD     BLE_SDK_Handler            ;  1: Reserved
        DCD     BLE_IRQHandler            ;  2: BLE
        DCD     DMA0_IRQHandler           ;  3: DMA0
        DCD     SPI_M_IRQHandler          ;  4: SPI_M
        DCD     SPI_S_IRQHandler          ;  5: SPI_S
        DCD     EXT0_IRQHandler           ;  6: GPIO0
        DCD     EXT1_IRQHandler           ;  7: GPIO1
        DCD     TIMER0_IRQHandler         ;  8: TIMER0
        DCD     TIMER1_IRQHandler         ;  9: TIMER1
        DCD     DUAL_TIMER_IRQHandler     ; 10: DUAL_TIMER0/DUAL_TIMER1
        DCD     QSPI0_IRQHandler          ; 11: QSPI0
        DCD     UART0_IRQHandler          ; 12: UART0
        DCD     UART1_IRQHandler          ; 13: UART1
        DCD     I2C0_IRQHandler           ; 14: I2C0
        DCD     I2C1_IRQHandler           ; 15: I2C1
        DCD     AES_IRQHandler            ; 16: AES
        DCD     HMAC_IRQHandler           ; 17: HMAC
        DCD     EXT2_IRQHandler           ; 18: GPIO2
        DCD     RNG_IRQHandler            ; 19: TRNG Interrupt
        DCD     PMU_IRQHandler            ; 20: PMU
        DCD     PKC_IRQHandler            ; 21: PKC
        DCD     XQSPI_IRQHandler          ; 22: XQSPI
        DCD     QSPI1_IRQHandler          ; 23: QSPI1
        DCD     PWR_CMD_IRQHandler        ; 24: PWR_CMD
        DCD     BLESLP_IRQHandler         ; 25: BLE Sleep
        DCD     SLPTIMER_IRQHandler       ; 26: Sleep Timer
        DCD     EXTWKUP_IRQHandler        ; 27: EXT Wakeup
        DCD     AON_WDT_IRQHandler        ; 28: AON_WDT
        DCD     I2S_M_IRQHandler          ; 29: I2S_M
        DCD     I2S_S_IRQHandler          ; 30: I2S_S
        DCD     ISO7816_IRQHandler        ; 31: ISO7816
        DCD     PRESENT_IRQHandler        ; 32: PRESENT
        DCD     CALENDAR_IRQHandler       ; 33: CALENDAR
        DCD     COMM_CORE_IRQHandler      ; 34: COMM_CORE
        DCD     DMA1_IRQHandler           ; 35: DMA1
        DCD     DMA2_IRQHandler           ; 36: DMA2
        DCD     DSPI_IRQHandler           ; 37: DSPI
        DCD     AON_IRQHandler            ; 38: AON
        DCD     PDM_IRQHandler            ; 39: PDM
        DCD     VTTBL_IRQHandler          ; 40: VTTBL
        DCD     CTE_FULL_IRQHandler       ; 41: CTE_FULL
        DCD     USB_IRQHandler            ; 42: USB
        DCD     GPADC_IRQHandler          ; 43: GPADC
        DCD     AON_PMU_BOD_FEDGE_IRQHandler ; 44: AON_PMU_BOD_FEDGE
        DCD     AON_PMU_MSIO_COMP_IRQHandler ; 45: AON_PMU_MSIO_COMP
        DCD     AON_PMU_USB_WKUP_IRQHandler  ; 46: AON_PMU_USB_WKUP

__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size  EQU   __Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =main_init
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
        B MemManage_Handler

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
        B BusFault_Handler

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
        B UsageFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
        B DebugMon_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B SysTick_Handler

        PUBWEAK WDT_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT_IRQHandler
        B WDT_IRQHandler

        ;PUBWEAK BLE_IRQHandler
        ;SECTION .text:CODE:REORDER:NOROOT(1)
;BLE_IRQHandler
        ;B BLE_IRQHandler

        PUBWEAK DMA0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA0_IRQHandler
        B DMA0_IRQHandler

        PUBWEAK SPI_M_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI_M_IRQHandler
        B SPI_M_IRQHandler

        PUBWEAK SPI_S_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI_S_IRQHandler
        B SPI_S_IRQHandler

        PUBWEAK EXT0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXT0_IRQHandler
        B EXT0_IRQHandler

        PUBWEAK EXT1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXT1_IRQHandler
        B EXT1_IRQHandler

        PUBWEAK TIMER0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER0_IRQHandler
        B TIMER0_IRQHandler

        PUBWEAK TIMER1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER1_IRQHandler
        B TIMER1_IRQHandler

        PUBWEAK DUAL_TIMER_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DUAL_TIMER_IRQHandler
        B DUAL_TIMER_IRQHandler

        PUBWEAK QSPI0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
QSPI0_IRQHandler
        B QSPI0_IRQHandler

        PUBWEAK UART0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART0_IRQHandler
        B UART0_IRQHandler

        PUBWEAK UART1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART1_IRQHandler
        B UART1_IRQHandler

        PUBWEAK I2C0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C0_IRQHandler
        B I2C0_IRQHandler

        PUBWEAK I2C1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C1_IRQHandler
        B I2C1_IRQHandler

        PUBWEAK AES_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
AES_IRQHandler
        B AES_IRQHandler

        PUBWEAK HMAC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
HMAC_IRQHandler
        B HMAC_IRQHandler

        PUBWEAK EXT2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXT2_IRQHandler
        B EXT2_IRQHandler

        PUBWEAK RNG_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RNG_IRQHandler
        B RNG_IRQHandler

        PUBWEAK PMU_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PMU_IRQHandler
        B PMU_IRQHandler

        PUBWEAK PKC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PKC_IRQHandler
        B PKC_IRQHandler

        PUBWEAK XQSPI_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
XQSPI_IRQHandler
        B XQSPI_IRQHandler

        PUBWEAK QSPI1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
QSPI1_IRQHandler
        B QSPI1_IRQHandler

        PUBWEAK PWR_CMD_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWR_CMD_IRQHandler
        B PWR_CMD_IRQHandler

        PUBWEAK BLESLP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
BLESLP_IRQHandler
        B BLESLP_IRQHandler

        PUBWEAK SLPTIMER_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SLPTIMER_IRQHandler
        B SLPTIMER_IRQHandler

        PUBWEAK EXTWKUP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXTWKUP_IRQHandler
        B EXTWKUP_IRQHandler

        PUBWEAK AON_WDT_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
AON_WDT_IRQHandler
        B AON_WDT_IRQHandler

        PUBWEAK I2S_M_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2S_M_IRQHandler
        B I2S_M_IRQHandler

        PUBWEAK I2S_S_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2S_S_IRQHandler
        B I2S_S_IRQHandler

        PUBWEAK ISO7816_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ISO7816_IRQHandler
        B ISO7816_IRQHandler

        PUBWEAK PRESENT_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PRESENT_IRQHandler
        B PRESENT_IRQHandler

        PUBWEAK CALENDAR_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CALENDAR_IRQHandler
        B CALENDAR_IRQHandler

        PUBWEAK COMM_CORE_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMM_CORE_IRQHandler
        B COMM_CORE_IRQHandler

        PUBWEAK DMA1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA1_IRQHandler
        B DMA1_IRQHandler

        PUBWEAK DMA2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA2_IRQHandler
        B DMA2_IRQHandler

    PUBWEAK DSPI_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DSPI_IRQHandler
        B DSPI_IRQHandler

    PUBWEAK AON_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
AON_IRQHandler
        B AON_IRQHandler

    PUBWEAK PDM_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PDM_IRQHandler
        B PDM_IRQHandler

    PUBWEAK VTTBL_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VTTBL_IRQHandler
        B VTTBL_IRQHandler

    PUBWEAK CTE_FULL_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CTE_FULL_IRQHandler
        B CTE_FULL_IRQHandler

    PUBWEAK USB_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
USB_IRQHandler
        B USB_IRQHandler

    PUBWEAK GPADC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPADC_IRQHandler
        B GPADC_IRQHandler

    PUBWEAK AON_PMU_BOD_FEDGE_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
AON_PMU_BOD_FEDGE_IRQHandler
        B AON_PMU_BOD_FEDGE_IRQHandler

    PUBWEAK AON_PMU_MSIO_COMP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
AON_PMU_MSIO_COMP_IRQHandler
        B AON_PMU_MSIO_COMP_IRQHandler

    PUBWEAK AON_PMU_USB_WKUP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
AON_PMU_USB_WKUP_IRQHandler
        B AON_PMU_USB_WKUP_IRQHandler
        END
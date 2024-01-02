/**************************************************************************//**
 * @file     startup_ARMCM4.s
 * @brief    CMSIS Core Device Startup File for
 *           ARMCM4 Device Series
 * @version  V5.00
 * @date     02. March 2016
 ******************************************************************************/
/*
 * Copyright (c) 2009-2016 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

    .syntax     unified
    .arch       armv7-m

    .section    .stack
    .align      3
//#ifdef __STACK_SIZE
//  .equ    Stack_Size, __STACK_SIZE
//#else
    .equ    Stack_Size, 0x00008000
//#endif
    .globl      __StackTop
    .globl      __StackLimit
__StackLimit:
    .space      Stack_Size
    .size       __StackLimit, . - __StackLimit
__StackTop:
    .size       __StackTop, . - __StackTop

    .section    .heap
    .align      3
#ifdef __HEAP_SIZE
    .equ        Heap_Size, __HEAP_SIZE
#else
    .equ        Heap_Size, 0x00000100
#endif
    .globl      __HeapBase
    .globl      __HeapLimit
__HeapBase:
    .if         Heap_Size
    .space      Heap_Size
    .endif
    .size       __HeapBase, . - __HeapBase
__HeapLimit:
    .size       __HeapLimit, . - __HeapLimit

    .section    .vectors
    .align      2
    .globl      __Vectors
__Vectors:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    MemManage_Handler     /* MPU Fault Handler */
    .long    BusFault_Handler      /* Bus Fault Handler */
    .long    UsageFault_Handler    /* Usage Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    DebugMon_Handler      /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    /* External interrupts */
    .long    WDT_IRQHandler        /*  0: Watchdog Timer             */
    .long    BLE_SDK_Handler       /*  1: Reserved                   */
    .long    BLE_IRQHandler        /*  2: BLE                        */
    .long    DMA0_IRQHandler       /*  3: DMA0                       */
    .long    SPI_M_IRQHandler      /*  4: SPI_M                      */
    .long    SPI_S_IRQHandler      /*  5: SPI_S                      */
    .long    EXT0_IRQHandler       /*  6: GPIO0                      */
    .long    EXT1_IRQHandler       /*  7: GPIO1                      */
    .long    TIMER0_IRQHandler     /*  8: TIMER0                     */
    .long    TIMER1_IRQHandler     /*  9: TIMER1                     */
    .long    DUAL_TIMER_IRQHandler /* 10: DUAL_TIMER0/DUAL_TIMER1    */
    .long    QSPI0_IRQHandler      /* 11: QSPI0                      */
    .long    UART0_IRQHandler      /* 12: UART0                      */
    .long    UART1_IRQHandler      /* 13: UART1                      */
    .long    I2C0_IRQHandler       /* 14: I2C0                       */
    .long    I2C1_IRQHandler       /* 15: I2C1                       */
    .long    AES_IRQHandler        /* 16: AES                        */
    .long    HMAC_IRQHandler       /* 17: HMAC                       */
    .long    EXT2_IRQHandler       /* 18: GPIO2                      */
    .long    RNG_IRQHandler        /* 19: TRNG Interrupt             */
    .long    PMU_IRQHandler        /* 20: PMU                        */
    .long    PKC_IRQHandler        /* 21: PKC                        */
    .long    XQSPI_IRQHandler      /* 22: XQSPI                      */
    .long    QSPI1_IRQHandler      /* 23: QSPI1                      */
    .long    PWR_CMD_IRQHandler    /* 24: PWR_CMD                    */
    .long    BLESLP_IRQHandler     /* 25: BLE Sleep                  */
    .long    SLPTIMER_IRQHandler   /* 26: Sleep Timer                */
    .long    EXTWKUP_IRQHandler    /* 27: EXT Wakeup                 */
    .long    AON_WDT_IRQHandler    /* 28: AON_WDT                    */
    .long    I2S_M_IRQHandler      /* 29: I2S_M                      */
    .long    I2S_S_IRQHandler      /* 30: I2S_S                      */
    .long    ISO7816_IRQHandler    /* 31: ISO7816                    */
    .long    PRESENT_IRQHandler    /* 32: PRESENT                    */
    .long    CALENDAR_IRQHandler   /* 33: CALENDAR                   */
    .long    COMM_CORE_IRQHandler  /* 34: COMM_CORE                  */
    .long    DMA1_IRQHandler       /* 35: DMA1                       */
    .long    DMA2_IRQHandler       /* 36: DMA2                       */
    .long    DSPI_IRQHandler       /* 37: DSPI                       */
    .long    AON_IRQHandler        /* 38: AON                        */
    .long    PDM_IRQHandler        /* 39: PDM                        */
    .long    VTTBL_IRQHandler      /* 40: VTTBL                      */
    .long    CTE_FULL_IRQHandler   /* 41: CTE_FULL                   */
    .long    USB_IRQHandler        /* 42: USB                        */
    .long    GPADC_IRQHandler      /* 43: GPADC                      */
    .long    AON_PMU_BOD_FEDGE_IRQHandler /* 44: AON_PMU_BOD_FEDGE   */
    .long    AON_PMU_MSIO_COMP_IRQHandler /* 45: AON_PMU_MSIO_COMP   */
    .long    AON_PMU_USB_WKUP_IRQHandler  /* 46: AON_PMU_USB_WKUP    */

    .size       __Vectors, . - __Vectors

    .text
    .thumb
    .thumb_func
    .align      2
    .globl      Reset_Handler
    .type       Reset_Handler, %function
Reset_Handler:

    bl    SystemInit

    bl    main_init

    .pool
    .size    Reset_Handler, . - Reset_Handler

    .align    1
    .thumb_func
    .weak    Default_Handler
    .type    Default_Handler, %function
Default_Handler:
    b    .
    .size    Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_irq_handler    handler_name
    .weak    \handler_name
    .set    \handler_name, Default_Handler
    .endm

    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler

    def_irq_handler    WDT_IRQHandler
    def_irq_handler    DMA0_IRQHandler
    def_irq_handler    SPI_M_IRQHandler
    def_irq_handler    SPI_S_IRQHandler
    def_irq_handler    EXT0_IRQHandler
    def_irq_handler    EXT1_IRQHandler
    def_irq_handler    TIMER0_IRQHandler
    def_irq_handler    TIMER1_IRQHandler
    def_irq_handler    DUAL_TIMER_IRQHandler
    def_irq_handler    QSPI0_IRQHandler
    def_irq_handler    UART0_IRQHandler
    def_irq_handler    UART1_IRQHandler
    def_irq_handler    I2C0_IRQHandler
    def_irq_handler    I2C1_IRQHandler
    def_irq_handler    AES_IRQHandler
    def_irq_handler    HMAC_IRQHandler
    def_irq_handler    EXT2_IRQHandler
    def_irq_handler    RNG_IRQHandler
    def_irq_handler    PMU_IRQHandler
    def_irq_handler    PKC_IRQHandler
    def_irq_handler    XQSPI_IRQHandler
    def_irq_handler    QSPI1_IRQHandler
    def_irq_handler    PWR_CMD_IRQHandler
    def_irq_handler    SLPTIMER_IRQHandler
    def_irq_handler    EXTWKUP_IRQHandler
    def_irq_handler    AON_WDT_IRQHandler
    def_irq_handler    I2S_M_IRQHandler
    def_irq_handler    I2S_S_IRQHandler
    def_irq_handler    ISO7816_IRQHandler
    def_irq_handler    PRESENT_IRQHandler
    def_irq_handler    CALENDAR_IRQHandler
    def_irq_handler    COMM_CORE_IRQHandler
    def_irq_handler    DMA1_IRQHandler
    def_irq_handler    DMA2_IRQHandler
    def_irq_handler    DSPI_IRQHandler
    def_irq_handler    AON_IRQHandler
    def_irq_handler    PDM_IRQHandler
    def_irq_handler    VTTBL_IRQHandler
    def_irq_handler    CTE_FULL_IRQHandler
    def_irq_handler    USB_IRQHandler
    def_irq_handler    GPADC_IRQHandler
    def_irq_handler    AON_PMU_BOD_FEDGE_IRQHandler
    def_irq_handler    AON_PMU_MSIO_COMP_IRQHandler
    def_irq_handler    AON_PMU_USB_WKUP_IRQHandler
    .end

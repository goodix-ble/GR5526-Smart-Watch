/**************************************************************************//**
 * @file     gr552xx.h
 * @brief    CMSIS Cortex-M# Core Peripheral Access Layer Header File for
 *           Device gr552xx
 * @version  V1.00
 * @date     03. Feb 2020
 ******************************************************************************/
/*
 * Copyright (c) 2016-2020, Shenzhen Huiding Technology Co., Ltd
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

/** @addtogroup Device_Included
  * @{
  */

/** @addtogroup GR54xx
  * @{
  */

#ifndef __GR552xx_H__
#define __GR552xx_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */



/**
 * @brief GR55xx Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */

/* ================================================================================================================= */
/* ================                           Interrupt Number Definition                           ================ */
/* ================================================================================================================= */
typedef enum IRQn
{
/* ==================================  ARM Cortex-M# Specific Interrupt Numbers  =================================== */

    NonMaskableInt_IRQn       = -14,  /**< -14  Non maskable Interrupt, cannot be stopped or preempted               */
    HardFault_IRQn            = -13,  /**< -13  Hard Fault, all classes of Fault                                     */
    MemoryManagement_IRQn     = -12,  /**< -12  Memory Management, MPU mismatch, including Access Violation
                                                and No Match                                                         */
    BusFault_IRQn             = -11,  /**< -11  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                related Fault                                                        */
    UsageFault_IRQn           = -10,  /**< -10  Usage Fault, i.e. Undef Instruction, Illegal State Transition        */
    SVCall_IRQn               =  -5,  /**< -5 System Service Call via SVC instruction                                */
    DebugMonitor_IRQn         =  -4,  /**< -4 Debug Monitor                                                          */
    PendSV_IRQn               =  -2,  /**< -2 Pendable request for system service                                    */
    SysTick_IRQn              =  -1,  /**< -1 System Tick Timer                                                      */

/* ======================================  <Device> Specific Interrupt Numbers  ==================================== */
    WDT_IRQn                  =   0,  /**< Watchdog Timer Interrupt                                                  */
    BLE_SDK_IRQn              =   1,  /**< BLE_SDK_SCHEDULE Interrupt                                                */
    BLE_IRQn                  =   2,  /**< BLE Interrupt                                                             */
    DMA0_IRQn                 =   3,  /**< DMA Interrupt                                                             */
    SPI_M_IRQn                =   4,  /**< SPI_M Interrupt                                                           */
    SPI_S_IRQn                =   5,  /**< SPI_S Interrupt                                                           */
    EXT0_IRQn                 =   6,  /**< External 0 Interrupt                                                      */
    EXT1_IRQn                 =   7,  /**< External 1 Interrupt                                                      */
    TIMER0_IRQn               =   8,  /**< Timer0 Interrupt                                                          */
    TIMER1_IRQn               =   9,  /**< Timer1 Interrupt                                                          */
    DUAL_TIMER_IRQn           =  10,  /**< Dual_Timer Interrupt                                                      */
    QSPI0_IRQn                =  11,  /**< QSPI0 Interrupt                                                           */
    UART0_IRQn                =  12,  /**< UART0 Interrupt                                                           */
    UART1_IRQn                =  13,  /**< UART1 Interrupt                                                           */
    I2C0_IRQn                 =  14,  /**< I2C0 Interrupt                                                            */
    I2C1_IRQn                 =  15,  /**< I2C1 Interrupt                                                            */
    AES_IRQn                  =  16,  /**< AES Interrupt                                                             */
    HMAC_IRQn                 =  17,  /**< HMAC Interrupt                                                            */
    EXT2_IRQn                 =  18,  /**< External 2 Interrupt                                                      */
    RNG_IRQn                  =  19,  /**< RNG Interrupt                                                             */
    BOD_ASSERT_IRQn           =  20,  /**< BOD Down Interrupt                                                        */
    PKC_IRQn                  =  21,  /**< PKC Interrupt                                                             */
    XQSPI_IRQn                =  22,  /**< XQSPI Interrupt                                                           */
    QSPI1_IRQn                =  23,  /**< QSPI1 Interrupt                                                           */
    OSPI_IRQn                 =  24,  /**< OSPI Interrupt                                                            */
    BLESLP_IRQn               =  25,  /**< BLE Sleep Interrupt                                                       */
    SLPTIMER_IRQn             =  26,  /**< Sleep Timer Interrupt                                                     */
    AON_EXT_IRQn              =  27,  /**< External Wakeup Interrupt                                                 */
    AON_WDT_IRQn              =  28,  /**< Always on Watchdog Interrupt                                              */
    I2S_M_IRQn                =  29,  /**< I2S_M Interrupt                                                           */
    I2S_S_IRQn                =  30,  /**< I2S_S Interrupt                                                           */
    ISO7816_IRQn              =  31,  /**< ISO7816 Interrupt                                                         */
    PRESENT_IRQn              =  32,  /**< Presnet Done Interrupt                                                    */
    CALENDAR_IRQn             =  33,  /**< AON Calendar Timer alarm Interrupt                                        */
    COMM_CORE_IRQn            =  34,  /**< Comm Core AHB timeout Interrupt                                           */
    DMA1_IRQn                 =  35,  /**< DMA1 Interrupt                                                            */
    DMA2_IRQn                 =  36,  /**< DMA2 Interrupt                                                            */
    DSPI_IRQn                 =  37,  /**< DDPI Interrupt                                                            */
    AON_IRQn                  =  38,  /**< AON Interrupt                                                             */
    PDM_IRQn                  =  39,  /**< PDM Interrupt                                                             */
    VTTBL_IRQn                =  40,  /**< VTTBL Interrupt                                                           */
    CTE_FULL_IRQn             =  41,  /**< CTE_FULL Interrupt                                                        */
    USB_IRQn                  =  42,  /**< USB Interrupt                                                             */
    TSI_DC_IRQn               =  43,  /**< TSI DC Interrupt                                                          */
    BOD_DEASSERT_IRQn         =  44,  /**< BOD Up Interrupt                                                          */
    COMP_IRQn                 =  45,  /**< Comparator Interrupt                                                      */
    USB_ATTACH_IRQn           =  46,  /**< USB Attach Interrupt                                                      */
    USB_DETACH_IRQn           =  47,  /**< USB Detach Interrupt                                                      */
    RTC1_IRQn                 =  48,  /**< RTC1 Interrupt                                                            */
    CPLL_DRIFT_IRQn           =  49,  /**< CPLL DRIFT Interrupt                                                      */
    CLK_CALIB_IRQn            =  50,  /**< CLOCK Calibration Done Interrupt                                          */
    TSI_GPU_IRQn              =  51,  /**< TSI GPU Interrupt                                                         */
    UART2_IRQn                =  52,  /**< UART2 Interrupt                                                           */
    UART3_IRQn                =  53,  /**< UART3 Interrupt                                                           */
    UART4_IRQn                =  54,  /**< UART4 Interrupt                                                           */
    UART5_IRQn                =  55,  /**< UART5 Interrupt                                                           */
    BLE_PWR_ON_IRQn           =  56,  /**< AON BLE sequencer power on done Interrupt                                 */
    BLE_PWR_DN_IRQn           =  57,  /**< AON BLE sequencer power off done Interrupt                                */
    I2C2_IRQn                 =  58,  /**< I2C2 Interrupt                                                            */
    I2C3_IRQn                 =  59,  /**< I2C3 Interrupt                                                            */
    I2C4_IRQn                 =  60,  /**< I2C4 Interrupt                                                            */
    I2C5_IRQn                 =  61,  /**< I2C5 Interrupt                                                            */
    XO_BIAS_SW_CHG_IRQn       =  62,  /**< XO BIAS switch changed Interrupt                                          */
    DDVS_ERR_IRQn             =  63,  /**< DDVS ctrl Error Interrupt                                                  */
    TSI_GPU_ERR_IRQn          =  64,  /**< TSI GPU Error Interrupt                                                   */
    QSPI2_IRQn                =  65,  /**< QSPI2 Interrupt                                                           */
    MAX_NUMS_IRQn             =  66,  /**< Last Interrupt                                                            */
} IRQn_Type;

/** @} */ /* End of group Peripheral     _interrupt_number_definition */

/* ================================================================================================================= */
/* ================                        Processor and Core Peripheral Section                    ================ */
/* ================================================================================================================= */

/* ===================================  Start of section using anonymous unions  =================================== */

/* ======================  Configuration of the ARM Cortex-M4 Processor and Core Peripherals  ====================== */
#define __CM4_REV                 0x0001U   /* Core revision r0p1 */
#define __MPU_PRESENT             1         /* MPU present */
#define __VTOR_PRESENT            1         /* VTOR present */
#define __NVIC_PRIO_BITS          8         /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0         /* Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             1         /* FPU present */

#include "core_cm4.h"             /*      Cortex-M4 processor and core peripherals */
#include "system_gr55xx.h"        /*      System Header */
#include <stdint.h>

#if   defined (__CC_ARM)
    #pragma push
    #pragma anon_unions
#elif defined (__ICCARM__)
    #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wc11-extensions"
    #pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
    /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
    /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
    #pragma warning 586
#elif defined (__CSMC__)
    /* anonymous unions are enabled by default */
#else
    #warning Not supported compiler type
#endif


/* ================================================================================================================= */
/* ================                       Device Specific Peripheral Section                        ================ */
/* ================================================================================================================= */

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief AES
  */
/**
  * @brief AES
  */
typedef struct _aes_regs
{
    __IOM uint32_t CTRL;                      /**< AES Controller Register,                                              Address offset: 0x0000 */
    __IOM uint32_t CFG;                       /**< AES Configuration Register ,                                          Address offset: 0x0004 */
    __IOM uint32_t STAT;                      /**< AES Status Register  ,                                                Address offset: 0x0008 */
    __IOM uint32_t INT;                       /**< AES Interrupt Register ,                                              Address offset: 0x000C */
    __IOM uint32_t XFE_SIZE;                  /**< AES Transfer Size Register,                                           Address offset: 0x0010 */
    __IOM uint32_t RD_START_ADDR;             /**< AES Read Start Address Register,                                      Address offset: 0x0014 */
    __IOM uint32_t WR_START_ADDR;             /**< AES Write Start Address Register,                                     Address offset: 0x0018 */
    __IOM uint32_t KEY_ADDR;                  /**< AES Key Address Register ,                                            Address offset: 0x001C */
    __IOM uint32_t DATA_OUT0;                 /**< AES Data Output 0 Register,                                           Address offset: 0x0020 */
    __IOM uint32_t DATA_OUT1;                 /**< AES Data Output 1 Register ,                                          Address offset: 0x0024 */
    __IOM uint32_t DATA_OUT2;                 /**< AES Data Output 2 Register,                                           Address offset: 0x0028 */
    __IOM uint32_t DATA_OUT3;                 /**< AES Data Output 3 Register ,                                          Address offset: 0x002C */
    __OM  uint32_t KEY0;                      /**< AES Key 0 Register,                                                   Address offset: 0x0030 */
    __IOM uint32_t KEY1;                      /**< AES Key 1 Register,                                                   Address offset: 0x0034 */
    __IOM uint32_t KEY2;                      /**< AES Key 2 Register,                                                   Address offset: 0x0038 */
    __IOM uint32_t KEY3;                      /**< AES Key 3 Register,                                                   Address offset: 0x003C */
    __IOM uint32_t KEY4;                      /**< AES Key 4 Register,                                                   Address offset: 0x0040 */
    __IOM uint32_t KEY5;                      /**< AES Key 5 Register,                                                   Address offset: 0x0044 */
    __IOM uint32_t KEY6;                      /**< AES Key 6 Register,                                                   Address offset: 0x0048 */
    __IOM uint32_t KEY7;                      /**< AES Key 7 Register,                                                   Address offset: 0x004C */
    __OM  uint32_t INIT_SSI;                  /**< AES Sbox Initial Seed Input Register ,                                Address offset: 0x0050 */
    __IOM uint32_t INIT_SSO;                  /**< AES Sbox Initial Seed Output Register,                                Address offset: 0x0054 */
    __IOM uint32_t MASK_SSI;                  /**< AES Sbox Seed Input Mask Register ,                                   Address offset: 0x0058 */
    __IOM uint32_t MASK_SSO;                  /**< AES Sbox Seed Input Mask Register ,                                   Address offset: 0x005C */
    __OM  uint32_t INIT_V0;                   /**< AES Initialization Vector 0 Register ,                                Address offset: 0x0060 */
    __IOM uint32_t INIT_V1;                   /**< AES Initialization Vector 1 Register ,                                Address offset: 0x0064 */
    __IOM uint32_t INIT_V2;                   /**< AES Initialization Vector 2 Register ,                                Address offset: 0x0068 */
    __IOM uint32_t INIT_V3;                   /**< AES Initialization Vector 3 Register ,                                Address offset: 0x006C */
    __IOM uint32_t DATA_IN0;                  /**< AES Data Input 0 Register ,                                           Address offset: 0x0070 */
    __IOM uint32_t DATA_IN1;                  /**< AES Data Input 1 Register ,                                           Address offset: 0x0074 */
    __IOM uint32_t DATA_IN2;                  /**< AES Data Input 2 Register ,                                           Address offset: 0x0078 */
    __IOM uint32_t DATA_IN3;                  /**< AES Data Input 3 Register ,                                           Address offset: 0x007C */
    __IOM uint32_t KEYPORT_MASK;              /**< AES Keyport Mask Register ,                                           Address offset: 0x0080 */
} aes_regs_t;


/**
  * @brief DMA
  */
#define DMA_REG(name)                   __IOM uint32_t name; __IOM uint32_t __pad_##name
/* DMA/Channel_x_Registers Registers */
typedef struct
{
    DMA_REG(SAR);                       /**< Source Address,                Address offset: 0x00 */
    DMA_REG(DAR);                       /**< Destination Address,           Address offset: 0x08 */
    DMA_REG(LLP);                       /**< Linked List Pointer,           Address offset: 0x10 */
    __IOM uint32_t CTL_LO;              /**< Control Register Low,          Address offset: 0x18 */
    __IOM uint32_t CTL_HI;              /**< Control Register High,         Address offset: 0x1C */
    DMA_REG(SSTAT);                     /**< Source Status,                 Address offset: 0x20 */
    DMA_REG(DSTAT);                     /**< Destination Status,            Address offset: 0x28 */
    DMA_REG(SSTATAR);                   /**< Source Status Address,         Address offset: 0x30 */
    DMA_REG(DSTATAR);                   /**< Destination Status Address,    Address offset: 0x38 */
    __IOM uint32_t CFG_LO;              /**< Configuration Register Low,    Address offset: 0x40 */
    __IOM uint32_t CFG_HI;              /**< Configuration Register High,   Address offset: 0x44 */
    DMA_REG(SGR);                       /**< Source Gather,                 Address offset: 0x48 */
    DMA_REG(DSR);                       /**< Destination Scatter,           Address offset: 0x50 */
} DMA_CH_REGS;

/* DMA/Interrupt_Registers Registers */
typedef struct
{
    __IO uint32_t RAW_CH_EVT[10];       /**< Raw channel event,             Address offset: 0x00 */
    __I  uint32_t STATUS_CH_EVT[10];    /**< Status channel event,          Address offset: 0x28 */
    __IO uint32_t MASK_CH_EVT[10];      /**< Mask channel event,            Address offset: 0x50 */
    __O  uint32_t CLEAR_CH_EVT[10];     /**< Clear channel event,           Address offset: 0x78 */
    DMA_REG(STATUS_EVT);                /**< Status event,                  Address offset: 0xA0 */
} DMA_INT_REGS;

/* DMA/Software_Handshake_Registers Registers */
typedef struct
{
    DMA_REG(REQ_SRC);                   /**< Source Transaction Request,            Address offset: 0x00 */
    DMA_REG(REQ_DST);                   /**< Destination Transaction Request,       Address offset: 0x08 */
    DMA_REG(SGL_RQ_SRC);                /**< Source Single Transaction Request,     Address offset: 0x20 */
    DMA_REG(SGL_RQ_DST);                /**< Destination Single Transaction Request,Address offset: 0x28 */
    DMA_REG(LST_SRC);                   /**< Source Last Transaction Request,       Address offset: 0x30 */
    DMA_REG(LST_DST);                   /**< Destination Last Transaction Request,  Address offset: 0x38 */
} DMA_HS_REGS;

/* DMA/Miscellaneous_Registers Registers */
typedef struct
{
    DMA_REG(CFG);                       /**< DMA Configuration,             Address offset: 0x00 */
    DMA_REG(CH_EN);                     /**< DMA Channel Enable,            Address offset: 0x08 */
    DMA_REG(ID);                        /**< DMA ID,                        Address offset: 0x20 */
    DMA_REG(TEST);                      /**< DMA Test,                      Address offset: 0x28 */
    DMA_REG(LP_TIMEOUT);                /**< DMA Low Power Timeout,         Address offset: 0x30 */
    DMA_REG(RESERVED);                  /**< Reserved,                      Address offset: 0x38 */
    DMA_REG(COMP_PARAMS_6);             /**< DMA Component Parameters 6,    Address offset: 0x40 */
    DMA_REG(COMP_PARAMS_5);             /**< DMA Component Parameters 5,    Address offset: 0x48 */
    DMA_REG(COMP_PARAMS_4);             /**< DMA Component Parameters 4,    Address offset: 0x50 */
    DMA_REG(COMP_PARAMS_3);             /**< DMA Component Parameters 3,    Address offset: 0x58 */
    DMA_REG(COMP_PARAMS_2);             /**< DMA Component Parameters 2,    Address offset: 0x60 */
    DMA_REG(COMP_PARAMS_1);             /**< DMA Component Parameters 1,    Address offset: 0x68 */
    DMA_REG(COMPS_ID);                  /**< DMA Component ID,              Address offset: 0x70 */
} DMA_MISC_REGS;

typedef struct _dma_regs
{
    DMA_CH_REGS         CHANNEL[8];     /**< DMA_REG_CH register,           Address offset: 0x000 */
    DMA_INT_REGS        EVENT;          /**< DMA_REG_INT register,          Address offset: 0x2C0 */
    DMA_HS_REGS         HANDSHAKE;      /**< DMA_REG_HS register,           Address offset: 0x368 */
    DMA_MISC_REGS       MISCELLANEOU;   /**< DMA_REG_MISC register,         Address offset: 0x3A8 */
} dma_regs_t;

/**
  * @brief DUAL_TIM
  */
typedef struct _dual_timer_regs
{
    __IOM uint32_t RELOAD;          /**< DUAL_TIM auto-reload register,            Address offset: 0x00 */
    __IM  uint32_t VALUE;           /**< DUAL_TIM counter value register,          Address offset: 0x04 */
    __IOM uint32_t CTRL;            /**< DUAL_TIM control register,                Address offset: 0x08 */
    __OM  uint32_t INTCLR;          /**< DUAL_TIM interrupt status clear register, Address offset: 0x0C */
    __IM  uint32_t RAW_INTSTAT;     /**< DUAL_TIM raw interrupt status register,   Address offset: 0x10 */
    __IM  uint32_t INTSTAT;         /**< DUAL_TIM interrupt status register,       Address offset: 0x14 */
    __IOM uint32_t BG_LOAD;         /**< DUAL_TIM background-reload register,      Address offset: 0x18 */
} dual_timer_regs_t;

/**
  * @brief GPIO
  */
typedef struct _gpio_regs
{
    __IOM uint32_t DATA;                /**< GPIO_REG_DATA register,            Address offset: 0x000 */
    __IOM uint32_t DATAOUT;             /**< GPIO_REG_DATAOUT register,         Address offset: 0x004 */
    __IM  uint32_t RESERVED0[2];        /**< GPIO_REG_RESERVED register,        Address offset: 0x008 */
    __IOM uint32_t OUTENSET;            /**< GPIO_REG_OUTENSET register,        Address offset: 0x010 */
    __IOM uint32_t OUTENCLR;            /**< GPIO_REG_OUTENCLR register,        Address offset: 0x014 */
    __IOM uint32_t ALTFUNCSET;          /**< GPIO_REG_ALTFUNCSET register,      Address offset: 0x018 */
    __IOM uint32_t ALTFUNCCLR;          /**< GPIO_REG_ALTFUNCCLR register,      Address offset: 0x01C */
    __IOM uint32_t INTENSET;            /**< GPIO_REG_INTENSET register,        Address offset: 0x020 */
    __IOM uint32_t INTENCLR;            /**< GPIO_REG_INTENCLR register,        Address offset: 0x024 */
    __IOM uint32_t INTTYPESET;          /**< GPIO_REG_INTTYPESET register,      Address offset: 0x028 */
    __IOM uint32_t INTTYPECLR;          /**< GPIO_REG_INTTYPECLR register,      Address offset: 0x02C */
    __IOM uint32_t INTPOLSET;           /**< GPIO_REG_INTPOLSET register,       Address offset: 0x030 */
    __IOM uint32_t INTPOLCLR;           /**< GPIO_REG_INTPOLCLR register,       Address offset: 0x034 */
    __IOM uint32_t INTSTAT;             /**< GPIO_REG_INTSTAT register,         Address offset: 0x038 */
    __IM  uint32_t RESERVED1;           /**< GPIO_REG_RESERVED register,        Address offset: 0x03C */
    __IOM uint32_t INTDBESET;           /**< GPIO_REG_INTDBESET register,       Address offset: 0x040 */
    __IOM uint32_t INTDBECLR;           /**< GPIO_REG_INTDBECLR register,       Address offset: 0x044 */
    __IM  uint32_t RESERVED2[238];      /**< GPIO_REG_RESERVED register,        Address offset: 0x048 */
    __IOM uint32_t MASKLOWBYTE[256];    /**< GPIO_REG_MASKLOWBYTE register,     Address offset: 0x400 */
    __IOM uint32_t MASKHIGHBYTE[256];   /**< GPIO_REG_MASKHIGHBYTE register,    Address offset: 0x500 */
} gpio_regs_t;

/**
  * @brief HMAC
  */
typedef struct _hmac_regs
{
    __IOM uint32_t CTRL;                      /**< HMAC Control Register,                                                Address offset: 0x0000 */
    __IOM uint32_t CFG;                       /**< HMAC Configuration Register,                                          Address offset: 0x0004 */
    __IOM uint32_t STAT;                      /**< HMAC Status Register,                                                 Address offset: 0x0008 */
    __IOM uint32_t XFE_SIZE;                  /**< HMAC Transfer Size Register,                                          Address offset: 0x000C */
    __IOM uint32_t INT;                       /**< HMAC Interrupt Register,                                              Address offset: 0x0010 */
    __IOM uint32_t RD_START_ADDR;             /**< HMAC Read Start Address Register,                                     Address offset: 0x0014 */
    __IOM uint32_t WR_START_ADDR;             /**< HMAC Write Start Address Register,                                    Address offset: 0x0018 */
    __IOM uint32_t RESERVED0[1];              /**< RESERVED,                                                             Address offset: 0x001C */
    __IOM uint32_t USER_HASH_0;               /**< HMAC User Hash 0 Register,                                            Address offset: 0x0020 */
    __IOM uint32_t USER_HASH_1;               /**< HMAC User Hash 1 Register,                                            Address offset: 0x0024 */
    __IOM uint32_t USER_HASH_2;               /**< HMAC User Hash 2 Register,                                            Address offset: 0x0028 */
    __IOM uint32_t USER_HASH_3;               /**< HMAC User Hash 3 Register,                                            Address offset: 0x002C */
    __IOM uint32_t USER_HASH_4;               /**< HMAC User Hash 4 Register,                                            Address offset: 0x0030 */
    __IOM uint32_t USER_HASH_5;               /**< HMAC User Hash 5 Register,                                            Address offset: 0x0034 */
    __IOM uint32_t USER_HASH_6;               /**< HMAC User Hash 6 Register,                                            Address offset: 0x0038 */
    __IOM uint32_t USER_HASH_7;               /**< HMAC User Hash 7 Register,                                            Address offset: 0x003C */
    __IOM uint32_t DATA_OUT;                  /**< HMAC Data Output Register,                                            Address offset: 0x0040 */
    __OM  uint32_t DATA_IN;                   /**< HMAC Data Input Register,                                             Address offset: 0x0044 */
    __IOM uint32_t KEY0;                      /**< HMAC Key 0 Register,                                                  Address offset: 0x0048 */
    __IOM uint32_t KEY1;                      /**< HMAC Key 1 Register,                                                  Address offset: 0x004C */
    __IOM uint32_t KEY2;                      /**< HMAC Key 2 Register,                                                  Address offset: 0x0050 */
    __IOM uint32_t KEY3;                      /**< HMAC Key 3 Register,                                                  Address offset: 0x0054 */
    __IOM uint32_t KEY4;                      /**< HMAC Key 4 Register,                                                  Address offset: 0x0058 */
    __IOM uint32_t KEY5;                      /**< HMAC Key 5 Register,                                                  Address offset: 0x005C */
    __IOM uint32_t KEY6;                      /**< HMAC Key 6 Register,                                                  Address offset: 0x0060 */
    __IOM uint32_t KEY7;                      /**< HMAC Key 7 Register,                                                  Address offset: 0x0064 */
    __OM  uint32_t KEY_ADDR;                  /**< HMAC Key Addr Register,                                               Address offset: 0x0068 */
    __OM  uint32_t KEYPORT_MASK;              /**< HMAC Keyport Mask Register,                                           Address offset: 0x006C */
} hmac_regs_t;

/**
  * @brief I2C
  */
typedef struct _i2c_regs
{
    __IOM uint32_t CTRL;                      /**< I2C Control Register,                                                 Address offset: 0x0000 */
    __IOM uint32_t TARGET_ADDR;               /**< I2C Target Address Register,                                          Address offset: 0x0004 */
    __IOM uint32_t S_ADDR;                    /**< I2C Slave Address Register,                                           Address offset: 0x0008 */
    __IOM uint32_t M_HS_ADDR;                 /**< I2C High-Speed Master Mode Code Address Register,                     Address offset: 0x000C */
    __IOM uint32_t DATA_CMD;                  /**< I2C RX/TX Data Buffer and Command Register,                           Address offset: 0x0010 */
    __IOM uint32_t SS_CLK_HCOUNT;             /**< Standard Speed I2C Clock SCL High Count Register,                     Address offset: 0x0014 */
    __IOM uint32_t SS_CLK_LCOUNT;             /**< Standard Speed I2C Clock SCL Low Count Register,                      Address offset: 0x0018 */
    __IOM uint32_t FS_CLK_HCOUNT;             /**< Fast Mode or Fast Mode Plus I2C Clock SCL High Count Register,        Address offset: 0x001C */
    __IOM uint32_t FS_CLK_LCOUNT;             /**< Fast Mode or Fast Mode Plus I2C Clock SCL Low Count Register,         Address offset: 0x0020 */
    __IOM uint32_t HS_CLK_HCOUNT;             /**< High Speed I2C Clock SCL High Count Register,                         Address offset: 0x0024 */
    __IOM uint32_t HS_CLK_LCOUNT;             /**< High Speed I2C Clock SCL Low Count Register,                          Address offset: 0x0028 */
    __IOM uint32_t INT_STAT;                  /**< I2C Interrupt Status Register,                                        Address offset: 0x002C */
    __IOM uint32_t INT_MASK;                  /**< I2C Interrupt Mask Register,                                          Address offset: 0x0030 */
    __IOM uint32_t RAW_INT_STAT;              /**< I2C Raw Interrupt Status Register,                                    Address offset: 0x0034 */
    __IOM uint32_t RX_FIFO_THD;               /**< I2C Receive FIFO Threshold Register,                                  Address offset: 0x0038 */
    __IOM uint32_t TX_FIFO_THD;               /**< I2C Transmit FIFO Threshold Register,                                 Address offset: 0x003C */
    __IOM uint32_t CLR_INT;                   /**< Clear Combined and Individual Interrupt Register,                     Address offset: 0x0040 */
    __IOM uint32_t CLR_RX_UNDER;              /**< Clear RX_UNDER Interrupt Register,                                    Address offset: 0x0044 */
    __IOM uint32_t CLR_RX_OVER;               /**< Clear RX_OVER Interrupt Register,                                     Address offset: 0x0048 */
    __IOM uint32_t CLR_TX_OVER;               /**< Clear TX_OVER Interrupt Register,                                     Address offset: 0x004C */
    __IOM uint32_t CLR_RD_REQ;                /**< Clear RD_REQ Interrupt Register,                                      Address offset: 0x0050 */
    __IOM uint32_t CLR_TX_ABORT;              /**< Clear TX_ABORT Interrupt Register,                                    Address offset: 0x0054 */
    __IOM uint32_t CLR_RX_DONE;               /**< Clear RX_DONE Interrupt Register,                                     Address offset: 0x0058 */
    __IOM uint32_t CLR_ACTIVITY;              /**< Clear ACTIVITY Interrupt Register,                                    Address offset: 0x005C */
    __IOM uint32_t CLR_STOP_DET;              /**< Clear STOP_DET Interrupt Register,                                    Address offset: 0x0060 */
    __IOM uint32_t CLR_START_DET;             /**< Clear START_DET Interrupt Register,                                   Address offset: 0x0064 */
    __IOM uint32_t CLR_GEN_CALL;              /**< Clear GEN_CALL Interrupt Register,                                    Address offset: 0x0068 */
    __IOM uint32_t EN;                        /**< I2C ENABLE Register,                                                  Address offset: 0x006C */
    __IOM uint32_t STAT;                      /**< I2C STATUS Register,                                                  Address offset: 0x0070 */
    __IOM uint32_t TX_FIFO_LEVEL;             /**< I2C Transmit FIFO Level Register,                                     Address offset: 0x0074 */
    __IOM uint32_t RX_FIFO_LEVEL;             /**< I2C Receive FIFO Level Register,                                      Address offset: 0x0078 */
    __IOM uint32_t SDA_HOLD;                  /**< I2C SDA Hold Time Length Register,                                    Address offset: 0x007C */
    __IOM uint32_t TX_ABORT_SRC;              /**< I2C SDA Hold Time Length Register,                                    Address offset: 0x0080 */
    __IOM uint32_t RESERVED0[1];              /**< RESERVED,                                                             Address offset: 0x0084 */
    __IOM uint32_t DMA_CTRL;                  /**< DMA Control Register,                                                 Address offset: 0x0088 */
    __IOM uint32_t DMA_TX_LEVEL;              /**< DMA Transmit Data Level Register,                                     Address offset: 0x008C */
    __IOM uint32_t DMA_RX_LEVEL;              /**< DMA Receive Data Level Register,                                      Address offset: 0x0090 */
    __IOM uint32_t SDA_SETUP;                 /**< I2C SDA Setup Register,                                               Address offset: 0x0094 */
    __IOM uint32_t ACK_GEN_CALL;              /**< I2C ACK General Call Register,                                        Address offset: 0x0098 */
    __IOM uint32_t EN_STAT;                   /**< I2C Enable Status Register,                                           Address offset: 0x009C */
    __IOM uint32_t FS_SPKLEN;                 /**< I2C SS, FS or FM+ spike suppression limit,                            Address offset: 0x00A0 */
    __IOM uint32_t HS_SPKLEN;                 /**< I2C HS spike suppression limit Register,                              Address offset: 0x00A4 */
    __IOM uint32_t RESERVED1[1];              /**< RESERVED,                                                             Address offset: 0x00A8 */
    __IOM uint32_t SCL_STUCK_TIMEOUT;         /**< I2C SCL Stuck at Low Timeout Register,                                Address offset: 0x00AC */
    __IOM uint32_t SDA_STUCK_TIMEOUT;         /**< I2C SDA Stuck at Low Timeout Register,                                Address offset: 0x00B0 */
    __IOM uint32_t CLR_SCL_STUCK_DET;         /**< Clear SCL Stuck at Low Detect interrupt Register,                     Address offset: 0x00B4 */
} i2c_regs_t;

/**
  * @brief I2S
  */
typedef struct _i2s_regs
{
    __IOM uint32_t EN;                        /**< I2S Enable Register,                                                  Address offset: 0x0000 */
    __IOM uint32_t RX_EN;                     /**< I2S Receiver Block Enable Register,                                   Address offset: 0x0004 */
    __IOM uint32_t TX_EN;                     /**< I2S Transmitter Block Enable Register ,                               Address offset: 0x0008 */
    __IOM uint32_t CLK_EN;                    /**< Clock Enable Register ,                                               Address offset: 0x000C */
    __IOM uint32_t SCLK_CFG;                  /**< Clock Configuration Register,                                         Address offset: 0x0010 */
    __OM  uint32_t RX_FIFO_RST;               /**< Receiver Block FIFO Reset Register,                                   Address offset: 0x0014 */
    __IOM uint32_t TX_FIFO_RST;               /**< Transmitter Block FIFO Reset Register,                                Address offset: 0x0018 */
    __IOM uint32_t RESERVED0[1];              /**< RESERVED,                                                             Address offset: 0x001C */
    __OM  uint32_t LEFT_BUF;                  /**< Left Buffer Register ,                                                Address offset: 0x0020 */
    __OM  uint32_t RIGHT_BUF;                 /**< Right Buffer Register ,                                               Address offset: 0x0024 */
    __IOM uint32_t RX_CH_EN;                  /**< Receive Enable Register,                                              Address offset: 0x0028 */
    __IOM uint32_t TX_CH_EN;                  /**< Transmit Enable Register,                                             Address offset: 0x002C */
    __IOM uint32_t RX_CFG;                    /**< Receive Configuration Register,                                       Address offset: 0x0030 */
    __IOM uint32_t TX_CFG;                    /**< Transmit Configuration Register ,                                     Address offset: 0x0034 */
    __IOM uint32_t INT_STAT;                  /**< Interrupt status Register ,                                           Address offset: 0x0038 */
    __IOM uint32_t INT_MASK;                  /**< Interrupt Mask Register,                                              Address offset: 0x003C */
    __IOM uint32_t RX_OVER;                   /**< Receive Overrun Register,                                             Address offset: 0x0040 */
    __IOM uint32_t TX_OVER;                   /**< Transmit Overrun Register,                                            Address offset: 0x0044 */
    __IOM uint32_t RX_FIFO_CFG;               /**< Receive FIFO Configuration Register ,                                 Address offset: 0x0048 */
    __IOM uint32_t TX_FIFO_CFG;               /**< Transmit FIFO Configuration Register,                                 Address offset: 0x004C */
    __OM  uint32_t RX_FIFO_FLUSH;             /**< Receive FIFO Flush Register ,                                         Address offset: 0x0050 */
    __IOM uint32_t TX_FIFO_FLUSH;             /**< Transmit FIFO Flush Register ,                                        Address offset: 0x0054 */
    __IOM uint32_t RESERVED1[90];             /**< RESERVED,                                                             Address offset: 0x0058 */
    __OM  uint32_t RX_DMA;                    /**< Receiver Block DMA Register ,                                         Address offset: 0x01C0 */
    __OM  uint32_t RST_RX_DMA;                /**< Reset Receiver Block DMA Register,                                    Address offset: 0x01C4 */
    __OM  uint32_t TX_DMA;                    /**< Transmitter Block DMA Register,                                       Address offset: 0x01C8 */
    __OM  uint32_t RST_TX_DMA;                /**< Reset Transmitter Block DMA Register,                                 Address offset: 0x01CC */
} i2s_regs_t;

/**
  * @brief ISO7816
  */
typedef struct _iso7816_regs
{
    __IOM uint32_t CTRL;                      /**< Control Register,                                                     Address offset: 0x0000 */
    __OM  uint32_t STAT;                      /**< Status Register,                                                      Address offset: 0x0004 */
    __IOM uint32_t CLK_CFG;                   /**< Clock Configuration Register ,                                        Address offset: 0x0008 */
    __IOM uint32_t RESERVED0[1];              /**< RESERVED,                                                             Address offset: 0x000C */
    __IOM uint32_t TIMES_CFG;                 /**< Times Configuration Register ,                                        Address offset: 0x0010 */
    __IOM uint32_t DATA_CFG;                  /**< Data Configuration Register ,                                         Address offset: 0x0014 */
    __IOM uint32_t ADDR;                      /**< Address Register ,                                                    Address offset: 0x0018 */
    __IOM uint32_t START_ADDR;                /**< Start Address Register ,                                              Address offset: 0x001C */
    __IOM uint32_t RX_END_ADDR;               /**< RX End Address Register ,                                             Address offset: 0x0020 */
    __IOM uint32_t TX_END_ADDR;               /**< TX End Address Register ,                                             Address offset: 0x0024 */
} iso7816_regs_t;


/**
  * @brief MCU_SUB
  */
typedef struct _mcu_sub_regs
{
    __IOM uint32_t SENSE_ADC_FIFO;            /**< Sense ADC Read FIFO Register,                                         Address offset: 0x0000 */
    __IOM uint32_t SENSE_FF_THRESH;           /**< Sense ADC FIFO Threshold Register,                                    Address offset: 0x0004 */
    __IOM uint32_t SENSE_ADC_STAT;            /**< Sense ADC Status Register,                                            Address offset: 0x0008 */
    __IOM uint32_t SENSE_ADC_CLK;             /**< Sense ADC Clock Register,                                             Address offset: 0x000C */
    __IOM uint32_t RESERVED0[1];              /**< RESERVED,                                                             Address offset: 0x0010 */
    __IOM uint32_t SENSE_ADC_GET_TKN_HW;      /**< Sense ADC get token for Hardware Register,                            Address offset: 0x0014 */
    __IOM uint32_t SENSE_ADC_GET_TKN_SW;      /**< Sense ADC get token for software Register,                            Address offset: 0x0018 */
    __IOM uint32_t SENSE_ADC_RET_TKN_HW;      /**< Sense ADC release the HW token Register,                              Address offset: 0x001C */
    __OM  uint32_t SENSE_ADC_RET_TKN_SW;      /**< Sense ADC release the SW token Register,                              Address offset: 0x0020 */
    __IOM uint32_t SENSE_ADC_TKN_STS;         /**< Sense ADC Token Status Register,                                      Address offset: 0x0024 */
    __IOM uint32_t RESERVED1[6];              /**< RESERVED,                                                             Address offset: 0x0028 */
    __IOM uint32_t CTE_FIFO_DATA;             /**< CTE FIFO data Register,                                               Address offset: 0x0040 */
    __IOM uint32_t CTE_FIFO_THRESH;           /**< CTE FIFO threshold Register,                                          Address offset: 0x0044 */
    __IOM uint32_t CTE_FIFO_STAT;             /**< CTE FIFO status Register,                                             Address offset: 0x0048 */
    __IOM uint32_t CTE_CFG;                   /**< CTE Configure Register,                                               Address offset: 0x004C */
    __IOM uint32_t PSRAM_MEM_CLK;             /**< PSRAM memory clock Register,                                          Address offset: 0x0050 */
    __IOM uint32_t RESERVED2[59];             /**< RESERVED,                                                             Address offset: 0x0054 */
    __IOM uint32_t USB_LP_CTRL;               /**< USB low power control Register ,                                      Address offset: 0x0140 */
    __IOM uint32_t USB_XCRV_CTRL;             /**< USB Transceiver control Register ,                                    Address offset: 0x0144 */
    __IOM uint32_t USB_XCRV_LDO;              /**< USB Transceiver LDO control Register ,                                Address offset: 0x0148 */
    __IOM uint32_t USB_SW_RST;                /**< USB reset Register ,                                                  Address offset: 0x014C */
    __IOM uint32_t RESERVED3[12];             /**< RESERVED,                                                             Address offset: 0x0150 */
    __IOM uint32_t QSPI_M_XIP;                /**< QSPI Master 0-2 XIP Mode Control,                                     Address offset: 0x0180 */
    __IOM uint32_t QSPI_M_HRESP_DBG;          /**< AHB Response Error Debug for QSPI,                                    Address offset: 0x0184 */
    __IOM uint32_t QSPI_M_CS_SETUP_DLY;       /**< QSPI CS Setup DELAY control Register,                                 Address offset: 0x0188 */
    __IOM uint32_t QSPI_M_CS_RELEASE_DLY;     /**< QSPI Release DELAY control Register,                                  Address offset: 0x018C */
    __IOM uint32_t RESERVED4[12];             /**< RESERVED,                                                             Address offset: 0x0188 */
    __IOM uint32_t DMA0_HS_SEL;               /**< DMA0 Handshake configuration Register,                                Address offset: 0x01C0 */
    __IOM uint32_t DMA1_HS_SEL;               /**< DMA1 Handshake configuration Register,                                Address offset: 0x01C4 */
    __IOM uint32_t RESERVED5[22];             /**< RESERVED,                                                             Address offset: 0x01C8 */
    __IOM uint32_t PAD_IE_N_AUTO;             /**< PAD_IE_N_AUTO,                                                        Address offset: 0x0220 */
    __IOM uint32_t BLE_FERP_CTL;              /**< BLE ferp control register,                                            Address offset: 0x0224 */
    __IOM uint32_t RESERVED6[1];              /**< RESERVED,                                                             Address offset: 0x0228 */
    __IOM uint32_t SECURITY_RESET;            /**< Sercurity block reset control register,                               Address offset: 0x022C */
    __IOM uint32_t PMU_ID;                    /**< analog timing control Register ,                                      Address offset: 0x0230 */
    __IOM uint32_t PWR_AVG_CTL_REG0;          /**< power average block control registers,                                Address offset: 0x0234 */
    __IOM uint32_t RESERVED7[7];              /**< RESERVED,                                                             Address offset: 0x0238 */
    __IOM uint32_t EFUSE_PWR_DELTA_0;         /**< Efuse timing pararmeter register 0.,                                  Address offset: 0x0254 */
    __IOM uint32_t EFUSE_PWR_DELTA_1;         /**< Efuse timing pararmeter register 1.,                                  Address offset: 0x0258 */
    __IOM uint32_t RESERVED8[1];              /**< RESERVED,                                                             Address offset: 0x025C */
    __IOM uint32_t EFUSE_PWR_CTRL_0;          /**< Efuse power controller register 0,                                    Address offset: 0x0260 */
    __IOM uint32_t EFUSE_PWR_CTRL_1;          /**< Efuse power controller register 1,                                    Address offset: 0x0264 */
    __IOM uint32_t I2S_CLK_CFG;               /**< I2S Clock Configure Register,                                         Address offset: 0x0268 */
    __IOM uint32_t I2S_DMA_MODE;              /**< I2S DMA mode register,                                                Address offset: 0x026C */
    __IOM uint32_t RESERVED9[4];              /**< RESERVED,                                                             Address offset: 0x0270 */
    __IOM uint32_t MCU_SUB_REG;               /**< MCU SUB register,                                                     Address offset: 0x0280 */
    __IOM uint32_t MCU_NMI_CFG;               /**< MCU NMI configure register,                                           Address offset: 0x0284 */
    __IOM uint32_t CPLL_IRQ_CFG;              /**< CPLL DRIFT IRQ configure register,                                    Address offset: 0x0288 */
    __IOM uint32_t AON_SW_RST;                /**< AON domain reset register,                                            Address offset: 0x028C */
    __IOM uint32_t RESERVED10[44];            /**< RESERVED,                                                             Address offset: 0x0288 */
    __IOM uint32_t APB_TIMER_DBG;             /**< MCU APB Timer debug register,                                         Address offset: 0x0340 */
    __IOM uint32_t APB_MON_DBG;               /**< bypass the bus monitor in AHB-APB bridge register,                    Address offset: 0x0344 */
    __IOM uint32_t RESERVED11[14];            /**< RESERVED,                                                             Address offset: 0x0348 */
    __IOM uint32_t MCU_RELEASE;               /**< MCU release register,                                                 Address offset: 0x0380 */
    __IOM uint32_t FPGA_CTRL;                 /**< FPGA control register,                                                Address offset: 0x0384 */
    __IOM uint32_t ST_CALIB;                  /**< ST_CALIB register,                                                    Address offset: 0x0388 */
    __IOM uint32_t RESERVED12[1];             /**< RESERVED,                                                             Address offset: 0x038C */
    __IOM uint32_t GPU_DBG;                   /**< GPU_DBG register,                                                     Address offset: 0x0390 */
} mcu_sub_regs_t;

/**
  * @brief PKC
  */
typedef struct _pkc_regs
{
    __IOM uint32_t CTRL;                      /**< PKC Controller Register ,                                             Address offset: 0x0000 */
    __IOM uint32_t CFG0;                      /**< PKC Configuration 0 Register ,                                        Address offset: 0x0004 */
    __IOM uint32_t CFG1;                      /**< PKC Configuration 1 Register ,                                        Address offset: 0x0008 */
    __IOM uint32_t CFG2;                      /**< PKC Configuration 2 Register ,                                        Address offset: 0x000C */
    __IOM uint32_t CFG3;                      /**< PKC Configuration 3 Register ,                                        Address offset: 0x0010 */
    __IOM uint32_t CFG4;                      /**< PKC Configuration 4 Register ,                                        Address offset: 0x0014 */
    __IOM uint32_t CFG5;                      /**< PKC Configuration 5 Register ,                                        Address offset: 0x0018 */
    __IOM uint32_t CFG6;                      /**< PKC Configuration 6 Register ,                                        Address offset: 0x001C */
    __IOM uint32_t CFG7;                      /**< PKC Configuration 7 Register ,                                        Address offset: 0x0020 */
    __IOM uint32_t CFG8;                      /**< PKC Configuration 8 Register ,                                        Address offset: 0x0024 */
    __IOM uint32_t CFG9;                      /**< PKC Configuration 9 Register ,                                        Address offset: 0x0028 */
    __IOM uint32_t CFG10;                     /**< PKC Configuration 10 Register ,                                       Address offset: 0x002C */
    __IOM uint32_t CFG11;                     /**< PKC Configuration 11 Register ,                                       Address offset: 0x0030 */
    __IOM uint32_t CFG12;                     /**< PKC Configuration 12 Register ,                                       Address offset: 0x0034 */
    __IOM uint32_t CFG13;                     /**< PKC Configuration 13 Register ,                                       Address offset: 0x0038 */
    __IOM uint32_t RESERVED0[1];              /**< RESERVED,                                                             Address offset: 0x003C */
    __IOM uint32_t SW_CTRL;                   /**< PKC Software Controller Register ,                                    Address offset: 0x0040 */
    __IOM uint32_t SW_CFG0;                   /**< PKC Software Configuration 0 Register ,                               Address offset: 0x0044 */
    __IOM uint32_t SW_CFG1;                   /**< PKC Software Configuration 1 Register ,                               Address offset: 0x0048 */
    __IOM uint32_t SW_CFG2;                   /**< PKC Software Configuration 2 Register ,                               Address offset: 0x004C */
    __IOM uint32_t SW_CFG3;                   /**< PKC Software Configuration 3 Register ,                               Address offset: 0x0050 */
    __IOM uint32_t SW_CFG4;                   /**< PKC Software Configuration 4 Register ,                               Address offset: 0x0054 */
    __IOM uint32_t SW_CFG5;                   /**< PKC Software Configuration 5 Register ,                               Address offset: 0x0058 */
    __IOM uint32_t SW_CFG6;                   /**< PKC Software Configuration 6 Register ,                               Address offset: 0x005C */
    __IOM uint32_t SW_CFG7;                   /**< PKC Software Configuration 7 Register ,                               Address offset: 0x0060 */
    __IOM uint32_t SW_CFG8;                   /**< PKC Software Configuration 8 Register ,                               Address offset: 0x0064 */
    __IOM uint32_t SW_CFG9;                   /**< PKC Software Configuration 9 Register ,                               Address offset: 0x0068 */
    __IOM uint32_t SW_CFG10;                  /**< PKC Software Configuration 10 Register ,                              Address offset: 0x006C */
    __IOM uint32_t SW_CFG11;                  /**< PKC Software Configuration 11 Register ,                              Address offset: 0x0070 */
    __IOM uint32_t SW_CFG12;                  /**< PKC Software Configuration 12 Register ,                              Address offset: 0x0074 */
    __IOM uint32_t SW_CFG13;                  /**< PKC Software Configuration 13 Register ,                              Address offset: 0x0078 */
    __IOM uint32_t RESERVED1[1];              /**< RESERVED,                                                             Address offset: 0x007C */
    __IOM uint32_t INT_STAT;                  /**< PKC Interrupt Status Register ,                                       Address offset: 0x0080 */
    __IOM uint32_t INT_EN;                    /**< PKC Interrupt Enable Register,                                        Address offset: 0x0084 */
    __IOM uint32_t STAT;                      /**< PKC Status Register ,                                                 Address offset: 0x0088 */
} pkc_regs_t;

/**
  * @brief PWM
  */
typedef struct _pwm_regs
{
    __IOM uint32_t MODE;            /**< PWM_REG_MODE,      Address, offset: 0x00 */
    __IOM uint32_t UPDATE;          /**< PWM_REG_UPDATE,    Address, offset: 0x04 */
    __IOM uint32_t PRD;             /**< PWM_REG_PRD,       Address, offset: 0x08 */
    __IOM uint32_t CMPA0;           /**< PWM_REG_CMPA0,     Address, offset: 0x0C */
    __IOM uint32_t CMPA1;           /**< PWM_REG_CMPA1,     Address, offset: 0x10 */
    __IOM uint32_t CMPB0;           /**< PWM_REG_CMPB0,     Address, offset: 0x14 */
    __IOM uint32_t CMPB1;           /**< PWM_REG_CMPB1,     Address, offset: 0x18 */
    __IOM uint32_t CMPC0;           /**< PWM_REG_CMPC0,     Address, offset: 0x1C */
    __IOM uint32_t CMPC1;           /**< PWM_REG_CMPC1,     Address, offset: 0x20 */
    __IOM uint32_t AQCTRL;          /**< PWM_REG_AQCTRL,    Address, offset: 0x24 */
    __IOM uint32_t BRPRD;           /**< PWM_REG_BRPRD,     Address, offset: 0x28 */
    __IOM uint32_t HOLD;            /**< PWM_REG_HOLD,      Address, offset: 0x2C */
} pwm_regs_t;

/**
  * @brief QSPI
  */
typedef struct _qspi_regs
{
    __IOM uint32_t CTRL0;           /**< QSPI_REG_CTRL0,           Address offset: 0x0000*/
    __IOM uint32_t CTRL1;           /**< QSPI_REG_CTRL1,           Address offset: 0x0004*/
    __IOM uint32_t QSPI_EN;         /**< QSPI_REG_QSPI_EN,         Address offset: 0x0008*/
    __IOM uint32_t MWC;             /**< QSPI_REG_MWC,             Address offset: 0x000C*/
    __IOM uint32_t SE;              /**< QSPI_REG_SE,              Address offset: 0x0010*/
    __IOM uint32_t BAUD;            /**< QSPI_REG_BAUD,            Address offset: 0x0014*/
    __IOM uint32_t TX_FTL;          /**< QSPI_REG_TX_FTL,          Address offset: 0x0018*/
    __IOM uint32_t RX_FTL;          /**< QSPI_REG_RX_FTL,          Address offset: 0x001C*/
    __IM  uint32_t TX_FL;           /**< QSPI_REG_TX_FL,           Address offset: 0x0020*/
    __IM  uint32_t RX_FL;           /**< QSPI_REG_RX_FL,           Address offset: 0x0024*/
    __IM  uint32_t STAT;            /**< QSPI_REG_STAT,            Address offset: 0x0028*/
    __IOM uint32_t INTMASK;         /**< QSPI_REG_INT_MASK,        Address offset: 0x002C*/
    __IM  uint32_t INTSTAT;         /**< QSPI_REG_INT_STAT,        Address offset: 0x0030*/
    __IM  uint32_t RAW_INTSTAT;     /**< QSPI_REG_RAW_INT_STAT,    Address offset: 0x0034*/
    __IM  uint32_t TXOIC;           /**< QSPI_REG_TXOIC,           Address offset: 0x0038*/
    __IM  uint32_t RXOIC;           /**< QSPI_REG_RXOIC,           Address offset: 0x003C*/
    __IM  uint32_t RXUIC;           /**< QSPI_REG_RXUIC,           Address offset: 0x0040*/
    __IM  uint32_t MSTIC;           /**< QSPI_REG_MSTIC,           Address offset: 0x0044*/
    __IM  uint32_t INTCLR;          /**< QSPI_REG_INT_CLR,         Address offset: 0x0048*/
    __IOM uint32_t DMAC;            /**< QSPI_REG_DMAC,            Address offset: 0x004C*/
    __IOM uint32_t DMA_TDL;         /**< QSPI_REG_DMA_TDL,         Address offset: 0x0050*/
    __IOM uint32_t DMA_RDL;         /**< QSPI_REG_DMA_RDL,         Address offset: 0x0054*/
    __IM  uint32_t ID;              /**< QSPI_REG_ID,              Address offset: 0x0058*/
    __IM  uint32_t VERSION_ID;      /**< QSPI_REG_VERSION_ID,      Address offset: 0x005C*/
    __IOM uint32_t DATA;            /**< QSPI_REG_DATA,            Address offset: 0x0060*/
    __IM  uint32_t REVERSED[35];    /**< QSPI_REG_REVERSED,        Address offset: 0x0064*/
    __IOM uint32_t RX_SAMPLE_DLY;   /**< QSPI_REG_RX_SAMPLE_DLY,   Address offset: 0x00F0*/
    __IOM uint32_t SPI_CTRL0;       /**< QSPI_REG_SPI_CTRL0,       Address offset: 0x00F4*/

    __IOM uint32_t DDR_DRIVE_EDGE;  /**< QSPI_REG_DDR_DRIVE_EDGE,  Address offset: 0x00F8*/
    __IOM uint32_t XIP_MODE_BITS;   /**< QSPI_REG_XIP_MODE_BITS,   Address offset: 0x00FC*/
    __IOM uint32_t XIP_INCR_INST;   /**< QSPI_REG_XIP_INCR_INST,   Address offset: 0x0100*/
    __IOM uint32_t XIP_WRAP_INST;   /**< QSPI_REG_XIP_WRAP_INST,   Address offset: 0x0104*/
    __IOM uint32_t XIP_CTRL;        /**< QSPI_REG_XIP_CTRL,        Address offset: 0x0108*/
    __IOM uint32_t XIP_SER;         /**< QSPI_REG_XIP_SER,         Address offset: 0x010C*/
    __IM  uint32_t XIP_RXOICR;      /**< QSPI_REG_XIP_RXOICR,      Address offset: 0x0110*/
    __IOM uint32_t XIP_CNT_TIME_OUT;/**< QSPI_REG_XIP_CNT_TIME_OUT,Address offset: 0x0114*/

    __IOM uint32_t SPI_CTRL1;       /**< QSPI_REG_SPI_CTRL0,       Address offset: 0x0118*/
    __IOM uint32_t SPI_TEIC;        /**< QSPI_REG_SPI_TEIC,        Address offset: 0x011C*/
    __IOM uint32_t _UNUSED_SPIDR;   /**< QSPI_REG_SPI_DR,          Address offset: 0x0120*/
    __IOM uint32_t _UNUSED_SPIAR;   /**< QSPI_REG_SPI_AR,          Address offset: 0x0124*/
    __IOM uint32_t _UNUSED_AXIAR0;  /**< QSPI_REG_AXI_AR0,         Address offset: 0x0128*/
    __IOM uint32_t _UNUSED_AXIAR1;  /**< QSPI_REG_AXI_AR1,         Address offset: 0x012C*/
    __IOM uint32_t _UNUSED_AXIECR;  /**< QSPI_REG_AXI_ECR,         Address offset: 0x0130*/
    __IOM uint32_t _UNUSED_DONECR;  /**< QSPI_REG_DON_ECR,         Address offset: 0x0134*/
    __IOM uint32_t _UNUSED_RSVD[2]; /**< QSPI_REG_RESD,            Address offset: 0x0138 ~ 0x013C*/

    __IOM uint32_t XIP_WR_INCR_INST;/**< QSPI_REG_XIP_WR_INCR_INST,Address offset: 0x0140*/
    __IOM uint32_t XIP_WR_WRAP_INST;/**< QSPI_REG_XIP_WR_WRAP_INST,Address offset: 0x0144*/
    __IOM uint32_t XIP_WR_CTRL;     /**< QSPI_REG_XIP_WR_CTRL,     Address offset: 0x0148*/
} qspi_regs_t;


/**
  * @brief Mobi OSPI, x means xccela bus
  */
typedef struct _ospi_x_regs {
    __IOM uint32_t MEM_BASE_ADDR;   /**<  the bottom/base address of the memory.  Address offset: 0x00 */
    __IOM uint32_t MEM_TOP_ADDR;    /**<  the top address of the memory.          Address offset: 0x04 */
    __IOM uint32_t GLOBAL_RESET;    /**<  global reset .                          Address offset: 0x08 */
    __IOM uint32_t ACCESS_TYPE;     /**<  access type : memory array or register. Address offset: 0x0C */
    __IOM uint32_t ACCESS_TIMING;   /**<  set the tcem,trc,tcph,page size.        Address offset: 0x10 */
    __IOM uint32_t DEEP_DOWN_CNTRL; /**<  deep power down control.                Address offset: 0x14 */
    __IOM uint32_t HALF_SLP_CNTRL;  /**<  half sleep control.                     Address offset: 0x18 */
    __IOM uint32_t INTERRUPT_CNTRL; /**<  control kinds of interrupt.             Address offset: 0x1C */
    __IM  uint32_t XFER_STATUS;     /**<  transfer status.                        Address offset: 0x20 */
    __IOM uint32_t CMD_CNTRL_1;     /**<  command control reg 1.                  Address offset: 0x24 */
    __IOM uint32_t CMD_CNTRL_2;     /**<  command control reg 2.                  Address offset: 0x28 */
    __IOM uint32_t DQS_TIMEOUT;     /**<  DQS timeout.                            Address offset: 0x2C */
    __IOM uint32_t PHY_CNTRL_0;     /**<  PHY control register 0.                 Address offset: 0x30 */
    __IOM uint32_t PHY_CNTRL_1;     /**<  PHY control register 1.                 Address offset: 0x34 */
    __IOM uint32_t PHY_CNTRL_2;     /**<  PHY control register 2.                 Address offset: 0x38 */
    __IOM uint32_t PHY_CNTRL_3;     /**<  PHY control register 3.                 Address offset: 0x3C */
    __IOM uint32_t PHY_CNTRL_4;     /**<  PHY control register 4.                 Address offset: 0x40 */
    __IOM uint32_t PHY_CNTRL_5;     /**<  PHY control register 5.                 Address offset: 0x44 */
    __IOM uint32_t PHY_CNTRL_6;     /**<  PHY control register 6.                 Address offset: 0x48 */
    __IOM uint32_t PHY_CNTRL_7;     /**<  PHY control register 7.                 Address offset: 0x4C */
    __IOM uint32_t READ_PREFETCH;   /**<  read prefetch enable.                   Address offset: 0x50 */
} ospi_x_regs_t;


/**
  * @brief SPI
  */
typedef struct _spi_regs
{
    __IOM uint32_t CTRL0;                     /**< SPI Control Register 0 ,                                              Address offset: 0x0000 */
    __IOM uint32_t CTRL1;                     /**< SPI Control Register 1,                                               Address offset: 0x0004 */
    __IOM uint32_t SSI_EN;                    /**< SSI Enable Register,                                                  Address offset: 0x0008 */
    __IOM uint32_t MW_CTRL;                   /**< Microwire Control Register,                                           Address offset: 0x000C */
    __IOM uint32_t S_EN;                      /**< Slave Enable Register,                                                Address offset: 0x0010 */
    __IOM uint32_t BAUD;                      /**< Baud Rate Register,                                                   Address offset: 0x0014 */
    __IOM uint32_t TX_FIFO_TL;                /**< Transmit FIFO Threshold Level Register,                               Address offset: 0x0018 */
    __IOM uint32_t RX_FIFO_TL;                /**< Receive FIFO Threshold Level,                                         Address offset: 0x001C */
    __IOM uint32_t TX_FIFO_LEVEL;             /**< Transmit FIFO Level Register,                                         Address offset: 0x0020 */
    __IOM uint32_t RX_FIFO_LEVEL;             /**< Receive FIFO Level Register ,                                         Address offset: 0x0024 */
    __IOM uint32_t STAT;                      /**< Status Register,                                                      Address offset: 0x0028 */
    __IOM uint32_t INT_MASK;                  /**< Interrupt Mask Register,                                              Address offset: 0x002C */
    __IOM uint32_t INT_STAT;                  /**< Interrupt Status Register ,                                           Address offset: 0x0030 */
    __IOM uint32_t RAW_INT_STAT;              /**< Raw Interrupt Status Register,                                        Address offset: 0x0034 */
    __IOM uint32_t TX_FIFO_OIC;               /**< Transmit FIFO Overflow Interrupt Clear Register,                      Address offset: 0x0038 */
    __IOM uint32_t RX_FIFO_OIC;               /**< Receive FIFO Overflow Interrupt Clear Register ,                      Address offset: 0x003C */
    __IOM uint32_t RX_FIFO_UIC;               /**< Receive FIFO Underflow Interrupt Clear Register ,                     Address offset: 0x0040 */
    __IOM uint32_t MULTI_M_IC;                /**< Multi-Master Interrupt Clear Register,                                Address offset: 0x0044 */
    __IOM uint32_t INT_CLR;                   /**< Interrupt Clear Register ,                                            Address offset: 0x0048 */
    __IOM uint32_t DMA_CTRL;                  /**< DMA Control Register ,                                                Address offset: 0x004C */
    __IOM uint32_t DMA_TX_DL;                 /**< DMA Transmit Data Level Register,                                     Address offset: 0x0050 */
    __IOM uint32_t DMA_RX_DL;                 /**< DMA Receive Data Level Register,                                      Address offset: 0x0054 */
    __IOM uint32_t RESERVED0[2];              /**< RESERVED,                                                             Address offset: 0x0058 */
    __IOM uint32_t DATA;                      /**< Data Register,                                                        Address offset: 0x0060 */
    __IM  uint32_t REVERSED[35];              /**< DATA Registers REVERSED,                                              Address offset: 0x0064 */
    __IOM uint32_t RX_SAMPLE_DLY;             /**< RX sample delay Register,                                             Address offset: 0x00F0 */
} spi_regs_t;

/**
  * @brief TIMER
  */
typedef struct _timer_regs
{
    __IOM uint32_t CTRL;            /**< TIM control register,          Address offset: 0x00 */
    __IOM uint32_t VALUE;           /**< TIM counter value register,    Address offset: 0x04 */
    __IOM uint32_t RELOAD;          /**< TIM auto-reload register,      Address offset: 0x08 */
    __IOM uint32_t INTSTAT;         /**< TIM interrupt status register, Address offset: 0x0C */
} timer_regs_t;

/**
  * @brief UART
  */
typedef struct _uart_regs
{
    union
    {
        __IOM uint32_t RBR;
        __IOM uint32_t DLL;
        __IOM uint32_t THR;
    } RBR_DLL_THR;                  /**< UART_REG_RBR_DLL_THR,  Address offset: 0x00 */
    union
    {
        __IOM uint32_t DLH;
        __IOM uint32_t IER;
    } DLH_IER;                      /**< UART_REG_DLH_IER,      Address offset: 0x04 */
    union
    {
        __IOM uint32_t FCR;
        __IOM uint32_t IIR;
    } FCR_IIR;                      /**< UART_REG_FCR_IIR,      Address offset: 0x08 */
    __IOM uint32_t LCR;             /**< UART_REG_LCR,          Address offset: 0x0C */
    __IOM uint32_t MCR;             /**< UART_REG_MCR,          Address offset: 0x10 */
    __IOM uint32_t LSR;             /**< UART_REG_LSR,          Address offset: 0x14 */
    __IOM uint32_t MSR;             /**< UART_REG_MSR,          Address offset: 0x18 */
    __IOM uint32_t SCRATCHPAD;      /**< UART_REG_SCRATCHPAD,   Address offset: 0x1C */
    __IOM uint32_t LPDLL;           /**< UART_REG_LPDLL,        Address offset: 0x20 */
    __IOM uint32_t LPDLH;           /**< UART_REG_LPDLH,        Address offset: 0x24 */
    __IOM uint32_t REVERSED0[2];    /**< REVERSED,              Address offset: 0x28 */
    union
    {
        __IOM uint32_t SRBR[16];
        __IOM uint32_t STHR[16];
    } SRBR_STHR;                    /**< UART_REG_SRBR_STHR,    Address offset: 0x30 */
    __IOM uint32_t FAR;             /**< UART_REG_FAR,          Address offset: 0x70 */
    __IOM uint32_t TFR;             /**< UART_REG_TFR,          Address offset: 0x74 */
    __IOM uint32_t TFW;             /**< UART_REG_TFW,          Address offset: 0x78 */
    __IOM uint32_t USR;             /**< UART_REG_USR,          Address offset: 0x7C */
    __IOM uint32_t TFL;             /**< UART_REG_TFL,          Address offset: 0x80 */
    __IOM uint32_t RFL;             /**< UART_REG_RFL,          Address offset: 0x84 */
    __IOM uint32_t SRR;             /**< UART_REG_SRR,          Address offset: 0x88 */
    __IOM uint32_t SRTS;            /**< UART_REG_SRTS,         Address offset: 0x8C */
    __IOM uint32_t SBCR;            /**< UART_REG_SBCR,         Address offset: 0x90 */
    __IOM uint32_t SDMAM;           /**< UART_REG_SDMAM,        Address offset: 0x94 */
    __IOM uint32_t SFE;             /**< UART_REG_SFE,          Address offset: 0x98 */
    __IOM uint32_t SRT;             /**< UART_REG_SRT,          Address offset: 0x9C */
    __IOM uint32_t STET;            /**< UART_REG_STET,         Address offset: 0xA0 */
    __IOM uint32_t HTX;             /**< UART_REG_HTX,          Address offset: 0xA4 */
    __IOM uint32_t DMASA;           /**< UART_REG_DMASA,        Address offset: 0xA8 */
    __IOM uint32_t TCR;             /**< UART_REG_TCR,          Address offset: 0xAC */
    __IOM uint32_t DE_EN;           /**< UART_REG_DE_EN,        Address offset: 0xB0 */
    __IOM uint32_t RE_EN;           /**< UART_REG_RE_EN,        Address offset: 0xB4 */
    __IOM uint32_t DET;             /**< UART_REG_DET,          Address offset: 0xB8 */
    __IOM uint32_t TAT;             /**< UART_REG_TAT,          Address offset: 0xBC */
    __IOM uint32_t DLF;             /**< UART_REG_DLF,          Address offset: 0xC0 */
    __IOM uint32_t RAR;             /**< UART_REG_RAR,          Address offset: 0xC4 */
    __IOM uint32_t TAR;             /**< UART_REG_TAR,          Address offset: 0xC8 */
    __IOM uint32_t LCR_EXT;         /**< UART_REG_LCR_EXT,      Address offset: 0xCC */
    __IOM uint32_t REVERSED1[9];    /**< REVERSED,              Address offset: 0xD0 */
    __IOM uint32_t CPR;             /**< UART_REG_CPR,          Address offset: 0xF4 */
    __IOM uint32_t UCV;             /**< UART_REG_UCV,          Address offset: 0xF8 */
    __IOM uint32_t CTR;             /**< UART_REG_CTR,          Address offset: 0xFC */
} uart_regs_t;

/**
  * @brief USB
  */
typedef struct _usb_regs
{
    __IOM uint32_t CTRL;                      /**< USB Control Register,                                                 Address offset: 0x0000 */
    __OM  uint32_t EP0_1_CTRL;                /**< USB EP0 EP1 Control Register,                                         Address offset: 0x0004 */
    __IOM uint32_t EP2_CTRL;                  /**< USB EP2 Control Register,                                             Address offset: 0x0008 */
    __IOM uint32_t EP3_CTRL;                  /**< USB EP3 Control Register,                                             Address offset: 0x000C */
    __IOM uint32_t EP_ATTR;                   /**< USB EP attribute Register,                                            Address offset: 0x0010 */
    __IOM uint32_t INT_STAT;                  /**< USB interrupt status Register,                                        Address offset: 0x0014 */
    __IOM uint32_t INT_EN;                    /**< USB interrupt enable Register,                                        Address offset: 0x0018 */
    __OM  uint32_t INT_CLR;                   /**< USB interrupt clear Register,                                         Address offset: 0x001C */
    __IOM uint32_t EP3_AHBM_RADDR;           /**<  EP3 AHB master read start address Register,                           Address offset: 0x0020 */
    __IOM uint32_t EP3_AHBM_CTRL;             /**< EP3 AHB master control Register,                                      Address offset: 0x0024 */
    __IOM uint32_t CTRL0;                     /**< USB Control Register 0 ,                                              Address offset: 0x0028 */
    __IOM uint32_t EP3_XFER_LEN;              /**< USB EP3 DMA total transfer length Register,                           Address offset: 0x002C */
    __IOM uint32_t RX_CNT;                    /**< USB received data sum Register,                                       Address offset: 0x0030 */
    __IOM uint32_t CFG_DESC_CTRL;             /**< config descriptor Control Register,                                   Address offset: 0x0034 */
    __IOM uint32_t STR_DESC0_CTRL;            /**< Language ID descriptor Control Register,                              Address offset: 0x0038 */
    __IOM uint32_t STR_DESC1_CTRL;            /**< string descriptor Control Register,                                   Address offset: 0x003C */
    __IOM uint32_t EP0_FIFO_ADDR;             /**< USB endpoint 0 FIFO address Register,                                 Address offset: 0x0040 */
    __IOM uint32_t EP1_FIFO_ADDR;             /**< USB endpoint 1 FIFO address Register,                                 Address offset: 0x0044 */
    __OM  uint32_t EP2_FIFO_ADDR;             /**< USB endpoint 2 FIFO address Register,                                 Address offset: 0x0048 */
    __IOM uint32_t EP3_FIFO_ADDR;             /**< USB endpoint 3 FIFO address Register,                                 Address offset: 0x004C */
    __IOM uint32_t SRAM_ADDR;                 /**< USB descriptor SRAM address Register,                                 Address offset: 0x0050 */
    __IOM uint32_t STR_DESC2_CTRL;            /**< string descriptor Control Register,                                   Address offset: 0x0054 */
    __IOM uint32_t STR_DESC3_CTRL;            /**< string descriptor Control Register,                                   Address offset: 0x0058 */
    __IOM uint32_t USB_RESERVED;              /**< usb reserced Register,                                                Address offset: 0x005C */
    __IOM uint32_t EP4_CTRL;                  /**< USB EP4 Control Register,                                             Address offset: 0x0060 */
    __IOM uint32_t EP4_AHBM_RADDR;            /**< EP4 AHB master read start address Register,                           Address offset: 0x0064 */
    __IOM uint32_t EP4_AHBM_CTRL;             /**< EP4 AHB master control Register,                                      Address offset: 0x0068 */
    __IOM uint32_t EP4_XFER_LEN;              /**< USB EP4 DMA total transfer length Register,                           Address offset: 0x006C */
    __IOM uint32_t EP5_CTRL;                  /**< EP5 control Register,                                                 Address offset: 0x0070 */
    __IOM uint32_t EP5_AHBM_RADDR;            /**< EP5 AHB master read start address Register,                           Address offset: 0x0074 */
    __IOM uint32_t EP5_XFER_LEN;              /**< USB EP5 DMA total transfer length Register,                           Address offset: 0x0078 */
    __IOM uint32_t EP5_TIMER;                 /**< USB EP5 DMA receive data timeout set Register,                        Address offset: 0x007C */
    __IOM uint32_t EP4_FIFO_WR_EN;            /**< USB EP4 FIFO write byte enable,                                       Address offset: 0x0080 */
    __IOM uint32_t EP4_FIFO_ADDR;             /**< USB EP4 FIFO address,                                                 Address offset: 0x0084 */
    __IOM uint32_t EP5_FIFO_ADDR;             /**< USB EP5 FIFO address,                                                 Address offset: 0x0088 */
    __IOM uint32_t EP5_RX_CNT;                /**< USB EP5 received data sum Register,                                   Address offset: 0x008C */
    __IOM uint32_t DEBUG_REG;                 /**< USB debug Register,                                                   Address offset: 0x0090 */
} usb_regs_t;

/**
  * @brief WDT
  */
typedef struct _wdt_regs
{
    __IOM uint32_t LOAD;            /**< WDT_REG_LOAD,          Address offset: 0x000 */
    __IOM uint32_t VALUE;           /**< WDT_REG_VALUE,         Address offset: 0x004 */
    __IOM uint32_t CTRL;            /**< WDT_REG_CONTROL,       Address offset: 0x008 */
    __IOM uint32_t INTCLR;          /**< WDT_REG_INTCLR,        Address offset: 0x00C */
    __IOM uint32_t RIS;             /**< WDT_REG_RIS,           Address offset: 0x010 */
    __IOM uint32_t MIS;             /**< WDT_REG_MIS,           Address offset: 0x014 */
    __IOM uint32_t REVERSED[762];   /**< REVERSED,              Address offset: 0x018 */
    __IOM uint32_t LOCK;            /**< WDT_REG_LOCK,          Address offset: 0xC00 */
} wdt_regs_t;

/**
  * @brief XQSPI
  */
/* XQSPI/Cache Registers */
typedef struct
{
    __IOM uint32_t CTRL0;               /**< Cache Control 0 Register,  Address offset: 0x00 */
    __IOM uint32_t CTRL1;               /**< Cache Control 1 Register,  Address offset: 0x04 */
    __IM  uint32_t HIT_COUNT;           /**< Cache hits count,          Address offset: 0x08 */
    __IM  uint32_t MISS_COUNT;          /**< Cache miss count,          Address offset: 0x0C */
    __IM  uint32_t STAT;                /**< Status Register,           Address offset: 0x10 */
    __IOM uint32_t BUF_FIRST_ADDR;      /**< Preload start address,     Address offset: 0x14 */
    __IOM uint32_t BUF_LAST_ADDR;       /**< Preload last address,      Address offset: 0x18 */
} CACHE_REGS;

/* XQSPI/QSPI Registers */
typedef struct
{
    __OM  uint32_t TX_DATA;             /**< Serial Data Transmit,         Address offset: 0x00 */
    __IM  uint32_t RX_DATA;             /**< Serial Data Receive,          Address offset: 0x04 */
    __IM  uint32_t RESERVED0;           /**< RESERVED,                     Address offset: 0x08 */
    __IOM uint32_t CTRL;                /**< Control,                      Address offset: 0x0C */
    __IOM uint32_t AUX_CTRL;            /**< Auxiliary Control,            Address offset: 0x10 */
    __IM  uint32_t STAT;                /**< Status Control,               Address offset: 0x14 */
    __IOM uint32_t SLAVE_SEL;           /**< Slave Select,                 Address offset: 0x18 */
    __IOM uint32_t SLAVE_SEL_POL;       /**< Slave Select Polarity,        Address offset: 0x1C */
    __IOM uint32_t INTEN;               /**< Interrupt Enable,             Address offset: 0x20 */
    __IM  uint32_t INTSTAT;             /**< Interrupt Status,             Address offset: 0x24 */
    __OM  uint32_t INTCLR;              /**< Interrupt Clear,              Address offset: 0x28 */
    __IM  uint32_t TX_FIFO_LVL;         /**< TX FIFO Level,                Address offset: 0x2C */
    __IM  uint32_t RX_FIFO_LVL;         /**< RX FIFO Level,                Address offset: 0x30 */
    __IM  uint32_t RESERVED1;           /**< RESERVED,                     Address offset: 0x34 */
    __IOM uint32_t MSTR_IT_DELAY;       /**< Master Inter-transfer Delay,  Address offset: 0x38 */
    __IOM uint32_t SPIEN;               /**< SPI Enable,                   Address offset: 0x3C */
    __IOM uint32_t SPI_GP_SET;          /**< GPO Set / GPO State,          Address offset: 0x40 */
    __IOM uint32_t SPI_GP_CLEAR;        /**< GPO Clear / GPI State,        Address offset: 0x44 */
    __IM  uint32_t RX_DATA0_31;         /**< 32-bit LSB word(0~31),        Address offset: 0x48 */
    __IM  uint32_t RX_DATA32_63;        /**< 32-bit LSB word(32~63),       Address offset: 0x4C */
    __IM  uint32_t RX_DATA64_95;        /**< 32-bit LSB word(64~95),       Address offset: 0x50 */
    __IM  uint32_t RX_DATA96_127;       /**< 32-bit MSB word(96~127),      Address offset: 0x54 */
    __OM  uint32_t P_IV;                /**< 32-BIT IV Key,                Address offset: 0x58 */
    __IOM uint32_t FLASH_WRITE;         /**< use for flash write,          Address offset: 0x5C */
    __IOM uint32_t P_KEY_VALID_KPORT;   /**< Status bit,                   Address offset: 0x60 */
    __IOM uint32_t P_RD_KEY_EN_KPORT;   /**< Enable read key on keyport,   Address offset: 0x64 */
    __IOM uint32_t P_KEY_ADDR;          /**< Present key address,          Address offset: 0x68 */
    __IOM uint32_t P_KEYPORT_MASK;      /**< Present key mask,             Address offset: 0x6C */
    __IOM uint32_t BYPASS;              /**< Enable Bypass,                Address offset: 0x70 */
    __IM  uint32_t RESERVED2;           /**< RESERVED,                     Address offset: 0x74 */
    __IOM uint32_t CS_IDLE_UNVLD_EN;    /**< CS idle unvalid enable,       Address offset: 0x78 */
} QSPI_REGS;

/* XQSPI/XIP Registers */
typedef struct
{
    __IOM uint32_t CTRL0;               /**< XIP Control 0 Register,       Address offset: 0x00 */
    __IOM uint32_t CTRL1;               /**< XIP Control 1 Register,       Address offset: 0x04 */
    __IOM uint32_t CTRL2;               /**< XIP Control 2 Register,       Address offset: 0x08 */
    __IOM uint32_t CTRL3;               /**< XIP Enable Request,           Address offset: 0x0C */
    __IOM uint32_t STAT;                /**< XIP Enable Output (Stat0),    Address offset: 0x10 */
    __IM  uint32_t INTEN;               /**< Interrupt Enable  (Intr0),    Address offset: 0x14 */
    __IM  uint32_t INTSTAT;             /**< Interrupt Status  (Intr1),    Address offset: 0x18 */
    __IM  uint32_t INTREQ;              /**< Interrupt Status  (Intr2),    Address offset: 0x1C */
    __OM  uint32_t INTSET;              /**< Interrupt Set     (Intr3),    Address offset: 0x20 */
    __OM  uint32_t INTCLR;              /**< Interrupt Clear   (Intr4),    Address offset: 0x24 */
    __IOM uint32_t SOFT_RST;            /**< XIP Control Software Reset,   Address offset: 0x28 */
} XIP_REGS;

typedef struct _xqspi_regs
{
    CACHE_REGS      CACHE;              /**< CACHE Registers,              Address offset: 0x000 */
    __IM  uint32_t  RESERVED0[249];
    QSPI_REGS       QSPI;               /**< QSPI Registers,               Address offset: 0x400 */
    __IM  uint32_t  RESERVED1[481];
    XIP_REGS        XIP;                /**< XIP Registers,                Address offset: 0xC00 */
} xqspi_regs_t;

/**
  * @brief EFUSE
  */
typedef struct _efuse_regs
{
    __IOM uint32_t TPGM;                /**< EFUSE_TPGM,                   Address offset: 0x000 */
    __IOM uint32_t PGENB;               /**< EFUSE_PGENB,                  Address offset: 0x004 */
    __IM  uint32_t TEST_MODE;           /**< EFUSE_TEST_MODE,              Address offset: 0x008 */
    __OM  uint32_t OPERATION;           /**< EFUSE_OPERATION,              Address offset: 0x00C */
    __IM  uint32_t STAT;                /**< EFUSE_STATUS,                 Address offset: 0x010 */
    __IOM uint32_t KEY_MASK;            /**< EFUSE_KEY_MASK,               Address offset: 0x014 */
    __IOM uint32_t CRC_ADDR;            /**< EFUSE_CRC_START_ADDR,         Address offset: 0x018 */
    __IM  uint32_t CRC_OUTPUT;          /**< EFUSE_CRC_OUTPUT,             Address offset: 0x01C */
    __IOM uint32_t TRIM_ADDR;           /**< EFUSE_TRIM_ADDR,              Address offset: 0x020 */
    __IOM uint32_t TRIM_LEN;            /**< EFUSE_TRIM_LEN,               Address offset: 0x024 */
    __IM  uint32_t TRIM[14];            /**< EFUSE_TRIM,                   Address offset: 0x028 */
} efuse_regs_t;

/**
  * @brief RNG
  */
typedef struct _rng_regs
{
    __IOM uint32_t CTRL;                /**< RNG_CTRL,                     Address offset: 0x000 */
    __IOM uint32_t STATUS;              /**< RNG_STATUS,                   Address offset: 0x004 */
    __IM  uint32_t DATA;                /**< RNG_DATA,                     Address offset: 0x008 */
    __IOM uint32_t RESERVED;            /**< RESERVED,                     Address offset: 0x00C */
    __IM  uint32_t LR_STATUS;           /**< RNG_LR_STATUS,                Address offset: 0x010 */
    __IOM uint32_t CONFIG;              /**< RNG_CONFIG,                   Address offset: 0x014 */
    __IOM uint32_t TSCON;               /**< RNG_TSCON,                    Address offset: 0x018 */
    __IOM uint32_t FROCFG;              /**< RNG_FROCFG,                   Address offset: 0x01C */
    __OM  uint32_t USER_SEED;           /**< RNG_USER_SEED,                Address offset: 0x020 */
    __IOM uint32_t LRCON;               /**< RNG_LRCON,                    Address offset: 0x024 */
} rng_regs_t;

/**
  * @brief DSPI
  */
typedef struct _dspi_regs
{
    __IOM uint32_t CTRL1;               /**< DSPI_CTRL1,                   Address offset: 0x000 */
    __IOM uint32_t CTRL2;               /**< DSPI_CTRL2,                   Address offset: 0x004 */
    __IOM uint32_t STAT;                /**< DSPI_STAT,                    Address offset: 0x008 */
    __IOM uint32_t DATA;                /**< DSPI_DATA,                    Address offset: 0x00C */
    __IOM uint32_t MODE;                /**< DSPI_MODE,                    Address offset: 0x010 */
} dspi_regs_t;

/**
  * @brief PDM
  */
typedef struct _pdm_regs
{
    __IOM uint32_t EN_L;                      /**< PDM Enables left PDM microphone Register,                             Address offset: 0x0000 */
    __IOM uint32_t IN_CFG_L;                  /**< PDM Left Input Configuration Register,                                Address offset: 0x0004 */
    __IOM uint32_t LPF_CFG_L;                 /**< PDM Left LPF Configuration Register,                                  Address offset: 0x0008 */
    __IOM uint32_t HPF_CFG_L;                 /**< PDM Left HPF Configuration Register,                                  Address offset: 0x000C */
    __IOM uint32_t PGA_CFG_L;                 /**< PDM Left PGA Configuration Register,                                  Address offset: 0x0010 */
    __IOM uint32_t DATA_L;                    /**< PDM Left DATA Configuration Register,                                 Address offset: 0x0014 */
    __IOM uint32_t INT_L;                     /**< PDM Left DATA Interrupt Register,                                     Address offset: 0x0018 */
    __IOM uint32_t EN_R;                      /**< PDM Enables Right PDM Microphone Register,                            Address offset: 0x001C */
    __IOM uint32_t IN_CFG_R;                  /**< PDM Right Input Configuration Register,                               Address offset: 0x0020 */
    __IOM uint32_t LPF_CFG_R;                 /**< PDM Right LPF Configuration Register,                                 Address offset: 0x0024 */
    __IM  uint32_t RESERVED0[6];              /**< RESERVED,                                                             Address offset: 0x0028 */
    __IOM uint32_t HPF_CFG_R;                 /**< PDM Right HPF Configuration Register,                                 Address offset: 0x0040 */
    __IOM uint32_t PGA_CFG_R;                 /**< PDM Right PGA Configuration Register,                                 Address offset: 0x0044 */
    __IOM uint32_t DATA_R;                    /**< PDM Right DATA Configuration Register,                                Address offset: 0x0048 */
    __IOM uint32_t INT_R;                     /**< PDM Right DATA Interrupt Register,                                    Address offset: 0x004C */
    __IM  uint32_t DATA;                      /**< PDM  DATA  Register,                                                  Address offset: 0x0050 */
    __IOM uint32_t CLK_DIV;                   /**< PDM CLK DIV Configuration Register,                                   Address offset: 0x0054 */
    __IOM uint32_t CLK;                       /**< PDM CLK Enable Configuration Register,                                Address offset: 0x0058 */
} pdm_regs_t;

/**
  */
/**
  * @brief GPADC
  */
typedef struct _gpadc_regs
{
    __IOM uint32_t CTRL0;                     /**< GPADC Control Register0,                                                Address offset: 0x0000 */
    __IOM uint32_t CTRL1;                     /**< GPADC Control Register1,                                                Address offset: 0x0004 */
    __IOM uint32_t COEF0;                     /**< GPADC coefficient 0 register,                                           Address offset: 0x0008 */
    __IOM uint32_t COEF1;                     /**< GPADC coefficient 1 register,                                           Address offset: 0x000C */
    __IOM uint32_t COEF2;                     /**< GPADC coefficient 2 register,                                           Address offset: 0x0010 */
    __IOM uint32_t COEF3;                     /**< GPADC coefficient 3 register,                                           Address offset: 0x0014 */
    __IOM uint32_t COEF4;                     /**< GPADC coefficient 4 register,                                           Address offset: 0x0018 */
    __IOM uint32_t COEF5;                     /**< GPADC coefficient 5 register,                                           Address offset: 0x001C */
    __IOM uint32_t COEF6;                     /**< GPADC coefficient 6 register,                                           Address offset: 0x0020 */
    __IOM uint32_t COEF7;                     /**< GPADC coefficient 7 register,                                           Address offset: 0x0024 */
    __IOM uint32_t COEF8;                     /**< GPADC coefficient 8 register,                                           Address offset: 0x0028 */
    __IOM uint32_t COEF9;                     /**< GPADC coefficient 9 register,                                           Address offset: 0x002C */
    __IOM uint32_t COEF10;                    /**< GPADC coefficient 10 register,                                          Address offset: 0x0030 */
    __IOM uint32_t COEF11;                    /**< GPADC coefficient 11 register,                                          Address offset: 0x0034 */
    __IOM uint32_t COEF12;                    /**< GPADC coefficient 12 register,                                          Address offset: 0x0038 */
    __IOM uint32_t COEF13;                    /**< GPADC coefficient 13 register,                                          Address offset: 0x003C */
    __IOM uint32_t COEF14;                    /**< GPADC coefficient 14 register,                                          Address offset: 0x0040 */
    __IM  uint32_t DATA;                      /**< GPADC data register,                                                    Address offset: 0x0044 */
    __IOM uint32_t CONSTANT;                  /**< GPADC constant register,                                                Address offset: 0x0048 */
    __IOM uint32_t OFFSET;                    /**< GPADC offset calibration register,                                      Address offset: 0x004C */
    __IOM uint32_t ANA_CTRL;                  /**< GPADC analog control register,                                          Address offset: 0x0050 */
    __IOM uint32_t RESERVED0[3];              /**< RESERVED0,                                                              Address offset: 0x0054 */
    __IM  uint32_t FIFO_RD;                   /**< GPADC Read FIFO Register,                                               Address offset: 0x0060 */
    __IOM uint32_t FIFO_THD;                  /**< GPADC FIFO Threshold Register,                                          Address offset: 0x0064 */
    __IOM uint32_t FIFO_STAT;                 /**< GPADC FIFO Status Register,                                             Address offset: 0x0068 */
    __IOM uint32_t RESERVED1[37];              /**< RESERVED1,                                                             Address offset: 0x006C */
    __IOM uint32_t ANA_MBG;                   /**< GPADC vdd(ldo23) control register,                                      Address offset: 0x0100 */
    __IOM uint32_t ANA_PGA;                   /**< GPADC PGA control register,                                             Address offset: 0x0104 */
    __IOM uint32_t ANA_MPX;                   /**< GPADC MPX test control register,                                        Address offset: 0x0108 */
} gpadc_regs_t;

/**
  * @brief SADC
  */
typedef struct _sadc_regs
{
    __IM  uint32_t FIFO_RD;                   /**< Sense ADC Read FIFO Register,                                         Address offset: 0x0000 */
    __IOM uint32_t FIFO_THD;                  /**< Sense ADC FIFO Threshold Register,                                    Address offset: 0x0004 */
    __IOM uint32_t FIFO_STAT;                 /**< Sense ADC Status Register,                                            Address offset: 0x0008 */
    __IOM uint32_t CLK;                       /**< Sense ADC Clock Register,                                             Address offset: 0x000C */
    __IOM uint32_t RESERVED0[1];              /**< RESERVED,                                                             Address offset: 0x0010 */
    __IOM uint32_t GET_TKN_HW;                /**< Sense ADC get token for Hardware Register,                            Address offset: 0x0014 */
    __IOM uint32_t GET_TKN_SW;                /**< Sense ADC get token for software Register,                            Address offset: 0x0018 */
    __IOM uint32_t RET_TKN_HW;                /**< Sense ADC release the HW token Register,                              Address offset: 0x001C */
    __OM  uint32_t RET_TKN_SW;                /**< Sense ADC release the SW token Register,                              Address offset: 0x0020 */
    __IOM uint32_t TKN_STAT;                  /**< Sense ADC Token Status Register,                                      Address offset: 0x0024 */
} sadc_regs_t;

/** @} */ /* End of group Peripheral_registers_structures */


/* ====================================  End of section using anonymous unions  ==================================== */
#if   defined (__CC_ARM)
    #pragma pop
#elif defined (__ICCARM__)
    /* leave anonymous unions enabled */
#elif (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
#elif defined (__GNUC__)
    /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
    /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
    #pragma warning restore
#elif defined (__CSMC__)
    /* anonymous unions are enabled by default */
#else
    #warning Not supported compiler type
#endif


/* ================================================================================================================= */
/* ================                     Device Specific Peripheral Address Map                      ================ */
/* ================================================================================================================= */

/** @addtogroup Peripheral_memory_map
  * @{
  */

#define ROM_BASE                ((uint32_t)0x00000000UL)
#define FLASH_BASE              ((uint32_t)0x00200000UL)
#define SRAM_BASE               ((uint32_t)0x30000000UL)
#define PERIPH_BASE             ((uint32_t)0x40000000UL)
#define PERIPH_QSPI_XIP_BASE    ((uint32_t)0xC0000000UL)

#define TIMER0_BASE             (PERIPH_BASE + 0x00000000UL)
#define TIMER1_BASE             (PERIPH_BASE + 0x00001000UL)
#define DUAL_TIM0_BASE          (PERIPH_BASE + 0x00002000UL)
#define DUAL_TIM1_BASE          (PERIPH_BASE + 0x00002020UL)
#define WDT_BASE                (PERIPH_BASE + 0x00008000UL)
#define USB_BASE                (PERIPH_BASE + 0x00009000UL)
#define SPIM_BASE               (PERIPH_BASE + 0x0000C000UL)
#define SPIS_BASE               (PERIPH_BASE + 0x0000C100UL)
#define I2C0_BASE               (PERIPH_BASE + 0x0000C300UL)
#define I2C1_BASE               (PERIPH_BASE + 0x0000C400UL)
#define I2C2_BASE               (PERIPH_BASE + 0x0000FA00UL)
#define I2C3_BASE               (PERIPH_BASE + 0x0000FB00UL)
#define I2C4_BASE               (PERIPH_BASE + 0x0000FC00UL)
#define I2C5_BASE               (PERIPH_BASE + 0x0000FD00UL)
#define UART0_BASE              (PERIPH_BASE + 0x0000C500UL)
#define UART1_BASE              (PERIPH_BASE + 0x0000C600UL)
#define UART2_BASE              (PERIPH_BASE + 0x0000C700UL)
#define UART3_BASE              (PERIPH_BASE + 0x0000C800UL)
#define UART4_BASE              (PERIPH_BASE + 0x0000C900UL)
#define UART5_BASE              (PERIPH_BASE + 0x0000CA00UL)
#define PWM0_BASE               (PERIPH_BASE + 0x0000CB00UL)
#define I2S_M_BASE              (PERIPH_BASE + 0x0000F200UL)
#define PWM1_BASE               (PERIPH_BASE + 0x0000CC00UL)
#define XQSPI_BASE              (PERIPH_BASE + 0x0000D000UL)
#define I2S_S_BASE              (PERIPH_BASE + 0x0000F000UL)
#define ISO7816_BASE            (PERIPH_BASE + 0x0000F400UL)
#define PDM_BASE                (PERIPH_BASE + 0x0000F700UL)
#define GPIO0_BASE              (PERIPH_BASE + 0x00010000UL)
#define GPIO1_BASE              (PERIPH_BASE + 0x00011000UL)
#define GPIO2_BASE              (PERIPH_BASE + 0x00012000UL)
#define DMA0_BASE               (PERIPH_BASE + 0x00019000UL)
#define DMA1_BASE               (PERIPH_BASE + 0x0001A000UL)
#define DMA2_BASE               (PERIPH_BASE + 0x0001B000UL)
#define PKC_BASE                (PERIPH_BASE + 0x00014000UL)
#define AES_BASE                (PERIPH_BASE + 0x00015000UL)
#define HMAC_BASE               (PERIPH_BASE + 0x00015800UL)
#define EFUSE_BASE              (PERIPH_BASE + 0x00016400UL)
#define RNG_BASE                (PERIPH_BASE + 0x00017800UL)

#define DSPI_BASE               (PERIPH_BASE + 0x0002E000UL)
#define QSPI0_BASE              (PERIPH_BASE + 0x00021000UL)
#define QSPI1_BASE              (PERIPH_BASE + 0x00022000UL)
#define QSPI2_BASE              (PERIPH_BASE + 0x00023000UL)
#define OSPI0_BASE              (PERIPH_BASE + 0x00024000UL)

#define QSPI0_XIP_BASE          (PERIPH_QSPI_XIP_BASE + 0x00000000UL)
#define QSPI1_XIP_BASE          (PERIPH_QSPI_XIP_BASE + 0x04000000UL)
#define QSPI2_XIP_BASE          (PERIPH_QSPI_XIP_BASE + 0x08000000UL)

#define OSPI0_XIP_BASE          (0x30080000UL)

#define PKC_SPRAM_BASE          (PKC_BASE    + 0x00000800UL)
#define KRAM_BASE               (PERIPH_BASE + 0x00017000UL)
#define EFUSE_STORAGE_BASE      (PERIPH_BASE + 0x00016000UL)
#define GPADC_BASE              (PERIPH_BASE + 0x0000F800UL)
#define SADC_BASE               (PERIPH_BASE + 0x0000E000UL)

/** @} */ /* End of group Peripheral_memory_map */


/* ================================================================================================================= */
/* ================                             Peripheral declaration                              ================ */
/* ================================================================================================================= */

/** @addtogroup Peripheral_declaration
  * @{
  */

#define TIMER0                  ((timer_regs_t *)TIMER0_BASE)
#define TIMER1                  ((timer_regs_t *)TIMER1_BASE)
#define DUAL_TIMER0             ((dual_timer_regs_t *)DUAL_TIM0_BASE)
#define DUAL_TIMER1             ((dual_timer_regs_t *)DUAL_TIM1_BASE)
#define WDT                     ((wdt_regs_t *)WDT_BASE)
#define USB                     ((usb_regs_t *)USB_BASE)
#define SPIM                    ((spi_regs_t *)SPIM_BASE)
#define SPIS                    ((spi_regs_t *)SPIS_BASE)
#define QSPI0                   ((qspi_regs_t *)QSPI0_BASE)
#define QSPI1                   ((qspi_regs_t *)QSPI1_BASE)
#define QSPI2                   ((qspi_regs_t *)QSPI2_BASE)
#define OSPI0                   ((ospi_x_regs_t *)OSPI0_BASE)
#define I2C0                    ((i2c_regs_t *)I2C0_BASE)
#define I2C1                    ((i2c_regs_t *)I2C1_BASE)
#define I2C2                    ((i2c_regs_t *)I2C2_BASE)
#define I2C3                    ((i2c_regs_t *)I2C3_BASE)
#define I2C4                    ((i2c_regs_t *)I2C4_BASE)
#define I2C5                    ((i2c_regs_t *)I2C5_BASE)
#define UART0                   ((uart_regs_t *)UART0_BASE)
#define UART1                   ((uart_regs_t *)UART1_BASE)
#define UART2                   ((uart_regs_t *)UART2_BASE)
#define UART3                   ((uart_regs_t *)UART3_BASE)
#define UART4                   ((uart_regs_t *)UART4_BASE)
#define UART5                   ((uart_regs_t *)UART5_BASE)
#define PWM0                    ((pwm_regs_t *)PWM0_BASE)
#define PWM1                    ((pwm_regs_t *)PWM1_BASE)
#define I2S_M                   ((i2s_regs_t *)I2S_M_BASE)
#define I2S_S                   ((i2s_regs_t *)I2S_S_BASE)
#define ISO7816                 ((iso7816_regs_t *)ISO7816_BASE)
#define XQSPI                   ((xqspi_regs_t *)XQSPI_BASE)
#define GPIO0                   ((gpio_regs_t *)GPIO0_BASE)
#define GPIO1                   ((gpio_regs_t *)GPIO1_BASE)
#define GPIO2                   ((gpio_regs_t *)GPIO2_BASE)
#define DMA0                    ((dma_regs_t *)DMA0_BASE)
#define DMA1                    ((dma_regs_t *)DMA1_BASE)
#define DMA2                    ((dma_regs_t *)DMA2_BASE)
#define PKC                     ((pkc_regs_t *)PKC_BASE)
#define AES                     ((aes_regs_t *)AES_BASE)
#define HMAC                    ((hmac_regs_t *)HMAC_BASE)
#define EFUSE                   ((efuse_regs_t *)EFUSE_BASE)
#define RNG                     ((rng_regs_t *)RNG_BASE)
#define DSPI                    ((dspi_regs_t *)DSPI_BASE)
#define PDM                     ((pdm_regs_t *)PDM_BASE)
#define GPADC                   ((gpadc_regs_t *)GPADC_BASE)

/** @} */ /* End of group Peripheral_declaration */

/** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/* ================================================================================================================= */
/* ================                                        AES                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for AES_CTRL register  *******************/
#define AES_CTRL_MODULE_EN_POS                              (0U)
#define AES_CTRL_MODULE_EN_Len                              (1U)
#define AES_CTRL_MODULE_EN_Msk                              (0x1UL << AES_CTRL_MODULE_EN_POS)
#define AES_CTRL_MODULE_EN                                  AES_CTRL_MODULE_EN_Msk

#define AES_CTRL_MCU_MODE_EN_POS                            (1U)
#define AES_CTRL_MCU_MODE_EN_Len                            (1U)
#define AES_CTRL_MCU_MODE_EN_Msk                            (0x1UL << AES_CTRL_MCU_MODE_EN_POS)
#define AES_CTRL_MCU_MODE_EN                                AES_CTRL_MCU_MODE_EN_Msk

#define AES_CTRL_DMA_MODE_EN_POS                            (2U)
#define AES_CTRL_DMA_MODE_EN_Len                            (1U)
#define AES_CTRL_DMA_MODE_EN_Msk                            (0x1UL << AES_CTRL_DMA_MODE_EN_POS)
#define AES_CTRL_DMA_MODE_EN                                AES_CTRL_DMA_MODE_EN_Msk

#define AES_CTRL_FKEY_EN_POS                                (3U)
#define AES_CTRL_FKEY_EN_Len                                (1U)
#define AES_CTRL_FKEY_EN_Msk                                (0x1UL << AES_CTRL_FKEY_EN_POS)
#define AES_CTRL_FKEY_EN                                    AES_CTRL_FKEY_EN_Msk

/*******************  Bit definition for AES_CFG register  *******************/
#define AES_CFG_KEY_MODE_POS                                (0U)
#define AES_CFG_KEY_MODE_Len                                (2U)
#define AES_CFG_KEY_MODE_Msk                                (0x3UL << AES_CFG_KEY_MODE_POS)
#define AES_CFG_KEY_MODE                                    AES_CFG_KEY_MODE_Msk

#define AES_CFG_FULL_MASK_EN_POS                            (3U)
#define AES_CFG_FULL_MASK_EN_Len                            (1U)
#define AES_CFG_FULL_MASK_EN_Msk                            (0x1UL << AES_CFG_FULL_MASK_EN_POS)
#define AES_CFG_FULL_MASK_EN                                AES_CFG_FULL_MASK_EN_Msk

#define AES_CFG_DEC_ENC_SEL_POS                             (4U)
#define AES_CFG_DEC_ENC_SEL_Len                             (1U)
#define AES_CFG_DEC_ENC_SEL_Msk                             (0x1UL << AES_CFG_DEC_ENC_SEL_POS)
#define AES_CFG_DEC_ENC_SEL                                 AES_CFG_DEC_ENC_SEL_Msk

#define AES_CFG_LOAD_SEED_POS                               (5U)
#define AES_CFG_LOAD_SEED_Len                               (1U)
#define AES_CFG_LOAD_SEED_Msk                               (0x1UL << AES_CFG_LOAD_SEED_POS)
#define AES_CFG_LOAD_SEED                                   AES_CFG_LOAD_SEED_Msk

#define AES_CFG_FIRST_BLK_POS                               (6U)
#define AES_CFG_FIRST_BLK_Len                               (1U)
#define AES_CFG_FIRST_BLK_Msk                               (0x1UL << AES_CFG_FIRST_BLK_POS)
#define AES_CFG_FIRST_BLK                                   AES_CFG_FIRST_BLK_Msk

#define AES_CFG_ENDIAN_POS                                  (7U)
#define AES_CFG_ENDIAN_Len                                  (1U)
#define AES_CFG_ENDIAN_Msk                                  (0x1UL << AES_CFG_ENDIAN_POS)
#define AES_CFG_ENDIAN                                      AES_CFG_ENDIAN_Msk

#define AES_CFG_OPT_MODE_POS                                (8U)
#define AES_CFG_OPT_MODE_Len                                (3U)
#define AES_CFG_OPT_MODE_Msk                                (0x7UL << AES_CFG_OPT_MODE_POS)
#define AES_CFG_OPT_MODE                                    AES_CFG_OPT_MODE_Msk

#define AES_CFG_KEY_TYPE_POS                                (11U)
#define AES_CFG_KEY_TYPE_Len                                (2U)
#define AES_CFG_KEY_TYPE_Msk                                (0x3UL << AES_CFG_KEY_TYPE_POS)
#define AES_CFG_KEY_TYPE                                    AES_CFG_KEY_TYPE_Msk

/*******************  Bit definition for AES_STAT register  *******************/
#define AES_STAT_READY_POS                                  (0U)
#define AES_STAT_READY_Len                                  (1U)
#define AES_STAT_READY_Msk                                  (0x1UL << AES_STAT_READY_POS)
#define AES_STAT_READY                                      AES_STAT_READY_Msk

#define AES_STAT_DMA_XFE_CPLT_POS                           (1U)
#define AES_STAT_DMA_XFE_CPLT_Len                           (1U)
#define AES_STAT_DMA_XFE_CPLT_Msk                           (0x1UL << AES_STAT_DMA_XFE_CPLT_POS)
#define AES_STAT_DMA_XFE_CPLT                               AES_STAT_DMA_XFE_CPLT_Msk

#define AES_STAT_DMA_XFE_ERR_POS                            (2U)
#define AES_STAT_DMA_XFE_ERR_Len                            (1U)
#define AES_STAT_DMA_XFE_ERR_Msk                            (0x1UL << AES_STAT_DMA_XFE_ERR_POS)
#define AES_STAT_DMA_XFE_ERR                                AES_STAT_DMA_XFE_ERR_Msk

#define AES_STAT_KEY_STAT_POS                               (3U)
#define AES_STAT_KEY_STAT_Len                               (1U)
#define AES_STAT_KEY_STAT_Msk                               (0x1UL << AES_STAT_KEY_STAT_POS)
#define AES_STAT_KEY_STAT                                   AES_STAT_KEY_STAT_Msk

/*******************  Bit definition for AES_INT register  *******************/
#define AES_INT_CPLT_INT_FLAG_POS                           (0U)
#define AES_INT_CPLT_INT_FLAG_Len                           (1U)
#define AES_INT_CPLT_INT_FLAG_Msk                           (0x1UL << AES_INT_CPLT_INT_FLAG_POS)
#define AES_INT_CPLT_INT_FLAG                               AES_INT_CPLT_INT_FLAG_Msk

#define AES_INT_CPLT_INT_EN_POS                             (1U)
#define AES_INT_CPLT_INT_EN_Len                             (1U)
#define AES_INT_CPLT_INT_EN_Msk                             (0x1UL << AES_INT_CPLT_INT_EN_POS)
#define AES_INT_CPLT_INT_EN                                 AES_INT_CPLT_INT_EN_Msk

/*******************  Bit definition for AES_XFE_SIZE register  *******************/
#define AES_XFE_SIZE_SIZE_POS                               (0U)
#define AES_XFE_SIZE_SIZE_Len                               (15U)
#define AES_XFE_SIZE_SIZE_Msk                               (0x7FFFUL << AES_XFE_SIZE_SIZE_POS)
#define AES_XFE_SIZE_SIZE                                   AES_XFE_SIZE_SIZE_Msk

/*******************  Bit definition for AES_RD_START_ADDR register  *******************/
#define AES_RD_START_ADDR_ADDR_POS                          (0U)
#define AES_RD_START_ADDR_ADDR_Len                          (32U)
#define AES_RD_START_ADDR_ADDR_Msk                          (0xFFFFFFFFUL << AES_RD_START_ADDR_ADDR_POS)
#define AES_RD_START_ADDR_ADDR                              AES_RD_START_ADDR_ADDR_Msk

/*******************  Bit definition for AES_WR_START_ADDR register  *******************/
#define AES_WR_START_ADDR_ADDR_POS                          (0U)
#define AES_WR_START_ADDR_ADDR_Len                          (32U)
#define AES_WR_START_ADDR_ADDR_Msk                          (0xFFFFFFFFUL << AES_WR_START_ADDR_ADDR_POS)
#define AES_WR_START_ADDR_ADDR                              AES_WR_START_ADDR_ADDR_Msk

/*******************  Bit definition for AES_KEY_ADDR register  *******************/
#define AES_KEY_ADDR_ADDR_POS                               (0U)
#define AES_KEY_ADDR_ADDR_Len                               (32U)
#define AES_KEY_ADDR_ADDR_Msk                               (0xFFFFFFFFUL << AES_KEY_ADDR_ADDR_POS)
#define AES_KEY_ADDR_ADDR                                   AES_KEY_ADDR_ADDR_Msk

/*******************  Bit definition for AES_DATA_OUT0 register  *******************/
#define AES_DATA_OUT0_DATA_OUT0_POS                         (0U)
#define AES_DATA_OUT0_DATA_OUT0_Len                         (32U)
#define AES_DATA_OUT0_DATA_OUT0_Msk                         (0xFFFFFFFFUL << AES_DATA_OUT0_DATA_OUT0_POS)
#define AES_DATA_OUT0_DATA_OUT0                             AES_DATA_OUT0_DATA_OUT0_Msk

/*******************  Bit definition for AES_DATA_OUT1 register  *******************/
#define AES_DATA_OUT1_DATA_OUT1_POS                         (0U)
#define AES_DATA_OUT1_DATA_OUT1_Len                         (32U)
#define AES_DATA_OUT1_DATA_OUT1_Msk                         (0xFFFFFFFFUL << AES_DATA_OUT1_DATA_OUT1_POS)
#define AES_DATA_OUT1_DATA_OUT1                             AES_DATA_OUT1_DATA_OUT1_Msk

/*******************  Bit definition for AES_DATA_OUT2 register  *******************/
#define AES_DATA_OUT2_DATA_OUT2_POS                         (0U)
#define AES_DATA_OUT2_DATA_OUT2_Len                         (32U)
#define AES_DATA_OUT2_DATA_OUT2_Msk                         (0xFFFFFFFFUL << AES_DATA_OUT2_DATA_OUT2_POS)
#define AES_DATA_OUT2_DATA_OUT2                             AES_DATA_OUT2_DATA_OUT2_Msk

/*******************  Bit definition for AES_DATA_OUT3 register  *******************/
#define AES_DATA_OUT3_DATA_OUT3_POS                         (0U)
#define AES_DATA_OUT3_DATA_OUT3_Len                         (32U)
#define AES_DATA_OUT3_DATA_OUT3_Msk                         (0xFFFFFFFFUL << AES_DATA_OUT3_DATA_OUT3_POS)
#define AES_DATA_OUT3_DATA_OUT3                             AES_DATA_OUT3_DATA_OUT3_Msk

/*******************  Bit definition for AES_KEY0 register  *******************/
#define AES_KEY0_KEY0_POS                                   (0U)
#define AES_KEY0_KEY0_Len                                   (32U)
#define AES_KEY0_KEY0_Msk                                   (0xFFFFFFFFUL << AES_KEY0_KEY0_POS)
#define AES_KEY0_KEY0                                       AES_KEY0_KEY0_Msk

/*******************  Bit definition for AES_KEY1 register  *******************/
#define AES_KEY1_KEY1_POS                                   (0U)
#define AES_KEY1_KEY1_Len                                   (32U)
#define AES_KEY1_KEY1_Msk                                   (0xFFFFFFFFUL << AES_KEY1_KEY1_POS)
#define AES_KEY1_KEY1                                       AES_KEY1_KEY1_Msk

/*******************  Bit definition for AES_KEY2 register  *******************/
#define AES_KEY2_KEY2_POS                                   (0U)
#define AES_KEY2_KEY2_Len                                   (32U)
#define AES_KEY2_KEY2_Msk                                   (0xFFFFFFFFUL << AES_KEY2_KEY2_POS)
#define AES_KEY2_KEY2                                       AES_KEY2_KEY2_Msk

/*******************  Bit definition for AES_KEY3 register  *******************/
#define AES_KEY3_KEY3_POS                                   (0U)
#define AES_KEY3_KEY3_Len                                   (32U)
#define AES_KEY3_KEY3_Msk                                   (0xFFFFFFFFUL << AES_KEY3_KEY3_POS)
#define AES_KEY3_KEY3                                       AES_KEY3_KEY3_Msk

/*******************  Bit definition for AES_KEY4 register  *******************/
#define AES_KEY4_KEY4_POS                                   (0U)
#define AES_KEY4_KEY4_Len                                   (32U)
#define AES_KEY4_KEY4_Msk                                   (0xFFFFFFFFUL << AES_KEY4_KEY4_POS)
#define AES_KEY4_KEY4                                       AES_KEY4_KEY4_Msk

/*******************  Bit definition for AES_KEY5 register  *******************/
#define AES_KEY5_KEY5_POS                                   (0U)
#define AES_KEY5_KEY5_Len                                   (32U)
#define AES_KEY5_KEY5_Msk                                   (0xFFFFFFFFUL << AES_KEY5_KEY5_POS)
#define AES_KEY5_KEY5                                       AES_KEY5_KEY5_Msk

/*******************  Bit definition for AES_KEY6 register  *******************/
#define AES_KEY6_KEY6_POS                                   (0U)
#define AES_KEY6_KEY6_Len                                   (32U)
#define AES_KEY6_KEY6_Msk                                   (0xFFFFFFFFUL << AES_KEY6_KEY6_POS)
#define AES_KEY6_KEY6                                       AES_KEY6_KEY6_Msk

/*******************  Bit definition for AES_KEY7 register  *******************/
#define AES_KEY7_KEY7_POS                                   (0U)
#define AES_KEY7_KEY7_Len                                   (32U)
#define AES_KEY7_KEY7_Msk                                   (0xFFFFFFFFUL << AES_KEY7_KEY7_POS)
#define AES_KEY7_KEY7                                       AES_KEY7_KEY7_Msk

/*******************  Bit definition for AES_INIT_SSI register  *******************/
#define AES_INIT_SSI_SEED_POS                               (0U)
#define AES_INIT_SSI_SEED_Len                               (32U)
#define AES_INIT_SSI_SEED_Msk                               (0xFFFFFFFFUL << AES_INIT_SSI_SEED_POS)
#define AES_INIT_SSI_SEED                                   AES_INIT_SSI_SEED_Msk

/*******************  Bit definition for AES_INIT_SSO register  *******************/
#define AES_INIT_SSO_SEED_POS                               (0U)
#define AES_INIT_SSO_SEED_Len                               (32U)
#define AES_INIT_SSO_SEED_Msk                               (0xFFFFFFFFUL << AES_INIT_SSO_SEED_POS)
#define AES_INIT_SSO_SEED                                   AES_INIT_SSO_SEED_Msk

/*******************  Bit definition for AES_MASK_SSI register  *******************/
#define AES_MASK_SSI_MASK_POS                               (0U)
#define AES_MASK_SSI_MASK_Len                               (32U)
#define AES_MASK_SSI_MASK_Msk                               (0xFFFFFFFFUL << AES_MASK_SSI_MASK_POS)
#define AES_MASK_SSI_MASK                                   AES_MASK_SSI_MASK_Msk

/*******************  Bit definition for AES_MASK_SSO register  *******************/
#define AES_MASK_SSO_MASK_POS                               (0U)
#define AES_MASK_SSO_MASK_Len                               (32U)
#define AES_MASK_SSO_MASK_Msk                               (0xFFFFFFFFUL << AES_MASK_SSO_MASK_POS)
#define AES_MASK_SSO_MASK                                   AES_MASK_SSO_MASK_Msk

/*******************  Bit definition for AES_INIT_V0 register  *******************/
#define AES_INIT_V0_VECTOR_POS                              (0U)
#define AES_INIT_V0_VECTOR_Len                              (32U)
#define AES_INIT_V0_VECTOR_Msk                              (0xFFFFFFFFUL << AES_INIT_V0_VECTOR_POS)
#define AES_INIT_V0_VECTOR                                  AES_INIT_V0_VECTOR_Msk

/*******************  Bit definition for AES_INIT_V1 register  *******************/
#define AES_INIT_V1_VECTOR_POS                              (0U)
#define AES_INIT_V1_VECTOR_Len                              (32U)
#define AES_INIT_V1_VECTOR_Msk                              (0xFFFFFFFFUL << AES_INIT_V1_VECTOR_POS)
#define AES_INIT_V1_VECTOR                                  AES_INIT_V1_VECTOR_Msk

/*******************  Bit definition for AES_INIT_V2 register  *******************/
#define AES_INIT_V2_VECTOR_POS                              (0U)
#define AES_INIT_V2_VECTOR_Len                              (32U)
#define AES_INIT_V2_VECTOR_Msk                              (0xFFFFFFFFUL << AES_INIT_V2_VECTOR_POS)
#define AES_INIT_V2_VECTOR                                  AES_INIT_V2_VECTOR_Msk

/*******************  Bit definition for AES_INIT_V3 register  *******************/
#define AES_INIT_V3_VECTOR_POS                              (0U)
#define AES_INIT_V3_VECTOR_Len                              (32U)
#define AES_INIT_V3_VECTOR_Msk                              (0xFFFFFFFFUL << AES_INIT_V3_VECTOR_POS)
#define AES_INIT_V3_VECTOR                                  AES_INIT_V3_VECTOR_Msk

/*******************  Bit definition for AES_DATA_IN0 register  *******************/
#define AES_DATA_IN0_DATA_IN0_POS                           (0U)
#define AES_DATA_IN0_DATA_IN0_Len                           (32U)
#define AES_DATA_IN0_DATA_IN0_Msk                           (0xFFFFFFFFUL << AES_DATA_IN0_DATA_IN0_POS)
#define AES_DATA_IN0_DATA_IN0                               AES_DATA_IN0_DATA_IN0_Msk

/*******************  Bit definition for AES_DATA_IN1 register  *******************/
#define AES_DATA_IN1_DATA_IN1_POS                           (0U)
#define AES_DATA_IN1_DATA_IN1_Len                           (32U)
#define AES_DATA_IN1_DATA_IN1_Msk                           (0xFFFFFFFFUL << AES_DATA_IN1_DATA_IN1_POS)
#define AES_DATA_IN1_DATA_IN1                               AES_DATA_IN1_DATA_IN1_Msk

/*******************  Bit definition for AES_DATA_IN2 register  *******************/
#define AES_DATA_IN2_DATA_IN2_POS                           (0U)
#define AES_DATA_IN2_DATA_IN2_Len                           (32U)
#define AES_DATA_IN2_DATA_IN2_Msk                           (0xFFFFFFFFUL << AES_DATA_IN2_DATA_IN2_POS)
#define AES_DATA_IN2_DATA_IN2                               AES_DATA_IN2_DATA_IN2_Msk

/*******************  Bit definition for AES_DATA_IN3 register  *******************/
#define AES_DATA_IN3_DATA_IN3_POS                           (0U)
#define AES_DATA_IN3_DATA_IN3_Len                           (32U)
#define AES_DATA_IN3_DATA_IN3_Msk                           (0xFFFFFFFFUL << AES_DATA_IN3_DATA_IN3_POS)
#define AES_DATA_IN3_DATA_IN3                               AES_DATA_IN3_DATA_IN3_Msk

/*******************  Bit definition for AES_KEYPORT_MASK register  *******************/
#define AES_KEYPORT_MASK_MASK_POS                           (0U)
#define AES_KEYPORT_MASK_MASK_Len                           (32U)
#define AES_KEYPORT_MASK_MASK_Msk                           (0xFFFFFFFFUL << AES_KEYPORT_MASK_MASK_POS)
#define AES_KEYPORT_MASK_MASK                               AES_KEYPORT_MASK_MASK_Msk

/* ================================================================================================================= */
/* ================                                        COMP                                     ================ */
/* ================================================================================================================= */

/*******************  Bit definition for COMP_REG_0 register  **********/

#define AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Pos                     (20U)
#define AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Len                     (4U)
#define AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Msk                     (0xFU << AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Pos)
#define AON_PMU_COMP_REG_0_CHANNEL_SEL_N                         AON_PMU_COMP_REG_0_CHANNEL_SEL_N_Msk

#define AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Pos                     (16U)
#define AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Len                     (4U)
#define AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Msk                     (0xFU << AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Pos)
#define AON_PMU_COMP_REG_0_CHANNEL_SEL_P                         AON_PMU_COMP_REG_0_CHANNEL_SEL_P_Msk

#define AON_PMU_COMP_REG_0_COMP_REF_CTRL_LV_Pos                  (8U)
#define AON_PMU_COMP_REG_0_COMP_REF_CTRL_LV_Len                  (8U)
#define AON_PMU_COMP_REG_0_COMP_REF_CTRL_LV_Msk                  (0xFFU << AON_PMU_COMP_REG_0_COMP_REF_CTRL_LV_Pos)
#define AON_PMU_COMP_REG_0_COMP_REF_CTRL_LV                      AON_PMU_COMP_REG_0_COMP_REF_CTRL_LV_Msk

#define AON_PMU_COMP_REG_0_WAKE_COMP_EN_Pos                      (7U)
#define AON_PMU_COMP_REG_0_WAKE_COMP_EN_Len                      (1U)
#define AON_PMU_COMP_REG_0_WAKE_COMP_EN_Msk                      (0x1U << AON_PMU_COMP_REG_0_WAKE_COMP_EN_Pos)
#define AON_PMU_COMP_REG_0_WAKE_COMP_EN                          AON_PMU_COMP_REG_0_WAKE_COMP_EN_Msk

#define AON_PMU_COMP_REG_0_COMP_BATT_LVL_CTRL_LV_Pos             (4U)
#define AON_PMU_COMP_REG_0_COMP_BATT_LVL_CTRL_LV_Len             (3U)
#define AON_PMU_COMP_REG_0_COMP_BATT_LVL_CTRL_LV_Msk             (0x7U << AON_PMU_COMP_REG_0_COMP_BATT_LVL_CTRL_LV_Pos)
#define AON_PMU_COMP_REG_0_COMP_BATT_LVL_CTRL_LV                 AON_PMU_COMP_REG_0_COMP_BATT_LVL_CTRL_LV_Msk

#define AON_PMU_COMP_REG_0_ICOMP_CTRL_LV_Pos                     (0U)
#define AON_PMU_COMP_REG_0_ICOMP_CTRL_LV_Len                     (4U)
#define AON_PMU_COMP_REG_0_ICOMP_CTRL_LV_Msk                     (0xFU << AON_PMU_COMP_REG_0_ICOMP_CTRL_LV_Pos)
#define AON_PMU_COMP_REG_0_ICOMP_CTRL_LV                         AON_PMU_COMP_REG_0_ICOMP_CTRL_LV_Msk

/*******************  Bit definition for COMP_REG_1 register  **********/

#define AON_PMU_COMP_REG_1_COMP_VREF_EN_Pos                              (15U)
#define AON_PMU_COMP_REG_1_COMP_VREF_Len                                 (1U)
#define AON_PMU_COMP_REG_1_COMP_VREF_EN_Msk                              (0x1U << AON_PMU_COMP_REG_1_COMP_VREF_EN_Pos)
#define AON_PMU_COMP_REG_1_COMP_VREF_EN                                   AON_PMU_COMP_REG_1_COMP_VREF_EN_Msk

#define AON_PMU_COMP_REG_1_COMP_VBAT_EN_Pos                              (14U)
#define AON_PMU_COMP_REG_1_COMP_VBAT_EN_Len                              (1U)
#define AON_PMU_COMP_REG_1_COMP_VBAT_EN_Msk                              (0x1U << AON_PMU_COMP_REG_1_COMP_VBAT_EN_Pos)
#define AON_PMU_COMP_REG_1_COMP_VBAT_EN                                  AON_PMU_COMP_REG_1_COMP_VBAT_EN_Msk

#define AON_PMU_COMP_REG_1_COMP_CASCRES_HALF_CTRL_Pos                    (13U)
#define AON_PMU_COMP_REG_1_COMP_CASCRES_HALF_CTRL_Len                    (1U)
#define AON_PMU_COMP_REG_1_COMP_CASCRES_HALF_CTRL_Msk                    (0x1U << AON_PMU_COMP_REG_1_COMP_CASCRES_HALF_CTRL_Pos)
#define AON_PMU_COMP_REG_1_COMP_CASCRES_HALF_CTRL                        AON_PMU_COMP_REG_1_COMP_CASCRES_HALF_CTRL_Msk

#define AON_PMU_COMP_REG_1_CLK_COMP_EN_Pos                               (12U)
#define AON_PMU_COMP_REG_1_CLK_COMP_EN_Len                               (1U)
#define AON_PMU_COMP_REG_1_CLK_COMP_EN_Msk                               (0x1U << AON_PMU_COMP_REG_1_CLK_COMP_EN_Pos)
#define AON_PMU_COMP_REG_1_CLK_COMP_EN                                   AON_PMU_COMP_REG_1_CLK_COMP_EN_Msk

#define AON_PMU_COMP_REG_1_CLK_COMP_EDGE_Pos                             (11U)
#define AON_PMU_COMP_REG_1_CLK_COMP_EDGE_Len                             (1U)
#define AON_PMU_COMP_REG_1_CLK_COMP_EDGE_Msk                             (0x1U << AON_PMU_COMP_REG_1_CLK_COMP_EDGE_Pos)
#define AON_PMU_COMP_REG_1_CLK_COMP_EDGE                                 AON_PMU_COMP_REG_1_CLK_COMP_EDGE_Msk

#define AON_PMU_COMP_REG_1_CLK_COMP_250K_Pos                             (10U)
#define AON_PMU_COMP_REG_1_CLK_COMP_250K_Len                             (1U)
#define AON_PMU_COMP_REG_1_CLK_COMP_250K_Msk                             (0x1U << AON_PMU_COMP_REG_1_CLK_COMP_250K_Pos)
#define AON_PMU_COMP_REG_1_CLK_COMP_250K                                 AON_PMU_COMP_REG_1_CLK_COMP_250K_Msk

#define AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_HYST_Pos                     (9U)
#define AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_HYST_Len                     (1U)
#define AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_HYST_Msk                     (0x1U << AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_HYST_Pos)
#define AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_HYST                        AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_HYST_Msk

#define AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_HYST_Pos                     (8U)
#define AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_HYST_Len                     (1U)
#define AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_HYST_Msk                     (0x1U << AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_HYST_Pos)
#define AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_HYST                        AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_HYST_Msk

#define AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_RES_DEGENERATION_Pos         (4U)
#define AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_RES_DEGENERATION_Len         (4U)
#define AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_RES_DEGENERATION_Msk         (0xFU << AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_RES_DEGENERATION_Pos)
#define AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_RES_DEGENERATION             AON_PMU_COMP_REG_1_CHANNEL_NEGATIVE_RES_DEGENERATION_Msk

#define AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_RES_DEGENERATION_Pos          (0U)
#define AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_RES_DEGENERATION_Len          (4U)
#define AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_RES_DEGENERATION_Msk          (0xFU << AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_RES_DEGENERATION_Pos)
#define AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_RES_DEGENERATION              AON_PMU_COMP_REG_1_CHANNEL_POSITIVE_RES_DEGENERATION_Msk

/* ================================================================================================================= */
/* ================                                        DMA                                      ================ */
/* ================================================================================================================= */

/*******************  Bit definition for DMA_SAR register  ********************/
#define DMA_SAR_CSA_Pos                                     (0U)
#define DMA_SAR_CSA_Len                                     (32U)
#define DMA_SAR_CSA_Msk                                     (0xFFFFFFFFU)
#define DMA_SAR_CSA                                         DMA_SAR_CSA_Msk

/*******************  Bit definition for DMA_DAR register  ********************/
#define DMA_DAR_CDA_Pos                                     (0U)
#define DMA_DAR_CDA_Len                                     (32U)
#define DMA_DAR_CDA_Msk                                     (0xFFFFFFFFU)
#define DMA_DAR_CDA                                         DMA_DAR_CDA_Msk

/*******************  Bit definition for DMA_LLP register  ********************/
#define DMA_LLP_LMS_Pos                                     (0U)
#define DMA_LLP_LMS_Len                                     (2U)
#define DMA_LLP_LMS_Msk                                     (0x3U << DMA_LLP_LMS_Pos)
#define DMA_LLP_LMS                                         DMA_LLP_LMS_Msk

#define DMA_LLP_LOC_Pos                                     (2U)
#define DMA_LLP_LOC_Len                                     (30U)
#define DMA_LLP_LOC_Msk                                     (0x3FFFFFFFU << DMA_LLP_LOC_Pos)
#define DMA_LLP_LOC                                         DMA_LLP_LOC_Msk

/*******************  Bit definition for DMA_CTLL register  *******************/
#define DMA_CTLL_LLP_SRC_EN_Pos                             (28U)
#define DMA_CTLL_LLP_SRC_EN_Len                             (1U)
#define DMA_CTLL_LLP_SRC_EN_Msk                             (0x1U << DMA_CTLL_LLP_SRC_EN_Pos)
#define DMA_CTLL_LLP_SRC_EN                                 DMA_CTLL_LLP_SRC_EN_Msk
#define DMA_CTLL_LLP_SRC_EN_ENABLE                          (0x1U << DMA_CTLL_LLP_SRC_EN_Pos)
#define DMA_CTLL_LLP_SRC_EN_DISABLE                         (0x0U << DMA_CTLL_LLP_SRC_EN_Pos)

#define DMA_CTLL_LLP_DST_EN_Pos                             (27U)
#define DMA_CTLL_LLP_DST_EN_Len                             (1U)
#define DMA_CTLL_LLP_DST_EN_Msk                             (0x1U << DMA_CTLL_LLP_DST_EN_Pos)
#define DMA_CTLL_LLP_DST_EN                                 DMA_CTLL_LLP_DST_EN_Msk
#define DMA_CTLL_LLP_DST_EN_ENABLE                          (0x1U << DMA_CTLL_LLP_DST_EN_Pos)
#define DMA_CTLL_LLP_DST_EN_DISABLE                         (0x0U << DMA_CTLL_LLP_DST_EN_Pos)

#define DMA_CTLL_TT_FC_Pos                                  (20U)
#define DMA_CTLL_TT_FC_Len                                  (2U)
#define DMA_CTLL_TT_FC_Msk                                  (0x3U << DMA_CTLL_TT_FC_Pos)
#define DMA_CTLL_TT_FC                                      DMA_CTLL_TT_FC_Msk
#define DMA_CTLL_TT_FC_M2M                                  (0x0U << DMA_CTLL_TT_FC_Pos)
#define DMA_CTLL_TT_FC_M2P                                  (0x1U << DMA_CTLL_TT_FC_Pos)
#define DMA_CTLL_TT_FC_P2M                                  (0x2U << DMA_CTLL_TT_FC_Pos)
#define DMA_CTLL_TT_FC_P2P                                  (0x3U << DMA_CTLL_TT_FC_Pos)

#define DMA_CTLL_DST_SCATTER_EN_Pos                         (18U)
#define DMA_CTLL_DST_SCATTER_EN_Len                         (1U)
#define DMA_CTLL_DST_SCATTER_EN_Msk                         (0x1U << DMA_CTLL_DST_SCATTER_EN_Pos)
#define DMA_CTLL_DST_SCATTER_EN                             DMA_CTLL_DST_SCATTER_EN_Msk
#define DMA_CTLL_DST_SCATTER_EN_ENABLE                      (0x1U << DMA_CTLL_DST_SCATTER_EN_Pos)
#define DMA_CTLL_DST_SCATTER_EN_DISABLE                     (0x0U << DMA_CTLL_DST_SCATTER_EN_Pos)

#define DMA_CTLL_SRC_GATHER_EN_Pos                          (17U)
#define DMA_CTLL_SRC_GATHER_EN_Len                          (1U)
#define DMA_CTLL_SRC_GATHER_EN_Msk                          (0x1U << DMA_CTLL_SRC_GATHER_EN_Pos)
#define DMA_CTLL_SRC_GATHER_EN                              DMA_CTLL_SRC_GATHER_EN_Msk
#define DMA_CTLL_SRC_GATHER_EN_ENABLE                       (0x1U << DMA_CTLL_SRC_GATHER_EN_Pos)
#define DMA_CTLL_SRC_GATHER_EN_DISABLE                      (0x0U << DMA_CTLL_SRC_GATHER_EN_Pos)

#define DMA_CTLL_SRC_MSIZE_Pos                              (14U)
#define DMA_CTLL_SRC_MSIZE_Len                              (3U)
#define DMA_CTLL_SRC_MSIZE_Msk                              (0x7U << DMA_CTLL_SRC_MSIZE_Pos)
#define DMA_CTLL_SRC_MSIZE                                  DMA_CTLL_SRC_MSIZE_Msk
#define DMA_CTLL_SRC_MSIZE_1                                (0x0U << DMA_CTLL_SRC_MSIZE_Pos)
#define DMA_CTLL_SRC_MSIZE_4                                (0x1U << DMA_CTLL_SRC_MSIZE_Pos)
#define DMA_CTLL_SRC_MSIZE_8                                (0x2U << DMA_CTLL_SRC_MSIZE_Pos)
#define DMA_CTLL_SRC_MSIZE_16                               (0x3U << DMA_CTLL_SRC_MSIZE_Pos)
#define DMA_CTLL_SRC_MSIZE_32                               (0x4U << DMA_CTLL_SRC_MSIZE_Pos)
#define DMA_CTLL_SRC_MSIZE_64                               (0x5U << DMA_CTLL_SRC_MSIZE_Pos)
#define DMA_CTLL_SRC_MSIZE_128                              (0x6U << DMA_CTLL_SRC_MSIZE_Pos)
#define DMA_CTLL_SRC_MSIZE_256                              (0x7U << DMA_CTLL_SRC_MSIZE_Pos)

#define DMA_CTLL_DST_MSIZE_Pos                              (11U)
#define DMA_CTLL_DST_MSIZE_Len                              (3U)
#define DMA_CTLL_DST_MSIZE_Msk                              (0x7U << DMA_CTLL_DST_MSIZE_Pos)
#define DMA_CTLL_DST_MSIZE                                  DMA_CTLL_DST_MSIZE_Msk
#define DMA_CTLL_DST_MSIZE_1                                (0x0U << DMA_CTLL_DST_MSIZE_Pos)
#define DMA_CTLL_DST_MSIZE_4                                (0x1U << DMA_CTLL_DST_MSIZE_Pos)
#define DMA_CTLL_DST_MSIZE_8                                (0x2U << DMA_CTLL_DST_MSIZE_Pos)
#define DMA_CTLL_DST_MSIZE_16                               (0x3U << DMA_CTLL_DST_MSIZE_Pos)
#define DMA_CTLL_DST_MSIZE_32                               (0x4U << DMA_CTLL_DST_MSIZE_Pos)
#define DMA_CTLL_DST_MSIZE_64                               (0x5U << DMA_CTLL_DST_MSIZE_Pos)
#define DMA_CTLL_DST_MSIZE_128                              (0x6U << DMA_CTLL_DST_MSIZE_Pos)
#define DMA_CTLL_DST_MSIZE_256                              (0x7U << DMA_CTLL_DST_MSIZE_Pos)

#define DMA_CTLL_SINC_Pos                                   (9U)
#define DMA_CTLL_SINC_Len                                   (2U)
#define DMA_CTLL_SINC_Msk                                   (0x3U << DMA_CTLL_SINC_Pos)
#define DMA_CTLL_SINC                                       DMA_CTLL_SINC_Msk
#define DMA_CTLL_SINC_INC                                   (0x0U << DMA_CTLL_SINC_Pos)
#define DMA_CTLL_SINC_DEC                                   (0x1U << DMA_CTLL_SINC_Pos)
#define DMA_CTLL_SINC_NO                                    (0x2U << DMA_CTLL_SINC_Pos)

#define DMA_CTLL_DINC_Pos                                   (7U)
#define DMA_CTLL_DINC_Len                                   (2U)
#define DMA_CTLL_DINC_Msk                                   (0x3U << DMA_CTLL_DINC_Pos)
#define DMA_CTLL_DINC                                       DMA_CTLL_DINC_Msk
#define DMA_CTLL_DINC_INC                                   (0x0U << DMA_CTLL_DINC_Pos)
#define DMA_CTLL_DINC_DEC                                   (0x1U << DMA_CTLL_DINC_Pos)
#define DMA_CTLL_DINC_NO                                    (0x2U << DMA_CTLL_DINC_Pos)

#define DMA_CTLL_SRC_TR_WIDTH_Pos                           (4U)
#define DMA_CTLL_SRC_TR_WIDTH_Len                           (2U)
#define DMA_CTLL_SRC_TR_WIDTH_Msk                           (0x3U << DMA_CTLL_SRC_TR_WIDTH_Pos)
#define DMA_CTLL_SRC_TR_WIDTH                               DMA_CTLL_SRC_TR_WIDTH_Msk
#define DMA_CTLL_SRC_TR_WIDTH_8                             (0x0U << DMA_CTLL_SRC_TR_WIDTH_Pos)
#define DMA_CTLL_SRC_TR_WIDTH_16                            (0x1U << DMA_CTLL_SRC_TR_WIDTH_Pos)
#define DMA_CTLL_SRC_TR_WIDTH_32                            (0x2U << DMA_CTLL_SRC_TR_WIDTH_Pos)

#define DMA_CTLL_DST_TR_WIDTH_Pos                           (1U)
#define DMA_CTLL_DST_TR_WIDTH_Len                           (2U)
#define DMA_CTLL_DST_TR_WIDTH_Msk                           (0x3U << DMA_CTLL_DST_TR_WIDTH_Pos)
#define DMA_CTLL_DST_TR_WIDTH                               DMA_CTLL_DST_TR_WIDTH_Msk
#define DMA_CTLL_DST_TR_WIDTH_8                             (0x0U << DMA_CTLL_DST_TR_WIDTH_Pos)
#define DMA_CTLL_DST_TR_WIDTH_16                            (0x1U << DMA_CTLL_DST_TR_WIDTH_Pos)
#define DMA_CTLL_DST_TR_WIDTH_32                            (0x2U << DMA_CTLL_DST_TR_WIDTH_Pos)

#define DMA_CTLL_INT_EN_Pos                                 (0U)
#define DMA_CTLL_INT_EN_Len                                 (1U)
#define DMA_CTLL_INT_EN_Msk                                 (0x1U << DMA_CTLL_INT_EN_Pos)
#define DMA_CTLL_INI_EN                                     DMA_CTLL_INT_EN_Msk

/*******************  Bit definition for DMA_CTLH register  *******************/
#define DMA_CTLH_BLOCK_TS_Pos                               (0U)
#define DMA_CTLH_BLOCK_TS_Len                               (12U)
#define DMA_CTLH_BLOCK_TS_Msk                               (0xFFFU << DMA_CTLH_BLOCK_TS_Pos)
#define DMA_CTLH_BLOCK_TS                                   DMA_CTLH_BLOCK_TS_Msk

/*******************  Bit definition for DMA_SSTAT register  *******************/
#define DMA_SSTAT_SSTAT_Pos                                 (0U)
#define DMA_SSTAT_SSTAT_Len                                 (32U)
#define DMA_SSTAT_SSTAT_Msk                                 (0xFFFFFFFFU << DMA_SSTAT_SSTAT_Pos)
#define DMA_SSTAT_SSTAT                                     DMA_SSTAT_SSTAT_Msk

/*******************  Bit definition for DMA_DSTAT register  *******************/
#define DMA_DSTAT_DSTAT_Pos                                 (0U)
#define DMA_DSTAT_DSTAT_Len                                 (32U)
#define DMA_DSTAT_DSTAT_Msk                                 (0xFFFFFFFFU << DMA_DSTAT_DSTAT_Pos)
#define DMA_DSTAT_DSTAT                                     DMA_DSTAT_DSTAT_Msk

/*******************  Bit definition for DMA_SSTATAR register  *******************/
#define DMA_SSTATAR_SSTATAR_Pos                             (0U)
#define DMA_SSTATAR_SSTATAR_Len                             (32U)
#define DMA_SSTATAR_SSTATAR_Msk                             (0xFFFFFFFFU << DMA_SSTATAR_SSTATAR_Pos)
#define DMA_SSTATAR_SSTATAR                                 DMA_SSTATAR_SSTATAR_Msk

/*******************  Bit definition for DMA_DSTATAR register  *******************/
#define DMA_DSTATAR_DSTATAR_Pos                             (0U)
#define DMA_DSTATAR_DSTATAR_Len                             (32U)
#define DMA_DSTATAR_DSTATAR_Msk                             (0xFFFFFFFFU << DMA_DSTATAR_DSTATAR_Pos)
#define DMA_DSTATAR_DSTATAR                                 DMA_DSTATAR_DSTATAR_Msk

/*******************  Bit definition for DMA_CFGL register  *******************/
#define DMA_CFGL_RELOAD_DST_Pos                             (31U)
#define DMA_CFGL_RELOAD_DST_Len                             (1U)
#define DMA_CFGL_RELOAD_DST_Msk                             (0x1U << DMA_CFGL_RELOAD_DST_Pos)
#define DMA_CFGL_RELOAD_DST                                 DMA_CFGL_RELOAD_DST_Msk

#define DMA_CFGL_RELOAD_SRC_Pos                             (30U)
#define DMA_CFGL_RELOAD_SRC_Len                             (1U)
#define DMA_CFGL_RELOAD_SRC_Msk                             (0x1U << DMA_CFGL_RELOAD_SRC_Pos)
#define DMA_CFGL_RELOAD_SRC                                 DMA_CFGL_RELOAD_SRC_Msk

#define DMA_CFGL_MAX_ABRST_Pos                              (20U)
#define DMA_CFGL_MAX_ABRST_Len                              (10U)
#define DMA_CFGL_MAX_ABRST_Msk                              (0x3FFU << DMA_CFGL_MAX_ABRST_Pos)
#define DMA_CFGL_MAX_ABRST                                  DMA_CFGL_MAX_ABRST_Msk

#define DMA_CFGL_HS_SEL_SRC_Pos                             (11U)
#define DMA_CFGL_HS_SEL_SRC_Len                             (1U)
#define DMA_CFGL_HS_SEL_SRC_Msk                             (0x1U << DMA_CFGL_HS_SEL_SRC_Pos)
#define DMA_CFGL_HS_SEL_SRC                                 DMA_CFGL_HS_SEL_SRC_Msk

#define DMA_CFGL_HS_SEL_DST_Pos                             (10U)
#define DMA_CFGL_HS_SEL_DST_Len                             (1U)
#define DMA_CFGL_HS_SEL_DST_Msk                             (0x1U << DMA_CFGL_HS_SEL_DST_Pos)
#define DMA_CFGL_HS_SEL_DST                                 DMA_CFGL_HS_SEL_DST_Msk

#define DMA_CFGL_FIFO_EMPTY_Pos                             (9U)
#define DMA_CFGL_FIFO_EMPTY_Len                             (1U)
#define DMA_CFGL_FIFO_EMPTY_Msk                             (0x1U << DMA_CFGL_FIFO_EMPTY_Pos)
#define DMA_CFGL_FIFO_EMPTY                                 DMA_CFGL_FIFO_EMPTY_Msk

#define DMA_CFGL_CH_SUSP_Pos                                (8U)
#define DMA_CFGL_CH_SUSP_Len                                (1U)
#define DMA_CFGL_CH_SUSP_Msk                                (0x1U << DMA_CFGL_CH_SUSP_Pos)
#define DMA_CFGL_CH_SUSP                                    DMA_CFGL_CH_SUSP_Msk

#define DMA_CFGL_CH_PRIOR_Pos                               (5U)
#define DMA_CFGL_CH_PRIOR_Len                               (3U)
#define DMA_CFGL_CH_PRIOR_Msk                               (0x7U << DMA_CFGL_CH_PRIOR_Pos)
#define DMA_CFGL_CH_PRIOR                                   DMA_CFGL_CH_PRIOR_Msk
#define DMA_CFGL_CH_PRIOR_0                                 (0x0U << DMA_CFGL_CH_PRIOR_Pos)
#define DMA_CFGL_CH_PRIOR_1                                 (0x1U << DMA_CFGL_CH_PRIOR_Pos)
#define DMA_CFGL_CH_PRIOR_2                                 (0x2U << DMA_CFGL_CH_PRIOR_Pos)
#define DMA_CFGL_CH_PRIOR_3                                 (0x3U << DMA_CFGL_CH_PRIOR_Pos)
#define DMA_CFGL_CH_PRIOR_4                                 (0x4U << DMA_CFGL_CH_PRIOR_Pos)
#define DMA_CFGL_CH_PRIOR_5                                 (0x5U << DMA_CFGL_CH_PRIOR_Pos)
#define DMA_CFGL_CH_PRIOR_6                                 (0x6U << DMA_CFGL_CH_PRIOR_Pos)
#define DMA_CFGL_CH_PRIOR_7                                 (0x7U << DMA_CFGL_CH_PRIOR_Pos)

/*******************  Bit definition for DMA_CFGH register  ********************/
#define DMA_CFGH_DST_PER_Pos                                (11U)
#define DMA_CFGH_DST_PER_Len                                (4U)
#define DMA_CFGH_DST_PER_Msk                                (0xFU << DMA_CFGH_DST_PER_Pos)
#define DMA_CFGH_DST_PER                                    DMA_CFGH_DST_PER_Msk

#define DMA_CFGH_SRC_PER_Pos                                (7U)
#define DMA_CFGH_SRC_PER_Len                                (4U)
#define DMA_CFGH_SRC_PER_Msk                                (0xFU << DMA_CFGH_SRC_PER_Pos)
#define DMA_CFGH_SRC_PER                                    DMA_CFGH_SRC_PER_Msk

#define DMA_CFGH_SS_UPD_EN_Pos                              (6U)
#define DMA_CFGH_SS_UPD_EN_Len                              (1U)
#define DMA_CFGH_SS_UPD_EN_Msk                              (0x1U << DMA_CFGH_SS_UPD_EN_Pos)
#define DMA_CFGH_SS_UPD_EN                                  DMA_CFGH_SS_UPD_EN_Msk
#define DMA_CFGH_SS_UPD_ENABLE                              (0x1U << DMA_CFGH_SS_UPD_EN_Pos)
#define DMA_CFGH_SS_UPD_DISABLE                             (0x0U << DMA_CFGH_SS_UPD_EN_Pos)

#define DMA_CFGH_DS_UPD_EN_Pos                              (5U)
#define DMA_CFGH_DS_UPD_EN_Len                              (1U)
#define DMA_CFGH_DS_UPD_EN_Msk                              (0x1U << DMA_CFGH_DS_UPD_EN_Pos)
#define DMA_CFGH_DS_UPD_EN                                  DMA_CFGH_DS_UPD_EN_Msk
#define DMA_CFGH_DS_UPD_ENABLE                              (0x1U << DMA_CFGH_DS_UPD_EN_Pos)
#define DMA_CFGH_DS_UPD_DISABLE                             (0x0U << DMA_CFGH_DS_UPD_EN_Pos)

#define DMA_CFGH_PROTCTL_Pos                                (2U)
#define DMA_CFGH_PROTCTL_Len                                (3U)
#define DMA_CFGH_PROTCTL_Msk                                (0x7U << DMA_CFGH_PROTCTL_Pos)
#define DMA_CFGH_PROTCTL                                    DMA_CFGH_PROTCTL_Msk

#define DMA_CFGH_FIFO_MODE_Pos                              (2U)
#define DMA_CFGH_FIFO_MODE_Len                              (1U)
#define DMA_CFGH_FIFO_MODE_Msk                              (0x1U << DMA_CFGH_FIFO_MODE_Pos)
#define DMA_CFGH_FIFO_MODE                                  DMA_CFGH_FIFO_MODE_Msk

/*******************  Bit definition for DMA_SGR register  *******************/
#define DMA_SGR_SGI_Pos                                     (0U)
#define DMA_SGR_SGI_Len                                     (20U)
#define DMA_SGR_SGI_Msk                                     (0xFFFFFU << DMA_SGR_SGI_Pos)
#define DMA_SGR_SGI                                         DMA_SGR_SGI_Msk

#define DMA_SGR_SGC_Pos                                     (20U)
#define DMA_SGR_SGC_Len                                     (12U)
#define DMA_SGR_SGC_Msk                                     (0xFFFU << DMA_SGR_SGC_Pos)
#define DMA_SGR_SGC                                         DMA_SGR_SGC_Msk

/*******************  Bit definition for DMA_DSR register  *******************/
#define DMA_DSR_DSI_Pos                                     (0U)
#define DMA_DSR_DSI_Len                                     (20U)
#define DMA_DSR_DSI_Msk                                     (0xFFFFFU << DMA_DSR_DSI_Pos)
#define DMA_DSR_DSI                                         DMA_DSR_DSI_Msk

#define DMA_DSR_DSC_Pos                                     (20U)
#define DMA_DSR_DSC_Len                                     (12U)
#define DMA_DSR_DSC_Msk                                     (0xFFFU << DMA_DSR_DSC_Pos)
#define DMA_DSR_DSC                                         DMA_DSR_DSC_Msk

/*******************  Bit definition for DMA_RAW_TFR register  *****************/
#define DMA_RAW_TFR_Pos                                     (0U)
#define DMA_RAW_TFR_Len                                     (8U)
#define DMA_RAW_TFR_Msk                                     (0xFFU << DMA_RAW_TFR_Pos)
#define DMA_RAW_TFR                                         DMA_RAW_TFR_Msk

/*******************  Bit definition for DMA_RAW_BLK register  *****************/
#define DMA_RAW_BLK_Pos                                     (0U)
#define DMA_RAW_BLK_Len                                     (8U)
#define DMA_RAW_BLK_Msk                                     (0xFFU << DMA_RAW_BLK_Pos)
#define DMA_RAW_BLK                                         DMA_RAW_BLK_Msk

/*******************  Bit definition for DMA_RAW_SRC_TRN register  *************/
#define DMA_RAW_SRC_TRN_Pos                                 (0U)
#define DMA_RAW_SRC_TRN_Len                                 (8U)
#define DMA_RAW_SRC_TRN_Msk                                 (0xFFU << DMA_RAW_SRC_TRN_Pos)
#define DMA_RAW_SRC_TRN                                     DMA_RAW_SRC_TRN_Msk

/*******************  Bit definition for DMA_RAW_DST_TRN register  *************/
#define DMA_RAW_DST_TRN_Pos                                 (0U)
#define DMA_RAW_DST_TRN_Len                                 (8U)
#define DMA_RAW_DST_TRN_Msk                                 (0xFFU << DMA_RAW_DST_TRN_Pos)
#define DMA_RAW_DST_TRN                                     DMA_RAW_DST_TRN_Msk

/*******************  Bit definition for DMA_RAW_ERR register  *****************/
#define DMA_RAW_ERR_Pos                                     (0U)
#define DMA_RAW_ERR_Len                                     (8U)
#define DMA_RAW_ERR_Msk                                     (0xFFU << DMA_RAW_ERR_Pos)
#define DMA_RAW_ERR                                         DMA_RAW_ERR_Msk

/*******************  Bit definition for DMA_STAT_TFR register  ****************/
#define DMA_STAT_TFR_Pos                                    (0U)
#define DMA_STAT_TFR_Len                                    (8U)
#define DMA_STAT_TFR_Msk                                    (0xFFUL << DMA_STAT_TFR_Pos)
#define DMA_STAT_TFR                                        DMA_STAT_TFR_Msk

/*******************  Bit definition for DMA_STAT_BLK register  ****************/
#define DMA_STAT_BLK_Pos                                    (0U)
#define DMA_STAT_BLK_Len                                    (8U)
#define DMA_STAT_BLK_Msk                                    (0xFFU << DMA_STAT_BLK_Pos)
#define DMA_STAT_BLK                                        DMA_STAT_BLK_Msk

/*******************  Bit definition for DMA_STAT_SRC_TRN register  ************/
#define DMA_STAT_SRC_TRN_Pos                                (0U)
#define DMA_STAT_SRC_TRN_Len                                (8U)
#define DMA_STAT_SRC_TRN_Msk                                (0xFFU << DMA_STAT_SRC_TRN_Pos)
#define DMA_STAT_SRC_TRN                                    DMA_STAT_SRC_TRN_Msk

/*******************  Bit definition for DMA_STAT_DST_TRN register  ************/
#define DMA_STAT_DST_TRN_Pos                                (0U)
#define DMA_STAT_DST_TRN_Len                                (8U)
#define DMA_STAT_DST_TRN_Msk                                (0xFFU << DMA_STAT_DST_TRN_Pos)
#define DMA_STAT_DST_TRN                                    DMA_STAT_DST_TRN_Msk

/*******************  Bit definition for DMA_STAT_ERR register  ****************/
#define DMA_STAT_ERR_Pos                                    (0U)
#define DMA_STAT_ERR_Len                                    (8U)
#define DMA_STAT_ERR_Msk                                    (0xFFU << DMA_STAT_ERR_Pos)
#define DMA_STAT_ERR                                        DMA_STAT_ERR_Msk

/*******************  Bit definition for DMA_MASK_TFR register  ****************/
#define DMA_MASK_TFR_WE_Pos                                 (8U)
#define DMA_MASK_TFR_WE_Len                                 (8U)
#define DMA_MASK_TFR_WE_Msk                                 (0xFFU << DMA_MASK_TFR_WE_Pos)
#define DMA_MASK_TFR_WE                                     DMA_MASK_TFR_WE_Msk

#define DMA_MASK_TFR_Pos                                    (0U)
#define DMA_MASK_TFR_Len                                    (8U)
#define DMA_MASK_TFR_Msk                                    (0xFFU << DMA_MASK_TFR_Pos)
#define DMA_MASK_TFR                                        DMA_MASK_TFR_Msk

/*******************  Bit definition for DMA_MASK_BLK register  ****************/
#define DMA_MASK_BLK_WE_Pos                                 (8U)
#define DMA_MASK_BLK_WE_Len                                 (8U)
#define DMA_MASK_BLK_WE_Msk                                 (0xFFU << DMA_MASK_BLK_WE_Pos)
#define DMA_MASK_BLK_WE                                     DMA_MASK_BLK_WE_Msk

#define DMA_MASK_BLK_Pos                                    (0U)
#define DMA_MASK_BLK_Len                                    (8U)
#define DMA_MASK_BLK_Msk                                    (0xFFU << DMA_MASK_BLK_Pos)
#define DMA_MASK_BLK                                        DMA_MASK_BLK_Msk

/*******************  Bit definition for DMA_MASK_SRC_TRN register  ************/
#define DMA_MASK_SRC_TRN_WE_Pos                             (8U)
#define DMA_MASK_SRC_TRN_WE_Len                             (8U)
#define DMA_MASK_SRC_TRN_WE_Msk                             (0x1U << DMA_MASK_SRC_TRN_WE_Pos)
#define DMA_MASK_SRC_TRN_WE                                 DMA_MASK_SRC_TRN_WE_Msk

#define DMA_MASK_SRC_TRN_Pos                                (0U)
#define DMA_MASK_SRC_TRN_Len                                (8U)
#define DMA_MASK_SRC_TRN_Msk                                (0xFFU << DMA_MASK_SRC_TRN_Pos)
#define DMA_MASK_SRC_TRN                                    DMA_MASK_SRC_TRN_Msk

/*******************  Bit definition for DMA_MASK_DST_TRN register  ************/
#define DMA_MASK_DST_TRN_WE_Pos                             (8U)
#define DMA_MASK_DST_TRN_WE_Len                             (8U)
#define DMA_MASK_DST_TRN_WE_Msk                             (0xFFU << DMA_MASK_DST_TRN_WE_Pos)
#define DMA_MASK_DST_TRN_WE                                 DMA_MASK_DST_TRN_WE_Msk

#define DMA_MASK_DST_TRN_Pos                                (0U)
#define DMA_MASK_DST_TRN_Len                                (8U)
#define DMA_MASK_DST_TRN_Msk                                (0xFFU << DMA_MASK_DST_TRN_Pos)
#define DMA_MASK_DST_TRN                                    DMA_MASK_DST_TRN_Msk

/*******************  Bit definition for DMA_MASK_ERR register  ****************/
#define DMA_MASK_ERR_WE_Pos                                 (8U)
#define DMA_MASK_ERR_WE_Len                                 (8U)
#define DMA_MASK_ERR_WE_Msk                                 (0xFFU << DMA_MASK_ERR_WE_Pos)
#define DMA_MASK_ERR_WE                                     DMA_MASK_ERR_WE_Msk

#define DMA_MASK_ERR_Pos                                    (0U)
#define DMA_MASK_ERR_Len                                    (8U)
#define DMA_MASK_ERR_Msk                                    (0xFFU << DMA_MASK_ERR_Pos)
#define DMA_MASK_ERR                                        DMA_MASK_ERR_Msk

/*******************  Bit definition for DMA_CLR_TFR register  *****************/
#define DMA_CLR_TFR_Pos                                     (0U)
#define DMA_CLR_TFR_Len                                     (8U)
#define DMA_CLR_TFR_Msk                                     (0xFFU << DMA_CLR_TFR_Pos)
#define DMA_CLR_TFR                                         DMA_CLR_TFR_Msk

/*******************  Bit definition for DMA_CLR_BLK register  *****************/
#define DMA_CLR_BLK_Pos                                     (0U)
#define DMA_CLR_BLK_Len                                     (8U)
#define DMA_CLR_BLK_Msk                                     (0xFFU << DMA_CLR_BLK_Pos)
#define DMA_CLR_BLK                                         DMA_CLR_BLK_Msk

/*******************  Bit definition for DMA_CLR_SRC_TRN register  *************/
#define DMA_CLR_SRC_TRN_Pos                                 (0U)
#define DMA_CLR_SRC_TRN_Len                                 (8U)
#define DMA_CLR_SRC_TRN_Msk                                 (0xFFU << DMA_CLR_SRC_TRN_Pos)
#define DMA_CLR_SRC_TRN                                     DMA_CLR_SRC_TRN_Msk

/*******************  Bit definition for DMA_CLR_DST_TRN register  *************/
#define DMA_CLR_DST_TRN_Pos                                 (0U)
#define DMA_CLR_DST_TRN_Len                                 (8U)
#define DMA_CLR_DST_TRN_Msk                                 (0xFFU << DMA_CLR_DST_TRN_Pos)
#define DMA_CLR_DST_TRN                                     DMA_CLR_DST_TRN_Msk

/*******************  Bit definition for DMA_CLR_ERR register  *****************/
#define DMA_CLR_ERR_Pos                                     (0U)
#define DMA_CLR_ERR_Len                                     (8U)
#define DMA_CLR_ERR_Msk                                     (0xFFU << DMA_CLR_ERR_Pos)
#define DMA_CLR_ERR                                         DMA_CLR_ERR_Msk

/*******************  Bit definition for DMA_STATUS_INT register  **************/
#define DMA_STAT_INT_ERR_Pos                                (4U)
#define DMA_STAT_INT_ERR_Len                                (1U)
#define DMA_STAT_INT_ERR_Msk                                (0x1U << DMA_STAT_INT_ERR_Pos)
#define DMA_STAT_INT_ERR                                    DMA_STAT_INT_ERR_Msk

#define DMA_STAT_INT_DST_Pos                                (3U)
#define DMA_STAT_INT_DST_Len                                (1U)
#define DMA_STAT_INT_DST_Msk                                (0x1U << DMA_STAT_INT_DST_Pos)
#define DMA_STAT_INT_DST                                    DMA_STAT_INT_DST_Msk

#define DMA_STAT_INT_SRC_Pos                                (2U)
#define DMA_STAT_INT_SRC_Len                                (1U)
#define DMA_STAT_INT_SRC_Msk                                (0x1U << DMA_STAT_INT_SRC_Pos)
#define DMA_STAT_INT_SRC                                    DMA_STAT_INT_SRC_Msk

#define DMA_STAT_INT_BLK_Pos                                (1U)
#define DMA_STAT_INT_BLK_Len                                (1U)
#define DMA_STAT_INT_BLK_Msk                                (0x1U << DMA_STAT_INT_BLK_Pos)
#define DMA_STAT_INT_BLK                                    DMA_STAT_INT_BLK_Msk

#define DMA_STAT_INT_TFR_Pos                                (0U)
#define DMA_STAT_INT_TFR_Len                                (1U)
#define DMA_STAT_INT_TFR_Msk                                (0x1U << DMA_STAT_INT_TFR_Pos)
#define DMA_STAT_INT_TFR                                    DMA_STAT_INT_TFR_Msk

/*******************  Bit definition for DMA_REQ_SRC_REG register  *************/
#define DMA_REQ_SRC_WE_Pos                                  (8U)
#define DMA_REQ_SRC_WE_Len                                  (8U)
#define DMA_REQ_SRC_WE_Msk                                  (0xFFU << DMA_REQ_SRC_WE_Pos)
#define DMA_REQ_SRC_WE                                      DMA_REQ_SRC_WE_Msk

#define DMA_REQ_SRC_Pos                                     (0U)
#define DMA_REQ_SRC_Len                                     (8U)
#define DMA_REQ_SRC_Msk                                     (0xFFU << DMA_REQ_SRC_Pos)
#define DMA_REQ_SRC                                         DMA_REQ_SRC_Msk

/*******************  Bit definition for DMA_REQ_DST_REG register  *************/
#define DMA_REQ_DST_WE_Pos                                  (8U)
#define DMA_REQ_DST_WE_Len                                  (8U)
#define DMA_REQ_DST_WE_Msk                                  (0xFFU << DMA_REQ_DST_WE_Pos)
#define DMA_REQ_DST_WE                                      DMA_REQ_DST_WE_Msk

#define DMA_REQ_DST_Pos                                     (0U)
#define DMA_REQ_DST_Len                                     (8U)
#define DMA_REQ_DST_Msk                                     (0xFFU << DMA_REQ_DST_Pos)
#define DMA_REQ_DST                                         DMA_REQ_DST_Msk

/*******************  Bit definition for DMA_SGL_REQ_SRC_REG register  *********/
#define DMA_SGL_REQ_SRC_WE_Pos                              (8U)
#define DMA_SGL_REQ_SRC_WE_Len                              (8U)
#define DMA_SGL_REQ_SRC_WE_Msk                              (0xFFU << DMA_SGL_REQ_SRC_WE_Pos)
#define DMA_SGL_REQ_SRC_WE                                  DMA_SGL_REQ_SRC_WE_Msk

#define DMA_SGL_REQ_SRC_Pos                                 (0U)
#define DMA_SGL_REQ_SRC_Len                                 (8U)
#define DMA_SGL_REQ_SRC_Msk                                 (0xFFU << DMA_SGL_REQ_SRC_Pos)
#define DMA_SGL_REQ_SRC                                     DMA_SGL_REQ_SRC_Msk

/*******************  Bit definition for DMA_SGL_REQ_DST_REG register  *********/
#define DMA_SGL_REQ_DST_WE_Pos                              (8U)
#define DMA_SGL_REQ_DST_WE_Len                              (8U)
#define DMA_SGL_REQ_DST_WE_Msk                              (0xFFU << DMA_SGL_REQ_DST_WE_Pos)
#define DMA_SGL_REQ_DST_WE                                  DMA_SGL_REQ_DST_WE_Msk

#define DMA_SGL_REQ_DST_Pos                                 (0U)
#define DMA_SGL_REQ_DST_Len                                 (8U)
#define DMA_SGL_REQ_DST_Msk                                 (0xFFU << DMA_SGL_REQ_DST_Pos)
#define DMA_SGL_REQ_DST                                     DMA_SGL_REQ_DST_Msk

/*******************  Bit definition for DMA_LST_SRC_REG register  *********/
#define DMA_LST_SRC_WE_Pos                                  (8U)
#define DMA_LST_SRC_WE_Len                                  (8U)
#define DMA_LST_SRC_WE_Msk                                  (0xFFU << DMA_LST_SRC_WE_Pos)
#define DMA_LST_SRC_WE                                      DMA_LST_SRC_WE_Msk

#define DMA_LST_SRC_Pos                                     (0U)
#define DMA_LST_SRC_Len                                     (8U)
#define DMA_LST_SRC_Msk                                     (0xFFU << DMA_LST_SRC_Pos)
#define DMA_LST_SRC                                         DMA_LST_SRC_Msk

/*******************  Bit definition for DMA_LST_DST_REG register  *********/
#define DMA_LST_DST_WE_Pos                                  (8U)
#define DMA_LST_DST_WE_Len                                  (8U)
#define DMA_LST_DST_WE_Msk                                  (0xFFU << DMA_LST_DST_WE_Pos)
#define DMA_LST_DST_WE                                      DMA_LST_DST_WE_Msk

#define DMA_LST_DST_Pos                                     (0U)
#define DMA_LST_DST_Len                                     (8U)
#define DMA_LST_DST_Msk                                     (0xFFU << DMA_LST_DST_Pos)
#define DMA_LST_DST                                         DMA_LST_DST_Msk

/*******************  Bit definition for DMA_CFG_REG register  ****************/
#define DMA_MODULE_CFG_EN_Pos                               (0U)
#define DMA_MODULE_CFG_EN_Len                               (1U)
#define DMA_MODULE_CFG_EN_Msk                               (0x1U << DMA_MODULE_CFG_EN_Pos)
#define DMA_MODULE_CFG_EN                                   DMA_MODULE_CFG_EN_Msk

/*******************  Bit definition for DMA_CH_EN_REG register  **************/
#define DMA_CH_WE_EN_Pos                                    (8U)
#define DMA_CH_WE_EN_Len                                    (8U)
#define DMA_CH_WE_EN_Msk                                    (0xFFU << DMA_CH_WE_EN_Pos)
#define DMA_CH_WE_EN                                        DMA_CH_WE_EN_Msk

#define DMA_CH_EN_Pos                                       (0U)
#define DMA_CH_EN_Len                                       (8U)
#define DMA_CH_EN_Msk                                       (0xFFU << DMA_CH_EN_Pos)
#define DMA_CH_EN                                           DMA_CH_EN_Msk

/* ================================================================================================================= */
/* ================                                     DUAL_TIMER                                    ================ */
/* ================================================================================================================= */

/*******************  Bit definition for DUAL_TIMER_RELOAD register  ************/
#define DUAL_TIMER_RELOAD_RELOAD_Pos                          (0U)
#define DUAL_TIMER_RELOAD_RELOAD_Len                          (32U)
#define DUAL_TIMER_RELOAD_RELOAD_Msk                          (0xFFFFFFFFU)
#define DUAL_TIMER_RELOAD_RELOAD                              DUAL_TIMER_RELOAD_RELOAD_Msk

/*******************  Bit definition for DUAL_TIMER_VALUE register  *************/
#define DUAL_TIMER_VALUE_VALUE_Pos                            (0U)
#define DUAL_TIMER_VALUE_VALUE_Len                            (32U)
#define DUAL_TIMER_VALUE_VALUE_Msk                            (0xFFFFFFFFU)
#define DUAL_TIMER_VALUE_VALUE                                DUAL_TIMER_VALUE_VALUE_Msk

/*******************  Bit definition for DUAL_TIMER_CTRL register  **************/
#define DUAL_TIMER_CTRL_EN_Pos                                (7U)
#define DUAL_TIMER_CTRL_EN_Len                                (1U)
#define DUAL_TIMER_CTRL_EN_Msk                                (0x1U << DUAL_TIMER_CTRL_EN_Pos)
#define DUAL_TIMER_CTRL_EN                                    DUAL_TIMER_CTRL_EN_Msk

#define DUAL_TIMER_CTRL_MODE_Pos                              (6U)
#define DUAL_TIMER_CTRL_MODE_Len                              (1U)
#define DUAL_TIMER_CTRL_MODE_Msk                              (0x1U << DUAL_TIMER_CTRL_MODE_Pos)
#define DUAL_TIMER_CTRL_MODE                                  DUAL_TIMER_CTRL_MODE_Msk

#define DUAL_TIMER_CTRL_INTEN_Pos                             (5U)
#define DUAL_TIMER_CTRL_INTEN_Len                             (1U)
#define DUAL_TIMER_CTRL_INTEN_Msk                             (0x1U << DUAL_TIMER_CTRL_INTEN_Pos)
#define DUAL_TIMER_CTRL_INTEN                                 DUAL_TIMER_CTRL_INTEN_Msk

#define DUAL_TIMER_CTRL_PRE_Pos                               (2U)
#define DUAL_TIMER_CTRL_PRE_Len                               (2U)
#define DUAL_TIMER_CTRL_PRE_Msk                               (0x3U << DUAL_TIMER_CTRL_PRE_Pos)
#define DUAL_TIMER_CTRL_PRE                                   DUAL_TIMER_CTRL_PRE_Msk

#define DUAL_TIMER_CTRL_SIZE_Pos                              (1U)
#define DUAL_TIMER_CTRL_SIZE_Len                              (1U)
#define DUAL_TIMER_CTRL_SIZE_Msk                              (0x1U << DUAL_TIMER_CTRL_SIZE_Pos)
#define DUAL_TIMER_CTRL_SIZE                                  DUAL_TIMER_CTRL_SIZE_Msk

#define DUAL_TIMER_CTRL_ONESHOT_Pos                           (0U)
#define DUAL_TIMER_CTRL_ONESHOT_Len                           (1U)
#define DUAL_TIMER_CTRL_ONESHOT_Msk                           (0x1U << DUAL_TIMER_CTRL_ONESHOT_Pos)
#define DUAL_TIMER_CTRL_ONESHOT                               DUAL_TIMER_CTRL_ONESHOT_Msk

/*******************  Bit definition for DUAL_TIMER_INT_CLR register  ***********/
#define DUAL_TIMER_INT_CLR_Pos                                (0U)
#define DUAL_TIMER_INT_CLR_Len                                (32U)
#define DUAL_TIMER_INT_CLR_Msk                                (0xFFFFFFFFU)
#define DUAL_TIMER_INT_CLR                                    DUAL_TIMER_INT_CLR_Msk

/*******************  Bit definition for DUAL_TIMER_RAW_INT_STAT register  ******/
#define DUAL_TIMER_RIS_RTI_Pos                                (0U)
#define DUAL_TIMER_RIS_RTI_Len                                (1U)
#define DUAL_TIMER_RIS_RTI_Msk                                (0x1U << DUAL_TIMER_RIS_RTI_Pos)
#define DUAL_TIMER_RIS_RTI                                    DUAL_TIMER_RIS_RTI_Msk

/*******************  Bit definition for DUAL_TIMER_INT_STAT register  **********/
#define DUAL_TIMER_ISR_TI_Pos                                 (0U)
#define DUAL_TIMER_ISR_TI_Len                                 (1U)
#define DUAL_TIMER_ISR_TI_Msk                                 (0x1U << DUAL_TIMER_ISR_TI_Pos)
#define DUAL_TIMER_ISR_TI                                     DUAL_TIMER_ISR_TI_Msk

/*******************  Bit definition for DUAL_TIMER_BGLOAD register  ************/
#define DUAL_TIMER_BLR_BL_Pos                                 (0U)
#define DUAL_TIMER_BLR_BL_Len                                 (32U)
#define DUAL_TIMER_BLR_BL_Msk                                 (0xFFFFFFFFU)
#define DUAL_TIMER_BLR_BL                                     DUAL_TIMER_BLR_BL_Msk


/* ================================================================================================================= */
/* ================                                        GPIO                                     ================ */
/* ================================================================================================================= */

/*******************  Bit definition for GPIO_DATA register  ******************/
#define GPIO_DATA_Pos                                       (0U)
#define GPIO_DATA_Len                                       (16U)
#define GPIO_DATA_Msk                                       (0xFFFFU << GPIO_DATA_Pos)
#define GPIO_DATA                                           GPIO_DATA_Msk          /**< Data */

/*******************  Bit definition for GPIO_DATAOUT register  ***************/
#define GPIO_DATAOUT_Pos                                    (0U)
#define GPIO_DATAOUT_Len                                    (16U)
#define GPIO_DATAOUT_Msk                                    (0xFFFFU << GPIO_DATAOUT_Pos)
#define GPIO_DATAOUT                                        GPIO_DATAOUT_Msk    /**< Data Output */

/*******************  Bit definition for GPIO_OUTENSET register  ***************/
#define GPIO_OUTENSET_Pos                                   (0U)
#define GPIO_OUTENSET_Len                                   (16U)
#define GPIO_OUTENSET_Msk                                   (0xFFFFU << GPIO_OUTENSET_Pos)
#define GPIO_OUTENSET                                       GPIO_OUTENSET_Msk    /**< Data Output Enable Set*/

/*******************  Bit definition for GPIO_OUTENCLR register  ***************/
#define GPIO_OUTENCLR_Pos                                   (0U)
#define GPIO_OUTENCLR_Len                                   (16U)
#define GPIO_OUTENCLR_Msk                                   (0xFFFFU << GPIO_OUTENCLR_Pos)
#define GPIO_OUTENCLR                                       GPIO_OUTENCLR_Msk    /**< Data Output Enable Clear */

/*******************  Bit definition for GPIO_INTENSET register  ***************/
#define GPIO_INTENSET_Pos                                   (0U)
#define GPIO_INTENSET_Len                                   (16U)
#define GPIO_INTENSET_Msk                                   (0xFFFFU << GPIO_INTENSET_Pos)
#define GPIO_INTENSET                                       GPIO_INTENSET_Msk    /**< Interrupt Enable Set */

/*******************  Bit definition for GPIO_INTENCLR register  ***************/
#define GPIO_INTENCLR_Pos                                   (0U)
#define GPIO_INTENCLR_Len                                   (16U)
#define GPIO_INTENCLR_Msk                                   (0xFFFFU << GPIO_INTENCLR_Pos)
#define GPIO_INTENCLR                                       GPIO_INTENCLR_Msk    /**< Interrupt Enable clear */

/*******************  Bit definition for GPIO_INTTYPESET register  ***************/
#define GPIO_INTTYPESET_Pos                                 (0U)
#define GPIO_INTTYPESET_Len                                 (16U)
#define GPIO_INTTYPESET_Msk                                 (0xFFFFU << GPIO_INTTYPESET_Pos)
#define GPIO_INTTYPESET                                     GPIO_INTTYPESET_Msk    /**< Interrupt Type Set */

/*******************  Bit definition for GPIO_INTTYPECLR register  ***************/
#define GPIO_INTTYPECLR_Pos                                 (0U)
#define GPIO_INTTYPECLR_Len                                 (16U)
#define GPIO_INTTYPECLR_Msk                                 (0xFFFFU << GPIO_INTTYPECLR_Pos)
#define GPIO_INTTYPECLR                                     GPIO_INTTYPECLR_Msk    /**< Interrupt Type Clear */

/*******************  Bit definition for GPIO_INTPOLSET register  ***************/
#define GPIO_INTPOLSET_Pos                                  (0U)
#define GPIO_INTPOLSET_Len                                  (16U)
#define GPIO_INTPOLSET_Msk                                  (0xFFFFU << GPIO_INTPOLSET_Pos)
#define GPIO_INTPOLSET                                      GPIO_INTPOLSET_Msk    /**< Interrupt Polarity-level Set */

/*******************  Bit definition for GPIO_INTPOLCLR register  ***************/
#define GPIO_INTPOLCLR_Pos                                  (0U)
#define GPIO_INTPOLCLR_Len                                  (16U)
#define GPIO_INTPOLCLR_Msk                                  (0xFFFFU << GPIO_INTPOLCLR_Pos)
#define GPIO_INTPOLCLR                                      GPIO_INTPOLCLR_Msk    /**< Interrupt Polarity-level Clear */

/*******************  Bit definition for GPIO_INTSTAT register  ***************/
#define GPIO_INTSTAT_Pos                                    (0U)
#define GPIO_INTSTAT_Len                                    (16U)
#define GPIO_INTSTAT_Msk                                    (0xFFFFU << GPIO_INTSTAT_Pos)
#define GPIO_INTSTAT                                        GPIO_INTSTAT_Msk    /**< Interrupt Status */

/*******************  Bit definition for GPIO_MASKLOWBYTE register  ***********/
#define GPIO_MASKLOWBYTE_DATA_Pos                           (0U)
#define GPIO_MASKLOWBYTE_DATA_Len                           (8U)
#define GPIO_MASKLOWBYTE_DATA_Msk                           (0xFFU << GPIO_MASKLOWBYTE_DATA_Pos)
#define GPIO_MASKLOWBYTE_DATA                               GPIO_MASKLOWBYTE_DATA_Msk   /**< Lower eight bits masked access */

/*******************  Bit definition for GPIO_MASKLOWBYTE register  ***********/
#define GPIO_MASKHIGHBYTE_DATA_Pos                          (8U)
#define GPIO_MASKHIGHBYTE_DATA_Len                          (8U)
#define GPIO_MASKHIGHBYTE_DATA_Msk                          (0xFFU << GPIO_MASKHIGHBYTE_DATA_Pos)
#define GPIO_MASKHIGHBYTE_DATA                              GPIO_MASKHIGHBYTE_DATA   /**< Higher eight bits masked access */


/* ================================================================================================================= */
/* ================                                       HMAC                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for HMAC_CTRL register  *******************/
#define HMAC_CTRL_EN_POS                                    (0U)
#define HMAC_CTRL_EN_Len                                    (1U)
#define HMAC_CTRL_EN_Msk                                    (0x1UL << HMAC_CTRL_EN_POS)
#define HMAC_CTRL_EN                                        HMAC_CTRL_EN_Msk

#define HMAC_CTRL_DMA_START_POS                             (1U)
#define HMAC_CTRL_DMA_START_Len                             (1U)
#define HMAC_CTRL_DMA_START_Msk                             (0x1UL << HMAC_CTRL_DMA_START_POS)
#define HMAC_CTRL_DMA_START                                 HMAC_CTRL_DMA_START_Msk

#define HMAC_CTRL_KEY_EN_POS                                (2U)
#define HMAC_CTRL_KEY_EN_Len                                (1U)
#define HMAC_CTRL_KEY_EN_Msk                                (0x1UL << HMAC_CTRL_KEY_EN_POS)
#define HMAC_CTRL_KEY_EN                                    HMAC_CTRL_KEY_EN_Msk

#define HMAC_CTRL_LST_TX_POS                                (3U)
#define HMAC_CTRL_LST_TX_Len                                (1U)
#define HMAC_CTRL_LST_TX_Msk                                (0x1UL << HMAC_CTRL_LST_TX_POS)
#define HMAC_CTRL_LST_TX                                    HMAC_CTRL_LST_TX_Msk

/*******************  Bit definition for HMAC_CFG register  *******************/
#define HMAC_CFG_HASH_POS                                   (0U)
#define HMAC_CFG_HASH_Len                                   (1U)
#define HMAC_CFG_HASH_Msk                                   (0x1UL << HMAC_CFG_HASH_POS)
#define HMAC_CFG_HASH                                       HMAC_CFG_HASH_Msk

#define HMAC_CFG_ENDIAN_POS                                 (1U)
#define HMAC_CFG_ENDIAN_Len                                 (1U)
#define HMAC_CFG_ENDIAN_Msk                                 (0x1UL << HMAC_CFG_ENDIAN_POS)
#define HMAC_CFG_ENDIAN                                     HMAC_CFG_ENDIAN_Msk

#define HMAC_CFG_KEY_TYPE_POS                               (2U)
#define HMAC_CFG_KEY_TYPE_Len                               (2U)
#define HMAC_CFG_KEY_TYPE_Msk                               (0x3UL << HMAC_CFG_KEY_TYPE_POS)
#define HMAC_CFG_KEY_TYPE                                   HMAC_CFG_KEY_TYPE_Msk

#define HMAC_CFG_CALC_TYPE_POS                              (4U)
#define HMAC_CFG_CALC_TYPE_Len                              (1U)
#define HMAC_CFG_CALC_TYPE_Msk                              (0x1UL << HMAC_CFG_CALC_TYPE_POS)
#define HMAC_CFG_CALC_TYPE                                  HMAC_CFG_CALC_TYPE_Msk

#define HMAC_CFG_PRIVT_MODE_POS                             (5U)
#define HMAC_CFG_PRIVT_MODE_Len                             (1U)
#define HMAC_CFG_PRIVT_MODE_Msk                             (0x1UL << HMAC_CFG_PRIVT_MODE_POS)
#define HMAC_CFG_PRIVT_MODE                                 HMAC_CFG_PRIVT_MODE_Msk

/*******************  Bit definition for HMAC_STAT register  *******************/
#define HMAC_STAT_HASH_READY_POS                            (0U)
#define HMAC_STAT_HASH_READY_Len                            (1U)
#define HMAC_STAT_HASH_READY_Msk                            (0x1UL << HMAC_STAT_HASH_READY_POS)
#define HMAC_STAT_HASH_READY                                HMAC_STAT_HASH_READY_Msk

#define HMAC_STAT_DMA_MSG_DONE_POS                          (1U)
#define HMAC_STAT_DMA_MSG_DONE_Len                          (1U)
#define HMAC_STAT_DMA_MSG_DONE_Msk                          (0x1UL << HMAC_STAT_DMA_MSG_DONE_POS)
#define HMAC_STAT_DMA_MSG_DONE                              HMAC_STAT_DMA_MSG_DONE_Msk

#define HMAC_STAT_DMA_TX_ERR_POS                            (2U)
#define HMAC_STAT_DMA_TX_ERR_Len                            (1U)
#define HMAC_STAT_DMA_TX_ERR_Msk                            (0x1UL << HMAC_STAT_DMA_TX_ERR_POS)
#define HMAC_STAT_DMA_TX_ERR                                HMAC_STAT_DMA_TX_ERR_Msk

#define HMAC_STAT_KEY_VALID_POS                             (3U)
#define HMAC_STAT_KEY_VALID_Len                             (1U)
#define HMAC_STAT_KEY_VALID_Msk                             (0x1UL << HMAC_STAT_KEY_VALID_POS)
#define HMAC_STAT_KEY_VALID                                 HMAC_STAT_KEY_VALID_Msk

#define HMAC_STAT_HMAC_READY_POS                            (4U)
#define HMAC_STAT_HMAC_READY_Len                            (1U)
#define HMAC_STAT_HMAC_READY_Msk                            (0x1UL << HMAC_STAT_HMAC_READY_POS)
#define HMAC_STAT_HMAC_READY                                HMAC_STAT_HMAC_READY_Msk

#define HMAC_STAT_DMA_TX_DONE_POS                           (5U)
#define HMAC_STAT_DMA_TX_DONE_Len                           (1U)
#define HMAC_STAT_DMA_TX_DONE_Msk                           (0x1UL << HMAC_STAT_DMA_TX_DONE_POS)
#define HMAC_STAT_DMA_TX_DONE                               HMAC_STAT_DMA_TX_DONE_Msk

/*******************  Bit definition for HMAC_XFE_SIZE register  *******************/
#define HMAC_XFE_SIZE_SIZE_POS                              (0U)
#define HMAC_XFE_SIZE_SIZE_Len                              (15U)
#define HMAC_XFE_SIZE_SIZE_Msk                              (0x7FFFUL << HMAC_XFE_SIZE_SIZE_POS)
#define HMAC_XFE_SIZE_SIZE                                  HMAC_XFE_SIZE_SIZE_Msk

/*******************  Bit definition for HMAC_INT register  *******************/
#define HMAC_INT_DONE_POS                                   (0U)
#define HMAC_INT_DONE_Len                                   (1U)
#define HMAC_INT_DONE_Msk                                   (0x1UL << HMAC_INT_DONE_POS)
#define HMAC_INT_DONE                                       HMAC_INT_DONE_Msk

#define HMAC_INT_EN_POS                                     (1U)
#define HMAC_INT_EN_Len                                     (1U)
#define HMAC_INT_EN_Msk                                     (0x1UL << HMAC_INT_EN_POS)
#define HMAC_INT_EN                                         HMAC_INT_EN_Msk

/*******************  Bit definition for HMAC_RD_START_ADDR register  *******************/
#define HMAC_RD_START_ADDR_ADDR_POS                         (0U)
#define HMAC_RD_START_ADDR_ADDR_Len                         (32U)
#define HMAC_RD_START_ADDR_ADDR_Msk                         (0xFFFFFFFFUL << HMAC_RD_START_ADDR_ADDR_POS)
#define HMAC_RD_START_ADDR_ADDR                             HMAC_RD_START_ADDR_ADDR_Msk

/*******************  Bit definition for HMAC_WR_START_ADDR register  *******************/
#define HMAC_WR_START_ADDR_ADDR_POS                         (0U)
#define HMAC_WR_START_ADDR_ADDR_Len                         (32U)
#define HMAC_WR_START_ADDR_ADDR_Msk                         (0xFFFFFFFFUL << HMAC_WR_START_ADDR_ADDR_POS)
#define HMAC_WR_START_ADDR_ADDR                             HMAC_WR_START_ADDR_ADDR_Msk

/*******************  Bit definition for HMAC_USER_HASH_0 register  *******************/
#define HMAC_USER_HASH_0_HASH_0_POS                         (0U)
#define HMAC_USER_HASH_0_HASH_0_Len                         (32U)
#define HMAC_USER_HASH_0_HASH_0_Msk                         (0xFFFFFFFFUL << HMAC_USER_HASH_0_HASH_0_POS)
#define HMAC_USER_HASH_0_HASH_0                             HMAC_USER_HASH_0_HASH_0_Msk

/*******************  Bit definition for HMAC_USER_HASH_1 register  *******************/
#define HMAC_USER_HASH_1_HASH_1_POS                         (0U)
#define HMAC_USER_HASH_1_HASH_1_Len                         (32U)
#define HMAC_USER_HASH_1_HASH_1_Msk                         (0xFFFFFFFFUL << HMAC_USER_HASH_1_HASH_1_POS)
#define HMAC_USER_HASH_1_HASH_1                             HMAC_USER_HASH_1_HASH_1_Msk

/*******************  Bit definition for HMAC_USER_HASH_2 register  *******************/
#define HMAC_USER_HASH_2_HASH_2_POS                         (0U)
#define HMAC_USER_HASH_2_HASH_2_Len                         (32U)
#define HMAC_USER_HASH_2_HASH_2_Msk                         (0xFFFFFFFFUL << HMAC_USER_HASH_2_HASH_2_POS)
#define HMAC_USER_HASH_2_HASH_2                             HMAC_USER_HASH_2_HASH_2_Msk

/*******************  Bit definition for HMAC_USER_HASH_3 register  *******************/
#define HMAC_USER_HASH_3_HASH_3_POS                         (0U)
#define HMAC_USER_HASH_3_HASH_3_Len                         (32U)
#define HMAC_USER_HASH_3_HASH_3_Msk                         (0xFFFFFFFFUL << HMAC_USER_HASH_3_HASH_3_POS)
#define HMAC_USER_HASH_3_HASH_3                             HMAC_USER_HASH_3_HASH_3_Msk

/*******************  Bit definition for HMAC_USER_HASH_4 register  *******************/
#define HMAC_USER_HASH_4_HASH_4_POS                         (0U)
#define HMAC_USER_HASH_4_HASH_4_Len                         (32U)
#define HMAC_USER_HASH_4_HASH_4_Msk                         (0xFFFFFFFFUL << HMAC_USER_HASH_4_HASH_4_POS)
#define HMAC_USER_HASH_4_HASH_4                             HMAC_USER_HASH_4_HASH_4_Msk

/*******************  Bit definition for HMAC_USER_HASH_5 register  *******************/
#define HMAC_USER_HASH_5_HASH_5_POS                         (0U)
#define HMAC_USER_HASH_5_HASH_5_Len                         (32U)
#define HMAC_USER_HASH_5_HASH_5_Msk                         (0xFFFFFFFFUL << HMAC_USER_HASH_5_HASH_5_POS)
#define HMAC_USER_HASH_5_HASH_5                             HMAC_USER_HASH_5_HASH_5_Msk

/*******************  Bit definition for HMAC_USER_HASH_6 register  *******************/
#define HMAC_USER_HASH_6_HASH_6_POS                         (0U)
#define HMAC_USER_HASH_6_HASH_6_Len                         (32U)
#define HMAC_USER_HASH_6_HASH_6_Msk                         (0xFFFFFFFFUL << HMAC_USER_HASH_6_HASH_6_POS)
#define HMAC_USER_HASH_6_HASH_6                             HMAC_USER_HASH_6_HASH_6_Msk

/*******************  Bit definition for HMAC_USER_HASH_7 register  *******************/
#define HMAC_USER_HASH_7_HASH_7_POS                         (0U)
#define HMAC_USER_HASH_7_HASH_7_Len                         (32U)
#define HMAC_USER_HASH_7_HASH_7_Msk                         (0xFFFFFFFFUL << HMAC_USER_HASH_7_HASH_7_POS)
#define HMAC_USER_HASH_7_HASH_7                             HMAC_USER_HASH_7_HASH_7_Msk

/*******************  Bit definition for HMAC_DATA_OUT register  *******************/
#define HMAC_DATA_OUT_DATA_POS                              (0U)
#define HMAC_DATA_OUT_DATA_Len                              (32U)
#define HMAC_DATA_OUT_DATA_Msk                              (0xFFFFFFFFUL << HMAC_DATA_OUT_DATA_POS)
#define HMAC_DATA_OUT_DATA                                  HMAC_DATA_OUT_DATA_Msk

/*******************  Bit definition for HMAC_DATA_IN register  *******************/
#define HMAC_DATA_IN_DATA_POS                               (0U)
#define HMAC_DATA_IN_DATA_Len                               (32U)
#define HMAC_DATA_IN_DATA_Msk                               (0xFFFFFFFFUL << HMAC_DATA_IN_DATA_POS)
#define HMAC_DATA_IN_DATA                                   HMAC_DATA_IN_DATA_Msk

/*******************  Bit definition for HMAC_KEY0 register  *******************/
#define HMAC_KEY0_KEY0_POS                                  (0U)
#define HMAC_KEY0_KEY0_Len                                  (32U)
#define HMAC_KEY0_KEY0_Msk                                  (0xFFFFFFFFUL << HMAC_KEY0_KEY0_POS)
#define HMAC_KEY0_KEY0                                      HMAC_KEY0_KEY0_Msk

/*******************  Bit definition for HMAC_KEY1 register  *******************/
#define HMAC_KEY1_KEY1_POS                                  (0U)
#define HMAC_KEY1_KEY1_Len                                  (32U)
#define HMAC_KEY1_KEY1_Msk                                  (0xFFFFFFFFUL << HMAC_KEY1_KEY1_POS)
#define HMAC_KEY1_KEY1                                      HMAC_KEY1_KEY1_Msk

/*******************  Bit definition for HMAC_KEY2 register  *******************/
#define HMAC_KEY2_KEY2_POS                                  (0U)
#define HMAC_KEY2_KEY2_Len                                  (32U)
#define HMAC_KEY2_KEY2_Msk                                  (0xFFFFFFFFUL << HMAC_KEY2_KEY2_POS)
#define HMAC_KEY2_KEY2                                      HMAC_KEY2_KEY2_Msk

/*******************  Bit definition for HMAC_KEY3 register  *******************/
#define HMAC_KEY3_KEY3_POS                                  (0U)
#define HMAC_KEY3_KEY3_Len                                  (32U)
#define HMAC_KEY3_KEY3_Msk                                  (0xFFFFFFFFUL << HMAC_KEY3_KEY3_POS)
#define HMAC_KEY3_KEY3                                      HMAC_KEY3_KEY3_Msk

/*******************  Bit definition for HMAC_KEY4 register  *******************/
#define HMAC_KEY4_KEY4_POS                                  (0U)
#define HMAC_KEY4_KEY4_Len                                  (32U)
#define HMAC_KEY4_KEY4_Msk                                  (0xFFFFFFFFUL << HMAC_KEY4_KEY4_POS)
#define HMAC_KEY4_KEY4                                      HMAC_KEY4_KEY4_Msk

/*******************  Bit definition for HMAC_KEY5 register  *******************/
#define HMAC_KEY5_KEY5_POS                                  (0U)
#define HMAC_KEY5_KEY5_Len                                  (32U)
#define HMAC_KEY5_KEY5_Msk                                  (0xFFFFFFFFUL << HMAC_KEY5_KEY5_POS)
#define HMAC_KEY5_KEY5                                      HMAC_KEY5_KEY5_Msk

/*******************  Bit definition for HMAC_KEY6 register  *******************/
#define HMAC_KEY6_KEY6_POS                                  (0U)
#define HMAC_KEY6_KEY6_Len                                  (32U)
#define HMAC_KEY6_KEY6_Msk                                  (0xFFFFFFFFUL << HMAC_KEY6_KEY6_POS)
#define HMAC_KEY6_KEY6                                      HMAC_KEY6_KEY6_Msk

/*******************  Bit definition for HMAC_KEY7 register  *******************/
#define HMAC_KEY7_KEY7_POS                                  (0U)
#define HMAC_KEY7_KEY7_Len                                  (32U)
#define HMAC_KEY7_KEY7_Msk                                  (0xFFFFFFFFUL << HMAC_KEY7_KEY7_POS)
#define HMAC_KEY7_KEY7                                      HMAC_KEY7_KEY7_Msk

/*******************  Bit definition for HMAC_KEY_ADDR register  *******************/
#define HMAC_KEY_ADDR_KEY_ADDR_POS                          (0U)
#define HMAC_KEY_ADDR_KEY_ADDR_Len                          (32U)
#define HMAC_KEY_ADDR_KEY_ADDR_Msk                          (0xFFFFFFFFUL << HMAC_KEY_ADDR_KEY_ADDR_POS)
#define HMAC_KEY_ADDR_KEY_ADDR                              HMAC_KEY_ADDR_KEY_ADDR_Msk

/*******************  Bit definition for HMAC_KEYPORT_MASK register  *******************/
#define HMAC_KEYPORT_MASK_MASK_POS                          (0U)
#define HMAC_KEYPORT_MASK_MASK_Len                          (32U)
#define HMAC_KEYPORT_MASK_MASK_Msk                          (0xFFFFFFFFUL << HMAC_KEYPORT_MASK_MASK_POS)
#define HMAC_KEYPORT_MASK_MASK                              HMAC_KEYPORT_MASK_MASK_Msk

/* ================================================================================================================= */
/* ================                                        I2C                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for I2C_CTRL register  *******************/
#define I2C_CTRL_M_MODE_POS                                 (0U)
#define I2C_CTRL_M_MODE_Len                                 (1U)
#define I2C_CTRL_M_MODE_Msk                                 (0x1UL << I2C_CTRL_M_MODE_POS)
#define I2C_CTRL_M_MODE                                     I2C_CTRL_M_MODE_Msk

#define I2C_CTRL_SPEED_POS                                  (1U)
#define I2C_CTRL_SPEED_Len                                  (2U)
#define I2C_CTRL_SPEED_Msk                                  (0x3UL << I2C_CTRL_SPEED_POS)
#define I2C_CTRL_SPEED                                      I2C_CTRL_SPEED_Msk

#define I2C_CTRL_ADDR_BIT_S_POS                             (3U)
#define I2C_CTRL_ADDR_BIT_S_Len                             (1U)
#define I2C_CTRL_ADDR_BIT_S_Msk                             (0x1UL << I2C_CTRL_ADDR_BIT_S_POS)
#define I2C_CTRL_ADDR_BIT_S                                 I2C_CTRL_ADDR_BIT_S_Msk

#define I2C_CTRL_ADDR_BIT_M_POS                             (4U)
#define I2C_CTRL_ADDR_BIT_M_Len                             (1U)
#define I2C_CTRL_ADDR_BIT_M_Msk                             (0x1UL << I2C_CTRL_ADDR_BIT_M_POS)
#define I2C_CTRL_ADDR_BIT_M                                 I2C_CTRL_ADDR_BIT_M_Msk

#define I2C_CTRL_RESTART_EN_POS                             (5U)
#define I2C_CTRL_RESTART_EN_Len                             (1U)
#define I2C_CTRL_RESTART_EN_Msk                             (0x1UL << I2C_CTRL_RESTART_EN_POS)
#define I2C_CTRL_RESTART_EN                                 I2C_CTRL_RESTART_EN_Msk

#define I2C_CTRL_S_DIS_POS                                  (6U)
#define I2C_CTRL_S_DIS_Len                                  (1U)
#define I2C_CTRL_S_DIS_Msk                                  (0x1UL << I2C_CTRL_S_DIS_POS)
#define I2C_CTRL_S_DIS                                      I2C_CTRL_S_DIS_Msk

#define I2C_CTRL_STOP_DET_INT_POS                           (7U)
#define I2C_CTRL_STOP_DET_INT_Len                           (1U)
#define I2C_CTRL_STOP_DET_INT_Msk                           (0x1UL << I2C_CTRL_STOP_DET_INT_POS)
#define I2C_CTRL_STOP_DET_INT                               I2C_CTRL_STOP_DET_INT_Msk

#define I2C_CTRL_TX_EMPTY_CTRL_POS                          (8U)
#define I2C_CTRL_TX_EMPTY_CTRL_Len                          (1U)
#define I2C_CTRL_TX_EMPTY_CTRL_Msk                          (0x1UL << I2C_CTRL_TX_EMPTY_CTRL_POS)
#define I2C_CTRL_TX_EMPTY_CTRL                              I2C_CTRL_TX_EMPTY_CTRL_Msk

#define I2C_CTRL_RXFIFO_FULL_HLD_POS                        (9U)
#define I2C_CTRL_RXFIFO_FULL_HLD_Len                        (1U)
#define I2C_CTRL_RXFIFO_FULL_HLD_Msk                        (0x1UL << I2C_CTRL_RXFIFO_FULL_HLD_POS)
#define I2C_CTRL_RXFIFO_FULL_HLD                            I2C_CTRL_RXFIFO_FULL_HLD_Msk

#define I2C_CTRL_STOP_DET_M_ACTIVE_POS                      (10U)
#define I2C_CTRL_STOP_DET_M_ACTIVE_Len                      (1U)
#define I2C_CTRL_STOP_DET_M_ACTIVE_Msk                      (0x1UL << I2C_CTRL_STOP_DET_M_ACTIVE_POS)
#define I2C_CTRL_STOP_DET_M_ACTIVE                          I2C_CTRL_STOP_DET_M_ACTIVE_Msk

#define I2C_CTRL_BUS_CLR_FEATURE_POS                        (11U)
#define I2C_CTRL_BUS_CLR_FEATURE_Len                        (1U)
#define I2C_CTRL_BUS_CLR_FEATURE_Msk                        (0x1UL << I2C_CTRL_BUS_CLR_FEATURE_POS)
#define I2C_CTRL_BUS_CLR_FEATURE                            I2C_CTRL_BUS_CLR_FEATURE_Msk

/*******************  Bit definition for I2C_TARGET_ADDR register  *******************/
#define I2C_TARGET_ADDR_TARGET_POS                          (0U)
#define I2C_TARGET_ADDR_TARGET_Len                          (10U)
#define I2C_TARGET_ADDR_TARGET_Msk                          (0x3FFUL << I2C_TARGET_ADDR_TARGET_POS)
#define I2C_TARGET_ADDR_TARGET                              I2C_TARGET_ADDR_TARGET_Msk

#define I2C_TARGET_ADDR_TX_CTRL_POS                         (10U)
#define I2C_TARGET_ADDR_TX_CTRL_Len                         (1U)
#define I2C_TARGET_ADDR_TX_CTRL_Msk                         (0x1UL << I2C_TARGET_ADDR_TX_CTRL_POS)
#define I2C_TARGET_ADDR_TX_CTRL                             I2C_TARGET_ADDR_TX_CTRL_Msk

#define I2C_TARGET_ADDR_SPECIAL_POS                         (11U)
#define I2C_TARGET_ADDR_SPECIAL_Len                         (1U)
#define I2C_TARGET_ADDR_SPECIAL_Msk                         (0x1UL << I2C_TARGET_ADDR_SPECIAL_POS)
#define I2C_TARGET_ADDR_SPECIAL                             I2C_TARGET_ADDR_SPECIAL_Msk

/*******************  Bit definition for I2C_S_ADDR register  *******************/
#define I2C_S_ADDR_S_ADDR_POS                               (0U)
#define I2C_S_ADDR_S_ADDR_Len                               (10U)
#define I2C_S_ADDR_S_ADDR_Msk                               (0x3FFUL << I2C_S_ADDR_S_ADDR_POS)
#define I2C_S_ADDR_S_ADDR                                   I2C_S_ADDR_S_ADDR_Msk

/*******************  Bit definition for I2C_M_HS_ADDR register  *******************/
#define I2C_M_HS_ADDR_HS_ADDR_POS                           (0U)
#define I2C_M_HS_ADDR_HS_ADDR_Len                           (3U)
#define I2C_M_HS_ADDR_HS_ADDR_Msk                           (0x7UL << I2C_M_HS_ADDR_HS_ADDR_POS)
#define I2C_M_HS_ADDR_HS_ADDR                               I2C_M_HS_ADDR_HS_ADDR_Msk

/*******************  Bit definition for I2C_DATA_CMD register  *******************/
#define I2C_DATA_CMD_DATA_POS                               (0U)
#define I2C_DATA_CMD_DATA_Len                               (8U)
#define I2C_DATA_CMD_DATA_Msk                               (0xFFUL << I2C_DATA_CMD_DATA_POS)
#define I2C_DATA_CMD_DATA                                   I2C_DATA_CMD_DATA_Msk

#define I2C_DATA_CMD_CMD_POS                                (8U)
#define I2C_DATA_CMD_CMD_Len                                (1U)
#define I2C_DATA_CMD_CMD_Msk                                (0x1UL << I2C_DATA_CMD_CMD_POS)
#define I2C_DATA_CMD_CMD                                    I2C_DATA_CMD_CMD_Msk

#define I2C_DATA_CMD_STOP_POS                               (9U)
#define I2C_DATA_CMD_STOP_Len                               (1U)
#define I2C_DATA_CMD_STOP_Msk                               (0x1UL << I2C_DATA_CMD_STOP_POS)
#define I2C_DATA_CMD_STOP                                   I2C_DATA_CMD_STOP_Msk

#define I2C_DATA_CMD_RESTART_POS                            (10U)
#define I2C_DATA_CMD_RESTART_Len                            (1U)
#define I2C_DATA_CMD_RESTART_Msk                            (0x1UL << I2C_DATA_CMD_RESTART_POS)
#define I2C_DATA_CMD_RESTART                                I2C_DATA_CMD_RESTART_Msk

/*******************  Bit definition for I2C_SS_CLK_HCOUNT register  *******************/
#define I2C_SS_CLK_HCOUNT_COUNT_POS                         (0U)
#define I2C_SS_CLK_HCOUNT_COUNT_Len                         (16U)
#define I2C_SS_CLK_HCOUNT_COUNT_Msk                         (0xFFFFUL << I2C_SS_CLK_HCOUNT_COUNT_POS)
#define I2C_SS_CLK_HCOUNT_COUNT                             I2C_SS_CLK_HCOUNT_COUNT_Msk

/*******************  Bit definition for I2C_SS_CLK_LCOUNT register  *******************/
#define I2C_SS_CLK_LCOUNT_COUNT_POS                         (0U)
#define I2C_SS_CLK_LCOUNT_COUNT_Len                         (16U)
#define I2C_SS_CLK_LCOUNT_COUNT_Msk                         (0xFFFFUL << I2C_SS_CLK_LCOUNT_COUNT_POS)
#define I2C_SS_CLK_LCOUNT_COUNT                             I2C_SS_CLK_LCOUNT_COUNT_Msk

/*******************  Bit definition for I2C_FS_CLK_HCOUNT register  *******************/
#define I2C_FS_CLK_HCOUNT_COUNT_POS                         (0U)
#define I2C_FS_CLK_HCOUNT_COUNT_Len                         (16U)
#define I2C_FS_CLK_HCOUNT_COUNT_Msk                         (0xFFFFUL << I2C_FS_CLK_HCOUNT_COUNT_POS)
#define I2C_FS_CLK_HCOUNT_COUNT                             I2C_FS_CLK_HCOUNT_COUNT_Msk

/*******************  Bit definition for I2C_FS_CLK_LCOUNT register  *******************/
#define I2C_FS_CLK_LCOUNT_COUNT_POS                         (0U)
#define I2C_FS_CLK_LCOUNT_COUNT_Len                         (16U)
#define I2C_FS_CLK_LCOUNT_COUNT_Msk                         (0xFFFFUL << I2C_FS_CLK_LCOUNT_COUNT_POS)
#define I2C_FS_CLK_LCOUNT_COUNT                             I2C_FS_CLK_LCOUNT_COUNT_Msk

/*******************  Bit definition for I2C_HS_CLK_HCOUNT register  *******************/
#define I2C_HS_CLK_HCOUNT_COUNT_POS                         (0U)
#define I2C_HS_CLK_HCOUNT_COUNT_Len                         (16U)
#define I2C_HS_CLK_HCOUNT_COUNT_Msk                         (0xFFFFUL << I2C_HS_CLK_HCOUNT_COUNT_POS)
#define I2C_HS_CLK_HCOUNT_COUNT                             I2C_HS_CLK_HCOUNT_COUNT_Msk

/*******************  Bit definition for I2C_HS_CLK_LCOUNT register  *******************/
#define I2C_HS_CLK_LCOUNT_COUNT_POS                         (0U)
#define I2C_HS_CLK_LCOUNT_COUNT_Len                         (16U)
#define I2C_HS_CLK_LCOUNT_COUNT_Msk                         (0xFFFFUL << I2C_HS_CLK_LCOUNT_COUNT_POS)
#define I2C_HS_CLK_LCOUNT_COUNT                             I2C_HS_CLK_LCOUNT_COUNT_Msk

/*******************  Bit definition for I2C_INT_STAT register  *******************/
#define I2C_INT_STAT_RAW_RX_UNDER_POS                       (0U)
#define I2C_INT_STAT_RAW_RX_UNDER_Len                       (1U)
#define I2C_INT_STAT_RAW_RX_UNDER_Msk                       (0x1UL << I2C_INT_STAT_RAW_RX_UNDER_POS)
#define I2C_INT_STAT_RAW_RX_UNDER                           I2C_INT_STAT_RAW_RX_UNDER_Msk

#define I2C_INT_STAT_RAW_RX_OVER_POS                        (1U)
#define I2C_INT_STAT_RAW_RX_OVER_Len                        (1U)
#define I2C_INT_STAT_RAW_RX_OVER_Msk                        (0x1UL << I2C_INT_STAT_RAW_RX_OVER_POS)
#define I2C_INT_STAT_RAW_RX_OVER                            I2C_INT_STAT_RAW_RX_OVER_Msk

#define I2C_INT_STAT_RAW_RX_FULL_POS                        (2U)
#define I2C_INT_STAT_RAW_RX_FULL_Len                        (1U)
#define I2C_INT_STAT_RAW_RX_FULL_Msk                        (0x1UL << I2C_INT_STAT_RAW_RX_FULL_POS)
#define I2C_INT_STAT_RAW_RX_FULL                            I2C_INT_STAT_RAW_RX_FULL_Msk

#define I2C_INT_STAT_RAW_TX_OVER_POS                        (3U)
#define I2C_INT_STAT_RAW_TX_OVER_Len                        (1U)
#define I2C_INT_STAT_RAW_TX_OVER_Msk                        (0x1UL << I2C_INT_STAT_RAW_TX_OVER_POS)
#define I2C_INT_STAT_RAW_TX_OVER                            I2C_INT_STAT_RAW_TX_OVER_Msk

#define I2C_INT_STAT_RAW_TX_EMPTY_POS                       (4U)
#define I2C_INT_STAT_RAW_TX_EMPTY_Len                       (1U)
#define I2C_INT_STAT_RAW_TX_EMPTY_Msk                       (0x1UL << I2C_INT_STAT_RAW_TX_EMPTY_POS)
#define I2C_INT_STAT_RAW_TX_EMPTY                           I2C_INT_STAT_RAW_TX_EMPTY_Msk

#define I2C_INT_STAT_RAW_RD_REQ_POS                         (5U)
#define I2C_INT_STAT_RAW_RD_REQ_Len                         (1U)
#define I2C_INT_STAT_RAW_RD_REQ_Msk                         (0x1UL << I2C_INT_STAT_RAW_RD_REQ_POS)
#define I2C_INT_STAT_RAW_RD_REQ                             I2C_INT_STAT_RAW_RD_REQ_Msk

#define I2C_INT_STAT_RAW_TX_ABORT_POS                       (6U)
#define I2C_INT_STAT_RAW_TX_ABORT_Len                       (1U)
#define I2C_INT_STAT_RAW_TX_ABORT_Msk                       (0x1UL << I2C_INT_STAT_RAW_TX_ABORT_POS)
#define I2C_INT_STAT_RAW_TX_ABORT                           I2C_INT_STAT_RAW_TX_ABORT_Msk

#define I2C_INT_STAT_RAW_RX_DONE_POS                        (7U)
#define I2C_INT_STAT_RAW_RX_DONE_Len                        (1U)
#define I2C_INT_STAT_RAW_RX_DONE_Msk                        (0x1UL << I2C_INT_STAT_RAW_RX_DONE_POS)
#define I2C_INT_STAT_RAW_RX_DONE                            I2C_INT_STAT_RAW_RX_DONE_Msk

#define I2C_INT_STAT_RAW_ACTIVITY_POS                       (8U)
#define I2C_INT_STAT_RAW_ACTIVITY_Len                       (1U)
#define I2C_INT_STAT_RAW_ACTIVITY_Msk                       (0x1UL << I2C_INT_STAT_RAW_ACTIVITY_POS)
#define I2C_INT_STAT_RAW_ACTIVITY                           I2C_INT_STAT_RAW_ACTIVITY_Msk

#define I2C_INT_STAT_RAW_STOP_DET_POS                       (9U)
#define I2C_INT_STAT_RAW_STOP_DET_Len                       (1U)
#define I2C_INT_STAT_RAW_STOP_DET_Msk                       (0x1UL << I2C_INT_STAT_RAW_STOP_DET_POS)
#define I2C_INT_STAT_RAW_STOP_DET                           I2C_INT_STAT_RAW_STOP_DET_Msk

#define I2C_INT_STAT_RAW_START_DET_POS                      (10U)
#define I2C_INT_STAT_RAW_START_DET_Len                      (1U)
#define I2C_INT_STAT_RAW_START_DET_Msk                      (0x1UL << I2C_INT_STAT_RAW_START_DET_POS)
#define I2C_INT_STAT_RAW_START_DET                          I2C_INT_STAT_RAW_START_DET_Msk

#define I2C_INT_STAT_RAW_GEN_CALL_POS                       (11U)
#define I2C_INT_STAT_RAW_GEN_CALL_Len                       (1U)
#define I2C_INT_STAT_RAW_GEN_CALL_Msk                       (0x1UL << I2C_INT_STAT_RAW_GEN_CALL_POS)
#define I2C_INT_STAT_RAW_GEN_CALL                           I2C_INT_STAT_RAW_GEN_CALL_Msk

#define I2C_INT_STAT_RAW_RESTART_DET_POS                    (12U)
#define I2C_INT_STAT_RAW_RESTART_DET_Len                    (1U)
#define I2C_INT_STAT_RAW_RESTART_DET_Msk                    (0x1UL << I2C_INT_STAT_RAW_RESTART_DET_POS)
#define I2C_INT_STAT_RAW_RESTART_DET                        I2C_INT_STAT_RAW_RESTART_DET_Msk

#define I2C_INT_STAT_RAW_M_HOLD_POS                         (13U)
#define I2C_INT_STAT_RAW_M_HOLD_Len                         (1U)
#define I2C_INT_STAT_RAW_M_HOLD_Msk                         (0x1UL << I2C_INT_STAT_RAW_M_HOLD_POS)
#define I2C_INT_STAT_RAW_M_HOLD                             I2C_INT_STAT_RAW_M_HOLD_Msk

#define I2C_INT_STAT_RAW_SCL_STUCKLOW_POS                   (14U)
#define I2C_INT_STAT_RAW_SCL_STUCKLOW_Len                   (1U)
#define I2C_INT_STAT_RAW_SCL_STUCKLOW_Msk                   (0x1UL << I2C_INT_STAT_RAW_SCL_STUCKLOW_POS)
#define I2C_INT_STAT_RAW_SCL_STUCKLOW                       I2C_INT_STAT_RAW_SCL_STUCKLOW_Msk

/*******************  Bit definition for I2C_INT_MASK register  *******************/
#define I2C_INT_MASK_MASK_RX_UNDER_POS                      (0U)
#define I2C_INT_MASK_MASK_RX_UNDER_Len                      (1U)
#define I2C_INT_MASK_MASK_RX_UNDER_Msk                      (0x1UL << I2C_INT_MASK_MASK_RX_UNDER_POS)
#define I2C_INT_MASK_MASK_RX_UNDER                          I2C_INT_MASK_MASK_RX_UNDER_Msk

#define I2C_INT_MASK_MASK_RX_OVER_POS                       (1U)
#define I2C_INT_MASK_MASK_RX_OVER_Len                       (1U)
#define I2C_INT_MASK_MASK_RX_OVER_Msk                       (0x1UL << I2C_INT_MASK_MASK_RX_OVER_POS)
#define I2C_INT_MASK_MASK_RX_OVER                           I2C_INT_MASK_MASK_RX_OVER_Msk

#define I2C_INT_MASK_MASK_RX_FULL_POS                       (2U)
#define I2C_INT_MASK_MASK_RX_FULL_Len                       (1U)
#define I2C_INT_MASK_MASK_RX_FULL_Msk                       (0x1UL << I2C_INT_MASK_MASK_RX_FULL_POS)
#define I2C_INT_MASK_MASK_RX_FULL                           I2C_INT_MASK_MASK_RX_FULL_Msk

#define I2C_INT_MASK_MASK_TX_OVER_POS                       (3U)
#define I2C_INT_MASK_MASK_TX_OVER_Len                       (1U)
#define I2C_INT_MASK_MASK_TX_OVER_Msk                       (0x1UL << I2C_INT_MASK_MASK_TX_OVER_POS)
#define I2C_INT_MASK_MASK_TX_OVER                           I2C_INT_MASK_MASK_TX_OVER_Msk

#define I2C_INT_MASK_MASK_TX_EMPTY_POS                      (4U)
#define I2C_INT_MASK_MASK_TX_EMPTY_Len                      (1U)
#define I2C_INT_MASK_MASK_TX_EMPTY_Msk                      (0x1UL << I2C_INT_MASK_MASK_TX_EMPTY_POS)
#define I2C_INT_MASK_MASK_TX_EMPTY                          I2C_INT_MASK_MASK_TX_EMPTY_Msk

#define I2C_INT_MASK_MASK_RD_REQ_POS                        (5U)
#define I2C_INT_MASK_MASK_RD_REQ_Len                        (1U)
#define I2C_INT_MASK_MASK_RD_REQ_Msk                        (0x1UL << I2C_INT_MASK_MASK_RD_REQ_POS)
#define I2C_INT_MASK_MASK_RD_REQ                            I2C_INT_MASK_MASK_RD_REQ_Msk

#define I2C_INT_MASK_MASK_TX_ABORT_POS                      (6U)
#define I2C_INT_MASK_MASK_TX_ABORT_Len                      (1U)
#define I2C_INT_MASK_MASK_TX_ABORT_Msk                      (0x1UL << I2C_INT_MASK_MASK_TX_ABORT_POS)
#define I2C_INT_MASK_MASK_TX_ABORT                          I2C_INT_MASK_MASK_TX_ABORT_Msk

#define I2C_INT_MASK_MASK_RX_DONE_POS                       (7U)
#define I2C_INT_MASK_MASK_RX_DONE_Len                       (1U)
#define I2C_INT_MASK_MASK_RX_DONE_Msk                       (0x1UL << I2C_INT_MASK_MASK_RX_DONE_POS)
#define I2C_INT_MASK_MASK_RX_DONE                           I2C_INT_MASK_MASK_RX_DONE_Msk

#define I2C_INT_MASK_MASK_ACTIVITY_POS                      (8U)
#define I2C_INT_MASK_MASK_ACTIVITY_Len                      (1U)
#define I2C_INT_MASK_MASK_ACTIVITY_Msk                      (0x1UL << I2C_INT_MASK_MASK_ACTIVITY_POS)
#define I2C_INT_MASK_MASK_ACTIVITY                          I2C_INT_MASK_MASK_ACTIVITY_Msk

#define I2C_INT_MASK_MASK_STOP_DET_POS                      (9U)
#define I2C_INT_MASK_MASK_STOP_DET_Len                      (1U)
#define I2C_INT_MASK_MASK_STOP_DET_Msk                      (0x1UL << I2C_INT_MASK_MASK_STOP_DET_POS)
#define I2C_INT_MASK_MASK_STOP_DET                          I2C_INT_MASK_MASK_STOP_DET_Msk

#define I2C_INT_MASK_MASK_START_DET_POS                     (10U)
#define I2C_INT_MASK_MASK_START_DET_Len                     (1U)
#define I2C_INT_MASK_MASK_START_DET_Msk                     (0x1UL << I2C_INT_MASK_MASK_START_DET_POS)
#define I2C_INT_MASK_MASK_START_DET                         I2C_INT_MASK_MASK_START_DET_Msk

#define I2C_INT_MASK_MASK_GEN_CALL_POS                      (11U)
#define I2C_INT_MASK_MASK_GEN_CALL_Len                      (1U)
#define I2C_INT_MASK_MASK_GEN_CALL_Msk                      (0x1UL << I2C_INT_MASK_MASK_GEN_CALL_POS)
#define I2C_INT_MASK_MASK_GEN_CALL                          I2C_INT_MASK_MASK_GEN_CALL_Msk

#define I2C_INT_MASK_MASK_RESTART_DET_POS                   (12U)
#define I2C_INT_MASK_MASK_RESTART_DET_Len                   (1U)
#define I2C_INT_MASK_MASK_RESTART_DET_Msk                   (0x1UL << I2C_INT_MASK_MASK_RESTART_DET_POS)
#define I2C_INT_MASK_MASK_RESTART_DET                       I2C_INT_MASK_MASK_RESTART_DET_Msk

#define I2C_INT_MASK_MASK_M_HOLD_POS                        (13U)
#define I2C_INT_MASK_MASK_M_HOLD_Len                        (1U)
#define I2C_INT_MASK_MASK_M_HOLD_Msk                        (0x1UL << I2C_INT_MASK_MASK_M_HOLD_POS)
#define I2C_INT_MASK_MASK_M_HOLD                            I2C_INT_MASK_MASK_M_HOLD_Msk

#define I2C_INT_MASK_MASK_SCL_STUCKLOW_POS                  (14U)
#define I2C_INT_MASK_MASK_SCL_STUCKLOW_Len                  (1U)
#define I2C_INT_MASK_MASK_SCL_STUCKLOW_Msk                  (0x1UL << I2C_INT_MASK_MASK_SCL_STUCKLOW_POS)
#define I2C_INT_MASK_MASK_SCL_STUCKLOW                      I2C_INT_MASK_MASK_SCL_STUCKLOW_Msk

/*******************  Bit definition for I2C_RAW_INT_STAT register  *******************/
#define I2C_RAW_INT_STAT_RX_UNDER_POS                       (0U)
#define I2C_RAW_INT_STAT_RX_UNDER_Len                       (1U)
#define I2C_RAW_INT_STAT_RX_UNDER_Msk                       (0x1UL << I2C_RAW_INT_STAT_RX_UNDER_POS)
#define I2C_RAW_INT_STAT_RX_UNDER                           I2C_RAW_INT_STAT_RX_UNDER_Msk

#define I2C_RAW_INT_STAT_RX_OVER_POS                        (1U)
#define I2C_RAW_INT_STAT_RX_OVER_Len                        (1U)
#define I2C_RAW_INT_STAT_RX_OVER_Msk                        (0x1UL << I2C_RAW_INT_STAT_RX_OVER_POS)
#define I2C_RAW_INT_STAT_RX_OVER                            I2C_RAW_INT_STAT_RX_OVER_Msk

#define I2C_RAW_INT_STAT_RX_FULL_POS                        (2U)
#define I2C_RAW_INT_STAT_RX_FULL_Len                        (1U)
#define I2C_RAW_INT_STAT_RX_FULL_Msk                        (0x1UL << I2C_RAW_INT_STAT_RX_FULL_POS)
#define I2C_RAW_INT_STAT_RX_FULL                            I2C_RAW_INT_STAT_RX_FULL_Msk

#define I2C_RAW_INT_STAT_TX_OVER_POS                        (3U)
#define I2C_RAW_INT_STAT_TX_OVER_Len                        (1U)
#define I2C_RAW_INT_STAT_TX_OVER_Msk                        (0x1UL << I2C_RAW_INT_STAT_TX_OVER_POS)
#define I2C_RAW_INT_STAT_TX_OVER                            I2C_RAW_INT_STAT_TX_OVER_Msk

#define I2C_RAW_INT_STAT_TX_EMPTY_POS                       (4U)
#define I2C_RAW_INT_STAT_TX_EMPTY_Len                       (1U)
#define I2C_RAW_INT_STAT_TX_EMPTY_Msk                       (0x1UL << I2C_RAW_INT_STAT_TX_EMPTY_POS)
#define I2C_RAW_INT_STAT_TX_EMPTY                           I2C_RAW_INT_STAT_TX_EMPTY_Msk

#define I2C_RAW_INT_STAT_RD_REQ_POS                         (5U)
#define I2C_RAW_INT_STAT_RD_REQ_Len                         (1U)
#define I2C_RAW_INT_STAT_RD_REQ_Msk                         (0x1UL << I2C_RAW_INT_STAT_RD_REQ_POS)
#define I2C_RAW_INT_STAT_RD_REQ                             I2C_RAW_INT_STAT_RD_REQ_Msk

#define I2C_RAW_INT_STAT_TX_ABORT_POS                       (6U)
#define I2C_RAW_INT_STAT_TX_ABORT_Len                       (1U)
#define I2C_RAW_INT_STAT_TX_ABORT_Msk                       (0x1UL << I2C_RAW_INT_STAT_TX_ABORT_POS)
#define I2C_RAW_INT_STAT_TX_ABORT                           I2C_RAW_INT_STAT_TX_ABORT_Msk

#define I2C_RAW_INT_STAT_RX_DONE_POS                        (7U)
#define I2C_RAW_INT_STAT_RX_DONE_Len                        (1U)
#define I2C_RAW_INT_STAT_RX_DONE_Msk                        (0x1UL << I2C_RAW_INT_STAT_RX_DONE_POS)
#define I2C_RAW_INT_STAT_RX_DONE                            I2C_RAW_INT_STAT_RX_DONE_Msk

#define I2C_RAW_INT_STAT_ACTIVITY_POS                       (8U)
#define I2C_RAW_INT_STAT_ACTIVITY_Len                       (1U)
#define I2C_RAW_INT_STAT_ACTIVITY_Msk                       (0x1UL << I2C_RAW_INT_STAT_ACTIVITY_POS)
#define I2C_RAW_INT_STAT_ACTIVITY                           I2C_RAW_INT_STAT_ACTIVITY_Msk

#define I2C_RAW_INT_STAT_STOP_DET_POS                       (9U)
#define I2C_RAW_INT_STAT_STOP_DET_Len                       (1U)
#define I2C_RAW_INT_STAT_STOP_DET_Msk                       (0x1UL << I2C_RAW_INT_STAT_STOP_DET_POS)
#define I2C_RAW_INT_STAT_STOP_DET                           I2C_RAW_INT_STAT_STOP_DET_Msk

#define I2C_RAW_INT_STAT_START_DET_POS                      (10U)
#define I2C_RAW_INT_STAT_START_DET_Len                      (1U)
#define I2C_RAW_INT_STAT_START_DET_Msk                      (0x1UL << I2C_RAW_INT_STAT_START_DET_POS)
#define I2C_RAW_INT_STAT_START_DET                          I2C_RAW_INT_STAT_START_DET_Msk

#define I2C_RAW_INT_STAT_GEN_CALL_POS                       (11U)
#define I2C_RAW_INT_STAT_GEN_CALL_Len                       (1U)
#define I2C_RAW_INT_STAT_GEN_CALL_Msk                       (0x1UL << I2C_RAW_INT_STAT_GEN_CALL_POS)
#define I2C_RAW_INT_STAT_GEN_CALL                           I2C_RAW_INT_STAT_GEN_CALL_Msk

#define I2C_RAW_INT_STAT_RESTART_DET_POS                    (12U)
#define I2C_RAW_INT_STAT_RESTART_DET_Len                    (1U)
#define I2C_RAW_INT_STAT_RESTART_DET_Msk                    (0x1UL << I2C_RAW_INT_STAT_RESTART_DET_POS)
#define I2C_RAW_INT_STAT_RESTART_DET                        I2C_RAW_INT_STAT_RESTART_DET_Msk

#define I2C_RAW_INT_STAT_M_HOLD_POS                         (13U)
#define I2C_RAW_INT_STAT_M_HOLD_Len                         (1U)
#define I2C_RAW_INT_STAT_M_HOLD_Msk                         (0x1UL << I2C_RAW_INT_STAT_M_HOLD_POS)
#define I2C_RAW_INT_STAT_M_HOLD                             I2C_RAW_INT_STAT_M_HOLD_Msk

#define I2C_RAW_INT_STAT_SCL_STUCKLOW_POS                   (14U)
#define I2C_RAW_INT_STAT_SCL_STUCKLOW_Len                   (1U)
#define I2C_RAW_INT_STAT_SCL_STUCKLOW_Msk                   (0x1UL << I2C_RAW_INT_STAT_SCL_STUCKLOW_POS)
#define I2C_RAW_INT_STAT_SCL_STUCKLOW                       I2C_RAW_INT_STAT_SCL_STUCKLOW_Msk

/*******************  Bit definition for I2C_RX_FIFO_THD register  *******************/
#define I2C_RX_FIFO_THD_THD_POS                             (0U)
#define I2C_RX_FIFO_THD_THD_Len                             (8U)
#define I2C_RX_FIFO_THD_THD_Msk                             (0xFFUL << I2C_RX_FIFO_THD_THD_POS)
#define I2C_RX_FIFO_THD_THD                                 I2C_RX_FIFO_THD_THD_Msk

/*******************  Bit definition for I2C_TX_FIFO_THD register  *******************/
#define I2C_TX_FIFO_THD_THD_POS                             (0U)
#define I2C_TX_FIFO_THD_THD_Len                             (8U)
#define I2C_TX_FIFO_THD_THD_Msk                             (0xFFUL << I2C_TX_FIFO_THD_THD_POS)
#define I2C_TX_FIFO_THD_THD                                 I2C_TX_FIFO_THD_THD_Msk

/*******************  Bit definition for I2C_CLR_INT register  *******************/
#define I2C_CLR_INT_CLR_INT_POS                             (0U)
#define I2C_CLR_INT_CLR_INT_Len                             (1U)
#define I2C_CLR_INT_CLR_INT_Msk                             (0x1UL << I2C_CLR_INT_CLR_INT_POS)
#define I2C_CLR_INT_CLR_INT                                 I2C_CLR_INT_CLR_INT_Msk

/*******************  Bit definition for I2C_CLR_RX_UNDER register  *******************/
#define I2C_CLR_RX_UNDER_CLR_RX_UNDER_POS                   (0U)
#define I2C_CLR_RX_UNDER_CLR_RX_UNDER_Len                   (1U)
#define I2C_CLR_RX_UNDER_CLR_RX_UNDER_Msk                   (0x1UL << I2C_CLR_RX_UNDER_CLR_RX_UNDER_POS)
#define I2C_CLR_RX_UNDER_CLR_RX_UNDER                       I2C_CLR_RX_UNDER_CLR_RX_UNDER_Msk

/*******************  Bit definition for I2C_CLR_RX_OVER register  *******************/
#define I2C_CLR_RX_OVER_CLR_RX_OVER_POS                     (0U)
#define I2C_CLR_RX_OVER_CLR_RX_OVER_Len                     (1U)
#define I2C_CLR_RX_OVER_CLR_RX_OVER_Msk                     (0x1UL << I2C_CLR_RX_OVER_CLR_RX_OVER_POS)
#define I2C_CLR_RX_OVER_CLR_RX_OVER                         I2C_CLR_RX_OVER_CLR_RX_OVER_Msk

/*******************  Bit definition for I2C_CLR_TX_OVER register  *******************/
#define I2C_CLR_TX_OVER_CLR_TX_OVER_POS                     (0U)
#define I2C_CLR_TX_OVER_CLR_TX_OVER_Len                     (1U)
#define I2C_CLR_TX_OVER_CLR_TX_OVER_Msk                     (0x1UL << I2C_CLR_TX_OVER_CLR_TX_OVER_POS)
#define I2C_CLR_TX_OVER_CLR_TX_OVER                         I2C_CLR_TX_OVER_CLR_TX_OVER_Msk

/*******************  Bit definition for I2C_CLR_RD_REQ register  *******************/
#define I2C_CLR_RD_REQ_CLR_RD_REQ_POS                       (0U)
#define I2C_CLR_RD_REQ_CLR_RD_REQ_Len                       (1U)
#define I2C_CLR_RD_REQ_CLR_RD_REQ_Msk                       (0x1UL << I2C_CLR_RD_REQ_CLR_RD_REQ_POS)
#define I2C_CLR_RD_REQ_CLR_RD_REQ                           I2C_CLR_RD_REQ_CLR_RD_REQ_Msk

/*******************  Bit definition for I2C_CLR_TX_ABORT register  *******************/
#define I2C_CLR_TX_ABORT_CLR_TX_ABORT_POS                   (0U)
#define I2C_CLR_TX_ABORT_CLR_TX_ABORT_Len                   (1U)
#define I2C_CLR_TX_ABORT_CLR_TX_ABORT_Msk                   (0x1UL << I2C_CLR_TX_ABORT_CLR_TX_ABORT_POS)
#define I2C_CLR_TX_ABORT_CLR_TX_ABORT                       I2C_CLR_TX_ABORT_CLR_TX_ABORT_Msk

/*******************  Bit definition for I2C_CLR_RX_DONE register  *******************/
#define I2C_CLR_RX_DONE_CLR_RX_DONE_POS                     (0U)
#define I2C_CLR_RX_DONE_CLR_RX_DONE_Len                     (1U)
#define I2C_CLR_RX_DONE_CLR_RX_DONE_Msk                     (0x1UL << I2C_CLR_RX_DONE_CLR_RX_DONE_POS)
#define I2C_CLR_RX_DONE_CLR_RX_DONE                         I2C_CLR_RX_DONE_CLR_RX_DONE_Msk

/*******************  Bit definition for I2C_CLR_ACTIVITY register  *******************/
#define I2C_CLR_ACTIVITY_CLR_ACTIVITY_POS                   (0U)
#define I2C_CLR_ACTIVITY_CLR_ACTIVITY_Len                   (1U)
#define I2C_CLR_ACTIVITY_CLR_ACTIVITY_Msk                   (0x1UL << I2C_CLR_ACTIVITY_CLR_ACTIVITY_POS)
#define I2C_CLR_ACTIVITY_CLR_ACTIVITY                       I2C_CLR_ACTIVITY_CLR_ACTIVITY_Msk

/*******************  Bit definition for I2C_CLR_STOP_DET register  *******************/
#define I2C_CLR_STOP_DET_CLR_STOP_DET_POS                   (0U)
#define I2C_CLR_STOP_DET_CLR_STOP_DET_Len                   (1U)
#define I2C_CLR_STOP_DET_CLR_STOP_DET_Msk                   (0x1UL << I2C_CLR_STOP_DET_CLR_STOP_DET_POS)
#define I2C_CLR_STOP_DET_CLR_STOP_DET                       I2C_CLR_STOP_DET_CLR_STOP_DET_Msk

/*******************  Bit definition for I2C_CLR_START_DET register  *******************/
#define I2C_CLR_START_DET_CLR_START_DET_POS                 (0U)
#define I2C_CLR_START_DET_CLR_START_DET_Len                 (1U)
#define I2C_CLR_START_DET_CLR_START_DET_Msk                 (0x1UL << I2C_CLR_START_DET_CLR_START_DET_POS)
#define I2C_CLR_START_DET_CLR_START_DET                     I2C_CLR_START_DET_CLR_START_DET_Msk

/*******************  Bit definition for I2C_CLR_GEN_CALL register  *******************/
#define I2C_CLR_GEN_CALL_CLR_GEN_CALL_POS                   (0U)
#define I2C_CLR_GEN_CALL_CLR_GEN_CALL_Len                   (1U)
#define I2C_CLR_GEN_CALL_CLR_GEN_CALL_Msk                   (0x1UL << I2C_CLR_GEN_CALL_CLR_GEN_CALL_POS)
#define I2C_CLR_GEN_CALL_CLR_GEN_CALL                       I2C_CLR_GEN_CALL_CLR_GEN_CALL_Msk

/*******************  Bit definition for I2C_EN register  *******************/
#define I2C_EN_ACTIVITY_POS                                 (0U)
#define I2C_EN_ACTIVITY_Len                                 (1U)
#define I2C_EN_ACTIVITY_Msk                                 (0x1UL << I2C_EN_ACTIVITY_POS)
#define I2C_EN_ACTIVITY                                     I2C_EN_ACTIVITY_Msk

#define I2C_EN_ABORT_POS                                    (1U)
#define I2C_EN_ABORT_Len                                    (1U)
#define I2C_EN_ABORT_Msk                                    (0x1UL << I2C_EN_ABORT_POS)
#define I2C_EN_ABORT                                        I2C_EN_ABORT_Msk

#define I2C_EN_TX_CMD_BLOCK_POS                             (2U)
#define I2C_EN_TX_CMD_BLOCK_Len                             (1U)
#define I2C_EN_TX_CMD_BLOCK_Msk                             (0x1UL << I2C_EN_TX_CMD_BLOCK_POS)
#define I2C_EN_TX_CMD_BLOCK                                 I2C_EN_TX_CMD_BLOCK_Msk

#define I2C_EN_SDA_STUCK_RECOVERY_POS                       (3U)
#define I2C_EN_SDA_STUCK_RECOVERY_Len                       (1U)
#define I2C_EN_SDA_STUCK_RECOVERY_Msk                       (0x1UL << I2C_EN_SDA_STUCK_RECOVERY_POS)
#define I2C_EN_SDA_STUCK_RECOVERY                           I2C_EN_SDA_STUCK_RECOVERY_Msk

/*******************  Bit definition for I2C_STAT register  *******************/
#define I2C_STAT_ACTIVITY_POS                               (0U)
#define I2C_STAT_ACTIVITY_Len                               (1U)
#define I2C_STAT_ACTIVITY_Msk                               (0x1UL << I2C_STAT_ACTIVITY_POS)
#define I2C_STAT_ACTIVITY                                   I2C_STAT_ACTIVITY_Msk

#define I2C_STAT_TX_FIFO_NF_POS                             (1U)
#define I2C_STAT_TX_FIFO_NF_Len                             (1U)
#define I2C_STAT_TX_FIFO_NF_Msk                             (0x1UL << I2C_STAT_TX_FIFO_NF_POS)
#define I2C_STAT_TX_FIFO_NF                                 I2C_STAT_TX_FIFO_NF_Msk

#define I2C_STAT_TX_FIFO_CE_POS                             (2U)
#define I2C_STAT_TX_FIFO_CE_Len                             (1U)
#define I2C_STAT_TX_FIFO_CE_Msk                             (0x1UL << I2C_STAT_TX_FIFO_CE_POS)
#define I2C_STAT_TX_FIFO_CE                                 I2C_STAT_TX_FIFO_CE_Msk

#define I2C_STAT_RX_FIFO_NE_POS                             (3U)
#define I2C_STAT_RX_FIFO_NE_Len                             (1U)
#define I2C_STAT_RX_FIFO_NE_Msk                             (0x1UL << I2C_STAT_RX_FIFO_NE_POS)
#define I2C_STAT_RX_FIFO_NE                                 I2C_STAT_RX_FIFO_NE_Msk

#define I2C_STAT_RX_FIFO_CF_POS                             (4U)
#define I2C_STAT_RX_FIFO_CF_Len                             (1U)
#define I2C_STAT_RX_FIFO_CF_Msk                             (0x1UL << I2C_STAT_RX_FIFO_CF_POS)
#define I2C_STAT_RX_FIFO_CF                                 I2C_STAT_RX_FIFO_CF_Msk

#define I2C_STAT_M_ACTIVITY_POS                             (5U)
#define I2C_STAT_M_ACTIVITY_Len                             (1U)
#define I2C_STAT_M_ACTIVITY_Msk                             (0x1UL << I2C_STAT_M_ACTIVITY_POS)
#define I2C_STAT_M_ACTIVITY                                 I2C_STAT_M_ACTIVITY_Msk

#define I2C_STAT_S_ACTIVITY_POS                             (6U)
#define I2C_STAT_S_ACTIVITY_Len                             (1U)
#define I2C_STAT_S_ACTIVITY_Msk                             (0x1UL << I2C_STAT_S_ACTIVITY_POS)
#define I2C_STAT_S_ACTIVITY                                 I2C_STAT_S_ACTIVITY_Msk

#define I2C_STAT_SDA_STUCK_RCVR_POS                         (11U)
#define I2C_STAT_SDA_STUCK_RCVR_Len                         (1U)
#define I2C_STAT_SDA_STUCK_RCVR_Msk                         (0x1UL << I2C_STAT_SDA_STUCK_RCVR_POS)
#define I2C_STAT_SDA_STUCK_RCVR                             I2C_STAT_SDA_STUCK_RCVR_Msk

/*******************  Bit definition for I2C_TX_FIFO_LEVEL register  *******************/
#define I2C_TX_FIFO_LEVEL_LEVEL_POS                         (0U)
#define I2C_TX_FIFO_LEVEL_LEVEL_Len                         (8U)
#define I2C_TX_FIFO_LEVEL_LEVEL_Msk                         (0xFFUL << I2C_TX_FIFO_LEVEL_LEVEL_POS)
#define I2C_TX_FIFO_LEVEL_LEVEL                             I2C_TX_FIFO_LEVEL_LEVEL_Msk

/*******************  Bit definition for I2C_RX_FIFO_LEVEL register  *******************/
#define I2C_RX_FIFO_LEVEL_LEVEL_POS                         (0U)
#define I2C_RX_FIFO_LEVEL_LEVEL_Len                         (8U)
#define I2C_RX_FIFO_LEVEL_LEVEL_Msk                         (0xFFUL << I2C_RX_FIFO_LEVEL_LEVEL_POS)
#define I2C_RX_FIFO_LEVEL_LEVEL                             I2C_RX_FIFO_LEVEL_LEVEL_Msk

/*******************  Bit definition for I2C_SDA_HOLD register  *******************/
#define I2C_SDA_HOLD_TX_HOLD_POS                            (0U)
#define I2C_SDA_HOLD_TX_HOLD_Len                            (16U)
#define I2C_SDA_HOLD_TX_HOLD_Msk                            (0xFFFFUL << I2C_SDA_HOLD_TX_HOLD_POS)
#define I2C_SDA_HOLD_TX_HOLD                                I2C_SDA_HOLD_TX_HOLD_Msk

#define I2C_SDA_HOLD_RX_HOLD_POS                            (16U)
#define I2C_SDA_HOLD_RX_HOLD_Len                            (8U)
#define I2C_SDA_HOLD_RX_HOLD_Msk                            (0xFFUL << I2C_SDA_HOLD_RX_HOLD_POS)
#define I2C_SDA_HOLD_RX_HOLD                                I2C_SDA_HOLD_RX_HOLD_Msk

/*******************  Bit definition for I2C_TX_ABORT_SRC register  *******************/
#define I2C_TX_ABORT_SRC_ABORT_7B_NOACK_POS                 (0U)
#define I2C_TX_ABORT_SRC_ABORT_7B_NOACK_Len                 (1U)
#define I2C_TX_ABORT_SRC_ABORT_7B_NOACK_Msk                 (0x1UL << I2C_TX_ABORT_SRC_ABORT_7B_NOACK_POS)
#define I2C_TX_ABORT_SRC_ABORT_7B_NOACK                     I2C_TX_ABORT_SRC_ABORT_7B_NOACK_Msk

#define I2C_TX_ABORT_SRC_ABORT_10B1_NOACK_POS               (1U)
#define I2C_TX_ABORT_SRC_ABORT_10B1_NOACK_Len               (1U)
#define I2C_TX_ABORT_SRC_ABORT_10B1_NOACK_Msk               (0x1UL << I2C_TX_ABORT_SRC_ABORT_10B1_NOACK_POS)
#define I2C_TX_ABORT_SRC_ABORT_10B1_NOACK                   I2C_TX_ABORT_SRC_ABORT_10B1_NOACK_Msk

#define I2C_TX_ABORT_SRC_ABORT_10B2_NOACK_POS               (2U)
#define I2C_TX_ABORT_SRC_ABORT_10B2_NOACK_Len               (1U)
#define I2C_TX_ABORT_SRC_ABORT_10B2_NOACK_Msk               (0x1UL << I2C_TX_ABORT_SRC_ABORT_10B2_NOACK_POS)
#define I2C_TX_ABORT_SRC_ABORT_10B2_NOACK                   I2C_TX_ABORT_SRC_ABORT_10B2_NOACK_Msk

#define I2C_TX_ABORT_SRC_ABORT_TX_NOACK_POS                 (3U)
#define I2C_TX_ABORT_SRC_ABORT_TX_NOACK_Len                 (1U)
#define I2C_TX_ABORT_SRC_ABORT_TX_NOACK_Msk                 (0x1UL << I2C_TX_ABORT_SRC_ABORT_TX_NOACK_POS)
#define I2C_TX_ABORT_SRC_ABORT_TX_NOACK                     I2C_TX_ABORT_SRC_ABORT_TX_NOACK_Msk

#define I2C_TX_ABORT_SRC_ABORT_GCALL_NOACK_POS              (4U)
#define I2C_TX_ABORT_SRC_ABORT_GCALL_NOACK_Len              (1U)
#define I2C_TX_ABORT_SRC_ABORT_GCALL_NOACK_Msk              (0x1UL << I2C_TX_ABORT_SRC_ABORT_GCALL_NOACK_POS)
#define I2C_TX_ABORT_SRC_ABORT_GCALL_NOACK                  I2C_TX_ABORT_SRC_ABORT_GCALL_NOACK_Msk

#define I2C_TX_ABORT_SRC_ABORT_GCALL_RD_POS                 (5U)
#define I2C_TX_ABORT_SRC_ABORT_GCALL_RD_Len                 (1U)
#define I2C_TX_ABORT_SRC_ABORT_GCALL_RD_Msk                 (0x1UL << I2C_TX_ABORT_SRC_ABORT_GCALL_RD_POS)
#define I2C_TX_ABORT_SRC_ABORT_GCALL_RD                     I2C_TX_ABORT_SRC_ABORT_GCALL_RD_Msk

#define I2C_TX_ABORT_SRC_ABORT_HS_ACKDET_POS                (6U)
#define I2C_TX_ABORT_SRC_ABORT_HS_ACKDET_Len                (1U)
#define I2C_TX_ABORT_SRC_ABORT_HS_ACKDET_Msk                (0x1UL << I2C_TX_ABORT_SRC_ABORT_HS_ACKDET_POS)
#define I2C_TX_ABORT_SRC_ABORT_HS_ACKDET                    I2C_TX_ABORT_SRC_ABORT_HS_ACKDET_Msk

#define I2C_TX_ABORT_SRC_ABORT_SBYTE_ACKDET_POS             (7U)
#define I2C_TX_ABORT_SRC_ABORT_SBYTE_ACKDET_Len             (1U)
#define I2C_TX_ABORT_SRC_ABORT_SBYTE_ACKDET_Msk             (0x1UL << I2C_TX_ABORT_SRC_ABORT_SBYTE_ACKDET_POS)
#define I2C_TX_ABORT_SRC_ABORT_SBYTE_ACKDET                 I2C_TX_ABORT_SRC_ABORT_SBYTE_ACKDET_Msk

#define I2C_TX_ABORT_SRC_ABORT_HS_NORSTRT_POS               (8U)
#define I2C_TX_ABORT_SRC_ABORT_HS_NORSTRT_Len               (1U)
#define I2C_TX_ABORT_SRC_ABORT_HS_NORSTRT_Msk               (0x1UL << I2C_TX_ABORT_SRC_ABORT_HS_NORSTRT_POS)
#define I2C_TX_ABORT_SRC_ABORT_HS_NORSTRT                   I2C_TX_ABORT_SRC_ABORT_HS_NORSTRT_Msk

#define I2C_TX_ABORT_SRC_ABORT_SBYTE_NORSTRT_POS            (9U)
#define I2C_TX_ABORT_SRC_ABORT_SBYTE_NORSTRT_Len            (1U)
#define I2C_TX_ABORT_SRC_ABORT_SBYTE_NORSTRT_Msk            (0x1UL << I2C_TX_ABORT_SRC_ABORT_SBYTE_NORSTRT_POS)
#define I2C_TX_ABORT_SRC_ABORT_SBYTE_NORSTRT                I2C_TX_ABORT_SRC_ABORT_SBYTE_NORSTRT_Msk

#define I2C_TX_ABORT_SRC_ABORT_10B_RD_NORSTR_POS            (10U)
#define I2C_TX_ABORT_SRC_ABORT_10B_RD_NORSTR_Len            (1U)
#define I2C_TX_ABORT_SRC_ABORT_10B_RD_NORSTR_Msk            (0x1UL << I2C_TX_ABORT_SRC_ABORT_10B_RD_NORSTR_POS)
#define I2C_TX_ABORT_SRC_ABORT_10B_RD_NORSTR                I2C_TX_ABORT_SRC_ABORT_10B_RD_NORSTR_Msk

#define I2C_TX_ABORT_SRC_ABORT_MASTER_DIS_POS               (11U)
#define I2C_TX_ABORT_SRC_ABORT_MASTER_DIS_Len               (1U)
#define I2C_TX_ABORT_SRC_ABORT_MASTER_DIS_Msk               (0x1UL << I2C_TX_ABORT_SRC_ABORT_MASTER_DIS_POS)
#define I2C_TX_ABORT_SRC_ABORT_MASTER_DIS                   I2C_TX_ABORT_SRC_ABORT_MASTER_DIS_Msk

#define I2C_TX_ABORT_SRC_ABORT_LOST_POS                     (12U)
#define I2C_TX_ABORT_SRC_ABORT_LOST_Len                     (1U)
#define I2C_TX_ABORT_SRC_ABORT_LOST_Msk                     (0x1UL << I2C_TX_ABORT_SRC_ABORT_LOST_POS)
#define I2C_TX_ABORT_SRC_ABORT_LOST                         I2C_TX_ABORT_SRC_ABORT_LOST_Msk

#define I2C_TX_ABORT_SRC_ABORT_SLVFLUSH_TXFIFO_POS          (13U)
#define I2C_TX_ABORT_SRC_ABORT_SLVFLUSH_TXFIFO_Len          (1U)
#define I2C_TX_ABORT_SRC_ABORT_SLVFLUSH_TXFIFO_Msk          (0x1UL << I2C_TX_ABORT_SRC_ABORT_SLVFLUSH_TXFIFO_POS)
#define I2C_TX_ABORT_SRC_ABORT_SLVFLUSH_TXFIFO              I2C_TX_ABORT_SRC_ABORT_SLVFLUSH_TXFIFO_Msk

#define I2C_TX_ABORT_SRC_ABORT_S_ARBLOST_POS                (14U)
#define I2C_TX_ABORT_SRC_ABORT_S_ARBLOST_Len                (1U)
#define I2C_TX_ABORT_SRC_ABORT_S_ARBLOST_Msk                (0x1UL << I2C_TX_ABORT_SRC_ABORT_S_ARBLOST_POS)
#define I2C_TX_ABORT_SRC_ABORT_S_ARBLOST                    I2C_TX_ABORT_SRC_ABORT_S_ARBLOST_Msk

#define I2C_TX_ABORT_SRC_ABORT_SLVRD_INTX_POS               (15U)
#define I2C_TX_ABORT_SRC_ABORT_SLVRD_INTX_Len               (1U)
#define I2C_TX_ABORT_SRC_ABORT_SLVRD_INTX_Msk               (0x1UL << I2C_TX_ABORT_SRC_ABORT_SLVRD_INTX_POS)
#define I2C_TX_ABORT_SRC_ABORT_SLVRD_INTX                   I2C_TX_ABORT_SRC_ABORT_SLVRD_INTX_Msk

#define I2C_TX_ABORT_SRC_ABORT_USER_ABORT_POS               (16U)
#define I2C_TX_ABORT_SRC_ABORT_USER_ABORT_Len               (1U)
#define I2C_TX_ABORT_SRC_ABORT_USER_ABORT_Msk               (0x1UL << I2C_TX_ABORT_SRC_ABORT_USER_ABORT_POS)
#define I2C_TX_ABORT_SRC_ABORT_USER_ABORT                   I2C_TX_ABORT_SRC_ABORT_USER_ABORT_Msk

#define I2C_TX_ABORT_SRC_ABORT_SDA_STUCK_POS                (17U)
#define I2C_TX_ABORT_SRC_ABORT_SDA_STUCK_Len                (1U)
#define I2C_TX_ABORT_SRC_ABORT_SDA_STUCK_Msk                (0x1UL << I2C_TX_ABORT_SRC_ABORT_SDA_STUCK_POS)
#define I2C_TX_ABORT_SRC_ABORT_SDA_STUCK                    I2C_TX_ABORT_SRC_ABORT_SDA_STUCK_Msk

#define I2C_TX_ABORT_SRC_TX_FLUSH_CNT_POS                   (23U)
#define I2C_TX_ABORT_SRC_TX_FLUSH_CNT_Len                   (9U)
#define I2C_TX_ABORT_SRC_TX_FLUSH_CNT_Msk                   (0x1FFUL << I2C_TX_ABORT_SRC_TX_FLUSH_CNT_POS)
#define I2C_TX_ABORT_SRC_TX_FLUSH_CNT                       I2C_TX_ABORT_SRC_TX_FLUSH_CNT_Msk

/*******************  Bit definition for I2C_DMA_CTRL register  *******************/
#define I2C_DMA_CTRL_RX_EN_POS                              (0U)
#define I2C_DMA_CTRL_RX_EN_Len                              (1U)
#define I2C_DMA_CTRL_RX_EN_Msk                              (0x1UL << I2C_DMA_CTRL_RX_EN_POS)
#define I2C_DMA_CTRL_RX_EN                                  I2C_DMA_CTRL_RX_EN_Msk

#define I2C_DMA_CTRL_TX_EN_POS                              (1U)
#define I2C_DMA_CTRL_TX_EN_Len                              (1U)
#define I2C_DMA_CTRL_TX_EN_Msk                              (0x1UL << I2C_DMA_CTRL_TX_EN_POS)
#define I2C_DMA_CTRL_TX_EN                                  I2C_DMA_CTRL_TX_EN_Msk

/*******************  Bit definition for I2C_DMA_TX_LEVEL register  *******************/
#define I2C_DMA_TX_LEVEL_LEVEL_POS                          (0U)
#define I2C_DMA_TX_LEVEL_LEVEL_Len                          (5U)
#define I2C_DMA_TX_LEVEL_LEVEL_Msk                          (0x1FUL << I2C_DMA_TX_LEVEL_LEVEL_POS)
#define I2C_DMA_TX_LEVEL_LEVEL                              I2C_DMA_TX_LEVEL_LEVEL_Msk

/*******************  Bit definition for I2C_DMA_RX_LEVEL register  *******************/
#define I2C_DMA_RX_LEVEL_LEVEL_POS                          (0U)
#define I2C_DMA_RX_LEVEL_LEVEL_Len                          (5U)
#define I2C_DMA_RX_LEVEL_LEVEL_Msk                          (0x1FUL << I2C_DMA_RX_LEVEL_LEVEL_POS)
#define I2C_DMA_RX_LEVEL_LEVEL                              I2C_DMA_RX_LEVEL_LEVEL_Msk

/*******************  Bit definition for I2C_SDA_SETUP register  *******************/
#define I2C_SDA_SETUP_SETUP_POS                             (0U)
#define I2C_SDA_SETUP_SETUP_Len                             (8U)
#define I2C_SDA_SETUP_SETUP_Msk                             (0xFFUL << I2C_SDA_SETUP_SETUP_POS)
#define I2C_SDA_SETUP_SETUP                                 I2C_SDA_SETUP_SETUP_Msk

/*******************  Bit definition for I2C_ACK_GEN_CALL register  *******************/
#define I2C_ACK_GEN_CALL_ACK_GEN_CALL_POS                   (0U)
#define I2C_ACK_GEN_CALL_ACK_GEN_CALL_Len                   (1U)
#define I2C_ACK_GEN_CALL_ACK_GEN_CALL_Msk                   (0x1UL << I2C_ACK_GEN_CALL_ACK_GEN_CALL_POS)
#define I2C_ACK_GEN_CALL_ACK_GEN_CALL                       I2C_ACK_GEN_CALL_ACK_GEN_CALL_Msk

/*******************  Bit definition for I2C_EN_STAT register  *******************/
#define I2C_EN_STAT_EN_POS                                  (0U)
#define I2C_EN_STAT_EN_Len                                  (1U)
#define I2C_EN_STAT_EN_Msk                                  (0x1UL << I2C_EN_STAT_EN_POS)
#define I2C_EN_STAT_EN                                      I2C_EN_STAT_EN_Msk

#define I2C_EN_STAT_S_DIS_BUSY_POS                          (1U)
#define I2C_EN_STAT_S_DIS_BUSY_Len                          (1U)
#define I2C_EN_STAT_S_DIS_BUSY_Msk                          (0x1UL << I2C_EN_STAT_S_DIS_BUSY_POS)
#define I2C_EN_STAT_S_DIS_BUSY                              I2C_EN_STAT_S_DIS_BUSY_Msk

#define I2C_EN_STAT_S_RX_DATA_LOST_POS                      (2U)
#define I2C_EN_STAT_S_RX_DATA_LOST_Len                      (1U)
#define I2C_EN_STAT_S_RX_DATA_LOST_Msk                      (0x1UL << I2C_EN_STAT_S_RX_DATA_LOST_POS)
#define I2C_EN_STAT_S_RX_DATA_LOST                          I2C_EN_STAT_S_RX_DATA_LOST_Msk

/*******************  Bit definition for I2C_FS_SPKLEN register  *******************/
#define I2C_FS_SPKLEN_FS_SPKLEN_POS                         (0U)
#define I2C_FS_SPKLEN_FS_SPKLEN_Len                         (8U)
#define I2C_FS_SPKLEN_FS_SPKLEN_Msk                         (0xFFUL << I2C_FS_SPKLEN_FS_SPKLEN_POS)
#define I2C_FS_SPKLEN_FS_SPKLEN                             I2C_FS_SPKLEN_FS_SPKLEN_Msk

/*******************  Bit definition for I2C_HS_SPKLEN register  *******************/
#define I2C_HS_SPKLEN_HS_SPKLEN_POS                         (0U)
#define I2C_HS_SPKLEN_HS_SPKLEN_Len                         (8U)
#define I2C_HS_SPKLEN_HS_SPKLEN_Msk                         (0xFFUL << I2C_HS_SPKLEN_HS_SPKLEN_POS)
#define I2C_HS_SPKLEN_HS_SPKLEN                             I2C_HS_SPKLEN_HS_SPKLEN_Msk

/* ================================================================================================================= */
/* ================                                        I2S                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for I2S_EN register  *******************/
#define I2S_EN_I2S_EN_POS                                   (0U)
#define I2S_EN_I2S_EN_Len                                   (1U)
#define I2S_EN_I2S_EN_Msk                                   (0x1UL << I2S_EN_I2S_EN_POS)
#define I2S_EN_I2S_EN                                       I2S_EN_I2S_EN_Msk

/*******************  Bit definition for I2S_RX_EN register  *******************/
#define I2S_RX_EN_RX_EN_POS                                 (0U)
#define I2S_RX_EN_RX_EN_Len                                 (1U)
#define I2S_RX_EN_RX_EN_Msk                                 (0x1UL << I2S_RX_EN_RX_EN_POS)
#define I2S_RX_EN_RX_EN                                     I2S_RX_EN_RX_EN_Msk

/*******************  Bit definition for I2S_TX_EN register  *******************/
#define I2S_TX_EN_TX_EN_POS                                 (0U)
#define I2S_TX_EN_TX_EN_Len                                 (1U)
#define I2S_TX_EN_TX_EN_Msk                                 (0x1UL << I2S_TX_EN_TX_EN_POS)
#define I2S_TX_EN_TX_EN                                     I2S_TX_EN_TX_EN_Msk

/*******************  Bit definition for I2S_CLK_EN register  *******************/
#define I2S_CLK_EN_CLK_EN_POS                               (0U)
#define I2S_CLK_EN_CLK_EN_Len                               (1U)
#define I2S_CLK_EN_CLK_EN_Msk                               (0x1UL << I2S_CLK_EN_CLK_EN_POS)
#define I2S_CLK_EN_CLK_EN                                   I2S_CLK_EN_CLK_EN_Msk

/*******************  Bit definition for I2S_SCLK_CFG register  *******************/
#define I2S_SCLK_CFG_SCLK_GAT_POS                           (0U)
#define I2S_SCLK_CFG_SCLK_GAT_Len                           (3U)
#define I2S_SCLK_CFG_SCLK_GAT_Msk                           (0x7UL << I2S_SCLK_CFG_SCLK_GAT_POS)
#define I2S_SCLK_CFG_SCLK_GAT                               I2S_SCLK_CFG_SCLK_GAT_Msk

#define I2S_SCLK_CFG_WS_SCLK_POS                            (3U)
#define I2S_SCLK_CFG_WS_SCLK_Len                            (2U)
#define I2S_SCLK_CFG_WS_SCLK_Msk                            (0x3UL << I2S_SCLK_CFG_WS_SCLK_POS)
#define I2S_SCLK_CFG_WS_SCLK                                I2S_SCLK_CFG_WS_SCLK_Msk

/*******************  Bit definition for I2S_RX_FIFO_RST register  *******************/
#define I2S_RX_FIFO_RST_RX_FIFO_RST_POS                     (0U)
#define I2S_RX_FIFO_RST_RX_FIFO_RST_Len                     (1U)
#define I2S_RX_FIFO_RST_RX_FIFO_RST_Msk                     (0x1UL << I2S_RX_FIFO_RST_RX_FIFO_RST_POS)
#define I2S_RX_FIFO_RST_RX_FIFO_RST                         I2S_RX_FIFO_RST_RX_FIFO_RST_Msk

/*******************  Bit definition for I2S_TX_FIFO_RST register  *******************/
#define I2S_TX_FIFO_RST_TX_FIFO_RST_POS                     (0U)
#define I2S_TX_FIFO_RST_TX_FIFO_RST_Len                     (1U)
#define I2S_TX_FIFO_RST_TX_FIFO_RST_Msk                     (0x1UL << I2S_TX_FIFO_RST_TX_FIFO_RST_POS)
#define I2S_TX_FIFO_RST_TX_FIFO_RST                         I2S_TX_FIFO_RST_TX_FIFO_RST_Msk

/*******************  Bit definition for I2S_LEFT_RX_BUF register  *******************/
#define I2S_LEFT_BUF_LEFT_BUF_POS                            (0U)
#define I2S_LEFT_BUF_LEFT_BUF_Len                            (32U)
#define I2S_LEFT_BUF_LEFT_BUF_Msk                            (0xFFFFFFFFUL << I2S_LEFT_BUF_LEFT_BUF_POS)
#define I2S_LEFT_BUF_LEFT_BUF                                 I2S_LEFT_BUF_LEFT_BUF_Msk

/*******************  Bit definition for I2S_RIGHT_RX_BUF register  *******************/
#define I2S_RIGHT_BUF_RIGHT_BUF_POS                          (0U)
#define I2S_RIGHT_BUF_RIGHT_BUF_Len                          (32U)
#define I2S_RIGHT_BUF_RIGHT_BUF_Msk                          (0xFFFFFFFFUL << I2S_RIGHT_BUF_RIGHT_BUF_POS)
#define I2S_RIGHT_BUF_RIGHT_BUF                               I2S_RIGHT_BUF_RIGHT_BUF_Msk

/*******************  Bit definition for I2S_RX_CH_EN register  *******************/
#define I2S_RX_CH_EN_RX_CH_EN_POS                           (0U)
#define I2S_RX_CH_EN_RX_CH_EN_Len                           (1U)
#define I2S_RX_CH_EN_RX_CH_EN_Msk                           (0x1UL << I2S_RX_CH_EN_RX_CH_EN_POS)
#define I2S_RX_CH_EN_RX_CH_EN                               I2S_RX_CH_EN_RX_CH_EN_Msk

/*******************  Bit definition for I2S_TX_CH_EN register  *******************/
#define I2S_TX_CH_EN_TX_CH_EN_POS                           (0U)
#define I2S_TX_CH_EN_TX_CH_EN_Len                           (1U)
#define I2S_TX_CH_EN_TX_CH_EN_Msk                           (0x1UL << I2S_TX_CH_EN_TX_CH_EN_POS)
#define I2S_TX_CH_EN_TX_CH_EN                               I2S_TX_CH_EN_TX_CH_EN_Msk

/*******************  Bit definition for I2S_RX_CFG register  *******************/
#define I2S_RX_CFG_WORD_LEN_POS                             (0U)
#define I2S_RX_CFG_WORD_LEN_Len                             (3U)
#define I2S_RX_CFG_WORD_LEN_Msk                             (0x7UL << I2S_RX_CFG_WORD_LEN_POS)
#define I2S_RX_CFG_WORD_LEN                                 I2S_RX_CFG_WORD_LEN_Msk

/*******************  Bit definition for I2S_TX_CFG register  *******************/
#define I2S_TX_CFG_WORD_LEN_POS                             (0U)
#define I2S_TX_CFG_WORD_LEN_Len                             (3U)
#define I2S_TX_CFG_WORD_LEN_Msk                             (0x7UL << I2S_TX_CFG_WORD_LEN_POS)
#define I2S_TX_CFG_WORD_LEN                                 I2S_TX_CFG_WORD_LEN_Msk

/*******************  Bit definition for I2S_INT_STAT register  *******************/
#define I2S_INT_STAT_RX_DATA_AVL_POS                        (0U)
#define I2S_INT_STAT_RX_DATA_AVL_Len                        (1U)
#define I2S_INT_STAT_RX_DATA_AVL_Msk                        (0x1UL << I2S_INT_STAT_RX_DATA_AVL_POS)
#define I2S_INT_STAT_RX_DATA_AVL                            I2S_INT_STAT_RX_DATA_AVL_Msk

#define I2S_INT_STAT_RX_FIFO_OVER_POS                       (1U)
#define I2S_INT_STAT_RX_FIFO_OVER_Len                       (1U)
#define I2S_INT_STAT_RX_FIFO_OVER_Msk                       (0x1UL << I2S_INT_STAT_RX_FIFO_OVER_POS)
#define I2S_INT_STAT_RX_FIFO_OVER                           I2S_INT_STAT_RX_FIFO_OVER_Msk

#define I2S_INT_STAT_TX_FIFO_EMPTY_POS                      (4U)
#define I2S_INT_STAT_TX_FIFO_EMPTY_Len                      (1U)
#define I2S_INT_STAT_TX_FIFO_EMPTY_Msk                      (0x1UL << I2S_INT_STAT_TX_FIFO_EMPTY_POS)
#define I2S_INT_STAT_TX_FIFO_EMPTY                          I2S_INT_STAT_TX_FIFO_EMPTY_Msk

#define I2S_INT_STAT_TX_FIFO_OVER_POS                       (5U)
#define I2S_INT_STAT_TX_FIFO_OVER_Len                       (1U)
#define I2S_INT_STAT_TX_FIFO_OVER_Msk                       (0x1UL << I2S_INT_STAT_TX_FIFO_OVER_POS)
#define I2S_INT_STAT_TX_FIFO_OVER                           I2S_INT_STAT_TX_FIFO_OVER_Msk

/*******************  Bit definition for I2S_INT_MASK register  *******************/
#define I2S_INT_MASK_RX_DAM_POS                             (0U)
#define I2S_INT_MASK_RX_DAM_Len                             (1U)
#define I2S_INT_MASK_RX_DAM_Msk                             (0x1UL << I2S_INT_MASK_RX_DAM_POS)
#define I2S_INT_MASK_RX_DAM                                 I2S_INT_MASK_RX_DAM_Msk

#define I2S_INT_MASK_RX_FOM_POS                             (1U)
#define I2S_INT_MASK_RX_FOM_Len                             (1U)
#define I2S_INT_MASK_RX_FOM_Msk                             (0x1UL << I2S_INT_MASK_RX_FOM_POS)
#define I2S_INT_MASK_RX_FOM                                 I2S_INT_MASK_RX_FOM_Msk

#define I2S_INT_MASK_TX_FEM_POS                             (4U)
#define I2S_INT_MASK_TX_FEM_Len                             (1U)
#define I2S_INT_MASK_TX_FEM_Msk                             (0x1UL << I2S_INT_MASK_TX_FEM_POS)
#define I2S_INT_MASK_TX_FEM                                 I2S_INT_MASK_TX_FEM_Msk

#define I2S_INT_MASK_TX_FOM_POS                             (5U)
#define I2S_INT_MASK_TX_FOM_Len                             (1U)
#define I2S_INT_MASK_TX_FOM_Msk                             (0x1UL << I2S_INT_MASK_TX_FOM_POS)
#define I2S_INT_MASK_TX_FOM                                 I2S_INT_MASK_TX_FOM_Msk

/*******************  Bit definition for I2S_RX_OVER register  *******************/
#define I2S_RX_OVER_RX_CLR_FDO_POS                          (0U)
#define I2S_RX_OVER_RX_CLR_FDO_Len                          (1U)
#define I2S_RX_OVER_RX_CLR_FDO_Msk                          (0x1UL << I2S_RX_OVER_RX_CLR_FDO_POS)
#define I2S_RX_OVER_RX_CLR_FDO                              I2S_RX_OVER_RX_CLR_FDO_Msk

/*******************  Bit definition for I2S_TX_OVER register  *******************/
#define I2S_TX_OVER_TX_CLR_FDO_POS                          (0U)
#define I2S_TX_OVER_TX_CLR_FDO_Len                          (1U)
#define I2S_TX_OVER_TX_CLR_FDO_Msk                          (0x1UL << I2S_TX_OVER_TX_CLR_FDO_POS)
#define I2S_TX_OVER_TX_CLR_FDO                              I2S_TX_OVER_TX_CLR_FDO_Msk

/*******************  Bit definition for I2S_RX_FIFO_CFG register  *******************/
#define I2S_RX_FIFO_CFG_RX_FIFO_TL_POS                      (0U)
#define I2S_RX_FIFO_CFG_RX_FIFO_TL_Len                      (4U)
#define I2S_RX_FIFO_CFG_RX_FIFO_TL_Msk                      (0xFUL << I2S_RX_FIFO_CFG_RX_FIFO_TL_POS)
#define I2S_RX_FIFO_CFG_RX_FIFO_TL                          I2S_RX_FIFO_CFG_RX_FIFO_TL_Msk

/*******************  Bit definition for I2S_TX_FIFO_CFG register  *******************/
#define I2S_TX_FIFO_CFG_TX_FIFO_TL_POS                      (0U)
#define I2S_TX_FIFO_CFG_TX_FIFO_TL_Len                      (4U)
#define I2S_TX_FIFO_CFG_TX_FIFO_TL_Msk                      (0xFUL << I2S_TX_FIFO_CFG_TX_FIFO_TL_POS)
#define I2S_TX_FIFO_CFG_TX_FIFO_TL                          I2S_TX_FIFO_CFG_TX_FIFO_TL_Msk

/*******************  Bit definition for I2S_RX_FIFO_FLUSH register  *******************/
#define I2S_RX_FIFO_FLUSH_RX_FIFO_RST_POS                   (0U)
#define I2S_RX_FIFO_FLUSH_RX_FIFO_RST_Len                   (1U)
#define I2S_RX_FIFO_FLUSH_RX_FIFO_RST_Msk                   (0x1UL << I2S_RX_FIFO_FLUSH_RX_FIFO_RST_POS)
#define I2S_RX_FIFO_FLUSH_RX_FIFO_RST                       I2S_RX_FIFO_FLUSH_RX_FIFO_RST_Msk

/*******************  Bit definition for I2S_TX_FIFO_FLUSH register  *******************/
#define I2S_TX_FIFO_FLUSH_TX_FIFO_RST_POS                   (0U)
#define I2S_TX_FIFO_FLUSH_TX_FIFO_RST_Len                   (1U)
#define I2S_TX_FIFO_FLUSH_TX_FIFO_RST_Msk                   (0x1UL << I2S_TX_FIFO_FLUSH_TX_FIFO_RST_POS)
#define I2S_TX_FIFO_FLUSH_TX_FIFO_RST                       I2S_TX_FIFO_FLUSH_TX_FIFO_RST_Msk

/*******************  Bit definition for I2S_RX_DMA register  *******************/
#define I2S_RX_DMA_RX_DMA_POS                               (0U)
#define I2S_RX_DMA_RX_DMA_Len                               (1U)
#define I2S_RX_DMA_RX_DMA_Msk                               (0x1UL << I2S_RX_DMA_RX_DMA_POS)
#define I2S_RX_DMA_RX_DMA                                   I2S_RX_DMA_RX_DMA_Msk

/*******************  Bit definition for I2S_RST_RX_DMA register  *******************/
#define I2S_RST_RX_DMA_RST_RX_DMA_POS                       (0U)
#define I2S_RST_RX_DMA_RST_RX_DMA_Len                       (1U)
#define I2S_RST_RX_DMA_RST_RX_DMA_Msk                       (0x1UL << I2S_RST_RX_DMA_RST_RX_DMA_POS)
#define I2S_RST_RX_DMA_RST_RX_DMA                           I2S_RST_RX_DMA_RST_RX_DMA_Msk

/*******************  Bit definition for I2S_TX_DMA register  *******************/
#define I2S_TX_DMA_TX_DMA_POS                               (0U)
#define I2S_TX_DMA_TX_DMA_Len                               (1U)
#define I2S_TX_DMA_TX_DMA_Msk                               (0x1UL << I2S_TX_DMA_TX_DMA_POS)
#define I2S_TX_DMA_TX_DMA                                   I2S_TX_DMA_TX_DMA_Msk

/*******************  Bit definition for I2S_RST_TX_DMA register  *******************/
#define I2S_RST_TX_DMA_RST_TX_DMA_POS                       (0U)
#define I2S_RST_TX_DMA_RST_TX_DMA_Len                       (1U)
#define I2S_RST_TX_DMA_RST_TX_DMA_Msk                       (0x1UL << I2S_RST_TX_DMA_RST_TX_DMA_POS)
#define I2S_RST_TX_DMA_RST_TX_DMA                           I2S_RST_TX_DMA_RST_TX_DMA_Msk


/* ================================================================================================================= */
/* ================                                        ISO7816                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for ISO7816_CTRL register  *******************/
#define ISO7816_CTRL_ACTION_POS                             (0U)
#define ISO7816_CTRL_ACTION_Len                             (3U)
#define ISO7816_CTRL_ACTION_Msk                             (0x7UL << ISO7816_CTRL_ACTION_POS)
#define ISO7816_CTRL_ACTION                                 ISO7816_CTRL_ACTION_Msk

#define ISO7816_CTRL_RX_RETYR_MC_POS                        (8U)
#define ISO7816_CTRL_RX_RETYR_MC_Len                        (1U)
#define ISO7816_CTRL_RX_RETYR_MC_Msk                        (0x1UL << ISO7816_CTRL_RX_RETYR_MC_POS)
#define ISO7816_CTRL_RX_RETYR_MC                            ISO7816_CTRL_RX_RETYR_MC_Msk

#define ISO7816_CTRL_TX_RETYR_MC_POS                        (12U)
#define ISO7816_CTRL_TX_RETYR_MC_Len                        (1U)
#define ISO7816_CTRL_TX_RETYR_MC_Msk                        (0x1UL << ISO7816_CTRL_TX_RETYR_MC_POS)
#define ISO7816_CTRL_TX_RETYR_MC                            ISO7816_CTRL_TX_RETYR_MC_Msk

#define ISO7816_CTRL_IRQ_DONE_CLR_POS                       (20U)
#define ISO7816_CTRL_IRQ_DONE_CLR_Len                       (1U)
#define ISO7816_CTRL_IRQ_DONE_CLR_Msk                       (0x1UL << ISO7816_CTRL_IRQ_DONE_CLR_POS)
#define ISO7816_CTRL_IRQ_DONE_CLR                           ISO7816_CTRL_IRQ_DONE_CLR_Msk

#define ISO7816_CTRL_IRQ_RX_EC_POS                          (21U)
#define ISO7816_CTRL_IRQ_RX_EC_Len                          (1U)
#define ISO7816_CTRL_IRQ_RX_EC_Msk                          (0x1UL << ISO7816_CTRL_IRQ_RX_EC_POS)
#define ISO7816_CTRL_IRQ_RX_EC                              ISO7816_CTRL_IRQ_RX_EC_Msk

#define ISO7816_CTRL_IRQ_RETYR_EC_POS                       (22U)
#define ISO7816_CTRL_IRQ_RETYR_EC_Len                       (1U)
#define ISO7816_CTRL_IRQ_RETYR_EC_Msk                       (0x1UL << ISO7816_CTRL_IRQ_RETYR_EC_POS)
#define ISO7816_CTRL_IRQ_RETYR_EC                           ISO7816_CTRL_IRQ_RETYR_EC_Msk

#define ISO7816_CTRL_IRQ_DMA_EC_POS                         (23U)
#define ISO7816_CTRL_IRQ_DMA_EC_Len                         (1U)
#define ISO7816_CTRL_IRQ_DMA_EC_Msk                         (0x1UL << ISO7816_CTRL_IRQ_DMA_EC_POS)
#define ISO7816_CTRL_IRQ_DMA_EC                             ISO7816_CTRL_IRQ_DMA_EC_Msk

#define ISO7816_CTRL_IRQ_STAT_EC_POS                        (24U)
#define ISO7816_CTRL_IRQ_STAT_EC_Len                        (1U)
#define ISO7816_CTRL_IRQ_STAT_EC_Msk                        (0x1UL << ISO7816_CTRL_IRQ_STAT_EC_POS)
#define ISO7816_CTRL_IRQ_STAT_EC                            ISO7816_CTRL_IRQ_STAT_EC_Msk

#define ISO7816_CTRL_IRQ_PRESENCE_CLR_POS                   (25U)
#define ISO7816_CTRL_IRQ_PRESENCE_CLR_Len                   (1U)
#define ISO7816_CTRL_IRQ_PRESENCE_CLR_Msk                   (0x1UL << ISO7816_CTRL_IRQ_PRESENCE_CLR_POS)
#define ISO7816_CTRL_IRQ_PRESENCE_CLR                       ISO7816_CTRL_IRQ_PRESENCE_CLR_Msk

#define ISO7816_CTRL_IRQ_TEST_CLR_POS                       (30U)
#define ISO7816_CTRL_IRQ_TEST_CLR_Len                       (1U)
#define ISO7816_CTRL_IRQ_TEST_CLR_Msk                       (0x1UL << ISO7816_CTRL_IRQ_TEST_CLR_POS)
#define ISO7816_CTRL_IRQ_TEST_CLR                           ISO7816_CTRL_IRQ_TEST_CLR_Msk

#define ISO7816_CTRL_IRQ_TEST_SET_POS                       (31U)
#define ISO7816_CTRL_IRQ_TEST_SET_Len                       (1U)
#define ISO7816_CTRL_IRQ_TEST_SET_Msk                       (0x1UL << ISO7816_CTRL_IRQ_TEST_SET_POS)
#define ISO7816_CTRL_IRQ_TEST_SET                           ISO7816_CTRL_IRQ_TEST_SET_Msk

/*******************  Bit definition for ISO7816_STAT register  *******************/
#define ISO7816_INTR_ALL                                       (0x43F00000)

#define ISO7816_STAT_PWR_STAT_POS                           (0U)
#define ISO7816_STAT_PWR_STAT_Len                           (4U)
#define ISO7816_STAT_PWR_STAT_Msk                           (0xFUL << ISO7816_STAT_PWR_STAT_POS)
#define ISO7816_STAT_PWR_STAT                               ISO7816_STAT_PWR_STAT_Msk

#define ISO7816_STAT_IO_STAT_POS                            (4U)
#define ISO7816_STAT_IO_STAT_Len                            (3U)
#define ISO7816_STAT_IO_STAT_Msk                            (0x7UL << ISO7816_STAT_IO_STAT_POS)
#define ISO7816_STAT_IO_STAT                                ISO7816_STAT_IO_STAT_Msk

#define ISO7816_STAT_RX_RETRY_MAX_POS                       (8U)
#define ISO7816_STAT_RX_RETRY_MAX_Len                       (3U)
#define ISO7816_STAT_RX_RETRY_MAX_Msk                       (0x7UL << ISO7816_STAT_RX_RETRY_MAX_POS)
#define ISO7816_STAT_RX_RETRY_MAX                           ISO7816_STAT_RX_RETRY_MAX_Msk

#define ISO7816_STAT_TX_RETRY_MAX_POS                       (12U)
#define ISO7816_STAT_TX_RETRY_MAX_Len                       (3U)
#define ISO7816_STAT_TX_RETRY_MAX_Msk                       (0x7UL << ISO7816_STAT_TX_RETRY_MAX_POS)
#define ISO7816_STAT_TX_RETRY_MAX                           ISO7816_STAT_TX_RETRY_MAX_Msk

#define ISO7816_STAT_BUSY_POS                               (16U)
#define ISO7816_STAT_BUSY_Len                               (1U)
#define ISO7816_STAT_BUSY_Msk                               (0x1UL << ISO7816_STAT_BUSY_POS)
#define ISO7816_STAT_BUSY                                   ISO7816_STAT_BUSY_Msk

#define ISO7816_STAT_PRESENCE_STAT_POS                      (17U)
#define ISO7816_STAT_PRESENCE_STAT_Len                      (1U)
#define ISO7816_STAT_PRESENCE_STAT_Msk                      (0x1UL << ISO7816_STAT_PRESENCE_STAT_POS)
#define ISO7816_STAT_PRESENCE_STAT                          ISO7816_STAT_PRESENCE_STAT_Msk

#define ISO7816_STAT_IRQ_DONE_POS                           (20U)
#define ISO7816_STAT_IRQ_DONE_Len                           (1U)
#define ISO7816_STAT_IRQ_DONE_Msk                           (0x1UL << ISO7816_STAT_IRQ_DONE_POS)
#define ISO7816_STAT_IRQ_DONE                               ISO7816_STAT_IRQ_DONE_Msk

#define ISO7816_STAT_IRQ_RX_ERR_POS                         (21U)
#define ISO7816_STAT_IRQ_RX_ERR_Len                         (1U)
#define ISO7816_STAT_IRQ_RX_ERR_Msk                         (0x1UL << ISO7816_STAT_IRQ_RX_ERR_POS)
#define ISO7816_STAT_IRQ_RX_ERR                             ISO7816_STAT_IRQ_RX_ERR_Msk

#define ISO7816_STAT_IRQ_RETRY_ERR_POS                      (22U)
#define ISO7816_STAT_IRQ_RETRY_ERR_Len                      (1U)
#define ISO7816_STAT_IRQ_RETRY_ERR_Msk                      (0x1UL << ISO7816_STAT_IRQ_RETRY_ERR_POS)
#define ISO7816_STAT_IRQ_RETRY_ERR                          ISO7816_STAT_IRQ_RETRY_ERR_Msk

#define ISO7816_STAT_IRQ_DMA_ERR_POS                        (23U)
#define ISO7816_STAT_IRQ_DMA_ERR_Len                        (1U)
#define ISO7816_STAT_IRQ_DMA_ERR_Msk                        (0x1UL << ISO7816_STAT_IRQ_DMA_ERR_POS)
#define ISO7816_STAT_IRQ_DMA_ERR                            ISO7816_STAT_IRQ_DMA_ERR_Msk

#define ISO7816_STAT_IRQ_STAT_ERR_POS                       (24U)
#define ISO7816_STAT_IRQ_STAT_ERR_Len                       (1U)
#define ISO7816_STAT_IRQ_STAT_ERR_Msk                       (0x1UL << ISO7816_STAT_IRQ_STAT_ERR_POS)
#define ISO7816_STAT_IRQ_STAT_ERR                           ISO7816_STAT_IRQ_STAT_ERR_Msk

#define ISO7816_STAT_IRQ_PRESENCE_POS                       (25U)
#define ISO7816_STAT_IRQ_PRESENCE_Len                       (1U)
#define ISO7816_STAT_IRQ_PRESENCE_Msk                       (0x1UL << ISO7816_STAT_IRQ_PRESENCE_POS)
#define ISO7816_STAT_IRQ_PRESENCE                           ISO7816_STAT_IRQ_PRESENCE_Msk

#define ISO7816_STAT_IRQ_TEST_POS                           (30U)
#define ISO7816_STAT_IRQ_TEST_Len                           (1U)
#define ISO7816_STAT_IRQ_TEST_Msk                           (0x1UL << ISO7816_STAT_IRQ_TEST_POS)
#define ISO7816_STAT_IRQ_TEST                               ISO7816_STAT_IRQ_TEST_Msk

/*******************  Bit definition for ISO7816_CLK_CFG register  *******************/
#define ISO7816_CLK_CFG_ETU_DIV_POS                         (0U)
#define ISO7816_CLK_CFG_ETU_DIV_Len                         (10U)
#define ISO7816_CLK_CFG_ETU_DIV_Msk                         (0x3FFUL << ISO7816_CLK_CFG_ETU_DIV_POS)
#define ISO7816_CLK_CFG_ETU_DIV                             ISO7816_CLK_CFG_ETU_DIV_Msk

#define ISO7816_CLK_CFG_CLK_DIV_POS                         (16U)
#define ISO7816_CLK_CFG_CLK_DIV_Len                         (8U)
#define ISO7816_CLK_CFG_CLK_DIV_Msk                         (0xFFUL << ISO7816_CLK_CFG_CLK_DIV_POS)
#define ISO7816_CLK_CFG_CLK_DIV                             ISO7816_CLK_CFG_CLK_DIV_Msk

#define ISO7816_CLK_CFG_CLK_STOP_SEL_POS                    (31U)
#define ISO7816_CLK_CFG_CLK_STOP_SEL_Len                    (1U)
#define ISO7816_CLK_CFG_CLK_STOP_SEL_Msk                    (0x1UL << ISO7816_CLK_CFG_CLK_STOP_SEL_POS)
#define ISO7816_CLK_CFG_CLK_STOP_SEL                        ISO7816_CLK_CFG_CLK_STOP_SEL_Msk

/*******************  Bit definition for ISO7816_TIMES_CFG register  *******************/
#define ISO7816_TIMES_CFG_GUARD_TIME_POS                    (0U)
#define ISO7816_TIMES_CFG_GUARD_TIME_Len                    (10U)
#define ISO7816_TIMES_CFG_GUARD_TIME_Msk                    (0x3FFUL << ISO7816_TIMES_CFG_GUARD_TIME_POS)
#define ISO7816_TIMES_CFG_GUARD_TIME                        ISO7816_TIMES_CFG_GUARD_TIME_Msk

#define ISO7816_TIMES_CFG_WAIT_TIME_POS                     (12U)
#define ISO7816_TIMES_CFG_WAIT_TIME_Len                     (18U)
#define ISO7816_TIMES_CFG_WAIT_TIME_Msk                     (0x3FFFFUL << ISO7816_TIMES_CFG_WAIT_TIME_POS)
#define ISO7816_TIMES_CFG_WAIT_TIME                         ISO7816_TIMES_CFG_WAIT_TIME_Msk

/*******************  Bit definition for ISO7816_DATA_CFG register  *******************/
#define ISO7816_DATA_CFG_CODING_POS                         (0U)
#define ISO7816_DATA_CFG_CODING_Len                         (1U)
#define ISO7816_DATA_CFG_CODING_Msk                         (0x1UL << ISO7816_DATA_CFG_CODING_POS)
#define ISO7816_DATA_CFG_CODING                             ISO7816_DATA_CFG_CODING_Msk

#define ISO7816_DATA_CFG_DETECT_CODING_POS                  (1U)
#define ISO7816_DATA_CFG_DETECT_CODING_Len                  (1U)
#define ISO7816_DATA_CFG_DETECT_CODING_Msk                  (0x1UL << ISO7816_DATA_CFG_DETECT_CODING_POS)
#define ISO7816_DATA_CFG_DETECT_CODING                      ISO7816_DATA_CFG_DETECT_CODING_Msk

#define ISO7816_DATA_CFG_RETRY_LIMIT_POS                    (4U)
#define ISO7816_DATA_CFG_RETRY_LIMIT_Len                    (3U)
#define ISO7816_DATA_CFG_RETRY_LIMIT_Msk                    (0x7UL << ISO7816_DATA_CFG_RETRY_LIMIT_POS)
#define ISO7816_DATA_CFG_RETRY_LIMIT                        ISO7816_DATA_CFG_RETRY_LIMIT_Msk

/*******************  Bit definition for ISO7816_ADDR register  *******************/
#define ISO7816_ADDR_ADDR_FRAC_POS                          (0U)
#define ISO7816_ADDR_ADDR_FRAC_Len                          (2U)
#define ISO7816_ADDR_ADDR_FRAC_Msk                          (0x3UL << ISO7816_ADDR_ADDR_FRAC_POS)
#define ISO7816_ADDR_ADDR_FRAC                              ISO7816_ADDR_ADDR_FRAC_Msk

#define ISO7816_ADDR_ADDR_POS                               (2U)
#define ISO7816_ADDR_ADDR_Len                               (18U)
#define ISO7816_ADDR_ADDR_Msk                               (0x3FFFFUL << ISO7816_ADDR_ADDR_POS)
#define ISO7816_ADDR_ADDR                                   ISO7816_ADDR_ADDR_Msk

/*******************  Bit definition for ISO7816_START_ADDR register  *******************/
#define ISO7816_START_ADDR_START_ADDR_POS                   (2U)
#define ISO7816_START_ADDR_START_ADDR_Len                   (18U)
#define ISO7816_START_ADDR_START_ADDR_Msk                   (0x3FFFFUL << ISO7816_START_ADDR_START_ADDR_POS)
#define ISO7816_START_ADDR_START_ADDR                       ISO7816_START_ADDR_START_ADDR_Msk

#define ISO7816_START_ADDR_BASE_ADDR_POS                    (20U)
#define ISO7816_START_ADDR_BASE_ADDR_Len                    (12U)
#define ISO7816_START_ADDR_BASE_ADDR_Msk                    (0xFFFUL << ISO7816_START_ADDR_BASE_ADDR_POS)
#define ISO7816_START_ADDR_BASE_ADDR                        ISO7816_START_ADDR_BASE_ADDR_Msk

/*******************  Bit definition for ISO7816_RX_END_ADDR register  *******************/
#define ISO7816_RX_END_ADDR_RX_END_AF_POS                   (0U)
#define ISO7816_RX_END_ADDR_RX_END_AF_Len                   (2U)
#define ISO7816_RX_END_ADDR_RX_END_AF_Msk                   (0x3UL << ISO7816_RX_END_ADDR_RX_END_AF_POS)
#define ISO7816_RX_END_ADDR_RX_END_AF                       ISO7816_RX_END_ADDR_RX_END_AF_Msk

#define ISO7816_RX_END_ADDR_RX_END_ADDR_POS                 (2U)
#define ISO7816_RX_END_ADDR_RX_END_ADDR_Len                 (18U)
#define ISO7816_RX_END_ADDR_RX_END_ADDR_Msk                 (0x3FFFFUL << ISO7816_RX_END_ADDR_RX_END_ADDR_POS)
#define ISO7816_RX_END_ADDR_RX_END_ADDR                     ISO7816_RX_END_ADDR_RX_END_ADDR_Msk

/*******************  Bit definition for ISO7816_TX_END_ADDR register  *******************/
#define ISO7816_TX_END_ADDR_TX_END_AF_POS                   (0U)
#define ISO7816_TX_END_ADDR_TX_END_AF_Len                   (2U)
#define ISO7816_TX_END_ADDR_TX_END_AF_Msk                   (0x3UL << ISO7816_TX_END_ADDR_TX_END_AF_POS)
#define ISO7816_TX_END_ADDR_TX_END_AF                       ISO7816_TX_END_ADDR_TX_END_AF_Msk

#define ISO7816_TX_END_ADDR_TX_END_ADDR_POS                 (2U)
#define ISO7816_TX_END_ADDR_TX_END_ADDR_Len                 (18U)
#define ISO7816_TX_END_ADDR_TX_END_ADDR_Msk                 (0x3FFFFUL << ISO7816_TX_END_ADDR_TX_END_ADDR_POS)
#define ISO7816_TX_END_ADDR_TX_END_ADDR                     ISO7816_TX_END_ADDR_TX_END_ADDR_Msk

/* ================================================================================================================= */
/* ================                                      MCU_SUB                                    ================ */
/* ================================================================================================================= */
/*******************  Bit definition for SENSE_ADC_FIFO register  ********************/
#define MCU_SUB_SNSADC_FF_DATA_Pos                          (0U)
#define MCU_SUB_SNSADC_FF_DATA_Len                          (32U)
#define MCU_SUB_SNSADC_FF_DATA_Msk                          (0xFFFFFFFFUL)
#define MCU_SUB_SNSADC_FF_DATA                              MCU_SUB_SNSADC_FF_DATA_Msk

/*******************  Bit definition for SENSE_FF_THRESH register  ********************/
#define MCU_SUB_SNSADC_FF_THRESH_Pos                        (0U)
#define MCU_SUB_SNSADC_FF_THRESH_Len                        (6U)
#define MCU_SUB_SNSADC_FF_THRESH_Msk                        (0x3FUL << MCU_SUB_SNSADC_FF_THRESH_Pos)
#define MCU_SUB_SNSADC_FF_THRESH                            MCU_SUB_SNSADC_FF_THRESH_Msk

#define MCU_SUB_SNSADC_FF_DMA_EN_Pos                        (16U)
#define MCU_SUB_SNSADC_FF_DMA_EN_Len                        (1U)
#define MCU_SUB_SNSADC_FF_DMA_EN_Msk                        (0x1UL << MCU_SUB_SNSADC_FF_DMA_EN_Pos)
#define MCU_SUB_SNSADC_FF_DMA_EN                            MCU_SUB_SNSADC_FF_DMA_EN_Msk

/*******************  Bit definition for SENSE_ADC_STAT register  *****/
#define MCU_SUB_SNSADC_STAT_FLUSH_Pos                       (16U)
#define MCU_SUB_SNSADC_STAT_FLUSH_Len                       (1U)
#define MCU_SUB_SNSADC_STAT_FLUSH_Msk                       (0x1UL << MCU_SUB_SNSADC_STAT_FLUSH_Pos)
#define MCU_SUB_SNSADC_STAT_FLUSH                            MCU_SUB_SNSADC_STAT_FLUSH_Msk

#define MCU_SUB_SNSADC_STAT_VAL_Pos                         (8U)
#define MCU_SUB_SNSADC_STAT_VAL_Len                         (1U)
#define MCU_SUB_SNSADC_STAT_VAL_Msk                         (0x1UL << MCU_SUB_SNSADC_STAT_VAL_Pos)
#define MCU_SUB_SNSADC_STAT_VAL                             MCU_SUB_SNSADC_STAT_VAL_Msk

#define MCU_SUB_SNSADC_STAT_FF_COUNT_Pos                    (0U)
#define MCU_SUB_SNSADC_STAT_FF_COUNT_Len                    (7U)
#define MCU_SUB_SNSADC_STAT_FF_COUNT_Msk                    (0x7FUL << MCU_SUB_SNSADC_STAT_FF_COUNT_Pos)
#define MCU_SUB_SNSADC_STAT_FF_COUNT                        MCU_SUB_SNSADC_STAT_FF_COUNT_Msk

/*******************  Bit definition for SENSE_ADC_CLK register  *****/
#define MCU_SUB_SNSADC_CLK_RD_Pos                           (16U)
#define MCU_SUB_SNSADC_CLK_RD_Len                           (3U)
#define MCU_SUB_SNSADC_CLK_RD_Msk                           (0x7UL << MCU_SUB_SNSADC_CLK_RD_Pos)
#define MCU_SUB_SNSADC_CLK_RD                               MCU_SUB_SNSADC_CLK_RD_Msk

#define MCU_SUB_SNSADC_CLK_WR_Pos                           (0U)
#define MCU_SUB_SNSADC_CLK_WR_Len                           (3U)
#define MCU_SUB_SNSADC_CLK_WR_Msk                           (0x7UL << MCU_SUB_SNSADC_CLK_WR_Pos)
#define MCU_SUB_SNSADC_CLK_WR                               MCU_SUB_SNSADC_CLK_WR_Msk
#define MCU_SUB_SNSADC_CLK_NONE                             (0x0UL << MCU_SUB_SNSADC_CLK_WR_Pos)
#define MCU_SUB_SNSADC_CLK_16K                              (0x1UL << MCU_SUB_SNSADC_CLK_WR_Pos)
#define MCU_SUB_SNSADC_CLK_8K                               (0x2UL << MCU_SUB_SNSADC_CLK_WR_Pos)
#define MCU_SUB_SNSADC_CLK_4K                               (0x3UL << MCU_SUB_SNSADC_CLK_WR_Pos)
#define MCU_SUB_SNSADC_CLK_16M                              (0x4UL << MCU_SUB_SNSADC_CLK_WR_Pos)
#define MCU_SUB_SNSADC_CLK_8M                               (0x5UL << MCU_SUB_SNSADC_CLK_WR_Pos)
#define MCU_SUB_SNSADC_CLK_4M                               (0x6UL << MCU_SUB_SNSADC_CLK_WR_Pos)
#define MCU_SUB_SNSADC_CLK_1M                               (0x7UL << MCU_SUB_SNSADC_CLK_WR_Pos)

/*******************  Bit definition for SADC_GET_TKN_HW register  *******************/
#define MCU_SUB_SNSADC_GET_TKN_HW_LOCKED_POS                          (0U)
#define MCU_SUB_SNSADC_GET_TKN_HW_LOCKED_Len                          (1U)
#define MCU_SUB_SNSADC_GET_TKN_HW_LOCKED_Msk                          ((0x1UL) << MCU_SUB_SNSADC_GET_TKN_HW_LOCKED_POS)
#define MCU_SUB_SNSADC_GET_TKN_HW_LOCKED                              MCU_SUB_SNSADC_GET_TKN_HW_LOCKED_Msk

#define MCU_SUB_SNSADC_GET_TKN_HW_OWNER_POS                          (8U)
#define MCU_SUB_SNSADC_GET_TKN_HW_OWNER_Len                          (1U)
#define MCU_SUB_SNSADC_GET_TKN_HW_OWNER_Msk                          ((0x1UL) << MCU_SUB_SNSADC_GET_TKN_HW_OWNER_POS)
#define MCU_SUB_SNSADC_GET_TKN_HW_OWNER                              MCU_SUB_SNSADC_GET_TKN_HW_OWNER_Msk

/*******************  Bit definition for SADC_GET_TKN_SW register  *******************/
#define MCU_SUB_SNSADC_GET_TKN_SW_LOCKED_POS                          (0U)
#define MCU_SUB_SNSADC_GET_TKN_SW_LOCKED_Len                          (1U)
#define MCU_SUB_SNSADC_GET_TKN_SW_LOCKED_Msk                          ((0x1UL) << MCU_SUB_SNSADC_GET_TKN_SW_LOCKED_POS)
#define MCU_SUB_SNSADC_GET_TKN_SW_LOCKED                              MCU_SUB_SNSADC_GET_TKN_SW_LOCKED_Msk

#define MCU_SUB_SNSADC_GET_TKN_SW_OWNER_POS                           (8U)
#define MCU_SUB_SNSADC_GET_TKN_SW_OWNER_Len                           (1U)
#define MCU_SUB_SNSADC_GET_TKN_SW_OWNER_Msk                           ((0x1UL) << MCU_SUB_SNSADC_GET_TKN_SW_OWNER_POS)
#define MCU_SUB_SNSADC_GET_TKN_SW_OWNER                               MCU_SUB_SNSADC_GET_TKN_SW_OWNER_Msk

#define MCU_SUB_SNSADC_TKN_LOCKED_SW                                   (0x00000101UL)
#define MCU_SUB_SNSADC_TKN_LOCKED_HW                                   (0x00000001UL)

/*******************  Bit definition for SADC_RET_TKN_HW register  *******************/
#define MCU_SUB_SNSADC_RET_TKN_HW_RELEASE_POS                         (0U)
#define MCU_SUB_SNSADC_RET_TKN_HW_RELEASE_Len                         (1U)
#define MCU_SUB_SNSADC_RET_TKN_HW_RELEASE_Msk                         ((0x1UL) << MCU_SUB_SNSADC_RET_TKN_HW_RELEASE_POS)
#define MCU_SUB_SNSADC_RET_TKN_HW_RELEASE                             MCU_SUB_SNSADC_RET_TKN_HW_RELEASE_Msk

/*******************  Bit definition for SADC_RET_TKN_SW register  *******************/
#define MCU_SUB_SNSADC_RET_TKN_SW_RELEASE_POS                         (0U)
#define MCU_SUB_SNSADC_RET_TKN_SW_RELEASE_Len                         (1U)
#define MCU_SUB_SNSADC_RET_TKN_SW_RELEASE_Msk                         ((0x1UL) << MCU_SUB_SNSADC_RET_TKN_SW_RELEASE_POS)
#define MCU_SUB_SNSADC_RET_TKN_SW_RELEASE                             MCU_SUB_SNSADC_RET_TKN_SW_RELEASE_Msk

/*******************  Bit definition for SADC_TKN_STAT register  *******************/
#define MCU_SUB_SNSADC_TKN_STAT_LOCKED_POS                            (0U)
#define MCU_SUB_SNSADC_TKN_STAT_LOCKED_Len                            (1U)
#define MCU_SUB_SNSADC_TKN_STAT_LOCKED_Msk                            ((0x1UL) << MCU_SUB_SNSADC_TKN_STAT_LOCKED_POS)
#define MCU_SUB_SNSADC_TKN_STAT_LOCKED                                MCU_SUB_SNSADC_TKN_STAT_LOCKED_Msk

#define MCU_SUB_SNSADC_TKN_STAT_OWNER_POS                             (8U)
#define MCU_SUB_SNSADC_TKN_STAT_OWNER_Len                             (1U)
#define MCU_SUB_SNSADC_TKN_STAT_OWNER_Msk                             ((0x1UL) << MCU_SUB_SNSADC_TKN_STAT_OWNER_POS)
#define MCU_SUB_SNSADC_TKN_STAT_OWNER                                 MCU_SUB_SNSADC_TKN_STAT_OWNER_Msk

/*******************  Bit definition for CTE_FIFO_DATA register  *******************/
#define MCU_SUB_CTE_FIFO_DATA_OUT_POS                                 (0U)
#define MCU_SUB_CTE_FIFO_DATA_OUT_Len                                 (32U)
#define MCU_SUB_CTE_FIFO_DATA_OUT_Msk                                 ((0xFFFFFFFFUL) << MCU_SUB_CTE_FIFO_DATA_OUT_POS)
#define MCU_SUB_CTE_FIFO_DATA_OUT                                     MCU_SUB_CTE_FIFO_DATA_OUT_Msk

/*******************  Bit definition for CTE_FIFO_THRESH register  *******************/
#define MCU_SUB_CTE_FIFO_THRESH_HOLD_POS                              (0U)
#define MCU_SUB_CTE_FIFO_THRESH_HOLD_Len                              (5U)
#define MCU_SUB_CTE_FIFO_THRESH_HOLD_Msk                              ((0x1FUL) << MCU_SUB_CTE_FIFO_THRESH_HOLD_POS)
#define MCU_SUB_CTE_FIFO_THRESH_HOLD                                  MCU_SUB_CTE_FIFO_THRESH_HOLD_Msk

/*******************  Bit definition for CTE_FIFO_STAT register  *******************/
#define MCU_SUB_CTE_FIFO_STAT_COUNT_POS                               (0U)
#define MCU_SUB_CTE_FIFO_STAT_COUNT_Len                               (6U)
#define MCU_SUB_CTE_FIFO_STAT_COUNT_Msk                               ((0x3FUL) << MCU_SUB_CTE_FIFO_STAT_COUNT_POS)
#define MCU_SUB_CTE_FIFO_STAT_COUNT                                   MCU_SUB_CTE_FIFO_STAT_COUNT_Msk

#define MCU_SUB_CTE_FIFO_STAT_VAL_POS                                 (8U)
#define MCU_SUB_CTE_FIFO_STAT_VAL_Len                                 (1U)
#define MCU_SUB_CTE_FIFO_STAT_VAL_Msk                                 ((0x1UL) << MCU_SUB_CTE_FIFO_STAT_VAL_POS)
#define MCU_SUB_CTE_FIFO_STAT_VAL                                     MCU_SUB_CTE_FIFO_STAT_VAL_Msk

#define MCU_SUB_CTE_FIFO_STAT_FULL_POS                                (10U)
#define MCU_SUB_CTE_FIFO_STAT_FULL_Len                                (1U)
#define MCU_SUB_CTE_FIFO_STAT_FULL_Msk                                ((0x1UL) << MCU_SUB_CTE_FIFO_STAT_FULL_POS)
#define MCU_SUB_CTE_FIFO_STAT_FULL                                    MCU_SUB_CTE_FIFO_STAT_FULL_Msk

/*******************  Bit definition for CTE_CFG register  *******************/
#define MCU_SUB_CTE_CFG_OVR_SAMP_ON_POS                               (0U)
#define MCU_SUB_CTE_CFG_OVR_SAMP_ON_Len                               (1U)
#define MCU_SUB_CTE_CFG_OVR_SAMP_ON_Msk                               ((0x1UL) << MCU_SUB_CTE_CFG_OVR_SAMP_ON_POS)
#define MCU_SUB_CTE_CFG_OVR_SAMP_ON                                   MCU_SUB_CTE_CFG_OVR_SAMP_ON_Msk

#define MCU_SUB_CTE_CFG_SAMP2CAP_POS                                  (16U)
#define MCU_SUB_CTE_CFG_SAMP2CAP_Len                                  (11U)
#define MCU_SUB_CTE_CFG_SAMP2CAP_Msk                                  ((0x7FFUL) << MCU_SUB_CTE_CFG_SAMP2CAP_POS)
#define MCU_SUB_CTE_CFG_SAMP2CAP                                      MCU_SUB_CTE_CFG_SAMP2CAP_Msk

/*******************  Bit definition for PSRAM_MEM_CLK register  *******************/
#define MCU_SUB_PSRAM_MEM_CLK_EN_POS                                  (0U)
#define MCU_SUB_PSRAM_MEM_CLK_EN_Len                                  (1U)
#define MCU_SUB_PSRAM_MEM_CLK_EN_Msk                                  ((0x1UL) << MCU_SUB_PSRAM_MEM_CLK_EN_POS)
#define MCU_SUB_PSRAM_MEM_CLK_EN                                      MCU_SUB_PSRAM_MEM_CLK_EN_Msk

#define MCU_SUB_PSRAM_MEM_CLK_SEL_POS                                 (8U)
#define MCU_SUB_PSRAM_MEM_CLK_SEL_Len                                 (2U)
#define MCU_SUB_PSRAM_MEM_CLK_SEL_Msk                                 ((0x3UL) << MCU_SUB_PSRAM_MEM_CLK_SEL_POS)
#define MCU_SUB_PSRAM_MEM_CLK_SEL                                     MCU_SUB_PSRAM_MEM_CLK_SEL_Msk

/***************  Bit definition for USB_LP_CTRL register  ********/
#define MCU_SUB_USB_LP_CTRL_SUSPEND_CLK_OFF_Pos             (0U)
#define MCU_SUB_USB_LP_CTRL_SUSPEND_CLK_OFF_Len             (1U)
#define MCU_SUB_USB_LP_CTRL_SUSPEND_CLK_OFF_Msk             (0x1U << MCU_SUB_USB_LP_CTRL_SUSPEND_CLK_OFF_Pos)
#define MCU_SUB_USB_LP_CTRL_SUSPEND_CLK_OFF                 MCU_SUB_USB_LP_CTRL_SUSPEND_CLK_OFF_Msk

#define MCU_SUB_USB_LP_CTRL_PMU_LP_EN_Pos                   (4U)
#define MCU_SUB_USB_LP_CTRL_PMU_LP_EN_Len                   (1U)
#define MCU_SUB_USB_LP_CTRL_PMU_LP_EN_Msk                   (0x1U << MCU_SUB_USB_LP_CTRL_PMU_LP_EN_Pos)
#define MCU_SUB_USB_LP_CTRL_PMU_LP_EN                       MCU_SUB_USB_LP_CTRL_PMU_LP_EN_Msk

#define MCU_SUB_USB_LP_CTRL_CLK_FORCE_OFF_Pos               (8U)
#define MCU_SUB_USB_LP_CTRL_CLK_FORCE_OFF_Len               (1U)
#define MCU_SUB_USB_LP_CTRL_CLK_FORCE_OFF_Msk               (0x1U << MCU_SUB_USB_LP_CTRL_CLK_FORCE_OFF_Pos)
#define MCU_SUB_USB_LP_CTRL_CLK_FORCE_OFF                   MCU_SUB_USB_LP_CTRL_CLK_FORCE_OFF_Msk

/***************  Bit definition for USB_XCRV_CTRL register  ********/
#define MCU_SUB_USB_XCRV_CTRL_SECMP_PD_Pos                  (0U)
#define MCU_SUB_USB_XCRV_CTRL_SECMP_PD_Len                  (1U)
#define MCU_SUB_USB_XCRV_CTRL_SECMP_PD_Msk                  (0x1U << MCU_SUB_USB_XCRV_CTRL_SECMP_PD_Pos)
#define MCU_SUB_USB_XCRV_CTRL_SECMP_PD                      MCU_SUB_USB_XCRV_CTRL_SECMP_PD_Msk

#define MCU_SUB_USB_XCRV_CTRL_SPEED_Pos                     (1U)
#define MCU_SUB_USB_XCRV_CTRL_SPEED_Len                     (1U)
#define MCU_SUB_USB_XCRV_CTRL_SPEED_Msk                     (0x1U << MCU_SUB_USB_XCRV_CTRL_SPEED_Pos)
#define MCU_SUB_USB_XCRV_CTRL_SPEED                         MCU_SUB_USB_XCRV_CTRL_SPEED_Msk

#define MCU_SUB_USB_XCRV_CTRL_SUSPEND_Pos                   (2U)
#define MCU_SUB_USB_XCRV_CTRL_SUSPEND_Len                   (1U)
#define MCU_SUB_USB_XCRV_CTRL_SUSPEND_Msk                   (0x1U << MCU_SUB_USB_XCRV_CTRL_SUSPEND_Pos)
#define MCU_SUB_USB_XCRV_CTRL_SUSPEND                       MCU_SUB_USB_XCRV_CTRL_SUSPEND_Msk

#define MCU_SUB_USB_XCRV_CTRL_BIAS_EN_Pos                   (3U)
#define MCU_SUB_USB_XCRV_CTRL_BIAS_EN_Len                   (1U)
#define MCU_SUB_USB_XCRV_CTRL_BIAS_EN_Msk                   (0x1U << MCU_SUB_USB_XCRV_CTRL_BIAS_EN_Pos)
#define MCU_SUB_USB_XCRV_CTRL_BIAS_EN                       MCU_SUB_USB_XCRV_CTRL_BIAS_EN_Msk

#define MCU_SUB_USB_XCRV_CTRL_RTRIMN_Pos                    (4U)
#define MCU_SUB_USB_XCRV_CTRL_RTRIMN_Len                    (3U)
#define MCU_SUB_USB_XCRV_CTRL_RTRIMN_Msk                    (0x7U << MCU_SUB_USB_XCRV_CTRL_RTRIMN_Pos)
#define MCU_SUB_USB_XCRV_CTRL_RTRIMN                        MCU_SUB_USB_XCRV_CTRL_RTRIMN_Msk

#define MCU_SUB_USB_XCRV_CTRL_RTRIMP_Pos                    (8U)
#define MCU_SUB_USB_XCRV_CTRL_RTRIMP_Len                    (3U)
#define MCU_SUB_USB_XCRV_CTRL_RTRIMP_Msk                    (0x7U << MCU_SUB_USB_XCRV_CTRL_RTRIMP_Pos)
#define MCU_SUB_USB_XCRV_CTRL_RTRIMP                        MCU_SUB_USB_XCRV_CTRL_RTRIMP_Msk

/***************  Bit definition for USB_XCRV_LDO register  ********/
#define MCU_SUB_USB_XCRV_LDO_VREF_TEST_EN_Pos               (0U)
#define MCU_SUB_USB_XCRV_LDO_VREF_TEST_EN_Len               (1U)
#define MCU_SUB_USB_XCRV_LDO_VREF_TEST_EN_Msk               (0x1U << MCU_SUB_USB_XCRV_LDO_VREF_TEST_EN_Pos)
#define MCU_SUB_USB_XCRV_LDO_VREF_TEST_EN                   MCU_SUB_USB_XCRV_LDO_VREF_TEST_EN_Msk

/***************  Bit definition for USB_SW_RST register  ********/
#define MCU_SUB_USB_SW_RST_EN_Pos                           (0U)
#define MCU_SUB_USB_SW_RST_EN_Len                           (1U)
#define MCU_SUB_USB_SW_RST_EN_Msk                           (0x1U << MCU_SUB_USB_SW_RST_EN_Pos)
#define MCU_SUB_USB_SW_RST_EN                               MCU_SUB_USB_SW_RST_EN_Msk

/***************  Bit definition for QSPI_M_XIP register  ********/
#define MCU_SUB_QSPI_M_XIP_M0_XIP_EN_OVR_Pos                (0U)
#define MCU_SUB_QSPI_M_XIP_M0_XIP_EN_OVR_Len                (1U)
#define MCU_SUB_QSPI_M_XIP_M0_XIP_EN_OVR_Msk                (0x1U << MCU_SUB_QSPI_M_XIP_M0_XIP_EN_OVR_Pos)
#define MCU_SUB_QSPI_M_XIP_M0_XIP_EN_OVR                    MCU_SUB_QSPI_M_XIP_M0_XIP_EN_OVR_Msk

#define MCU_SUB_QSPI_M_XIP_M0_XIP_EN_VAL_Pos                (1U)
#define MCU_SUB_QSPI_M_XIP_M0_XIP_EN_VAL_Len                (1U)
#define MCU_SUB_QSPI_M_XIP_M0_XIP_EN_VAL_Msk                (0x1U << MCU_SUB_QSPI_M_XIP_M0_XIP_EN_VAL_Pos)
#define MCU_SUB_QSPI_M_XIP_M0_XIP_EN_VAL                    MCU_SUB_QSPI_M_XIP_M0_XIP_EN_VAL_Msk

#define MCU_SUB_QSPI_M_XIP_M0_ENDIAN_MODE_Pos               (2U)
#define MCU_SUB_QSPI_M_XIP_M0_ENDIAN_MODE_Len               (2U)
#define MCU_SUB_QSPI_M_XIP_M0_ENDIAN_MODE_Msk               (0x3U << MCU_SUB_QSPI_M_XIP_M0_ENDIAN_MODE_Pos)
#define MCU_SUB_QSPI_M_XIP_M0_ENDIAN_MODE                   MCU_SUB_QSPI_M_XIP_M0_ENDIAN_MODE_Msk

#define MCU_SUB_QSPI_M_XIP_M1_XIP_EN_OVR_Pos                (4U)
#define MCU_SUB_QSPI_M_XIP_M1_XIP_EN_OVR_Len                (1U)
#define MCU_SUB_QSPI_M_XIP_M1_XIP_EN_OVR_Msk                (0x1U << MCU_SUB_QSPI_M_XIP_M1_XIP_EN_OVR_Pos)
#define MCU_SUB_QSPI_M_XIP_M1_XIP_EN_OVR                    MCU_SUB_QSPI_M_XIP_M1_XIP_EN_OVR_Msk

#define MCU_SUB_QSPI_M_XIP_M1_XIP_EN_VAL_Pos                (5U)
#define MCU_SUB_QSPI_M_XIP_M1_XIP_EN_VAL_Len                (1U)
#define MCU_SUB_QSPI_M_XIP_M1_XIP_EN_VAL_Msk                (0x1U << MCU_SUB_QSPI_M_XIP_M1_XIP_EN_VAL_Pos)
#define MCU_SUB_QSPI_M_XIP_M1_XIP_EN_VAL                    MCU_SUB_QSPI_M_XIP_M1_XIP_EN_VAL_Msk

#define MCU_SUB_QSPI_M_XIP_M1_ENDIAN_MODE_Pos               (6U)
#define MCU_SUB_QSPI_M_XIP_M1_ENDIAN_MODE_Len               (2U)
#define MCU_SUB_QSPI_M_XIP_M1_ENDIAN_MODE_Msk               (0x3U << MCU_SUB_QSPI_M_XIP_M1_ENDIAN_MODE_Pos)
#define MCU_SUB_QSPI_M_XIP_M1_ENDIAN_MODE                   MCU_SUB_QSPI_M_XIP_M1_ENDIAN_MODE_Msk

#define MCU_SUB_QSPI_M_XIP_M2_XIP_EN_OVR_Pos                (8U)
#define MCU_SUB_QSPI_M_XIP_M2_XIP_EN_OVR_Len                (1U)
#define MCU_SUB_QSPI_M_XIP_M2_XIP_EN_OVR_Msk                (0x1U << MCU_SUB_QSPI_M_XIP_M2_XIP_EN_OVR_Pos)
#define MCU_SUB_QSPI_M_XIP_M2_XIP_EN_OVR                    MCU_SUB_QSPI_M_XIP_M2_XIP_EN_OVR_Msk

#define MCU_SUB_QSPI_M_XIP_M2_XIP_EN_VAL_Pos                (9U)
#define MCU_SUB_QSPI_M_XIP_M2_XIP_EN_VAL_Len                (1U)
#define MCU_SUB_QSPI_M_XIP_M2_XIP_EN_VAL_Msk                (0x1U << MCU_SUB_QSPI_M_XIP_M2_XIP_EN_VAL_Pos)
#define MCU_SUB_QSPI_M_XIP_M2_XIP_EN_VAL                    MCU_SUB_QSPI_M_XIP_M2_XIP_EN_VAL_Msk

#define MCU_SUB_QSPI_M_XIP_M2_ENDIAN_MODE_Pos               (10U)
#define MCU_SUB_QSPI_M_XIP_M2_ENDIAN_MODE_Len               (2U)
#define MCU_SUB_QSPI_M_XIP_M2_ENDIAN_MODE_Msk               (0x3U << MCU_SUB_QSPI_M_XIP_M2_ENDIAN_MODE_Pos)
#define MCU_SUB_QSPI_M_XIP_M2_ENDIAN_MODE                   MCU_SUB_QSPI_M_XIP_M2_ENDIAN_MODE_Msk

#define MCU_SUB_QSPI_M_XIP_EN_OVR_Msk                       (MCU_SUB_QSPI_M_XIP_M0_XIP_EN_OVR_Msk | MCU_SUB_QSPI_M_XIP_M1_XIP_EN_OVR_Msk | MCU_SUB_QSPI_M_XIP_M2_XIP_EN_OVR_Msk)

#define MCU_SUB_QSPI_M_XIP_EN_VAL_Msk                       (MCU_SUB_QSPI_M_XIP_M0_XIP_EN_VAL_Msk | MCU_SUB_QSPI_M_XIP_M1_XIP_EN_VAL_Msk | MCU_SUB_QSPI_M_XIP_M2_XIP_EN_VAL_Msk)

#define MCU_SUB_QSPI_M_XIP_ENDIAN_ORDER                     (MCU_SUB_QSPI_M_XIP_M0_ENDIAN_MODE | MCU_SUB_QSPI_M_XIP_M1_ENDIAN_MODE | MCU_SUB_QSPI_M_XIP_M2_ENDIAN_MODE)

#define MCU_SUB_QSPI_M_XIP_M0_DYNAMIC_LE_EN_Pos             (29U)
#define MCU_SUB_QSPI_M_XIP_M0_DYNAMIC_LE_EN_Len             (1U)
#define MCU_SUB_QSPI_M_XIP_M0_DYNAMIC_LE_EN_Msk             (0x1U << MCU_SUB_QSPI_M_XIP_M0_DYNAMIC_LE_EN_Pos)
#define MCU_SUB_QSPI_M_XIP_M0_DYNAMIC_LE_EN                 MCU_SUB_QSPI_M_XIP_M0_DYNAMIC_LE_EN_Msk

#define MCU_SUB_QSPI_M_XIP_M1_DYNAMIC_LE_EN_Pos             (30U)
#define MCU_SUB_QSPI_M_XIP_M1_DYNAMIC_LE_EN_Len             (1U)
#define MCU_SUB_QSPI_M_XIP_M1_DYNAMIC_LE_EN_Msk             (0x1U << MCU_SUB_QSPI_M_XIP_M1_DYNAMIC_LE_EN_Pos)
#define MCU_SUB_QSPI_M_XIP_M1_DYNAMIC_LE_EN                 MCU_SUB_QSPI_M_XIP_M1_DYNAMIC_LE_EN_Msk

#define MCU_SUB_QSPI_M_XIP_M2_DYNAMIC_LE_EN_Pos             (31U)
#define MCU_SUB_QSPI_M_XIP_M2_DYNAMIC_LE_EN_Len             (1U)
#define MCU_SUB_QSPI_M_XIP_M2_DYNAMIC_LE_EN_Msk             (0x1U << MCU_SUB_QSPI_M_XIP_M2_DYNAMIC_LE_EN_Pos)
#define MCU_SUB_QSPI_M_XIP_M2_DYNAMIC_LE_EN                 MCU_SUB_QSPI_M_XIP_M2_DYNAMIC_LE_EN_Msk

/***************  Bit definition for QSPI_M_HRESP_DBG register  ***/
#define MCU_SUB_QSPI_M_HRESP_ERR_MASK_EN_Pos                (0U)
#define MCU_SUB_QSPI_M_HRESP_ERR_MASK_EN_Len                (1U)
#define MCU_SUB_QSPI_M_HRESP_ERR_MASK_EN_Msk                (0x1U << MCU_SUB_QSPI_M_HRESP_ERR_MASK_EN_Pos)
#define MCU_SUB_QSPI_M_HRESP_ERR_MASK_EN                    MCU_SUB_QSPI_M_HRESP_ERR_MASK_EN_Msk

/***************  Bit definition for QSPI_M_CS_SETUP_DLY register  ***/
#define MCU_SUB_QSPI_M0_CS_SETUP_DLY_Pos                     (0U)
#define MCU_SUB_QSPI_M0_CS_SETUP_DLY_Len                     (8U)
#define MCU_SUB_QSPI_M0_CS_SETUP_DLY_Msk                     (0xFFU << MCU_SUB_QSPI_M0_CS_SETUP_DLY_Pos)
#define MCU_SUB_QSPI_M0_CS_SETUP_DLY                         MCU_SUB_QSPI_M0_CS_SETUP_DLY_Msk

#define MCU_SUB_QSPI_M1_CS_SETUP_DLY_Pos                     (8U)
#define MCU_SUB_QSPI_M1_CS_SETUP_DLY_Len                     (8U)
#define MCU_SUB_QSPI_M1_CS_SETUP_DLY_Msk                     (0xFFU << MCU_SUB_QSPI_M1_CS_SETUP_DLY_Pos)
#define MCU_SUB_QSPI_M1_CS_SETUP_DLY                         MCU_SUB_QSPI_M1_CS_SETUP_DLY_Msk

#define MCU_SUB_QSPI_M2_CS_SETUP_DLY_Pos                     (16U)
#define MCU_SUB_QSPI_M2_CS_SETUP_DLY_Len                     (8U)
#define MCU_SUB_QSPI_M2_CS_SETUP_DLY_Msk                     (0xFFU << MCU_SUB_QSPI_M2_CS_SETUP_DLY_Pos)
#define MCU_SUB_QSPI_M2_CS_SETUP_DLY                         MCU_SUB_QSPI_M2_CS_SETUP_DLY_Msk

/***************  Bit definition for QSPI_M_SCK_DELAY register  ***/
#define MCU_SUB_QSPI_M0_CS_RELEASE_DLY_Pos                   (0U)
#define MCU_SUB_QSPI_M0_CS_RELEASE_DLY_Len                   (8U)
#define MCU_SUB_QSPI_M0_CS_RELEASE_DLY_Msk                   (0xFFU << MCU_SUB_QSPI_M0_CS_RELEASE_DLY_Pos)
#define MCU_SUB_QSPI_M0_CS_RELEASE_DLY                       MCU_SUB_QSPI_M0_CS_RELEASE_DLY_Msk

#define MCU_SUB_QSPI_M1_CS_RELEASE_DLY_Pos                   (8U)
#define MCU_SUB_QSPI_M1_CS_RELEASE_DLY_Len                   (8U)
#define MCU_SUB_QSPI_M1_CS_RELEASE_DLY_Msk                   (0xFFU << MCU_SUB_QSPI_M1_CS_RELEASE_DLY_Pos)
#define MCU_SUB_QSPI_M1_CS_RELEASE_DLY                       MCU_SUB_QSPI_M1_CS_RELEASE_DLY_Msk

#define MCU_SUB_QSPI_M2_CS_RELEASE_DLY_Pos                   (16U)
#define MCU_SUB_QSPI_M2_CS_RELEASE_DLY_Len                   (8U)
#define MCU_SUB_QSPI_M2_CS_RELEASE_DLY_Msk                   (0xFFU << MCU_SUB_QSPI_M2_CS_RELEASE_DLY_Pos)
#define MCU_SUB_QSPI_M2_CS_RELEASE_DLY                       MCU_SUB_QSPI_M2_CS_RELEASE_DLY_Msk

/***************  Bit definition for DMA0_HS_SEL register  ********/
#define MCU_SUB_DMA0_HS2_3_SEL_Pos                          (2U)
#define MCU_SUB_DMA0_HS2_3_SEL_Len                          (1U)
#define MCU_SUB_DMA0_HS2_3_SEL_Msk                          (0x1U << MCU_SUB_DMA0_HS2_3_SEL_Pos)
#define MCU_SUB_DMA0_HS2_3_SEL                              MCU_SUB_DMA0_HS2_3_SEL_Msk
#define MCU_SUB_DMA0_HS2_3_SEL_H0                           (0x0U << MCU_SUB_DMA0_HS2_3_SEL_Pos)
#define MCU_SUB_DMA0_HS2_3_SEL_H1                           (0x1U << MCU_SUB_DMA0_HS2_3_SEL_Pos)

#define MCU_SUB_DMA0_HS4_5_SEL_Pos                          (4U)
#define MCU_SUB_DMA0_HS4_5_SEL_Len                          (1U)
#define MCU_SUB_DMA0_HS4_5_SEL_Msk                          (0x1U << MCU_SUB_DMA0_HS4_5_SEL_Pos)
#define MCU_SUB_DMA0_HS4_5_SEL                              MCU_SUB_DMA0_HS4_5_SEL_Msk
#define MCU_SUB_DMA0_HS4_5_SEL_H0                           (0x0U << MCU_SUB_DMA0_HS4_5_SEL_Pos)
#define MCU_SUB_DMA0_HS4_5_SEL_H1                           (0x1U << MCU_SUB_DMA0_HS4_5_SEL_Pos)

#define MCU_SUB_DMA0_HS6_7_SEL_Pos                          (6U)
#define MCU_SUB_DMA0_HS6_7_SEL_Len                          (1U)
#define MCU_SUB_DMA0_HS6_7_SEL_Msk                          (0x1U << MCU_SUB_DMA0_HS6_7_SEL_Pos)
#define MCU_SUB_DMA0_HS6_7_SEL                              MCU_SUB_DMA0_HS6_7_SEL_Msk
#define MCU_SUB_DMA0_HS6_7_SEL_H0                           (0x0U << MCU_SUB_DMA0_HS6_7_SEL_Pos)
#define MCU_SUB_DMA0_HS6_7_SEL_H1                           (0x1U << MCU_SUB_DMA0_HS6_7_SEL_Pos)

#define MCU_SUB_DMA0_HS8_9_SEL_Pos                          (8U)
#define MCU_SUB_DMA0_HS8_9_SEL_Len                          (1U)
#define MCU_SUB_DMA0_HS8_9_SEL_Msk                          (0x1U << MCU_SUB_DMA0_HS8_9_SEL_Pos)
#define MCU_SUB_DMA0_HS8_9_SEL                              MCU_SUB_DMA0_HS8_9_SEL_Msk
#define MCU_SUB_DMA0_HS8_9_SEL_H0                           (0x0U << MCU_SUB_DMA0_HS8_9_SEL_Pos)
#define MCU_SUB_DMA0_HS8_9_SEL_H1                           (0x1U << MCU_SUB_DMA0_HS8_9_SEL_Pos)

#define MCU_SUB_DMA0_HS10_11_SEL_Pos                        (10U)
#define MCU_SUB_DMA0_HS10_11_SEL_Len                        (1U)
#define MCU_SUB_DMA0_HS10_11_SEL_Msk                        (0x1U << MCU_SUB_DMA0_HS10_11_SEL_Pos)
#define MCU_SUB_DMA0_HS10_11_SEL                            MCU_SUB_DMA0_HS10_11_SEL_Msk
#define MCU_SUB_DMA0_HS10_11_SEL_H0                         (0x0U << MCU_SUB_DMA0_HS10_11_SEL_Pos)
#define MCU_SUB_DMA0_HS10_11_SEL_H1                         (0x1U << MCU_SUB_DMA0_HS10_11_SEL_Pos)

#define MCU_SUB_DMA0_HS12_13_SEL_Pos                        (12U)
#define MCU_SUB_DMA0_HS12_13_SEL_Len                        (1U)
#define MCU_SUB_DMA0_HS12_13_SEL_Msk                        (0x1U << MCU_SUB_DMA0_HS12_13_SEL_Pos)
#define MCU_SUB_DMA0_HS12_13_SEL                            MCU_SUB_DMA0_HS12_13_SEL_Msk
#define MCU_SUB_DMA0_HS12_13_SEL_H0                         (0x0U << MCU_SUB_DMA0_HS12_13_SEL_Pos)
#define MCU_SUB_DMA0_HS12_13_SEL_H1                         (0x1U << MCU_SUB_DMA0_HS12_13_SEL_Pos)

#define MCU_SUB_DMA0_HS14_15_SEL_Pos                        (14U)
#define MCU_SUB_DMA0_HS14_15_SEL_Len                        (1U)
#define MCU_SUB_DMA0_HS14_15_SEL_Msk                        (0x1U << MCU_SUB_DMA0_HS14_15_SEL_Pos)
#define MCU_SUB_DMA0_HS14_15_SEL                            MCU_SUB_DMA0_HS14_15_SEL_Msk
#define MCU_SUB_DMA0_HS14_15_SEL_H0                         (0x0U << MCU_SUB_DMA0_HS14_15_SEL_Pos)
#define MCU_SUB_DMA0_HS14_15_SEL_H1                         (0x1U << MCU_SUB_DMA0_HS14_15_SEL_Pos)

/***************  Bit definition for DMA1_HS_SEL register  ********/
#define MCU_SUB_DMA1_HS0_1_SEL_Pos                          (0U)
#define MCU_SUB_DMA1_HS0_1_SEL_Len                          (1U)
#define MCU_SUB_DMA1_HS0_1_SEL_Msk                          (0x1U << MCU_SUB_DMA1_HS0_1_SEL_Pos)
#define MCU_SUB_DMA1_HS0_1_SEL                              MCU_SUB_DMA1_HS0_1_SEL_Msk
#define MCU_SUB_DMA1_HS0_1_SEL_H0                           (0x0U << MCU_SUB_DMA1_HS0_1_SEL_Pos)
#define MCU_SUB_DMA1_HS0_1_SEL_H1                           (0x1U << MCU_SUB_DMA1_HS0_1_SEL_Pos)

#define MCU_SUB_DMA1_HS2_3_SEL_Pos                          (2U)
#define MCU_SUB_DMA1_HS2_3_SEL_Len                          (1U)
#define MCU_SUB_DMA1_HS2_3_SEL_Msk                          (0x1U << MCU_SUB_DMA1_HS2_3_SEL_Pos)
#define MCU_SUB_DMA1_HS2_3_SEL                              MCU_SUB_DMA1_HS2_3_SEL_Msk
#define MCU_SUB_DMA1_HS2_3_SEL_H0                           (0x0U << MCU_SUB_DMA1_HS2_3_SEL_Pos)
#define MCU_SUB_DMA1_HS2_3_SEL_H1                           (0x1U << MCU_SUB_DMA1_HS2_3_SEL_Pos)

#define MCU_SUB_DMA1_HS4_5_SEL_Pos                          (4U)
#define MCU_SUB_DMA1_HS4_5_SEL_Len                          (1U)
#define MCU_SUB_DMA1_HS4_5_SEL_Msk                          (0x1U << MCU_SUB_DMA1_HS4_5_SEL_Pos)
#define MCU_SUB_DMA1_HS4_5_SEL                              MCU_SUB_DMA1_HS4_5_SEL_Msk
#define MCU_SUB_DMA1_HS4_5_SEL_H0                           (0x0U << MCU_SUB_DMA1_HS4_5_SEL_Pos)
#define MCU_SUB_DMA1_HS4_5_SEL_H1                           (0x1U << MCU_SUB_DMA1_HS4_5_SEL_Pos)

#define MCU_SUB_DMA1_HS6_7_SEL_Pos                          (6U)
#define MCU_SUB_DMA1_HS6_7_SEL_Len                          (1U)
#define MCU_SUB_DMA1_HS6_7_SEL_Msk                          (0x1U << MCU_SUB_DMA1_HS6_7_SEL_Pos)
#define MCU_SUB_DMA1_HS6_7_SEL                              MCU_SUB_DMA1_HS6_7_SEL_Msk
#define MCU_SUB_DMA1_HS6_7_SEL_H0                           (0x0U << MCU_SUB_DMA1_HS6_7_SEL_Pos)
#define MCU_SUB_DMA1_HS6_7_SEL_H1                           (0x1U << MCU_SUB_DMA1_HS6_7_SEL_Pos)

#define MCU_SUB_DMA1_HS8_9_SEL_Pos                          (8U)
#define MCU_SUB_DMA1_HS8_9_SEL_Len                          (1U)
#define MCU_SUB_DMA1_HS8_9_SEL_Msk                          (0x1U << MCU_SUB_DMA1_HS8_9_SEL_Pos)
#define MCU_SUB_DMA1_HS8_9_SEL                              MCU_SUB_DMA1_HS8_9_SEL_Msk
#define MCU_SUB_DMA1_HS8_9_SEL_H0                           (0x0U << MCU_SUB_DMA1_HS8_9_SEL_Pos)
#define MCU_SUB_DMA1_HS8_9_SEL_H1                           (0x1U << MCU_SUB_DMA1_HS8_9_SEL_Pos)

#define MCU_SUB_DMA1_HS10_11_SEL_Pos                        (10U)
#define MCU_SUB_DMA1_HS10_11_SEL_Len                        (1U)
#define MCU_SUB_DMA1_HS10_11_SEL_Msk                        (0x1U << MCU_SUB_DMA1_HS10_11_SEL_Pos)
#define MCU_SUB_DMA1_HS10_11_SEL                            MCU_SUB_DMA1_HS10_11_SEL_Msk
#define MCU_SUB_DMA1_HS10_11_SEL_H0                         (0x0U << MCU_SUB_DMA1_HS10_11_SEL_Pos)
#define MCU_SUB_DMA1_HS10_11_SEL_H1                         (0x1U << MCU_SUB_DMA1_HS10_11_SEL_Pos)

#define MCU_SUB_DMA1_HS14_15_SEL_Pos                        (14U)
#define MCU_SUB_DMA1_HS14_15_SEL_Len                        (1U)
#define MCU_SUB_DMA1_HS14_15_SEL_Msk                        (0x1U << MCU_SUB_DMA1_HS14_15_SEL_Pos)
#define MCU_SUB_DMA1_HS14_15_SEL                            MCU_SUB_DMA1_HS14_15_SEL_Msk
#define MCU_SUB_DMA1_HS14_15_SEL_H0                         (0x0U << MCU_SUB_DMA1_HS14_15_SEL_Pos)
#define MCU_SUB_DMA1_HS14_15_SEL_H1                         (0x1U << MCU_SUB_DMA1_HS14_15_SEL_Pos)

/***************  Bit definition for PAD_IE_N_AUTO register  ********/
#define MCU_SUB_PAD_IE_N_AUTO_DPAD_EN_Pos                   (0U)
#define MCU_SUB_PAD_IE_N_AUTO_DPAD_EN_Len                   (1U)
#define MCU_SUB_PAD_IE_N_AUTO_DPAD_EN_Msk                   (0x1U << MCU_SUB_PAD_IE_N_AUTO_DPAD_EN_Pos)
#define MCU_SUB_PAD_IE_N_AUTO_DPAD_EN                       MCU_SUB_PAD_IE_N_AUTO_DPAD_EN_Msk

#define MCU_SUB_PAD_IE_N_AUTO_MSIO_EN_Pos                   (1U)
#define MCU_SUB_PAD_IE_N_AUTO_MSIO_EN_Len                   (1U)
#define MCU_SUB_PAD_IE_N_AUTO_MSIO_EN_Msk                   (0x1U << MCU_SUB_PAD_IE_N_AUTO_MSIO_EN_Pos)
#define MCU_SUB_PAD_IE_N_AUTO_MSIO_EN                       MCU_SUB_PAD_IE_N_AUTO_MSIO_EN_Msk

#define MCU_SUB_PAD_IE_N_AUTO_MISC_EN_Pos                   (2U)
#define MCU_SUB_PAD_IE_N_AUTO_MISC_EN_Len                   (1U)
#define MCU_SUB_PAD_IE_N_AUTO_MISC_EN_Msk                   (0x1U << MCU_SUB_PAD_IE_N_AUTO_MISC_EN_Pos)
#define MCU_SUB_PAD_IE_N_AUTO_MISC_EN                       MCU_SUB_PAD_IE_N_AUTO_MISC_EN_Msk

/***************  Bit definition for BLE_FERP_CTL register  ********/
#define MCU_SUB_BLE_FERP_CTL_FERP_EN_Pos                    (0U)
#define MCU_SUB_BLE_FERP_CTL_FERP_EN_Len                    (1U)
#define MCU_SUB_BLE_FERP_CTL_FERP_EN_Msk                    (0x1U << MCU_SUB_BLE_FERP_CTL_FERP_EN_Pos)
#define MCU_SUB_BLE_FERP_CTL_FERP_EN                        MCU_SUB_BLE_FERP_CTL_FERP_EN_Msk

#define MCU_SUB_BLE_FERP_CTL_TESTBUS_SEL_Pos                (4U)
#define MCU_SUB_BLE_FERP_CTL_TESTBUS_SEL_Len                (3U)
#define MCU_SUB_BLE_FERP_CTL_TESTBUS_SEL_Msk                (0x7U << MCU_SUB_BLE_FERP_CTL_TESTBUS_SEL_Pos)
#define MCU_SUB_BLE_FERP_CTL_TESTBUS_SEL                    MCU_SUB_BLE_FERP_CTL_TESTBUS_SEL_Msk

/***************  Bit definition for SECURITY_RESET register  ********/
#define MCU_SUB_SECURITY_RESET_AES_Pos                      (0U)
#define MCU_SUB_SECURITY_RESET_AES_Len                      (1U)
#define MCU_SUB_SECURITY_RESET_AES_Msk                      (0x1U << MCU_SUB_SECURITY_RESET_AES_Pos)
#define MCU_SUB_SECURITY_RESET_AES                          MCU_SUB_SECURITY_RESET_AES_Msk

#define MCU_SUB_SECURITY_RESET_HAMC_Pos                     (1U)
#define MCU_SUB_SECURITY_RESET_HAMC_Len                     (1U)
#define MCU_SUB_SECURITY_RESET_HAMC_Msk                     (0x1U << MCU_SUB_SECURITY_RESET_HAMC_Pos)
#define MCU_SUB_SECURITY_RESET_HAMC                         MCU_SUB_SECURITY_RESET_HAMC_Msk

#define MCU_SUB_SECURITY_RESET_PKC_Pos                      (2U)
#define MCU_SUB_SECURITY_RESET_PKC_Len                      (1U)
#define MCU_SUB_SECURITY_RESET_PKC_Msk                      (0x1U << MCU_SUB_SECURITY_RESET_PKC_Pos)
#define MCU_SUB_SECURITY_RESET_PKC                          MCU_SUB_SECURITY_RESET_PKC_Msk

#define MCU_SUB_SECURITY_RESET_EFUSE_Pos                    (3U)
#define MCU_SUB_SECURITY_RESET_EFUSE_Len                    (1U)
#define MCU_SUB_SECURITY_RESET_EFUSE_Msk                    (0x1U << MCU_SUB_SECURITY_RESET_EFUSE_Pos)
#define MCU_SUB_SECURITY_RESET_EFUSE                        MCU_SUB_SECURITY_RESET_EFUSE_Msk

#define MCU_SUB_SECURITY_RESET_RAMKEY_Pos                   (4U)
#define MCU_SUB_SECURITY_RESET_RAMKEY_Len                   (1U)
#define MCU_SUB_SECURITY_RESET_RAMKEY_Msk                   (0x1U << MCU_SUB_SECURITY_RESET_RAMKEY_Pos)
#define MCU_SUB_SECURITY_RESET_RAMKEY                       MCU_SUB_SECURITY_RESET_RAMKEY_Msk

#define MCU_SUB_SECURITY_RESET_TRNG_Pos                     (5U)
#define MCU_SUB_SECURITY_RESET_TRNG_Len                     (1U)
#define MCU_SUB_SECURITY_RESET_TRNG_Msk                     (0x1U << MCU_SUB_SECURITY_RESET_TRNG_Pos)
#define MCU_SUB_SECURITY_RESET_TRNG                         MCU_SUB_SECURITY_RESET_TRNG_Msk

#define MCU_SUB_SECURITY_RESET_PRESENT_Pos                  (6U)
#define MCU_SUB_SECURITY_RESET_PRESENT_Len                  (1U)
#define MCU_SUB_SECURITY_RESET_PRESENT_Msk                  (0x1U << MCU_SUB_SECURITY_RESET_PRESENT_Pos)
#define MCU_SUB_SECURITY_RESET_PRESENT                      MCU_SUB_SECURITY_RESET_PRESENT_Msk

/***************  Bit definition for PMU_ID register  ********/
#define MCU_SUB_PMU_ID_Pos                                  (0U)
#define MCU_SUB_PMU_ID_Len                                  (8U)
#define MCU_SUB_PMU_ID_Msk                                  (0xFFU << MCU_SUB_PMU_ID_Pos)
#define MCU_SUB_PMU_ID                                      MCU_SUB_PMU_ID_Msk


/***************  Bit definition for PWR_AVG_CTL_REG0 register  ********/
#define MCU_SUB_PWR_AVG_CTL0_TPA_ADC_OUT_Pos                (24U)
#define MCU_SUB_PWR_AVG_CTL0_TPA_ADC_OUT_Len                (8U)
#define MCU_SUB_PWR_AVG_CTL0_TPA_ADC_OUT_Msk                (0xFFU << MCU_SUB_PWR_AVG_CTL0_TPA_ADC_OUT_Pos)
#define MCU_SUB_PWR_AVG_CTL0_TPA_ADC_OUT                    MCU_SUB_PWR_AVG_CTL0_TPA_ADC_OUT_Msk

#define MCU_SUB_PWR_AVG_CTL0_AVG_PWR_ERR_Pos                (18U)
#define MCU_SUB_PWR_AVG_CTL0_AVG_PWR_ERR_Len                (1U)
#define MCU_SUB_PWR_AVG_CTL0_AVG_PWR_ERR_Msk                (0x1U << MCU_SUB_PWR_AVG_CTL0_AVG_PWR_ERR_Pos)
#define MCU_SUB_PWR_AVG_CTL0_AVG_PWR_ERR                    MCU_SUB_PWR_AVG_CTL0_AVG_PWR_ERR_Msk

#define MCU_SUB_PWR_AVG_CTL0_AVG_PWR_RDY_Pos                (16U)
#define MCU_SUB_PWR_AVG_CTL0_AVG_PWR_RDY_Len                (1U)
#define MCU_SUB_PWR_AVG_CTL0_AVG_PWR_RDY_Msk                (0x1U << MCU_SUB_PWR_AVG_CTL0_AVG_PWR_RDY_Pos)
#define MCU_SUB_PWR_AVG_CTL0_AVG_PWR_RDY                    MCU_SUB_PWR_AVG_CTL0_AVG_PWR_RDY_Msk

#define MCU_SUB_PWR_AVG_CTL0_AVG_PWR_Pos                    (8U)
#define MCU_SUB_PWR_AVG_CTL0_AVG_PWR_Len                    (8U)
#define MCU_SUB_PWR_AVG_CTL0_AVG_PWR_Msk                    (0xFFU << MCU_SUB_PWR_AVG_CTL0_AVG_PWR_Pos)
#define MCU_SUB_PWR_AVG_CTL0_AVG_PWR                        MCU_SUB_PWR_AVG_CTL0_AVG_PWR_Msk

#define MCU_SUB_PWR_AVG_CTL0_SAMPL_PWR_Pos                  (4U)
#define MCU_SUB_PWR_AVG_CTL0_SAMPL_PWR_Len                  (4U)
#define MCU_SUB_PWR_AVG_CTL0_SAMPL_PWR_Msk                  (0xFU << MCU_SUB_PWR_AVG_CTL0_SAMPL_PWR_Pos)
#define MCU_SUB_PWR_AVG_CTL0_SAMPL_PWR                      MCU_SUB_PWR_AVG_CTL0_SAMPL_PWR_Msk

#define MCU_SUB_PWR_AVG_CTL0_BLE_F_TX_EN_Pos                (3U)
#define MCU_SUB_PWR_AVG_CTL0_BLE_F_TX_EN_Len                (1U)
#define MCU_SUB_PWR_AVG_CTL0_BLE_F_TX_EN_Msk                (0x1 << MCU_SUB_PWR_AVG_CTL0_BLE_F_TX_EN_Pos)
#define MCU_SUB_PWR_AVG_CTL0_BLE_F_TX_EN                    MCU_SUB_PWR_AVG_CTL0_BLE_F_TX_EN_Msk

#define MCU_SUB_PWR_AVG_CTL0_ONESHOT_EN_Pos                 (2U)
#define MCU_SUB_PWR_AVG_CTL0_ONESHOT_EN_Len                 (1U)
#define MCU_SUB_PWR_AVG_CTL0_ONESHOT_EN_Msk                 (0x1 << MCU_SUB_PWR_AVG_CTL0_ONESHOT_EN_Pos)
#define MCU_SUB_PWR_AVG_CTL0_ONESHOT_EN                     MCU_SUB_PWR_AVG_CTL0_ONESHOT_EN_Msk

#define MCU_SUB_PWR_AVG_CTL0_PWR_AVG_EN_Pos                 (0U)
#define MCU_SUB_PWR_AVG_CTL0_PWR_AVG_EN_Len                 (1U)
#define MCU_SUB_PWR_AVG_CTL0_PWR_AVG_EN_Msk                 (0x1 << MCU_SUB_PWR_AVG_CTL0_PWR_AVG_EN_Pos)
#define MCU_SUB_PWR_AVG_CTL0_PWR_AVG_EN                     MCU_SUB_PWR_AVG_CTL0_PWR_AVG_EN_Msk

/***************  Bit definition for EFUSE_PWR_DELTA_0 register  ********/
#define MCU_SUB_EFUSE_PWR_DELTA1_Pos                        (16U)
#define MCU_SUB_EFUSE_PWR_DELTA1_Len                        (16U)
#define MCU_SUB_EFUSE_PWR_DELTA1_Msk                        (0xFFFFU << MCU_SUB_EFUSE_PWR_DELTA1_Pos)
#define MCU_SUB_EFUSE_PWR_DELTA1                            MCU_SUB_EFUSE_PWR_DELTA1_Msk

#define MCU_SUB_EFUSE_PWR_DELTA0_Pos                        (0U)
#define MCU_SUB_EFUSE_PWR_DELTA0_Len                        (16U)
#define MCU_SUB_EFUSE_PWR_DELTA0_Msk                        (0xFFFFU << MCU_SUB_EFUSE_PWR_DELTA0_Pos)
#define MCU_SUB_EFUSE_PWR_DELTA0                            MCU_SUB_EFUSE_PWR_DELTA0_Msk

/***************  Bit definition for EFUSE_PWR_DELTA_1 register  ********/
#define MCU_SUB_EFUSE_PWR_DELTA2_Pos                        (0U)
#define MCU_SUB_EFUSE_PWR_DELTA2_Len                        (16U)
#define MCU_SUB_EFUSE_PWR_DELTA2_Msk                        (0xFFFFU << MCU_SUB_EFUSE_PWR_DELTA2_Pos)
#define MCU_SUB_EFUSE_PWR_DELTA2                            MCU_SUB_EFUSE_PWR_DELTA2_Msk

/***************  Bit definition for EFUSE_PWR_CTRL_0 register  ********/
#define MCU_SUB_EFUSE_PWR_CTL0_STP_Pos                      (4U)
#define MCU_SUB_EFUSE_PWR_CTL0_STP_Len                      (1U)
#define MCU_SUB_EFUSE_PWR_CTL0_STP_Msk                      (0x1U << MCU_SUB_EFUSE_PWR_CTL0_STP_Pos)
#define MCU_SUB_EFUSE_PWR_CTL0_STP                          MCU_SUB_EFUSE_PWR_CTL0_STP_Msk

#define MCU_SUB_EFUSE_PWR_CTL0_BGN_Pos                      (2U)
#define MCU_SUB_EFUSE_PWR_CTL0_BGN_Len                      (1U)
#define MCU_SUB_EFUSE_PWR_CTL0_BGN_Msk                      (0x1U << MCU_SUB_EFUSE_PWR_CTL0_BGN_Pos)
#define MCU_SUB_EFUSE_PWR_CTL0_BGN                          MCU_SUB_EFUSE_PWR_CTL0_BGN_Msk

#define MCU_SUB_EFUSE_PWR_CTL0_EN_Pos                       (0U)
#define MCU_SUB_EFUSE_PWR_CTL0_EN_Len                       (1U)
#define MCU_SUB_EFUSE_PWR_CTL0_EN_Msk                       (0x1U << MCU_SUB_EFUSE_PWR_CTL0_EN_Pos)
#define MCU_SUB_EFUSE_PWR_CTL0_EN                           MCU_SUB_EFUSE_PWR_CTL0_EN_Msk

/***************  Bit definition for EFUSE_PWR_CTRL_1 register  ********/
#define MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE_Pos                 (4U)
#define MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE_Len                 (1U)
#define MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE_Msk                 (0x1U << MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE_Pos)
#define MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE                     MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE_Msk

#define MCU_SUB_EFUSE_PWR_CTL0_EN_DONE_Pos                  (0U)
#define MCU_SUB_EFUSE_PWR_CTL0_EN_DONE_Len                  (1U)
#define MCU_SUB_EFUSE_PWR_CTL0_EN_DONE_Msk                  (0x1U << MCU_SUB_EFUSE_PWR_CTL0_EN_DONE_Pos)
#define MCU_SUB_EFUSE_PWR_CTL0_EN_DONE                      MCU_SUB_EFUSE_PWR_CTL0_EN_DONE_Msk

/***************  Bit definition for I2S_CLK_CFG register  ********/
#define MCU_SUB_I2S_CLK_CFG_SRC_CLK_EN_Pos                  (20U)
#define MCU_SUB_I2S_CLK_CFG_SRC_CLK_EN_Len                  (1U)
#define MCU_SUB_I2S_CLK_CFG_SRC_CLK_EN_Msk                  (0x1U << MCU_SUB_I2S_CLK_CFG_SRC_CLK_EN_Pos)
#define MCU_SUB_I2S_CLK_CFG_SRC_CLK_EN                      MCU_SUB_I2S_CLK_CFG_SRC_CLK_EN_Msk

#define MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL_Pos                 (18U)
#define MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL_Len                 (2U)
#define MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL_Msk                 (0x3U << MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL_Pos)
#define MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL                     MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL_Msk

#define MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN_Pos                  (16U)
#define MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN_Len                  (1U)
#define MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN_Msk                  (0x1U << MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN_Pos)
#define MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN                      MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN_Msk

#define MCU_SUB_I2S_CLK_CFG_DIV_CNT_Pos                     (0U)
#define MCU_SUB_I2S_CLK_CFG_DIV_CNT_Len                     (12U)
#define MCU_SUB_I2S_CLK_CFG_DIV_CNT_Msk                     (0xFFFU << MCU_SUB_I2S_CLK_CFG_DIV_CNT_Pos)
#define MCU_SUB_I2S_CLK_CFG_DIV_CNT                         MCU_SUB_I2S_CLK_CFG_DIV_CNT_Msk

/***************  Bit definition for I2S_DMA_MODE_SET register  ********/
#define MCU_SUB_I2S_DMA_MODE_S_SET_Pos                      (4U)
#define MCU_SUB_I2S_DMA_MODE_S_SET_Len                      (1U)
#define MCU_SUB_I2S_DMA_MODE_S_SET_Msk                      (0x1U << MCU_SUB_I2S_DMA_MODE_S_SET_Pos)
#define MCU_SUB_I2S_DMA_MODE_S_SET                          MCU_SUB_I2S_DMA_MODE_S_SET_Msk

#define MCU_SUB_I2S_DMA_MODE_SET_Pos                        (0U)
#define MCU_SUB_I2S_DMA_MODE_SET_Len                        (1U)
#define MCU_SUB_I2S_DMA_MODE_SET_Msk                        (0x1U << MCU_SUB_I2S_DMA_MODE_SET_Pos)
#define MCU_SUB_I2S_DMA_MODE_SET                            MCU_SUB_I2S_DMA_MODE_SET_Msk

/***************  Bit definition for MCU_SUB_REG0 register  ********/
#define MCU_SUB_MCU_SUB_REG0_GDX_REG_WAIT_STATE_Pos         (12U)
#define MCU_SUB_MCU_SUB_REG0_GDX_REG_WAIT_STATE_Len         (2U)
#define MCU_SUB_MCU_SUB_REG0_GDX_REG_WAIT_STATE_Msk         (0x3U << MCU_SUB_MCU_SUB_REG0_GDX_REG_WAIT_STATE_Pos)
#define MCU_SUB_MCU_SUB_REG0_GDX_REG_WAIT_STATE             MCU_SUB_MCU_SUB_REG0_GDX_REG_WAIT_STATE_Msk

#define MCU_SUB_MCU_SUB_REG0_MEM_BOND_OPT_Pos               (4U)
#define MCU_SUB_MCU_SUB_REG0_MEM_BOND_OPT_Len               (2U)
#define MCU_SUB_MCU_SUB_REG0_MEM_BOND_OPT_Msk               (0x3U << MCU_SUB_MCU_SUB_REG0_MEM_BOND_OPT_Pos)
#define MCU_SUB_MCU_SUB_REG0_MEM_BOND_OPT                    MCU_SUB_MCU_SUB_REG0_MEM_BOND_OPT_Msk
#define MCU_SUB_MCU_SUB_REG0_MEM_BOND_OPT_64K               (0x00 << MCU_SUB_MCU_SUB_REG0_MEM_BOND_OPT_Pos)
#define MCU_SUB_MCU_SUB_REG0_MEM_BOND_OPT_128K              (0x01 << MCU_SUB_MCU_SUB_REG0_MEM_BOND_OPT_Pos)
#define MCU_SUB_MCU_SUB_REG0_MEM_BOND_OPT_256K              (0x03 << MCU_SUB_MCU_SUB_REG0_MEM_BOND_OPT_Pos)


/*******************  Bit definition for MCU_SUB_MCU_NMI_CFG register  *******************/
#define MCU_SUB_MCU_NMI_CFG_MCU_NMI_SEL_Pos                 (0U)
#define MCU_SUB_MCU_NMI_CFG_MCU_NMI_SEL_Len                 (10U)
#define MCU_SUB_MCU_NMI_CFG_MCU_NMI_SEL_Msk                 (0x3FFUL << MCU_SUB_MCU_NMI_CFG_MCU_NMI_SEL_Pos)
#define MCU_SUB_MCU_NMI_CFG_MCU_NMI_SEL                     MCU_SUB_MCU_NMI_CFG_MCU_NMI_SEL_Msk

/*******************  Bit definition for MCU_SUB_CPLL_IRQ_CFG register  *******************/
#define MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_EN_Pos               (0U)
#define MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_EN_Len               (1U)
#define MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_EN_Msk               (0x1UL << MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_EN_Pos)
#define MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_EN                   MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_EN_Msk

#define MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_STAT_Pos             (1U)
#define MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_STAT_Len             (1U)
#define MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_STAT_Msk             (0x1UL << MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_STAT_Pos)
#define MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_STAT                 MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_STAT_Msk

#define MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_CLR_Pos              (2U)
#define MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_CLR_Len              (1U)
#define MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_CLR_Msk              (0x1UL << MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_CLR_Pos)
#define MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_CLR                  MCU_SUB_CPLL_IRQ_CFG_DRIFT_IRQ_CLR_Msk

/*******************  Bit definition for MCU_SUB_AON_SW_RST register  *******************/
#define MCU_SUB_AON_SW_RST_PARTIAL_Pos                      (0U)
#define MCU_SUB_AON_SW_RST_PARTIAL_Len                      (1U)
#define MCU_SUB_AON_SW_RST_PARTIAL_Msk                      (0x1UL << MCU_SUB_AON_SW_RST_PARTIAL_Pos)
#define MCU_SUB_AON_SW_RST_PARTIAL                          MCU_SUB_AON_SW_RST_PARTIAL_Msk

#define MCU_SUB_AON_SW_RST_FULL_Pos                         (8U)
#define MCU_SUB_AON_SW_RST_FULL_Len                         (1U)
#define MCU_SUB_AON_SW_RST_FULL_Msk                         (0x1UL << MCU_SUB_AON_SW_RST_FULL_Pos)
#define MCU_SUB_AON_SW_RST_FULL                             MCU_SUB_AON_SW_RST_FULL_Msk

#define MCU_SUB_AON_SW_RST_SET_Pos                          (16U)
#define MCU_SUB_AON_SW_RST_SET_Len                          (16U)
#define MCU_SUB_AON_SW_RST_SET_Msk                          (0xFFFFUL << MCU_SUB_AON_SW_RST_SET_Pos)
#define MCU_SUB_AON_SW_RST_SET                              MCU_SUB_AON_SW_RST_SET_Msk

/*******************  Bit definition for MCU_SUB_APB_TIMER_DBG register  *******************/
#define MCU_SUB_APB_TIMER_DBG_TIMER0_POS                    (0U)
#define MCU_SUB_APB_TIMER_DBG_TIMER0_Len                    (1U)
#define MCU_SUB_APB_TIMER_DBG_TIMER0_Msk                    (0x1UL << MCU_SUB_APB_TIMER_DBG_TIMER0_POS)
#define MCU_SUB_APB_TIMER_DBG_TIMER0                        MCU_SUB_APB_TIMER_DBG_TIMER0_Msk

#define MCU_SUB_APB_TIMER_DBG_TIMER1_POS                    (1U)
#define MCU_SUB_APB_TIMER_DBG_TIMER1_Len                    (1U)
#define MCU_SUB_APB_TIMER_DBG_TIMER1_Msk                    (0x1U << MCU_SUB_APB_TIMER_DBG_TIMER1_POS)
#define MCU_SUB_APB_TIMER_DBG_TIMER1                        MCU_SUB_APB_TIMER_DBG_TIMER1_Msk

#define MCU_SUB_APB_TIMER_DBG_DUAL_TIMER_POS                (2U)
#define MCU_SUB_APB_TIMER_DBG_DUAL_TIMER_Len                (1U)
#define MCU_SUB_APB_TIMER_DBG_DUAL_TIMER_Msk                (0x1U << MCU_SUB_APB_TIMER_DBG_DUAL_TIMER_POS)
#define MCU_SUB_APB_TIMER_DBG_DUAL_TIMER                    MCU_SUB_APB_TIMER_DBG_DUAL_TIMER_Msk

#define MCU_SUB_APB_TIMER_DBG_WDT_POS                       (3U)
#define MCU_SUB_APB_TIMER_DBG_WDT_Len                       (1U)
#define MCU_SUB_APB_TIMER_DBG_WDT_Msk                       (0x1UL << MCU_SUB_APB_TIMER_DBG_WDT_POS)
#define MCU_SUB_APB_TIMER_DBG_WDT                           MCU_SUB_APB_TIMER_DBG_WDT_Msk

/***************  Bit definition for MCU_APB_MON_DBG register  ********/
#define MCU_SUB_APB_MON_DBG_Pos                             (0U)
#define MCU_SUB_APB_MON_DBG_Len                             (1U)
#define MCU_SUB_APB_MON_DBG_Msk                             (0x1U << MCU_SUB_APB_MON_DBG_Pos)
#define MCU_SUB_APB_MON_DBG                                 MCU_SUB_APB_MON_DBG_Msk

/***************  Bit definition for MCU_RELEASE register  ********/
#define MCU_SUB_MCU_RELEASE_Pos                             (0U)
#define MCU_SUB_MCU_RELEASE_Len                             (32U)
#define MCU_SUB_MCU_RELEASE_Msk                             (0xFFFFFFFFU)
#define MCU_SUB_MCU_RELEASE                                 MCU_SUB_MCU_RELEASE_Msk

/***************  Bit definition for FPGA_CTRL_REG register  ********/
#define MCU_SUB_FPGA_CTRL_REG_EXIST_Pos                     (4U)
#define MCU_SUB_FPGA_CTRL_REG_EXIST_Len                     (1U)
#define MCU_SUB_FPGA_CTRL_REG_EXIST_Msk                     (0x1U << MCU_SUB_FPGA_CTRL_REG_EXIST_Pos)
#define MCU_SUB_FPGA_CTRL_REG_EXIST                         MCU_SUB_FPGA_CTRL_REG_EXIST_Msk

#define MCU_SUB_FPGA_CTRL_REG_MUX_SEL_Pos                   (0U)
#define MCU_SUB_FPGA_CTRL_REG_MUX_SEL_Len                   (2U)
#define MCU_SUB_FPGA_CTRL_REG_MUX_SEL_Msk                   (0x3U << MCU_SUB_FPGA_CTRL_REG_MUX_SEL_Pos)
#define MCU_SUB_FPGA_CTRL_REG_MUX_SEL                       MCU_SUB_FPGA_CTRL_REG_MUX_SEL_Msk

/**********************  Bit definition for ST_CALIB_REG register  ***********************************/
#define MCU_SUB_ST_CALIB_REG_STCALIB_CLK_Pos                (28U)
#define MCU_SUB_ST_CALIB_REG_STCALIB_CLK_Len                (1U)
#define MCU_SUB_ST_CALIB_REG_STCALIB_CLK_Msk                (0x1U << MCU_SUB_ST_CALIB_REG_STCALIB_CLK_Pos)
#define MCU_SUB_ST_CALIB_REG_STCALIB_CLK                    MCU_SUB_ST_CALIB_REG_STCALIB_CLK_Msk

#define MCU_SUB_ST_CALIB_REG_STCALIB_Pos                    (0U)
#define MCU_SUB_ST_CALIB_REG_STCALIB_Len                    (26U)
#define MCU_SUB_ST_CALIB_REG_STCALIB_Msk                    (0x7FFFFFFU << MCU_SUB_ST_CALIB_REG_STCALIB_Pos)
#define MCU_SUB_ST_CALIB_REG_STCALIB                        MCU_SUB_ST_CALIB_REG_STCALIB_Msk

/**********************  Bit definition for GPU_DBG register  ***********************************/
#define MCU_SUB_GPU_FLAG_SET_Pos                            (0U)
#define MCU_SUB_GPU_FLAG_SET_Len                            (4U)
#define MCU_SUB_GPU_FLAG_SET_Msk                            (0xFU << MCU_SUB_GPU_FLAG_SET_Pos)
#define MCU_SUB_GPU_FLAG_SET                                MCU_SUB_GPU_FLAG_SET_Msk

#define MCU_SUB_GPU_FLAG_RESET_Pos                          (4U)
#define MCU_SUB_GPU_FLAG_RESET_Len                          (4U)
#define MCU_SUB_GPU_FLAG_RESET_Msk                          (0xFU << MCU_SUB_GPU_FLAG_RESET_Pos)
#define MCU_SUB_GPU_FLAG_RESET                              MCU_SUB_GPU_FLAG_RESET_Msk

#define MCU_SUB_GPU_GP_FLAG_Pos                             (8U)
#define MCU_SUB_GPU_GP_FLAG_Len                             (4U)
#define MCU_SUB_GPU_GP_FLAG_Msk                             (0xFU << MCU_SUB_GPU_GP_FLAG_Pos)
#define MCU_SUB_GPU_GP_FLAG                                 MCU_SUB_GPU_GP_FLAG_Msk

#define MCU_SUB_GPU_ACTIVE_Pos                              (31U)
#define MCU_SUB_GPU_ACTIVE_Len                              (1U)
#define MCU_SUB_GPU_ACTIVE_Msk                              (0x1U << MCU_SUB_GPU_ACTIVE_Pos)
#define MCU_SUB_GPU_ACTIVE                                  MCU_SUB_GPU_ACTIVE_Msk


/* ================================================================================================================= */
/* ================                                        PKC                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for PKC_CTRL register  *******************/
#define PKC_CTRL_EN_POS                                     (0U)
#define PKC_CTRL_EN_Len                                     (1U)
#define PKC_CTRL_EN_Msk                                     (0x1UL << PKC_CTRL_EN_POS)
#define PKC_CTRL_EN                                         PKC_CTRL_EN_Msk

#define PKC_CTRL_START_POS                                  (1U)
#define PKC_CTRL_START_Len                                  (1U)
#define PKC_CTRL_START_Msk                                  (0x1UL << PKC_CTRL_START_POS)
#define PKC_CTRL_START                                      PKC_CTRL_START_Msk

#define PKC_CTRL_SW_CTRL_POS                                (4U)
#define PKC_CTRL_SW_CTRL_Len                                (1U)
#define PKC_CTRL_SW_CTRL_Msk                                (0x1UL << PKC_CTRL_SW_CTRL_POS)
#define PKC_CTRL_SW_CTRL                                    PKC_CTRL_SW_CTRL_Msk

#define PKC_CTRL_RST_POS                                    (8U)
#define PKC_CTRL_RST_Len                                    (1U)
#define PKC_CTRL_RST_Msk                                    (0x1UL << PKC_CTRL_RST_POS)
#define PKC_CTRL_RST                                        PKC_CTRL_RST_Msk

/*******************  Bit definition for PKC_CFG0 register  *******************/
#define PKC_CFG0_K_POINT_POS                                (0U)
#define PKC_CFG0_K_POINT_Len                                (9U)
#define PKC_CFG0_K_POINT_Msk                                (0x1FFUL << PKC_CFG0_K_POINT_POS)
#define PKC_CFG0_K_POINT                                    PKC_CFG0_K_POINT_Msk

#define PKC_CFG0_R_POINT_POS                                (16U)
#define PKC_CFG0_R_POINT_Len                                (9U)
#define PKC_CFG0_R_POINT_Msk                                (0x1FFUL << PKC_CFG0_R_POINT_POS)
#define PKC_CFG0_R_POINT                                    PKC_CFG0_R_POINT_Msk

/*******************  Bit definition for PKC_CFG1 register  *******************/
#define PKC_CFG1_P_POINT_POS                                (0U)
#define PKC_CFG1_P_POINT_Len                                (9U)
#define PKC_CFG1_P_POINT_Msk                                (0x1FFUL << PKC_CFG1_P_POINT_POS)
#define PKC_CFG1_P_POINT                                    PKC_CFG1_P_POINT_Msk

#define PKC_CFG1_R2_POINT_POS                               (16U)
#define PKC_CFG1_R2_POINT_Len                               (9U)
#define PKC_CFG1_R2_POINT_Msk                               (0x1FFUL << PKC_CFG1_R2_POINT_POS)
#define PKC_CFG1_R2_POINT                                   PKC_CFG1_R2_POINT_Msk

/*******************  Bit definition for PKC_CFG2 register  *******************/
#define PKC_CFG2_GX_POINT_POS                               (0U)
#define PKC_CFG2_GX_POINT_Len                               (9U)
#define PKC_CFG2_GX_POINT_Msk                               (0x1FFUL << PKC_CFG2_GX_POINT_POS)
#define PKC_CFG2_GX_POINT                                   PKC_CFG2_GX_POINT_Msk

#define PKC_CFG2_GY_POINT_POS                               (16U)
#define PKC_CFG2_GY_POINT_Len                               (9U)
#define PKC_CFG2_GY_POINT_Msk                               (0x1FFUL << PKC_CFG2_GY_POINT_POS)
#define PKC_CFG2_GY_POINT                                   PKC_CFG2_GY_POINT_Msk

/*******************  Bit definition for PKC_CFG3 register  *******************/
#define PKC_CFG3_GZ_POINT_POS                               (0U)
#define PKC_CFG3_GZ_POINT_Len                               (9U)
#define PKC_CFG3_GZ_POINT_Msk                               (0x1FFUL << PKC_CFG3_GZ_POINT_POS)
#define PKC_CFG3_GZ_POINT                                   PKC_CFG3_GZ_POINT_Msk

#define PKC_CFG3_R0X_POINT_POS                              (16U)
#define PKC_CFG3_R0X_POINT_Len                              (9U)
#define PKC_CFG3_R0X_POINT_Msk                              (0x1FFUL << PKC_CFG3_R0X_POINT_POS)
#define PKC_CFG3_R0X_POINT                                  PKC_CFG3_R0X_POINT_Msk

/*******************  Bit definition for PKC_CFG4 register  *******************/
#define PKC_CFG4_R0Y_POINT_POS                              (0U)
#define PKC_CFG4_R0Y_POINT_Len                              (9U)
#define PKC_CFG4_R0Y_POINT_Msk                              (0x1FFUL << PKC_CFG4_R0Y_POINT_POS)
#define PKC_CFG4_R0Y_POINT                                  PKC_CFG4_R0Y_POINT_Msk

#define PKC_CFG4_R0Z_POINT_POS                              (16U)
#define PKC_CFG4_R0Z_POINT_Len                              (9U)
#define PKC_CFG4_R0Z_POINT_Msk                              (0x1FFUL << PKC_CFG4_R0Z_POINT_POS)
#define PKC_CFG4_R0Z_POINT                                  PKC_CFG4_R0Z_POINT_Msk

/*******************  Bit definition for PKC_CFG5 register  *******************/
#define PKC_CFG5_R1X_POINT_POS                              (0U)
#define PKC_CFG5_R1X_POINT_Len                              (9U)
#define PKC_CFG5_R1X_POINT_Msk                              (0x1FFUL << PKC_CFG5_R1X_POINT_POS)
#define PKC_CFG5_R1X_POINT                                  PKC_CFG5_R1X_POINT_Msk

#define PKC_CFG5_R1Y_POINT_POS                              (16U)
#define PKC_CFG5_R1Y_POINT_Len                              (9U)
#define PKC_CFG5_R1Y_POINT_Msk                              (0x1FFUL << PKC_CFG5_R1Y_POINT_POS)
#define PKC_CFG5_R1Y_POINT                                  PKC_CFG5_R1Y_POINT_Msk

/*******************  Bit definition for PKC_CFG6 register  *******************/
#define PKC_CFG6_R1Z_POINT_POS                              (0U)
#define PKC_CFG6_R1Z_POINT_Len                              (9U)
#define PKC_CFG6_R1Z_POINT_Msk                              (0x1FFUL << PKC_CFG6_R1Z_POINT_POS)
#define PKC_CFG6_R1Z_POINT                                  PKC_CFG6_R1Z_POINT_Msk

#define PKC_CFG6_TEMP1_POINT_POS                            (16U)
#define PKC_CFG6_TEMP1_POINT_Len                            (9U)
#define PKC_CFG6_TEMP1_POINT_Msk                            (0x1FFUL << PKC_CFG6_TEMP1_POINT_POS)
#define PKC_CFG6_TEMP1_POINT                                PKC_CFG6_TEMP1_POINT_Msk

/*******************  Bit definition for PKC_CFG7 register  *******************/
#define PKC_CFG7_TEMP2_POINT_POS                            (0U)
#define PKC_CFG7_TEMP2_POINT_Len                            (9U)
#define PKC_CFG7_TEMP2_POINT_Msk                            (0x1FFUL << PKC_CFG7_TEMP2_POINT_POS)
#define PKC_CFG7_TEMP2_POINT                                PKC_CFG7_TEMP2_POINT_Msk

#define PKC_CFG7_TEMP3_POINT_POS                            (16U)
#define PKC_CFG7_TEMP3_POINT_Len                            (9U)
#define PKC_CFG7_TEMP3_POINT_Msk                            (0x1FFUL << PKC_CFG7_TEMP3_POINT_POS)
#define PKC_CFG7_TEMP3_POINT                                PKC_CFG7_TEMP3_POINT_Msk

/*******************  Bit definition for PKC_CFG8 register  *******************/
#define PKC_CFG8_TEMP4_POINT_POS                            (0U)
#define PKC_CFG8_TEMP4_POINT_Len                            (9U)
#define PKC_CFG8_TEMP4_POINT_Msk                            (0x1FFUL << PKC_CFG8_TEMP4_POINT_POS)
#define PKC_CFG8_TEMP4_POINT                                PKC_CFG8_TEMP4_POINT_Msk

#define PKC_CFG8_TEMP5_POINT_POS                            (16U)
#define PKC_CFG8_TEMP5_POINT_Len                            (9U)
#define PKC_CFG8_TEMP5_POINT_Msk                            (0x1FFUL << PKC_CFG8_TEMP5_POINT_POS)
#define PKC_CFG8_TEMP5_POINT                                PKC_CFG8_TEMP5_POINT_Msk

/*******************  Bit definition for PKC_CFG9 register  *******************/
#define PKC_CFG9_TEMP6_POINT_POS                            (0U)
#define PKC_CFG9_TEMP6_POINT_Len                            (9U)
#define PKC_CFG9_TEMP6_POINT_Msk                            (0x1FFUL << PKC_CFG9_TEMP6_POINT_POS)
#define PKC_CFG9_TEMP6_POINT                                PKC_CFG9_TEMP6_POINT_Msk

#define PKC_CFG9_CONT1_POINT_POS                            (16U)
#define PKC_CFG9_CONT1_POINT_Len                            (9U)
#define PKC_CFG9_CONT1_POINT_Msk                            (0x1FFUL << PKC_CFG9_CONT1_POINT_POS)
#define PKC_CFG9_CONT1_POINT                                PKC_CFG9_CONT1_POINT_Msk

/*******************  Bit definition for PKC_CFG10 register  *******************/
#define PKC_CFG10_X1_POINT_POS                              (0U)
#define PKC_CFG10_X1_POINT_Len                              (9U)
#define PKC_CFG10_X1_POINT_Msk                              (0x1FFUL << PKC_CFG10_X1_POINT_POS)
#define PKC_CFG10_X1_POINT                                  PKC_CFG10_X1_POINT_Msk

#define PKC_CFG10_X2_POINT_POS                              (16U)
#define PKC_CFG10_X2_POINT_Len                              (9U)
#define PKC_CFG10_X2_POINT_Msk                              (0x1FFUL << PKC_CFG10_X2_POINT_POS)
#define PKC_CFG10_X2_POINT                                  PKC_CFG10_X2_POINT_Msk

/*******************  Bit definition for PKC_CFG11 register  *******************/
#define PKC_CFG11_MIT_POINT_POS                             (0U)
#define PKC_CFG11_MIT_POINT_Len                             (9U)
#define PKC_CFG11_MIT_POINT_Msk                             (0x1FFUL << PKC_CFG11_MIT_POINT_POS)
#define PKC_CFG11_MIT_POINT                                 PKC_CFG11_MIT_POINT_Msk

#define PKC_CFG11_KT_POINT_POS                              (16U)
#define PKC_CFG11_KT_POINT_Len                              (9U)
#define PKC_CFG11_KT_POINT_Msk                              (0x1FFUL << PKC_CFG11_KT_POINT_POS)
#define PKC_CFG11_KT_POINT                                  PKC_CFG11_KT_POINT_Msk

/*******************  Bit definition for PKC_CFG12 register  *******************/
#define PKC_CFG12_A_POINT_POS                               (0U)
#define PKC_CFG12_A_POINT_Len                               (9U)
#define PKC_CFG12_A_POINT_Msk                               (0x1FFUL << PKC_CFG12_A_POINT_POS)
#define PKC_CFG12_A_POINT                                   PKC_CFG12_A_POINT_Msk

#define PKC_CFG12_B_POINT_POS                               (16U)
#define PKC_CFG12_B_POINT_Len                               (9U)
#define PKC_CFG12_B_POINT_Msk                               (0x1FFUL << PKC_CFG12_B_POINT_POS)
#define PKC_CFG12_B_POINT                                   PKC_CFG12_B_POINT_Msk

/*******************  Bit definition for PKC_CFG13 register  *******************/
#define PKC_CFG13_CONSTQ_POS                                (0U)
#define PKC_CFG13_CONSTQ_Len                                (32U)
#define PKC_CFG13_CONSTQ_Msk                                (0xFFFFFFFFUL << PKC_CFG13_CONSTQ_POS)
#define PKC_CFG13_CONSTQ                                    PKC_CFG13_CONSTQ_Msk

/*******************  Bit definition for PKC_SW_CTRL register  *******************/
#define PKC_SW_CTRL_START_POS                               (0U)
#define PKC_SW_CTRL_START_Len                               (1U)
#define PKC_SW_CTRL_START_Msk                               (0x1UL << PKC_SW_CTRL_START_POS)
#define PKC_SW_CTRL_START                                   PKC_SW_CTRL_START_Msk

#define PKC_SW_CTRL_MODE_POS                                (4U)
#define PKC_SW_CTRL_MODE_Len                                (3U)
#define PKC_SW_CTRL_MODE_Msk                                (0x7UL << PKC_SW_CTRL_MODE_POS)
#define PKC_SW_CTRL_MODE                                    PKC_SW_CTRL_MODE_Msk

#define PKC_SW_CTRL_DM_EN_POS                               (8U)
#define PKC_SW_CTRL_DM_EN_Len                               (1U)
#define PKC_SW_CTRL_DM_EN_Msk                               (0x1UL << PKC_SW_CTRL_DM_EN_POS)
#define PKC_SW_CTRL_DM_EN                                   PKC_SW_CTRL_DM_EN_Msk

#define PKC_SW_CTRL_RCG_EN_POS                              (9U)
#define PKC_SW_CTRL_RCG_EN_Len                              (1U)
#define PKC_SW_CTRL_RCG_EN_Msk                              (0x1UL << PKC_SW_CTRL_RCG_EN_POS)
#define PKC_SW_CTRL_RCG_EN                                  PKC_SW_CTRL_RCG_EN_Msk

/*******************  Bit definition for PKC_SW_CFG0 register  *******************/
#define PKC_SW_CFG0_MMA_POINT_POS                           (0U)
#define PKC_SW_CFG0_MMA_POINT_Len                           (9U)
#define PKC_SW_CFG0_MMA_POINT_Msk                           (0x1FFUL << PKC_SW_CFG0_MMA_POINT_POS)
#define PKC_SW_CFG0_MMA_POINT                               PKC_SW_CFG0_MMA_POINT_Msk

#define PKC_SW_CFG0_MMB_POINT_POS                           (16U)
#define PKC_SW_CFG0_MMB_POINT_Len                           (9U)
#define PKC_SW_CFG0_MMB_POINT_Msk                           (0x1FFUL << PKC_SW_CFG0_MMB_POINT_POS)
#define PKC_SW_CFG0_MMB_POINT                               PKC_SW_CFG0_MMB_POINT_Msk

/*******************  Bit definition for PKC_SW_CFG1 register  *******************/
#define PKC_SW_CFG1_MMP_POINT_POS                           (0U)
#define PKC_SW_CFG1_MMP_POINT_Len                           (9U)
#define PKC_SW_CFG1_MMP_POINT_Msk                           (0x1FFUL << PKC_SW_CFG1_MMP_POINT_POS)
#define PKC_SW_CFG1_MMP_POINT                               PKC_SW_CFG1_MMP_POINT_Msk

#define PKC_SW_CFG1_MMC_POINT_POS                           (16U)
#define PKC_SW_CFG1_MMC_POINT_Len                           (9U)
#define PKC_SW_CFG1_MMC_POINT_Msk                           (0x1FFUL << PKC_SW_CFG1_MMC_POINT_POS)
#define PKC_SW_CFG1_MMC_POINT                               PKC_SW_CFG1_MMC_POINT_Msk

/*******************  Bit definition for PKC_SW_CFG2 register  *******************/
#define PKC_SW_CFG2_MASA_POINT_POS                          (0U)
#define PKC_SW_CFG2_MASA_POINT_Len                          (9U)
#define PKC_SW_CFG2_MASA_POINT_Msk                          (0x1FFUL << PKC_SW_CFG2_MASA_POINT_POS)
#define PKC_SW_CFG2_MASA_POINT                              PKC_SW_CFG2_MASA_POINT_Msk

#define PKC_SW_CFG2_MASB_POINT_POS                          (16U)
#define PKC_SW_CFG2_MASB_POINT_Len                          (9U)
#define PKC_SW_CFG2_MASB_POINT_Msk                          (0x1FFUL << PKC_SW_CFG2_MASB_POINT_POS)
#define PKC_SW_CFG2_MASB_POINT                              PKC_SW_CFG2_MASB_POINT_Msk

/*******************  Bit definition for PKC_SW_CFG3 register  *******************/
#define PKC_SW_CFG3_MASP_POINT_POS                          (0U)
#define PKC_SW_CFG3_MASP_POINT_Len                          (9U)
#define PKC_SW_CFG3_MASP_POINT_Msk                          (0x1FFUL << PKC_SW_CFG3_MASP_POINT_POS)
#define PKC_SW_CFG3_MASP_POINT                              PKC_SW_CFG3_MASP_POINT_Msk

#define PKC_SW_CFG3_MASC_POINT_POS                          (16U)
#define PKC_SW_CFG3_MASC_POINT_Len                          (9U)
#define PKC_SW_CFG3_MASC_POINT_Msk                          (0x1FFUL << PKC_SW_CFG3_MASC_POINT_POS)
#define PKC_SW_CFG3_MASC_POINT                              PKC_SW_CFG3_MASC_POINT_Msk

/*******************  Bit definition for PKC_SW_CFG4 register  *******************/
#define PKC_SW_CFG4_MIU_POINT_POS                           (0U)
#define PKC_SW_CFG4_MIU_POINT_Len                           (9U)
#define PKC_SW_CFG4_MIU_POINT_Msk                           (0x1FFUL << PKC_SW_CFG4_MIU_POINT_POS)
#define PKC_SW_CFG4_MIU_POINT                               PKC_SW_CFG4_MIU_POINT_Msk

#define PKC_SW_CFG4_MIV_POINT_POS                           (16U)
#define PKC_SW_CFG4_MIV_POINT_Len                           (9U)
#define PKC_SW_CFG4_MIV_POINT_Msk                           (0x1FFUL << PKC_SW_CFG4_MIV_POINT_POS)
#define PKC_SW_CFG4_MIV_POINT                               PKC_SW_CFG4_MIV_POINT_Msk

/*******************  Bit definition for PKC_SW_CFG5 register  *******************/
#define PKC_SW_CFG5_MIX1_POINT_POS                          (0U)
#define PKC_SW_CFG5_MIX1_POINT_Len                          (9U)
#define PKC_SW_CFG5_MIX1_POINT_Msk                          (0x1FFUL << PKC_SW_CFG5_MIX1_POINT_POS)
#define PKC_SW_CFG5_MIX1_POINT                              PKC_SW_CFG5_MIX1_POINT_Msk

#define PKC_SW_CFG5_MIX2_POINT_POS                          (16U)
#define PKC_SW_CFG5_MIX2_POINT_Len                          (9U)
#define PKC_SW_CFG5_MIX2_POINT_Msk                          (0x1FFUL << PKC_SW_CFG5_MIX2_POINT_POS)
#define PKC_SW_CFG5_MIX2_POINT                              PKC_SW_CFG5_MIX2_POINT_Msk

/*******************  Bit definition for PKC_SW_CFG6 register  *******************/
#define PKC_SW_CFG6_MIT_POINT_POS                           (0U)
#define PKC_SW_CFG6_MIT_POINT_Len                           (9U)
#define PKC_SW_CFG6_MIT_POINT_Msk                           (0x1FFUL << PKC_SW_CFG6_MIT_POINT_POS)
#define PKC_SW_CFG6_MIT_POINT                               PKC_SW_CFG6_MIT_POINT_Msk

/*******************  Bit definition for PKC_SW_CFG7 register  *******************/
#define PKC_SW_CFG7_LEN_POS                                 (0U)
#define PKC_SW_CFG7_LEN_Len                                 (9U)
#define PKC_SW_CFG7_LEN_Msk                                 (0x1FFUL << PKC_SW_CFG7_LEN_POS)
#define PKC_SW_CFG7_LEN                                     PKC_SW_CFG7_LEN_Msk

/*******************  Bit definition for PKC_SW_CFG8 register  *******************/
#define PKC_SW_CFG8_MIK_OUT_POS                             (0U)
#define PKC_SW_CFG8_MIK_OUT_Len                             (13U)
#define PKC_SW_CFG8_MIK_OUT_Msk                             (0x1FFFUL << PKC_SW_CFG8_MIK_OUT_POS)
#define PKC_SW_CFG8_MIK_OUT                                 PKC_SW_CFG8_MIK_OUT_Msk

/*******************  Bit definition for PKC_SW_CFG9 register  *******************/
#define PKC_SW_CFG9_RDM_SEED_POS                            (0U)
#define PKC_SW_CFG9_RDM_SEED_Len                            (32U)
#define PKC_SW_CFG9_RDM_SEED_Msk                            (0xFFFFFFFFUL << PKC_SW_CFG9_RDM_SEED_POS)
#define PKC_SW_CFG9_RDM_SEED                                PKC_SW_CFG9_RDM_SEED_Msk

/*******************  Bit definition for PKC_SW_CFG10 register  *******************/
#define PKC_SW_CFG10_BMA_POINT_POS                          (0U)
#define PKC_SW_CFG10_BMA_POINT_Len                          (9U)
#define PKC_SW_CFG10_BMA_POINT_Msk                          (0x1FFUL << PKC_SW_CFG10_BMA_POINT_POS)
#define PKC_SW_CFG10_BMA_POINT                              PKC_SW_CFG10_BMA_POINT_Msk

#define PKC_SW_CFG10_BMB_POINT_POS                          (16U)
#define PKC_SW_CFG10_BMB_POINT_Len                          (9U)
#define PKC_SW_CFG10_BMB_POINT_Msk                          (0x1FFUL << PKC_SW_CFG10_BMB_POINT_POS)
#define PKC_SW_CFG10_BMB_POINT                              PKC_SW_CFG10_BMB_POINT_Msk

/*******************  Bit definition for PKC_SW_CFG11 register  *******************/
#define PKC_SW_CFG11_BMC_POINT_POS                          (0U)
#define PKC_SW_CFG11_BMC_POINT_Len                          (9U)
#define PKC_SW_CFG11_BMC_POINT_Msk                          (0x1FFUL << PKC_SW_CFG11_BMC_POINT_POS)
#define PKC_SW_CFG11_BMC_POINT                              PKC_SW_CFG11_BMC_POINT_Msk

#define PKC_SW_CFG11_BAA_POINT_POS                          (16U)
#define PKC_SW_CFG11_BAA_POINT_Len                          (9U)
#define PKC_SW_CFG11_BAA_POINT_Msk                          (0x1FFUL << PKC_SW_CFG11_BAA_POINT_POS)
#define PKC_SW_CFG11_BAA_POINT                              PKC_SW_CFG11_BAA_POINT_Msk

/*******************  Bit definition for PKC_SW_CFG12 register  *******************/
#define PKC_SW_CFG12_BAB_POINT_POS                          (0U)
#define PKC_SW_CFG12_BAB_POINT_Len                          (9U)
#define PKC_SW_CFG12_BAB_POINT_Msk                          (0x1FFUL << PKC_SW_CFG12_BAB_POINT_POS)
#define PKC_SW_CFG12_BAB_POINT                              PKC_SW_CFG12_BAB_POINT_Msk

#define PKC_SW_CFG12_BAC_POINT_POS                          (16U)
#define PKC_SW_CFG12_BAC_POINT_Len                          (9U)
#define PKC_SW_CFG12_BAC_POINT_Msk                          (0x1FFUL << PKC_SW_CFG12_BAC_POINT_POS)
#define PKC_SW_CFG12_BAC_POINT                              PKC_SW_CFG12_BAC_POINT_Msk

/*******************  Bit definition for PKC_SW_CFG13 register  *******************/
#define PKC_SW_CFG13_RCG_SEED_POS                           (0U)
#define PKC_SW_CFG13_RCG_SEED_Len                           (32U)
#define PKC_SW_CFG13_RCG_SEED_Msk                           (0xFFFFFFFFUL << PKC_SW_CFG13_RCG_SEED_POS)
#define PKC_SW_CFG13_RCG_SEED                               PKC_SW_CFG13_RCG_SEED_Msk

/*******************  Bit definition for PKC_INT_STAT register  *******************/
#define PKC_INT_STAT_CPLT_INT_FLAG_POS                      (0U)
#define PKC_INT_STAT_CPLT_INT_FLAG_Len                      (1U)
#define PKC_INT_STAT_CPLT_INT_FLAG_Msk                      (0x1UL << PKC_INT_STAT_CPLT_INT_FLAG_POS)
#define PKC_INT_STAT_CPLT_INT_FLAG                          PKC_INT_STAT_CPLT_INT_FLAG_Msk

#define PKC_INT_STAT_ERR_INT_FLAG_POS                       (1U)
#define PKC_INT_STAT_ERR_INT_FLAG_Len                       (1U)
#define PKC_INT_STAT_ERR_INT_FLAG_Msk                       (0x1UL << PKC_INT_STAT_ERR_INT_FLAG_POS)
#define PKC_INT_STAT_ERR_INT_FLAG                           PKC_INT_STAT_ERR_INT_FLAG_Msk

#define PKC_INT_STAT_BIAO_INT_FLAG_POS                      (2U)
#define PKC_INT_STAT_BIAO_INT_FLAG_Len                      (1U)
#define PKC_INT_STAT_BIAO_INT_FLAG_Msk                      (0x1UL << PKC_INT_STAT_BIAO_INT_FLAG_POS)
#define PKC_INT_STAT_BIAO_INT_FLAG                          PKC_INT_STAT_BIAO_INT_FLAG_Msk

/*******************  Bit definition for PKC_INT_EN register  *******************/
#define PKC_INT_EN_CPLT_INT_EN_POS                          (0U)
#define PKC_INT_EN_CPLT_INT_EN_Len                          (1U)
#define PKC_INT_EN_CPLT_INT_EN_Msk                          (0x1UL << PKC_INT_EN_CPLT_INT_EN_POS)
#define PKC_INT_EN_CPLT_INT_EN                              PKC_INT_EN_CPLT_INT_EN_Msk

#define PKC_INT_EN_ERR_INT_EN_POS                           (1U)
#define PKC_INT_EN_ERR_INT_EN_Len                           (1U)
#define PKC_INT_EN_ERR_INT_EN_Msk                           (0x1UL << PKC_INT_EN_ERR_INT_EN_POS)
#define PKC_INT_EN_ERR_INT_EN                               PKC_INT_EN_ERR_INT_EN_Msk

#define PKC_INT_EN_BIAO_INT_EN_POS                          (2U)
#define PKC_INT_EN_BIAO_INT_EN_Len                          (1U)
#define PKC_INT_EN_BIAO_INT_EN_Msk                          (0x1UL << PKC_INT_EN_BIAO_INT_EN_POS)
#define PKC_INT_EN_BIAO_INT_EN                              PKC_INT_EN_BIAO_INT_EN_Msk

/*******************  Bit definition for PKC_STAT register  *******************/
#define PKC_STAT_BUSY_POS                                   (0U)
#define PKC_STAT_BUSY_Len                                   (1U)
#define PKC_STAT_BUSY_Msk                                   (0x1UL << PKC_STAT_BUSY_POS)
#define PKC_STAT_BUSY                                       PKC_STAT_BUSY_Msk


/* ================================================================================================================= */
/* ================                                        PWM                                      ================ */
/* ================================================================================================================= */

/*******************  Bit definition for PWM_MODE register  *******************/
#define PWM_MODE_EN_Pos                                     (0U)
#define PWM_MODE_EN_Len                                     (1U)
#define PWM_MODE_EN_Msk                                     (0x1U << PWM_MODE_EN_Pos)
#define PWM_MODE_EN                                         PWM_MODE_EN_Msk

#define PWM_MODE_PAUSE_Pos                                  (1U)
#define PWM_MODE_PAUSE_Len                                  (1U)
#define PWM_MODE_PAUSE_Msk                                  (0x1U << PWM_MODE_PAUSE_Pos)
#define PWM_MODE_PAUSE                                      PWM_MODE_PAUSE_Msk

#define PWM_MODE_BREATHEN_Pos                               (2U)
#define PWM_MODE_BREATHEN_Len                               (1U)
#define PWM_MODE_BREATHEN_Msk                               (0x1U << PWM_MODE_BREATHEN_Pos)
#define PWM_MODE_BREATHEN                                   PWM_MODE_BREATHEN_Msk

#define PWM_MODE_DPENA_Pos                                  (3U)
#define PWM_MODE_DPENA_Len                                  (1U)
#define PWM_MODE_DPENA_Msk                                  (0x1U << PWM_MODE_DPENA_Pos)
#define PWM_MODE_DPENA                                      PWM_MODE_DPENA_Msk

#define PWM_MODE_DPENB_Pos                                  (4U)
#define PWM_MODE_DPENB_Len                                  (1U)
#define PWM_MODE_DPENB_Msk                                  (0x1U << PWM_MODE_DPENB_Pos)
#define PWM_MODE_DPENB                                      PWM_MODE_DPENB_Msk

#define PWM_MODE_DPENC_Pos                                  (5U)
#define PWM_MODE_DPENC_Len                                  (1U)
#define PWM_MODE_DPENC_Msk                                  (0x1U << PWM_MODE_DPENC_Pos)
#define PWM_MODE_DPENC                                      PWM_MODE_DPENC_Msk

#define PWM_MODE_FLICKER_PAUSE_LEVEL_A_Pos                  (6U)
#define PWM_MODE_FLICKER_PAUSE_LEVEL_A_Len                  (1U)
#define PWM_MODE_FLICKER_PAUSE_LEVEL_A_Msk                  (0x1U << PWM_MODE_FLICKER_PAUSE_LEVEL_A_Pos)
#define PWM_MODE_FLICKER_PAUSE_LEVEL_A                      PWM_MODE_FLICKER_PAUSE_LEVEL_A_Msk

#define PWM_MODE_FLICKER_PAUSE_LEVEL_B_Pos                  (7U)
#define PWM_MODE_FLICKER_PAUSE_LEVEL_B_Len                  (1U)
#define PWM_MODE_FLICKER_PAUSE_LEVEL_B_Msk                  (0x1U << PWM_MODE_FLICKER_PAUSE_LEVEL_B_Pos)
#define PWM_MODE_FLICKER_PAUSE_LEVEL_B                      PWM_MODE_FLICKER_PAUSE_LEVEL_B_Msk

#define PWM_MODE_FLICKER_PAUSE_LEVEL_C_Pos                  (8U)
#define PWM_MODE_FLICKER_PAUSE_LEVEL_C_Len                  (1U)
#define PWM_MODE_FLICKER_PAUSE_LEVEL_C_Msk                  (0x1U << PWM_MODE_FLICKER_PAUSE_LEVEL_C_Pos)
#define PWM_MODE_FLICKER_PAUSE_LEVEL_C                      PWM_MODE_FLICKER_PAUSE_LEVEL_C_Msk

#define PWM_MODE_BREATH_PAUSE_LEVEL_Pos                     (9U)
#define PWM_MODE_BREATH_PAUSE_LEVEL_Len                     (1U)
#define PWM_MODE_BREATH_PAUSE_LEVEL_Msk                     (0x1U << PWM_MODE_BREATH_PAUSE_LEVEL_Pos)
#define PWM_MODE_BREATH_PAUSE_LEVEL                         PWM_MODE_BREATH_PAUSE_LEVEL_Msk

/*******************  Bit definition for PWM_UPDATE register  *****************/
#define PWM_UPDATE_SAG_Pos                                  (0U)
#define PWM_UPDATE_SAG_Len                                  (1U)
#define PWM_UPDATE_SAG_Msk                                  (0x1U << PWM_UPDATE_SAG_Pos)
#define PWM_UPDATE_SAG                                      PWM_UPDATE_SAG_Msk

#define PWM_UPDATE_SA_Pos                                   (1U)
#define PWM_UPDATE_SA_Len                                   (1U)
#define PWM_UPDATE_SA_Msk                                   (0x1U << PWM_UPDATE_SA_Pos)
#define PWM_UPDATE_SA                                       PWM_UPDATE_SA_Msk

#define PWM_UPDATE_SSPRD_Pos                                (8U)
#define PWM_UPDATE_SSPRD_Len                                (1U)
#define PWM_UPDATE_SSPRD_Msk                                (0x1U << PWM_UPDATE_SSPRD_Pos)
#define PWM_UPDATE_SSPRD                                    PWM_UPDATE_SSPRD_Msk

#define PWM_UPDATE_SSCMPA0_Pos                              (9U)
#define PWM_UPDATE_SSCMPA0_Len                              (1U)
#define PWM_UPDATE_SSCMPA0_Msk                              (0x1U << PWM_UPDATE_SSCMPA0_Pos)
#define PWM_UPDATE_SSCMPA0                                  PWM_UPDATE_SSCMPA0_Msk

#define PWM_UPDATE_SSCMPA1_Pos                              (10U)
#define PWM_UPDATE_SSCMPA1_Len                              (1U)
#define PWM_UPDATE_SSCMPA1_Msk                              (0x1U << PWM_UPDATE_SSCMPA1_Pos)
#define PWM_UPDATE_SSCMPA1                                  PWM_UPDATE_SSCMPA1_Msk

#define PWM_UPDATE_SSCMPB0_Pos                              (11U)
#define PWM_UPDATE_SSCMPB0_Len                              (1U)
#define PWM_UPDATE_SSCMPB0_Msk                              (0x1U << PWM_UPDATE_SSCMPB0_Pos)
#define PWM_UPDATE_SSCMPB0                                  PWM_UPDATE_SSCMPB0_Msk

#define PWM_UPDATE_SSCMPB1_Pos                              (12U)
#define PWM_UPDATE_SSCMPB1_Len                              (1U)
#define PWM_UPDATE_SSCMPB1_Msk                              (0x1U << PWM_UPDATE_SSCMPB1_Pos)
#define PWM_UPDATE_SSCMPB1                                  PWM_UPDATE_SSCMPB1_Msk

#define PWM_UPDATE_SSCMPC0_Pos                              (13U)
#define PWM_UPDATE_SSCMPC0_Len                              (1U)
#define PWM_UPDATE_SSCMPC0_Msk                              (0x1U << PWM_UPDATE_SSCMPC0_Pos)
#define PWM_UPDATE_SSCMPC0                                  PWM_UPDATE_SSCMPC0_Msk

#define PWM_UPDATE_SSCMPC1_Pos                              (14U)
#define PWM_UPDATE_SSCMPC1_Len                              (1U)
#define PWM_UPDATE_SSCMPC1_Msk                              (0x1U << PWM_UPDATE_SSCMPC1_Pos)
#define PWM_UPDATE_SSCMPC1                                  PWM_UPDATE_SSCMPC1_Msk

#define PWM_UPDATE_SSBRPRD_Pos                              (15U)
#define PWM_UPDATE_SSBRPRD_Len                              (1U)
#define PWM_UPDATE_SSBRPRD_Msk                              (0x1U << PWM_UPDATE_SSBRPRD_Pos)
#define PWM_UPDATE_SSBRPRD                                  PWM_UPDATE_SSBRPRD_Msk

#define PWM_UPDATE_SSHOLD_Pos                               (16U)
#define PWM_UPDATE_SSHOLD_Len                               (1U)
#define PWM_UPDATE_SSHOLD_Msk                               (0x1U << PWM_UPDATE_SSHOLD_Pos)
#define PWM_UPDATE_SSHOLD                                   PWM_UPDATE_SSHOLD_Msk

#define PWM_UPDATE_SSAQCTRL_Pos                             (17U)
#define PWM_UPDATE_SSAQCTRL_Len                             (1U)
#define PWM_UPDATE_SSAQCTRL_Msk                             (0x1U << PWM_UPDATE_SSAQCTRL_Pos)
#define PWM_UPDATE_SSAQCTRL                                 PWM_UPDATE_SSAQCTRL_Msk

/*******************  Bit definition for PWM_PRD register  ********************/
#define PWM_PRD_PRD_Pos                                     (0U)
#define PWM_PRD_PRD_Len                                     (32U)
#define PWM_PRD_PRD_Msk                                     (0xFFFFFFFFU)
#define PWM_PRD_PRD                                         PWM_PRD_PRD_Msk

/*******************  Bit definition for PWM_CMPA0 register  ******************/
#define PWM_CMPA0_CMPA0_Pos                                 (0U)
#define PWM_CMPA0_CMPA0_Len                                 (32U)
#define PWM_CMPA0_CMPA0_Msk                                 (0xFFFFFFFFU)
#define PWM_CMPA0_CMPA0                                     PWM_CMPA0_CMPA0_Msk

/*******************  Bit definition for PWM_CMPA1 register  ******************/
#define PWM_CMPA1_CMPA1_Pos                                 (0U)
#define PWM_CMPA1_CMPA1_Len                                 (32U)
#define PWM_CMPA1_CMPA1_Msk                                 (0xFFFFFFFFU)
#define PWM_CMPA1_CMPA1                                     PWM_CMPA1_CMPA1_Msk

/*******************  Bit definition for PWM_CMPB0 register  ******************/
#define PWM_CMPB0_CMPB0_Pos                                 (0U)
#define PWM_CMPB0_CMPB0_Len                                 (32U)
#define PWM_CMPB0_CMPB0_Msk                                 (0xFFFFFFFFU)
#define PWM_CMPB0_CMPB0                                     PWM_CMPB0_CMPB0_Msk

/*******************  Bit definition for PWM_CMPB1 register  ******************/
#define PWM_CMPB1_CMPB1_Pos                                 (0U)
#define PWM_CMPB1_CMPB1_Len                                 (32U)
#define PWM_CMPB1_CMPB1_Msk                                 (0xFFFFFFFFU)
#define PWM_CMPB1_CMPB1                                     PWM_CMPB1_CMPB1_Msk

/*******************  Bit definition for PWM_CMPC0 register  ******************/
#define PWM_CMPC0_CMPC0_Pos                                 (0U)
#define PWM_CMPC0_CMPC0_Len                                 (32U)
#define PWM_CMPC0_CMPC0_Msk                                 (0xFFFFFFFFU)
#define PWM_CMPC0_CMPC0                                     PWM_CMPC0_CMPC0_Msk

/*******************  Bit definition for PWM_CMPC1 register  ******************/
#define PWM_CMPC1_CMPC1_Pos                                 (0U)
#define PWM_CMPC1_CMPC1_Len                                 (32U)
#define PWM_CMPC1_CMPC1_Msk                                 (0xFFFFFFFFU)
#define PWM_CMPC1_CMPC1                                     PWM_CMPC1_CMPC1_Msk

/*******************  Bit definition for PWM_AQCTRL register  *****************/
#define PWM_AQCTRL_A0_Pos                                   (0U)
#define PWM_AQCTRL_A0_Len                                   (2U)
#define PWM_AQCTRL_A0_Msk                                   (0x3U << PWM_AQCTRL_A0_Pos)
#define PWM_AQCTRL_A0                                       PWM_AQCTRL_A0_Msk

#define PWM_AQCTRL_A1_Pos                                   (2U)
#define PWM_AQCTRL_A1_Len                                   (2U)
#define PWM_AQCTRL_A1_Msk                                   (0x3U << PWM_AQCTRL_A1_Pos)
#define PWM_AQCTRL_A1                                       PWM_AQCTRL_A1_Msk

#define PWM_AQCTRL_B0_Pos                                   (4U)
#define PWM_AQCTRL_B0_Len                                   (2U)
#define PWM_AQCTRL_B0_Msk                                   (0x3U << PWM_AQCTRL_B0_Pos)
#define PWM_AQCTRL_B0                                       PWM_AQCTRL_B0_Msk

#define PWM_AQCTRL_B1_Pos                                   (6U)
#define PWM_AQCTRL_B1_Len                                   (2U)
#define PWM_AQCTRL_B1_Msk                                   (0x3U << PWM_AQCTRL_B1_Pos)
#define PWM_AQCTRL_B1                                       PWM_AQCTRL_B1_Msk

#define PWM_AQCTRL_C0_Pos                                   (8U)
#define PWM_AQCTRL_C0_Len                                   (2U)
#define PWM_AQCTRL_C0_Msk                                   (0x3U << PWM_AQCTRL_C0_Pos)
#define PWM_AQCTRL_C0                                       PWM_AQCTRL_C0_Msk

#define PWM_AQCTRL_C1_Pos                                   (10U)
#define PWM_AQCTRL_C1_Len                                   (2U)
#define PWM_AQCTRL_C1_Msk                                   (0x3U << PWM_AQCTRL_C1_Pos)
#define PWM_AQCTRL_C1                                       PWM_AQCTRL_C1_Msk

/*******************  Bit definition for PWM_BRPRD register  ******************/
#define PWM_BRPRD_BRPRD_Pos                                 (0U)
#define PWM_BRPRD_BRPRD_Len                                 (32U)
#define PWM_BRPRD_BRPRD_Msk                                 (0xFFFFFFFFU)
#define PWM_BRPRD_BRPRD                                     PWM_BRPRD_BRPRD_Msk

/*******************  Bit definition for PWM_HOLD register  *******************/
#define PWM_HOLD_HOLD_Pos                                   (0U)
#define PWM_HOLD_HOLD_Len                                   (24U)
#define PWM_HOLD_HOLD_Msk                                   (0x00FFFFFFU)
#define PWM_HOLD_HOLD                                       PWM_HOLD_HOLD_Msk


/* ================================================================================================================= */
/* ================                                        QSPI                                      ================ */
/* ================================================================================================================= */

/*******************  Bit definition for QSPI_CTRL0 register  ******************/

#define QSPI_CTRL0_IS_MST_Pos                               (31U)
#define QSPI_CTRL0_IS_MST_Len                               (1U)
#define QSPI_CTRL0_IS_MST_Msk                               (0x1U << QSPI_CTRL0_IS_MST_Pos)
#define QSPI_CTRL0_IS_MST                                   QSPI_CTRL0_IS_MST_Msk

#define QSPI_CTRL0_RSVD_26_31                               (26u)

#define QSPI_CTRL0_DWS_EN_Pos                               (25U)
#define QSPI_CTRL0_DWS_EN_Len                               (1U)
#define QSPI_CTRL0_DWS_EN_Msk                               (0x1U << QSPI_CTRL0_DWS_EN_Pos)
#define QSPI_CTRL0_DWS_EN                                   QSPI_CTRL0_DWS_EN_Msk

#define QSPI_CTRL0_HYPERBUS_EN_Pos                          (24U)
#define QSPI_CTRL0_HYPERBUS_EN_Len                          (1U)
#define QSPI_CTRL0_HYPERBUS_EN_Msk                          (0x1U << QSPI_CTRL0_HYPERBUS_EN_Pos)
#define QSPI_CTRL0_HYPERBUS_EN                              QSPI_CTRL0_HYPERBUS_EN_Msk

#define QSPI_CTRL0_SPIFRF_Pos                               (22U)
#define QSPI_CTRL0_SPIFRF_Len                               (2U)
#define QSPI_CTRL0_SPIFRF_Msk                               (0x3U << QSPI_CTRL0_SPIFRF_Pos)
#define QSPI_CTRL0_SPIFRF                                   QSPI_CTRL0_SPIFRF_Msk

#define QSPI_CTRL0_RSVD_20_21                               (20u)

#define QSPI_CTRL0_CFS_Pos                                  (16U)
#define QSPI_CTRL0_CFS_Len                                  (4U)
#define QSPI_CTRL0_CFS_Msk                                  (0xFU << QSPI_CTRL0_CFS_Pos)
#define QSPI_CTRL0_CFS                                      QSPI_CTRL0_CFS_Msk

#define QSPI_CTRL0_RSVD_15                                  (15u)

#define QSPI_CTRL0_SSTEN_Pos                                (14U)
#define QSPI_CTRL0_SSTEN_Len                                (1U)
#define QSPI_CTRL0_SSTEN_Msk                                (0x1U << QSPI_CTRL0_SSTEN_Pos)
#define QSPI_CTRL0_SSTEN                                    QSPI_CTRL0_SSTEN_Msk

#define QSPI_CTRL0_SRL_Pos                                  (13U)
#define QSPI_CTRL0_SRL_Len                                  (1U)
#define QSPI_CTRL0_SRL_Msk                                  (0x1U << QSPI_CTRL0_SRL_Pos)
#define QSPI_CTRL0_SRL                                      QSPI_CTRL0_SRL_Msk

#define QSPI_CTRL0_SLVOE_Pos                                (12U)
#define QSPI_CTRL0_SLVOE_Len                                (1U)
#define QSPI_CTRL0_SLVOE_Msk                                (0x1U << QSPI_CTRL0_SLVOE_Pos)
#define QSPI_CTRL0_SLVOE                                    QSPI_CTRL0_SLVOE_Msk

#define QSPI_CTRL0_TMOD_Pos                                 (10U)
#define QSPI_CTRL0_TMOD_Len                                 (2U)
#define QSPI_CTRL0_TMOD_Msk                                 (0x3U << QSPI_CTRL0_TMOD_Pos)
#define QSPI_CTRL0_TMOD                                     QSPI_CTRL0_TMOD_Msk

#define QSPI_CTRL0_SCPOL_Pos                                (9U)
#define QSPI_CTRL0_SCPOL_Len                                (1U)
#define QSPI_CTRL0_SCPOL_Msk                                (0x1U << QSPI_CTRL0_SCPOL_Pos)
#define QSPI_CTRL0_SCPOL                                    QSPI_CTRL0_SCPOL_Msk

#define QSPI_CTRL0_SCPHA_Pos                                (8U)
#define QSPI_CTRL0_SCPHA_Len                                (1U)
#define QSPI_CTRL0_SCPHA_Msk                                (0x1U << QSPI_CTRL0_SCPHA_Pos)
#define QSPI_CTRL0_SCPHA                                    QSPI_CTRL0_SCPHA_Msk

#define QSPI_CTRL0_FRF_Pos                                  (6U)
#define QSPI_CTRL0_FRF_Len                                  (2U)
#define QSPI_CTRL0_FRF_Msk                                  (0x3U << QSPI_CTRL0_FRF_Pos)
#define QSPI_CTRL0_FRF                                      QSPI_CTRL0_FRF_Msk

#define QSPI_CTRL0_RSVD_5                                   (5u)

#define QSPI_CTRL0_DFS32_Pos                                (0U)
#define QSPI_CTRL0_DFS32_Len                                (5U)
#define QSPI_CTRL0_DFS32_Msk                                (0x1FU << QSPI_CTRL0_DFS32_Pos)
#define QSPI_CTRL0_DFS32                                    QSPI_CTRL0_DFS32_Msk

#define QSPI_CTRL0_DFS_Pos                                  QSPI_CTRL0_DFS32_Pos
#define QSPI_CTRL0_DFS_Len                                  QSPI_CTRL0_DFS32_Len
#define QSPI_CTRL0_DFS_Msk                                  QSPI_CTRL0_DFS32_Msk
#define QSPI_CTRL0_DFS                                      QSPI_CTRL0_DFS32

/*******************  Bit definition for QSPI_CTRL1 register  ******************/
#define QSPI_CTRL1_NDF_Pos                                  (0U)
#define QSPI_CTRL1_NDF_Len                                  (16U)
#define QSPI_CTRL1_NDF_Msk                                  (0xFFFFU << QSPI_CTRL1_NDF_Pos)
#define QSPI_CTRL1_NDF                                      QSPI_CTRL1_NDF_Msk

/*******************  Bit definition for QSPI_SSIEN register  ******************/
#define QSPI_SSI_EN_Pos                                    (0U)
#define QSPI_SSI_EN_Len                                    (1U)
#define QSPI_SSI_EN_Msk                                    (0x1U << QSPI_SSI_EN_Pos)
#define QSPI_SSI_EN                                        QSPI_SSI_EN_Msk

/*******************  Bit definition for QSPI_MWC register  ********************/
#define QSPI_MWC_MHS_Pos                                    (2U)
#define QSPI_MWC_MHS_Len                                    (1U)
#define QSPI_MWC_MHS_Msk                                    (0x1U << QSPI_MWC_MHS_Pos)
#define QSPI_MWC_MHS                                        QSPI_MWC_MHS_Msk

#define QSPI_MWC_MDD_Pos                                    (1U)
#define QSPI_MWC_MDD_Len                                    (1U)
#define QSPI_MWC_MDD_Msk                                    (0x1U << QSPI_MWC_MDD_Pos)
#define QSPI_MWC_MDD                                        QSPI_MWC_MDD_Msk

#define QSPI_MWC_MWMOD_Pos                                  (0U)
#define QSPI_MWC_MWMOD_Len                                  (1U)
#define QSPI_MWC_MWMOD_Msk                                  (0x1U << QSPI_MWC_MWMOD_Pos)
#define QSPI_MWC_MWMOD                                      QSPI_MWC_MWMOD_Msk

/*******************  Bit definition for QSPI_SE register  *********************/
#define QSPI_SE_SLAVE1_Pos                                  (1U)
#define QSPI_SE_SLAVE1_Len                                  (1U)
#define QSPI_SE_SLAVE1_Msk                                  (0x1U << QSPI_SE_SLAVE1_Pos)
#define QSPI_SE_SLAVE1                                      QSPI_SE_SLAVE1_Msk

#define QSPI_SE_SLAVE0_Pos                                  (0U)
#define QSPI_SE_SLAVE0_Len                                  (1U)
#define QSPI_SE_SLAVE0_Msk                                  (0x1U << QSPI_SE_SLAVE0_Pos)
#define QSPI_SE_SLAVE0                                      QSPI_SE_SLAVE0_Msk

/*******************  Bit definition for QSPI_BAUD register  *******************/
#define QSPI_BAUD_SCKDIV_Pos                                (1U)
#define QSPI_BAUD_SCKDIV_Len                                (15U)
#define QSPI_BAUD_SCKDIV_Msk                                (0x7FFFUL << QSPI_BAUD_SCKDIV_Pos)
#define QSPI_BAUD_SCKDIV                                    QSPI_BAUD_SCKDIV_Msk

/*******************  Bit definition for QSPI_TXFTL register  ******************/
#define QSPI_TXFTHR_TFT_Pos                                 (16U)
#define QSPI_TXFTHR_TFT_Len                                 (5U)
#define QSPI_TXFTHR_TFT_Msk                                 (0x1FU << QSPI_TXFTHR_TFT_Pos)
#define QSPI_TXFTHR_TFT                                     QSPI_TXFTHR_TFT_Msk

#define QSPI_TXFTL_TFT_Pos                                  (0U)
#define QSPI_TXFTL_TFT_Len                                  (5U)
#define QSPI_TXFTL_TFT_Msk                                  (0x1FU << QSPI_TXFTL_TFT_Pos)
#define QSPI_TXFTL_TFT                                      QSPI_TXFTL_TFT_Msk

/*******************  Bit definition for QSPI_RXFTL register  ******************/
#define QSPI_RXFTL_RFT_Pos                                  (0U)
#define QSPI_RXFTL_RFT_Len                                  (5U)
#define QSPI_RXFTL_RFT_Msk                                  (0x1FU << QSPI_RXFTL_RFT_Pos)
#define QSPI_RXFTL_RFT                                      QSPI_RXFTL_RFT_Msk

/*******************  Bit definition for QSPI_TXFL register  *******************/
#define QSPI_TXFL_TXTFL_Pos                                 (0U)
#define QSPI_TXFL_TXTFL_Len                                 (6U)
#define QSPI_TXFL_TXTFL_Msk                                 (0x3FU << QSPI_TXFL_TXTFL_Pos)
#define QSPI_TXFL_TXTFL                                     QSPI_TXFL_TXTFL_Msk

/*******************  Bit definition for QSPI_RXFL register  *******************/
#define QSPI_RXFL_RXTFL_Pos                                 (0U)
#define QSPI_RXFL_RXTFL_Len                                 (6U)
#define QSPI_RXFL_RXTFL_Msk                                 (0x3FU << QSPI_RXFL_RXTFL_Pos)
#define QSPI_RXFL_RXTFL                                     QSPI_RXFL_RXTFL_Msk

/*******************  Bit definition for QSPI_STAT register  *******************/
#define QSPI_STAT_DCOL_Pos                                  (6U)
#define QSPI_STAT_DCOL_Len                                  (1U)
#define QSPI_STAT_DCOL_Msk                                  (0x1U << QSPI_STAT_DCOL_Pos)
#define QSPI_STAT_DCOL                                      QSPI_STAT_DCOL_Msk

#define QSPI_STAT_TXE_Pos                                   (5U)
#define QSPI_STAT_TXE_Len                                   (1U)
#define QSPI_STAT_TXE_Msk                                   (0x1U << QSPI_STAT_TXE_Pos)
#define QSPI_STAT_TXE                                       QSPI_STAT_TXE_Msk

#define QSPI_STAT_RFF_Pos                                   (4U)
#define QSPI_STAT_RFF_Len                                   (1U)
#define QSPI_STAT_RFF_Msk                                   (0x1U << QSPI_STAT_RFF_Pos)
#define QSPI_STAT_RFF                                       QSPI_STAT_RFF_Msk

#define QSPI_STAT_RFNE_Pos                                  (3U)
#define QSPI_STAT_RFNE_Len                                  (1U)
#define QSPI_STAT_RFNE_Msk                                  (0x1U << QSPI_STAT_RFNE_Pos)
#define QSPI_STAT_RFNE                                      QSPI_STAT_RFNE_Msk

#define QSPI_STAT_TFE_Pos                                   (2U)
#define QSPI_STAT_TFE_Len                                   (1U)
#define QSPI_STAT_TFE_Msk                                   (0x1U << QSPI_STAT_TFE_Pos)
#define QSPI_STAT_TFE                                       QSPI_STAT_TFE_Msk

#define QSPI_STAT_TFNF_Pos                                  (1U)
#define QSPI_STAT_TFNF_Len                                  (1U)
#define QSPI_STAT_TFNF_Msk                                  (0x1U << QSPI_STAT_TFNF_Pos)
#define QSPI_STAT_TFNF                                      QSPI_STAT_TFNF_Msk

#define QSPI_STAT_BUSY_Pos                                  (0U)
#define QSPI_STAT_BUSY_Len                                  (1U)
#define QSPI_STAT_BUSY_Msk                                  (0x1U << QSPI_STAT_BUSY_Pos)
#define QSPI_STAT_BUSY                                      QSPI_STAT_BUSY_Msk

/*******************  Bit definition for QSPI_INTMASK register  ****************/
#define QSPI_INTMASK_SPITEIM_Pos                            (10U)
#define QSPI_INTMASK_SPITEIM_Len                            (1U)
#define QSPI_INTMASK_SPITEIM_Msk                            (0x1U << QSPI_INTMASK_SPITEIM_Pos)
#define QSPI_INTMASK_SPITEIM                                QSPI_INTMASK_SPITEIM_Msk

#define QSPI_INTMASK_TXUIM_Pos                              (7U)
#define QSPI_INTMASK_TXUIM_Len                              (1U)
#define QSPI_INTMASK_TXUIM_Msk                              (0x1U << QSPI_INTMASK_TXUIM_Pos)
#define QSPI_INTMASK_TXUIM                                  QSPI_INTMASK_TXUIM_Msk

#define QSPI_INTMASK_XRXOIM_Pos                             (6U)
#define QSPI_INTMASK_XRXOIM_Len                             (1U)
#define QSPI_INTMASK_XRXOIM_Msk                             (0x1U << QSPI_INTMASK_XRXOIM_Pos)
#define QSPI_INTMASK_XRXOIM                                 QSPI_INTMASK_XRXOIM_Msk

#define QSPI_INTMASK_MSTIM_Pos                              (5U)
#define QSPI_INTMASK_MSTIM_Len                              (1U)
#define QSPI_INTMASK_MSTIM_Msk                              (0x1U << QSPI_INTMASK_MSTIM_Pos)
#define QSPI_INTMASK_MSTIM                                  QSPI_INTMASK_MSTIM_Msk

#define QSPI_INTMASK_RXFIM_Pos                              (4U)
#define QSPI_INTMASK_RXFIM_Len                              (1U)
#define QSPI_INTMASK_RXFIM_Msk                              (0x1U << QSPI_INTMASK_RXFIM_Pos)
#define QSPI_INTMASK_RXFIM                                  QSPI_INTMASK_RXFIM_Msk

#define QSPI_INTMASK_RXOIM_Pos                              (3U)
#define QSPI_INTMASK_RXOIM_Len                              (1U)
#define QSPI_INTMASK_RXOIM_Msk                              (0x1U << QSPI_INTMASK_RXOIM_Pos)
#define QSPI_INTMASK_RXOIM                                  QSPI_INTMASK_RXOIM_Msk

#define QSPI_INTMASK_RXUIM_Pos                              (2U)
#define QSPI_INTMASK_RXUIM_Len                              (1U)
#define QSPI_INTMASK_RXUIM_Msk                              (0x1U << QSPI_INTMASK_RXUIM_Pos)
#define QSPI_INTMASK_RXUIM                                  QSPI_INTMASK_RXUIM_Msk

#define QSPI_INTMASK_TXOIM_Pos                              (1U)
#define QSPI_INTMASK_TXOIM_Len                              (1U)
#define QSPI_INTMASK_TXOIM_Msk                              (0x1U << QSPI_INTMASK_TXOIM_Pos)
#define QSPI_INTMASK_TXOIM                                  QSPI_INTMASK_TXOIM_Msk

#define QSPI_INTMASK_TXEIM_Pos                              (0U)
#define QSPI_INTMASK_TXEIM_Len                              (1U)
#define QSPI_INTMASK_TXEIM_Msk                              (0x1U << QSPI_INTMASK_TXEIM_Pos)
#define QSPI_INTMASK_TXEIM                                  QSPI_INTMASK_TXEIM_Msk

/*******************  Bit definition for QSPI_INTSTAT register  ****************/
#define QSPI_INTMASK_SPITEIS_Pos                             (10U)
#define QSPI_INTMASK_SPITEIS_Len                             (1U)
#define QSPI_INTMASK_SPITEIS_Msk                             (0x1U << QSPI_INTMASK_SPITEIS_Pos)
#define QSPI_INTMASK_SPITEIS                                 QSPI_INTMASK_SPITEIS_Msk

#define QSPI_INTMASK_TXUIS_Pos                              (7U)
#define QSPI_INTMASK_TXUIS_Len                              (1U)
#define QSPI_INTMASK_TXUIS_Msk                              (0x1U << QSPI_INTMASK_TXUIS_Pos)
#define QSPI_INTMASK_TXUIS                                  QSPI_INTMASK_TXUIS_Msk

#define QSPI_INTSTAT_XRXOIS_Pos                             (6U)
#define QSPI_INTSTAT_XRXOIS_Len                             (1U)
#define QSPI_INTSTAT_XRXOIS_Msk                             (0x1U << QSPI_INTSTAT_XRXOIS_Pos)
#define QSPI_INTSTAT_XRXOIS                                 QSPI_INTSTAT_XRXOIS_Msk

#define QSPI_INTSTAT_MSTIS_Pos                              (5U)
#define QSPI_INTSTAT_MSTIS_Len                              (1U)
#define QSPI_INTSTAT_MSTIS_Msk                              (0x1U << QSPI_INTSTAT_MSTIS_Pos)
#define QSPI_INTSTAT_MSTIS                                  QSPI_INTSTAT_MSTIS_Msk

#define QSPI_INTSTAT_RXFIS_Pos                              (4U)
#define QSPI_INTSTAT_RXFIS_Len                              (1U)
#define QSPI_INTSTAT_RXFIS_Msk                              (0x1U << QSPI_INTSTAT_RXFIS_Pos)
#define QSPI_INTSTAT_RXFIS                                  QSPI_INTSTAT_RXFIS_Msk

#define QSPI_INTSTAT_RXOIS_Pos                              (3U)
#define QSPI_INTSTAT_RXOIS_Len                              (1U)
#define QSPI_INTSTAT_RXOIS_Msk                              (0x1U << QSPI_INTSTAT_RXOIS_Pos)
#define QSPI_INTSTAT_RXOIS                                  QSPI_INTSTAT_RXOIS_Msk

#define QSPI_INTSTAT_RXUIS_Pos                              (2U)
#define QSPI_INTSTAT_RXUIS_Len                              (1U)
#define QSPI_INTSTAT_RXUIS_Msk                              (0x1U << QSPI_INTSTAT_RXUIS_Pos)
#define QSPI_INTSTAT_RXUIS                                  QSPI_INTSTAT_RXUIS_Msk

#define QSPI_INTSTAT_TXOIS_Pos                              (1U)
#define QSPI_INTSTAT_TXOIS_Len                              (1U)
#define QSPI_INTSTAT_TXOIS_Msk                              (0x1U << QSPI_INTSTAT_TXOIS_Pos)
#define QSPI_INTSTAT_TXOIS                                  QSPI_INTSTAT_TXOIS_Msk

#define QSPI_INTSTAT_TXEIS_Pos                              (0U)
#define QSPI_INTSTAT_TXEIS_Len                              (1U)
#define QSPI_INTSTAT_TXEIS_Msk                              (0x1U << QSPI_INTSTAT_TXEIS_Pos)
#define QSPI_INTSTAT_TXEIS                                  QSPI_INTSTAT_TXEIS_Msk

/*******************  Bit definition for QSPI_RAW_INTSTAT register  ************/
#define QSPI_RAW_INTMASK_SPITEIR_Pos                        (10U)
#define QSPI_RAW_INTMASK_SPITEIR_Len                        (1U)
#define QSPI_RAW_INTMASK_SPITEIR_Msk                        (0x1U << QSPI_RAW_INTMASK_SPITEIR_Pos)
#define QSPI_RAW_INTMASK_SPITEIR                            QSPI_RAW_INTMASK_SPITEIR_Msk

#define QSPI_RAW_INTMASK_TXUIR_Pos                          (7U)
#define QSPI_RAW_INTMASK_TXUIR_Len                          (1U)
#define QSPI_RAW_INTMASK_TXUIR_Msk                          (0x1U << QSPI_RAW_INTMASK_TXUIR_Pos)
#define QSPI_RAW_INTMASK_TXUIR                              QSPI_RAW_INTMASK_TXUIR_Msk

#define QSPI_RAW_INTSTAT_XRXOIR_Pos                         (6U)
#define QSPI_RAW_INTSTAT_XRXOIR_Len                         (1U)
#define QSPI_RAW_INTSTAT_XRXOIR_Msk                         (0x1U << QSPI_RAW_INTSTAT_XRXOIR_Pos)
#define QSPI_RAW_INTSTAT_XRXOIR                             QSPI_RAW_INTSTAT_XRXOIR_Msk

#define QSPI_RAW_INTSTAT_MSTIR_Pos                          (5U)
#define QSPI_RAW_INTSTAT_MSTIR_Len                          (1U)
#define QSPI_RAW_INTSTAT_MSTIR_Msk                          (0x1U << QSPI_RAW_INTSTAT_MSTIR_Pos)
#define QSPI_RAW_INTSTAT_MSTIR                              QSPI_RAW_INTSTAT_MSTIR_Msk

#define QSPI_RAW_INTSTAT_RXFIR_Pos                          (4U)
#define QSPI_RAW_INTSTAT_RXFIR_Len                          (1U)
#define QSPI_RAW_INTSTAT_RXFIR_Msk                          (0x1U << QSPI_RAW_INTSTAT_RXFIR_Pos)
#define QSPI_RAW_INTSTAT_RXFIR                              QSPI_RAW_INTSTAT_RXFIR_Msk

#define QSPI_RAW_INTSTAT_RXOIR_Pos                          (3U)
#define QSPI_RAW_INTSTAT_RXOIR_Len                          (1U)
#define QSPI_RAW_INTSTAT_RXOIR_Msk                          (0x1U << QSPI_RAW_INTSTAT_RXOIR_Pos)
#define QSPI_RAW_INTSTAT_RXOIR                              QSPI_RAW_INTSTAT_RXOIR_Msk

#define QSPI_RAW_INTSTAT_RXUIR_Pos                          (2U)
#define QSPI_RAW_INTSTAT_RXUIR_Len                          (1U)
#define QSPI_RAW_INTSTAT_RXUIR_Msk                          (0x1U << QSPI_RAW_INTSTAT_RXUIR_Pos)
#define QSPI_RAW_INTSTAT_RXUIR                              QSPI_RAW_INTSTAT_RXUIR_Msk

#define QSPI_RAW_INTSTAT_TXOIR_Pos                          (1U)
#define QSPI_RAW_INTSTAT_TXOIR_Len                          (1U)
#define QSPI_RAW_INTSTAT_TXOIR_Msk                          (0x1U << QSPI_RAW_INTSTAT_TXOIR_Pos)
#define QSPI_RAW_INTSTAT_TXOIR                              QSPI_RAW_INTSTAT_TXOIR_Msk

#define QSPI_RAW_INTSTAT_TXEIR_Pos                          (0U)
#define QSPI_RAW_INTSTAT_TXEIR_Len                          (1U)
#define QSPI_RAW_INTSTAT_TXEIR_Msk                          (0x1U << QSPI_RAW_INTSTAT_TXEIR_Pos)
#define QSPI_RAW_INTSTAT_TXEIR                              QSPI_RAW_INTSTAT_TXEIR_Msk

/*******************  Bit definition for QSPI_TXOIC register  ******************/
#define QSPI_TXOIC_TXOIC_Pos                                (0U)
#define QSPI_TXOIC_TXOIC_Len                                (1U)
#define QSPI_TXOIC_TXOIC_Msk                                (0x1U << QSPI_TXOIC_TXOIC_Pos)
#define QSPI_TXOIC_TXOIC                                    QSPI_TXOIC_TXOIC_Msk

#define QSPI_TXEIC_TXEIC_Pos                                QSPI_TXOIC_TXOIC_Pos
#define QSPI_TXEIC_TXEIC_Len                                QSPI_TXOIC_TXOIC_Len
#define QSPI_TXEIC_TXEIC_Msk                                QSPI_TXOIC_TXOIC_Msk
#define QSPI_TXEIC_TXEIC                                    QSPI_TXOIC_TXOIC

/*******************  Bit definition for QSPI_RXOIC register  ******************/
#define QSPI_RXOIC_RXOIC_Pos                                (0U)
#define QSPI_RXOIC_RXOIC_Len                                (1U)
#define QSPI_RXOIC_RXOIC_Msk                                (0x1U << QSPI_RXOIC_RXOIC_Pos)
#define QSPI_RXOIC_RXOIC                                    QSPI_RXOIC_RXOIC_Msk

/*******************  Bit definition for QSPI_RXUIC register  ******************/
#define QSPI_RXUIC_RXUIC_Pos                                (0U)
#define QSPI_RXUIC_RXUIC_Len                                (1U)
#define QSPI_RXUIC_RXUIC_Msk                                (0x1U << QSPI_RXUIC_RXUIC_Pos)
#define QSPI_RXUIC_RXUIC                                    QSPI_RXUIC_RXUIC_Msk

/*******************  Bit definition for QSPI_MSTIC register  ******************/
#define QSPI_MSTIC_MSTIC_Pos                                (0U)
#define QSPI_MSTIC_MSTIC_Len                                (1U)
#define QSPI_MSTIC_MSTIC_Msk                                (0x1U << QSPI_MSTIC_MSTIC_Pos)
#define QSPI_MSTIC_MSTIC                                    QSPI_MSTIC_MSTIC_Msk

/*******************  Bit definition for QSPI_INTCLR register  ******************/
#define QSPI_INTCLR_INTCLR_Pos                              (0U)
#define QSPI_INTCLR_INTCLR_Len                              (1U)
#define QSPI_INTCLR_INTCLR_Msk                              (0x1U << QSPI_INTCLR_INTCLR_Pos)
#define QSPI_INTCLR_INTCLR                                  QSPI_INTCLR_INTCLR_Msk

/*******************  Bit definition for QSPI_DMAC register  *******************/
#define QSPI_DMAC_TDMAE_Pos                                 (1U)
#define QSPI_DMAC_TDMAE_Len                                 (1U)
#define QSPI_DMAC_TDMAE_Msk                                 (0x1U << QSPI_DMAC_TDMAE_Pos)
#define QSPI_DMAC_TDMAE                                     QSPI_DMAC_TDMAE_Msk

#define QSPI_DMAC_RDMAE_Pos                                 (0U)
#define QSPI_DMAC_RDMAE_Len                                 (1U)
#define QSPI_DMAC_RDMAE_Msk                                 (0x1U << QSPI_DMAC_RDMAE_Pos)
#define QSPI_DMAC_RDMAE                                     QSPI_DMAC_RDMAE_Msk

/*******************  Bit definition for QSPI_DMATDL register  *****************/
#define QSPI_DMATDL_DMATDL_Pos                              (0U)
#define QSPI_DMATDL_DMATDL_Len                              (6U)
#define QSPI_DMATDL_DMATDL_Msk                              (0x3FU << QSPI_DMATDL_DMATDL_Pos)
#define QSPI_DMATDL_DMATDL                                  QSPI_DMATDL_DMATDL_Msk

/*******************  Bit definition for QSPI_DMARDL register  *****************/
#define QSPI_DMARDL_DMARDL_Pos                              (0U)
#define QSPI_DMARDL_DMARDL_Len                              (6U)
#define QSPI_DMARDL_DMARDL_Msk                              (0x3FU << QSPI_DMARDL_DMARDL_Pos)
#define QSPI_DMARDL_DMARDL                                  QSPI_DMARDL_DMARDL_Msk

/*******************  Bit definition for QSPI_IDCODE register  *****************/
#define QSPI_IDCODE_ID_Pos                                  (0U)
#define QSPI_IDCODE_ID_Len                                  (32U)
#define QSPI_IDCODE_ID_Msk                                  (0xFFFFFFFFU)
#define QSPI_IDCODE_ID                                      QSPI_IDCODE_ID_Msk

/*******************  Bit definition for QSPI_COMP register  *******************/
#define QSPI_COMP_VERSION_Pos                               (0U)
#define QSPI_COMP_VERSION_Len                               (32U)
#define QSPI_COMP_VERSION_Msk                               (0xFFFFFFFFU)
#define QSPI_COMP_VERSION                                   QSPI_COMP_VERSION_Msk

/*******************  Bit definition for QSPI_DATA register  *******************/
#define QSPI_DATA_REG_Pos                                   (0U)
#define QSPI_DATA_REG_Len                                   (32U)
#define QSPI_DATA_REG_Msk                                   (0xFFFFFFFFU)
#define QSPI_DATA_REG                                       QSPI_DATA_REG_Msk

/*******************  Bit definition for QSPI_RX register  *********************/
#define QSPI_RX_SAMPLE_EDGE_Pos                             (16U)
#define QSPI_RX_SAMPLE_EDGE_Len                             (1U)
#define QSPI_RX_SAMPLE_EDGE_Msk                             (0x1U << QSPI_RX_SAMPLE_EDGE_Pos)
#define QSPI_RX_SAMPLE_EDGE                                 QSPI_RX_SAMPLE_EDGE_Msk

#define QSPI_RX_SAMPLEDLY_Pos                               (0U)
#define QSPI_RX_SAMPLEDLY_Len                               (8U)
#define QSPI_RX_SAMPLEDLY_Msk                               (0xFFU << QSPI_RX_SAMPLEDLY_Pos)
#define QSPI_RX_SAMPLEDLY                                   QSPI_RX_SAMPLEDLY_Msk

/*******************  Bit definition for QSPI_SCTRL0 register  *****************/
#define QSPI_SCTRL0_CLK_STRETCH_EN_Pos                      (30U)
#define QSPI_SCTRL0_CLK_STRETCH_EN_Len                      (1U)
#define QSPI_SCTRL0_CLK_STRETCH_EN_Msk                      (0x1U << QSPI_SCTRL0_CLK_STRETCH_EN_Pos)
#define QSPI_SCTRL0_CLK_STRETCH_EN                          QSPI_SCTRL0_CLK_STRETCH_EN_Msk

#define QSPI_SCTRL0_XIP_PREFETCH_EN_Pos                     (29U)
#define QSPI_SCTRL0_XIP_PREFETCH_EN_Len                     (1U)
#define QSPI_SCTRL0_XIP_PREFETCH_EN_Msk                     (0x1U << QSPI_SCTRL0_XIP_PREFETCH_EN_Pos)
#define QSPI_SCTRL0_XIP_PREFETCH_EN                         QSPI_SCTRL0_XIP_PREFETCH_EN_Msk

#define QSPI_SCTRL0_XIP_MBL_Pos                             (26U)
#define QSPI_SCTRL0_XIP_MBL_Len                             (2U)
#define QSPI_SCTRL0_XIP_MBL_Msk                             (0x3U << QSPI_SCTRL0_XIP_MBL_Pos)
#define QSPI_SCTRL0_XIP_MBL                                 QSPI_SCTRL0_XIP_MBL_Msk

#define QSPI_SCTRL0_RXDS_SIG_EN_Pos                         (25U)
#define QSPI_SCTRL0_RXDS_SIG_EN_Len                         (1U)
#define QSPI_SCTRL0_RXDS_SIG_EN_Msk                         (0x1U << QSPI_SCTRL0_RXDS_SIG_EN_Pos)
#define QSPI_SCTRL0_RXDS_SIG_EN                             QSPI_SCTRL0_RXDS_SIG_EN_Msk

#define QSPI_SCTRL0_DM_EN_Pos                               (24U)
#define QSPI_SCTRL0_DM_EN_Len                               (1U)
#define QSPI_SCTRL0_DM_EN_Msk                               (0x1U << QSPI_SCTRL0_DM_EN_Pos)
#define QSPI_SCTRL0_DM_EN                                   QSPI_SCTRL0_DM_EN_Msk

#define QSPI_SCTRL0_XIP_CONT_XFER_EN_Pos                    (21U)
#define QSPI_SCTRL0_XIP_CONT_XFER_EN_Len                    (1U)
#define QSPI_SCTRL0_XIP_CONT_XFER_EN_Msk                    (0x1U << QSPI_SCTRL0_XIP_CONT_XFER_EN_Pos)
#define QSPI_SCTRL0_XIP_CONT_XFER_EN                        QSPI_SCTRL0_XIP_CONT_XFER_EN_Msk

#define QSPI_SCTRL0_XIP_INST_EN_Pos                         (20U)
#define QSPI_SCTRL0_XIP_INST_EN_Len                         (1U)
#define QSPI_SCTRL0_XIP_INST_EN_Msk                         (0x1U << QSPI_SCTRL0_XIP_INST_EN_Pos)
#define QSPI_SCTRL0_XIP_INST_EN                             QSPI_SCTRL0_XIP_INST_EN_Msk

#define QSPI_SCTRL0_XIP_DFS_HC_Pos                          (19U)
#define QSPI_SCTRL0_XIP_DFS_HC_Len                          (1U)
#define QSPI_SCTRL0_XIP_DFS_HC_Msk                          (0x1U << QSPI_SCTRL0_XIP_DFS_HC_Pos)
#define QSPI_SCTRL0_XIP_DFS_HC                              QSPI_SCTRL0_XIP_DFS_HC_Msk

#define QSPI_SCTRL0_RXDS_EN_Pos                             (18U)
#define QSPI_SCTRL0_RXDS_EN_Len                             (1U)
#define QSPI_SCTRL0_RXDS_EN_Msk                             (0x1U << QSPI_SCTRL0_RXDS_EN_Pos)
#define QSPI_SCTRL0_RXDS_EN                                 QSPI_SCTRL0_RXDS_EN_Msk

#define QSPI_SCTRL0_INST_DDR_EN_Pos                         (17U)
#define QSPI_SCTRL0_INST_DDR_EN_Len                         (1U)
#define QSPI_SCTRL0_INST_DDR_EN_Msk                         (0x1U << QSPI_SCTRL0_INST_DDR_EN_Pos)
#define QSPI_SCTRL0_INST_DDR_EN                             QSPI_SCTRL0_INST_DDR_EN_Msk

#define QSPI_SCTRL0_DDR_EN_Pos                              (16U)
#define QSPI_SCTRL0_DDR_EN_Len                              (1U)
#define QSPI_SCTRL0_DDR_EN_Msk                              (0x1U << QSPI_SCTRL0_DDR_EN_Pos)
#define QSPI_SCTRL0_DDR_EN                                  QSPI_SCTRL0_DDR_EN_Msk

#define QSPI_SCTRL0_WAITCYCLES_Pos                          (11U)
#define QSPI_SCTRL0_WAITCYCLES_Len                          (5U)
#define QSPI_SCTRL0_WAITCYCLES_Msk                          (0x1FU << QSPI_SCTRL0_WAITCYCLES_Pos)
#define QSPI_SCTRL0_WAITCYCLES                              QSPI_SCTRL0_WAITCYCLES_Msk

#define QSPI_SCTRL0_INSTL_Pos                               (8U)
#define QSPI_SCTRL0_INSTL_Len                               (2U)
#define QSPI_SCTRL0_INSTL_Msk                               (0x03U << QSPI_SCTRL0_INSTL_Pos)
#define QSPI_SCTRL0_INSTL                                   QSPI_SCTRL0_INSTL_Msk

#define QSPI_SCTRL0_XIP_MD_BIT_EN_Pos                       (7U)
#define QSPI_SCTRL0_XIP_MD_BIT_EN_Len                       (1U)
#define QSPI_SCTRL0_XIP_MD_BIT_EN_Msk                       (0x1U << QSPI_SCTRL0_XIP_MD_BIT_EN_Pos)
#define QSPI_SCTRL0_XIP_MD_BIT_EN                           QSPI_SCTRL0_XIP_MD_BIT_EN_Msk

#define QSPI_SCTRL0_ADDRL_Pos                               (2U)
#define QSPI_SCTRL0_ADDRL_Len                               (4U)
#define QSPI_SCTRL0_ADDRL_Msk                               (0x0FU << QSPI_SCTRL0_ADDRL_Pos)
#define QSPI_SCTRL0_ADDRL                                   QSPI_SCTRL0_ADDRL_Msk

#define QSPI_SCTRL0_TRANSTYPE_Pos                           (0U)
#define QSPI_SCTRL0_TRANSTYPE_Len                           (2U)
#define QSPI_SCTRL0_TRANSTYPE_Msk                           (0x03U << QSPI_SCTRL0_TRANSTYPE_Pos)
#define QSPI_SCTRL0_TRANSTYPE                               QSPI_SCTRL0_TRANSTYPE_Msk

/*******************  Bit definition for QSPI_XIP_MODE_BITS register  *****************/
#define QSPI_XIP_MODE_BITS_Pos                              (0U)
#define QSPI_XIP_MODE_BITS_Len                              (16U)
#define QSPI_XIP_MODE_BITS_Msk                              (0xFFFFU << QSPI_XIP_MODE_BITS_Pos)
#define QSPI_XIP_MODE_BITS                                  QSPI_XIP_MODE_BITS_Msk

/*******************  Bit definition for QSPI_XIP_INCR_INST register  *****************/
#define QSPI_XIP_INCR_INST_Pos                              (0U)
#define QSPI_XIP_INCR_INST_Len                              (16U)
#define QSPI_XIP_INCR_INST_Msk                              (0xFFFFU << QSPI_XIP_INCR_INST_Pos)
#define QSPI_XIP_INCR_INST                                  QSPI_XIP_INCR_INST_Msk

/*******************  Bit definition for QSPI_XIP_WRAP_INST register  *****************/
#define QSPI_XIP_WRAP_INST_Pos                              (0U)
#define QSPI_XIP_WRAP_INST_Len                              (16U)
#define QSPI_XIP_WRAP_INST_Msk                              (0xFFFFU << QSPI_XIP_WRAP_INST_Pos)
#define QSPI_XIP_WRAP_INST                                  QSPI_XIP_WRAP_INST_Msk

/*******************  Bit definition for QSPI_XIP_CTRL register  *****************/
#define QSPI_XCTRL_XIP_PREFETCH_EN_Pos                      (29U)
#define QSPI_XCTRL_XIP_PREFETCH_EN_Len                      (1U)
#define QSPI_XCTRL_XIP_PREFETCH_EN_Msk                      (0x1U << QSPI_XCTRL_XIP_PREFETCH_EN_Pos)
#define QSPI_XCTRL_XIP_PREFETCH_EN                          QSPI_XCTRL_XIP_PREFETCH_EN_Msk

#define QSPI_XCTRL_XIP_MBL_Pos                              (26U)
#define QSPI_XCTRL_XIP_MBL_Len                              (2U)
#define QSPI_XCTRL_XIP_MBL_Msk                              (0x3U << QSPI_XCTRL_XIP_MBL_Pos)
#define QSPI_XCTRL_XIP_MBL                                  QSPI_XCTRL_XIP_MBL_Msk

#define QSPI_XCTRL_RXDS_SIG_EN_Pos                          (25U)
#define QSPI_XCTRL_RXDS_SIG_EN_Len                          (1U)
#define QSPI_XCTRL_RXDS_SIG_EN_Msk                          (0x1U << QSPI_XCTRL_RXDS_SIG_EN_Pos)
#define QSPI_XCTRL_RXDS_SIG_EN                              QSPI_XCTRL_RXDS_SIG_EN_Msk

#define QSPI_XCTRL_HYPERBUS_EN_Pos                          (24U)
#define QSPI_XCTRL_HYPERBUS_EN_Len                          (1U)
#define QSPI_XCTRL_HYPERBUS_EN_Msk                          (0x1U << QSPI_XCTRL_HYPERBUS_EN_Pos)
#define QSPI_XCTRL_HYPERBUS_EN                              QSPI_XCTRL_HYPERBUS_EN_Msk

#define QSPI_XCTRL_CONT_XFER_EN_Pos                         (23U)
#define QSPI_XCTRL_CONT_XFER_EN_Len                         (1U)
#define QSPI_XCTRL_CONT_XFER_EN_Msk                         (0x1U << QSPI_XCTRL_CONT_XFER_EN_Pos)
#define QSPI_XCTRL_CONT_XFER_EN                             QSPI_XCTRL_CONT_XFER_EN_Msk

#define QSPI_XCTRL_INST_EN_Pos                              (22U)
#define QSPI_XCTRL_INST_EN_Len                              (1U)
#define QSPI_XCTRL_INST_EN_Msk                              (0x1U << QSPI_XCTRL_INST_EN_Pos)
#define QSPI_XCTRL_INST_EN                                  QSPI_XCTRL_INST_EN_Msk

#define QSPI_XCTRL_RXDS_EN_Pos                              (21U)
#define QSPI_XCTRL_RXDS_EN_Len                              (1U)
#define QSPI_XCTRL_RXDS_EN_Msk                              (0x1U << QSPI_XCTRL_RXDS_EN_Pos)
#define QSPI_XCTRL_RXDS_EN                                  QSPI_XCTRL_RXDS_EN_Msk

#define QSPI_XCTRL_INST_DDR_EN_Pos                          (20U)
#define QSPI_XCTRL_INST_DDR_EN_Len                          (1U)
#define QSPI_XCTRL_INST_DDR_EN_Msk                          (0x1U << QSPI_XCTRL_INST_DDR_EN_Pos)
#define QSPI_XCTRL_INST_DDR_EN                              QSPI_XCTRL_INST_DDR_EN_Msk

#define QSPI_XCTRL_DDR_EN_Pos                               (19U)
#define QSPI_XCTRL_DDR_EN_Len                               (1U)
#define QSPI_XCTRL_DDR_EN_Msk                               (0x1U << QSPI_XCTRL_DDR_EN_Pos)
#define QSPI_XCTRL_DDR_EN                                   QSPI_XCTRL_DDR_EN_Msk

#define QSPI_XCTRL_DFS_HC_Pos                               (18U)
#define QSPI_XCTRL_DFS_HC_Len                               (1U)
#define QSPI_XCTRL_DFS_HC_Msk                               (0x1U << QSPI_XCTRL_DFS_HC_Pos)
#define QSPI_XCTRL_DFS_HC                                   QSPI_XCTRL_DFS_HC_Msk

#define QSPI_XCTRL_WAITCYCLES_Pos                           (13U)
#define QSPI_XCTRL_WAITCYCLES_Len                           (5U)
#define QSPI_XCTRL_WAITCYCLES_Msk                           (0x1FU << QSPI_XCTRL_WAITCYCLES_Pos)
#define QSPI_XCTRL_WAITCYCLES                               QSPI_XCTRL_WAITCYCLES_Msk

#define QSPI_XCTRL_MD_BIT_EN_Pos                            (12U)
#define QSPI_XCTRL_MD_BIT_EN_Len                            (1U)
#define QSPI_XCTRL_MD_BIT_EN_Msk                            (0x1U << QSPI_XCTRL_MD_BIT_EN_Pos)
#define QSPI_XCTRL_MD_BIT_EN                                QSPI_XCTRL_MD_BIT_EN_Msk

#define QSPI_XCTRL_INSTL_Pos                                (9U)
#define QSPI_XCTRL_INSTL_Len                                (2U)
#define QSPI_XCTRL_INSTL_Msk                                (0x03U << QSPI_XCTRL_INSTL_Pos)
#define QSPI_XCTRL_INSTL                                    QSPI_XCTRL_INSTL_Msk

#define QSPI_XCTRL_ADDRL_Pos                                (4U)
#define QSPI_XCTRL_ADDRL_Len                                (4U)
#define QSPI_XCTRL_ADDRL_Msk                                (0x0FU << QSPI_XCTRL_ADDRL_Pos)
#define QSPI_XCTRL_ADDRL                                    QSPI_XCTRL_ADDRL_Msk

#define QSPI_XCTRL_TRANSTYPE_Pos                            (2U)
#define QSPI_XCTRL_TRANSTYPE_Len                            (2U)
#define QSPI_XCTRL_TRANSTYPE_Msk                            (0x03U << QSPI_XCTRL_TRANSTYPE_Pos)
#define QSPI_XCTRL_TRANSTYPE                                QSPI_XCTRL_TRANSTYPE_Msk

#define QSPI_XCTRL_FRF_Pos                                  (0U)
#define QSPI_XCTRL_FRF_Len                                  (2U)
#define QSPI_XCTRL_FRF_Msk                                  (0x03U << QSPI_XCTRL_FRF_Pos)
#define QSPI_XCTRL_FRF                                      QSPI_XCTRL_FRF_Msk

/*******************  Bit definition for QSPI_XIP_SER register  *****************/
#define QSPI_XIP_SLAVE0_EN_Pos                              (0U)
#define QSPI_XIP_SLAVE0_EN_Len                              (1U)
#define QSPI_XIP_SLAVE0_EN_Msk                              (0x1U << QSPI_XIP_SLAVE0_EN_Pos)
#define QSPI_XIP_SLAVE0_EN                                  QSPI_XIP_SLAVE0_EN_Msk

/*******************  Bit definition for QSPI_XIP_RXOICR register  *****************/
#define QSPI_XIP_XRXOIC_Pos                                 (0U)
#define QSPI_XIP_XRXOIC_Len                                 (1U)
#define QSPI_XIP_XRXOIC_Msk                                 (0x1U << QSPI_XIP_XRXOIC_Pos)
#define QSPI_XIP_XRXOIC                                     QSPI_XIP_XRXOIC_Msk

/*******************  Bit definition for QSPI_XIP_CNT_TIMEOUT register  *****************/
#define QSPI_XIP_TOCNT_Pos                                  (0U)
#define QSPI_XIP_TOCNT_Len                                  (8U)
#define QSPI_XIP_TOCNT_Msk                                  (0xFFU << QSPI_XIP_TOCNT_Pos)
#define QSPI_XIP_TOCNT                                      QSPI_XIP_TOCNT_Msk


/*******************  Bit definition for QSPI_SPI_CTRLR1 register  *****************/
#define QSPI_SCTRL1_MAX_WS_Pos                              (8U)
#define QSPI_SCTRL1_MAX_WS_Len                              (4U)
#define QSPI_SCTRL1_MAX_WS_Msk                              (0xFU << QSPI_SCTRL1_MAX_WS_Pos)
#define QSPI_SCTRL1_MAX_WS                                  QSPI_SCTRL1_MAX_WS_Msk

#define QSPI_SCTRL1_DYN_WS_Pos                              (3U)
#define QSPI_SCTRL1_DYN_WS_Len                              (5U)
#define QSPI_SCTRL1_DYN_WS_Msk                              (0x1FU << QSPI_SCTRL1_DYN_WS_Pos)
#define QSPI_SCTRL1_DYN_WS                                  QSPI_SCTRL1_DYN_WS_Msk


/*******************  Bit definition for QSPI_SPITECR register     *****************/
#define QSPI_SPITE_TECLR_Pos                                (0U)
#define QSPI_SPITE_TECLR_Len                                (1U)
#define QSPI_SPITE_TECLR_Msk                                (0x1U << QSPI_SPITE_TECLR_Pos)
#define QSPI_SPITE_TECLR                                    QSPI_SPITE_TECLR_Msk


/*******************  Bit definition for QSPI_XIP_WRITE_INCR_INST register    ******/
#define QSPI_XIP_WR_INCR_INST_Pos                           (0U)
#define QSPI_XIP_WR_INCR_INST_Len                           (16U)
#define QSPI_XIP_WR_INCR_INST_Msk                           (0xFFFFU << QSPI_XIP_WR_INCR_INST_Pos)
#define QSPI_XIP_WR_INCR_INST                               QSPI_XIP_WR_INCR_INST_Msk

/*******************  Bit definition for QSPI_XIP_WRITE_WRAP_INST register    ******/
#define QSPI_XIP_WR_WRAP_INST_Pos                           (0U)
#define QSPI_XIP_WR_WRAP_INST_Len                           (16U)
#define QSPI_XIP_WR_WRAP_INST_Msk                           (0xFFFFU << QSPI_XIP_WR_WRAP_INST_Pos)
#define QSPI_XIP_WR_WRAP_INST                               QSPI_XIP_WR_WRAP_INST_Msk

/*******************  Bit definition for QSPI_XIP_WRITE_CTRL register    ***********/
#define QSPI_XIP_WR_CTRL_WAITCYCLES_Pos                     (16U)
#define QSPI_XIP_WR_CTRL_WAITCYCLES_Len                     (5U)
#define QSPI_XIP_WR_CTRL_WAITCYCLES_Msk                     (0x1FU << QSPI_XIP_WR_CTRL_WAITCYCLES_Pos)
#define QSPI_XIP_WR_CTRL_WAITCYCLES                         QSPI_XIP_WR_CTRL_WAITCYCLES_Msk

#define QSPI_XIP_WR_CTRL_INSTL_Pos                          (8U)
#define QSPI_XIP_WR_CTRL_INSTL_Len                          (2U)
#define QSPI_XIP_WR_CTRL_INSTL_Msk                          (0x03U << QSPI_XIP_WR_CTRL_INSTL_Pos)
#define QSPI_XIP_WR_CTRL_INSTL                              QSPI_XIP_WR_CTRL_INSTL_Msk

#define QSPI_XIP_WR_CTRL_ADDRL_Pos                          (4U)
#define QSPI_XIP_WR_CTRL_ADDRL_Len                          (4U)
#define QSPI_XIP_WR_CTRL_ADDRL_Msk                          (0x0FU << QSPI_XIP_WR_CTRL_ADDRL_Pos)
#define QSPI_XIP_WR_CTRL_ADDRL                              QSPI_XIP_WR_CTRL_ADDRL_Msk

#define QSPI_XIP_WR_CTRL_TRANSTYPE_Pos                      (2U)
#define QSPI_XIP_WR_CTRL_TRANSTYPE_Len                      (2U)
#define QSPI_XIP_WR_CTRL_TRANSTYPE_Msk                      (0x03U << QSPI_XIP_WR_CTRL_TRANSTYPE_Pos)
#define QSPI_XIP_WR_CTRL_TRANSTYPE                          QSPI_XIP_WR_CTRL_TRANSTYPE_Msk

#define QSPI_XIP_WR_CTRL_FRF_Pos                            (0U)
#define QSPI_XIP_WR_CTRL_FRF_Len                            (2U)
#define QSPI_XIP_WR_CTRL_FRF_Msk                            (0x03U << QSPI_XIP_WR_CTRL_FRF_Pos)
#define QSPI_XIP_WR_CTRL_FRF                                QSPI_XIP_WR_CTRL_FRF_Msk

/* ================================================================================================================= */
/* ================                                        SPI                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for SPI_CTRL0 register  *******************/
#define SPI_CTRL0_FRAME_FORMAT_POS                          (4U)
#define SPI_CTRL0_FRAME_FORMAT_Len                          (2U)
#define SPI_CTRL0_FRAME_FORMAT_Msk                          (0x3UL << SPI_CTRL0_FRAME_FORMAT_POS)
#define SPI_CTRL0_FRAME_FORMAT                              SPI_CTRL0_FRAME_FORMAT_Msk

#define SPI_CTRL0_SERIAL_CLK_PHASE_POS                      (6U)
#define SPI_CTRL0_SERIAL_CLK_PHASE_Len                      (1U)
#define SPI_CTRL0_SERIAL_CLK_PHASE_Msk                      (0x1UL << SPI_CTRL0_SERIAL_CLK_PHASE_POS)
#define SPI_CTRL0_SERIAL_CLK_PHASE                          SPI_CTRL0_SERIAL_CLK_PHASE_Msk

#define SPI_CTRL0_SERIAL_CLK_POL_POS                        (7U)
#define SPI_CTRL0_SERIAL_CLK_POL_Len                        (1U)
#define SPI_CTRL0_SERIAL_CLK_POL_Msk                        (0x1UL << SPI_CTRL0_SERIAL_CLK_POL_POS)
#define SPI_CTRL0_SERIAL_CLK_POL                            SPI_CTRL0_SERIAL_CLK_POL_Msk

#define SPI_CTRL0_XFE_MODE_POS                              (8U)
#define SPI_CTRL0_XFE_MODE_Len                              (2U)
#define SPI_CTRL0_XFE_MODE_Msk                              (0x3UL << SPI_CTRL0_XFE_MODE_POS)
#define SPI_CTRL0_XFE_MODE                                  SPI_CTRL0_XFE_MODE_Msk

#define SPI_CTRL0_S_OUT_EN_POS                              (10U)
#define SPI_CTRL0_S_OUT_EN_Len                              (1U)
#define SPI_CTRL0_S_OUT_EN_Msk                              (0x1UL << SPI_CTRL0_S_OUT_EN_POS)
#define SPI_CTRL0_S_OUT_EN                                  SPI_CTRL0_S_OUT_EN_Msk

#define SPI_CTRL0_SHIFT_REG_LOOP_POS                        (11U)
#define SPI_CTRL0_SHIFT_REG_LOOP_Len                        (1U)
#define SPI_CTRL0_SHIFT_REG_LOOP_Msk                        (0x1UL << SPI_CTRL0_SHIFT_REG_LOOP_POS)
#define SPI_CTRL0_SHIFT_REG_LOOP                            SPI_CTRL0_SHIFT_REG_LOOP_Msk

#define SPI_CTRL0_CTRL_FRAME_SIZE_POS                       (12U)
#define SPI_CTRL0_CTRL_FRAME_SIZE_Len                       (4U)
#define SPI_CTRL0_CTRL_FRAME_SIZE_Msk                       (0xFUL << SPI_CTRL0_CTRL_FRAME_SIZE_POS)
#define SPI_CTRL0_CTRL_FRAME_SIZE                           SPI_CTRL0_CTRL_FRAME_SIZE_Msk

#define SPI_CTRL0_DATA_FRAME_SIZE_POS                       (16U)
#define SPI_CTRL0_DATA_FRAME_SIZE_Len                       (5U)
#define SPI_CTRL0_DATA_FRAME_SIZE_Msk                       (0x1FUL << SPI_CTRL0_DATA_FRAME_SIZE_POS)
#define SPI_CTRL0_DATA_FRAME_SIZE                           SPI_CTRL0_DATA_FRAME_SIZE_Msk

#define SPI_CTRL0_S_ST_EN_POS                               (24U)
#define SPI_CTRL0_S_ST_EN_Len                               (1U)
#define SPI_CTRL0_S_ST_EN_Msk                               (0x1UL << SPI_CTRL0_S_ST_EN_POS)
#define SPI_CTRL0_S_ST_EN                                   SPI_CTRL0_S_ST_EN_Msk

/*******************  Bit definition for SPI_CTRL1 register  *******************/
#define SPI_CTRL1_NUM_DATA_FRAME_POS                        (0U)
#define SPI_CTRL1_NUM_DATA_FRAME_Len                        (16U)
#define SPI_CTRL1_NUM_DATA_FRAME_Msk                        (0xFFFFUL << SPI_CTRL1_NUM_DATA_FRAME_POS)
#define SPI_CTRL1_NUM_DATA_FRAME                            SPI_CTRL1_NUM_DATA_FRAME_Msk

/*******************  Bit definition for SPI_SSI_EN register  *******************/
#define SPI_SSI_EN_POS                                      (0U)
#define SPI_SSI_EN_Len                                      (1U)
#define SPI_SSI_EN_Msk                                      (0x1UL << SPI_SSI_EN_POS)
#define SPI_SSI_EN                                          SPI_SSI_EN_Msk

/*******************  Bit definition for SPI_MW_CTRL register  *******************/
#define SPI_MW_CTRL_MW_XFE_MODE_POS                         (0U)
#define SPI_MW_CTRL_MW_XFE_MODE_Len                         (1U)
#define SPI_MW_CTRL_MW_XFE_MODE_Msk                         (0x1UL << SPI_MW_CTRL_MW_XFE_MODE_POS)
#define SPI_MW_CTRL_MW_XFE_MODE                             SPI_MW_CTRL_MW_XFE_MODE_Msk

#define SPI_MW_CTRL_MW_DIR_DW_POS                           (1U)
#define SPI_MW_CTRL_MW_DIR_DW_Len                           (1U)
#define SPI_MW_CTRL_MW_DIR_DW_Msk                           (0x1UL << SPI_MW_CTRL_MW_DIR_DW_POS)
#define SPI_MW_CTRL_MW_DIR_DW                               SPI_MW_CTRL_MW_DIR_DW_Msk

#define SPI_MW_CTRL_MW_HSG_POS                              (2U)
#define SPI_MW_CTRL_MW_HSG_Len                              (1U)
#define SPI_MW_CTRL_MW_HSG_Msk                              (0x1UL << SPI_MW_CTRL_MW_HSG_POS)
#define SPI_MW_CTRL_MW_HSG                                  SPI_MW_CTRL_MW_HSG_Msk

/*******************  Bit definition for SPI_S_EN register  *******************/
#define SPI_SLA_S0_SEL_EN_POS                               (0U)
#define SPI_SLA_S0_SEL_EN_Len                               (1U)
#define SPI_SLA_S0_SEL_EN_Msk                               (0x1UL << SPI_SLA_S0_SEL_EN_POS)
#define SPI_SLA_S0_SEL_EN                                   SPI_SLA_S0_SEL_EN_Msk

#define SPI_SLA_S1_SEL_EN_POS                               (1U)
#define SPI_SLA_S1_SEL_EN_Len                               (1U)
#define SPI_SLA_S1_SEL_EN_Msk                               (0x1UL << SPI_SLA_S1_SEL_EN_POS)
#define SPI_SLA_S1_SEL_EN                                   SPI_SLA_S1_SEL_EN_Msk

/*******************  Bit definition for SPI_BAUD register  *******************/
#define SPI_BAUD_CLK_DIV_POS                                (0U)
#define SPI_BAUD_CLK_DIV_Len                                (16U)
#define SPI_BAUD_CLK_DIV_Msk                                (0xFFFFUL << SPI_BAUD_CLK_DIV_POS)
#define SPI_BAUD_CLK_DIV                                    SPI_BAUD_CLK_DIV_Msk

/*******************  Bit definition for SPI_TX_FIFO_TL register  *******************/
#define SPI_TX_FIFO_TL_TX_FIFO_THD_POS                      (0U)
#define SPI_TX_FIFO_TL_TX_FIFO_THD_Len                      (4U)
#define SPI_TX_FIFO_TL_TX_FIFO_THD_Msk                      (0xFUL << SPI_TX_FIFO_TL_TX_FIFO_THD_POS)
#define SPI_TX_FIFO_TL_TX_FIFO_THD                          SPI_TX_FIFO_TL_TX_FIFO_THD_Msk

/*******************  Bit definition for SPI_RX_FIFO_TL register  *******************/
#define SPI_RX_FIFO_TL_RX_FIFO_THD_POS                      (0U)
#define SPI_RX_FIFO_TL_RX_FIFO_THD_Len                      (4U)
#define SPI_RX_FIFO_TL_RX_FIFO_THD_Msk                      (0xFUL << SPI_RX_FIFO_TL_RX_FIFO_THD_POS)
#define SPI_RX_FIFO_TL_RX_FIFO_THD                          SPI_RX_FIFO_TL_RX_FIFO_THD_Msk

/*******************  Bit definition for SPI_TX_FIFO_LEVEL register  *******************/
#define SPI_TX_FIFO_LEVEL_TX_FIFO_LEVEL_POS                 (0U)
#define SPI_TX_FIFO_LEVEL_TX_FIFO_LEVEL_Len                 (5U)
#define SPI_TX_FIFO_LEVEL_TX_FIFO_LEVEL_Msk                 (0x1FUL << SPI_TX_FIFO_LEVEL_TX_FIFO_LEVEL_POS)
#define SPI_TX_FIFO_LEVEL_TX_FIFO_LEVEL                     SPI_TX_FIFO_LEVEL_TX_FIFO_LEVEL_Msk

/*******************  Bit definition for SPI_RX_FIFO_LEVEL register  *******************/
#define SPI_RX_FIFO_LEVEL_RX_FIFO_LEVEL_POS                 (0U)
#define SPI_RX_FIFO_LEVEL_RX_FIFO_LEVEL_Len                 (5U)
#define SPI_RX_FIFO_LEVEL_RX_FIFO_LEVEL_Msk                 (0x1FUL << SPI_RX_FIFO_LEVEL_RX_FIFO_LEVEL_POS)
#define SPI_RX_FIFO_LEVEL_RX_FIFO_LEVEL                     SPI_RX_FIFO_LEVEL_RX_FIFO_LEVEL_Msk

/*******************  Bit definition for SPI_STAT register  *******************/
#define SPI_STAT_SSI_BUSY_POS                               (0U)
#define SPI_STAT_SSI_BUSY_Len                               (1U)
#define SPI_STAT_SSI_BUSY_Msk                               (0x1UL << SPI_STAT_SSI_BUSY_POS)
#define SPI_STAT_SSI_BUSY                                   SPI_STAT_SSI_BUSY_Msk

#define SPI_STAT_TX_FIFO_NF_POS                             (1U)
#define SPI_STAT_TX_FIFO_NF_Len                             (1U)
#define SPI_STAT_TX_FIFO_NF_Msk                             (0x1UL << SPI_STAT_TX_FIFO_NF_POS)
#define SPI_STAT_TX_FIFO_NF                                 SPI_STAT_TX_FIFO_NF_Msk

#define SPI_STAT_TX_FIFO_EMPTY_POS                          (2U)
#define SPI_STAT_TX_FIFO_EMPTY_Len                          (1U)
#define SPI_STAT_TX_FIFO_EMPTY_Msk                          (0x1UL << SPI_STAT_TX_FIFO_EMPTY_POS)
#define SPI_STAT_TX_FIFO_EMPTY                              SPI_STAT_TX_FIFO_EMPTY_Msk

#define SPI_STAT_RX_FIFO_NE_POS                             (3U)
#define SPI_STAT_RX_FIFO_NE_Len                             (1U)
#define SPI_STAT_RX_FIFO_NE_Msk                             (0x1UL << SPI_STAT_RX_FIFO_NE_POS)
#define SPI_STAT_RX_FIFO_NE                                 SPI_STAT_RX_FIFO_NE_Msk

#define SPI_STAT_RX_FIFO_FULL_POS                           (4U)
#define SPI_STAT_RX_FIFO_FULL_Len                           (1U)
#define SPI_STAT_RX_FIFO_FULL_Msk                           (0x1UL << SPI_STAT_RX_FIFO_FULL_POS)
#define SPI_STAT_RX_FIFO_FULL                               SPI_STAT_RX_FIFO_FULL_Msk

#define SPI_STAT_TX_ERR_POS                                 (5U)
#define SPI_STAT_TX_ERR_Len                                 (1U)
#define SPI_STAT_TX_ERR_Msk                                 (0x1UL << SPI_STAT_TX_ERR_POS)
#define SPI_STAT_TX_ERR                                     SPI_STAT_TX_ERR_Msk

#define SPI_STAT_DATA_COLN_ERR_POS                          (6U)
#define SPI_STAT_DATA_COLN_ERR_Len                          (1U)
#define SPI_STAT_DATA_COLN_ERR_Msk                          (0x1UL << SPI_STAT_DATA_COLN_ERR_POS)
#define SPI_STAT_DATA_COLN_ERR                              SPI_STAT_DATA_COLN_ERR_Msk

/*******************  Bit definition for SPI_INT_MASK register  *******************/
#define SPI_INT_MASK_TX_FIFO_EIM_POS                        (0U)
#define SPI_INT_MASK_TX_FIFO_EIM_Len                        (1U)
#define SPI_INT_MASK_TX_FIFO_EIM_Msk                        (0x1UL << SPI_INT_MASK_TX_FIFO_EIM_POS)
#define SPI_INT_MASK_TX_FIFO_EIM                            SPI_INT_MASK_TX_FIFO_EIM_Msk

#define SPI_INT_MASK_TX_FIFO_OIM_POS                        (1U)
#define SPI_INT_MASK_TX_FIFO_OIM_Len                        (1U)
#define SPI_INT_MASK_TX_FIFO_OIM_Msk                        (0x1UL << SPI_INT_MASK_TX_FIFO_OIM_POS)
#define SPI_INT_MASK_TX_FIFO_OIM                            SPI_INT_MASK_TX_FIFO_OIM_Msk

#define SPI_INT_MASK_RX_FIFO_UIM_POS                        (2U)
#define SPI_INT_MASK_RX_FIFO_UIM_Len                        (1U)
#define SPI_INT_MASK_RX_FIFO_UIM_Msk                        (0x1UL << SPI_INT_MASK_RX_FIFO_UIM_POS)
#define SPI_INT_MASK_RX_FIFO_UIM                            SPI_INT_MASK_RX_FIFO_UIM_Msk

#define SPI_INT_MASK_RX_FIFO_OIM_POS                        (3U)
#define SPI_INT_MASK_RX_FIFO_OIM_Len                        (1U)
#define SPI_INT_MASK_RX_FIFO_OIM_Msk                        (0x1UL << SPI_INT_MASK_RX_FIFO_OIM_POS)
#define SPI_INT_MASK_RX_FIFO_OIM                            SPI_INT_MASK_RX_FIFO_OIM_Msk

#define SPI_INT_MASK_RX_FIFO_FIM_POS                        (4U)
#define SPI_INT_MASK_RX_FIFO_FIM_Len                        (1U)
#define SPI_INT_MASK_RX_FIFO_FIM_Msk                        (0x1UL << SPI_INT_MASK_RX_FIFO_FIM_POS)
#define SPI_INT_MASK_RX_FIFO_FIM                            SPI_INT_MASK_RX_FIFO_FIM_Msk

#define SPI_INT_MASK_MULTI_M_CIM_POS                        (5U)
#define SPI_INT_MASK_MULTI_M_CIM_Len                        (1U)
#define SPI_INT_MASK_MULTI_M_CIM_Msk                        (0x1UL << SPI_INT_MASK_MULTI_M_CIM_POS)
#define SPI_INT_MASK_MULTI_M_CIM                            SPI_INT_MASK_MULTI_M_CIM_Msk

/*******************  Bit definition for SPI_INT_STAT register  *******************/
#define SPI_INT_STAT_TX_FIFO_EIS_POS                        (0U)
#define SPI_INT_STAT_TX_FIFO_EIS_Len                        (1U)
#define SPI_INT_STAT_TX_FIFO_EIS_Msk                        (0x1UL << SPI_INT_STAT_TX_FIFO_EIS_POS)
#define SPI_INT_STAT_TX_FIFO_EIS                            SPI_INT_STAT_TX_FIFO_EIS_Msk

#define SPI_INT_STAT_TX_FIFO_OIS_POS                        (1U)
#define SPI_INT_STAT_TX_FIFO_OIS_Len                        (1U)
#define SPI_INT_STAT_TX_FIFO_OIS_Msk                        (0x1UL << SPI_INT_STAT_TX_FIFO_OIS_POS)
#define SPI_INT_STAT_TX_FIFO_OIS                            SPI_INT_STAT_TX_FIFO_OIS_Msk

#define SPI_INT_STAT_RX_FIFO_UIS_POS                        (2U)
#define SPI_INT_STAT_RX_FIFO_UIS_Len                        (1U)
#define SPI_INT_STAT_RX_FIFO_UIS_Msk                        (0x1UL << SPI_INT_STAT_RX_FIFO_UIS_POS)
#define SPI_INT_STAT_RX_FIFO_UIS                            SPI_INT_STAT_RX_FIFO_UIS_Msk

#define SPI_INT_STAT_RX_FIFO_OIS_POS                        (3U)
#define SPI_INT_STAT_RX_FIFO_OIS_Len                        (1U)
#define SPI_INT_STAT_RX_FIFO_OIS_Msk                        (0x1UL << SPI_INT_STAT_RX_FIFO_OIS_POS)
#define SPI_INT_STAT_RX_FIFO_OIS                            SPI_INT_STAT_RX_FIFO_OIS_Msk

#define SPI_INT_STAT_RX_FIFO_FIS_POS                        (4U)
#define SPI_INT_STAT_RX_FIFO_FIS_Len                        (1U)
#define SPI_INT_STAT_RX_FIFO_FIS_Msk                        (0x1UL << SPI_INT_STAT_RX_FIFO_FIS_POS)
#define SPI_INT_STAT_RX_FIFO_FIS                            SPI_INT_STAT_RX_FIFO_FIS_Msk

#define SPI_INT_STAT_MULTI_M_CIS_POS                        (5U)
#define SPI_INT_STAT_MULTI_M_CIS_Len                        (1U)
#define SPI_INT_STAT_MULTI_M_CIS_Msk                        (0x1UL << SPI_INT_STAT_MULTI_M_CIS_POS)
#define SPI_INT_STAT_MULTI_M_CIS                            SPI_INT_STAT_MULTI_M_CIS_Msk

/*******************  Bit definition for SPI_RAW_INT_STAT register  *******************/
#define SPI_RAW_INT_STAT_TX_FIFO_ERIS_POS                   (0U)
#define SPI_RAW_INT_STAT_TX_FIFO_ERIS_Len                   (1U)
#define SPI_RAW_INT_STAT_TX_FIFO_ERIS_Msk                   (0x1UL << SPI_RAW_INT_STAT_TX_FIFO_ERIS_POS)
#define SPI_RAW_INT_STAT_TX_FIFO_ERIS                       SPI_RAW_INT_STAT_TX_FIFO_ERIS_Msk

#define SPI_RAW_INT_STAT_TX_FIFO_ORIS_POS                   (1U)
#define SPI_RAW_INT_STAT_TX_FIFO_ORIS_Len                   (1U)
#define SPI_RAW_INT_STAT_TX_FIFO_ORIS_Msk                   (0x1UL << SPI_RAW_INT_STAT_TX_FIFO_ORIS_POS)
#define SPI_RAW_INT_STAT_TX_FIFO_ORIS                       SPI_RAW_INT_STAT_TX_FIFO_ORIS_Msk

#define SPI_RAW_INT_STAT_RX_FIFO_URIS_POS                   (2U)
#define SPI_RAW_INT_STAT_RX_FIFO_URIS_Len                   (1U)
#define SPI_RAW_INT_STAT_RX_FIFO_URIS_Msk                   (0x1UL << SPI_RAW_INT_STAT_RX_FIFO_URIS_POS)
#define SPI_RAW_INT_STAT_RX_FIFO_URIS                       SPI_RAW_INT_STAT_RX_FIFO_URIS_Msk

#define SPI_RAW_INT_STAT_RX_FIFO_ORIS_POS                   (3U)
#define SPI_RAW_INT_STAT_RX_FIFO_ORIS_Len                   (1U)
#define SPI_RAW_INT_STAT_RX_FIFO_ORIS_Msk                   (0x1UL << SPI_RAW_INT_STAT_RX_FIFO_ORIS_POS)
#define SPI_RAW_INT_STAT_RX_FIFO_ORIS                       SPI_RAW_INT_STAT_RX_FIFO_ORIS_Msk

#define SPI_RAW_INT_STAT_RX_FIFO_FRIS_POS                   (4U)
#define SPI_RAW_INT_STAT_RX_FIFO_FRIS_Len                   (1U)
#define SPI_RAW_INT_STAT_RX_FIFO_FRIS_Msk                   (0x1UL << SPI_RAW_INT_STAT_RX_FIFO_FRIS_POS)
#define SPI_RAW_INT_STAT_RX_FIFO_FRIS                       SPI_RAW_INT_STAT_RX_FIFO_FRIS_Msk

#define SPI_RAW_INT_STAT_MULTI_M_CRIS_POS                   (5U)
#define SPI_RAW_INT_STAT_MULTI_M_CRIS_Len                   (1U)
#define SPI_RAW_INT_STAT_MULTI_M_CRIS_Msk                   (0x1UL << SPI_RAW_INT_STAT_MULTI_M_CRIS_POS)
#define SPI_RAW_INT_STAT_MULTI_M_CRIS                       SPI_RAW_INT_STAT_MULTI_M_CRIS_Msk

/*******************  Bit definition for SPI_TX_FIFO_OIC register  *******************/
#define SPI_TX_FIFO_OIC_TX_FIFO_OIC_POS                     (0U)
#define SPI_TX_FIFO_OIC_TX_FIFO_OIC_Len                     (1U)
#define SPI_TX_FIFO_OIC_TX_FIFO_OIC_Msk                     (0x1UL << SPI_TX_FIFO_OIC_TX_FIFO_OIC_POS)
#define SPI_TX_FIFO_OIC_TX_FIFO_OIC                         SPI_TX_FIFO_OIC_TX_FIFO_OIC_Msk

/*******************  Bit definition for SPI_RX_FIFO_OIC register  *******************/
#define SPI_RX_FIFO_OIC_RX_FIFO_OIC_POS                     (0U)
#define SPI_RX_FIFO_OIC_RX_FIFO_OIC_Len                     (1U)
#define SPI_RX_FIFO_OIC_RX_FIFO_OIC_Msk                     (0x1UL << SPI_RX_FIFO_OIC_RX_FIFO_OIC_POS)
#define SPI_RX_FIFO_OIC_RX_FIFO_OIC                         SPI_RX_FIFO_OIC_RX_FIFO_OIC_Msk

/*******************  Bit definition for SPI_RX_FIFO_UIC register  *******************/
#define SPI_RX_FIFO_UIC_RX_FIFO_UIC_POS                     (0U)
#define SPI_RX_FIFO_UIC_RX_FIFO_UIC_Len                     (1U)
#define SPI_RX_FIFO_UIC_RX_FIFO_UIC_Msk                     (0x1UL << SPI_RX_FIFO_UIC_RX_FIFO_UIC_POS)
#define SPI_RX_FIFO_UIC_RX_FIFO_UIC                         SPI_RX_FIFO_UIC_RX_FIFO_UIC_Msk

/*******************  Bit definition for SPI_MULTI_M_IC register  *******************/
#define SPI_MULTI_M_IC_MULTI_M_IC_POS                       (0U)
#define SPI_MULTI_M_IC_MULTI_M_IC_Len                       (1U)
#define SPI_MULTI_M_IC_MULTI_M_IC_Msk                       (0x1UL << SPI_MULTI_M_IC_MULTI_M_IC_POS)
#define SPI_MULTI_M_IC_MULTI_M_IC                           SPI_MULTI_M_IC_MULTI_M_IC_Msk

/*******************  Bit definition for SPI_INT_CLR register  *******************/
#define SPI_INT_CLR_INT_CLR_POS                             (0U)
#define SPI_INT_CLR_INT_CLR_Len                             (1U)
#define SPI_INT_CLR_INT_CLR_Msk                             (0x1UL << SPI_INT_CLR_INT_CLR_POS)
#define SPI_INT_CLR_INT_CLR                                 SPI_INT_CLR_INT_CLR_Msk

/*******************  Bit definition for SPI_DMA_CTRL register  *******************/
#define SPI_DMA_CTRL_RX_DMA_EN_POS                          (0U)
#define SPI_DMA_CTRL_RX_DMA_EN_Len                          (1U)
#define SPI_DMA_CTRL_RX_DMA_EN_Msk                          (0x1UL << SPI_DMA_CTRL_RX_DMA_EN_POS)
#define SPI_DMA_CTRL_RX_DMA_EN                              SPI_DMA_CTRL_RX_DMA_EN_Msk

#define SPI_DMA_CTRL_TX_DMA_EN_POS                          (1U)
#define SPI_DMA_CTRL_TX_DMA_EN_Len                          (1U)
#define SPI_DMA_CTRL_TX_DMA_EN_Msk                          (0x1UL << SPI_DMA_CTRL_TX_DMA_EN_POS)
#define SPI_DMA_CTRL_TX_DMA_EN                              SPI_DMA_CTRL_TX_DMA_EN_Msk

/*******************  Bit definition for SPI_DMA_TX_DL register  *******************/
#define SPI_DMA_TX_DL_DMA_TX_DL_POS                         (0U)
#define SPI_DMA_TX_DL_DMA_TX_DL_Len                         (4U)
#define SPI_DMA_TX_DL_DMA_TX_DL_Msk                         (0xFUL << SPI_DMA_TX_DL_DMA_TX_DL_POS)
#define SPI_DMA_TX_DL_DMA_TX_DL                             SPI_DMA_TX_DL_DMA_TX_DL_Msk

/*******************  Bit definition for SPI_DMA_RX_DL register  *******************/
#define SPI_DMA_RX_DL_DMA_RX_DL_POS                         (0U)
#define SPI_DMA_RX_DL_DMA_RX_DL_Len                         (4U)
#define SPI_DMA_RX_DL_DMA_RX_DL_Msk                         (0xFUL << SPI_DMA_RX_DL_DMA_RX_DL_POS)
#define SPI_DMA_RX_DL_DMA_RX_DL                             SPI_DMA_RX_DL_DMA_RX_DL_Msk

/*******************  Bit definition for SPI_DATA register  *******************/
#define SPI_DATA_DATA_POS                                   (0U)
#define SPI_DATA_DATA_Len                                   (32U)
#define SPI_DATA_DATA_Msk                                   (0xFFFFFFFFUL << SPI_DATA_DATA_POS)
#define SPI_DATA_DATA                                       SPI_DATA_DATA_Msk

/*******************  Bit definition for SPI_RX_SAMPLEDLY register  *******************/
#define SPI_RX_SAMPLEDLY_POS                                (0U)
#define SPI_RX_SAMPLEDLY_Len                                (3U)
#define SPI_RX_SAMPLEDLY_Msk                                (0x07UL << SPI_RX_SAMPLEDLY_POS)
#define SPI_RX_SAMPLEDLY                                    SPI_RX_SAMPLEDLY_Msk

/* ================================================================================================================= */
/* ================                                       OSPI-xccela                               ================ */
/* ================================================================================================================= */

/*******************  Bit definition for MEM_BASE_ADDR register  *******************/
#define OSPI_X_MEM_BASE_ADDR_POS                             (0U)
#define OSPI_X_MEM_BASE_ADDR_Len                             (25U)
#define OSPI_X_MEM_BASE_ADDR_Msk                             (0x1FFFFFFUL << OSPI_X_MEM_BASE_ADDR_POS)
#define OSPI_X_MEM_BASE_ADDR                                 OSPI_X_MEM_BASE_ADDR_Msk

/*******************  Bit definition for MEM_TOP_ADDR register  *******************/
#define OSPI_X_MEM_TOP_ADDR_POS                              (0U)
#define OSPI_X_MEM_TOP_ADDR_Len                              (25U)
#define OSPI_X_MEM_TOP_ADDR_Msk                              (0x1FFFFFFUL << OSPI_X_MEM_TOP_ADDR_POS)
#define OSPI_X_MEM_TOP_ADDR                                  OSPI_X_MEM_TOP_ADDR_Msk

/*******************  Bit definition for GLOBAL_RESET register  *******************/
#define OSPI_X_GLOBAL_RST_EN_POS                             (0U)
#define OSPI_X_GLOBAL_RST_EN_Len                             (1U)
#define OSPI_X_GLOBAL_RST_EN_Msk                             (0x01UL << OSPI_X_GLOBAL_RST_EN_POS)
#define OSPI_X_GLOBAL_RST_EN                                 OSPI_X_GLOBAL_RST_EN_Msk

#define OSPI_X_TRST_CNT_POS                                  (1U)
#define OSPI_X_TRST_CNT_Len                                  (9U)
#define OSPI_X_TRST_CNT_Msk                                  (0x1FFUL << OSPI_X_TRST_CNT_POS)
#define OSPI_X_TRST_CNT                                      OSPI_X_TRST_CNT_Msk

/*******************  Bit definition for ACCESS_TYPE register  *******************/
#define OSPI_X_ACCESS_TYPE_POS                               (0U)
#define OSPI_X_ACCESS_TYPE_Len                               (1U)
#define OSPI_X_ACCESS_TYPE_Msk                               (0x01UL << OSPI_X_ACCESS_TYPE_POS)
#define OSPI_X_ACCESS_TYPE                                   OSPI_X_ACCESS_TYPE_Msk

/*******************  Bit definition for ACCESS_TIMING register  *******************/
#define OSPI_X_TCEM_IGNORE_POS                               (0U)
#define OSPI_X_TCEM_IGNORE_Len                               (1U)
#define OSPI_X_TCEM_IGNORE_Msk                               (0x01UL << OSPI_X_TCEM_IGNORE_POS)
#define OSPI_X_TCEM_IGNORE                                   OSPI_X_TCEM_IGNORE_Msk

#define OSPI_X_TCEM_CNT_POS                                  (1U)
#define OSPI_X_TCEM_CNT_Len                                  (12U)
#define OSPI_X_TCEM_CNT_Msk                                  (0xFFFUL << OSPI_X_TCEM_CNT_POS)
#define OSPI_X_TCEM_CNT                                      OSPI_X_TCEM_CNT_Msk

#define OSPI_X_TRC_CNT_POS                                   (13U)
#define OSPI_X_TRC_CNT_Len                                   (4U)
#define OSPI_X_TRC_CNT_Msk                                   (0x0FUL << OSPI_X_TRC_CNT_POS)
#define OSPI_X_TRC_CNT                                       OSPI_X_TRC_CNT_Msk

#define OSPI_X_TCPH_CNT_POS                                  (17U)
#define OSPI_X_TCPH_CNT_Len                                  (3U)
#define OSPI_X_TCPH_CNT_Msk                                  (0x07UL << OSPI_X_TCPH_CNT_POS)
#define OSPI_X_TCPH_CNT                                      OSPI_X_TCPH_CNT_Msk

#define OSPI_X_MEM_PAGE_SIZE_POS                             (20U)
#define OSPI_X_MEM_PAGE_SIZE_Len                             (4U)
#define OSPI_X_MEM_PAGE_SIZE_Msk                             (0x0FUL << OSPI_X_MEM_PAGE_SIZE_POS)
#define OSPI_X_MEM_PAGE_SIZE                                 OSPI_X_MEM_PAGE_SIZE_Msk

/*******************  Bit definition for DEEP_DOWN_CNTRL register  *******************/
#define OSPI_X_DPD_ENTRY_POS                                 (0U)
#define OSPI_X_DPD_ENTRY_Len                                 (1U)
#define OSPI_X_DPD_ENTRY_Msk                                 (0x01UL << OSPI_X_DPD_ENTRY_POS)
#define OSPI_X_DPD_ENTRY                                     OSPI_X_DPD_ENTRY_Msk

#define OSPI_X_DPD_EXIT_POS                                  (1U)
#define OSPI_X_DPD_EXIT_Len                                  (1U)
#define OSPI_X_DPD_EXIT_Msk                                  (0x01UL << OSPI_X_DPD_EXIT_POS)
#define OSPI_X_DPD_EXIT                                      OSPI_X_DPD_EXIT_Msk

#define OSPI_X_TXDPD_TIME_IGNORE_POS                         (2U)
#define OSPI_X_TXDPD_TIME_IGNORE_Len                         (1U)
#define OSPI_X_TXDPD_TIME_IGNORE_Msk                         (0x01UL << OSPI_X_TXDPD_TIME_IGNORE_POS)
#define OSPI_X_TXDPD_TIME_IGNORE                             OSPI_X_TXDPD_TIME_IGNORE_Msk

#define OSPI_X_TXDPD_CNT_POS                                 (3U)
#define OSPI_X_TXDPD_CNT_Len                                 (16U)
#define OSPI_X_TXDPD_CNT_Msk                                 (0xFFFFUL << OSPI_X_TXDPD_CNT_POS)
#define OSPI_X_TXDPD_CNT                                     OSPI_X_TXDPD_CNT_Msk

#define OSPI_X_DPD_EXIT_CYCLE_CNT_POS                        (19U)
#define OSPI_X_DPD_EXIT_CYCLE_CNT_Len                        (4U)
#define OSPI_X_DPD_EXIT_CYCLE_CNT_Msk                        (0x0FUL << OSPI_X_DPD_EXIT_CYCLE_CNT_POS)
#define OSPI_X_DPD_EXIT_CYCLE_CNT                            OSPI_X_DPD_EXIT_CYCLE_CNT_Msk
#define OSPI_X_TXPDPD_CNT                                    OSPI_X_DPD_EXIT_CYCLE_CNT

/*******************  Bit definition for HALF_SLP_CNTRL register  *******************/
#define OSPI_X_HS_ENTRY_POS                                  (0U)
#define OSPI_X_HS_ENTRY_Len                                  (1U)
#define OSPI_X_HS_ENTRY_Msk                                  (0x01UL << OSPI_X_HS_ENTRY_POS)
#define OSPI_X_HS_ENTRY                                      OSPI_X_HS_ENTRY_Msk

#define OSPI_X_HS_EXIT_POS                                   (1U)
#define OSPI_X_HS_EXIT_Len                                   (1U)
#define OSPI_X_HS_EXIT_Msk                                   (0x01UL << OSPI_X_HS_EXIT_POS)
#define OSPI_X_HS_EXIT                                       OSPI_X_HS_EXIT_Msk

#define OSPI_X_TXHS_TIME_IGNORE_POS                          (2U)
#define OSPI_X_TXHS_TIME_IGNORE_Len                          (1U)
#define OSPI_X_TXHS_TIME_IGNORE_Msk                          (0x01UL << OSPI_X_TXHS_TIME_IGNORE_POS)
#define OSPI_X_TXHS_TIME_IGNORE                              OSPI_X_TXHS_TIME_IGNORE_Msk

#define OSPI_X_TXHS_CNT_POS                                  (3U)
#define OSPI_X_TXHS_CNT_Len                                  (15U)
#define OSPI_X_TXHS_CNT_Msk                                  (0x7FFFUL << OSPI_X_TXHS_CNT_POS)
#define OSPI_X_TXHS_CNT                                      OSPI_X_TXHS_CNT_Msk

#define OSPI_X_HS_EXIT_CYCLE_CNT_POS                         (18U)
#define OSPI_X_HS_EXIT_CYCLE_CNT_Len                         (4U)
#define OSPI_X_HS_EXIT_CYCLE_CNT_Msk                         (0x0FUL << OSPI_X_HS_EXIT_CYCLE_CNT_POS)
#define OSPI_X_HS_EXIT_CYCLE_CNT                             OSPI_X_HS_EXIT_CYCLE_CNT_Msk

/*******************  Bit definition for INTERRUPT_CNTRL register  *******************/
#define OSPI_X_GLOBAL_RST_IE_POS                              (0U)
#define OSPI_X_GLOBAL_RST_IE_Len                              (1U)
#define OSPI_X_GLOBAL_RST_IE_Msk                              (0x01UL << OSPI_X_GLOBAL_RST_IE_POS)
#define OSPI_X_GLOBAL_RST_IE                                  OSPI_X_GLOBAL_RST_IE_Msk

#define OSPI_X_HS_ENTRY_IE_POS                                (1U)
#define OSPI_X_HS_ENTRY_IE_Len                                (1U)
#define OSPI_X_HS_ENTRY_IE_Msk                                (0x01UL << OSPI_X_HS_ENTRY_IE_POS)
#define OSPI_X_HS_ENTRY_IE                                    OSPI_X_HS_ENTRY_IE_Msk

#define OSPI_X_HS_EXIT_IE_POS                                 (2U)
#define OSPI_X_HS_EXIT_IE_Len                                 (1U)
#define OSPI_X_HS_EXIT_IE_Msk                                 (0x01UL << OSPI_X_HS_EXIT_IE_POS)
#define OSPI_X_HS_EXIT_IE                                     OSPI_X_HS_EXIT_IE_Msk

#define OSPI_X_DPD_ENTRY_IE_POS                               (3U)
#define OSPI_X_DPD_ENTRY_IE_Len                               (1U)
#define OSPI_X_DPD_ENTRY_IE_Msk                               (0x01UL << OSPI_X_DPD_ENTRY_IE_POS)
#define OSPI_X_DPD_ENTRY_IE                                   OSPI_X_DPD_ENTRY_IE_Msk

#define OSPI_X_DPD_EXIT_IE_POS                                (4U)
#define OSPI_X_DPD_EXIT_IE_Len                                (1U)
#define OSPI_X_DPD_EXIT_IE_Msk                                (0x01UL << OSPI_X_DPD_EXIT_IE_POS)
#define OSPI_X_DPD_EXIT_IE                                    OSPI_X_DPD_EXIT_IE_Msk

#define OSPI_X_DQS_TIMEOUT_IE_POS                             (5U)
#define OSPI_X_DQS_TIMEOUT_IE_Len                             (1U)
#define OSPI_X_DQS_TIMEOUT_IE_Msk                             (0x01UL << OSPI_X_DQS_TIMEOUT_IE_POS)
#define OSPI_X_DQS_TIMEOUT_IE                                 OSPI_X_DQS_TIMEOUT_IE_Msk

/*******************  Bit definition for XFER_STATUS register  *******************/
#define OSPI_X_GLOBAL_RST_DONE_POS                            (0U)
#define OSPI_X_GLOBAL_RST_DONE_Len                            (1U)
#define OSPI_X_GLOBAL_RST_DONE_Msk                            (0x01UL << OSPI_X_GLOBAL_RST_DONE_POS)
#define OSPI_X_GLOBAL_RST_DONE                                OSPI_X_GLOBAL_RST_DONE_Msk

#define OSPI_X_HS_ENTRY_DONE_POS                              (1U)
#define OSPI_X_HS_ENTRY_DONE_Len                              (1U)
#define OSPI_X_HS_ENTRY_DONE_Msk                              (0x01UL << OSPI_X_HS_ENTRY_DONE_POS)
#define OSPI_X_HS_ENTRY_DONE                                  OSPI_X_HS_ENTRY_DONE_Msk

#define OSPI_X_HS_EXIT_DONE_POS                               (2U)
#define OSPI_X_HS_EXIT_DONE_Len                               (1U)
#define OSPI_X_HS_EXIT_DONE_Msk                               (0x01UL << OSPI_X_HS_EXIT_DONE_POS)
#define OSPI_X_HS_EXIT_DONE                                   OSPI_X_HS_EXIT_DONE_Msk

#define OSPI_X_DPD_ENTRY_DONE_POS                             (3U)
#define OSPI_X_DPD_ENTRY_DONE_Len                             (1U)
#define OSPI_X_DPD_ENTRY_DONE_Msk                             (0x01UL << OSPI_X_DPD_ENTRY_DONE_POS)
#define OSPI_X_DPD_ENTRY_DONE                                 OSPI_X_DPD_ENTRY_DONE_Msk

#define OSPI_X_DPD_EXIT_DONE_POS                              (4U)
#define OSPI_X_DPD_EXIT_DONE_Len                              (1U)
#define OSPI_X_DPD_EXIT_DONE_Msk                              (0x01UL << OSPI_X_DPD_EXIT_DONE_POS)
#define OSPI_X_DPD_EXIT_DONE                                  OSPI_X_DPD_EXIT_DONE_Msk

#define OSPI_X_DQS_NON_TOGGLE_ERR_POS                         (5U)
#define OSPI_X_DQS_NON_TOGGLE_ERR_Len                         (1U)
#define OSPI_X_DQS_NON_TOGGLE_ERR_Msk                         (0x01UL << OSPI_X_DQS_NON_TOGGLE_ERR_POS)
#define OSPI_X_DQS_NON_TOGGLE_ERR                             OSPI_X_DQS_NON_TOGGLE_ERR_Msk


/*******************  Bit definition for CMD_CNTRL_1 register  *******************/
#define OSPI_X_CMD_SYNC_RD_POS                                (0U)
#define OSPI_X_CMD_SYNC_RD_Len                                (8U)
#define OSPI_X_CMD_SYNC_RD_Msk                                (0xFFUL << OSPI_X_CMD_SYNC_RD_POS)
#define OSPI_X_CMD_SYNC_RD                                    OSPI_X_CMD_SYNC_RD_Msk

#define OSPI_X_CMD_SYNC_WR_POS                                (8U)
#define OSPI_X_CMD_SYNC_WR_Len                                (8U)
#define OSPI_X_CMD_SYNC_WR_Msk                                (0xFFUL << OSPI_X_CMD_SYNC_WR_POS)
#define OSPI_X_CMD_SYNC_WR                                    OSPI_X_CMD_SYNC_WR_Msk

#define OSPI_X_CMD_BURST_RD_POS                               (16U)
#define OSPI_X_CMD_BURST_RD_Len                               (8U)
#define OSPI_X_CMD_BURST_RD_Msk                               (0xFFUL << OSPI_X_CMD_BURST_RD_POS)
#define OSPI_X_CMD_BURST_RD                                   OSPI_X_CMD_BURST_RD_Msk

#define OSPI_X_CMD_BURST_WR_POS                               (24U)
#define OSPI_X_CMD_BURST_WR_Len                               (8U)
#define OSPI_X_CMD_BURST_WR_Msk                               (0xFFUL << OSPI_X_CMD_BURST_WR_POS)
#define OSPI_X_CMD_BURST_WR                                   OSPI_X_CMD_BURST_WR_Msk


/*******************  Bit definition for CMD_CNTRL_2 register  *******************/
#define OSPI_X_CMD_REG_RD_POS                                 (0U)
#define OSPI_X_CMD_REG_RD_Len                                 (8U)
#define OSPI_X_CMD_REG_RD_Msk                                 (0xFFUL << OSPI_X_CMD_REG_RD_POS)
#define OSPI_X_CMD_REG_RD                                     OSPI_X_CMD_REG_RD_Msk

#define OSPI_X_CMD_REG_WR_POS                                 (8U)
#define OSPI_X_CMD_REG_WR_Len                                 (8U)
#define OSPI_X_CMD_REG_WR_Msk                                 (0xFFUL << OSPI_X_CMD_REG_WR_POS)
#define OSPI_X_CMD_REG_WR                                     OSPI_X_CMD_REG_WR_Msk

#define OSPI_X_CMD_GLOBAL_RST_POS                             (16U)
#define OSPI_X_CMD_GLOBAL_RST_Len                             (8U)
#define OSPI_X_CMD_GLOBAL_RST_Msk                             (0xFFUL << OSPI_X_CMD_GLOBAL_RST_POS)
#define OSPI_X_CMD_GLOBAL_RST                                 OSPI_X_CMD_GLOBAL_RST_Msk


/*******************  Bit definition for DQS_TIMEOUT register  *******************/
#define OSPI_X_DQS_NON_TGL_TIMEOUT_POS                        (0U)
#define OSPI_X_DQS_NON_TGL_TIMEOUT_Len                        (5U)
#define OSPI_X_DQS_NON_TGL_TIMEOUT_Msk                        (0x1FUL << OSPI_X_DQS_NON_TGL_TIMEOUT_POS)
#define OSPI_X_DQS_NON_TGL_TIMEOUT                            OSPI_X_DQS_NON_TGL_TIMEOUT_Msk


/*******************  Bit definition for READ_PREFETCH register  *******************/
#define OSPI_X_RD_DATA_PREFETCH_POS                           (0U)
#define OSPI_X_RD_DATA_PREFETCH_Len                           (1U)
#define OSPI_X_RD_DATA_PREFETCH_Msk                           (0x01UL << OSPI_X_RD_DATA_PREFETCH_POS)
#define OSPI_X_RD_DATA_PREFETCH                               OSPI_X_RD_DATA_PREFETCH_Msk


/* ================================================================================================================= */
/* ================                                       TIMER                                     ================ */
/* ================================================================================================================= */
/*******************  Bit definition for TIMER_CTRL register  *******************/
#define TIMER_CTRL_INTEN_Pos                                (3U)
#define TIMER_CTRL_INTEN_Len                                (1U)
#define TIMER_CTRL_INTEN_Msk                                (0x1U << TIMER_CTRL_INTEN_Pos)
#define TIMER_CTRL_INTEN                                    TIMER_CTRL_INTEN_Msk

#define TIMER_CTRL_EN_Pos                                   (0U)
#define TIMER_CTRL_EN_Len                                   (1U)
#define TIMER_CTRL_EN_Msk                                   (0x1U << TIMER_CTRL_EN_Pos)
#define TIMER_CTRL_EN                                       TIMER_CTRL_EN_Msk

/*******************  Bit definition for TIMER_VALUE register  ******************/
#define TIMER_VALUE_VALUE_Pos                               (0U)
#define TIMER_VALUE_VALUE_Len                               (32U)
#define TIMER_VALUE_VALUE_Msk                               (0xFFFFFFFFU)
#define TIMER_VALUE_VALUE                                   TIMER_VALUE_VALUE_Msk

/*******************  Bit definition for TIMER_RELOAD register  *****************/
#define TIMER_RELOAD_RELOAD_Pos                             (0U)
#define TIMER_RELOAD_RELOAD_Len                             (32U)
#define TIMER_RELOAD_RELOAD_Msk                             (0xFFFFFFFFU)
#define TIMER_RELOAD_RELOAD                                 TIMER_RELOAD_RELOAD_Msk

/*******************  Bit definition for TIMER_RELOAD register  *****************/
#define TIMER_INT_STAT_Pos                                  (0U)
#define TIMER_INT_STAT_Len                                  (1U)
#define TIMER_INT_STAT_Msk                                  (0x1U << TIMER_INT_STAT_Pos)
#define TIMER_INT_STAT                                       TIMER_INT_STAT_Msk


/* ================================================================================================================= */
/* ================                                       UART                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for UART_RBR register  *******************/
#define UART_RBR_RBR_Pos                                    (0U)
#define UART_RBR_RBR_Len                                    (8U)
#define UART_RBR_RBR_Msk                                    (0xFFU << UART_RBR_RBR_Pos)
#define UART_RBR_RBR                                        UART_RBR_RBR_Msk  /**< Receive Buffer Register */

/*******************  Bit definition for UART_DLL register  *******************/
#define UART_DLL_DLL_Pos                                    (0U)
#define UART_DLL_DLL_Len                                    (8U)
#define UART_DLL_DLL_Msk                                    (0xFFU << UART_DLL_DLL_Pos)
#define UART_DLL_DLL                                        UART_DLL_DLL_Msk  /**< Divisor Latch (Low) */

/*******************  Bit definition for UART_THR register  *******************/
#define UART_THR_THR_Pos                                    (0U)
#define UART_THR_THR_Len                                    (8U)
#define UART_THR_THR_Msk                                    (0xFFU << UART_THR_THR_Pos)
#define UART_THR_THR                                        UART_THR_THR_Msk  /**< Transmit Holding Register */

/*******************  Bit definition for UART_DLH register  *******************/
#define UART_DLH_DLH_Pos                                    (0U)
#define UART_DLH_DLH_Len                                    (8U)
#define UART_DLH_DLH_Msk                                    (0xFFU << UART_DLH_DLH_Pos)
#define UART_DLH_DLH                                        UART_DLH_DLH_Msk  /**< Divisor Latch (High) */

/*******************  Bit definition for UART_IER register  *******************/
#define UART_IER_PTIME_Pos                                  (7U)
#define UART_IER_PTIME_Len                                  (1U)
#define UART_IER_PTIME_Msk                                  (0x1U << UART_IER_PTIME_Pos)
#define UART_IER_PTIME                                      UART_IER_PTIME_Msk  /**< Programmable THRE Interrupt Mode Enable */

#define UART_IER_ELCOLR_Pos                                 (4U)
#define UART_IER_ELCOLR_Len                                 (1U)
#define UART_IER_ELCOLR_Msk                                 (0x1U << UART_IER_ELCOLR_Pos)
#define UART_IER_ELCOLR                                     UART_IER_ELCOLR_Msk /**< Enable Auto Clear LSR Register by read RBR/LSR, read only */

#define UART_IER_EDSSI_Pos                                  (3U)
#define UART_IER_EDSSI_Len                                  (1U)
#define UART_IER_EDSSI_Msk                                  (0x1U << UART_IER_EDSSI_Pos)
#define UART_IER_EDSSI                                      UART_IER_EDSSI_Msk  /**< Enable Modem Status Interrupt */

#define UART_IER_ERLS_Pos                                   (2U)
#define UART_IER_ERLS_Len                                   (1U)
#define UART_IER_ERLS_Msk                                   (0x1U << UART_IER_ERLS_Pos)
#define UART_IER_ERLS                                       UART_IER_ERLS_Msk   /**< Enable Receiver Line Status Interrupt */

#define UART_IER_ETBEI_Pos                                  (1U)
#define UART_IER_ETBEI_Len                                  (1U)
#define UART_IER_ETBEI_Msk                                  (0x1U << UART_IER_ETBEI_Pos)
#define UART_IER_ETBEI                                      UART_IER_ETBEI_Msk  /**< Enable Transmit Holding Register Empty Interrupt */

#define UART_IER_ERBFI_Pos                                  (0U)
#define UART_IER_ERBFI_Len                                  (1U)
#define UART_IER_ERBFI_Msk                                  (0x1U << UART_IER_ERBFI_Pos)
#define UART_IER_ERBFI                                      UART_IER_ERBFI_Msk  /**< Enable Received Data Available Interrupt */

/*******************  Bit definition for UART_FCR register  *******************/
#define UART_FCR_RT_Pos                                     (6U)
#define UART_FCR_RT_Len                                     (2U)
#define UART_FCR_RT_Msk                                     (0x3U << UART_FCR_RT_Pos)
#define UART_FCR_RT                                         UART_FCR_RT_Msk             /**< RCVR Trigger */
#define UART_FCR_RT_CHAR_1                                  (0x0U << UART_FCR_RT_Pos)  /**< RX FIFO 1 Char */
#define UART_FCR_RT_QUARTER_FULL                            (0x1U << UART_FCR_RT_Pos)  /**< RX FIFO Quater Full*/
#define UART_FCR_RT_HALF_FULL                               (0x2U << UART_FCR_RT_Pos)  /**< RX FIFO Half Full */
#define UART_FCR_RT_FULL_2                                  (0x3U << UART_FCR_RT_Pos)  /**< RX FIFO 2 less than Full */

#define UART_FCR_TET_Pos                                    (4U)
#define UART_FCR_TET_Len                                    (2U)
#define UART_FCR_TET_Msk                                    (0x3U << UART_FCR_TET_Pos)
#define UART_FCR_TET                                        UART_FCR_TET_Msk            /**< TX Empty Trigger */
#define UART_FCR_TET_EMPTY                                  (0x0U << UART_FCR_TET_Pos)   /**< TX FIFO Empty */
#define UART_FCR_TET_CHAR_2                                 (0x1U << UART_FCR_TET_Pos)   /**< TX FIFO 2 chars */
#define UART_FCR_TET_QUARTER_FULL                           (0x2U << UART_FCR_TET_Pos)   /**< TX FIFO Quater Full */
#define UART_FCR_TET_HALF_FULL                              (0x3U << UART_FCR_TET_Pos)   /**< TX FIFO Half Full */

#define UART_FCR_XFIFOR_Pos                                 (2U)
#define UART_FCR_XFIFOR_Len                                 (1U)
#define UART_FCR_XFIFOR_Msk                                 (0x1U << UART_FCR_XFIFOR_Pos)
#define UART_FCR_XFIFOR                                     UART_FCR_XFIFOR_Msk /**< XMIT FIFO Reset */

#define UART_FCR_RFIFOR_Pos                                 (1U)
#define UART_FCR_RFIFOR_Len                                 (1U)
#define UART_FCR_RFIFOR_Msk                                 (0x1U << UART_FCR_RFIFOR_Pos)
#define UART_FCR_RFIFOR                                     UART_FCR_RFIFOR_Msk /**< RCVR FIFO Reset */

#define UART_FCR_FIFOE_Pos                                  (0U)
#define UART_FCR_FIFOE_Len                                  (1U)
#define UART_FCR_FIFOE_Msk                                  (0x1U << UART_FCR_FIFOE_Pos)
#define UART_FCR_FIFOE                                      UART_FCR_FIFOE_Msk  /**< FIFO Enable */

/*******************  Bit definition for UART_IIR register  *******************/
#define UART_IIR_IID_Pos                                    (0U)
#define UART_IIR_IID_Len                                    (4U)
#define UART_IIR_IID_Msk                                    (0xFU << UART_IIR_IID_Pos)
#define UART_IIR_IID                                        UART_IIR_IID_Msk            /**< Interrupt ID */
#define UART_IIR_IID_MS                                     (0x0U << UART_IIR_IID_Pos)  /**< Modem Status */
#define UART_IIR_IID_NIP                                    (0x1U << UART_IIR_IID_Pos)  /**< No Interrupt Pending */
#define UART_IIR_IID_THRE                                   (0x2U << UART_IIR_IID_Pos)  /**< THR Empty */
#define UART_IIR_IID_RDA                                    (0x4U << UART_IIR_IID_Pos)  /**< Received Data Available */
#define UART_IIR_IID_RLS                                    (0x6U << UART_IIR_IID_Pos)  /**< Receiver Line Status */
#define UART_IIR_IID_CTO                                    (0xCU << UART_IIR_IID_Pos)  /**< Character Timeout */

/*******************  Bit definition for UART_LCR register  *******************/
#define UART_LCR_DLAB_Pos                                   (7U)
#define UART_LCR_DLAB_Len                                   (1U)
#define UART_LCR_DLAB_Msk                                   (0x1U << UART_LCR_DLAB_Pos)
#define UART_LCR_DLAB                                       UART_LCR_DLAB_Msk           /**< Divisor Latch Access */

#define UART_LCR_BC_Pos                                     (6U)
#define UART_LCR_BC_Len                                     (1U)
#define UART_LCR_BC_Msk                                     (0x1U << UART_LCR_BC_Pos)
#define UART_LCR_BC                                         UART_LCR_BC_Msk             /**< Break Control */

#define UART_LCR_PARITY_Pos                                 (3U)
#define UART_LCR_PARITY_Len                                 (3U)
#define UART_LCR_PARITY_Msk                                 (0x7U << UART_LCR_PARITY_Pos)
#define UART_LCR_PARITY                                     UART_LCR_PARITY_Msk             /**< Parity, SP,EPS,PEN bits */
#define UART_LCR_PARITY_NONE                                (0x0U << UART_LCR_PARITY_Pos)   /**< Parity none */
#define UART_LCR_PARITY_ODD                                 (0x1U << UART_LCR_PARITY_Pos)   /**< Parity odd */
#define UART_LCR_PARITY_EVEN                                (0x3U << UART_LCR_PARITY_Pos)   /**< Parity even */
#define UART_LCR_PARITY_SP0                                 (0x5U << UART_LCR_PARITY_Pos)   /**< Parity stick 0 */
#define UART_LCR_PARITY_SP1                                 (0x7U << UART_LCR_PARITY_Pos)   /**< Parity stick 1 */

#define UART_LCR_STOP_Pos                                   (2U)
#define UART_LCR_STOP_Msk                                   (0x1U << UART_LCR_STOP_Pos)
#define UART_LCR_STOP                                       UART_LCR_STOP_Msk               /**< Stop bit */
#define UART_LCR_STOP_1                                     (0x0U << UART_LCR_STOP_Pos)     /**< Stop bit 1 */
#define UART_LCR_STOP_1_5                                   (0x1U << UART_LCR_STOP_Pos)     /**< Stop bit 1.5 (DLS = 0) */
#define UART_LCR_STOP_2                                     (0x1U << UART_LCR_STOP_Pos)     /**< Stop bit 2 (DLS != 0) */

#define UART_LCR_DLS_Pos                                    (0U)
#define UART_LCR_DLS_Msk                                    (0x3U << UART_LCR_DLS_Pos)
#define UART_LCR_DLS                                        UART_LCR_DLS_Msk                /**< Data Length Select */
#define UART_LCR_DLS_5                                      (0x0U << UART_LCR_DLS_Pos)      /**< Data bits 5 */
#define UART_LCR_DLS_6                                      (0x1U << UART_LCR_DLS_Pos)      /**< Data bits 6 */
#define UART_LCR_DLS_7                                      (0x2U << UART_LCR_DLS_Pos)      /**< Data bits 7 */
#define UART_LCR_DLS_8                                      (0x3U << UART_LCR_DLS_Pos)      /**< Data bits 8 */

/*******************  Bit definition for UART_MCR register  *******************/
#define UART_MCR_SIRE_Pos                                   (6U)
#define UART_MCR_SIRE_Len                                   (1U)
#define UART_MCR_SIRE_Msk                                   (0x1 << UART_MCR_SIRE_Pos)
#define UART_MCR_SIRE                                       UART_MCR_SIRE_Msk      /**< SIR mode enable */

#define UART_MCR_AFCE_Pos                                   (5U)
#define UART_MCR_AFCE_Len                                   (1U)
#define UART_MCR_AFCE_Msk                                   (0x1U << UART_MCR_AFCE_Pos)
#define UART_MCR_AFCE                                       UART_MCR_AFCE_Msk       /**< Auto flow contrl enable */

#define UART_MCR_LOOPBACK_Pos                               (4U)
#define UART_MCR_LOOPBACK_Len                               (1U)
#define UART_MCR_LOOPBACK_Msk                               (0x1U << UART_MCR_LOOPBACK_Pos)
#define UART_MCR_LOOPBACK                                   UART_MCR_LOOPBACK_Msk   /**< LoopBack */

#define UART_MCR_RTS_Pos                                    (1U)
#define UART_MCR_RTS_Len                                    (1U)
#define UART_MCR_RTS_Msk                                    (0x1U << UART_MCR_RTS_Pos)
#define UART_MCR_RTS                                        UART_MCR_RTS_Msk        /**< Request To Send */

/*******************  Bit definition for UART_LSR register  *******************/
#define UART_LSR_RFE_Pos                                    (7U)
#define UART_LSR_RFE_Len                                    (1U)
#define UART_LSR_RFE_Msk                                    (0x1U << UART_LSR_RFE_Pos)
#define UART_LSR_RFE                                        UART_LSR_RFE_Msk    /**< Receiver FIFO Error */

#define UART_LSR_TEMT_Pos                                   (6U)
#define UART_LSR_TEMT_Len                                   (1U)
#define UART_LSR_TEMT_Msk                                   (0x1U << UART_LSR_TEMT_Pos)
#define UART_LSR_TEMT                                       UART_LSR_TEMT_Msk   /**< Transmitter Empty */

#define UART_LSR_THRE_Pos                                   (5U)
#define UART_LSR_THRE_Len                                   (1U)
#define UART_LSR_THRE_Msk                                   (0x1U << UART_LSR_THRE_Pos)
#define UART_LSR_THRE                                       UART_LSR_THRE_Msk   /**< Transmit Holding Register Empty */

#define UART_LSR_BI_Pos                                     (4U)
#define UART_LSR_BI_Len                                     (1U)
#define UART_LSR_BI_Msk                                     (0x1U << UART_LSR_BI_Pos)
#define UART_LSR_BI                                         UART_LSR_BI_Msk     /**< Break Interrupt */

#define UART_LSR_FE_Pos                                     (3U)
#define UART_LSR_FE_Len                                     (1U)
#define UART_LSR_FE_Msk                                     (0x1U << UART_LSR_FE_Pos)
#define UART_LSR_FE                                         UART_LSR_FE_Msk     /**< Framing Error */

#define UART_LSR_PE_Pos                                     (2U)
#define UART_LSR_PE_Len                                     (1U)
#define UART_LSR_PE_Msk                                     (0x1U << UART_LSR_PE_Pos)
#define UART_LSR_PE                                         UART_LSR_PE_Msk     /**< Parity Error */

#define UART_LSR_OE_Pos                                     (1U)
#define UART_LSR_OE_Len                                     (1U)
#define UART_LSR_OE_Msk                                     (0x1U << UART_LSR_OE_Pos)
#define UART_LSR_OE                                         UART_LSR_OE_Msk     /**< Overrun error */

#define UART_LSR_DR_Pos                                     (0U)
#define UART_LSR_DR_Msk                                     (0x1U << UART_LSR_DR_Pos)
#define UART_LSR_DR                                         UART_LSR_DR_Msk     /**< Data Ready */

/*******************  Bit definition for UART_MSR register  *******************/
#define UART_MSR_CTS_Pos                                    (4U)
#define UART_MSR_CTS_Len                                    (1U)
#define UART_MSR_CTS_Msk                                    (0x1U << UART_MSR_CTS_Pos)
#define UART_MSR_CTS                                        UART_MSR_CTS_Msk    /**< Clear To Send */

#define UART_MSR_DCTS_Pos                                   (0U)
#define UART_MSR_DCTS_Len                                   (1U)
#define UART_MSR_DCTS_Msk                                   (0x1U << UART_MSR_DCTS_Pos)
#define UART_MSR_DCTS                                       UART_MSR_DCTS_Msk   /**< Delta Clear To Send */

/*******************  Bit definition for UART_USR register  *******************/
#define UART_USR_RFF_Pos                                    (4U)
#define UART_USR_RFF_Len                                    (1U)
#define UART_USR_RFF_Msk                                    (0x1U << UART_USR_RFF_Pos)
#define UART_USR_RFF                                        UART_USR_RFF_Msk    /**< Receive FIFO Full */

#define UART_USR_RFNE_Pos                                   (3U)
#define UART_USR_RFNE_Len                                   (1U)
#define UART_USR_RFNE_Msk                                   (0x1U << UART_USR_RFNE_Pos)
#define UART_USR_RFNE                                       UART_USR_RFNE_Msk   /**< Receive FIFO Not Empty */

#define UART_USR_TFE_Pos                                    (2U)
#define UART_USR_TFE_Len                                    (1U)
#define UART_USR_TFE_Msk                                    (0x1U << UART_USR_TFE_Pos)
#define UART_USR_TFE                                        UART_USR_TFE_Msk    /**< Transmit FIFO Empty */

#define UART_USR_TFNF_Pos                                   (1U)
#define UART_USR_TFNF_Len                                   (1U)
#define UART_USR_TFNF_Msk                                   (0x1U << UART_USR_TFNF_Pos)
#define UART_USR_TFNF                                       UART_USR_TFNF_Msk   /**< Transmit FIFO Not Full */

/*******************  Bit definition for UART_TFL register  *******************/
/* Transmit FIFO Level bits */
#define UART_TFL_TFL_Pos                                    (0U)
#define UART_TFL_TFL_Len                                    (7U)
#define UART_TFL_TFL_Msk                                    (0x7FU << UART_TFL_TFL_Pos)
#define UART_TFL_TFL                                        UART_TFL_TFL_Msk    /**< Transmit FIFO Level */

/*******************  Bit definition for UART_RFL register  *******************/
/* Receive FIFO Level bits */
#define UART_RFL_RFL_Pos                                    (0U)
#define UART_RFL_RFL_Len                                    (7U)
#define UART_RFL_RFL_Msk                                    (0x7FU << UART_RFL_RFL_Pos)
#define UART_RFL_RFL                                        UART_RFL_RFL_Msk    /**< Receive FIFO Level */

/*******************  Bit definition for UART_SRR register  *******************/
/* XMIT FIFO Reset bit */
#define UART_SRR_XFR_Pos                                    (2U)
#define UART_SRR_XFR_Len                                    (1U)
#define UART_SRR_XFR_Msk                                    (0x1U << UART_SRR_XFR_Pos)
#define UART_SRR_XFR                                        UART_SRR_XFR_Msk    /**< XMIT FIFO Reset */

/* RCVR FIFO Reset bit */
#define UART_SRR_RFR_Pos                                    (1U)
#define UART_SRR_RFR_Len                                    (1U)
#define UART_SRR_RFR_Msk                                    (0x1U << UART_SRR_RFR_Pos)
#define UART_SRR_RFR                                        UART_SRR_RFR_Msk    /**< RCVR FIFO Reset */

/* UART Reset Enable bit */
#define UART_SRR_UR_Pos                                     (0U)
#define UART_SRR_UR_Len                                     (1U)
#define UART_SRR_UR_Msk                                     (0x1U << UART_SRR_UR_Pos)
#define UART_SRR_UR                                         UART_SRR_UR_Msk     /**< UART Reset */

/*******************  Bit definition for UART_SRTS register  *******************/
#define UART_SRTS_SRTS_Pos                                  (0U)
#define UART_SRTS_SRTS_Len                                  (1U)
#define UART_SRTS_SRTS_Msk                                  (0x1U << UART_SRTS_SRTS_Pos)
#define UART_SRTS_SRTS                                      UART_SRTS_SRTS_Msk  /**< Shadow Request to Send */

/*******************  Bit definition for UART_SBCR register  *******************/
#define UART_SBCR_SBCR_Pos                                  (0U)
#define UART_SBCR_SBCR_Len                                  (1U)
#define UART_SBCR_SBCR_Msk                                  (0x1U << UART_SBCR_SBCR_Pos)
#define UART_SBCR_SBCR                                      UART_SBCR_SBCR_Msk  /**< Shadow Break Control */

/*******************  Bit definition for UART_SFE register  *******************/
#define UART_SFE_SFE_Pos                                    (0U)
#define UART_SFE_SFE_Len                                    (1U)
#define UART_SFE_SFE_Msk                                    (0x1U << UART_SFE_SFE_Pos)
#define UART_SFE_SFE                                        UART_SFE_SFE_Msk    /**< Shadow FIFO Enable */

/*******************  Bit definition for UART_SRT register  *******************/
#define UART_SRT_SRT_Pos                                    (0U)
#define UART_SRT_SRT_Len                                    (2U)
#define UART_SRT_SRT_Msk                                    (0x3U << UART_SRT_SRT_Pos)
#define UART_SRT_SRT                                        UART_SRT_SRT_Msk
#define UART_SRT_SRT_CHAR_1                                 (0x0U << UART_SRT_SRT_Pos)  /**< RX FIFO 1 Char */
#define UART_SRT_SRT_QUARTER_FULL                           (0x1U << UART_SRT_SRT_Pos)  /**< RX FIFO Quater Full*/
#define UART_SRT_SRT_HALF_FULL                              (0x2U << UART_SRT_SRT_Pos)  /**< RX FIFO Half Full */
#define UART_SRT_SRT_FULL_2                                 (0x3U << UART_SRT_SRT_Pos)  /**< RX FIFO 2 less than Full */

/*******************  Bit definition for UART_STET register  *******************/
#define UART_STET_STET_Pos                                  (0U)
#define UART_STET_STET_Len                                  (2U)
#define UART_STET_STET_Msk                                  (0x3U << UART_STET_STET_Pos)
#define UART_STET_STET                                      UART_STET_STET_Msk
#define UART_STET_STET_EMPTY                                (0x0U << UART_STET_STET_Pos)    /**< TX FIFO Empty */
#define UART_STET_STET_CHAR_2                               (0x1U << UART_STET_STET_Pos)    /**< TX FIFO 2 chars */
#define UART_STET_STET_QUARTER_FULL                         (0x2U << UART_STET_STET_Pos)    /**< TX FIFO Quater Full */
#define UART_STET_STET_HALF_FULL                            (0x3U << UART_STET_STET_Pos)    /**< TX FIFO Half Full */

/*******************  Bit definition for UART_HTX register  *******************/
#define UART_HTX_HTX_Pos                                    (0U)
#define UART_HTX_HTX_Len                                    (1U)
#define UART_HTX_HTX_Msk                                    (0x1U << UART_HTX_HTX_Pos)
#define UART_HTX_HTX                                        UART_HTX_HTX_Msk    /**< Halt TX */

/*******************  Bit definition for UART_DLF register  *******************/
#define UART_DLF_DLF_Pos                                    (0U)
#define UART_DLF_DLF_Len                                    (1U)
#define UART_DLF_DLF_Msk                                    (0x1U << UART_DLF_DLF_Pos)
#define UART_DLF_DLF                                        UART_DLF_DLF_Msk    /**< Fractional part of divisor */

/* ================================================================================================================= */
/* ================                                        USB                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for USB_CTRL register  *******************/
#define USB_CTRL_MCU_ENUM_Pos                               (0U)
#define USB_CTRL_MCU_ENUM_Len                               (1U)
#define USB_CTRL_MCU_ENUM_Msk                               (0x1UL << USB_CTRL_MCU_ENUM_Pos)
#define USB_CTRL_MCU_ENUM                                   USB_CTRL_MCU_ENUM_Msk

#define USB_CTRL_EP0_OUT_DATA_RDY_Pos                       (1U)
#define USB_CTRL_EP0_OUT_DATA_RDY_Len                       (1U)
#define USB_CTRL_EP0_OUT_DATA_RDY_Msk                       (0x1UL << USB_CTRL_EP0_OUT_DATA_RDY_Pos)
#define USB_CTRL_EP0_OUT_DATA_RDY                           USB_CTRL_EP0_OUT_DATA_RDY_Msk

#define USB_CTRL_MCU_WAKEUP_Pos                             (2U)
#define USB_CTRL_MCU_WAKEUP_Len                             (1U)
#define USB_CTRL_MCU_WAKEUP_Msk                             (0x1UL << USB_CTRL_MCU_WAKEUP_Pos)
#define USB_CTRL_MCU_WAKEUP                                 USB_CTRL_MCU_WAKEUP_Msk

#define USB_CTRL_DEV_REMOTE_WAKEUP_Pos                      (3U)
#define USB_CTRL_DEV_REMOTE_WAKEUP_Len                      (1U)
#define USB_CTRL_DEV_REMOTE_WAKEUP_Msk                      (0x1UL << USB_CTRL_DEV_REMOTE_WAKEUP_Pos)
#define USB_CTRL_DEV_REMOTE_WAKEUP                          USB_CTRL_DEV_REMOTE_WAKEUP_Msk

#define USB_CTRL_ADDR_STAT_Pos                              (4U)
#define USB_CTRL_ADDR_STAT_Len                              (1U)
#define USB_CTRL_ADDR_STAT_Msk                              (0x1UL << USB_CTRL_ADDR_STAT_Pos)
#define USB_CTRL_ADDR_STAT                                  USB_CTRL_ADDR_STAT_Msk

#define USB_CTRL_CFG_STAT_Pos                               (5U)
#define USB_CTRL_CFG_STAT_Len                               (1U)
#define USB_CTRL_CFG_STAT_Msk                               (0x1UL << USB_CTRL_CFG_STAT_Pos)
#define USB_CTRL_CFG_STAT                                   USB_CTRL_CFG_STAT_Msk

#define USB_CTRL_CMD_OK_Pos                                 (6U)
#define USB_CTRL_CMD_OK_Len                                 (1U)
#define USB_CTRL_CMD_OK_Msk                                 (0x1UL << USB_CTRL_CMD_OK_Pos)
#define USB_CTRL_CMD_OK                                     USB_CTRL_CMD_OK_Msk

#define USB_CTRL_CMD_ERR_Pos                                (7U)
#define USB_CTRL_CMD_ERR_Len                                (1U)
#define USB_CTRL_CMD_ERR_Msk                                (0x1UL << USB_CTRL_CMD_ERR_Pos)
#define USB_CTRL_CMD_ERR                                    USB_CTRL_CMD_ERR_Msk

#define USB_CTRL_FUNC_ADDR_Pos                              (8U)
#define USB_CTRL_FUNC_ADDR_Len                              (7U)
#define USB_CTRL_FUNC_ADDR_Msk                              (0x7FUL << USB_CTRL_FUNC_ADDR_Pos)
#define USB_CTRL_FUNC_ADDR                                  USB_CTRL_FUNC_ADDR_Msk

#define USB_CTRL_EP1_OUT_DATA_RDY_Pos                       (15U)
#define USB_CTRL_EP1_OUT_DATA_RDY_Len                       (1U)
#define USB_CTRL_EP1_OUT_DATA_RDY_Msk                       (0x1UL << USB_CTRL_EP1_OUT_DATA_RDY_Pos)
#define USB_CTRL_EP1_OUT_DATA_RDY                           USB_CTRL_EP1_OUT_DATA_RDY_Msk

#define USB_CTRL_EP0_FIFO_SWITCH_Pos                        (16U)
#define USB_CTRL_EP0_FIFO_SWITCH_Len                        (1U)
#define USB_CTRL_EP0_FIFO_SWITCH_Msk                        (0x1UL << USB_CTRL_EP0_FIFO_SWITCH_Pos)
#define USB_CTRL_EP0_FIFO_SWITCH                            USB_CTRL_EP0_FIFO_SWITCH_Msk

#define USB_CTRL_EP5_OUT_DATA_RDY_Pos                       (17U)
#define USB_CTRL_EP5_OUT_DATA_RDY_Len                       (1U)
#define USB_CTRL_EP5_OUT_DATA_RDY_Msk                       (0x1UL << USB_CTRL_EP5_OUT_DATA_RDY_Pos)
#define USB_CTRL_EP5_OUT_DATA_RDY                           USB_CTRL_EP5_OUT_DATA_RDY_Msk

/*******************  Bit definition for USB_EP0_1_CTRL register  *******************/
#define USB_EP0_CTRL_DATA_RDY_Pos                           (0U)
#define USB_EP0_CTRL_DATA_RDY_Len                           (1U)
#define USB_EP0_CTRL_DATA_RDY_Msk                           (0x1UL << USB_EP0_CTRL_DATA_RDY_Pos)
#define USB_EP0_CTRL_DATA_RDY                               USB_EP0_CTRL_DATA_RDY_Msk

#define USB_EP0_CTRL_IFIFO_CLR_Pos                          (1U)
#define USB_EP0_CTRL_IFIFO_CLR_Len                          (1U)
#define USB_EP0_CTRL_IFIFO_CLR_Msk                          (0x1UL << USB_EP0_CTRL_IFIFO_CLR_Pos)
#define USB_EP0_CTRL_IFIFO_CLR                              USB_EP0_CTRL_IFIFO_CLR_Msk

#define USB_EP1_CTRL_IFIFO_CLR_Pos                          (2U)
#define USB_EP1_CTRL_IFIFO_CLR_Len                          (1U)
#define USB_EP1_CTRL_IFIFO_CLR_Msk                          (0x1UL << USB_EP1_CTRL_IFIFO_CLR_Pos)
#define USB_EP1_CTRL_IFIFO_CLR                              USB_EP1_CTRL_IFIFO_CLR_Msk

/*******************  Bit definition for USB_EP2_CTRL register  *******************/
#define USB_EP2_CTRL_DATA_RDY_Pos                           (0U)
#define USB_EP2_CTRL_DATA_RDY_Len                           (1U)
#define USB_EP2_CTRL_DATA_RDY_Msk                           (0x1UL << USB_EP2_CTRL_DATA_RDY_Pos)
#define USB_EP2_CTRL_DATA_RDY                               USB_EP2_CTRL_DATA_RDY_Msk

#define USB_EP2_CTRL_IFIFO_CLR_Pos                          (1U)
#define USB_EP2_CTRL_IFIFO_CLR_Len                          (1U)
#define USB_EP2_CTRL_IFIFO_CLR_Msk                          (0x1UL << USB_EP2_CTRL_IFIFO_CLR_Pos)
#define USB_EP2_CTRL_IFIFO_CLR                              USB_EP2_CTRL_IFIFO_CLR_Msk

/*******************  Bit definition for USB_EP3_CTRL register  *******************/
#define USB_EP3_CTRL_DATA_RDY_Pos                           (0U)
#define USB_EP3_CTRL_DATA_RDY_Len                           (1U)
#define USB_EP3_CTRL_DATA_RDY_Msk                           (0x1UL << USB_EP3_CTRL_DATA_RDY_Pos)
#define USB_EP3_CTRL_DATA_RDY                               USB_EP3_CTRL_DATA_RDY_Msk

#define USB_EP3_CTRL_IFIFO_CLR_Pos                          (1U)
#define USB_EP3_CTRL_IFIFO_CLR_Len                          (1U)
#define USB_EP3_CTRL_IFIFO_CLR_Msk                          (0x1UL << USB_EP3_CTRL_IFIFO_CLR_Pos)
#define USB_EP3_CTRL_IFIFO_CLR                              USB_EP3_CTRL_IFIFO_CLR_Msk

/*******************  Bit definition for USB_EP_ATTR register  *******************/
#define USB_EP_ATTR_EP1_Pos                                 (0U)
#define USB_EP_ATTR_EP1_Len                                 (2U)
#define USB_EP_ATTR_EP1_Msk                                 (0x3UL << USB_EP_ATTR_EP1_Pos)
#define USB_EP_ATTR_EP1                                     USB_EP_ATTR_EP1_Msk
#define USB_EP_ATTR_EP1_INT                                 (0x0UL << USB_EP_ATTR_EP1_Pos)
#define USB_EP_ATTR_EP1_ISO                                 (0x1UL << USB_EP_ATTR_EP1_Pos)
#define USB_EP_ATTR_EP1_BULK                                (0x2UL << USB_EP_ATTR_EP1_Pos)

#define USB_EP_ATTR_EP1_HALT_MCU_Pos                        (2U)
#define USB_EP_ATTR_EP1_HALT_MCU_Len                        (1U)
#define USB_EP_ATTR_EP1_HALT_MCU_Msk                        (0x1UL << USB_EP_ATTR_EP1_HALT_MCU_Pos)
#define USB_EP_ATTR_EP1_HALT_MCU                            USB_EP_ATTR_EP1_HALT_MCU_Msk

#define USB_EP_ATTR_EP2_Pos                                 (4U)
#define USB_EP_ATTR_EP2_Len                                 (2U)
#define USB_EP_ATTR_EP2_Msk                                 (0x3UL << USB_EP_ATTR_EP2_Pos)
#define USB_EP_ATTR_EP2                                     USB_EP_ATTR_EP2_Msk
#define USB_EP_ATTR_EP2_INT                                 (0x0UL << USB_EP_ATTR_EP2_Pos)
#define USB_EP_ATTR_EP2_ISO                                 (0x1UL << USB_EP_ATTR_EP2_Pos)
#define USB_EP_ATTR_EP2_BULK                                (0x2UL << USB_EP_ATTR_EP2_Pos)

#define USB_EP_ATTR_EP2_HALT_MCU_Pos                        (6U)
#define USB_EP_ATTR_EP2_HALT_MCU_Len                        (1U)
#define USB_EP_ATTR_EP2_HALT_MCU_Msk                        (0x1UL << USB_EP_ATTR_EP2_HALT_MCU_Pos)
#define USB_EP_ATTR_EP2_HALT_MCU                            USB_EP_ATTR_EP2_HALT_MCU_Msk

#define USB_EP_ATTR_EP3_Pos                                 (8U)
#define USB_EP_ATTR_EP3_Len                                 (2U)
#define USB_EP_ATTR_EP3_Msk                                 (0x3UL << USB_EP_ATTR_EP3_Pos)
#define USB_EP_ATTR_EP3                                     USB_EP_ATTR_EP3_Msk
#define USB_EP_ATTR_EP3_INT                                 (0x0UL << USB_EP_ATTR_EP3_Pos)
#define USB_EP_ATTR_EP3_ISO                                 (0x1UL << USB_EP_ATTR_EP3_Pos)
#define USB_EP_ATTR_EP3_BULK                                (0x2UL << USB_EP_ATTR_EP3_Pos)

#define USB_EP_ATTR_EP3_HALT_MCU_Pos                        (10U)
#define USB_EP_ATTR_EP3_HALT_MCU_Len                        (1U)
#define USB_EP_ATTR_EP3_HALT_MCU_Msk                        (0x1UL << USB_EP_ATTR_EP3_HALT_MCU_Pos)
#define USB_EP_ATTR_EP3_HALT_MCU                            USB_EP_ATTR_EP3_HALT_MCU_Msk

/*******************  Bit definition for USB_INT_STAT register  *******************/
#define USB_INT_STAT_ALL_Pos                                (0U)
#define USB_INT_STAT_ALL_Len                                (26U)
#define USB_INT_STAT_ALL_Msk                                (0x3FFFFFF << USB_INT_STAT_ALL_Pos)
#define USB_INT_STAT_ALL                                    USB_INT_STAT_ALL_Msk

#define USB_INT_STAT_SUSPEND_Pos                            (0U)
#define USB_INT_STAT_SUSPEND_Len                            (1U)
#define USB_INT_STAT_SUSPEND_Msk                            (0x1UL << USB_INT_STAT_SUSPEND_Pos)
#define USB_INT_STAT_SUSPEND                                USB_INT_STAT_SUSPEND_Msk

#define USB_INT_STAT_EP0_OUT_READY_Pos                      (1U)
#define USB_INT_STAT_EP0_OUT_READY_Len                      (1U)
#define USB_INT_STAT_EP0_OUT_READY_Msk                      (0x1UL << USB_INT_STAT_EP0_OUT_READY_Pos)
#define USB_INT_STAT_EP0_OUT_READY                          USB_INT_STAT_EP0_OUT_READY_Msk

#define USB_INT_STAT_EP1_OUT_READY_Pos                      (2U)
#define USB_INT_STAT_EP1_OUT_READY_Len                      (1U)
#define USB_INT_STAT_EP1_OUT_READY_Msk                      (0x1UL << USB_INT_STAT_EP1_OUT_READY_Pos)
#define USB_INT_STAT_EP1_OUT_READY                          USB_INT_STAT_EP1_OUT_READY_Msk

#define USB_INT_STAT_CRC16_ERR_Pos                          (3U)
#define USB_INT_STAT_CRC16_ERR_Len                          (1U)
#define USB_INT_STAT_CRC16_ERR_Msk                          (0x1UL << USB_INT_STAT_CRC16_ERR_Pos)
#define USB_INT_STAT_CRC16_ERR                              USB_INT_STAT_CRC16_ERR_Msk

#define USB_INT_STAT_UPID_ERR_Pos                           (4U)
#define USB_INT_STAT_UPID_ERR_Len                           (1U)
#define USB_INT_STAT_UPID_ERR_Msk                           (0x1UL << USB_INT_STAT_UPID_ERR_Pos)
#define USB_INT_STAT_UPID_ERR                               USB_INT_STAT_UPID_ERR_Msk

#define USB_INT_STAT_TIMEOUT_ERR_Pos                        (5U)
#define USB_INT_STAT_TIMEOUT_ERR_Len                        (1U)
#define USB_INT_STAT_TIMEOUT_ERR_Msk                        (0x1UL << USB_INT_STAT_TIMEOUT_ERR_Pos)
#define USB_INT_STAT_TIMEOUT_ERR                            USB_INT_STAT_TIMEOUT_ERR_Msk

#define USB_INT_STAT_SEQ_ERR_Pos                            (6U)
#define USB_INT_STAT_SEQ_ERR_Len                            (1U)
#define USB_INT_STAT_SEQ_ERR_Msk                            (0x1UL << USB_INT_STAT_SEQ_ERR_Pos)
#define USB_INT_STAT_SEQ_ERR                                USB_INT_STAT_SEQ_ERR_Msk

#define USB_INT_STAT_PID_CKS_ERR_Pos                        (7U)
#define USB_INT_STAT_PID_CKS_ERR_Len                        (1U)
#define USB_INT_STAT_PID_CKS_ERR_Msk                        (0x1UL << USB_INT_STAT_PID_CKS_ERR_Pos)
#define USB_INT_STAT_PID_CKS_ERR                            USB_INT_STAT_PID_CKS_ERR_Msk

#define USB_INT_STAT_PID_CRC_ERR_Pos                        (8U)
#define USB_INT_STAT_PID_CRC_ERR_Len                        (1U)
#define USB_INT_STAT_PID_CRC_ERR_Msk                        (0x1UL << USB_INT_STAT_PID_CRC_ERR_Pos)
#define USB_INT_STAT_PID_CRC_ERR                            USB_INT_STAT_PID_CRC_ERR_Msk

#define USB_INT_STAT_HOST_RESET_Pos                         (9U)
#define USB_INT_STAT_HOST_RESET_Len                         (1U)
#define USB_INT_STAT_HOST_RESET_Msk                         (0x1UL << USB_INT_STAT_HOST_RESET_Pos)
#define USB_INT_STAT_HOST_RESET                             USB_INT_STAT_HOST_RESET_Msk

#define USB_INT_STAT_AHB_XFER_ERR_Pos                       (10U)
#define USB_INT_STAT_AHB_XFER_ERR_Len                       (1U)
#define USB_INT_STAT_AHB_XFER_ERR_Msk                       (0x1UL << USB_INT_STAT_AHB_XFER_ERR_Pos)
#define USB_INT_STAT_AHB_XFER_ERR                           USB_INT_STAT_AHB_XFER_ERR_Msk

#define USB_INT_STAT_NSE_ERR_Pos                            (11U)
#define USB_INT_STAT_NSE_ERR_Len                            (1U)
#define USB_INT_STAT_NSE_ERR_Msk                            (0x1UL << USB_INT_STAT_NSE_ERR_Pos)
#define USB_INT_STAT_NSE_ERR                                USB_INT_STAT_NSE_ERR_Msk

#define USB_INT_STAT_EP3_AHB_XFER_DONE_Pos                  (12U)
#define USB_INT_STAT_EP3_AHB_XFER_DONE_Len                  (1U)
#define USB_INT_STAT_EP3_AHB_XFER_DONE_Msk                  (0x1UL << USB_INT_STAT_EP3_AHB_XFER_DONE_Pos)
#define USB_INT_STAT_EP3_AHB_XFER_DONE                      USB_INT_STAT_EP3_AHB_XFER_DONE_Msk

#define USB_INT_STAT_SYNC_ERR_Pos                           (13U)
#define USB_INT_STAT_SYNC_ERR_Len                           (1U)
#define USB_INT_STAT_SYNC_ERR_Msk                           (0x1UL << USB_INT_STAT_SYNC_ERR_Pos)
#define USB_INT_STAT_SYNC_ERR                               USB_INT_STAT_SYNC_ERR_Msk

#define USB_INT_STAT_BIT_STUFF_ERR_Pos                      (14U)
#define USB_INT_STAT_BIT_STUFF_ERR_Len                      (1U)
#define USB_INT_STAT_BIT_STUFF_ERR_Msk                      (0x1UL << USB_INT_STAT_BIT_STUFF_ERR_Pos)
#define USB_INT_STAT_BIT_STUFF_ERR                          USB_INT_STAT_BIT_STUFF_ERR_Msk

#define USB_INT_STAT_BYTE_ERR_Pos                           (15U)
#define USB_INT_STAT_BYTE_ERR_Len                           (1U)
#define USB_INT_STAT_BYTE_ERR_Msk                           (0x1UL << USB_INT_STAT_BYTE_ERR_Pos)
#define USB_INT_STAT_BYTE_ERR                               USB_INT_STAT_BYTE_ERR_Msk

#define USB_INT_STAT_SOF_Pos                                (16U)
#define USB_INT_STAT_SOF_Len                                (1U)
#define USB_INT_STAT_SOF_Msk                                (0x1UL << USB_INT_STAT_SOF_Pos)
#define USB_INT_STAT_SOF                                    USB_INT_STAT_SOF_Msk

#define USB_INT_STAT_EP0_TX_DONE_Pos                        (17U)
#define USB_INT_STAT_EP0_TX_DONE_Len                        (1U)
#define USB_INT_STAT_EP0_TX_DONE_Msk                        (0x1UL << USB_INT_STAT_EP0_TX_DONE_Pos)
#define USB_INT_STAT_EP0_TX_DONE                            USB_INT_STAT_EP0_TX_DONE_Msk

#define USB_INT_STAT_EP2_TX_DONE_Pos                        (18U)
#define USB_INT_STAT_EP2_TX_DONE_Len                        (1U)
#define USB_INT_STAT_EP2_TX_DONE_Msk                        (0x1UL << USB_INT_STAT_EP2_TX_DONE_Pos)
#define USB_INT_STAT_EP2_TX_DONE                            USB_INT_STAT_EP2_TX_DONE_Msk

#define USB_INT_STAT_EP3_TX_DONE_Pos                        (19U)
#define USB_INT_STAT_EP3_TX_DONE_Len                        (1U)
#define USB_INT_STAT_EP3_TX_DONE_Msk                        (0x1UL << USB_INT_STAT_EP3_TX_DONE_Pos)
#define USB_INT_STAT_EP3_TX_DONE                            USB_INT_STAT_EP3_TX_DONE_Msk

#define USB_INT_STAT_INTO_CONFIG_Pos                        (20U)
#define USB_INT_STAT_INTO_CONFIG_Len                        (1U)
#define USB_INT_STAT_INTO_CONFIG_Msk                        (0x1UL << USB_INT_STAT_INTO_CONFIG_Pos)
#define USB_INT_STAT_INTO_CONFIG                            USB_INT_STAT_INTO_CONFIG_Msk

#define USB_INT_STAT_EP5_OUT_READY_Pos                      (21U)
#define USB_INT_STAT_EP5_OUT_READY_Len                      (1U)
#define USB_INT_STAT_EP5_OUT_READY_Msk                      (0x1UL << USB_INT_STAT_EP5_OUT_READY_Pos)
#define USB_INT_STAT_EP5_OUT_READY                          USB_INT_STAT_EP5_OUT_READY_Msk

#define USB_INT_STAT_EP4_AHB_XFER_DONE_Pos                  (22U)
#define USB_INT_STAT_EP4_AHB_XFER_DONE_Len                  (1U)
#define USB_INT_STAT_EP4_AHB_XFER_DONE_Msk                  (0x1UL << USB_INT_STAT_EP4_AHB_XFER_DONE_Pos)
#define USB_INT_STAT_EP4_AHB_XFER_DONE                      USB_INT_STAT_EP4_AHB_XFER_DONE_Msk

#define USB_INT_STAT_EP4_TX_DONE_Pos                        (23U)
#define USB_INT_STAT_EP4_TX_DONE_Len                        (1U)
#define USB_INT_STAT_EP4_TX_DONE_Msk                        (0x1UL << USB_INT_STAT_EP4_TX_DONE_Pos)
#define USB_INT_STAT_EP4_TX_DONE                            USB_INT_STAT_EP4_TX_DONE_Msk

#define USB_INT_STAT_EP5_AHB_XFER_DONE_Pos                  (24U)
#define USB_INT_STAT_EP5_AHB_XFER_DONE_Len                  (1U)
#define USB_INT_STAT_EP5_AHB_XFER_DONE_Msk                  (0x1UL << USB_INT_STAT_EP5_AHB_XFER_DONE_Pos)
#define USB_INT_STAT_EP5_AHB_XFER_DONE                      USB_INT_STAT_EP5_AHB_XFER_DONE_Msk

#define USB_INT_STAT_EP5_TIMER_OUT_ERR_Pos                  (25U)
#define USB_INT_STAT_EP5_TIMER_OUT_ERR_Len                  (1U)
#define USB_INT_STAT_EP5_TIMER_OUT_ERR_Msk                  (0x1UL << USB_INT_STAT_EP5_TIMER_OUT_ERR_Pos)
#define USB_INT_STAT_EP5_TIMER_OUT_ERR                      USB_INT_STAT_EP5_TIMER_OUT_ERR_Msk

/*******************  Bit definition for USB_INT_EN register  *******************/
#define USB_INT_EN_ALL_Pos                                  (0U)
#define USB_INT_EN_ALL_Len                                  (26U)
#define USB_INT_EN_ALL_Msk                                  (0x3FFFFFF << USB_INT_EN_ALL_Pos)
#define USB_INT_EN_ALL                                      USB_INT_EN_ALL_Msk
#define USB_INT_EN_RESET_VAL                                (0x3EFDFFF << USB_INT_EN_ALL_Pos)

#define USB_INT_EN_SUSPEND_Pos                              (0U)
#define USB_INT_EN_SUSPEND_Len                              (1U)
#define USB_INT_EN_SUSPEND_Msk                              (0x1UL << USB_INT_EN_SUSPEND_Pos)
#define USB_INT_EN_SUSPEND                                  USB_INT_EN_SUSPEND_Msk

#define USB_INT_EN_EP0_OUT_READY_Pos                        (1U)
#define USB_INT_EN_EP0_OUT_READY_Len                        (1U)
#define USB_INT_EN_EP0_OUT_READY_Msk                        (0x1UL << USB_INT_EN_EP0_OUT_READY_Pos)
#define USB_INT_EN_EP0_OUT_READY                            USB_INT_EN_EP0_OUT_READY_Msk

#define USB_INT_EN_EP1_OUT_READY_Pos                        (2U)
#define USB_INT_EN_EP1_OUT_READY_Len                        (1U)
#define USB_INT_EN_EP1_OUT_READY_Msk                        (0x1UL << USB_INT_EN_EP1_OUT_READY_Pos)
#define USB_INT_EN_EP1_OUT_READY                            USB_INT_EN_EP1_OUT_READY_Msk

#define USB_INT_EN_CRC16_ERR_Pos                            (3U)
#define USB_INT_EN_CRC16_ERR_Len                            (1U)
#define USB_INT_EN_CRC16_ERR_Msk                            (0x1UL << USB_INT_EN_CRC16_ERR_Pos)
#define USB_INT_EN_CRC16_ERR                                USB_INT_EN_CRC16_ERR_Msk

#define USB_INT_EN_UPID_ERR_Pos                             (4U)
#define USB_INT_EN_UPID_ERR_Len                             (1U)
#define USB_INT_EN_UPID_ERR_Msk                             (0x1UL << USB_INT_EN_UPID_ERR_Pos)
#define USB_INT_EN_UPID_ERR                                 USB_INT_EN_UPID_ERR_Msk

#define USB_INT_EN_TIMEOUT_ERR_Pos                          (5U)
#define USB_INT_EN_TIMEOUT_ERR_Len                          (1U)
#define USB_INT_EN_TIMEOUT_ERR_Msk                          (0x1UL << USB_INT_EN_TIMEOUT_ERR_Pos)
#define USB_INT_EN_TIMEOUT_ERR                              USB_INT_EN_TIMEOUT_ERR_Msk

#define USB_INT_EN_SEQ_ERR_Pos                              (6U)
#define USB_INT_EN_SEQ_ERR_Len                              (1U)
#define USB_INT_EN_SEQ_ERR_Msk                              (0x1UL << USB_INT_EN_SEQ_ERR_Pos)
#define USB_INT_EN_SEQ_ERR                                  USB_INT_EN_SEQ_ERR_Msk

#define USB_INT_EN_PID_CKS_ERR_Pos                          (7U)
#define USB_INT_EN_PID_CKS_ERR_Len                          (1U)
#define USB_INT_EN_PID_CKS_ERR_Msk                          (0x1UL << USB_INT_EN_PID_CKS_ERR_Pos)
#define USB_INT_EN_PID_CKS_ERR                              USB_INT_EN_PID_CKS_ERR_Msk

#define USB_INT_EN_PID_CRC_ERR_Pos                          (8U)
#define USB_INT_EN_PID_CRC_ERR_Len                          (1U)
#define USB_INT_EN_PID_CRC_ERR_Msk                          (0x1UL << USB_INT_EN_PID_CRC_ERR_Pos)
#define USB_INT_EN_PID_CRC_ERR                              USB_INT_EN_PID_CRC_ERR_Msk

#define USB_INT_EN_HOST_RESET_Pos                           (9U)
#define USB_INT_EN_HOST_RESET_Len                           (1U)
#define USB_INT_EN_HOST_RESET_Msk                           (0x1UL << USB_INT_EN_HOST_RESET_Pos)
#define USB_INT_EN_HOST_RESET                               USB_INT_EN_HOST_RESET_Msk

#define USB_INT_EN_AHB_XFER_ERR_Pos                         (10U)
#define USB_INT_EN_AHB_XFER_ERR_Len                         (1U)
#define USB_INT_EN_AHB_XFER_ERR_Msk                         (0x1UL << USB_INT_EN_AHB_XFER_ERR_Pos)
#define USB_INT_EN_AHB_XFER_ERR                             USB_INT_EN_AHB_XFER_ERR_Msk

#define USB_INT_EN_NSE_ERR_Pos                              (11U)
#define USB_INT_EN_NSE_ERR_Len                              (1U)
#define USB_INT_EN_NSE_ERR_Msk                              (0x1UL << USB_INT_EN_NSE_ERR_Pos)
#define USB_INT_EN_NSE_ERR                                  USB_INT_EN_NSE_ERR_Msk

#define USB_INT_EN_EP3_AHB_XFER_DONE_Pos                    (12U)
#define USB_INT_EN_EP3_AHB_XFER_DONE_Len                    (1U)
#define USB_INT_EN_EP3_AHB_XFER_DONE_Msk                    (0x1UL << USB_INT_EN_EP3_AHB_XFER_DONE_Pos)
#define USB_INT_EN_EP3_AHB_XFER_DONE                        USB_INT_EN_EP3_AHB_XFER_DONE_Msk

#define USB_INT_EN_SYNC_ERR_Pos                             (13U)
#define USB_INT_EN_SYNC_ERR_Len                             (1U)
#define USB_INT_EN_SYNC_ERR_Msk                             (0x1UL << USB_INT_EN_SYNC_ERR_Pos)
#define USB_INT_EN_SYNC_ERR                                 USB_INT_EN_SYNC_ERR_Msk

#define USB_INT_EN_BIT_STUFF_ERR_Pos                        (14U)
#define USB_INT_EN_BIT_STUFF_ERR_Len                        (1U)
#define USB_INT_EN_BIT_STUFF_ERR_Msk                        (0x1UL << USB_INT_EN_BIT_STUFF_ERR_Pos)
#define USB_INT_EN_BIT_STUFF_ERR                            USB_INT_EN_BIT_STUFF_ERR_Msk

#define USB_INT_EN_BYTE_ERR_Pos                             (15U)
#define USB_INT_EN_BYTE_ERR_Len                             (1U)
#define USB_INT_EN_BYTE_ERR_Msk                             (0x1UL << USB_INT_EN_BYTE_ERR_Pos)
#define USB_INT_EN_BYTE_ERR                                 USB_INT_EN_BYTE_ERR_Msk

#define USB_INT_EN_SOF_Pos                                  (16U)
#define USB_INT_EN_SOF_Len                                  (1U)
#define USB_INT_EN_SOF_Msk                                  (0x1UL << USB_INT_EN_SOF_Pos)
#define USB_INT_EN_SOF                                      USB_INT_EN_SOF_Msk

#define USB_INT_EN_EP0_TX_DONE_Pos                          (17U)
#define USB_INT_EN_EP0_TX_DONE_Len                          (1U)
#define USB_INT_EN_EP0_TX_DONE_Msk                          (0x1UL << USB_INT_EN_EP0_TX_DONE_Pos)
#define USB_INT_EN_EP0_TX_DONE                              USB_INT_EN_EP0_TX_DONE_Msk

#define USB_INT_EN_EP2_TX_DONE_Pos                          (18U)
#define USB_INT_EN_EP2_TX_DONE_Len                          (1U)
#define USB_INT_EN_EP2_TX_DONE_Msk                          (0x1UL << USB_INT_EN_EP2_TX_DONE_Pos)
#define USB_INT_EN_EP2_TX_DONE                              USB_INT_EN_EP2_TX_DONE_Msk

#define USB_INT_EN_EP3_TX_DONE_Pos                          (19U)
#define USB_INT_EN_EP3_TX_DONE_Len                          (1U)
#define USB_INT_EN_EP3_TX_DONE_Msk                          (0x1UL << USB_INT_EN_EP3_TX_DONE_Pos)
#define USB_INT_EN_EP3_TX_DONE                              USB_INT_EN_EP3_TX_DONE_Msk

#define USB_INT_EN_INTO_CONFIG_Pos                          (20U)
#define USB_INT_EN_INTO_CONFIG_Len                          (1U)
#define USB_INT_EN_INTO_CONFIG_Msk                          (0x1UL << USB_INT_EN_INTO_CONFIG_Pos)
#define USB_INT_EN_INTO_CONFIG                              USB_INT_EN_INTO_CONFIG_Msk

#define USB_INT_EN_EP5_OUT_READY_Pos                        (21U)
#define USB_INT_EN_EP5_OUT_READY_Len                        (1U)
#define USB_INT_EN_EP5_OUT_READY_Msk                        (0x1UL << USB_INT_EN_EP5_OUT_READY_Pos)
#define USB_INT_EN_EP5_OUT_READY                            USB_INT_EN_EP5_OUT_READY_Msk

#define USB_INT_EN_EP4_AHB_XFER_DONE_Pos                    (22U)
#define USB_INT_EN_EP4_AHB_XFER_DONE_Len                    (1U)
#define USB_INT_EN_EP4_AHB_XFER_DONE_Msk                    (0x1UL << USB_INT_EN_EP4_AHB_XFER_DONE_Pos)
#define USB_INT_EN_EP4_AHB_XFER_DONE                        USB_INT_EN_EP4_AHB_XFER_DONE_Msk

#define USB_INT_EN_EP4_TX_DONE_Pos                          (23U)
#define USB_INT_EN_EP4_TX_DONE_Len                          (1U)
#define USB_INT_EN_EP4_TX_DONE_Msk                          (0x1UL << USB_INT_EN_EP4_TX_DONE_Pos)
#define USB_INT_EN_EP4_TX_DONE                              USB_INT_EN_EP4_TX_DONE_Msk

#define USB_INT_EN_EP5_AHB_XFER_DONE_Pos                    (24U)
#define USB_INT_EN_EP5_AHB_XFER_DONE_Len                    (1U)
#define USB_INT_EN_EP5_AHB_XFER_DONE_Msk                    (0x1UL << USB_INT_EN_EP5_AHB_XFER_DONE_Pos)
#define USB_INT_EN_EP5_AHB_XFER_DONE                        USB_INT_EN_EP5_AHB_XFER_DONE_Msk

#define USB_INT_EN_EP5_TIMER_OUT_ERR_Pos                    (25U)
#define USB_INT_EN_EP5_TIMER_OUT_ERR_Len                    (1U)
#define USB_INT_EN_EP5_TIMER_OUT_ERR_Msk                    (0x1UL << USB_INT_EN_EP5_TIMER_OUT_ERR_Pos)
#define USB_INT_EN_EP5_TIMER_OUT_ERR                        USB_INT_EN_EP5_TIMER_OUT_ERR_Msk

/*******************  Bit definition for USB_INT_CLR register  *******************/
#define USB_INT_CLR_ALL_Pos                                 (0U)
#define USB_INT_CLR_ALL_Len                                 (26U)
#define USB_INT_CLR_ALL_Msk                                 (0x3FFFFFF << USB_INT_CLR_ALL_Pos)
#define USB_INT_CLR_ALL                                     USB_INT_CLR_ALL_Msk

#define USB_INT_CLR_SUSPEND_Pos                             (0U)
#define USB_INT_CLR_SUSPEND_Len                             (1U)
#define USB_INT_CLR_SUSPEND_Msk                             (0x1UL << USB_INT_CLR_SUSPEND_Pos)
#define USB_INT_CLR_SUSPEND                                 USB_INT_CLR_SUSPEND_Msk

#define USB_INT_CLR_EP0_OUT_READY_Pos                       (1U)
#define USB_INT_CLR_EP0_OUT_READY_Len                       (1U)
#define USB_INT_CLR_EP0_OUT_READY_Msk                       (0x1UL << USB_INT_CLR_EP0_OUT_READY_Pos)
#define USB_INT_CLR_EP0_OUT_READY                           USB_INT_CLR_EP0_OUT_READY_Msk

#define USB_INT_CLR_EP1_OUT_READY_Pos                       (2U)
#define USB_INT_CLR_EP1_OUT_READY_Len                       (1U)
#define USB_INT_CLR_EP1_OUT_READY_Msk                       (0x1UL << USB_INT_CLR_EP1_OUT_READY_Pos)
#define USB_INT_CLR_EP1_OUT_READY                           USB_INT_CLR_EP1_OUT_READY_Msk

#define USB_INT_CLR_CRC16_ERR_Pos                           (3U)
#define USB_INT_CLR_CRC16_ERR_Len                           (1U)
#define USB_INT_CLR_CRC16_ERR_Msk                           (0x1UL << USB_INT_CLR_CRC16_ERR_Pos)
#define USB_INT_CLR_CRC16_ERR                               USB_INT_CLR_CRC16_ERR_Msk

#define USB_INT_CLR_UPID_ERR_Pos                            (4U)
#define USB_INT_CLR_UPID_ERR_Len                            (1U)
#define USB_INT_CLR_UPID_ERR_Msk                            (0x1UL << USB_INT_CLR_UPID_ERR_Pos)
#define USB_INT_CLR_UPID_ERR                                USB_INT_CLR_UPID_ERR_Msk

#define USB_INT_CLR_TIMEOUT_ERR_Pos                         (5U)
#define USB_INT_CLR_TIMEOUT_ERR_Len                         (1U)
#define USB_INT_CLR_TIMEOUT_ERR_Msk                         (0x1UL << USB_INT_CLR_TIMEOUT_ERR_Pos)
#define USB_INT_CLR_TIMEOUT_ERR                             USB_INT_CLR_TIMEOUT_ERR_Msk

#define USB_INT_CLR_SEQ_ERR_Pos                             (6U)
#define USB_INT_CLR_SEQ_ERR_Len                             (1U)
#define USB_INT_CLR_SEQ_ERR_Msk                             (0x1UL << USB_INT_CLR_SEQ_ERR_Pos)
#define USB_INT_CLR_SEQ_ERR                                 USB_INT_CLR_SEQ_ERR_Msk

#define USB_INT_CLR_PID_CKS_ERR_Pos                         (7U)
#define USB_INT_CLR_PID_CKS_ERR_Len                         (1U)
#define USB_INT_CLR_PID_CKS_ERR_Msk                         (0x1UL << USB_INT_CLR_PID_CKS_ERR_Pos)
#define USB_INT_CLR_PID_CKS_ERR                             USB_INT_CLR_PID_CKS_ERR_Msk

#define USB_INT_CLR_PID_CRC_ERR_Pos                         (8U)
#define USB_INT_CLR_PID_CRC_ERR_Len                         (1U)
#define USB_INT_CLR_PID_CRC_ERR_Msk                         (0x1UL << USB_INT_CLR_PID_CRC_ERR_Pos)
#define USB_INT_CLR_PID_CRC_ERR                             USB_INT_CLR_PID_CRC_ERR_Msk

#define USB_INT_CLR_HOST_RESET_Pos                          (9U)
#define USB_INT_CLR_HOST_RESET_Len                          (1U)
#define USB_INT_CLR_HOST_RESET_Msk                          (0x1UL << USB_INT_CLR_HOST_RESET_Pos)
#define USB_INT_CLR_HOST_RESET                              USB_INT_CLR_HOST_RESET_Msk

#define USB_INT_CLR_AHB_XFER_ERR_Pos                        (10U)
#define USB_INT_CLR_AHB_XFER_ERR_Len                        (1U)
#define USB_INT_CLR_AHB_XFER_ERR_Msk                        (0x1UL << USB_INT_CLR_AHB_XFER_ERR_Pos)
#define USB_INT_CLR_AHB_XFER_ERR                            USB_INT_CLR_AHB_XFER_ERR_Msk

#define USB_INT_CLR_NSE_ERR_Pos                             (11U)
#define USB_INT_CLR_NSE_ERR_Len                             (1U)
#define USB_INT_CLR_NSE_ERR_Msk                             (0x1UL << USB_INT_CLR_NSE_ERR_Pos)
#define USB_INT_CLR_NSE_ERR                                 USB_INT_CLR_NSE_ERR_Msk

#define USB_INT_CLR_EP3_AHB_XFER_DONE_Pos                   (12U)
#define USB_INT_CLR_EP3_AHB_XFER_DONE_Len                   (1U)
#define USB_INT_CLR_EP3_AHB_XFER_DONE_Msk                   (0x1UL << USB_INT_CLR_EP3_AHB_XFER_DONE_Pos)
#define USB_INT_CLR_EP3_AHB_XFER_DONE                       USB_INT_CLR_EP3_AHB_XFER_DONE_Msk

#define USB_INT_CLR_SYNC_ERR_Pos                            (13U)
#define USB_INT_CLR_SYNC_ERR_Len                            (1U)
#define USB_INT_CLR_SYNC_ERR_Msk                            (0x1UL << USB_INT_CLR_SYNC_ERR_Pos)
#define USB_INT_CLR_SYNC_ERR                                USB_INT_CLR_SYNC_ERR_Msk

#define USB_INT_CLR_BIT_STUFF_ERR_Pos                       (14U)
#define USB_INT_CLR_BIT_STUFF_ERR_Len                       (1U)
#define USB_INT_CLR_BIT_STUFF_ERR_Msk                       (0x1UL << USB_INT_CLR_BIT_STUFF_ERR_Pos)
#define USB_INT_CLR_BIT_STUFF_ERR                           USB_INT_CLR_BIT_STUFF_ERR_Msk

#define USB_INT_CLR_BYTE_ERR_Pos                            (15U)
#define USB_INT_CLR_BYTE_ERR_Len                            (1U)
#define USB_INT_CLR_BYTE_ERR_Msk                            (0x1UL << USB_INT_CLR_BYTE_ERR_Pos)
#define USB_INT_CLR_BYTE_ERR                                USB_INT_CLR_BYTE_ERR_Msk

#define USB_INT_CLR_SOF_Pos                                 (16U)
#define USB_INT_CLR_SOF_Len                                 (1U)
#define USB_INT_CLR_SOF_Msk                                 (0x1UL << USB_INT_CLR_SOF_Pos)
#define USB_INT_CLR_SOF                                     USB_INT_CLR_SOF_Msk

#define USB_INT_CLR_EP0_TX_DONE_Pos                         (17U)
#define USB_INT_CLR_EP0_TX_DONE_Len                         (1U)
#define USB_INT_CLR_EP0_TX_DONE_Msk                         (0x1UL << USB_INT_CLR_EP0_TX_DONE_Pos)
#define USB_INT_CLR_EP0_TX_DONE                             USB_INT_CLR_EP0_TX_DONE_Msk

#define USB_INT_CLR_EP2_TX_DONE_Pos                         (18U)
#define USB_INT_CLR_EP2_TX_DONE_Len                         (1U)
#define USB_INT_CLR_EP2_TX_DONE_Msk                         (0x1UL << USB_INT_CLR_EP2_TX_DONE_Pos)
#define USB_INT_CLR_EP2_TX_DONE                             USB_INT_CLR_EP2_TX_DONE_Msk

#define USB_INT_CLR_EP3_TX_DONE_Pos                         (19U)
#define USB_INT_CLR_EP3_TX_DONE_Len                         (1U)
#define USB_INT_CLR_EP3_TX_DONE_Msk                         (0x1UL << USB_INT_CLR_EP3_TX_DONE_Pos)
#define USB_INT_CLR_EP3_TX_DONE                             USB_INT_CLR_EP3_TX_DONE_Msk

#define USB_INT_CLR_INTO_CONFIG_Pos                         (20U)
#define USB_INT_CLR_INTO_CONFIG_Len                         (1U)
#define USB_INT_CLR_INTO_CONFIG_Msk                         (0x1UL << USB_INT_CLR_INTO_CONFIG_Pos)
#define USB_INT_CLR_INTO_CONFIG                             USB_INT_CLR_INTO_CONFIG_Msk

#define USB_INT_CLR_EP5_OUT_READY_Pos                       (21U)
#define USB_INT_CLR_EP5_OUT_READY_Len                       (1U)
#define USB_INT_CLR_EP5_OUT_READY_Msk                       (0x1UL << USB_INT_CLR_EP5_OUT_READY_Pos)
#define USB_INT_CLR_EP5_OUT_READY                           USB_INT_CLR_EP5_OUT_READY_Msk

#define USB_INT_CLR_EP4_AHB_XFER_DONE_Pos                   (22U)
#define USB_INT_CLR_EP4_AHB_XFER_DONE_Len                   1U)
#define USB_INT_CLR_EP4_AHB_XFER_DONE_Msk                   (0x1UL << USB_INT_CLR_EP4_AHB_XFER_DONE_Pos)
#define USB_INT_CLR_EP4_AHB_XFER_DONE                       USB_INT_CLR_EP4_AHB_XFER_DONE_Msk

#define USB_INT_CLR_EP4_TX_DONE_Pos                         (23U)
#define USB_INT_CLR_EP4_TX_DONE_Len                         (1U)
#define USB_INT_CLR_EP4_TX_DONE_Msk                         (0x1UL << USB_INT_CLR_EP4_TX_DONE_Pos)
#define USB_INT_CLR_EP4_TX_DONE                             USB_INT_CLR_EP4_TX_DONE_Msk

#define USB_INT_CLR_EP5_AHB_XFER_DONE_Pos                   (24U)
#define USB_INT_CLR_EP5_AHB_XFER_DONE_Len                   (1U)
#define USB_INT_CLR_EP5_AHB_XFER_DONE_Msk                   (0x1UL << USB_INT_CLR_EP5_AHB_XFER_DONE_Pos)
#define USB_INT_CLR_EP5_AHB_XFER_DONE                       USB_INT_CLR_EP5_AHB_XFER_DONE_Msk

#define USB_INT_CLR_EP5_TIMER_OUT_ERR_Pos                   (25U)
#define USB_INT_CLR_EP5_TIMER_OUT_ERR_Len                   (1U)
#define USB_INT_CLR_EP5_TIMER_OUT_ERR_Msk                   (0x1UL << USB_INT_EN_EP5_TIMER_OUT_ERR_Pos)
#define USB_INT_CLR_EP5_TIMER_OUT_ERR                       USB_INT_EN_EP5_TIMER_OUT_ERR_Msk

/*******************  Bit definition for USB_EP3_AHBM_RADDR register  *******************/
#define USB_EP3_AHBM_RADDR_RD_START_ADDR_Pos                (0U)
#define USB_EP3_AHBM_RADDR_RD_START_ADDR_Len                (32U)
#define USB_EP3_AHBM_RADDR_RD_START_ADDR_Msk                (0xFFFFFFFFUL << USB_EP3_AHBM_RADDR_RD_START_ADDR_Pos)
#define USB_EP3_AHBM_RADDR_RD_START_ADDR                    USB_EP3_AHBM_RADDR_RD_START_ADDR_Msk

/*******************  Bit definition for USB_EP3_AHBM_CTRL register  *******************/
#define USB_EP3_AHBM_CTRL_EN_Pos                            (0U)
#define USB_EP3_AHBM_CTRL_EN_Len                            (1U)
#define USB_EP3_AHBM_CTRL_EN_Msk                            (0x1UL << USB_EP3_AHBM_CTRL_EN_Pos)
#define USB_EP3_AHBM_CTRL_EN                                USB_EP3_AHBM_CTRL_EN_Msk

#define USB_EP3_AHBM_CTRL_BURST_SIZE_Pos                    (8U)
#define USB_EP3_AHBM_CTRL_BURST_SIZE_Len                    (8U)
#define USB_EP3_AHBM_CTRL_BURST_SIZE_Msk                    (0xFFUL << USB_EP3_AHBM_CTRL_BURST_SIZE_Pos)
#define USB_EP3_AHBM_CTRL_BURST_SIZE                        USB_EP3_AHBM_CTRL_BURST_SIZE_Msk

/*******************  Bit definition for USB_CTRL0 register  *******************/
#define USB_CTRL0_TEST_MODE_Pos                             (0U)
#define USB_CTRL0_TEST_MODE_Len                             (1U)
#define USB_CTRL0_TEST_MODE_Msk                             (0x1UL << USB_CTRL0_TEST_MODE_Pos)
#define USB_CTRL0_TEST_MODE                                 USB_CTRL0_TEST_MODE_Msk

#define USB_CTRL0_DRIVE_DP_Pos                              (1U)
#define USB_CTRL0_DRIVE_DP_Len                              (1U)
#define USB_CTRL0_DRIVE_DP_Msk                              (0x1UL << USB_CTRL0_DRIVE_DP_Pos)
#define USB_CTRL0_DRIVE_DP                                  USB_CTRL0_DRIVE_DP_Msk

#define USB_CTRL0_DRIVE_DM_Pos                              (2U)
#define USB_CTRL0_DRIVE_DM_Len                              (1U)
#define USB_CTRL0_DRIVE_DM_Msk                              (0x1UL << USB_CTRL0_DRIVE_DM_Pos)
#define USB_CTRL0_DRIVE_DM                                  USB_CTRL0_DRIVE_DM_Msk

#define USB_CTRL0_XCVR_OEB_Pos                              (3U)
#define USB_CTRL0_XCVR_OEB_Len                              (1U)
#define USB_CTRL0_XCVR_OEB_Msk                              (0x1UL << USB_CTRL0_XCVR_OEB_Pos)
#define USB_CTRL0_XCVR_OEB                                  USB_CTRL0_XCVR_OEB_Msk

#define USB_CTRL0_XCVR_DP_RPU_EN_Pos                        (4U)
#define USB_CTRL0_XCVR_DP_RPU_EN_Len                        (1U)
#define USB_CTRL0_XCVR_DP_RPU_EN_Msk                        (0x1UL << USB_CTRL0_XCVR_DP_RPU_EN_Pos)
#define USB_CTRL0_XCVR_DP_RPU_EN                            USB_CTRL0_XCVR_DP_RPU_EN_Msk

#define USB_CTRL0_XCVR_DM_RPU_EN_Pos                        (5U)
#define USB_CTRL0_XCVR_DM_RPU_EN_Len                        (1U)
#define USB_CTRL0_XCVR_DM_RPU_EN_Msk                        (0x1UL << USB_CTRL0_XCVR_DM_RPU_EN_Pos)
#define USB_CTRL0_XCVR_DM_RPU_EN                            USB_CTRL0_XCVR_DM_RPU_EN_Msk

#define USB_CTRL0_XCVR_DP_RPUSW_EN_Pos                      (6U)
#define USB_CTRL0_XCVR_DP_RPUSW_EN_Len                      (1U)
#define USB_CTRL0_XCVR_DP_RPUSW_EN_Msk                      (0x1UL << USB_CTRL0_XCVR_DP_RPUSW_EN_Pos)
#define USB_CTRL0_XCVR_DP_RPUSW_EN                          USB_CTRL0_XCVR_DP_RPUSW_EN_Msk

#define USB_CTRL0_XCVR_DM_RPUSW_EN_Pos                      (7U)
#define USB_CTRL0_XCVR_DM_RPUSW_EN_Len                      (1U)
#define USB_CTRL0_XCVR_DM_RPUSW_EN_Msk                      (0x1UL << USB_CTRL0_XCVR_DM_RPUSW_EN_Pos)
#define USB_CTRL0_XCVR_DM_RPUSW_EN                          USB_CTRL0_XCVR_DM_RPUSW_EN_Msk

#define USB_CTRL0_XCVR_DP_RPD_EN_Pos                        (8U)
#define USB_CTRL0_XCVR_DP_RPD_EN_Len                        (1U)
#define USB_CTRL0_XCVR_DP_RPD_EN_Msk                        (0x1UL << USB_CTRL0_XCVR_DP_RPD_EN_Pos)
#define USB_CTRL0_XCVR_DP_RPD_EN                            USB_CTRL0_XCVR_DP_RPD_EN_Msk

#define USB_CTRL0_XCVR_DM_RPD_EN_Pos                        (9U)
#define USB_CTRL0_XCVR_DM_RPD_EN_Len                        (1U)
#define USB_CTRL0_XCVR_DM_RPD_EN_Msk                        (0x1UL << USB_CTRL0_XCVR_DM_RPD_EN_Pos)
#define USB_CTRL0_XCVR_DM_RPD_EN                            USB_CTRL0_XCVR_DM_RPD_EN_Msk

#define USB_CTRL0_OUTPUT_ENDIAN_CTRL_Pos                    (13U)
#define USB_CTRL0_OUTPUT_ENDIAN_CTRL_Len                    (1U)
#define USB_CTRL0_OUTPUT_ENDIAN_CTRL_Msk                    (0x1UL << USB_CTRL0_OUTPUT_ENDIAN_CTRL_Pos)
#define USB_CTRL0_OUTPUT_ENDIAN_CTRL                        USB_CTRL0_OUTPUT_ENDIAN_CTRL_Msk

#define USB_CTRL0_INPUT_ENDIAN_CTRL_Pos                     (14U)
#define USB_CTRL0_INPUT_ENDIAN_CTRL_Len                     (1U)
#define USB_CTRL0_INPUT_ENDIAN_CTRL_Msk                     (0x1UL << USB_CTRL0_INPUT_ENDIAN_CTRL_Pos)
#define USB_CTRL0_INPUT_ENDIAN_CTRL                         USB_CTRL0_INPUT_ENDIAN_CTRL_Msk

#define USB_CTRL0_RXD_STAT_Pos                              (15U)
#define USB_CTRL0_RXD_STAT_Len                              (1U)
#define USB_CTRL0_RXD_STAT_Msk                              (0x1UL << USB_CTRL0_RXD_STAT_Pos)
#define USB_CTRL0_RXD_STAT                                  USB_CTRL0_RXD_STAT_Msk

#define USB_CTRL0_PROBE_SEL_Pos                             (16U)
#define USB_CTRL0_PROBE_SEL_Len                             (3U)
#define USB_CTRL0_PROBE_SEL_Msk                             (0x7UL << USB_CTRL0_PROBE_SEL_Pos)
#define USB_CTRL0_PROBE_SEL                                 USB_CTRL0_PROBE_SEL_Msk

/*******************  Bit definition for USB_EP3_XFER_LEN register  *******************/
#define USB_EP3_XFER_LEN_Pos                                (0U)
#define USB_EP3_XFER_LEN_Len                                (16U)
#define USB_EP3_XFER_LEN_Msk                                (0xFFFFUL << USB_EP3_XFER_LEN_Pos)
#define USB_EP3_XFER_LEN                                    USB_EP3_XFER_LEN_Msk

/*******************  Bit definition for USB_RX_CNT register  *******************/
#define USB_RX_CNT_EP0_RX_DATA_SUM_Pos                      (0U)
#define USB_RX_CNT_EP0_RX_DATA_SUM_Len                      (8U)
#define USB_RX_CNT_EP0_RX_DATA_SUM_Msk                      (0xFFUL << USB_RX_CNT_EP0_RX_DATA_SUM_Pos)
#define USB_RX_CNT_EP0_RX_DATA_SUM                          USB_RX_CNT_EP0_RX_DATA_SUM_Msk

#define USB_RX_CNT_EP1_RX_DATA_SUM_Pos                      (8U)
#define USB_RX_CNT_EP1_RX_DATA_SUM_Len                      (8U)
#define USB_RX_CNT_EP1_RX_DATA_SUM_Msk                      (0xFFUL << USB_RX_CNT_EP1_RX_DATA_SUM_Pos)
#define USB_RX_CNT_EP1_RX_DATA_SUM                          USB_RX_CNT_EP1_RX_DATA_SUM_Msk

/*******************  Bit definition for USB_CFG_DESC_CTRL register  *******************/
#define USB_CFG_DESC_CTRL_START_Pos                         (0U)
#define USB_CFG_DESC_CTRL_START_Len                         (8U)
#define USB_CFG_DESC_CTRL_START_Msk                         (0xFFUL << USB_CFG_DESC_CTRL_START_Pos)
#define USB_CFG_DESC_CTRL_START                             USB_CFG_DESC_CTRL_START_Msk

#define USB_CFG_DESC_CTRL_SIZE_Pos                          (8U)
#define USB_CFG_DESC_CTRL_SIZE_Len                          (8U)
#define USB_CFG_DESC_CTRL_SIZE_Msk                          (0xFFUL << USB_CFG_DESC_CTRL_SIZE_Pos)
#define USB_CFG_DESC_CTRL_SIZE                              USB_CFG_DESC_CTRL_SIZE_Msk

/*******************  Bit definition for USB_STR_DESC0_CTRL register  *******************/
#define USB_STR_DESC0_CTRL_START_Pos                        (0U)
#define USB_STR_DESC0_CTRL_START_Len                        (8U)
#define USB_STR_DESC0_CTRL_START_Msk                        (0xFFUL << USB_STR_DESC0_CTRL_START_Pos)
#define USB_STR_DESC0_CTRL_START                            USB_STR_DESC0_CTRL_START_Msk

#define USB_STR_DESC0_CTRL_SIZE_Pos                         (8U)
#define USB_STR_DESC0_CTRL_SIZE_Len                         (8U)
#define USB_STR_DESC0_CTRL_SIZE_Msk                         (0xFFUL << USB_STR_DESC0_CTRL_SIZE_Pos)
#define USB_STR_DESC0_CTRL_SIZE                             USB_STR_DESC0_CTRL_SIZE_Msk

/*******************  Bit definition for USB_STR_DESC1_CTRL register  *******************/
#define USB_STR_DESC1_CTRL_START_Pos                        (0U)
#define USB_STR_DESC1_CTRL_START_Len                        (8U)
#define USB_STR_DESC1_CTRL_START_Msk                        (0xFFUL << USB_STR_DESC1_CTRL_START_Pos)
#define USB_STR_DESC1_CTRL_START                            USB_STR_DESC1_CTRL_START_Msk

#define USB_STR_DESC1_CTRL_SIZE_Pos                         (8U)
#define USB_STR_DESC1_CTRL_SIZE_Len                         (8U)
#define USB_STR_DESC1_CTRL_SIZE_Msk                         (0xFFUL << USB_STR_DESC1_CTRL_SIZE_Pos)
#define USB_STR_DESC1_CTRL_SIZE                             USB_STR_DESC1_CTRL_SIZE_Msk

/*******************  Bit definition for USB_EP0_FIFO_ADDR register  *******************/
#define USB_EP0_FIFO_ADDR_Pos                               (0U)
#define USB_EP0_FIFO_ADDR_Len                               (32U)
#define USB_EP0_FIFO_ADDR_Msk                               (0xFFFFFFFFUL << USB_EP0_FIFO_ADDR_Pos)
#define USB_EP0_FIFO_ADDR                                   USB_EP0_FIFO_ADDR_Msk

/*******************  Bit definition for USB_EP1_FIFO_ADDR register  *******************/
#define USB_EP1_FIFO_ADDR_Pos                               (0U)
#define USB_EP1_FIFO_ADDR_Len                               (32U)
#define USB_EP1_FIFO_ADDR_Msk                               (0xFFFFFFFFUL << USB_EP1_FIFO_ADDR_Pos)
#define USB_EP1_FIFO_ADDR                                   USB_EP1_FIFO_ADDR_Msk

/*******************  Bit definition for USB_EP2_FIFO_ADDR register  *******************/
#define USB_EP2_FIFO_ADDR_Pos                               (0U)
#define USB_EP2_FIFO_ADDR_Len                               (32U)
#define USB_EP2_FIFO_ADDR_Msk                               (0xFFFFFFFFUL << USB_EP2_FIFO_ADDR_Pos)
#define USB_EP2_FIFO_ADDR                                   USB_EP2_FIFO_ADDR_Msk

/*******************  Bit definition for USB_EP3_FIFO_ADDR register  *******************/
#define USB_EP3_FIFO_ADDR_Pos                               (0U)
#define USB_EP3_FIFO_ADDR_Len                               (32U)
#define USB_EP3_FIFO_ADDR_Msk                               (0xFFFFFFFFUL << USB_EP3_FIFO_ADDR_Pos)
#define USB_EP3_FIFO_ADDR                                   USB_EP3_FIFO_ADDR_Msk

/*******************  Bit definition for USB_SRAM_ADDR register  *******************/
#define USB_SRAM_ADDR_DESC_SRAM_Pos                         (0U)
#define USB_SRAM_ADDR_DESC_SRAM_Len                         (32U)
#define USB_SRAM_ADDR_DESC_SRAM_Msk                         (0xFFFFFFFFUL << USB_SRAM_ADDR_DESC_SRAM_Pos)
#define USB_SRAM_ADDR_DESC_SRAM                             USB_SRAM_ADDR_DESC_SRAM_Msk

/*******************  Bit definition for USB_STR_DESC2_CTRL register  *******************/
#define USB_STR_DESC2_CTRL_START_Pos                        (0U)
#define USB_STR_DESC2_CTRL_START_Len                        (8U)
#define USB_STR_DESC2_CTRL_START_Msk                        (0xFFUL << USB_STR_DESC2_CTRL_START_Pos)
#define USB_STR_DESC2_CTRL_START                            USB_STR_DESC2_CTRL_START_Msk

#define USB_STR_DESC2_CTRL_SIZE_Pos                         (8U)
#define USB_STR_DESC2_CTRL_SIZE_Len                         (8U)
#define USB_STR_DESC2_CTRL_SIZE_Msk                         (0xFFUL << USB_STR_DESC2_CTRL_SIZE_Pos)
#define USB_STR_DESC2_CTRL_SIZE                             USB_STR_DESC2_CTRL_SIZE_Msk

/*******************  Bit definition for USB_STR_DESC3_CTRL register  *******************/
#define USB_STR_DESC3_CTRL_START_Pos                        (0U)
#define USB_STR_DESC3_CTRL_START_Len                        (8U)
#define USB_STR_DESC3_CTRL_START_Msk                        (0xFFUL << USB_STR_DESC3_CTRL_START_Pos)
#define USB_STR_DESC3_CTRL_START                            USB_STR_DESC3_CTRL_START_Msk

#define USB_STR_DESC3_CTRL_SIZE_Pos                         (8U)
#define USB_STR_DESC3_CTRL_SIZE_Len                         (8U)
#define USB_STR_DESC3_CTRL_SIZE_Msk                         (0xFFUL << USB_STR_DESC3_CTRL_SIZE_Pos)
#define USB_STR_DESC3_CTRL_SIZE                             USB_STR_DESC3_CTRL_SIZE_Msk

/*******************  Bit definition for USB_EP4_CTRL register  *******************/
#define USB_EP4_CTRL_DATA_RDY_Pos                           (0U)
#define USB_EP4_CTRL_DATA_RDY_Len                           (1U)
#define USB_EP4_CTRL_DATA_RDY_Msk                           (0x1UL << USB_EP4_CTRL_DATA_RDY_Pos)
#define USB_EP4_CTRL_DATA_RDY                               USB_EP4_CTRL_DATA_RDY_Msk

#define USB_EP4_CTRL_IFIFO_CLR_Pos                          (1U)
#define USB_EP4_CTRL_IFIFO_CLR_Len                          (1U)
#define USB_EP4_CTRL_IFIFO_CLR_Msk                          (0x1UL << USB_EP4_CTRL_IFIFO_CLR_Pos)
#define USB_EP4_CTRL_IFIFO_CLR                              USB_EP4_CTRL_IFIFO_CLR_Msk

#define USB_EP4_CTRL_EMPTY_PACKET_EN_Pos                    (2U)
#define USB_EP4_CTRL_EMPTY_PACKET_EN_Len                    (1U)
#define USB_EP4_CTRL_EMPTY_PACKET_EN_Msk                    (0x1UL << USB_EP4_CTRL_EMPTY_PACKET_EN_Pos)
#define USB_EP4_CTRL_EMPTY_PACKET_EN                        USB_EP4_CTRL_EMPTY_PACKET_EN_Msk

/*******************  Bit definition for USB_EP4_AHBM_RADDR register  *******************/
#define USB_EP4_AHBM_RADDR_RD_START_ADDR_Pos                (0U)
#define USB_EP4_AHBM_RADDR_RD_START_ADDR_Len                (32U)
#define USB_EP4_AHBM_RADDR_RD_START_ADDR_Msk                (0xFFFFFFFFUL << USB_EP4_AHBM_RADDR_RD_START_ADDR_Pos)
#define USB_EP4_AHBM_RADDR_RD_START_ADDR                    USB_EP4_AHBM_RADDR_RD_START_ADDR_Msk

/*******************  Bit definition for USB_EP4_AHBM_CTRL register  *******************/
#define USB_EP4_AHBM_CTRL_EN_Pos                            (0U)
#define USB_EP4_AHBM_CTRL_EN_Len                            (1U)
#define USB_EP4_AHBM_CTRL_EN_Msk                            (0x1UL << USB_EP4_AHBM_CTRL_EN_Pos)
#define USB_EP4_AHBM_CTRL_EN                                USB_EP4_AHBM_CTRL_EN_Msk

#define USB_EP4_AHBM_CTRL_BURST_SIZE_Pos                    (8U)
#define USB_EP4_AHBM_CTRL_BURST_SIZE_Len                    (10U)
#define USB_EP4_AHBM_CTRL_BURST_SIZE_Msk                    (0x3FFUL << USB_EP4_AHBM_CTRL_BURST_SIZE_Pos)
#define USB_EP4_AHBM_CTRL_BURST_SIZE                        USB_EP4_AHBM_CTRL_BURST_SIZE_Msk

/*******************  Bit definition for USB_EP4_XFER_LEN register  *******************/
#define USB_EP4_XFER_LEN_Pos                                (0U)
#define USB_EP4_XFER_LEN_Len                                (16U)
#define USB_EP4_XFER_LEN_Msk                                (0xFFFFUL << USB_EP4_XFER_LEN_Pos)
#define USB_EP4_XFER_LEN                                    USB_EP4_XFER_LEN_Msk

/*******************  Bit definition for USB_EP5_AHBM_CTRL register  *******************/
#define USB_EP5_CTRL_AHBM_EN_Pos                            (0U)
#define USB_EP5_CTRL_AHBM_EN_Len                            (1U)
#define USB_EP5_CTRL_AHBM_EN_Msk                            (0x1UL << USB_EP5_CTRL_AHBM_EN_Pos)
#define USB_EP5_CTRL_AHBM_EN                                USB_EP5_CTRL_AHBM_EN_Msk

#define USB_EP5_CTRL_FIFO_CLR_Pos                           (1U)
#define USB_EP5_CTRL_FIFO_CLR_Len                           (1U)
#define USB_EP5_CTRL_FIFO_CLR_Msk                           (0x1UL << USB_EP5_CTRL_FIFO_CLR_Pos)
#define USB_EP5_CTRL_FIFO_CLR                               USB_EP5_CTRL_FIFO_CLR_Msk

#define USB_EP5_CTRL_RX_CNT_NO_OVERWRITE_Pos                (2U)
#define USB_EP5_CTRL_RX_CNT_NO_OVERWRITE_Len                (1U)
#define USB_EP5_CTRL_RX_CNT_NO_OVERWRITE_Msk                (0x1UL << USB_EP5_CTRL_RX_CNT_NO_OVERWRITE_Pos)
#define USB_EP5_CTRL_RX_CNT_NO_OVERWRITE                    USB_EP5_CTRL_RX_CNT_NO_OVERWRITE_Msk

/*******************  Bit definition for USB_EP5_AHBM_RADDR register  *******************/
#define USB_EP5_AHBM_RADDR_RD_START_ADDR_Pos                (0U)
#define USB_EP5_AHBM_RADDR_RD_START_ADDR_Len                (32U)
#define USB_EP5_AHBM_RADDR_RD_START_ADDR_Msk                (0xFFFFFFFFUL << USB_EP5_AHBM_RADDR_RD_START_ADDR_Pos)
#define USB_EP5_AHBM_RADDR_RD_START_ADDR                    USB_EP5_AHBM_RADDR_RD_START_ADDR_Msk

/*******************  Bit definition for USB_EP5_XFER_LEN register  *******************/
#define USB_EP5_XFER_LEN_Pos                                (0U)
#define USB_EP5_XFER_LEN_Len                                (16U)
#define USB_EP5_XFER_LEN_Msk                                (0xFFFFUL << USB_EP5_XFER_LEN_Pos)
#define USB_EP5_XFER_LEN                                    USB_EP5_XFER_LEN_Msk

/*******************  Bit definition for USB_EP5_TIMER register  *******************/
#define USB_EP5_TIMER_VAL_Pos                               (0U)
#define USB_EP5_TIMER_VAL_Len                               (9U)
#define USB_EP5_TIMER_VAL_Msk                               (0x3FFUL << USB_EP5_TIMER_VAL_Pos)
#define USB_EP5_TIMER_VAL                                   USB_EP5_TIMER_VAL_Msk

/*******************  Bit definition for USB_EP4_FIFO_WR_EN register  *******************/
#define USB_EP4_FIFO_WR_EN_Pos                              (0U)
#define USB_EP4_FIFO_WR_EN_Len                              (4U)
#define USB_EP4_FIFO_WR_EN_Msk                              (0xFUL << USB_EP4_FIFO_WR_EN_Pos)
#define USB_EP4_FIFO_WR_EN                                  USB_EP4_FIFO_WR_EN_Msk

/*******************  Bit definition for USB_EP4_FIFO_ADDR register  *******************/
#define USB_EP4_FIFO_ADDR_Pos                               (0U)
#define USB_EP4_FIFO_ADDR_Len                               (32U)
#define USB_EP4_FIFO_ADDR_Msk                               (0xFFFFFFFFUL << USB_EP4_FIFO_ADDR_Pos)
#define USB_EP4_FIFO_ADDR                                   USB_EP4_FIFO_ADDR_Msk

/*******************  Bit definition for USB_EP5_FIFO_ADDR register  *******************/
#define USB_EP5_FIFO_ADDR_Pos                               (0U)
#define USB_EP5_FIFO_ADDR_Len                               (32U)
#define USB_EP5_FIFO_ADDR_Msk                               (0xFFFFFFFFUL << USB_EP5_FIFO_ADDR_Pos)
#define USB_EP5_FIFO_ADDR                                   USB_EP5_FIFO_ADDR_Msk

/*******************  Bit definition for USB_EP5_RX_CNT register  *******************/
#define USB_EP5_RX_CNT_Pos                                  (0U)
#define USB_EP5_RX_CNT_Len                                  (32U)
#define USB_EP5_RX_CNT_Msk                                  (0xFFFFFFFFUL << USB_EP5_RX_CNT_Pos)
#define USB_EP5_RX_CNT                                      USB_EP5_RX_CNT_Msk

/*******************  Bit definition for USB_DEBUG register  *******************/
#define USB_DEBUG_PROBE_Pos                                 (0U)
#define USB_DEBUG_PROBE_Len                                 (4U)
#define USB_DEBUG_PROBE_Msk                                 (0xFUL << USB_DEBUG_PROBE_Pos)
#define USB_DEBUG_PROBE                                     USB_DEBUG_PROBE_Msk

#define USB_DEBUG_EP3_FIFO_EMPTY_Pos                        (4U)
#define USB_DEBUG_EP3_FIFO_EMPTY_Len                        (1U)
#define USB_DEBUG_EP3_FIFO_EMPTY_Msk                        (0x1UL << USB_DEBUG_EP3_FIFO_EMPTY_Pos)
#define USB_DEBUG_EP3_FIFO_EMPTY                            USB_DEBUG_EP3_FIFO_EMPTY_Msk
                                                         
#define USB_DEBUG_EP3_FIFO_FULL_Pos                         (5U)
#define USB_DEBUG_EP3_FIFO_FULL_Len                         (1U)
#define USB_DEBUG_EP3_FIFO_FULL_Msk                         (0x1UL << USB_DEBUG_EP3_FIFO_FULL_Pos)
#define USB_DEBUG_EP3_FIFO_FULL                             USB_DEBUG_EP3_FIFO_FULL_Msk

#define USB_DEBUG_EP4_FIFO_EMPTY_Pos                        (6U)
#define USB_DEBUG_EP4_FIFO_EMPTY_Len                        (1U)
#define USB_DEBUG_EP4_FIFO_EMPTY_Msk                        (0x1UL << USB_DEBUG_EP4_FIFO_EMPTY_Pos)
#define USB_DEBUG_EP4_FIFO_EMPTY                             USB_DEBUG_EP4_FIFO_EMPTY_Msk

#define USB_DEBUG_EP4_FIFO_FULL_Pos                         (7U)
#define USB_DEBUG_EP4_FIFO_FULL_Len                         (1U)
#define USB_DEBUG_EP4_FIFO_FULL_Msk                         (0x1UL << USB_DEBUG_EP4_FIFO_FULL_Pos)
#define USB_DEBUG_EP4_FIFO_FULL                              USB_DEBUG_EP4_FIFO_FULL_Msk

#define USB_DEBUG_EP5_FIFO_EMPTY_Pos                        (8U)
#define USB_DEBUG_EP5_FIFO_EMPTY_Len                        (1U)
#define USB_DEBUG_EP5_FIFO_EMPTY_Msk                        (0x1UL << USB_DEBUG_EP5_FIFO_EMPTY_Pos)
#define USB_DEBUG_EP5_FIFO_EMPTY                             USB_DEBUG_EP5_FIFO_EMPTY_Msk

/* ================================================================================================================= */
/* ================                                        WDT                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for WDT_LOAD register  ********************/
#define WDT_LOAD_Pos                                       (0U)
#define WDT_LOAD_Len                                       (32U)
#define WDT_LOAD_Msk                                       (0xFFFFFFFFU << WDT_LOAD_Pos)
#define WDT_LOAD                                           WDT_LOAD_Msk

/*******************  Bit definition for WDT_LOAD register  ********************/
#define WDT_VAL_Pos                                        (0U)
#define WDT_VAL_Len                                        (32U)
#define WDT_VAL_Msk                                        (0xFFFFFFFFU << WDT_VAL_Pos)
#define WDT_VAL                                            WDT_VAL_Msk

/*******************  Bit definition for WDT_CTRL register  ********************/
#define WDT_CTRL_INTEN_Pos                                  (0U)
#define WDT_CTRL_INTEN_Len                                  (1U)
#define WDT_CTRL_INTEN_Msk                                  (0x1U << WDT_CTRL_INTEN_Pos)
#define WDT_CTRL_INTEN                                      WDT_CTRL_INTEN_Msk      /**< Interrupt Enable */

#define WDT_CTRL_RSTEN_Pos                                  (1U)
#define WDT_CTRL_RSTEN_Len                                  (1U)
#define WDT_CTRL_RSTEN_Msk                                  (0x1U << WDT_CTRL_RSTEN_Pos)
#define WDT_CTRL_RSTEN                                      WDT_CTRL_RSTEN_Msk      /**< Reset Enable */

/*******************  Bit definition for WDT_INTCLR register  ********************/
#define WDT_INTCLR_Pos                                      (0U)
#define WDT_INTCLR_Len                                      (1U)
#define WDT_INTCLR_Msk                                      (0x1U << WDT_INTCLR_Pos)
#define WDT_INTCLR                                          WDT_INTCLR_Msk   /**< Interrupt status clear */

/*******************  Bit definition for WDT_RIS register  ********************/
#define WDT_RAW_INTSTAT_Pos                                 (0U)
#define WDT_RAW_INTSTAT_Len                                 (1U)
#define WDT_RAW_INTSTAT_Msk                                 (0x1U << WDT_RAW_INTSTAT_Pos)
#define WDT_RAW_INTSTAT                                     WDT_RAW_INTSTAT_Msk   /**< Raw Interrupt status */

/*******************  Bit definition for WDT_MIS register  ********************/
#define WDT_MIS_INTSTAT_Pos                                 (0U)
#define WDT_MIS_INTSTAT_Len                                 (1U)
#define WDT_MIS_INTSTAT_Msk                                 (0x1U << WDT_MIS_INTSTAT_Pos)
#define WDT_MIS_INTSTAT                                     WDT_MIS_INTSTAT_Msk     /**< Interrupt status */

/*******************  Bit definition for WDT_LOCK register  ********************/
#define WDT_LOCK_WR_EN_STAT_Pos                             (0U)
#define WDT_LOCK_WR_EN_STAT_Len                             (1U)
#define WDT_LOCK_WR_EN_STAT_Msk                             (0x1U << WDT_LOCK_WR_EN_STAT_Pos)
#define WDT_LOCK_WR_EN_STAT                                 WDT_LOCK_WR_EN_STAT_Msk     /**< write access enable status */

#define WDT_LOCK_WR_EN_Pos                                  (1U)
#define WDT_LOCK_WR_EN_Len                                  (31U)
#define WDT_LOCK_WR_EN_Msk                                  (0x7FFFFFFFU << WDT_LOCK_WR_EN_Pos)
#define WDT_LOCK_WR_EN                                      WDT_LOCK_WR_EN_Msk     /**< write 0x1ACCE551 to accesss all other registers */


/* ================================================================================================================= */
/* ================                                       XQSPI                                     ================ */
/* ================================================================================================================= */
/*******************  Bit definition for XQSPI_CACHE_CTRL0 register  **********/
#define XQSPI_CACHE_CTRL0_DIRECT_MAP_EN_Pos                 (11U)
#define XQSPI_CACHE_CTRL0_DIRECT_MAP_EN_Len                 (1U)
#define XQSPI_CACHE_CTRL0_DIRECT_MAP_EN_Msk                 (0x1U << XQSPI_CACHE_CTRL0_DIRECT_MAP_EN_Pos)
#define XQSPI_CACHE_CTRL0_DIRECT_MAP_EN                     XQSPI_CACHE_CTRL0_DIRECT_MAP_EN_Msk

#define XQSPI_CACHE_CTRL0_CLK_FORCE_EN_Pos                  (7U)
#define XQSPI_CACHE_CTRL0_CLK_FORCE_EN_Len                  (4U)
#define XQSPI_CACHE_CTRL0_CLK_FORCE_EN_Msk                  (0xFU << XQSPI_CACHE_CTRL0_CLK_FORCE_EN_Pos)
#define XQSPI_CACHE_CTRL0_CLK_FORCE_EN                      XQSPI_CACHE_CTRL0_CLK_FORCE_EN_Msk

#define XQSPI_CACHE_CTRL0_BUF_DIS_Pos                       (6U)
#define XQSPI_CACHE_CTRL0_BUF_DIS_Len                       (1U)
#define XQSPI_CACHE_CTRL0_BUF_DIS_Msk                       (0x1U << XQSPI_CACHE_CTRL0_BUF_DIS_Pos)
#define XQSPI_CACHE_CTRL0_BUF_DIS                           XQSPI_CACHE_CTRL0_BUF_DIS_Msk

#define XQSPI_CACHE_CTRL0_DIS_SEQ_Pos                       (5U)
#define XQSPI_CACHE_CTRL0_DIS_SEQ_Len                       (1U)
#define XQSPI_CACHE_CTRL0_DIS_SEQ_Msk                       (0x1U << XQSPI_CACHE_CTRL0_DIS_SEQ_Pos)
#define XQSPI_CACHE_CTRL0_DIS_SEQ                           XQSPI_CACHE_CTRL0_DIS_SEQ_Msk

#define XQSPI_CACHE_CTRL0_HITMISS_Pos                       (4U)
#define XQSPI_CACHE_CTRL0_HITMISS_Len                       (1U)
#define XQSPI_CACHE_CTRL0_HITMISS_Msk                       (0x1U << XQSPI_CACHE_CTRL0_HITMISS_Pos)
#define XQSPI_CACHE_CTRL0_HITMISS                           XQSPI_CACHE_CTRL0_HITMISS_Msk

#define XQSPI_CACHE_CTRL0_FIFO_Pos                          (3U)
#define XQSPI_CACHE_CTRL0_FIFO_Len                          (1U)
#define XQSPI_CACHE_CTRL0_FIFO_Msk                          (0x1U << XQSPI_CACHE_CTRL0_FIFO_Pos)
#define XQSPI_CACHE_CTRL0_FIFO                              XQSPI_CACHE_CTRL0_FIFO_Msk

#define XQSPI_CACHE_CTRL0_FLUSH_Pos                         (1U)
#define XQSPI_CACHE_CTRL0_FLUSH_Len                         (1U)
#define XQSPI_CACHE_CTRL0_FLUSH_Msk                         (0x1U << XQSPI_CACHE_CTRL0_FLUSH_Pos)
#define XQSPI_CACHE_CTRL0_FLUSH                             XQSPI_CACHE_CTRL0_FLUSH_Msk

#define XQSPI_CACHE_CTRL0_DIS_Pos                           (0U)
#define XQSPI_CACHE_CTRL0_DIS_Len                           (1U)
#define XQSPI_CACHE_CTRL0_DIS_Msk                           (0x1U << XQSPI_CACHE_CTRL0_DIS_Pos)
#define XQSPI_CACHE_CTRL0_DIS                               XQSPI_CACHE_CTRL0_DIS_Msk

/*******************  Bit definition for XQSPI_CACHE_CTRL1 register  **********/
#define XQSPI_CACHE_CTRL1_DBGMUX_EN_Pos                     (4U)
#define XQSPI_CACHE_CTRL1_DBGMUX_EN_Len                     (1U)
#define XQSPI_CACHE_CTRL1_DBGMUX_EN_Msk                     (0x1U << XQSPI_CACHE_CTRL1_DBGMUX_EN_Pos)
#define XQSPI_CACHE_CTRL1_DBGMUX_EN                         XQSPI_CACHE_CTRL1_DBGMUX_EN_Msk

#define XQSPI_CACHE_CTRL1_DBGBUS_SEL_Pos                    (0U)
#define XQSPI_CACHE_CTRL1_DBGBUS_SEL_Len                    (4U)
#define XQSPI_CACHE_CTRL1_DBGBUS_SEL_Msk                    (0xFU << XQSPI_CACHE_CTRL1_DBGBUS_SEL_Pos)
#define XQSPI_CACHE_CTRL1_DBGBUS_SEL                        XQSPI_CACHE_CTRL1_DBGBUS_SEL_Msk

/*******************  Bit definition for XQSPI_CACHE_HITCOUNT register  *******/
#define XQSPI_CACHE_HITCOUNT_Pos                            (0U)
#define XQSPI_CACHE_HITCOUNT_Len                            (32U)
#define XQSPI_CACHE_HITCOUNT_Msk                            (0xFFFFFFFFU)
#define XQSPI_CACHE_HITCOUNT                                XQSPI_CACHE_HITCOUNT_Msk

/*******************  Bit definition for XQSPI_CACHE_MISSCOUNT register  ******/
#define XQSPI_CACHE_MISSCOUNT_Pos                           (0U)
#define XQSPI_CACHE_MISSCOUNT_Len                           (32U)
#define XQSPI_CACHE_MISSCOUNT_Msk                           (0xFFFFFFFFU)
#define XQSPI_CACHE_MISSCOUNT                               XQSPI_CACHE_MISSCOUNT_Msk

/*******************  Bit definition for XQSPI_CACHE_STAT register  ***********/
#define XQSPI_CACHE_BUF_BUSY_Pos                            (2U)
#define XQSPI_CACHE_BUF_BUSY_Len                            (1U)
#define XQSPI_CACHE_BUF_BUSY_Msk                            (0x1U << XQSPI_CACHE_BUF_BUSY_Pos)
#define XQSPI_CACHE_BUF_BUSY                                XQSPI_CACHE_BUF_BUSY_Msk

#define XQSPI_CACHE_BUF_ADDR_REACHED_Pos                    (1U)
#define XQSPI_CACHE_BUF_ADDR_REACHED_Len                    (1U)
#define XQSPI_CACHE_BUF_ADDR_REACHED_Msk                    (0x1U << XQSPI_CACHE_BUF_ADDR_REACHED_Pos)
#define XQSPI_CACHE_BUF_ADDR_REACHED                        XQSPI_CACHE_BUF_ADDR_REACHED_Msk

#define XQSPI_CACHE_STAT_Pos                                (0U)
#define XQSPI_CACHE_STAT_Len                                (1U)
#define XQSPI_CACHE_STAT_Msk                                (0x1U << XQSPI_CACHE_STAT_Pos)
#define XQSPI_CACHE_STAT                                    XQSPI_CACHE_STAT_Msk

/*******************  Bit definition for XQSPI_CACHE_BUF_FIRST_ADDR register  ***********/
#define XQSPI_CACHE_BUF_FISRT_ADDR_Pos                      (0U)
#define XQSPI_CACHE_BUF_FISRT_ADDR_Len                      (32U)
#define XQSPI_CACHE_BUF_FISRT_ADDR_Msk                      (0xFFFFFFFFU)
#define XQSPI_CACHE_BUF_FISRT_ADDR                          XQSPI_CACHE_BUF_FISRT_ADDR_Msk

/*******************  Bit definition for XQSPI_CACHE_BUF_LAST_ADDR register  ***********/
#define XQSPI_CACHE_BUF_LAST_ADDR_Pos                       (0U)
#define XQSPI_CACHE_BUF_LAST_ADDR_Len                       (32U)
#define XQSPI_CACHE_BUF_LAST_ADDR_Msk                       (0xFFFFFFFFU)
#define XQSPI_CACHE_BUF_LAST_ADDR                           XQSPI_CACHE_BUF_LAST_ADDR_Msk

/*******************  Bit definition for XQSPI_XIP_CFG register  **************/
#define XQSPI_XIP_CFG_CMD_Pos                               (0U)
#define XQSPI_XIP_CFG_CMD_Len                               (8U)
#define XQSPI_XIP_CFG_CMD_Msk                               (0xFFU << XQSPI_XIP_CFG_CMD_Pos)
#define XQSPI_XIP_CFG_CMD                                   XQSPI_XIP_CFG_CMD_Msk

#define XQSPI_XIP_CFG_LE32_Pos                              (8U)
#define XQSPI_XIP_CFG_LE32_Len                              (1U)
#define XQSPI_XIP_CFG_LE32_Msk                              (0x1U << XQSPI_XIP_CFG_LE32_Pos)
#define XQSPI_XIP_CFG_LE32                                  XQSPI_XIP_CFG_LE32_Msk

#define XQSPI_XIP_CFG_ADDR4_Pos                             (7U)
#define XQSPI_XIP_CFG_ADDR4_Len                             (1U)
#define XQSPI_XIP_CFG_ADDR4_Msk                             (0x1U << XQSPI_XIP_CFG_ADDR4_Pos)
#define XQSPI_XIP_CFG_ADDR4                                 XQSPI_XIP_CFG_ADDR4_Msk

#define XQSPI_XIP_CFG_CPOL_Pos                              (6U)
#define XQSPI_XIP_CFG_CPOL_Len                              (1U)
#define XQSPI_XIP_CFG_CPOL_Msk                              (0x1U << XQSPI_XIP_CFG_CPOL_Pos)
#define XQSPI_XIP_CFG_CPOL                                  XQSPI_XIP_CFG_CPOL_Msk

#define XQSPI_XIP_CFG_CPHA_Pos                              (5U)
#define XQSPI_XIP_CFG_CPHA_Len                              (1U)
#define XQSPI_XIP_CFG_CPHA_Msk                              (0x1U << XQSPI_XIP_CFG_CPHA_Pos)
#define XQSPI_XIP_CFG_CPHA                                  XQSPI_XIP_CFG_CPHA_Msk

#define XQSPI_XIP_CFG_SS_Pos                                (1U)
#define XQSPI_XIP_CFG_SS_Len                                (4U)
#define XQSPI_XIP_CFG_SS_Msk                                (0xFU << XQSPI_XIP_CFG_SS_Pos)
#define XQSPI_XIP_CFG_SS                                    XQSPI_XIP_CFG_SS_Msk

#define XQSPI_XIP_CFG_HPEN_Pos                              (0U)
#define XQSPI_XIP_CFG_HPEN_Len                              (1U)
#define XQSPI_XIP_CFG_HPEN_Msk                              (0x1U << XQSPI_XIP_CFG_HPEN_Pos)
#define XQSPI_XIP_CFG_HPEN                                  XQSPI_XIP_CFG_HPEN_Msk

#define XQSPI_XIP_CFG_ENDDUMMY_Pos                          (12U)
#define XQSPI_XIP_CFG_ENDDUMMY_Len                          (2U)
#define XQSPI_XIP_CFG_ENDDUMMY_Msk                          (0x3U << XQSPI_XIP_CFG_ENDDUMMY_Pos)
#define XQSPI_XIP_CFG_ENDDUMMY                              XQSPI_XIP_CFG_ENDDUMMY_Msk

#define XQSPI_XIP_CFG_DUMMYCYCLES_Pos                       (8U)
#define XQSPI_XIP_CFG_DUMMYCYCLES_Len                       (4U)
#define XQSPI_XIP_CFG_DUMMYCYCLES_Msk                       (0xFU << XQSPI_XIP_CFG_DUMMYCYCLES_Pos)
#define XQSPI_XIP_CFG_DUMMYCYCLES                           XQSPI_XIP_CFG_DUMMYCYCLES_Msk

#define XQSPI_XIP_CFG_HPMODE_Pos                            (0U)
#define XQSPI_XIP_CFG_HPMODE_Len                            (8U)
#define XQSPI_XIP_CFG_HPMODE_Msk                            (0xFFUL << XQSPI_XIP_CFG_HPMODE_Pos)
#define XQSPI_XIP_CFG_HPMODE                                XQSPI_XIP_CFG_HPMODE_Msk

/*******************  Bit definition for XQSPI_XIP_EN register  ***************/
#define XQSPI_XIP_EN_REQ_Pos                                (0U)
#define XQSPI_XIP_EN_REQ_Len                                (1U)
#define XQSPI_XIP_EN_REQ_Msk                                (0x1U << XQSPI_XIP_EN_REQ_Pos)
#define XQSPI_XIP_EN_REQ                                    XQSPI_XIP_EN_REQ_Msk

#define XQSPI_XIP_EN_OUT_Pos                                (0U)
#define XQSPI_XIP_EN_OUT_Len                                (1U)
#define XQSPI_XIP_EN_OUT_Msk                                (0x1U << XQSPI_XIP_EN_OUT_Pos)
#define XQSPI_XIP_EN_OUT                                    XQSPI_XIP_EN_OUT_Msk

/*******************  Bit definition for XQSPI_XIP_INT0 register  *************/
#define XQSPI_XIP_INT_EN_Pos                                (0U)
#define XQSPI_XIP_INT_EN_Len                                (1U)
#define XQSPI_XIP_INT_EN_Msk                                (0x1U << XQSPI_XIP_INT_EN_Pos)
#define XQSPI_XIP_INT_EN                                    XQSPI_XIP_INT_EN_Msk

/*******************  Bit definition for XQSPI_XIP_INT1 register  *************/
#define XQSPI_XIP_INT_STAT_Pos                              (0U)
#define XQSPI_XIP_INT_STAT_Len                              (1U)
#define XQSPI_XIP_INT_STAT_Msk                              (0x1U << XQSPI_XIP_INT_STAT_Pos)
#define XQSPI_XIP_INT_STAT                                  XQSPI_XIP_INT_STAT_Msk

/*******************  Bit definition for XQSPI_XIP_INT2 register  *************/
#define XQSPI_XIP_INT_REQ_Pos                               (0U)
#define XQSPI_XIP_INT_REQ_Len                               (1U)
#define XQSPI_XIP_INT_REQ_Msk                               (0x1U << XQSPI_XIP_INT_REQ_Pos)
#define XQSPI_XIP_INT_REQ                                   XQSPI_XIP_INT_REQ_Msk

/*******************  Bit definition for XQSPI_XIP_INT3 register  *************/
#define XQSPI_XIP_INT_SET_Pos                               (0U)
#define XQSPI_XIP_INT_SET_Len                               (1U)
#define XQSPI_XIP_INT_SET_Msk                               (0x1U << XQSPI_XIP_INT_SET_Pos)
#define XQSPI_XIP_INT_SET                                   XQSPI_XIP_INT_SET_Msk

/*******************  Bit definition for XQSPI_XIP_INT4 register  *************/
#define XQSPI_XIP_INT_CLR_Pos                               (0U)
#define XQSPI_XIP_INT_CLR_Len                               (1U)
#define XQSPI_XIP_INT_CLR_Msk                               (0x1U << XQSPI_XIP_INT_CLR_Pos)
#define XQSPI_XIP_INT_CLR                                   XQSPI_XIP_INT_CLR_Msk

/*******************  Bit definition for XQSPI_XIP_SOFT_RST register  *************/
#define XQSPI_XIP_SOFT_RST_Pos                              (0U)
#define XQSPI_XIP_SOFT_RST_Len                              (1U)
#define XQSPI_XIP_SOFT_RST_Msk                              (0x1U << XQSPI_XIP_SOFT_RST_Pos)
#define XQSPI_XIP_SOFT_RST                                  XQSPI_XIP_SOFT_RST_Msk

/*******************  Bit definition for XQSPI_QSPI_STAT register  ************/
#define XQSPI_QSPI_STAT_RXFULL_Pos                          (7U)
#define XQSPI_QSPI_STAT_RXFULL_Len                          (1U)
#define XQSPI_QSPI_STAT_RXFULL_Msk                          (0x1U << XQSPI_QSPI_STAT_RXFULL_Pos)
#define XQSPI_QSPI_STAT_RXFULL                              XQSPI_QSPI_STAT_RXFULL_Msk

#define XQSPI_QSPI_STAT_RXWMARK_Pos                         (6U)
#define XQSPI_QSPI_STAT_RXWMARK_Len                         (1U)
#define XQSPI_QSPI_STAT_RXWMARK_Msk                         (0x1U << XQSPI_QSPI_STAT_RXWMARK_Pos)
#define XQSPI_QSPI_STAT_RXWMARK                             XQSPI_QSPI_STAT_RXWMARK_Msk

#define XQSPI_QSPI_STAT_RXEMPTY_Pos                         (5U)
#define XQSPI_QSPI_STAT_RXEMPTY_Len                         (1U)
#define XQSPI_QSPI_STAT_RXEMPTY_Msk                         (0x1U << XQSPI_QSPI_STAT_RXEMPTY_Pos)
#define XQSPI_QSPI_STAT_RXEMPTY                             XQSPI_QSPI_STAT_RXEMPTY_Msk

#define XQSPI_QSPI_STAT_TXFULL_Pos                          (4U)
#define XQSPI_QSPI_STAT_TXFULL_Len                          (1U)
#define XQSPI_QSPI_STAT_TXFULL_Msk                          (0x1U << XQSPI_QSPI_STAT_TXFULL_Pos)
#define XQSPI_QSPI_STAT_TXFULL                              XQSPI_QSPI_STAT_TXFULL_Msk

#define XQSPI_QSPI_STAT_TXWMARK_Pos                         (3U)
#define XQSPI_QSPI_STAT_TXWMARK_Len                         (1U)
#define XQSPI_QSPI_STAT_TXWMARK_Msk                         (0x1U << XQSPI_QSPI_STAT_TXWMARK_Pos)
#define XQSPI_QSPI_STAT_TXWMARK                             XQSPI_QSPI_STAT_TXWMARK_Msk

#define XQSPI_QSPI_STAT_TXEMPTY_Pos                         (2U)
#define XQSPI_QSPI_STAT_TXEMPTY_Len                         (1U)
#define XQSPI_QSPI_STAT_TXEMPTY_Msk                         (0x1U << XQSPI_QSPI_STAT_TXEMPTY_Pos)
#define XQSPI_QSPI_STAT_TXEMPTY                             XQSPI_QSPI_STAT_TXEMPTY_Msk

#define XQSPI_QSPI_STAT_XFERIP_Pos                          (0U)
#define XQSPI_QSPI_STAT_XFERIP_Len                          (1U)
#define XQSPI_QSPI_STAT_XFERIP_Msk                          (0x1U << XQSPI_QSPI_STAT_XFERIP_Pos)
#define XQSPI_QSPI_STAT_XFERIP                              XQSPI_QSPI_STAT_XFERIP_Msk

/*******************  Bit definition for XQSPI_QSPI_FIFO register  ************/
#define XQSPI_QSPI_FIFO_TX_Pos                              (0U)
#define XQSPI_QSPI_FIFO_TX_Len                              (32U)
#define XQSPI_QSPI_FIFO_TX_Msk                              (0xFFFFFFFFU)
#define XQSPI_QSPI_FIFO_TX                                  XQSPI_QSPI_FIFO_TX_Msk

#define XQSPI_QSPI_FIFO_RX_Pos                              (0U)
#define XQSPI_QSPI_FIFO_RX_Len                              (32U)
#define XQSPI_QSPI_FIFO_RX_Msk                              (0xFFFFFFFFU)
#define XQSPI_QSPI_FIFO_RX                                  XQSPI_QSPI_FIFO_RX_Msk

/*******************  Bit definition for XQSPI_QSPI_CTRL register  ************/
#define XQSPI_QSPI_CTRL_TXWMARK_Pos                         (14U)
#define XQSPI_QSPI_CTRL_TXWMARK_Len                         (2U)
#define XQSPI_QSPI_CTRL_TXWMARK_Msk                         (0x3U << XQSPI_QSPI_CTRL_TXWMARK_Pos)
#define XQSPI_QSPI_CTRL_TXWMARK                             XQSPI_QSPI_CTRL_TXWMARK_Msk

#define XQSPI_QSPI_CTRL_RXWMARK_Pos                         (12U)
#define XQSPI_QSPI_CTRL_RXWMARK_Len                         (2U)
#define XQSPI_QSPI_CTRL_RXWMARK_Msk                         (0x3U << XQSPI_QSPI_CTRL_RXWMARK_Pos)
#define XQSPI_QSPI_CTRL_RXWMARK                             XQSPI_QSPI_CTRL_RXWMARK_Msk

#define XQSPI_QSPI_CTRL_MWAITEN_Pos                         (11U)
#define XQSPI_QSPI_CTRL_MWAITEN_Len                         (1U)
#define XQSPI_QSPI_CTRL_MWAITEN_Msk                         (0x1U << XQSPI_QSPI_CTRL_MWAITEN_Pos)
#define XQSPI_QSPI_CTRL_MWAITEN                             XQSPI_QSPI_CTRL_MWAITEN_Msk

#define XQSPI_QSPI_CTRL_DMA_Pos                             (10U)
#define XQSPI_QSPI_CTRL_DMA_Len                             (1U)
#define XQSPI_QSPI_CTRL_DMA_Msk                             (0x1U << XQSPI_QSPI_CTRL_DMA_Pos)
#define XQSPI_QSPI_CTRL_DMA                                 XQSPI_QSPI_CTRL_DMA_Msk

#define XQSPI_QSPI_CTRL_MASTER_Pos                          (5U)
#define XQSPI_QSPI_CTRL_MASTER_Len                          (1U)
#define XQSPI_QSPI_CTRL_MASTER_Msk                          (0x1U << XQSPI_QSPI_CTRL_MASTER_Pos)
#define XQSPI_QSPI_CTRL_MASTER                              XQSPI_QSPI_CTRL_MASTER_Msk

#define XQSPI_QSPI_CTRL_CPOL_Pos                            (4U)
#define XQSPI_QSPI_CTRL_CPOL_Len                            (1U)
#define XQSPI_QSPI_CTRL_CPOL_Msk                            (0x1U << XQSPI_QSPI_CTRL_CPOL_Pos)
#define XQSPI_QSPI_CTRL_CPOL                                XQSPI_QSPI_CTRL_CPOL_Msk

#define XQSPI_QSPI_CTRL_CPHA_Pos                            (3U)
#define XQSPI_QSPI_CTRL_CPHA_Len                            (1U)
#define XQSPI_QSPI_CTRL_CPHA_Msk                            (0x1U << XQSPI_QSPI_CTRL_CPHA_Pos)
#define XQSPI_QSPI_CTRL_CPHA                                XQSPI_QSPI_CTRL_CPHA_Msk

#define XQSPI_QSPI_CTRL_MSB1ST_Pos                          (2U)
#define XQSPI_QSPI_CTRL_MSB1ST_Len                          (1U)
#define XQSPI_QSPI_CTRL_MSB1ST_Msk                          (0x1U << XQSPI_QSPI_CTRL_MSB1ST_Pos)
#define XQSPI_QSPI_CTRL_MSB1ST                              XQSPI_QSPI_CTRL_MSB1ST_Msk

#define XQSPI_QSPI_CTRL_CONTXFER_Pos                        (0U)
#define XQSPI_QSPI_CTRL_CONTXFER_Len                        (1U)
#define XQSPI_QSPI_CTRL_CONTXFER_Msk                        (0x1U << XQSPI_QSPI_CTRL_CONTXFER_Pos)
#define XQSPI_QSPI_CTRL_CONTXFER                            XQSPI_QSPI_CTRL_CONTXFER_Msk

/*******************  Bit definition for XQSPI_QSPI_AUXCTRL register  *********/
#define XQSPI_QSPI_AUXCTRL_CONTXFERX_Pos                    (7U)
#define XQSPI_QSPI_AUXCTRL_CONTXFERX_Len                    (1U)
#define XQSPI_QSPI_AUXCTRL_CONTXFERX_Msk                    (0x1U << XQSPI_QSPI_AUXCTRL_CONTXFERX_Pos)
#define XQSPI_QSPI_AUXCTRL_CONTXFERX                        XQSPI_QSPI_AUXCTRL_CONTXFERX_Msk

#define XQSPI_QSPI_AUXCTRL_BITSIZE_Pos                      (4U)
#define XQSPI_QSPI_AUXCTRL_BITSIZE_Len                      (3U)
#define XQSPI_QSPI_AUXCTRL_BITSIZE_Msk                      (0x7U << XQSPI_QSPI_AUXCTRL_BITSIZE_Pos)
#define XQSPI_QSPI_AUXCTRL_BITSIZE                          XQSPI_QSPI_AUXCTRL_BITSIZE_Msk

#define XQSPI_QSPI_AUXCTRL_INHIBITDIN_Pos                   (3U)
#define XQSPI_QSPI_AUXCTRL_INHIBITDIN_Len                   (1U)
#define XQSPI_QSPI_AUXCTRL_INHIBITDIN_Msk                   (0x1U << XQSPI_QSPI_AUXCTRL_INHIBITDIN_Pos)
#define XQSPI_QSPI_AUXCTRL_INHIBITDIN                       XQSPI_QSPI_AUXCTRL_INHIBITDIN_Msk

#define XQSPI_QSPI_AUXCTRL_INHIBITDOUT_Pos                  (2U)
#define XQSPI_QSPI_AUXCTRL_INHIBITDOUT_Len                  (1U)
#define XQSPI_QSPI_AUXCTRL_INHIBITDOUT_Msk                  (0x1U << XQSPI_QSPI_AUXCTRL_INHIBITDOUT_Pos)
#define XQSPI_QSPI_AUXCTRL_INHIBITDOUT                      XQSPI_QSPI_AUXCTRL_INHIBITDOUT_Msk

#define XQSPI_QSPI_AUXCTRL_QMODE_Pos                        (0U)
#define XQSPI_QSPI_AUXCTRL_QMODE_Len                        (2U)
#define XQSPI_QSPI_AUXCTRL_QMODE_Msk                        (0x3U << XQSPI_QSPI_AUXCTRL_QMODE_Pos)
#define XQSPI_QSPI_AUXCTRL_QMODE                            XQSPI_QSPI_AUXCTRL_QMODE_Msk

/*******************  Bit definition for XQSPI_QSPI_SS register  **************/
#define XQSPI_QSPI_SS_OUT3_Pos                              (3U)
#define XQSPI_QSPI_SS_OUT3_Len                              (1U)
#define XQSPI_QSPI_SS_OUT3_Msk                              (0x1U << XQSPI_QSPI_SS_OUT3_Pos)
#define XQSPI_QSPI_SS_OUT3                                  XQSPI_QSPI_SS_OUT3_Msk

#define XQSPI_QSPI_SS_OUT2_Pos                              (2U)
#define XQSPI_QSPI_SS_OUT2_Len                              (1U)
#define XQSPI_QSPI_SS_OUT2_Msk                              (0x1U << XQSPI_QSPI_SS_OUT2_Pos)
#define XQSPI_QSPI_SS_OUT2                                  XQSPI_QSPI_SS_OUT2_Msk

#define XQSPI_QSPI_SS_OUT1_Pos                              (1U)
#define XQSPI_QSPI_SS_OUT1_Len                              (1U)
#define XQSPI_QSPI_SS_OUT1_Msk                              (0x1U << XQSPI_QSPI_SS_OUT1_Pos)
#define XQSPI_QSPI_SS_OUT1                                  XQSPI_QSPI_SS_OUT1_Msk

#define XQSPI_QSPI_SS_OUT0_Pos                              (0U)
#define XQSPI_QSPI_SS_OUT0_Len                              (1U)
#define XQSPI_QSPI_SS_OUT0_Msk                              (0x1U << XQSPI_QSPI_SS_OUT0_Pos)
#define XQSPI_QSPI_SS_OUT0                                  XQSPI_QSPI_SS_OUT0_Msk

/*******************  Bit definition for XQSPI_QSPI_SS_POL register  **********/
#define XQSPI_QSPI_SS_POL3_Pos                              (3U)
#define XQSPI_QSPI_SS_POL3_Len                              (1U)
#define XQSPI_QSPI_SS_POL3_Msk                              (0x1U << XQSPI_QSPI_SS_POL3_Pos)
#define XQSPI_QSPI_SS_POL3                                  XQSPI_QSPI_SS_POL3_Msk

#define XQSPI_QSPI_SS_POL2_Pos                              (2U)
#define XQSPI_QSPI_SS_POL2_Len                              (1U)
#define XQSPI_QSPI_SS_POL2_Msk                              (0x1U << XQSPI_QSPI_SS_POL2_Pos)
#define XQSPI_QSPI_SS_POL2                                  XQSPI_QSPI_SS_POL2_Msk

#define XQSPI_QSPI_SS_POL1_Pos                              (1U)
#define XQSPI_QSPI_SS_POL1_Len                              (1U)
#define XQSPI_QSPI_SS_POL1_Msk                              (0x1U << XQSPI_QSPI_SS_POL1_Pos)
#define XQSPI_QSPI_SS_POL1                                  XQSPI_QSPI_SS_POL1_Msk

#define XQSPI_QSPI_SS_POL0_Pos                              (0U)
#define XQSPI_QSPI_SS_POL0_Len                              (1U)
#define XQSPI_QSPI_SS_POL0_Msk                              (0x1U << XQSPI_QSPI_SS_POL0_Pos)
#define XQSPI_QSPI_SS_POL0                                  XQSPI_QSPI_SS_POL0_Msk

/*******************  Bit definition for XQSPI_QSPI_INT_EN register  **********/
#define XQSPI_QSPI_INT_EN_Pos                               (0U)
#define XQSPI_QSPI_INT_EN_Len                               (7U)
#define XQSPI_QSPI_INT_EN_Msk                               (0x7FUL << XQSPI_QSPI_INT_EN_Pos)
#define XQSPI_QSPI_INT_EN                                   XQSPI_QSPI_INT_EN_Msk

/*******************  Bit definition for XQSPI_QSPI_INT_STAT register  ********/
#define XQSPI_QSPI_INT_STAT_Pos                             (0U)
#define XQSPI_QSPI_INT_STAT_Len                             (7U)
#define XQSPI_QSPI_INT_STAT_Msk                             (0x7FUL << XQSPI_QSPI_INT_STAT_Pos)
#define XQSPI_QSPI_INT_STAT                                 XQSPI_QSPI_INT_STAT_Msk

/*******************  Bit definition for XQSPI_QSPI_INT_CLR register  *********/
#define XQSPI_QSPI_INT_CLR_Pos                              (0U)
#define XQSPI_QSPI_INT_CLR_Len                              (7U)
#define XQSPI_QSPI_INT_CLR_Msk                              (0x7FUL << XQSPI_QSPI_INT_CLR_Pos)
#define XQSPI_QSPI_INT_CLR                                  XQSPI_QSPI_INT_CLR_Msk

/*******************  Bit definition for XQSPI Interrupt Bit Mapping  *********/
#define XQSPI_QSPI_GPI_HI_PULSE1_Pos                        (6U)
#define XQSPI_QSPI_GPI_HI_PULSE1_Len                        (1U)
#define XQSPI_QSPI_GPI_HI_PULSE1_Msk                        (0x1U << XQSPI_QSPI_GPI_HI_PULSE1_Pos)
#define XQSPI_QSPI_GPI_HI_PULSE0_Pos                        (5U)
#define XQSPI_QSPI_GPI_HI_PULSE0_Len                        (1U)
#define XQSPI_QSPI_GPI_HI_PULSE0_Msk                        (0x1U << XQSPI_QSPI_GPI_HI_PULSE0_Pos)
#define XQSPI_QSPI_XFER_DPULSE_Pos                          (4U)
#define XQSPI_QSPI_XFER_DPULSE_Len                          (1U)
#define XQSPI_QSPI_XFER_DPULSE_Msk                          (0x1U << XQSPI_QSPI_XFER_DPULSE_Pos)
#define XQSPI_QSPI_RX_FPULSE_Pos                            (3U)
#define XQSPI_QSPI_RX_FPULSE_Len                            (1U)
#define XQSPI_QSPI_RX_FPULSE_Msk                            (0x1U << XQSPI_QSPI_RX_FPULSE_Pos)
#define XQSPI_QSPI_RX_WPULSE_Pos                            (2U)
#define XQSPI_QSPI_RX_WPULSE_Len                            (1U)
#define XQSPI_QSPI_RX_WPULSE_Msk                            (0x1U << XQSPI_QSPI_RX_WPULSE_Pos)
#define XQSPI_QSPI_TX_WPULSE_Pos                            (1U)
#define XQSPI_QSPI_TX_WPULSE_Len                            (1U)
#define XQSPI_QSPI_TX_WPULSE_Msk                            (0x1U << XQSPI_QSPI_TX_WPULSE_Pos)
#define XQSPI_QSPI_TX_EPULSE_Pos                            (0U)
#define XQSPI_QSPI_TX_EPULSE_Len                            (1U)
#define XQSPI_QSPI_TX_EPULSE_Msk                            (0x1U << XQSPI_QSPI_TX_EPULSE_Pos)

/*******************  Bit definition for XQSPI_QSPI_TXFIFOLVL register  *******/
#define XQSPI_QSPI_TXFIFOLVL_Pos                            (0U)
#define XQSPI_QSPI_TXFIFOLVL_Len                            (7U)
#define XQSPI_QSPI_TXFIFOLVL_Msk                            (0x7FUL << XQSPI_QSPI_TXFIFOLVL_Pos)
#define XQSPI_QSPI_TXFIFOLVL                                XQSPI_QSPI_TXFIFOLVL_Msk

/*******************  Bit definition for XQSPI_QSPI_RXFIFOLVL register  *******/
#define XQSPI_QSPI_RXFIFOLVL_Pos                            (0U)
#define XQSPI_QSPI_RXFIFOLVL_Len                            (7U)
#define XQSPI_QSPI_RXFIFOLVL_Msk                            (0x7FUL << XQSPI_QSPI_RXFIFOLVL_Pos)
#define XQSPI_QSPI_RXFIFOLVL                                XQSPI_QSPI_RXFIFOLVL_Msk

/*******************  Bit definition for XQSPI_QSPI_MWAIT register  ***********/
#define XQSPI_QSPI_MWAIT_MWAIT_Pos                          (0U)
#define XQSPI_QSPI_MWAIT_MWAIT_Len                          (8U)
#define XQSPI_QSPI_MWAIT_MWAIT_Msk                          (0xFFUL << XQSPI_QSPI_MWAIT_MWAIT_Pos)
#define XQSPI_QSPI_MWAIT_MWAIT                              XQSPI_QSPI_MWAIT_MWAIT_Msk

/*******************  Bit definition for XQSPI_QSPI_EN register  **************/
#define XQSPI_QSPI_EN_EN_Pos                                (0U)
#define XQSPI_QSPI_EN_EN_Len                                (1U)
#define XQSPI_QSPI_EN_EN_Msk                                (0x1U << XQSPI_QSPI_EN_EN_Pos)
#define XQSPI_QSPI_EN_EN                                    XQSPI_QSPI_EN_EN_Msk

/*******************  Bit definition for XQSPI_QSPI_GPOSET_GPOSET register  ***/
#define XQSPI_QSPI_GPOSET_GPOSET_Pos                        (0U)
#define XQSPI_QSPI_GPOSET_GPOSET_Len                        (8U)
#define XQSPI_QSPI_GPOSET_GPOSET_Msk                        (0xFFUL << XQSPI_QSPI_GPOSET_GPOSET_Pos)
#define XQSPI_QSPI_GPOSET_GPOSET                            XQSPI_QSPI_GPOSET_GPOSET_Msk

/*******************  Bit definition for XQSPI_QSPI_GPOCLR_GPOCLR register  ***/
#define XQSPI_QSPI_GPOCLR_GPOCLR_Pos                        (0U)
#define XQSPI_QSPI_GPOCLR_GPOCLR_Len                        (8U)
#define XQSPI_QSPI_GPOCLR_GPOCLR_Msk                        (0xFFUL << XQSPI_QSPI_GPOCLR_GPOCLR_Pos)
#define XQSPI_QSPI_GPOCLR_GPOCLR                            XQSPI_QSPI_GPOCLR_GPOCLR_Msk

/*******************  Bit definition for XQSPI_QSPI_FLASH_WRITE register  ***/
#define XQSPI_QSPI_FLASH_WRITE_Pos                          (0U)
#define XQSPI_QSPI_FLASH_WRITE_Len                          (1U)
#define XQSPI_QSPI_FLASH_WRITE_Msk                          (0xFFUL << XQSPI_QSPI_FLASH_WRITE_Pos)
#define XQSPI_QSPI_FLASH_WRITE                              XQSPI_QSPI_FLASH_WRITE_Msk

/*******************  Bit definition for XQSPI_QSPI_PRESENT_BYPASS register  ***/
#define XQSPI_QSPI_PRESENT_BYPASS_Pos                       (0U)
#define XQSPI_QSPI_PRESENT_BYPASS_Len                       (1U)
#define XQSPI_QSPI_PRESENT_BYPASS_Msk                       (0xFFUL << XQSPI_QSPI_PRESENT_BYPASS_Pos)
#define XQSPI_QSPI_PRESENT_BYPASS                           XQSPI_QSPI_PRESENT_BYPASS_Msk

/*******************  Bit definition for XQSPI_QSPI_CS_IDLE_UNVLD_EN register  ***/
#define XQSPI_QSPI_CS_IDLE_UNVLD_EN_Pos                     (0U)
#define XQSPI_QSPI_CS_IDLE_UNVLD_EN_Len                     (1U)
#define XQSPI_QSPI_CS_IDLE_UNVLD_EN_Msk                     (0x1U << XQSPI_QSPI_CS_IDLE_UNVLD_EN_Pos)
#define XQSPI_QSPI_CS_IDLE_UNVLD_EN                         XQSPI_QSPI_CS_IDLE_UNVLD_EN_Msk

#define XQSPI_QSPI_1ST_PRETETCH_DIS_Pos                     (1U)
#define XQSPI_QSPI_1ST_PRETETCH_DIS_Len                     (1U)
#define XQSPI_QSPI_1ST_PRETETCH_DIS_Msk                     (0x1U << XQSPI_QSPI_1ST_PRETETCH_DIS_Pos)
#define XQSPI_QSPI_1ST_PRETETCH_DIS                         XQSPI_QSPI_1ST_PRETETCH_DIS_Msk

#define XQSPI_QSPI_KEY_PULSE_DIS_Pos                        (2U)
#define XQSPI_QSPI_KEY_PULSE_DIS_Len                        (1U)
#define XQSPI_QSPI_KEY_PULSE_DIS_Msk                        (0x1U << XQSPI_QSPI_KEY_PULSE_DIS_Pos)
#define XQSPI_QSPI_KEY_PULSE_DIS                            XQSPI_QSPI_KEY_PULSE_DIS_Msk

/* =============================================================================================================== */
/* ================                                       EFUSE                                   ================ */
/* =============================================================================================================== */
/*******************  Bit definition for EFUSE_TPGM register  **********/
#define EFUSE_TPGM_TIME_Pos                                 (0U)
#define EFUSE_TPGM_TIME_Len                                 (12U)
#define EFUSE_TPGM_TIME_Msk                                 (0xFFFUL << EFUSE_TPGM_TIME_Pos)
#define EFUSE_TPGM_TIME                                     EFUSE_TPGM_TIME_Msk

#define EFUSE_TPGM_MAIN_OR_BACKUP_Pos                       (12U)
#define EFUSE_TPGM_MAIN_OR_BACKUP_Len                       (1U)
#define EFUSE_TPGM_MAIN_OR_BACKUP_Msk                       (0x1UL << EFUSE_TPGM_MAIN_OR_BACKUP_Pos)
#define EFUSE_TPGM_MAIN_OR_BACKUP                           EFUSE_TPGM_MAIN_OR_BACKUP_Msk

#define EFUSE_TPGM_CRC_CHECK_LEN_Pos                        (16U)
#define EFUSE_TPGM_CRC_CHECK_LEN_Len                        (6U)
#define EFUSE_TPGM_CRC_CHECK_LEN_Msk                        (0x3FUL << EFUSE_TPGM_CRC_CHECK_LEN_Pos)
#define EFUSE_TPGM_CRC_CHECK_LEN                            EFUSE_TPGM_CRC_CHECK_LEN_Msk

#define EFUSE_TPGM_WRITE_INTERVAL_Pos                       (24U)
#define EFUSE_TPGM_WRITE_INTERVAL_Len                       (8U)
#define EFUSE_TPGM_WRITE_INTERVAL_Msk                       (0xFFUL << EFUSE_TPGM_WRITE_INTERVAL_Pos)
#define EFUSE_TPGM_WRITE_INTERVAL                           EFUSE_TPGM_WRITE_INTERVAL_Msk

/*******************  Bit definition for EFUSE_PGENB register  **********/
#define EFUSE_PGENB_SIG_Pos                                 (0U)
#define EFUSE_PGENB_SIG_Len                                 (1U)
#define EFUSE_PGENB_SIG_Msk                                 (0x1UL << EFUSE_PGENB_SIG_Pos)
#define EFUSE_PGENB_SIG                                     EFUSE_PGENB_SIG_Msk

/*******************  Bit definition for EFUSE_TEST_MODE register  **********/
#define EFUSE_TEST_MODE_Pos                                 (0U)
#define EFUSE_TEST_MODE_Len                                 (16U)
#define EFUSE_TEST_MODE_Msk                                 (0xFFFFUL << EFUSE_TEST_MODE_Pos)
#define EFUSE_TEST_MODE                                     EFUSE_TEST_MODE_Msk

/*******************  Bit definition for EFUSE_OPERATION register  **********/
#define EFUSE_OPER_WRITE_KEYRAM_Pos                         (0U)
#define EFUSE_OPER_WRITE_KEYRAM_Len                         (1U)
#define EFUSE_OPER_WRITE_KEYRAM_Msk                         (0x1UL << EFUSE_OPER_WRITE_KEYRAM_Pos)
#define EFUSE_OPER_WRITE_KEYRAM                             EFUSE_OPER_WRITE_KEYRAM_Msk

#define EFUSE_OPER_INIT_CHECK_Pos                           (1U)
#define EFUSE_OPER_INIT_CHECK_Len                           (1U)
#define EFUSE_OPER_INIT_CHECK_Msk                           (0x1UL << EFUSE_OPER_INIT_CHECK_Pos)
#define EFUSE_OPER_INIT_CHECK                               EFUSE_OPER_INIT_CHECK_Msk

#define EFUSE_OPER_CRC_CHECK_Pos                            (2U)
#define EFUSE_OPER_CRC_CHECK_Len                            (1U)
#define EFUSE_OPER_CRC_CHECK_Msk                            (0x1UL << EFUSE_OPER_CRC_CHECK_Pos)
#define EFUSE_OPER_CRC_CHECK                                EFUSE_OPER_CRC_CHECK_Msk

#define EFUSE_OPER_READ_TRIM_Pos                            (3U)
#define EFUSE_OPER_READ_TRIM_Len                            (1U)
#define EFUSE_OPER_READ_TRIM_Msk                            (0x1UL << EFUSE_OPER_READ_TRIM_Pos)
#define EFUSE_OPER_READ_TRIM                                EFUSE_OPER_READ_TRIM_Msk

#define EFUSE_OPER_RD_TEST_MODE_Pos                         (4U)
#define EFUSE_OPER_RD_TEST_MODE_Len                         (1U)
#define EFUSE_OPER_RD_TEST_MODE_Msk                         (0x1UL << EFUSE_OPER_RD_TEST_MODE_Pos)
#define EFUSE_OPER_RD_TEST_MODE                             EFUSE_OPER_RD_TEST_MODE_Msk

/*******************  Bit definition for EFUSE_STATUS register  **********/
#define EFUSE_STATUS_WRITE_KEYRAM_BUSY_Pos                  (0U)
#define EFUSE_STATUS_WRITE_KEYRAM_BUSY_Len                  (1U)
#define EFUSE_STATUS_WRITE_KEYRAM_BUSY_Msk                  (0x1UL << EFUSE_STATUS_WRITE_KEYRAM_BUSY_Pos)
#define EFUSE_STATUS_WRITE_KEYRAM_BUSY                      EFUSE_STATUS_WRITE_KEYRAM_BUSY_Msk

#define EFUSE_STATUS_READ_TRIM_DONE_Pos                     (1U)
#define EFUSE_STATUS_READ_TRIM_DONE_Len                     (1U)
#define EFUSE_STATUS_READ_TRIM_DONE_Msk                     (0x1UL << EFUSE_STATUS_READ_TRIM_DONE_Pos)
#define EFUSE_STATUS_READ_TRIM_DONE                         EFUSE_STATUS_READ_TRIM_DONE_Msk

#define EFUSE_STATUS_TRIM_CRC_SUCCESS_Pos                   (2U)
#define EFUSE_STATUS_TRIM_CRC_SUCCESS_Len                   (1U)
#define EFUSE_STATUS_TRIM_CRC_SUCCESS_Msk                   (0x1UL << EFUSE_STATUS_TRIM_CRC_SUCCESS_Pos)
#define EFUSE_STATUS_TRIM_CRC_SUCCESS                       EFUSE_STATUS_TRIM_CRC_SUCCESS_Msk

#define EFUSE_STATUS_INIT_DONE_Pos                          (3U)
#define EFUSE_STATUS_INIT_DONE_Len                          (1U)
#define EFUSE_STATUS_INIT_DONE_Msk                          (0x1UL << EFUSE_STATUS_INIT_DONE_Pos)
#define EFUSE_STATUS_INIT_DONE                              EFUSE_STATUS_INIT_DONE_Msk

#define EFUSE_STATUS_INIT_SUCCESS_Pos                       (4U)
#define EFUSE_STATUS_INIT_SUCCESS_Len                       (1U)
#define EFUSE_STATUS_INIT_SUCCESS_Msk                       (0x1UL << EFUSE_STATUS_INIT_SUCCESS_Pos)
#define EFUSE_STATUS_INIT_SUCCESS                           EFUSE_STATUS_INIT_SUCCESS_Msk

#define EFUSE_STATUS_CRC_CHECK_DONE_Pos                     (5U)
#define EFUSE_STATUS_CRC_CHECK_DONE_Len                     (1U)
#define EFUSE_STATUS_CRC_CHECK_DONE_Msk                     (0x1UL << EFUSE_STATUS_CRC_CHECK_DONE_Pos)
#define EFUSE_STATUS_CRC_CHECK_DONE                         EFUSE_STATUS_CRC_CHECK_DONE_Msk

#define EFUSE_STATUS_WRITE_DONE_Pos                         (6U)
#define EFUSE_STATUS_WRITE_DONE_Len                         (1U)
#define EFUSE_STATUS_WRITE_DONE_Msk                         (0x1UL << EFUSE_STATUS_WRITE_DONE_Pos)
#define EFUSE_STATUS_WRITE_DONE                             EFUSE_STATUS_WRITE_DONE_Msk

#define EFUSE_STATUS_TEST_MODE_DONE_Pos                     (7U)
#define EFUSE_STATUS_TEST_MODE_DONE_Len                     (1U)
#define EFUSE_STATUS_TEST_MODE_DONE_Msk                     (0x1UL << EFUSE_STATUS_TEST_MODE_DONE_Pos)
#define EFUSE_STATUS_TEST_MODE_DONE                         EFUSE_STATUS_TEST_MODE_DONE_Msk

/*******************  Bit definition for EFUSE_KEY_MASK register  **********/
#define EFUSE_KEY_MASK_Pos                                  (0U)
#define EFUSE_KEY_MASK_Len                                  (32U)
#define EFUSE_KEY_MASK_Msk                                  (0x1UL << EFUSE_KEY_MASK_Pos)
#define EFUSE_KEY_MASK                                      EFUSE_KEY_MASK_Msk

/*******************  Bit definition for EFUSE_CRC_START_ADDR register  **********/
#define EFUSE_CRC_START_CHECK_ADDR_Pos                      (0U)
#define EFUSE_CRC_START_CHECK_ADDR_Len                      (32U)
#define EFUSE_CRC_START_CHECK_ADDR_Msk                      (0xFFFFFFFFU)
#define EFUSE_CRC_START_CHECK_ADDR                          EFUSE_CRC_START_CHECK_ADDR_Msk

/*******************  Bit definition for EFUSE_CRC_OUTPUT register  **********/
#define EFUSE_CRC_OUTPUT_VALUE_Pos                          (0U)
#define EFUSE_CRC_OUTPUT_VALUE_Len                          (32U)
#define EFUSE_CRC_OUTPUT_VALUE_Msk                          (0xFFFFFFFFU)
#define EFUSE_CRC_OUTPUT_VALUE                              EFUSE_CRC_OUTPUT_VALUE_Msk

/*******************  Bit definition for EFUSE_TRIM_ADDR register  **********/
#define EFUSE_TRIM_START_ADDR_Pos                           (0U)
#define EFUSE_TRIM_START_ADDR_Len                           (32U)
#define EFUSE_TRIM_START_ADDR_Msk                           (0xFFFFFFFFU)
#define EFUSE_TRIM_START_ADDR                               EFUSE_TRIM_START_ADDR_Msk

/*******************  Bit definition for EFUSE_TRIM_LEN register  **********/
#define EFUSE_TRIM_LENGTH_Pos                               (0U)
#define EFUSE_TRIM_LENGTH_Len                               (5U)
#define EFUSE_TRIM_LENGTH_Msk                               (0x1FU << EFUSE_TRIM_LENGTH_Pos)
#define EFUSE_TRIM_LENGTH                                   EFUSE_TRIM_LENGTH_Msk

/*******************  Bit definition for EFUSE_TRIM register  **************/
#define EFUSE_TRIM_Pos                                      (0U)
#define EFUSE_TRIM_Len                                      (32U)
#define EFUSE_TRIM_Msk                                      (0xFFFFFFFFU)
#define EFUSE_TRIM                                          EFUSE_TRIM_Msk

/* =============================================================================================================== */
/* ================                                      RNG                                      ================ */
/* =============================================================================================================== */
/*******************  Bit definition for RNG_CTRL register  **********/
#define RNG_CTRL_RUN_EN_Pos                                 (0U)
#define RNG_CTRL_RUN_EN_Len                                 (1U)
#define RNG_CTRL_RUN_EN_Msk                                 (0x1UL << RNG_CTRL_RUN_EN_Pos)
#define RNG_CTRL_RUN_EN                                     RNG_CTRL_RUN_EN_Msk

/*******************  Bit definition for RNG_STATUS register  **********/
#define RNG_STATUS_READY_Pos                                (0U)
#define RNG_STATUS_READY_Len                                (1U)
#define RNG_STATUS_READY_Msk                                (0x1UL << RNG_STATUS_READY_Pos)
#define RNG_STATUS_READY                                    RNG_STATUS_READY_Msk

/*******************  Bit definition for RNG_DATA register  **********/
#define RNG_DATA_VALUE_Pos                                  (0U)
#define RNG_DATA_VALUE_Len                                  (32U)
#define RNG_DATA_VALUE_Msk                                  (0xFFFFFFFF)
#define RNG_DATA_VALUE                                      RNG_DATA_VALUE_Msk

/*******************  Bit definition for RNG_LR_STATUS register  *********/
#define RNG_LR_STATUS_FLAG_Pos                              (0U)
#define RNG_LR_STATUS_FLAG_Len                              (1U)
#define RNG_LR_STATUS_FLAG_Msk                              (0x1UL << RNG_LR_STATUS_FLAG_Pos)
#define RNG_LR_STATUS_FLAG                                  RNG_LR_STATUS_FLAG_Msk
#define RNG_LR_STATUS_CNT_Pos                               (1U)
#define RNG_LR_STATUS_CNT_Len                               (8U)
#define RNG_LR_STATUS_CNT_Msk                               (0xFFUL << RNG_LR_STATUS_CNT_Pos)
#define RNG_LR_STATUS_CNT                                   RNG_LR_STATUS_CNT_Msk

/*******************  Bit definition for RNG_CONFIG register  ************/
#define RNG_CONFIG_OUT_MODE_Pos                             (0U)
#define RNG_CONFIG_OUT_MODE_Len                             (4U)
#define RNG_CONFIG_OUT_MODE_Msk                             (0xFUL << RNG_CONFIG_OUT_MODE_Pos)
#define RNG_CONFIG_OUT_MODE                                 RNG_CONFIG_OUT_MODE_Msk
#define RNG_CONFIG_LFSR_XOR_SEL_Pos                         (4U)
#define RNG_CONFIG_LFSR_XOR_SEL_Len                         (3U)
#define RNG_CONFIG_LFSR_XOR_SEL_Msk                         (0x7UL << RNG_CONFIG_LFSR_XOR_SEL_Pos)
#define RNG_CONFIG_LFSR_XOR_SEL                             RNG_CONFIG_LFSR_XOR_SEL_Msk
#define RNG_CONFIG_POST_MODE_Pos                            (7U)
#define RNG_CONFIG_POST_MODE_Len                            (2U)
#define RNG_CONFIG_POST_MODE_Msk                            (0x3UL << RNG_CONFIG_POST_MODE_Pos)
#define RNG_CONFIG_POST_MODE                                RNG_CONFIG_POST_MODE_Msk
#define RNG_CONFIG_LFSR_MODE_Pos                            (9U)
#define RNG_CONFIG_LFSR_MODE_Len                            (1U)
#define RNG_CONFIG_LFSR_MODE_Msk                            (0x1UL << RNG_CONFIG_LFSR_MODE_Pos)
#define RNG_CONFIG_LFSR_MODE                                RNG_CONFIG_LFSR_MODE_Msk
#define RNG_CONFIG_LFSR_SEED_SEL_Pos                        (10U)
#define RNG_CONFIG_LFSR_SEED_SEL_Len                        (3U)
#define RNG_CONFIG_LFSR_SEED_SEL_Msk                        (0x7UL << RNG_CONFIG_LFSR_SEED_SEL_Pos)
#define RNG_CONFIG_LFSR_SEED_SEL                            RNG_CONFIG_LFSR_SEED_SEL_Msk
#define RNG_CONFIG_IRQ_EN_Pos                               (13U)
#define RNG_CONFIG_IRQ_EN_Len                               (1U)
#define RNG_CONFIG_IRQ_EN_Msk                               (0x1UL << RNG_CONFIG_IRQ_EN_Pos)
#define RNG_CONFIG_IRQ_EN                                   RNG_CONFIG_IRQ_EN_Msk
#define RNG_CONFIG_FRO_EN_Pos                               (15U)
#define RNG_CONFIG_FRO_EN_Len                               (1U)
#define RNG_CONFIG_FRO_EN_Msk                               (0x1UL << RNG_CONFIG_FRO_EN_Pos)
#define RNG_CONFIG_FRO_EN                                   RNG_CONFIG_FRO_EN_Msk

/*******************  Bit definition for RNG_TSCON register  *************/
#define RNG_TSCON_TRDY_TIME_Pos                             (0U)
#define RNG_TSCON_TRDY_TIME_Len                             (8U)
#define RNG_TSCON_TRDY_TIME_Msk                             (0xFUL << RNG_TSCON_TRDY_TIME_Pos)
#define RNG_TSCON_TRDY_TIME                                 RNG_TSCON_TRDY_TIME_Msk
#define RNG_TSCON_FRO_CHAIN_Pos                             (11U)
#define RNG_TSCON_FRO_CHAIN_Len                             (4U)
#define RNG_TSCON_FRO_CHAIN_Msk                             (0xFUL << RNG_TSCON_FRO_CHAIN_Pos)
#define RNG_TSCON_FRO_CHAIN                                 RNG_TSCON_FRO_CHAIN_Msk

/*******************  Bit definition for RNG_FROCFG register  *************/
#define RNG_FROCFG_CHAINE_EN_Pos                            (0U)
#define RNG_FROCFG_CHAINE_EN_Len                            (8U)
#define RNG_FROCFG_CHAINE_EN_Msk                            (0xFFUL << RNG_FROCFG_CHAINE_EN_Pos)
#define RNG_FROCFG_CHAINE_EN                                RNG_FROCFG_CHAINE_EN_Msk
#define RNG_FROCFG_TEST_IN_Pos                              (8U)
#define RNG_FROCFG_TEST_IN_Len                              (8U)
#define RNG_FROCFG_TEST_IN_Msk                              (0xFFUL << RNG_FROCFG_TEST_IN_Pos)
#define RNG_FROCFG_TEST_IN                                  RNG_FROCFG_TEST_IN_Msk

/*******************  Bit definition for RNG_USER_SEED register  *************/
#define RNG_USER_SEED_Pos                                   (0U)
#define RNG_USER_SEED_Len                                   (16U)
#define RNG_USER_SEED_Msk                                   (0xFFUL << RNG_USER_SEED_Pos)
#define RNG_USER_SEED                                       RNG_USER_SEED_Msk

/*******************  Bit definition for RNG_LRCON register  *****************/
#define RNG_LRCON_TEST_EN_Pos                               (0U)
#define RNG_LRCON_TEST_EN_Len                               (1U)
#define RNG_LRCON_TEST_EN_Msk                               (0x1UL << RNG_LRCON_TEST_EN_Pos)
#define RNG_LRCON_TEST_EN                                   RNG_LRCON_TEST_EN_Msk
#define RNG_LRCON_TEST_LIMIT_Pos                            (1U)
#define RNG_LRCON_TEST_LIMIT_Len                            (5U)
#define RNG_LRCON_TEST_LIMIT_Msk                            (0x1FUL << RNG_LRCON_TEST_LIMIT_Pos)
#define RNG_LRCON_TEST_LIMIT                                RNG_LRCON_TEST_LIMIT_Msk

/* =============================================================================================================== */
/* ================                                     DSPI                                      ================ */
/* =============================================================================================================== */
/*******************  Bit definition for DSPI_CR1 register  *****************/
#define DSPI_CR1_CPHA_Pos                                  (0U)
#define DSPI_CR1_CPHA_Len                                   (1)
#define DSPI_CR1_CPHA_Msk                                   (0x1UL << DSPI_CR1_CPHA_Pos)
#define DSPI_CR1_CPHA                                       DSPI_CR1_CPHA_Msk

#define DSPI_CR1_CPOL_Pos                                   (1U)
#define DSPI_CR1_CPOL_Len                                   (1)
#define DSPI_CR1_CPOL_Msk                                   (0x1UL << DSPI_CR1_CPOL_Pos)
#define DSPI_CR1_CPOL                                       DSPI_CR1_CPOL_Msk

#define DSPI_CR1_MSTR_Pos                                   (2U)
#define DSPI_CR1_MSTR_Len                                   (1)
#define DSPI_CR1_MSTR_Msk                                   (0x1UL << DSPI_CR1_MSTR_Pos)
#define DSPI_CR1_MSTR                                       DSPI_CR1_MSTR_Msk

#define DSPI_CR1_BAUD_Pos                                   (3U)
#define DSPI_CR1_BAUD_Len                                   (3)
#define DSPI_CR1_BAUD_Msk                                   (0x7UL << DSPI_CR1_BAUD_Pos)
#define DSPI_CR1_BAUD                                       DSPI_CR1_BAUD_Msk

#define DSPI_CR1_EN_Pos                                     (6U)
#define DSPI_CR1_EN_Len                                     (1)
#define DSPI_CR1_EN_Msk                                     (0x1UL << DSPI_CR1_EN_Pos)
#define DSPI_CR1_EN                                         DSPI_CR1_EN_Msk

#define DSPI_CR1_LSBFIRST_Pos                               (7U)
#define DSPI_CR1_LSBFIRST_Len                               (1)
#define DSPI_CR1_LSBFIRST_Msk                               (0x1UL << DSPI_CR1_LSBFIRST_Pos)
#define DSPI_CR1_LSBFIRST                                   DSPI_CR1_LSBFIRST_Msk

#define DSPI_CR1_SLAVE_Pos                                  (8U)
#define DSPI_CR1_SLAVE_Len                                  (1)
#define DSPI_CR1_SLAVE_Msk                                  (0x1UL << DSPI_CR1_SLAVE_Pos)
#define DSPI_CR1_SLAVE                                      DSPI_CR1_SLAVE_Msk

#define DSPI_CR1_SSM_Pos                                    (9U)
#define DSPI_CR1_SSM_Len                                    (1)
#define DSPI_CR1_SSM_Msk                                    (0x1UL << DSPI_CR1_SSM_Pos)
#define DSPI_CR1_SSM                                        DSPI_CR1_SSM_Msk

#define DSPI_CR1_RXONLY_Pos                                 (10U)
#define DSPI_CR1_RXONLY_Len                                 (1)
#define DSPI_CR1_RXONLY_Msk                                 (0x1UL << DSPI_CR1_RXONLY_Pos)
#define DSPI_CR1_RXONLY                                     DSPI_CR1_RXONLY_Msk

#define DSPI_CR1_FIFOTH_Pos                                 (11U)
#define DSPI_CR1_FIFOTH_Len                                 (2)
#define DSPI_CR1_FIFOTH_Msk                                 (0x3UL << DSPI_CR1_FIFOTH_Pos)
#define DSPI_CR1_FIFOTH                                     DSPI_CR1_FIFOTH_Msk
#define DSPI_CR1_FIFOTH_8                                   (0x0 << DSPI_CR1_FIFOTH_Pos)
#define DSPI_CR1_FIFOTH_4                                   (0x1 << DSPI_CR1_FIFOTH_Pos)
#define DSPI_CR1_FIFOTH_2                                   (0x2 << DSPI_CR1_FIFOTH_Pos)
#define DSPI_CR1_FIFOTH_1                                   (0x3 << DSPI_CR1_FIFOTH_Pos)

#define DSPI_CR1_REFIFO_Pos                                 (13U)
#define DSPI_CR1_REFIFO_Len                                 (1)
#define DSPI_CR1_REFIFO_Msk                                 (0x1UL << DSPI_CR1_REFIFO_Pos)
#define DSPI_CR1_REFIFO                                     DSPI_CR1_REFIFO_Msk

#define DSPI_CR1_BIDIOE_Pos                                 (14U)
#define DSPI_CR1_BIDIOE_Len                                 (1)
#define DSPI_CR1_BIDIOE_Msk                                 (0x1UL << DSPI_CR1_BIDIOE_Pos)
#define DSPI_CR1_BIDIOE                                     DSPI_CR1_BIDIOE_Msk

#define DSPI_CR1_BIDIDE_Pos                                 (15U)
#define DSPI_CR1_BIDIDE_Len                                 (1)
#define DSPI_CR1_BIDIDE_Msk                                 (0x1UL << DSPI_CR1_BIDIDE_Pos)
#define DSPI_CR1_BIDIDE                                     DSPI_CR1_BIDIDE_Msk

/*******************  Bit definition for DSPI_CR2 register  *****************/
#define DSPI_CR2_RXDMAEN_Pos                                (0U)
#define DSPI_CR2_RXDMAEN_Len                                (1)
#define DSPI_CR2_RXDMAEN_Msk                                (0x1UL << DSPI_CR2_RXDMAEN_Pos)
#define DSPI_CR2_RXDMAEN                                    DSPI_CR2_RXDMAEN_Msk

#define DSPI_CR2_TXDMAEN_Pos                                (1U)
#define DSPI_CR2_TXDMAEN_Len                                (1)
#define DSPI_CR2_TXDMAEN_Msk                                (0x1UL << DSPI_CR2_TXDMAEN_Pos)
#define DSPI_CR2_TXDMAEN                                    DSPI_CR2_TXDMAEN_Msk

#define DSPI_CR2_SSOE_Pos                                   (2U)
#define DSPI_CR2_SSOE_Len                                   (1)
#define DSPI_CR2_SSOE_Msk                                   (0x1UL << DSPI_CR2_SSOE_Pos)
#define DSPI_CR2_SSOE                                       DSPI_CR2_SSOE_Msk

#define DSPI_CR2_NSSP_Pos                                   (3U)
#define DSPI_CR2_NSSP_Len                                   (1)
#define DSPI_CR2_NSSP_Msk                                   (0x1UL << DSPI_CR2_NSSP_Pos)
#define DSPI_CR2_NSSP                                       DSPI_CR2_NSSP_Msk

#define DSPI_CR2_FRF_Pos                                    (4U)
#define DSPI_CR2_FRF_Len                                    (1)
#define DSPI_CR2_FRF_Msk                                    (0x1UL << DSPI_CR2_FRF_Pos)
#define DSPI_CR2_FRF                                        DSPI_CR2_FRF_Msk

#define DSPI_CR2_ERRIE_Pos                                  (5U)
#define DSPI_CR2_ERRIE_Len                                  (1)
#define DSPI_CR2_ERRIE_Msk                                  (0x1UL << DSPI_CR2_ERRIE_Pos)
#define DSPI_CR2_ERRIE                                      DSPI_CR2_ERRIE_Msk

#define DSPI_CR2_RXNEIE_Pos                                 (6U)
#define DSPI_CR2_RXNEIE_Len                                 (1)
#define DSPI_CR2_RXNEIE_Msk                                 (0x1UL << DSPI_CR2_RXNEIE_Pos)
#define DSPI_CR2_RXNEIE                                     DSPI_CR2_RXNEIE_Msk

#define DSPI_CR2_TXEIE_Pos                                  (7U)
#define DSPI_CR2_TXEIE_Len                                  (1)
#define DSPI_CR2_TXEIE_Msk                                  (0x1UL << DSPI_CR2_TXEIE_Pos)
#define DSPI_CR2_TXEIE                                      DSPI_CR2_TXEIE_Msk

#define DSPI_CR2_DS_Pos                                     (8U)
#define DSPI_CR2_DS_Len                                     (5)
#define DSPI_CR2_DS_Msk                                     (0x1FUL << DSPI_CR2_DS_Pos)
#define DSPI_CR2_DS                                         DSPI_CR2_DS_Msk

#define DSPI_CR2_LDMA_RX_Pos                                (13U)
#define DSPI_CR2_LDMA_RX_Len                                (1)
#define DSPI_CR2_LDMA_RX_Msk                                (0x1UL << DSPI_CR2_LDMA_RX_Pos)
#define DSPI_CR2_LDMA_RX                                    DSPI_CR2_LDMA_RX_Msk

#define DSPI_CR2_LDMA_TX_Pos                                (14U)
#define DSPI_CR2_LDMA_TX_Len                                (1)
#define DSPI_CR2_LDMA_TX_Msk                                (0x1UL << DSPI_CR2_LDMA_TX_Pos)
#define DSPI_CR2_LDMA_TX                                    DSPI_CR2_LDMA_TX_Msk

/*******************  Bit definition for DSPI_STAT register  *****************/
#define DSPI_STAT_RXNE_Pos                                  (0U)
#define DSPI_STAT_RXNE_Len                                  (1)
#define DSPI_STAT_RXNE_Msk                                  (0x1UL << DSPI_STAT_RXNE_Pos)
#define DSPI_STAT_RXNE                                      DSPI_STAT_RXNE_Msk

#define DSPI_STAT_TXE_Pos                                   (1U)
#define DSPI_STAT_TXE_Len                                   (1)
#define DSPI_STAT_TXE_Msk                                   (0x1UL << DSPI_STAT_TXE_Pos)
#define DSPI_STAT_TXE                                       DSPI_STAT_TXE_Msk

#define DSPI_STAT_MODF_Pos                                  (5U)
#define DSPI_STAT_MODF_Len                                  (1)
#define DSPI_STAT_MODF_Msk                                  (0x1UL << DSPI_STAT_MODF_Pos)
#define DSPI_STAT_MODF                                      DSPI_STAT_MODF_Msk

#define DSPI_STAT_OVR_Pos                                   (6U)
#define DSPI_STAT_OVR_Len                                   (1)
#define DSPI_STAT_OVRF_Msk                                  (0x1UL << DSPI_STAT_OVR_Pos)
#define DSPI_STAT_OVR                                       DSPI_STAT_OVRF_Msk

#define DSPI_STAT_BUSY_Pos                                  (7U)
#define DSPI_STAT_BUSY_Len                                  (1)
#define DSPI_STAT_BUSY_Msk                                  (0x1UL << DSPI_STAT_BUSY_Pos)
#define DSPI_STAT_BUSY                                      DSPI_STAT_BUSY_Msk

#define DSPI_STAT_FRE_Pos                                   (8U)
#define DSPI_STAT_FRE_Len                                   (1)
#define DSPI_STAT_FRE_Msk                                   (0x1UL << DSPI_STAT_FRE_Pos)
#define DSPI_STAT_FRE                                       DSPI_STAT_FRE_Msk

#define DSPI_STAT_FIFORXCNT_Pos                             (9U)
#define DSPI_STAT_FIFORXCNT_Len                             (4)
#define DSPI_STAT_FIFORXCNT_Msk                             (0xFUL << DSPI_STAT_FIFORXCNT_Pos)
#define DSPI_STAT_FIFORXCNT                                 DSPI_STAT_FIFORXCNT_Msk

#define DSPI_STAT_FIFOTXCNT_Pos                             (13U)
#define DSPI_STAT_FIFOTXCNT_Len                             (4)
#define DSPI_STAT_FIFOTXCNT_Msk                             (0xFUL << DSPI_STAT_FIFOTXCNT_Pos)
#define DSPI_STAT_FIFOTXCNT                                 DSPI_STAT_FIFOTXCNT_Msk

/*******************  Bit definition for DSPI_DATA register  *****************/
#define DSPI_DATA_Pos                                       (0U)
#define DSPI_DATA_Len                                       (32)
#define DSPI_DATA_Msk                                       (0xFFFFFFFFU)
#define DSPI_DATA                                           DSPI_DATA_Msk

/*******************  Bit definition for DSPI_MODE register  *****************/
#define DSPI_MODE_SPIMODE_Pos                               (0U)
#define DSPI_MODE_SPIMODE_Len                               (2)
#define DSPI_MODE_SPIMODE_Msk                               (0x3UL << DSPI_MODE_SPIMODE_Pos)
#define DSPI_MODE_SPIMODE                                   DSPI_MODE_SPIMODE_Msk

#define DSPI_MODE_DCX_Pos                                   (2U)
#define DSPI_MODE_DCX_Len                                   (1)
#define DSPI_MODE_DCX_Msk                                   (0x1UL << DSPI_MODE_DCX_Pos)
#define DSPI_MODE_DCX                                       DSPI_MODE_DCX_Msk

/* =================================================================================================================== */
/* ================                                  PDM                                 ================ */
/* =================================================================================================================== */
/*******************  Bit definition for PDM_EN_L register  *******************/
#define PDM_EN_L_EN_RX_POS                                  (0U)
#define PDM_EN_L_EN_RX_Len                                  (1U)
#define PDM_EN_L_EN_RX_Msk                                  (0x1UL << PDM_EN_L_EN_RX_POS)
#define PDM_EN_L_EN_RX                                      PDM_EN_L_EN_RX_Msk
#define PDM_EN_L_EN_RX_ENABLE                               (0x1U << PDM_EN_L_EN_RX_POS)
#define PDM_EN_L_EN_RX_DISABLE                              (0x0U << PDM_EN_L_EN_RX_POS)

#define PDM_EN_L_SMP_DMIC_POS                               (1U)
#define PDM_EN_L_SMP_DMIC_Len                               (1U)
#define PDM_EN_L_SMP_DMIC_Msk                               (0x1UL << PDM_EN_L_SMP_DMIC_POS)
#define PDM_EN_L_SMP_DMIC                                   PDM_EN_L_SMP_DMIC_Msk
#define PDM_EN_L_SMP_DMIC_ENABLE                            (0x1U << PDM_EN_L_SMP_DMIC_POS)
#define PDM_EN_L_SMP_DMIC_DISABLE                           (0x0U << PDM_EN_L_SMP_DMIC_POS)

#define PDM_EN_L_EN_STAGE0_POS                              (2U)
#define PDM_EN_L_EN_STAGE0_Len                              (1U)
#define PDM_EN_L_EN_STAGE0_Msk                              (0x1UL << PDM_EN_L_EN_STAGE0_POS)
#define PDM_EN_L_EN_STAGE0                                  PDM_EN_L_EN_STAGE0_Msk
#define PDM_EN_L_EN_STAGE0_ENABLE                           (0x1U << PDM_EN_L_EN_STAGE0_POS)
#define PDM_EN_L_EN_STAGE0_DISABLE                          (0x0U << PDM_EN_L_EN_STAGE0_POS)

#define PDM_EN_L_EN_STAGE1_POS                              (3U)
#define PDM_EN_L_EN_STAGE1_Len                              (1U)
#define PDM_EN_L_EN_STAGE1_Msk                              (0x1UL << PDM_EN_L_EN_STAGE1_POS)
#define PDM_EN_L_EN_STAGE1                                  PDM_EN_L_EN_STAGE1_Msk
#define PDM_EN_L_EN_STAGE1_ENABLE                           (0x1U << PDM_EN_L_EN_STAGE1_POS)
#define PDM_EN_L_EN_STAGE1_DISABLE                          (0x0U << PDM_EN_L_EN_STAGE1_POS)

#define PDM_EN_L_EN_STAGE2_POS                              (4U)
#define PDM_EN_L_EN_STAGE2_Len                              (1U)
#define PDM_EN_L_EN_STAGE2_Msk                              (0x1UL << PDM_EN_L_EN_STAGE2_POS)
#define PDM_EN_L_EN_STAGE2                                  PDM_EN_L_EN_STAGE2_Msk
#define PDM_EN_L_EN_STAGE2_ENABLE                           (0x1U << PDM_EN_L_EN_STAGE2_POS)
#define PDM_EN_L_EN_STAGE2_DISABLE                          (0x0U << PDM_EN_L_EN_STAGE2_POS)

#define PDM_EN_L_EN_STAGE3_POS                              (5U)
#define PDM_EN_L_EN_STAGE3_Len                              (1U)
#define PDM_EN_L_EN_STAGE3_Msk                              (0x1UL << PDM_EN_L_EN_STAGE3_POS)
#define PDM_EN_L_EN_STAGE3                                  PDM_EN_L_EN_STAGE3_Msk
#define PDM_EN_L_EN_STAGE3_ENABLE                           (0x1U << PDM_EN_L_EN_STAGE3_POS)
#define PDM_EN_L_EN_STAGE3_DISABLE                          (0x0U << PDM_EN_L_EN_STAGE3_POS)

#define PDM_EN_L_EN_STAGE4_POS                              (6U)
#define PDM_EN_L_EN_STAGE4_Len                              (1U)
#define PDM_EN_L_EN_STAGE4_Msk                              (0x1UL << PDM_EN_L_EN_STAGE4_POS)
#define PDM_EN_L_EN_STAGE4                                  PDM_EN_L_EN_STAGE4_Msk
#define PDM_EN_L_EN_STAGE4_ENABLE                           (0x1U << PDM_EN_L_EN_STAGE4_POS)
#define PDM_EN_L_EN_STAGE4_DISABLE                          (0x0U << PDM_EN_L_EN_STAGE4_POS)

#define PDM_EN_L_EN_STAGE5_POS                              (7U)
#define PDM_EN_L_EN_STAGE5_Len                              (1U)
#define PDM_EN_L_EN_STAGE5_Msk                              (0x1UL << PDM_EN_L_EN_STAGE5_POS)
#define PDM_EN_L_EN_STAGE5                                  PDM_EN_L_EN_STAGE5_Msk
#define PDM_EN_L_EN_STAGE5_ENABLE                           (0x1U << PDM_EN_L_EN_STAGE5_POS)
#define PDM_EN_L_EN_STAGE5_DISABLE                          (0x0U << PDM_EN_L_EN_STAGE5_POS)

#define PDM_EN_L_EN_STAGE6_POS                              (8U)
#define PDM_EN_L_EN_STAGE6_Len                              (1U)
#define PDM_EN_L_EN_STAGE6_Msk                              (0x1UL << PDM_EN_L_EN_STAGE6_POS)
#define PDM_EN_L_EN_STAGE6                                  PDM_EN_L_EN_STAGE6_Msk
#define PDM_EN_L_EN_STAGE6_ENABLE                           (0x1U << PDM_EN_L_EN_STAGE6_POS)
#define PDM_EN_L_EN_STAGE6_DISABLE                          (0x0U << PDM_EN_L_EN_STAGE6_POS)

#define PDM_EN_L_EN_STAGE7_POS                              (9U)
#define PDM_EN_L_EN_STAGE7_Len                              (1U)
#define PDM_EN_L_EN_STAGE7_Msk                              (0x1UL << PDM_EN_L_EN_STAGE7_POS)
#define PDM_EN_L_EN_STAGE7                                  PDM_EN_L_EN_STAGE7_Msk
#define PDM_EN_L_EN_STAGE7_ENABLE                           (0x1U << PDM_EN_L_EN_STAGE7_POS)
#define PDM_EN_L_EN_STAGE7_DISABLE                          (0x0U << PDM_EN_L_EN_STAGE7_POS)

#define PDM_EN_L_EN_HPF_POS                                 (10U)
#define PDM_EN_L_EN_HPF_Len                                 (1U)
#define PDM_EN_L_EN_HPF_Msk                                 (0x1UL << PDM_EN_L_EN_HPF_POS)
#define PDM_EN_L_EN_HPF                                     PDM_EN_L_EN_HPF_Msk
#define PDM_EN_L_EN_HPF_ENABLE                              (0x1U << PDM_EN_L_EN_HPF_POS)
#define PDM_EN_L_EN_HPF_DISABLE                             (0x0U << PDM_EN_L_EN_HPF_POS)

/*******************  Bit definition for PDM_IN_CFG_L register  *******************/
#define PDM_IN_CFG_L_RX_UPSMP_POS                           (0U)
#define PDM_IN_CFG_L_RX_UPSMP_Len                           (1U)
#define PDM_IN_CFG_L_RX_UPSMP_Msk                           (0x1UL << PDM_IN_CFG_L_RX_UPSMP_POS)
#define PDM_IN_CFG_L_RX_UPSMP                               PDM_IN_CFG_L_RX_UPSMP_Msk

#define PDM_IN_CFG_L_STAGE_INIT_POS                         (1U)
#define PDM_IN_CFG_L_STAGE_INIT_Len                         (2U)
#define PDM_IN_CFG_L_STAGE_INIT_Msk                         (0x3UL << PDM_IN_CFG_L_STAGE_INIT_POS)
#define PDM_IN_CFG_L_STAGE_INIT                             PDM_IN_CFG_L_STAGE_INIT_Msk

/*******************  Bit definition for PDM_LPF_CFG_L register  *******************/
#define PDM_LPF_CFG_L_UPSMP_FACTOR_POS                      (0U)
#define PDM_LPF_CFG_L_UPSMP_FACTOR_Len                      (2U)
#define PDM_LPF_CFG_L_UPSMP_FACTOR_Msk                      (0x3UL << PDM_LPF_CFG_L_UPSMP_FACTOR_POS)
#define PDM_LPF_CFG_L_UPSMP_FACTOR                          PDM_LPF_CFG_L_UPSMP_FACTOR_Msk

/*******************  Bit definition for PDM_HPF_CFG_L register  *******************/
#define PDM_HPF_CFG_L_BYPASS_POS                            (0U)
#define PDM_HPF_CFG_L_BYPASS_Len                            (1U)
#define PDM_HPF_CFG_L_BYPASS_Msk                            (0x1UL << PDM_HPF_CFG_L_BYPASS_POS)
#define PDM_HPF_CFG_L_BYPASS                                PDM_HPF_CFG_L_BYPASS_Msk
#define PDM_HPF_CFG_L_BYPASS_ENABLE                         (0x1U << PDM_HPF_CFG_L_BYPASS_POS)
#define PDM_HPF_CFG_L_BYPASS_DISABLE                        (0x0U << PDM_HPF_CFG_L_BYPASS_POS)

#define PDM_HPF_CFG_L_CORNER_POS                            (1U)
#define PDM_HPF_CFG_L_CORNER_Len                            (2U)
#define PDM_HPF_CFG_L_CORNER_Msk                            (0x3UL << PDM_HPF_CFG_L_CORNER_POS)
#define PDM_HPF_CFG_L_CORNER                                PDM_HPF_CFG_L_CORNER_Msk
#define PDM_HPF_CFG_L_CORNER_0_25                           (0x0U << PDM_HPF_CFG_L_CORNER_POS)
#define PDM_HPF_CFG_L_CORNER_1                              (0x1U << PDM_HPF_CFG_L_CORNER_POS)
#define PDM_HPF_CFG_L_CORNER_4                              (0x2U << PDM_HPF_CFG_L_CORNER_POS)
#define PDM_HPF_CFG_L_CORNER_16                             (0x3U << PDM_HPF_CFG_L_CORNER_POS)

#define PDM_HPF_CFG_L_FREEZE_EN_POS                         (3U)
#define PDM_HPF_CFG_L_FREEZE_EN_Len                         (1U)
#define PDM_HPF_CFG_L_FREEZE_EN_Msk                         (0x1UL << PDM_HPF_CFG_L_FREEZE_EN_POS)
#define PDM_HPF_CFG_L_FREEZE_EN                             PDM_HPF_CFG_L_FREEZE_EN_Msk
#define PDM_HPF_CFG_L_FREEZE_ENABLE                         (0x1U << PDM_HPF_CFG_L_FREEZE_EN_POS)
#define PDM_HPF_CFG_L_FREEZE_DISABLE                        (0x0U << PDM_HPF_CFG_L_FREEZE_EN_POS)

/*******************  Bit definition for PDM_PGA_CFG_L register  *******************/
#define PDM_PGA_CFG_L_VAL_POS                               (0U)
#define PDM_PGA_CFG_L_VAL_Len                               (14U)
#define PDM_PGA_CFG_L_VAL_Msk                               (0x3FFFUL << PDM_PGA_CFG_L_VAL_POS)
#define PDM_PGA_CFG_L_VAL                                   PDM_PGA_CFG_L_VAL_Msk

/*******************  Bit definition for PDM_DATA_L register  *******************/
#define PDM_DATA_L_DATA_POS                                 (0U)
#define PDM_DATA_L_DATA_Len                                 (16U)
#define PDM_DATA_L_DATA_Msk                                 (0xFFFFUL << PDM_DATA_L_DATA_POS)
#define PDM_DATA_L_DATA                                     PDM_DATA_L_DATA_Msk

#define PDM_DATA_L_VALID_POS                                (16U)
#define PDM_DATA_L_VALID_Len                                (1U)
#define PDM_DATA_L_VALID_Msk                                (0x1UL << PDM_DATA_L_VALID_POS)
#define PDM_DATA_L_VALID                                    PDM_DATA_L_VALID_Msk

#define PDM_DATA_L_OVER_POS                                 (17U)
#define PDM_DATA_L_OVER_Len                                 (1U)
#define PDM_DATA_L_OVER_Msk                                 (0x1UL << PDM_DATA_L_OVER_POS)
#define PDM_DATA_L_OVER                                     PDM_DATA_L_OVER_Msk

#define PDM_DATA_L_VALID_DMA_MASK_POS                       (18U)
#define PDM_DATA_L_VALID_DMA_MASK_Len                       (1U)
#define PDM_DATA_L_VALID_DMA_MASK_Msk                       (0x1UL << PDM_DATA_L_VALID_DMA_MASK_POS)
#define PDM_DATA_L_VALID_DMA_MASK                           PDM_DATA_L_VALID_DMA_MASK_Msk
#define PDM_DATA_L_VALID_DMA_MASK_ENABLE                    (0x1UL << PDM_DATA_L_VALID_DMA_MASK_POS)
#define PDM_DATA_L_VALID_DMA_MASK_DISABLE                   (0x0UL << PDM_DATA_L_VALID_DMA_MASK_POS)

/*******************  Bit definition for PDM_INT_L register  *******************/
#define PDM_INT_L_VALID_MASK_POS                            (0U)
#define PDM_INT_L_VALID_MASK_Len                            (1U)
#define PDM_INT_L_VALID_MASK_Msk                            (0x1UL << PDM_INT_L_VALID_MASK_POS)
#define PDM_INT_L_VALID_MASK                                PDM_INT_L_VALID_MASK_Msk

#define PDM_INT_L_OVER_MASK_POS                             (1U)
#define PDM_INT_L_OVER_MASK_Len                             (1U)
#define PDM_INT_L_OVER_MASK_Msk                             (0x1UL << PDM_INT_L_OVER_MASK_POS)
#define PDM_INT_L_OVER_MASK                                 PDM_INT_L_OVER_MASK_Msk

/*******************  Bit definition for PDM_EN_R register  *******************/
#define PDM_EN_R_EN_RX_POS                                  (0U)
#define PDM_EN_R_EN_RX_Len                                  (1U)
#define PDM_EN_R_EN_RX_Msk                                  (0x1UL << PDM_EN_R_EN_RX_POS)
#define PDM_EN_R_EN_RX                                      PDM_EN_R_EN_RX_Msk
#define PDM_EN_R_EN_RX_ENABLE                               (0x1U << PDM_EN_R_EN_RX_POS)
#define PDM_EN_R_EN_RX_DISABLE                              (0x0U << PDM_EN_R_EN_RX_POS)

#define PDM_EN_R_SMP_DMIC_POS                               (1U)
#define PDM_EN_R_SMP_DMIC_Len                               (1U)
#define PDM_EN_R_SMP_DMIC_Msk                               (0x1UL << PDM_EN_R_SMP_DMIC_POS)
#define PDM_EN_R_SMP_DMIC                                   PDM_EN_R_SMP_DMIC_Msk
#define PDM_EN_R_SMP_DMIC_ENABLE                            (0x1U << PDM_EN_R_SMP_DMIC_POS)
#define PDM_EN_R_SMP_DMIC_DISABLE                           (0x0U << PDM_EN_R_SMP_DMIC_POS)

#define PDM_EN_R_EN_STAGE0_POS                              (2U)
#define PDM_EN_R_EN_STAGE0_Len                              (1U)
#define PDM_EN_R_EN_STAGE0_Msk                              (0x1UL << PDM_EN_R_EN_STAGE0_POS)
#define PDM_EN_R_EN_STAGE0                                  PDM_EN_R_EN_STAGE0_Msk
#define PDM_EN_R_EN_STAGE0_ENABLE                           (0x1U << PDM_EN_R_EN_STAGE0_POS)
#define PDM_EN_R_EN_STAGE0_DISABLE                          (0x0U << PDM_EN_R_EN_STAGE0_POS)

#define PDM_EN_R_EN_STAGE1_POS                              (3U)
#define PDM_EN_R_EN_STAGE1_Len                              (1U)
#define PDM_EN_R_EN_STAGE1_Msk                              (0x1UL << PDM_EN_R_EN_STAGE1_POS)
#define PDM_EN_R_EN_STAGE1                                  PDM_EN_R_EN_STAGE1_Msk
#define PDM_EN_R_EN_STAGE1_ENABLE                           (0x1U << PDM_EN_R_EN_STAGE1_POS)
#define PDM_EN_R_EN_STAGE1_DISABLE                          (0x0U << PDM_EN_R_EN_STAGE1_POS)

#define PDM_EN_R_EN_STAGE2_POS                              (4U)
#define PDM_EN_R_EN_STAGE2_Len                              (1U)
#define PDM_EN_R_EN_STAGE2_Msk                              (0x1UL << PDM_EN_R_EN_STAGE2_POS)
#define PDM_EN_R_EN_STAGE2                                  PDM_EN_R_EN_STAGE2_Msk
#define PDM_EN_R_EN_STAGE2_ENABLE                           (0x1U << PDM_EN_R_EN_STAGE2_POS)
#define PDM_EN_R_EN_STAGE2_DISABLE                          (0x0U << PDM_EN_R_EN_STAGE2_POS)

#define PDM_EN_R_EN_STAGE3_POS                              (5U)
#define PDM_EN_R_EN_STAGE3_Len                              (1U)
#define PDM_EN_R_EN_STAGE3_Msk                              (0x1UL << PDM_EN_R_EN_STAGE3_POS)
#define PDM_EN_R_EN_STAGE3                                  PDM_EN_R_EN_STAGE3_Msk
#define PDM_EN_R_EN_STAGE3_ENABLE                           (0x1U << PDM_EN_R_EN_STAGE3_POS)
#define PDM_EN_R_EN_STAGE3_DISABLE                          (0x0U << PDM_EN_R_EN_STAGE3_POS)

#define PDM_EN_R_EN_STAGE4_POS                              (6U)
#define PDM_EN_R_EN_STAGE4_Len                              (1U)
#define PDM_EN_R_EN_STAGE4_Msk                              (0x1UL << PDM_EN_R_EN_STAGE4_POS)
#define PDM_EN_R_EN_STAGE4                                  PDM_EN_R_EN_STAGE4_Msk
#define PDM_EN_R_EN_STAGE4_ENABLE                           (0x1U << PDM_EN_R_EN_STAGE4_POS)
#define PDM_EN_R_EN_STAGE4_DISABLE                          (0x0U << PDM_EN_R_EN_STAGE4_POS)

#define PDM_EN_R_EN_STAGE5_POS                              (7U)
#define PDM_EN_R_EN_STAGE5_Len                              (1U)
#define PDM_EN_R_EN_STAGE5_Msk                              (0x1UL << PDM_EN_R_EN_STAGE5_POS)
#define PDM_EN_R_EN_STAGE5                                  PDM_EN_R_EN_STAGE5_Msk
#define PDM_EN_R_EN_STAGE5_ENABLE                           (0x1U << PDM_EN_R_EN_STAGE5_POS)
#define PDM_EN_R_EN_STAGE5_DISABLE                          (0x0U << PDM_EN_R_EN_STAGE5_POS)

#define PDM_EN_R_EN_STAGE6_POS                              (8U)
#define PDM_EN_R_EN_STAGE6_Len                              (1U)
#define PDM_EN_R_EN_STAGE6_Msk                              (0x1UL << PDM_EN_R_EN_STAGE6_POS)
#define PDM_EN_R_EN_STAGE6                                  PDM_EN_R_EN_STAGE6_Msk
#define PDM_EN_R_EN_STAGE6_ENABLE                           (0x1U << PDM_EN_R_EN_STAGE6_POS)
#define PDM_EN_R_EN_STAGE6_DISABLE                          (0x0U << PDM_EN_R_EN_STAGE6_POS)

#define PDM_EN_R_EN_STAGE7_POS                              (9U)
#define PDM_EN_R_EN_STAGE7_Len                              (1U)
#define PDM_EN_R_EN_STAGE7_Msk                              (0x1UL << PDM_EN_R_EN_STAGE7_POS)
#define PDM_EN_R_EN_STAGE7                                  PDM_EN_R_EN_STAGE7_Msk
#define PDM_EN_R_EN_STAGE7_ENABLE                           (0x1U << PDM_EN_R_EN_STAGE7_POS)
#define PDM_EN_R_EN_STAGE7_DISABLE                          (0x0U << PDM_EN_R_EN_STAGE7_POS)

#define PDM_EN_R_EN_HPF_POS                                 (10U)
#define PDM_EN_R_EN_HPF_Len                                 (1U)
#define PDM_EN_R_EN_HPF_Msk                                 (0x1UL << PDM_EN_R_EN_HPF_POS)
#define PDM_EN_R_EN_HPF                                     PDM_EN_R_EN_HPF_Msk
#define PDM_EN_R_EN_HPF_ENABLE                              (0x1U << PDM_EN_R_EN_HPF_POS)
#define PDM_EN_R_EN_HPF_DISABLE                             (0x0U << PDM_EN_R_EN_HPF_POS)

/*******************  Bit definition for PDM_IN_CFG_R register  *******************/
#define PDM_IN_CFG_R_RX_UPSMP_POS                           (0U)
#define PDM_IN_CFG_R_RX_UPSMP_Len                           (1U)
#define PDM_IN_CFG_R_RX_UPSMP_Msk                           (0x1UL << PDM_IN_CFG_R_RX_UPSMP_POS)
#define PDM_IN_CFG_R_RX_UPSMP                               PDM_IN_CFG_R_RX_UPSMP_Msk

#define PDM_IN_CFG_R_STAGE_INIT_POS                         (1U)
#define PDM_IN_CFG_R_STAGE_INIT_Len                         (2U)
#define PDM_IN_CFG_R_STAGE_INIT_Msk                         (0x3UL << PDM_IN_CFG_R_STAGE_INIT_POS)
#define PDM_IN_CFG_R_STAGE_INIT                             PDM_IN_CFG_R_STAGE_INIT_Msk

/*******************  Bit definition for PDM_LPF_CFG_R register  *******************/
#define PDM_LPF_CFG_R_UPSMP_FACTOR_POS                      (0U)
#define PDM_LPF_CFG_R_UPSMP_FACTOR_Len                      (2U)
#define PDM_LPF_CFG_R_UPSMP_FACTOR_Msk                      (0x3UL << PDM_LPF_CFG_R_UPSMP_FACTOR_POS)
#define PDM_LPF_CFG_R_UPSMP_FACTOR                          PDM_LPF_CFG_R_UPSMP_FACTOR_Msk

/*******************  Bit definition for PDM_HPF_CFG_R register  *******************/
#define PDM_HPF_CFG_R_BYPASS_POS                            (0U)
#define PDM_HPF_CFG_R_BYPASS_Len                            (1U)
#define PDM_HPF_CFG_R_BYPASS_Msk                            (0x1UL << PDM_HPF_CFG_R_BYPASS_POS)
#define PDM_HPF_CFG_R_BYPASS                                PDM_HPF_CFG_R_BYPASS_Msk
#define PDM_HPF_CFG_R_BYPASS_ENABLE                         (0x1U << PDM_HPF_CFG_R_BYPASS_POS)
#define PDM_HPF_CFG_R_BYPASS_DISABLE                        (0x0U << PDM_HPF_CFG_R_BYPASS_POS)

#define PDM_HPF_CFG_R_CORNER_POS                            (1U)
#define PDM_HPF_CFG_R_CORNER_Len                            (2U)
#define PDM_HPF_CFG_R_CORNER_Msk                            (0x3UL << PDM_HPF_CFG_R_CORNER_POS)
#define PDM_HPF_CFG_R_CORNER                                PDM_HPF_CFG_R_CORNER_Msk
#define PDM_HPF_CFG_R_CORNER_0_25                           (0x0U << PDM_HPF_CFG_R_CORNER_POS)
#define PDM_HPF_CFG_R_CORNER_1                              (0x1U << PDM_HPF_CFG_R_CORNER_POS)
#define PDM_HPF_CFG_R_CORNER_4                              (0x2U << PDM_HPF_CFG_R_CORNER_POS)
#define PDM_HPF_CFG_R_CORNER_16                             (0x3U << PDM_HPF_CFG_R_CORNER_POS)

#define PDM_HPF_CFG_R_FREEZE_EN_POS                         (3U)
#define PDM_HPF_CFG_R_FREEZE_EN_Len                         (1U)
#define PDM_HPF_CFG_R_FREEZE_EN_Msk                         (0x1UL << PDM_HPF_CFG_R_FREEZE_EN_POS)
#define PDM_HPF_CFG_R_FREEZE_EN                             PDM_HPF_CFG_R_FREEZE_EN_Msk
#define PDM_HPF_CFG_R_FREEZE_ENABLE                         (0x1U << PDM_HPF_CFG_R_FREEZE_EN_POS)
#define PDM_HPF_CFG_R_FREEZE_DISABLE                        (0x0U << PDM_HPF_CFG_R_FREEZE_EN_POS)

/*******************  Bit definition for PDM_PGA_CFG_R register  *******************/
#define PDM_PGA_CFG_R_VAL_POS                               (0U)
#define PDM_PGA_CFG_R_VAL_Len                               (14U)
#define PDM_PGA_CFG_R_VAL_Msk                               (0x3FFFUL << PDM_PGA_CFG_R_VAL_POS)
#define PDM_PGA_CFG_R_VAL                                   PDM_PGA_CFG_R_VAL_Msk

/*******************  Bit definition for PDM_DATA_R register  *******************/
#define PDM_DATA_R_DATA_POS                                 (0U)
#define PDM_DATA_R_DATA_Len                                 (16U)
#define PDM_DATA_R_DATA_Msk                                 (0xFFFFUL << PDM_DATA_R_DATA_POS)
#define PDM_DATA_R_DATA                                     PDM_DATA_R_DATA_Msk

#define PDM_DATA_R_VALID_POS                                (16U)
#define PDM_DATA_R_VALID_Len                                (1U)
#define PDM_DATA_R_VALID_Msk                                (0x1UL << PDM_DATA_R_VALID_POS)
#define PDM_DATA_R_VALID                                    PDM_DATA_R_VALID_Msk

#define PDM_DATA_R_OVER_POS                                 (17U)
#define PDM_DATA_R_OVER_Len                                 (1U)
#define PDM_DATA_R_OVER_Msk                                 (0x1UL << PDM_DATA_R_OVER_POS)
#define PDM_DATA_R_OVER                                     PDM_DATA_R_OVER_Msk

#define PDM_DATA_R_VALID_DMA_MASK_POS                       (18U)
#define PDM_DATA_R_VALID_DMA_MASK_Len                       (1U)
#define PDM_DATA_R_VALID_DMA_MASK_Msk                       (0x1UL << PDM_DATA_R_VALID_DMA_MASK_POS)
#define PDM_DATA_R_VALID_DMA_MASK                           PDM_DATA_R_VALID_DMA_MASK_Msk
#define PDM_DATA_R_VALID_DMA_MASK_ENABLE                    (0x1UL << PDM_DATA_R_VALID_DMA_MASK_POS)
#define PDM_DATA_R_VALID_DMA_MASK_DISABLE                   (0x0UL << PDM_DATA_R_VALID_DMA_MASK_POS)

/*******************  Bit definition for PDM_INT_R register  *******************/
#define PDM_INT_R_VALID_MASK_POS                            (0U)
#define PDM_INT_R_VALID_MASK_Len                            (1U)
#define PDM_INT_R_VALID_MASK_Msk                            (0x1UL << PDM_INT_R_VALID_MASK_POS)
#define PDM_INT_R_VALID_MASK                                PDM_INT_R_VALID_MASK_Msk

#define PDM_INT_R_OVER_MASK_POS                             (1U)
#define PDM_INT_R_OVER_MASK_Len                             (1U)
#define PDM_INT_R_OVER_MASK_Msk                             (0x1UL << PDM_INT_R_OVER_MASK_POS)
#define PDM_INT_R_OVER_MASK                                 PDM_INT_R_OVER_MASK_Msk


/*******************  Bit definition for PDM_DATA register  *******************/
#define PDM_DATA_DATA_L_POS                                 (0U)
#define PDM_DATA_DATA_L_Len                                 (16U)
#define PDM_DATA_DATA_L_Msk                                 (0xFFFFUL) << PDM_DATA_DATA_L_POS)
#define PDM_DATA_DATA_L                                     PDM_DATA_DATA_L_Msk

#define PDM_DATA_DATA_R_POS                                 (16U)
#define PDM_DATA_DATA_R_Len                                 (16U)
#define PDM_DATA_DATA_R_Msk                                 (0xFFFFUL) << PDM_DATA_DATA_R_POS)
#define PDM_DATA_DATA_R                                     PDM_DATA_DATA_R_Msk

/*******************  Bit definition for PDM_CLK_DIV register  *******************/
#define PDM_CLK_DIV_POSEDGE_EN_PULSE_CFG_POS                (0U)
#define PDM_CLK_DIV_POSEDGE_EN_PULSE_CFG_Len                (4U)
#define PDM_CLK_DIV_POSEDGE_EN_PULSE_CFG_Msk                ((0xFUL) << PDM_CLK_DIV_POSEDGE_EN_PULSE_CFG_POS)
#define PDM_CLK_DIV_POSEDGE_EN_PULSE_CFG                    PDM_CLK_DIV_POSEDGE_EN_PULSE_CFG_Msk

#define PDM_CLK_DIV_NEGEDGE_EN_PULSE_CFG_POS                (8U)
#define PDM_CLK_DIV_NEGEDGE_EN_PULSE_CFG_Len                (4U)
#define PDM_CLK_DIV_NEGEDGE_EN_PULSE_CFG_Msk                ((0xFUL) << PDM_CLK_DIV_NEGEDGE_EN_PULSE_CFG_POS)
#define PDM_CLK_DIV_NEGEDGE_EN_PULSE_CFG                    PDM_CLK_DIV_NEGEDGE_EN_PULSE_CFG_Msk


/*******************  Bit definition for PDM_CLK register  *******************/
#define PDM_CLK_EN_POS                                      (0U)
#define PDM_CLK_EN_Len                                      (1U)
#define PDM_CLK_EN_Msk                                      (0x1UL << PDM_CLK_EN_POS)
#define PDM_CLK_EN                                          PDM_CLK_EN_Msk
#define PDM_CLK_EN_ENABLE                                   (0x1UL << PDM_CLK_EN_POS)
#define PDM_CLK_EN_DISABLE                                  (0x0UL << PDM_CLK_EN_POS)

#define PDM_CLK_SAMPLE_RATE_POS                             (8U)
#define PDM_CLK_SAMPLE_RATE_Len                             (2U)
#define PDM_CLK_SAMPLE_RATE_Msk                             (0x3UL << PDM_CLK_SAMPLE_RATE_POS)
#define PDM_CLK_SAMPLE_RATE                                 PDM_CLK_SAMPLE_RATE_Msk
#define PDM_CLK_SAMPLE_RATE_15_625K                         (0x0UL << PDM_CLK_SAMPLE_RATE_POS)
#define PDM_CLK_SAMPLE_RATE_16K                             (0x1UL << PDM_CLK_SAMPLE_RATE_POS)
#define PDM_CLK_SAMPLE_RATE_8K                              (0x3UL << PDM_CLK_SAMPLE_RATE_POS)

/* ================================================================================================================= */
/* ================                                        GPADC                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for GPADC_CTRL0 register  *******************/
#define GPADC_CTRL0_M_POS                                   (0U)
#define GPADC_CTRL0_M_Len                                   (4U)
#define GPADC_CTRL0_M_Msk                                   ((0xFUL) << GPADC_CTRL0_M_POS)
#define GPADC_CTRL0_M                                       GPADC_CTRL0_M_Msk

#define GPADC_CTRL0_P_POS                                   (4U)
#define GPADC_CTRL0_P_Len                                   (2U)
#define GPADC_CTRL0_P_Msk                                   ((0x3UL) << GPADC_CTRL0_P_POS)
#define GPADC_CTRL0_P                                       GPADC_CTRL0_P_Msk

#define GPADC_CTRL0_N_POS                                   (6U)
#define GPADC_CTRL0_N_Len                                   (6U)
#define GPADC_CTRL0_N_Msk                                   ((0x3FUL) << GPADC_CTRL0_N_POS)
#define GPADC_CTRL0_N                                       GPADC_CTRL0_N_Msk

#define GPADC_CTRL0_CONV_NUM_POS                            (12U)
#define GPADC_CTRL0_CONV_NUM_Len                            (4U)
#define GPADC_CTRL0_CONV_NUM_Msk                            ((0xFUL) << GPADC_CTRL0_CONV_NUM_POS)
#define GPADC_CTRL0_CONV_NUM                                GPADC_CTRL0_CONV_NUM_Msk

#define GPADC_CTRL0_TD_T_POS                                (16U)
#define GPADC_CTRL0_TD_T_Len                                (8U)
#define GPADC_CTRL0_TD_T_Msk                                ((0xFFUL) << GPADC_CTRL0_TD_T_POS)
#define GPADC_CTRL0_TD_T                                    GPADC_CTRL0_TD_T_Msk

#define GPADC_CTRL0_TS_DIV_POS                              (24U)
#define GPADC_CTRL0_TS_DIV_Len                              (3U)
#define GPADC_CTRL0_TS_DIV_Msk                              ((0x7UL) << GPADC_CTRL0_TS_DIV_POS)
#define GPADC_CTRL0_TS_DIV                                  GPADC_CTRL0_TS_DIV_Msk

#define GPADC_CTRL0_SAMPLE_MODE_POS                         (27U)
#define GPADC_CTRL0_SAMPLE_MODE_Len                         (1U)
#define GPADC_CTRL0_SAMPLE_MODE_Msk                         ((0x1UL) << GPADC_CTRL0_SAMPLE_MODE_POS)
#define GPADC_CTRL0_SAMPLE_MODE                             GPADC_CTRL0_SAMPLE_MODE_Msk

#define GPADC_CTRL0_DAC_CLK_INV_POS                         (28U)
#define GPADC_CTRL0_DAC_CLK_INV_Len                         (1U)
#define GPADC_CTRL0_DAC_CLK_INV_Msk                         ((0x1UL) << GPADC_CTRL0_DAC_CLK_INV_POS)
#define GPADC_CTRL0_DAC_CLK_INV                             GPADC_CTRL0_DAC_CLK_INV_Msk

#define GPADC_CTRL0_CALI_CAP_POS                            (29U)
#define GPADC_CTRL0_CALI_CAP_Len                            (1U)
#define GPADC_CTRL0_CALI_CAP_Msk                            ((0x1UL) << GPADC_CTRL0_CALI_CAP_POS)
#define GPADC_CTRL0_CALI_CAP                                GPADC_CTRL0_CALI_CAP_Msk

#define GPADC_CTRL0_ADC_EN_POS                              (30U)
#define GPADC_CTRL0_ADC_EN_Len                              (1U)
#define GPADC_CTRL0_ADC_EN_Msk                              ((0x1UL) << GPADC_CTRL0_ADC_EN_POS)
#define GPADC_CTRL0_ADC_EN                                  GPADC_CTRL0_ADC_EN_Msk

#define GPADC_CTRL0_CONV_SEL_POS                            (31U)
#define GPADC_CTRL0_CONV_SEL_Len                            (1U)
#define GPADC_CTRL0_CONV_SEL_Msk                            ((0x1UL) << GPADC_CTRL0_CONV_SEL_POS)
#define GPADC_CTRL0_CONV_SEL                                GPADC_CTRL0_CONV_SEL_Msk


/*******************  Bit definition for GPADC_CTRL1 register  *******************/
#define GPADC_CTRL1_FACTOR_POS                              (0U)
#define GPADC_CTRL1_FACTOR_Len                              (16U)
#define GPADC_CTRL1_FACTOR_Msk                              ((0xFFFFUL) << GPADC_CTRL1_FACTOR_POS)
#define GPADC_CTRL1_FACTOR                                  GPADC_CTRL1_FACTOR_Msk

#define GPADC_CTRL1_OBS_EN_POS                              (16U)
#define GPADC_CTRL1_OBS_EN_Len                              (1U)
#define GPADC_CTRL1_OBS_EN_Msk                              ((0x1UL) << GPADC_CTRL1_OBS_EN_POS)
#define GPADC_CTRL1_OBS_EN                                  GPADC_CTRL1_OBS_EN_Msk

#define GPADC_CTRL1_ADJUST_SP_EN_POS                        (17U)
#define GPADC_CTRL1_ADJUST_SP_EN_Len                        (1U)
#define GPADC_CTRL1_ADJUST_SP_EN_Msk                        ((0x1UL) << GPADC_CTRL1_ADJUST_SP_EN_POS)
#define GPADC_CTRL1_ADJUST_SP_EN                            GPADC_CTRL1_ADJUST_SP_EN_Msk

#define GPADC_CTRL1_ADJUST_SP_SEL_POS                       (18U)
#define GPADC_CTRL1_ADJUST_SP_SEL_Len                       (2U)
#define GPADC_CTRL1_ADJUST_SP_SEL_Msk                       ((0x3UL) << GPADC_CTRL1_ADJUST_SP_SEL_POS)
#define GPADC_CTRL1_ADJUST_SP_SEL                           GPADC_CTRL1_ADJUST_SP_SEL_Msk


/*******************  Bit definition for GPADC_COEF0 register  *******************/
#define GPADC_COEF0_COE0_POS                                (0U)
#define GPADC_COEF0_COE0_Len                                (20U)
#define GPADC_COEF0_COE0_Msk                                ((0xFFFFFUL) << GPADC_COEF0_COE0_POS)
#define GPADC_COEF0_COE0                                    GPADC_COEF0_COE0_Msk


/*******************  Bit definition for GPADC_COEF1 register  *******************/
#define GPADC_COEF1_COE1_POS                                (0U)
#define GPADC_COEF1_COE1_Len                                (20U)
#define GPADC_COEF1_COE1_Msk                                ((0xFFFFFUL) << GPADC_COEF1_COE1_POS)
#define GPADC_COEF1_COE1                                    GPADC_COEF1_COE1_Msk


/*******************  Bit definition for GPADC_COEF2 register  *******************/
#define GPADC_COEF2_COE2_POS                                (0U)
#define GPADC_COEF2_COE2_Len                                (20U)
#define GPADC_COEF2_COE2_Msk                                ((0xFFFFFUL) << GPADC_COEF2_COE2_POS)
#define GPADC_COEF2_COE2                                    GPADC_COEF2_COE2_Msk


/*******************  Bit definition for GPADC_COEF3 register  *******************/
#define GPADC_COEF3_COE3_POS                                (0U)
#define GPADC_COEF3_COE3_Len                                (20U)
#define GPADC_COEF3_COE3_Msk                                ((0xFFFFFUL) << GPADC_COEF3_COE3_POS)
#define GPADC_COEF3_COE3                                    GPADC_COEF3_COE3_Msk


/*******************  Bit definition for GPADC_COEF4 register  *******************/
#define GPADC_COEF4_COE4_POS                                (0U)
#define GPADC_COEF4_COE4_Len                                (20U)
#define GPADC_COEF4_COE4_Msk                                ((0xFFFFFUL) << GPADC_COEF4_COE4_POS)
#define GPADC_COEF4_COE4                                    GPADC_COEF4_COE4_Msk


/*******************  Bit definition for GPADC_COEF5 register  *******************/
#define GPADC_COEF5_COE5_POS                                (0U)
#define GPADC_COEF5_COE5_Len                                (20U)
#define GPADC_COEF5_COE5_Msk                                ((0xFFFFFUL) << GPADC_COEF5_COE5_POS)
#define GPADC_COEF5_COE5                                    GPADC_COEF5_COE5_Msk


/*******************  Bit definition for GPADC_COEF6 register  *******************/
#define GPADC_COEF6_COE6_POS                                (0U)
#define GPADC_COEF6_COE6_Len                                (20U)
#define GPADC_COEF6_COE6_Msk                                ((0xFFFFFUL) << GPADC_COEF6_COE6_POS)
#define GPADC_COEF6_COE6                                    GPADC_COEF6_COE6_Msk


/*******************  Bit definition for GPADC_COEF7 register  *******************/
#define GPADC_COEF7_COE7_POS                                (0U)
#define GPADC_COEF7_COE7_Len                                (20U)
#define GPADC_COEF7_COE7_Msk                                ((0xFFFFFUL) << GPADC_COEF7_COE7_POS)
#define GPADC_COEF7_COE7                                    GPADC_COEF7_COE7_Msk


/*******************  Bit definition for GPADC_COEF8 register  *******************/
#define GPADC_COEF8_COE8_POS                                (0U)
#define GPADC_COEF8_COE8_Len                                (20U)
#define GPADC_COEF8_COE8_Msk                                ((0xFFFFFUL) << GPADC_COEF8_COE8_POS)
#define GPADC_COEF8_COE8                                    GPADC_COEF8_COE8_Msk


/*******************  Bit definition for GPADC_COEF9 register  *******************/
#define GPADC_COEF9_COE9_POS                                (0U)
#define GPADC_COEF9_COE9_Len                                (20U)
#define GPADC_COEF9_COE9_Msk                                ((0xFFFFFUL) << GPADC_COEF9_COE9_POS)
#define GPADC_COEF9_COE9                                    GPADC_COEF9_COE9_Msk


/*******************  Bit definition for GPADC_COEF10 register  *******************/
#define GPADC_COEF10_COE10_POS                              (0U)
#define GPADC_COEF10_COE10_Len                              (20U)
#define GPADC_COEF10_COE10_Msk                              ((0xFFFFFUL) << GPADC_COEF10_COE10_POS)
#define GPADC_COEF10_COE10                                  GPADC_COEF10_COE10_Msk


/*******************  Bit definition for GPADC_COEF11 register  *******************/
#define GPADC_COEF11_COE11_POS                              (0U)
#define GPADC_COEF11_COE11_Len                              (20U)
#define GPADC_COEF11_COE11_Msk                              ((0xFFFFFUL) << GPADC_COEF11_COE11_POS)
#define GPADC_COEF11_COE11                                  GPADC_COEF11_COE11_Msk


/*******************  Bit definition for GPADC_COEF12 register  *******************/
#define GPADC_COEF12_COE12_POS                              (0U)
#define GPADC_COEF12_COE12_Len                              (20U)
#define GPADC_COEF12_COE12_Msk                              ((0xFFFFFUL) << GPADC_COEF12_COE12_POS)
#define GPADC_COEF12_COE12                                  GPADC_COEF12_COE12_Msk


/*******************  Bit definition for GPADC_COEF13 register  *******************/
#define GPADC_COEF13_COE13_POS                              (0U)
#define GPADC_COEF13_COE13_Len                              (20U)
#define GPADC_COEF13_COE13_Msk                              ((0xFFFFFUL) << GPADC_COEF13_COE13_POS)
#define GPADC_COEF13_COE13                                  GPADC_COEF13_COE13_Msk


/*******************  Bit definition for GPADC_COEF14 register  *******************/
#define GPADC_COEF14_COE14_POS                              (0U)
#define GPADC_COEF14_COE14_Len                              (20U)
#define GPADC_COEF14_COE14_Msk                              ((0xFFFFFUL) << GPADC_COEF14_COE14_POS)
#define GPADC_COEF14_COE14                                  GPADC_COEF14_COE14_Msk


/*******************  Bit definition for GPADC_DATA register  *******************/
#define GPADC_DATA_DATA_P_POS                               (0U)
#define GPADC_DATA_DATA_P_Len                               (16U)
#define GPADC_DATA_DATA_P_Msk                               ((0xFFFFUL) << GPADC_DATA_DATA_P_POS)
#define GPADC_DATA_DATA_P                                   GPADC_DATA_DATA_P_Msk

#define GPADC_DATA_DATA_N_POS                               (16U)
#define GPADC_DATA_DATA_N_Len                               (16U)
#define GPADC_DATA_DATA_N_Msk                               ((0xFFFFUL) << GPADC_DATA_DATA_N_POS)
#define GPADC_DATA_DATA_N                                   GPADC_DATA_DATA_N_Msk


/*******************  Bit definition for GPADC_CONSTANT register  *******************/
#define GPADC_CONSTANT_HALF_POS                             (0U)
#define GPADC_CONSTANT_HALF_Len                             (13U)
#define GPADC_CONSTANT_HALF_Msk                             ((0x1FFFUL) << GPADC_CONSTANT_HALF_POS)
#define GPADC_CONSTANT_HALF                                 GPADC_CONSTANT_HALF_Msk

#define GPADC_CONSTANT_TRANS_POS                            (16U)
#define GPADC_CONSTANT_TRANS_Len                            (15U)
#define GPADC_CONSTANT_TRANS_Msk                            ((0x7FFFUL) << GPADC_CONSTANT_TRANS_POS)
#define GPADC_CONSTANT_TRANS                                GPADC_CONSTANT_TRANS_Msk


/*******************  Bit definition for GPADC_OFFSET register  *******************/
#define GPADC_OFFSET_OFFSET_VALUE_POS                       (0U)
#define GPADC_OFFSET_OFFSET_VALUE_Len                       (16U)
#define GPADC_OFFSET_OFFSET_VALUE_Msk                       ((0xFFFFUL) << GPADC_OFFSET_OFFSET_VALUE_POS)
#define GPADC_OFFSET_OFFSET_VALUE                           GPADC_OFFSET_OFFSET_VALUE_Msk

#define GPADC_OFFSET_OFFSET_AUTO_LOAD_POS                   (16U)
#define GPADC_OFFSET_OFFSET_AUTO_LOAD_Len                   (1U)
#define GPADC_OFFSET_OFFSET_AUTO_LOAD_Msk                   ((0x1UL) << GPADC_OFFSET_OFFSET_AUTO_LOAD_POS)
#define GPADC_OFFSET_OFFSET_AUTO_LOAD                       GPADC_OFFSET_OFFSET_AUTO_LOAD_Msk

#define GPADC_OFFSET_OFFSET_EN_POS                          (17U)
#define GPADC_OFFSET_OFFSET_EN_Len                          (1U)
#define GPADC_OFFSET_OFFSET_EN_Msk                          ((0x1UL) << GPADC_OFFSET_OFFSET_EN_POS)
#define GPADC_OFFSET_OFFSET_EN                              GPADC_OFFSET_OFFSET_EN_Msk


/*******************  Bit definition for GPADC_ANA_CTRL register  *******************/
#define GPADC_ANA_CTRL_VREF_PD_POS                          (0U)
#define GPADC_ANA_CTRL_VREF_PD_Len                          (1U)
#define GPADC_ANA_CTRL_VREF_PD_Msk                          ((0x1UL) << GPADC_ANA_CTRL_VREF_PD_POS)
#define GPADC_ANA_CTRL_VREF_PD                              GPADC_ANA_CTRL_VREF_PD_Msk

#define GPADC_ANA_CTRL_VREF_SEL_POS                         (1U)
#define GPADC_ANA_CTRL_VREF_SEL_Len                         (1U)
#define GPADC_ANA_CTRL_VREF_SEL_Msk                         ((0x1UL) << GPADC_ANA_CTRL_VREF_SEL_POS)
#define GPADC_ANA_CTRL_VREF_SEL                             GPADC_ANA_CTRL_VREF_SEL_Msk

#define GPADC_ANA_CTRL_ADC_INPUT_POS                        (4U)
#define GPADC_ANA_CTRL_ADC_INPUT_Len                        (1U)
#define GPADC_ANA_CTRL_ADC_INPUT_Msk                        ((0x1UL) << GPADC_ANA_CTRL_ADC_INPUT_POS)
#define GPADC_ANA_CTRL_ADC_INPUT                            GPADC_ANA_CTRL_ADC_INPUT_Msk

#define GPADC_ANA_CTRL_ADC_INPUT_SEL_POS                    (5U)
#define GPADC_ANA_CTRL_ADC_INPUT_SEL_Len                    (2U)
#define GPADC_ANA_CTRL_ADC_INPUT_SEL_Msk                    ((0x3UL) << GPADC_ANA_CTRL_ADC_INPUT_SEL_POS)
#define GPADC_ANA_CTRL_ADC_INPUT_SEL                        GPADC_ANA_CTRL_ADC_INPUT_SEL_Msk

#define GPADC_ANA_CTRL_CALI_SEL_POS                         (8U)
#define GPADC_ANA_CTRL_CALI_SEL_Len                         (2U)
#define GPADC_ANA_CTRL_CALI_SEL_Msk                         ((0x3UL) << GPADC_ANA_CTRL_CALI_SEL_POS)
#define GPADC_ANA_CTRL_CALI_SEL                             GPADC_ANA_CTRL_CALI_SEL_Msk

#define GPADC_ANA_CTRL_CH_SEL_P_POS                         (16U)
#define GPADC_ANA_CTRL_CH_SEL_P_Len                         (3U)
#define GPADC_ANA_CTRL_CH_SEL_P_Msk                         ((0x7UL) << GPADC_ANA_CTRL_CH_SEL_P_POS)
#define GPADC_ANA_CTRL_CH_SEL_P                             GPADC_ANA_CTRL_CH_SEL_P_Msk

#define GPADC_ANA_CTRL_CH_SEL_N_POS                         (12U)
#define GPADC_ANA_CTRL_CH_SEL_N_Len                         (3U)
#define GPADC_ANA_CTRL_CH_SEL_N_Msk                         ((0x7UL) << GPADC_ANA_CTRL_CH_SEL_N_POS)
#define GPADC_ANA_CTRL_CH_SEL_N                             GPADC_ANA_CTRL_CH_SEL_N_Msk

/*******************  Bit definition for GPADC_FIFO_RD register  *******************/
#define GPADC_FIFO_RD_DATA_POS                               (0U)
#define GPADC_FIFO_RD_DATA_Len                               (32U)
#define GPADC_FIFO_RD_DATA_Msk                               (0xFFFFFFFFUL << GPADC_FIFO_RD_DATA_POS)
#define GPADC_FIFO_RD_DATA                                   GPADC_FIFO_RD_DATA_Msk

/*******************  Bit definition for GPADC_FIFO_THD register  *******************/
#define GPADC_FIFO_THD_POS                                   (0U)
#define GPADC_FIFO_THD_Len                                   (6U)
#define GPADC_FIFO_THD_Msk                                   (0x3FUL << GPADC_FIFO_THD_POS)
#define GPADC_FIFO_THD                                       GPADC_FIFO_THD_Msk

/*******************  Bit definition for GPADC_FIFO_STAT register  *******************/
#define GPADC_FIFO_STAT_COUNT_POS                            (0U)
#define GPADC_FIFO_STAT_COUNT_Len                            (7U)
#define GPADC_FIFO_STAT_COUNT_Msk                            (0x7FUL << GPADC_FIFO_STAT_COUNT_POS)
#define GPADC_FIFO_STAT_COUNT                                GPADC_FIFO_STAT_COUNT_Msk

#define GPADC_FIFO_STAT_VALID_POS                            (8U)
#define GPADC_FIFO_STAT_VALID_Len                            (1U)
#define GPADC_FIFO_STAT_VALID_Msk                            (0x1UL << GPADC_FIFO_STAT_VALID_POS)
#define GPADC_FIFO_STAT_VALID                                GPADC_FIFO_STAT_VALID_Msk

#define GPADC_FIFO_STAT_FLUSH_POS                            (16U)
#define GPADC_FIFO_STAT_FLUSH_Len                            (1U)
#define GPADC_FIFO_STAT_FLUSH_Msk                            (0x1UL << GPADC_FIFO_STAT_FLUSH_POS)
#define GPADC_FIFO_STAT_FLUSH                                GPADC_FIFO_STAT_FLUSH_Msk

/*******************  Bit definition for GPADC_ANA_MBG register  *******************/
#define GPADC_ANA_MBG_LDO23_SEL_POS                                   (4U)
#define GPADC_ANA_MBG_LDO23_SEL_Len                                   (4U)
#define GPADC_ANA_MBG_LDO23_SEL_Msk                                   (0xFUL << GPADC_ANA_MBG_LDO23_SEL_POS)
#define GPADC_ANA_MBG_LDO23_SEL                                       GPADC_ANA_MBG_LDO23_SEL_Msk

/*******************  Bit definition for GPADC_ANA_PGA register  *******************/
#define GPADC_ANA_PGA_PD_POS                                   (4U)
#define GPADC_ANA_PGA_PD_Len                                   (1U)
#define GPADC_ANA_PGA_PD_Msk                                   (0x1UL << GPADC_ANA_PGA_PD_POS)
#define GPADC_ANA_PGA_PD                                       GPADC_ANA_PGA_PD_Msk

#define GPADC_ANA_PGA_GAIN_CTRL_POS                                   (12U)
#define GPADC_ANA_PGA_GAIN_CTRL_Len                                   (5U)
#define GPADC_ANA_PGA_GAIN_CTRL_Msk                                   (0x1FUL << GPADC_ANA_PGA_GAIN_CTRL_POS)
#define GPADC_ANA_PGA_GAIN_CTRL                                       GPADC_ANA_PGA_GAIN_CTRL_Msk

/* ================================================================================================================= */
/* ================                                        SADC                                     ================ */
/* ================================================================================================================= */
/*******************  Bit definition for SADC_FIFO_RD register  *******************/
#define SADC_FIFO_RD_DATA_POS                               (0U)
#define SADC_FIFO_RD_DATA_Len                               (32U)
#define SADC_FIFO_RD_DATA_Msk                               (0xFFFFFFFFUL << SADC_FIFO_RD_DATA_POS)
#define SADC_FIFO_RD_DATA                                   SADC_FIFO_RD_DATA_Msk

/*******************  Bit definition for SADC_FIFO_THD register  *******************/
#define SADC_FIFO_THD_FIFO_THD_POS                          (0U)
#define SADC_FIFO_THD_FIFO_THD_Len                          (6U)
#define SADC_FIFO_THD_FIFO_THD_Msk                          (0x3FUL << SADC_FIFO_THD_FIFO_THD_POS)
#define SADC_FIFO_THD_FIFO_THD                              SADC_FIFO_THD_FIFO_THD_Msk

/*******************  Bit definition for SADC_FIFO_STAT register  *******************/
#define SADC_FIFO_STAT_COUNT_POS                            (0U)
#define SADC_FIFO_STAT_COUNT_Len                            (7U)
#define SADC_FIFO_STAT_COUNT_Msk                            (0x7FUL << SADC_FIFO_STAT_COUNT_POS)
#define SADC_FIFO_STAT_COUNT                                SADC_FIFO_STAT_COUNT_Msk

#define SADC_FIFO_STAT_VALID_POS                            (8U)
#define SADC_FIFO_STAT_VALID_Len                            (1U)
#define SADC_FIFO_STAT_VALID_Msk                            (0x1UL << SADC_FIFO_STAT_VALID_POS)
#define SADC_FIFO_STAT_VALID                                SADC_FIFO_STAT_VALID_Msk

#define SADC_FIFO_STAT_FLUSH_POS                            (16U)
#define SADC_FIFO_STAT_FLUSH_Len                            (1U)
#define SADC_FIFO_STAT_FLUSH_Msk                            (0x1UL << SADC_FIFO_STAT_FLUSH_POS)
#define SADC_FIFO_STAT_FLUSH                                SADC_FIFO_STAT_FLUSH_Msk

/*******************  Bit definition for SADC_CLK register  *******************/
#define SADC_CLK_CLK_SEL_POS                                (0U)
#define SADC_CLK_CLK_SEL_Len                                (3U)
#define SADC_CLK_CLK_SEL_Msk                                (0x7UL << SADC_CLK_CLK_SEL_POS)
#define SADC_CLK_CLK_SEL                                    SADC_CLK_CLK_SEL_Msk

#define SADC_CLK_CLK_RD_POS                                 (16U)
#define SADC_CLK_CLK_RD_Len                                 (3U)
#define SADC_CLK_CLK_RD_Msk                                 (0x7UL << SADC_CLK_CLK_RD_POS)
#define SADC_CLK_CLK_RD                                     SADC_CLK_CLK_RD_Msk

#define SADC_CLK_None_POS                                   (19U)
#define SADC_CLK_None_Len                                   (13U)
#define SADC_CLK_None_Msk                                   (0x1FFFUL << SADC_CLK_None_POS)
#define SADC_CLK_None                                       SADC_CLK_None_Msk

/*******************  Bit definition for SADC_GET_TKN_HW register  *******************/
#define SADC_GET_TKN_HW_LOCKED_POS                          (0U)
#define SADC_GET_TKN_HW_LOCKED_Len                          (1U)
#define SADC_GET_TKN_HW_LOCKED_Msk                          (0x1UL << SADC_GET_TKN_HW_LOCKED_POS)
#define SADC_GET_TKN_HW_LOCKED                              SADC_GET_TKN_HW_LOCKED_Msk

#define SADC_GET_TKN_HW_OWNER_POS                           (8U)
#define SADC_GET_TKN_HW_OWNER_Len                           (1U)
#define SADC_GET_TKN_HW_OWNER_Msk                           (0x1UL << SADC_GET_TKN_HW_OWNER_POS)
#define SADC_GET_TKN_HW_OWNER                               SADC_GET_TKN_HW_OWNER_Msk

/*******************  Bit definition for SADC_GET_TKN_SW register  *******************/
#define SADC_GET_TKN_SW_LOCKED_POS                          (0U)
#define SADC_GET_TKN_SW_LOCKED_Len                          (1U)
#define SADC_GET_TKN_SW_LOCKED_Msk                          (0x1UL << SADC_GET_TKN_SW_LOCKED_POS)
#define SADC_GET_TKN_SW_LOCKED                              SADC_GET_TKN_SW_LOCKED_Msk

#define SADC_GET_TKN_SW_OWNER_POS                           (8U)
#define SADC_GET_TKN_SW_OWNER_Len                           (1U)
#define SADC_GET_TKN_SW_OWNER_Msk                           (0x1UL << SADC_GET_TKN_SW_OWNER_POS)
#define SADC_GET_TKN_SW_OWNER                               SADC_GET_TKN_SW_OWNER_Msk

/*******************  Bit definition for SADC_RET_TKN_HW register  *******************/
#define SADC_RET_TKN_HW_RELEASE_POS                         (0U)
#define SADC_RET_TKN_HW_RELEASE_Len                         (1U)
#define SADC_RET_TKN_HW_RELEASE_Msk                         (0x1UL << SADC_RET_TKN_HW_RELEASE_POS)
#define SADC_RET_TKN_HW_RELEASE                             SADC_RET_TKN_HW_RELEASE_Msk

/*******************  Bit definition for SADC_RET_TKN_SW register  *******************/
#define SADC_RET_TKN_SW_RELEASE_POS                         (0U)
#define SADC_RET_TKN_SW_RELEASE_Len                         (1U)
#define SADC_RET_TKN_SW_RELEASE_Msk                         (0x1UL << SADC_RET_TKN_SW_RELEASE_POS)
#define SADC_RET_TKN_SW_RELEASE                             SADC_RET_TKN_SW_RELEASE_Msk

/*******************  Bit definition for SADC_TKN_STAT register  *******************/
#define SADC_TKN_STAT_LOCKED_POS                            (0U)
#define SADC_TKN_STAT_LOCKED_Len                            (1U)
#define SADC_TKN_STAT_LOCKED_Msk                            (0x1UL << SADC_TKN_STAT_LOCKED_POS)
#define SADC_TKN_STAT_LOCKED                                SADC_TKN_STAT_LOCKED_Msk

#define SADC_TKN_STAT_OWNER_POS                             (8U)
#define SADC_TKN_STAT_OWNER_Len                             (1U)
#define SADC_TKN_STAT_OWNER_Msk                             (0x1UL << SADC_TKN_STAT_OWNER_POS)
#define SADC_TKN_STAT_OWNER                                 SADC_TKN_STAT_OWNER_Msk

/* ================================================================================================================= */
/* ================                                        SADC                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for SADC_FIFO_RD register  *******************/
#define SADC_FIFO_RD_DATA_POS                               (0U)
#define SADC_FIFO_RD_DATA_Len                               (32U)
#define SADC_FIFO_RD_DATA_Msk                               (0xFFFFFFFFUL << SADC_FIFO_RD_DATA_POS)
#define SADC_FIFO_RD_DATA                                   SADC_FIFO_RD_DATA_Msk

/*******************  Bit definition for SADC_FIFO_THD register  *******************/
#define SADC_FIFO_THD_FIFO_THD_POS                          (0U)
#define SADC_FIFO_THD_FIFO_THD_Len                          (6U)
#define SADC_FIFO_THD_FIFO_THD_Msk                          (0x3FUL << SADC_FIFO_THD_FIFO_THD_POS)
#define SADC_FIFO_THD_FIFO_THD                              SADC_FIFO_THD_FIFO_THD_Msk

/*******************  Bit definition for SADC_FIFO_STAT register  *******************/
#define SADC_FIFO_STAT_COUNT_POS                            (0U)
#define SADC_FIFO_STAT_COUNT_Len                            (7U)
#define SADC_FIFO_STAT_COUNT_Msk                            (0x7FUL << SADC_FIFO_STAT_COUNT_POS)
#define SADC_FIFO_STAT_COUNT                                SADC_FIFO_STAT_COUNT_Msk

#define SADC_FIFO_STAT_VALID_POS                            (8U)
#define SADC_FIFO_STAT_VALID_Len                            (1U)
#define SADC_FIFO_STAT_VALID_Msk                            (0x1UL << SADC_FIFO_STAT_VALID_POS)
#define SADC_FIFO_STAT_VALID                                SADC_FIFO_STAT_VALID_Msk

#define SADC_FIFO_STAT_FLUSH_POS                            (16U)
#define SADC_FIFO_STAT_FLUSH_Len                            (1U)
#define SADC_FIFO_STAT_FLUSH_Msk                            (0x1UL << SADC_FIFO_STAT_FLUSH_POS)
#define SADC_FIFO_STAT_FLUSH                                SADC_FIFO_STAT_FLUSH_Msk

/*******************  Bit definition for SADC_CLK register  *******************/
#define SADC_CLK_CLK_SEL_POS                                (0U)
#define SADC_CLK_CLK_SEL_Len                                (3U)
#define SADC_CLK_CLK_SEL_Msk                                (0x7UL << SADC_CLK_CLK_SEL_POS)
#define SADC_CLK_CLK_SEL                                    SADC_CLK_CLK_SEL_Msk

#define SADC_CLK_CLK_RD_POS                                 (16U)
#define SADC_CLK_CLK_RD_Len                                 (3U)
#define SADC_CLK_CLK_RD_Msk                                 (0x7UL << SADC_CLK_CLK_RD_POS)
#define SADC_CLK_CLK_RD                                     SADC_CLK_CLK_RD_Msk

/*******************  Bit definition for SADC_GET_TKN_HW register  *******************/
#define SADC_GET_TKN_HW_LOCKED_POS                          (0U)
#define SADC_GET_TKN_HW_LOCKED_Len                          (1U)
#define SADC_GET_TKN_HW_LOCKED_Msk                          (0x1UL << SADC_GET_TKN_HW_LOCKED_POS)
#define SADC_GET_TKN_HW_LOCKED                              SADC_GET_TKN_HW_LOCKED_Msk

#define SADC_GET_TKN_HW_OWNER_POS                           (8U)
#define SADC_GET_TKN_HW_OWNER_Len                           (1U)
#define SADC_GET_TKN_HW_OWNER_Msk                           (0x1UL << SADC_GET_TKN_HW_OWNER_POS)
#define SADC_GET_TKN_HW_OWNER                               SADC_GET_TKN_HW_OWNER_Msk

/*******************  Bit definition for SADC_GET_TKN_SW register  *******************/
#define SADC_GET_TKN_SW_LOCKED_POS                          (0U)
#define SADC_GET_TKN_SW_LOCKED_Len                          (1U)
#define SADC_GET_TKN_SW_LOCKED_Msk                          (0x1UL << SADC_GET_TKN_SW_LOCKED_POS)
#define SADC_GET_TKN_SW_LOCKED                              SADC_GET_TKN_SW_LOCKED_Msk

#define SADC_GET_TKN_SW_OWNER_POS                           (8U)
#define SADC_GET_TKN_SW_OWNER_Len                           (1U)
#define SADC_GET_TKN_SW_OWNER_Msk                           (0x1UL << SADC_GET_TKN_SW_OWNER_POS)
#define SADC_GET_TKN_SW_OWNER                               SADC_GET_TKN_SW_OWNER_Msk

/*******************  Bit definition for SADC_RET_TKN_HW register  *******************/
#define SADC_RET_TKN_HW_RELEASE_POS                         (0U)
#define SADC_RET_TKN_HW_RELEASE_Len                         (1U)
#define SADC_RET_TKN_HW_RELEASE_Msk                         (0x1UL << SADC_RET_TKN_HW_RELEASE_POS)
#define SADC_RET_TKN_HW_RELEASE                             SADC_RET_TKN_HW_RELEASE_Msk

/*******************  Bit definition for SADC_RET_TKN_SW register  *******************/
#define SADC_RET_TKN_SW_RELEASE_POS                         (0U)
#define SADC_RET_TKN_SW_RELEASE_Len                         (1U)
#define SADC_RET_TKN_SW_RELEASE_Msk                         (0x1UL << SADC_RET_TKN_SW_RELEASE_POS)
#define SADC_RET_TKN_SW_RELEASE                             SADC_RET_TKN_SW_RELEASE_Msk

/*******************  Bit definition for SADC_TKN_STAT register  *******************/
#define SADC_TKN_STAT_LOCKED_POS                            (0U)
#define SADC_TKN_STAT_LOCKED_Len                            (1U)
#define SADC_TKN_STAT_LOCKED_Msk                            (0x1UL << SADC_TKN_STAT_LOCKED_POS)
#define SADC_TKN_STAT_LOCKED                                SADC_TKN_STAT_LOCKED_Msk

#define SADC_TKN_STAT_OWNER_POS                             (8U)
#define SADC_TKN_STAT_OWNER_Len                             (1U)
#define SADC_TKN_STAT_OWNER_Msk                             (0x1UL << SADC_TKN_STAT_OWNER_POS)
#define SADC_TKN_STAT_OWNER                                 SADC_TKN_STAT_OWNER_Msk


/* ================================================================================================================= */
/* ================                                        PIN_MUX                                  ================ */
/* ================================================================================================================= */
/*******************  Bit definition for PIN_MUX_DPAD_MUX_CTRL_00_07 register  *******************/
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_00_POS              (0U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_00_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_00_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_00_POS)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_00                  PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_00_Msk

#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_01_POS              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_01_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_01_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_01_POS)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_01                  PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_01_Msk

#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_02_POS              (8U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_02_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_02_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_02_POS)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_02                  PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_02_Msk

#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_03_POS              (12U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_03_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_03_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_03_POS)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_03                  PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_03_Msk

#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_04_POS              (16U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_04_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_04_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_04_POS)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_04                  PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_04_Msk

#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_05_POS              (20U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_05_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_05_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_05_POS)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_05                  PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_05_Msk

#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_06_POS              (24U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_06_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_06_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_06_POS)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_06                  PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_06_Msk

#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_07_POS              (28U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_07_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_07_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_07_POS)
#define PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_07                  PIN_MUX_DPAD_MUX_CTRL_00_07_SEL_07_Msk

/*******************  Bit definition for PIN_MUX_DPAD_MUX_CTRL_08_15 register  *******************/
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_08_POS              (0U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_08_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_08_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_08_POS)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_08                  PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_08_Msk

#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_09_POS              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_09_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_09_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_09_POS)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_09                  PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_09_Msk

#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_10_POS              (8U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_10_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_10_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_10_POS)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_10                  PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_10_Msk

#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_11_POS              (12U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_11_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_11_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_11_POS)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_11                  PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_11_Msk

#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_12_POS              (16U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_12_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_12_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_12_POS)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_12                  PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_12_Msk

#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_13_POS              (20U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_13_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_13_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_13_POS)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_13                  PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_13_Msk

#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_14_POS              (24U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_14_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_14_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_14_POS)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_14                  PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_14_Msk

#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_15_POS              (28U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_15_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_15_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_15_POS)
#define PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_15                  PIN_MUX_DPAD_MUX_CTRL_08_15_SEL_15_Msk

/*******************  Bit definition for PIN_MUX_DPAD_MUX_CTRL_16_23 register  *******************/
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_16_POS              (0U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_16_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_16_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_16_POS)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_16                  PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_16_Msk

#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_17_POS              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_17_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_17_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_17_POS)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_17                  PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_17_Msk

#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_18_POS              (8U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_18_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_18_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_18_POS)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_18                  PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_18_Msk

#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_19_POS              (12U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_19_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_19_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_19_POS)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_19                  PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_19_Msk

#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_20_POS              (16U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_20_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_20_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_20_POS)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_20                  PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_20_Msk

#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_21_POS              (20U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_21_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_21_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_21_POS)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_21                  PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_21_Msk

#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_22_POS              (24U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_22_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_22_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_22_POS)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_22                  PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_22_Msk

#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_23_POS              (28U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_23_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_23_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_23_POS)
#define PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_23                  PIN_MUX_DPAD_MUX_CTRL_16_23_SEL_23_Msk

/*******************  Bit definition for PIN_MUX_DPAD_MUX_CTRL_24_31 register  *******************/
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_24_POS              (0U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_24_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_24_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_24_POS)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_24                  PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_24_Msk

#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_25_POS              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_25_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_25_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_25_POS)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_25                  PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_25_Msk

#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_26_POS              (8U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_26_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_26_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_26_POS)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_26                  PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_26_Msk

#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_27_POS              (12U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_27_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_27_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_27_POS)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_27                  PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_27_Msk

#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_28_POS              (16U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_28_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_28_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_28_POS)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_28                  PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_28_Msk

#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_29_POS              (20U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_29_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_29_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_29_POS)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_29                  PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_29_Msk

#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_30_POS              (24U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_30_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_30_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_30_POS)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_30                  PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_30_Msk

#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_31_POS              (28U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_31_Len              (4U)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_31_Msk              (0xFUL << PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_31_POS)
#define PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_31                  PIN_MUX_DPAD_MUX_CTRL_24_31_SEL_31_Msk

/*******************  Bit definition for PIN_MUX_AON_PAD_MUX_CTRL register  *******************/
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_00_POS                 (0U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_00_Len                 (3U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_00_Msk                 (0x7UL << PIN_MUX_AON_PAD_MUX_CTRL_SEL_00_POS)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_00                     PIN_MUX_AON_PAD_MUX_CTRL_SEL_00_Msk

#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_01_POS                 (4U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_01_Len                 (3U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_01_Msk                 (0x7UL << PIN_MUX_AON_PAD_MUX_CTRL_SEL_01_POS)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_01                     PIN_MUX_AON_PAD_MUX_CTRL_SEL_01_Msk

#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_02_POS                 (8U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_02_Len                 (3U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_02_Msk                 (0x7UL << PIN_MUX_AON_PAD_MUX_CTRL_SEL_02_POS)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_02                     PIN_MUX_AON_PAD_MUX_CTRL_SEL_02_Msk

#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_03_POS                 (12U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_03_Len                 (3U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_03_Msk                 (0x7UL << PIN_MUX_AON_PAD_MUX_CTRL_SEL_03_POS)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_03                     PIN_MUX_AON_PAD_MUX_CTRL_SEL_03_Msk

#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_04_POS                 (16U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_04_Len                 (3U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_04_Msk                 (0x7UL << PIN_MUX_AON_PAD_MUX_CTRL_SEL_04_POS)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_04                     PIN_MUX_AON_PAD_MUX_CTRL_SEL_04_Msk

#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_05_POS                 (20U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_05_Len                 (3U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_05_Msk                 (0x7UL << PIN_MUX_AON_PAD_MUX_CTRL_SEL_05_POS)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_05                     PIN_MUX_AON_PAD_MUX_CTRL_SEL_05_Msk

#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_06_POS                 (24U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_06_Len                 (3U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_06_Msk                 (0x7UL << PIN_MUX_AON_PAD_MUX_CTRL_SEL_06_POS)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_06                     PIN_MUX_AON_PAD_MUX_CTRL_SEL_06_Msk

#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_07_POS                 (28U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_07_Len                 (3U)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_07_Msk                 (0x7UL << PIN_MUX_AON_PAD_MUX_CTRL_SEL_07_POS)
#define PIN_MUX_AON_PAD_MUX_CTRL_SEL_07                     PIN_MUX_AON_PAD_MUX_CTRL_SEL_07_Msk

/*******************  Bit definition for PIN_MUX_MSIO_APAD_MUX_CTRL register  *******************/
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_00_POS               (0U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_00_Len               (3U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_00_Msk               (0x7UL << PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_00_POS)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_00                   PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_00_Msk

#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_01_POS               (4U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_01_Len               (3U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_01_Msk               (0x7UL << PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_01_POS)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_01                   PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_01_Msk

#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_02_POS               (8U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_02_Len               (3U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_02_Msk               (0x7UL << PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_02_POS)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_02                   PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_02_Msk

#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_03_POS               (12U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_03_Len               (3U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_03_Msk               (0x7UL << PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_03_POS)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_03                   PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_03_Msk

#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_04_POS               (16U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_04_Len               (3U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_04_Msk               (0x7UL << PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_04_POS)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_04                   PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_04_Msk

#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_05_POS               (20U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_05_Len               (3U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_05_Msk               (0x7UL << PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_05_POS)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_05                   PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_05_Msk

#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_06_POS               (24U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_06_Len               (3U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_06_Msk               (0x7UL << PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_06_POS)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_06                   PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_06_Msk

#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_07_POS               (28U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_07_Len               (3U)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_07_Msk               (0x7UL << PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_07_POS)
#define PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_07                   PIN_MUX_MSIO_APAD_MUX_CTRL_SEL_07_Msk

/*******************  Bit definition for PIN_MUX_MSIO_BPAD_MUX_CTRL register  *******************/
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_00_POS               (0U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_00_Len               (3U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_00_Msk               (0x7UL << PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_00_POS)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_00                   PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_00_Msk

#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_01_POS               (4U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_01_Len               (3U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_01_Msk               (0x7UL << PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_01_POS)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_01                   PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_01_Msk

#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_02_POS               (8U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_02_Len               (3U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_02_Msk               (0x7UL << PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_02_POS)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_02                   PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_02_Msk

#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_03_POS               (12U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_03_Len               (3U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_03_Msk               (0x7UL << PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_03_POS)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_03                   PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_03_Msk

#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_04_POS               (16U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_04_Len               (3U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_04_Msk               (0x7UL << PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_04_POS)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_04                   PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_04_Msk

#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_05_POS               (20U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_05_Len               (3U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_05_Msk               (0x7UL << PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_05_POS)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_05                   PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_05_Msk

#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_06_POS               (24U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_06_Len               (3U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_06_Msk               (0x7UL << PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_06_POS)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_06                   PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_06_Msk

#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_07_POS               (28U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_07_Len               (3U)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_07_Msk               (0x7UL << PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_07_POS)
#define PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_07                   PIN_MUX_MSIO_BPAD_MUX_CTRL_SEL_07_Msk

/** @} */ /* End of group Peripheral_Registers_Bits_Definition */

/** @addtogroup Exported_macros
  * @{
  */
/****************************** GPIO instances ********************************/
#define IS_GPIO_ALL_INSTANCE(__INSTANCE__)      (((__INSTANCE__) == GPIO0) || \
                                                 ((__INSTANCE__) == GPIO1) || \
                                                 ((__INSTANCE__) == GPIO2))

/****************************** I2C instances *********************************/
#define IS_I2C_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == I2C0) || \
                                                 ((__INSTANCE__) == I2C1))

/****************************** I2S instances *********************************/
#define IS_I2S_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == I2S_M) || \
                                                 ((__INSTANCE__) == I2S_S))

/****************************** UART instances ********************************/
#define IS_UART_ALL_INSTANCE(__INSTANCE__)      (((__INSTANCE__) == UART0) || \
                                                 ((__INSTANCE__) == UART1) || \
                                                 ((__INSTANCE__) == UART2) || \
                                                 ((__INSTANCE__) == UART3) || \
                                                 ((__INSTANCE__) == UART4) || \
                                                 ((__INSTANCE__) == UART5))

/******************** UART instances : Support of DMA *************************/
#define IS_UART_DMA_INSTANCE(__INSTANCE__)      (((__INSTANCE__) == UART0))

/****************************** TIM instances *********************************/
#define IS_TIMER_ALL_INSTANCE(__INSTANCE__)     (((__INSTANCE__) == TIMER0) || \
                                                 ((__INSTANCE__) == TIMER1))

/****************************** DUAL TIM instances ****************************/
#define IS_DUAL_TIM_ALL_INSTANCE(__INSTANCE__)  (((__INSTANCE__) == DUAL_TIMER0) || \
                                                 ((__INSTANCE__) == DUAL_TIMER1))

/****************************** PWM instances *********************************/
#define IS_PWM_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == PWM0) || \
                                                 ((__INSTANCE__) == PWM1))

/****************************** WDT instances *********************************/
#define IS_WDT_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == WDT))

/****************************** SPI instances *********************************/
#define IS_SPI_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == SPIM) || \
                                                 ((__INSTANCE__) == SPIS))

/****************************** QSPI instances ********************************/
#define IS_QSPI_ALL_INSTANCE(__INSTANCE__)      (((__INSTANCE__) == QSPI0) || \
                                                 ((__INSTANCE__) == QSPI1) || \
                                                 ((__INSTANCE__) == QSPI2) )

/****************************** PKC instances *********************************/
#define IS_PKC_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == PKC))

/****************************** AES Instances *********************************/
#define IS_AES_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == AES))

/****************************** HMAC Instances ********************************/
#define IS_HMAC_ALL_INSTANCE(__INSTANCE__)      (((__INSTANCE__) == HMAC))

/****************************** XQSPI Instances *******************************/
#define IS_XQSPI_ALL_INSTANCE(__INSTANCE__)     (((__INSTANCE__) == XQSPI))

/****************************** EFUSE Instances *******************************/
#define IS_EFUSE_ALL_INSTANCE(__INSTANCE__)     (((__INSTANCE__) == EFUSE))

/****************************** RNG Instances *******************************/
#define IS_RNG_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == RNG))

/****************************** DSPI Instances *******************************/
#define IS_DSPI_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == DSPI))

/****************************** PDM Instances *******************************/
#define IS_PDM_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == PDM))

/****************************** GPADC Instances *******************************/
#define IS_GPADC_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == GPADC))

/****************************** USB Instances *******************************/
#define IS_USB_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == USB))

/** @} */ /* End of group Exported_macros */


#ifdef __cplusplus
}
#endif

#endif /* __GR552xx_H__ */

/** @} */ /* End of group GR54xx */

/** @} */ /* End of group CMSIS_Device */

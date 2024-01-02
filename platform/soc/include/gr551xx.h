/**************************************************************************//**
 * @file     gr551xx.h
 * @brief    CMSIS Cortex-M# Core Peripheral Access Layer Header File for
 *           Device gr551xx
 * @version  V1.00
 * @date     12. June 2018
 ******************************************************************************/
/*
 * Copyright (c) 2016-2018, Shenzhen Huiding Technology Co., Ltd
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

/** @addtogroup GR551xx
  * @{
  */

#ifndef __GR551xx_H__
#define __GR551xx_H__

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
    AON_EXT_IRQn              =  18,  /**< External Wakeup Interrupt                                                 */
    RNG_IRQn                  =  19,  /**< RNG Interrupt                                                             */
    PMU_IRQn                  =  20,  /**< PMU Interrupt                                                             */
    PKC_IRQn                  =  21,  /**< PKC Interrupt                                                             */
    XQSPI_IRQn                =  22,  /**< XQSPI Interrupt                                                           */
    QSPI1_IRQn                =  23,  /**< QSPI1 Interrupt                                                           */
    PWR_CMD_IRQn              =  24,  /**< POWER CMD ACK Interrupt                                                   */
    BLESLP_IRQn               =  25,  /**< BLE Sleep Interrupt                                                       */
    SLPTIMER_IRQn             =  26,  /**< Sleep Timer Interrupt                                                     */
    COMP_IRQn                 =  27,  /**< Comparator Interrupt                                                      */
    AON_WDT_IRQn              =  28,  /**< Always on Watchdog Interrupt                                              */
    I2S_M_IRQn                =  29,  /**< I2S_M Interrupt                                                           */
    I2S_S_IRQn                =  30,  /**< I2S_S Interrupt                                                           */
    ISO7816_IRQn              =  31,  /**< ISO7816 Interrupt                                                         */
    PRESENT_IRQn              =  32,  /**< Presnet Done Interrupt                                                    */
    CALENDAR_IRQn             =  33,  /**< AON Calendar Timer Interrupt                                              */
    MAX_NUMS_IRQn             =  34,  /**< Last Interrupt                                                            */
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
typedef struct _aes_regs
{
    __IOM uint32_t CTRL;                /**< AES_REG_CTRL,          Address offset: 0x00 */
    __IOM uint32_t CONFIG;              /**< AES_REG_CONFIG,        Address offset: 0x04 */
    __IM  uint32_t STATUS;              /**< AES_REG_STATUS,        Address offset: 0x08 */
    __IOM uint32_t INTERRUPT;           /**< AES_REG_INTERRUPT,     Address offset: 0x0C */
    __IOM uint32_t TRAN_SIZE;           /**< AES_REG_TRAN_SIZE,     Address offset: 0x10 */
    __IOM uint32_t RSTART_ADDR;         /**< AES_REG_RSTART_ADDR,   Address offset: 0x14 */
    __IOM uint32_t WSTART_ADDR;         /**< AES_REG_WSTART_ADDR,   Address offset: 0x18 */
    __IOM uint32_t KEY_ADDR;            /**< AES_REG_KEY_ADDR,      Address offset: 0x1C */
    __IM  uint32_t DATA_OUT[4];         /**< AES_REG_DATA_OUT,      Address offset: 0x20 */
    __OM  uint32_t KEY[8];              /**< AES_REG_KEY,           Address offset: 0x30 */
    __IOM uint32_t SEED_IN;             /**< AES_REG_SEED_IN,       Address offset: 0x50 */
    __IOM uint32_t SEED_OUT;            /**< AES_REG_SEED_OUT,      Address offset: 0x54 */
    __IOM uint32_t SEED_IMASK;          /**< AES_REG_SEED_IMASK,    Address offset: 0x58 */
    __IOM uint32_t SEED_OSBOX;          /**< AES_REG_SEED_OSBOX,    Address offset: 0x5C */
    __OM  uint32_t VECTOR_INIT[4];      /**< AES_REG_VECTOR_INIT,   Address offset: 0x60 */
    __OM  uint32_t DATA_IN[4];          /**< AES_REG_DATA_IN,       Address offset: 0x70 */
    __OM  uint32_t KPORT_MASK;          /**< AES_REG_KPORT_MASK,    Address offset: 0x80 */
} aes_regs_t;

/**
  * @brief AON
  */
typedef struct _aon_regs
{
    __IOM uint32_t SOFTWARE_0;              /**< AON_REG_SOFTWARE_0,            Address offset: 0x00 */
    __IOM uint32_t PWR_RET01;               /**< AON_REG_PWR_RET01,             Address offset: 0x04 */
    __IOM uint32_t SNSADC_CFG;              /**< AON_REG_SNSADC_CFG,            Address offset: 0x08 */
    __IOM uint32_t RF_REG_0;                /**< AON_REG_RF_REG_0,              Address offset: 0x0C */
    __IOM uint32_t RF_REG_1;                /**< AON_REG_RF_REG_1,              Address offset: 0x10 */
    __IOM uint32_t RF_REG_2;                /**< AON_REG_RF_REG_2,              Address offset: 0x14 */
    __IOM uint32_t CALENDAR_TIMER_CTL;      /**< AON_REG_CALENDAR_TIMER_CTL,    Address offset: 0x18 */
    __IOM uint32_t MEM_STD_OVR;             /**< AON_REG_MEM_STD_OVR,           Address offset: 0x1C */
    __IOM uint32_t RF_REG_3;                /**< AON_REG_RF_REG_3,              Address offset: 0x20 */
    __IOM uint32_t RF_REG_4;                /**< AON_REG_RF_REG_4,              Address offset: 0x24 */
    __IOM uint32_t RF_REG_5;                /**< AON_REG_RF_REG_5,              Address offset: 0x28 */
    __IOM uint32_t RF_REG_6;                /**< AON_REG_RF_REG_6,              Address offset: 0x2C */
    __IOM uint32_t RF_REG_7;                /**< AON_REG_RF_REG_7,              Address offset: 0x30 */
    __IOM uint32_t RF_REG_8;                /**< AON_REG_RF_REG_8,              Address offset: 0x34 */
    __IOM uint32_t RF_REG_9;                /**< AON_REG_RF_REG_9,              Address offset: 0x38 */
    __IOM uint32_t MSIO_PAD_CFG_0;          /**< AON_REG_MSIO_PAD_CFG_0,        Address offset: 0x3C */
    __IOM uint32_t MSIO_PAD_CFG_1;          /**< AON_REG_MSIO_PAD_CFG_1,        Address offset: 0x40 */
    __IOM uint32_t SLP_EVENT;               /**< AON_REG_SLP_EVENT,             Address offset: 0x44 */
    __IOM uint32_t WARM_BOOT_TIME;          /**< AON_REG_WARM_BOOT_TIME,        Address offset: 0x48 */
    __IOM uint32_t RF_REG_10;               /**< AON_REG_RF_REG_10,             Address offset: 0x4C */
    __IOM uint32_t AON_PAD_CTL0;            /**< AON_REG_AON_PAD_CTL0,          Address offset: 0x50 */
    __IOM uint32_t MEM_N_SLP_CTL;           /**< AON_REG_MEM_N_SLP_CTL,         Address offset: 0x54 */
    __IOM uint32_t EXT_WKUP_CTL;            /**< AON_REG_EXT_WKUP_CTL,          Address offset: 0x58 */
    __IOM uint32_t AON_PAD_CTL1;            /**< AON_REG_AON_PAD_CTL1,          Address offset: 0x5C */
    __IOM uint32_t SOFTWARE_1;              /**< AON_REG_SOFTWARE_1,            Address offset: 0x60 */
    __IOM uint32_t MEM_PWR_SLP;             /**< AON_REG_MEM_PWR_SLP,           Address offset: 0x64 */
    __IOM uint32_t MEM_PWR_WKUP;            /**< AON_REG_MEM_PWR_WKUP,          Address offset: 0x68 */
    __IOM uint32_t PWR_RET27;               /**< AON_REG_PWR_RET27,             Address offset: 0x6C */
    __IOM uint32_t PWR_RET28;               /**< AON_REG_PWR_RET28,             Address offset: 0x70 */
    __IOM uint32_t PWR_RET29;               /**< AON_REG_PWR_RET29,             Address offset: 0x74 */
    __IOM uint32_t SOFTWARE_2;              /**< AON_REG_SOFTWARE_2,            Address offset: 0x78 */
    __IOM uint32_t PWR_RET31;               /**< AON_REG_PWR_RET31,             Address offset: 0x7C */
    __IOM uint32_t PSC_CMD;                 /**< AON_REG_PSC_CMD,               Address offset: 0x80 */
    __IOM uint32_t PSC_CMD_OPC;             /**< AON_REG_PSC_CMD_OPC,           Address offset: 0x84 */
    __IM  uint32_t MCU_RELEASE;             /**< AON_REG_MCU_RELEASE,           Address offset: 0x88 */
    __IM  uint32_t RESERVED0;               /**< Reserved,                      Address offset: 0x8C */
    __IOM uint32_t TIMER_VALUE;             /**< AON_REG_TIMER_VALUE,           Address offset: 0x90 */
    __IM  uint32_t TIMER_VAL;               /**< AON_REG_TIMER_VAL,             Address offset: 0x94 */
    __IM  uint32_t RESERVED1[13];           /**< Reserved,                      Address offset: 0x98 */
    __IOM uint32_t FPGA_CTRL;               /**< AON_REG_FPGA_CTRL,             Address offset: 0xCC */
    __IM  uint32_t RESERVED2[5];            /**< Reserved,                      Address offset: 0xD0 */
    __IOM uint32_t ST_CALIB;                /**< AON_REG_ST_CALIB_REG,          Address offset: 0xE4 */
} aon_regs_t;

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
    __IOM uint32_t RELOAD;          /**< DUAL_TIMER auto-reload register,            Address offset: 0x00 */
    __IM  uint32_t VALUE;           /**< DUAL_TIMER counter value register,          Address offset: 0x04 */
    __IOM uint32_t CTRL;            /**< DUAL_TIMER control register,                Address offset: 0x08 */
    __OM  uint32_t INTCLR;          /**< DUAL_TIMER interrupt status clear register, Address offset: 0x0C */
    __IM  uint32_t RAW_INTSTAT;     /**< DUAL_TIMER raw interrupt status register,   Address offset: 0x10 */
    __IM  uint32_t INTSTAT;         /**< DUAL_TIMER interrupt status register,       Address offset: 0x14 */
    __IOM uint32_t BG_LOAD;         /**< DUAL_TIMER background-reload register,      Address offset: 0x18 */
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
    __IM  uint32_t RESERVED1[241];      /**< GPIO_REG_RESERVED register,        Address offset: 0x03C */
    __IOM uint32_t MASKLOWBYTE[256];    /**< GPIO_REG_MASKLOWBYTE register,     Address offset: 0x400 */
    __IOM uint32_t MASKHIGHBYTE[256];   /**< GPIO_REG_MASKHIGHBYTE register,    Address offset: 0x500 */
} gpio_regs_t;

/**
  * @brief HMAC
  */
typedef struct _hmac_regs
{
    __IOM uint32_t CTRL;                /**< HMAC_REG_CTRL register,            Adderss offset: 0x00 */
    __IOM uint32_t CONFIG;              /**< HMAC_REG_CONFIG register,          Adderss offset: 0x04 */
    __IM  uint32_t STATUS;              /**< HMAC_REG_STATUS register,          Adderss offset: 0x08 */
    __IOM uint32_t TRAN_SIZE;           /**< HMAC_REG_TRAN_SIZE register,       Adderss offset: 0x0C */
    __IOM uint32_t INTERRUPT;           /**< HMAC_REG_INTERRUPT register,       Adderss offset: 0x10 */
    __IOM uint32_t RSTART_ADDR;         /**< HMAC_REG_RSTART_ADDR register,     Adderss offset: 0x14 */
    __IOM uint32_t WSTART_ADDR;         /**< HMAC_REG_WSTART_ADDR register,     Adderss offset: 0x18 */
    __IM  uint32_t REVERSED0;           /**< HMAC_REG_REVERSED register,        Adderss offset: 0x1C */
    __IOM uint32_t USER_HASH[8];        /**< HMAC_REG_USER_HASH register,       Adderss offset: 0x20 */
    __IM  uint32_t FIFO_OUT;            /**< HMAC_REG_FIFO_OUT register,        Adderss offset: 0x40 */
    __OM  uint32_t MESSAGE_FIFO;        /**< HMAC_REG_MESSAGE_FIFO register,    Adderss offset: 0x44 */
    __OM  uint32_t KEY[8];              /**< HMAC_REG_KEY register,             Adderss offset: 0x48 */
    __IOM uint32_t KEY_ADDR;            /**< HMAC_REG_KEY_ADDR register,        Adderss offset: 0x68 */
    __OM  uint32_t KPORT_MASK;          /**< HMAC_REG_KPORT_MASK register,      Adderss offset: 0x6C */
} hmac_regs_t;

/**
  * @brief I2C
  */
typedef struct _i2c_regs
{
    __IOM uint32_t CON;                        /**< I2C control,                                      Address offset: 0x00 */
    __IOM uint32_t TAR;                        /**< I2C target address,                               Address offset: 0x04 */
    __IOM uint32_t SAR;                        /**< I2C slave address,                                Address offset: 0x08 */
    __IOM uint32_t HS_MADDR;                   /**< I2C HS Master Mode Code address,                  Address offset: 0x0C */
    __IOM uint32_t DATA_CMD;                   /**< I2C Rx/Tx Data Buffer and Command,                Address offset: 0x10 */
    __IOM uint32_t SS_SCL_HCNT;                /**< Standard Speed I2C clock SCL High Count,          Address offset: 0x14 */
    __IOM uint32_t SS_SCL_LCNT;                /**< Standard Speed I2C clock SCL Low Count,           Address offset: 0x18 */
    __IOM uint32_t FS_SCL_HCNT;                /**< Fast Speed I2C clock SCL Low Count,               Address offset: 0x1C */
    __IOM uint32_t FS_SCL_LCNT;                /**< Fast Speed I2C clock SCL Low Count,               Address offset: 0x20 */
    __IOM uint32_t HS_SCL_HCNT;                /**< High Speed I2C clock SCL Low Count,               Address offset: 0x24 */
    __IOM uint32_t HS_SCL_LCNT;                /**< High Speed I2C clock SCL Low Count,               Address offset: 0x28 */
    __IM  uint32_t INTR_STAT;                  /**< I2C Interrupt Status,                             Address offset: 0x2C */
    __IOM uint32_t INTR_MASK;                  /**< I2C Interrupt Mask,                               Address offset: 0x30 */
    __IM  uint32_t RAW_INTR_STAT;              /**< I2C Raw Interrupt Status,                         Address offset: 0x34 */
    __IOM uint32_t RX_TL;                      /**< I2C Receive FIFO Threshold,                       Address offset: 0x38 */
    __IOM uint32_t TX_TL;                      /**< I2C Transmit FIFO Threshold,                      Address offset: 0x3C */
    __IM  uint32_t CLR_INTR;                   /**< Clear combined and Individual Interrupts,         Address offset: 0x40 */
    __IM  uint32_t CLR_RX_UNDER;               /**< Clear RX_UNDER Interrupt,                         Address offset: 0x44 */
    __IM  uint32_t CLR_RX_OVER;                /**< Clear RX_OVER Interrupt,                          Address offset: 0x48 */
    __IM  uint32_t CLR_TX_OVER;                /**< Clear TX_OVER Interrupt,                          Address offset: 0x4C */
    __IM  uint32_t CLR_RD_REQ;                 /**< Clear RQ_REQ Interrupt,                           Address offset: 0x50 */
    __IM  uint32_t CLR_TX_ABRT;                /**< Clear TX_ABRT Interrupt,                          Address offset: 0x54 */
    __IM  uint32_t CLR_RX_DONE;                /**< Clear RX_DONE Interrupt,                          Address offset: 0x58 */
    __IM  uint32_t CLR_ACTIVITY;               /**< Clear ACTIVITY Interrupt,                         Address offset: 0x5C */
    __IM  uint32_t CLR_STOP_DET;               /**< Clear STOP_DET Interrupt,                         Address offset: 0x60 */
    __IM  uint32_t CLR_START_DET;              /**< Clear START_DET Interrupt,                        Address offset: 0x64 */
    __IM  uint32_t CLR_GEN_CALL;               /**< Clear GEN_CALL Interrupt,                         Address offset: 0x68 */
    __IOM uint32_t ENABLE;                     /**< I2C Enable,                                       Address offset: 0x6C */
    __IM  uint32_t STATUS;                     /**< I2C Status,                                       Address offset: 0x70 */
    __IM  uint32_t TXFLR;                      /**< Transmit FIFO Level Register,                     Address offset: 0x74 */
    __IM  uint32_t RXFLR;                      /**< Receive FIFO Level Register,                      Address offset: 0x78 */
    __IOM uint32_t SDA_HOLD;                   /**< SDA Hold Time Length Reg,                         Address offset: 0x7C */
    __IM  uint32_t TX_ABRT_SOURCE;             /**< I2C Transmit Abort Status Reg,                    Address offset: 0x80 */
    __IOM uint32_t SLV_DATA_NACK_ONLY;         /**< Generate SLV_DATA_NACK Register,                  Address offset: 0x84 */
    __IOM uint32_t DMA_CR;                     /**< DMA Control Register,                             Address offset: 0x88 */
    __IOM uint32_t DMA_TDLR;                   /**< DMA Transmit Data Level,                          Address offset: 0x8C */
    __IOM uint32_t DMA_RDLR;                   /**< DMA Receive Data Level,                           Address offset: 0x90 */
    __IOM uint32_t SDA_SETUP;                  /**< SDA Setup Register,                               Address offset: 0x94 */
    __IOM uint32_t ACK_GENERAL_CALL;           /**< ACK General Call Register,                        Address offset: 0x98 */
    __IM  uint32_t ENABLE_STATUS;              /**< Enable Status Register,                           Address offset: 0x9C */
    __IOM uint32_t FS_SPKLEN;                  /**< ISS and FS spike suppression limit,               Address offset: 0xA0 */
    __IOM uint32_t HS_SPKLEN;                  /**< HS spike suppression limit,                       Address offset: 0xA4 */
    __IM  uint32_t CLR_RESTART_DET;            /**< Clear RESTART_DET Interrupt Register,             Address offset: 0xA8 */
    __IOM uint32_t SCL_STUCK_AT_LOW_TIMEOUT;   /**< I2C SCL Stuck at Low Timeout,                     Address offset: 0xAC */
    __IOM uint32_t SDA_STUCK_AT_LOW_TIMEOUT;   /**< I2C SDA Stuck at Low Timeout,                     Address offset: 0xB0 */
    __IM  uint32_t CLR_SCL_STUCK_DET;          /**< Clear SCL Stuck at Low Detect Interrupt Register, Address offset: 0xB4 */
    __IM  uint32_t DEVICE_ID;                  /**< I2C Device-ID Register,                           Address offset: 0xB8 */
    __IOM uint32_t SMBUS_CLK_LOW_SEXT;         /**< SMBus Slave Clock Extend Timeout Register,        Address offset: 0xBC */
    __IOM uint32_t SMBUS_CLK_LOW_MEXT;         /**< SMBus Master Clock Extend Timeout Register,       Address offset: 0xC0 */
    __IOM uint32_t SMBUS_THIGH_MAX_IDLE_COUNT; /**< SMBus Master THigh MAX Bus-idle count Register,   Address offset: 0xC4 */
    __IM  uint32_t SMBUS_INTSTAT;              /**< SMBUS Interrupt Status Register,                  Address offset: 0xC8 */
    __IOM uint32_t SMBUS_INTMASK;              /**< SMBus Interrupt Mask Register,                    Address offset: 0xCC */
    __IM  uint32_t SMBUS_RAW_INTSTAT;          /**< SMBus Raw Interrupt Status Register,              Address offset: 0xD0 */
    __OM  uint32_t SMBUS_INTCLR;               /**< SMBus Clear Interrupt Register,                   Address offset: 0xD4 */
    __IOM uint32_t OPTIONAL_SAR;               /**< I2C Optional Slave Address Register,              Address offset: 0xD8 */
    __IOM uint32_t SMBUS_UDID_LSB;             /**< SMBUS ARP UDID LSB Register,                      Address offset: 0xDC */
    __IM  uint32_t RESERVED[5];                /**< I2C RESERVED register,                            Address offset: 0xE0 */
    __IM  uint32_t COMP_PARAM_1;               /**< Component Parameter Register 1,                   Address offset: 0xF4 */
    __IM  uint32_t COMP_VERSION;               /**< Component Version Register,                       Address offset: 0xF8 */
    __IM  uint32_t COMP_TYPE;                  /**< Component Type Register,                          Address offset: 0xFC */
} i2c_regs_t;

/**
  * @brief I2S
  */
typedef struct
{
    __IOM uint32_t DATA_L;                  /**< Left TX/RX buffer register,                Address offset: 0x000 */
    __IOM uint32_t DATA_R;                  /**< Right TX/RX buffer register,               Address offset: 0x004 */
    __IOM uint32_t RXEN;                    /**< RX enable,                                 Address offset: 0x008 */
    __IOM uint32_t TXEN;                    /**< TX enable,                                 Address offset: 0x00C */
    __IOM uint32_t RXSIZE;                  /**< RX data size,                              Address offset: 0x010 */
    __IOM uint32_t TXSIZE;                  /**< TX data size,                              Address offset: 0x014 */
    __IM  uint32_t INTSTAT;                 /**< Interrupt status,                          Address offset: 0x018 */
    __IOM uint32_t INTMASK;                 /**< Interrupt mask,                            Address offset: 0x01C */
    __IM  uint32_t RXOVR;                   /**< RX FIFO overflow flag, read to clear,      Address offset: 0x020 */
    __IM  uint32_t TXOVR;                   /**< TX FIFO overflow flag, read to clear,      Address offset: 0x024 */
    __IOM uint32_t RXFIFO_TL;               /**< RX FIFO threshold level,                   Address offset: 0x028 */
    __IOM uint32_t TXFIFO_TL;               /**< TX FIFO threshold level,                   Address offset: 0x02C */
    __OM  uint32_t RXFIFO_FLUSH;            /**< RX FIFO flush,                             Address offset: 0x030 */
    __OM  uint32_t TXFIFO_FLUSH;            /**< TX FIFO flush,                             Address offset: 0x034 */
    __IM  uint32_t RESERVED[2];             /**< Reversed,                                  Address offset: 0x038 */
}I2S_CHANNEL_REGS;

typedef struct _i2s_regs
{
    __IOM uint32_t   ENABLE;                /**< I2S enable,                                Address offset: 0x000 */
    __IOM uint32_t   RBEN;                  /**< I2S receiver block enable,                 Address offset: 0x004 */
    __IOM uint32_t   TBEN;                  /**< I2S transmitter block enable,              Address offset: 0x008 */
    __IOM uint32_t   CLKEN;                 /**< Clock enable,                              Address offset: 0x00C */
    __IOM uint32_t   CLKCONFIG;             /**< Clock configuration,                       Address offset: 0x010 */
    __OM  uint32_t   RXFIFO_RST;            /**< Receiver block FIFO reset,                 Address offset: 0x014 */
    __OM  uint32_t   TXFIFO_RST;            /**< Transmitter block FIFO reset,              Address offset: 0x018 */
    __IM  uint32_t   RESERVED0;             /**< Reversed,                                  Address offset: 0x01C */
    I2S_CHANNEL_REGS I2S_CHANNEL[4];        /**< I2S channels registers,                    Address offset: 0x020 */
    __IM  uint32_t   RESERVED1[40];         /**< Reversed,                                  Address offset: 0x120 */
    __IM  uint32_t   RXDMA;                 /**< Receiver block DMA register,               Address offset: 0x1C0 */
    __OM  uint32_t   RXDMA_RST;             /**< Receiver block DMA reset,                  Address offset: 0x1C4 */
    __OM  uint32_t   TXDMA;                 /**< Transmitter block DMA register,            Address offset: 0x1C8 */
    __OM  uint32_t   TXDMA_RST;             /**< Transmitter block DMA reset,               Address offset: 0x1CC */
    __IM  uint32_t   RESERVED2[8];          /**< Reversed,                                  Address offset: 0x1D0 */
    __IM  uint32_t   I2S_PARAM2;            /**< I2S component parameter register 2,        Address offset: 0x1F0 */
    __IM  uint32_t   I2S_PARAM1;            /**< I2S component parameter register 1,        Address offset: 0x1F4 */
    __IM  uint32_t   I2S_VERSION;           /**< I2S component version,                     Address offset: 0x1F8 */
    __IM  uint32_t   I2S_TYPE;              /**< I2S component type,                        Address offset: 0x1FC */
} i2s_regs_t;

/**
  * @brief ISO7816
  */
typedef struct _ISO7816_regs
{
    __OM  uint32_t   CTRL;                  /**< ISO7816 control,                           Address offset: 0x000 */
    __IM  uint32_t   STAT;                  /**< ISO7816 status,                            Address offset: 0x004 */
    __IOM uint32_t   CLK_CFG;               /**< ISO7816 clock configuration,               Address offset: 0x008 */
    __IOM uint32_t   VCC_CFG;               /**< Supply voltage configuration,              Address offset: 0x00C */
    __IOM uint32_t   TIMES;                 /**< Times,                                     Address offset: 0x010 */
    __IOM uint32_t   DATA_CFG;              /**< Data configuration,                        Address offset: 0x014 */
    __IM  uint32_t   ADDR;                  /**< Address,                                   Address offset: 0x018 */
    __IOM uint32_t   STRT_ADDR;             /**< Start address,                             Address offset: 0x01C */
    __IOM uint32_t   RX_END_ADDR;           /**< RX end address,                            Address offset: 0x020 */
    __IOM uint32_t   TX_END_ADDR;           /**< TX end address,                            Address offset: 0x024 */
} iso7816_regs_t;

/**
  * @brief MCU_SUB
  */
typedef struct _mcu_sub_regs
{
    __IM  uint32_t SENSE_ADC_FIFO;          /**< MCU_SUB_REG_SENSE_ADC_FIFO,        Address offset: 0x000 */
    __IM  uint32_t RESERVED0[63];           /**< RESERVED,                          Address offset: 0x004 */
    __IOM uint32_t SENSE_FF_THRESH;         /**< MCU_SUB_REG_SENSE_FF_THRESH,       Address offset: 0x100 */
    __IM  uint32_t SENSE_ADC_STAT;          /**< MCU_SUB_REG_SENSE_ADC_STAT,        Address offset: 0x104 */
    __IM  uint32_t RESERVED1[62];           /**< RESERVED,                          Address offset: 0x108 */
    __IOM uint32_t COMM_TMR_DEEPSLPSTAT;    /**< MCU_SUB_REG_COMM_TMR_DEEPSLPSTAT,  Address offset: 0x200 */
    __IM  uint32_t RESERVED2;               /**< RESERVED,                          Address offset: 0x204 */
    __IOM uint32_t DPAD_RE_N_BUS;           /**< MCU_SUB_REG_DPAD_RE_N_BUS,         Address offset: 0x208 */
    __IM  uint32_t RESERVED3;               /**< RESERVED,                          Address offset: 0x20C */
    __IOM uint32_t DPAD_RTYP_BUS;           /**< MCU_SUB_REG_DPAD_RTYP_BUS,         Address offset: 0x210 */
    __IM  uint32_t RESERVED4;               /**< RESERVED,                          Address offset: 0x214 */
    __IOM uint32_t DPAD_IE_N_BUS;           /**< MCU_SUB_REG_DPAD_IE_N_BUS,         Address offset: 0x218 */
    __IM  uint32_t RESERVED5;               /**< RESERVED,                          Address offset: 0x21C */
    __IOM uint32_t MSIO_REG0;               /**< MCU_SUB_REG_MSIO_REG,              Address offset: 0x220 */
    __IOM uint32_t BLE_FERP_CTL;            /**< MCU_SUB_REG_BLE_FERP_CTL,          Address offset: 0x224 */
    __IOM uint32_t DMA_ACC_SEL;             /**< MCU_SUB_REG_DMA_ACC_SEL,           Address offset: 0x228 */
    __IOM uint32_t SECURITY_RESET;          /**< MCU_SUB_REG_SECURITY_RESET,        Address offset: 0x22C */
    __IOM uint32_t PMU_ID;                  /**< MCU_SUB_REG_PMU_ID,                Address offset: 0x230 */
    __IOM uint32_t PWR_AVG_CTL;             /**< MCU_SUB_REG_PWR_AVG_CTL_REG,       Address offset: 0x234 */
    __IOM uint32_t CLK_CAL_CTL[2];          /**< MCU_SUB_REG_CLK_CAL_CTL_REG0~1,    Address offset: 0x238 */
    __IOM uint32_t DPAD_MUX_CTL0_7;         /**< MCU_SUB_REG_DPAD_MUX_CTL_00_07,    Address offset: 0x240 */
    __IOM uint32_t DPAD_MUX_CTL8_15;        /**< MCU_SUB_REG_DPAD_MUX_CTL_08_15,    Address offset: 0x244 */
    __IOM uint32_t DPAD_MUX_CTL16_23;       /**< MCU_SUB_REG_DPAD_MUX_CTL_16_23,    Address offset: 0x248 */
    __IOM uint32_t DPAD_MUX_CTL24_31;       /**< MCU_SUB_REG_DPAD_MUX_CTL_24_31,    Address offset: 0x24C */
    __IM  uint32_t RESERVED6;               /**< RESERVED,                          Address offset: 0x250 */
    __IOM uint32_t EFUSE_PWR_DELTA[2];      /**< MCU_SUB_REG_EFUSE_PWR_DELTA0~1,    Address offset: 0x254 */
    __IM  uint32_t RESERVED7;               /**< RESERVED,                          Address offset: 0x250 */
    __IOM uint32_t EFUSE_PWR_CTRL[2];       /**< MCU_SUB_REG_EFUSE_PWR_CTRL0~1,     Address offset: 0x260 */
    __IOM uint32_t I2S_CLK_CFG;             /**< MCU_SUB_REG_I2S_CLK_CFG,           Address offset: 0x268 */
    __IM  uint32_t RESERVED8[5];            /**< RESERVED,                          Address offset: 0x26C */
    __IOM uint32_t MCU_SUB_REG;             /**< MCU_SUB_REG_MCU_SUB_REG,           Address offset: 0x280 */
    __IM  uint32_t RESERVED9[3];            /**< RESERVED,                          Address offset: 0x284 */
    __IOM uint32_t AON_PAD_MUX_CTL;         /**< MCU_SUB_REG_AON_PAD_MUX_CTL,       Address offset: 0x290 */
    __IOM uint32_t MSIO_PAD_MUX_CTL;        /**< MCU_SUB_REG_MSIO_PAD_MUX_CTL,      Address offset: 0x294 */
    __IM  uint32_t RESERVED10[2];           /**< RESERVED,                          Address offset: 0x298 */
    __IOM uint32_t MCU_SUBSYS_CG_CTRL[3];   /**< MCU_SUB_REG_MCU_SUBSYS_CG_CTRL0~2, Address offset: 0x2A0 */
    __IOM uint32_t MCU_PERIPH_CG;           /**< MCU_SUB_REG_MCU_PERIPH_CG,         Address offset: 0x2AC */
    __IOM uint32_t MCU_SUBSYS_CLK_CTRL;     /**< MCU_SUB_REG_MCU_SUBSYS_CLK_CTRL,   Address offset: 0x2B0 */
    __IM  uint32_t RESERVED11[3];           /**< RESERVED,                          Address offset: 0x2B4 */
    __IOM uint32_t BLE_DSLEEP_CORR_EN;      /**< MCU_SUB_REG_BLE_DSLEEP_CORR_EN,    Address offset: 0x2C0 */
    __IOM uint32_t BLE_DSLEEP_HW_TIM_CORR;  /**< MCU_SUB_REG_BLE_DSLEEP_HW_TIM_CORR,Address offset: 0x2C4 */
} mcu_sub_regs_t;

/**
  * @brief PKC
  */
typedef struct _pkc_regs
{
    __IOM uint32_t CTRL;            /**< PKC_REG_CTRL,          Address offset: 0x00 */
    __IOM uint32_t CONFIG0;         /**< PKC_REG_CONFIG0,       Address offset: 0x04 */
    __IOM uint32_t CONFIG1;         /**< PKC_REG_CONFIG1,       Address offset: 0x08 */
    __IOM uint32_t CONFIG2;         /**< PKC_REG_CONFIG2,       Address offset: 0x0C */
    __IOM uint32_t CONFIG3;         /**< PKC_REG_CONFIG3,       Address offset: 0x10 */
    __IOM uint32_t CONFIG4;         /**< PKC_REG_CONFIG4,       Address offset: 0x14 */
    __IOM uint32_t CONFIG5;         /**< PKC_REG_CONFIG5,       Address offset: 0x18 */
    __IOM uint32_t CONFIG6;         /**< PKC_REG_CONFIG6,       Address offset: 0x1C */
    __IOM uint32_t CONFIG7;         /**< PKC_REG_CONFIG7,       Address offset: 0x20 */
    __IOM uint32_t CONFIG8;         /**< PKC_REG_CONFIG8,       Address offset: 0x24 */
    __IOM uint32_t CONFIG9;         /**< PKC_REG_CONFIG9,       Address offset: 0x28 */
    __IOM uint32_t CONFIG10;        /**< PKC_REG_CONFIG10,      Address offset: 0x2C */
    __IOM uint32_t CONFIG11;        /**< PKC_REG_CONFIG11,      Address offset: 0x30 */
    __IOM uint32_t CONFIG12;        /**< PKC_REG_CONFIG12,      Address offset: 0x34 */
    __IOM uint32_t CONFIG13;        /**< PKC_REG_CONFIG13,      Address offset: 0x38 */
    __IM  uint32_t REVERSED0;       /**< PKC_REG_REVERSED,      Address offset: 0x3C */
    __IOM uint32_t SW_CTRL;         /**< PKC_REG_SW_CTRL,       Address offset: 0x40 */
    __IOM uint32_t SW_CONFIG0;      /**< PKC_REG_SW_CONFIG,     Address offset: 0x44 */
    __IOM uint32_t SW_CONFIG1;      /**< PKC_REG_SW_CONFIG,     Address offset: 0x48 */
    __IOM uint32_t SW_CONFIG2;      /**< PKC_REG_SW_CONFIG,     Address offset: 0x4C */
    __IOM uint32_t SW_CONFIG3;      /**< PKC_REG_SW_CONFIG,     Address offset: 0x50 */
    __IOM uint32_t SW_CONFIG4;      /**< PKC_REG_SW_CONFIG,     Address offset: 0x54 */
    __IOM uint32_t SW_CONFIG5;      /**< PKC_REG_SW_CONFIG,     Address offset: 0x58 */
    __IOM uint32_t SW_CONFIG6;      /**< PKC_REG_SW_CONFIG,     Address offset: 0x5C */
    __IOM uint32_t SW_CONFIG7;      /**< PKC_REG_SW_CONFIG,     Address offset: 0x60 */
    __IM  uint32_t SW_CONFIG8;      /**< PKC_REG_SW_CONFIG,     Address offset: 0x64 */
    __IOM uint32_t SW_CONFIG9;      /**< PKC_REG_SW_CONFIG,     Address offset: 0x68 */
    __IOM uint32_t SW_CONFIG10;     /**< PKC_REG_SW_CONFIG,     Address offset: 0x6C */
    __IOM uint32_t SW_CONFIG11;     /**< PKC_REG_SW_CONFIG,     Address offset: 0x70 */
    __IOM uint32_t SW_CONFIG12;     /**< PKC_REG_SW_CONFIG,     Address offset: 0x74 */
    __IOM uint32_t SW_CONFIG13;     /**< PKC_REG_SW_CONFIG,     Address offset: 0x78 */
    __IM  uint32_t REVERSED1;       /**< PKC_REG_REVERSED,      Address offset: 0x7C */
    __IOM uint32_t INTSTAT;         /**< PKC_REG_INT_STATUS,    Address offset: 0x80 */
    __IOM uint32_t INTEN;           /**< PKC_REG_INT_ENABLE,    Address offset: 0x84 */
    __IM  uint32_t WORKSTAT;        /**< PKC_REG_WORK_STATUS,   Address offset: 0x88 */
    __IM  uint32_t REVERSED2;       /**< PKC_REG_REVERSED,      Address offset: 0x8C */
    __IOM uint32_t DUMMY0;          /**< PKC_REG_DUMMY0,        Address offset: 0x90 */
    __IOM uint32_t DUMMY1;          /**< PKC_REG_DUMMY1,        Address offset: 0x94 */
    __IOM uint32_t DUMMY2;          /**< PKC_REG_DUMMY2,        Address offset: 0x98 */
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
  * @brief SSI
  */
typedef struct _ssi_regs
{
    __IOM uint32_t CTRL0;           /**< SSI_REG_CTRL0,         Address offset: 0x00 */
    __IOM uint32_t CTRL1;           /**< SSI_REG_CTRL1,         Address offset: 0x04 */
    __IOM uint32_t SSI_EN;          /**< SSI_REG_SSI_EN,        Address offset: 0x08 */
    __IOM uint32_t MWC;             /**< SSI_REG_MWC,           Address offset: 0x0C */
    __IOM uint32_t SE;              /**< SSI_REG_SE,            Address offset: 0x10 */
    __IOM uint32_t BAUD;            /**< SSI_REG_BAUD,          Address offset: 0x14 */
    __IOM uint32_t TX_FTL;          /**< SSI_REG_TX_FTL,        Address offset: 0x18 */
    __IOM uint32_t RX_FTL;          /**< SSI_REG_RX_FTL,        Address offset: 0x1C */
    __IM  uint32_t TX_FL;           /**< SSI_REG_TX_FL,         Address offset: 0x20 */
    __IM  uint32_t RX_FL;           /**< SSI_REG_RX_FL,         Address offset: 0x24 */
    __IM  uint32_t STAT;            /**< SSI_REG_STAT,          Address offset: 0x28 */
    __IOM uint32_t INTMASK;         /**< SSI_REG_INT_MASK,      Address offset: 0x2C */
    __IM  uint32_t INTSTAT;         /**< SSI_REG_INT_STAT,      Address offset: 0x30 */
    __IM  uint32_t RAW_INTSTAT;     /**< SSI_REG_RAW_INT_STAT,  Address offset: 0x34 */
    __IM  uint32_t TXOIC;           /**< SSI_REG_TXOIC,         Address offset: 0x38 */
    __IM  uint32_t RXOIC;           /**< SSI_REG_RXOIC,         Address offset: 0x3C */
    __IM  uint32_t RXUIC;           /**< SSI_REG_RXUIC,         Address offset: 0x40 */
    __IM  uint32_t MSTIC;           /**< SSI_REG_MSTIC,         Address offset: 0x44 */
    __IM  uint32_t INTCLR;          /**< SSI_REG_INT_CLR,       Address offset: 0x48 */
    __IOM uint32_t DMAC;            /**< SSI_REG_DMAC,          Address offset: 0x4C */
    __IOM uint32_t DMA_TDL;         /**< SSI_REG_DMA_TDL,       Address offset: 0x50 */
    __IOM uint32_t DMA_RDL;         /**< SSI_REG_DMA_RDL,       Address offset: 0x54 */
    __IM  uint32_t ID;              /**< SSI_REG_ID,            Address offset: 0x58 */
    __IM  uint32_t VERSION_ID;      /**< SSI_REG_VERSION_ID,    Address offset: 0x5C */
    __IOM uint32_t DATA;            /**< SSI_REG_DATA,          Address offset: 0x60 */
    __IM  uint32_t REVERSED[35];    /**< SSI_REG_REVERSED,      Address offset: 0x64 */
    __IOM uint32_t RX_SAMPLE_DLY;   /**< SSI_REG_RX_SAMPLE_DLY, Address offset: 0xF0 */
    __IOM uint32_t SPI_CTRL0;       /**< SSI_REG_SPI_CTRL0,     Address offset: 0xF4 */
} ssi_regs_t;

/**
  * @brief TIM
  */
typedef struct _timer_regs
{
    __IOM uint32_t CTRL;            /**< TIMER control register,          Address offset: 0x00 */
    __IOM uint32_t VALUE;           /**< TIMER counter value register,    Address offset: 0x04 */
    __IOM uint32_t RELOAD;          /**< TIMER auto-reload register,      Address offset: 0x08 */
    __IOM uint32_t INTSTAT;         /**< TIMER interrupt status register, Address offset: 0x0C */
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
    __IM  uint32_t  RESERVED1[483];
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
#define FLASH_BASE              ((uint32_t)0x01000000UL)
#define SRAM_BASE               ((uint32_t)0x30000000UL)
#define PERIPH_BASE             ((uint32_t)0xA0000000UL)

#define TIMER0_BASE             (PERIPH_BASE + 0x00000000UL)
#define TIMER1_BASE             (PERIPH_BASE + 0x00001000UL)
#define DUAL_TIMER0_BASE        (PERIPH_BASE + 0x00002000UL)
#define DUAL_TIMER1_BASE        (PERIPH_BASE + 0x00002020UL)
#define WDT_BASE                (PERIPH_BASE + 0x00008000UL)
#define SPIM_BASE               (PERIPH_BASE + 0x0000C000UL)
#define SPIS_BASE               (PERIPH_BASE + 0x0000C100UL)
#define QSPI0_BASE              (PERIPH_BASE + 0x0000C200UL)
#define I2C0_BASE               (PERIPH_BASE + 0x0000C300UL)
#define I2C1_BASE               (PERIPH_BASE + 0x0000C400UL)
#define AON_BASE                (PERIPH_BASE + 0x0000C500UL)
#define UART0_BASE              (PERIPH_BASE + 0x0000C600UL)
#define UART1_BASE              (PERIPH_BASE + 0x0000C700UL)
#define QSPI1_BASE              (PERIPH_BASE + 0x0000C800UL)
#define PWM0_BASE               (PERIPH_BASE + 0x0000C900UL)
#define I2S_M_BASE              (PERIPH_BASE + 0x0000CA00UL)
#define PWM1_BASE               (PERIPH_BASE + 0x0000CC00UL)
#define XQSPI_BASE              (PERIPH_BASE + 0x0000D000UL)
#define MCU_SUB_BASE            (PERIPH_BASE + 0x0000E000UL)
#define I2S_S_BASE              (PERIPH_BASE + 0x0000F000UL)
#define ISO7816_BASE            (PERIPH_BASE + 0x0000F200UL)
#define GPIO0_BASE              (PERIPH_BASE + 0x00010000UL)
#define GPIO1_BASE              (PERIPH_BASE + 0x00011000UL)
#define GPIO2_BASE              (PERIPH_BASE + 0x00012000UL)
#define DMA_BASE                (PERIPH_BASE + 0x00013000UL)
#define PKC_BASE                (PERIPH_BASE + 0x00014000UL)
#define AES_BASE                (PERIPH_BASE + 0x00015400UL)
#define HMAC_BASE               (PERIPH_BASE + 0x00015800UL)
#define EFUSE_BASE              (PERIPH_BASE + 0x00016400UL)
#define RNG_BASE                (PERIPH_BASE + 0x00017800UL)

#define PKC_SPRAM_BASE          (PKC_BASE    + 0x00000800UL)
#define KRAM_BASE               (PERIPH_BASE + 0x00017000UL)
#define EFUSE_STORAGE_BASE      (PERIPH_BASE + 0x00016000UL)

/** @} */ /* End of group Peripheral_memory_map */


/* ================================================================================================================= */
/* ================                             Peripheral declaration                              ================ */
/* ================================================================================================================= */

/** @addtogroup Peripheral_declaration
  * @{
  */

#define TIMER0                  ((timer_regs_t *)TIMER0_BASE)
#define TIMER1                  ((timer_regs_t *)TIMER1_BASE)
#define DUAL_TIMER0             ((dual_timer_regs_t *)DUAL_TIMER0_BASE)
#define DUAL_TIMER1             ((dual_timer_regs_t *)DUAL_TIMER1_BASE)
#define WDT                     ((wdt_regs_t *)WDT_BASE)
#define SPIM                    ((ssi_regs_t *)SPIM_BASE)
#define SPIS                    ((ssi_regs_t *)SPIS_BASE)
#define QSPI0                   ((ssi_regs_t *)QSPI0_BASE)
#define QSPI1                   ((ssi_regs_t *)QSPI1_BASE)
#define I2C0                    ((i2c_regs_t *)I2C0_BASE)
#define I2C1                    ((i2c_regs_t *)I2C1_BASE)
#define AON                     ((aon_regs_t *)AON_BASE)
#define UART0                   ((uart_regs_t *)UART0_BASE)
#define UART1                   ((uart_regs_t *)UART1_BASE)
#define PWM0                    ((pwm_regs_t *)PWM0_BASE)
#define PWM1                    ((pwm_regs_t *)PWM1_BASE)
#define I2S_M                   ((i2s_regs_t *)I2S_M_BASE)
#define I2S_S                   ((i2s_regs_t *)I2S_S_BASE)
#define ISO7816                 ((iso7816_regs_t *)ISO7816_BASE)
#define XQSPI                   ((xqspi_regs_t *)XQSPI_BASE)
#define MCU_SUB                 ((mcu_sub_regs_t *)MCU_SUB_BASE)
#define GPIO0                   ((gpio_regs_t *)GPIO0_BASE)
#define GPIO1                   ((gpio_regs_t *)GPIO1_BASE)
#define GPIO2                   ((gpio_regs_t *)GPIO2_BASE)
#define DMA                     ((dma_regs_t *)DMA_BASE)
#define PKC                     ((pkc_regs_t *)PKC_BASE)
#define AES                     ((aes_regs_t *)AES_BASE)
#define HMAC                    ((hmac_regs_t *)HMAC_BASE)
#define EFUSE                   ((efuse_regs_t *)EFUSE_BASE)
#define RNG                     ((rng_regs_t *)RNG_BASE)

/** @} */ /* End of group Peripheral_declaration */

/** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/* ================================================================================================================= */
/* ================                                        AES                                      ================ */
/* ================================================================================================================= */

/*******************  Bit definition for AES_CTRL register  *******************/
#define AES_CTRL_ENABLE_Pos                                 (0U)
#define AES_CTRL_ENABLE_Len                                 (1U)
#define AES_CTRL_ENABLE_Msk                                 (1U << AES_CTRL_ENABLE_Pos)
#define AES_CTRL_ENABLE                                     AES_CTRL_ENABLE_Msk

#define AES_CTRL_START_NORMAL_Pos                           (1U)
#define AES_CTRL_START_NORMAL_Len                           (1U)
#define AES_CTRL_START_NORMAL_Msk                           (1U << AES_CTRL_START_NORMAL_Pos)
#define AES_CTRL_START_NORMAL                               AES_CTRL_START_NORMAL_Msk

#define AES_CTRL_START_DMA_Pos                              (2U)
#define AES_CTRL_START_DMA_Len                              (1U)
#define AES_CTRL_START_DMA_Msk                              (1U << AES_CTRL_START_DMA_Pos)
#define AES_CTRL_START_DMA                                  AES_CTRL_START_DMA_Msk

#define AES_CTRL_ENABLE_RKEY_Pos                            (3U)
#define AES_CTRL_ENABLE_RKEY_Len                            (1U)
#define AES_CTRL_ENABLE_RKEY_Msk                            (1U << AES_CTRL_ENABLE_RKEY_Pos)
#define AES_CTRL_ENABLE_RKEY                                AES_CTRL_ENABLE_RKEY_Msk

/*******************  Bit definition for AES_CONFIG register  *******************/
#define AES_CONFIG_KEYMODE_Pos                              (0U)
#define AES_CONFIG_KEYMODE_Len                              (2U)
#define AES_CONFIG_KEYMODE_Msk                              (3U << AES_CONFIG_KEYMODE_Pos)
#define AES_CONFIG_KEYMODE                                  AES_CONFIG_KEYMODE_Msk

#define AES_CONFIG_ENABLE_FULLMASK_Pos                      (3U)
#define AES_CONFIG_ENABLE_FULLMASK_Len                      (1U)
#define AES_CONFIG_ENABLE_FULLMASK_Msk                      (1U << AES_CONFIG_ENABLE_FULLMASK_Pos)
#define AES_CONFIG_ENABLE_FULLMASK                          AES_CONFIG_ENABLE_FULLMASK_Msk

#define AES_CONFIG_ENABLE_ENCRYPTION_Pos                    (4U)
#define AES_CONFIG_ENABLE_ENCRYPTION_Len                    (1U)
#define AES_CONFIG_ENABLE_ENCRYPTION_Msk                    (1U << AES_CONFIG_ENABLE_ENCRYPTION_Pos)
#define AES_CONFIG_ENABLE_ENCRYPTION                        AES_CONFIG_ENABLE_ENCRYPTION_Msk

#define AES_CONFIG_LOADSEED_Pos                             (5U)
#define AES_CONFIG_LOADSEED_Len                             (1U)
#define AES_CONFIG_LOADSEED_Msk                             (1U << AES_CONFIG_LOADSEED_Pos)
#define AES_CONFIG_LOADSEED                                 AES_CONFIG_LOADSEED_Msk

#define AES_CONFIG_FIRSTBLOCK_Pos                           (6U)
#define AES_CONFIG_FIRSTBLOCK_Len                           (1U)
#define AES_CONFIG_FIRSTBLOCK_Msk                           (1U << AES_CONFIG_FIRSTBLOCK_Pos)
#define AES_CONFIG_FIRSTBLOCK                               AES_CONFIG_FIRSTBLOCK_Msk

#define AES_CONFIG_ENDIAN_Pos                               (7U)
#define AES_CONFIG_ENDIAN_Len                               (1U)
#define AES_CONFIG_ENDIAN_Msk                               (1U << AES_CONFIG_ENDIAN_Pos)
#define AES_CONFIG_ENDIAN                                   AES_CONFIG_ENDIAN_Msk

#define AES_CONFIG_OPMODE_Pos                               (8U)
#define AES_CONFIG_OPMODE_Len                               (3U)
#define AES_CONFIG_OPMODE_Msk                               (7U << AES_CONFIG_OPMODE_Pos)
#define AES_CONFIG_OPMODE                                   AES_CONFIG_OPMODE_Msk

#define AES_CONFIG_KEYTYPE_Pos                              (11U)
#define AES_CONFIG_KEYTYPE_Len                              (2U)
#define AES_CONFIG_KEYTYPE_Msk                              (3U << AES_CONFIG_KEYTYPE_Pos)
#define AES_CONFIG_KEYTYPE                                  AES_CONFIG_KEYTYPE_Msk

/*******************  Bit definition for AES_STATUS register  *******************/
#define AES_STATUS_READY_Pos                                (0U)
#define AES_STATUS_READY_Len                                (1U)
#define AES_STATUS_READY_Msk                                (1U << AES_STATUS_READY_Pos)
#define AES_STATUS_READY                                    AES_STATUS_READY_Msk

#define AES_STATUS_TRANSDONE_Pos                            (1U)
#define AES_STATUS_TRANSDONE_Len                            (1U)
#define AES_STATUS_TRANSDONE_Msk                            (1U << AES_STATUS_TRANSDONE_Pos)
#define AES_STATUS_TRANSDONE                                AES_STATUS_TRANSDONE_Msk

#define AES_STATUS_TRANSERR_Pos                             (2U)
#define AES_STATUS_TRANSERR_Len                             (1U)
#define AES_STATUS_TRANSERR_Msk                             (1U << AES_STATUS_TRANSERR_Pos)
#define AES_STATUS_TRANSERR                                 AES_STATUS_TRANSERR_Msk

#define AES_STATUS_KEYVALID_Pos                             (3U)
#define AES_STATUS_KEYVALID_Len                             (1U)
#define AES_STATUS_KEYVALID_Msk                             (1U << AES_STATUS_KEYVALID_Pos)
#define AES_STATUS_KEYVALID                                 AES_STATUS_KEYVALID_Msk

/*******************  Bit definition for AES_INTERRUPT register  *******************/
#define AES_INTERRUPT_DONE_Pos                              (0U)
#define AES_INTERRUPT_DONE_Len                              (1U)
#define AES_INTERRUPT_DONE_Msk                              (1U << AES_INTERRUPT_DONE_Pos)
#define AES_INTERRUPT_DONE                                  AES_INTERRUPT_DONE_Msk

#define AES_INTERRUPT_ENABLE_Pos                            (1U)
#define AES_INTERRUPT_ENABLE_Len                            (1U)
#define AES_INTERRUPT_ENABLE_Msk                            (1U << AES_INTERRUPT_ENABLE_Pos)
#define AES_INTERRUPT_ENABLE                                AES_INTERRUPT_ENABLE_Msk

/*******************  Bit definition for AES_TRAN_SIZE register  *******************/
#define AES_TRAN_SIZE_Pos                                   (0U)
#define AES_TRAN_SIZE_Len                                   (15U)
#define AES_TRAN_SIZE_Msk                                   (0x00007FFFU)
#define AES_TRAN_SIZE                                       AES_TRAN_SIZE_Msk

/*******************  Bit definition for AES_RSTART_ADDR register  *******************/
#define AES_RSTART_ADDR_Pos                                 (0U)
#define AES_RSTART_ADDR_Len                                 (32U)
#define AES_RSTART_ADDR_Msk                                 (0xFFFFFFFFU)
#define AES_RSTART_ADDR                                     AES_RSTART_ADDR_Msk

/*******************  Bit definition for AES_WSTART_ADDR register  *******************/
#define AES_WSTART_ADDR_Pos                                 (0U)
#define AES_WSTART_ADDR_Len                                 (32U)
#define AES_WSTART_ADDR_Msk                                 (0xFFFFFFFFU)
#define AES_WSTART_ADDR                                     AES_WSTART_ADDR_Msk

/*******************  Bit definition for AES_KEY_ADDR register  *******************/
#define AES_KEY_ADDR_Pos                                    (0U)
#define AES_KEY_ADDR_Len                                    (32U)
#define AES_KEY_ADDR_Msk                                    (0xFFFFFFFFU)
#define AES_KEY_ADDR                                        AES_KEY_ADDR_Msk

/*******************  Bit definition for AES_DATA_OUT register  *******************/
#define AES_DATA_OUT_Pos                                    (0U)
#define AES_DATA_OUT_Len                                    (32U)
#define AES_DATA_OUT_Msk                                    (0xFFFFFFFFU)
#define AES_DATA_OUT                                        AES_DATA_OUT_Msk

/*******************  Bit definition for AES_KEY register  *******************/
#define AES_KEY_Pos                                         (0U)
#define AES_KEY_Len                                         (32U)
#define AES_KEY_Msk                                         (0xFFFFFFFFU)
#define AES_KEY                                             AES_KEY_Msk

/*******************  Bit definition for AES_SEED_IN register  *******************/
#define AES_SEED_IN_Pos                                     (0U)
#define AES_SEED_IN_Len                                     (32U)
#define AES_SEED_IN_Msk                                     (0xFFFFFFFFU)
#define AES_SEED_IN                                         AES_SEED_IN_Msk

/*******************  Bit definition for AES_SEED_OUT register  *******************/
#define AES_SEED_OUT_Pos                                    (0U)
#define AES_SEED_OUT_Len                                    (32U)
#define AES_SEED_OUT_Msk                                    (0xFFFFFFFFU)
#define AES_SEED_OUT                                        AES_SEED_OUT_Msk

/*******************  Bit definition for AES_SEED_IMASK register  *******************/
#define AES_SEED_IMASK_Pos                                  (0U)
#define AES_SEED_IMASK_Len                                  (32U)
#define AES_SEED_IMASK_Msk                                  (0xFFFFFFFFU)
#define AES_SEED_IMASK                                      AES_SEED_IMASK_Msk

/*******************  Bit definition for AES_SEED_OSBOX register  *******************/
#define AES_SEED_OSBOX_Pos                                  (0U)
#define AES_SEED_OSBOX_Len                                  (32U)
#define AES_SEED_OSBOX_Msk                                  (0xFFFFFFFFU)
#define AES_SEED_OSBOX                                      AES_SEED_OSBOX_Msk

/*******************  Bit definition for AES_VECTOR_INIT register  *******************/
#define AES_VECTOR_INIT_Pos                                 (0U)
#define AES_VECTOR_INIT_Len                                 (32U)
#define AES_VECTOR_INIT_Msk                                 (0xFFFFFFFFU)
#define AES_VECTOR_INIT                                     AES_VECTOR_INIT_Msk

/*******************  Bit definition for AES_DATA_IN register  *******************/
#define AES_DATA_IN_Pos                                     (0U)
#define AES_DATA_IN_Len                                     (32U)
#define AES_DATA_IN_Msk                                     (0xFFFFFFFFU)
#define AES_DATA_IN                                         AES_DATA_IN_Msk


/* ================================================================================================================= */
/* ================                                        AON                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for AON_REG_PWR_RET01 register  **********/
#define AON_PWR_REG01_WAKE_UP_SEL_Pos                       (24U)
#define AON_PWR_REG01_WAKE_UP_SEL_Len                       (6U)
#define AON_PWR_REG01_WAKE_UP_SEL_Msk                       (0x3FU << AON_PWR_REG01_WAKE_UP_SEL_Pos)
#define AON_PWR_REG01_WAKE_UP_SEL                           AON_PWR_REG01_WAKE_UP_SEL_Msk
#define AON_PWR_REG01_WAKE_UP_SEL_TIMER                     (0x1U << AON_PWR_REG01_WAKE_UP_SEL_Pos)
#define AON_PWR_REG01_WAKE_UP_SEL_EXTWKUP                   (0x2U << AON_PWR_REG01_WAKE_UP_SEL_Pos)
#define AON_PWR_REG01_WAKE_UP_SEL_BLE                       (0x4U << AON_PWR_REG01_WAKE_UP_SEL_Pos)
#define AON_PWR_REG01_WAKE_UP_SEL_CALENDAR                  (0x8U << AON_PWR_REG01_WAKE_UP_SEL_Pos)
#define AON_PWR_REG01_WAKE_UP_SEL_PMU_BOD_FEDGE             (0x10U << AON_PWR_REG01_WAKE_UP_SEL_Pos)
#define AON_PWR_REG01_WAKE_UP_SEL_MSIO_COMP                 (0x20U << AON_PWR_REG01_WAKE_UP_SEL_Pos)

#define AON_PWR_REG01_SMC_WAKEUP_REQ_Pos                    (22U)
#define AON_PWR_REG01_SMC_WAKEUP_REQ_Len                    (1U)
#define AON_PWR_REG01_SMC_WAKEUP_REQ_Msk                    (0x1U << AON_PWR_REG01_SMC_WAKEUP_REQ_Pos)
#define AON_PWR_REG01_SMC_WAKEUP_REQ                        AON_PWR_REG01_SMC_WAKEUP_REQ_Msk

#define AON_PWR_REG01_XF_TAG_RET_Pos                        (21U)
#define AON_PWR_REG01_XF_TAG_RET_Len                        (1U)
#define AON_PWR_REG01_XF_TAG_RET_Msk                        (0x1U << AON_PWR_REG01_XF_TAG_RET_Pos)
#define AON_PWR_REG01_XF_TAG_RET                            AON_PWR_REG01_XF_TAG_RET_Msk

#define AON_PWR_REG01_XF_SCK_CLK_SEL_Pos                    (18U)
#define AON_PWR_REG01_XF_SCK_CLK_SEL_Len                    (3U)
#define AON_PWR_REG01_XF_SCK_CLK_SEL_Msk                    (0x7U << AON_PWR_REG01_XF_SCK_CLK_SEL_Pos)
#define AON_PWR_REG01_XF_SCK_CLK_SEL                        AON_PWR_REG01_XF_SCK_CLK_SEL_Msk

#define AON_PWR_REG01_SWD_ENABLE_Pos                        (17U)
#define AON_PWR_REG01_SWD_ENABLE_Len                        (1U)
#define AON_PWR_REG01_SWD_ENABLE_Msk                        (0x7U << AON_PWR_REG01_SWD_ENABLE_Pos)
#define AON_PWR_REG01_SWD_ENABLE                            AON_PWR_REG01_SWD_ENABLE_Msk

#define AON_PWR_REG01_BM_REMAP_Pos                          (13U)
#define AON_PWR_REG01_BM_REMAP_Len                          (4U)
#define AON_PWR_REG01_BM_REMAP_Msk                          (0xFU << AON_PWR_REG01_BM_REMAP_Pos)
#define AON_PWR_REG01_BM_REMAP                              AON_PWR_REG01_BM_REMAP_Msk

#define AON_PWR_REG01_COMM_CORE_RST_N_Pos                   (12U)
#define AON_PWR_REG01_COMM_CORE_RST_N_Len                   (1U)
#define AON_PWR_REG01_COMM_CORE_RST_N_Msk                   (0x1U << AON_PWR_REG01_COMM_CORE_RST_N_Pos)
#define AON_PWR_REG01_COMM_CORE_RST_N                       AON_PWR_REG01_COMM_CORE_RST_N_Msk

#define AON_PWR_REG01_COMM_TIMER_RST_N_Pos                  (11U)
#define AON_PWR_REG01_COMM_TIMER_RST_N_Len                  (1U)
#define AON_PWR_REG01_COMM_TIMER_RST_N_Msk                  (0x1U << AON_PWR_REG01_COMM_TIMER_RST_N_Pos)
#define AON_PWR_REG01_COMM_TIMER_RST_N                      AON_PWR_REG01_COMM_TIMER_RST_N_Msk

#define AON_PWR_REG01_ISO_EN_PD_COMM_TIMER_Pos              (9U)
#define AON_PWR_REG01_ISO_EN_PD_COMM_TIMER_Len              (1U)
#define AON_PWR_REG01_ISO_EN_PD_COMM_TIMER_Msk              (0x1U << AON_PWR_REG01_ISO_EN_PD_COMM_TIMER_Pos)
#define AON_PWR_REG01_ISO_EN_PD_COMM_TIMER                  AON_PWR_REG01_ISO_EN_PD_COMM_TIMER_Msk

#define AON_PWR_REG01_ISO_EN_PD_COMM_CORE_Pos               (8U)
#define AON_PWR_REG01_ISO_EN_PD_COMM_CORE_Len               (1U)
#define AON_PWR_REG01_ISO_EN_PD_COMM_CORE_Msk               (0x1U << AON_PWR_REG01_ISO_EN_PD_COMM_CORE_Pos)
#define AON_PWR_REG01_ISO_EN_PD_COMM_CORE                   AON_PWR_REG01_ISO_EN_PD_COMM_CORE_Msk

#define AON_PWR_REG01_PWR_EN_PD_COMM_TIMER_Pos              (7U)
#define AON_PWR_REG01_PWR_EN_PD_COMM_TIMER_Len              (1U)
#define AON_PWR_REG01_PWR_EN_PD_COMM_TIMER_Msk              (0x1U << AON_PWR_REG01_PWR_EN_PD_COMM_TIMER_Pos)
#define AON_PWR_REG01_PWR_EN_PD_COMM_TIMER                  AON_PWR_REG01_PWR_EN_PD_COMM_TIMER_Msk

#define AON_PWR_REG01_PWR_EN_PD_COMM_CORE_Pos               (6U)
#define AON_PWR_REG01_PWR_EN_PD_COMM_CORE_Len               (1U)
#define AON_PWR_REG01_PWR_EN_PD_COMM_CORE_Msk               (0x1U << AON_PWR_REG01_PWR_EN_PD_COMM_CORE_Pos)
#define AON_PWR_REG01_PWR_EN_PD_COMM_CORE                   AON_PWR_REG01_PWR_EN_PD_COMM_CORE_Msk

#define AON_PWR_REG01_EFLASH_PAD_EN_Pos                     (5U)
#define AON_PWR_REG01_EFLASH_PAD_EN_Len                     (1U)
#define AON_PWR_REG01_EFLASH_PAD_EN_Msk                     (0x1U << AON_PWR_REG01_EFLASH_PAD_EN_Pos)
#define AON_PWR_REG01_EFLASH_PAD_EN                         AON_PWR_REG01_EFLASH_PAD_EN_Msk

#define AON_PWR_REG01_XO_2MHZ_ENA_Pos                       (4U)
#define AON_PWR_REG01_XO_2MHZ_ENA_Len                       (1U)
#define AON_PWR_REG01_XO_2MHZ_ENA_Msk                       (0x1U << AON_PWR_REG01_XO_2MHZ_ENA_Pos)
#define AON_PWR_REG01_XO_2MHZ_ENA                           AON_PWR_REG01_XO_2MHZ_ENA_Msk

#define AON_PWR_REG01_XF_XO_DIV1_Pos                        (3U)
#define AON_PWR_REG01_XF_XO_DIV1_Len                        (1U)
#define AON_PWR_REG01_XF_XO_DIV1_Msk                        (0x1U << AON_PWR_REG01_XF_XO_DIV1_Pos)
#define AON_PWR_REG01_XF_XO_DIV1                            AON_PWR_REG01_XF_XO_DIV1_Msk

#define AON_PWR_REG01_SYS_CLK_SEL_Pos                       (0U)
#define AON_PWR_REG01_SYS_CLK_SEL_Len                       (3U)
#define AON_PWR_REG01_SYS_CLK_SEL_Msk                       (0x7U << AON_PWR_REG01_SYS_CLK_SEL_Pos)
#define AON_PWR_REG01_SYS_CLK_SEL                           AON_PWR_REG01_SYS_CLK_SEL_Msk

/*******************  Bit definition for AON_REG_SNSADC_CFG register  **********/
#define AON_SNSADC_CFG_SNSADC_REG4_Pos                      (24U)
#define AON_SNSADC_CFG_SNSADC_REG4_Len                      (8U)
#define AON_SNSADC_CFG_SNSADC_REG4_Msk                      (0xFFU << AON_SNSADC_CFG_SNSADC_REG4_Pos)
#define AON_SNSADC_CFG_SNSADC_REG4                          AON_SNSADC_CFG_SNSADC_REG4_Msk
#define AON_SNSADC_CFG_MAS_RST_Pos                          (31U)
#define AON_SNSADC_CFG_MAS_RST_Msk                          (0x1U <<  AON_SNSADC_CFG_MAS_RST_Pos)
#define AON_SNSADC_CFG_EN_Pos                               (30U)
#define AON_SNSADC_CFG_EN_Msk                               (0x1U <<  AON_SNSADC_CFG_EN_Pos)
#define AON_SNSADC_CFG_REF_SEL_Pos                          (27U)
#define AON_SNSADC_CFG_REF_SEL_Msk                          (0x7U <<  AON_SNSADC_CFG_REF_SEL_Pos)
#define AON_SNSADC_CFG_REF_HP_Pos                           (24U)
#define AON_SNSADC_CFG_REF_HP_Msk                           (0x7U <<  AON_SNSADC_CFG_REF_HP_Pos)

#define AON_SNSADC_CFG_SNSADC_REG3_Pos                      (16U)
#define AON_SNSADC_CFG_SNSADC_REG3_Len                      (8U)
#define AON_SNSADC_CFG_SNSADC_REG3_Msk                      (0xFFU << AON_SNSADC_CFG_SNSADC_REG3_Pos)
#define AON_SNSADC_CFG_SNSADC_REG3                          AON_SNSADC_CFG_SNSADC_REG3_Msk
#define AON_SNSADC_CFG_CHN_P_Pos                            (19U)
#define AON_SNSADC_CFG_CHN_P_Msk                            (0x7U <<  AON_SNSADC_CFG_CHN_P_Pos)
#define AON_SNSADC_CFG_CHN_N_Pos                            (16U)
#define AON_SNSADC_CFG_CHN_N_Msk                            (0x7U <<  AON_SNSADC_CFG_CHN_N_Pos)

#define AON_SNSADC_CFG_SNSADC_REG2_Pos                      (8U)
#define AON_SNSADC_CFG_SNSADC_REG2_Len                      (8U)
#define AON_SNSADC_CFG_SNSADC_REG2_Msk                      (0xFFU << AON_SNSADC_CFG_SNSADC_REG2_Pos)
#define AON_SNSADC_CFG_SNSADC_REG2                          AON_SNSADC_CFG_SNSADC_REG2_Msk
#define AON_SNSADC_CFG_TEMP_EN_Pos                          (15U)
#define AON_SNSADC_CFG_TEMP_EN_Msk                          (0x1U <<  AON_SNSADC_CFG_TEMP_EN_Pos)
#define AON_SNSADC_CFG_VBAT_EN_Pos                          (14U)
#define AON_SNSADC_CFG_VBAT_EN_Msk                          (0x1U <<  AON_SNSADC_CFG_VBAT_EN_Pos)
#define AON_SNSADC_CFG_SINGLE_EN_Pos                        (13U)
#define AON_SNSADC_CFG_SINGLE_EN_Msk                        (0x1U <<  AON_SNSADC_CFG_SINGLE_EN_Pos)
#define AON_SNSADC_CFG_OFS_CAL_EN_Pos                       (12U)
#define AON_SNSADC_CFG_OFS_CAL_EN_Msk                       (0x1U <<  AON_SNSADC_CFG_OFS_CAL_EN_Pos)
#define AON_SNSADC_CFG_DYMAMIC_Pos                          (8U)
#define AON_SNSADC_CFG_DYMAMIC_Msk                          (0x7U <<  AON_SNSADC_CFG_DYMAMIC_Pos)

#define AON_SNSADC_CFG_SNSADC_REG1_Pos                      (0U)
#define AON_SNSADC_CFG_SNSADC_REG1_Len                      (8U)
#define AON_SNSADC_CFG_SNSADC_REG1_Msk                      (0xFFU << AON_SNSADC_CFG_SNSADC_REG1_Pos)
#define AON_SNSADC_CFG_SNSADC_REG1                          AON_SNSADC_CFG_SNSADC_REG1_Msk
#define AON_SNSADC_CFG_REF_VALUE_Pos                        (0U)
#define AON_SNSADC_CFG_REF_VALUE_Msk                        (0xFU <<  AON_SNSADC_CFG_REF_VALUE_Pos)

/*******************  Bit definition for AON_REG_RF_REG_0 register  **********/
#define AON_RF_REG_0_IO_LDO_REG1_Pos                        (24U)
#define AON_RF_REG_0_IO_LDO_REG1_Len                        (8U)
#define AON_RF_REG_0_IO_LDO_REG1_Msk                        (0xFFU << AON_RF_REG_0_IO_LDO_REG1_Pos)
#define AON_RF_REG_0_IO_LDO_REG1                            AON_RF_REG_0_IO_LDO_REG1_Msk

#define AON_RF_REG_0_LPD_REG2_Pos                           (16U)
#define AON_RF_REG_0_LPD_REG2_Len                           (8U)
#define AON_RF_REG_0_LPD_REG2_Msk                           (0xFFU << AON_RF_REG_0_LPD_REG2_Pos)
#define AON_RF_REG_0_LPD_REG2                               AON_RF_REG_0_LPD_REG2_Msk

#define AON_RF_REG_0_LPD_REG1_Pos                           (8U)
#define AON_RF_REG_0_LPD_REG1_Len                           (8U)
#define AON_RF_REG_0_LPD_REG1_Msk                           (0xFFU << AON_RF_REG_0_LPD_REG1_Pos)
#define AON_RF_REG_0_LPD_REG1                               AON_RF_REG_0_LPD_REG1_Msk

#define AON_RF_REG_0_RTC_REG1_Pos                           (0U)
#define AON_RF_REG_0_RTC_REG1_Len                           (8U)
#define AON_RF_REG_0_RTC_REG1_Msk                           (0xFFU << AON_RF_REG_0_RTC_REG1_Pos)
#define AON_RF_REG_0_RTC_REG1                               AON_RF_REG_0_RTC_REG1_Msk

#define AON_RF_REG_0_DYN_CLK_CTRL_Pos                       (16U)
#define AON_RF_REG_0_DYN_CLK_CTRL_Len                       (3U)
#define AON_RF_REG_0_DYN_CLK_CTRL_Msk                       (0x7U <<  AON_RF_REG_0_DYN_CLK_CTRL_Pos)
#define AON_RF_REG_0_DYN_CLK_CTRL                           AON_RF_REG_0_DYN_CLK_CTRL_Msk

#define AON_RF_REG_0_BGAP_STATIC_EN_LV_Pos                  (19U)
#define AON_RF_REG_0_BGAP_STATIC_EN_LV_Len                  (1U)
#define AON_RF_REG_0_BGAP_STATIC_EN_LV_Msk                  (0x1U <<  AON_RF_REG_0_BGAP_STATIC_EN_LV_Pos)
#define AON_RF_REG_0_BGAP_STATIC_EN_LV_EN                   (0x1U <<  AON_RF_REG_0_BGAP_STATIC_EN_LV_Pos)
#define AON_RF_REG_0_BGAP_STATIC_EN_LV_DIS                  (0x0U <<  AON_RF_REG_0_BGAP_STATIC_EN_LV_Pos)

#define AON_RF_REG_0_RCOSC_BIAS_CNTRL_Pos                   (22)
#define AON_RF_REG_0_RCOSC_BIAS_CNTRL_Len                   (2U)
#define AON_RF_REG_0_RCOSC_BIAS_CNTRL_Msk                   (0x03 << AON_RF_REG_0_BGAP_STATIC_EN_LV_Pos)


#define AON_RF_REG_0_CTRL_TEMPCO_Pos                        (13U)
#define AON_RF_REG_0_CTRL_TEMPCO_Len                        (3U)
#define AON_RF_REG_0_CTRL_TEMPCO_Msk                        (0x7U << AON_RF_REG_0_CTRL_TEMPCO_Pos)
#define AON_RF_REG_0_CTRL_TEMPCO                            AON_RF_REG_0_CTRL_TEMPCO_Msk

#define AON_RF_REG_0_CTRL_RET_Pos                           (11U)
#define AON_RF_REG_0_CTRL_RET_Len                           (2U)
#define AON_RF_REG_0_CTRL_RET_Msk                           (0x3U << AON_RF_REG_0_CTRL_RET_Pos)
#define AON_RF_REG_0_CTRL_RET                               AON_RF_REG_0_CTRL_RET_Msk

#define AON_RF_REG_0_CTRL_BGAP_Pos                          (9U)
#define AON_RF_REG_0_CTRL_BGAP_Len                          (2U)
#define AON_RF_REG_0_CTRL_BGAP_Msk                          (0x3U << AON_RF_REG_0_CTRL_BGAP_Pos)
#define AON_RF_REG_0_CTRL_BGAP                              AON_RF_REG_0_CTRL_BGAP_Msk

#define AON_RF_REG_0_BGAP_VOLTAGE_EN_Pos                    (8U)
#define AON_RF_REG_0_BGAP_VOLTAGE_EN_Len                    (1U)
#define AON_RF_REG_0_BGAP_VOLTAGE_ON                        (0x1U << AON_RF_REG_0_BGAP_VOLTAGE_EN_Pos)
#define AON_RF_REG_0_BGAP_VOLTAGE_OFF                       (0x0U << AON_RF_REG_0_BGAP_VOLTAGE_EN_Pos)

#define AON_RF_REG_0_LPD_REG1_Pos                           (8U)
#define AON_RF_REG_0_LPD_REG1_Len                           (8U)
#define AON_RF_REG_0_LPD_REG1_Msk                           (0xFFU <<  AON_RF_REG_0_LPD_REG1_Pos)
#define AON_RF_REG_0_LPD_REG1                               AON_RF_REG_0_LPD_REG1_Msk

#define AON_RF_REG_0_RTC_REG1_Pos                           (0U)
#define AON_RF_REG_0_RTC_REG1_Len                           (8U)
#define AON_RF_REG_0_RTC_REG1_Msk                           (0xFFU <<  AON_RF_REG_0_RTC_REG1_Pos)
#define AON_RF_REG_0_RTC_REG1                               AON_RF_REG_0_RTC_REG1_Msk

/*******************  Bit definition for AON_REG_RF_REG_1 register  **********/
#define AON_RF_REG_1_DCDC_REG4_Pos                          (24U)
#define AON_RF_REG_1_DCDC_REG4_Len                          (8U)
#define AON_RF_REG_1_DCDC_REG4_Msk                          (0xFFU << AON_RF_REG_1_DCDC_REG4_Pos)
#define AON_RF_REG_1_DCDC_REG4                              AON_RF_REG_1_DCDC_REG4_Msk

#define AON_RF_REG_1_DCDC_REG3_Pos                          (16U)
#define AON_RF_REG_1_DCDC_REG3_Len                          (8U)
#define AON_RF_REG_1_DCDC_REG3_Msk                          (0xFFU << AON_RF_REG_1_DCDC_REG3_Pos)
#define AON_RF_REG_1_DCDC_REG3                              AON_RF_REG_1_DCDC_REG3_Msk

#define AON_RF_REG_1_EN_INJ_Pos                             (14U)
#define AON_RF_REG_1_EN_INJ_Msk                             (0x1 << AON_RF_REG_1_EN_INJ_Pos)
#define AON_RF_REG_1_EN_INJ_ON                              (0x1 << AON_RF_REG_1_EN_INJ_Pos)
#define AON_RF_REG_1_EN_INJ_OFF                             (0x0 << AON_RF_REG_1_EN_INJ_Pos)

#define AON_RF_REG_1_TON_Pos                                (11U)
#define AON_RF_REG_1_TON_Len                                (3U)
#define AON_RF_REG_1_TON_Msk                                (0x7U << AON_RF_REG_1_TON_Pos)
#define AON_RF_REG_1_TON                                    AON_RF_REG_1_TON_Msk

#define AON_RF_REG_1_DCDC_REG2_Pos                          (8U)
#define AON_RF_REG_1_DCDC_REG2_Len                          (8U)
#define AON_RF_REG_1_DCDC_REG2_Msk                          (0xFFU << AON_RF_REG_1_DCDC_REG2_Pos)
#define AON_RF_REG_1_DCDC_REG2                              AON_RF_REG_1_DCDC_REG2_Msk

#define AON_RF_REG_1_DCDC_REG1_Pos                          (0U)
#define AON_RF_REG_1_DCDC_REG1_Len                          (8U)
#define AON_RF_REG_1_DCDC_REG1_Msk                          (0xFFU << AON_RF_REG_1_DCDC_REG1_Pos)
#define AON_RF_REG_1_DCDC_REG1                              AON_RF_REG_1_DCDC_REG1_Msk

/*******************  Bit definition for AON_REG_RF_REG_2 register  **********/
#define AON_RF_REG_2_TON_EN_Pos                             (17U)
#define AON_RF_REG_2_TON_EN_Msk                             (0x1U << AON_RF_REG_2_TON_EN_Pos)
#define AON_RF_REG_2_TON_EN_ON                              (0x1  << AON_RF_REG_2_TON_EN_Pos)
#define AON_RF_REG_2_TON_EN_OFF                             (0x0  << AON_RF_REG_2_TON_EN_Pos)

#define AON_RF_REG_2_GP_REG2_Pos                            (16U)
#define AON_RF_REG_2_GP_REG2_Len                            (8U)
#define AON_RF_REG_2_GP_REG2_Msk                            (0xFFU << AON_RF_REG_2_GP_REG2_Pos)
#define AON_RF_REG_2_GP_REG2                                AON_RF_REG_2_GP_REG2_Msk

#define AON_RF_REG_2_GP_REG1_Pos                            (8U)
#define AON_RF_REG_2_GP_REG1_Len                            (8U)
#define AON_RF_REG_2_GP_REG1_Msk                            (0xFFU << AON_RF_REG_2_GP_REG1_Pos)
#define AON_RF_REG_2_GP_REG1                                AON_RF_REG_2_GP_REG1_Msk
#define AON_RF_REG_2_EFUSE_VDD_EN                           (0x4U << AON_RF_REG_2_GP_REG1_Pos)

#define AON_RF_REG_2_MUX1_REG1_Pos                          (0U)
#define AON_RF_REG_2_MUX1_REG1_Len                          (8U)
#define AON_RF_REG_2_MUX1_REG1_Msk                          (0xFFU << AON_RF_REG_2_MUX1_REG1_Pos)
#define AON_RF_REG_2_MUX1_REG1                              AON_RF_REG_2_MUX1_REG1_Msk

/*******************  Bit definition for AON_REG_CALENDAR_TIMER_CTL register  **********/
#define AON_CALENDAR_TIMER_CTL_WRAP_INT_EN_Pos              (13U)
#define AON_CALENDAR_TIMER_CTL_WRAP_INT_EN_Len              (1U)
#define AON_CALENDAR_TIMER_CTL_WRAP_INT_EN_Msk              (0x1U << AON_CALENDAR_TIMER_CTL_WRAP_INT_EN_Pos)
#define AON_CALENDAR_TIMER_CTL_WRAP_INT_EN                  AON_CALENDAR_TIMER_CTL_WRAP_INT_EN_Msk

#define AON_CALENDAR_TIMER_CTL_ALARM_INT_EN_Pos             (12U)
#define AON_CALENDAR_TIMER_CTL_ALARM_INT_EN_Len             (1U)
#define AON_CALENDAR_TIMER_CTL_ALARM_INT_EN_Msk             (0x1U << AON_CALENDAR_TIMER_CTL_ALARM_INT_EN_Pos)
#define AON_CALENDAR_TIMER_CTL_ALARM_INT_EN                 AON_CALENDAR_TIMER_CTL_ALARM_INT_EN_Msk

#define AON_CALENDAR_TIMER_CTL_CLK_SEL_Pos                  (8U)
#define AON_CALENDAR_TIMER_CTL_CLK_SEL_Len                  (3U)
#define AON_CALENDAR_TIMER_CTL_CLK_SEL_Msk                  (0x7U << AON_CALENDAR_TIMER_CTL_CLK_SEL_Pos)
#define AON_CALENDAR_TIMER_CTL_CLK_SEL                      AON_CALENDAR_TIMER_CTL_CLK_SEL_Msk

#define AON_CALENDAR_TIMER_CTL_WRAP_CNT_Pos                 (4U)
#define AON_CALENDAR_TIMER_CTL_WRAP_CNT_Len                 (4U)
#define AON_CALENDAR_TIMER_CTL_WRAP_CNT_Msk                 (0xFU << AON_CALENDAR_TIMER_CTL_WRAP_CNT_Pos)
#define AON_CALENDAR_TIMER_CTL_WRAP_CNT                     AON_CALENDAR_TIMER_CTL_WRAP_CNT_Msk

#define AON_CALENDAR_TIMER_CTL_ALARM_VAL_LOAD_Pos           (2U)
#define AON_CALENDAR_TIMER_CTL_ALARM_VAL_LOAD_Len           (1U)
#define AON_CALENDAR_TIMER_CTL_ALARM_VAL_LOAD_Msk           (0x1U << AON_CALENDAR_TIMER_CTL_ALARM_VAL_LOAD_Pos)
#define AON_CALENDAR_TIMER_CTL_ALARM_VAL_LOAD               AON_CALENDAR_TIMER_CTL_ALARM_VAL_LOAD_Msk

#define AON_CALENDAR_TIMER_CTL_VAL_LOAD_Pos                 (1U)
#define AON_CALENDAR_TIMER_CTL_VAL_LOAD_Len                 (1U)
#define AON_CALENDAR_TIMER_CTL_VAL_LOAD_Msk                 (0x1U << AON_CALENDAR_TIMER_CTL_VAL_LOAD_Pos)
#define AON_CALENDAR_TIMER_CTL_VAL_LOAD                     AON_CALENDAR_TIMER_CTL_VAL_LOAD_Msk

#define AON_CALENDAR_TIMER_CTL_EN_Pos                       (0U)
#define AON_CALENDAR_TIMER_CTL_EN_Len                       (1U)
#define AON_CALENDAR_TIMER_CTL_EN_Msk                       (0x1U << AON_CALENDAR_TIMER_CTL_EN_Pos)
#define AON_CALENDAR_TIMER_CTL_EN                           AON_CALENDAR_TIMER_CTL_EN_Msk

/*******************  Bit definition for AON_REG_MEM_STD_OVR register  **********/
#define AON_MEM_STD_OVR_MCU_KEYRAM_STDBY_N_Pos              (30U)
#define AON_MEM_STD_OVR_MCU_KEYRAM_STDBY_N_Len              (1U)
#define AON_MEM_STD_OVR_MCU_KEYRAM_STDBY_N_Msk              (0x1U << AON_MEM_STD_OVR_MCU_KEYRAM_STDBY_N_Pos)
#define AON_MEM_STD_OVR_MCU_KEYRAM_STDBY_N                  AON_MEM_STD_OVR_MCU_KEYRAM_STDBY_N_Msk

#define AON_MEM_STD_OVR_PMEM_STDBY_N_Pos                    (29U)
#define AON_MEM_STD_OVR_PMEM_STDBY_N_Len                    (1U)
#define AON_MEM_STD_OVR_PMEM_STDBY_N_Msk                    (0x1U << AON_MEM_STD_OVR_PMEM_STDBY_N_Pos)
#define AON_MEM_STD_OVR_PMEM_STDBY_N                        AON_MEM_STD_OVR_PMEM_STDBY_N_Msk

#define AON_MEM_STD_OVR_MCU_ICACHE_STDBY_N_Pos              (28U)
#define AON_MEM_STD_OVR_MCU_ICACHE_STDBY_N_Len              (1U)
#define AON_MEM_STD_OVR_MCU_ICACHE_STDBY_N_Msk              (1U << AON_MEM_STD_OVR_MCU_ICACHE_STDBY_N_Pos)
#define AON_MEM_STD_OVR_MCU_ICACHE_STDBY_N                  AON_MEM_STD_OVR_MCU_ICACHE_STDBY_N_Msk

#define AON_MEM_STD_OVR_MCU_HTM_STDBY_N_Pos                 (27U)
#define AON_MEM_STD_OVR_MCU_HTM_STDBY_N_Len                 (1U)
#define AON_MEM_STD_OVR_MCU_HTM_STDBY_N_Msk                 (1U << AON_MEM_STD_OVR_MCU_HTM_STDBY_N_Pos)
#define AON_MEM_STD_OVR_MCU_HTM_STDBY_N                     AON_MEM_STD_OVR_MCU_HTM_STDBY_N_Msk

#define AON_MEM_STD_OVR_MCU_MEM_STDBY_N_Pos                 (16U)
#define AON_MEM_STD_OVR_MCU_MEM_STDBY_N_Len                 (11U)
#define AON_MEM_STD_OVR_MCU_MEM_STDBY_N_Msk                 (0x7FFU << AON_MEM_STD_OVR_MCU_MEM_STDBY_N_Pos)
#define AON_MEM_STD_OVR_MCU_MEM_STDBY_N                     AON_MEM_STD_OVR_MCU_MEM_STDBY_N_Msk

#define AON_MEM_STD_OVR_KEYRAM_ISO_VDD_N_Pos                (14U)
#define AON_MEM_STD_OVR_KEYRAM_ISO_VDD_N_Len                (1U)
#define AON_MEM_STD_OVR_KEYRAM_ISO_VDD_N_Msk                (0x1U << AON_MEM_STD_OVR_KEYRAM_ISO_VDD_N_Pos)
#define AON_MEM_STD_OVR_KEYRAM_ISO_VDD_N                    AON_MEM_STD_OVR_KEYRAM_ISO_VDD_N_Msk

#define AON_MEM_STD_OVR_EF_CACHE_ISO_VDD_N_Pos              (13U)
#define AON_MEM_STD_OVR_EF_CACHE_ISO_VDD_N_Len              (1U)
#define AON_MEM_STD_OVR_EF_CACHE_ISO_VDD_N_Msk              (0x1U << AON_MEM_STD_OVR_EF_CACHE_ISO_VDD_N_Pos)
#define AON_MEM_STD_OVR_EF_CACHE_ISO_VDD_N                  AON_MEM_STD_OVR_EF_CACHE_ISO_VDD_N_Msk

#define AON_MEM_STD_OVR_HTABLE_ISO_VDD_N_Pos                (12U)
#define AON_MEM_STD_OVR_HTABLE_ISO_VDD_N_Len                (1U)
#define AON_MEM_STD_OVR_HTABLE_ISO_VDD_N_Msk                (0x1U << AON_MEM_STD_OVR_HTABLE_ISO_VDD_N_Pos)
#define AON_MEM_STD_OVR_HTABLE_ISO_VDD_N                    AON_MEM_STD_OVR_HTABLE_ISO_VDD_N_Msk

#define AON_MEM_STD_OVR_PMEM_ISO_VDD_N_Pos                  (11U)
#define AON_MEM_STD_OVR_PMEM_ISO_VDD_N_Len                  (1U)
#define AON_MEM_STD_OVR_PMEM_ISO_VDD_N_Msk                  (0x1U << AON_MEM_STD_OVR_PMEM_ISO_VDD_N_Pos)
#define AON_MEM_STD_OVR_PMEM_ISO_VDD_N                      AON_MEM_STD_OVR_PMEM_ISO_VDD_N_Msk

#define AON_MEM_STD_OVR_MCU_MEM_ISO_VDD_N_Pos               (0U)
#define AON_MEM_STD_OVR_MCU_MEM_ISO_VDD_N_Len               (11U)
#define AON_MEM_STD_OVR_MCU_MEM_ISO_VDD_N_Msk               (0x7FF << AON_MEM_STD_OVR_MCU_MEM_ISO_VDD_N_Pos)
#define AON_MEM_STD_OVR_MCU_MEM_ISO_VDD_N                   AON_MEM_STD_OVR_MCU_MEM_ISO_VDD_N_Msk

/*******************  Bit definition for AON_REG_RF_REG_3 register  **********/
#define AON_RF_REG_3_IO_LDO_REG2_Pos                        (24U)
#define AON_RF_REG_3_IO_LDO_REG2_Len                        (8U)
#define AON_RF_REG_3_IO_LDO_REG2_Msk                        (0xFFU << AON_RF_REG_3_IO_LDO_REG2_Pos)
#define AON_RF_REG_3_IO_LDO_REG2                            AON_RF_REG_3_IO_LDO_REG2_Msk

#define AON_RF_REG_3_LDO_5V_REG1_Pos                        (8U)
#define AON_RF_REG_3_LDO_5V_REG1_Len                        (8U)
#define AON_RF_REG_3_LDO_5V_REG1_Msk                        (0xFFU << AON_RF_REG_3_LDO_5V_REG1_Pos)
#define AON_RF_REG_3_LDO_5V_REG1                            AON_RF_REG_3_LDO_5V_REG1_Msk

#define AON_RF_REG_3_RTC_EN_Pos                             (7U)
#define AON_RF_REG_3_RTC_EN_Len                             (1U)
#define AON_RF_REG_3_RTC_EN_Msk                             (0x1U << AON_RF_REG_3_RTC_EN_Pos)
#define AON_RF_REG_3_RTC_EN                                 AON_RF_REG_3_RTC_EN_Msk
#define AON_RF_REG_3_RTC_DIS                                (0x0U <<  AON_RF_REG_3_RTC_EN_Pos)

#define AON_RF_REG_3_BOD_STATIC_LV_Pos                      (5U)
#define AON_RF_REG_3_BOD_STATIC_LV_Len                      (1U)
#define AON_RF_REG_3_BOD_STATIC_LV_Msk                      (0x1U <<  AON_RF_REG_3_BOD_STATIC_LV_Pos)
#define AON_RF_REG_3_BOD_STATIC_LV_EN                       (0x1U <<  AON_RF_REG_3_BOD_STATIC_LV_Pos)
#define AON_RF_REG_3_BOD_STATIC_LV_DIS                      (0x0U <<  AON_RF_REG_3_BOD_STATIC_LV_Pos)
#define AON_RF_REG_3_BOD_LVL_CTRL_LV_Pos                    (2U)
#define AON_RF_REG_3_BOD_LVL_CTRL_LV_Len                    (3U)
#define AON_RF_REG_3_BOD_LVL_CTRL_LV_Msk                    (0x7U <<  AON_RF_REG_3_BOD_LVL_CTRL_LV_Pos)
#define AON_RF_REG_3_BOD_LVL_CTRL_LV                        AON_RF_REG_3_BOD_LVL_CTRL_LV_Msk
#define AON_RF_REG_3_BOD2_EN_Pos                            (1U)
#define AON_RF_REG_3_BOD2_EN_Len                            (1U)
#define AON_RF_REG_3_BOD2_EN_Msk                            (0x1U <<  AON_RF_REG_3_BOD2_EN_Pos)
#define AON_RF_REG_3_BOD2_EN                                (0x1U <<  AON_RF_REG_3_BOD2_EN_Pos)
#define AON_RF_REG_3_BOD2_DIS                               (0x0U <<  AON_RF_REG_3_BOD2_EN_Pos)

#define AON_RF_REG_3_BOD_EN_Pos                             (0U)
#define AON_RF_REG_3_BOD_EN_Len                             (1U)
#define AON_RF_REG_3_BOD_EN_Msk                             (0x1U <<  AON_RF_REG_3_BOD_EN_Pos)
#define AON_RF_REG_3_BOD_EN                                 (0x1U <<  AON_RF_REG_3_BOD_EN_Pos)
#define AON_RF_REG_3_BOD_DIS                                (0x0U <<  AON_RF_REG_3_BOD_EN_Pos)

#define AON_RF_REG_3_BOD_REG1_Pos                           (0U)
#define AON_RF_REG_3_BOD_REG1_Len                           (8U)
#define AON_RF_REG_3_BOD_REG1_Msk                           (0xFFU << AON_RF_REG_3_BOD_REG1_Pos)
#define AON_RF_REG_3_BOD_REG1                               AON_RF_REG_3_BOD_REG1_Msk

/*******************  Bit definition for AON_REG_RF_REG_4 register  **********/
#define AON_RF_REG_4_DIG_LDO_REG1_Pos                       (16U)
#define AON_RF_REG_4_DIG_LDO_REG1_Len                       (8U)
#define AON_RF_REG_4_DIG_LDO_REG1_Msk                       (0xFFU << AON_RF_REG_4_DIG_LDO_REG1_Pos)
#define AON_RF_REG_4_DIG_LDO_REG1                           AON_RF_REG_4_DIG_LDO_REG1_Msk

#define AON_RF_REG_4_CLK_PERIOD_Pos                         (12U)
#define AON_RF_REG_4_CLK_PERIOD_Len                         (4U)
#define AON_RF_REG_4_CLK_PERIOD_Msk                         (0xF << AON_RF_REG_4_CLK_PERIOD_Pos)
#define AON_RF_REG_4_CLK_PERIOD                             AON_RF_REG_4_CLK_PERIOD_Msk

#define AON_RF_REG_4_DCDC_REG6_Pos                          (8U)
#define AON_RF_REG_4_DCDC_REG6_Len                          (8U)
#define AON_RF_REG_4_DCDC_REG6_Msk                          (0xFFU << AON_RF_REG_4_DCDC_REG6_Pos)
#define AON_RF_REG_4_DCDC_REG6                              AON_RF_REG_4_DCDC_REG6_Msk

#define AON_RF_REG_4_DCDC_REG5_Pos                          (0U)
#define AON_RF_REG_4_DCDC_REG5_Len                          (8U)
#define AON_RF_REG_4_DCDC_REG5_Msk                          (0xFFU << AON_RF_REG_4_DCDC_REG5_Pos)
#define AON_RF_REG_4_DCDC_REG5                              AON_RF_REG_4_DCDC_REG5_Msk

/*******************  Bit definition for AON_REG_RF_REG_5 register  **********/
#define AON_RF_REG_5_MUX_REG2_Pos                           (24U)
#define AON_RF_REG_5_MUX_REG2_Len                           (8U)
#define AON_RF_REG_5_MUX_REG2_Msk                           (0xFFU << AON_RF_REG_5_MUX_REG2_Pos)
#define AON_RF_REG_5_MUX_REG2                               AON_RF_REG_5_MUX_REG2_Msk

#define AON_RF_REG_5_MUX_REG1_Pos                           (16U)
#define AON_RF_REG_5_MUX_REG1_Len                           (8U)
#define AON_RF_REG_5_MUX_REG1_Msk                           (0xFFU << AON_RF_REG_5_MUX_REG1_Pos)
#define AON_RF_REG_5_MUX_REG1                               AON_RF_REG_5_MUX_REG1_Msk

#define AON_RF_REG_5_LPD_REG3_Pos                           (0U)
#define AON_RF_REG_5_LPD_REG3_Len                           (8U)
#define AON_RF_REG_5_LPD_REG3_Msk                           (0xFFU << AON_RF_REG_5_LPD_REG3_Pos)
#define AON_RF_REG_5_LPD_REG3                               AON_RF_REG_5_LPD_REG3_Msk

#define AON_RF_REG_5_RCOSC_EN_Pos                           (7)
#define AON_RF_REG_5_RCOSC_EN_Len                           (1U)
#define AON_RF_REG_5_RCOSC_EN_Msk                           (0x01 << AON_RF_REG_5_RCOSC_EN_Pos)
#define AON_RF_REG_5_RCOSC_EN                               (AON_RF_REG_5_RCOSC_EN_Msk)

#define AON_RF_REG_5_RCOSC_DELAY_EN_Pos                     (6)
#define AON_RF_REG_5_RCOSC_DELAY_EN_Len                     (1U)
#define AON_RF_REG_5_RCOSC_DELAY_EN_Msk                     (0x01 << AON_RF_REG_5_RCOSC_DELAY_EN_Pos)
#define AON_RF_REG_5_RCOSC_DELAY_EN                         (0x01 << AON_RF_REG_5_RCOSC_DELAY_EN_Pos)

#define AON_RF_REG_5_RCOSC_RDIV_DELAY_Pos                   (3)
#define AON_RF_REG_5_RCOSC_RDIV_DELAY_Len                   (3U)
#define AON_RF_REG_5_RCOSC_RDIV_DELAY_Msk                   (0x07 << AON_RF_REG_5_RCOSC_RDIV_DELAY_Pos)
#define AON_RF_REG_5_RCOSC_RDIV_DELAY                       (AON_RF_REG_5_RCOSC_RDIV_DELAY_Msk)

#define AON_RF_REG_5_RCOSC_RESN_CNTRL_Pos                   (0)
#define AON_RF_REG_5_RCOSC_RESN_CNTRL_Len                   (3U)
#define AON_RF_REG_5_RCOSC_RESN_CNTRL_Msk                   (0x07 << AON_RF_REG_5_RCOSC_RESN_CNTRL_Pos)
#define AON_RF_REG_5_RCOSC_RESN_CNTRL                       (AON_RF_REG_5_RCOSC_RESN_CNTRL_Msk)

/*******************  Bit definition for AON_REG_RF_REG_6 register  **********/
#define AON_RF_REG_6_CPLL_REG1_Pos                          (0U)
#define AON_RF_REG_6_CPLL_REG1_Len                          (32U)
#define AON_RF_REG_6_CPLL_REG1_Msk                          (0xFFFFFFFFU)
#define AON_RF_REG_6_CPLL_REG1                              AON_RF_REG_6_CPLL_REG1_Msk

/*******************  Bit definition for AON_REG_RF_REG_7 register  **********/
#define AON_RF_REG_7_CPLL_REG2_Pos                          (0U)
#define AON_RF_REG_7_CPLL_REG2_Len                          (32U)
#define AON_RF_REG_7_CPLL_REG2_Msk                          (0xFFFFFFFFU)
#define AON_RF_REG_7_CPLL_REG2                              AON_RF_REG_7_CPLL_REG2_Msk

/*******************  Bit definition for AON_REG_RF_REG_8 register  **********/
#define AON_RF_REG_8_XO_REG1_Pos                            (0U)
#define AON_RF_REG_8_XO_REG1_Len                            (32U)
#define AON_RF_REG_8_XO_REG1_Msk                            (0xFFFFFFFFU)
#define AON_RF_REG_8_XO_REG1                                AON_RF_REG_8_XO_REG1_Msk

/*******************  Bit definition for AON_REG_RF_REG_9 register  **********/
#define AON_RF_REG_9_XO_REG2_Pos                            (0U)
#define AON_RF_REG_9_XO_REG2_Len                            (32U)
#define AON_RF_REG_9_XO_REG2_Msk                            (0xFFFFFFFFU)
#define AON_RF_REG_9_XO_REG2                                AON_RF_REG_9_XO_REG2_Msk

/*******************  Bit definition for AON_REG_MSIO_PAD_CFG_0 register  **********/
#define AON_MSIO_PAD_CFG_0_OE_N_Pos                         (24U)
#define AON_MSIO_PAD_CFG_0_OE_N_Len                         (5U)
#define AON_MSIO_PAD_CFG_0_OE_N_Msk                         (0x1FU << AON_MSIO_PAD_CFG_0_OE_N_Pos)
#define AON_MSIO_PAD_CFG_0_OE_N                             AON_MSIO_PAD_CFG_0_OE_N_Msk

#define AON_MSIO_PAD_CFG_0_IE_N_Pos                         (16U)
#define AON_MSIO_PAD_CFG_0_IE_N_Len                         (5U)
#define AON_MSIO_PAD_CFG_0_IE_N_Msk                         (0x1FU << AON_MSIO_PAD_CFG_0_IE_N_Pos)
#define AON_MSIO_PAD_CFG_0_IE_N                             AON_MSIO_PAD_CFG_0_IE_N_Msk

#define AON_MSIO_PAD_CFG_0_IN_Pos                           (8U)
#define AON_MSIO_PAD_CFG_0_IN_Len                           (5U)
#define AON_MSIO_PAD_CFG_0_IN_Msk                           (0x1FU << AON_MSIO_PAD_CFG_0_IN_Pos)
#define AON_MSIO_PAD_CFG_0_IN                               AON_MSIO_PAD_CFG_0_IN_Msk

#define AON_MSIO_PAD_CFG_0_RE_N_Pos                         (0U)
#define AON_MSIO_PAD_CFG_0_RE_N_Len                         (5U)
#define AON_MSIO_PAD_CFG_0_RE_N_Msk                         (0x1FU << AON_MSIO_PAD_CFG_0_RE_N_Pos)
#define AON_MSIO_PAD_CFG_0_RE_N                             AON_MSIO_PAD_CFG_0_RE_N_Msk

/*******************  Bit definition for AON_REG_MSIO_PAD_CFG_1 register  **********/
#define AON_MSIO_PAD_CFG_1_ADC_CLK_EN_Pos                   (31U)
#define AON_MSIO_PAD_CFG_1_ADC_CLK_EN_Len                   (1U)
#define AON_MSIO_PAD_CFG_1_ADC_CLK_EN_Msk                   (0x1U << AON_MSIO_PAD_CFG_1_ADC_CLK_EN_Pos)
#define AON_MSIO_PAD_CFG_1_ADC_CLK_EN                       AON_MSIO_PAD_CFG_1_ADC_CLK_EN_Msk

#define AON_MSIO_PAD_CFG_1_ADC_CLK_SEL_Pos                  (28U)
#define AON_MSIO_PAD_CFG_1_ADC_CLK_SEL_Len                  (3U)
#define AON_MSIO_PAD_CFG_1_ADC_CLK_SEL_Msk                  (0x7U << AON_MSIO_PAD_CFG_1_ADC_CLK_SEL_Pos)
#define AON_MSIO_PAD_CFG_1_ADC_CLK_SEL                      AON_MSIO_PAD_CFG_1_ADC_CLK_SEL_Msk

#define AON_MSIO_PAD_CFG_1_MCU_OVR_Pos                      (22U)
#define AON_MSIO_PAD_CFG_1_MCU_OVR_Len                      (5U)
#define AON_MSIO_PAD_CFG_1_MCU_OVR_Msk                      (0x1FU << AON_MSIO_PAD_CFG_1_MCU_OVR_Pos)
#define AON_MSIO_PAD_CFG_1_MCU_OVR                          AON_MSIO_PAD_CFG_1_MCU_OVR_Msk

#define AON_COMM_DEEPSLCNTL_EXTWKUPDSB_Pos                  (21U)
#define AON_COMM_DEEPSLCNTL_EXTWKUPDSB_Len                  (1U)
#define AON_COMM_DEEPSLCNTL_EXTWKUPDSB_Msk                  (0x1U << AON_COMM_DEEPSLCNTL_EXTWKUPDSB_Pos)
#define AON_COMM_DEEPSLCNTL_EXTWKUPDSB                      AON_COMM_DEEPSLCNTL_EXTWKUPDSB_Msk

#define AON_COMM_DEEPSLCNTL_SOFT_WAKEUP_REQ_Pos             (20U)
#define AON_COMM_DEEPSLCNTL_SOFT_WAKEUP_REQ_Len             (1U)
#define AON_COMM_DEEPSLCNTL_SOFT_WAKEUP_REQ_Msk             (0x1U << AON_COMM_DEEPSLCNTL_SOFT_WAKEUP_REQ_Pos)
#define AON_COMM_DEEPSLCNTL_SOFT_WAKEUP_REQ                 AON_COMM_DEEPSLCNTL_SOFT_WAKEUP_REQ_Msk

#define AON_COMM_DEEPSLCNTL_DEEP_SLEEP_ON_Pos               (18U)
#define AON_COMM_DEEPSLCNTL_DEEP_SLEEP_ON_Len               (1U)
#define AON_COMM_DEEPSLCNTL_DEEP_SLEEP_ON_Msk               (0x1U << AON_COMM_DEEPSLCNTL_DEEP_SLEEP_ON_Pos)
#define AON_COMM_DEEPSLCNTL_DEEP_SLEEP_ON                   AON_COMM_DEEPSLCNTL_DEEP_SLEEP_ON_Msk

#define AON_COMM_DEEPSLCNTL_RADIO_SLEEP_EN_Pos              (17U)
#define AON_COMM_DEEPSLCNTL_RADIO_SLEEP_EN_Len              (1U)
#define AON_COMM_DEEPSLCNTL_RADIO_SLEEP_EN_Msk              (0x1U << AON_COMM_DEEPSLCNTL_RADIO_SLEEP_EN_Pos)
#define AON_COMM_DEEPSLCNTL_RADIO_SLEEP_EN                  AON_COMM_DEEPSLCNTL_RADIO_SLEEP_EN_Msk

#define AON_COMM_DEEPSLCNTL_OSC_SLEEP_EN_Pos                (16U)
#define AON_COMM_DEEPSLCNTL_OSC_SLEEP_EN_Len                (1U)
#define AON_COMM_DEEPSLCNTL_OSC_SLEEP_EN_Msk                (0x1U << AON_COMM_DEEPSLCNTL_OSC_SLEEP_EN_Pos)
#define AON_COMM_DEEPSLCNTL_OSC_SLEEP_EN                    AON_COMM_DEEPSLCNTL_OSC_SLEEP_EN_Msk

#define AON_COMM_DEEPSLCNTL_DEEP_SLEEP_STAT_Pos             (15U)
#define AON_COMM_DEEPSLCNTL_DEEP_SLEEP_STAT_Len             (1U)
#define AON_COMM_DEEPSLCNTL_DEEP_SLEEP_STAT_Msk             (0x1U << AON_COMM_DEEPSLCNTL_DEEP_SLEEP_STAT_Pos)
#define AON_COMM_DEEPSLCNTL_DEEP_SLEEP_STAT                 AON_COMM_DEEPSLCNTL_DEEP_SLEEP_STAT_Msk

#define AON_MSIO_PAD_CFG_1_RTYPE_Pos                        (8U)
#define AON_MSIO_PAD_CFG_1_RTYPE_Len                        (5U)
#define AON_MSIO_PAD_CFG_1_RTYPE_Msk                        (0x1FU << AON_MSIO_PAD_CFG_1_RTYPE_Pos)
#define AON_MSIO_PAD_CFG_1_RTYPE                            AON_MSIO_PAD_CFG_1_RTYPE_Msk

#define AON_MSIO_PAD_CFG_1_AE_N_Pos                         (0U)
#define AON_MSIO_PAD_CFG_1_AE_N_Len                         (5U)
#define AON_MSIO_PAD_CFG_1_AE_N_Msk                         (0x1FU << AON_MSIO_PAD_CFG_1_AE_N_Pos)
#define AON_MSIO_PAD_CFG_1_AE_N                             AON_MSIO_PAD_CFG_1_AE_N_Msk

/*******************  Bit definition for AON_REG_SLP_EVENT register  **********/
#define AON_SLP_EVENT_SLP_TIMER_MODE_Pos                    (30U)
#define AON_SLP_EVENT_SLP_TIMER_MODE_Len                    (2U)
#define AON_SLP_EVENT_SLP_TIMER_MODE_Msk                    (0x3U << AON_SLP_EVENT_SLP_TIMER_MODE_Pos)
#define AON_SLP_EVENT_SLP_TIMER_MODE                        AON_SLP_EVENT_SLP_TIMER_MODE_Msk
#define AON_SLP_EVENT_SLP_TIMER_MODE_NORMAL                 (0x0U << AON_SLP_EVENT_SLP_TIMER_MODE_Pos)
#define AON_SLP_EVENT_SLP_TIMER_MODE_SINGLE                 (0x1U << AON_SLP_EVENT_SLP_TIMER_MODE_Pos)
#define AON_SLP_EVENT_SLP_TIMER_MODE_RELOAD                 (0x2U << AON_SLP_EVENT_SLP_TIMER_MODE_Pos)
#define AON_SLP_EVENT_SLP_TIMER_MODE_DISABLE                (0x3U << AON_SLP_EVENT_SLP_TIMER_MODE_Pos)

#define AON_SLP_EVENT_EXT_WKUP_STATUS_Pos                   (16U)
#define AON_SLP_EVENT_EXT_WKUP_STATUS_Len                   (8U)
#define AON_SLP_EVENT_EXT_WKUP_STATUS_Msk                   (0xFFU << AON_SLP_EVENT_EXT_WKUP_STATUS_Pos)
#define AON_SLP_EVENT_EXT_WKUP_STATUS                       AON_SLP_EVENT_EXT_WKUP_STATUS_Msk

#define AON_SLP_EVENT_CALENDAR_TIMER_WRAP_Pos               (9U)
#define AON_SLP_EVENT_CALENDAR_TIMER_WRAP_Len               (1U)
#define AON_SLP_EVENT_CALENDAR_TIMER_WRAP_Msk               (0x1U << AON_SLP_EVENT_CALENDAR_TIMER_WRAP_Pos)
#define AON_SLP_EVENT_CALENDAR_TIMER_WRAP                   AON_SLP_EVENT_CALENDAR_TIMER_WRAP_Msk

#define AON_SLP_EVENT_CALENDAR_TIMER_ALARM_Pos              (8U)
#define AON_SLP_EVENT_CALENDAR_TIMER_ALARM_Len              (1U)
#define AON_SLP_EVENT_CALENDAR_TIMER_ALARM_Msk              (0x1U << AON_SLP_EVENT_CALENDAR_TIMER_ALARM_Pos)
#define AON_SLP_EVENT_CALENDAR_TIMER_ALARM                  AON_SLP_EVENT_CALENDAR_TIMER_ALARM_Msk

#define AON_SLP_EVENT_WDT_REBOOT_Pos                        (6U)
#define AON_SLP_EVENT_WDT_REBOOT_Len                        (1U)
#define AON_SLP_EVENT_WDT_REBOOT_Msk                        (0x1U << AON_SLP_EVENT_WDT_REBOOT_Pos)
#define AON_SLP_EVENT_WDT_REBOOT                            AON_SLP_EVENT_WDT_REBOOT_Msk

#define AON_SLP_EVENT_PMU_MSIO_COMP_Pos                     (4U)
#define AON_SLP_EVENT_PMU_MSIO_COMP_Len                     (1U)
#define AON_SLP_EVENT_PMU_MSIO_COMP_Msk                     (0x1U << AON_SLP_EVENT_PMU_MSIO_COMP_Pos)
#define AON_SLP_EVENT_PMU_MSIO_COMP                         AON_SLP_EVENT_PMU_MSIO_COMP_Msk

#define AON_SLP_EVENT_PMU_BOD_FEDGE_Pos                     (3U)
#define AON_SLP_EVENT_PMU_BOD_FEDGE_Len                     (1U)
#define AON_SLP_EVENT_PMU_BOD_FEDGE_Msk                     (0x1U << AON_SLP_EVENT_PMU_BOD_FEDGE_Pos)
#define AON_SLP_EVENT_PMU_BOD_FEDGE                         AON_SLP_EVENT_PMU_BOD_FEDGE_Msk

#define AON_SLP_EVENT_EXTWKUP_Pos                           (2U)
#define AON_SLP_EVENT_EXTWKUP_Len                           (1U)
#define AON_SLP_EVENT_EXTWKUP_Msk                           (0x1U << AON_SLP_EVENT_EXTWKUP_Pos)
#define AON_SLP_EVENT_EXTWKUP                               AON_SLP_EVENT_EXTWKUP_Msk

#define AON_SLP_EVENT_TIMER_Pos                             (1U)
#define AON_SLP_EVENT_TIMER_Len                             (1U)
#define AON_SLP_EVENT_TIMER_Msk                             (0x1U << AON_SLP_EVENT_TIMER_Pos)
#define AON_SLP_EVENT_TIMER                                 AON_SLP_EVENT_TIMER_Msk

#define AON_SLP_EVENT_SMCOSCEN_Pos                          (0U)
#define AON_SLP_EVENT_SMCOSCEN_Len                          (1U)
#define AON_SLP_EVENT_SMCOSCEN_Msk                          (0x1U << AON_SLP_EVENT_SMCOSCEN_Pos)
#define AON_SLP_EVENT_SMCOSCEN                              AON_SLP_EVENT_SMCOSCEN_Msk

/*******************  Bit definition for AON_REG_WARM_BOOT_TIME register  **********/
#define AON_WARM_BOOT_TIME_TUNE_C_Pos                       (24U)
#define AON_WARM_BOOT_TIME_TUNE_C_Len                       (7U)
#define AON_WARM_BOOT_TIME_TUNE_C_Msk                       (0x7FU << AON_WARM_BOOT_TIME_TUNE_C_Pos)
#define AON_WARM_BOOT_TIME_TUNE_C                           AON_WARM_BOOT_TIME_TUNE_C_Msk

#define AON_WARM_BOOT_TIME_DIG_LDO_D_Pos                    (16U)
#define AON_WARM_BOOT_TIME_DIG_LDO_D_Len                    (7U)
#define AON_WARM_BOOT_TIME_DIG_LDO_D_Msk                    (0x7FU << AON_WARM_BOOT_TIME_DIG_LDO_D_Pos)
#define AON_WARM_BOOT_TIME_DIG_LDO_D                        AON_WARM_BOOT_TIME_DIG_LDO_D_Msk

#define AON_WARM_BOOT_TIME_COUNTER_B_Pos                    (8U)
#define AON_WARM_BOOT_TIME_COUNTER_B_Len                    (7U)
#define AON_WARM_BOOT_TIME_COUNTER_B_Msk                    (0x7FU << AON_WARM_BOOT_TIME_COUNTER_B_Pos)
#define AON_WARM_BOOT_TIME_COUNTER_B                        AON_WARM_BOOT_TIME_COUNTER_B_Msk

#define AON_WARM_BOOT_TIME_COUNTER_A_Pos                    (0U)
#define AON_WARM_BOOT_TIME_COUNTER_A_Len                    (7U)
#define AON_WARM_BOOT_TIME_COUNTER_A_Msk                    (0x7FU << AON_WARM_BOOT_TIME_COUNTER_A_Pos)
#define AON_WARM_BOOT_TIME_COUNTER_A                        AON_WARM_BOOT_TIME_COUNTER_A_Msk

/*******************  Bit definition for AON_REG_RF_REG_10 register  **********/
#define AON_RF_REG_10_MSIO_0                                (0U)
#define AON_RF_REG_10_MSIO_1                                (1U)
#define AON_RF_REG_10_MSIO_2                                (2U)
#define AON_RF_REG_10_MSIO_3                                (3U)
#define AON_RF_REG_10_MSIO_4                                (4U)
#define AON_RF_REG_10_VTEMP                                 (5U)
#define AON_RF_REG_10_VBATT                                 (6U)
#define AON_RF_REG_10_VREF                                  (7U)

#define AON_RF_REG_10_XO_BYP_Pos                            (24U)
#define AON_RF_REG_10_XO_BYP_Len                            (1U)
#define AON_RF_REG_10_XO_BYP_Msk                            (0x1U << AON_RF_REG_10_XO_BYP_Pos)
#define AON_RF_REG_10_XO_BYP                                AON_RF_REG_10_XO_BYP_Msk

#define AON_RF_REG_10_COMP_REF_CTRL_LV_Pos                  (16U)
#define AON_RF_REG_10_COMP_REF_CTRL_LV_Len                  (6U)
#define AON_RF_REG_10_COMP_REF_CTRL_LV_Msk                  (0x3FU << AON_RF_REG_10_COMP_REF_CTRL_LV_Pos)
#define AON_RF_REG_10_COMP_REF_CTRL_LV                      AON_RF_REG_10_COMP_REF_CTRL_LV_Msk

#define AON_RF_REG_10_LPD_REG6_Pos                          (16U)
#define AON_RF_REG_10_LPD_REG6_Len                          (8U)
#define AON_RF_REG_10_LPD_REG6_Msk                          (0xFFU << AON_RF_REG_10_LPD_REG6_Pos)
#define AON_RF_REG_10_LPD_REG6                              AON_RF_REG_10_LPD_REG6_Msk

#define AON_RF_REG_10_COMP_BATT_LVL_CTRL_LV_Pos             (12U)
#define AON_RF_REG_10_COMP_BATT_LVL_CTRL_LV_Len             (3U)
#define AON_RF_REG_10_COMP_BATT_LVL_CTRL_LV_Msk             (0x7U << AON_RF_REG_10_COMP_BATT_LVL_CTRL_LV_Pos)
#define AON_RF_REG_10_COMP_BATT_LVL_CTRL_LV                 AON_RF_REG_10_COMP_BATT_LVL_CTRL_LV_Msk

#define AON_RF_REG_10_ICOMP_CTRL_LV_Pos                     (8U)
#define AON_RF_REG_10_ICOMP_CTRL_LV_Len                     (4U)
#define AON_RF_REG_10_ICOMP_CTRL_LV_Msk                     (0xFU << AON_RF_REG_10_ICOMP_CTRL_LV_Pos)
#define AON_RF_REG_10_ICOMP_CTRL_LV                         AON_RF_REG_10_ICOMP_CTRL_LV_Msk

#define AON_RF_REG_10_LPD_REG5_Pos                          (8U)
#define AON_RF_REG_10_LPD_REG5_Len                          (8U)
#define AON_RF_REG_10_LPD_REG5_Msk                          (0xFFU << AON_RF_REG_10_LPD_REG5_Pos)
#define AON_RF_REG_10_LPD_REG5                              AON_RF_REG_10_LPD_REG5_Msk

#define AON_RF_REG_10_WAKE_COMP_EN_Pos                      (6U)
#define AON_RF_REG_10_WAKE_COMP_EN_Len                      (1U)
#define AON_RF_REG_10_WAKE_COMP_EN_Msk                      (0x1U << AON_RF_REG_10_WAKE_COMP_EN_Pos)
#define AON_RF_REG_10_WAKE_COMP_EN                          AON_RF_REG_10_WAKE_COMP_EN_Msk

#define AON_RF_REG_10_CHANNEL_SEL_N_Pos                     (3U)
#define AON_RF_REG_10_CHANNEL_SEL_N_Len                     (3U)
#define AON_RF_REG_10_CHANNEL_SEL_N_Msk                     (0x7U << AON_RF_REG_10_CHANNEL_SEL_N_Pos)
#define AON_RF_REG_10_CHANNEL_SEL_N                         AON_RF_REG_10_CHANNEL_SEL_N_Msk

#define AON_RF_REG_10_CHANNEL_SEL_P_Pos                     (0U)
#define AON_RF_REG_10_CHANNEL_SEL_P_Len                     (3U)
#define AON_RF_REG_10_CHANNEL_SEL_P_Msk                     (0x7U << AON_RF_REG_10_CHANNEL_SEL_P_Pos)
#define AON_RF_REG_10_CHANNEL_SEL_P                         AON_RF_REG_10_CHANNEL_SEL_P_Msk

#define AON_RF_REG_10_LPD_REG4_Pos                          (0U)
#define AON_RF_REG_10_LPD_REG4_Len                          (8U)
#define AON_RF_REG_10_LPD_REG4_Msk                          (0xFFU << AON_RF_REG_10_LPD_REG4_Pos)
#define AON_RF_REG_10_LPD_REG4                              AON_RF_REG_10_LPD_REG4_Msk

/*******************  Bit definition for AON_REG_AON_PAD_CTL0 register  **************/
#define AON_PAD_CTL0_COMM_TIMER_CLK_SEL_Pos                 (28U)
#define AON_PAD_CTL0_COMM_TIMER_CLK_SEL_Len                 (2U)
#define AON_PAD_CTL0_COMM_TIMER_CLK_SEL_Msk                 (0x3U << AON_PAD_CTL0_COMM_TIMER_CLK_SEL_Pos)
#define AON_PAD_CTL0_COMM_TIMER_CLK_SEL                     AON_PAD_CTL0_COMM_TIMER_CLK_SEL_Msk
#define AON_PAD_CTL0_COMM_TIMER_CLK_SEL_RNG                 (0x00 << AON_PAD_CTL0_COMM_TIMER_CLK_SEL_Pos)
#define AON_PAD_CTL0_COMM_TIMER_CLK_SEL_RTC                 (0x01 << AON_PAD_CTL0_COMM_TIMER_CLK_SEL_Pos)
#define AON_PAD_CTL0_COMM_TIMER_CLK_SEL_RNG2                (0x03 << AON_PAD_CTL0_COMM_TIMER_CLK_SEL_Pos)

#define AON_PAD_CTL0_MCU_OVR_Pos                            (16U)
#define AON_PAD_CTL0_MCU_OVR_Len                            (8U)
#define AON_PAD_CTL0_MCU_OVR_Msk                            (0xFFU << AON_PAD_CTL0_MCU_OVR_Pos)
#define AON_PAD_CTL0_MCU_OVR                                AON_PAD_CTL0_MCU_OVR_Msk

#define AON_PAD_CTL0_GPO_RTYPE_Pos                          (8U)
#define AON_PAD_CTL0_GPO_RTYPE_Len                          (8U)
#define AON_PAD_CTL0_GPO_RTYPE_Msk                          (0xFFU << AON_PAD_CTL0_GPO_RTYPE_Pos)
#define AON_PAD_CTL0_GPO_RTYPE                              AON_PAD_CTL0_GPO_RTYPE_Msk

#define AON_PAD_CTL0_GPO_RE_N_Pos                           (0U)
#define AON_PAD_CTL0_GPO_RE_N_Len                           (8U)
#define AON_PAD_CTL0_GPO_RE_N_Msk                           (0xFFU << AON_PAD_CTL0_GPO_RE_N_Pos)
#define AON_PAD_CTL0_GPO_RE_N                               AON_PAD_CTL0_GPO_RE_N_Msk

/*******************  Bit definition for AON_REG_MEM_N_SLP_CTL register  ****************/
#define AON_MEM_CTL_DPAD_LE_WKUP_VAL_Pos                    (25U)
#define AON_MEM_CTL_DPAD_LE_WKUP_VAL_Len                    (1U)
#define AON_MEM_CTL_DPAD_LE_WKUP_VAL_Msk                    (0x1U << AON_MEM_CTL_DPAD_LE_WKUP_VAL_Pos)
#define AON_MEM_CTL_DPAD_LE_WKUP_VAL                        AON_MEM_CTL_DPAD_LE_WKUP_VAL_Msk

#define AON_MEM_CTL_DPAD_LE_SLP_VAL_Pos                     (24U)
#define AON_MEM_CTL_DPAD_LE_SLP_VAL_Len                     (1U)
#define AON_MEM_CTL_DPAD_LE_SLP_VAL_Msk                     (0x1U << AON_MEM_CTL_DPAD_LE_SLP_VAL_Pos)
#define AON_MEM_CTL_DPAD_LE_SLP_VAL                         AON_MEM_CTL_DPAD_LE_SLP_VAL_Msk

#define AON_MEM_CTL_SLP_Pos                                 (16U)
#define AON_MEM_CTL_SLP_Len                                 (7U)
#define AON_MEM_CTL_SLP_Msk                                 (0x7FU << AON_MEM_CTL_SLP_Pos)
#define AON_MEM_CTL_SLP_EN                                  AON_MEM_CTL_SLP_Msk
#define AON_MEM_CTL_SLP_ALL                                 (AON_MEM_CTL_SLP_TRN_OFF_DCDC      | AON_MEM_CTL_SLP_TRN_OFF_XO |\
                                                             AON_MEM_CTL_SLP_TRN_OFF_PLL_EN    | AON_MEM_CTL_SLP_TRN_OFF_PLL_TUNE |\
                                                             AON_MEM_CTL_SLP_TRN_OFF_LDO_EN    | AON_MEM_CTL_SLP_TRN_OFF_PLL_RST |\
                                                             AON_MEM_CTL_SLP_TRN_OFF_IO_LDO_EN )

#define AON_MEM_CTL_SLP_TRN_OFF_IO_LDO_EN_Pos               (22U)
#define AON_MEM_CTL_SLP_TRN_OFF_IO_LDO_EN_Len               (1U)
#define AON_MEM_CTL_SLP_TRN_OFF_IO_LDO_EN_Msk               (0x1U << AON_MEM_CTL_SLP_TRN_OFF_IO_LDO_EN_Pos)
#define AON_MEM_CTL_SLP_TRN_OFF_IO_LDO_EN                   AON_MEM_CTL_SLP_TRN_OFF_IO_LDO_EN_Msk

#define AON_MEM_CTL_SLP_TRN_OFF_PLL_RST_Pos                 (21U)
#define AON_MEM_CTL_SLP_TRN_OFF_PLL_RST_Len                 (1U)
#define AON_MEM_CTL_SLP_TRN_OFF_PLL_RST_Msk                 (0x1U << AON_MEM_CTL_SLP_TRN_OFF_PLL_RST_Pos)
#define AON_MEM_CTL_SLP_TRN_OFF_PLL_RST                     AON_MEM_CTL_SLP_TRN_OFF_PLL_RST_Msk

#define AON_MEM_CTL_SLP_TRN_OFF_LDO_EN_Pos                  (20U)
#define AON_MEM_CTL_SLP_TRN_OFF_LDO_EN_Len                  (1U)
#define AON_MEM_CTL_SLP_TRN_OFF_LDO_EN_Msk                  (0x1U << AON_MEM_CTL_SLP_TRN_OFF_LDO_EN_Pos)
#define AON_MEM_CTL_SLP_TRN_OFF_LDO_EN                      AON_MEM_CTL_SLP_TRN_OFF_LDO_EN_Msk

#define AON_MEM_CTL_SLP_TRN_OFF_PLL_TUNE_Pos                (19U)
#define AON_MEM_CTL_SLP_TRN_OFF_PLL_TUNE_Len                (1U)
#define AON_MEM_CTL_SLP_TRN_OFF_PLL_TUNE_Msk                (0x1U << AON_MEM_CTL_SLP_TRN_OFF_PLL_TUNE_Pos)
#define AON_MEM_CTL_SLP_TRN_OFF_PLL_TUNE                    AON_MEM_CTL_SLP_TRN_OFF_PLL_TUNE_Msk

#define AON_MEM_CTL_SLP_TRN_OFF_PLL_EN_Pos                  (18U)
#define AON_MEM_CTL_SLP_TRN_OFF_PLL_EN_Len                  (1U)
#define AON_MEM_CTL_SLP_TRN_OFF_PLL_EN_Msk                  (0x1U << AON_MEM_CTL_SLP_TRN_OFF_PLL_EN_Pos)
#define AON_MEM_CTL_SLP_TRN_OFF_PLL_EN                      AON_MEM_CTL_SLP_TRN_OFF_PLL_EN_Msk

#define AON_MEM_CTL_SLP_TRN_OFF_XO_Pos                      (17U)
#define AON_MEM_CTL_SLP_TRN_OFF_XO_Len                      (1U)
#define AON_MEM_CTL_SLP_TRN_OFF_XO_Msk                      (0x1U << AON_MEM_CTL_SLP_TRN_OFF_XO_Pos)
#define AON_MEM_CTL_SLP_TRN_OFF_XO                          AON_MEM_CTL_SLP_TRN_OFF_XO_Msk

#define AON_MEM_CTL_SLP_TRN_OFF_DCDC_Pos                    (16U)
#define AON_MEM_CTL_SLP_TRN_OFF_DCDC_Len                    (1U)
#define AON_MEM_CTL_SLP_TRN_OFF_DCDC_Msk                    (0x1U << AON_MEM_CTL_SLP_TRN_OFF_DCDC_Pos)
#define AON_MEM_CTL_SLP_TRN_OFF_DCDC                        AON_MEM_CTL_SLP_TRN_OFF_DCDC_Msk

#define AON_MEM_CTL_MEM_BTRM_Pos                            (8U)
#define AON_MEM_CTL_MEM_BTRM_Len                            (4U)
#define AON_MEM_CTL_MEM_BTRM_Msk                            (0xFU << AON_MEM_CTL_MEM_BTRM_Pos)
#define AON_MEM_CTL_MEM_BTRM                                AON_MEM_CTL_MEM_BTRM_Msk

#define AON_MEM_CTL_NON_CRITICAL_MEM_RWM_Pos                (6U)
#define AON_MEM_CTL_NON_CRITICAL_MEM_RWM_Len                (2U)
#define AON_MEM_CTL_NON_CRITICAL_MEM_RWM_Msk                (0x3U << AON_MEM_CTL_NON_CRITICAL_MEM_RWM_Pos)
#define AON_MEM_CTL_NON_CRITICAL_MEM_RWM                    AON_MEM_CTL_NON_CRITICAL_MEM_RWM_Msk

#define AON_MEM_CTL_NON_CRITICAL_MEM_WM_Pos                 (5U)
#define AON_MEM_CTL_NON_CRITICAL_MEM_WM_Len                 (1U)
#define AON_MEM_CTL_NON_CRITICAL_MEM_WM_Msk                 (0x1U << AON_MEM_CTL_NON_CRITICAL_MEM_WM_Pos)
#define AON_MEM_CTL_NON_CRITICAL_MEM_WM                     AON_MEM_CTL_NON_CRITICAL_MEM_WM_Msk

#define AON_MEM_CTL_NON_CRITICAL_MEM_RM_Pos                 (4U)
#define AON_MEM_CTL_NON_CRITICAL_MEM_RM_Len                 (1U)
#define AON_MEM_CTL_NON_CRITICAL_MEM_RM_Msk                 (0x1U << AON_MEM_CTL_NON_CRITICAL_MEM_RM_Pos)
#define AON_MEM_CTL_NON_CRITICAL_MEM_RM                     AON_MEM_CTL_NON_CRITICAL_MEM_RM_Msk

#define AON_MEM_CTL_CRITICAL_MEM_RWM_Pos                    (2U)
#define AON_MEM_CTL_CRITICAL_MEM_RWM_Len                    (2U)
#define AON_MEM_CTL_CRITICAL_MEM_RWM_Msk                    (0x3U << AON_MEM_CTL_CRITICAL_MEM_RWM_Pos)
#define AON_MEM_CTL_CRITICAL_MEM_RWM                        AON_MEM_CTL_CRITICAL_MEM_RWM_Msk

#define AON_MEM_CTL_CRITICAL_MEM_WM_Pos                     (1U)
#define AON_MEM_CTL_CRITICAL_MEM_WM_Len                     (1U)
#define AON_MEM_CTL_CRITICAL_MEM_WM_Msk                     (0x1U << AON_MEM_CTL_CRITICAL_MEM_WM_Pos)
#define AON_MEM_CTL_CRITICAL_MEM_WM                         AON_MEM_CTL_CRITICAL_MEM_WM_Msk

#define AON_MEM_CTL_CRITICAL_MEM_RM_Pos                     (0U)
#define AON_MEM_CTL_CRITICAL_MEM_RM_Len                     (1U)
#define AON_MEM_CTL_CRITICAL_MEM_RM_Msk                     (0x1U << AON_MEM_CTL_CRITICAL_MEM_RM_Pos)
#define AON_MEM_CTL_CRITICAL_MEM_RM                         AON_MEM_CTL_CRITICAL_MEM_RM_Msk

/*********************  Bit definition for AON_REG_EXT_WKUP_CTL register  ************************************/
#define AON_EXT_WKUP_CTL_WDT_ALARM_Pos                      (27U)
#define AON_EXT_WKUP_CTL_WDT_ALARM_Len                      (5U)
#define AON_EXT_WKUP_CTL_WDT_ALARM_Msk                      (0x1FU << AON_EXT_WKUP_CTL_WDT_ALARM_Pos)
#define AON_EXT_WKUP_CTL_WDT_ALARM                          AON_EXT_WKUP_CTL_WDT_ALARM_Msk

#define AON_EXT_WKUP_CTL_WDT_RUNNING_Pos                    (26U)
#define AON_EXT_WKUP_CTL_WDT_RUNNING_Len                    (1U)
#define AON_EXT_WKUP_CTL_WDT_RUNNING_Msk                    (0x1U << AON_EXT_WKUP_CTL_WDT_RUNNING_Pos)
#define AON_EXT_WKUP_CTL_WDT_RUNNING                        AON_EXT_WKUP_CTL_WDT_RUNNING_Msk

#define AON_EXT_WKUP_CTL_WDT_RELOAD_Pos                     (25U)
#define AON_EXT_WKUP_CTL_WDT_RELOAD_Len                     (1U)
#define AON_EXT_WKUP_CTL_WDT_RELOAD_Msk                     (0x1U << AON_EXT_WKUP_CTL_WDT_RELOAD_Pos)
#define AON_EXT_WKUP_CTL_WDT_RELOAD                         AON_EXT_WKUP_CTL_WDT_RELOAD_Msk

#define AON_EXT_WKUP_CTL_WDT_EN_Pos                         (24U)
#define AON_EXT_WKUP_CTL_WDT_EN_Len                         (1U)
#define AON_EXT_WKUP_CTL_WDT_EN_Msk                         (0x1U << AON_EXT_WKUP_CTL_WDT_EN_Pos)
#define AON_EXT_WKUP_CTL_WDT_EN                             AON_EXT_WKUP_CTL_WDT_EN_Msk

#define AON_EXT_WKUP_CTL_TYPE_Pos                           (16U)
#define AON_EXT_WKUP_CTL_TYPE_Len                           (8U)
#define AON_EXT_WKUP_CTL_TYPE_Msk                           (0xFFU << AON_EXT_WKUP_CTL_TYPE_Pos)
#define AON_EXT_WKUP_CTL_TYPE                               AON_EXT_WKUP_CTL_TYPE_Msk

#define AON_EXT_WKUP_CTL_INVERT_Pos                         (8U)
#define AON_EXT_WKUP_CTL_INVERT_Len                         (8U)
#define AON_EXT_WKUP_CTL_INVERT_Msk                         (0xFFU << AON_EXT_WKUP_CTL_INVERT_Pos)
#define AON_EXT_WKUP_CTL_INVERT                             AON_EXT_WKUP_CTL_INVERT_Msk

#define AON_EXT_WKUP_CTL_SRC_EN_Pos                         (0U)
#define AON_EXT_WKUP_CTL_SRC_EN_Len                         (8U)
#define AON_EXT_WKUP_CTL_SRC_EN_Msk                         (0xFFU << AON_EXT_WKUP_CTL_SRC_EN_Pos)
#define AON_EXT_WKUP_CTL_SRC_EN                             AON_EXT_WKUP_CTL_SRC_EN_Msk

/*********************  Bit definition for AON_REG_AON_PAD_CTL1 register  ***************************************/
#define AON_PAD_CTL1_TIMER_READ_SEL_Pos                     (30U)
#define AON_PAD_CTL1_TIMER_READ_SEL_Len                     (2U)
#define AON_PAD_CTL1_TIMER_READ_SEL_Msk                     (0x3U << AON_PAD_CTL1_TIMER_READ_SEL_Pos)
#define AON_PAD_CTL1_TIMER_READ_SEL                         AON_PAD_CTL1_TIMER_READ_SEL_Msk
#define AON_PAD_CTL1_TIMER_READ_SEL_CAL_TIMER               (0x0U << AON_PAD_CTL1_TIMER_READ_SEL_Pos)
#define AON_PAD_CTL1_TIMER_READ_SEL_AON_WDT                 (0x1U << AON_PAD_CTL1_TIMER_READ_SEL_Pos)
#define AON_PAD_CTL1_TIMER_READ_SEL_SLP_TIMER               (0x2U << AON_PAD_CTL1_TIMER_READ_SEL_Pos)
#define AON_PAD_CTL1_TIMER_READ_SEL_CAL_ALARM               (0x3U << AON_PAD_CTL1_TIMER_READ_SEL_Pos)

#define AON_PAD_CTL1_MEM_STDBY_VDDISO_OVR_EN_Pos            (25U)
#define AON_PAD_CTL1_MEM_STDBY_VDDISO_OVR_EN_Len            (1U)
#define AON_PAD_CTL1_MEM_STDBY_VDDISO_OVR_EN_Msk            (0x1U << AON_PAD_CTL1_MEM_STDBY_VDDISO_OVR_EN_Pos)
#define AON_PAD_CTL1_MEM_STDBY_VDDISO_OVR_EN                AON_PAD_CTL1_MEM_STDBY_VDDISO_OVR_EN_Msk

#define AON_PAD_CTL1_O_AON_GPI_Pos                          (16U)
#define AON_PAD_CTL1_O_AON_GPI_Len                          (6U)
#define AON_PAD_CTL1_O_AON_GPI_Msk                          (0x3FU <<  AON_PAD_CTL1_O_AON_GPI_Pos)
#define AON_PAD_CTL1_O_AON_GPI                              AON_PAD_CTL1_O_AON_GPI_Msk

#define AON_PAD_CTL1_AON_GPO_Pos                            (8U)
#define AON_PAD_CTL1_AON_GPO_Len                            (8U)
#define AON_PAD_CTL1_AON_GPO_Msk                            (0xFFU << AON_PAD_CTL1_AON_GPO_Pos)
#define AON_PAD_CTL1_AON_GPO                                AON_PAD_CTL1_AON_GPO_Msk

#define AON_PAD_CTL1_AON_GPO_OE_N_Pos                       (0U)
#define AON_PAD_CTL1_AON_GPO_OE_N_Len                       (8U)
#define AON_PAD_CTL1_AON_GPO_OE_N_Msk                       (0xFFU << AON_PAD_CTL1_AON_GPO_OE_N_Pos)
#define AON_PAD_CTL1_AON_GPO_OE_N                           AON_PAD_CTL1_AON_GPO_OE_N_Msk

/*********************  Bit definition for AON_REG_MEM_PWR_SLP register  **************************************/
#define AON_MEM_PWR_SLP_PD_MCU_KEYRAM_Pos                   (28U)
#define AON_MEM_PWR_SLP_PD_MCU_KEYRAM_Len                   (2U)
#define AON_MEM_PWR_SLP_PD_MCU_KEYRAM_Msk                   (0x3U << AON_MEM_PWR_SLP_PD_MCU_KEYRAM_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_KEYRAM                       AON_MEM_PWR_SLP_PD_MCU_KEYRAM_Msk

#define AON_MEM_PWR_SLP_PD_PACKET_MEM_Pos                   (26U)
#define AON_MEM_PWR_SLP_PD_PACKET_MEM_Len                   (2U)
#define AON_MEM_PWR_SLP_PD_PACKET_MEM_Msk                   (0x3U << AON_MEM_PWR_SLP_PD_PACKET_MEM_Pos)
#define AON_MEM_PWR_SLP_PD_PACKET_MEM                       AON_MEM_PWR_SLP_PD_PACKET_MEM_Msk

#define AON_MEM_PWR_SLP_PD_MCU_ICACHE_Pos                   (24U)
#define AON_MEM_PWR_SLP_PD_MCU_ICACHE_Len                   (2U)
#define AON_MEM_PWR_SLP_PD_MCU_ICACHE_Msk                   (0x3U << AON_MEM_PWR_SLP_PD_MCU_ICACHE_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_ICACHE                       AON_MEM_PWR_SLP_PD_MCU_ICACHE_Msk

#define AON_MEM_PWR_SLP_PD_MCU_HTM_Pos                      (22U)
#define AON_MEM_PWR_SLP_PD_MCU_HTM_Len                      (2U)
#define AON_MEM_PWR_SLP_PD_MCU_HTM_Msk                      (0x3U << AON_MEM_PWR_SLP_PD_MCU_HTM_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_HTM                          AON_MEM_PWR_SLP_PD_MCU_HTM_Msk

#define AON_MEM_PWR_SLP_PD_MCU_10_Pos                       (20U)
#define AON_MEM_PWR_SLP_PD_MCU_10_Len                       (2U)
#define AON_MEM_PWR_SLP_PD_MCU_10_Msk                       (0x3U << AON_MEM_PWR_SLP_PD_MCU_10_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_10                           AON_MEM_PWR_SLP_PD_MCU_10_Msk

#define AON_MEM_PWR_SLP_PD_MCU_09_Pos                       (18U)
#define AON_MEM_PWR_SLP_PD_MCU_09_Len                       (2U)
#define AON_MEM_PWR_SLP_PD_MCU_09_Msk                       (0x3U << AON_MEM_PWR_SLP_PD_MCU_09_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_09                           AON_MEM_PWR_SLP_PD_MCU_09_Msk

#define AON_MEM_PWR_SLP_PD_MCU_08_Pos                       (16U)
#define AON_MEM_PWR_SLP_PD_MCU_08_Len                       (2U)
#define AON_MEM_PWR_SLP_PD_MCU_08_Msk                       (0x3U << AON_MEM_PWR_SLP_PD_MCU_08_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_08                           AON_MEM_PWR_SLP_PD_MCU_08_Msk

#define AON_MEM_PWR_SLP_PD_MCU_07_Pos                       (14U)
#define AON_MEM_PWR_SLP_PD_MCU_07_Len                       (2U)
#define AON_MEM_PWR_SLP_PD_MCU_07_Msk                       (0x3U << AON_MEM_PWR_SLP_PD_MCU_07_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_07                           AON_MEM_PWR_SLP_PD_MCU_07_Msk

#define AON_MEM_PWR_SLP_PD_MCU_06_Pos                       (12U)
#define AON_MEM_PWR_SLP_PD_MCU_06_Len                       (2U)
#define AON_MEM_PWR_SLP_PD_MCU_06_Msk                       (0x3U << AON_MEM_PWR_SLP_PD_MCU_06_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_06                           AON_MEM_PWR_SLP_PD_MCU_06_Msk

#define AON_MEM_PWR_SLP_PD_MCU_05_Pos                       (10U)
#define AON_MEM_PWR_SLP_PD_MCU_05_Len                       (2U)
#define AON_MEM_PWR_SLP_PD_MCU_05_Msk                       (0x3U << AON_MEM_PWR_SLP_PD_MCU_05_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_05                           AON_MEM_PWR_SLP_PD_MCU_05_Msk

#define AON_MEM_PWR_SLP_PD_MCU_04_Pos                       (8U)
#define AON_MEM_PWR_SLP_PD_MCU_04_Len                       (2U)
#define AON_MEM_PWR_SLP_PD_MCU_04_Msk                       (0x3U << AON_MEM_PWR_SLP_PD_MCU_04_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_04                           AON_MEM_PWR_SLP_PD_MCU_04_Msk

#define AON_MEM_PWR_SLP_PD_MCU_03_Pos                       (6U)
#define AON_MEM_PWR_SLP_PD_MCU_03_Len                       (2U)
#define AON_MEM_PWR_SLP_PD_MCU_03_Msk                       (0x3U << AON_MEM_PWR_SLP_PD_MCU_03_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_03                           AON_MEM_PWR_SLP_PD_MCU_03_Msk

#define AON_MEM_PWR_SLP_PD_MCU_02_Pos                       (4U)
#define AON_MEM_PWR_SLP_PD_MCU_02_Len                       (2U)
#define AON_MEM_PWR_SLP_PD_MCU_02_Msk                       (0x3U << AON_MEM_PWR_SLP_PD_MCU_02_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_02                           AON_MEM_PWR_SLP_PD_MCU_02_Msk

#define AON_MEM_PWR_SLP_PD_MCU_01_Pos                       (2U)
#define AON_MEM_PWR_SLP_PD_MCU_01_Len                       (2U)
#define AON_MEM_PWR_SLP_PD_MCU_01_Msk                       (0x3U << AON_MEM_PWR_SLP_PD_MCU_01_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_01                           AON_MEM_PWR_SLP_PD_MCU_01_Msk

#define AON_MEM_PWR_SLP_PD_MCU_00_Pos                       (0U)
#define AON_MEM_PWR_SLP_PD_MCU_00_Len                       (2U)
#define AON_MEM_PWR_SLP_PD_MCU_00_Msk                       (0x3U << AON_MEM_PWR_SLP_PD_MCU_00_Pos)
#define AON_MEM_PWR_SLP_PD_MCU_00                           AON_MEM_PWR_SLP_PD_MCU_00_Msk

/****************************  Bit definition for AON_REG_MEM_PWR_WKUP register  *****************************/
#define AON_MEM_PWR_WKUP_PD_MCU_KEYRAM_Pos                  (28U)
#define AON_MEM_PWR_WKUP_PD_MCU_KEYRAM_Len                  (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_KEYRAM_Msk                  (0x3U << AON_MEM_PWR_WKUP_PD_MCU_KEYRAM_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_KEYRAM                      AON_MEM_PWR_WKUP_PD_MCU_KEYRAM_Msk

#define AON_MEM_PWR_WKUP_PD_PACKET_MEM_Pos                  (26U)
#define AON_MEM_PWR_WKUP_PD_PACKET_MEM_Len                  (2U)
#define AON_MEM_PWR_WKUP_PD_PACKET_MEM_Msk                  (0x3U << AON_MEM_PWR_WKUP_PD_PACKET_MEM_Pos)
#define AON_MEM_PWR_WKUP_PD_PACKET_MEM                      AON_MEM_PWR_WKUP_PD_PACKET_MEM_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_ICACHE_Pos                  (24U)
#define AON_MEM_PWR_WKUP_PD_MCU_ICACHE_Len                  (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_ICACHE_Msk                  (0x3U << AON_MEM_PWR_WKUP_PD_MCU_ICACHE_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_ICACHE                      AON_MEM_PWR_WKUP_PD_MCU_ICACHE_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_HTM_Pos                     (22U)
#define AON_MEM_PWR_WKUP_PD_MCU_HTM_Len                     (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_HTM_Msk                     (0x3U << AON_MEM_PWR_WKUP_PD_MCU_HTM_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_HTM                         AON_MEM_PWR_WKUP_PD_MCU_HTM_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_10_Pos                      (20U)
#define AON_MEM_PWR_WKUP_PD_MCU_10_Len                      (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_10_Msk                      (0x3U << AON_MEM_PWR_WKUP_PD_MCU_10_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_10                          AON_MEM_PWR_WKUP_PD_MCU_10_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_09_Pos                      (18U)
#define AON_MEM_PWR_WKUP_PD_MCU_09_Len                      (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_09_Msk                      (0x3U << AON_MEM_PWR_WKUP_PD_MCU_09_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_09                          AON_MEM_PWR_WKUP_PD_MCU_09_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_08_Pos                      (16U)
#define AON_MEM_PWR_WKUP_PD_MCU_08_Len                      (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_08_Msk                      (0x3U << AON_MEM_PWR_WKUP_PD_MCU_08_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_08                          AON_MEM_PWR_WKUP_PD_MCU_08_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_07_Pos                      (14U)
#define AON_MEM_PWR_WKUP_PD_MCU_07_Len                      (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_07_Msk                      (0x3U << AON_MEM_PWR_WKUP_PD_MCU_07_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_07                          AON_MEM_PWR_WKUP_PD_MCU_07_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_06_Pos                      (12U)
#define AON_MEM_PWR_WKUP_PD_MCU_06_Len                      (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_06_Msk                      (0x3U << AON_MEM_PWR_WKUP_PD_MCU_06_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_06                          AON_MEM_PWR_WKUP_PD_MCU_06_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_05_Pos                      (10U)
#define AON_MEM_PWR_WKUP_PD_MCU_05_Len                      (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_05_Msk                      (0x3U << AON_MEM_PWR_WKUP_PD_MCU_05_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_05                          AON_MEM_PWR_WKUP_PD_MCU_05_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_04_Pos                      (8U)
#define AON_MEM_PWR_WKUP_PD_MCU_04_Len                      (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_04_Msk                      (0x3U << AON_MEM_PWR_WKUP_PD_MCU_04_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_04                          AON_MEM_PWR_WKUP_PD_MCU_04_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_03_Pos                      (6U)
#define AON_MEM_PWR_WKUP_PD_MCU_03_Len                      (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_03_Msk                      (0x3U << AON_MEM_PWR_WKUP_PD_MCU_03_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_03                          AON_MEM_PWR_WKUP_PD_MCU_03_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_02_Pos                      (4U)
#define AON_MEM_PWR_WKUP_PD_MCU_02_Len                      (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_02_Msk                      (0x3U << AON_MEM_PWR_WKUP_PD_MCU_02_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_02                          AON_MEM_PWR_WKUP_PD_MCU_02_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_01_Pos                      (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_01_Len                      (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_01_Msk                      (0x3U << AON_MEM_PWR_WKUP_PD_MCU_01_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_01                          AON_MEM_PWR_WKUP_PD_MCU_01_Msk

#define AON_MEM_PWR_WKUP_PD_MCU_00_Pos                      (0U)
#define AON_MEM_PWR_WKUP_PD_MCU_00_Len                      (2U)
#define AON_MEM_PWR_WKUP_PD_MCU_00_Msk                      (0x3U << AON_MEM_PWR_WKUP_PD_MCU_00_Pos)
#define AON_MEM_PWR_WKUP_PD_MCU_00                          AON_MEM_PWR_WKUP_PD_MCU_00_Msk

/***********************  Bit definition for AON_REG_PWR_RET28 register  ***************************/
#define AON_COMM_TMR_DEEPSLWKUP_DEEPSLTIME_Pos              (0U)
#define AON_COMM_TMR_DEEPSLWKUP_DEEPSLTIME_Len              (32U)
#define AON_COMM_TMR_DEEPSLWKUP_DEEPSLTIME_Msk              (0xFFFFFFFFU)
#define AON_COMM_TMR_DEEPSLWKUP_DEEPSLTIME                  AON_COMM_TMR_DEEPSLWKUP_DEEPSLTIME_Msk

/***********************  Bit definition for AON_REG_PWR_RET29 register  ************************/
#define AON_COMM_TMR_ENBPRESET_TWEXT_Pos                    (21U)
#define AON_COMM_TMR_ENBPRESET_TWEXT_Len                    (11U)
#define AON_COMM_TMR_ENBPRESET_TWEXT_Msk                    (0x07FFU << AON_COMM_TMR_ENBPRESET_TWEXT_Pos)
#define AON_COMM_TMR_ENBPRESET_TWEXT                        AON_COMM_TMR_ENBPRESET_TWEXT_Msk

#define AON_COMM_TMR_ENBPRESET_TWOSC_Pos                    (10U)
#define AON_COMM_TMR_ENBPRESET_TWOSC_Len                    (11U)
#define AON_COMM_TMR_ENBPRESET_TWOSC_Msk                    (0x07FFU << AON_COMM_TMR_ENBPRESET_TWOSC_Pos)
#define AON_COMM_TMR_ENBPRESET_TWOSC                        AON_COMM_TMR_ENBPRESET_TWOSC_Msk

#define AON_COMM_TMR_ENBPRESET_TWRM_Pos                     (0U)
#define AON_COMM_TMR_ENBPRESET_TWRM_Len                     (10U)
#define AON_COMM_TMR_ENBPRESET_TWRM_Msk                     (0x03FFU << AON_COMM_TMR_ENBPRESET_TWRM_Pos)
#define AON_COMM_TMR_ENBPRESET_TWRM                         AON_COMM_TMR_ENBPRESET_TWRM_Msk

/***********************  Bit definition for AON_REG_PWR_RET31 register  *********************************/
#define AON_PWR_REG31_FPGA_DBG_MUX_SEL_Pos                  (0U)
#define AON_PWR_REG31_FPGA_DBG_MUX_SEL_Len                  (2U)
#define AON_PWR_REG31_FPGA_DBG_MUX_SEL_Msk                  (0x3U <<  AON_PWR_REG31_FPGA_DBG_MUX_SEL_Pos)
#define AON_PWR_REG31_FPGA_DBG_MUX_SEL                      AON_PWR_REG31_FPGA_DBG_MUX_SEL_Msk

/***********************  Bit definition for AON_REG_PSC_CMD register  ***************************************/
#define AON_PSC_CMD_MCU_PWR_BUSY_Pos                        (1U)
#define AON_PSC_CMD_MCU_PWR_BUSY_Len                        (1U)
#define AON_PSC_CMD_MCU_PWR_BUSY_Msk                        (0x1U << AON_PSC_CMD_MCU_PWR_BUSY_Pos)
#define AON_PSC_CMD_MCU_PWR_BUSY                            AON_PSC_CMD_MCU_PWR_BUSY_Msk

#define AON_PSC_CMD_MCU_PWR_REQ_Pos                         (0U)
#define AON_PSC_CMD_MCU_PWR_REQ_Len                         (1U)
#define AON_PSC_CMD_MCU_PWR_REQ_Msk                         (0x1U << AON_PSC_CMD_MCU_PWR_REQ_Pos)
#define AON_PSC_CMD_MCU_PWR_REQ                             AON_PSC_CMD_MCU_PWR_REQ_Msk

/**********************  Bit definition for AON_REG_PSC_CMD_OPC register  ************************************/
#define AON_PSC_CMD_OPC_OPCODE_Pos                          (0U)
#define AON_PSC_CMD_OPC_OPCODE_Len                          (8U)
#define AON_PSC_CMD_OPC_OPCODE_Msk                          (0xFFU <<  AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE                              AON_PSC_CMD_OPC_OPCODE_Msk
#define AON_PSC_CMD_OPC_OPCODE_LOOPBACK                     (0x0U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_EF_DIR_ON                    (0x1U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_32_TIMER_LD                  (0x2U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_DEEP_SLEEP                   (0x3U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_EF_DIR_OFF                   (0x4U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_EXT_CLK                      (0x5U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_RNG_CLK                      (0x6U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_RTC_CLK                      (0x7U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_RNG2_CLK                     (0x8U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_LD_MEM_SLP_CFG               (0x9U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_LD_MEM_WKUP_CFG              (0xAU << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_DPAD_LE_HI                    (0xBU << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_DPAD_LE_LO                    (0xCU << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_SLP_TIMER_MODE_0             (0x10U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_SLP_TIMER_MODE_1             (0x11U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_SLP_TIMER_MODE_2             (0x12U << AON_PSC_CMD_OPC_OPCODE_Pos)
#define AON_PSC_CMD_OPC_OPCODE_SLP_TIMER_MODE_3             (0x13U << AON_PSC_CMD_OPC_OPCODE_Pos)

/*******************  Bit definition for AON_REG_TIMER_VALUE register  **********/
#define AON_TIMER_VALUE_PWR_CTL_TIMER_32B_Pos               (0U)
#define AON_TIMER_VALUE_PWR_CTL_TIMER_32B_Len               (32U)
#define AON_TIMER_VALUE_PWR_CTL_TIMER_32B_Msk               (0xFFFFFFFFU)
#define AON_TIMER_VALUE_PWR_CTL_TIMER_32B                   AON_TIMER_VALUE_PWR_CTL_TIMER_32B_Msk

/*********************  Bit definition for AON_REG_TIMER_VAL register  **************************************/
#define AON_TIMER_VAL_READ_Pos                              (0U)
#define AON_TIMER_VAL_READ_Len                              (32U)
#define AON_TIMER_VAL_READ_Msk                              (0xFFFFFFFFU)
#define AON_TIMER_VAL_READ                                  AON_TIMER_VAL_READ_Msk

/***********************  Bit definition for AON_REG_FPGA_CTRL register  *********************************/
#define AON_REG_FPGA_CTRL_MUX_SEL_Pos                       (0U)
#define AON_REG_FPGA_CTRL_MUX_SEL_Len                       (2U)
#define AON_REG_FPGA_CTRL_MUX_SEL_Msk                       (0x3U << AON_REG_FPGA_CTRL_MUX_SEL_Pos)
#define AON_REG_FPGA_CTRL_MUX_SEL                           AON_REG_FPGA_CTRL_MUX_SEL_Msk

#define AON_REG_FPGA_CTRL_EXIST_Pos                         (4U)
#define AON_REG_FPGA_CTRL_EXIST_Len                         (1U)
#define AON_REG_FPGA_CTRL_EXIST_Msk                         (0x1U << AON_REG_FPGA_CTRL_EXIST_Pos)
#define AON_REG_FPGA_CTRL_EXIST                             AON_REG_FPGA_CTRL_EXIST_Msk

/**********************  Bit definition for AON_REG_ST_CALIB_REG register  ***********************************/
#define AON_ST_CALIB_REG_STCALIB_Pos                        (0U)
#define AON_ST_CALIB_REG_STCALIB_Len                        (27U)
#define AON_ST_CALIB_REG_STCALIB_Msk                        (0x7FFFFFFU <<  AON_ST_CALIB_REG_STCALIB_Pos)
#define AON_ST_CALIB_REG_STCALIB                            AON_ST_CALIB_REG_STCALIB_Msk


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

/*******************  Bit definition for DMA_CTLL register  *******************/
#define DMA_CTLL_TT_FC_Pos                                  (20U)
#define DMA_CTLL_TT_FC_Len                                  (2U)
#define DMA_CTLL_TT_FC_Msk                                  (0x3U << DMA_CTLL_TT_FC_Pos)
#define DMA_CTLL_TT_FC                                      DMA_CTLL_TT_FC_Msk
#define DMA_CTLL_TT_FC_M2M                                  (0x0U << DMA_CTLL_TT_FC_Pos)
#define DMA_CTLL_TT_FC_M2P                                  (0x1U << DMA_CTLL_TT_FC_Pos)
#define DMA_CTLL_TT_FC_P2M                                  (0x2U << DMA_CTLL_TT_FC_Pos)
#define DMA_CTLL_TT_FC_P2P                                  (0x3U << DMA_CTLL_TT_FC_Pos)

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

/*******************  Bit definition for DMA_CFGL register  *******************/
#define DMA_CFGL_RELOAD_DST_Pos                             (31U)
#define DMA_CFGL_RELOAD_DST_Len                             (1U)
#define DMA_CFGL_RELOAD_DST_Msk                             (0x1U << DMA_CFGL_RELOAD_DST_Pos)
#define DMA_CFGL_RELOAD_DST                                 DMA_CFGL_RELOAD_DST_Msk

#define DMA_CFGL_RELOAD_SRC_Pos                             (30U)
#define DMA_CFGL_RELOAD_SRC_Len                             (1U)
#define DMA_CFGL_RELOAD_SRC_Msk                             (0x1U << DMA_CFGL_RELOAD_SRC_Pos)
#define DMA_CFGL_RELOAD_SRC                                 DMA_CFGL_RELOAD_SRC_Msk

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

#define DMA_CFGH_PROTCTL_Pos                                (2U)
#define DMA_CFGH_PROTCTL_Len                                (3U)
#define DMA_CFGH_PROTCTL_Msk                                (0x7U << DMA_CFGH_PROTCTL_Pos)
#define DMA_CFGH_PROTCTL                                    DMA_CFGH_PROTCTL_Msk

#define DMA_CFGH_FIFO_MODE_Pos                              (2U)
#define DMA_CFGH_FIFO_MODE_Len                              (1U)
#define DMA_CFGH_FIFO_MODE_Msk                              (0x1U << DMA_CFGH_FIFO_MODE_Pos)
#define DMA_CFGH_FIFO_MODE                                  DMA_CFGH_FIFO_MODE_Msk

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
/* ================                                    DUAL_TIMER                                   ================ */
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

/*******************  Bit definition for HMAC_CTRL register  ******************/
#define HMAC_CTRL_ENABLE_Pos                                (0U)
#define HMAC_CTRL_ENABLE_Len                                (1U)
#define HMAC_CTRL_ENABLE_Msk                                (1U << HMAC_CTRL_ENABLE_Pos)
#define HMAC_CTRL_ENABLE                                    HMAC_CTRL_ENABLE_Msk

#define HMAC_CTRL_START_DMA_Pos                             (1U)
#define HMAC_CTRL_START_DMA_Len                             (1U)
#define HMAC_CTRL_START_DMA_Msk                             (1U << HMAC_CTRL_START_DMA_Pos)
#define HMAC_CTRL_START_DMA                                 HMAC_CTRL_START_DMA_Msk

#define HMAC_CTRL_ENABLE_RKEY_Pos                           (2U)
#define HMAC_CTRL_ENABLE_RKEY_Len                           (1U)
#define HMAC_CTRL_ENABLE_RKEY_Msk                           (1U << HMAC_CTRL_ENABLE_RKEY_Pos)
#define HMAC_CTRL_ENABLE_RKEY                               HMAC_CTRL_ENABLE_RKEY_Msk

#define HMAC_CTRL_LASTTRANSFER_Pos                          (3U)
#define HMAC_CTRL_LASTTRANSFER_Len                          (1U)
#define HMAC_CTRL_LASTTRANSFER_Msk                          (1U << HMAC_CTRL_LASTTRANSFER_Pos)
#define HMAC_CTRL_LASTTRANSFER                              HMAC_CTRL_LASTTRANSFER_Msk

/*******************  Bit definition for HMAC_CONFIG register  ****************/
#define HMAC_CONFIG_ENABLE_USERHASH_Pos                     (0U)
#define HMAC_CONFIG_ENABLE_USERHASH_Len                     (1U)
#define HMAC_CONFIG_ENABLE_USERHASH_Msk                     (1U << HMAC_CONFIG_ENABLE_USERHASH_Pos)
#define HMAC_CONFIG_ENABLE_USERHASH                         HMAC_CONFIG_ENABLE_USERHASH_Msk

#define HMAC_CONFIG_ENDIAN_Pos                              (1U)
#define HMAC_CONFIG_ENDIAN_Len                              (1U)
#define HMAC_CONFIG_ENDIAN_Msk                              (1U << HMAC_CONFIG_ENDIAN_Pos)
#define HMAC_CONFIG_ENDIAN                                  HMAC_CONFIG_ENDIAN_Msk

#define HMAC_CONFIG_KEYTYPE_Pos                             (2U)
#define HMAC_CONFIG_KEYTYPE_Len                             (2U)
#define HMAC_CONFIG_KEYTYPE_Msk                             (3U << HMAC_CONFIG_KEYTYPE_Pos)
#define HMAC_CONFIG_KEYTYPE                                 HMAC_CONFIG_KEYTYPE_Msk

#define HMAC_CONFIG_CALCTYPE_Pos                            (4U)
#define HMAC_CONFIG_CALCTYPE_Len                            (1U)
#define HMAC_CONFIG_CALCTYPE_Msk                            (1U << HMAC_CONFIG_CALCTYPE_Pos)
#define HMAC_CONFIG_CALCTYPE                                HMAC_CONFIG_CALCTYPE_Msk

#define HMAC_CONFIG_PRIVATE_Pos                             (5U)
#define HMAC_CONFIG_PRIVATE_Len                             (1U)
#define HMAC_CONFIG_PRIVATE_Msk                             (1U << HMAC_CONFIG_PRIVATE_Pos)
#define HMAC_CONFIG_PRIVATE                                 HMAC_CONFIG_PRIVATE_Msk

/*******************  Bit definition for HMAC_STATUS register  ****************/
#define HMAC_STATUS_DATAREADY_SHA_Pos                       (0U)
#define HMAC_STATUS_DATAREADY_SHA_Len                       (1U)
#define HMAC_STATUS_DATAREADY_SHA_Msk                       (1U << HMAC_STATUS_DATAREADY_SHA_Pos)
#define HMAC_STATUS_DATAREADY_SHA                           HMAC_STATUS_DATAREADY_SHA_Msk

#define HMAC_STATUS_MESSAGEDONE_DMA_Pos                     (1U)
#define HMAC_STATUS_MESSAGEDONE_DMA_Len                     (1U)
#define HMAC_STATUS_MESSAGEDONE_DMA_Msk                     (1U << HMAC_STATUS_MESSAGEDONE_DMA_Pos)
#define HMAC_STATUS_MESSAGEDONE_DMA                         HMAC_STATUS_MESSAGEDONE_DMA_Msk

#define HMAC_STATUS_TRANSERR_DMA_Pos                        (2U)
#define HMAC_STATUS_TRANSERR_DMA_Len                        (1U)
#define HMAC_STATUS_TRANSERR_DMA_Msk                        (1U << HMAC_STATUS_TRANSERR_DMA_Pos)
#define HMAC_STATUS_TRANSERR_DMA                            HMAC_STATUS_TRANSERR_DMA_Msk

#define HMAC_STATUS_KEYVALID_Pos                            (3U)
#define HMAC_STATUS_KEYVALID_Len                            (1U)
#define HMAC_STATUS_KEYVALID_Msk                            (1U << HMAC_STATUS_KEYVALID_Pos)
#define HMAC_STATUS_KEYVALID                                HMAC_STATUS_KEYVALID_Msk

#define HMAC_STATUS_DATAREADY_HMAC_Pos                      (4U)
#define HMAC_STATUS_DATAREADY_HMAC_Len                      (1U)
#define HMAC_STATUS_DATAREADY_HMAC_Msk                      (1U << HMAC_STATUS_DATAREADY_HMAC_Pos)
#define HMAC_STATUS_DATAREADY_HMAC                          HMAC_STATUS_DATAREADY_HMAC_Msk

#define HMAC_STATUS_TRANSDONE_DMA_Pos                       (5U)
#define HMAC_STATUS_TRANSDONE_DMA_Len                       (1U)
#define HMAC_STATUS_TRANSDONE_DMA_Msk                       (1U << HMAC_STATUS_TRANSDONE_DMA_Pos)
#define HMAC_STATUS_TRANSDONE_DMA                           HMAC_STATUS_TRANSDONE_DMA_Msk

/*******************  Bit definition for HMAC_TRAN_SIZE register  *************/
#define HMAC_TRANSIZE_Pos                                   (0U)
#define HMAC_TRANSIZE_Len                                   (15U)
#define HMAC_TRANSIZE_Msk                                   (0x7FFFU << HMAC_TRANSIZE_Pos)
#define HMAC_TRANSIZE                                       HMAC_TRANSIZE_Msk

/*******************  Bit definition for HMAC_INTERRUPT register  *************/
#define HMAC_INTERRUPT_DONE_Pos                             (0U)
#define HMAC_INTERRUPT_DONE_Len                             (1U)
#define HMAC_INTERRUPT_DONE_Msk                             (1U << HMAC_INTERRUPT_DONE_Pos)
#define HMAC_INTERRUPT_DONE                                 HMAC_INTERRUPT_DONE_Msk

#define HMAC_INTERRUPT_ENABLE_Pos                           (1U)
#define HMAC_INTERRUPT_ENABLE_Len                           (1U)
#define HMAC_INTERRUPT_ENABLE_Msk                           (1U << HMAC_INTERRUPT_ENABLE_Pos)
#define HMAC_INTERRUPT_ENABLE                               HMAC_INTERRUPT_ENABLE_Msk

/*******************  Bit definition for HMAC_RSTART_ADDR register  ***********/
#define HMAC_RSTART_ADDR_Pos                                (0U)
#define HMAC_RSTART_ADDR_Len                                (32U)
#define HMAC_RSTART_ADDR_Msk                                (0xFFFFFFFFU << HMAC_RSTART_ADDR_Pos)
#define HMAC_RSTART_ADDR                                    HMAC_RSTART_ADDR_Msk

/*******************  Bit definition for HMAC_WSTART_ADDR register  ***********/
#define HMAC_WSTART_ADDR_Pos                                (0U)
#define HMAC_WSTART_ADDR_Len                                (32U)
#define HMAC_WSTART_ADDR_Msk                                (0xFFFFFFFFU << HMAC_WSTART_ADDR_Pos)
#define HMAC_WSTART_ADDR                                    HMAC_WSTART_ADDR_Msk

/*******************  Bit definition for HMAC_USER_HASH register  *************/
#define HMAC_USERHASH_Pos                                   (0U)
#define HMAC_USERHASH_Len                                   (32U)
#define HMAC_USERHASH_Msk                                   (0xFFFFFFFFU << HMAC_USERHASH_Pos)
#define HMAC_USERHASH                                       HMAC_USERHASH_Msk

/*******************  Bit definition for HMAC_FIFO_OUT register  **************/
#define HMAC_FIFO_OUT_Pos                                   (0U)
#define HMAC_FIFO_OUT_Len                                   (32U)
#define HMAC_FIFO_OUT_Msk                                   (0xFFFFFFFFU << HMAC_FIFO_OUT_Pos)
#define HMAC_FIFO_OUT                                       HMAC_FIFO_OUT_Msk

/*******************  Bit definition for HMAC_MESSAGE_FIFO register  **********/
#define HMAC_FIFO_MESSAGE_Pos                               (0U)
#define HMAC_FIFO_MESSAGE_Len                               (32U)
#define HMAC_FIFO_MESSAGE_Msk                               (0xFFFFFFFFU << HMAC_FIFO_MESSAGE_Pos)
#define HMAC_FIFO_MESSAGE                                   HMAC_FIFO_MESSAGE_Msk

/*******************  Bit definition for HMAC_KEY register  *******************/
#define HMAC_KEY_Pos                                        (0U)
#define HMAC_KEY_Len                                        (32U)
#define HMAC_KEY_Msk                                        (0xFFFFFFFFU << HMAC_KEY_Pos)
#define HMAC_KEY                                            HMAC_KEY_Msk

/*******************  Bit definition for HMAC_KEY_ADDR register  **************/
#define HMAC_KEY_ADDR_Pos                                   (0U)
#define HMAC_KEY_ADDR_Len                                   (32U)
#define HMAC_KEY_ADDR_Msk                                   (0xFFFFFFFFU << HMAC_KEY_ADDR_Pos)
#define HMAC_KEY_ADDR                                       HMAC_KEY_ADDR_Msk

/*******************  Bit definition for HMAC_KPORT_MASK register  ************/
#define HMAC_KPORT_MASK_Pos                                 (0U)
#define HMAC_KPORT_MASK_Len                                 (32U)
#define HMAC_KPORT_MASK_Msk                                 (0xFFFFFFFFU << HMAC_KPORT_MASK_Pos)
#define HMAC_KPORT_MASK                                     HMAC_KPORT_MASK_Msk


/* ================================================================================================================= */
/* ================                                        I2C                                      ================ */
/* ================================================================================================================= */
#define I2C_TXFIFO_SIZE                                     (8U)
#define I2C_RXFIFO_SIZE                                     (8U)

/*******************  Bit definition for IC_CON register  *********************/
#define I2C_CON_BUS_CLR_FEATURE_CTRL_Pos                    (11U)
#define I2C_CON_BUS_CLR_FEATURE_CTRL_Len                    (1U)
#define I2C_CON_BUS_CLR_FEATURE_CTRL_Msk                    (0x1U << I2C_CON_BUS_CLR_FEATURE_CTRL_Pos)
#define I2C_CON_BUS_CLR_FEATURE_CTRL                        I2C_CON_BUS_CLR_FEATURE_CTRL_Msk

#define I2C_CON_STOP_DET_IF_MASTER_ACTIVE_Pos               (10U)
#define I2C_CON_STOP_DET_IF_MASTER_ACTIVE_Len               (1U)
#define I2C_CON_STOP_DET_IF_MASTER_ACTIVE_Msk               (0x1U << I2C_CON_STOP_DET_IF_MASTER_ACTIVE_Pos)
#define I2C_CON_STOP_DET_IF_MASTER_ACTIVE                   I2C_CON_STOP_DET_IF_MASTER_ACTIVE_Msk

#define I2C_CON_RX_FIFO_FULL_HLD_CTRL_Pos                   (9U)
#define I2C_CON_RX_FIFO_FULL_HLD_CTRL_Len                   (1U)
#define I2C_CON_RX_FIFO_FULL_HLD_CTRL_Msk                   (0x1U << I2C_CON_RX_FIFO_FULL_HLD_CTRL_Pos)
#define I2C_CON_RX_FIFO_FULL_HLD_CTRL                       I2C_CON_RX_FIFO_FULL_HLD_CTRL_Msk

#define I2C_CON_TX_EMPTY_CTRL_Pos                           (8U)
#define I2C_CON_TX_EMPTY_CTRL_Len                           (1U)
#define I2C_CON_TX_EMPTY_CTRL_Msk                           (0x1U << I2C_CON_TX_EMPTY_CTRL_Pos)
#define I2C_CON_TX_EMPTY_CTRL                               I2C_CON_TX_EMPTY_CTRL_Msk

#define I2C_CON_STOP_DET_IF_ADDRESSED_Pos                   (7U)
#define I2C_CON_STOP_DET_IF_ADDRESSED_Len                   (1U)
#define I2C_CON_STOP_DET_IF_ADDRESSED_Msk                   (0x1U << I2C_CON_STOP_DET_IF_ADDRESSED_Pos)
#define I2C_CON_STOP_DET_IF_ADDRESSED                       I2C_CON_STOP_DET_IF_ADDRESSED_Msk

#define I2C_CON_SLV_DIS_Pos                                 (6U)
#define I2C_CON_SLV_DIS_Len                                 (1U)
#define I2C_CON_SLV_DIS_Msk                                 (0x1U << I2C_CON_SLV_DIS_Pos)
#define I2C_CON_SLV_DIS                                     I2C_CON_SLV_DIS_Msk

#define I2C_CON_RESTART_EN_Pos                              (5U)
#define I2C_CON_RESTART_EN_Len                              (1U)
#define I2C_CON_RESTART_EN_Msk                              (0x1U << I2C_CON_RESTART_EN_Pos)
#define I2C_CON_RESTART_EN                                  I2C_CON_RESTART_EN_Msk

#define I2C_CON_10BITADDR_MST_Pos                           (4U)
#define I2C_CON_10BITADDR_MST_Len                           (1U)
#define I2C_CON_10BITADDR_MST_Msk                           (0x1U << I2C_CON_10BITADDR_MST_Pos)
#define I2C_CON_10BITADDR_MST                               I2C_CON_10BITADDR_MST_Msk

#define I2C_CON_10BITADDR_SLV_Pos                           (3U)
#define I2C_CON_10BITADDR_SLV_Len                           (1U)
#define I2C_CON_10BITADDR_SLV_Msk                           (0x1U << I2C_CON_10BITADDR_SLV_Pos)
#define I2C_CON_10BITADDR_SLV                               I2C_CON_10BITADDR_SLV_Msk

#define I2C_CON_SPEED_Pos                                   (1U)
#define I2C_CON_SPEED_Len                                   (2U)
#define I2C_CON_SPEED_Msk                                   (0x3U << I2C_CON_SPEED_Pos)
#define I2C_CON_SPEED                                       I2C_CON_SPEED_Msk
#define I2C_CON_SPEED_STANDARD                              (0x1U << I2C_CON_SPEED_Pos)
#define I2C_CON_SPEED_FAST                                  (0x2U << I2C_CON_SPEED_Pos)
#define I2C_CON_SPEED_HIGH                                  (0x3U << I2C_CON_SPEED_Pos)

#define I2C_CON_MST_MODE_Pos                                (0U)
#define I2C_CON_MST_MODE_Len                                (1U)
#define I2C_CON_MST_MODE_Msk                                (0x1U << I2C_CON_MST_MODE_Pos)
#define I2C_CON_MST_MODE                                    I2C_CON_MST_MODE_Msk

/*******************  Bit definition for IC_TAR register  *********************/
#define I2C_TAR_SPECIAL_Pos                                 (11U)
#define I2C_TAR_SPECIAL_Len                                 (1U)
#define I2C_TAR_SPECIAL_Msk                                 (0x1U << I2C_TAR_SPECIAL_Pos)
#define I2C_TAR_SPECIAL                                     I2C_TAR_SPECIAL_Msk

#define I2C_TAR_GC_OR_START_Pos                             (10U)
#define I2C_TAR_GC_OR_START_Len                             (1U)
#define I2C_TAR_GC_OR_START_Msk                             (0x0U << I2C_TAR_GC_OR_START_Pos)
#define I2C_TAR_GC_OR_START                                 I2C_TAR_GC_OR_START_Msk

#define I2C_TAR_ADDR_Pos                                    (0U)
#define I2C_TAR_ADDR_Len                                    (10U)
#define I2C_TAR_ADDR_Msk                                    (0x3FFU << I2C_TAR_ADDR_Pos)
#define I2C_TAR_ADDR                                        I2C_TAR_ADDR_Msk
#define I2C_TAR_ADDR_7BIT                                   (0x07FU << I2C_TAR_ADDR_Pos)
#define I2C_TAR_ADDR_10BIT                                  (0x3FFU << I2C_TAR_ADDR_Pos)

/*******************  Bit definition for IC_SAR register  *********************/
#define I2C_SAR_ADDR_Pos                                    (0U)
#define I2C_SAR_ADDR_Len                                    (10U)
#define I2C_SAR_ADDR_Msk                                    (0x3FFU << I2C_SAR_ADDR_Pos)
#define I2C_SAR_ADDR_7BIT                                   (0x07FU << I2C_SAR_ADDR_Pos)
#define I2C_SAR_ADDR_10BIT                                  (0x3FFU << I2C_SAR_ADDR_Pos)

/*******************  Bit definition for IC_HS_MADDR register  ****************/
#define I2C_HS_MADDR_HS_MAR_Pos                             (0U)
#define I2C_HS_MADDR_HS_MAR_Len                             (3U)
#define I2C_HS_MADDR_HS_MAR_Msk                             (0x7U << I2C_HS_MADDR_HS_MAR_Pos)
#define I2C_HS_MADDR_HS_MAR                                 I2C_HS_MADDR_HS_MAR_Msk

/*******************  Bit definition for IC_DATA_CMD register  ****************/
#define I2C_DATA_CMD_RESTART_Pos                            (10U)
#define I2C_DATA_CMD_RESTART_Len                            (1U)
#define I2C_DATA_CMD_RESTART_Msk                            (0x1U << I2C_DATA_CMD_RESTART_Pos)
#define I2C_DATA_CMD_RESTART                                I2C_DATA_CMD_RESTART_Msk

#define I2C_DATA_CMD_STOP_Pos                               (9U)
#define I2C_DATA_CMD_STOP_Len                               (1U)
#define I2C_DATA_CMD_STOP_Msk                               (0x1U << I2C_DATA_CMD_STOP_Pos)
#define I2C_DATA_CMD_STOP                                   I2C_DATA_CMD_STOP_Msk

#define I2C_DATA_CMD_CMD_Pos                                (8U)
#define I2C_DATA_CMD_CMD_Len                                (1U)
#define I2C_DATA_CMD_CMD_Msk                                (0x1U << I2C_DATA_CMD_CMD_Pos)
#define I2C_DATA_CMD_CMD                                    I2C_DATA_CMD_CMD_Msk

#define I2C_DATA_CMD_DAT_Pos                                (0U)
#define I2C_DATA_CMD_DAT_Len                                (8U)
#define I2C_DATA_CMD_DAT_Msk                                (0xFFU << I2C_DATA_CMD_DAT_Pos)
#define I2C_DATA_CMD_DAT                                    I2C_DATA_CMD_DAT_Msk

/*******************  Bit definition for IC_SS_SCL_HCNT register  *************/
#define I2C_SS_SCL_HCNT_Pos                                 (0U)
#define I2C_SS_SCL_HCNT_Len                                 (16U)
#define I2C_SS_SCL_HCNT_Msk                                 (0xFFFFU << I2C_SS_SCL_HCNT_Pos)
#define I2C_SS_SCL_HCNT                                     I2C_SS_SCL_HCNT_Msk

/*******************  Bit definition for IC_SS_SCL_LCNT register  *************/
#define I2C_SS_SCL_LCNT_Pos                                 (0U)
#define I2C_SS_SCL_LCNT_Len                                 (16U)
#define I2C_SS_SCL_LCNT_Msk                                 (0xFFFFU << I2C_SS_SCL_LCNT_Pos)
#define I2C_SS_SCL_LCNT                                     I2C_SS_SCL_LCNT_Msk

/*******************  Bit definition for IC_FS_SCL_HCNT register  *************/
#define I2C_FS_SCL_HCNT_Pos                                 (0U)
#define I2C_FS_SCL_HCNT_Len                                 (16U)
#define I2C_FS_SCL_HCNT_Msk                                 (0xFFFFU << I2C_FS_SCL_HCNT_Pos)
#define I2C_FS_SCL_HCNT                                     I2C_FS_SCL_HCNT_Msk

/*******************  Bit definition for IC_FS_SCL_LCNT register  *************/
#define I2C_FS_SCL_LCNT_Pos                                 (0U)
#define I2C_FS_SCL_LCNT_Len                                 (16U)
#define I2C_FS_SCL_LCNT_Msk                                 (0xFFFFU << I2C_FS_SCL_LCNT_Pos)
#define I2C_FS_SCL_LCNT                                     I2C_FS_SCL_LCNT_Msk

/*******************  Bit definition for IC_HS_SCL_HCNT register  *************/
#define I2C_HS_SCL_HCNT_Pos                                 (0U)
#define I2C_HS_SCL_HCNT_Len                                 (16U)
#define I2C_HS_SCL_HCNT_Msk                                 (0xFFFFU << I2C_HS_SCL_HCNT_Pos)
#define I2C_HS_SCL_HCNT                                     I2C_HS_SCL_HCNT_Msk

/*******************  Bit definition for IC_HS_SCL_LCNT register  *************/
#define I2C_HS_SCL_LCNT_Pos                                 (0U)
#define I2C_HS_SCL_LCNT_Len                                 (16U)
#define I2C_HS_SCL_LCNT_Msk                                 (0xFFFFU << I2C_HS_SCL_LCNT_Pos)
#define I2C_HS_SCL_LCNT                                     I2C_HS_SCL_LCNT_Msk

/**  Bit definition for IC_INTR_STAT/IC_INTR_MASK/IC_RAW_INTR_STAT register  **/
#define I2C_INTR_ALL                                        (0x3FFFU)

#define I2C_INTR_MST_ON_HOLD_Pos                            (13U)
#define I2C_INTR_MST_ON_HOLD_Len                            (1U)
#define I2C_INTR_MST_ON_HOLD_Msk                            (0x1U << I2C_INTR_MST_ON_HOLD_Pos)
#define I2C_INTR_MST_ON_HOLD                                I2C_INTR_MST_ON_HOLD_Msk

#define I2C_INTR_RESTART_DET_Pos                            (12U)
#define I2C_INTR_RESTART_DET_Len                            (1U)
#define I2C_INTR_RESTART_DET_Msk                            (0x1U << I2C_INTR_RESTART_DET_Pos)
#define I2C_INTR_RESTART_DET                                I2C_INTR_RESTART_DET_Msk

#define I2C_INTR_GEN_CALL_Pos                               (11U)
#define I2C_INTR_GEN_CALL_Len                               (1U)
#define I2C_INTR_GEN_CALL_Msk                               (0x1U << I2C_INTR_GEN_CALL_Pos)
#define I2C_INTR_GEN_CALL                                   I2C_INTR_GEN_CALL_Msk

#define I2C_INTR_START_DET_Pos                              (10U)
#define I2C_INTR_START_DET_Len                              (1U)
#define I2C_INTR_START_DET_Msk                              (0x1U << I2C_INTR_START_DET_Pos)
#define I2C_INTR_START_DET                                  I2C_INTR_START_DET_Msk

#define I2C_INTR_STOP_DET_Pos                               (9U)
#define I2C_INTR_STOP_DET_Len                               (1U)
#define I2C_INTR_STOP_DET_Msk                               (0x1U << I2C_INTR_STOP_DET_Pos)
#define I2C_INTR_STOP_DET                                   I2C_INTR_STOP_DET_Msk

#define I2C_INTR_ACTIVITY_Pos                               (8U)
#define I2C_INTR_ACTIVITY_Len                               (1U)
#define I2C_INTR_ACTIVITY_Msk                               (0x1U << I2C_INTR_ACTIVITY_Pos)
#define I2C_INTR_ACTIVITY                                   I2C_INTR_ACTIVITY_Msk

#define I2C_INTR_RX_DONE_Pos                                (7U)
#define I2C_INTR_RX_DONE_Len                                (1U)
#define I2C_INTR_RX_DONE_Msk                                (0x1U << I2C_INTR_RX_DONE_Pos)
#define I2C_INTR_RX_DONE                                    I2C_INTR_RX_DONE_Msk

#define I2C_INTR_TX_ABRT_Pos                                (6U)
#define I2C_INTR_TX_ABRT_Len                                (1U)
#define I2C_INTR_TX_ABRT_Msk                                (0x1U << I2C_INTR_TX_ABRT_Pos)
#define I2C_INTR_TX_ABRT                                    I2C_INTR_TX_ABRT_Msk

#define I2C_INTR_RD_REQ_Pos                                 (5U)
#define I2C_INTR_RD_REQ_Len                                 (1U)
#define I2C_INTR_RD_REQ_Msk                                 (0x1U << I2C_INTR_RD_REQ_Pos)
#define I2C_INTR_RD_REQ                                     I2C_INTR_RD_REQ_Msk

#define I2C_INTR_TX_EMPTY_Pos                               (4U)
#define I2C_INTR_TX_EMPTY_Len                               (1U)
#define I2C_INTR_TX_EMPTY_Msk                               (0x1U << I2C_INTR_TX_EMPTY_Pos)
#define I2C_INTR_TX_EMPTY                                   I2C_INTR_TX_EMPTY_Msk

#define I2C_INTR_TX_OVER_Pos                                (3U)
#define I2C_INTR_TX_OVER_Len                                (1U)
#define I2C_INTR_TX_OVER_Msk                                (0x1U << I2C_INTR_TX_OVER_Pos)
#define I2C_INTR_TX_OVER                                    I2C_INTR_TX_OVER_Msk

#define I2C_INTR_RX_FULL_Pos                                (2U)
#define I2C_INTR_RX_FULL_Len                                (1U)
#define I2C_INTR_RX_FULL_Msk                                (0x1U << I2C_INTR_RX_FULL_Pos)
#define I2C_INTR_RX_FULL                                    I2C_INTR_RX_FULL_Msk

#define I2C_INTR_RX_OVER_Pos                                (1U)
#define I2C_INTR_RX_OVER_Len                                (1U)
#define I2C_INTR_RX_OVER_Msk                                (0x1U << I2C_INTR_RX_OVER_Pos)
#define I2C_INTR_RX_OVER                                    I2C_INTR_RX_OVER_Msk

#define I2C_INTR_RX_UNDER_Pos                               (0U)
#define I2C_INTR_RX_UNDER_Len                               (1U)
#define I2C_INTR_RX_UNDER_Msk                               (0x1U << I2C_INTR_RX_UNDER_Pos)
#define I2C_INTR_RX_UNDER                                   I2C_INTR_RX_UNDER_Msk

/*******************  Bit definition for IC_RX_TL register  *******************/
#define I2C_RX_TL_RXTL_Pos                                  (0U)
#define I2C_RX_TL_RXTL_Len                                  (8U)
#define I2C_RX_TL_RXTL_Msk                                  (0xFFU << I2C_RX_TL_RXTL_Pos)
#define I2C_RX_TL_RXTL                                      I2C_RX_TL_RXTL_Msk

/*******************  Bit definition for IC_TX_TL register  *******************/
#define I2C_TX_TL_TXTL_Pos                                  (0U)
#define I2C_TX_TL_TXTL_Len                                  (8U)
#define I2C_TX_TL_TXTL_Msk                                  (0xFFU << I2C_TX_TL_TXTL_Pos)
#define I2C_TX_TL_TXTL                                      I2C_TX_TL_TXTL_Msk

/*******************  Bit definition for IC_ENABLE register  ******************/
// #define I2C_ENABLE_TX_CMD_BLOCK_Pos             (2U)
// #define I2C_ENABLE_TX_CMD_BLOCK_Len             (1U)
// #define I2C_ENABLE_TX_CMD_BLOCK_Msk             (0x1U << I2C_ENABLE_TX_CMD_BLOCK_Pos)
// #define I2C_ENABLE_TX_CMD_BLOCK                 I2C_ENABLE_TX_CMD_BLOCK_Msk
#define I2C_ENABLE_ABORT_Pos                                (1U)
#define I2C_ENABLE_ABORT_Len                                (1U)
#define I2C_ENABLE_ABORT_Msk                                (0x1U << I2C_ENABLE_ABORT_Pos)
#define I2C_ENABLE_ABORT                                    I2C_ENABLE_ABORT_Msk

#define I2C_ENABLE_ENABLE_Pos                               (0U)
#define I2C_ENABLE_ENABLE_Len                               (1U)
#define I2C_ENABLE_ENABLE_Msk                               (0x1U << I2C_ENABLE_ENABLE_Pos)
#define I2C_ENABLE_ENABLE                                   I2C_ENABLE_ENABLE_Msk

/*******************  Bit definition for IC_STATUS register  ******************/
#define I2C_STATUS_SLV_ACTIVITY_Pos                         (6U)
#define I2C_STATUS_SLV_ACTIVITY_Len                         (1U)
#define I2C_STATUS_SLV_ACTIVITY_Msk                         (0x1U << I2C_STATUS_SLV_ACTIVITY_Pos)
#define I2C_STATUS_SLV_ACTIVITY                             I2C_STATUS_SLV_ACTIVITY_Msk

#define I2C_STATUS_MST_ACTIVITY_Pos                         (5U)
#define I2C_STATUS_MST_ACTIVITY_Len                         (1U)
#define I2C_STATUS_MST_ACTIVITY_Msk                         (0x1U << I2C_STATUS_MST_ACTIVITY_Pos)
#define I2C_STATUS_MST_ACTIVITY                             I2C_STATUS_MST_ACTIVITY_Msk

#define I2C_STATUS_RFF_Pos                                  (4U)
#define I2C_STATUS_RFF_Len                                  (1U)
#define I2C_STATUS_RFF_Msk                                  (0x1U << I2C_STATUS_RFF_Pos)
#define I2C_STATUS_RFF                                      I2C_STATUS_RFF_Msk

#define I2C_STATUS_RFNE_Pos                                 (3U)
#define I2C_STATUS_RFNE_Len                                 (1U)
#define I2C_STATUS_RFNE_Msk                                 (0x1U << I2C_STATUS_RFNE_Pos)
#define I2C_STATUS_RFNE                                     I2C_STATUS_RFNE_Msk

#define I2C_STATUS_TFE_Pos                                  (2U)
#define I2C_STATUS_TFE_Len                                  (1U)
#define I2C_STATUS_TFE_Msk                                  (0x1U << I2C_STATUS_TFE_Pos)
#define I2C_STATUS_TFE                                      I2C_STATUS_TFE_Msk

#define I2C_STATUS_TFNF_Pos                                 (1U)
#define I2C_STATUS_TFNF_Len                                 (1U)
#define I2C_STATUS_TFNF_Msk                                 (0x1U << I2C_STATUS_TFNF_Pos)
#define I2C_STATUS_TFNF                                     I2C_STATUS_TFNF_Msk

#define I2C_STATUS_ACTIVITY_Pos                             (0U)
#define I2C_STATUS_ACTIVITY_Len                             (1U)
#define I2C_STATUS_ACTIVITY_Msk                             (0x1U << I2C_STATUS_ACTIVITY_Pos)
#define I2C_STATUS_ACTIVITY                                 I2C_STATUS_ACTIVITY_Msk

/*******************  Bit definition for IC_RXFLR register  *******************/
#define I2C_RXFLR_RXFLR_Pos                                 (0U)
#define I2C_RXFLR_RXFLR_Len                                 (8U)
#define I2C_RXFLR_RXFLR_Msk                                 (0xFFU << I2C_RXFLR_RXFLR_Pos)
#define I2C_RXFLR_RXFLR                                     I2C_RXFLR_RXFLR_Msk

/*******************  Bit definition for IC_TXFLR register  *******************/
#define I2C_TXFLR_TXFLR_Pos                                 (0U)
#define I2C_TXFLR_TXFLR_Len                                 (8U)
#define I2C_TXFLR_TXFLR_Msk                                 (0xFFU << I2C_TXFLR_TXFLR_Pos)
#define I2C_TXFLR_TXFLR                                     I2C_TXFLR_TXFLR_Msk

/*******************  Bit definition for IC_SDA_HOLD register  *******************/
#define I2C_SDA_HOLD_RX_HOLD_Pos                            (16U)
#define I2C_SDA_HOLD_RX_HOLD_Len                            (16U)
#define I2C_SDA_HOLD_RX_HOLD_Msk                            (0xFFFFU << I2C_SDA_HOLD_RX_HOLD_Pos)
#define I2C_SDA_HOLD_RX_HOLD                                I2C_SDA_HOLD_RX_HOLD_Msk

#define I2C_SDA_HOLD_TX_HOLD_Pos                            (0U)
#define I2C_SDA_HOLD_TX_HOLD_Len                            (16U)
#define I2C_SDA_HOLD_TX_HOLD_Msk                            (0xFFFFU << I2C_SDA_HOLD_TX_HOLD_Pos)
#define I2C_SDA_HOLD_TX_HOLD                                I2C_SDA_HOLD_TX_HOLD_Msk

/*******************  Bit definition for IC_TX_ABRT_SOURCE register  **********/
#define I2C_TX_ABRT_SRC_TX_FLUSH_CNT_Pos                    (23U)
#define I2C_TX_ABRT_SRC_TX_FLUSH_CNT_Len                    (9U)
#define I2C_TX_ABRT_SRC_TX_FLUSH_CNT_Msk                    (0x3FFU << I2C_TX_ABRT_SRC_TX_FLUSH_CNT_Pos)
#define I2C_TX_ABRT_SRC_TX_FLUSH_CNT                        I2C_TX_ABRT_SRC_TX_FLUSH_CNT_Msk

#define I2C_TX_ABRT_SRC_USER_ABRT_Pos                       (16U)
#define I2C_TX_ABRT_SRC_USER_ABRT_Len                       (1U)
#define I2C_TX_ABRT_SRC_USER_ABRT_Msk                       (0x1U << I2C_TX_ABRT_SRC_USER_ABRT_Pos)
#define I2C_TX_ABRT_SRC_USER_ABRT                           I2C_TX_ABRT_SRC_USER_ABRT_Msk

#define I2C_TX_ABRT_SRC_SLVRD_INTX_Pos                      (15U)
#define I2C_TX_ABRT_SRC_SLVRD_INTX_Len                      (1U)
#define I2C_TX_ABRT_SRC_SLVRD_INTX_Msk                      (0x1U << I2C_TX_ABRT_SRC_SLVRD_INTX_Pos)
#define I2C_TX_ABRT_SRC_SLVRD_INTX                          I2C_TX_ABRT_SRC_SLVRD_INTX_Msk

#define I2C_TX_ABRT_SRC_SLV_ARBLOST_Pos                     (14U)
#define I2C_TX_ABRT_SRC_SLV_ARBLOST_Len                     (1U)
#define I2C_TX_ABRT_SRC_SLV_ARBLOST_Msk                     (0x1U << I2C_TX_ABRT_SRC_SLV_ARBLOST_Pos)
#define I2C_TX_ABRT_SRC_SLV_ARBLOST                         I2C_TX_ABRT_SRC_SLV_ARBLOST_Msk

#define I2C_TX_ABRT_SRC_SLVFLUSH_TXFIFO_Pos                 (13U)
#define I2C_TX_ABRT_SRC_SLVFLUSH_TXFIFO_Len                 (1U)
#define I2C_TX_ABRT_SRC_SLVFLUSH_TXFIFO_Msk                 (0x1U << I2C_TX_ABRT_SRC_SLVFLUSH_TXFIFO_Pos)
#define I2C_TX_ABRT_SRC_SLVFLUSH_TXFIFO                     I2C_TX_ABRT_SRC_SLVFLUSH_TXFIFO_Msk

#define I2C_TX_ABRT_SRC_ARB_LOST_Pos                        (12U)
#define I2C_TX_ABRT_SRC_ARB_LOST_Len                        (1U)
#define I2C_TX_ABRT_SRC_ARB_LOST_Msk                        (0x1U << I2C_TX_ABRT_SRC_ARB_LOST_Pos)
#define I2C_TX_ABRT_SRC_ARB_LOST                            I2C_TX_ABRT_SRC_ARB_LOST_Msk

#define I2C_TX_ABRT_SRC_MST_DIS_Pos                         (11U)
#define I2C_TX_ABRT_SRC_MST_DIS_Len                         (1U)
#define I2C_TX_ABRT_SRC_MST_DIS_Msk                         (0x1U << I2C_TX_ABRT_SRC_MST_DIS_Pos)
#define I2C_TX_ABRT_SRC_MST_DIS                             I2C_TX_ABRT_SRC_MST_DIS_Msk

#define I2C_TX_ABRT_SRC_10B_RD_NORSTRT_Pos                  (10U)
#define I2C_TX_ABRT_SRC_10B_RD_NORSTRT_Len                  (1U)
#define I2C_TX_ABRT_SRC_10B_RD_NORSTRT_Msk                  (0x1U << I2C_TX_ABRT_SRC_10B_RD_NORSTRT_Pos)
#define I2C_TX_ABRT_SRC_10B_RD_NORSTRT                      I2C_TX_ABRT_SRC_10B_RD_NORSTRT_Msk

#define I2C_TX_ABRT_SRC_SBYTE_NORSTRT_Pos                   (9U)
#define I2C_TX_ABRT_SRC_SBYTE_NORSTRT_Len                   (1U)
#define I2C_TX_ABRT_SRC_SBYTE_NORSTRT_Msk                   (0x1U << I2C_TX_ABRT_SRC_SBYTE_NORSTRT_Pos)
#define I2C_TX_ABRT_SRC_SBYTE_NORSTRT                       I2C_TX_ABRT_SRC_SBYTE_NORSTRT_Msk

#define I2C_TX_ABRT_SRC_HS_NORSTRT_Pos                      (8U)
#define I2C_TX_ABRT_SRC_HS_NORSTRT_Len                      (1U)
#define I2C_TX_ABRT_SRC_HS_NORSTRT_Msk                      (0x1U << I2C_TX_ABRT_SRC_HS_NORSTRT_Pos)
#define I2C_TX_ABRT_SRC_HS_NORSTRT                          I2C_TX_ABRT_SRC_HS_NORSTRT_Msk

#define I2C_TX_ABRT_SRC_SBYTE_ACKDET_Pos                    (7U)
#define I2C_TX_ABRT_SRC_SBYTE_ACKDET_Len                    (1U)
#define I2C_TX_ABRT_SRC_SBYTE_ACKDET_Msk                    (0x1U << I2C_TX_ABRT_SRC_SBYTE_ACKDET_Pos)
#define I2C_TX_ABRT_SRC_SBYTE_ACKDET                        I2C_TX_ABRT_SRC_SBYTE_ACKDET_Msk

#define I2C_TX_ABRT_SRC_HS_ACKDET_Pos                       (6U)
#define I2C_TX_ABRT_SRC_HS_ACKDET_Len                       (1U)
#define I2C_TX_ABRT_SRC_HS_ACKDET_Msk                       (0x1U << I2C_TX_ABRT_SRC_HS_ACKDET_Pos)
#define I2C_TX_ABRT_SRC_HS_ACKDET                           I2C_TX_ABRT_SRC_HS_ACKDET_Msk

#define I2C_TX_ABRT_SRC_GCALL_READ_Pos                      (5U)
#define I2C_TX_ABRT_SRC_GCALL_READ_Len                      (1U)
#define I2C_TX_ABRT_SRC_GCALL_READ_Msk                      (0x1U << I2C_TX_ABRT_SRC_GCALL_READ_Pos)
#define I2C_TX_ABRT_SRC_GCALL_READ                          I2C_TX_ABRT_SRC_GCALL_READ_Msk

#define I2C_TX_ABRT_SRC_GCALL_NOACK_Pos                     (4U)
#define I2C_TX_ABRT_SRC_GCALL_NOACK_Len                     (1U)
#define I2C_TX_ABRT_SRC_GCALL_NOACK_Msk                     (0x1U << I2C_TX_ABRT_SRC_GCALL_NOACK_Pos)
#define I2C_TX_ABRT_SRC_GCALL_NOACK                         I2C_TX_ABRT_SRC_GCALL_NOACK_Msk

#define I2C_TX_ABRT_SRC_TXDATA_NOACK_Pos                    (3U)
#define I2C_TX_ABRT_SRC_TXDATA_NOACK_Len                    (1U)
#define I2C_TX_ABRT_SRC_TXDATA_NOACK_Msk                    (0x1U << I2C_TX_ABRT_SRC_TXDATA_NOACK_Pos)
#define I2C_TX_ABRT_SRC_TXDATA_NOACK                        I2C_TX_ABRT_SRC_TXDATA_NOACK_Msk

#define I2C_TX_ABRT_SRC_10ADDR2_NOACK_Pos                   (2U)
#define I2C_TX_ABRT_SRC_10ADDR2_NOACK_Len                   (1U)
#define I2C_TX_ABRT_SRC_10ADDR2_NOACK_Msk                   (0x1U << I2C_TX_ABRT_SRC_10ADDR2_NOACK_Pos)
#define I2C_TX_ABRT_SRC_10ADDR2_NOACK                       I2C_TX_ABRT_SRC_10ADDR2_NOACK_Msk

#define I2C_TX_ABRT_SRC_10ADDR1_NOACK_Pos                   (1U)
#define I2C_TX_ABRT_SRC_10ADDR1_NOACK_Len                   (1U)
#define I2C_TX_ABRT_SRC_10ADDR1_NOACK_Msk                   (0x1U << I2C_TX_ABRT_SRC_10ADDR1_NOACK_Pos)
#define I2C_TX_ABRT_SRC_10ADDR1_NOACK                       I2C_TX_ABRT_SRC_10ADDR1_NOACK_Msk

#define I2C_TX_ABRT_SRC_7B_ADDR_NOACK_Pos                   (0U)
#define I2C_TX_ABRT_SRC_7B_ADDR_NOACK_Len                   (1U)
#define I2C_TX_ABRT_SRC_7B_ADDR_NOACK_Msk                   (0x1U << I2C_TX_ABRT_SRC_7B_ADDR_NOACK_Pos)
#define I2C_TX_ABRT_SRC_7B_ADDR_NOACK                       I2C_TX_ABRT_SRC_7B_ADDR_NOACK_Msk

/*******************  Bit definition for IC_DMA_CR register  *******************/
#define I2C_DMA_CR_TDMAE_Pos                                (1U)
#define I2C_DMA_CR_TDMAE_Len                                (1U)
#define I2C_DMA_CR_TDMAE_Msk                                (0x1U << I2C_DMA_CR_TDMAE_Pos)
#define I2C_DMA_CR_TDMAE                                    I2C_DMA_CR_TDMAE_Msk

#define I2C_DMA_CR_RDMAE_Pos                                (0U)
#define I2C_DMA_CR_RDMAE_Len                                (1U)
#define I2C_DMA_CR_RDMAE_Msk                                (0x1U << I2C_DMA_CR_RDMAE_Pos)
#define I2C_DMA_CR_RDMAE                                    I2C_DMA_CR_RDMAE_Msk

/*******************  Bit definition for IC_DMA_TDLR register  ****************/
#define I2C_DMA_TDLR_DMATDL_Pos                             (0U)
#define I2C_DMA_TDLR_DMATDL_Len                             (8U)
#define I2C_DMA_TDLR_DMATDL_Msk                             (0xFFU << I2C_DMA_TDLR_DMATDL_Pos)
#define I2C_DMA_TDLR_DMATDL                                 I2C_DMA_TDLR_DMATDL_Msk

/*******************  Bit definition for IC_DMA_RDLR register  ****************/
#define I2C_DMA_RDLR_DMARDL_Pos                             (0U)
#define I2C_DMA_RDLR_DMARDL_Len                             (8U)
#define I2C_DMA_RDLR_DMARDL_Msk                             (0xFFU << I2C_DMA_RDLR_DMARDL_Pos)
#define I2C_DMA_RDLR_DMARDL                                 I2C_DMA_RDLR_DMARDL_Msk

/*******************  Bit definition for IC_SDA_SETUP register  ***************/
#define I2C_SDA_SETUP_SDA_SETUP_Pos                         (0U)
#define I2C_SDA_SETUP_SDA_SETUP_Len                         (8U)
#define I2C_SDA_SETUP_SDA_SETUP_Msk                         (0xFFU << I2C_SDA_SETUP_SDA_SETUP_Pos)
#define I2C_SDA_SETUP_SDA_SETUP                             I2C_SDA_SETUP_SDA_SETUP_Msk

/*******************  Bit definition for IC_ACK_GENERAL_CALL register  ********/
#define I2C_ACK_GENERAL_CALL_ACK_GC_Pos                     (0U)
#define I2C_ACK_GENERAL_CALL_ACK_GC_Len                     (1U)
#define I2C_ACK_GENERAL_CALL_ACK_GC_Msk                     (0x1U << I2C_ACK_GENERAL_CALL_ACK_GC_Pos)
#define I2C_ACK_GENERAL_CALL_ACK_GC                         I2C_ACK_GENERAL_CALL_ACK_GC_Msk

/*******************  Bit definition for IC_ENABLE_STATUS register  ***********/
#define I2C_ENABLE_STATUS_SLV_RX_LOST_Pos                   (2U)
#define I2C_ENABLE_STATUS_SLV_RX_LOST_Len                   (1U)
#define I2C_ENABLE_STATUS_SLV_RX_LOST_Msk                   (0x1U << I2C_ENABLE_STATUS_SLV_RX_LOST_Pos)
#define I2C_ENABLE_STATUS_SLV_RX_LOST                       I2C_ENABLE_STATUS_SLV_RX_LOST_Msk

#define I2C_ENABLE_STATUS_SLV_DIS_WHL_BUSY_Pos              (1U)
#define I2C_ENABLE_STATUS_SLV_DIS_WHL_BUSY_Len              (1U)
#define I2C_ENABLE_STATUS_SLV_DIS_WHL_BUSY_Msk              (0x1U << I2C_ENABLE_STATUS_SLV_DIS_WHL_BUSY_Pos)
#define I2C_ENABLE_STATUS_SLV_DIS_WHL_BUSY                  I2C_ENABLE_STATUS_SLV_DIS_WHL_BUSY_Msk

#define I2C_ENABLE_STATUS_IC_EN_Pos                         (0U)
#define I2C_ENABLE_STATUS_IC_EN_Len                         (1U)
#define I2C_ENABLE_STATUS_IC_EN_Msk                         (0x1U << I2C_ENABLE_STATUS_IC_EN_Pos)
#define I2C_ENABLE_STATUS_IC_EN                             I2C_ENABLE_STATUS_IC_EN_Msk

/*******************  Bit definition for IC_FS_SPKLEN register  ***************/
#define I2C_FS_SPKLEN_FS_SPKLEN_Pos                         (0U)
#define I2C_FS_SPKLEN_FS_SPKLEN_Len                         (8U)
#define I2C_FS_SPKLEN_FS_SPKLEN_Msk                         (0xFFU << I2C_FS_SPKLEN_FS_SPKLEN_Pos)
#define I2C_FS_SPKLEN_FS_SPKLEN                             I2C_FS_SPKLEN_FS_SPKLEN_Msk

/*******************  Bit definition for IC_HS_SPKLEN register  ***************/
#define I2C_HS_SPKLEN_HS_SPKLEN_Pos                         (0U)
#define I2C_HS_SPKLEN_HS_SPKLEN_Len                         (8U)
#define I2C_HS_SPKLEN_HS_SPKLEN_Msk                         (0xFFU << I2C_HS_SPKLEN_HS_SPKLEN_Pos)
#define I2C_HS_SPKLEN_HS_SPKLEN                             I2C_HS_SPKLEN_HS_SPKLEN_Msk


/* ================================================================================================================= */
/* ================                                        I2S                                      ================ */
/* ================================================================================================================= */
#define I2S_TXFIFO_SIZE                                     (16U)
#define I2S_RXFIFO_SIZE                                     (16U)

/*******************  Bit definition for ENABLE register  *********************/
#define I2S_ENABLE_EN_Pos                                   (0U)
#define I2S_ENABLE_EN_Len                                   (1U)
#define I2S_ENABLE_EN_Msk                                   (0x1U << I2S_ENABLE_EN_Pos)
#define I2S_ENABLE_EN                                       I2S_ENABLE_EN_Msk

/*******************  Bit definition for RBEN register  ***********************/
#define I2S_RBEN_EN_Pos                                     (0U)
#define I2S_RBEN_EN_Len                                     (1U)
#define I2S_RBEN_EN_Msk                                     (0x1U << I2S_RBEN_EN_Pos)
#define I2S_RBEN_EN                                         I2S_RBEN_EN_Msk

/*******************  Bit definition for TBEN register  ***********************/
#define I2S_TBEN_EN_Pos                                     (0U)
#define I2S_TBEN_EN_Len                                     (1U)
#define I2S_TBEN_EN_Msk                                     (0x1U << I2S_TBEN_EN_Pos)
#define I2S_TBEN_EN                                         I2S_TBEN_EN_Msk

/*******************  Bit definition for CLKEN register  **********************/
#define I2S_CLKEN_EN_Pos                                    (0U)
#define I2S_CLKEN_EN_Len                                    (1U)
#define I2S_CLKEN_EN_Msk                                    (0x1U << I2S_CLKEN_EN_Pos)
#define I2S_CLKEN_EN                                        I2S_CLKEN_EN_Msk

/*****************  Bit definition for CLKCONFIG register  ********************/
#define I2S_CLKCONFIG_WSS_Pos                               (3U)
#define I2S_CLKCONFIG_WSS_Len                               (2U)
#define I2S_CLKCONFIG_WSS_Msk                               (0x3U << I2S_CLKCONFIG_WSS_Pos)
#define I2S_CLKCONFIG_WSS                                   I2S_CLKCONFIG_WSS_Msk

#define I2S_CLKCONFIG_SCLKG_Pos                             (0U)
#define I2S_CLKCONFIG_SCLKG_Len                             (3U)
#define I2S_CLKCONFIG_SCLKG_Msk                             (0x7U << I2S_CLKCONFIG_SCLKG_Pos)
#define I2S_CLKCONFIG_SCLKG                                 I2S_CLKCONFIG_SCLKG_Msk

/*****************  Bit definition for RXFIFO_RST register  *******************/
#define I2S_RXFIFO_RST_Pos                                  (0U)
#define I2S_RXFIFO_RST_Len                                  (1U)
#define I2S_RXFIFO_RST_Msk                                  (0x1U << I2S_RXFIFO_RST_Pos)
#define I2S_RXFIFO_RST                                      I2S_RXFIFO_RST_Msk

/*****************  Bit definition for TXFIFO_RST register  *******************/
#define I2S_TXFIFO_RST_Pos                                  (0U)
#define I2S_TXFIFO_RST_Len                                  (1U)
#define I2S_TXFIFO_RST_Msk                                  (0x1U << I2S_TXFIFO_RST_Pos)
#define I2S_TXFIFO_RST                                      I2S_TXFIFO_RST_Msk

/*******************  Bit definition for DATA_L register  *********************/
#define I2S_DATA_L_Pos                                      (0U)
#define I2S_DATA_L_Len                                      (32U)
#define I2S_DATA_L_Msk                                      (0xFFFFFFFFU)
#define I2S_DATA_L                                          I2S_DATA_L_Msk

/*******************  Bit definition for DATA_R register  *********************/
#define I2S_DATA_R_Pos                                      (0U)
#define I2S_DATA_R_Len                                      (32U)
#define I2S_DATA_R_Msk                                      (0xFFFFFFFFU)
#define I2S_DATA_R                                          I2S_DATA_R_Msk

/********************  Bit definition for RXEN register  **********************/
#define I2S_RXEN_EN_Pos                                     (0U)
#define I2S_RXEN_EN_Len                                     (1U)
#define I2S_RXEN_EN_Msk                                     (0x1U << I2S_RXEN_EN_Pos)
#define I2S_RXEN_EN                                         I2S_RXEN_EN_Msk

/********************  Bit definition for TXEN register  **********************/
#define I2S_TXEN_EN_Pos                                     (0U)
#define I2S_TXEN_EN_Len                                     (1U)
#define I2S_TXEN_EN_Msk                                     (0x1U << I2S_TXEN_EN_Pos)
#define I2S_TXEN_EN                                         I2S_TXEN_EN_Msk

/*******************  Bit definition for RXSIZE register  *********************/
#define I2S_RXSIZE_WLEN_Pos                                 (0U)
#define I2S_RXSIZE_WLEN_Len                                 (3U)
#define I2S_RXSIZE_WLEN_Msk                                 (0x7U << I2S_RXSIZE_WLEN_Pos)
#define I2S_RXSIZE_WLEN                                     I2S_RXSIZE_WLEN_Msk

/*******************  Bit definition for TXSIZE register  *********************/
#define I2S_TXSIZE_WLEN_Pos                                 (0U)
#define I2S_TXSIZE_WLEN_Len                                 (3U)
#define I2S_TXSIZE_WLEN_Msk                                 (0x7U << I2S_TXSIZE_WLEN_Pos)
#define I2S_TXSIZE_WLEN                                     I2S_TXSIZE_WLEN_Msk

/*******************  Bit definition for INTSTAT register  ********************/
#define I2S_INTSTAT_TXFO_Pos                                (5U)
#define I2S_INTSTAT_TXFO_Len                                (1U)
#define I2S_INTSTAT_TXFO_Msk                                (0x1U << I2S_INTSTAT_TXFO_Pos)
#define I2S_INTSTAT_TXFO                                    I2S_INTSTAT_TXFO_Msk

#define I2S_INTSTAT_TXFE_Pos                                (4U)
#define I2S_INTSTAT_TXFE_Len                                (1U)
#define I2S_INTSTAT_TXFE_Msk                                (0x1U << I2S_INTSTAT_TXFE_Pos)
#define I2S_INTSTAT_TXFE                                    I2S_INTSTAT_TXFE_Msk

#define I2S_INTSTAT_RXFO_Pos                                (1U)
#define I2S_INTSTAT_RXFO_Len                                (1U)
#define I2S_INTSTAT_RXFO_Msk                                (0x1U << I2S_INTSTAT_RXFO_Pos)
#define I2S_INTSTAT_RXFO                                    I2S_INTSTAT_RXFO_Msk

#define I2S_INTSTAT_RXDA_Pos                                (0U)
#define I2S_INTSTAT_RXDA_Len                                (1U)
#define I2S_INTSTAT_RXDA_Msk                                (0x1U << I2S_INTSTAT_RXDA_Pos)
#define I2S_INTSTAT_RXDA                                    I2S_INTSTAT_RXDA_Msk

/*******************  Bit definition for INTMASK register  ********************/
#define I2S_INTMASK_TXFO_Pos                                (5U)
#define I2S_INTMASK_TXFO_Len                                (1U)
#define I2S_INTMASK_TXFO_Msk                                (0x1U << I2S_INTSTAT_TXFO_Pos)
#define I2S_INTMASK_TXFO                                    I2S_INTSTAT_TXFO_Msk

#define I2S_INTMASK_TXFE_Pos                                (4U)
#define I2S_INTMASK_TXFE_Len                                (1U)
#define I2S_INTMASK_TXFE_Msk                                (0x1U << I2S_INTSTAT_TXFE_Pos)
#define I2S_INTMASK_TXFE                                    I2S_INTSTAT_TXFE_Msk

#define I2S_INTMASK_RXFO_Pos                                (1U)
#define I2S_INTMASK_RXFO_Len                                (1U)
#define I2S_INTMASK_RXFO_Msk                                (0x1U << I2S_INTSTAT_RXFO_Pos)
#define I2S_INTMASK_RXFO                                    I2S_INTSTAT_RXFO_Msk

#define I2S_INTMASK_RXDA_Pos                                (0U)
#define I2S_INTMASK_RXDA_Len                                (1U)
#define I2S_INTMASK_RXDA_Msk                                (0x1U << I2S_INTSTAT_RXDA_Pos)
#define I2S_INTMASK_RXDA                                    I2S_INTSTAT_RXDA_Msk

/********************  Bit definition for RXOVR register  *********************/
#define I2S_RXOVR_RXCHO_Pos                                 (0U)
#define I2S_RXOVR_RXCHO_Len                                 (1U)
#define I2S_RXOVR_RXCHO_Msk                                 (0x1U << I2S_RXOVR_RXCHO_Pos)
#define I2S_RXOVR_RXCHO                                     I2S_RXOVR_RXCHO_Msk

/********************  Bit definition for TXOVR register  *********************/
#define I2S_TXOVR_TXCHO_Pos                                 (0U)
#define I2S_TXOVR_TXCHO_Len                                 (1U)
#define I2S_TXOVR_TXCHO_Msk                                 (0x1U << I2S_TXOVR_TXCHO_Pos)
#define I2S_TXOVR_TXCHO                                     I2S_TXOVR_TXCHO_Msk

/******************  Bit definition for RXFIFO_TL register  *******************/
#define I2S_RXFIFO_TL_Pos                                   (0U)
#define I2S_RXFIFO_TL_Len                                   (4U)
#define I2S_RXFIFO_TL_Msk                                   (0xFU << I2S_RXFIFO_TL_Pos)
#define I2S_RXFIFO_TL                                       I2S_RXFIFO_TL_Msk

/******************  Bit definition for TXFIFO_TL register  *******************/
#define I2S_TXFIFO_TL_Pos                                   (0U)
#define I2S_TXFIFO_TL_Len                                   (4U)
#define I2S_TXFIFO_TL_Msk                                   (0xFU << I2S_TXFIFO_TL_Pos)
#define I2S_TXFIFO_TL                                       I2S_TXFIFO_TL_Msk

/****************  Bit definition for RXFIFO_FLUSH register  ******************/
#define I2S_RXFIFO_FLUSH_Pos                                (0U)
#define I2S_RXFIFO_FLUSH_Len                                (1U)
#define I2S_RXFIFO_FLUSH_Msk                                (0x1U << I2S_RXFIFO_FLUSH_Pos)
#define I2S_RXFIFO_FLUSH                                    I2S_RXFIFO_FLUSH_Msk

/****************  Bit definition for TXFIFO_FLUSH register  ******************/
#define I2S_TXFIFO_FLUSH_Pos                                (0U)
#define I2S_TXFIFO_FLUSH_Len                                (1U)
#define I2S_TXFIFO_FLUSH_Msk                                (0x1U << I2S_TXFIFO_FLUSH_Pos)
#define I2S_TXFIFO_FLUSH                                    I2S_TXFIFO_FLUSH_Msk

/********************  Bit definition for RXDMA register  *********************/
#define I2S_RXDMA_Pos                                       (0U)
#define I2S_RXDMA_Len                                       (32U)
#define I2S_RXDMA_Msk                                       (0xFFFFFFFFU)
#define I2S_RXDMA                                           I2S_RXDMA_Msk

/******************  Bit definition for RXDMA_RST register  *******************/
#define I2S_RXDMA_RST_Pos                                   (0U)
#define I2S_RXDMA_RST_Len                                   (1U)
#define I2S_RXDMA_RST_Msk                                   (0x1U << I2S_RXDMA_RST_Pos)
#define I2S_RXDMA_RST                                       I2S_RXDMA_RST_Msk

/********************  Bit definition for TXDMA register  *********************/
#define I2S_TXDMA_Pos                                       (0U)
#define I2S_TXDMA_Len                                       (32U)
#define I2S_TXDMA_Msk                                       (0xFFFFFFFFU)
#define I2S_TXDMA                                           I2S_TXDMA_Msk

/******************  Bit definition for TXDMA_RST register  *******************/
#define I2S_TXDMA_RST_Pos                                   (0U)
#define I2S_TXDMA_RST_Len                                   (1U)
#define I2S_TXDMA_RST_Msk                                   (0x1U << I2S_TXDMA_RST_Pos)
#define I2S_TXDMA_RST                                       I2S_TXDMA_RST_Msk

/******************  Bit definition for I2S_PARAM2 register  ******************/
#define I2S_PARAM2_RXSIZE_3_Pos                             (10U)
#define I2S_PARAM2_RXSIZE_3_Len                             (3U)
#define I2S_PARAM2_RXSIZE_3_Msk                             (0x7U << I2S_PARAM2_RXSIZE_3_Pos)
#define I2S_PARAM2_RXSIZE_3                                 I2S_PARAM2_RXSIZE_3_Msk

#define I2S_PARAM2_RXSIZE_2_Pos                             (7U)
#define I2S_PARAM2_RXSIZE_2_Len                             (3U)
#define I2S_PARAM2_RXSIZE_2_Msk                             (0x7U << I2S_PARAM2_RXSIZE_2_Pos)
#define I2S_PARAM2_RXSIZE_2                                 I2S_PARAM2_RXSIZE_2_Msk

#define I2S_PARAM2_RXSIZE_1_Pos                             (3U)
#define I2S_PARAM2_RXSIZE_1_Len                             (3U)
#define I2S_PARAM2_RXSIZE_1_Msk                             (0x7U << I2S_PARAM2_RXSIZE_1_Pos)
#define I2S_PARAM2_RXSIZE_1                                 I2S_PARAM2_RXSIZE_1_Msk

#define I2S_PARAM2_RXSIZE_0_Pos                             (0U)
#define I2S_PARAM2_RXSIZE_0_Len                             (3U)
#define I2S_PARAM2_RXSIZE_0_Msk                             (0x7U << I2S_PARAM2_RXSIZE_0_Pos)
#define I2S_PARAM2_RXSIZE_0                                 I2S_PARAM2_RXSIZE_0_Msk

/******************  Bit definition for I2S_PARAM1 register  ******************/
#define I2S_PARAM1_TXSIZE_3_Pos                             (25U)
#define I2S_PARAM1_TXSIZE_3_Len                             (3U)
#define I2S_PARAM1_TXSIZE_3_Msk                             (0x7U << I2S_PARAM1_TXSIZE_3_Pos)
#define I2S_PARAM1_TXSIZE_3                                 I2S_PARAM1_TXSIZE_3_Msk

#define I2S_PARAM1_TXSIZE_2_Pos                             (22U)
#define I2S_PARAM1_TXSIZE_2_Len                             (3U)
#define I2S_PARAM1_TXSIZE_2_Msk                             (0x7U << I2S_PARAM1_TXSIZE_2_Pos)
#define I2S_PARAM1_TXSIZE_2                                 I2S_PARAM1_TXSIZE_2_Msk

#define I2S_PARAM1_TXSIZE_1_Pos                             (19U)
#define I2S_PARAM1_TXSIZE_1_Len                             (3U)
#define I2S_PARAM1_TXSIZE_1_Msk                             (0x7U << I2S_PARAM1_TXSIZE_1_Pos)
#define I2S_PARAM1_TXSIZE_1                                 I2S_PARAM1_TXSIZE_1_Msk

#define I2S_PARAM1_TXSIZE_0_Pos                             (16U)
#define I2S_PARAM1_TXSIZE_0_Len                             (3U)
#define I2S_PARAM1_TXSIZE_0_Msk                             (0x7U << I2S_PARAM1_TXSIZE_0_Pos)
#define I2S_PARAM1_TXSIZE_0                                 I2S_PARAM1_TXSIZE_0_Msk

#define I2S_PARAM1_TXCHN_Pos                                (9U)
#define I2S_PARAM1_TXCHN_Len                                (2U)
#define I2S_PARAM1_TXCHN_Msk                                (0x3U << I2S_PARAM1_TXCHN_Pos)
#define I2S_PARAM1_TXCHN                                    I2S_PARAM1_TXCHN_Msk

#define I2S_PARAM1_RXCHN_Pos                                (7U)
#define I2S_PARAM1_RXCHN_Len                                (2U)
#define I2S_PARAM1_RXCHN_Msk                                (0x3U << I2S_PARAM1_RXCHN_Pos)
#define I2S_PARAM1_RXCHN                                    I2S_PARAM1_RXCHN_Msk

#define I2S_PARAM1_RXBLOCK_Pos                              (6U)
#define I2S_PARAM1_RXBLOCK_Len                              (1U)
#define I2S_PARAM1_RXBLOCK_Msk                              (0x1U << I2S_PARAM1_RXBLOCK_Pos)
#define I2S_PARAM1_RXBLOCK                                  I2S_PARAM1_RXBLOCK_Msk

#define I2S_PARAM1_TXBLOCK_Pos                              (5U)
#define I2S_PARAM1_TXBLOCK_Len                              (1U)
#define I2S_PARAM1_TXBLOCK_Msk                              (0x1U << I2S_PARAM1_TXBLOCK_Pos)
#define I2S_PARAM1_TXBLOCK                                  I2S_PARAM1_TXBLOCK_Msk

#define I2S_PARAM1_MODE_Pos                                 (4U)
#define I2S_PARAM1_MODE_Len                                 (1U)
#define I2S_PARAM1_MODE_Msk                                 (0x1U << I2S_PARAM1_MODE_Pos)
#define I2S_PARAM1_MODE                                     I2S_PARAM1_MODE_Msk

#define I2S_PARAM1_FIFO_DEPTH_Pos                           (2U)
#define I2S_PARAM1_FIFO_DEPTH_Len                           (2U)
#define I2S_PARAM1_FIFO_DEPTH_Msk                           (0x3U << I2S_PARAM1_FIFO_DEPTH_Pos)
#define I2S_PARAM1_FIFO_DEPTH                               I2S_PARAM1_FIFO_DEPTH_Msk

#define I2S_PARAM1_APB_DATA_WIDTH_Pos                       (0U)
#define I2S_PARAM1_APB_DATA_WIDTH_Len                       (2U)
#define I2S_PARAM1_APB_DATA_WIDTH_Msk                       (0x3U << I2S_PARAM1_APB_DATA_WIDTH_Pos)
#define I2S_PARAM1_APB_DATA_WIDTH                           I2S_PARAM1_APB_DATA_WIDTH_Msk

/******************  Bit definition for I2S_VERSION register  *****************/
#define I2S_COMP_VERSION_Pos                                (0U)
#define I2S_COMP_VERSION_Len                                (32U)
#define I2S_COMP_VERSION_Msk                                (0xFFFFFFFFU)
#define I2S_COMP_VERSION                                    I2S_COMP_VERSION_Msk

/*******************  Bit definition for I2S_TYPE register  *******************/
#define I2S_COMP_TYPE_Pos                                   (0U)
#define I2S_COMP_TYPE_Len                                   (32U)
#define I2S_COMP_TYPE_Msk                                   (0xFFFFFFFFU)
#define I2S_COMP_TYPE                                       I2S_COMP_TYPE_Msk


/* ================================================================================================================= */
/* ================                                        ISO7816                                  ================ */
/* ================================================================================================================= */
/*******************  Bit definition for ISO7816_CTRL/ISO7816_STAT register  ******************/
#define ISO7816_INTR_ALL                                    (0x43F00000)

#define ISO7816_INTR_TEST_Pos                               (30U)
#define ISO7816_INTR_TEST_Len                               (1U)
#define ISO7816_INTR_TEST_Msk                               (0x1U << ISO7816_INTR_TEST_Pos)
#define ISO7816_INTR_TEST                                   ISO7816_INTR_TEST_Msk

#define ISO7816_INTR_PRESENCE_Pos                           (25U)
#define ISO7816_INTR_PRESENCE_Len                           (1U)
#define ISO7816_INTR_PRESENCE_Msk                           (0x1U << ISO7816_INTR_PRESENCE_Pos)
#define ISO7816_INTR_PRESENCE                               ISO7816_INTR_PRESENCE_Msk

#define ISO7816_INTR_STATE_ERR_Pos                          (24U)
#define ISO7816_INTR_STATE_ERR_Len                          (1U)
#define ISO7816_INTR_STATE_ERR_Msk                          (0x1U << ISO7816_INTR_STATE_ERR_Pos)
#define ISO7816_INTR_STATE_ERR                              ISO7816_INTR_STATE_ERR_Msk

#define ISO7816_INTR_DMA_ERR_Pos                            (23U)
#define ISO7816_INTR_DMA_ERR_Len                            (1U)
#define ISO7816_INTR_DMA_ERR_Msk                            (0x1U << ISO7816_INTR_DMA_ERR_Pos)
#define ISO7816_INTR_DMA_ERR                                ISO7816_INTR_DMA_ERR_Msk

#define ISO7816_INTR_RETRY_ERR_Pos                          (22U)
#define ISO7816_INTR_RETRY_ERR_Len                          (1U)
#define ISO7816_INTR_RETRY_ERR_Msk                          (0x1U << ISO7816_INTR_RETRY_ERR_Pos)
#define ISO7816_INTR_RETRY_ERR                              ISO7816_INTR_RETRY_ERR_Msk

#define ISO7816_INTR_RX_ERR_Pos                             (21U)
#define ISO7816_INTR_RX_ERR_Len                             (1U)
#define ISO7816_INTR_RX_ERR_Msk                             (0x1U << ISO7816_INTR_RX_ERR_Pos)
#define ISO7816_INTR_RX_ERR                                 ISO7816_INTR_RX_ERR_Msk

#define ISO7816_INTR_DONE_Pos                               (20U)
#define ISO7816_INTR_DONE_Len                               (1U)
#define ISO7816_INTR_DONE_Msk                               (0x1U << ISO7816_INTR_DONE_Pos)
#define ISO7816_INTR_DONE                                   ISO7816_INTR_DONE_Msk

/*******************  Bit definition for ISO7816_CTRL register  ******************/
#define ISO7816_CTRL_IRQ_TEST_SET_Pos                       (31U)
#define ISO7816_CTRL_IRQ_TEST_SET_Len                       (1U)
#define ISO7816_CTRL_IRQ_TEST_SET_Msk                       (0x1U << ISO7816_CTRL_IRQ_TEST_SET_Pos)
#define ISO7816_CTRL_IRQ_TEST_SET                           ISO7816_CTRL_IRQ_TEST_SET_Msk

#define ISO7816_CTRL_TX_RETRY_MAX_CLR_Pos                   (12U)
#define ISO7816_CTRL_TX_RETRY_MAX_CLR_Len                   (1U)
#define ISO7816_CTRL_TX_RETRY_MAX_CLR_Msk                   (0x1U << ISO7816_CTRL_TX_RETRY_MAX_CLR_Pos)
#define ISO7816_CTRL_TX_RETRY_MAX_CLR                       ISO7816_CTRL_TX_RETRY_MAX_CLR_Msk

#define ISO7816_CTRL_RX_RETRY_MAX_CLR_Pos                   (8U)
#define ISO7816_CTRL_RX_RETRY_MAX_CLR_Len                   (1U)
#define ISO7816_CTRL_RX_RETRY_MAX_CLR_Msk                   (0x1U << ISO7816_CTRL_RX_RETRY_MAX_CLR_Pos)
#define ISO7816_CTRL_RX_RETRY_MAX_CLR                       ISO7816_CTRL_RX_RETRY_MAX_CLR_Msk

#define ISO7816_CTRL_ACTION_Pos                             (0U)
#define ISO7816_CTRL_ACTION_Len                             (3U)
#define ISO7816_CTRL_ACTION_Msk                             (0x7U << ISO7816_CTRL_ACTION_Pos)
#define ISO7816_CTRL_ACTION                                 ISO7816_CTRL_ACTION_Msk

/*******************  Bit definition for ISO7816_STAT register  ******************/
#define ISO7816_STAT_PRESENCE_Pos                           (17U)
#define ISO7816_STAT_PRESENCE_Len                           (1U)
#define ISO7816_STAT_PRESENCE_Msk                           (0x1U << ISO7816_STAT_PRESENCE_Pos)
#define ISO7816_STAT_PRESENCE                               ISO7816_STAT_PRESENCE_Msk

#define ISO7816_STAT_BUSY_Pos                               (16U)
#define ISO7816_STAT_BUSY_Len                               (1U)
#define ISO7816_STAT_BUSY_Msk                               (0x1U << ISO7816_STAT_BUSY_Pos)
#define ISO7816_STAT_BUSY                                   ISO7816_STAT_BUSY_Msk

#define ISO7816_STAT_TX_RETRY_MAX_Pos                       (12U)
#define ISO7816_STAT_TX_RETRY_MAX_Len                       (3U)
#define ISO7816_STAT_TX_RETRY_MAX_Msk                       (0x7U << ISO7816_STAT_TX_RETRY_MAX_Pos)
#define ISO7816_STAT_TX_RETRY_MAX                           ISO7816_STAT_TX_RETRY_MAX_Msk

#define ISO7816_STAT_RX_RETRY_MAX_Pos                       (8U)
#define ISO7816_STAT_RX_RETRY_MAX_Len                       (3U)
#define ISO7816_STAT_RX_RETRY_MAX_Msk                       (0x7U << ISO7816_STAT_RX_RETRY_MAX_Pos)
#define ISO7816_STAT_RX_RETRY_MAX                           ISO7816_STAT_RX_RETRY_MAX_Msk

#define ISO7816_STAT_IO_STATE_Pos                           (4U)
#define ISO7816_STAT_IO_STATE_Len                           (3U)
#define ISO7816_STAT_IO_STATE_Msk                           (0x7U << ISO7816_STAT_IO_STATE_Pos)
#define ISO7816_STAT_IO_STATE                               ISO7816_STAT_IO_STATE_Msk

#define ISO7816_STAT_PWR_STATE_Pos                          (0U)
#define ISO7816_STAT_PWR_STATE_Len                          (4U)
#define ISO7816_STAT_PWR_STATE_Msk                          (0xFU << ISO7816_STAT_PWR_STATE_Pos)
#define ISO7816_STAT_PWR_STATE                              ISO7816_STAT_PWR_STATE_Msk

/*******************  Bit definition for ISO7816_CLK_CFG register  ******************/
#define ISO7816_CLK_CFG_CLK_STOP_SEL_Pos                    (31U)
#define ISO7816_CLK_CFG_CLK_STOP_SEL_Len                    (1U)
#define ISO7816_CLK_CFG_CLK_STOP_SEL_Msk                    (0x1U << ISO7816_CLK_CFG_CLK_STOP_SEL_Pos)
#define ISO7816_CLK_CFG_CLK_STOP_SEL                        ISO7816_CLK_CFG_CLK_STOP_SEL_Msk

#define ISO7816_CLK_CFG_CLK_DIV_Pos                         (16U)
#define ISO7816_CLK_CFG_CLK_DIV_Len                         (8U)
#define ISO7816_CLK_CFG_CLK_DIV_Msk                         (0xFFU << ISO7816_CLK_CFG_CLK_DIV_Pos)
#define ISO7816_CLK_CFG_CLK_DIV                             ISO7816_CLK_CFG_CLK_DIV_Msk

#define ISO7816_CLK_CFG_ETU_DIV_Pos                         (0U)
#define ISO7816_CLK_CFG_ETU_DIV_Len                         (10U)
#define ISO7816_CLK_CFG_ETU_DIV_Msk                         (0x3FFU << ISO7816_CLK_CFG_ETU_DIV_Pos)
#define ISO7816_CLK_CFG_ETU_DIV                             ISO7816_CLK_CFG_ETU_DIV_Msk

/*******************  Bit definition for ISO7816_VCC_CFG register  ******************/
#define ISO7816_VCC_CFG_VCC_SEL_Pos                         (0U)
#define ISO7816_VCC_CFG_VCC_SEL_Len                         (2U)
#define ISO7816_VCC_CFG_VCC_SEL_Msk                         (0x3U << ISO7816_VCC_CFG_VCC_SEL_Pos)
#define ISO7816_VCC_CFG_VCC_SEL                             ISO7816_VCC_CFG_VCC_SEL_Msk

/*******************  Bit definition for ISO7816_TIMES register  ********************/
#define ISO7816_TIMES_WAITTIME_Pos                          (12U)
#define ISO7816_TIMES_WAITTIME_Len                          (18U)
#define ISO7816_TIMES_WAITTIME_Msk                          (0x3FFFFU << ISO7816_TIMES_WAITTIME_Pos)
#define ISO7816_TIMES_WAITTIME                              ISO7816_TIMES_WAITTIME_Msk

#define ISO7816_TIMES_DUARDTIME_Pos                         (0U)
#define ISO7816_TIMES_DUARDTIME_Len                         (10U)
#define ISO7816_TIMES_DUARDTIME_Msk                         (0x3FFU << ISO7816_TIMES_DUARDTIME_Pos)
#define ISO7816_TIMES_DUARDTIME                             ISO7816_TIMES_DUARDTIME_Msk

/*******************  Bit definition for ISO7816_DATA_CFG register  ********************/
#define ISO7816_DATA_CFG_RETRY_LIMIT_Pos                    (4U)
#define ISO7816_DATA_CFG_RETRY_LIMIT_Len                    (3U)
#define ISO7816_DATA_CFG_RETRY_LIMIT_Msk                    (0x7U << ISO7816_DATA_CFG_RETRY_LIMIT_Pos)
#define ISO7816_DATA_CFG_RETRY_LIMIT                        ISO7816_DATA_CFG_RETRY_LIMIT_Msk

#define ISO7816_DATA_CFG_DETECT_CODING_Pos                  (1U)
#define ISO7816_DATA_CFG_DETECT_CODING_Len                  (1U)
#define ISO7816_DATA_CFG_DETECT_CODING_Msk                  (0x1U << ISO7816_DATA_CFG_DETECT_CODING_Pos)
#define ISO7816_DATA_CFG_DETECT_CODING                      ISO7816_DATA_CFG_DETECT_CODING_Msk

#define ISO7816_DATA_CFG_CODING_Pos                         (0U)
#define ISO7816_DATA_CFG_CODING_Len                         (1U)
#define ISO7816_DATA_CFG_CODING_Msk                         (0x1U << ISO7816_DATA_CFG_CODING_Pos)
#define ISO7816_DATA_CFG_CODING                             ISO7816_DATA_CFG_CODING_Msk

/*******************  Bit definition for ISO7816_ADDR register  ********************/
#define ISO7816_ADDR_CUR_Pos                                (2U)
#define ISO7816_ADDR_CUR_Len                                (18U)
#define ISO7816_ADDR_CUR_Msk                                (0x3FFFFU << ISO7816_ADDR_CUR_Pos)
#define ISO7816_ADDR_CUR                                    ISO7816_ADDR_CUR_Msk

#define ISO7816_ADDR_FRAC_Pos                               (0U)
#define ISO7816_ADDR_FRAC_Len                               (2U)
#define ISO7816_ADDR_FRAC_Msk                               (0x3U << ISO7816_ADDR_FRAC_Pos)
#define ISO7816_ADDR_FRAC                                   ISO7816_ADDR_FRAC_Msk

/*******************  Bit definition for ISO7816_STRT_ADDR register  ********************/
#define ISO7816_STRT_ADDR_BASE_ADDR_Pos                     (20U)
#define ISO7816_STRT_ADDR_BASE_ADDR_Len                     (12U)
#define ISO7816_STRT_ADDR_BASE_ADDR_Msk                     (0xFFFU << ISO7816_STRT_ADDR_BASE_ADDR_Pos)
#define ISO7816_STRT_ADDR_BASE_ADDR                         ISO7816_STRT_ADDR_BASE_ADDR_Msk

#define ISO7816_STRT_ADDR_STRT_ADDR_Pos                     (2U)
#define ISO7816_STRT_ADDR_STRT_ADDR_Len                     (18U)
#define ISO7816_STRT_ADDR_STRT_ADDR_Msk                     (0x3FFFFU << ISO7816_STRT_ADDR_STRT_ADDR_Pos)
#define ISO7816_STRT_ADDR_STRT_ADDR                         ISO7816_STRT_ADDR_STRT_ADDR_Msk

/*******************  Bit definition for ISO7816_RX_END_ADDR register  ********************/
#define ISO7816_RX_END_ADDR_Pos                             (2U)
#define ISO7816_RX_END_ADDR_Len                             (18U)
#define ISO7816_RX_END_ADDR_Msk                             (0x3FFFFU << ISO7816_RX_END_ADDR_Pos)
#define ISO7816_RX_END_ADDR                                 ISO7816_RX_END_ADDR_Msk

#define ISO7816_RX_END_ADDR_FRAC_Pos                        (0U)
#define ISO7816_RX_END_ADDR_FRAC_Len                        (2U)
#define ISO7816_RX_END_ADDR_FRAC_Msk                        (0x3U << ISO7816_RX_END_ADDR_FRAC_Pos)
#define ISO7816_RX_END_ADDR_FRAC                            ISO7816_RX_END_ADDR_FRAC_Msk

/*******************  Bit definition for ISO7816_TX_END_ADDR register  ********************/
#define ISO7816_TX_END_ADDR_Pos                             (2U)
#define ISO7816_TX_END_ADDR_Len                             (18U)
#define ISO7816_TX_END_ADDR_Msk                             (0x3FFFFU << ISO7816_TX_END_ADDR_Pos)
#define ISO7816_TX_END_ADDR                                 ISO7816_TX_END_ADDR_Msk

#define ISO7816_TX_END_ADDR_FRAC_Pos                        (0U)
#define ISO7816_TX_END_ADDR_FRAC_Len                        (2U)
#define ISO7816_TX_END_ADDR_FRAC_Msk                        (0x3U << ISO7816_TX_END_ADDR_FRAC_Pos)
#define ISO7816_TX_END_ADDR_FRAC                            ISO7816_TX_END_ADDR_FRAC_Msk

/* ================================================================================================================= */
/* ================                                      MCU_SUB                                    ================ */
/* ================================================================================================================= */
/*******************  Bit definition for MCU_SUB_REG_SENSE_ADC_FIFO register  ****************/
#define MCU_SUB_SNSADC_FF_DATA_Pos                          (0U)
#define MCU_SUB_SNSADC_FF_DATA_Len                          (32U)
#define MCU_SUB_SNSADC_FF_DATA_Msk                          (0xFFFFFFFFU)
#define MCU_SUB_SNSADC_FF_DATA                              MCU_SUB_SNSADC_FF_DATA_Msk

/*******************  Bit definition for MCU_SUB_REG_SENSE_FF_THRESH register  ****/
#define MCU_SUB_SNSADC_FF_THRESH_Pos                        (0U)
#define MCU_SUB_SNSADC_FF_THRESH_Len                        (6U)
#define MCU_SUB_SNSADC_FF_THRESH_Msk                        (0x3FU << MCU_SUB_SNSADC_FF_THRESH_Pos)
#define MCU_SUB_SNSADC_FF_THRESH                            MCU_SUB_SNSADC_FF_THRESH_Msk

/*******************  Bit definition for MCU_SUB_REG_SENSE_ADC_STAT register  *****/
#define MCU_SUB_SNSADC_STAT_VAL_Pos                         (8U)
#define MCU_SUB_SNSADC_STAT_VAL_Len                         (1U)
#define MCU_SUB_SNSADC_STAT_VAL_Msk                         (0x1U << MCU_SUB_SNSADC_STAT_VAL_Pos)
#define MCU_SUB_SNSADC_STAT_VAL                             MCU_SUB_SNSADC_STAT_VAL_Msk

#define MCU_SUB_SNSADC_STAT_FF_COUNT_Pos                    (0U)
#define MCU_SUB_SNSADC_STAT_FF_COUNT_Len                    (7U)
#define MCU_SUB_SNSADC_STAT_FF_COUNT_Msk                    (0x7FU << MCU_SUB_SNSADC_STAT_FF_COUNT_Pos)
#define MCU_SUB_SNSADC_STAT_FF_COUNT                        MCU_SUB_SNSADC_STAT_FF_COUNT_Msk


/*******************  Bit definition for MCU_SUB_REG_COMM_TMR_DEEPSLPSTAT register  ***********/
#define MCU_SUB_COMM_TMR_DEEPSLPSTAT_DEEPSLDUR_Pos          (0U)
#define MCU_SUB_COMM_TMR_DEEPSLPSTAT_DEEPSLDUR_Len          (32U)
#define MCU_SUB_COMM_TMR_DEEPSLPSTAT_DEEPSLDUR_Msk          (0xFFFFFFFFU)
#define MCU_SUB_COMM_TMR_DEEPSLPSTAT_DEEPSLDUR              MCU_SUB_COMM_TMR_DEEPSLPSTAT_DEEPSLDUR_Msk

/***************  Bit definition for MCU_SUB_REG_DPAD_RE_N_BUS register  ********/
#define MCU_SUB_DPAD_RE_N_BUS_Pos                           (0U)
#define MCU_SUB_DPAD_RE_N_BUS_Len                           (32U)
#define MCU_SUB_DPAD_RE_N_BUS_Msk                           (0xFFFFFFFFU)
#define MCU_SUB_DPAD_RE_N_BUS                               MCU_SUB_DPAD_RE_N_BUS_Msk

/*************  Bit definition for MCU_SUB_REG_DPAD_RTYP_BUS register  **********/
#define MCU_SUB_DPAD_RTYP_BUS_Pos                           (0U)
#define MCU_SUB_DPAD_RTYP_BUS_Len                           (32U)
#define MCU_SUB_DPAD_RTYP_BUS_Msk                           (0xFFFFFFFFU)
#define MCU_SUB_DPAD_RTYP_BUS                               MCU_SUB_DPAD_RTYP_BUS_Msk

/**********  Bit definition for MCU_SUB_REG_DPAD_IE_N_BUS register  *************/
#define MCU_SUB_DPAD_IE_N_BUS_Pos                           (0U)
#define MCU_SUB_DPAD_IE_N_BUS_Len                           (32U)
#define MCU_SUB_DPAD_IE_N_BUS_Msk                           (0xFFFFFFFFU)
#define MCU_SUB_DPAD_IE_N_BUS                               MCU_SUB_DPAD_IE_N_BUS_Msk

/**********  Bit definition for MCU_SUB_REG_MSIO_REG register  ******************/
#define MCU_SUB_MSIO_REG0_PFAST_CS_CTRL_Pos                 (8U)
#define MCU_SUB_MSIO_REG0_PFAST_CS_CTRL_Len                 (4U)
#define MCU_SUB_MSIO_REG0_PFAST_CS_CTRL_Msk                 (0xFU << MCU_SUB_MSIO_REG0_PFAST_CS_CTRL_Pos)
#define MCU_SUB_MSIO_REG0_PFAST_CS_CTRL                     MCU_SUB_MSIO_REG0_PFAST_CS_CTRL_Msk

#define MCU_SUB_MSIO_REG0_MSIO_C_Pos                        (0U)
#define MCU_SUB_MSIO_REG0_MSIO_C_Len                        (5U)
#define MCU_SUB_MSIO_REG0_MSIO_C_Msk                        (0x1FU << MCU_SUB_MSIO_REG0_MSIO_C_Pos)
#define MCU_SUB_MSIO_REG0_MSIO_C                            MCU_SUB_MSIO_REG0_MSIO_C_Msk

/**********  Bit definition for MCU_SUB_REG_BLE_FERP_CTL register  ****************/
#define MUC_SUB_BLE_FERP_CTL_TESTBUS_SEL_Pos                (4U)
#define MUC_SUB_BLE_FERP_CTL_TESTBUS_SEL_Len                (3U)
#define MUC_SUB_BLE_FERP_CTL_TESTBUS_SEL_Msk                (0x7U << MUC_SUB_BLE_FERP_CTL_TESTBUS_SEL_Pos)
#define MUC_SUB_BLE_FERP_CTL_TESTBUS_SEL                    MUC_SUB_BLE_FERP_CTL_TESTBUS_SEL_Msk

#define MCU_SUB_BLE_FERP_CTL_FERP_EN_Pos                    (0U)
#define MCU_SUB_BLE_FERP_CTL_FERP_EN_Len                    (1U)
#define MCU_SUB_BLE_FERP_CTL_FERP_EN_Msk                    (0x1U << MCU_SUB_BLE_FERP_CTL_FERP_EN_Pos)
#define MCU_SUB_BLE_FERP_CTL_FERP_EN                        MCU_SUB_BLE_FERP_CTL_FERP_EN_Msk

/**********  Bit definition for MCU_SUB_REG_DMA_ACC_SEL register  ****************/
#define MCU_SUB_DMA_ACC_SEL_I2C1_I2SS_Pos                   (1U)
#define MCU_SUB_DMA_ACC_SEL_I2C1_I2SS_Len                   (1U)
#define MCU_SUB_DMA_ACC_SEL_I2C1_I2SS_Msk                   (0x1U << MCU_SUB_DMA_ACC_SEL_I2C1_I2SS_Pos)
#define MCU_SUB_DMA_ACC_SEL_I2C1_I2SS                       MCU_SUB_DMA_ACC_SEL_I2C1_I2SS_Msk

#define MCU_SUB_DMA_ACC_SEL_QSPI1_I2SM_Pos                  (0U)
#define MCU_SUB_DMA_ACC_SEL_QSPI1_I2SM_Len                  (1U)
#define MCU_SUB_DMA_ACC_SEL_QSPI1_I2SM_Msk                  (0x1U << MCU_SUB_DMA_ACC_SEL_QSPI1_I2SM_Pos)
#define MCU_SUB_DMA_ACC_SEL_QSPI1_I2SM                      MCU_SUB_DMA_ACC_SEL_QSPI1_I2SM_Msk

/**********  Bit definition for MCU_SUB_REG_SECURITY_RESET register  ****************/
#define MCU_SUB_SECURITY_RESET_PRESENT_Pos                  (6U)
#define MCU_SUB_SECURITY_RESET_PRESENT_Len                  (1U)
#define MCU_SUB_SECURITY_RESET_PRESENT_Msk                  (0x1U << MCU_SUB_SECURITY_RESET_PRESENT_Pos)
#define MCU_SUB_SECURITY_RESET_PRESENT                      MCU_SUB_SECURITY_RESET_PRESENT_Msk

#define MCU_SUB_SECURITY_RESET_TRNG_Pos                     (5U)
#define MCU_SUB_SECURITY_RESET_TRNG_Len                     (1U)
#define MCU_SUB_SECURITY_RESET_TRNG_Msk                     (0x1U << MCU_SUB_SECURITY_RESET_TRNG_Pos)
#define MCU_SUB_SECURITY_RESET_TRNG                         MCU_SUB_SECURITY_RESET_TRNG_Msk

#define MCU_SUB_SECURITY_RESET_RAMKEY_Pos                   (4U)
#define MCU_SUB_SECURITY_RESET_RAMKEY_Len                   (1U)
#define MCU_SUB_SECURITY_RESET_RAMKEY_Msk                   (0x1U << MCU_SUB_SECURITY_RESET_RAMKEY_Pos)
#define MCU_SUB_SECURITY_RESET_RAMKEY                       MCU_SUB_SECURITY_RESET_RAMKEY_Msk

#define MCU_SUB_SECURITY_RESET_EFUSE_Pos                    (3U)
#define MCU_SUB_SECURITY_RESET_EFUSE_Len                    (1U)
#define MCU_SUB_SECURITY_RESET_EFUSE_Msk                    (0x1U << MCU_SUB_SECURITY_RESET_EFUSE_Pos)
#define MCU_SUB_SECURITY_RESET_EFUSE                        MCU_SUB_SECURITY_RESET_EFUSE_Msk

#define MCU_SUB_SECURITY_RESET_PKC_Pos                      (2U)
#define MCU_SUB_SECURITY_RESET_PKC_Len                      (1U)
#define MCU_SUB_SECURITY_RESET_PKC_Msk                      (0x1U << MCU_SUB_SECURITY_RESET_PKC_Pos)
#define MCU_SUB_SECURITY_RESET_PKC                          MCU_SUB_SECURITY_RESET_PKC_Msk

#define MCU_SUB_SECURITY_RESET_HMAC_Pos                     (1U)
#define MCU_SUB_SECURITY_RESET_HMAC_Len                     (1U)
#define MCU_SUB_SECURITY_RESET_HMAC_Msk                     (0x1U << MCU_SUB_SECURITY_RESET_HMAC_Pos)
#define MCU_SUB_SECURITY_RESET_HMAC                         MCU_SUB_SECURITY_RESET_HMAC_Msk

#define MCU_SUB_SECURITY_RESET_AES_Pos                      (0U)
#define MCU_SUB_SECURITY_RESET_AES_Len                      (1U)
#define MCU_SUB_SECURITY_RESET_AES_Msk                      (0x1U << MCU_SUB_SECURITY_RESET_AES_Pos)
#define MCU_SUB_SECURITY_RESET_AES                          MCU_SUB_SECURITY_RESET_AES_Msk

/**********  Bit definition for MCU_SUB_REG_PMU_ID_REG register  *****************/
#define MCU_SUB_PMU_ID_Pos                                  (0U)
#define MCU_SUB_PMU_ID_Len                                  (8U)
#define MCU_SUB_PMU_ID_Msk                                  (0xFFU << MCU_SUB_PMU_ID_Pos)
#define MCU_SUB_PMU_ID                                      MCU_SUB_PMU_ID_Msk

/**********  Bit definition for MCU_SUB_REG_PWR_AVG_CTL_REG register  ************/
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

/**********  Bit definition for MCU_SUB_REG_CLK_CAL_CTL_REG0 register  ************/
#define MCU_SUB_CLK_CAL_CTL0_EN_Pos                         (0U)
#define MCU_SUB_CLK_CAL_CTL0_EN_Len                         (1U)
#define MCU_SUB_CLK_CAL_CTL0_EN_Msk                         (0x1U << MCU_SUB_CLK_CAL_CTL0_EN_Pos)
#define MCU_SUB_CLK_CAL_CTL0_EN                             MCU_SUB_CLK_CAL_CTL0_EN_Msk

#define MCU_SUB_CLK_CAL_CTL0_CNT_LOAD_EN_Pos                (1U)
#define MCU_SUB_CLK_CAL_CTL0_CNT_LOAD_EN_Len                (1U)
#define MCU_SUB_CLK_CAL_CTL0_CNT_LOAD_EN_Msk                (0x1U << MCU_SUB_CLK_CAL_CTL0_CNT_LOAD_EN_Pos)
#define MCU_SUB_CLK_CAL_CTL0_CNT_LOAD_EN                    MCU_SUB_CLK_CAL_CTL0_CNT_LOAD_EN_Msk

#define MCU_SUB_CLK_CAL_CTL0_CNT_LOAD_VAL_Pos               (4U)
#define MCU_SUB_CLK_CAL_CTL0_CNT_LOAD_VAL_Len               (12U)
#define MCU_SUB_CLK_CAL_CTL0_CNT_LOAD_VAL_Msk               (0xFFFU << MCU_SUB_CLK_CAL_CTL0_CNT_LOAD_VAL_Pos)
#define MCU_SUB_CLK_CAL_CTL0_CNT_LOAD_VAL                   MCU_SUB_CLK_CAL_CTL0_CNT_LOAD_VAL_Msk

#define MCU_SUB_CLK_CAL_CTL0_STS_CNT_RDY_Pos                (16U)
#define MCU_SUB_CLK_CAL_CTL0_STS_CNT_RDY_Len                (1U)
#define MCU_SUB_CLK_CAL_CTL0_STS_CNT_RDY_Msk                (0x1U << MCU_SUB_CLK_CAL_CTL0_STS_CNT_RDY_Pos)
#define MCU_SUB_CLK_CAL_CTL0_STS_CNT_RDY                    MCU_SUB_CLK_CAL_CTL0_STS_CNT_RDY_Msk

/**********  Bit definition for MCU_SUB_REG_CLK_CAL_CTL_REG1 register  ************/
#define MCU_SUB_CLK_CAL_CTL1_STS_CNT_VAL_Pos                (0U)
#define MCU_SUB_CLK_CAL_CTL1_STS_CNT_VAL_Len                (24U)
#define MCU_SUB_CLK_CAL_CTL1_STS_CNT_VAL_Msk                (0xFFFFFFU << MCU_SUB_CLK_CAL_CTL1_STS_CNT_VAL_Pos)
#define MCU_SUB_CLK_CAL_CTL1_STS_CNT_VAL                    MCU_SUB_CLK_CAL_CTL1_STS_CNT_VAL_Msk

/**********  Bit definition for MCU_SUB_REG_DPAD_MUX_CTL_00_07 register  **********/
#define MCU_SUB_DPAD_MUX_CTL_00_07_Pos                      (0U)
#define MCU_SUB_DPAD_MUX_CTL_00_07_Len                      (32U)
#define MCU_SUB_DPAD_MUX_CTL_00_07_Msk                      (0xFFFFFFFFU)
#define MCU_SUB_DPAD_MUX_CTL_00_07                          MCU_SUB_DPAD_MUX_CTL_00_07_Msk

#define MCU_SUB_DPAD_MUX_CTL_SEL_Msk                        (0xFU)
#define MCU_SUB_DPAD_MUX_CTL_SEL_00                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 0)
#define MCU_SUB_DPAD_MUX_CTL_SEL_01                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 4)
#define MCU_SUB_DPAD_MUX_CTL_SEL_02                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 8)
#define MCU_SUB_DPAD_MUX_CTL_SEL_03                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 12)
#define MCU_SUB_DPAD_MUX_CTL_SEL_04                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 16)
#define MCU_SUB_DPAD_MUX_CTL_SEL_05                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 20)
#define MCU_SUB_DPAD_MUX_CTL_SEL_06                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 24)
#define MCU_SUB_DPAD_MUX_CTL_SEL_07                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 28)

/**********  Bit definition for MCU_SUB_REG_DPAD_MUX_CTL_08_15 register  **********/
#define MCU_SUB_DPAD_MUX_CTL_08_15_Pos                      (0U)
#define MCU_SUB_DPAD_MUX_CTL_08_15_Len                      (32U)
#define MCU_SUB_DPAD_MUX_CTL_08_15_Msk                      (0xFFFFFFFFU)
#define MCU_SUB_DPAD_MUX_CTL_08_15                          MCU_SUB_DPAD_MUX_CTL_08_15_Msk

#define MCU_SUB_DPAD_MUX_CTL_SEL_08                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 0)
#define MCU_SUB_DPAD_MUX_CTL_SEL_09                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 4)
#define MCU_SUB_DPAD_MUX_CTL_SEL_10                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 8)
#define MCU_SUB_DPAD_MUX_CTL_SEL_11                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 12)
#define MCU_SUB_DPAD_MUX_CTL_SEL_12                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 16)
#define MCU_SUB_DPAD_MUX_CTL_SEL_13                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 20)
#define MCU_SUB_DPAD_MUX_CTL_SEL_14                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 24)
#define MCU_SUB_DPAD_MUX_CTL_SEL_15                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 28)

/**********  Bit definition for MCU_SUB_REG_DPAD_MUX_CTL_16_23 register  **********/
#define MCU_SUB_DPAD_MUX_CTL_16_23_Pos                      (0U)
#define MCU_SUB_DPAD_MUX_CTL_16_23_Len                      (32U)
#define MCU_SUB_DPAD_MUX_CTL_16_23_Msk                      (0xFFFFFFFFU)
#define MCU_SUB_DPAD_MUX_CTL_16_23                          MCU_SUB_DPAD_MUX_CTL_16_23_Msk

#define MCU_SUB_DPAD_MUX_CTL_SEL_16                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 0)
#define MCU_SUB_DPAD_MUX_CTL_SEL_17                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 4)
#define MCU_SUB_DPAD_MUX_CTL_SEL_18                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 8)
#define MCU_SUB_DPAD_MUX_CTL_SEL_19                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 12)
#define MCU_SUB_DPAD_MUX_CTL_SEL_20                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 16)
#define MCU_SUB_DPAD_MUX_CTL_SEL_21                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 20)
#define MCU_SUB_DPAD_MUX_CTL_SEL_22                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 24)
#define MCU_SUB_DPAD_MUX_CTL_SEL_23                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 28)

/**********  Bit definition for MCU_SUB_REG_DPAD_MUX_CTL_24_31 register  ***********/
#define MCU_SUB_DPAD_MUX_CTL_24_31_Pos                      (0U)
#define MCU_SUB_DPAD_MUX_CTL_24_31_Len                      (32U)
#define MCU_SUB_DPAD_MUX_CTL_24_31_Msk                      (0xFFFFFFFFU)
#define MCU_SUB_DPAD_MUX_CTL_24_31                          MCU_SUB_DPAD_MUX_CTL_24_31_Msk

#define MCU_SUB_DPAD_MUX_CTL_SEL_24                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 0)
#define MCU_SUB_DPAD_MUX_CTL_SEL_25                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 4)
#define MCU_SUB_DPAD_MUX_CTL_SEL_26                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 8)
#define MCU_SUB_DPAD_MUX_CTL_SEL_27                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 12)
#define MCU_SUB_DPAD_MUX_CTL_SEL_28                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 16)
#define MCU_SUB_DPAD_MUX_CTL_SEL_29                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 20)
#define MCU_SUB_DPAD_MUX_CTL_SEL_30                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 24)
#define MCU_SUB_DPAD_MUX_CTL_SEL_31                         (MCU_SUB_DPAD_MUX_CTL_SEL_Msk << 28)

/**********  Bit definition for MCU_SUB_REG_EFUSE_PWR_DELTA0 register  ***********/
#define MCU_SUB_EFUSE_PWR_DELTA0_Pos                        (0U)
#define MCU_SUB_EFUSE_PWR_DELTA0_Len                        (16U)
#define MCU_SUB_EFUSE_PWR_DELTA0_Msk                        (0xFFFFU << MCU_SUB_EFUSE_PWR_DELTA0_Pos)
#define MCU_SUB_EFUSE_PWR_DELTA0                            MCU_SUB_EFUSE_PWR_DELTA0_Msk

#define MCU_SUB_EFUSE_PWR_DELTA1_Pos                        (16U)
#define MCU_SUB_EFUSE_PWR_DELTA1_Len                        (16U)
#define MCU_SUB_EFUSE_PWR_DELTA1_Msk                        (0xFFFFU << MCU_SUB_EFUSE_PWR_DELTA1_Pos)
#define MCU_SUB_EFUSE_PWR_DELTA1                            MCU_SUB_EFUSE_PWR_DELTA1_Msk

/**********  Bit definition for MCU_SUB_REG_EFUSE_PWR_DELTA0 register  ***********/
#define MCU_SUB_EFUSE_PWR_DELTA2_Pos                        (0U)
#define MCU_SUB_EFUSE_PWR_DELTA2_Len                        (16U)
#define MCU_SUB_EFUSE_PWR_DELTA2_Msk                        (0xFFFFU << MCU_SUB_EFUSE_PWR_DELTA2_Pos)
#define MCU_SUB_EFUSE_PWR_DELTA2                            MCU_SUB_EFUSE_PWR_DELTA2_Msk

/**********  Bit definition for MCU_SUB_REG_EFUSE_PWR_CTRL0 register  ***********/
#define MCU_SUB_EFUSE_PWR_CTL0_EN_Pos                       (0U)
#define MCU_SUB_EFUSE_PWR_CTL0_EN_Len                       (1U)
#define MCU_SUB_EFUSE_PWR_CTL0_EN_Msk                       (0x1U << MCU_SUB_EFUSE_PWR_CTL0_EN_Pos)
#define MCU_SUB_EFUSE_PWR_CTL0_EN                           MCU_SUB_EFUSE_PWR_CTL0_EN_Msk

#define MCU_SUB_EFUSE_PWR_CTL0_BGN_Pos                      (2U)
#define MCU_SUB_EFUSE_PWR_CTL0_BGN_Len                      (1U)
#define MCU_SUB_EFUSE_PWR_CTL0_BGN_Msk                      (0x1U << MCU_SUB_EFUSE_PWR_CTL0_BGN_Pos)
#define MCU_SUB_EFUSE_PWR_CTL0_BGN                          MCU_SUB_EFUSE_PWR_CTL0_BGN_Msk

#define MCU_SUB_EFUSE_PWR_CTL0_STP_Pos                      (4U)
#define MCU_SUB_EFUSE_PWR_CTL0_STP_Len                      (1U)
#define MCU_SUB_EFUSE_PWR_CTL0_STP_Msk                      (0x1U << MCU_SUB_EFUSE_PWR_CTL0_STP_Pos)
#define MCU_SUB_EFUSE_PWR_CTL0_STP                          MCU_SUB_EFUSE_PWR_CTL0_STP_Msk

/**********  Bit definition for MCU_SUB_REG_EFUSE_PWR_CTRL1 register  ***********/
#define MCU_SUB_EFUSE_PWR_CTL0_EN_DONE_Pos                  (0U)
#define MCU_SUB_EFUSE_PWR_CTL0_EN_DONE_Len                  (1U)
#define MCU_SUB_EFUSE_PWR_CTL0_EN_DONE_Msk                  (0x1U << MCU_SUB_EFUSE_PWR_CTL0_EN_DONE_Pos)
#define MCU_SUB_EFUSE_PWR_CTL0_EN_DONE                      MCU_SUB_EFUSE_PWR_CTL0_EN_DONE_Msk

#define MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE_Pos                 (4U)
#define MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE_Len                 (1U)
#define MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE_Msk                 (0x1U << MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE_Pos)
#define MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE                     MCU_SUB_EFUSE_PWR_CTL0_DIS_DONE_Msk

/**********  Bit definition for MCU_SUB_REG_I2S_CLK_CFG register  ***********/
#define MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL_Pos                 (18U)
#define MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL_Len                 (1U)
#define MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL_Msk                 (0x1U << MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL_Pos)
#define MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL                     MCU_SUB_I2S_CLK_CFG_SRC_CLK_SEL_Msk

#define MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN_Pos                  (16U)
#define MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN_Len                  (1U)
#define MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN_Msk                  (0x1U << MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN_Pos)
#define MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN                      MCU_SUB_I2S_CLK_CFG_CLK_DIV_EN_Msk

#define MCU_SUB_I2S_CLK_CFG_DIV_CNT_Pos                     (0U)
#define MCU_SUB_I2S_CLK_CFG_DIV_CNT_Len                     (12U)
#define MCU_SUB_I2S_CLK_CFG_DIV_CNT_Msk                     (0xFFFU << MCU_SUB_I2S_CLK_CFG_DIV_CNT_Pos)
#define MCU_SUB_I2S_CLK_CFG_DIV_CNT                         MCU_SUB_I2S_CLK_CFG_DIV_CNT_Msk

/**********  Bit definition for MCU_SUB_REG_AON_PAD_MUX_CTL register  ***********/
#define MCU_SUB_AON_MUX_CTL_00_07_Pos                       (0U)
#define MCU_SUB_AON_MUX_CTL_00_07_Len                       (32U)
#define MCU_SUB_AON_MUX_CTL_00_07_Msk                       (0x00777770U)
#define MCU_SUB_AON_MUX_CTL_00_07                           MCU_SUB_AON_MUX_CTL_00_07_Msk

#define MCU_SUB_AON_MUX_CTL_SEL_Msk                         (0x7U)
#define MCU_SUB_AON_MUX_CTL_SEL_01                          (MCU_SUB_AON_MUX_CTL_SEL_Msk << 4)
#define MCU_SUB_AON_MUX_CTL_SEL_02                          (MCU_SUB_AON_MUX_CTL_SEL_Msk << 8)
#define MCU_SUB_AON_MUX_CTL_SEL_03                          (MCU_SUB_AON_MUX_CTL_SEL_Msk << 12)
#define MCU_SUB_AON_MUX_CTL_SEL_04                          (MCU_SUB_AON_MUX_CTL_SEL_Msk << 16)
#define MCU_SUB_AON_MUX_CTL_SEL_05                          (MCU_SUB_AON_MUX_CTL_SEL_Msk << 20)

/**********  Bit definition for MCU_SUB_REG_MSIO_PAD_MUX_CTL register  ***********/
#define MCU_SUB_MSIO_MUX_CTL_00_04_Pos                      (0U)
#define MCU_SUB_MSIO_MUX_CTL_00_04_Len                      (32U)
#define MCU_SUB_MSIO_MUX_CTL_00_04_Msk                      (0x77777U)
#define MCU_SUB_MSIO_MUX_CTL_00_04                          MCU_SUB_MSIO_MUX_CTL_00_04_Msk

#define MCU_SUB_MSIO_MUX_CTL_SEL_Msk                        (0x7U)
#define MCU_SUB_MSIO_MUX_CTL_SEL_00                         (MCU_SUB_MSIO_MUX_CTL_SEL_Msk << 0)
#define MCU_SUB_MSIO_MUX_CTL_SEL_01                         (MCU_SUB_MSIO_MUX_CTL_SEL_Msk << 4)
#define MCU_SUB_MSIO_MUX_CTL_SEL_02                         (MCU_SUB_MSIO_MUX_CTL_SEL_Msk << 8)
#define MCU_SUB_MSIO_MUX_CTL_SEL_03                         (MCU_SUB_MSIO_MUX_CTL_SEL_Msk << 12)
#define MCU_SUB_MSIO_MUX_CTL_SEL_04                         (MCU_SUB_MSIO_MUX_CTL_SEL_Msk << 16)

/**********  Bit definition for MCU_SUB_REG_MCU_SUBSYS_CG_CTRL_0 register  ***********/
#define MCU_SUB_WFI_MSK_HCLK_0                              (0xFFFU)

#define MCU_SUB_WFI_I2S_S_HCLK_Pos                          (11U)
#define MCU_SUB_WFI_I2S_S_HCLK_Len                          (1U)
#define MCU_SUB_WFI_I2S_S_HCLK_Msk                          (0x01 << MCU_SUB_WFI_I2S_S_HCLK_Pos)
#define MCU_SUB_WFI_I2S_S_HCLK                              MCU_SUB_WFI_I2S_S_HCLK_Msk

#define MCU_SUB_WFI_SERIAL_HCLK_Pos                         (10U)
#define MCU_SUB_WFI_SERIAL_HCLK_Len                         (1U)
#define MCU_SUB_WFI_SERIAL_HCLK_Msk                         (0x01 << MCU_SUB_WFI_SERIAL_HCLK_Pos)
#define MCU_SUB_WFI_SERIAL_HCLK                             MCU_SUB_WFI_SERIAL_HCLK_Msk

#define MCU_SUB_WFI_APB_SUB_HCLK_Pos                        (9U)
#define MCU_SUB_WFI_APB_SUB_HCLK_Len                        (1U)
#define MCU_SUB_WFI_APB_SUB_HCLK_Msk                        (0x01 << MCU_SUB_WFI_APB_SUB_HCLK_Pos)
#define MCU_SUB_WFI_APB_SUB_HCLK                            MCU_SUB_WFI_APB_SUB_HCLK_Msk

#define MCU_SUB_WFI_BLE_BRG_HCLK_Pos                        (8U)
#define MCU_SUB_WFI_BLE_BRG_HCLK_Len                        (1U)
#define MCU_SUB_WFI_BLE_BRG_HCLK_Msk                        (0x01 << MCU_SUB_WFI_BLE_BRG_HCLK_Pos)
#define MCU_SUB_WFI_BLE_BRG_HCLK                            MCU_SUB_WFI_BLE_BRG_HCLK_Msk

#define MCU_SUB_WFI_DMA_HCLK_Pos                            (7U)
#define MCU_SUB_WFI_DMA_HCLK_Len                            (1U)
#define MCU_SUB_WFI_DMA_HCLK_Msk                            (0x01 << MCU_SUB_WFI_DMA_HCLK_Pos)
#define MCU_SUB_WFI_DMA_HCLK                                MCU_SUB_WFI_DMA_HCLK_Msk

#define MCU_SUB_WFI_GPIO_HCLK_Pos                           (6U)
#define MCU_SUB_WFI_GPIO_HCLK_Len                           (1U)
#define MCU_SUB_WFI_GPIO_HCLK_Msk                           (0x01 << MCU_SUB_WFI_GPIO_HCLK_Pos)
#define MCU_SUB_WFI_GPIO_HCLK                               MCU_SUB_WFI_GPIO_HCLK_Msk

#define MCU_SUB_WFI_SNSADC_HCLK_Pos                         (5U)
#define MCU_SUB_WFI_SNSADC_HCLK_Len                         (1U)
#define MCU_SUB_WFI_SNSADC_HCLK_Msk                         (0x01 << MCU_SUB_WFI_SNSADC_HCLK_Pos)
#define MCU_SUB_WFI_SNSADC_HCLK                             MCU_SUB_WFI_SNSADC_HCLK_Msk

#define MCU_SUB_WFI_ROM_HCLK_Pos                            (4U)
#define MCU_SUB_WFI_ROM_HCLK_Len                            (1U)
#define MCU_SUB_WFI_ROM_HCLK_Msk                            (0x01 << MCU_SUB_WFI_ROM_HCLK_Pos)
#define MCU_SUB_WFI_ROM_HCLK                                MCU_SUB_WFI_ROM_HCLK_Msk

#define MCU_SUB_WFI_PWM_HCLK_Pos                            (3U)
#define MCU_SUB_WFI_PWM_HCLK_Len                            (1U)
#define MCU_SUB_WFI_PWM_HCLK_Msk                            (0x01 << MCU_SUB_WFI_PWM_HCLK_Pos)
#define MCU_SUB_WFI_PWM_HCLK                                MCU_SUB_WFI_PWM_HCLK_Msk

#define MCU_SUB_WFI_HTB_HCLK_Pos                            (2U)
#define MCU_SUB_WFI_HTB_HCLK_Len                            (1U)
#define MCU_SUB_WFI_HTB_HCLK_Msk                            (0x01 << MCU_SUB_WFI_HTB_HCLK_Pos)
#define MCU_SUB_WFI_HTB_HCLK                                MCU_SUB_WFI_HTB_HCLK_Msk

#define MCU_SUB_WFI_SIM_HCLK_Pos                            (1U)
#define MCU_SUB_WFI_SIM_HCLK_Len                            (1U)
#define MCU_SUB_WFI_SIM_HCLK_Msk                            (0x01 << MCU_SUB_WFI_SIM_HCLK_Pos)
#define MCU_SUB_WFI_SIM_HCLK                                MCU_SUB_WFI_SIM_HCLK_Msk

#define MCU_SUB_WFI_SECU_HCLK_Pos                           (0U)
#define MCU_SUB_WFI_SECU_HCLK_Len                           (1U)
#define MCU_SUB_WFI_SECU_HCLK_Msk                           (0x01 << MCU_SUB_WFI_SECU_HCLK_Pos)
#define MCU_SUB_WFI_SECU_HCLK                               MCU_SUB_WFI_SECU_HCLK_Msk

/**********  Bit definition for MCU_SUB_REG_MCU_SUBSYS_CG_CTRL_1 register  ***********/
#define MCU_SUB_FORCE_MSK_HCLK_0                            (0xFFFU)

#define MCU_SUB_FORCE_I2S_S_HCLK_Pos                        (11U)
#define MCU_SUB_FORCE_I2S_S_HCLK_Len                        (1U)
#define MCU_SUB_FORCE_I2S_S_HCLK_Msk                        (0x01 << MCU_SUB_FORCE_I2S_S_HCLK_Pos)
#define MCU_SUB_FORCE_I2S_S_HCLK                            MCU_SUB_FORCE_I2S_S_HCLK_Msk

#define MCU_SUB_FORCE_SERIAL_HCLK_Pos                       (10U)
#define MCU_SUB_FORCE_SERIAL_HCLK_Len                       (1U)
#define MCU_SUB_FORCE_SERIAL_HCLK_Msk                       (0x01 << MCU_SUB_FORCE_SERIAL_HCLK_Pos)
#define MCU_SUB_FORCE_SERIAL_HCLK                           MCU_SUB_FORCE_SERIAL_HCLK_Msk

#define MCU_SUB_FORCE_APB_SUB_HCLK_Pos                      (9U)
#define MCU_SUB_FORCE_APB_SUB_HCLK_Len                      (1U)
#define MCU_SUB_FORCE_APB_SUB_HCLK_Msk                      (0x01 << MCU_SUB_FORCE_APB_SUB_HCLK_Pos)
#define MCU_SUB_FORCE_APB_SUB_HCLK                          MCU_SUB_FORCE_APB_SUB_HCLK_Msk

#define MCU_SUB_FORCE_BLE_BRG_HCLK_Pos                      (8U)
#define MCU_SUB_FORCE_BLE_BRG_HCLK_Len                      (1U)
#define MCU_SUB_FORCE_BLE_BRG_HCLK_Msk                      (0x01 << MCU_SUB_FORCE_BLE_BRG_HCLK_Pos)
#define MCU_SUB_FORCE_BLE_BRG_HCLK                          MCU_SUB_FORCE_BLE_BRG_HCLK_Msk

#define MCU_SUB_FORCE_DMA_HCLK_Pos                          (7U)
#define MCU_SUB_FORCE_DMA_HCLK_Len                          (1U)
#define MCU_SUB_FORCE_DMA_HCLK_Msk                          (0x01 << MCU_SUB_FORCE_DMA_HCLK_Pos)
#define MCU_SUB_FORCE_DMA_HCLK                              MCU_SUB_FORCE_DMA_HCLK_Msk

#define MCU_SUB_FORCE_GPIO_HCLK_Pos                         (6U)
#define MCU_SUB_FORCE_GPIO_HCLK_Len                         (1U)
#define MCU_SUB_FORCE_GPIO_HCLK_Msk                         (0x01 << MCU_SUB_FORCE_GPIO_HCLK_Pos)
#define MCU_SUB_FORCE_GPIO_HCLK                             MCU_SUB_FORCE_GPIO_HCLK_Msk

#define MCU_SUB_FORCE_SNSADC_HCLK_Pos                       (5U)
#define MCU_SUB_FORCE_SNSADC_HCLK_Len                       (1U)
#define MCU_SUB_FORCE_SNSADC_HCLK_Msk                       (0x01 << MCU_SUB_FORCE_SNSADC_HCLK_Pos)
#define MCU_SUB_FORCE_SNSADC_HCLK                           MCU_SUB_FORCE_SNSADC_HCLK_Msk

#define MCU_SUB_FORCE_ROM_HCLK_Pos                          (4U)
#define MCU_SUB_FORCE_ROM_HCLK_Len                          (1U)
#define MCU_SUB_FORCE_ROM_HCLK_Msk                          (0x01 << MCU_SUB_FORCE_ROM_HCLK_Pos)
#define MCU_SUB_FORCE_ROM_HCLK                              MCU_SUB_FORCE_ROM_HCLK_Msk

#define MCU_SUB_FORCE_PWM_HCLK_Pos                          (3U)
#define MCU_SUB_FORCE_PWM_HCLK_Len                          (1U)
#define MCU_SUB_FORCE_PWM_HCLK_Msk                          (0x01 << MCU_SUB_FORCE_PWM_HCLK_Pos)
#define MCU_SUB_FORCE_PWM_HCLK                              MCU_SUB_FORCE_PWM_HCLK_Msk

#define MCU_SUB_FORCE_HTB_HCLK_Pos                          (2U)
#define MCU_SUB_FORCE_HTB_HCLK_Len                          (1U)
#define MCU_SUB_FORCE_HTB_HCLK_Msk                          (0x01 << MCU_SUB_FORCE_HTB_HCLK_Pos)
#define MCU_SUB_FORCE_HTB_HCLK                              MCU_SUB_FORCE_HTB_HCLK_Msk

#define MCU_SUB_FORCE_SIM_HCLK_Pos                          (1U)
#define MCU_SUB_FORCE_SIM_HCLK_Len                          (1U)
#define MCU_SUB_FORCE_SIM_HCLK_Msk                          (0x01 << MCU_SUB_FORCE_SIM_HCLK_Pos)
#define MCU_SUB_FORCE_SIM_HCLK                              MCU_SUB_FORCE_SIM_HCLK_Msk

#define MCU_SUB_FORCE_SECU_HCLK_Pos                         (0U)
#define MCU_SUB_FORCE_SECU_HCLK_Len                         (1U)
#define MCU_SUB_FORCE_SECU_HCLK_Msk                         (0x01 << MCU_SUB_FORCE_SECU_HCLK_Pos)
#define MCU_SUB_FORCE_SECU_HCLK                             MCU_SUB_FORCE_SECU_HCLK_Msk

/**********  Bit definition for MCU_SUB_REG_MCU_SUBSYS_CG_CTRL_2 register  ***********/
#define MCU_SUB_FORCE_MSK_HCLK_1                            (0x00070000U)

#define MCU_SUB_FORCE_SRAM_HCLK_Pos                         (18U)
#define MCU_SUB_FORCE_SRAM_HCLK_Len                         (1U)
#define MCU_SUB_FORCE_SRAM_HCLK_Msk                         (0x1UL << MCU_SUB_FORCE_SRAM_HCLK_Pos)
#define MCU_SUB_FORCE_SRAM_HCLK                             MCU_SUB_FORCE_SRAM_HCLK_Msk

#define MCU_SUB_FORCE_XF_XQSPI_HCLK_Pos                     (17U)
#define MCU_SUB_FORCE_XF_XQSPI_HCLK_Len                     (1U)
#define MCU_SUB_FORCE_XF_XQSPI_HCLK_Msk                     (0x1UL << MCU_SUB_FORCE_XF_XQSPI_HCLK_Pos)
#define MCU_SUB_FORCE_XF_XQSPI_HCLK                         MCU_SUB_FORCE_XF_XQSPI_HCLK_Msk

#define MCU_SUB_FORCE_AON_MCUSUB_HCLK_Pos                   (16U)
#define MCU_SUB_FORCE_AON_MCUSUB_HCLK_Len                   (1U)
#define MCU_SUB_FORCE_AON_MCUSUB_HCLK_Msk                   (0x1UL << MCU_SUB_FORCE_AON_MCUSUB_HCLK_Pos)
#define MCU_SUB_FORCE_AON_MCUSUB_HCLK                       MCU_SUB_FORCE_AON_MCUSUB_HCLK_Msk

#define MCU_SUB_WFI_MSK_HCLK_1                              (0x00000007U)

#define MCU_SUB_WFI_SRAM_HCLK_Pos                           (2U)
#define MCU_SUB_WFI_SRAM_HCLK_Len                           (1U)
#define MCU_SUB_WFI_SRAM_HCLK_Msk                           (0x1UL << MCU_SUB_WFI_SRAM_HCLK_Pos)
#define MCU_SUB_WFI_SRAM_HCLK                               MCU_SUB_WFI_SRAM_HCLK_Msk

#define MCU_SUB_WFI_XF_XQSPI_HCLK_Pos                       (1U)
#define MCU_SUB_WFI_XF_XQSPI_HCLK_Len                       (1U)
#define MCU_SUB_WFI_XF_XQSPI_HCLK_Msk                       (0x1UL << MCU_SUB_WFI_XF_XQSPI_HCLK_Pos)
#define MCU_SUB_WFI_XF_XQSPI_HCLK                           MCU_SUB_WFI_XF_XQSPI_HCLK_Msk

#define MCU_SUB_WFI_AON_MCUSUB_HCLK_Pos                     (0U)
#define MCU_SUB_WFI_AON_MCUSUB_HCLK_Len                     (1U)
#define MCU_SUB_WFI_AON_MCUSUB_HCLK_Msk                     (0x1UL << MCU_SUB_WFI_AON_MCUSUB_HCLK_Pos)
#define MCU_SUB_WFI_AON_MCUSUB_HCLK                         MCU_SUB_WFI_AON_MCUSUB_HCLK_Msk

/**********  Bit definition for MCU_SUB_REG_MCU_PERIPH_GC register  ***********/
#define MCU_SUB_FORCE_MSK_HCLK_2                            (0x0A01FF00U)
#define MCU_SUB_WFI_MSK_HCLK_2                              (0x05000000U)
#define MCU_SUB_FORCE_XQSPI_DIV4_PCLK_Pos                   (27U)
#define MCU_SUB_FORCE_XQSPI_DIV4_PCLK_Len                   (1U)
#define MCU_SUB_FORCE_XQSPI_DIV4_PCLK_Msk                   (0x1UL << MCU_SUB_FORCE_XQSPI_DIV4_PCLK_Pos)
#define MCU_SUB_FORCE_XQSPI_DIV4_PCLK                       MCU_SUB_FORCE_XQSPI_DIV4_PCLK_Msk

#define MCU_SUB_WFI_XQSPI_DIV4_PCLK_Pos                     (26U)
#define MCU_SUB_WFI_XQSPI_DIV4_PCLK_Len                     (1U)
#define MCU_SUB_WFI_XQSPI_DIV4_PCLK_Msk                     (0x1UL << MCU_SUB_WFI_XQSPI_DIV4_PCLK_Pos)
#define MCU_SUB_WFI_XQSPI_DIV4_PCLK                         MCU_SUB_WFI_XQSPI_DIV4_PCLK_Msk

#define MCU_SUB_FORCE_SECU_DIV4_PCLK_Pos                    (25U)
#define MCU_SUB_FORCE_SECU_DIV4_PCLK_Len                    (1U)
#define MCU_SUB_FORCE_SECU_DIV4_PCLK_Msk                    (0x1UL << MCU_SUB_FORCE_SECU_DIV4_PCLK_Pos)
#define MCU_SUB_FORCE_SECU_DIV4_PCLK                        MCU_SUB_FORCE_SECU_DIV4_PCLK_Msk

#define MCU_SUB_WFI_SECU_DIV4_PCLK_Pos                      (24U)
#define MCU_SUB_WFI_SECU_DIV4_PCLK_Len                      (1U)
#define MCU_SUB_WFI_SECU_DIV4_PCLK_Msk                      (0x1UL << MCU_SUB_WFI_SECU_DIV4_PCLK_Pos)
#define MCU_SUB_WFI_SECU_DIV4_PCLK                          MCU_SUB_WFI_SECU_DIV4_PCLK_Msk

#define MCU_SUB_FORCE_I2S_HCLK_Pos                          (16U)
#define MCU_SUB_FORCE_I2S_HCLK_Len                          (1U)
#define MCU_SUB_FORCE_I2S_HCLK_Msk                          (0x1UL << MCU_SUB_FORCE_I2S_HCLK_Pos)
#define MCU_SUB_FORCE_I2S_HCLK                              MCU_SUB_FORCE_I2S_HCLK_Msk

#define MCU_SUB_FORCE_QSPI1_HCLK_Pos                        (15U)
#define MCU_SUB_FORCE_QSPI1_HCLK_Len                        (1U)
#define MCU_SUB_FORCE_QSPI1_HCLK_Msk                        (0x1UL << MCU_SUB_FORCE_QSPI1_HCLK_Pos)
#define MCU_SUB_FORCE_QSPI1_HCLK                            MCU_SUB_FORCE_QSPI1_HCLK_Msk

#define MCU_SUB_FORCE_QSPI0_HCLK_Pos                        (14U)
#define MCU_SUB_FORCE_QSPI0_HCLK_Len                        (1U)
#define MCU_SUB_FORCE_QSPI0_HCLK_Msk                        (0x1UL << MCU_SUB_FORCE_QSPI0_HCLK_Pos)
#define MCU_SUB_FORCE_QSPI0_HCLK                            MCU_SUB_FORCE_QSPI0_HCLK_Msk

#define MCU_SUB_FORCE_SPIS_HCLK_Pos                         (13U)
#define MCU_SUB_FORCE_SPIS_HCLK_Len                         (1U)
#define MCU_SUB_FORCE_SPIS_HCLK_Msk                         (0x1UL << MCU_SUB_FORCE_SPIS_HCLK_Pos)
#define MCU_SUB_FORCE_SPIS_HCLK                             MCU_SUB_FORCE_SPIS_HCLK_Msk

#define MCU_SUB_FORCE_SPIM_HCLK_Pos                         (12U)
#define MCU_SUB_FORCE_SPIM_HCLK_Len                         (1U)
#define MCU_SUB_FORCE_SPIM_HCLK_Msk                         (0x1UL << MCU_SUB_FORCE_SPIM_HCLK_Pos)
#define MCU_SUB_FORCE_SPIM_HCLK                             MCU_SUB_FORCE_SPIM_HCLK_Msk

#define MCU_SUB_FORCE_I2C1_HCLK_Pos                         (11U)
#define MCU_SUB_FORCE_I2C1_HCLK_Len                         (1U)
#define MCU_SUB_FORCE_I2C1_HCLK_Msk                         (0x1UL << MCU_SUB_FORCE_I2C1_HCLK_Pos)
#define MCU_SUB_FORCE_I2C1_HCLK                             MCU_SUB_FORCE_I2C1_HCLK_Msk

#define MCU_SUB_FORCE_I2C0_HCLK_Pos                         (10U)
#define MCU_SUB_FORCE_I2C0_HCLK_Len                         (1U)
#define MCU_SUB_FORCE_I2C0_HCLK_Msk                         (0x1UL << MCU_SUB_FORCE_I2C0_HCLK_Pos)
#define MCU_SUB_FORCE_I2C0_HCLK                             MCU_SUB_FORCE_I2C0_HCLK_Msk

#define MCU_SUB_FORCE_UART1_HCLK_Pos                        (9U)
#define MCU_SUB_FORCE_UART1_HCLK_Len                        (1U)
#define MCU_SUB_FORCE_UART1_HCLK_Msk                        (0x1UL << MCU_SUB_FORCE_UART1_HCLK_Pos)
#define MCU_SUB_FORCE_UART1_HCLK                            MCU_SUB_FORCE_UART1_HCLK_Msk

#define MCU_SUB_FORCE_UART0_HCLK_Pos                        (8U)
#define MCU_SUB_FORCE_UART0_HCLK_Len                        (1U)
#define MCU_SUB_FORCE_UART0_HCLK_Msk                        (0x1UL << MCU_SUB_FORCE_UART0_HCLK_Pos)
#define MCU_SUB_FORCE_UART0_HCLK                            MCU_SUB_FORCE_UART0_HCLK_Msk

#define MCU_SUB_I2S_LP_EN_Pos                               (2U)
#define MCU_SUB_I2S_LP_EN_Len                               (1U)
#define MCU_SUB_I2S_LP_EN_Msk                               (0x1UL << MCU_SUB_I2S_LP_EN_Pos)
#define MCU_SUB_I2S_LP_EN                                   MCU_SUB_I2S_LP_EN_Msk

#define MCU_SUB_UART_LP_PCLK_EN_Pos                         (1U)
#define MCU_SUB_UART_LP_PCLK_EN_Len                         (1U)
#define MCU_SUB_UART_LP_PCLK_EN_Msk                         (0x1UL << MCU_SUB_UART_LP_PCLK_EN_Pos)
#define MCU_SUB_UART_LP_PCLK_EN                             MCU_SUB_UART_LP_PCLK_EN_Msk

#define MCU_SUB_UART_LP_SCLK_EN_Pos                         (0U)
#define MCU_SUB_UART_LP_SCLK_EN_Len                         (1U)
#define MCU_SUB_UART_LP_SCLK_EN_Msk                         (0x1UL << MCU_SUB_UART_LP_SCLK_EN_Pos)
#define MCU_SUB_UART_LP_SCLK_EN                             MCU_SUB_UART_LP_SCLK_EN_Msk

/**********  Bit definition for MCU_SUB_REG_BLE_DSLEEP_CORR_EN register  ***********/
#define MCU_SUB_BLE_DSLEEP_CORR_EN_Pos                      (0U)
#define MCU_SUB_BLE_DSLEEP_CORR_EN_Len                      (2U)
#define MCU_SUB_BLE_DSLEEP_CORR_EN_Msk                      (0x3U << MCU_SUB_BLE_DSLEEP_CORR_EN_Pos)
#define MCU_SUB_BLE_DSLEEP_CORR_HW_EN                       (0x02 << MCU_SUB_BLE_DSLEEP_CORR_EN_Pos)
#define MCU_SUB_BLE_DSLEEP_CORR_SW_EN                       (0x01 << MCU_SUB_BLE_DSLEEP_CORR_EN_Pos)

/**********  Bit definition for MCU_SUB_REG_BLE_DSLEEP_HW_TIM_CORR register  ***********/
#define MCU_SUB_BLE_DSLEEP_HW_TIM_CORR_CLK_PERIOD_Pos       (12U)
#define MCU_SUB_BLE_DSLEEP_HW_TIM_CORR_CLK_PERIOD_Len       (18U)
#define MCU_SUB_BLE_DSLEEP_HW_TIM_CORR_CLK_PERIOD_Msk       (0x3FFFFU << MCU_SUB_BLE_DSLEEP_HW_TIM_CORR_CLK_PERIOD_Pos)
#define MCU_SUB_BLE_DSLEEP_HW_TIM_CORR_CLK_PERIOD           MCU_SUB_BLE_DSLEEP_HW_TIM_CORR_CLK_PERIOD_Msk

#define MCU_SUB_BLE_DSLEEP_HW_TIM_CORR_DELAY_Pos            (0U)
#define MCU_SUB_BLE_DSLEEP_HW_TIM_CORR_DELAY_Len            (9U)
#define MCU_SUB_BLE_DSLEEP_HW_TIM_CORR_DELAY_Msk            (0x1FFU << MCU_SUB_BLE_DSLEEP_HW_TIM_CORR_DELAY_Pos)
#define MCU_SUB_BLE_DSLEEP_HW_TIM_CORR_DELAY                MCU_SUB_BLE_DSLEEP_HW_TIM_CORR_DELAY_Msk



/* ================================================================================================================= */
/* ================                                        PKC                                      ================ */
/* ================================================================================================================= */

/*******************  Bit definition for PKC_CTRL register  *******************/
#define PKC_CTRL_EN_Pos                                     (0U)
#define PKC_CTRL_EN_Len                                     (1U)
#define PKC_CTRL_EN_Msk                                     (0x1U << PKC_CTRL_EN_Pos)
#define PKC_CTRL_EN                                         PKC_CTRL_EN_Msk

#define PKC_CTRL_START_Pos                                  (1U)
#define PKC_CTRL_START_Len                                  (1U)
#define PKC_CTRL_START_Msk                                  (0x1U << PKC_CTRL_START_Pos)
#define PKC_CTRL_START                                      PKC_CTRL_START_Msk

#define PKC_CTRL_SWCTRL_Pos                                 (4U)
#define PKC_CTRL_SWCTRL_Len                                 (1U)
#define PKC_CTRL_SWCTRL_Msk                                 (0x1U << PKC_CTRL_SWCTRL_Pos)
#define PKC_CTRL_SWCTRL                                     PKC_CTRL_SWCTRL_Msk

#define PKC_CTRL_SWRST_Pos                                  (8U)
#define PKC_CTRL_SWRST_Len                                  (1U)
#define PKC_CTRL_SWRST_Msk                                  (0x1U << PKC_CTRL_SWRST_Pos)
#define PKC_CTRL_SWRST                                      PKC_CTRL_SWRST_Msk

/*******************  Bit definition for PKC_CONFIG0 register  ****************/
#define PKC_CONFIG0_KPTR_Pos                                (0U)
#define PKC_CONFIG0_KPTR_Len                                (9U)
#define PKC_CONFIG0_KPTR_Msk                                (0x000001FFU)
#define PKC_CONFIG0_KPTR                                    PKC_CONFIG0_KPTR_Msk

#define PKC_CONFIG0_RPTR_Pos                                (16U)
#define PKC_CONFIG0_RPTR_Len                                (9U)
#define PKC_CONFIG0_RPTR_Msk                                (0x01FF0000U)
#define PKC_CONFIG0_RPTR                                    PKC_CONFIG0_RPTR_Msk

/*******************  Bit definition for PKC_CONFIG1 register  ****************/
#define PKC_CONFIG1_PPTR_Pos                                (0U)
#define PKC_CONFIG1_PPTR_Len                                (9U)
#define PKC_CONFIG1_PPTR_Msk                                (0x000001FFU)
#define PKC_CONFIG1_PPTR                                    PKC_CONFIG1_PPTR_Msk

#define PKC_CONFIG1_RSQPTR_Pos                              (16U)
#define PKC_CONFIG1_RSQPTR_Len                              (9U)
#define PKC_CONFIG1_RSQPTR_Msk                              (0x01FF0000U)
#define PKC_CONFIG1_RSQPTR                                  PKC_CONFIG1_RSQPTR_Msk

/*******************  Bit definition for PKC_CONFIG2 register  ****************/
#define PKC_CONFIG2_GXPTR_Pos                               (0U)
#define PKC_CONFIG2_GXPTR_Len                               (9U)
#define PKC_CONFIG2_GXPTR_Msk                               (0x000001FFU)
#define PKC_CONFIG2_GXPTR                                   PKC_CONFIG2_GXPTR_Msk

#define PKC_CONFIG2_GYPTR_Pos                               (16U)
#define PKC_CONFIG2_GYPTR_Len                               (9U)
#define PKC_CONFIG2_GYPTR_Msk                               (0x01FF0000U)
#define PKC_CONFIG2_GYPTR                                   PKC_CONFIG2_GYPTR_Msk

/*******************  Bit definition for PKC_CONFIG3 register  ****************/
#define PKC_CONFIG3_GZPTR_Pos                               (0U)
#define PKC_CONFIG3_GZPTR_Len                               (9U)
#define PKC_CONFIG3_GZPTR_Msk                               (0x000001FFU)
#define PKC_CONFIG3_GZPTR                                   PKC_CONFIG3_GZPTR_Msk

#define PKC_CONFIG3_R0XPTR_Pos                              (16U)
#define PKC_CONFIG3_R0XPTR_Len                              (9U)
#define PKC_CONFIG3_R0XPTR_Msk                              (0x01FF0000U)
#define PKC_CONFIG3_R0XPTR                                  PKC_CONFIG3_R0XPTR_Msk

/*******************  Bit definition for PKC_CONFIG4 register  ****************/
#define PKC_CONFIG4_R0YPTR_Pos                              (0U)
#define PKC_CONFIG4_R0YPTR_Len                              (9U)
#define PKC_CONFIG4_R0YPTR_Msk                              (0x000001FFU)
#define PKC_CONFIG4_R0YPTR                                  PKC_CONFIG4_R0YPTR_Msk

#define PKC_CONFIG4_R0ZPTR_Pos                              (16U)
#define PKC_CONFIG4_R0ZPTR_Len                              (9U)
#define PKC_CONFIG4_R0ZPTR_Msk                              (0x01FF0000U)
#define PKC_CONFIG4_R0ZPTR                                  PKC_CONFIG4_R0ZPTR_Msk

/*******************  Bit definition for PKC_CONFIG5 register  ****************/
#define PKC_CONFIG5_R1XPTR_Pos                              (0U)
#define PKC_CONFIG5_R1XPTR_Len                              (9U)
#define PKC_CONFIG5_R1XPTR_Msk                              (0x000001FFU)
#define PKC_CONFIG5_R1XPTR                                  PKC_CONFIG5_R1XPTR_Msk

#define PKC_CONFIG5_R1YPTR_Pos                              (16U)
#define PKC_CONFIG5_R1YPTR_Len                              (9U)
#define PKC_CONFIG5_R1YPTR_Msk                              (0x01FF0000U)
#define PKC_CONFIG5_R1YPTR                                  PKC_CONFIG5_R1YPTR_Msk

/*******************  Bit definition for PKC_CONFIG6 register  ****************/
#define PKC_CONFIG6_R1ZPTR_Pos                              (0U)
#define PKC_CONFIG6_R1ZPTR_Len                              (9U)
#define PKC_CONFIG6_R1ZPTR_Msk                              (0x000001FFU)
#define PKC_CONFIG6_R1ZPTR                                  PKC_CONFIG6_R1ZPTR_Msk

#define PKC_CONFIG6_TMP1PTR_Pos                             (16U)
#define PKC_CONFIG6_TMP1PTR_Len                             (9U)
#define PKC_CONFIG6_TMP1PTR_Msk                             (0x01FF0000U)
#define PKC_CONFIG6_TMP1PTR                                 PKC_CONFIG6_TMP1PTR_Msk

/*******************  Bit definition for PKC_CONFIG7 register  ****************/
#define PKC_CONFIG7_TMP2PTR_Pos                             (0U)
#define PKC_CONFIG7_TMP2PTR_Len                             (9U)
#define PKC_CONFIG7_TMP2PTR_Msk                             (0x000001FFU)
#define PKC_CONFIG7_TMP2PTR                                 PKC_CONFIG7_TMP2PTR_Msk

#define PKC_CONFIG7_TMP3PTR_Pos                             (16U)
#define PKC_CONFIG7_TMP3PTR_Len                             (9U)
#define PKC_CONFIG7_TMP3PTR_Msk                             (0x01FF0000U)
#define PKC_CONFIG7_TMP3PTR                                 PKC_CONFIG7_TMP3PTR_Msk

/*******************  Bit definition for PKC_CONFIG8 register  ****************/
#define PKC_CONFIG8_TMP4PTR_Pos                             (0U)
#define PKC_CONFIG8_TMP4PTR_Len                             (9U)
#define PKC_CONFIG8_TMP4PTR_Msk                             (0x000001FFU)
#define PKC_CONFIG8_TMP4PTR                                 PKC_CONFIG8_TMP4PTR_Msk

#define PKC_CONFIG8_TMP5PTR_Pos                             (16U)
#define PKC_CONFIG8_TMP5PTR_Len                             (9U)
#define PKC_CONFIG8_TMP5PTR_Msk                             (0x01FF0000U)
#define PKC_CONFIG8_TMP5PTR                                 PKC_CONFIG8_TMP5PTR_Msk

/*******************  Bit definition for PKC_CONFIG9 register  ****************/
#define PKC_CONFIG9_TMP6PTR_Pos                             (0U)
#define PKC_CONFIG9_TMP6PTR_Len                             (9U)
#define PKC_CONFIG9_TMP6PTR_Msk                             (0x000001FFU)
#define PKC_CONFIG9_TMP6PTR                                 PKC_CONFIG9_TMP6PTR_Msk

#define PKC_CONFIG9_CONST1PTR_Pos                           (16U)
#define PKC_CONFIG9_CONST1PTR_Len                           (9U)
#define PKC_CONFIG9_CONST1PTR_Msk                           (0x01FF0000U)
#define PKC_CONFIG9_CONST1PTR                               PKC_CONFIG9_CONST1PTR_Msk

/*******************  Bit definition for PKC_CONFIG10 register  ****************/
#define PKC_CONFIG10_X1PTR_Pos                              (0U)
#define PKC_CONFIG10_X1PTR_Len                              (9U)
#define PKC_CONFIG10_X1PTR_Msk                              (0x000001FFU)
#define PKC_CONFIG10_X1PTR                                  PKC_CONFIG10_X1PTR_Msk

#define PKC_CONFIG10_X2PTR_Pos                              (16U)
#define PKC_CONFIG10_X2PTR_Len                              (9U)
#define PKC_CONFIG10_X2PTR_Msk                              (0x01FF0000U)
#define PKC_CONFIG10_X2PTR                                  PKC_CONFIG10_X2PTR_Msk

/*******************  Bit definition for PKC_CONFIG11 register  ****************/
#define PKC_CONFIG11_MITMPPTR_Pos                           (0U)
#define PKC_CONFIG11_MITMPPTR_Len                           (9U)
#define PKC_CONFIG11_MITMPPTR_Msk                           (0x000001FFU)
#define PKC_CONFIG11_MITMPPTR                               PKC_CONFIG11_MITMPPTR_Msk

#define PKC_CONFIG11_TMPKPTR_Pos                            (16U)
#define PKC_CONFIG11_TMPKPTR_Len                            (9U)
#define PKC_CONFIG11_TMPKPTR_Msk                            (0x01FF0000U)
#define PKC_CONFIG11_TMPKPTR                                PKC_CONFIG11_TMPKPTR_Msk

/*******************  Bit definition for PKC_CONFIG12 register  ****************/
#define PKC_CONFIG12_APTR_Pos                               (0U)
#define PKC_CONFIG12_APTR_Len                               (9U)
#define PKC_CONFIG12_APTR_Msk                               (0x000001FFU)
#define PKC_CONFIG12_APTR                                   PKC_CONFIG12_APTR_Msk

#define PKC_CONFIG12_BPTR_Pos                               (16U)
#define PKC_CONFIG12_BPTR_Len                               (9U)
#define PKC_CONFIG12_BPTR_Msk                               (0x01FF0000U)
#define PKC_CONFIG12_BPTR                                   PKC_CONFIG12_BPTR_Msk

/*******************  Bit definition for PKC_CONFIG13 register  ****************/
#define PKC_CONFIG13_CONSTQ_Pos                             (0U)
#define PKC_CONFIG13_CONSTQ_Len                             (32U)
#define PKC_CONFIG13_CONSTQ_Msk                             (0xFFFFFFFFU)
#define PKC_CONFIG13_CONSTQ                                 PKC_CONFIG13_CONSTQ_Msk

/*******************  Bit definition for PKC_SW_CTRL register  *****************/
#define PKC_SW_CTRL_OPSTART_Pos                             (0U)
#define PKC_SW_CTRL_OPSTART_Len                             (1U)
#define PKC_SW_CTRL_OPSTART_Msk                             (0x1U << PKC_SW_CTRL_OPSTART_Pos)
#define PKC_SW_CTRL_OPSTART                                 PKC_SW_CTRL_OPSTART_Msk

#define PKC_SW_CTRL_OPMODE_Pos                              (4U)
#define PKC_SW_CTRL_OPMODE_Len                              (3U)
#define PKC_SW_CTRL_OPMODE_Msk                              (0x7U << PKC_SW_CTRL_OPMODE_Pos)
#define PKC_SW_CTRL_OPMODE                                  PKC_SW_CTRL_OPMODE_Msk

#define PKC_SW_CTRL_STARTDM_Pos                             (8U)
#define PKC_SW_CTRL_STARTDM_Len                             (1U)
#define PKC_SW_CTRL_STARTDM_Msk                             (0x1U << PKC_SW_CTRL_STARTDM_Pos)
#define PKC_SW_CTRL_STARTDM                                 PKC_SW_CTRL_STARTDM_Msk

#define PKC_SW_CTRL_RANDEN_Pos                              (9U)
#define PKC_SW_CTRL_RANDEN_Len                              (1U)
#define PKC_SW_CTRL_RANDEN_Msk                              (0x1U << PKC_SW_CTRL_RANDEN_Pos)
#define PKC_SW_CTRL_RANDEN                                  PKC_SW_CTRL_RANDEN_Msk

/*******************  Bit definition for PKC_SW_CONFIG0  register  ************/
#define PKC_SW_CONFIG0_MMAPTR_Pos                           (0U)
#define PKC_SW_CONFIG0_MMAPTR_Len                           (9U)
#define PKC_SW_CONFIG0_MMAPTR_Msk                           (0x000001FFU)
#define PKC_SW_CONFIG0_MMAPTR                               PKC_SW_CONFIG0_MMAPTR_Msk

#define PKC_SW_CONFIG0_MMBPTR_Pos                           (16U)
#define PKC_SW_CONFIG0_MMBPTR_Len                           (9U)
#define PKC_SW_CONFIG0_MMBPTR_Msk                           (0x01FF0000U)
#define PKC_SW_CONFIG0_MMBPTR                               PKC_SW_CONFIG0_MMBPTR_Msk

/*******************  Bit definition for PKC_SW_CONFIG1  register  ************/
#define PKC_SW_CONFIG1_MMPPTR_Pos                           (0U)
#define PKC_SW_CONFIG1_MMPPTR_Len                           (9U)
#define PKC_SW_CONFIG1_MMPPTR_Msk                           (0x000001FFU)
#define PKC_SW_CONFIG1_MMPPTR                               PKC_SW_CONFIG1_MMPPTR_Msk

#define PKC_SW_CONFIG1_MMCPTR_Pos                           (16U)
#define PKC_SW_CONFIG1_MMCPTR_Len                           (9U)
#define PKC_SW_CONFIG1_MMCPTR_Msk                           (0x01FF0000U)
#define PKC_SW_CONFIG1_MMCPTR                               PKC_SW_CONFIG1_MMCPTR_Msk

/*******************  Bit definition for PKC_SW_CONFIG2  register  ************/
#define PKC_SW_CONFIG2_MASAPTR_Pos                          (0U)
#define PKC_SW_CONFIG2_MASAPTR_Len                          (9U)
#define PKC_SW_CONFIG2_MASAPTR_Msk                          (0x000001FFU)
#define PKC_SW_CONFIG2_MASAPTR                              PKC_SW_CONFIG2_MASAPTR_Msk

#define PKC_SW_CONFIG2_MASBPTR_Pos                          (16U)
#define PKC_SW_CONFIG2_MASBPTR_Len                          (9U)
#define PKC_SW_CONFIG2_MASBPTR_Msk                          (0x01FF0000U)
#define PKC_SW_CONFIG2_MASBPTR                              PKC_SW_CONFIG2_MASBPTR_Msk

/*******************  Bit definition for PKC_SW_CONFIG3  register  ************/
#define PKC_SW_CONFIG3_MASPPTR_Pos                          (0U)
#define PKC_SW_CONFIG3_MASPPTR_Len                          (9U)
#define PKC_SW_CONFIG3_MASPPTR_Msk                          (0x000001FFU)
#define PKC_SW_CONFIG3_MASPPTR                              PKC_SW_CONFIG3_MASPPTR_Msk

#define PKC_SW_CONFIG3_MASCPTR_Pos                          (16U)
#define PKC_SW_CONFIG3_MASCPTR_Len                          (9U)
#define PKC_SW_CONFIG3_MASCPTR_Msk                          (0x01FF0000U)
#define PKC_SW_CONFIG3_MASCPTR                              PKC_SW_CONFIG3_MASCPTR_Msk

/*******************  Bit definition for PKC_SW_CONFIG4  register  ************/
#define PKC_SW_CONFIG4_MIUPTR_Pos                           (0U)
#define PKC_SW_CONFIG4_MIUPTR_Len                           (9U)
#define PKC_SW_CONFIG4_MIUPTR_Msk                           (0x000001FFU)
#define PKC_SW_CONFIG4_MIUPTR                               PKC_SW_CONFIG4_MIUPTR_Msk

#define PKC_SW_CONFIG4_MIVPTR_Pos                           (16U)
#define PKC_SW_CONFIG4_MIVPTR_Len                           (9U)
#define PKC_SW_CONFIG4_MIVPTR_Msk                           (0x01FF0000U)
#define PKC_SW_CONFIG4_MIVPTR                               PKC_SW_CONFIG4_MIVPTR_Msk

/*******************  Bit definition for PKC_SW_CONFIG5  register  ************/
#define PKC_SW_CONFIG5_MIX1PTR_Pos                          (0U)
#define PKC_SW_CONFIG5_MIX1PTR_Len                          (9U)
#define PKC_SW_CONFIG5_MIX1PTR_Msk                          (0x000001FFU)
#define PKC_SW_CONFIG5_MIX1PTR                              PKC_SW_CONFIG5_MIX1PTR_Msk

#define PKC_SW_CONFIG5_MIX2PTR_Pos                          (16U)
#define PKC_SW_CONFIG5_MIX2PTR_Len                          (9U)
#define PKC_SW_CONFIG5_MIX2PTR_Msk                          (0x01FF0000U)
#define PKC_SW_CONFIG5_MIX2PTR                              PKC_SW_CONFIG5_MIX2PTR_Msk

/*******************  Bit definition for PKC_SW_CONFIG6  register  ************/
#define PKC_SW_CONFIG6_MITMPPTR_Pos                         (0U)
#define PKC_SW_CONFIG6_MITMPPTR_Len                         (9U)
#define PKC_SW_CONFIG6_MITMPPTR_Msk                         (0x000001FFU)
#define PKC_SW_CONFIG6_MITMPPTR                             PKC_SW_CONFIG6_MITMPPTR_Msk

/*******************  Bit definition for PKC_SW_CONFIG7  register  ************/
#define PKC_SW_CONFIG7_WORDLEN_Pos                          (0U)
#define PKC_SW_CONFIG7_WORDLEN_Len                          (6U)
#define PKC_SW_CONFIG7_WORDLEN_Msk                          (0x0000003FU)
#define PKC_SW_CONFIG7_WORDLEN                              PKC_SW_CONFIG7_WORDLEN_Msk

/*******************  Bit definition for PKC_SW_CONFIG8  register  ************/
#define PKC_SW_CONFIG8_MIKOUT_Pos                           (0U)
#define PKC_SW_CONFIG8_MIKOUT_Len                           (13U)
#define PKC_SW_CONFIG8_MIKOUT_Msk                           (0x00001FFFU)
#define PKC_SW_CONFIG8_MIKOUT                               PKC_SW_CONFIG8_MIKOUT_Msk

/*******************  Bit definition for PKC_SW_CONFIG9  register  ************/
#define PKC_SW_CONFIG9_DMRNGSEED_Pos                        (0U)
#define PKC_SW_CONFIG9_DMRNGSEED_Len                        (32U)
#define PKC_SW_CONFIG9_DMRNGSEED_Msk                        (0xFFFFFFFFU)
#define PKC_SW_CONFIG9_DMRNGSEED                            PKC_SW_CONFIG9_DMRNGSEED_Msk

/*******************  Bit definition for PKC_SW_CONFIG10  register  ************/
#define PKC_SW_CONFIG10_BMAPTR_Pos                          (0U)
#define PKC_SW_CONFIG10_BMAPTR_Len                          (9U)
#define PKC_SW_CONFIG10_BMAPTR_Msk                          (0x000001FFU)
#define PKC_SW_CONFIG10_BMAPTR                              PKC_SW_CONFIG10_BMAPTR_Msk

#define PKC_SW_CONFIG10_BMBPTR_Pos                          (16U)
#define PKC_SW_CONFIG10_BMBPTR_Len                          (9U)
#define PKC_SW_CONFIG10_BMBPTR_Msk                          (0x01FF0000U)
#define PKC_SW_CONFIG10_BMBPTR                              PKC_SW_CONFIG10_BMBPTR_Msk

/*******************  Bit definition for PKC_SW_CONFIG11  register  ************/
#define PKC_SW_CONFIG11_BMCPTR_Pos                          (0U)
#define PKC_SW_CONFIG11_BMCPTR_Len                          (9U)
#define PKC_SW_CONFIG11_BMCPTR_Msk                          (0x000001FFU)
#define PKC_SW_CONFIG11_BMCPTR                              PKC_SW_CONFIG11_BMCPTR_Msk

#define PKC_SW_CONFIG11_BAAPTR_Pos                          (16U)
#define PKC_SW_CONFIG11_BAAPTR_Len                          (9U)
#define PKC_SW_CONFIG11_BAAPTR_Msk                          (0x01FF0000U)
#define PKC_SW_CONFIG11_BAAPTR                              PKC_SW_CONFIG11_BAAPTR_Msk

/*******************  Bit definition for PKC_SW_CONFIG12  register  ************/
#define PKC_SW_CONFIG12_BABPTR_Pos                          (0U)
#define PKC_SW_CONFIG12_BABPTR_Len                          (9U)
#define PKC_SW_CONFIG12_BABPTR_Msk                          (0x000001FFU)
#define PKC_SW_CONFIG12_BABPTR                              PKC_SW_CONFIG12_BABPTR_Msk

#define PKC_SW_CONFIG12_BACPTR_Pos                          (16U)
#define PKC_SW_CONFIG12_BACPTR_Len                          (9U)
#define PKC_SW_CONFIG12_BACPTR_Msk                          (0x01FF0000U)
#define PKC_SW_CONFIG12_BACPTR                              PKC_SW_CONFIG12_BACPTR_Msk

/*******************  Bit definition for PKC_SW_CONFIG13  register  ************/
#define PKC_SW_CONFIG13_RANDSEED_Pos                        (0U)
#define PKC_SW_CONFIG13_RANDSEED_Len                        (32U)
#define PKC_SW_CONFIG13_RANDSEED_Msk                        (0xFFFFFFFFU)
#define PKC_SW_CONFIG13_RANDSEED                            PKC_SW_CONFIG13_RANDSEED_Msk

/*******************  Bit definition for PKC_INT_STATUS  register  ************/
#define PKC_INTSTAT_DONE_Pos                                (0U)
#define PKC_INTSTAT_DONE_Len                                (1U)
#define PKC_INTSTAT_DONE_Msk                                (0x1U << PKC_INTSTAT_DONE_Pos)
#define PKC_INTSTAT_DONE                                    PKC_INTSTAT_DONE_Msk

#define PKC_INTSTAT_ERR_Pos                                 (1U)
#define PKC_INTSTAT_ERR_Len                                 (1U)
#define PKC_INTSTAT_ERR_Msk                                 (0x1U << PKC_INTSTAT_ERR_Pos)
#define PKC_INTSTAT_ERR                                     PKC_INTSTAT_ERR_Msk

#define PKC_INTSTAT_BAOVF_Pos                               (2U)
#define PKC_INTSTAT_BAOVF_Len                               (1U)
#define PKC_INTSTAT_BAOVF_Msk                               (0x1U << PKC_INTSTAT_BAOVF_Pos)
#define PKC_INTSTAT_BAOVF                                   PKC_INTSTAT_BAOVF_Msk

/*******************  Bit definition for PKC_INT_ENABLE  register  ************/
#define PKC_INTEN_DONE_Pos                                  (0U)
#define PKC_INTEN_DONE_Len                                  (1U)
#define PKC_INTEN_DONE_Msk                                  (0x1U << PKC_INTEN_DONE_Pos)
#define PKC_INTEN_DONE                                      PKC_INTEN_DONE_Msk

#define PKC_INTEN_ERR_Pos                                   (1U)
#define PKC_INTEN_ERR_Len                                   (1U)
#define PKC_INTEN_ERR_Msk                                   (0x1U << PKC_INTEN_ERR_Pos)
#define PKC_INTEN_ERR                                       PKC_INTEN_ERR_Msk

#define PKC_INTEN_BAOVF_Pos                                 (2U)
#define PKC_INTEN_BAOVF_Len                                 (1U)
#define PKC_INTEN_BAOVF_Msk                                 (0x1U << PKC_INTEN_BAOVF_Pos)
#define PKC_INTEN_BAOVF                                     PKC_INTEN_BAOVF_Msk

/*******************  Bit definition for PKC_WORK_STATUS  register  ***********/
#define PKC_WORKSTAT_BUSY_Pos                               (0U)
#define PKC_WORKSTAT_BUSY_Len                               (1U)
#define PKC_WORKSTAT_BUSY_Msk                               (0x1U << PKC_WORKSTAT_BUSY_Pos)
#define PKC_WORKSTAT_BUSY                                   PKC_WORKSTAT_BUSY_Msk

/*******************  Bit definition for PKC_DUMMY0  register  ****************/
#define PKC_DUMMY0_DUMMY0_Pos                               (0U)
#define PKC_DUMMY0_DUMMY0_Len                               (32U)
#define PKC_DUMMY0_DUMMY0_Msk                               (0xFFFFFFFFU)
#define PKC_DUMMY0_DUMMY0                                   PKC_DUMMY0_DUMMY0_Msk

/*******************  Bit definition for PKC_DUMMY1  register  ****************/
#define PKC_DUMMY1_DUMMY1_Pos                               (0U)
#define PKC_DUMMY1_DUMMY1_Len                               (32U)
#define PKC_DUMMY1_DUMMY1_Msk                               (0xFFFFFFFFU)
#define PKC_DUMMY1_DUMMY1                                   PKC_DUMMY1_DUMMY1_Msk

/*******************  Bit definition for PKC_DUMMY2  register  ****************/
#define PKC_DUMMY2_DUMMY2_Pos                               (0U)
#define PKC_DUMMY2_DUMMY2_Len                               (32U)
#define PKC_DUMMY2_DUMMY2_Msk                               (0xFFFFFFFFU)
#define PKC_DUMMY2_DUMMY2                                   PKC_DUMMY2_DUMMY2_Msk


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

#define PWM_UPDATE_SSPAUSE_Pos                              (15U)
#define PWM_UPDATE_SSPAUSE_Len                              (1U)
#define PWM_UPDATE_SSPAUSE_Msk                              (0x1U << PWM_UPDATE_SSPAUSE_Pos)
#define PWM_UPDATE_SSPAUSE                                  PWM_UPDATE_SSPAUSE_Msk

#define PWM_UPDATE_SSBRPRD_Pos                              (16U)
#define PWM_UPDATE_SSBRPRD_Len                              (1U)
#define PWM_UPDATE_SSBRPRD_Msk                              (0x1U << PWM_UPDATE_SSBRPRD_Pos)
#define PWM_UPDATE_SSBRPRD                                  PWM_UPDATE_SSBRPRD_Msk

#define PWM_UPDATE_SSHOLD_Pos                               (17U)
#define PWM_UPDATE_SSHOLD_Len                               (1U)
#define PWM_UPDATE_SSHOLD_Msk                               (0x1U << PWM_UPDATE_SSHOLD_Pos)
#define PWM_UPDATE_SSHOLD                                   PWM_UPDATE_SSHOLD_Msk

#define PWM_UPDATE_SSAQCTRL_Pos                             (18U)
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
/* ================                                        SSI                                      ================ */
/* ================================================================================================================= */
/*******************  Bit definition for SSI_CTRL0 register  ******************/
#define SSI_CTRL0_SSTEN_Pos                                 (24U)
#define SSI_CTRL0_SSTEN_Len                                 (1U)
#define SSI_CTRL0_SSTEN_Msk                                 (0x1U << SSI_CTRL0_SSTEN_Pos)
#define SSI_CTRL0_SSTEN                                     SSI_CTRL0_SSTEN_Msk

#define SSI_CTRL0_SPIFRF_Pos                                (21U)
#define SSI_CTRL0_SPIFRF_Len                                (2U)
#define SSI_CTRL0_SPIFRF_Msk                                (0x3U << SSI_CTRL0_SPIFRF_Pos)
#define SSI_CTRL0_SPIFRF                                    SSI_CTRL0_SPIFRF_Msk

#define SSI_CTRL0_DFS32_Pos                                 (16U)
#define SSI_CTRL0_DFS32_Len                                 (5U)
#define SSI_CTRL0_DFS32_Msk                                 (0x1FU << SSI_CTRL0_DFS32_Pos)
#define SSI_CTRL0_DFS32                                     SSI_CTRL0_DFS32_Msk

#define SSI_CTRL0_CFS_Pos                                   (12U)
#define SSI_CTRL0_CFS_Len                                   (4U)
#define SSI_CTRL0_CFS_Msk                                   (0xFU << SSI_CTRL0_CFS_Pos)
#define SSI_CTRL0_CFS                                       SSI_CTRL0_CFS_Msk

#define SSI_CTRL0_SRL_Pos                                   (11U)
#define SSI_CTRL0_SRL_Len                                   (1U)
#define SSI_CTRL0_SRL_Msk                                   (0x1U << SSI_CTRL0_SRL_Pos)
#define SSI_CTRL0_SRL                                       SSI_CTRL0_SRL_Msk

#define SSI_CTRL0_SLVOE_Pos                                 (10U)
#define SSI_CTRL0_SLVOE_Len                                 (1U)
#define SSI_CTRL0_SLVOE_Msk                                 (0x1U << SSI_CTRL0_SLVOE_Pos)
#define SSI_CTRL0_SLVOE                                     SSI_CTRL0_SLVOE_Msk

#define SSI_CTRL0_TMOD_Pos                                  (8U)
#define SSI_CTRL0_TMOD_Len                                  (2U)
#define SSI_CTRL0_TMOD_Msk                                  (0x3U << SSI_CTRL0_TMOD_Pos)
#define SSI_CTRL0_TMOD                                      SSI_CTRL0_TMOD_Msk

#define SSI_CTRL0_SCPOL_Pos                                 (7U)
#define SSI_CTRL0_SCPOL_Len                                 (1U)
#define SSI_CTRL0_SCPOL_Msk                                 (0x1U << SSI_CTRL0_SCPOL_Pos)
#define SSI_CTRL0_SCPOL                                     SSI_CTRL0_SCPOL_Msk

#define SSI_CTRL0_SCPHA_Pos                                 (6U)
#define SSI_CTRL0_SCPHA_Len                                 (1U)
#define SSI_CTRL0_SCPHA_Msk                                 (0x1U << SSI_CTRL0_SCPHA_Pos)
#define SSI_CTRL0_SCPHA                                     SSI_CTRL0_SCPHA_Msk

#define SSI_CTRL0_FRF_Pos                                   (4U)
#define SSI_CTRL0_FRF_Len                                   (2U)
#define SSI_CTRL0_FRF_Msk                                   (0x3U << SSI_CTRL0_FRF_Pos)
#define SSI_CTRL0_FRF                                       SSI_CTRL0_FRF_Msk

#define SSI_CTRL1_NDF_Pos                                   (0U)
#define SSI_CTRL1_NDF_Len                                   (16U)
#define SSI_CTRL1_NDF_Msk                                   (0xFFFFU << SSI_CTRL1_NDF_Pos)
#define SSI_CTRL1_NDF                                       SSI_CTRL1_NDF_Msk

/*******************  Bit definition for SSI_SSIEN register  ******************/
#define SSI_SSIEN_EN_Pos                                    (0U)
#define SSI_SSIEN_EN_Len                                    (1U)
#define SSI_SSIEN_EN_Msk                                    (0x1U << SSI_SSIEN_EN_Pos)
#define SSI_SSIEN_EN                                        SSI_SSIEN_EN_Msk

/*******************  Bit definition for SSI_MWC register  ********************/
#define SSI_MWC_MHS_Pos                                     (2U)
#define SSI_MWC_MHS_Len                                     (1U)
#define SSI_MWC_MHS_Msk                                     (0x1U << SSI_MWC_MHS_Pos)
#define SSI_MWC_MHS                                         SSI_MWC_MHS_Msk

#define SSI_MWC_MDD_Pos                                     (1U)
#define SSI_MWC_MDD_Len                                     (1U)
#define SSI_MWC_MDD_Msk                                     (0x1U << SSI_MWC_MDD_Pos)
#define SSI_MWC_MDD                                         SSI_MWC_MDD_Msk

#define SSI_MWC_MWMOD_Pos                                   (0U)
#define SSI_MWC_MWMOD_Len                                   (1U)
#define SSI_MWC_MWMOD_Msk                                   (0x1U << SSI_MWC_MWMOD_Pos)
#define SSI_MWC_MWMOD                                       SSI_MWC_MWMOD_Msk

/*******************  Bit definition for SSI_SE register  *********************/
#define SSI_SE_SLAVE1_Pos                                   (1U)
#define SSI_SE_SLAVE1_Len                                   (1U)
#define SSI_SE_SLAVE1_Msk                                   (0x1U << SSI_SE_SLAVE1_Pos)
#define SSI_SE_SLAVE1                                       SSI_SE_SLAVE1_Msk

#define SSI_SE_SLAVE0_Pos                                   (0U)
#define SSI_SE_SLAVE0_Len                                   (1U)
#define SSI_SE_SLAVE0_Msk                                   (0x1U << SSI_SE_SLAVE0_Pos)
#define SSI_SE_SLAVE0                                       SSI_SE_SLAVE0_Msk

/*******************  Bit definition for SSI_BAUD register  *******************/
#define SSI_BAUD_SCKDIV_Pos                                 (0U)
#define SSI_BAUD_SCKDIV_Len                                 (16U)
#define SSI_BAUD_SCKDIV_Msk                                 (0xFFFFUL << SSI_BAUD_SCKDIV_Pos)
#define SSI_BAUD_SCKDIV                                     SSI_BAUD_SCKDIV_Msk

/*******************  Bit definition for SSI_TXFTL register  ******************/
#define SSI_TXFTL_TFT_Pos                                   (0U)
#define SSI_TXFTL_TFT_Len                                   (3U)
#define SSI_TXFTL_TFT_Msk                                   (0x7U << SSI_TXFTL_TFT_Pos)
#define SSI_TXFTL_TFT                                       SSI_TXFTL_TFT_Msk

/*******************  Bit definition for SSI_RXFTL register  ******************/
#define SSI_RXFTL_RFT_Pos                                   (0U)
#define SSI_RXFTL_RFT_Len                                   (3U)
#define SSI_RXFTL_RFT_Msk                                   (0x7U << SSI_RXFTL_RFT_Pos)
#define SSI_RXFTL_RFT                                       SSI_RXFTL_RFT_Msk

/*******************  Bit definition for SSI_TXFL register  *******************/
#define SSI_TXFL_TXTFL_Pos                                  (0U)
#define SSI_TXFL_TXTFL_Len                                  (4U)
#define SSI_TXFL_TXTFL_Msk                                  (0xFU << SSI_TXFL_TXTFL_Pos)
#define SSI_TXFL_TXTFL                                      SSI_TXFL_TXTFL_Msk

/*******************  Bit definition for SSI_RXFL register  *******************/
#define SSI_RXFL_RXTFL_Pos                                  (0U)
#define SSI_RXFL_RXTFL_Len                                  (4U)
#define SSI_RXFL_RXTFL_Msk                                  (0xFU << SSI_RXFL_RXTFL_Pos)
#define SSI_RXFL_RXTFL                                      SSI_RXFL_RXTFL_Msk

/*******************  Bit definition for SSI_STAT register  *******************/
#define SSI_STAT_DCOL_Pos                                   (6U)
#define SSI_STAT_DCOL_Len                                   (1U)
#define SSI_STAT_DCOL_Msk                                   (0x1U << SSI_STAT_DCOL_Pos)
#define SSI_STAT_DCOL                                       SSI_STAT_DCOL_Msk

#define SSI_STAT_TXE_Pos                                    (5U)
#define SSI_STAT_TXE_Len                                    (1U)
#define SSI_STAT_TXE_Msk                                    (0x1U << SSI_STAT_TXE_Pos)
#define SSI_STAT_TXE                                        SSI_STAT_TXE_Msk

#define SSI_STAT_RFF_Pos                                    (4U)
#define SSI_STAT_RFF_Len                                    (1U)
#define SSI_STAT_RFF_Msk                                    (0x1U << SSI_STAT_RFF_Pos)
#define SSI_STAT_RFF                                        SSI_STAT_RFF_Msk

#define SSI_STAT_RFNE_Pos                                   (3U)
#define SSI_STAT_RFNE_Len                                   (1U)
#define SSI_STAT_RFNE_Msk                                   (0x1U << SSI_STAT_RFNE_Pos)
#define SSI_STAT_RFNE                                       SSI_STAT_RFNE_Msk

#define SSI_STAT_TFE_Pos                                    (2U)
#define SSI_STAT_TFE_Len                                    (1U)
#define SSI_STAT_TFE_Msk                                    (0x1U << SSI_STAT_TFE_Pos)
#define SSI_STAT_TFE                                        SSI_STAT_TFE_Msk

#define SSI_STAT_TFNF_Pos                                   (1U)
#define SSI_STAT_TFNF_Len                                   (1U)
#define SSI_STAT_TFNF_Msk                                   (0x1U << SSI_STAT_TFNF_Pos)
#define SSI_STAT_TFNF                                       SSI_STAT_TFNF_Msk

#define SSI_STAT_BUSY_Pos                                   (0U)
#define SSI_STAT_BUSY_Len                                   (1U)
#define SSI_STAT_BUSY_Msk                                   (0x1U << SSI_STAT_BUSY_Pos)
#define SSI_STAT_BUSY                                       SSI_STAT_BUSY_Msk

/*******************  Bit definition for SSI_INTMASK register  ****************/
#define SSI_INTMASK_MSTIM_Pos                               (5U)
#define SSI_INTMASK_MSTIM_Len                               (1U)
#define SSI_INTMASK_MSTIM_Msk                               (0x1U << SSI_INTMASK_MSTIM_Pos)
#define SSI_INTMASK_MSTIM                                   SSI_INTMASK_MSTIM_Msk

#define SSI_INTMASK_RXFIM_Pos                               (4U)
#define SSI_INTMASK_RXFIM_Len                               (1U)
#define SSI_INTMASK_RXFIM_Msk                               (0x1U << SSI_INTMASK_RXFIM_Pos)
#define SSI_INTMASK_RXFIM                                   SSI_INTMASK_RXFIM_Msk

#define SSI_INTMASK_RXOIM_Pos                               (3U)
#define SSI_INTMASK_RXOIM_Len                               (1U)
#define SSI_INTMASK_RXOIM_Msk                               (0x1U << SSI_INTMASK_RXOIM_Pos)
#define SSI_INTMASK_RXOIM                                   SSI_INTMASK_RXOIM_Msk

#define SSI_INTMASK_RXUIM_Pos                               (2U)
#define SSI_INTMASK_RXUIM_Len                               (1U)
#define SSI_INTMASK_RXUIM_Msk                               (0x1U << SSI_INTMASK_RXUIM_Pos)
#define SSI_INTMASK_RXUIM                                   SSI_INTMASK_RXUIM_Msk

#define SSI_INTMASK_TXOIM_Pos                               (1U)
#define SSI_INTMASK_TXOIM_Len                               (1U)
#define SSI_INTMASK_TXOIM_Msk                               (0x1U << SSI_INTMASK_TXOIM_Pos)
#define SSI_INTMASK_TXOIM                                   SSI_INTMASK_TXOIM_Msk

#define SSI_INTMASK_TXEIM_Pos                               (0U)
#define SSI_INTMASK_TXEIM_Len                               (1U)
#define SSI_INTMASK_TXEIM_Msk                               (0x1U << SSI_INTMASK_TXEIM_Pos)
#define SSI_INTMASK_TXEIM                                   SSI_INTMASK_TXEIM_Msk

/*******************  Bit definition for SSI_INTSTAT register  ****************/
#define SSI_INTSTAT_MSTIS_Pos                               (5U)
#define SSI_INTSTAT_MSTIS_Len                               (1U)
#define SSI_INTSTAT_MSTIS_Msk                               (0x1U << SSI_INTSTAT_MSTIS_Pos)
#define SSI_INTSTAT_MSTIS                                   SSI_INTSTAT_MSTIS_Msk

#define SSI_INTSTAT_RXFIS_Pos                               (4U)
#define SSI_INTSTAT_RXFIS_Len                               (1U)
#define SSI_INTSTAT_RXFIS_Msk                               (0x1U << SSI_INTSTAT_RXFIS_Pos)
#define SSI_INTSTAT_RXFIS                                   SSI_INTSTAT_RXFIS_Msk

#define SSI_INTSTAT_RXOIS_Pos                               (3U)
#define SSI_INTSTAT_RXOIS_Len                               (1U)
#define SSI_INTSTAT_RXOIS_Msk                               (0x1U << SSI_INTSTAT_RXOIS_Pos)
#define SSI_INTSTAT_RXOIS                                   SSI_INTSTAT_RXOIS_Msk

#define SSI_INTSTAT_RXUIS_Pos                               (2U)
#define SSI_INTSTAT_RXUIS_Len                               (1U)
#define SSI_INTSTAT_RXUIS_Msk                               (0x1U << SSI_INTSTAT_RXUIS_Pos)
#define SSI_INTSTAT_RXUIS                                   SSI_INTSTAT_RXUIS_Msk

#define SSI_INTSTAT_TXOIS_Pos                               (1U)
#define SSI_INTSTAT_TXOIS_Len                               (1U)
#define SSI_INTSTAT_TXOIS_Msk                               (0x1U << SSI_INTSTAT_TXOIS_Pos)
#define SSI_INTSTAT_TXOIS                                   SSI_INTSTAT_TXOIS_Msk

#define SSI_INTSTAT_TXEIS_Pos                               (0U)
#define SSI_INTSTAT_TXEIS_Len                               (1U)
#define SSI_INTSTAT_TXEIS_Msk                               (0x1U << SSI_INTSTAT_TXEIS_Pos)
#define SSI_INTSTAT_TXEIS                                   SSI_INTSTAT_TXEIS_Msk

/*******************  Bit definition for SSI_RAW_INTSTAT register  ************/
#define SSI_RAW_INTSTAT_MSTIR_Pos                           (5U)
#define SSI_RAW_INTSTAT_MSTIR_Len                           (1U)
#define SSI_RAW_INTSTAT_MSTIR_Msk                           (0x1U << SSI_RAW_INTSTAT_MSTIR_Pos)
#define SSI_RAW_INTSTAT_MSTIR                               SSI_RAW_INTSTAT_MSTIR_Msk

#define SSI_RAW_INTSTAT_RXFIR_Pos                           (4U)
#define SSI_RAW_INTSTAT_RXFIR_Len                           (1U)
#define SSI_RAW_INTSTAT_RXFIR_Msk                           (0x1U << SSI_RAW_INTSTAT_RXFIR_Pos)
#define SSI_RAW_INTSTAT_RXFIR                               SSI_RAW_INTSTAT_RXFIR_Msk

#define SSI_RAW_INTSTAT_RXOIR_Pos                           (3U)
#define SSI_RAW_INTSTAT_RXOIR_Len                           (1U)
#define SSI_RAW_INTSTAT_RXOIR_Msk                           (0x1U << SSI_RAW_INTSTAT_RXOIR_Pos)
#define SSI_RAW_INTSTAT_RXOIR                               SSI_RAW_INTSTAT_RXOIR_Msk

#define SSI_RAW_INTSTAT_RXUIR_Pos                           (2U)
#define SSI_RAW_INTSTAT_RXUIR_Len                           (1U)
#define SSI_RAW_INTSTAT_RXUIR_Msk                           (0x1U << SSI_RAW_INTSTAT_RXUIR_Pos)
#define SSI_RAW_INTSTAT_RXUIR                               SSI_RAW_INTSTAT_RXUIR_Msk

#define SSI_RAW_INTSTAT_TXOIR_Pos                           (1U)
#define SSI_RAW_INTSTAT_TXOIR_Len                           (1U)
#define SSI_RAW_INTSTAT_TXOIR_Msk                           (0x1U << SSI_RAW_INTSTAT_TXOIR_Pos)
#define SSI_RAW_INTSTAT_TXOIR                               SSI_RAW_INTSTAT_TXOIR_Msk

#define SSI_RAW_INTSTAT_TXEIR_Pos                           (0U)
#define SSI_RAW_INTSTAT_TXEIR_Len                           (1U)
#define SSI_RAW_INTSTAT_TXEIR_Msk                           (0x1U << SSI_RAW_INTSTAT_TXEIR_Pos)
#define SSI_RAW_INTSTAT_TXEIR                               SSI_RAW_INTSTAT_TXEIR_Msk

/*******************  Bit definition for SSI_TXOIC register  ******************/
#define SSI_TXOIC_TXOIC_Pos                                 (0U)
#define SSI_TXOIC_TXOIC_Len                                 (1U)
#define SSI_TXOIC_TXOIC_Msk                                 (0x1U << SSI_TXOIC_TXOIC_Pos)
#define SSI_TXOIC_TXOIC                                     SSI_TXOIC_TXOIC_Msk

/*******************  Bit definition for SSI_RXOIC register  ******************/
#define SSI_RXOIC_RXOIC_Pos                                 (0U)
#define SSI_RXOIC_RXOIC_Len                                 (1U)
#define SSI_RXOIC_RXOIC_Msk                                 (0x1U << SSI_RXOIC_RXOIC_Pos)
#define SSI_RXOIC_RXOIC                                     SSI_RXOIC_RXOIC_Msk

#define SSI_RXUIC_RXUIC_Pos                                 (0U)
#define SSI_RXUIC_RXUIC_Len                                 (1U)
#define SSI_RXUIC_RXUIC_Msk                                 (0x1U << SSI_RXUIC_RXUIC_Pos)
#define SSI_RXUIC_RXUIC                                     SSI_RXUIC_RXUIC_Msk

/*******************  Bit definition for SSI_MSTIC register  ******************/
#define SSI_MSTIC_MSTIC_Pos                                 (0U)
#define SSI_MSTIC_MSTIC_Len                                 (1U)
#define SSI_MSTIC_MSTIC_Msk                                 (0x1U << SSI_MSTIC_MSTIC_Pos)
#define SSI_MSTIC_MSTIC                                     SSI_MSTIC_MSTIC_Msk

/*******************  Bit definition for SSI_INTCLR register  ******************/
#define SSI_INTCLR_INTCLR_Pos                               (0U)
#define SSI_INTCLR_INTCLR_Len                               (1U)
#define SSI_INTCLR_INTCLR_Msk                               (0x1U << SSI_INTCLR_INTCLR_Pos)
#define SSI_INTCLR_INTCLR                                   SSI_INTCLR_INTCLR_Msk

/*******************  Bit definition for SSI_DMAC register  *******************/
#define SSI_DMAC_TDMAE_Pos                                  (1U)
#define SSI_DMAC_TDMAE_Len                                  (1U)
#define SSI_DMAC_TDMAE_Msk                                  (0x1U << SSI_DMAC_TDMAE_Pos)
#define SSI_DMAC_TDMAE                                      SSI_DMAC_TDMAE_Msk

#define SSI_DMAC_RDMAE_Pos                                  (0U)
#define SSI_DMAC_RDMAE_Len                                  (1U)
#define SSI_DMAC_RDMAE_Msk                                  (0x1U << SSI_DMAC_RDMAE_Pos)
#define SSI_DMAC_RDMAE                                      SSI_DMAC_RDMAE_Msk

/*******************  Bit definition for SSI_DMATDL register  *****************/
#define SSI_DMATDL_DMATDL_Pos                               (0U)
#define SSI_DMATDL_DMATDL_Len                               (4U)
#define SSI_DMATDL_DMATDL_Msk                               (0xFU << SSI_DMATDL_DMATDL_Pos)
#define SSI_DMATDL_DMATDL                                   SSI_DMATDL_DMATDL_Msk

#define SSI_DMARDL_DMARDL_Pos                               (0U)
#define SSI_DMARDL_DMARDL_Len                               (4U)
#define SSI_DMARDL_DMARDL_Msk                               (0xFU << SSI_DMARDL_DMARDL_Pos)
#define SSI_DMARDL_DMARDL                                   SSI_DMARDL_DMARDL_Msk

/*******************  Bit definition for SSI_IDCODE register  *****************/
#define SSI_IDCODE_ID_Pos                                   (0U)
#define SSI_IDCODE_ID_Len                                   (32U)
#define SSI_IDCODE_ID_Msk                                   (0xFFFFFFFFU)
#define SSI_IDCODE_ID                                       SSI_IDCODE_ID_Msk

/*******************  Bit definition for SSI_COMP register  *******************/
#define SSI_COMP_VERSION_Pos                                (0U)
#define SSI_COMP_VERSION_Len                                (32U)
#define SSI_COMP_VERSION_Msk                                (0xFFFFFFFFU)
#define SSI_COMP_VERSION                                    SSI_COMP_VERSION_Msk

/*******************  Bit definition for SSI_DATA register  *******************/
#define SSI_DATA_REG_Pos                                    (0U)
#define SSI_DATA_REG_Len                                    (32U)
#define SSI_DATA_REG_Msk                                    (0xFFFFFFFFU)
#define SSI_DATA_REG                                        SSI_DATA_REG_Msk

/*******************  Bit definition for SSI_RX register  *********************/
#define SSI_RX_SAMPLEDLY_Pos                                (0U)
#define SSI_RX_SAMPLEDLY_Len                                (8U)
#define SSI_RX_SAMPLEDLY_Msk                                (0xFFU << SSI_RX_SAMPLEDLY_Pos)
#define SSI_RX_SAMPLEDLY                                    SSI_RX_SAMPLEDLY_Msk

/*******************  Bit definition for SSI_SCTRL0 register  *****************/
#define SSI_SCTRL0_WAITCYCLES_Pos                           (11U)
#define SSI_SCTRL0_WAITCYCLES_Len                           (5U)
#define SSI_SCTRL0_WAITCYCLES_Msk                           (0x1FU << SSI_SCTRL0_WAITCYCLES_Pos)
#define SSI_SCTRL0_WAITCYCLES                               SSI_SCTRL0_WAITCYCLES_Msk

#define SSI_SCTRL0_INSTL_Pos                                (8U)
#define SSI_SCTRL0_INSTL_Len                                (2U)
#define SSI_SCTRL0_INSTL_Msk                                (0x03U << SSI_SCTRL0_INSTL_Pos)
#define SSI_SCTRL0_INSTL                                    SSI_SCTRL0_INSTL_Msk

#define SSI_SCTRL0_ADDRL_Pos                                (2U)
#define SSI_SCTRL0_ADDRL_Len                                (4U)
#define SSI_SCTRL0_ADDRL_Msk                                (0x0FU << SSI_SCTRL0_ADDRL_Pos)
#define SSI_SCTRL0_ADDRL                                    SSI_SCTRL0_ADDRL_Msk

#define SSI_SCTRL0_TRANSTYPE_Pos                            (0U)
#define SSI_SCTRL0_TRANSTYPE_Len                            (2U)
#define SSI_SCTRL0_TRANSTYPE_Msk                            (0x03U << SSI_SCTRL0_TRANSTYPE_Pos)
#define SSI_SCTRL0_TRANSTYPE                                SSI_SCTRL0_TRANSTYPE_Msk


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
#define UART_TXFIFO_SIZE                                    128
#define UART_RXFIFO_SIZE                                    128

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
/* ================                                        WDT                                      ================ */
/* ================================================================================================================= */
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

/*******************  Bit definition for WDT_MIS register  ********************/
#define WDT_MIS_INTSTAT_Pos                                 (0U)
#define WDT_MIS_INTSTAT_Len                                 (1U)
#define WDT_MIS_INTSTAT_Msk                                 (0x1U << WDT_MIS_INTSTAT_Pos)
#define WDT_MIS_INTSTAT                                     WDT_MIS_INTSTAT_Msk     /**< Interrupt status */


/* ================================================================================================================= */
/* ================                                       XQSPI                                     ================ */
/* ================================================================================================================= */
/*******************  Bit definition for XQSPI_CACHE_CTRL0 register  **********/
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

/** @} */ /* End of group Peripheral_Registers_Bits_Definition */

/** @addtogroup Exported_macros
  * @{
  */
/****************************** GPIO instances ********************************/
#define IS_GPIO_ALL_INSTANCE(__INSTANCE__)      (((__INSTANCE__) == GPIO0) || \
                                                 ((__INSTANCE__) == GPIO1))

/****************************** I2C instances *********************************/
#define IS_I2C_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == I2C0) || \
                                                 ((__INSTANCE__) == I2C1))

/****************************** I2S instances *********************************/
#define IS_I2S_ALL_INSTANCE(__INSTANCE__)       (((__INSTANCE__) == I2S_M) || \
                                                 ((__INSTANCE__) == I2S_S))

/****************************** UART instances ********************************/
#define IS_UART_ALL_INSTANCE(__INSTANCE__)      (((__INSTANCE__) == UART0) || \
                                                 ((__INSTANCE__) == UART1))

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
                                                 ((__INSTANCE__) == QSPI1))

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


/** @} */ /* End of group Exported_macros */


#ifdef __cplusplus
}
#endif

#endif /* __GR551xx_H__ */

/** @} */ /* End of group GR551xx */

/** @} */ /* End of group CMSIS_Device */

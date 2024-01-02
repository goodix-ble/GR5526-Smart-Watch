/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_ospi_x.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of SPI LL library.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2021 GOODIX
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
 ****************************************************************************************
 */

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup LL_DRIVER LL Driver
  * @{
  */

/** @defgroup LL_OSPI OSPI
  * @brief OSPI LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_OSPI_X_H__
#define __GR55xx_LL_OSPI_X_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"
#include "math.h"


#if defined (OSPI0)

/** @defgroup LL_OSPI_X_DRIVER_STRUCTURES Structures
  * @{
  */
/** @brief OSPI init structure. */
typedef struct _ll_ospi_x_init_t {

    uint32_t clock_freq;                    /**< Specifies the Clock Freq for OSPI */

    uint32_t dqs_timeout_ns;                /**< DQS timeout in ns */

    uint32_t mem_base_address;              /**< memory base address */

    uint32_t mem_top_address;               /**< memory top address, max 0x1FFFFFF */

    uint32_t mem_page_size;                 /**< memory page size      */

    uint32_t timing_max_tCEM_us;            /**< max tCEM time in us   */

    uint32_t timing_min_tRST_us;            /**< min tRST time in us   */

    uint32_t timing_min_tCPH_ns;            /**< min tCPH time in ns   */

    uint32_t timing_min_tRC_ns;             /**< min tRC time in ns    */

    uint32_t timing_min_tXPDPD_ns;          /**< min tXPDPD time in ns */

    uint32_t timing_min_tXDPD_us;           /**< min tXDPD time in us  */

    uint32_t timing_min_tXPHS_ns;           /**< min tXPHS time in ns  */

    uint32_t timing_min_tXHS_us;            /**< min tXHS time in us   */

    uint8_t is_tcem_ignore;                 /**< let controller ignore the tCEM or not  */

    uint8_t is_txdpd_ignore;                /**< let controller ignore the txdpd or not */

    uint8_t is_txhs_ignore;                 /**< let controller ignore the txhs or not  */

    uint8_t cmd_linear_burst_rd;            /**< Linear Burst Read Command    */

    uint8_t cmd_linear_burst_wr;            /**< Linear Burst Write Command   */

    uint8_t cmd_sync_rd;                    /**< Sync Read Command            */

    uint8_t cmd_sync_wr;                    /**< Sync Write Command           */

    uint8_t cmd_mode_register_rd;           /**< Register Mode Read Command   */

    uint8_t cmd_mode_register_wr;           /**< Register Mode Write Command  */

    uint8_t cmd_global_reset;               /**< Global Reset Command         */

} ll_ospi_x_init_t;
/** @} */

/** @defgroup LL_OSPI_X_DRIVER_DEFINES Defines
  * @{
  */
/** @defgroup LL_OSPI_X_TCEM_TIME_IGNORE tCEM time ignore enable
  * @brief    Ignore the tCEM or not
  * @{
  */
#define LL_OSPI_X_TCEM_TIME_IGNORE_DISABLE              0x00
#define LL_OSPI_X_TCEM_TIME_IGNORE_ENABLE               0x01
/** @} */

/** @defgroup LL_OSPI_X_TXHS_TIME_IGNORE tXHS time ognore enable
  * @brief    Ignore the tXHS or not
  * @{
  */
#define LL_OSPI_X_TXHS_TIME_IGNORE_DISABLE              0x00
#define LL_OSPI_X_TXHS_TIME_IGNORE_ENABLE               0x01
/** @} */

/** @defgroup LL_OSPI_X_ACCESS_TYPE PSRAM access type
  * @brief    Access type for PSRAM
  * @{
  */
#define LL_OSPI_X_ACCESS_TYPE_MEMORY_ARRAY              0x00
#define LL_OSPI_X_ACCESS_TYPE_MODE_REGISTER             0x01
/** @} */

/** @defgroup LL_OSPI_X_TXDPD_TIME_IGNORE Ignore the tXDPD enable
  * @brief    Ignore the tXDPD or not
  * @{
  */
#define LL_OSPI_X_TXDPD_TIME_IGNORE_DISABLE             0x00
#define LL_OSPI_X_TXDPD_TIME_IGNORE_ENABLE              0x01
/** @} */

/** @defgroup LL_OSPI_X_READ_PREFETCH Read prefetch enable
  * @brief    Enable the read prefetch or not
  * @{
  */
#define LL_OSPI_X_READ_PREFETCH_DISABLE                 0x00
#define LL_OSPI_X_READ_PREFETCH_ENABLE                  0x01
/** @} */

/** @defgroup LL_OSPI_X_INTERRUPT OSPI Interrupt enable
  * @brief    Enable/Disable the OSPI Interrupt
  * @{
  */
#define LL_OSPI_X_INTERRUPT_DISABLE                     0x00
#define LL_OSPI_X_INTERRUPT_ENABLE                      0x01
/** @} */

/** @defgroup LL_OSPI_X_MEM_PAGE_SIZE Page Size for PSRAM
  * @brief    Page Size for PSRAM
  * @{
  */
#define LL_OSPI_X_MEM_PAGE_SIZE_64Bytes                 0x06
#define LL_OSPI_X_MEM_PAGE_SIZE_128Bytes                0x07
#define LL_OSPI_X_MEM_PAGE_SIZE_256Bytes                0x08
#define LL_OSPI_X_MEM_PAGE_SIZE_512Bytes                0x09
#define LL_OSPI_X_MEM_PAGE_SIZE_1024Bytes               0x0A
#define LL_OSPI_X_MEM_PAGE_SIZE_2048Bytes               0x0B
#define LL_OSPI_X_MEM_PAGE_SIZE_4096Bytes               0x0C
/** @} */
/** @} */

/** @defgroup LL_OSPI_X_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
  * @brief  Set memory base address
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  base_address - base address
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_mem_base_address(ospi_x_regs_t * OSPIx, uint32_t base_address)
{
    MODIFY_REG(OSPIx->MEM_BASE_ADDR, OSPI_X_MEM_BASE_ADDR, base_address << OSPI_X_MEM_BASE_ADDR_POS);
}

/**
  * @brief  Get memory base address
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval base address
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_mem_base_address(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->MEM_BASE_ADDR, OSPI_X_MEM_BASE_ADDR) >> OSPI_X_MEM_BASE_ADDR_POS);
}

/**
  * @brief  Set memory top address
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  top_address - top address
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_mem_top_address(ospi_x_regs_t * OSPIx, uint32_t top_address)
{
    MODIFY_REG(OSPIx->MEM_TOP_ADDR, OSPI_X_MEM_TOP_ADDR, top_address << OSPI_X_MEM_TOP_ADDR_POS);
}

/**
  * @brief  Get memory top address
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval top address
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_mem_top_address(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->MEM_TOP_ADDR, OSPI_X_MEM_TOP_ADDR) >> OSPI_X_MEM_TOP_ADDR_POS);
}

/**
  * @brief  Set clock count for tRST
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  trst_cnt - clock count for tRST
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_trst_count(ospi_x_regs_t * OSPIx, uint32_t trst_cnt)
{
    MODIFY_REG(OSPIx->GLOBAL_RESET, OSPI_X_TRST_CNT, trst_cnt << OSPI_X_TRST_CNT_POS);
}

/**
  * @brief  Get clock count for tRST
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval none
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_trst_count(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->GLOBAL_RESET, OSPI_X_TRST_CNT) >> OSPI_X_TRST_CNT_POS);
}

/**
  * @brief  Trigger Global Reset Command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_enable_global_reset(ospi_x_regs_t * OSPIx)
{
    MODIFY_REG(OSPIx->GLOBAL_RESET, OSPI_X_GLOBAL_RST_EN, 1 << OSPI_X_GLOBAL_RST_EN_POS);
}

/**
  * @brief  Set access type for PSRAM
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  access_type - @ref LL_OSPI_X_ACCESS_TYPE
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_access_type(ospi_x_regs_t * OSPIx, uint32_t access_type)
{
    MODIFY_REG(OSPIx->ACCESS_TYPE, OSPI_X_ACCESS_TYPE, access_type << OSPI_X_ACCESS_TYPE_POS);
}

/**
  * @brief  Get access type for PSRAM
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval LL_OSPI_X_ACCESS_TYPE
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_access_type(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->ACCESS_TYPE, OSPI_X_ACCESS_TYPE) >> OSPI_X_ACCESS_TYPE_POS);
}

/**
  * @brief  Set ignoring the tcem or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  is_ignore - LL_OSPI_X_TCEM_TIME_IGNORE
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_tcem_ignore(ospi_x_regs_t * OSPIx, uint32_t is_ignore)
{
    MODIFY_REG(OSPIx->ACCESS_TIMING, OSPI_X_TCEM_IGNORE, is_ignore << OSPI_X_TCEM_IGNORE_POS);
}

/**
  * @brief  Get whether ignoring the tcem or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval LL_OSPI_X_TCEM_TIME_IGNORE
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_tcem_ignore(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->ACCESS_TIMING, OSPI_X_TCEM_IGNORE) >> OSPI_X_TCEM_IGNORE_POS);
}

/**
  * @brief  Set clock count for the tcem
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  tcem_cnt - clock count for the tcem
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_tcem_count(ospi_x_regs_t * OSPIx, uint32_t tcem_cnt)
{
    MODIFY_REG(OSPIx->ACCESS_TIMING, OSPI_X_TCEM_CNT, tcem_cnt << OSPI_X_TCEM_CNT_POS);
}

/**
  * @brief  Get clock count for the tcem
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval clock count for the tcem
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_tcem_count(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->ACCESS_TIMING, OSPI_X_TCEM_CNT) >> OSPI_X_TCEM_CNT_POS);
}

/**
  * @brief  Set clock count for the tRC
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  trc_cnt - clock count for the tRC
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_trc_count(ospi_x_regs_t * OSPIx, uint32_t trc_cnt)
{
    MODIFY_REG(OSPIx->ACCESS_TIMING, OSPI_X_TRC_CNT, trc_cnt << OSPI_X_TRC_CNT_POS);
}

/**
  * @brief  Get clock count for the tRC
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval clock count for the tRC
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_trc_count(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->ACCESS_TIMING, OSPI_X_TRC_CNT) >> OSPI_X_TRC_CNT_POS);
}

/**
  * @brief  Set clock count for the tCPH
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  tcph_cnt - clock count for the tCPH
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_tcph_count(ospi_x_regs_t * OSPIx, uint32_t tcph_cnt)
{
    MODIFY_REG(OSPIx->ACCESS_TIMING, OSPI_X_TCPH_CNT, tcph_cnt << OSPI_X_TCPH_CNT_POS);
}

/**
  * @brief  Get clock count for the tCPH
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval clock count for the tCPH
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_tcph_count(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->ACCESS_TIMING, OSPI_X_TCPH_CNT) >> OSPI_X_TCPH_CNT_POS);
}

/**
  * @brief  Set memory page size
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  mem_page_size - LL_OSPI_X_MEM_PAGE_SIZE
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_mem_page_size(ospi_x_regs_t * OSPIx, uint32_t mem_page_size)
{
    MODIFY_REG(OSPIx->ACCESS_TIMING, OSPI_X_MEM_PAGE_SIZE, mem_page_size << OSPI_X_MEM_PAGE_SIZE_POS);
}

/**
  * @brief  Get memory page size
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval LL_OSPI_X_MEM_PAGE_SIZE
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_mem_page_size(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->ACCESS_TIMING, OSPI_X_MEM_PAGE_SIZE) >> OSPI_X_MEM_PAGE_SIZE_POS);
}

/**
  * @brief  Trigger the entry deep power down command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_entry_dpd(ospi_x_regs_t * OSPIx)
{
    MODIFY_REG(OSPIx->DEEP_DOWN_CNTRL, OSPI_X_DPD_ENTRY, 0x01 << OSPI_X_DPD_ENTRY_POS);
}

/**
  * @brief  Trigger the exit deep power down command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_exit_dpd(ospi_x_regs_t * OSPIx)
{
    MODIFY_REG(OSPIx->DEEP_DOWN_CNTRL, OSPI_X_DPD_EXIT, 0x01 << OSPI_X_DPD_EXIT_POS);
}

/**
  * @brief  Set ignoring the txdpd or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  is_ignore - LL_OSPI_X_TXDPD_TIME_IGNORE
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_txdpd_ignore(ospi_x_regs_t * OSPIx, uint32_t is_ignore)
{
    MODIFY_REG(OSPIx->DEEP_DOWN_CNTRL, OSPI_X_TXDPD_TIME_IGNORE, is_ignore << OSPI_X_TXDPD_TIME_IGNORE_POS);
}

/**
  * @brief  Get whether ignoring the txdpd or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval LL_OSPI_X_TXDPD_TIME_IGNORE
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_txdpd_ignore(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->DEEP_DOWN_CNTRL, OSPI_X_TXDPD_TIME_IGNORE) >> OSPI_X_TXDPD_TIME_IGNORE_POS);
}

/**
  * @brief  Set clock count for the tXDPD
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  txdpd_cnt - clock count for the tXDPD
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_txdpd_count(ospi_x_regs_t * OSPIx, uint32_t txdpd_cnt)
{
    MODIFY_REG(OSPIx->DEEP_DOWN_CNTRL, OSPI_X_TXDPD_CNT, txdpd_cnt << OSPI_X_TXDPD_CNT_POS);
}

/**
  * @brief  Get clock count for the tXDPD
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval clock count for the tXDPD
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_txdpd_count(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->DEEP_DOWN_CNTRL, OSPI_X_TXDPD_CNT) >> OSPI_X_TXDPD_CNT_POS);
}

/**
  * @brief  Set clock count for the tXPDPD
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  txpdpd_cnt - clock count for the tXPDPD
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_txpdpd_count(ospi_x_regs_t * OSPIx, uint32_t txpdpd_cnt)
{
    MODIFY_REG(OSPIx->DEEP_DOWN_CNTRL, OSPI_X_TXPDPD_CNT, txpdpd_cnt << OSPI_X_DPD_EXIT_CYCLE_CNT_POS);
}

/**
  * @brief  Get clock count for the tXPDPD
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval clock count for the tXPDPD
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_txpdpd_count(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->DEEP_DOWN_CNTRL, OSPI_X_TXPDPD_CNT) >> OSPI_X_DPD_EXIT_CYCLE_CNT_POS);
}

/**
  * @brief  Trigger the entry half sleep command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_entry_half_sleep(ospi_x_regs_t * OSPIx)
{
    MODIFY_REG(OSPIx->HALF_SLP_CNTRL, OSPI_X_HS_ENTRY, 0x01 << OSPI_X_HS_ENTRY_POS);
}

/**
  * @brief  Trigger the exit half sleep command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_exit_half_sleep(ospi_x_regs_t * OSPIx)
{
    MODIFY_REG(OSPIx->HALF_SLP_CNTRL, OSPI_X_HS_EXIT, 0x01 << OSPI_X_HS_EXIT_POS);
}

/**
  * @brief  Set ignoring the tXHS or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  is_ignore - LL_OSPI_X_TXHS_TIME_IGNORE
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_txhs_ignore(ospi_x_regs_t * OSPIx, uint32_t is_ignore)
{
    MODIFY_REG(OSPIx->HALF_SLP_CNTRL, OSPI_X_TXHS_TIME_IGNORE, is_ignore << OSPI_X_TXHS_TIME_IGNORE_POS);
}

/**
  * @brief  Get whether ignoring the tXHS or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval LL_OSPI_X_TXHS_TIME_IGNORE
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_txhs_ignore(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->HALF_SLP_CNTRL, OSPI_X_TXHS_TIME_IGNORE) >> OSPI_X_TXHS_TIME_IGNORE_POS);
}

/**
  * @brief  Set clock count for the tXHS
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  txhs_count - clock count for the tXHS
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_txhs_count(ospi_x_regs_t * OSPIx, uint32_t txhs_count)
{
    MODIFY_REG(OSPIx->HALF_SLP_CNTRL, OSPI_X_TXHS_CNT, txhs_count << OSPI_X_TXHS_CNT_POS);
}

/**
  * @brief  Get clock count for the tXHS
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval clock count for the tXHS
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_txhs_count(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->HALF_SLP_CNTRL, OSPI_X_TXHS_CNT) >> OSPI_X_TXHS_CNT_POS);
}

/**
  * @brief  Set clock count for the tXPHS
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  txphs_count - clock count for the tXPHS
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_txphs_count(ospi_x_regs_t * OSPIx, uint32_t txphs_count)
{
    MODIFY_REG(OSPIx->HALF_SLP_CNTRL, OSPI_X_HS_EXIT_CYCLE_CNT, txphs_count << OSPI_X_HS_EXIT_CYCLE_CNT_POS);
}

/**
  * @brief  Get clock count for the tXPHS
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval clock count for the tXPHS
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_txphs_count(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->HALF_SLP_CNTRL, OSPI_X_HS_EXIT_CYCLE_CNT) >> OSPI_X_HS_EXIT_CYCLE_CNT_POS);
}

/**
  * @brief  Enable the global reset interrupt or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  intr - LL_OSPI_X_INTERRUPT
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_global_rst_interrupt(ospi_x_regs_t * OSPIx, uint32_t intr)
{
    MODIFY_REG(OSPIx->INTERRUPT_CNTRL, OSPI_X_GLOBAL_RST_IE, intr << OSPI_X_GLOBAL_RST_IE_POS);
}

/**
  * @brief  Check whether the global reset interrupt is enabled or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval LL_OSPI_X_INTERRUPT
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_global_rst_interrupt_enabled(ospi_x_regs_t * OSPIx)
{
    return (READ_BITS(OSPIx->INTERRUPT_CNTRL, OSPI_X_GLOBAL_RST_IE) == OSPI_X_GLOBAL_RST_IE) ? 1 : 0;
}

/**
  * @brief  Enable the half sleep entry interrupt or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  intr - LL_OSPI_X_INTERRUPT
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_half_sleep_entry_interrupt(ospi_x_regs_t * OSPIx, uint32_t intr)
{
    MODIFY_REG(OSPIx->INTERRUPT_CNTRL, OSPI_X_HS_ENTRY_IE, intr << OSPI_X_HS_ENTRY_IE_POS);
}

/**
  * @brief  Check whether the half sleep entry interrupt is enabled or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval LL_OSPI_X_INTERRUPT
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_half_sleep_entry_interrupt_enabled(ospi_x_regs_t * OSPIx)
{
    return (READ_BITS(OSPIx->INTERRUPT_CNTRL, OSPI_X_HS_ENTRY_IE) == OSPI_X_HS_ENTRY_IE) ? 1 : 0;
}

/**
  * @brief  Enable the half sleep exit interrupt or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  intr - LL_OSPI_X_INTERRUPT
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_half_sleep_exit_interrupt(ospi_x_regs_t * OSPIx, uint32_t intr)
{
    MODIFY_REG(OSPIx->INTERRUPT_CNTRL, OSPI_X_HS_EXIT_IE, intr << OSPI_X_HS_EXIT_IE_POS);
}

/**
  * @brief  Check whether the half sleep exit interrupt is enabled or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval LL_OSPI_X_INTERRUPT
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_half_sleep_exit_interrupt_enabled(ospi_x_regs_t * OSPIx)
{
    return (READ_BITS(OSPIx->INTERRUPT_CNTRL, OSPI_X_HS_EXIT_IE) == OSPI_X_HS_EXIT_IE) ? 1 : 0;
}

/**
  * @brief  Enable the deep power down entry interrupt or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  intr - LL_OSPI_X_INTERRUPT
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_dpd_entry_interrupt(ospi_x_regs_t * OSPIx, uint32_t intr)
{
    MODIFY_REG(OSPIx->INTERRUPT_CNTRL, OSPI_X_DPD_ENTRY_IE, intr << OSPI_X_DPD_ENTRY_IE_POS);
}

/**
  * @brief  Check whether the deep power down entry interrupt is enabled or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval LL_OSPI_X_INTERRUPT
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_dpd_entry_interrupt_enabled(ospi_x_regs_t * OSPIx)
{
    return (READ_BITS(OSPIx->INTERRUPT_CNTRL, OSPI_X_DPD_ENTRY_IE) == OSPI_X_DPD_ENTRY_IE) ? 1 : 0;
}

/**
  * @brief  Enable the deep power down exit interrupt or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  intr - LL_OSPI_X_INTERRUPT
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_dpd_exit_interrupt(ospi_x_regs_t * OSPIx, uint32_t intr)
{
    MODIFY_REG(OSPIx->INTERRUPT_CNTRL, OSPI_X_DPD_EXIT_IE, intr << OSPI_X_DPD_EXIT_IE_POS);
}

/**
  * @brief  Check whether the deep power down exit interrupt is enabled or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval LL_OSPI_X_INTERRUPT
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_dpd_exit_interrupt_enabled(ospi_x_regs_t * OSPIx)
{
    return (READ_BITS(OSPIx->INTERRUPT_CNTRL, OSPI_X_DPD_EXIT_IE) == OSPI_X_DPD_EXIT_IE) ? 1 : 0;
}

/**
  * @brief  Enable the dqs timeout interrupt or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  intr - LL_OSPI_X_INTERRUPT
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_dqs_timeout_interrupt(ospi_x_regs_t * OSPIx, uint32_t intr)
{
    MODIFY_REG(OSPIx->INTERRUPT_CNTRL, OSPI_X_DQS_TIMEOUT_IE, intr << OSPI_X_DQS_TIMEOUT_IE_POS);
}

/**
  * @brief  Check whether the dqs timeout interrupt is enabled or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval LL_OSPI_X_INTERRUPT
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_dqs_timeout_interrupt_enabled(ospi_x_regs_t * OSPIx)
{
    return (READ_BITS(OSPIx->INTERRUPT_CNTRL, OSPI_X_DQS_TIMEOUT_IE) == OSPI_X_DQS_TIMEOUT_IE) ? 1 : 0;
}

/**
  * @brief  Check whether the global reset is done or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval 1 - done ; 0 - not done
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_global_rst_done(ospi_x_regs_t * OSPIx)
{
    uint32_t status = OSPIx->XFER_STATUS;
    return ((status & 0x01) > 0 ?  1 :  0);
}

/**
  * @brief  Check whether the half sleep entry operation is done or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval 1 - done ; 0 - not done
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_half_sleep_entry_done(ospi_x_regs_t * OSPIx)
{
    return (READ_BITS(OSPIx->XFER_STATUS, OSPI_X_HS_ENTRY_DONE) == OSPI_X_HS_ENTRY_DONE) ? 1 : 0;
}

/**
  * @brief  Check whether the half sleep exit operation is done or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval 1 - done ; 0 - not done
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_half_sleep_exit_done(ospi_x_regs_t * OSPIx)
{
    return (READ_BITS(OSPIx->XFER_STATUS, OSPI_X_HS_EXIT_DONE) == OSPI_X_HS_EXIT_DONE) ? 1 : 0;
}

/**
  * @brief  Check whether the deep power down entry operation is done or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval 1 - done ; 0 - not done
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_dpd_entry_done(ospi_x_regs_t * OSPIx)
{
    return (READ_BITS(OSPIx->XFER_STATUS, OSPI_X_DPD_ENTRY_DONE) == OSPI_X_DPD_ENTRY_DONE) ? 1 : 0;
}

/**
  * @brief  Check whether the deep power down exit operation is done or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval 1 - done ; 0 - not done
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_dpd_exit_done(ospi_x_regs_t * OSPIx)
{
    return (READ_BITS(OSPIx->XFER_STATUS, OSPI_X_DPD_EXIT_DONE) == OSPI_X_DPD_EXIT_DONE) ? 1 : 0;
}

/**
  * @brief  Check whether the dqs timeout error happen or not
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval 1 - happen ; 0 - not happen
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_dqs_timeout_err(ospi_x_regs_t * OSPIx)
{
    return (READ_BITS(OSPIx->XFER_STATUS, OSPI_X_DQS_NON_TOGGLE_ERR) == OSPI_X_DQS_NON_TOGGLE_ERR) ? 1 : 0;
}

/**
  * @brief  Set sync read command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  cmd    sync read command
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_sync_read_cmd(ospi_x_regs_t * OSPIx, uint32_t cmd)
{
    MODIFY_REG(OSPIx->CMD_CNTRL_1, OSPI_X_CMD_SYNC_RD, cmd << OSPI_X_CMD_SYNC_RD_POS);
}

/**
  * @brief  Get sync read command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval sync read command
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_sync_read_cmd(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->CMD_CNTRL_1, OSPI_X_CMD_SYNC_RD) >> OSPI_X_CMD_SYNC_RD_POS);
}

/**
  * @brief  Set sync write command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  cmd    sync write command
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_sync_write_cmd(ospi_x_regs_t * OSPIx, uint32_t cmd)
{
    MODIFY_REG(OSPIx->CMD_CNTRL_1, OSPI_X_CMD_SYNC_WR, cmd << OSPI_X_CMD_SYNC_WR_POS);
}

/**
  * @brief  Get sync write command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval sync write command
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_sync_write_cmd(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->CMD_CNTRL_1, OSPI_X_CMD_SYNC_WR) >> OSPI_X_CMD_SYNC_WR_POS);
}

/**
  * @brief  Set linear burst read command
  *--------------------------
  * @param  OSPIx: OSPI instance
  * @param  cmd:   Linear burst read command
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_burst_read_cmd(ospi_x_regs_t * OSPIx, uint32_t cmd)
{
    MODIFY_REG(OSPIx->CMD_CNTRL_1, OSPI_X_CMD_BURST_RD, cmd << OSPI_X_CMD_BURST_RD_POS);
}

/**
  * @brief  Get linear burst read command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval linear burst read command
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_burst_read_cmd(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->CMD_CNTRL_1, OSPI_X_CMD_BURST_RD) >> OSPI_X_CMD_BURST_RD_POS);
}

/**
  * @brief  Set linear burst write command
  *--------------------------
  * @param  OSPIx: OSPI instance
  * @param  cmd:   Linear burst write command
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_burst_write_cmd(ospi_x_regs_t * OSPIx, uint32_t cmd)
{
    MODIFY_REG(OSPIx->CMD_CNTRL_1, OSPI_X_CMD_BURST_WR, cmd << OSPI_X_CMD_BURST_WR_POS);
}

/**
  * @brief  Get linear burst write command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval linear burst write command
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_burst_write_cmd(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->CMD_CNTRL_1, OSPI_X_CMD_BURST_WR) >> OSPI_X_CMD_BURST_WR_POS);
}

/**
  * @brief  Set register mode read command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  cmd   register mode read
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_register_read_cmd(ospi_x_regs_t * OSPIx, uint32_t cmd)
{
    MODIFY_REG(OSPIx->CMD_CNTRL_2, OSPI_X_CMD_REG_RD, cmd << OSPI_X_CMD_REG_RD_POS);
}

/**
  * @brief  Get register mode read command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval register mode read command
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_register_read_cmd(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->CMD_CNTRL_2, OSPI_X_CMD_REG_RD) >> OSPI_X_CMD_REG_RD_POS);
}

/**
  * @brief  Set register mode write command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  cmd    register mode write command
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_register_write_cmd(ospi_x_regs_t * OSPIx, uint32_t cmd)
{
    MODIFY_REG(OSPIx->CMD_CNTRL_2, OSPI_X_CMD_REG_WR, cmd << OSPI_X_CMD_REG_WR_POS);
}

/**
  * @brief  Get register mode write command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval register mode write command
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_register_write_cmd(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->CMD_CNTRL_2, OSPI_X_CMD_REG_WR) >> OSPI_X_CMD_REG_WR_POS);
}

/**
  * @brief  Set global reset command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @param  cmd    sync read command
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_global_rst_cmd(ospi_x_regs_t * OSPIx, uint32_t cmd)
{
    MODIFY_REG(OSPIx->CMD_CNTRL_2, OSPI_X_CMD_GLOBAL_RST, cmd << OSPI_X_CMD_GLOBAL_RST_POS);
}

/**
  * @brief  Get global reset command
  *--------------------------
  * @param  OSPIx OSPI instance
  * @retval global reset command
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_global_rst_cmd(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->CMD_CNTRL_2, OSPI_X_CMD_GLOBAL_RST) >> OSPI_X_CMD_GLOBAL_RST_POS);
}

/**
  * @brief Set clock count for DQS timeout
  *--------------------------
  * @param OSPIx OSPI instance
  * @param timeout_clk clock count for DQS timeout
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_dqs_timeout(ospi_x_regs_t * OSPIx, uint32_t timeout_clk)
{
    timeout_clk = (timeout_clk > 31) ? 31 : timeout_clk;
    MODIFY_REG(OSPIx->DQS_TIMEOUT, OSPI_X_DQS_NON_TGL_TIMEOUT, timeout_clk << OSPI_X_DQS_NON_TGL_TIMEOUT_POS);
}

/**
  * @brief Get clock count for DQS timeout
  *--------------------------
  * @param OSPIx OSPI instance
  * @retval clock count for DQS timeout
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_dqs_timeout(ospi_x_regs_t * OSPIx)
{
    return (uint32_t)(READ_BITS(OSPIx->DQS_TIMEOUT, OSPI_X_DQS_NON_TGL_TIMEOUT) >> OSPI_X_DQS_NON_TGL_TIMEOUT_POS);
}

/**
  * @brief Enable the read prefetch feature or not
  *--------------------------
  * @param OSPIx OSPI instance
  * @param is_prefetch - LL_OSPI_X_READ_PREFETCH
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_read_prefetch(ospi_x_regs_t * OSPIx, uint32_t is_prefetch)
{
    MODIFY_REG(OSPIx->READ_PREFETCH, OSPI_X_RD_DATA_PREFETCH, is_prefetch << OSPI_X_RD_DATA_PREFETCH_POS);
}

/**
  * @brief Check whether the read prefetch feature is enabled or not
  *--------------------------
  * @param OSPIx OSPI instance
  * @retval LL_OSPI_X_READ_PREFETCH
  */
__STATIC_INLINE uint32_t ll_ospi_x_is_read_prefetch_enabled(ospi_x_regs_t * OSPIx)
{
    return (READ_BITS(OSPIx->READ_PREFETCH, OSPI_X_RD_DATA_PREFETCH) == OSPI_X_RD_DATA_PREFETCH) ? 1 : 0;
}

/**
  * @brief Set the Phy Delay TAP
  *--------------------------
  * @param OSPIx OSPI instance
  * @param phy_delay 
  * @retval none
  */
__STATIC_INLINE void ll_ospi_x_set_phy_delay(ospi_x_regs_t * OSPIx, uint32_t phy_delay)
{
    MODIFY_REG(OSPIx->PHY_CNTRL_0, 0xFF, phy_delay);
}

/**
  * @brief Get the Phy Delay TAP
  *--------------------------
  * @param OSPIx OSPI instance
  * @retval LL_OSPI_X_READ_PREFETCH
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_phy_delay(ospi_x_regs_t * OSPIx)
{
    return READ_BITS(OSPIx->PHY_CNTRL_0, 0xFF);
}

/**
  * @brief Get memory starting address in AHB
  *--------------------------
  * @param OSPIx OSPI instance
  * @retval OSPI0_XIP_BASE
  */
__STATIC_INLINE uint32_t ll_ospi_x_get_xip_base_address(ospi_x_regs_t * OSPIx) {
    return OSPI0_XIP_BASE;
}

/**
  * @brief  Configure the ospi_x unit.
  * @param  OSPIx OSPI instance
  * @param  p_ospi_x_init pointer to a @ref ll_ospi_x_init_t structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: not applicable
  */
error_status_t ll_ospi_x_init(ospi_x_regs_t * OSPIx, ll_ospi_x_init_t * p_ospi_x_init);

/**
  * @brief  Set OSPI registers to their reset values.
  * @param  OSPIx OSPI instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: invalid spi instance
  */
error_status_t ll_ospi_x_deinit(ospi_x_regs_t * OSPIx);
/** @} */
#endif /* OSPI0 */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_OSPI_X_H__ */

/** @} */

/** @} */

/** @} */

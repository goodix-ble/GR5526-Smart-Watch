/**
 ****************************************************************************************
 *
 * @file    app_graphics_ospi.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of OSPI app library.
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

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_GRAPHICS_OSPI OSPI
  * @brief OSPI APP module driver.
  * @{
  */

#ifndef __APP_GRAPHICS_OSPI_H__
#define __APP_GRAPHICS_OSPI_H__

#include "gr55xx.h"
#include "gr55xx_ll_ospi_x.h"
#include "stdbool.h"
#include "app_drv_error.h"
#include "app_drv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup APP_GRAPHICS_OSPI_ENUM Enumerations
  * @{
  */

/**
  * @brief OSPI Clock Frequency Enumerations definition
  */
typedef enum {
    OSPI_CLOCK_FREQ_48MHz = 0x00,
    OSPI_CLOCK_FREQ_32MHz = 0x01,
    OSPI_CLOCK_FREQ_24MHz = 0x02,
    OSPI_CLOCK_FREQ_16MHz = 0x03,
} app_ospi_clock_freq_e;


/**
  * @brief OSPI PHY Delay Value Enumerations definition
  */
typedef enum {
    OSPI_PHY_DELAY_TAP_2 = 0x01,    /* phy delay 2 about ? ns */
    OSPI_PHY_DELAY_TAP_3 = 0x02,    /* phy delay 3 about ? ns */
    OSPI_PHY_DELAY_TAP_4 = 0x03,    /* phy delay 4 about ? ns */
} app_ospi_phy_delay_e;

/**
  * @brief PSRAM Drive Strength Enumerations definition
  */
typedef enum {
    OSPI_PSRAM_DRV_STR_FULL         = 0x00,    /* about 25  Ohm */
    OSPI_PSRAM_DRV_STR_HALF         = 0x01,    /* about 50  Ohm */
    OSPI_PSRAM_DRV_STR_ONE_FORTH    = 0x02,    /* about 100 Ohm */
    OSPI_PSRAM_DRV_STR_ONE_EIGHTH   = 0x03,    /* about 200 Ohm */
} app_ospi_psram_drv_strength_e;

/**
  * @brief PSRAM Read Latency Enumerations definition
  */
typedef enum {
    OSPI_PSRAM_RD_LATENCY_3   = 0x00,
    OSPI_PSRAM_RD_LATENCY_4   = 0x01,
    OSPI_PSRAM_RD_LATENCY_5   = 0x02,
    OSPI_PSRAM_RD_LATENCY_6   = 0x03,
    OSPI_PSRAM_RD_LATENCY_7   = 0x04,
} app_ospi_psram_rd_latency_e;

/**
  * @brief PSRAM Write Latency Enumerations definition
  */
typedef enum {
    OSPI_PSRAM_WR_LATENCY_3   = 0x00,
    OSPI_PSRAM_WR_LATENCY_4   = 0x04,
    OSPI_PSRAM_WR_LATENCY_5   = 0x02,
    OSPI_PSRAM_WR_LATENCY_6   = 0x06,
    OSPI_PSRAM_WR_LATENCY_7   = 0x01,
} app_ospi_psram_wr_latency_e;

/**
  * @brief PSRAM Access Mode Enumerations definition
  */
typedef enum {
    OSPI_ACCESS_MEMORY = 0,         /* read & write the psram */
    OSPI_ACCESS_REGISTER,           /* use to access the register */
    OSPI_ACCESS_UNSET,
} app_ospi_access_mode_e;

/**
  * @brief PSRAM Work Mode Enumerations definition
  */
typedef enum {
    OSPI_STATE_DEEP_SLEEP   = 0,        /* deep sleep, all memory data loss except register data */
    OSPI_STATE_HALF_SLEEP   = 1,        /* half sleep, all data kept */
    OSPI_STATE_ACTIVE       = 2,        /* active, not sleep    */
} app_ospi_work_state_e;

/**
  * @brief PSRAM Partial Array Refresh Enumerations definition
  */
typedef enum {
    OSPI_PASR_FULL = 0x00,          /* refresh full array */
    OSPI_PASR_BOTTOM_4MB = 0x01,    /* refresh Bottom 4MB array, [000000h,3FFFFFh] */
    OSPI_PASR_BOTTOM_2MB = 0x02,    /* refresh Bottom 2MB array, [000000h,1FFFFFh] */
    OSPI_PASR_BOTTOM_1MB = 0x03,    /* refresh Bottom 1MB array, [000000h,0FFFFFh] */
    OSPI_PASR_NONE = 0x04,          /* refresh None */
    OSPI_PASR_TOP_4MB = 0x05,       /* refresh Top 4MB array, [400000h,7FFFFFh] */
    OSPI_PASR_TOP_2MB = 0x06,       /* refresh Top 2MB array, [600000h,7FFFFFh] */
    OSPI_PASR_TOP_1MB = 0x07,       /* refresh Top 1MB array, [700000h,7FFFFFh] */
    OSPI_PASR_MAX,                  /* MAX mark */
} app_ospi_pasr_e;


/** @} */

/** @addtogroup APP_OSPI_STRUCTURES Structures
  * @{
  */

/**
  * @brief Define init params for OSPI
  */
typedef struct {
    app_ospi_clock_freq_e           ospi_freq;            /**< Specify ospi freq, Ref Optional values of app_ospi_clock_freq_e */
    app_ospi_psram_drv_strength_e   drv_strength;         /**< Specify psram's drive strength, Ref Optional values of app_ospi_psram_drv_strength_e */
    app_ospi_psram_rd_latency_e     rd_lc;                /**< Specify psram's read latency, Ref Optional values of app_ospi_clock_freq_e */
    app_ospi_psram_wr_latency_e     wr_lc;                /**< Specify psram's write latency, Ref Optional values of app_ospi_clock_freq_e */
    uint32_t                        phy_delay;            /**< Specify the delay tap for ospi phy, Ref Optional values of app_ospi_phy_delay_e */
    uint8_t                         is_read_prefetch;     /**< Specify whether to enable the read prefetch feature or not */
} app_graphics_ospi_params_t;

/** @} */

/** @addtogroup APP_OSPI_TYPEDEFS Type definitions
  * @{
  */

/**
  * @brief Define IRQ handler for OSPI
  */
typedef void (* ospi_irq_handler)(void);

/**
  * @brief Define PSRAM reload function for OSPI
  */
typedef void (* psram_reload_func_t)(void);

/** @} */

/**
  * @defgroup  APP_OSPI_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup APP_OSPI_Exported_Constants OSPI Exported Constants
  * @{
  */
#define OSPI_PSRAM_DEVICE_ID                0x13                            /**< PSRAM ID */
#define OSPI_PSRAM_MIN_XIP_ADDRESS          OSPI0_XIP_BASE                  /**< XIP Base Address */
#define OSPI_PSRAM_MAX_XIP_ADDRESS          (OSPI0_XIP_BASE + 0x7FFFFF)     /**< 64Mbit Space */
#define OSPI_PSRAM_BYTE_SIZE                (0x800000)                      /**< 64Mbit Space */

/**
  * @brief PSRAM init parameter definition
  */
#define PSRAM_INIT_PARAMS_Default   \
    {   \
        .ospi_freq          = OSPI_CLOCK_FREQ_48MHz,        \
        .drv_strength       = OSPI_PSRAM_DRV_STR_ONE_EIGHTH, \
        .rd_lc              = OSPI_PSRAM_RD_LATENCY_3,      \
        .wr_lc              = OSPI_PSRAM_WR_LATENCY_3,      \
        .phy_delay          =  1,                           \
        .is_read_prefetch   =  0,                           \
    }
/** @} */

/** @} */


/** @addtogroup APP_OSPI_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the APP OSPI DRIVER according to the specified parameters.
 *
 * @param[in]  p_params: Pointer to app_graphics_ospi_params_t parameter which contains the
 *                       configuration information for the specified OSPI module.
 *
 * @return APP_DRV_*
 ****************************************************************************************
 */
uint16_t app_graphics_ospi_init(app_graphics_ospi_params_t * p_params);

/**
 ****************************************************************************************
 * @brief  De-Initialize the APP OSPI DRIVER .
 *
 ****************************************************************************************
 */
void app_graphics_ospi_deinit(void);

/**
 ****************************************************************************************
 * @brief  Reset the OSPI Controller.
 *
 ****************************************************************************************
 */
void app_graphics_ospi_reset(void);

/**
 ****************************************************************************************
 * @brief  Register the OSPI reload function for PSRAM recovery after deep sleep.
 *
 * @param[in]  psram_reload_func: Reload the resources to PSRAM after PSRAM deep sleep.
 ****************************************************************************************
 */
void app_graphics_ospi_register_psram_reload_func(psram_reload_func_t psram_reload_func);

/**
 ****************************************************************************************
 * @brief  Set the OSPI low power state for system sleep.
 *
 * @param[in]  state: Ref the @ref app_ospi_work_state_e
 ****************************************************************************************
 */
void app_graphics_ospi_set_sleep_state(app_ospi_work_state_e state);

/**
 ****************************************************************************************
 * @brief  Get the OSPI low power state for system sleep.
 *
 * @return state: Ref the @ref app_ospi_work_state_e
 ****************************************************************************************
 */
app_ospi_work_state_e app_graphics_ospi_get_sleep_state(void);

/**
 ****************************************************************************************
 * @brief  Get the OSPI.PSRAM Base Address
 *
 * @return OSPI.PSRAM Base Address
 ****************************************************************************************
 */
uint32_t app_graphics_ospi_get_base_address(void);

/**
 ****************************************************************************************
 * @brief  Set Partial Refresh Array for OSPI PSRAM
 *
 * @param[in]  area: please refer to app_ospi_pasr_e Enumerations
 *
 ****************************************************************************************
 */
void app_graphics_ospi_set_pasr(app_ospi_pasr_e area);

/**
 ****************************************************************************************
 * @brief  Set Partial Refresh Array for OSPI PSRAM with PSRAM used address
 *
 * @param[in]  psram_addr: PSRAM highest used address
 *
 ****************************************************************************************
 */
void app_graphics_ospi_pasr_update(uint32_t psram_addr);

/**
 ****************************************************************************************
 * @brief  Check address is located at ospi.psram or not
 *
 * @return OSPI.PSRAM Base Address
 ****************************************************************************************
 */
__STATIC_INLINE bool app_graphics_is_ospi_address(uint32_t address) {
    return ((address >= OSPI_PSRAM_MIN_XIP_ADDRESS) && (address <= OSPI_PSRAM_MAX_XIP_ADDRESS));
}

/**
 ****************************************************************************************
 * @brief  Enable/Disable read prefetch of OSPI PSRAM
 *
 * @return OSPI.PSRAM Base Address
 ****************************************************************************************
 */
__STATIC_INLINE void app_graphics_ospi_set_read_prefetch(bool enable) {
    ll_ospi_x_set_read_prefetch(OSPI0, enable ? 1 : 0);
}

/**
 ****************************************************************************************
 * @brief  Get read prefetch state of OSPI PSRAM
 *
 * @return OSPI.PSRAM Base Address
 ****************************************************************************************
 */
__STATIC_INLINE bool app_graphics_ospi_get_read_prefetch(void) {
    return ll_ospi_x_is_read_prefetch_enabled(OSPI0);
}

/**
 ****************************************************************************************
 * @brief  Get the OSPI.PSRAM Size in Bytes
 *
 * @return OSPI.PSRAM Space Size in Bytes
 ****************************************************************************************
 */
uint32_t app_graphics_ospi_get_byte_size(void);

/**
 ****************************************************************************************
 * @brief  Set the OSPI work state.
 * @param[in]  state: please refer to app_ospi_work_state_e Enumerations
 ****************************************************************************************
 */
void app_graphics_ospi_set_power_state(app_ospi_work_state_e state);

/**
 ****************************************************************************************
 * @brief  Adjust the Digcore policy for system (including mcu.ble.graphics)
 *
 * @return
 ****************************************************************************************
 */
void app_graphics_adjust_dcore_policy(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __APP_GRAPHICS_OSPI_H__ */

/** @} */
/** @} */
/** @} */

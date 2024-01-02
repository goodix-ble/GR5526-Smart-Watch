/**
 *******************************************************************************
 *
 * @file gr55xx_sys.h
 *
 * @brief GR55XX System API
 *
 *******************************************************************************
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

/**
 @addtogroup SYSTEM
 @{
 */

/**
 * @addtogroup SYS System SDK
 * @{
 * @brief Definitions and prototypes for the system SDK interface.
*/



#ifndef __GR55XX_SYS_SDK_H__
#define __GR55XX_SYS_SDK_H__

#include "gr55xx_sys_cfg.h"
#include "gr55xx_nvds.h"
#include "gr55xx_pwr.h"
#include "ble_error.h"
#include "gr55xx_hal_adc.h"
#include "gr55xx_hal_exflash.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/** @addtogroup GR55XX_SYS_DEFINES Defines
 * @{
 */
#define SYS_INVALID_TIMER_ID              0xFF              /**< Invalid system Timer ID. */
#define SYS_BD_ADDR_LEN                   BLE_GAP_ADDR_LEN      /**< Length of Bluetoth Device Address. */
#define SYS_CHIP_UID_LEN                  0x10              /**< Length of Bluetoth Chip UID. */
#define SYS_SET_BD_ADDR(BD_ADDR_ARRAY)    nvds_put(0xC001, SYS_BD_ADDR_LEN, BD_ADDR_ARRAY)  /**< NVDS put BD address. */
/** @} */

/**
 * @defgroup GR55XX_SYS_TYPEDEFS Type definitions
 * @{
 */
/**@brief The function pointers to register event callback. */
typedef void (*callback_t)(int);

/** @brief Timer callback type. */
typedef void (*timer_callback_t)(uint8_t timer_id);

/**@brief Printf callback type. */
typedef int (*vprintf_callback_t) (const char *fmt, __va_list argp);

/**@brief raw log callback type. */
typedef uint16_t (*raw_log_send_cb_t) (uint8_t *p_data, uint16_t length);

/**@brief Low power clock update function type. */
typedef void (*void_func_t)(void);

/**@brief Low power clock update function type. */
typedef int32_t (*int_func_t)(void);

/**@brief Function type for saving user context before deep sleep. */
typedef void (*sys_context_func_t)(void);

/**@brief Error assert callback type. */
typedef void (*assert_err_cb_t)(const char *expr, const char *file, int line);

/**@brief Parameter assert callback type. */
typedef void (*assert_param_cb_t)(int param0, int param1, const char *file, int line);

/**@brief Warning assert callback type. */
typedef void (*assert_warn_cb_t)(int param0, int param1, const char *file, int line);
/** @} */

/** @addtogroup GR55XX_SYS_ENUMERATIONS Enumerations
* @{*/
/**@brief Definition of Device SRAM Size Enumerations. */
typedef enum
{
    SYS_DEV_SRAM_288K          = 0x01,    /**< Supported 288K SRAM. */
    SYS_DEV_SRAM_512K          = 0x00,    /**< Supported 512K SRAM. */
} sram_size_t;

/**@brief package type. */
typedef enum
{
    PACKAGE_NONE           = 0,                    /**< Package unused. */

    PACKAGE_GR5526VGBIP   = 0x0100,       /**< BGA83 with 8Mb flash/GPU/PSRAM, with cap, no GND3 WB, add dummy Die. */
    PACKAGE_GR5526VGBI     = 0x0101,       /**< BGA83 with 8Mb flash. no GPU/PSRAM. */
    PACKAGE_GR5526_DEBUG = 0x0102,       /**< BGA83 debug chip, test with XQSPI/OSPI pin-out. */

    PACKAGE_GR5526RGNIP    = 0x0200,     /**< QFN68 with 8Mb flash/GPU/PSRAM, no GND3 WB, add dummy Die. */
    PACKAGE_GR5526RGNI     = 0x0201,      /**< QFN68 with 8Mb flash. no GPU/PSRAM. */
    PACKAGE_GR5526RJNIP    = 0x0202,      /**< QFN68 with 16Mb flash/GPU/PSRAM, no GND3 WB, add dummy Die. */
} package_type_t;
/** @} */

/** @addtogroup GR55XX_SYS_STRUCTURES Structures
 * @{ */
/**@brief SDK version definition. */
typedef struct
{
    uint8_t  major;                         /**< Major version. */
    uint8_t  minor;                         /**< Minor version. */
    uint16_t build;                         /**< Build number. */
    uint32_t commit_id;                     /**< commit ID. */
}sdk_version_t;

/**@brief Assert callbacks.*/
typedef struct
{
    assert_err_cb_t   assert_err_cb;    /**< Assert error type callback. */
    assert_param_cb_t assert_param_cb;  /**< Assert parameter error type callback. */
    assert_warn_cb_t  assert_warn_cb;   /**< Assert warning type callback. */
}sys_assert_cb_t;

/**@brief RF trim parameter information definition. */
typedef struct
{
    int8_t  rssi_cali;    /**< RSSI calibration. */
    int8_t  tx_power;     /**< TX power. */
} rf_trim_info_t;

/**@brief Comparator trim information definition. */
typedef struct
{
    uint16_t slope_int_no1;
    uint16_t slope_int_no2;
} comp_trim_t;

/**@brief PMU trim parameter information definition. */
typedef struct
{
    uint8_t  io_ldo_bypass;    /**< IO LDO bypass */
    uint8_t  io_ldo_vout;      /**< IO LDO Vout. */
    uint8_t  dig_ldo_1p05;     /**< DIG LDO 1.05V.*/
    uint8_t  dig_ldo_0p9;      /**< DIG LDO 0.9V */
    uint8_t  dcdc_vout1p15;    /**< DCDC Vout 1.15V */
    uint8_t  dcdc_vout1p05;    /**< DCDC Vout 1.05V */
} pmu_trim_info_t;

/**@brief Warm boot timing parameters(unit: us). */
typedef struct
{
    uint16_t fast_ldo_prep; /**< Fast ldo prep. */
    uint16_t hf_osc_prep;   /**< HF Osc prep. */
    uint16_t dcdc_prep;     /**< DCDC prep. */
    uint16_t dig_ldo_prep;  /**< Dig ldo prep. */
    uint16_t xo_prep;       /**< XO prep. */
    uint16_t pll_prep;      /**< PLL prep. */
    uint16_t pll_lock;      /**< PLL lock. */
    uint16_t pwr_sw_prep;   /**< PWR sw prep. */
} boot_timing_params_t;

/** @} */

/** @addtogroup GR55XX_SYS_FUNCTIONS Functions
 * @{ */
/**
 *****************************************************************************************
 * @brief Output debug logs.
 *
 * @param[in] format: Pointer to the log information.
 *****************************************************************************************
 */
void sys_app_printf(const char *format, ...);

/**
 *****************************************************************************************
 * @brief Delay the function execution.
 *
 * @param[in] us:  Microsecond.
 *****************************************************************************************
 */
void delay_us(uint32_t us);

/**
 *****************************************************************************************
 * @brief Delay the function execution.
 *
 * @param[in] ms:  Millisecond.
 *****************************************************************************************
 */
void delay_ms(uint32_t ms);

/**
 *****************************************************************************************
 * @brief Memory allocation.
 *
 * @param[in] size:  Requested memory size.
 *
 * @return Valid memory location if successful, else null.
 *****************************************************************************************
 */
void *sys_malloc(uint32_t size);

/**
 *****************************************************************************************
 * @brief Free allocated memory.
 *
 * @param[in] p_mem: Pointer to memory block.
 *****************************************************************************************
 */
void sys_free(void *p_mem);

/**
 *****************************************************************************************
 * @brief Register signal handler.
 *
 * @note This function is mainly used to register the upper-layer APP callback functions to the protocol layer,
 *       which will be invoked when there are event responses in the protocol layer.
 *****************************************************************************************
 */
void sys_signal_handler_register(callback_t isr_handler);

/**
 *****************************************************************************************
 * @brief Get SDK version.
 *
 * @note This function is mainly used to get the version of SDK.
 *
 * @param[out] p_version: The pointer to struct of @ref sdk_version_t.
 *****************************************************************************************
 */
void sys_sdk_verison_get(sdk_version_t *p_version);

/**
 *****************************************************************************************
 * @brief Save system context.
 *
 * @note This function is used to save system context before the system goes to deep sleep.
 *       Boot codes will be used to restore system context in the wakeup procedure.
 *****************************************************************************************
 */
void sys_context_save(void);

/**
 *****************************************************************************************
 * @brief Load system context.
 *
 * @note This function is used to load system context after the system goes to deep sleep.
 *****************************************************************************************
 */
void restore_sys_context(void);

/**
 *****************************************************************************************
 * @brief Save system registers.
 *
 * @note This function is used to save system register before the system goes to deep sleep.
 *
 * @param[in] p_address: The pointer to register address.
 * @param[in] value: The register value to be saved, it will be restored when system wakes up.
 *****************************************************************************************
 */
void sys_regs_save(volatile uint32_t *p_address, uint32_t value);

/**
 *****************************************************************************************
 * @brief Generate checksum info for system context.
 *
 * @note This function is used to generate checksum for system context, it will be called
 *       before deep sleep in power management module.
 *****************************************************************************************
 */
void sys_context_checksum_gen(void);

/**
 *****************************************************************************************
 * @brief Register user-saved function.
 *
 * @note This function is used to register user-level saved function, which will be executed
 *       before deep sleep in power management module.
 *****************************************************************************************
 */
void sys_context_save_register(sys_context_func_t before, sys_context_func_t after, void *context_param);

/**
 *****************************************************************************************
 * @brief Encrypt and decrypt data using Present.
 *
 * @note  This function is only used to encrypt and decrypt data that needs to be stored in Flash.
 *
 * @param[in]  addr:   Operation address (Flash address minus Flash start address).
 * @param[in]  input:  Data before encryption and decryption.
 * @param[in]  size:   Data size.
 * @param[out] output: Data after encryption and decryption.
 *****************************************************************************************
 */
void sys_security_data_use_present(uint32_t addr, uint8_t *input, uint32_t size, uint8_t *output);

/**
 *****************************************************************************************
 * @brief Check the chip's security level.
 *
 * @retval 0 Security is not supported.
 * @retval 1 Security is supported.
 *****************************************************************************************
 */
uint32_t sys_security_enable_status_check(void);

/**
 *****************************************************************************************
 * @brief Calculate the HMAC code of the data.
 *
 * @param[in]  data:   Calculate the raw data of HMAC.
 * @param[in]  size:   Data size.
 * @param[out] hmac:   Calculate the result of HMAC.
 *****************************************************************************************
 */
uint8_t sys_security_calculate_hmac(void *data, uint32_t size, uint8_t *hmac);

/**
 *****************************************************************************************
 * @brief Get the RF trim information.
 *
 * @param[out] p_rf_trim: The pointer to struct of @ref rf_trim_info_t.
 * @retval 0  Operation is OK.
 * @retval 1  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_rf_trim_get(rf_trim_info_t *p_rf_trim);

/**
 *****************************************************************************************
 * @brief Get the ADC trim information.
 *
 * @param[out] p_adc_trim: The pointer to struct of adc_trim_info_t.
 * @retval 0  Operation is OK.
 * @retval 1  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_adc_trim_get(adc_trim_info_t *p_adc_trim);

/**
 *****************************************************************************************
 * @brief Get the Flash timing information.
 *
 * @param[out] p_flash_timing: The pointer to struct of exflash_timing_param_t.
 * @retval 0  Operation is OK.
 * @retval 1  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_flash_timing_get(exflash_timing_param_t *p_flash_timing);

/**
 *****************************************************************************************
 * @brief Get the copmparator trim information.
 *
 * @param[out] p_comp_trim: The pointer to struct of @ref comp_trim_t.
 * @retval  0  Operation is OK.
 * @retval  1  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_comp_trim_get(comp_trim_t *p_comp_trim);

/**
 *****************************************************************************************
 * @brief Get the PMU trim information.
 *
 * @param[out] p_pmu_trim: The pointer to struct of @ref pmu_trim_info_t.
 * @retval 0   Operation is OK.
 * @retval 1   the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_pmu_trim_get(pmu_trim_info_t *p_pmu_trim);

/**
 *****************************************************************************************
 * @brief Get the crystal trim information.
 *
 * @param[out] p_crystal_trim: offset information for crystal.
 * @retval 0   Operation is OK.
 * @retval 1   the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_crystal_trim_get(uint16_t *p_crystal_trim);

/**
 *****************************************************************************************
 * @brief Get the trim checksum.
 *
 * @param[out] p_trim_sum: The pointer to the buffer for trim checksum.
 * @retval 0   Operation is OK.
 * @retval 1   the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_trim_sum_get(uint16_t *p_trim_sum);

/**
 *****************************************************************************************
 * @brief Get the device address information.
 *
 * @param[out] p_device_addr: Bluetooth address by default.
 * @retval 0   Operation is OK.
 * @retval 1   the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_addr_get(uint8_t *p_device_addr);

/**
 *****************************************************************************************
 * @brief Get the device UID information.
 *
 * @param[out] p_device_uid: Device chip UID.
 * @retval 0   Operation is OK.
 * @retval 1   the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_uid_get(uint8_t *p_device_uid);

/**
 *****************************************************************************************
 * @brief Get the LP gain offset 2M information.
 *
 * @param[out] p_offset: the offset of LP gain.
 * @retval 0   Operation is OK.
 * @retval 1   the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_lp_gain_offset_2m_get(uint8_t *p_offset);

/**
 *****************************************************************************************
 * @brief Get the RAM size information.
 *
 * @param[out] p_sram_size: The pointer to enumeration of @ref sram_size_t.
 * @retval 0   Operation is OK.
 * @retval 1   the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_sram_get(sram_size_t *p_sram_size);

/**
 *****************************************************************************************
 * @brief Get the chip's package type.
 *
 * @param[out] p_package_type: The pointer to enumeration of @ref package_type_t.
 * @retval 0   Operation is OK.
 * @retval 1   the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_package_get(package_type_t *p_package_type);

/**
 *****************************************************************************************
 * @brief Set low power CLK frequency.
 *
 * @param[in] user_lpclk: CLK frequency.
 *****************************************************************************************
 */
void sys_lpclk_set(uint32_t user_lpclk);

/**
 ****************************************************************************************
 * @brief Convert a duration in μs into a duration in lp cycles.
 *
 * The function converts a duration in μs into a duration in lp cycles, according to the
 * low power clock frequency (32768Hz or 32000Hz).
 *
 * @param[in] us:    Duration in μs.
 *
 * @return Duration in lpcycles.
 ****************************************************************************************
 */
uint32_t sys_us_2_lpcycles(uint32_t us);

/**
 ****************************************************************************************
 * @brief Convert a duration in lp cycles into a duration in half μs.
 *
 * The function converts a duration in lp cycles into a duration in half μs, according to the
 * low power clock frequency (32768Hz or 32000Hz).
 * @param[in]     lpcycles:    Duration in lp cycles.
 * @param[in,out] error_corr:  Insert and retrieve error created by truncating the LP Cycle Time to a half μs (in half μs).
 *
 * @return Duration in half μs
 ****************************************************************************************
 */
uint32_t sys_lpcycles_2_hus(uint32_t lpcycles, uint32_t *error_corr);

/**
 *****************************************************************************************
 * @brief Set BLE Sleep HeartBeat Period.
 * @note  The BLE Sleep HeartBeat Period is used to Wakeup BLE Periodically when BLE is IDLE.
 *
 * @param[in] period_hus: The wake up duration of BLE when BLE is IDEL.
 *            Range 0x00000000-0xFFFFFFFF (in unit of μs).
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 *****************************************************************************************
 */
uint16_t sys_ble_heartbeat_period_set(uint32_t period_hus);


/**
 *****************************************************************************************
 * @brief Get BLE Sleep HeartBeat Period.
 * @note  The BLE Sleep HeartBeat Period is used to Wakeup BLE Periodically when BLE is IDLE.
 *
 * @param[in] p_period_hus: Pointer to the wake up duration.
 *            Range 0x00000000-0xFFFFFFFF (in unit of μs).
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 *****************************************************************************************
 */
uint16_t sys_ble_heartbeat_period_get(uint32_t* p_period_hus);

/**
 ****************************************************************************************
 * @brief Set system maximum usage ratio of message heap.
 *
 * The function will used to set message ratio of message heap.
 * Valid ratio range is 50 - 100 percent in full message size.
 *
 * @param[in]     usage_ratio:  Usage ratio of message heap size.
 *
 ****************************************************************************************
 */
void sys_max_msg_usage_ratio_set(uint8_t usage_ratio);

/**
 ****************************************************************************************
 * @brief Set system lld layer maximum usage ratio of message heap.
 *
 * The function will used to set message ratio of message heap.
 * Valid ratio range is 50 - 100 percent in full message size.
 *
 * @param[in]     usage_ratio:  Usage ratio of message heap size.
 *
 ****************************************************************************************
 */
void sys_lld_max_msg_usage_ratio_set(uint8_t usage_ratio);

/**
 ****************************************************************************************
 * @brief Get system message heap usage ratio.
 *
 * The function will used to get message ratio of message heap.
 * This ratio is heap used percent in full message size.
 *
 * @return current heap used percent.
 ****************************************************************************************
 */
uint8_t sys_msg_usage_ratio_get(void);

/**
 ****************************************************************************************
 * @brief Get system environment heap usage ratio.
 *
 * The function will used to get environment ratio of environment heap.
 * This ratio is heap used percent in full environment size.
 *
 * @return current heap used percent.
 ****************************************************************************************
 */
uint8_t sys_env_usage_ratio_get(void);

/**
 ****************************************************************************************
 * @brief Get system attriute database heap usage ratio.
 *
 * The function will used to get attriute database ratio of attriute database heap.
 * This ratio is heap used percent in full attriute database size.
 *
 * @return current heap used percent.
 ****************************************************************************************
 */
uint8_t sys_attdb_usage_ratio_get(void);

/**
 ****************************************************************************************
 * @brief Get system non retention heap usage ratio.
 *
 * The function will used to get non retention ratio of non retention heap.
 * This ratio is heap used percent in full non retention size.
 *
 * @return current heap used percent.
 ****************************************************************************************
 */
uint8_t sys_nonret_usage_ratio_get(void);

/**
 ****************************************************************************************
 * @brief Get low power CLK frequency.
 *
 * This function is used to get the low power clock frequency.
 *
 * @return Low power CLK frequency.
 ****************************************************************************************
 */
uint32_t sys_lpclk_get(void);

/**
 ****************************************************************************************
 * @brief Get low power CLK period.
 *
 * This function is used to get the low power CLK period.
 *
 * @return Low power CLK period.
 ****************************************************************************************
 */
uint32_t sys_lpper_get(void);

/**
 *****************************************************************************************
 * @brief Register assert callbacks.
 *
 * @param[in] p_assert_cb: Pointer to assert callbacks.
 *****************************************************************************************
 */
void sys_assert_cb_register(sys_assert_cb_t *p_assert_cb);

/**
 ****************************************************************************************
 * @brief Get status of ke_event list
 * @retval  true   ke_event not busy
 * @retval  false  ke_event busy.
 ****************************************************************************************
 */
bool sys_ke_sleep_check(void);

/**
 ****************************************************************************************
 * @brief Enable swd function
 ****************************************************************************************
 */
void sys_swd_enable(void);

/**
 ****************************************************************************************
 * @brief Diable swd function
 ****************************************************************************************
 */
void sys_swd_disable(void);

/**
 ****************************************************************************************
 * @brief Register the callback function of the extended llcp process
 *
 * @param[in]  conn_idx            Connect index.
 * @param[in]  interval            Connect interval (unit: 312.5 us)
 * @param[in]  latency             Connect latency (unit of connection event)
 * @param[in]  superv_to           Link supervision timeout (unit of 10 ms)
 *
 * @return     status             Error status
 ****************************************************************************************
 */
uint8_t sys_sdk_ultra_conn_update(uint8_t conn_idx, uint16_t interval, uint16_t latency, uint16_t superv_to);

/**
 ****************************************************************************************
 * @brief    Reverse byte order (32 bit).  For example, 0x12345678 becomes 0x78563412.
 * @returns  Reversed value
 ****************************************************************************************
 */
uint32_t sys_reverse_word(uint32_t value);

/**
 ****************************************************************************************
 * @brief    Reverse byte order (16 bit). For example, 0x1234 becomes 0x3412.
 * @returns  Reversed value
 ****************************************************************************************
 */
uint16_t sys_reverse_hword(uint16_t value);



/** @} */
#endif

/** @} */
/** @} */

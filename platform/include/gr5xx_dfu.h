/**
 ******************************************************************************
 *
 * @file gr5xx_dfu.h
 *
 * @brief Device Firmware Update API
 *
 ******************************************************************************
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
* @addtogroup SYSTEM
* @{
*/
/**
 @addtogroup DFU Device Firmware Update
 @{
 @brief Definitions and prototypes for the DFU interface.
*/

#ifndef __GR5XX_DFU_H__
#define __GR5XX_DFU_H__

#include <stdbool.h>
#include <stdint.h>
#include "custom_config.h"

/** @addtogroup DFU_DEFINES Defines
 * @{ */
#define DFU_IMG_INFO_PATTERN            0x4744      /**< The pattern of image info. */
/**@} */

/**@addtogroup DFU_STRUCTURES Structures
 * @{ */
#ifdef SOC_GR5515
/**@brief Boot information definition. */
typedef struct
{
    uint32_t bin_size;                /**< Firmware Size. */
    uint32_t check_sum;               /**< Firmware Check Sum Value. */
    uint32_t load_addr;               /**< Firmware Load Address. */
    uint32_t run_addr ;               /**< Firmware Run Address. */
    uint32_t xqspi_xip_cmd;           /**< XIP Read Mode. 0x03: Read mode, 0x0B: Fast Read mode, 0x3B: DualOut Fast Read mode, 0xBB: DualIO Fast Read mode, 0x6B: QuadOut Fast Read mode, 0xEB: QuadIO Fast Read mode */
    uint32_t xqspi_speed: 4;          /**< Bit: 0-3  clock speed. 0 :64 MHz, 1:48 MHz, 2:32 MHz, 3:24 MHz, 4:16 MHz. */
    uint32_t code_copy_mode: 1;       /**< Bit: 4 code copy mode. 0:XIP,1:QSPI. */
    uint32_t system_clk: 3;           /**< Bit: 5-7 system clock. 0:64 MHz, 1:48 MHz, 2:32 MHz(xo), 3:24 MHz, 4:16 MHz, 5:32 MHz(cpll). */
    uint32_t check_image:1;           /**< Bit: 8 check image. */
    uint32_t boot_delay:1;            /**< Bit: Boot delay flag. */
    uint32_t is_dap_boot:1;           /**< Bit: 11 check if boot dap mode. */
    uint32_t reserved:21;             /**< Bit: 24 reserved. */
} dfu_boot_info_t;

/**@brief DFU used functions config definition. */
typedef struct
{
    void (*dfu_ble_send_data)(uint8_t *p_data, uint16_t length);                                  /**< The function is used to send data to master by BLE. */
    void (*dfu_uart_send_data)(uint8_t *p_data, uint16_t length);                                 /**< The function is used to send data to master by UART. */
    uint32_t (*dfu_flash_read)(const uint32_t addr, uint8_t *p_buf, const uint32_t size);         /**< The function is used to read data from flash. */
    uint32_t (*dfu_flash_write)(const uint32_t addr, const uint8_t *p_buf, const uint32_t size);  /**< The function is used to write data to flash. */
    bool (*dfu_flash_erase)(const uint32_t addr, const uint32_t size);                            /**< The function is used to erase flash by address. */
    bool (*dfu_flash_erase_chip)(void);                                                           /**< The function is used to erase flash chip. */
    void (*dfu_flash_set_security)(bool enable);                                                  /**< The function is used to set the flash security mode as Enable or Disable. */
    bool (*dfu_flash_get_security)(void);                                                         /**< The function is used to get the flash security mode (Enable or Disable). */
    void (*dfu_flash_get_info)(uint32_t *id, uint32_t *size);                                     /**< The function is used to get the flash id and size. */
} dfu_func_t;

/**@brief SPI used functions config definition. */
typedef struct
{
    void (*dfu_spi_flash_init)(uint8_t *p_data);                                                   /**< The function is used to config flash spi. */
    uint32_t (*dfu_spi_flash_read)(uint32_t addr, uint8_t *buf, uint32_t size);                    /**< The function is used to read external flash . */
    uint32_t (*dfu_spi_flash_write)(uint32_t addr, uint8_t *buf, uint32_t size);                   /**< The function is used to write external flash. */
    bool (*dfu_spi_flash_erase)(uint32_t addr, uint32_t size);                                     /**< The function is used to erase external flash by address. */
    bool (*dfu_spi_flash_erase_chip)(void);                                                        /**< The function is used to erase exteral flash chip. */
    void (*dfu_spi_flash_get_info)(uint32_t *id, uint32_t *size);                                  /**< The function is used to get external flash id and size. */
}dfu_spi_flash_func_t;

/**@brief DFU program state callback definition. */
typedef struct
{
    void (*dfu_program_start_callback)(void);          /**<DFU program start callback. */
    void (*dfu_programing_callback)(uint8_t pro);      /**<DFU programing callback. */
    void (*dfu_program_end_callback)(uint8_t status);  /**<DFU program end callback. */
} dfu_pro_callback_t;

#endif

#if defined(SOC_GR5526) || defined(SOC_GR5525)
/**@brief Boot information definition. */
typedef struct
{
    uint32_t bin_size;                /**< Firmware Size. */
    uint32_t check_sum;               /**< Firmware Check Sum Value. */
    uint32_t load_addr;               /**< Firmware Load Address. */
    uint32_t run_addr ;               /**< Firmware Run Address. */
    uint32_t xqspi_xip_cmd;           /**< XIP Read Mode. 0x03: Read mode, 0x0B: Fast Read mode, 0x3B: DualOut Fast Read mode, 0xBB: DualIO Fast Read mode, 0x6B: QuadOut Fast Read mode, 0xEB: QuadIO Fast Read mode */
    uint32_t xqspi_speed: 4;          /**< Bit: 0-3  clock speed. 0 :64 MHz, 1:48 MHz, 2:32 MHz, 3:24 MHz, 4:16 MHz. */
    uint32_t code_copy_mode: 1;       /**< Bit: 4 code copy mode. 0:XIP,1:QSPI. */
    uint32_t system_clk: 3;           /**< Bit: 5-7 system clock. 0:64 MHz, 1:48 MHz, 2:32 MHz(xo), 3:24 MHz, 4:16 MHz, 5:32 MHz(cpll). */
    uint32_t check_image:1;           /**< Bit: 8 check image. */
    uint32_t boot_delay:1;            /**< Bit: Boot delay flag. */
    uint32_t is_dap_boot:1;           /**< Bit: 11 check if boot dap mode. */
    uint32_t reserved:21;             /**< Bit: 24 reserved. */
} dfu_boot_info_t;

/**@brief DFU used functions config definition. */
typedef struct
{
    void (*dfu_ble_send_data)(uint8_t *p_data, uint16_t length);                                  /**< The function is used to send data to master by BLE. */
    void (*dfu_uart_send_data)(uint8_t *p_data, uint16_t length);                                 /**< The function is used to send data to master by UART. */
    uint32_t (*dfu_flash_read)(const uint32_t addr, uint8_t *p_buf, const uint32_t size);         /**< The function is used to read data from flash. */
    uint32_t (*dfu_flash_write)(const uint32_t addr, const uint8_t *p_buf, const uint32_t size);  /**< The function is used to write data to flash. */
    bool (*dfu_flash_erase)(const uint32_t addr, const uint32_t size);                            /**< The function is used to erase flash by address. */
    bool (*dfu_flash_erase_chip)(void);                                                           /**< The function is used to erase flash chip. */
    void (*dfu_flash_set_security)(bool enable);                                                  /**< The function is used to set the flash security mode as Enable or Disable. */
    bool (*dfu_flash_get_security)(void);                                                         /**< The function is used to get the flash security mode (Enable or Disable). */
    void (*dfu_flash_get_info)(uint32_t *id, uint32_t *size);                                     /**< The function is used to get the flash id and size. */
    uint32_t (*dfu_flash_feat_enable)(uint32_t feature);
} dfu_func_t;

/**@brief SPI used functions config definition. */
typedef struct
{
    void (*dfu_spi_flash_init)(uint8_t *p_data);                                                   /**< The function is used to config flash spi. */
    uint32_t (*dfu_spi_flash_read)(uint32_t addr, uint8_t *buf, uint32_t size);                    /**< The function is used to read external flash . */
    uint32_t (*dfu_spi_flash_write)(uint32_t addr, uint8_t *buf, uint32_t size);                   /**< The function is used to write external flash. */
    bool (*dfu_spi_flash_erase)(uint32_t addr, uint32_t size);                                     /**< The function is used to erase external flash by address. */
    bool (*dfu_spi_flash_erase_chip)(void);                                                        /**< The function is used to erase exteral flash chip. */
    void (*dfu_spi_flash_get_info)(uint32_t *id, uint32_t *size);                                  /**< The function is used to get external flash id and size. */
}dfu_spi_flash_func_t;

/**@brief DFU used buffer config definition. */
typedef struct
{
    uint8_t  *rec_data_buffer;
    uint16_t  rec_data_buffer_size;
    uint8_t  *flash_op_buffer;
    uint16_t  flash_op_buffer_size;
}dfu_buffer_t;

/**@brief DFU program state callback definition. */
typedef struct
{
    void (*dfu_program_start_callback)(void);          /**<DFU program start callback. */
    void (*dfu_programing_callback)(uint8_t pro);      /**<DFU programing callback. */
    void (*dfu_program_end_callback)(uint8_t status);  /**<DFU program end callback. */
} dfu_pro_callback_t;

#endif

 
#ifdef SOC_GR5332
/**@brief Boot information definition. */
typedef struct
{   
    uint32_t bin_size;                /**< Firmware Size. */
    uint32_t check_sum;               /**< Firmware Check Sum Value. */
    uint32_t load_addr;               /**< Firmware Load Address. */
    uint32_t run_addr ;               /**< Firmware Run Address. */
    uint32_t xqspi_xip_cmd;           /**< XIP Read Mode. 0x03: Read mode, 0x0B: Fast Read mode, 0x3B: DualOut Fast Read mode, 0xBB: DualIO Fast Read mode, 0x6B: QuadOut Fast Read mode, 0xEB: QuadIO Fast Read mode */
    uint32_t xqspi_speed:4;           /**< bit: 0..3  clock speed */
    uint32_t code_copy_mode:1;        /**< bit: 4 code copy mode */
    uint32_t system_clk:3;            /**< bit: 5..7 system clock */
    uint32_t check_image:1;           /**< bit: 8 check image */
    uint32_t boot_delay:1;            /**< bit: 9 boot delay time */
    uint32_t reserved:22;             /**< bit: 22 reserved */
}dfu_boot_info_t;

/**@brief DFU used functions config definition. */
typedef struct
{
    void (*dfu_ble_send_data)(uint8_t *data, uint16_t size);
    void (*dfu_uart_send_data)(uint8_t *data, uint16_t size);

    uint32_t (*dfu_flash_read)(uint32_t addr, uint8_t *buf, uint32_t size);
    uint32_t (*dfu_flash_write)(uint32_t addr, uint8_t *buf, uint32_t size);
    uint32_t (*dfu_flash_erase)(uint32_t erase_type, uint32_t addr, uint32_t size);

    void     (*dfu_flash_get_info)(uint32_t *id, uint32_t *size);
    uint32_t (*dfu_flash_feat_enable)(uint32_t feature);
}dfu_func_t;

/**@brief SPI used functions config definition. */
typedef struct
{
    void (*dfu_spi_flash_init)(uint8_t *p_data);                                                   /**< The function is used to config flash spi. */
    uint32_t (*dfu_spi_flash_read)(uint32_t addr, uint8_t *buf, uint32_t size);                    /**< The function is used to read external flash . */
    uint32_t (*dfu_spi_flash_write)(uint32_t addr, uint8_t *buf, uint32_t size);                   /**< The function is used to write external flash. */
    bool (*dfu_spi_flash_erase)(uint32_t erase_type, uint32_t addr, uint32_t size);                /**< The function is used to erase external flash by address. */
    void (*dfu_spi_flash_get_info)(uint32_t *id, uint32_t *size);                                  /**< The function is used to get external flash id and size. */
    uint32_t (*dfu_flash_feat_enable)(uint32_t feature);                                           /**< The function is used to enable dfu flash feature. */
}dfu_spi_flash_func_t;

/**@brief DFU used buffers definition. */
typedef struct
{
    uint8_t   *frame_buf;   /**< DFU command frame buffer. */
    uint32_t   frame_size;  /**< DFU command frame buffer size. */
    uint8_t   *jlink_buf;   /**< DFU command jlink buffer. */
    uint32_t   jlink_size;  /**< DFU command jlink buffer size. */
    uint8_t   *cache_buf;   /**< DFU function call cache buffer. */
    uint32_t   cache_size;  /**< DFU function call cache buffer size. */
    uint8_t   *env_buf;     /**< DFU function call env buffer. */
    uint32_t   env_size;    /**< DFU function call env buffer size. */
} dfu_buf_table_t;

/**@brief DFU program state callback definition. */
typedef struct
{
    void (*dfu_program_start_callback)(void);          /**<DFU program start callback. */
    void (*dfu_programing_callback)(uint8_t pro);      /**<DFU programing callback. */
    void (*dfu_program_end_callback)(uint8_t status);  /**<DFU program end callback. */
    void (*dfu_flash_copy_process)(uint32_t addr);     /**<DFU flash copy callback. */
} dfu_pro_callback_t;

#endif

/**@brief DFU frame information definition. */
typedef struct
{
    uint16_t    cmd_type;   /**< CMD type. */
    uint16_t    data_len;   /**< Length of data. */
    uint8_t    *data;       /**< Pointer to data. */
    uint16_t    check_sum;  /**< Check sum. */
} dfu_receive_frame_t;

/**@brief Image information definition. */
typedef struct
{
    uint16_t        pattern;        /**< Image info pattern. */
    uint16_t        version;        /**< Image version. */
    dfu_boot_info_t boot_info;      /**< Image boot info. */
    uint8_t         comments[12];   /**< Image comments. */
} dfu_image_info_t;

/**@addtogroup DFU_TYPEDEF Typedefs
 * @{ */
/**@brief DFU CMD handler typed. */
typedef void(*dfu_receive_cmd_handler_t)(dfu_receive_frame_t *p_frame);
/** @} */


/** @addtogroup DFU_FUNCTIONS Functions
 * @{ */
#ifdef SOC_GR5515
/**
 *****************************************************************************************
 * @brief Function for initializing the DFU Used and Program State Callback.
 *
 * @note When APP wants to add DFU features, the flash_read and flash_write functions should be registered.
 *       To creat own DFU bootloaders, all functions in dfu_func_t should be registered. In order to show
 *       DFU program states by own DFU bootloaders, all functions in @ref dfu_pro_callback_t should be registered.
 *
 * @param[in]  p_app_dfu_func: DFU used functions.
 * @param[in]  p_dfu_buffer: DFU data receiving buffer.
 * @param[in]  p_dfu_callback: DFU program state callback functions.
 *****************************************************************************************
 */
void dfu_init(dfu_func_t *p_app_dfu_func, uint8_t* p_dfu_buffer, dfu_pro_callback_t *p_dfu_callback);
#endif

#if defined(SOC_GR5526) || defined(SOC_GR5525)
/**
 *****************************************************************************************
 * @brief Function for initializing the DFU Used and Program State Callback.
 *
 * @note When APP wants to add DFU features, the flash_read and flash_write functions should be registered.
 *       To creat own DFU bootloaders, all functions in dfu_func_t should be registered. In order to show
 *       DFU program states by own DFU bootloaders, all functions in @ref dfu_pro_callback_t should be registered.
 *
 * @param[in]  p_app_dfu_func: DFU used functions.
 * @param[in]  p_dfu_buffer: DFU data receiving buffer.
 * @param[in]  p_dfu_callback: DFU program state callback functions.
 *****************************************************************************************
 */
void dfu_init(dfu_func_t *p_app_dfu_func, dfu_buffer_t *p_dfu_buffer, dfu_pro_callback_t *p_dfu_callback);
#endif

#ifdef SOC_GR5332
/**
 *****************************************************************************************
 * @brief Function for initializing the DFU Used and Program State Callback.
 *
 * @note When APP wants to add DFU features, the flash_read and flash_write functions should be registered.
 *       To creat own DFU bootloaders, all functions in dfu_func_t should be registered. In order to show
 *       DFU program states by own DFU bootloaders, all functions in @ref dfu_pro_callback_t should be registered.
 *
 * @param[in]  p_app_dfu_func: DFU used functions.
 * @param[in]  p_dfu_buffer: DFU data receiving buffer.
 * @param[in]  p_callback: DFU program state callback functions.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
uint16_t dfu_init(dfu_func_t *p_app_dfu_func, dfu_buf_table_t* p_dfu_buffer, dfu_pro_callback_t *p_callback);
#endif


/**
 *****************************************************************************************
 * @brief Function for reset the DFU cmd parse state.
 *
 * @note When APP wants to add DFU timeout feature, shoulde reset DFU cmd parse state when timeout.
 *****************************************************************************************
 */
void dfu_cmd_parse_state_reset(void);

/**
 *****************************************************************************************
 * @brief Function for set DFU cmd handler.
 *
 * @note In Apps, if the user wants to jump to ROM DFU run,but want to be able to control the operation permissions of DFU,
 *       this function can be called.
 *
 * @param[in]  index:       The index of DFU cmd.
 * @param[in]  cmd:         The code of DFU cmd.
 * @param[in]  cmd_handler: The handler of DFU cmd.
 *****************************************************************************************
 */
void dfu_set_cmd_handler(uint8_t index, uint16_t cmd, dfu_receive_cmd_handler_t cmd_handler);

/**
 *****************************************************************************************
 * @brief Function for setting the BLE MTU size.
 *
 * @note If the user wants to create his or her own DFU bootloader, the DFU MTU size should be 
 *       set by using this function (when the MTU size is updated, this function should be called).
 *
 * @param[in]  mtu_size: The BLE MTU size.
 *****************************************************************************************
 */
void dfu_ble_set_mtu_size(uint16_t mtu_size);

/**
 *****************************************************************************************
 * @brief This function should be called when BLE stack sends data completely.
 *
 * @note This function should be used when the user wants to create his or her own DFU bootloader.
 *
 * @retval void
 *****************************************************************************************
 */
void dfu_ble_send_data_cmpl_process(void);

/**
 *****************************************************************************************
 * @brief This function should be called when BLE receives data.
 *
 * @note This function should be used when the user wants to create his or her own DFU bootloader.
 *
 * @param[in]  p_data: The received data from BLE.
 * @param[in]  length: The length of data.
 *****************************************************************************************
 */
void dfu_ble_receive_data_process(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief This function should be called when UART receives data.
 *
 * @note This function should be used when the user wants to create his or her own DFU bootloader.
 *
 * @param[in]  p_data: The received data from UART
 * @param[in]  length: The length of data
 *****************************************************************************************
 */
void dfu_uart_receive_data_process(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Function for initializing the DFU SPI Flash Callback.
 *
 * @note When APP wants to add DFU operationally external flash feature, the dfu_spi_flash_func_t should be registered.
 *
 * @param[in]  spi_flash_func: DFU operationally external flash used functions.
 *****************************************************************************************
 */
void dfu_spi_flash_func_config(dfu_spi_flash_func_t *spi_flash_func);

/** @} */
#endif
/** @} */
/** @} */

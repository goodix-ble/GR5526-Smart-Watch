/**
 ****************************************************************************************
 *
 * @file    app_qspi.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of QSPI app library.
 *
 ****************************************************************************************
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
 ****************************************************************************************
 */

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_QSPI QSPI
  * @brief QSPI APP module driver.
  * @{
  */


#ifndef _APP_QSPI_H_
#define _APP_QSPI_H_

#include "app_drv_config.h"
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
#include "gr533x_hal.h"
#else
#include "gr55xx_hal.h"
#endif

#include "app_io.h"
#include "app_dma.h"
#include "app_drv_error.h"
#include "app_drv_config.h"
#include "stdbool.h"
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
#include "app_qspi_user_config.h"
#endif
#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_QSPI_MODULE_ENABLED

/**
  * @defgroup  APP_QSPI_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup APP_QSPI_Exported_Constants QSPI Exported Constants
  * @{
  */

/** @addtogroup APP_QSPI_PIN Qspi pin defines
  * @{
  */
#define APP_QSPI_PIN_ENABLE                     1    /**< QSPI pin enable  */
#define APP_QSPI_PIN_DISABLE                    0    /**< QSPI pin disable */
/** @} */

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
/** @defgroup APP_MAX_XFER MAX_XREF Defines
  * @{
  */
#define QSPI_MAX_XFER_SIZE_ONCE                 (0xFFFCu)            /**< max xfer beat in every qspi xfer */
#define DMA_MAX_XFER_SIZE_ONCE                  (4095u)              /**< max xfer beat in every dma xfer  */
/** @} */

//#define APP_STORAGE_RAM_ID    0xf                  /**< Special ID to handle RAM Source */
#endif
/** @} */

/** @} */


/** @addtogroup APP_QSPI_ENUM Enumerations
  * @{
  */

/**
  * @brief QSPI module Enumerations definition
  */
typedef enum
{
    APP_QSPI_ID_0,              /**< QSPI module 0 */
    APP_QSPI_ID_1,              /**< QSPI module 1 */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    APP_QSPI_ID_2,              /**< QSPI module 2 */
#endif
    APP_QSPI_ID_MAX,             /**< Only for check parameter, not used as input parameters. */

    APP_STORAGE_RAM_ID = 0xf,
} app_qspi_id_t;

/**
* @brief APP QSPI Event Type
*/
typedef enum
{
    APP_QSPI_EVT_ERROR,          /**< Error reported by QSPI peripheral. */
    APP_QSPI_EVT_TX_CPLT,        /**< Requested TX transfer completed.   */
    APP_QSPI_EVT_RX_DATA,        /**< Requested RX transfer completed.   */
    APP_QSPI_EVT_ABORT,          /**< Abort reported by QSPI peripheral. */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    APP_QSPI_EVT_ASYNC_WR_SCRN_CPLT,    /**< ASYNC-Write Quad Screen Complete */
    APP_QSPI_EVT_ASYNC_WR_SCRN_FAIL,    /**< ASYNC-Write Quad Screen Fail */
#endif
} app_qspi_evt_type_t;

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
/**
  * @brief QSPI Memory-Mapped Endian Mode
  */
typedef enum {
    APP_QSPI_MMAP_ENDIAN_MODE_0 = 0,    /**< MMAP Endian Mode 0 */
    APP_QSPI_MMAP_ENDIAN_MODE_1,        /**< MMAP Endian Mode 1 */
    APP_QSPI_MMAP_ENDIAN_MODE_2,        /**< MMAP Endian Mode 2 */
} app_qspi_mmap_endian_mode_e;

/**
  * @brief QSPI Device supporting Memory-Mapped Mode
  */
typedef enum {
    APP_QSPI_DEVICE_UNSET = 0,            /**< UnSet Device */
    APP_QSPI_DEVICE_FLASH = 1,            /**< Flash Device */
    APP_QSPI_DEVICE_PSRAM = 2,            /**< PSRAM Device */
} app_qspi_device_e;

/**
  * @brief Define PSRAM's Read Command for Memory-Mapped Mode
  */
typedef enum {
    PSRAM_MMAP_CMD_QREAD_0BH         = 0x00,    /**< 0BH in Quad */
    PSRAM_MMAP_CMD_QREAD_EBH         = 0x01,    /**< EBH in Quad */
    PSRAM_MMAP_CMD_READ_MAX
} app_qspi_psram_mmap_rd_cmd_e;

/**
  * @brief Define PSRAM's Write Command for Memory-Mapped Mode
  */
typedef enum {
    PSRAM_MMAP_CMD_QWRITE_02H        = 0x00,    /**< 02H in Quad */
    PSRAM_MMAP_CMD_QWRITE_38H        = 0x01,    /**< 38H in Quad */
    PSRAM_MMAP_CMD_WRITE_MAX
} app_qspi_psram_mmap_wr_cmd_e;

/**
  * @brief Define Flash's Read Command for Memory-Mapped Mode
  */
typedef enum {
    FLASH_MMAP_CMD_DREAD_3BH         = 0x00,    /**< 3BH in Dual */
    FLASH_MMAP_CMD_2READ_BBH         = 0x01,    /**< BBH in 2Read */
    FLASH_MMAP_CMD_2READ_BBH_SIOO    = 0x02,    /**< BBH in 2Read with SIOO mode */

    FLASH_MMAP_CMD_QREAD_6BH         = 0x03,    /**< 6BH in QRead */
    FLASH_MMAP_CMD_4READ_EBH         = 0x04,    /**< EBH in 4Read */
    FLASH_MMAP_CMD_4READ_EBH_SIOO    = 0x05,    /**< EBH in 4Read with SIOO mode */
    FLASH_MMAP_CMD_READ_MAX,
} app_qspi_flash_mmap_rd_cmd_e;

/**
* @brief APP QSPI interface type when drawing screen
*/
typedef enum {
    DRAW_TYPE_IF_NONE = 0x00,
    DRAW_TYPE_IF_DUAL_SCREEN,
    DRAW_TYPE_IF_VERI_LINKED_SCREEN,
} app_qspi_draw_screen_interface_type_e;

/**
  * @brief Define Pin Groups for QSPI
  */
typedef enum {

    QSPI0_PIN_GROUP_0 = 0x00,            /**< QSPI0 Pinmux Group  */
                                                /* CS  : GPIO_26.MUX0 */
                                                /* CLK : GPIO_21.MUX0 */
                                                /* IO0 : GPIO_22.MUX0 */
                                                /* IO1 : GPIO_23.MUX0 */
                                                /* IO2 : GPIO_24.MUX0 */
                                                /* IO3 : GPIO_25.MUX0 */

    QSPI1_PIN_GROUP_0 = 0x01,            /**< QSPI1 Pinmux Group 0 */
                                                /* CS  : GPIO_10.MUX0 */
                                                /* CLK : GPIO_15.MUX0 */
                                                /* IO0 : GPIO_11.MUX0 */
                                                /* IO1 : GPIO_12.MUX0 */
                                                /* IO2 : GPIO_13.MUX0 */
                                                /* IO3 : GPIO_14.MUX0 */

    QSPI2_PIN_GROUP_0 = 0x02,            /**< QSPI2 Pinmux Group 0 */
                                                /* CS  : GPIO_27.MUX0 */
                                                /* CLK : GPIO_16.MUX0 */
                                                /* IO0 : GPIO_17.MUX0 */
                                                /* IO1 : GPIO_18.MUX0 */
                                                /* IO2 : GPIO_19.MUX0 */
                                                /* IO3 : GPIO_20.MUX0 */
    QSPIx_PIN_GROUP_MAX,
} qspi_pins_group_e;

/**
  * @brief Define BLIT Mode By DMA
  */
typedef enum {
    BLIT_BY_DMA_SG  = 0,             /**< need enable BLIT_IMAGE_FEATURE_SUPPORT */
    BLIT_BY_DMA_LLP = 1,             /**< need enable PSRAM_LLP_FEATURE_SUPPORT */
} blit_xfer_type_e;

/** @} */

/** @addtogroup APP_QSPI_STRUCTURES Structures
  * @{
  */

/**
  * @brief QSPI memory-mapped configuration Structures
  */
typedef struct {
    app_qspi_device_e                   dev_type;           /**< Specifies the device type of QSPI */
    app_qspi_psram_mmap_wr_cmd_e        psram_wr;           /**< Specifies the write command for PSRAM, Flash does not support this */
    union {
        app_qspi_flash_mmap_rd_cmd_e    flash_rd;           /**< Read command for flash device */
        app_qspi_psram_mmap_rd_cmd_e    psram_rd;           /**< Read command for psram device */
    } rd;                                                   /**< Specifies read command by real device */
    void * set;                                             /**< Reserved */
} app_qspi_mmap_device_t;
#endif

/**
  * @brief QSPI IO configuration Structures
  */
typedef struct
{
    app_io_type_t  type;         /**< Specifies the type of QSPI IO. */
    app_io_mux_t   mux;          /**< Specifies the Peripheral to be connected to the selected pins. */
    uint32_t       pin;          /**< Specifies the IO pins to be configured.
                                      This parameter can be any value of @ref GR5xxx_pins. */
    app_io_mode_t  mode;         /**< Specifies the IO Mode for pins. */
    app_io_pull_t  pull;         /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
    uint8_t        enable;       /**< Enable or disable the pin. */
} app_qspi_pin_t;

/**
  * @brief QSPI configuration Structures
  */
typedef struct
{
    app_qspi_pin_t cs;           /**< Set the configuration of QSPI CS pin. */
    app_qspi_pin_t clk;          /**< Set the configuration of QSPI CLK pin. */
    app_qspi_pin_t io_0;         /**< Set the configuration of QSPI IO0 pin. */
    app_qspi_pin_t io_1;         /**< Set the configuration of QSPI IO1 pin. */
    app_qspi_pin_t io_2;         /**< Set the configuration of QSPI IO2 pin. */
    app_qspi_pin_t io_3;         /**< Set the configuration of QSPI IO3 pin. */
} app_qspi_pin_cfg_t;

/**
  * @brief QSPI DMA configuration structure definition
  */
typedef struct
{
    dma_regs_t *                dma_instance;        /**< Specifies the dma instance of QSPI.   */
    dma_channel_t               dma_channel;         /**< Specifies the dma channel of QSPI.    */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    uint32_t                    wait_timeout_ms;     /**< Specifies timeout time of polling and dead wait, ms. */
    uint32_t                    extend;              /**< Specifies extend segment, to use */
#endif
} app_qspi_dma_cfg_t;
/** @} */

/** @addtogroup APP_QSPI_TYPEDEFS Type Definitions
  * @{
  */

/**
  * @brief QSPI command structure definition
  */
typedef qspi_command_t app_qspi_command_t;
/** @} */

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
/** @addtogroup APP_QSPI_STRUCTURES Structures
  * @{
  */
typedef struct
{
    uint32_t instruction;               /**< Specifies the Instruction to be sent.
                                             This parameter can be a value (8-bit) between 0x00 and 0xFF. */

    uint32_t leading_address;           /**< Specifies the Address to be sent (Size from 1 to 4 bytes according AddressSize).
                                             This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF. */

    uint32_t ongoing_address;           /**< Specifies the Address to be sent (Size from 1 to 4 bytes according AddressSize).
                                             This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF. */

    uint32_t instruction_size;          /**< Specifies the Instruction Size.
                                             This parameter can be a value of @ref QSPI_Instruction_Size. */

    uint32_t address_size;              /**< Specifies the Address Size.
                                             This parameter can be a value of @ref QSPI_Address_Size. */

    uint32_t dummy_cycles;              /**< Specifies the Number of Dummy Cycles.
                                             This parameter can be a number between 0 and 31. */

    uint32_t data_size;                /**< Specifies the QSPI address width.
                                             This parameter can be a value of @ref QSPI_Data_Size. */

    uint32_t instruction_address_mode;  /**< Specifies the Instruction and Address Mode.
                                             This parameter can be a value of @ref QSPI_Inst_Addr_Mode. */

    uint32_t data_mode;                 /**< Specifies the Data Mode (used for dummy cycles and data phases).
                                             This parameter can be a value of @ref QSPI_Data_Mode. */

    bool     is_one_take_cs;            /**< Sent all data in one take CS or in multiple independent CS */

} app_qspi_screen_command_t;
/** @} */
#endif

/** @addtogroup APP_QSPI_STRUCTURES Structures
  * @{
  */
/**
  * @brief QSPI event structure definition
  */
typedef struct
{
    app_qspi_evt_type_t type;    /**< Type of event. */
    union
    {
        uint32_t error_code;     /**< QSPI Error code . */
        uint16_t size;           /**< QSPI transmitted/received counter. */
    } data;                      /**< Event data. */
} app_qspi_evt_t;

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
/**
  * @brief Screen Info. structure definition
  */
typedef struct {
    unsigned int scrn_pixel_stride;              /**< screen pixel stride, such as 390 */
    unsigned int scrn_pixel_width;               /**< screen pixel width, such as 390, width must be less or equal than stride */
    unsigned int scrn_pixel_height;              /**< screen pixel height, such as 390 */
    unsigned int scrn_pixel_depth;               /**< pixel depth, unit: byte, such as 2 */
} app_qspi_screen_info_t;


/**
  * @brief Scroll-Screen structure definition
  */
typedef struct {
    uint32_t    first_frame_start_address;          /**< if scroll the screen horizontally : first stands for left image, second stands for right image */
    uint32_t    second_frame_start_address;         /**< if scroll the screen vertically : first stands for up image, second stands for down image */
    uint32_t    scroll_coordinate;                  /**< scroll to where , must be even number, belongs to [0, SCREEN_PIXEL_WIDTH] */
    bool        is_horizontal_scroll;               /**< true : screen horizontally ; false : screen vertically */
} app_qspi_screen_scroll_t;

/**
  * @brief Vertical Scroll-Screen Linked List Structure
  */
typedef struct _screen_veri_link_scroll_t {
    uint32_t    frame_ahb_start_address;            /**< Start AHB address for current frame */
    uint32_t    frame_offset_lines;                 /**< Offset lines for current frame */
    uint32_t    frame_draw_lines;                   /**< Drawing lines (start from offset line) for current frame */
    struct _screen_veri_link_scroll_t * next;       /**< Linked to next node, if empty, Set NULL */
} app_qspi_screen_veri_link_scroll_t;

/**
  * @brief One block of screen Structure
  */
typedef struct {
    uint32_t    frame_ahb_start_address;            /**< Start AHB address for current block */
    uint32_t    frame_offset_lines;                 /**< Offset lines for current block */
    uint32_t    frame_draw_lines;                   /**< Drawing lines (start from offset line) for current block */
} app_qspi_screen_block_t;

/**
  * @brief Dual Screen (HOR/VER) Scroll Structure, just used for inner driver
  */
typedef struct {
    app_qspi_screen_scroll_t    scroll_config;          /**< Start AHB address for current frame */
    uint32_t                    image_1_ahb_address;    /**< Record first frame starting AHB address */
    uint32_t                    image_2_ahb_address;    /**< Record second frame starting AHB address */
    uint32_t                    this_send_lines;        /**< Record current sent lines */
    uint32_t                    total_sent_lines;       /**< Record total sent lines already */
    uint32_t                    sent_line_order;        /**< Record current line order */
} dual_screen_scroll_t;

/**
  * @brief Vertical Linked List Screen-Scroll Structure, just used for inner driver
  */
typedef struct {
    app_qspi_screen_veri_link_scroll_t   vl_scroll;         /**< Start AHB address for current frame */
    app_qspi_screen_veri_link_scroll_t * p_cur_scroll;      /**< Record current linked node */
    uint32_t                             total_sent_lines;  /**< Record total sent lines */
} veri_linked_screen_scroll_t;

/**
  * @brief Async Draw Screen Structure, just used for inner driver
  */
typedef struct {
    app_qspi_draw_screen_interface_type_e   if_type;                    /**< drawing interface type */
    app_qspi_id_t                           screen_id;                  /**< Specify the Screen ID */
    app_qspi_id_t                           storage_id;                 /**< Specify the Memory ID */
    uint32_t                                llp_cfg_right_shift_bit;    /**< Record the llp shift bit configure */
    uint32_t                                llp_cfg_ctrl_low;           /**< Record the llp control bits configure */
    app_qspi_screen_info_t                  screen_info;                /**< Record the basic screen info */
    app_qspi_screen_command_t               qspi_screen_command;        /**< Record the basic screen command */
    union {
        dual_screen_scroll_t                dual_ss;                    /**< info for dual screen-scroll */
        veri_linked_screen_scroll_t         veri_linked_ss;             /**< info for veri linked screen-scroll */
    } ss;                                                               /**< union record */
} app_qspi_async_draw_screen_info_t;


/**
  * @brief Blit Image structure definition
  */
typedef struct {
    uint32_t src_img_address;           /**< source address, just offset in storage device */
    uint32_t src_img_w;                 /**< pixel width of source image */
    uint32_t src_img_h;                 /**< pixel height of source image */
    uint32_t src_img_x;                 /**< x-coordinate of source image, left-top point is Coordinate origin */
    uint32_t src_img_x_delta;           /**< blit image width in pixel, do not multiple pixel depth */
    uint32_t src_img_y;                 /**< y-coordinate of source image, left-top point is Coordinate origin */
    uint32_t src_img_y_delta;           /**< blit image height in pixel, do not multiple pixel depth */
    uint32_t dst_buff_address;          /**< destination buffer address */
    uint32_t dst_buff_width;            /**< pixel width of destination buffer */
    uint32_t dst_buff_height;           /**< pixel height of destination buffer */
    uint32_t dst_buff_x;                /**< x-coordinate of destination buffer, left-top point is Coordinate origin */
    uint32_t dst_buff_y;                /**< y-coordinate of destination buffer, left-top point is Coordinate origin */
    uint32_t pixel_depth;               /**< pixel depth in byte */
} blit_image_config_t;
#endif
/** @} */

/** @addtogroup APP_QSPI_TYPEDEFS Type Definitions
  * @{
  */

/**
  * @brief QSPI event callback definition
  */
typedef void (*app_qspi_evt_handler_t)(app_qspi_evt_t *p_evt);
/** @} */

/** @addtogroup APP_QSPI_ENUM Enumerations
  * @{
  */
/**@brief App qspi state types. */
typedef enum
{
    APP_QSPI_INVALID = 0,
    APP_QSPI_ACTIVITY,
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    APP_QSPI_SLEEP,
#endif
} app_qspi_state_t;

/**@brief App qspi dma_state types. */
typedef enum
{
    APP_QSPI_DMA_INVALID = 0,
    APP_QSPI_DMA_ACTIVITY,
} app_qspi_dma_state_t;

/** @} */

/** @addtogroup APP_QSPI_STRUCTURES Structures
  * @{
  */

/**
  * @brief QSPI device structure definition
  */
typedef struct {
    qspi_handle_t           handle;                             /**< QSPI handle Structure. */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    app_qspi_dma_cfg_t      dma_cfg;                            /**< QSPI DMA configuration structure. */
#endif
    app_qspi_pin_cfg_t      *p_pin_cfg;                         /**< QSPI configuration Structures. */
    dma_id_t                dma_id;                             /**< DMA id. */
    app_qspi_state_t        qspi_state;                         /**< App qspi state types. */
    app_qspi_dma_state_t    qspi_dma_state;                     /**< App qspi dma_state types. */
    volatile bool           start_flag;                         /**< start flag. */
    app_qspi_evt_handler_t  evt_handler;                        /**< QSPI event callback. */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    app_qspi_mmap_device_t  mounted_mmap_device;                /**< QSPI memory-mapped configuration Structures. */

    volatile uint8_t        mmap_endian_mode;                   /**< mmap endian mode. */
    volatile bool           is_mmap_inited;                     /**< mmap inited. */
    volatile bool           is_mmap_prefetch_en;                /**< mmap prefetch. */
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    volatile bool           is_used_dma;                        /**< used dma. */
    volatile bool           is_dma_mode_m2m;                    /**< dma mode. */
#endif
    volatile bool           is_rx_done;                         /**< rx done. */
    volatile bool           is_tx_done;                         /**< tx done. */
    volatile bool           is_xfer_err;                        /**< xfer err. */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    volatile bool           is_dma_done;                        /**< dma done. */
    volatile bool           is_async_write_screen;              /**< async write screen. */
#endif
}qspi_env_t;

/**
  * @brief QSPI parameters structure definition
  */
typedef struct
{
    app_qspi_id_t      id;       /**< specified QSPI module ID. */
    app_qspi_pin_cfg_t pin_cfg;  /**< the pin configuration information for the specified QSPI module. */
    app_qspi_dma_cfg_t dma_cfg;  /**< QSPI DMA configuration. */
    qspi_init_t        init;     /**< QSPI communication parameters. */
    qspi_env_t         qspi_env;  /**< QSPI device structure definition. */
} app_qspi_params_t;
/** @} */


#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
/** @addtogroup HAL_APP_QSPI_DRIVER_CONSTANTS Constants
  * @{
  */

/**
  * @brief The Reference to pin groups define
  */
extern const app_qspi_pin_cfg_t g_qspi_pin_groups[QSPIx_PIN_GROUP_MAX];

/** @} */
#endif


/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_QSPI_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP QSPI DRIVER according to the specified parameters
 *         in the app_qspi_params_t and app_qspi_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_qspi_params_t parameter which contains the
 *                       configuration information for the specified QSPI module.
 * @param[in]  evt_handler: QSPI user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_qspi_init(app_qspi_params_t *p_params, app_qspi_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP QSPI DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_qspi_deinit(app_qspi_id_t id);

/**
 ****************************************************************************************
 * @brief  Abort qspi communication with Interrupt.
 *
 * @param[in]  id: QSPI module ID.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_abort(app_qspi_id_t id);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
/**
 ****************************************************************************************
 * @brief  Config the memory mapped mode (also called XIP mode) and Active the mode
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  dev: device config for mmapped mode
 *
 * @return true/false
 ****************************************************************************************
 */
bool app_qspi_config_memory_mappped(app_qspi_id_t id, app_qspi_mmap_device_t dev);

/**
 ****************************************************************************************
 * @brief  Active or Deactive memory mapped mode (also called XIP mode)
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  is_active: true - Active the mode; false - Deactive the mode
 *
 * @return true/false
 ****************************************************************************************
 */
bool app_qspi_active_memory_mappped(app_qspi_id_t id, bool is_active);
#endif

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction, address and dummy cycles in blocking mode.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  id: which QSPI module want to receive.
 * @param[in]  p_cmd: Pointer to a app_qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_command_receive_sync(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction, address and dummy cycles in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  id: which QSPI module want to receive.
 * @param[in]  p_cmd: Pointer to a app_qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_command_receive_async(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction, address and dummy cycles in blocking mode.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  id: which QSPI module want to transmit.
 * @param[in]  p_cmd: Pointer to a app_qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_command_transmit_sync(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction, address and dummy cycles in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  id: which QSPI module want to transmit.
 * @param[in]  p_cmd: Pointer to a app_qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_command_transmit_async(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Transmit only instruction in blocking mode.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  id: which QSPI module want to transmit command.
 * @param[in]  p_cmd: Pointer to a app_qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_command_sync(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit instruction in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  id: which QSPI module want to transmit command.
 * @param[in]  p_cmd: Pointer to a app_qspi_command_t structure that contains the instruction and address for data transfer.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_command_async(app_qspi_id_t id, app_qspi_command_t *p_cmd);

/**
 ****************************************************************************************
 * @brief  Transmit data without command, support std/dual/quad mode
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  qspi_mode : @ref QSPI_DATA_MODE_SPI
 *						   @ref QSPI_DATA_MODE_DUALSPI
 *						   @ref QSPI_DATA_MODE_QUADSPI
 * @param[in]  data_width :@ref QSPI_DATASIZE_08_BITS
 *						   @ref QSPI_DATASIZE_16_BITS
 *						   @ref QSPI_DATASIZE_32_BITS
 * @param[in]  p_data : data Pointer to transmit
 * @param[in]  length : byte length of data
 * @param[in]  timeout: Timeout duration
 *
 * @return true/false
 ****************************************************************************************
 */
uint16_t app_qspi_transmit_sync_ex(app_qspi_id_t id, uint32_t qspi_mode, uint32_t data_width, uint8_t *p_data, uint32_t length, uint32_t timeout);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in blocking mode with standard SPI.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  id: which QSPI module want to transmit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_transmit_sync(app_qspi_id_t id, uint8_t *p_data, uint32_t length, uint32_t timeout);
#endif

/**
 ****************************************************************************************
 * @brief  Transmit data without command, support std/dual/quad mode
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  qspi_mode : @ref QSPI_DATA_MODE_SPI
 *						   @ref QSPI_DATA_MODE_DUALSPI
 *						   @ref QSPI_DATA_MODE_QUADSPI
 * @param[in]  data_width :@ref QSPI_DATASIZE_08_BITS
 *						   @ref QSPI_DATASIZE_16_BITS
 *						   @ref QSPI_DATASIZE_32_BITS
 * @param[in]  p_data : data Pointer to transmit
 * @param[in]  length : byte length of data
 *
 * @return true/false
 ****************************************************************************************
 */
uint16_t app_qspi_transmit_async_ex(app_qspi_id_t id, uint32_t qspi_mode, uint32_t data_width, uint8_t *p_data, uint32_t length);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
/**
 ****************************************************************************************
 * @brief Transmit an amount of data in non-blocking mode at standard SPI with Interrupt.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  id: which QSPI module want to transmit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_transmit_async(app_qspi_id_t id, uint8_t *p_data, uint32_t length);
#endif

/**
 ****************************************************************************************
 * @brief  Receive data without command, support std/dual/quad mode
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  qspi_mode : @ref QSPI_DATA_MODE_SPI
 *                         @ref QSPI_DATA_MODE_DUALSPI
 *                         @ref QSPI_DATA_MODE_QUADSPI
 * @param[in]  data_width :@ref QSPI_DATASIZE_08_BITS
 *                         @ref QSPI_DATASIZE_16_BITS
 *                         @ref QSPI_DATASIZE_32_BITS
 * @param[in]  p_data : data Pointer to transmit
 * @param[in]  length : byte length of data
 * @param[in]  timeout: Timeout duration
 *
 * @return true/false
 ****************************************************************************************
 */
uint16_t app_qspi_receive_sync_ex(app_qspi_id_t id, uint32_t qspi_mode, uint32_t data_width, uint8_t *p_data, uint32_t length, uint32_t timeout);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
/**
 ****************************************************************************************
 * @brief  Receive an amount of data in blocking mode with standard SPI.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  id: which QSPI module want to receive.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be received in bytes
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_receive_sync(app_qspi_id_t id, uint8_t *p_data, uint32_t length, uint32_t timeout);
#endif

/**
 ****************************************************************************************
 * @brief  Receive data without command, support std/dual/quad mode
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  qspi_mode : @ref QSPI_DATA_MODE_SPI
 *                         @ref QSPI_DATA_MODE_DUALSPI
 *                         @ref QSPI_DATA_MODE_QUADSPI
 * @param[in]  data_width :@ref QSPI_DATASIZE_08_BITS
 *                         @ref QSPI_DATASIZE_16_BITS
 *                         @ref QSPI_DATASIZE_32_BITS
 * @param[in]  p_data : data Pointer to transmit
 * @param[in]  length : byte length of data
 *
 * @return true/false
 ****************************************************************************************
 */
uint16_t app_qspi_receive_async_ex(app_qspi_id_t id, uint32_t qspi_mode, uint32_t data_width, uint8_t *p_data, uint32_t length);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
/**
 ****************************************************************************************
 * @brief Receive an amount of data in non-blocking mode at standard SPI with Interrupt.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  id: which QSPI module want to receive.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be received in bytes
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_receive_async(app_qspi_id_t id, uint8_t *p_data, uint32_t length);
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
/**
 ****************************************************************************************
 * @brief  Set Data Endian Mode to Read in Memory mapped Mode(XIP Mode)
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  mode : endian mode
 * @return true/false
 ****************************************************************************************
 */
bool app_qspi_mmap_set_endian_mode(app_qspi_id_t id, app_qspi_mmap_endian_mode_e mode);

/**
 ****************************************************************************************
 * @brief  Read U8 Data in Memory mapped Mode(XIP Mode), can be used in flash/psram device
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  address : the address of device connected to QSPI, start from 0x000000
 * @return the read data
 ****************************************************************************************
 */
uint8_t app_qspi_mmap_read_u8(app_qspi_id_t id, uint32_t address) ;

/**
 ****************************************************************************************
 * @brief  Read U16 Data in Memory mapped Mode(XIP Mode), The Data is ordered by the order in flash/psram device
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  address : the address of device connected to QSPI, start from 0x000000
 * @return the read data
 ****************************************************************************************
 */
uint16_t app_qspi_mmap_read_u16(app_qspi_id_t id, uint32_t address);

/**
 ****************************************************************************************
 * @brief  Read U32 Data in Memory mapped Mode(XIP Mode), The Data is ordered by the order in flash/psram device
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  address : the address of device connected to QSPI, start from 0x000000
 * @return the read data
 ****************************************************************************************
 */
uint32_t app_qspi_mmap_read_u32(app_qspi_id_t id, uint32_t address);

/**
 ****************************************************************************************
 * @brief  Read data block in Memory mapped Mode(XIP Mode), The Data is ordered by the order in flash/psram device
 *
 * @param[in]  id : QSPI module ID.
 * @param[in]  address : the address of device connected to QSPI, start from 0x000000
 * @param[in]  buffer  : memory pointer to save the read  data
 * @param[in]  length  : the read length in byte
 * @return true/false
 ****************************************************************************************
 */
bool app_qspi_mmap_read_block(app_qspi_id_t id, uint32_t address, uint8_t * buffer, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Return the XIP Base Address of QSPI Instance.
 *
 * @param[in] id: QSPI module ID.
 *
 * @return  The XIP base address.
 ****************************************************************************************
 */
uint32_t app_qspi_get_xip_base_address(app_qspi_id_t id);

#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
/**
 ****************************************************************************************
 * @brief Transmit an amount of data in QPI mode (Async Mode).
 * @param[in] id:         Which QSPI module want to Transmit.
 * @param[in] data_width: Just support @ref QSPI_DATASIZE_08_BITS @ref QSPI_DATASIZE_16_BITS @ref QSPI_DATASIZE_32_BITS
 * @param[in] p_data:     Pointer to data buffer
 * @param[in] length:     Amount of data to be transmitted in bytes
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_qspi_transmit_in_qpi_async(app_qspi_id_t id, uint32_t data_width, uint8_t *p_data, uint32_t length);
#endif

/**
 ****************************************************************************************
 * @brief  Return the QSPI handle.
 *
 * @param[in]  id: QSPI module ID.
 *
 * @return Pointer to the specified ID's QSPI handle.
 ****************************************************************************************
 */
qspi_handle_t *app_qspi_get_handle(app_qspi_id_t id);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */

/** @} */

/** @} */

/**
 ****************************************************************************************
 *
 * @file    app_graphics_dc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of DC app library.
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

/** @defgroup APP_GRAPHICS_DC DC
  * @brief GRAPHICS_DC APP module driver.
  * @{
  */

#ifndef __APP_GRAPHICS_GRAPHICS_DC_H__
#define __APP_GRAPHICS_GRAPHICS_DC_H__

#include "gr55xx.h"
#include "app_io.h"
#include "hal_gdc.h"
#include "hal_gdc_regs.h"
#include "hal_gdc_mipi.h"

/** @addtogroup APP_GRAPHICS_DC_ENUM Enumerations
  * @{
  */

/**
  * @brief Define SPI work Mode for DC
  */
typedef enum {
    GDC_MODE_SPI = 0,       /**<  By 1-wire SPI */
    GDC_MODE_DSPI,          /**< 1bit cmd + 8bit data, and DCX signal */
    GDC_MODE_QSPI,          /**<  By Quad SPI */
} graphics_dc_mspi_e;


/**
  * @brief Define Clock Frequency for DC
  */
typedef enum {
    GDC_CLOCK_FREQ_48MHz = 0x00,    /**< DC clock, 48MHz */
    GDC_CLOCK_FREQ_24MHz = 0x03,    /**< DC clock, 24MHz */
    GDC_CLOCK_FREQ_12MHz = 0x05,    /**< DC clock, 12MHz */
    GDC_CLOCK_FREQ_6MHz  = 0x09,    /**< DC clock, 6MHz  */
    GDC_CLOCK_FREQ_3MHz  = 0x11,    /**< DC clock, 3MHz  */
} graphics_dc_clock_freq_e;


/**
  * @brief Define Clock Mode for DC
  */
typedef enum {
    GDC_CLOCK_MODE_0 = 0x00,        /**< DC clock mode 0 */
    GDC_CLOCK_MODE_1 = 0x01,        /**< DC clock mode 1 */
    GDC_CLOCK_MODE_2 = 0x02,        /**< DC clock mode 2 */
    GDC_CLOCK_MODE_3 = 0x03,        /**< DC clock mode 3 */
} graphics_dc_clock_mode_e;


/**
  * @brief Define Delay Clock for DC Tcsu
  */
typedef enum {
    GDC_TCSU_CYCLE_0 = 0x00,        /**< delay 0 clock cycle */
    GDC_TCSU_CYCLE_1 = 0x01,        /**< delay 1 clock cycle */
    GDC_TCSU_CYCLE_2 = 0x02,        /**< delay 2 clock cycle */
    GDC_TCSU_CYCLE_3 = 0x03,        /**< delay 3 clock cycle */
    GDC_TCSU_CYCLE_4 = 0x04,        /**< delay 4 clock cycle */
} graphics_dc_tcsu_cycle_e;

/**
  * @brief Display Controller Power Mode Enumerations definition
  */
typedef enum {
    GDC_POWER_STATE_SLEEP = 0,        /* sleep  state */
    GDC_POWER_STATE_ACTIVE = 1,       /* active state */
} graphics_dc_power_state_e;

/** @} */


/** @addtogroup APP_GRAPHICS_DC_STRUCTURES Structures
  * @{
  */

/** @defgroup GRAPHICS_DC_Configuration GRAPHICS DC Configuration
  * @{
  */
/**
  * @brief QSPI IO configuration Structures
  */
typedef struct
{
    app_io_pull_t  pull;                /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
    uint8_t        enable;              /**< Enable or disable the pin. */
} app_graphics_dc_pin_t;


/**
  * @brief define DC pins
  */
typedef struct
{
    app_graphics_dc_pin_t csn;         /**< Set the configuration of QSPI CS pin. */
    app_graphics_dc_pin_t clk;         /**< Set the configuration of QSPI CLK pin. */
    app_graphics_dc_pin_t io0;         /**< Set the configuration of QSPI IO0 pin. */
    app_graphics_dc_pin_t io1;         /**< Set the configuration of QSPI IO1 pin. */
    app_graphics_dc_pin_t io2;         /**< Set the configuration of QSPI IO2 pin. */
    app_graphics_dc_pin_t io3;         /**< Set the configuration of QSPI IO3 pin. */
    app_graphics_dc_pin_t dcx;         /**< Set the configuration of QSPI IO3 pin. */
} app_graphics_dc_pins_t;
/** @} */

/** @} */


/** @addtogroup APP_GRAPHICS_DC_ENUM Enumerations
  * @{
  */
/**
  * @brief Define work layers for DC
  */
typedef enum {
    GDC_ONE_LAYER_MODE = 0x00,          /**< 1 layer mode */
    GDC_TWO_LAYER_MODE = 0x01,          /**< 2 layer mode */
} graphics_dc_layer_mode_e;


/**
  * @brief Define the data format for frame buffer of DC
  */
typedef enum {
    GDC_DATA_FORMAT_RGB565      = HAL_GDC_RGB565,    /**< FrameBuffer is RGA565,   16bit, no Alpha  */
    GDC_DATA_FORMAT_RGB24       = HAL_GDC_RGB24,     /**< FrameBuffer is RGA24,    24bit, no Alpha  */
    GDC_DATA_FORMAT_RGBA8888    = HAL_GDC_RGBA8888,  /**< FrameBuffer is RGBA8888, 32bit with Alpha */
    GDC_DATA_FORMAT_ABGR8888    = HAL_GDC_ABGR8888,  /**< FrameBuffer is ABGR8888, 32bit with Alpha */
    GDC_DATA_FORMAT_ARGB8888    = HAL_GDC_ARGB8888,  /**< FrameBuffer is ARGB8888, 32bit with Alpha */
    GDC_DATA_FORMAT_BGRA8888    = HAL_GDC_BGRA8888,  /**< FrameBuffer is BGRA8888, 32bit with Alpha */
    GDC_DATA_FORMAT_TSC4        = HAL_GDC_TSC4,      /**< FrameBuffer is RGB565 compressed by TSC4  */
    GDC_DATA_FORMAT_TSC6        = HAL_GDC_TSC6,      /**< FrameBuffer is *888 compressed by TSC6  */
    GDC_DATA_FORMAT_TSC6A       = HAL_GDC_TSC6A,     /**< FrameBuffer is *8888 compressed by TSC6A  */
} graphics_dc_data_format_e;


/**
  * @brief Define the Output MIPI Timing for DATA Phase of DC
  *     Timing of MIPICFG_2RGB888_OPT1 is True MIPICFG_2RGB888_OPT0, and
  *     Timing of MIPICFG_2RGB888_OPT0 is True MIPICFG_2RGB888_OPT1, They need to exchange !!!
  */
typedef enum {
    GDC_MIPICFG_SPI_RGB565_OPT0     = MIPICFG_1RGB565_OPT0,     /**< Sent in  SPI Mode, Output format is RGB565 with option.0 */
    GDC_MIPICFG_SPI_RGB888_OPT0     = MIPICFG_1RGB888_OPT0,     /**< Sent in  SPI Mode, Output format is RGB565 with option.0 */
    GDC_MIPICFG_DSPI_RGB565_OPT0    = MIPICFG_2RGB565_OPT0,     /**< Sent in DSPI Mode, Output format is RGB565 with option.0 */
    GDC_MIPICFG_DSPI_RGB888_OPT0    = MIPICFG_2RGB888_OPT1,     /**< Sent in DSPI Mode, Output format is RGB888 with option.0 */
    GDC_MIPICFG_DSPI_RGB888_OPT1    = MIPICFG_2RGB888_OPT0,     /**< Sent in DSPI Mode, Output format is RGB888 with option.1 */
    GDC_MIPICFG_QSPI_RGB565_OPT0    = MIPICFG_4RGB565_OPT0,     /**< Sent in QSPI Mode, Output format is RGB565 with option.0 */
    GDC_MIPICFG_QSPI_RGB888_OPT0    = MIPICFG_4RGB888_OPT0,     /**< Sent in QSPI Mode, Output format is RGB888 with option.0 */
} graphics_dc_mipi_format_e;


/**
  * @brief Define the Output pixel bits for DC
  */
typedef enum {
    GDC_OUT_PIXEL_BITS_16 = 16,                 /**< Output pixel 16 bits  */
    GDC_OUT_PIXEL_BITS_24 = 24,                 /**< Output pixel 24 bits  */
    GDC_OUT_PIXEL_BITS_NOT_SUPPORT = 0xFF,      /**< Not support  */
} graphics_dc_out_pixel_bits_e;

/**
  * @brief Define the Output Frame Timing for DC
  */
typedef enum {
    GDC_SPI_FRAME_TIMING_0  =  0x00,            /**< 8Bit CMD::24Bit ADDR::Ndata, All Sent in SPI  */
    GDC_DSPI_FRAME_TIMING_0,                    /**< 8Bit CMD Sent in SPI::NO ADDR::Ndata Sent in DSPI, with DCX */
    GDC_QSPI_FRAME_TIMING_0,                    /**< 8Bit CMD::24Bit ADDR Sent in SPI, All Data Sent in QSPI  */
    GDC_QSPI_FRAME_TIMING_1,                    /**< 8Bit CMD Sent in SPI, 24Bit ADDR and All data Sent in QSPI  */
} app_graphics_dc_frame_timing_e;


/**
  * @brief Define the bits of address phase for DC Frame
  */
typedef enum {
    GDC_FRAME_ADDRESS_WIDTH_NONE    = 0xFF,         /**< Not support  */
    GDC_FRAME_ADDRESS_WIDTH_08BIT   = MIPI_CMD08,   /**< Frame address width 8bits  */
    GDC_FRAME_ADDRESS_WIDTH_16BIT   = MIPI_CMD16,   /**< Frame address width 16bits  */
    GDC_FRAME_ADDRESS_WIDTH_24BIT   = MIPI_CMD24,   /**< Frame address width 24bits  */
} app_graphics_dc_frame_address_width_e;

/**
  * @brief Define access type for DC
  */
typedef enum {
    GDC_ACCESS_TYPE_SYNC = 0,                       /**< SYNC access type  */
    GDC_ACCESS_TYPE_ASYNC,                          /**< ASYNC access type */
} app_graphics_dc_access_type_e;


/**
  * @brief Define frame output result for DC
  */
typedef enum {
    GDC_FRAME_RES_SUCCESS = 0x00,       /**< frame sent success */
    GDC_FRAME_RES_ASYNC_WAIT,           /**< frame sent, but need to get the result in async callback */
    GDC_FRAME_RES_FAIL,                 /**< frame sent fail */
    GDC_FRAME_RES_UNSUPPORT,            /**< frame format/command not support,please check config params */
} app_graphics_dc_frame_result_e;

/** @} */

/** @addtogroup APP_GRAPHICS_DC_STRUCTURES Structures
  * @{
  */

/**
  * @brief Define init params for DC
  */
typedef struct {
    graphics_dc_mspi_e          mspi_mode;             /**< Specify spi mode, Ref Optional values of graphics_dc_mspi_e */
    graphics_dc_clock_freq_e    clock_freq;            /**< Specify dc clock freq, Ref Optional values of graphics_dc_clock_freq_e */
    graphics_dc_clock_mode_e    clock_mode;            /**< Specify dc clock mode, Ref Optional values of graphics_dc_clock_mode_e */
    graphics_dc_tcsu_cycle_e    tcsu_cycle;            /**< Specify cs setup delay, Ref Optional values of graphics_dc_tcsu_cycle_e */
    graphics_dc_layer_mode_e    layer_mode;            /**< Specify which layer to flush, Ref Optional values of graphics_dc_layer_mode_e */
    graphics_dc_mipi_format_e   mipicfg_format;        /**< Specify mipi timing format, Ref Optional values of graphics_dc_mipi_format_e */
    uint16_t                    resolution_x;          /**< Specify the x resolution in pixels */
    uint16_t                    resolution_y;          /**< Specify the y resolution in pixels */
    app_graphics_dc_pins_t      pins_cfg;              /**< Specify pins state */
} app_graphics_dc_params_t;

/**
  * @brief Define DC Frame Layer configuration
  */
typedef struct {
    void *                      frame_baseaddr ;        /**< Frame Address */
    uint32_t                    resolution_x;           /**< Resolution X */
    uint32_t                    resolution_y;           /**< Resolution Y */
    int32_t                     row_stride;             /**< Stride */
    int32_t                     start_x;                /**< Start Rendering X Coordinator */
    int32_t                     start_y;                /**< Start Rendering Y Coordinator */
    uint32_t                    size_x;                 /**< Rendering Size X */
    uint32_t                    size_y;                 /**< Rendering Size Y */
    uint8_t                     alpha;                  /**< Alpha */
    uint8_t                     blendmode;              /**< Blending Mode */
    graphics_dc_data_format_e   data_format;            /**< Format */
} app_graphics_dc_framelayer_t;


/**
  * @brief Define Control Command for DC Frame
  */
typedef struct {
    uint8_t                                 command;            /**< Command phase for display timing */
    uint32_t                                address;            /**< Address phase for display timing,if no address phase, ignore this */
    app_graphics_dc_frame_address_width_e   address_width;      /**< Optional values: @ref GDC_FRAME_ADDRESS_WIDTH_NONE
                                                                                      @ref GDC_FRAME_ADDRESS_WIDTH_08BIT
                                                                                      @ref GDC_FRAME_ADDRESS_WIDTH_16BIT
                                                                                      @ref GDC_FRAME_ADDRESS_WIDTH_24BIT */
    app_graphics_dc_frame_timing_e          frame_timing;       /**< Specify the supported frame timing */
} app_graphics_dc_cmd_t;

/** @} */


/**
  * @defgroup  APP_GRAPHICS_DC_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup GRAPHICS_DC_Exported_Constants DC Exported Constants
  * @{
  */

/** @defgroup GRAPHICS_DC_PIN DC Pins Define
  * @{
  */
#define GRAPHICS_DC_CSN_PORT                    APP_IO_TYPE_GPIOB       /**< Define DC CSN PORT */
#define GRAPHICS_DC_CSN_PIN                     APP_IO_PIN_11           /**< Define DC CSN PIN */
#define GRAPHICS_DC_CSN_PIN_MUX                 APP_IO_MUX_1            /**< Define DC CSN PIN.MUX */

#define GRAPHICS_DC_CLK_PORT                    APP_IO_TYPE_GPIOB       /**< Define DC CLK PORT */
#define GRAPHICS_DC_CLK_PIN                     APP_IO_PIN_0            /**< Define DC CLK PIN */
#define GRAPHICS_DC_CLK_PIN_MUX                 APP_IO_MUX_1            /**< Define DC CLK PIN.MUX */

#define GRAPHICS_DC_IO0_PORT                    APP_IO_TYPE_GPIOB       /**< Define DC IO0 PORT */
#define GRAPHICS_DC_IO0_PIN                     APP_IO_PIN_1            /**< Define DC IO0 PIN */
#define GRAPHICS_DC_IO0_PIN_MUX                 APP_IO_MUX_1            /**< Define DC IO0 PIN.MUX */

#define GRAPHICS_DC_IO1_PORT                    APP_IO_TYPE_GPIOB       /**< Define DC IO1 PORT */
#define GRAPHICS_DC_IO1_PIN                     APP_IO_PIN_2            /**< Define DC IO1 PIN */
#define GRAPHICS_DC_IO1_PIN_MUX                 APP_IO_MUX_1            /**< Define DC IO1 PIN.MUX */

#define GRAPHICS_DC_IO2_PORT                    APP_IO_TYPE_GPIOB       /**< Define DC IO2 PORT */
#define GRAPHICS_DC_IO2_PIN                     APP_IO_PIN_3            /**< Define DC IO2 PIN */
#define GRAPHICS_DC_IO2_PIN_MUX                 APP_IO_MUX_1            /**< Define DC IO2 PIN.MUX */

#define GRAPHICS_DC_IO3_PORT                    APP_IO_TYPE_GPIOB       /**< Define DC IO3 PORT */
#define GRAPHICS_DC_IO3_PIN                     APP_IO_PIN_4            /**< Define DC IO3 PIN */
#define GRAPHICS_DC_IO3_PIN_MUX                 APP_IO_MUX_1            /**< Define DC IO3 PIN.MUX */

#define GRAPHICS_DC_DCX_PORT                    APP_IO_TYPE_GPIOB       /**< Define DC DCX PORT */
#define GRAPHICS_DC_DCX_PIN                     APP_IO_PIN_13           /**< Define DC DCX PIN */
#define GRAPHICS_DC_DCX_PIN_MUX                 APP_IO_MUX_5            /**< Define DC DCX PIN.MUX */
/** @} */

/** @defgroup GRAPHICS_DC_LAYER DC Layers Define
  * @{
  */
#define GRAPHICS_DC_LAYER_0                     0u          /**< Define DC Layer 0 */
#define GRAPHICS_DC_LAYER_1                     1u          /**< Define DC Layer 1 */
/** @} */

/** @defgroup GRAPHICS_DC_EVT IRQ callback events Define
  * @{
  */
#define GDC_IRQ_EVT_FRAME_TRANSMITION_END       0x01        /**< Define Frame Xfer End event */
#define GDC_IRQ_EVT_CMD_TRANSMITION_END         0x02        /**< Define CMD Xfer End event */
/** @} */

/** @defgroup GRAPHICS_DC_BASEADDR DC registers memory base address Define
  * @{
  */
#define GRAPHICS_DC_BASEADDR                    0xA3FF4000  /**< Define DC registers memory base address */
/** @} */

/** @} */

/** @} */


/** @addtogroup APP_GRAPHICS_DC_TYPEDEFS Type definitions
  * @{
  */
/**
  * @brief DC IRQ callback definition
  */
typedef void (* graphics_dc_irq_event_notify_cb )(uint32_t evt);

/**
  * @brief DC Refresh callback definition
  */
typedef void (* graphics_dc_set_refresh_area_cb )(uint32_t mark, uint32_t x_start, uint32_t x_end, uint32_t y_start, uint32_t y_end);

/** @} */


/** @addtogroup APP_GRAPHICS_DC_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief init Graphics DC dev
 *
 * @param[in] dc_params: pointer to dc init params
 * @param[in] evt_cb: event callback
 *            Note: GDC_IRQ_EVT_FRAME_TRANSMITION_END  & GDC_IRQ_EVT_CMD_TRANSMITION_END
 * @retval ::APP_DRV_SUCCESS
 * @retval ::APP_DRV_ERR_HAL
 * @retval ::APP_DRV_ERR_POINTER_NULL
 ****************************************************************************************
 */
uint16_t graphics_dc_init(app_graphics_dc_params_t * dc_params, graphics_dc_irq_event_notify_cb evt_cb);

/**
 ****************************************************************************************
 * @brief de-init Graphics DC dev, just called when needed to reboot/reset
 *
 ****************************************************************************************
 */
void graphics_dc_deinit(void);

/**
 ****************************************************************************************
 * @brief re-init i/o for Graphics DC dev with pre-init i/o setting
 *
 ****************************************************************************************
 */
void graphics_dc_pins_reinit(void);

/**
 *****************************************************************************************
 * @brief Switch power state for DC module
 *
 * @param[in] state: power state to switch
 *
 * @return none
 *****************************************************************************************
 */
void app_graphics_dc_set_power_state(graphics_dc_power_state_e state);

/**
 *****************************************************************************************
 * @brief DC clock frequency set
 *
 * @param[in] clock_freq: DC clock frequency
 *
 * @return none
 *****************************************************************************************
 */
void app_graphics_dc_freq_set(graphics_dc_clock_freq_e clock_freq);

/**
 ****************************************************************************************
 * @brief Send 1 Byte CMD,3 Byte ADDR And N Byte Data in 1-wire SPI Mode
 * @note  Timing Diagram :
 *   CSN: |_________________________________________________|
 *   CLK: __|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|___
 *   IO0: __[ -1Byte CMD- ][ -3Byte CMD- ][ -NByte Data- ]__
 * @param[in] cmd_8bit: 8bits command
 * @param[in] address_24bit: 24bits address
 * @param[in] data: Pointer to data buffer
 * @param[in] length: Data length
 ****************************************************************************************
 */
void app_graphics_dc_spi_send(uint8_t cmd_8bit, uint32_t address_24bit, uint8_t * data, uint32_t length);


/**
 ****************************************************************************************
 * @brief Send single cmd in 3-wire mode for DSPI (no DCX and 1 more MSB Bit for cmd indicator)
 * @note  Timing Diagram :
 *   CSN: |______________________________________|
 *   CLK: __|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|__
 *   SD0: __[0][ ---------8Bit CMD----------- ]__   (9-Bit in All)
 *   DCX: _______________________________________   (Always Low)
 * @param[in] cmd: 8bits command
 ****************************************************************************************
 */
void app_graphics_dc_dspi_send_cmd_in_3wire_1lane(uint8_t cmd);


/**
 ****************************************************************************************
 * @brief Send single cmd &data in 3-wire mode for DSPI (no DCX and 1 more MSB Bit for cmd/data indicator)
 * @note  Timing Diagram :
 *   CSN: |_________________________________________________________________________|
 *   CLK: __|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|__
 *   SD0: __[0][ ---------8Bit CMD------------][1][ ---------8Bit DATA----------- ]__   (18-Bit in All)
 *   DCX: ___________________________________________________________________________   (Always Low)
 * @param[in] cmd: 8bits command
 * @param[in] data: 8bits data
 ****************************************************************************************
 */
void app_graphics_dc_dspi_send_cmd_data_in_3wire_1lane(uint8_t cmd, uint8_t data) ;


/**
 ****************************************************************************************
 * @brief Send cmd &data in 4-wire mode for DSPI (DCX as SD1, and 1 more MSB Bit for cmd/data indicator)
 * @note Timing Diagram :
 *   CSN: |___________________________________________|
 *   CLK: __|-|_|-|_|...|-|_|-|_|-|_|-|_|....|-|_|-|___
 *   SD0: __[0][ ---H8Bit CMD--][1][ --H8Bit DATA-- ]__   (18Bit in All )
 *   DCX: __[0][ ---L8Bit CMD--][1][ --L8Bit DATA-- ]__   (use DCX as SD1)
 * @param[in] cmd: 16bits command
 * @param[in] data: 16bits data
 ****************************************************************************************
 */
void app_graphics_dc_dspi_send_cmd_data_in_4wire_2lane(uint16_t cmd,  uint16_t data);


/**
 ****************************************************************************************
 * @brief Send cmd &data in 4-wire mode for DSPI (DCX as SD1, and 1 more MSB Bit for cmd/data indicator)
 * @note Timing Diagram :
 *   CSN: |___________________________________________________________________________|
 *   CLK: __|-|_|-|_|...|-|_|-|_|-|_|-|_|....|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|...-|__
 *   SD0: __[0][ ---H8Bit CMD--][1][ --H8Bit DATA-- ][1][ --H8Bit DATA-- ][1][......]__   (Nx9Bit in All )
 *   DCX: __[0][ ---L8Bit CMD--][1][ --L8Bit DATA-- ][1][ --L8Bit DATA-- ][1][......]__   (use DCX as SD1)
 * @param[in] cmd: 16bits command
 * @param[in] data: Pointer to data buffer
 * @param[in] length: Data length
 ****************************************************************************************
 */
void app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(uint16_t cmd,  uint16_t * data , int  length);


/**
 ****************************************************************************************
 * @brief Send one whole frame by DC
 *
 * @param[in] which_layer:
 *         @arg @ref GRAPHICS_DC_LAYER_0
 *         @arg @ref GRAPHICS_DC_LAYER_1
 * @param[in] frame_layer: pointer to dc layer setting
 * @param[in] dc_cmd: pointer to DC control command
 * @param[in] access_type:
 *         @arg @ref GDC_ACCESS_TYPE_SYNC, send frame sync
 *         @arg @ref GDC_ACCESS_TYPE_ASYNC, send frame async, must handle the frame result in callback
 ****************************************************************************************
 */
app_graphics_dc_frame_result_e app_graphics_dc_send_single_frame(uint32_t which_layer, app_graphics_dc_framelayer_t * frame_layer, app_graphics_dc_cmd_t * dc_cmd, app_graphics_dc_access_type_e access_type);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __APP_GRAPHICS_GRAPHICS_DC_H__ */

/** @} */
/** @} */
/** @} */

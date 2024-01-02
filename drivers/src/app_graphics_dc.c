/**
  ****************************************************************************************
  * @file    app_graphics_dc.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2021 GOODIX All rights reserved.

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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "gr_soc.h"
#include "string.h"
#include "app_pwr_mgmt.h"
#include "app_drv_error.h"
#include "app_io.h"
#include "app_graphics_dc.h"

#include "grx_hal.h"
#include "hal_gdc_mipi.h"
#include "hal_gdc_regs.h"
#include "hal_gdc.h"
#ifdef ENV_USE_FREERTOS
#ifdef USE_OSAL
#include "osal.h"
#else // USE_OSAL
#include "app_rtos_cfg.h"
#include "FreeRTOS.h"
#include "task.h"
#endif // USE_OSAL
#elif 1==CONFIG_ZEPHYR_OS
#include <zephyr/kernel.h>
#endif


#pragma  diag_suppress      177

/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef ENV_USE_FREERTOS
    #define GPU_WAIT_IRQ_USING_SEMAPHORE    (1u)
#else
    #define GPU_WAIT_IRQ_USING_SEMAPHORE    (0u)
#endif

#define GPU_WAIT_TIMEOUT_MS             (1000)
#define GREG(reg)                       (*((volatile uint32_t *)(reg)))
#define GDC_REG_REF(Reg)                (*((volatile uint32_t *)(GRAPHICS_DC_BASEADDR + Reg)))
#define GDC_SET_Bit(Reg, Bit)           SET_BITS(GDC_REG_REF(Reg),   Bit)
#define GDC_CLR_Bit(Reg, Bit)           CLEAR_BITS(GDC_REG_REF(Reg), Bit)

#define DC_IRQ_CMD_END_BIT              (1u << 6)
#define DC_IRQ_FRAME_END_BIT            (1u << 4)
#define GDC_ENABLE_CMD_END_IRQ()        GDC_SET_Bit(HAL_GDC_REG_INTERRUPT, DC_IRQ_CMD_END_BIT)
#define GDC_ENABLE_FRAME_END_IRQ()      GDC_SET_Bit(HAL_GDC_REG_INTERRUPT, DC_IRQ_FRAME_END_BIT)

#define FRAME_ASYNC_MARK_SPI_HOLD       0x01
#define FRAME_ASYNC_MARK_FORCE_CS       0x02


#define GRAPHICS_MCU_CONF_ENABLE        1u

/// Define Block debug print info control macro
#define GDC_DBG_INFO_ENABLE 0
#if GDC_DBG_INFO_ENABLE
#define GDC_DBG_PRINTF printf
#else
#define GDC_DBG_PRINTF(fmt, ...)
#endif

typedef struct {
    app_graphics_dc_params_t        init_params;
    graphics_dc_irq_event_notify_cb irq_evt_cb;
    volatile uint32_t               clock_mode;
    volatile uint8_t                is_inited;
    volatile uint8_t                is_cmd_cplt;
    volatile uint8_t                is_frame_cplt;
    volatile uint8_t                is_async_mark;
} app_graphics_dc_t;

/*
 * STATIC DECLRATION
 *****************************************************************************************
 */

void TSI_DC_IRQHandler(void);

#if GPU_WAIT_IRQ_USING_SEMAPHORE > 0u
    #if CONFIG_ZEPHYR_OS
        static struct k_sem s_dc_irq_sem;
    #else
#ifdef USE_OSAL
        static osal_sema_handle_t s_dc_irq_sem;
#else
        APP_DRV_SEM_STATIC(s_dc_irq_sem);
#endif
    #endif
#endif
static uint16_t                         dc_pins_init(app_graphics_dc_pins_t pins);
static void                             dc_pins_deinit(void);
static int                              _dc_irq_sem_init(void);
static int                              _dc_irq_sem_take(void);
static int                              _dc_irq_sem_give(void);
static void                             dc_clock_on(graphics_dc_clock_freq_e clock);
static void                             dc_clock_off(void);
static void                             dc_enable_cmd_irq(void);
static void                             dc_wait_cmd_irq(void);
static void                             dc_enable_frame_irq(void);
static void                             dc_wait_frame_end(void);
static void                             dc_wait_cs_deassert(void);
static graphics_dc_out_pixel_bits_e     dc_get_out_pixel_bits(void);
static void                             dc_set_clock_mode(graphics_dc_clock_mode_e mode);
static void                             dc_set_tcsu_delay_cycle(graphics_dc_tcsu_cycle_e delay);
static void                             dc_irq_event_callback(uint32_t evt);
static app_graphics_dc_t                s_graphics_dc_env;
#if GPU_WAIT_IRQ_USING_SEMAPHORE > 0u
static volatile int                     s_dc_sem_give_cnt = 0;
static volatile int                     s_dc_sem_take_cnt = 0;
#endif
/*
 * PUBLIC METHODS
 *****************************************************************************************
 */
uint16_t graphics_dc_init(app_graphics_dc_params_t * dc_params, graphics_dc_irq_event_notify_cb evt_cb) {

    s_graphics_dc_env.is_inited = 0;
    if (NULL == dc_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    dc_pins_init(dc_params->pins_cfg);
    dc_clock_on(dc_params->clock_freq);

    if(0 != hal_gdc_init()) {
        return APP_DRV_ERR_HAL;
    }

    /* Set clock mode */
    dc_set_clock_mode(dc_params->clock_mode);

    /* Set Tcsu Delay */
    dc_set_tcsu_delay_cycle(dc_params->tcsu_cycle);

    soc_register_nvic(TSI_DC_IRQn, (uint32_t)TSI_DC_IRQHandler);
    /* Clear the interrupt */
    hal_gdc_reg_write(HAL_GDC_REG_INTERRUPT, 0);
    NVIC_ClearPendingIRQ(TSI_DC_IRQn);
    NVIC_EnableIRQ(TSI_DC_IRQn);

    memcpy((void*)&s_graphics_dc_env.init_params, (void*)dc_params,    sizeof(app_graphics_dc_params_t));
    s_graphics_dc_env.irq_evt_cb    = evt_cb;
    s_graphics_dc_env.is_inited     = 1;
    s_graphics_dc_env.is_cmd_cplt   = 0;
    s_graphics_dc_env.is_frame_cplt = 0;
    s_graphics_dc_env.is_async_mark = 0;
    _dc_irq_sem_init();
    _dc_irq_sem_give();

#if GRAPHICS_MCU_CONF_ENABLE > 0u
    // This configuration could be changed to save energy
    MCU_RET->MCU_SUBSYS_CG_CTRL[0] = 0; // MCU clock related
    MCU_RET->MCU_SUBSYS_CG_CTRL[1] = 0; // MCU clock related
    // Prevent WFI turn off mcu register clock
    MCU_RET->MCU_SUBSYS_CG_CTRL[2] = 0;
    MCU_RET->MCU_PERIPH_PCLK_OFF = 0; // XQSPI related
    MCU_RET->MCU_PERIPH_CG_LP_EN = 0; // QSPI related
    MCU_RET->MCU_PERIPH_CLK_SLP_OFF = 0; // PSRAM related
#endif
    return APP_DRV_SUCCESS;
}

void graphics_dc_deinit(void) {
    NVIC_DisableIRQ(TSI_DC_IRQn);
    NVIC_ClearPendingIRQ(TSI_DC_IRQn);
    dc_clock_off();
    dc_pins_deinit();
}


void graphics_dc_pins_reinit(void) {
    dc_pins_init(s_graphics_dc_env.init_params.pins_cfg);
}

void app_graphics_dc_set_power_state(graphics_dc_power_state_e state) {
    switch(state) {
        case GDC_POWER_STATE_SLEEP:{
            _dc_irq_sem_give();
#if !CONFIG_ZEPHYR_OS
            GLOBAL_EXCEPTION_DISABLE();
            hal_pwr_mgmt_set_extra_device_state(EXTRA_DEVICE_NUM_DC, IDLE);  
            GLOBAL_EXCEPTION_ENABLE();
#endif
            break;
        }
        case GDC_POWER_STATE_ACTIVE:{
            _dc_irq_sem_take();
#if !CONFIG_ZEPHYR_OS
            GLOBAL_EXCEPTION_DISABLE();
            app_graphics_dc_params_t * dc_params = &s_graphics_dc_env.init_params;
            dc_clock_on(dc_params->clock_freq);
            dc_set_clock_mode(dc_params->clock_mode);
            dc_set_tcsu_delay_cycle(dc_params->tcsu_cycle);
            hal_gdc_reg_write(HAL_GDC_REG_INTERRUPT, 0);
            hal_pwr_mgmt_set_extra_device_state(EXTRA_DEVICE_NUM_DC, ACTIVE);
            NVIC_ClearPendingIRQ(TSI_DC_IRQn);
            NVIC_EnableIRQ(TSI_DC_IRQn);
            GLOBAL_EXCEPTION_ENABLE();
#endif
            break;
        }
        default:{
            break;
        }
    }
}

void app_graphics_dc_freq_set(graphics_dc_clock_freq_e clock_freq){
    s_graphics_dc_env.init_params.clock_freq = clock_freq;
}

/*******************************************************************************************
 * Send 1 Byte CMD,3 Byte ADDR And N Byte Data in 1-wire SPI Mode
 * Timing Diagram :
 *   CSN: |_________________________________________________|
 *   CLK: __|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|___
 *   IO0: __[ -1Byte CMD- ][ -3Byte CMD- ][ -NByte Data- ]__
 *******************************************************************************************/
void app_graphics_dc_spi_send(uint8_t cmd_8bit, uint32_t address_24bit, uint8_t * data, uint32_t length) {
    hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode | MIPICFG_QSPI | MIPICFG_SPI4 | MIPICFG_DBI_EN  | MIPICFG_RESX | s_graphics_dc_env.init_params.mipicfg_format | MIPICFG_DIS_TE);
    dc_enable_cmd_irq();
    GDC_SET_Bit(HAL_GDC_REG_DBIB_CFG, MIPICFG_SPI_HOLD);
    hal_gdc_MIPI_out (MIPI_DBIB_CMD | MIPI_MASK_QSPI | MIPI_CMD08 | cmd_8bit );            // Command Header 8-bit
    hal_gdc_MIPI_out (MIPI_DBIB_CMD | MIPI_MASK_QSPI | MIPI_CMD24 | address_24bit );       // Address 24-bit
    for(uint32_t i = 0; i < length; i++) {
        hal_gdc_MIPI_out (MIPI_MASK_QSPI | MIPI_CMD08 | data[i] );
    }

    GDC_CLR_Bit(HAL_GDC_REG_DBIB_CFG, MIPICFG_SPI_HOLD);
    dc_wait_cmd_irq();

    return;
}

/*******************************************************************************************
 * Send single cmd in 3-wire mode for DSPI (no DCX and 1 more MSB Bit for cmd indicator)
 * Timing Diagram :
 *   CSN: |______________________________________|
 *   CLK: __|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|__
 *   SD0: __[0][ ---------8Bit CMD----------- ]__   (9-Bit in All)
 *   DCX: _______________________________________   (Always Low)
 *******************************************************************************************/
void app_graphics_dc_dspi_send_cmd_in_3wire_1lane(uint8_t cmd) {
    dc_enable_cmd_irq();
    hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode | MIPICFG_1RGB332_OPT0 | MIPICFG_SPI | MIPICFG_SPI3 | MIPICFG_RESX | MIPICFG_DBI_EN | MIPICFG_SPIDC_DQSPI | MIPICFG_DSPI_SPIX);
    hal_gdc_MIPI_out( MIPI_DBIB_CMD | cmd);
    dc_wait_cmd_irq();
}

/*******************************************************************************************
 * Send single cmd &data in 3-wire mode for DSPI (no DCX and 1 more MSB Bit for cmd/data indicator)
 * Timing Diagram :
 *   CSN: |_________________________________________________________________________|
 *   CLK: __|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|__
 *   SD0: __[0][ ---------8Bit CMD------------][1][ ---------8Bit DATA----------- ]__   (18-Bit in All)
 *   DCX: ___________________________________________________________________________   (Always Low)
 *******************************************************************************************/
void app_graphics_dc_dspi_send_cmd_data_in_3wire_1lane(uint8_t cmd, uint8_t data) {
    hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode | MIPICFG_1RGB332_OPT0 | MIPICFG_SPI | MIPICFG_SPI3 | MIPICFG_RESX | MIPICFG_DBI_EN | MIPICFG_SPIDC_DQSPI | MIPICFG_SPI_HOLD);
    hal_gdc_MIPI_out( MIPI_DBIB_CMD | cmd);
    hal_gdc_MIPI_out( data);
    hal_gdc_reg_write(HAL_GDC_REG_DBIB_CFG, (hal_gdc_reg_read(HAL_GDC_REG_DBIB_CFG) & (~MIPICFG_SPI_HOLD)));
    dc_wait_cs_deassert();
}

/*******************************************************************************************
 * Send cmd &data in 4-wire mode for DSPI (DCX as SD1, and 1 more MSB Bit for cmd/data indicator)
 * Timing Diagram :
 *   CSN: |___________________________________________|
 *   CLK: __|-|_|-|_|...|-|_|-|_|-|_|-|_|....|-|_|-|___
 *   SD0: __[0][ ---H8Bit CMD--][1][ --H8Bit DATA-- ]__   (18Bit in All )
 *   DCX: __[0][ ---L8Bit CMD--][1][ --L8Bit DATA-- ]__   (use DCX as SD1)
 *******************************************************************************************/
void app_graphics_dc_dspi_send_cmd_data_in_4wire_2lane(uint16_t cmd,  uint16_t data) {
    hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode | MIPICFG_2RGB565_OPT0 | MIPICFG_DSPI | MIPICFG_DSPI_SPIX | MIPICFG_SPI3 | MIPICFG_RESX | MIPICFG_DBI_EN | MIPICFG_SPIDC_DQSPI | MIPICFG_SPI_HOLD);
    hal_gdc_MIPI_out( MIPI_DBIB_CMD | MIPI_CMD16 | cmd);
    hal_gdc_MIPI_out( MIPI_CMD16 | data);
    hal_gdc_reg_write(HAL_GDC_REG_DBIB_CFG, (hal_gdc_reg_read(HAL_GDC_REG_DBIB_CFG) & (~MIPICFG_SPI_HOLD)));
    dc_wait_cs_deassert();
}

/*******************************************************************************************
 * Send cmd &data in 4-wire mode for DSPI (DCX as SD1, and 1 more MSB Bit for cmd/data indicator)
 * Timing Diagram :
 *   CSN: |___________________________________________________________________________|
 *   CLK: __|-|_|-|_|...|-|_|-|_|-|_|-|_|....|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|...-|__
 *   SD0: __[0][ ---H8Bit CMD--][1][ --H8Bit DATA-- ][1][ --H8Bit DATA-- ][1][......]__   (Nx9Bit in All )
 *   DCX: __[0][ ---L8Bit CMD--][1][ --L8Bit DATA-- ][1][ --L8Bit DATA-- ][1][......]__   (use DCX as SD1)
 *******************************************************************************************/
void app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(uint16_t cmd,  uint16_t * data , int  length) {
    hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode | MIPICFG_2RGB565_OPT0 | MIPICFG_DSPI | MIPICFG_DSPI_SPIX | MIPICFG_SPI3 | MIPICFG_RESX | MIPICFG_DBI_EN | MIPICFG_SPIDC_DQSPI | MIPICFG_SPI_HOLD);
    hal_gdc_MIPI_out( MIPI_DBIB_CMD | MIPI_CMD16 | cmd);
    for(int i = 0; i  < length;  i++){
        hal_gdc_MIPI_out( MIPI_CMD16 | data[i]);
    }
    hal_gdc_reg_write(HAL_GDC_REG_DBIB_CFG, (hal_gdc_reg_read(HAL_GDC_REG_DBIB_CFG) & (~MIPICFG_SPI_HOLD)));
    dc_wait_cs_deassert();
    return;
}


app_graphics_dc_frame_result_e app_graphics_dc_send_single_frame(uint32_t which_layer, app_graphics_dc_framelayer_t * frame_layer, app_graphics_dc_cmd_t * dc_cmd, app_graphics_dc_access_type_e access_type)
{
    hal_gdc_layer_t _dc_layer;
    app_graphics_dc_frame_result_e res = GDC_FRAME_RES_FAIL;

    if(which_layer > GRAPHICS_DC_LAYER_1) {
        return res;
    }

    /* Set DC Layer */
    {
        _dc_layer.baseaddr_virt = frame_layer->frame_baseaddr;
        _dc_layer.baseaddr_phys = (uintptr_t)frame_layer->frame_baseaddr;
        _dc_layer.resx          = frame_layer->resolution_x;
        _dc_layer.resy          = frame_layer->resolution_y;
        _dc_layer.stride        = frame_layer->row_stride;
        _dc_layer.startx        = frame_layer->start_x;
        _dc_layer.starty        = frame_layer->start_y;
        _dc_layer.sizex         = frame_layer->size_x;
        _dc_layer.sizey         = frame_layer->size_y;
        _dc_layer.alpha         = frame_layer->alpha;
        _dc_layer.blendmode     = frame_layer->blendmode;
        _dc_layer.format        = (hal_gdc_format_t)frame_layer->data_format;
        _dc_layer.buscfg        = 0;
        _dc_layer.mode          = 0;
        _dc_layer.u_base        = 0;
        _dc_layer.v_base        = 0;
        _dc_layer.u_stride      = 0;
        _dc_layer.v_stride      = 0;
    }

    hal_gdc_timing(_dc_layer.resx, 1, 1, 1, _dc_layer.resy, 1, 1, 1);     // Set all to 1 besides x & y
    hal_gdc_set_layer(which_layer, &_dc_layer);                      // Program hal_gdc Layer
    hal_gdc_layer_enable(which_layer);                           // Enable hal_gdc Layer

    NVIC_EnableIRQ(TSI_DC_IRQn);
    switch(dc_cmd->frame_timing) {
        case GDC_SPI_FRAME_TIMING_0:
        {
            switch(dc_get_out_pixel_bits()) {
                case GDC_OUT_PIXEL_BITS_16:
                case GDC_OUT_PIXEL_BITS_24:
                {
                    /* Trigger frame transmission */
                    hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode | MIPICFG_SPI4 | s_graphics_dc_env.init_params.mipicfg_format | MIPICFG_DBI_EN | MIPICFG_RESX | MIPICFG_SPI_HOLD);
                    hal_gdc_MIPI_out    ( MIPI_DBIB_CMD | MIPI_MASK_QSPI | dc_cmd->command);
                    hal_gdc_MIPI_out    ( MIPI_DBIB_CMD | MIPI_MASK_QSPI | dc_cmd->address_width | dc_cmd->address);
                    dc_enable_frame_irq();
                    hal_gdc_set_mode(HAL_GDC_ONE_FRAME);                      /* trigger the frame transmission to start */
                    if(GDC_ACCESS_TYPE_SYNC == access_type) {
                        s_graphics_dc_env.is_async_mark = 0;
                        dc_wait_frame_end();
                        hal_gdc_reg_write(HAL_GDC_REG_DBIB_CFG, (hal_gdc_reg_read(HAL_GDC_REG_DBIB_CFG) & (~MIPICFG_SPI_HOLD)));
                        res = GDC_FRAME_RES_SUCCESS;
                    } else {
                        s_graphics_dc_env.is_async_mark = FRAME_ASYNC_MARK_SPI_HOLD;
                        res = GDC_FRAME_RES_ASYNC_WAIT;
                    }
                }
                break;

                default:
                {
                    GDC_DBG_PRINTF("Err: NOT SUPPORT!\r\n");
                    res = GDC_FRAME_RES_UNSUPPORT;
                }
                break;
            }
        }
        break;

        case GDC_DSPI_FRAME_TIMING_0:
        {
            switch(dc_get_out_pixel_bits()) {
                case GDC_OUT_PIXEL_BITS_16:
                {
                    hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode | MIPICFG_1RGB332_OPT0 | MIPICFG_SPI | MIPICFG_SPI3 | MIPICFG_RESX | MIPICFG_FRC_CSX_0 | MIPICFG_DBI_EN);
                    hal_gdc_MIPI_out(MIPI_DBIB_CMD | dc_cmd->command);
                    delay_us(5);
                    hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode | s_graphics_dc_env.init_params.mipicfg_format | MIPICFG_DSPI | MIPICFG_DSPI_SPIX | MIPICFG_SPI3 | MIPICFG_RESX | MIPICFG_FRC_CSX_0 | MIPICFG_DBI_EN | MIPICFG_SPIDC_DQSPI);
                    dc_enable_frame_irq();
                    hal_gdc_set_mode(HAL_GDC_ONE_FRAME);                      /* trigger the frame transmission to start */
                    if(GDC_ACCESS_TYPE_SYNC == access_type) {
                        s_graphics_dc_env.is_async_mark = 0;
                        dc_wait_frame_end();
                        hal_gdc_reg_write(HAL_GDC_REG_DBIB_CFG, (hal_gdc_reg_read(HAL_GDC_REG_DBIB_CFG) & (~MIPICFG_FRC_CSX_0)));   /* Release CS */
                        res = GDC_FRAME_RES_SUCCESS;
                    } else {
                        s_graphics_dc_env.is_async_mark = FRAME_ASYNC_MARK_FORCE_CS;
                        res = GDC_FRAME_RES_ASYNC_WAIT;
                    }
                }
                break;

                case GDC_OUT_PIXEL_BITS_24:
                {
                    hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode | MIPICFG_1RGB888_OPT0 | MIPICFG_SPI | MIPICFG_SPI3 | MIPICFG_RESX | MIPICFG_FRC_CSX_0 | MIPICFG_DBI_EN);
                    hal_gdc_MIPI_out(MIPI_DBIB_CMD | dc_cmd->command);
                    delay_us(5);
                    if(GDC_MIPICFG_DSPI_RGB888_OPT0 == s_graphics_dc_env.init_params.mipicfg_format) {
                        hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode | GDC_MIPICFG_DSPI_RGB888_OPT0 | MIPICFG_DSPI | MIPICFG_SPI3 | MIPICFG_RESX | MIPICFG_FRC_CSX_0 | MIPICFG_DBI_EN | MIPICFG_SPIDC_DQSPI | MIPICFG_DSPI_SPIX);
                    } else if (GDC_MIPICFG_DSPI_RGB888_OPT1 == s_graphics_dc_env.init_params.mipicfg_format) {
                        hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode | GDC_MIPICFG_DSPI_RGB888_OPT1 | MIPICFG_DSPI | MIPICFG_SPI3 | MIPICFG_RESX | MIPICFG_FRC_CSX_0 | MIPICFG_DBI_EN | MIPICFG_SPIDC_DQSPI);
                    }
                    dc_enable_frame_irq();
                    hal_gdc_set_mode(HAL_GDC_ONE_FRAME);                      /* trigger the frame transmission to start */
                    if(GDC_ACCESS_TYPE_SYNC == access_type) {
                        s_graphics_dc_env.is_async_mark = 0;
                        dc_wait_frame_end();
                        hal_gdc_reg_write(HAL_GDC_REG_DBIB_CFG, (hal_gdc_reg_read(HAL_GDC_REG_DBIB_CFG) & (~MIPICFG_FRC_CSX_0)));   /* Release CS */
                        res = GDC_FRAME_RES_SUCCESS;
                    } else {
                        s_graphics_dc_env.is_async_mark = FRAME_ASYNC_MARK_FORCE_CS;
                        res = GDC_FRAME_RES_ASYNC_WAIT;
                    }
                }
                break;

                default:
                {
                    GDC_DBG_PRINTF("Err: NOT SUPPORT!\r\n");
                    res = GDC_FRAME_RES_UNSUPPORT;
                }
                break;
            }
        }
        break;

        case GDC_QSPI_FRAME_TIMING_0:
        {
            /* Trigger frame transmission */
            switch(dc_get_out_pixel_bits()) {
                case GDC_OUT_PIXEL_BITS_16:
                case GDC_OUT_PIXEL_BITS_24:
                {
                    hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode |  MIPICFG_QSPI | MIPICFG_SPI4 | MIPICFG_DBI_EN  | MIPICFG_RESX | s_graphics_dc_env.init_params.mipicfg_format | MIPICFG_DIS_TE | MIPICFG_SPI_HOLD);
                    hal_gdc_MIPI_out    ( MIPI_DBIB_CMD | MIPI_MASK_QSPI | dc_cmd->command);
                    hal_gdc_MIPI_out    ( MIPI_DBIB_CMD | MIPI_MASK_QSPI | dc_cmd->address_width | dc_cmd->address);
                    dc_enable_frame_irq();
                    hal_gdc_set_mode(HAL_GDC_ONE_FRAME);
                    if(GDC_ACCESS_TYPE_SYNC == access_type) {
                        s_graphics_dc_env.is_async_mark = 0;
                        dc_wait_frame_end();
                        hal_gdc_reg_write(HAL_GDC_REG_DBIB_CFG, (hal_gdc_reg_read(HAL_GDC_REG_DBIB_CFG) & (~MIPICFG_SPI_HOLD)));
                        res = GDC_FRAME_RES_SUCCESS;
                    } else {
                        s_graphics_dc_env.is_async_mark = FRAME_ASYNC_MARK_SPI_HOLD;
                        res = GDC_FRAME_RES_ASYNC_WAIT;
                    }
                }
                break;
                default:
                {
                    GDC_DBG_PRINTF("Err: NOT SUPPORT!\r\n");
                    res = GDC_FRAME_RES_UNSUPPORT;
                }
                break;
            }
        }
        break;

        case GDC_QSPI_FRAME_TIMING_1:
        {
            switch(dc_get_out_pixel_bits()) {
                case GDC_OUT_PIXEL_BITS_16:
                {
                    //hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode |  MIPICFG_QSPI | MIPICFG_SPI4 | MIPICFG_DBI_EN  | MIPICFG_RESX | MIPICFG_4RGB888_OPT0 | MIPICFG_DIS_TE | MIPICFG_FRC_CSX_0);
                    hal_gdc_MIPI_CFG_out(MIPICFG_SPI_CPOL |  MIPICFG_QSPI | MIPICFG_SPI4 | MIPICFG_DBI_EN  | MIPICFG_RESX | MIPICFG_4RGB888_OPT0 | MIPICFG_DIS_TE | MIPICFG_FRC_CSX_0);
                    hal_gdc_MIPI_out(MIPI_DBIB_CMD | MIPI_MASK_QSPI | dc_cmd->command);
                    hal_gdc_MIPI_out(MIPI_DBIB_CMD | MIPI_CMD24     | dc_cmd->address);
                    delay_us(5);
                    hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode |  MIPICFG_QSPI | MIPICFG_SPI4 | MIPICFG_DBI_EN  | MIPICFG_RESX | s_graphics_dc_env.init_params.mipicfg_format | MIPICFG_DIS_TE | MIPICFG_FRC_CSX_0);
                    dc_enable_frame_irq();
                    hal_gdc_set_mode(HAL_GDC_ONE_FRAME);                      /* trigger the frame transmission to start */

                    if(GDC_ACCESS_TYPE_SYNC == access_type) {
                        s_graphics_dc_env.is_async_mark = 0;
                        dc_wait_frame_end();
                        hal_gdc_reg_write(HAL_GDC_REG_DBIB_CFG, (hal_gdc_reg_read(HAL_GDC_REG_DBIB_CFG) & (~MIPICFG_FRC_CSX_0)));   /* Release CS */
                        res = GDC_FRAME_RES_SUCCESS;
                    } else {
                        s_graphics_dc_env.is_async_mark = FRAME_ASYNC_MARK_FORCE_CS;
                        res = GDC_FRAME_RES_ASYNC_WAIT;
                    }
                }
                break;

                case GDC_OUT_PIXEL_BITS_24:
                {
                    hal_gdc_MIPI_CFG_out(s_graphics_dc_env.clock_mode |  MIPICFG_QSPI | MIPICFG_SPI4 | MIPICFG_DBI_EN  | MIPICFG_RESX | s_graphics_dc_env.init_params.mipicfg_format | MIPICFG_DIS_TE | MIPICFG_SPI_HOLD);
                    hal_gdc_MIPI_out    ( MIPI_DBIB_CMD | MIPI_MASK_QSPI | dc_cmd->command);
                    hal_gdc_MIPI_out    ( MIPI_DBIB_CMD | MIPI_CMD24     | dc_cmd->address);
                    dc_enable_frame_irq();
                    hal_gdc_set_mode(HAL_GDC_ONE_FRAME);
                    if(GDC_ACCESS_TYPE_SYNC == access_type) {
                        s_graphics_dc_env.is_async_mark = 0;
                        dc_wait_frame_end();
                        hal_gdc_reg_write(HAL_GDC_REG_DBIB_CFG, (hal_gdc_reg_read(HAL_GDC_REG_DBIB_CFG) & (~MIPICFG_SPI_HOLD)));
                        res = GDC_FRAME_RES_SUCCESS;
                    } else {
                        s_graphics_dc_env.is_async_mark = FRAME_ASYNC_MARK_SPI_HOLD;
                        res = GDC_FRAME_RES_ASYNC_WAIT;
                    }
                }
                break;

                default:
                {
                    GDC_DBG_PRINTF("Err: NOT SUPPORT!\r\n");
                    res = GDC_FRAME_RES_UNSUPPORT;
                }
                break;
            }
            /* Trigger frame transmission */
        }
        break;

        default:
        {
            GDC_DBG_PRINTF("Err: NOT SUPPORT!\r\n");
            res = GDC_FRAME_RES_UNSUPPORT;
        }
        break;
    }

    return res;
}


/*
 * STATIC METHODS
 *****************************************************************************************
 */
static uint16_t dc_pins_init(app_graphics_dc_pins_t pins)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.mode = APP_IO_MODE_MUX;

    /* csn */
    if(pins.csn.enable) {
        io_init.pull = pins.csn.pull;
        io_init.pin  = GRAPHICS_DC_CSN_PIN;
        io_init.mux  = GRAPHICS_DC_CSN_PIN_MUX;
        err_code = app_io_init(GRAPHICS_DC_CSN_PORT, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    /* clk */
    if(pins.clk.enable) {
        io_init.pull = pins.clk.pull;
        io_init.pin  = GRAPHICS_DC_CLK_PIN;
        io_init.mux  = GRAPHICS_DC_CLK_PIN_MUX;
        err_code = app_io_init(GRAPHICS_DC_CLK_PORT, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    /* io0 */
    if(pins.io0.enable) {
        io_init.pull = pins.io0.pull;
        io_init.pin  = GRAPHICS_DC_IO0_PIN;
        io_init.mux  = GRAPHICS_DC_IO0_PIN_MUX;
        err_code = app_io_init(GRAPHICS_DC_IO0_PORT, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    /* io1 */
    if(pins.io1.enable) {
        io_init.pull = pins.io1.pull;
        io_init.pin  = GRAPHICS_DC_IO1_PIN;
        io_init.mux  = GRAPHICS_DC_IO1_PIN_MUX;
        err_code = app_io_init(GRAPHICS_DC_IO1_PORT, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    /* io2 */
    if(pins.io2.enable) {
        io_init.pull = pins.io2.pull;
        io_init.pin  = GRAPHICS_DC_IO2_PIN;
        io_init.mux  = GRAPHICS_DC_IO2_PIN_MUX;
        err_code = app_io_init(GRAPHICS_DC_IO2_PORT, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    /* io3 */
    if(pins.io3.enable) {
        io_init.pull = pins.io3.pull;
        io_init.pin  = GRAPHICS_DC_IO3_PIN;
        io_init.mux  = GRAPHICS_DC_IO3_PIN_MUX;
        err_code = app_io_init(GRAPHICS_DC_IO3_PORT, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    /* dcx */
    if(pins.dcx.enable) {
        io_init.pull = pins.dcx.pull;
        io_init.pin  = GRAPHICS_DC_DCX_PIN;
        io_init.mux  = GRAPHICS_DC_DCX_PIN_MUX;
        err_code = app_io_init(GRAPHICS_DC_DCX_PORT, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    return err_code;
}

static void dc_pins_deinit(void) {
    if(s_graphics_dc_env.init_params.pins_cfg.csn.enable)   app_io_deinit(GRAPHICS_DC_CSN_PORT, GRAPHICS_DC_CSN_PIN);
    if(s_graphics_dc_env.init_params.pins_cfg.clk.enable)   app_io_deinit(GRAPHICS_DC_CLK_PORT, GRAPHICS_DC_CLK_PIN);
    if(s_graphics_dc_env.init_params.pins_cfg.io0.enable)   app_io_deinit(GRAPHICS_DC_IO0_PORT, GRAPHICS_DC_IO0_PIN);
    if(s_graphics_dc_env.init_params.pins_cfg.io1.enable)   app_io_deinit(GRAPHICS_DC_IO1_PORT, GRAPHICS_DC_IO1_PIN);
    if(s_graphics_dc_env.init_params.pins_cfg.io2.enable)   app_io_deinit(GRAPHICS_DC_IO2_PORT, GRAPHICS_DC_IO2_PIN);
    if(s_graphics_dc_env.init_params.pins_cfg.io3.enable)   app_io_deinit(GRAPHICS_DC_IO3_PORT, GRAPHICS_DC_IO3_PIN);
    if(s_graphics_dc_env.init_params.pins_cfg.dcx.enable)   app_io_deinit(GRAPHICS_DC_DCX_PORT, GRAPHICS_DC_DCX_PIN);

    return;
}

static void dc_clock_on(graphics_dc_clock_freq_e clock) {
    MCU_RET->MCU_MISC_CLK = (MCU_RET->MCU_MISC_CLK) & 0x3E;
    hal_gdc_reg_write(HAL_GDC_REG_CLKCTRL_CG, 0x01);
    hal_gdc_clkdiv(0, (uint32_t)clock, 4, 0);
}

static void dc_clock_off(void) {
    MCU_RET->MCU_MISC_CLK = (MCU_RET->MCU_MISC_CLK) | 1u;
}

void app_graphics_dc_sleep(void)
{
    MCU_RET->MCU_MISC_CLK = (MCU_RET->MCU_MISC_CLK) | 1u;
}

static void dc_enable_cmd_irq(void) {
    s_graphics_dc_env.is_cmd_cplt = 0;
    GDC_ENABLE_CMD_END_IRQ();
}

static void dc_wait_cmd_irq(void) {
    while(!s_graphics_dc_env.is_cmd_cplt);
}

static void dc_enable_frame_irq(void) {
    s_graphics_dc_env.is_frame_cplt = 0;
    GDC_ENABLE_FRAME_END_IRQ();
}

static void dc_wait_frame_end(void)
{
    while(!s_graphics_dc_env.is_frame_cplt);
}

static void dc_wait_cs_deassert(void) {
    while( (hal_gdc_reg_read(HAL_GDC_REG_STATUS) & 0x15C00) != 0) {}
    delay_us(1);
    return;
}

static graphics_dc_out_pixel_bits_e dc_get_out_pixel_bits(void) {

    graphics_dc_out_pixel_bits_e o_pixel_bits = GDC_OUT_PIXEL_BITS_NOT_SUPPORT;

    if(!s_graphics_dc_env.is_inited) {
        return GDC_OUT_PIXEL_BITS_NOT_SUPPORT;
    }

    switch(s_graphics_dc_env.init_params.mipicfg_format) {
        case GDC_MIPICFG_SPI_RGB565_OPT0  :
        case GDC_MIPICFG_DSPI_RGB565_OPT0 :
        case GDC_MIPICFG_QSPI_RGB565_OPT0 :
        {
            o_pixel_bits = GDC_OUT_PIXEL_BITS_16;
        }
        break;

        case GDC_MIPICFG_SPI_RGB888_OPT0  :
        case GDC_MIPICFG_DSPI_RGB888_OPT0 :
        case GDC_MIPICFG_DSPI_RGB888_OPT1 :
        case GDC_MIPICFG_QSPI_RGB888_OPT0 :
        {
            o_pixel_bits = GDC_OUT_PIXEL_BITS_24;
        }
        break;

        default: break;
    }

    return o_pixel_bits;
}

static void dc_set_clock_mode(graphics_dc_clock_mode_e mode) {

    if(mode > GDC_CLOCK_MODE_3)
    {
        return;
    }

    s_graphics_dc_env.clock_mode = 0;

    switch(mode) {
        case GDC_CLOCK_MODE_0:
        {
            GDC_CLR_Bit(HAL_GDC_REG_DBIB_CFG, MIPICFG_SPI_CPOL);
            GDC_CLR_Bit(HAL_GDC_REG_DBIB_CFG, MIPICFG_SPI_CPHA);
            s_graphics_dc_env.clock_mode = 0x00;
        }
        break;

        case GDC_CLOCK_MODE_1:
        {
            GDC_CLR_Bit(HAL_GDC_REG_DBIB_CFG, MIPICFG_SPI_CPOL);
            GDC_SET_Bit(HAL_GDC_REG_DBIB_CFG, MIPICFG_SPI_CPHA);
            s_graphics_dc_env.clock_mode = MIPICFG_SPI_CPHA;
        }
        break;

        case GDC_CLOCK_MODE_2:
        {
            GDC_SET_Bit(HAL_GDC_REG_DBIB_CFG, MIPICFG_SPI_CPOL);
            GDC_CLR_Bit(HAL_GDC_REG_DBIB_CFG, MIPICFG_SPI_CPHA);
            s_graphics_dc_env.clock_mode = MIPICFG_SPI_CPOL;
        }
        break;

        case GDC_CLOCK_MODE_3:
        {
            GDC_SET_Bit(HAL_GDC_REG_DBIB_CFG, MIPICFG_SPI_CPOL);
            GDC_SET_Bit(HAL_GDC_REG_DBIB_CFG, MIPICFG_SPI_CPHA);
            s_graphics_dc_env.clock_mode = MIPICFG_SPI_CPOL | MIPICFG_SPI_CPHA;
        }
        break;
    }

    return;
}

static void dc_set_tcsu_delay_cycle(graphics_dc_tcsu_cycle_e delay) {
    if(delay > GDC_TCSU_CYCLE_4) {
        return;
    }

    GDC_SET_Bit(HAL_GDC_REG_FORMAT_CTRL3, (delay << 10) | (delay << 13));
    return;
}

static void dc_irq_event_callback(uint32_t evt) {
    if(evt == GDC_IRQ_EVT_CMD_TRANSMITION_END) {
        s_graphics_dc_env.is_cmd_cplt = 1;
    } else if(evt == GDC_IRQ_EVT_FRAME_TRANSMITION_END) {
        if(FRAME_ASYNC_MARK_SPI_HOLD == s_graphics_dc_env.is_async_mark) {
            hal_gdc_reg_write(HAL_GDC_REG_DBIB_CFG, (hal_gdc_reg_read(HAL_GDC_REG_DBIB_CFG) & (~MIPICFG_SPI_HOLD)));
        } else if(FRAME_ASYNC_MARK_FORCE_CS == s_graphics_dc_env.is_async_mark) {
            hal_gdc_reg_write(HAL_GDC_REG_DBIB_CFG, (hal_gdc_reg_read(HAL_GDC_REG_DBIB_CFG) & (~MIPICFG_FRC_CSX_0)));
        }

        s_graphics_dc_env.is_frame_cplt   = 1;
    }

    if(s_graphics_dc_env.irq_evt_cb != NULL) {
        s_graphics_dc_env.irq_evt_cb(evt);
    }
}

/*
 * Porting APIs
 *****************************************************************************************
 */

/* Called by hal_gdc_init */
int32_t hal_gdc_sys_init(void) {
    return 0;
}

void hal_gdc_wait_vsync(void)
{
    if ( hal_gdc_reg_read(HAL_GDC_REG_STATUS) == 0U )
    {
        return;
    }

    /* Schedule an interrupt on next FRAME-END */
    hal_gdc_reg_write(HAL_GDC_REG_INTERRUPT, 1 << 4);

    while( hal_gdc_reg_read( HAL_GDC_REG_INTERRUPT ) != 0U ) {}
}

uint32_t  hal_gdc_reg_read(uint32_t reg)
{
    return *((volatile uint32_t *)(GRAPHICS_DC_BASEADDR + reg));
}

void hal_gdc_reg_write(uint32_t reg, uint32_t value)
{
    *((volatile uint32_t *)(GRAPHICS_DC_BASEADDR + reg)) = value;
}

/*
 * IRQ Service
 *****************************************************************************************
 */
void TSI_DC_IRQHandler(void) {
    uint32_t irq = hal_gdc_reg_read(HAL_GDC_REG_INTERRUPT);

    if(irq & DC_IRQ_CMD_END_BIT) {
        dc_irq_event_callback(GDC_IRQ_EVT_CMD_TRANSMITION_END);
    } else if(irq & DC_IRQ_FRAME_END_BIT) {
        dc_irq_event_callback(GDC_IRQ_EVT_FRAME_TRANSMITION_END);
    }

    /* Clear the interrupt */
    hal_gdc_reg_write(HAL_GDC_REG_INTERRUPT, 0);

    if(irq & DC_IRQ_FRAME_END_BIT)
    {
        app_graphics_dc_set_power_state(GDC_POWER_STATE_SLEEP);
    }
}

static int _dc_irq_sem_init(void) {
#if GPU_WAIT_IRQ_USING_SEMAPHORE > 0u
    #if CONFIG_ZEPHYR_OS
        int ret;
        ret = k_sem_init(&s_dc_irq_sem, 0, 1);
        return ret;
    #else
        #ifdef USE_OSAL
            if (OSAL_SUCCESS == osal_sema_binary_create(&s_dc_irq_sem)) {
                return 1;
            }
        #else
            if(APP_DRV_SUCCESS == app_driver_sem_init(&s_dc_irq_sem)) {
                    return 1;
            }
        #endif
    #endif
    return 0;
#else
    return 0;
#endif
}

static int _dc_irq_sem_take(void) {
#if GPU_WAIT_IRQ_USING_SEMAPHORE > 0u
    s_dc_sem_take_cnt++;
    #if CONFIG_ZEPHYR_OS
        k_sem_take(&s_dc_irq_sem, K_MSEC(GPU_WAIT_TIMEOUT_MS));
    #else
        #ifdef USE_OSAL
            osal_sema_take(s_dc_irq_sem, GPU_WAIT_TIMEOUT_MS);
        #else
            app_driver_sem_pend(s_dc_irq_sem, GPU_WAIT_TIMEOUT_MS);
        #endif
    #endif
    return 1;
#else
    return 0;
#endif
}

static int _dc_irq_sem_give(void) {
#if GPU_WAIT_IRQ_USING_SEMAPHORE > 0u
    s_dc_sem_give_cnt++;
    #if CONFIG_ZEPHYR_OS
        k_sem_give(&s_dc_irq_sem);
    #else
        #ifdef USE_OSAL
            osal_sema_give(s_dc_irq_sem);
        #else
            app_driver_sem_post_from_isr((sem_t)s_dc_irq_sem);
        #endif
    #endif
    return 1;
#else
    return 0;
#endif
}


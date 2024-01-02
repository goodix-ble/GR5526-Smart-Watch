#include "app_graphics_dc.h"
#include "app_qspi.h"
#include "app_qspi_dma.h"

#ifdef USE_OSAL
    #include "osal.h"
#else
    #include "FreeRTOS.h"
    #include "semphr.h"
#endif

#include "graphics_hybrid_rm69330_drv.h"

#include <stdarg.h>

#define DC_SPI_SEND_CMD(cmd)            \
    do                                  \
    {                                   \
        dc_spi_send_sync(cmd, NULL, 0); \
    } while (0)

#define DC_SPI_SEND_CMD_1P(cmd, p1)       \
    do                                   \
    {                                    \
        uint8_t data = p1;               \
        dc_spi_send_sync(cmd, &data, 1); \
    } while (0)

#define DC_SPI_SEND_CMD_2P(cmd, p1, p2) \
    do                                  \
    {                                   \
        uint8_t data[2] = {p1, p2};     \
        dc_spi_send_sync(cmd, data, 2); \
    } while (0)

typedef enum
{
    DISP_IFCE_DC = 0,
    DISP_IFCE_QSPI,
    DISP_IFCE_MAX,
} disp_ifce_t;

static disp_ifce_t s_disp_ifce = DISP_IFCE_DC;
static uint32_t s_qspi_tailbeats;
static uint16_t *s_qspi_tailaddr;
static dma_block_config_t *s_qspi_llp_nodes;
#ifdef USE_OSAL
static osal_sema_handle_t s_qspi_flush_sema;
#else
static SemaphoreHandle_t s_qspi_flush_sema;
#endif

static qspi_command_t s_qspi_flush_cmd = {
    .instruction = 0x12,
    .instruction_size = QSPI_INSTSIZE_08_BITS,
    .address = 0x002C00,
    .address_size = QSPI_ADDRSIZE_24_BITS,
    .data_size = QSPI_DATASIZE_16_BITS,
    .instruction_address_mode = QSPI_INST_IN_SPI_ADDR_IN_SPIFRF,
    .dummy_cycles = 0,
    .data_mode = QSPI_DATA_MODE_QUADSPI,
    .clock_stretch_en = LL_QSPI_CLK_STRETCH_ENABLE,
};

static app_qspi_params_t s_qspi_param = {
    .id = APP_QSPI_ID_2,
    .pin_cfg = {
        .cs = {
            .type = APP_IO_TYPE_GPIOB,
            .mux = APP_IO_MUX_0,
            .pin = APP_IO_PIN_11,
            .mode = APP_IO_MODE_MUX,
            .pull = APP_IO_NOPULL,
            .enable = 1,
        },
        .clk = {
            .type = APP_IO_TYPE_GPIOB,
            .mux = APP_IO_MUX_0,
            .pin = APP_IO_PIN_0,
            .mode = APP_IO_MODE_MUX,
            .pull = APP_IO_NOPULL,
            .enable = 1,
        },
        .io_0 = {
            .type = APP_IO_TYPE_GPIOB,
            .mux = APP_IO_MUX_0,
            .pin = APP_IO_PIN_1,
            .mode = APP_IO_MODE_MUX,
            .pull = APP_IO_NOPULL,
            .enable = 1,
        },
        .io_1 = {
            .type = APP_IO_TYPE_GPIOB,
            .mux = APP_IO_MUX_0,
            .pin = APP_IO_PIN_2,
            .mode = APP_IO_MODE_MUX,
            .pull = APP_IO_NOPULL,
            .enable = 1,
        },
        .io_2 = {
            .type = APP_IO_TYPE_GPIOB,
            .mux = APP_IO_MUX_0,
            .pin = APP_IO_PIN_3,
            .mode = APP_IO_MODE_MUX,
            .pull = APP_IO_NOPULL,
            .enable = 1,
        },
        .io_3 = {
            .type = APP_IO_TYPE_GPIOB,
            .mux = APP_IO_MUX_0,
            .pin = APP_IO_PIN_4,
            .mode = APP_IO_MODE_MUX,
            .pull = APP_IO_NOPULL,
            .enable = 1,
        },
    },
    .dma_cfg = {
        .dma_instance = DMA1,
        .dma_channel = DMA_Channel0,
        .wait_timeout_ms = 3000,
        .extend = 0,
    },
    .init = {
        .clock_mode = QSPI_CLOCK_MODE_3,
        .clock_prescaler = 2,
        .rx_sample_delay = 0,
    },
};

static void rm69330_disp_init(void);
static void lcd_initial_sequence(void);
static void dc_spi_send_sync(uint32_t cmd, uint8_t *data, uint32_t data_len);
static void qspi_spi_send_sync(uint32_t cmd, uint8_t *data, uint32_t data_len);
static void switch_display_interface(disp_ifce_t ifce);
static void qspi_event_cb(app_qspi_evt_t *evt);
static void dc_event_cb(uint32_t evt);

void graphics_hybrid_rm69330_init(uint16_t screen_w, uint16_t screen_h, graphics_dc_mipi_format_e mipi_format)
{
    // Init QSPI
    app_qspi_init(&s_qspi_param, qspi_event_cb);
    app_qspi_dma_init(&s_qspi_param);

#ifdef USE_OSAL
    // Init QSPI flush semaphore
    osal_sema_binary_create(&s_qspi_flush_sema);
    osal_sema_give(s_qspi_flush_sema);

    // Create poll for QSPI DMA LLP nodes
    s_qspi_llp_nodes = (dma_block_config_t *)osal_heap_malloc((1 + (screen_w * screen_h / 4092)) * sizeof(dma_block_config_t));
#else
    // Init QSPI flush semaphore
    s_qspi_flush_sema = xSemaphoreCreateBinary();
    xSemaphoreGive(s_qspi_flush_sema);

    // Create poll for QSPI DMA LLP nodes
    s_qspi_llp_nodes = (dma_block_config_t *)pvPortMalloc((1 + (screen_w * screen_h / 4092)) * sizeof(dma_block_config_t));
#endif

    // Init DC
    app_graphics_dc_params_t dc_param = {
        .mspi_mode = GDC_MODE_QSPI,
        .clock_freq = GDC_CLOCK_FREQ_48MHz,
        .clock_mode = GDC_CLOCK_MODE_0,
        .tcsu_cycle = GDC_TCSU_CYCLE_1,
        .layer_mode = GDC_ONE_LAYER_MODE,
        .mipicfg_format = mipi_format,
        .resolution_x = screen_w,
        .resolution_y = screen_h,
        .pins_cfg = {
            .csn = {
                .pull = APP_IO_PULLUP,
                .enable = ENABLE,
            },
            .clk = {
                .pull = APP_IO_PULLUP,
                .enable = ENABLE,
            },
            .io0 = {
                .pull = APP_IO_PULLUP,
                .enable = ENABLE,
            },
            .io1 = {
                .pull = APP_IO_PULLUP,
                .enable = ENABLE,
            },
            .io2 = {
                .pull = APP_IO_PULLUP,
                .enable = ENABLE,
            },
            .io3 = {
                .pull = APP_IO_PULLUP,
                .enable = ENABLE,
            },
            .dcx = {
                .pull = APP_IO_PULLUP,
                .enable = DISABLE,
            },
        },
    };
    graphics_dc_init(&dc_param, dc_event_cb);

    // Init Display module
    rm69330_disp_init();
}

void graphics_hybrid_rm69330_deinit(void)
{
    // TODO: Deinit
}

void graphics_hybrid_rm69330_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
    x1 += 12;
    x2 += 12;
    uint8_t data_2a[4] = {
        (x1 & 0xFF00) >> 8,
        x1 & 0x00FF,
        (x2 & 0xFF00) >> 8,
        x2 & 0x00FF,
    };

    uint8_t data_2b[4] = {
        (y1 & 0xFF00) >> 8,
        y1 & 0x00FF,
        (y2 & 0xFF00) >> 8,
        y2 & 0x00FF,
    };

    if (s_disp_ifce == DISP_IFCE_DC)
    {
        dc_spi_send_sync(0x2A, data_2a, 4);
        dc_spi_send_sync(0x2B, data_2b, 4);
    }
    else
    {
        qspi_spi_send_sync(0x2A, data_2a, 4);
        qspi_spi_send_sync(0x2B, data_2b, 4);
    }
}

void graphics_hybrid_rm69330_flush(void *buf, uint32_t buf_format, uint16_t w, uint16_t h)
{
    if (buf_format == GDC_DATA_FORMAT_RGB565)
    {
        switch_display_interface(DISP_IFCE_QSPI);
    }
    else
    {
        switch_display_interface(DISP_IFCE_DC);
    }

    if (s_disp_ifce == DISP_IFCE_DC)
    {
        app_graphics_dc_cmd_t dc_cmd;

        dc_cmd.command = 0x12;
        dc_cmd.address = 0x002C00;
        dc_cmd.address_width = GDC_FRAME_ADDRESS_WIDTH_24BIT;
        dc_cmd.frame_timing = GDC_QSPI_FRAME_TIMING_1;

        app_graphics_dc_framelayer_t dc_layer = {
            .frame_baseaddr = (void *)buf,
            .resolution_x = w,
            .resolution_y = h,
            .row_stride = -1,
            .start_x = 0,
            .start_y = 0,
            .size_x = w,
            .size_y = h,
            .alpha = 0,
            .blendmode = HAL_GDC_BL_SRC,
            .data_format = (graphics_dc_data_format_e)buf_format,
        };

        app_graphics_dc_send_single_frame(GRAPHICS_DC_LAYER_0, &dc_layer, &dc_cmd, GDC_ACCESS_TYPE_ASYNC);
    }
    else if (s_disp_ifce == DISP_IFCE_QSPI)
    {
        uint32_t beats = w * h;
        uint32_t beats_rounded = beats / 0xFFFE;
        beats_rounded *= 0xFFFE;
        s_qspi_tailbeats = beats - beats_rounded;
        uint8_t i = 0;
        uint16_t *src_16 = (uint16_t *)buf;

        s_qspi_flush_cmd.address = 0x002C00;
        s_qspi_flush_cmd.length = beats_rounded * 2;

        while (beats_rounded > 0)
        {
            uint32_t node_size = beats_rounded > 4092 ? 4092 : beats_rounded;
            s_qspi_llp_nodes[i].CTL_L = DMA_CTLL_INI_EN | DMA_SRC_INCREMENT | DMA_DST_NO_CHANGE | DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_DISABLE | DMA_LLP_SRC_ENABLE | DMA_LLP_DST_DISABLE | DMA_MEMORY_TO_PERIPH | LL_DMA_SRC_BURST_LENGTH_16 | LL_DMA_DST_BURST_LENGTH_16 | DMA_SDATAALIGN_HALFWORD | DMA_DDATAALIGN_HALFWORD;
            s_qspi_llp_nodes[i].CTL_H = node_size;
            s_qspi_llp_nodes[i].src_address = (uint32_t)src_16;
            s_qspi_llp_nodes[i].src_status = 0;
            s_qspi_llp_nodes[i].dst_address = 0;
            s_qspi_llp_nodes[i].dst_status = 0;
            s_qspi_llp_nodes[i].p_lli = NULL;

            if (i > 0)
            {
                s_qspi_llp_nodes[i - 1].p_lli = &s_qspi_llp_nodes[i];
            }

            src_16 += node_size;
            i++;
            beats_rounded -= node_size;
        }
        s_qspi_tailaddr = src_16;

        dma_llp_config_t llp_config = {
            .llp_src_en = DMA_LLP_SRC_ENABLE,
            .llp_dst_en = DMA_LLP_DST_DISABLE,
            .head_lli = s_qspi_llp_nodes,
        };

        hal_qspi_command_transmit_dma_llp(&s_qspi_param.qspi_env.handle, &s_qspi_flush_cmd, &llp_config);
    }
}

void graphics_hybrid_rm69330_set_on(bool on)    
{
    if (s_disp_ifce == DISP_IFCE_DC)
    {
        app_graphics_dc_set_power_state(GDC_POWER_STATE_ACTIVE);
        if(on) {
            DC_SPI_SEND_CMD(0x29);
        } else {
            DC_SPI_SEND_CMD(0x28);
        }
        app_graphics_dc_set_power_state(GDC_POWER_STATE_SLEEP);
    }
    else if (s_disp_ifce == DISP_IFCE_QSPI)
    {
        switch_display_interface(DISP_IFCE_DC);

        app_graphics_dc_set_power_state(GDC_POWER_STATE_ACTIVE);
        if(on) {
            DC_SPI_SEND_CMD(0x29);
        } else {
            DC_SPI_SEND_CMD(0x28);
        }
//        app_graphics_dc_set_power_state(GDC_POWER_STATE_SLEEP);

//        switch_display_interface(DISP_IFCE_QSPI);
    }
}

void graphics_hybrid_rm69330_wait_ready(void)
{
    if (s_disp_ifce == DISP_IFCE_DC)
    {
        app_graphics_dc_set_power_state(GDC_POWER_STATE_ACTIVE);
    }
    else if (s_disp_ifce == DISP_IFCE_QSPI)
    {
#ifdef USE_OSAL
        osal_sema_take(s_qspi_flush_sema, 1000);
#else
        xSemaphoreTake(s_qspi_flush_sema, 1000);
#endif
    }
}

void graphics_hybrid_rm69330_sleep(void)
{
    // TODO: RM69330 Deep Sleep & Always On Display
}

void graphics_hybrid_rm69330_wakeup(void)
{
    // TODO: RM69330 Wakeup
}

static void rm69330_disp_init(void)
{
    // Do hardware reset
    app_io_init_t reset_pin = {
        .pin = APP_IO_PIN_6,
        .mode = APP_IO_MODE_OUTPUT,
        .pull = APP_IO_NOPULL,
        .mux = APP_IO_MUX,
    };
    app_io_init(APP_IO_TYPE_AON, &reset_pin);
    app_io_write_pin(APP_IO_TYPE_AON, APP_IO_PIN_6, APP_IO_PIN_RESET);
    delay_ms(100);
    app_io_write_pin(APP_IO_TYPE_AON, APP_IO_PIN_6, APP_IO_PIN_SET);
    delay_ms(100);

    // Initial Sequences
    lcd_initial_sequence();
}

static void lcd_initial_sequence(void)
{
#define TE_SCANLINE 300

    DC_SPI_SEND_CMD(0x11);
    delay_ms(5);
    DC_SPI_SEND_CMD_1P(0x36, 0x00);                                   // MADCTR: Scan Direction Control
    DC_SPI_SEND_CMD_1P(0x3A, 0x75);                                   // COLMOD: Interface Pixel Format
    DC_SPI_SEND_CMD_1P(0x51, 0xFF);                                   // WRDISBV: Write Display Brightness
    DC_SPI_SEND_CMD_1P(0xC4, 0x80);                                   // SetDSPIMode: Set DSPI Mode
    DC_SPI_SEND_CMD_2P(0x44, (TE_SCANLINE) >> 8, (TE_SCANLINE)&0xFF); // STESL: Set Tear Scanline
    DC_SPI_SEND_CMD_1P(0x35, 0x00);                                   // TEON: Tearing Effect Line On
    DC_SPI_SEND_CMD_1P(0x2840, 0x30);                                 // Unknown: Adjust Refresh Rate
    delay_ms(20);
    DC_SPI_SEND_CMD(0x29);
    delay_ms(20);
}

static void dc_spi_send_sync(uint32_t cmd, uint8_t *data, uint32_t data_len)
{
    if ((cmd & 0xFF) == cmd)
    {
        cmd <<= 8;
    }
    app_graphics_dc_spi_send(0x02, cmd, data, data_len);
}

static void qspi_spi_send_sync(uint32_t cmd, uint8_t *data, uint32_t data_len)
{
    uint16_t ret;
    app_qspi_command_t qspi_cmd;
    qspi_cmd.instruction = 0x02;
    qspi_cmd.address = cmd << 8;
    qspi_cmd.instruction_size = QSPI_INSTSIZE_08_BITS;
    qspi_cmd.address_size = QSPI_ADDRSIZE_24_BITS;
    qspi_cmd.dummy_cycles = 0;
    qspi_cmd.data_size = QSPI_DATASIZE_08_BITS;
    qspi_cmd.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI;
    qspi_cmd.data_mode = QSPI_DATA_MODE_SPI;
    qspi_cmd.length = data_len;
    qspi_cmd.clock_stretch_en = LL_QSPI_CLK_STRETCH_ENABLE;
    uint8_t zero = 0;
#ifdef USE_OSAL
    osal_task_disable_interrupts();
    ret = app_qspi_command_transmit_sync(APP_QSPI_ID_2, &qspi_cmd, data ? data : &zero, 1000);
    osal_task_enable_interrupts();
#else
    portDISABLE_INTERRUPTS();
    ret = app_qspi_command_transmit_sync(APP_QSPI_ID_2, &qspi_cmd, data ? data : &zero, 1000);
    portENABLE_INTERRUPTS();
#endif
    if (ret != 0)
    {
        printf("QSPI Command Transmit Failed!: %d\n", ret);
    }
}

static void switch_display_interface(disp_ifce_t ifce)
{
    if (s_disp_ifce == ifce)
    {
        return;
    }

    // Switch Pinmux
    uint32_t pmx16to23 = MCU_RET->DPAD_MUX_CTL_16_23;
    pmx16to23 &= 0xFFF00000U;
    uint32_t pmx24to31 = MCU_RET->DPAD_MUX_CTL_24_31;
    pmx24to31 &= 0xFFFF0FFFU;

    if (ifce == DISP_IFCE_DC)
    {
        pmx16to23 |= 0x00011111U;
        pmx24to31 |= 0x00001000U;
        app_graphics_dc_set_power_state(GDC_POWER_STATE_ACTIVE);
    }
    else
    {
        app_graphics_dc_set_power_state(GDC_POWER_STATE_SLEEP);
    }

    MCU_RET->DPAD_MUX_CTL_16_23 = pmx16to23;
    MCU_RET->DPAD_MUX_CTL_24_31 = pmx24to31;

    s_disp_ifce = ifce;
}

static void qspi_event_cb(app_qspi_evt_t *evt)
{
#ifndef USE_OSAL
    BaseType_t xHigherPriorityTaskWoken;
#endif

    switch (evt->type)
    {
    case APP_QSPI_EVT_TX_CPLT:
        if (s_qspi_tailbeats > 0)
        {
            // Tail Transmit
            uint8_t i = 0;
            uint16_t *src_16 = s_qspi_tailaddr;

            s_qspi_flush_cmd.address = 0x003C00;
            s_qspi_flush_cmd.length = s_qspi_tailbeats * 2;

            while (s_qspi_tailbeats > 0)
            {
                uint32_t node_size = s_qspi_tailbeats > 4092 ? 4092 : s_qspi_tailbeats;
                s_qspi_llp_nodes[i].CTL_L = DMA_CTLL_INI_EN | DMA_SRC_INCREMENT | DMA_DST_NO_CHANGE | DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_DISABLE | DMA_LLP_SRC_ENABLE | DMA_LLP_DST_DISABLE | DMA_MEMORY_TO_PERIPH | LL_DMA_SRC_BURST_LENGTH_16 | LL_DMA_DST_BURST_LENGTH_16 | DMA_SDATAALIGN_HALFWORD | DMA_DDATAALIGN_HALFWORD;
                s_qspi_llp_nodes[i].CTL_H = node_size;
                s_qspi_llp_nodes[i].src_address = (uint32_t)src_16;
                s_qspi_llp_nodes[i].src_status = 0;
                s_qspi_llp_nodes[i].dst_address = 0;
                s_qspi_llp_nodes[i].dst_status = 0;
                s_qspi_llp_nodes[i].p_lli = NULL;

                if (i > 0)
                {
                    s_qspi_llp_nodes[i - 1].p_lli = &s_qspi_llp_nodes[i];
                }

                src_16 += node_size;
                i++;
                s_qspi_tailbeats -= node_size;
            }

            dma_llp_config_t llp_config = {
                .llp_src_en = DMA_LLP_SRC_ENABLE,
                .llp_dst_en = DMA_LLP_DST_DISABLE,
                .head_lli = s_qspi_llp_nodes,
            };

            hal_qspi_command_transmit_dma_llp(&s_qspi_param.qspi_env.handle, &s_qspi_flush_cmd, &llp_config);
        }
        else
        {
#ifdef USE_OSAL
            osal_sema_give(s_qspi_flush_sema);
#else
            xSemaphoreGiveFromISR(s_qspi_flush_sema, &xHigherPriorityTaskWoken);
#endif
        }
        break;

    case APP_QSPI_EVT_ERROR:
    case APP_QSPI_EVT_ABORT:
        printf("QSPI ERROR!!!!!\n");
#ifdef USE_OSAL
        osal_sema_give(s_qspi_flush_sema);
#else
        xSemaphoreGiveFromISR(s_qspi_flush_sema, &xHigherPriorityTaskWoken);
#endif
        break;

    default:
        break;
    }

#ifndef USE_OSAL
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif
}

static void dc_event_cb(uint32_t evt)
{
}

#include "app_qspi.h"
#include "app_qspi_dma.h"
#include "app_log.h"
#include "app_gpiote.h"
#include "app_io.h"
#include "amo139_display_454.h"


#define SCREEN_QSPI_ID                           APP_QSPI_ID_2

#define DEFAULT_SCRN_MODE_CONFIG                 {DMA1, DMA_Channel0, 3000, 0}
#define DEFAULT_SCRN_QSPI_CONFIG                 {16, QSPI_CLOCK_MODE_3, 0}
#define DEFAULT_SCRN_PARAM_CONFIG                {SCREEN_QSPI_ID, g_qspi_pin_groups[QSPI2_PIN_GROUP_0], DEFAULT_SCRN_MODE_CONFIG, DEFAULT_SCRN_QSPI_CONFIG}

/* misc pins */
#if 0 // EVB_BOARD
    #define DISP_RST_CONFIG                  { APP_IO_TYPE_AON, APP_IO_PIN_5, APP_IO_MODE_OUTPUT, APP_IO_NOPULL, NULL}
    #define DISP_TE_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_1, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
    #define DISP_DC_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_6, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
#else //SK Board
    #define DISP_RST_CONFIG                  { APP_IO_TYPE_AON, APP_IO_PIN_6, APP_IO_MODE_OUTPUT,  APP_IO_NOPULL, NULL}
    #define DISP_TE_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_5, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
    #define DISP_DC_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_7, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
#endif


#define LCD_QSPI_MODE       1


static void lcd_ca_ra_set(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);

static  app_qspi_params_t g_qspi_screen_params;
static  app_qspi_evt_handler_t display_evt_handler;
static volatile uint8_t g_master_tdone = 0;
static volatile uint8_t g_master_rdone = 0;

static void app_qspi_callback(app_qspi_evt_t *p_evt)
{
    if (p_evt->type == APP_QSPI_EVT_TX_CPLT)
    {
        g_master_tdone = 1;
    }
    if (p_evt->type == APP_QSPI_EVT_RX_DATA)
    {
        g_master_rdone = 1;
    }
    if ((p_evt->type == APP_QSPI_EVT_ERROR) || (p_evt->type == APP_QSPI_EVT_ABORT))
    {
        g_master_tdone = 1;
        g_master_rdone = 1;
    }
    if((p_evt->type == APP_QSPI_EVT_ASYNC_WR_SCRN_CPLT) || (p_evt->type == APP_QSPI_EVT_ASYNC_WR_SCRN_FAIL))
    {
        if(display_evt_handler != NULL)
        {
            display_evt_handler(p_evt);
        }
    }
}

static bool amo139_qspi_init(app_qspi_id_t id, uint32_t clock_prescaler, app_qspi_evt_handler_t evt_handler)
{
    uint32_t psram_id = 0;
    uint16_t ret;
    app_qspi_params_t p_params = DEFAULT_SCRN_PARAM_CONFIG;

    display_evt_handler = evt_handler;

    g_qspi_screen_params = p_params;
    g_qspi_screen_params.id                    = id;
    g_qspi_screen_params.init.clock_prescaler  = clock_prescaler;

    ret = app_qspi_init(&g_qspi_screen_params, app_qspi_callback);
    if (ret != 0)
    {
        APP_LOG_ERROR("QSPI initial failed! Please check the input paraments.");
        return false;
    }

    ret = app_qspi_dma_init(&g_qspi_screen_params);
    if (ret != 0)
    {
        APP_LOG_ERROR("QSPI initial dma failed! Please check the input paraments.");
        return false;
    }

    return true;
}

void am139_screen_deinit(app_qspi_id_t id)
{
    app_qspi_deinit(id);
}

static void lcd_io_init(app_gpiote_param_t io_para)
{
    app_io_init_t io_init;
    io_init.pin  = io_para.pin;
    io_init.mode = io_para.mode;
    io_init.pull = io_para.pull;
    io_init.mux  = APP_IO_MUX_7;

    app_io_init(io_para.type, &io_init);
}

static void qspi_screen_misc_pins_init(void)
{
    app_gpiote_param_t io_para1 = DISP_RST_CONFIG;
    lcd_io_init(io_para1);
    app_gpiote_param_t io_para2 = DISP_TE_CONFIG;
    lcd_io_init(io_para2);
    app_gpiote_param_t io_para3 = DISP_DC_CONFIG;
    lcd_io_init(io_para3);
}

static void lcd_rst_ctrl(uint8_t level)
{
    app_io_pin_state_t pin_state;
    app_gpiote_param_t rst_pin = DISP_RST_CONFIG;
    app_io_type_t pin_type = rst_pin.type;
    uint32_t pin = rst_pin.pin;

    pin_state = level ? APP_IO_PIN_SET : APP_IO_PIN_RESET;
    app_io_write_pin(pin_type, pin, pin_state);
}


static void lcd_write(uint8_t cmd, uint8_t *data, uint32_t data_len)
{
    app_qspi_command_t qspi_cmd;
    qspi_cmd.instruction        = 0x02;
    qspi_cmd.address            = ((uint16_t)cmd) << 8;
    qspi_cmd.instruction_size   = QSPI_INSTSIZE_08_BITS;
    qspi_cmd.address_size       = QSPI_ADDRSIZE_24_BITS;
    qspi_cmd.dummy_cycles       = 0;
    qspi_cmd.data_size          = QSPI_DATASIZE_08_BITS;
    qspi_cmd.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI;
    qspi_cmd.data_mode          = QSPI_DATA_MODE_SPI;
    qspi_cmd.length             = data_len;
    qspi_cmd.clock_stretch_en   = LL_QSPI_CLK_STRETCH_ENABLE;

    g_master_tdone = 0;
    app_qspi_dma_command_transmit_async(SCREEN_QSPI_ID, &qspi_cmd, data);
    while(g_master_tdone == 0);
}

/* send CA and 2 parameter */
static void lcd_send_ca2p(uint8_t cmd, uint8_t p1, uint8_t p2)
{
    uint8_t p[2] = {p1, p2};
    lcd_write(cmd, &p[0], 2);
}

/* send CA and 1 parameter */
static void lcd_send_ca1p(uint8_t cmd, uint8_t param)
{
    lcd_write(cmd, &param, 1);
}

/* send CA and 0 parameter */
static void lcd_send_ca(uint8_t cmd) {
    uint8_t cmds[4] = {0x02, 0x00, 0x00, 0x00};

    cmds[2] = cmd;

    g_master_tdone = 0;
    app_qspi_dma_transmit_async(SCREEN_QSPI_ID, &cmds[0], 4);
    while(g_master_tdone == 0);

    return;
}



static void lcd_cmd_sequence(uint32_t res)
{
    switch(res) {
        case 454:
        {
#if 0 == LCD_QSPI_MODE /* SPI Mode */
            //lcd_send_ca(0x01);            /* soft reset, Don't CALL This */
            lcd_send_ca(0x11);              /* sleep out */
            delay_ms(10);
            lcd_send_ca(0x13);              /* Normal Display mode on */
            lcd_send_ca1p(0x36, 0x00);      /* RGB Order */
            lcd_send_ca1p(0x3A, 0x05);      /* IFPF: 16bit color */
            lcd_send_ca(0x47);              /* SPI read On */
            lcd_send_ca(0x49);              /* AON Mode on */

            lcd_send_ca2p(0x4A, 0xFF, 0x00);    /* Brightness Value */
            lcd_send_ca1p(0x4F, 0x00);          /* Disable deep sleep */
            lcd_send_ca2p(0x51, 0xFF, 0x00);    /* Brightness Value */
            lcd_send_ca1p(0xc4, 0x04);          /* Set 1-SPI Mode*/

            lcd_ca_ra_set(0, 453, 0, 453);
            lcd_send_ca2p(0x44, 0x01, 0xc5);    /* Set Tear Scan Line */
            lcd_send_ca1p(0x35, 0x00);          /* Tearing effect line on */
            lcd_send_ca1p(0x53, 0x20);          /* Brightness Control On & Display Dimming Off */

            lcd_send_ca(0x29);                  /* Display on */
            delay_ms(20);
#else   /* QSPI Mode */
            lcd_send_ca(0x11);              /* sleep out */
            delay_ms(10);
            //lcd_send_ca(0x13);              /* Normal Display mode on */
            lcd_send_ca1p(0x36, 0x00);      /* RGB Order */
            lcd_send_ca1p(0x3A, 0x75);      /* IFPF: 16bit color */
            //lcd_send_ca(0x47);              /* SPI read On */
            //lcd_send_ca(0x49);              /* AON Mode on */

            lcd_send_ca2p(0x4A, 0xFF, 0x03);    /* Brightness Value */
            lcd_send_ca1p(0x4F, 0x00);          /* Disable deep sleep */
            lcd_send_ca2p(0x51, 0xFF, 0x03);    /* Brightness Value */
            lcd_send_ca1p(0xc4, 0x00);          /* Set Quad-SPI Mode*/

            lcd_ca_ra_set(0, 453, 0, 453);
            lcd_send_ca2p(0x44, 0x01, 0xc5);    /* Set Tear Scan Line */
            lcd_send_ca1p(0x35, 0x00);          /* Tearing effect line on */
            lcd_send_ca1p(0x53, 0x28);          /* Brightness Control On & Display Dimming Off */

            delay_ms(20);
            lcd_send_ca(0x29);                  /* Display on */
            delay_ms(20);
#endif

        }
        break;

        default:
        {}
        break;
    }
}

static void lcd_ca_ra_set(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
    uint8_t data[4];

    data[0] = (x1 & 0xff00) >> 8;
    data[1] = x1 & 0x00ff;
    data[2] = (x2 & 0xff00) >> 8;
    data[3] = x2 & 0x00ff;
    lcd_write(0x2a, data, 4);

    data[0] = (y1 & 0xff00) >> 8;
    data[1] = y1 & 0x00ff;
    data[2] = (y2 & 0xff00) >> 8;
    data[3] = y2 & 0x00ff;
    lcd_write(0x2b, data, 4);
}

void amo139_screen_init_basic(app_qspi_id_t id, uint32_t clock_prescaler, uint32_t res, app_qspi_evt_handler_t evt_handler) {
    amo139_qspi_init(id, clock_prescaler, evt_handler);

    qspi_screen_misc_pins_init();
    lcd_rst_ctrl(0);
    delay_ms(100);
    lcd_rst_ctrl(1);
    delay_ms(100);

    lcd_cmd_sequence(res);
}


void amo139_screen_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2) {
    lcd_ca_ra_set(x1, x2, y1, y2);
}


/* Test Code */
#if 0

void qspi_am_lcd_test(void){
    app_qspi_screen_command_t   screen_cmd;
    app_qspi_screen_info_t      screen_info;
    app_qspi_screen_scroll_t    scroll_config;

#if 0 == LCD_QSPI_MODE
    screen_cmd.instruction              = 0x02;
    screen_cmd.instruction_size         = QSPI_INSTSIZE_08_BITS;
    screen_cmd.leading_address          = 0x002C00;
    screen_cmd.ongoing_address          = 0x003C00;
    screen_cmd.address_size             = QSPI_ADDRSIZE_24_BITS;
    screen_cmd.data_size                = QSPI_DATASIZE_16_BITS;
    screen_cmd.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI;
    screen_cmd.dummy_cycles             = 0;
    screen_cmd.data_mode                = QSPI_DATA_MODE_SPI;
    screen_cmd.is_one_take_cs           = true;
#else
    screen_cmd.instruction              = 0x32;
    screen_cmd.instruction_size         = QSPI_INSTSIZE_08_BITS;
    screen_cmd.leading_address          = 0x002C00;
    screen_cmd.ongoing_address          = 0x003C00;
    screen_cmd.address_size             = QSPI_ADDRSIZE_24_BITS;
    screen_cmd.data_size                = QSPI_DATASIZE_16_BITS;
    screen_cmd.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI;
    screen_cmd.dummy_cycles             = 0;
    screen_cmd.data_mode                = QSPI_DATA_MODE_QUADSPI;
    screen_cmd.is_one_take_cs           = true;
#endif

    screen_info.scrn_pixel_width  = 454;
    screen_info.scrn_pixel_height = 454;
    screen_info.scrn_pixel_depth  = 2;

    scroll_config.first_frame_start_address  = (uint32_t)&hehua_454_rgba565[0];
    scroll_config.second_frame_start_address = (uint32_t)&hehua_454_rgba565[0];                             /* Load from xqspi-flash */
    scroll_config.is_horizontal_scroll  = false;//true;

    amo139_screen_set_show_area(0, 454 - 1, 0, 454 - 1);
    scroll_config.scroll_coordinate = 0;

	/*
     * qspi_display_clear_flag();
     * app_qspi_async_draw_screen(Q_DISPLAY_QSPI_ID, Q_PSRAM_QSPI_ID, &screen_cmd, &screen_info, &scroll_config, true);
     * qspi_display_wait_cplt();
	 */
}
#endif

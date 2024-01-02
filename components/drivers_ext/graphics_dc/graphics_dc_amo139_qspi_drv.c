#include "graphics_dc_lcd_drv.h"

#pragma diag_suppress 177

/*
 * Defines
 *****************************************************************************************
 */
#define _DC_QSPI_PINS_CFG()     \
    {   \
        .csn = {APP_IO_PULLUP, ENABLE},  \
        .clk = {APP_IO_PULLUP, ENABLE},  \
        .io0 = {APP_IO_PULLUP, ENABLE},  \
        .io1 = {APP_IO_PULLUP, ENABLE},  \
        .io2 = {APP_IO_PULLUP, ENABLE},  \
        .io3 = {APP_IO_PULLUP, ENABLE},  \
        .dcx = {APP_IO_PULLUP, DISABLE}, \
    }

#define _DC_QSPI_CFG()   \
    {   \
        .mspi_mode      = GDC_MODE_QSPI,                 \
        .clock_freq     = GDC_CLOCK_FREQ_48MHz,          \
        .clock_mode     = GDC_CLOCK_MODE_0,              \
        .tcsu_cycle     = GDC_TCSU_CYCLE_1,              \
        .layer_mode     = GDC_ONE_LAYER_MODE,            \
        .mipicfg_format = GDC_MIPICFG_QSPI_RGB565_OPT0,  \
        .resolution_x   = 454,                           \
        .resolution_y   = 454,                           \
        .pins_cfg       = _DC_QSPI_PINS_CFG(),    \
    }

#if IS_SK_BOARD > 0u
    #define  LCD_RESET_PIN      APP_IO_PIN_6              /* Rest PIN in SK  Board */
    #define  LCD_RESET_GPIO     APP_IO_TYPE_AON
#else
    #define  LCD_RESET_PIN      APP_IO_PIN_5              /* Rest PIN in EVB Board */
    #define  LCD_RESET_GPIO     APP_IO_TYPE_AON
#endif



/*
 * Declaration
 *****************************************************************************************
 */
static void         fls_ams139_qspi_lcd_reset_set(uint8_t level);
static void         fls_ams139_qspi_lcd_write(uint8_t cmd, uint8_t *data, uint32_t data_len);
static void         fls_ams139_qspi_lcd_cmd_sequence(lcd_res_e res, lcd_pixel_mode_e pixel_mode);
static void         fls_ams139_qspi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode);
static void         dc_irq_event_notify(uint32_t evt);


/*
 * Public
 *****************************************************************************************
 */

void graphics_dc_am139_qspi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode, graphics_dc_mipi_format_e mipi_format) {
    app_graphics_dc_params_t dc_params = _DC_QSPI_CFG();
    dc_params.mipicfg_format = mipi_format;
    graphics_dc_init(&dc_params, dc_irq_event_notify);
    fls_ams139_qspi_lcd_init(res, pixel_mode);
}

void graphics_dc_am139_qspi_send_frame(app_graphics_dc_framelayer_t layer0, uint32_t lcd_w, uint32_t lcd_h) {

    app_graphics_dc_cmd_t   dc_cmd;

    dc_cmd.command       = 0x12;
    dc_cmd.address       = 0x002C00;
    dc_cmd.address_width = GDC_FRAME_ADDRESS_WIDTH_24BIT;
    dc_cmd.frame_timing  = GDC_QSPI_FRAME_TIMING_1;

    uint16_t x1 = (((lcd_w -  layer0.resolution_x)/2)/2)*2;
    uint16_t x2 = x1 + layer0.resolution_x - 1;
    uint16_t y1 = (((lcd_h - layer0.resolution_y)/2)/2)*2;
    uint16_t y2 =  y1 + layer0.resolution_y - 1;

    graphics_dc_am139_qspi_lcd_set_show_area(x1, x2, y1, y2);
    app_graphics_dc_send_single_frame(GRAPHICS_DC_LAYER_0, &layer0, &dc_cmd, GDC_ACCESS_TYPE_SYNC);

    return;
}


/*
 * Static
 *****************************************************************************************
 */
static void dc_irq_event_notify(uint32_t evt)
{

}

/* USE GPIO31 to control RESET */
static void fls_ams139_qspi_lcd_reset_set(uint8_t level)
{
    app_io_init_t io_init ;
    io_init.mode = APP_IO_MODE_OUTPUT ;
    io_init.mux  = APP_IO_MUX;
    io_init.pin  = LCD_RESET_PIN;
    io_init.pull = APP_IO_PULLUP;

    app_io_init(LCD_RESET_GPIO, &io_init);

    if(level) {
        app_io_write_pin(LCD_RESET_GPIO, LCD_RESET_PIN, APP_IO_PIN_SET);
    } else {
        app_io_write_pin(LCD_RESET_GPIO, LCD_RESET_PIN, APP_IO_PIN_RESET);
    }
}

static void fls_ams139_qspi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode)
{
    fls_ams139_qspi_lcd_reset_set(0);
    delay_ms(100);
    fls_ams139_qspi_lcd_reset_set(1);
    delay_ms(100);
    fls_ams139_qspi_lcd_cmd_sequence(res, pixel_mode);
}


/* send CA and 0 parameter */
static void lcd_send_ca(uint8_t cmd) {
    app_graphics_dc_spi_send(0x02, ((uint32_t)cmd) << 8, NULL, 0);
}


/* send CA and 1 parameter */
static void lcd_send_ca1p(uint8_t cmd, uint8_t param)
{
    app_graphics_dc_spi_send(0x02, ((uint32_t)cmd) << 8, &param, 1);
}


/* send CA and 2 parameter */
static void lcd_send_ca2p(uint8_t cmd, uint8_t p1, uint8_t p2)
{
    uint8_t p[2] = {p1, p2};

    app_graphics_dc_spi_send(0x02, ((uint32_t)cmd) << 8, &p[0], 2);
}


static void fls_ams139_qspi_lcd_write(uint8_t cmd, uint8_t *data, uint32_t data_len)
{
    app_graphics_dc_spi_send(0x02, ((uint32_t)cmd) << 8, data, data_len);
}


static void fls_ams139_qspi_lcd_cmd_sequence(lcd_res_e res, lcd_pixel_mode_e pixel_mode)
{
    switch(res) {
        case LCD_RES_454:
        {
            if(pixel_mode == LCD_PIXEL_mode_16bit) {
                /* QSPI Mode */
                lcd_send_ca(0x11);                /* sleep out */
                delay_ms(10);
                //lcd_send_ca(0x13);              /* Normal Display mode on */
                lcd_send_ca1p(0x36, 0x00);        /* RGB Order */
                lcd_send_ca1p(0x3A, 0x75);        /* IFPF: 16bit color */
                //lcd_send_ca(0x47);              /* SPI read On */
                //lcd_send_ca(0x49);              /* AON Mode on */

                lcd_send_ca2p(0x4A, 0xFF, 0x03);    /* Brightness Value */
                lcd_send_ca1p(0x4F, 0x00);          /* Disable deep sleep */
                lcd_send_ca2p(0x51, 0xFF, 0x03);    /* Brightness Value */
                lcd_send_ca1p(0xc4, 0x00);          /* Set Quad-SPI Mode*/

                graphics_dc_am139_qspi_lcd_set_show_area(0, 453, 0, 453);
                lcd_send_ca2p(0x44, 0x01, 0xc5);    /* Set Tear Scan Line */
                lcd_send_ca1p(0x35, 0x00);          /* Tearing effect line on */
                lcd_send_ca1p(0x53, 0x28);          /* Brightness Control On & Display Dimming Off */

                delay_ms(20);
                lcd_send_ca(0x29);                  /* Display on */
                delay_ms(20);
            } else {  /* 24bit */
                /* QSPI Mode */
                lcd_send_ca(0x11);                /* sleep out */
                delay_ms(10);
                //lcd_send_ca(0x13);              /* Normal Display mode on */
                lcd_send_ca1p(0x36, 0x00);        /* RGB Order */
                lcd_send_ca1p(0x3A, 0x77);        /* IFPF: 16bit color */
                //lcd_send_ca(0x47);              /* SPI read On */
                //lcd_send_ca(0x49);              /* AON Mode on */

                lcd_send_ca2p(0x4A, 0xFF, 0x03);    /* Brightness Value */
                lcd_send_ca1p(0x4F, 0x00);          /* Disable deep sleep */
                lcd_send_ca2p(0x51, 0xFF, 0x03);    /* Brightness Value */
                lcd_send_ca1p(0xc4, 0x00);          /* Set Quad-SPI Mode*/

                graphics_dc_am139_qspi_lcd_set_show_area(0, 453, 0, 453);
                lcd_send_ca2p(0x44, 0x01, 0xc5);    /* Set Tear Scan Line */
                lcd_send_ca1p(0x35, 0x00);          /* Tearing effect line on */
                lcd_send_ca1p(0x53, 0x28);          /* Brightness Control On & Display Dimming Off */

                delay_ms(20);
                lcd_send_ca(0x29);                  /* Display on */
                delay_ms(20);
            }
        }
        break;

        case LCD_RES_360:
        {
            if(pixel_mode == LCD_PIXEL_mode_16bit) {
                /* QSPI Mode */
                lcd_send_ca(0x11);                  /* sleep out */
                delay_ms(10);
                //_lcd_send_ca(0x13);                /* Normal Display mode on */
                lcd_send_ca1p(0x36, 0x00);          /* RGB Order */
                lcd_send_ca1p(0x3A, 0x75);          /* IFPF: 16bit color */
                //_lcd_send_ca(0x47);                /* SPI read On */
                //_lcd_send_ca(0x49);                /* AON Mode on */

                lcd_send_ca2p(0x4A, 0xFF, 0x03);    /* Brightness Value */
                lcd_send_ca1p(0x4F, 0x00);          /* Disable deep sleep */
                lcd_send_ca2p(0x51, 0xFF, 0x03);    /* Brightness Value */
                lcd_send_ca1p(0xc4, 0x00);          /* Set Quad-SPI Mode*/

                graphics_dc_am139_qspi_lcd_set_show_area(0, 360 - 1, 0, 360 - 1);

                lcd_send_ca2p(0x44, 0x01, 0xc5);    /* Set Tear Scan Line */
                lcd_send_ca1p(0x35, 0x00);          /* Tearing effect line on */
                lcd_send_ca1p(0x53, 0x28);          /* Brightness Control On & Display Dimming Off */

                delay_ms(20);
                lcd_send_ca(0x29);                  /* Display on */
                delay_ms(20);
            } else {
                /* 24bit */
                /* QSPI Mode */
                lcd_send_ca(0x11);                  /* sleep out */
                delay_ms(10);
                //_lcd_send_ca(0x13);                /* Normal Display mode on */
                lcd_send_ca1p(0x36, 0x00);          /* RGB Order */
                lcd_send_ca1p(0x3A, 0x77);          /* IFPF: 16bit color */
                //_lcd_send_ca(0x47);                /* SPI read On */
                //_lcd_send_ca(0x49);                /* AON Mode on */

                lcd_send_ca2p(0x4A, 0xFF, 0x03);    /* Brightness Value */
                lcd_send_ca1p(0x4F, 0x00);          /* Disable deep sleep */
                lcd_send_ca2p(0x51, 0xFF, 0x03);    /* Brightness Value */
                lcd_send_ca1p(0xc4, 0x00);          /* Set Quad-SPI Mode*/

                graphics_dc_am139_qspi_lcd_set_show_area(0, 360 - 1, 0, 360 - 1);

                lcd_send_ca2p(0x44, 0x01, 0xc5);    /* Set Tear Scan Line */
                lcd_send_ca1p(0x35, 0x00);          /* Tearing effect line on */
                lcd_send_ca1p(0x53, 0x28);          /* Brightness Control On & Display Dimming Off */

                delay_ms(20);
                lcd_send_ca(0x29);                  /* Display on */
                delay_ms(20);
            }
        }
        break;

        default:
        {}
        break;
    }
}

void graphics_dc_am139_qspi_lcd_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
    uint8_t data[4];

    data[0] = (x1 & 0xff00) >> 8;
    data[1] = x1 & 0x00ff;
    data[2] = (x2 & 0xff00) >> 8;
    data[3] = x2 & 0x00ff;
    fls_ams139_qspi_lcd_write(0x2a, data, 4);

    data[0] = (y1 & 0xff00) >> 8;
    data[1] = y1 & 0x00ff;
    data[2] = (y2 & 0xff00) >> 8;
    data[3] = y2 & 0x00ff;
    fls_ams139_qspi_lcd_write(0x2b, data, 4);
}

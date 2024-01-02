#include "graphics_dc_lcd_drv.h"

#pragma diag_suppress 177

/*
 * Defines
 *****************************************************************************************
 */
#define RM69330_DC_QSPI_PINS_CFG()     \
    {   \
        .csn = {APP_IO_PULLUP, ENABLE},  \
        .clk = {APP_IO_PULLUP, ENABLE},  \
        .io0 = {APP_IO_PULLUP, ENABLE},  \
        .io1 = {APP_IO_PULLUP, ENABLE},  \
        .io2 = {APP_IO_PULLUP, ENABLE},  \
        .io3 = {APP_IO_PULLUP, ENABLE},  \
        .dcx = {APP_IO_PULLUP, DISABLE}, \
    }

#define RM69330_DC_QSPI_CFG()   \
    {   \
        .mspi_mode      = GDC_MODE_QSPI,                 \
        .clock_freq     = GDC_CLOCK_FREQ_48MHz,          \
        .clock_mode     = GDC_CLOCK_MODE_0,              \
        .tcsu_cycle     = GDC_TCSU_CYCLE_1,              \
        .layer_mode     = GDC_ONE_LAYER_MODE,            \
        .mipicfg_format = GDC_MIPICFG_QSPI_RGB565_OPT0,  \
        .resolution_x   = 454,                           \
        .resolution_y   = 454,                           \
        .pins_cfg       = RM69330_DC_QSPI_PINS_CFG(),    \
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
static void         rm69330_qspi_lcd_reset_set(uint8_t level);
static void         rm69330_qspi_lcd_write(uint8_t cmd, uint8_t *data, uint32_t data_len);
static void         rm69330_qspi_lcd_cmd_send(uint8_t cmd, uint8_t param);
static void         rm69330_qspi_lcd_cmd_sequence(lcd_res_e res, lcd_pixel_mode_e pixel_mode);
static void         rm69330_qspi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode);
static void         dc_irq_event_notify(uint32_t evt);

static uint32_t s_cur_resolution = LCD_RES_454;

/*
 * Public
 *****************************************************************************************
 */

void graphics_dc_rm69330_qspi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode, graphics_dc_mipi_format_e mipi_format) {
    app_graphics_dc_params_t dc_params = RM69330_DC_QSPI_CFG();
    dc_params.mipicfg_format = mipi_format;
    graphics_dc_init(&dc_params, dc_irq_event_notify);
    rm69330_qspi_lcd_init(res, pixel_mode);

    s_cur_resolution = res;
}

void graphics_dc_rm69330_qspi_send_frame(app_graphics_dc_framelayer_t layer0, uint32_t lcd_w, uint32_t lcd_h) {

    app_graphics_dc_cmd_t   dc_cmd;

    dc_cmd.command       = 0x12;
    dc_cmd.address       = 0x002C00;
    dc_cmd.address_width = GDC_FRAME_ADDRESS_WIDTH_24BIT;
    dc_cmd.frame_timing  = GDC_QSPI_FRAME_TIMING_1;

    uint16_t x1 = (((lcd_w -  layer0.resolution_x)/2)/2)*2;
    uint16_t x2 = x1 + layer0.resolution_x - 1;
    uint16_t y1 = (((lcd_h - layer0.resolution_y)/2)/2)*2;
    uint16_t y2 =  y1 + layer0.resolution_y - 1;

    graphics_dc_rm69330_qspi_lcd_set_show_area(x1, x2, y1, y2);
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
static void rm69330_qspi_lcd_reset_set(uint8_t level)
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

static void rm69330_qspi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode)
{
    rm69330_qspi_lcd_reset_set(0);
    delay_ms(100);
    rm69330_qspi_lcd_reset_set(1);
    delay_ms(100);
    rm69330_qspi_lcd_cmd_sequence(res, pixel_mode);
}

static void rm69330_qspi_lcd_write(uint8_t cmd, uint8_t *data, uint32_t data_len)
{
    app_graphics_dc_spi_send(0x02, ((uint32_t)cmd) << 8, data, data_len);
}

static void rm69330_qspi_lcd_cmd_send(uint8_t cmd, uint8_t param)
{
    rm69330_qspi_lcd_write(cmd, &param, 1);
}

static void rm69330_qspi_lcd_cmd_send_2B(uint8_t cmd, uint8_t param, uint8_t param2)
{
    uint8_t p[2]={param,param2};
    rm69330_qspi_lcd_write(cmd, p, 2);
}

static void rm69330_qspi_lcd_write_16(uint16_t cmd, uint8_t *data, uint32_t data_len)
{
    app_graphics_dc_spi_send(0x02, (uint32_t)cmd, data, data_len);
}


void rm69330_qspi_lcd_cmd_send_16(uint16_t cmd, uint8_t param)
{
    rm69330_qspi_lcd_write_16(cmd, &param, 1);
}

static void rm69330_qspi_lcd_cmd_sequence(lcd_res_e res, lcd_pixel_mode_e pixel_mode)
{

#define SPI_CMD(cmd) rm69330_qspi_lcd_write(cmd, NULL, 0)
#define SPI_CMD_P(cmd, p) rm69330_qspi_lcd_cmd_send(cmd, p)
#define SPI_CMD_PP(cmd, p1, p2) rm69330_qspi_lcd_cmd_send_2B(cmd, p1, p2)
#define SPI_CMD16(cmd16) rm69330_qspi_lcd_write_16(cmd16, NULL, 0)
#define SPI_CMD16_P(cmd16, p) rm69330_qspi_lcd_cmd_send_16(cmd16, p)

#define TE_SCANLINE 300

    SPI_CMD(0x11); // SLPOUT: Sleep Out
    delay_ms(5);
    SPI_CMD_P(0x36, 0x00); // MADCTR: Scan Direction Control
    if(pixel_mode == LCD_PIXEL_mode_16bit) {
        SPI_CMD_P(0x3A, 0x75); // COLMOD: Interface Pixel Format - RGB565
    } else if(pixel_mode == LCD_PIXEL_mode_24bit) {
        SPI_CMD_P(0x3A, 0x77); // COLMOD: Interface Pixel Format - RGB888
    }
    SPI_CMD_P(0x51, 0xFF); // WRDISBV: Write Display Brightness
    SPI_CMD_P(0xC4, 0x80); // SetDSPIMode: Set DSPI Mode
    SPI_CMD_PP(0x44, (TE_SCANLINE) >> 8, (TE_SCANLINE)&0xFF);  // STESL: Set Tear Scanline
    SPI_CMD_P(0x35, 0x00); // TEON: Tearing Effect Line On
    SPI_CMD16_P(0x2840, 0x30); // Unknown: Adjust Refresh Rate
    delay_ms(20);
    SPI_CMD(0x29);
    delay_ms(20);
}

void graphics_dc_rm69330_qspi_lcd_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
    uint8_t data[4];

    if(s_cur_resolution == LCD_RES_454) {
        x1 += 12;
        x2 += 12;
    } else if(s_cur_resolution == LCD_RES_390) {
        x1 += 6;
        x2 += 6;
    }

    data[0] = (x1 & 0xff00) >> 8;
    data[1] = x1 & 0x00ff;
    data[2] = (x2 & 0xff00) >> 8;
    data[3] = x2 & 0x00ff;
    rm69330_qspi_lcd_write(0x2a, data, 4);

    data[0] = (y1 & 0xff00) >> 8;
    data[1] = y1 & 0x00ff;
    data[2] = (y2 & 0xff00) >> 8;
    data[3] = y2 & 0x00ff;
    rm69330_qspi_lcd_write(0x2b, data, 4);
}

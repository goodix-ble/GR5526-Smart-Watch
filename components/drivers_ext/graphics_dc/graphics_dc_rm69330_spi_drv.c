#include "graphics_dc_lcd_drv.h"
#include "FreeRTOS.h"


/*
 * Defines
 *****************************************************************************************
 */
#define RM69330_DC_SPI_PINS_CFG()     \
    {   \
        .csn = {APP_IO_PULLUP, ENABLE},  \
        .clk = {APP_IO_PULLUP, ENABLE},  \
        .io0 = {APP_IO_PULLUP, ENABLE},  \
        .io1 = {APP_IO_PULLUP, ENABLE},  \
        .io2 = {APP_IO_PULLUP, ENABLE},  \
        .io3 = {APP_IO_PULLUP, ENABLE},  \
        .dcx = {APP_IO_PULLUP, DISABLE}, \
    }

#define RM69330_DC_SPI_CFG()   \
    {   \
        .mspi_mode      = GDC_MODE_SPI,                  \
        .clock_freq     = GDC_CLOCK_FREQ_48MHz,          \
        .clock_mode     = GDC_CLOCK_MODE_0,              \
        .tcsu_cycle     = GDC_TCSU_CYCLE_1,              \
        .layer_mode     = GDC_ONE_LAYER_MODE,            \
        .mipicfg_format = GDC_MIPICFG_SPI_RGB565_OPT0,   \
        .resolution_x   = 454,                           \
        .resolution_y   = 454,                           \
        .pins_cfg       = RM69330_DC_SPI_PINS_CFG(),     \
    }

#define  LCD_RESET_GPIO     APP_IO_TYPE_AON

#if IS_SK_BOARD > 0u
    #define  LCD_RESET_PIN      APP_IO_PIN_6              /* Rest PIN in SK  Board */
    #define  LCD_RESET_GPIO     APP_IO_TYPE_AON
#else
    #define  LCD_RESET_PIN      APP_IO_PIN_5              /* Rest PIN in EVB Board */
    #define  LCD_RESET_GPIO     APP_IO_TYPE_AON
#endif


#if LCD_RES_CURRENT == LCD_RES_454
    #define LCD_RES_W   454u
    #define LCD_RES_H   454u
#elif LCD_RES_CURRENT == LCD_RES_390
    #define LCD_RES_W   390u
    #define LCD_RES_H   390u
#else
    #error "NOT Define!"
#endif


/*
 * Declaration
 *****************************************************************************************
 */
static void         rm69330_spi_lcd_reset_set(uint8_t level);
static void         rm69330_spi_lcd_write(uint8_t cmd, uint8_t *data, uint32_t data_len);
static void         rm69330_spi_lcd_cmd_send(uint8_t cmd, uint8_t param);
static void         rm69330_spi_lcd_cmd_sequence(lcd_res_e res, lcd_pixel_mode_e pixel_mode);
static void         rm69330_spi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode);
static void         dc_irq_event_notify(uint32_t evt);

static uint32_t s_cur_resolution = LCD_RES_454;
/*
 * Public
 *****************************************************************************************
 */

void graphics_dc_rm69330_spi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode, graphics_dc_mipi_format_e mipi_format) {
    app_graphics_dc_params_t dc_params = RM69330_DC_SPI_CFG();

    dc_params.mipicfg_format = mipi_format;

    graphics_dc_init(&dc_params, dc_irq_event_notify);
    rm69330_spi_lcd_init(res, pixel_mode);

	s_cur_resolution = res;
}

void graphics_dc_rm69330_spi_send_frame(app_graphics_dc_framelayer_t layer0, uint32_t lcd_w, uint32_t lcd_h) {

    app_graphics_dc_cmd_t   dc_cmd;

    dc_cmd.command       = 0x02;
    dc_cmd.address       = 0x002C00;
    dc_cmd.address_width = GDC_FRAME_ADDRESS_WIDTH_24BIT;
    dc_cmd.frame_timing  = GDC_SPI_FRAME_TIMING_0;

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

static void dc_irq_event_notify(uint32_t evt) {
    //printf("DC Event: %d   \r\n", evt);
}


/* B0.EVB USE AON_GPIO_5 to control RESET */
static void rm69330_spi_lcd_reset_set(uint8_t level)
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

static void rm69330_spi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode)
{
    rm69330_spi_lcd_reset_set(0);
    delay_ms(100);
    rm69330_spi_lcd_reset_set(1);
    delay_ms(100);
    rm69330_spi_lcd_cmd_sequence(res, pixel_mode);
}

static void rm69330_spi_lcd_write(uint8_t cmd, uint8_t *data, uint32_t data_len)
{
    app_graphics_dc_spi_send(0x02, ((uint32_t)cmd) << 8, data, data_len);
}

static void rm69330_spi_lcd_cmd_send(uint8_t cmd, uint8_t param)
{
    rm69330_spi_lcd_write(cmd, &param, 1);
}

static void rm69330_spi_lcd_cmd_sequence(lcd_res_e res, lcd_pixel_mode_e pixel_mode)
{
    switch(res) {
        case LCD_RES_390:
        {
            rm69330_spi_lcd_cmd_send(0xfe, 0x01);
            rm69330_spi_lcd_cmd_send(0x05, 0x10);
            rm69330_spi_lcd_cmd_send(0x06, 0x62);
            rm69330_spi_lcd_cmd_send(0x0d, 0x00);
            rm69330_spi_lcd_cmd_send(0x0e, 0x81);
            rm69330_spi_lcd_cmd_send(0x0f, 0x81);
            rm69330_spi_lcd_cmd_send(0x10, 0x11);
            rm69330_spi_lcd_cmd_send(0x11, 0x81);
            rm69330_spi_lcd_cmd_send(0x12, 0x81);
            rm69330_spi_lcd_cmd_send(0x13, 0x80);
            rm69330_spi_lcd_cmd_send(0x14, 0x80);
            rm69330_spi_lcd_cmd_send(0x15, 0x81);
            rm69330_spi_lcd_cmd_send(0x16, 0x81);
            rm69330_spi_lcd_cmd_send(0x18, 0x66);
            rm69330_spi_lcd_cmd_send(0x19, 0x88);
            rm69330_spi_lcd_cmd_send(0x5b, 0x10);
            rm69330_spi_lcd_cmd_send(0x5c, 0x55);
            rm69330_spi_lcd_cmd_send(0x62, 0x19);
            rm69330_spi_lcd_cmd_send(0x63, 0x19);
            rm69330_spi_lcd_cmd_send(0x70, 0x55);
            rm69330_spi_lcd_cmd_send(0x74, 0x0c);
            rm69330_spi_lcd_cmd_send(0xc5, 0x10);
            rm69330_spi_lcd_cmd_send(0x25, 0x03);
            rm69330_spi_lcd_cmd_send(0x26, 0x80);
            rm69330_spi_lcd_cmd_send(0x27, 0x08);
            rm69330_spi_lcd_cmd_send(0x28, 0x08);
            rm69330_spi_lcd_cmd_send(0x2a, 0x23);
            rm69330_spi_lcd_cmd_send(0x2b, 0x80);
            rm69330_spi_lcd_cmd_send(0x2d, 0x08);
            rm69330_spi_lcd_cmd_send(0x2f, 0x08);
            rm69330_spi_lcd_cmd_send(0x30, 0x43);   //0x43: 15Hz
            rm69330_spi_lcd_cmd_send(0x66, 0x90);
            rm69330_spi_lcd_cmd_send(0x72, 0x1a);
            rm69330_spi_lcd_cmd_send(0x73, 0x13);
            rm69330_spi_lcd_cmd_send(0xfe, 0x01);
            rm69330_spi_lcd_cmd_send(0x6a, 0x17);   //0x17: -2.2V for rt4723
            rm69330_spi_lcd_cmd_send(0x1b, 0x00);
            rm69330_spi_lcd_cmd_send(0x1d, 0x03);
            rm69330_spi_lcd_cmd_send(0x1e, 0x03);
            rm69330_spi_lcd_cmd_send(0x1f, 0x0c);
            rm69330_spi_lcd_cmd_send(0x20, 0x03);
            rm69330_spi_lcd_cmd_send(0xfe, 0x01);
            rm69330_spi_lcd_cmd_send(0x36, 0x00);
            rm69330_spi_lcd_cmd_send(0x6c, 0x80);
            rm69330_spi_lcd_cmd_send(0x6d, 0x19);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x63, 0x00);
            rm69330_spi_lcd_cmd_send(0x64, 0x0e);
            rm69330_spi_lcd_cmd_send(0xfe, 0x02);
            rm69330_spi_lcd_cmd_send(0xa9, 0x40);
            rm69330_spi_lcd_cmd_send(0xaa, 0xb8);
            rm69330_spi_lcd_cmd_send(0xab, 0x01);
            rm69330_spi_lcd_cmd_send(0xfe, 0x03);
            rm69330_spi_lcd_cmd_send(0xa9, 0x40);
            rm69330_spi_lcd_cmd_send(0xaa, 0x90);
            rm69330_spi_lcd_cmd_send(0xab, 0x01);
            rm69330_spi_lcd_cmd_send(0xfe, 0x01);
            rm69330_spi_lcd_cmd_send(0x3a, 0x00);
            rm69330_spi_lcd_cmd_send(0x3b, 0x41);
            rm69330_spi_lcd_cmd_send(0x3d, 0x17);
            rm69330_spi_lcd_cmd_send(0x3f, 0x42);
            rm69330_spi_lcd_cmd_send(0x40, 0x17);
            rm69330_spi_lcd_cmd_send(0x41, 0x06);
            rm69330_spi_lcd_cmd_send(0x37, 0x0c);
            rm69330_spi_lcd_cmd_send(0xfe, 0x0c);
            rm69330_spi_lcd_cmd_send(0x07, 0x1f);
            rm69330_spi_lcd_cmd_send(0x08, 0x2f);
            rm69330_spi_lcd_cmd_send(0x09, 0x3f);
            rm69330_spi_lcd_cmd_send(0x0a, 0x4f);
            rm69330_spi_lcd_cmd_send(0x0b, 0x5f);
            rm69330_spi_lcd_cmd_send(0x0c, 0x6f);
            rm69330_spi_lcd_cmd_send(0x0d, 0xff);
            rm69330_spi_lcd_cmd_send(0x0e, 0xff);
            rm69330_spi_lcd_cmd_send(0x0f, 0xff);
            rm69330_spi_lcd_cmd_send(0x10, 0xff);
            rm69330_spi_lcd_cmd_send(0xfe, 0x01);
            rm69330_spi_lcd_cmd_send(0x42, 0x14);
            rm69330_spi_lcd_cmd_send(0x43, 0x41);
            rm69330_spi_lcd_cmd_send(0x44, 0x25);
            rm69330_spi_lcd_cmd_send(0x45, 0x52);
            rm69330_spi_lcd_cmd_send(0x46, 0x36);
            rm69330_spi_lcd_cmd_send(0x47, 0x63);
            rm69330_spi_lcd_cmd_send(0x48, 0x41);
            rm69330_spi_lcd_cmd_send(0x49, 0x14);
            rm69330_spi_lcd_cmd_send(0x4a, 0x52);
            rm69330_spi_lcd_cmd_send(0x4b, 0x25);
            rm69330_spi_lcd_cmd_send(0x4c, 0x63);
            rm69330_spi_lcd_cmd_send(0x4d, 0x36);
            rm69330_spi_lcd_cmd_send(0x4e, 0x36);
            rm69330_spi_lcd_cmd_send(0x4f, 0x63);
            rm69330_spi_lcd_cmd_send(0x50, 0x25);
            rm69330_spi_lcd_cmd_send(0x51, 0x52);
            rm69330_spi_lcd_cmd_send(0x52, 0x14);
            rm69330_spi_lcd_cmd_send(0x53, 0x41);
            rm69330_spi_lcd_cmd_send(0x54, 0x63);
            rm69330_spi_lcd_cmd_send(0x55, 0x36);
            rm69330_spi_lcd_cmd_send(0x56, 0x52);
            rm69330_spi_lcd_cmd_send(0x57, 0x25);
            rm69330_spi_lcd_cmd_send(0x58, 0x41);
            rm69330_spi_lcd_cmd_send(0x59, 0x14);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x5d, 0x01);
            rm69330_spi_lcd_cmd_send(0x75, 0x08);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x5e, 0x9f);
            rm69330_spi_lcd_cmd_send(0x5f, 0x43);
            rm69330_spi_lcd_cmd_send(0x60, 0xff);
            rm69330_spi_lcd_cmd_send(0x61, 0xff);
            rm69330_spi_lcd_cmd_send(0x62, 0xff);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x76, 0xff);
            rm69330_spi_lcd_cmd_send(0x77, 0xff);
            rm69330_spi_lcd_cmd_send(0x78, 0x10);
            rm69330_spi_lcd_cmd_send(0x79, 0xf2);
            rm69330_spi_lcd_cmd_send(0x7a, 0xff);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x00, 0x8d);
            rm69330_spi_lcd_cmd_send(0x01, 0x00);
            rm69330_spi_lcd_cmd_send(0x02, 0x00);
            rm69330_spi_lcd_cmd_send(0x03, 0x05);
            rm69330_spi_lcd_cmd_send(0x04, 0x00);
            rm69330_spi_lcd_cmd_send(0x05, 0x05);
            rm69330_spi_lcd_cmd_send(0x06, 0x00);
            rm69330_spi_lcd_cmd_send(0x07, 0x00);
            rm69330_spi_lcd_cmd_send(0x08, 0x00);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x09, 0xcc);
            rm69330_spi_lcd_cmd_send(0x0a, 0x00);
            rm69330_spi_lcd_cmd_send(0x0b, 0x02);
            rm69330_spi_lcd_cmd_send(0x0c, 0x00);
            rm69330_spi_lcd_cmd_send(0x0d, 0x60);
            rm69330_spi_lcd_cmd_send(0x0e, 0x06);
            rm69330_spi_lcd_cmd_send(0x0f, 0x2c);
            rm69330_spi_lcd_cmd_send(0x10, 0x53);
            rm69330_spi_lcd_cmd_send(0x11, 0x00);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x12, 0xcc);
            rm69330_spi_lcd_cmd_send(0x13, 0x00);
            rm69330_spi_lcd_cmd_send(0x14, 0x02);
            rm69330_spi_lcd_cmd_send(0x15, 0x00);
            rm69330_spi_lcd_cmd_send(0x16, 0x60);
            rm69330_spi_lcd_cmd_send(0x17, 0x05);
            rm69330_spi_lcd_cmd_send(0x18, 0x2c);
            rm69330_spi_lcd_cmd_send(0x19, 0x53);
            rm69330_spi_lcd_cmd_send(0x1a, 0x00);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x1b, 0xdc);
            rm69330_spi_lcd_cmd_send(0x1c, 0x00);
            rm69330_spi_lcd_cmd_send(0x1d, 0x04);
            rm69330_spi_lcd_cmd_send(0x1e, 0x02);
            rm69330_spi_lcd_cmd_send(0x1f, 0x18);
            rm69330_spi_lcd_cmd_send(0x20, 0x06);
            rm69330_spi_lcd_cmd_send(0x21, 0x3d);
            rm69330_spi_lcd_cmd_send(0x22, 0x75);
            rm69330_spi_lcd_cmd_send(0x23, 0x00);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x24, 0xdc);
            rm69330_spi_lcd_cmd_send(0x25, 0x00);
            rm69330_spi_lcd_cmd_send(0x26, 0x04);
            rm69330_spi_lcd_cmd_send(0x27, 0x02);
            rm69330_spi_lcd_cmd_send(0x28, 0x18);
            rm69330_spi_lcd_cmd_send(0x29, 0x04);
            rm69330_spi_lcd_cmd_send(0x2a, 0x3d);
            rm69330_spi_lcd_cmd_send(0x2b, 0x75);
            rm69330_spi_lcd_cmd_send(0x2d, 0x00);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x53, 0x8a);
            rm69330_spi_lcd_cmd_send(0x54, 0x78);
            rm69330_spi_lcd_cmd_send(0x55, 0x08);
            rm69330_spi_lcd_cmd_send(0x56, 0x0a);
            rm69330_spi_lcd_cmd_send(0x58, 0x2a);
            rm69330_spi_lcd_cmd_send(0x59, 0x00);
            rm69330_spi_lcd_cmd_send(0x65, 0x02);
            rm69330_spi_lcd_cmd_send(0x66, 0x0a);
            rm69330_spi_lcd_cmd_send(0x67, 0x00);
            rm69330_spi_lcd_cmd_send(0xfe, 0x07);
            rm69330_spi_lcd_cmd_send(0x15, 0x04);

            rm69330_spi_lcd_cmd_send(0xfe, 0x00);  //Write CMD mode page
            rm69330_spi_lcd_cmd_send(0xc4, 0x80);  //Set_DSPI Mode
            rm69330_spi_lcd_cmd_send(0x35, 0x00);  //Tearing effect line on
            rm69330_spi_lcd_cmd_send(0x51, 0xff);  //display brightness

            if(pixel_mode == LCD_PIXEL_mode_16bit) {
                rm69330_spi_lcd_cmd_send(0x3a, 0x75);  //16bit rgb
            } else if(pixel_mode == LCD_PIXEL_mode_24bit) {
                rm69330_spi_lcd_cmd_send(0x3a, 0x77);  //24bit rgb
            } else {

            }

            rm69330_spi_lcd_cmd_send(0x11, 0x00);  //wake up
            delay_ms(20);
            //rm69330_spi_lcd_cmd_send(0x23, 0x00);  //all pixel on
            rm69330_spi_lcd_cmd_send(0x29, 0x00);  //display on
            delay_ms(20);
            //rm69330_spi_lcd_cmd_send(0x12);  //Partial display mode on
        }
        break;

        case LCD_RES_454:
        {
            rm69330_spi_lcd_cmd_send(0xfe, 0x01);
            rm69330_spi_lcd_cmd_send(0x05, 0x00);
            rm69330_spi_lcd_cmd_send(0x06, 0x72);
            rm69330_spi_lcd_cmd_send(0x0d, 0x00);
            rm69330_spi_lcd_cmd_send(0x0e, 0x81);
            rm69330_spi_lcd_cmd_send(0x0f, 0x81);
            rm69330_spi_lcd_cmd_send(0x10, 0x11);
            rm69330_spi_lcd_cmd_send(0x11, 0x81);
            rm69330_spi_lcd_cmd_send(0x12, 0x81);
            rm69330_spi_lcd_cmd_send(0x13, 0x80);
            rm69330_spi_lcd_cmd_send(0x14, 0x80);
            rm69330_spi_lcd_cmd_send(0x15, 0x81);
            rm69330_spi_lcd_cmd_send(0x16, 0x81);
            rm69330_spi_lcd_cmd_send(0x18, 0x66);
            rm69330_spi_lcd_cmd_send(0x19, 0x88);
            rm69330_spi_lcd_cmd_send(0x5b, 0x10);
            rm69330_spi_lcd_cmd_send(0x5c, 0x55);
            rm69330_spi_lcd_cmd_send(0x62, 0x19);
            rm69330_spi_lcd_cmd_send(0x63, 0x19);
            rm69330_spi_lcd_cmd_send(0x70, 0x54);
            rm69330_spi_lcd_cmd_send(0x74, 0x0c);
            rm69330_spi_lcd_cmd_send(0xc5, 0x10);

            rm69330_spi_lcd_cmd_send(0xfe, 0x01);
            rm69330_spi_lcd_cmd_send(0x25, 0x03);
            rm69330_spi_lcd_cmd_send(0x26, 0x32);
            rm69330_spi_lcd_cmd_send(0x27, 0x0a);
            rm69330_spi_lcd_cmd_send(0x28, 0x08);
            rm69330_spi_lcd_cmd_send(0x2a, 0x03);
            rm69330_spi_lcd_cmd_send(0x2b, 0x32);
            rm69330_spi_lcd_cmd_send(0x2d, 0x0a);
            rm69330_spi_lcd_cmd_send(0x2f, 0x08);
            rm69330_spi_lcd_cmd_send(0x30, 0x43);   //0x43: 15Hz

            rm69330_spi_lcd_cmd_send(0x66, 0x90);
            rm69330_spi_lcd_cmd_send(0x72, 0x1a);
            rm69330_spi_lcd_cmd_send(0x73, 0x13);

            rm69330_spi_lcd_cmd_send(0xfe, 0x01);
            rm69330_spi_lcd_cmd_send(0x6a, 0x17);   //0x17: -2.2V for rt4723
            rm69330_spi_lcd_cmd_send(0x1b, 0x00);
            rm69330_spi_lcd_cmd_send(0x1d, 0x03);
            rm69330_spi_lcd_cmd_send(0x1e, 0x03);
            rm69330_spi_lcd_cmd_send(0x1f, 0x03);
            rm69330_spi_lcd_cmd_send(0x20, 0x03);
            rm69330_spi_lcd_cmd_send(0xfe, 0x01);
            rm69330_spi_lcd_cmd_send(0x36, 0x00);
            rm69330_spi_lcd_cmd_send(0x6c, 0x80);
            rm69330_spi_lcd_cmd_send(0x6d, 0x19);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x63, 0x00);
            rm69330_spi_lcd_cmd_send(0x64, 0x0e);
            rm69330_spi_lcd_cmd_send(0xfe, 0x02);
            rm69330_spi_lcd_cmd_send(0xa9, 0x30);
            rm69330_spi_lcd_cmd_send(0xaa, 0xb9);
            rm69330_spi_lcd_cmd_send(0xab, 0x01);
            rm69330_spi_lcd_cmd_send(0xfe, 0x03);
            rm69330_spi_lcd_cmd_send(0xa9, 0x30);
            rm69330_spi_lcd_cmd_send(0xaa, 0x90);
            rm69330_spi_lcd_cmd_send(0xab, 0x01);

            rm69330_spi_lcd_cmd_send(0xfe, 0x0c);
            rm69330_spi_lcd_cmd_send(0x07, 0x1f);
            rm69330_spi_lcd_cmd_send(0x08, 0x2f);
            rm69330_spi_lcd_cmd_send(0x09, 0x3f);
            rm69330_spi_lcd_cmd_send(0x0a, 0x4f);
            rm69330_spi_lcd_cmd_send(0x0b, 0x5f);
            rm69330_spi_lcd_cmd_send(0x0c, 0x6f);
            rm69330_spi_lcd_cmd_send(0x0d, 0xff);
            rm69330_spi_lcd_cmd_send(0x0e, 0xff);
            rm69330_spi_lcd_cmd_send(0x0f, 0xff);
            rm69330_spi_lcd_cmd_send(0x10, 0xff);
            rm69330_spi_lcd_cmd_send(0xfe, 0x01);
            rm69330_spi_lcd_cmd_send(0x42, 0x14);
            rm69330_spi_lcd_cmd_send(0x43, 0x41);
            rm69330_spi_lcd_cmd_send(0x44, 0x25);
            rm69330_spi_lcd_cmd_send(0x45, 0x52);
            rm69330_spi_lcd_cmd_send(0x46, 0x36);
            rm69330_spi_lcd_cmd_send(0x47, 0x63);
            rm69330_spi_lcd_cmd_send(0x48, 0x41);
            rm69330_spi_lcd_cmd_send(0x49, 0x14);
            rm69330_spi_lcd_cmd_send(0x4a, 0x52);
            rm69330_spi_lcd_cmd_send(0x4b, 0x25);
            rm69330_spi_lcd_cmd_send(0x4c, 0x63);
            rm69330_spi_lcd_cmd_send(0x4d, 0x36);
            rm69330_spi_lcd_cmd_send(0x4e, 0x16);
            rm69330_spi_lcd_cmd_send(0x4f, 0x61);
            rm69330_spi_lcd_cmd_send(0x50, 0x25);
            rm69330_spi_lcd_cmd_send(0x51, 0x52);
            rm69330_spi_lcd_cmd_send(0x52, 0x34);
            rm69330_spi_lcd_cmd_send(0x53, 0x43);
            rm69330_spi_lcd_cmd_send(0x54, 0x61);
            rm69330_spi_lcd_cmd_send(0x55, 0x16);
            rm69330_spi_lcd_cmd_send(0x56, 0x52);
            rm69330_spi_lcd_cmd_send(0x57, 0x25);
            rm69330_spi_lcd_cmd_send(0x58, 0x43);
            rm69330_spi_lcd_cmd_send(0x59, 0x34);

            rm69330_spi_lcd_cmd_send(0xfe, 0x01);
            rm69330_spi_lcd_cmd_send(0x3a, 0x00);
            rm69330_spi_lcd_cmd_send(0x3b, 0x00);
            rm69330_spi_lcd_cmd_send(0x3d, 0x12);
            rm69330_spi_lcd_cmd_send(0x3f, 0x37);
            rm69330_spi_lcd_cmd_send(0x40, 0x12);
            rm69330_spi_lcd_cmd_send(0x41, 0x0f);
            rm69330_spi_lcd_cmd_send(0x37, 0x0c);

            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            //rm69330_spi_lcd_cmd_send(0x5d, 0x01);
            //rm69330_spi_lcd_cmd_send(0x75, 0x08);
            //rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x5e, 0x0f);
            rm69330_spi_lcd_cmd_send(0x5f, 0x12);
            rm69330_spi_lcd_cmd_send(0x60, 0xff);
            rm69330_spi_lcd_cmd_send(0x61, 0xff);
            rm69330_spi_lcd_cmd_send(0x62, 0xff);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x76, 0xff);
            rm69330_spi_lcd_cmd_send(0x77, 0xff);
            rm69330_spi_lcd_cmd_send(0x78, 0x49);
            rm69330_spi_lcd_cmd_send(0x79, 0xf3);
            rm69330_spi_lcd_cmd_send(0x7a, 0xff);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x00, 0x9d);
            rm69330_spi_lcd_cmd_send(0x01, 0x00);
            rm69330_spi_lcd_cmd_send(0x02, 0x00);
            rm69330_spi_lcd_cmd_send(0x03, 0x00);
            rm69330_spi_lcd_cmd_send(0x04, 0x00);
            rm69330_spi_lcd_cmd_send(0x05, 0x01);
            rm69330_spi_lcd_cmd_send(0x06, 0x01);
            rm69330_spi_lcd_cmd_send(0x07, 0x01);
            rm69330_spi_lcd_cmd_send(0x08, 0x00);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x09, 0xdc);
            rm69330_spi_lcd_cmd_send(0x0a, 0x00);
            rm69330_spi_lcd_cmd_send(0x0b, 0x02);
            rm69330_spi_lcd_cmd_send(0x0c, 0x00);
            rm69330_spi_lcd_cmd_send(0x0d, 0x08);
            rm69330_spi_lcd_cmd_send(0x0e, 0x01);
            rm69330_spi_lcd_cmd_send(0x0f, 0xce);
            rm69330_spi_lcd_cmd_send(0x10, 0x16);
            rm69330_spi_lcd_cmd_send(0x11, 0x00);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x12, 0xdc);
            rm69330_spi_lcd_cmd_send(0x13, 0x00);
            rm69330_spi_lcd_cmd_send(0x14, 0x02);
            rm69330_spi_lcd_cmd_send(0x15, 0x00);
            rm69330_spi_lcd_cmd_send(0x16, 0x08);
            rm69330_spi_lcd_cmd_send(0x17, 0x02);
            rm69330_spi_lcd_cmd_send(0x18, 0xce);
            rm69330_spi_lcd_cmd_send(0x19, 0x16);
            rm69330_spi_lcd_cmd_send(0x1a, 0x00);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x1b, 0xdc);
            rm69330_spi_lcd_cmd_send(0x1c, 0x00);
            rm69330_spi_lcd_cmd_send(0x1d, 0x02);
            rm69330_spi_lcd_cmd_send(0x1e, 0x00);
            rm69330_spi_lcd_cmd_send(0x1f, 0x08);
            rm69330_spi_lcd_cmd_send(0x20, 0x01);
            rm69330_spi_lcd_cmd_send(0x21, 0xce);
            rm69330_spi_lcd_cmd_send(0x22, 0x16);
            rm69330_spi_lcd_cmd_send(0x23, 0x00);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x24, 0xdc);
            rm69330_spi_lcd_cmd_send(0x25, 0x00);
            rm69330_spi_lcd_cmd_send(0x26, 0x02);
            rm69330_spi_lcd_cmd_send(0x27, 0x00);
            rm69330_spi_lcd_cmd_send(0x28, 0x08);
            rm69330_spi_lcd_cmd_send(0x29, 0x02);
            rm69330_spi_lcd_cmd_send(0x2a, 0xce);
            rm69330_spi_lcd_cmd_send(0x2b, 0x16);
            rm69330_spi_lcd_cmd_send(0x2d, 0x00);
            rm69330_spi_lcd_cmd_send(0xfe, 0x04);
            rm69330_spi_lcd_cmd_send(0x53, 0x8a);
            rm69330_spi_lcd_cmd_send(0x54, 0x00);
            rm69330_spi_lcd_cmd_send(0x55, 0x03);
            rm69330_spi_lcd_cmd_send(0x56, 0x01);
            rm69330_spi_lcd_cmd_send(0x58, 0x01);
            rm69330_spi_lcd_cmd_send(0x59, 0x00);
            rm69330_spi_lcd_cmd_send(0x65, 0x76);
            rm69330_spi_lcd_cmd_send(0x66, 0x19);
            rm69330_spi_lcd_cmd_send(0x67, 0x00);
            rm69330_spi_lcd_cmd_send(0xfe, 0x07);
            rm69330_spi_lcd_cmd_send(0x15, 0x04);

            rm69330_spi_lcd_cmd_send(0xfe, 0x05);
            rm69330_spi_lcd_cmd_send(0x4c, 0x01);
            rm69330_spi_lcd_cmd_send(0x4d, 0x82);
            rm69330_spi_lcd_cmd_send(0x4e, 0x04);
            rm69330_spi_lcd_cmd_send(0x4f, 0x00);
            rm69330_spi_lcd_cmd_send(0x50, 0x20);
            rm69330_spi_lcd_cmd_send(0x51, 0x10);
            rm69330_spi_lcd_cmd_send(0x52, 0x04);
            rm69330_spi_lcd_cmd_send(0x53, 0x41);
            rm69330_spi_lcd_cmd_send(0x54, 0x0a);
            rm69330_spi_lcd_cmd_send(0x55, 0x08);
            rm69330_spi_lcd_cmd_send(0x56, 0x00);
            rm69330_spi_lcd_cmd_send(0x57, 0x28);
            rm69330_spi_lcd_cmd_send(0x58, 0x00);
            rm69330_spi_lcd_cmd_send(0x59, 0x80);
            rm69330_spi_lcd_cmd_send(0x5a, 0x04);
            rm69330_spi_lcd_cmd_send(0x5b, 0x10);
            rm69330_spi_lcd_cmd_send(0x5c, 0x20);
            rm69330_spi_lcd_cmd_send(0x5d, 0x00);
            rm69330_spi_lcd_cmd_send(0x5e, 0x04);
            rm69330_spi_lcd_cmd_send(0x5f, 0x0a);
            rm69330_spi_lcd_cmd_send(0x60, 0x01);
            rm69330_spi_lcd_cmd_send(0x61, 0x08);
            rm69330_spi_lcd_cmd_send(0x62, 0x00);
            rm69330_spi_lcd_cmd_send(0x63, 0x20);
            rm69330_spi_lcd_cmd_send(0x64, 0x40);
            rm69330_spi_lcd_cmd_send(0x65, 0x04);
            rm69330_spi_lcd_cmd_send(0x66, 0x02);
            rm69330_spi_lcd_cmd_send(0x67, 0x48);
            rm69330_spi_lcd_cmd_send(0x68, 0x4c);
            rm69330_spi_lcd_cmd_send(0x69, 0x02);
            rm69330_spi_lcd_cmd_send(0x6a, 0x12);
            rm69330_spi_lcd_cmd_send(0x6b, 0x00);
            rm69330_spi_lcd_cmd_send(0x6c, 0x48);
            rm69330_spi_lcd_cmd_send(0x6d, 0xa0);
            rm69330_spi_lcd_cmd_send(0x6e, 0x08);
            rm69330_spi_lcd_cmd_send(0x6f, 0x04);
            rm69330_spi_lcd_cmd_send(0x70, 0x05);
            rm69330_spi_lcd_cmd_send(0x71, 0x92);
            rm69330_spi_lcd_cmd_send(0x72, 0x00);
            rm69330_spi_lcd_cmd_send(0x73, 0x18);
            rm69330_spi_lcd_cmd_send(0x74, 0xa0);
            rm69330_spi_lcd_cmd_send(0x75, 0x00);
            rm69330_spi_lcd_cmd_send(0x76, 0x00);
            rm69330_spi_lcd_cmd_send(0x77, 0xe4);
            rm69330_spi_lcd_cmd_send(0x78, 0x00);
            rm69330_spi_lcd_cmd_send(0x79, 0x04);
            rm69330_spi_lcd_cmd_send(0x7a, 0x02);
            rm69330_spi_lcd_cmd_send(0x7b, 0x01);
            rm69330_spi_lcd_cmd_send(0x7c, 0x00);
            rm69330_spi_lcd_cmd_send(0x7d, 0x00);
            rm69330_spi_lcd_cmd_send(0x7e, 0x24);
            rm69330_spi_lcd_cmd_send(0x7f, 0x4c);
            rm69330_spi_lcd_cmd_send(0x80, 0x04);
            rm69330_spi_lcd_cmd_send(0x81, 0x0a);
            rm69330_spi_lcd_cmd_send(0x82, 0x02);
            rm69330_spi_lcd_cmd_send(0x83, 0xc1);
            rm69330_spi_lcd_cmd_send(0x84, 0x02);
            rm69330_spi_lcd_cmd_send(0x85, 0x18);
            rm69330_spi_lcd_cmd_send(0x86, 0x90);
            rm69330_spi_lcd_cmd_send(0x87, 0x60);
            rm69330_spi_lcd_cmd_send(0x88, 0x88);
            rm69330_spi_lcd_cmd_send(0x89, 0x02);
            rm69330_spi_lcd_cmd_send(0x8a, 0x09);
            rm69330_spi_lcd_cmd_send(0x8b, 0x0c);
            rm69330_spi_lcd_cmd_send(0x8c, 0x18);
            rm69330_spi_lcd_cmd_send(0x8d, 0x90);
            rm69330_spi_lcd_cmd_send(0x8e, 0x10);
            rm69330_spi_lcd_cmd_send(0x8f, 0x08);
            rm69330_spi_lcd_cmd_send(0x90, 0x00);
            rm69330_spi_lcd_cmd_send(0x91, 0x10);
            rm69330_spi_lcd_cmd_send(0x92, 0xa8);
            rm69330_spi_lcd_cmd_send(0x93, 0x00);
            rm69330_spi_lcd_cmd_send(0x94, 0x04);
            rm69330_spi_lcd_cmd_send(0x95, 0x0a);
            rm69330_spi_lcd_cmd_send(0x96, 0x00);
            rm69330_spi_lcd_cmd_send(0x97, 0x08);
            rm69330_spi_lcd_cmd_send(0x98, 0x10);
            rm69330_spi_lcd_cmd_send(0x99, 0x28);
            rm69330_spi_lcd_cmd_send(0x9a, 0x08);
            rm69330_spi_lcd_cmd_send(0x9b, 0x04);
            rm69330_spi_lcd_cmd_send(0x9c, 0x02);
            rm69330_spi_lcd_cmd_send(0x9d, 0x03);

            rm69330_spi_lcd_cmd_send(0xfe, 0x0c);
            rm69330_spi_lcd_cmd_send(0x25, 0x00);
            rm69330_spi_lcd_cmd_send(0x31, 0xef);
            rm69330_spi_lcd_cmd_send(0x32, 0xe3);
            rm69330_spi_lcd_cmd_send(0x33, 0x00);
            rm69330_spi_lcd_cmd_send(0x34, 0xe3);
            rm69330_spi_lcd_cmd_send(0x35, 0xe3);
            rm69330_spi_lcd_cmd_send(0x36, 0x80);
            rm69330_spi_lcd_cmd_send(0x37, 0x00);
            rm69330_spi_lcd_cmd_send(0x38, 0x79);
            rm69330_spi_lcd_cmd_send(0x39, 0x00);
            rm69330_spi_lcd_cmd_send(0x3a, 0x00);
            rm69330_spi_lcd_cmd_send(0x3b, 0x00);
            rm69330_spi_lcd_cmd_send(0x3d, 0x00);
            rm69330_spi_lcd_cmd_send(0x3f, 0x00);
            rm69330_spi_lcd_cmd_send(0x40, 0x00);
            rm69330_spi_lcd_cmd_send(0x41, 0x00);
            rm69330_spi_lcd_cmd_send(0x42, 0x00);
            rm69330_spi_lcd_cmd_send(0x43, 0x01);

            rm69330_spi_lcd_cmd_send(0xfe, 0x00);  //Write CMD mode page
            rm69330_spi_lcd_cmd_send(0xc4, 0x80);  //Set_DSPI Mode
            rm69330_spi_lcd_cmd_send(0x35, 0x00);  //Tearing effect line on
            rm69330_spi_lcd_cmd_send(0x51, 0xff);  //display brightness

            if(pixel_mode == LCD_PIXEL_mode_16bit) {
                rm69330_spi_lcd_cmd_send(0x3a, 0x75);  //16bit rgb
            } else if(pixel_mode == LCD_PIXEL_mode_24bit) {
                rm69330_spi_lcd_cmd_send(0x3a, 0x77);  //24bit rgb
            } else {

            }

            rm69330_spi_lcd_cmd_send(0x11, 0x00);       //wake up
            delay_ms(20);
            //rm69330_spi_lcd_cmd_send(0x23, 0x00);     //all pixel on
            rm69330_spi_lcd_cmd_send(0x29, 0x00);       //display on
            delay_ms(20);
            //rm69330_spi_lcd_cmd_send(0x12);           //Partial display mode on
        }
        break;

        default:
        {}
        break;
    }
}

void graphics_dc_rm69330_spi_lcd_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
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
    rm69330_spi_lcd_write(0x2a, data, 4);

    data[0] = (y1 & 0xff00) >> 8;
    data[1] = y1 & 0x00ff;
    data[2] = (y2 & 0xff00) >> 8;
    data[3] = y2 & 0x00ff;
    rm69330_spi_lcd_write(0x2b, data, 4);
}

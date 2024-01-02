#include "graphics_dc_lcd_drv.h"
#include "FreeRTOS.h"

/*
 * Defines
 *****************************************************************************************
 */
#define RM69330_DC_DSPI_PINS_CFG()     \
    {   \
        .csn = {APP_IO_PULLUP, ENABLE},  \
        .clk = {APP_IO_PULLUP, ENABLE},  \
        .io0 = {APP_IO_PULLUP, ENABLE},  \
        .io1 = {APP_IO_PULLUP, ENABLE},  \
        .io2 = {APP_IO_PULLUP, DISABLE},  \
        .io3 = {APP_IO_PULLUP, DISABLE},  \
        .dcx = {APP_IO_PULLUP, ENABLE}, \
    }

#define RM69330_DC_DSPI_CFG()   \
    {   \
        .mspi_mode      = GDC_MODE_DSPI,                 \
        .clock_freq     = GDC_CLOCK_FREQ_48MHz,          \
        .clock_mode     = GDC_CLOCK_MODE_0,              \
        .tcsu_cycle     = GDC_TCSU_CYCLE_1,              \
        .layer_mode     = GDC_ONE_LAYER_MODE,            \
        .mipicfg_format = GDC_MIPICFG_DSPI_RGB565_OPT0,  \
        .resolution_x   = 240,                           \
        .resolution_y   = 240,                           \
        .pins_cfg       = RM69330_DC_DSPI_PINS_CFG(),    \
    }

/****************************************************
 * DSPI LCD Size : 240*240*RGB565
 ****************************************************/
#define DSPI_LCD_WIDTH                      240u
#define DSPI_LCD_HEIGHT                     240u

#define DSPI_ST7789V_X_START                0u
#define DSPI_ST7789V_X_END                  (DSPI_LCD_WIDTH - 1)
#define DSPI_ST7789V_Y_START                40u      /* must start from 40 */
#define DSPI_ST7789V_Y_END                  (DSPI_ST7789V_Y_START + DSPI_LCD_HEIGHT - 1u)

/* USE GPIO30 to control RESET */
#define LCD_RESET_PIN                       APP_IO_PIN_14
#define LCD_RESET_GPIO                      APP_IO_TYPE_GPIOB

#define LCD_RES_W                           240u
#define LCD_RES_H                           240u

/*
 * Declaration
 *****************************************************************************************
 */
static void         st7789_dspi_lcd_set_reset_pin(uint8_t level);
static void         st7789_dspi_lcd_reset(void);
static void         st7789_dspi_cmd_seq(void);
static void         dc_irq_event_notify(uint32_t evt);

/*
 * Public
 *****************************************************************************************
 */

void graphics_dc_st7789_dspi_lcd_init(lcd_res_e res, lcd_pixel_mode_e pixel_mode, graphics_dc_mipi_format_e mipi_format) {

    app_graphics_dc_params_t dc_params = RM69330_DC_DSPI_CFG();
    dc_params.mipicfg_format = mipi_format;
    graphics_dc_init(&dc_params, dc_irq_event_notify);

    st7789_dspi_lcd_reset();
    st7789_dspi_cmd_seq();
}


/*
 * STATIC
 *****************************************************************************************
 */
static void st7789_dspi_lcd_set_reset_pin(uint8_t level)
{
    app_io_init_t io_init ;
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.mux  = APP_IO_MUX;
    io_init.pin  = LCD_RESET_PIN;
    io_init.pull = APP_IO_PULLUP;

    app_io_init(LCD_RESET_GPIO, &io_init);
    
    if (level) {
        app_io_write_pin(LCD_RESET_GPIO, LCD_RESET_PIN, APP_IO_PIN_SET);
    } else {
        app_io_write_pin(LCD_RESET_GPIO, LCD_RESET_PIN, APP_IO_PIN_RESET);
    }   
}

static void st7789_dspi_lcd_reset(void)  {
    st7789_dspi_lcd_set_reset_pin(0);
    delay_ms(100);
    st7789_dspi_lcd_set_reset_pin(1);
    delay_ms(100);
}

void graphics_dc_st7789_dspi_lcd_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
    uint16_t data[4];

    data[0] = (x1 & 0xff00);
    data[1] = (x1 & 0x00ff) << 8;
    data[2] = (x2 & 0xff00);
    data[3] = (x2 & 0x00ff) << 8;
    app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(0x2a00, data, 4);

    data[0] = (y1 & 0xff00);
    data[1] = (y1 & 0x00ff) << 8;
    data[2] = (y2 & 0xff00);
    data[3] = (y2 & 0x00ff) << 8;
    app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(0x2b00, data, 4);
}


static void dc_irq_event_notify(uint32_t evt) {
    //printf("DC Event: %d   \r\n", evt);
}

static void st7789_dspi_cmd_seq(void){
    // Write_Command(0x01);
    //app_graphics_dc_dspi_send_cmd_in_3wire_1lane(0x01);

    // Write_Command(0x11);
    app_graphics_dc_dspi_send_cmd_in_3wire_1lane(0x11);

    // Write_Command_Data(0xE7, 0x10);
    app_graphics_dc_dspi_send_cmd_data_in_3wire_1lane(0xE7, 0x10);

    // Write_Command_Data(0x36, 0x00);
    app_graphics_dc_dspi_send_cmd_data_in_4wire_2lane(0x3600, 0x0000);

    // Write_Command_Data(0x3A, 0x05);
    app_graphics_dc_dspi_send_cmd_data_in_4wire_2lane(0x3A00, 0x0500);

    // Write_Command_Data(0xB2, b2_data);
    uint16_t b2_data[5] = {0x0C00, 0x0C00, 0x0000, 0x3300, 0x3300};
    app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(0xB200, b2_data, 5);

    // Write_Command_Data(0xB7, 0x35);
    app_graphics_dc_dspi_send_cmd_data_in_4wire_2lane(0xB700, 0x3500);

    // Write_Command_Data(0xBB, 0x1E);
    app_graphics_dc_dspi_send_cmd_data_in_4wire_2lane(0xBB00, 0x1E00);

    // Write_Command_Data(0xC0, 0x2C);
    app_graphics_dc_dspi_send_cmd_data_in_4wire_2lane(0xC000, 0x2C00);

    // Write_Command_Data(0xC2, 0x01);
    app_graphics_dc_dspi_send_cmd_data_in_4wire_2lane(0xC200, 0x0100);

    // Write_Command_Data(0xC3, 0x03);
    app_graphics_dc_dspi_send_cmd_data_in_4wire_2lane(0xC300, 0x0300);

    // Write_Command_Data(0xC4, 0x20);
    app_graphics_dc_dspi_send_cmd_data_in_4wire_2lane(0xC400, 0x2000);

    // Write_Command_Data(0xC6, 0x0F);
    app_graphics_dc_dspi_send_cmd_data_in_4wire_2lane(0xC600, 0x0F00);

    // Write_Command_Data(0xD0, d0_data);
    uint16_t d0_data[] = {0xA400, 0xA100};
    app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(0xD000, d0_data, 2);

    // Write_Command_Data(0xE0, e0_data);
    uint16_t e0_data[] = {0xD000, 0x0600, 0x0C00, 0x0800, 0x0800, 0x2500, 0x3300,
                          0x3F00, 0x4F00, 0x2900, 0x1700, 0x1400, 0x3300, 0x3200};
    app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(0xE000, e0_data, 14);

    // Write_Command_Data(0xE1, e0_data);
    uint16_t e1_data[] = {0xD000, 0x0700, 0x0C00, 0x0900, 0x0800, 0x2500, 0x3400,
                          0x3800, 0x4E00, 0x2900, 0x1700, 0x1400, 0x3200, 0x3200};
    app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(0xE100, e1_data, 14);

#if 0
    // Write_Command_Data(0x2A, _2A_data);
    uint16_t _2A_data[] = {0x0000, 0x0000, 0x0000, 0xEF00};
    app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(0x2A00, _2A_data, 4);

    // Write_Command_Data(0x2B, _2B_data);
    uint16_t _2B_data[] = {0x0000, 0x0000, 0x0000, 0xEF00};
    app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(0x2B00, _2B_data, 4);
#endif

    // Write_Command(0x21);
    app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(0x2100, NULL, 0);

    // Write_Command(0x29);
    app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(0x2900, NULL, 0);

    //Write_Command(0x2C);
    app_graphics_dc_dspi_send_cmd_datas_in_4wire_2lane(0x2C00, NULL, 0);

    return;
}


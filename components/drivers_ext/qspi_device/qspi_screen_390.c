#include "app_qspi.h"
#include "app_qspi_dma.h"
#include "app_log.h"
#include "app_gpiote.h"
#include "app_io.h"
#include "qspi_screen_390.h"


#define SCREEN_QSPI_ID                           APP_QSPI_ID_2

#define DEFAULT_SCRN_MODE_CONFIG                 {DMA1, DMA_Channel0, 3000, 0}
#define DEFAULT_SCRN_QSPI_CONFIG                 {2, QSPI_CLOCK_MODE_3, 0}
#define DEFAULT_SCRN_PARAM_CONFIG                {SCREEN_QSPI_ID, g_qspi_pin_groups[QSPI2_PIN_GROUP_0], DEFAULT_SCRN_MODE_CONFIG, DEFAULT_SCRN_QSPI_CONFIG}

/* misc pins */
#if 0 // EVB_BOARD
    #define DISP_RST_CONFIG                  { APP_IO_TYPE_AON, APP_IO_PIN_5, APP_IO_MODE_OUTPUT, APP_IO_NOPULL, NULL}
    #define DISP_TE_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_1, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
    #define DISP_DC_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_6, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
#else //SK Board
    #define DISP_RST_CONFIG                  { APP_IO_TYPE_AON, APP_IO_PIN_6, APP_IO_MODE_OUTPUT, APP_IO_NOPULL, NULL}
    #define DISP_TE_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_5, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
    #define DISP_DC_CONFIG                   { APP_IO_TYPE_AON, APP_IO_PIN_7, APP_IO_MODE_INPUT,   APP_IO_NOPULL, NULL}
#endif

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

uint32_t screen_init(app_qspi_id_t id, uint32_t clock_prescaler, app_qspi_evt_handler_t evt_handler)
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
        return 1;
    }

    ret = app_qspi_dma_init(&g_qspi_screen_params);
    if (ret != 0)
    {
        APP_LOG_ERROR("QSPI initial dma failed! Please check the input paraments.");
        return 1;
    }

    return psram_id;
}

void qspi_screen_deinit(app_qspi_id_t id)
{
    app_qspi_deinit(id);
}

static void lcd_io_init(app_gpiote_param_t io_para)
{
    app_io_init_t io_init;
    io_init.pin  = io_para.pin;
    io_init.mode = io_para.mode;
    io_init.pull = io_para.pull;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5525X)
        io_init.mux  = APP_IO_MUX_7;
#else
        io_init.mux  = APP_IO_MUX_8;
#endif

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

static void lcd_cmd_send(uint8_t cmd, uint8_t param)
{
    lcd_write(cmd, &param, 1);
}


static void lcd_cmd_sequence(rm69330_lcd_res_e res)
{
    switch(res) {
        case RM69330_LCD_RES_390:
        {
            lcd_cmd_send(0xfe, 0x01);
            lcd_cmd_send(0x05, 0x10);
            lcd_cmd_send(0x06, 0x62);
            lcd_cmd_send(0x0d, 0x00);
            lcd_cmd_send(0x0e, 0x81);
            lcd_cmd_send(0x0f, 0x81);
            lcd_cmd_send(0x10, 0x11);
            lcd_cmd_send(0x11, 0x81);
            lcd_cmd_send(0x12, 0x81);
            lcd_cmd_send(0x13, 0x80);
            lcd_cmd_send(0x14, 0x80);
            lcd_cmd_send(0x15, 0x81);
            lcd_cmd_send(0x16, 0x81);
            lcd_cmd_send(0x18, 0x66);
            lcd_cmd_send(0x19, 0x88);
            lcd_cmd_send(0x5b, 0x10);
            lcd_cmd_send(0x5c, 0x55);
            lcd_cmd_send(0x62, 0x19);
            lcd_cmd_send(0x63, 0x19);
            lcd_cmd_send(0x70, 0x55);
            lcd_cmd_send(0x74, 0x0c);
            lcd_cmd_send(0xc5, 0x10);
            lcd_cmd_send(0x25, 0x03);
            lcd_cmd_send(0x26, 0x80);
            lcd_cmd_send(0x27, 0x08);
            lcd_cmd_send(0x28, 0x08);
            lcd_cmd_send(0x2a, 0x23);
            lcd_cmd_send(0x2b, 0x80);
            lcd_cmd_send(0x2d, 0x08);
            lcd_cmd_send(0x2f, 0x08);
            lcd_cmd_send(0x30, 0x43);   //0x43: 15Hz
            lcd_cmd_send(0x66, 0x90);
            lcd_cmd_send(0x72, 0x1a);
            lcd_cmd_send(0x73, 0x13);
            lcd_cmd_send(0xfe, 0x01);
            lcd_cmd_send(0x6a, 0x17);   //0x17: -2.2V for rt4723
            lcd_cmd_send(0x1b, 0x00);
            lcd_cmd_send(0x1d, 0x03);
            lcd_cmd_send(0x1e, 0x03);
            lcd_cmd_send(0x1f, 0x0c);
            lcd_cmd_send(0x20, 0x03);
            lcd_cmd_send(0xfe, 0x01);
            lcd_cmd_send(0x36, 0x00);
            lcd_cmd_send(0x6c, 0x80);
            lcd_cmd_send(0x6d, 0x19);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x63, 0x00);
            lcd_cmd_send(0x64, 0x0e);
            lcd_cmd_send(0xfe, 0x02);
            lcd_cmd_send(0xa9, 0x40);
            lcd_cmd_send(0xaa, 0xb8);
            lcd_cmd_send(0xab, 0x01);
            lcd_cmd_send(0xfe, 0x03);
            lcd_cmd_send(0xa9, 0x40);
            lcd_cmd_send(0xaa, 0x90);
            lcd_cmd_send(0xab, 0x01);
            lcd_cmd_send(0xfe, 0x01);
            lcd_cmd_send(0x3a, 0x00);
            lcd_cmd_send(0x3b, 0x41);
            lcd_cmd_send(0x3d, 0x17);
            lcd_cmd_send(0x3f, 0x42);
            lcd_cmd_send(0x40, 0x17);
            lcd_cmd_send(0x41, 0x06);
            lcd_cmd_send(0x37, 0x0c);
            lcd_cmd_send(0xfe, 0x0c);
            lcd_cmd_send(0x07, 0x1f);
            lcd_cmd_send(0x08, 0x2f);
            lcd_cmd_send(0x09, 0x3f);
            lcd_cmd_send(0x0a, 0x4f);
            lcd_cmd_send(0x0b, 0x5f);
            lcd_cmd_send(0x0c, 0x6f);
            lcd_cmd_send(0x0d, 0xff);
            lcd_cmd_send(0x0e, 0xff);
            lcd_cmd_send(0x0f, 0xff);
            lcd_cmd_send(0x10, 0xff);
            lcd_cmd_send(0xfe, 0x01);
            lcd_cmd_send(0x42, 0x14);
            lcd_cmd_send(0x43, 0x41);
            lcd_cmd_send(0x44, 0x25);
            lcd_cmd_send(0x45, 0x52);
            lcd_cmd_send(0x46, 0x36);
            lcd_cmd_send(0x47, 0x63);
            lcd_cmd_send(0x48, 0x41);
            lcd_cmd_send(0x49, 0x14);
            lcd_cmd_send(0x4a, 0x52);
            lcd_cmd_send(0x4b, 0x25);
            lcd_cmd_send(0x4c, 0x63);
            lcd_cmd_send(0x4d, 0x36);
            lcd_cmd_send(0x4e, 0x36);
            lcd_cmd_send(0x4f, 0x63);
            lcd_cmd_send(0x50, 0x25);
            lcd_cmd_send(0x51, 0x52);
            lcd_cmd_send(0x52, 0x14);
            lcd_cmd_send(0x53, 0x41);
            lcd_cmd_send(0x54, 0x63);
            lcd_cmd_send(0x55, 0x36);
            lcd_cmd_send(0x56, 0x52);
            lcd_cmd_send(0x57, 0x25);
            lcd_cmd_send(0x58, 0x41);
            lcd_cmd_send(0x59, 0x14);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x5d, 0x01);
            lcd_cmd_send(0x75, 0x08);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x5e, 0x9f);
            lcd_cmd_send(0x5f, 0x43);
            lcd_cmd_send(0x60, 0xff);
            lcd_cmd_send(0x61, 0xff);
            lcd_cmd_send(0x62, 0xff);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x76, 0xff);
            lcd_cmd_send(0x77, 0xff);
            lcd_cmd_send(0x78, 0x10);
            lcd_cmd_send(0x79, 0xf2);
            lcd_cmd_send(0x7a, 0xff);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x00, 0x8d);
            lcd_cmd_send(0x01, 0x00);
            lcd_cmd_send(0x02, 0x00);
            lcd_cmd_send(0x03, 0x05);
            lcd_cmd_send(0x04, 0x00);
            lcd_cmd_send(0x05, 0x05);
            lcd_cmd_send(0x06, 0x00);
            lcd_cmd_send(0x07, 0x00);
            lcd_cmd_send(0x08, 0x00);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x09, 0xcc);
            lcd_cmd_send(0x0a, 0x00);
            lcd_cmd_send(0x0b, 0x02);
            lcd_cmd_send(0x0c, 0x00);
            lcd_cmd_send(0x0d, 0x60);
            lcd_cmd_send(0x0e, 0x06);
            lcd_cmd_send(0x0f, 0x2c);
            lcd_cmd_send(0x10, 0x53);
            lcd_cmd_send(0x11, 0x00);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x12, 0xcc);
            lcd_cmd_send(0x13, 0x00);
            lcd_cmd_send(0x14, 0x02);
            lcd_cmd_send(0x15, 0x00);
            lcd_cmd_send(0x16, 0x60);
            lcd_cmd_send(0x17, 0x05);
            lcd_cmd_send(0x18, 0x2c);
            lcd_cmd_send(0x19, 0x53);
            lcd_cmd_send(0x1a, 0x00);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x1b, 0xdc);
            lcd_cmd_send(0x1c, 0x00);
            lcd_cmd_send(0x1d, 0x04);
            lcd_cmd_send(0x1e, 0x02);
            lcd_cmd_send(0x1f, 0x18);
            lcd_cmd_send(0x20, 0x06);
            lcd_cmd_send(0x21, 0x3d);
            lcd_cmd_send(0x22, 0x75);
            lcd_cmd_send(0x23, 0x00);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x24, 0xdc);
            lcd_cmd_send(0x25, 0x00);
            lcd_cmd_send(0x26, 0x04);
            lcd_cmd_send(0x27, 0x02);
            lcd_cmd_send(0x28, 0x18);
            lcd_cmd_send(0x29, 0x04);
            lcd_cmd_send(0x2a, 0x3d);
            lcd_cmd_send(0x2b, 0x75);
            lcd_cmd_send(0x2d, 0x00);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x53, 0x8a);
            lcd_cmd_send(0x54, 0x78);
            lcd_cmd_send(0x55, 0x08);
            lcd_cmd_send(0x56, 0x0a);
            lcd_cmd_send(0x58, 0x2a);
            lcd_cmd_send(0x59, 0x00);
            lcd_cmd_send(0x65, 0x02);
            lcd_cmd_send(0x66, 0x0a);
            lcd_cmd_send(0x67, 0x00);
            lcd_cmd_send(0xfe, 0x07);
            lcd_cmd_send(0x15, 0x04);

            lcd_cmd_send(0xfe, 0x00);  //Write CMD mode page
            lcd_cmd_send(0xc4, 0x80);  //Set_DSPI Mode
            lcd_cmd_send(0x35, 0x00);  //Tearing effect line on
            lcd_cmd_send(0x51, 0xff);  //display brightness
            lcd_cmd_send(0x3a, 0x75);  //16bit rgb

            lcd_cmd_send(0x11, 0x00);  //wake up
            delay_ms(20);
            //lcd_cmd_send(0x23, 0x00);  //all pixel on
            lcd_cmd_send(0x29, 0x00);  //display on
            delay_ms(20);
            //lcd_cmd_send(0x12);  //Partial display mode on
        }
        break;

        case RM69330_LCD_RES_454:
        {
            lcd_cmd_send(0xfe, 0x01);
            lcd_cmd_send(0x05, 0x00);
            lcd_cmd_send(0x06, 0x72);
            lcd_cmd_send(0x0d, 0x00);
            lcd_cmd_send(0x0e, 0x81);
            lcd_cmd_send(0x0f, 0x81);
            lcd_cmd_send(0x10, 0x11);
            lcd_cmd_send(0x11, 0x81);
            lcd_cmd_send(0x12, 0x81);
            lcd_cmd_send(0x13, 0x80);
            lcd_cmd_send(0x14, 0x80);
            lcd_cmd_send(0x15, 0x81);
            lcd_cmd_send(0x16, 0x81);
            lcd_cmd_send(0x18, 0x66);
            lcd_cmd_send(0x19, 0x88);
            lcd_cmd_send(0x5b, 0x10);
            lcd_cmd_send(0x5c, 0x55);
            lcd_cmd_send(0x62, 0x19);
            lcd_cmd_send(0x63, 0x19);
            lcd_cmd_send(0x70, 0x54);
            lcd_cmd_send(0x74, 0x0c);
            lcd_cmd_send(0xc5, 0x10);

            lcd_cmd_send(0xfe, 0x01);
            lcd_cmd_send(0x25, 0x03);
            lcd_cmd_send(0x26, 0x32);
            lcd_cmd_send(0x27, 0x0a);
            lcd_cmd_send(0x28, 0x08);
            lcd_cmd_send(0x2a, 0x03);
            lcd_cmd_send(0x2b, 0x32);
            lcd_cmd_send(0x2d, 0x0a);
            lcd_cmd_send(0x2f, 0x08);
            lcd_cmd_send(0x30, 0x43);   //0x43: 15Hz

            lcd_cmd_send(0x66, 0x90);
            lcd_cmd_send(0x72, 0x1a);
            lcd_cmd_send(0x73, 0x13);

            lcd_cmd_send(0xfe, 0x01);
            lcd_cmd_send(0x6a, 0x17);   //0x17: -2.2V for rt4723
            lcd_cmd_send(0x1b, 0x00);
            lcd_cmd_send(0x1d, 0x03);
            lcd_cmd_send(0x1e, 0x03);
            lcd_cmd_send(0x1f, 0x03);
            lcd_cmd_send(0x20, 0x03);
            lcd_cmd_send(0xfe, 0x01);
            lcd_cmd_send(0x36, 0x00);
            lcd_cmd_send(0x6c, 0x80);
            lcd_cmd_send(0x6d, 0x19);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x63, 0x00);
            lcd_cmd_send(0x64, 0x0e);
            lcd_cmd_send(0xfe, 0x02);
            lcd_cmd_send(0xa9, 0x30);
            lcd_cmd_send(0xaa, 0xb9);
            lcd_cmd_send(0xab, 0x01);
            lcd_cmd_send(0xfe, 0x03);
            lcd_cmd_send(0xa9, 0x30);
            lcd_cmd_send(0xaa, 0x90);
            lcd_cmd_send(0xab, 0x01);


            lcd_cmd_send(0xfe, 0x0c);
            lcd_cmd_send(0x07, 0x1f);
            lcd_cmd_send(0x08, 0x2f);
            lcd_cmd_send(0x09, 0x3f);
            lcd_cmd_send(0x0a, 0x4f);
            lcd_cmd_send(0x0b, 0x5f);
            lcd_cmd_send(0x0c, 0x6f);
            lcd_cmd_send(0x0d, 0xff);
            lcd_cmd_send(0x0e, 0xff);
            lcd_cmd_send(0x0f, 0xff);
            lcd_cmd_send(0x10, 0xff);
            lcd_cmd_send(0xfe, 0x01);
            lcd_cmd_send(0x42, 0x14);
            lcd_cmd_send(0x43, 0x41);
            lcd_cmd_send(0x44, 0x25);
            lcd_cmd_send(0x45, 0x52);
            lcd_cmd_send(0x46, 0x36);
            lcd_cmd_send(0x47, 0x63);
            lcd_cmd_send(0x48, 0x41);
            lcd_cmd_send(0x49, 0x14);
            lcd_cmd_send(0x4a, 0x52);
            lcd_cmd_send(0x4b, 0x25);
            lcd_cmd_send(0x4c, 0x63);
            lcd_cmd_send(0x4d, 0x36);
            lcd_cmd_send(0x4e, 0x16);
            lcd_cmd_send(0x4f, 0x61);
            lcd_cmd_send(0x50, 0x25);
            lcd_cmd_send(0x51, 0x52);
            lcd_cmd_send(0x52, 0x34);
            lcd_cmd_send(0x53, 0x43);
            lcd_cmd_send(0x54, 0x61);
            lcd_cmd_send(0x55, 0x16);
            lcd_cmd_send(0x56, 0x52);
            lcd_cmd_send(0x57, 0x25);
            lcd_cmd_send(0x58, 0x43);
            lcd_cmd_send(0x59, 0x34);

            lcd_cmd_send(0xfe, 0x01);
            lcd_cmd_send(0x3a, 0x00);
            lcd_cmd_send(0x3b, 0x00);
            lcd_cmd_send(0x3d, 0x12);
            lcd_cmd_send(0x3f, 0x37);
            lcd_cmd_send(0x40, 0x12);
            lcd_cmd_send(0x41, 0x0f);
            lcd_cmd_send(0x37, 0x0c);

            lcd_cmd_send(0xfe, 0x04);
        //    lcd_cmd_send(0x5d, 0x01);
        //    lcd_cmd_send(0x75, 0x08);
        //    lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x5e, 0x0f);
            lcd_cmd_send(0x5f, 0x12);
            lcd_cmd_send(0x60, 0xff);
            lcd_cmd_send(0x61, 0xff);
            lcd_cmd_send(0x62, 0xff);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x76, 0xff);
            lcd_cmd_send(0x77, 0xff);
            lcd_cmd_send(0x78, 0x49);
            lcd_cmd_send(0x79, 0xf3);
            lcd_cmd_send(0x7a, 0xff);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x00, 0x9d);
            lcd_cmd_send(0x01, 0x00);
            lcd_cmd_send(0x02, 0x00);
            lcd_cmd_send(0x03, 0x00);
            lcd_cmd_send(0x04, 0x00);
            lcd_cmd_send(0x05, 0x01);
            lcd_cmd_send(0x06, 0x01);
            lcd_cmd_send(0x07, 0x01);
            lcd_cmd_send(0x08, 0x00);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x09, 0xdc);
            lcd_cmd_send(0x0a, 0x00);
            lcd_cmd_send(0x0b, 0x02);
            lcd_cmd_send(0x0c, 0x00);
            lcd_cmd_send(0x0d, 0x08);
            lcd_cmd_send(0x0e, 0x01);
            lcd_cmd_send(0x0f, 0xce);
            lcd_cmd_send(0x10, 0x16);
            lcd_cmd_send(0x11, 0x00);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x12, 0xdc);
            lcd_cmd_send(0x13, 0x00);
            lcd_cmd_send(0x14, 0x02);
            lcd_cmd_send(0x15, 0x00);
            lcd_cmd_send(0x16, 0x08);
            lcd_cmd_send(0x17, 0x02);
            lcd_cmd_send(0x18, 0xce);
            lcd_cmd_send(0x19, 0x16);
            lcd_cmd_send(0x1a, 0x00);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x1b, 0xdc);
            lcd_cmd_send(0x1c, 0x00);
            lcd_cmd_send(0x1d, 0x02);
            lcd_cmd_send(0x1e, 0x00);
            lcd_cmd_send(0x1f, 0x08);
            lcd_cmd_send(0x20, 0x01);
            lcd_cmd_send(0x21, 0xce);
            lcd_cmd_send(0x22, 0x16);
            lcd_cmd_send(0x23, 0x00);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x24, 0xdc);
            lcd_cmd_send(0x25, 0x00);
            lcd_cmd_send(0x26, 0x02);
            lcd_cmd_send(0x27, 0x00);
            lcd_cmd_send(0x28, 0x08);
            lcd_cmd_send(0x29, 0x02);
            lcd_cmd_send(0x2a, 0xce);
            lcd_cmd_send(0x2b, 0x16);
            lcd_cmd_send(0x2d, 0x00);
            lcd_cmd_send(0xfe, 0x04);
            lcd_cmd_send(0x53, 0x8a);
            lcd_cmd_send(0x54, 0x00);
            lcd_cmd_send(0x55, 0x03);
            lcd_cmd_send(0x56, 0x01);
            lcd_cmd_send(0x58, 0x01);
            lcd_cmd_send(0x59, 0x00);
            lcd_cmd_send(0x65, 0x76);
            lcd_cmd_send(0x66, 0x19);
            lcd_cmd_send(0x67, 0x00);
            lcd_cmd_send(0xfe, 0x07);
            lcd_cmd_send(0x15, 0x04);

            lcd_cmd_send(0xfe, 0x05);
            lcd_cmd_send(0x4c, 0x01);
            lcd_cmd_send(0x4d, 0x82);
            lcd_cmd_send(0x4e, 0x04);
            lcd_cmd_send(0x4f, 0x00);
            lcd_cmd_send(0x50, 0x20);
            lcd_cmd_send(0x51, 0x10);
            lcd_cmd_send(0x52, 0x04);
            lcd_cmd_send(0x53, 0x41);
            lcd_cmd_send(0x54, 0x0a);
            lcd_cmd_send(0x55, 0x08);
            lcd_cmd_send(0x56, 0x00);
            lcd_cmd_send(0x57, 0x28);
            lcd_cmd_send(0x58, 0x00);
            lcd_cmd_send(0x59, 0x80);
            lcd_cmd_send(0x5a, 0x04);
            lcd_cmd_send(0x5b, 0x10);
            lcd_cmd_send(0x5c, 0x20);
            lcd_cmd_send(0x5d, 0x00);
            lcd_cmd_send(0x5e, 0x04);
            lcd_cmd_send(0x5f, 0x0a);
            lcd_cmd_send(0x60, 0x01);
            lcd_cmd_send(0x61, 0x08);
            lcd_cmd_send(0x62, 0x00);
            lcd_cmd_send(0x63, 0x20);
            lcd_cmd_send(0x64, 0x40);
            lcd_cmd_send(0x65, 0x04);
            lcd_cmd_send(0x66, 0x02);
            lcd_cmd_send(0x67, 0x48);
            lcd_cmd_send(0x68, 0x4c);
            lcd_cmd_send(0x69, 0x02);
            lcd_cmd_send(0x6a, 0x12);
            lcd_cmd_send(0x6b, 0x00);
            lcd_cmd_send(0x6c, 0x48);
            lcd_cmd_send(0x6d, 0xa0);
            lcd_cmd_send(0x6e, 0x08);
            lcd_cmd_send(0x6f, 0x04);
            lcd_cmd_send(0x70, 0x05);
            lcd_cmd_send(0x71, 0x92);
            lcd_cmd_send(0x72, 0x00);
            lcd_cmd_send(0x73, 0x18);
            lcd_cmd_send(0x74, 0xa0);
            lcd_cmd_send(0x75, 0x00);
            lcd_cmd_send(0x76, 0x00);
            lcd_cmd_send(0x77, 0xe4);
            lcd_cmd_send(0x78, 0x00);
            lcd_cmd_send(0x79, 0x04);
            lcd_cmd_send(0x7a, 0x02);
            lcd_cmd_send(0x7b, 0x01);
            lcd_cmd_send(0x7c, 0x00);
            lcd_cmd_send(0x7d, 0x00);
            lcd_cmd_send(0x7e, 0x24);
            lcd_cmd_send(0x7f, 0x4c);
            lcd_cmd_send(0x80, 0x04);
            lcd_cmd_send(0x81, 0x0a);
            lcd_cmd_send(0x82, 0x02);
            lcd_cmd_send(0x83, 0xc1);
            lcd_cmd_send(0x84, 0x02);
            lcd_cmd_send(0x85, 0x18);
            lcd_cmd_send(0x86, 0x90);
            lcd_cmd_send(0x87, 0x60);
            lcd_cmd_send(0x88, 0x88);
            lcd_cmd_send(0x89, 0x02);
            lcd_cmd_send(0x8a, 0x09);
            lcd_cmd_send(0x8b, 0x0c);
            lcd_cmd_send(0x8c, 0x18);
            lcd_cmd_send(0x8d, 0x90);
            lcd_cmd_send(0x8e, 0x10);
            lcd_cmd_send(0x8f, 0x08);
            lcd_cmd_send(0x90, 0x00);
            lcd_cmd_send(0x91, 0x10);
            lcd_cmd_send(0x92, 0xa8);
            lcd_cmd_send(0x93, 0x00);
            lcd_cmd_send(0x94, 0x04);
            lcd_cmd_send(0x95, 0x0a);
            lcd_cmd_send(0x96, 0x00);
            lcd_cmd_send(0x97, 0x08);
            lcd_cmd_send(0x98, 0x10);
            lcd_cmd_send(0x99, 0x28);
            lcd_cmd_send(0x9a, 0x08);
            lcd_cmd_send(0x9b, 0x04);
            lcd_cmd_send(0x9c, 0x02);
            lcd_cmd_send(0x9d, 0x03);

            lcd_cmd_send(0xfe, 0x0c);
            lcd_cmd_send(0x25, 0x00);
            lcd_cmd_send(0x31, 0xef);
            lcd_cmd_send(0x32, 0xe3);
            lcd_cmd_send(0x33, 0x00);
            lcd_cmd_send(0x34, 0xe3);
            lcd_cmd_send(0x35, 0xe3);
            lcd_cmd_send(0x36, 0x80);
            lcd_cmd_send(0x37, 0x00);
            lcd_cmd_send(0x38, 0x79);
            lcd_cmd_send(0x39, 0x00);
            lcd_cmd_send(0x3a, 0x00);
            lcd_cmd_send(0x3b, 0x00);
            lcd_cmd_send(0x3d, 0x00);
            lcd_cmd_send(0x3f, 0x00);
            lcd_cmd_send(0x40, 0x00);
            lcd_cmd_send(0x41, 0x00);
            lcd_cmd_send(0x42, 0x00);
            lcd_cmd_send(0x43, 0x01);


            lcd_cmd_send(0xfe, 0x00);  //Write CMD mode page
            lcd_cmd_send(0xc4, 0x80);  //Set_DSPI Mode
            lcd_cmd_send(0x35, 0x00);  //Tearing effect line on
            lcd_cmd_send(0x51, 0xff);  //display brightness
            lcd_cmd_send(0x3a, 0x75);  //16bit rgb

            lcd_cmd_send(0x11, 0x00);  //wake up
            delay_ms(20);
            //lcd_cmd_send(0x23, 0x00);  //all pixel on
            lcd_cmd_send(0x29, 0x00);  //display on
            delay_ms(20);
            //lcd_cmd_send(0x12);  //Partial display mode on
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

void qspi_screen_init_basic(app_qspi_id_t id, uint32_t clock_prescaler, rm69330_lcd_res_e res, app_qspi_evt_handler_t evt_handler) {
    screen_init(id, clock_prescaler, evt_handler);

    qspi_screen_misc_pins_init();
    lcd_rst_ctrl(0);
    delay_ms(100);
    lcd_rst_ctrl(1);
    delay_ms(100);

    lcd_cmd_sequence(res);
}

void qspi_screen_init(app_qspi_id_t id, uint32_t clock_prescaler, app_qspi_evt_handler_t evt_handler) {
    rm69330_lcd_res_e default_res = RM69330_LCD_RES_390;
    qspi_screen_init_basic(id, clock_prescaler, default_res, evt_handler);
}

void qspi_screen_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2) {
    lcd_ca_ra_set(x1, x2, y1, y2);
}

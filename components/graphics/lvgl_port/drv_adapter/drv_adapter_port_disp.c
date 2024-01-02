#include "drv_adapter_port.h"
#include "graphics_dc_lcd_drv.h"
#include "graphics_hybrid_rm69330_drv.h"

#define DISPLAY_SCRN_TYPE       0       /* 0 - RM69330; 1 - FLS AMO139 */
#define TE_WAITING_USE_SEMA     1

/***********************************************************************************
 *                 Static Declarations For Display
 ***********************************************************************************/
static void _disp_drv_init(disp_drv_t * dev) ;
static void _disp_drv_deinit(disp_drv_t * dev);
static void _disp_drv_set_show_area(disp_drv_t * dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) ;
static void _disp_drv_flush(disp_drv_t * dev, void * buff, uint32_t buff_format, uint16_t w, uint16_t h);
static void _disp_drv_wait_te(disp_drv_t * dev) ;
static void _disp_drv_wait_to_flush(disp_drv_t * dev) ;
static void _disp_drv_on(disp_drv_t * dev, bool on) ;
static void _disp_drv_set_brightness(disp_drv_t * dev, uint32_t percent) ;
static void _disp_drv_sleep(disp_drv_t * dev) ;
static void _disp_drv_wakeup(disp_drv_t * dev) ;
static void _disp_drv_te_evt_callback(app_io_evt_t *p_evt);

#if TE_WAITING_USE_SEMA
    #include "FreeRTOS.h"
    #include "task.h"
    #include "semphr.h"
    static SemaphoreHandle_t s_te_sema;
#else
    static volatile bool s_te_sema;
#endif // TE_WAITING_USE_SEMA

static volatile bool s_te_waiting = false;



/***********************************************************************************
 *                 Public Implements
 ***********************************************************************************/

void drv_adapter_disp_register(void) {

    disp_drv_t _disp_dev = {
        .idx                        = 0,
        .dev_id                     = 0,
        .user_data                  = NULL,
        .disp_drv_init              = _disp_drv_init,
        .disp_drv_deinit            = _disp_drv_deinit,
        .disp_drv_set_show_area     = _disp_drv_set_show_area,
        .disp_drv_flush             = _disp_drv_flush,
        .disp_drv_wait_te           = _disp_drv_wait_te,
        .disp_drv_on                = _disp_drv_on,
        .disp_drv_set_brightness    = _disp_drv_set_brightness,
        .disp_drv_wait_to_flush     = _disp_drv_wait_to_flush,
        .disp_drv_sleep             = _disp_drv_sleep,
        .disp_drv_wakeup            = _disp_drv_wakeup,
    };

    drv_adapter_disp_reg(0, &_disp_dev);

    return;
}


/***********************************************************************************
 *                 Static Implements for Display
 ***********************************************************************************/

static void _disp_drv_init(disp_drv_t * dev) {
    dev = dev;
#if DISPLAY_SCRN_TYPE == 0
    graphics_hybrid_rm69330_init(454, 454, GDC_MIPICFG_QSPI_RGB565_OPT0);
#else
    graphics_dc_am139_qspi_lcd_init(LCD_RES_454, LCD_PIXEL_mode_16bit, GDC_MIPICFG_QSPI_RGB565_OPT0);
#endif

    app_io_init_t te_init = {
        .pin = APP_IO_PIN_5,
        .mode = APP_IO_MODE_IT_FALLING,
        .pull = APP_IO_PULLUP,
        .mux = APP_IO_MUX,
    };
    app_io_event_register_cb(APP_IO_TYPE_AON, &te_init, _disp_drv_te_evt_callback, NULL);

#if TE_WAITING_USE_SEMA
    s_te_sema = xSemaphoreCreateBinary();
#endif // TE_WAITING_USE_SEMA
}

static void _disp_drv_set_show_area(disp_drv_t * dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    dev = dev;
#if DISPLAY_SCRN_TYPE == 0
    graphics_hybrid_rm69330_set_show_area(x1, x2, y1, y2);
#else
    graphics_dc_am139_qspi_lcd_set_show_area(x1, x2, y1, y2);
#endif
}

static void _disp_drv_deinit(disp_drv_t * dev) {

}

static void _disp_drv_flush(disp_drv_t * dev, void * buff, uint32_t buff_format, uint16_t w, uint16_t h) {
    graphics_hybrid_rm69330_flush(buff, buff_format, w, h);
}

static void _disp_drv_wait_te(disp_drv_t * dev) {
    s_te_waiting = true;
#if TE_WAITING_USE_SEMA
    xSemaphoreTake(s_te_sema, portMAX_DELAY);
#else
    while(!s_te_sema);
    s_te_sema = false;
#endif // TE_WAITING_USE_SEMA
    s_te_waiting = false;
}

static void _disp_drv_wait_to_flush(disp_drv_t * dev) {
    graphics_hybrid_rm69330_wait_ready();
}

static void _disp_drv_on(disp_drv_t * dev, bool on) {
    printf("Set Disp %s \r\n", on ? "ON" : "OFF");
    graphics_hybrid_rm69330_set_on(on);
}

static void _disp_drv_set_brightness(disp_drv_t * dev, uint32_t percent) {
    printf("Set Disp Brightness %d \r\n", percent);
}

static void _disp_drv_sleep(disp_drv_t * dev) {

}

static void _disp_drv_wakeup(disp_drv_t * dev) {

}

static void _disp_drv_te_evt_callback(app_io_evt_t *p_evt) {
    if (s_te_waiting)
    {
#if TE_WAITING_USE_SEMA
        BaseType_t xHigherPriorityTaskWoken;
        xSemaphoreGiveFromISR(s_te_sema, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
        s_te_sema = true;
#endif // TE_WAITING_USE_SEMA
    }
}


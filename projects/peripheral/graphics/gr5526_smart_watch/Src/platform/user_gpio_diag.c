#include "gr55xx_sys.h"
#include "app_io.h"

void gpio_8_high()
{
    app_io_write_pin(APP_IO_TYPE_GPIOA, GPIO_PIN_8, APP_IO_PIN_SET);
}

void gpio_8_low()
{
    app_io_write_pin(APP_IO_TYPE_GPIOA, GPIO_PIN_8, APP_IO_PIN_RESET);
}

void gpio_init(void)
{
    app_io_init_t io_config = APP_IO_DEFAULT_CONFIG;
    io_config.pin = GPIO_PIN_8;
    io_config.mode = APP_IO_MODE_OUTPUT;
    app_io_init(APP_IO_TYPE_GPIOA, &io_config);
}

void app_msio_output_init(void)
{
    app_io_init_t io_init = 
    {
        .pin = APP_IO_PIN_0 | APP_IO_PIN_1 | APP_IO_PIN_2 | APP_IO_PIN_3,
        .mode = APP_IO_MODE_OUTPUT,
        .pull = APP_IO_PULLUP,
        .mux = APP_IO_MUX_7,
    };
    app_io_init(APP_IO_TYPE_MSIO, &io_init);
}


void app_msio_level_set(uint32_t pin, uint8_t level)
{
    app_io_write_pin(APP_IO_TYPE_MSIO, 0x00000001 << pin, (app_io_pin_state_t)level);
}


void app_msio_level_toggle(uint32_t pin)
{
    app_io_toggle_pin(APP_IO_TYPE_MSIO, 0x00000001 << pin);
}

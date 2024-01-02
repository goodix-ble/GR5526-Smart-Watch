#include "drv_adapter_display.h"

static disp_drv_t _disp_drv_dev[DISPLAY_DEV_MAX];
static uint32_t       _s_current_disp_dev_idx = 0;


bool drv_adapter_disp_reg(uint32_t index, disp_drv_t * dev)
{
    if(index >= DISPLAY_DEV_MAX) {
        return false;
    }

    _disp_drv_dev[index].idx                        = index;
    _disp_drv_dev[index].dev_id                     = dev->dev_id;
    _disp_drv_dev[index].user_data                  = dev->user_data;
    _disp_drv_dev[index].disp_drv_init              = dev->disp_drv_init;
    _disp_drv_dev[index].disp_drv_deinit            = dev->disp_drv_deinit;
    _disp_drv_dev[index].disp_drv_set_show_area     = dev->disp_drv_set_show_area;
    _disp_drv_dev[index].disp_drv_flush             = dev->disp_drv_flush;
    _disp_drv_dev[index].disp_drv_wait_te           = dev->disp_drv_wait_te;
    _disp_drv_dev[index].disp_drv_wait_to_flush     = dev->disp_drv_wait_to_flush;
    _disp_drv_dev[index].disp_drv_on                = dev->disp_drv_on;
    _disp_drv_dev[index].disp_drv_set_brightness    = dev->disp_drv_set_brightness;
    _disp_drv_dev[index].disp_drv_sleep             = dev->disp_drv_sleep;
    _disp_drv_dev[index].disp_drv_sleep             = dev->disp_drv_sleep;
    _disp_drv_dev[index].disp_drv_wakeup            = dev->disp_drv_wakeup;

    _s_current_disp_dev_idx = index;

    return true;
}

void drv_adapter_disp_init(void) {

    disp_drv_t * dev = &_disp_drv_dev[_s_current_disp_dev_idx];

    if(dev->disp_drv_init) {
        dev->disp_drv_init(dev);
    }

    return;
}

void drv_adapter_disp_deinit(void) {
    disp_drv_t * dev = &_disp_drv_dev[_s_current_disp_dev_idx];

    if(dev->disp_drv_deinit) {
        dev->disp_drv_deinit(dev);
    }

    return;
}

void drv_adapter_disp_set_show_area(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {

    disp_drv_t * dev = &_disp_drv_dev[_s_current_disp_dev_idx];

    static uint16_t last_x1 = 0;
    static uint16_t last_y1 = 0;
    static uint16_t last_x2 = 0;
    static uint16_t last_y2 = 0;

    if (last_x1 == x1 && last_y1 == y1 && last_x2 == x2 && last_y2 == y2)
    {
        return;
    }

    last_x1 = x1;
    last_y1 = y1;
    last_x2 = x2;
    last_y2 = y2;

    if(dev->disp_drv_set_show_area) {
        dev->disp_drv_set_show_area(dev, x1, y1, x2, y2);
    }

    return;
}


void drv_adapter_disp_wait_to_flush(void) {
    disp_drv_t * dev = &_disp_drv_dev[_s_current_disp_dev_idx];

    if(dev->disp_drv_wait_to_flush) {
        dev->disp_drv_wait_to_flush(dev);
    }

    return;
}

void drv_adapter_disp_flush(void * buff, uint32_t buff_format, uint16_t w, uint16_t h) {
    disp_drv_t * dev = &_disp_drv_dev[_s_current_disp_dev_idx];

    if(dev->disp_drv_flush) {
        dev->disp_drv_flush(dev, buff, buff_format, w, h);
    }

    return;
}

void drv_adapter_disp_wait_te(void) {
    disp_drv_t * dev = &_disp_drv_dev[_s_current_disp_dev_idx];

    if(dev->disp_drv_wait_te) {
        dev->disp_drv_wait_te(dev);
    }
}

void drv_adapter_disp_on(bool is_on) {
    disp_drv_t * dev = &_disp_drv_dev[_s_current_disp_dev_idx];

    if(dev->disp_drv_on) {
        dev->disp_drv_on(dev, is_on);
    }
}

void drv_adapter_disp_set_brightness(uint32_t percent) {
    disp_drv_t * dev = &_disp_drv_dev[_s_current_disp_dev_idx];

    if(dev->disp_drv_set_brightness) {
        dev->disp_drv_set_brightness(dev, percent % 101);
    }
}

void drv_adapter_disp_sleep(void) {
    disp_drv_t * dev = &_disp_drv_dev[_s_current_disp_dev_idx];

    if(dev->disp_drv_sleep) {
        dev->disp_drv_sleep(dev);
    }
}

void drv_adapter_disp_wakeup(void) {
    disp_drv_t * dev = &_disp_drv_dev[_s_current_disp_dev_idx];

    if(dev->disp_drv_wakeup) {
        dev->disp_drv_wakeup(dev);
    }
}


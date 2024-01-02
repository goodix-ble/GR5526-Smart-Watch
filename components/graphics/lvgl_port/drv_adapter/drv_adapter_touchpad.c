#include "drv_adapter_touchpad.h"
#include "string.h"

static touchpad_drv_t _touchpad_drv_dev = {
    .id                             = 0,
    .touchpad_drv_init              = NULL,
    .touchpad_drv_deinit            = NULL,
    .touchpad_drv_read_pointer      = NULL,
    .touchpad_drv_sleep             = NULL,
    .touchpad_drv_wakeup            = NULL,
};

bool drv_adapter_touchpad_reg(touchpad_drv_t * dev)
{
    if(NULL == dev) {
        return false;
    }

    _touchpad_drv_dev.id                             = dev->id;
    _touchpad_drv_dev.touchpad_drv_init              = dev->touchpad_drv_init;
    _touchpad_drv_dev.touchpad_drv_deinit            = dev->touchpad_drv_deinit;
    _touchpad_drv_dev.touchpad_drv_read_pointer      = dev->touchpad_drv_read_pointer;
    _touchpad_drv_dev.touchpad_drv_sleep             = dev->touchpad_drv_sleep;
    _touchpad_drv_dev.touchpad_drv_wakeup            = dev->touchpad_drv_wakeup;

    return true;
}

bool drv_adapter_touchpad_init(void) {
    touchpad_drv_t * dev = &_touchpad_drv_dev;
    if(NULL != dev->touchpad_drv_init) {
        return dev->touchpad_drv_init(dev);
    }

    return false;
}

bool drv_adapter_touchpad_deinit(void) {
    touchpad_drv_t * dev = &_touchpad_drv_dev;
    if(NULL != dev->touchpad_drv_deinit) {
        return dev->touchpad_drv_deinit(dev);
    }

    return false;
}

bool drv_adapter_touchpad_read_pointer(int16_t * x, int16_t *y) {
    touchpad_drv_t * dev = &_touchpad_drv_dev;
    if(NULL != dev->touchpad_drv_read_pointer) {
        return dev->touchpad_drv_read_pointer(dev, x, y);
    }

    return false;
}

bool drv_adapter_touchpad_sleep(void) {
    touchpad_drv_t * dev = &_touchpad_drv_dev;
    if(NULL != dev->touchpad_drv_sleep) {
        return dev->touchpad_drv_sleep(dev);
    }

    return false;
}

bool drv_adapter_touchpad_wakeup(void) {
    touchpad_drv_t * dev = &_touchpad_drv_dev;
    if(NULL != dev->touchpad_drv_wakeup) {
        return dev->touchpad_drv_wakeup(dev);
    }

    return false;
}

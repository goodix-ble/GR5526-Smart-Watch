#include "drv_adapter_norflash.h"

static norflash_drv_t _norf_drv_dev[ADAPTER_NORFLASH_DEV_MAX];
static uint32_t       _s_cur_norf_dev_idx = 0;


bool drv_adapter_norflash_reg(uint32_t index, norflash_drv_t * dev)
{
    if(index >= ADAPTER_NORFLASH_DEV_MAX) {
        return false;
    }

    _norf_drv_dev[index].idx                            = index;
    _norf_drv_dev[index].user_data                      = dev->user_data;
    _norf_drv_dev[index].sector_buff                    = dev->sector_buff;
    _norf_drv_dev[index].sector_size                    = dev->sector_size;
    _norf_drv_dev[index].norflash_drv_init              = dev->norflash_drv_init;
    _norf_drv_dev[index].norflash_drv_deinit            = dev->norflash_drv_deinit;
    _norf_drv_dev[index].norflash_drv_read              = dev->norflash_drv_read;
    _norf_drv_dev[index].norflash_drv_write             = dev->norflash_drv_write;
    _norf_drv_dev[index].norflash_drv_update            = dev->norflash_drv_update;
    _norf_drv_dev[index].norflash_drv_erase             = dev->norflash_drv_erase;
    _norf_drv_dev[index].norflash_drv_set_mmap_mode     = dev->norflash_drv_set_mmap_mode;
    _norf_drv_dev[index].norflash_drv_sleep             = dev->norflash_drv_sleep;
    _norf_drv_dev[index].norflash_drv_wakeup            = dev->norflash_drv_wakeup;

    _s_cur_norf_dev_idx = index;

    return true;
}


bool drv_adapter_norflash_init(void) {
    norflash_drv_t * dev = &_norf_drv_dev[_s_cur_norf_dev_idx];

    if(_norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_init) {
        return _norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_init(dev);
    }

    return false;
}

bool drv_adapter_norflash_deinit(void) {
    norflash_drv_t * dev = &_norf_drv_dev[_s_cur_norf_dev_idx];

    if(_norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_deinit) {
        return _norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_deinit(dev);
    }

    return false;
}

bool drv_adapter_norflash_set_sector_buff(uint32_t index, void * sec_buff, uint32_t sec_size) {
    if(index >= ADAPTER_NORFLASH_DEV_MAX) {
        return false;
    }

    if((sec_buff == ((void*)0) ) || (sec_size == 0)) {
        return false;
    }

    norflash_drv_t * dev = &_norf_drv_dev[_s_cur_norf_dev_idx];

    dev->sector_buff = sec_buff;
    dev->sector_size = sec_size;

    return true;
}

bool drv_adapter_norflash_write(uint32_t dst, uint8_t * buff, uint32_t len) {
    norflash_drv_t * dev = &_norf_drv_dev[_s_cur_norf_dev_idx];

    if(_norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_write) {
        return _norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_write(dev, dst, buff, len);
    }

    return false;
}

bool drv_adapter_norflash_read(uint32_t addr, uint8_t * buff, uint32_t len) {
    norflash_drv_t * dev = &_norf_drv_dev[_s_cur_norf_dev_idx];

    if(_norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_read) {
        return _norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_read(dev, addr, buff, len);
    }

    return false;
}

bool drv_adapter_norflash_update(uint32_t addr, uint8_t * buff, uint32_t len) {
    norflash_drv_t * dev = &_norf_drv_dev[_s_cur_norf_dev_idx];

    if(_norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_update) {
        return _norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_update(dev, addr, buff, len);
    }

    return false;
}

bool drv_adapter_norflash_erase(uint32_t addr, uint32_t mode) {
    norflash_drv_t * dev = &_norf_drv_dev[_s_cur_norf_dev_idx];

    if(_norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_erase) {
        return _norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_erase(dev, addr, mode);
    }

    return false;
}

bool drv_adapter_norflash_set_mmap_mode(bool mmap) {
    norflash_drv_t * dev = &_norf_drv_dev[_s_cur_norf_dev_idx];

    if(_norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_set_mmap_mode) {
        return _norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_set_mmap_mode(dev, mmap);
    }

    return false;
}

bool drv_adapter_norflash_sleep(void) {
    norflash_drv_t * dev = &_norf_drv_dev[_s_cur_norf_dev_idx];

    if(_norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_sleep) {
        return _norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_sleep(dev);
    }

    return false;
}

bool drv_adapter_norflash_wakeup(void) {
    norflash_drv_t * dev = &_norf_drv_dev[_s_cur_norf_dev_idx];

    if(_norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_wakeup) {
        return _norf_drv_dev[_s_cur_norf_dev_idx].norflash_drv_wakeup(dev);
    }

    return false;
}

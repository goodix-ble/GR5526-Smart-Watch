#include "drv_adapter_port.h"
#include "qspi_norflash_v2.h"
#include "string.h"


/***********************************************************************************
 *                 Static Declarations For Nor-Flash
 ***********************************************************************************/
static bool _norflash_drv_init(norflash_drv_t * dev);
static bool _norflash_drv_deinit(norflash_drv_t * dev);
static bool _norflash_drv_write(norflash_drv_t * dev, uint32_t dst, uint8_t * buff, uint32_t len);
static bool _norflash_drv_read(norflash_drv_t * dev, uint32_t addr, uint8_t * buff, uint32_t len);
static bool _norflash_drv_update(norflash_drv_t * dev, uint32_t addr, uint8_t * buff, uint32_t len);
static bool _norflash_drv_erase(norflash_drv_t * dev, uint32_t addr, uint32_t mode);
static bool _norflash_drv_set_mmap_mode(norflash_drv_t * dev, bool mmap);
static bool _norflash_drv_sleep(norflash_drv_t * dev);
static bool _norflash_drv_wakeup(norflash_drv_t * dev);

typedef struct {
    app_qspi_id_t           qspi_id;
    uint16_t                clock_prescale;
    uint32_t                dev_id;
	uint32_t                density;
    bool                    is_mmap_mode;
    norf_rmode_e            read_mode;
    norf_wmode_e            write_mode;
    norf_data_xfer_width_e  data_xfer_width;
    app_qspi_pin_cfg_t *    pin_cfg;
} _norf_port_param_t;

static _norf_port_param_t  s_norf_port_param;
static const      uint32_t s_norf_sector_len = 4096;
static __align(4) uint8_t  s_norf_sector_buff[4096];


/***********************************************************************************
 *                 Public Implements
 ***********************************************************************************/

void drv_adapter_norflash_register(void) {

    s_norf_port_param.read_mode       = NORF_RMODE_2xIO_READ;
    s_norf_port_param.write_mode      = NORF_WMODE_PP;
    s_norf_port_param.data_xfer_width = NORF_XFER_WIDTH_BYTE;
    s_norf_port_param.is_mmap_mode    = false;

    norflash_drv_t _norf_dev = {
        .idx                        = 0,
        .user_data                  = (void*) &s_norf_port_param,
        .sector_buff                = (void*) &s_norf_sector_buff[0],
        .sector_size                = s_norf_sector_len,
        .norflash_drv_init          = _norflash_drv_init,
        .norflash_drv_deinit        = _norflash_drv_deinit,
        .norflash_drv_read          = _norflash_drv_read,
        .norflash_drv_write         = _norflash_drv_write,
        .norflash_drv_update        = _norflash_drv_update,
        .norflash_drv_erase         = _norflash_drv_erase,
        .norflash_drv_set_mmap_mode = _norflash_drv_set_mmap_mode,
        .norflash_drv_sleep         = _norflash_drv_sleep,
        .norflash_drv_wakeup        = _norflash_drv_wakeup,
    };

    drv_adapter_norflash_reg(0, &_norf_dev);

    return;
}


/***********************************************************************************
 *                 Static Implements for Nor-Flash
 ***********************************************************************************/

static bool _norflash_drv_init(norflash_drv_t * dev) {

    _norf_port_param_t * np = (_norf_port_param_t*) dev->user_data;

    np->dev_id = qspi_norf_init(NORFLASH_DEV_QSPI_ID, NORFLASH_DEV_CLOCK_PREESCALER , (app_qspi_pin_cfg_t *)&NORFLASH_DEV_PIN_CFG);
    printf("Flash Dev ID: 0x%02x\r\n", np->dev_id);

    if((np->dev_id == 0xFF) || (np->dev_id == 0x00)) {
        return false;
    }

    np->density = qspi_norf_read_dev_density();
    printf("Flash Dev Density: 0x%08x Bytes\r\n", np->dev_id);

    qspi_norf_set_mmap(true);

    np->qspi_id        = NORFLASH_DEV_QSPI_ID;
    np->clock_prescale = NORFLASH_DEV_CLOCK_PREESCALER;
    np->pin_cfg        = (app_qspi_pin_cfg_t *)&NORFLASH_DEV_PIN_CFG;
    np->is_mmap_mode   = true;

    return true;
}

static bool _norflash_drv_deinit(norflash_drv_t * dev)
{
    _norf_port_param_t * np = (_norf_port_param_t*) dev->user_data;

    np->dev_id          = 0x00;
    np->density         = 0x00;
    np->is_mmap_mode    = false;
    np->data_xfer_width = NORF_XFER_WIDTH_BYTE;
    np->read_mode       = NORF_RMODE_READ;
    np->write_mode      = NORF_WMODE_PP;

    return qspi_norf_deinit();

}

static bool _norflash_drv_write(norflash_drv_t * dev, uint32_t addr, uint8_t * buff, uint32_t len) {
    _norf_port_param_t * np = (_norf_port_param_t*) dev->user_data;
    uint32_t page_ofs       = 0;
    uint32_t write_size     = 0;
    uint32_t write_cnt      = len;
    bool     ret            = true;

    while(write_cnt)
    {
        page_ofs   = addr & (NORFLASH_DEV_DEFAULT_PAGE_SIZE - 1);
        write_size = NORFLASH_DEV_DEFAULT_PAGE_SIZE - page_ofs;

        if (write_cnt < write_size)
        {
            write_size = write_cnt;
            write_cnt  = 0;
        }
        else
        {
            write_cnt -= write_size;
        }

        ret = qspi_norf_dev_write(addr, buff, write_size, np->write_mode, np->data_xfer_width);

        addr += write_size;
        buff += write_size;

        if(!ret) {
            break;
        }
    }

    return ret;
}


static bool _norflash_drv_read(norflash_drv_t * dev, uint32_t addr, uint8_t * buff, uint32_t len) {
    _norf_port_param_t * np = (_norf_port_param_t*) dev->user_data;
    uint32_t max_xfer_bytes = 0;
    uint32_t xfer_len       = 0;
    bool     ret            = true;

    // Must <= 4095 Beat, choose 4000 here
    if(NORF_XFER_WIDTH_BYTE == np->data_xfer_width) {
        max_xfer_bytes = 4000;
    } else if(NORF_XFER_WIDTH_HALFWORD == np->data_xfer_width) {
        max_xfer_bytes = 8000;
    } else if(NORF_XFER_WIDTH_WORD == np->data_xfer_width) {
        max_xfer_bytes = 16000;
    } else {
        return false;
    }

    while(len) {
        xfer_len = max_xfer_bytes < len ? max_xfer_bytes : len;
        ret      = qspi_norf_dev_read(addr, buff, xfer_len, np->read_mode, np->data_xfer_width);
        len     -= xfer_len;
        addr    += xfer_len;
        buff    += xfer_len;

        if(!ret) {
            break;
        }
    }

    return ret;
}


static bool _norflash_drv_update(norflash_drv_t * dev, uint32_t addr, uint8_t * buffer, uint32_t len) {
    _norf_port_param_t * np = (_norf_port_param_t*) dev->user_data;
    bool     ret            = true;
    uint32_t pre_offset     = addr & (NORFLASH_DEV_DEFAULT_PAGE_SIZE - 1);
    uint32_t pre_len        = NORFLASH_DEV_DEFAULT_PAGE_SIZE - pre_offset;
    uint32_t pre_page_base  = (addr / NORFLASH_DEV_DEFAULT_PAGE_SIZE) * NORFLASH_DEV_DEFAULT_PAGE_SIZE;
    uint32_t page_cnt       = 0;
    uint32_t curr_addr      = 0;
    uint8_t * buff          = buffer;

    // 1. There is patial pre-page to write - Read Back Then Erase & Write.
    if(pre_offset > 0) {

        pre_len  = pre_len < len ? pre_len : len;

        memset(dev->sector_buff, 0, dev->sector_size);
        ret      = qspi_norf_dev_read(pre_page_base, dev->sector_buff, NORFLASH_DEV_DEFAULT_PAGE_SIZE, np->read_mode, np->data_xfer_width);

        if(!ret) {
            return false;
        }

        memcpy((void*)((uint32_t)dev->sector_buff + pre_offset), buff, pre_len);

        _norflash_drv_erase(dev, pre_page_base, ADAPTER_NORFFLASH_ERASE_PAGE);

        ret      = qspi_norf_dev_write(addr, dev->sector_buff, NORFLASH_DEV_DEFAULT_PAGE_SIZE, np->write_mode, np->data_xfer_width);
        if(!ret) {
            return false;
        }

        len  -= pre_len;
        buff += pre_len;
        page_cnt ++;
    }

    // 2. Whole Pages int the middle to write
    uint32_t page_left  = len / NORFLASH_DEV_DEFAULT_PAGE_SIZE;
    uint32_t bytes_left = len % NORFLASH_DEV_DEFAULT_PAGE_SIZE;

    while(page_left) {

        curr_addr  = pre_page_base + page_cnt*NORFLASH_DEV_DEFAULT_PAGE_SIZE;

        _norflash_drv_erase(dev, curr_addr, ADAPTER_NORFFLASH_ERASE_PAGE);

        ret = qspi_norf_dev_write(curr_addr, buff, NORFLASH_DEV_DEFAULT_PAGE_SIZE, np->write_mode, np->data_xfer_width);
        if(!ret) {
            return false;
        }

        buff += NORFLASH_DEV_DEFAULT_PAGE_SIZE;
        page_cnt ++;
        page_left --;
    }

    // 3. Left Bytes inthe tail to write
    if(bytes_left > 0) {

        memset(dev->sector_buff, 0, dev->sector_size);

        curr_addr = pre_page_base + page_cnt*NORFLASH_DEV_DEFAULT_PAGE_SIZE;
        ret       = qspi_norf_dev_read(curr_addr, dev->sector_buff, NORFLASH_DEV_DEFAULT_PAGE_SIZE, np->read_mode, np->data_xfer_width);

        if(!ret) {
            return false;
        }

        memcpy((void*)dev->sector_buff, buff, bytes_left);

        _norflash_drv_erase(dev, curr_addr, ADAPTER_NORFFLASH_ERASE_PAGE);

        ret      = qspi_norf_dev_write(curr_addr, dev->sector_buff, NORFLASH_DEV_DEFAULT_PAGE_SIZE, np->write_mode, np->data_xfer_width);
        if(!ret) {
            return false;
        }
    }

    return true;
}

static bool _norflash_drv_erase(norflash_drv_t * dev, uint32_t addr, uint32_t mode) {
    uint32_t adjust_addr = 0x00;
    norf_emode_e emode;
    switch(mode) {
        case ADAPTER_NORFFLASH_ERASE_PAGE:
        {
            adjust_addr = addr & 0xFFFFFF00;
            emode       = NORF_EMODE_PAGE;
        }
        break;

        case ADAPTER_NORFFLASH_ERASE_SECTOR:
        {
            adjust_addr = addr & 0xFFFFF000;
            emode       = NORF_EMODE_SECTOR;
        }
        break;

        case ADAPTER_NORFFLASH_ERASE_BLOCK:
        {
            adjust_addr = addr & 0xFFFF8000;
            emode       = NORF_EMODE_BLOCK_32K;
        }
        break;

        case ADAPTER_NORFFLASH_ERASE_CHIP:
        {
            adjust_addr = addr & 0xFFFF8000;
            emode       = NORF_EMODE_CHIP;
        }
        break;

        default:
        {
            return false;
        }
    }

    return qspi_norf_dev_erase(adjust_addr, emode);
}

static bool _norflash_drv_set_mmap_mode(norflash_drv_t * dev, bool mmap) {
    _norf_port_param_t * np = (_norf_port_param_t*) dev->user_data;

    if(mmap) {
        app_qspi_active_memory_mappped(np->qspi_id, true);
    } else {
        app_qspi_active_memory_mappped(np->qspi_id, false);
    }

    return true;
}

static bool _norflash_drv_sleep(norflash_drv_t * dev) {
    dev = dev;
    qspi_norf_set_sleep(true);
    return true;
}


static bool _norflash_drv_wakeup(norflash_drv_t * dev) {
    dev = dev;
    qspi_norf_set_sleep(false);
    return true;
}


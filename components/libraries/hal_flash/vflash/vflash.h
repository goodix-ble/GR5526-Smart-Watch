#ifndef __VFLASH_H__
#define __VFLASH_H__

#include <stdint.h>
#include <stdbool.h>

bool vflash_init(void);

uint32_t vflash_read(uint32_t addr, uint8_t *buf, uint32_t size);

uint32_t vflash_write(uint32_t addr, const uint8_t *buf, uint32_t size);

bool vflash_erase(uint32_t addr, uint32_t size);

uint32_t vflash_sector_size(void);

#endif // __VFLASH_H__


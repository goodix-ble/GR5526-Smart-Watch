#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define VFLASH_SECTOR_SIZE 0x1000
#define VFLASH_SIZE (VFLASH_SECTOR_SIZE * NVDS_NUM_SECTOR)

extern uint32_t nvds_get_start_addr(void);

/* *****************************************************************************
 * Local variables
 */
static uint8_t vflash_ram[VFLASH_SIZE];

/* *****************************************************************************
 * Local functions
 */
static uint32_t addr_translate(uint32_t addr);

/* *****************************************************************************
 * Function definitions
 */
bool vflash_init(void)
{
    memset(vflash_ram, 0xFF, VFLASH_SIZE);

    return true;
}

uint32_t vflash_sector_size(void)
{
    return VFLASH_SECTOR_SIZE;
}

uint32_t vflash_read(uint32_t addr, uint8_t *buf, uint32_t size)
{
    uint32_t v_addr = addr_translate(addr);

    memcpy(buf, (uint8_t *)v_addr, size);

    return size;
}

uint32_t vflash_write(uint32_t addr, const uint8_t *buf, uint32_t size)
{
    uint32_t v_addr = addr_translate(addr);

    memcpy((uint8_t *)v_addr, buf, size);

    return size;
}

bool vflash_erase(uint32_t addr, uint32_t size)
{
    uint32_t v_addr = addr_translate(addr);

    memset((uint8_t *)v_addr, 0xFF, size);

    return true;
}

uint32_t addr_translate(uint32_t addr)
{
    return ((uint32_t)(addr - nvds_get_start_addr() + vflash_ram));
}


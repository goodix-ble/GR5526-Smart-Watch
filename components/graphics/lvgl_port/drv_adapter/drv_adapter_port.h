#ifndef __DRV_ADAPTER_PORT_H__
#define __DRV_ADAPTER_PORT_H__

#include "drv_adapter_display.h"
#include "drv_adapter_norflash.h"
#include "drv_adapter_touchpad.h"

#define NORFLASH_DEV_DEFAULT_PAGE_SIZE                  0x100       /**< default page size: 256 bytes */
#define NORFLASH_DEV_DEFAULT_SECTOR_SIZE                0x1000      /**< default page size: 4096 bytes */

#define NORFLASH_DEV_QSPI_ID                            APP_QSPI_ID_0                           /**< QSPI id to connect the norf */
#define NORFLASH_DEV_CLOCK_PREESCALER                   2u                                      /**< clock prescaler for qspi */
#define NORFLASH_DEV_PIN_CFG                            (g_qspi_pin_groups[QSPI0_PIN_GROUP_0])  /**< pin config for qspi */
#define NORFLASH_DEV_DEV_ID                             0x0B                                    /**< 0x0B - XTX; 0x85 - PUYA */

void drv_adapter_disp_register(void);

void drv_adapter_norflash_register(void);

void drv_adapter_touchpad_register(void);

#endif /* __DRV_ADAPTER_PORT_H__ */

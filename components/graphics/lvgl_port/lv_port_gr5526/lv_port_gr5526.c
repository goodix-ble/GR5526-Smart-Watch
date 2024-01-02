
#include "app_qspi.h"
#include "app_graphics_ospi.h"
#include "app_graphics_mem.h"
#include "app_graphics_gpu.h"
#include "drv_adapter_port.h"
#include "platform_sdk.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lv_port_gr5526.h"


void lvgl_port_init(void)
{
    /* hardware resource init. */
    /* PSRAM */
    app_graphics_ospi_params_t params = PSRAM_INIT_PARAMS_Default;
    app_graphics_ospi_init(&params);

    mem_pwr_mgmt_mode_set(MEM_POWER_FULL_MODE);
    app_graphics_mem_init((void*)GFX_MEM_BASE, GFX_MEM_SIZE);

    /* GPU */
    graphics_gpu_init(NULL);

    /* Nor flash */
    drv_adapter_norflash_init();

    /* lvgl port */
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();
}

void lv_port_res_mode_set(uint8_t mode) {
    if(mode == 0){
        app_qspi_mmap_set_endian_mode(NORFLASH_DEV_QSPI_ID, APP_QSPI_MMAP_ENDIAN_MODE_0);
    }else if(mode == 1){
        app_qspi_mmap_set_endian_mode(NORFLASH_DEV_QSPI_ID, APP_QSPI_MMAP_ENDIAN_MODE_2);
    }else if(mode == 2){
        app_qspi_mmap_set_endian_mode(NORFLASH_DEV_QSPI_ID, APP_QSPI_MMAP_ENDIAN_MODE_2);
    }
}

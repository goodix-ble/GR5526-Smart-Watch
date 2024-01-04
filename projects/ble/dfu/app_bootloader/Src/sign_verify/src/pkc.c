#include "grx_sys.h"
#include "bootloader_config.h"

#ifndef SOC_GR5332
#if (BOOTLOADER_SIGN_ENABLE && !BOOTLOADER_BOOT_PORT_ENABLE)

#include "drv_common.h"
#include "drv_pkc.h"

#include "pkc.h"
#include "dfu_port.h"

#define REGS_PCK_BASE_ADDR                   (CHIP_REGS_BASE_ADDR_SEC + 0x4000)
static gm_pkc_v1_config_t pkc_v1_config =
{
    .reg_base = (void*)REGS_PCK_BASE_ADDR,
};

static gm_pkc_v1_data_t pkc_v1_data =
{
    .interrupt_enable = 0,
    .interrupt_flag = 0,
    .pkc_done_flag = 0,
    .pkc_error_flag = 0,
    .pkc_overflow_flag = 0,
    .pkc_busy = 0,
    .current_status = 0,
    .point_multiplication_result = NULL,
    .count_for_rng = 0,
    .random_history = 0,
    .gen_count = 0,
};

static device_config_t pkc_device_config =
{
    .name = "pkc",
    .config_info = &pkc_v1_config

};

static device_t pkc_dev =
{
    .driver_data = &pkc_v1_data,
    .config = &pkc_device_config
};


#define PKC0 0

static  device_t *gm_device_get_instance(uint32_t pkc0)
{
    pkc0 = pkc0;
    return &pkc_dev;
}

int bl_pkc_modular_exponet_65537(uint32_t word_bit_len, uint32_t in_a[],uint32_t in_prime[], uint32_t r_square[], uint32_t constq, uint32_t out_result[])
{
    gm_drv_ret_e err = GM_DRV_OK;
    device_t* ecc = gm_device_get_instance(PKC0);
    err = drv_modular_exponet_65537(ecc, word_bit_len, in_a, in_prime, r_square, constq,out_result);
    return (int)err;
}

#endif

#endif

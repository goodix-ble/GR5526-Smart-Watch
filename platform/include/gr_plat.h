#ifndef GR_PLAT_H
#define GR_PLAT_H

#include <stdint.h>

#define MAX_COMMENTS_CNT        12
#define MAX_RESERVED_1_CNT      6
typedef struct __attribute((packed))
{
    uint32_t    app_pattern;
    uint32_t    app_info_version;
    uint32_t    chip_ver;
    uint32_t    load_addr;
    uint32_t    run_addr;
    uint32_t    app_info_sum;
    uint8_t     check_img;
    uint8_t     boot_delay;
    uint8_t     sec_cfg;
    uint8_t     reserved0;
    uint8_t     comments[MAX_COMMENTS_CNT];
    uint32_t    reserved1[MAX_RESERVED_1_CNT];
} APP_INFO_t;

#define APP_INFO_ADDR           (APP_CODE_RUN_ADDR+0x200)
#define APP_INFO_PATTERN_VALUE  0x47525858
#define APP_INFO_VERSION        0x1
#define CHECK_SUM               (APP_INFO_PATTERN_VALUE+APP_INFO_VERSION+CHIP_VER+APP_CODE_LOAD_ADDR+APP_CODE_RUN_ADDR)

extern void __main(void);

extern void sdk_init(void);

#endif

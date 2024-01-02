#ifndef __GR55XX_EFUSE_LAYOUT_H__
#define __GR55XX_EFUSE_LAYOUT_H__

#include<stdio.h>
#include<stdint.h>

#define EFUSE_BT_ADDR_SIZE        (6)
#define EFUSE_CHIP_UID_SIZE       (16)
#define EFUSE_CHIP_ID_SIZE        (6)
#define EFUSE_PRODUCT_ID_SIZE     (2)
#define FW_PUBLIC_KEY_HASH_SIZE   (16)
#define ROOT_PUBLIC_KEY_HASH_SIZE (16)

#define EFUSE_TRIM_PATTERN        (0x4744)

#define EFUSE_IO_LDO_SEL_MASK     (0x01)
#define EFUSE_IO_LDO_BYPASS_MASK  (0x02)

/* Make sure it is same with the one in \boot_security\BL0\inc\efuse.h. */
typedef struct _usb_trim_t
{
    uint16_t usb_rg_xcvr_rtrimp : 4;
    uint16_t usb_rg_xcvr_rtrimn: 4;
    uint16_t usb_rg_ldo33_vesl : 4;
    uint16_t usb_rh_ldo33_bias: 4;
} __attribute__ ((packed)) usb_trim_t;

typedef struct _efuse_trim0_t
{
    uint8_t    chip_uid[EFUSE_CHIP_UID_SIZE];
    uint8_t    ate_version;
    uint8_t    reserved0[2];
    uint8_t    io_ldo_sel;
    uint8_t    bt_addr[EFUSE_BT_ADDR_SIZE];
    uint16_t   xo_offset;
    uint16_t   pattern;
    uint16_t   trim_sum;
    uint16_t   hw_version;
    uint16_t   chip_id;
    uint16_t   package;
    uint16_t   flash_size;
    uint16_t   ram_size;
    uint8_t    reserved1[2];
    uint8_t    dcdc_vout1p15;
    uint8_t    dcdc_vout1p05;
    uint8_t    dig_ldo_0p9;
    uint8_t    dig_ldo_1p05;
    uint8_t    io_ldo_1p8;
    uint8_t    io_ldo_3p0;
    uint8_t    reserved2[4];
    uint8_t    tx_power;
    uint8_t    rssi_cali;
    uint8_t    lp_gain_offset_2m;
    uint8_t    reserved3;
    usb_trim_t usb_trim;
} __attribute__ ((packed)) efuse_trim0_t;


typedef struct _efuse_exflash_timing_t
{
    uint8_t flash_tVSL;      /* tVSL: VCC(min.) to device operation. min 10us, unit: 10us */
    uint8_t flash_tESL;      /* tESL:  Erase suspend latency. max 30us, unit: 5us */
    uint8_t flash_tPSL;      /* tPSL:  Program suspend latency. max 30us, unit: 5us */
    uint8_t flash_tPRS;      /* tPRS: Latency between program resume and next suspend. max 30us, unit: 5us */
    uint8_t flash_tERS;      /* tERS: Latency between erase resume and next suspend. max 30us, unit: 5us */
    uint8_t flash_tDP;       /* tDP:  CS# High to Deep Power-down Mode. unit: 5us */
    uint8_t flash_tRES2;     /* tRES2:  CS# High To Standby Mode With Electronic Signature Read max 8us, unit: 5us */
    uint8_t flash_tRDINT;    /* tRDINT: Read status register interval after write opertion. Uint: 5us */
} __attribute__ ((packed)) efuse_exflash_timing_t;


typedef struct _efuse_sadc_trim_t
{
    uint16_t offset_int_0p8;
    uint16_t slope_int_0p8;
    uint16_t offset_int_1p2;
    uint16_t slope_int_1p2;
    uint16_t offset_int_1p6;
    uint16_t slope_int_1p6;
    uint16_t offset_ext_1p0;
    uint16_t slope_ext_1p0;
    uint16_t temp;
} __attribute__ ((packed)) efuse_sadc_trim_t;

typedef struct _efuse_comp_trim_t
{
    uint16_t slope_int_no1;
    uint16_t slope_int_no2;
} __attribute__ ((packed)) efuse_comp_trim_t;

typedef struct _efuse_trim1_t
{
    efuse_sadc_trim_t      sadc_trim;
    efuse_exflash_timing_t flash_timing;
    efuse_comp_trim_t      comp_trim;
    uint16_t               temp_ref;
    uint8_t                reserved[28];
} __attribute__ ((packed)) efuse_trim1_t;


typedef struct {
    /* Configurations 4Byte*/
    uint32_t isp_uart_bypass : 1;        // 1: Not Support UART ISP Process, 0: Support UART ISP Process
    uint32_t isp_usb_bypass : 1;         // 1: Not Support USB ISP Process, 0: Support USB ISP Process
    uint32_t isp_jlink_bypass : 1;       // 1: Not Support JLINK ISP Process, 0: Support JLINK ISP Process
    uint32_t enc_boot_system_clk : 3;    // 0: XO-16M, 1: PLL-64M, 2: PLL-96M, 3: PLL-48M, 4: PLL-24M, 5: PLL-16M, 6: PLL-32M
    uint32_t enc_boot_flash_clk : 3;     // 0: 16MHz, 1: 48MHz, 2: 32MHz, 3: 24MHz, 4:64MHz
    uint32_t enc_boot_xip_read_cmd : 8;  // 0x03, 0x0B, 0x3B, 0xBB, 0x6B, 0xEB
    uint32_t memory_power_size : 1;      // 0: 512KB, 1: 288KB
    uint32_t reserved : 14;
} __attribute__ ((packed)) config_t;

typedef struct {
    /* Configurations 4Byte*/
    config_t config;

    uint16_t swd_disable;
    uint16_t enc_mode;
    uint32_t crc32;

    uint8_t  chip_id[EFUSE_CHIP_ID_SIZE];
    uint8_t  product_id[EFUSE_PRODUCT_ID_SIZE];
    uint8_t  fw_public_key_hash[FW_PUBLIC_KEY_HASH_SIZE];
    uint8_t  root_public_key_hash[ROOT_PUBLIC_KEY_HASH_SIZE];
} __attribute__ ((packed)) bl_efuse_info_t;

#endif //__GR55XX_EFUSE_LAYOUT_H__

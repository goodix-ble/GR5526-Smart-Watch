/*
* Copyright (C) 2017, Shenzhen Goodix Technology Co., Ltd.
* All Rights Reserved.
*/

#ifndef __DRV_PKC_H__
#define __DRV_PKC_H__

#include <drv_common.h>

/**
 * \brief 256-bit ecc is composed of an array of 8 uint32_t
 */
#define ECC_U32_LENGTH                    (8)

/**
 * \brief Defines a data structure of a point, include x-axis and y axis. And (0,0) is defined as the point of infinite.
 */
typedef struct _gm_ecc_point
{
    uint32_t x[ECC_U32_LENGTH];
    uint32_t y[ECC_U32_LENGTH];
} gm_ecc_point_t;

typedef void (* gm_ecc_hardware_finish_callback_t)(uint32_t error);

typedef struct
{
    void*  reg_base;
    uint32_t irq_num;
    uint32_t clk_type;
    gm_ecc_hardware_finish_callback_t callback;
} gm_pkc_v1_config_t;

typedef struct _gm_pkc_v1_data
{
    volatile uint8_t interrupt_enable;
    volatile uint8_t interrupt_flag;
    volatile uint8_t pkc_done_flag;
    volatile uint8_t pkc_error_flag;
    volatile uint8_t pkc_overflow_flag;
    volatile uint8_t pkc_busy;
    volatile uint8_t current_status;
    volatile gm_ecc_point_t* point_multiplication_result;

    volatile uint32_t count_for_rng;
    volatile uint32_t random_history;
    volatile uint32_t gen_count;

} gm_pkc_v1_data_t;

typedef struct
{
    char        *name;
    const void  *config_info;
} device_config_t;

typedef struct device
{
    volatile device_config_t    *config;
    const void                  *driver_api;
    void                        *driver_data;
    uint32_t                    reference_num;
    uint32_t                    wakeup_src;
    uint8_t                     wakeup_enable;
} device_t;
gm_drv_ret_e drv_modular_exponet_65537(device_t * dev, uint32_t word_bit_len, uint32_t in_a[],uint32_t in_prime[],uint32_t r_square[],uint32_t constp, uint32_t out_result[]);
#endif /*__DRV_PKC_H__*/


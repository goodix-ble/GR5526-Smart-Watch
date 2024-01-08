


/*
* Copyright (C) 2017, Shenzhen Goodix Technology Co., Ltd.
* All Rights Reserved.
*/

#ifndef __PKC_H__
#define __PKC_H__

#include "grx_sys.h"

int bl_pkc_modular_exponet_65537(uint32_t word_bit_len, uint32_t in_a[],uint32_t in_prime[], uint32_t r_square[], uint32_t constq, uint32_t out_result[]);

#endif // __PKC_H__


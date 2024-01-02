/**
 ****************************************************************************************
 *
 * @file    crypto_pkc_port.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of crypto PKC library.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2022 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************************
 */

#ifndef __CRYPTO_PKC_PORT_H__
#define __CRYPTO_PKC_PORT_H__

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t (*pkc_rng_func_ptr)(void);

void pkc_setbit(uint32_t q[], uint32_t in_prime[], uint32_t k, uint32_t bits_len);
void pkc_zeroize(void *start_address, uint32_t u32_len);
int32_t pkc_number_compare_to_const(uint32_t a[], uint32_t b, uint32_t bits_len);
uint8_t pkc_safe_compare(pkc_rng_func_ptr rng32, uint32_t *src, uint32_t *dest, uint32_t u32_len);
int32_t pkc_number_compare(uint32_t a[], uint32_t b[], uint32_t bits_len);
void pkc_read_oct_string(uint32_t *big_number, uint8_t* buffer, uint32_t byte_size);

#ifdef __cplusplus
}
#endif

#endif /* __CRYPTO_PKC_PORT_H__ */

/**
 ****************************************************************************************
 *
 * @file    crypto_ecc_port.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of crypto ECC library.
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

#ifndef __CRYPTO_ECC_PORT_H__
#define __CRYPTO_ECC_PORT_H__

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "crypto_ecc.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t hw_ecc_rng32(void);
void hw_ecc_point_mul(algo_ecc_config_t *ecc_calc_options,
                      uint32_t k[ECC_U32_LENGTH],
                      algo_ecc_point_t *Q,
                      algo_ecc_point_t *result);
void hw_ecc_sha(const uint8_t *message, uint32_t message_byte_length, uint8_t output[32]);
void hw_ecc_modular_compare(algo_ecc_config_t *ecc_calc_options,
                            uint32_t in_a[],
                            uint32_t in_prime[],
                            uint32_t result[]);
void ecc_modular_inverse(algo_ecc_config_t *ecc_config,
                         uint32_t in_a[],
                         uint32_t in_prime[],
                         uint32_t r_square[],
                         uint32_t constq,
                         uint32_t out_a_inverse[]);
void hw_ecc_montgomery_inverse(
    algo_ecc_config_t *ecc_calc_options, uint32_t in_a[], uint32_t in_prime[], uint32_t constp, uint32_t out_x[]);
void hw_ecc_modular_sub(
    algo_ecc_config_t *ecc_calc_options, uint32_t in_a[], uint32_t in_b[], uint32_t in_prime[], uint32_t result[]);
void hw_ecc_montgomery_mul(algo_ecc_config_t *ecc_calc_options,
                           uint32_t in_a[],
                           uint32_t in_b[],
                           uint32_t in_prime[],
                           uint32_t constp,
                           uint32_t result[]);
void hw_ecc_modular_add(
    algo_ecc_config_t *ecc_calc_options, uint32_t in_a[], uint32_t in_b[], uint32_t in_prime[], uint32_t result[]);
void ecc_modular_multiply(algo_ecc_config_t *ecc_config,
                          uint32_t in_a[],
                          uint32_t in_b[],
                          uint32_t in_prime[],
                          uint32_t r_square[],
                          uint32_t constq,
                          uint32_t out_result[]);
uint32_t ecc_is_infinite_point(algo_ecc_point_t *point);

#ifdef __cplusplus
}
#endif

#endif /* __CRYPTO_ECC_PORT_H__ */

/**
 ****************************************************************************************
 *
 * @file    crypto_rsa_port.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of crypto RSA library.
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

#ifndef __CRYPTO_RSA_PORT_H__
#define __CRYPTO_RSA_PORT_H__

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "crypto_rsa.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t hw_rsa_rng32(void);
void hw_rsa_sha(const uint8_t *message, uint32_t message_byte_length, uint8_t output[32]);
void hw_rsa_modular_left_shift(algo_rsa_config_t *rsa_calc_options,
                               uint32_t bits_len,
                               uint32_t in_a[],
                               uint32_t in_prime[],
                               uint32_t shift_bits,
                               uint32_t result[]);
void hw_rsa_modular_compare(algo_rsa_config_t *rsa_calc_options,
                            uint32_t bits_len,
                            uint32_t in_a[],
                            uint32_t in_prime[],
                            uint32_t result[]);
void rsa_modular_inverse(algo_rsa_config_t *rsa_config,
                         uint32_t bits_len,
                         uint32_t in_a[],
                         uint32_t in_prime[],
                         uint32_t r_square[],
                         uint32_t constq,
                         uint32_t out_a_inverse[]);
void hw_rsa_montgomery_inverse(algo_rsa_config_t *rsa_calc_options,
                               uint32_t bits_len,
                               uint32_t in_a[],
                               uint32_t in_prime[],
                               uint32_t constp,
                               uint32_t out_x[]);
void hw_rsa_modular_exponent(algo_rsa_config_t *rsa_calc_options,
                             uint32_t bits_len,
                             uint32_t in_a[],
                             uint32_t in_b[],
                             uint32_t in_prime[],
                             uint32_t r_square[],
                             uint32_t constq,
                             uint32_t result[]);
void hw_rsa_montgomery_mul(algo_rsa_config_t *rsa_calc_options,
                           uint32_t bis_len,
                           uint32_t in_a[],
                           uint32_t in_b[],
                           uint32_t in_prime[],
                           uint32_t constp,
                           uint32_t result[]);
int rsa_safer_memcmp( const void *a, const void *b, size_t n );

/**
 *  \brief compute modular exponent for RSA:  out_result = (in_a ^in_b) mod in_prime
 *
 *  \param[in] rsa_config rsa_config \ref algo_rsa_config_t
 *
 *  \param[in] bits_len bit width of modular number of in_prime, supports up to 2048 bits
 *
 *  \param[in] in_a the base number of bits_len bits
 *
 *  \param[in] in_b the exponent number of bits_len bits;
 *
 *  \param[in] in_prime the modular number
 *
 *  \param[in] r_square R^2 mod in_prime, where R = 2 ^ bits_len
 *
 *  \param[in] constp montgomery multiplication constant of in_prime
 *
 *  \param[out] out_result in_a ^(in_b) mod in_prime
 *
 *  \return
 *      \li \ref RSA_ERROR_PARAMETER : NULL input pointer
 *      \li \ref RSA_OK
 */
void rsa_modular_exponent(algo_rsa_config_t *rsa_config,
                          uint32_t bits_len,
                          uint32_t in_a[],
                          uint32_t in_b[],
                          uint32_t in_prime[],
                          uint32_t r_square[],
                          uint32_t constp,
                          uint32_t out_result[]);

#ifdef __cplusplus
}
#endif

#endif /* __CRYPTO_RSA_PORT_H__ */

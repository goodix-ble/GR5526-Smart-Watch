/**
 ****************************************************************************************
 *
 * @file    crypto_pkc.h
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

#ifndef __CRYPTO_PKC_H__
#define __CRYPTO_PKC_H__

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "grx_hal.h"
#include "crypto_ecc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief modular shift operation: result(bits_len bit) = (2 ^ shift_bits) * in_a (bits_len bit) mod in_prime (bits_len bit)
 *
 * \param[in] bits_len number bit-length, support 256, 288, 320, ... up to 2048 bits.(32 bits per step)
 *
 * \param[in] in_a first input operand (bits_len bit).
 *
 * \param[in] in_prime input prime (bits_len bit).
 *
 * \param[in] shift_bits number of shift bits.
 *
 * \param[out] result bits_len bit output.
 *
 * \return  operation status
 *  \li \ref HAL_ERROR
 *  \li \ref HAL_OK
 */
hal_status_t hal_pkc_modular_left_shift_handle(uint32_t bits_len, uint32_t in_a[], uint32_t in_prime[],
            uint32_t shift_bits, uint32_t result[]);

/**
 * \brief modular compare operation: result(bits_len bit) = in_a(bits_len bit) mod in_prime(bits_len bit)
 *
 * \param[in] bits_len number bit-length, support 256, 288, 320, ... up to 2048 bits.(32 bits per step)
 *
 * \param[in] in_a first input operand (bits_len bit).
 *
 * \param[in] in_prime input prime (bits_len bit).
 *
 * \param[out] result bits_len bit output.
 *
 * \return  operation status
 *  \li \ref HAL_ERROR
 *  \li \ref HAL_OK
 */
hal_status_t hal_pkc_modular_compare_handle(uint32_t bits_len, uint32_t in_a[],uint32_t in_prime[],
            uint32_t result[]);

/**
 * \brief Montgomery inverse operation: (out_x,outk) = in_a ^(-1) * 2 ^ (out_k) mod in_prime, where bits_len <= out_k <= 2*bits_len
 *
 * \param[in] bits_len number bit-length, support 256, 288, 320, ... up to 2048 bits.(32 bits per step)
 *
 * \param[in] in_a first input operand (bits_len bit).
 *
 * \param[in] in_prime input prime (bits_len bit).
 *
 * \param[out] out_x bits_len bit output.
 *
 * \param[out] out_k output k (13 bits, denoted by a uint32_t variable).
 *
 * \return
 *  \li \ref HAL_ERROR
 *  \li \ref HAL_OK
 */
hal_status_t hal_pkc_montgomery_inverse(uint32_t bits_len, uint32_t in_a[], uint32_t in_prime[], uint32_t constp, uint32_t out_x[]);

/**
 * \brief compute modular exponent for RSA:  result = in_a ^(in_b) mod in_prime
 *
 * \param[in] bits_len RSA number bit width, hardware supports up to 2048 bits
 *
 * \param[in] in_a the base number of bits_len bits
 *
 * \param[in] in_b the exponet number of bits_len bits
 *
 * \param[in] in_prime the modular number
 *
 * \param[in] r_square R^2 mod in_prime, where R = 2 ^ bits_len. If this field is NULL, the program will interanly
 *            compute R^2 mod p and constq, at a cost of performance degradation.
 *
 * \param[in] constq Montgomery multiplication constant of in_prime
 *
 * \param[out] result in_a ^(in_b) mod in_prime
 *
 * \return  operation status
 *  \li \ref HAL_ERROR
 *  \li \ref HAL_OK
 */
hal_status_t hal_pkc_rsa_modular_exponent_handle(uint32_t bits_len, uint32_t in_a[], uint32_t in_b[],
          uint32_t in_prime[], uint32_t r_square[], uint32_t constq,uint32_t result[]);

/**
 * \brief Montgomery multiply operation: result = in_a * in_b *R^(-1) mod in_prime, where R = 2^bits_len
 *
 * \param[in] bits_len number bit-length, support 256, 288, 320, ... up to 2048 bits.(32 bits per step)
 *
 * \param[in] in_a first input operand (bits_len bit).
 *
 * \param[in] in_b second input operand (bits_lenbit).
 *
 * \param[in] in_prime input prime (bits_len bit).
 *
 * \param[in] constq Montgomery multiplication constant for in_prime, where constq = (-in_prime[0]) ^(-1) mod 2^32
 *
 * \param[out] result bits_len bit output.
 *
 * \return  operation status
 *  \li \ref HAL_ERROR
 *  \li \ref HAL_OK
 */
hal_status_t hal_pkc_montgomery_mul(uint32_t bits_len, uint32_t in_a[], uint32_t in_b[], uint32_t in_prime[],
           uint32_t constq, uint32_t result[]);

/**
 * \brief set ECC Curve type
 *
 * \param[in] ECC Curve type.
 *
 */
ecc_curve_init_t *pkc_set_curve_p256_params(algo_ecc_curve_type_e curve);

/**
 * \brief Set user defined curve
 *
 * \param[in] curve ECC curve parameters, for NIST-P256 curve, invokers may fill this parameter with NULL
 *                  Note that hal_pkc_init inits NIST-P256 curve, invokers may ignore this API if only use
 *                  NIST-P256 curve.
 *                  For other curves, invokers must fill the curve parameters \ref ecc_curve_parameter_t
 *
 */
void hal_set_curve(ecc_curve_init_t * curve);

/**
 * \brief ECC point multiplication operation based on NIST P256 (also known as SECP256R1) curve.
 * If secure mode is set(by hal_pkc_init), anti-DPA security measures will be employed,
 * at a cost of performance loss (about 25%-35%)
 *
 * \param[in] k input 256 bit number.
 *
 * \param[in] Q input ecc point (x:256 bit, y:256 bit). If #Q is null, then Q point is equal G point
 *
 * \param[out] result output ecc point (x:256 bit, y:256 bit) -- result = k * point
 *
 * \return  operation status
 *  \li \ref HAL_ERROR
 *  \li \ref HAL_OK
 */
hal_status_t hal_pkc_ecc_point_mul_handle(uint32_t k[ECC_U32_LENGTH], ecc_point_t *Q, ecc_point_t *result);

/**
 * \brief modular sub operation: result(256 bit) = in_a(256 bit) - in_b(256 bit) mod in_prime(256 bit)
 *
 * \param[in] bits_len number bit-length, support 256, 288, 320, ... up to 2048 bits.(32 bits per step)
 *
 * \param[in] in_a first input operand (bits_len bit).
 *
 * \param[in] in_b second input operand (bits_len bit).
 *
 * \param[in] in_prime input prime (bits_len bit).
 *
 * \param[out] result bits_len bit output.
 *
 * \return  operation status
 *  \li \ref HAL_ERROR
 *  \li \ref HAL_OK
 */
hal_status_t hal_pkc_modular_sub_handle(uint32_t bits_len, uint32_t in_a[], uint32_t in_b[], uint32_t in_prime[],
           uint32_t result[]);

/**
 * \brief modular add operation: result(256 bit) = in_a(256 bit) + in_b(256 bit) mod in_prime(256 bit)
 *
 * \param[in] bits_len number bit-length, support 256, 288, 320, ... up to 2048 bits.(32 bits per step)
 *
 * \param[in] in_a first input operand (bits_len bit).
 *
 * \param[in] in_b second input operand (bits_len bit).
 *
 * \param[in] in_prime input prime (bits_len bit).
 *
 * \param[out] result bits_len bit output.
 *
 * \return  operation status
 *  \li \ref HAL_ERROR
 *  \li \ref HAL_OK
 */
hal_status_t hal_pkc_modular_add_handle(uint32_t bits_len, uint32_t in_a[], uint32_t in_b[], uint32_t in_prime[],
            uint32_t result[]);

#ifdef __cplusplus
}
#endif

#endif /* __CRYPTO_PKC_H__ */

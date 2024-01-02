/**
 ****************************************************************************************
 *
 * @file    crypto_rsa.h
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

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup CRYPTO_DRIVER CRYPTO DRIVER
 *  @{
 */

/** @defgroup CRYPTO_RSA RSA
  * @brief RSA CRYPTO driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRYPTO_RSA_H__
#define __CRYPTO_RSA_H__

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/** @defgroup CRYPTO_RSA_MACRO Defines
  * @{
  */

//#define RSA_OPENSSL_SEQ     /**< Open with Openssl sequence.*/
#define RSA_PKCS1_V21 (1)   /**< PKCS1_PSS sign, as defined in PKCS#1_2.1/2.2: same message and private key will produce different result, since random salt is different each time.*/
#define RSA_PKCS1_V15 (0)   /**< PKCS1_V15 sign, same message and private key will produce same result.*/
#define RSA_U32_LENGTH (64) /**<Max supported RSA algorithm is RSA 2048, which is composed of an array of 64 uint32_t.*/
/** @} */

/** @addtogroup CRYPTO_RSA_ENUM Enumerations
  * @{
  */

/**
 * @brief This defines the process state of rsa e type value.
 */
typedef enum _algo_rsa_public_expoent
{

    RSA_E_65537 = 65537, /**< for e = 65537; recomended. */
    RSA_E_17 = 17,       /**< for e = 17. */
    RSA_E_3 = 3,         /**< for e = 3. */

} algo_rsa_public_exponent_e;

/**
 * @brief RSA Return Code.
 */
typedef enum _algo_rsa_ret {

    RSA_OK = 0,                          /**< RSA Return OK. */
    RSA_ERROR_PARAMETER = -100001,       /**< RSA Return PARAMETER ERROR. */
    RSA_ERROR_SIGN_FUNCTION = -100002,   /**< RSA Return SIGN ERROR. */
    RSA_ERROR_VERIFY_FUNCTION = -100003, /**< RSA Return VERIFY ERROR. */
    RSA_ERROR_RANDOM = -100004,          /**< RSA Return Random ERROR. */
    RSA_ERROR_INTERFACE_EMPTY = -100005, /**< RSA Return Interface Empty. */
} algo_rsa_ret_e;
/** @} */

/** @addtogroup CRYPTO_RSA_CONTEXT_STRUCTURES Structures
 * @{
 */

/**
 * @brief  RSA Computation config
 */
typedef struct _algo_rsa_config {
    uint8_t sign_pkcs_type;
    /**< RSA operation type, must be one of the following value
     * \li \ref RSA_PKCS1_V21
     * \li \ref RSA_PKCS1_V15
     */
} algo_rsa_config_t;
/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CRYPTO_RSA_FUNCTIONS Functions
  * @{
  */

/**
 *******************************************************************************************
 *  @brief sign hash according to PKCS#1 standard
 *  Sign the message_hash by private_key {private_key, e, in_prime, r_square, constp}, output to sig.
 *
 *  @param[in] rsa_config: rsa_config \ref algo_rsa_config_t
 *
 *  @param[in] bits_len: bit width of modular number of in_prime, supports up to 2048 bits
 *
 *  @param[in] message:  input message.
 *  @param[in] message_len:  input message len.
 *  @param[in] private_key:  the private key length is bits_len
 *
 *  @param[in] e: the exponent number may be 3, 17 or 65537 (recommended 65537), \ref algo_rsa_public_exponent_e
 *
 *  @param[in] in_prime: the modular number, which must be odd number
 *
 *  @param[in] r_square: R^2 mod in_prime, where R = 2 ^ bits_len. If r_square is NULL, function will compute r_square and constp internally
 *
 *  @param[in] constp: montgomery multiplication constant of in_prime
 *
 *  @param[out] output signature: supports up to 2048 bits. NOTE: as PKCS#1 spec. the output is octstring, or big endian
 *
 *  @param[in] is_hash: if the message input is raw data, this parameter is 1, otherwise is 0
 *
 *  @retval::RSA_ERROR_PARAMETER  NULL input pointer.
 *  @retval::RSA_ERROR_VERIFY_FUNCTION  RSA VERIFY function is error.
 *  @retval::RSA_OK: execute successfully.
 *******************************************************************************************
 */
algo_rsa_ret_e crypto_rsa_pkcs1_sign(algo_rsa_config_t *rsa_config, uint32_t bits_len, uint8_t *message, uint32_t message_len, uint32_t private_key[], algo_rsa_public_exponent_e e,
           uint32_t in_prime[],uint32_t r_square[],uint32_t constp, uint8_t sig[], uint8_t is_hash);

/**
 *******************************************************************************************
 *  @brief verification signature according to PKCS#1 standard
 *  Verify the sig of message_hash by public key {e, in_prime, r_square, constp}
 *
 *  @param[in] rsa_config: rsa_config \ref algo_rsa_config_t, NOTE: for malibu firmware verification, please use RSA_PKCS1_V21 verify
 *
 *  @param[in] bits_len: bit width of modular number of in_prime, supports up to 2048 bits
 *
 *  @param[in] message:  input message.
 *  @param[in] message_len:  input message len.
 *
 *  @param[in] e: the exponent number may be 3, 17 or 65537 (recommended 65537), \ref algo_rsa_public_exponent_e
 *
 *  @param[in] in_prime: the modular number, which must be odd number
 *
 *  @param[in] r_square: R^2 mod in_prime, where R = 2 ^ bits_len. If r_square is NULL, function will compute r_square and constp internally
 *
 *  @param[in] constp: montgomery multiplication constant of in_prime
 *
 *  @param[in] input signature: supports up to 2048 bits. NOTE: as PKCS#1 spec. the output is octstring, or big endian
 *
 *  @param[in] is_hash: if the message input is raw data, this parameter is 1, otherwise is 0
 *
 *  @retval::RSA_ERROR_PARAMETER  NULL input pointer.
 *  @retval::RSA_ERROR_RNG_FUNCTION  RNG function is NULL.
 *  @retval::RSA_OK: execute successfully.
 *******************************************************************************************
 */
algo_rsa_ret_e crypto_rsa_pkcs1_verify(algo_rsa_config_t *rsa_config, uint32_t bits_len, uint8_t *message, uint32_t message_len, algo_rsa_public_exponent_e e,
           uint32_t in_prime[],uint32_t r_square[],uint32_t constp, const uint8_t sig[], uint8_t is_hash);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __CRYPTO_RSA_H__ */

/** @} */
/** @} */
/** @} */

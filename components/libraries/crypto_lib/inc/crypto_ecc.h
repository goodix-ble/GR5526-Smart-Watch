/**
 ****************************************************************************************
 *
 * @file    crypto_ecc.h
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

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup CRYPTO_DRIVER CRYPTO DRIVER
 *  @{
 */

/** @defgroup CRYPTO_ECC ECC
  * @brief ECC CRYPTO driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRYPTO_ECC_H__
#define __CRYPTO_ECC_H__

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/** @defgroup CRYPTO_ECC_MACRO Defines
  * @{
  */

#define ECC_U32_LENGTH   (8)  /**< ECC supported 256-bit. */
#define ECC_HASH_SHA_256 (1)  /**< ECC use hash sha256. */
#define ECC_HASH_NONE    (0)  /**< ECC without hash. */
/** @} */

/** @addtogroup CRYPTO_ECC_ENUM Enumerations
  * @{
  */

/**
 * @brief ECC Return code.
 */
typedef enum _algo_ecc_ret {

    ECC_OK = 0,                       /**< ECC Return OK. */
    ECC_ERROR_PARAMETER = 10000,      /**< ECC Return Parameter Error. */
    ECC_ERROR_SIGN,                   /**< ECC Return Sign Error. */
    ECC_ERROR_VERIFY,                 /**< ECC Return Verify Error. */
    ECC_ERROR_POINT_NOT_ON_CURVE,     /**< ECC Return Point is not on curve Error. */
    ECC_ERROR_INTERFACE_EMPTY,        /**< ECC Return interface not found. */
} algo_ecc_ret_e;

/**
 * @brief ECC Curve type.
 */
typedef enum _algo_ecc_curve_type {

    ECC_CURVE_SECP256R1,      /**< NIST SECP256R1 Curve. */
    ECC_CURVE_SECP256K1,      /**< NIST SECP256K1 Curve. */
} algo_ecc_curve_type_e;
/** @} */

/** @addtogroup CRYPTO_ECC_CONTEXT_STRUCTURES Structures
 * @{
 */

/** @defgroup ECC Point Structurs Definition
 * @{
 */

/**
 * @brief  Defines a data structure of a point, include x-axis and y-axis. And (0,0) is defined as the point of infinite.
 */
typedef struct _algo_ecc_point {

    uint32_t x[ECC_U32_LENGTH]; /**< point' x-axis, interger format (ANSI X9.62 - 2005 (Page 27))*/
    uint32_t y[ECC_U32_LENGTH]; /**< point' y-axis, interger format (ANSI X9.62 - 2005 (Page 27))*/

} algo_ecc_point_t;
/** @} */

/** @defgroup ECC Curve Structurs Definition
 * @{
 */

/**
 * @brief User defined elliptic curve description.
 */
typedef struct _algo_ecc_curve_parameter {

    /**
     * parameter A in Montgomery Filed! a = a * R, denoted integer format as ANSI X9.62 - 2005 (Page 27).
     * For example NIST P256 a = {0xfffffffc,0xffffffff,0xffffffff,0x00000003,0x00000000,0x00000000,0x00000004,0xfffffffc};
     */
    uint32_t a[ECC_U32_LENGTH]; /**< parameter A in Montgomery Filed! */

    /**
     * parameter B in Montgomery Filed! b = b * R, denoted integer format as ANSI X9.62 - 2005 (Page 27).
     * For example NIST P256 b = {0x29c4bddf,0xd89cdf62,0x78843090,0xacf005cd,0xf7212ed6,0xe5a220ab,0x04874834,0xdc30061d};
     */
    uint32_t b[ECC_U32_LENGTH]; /**< parameter B in Montgomery Filed! */

    /**
     * parameter p, denoted integer format as ANSI X9.62 - 2005 (Page 27).
     * For example NIST P256 p = {0xffffffff, 0x00000001, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff};
     */
    uint32_t p[ECC_U32_LENGTH];  /**< parameter P in curve */

    /**
     * parameter R ^ 2 mod p, where R = 2 ^ 256, denoted integer format as ANSI X9.62 - 2005 (Page 27).
     * For example NIST P256 R^2 mod p {0x00000004, 0xfffffffd, 0xffffffff, 0xfffffffe, 0xfffffffb, 0xffffffff, 0x00000000, 0x00000003}
     */
    uint32_t p_r_square[ECC_U32_LENGTH]; /**< R^2 mod p */

    /**
     * Montgomery multiplication constant for prime p
     */
    uint32_t constp;  /**< Montgomery multiplication constant for prime p */

    /**
     * parameter p, denoted integer format as ANSI X9.62 - 2005 (Page 27).
     * For example NIST P256 n = {0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xbce6faad, 0xa7179e84, 0xf3b9cac2, 0xfc632551}
     */
    uint32_t n[ECC_U32_LENGTH]; /**< NIST P256 n */

    /**
     * parameter R ^ 2 mod n, where R = 2 ^ 256, denoted integer format as ANSI X9.62 - 2005 (Page 27).
     * For example NIST P256  R^2 mod n {0x66e12d94, 0xf3d95620, 0x2845b239, 0x2b6bec59, 0x4699799c, 0x49bd6fa6, 0x83244c95, 0xbe79eea2}
     */
    uint32_t n_r_square[ECC_U32_LENGTH];  /**< R^2 mod n */

    /**
     * Montgomery multiplication constant for prime n
     */
    uint32_t constn; /**< Montgomery multiplication constant for prime n */

    /**
     * parameter h for ecc curve
     */
    uint32_t h; /**< parameter h for ecc curve */

    /**
     * generation point for ecc curve
     */
    algo_ecc_point_t G; /**< generation point for ecc curve */

} algo_ecc_curve_parameter_t;
/** @} */

/** @defgroup ECC Config Structurs Definition
 * @{
 */

/**
 * @brief ECC Computation config
 * \note It is recommended to set config via API ecc_init_config
 */
typedef struct _algo_ecc_config {

    /* Followings parameters are for Both ECC/RSA */
    /**
     * This member MUST BE specified. The default p256 params can be get from ROM
     */
    algo_ecc_curve_parameter_t *curve; /**< ecc curve */

} algo_ecc_config_t;
/** @} */

/** @defgroup ECDSA Config Structurs Definition
 * @{
 */

/**
 * @brief The ECDSA context structure
 */
typedef struct _algo_ecc_ecdsa_config {

    /**
     * ECDSA compute options, include curve type.
     */
    algo_ecc_config_t calc_options;  /**< ecc config options */

    /**
     * ECDSA our secret value, (a random number < n)
     */
    uint32_t our_secret_value[ECC_U32_LENGTH];  /**<ECDSA our secret value, (a random number < n) */

    /**
     * ECDSA our public value, our_public_point = our_secret_value * G
     */
    algo_ecc_point_t our_public_point;  /**<ECDSA our public value, our_public_point = our_secret_value * G */

    /**
     * ECDSA k, just for test.
     */
   uint32_t k[ECC_U32_LENGTH];  /**<ECDSA k value,just for test.when test end,we will delete this parameter */

} algo_ecc_ecdsa_config_t;
/** @} */

/** @defgroup ECDH Config Structurs Definition
 * @{
 */

/**
 * @brief An elliptic Diffie Hellman description
 */
typedef struct _algo_ecc_ecdh_config {

    /**
     * ECDH compute options, include curve type.
     */
    algo_ecc_config_t calc_options; /**< ecc config options */

    /**
     * ECDH our secret value, (a random number < n)
     */
    uint32_t our_secret_value[ECC_U32_LENGTH]; /**< ECDH our secret value, (a random number < n) */

    /**
     * ECDH our public value, our_public_point = our_secret_value * G
     */
    algo_ecc_point_t our_public_point; /**< ECDH our public value, our_public_point = our_secret_value * G*/

    /**
     * ECDH peer's public value
     */
    algo_ecc_point_t peer_public_point; /**< ECDH peer's public value */

    /**
     * ECDH shared point, shared_point = our_secret_value * peer_public_point,
     * and DHKey is shared_point.x
     */
    algo_ecc_point_t shared_point; /**< ECDH shared point, shared_point = our_secret_value * peer_public_point */

} algo_ecc_ecdh_config_t;
/** @} */
/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CRYPTO_ECC_FUNCTIONS Functions
  * @{
  */

// ECDSA  APIs//
/**
 *****************************************************************************************
 *  @brief ECDSA init config, such as curve type and this function will internally set rng, crc32 and sha function
 *
 *  @param[in] ecdsa_data:  ECC ECDSA data structure, refer to \ref algo_ecc_ecdh_config_t.
 *
 *  @param[in] curve: ECC curve type, refer to \ref algo_ecc_curve_type_e.
 *****************************************************************************************
 */
void crypto_ecc_ecdsa_init(algo_ecc_ecdsa_config_t *ecdsa_data, algo_ecc_curve_type_e curve);

/**
 *****************************************************************************************
 *  @brief generate our ECDSA secret key and public key
 *
 *  @param[in] ecdsa_data:  ECC ECDSA data structure, refer to \ref algo_ecc_ecdh_config_t. Generated data are write to ecdh_data members
 *
 *  @retval::ECC_ERROR_PARAMETER:NULL input pointer.
 *  @retval::ECC_OK: execute successfully.
 *****************************************************************************************
 */
algo_ecc_ret_e crypto_ecc_ecdsa_gen_secret_and_public(algo_ecc_ecdsa_config_t *ecdsa_data);

/**
 *****************************************************************************************
 *  @brief sign message with private key and output signiture pair {r,s};
 *  \note r, s are integer format
 *
 *  @param[in] ecdsa_calc_options: algo_ecdsa_config_t
 *
 *  @param[in] hash_func: choose hash function.
 *
 *  @param[in] message:  input message, interpreted as binary string.
 *
 *  @param[in] message_byte_length:  input message length in byte.
 *
 *  @param[out] out_signiture_r:  output 256-bit signiture r, integer format.
 *
 *  @param[out] out_signiture_s:  output 256-bit signiture s, integer format.
 *
 *  @retval::ECC_ERROR_PARAMETER:NULL input pointer.
 *  @retval::ECC_ERROR_SIGN:ECC sign failed.
 *  @retval::ECC_OK: execute successfully.
 *****************************************************************************************
 */
algo_ecc_ret_e crypto_ecc_ecdsa_sign(algo_ecc_ecdsa_config_t *ecdsa_calc_options, uint8_t hash_func,
            uint8_t *message, uint32_t message_byte_length,
            uint32_t out_signiture_r[ECC_U32_LENGTH], uint32_t out_signiture_s[ECC_U32_LENGTH]);

/**
 *****************************************************************************************
 *  @brief verify signiture pair {r,s} for message;
 *
 *  @param[in] ecdsa_calc_options: algo_ecdsa_config_t
 *
 *  @param[in] hash_func: choose hash function.
 *
 *  @param[in] message:  input message, interpreted as binary string.
 *
 *  @param[in] message_byte_length:  input message length in byte.
 *
 *  @param[in] in_signiture_r:  input 256-bit signiture r, integer format.
 *
 *  @param[in] in_signiture_s:  input 256-bit signiture s, integer format.
 *
 *  @retval::ECC_ERROR_PARAMETER:NULL input pointer.
 *  @retval::ECC_ERROR_VERIFY:ECC verify failed.
 *  @retval::ECC_ERROR_POINT_NOT_ON_CURVE: point add result is not on the p256 curves.
 *  @retval::ECC_OK: execute successfully.
 *****************************************************************************************
 */
algo_ecc_ret_e crypto_ecc_ecdsa_verify(algo_ecc_ecdsa_config_t *ecdsa_calc_options, uint8_t hash_func,
            uint8_t *message, uint32_t message_byte_length,
            uint32_t in_signiture_r[ECC_U32_LENGTH], uint32_t in_signiture_s[ECC_U32_LENGTH]);

// ECDH  APIs//
/**
 *****************************************************************************************
 *  @brief ECDH init config, such as curve type and this function will internally set rng, crc32 and sha function
 *
 *  @param[in] ecdh_data: ECC Diffie-Hellman data structure, refer to \ref algo_ecc_ecdh_config_t.
 *
 *  @param[in] curve: ECC curve type, refer to \ref algo_ecc_curve_type_e.
 *****************************************************************************************
 */
void crypto_ecc_ecdh_init(algo_ecc_ecdh_config_t *ecdh_data, algo_ecc_curve_type_e curve);

/**
 *****************************************************************************************
 *  @brief generate our ECDH secret key and public key
 *
 *  @param[in] ecdh_data:  ECC Diffie-Hellman data structure, refer to \ref algo_ecc_ecdh_config_t. Generated data are write to ecdh_data members
 *
 *  @retval::ECC_ERROR_PARAMETER:NULL input pointer.
 *  @retval::ECC_OK: execute successfully.
 *****************************************************************************************
 */
algo_ecc_ret_e crypto_ecc_ecdh_gen_secret_and_public(algo_ecc_ecdh_config_t *ecdh_data);

/**
 *****************************************************************************************
 *  @brief generate shared secret via peer's public key
 *
 *  @param[in] ecdh_data:  ECC Diffie-Hellman data structure, refer to \ref algo_ecc_ecdh_config_t. Generated data are write to ecdh_data members
 *
 *  @param[in] qb:  peer's public information.
 *
 *  @retval::ECC_ERROR_PARAMETER:NULL input pointer.
 *  @retval::ECC_ERROR_POINT_NOT_ON_CURVE:qb is not on the p256 curves.
 *  @retval::ECC_OK: execute successfully.
 *****************************************************************************************
 */
algo_ecc_ret_e crypto_ecc_ecdh_compute_shared(algo_ecc_ecdh_config_t *ecdh_data, algo_ecc_point_t *qb);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __CRYPTO_ECC_H__ */

/** @} */
/** @} */
/** @} */

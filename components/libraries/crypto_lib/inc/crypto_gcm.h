/**
 ****************************************************************************************
 *
 * @file    crypto_gcm.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of crypto GCM library.
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

/** @defgroup CRYPTO_GCM GCM
  * @brief GCM CRYPTO driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRYPTO_GCM_H__
#define __CRYPTO_GCM_H__

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "crypto_aes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/** @addtogroup CRYPTO_GCM_CONTEXT_STRUCTURES Structures
 * @{
 */

/** @defgroup GCM Context Structurs Definition
 * @{
 */

/**
 * @brief This defines the structure of gcm context.
 */
typedef struct crypto_gcm_context
{
    crypto_aes_context cipher_ctx;        /**< The cipher context used. */
    uint64_t HL[16];                      /**< Precalculated HTable low. */
    uint64_t HH[16];                      /**< Precalculated HTable high. */
    uint64_t len;                         /**< The total length of the encrypted data. */
    uint64_t add_len;                     /**< The total length of the additional data. */
    unsigned char base_ectr[16];          /**< The first ECTR for tag. */
    unsigned char y[16];                  /**< The Y working value. */
    unsigned char buf[16];                /**< The buf working value. */
    int mode;                             /**< The operation to perform:
                                               #AES_ENCRYPT or
                                               #AES_DECRYPT. */
}crypto_gcm_context;
/** @} */
/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CRYPTO_GCM_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  crypto gcm init.
 *
 * @param[in]  ctx: gcm context.
 *
 ****************************************************************************************
 */
void crypto_gcm_init( crypto_gcm_context *ctx );

/**
 ****************************************************************************************
 * @brief  crypto gcm set key.
 *
 * @param[in]  ctx: gcm context.
 * @param[in]  key: encryption/decryption key.
 * @param[in]  keybits: must be 128, 192 or 256.
 *
 * @retval ::-1: The gcm set key error.
 * @retval ::0: The gcm set key successfully.
 ****************************************************************************************
 */
int crypto_gcm_setkey( crypto_gcm_context *ctx, const uint8_t *key, uint32_t keybits );

/**
 ****************************************************************************************
 * @brief  start GCM encryption or decryption.
 *
 * @param[in]  ctx: gcm context.
 * @param[in]  mode: AES_ENCRYPT or AES_DECRYPT.
 * @param[in]  iv: The initialization vector.
 * @param[in]  iv_len: The length of the IV.
 * @param[in]  add: The buffer holding the additional data, or NULL if add_len is 0.
 * @param[in]  add_len: The length of the additional data.
 *
 * @retval ::-1: The gcm start error.
 * @retval ::0: The gcm start successfully.
 ****************************************************************************************
 */
int crypto_gcm_starts( crypto_gcm_context *ctx, int mode, const uint8_t *iv, uint32_t iv_len, const uint8_t *add, uint32_t add_len );

/**
 ****************************************************************************************
 * @brief  update GCM encryption or decryption buffer.
 *
 * @param[in]  ctx: gcm context.
 * @param[in]  length: The length of the input data. This must be a multiple of
 *                     16 except in the last call before crypto_gcm_finish().
 * @param[in]  input: The buffer holding the input data.
 * @param[out] output: The buffer for holding the output data.
 *
 * @retval ::-1: The gcm update error.
 * @retval ::0: The gcm update successfully.
 ****************************************************************************************
 */
int crypto_gcm_update( crypto_gcm_context *ctx, uint32_t length, const uint8_t *input, uint8_t *output );

/**
 ****************************************************************************************
 * @brief  GCM generates the authentication tag.
 *
 * @param[in]  ctx: gcm context.
 * @param[out] tag: The buffer for holding the tag.
 * @param[in]  tag_len: The length of the tag to generate.
 *
 * @retval ::-1: The gcm generates tag error.
 * @retval ::0: The gcm generates tag successfully.
 ****************************************************************************************
 */
int crypto_gcm_finish( crypto_gcm_context *ctx, uint8_t *tag, uint32_t tag_len );

/**
 ****************************************************************************************
 * @brief  crypto gcm free.
 *
 * @param[in]  ctx: gcm context.
 *
 ****************************************************************************************
 */
void crypto_gcm_free( crypto_gcm_context *ctx );

/**
 ****************************************************************************************
 * @brief  GCM encryption or decryption.
 *
 * @param[in]  ctx: gcm context.
 * @param[in]  mode: AES_ENCRYPT or AES_DECRYPT.
 * @param[in]  length: The length of the input data.
 * @param[in]  iv: The initialization vector.
 * @param[in]  iv_len: The length of the IV.
 * @param[in]  add: The buffer holding the additional data, or NULL if add_len is 0.
 * @param[in]  add_len: The length of the additional data.
 * @param[in]  input: The buffer holding the input data.
 * @param[out] output: The buffer for holding the output data.
 * @param[out] tag: The buffer for holding the tag.
 * @param[in]  tag_len: The length of the tag to generate.
 *
 * @retval ::-1: The gcm encryption or decryption error.
 * @retval ::0: The gcm encryption or decryption successfully.
 ****************************************************************************************
 */
int crypto_gcm_crypt_and_tag( crypto_gcm_context *ctx, int mode, uint32_t length,
                              const uint8_t *iv, uint32_t iv_len, const uint8_t *add, uint32_t add_len,
                              const uint8_t *input, uint8_t *output, uint32_t tag_len, uint8_t *tag );

/**
 ****************************************************************************************
 * @brief  GCM authenticated decryption.
 *
 * @param[in]  ctx: gcm context.
 * @param[in]  length: The length of the input data.
 * @param[in]  iv: The initialization vector.
 * @param[in]  iv_len: The length of the IV.
 * @param[in]  add: The buffer holding the additional data, or NULL if add_len is 0.
 * @param[in]  add_len: The length of the additional data.
 * @param[in]  tag: The buffer for holding the tag.
 * @param[in]  tag_len: The length of the tag to generate.
 * @param[in]  input: The buffer holding the input data.
 * @param[out] output: The buffer for holding the output data.
 *
 * @retval ::-1: The gcm authenticated decryption error.
 * @retval ::0: The gcm authenticated decryption successfully.
 ****************************************************************************************
 */
int crypto_gcm_auth_decrypt( crypto_gcm_context *ctx, uint32_t length,
                             const uint8_t *iv, uint32_t iv_len, const uint8_t *add, uint32_t add_len,
                             const uint8_t *tag, uint32_t tag_len, const uint8_t *input, uint8_t *output );
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __CRYPTO_GCM_H__ */

/** @} */
/** @} */
/** @} */

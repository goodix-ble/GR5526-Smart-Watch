/**
 ****************************************************************************************
 *
 * @file    crypto_sha256.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of crypto SHA256 library.
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

/** @defgroup CRYPTO_SHA SHA
  * @brief SHA CRYPTO driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRYPTO_SHA256_H__
#define __CRYPTO_SHA256_H__

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
/** @defgroup CRYPTO_SHA_MACRO Defines
  * @{
  */

/**
 * @brief   SHA256 SIZE
 */
#define SHA256_SIZE 32
/**
 * @brief   SHA256 MAX SIZE
 */
#define SHA256_MAX_SIZE 32
/**
 * @brief   SHA256 BLOCK SIZE
 */
#define SHA256_BLOCK_SIZE 64
/**
 * @brief   HMAC-SHA256 KEY SIZE
 */
#define HMAC_SHA256_KEY_SIZE 32
/** @} */

/** @addtogroup CRYPTO_SHA_CONTEXT_STRUCTURES Structures
 * @{
 */

/** @defgroup SHA Context Structurs Definition
 * @{
 */

/**
 * @brief This defines the structure of sha256 context.
 */
typedef struct crypto_sha256_context
{
    uint32_t total;      /**< The number of Bytes processed. */
    uint32_t block_size; /**< block size. */
    uint8_t *input;      /**< input data. */
    void *instance;      /**< The sha256 instance. */
} crypto_sha256_context;
/** @} */
/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CRYPTO_SHA_FUNCTIONS Functions
  * @{
  */

/**
 *******************************************************************************************
 * @brief          This function initializes a SHA-256 context.
 *
 * @param[in]      ctx: The SHA-256 context to initialize. This must not be NULL.
 *
 *******************************************************************************************
 */
void crypto_sha256_init(crypto_sha256_context *ctx);

/**
 *******************************************************************************************
 * @brief          This function clears a SHA-256 context.
 *
 * @param[in]      ctx: The SHA-256 context to clear. This may be NULL,
 *                 in which case this function does nothing. If it
 *                 is not \c NULL, it must point to an initialized
 *                 SHA-256 context.
 *******************************************************************************************
 */
void crypto_sha256_free(crypto_sha256_context *ctx);

/**
 *******************************************************************************************
 * @brief          This function clones the state of a SHA-256 context.
 *
 * @param[out]     dst: The destination context. This must be initialized.
 * @param[in]      src: The context to clone. This must be initialized.
 *******************************************************************************************
 */
void crypto_sha256_clone(crypto_sha256_context *dst, const crypto_sha256_context *src);

/**
 *******************************************************************************************
 * @brief          This function starts a SHA-256 calculation.
 *
 * @param[in]      ctx: The SHA-256 context to use. This must be initialized.
 *
 * @retval::-1:NULL input pointer.
 * @retval::0: execute successfully.
 *******************************************************************************************
 */
int crypto_sha256_starts(crypto_sha256_context *ctx);

/**
 *******************************************************************************************
 * @brief          This function feeds an input buffer into an ongoing
 *                 SHA-256 calculation.
 *
 * @param[in]      ctx: The SHA-256 context. This must be initialized
 *                 and have a hash operation started.
 * @param[in]      input: The buffer holding the input data. This must
 *                 be a readable buffer of length \p ilen Bytes.
 * @param[in]      ilen: The length of the input data in Bytes.
 *
 * @retval::-1:NULL input pointer.
 * @retval::0: execute successfully.
 *******************************************************************************************
 */
int crypto_sha256_update(crypto_sha256_context *ctx, const uint8_t *input, size_t ilen);

/**
 *******************************************************************************************
 * @brief          This function finishes the SHA-256 operation, and writes
 *                 the result to the output buffer. This function is for
 *                 internal use only.
 *
 * @param[in]      ctx: The SHA-256 context. This must be initialized
 *                 and have a hash operation started.
 * @param[out]     output: SHA-256 result.
 *                 This must be a writable buffer of length \c 32 Bytes.
 *
 * @retval::-1:NULL input pointer.
 * @retval::0: execute successfully.
 *******************************************************************************************
 */
int crypto_sha256_finish(crypto_sha256_context *ctx, uint8_t output[32]);

/**
 *******************************************************************************************
 * @brief          This function calculates the SHA-256 into a buffer.
 *
 *                 The function allocates the context, performs the
 *                 calculation, and frees the context.
 *
 *                 The SHA-256 result is calculated as
 *                 output = SHA-256(input buffer).
 *
 * @param[in]      input: The buffer holding the input data. This must be
 *                 a readable buffer of length \p ilen Bytes.
 * @param[in]      ilen: The length of the input data in Bytes.
 * @param[out]     output: SHA-256 result.
 *                 This must be a writable buffer of length \c 32 Bytes.
 *
 * @retval::-1:NULL input pointer.
 * @retval::0: execute successfully.
 *******************************************************************************************
 */
int crypto_sha256(const uint8_t *input, size_t ilen, uint8_t output[32]);

/**
 *******************************************************************************************
 * @brief          This function starts a HMAC-SHA-256 calculation.
 *
 * @param[in]      ctx: The HMAC-SHA-256 context to use. This must be initialized.
 * @param[in]      key: The HMAC secret key.
 * @param[in]      keylen: The length of the HMAC key in Bytes.
 *
 * @retval::-1:NULL input pointer.
 * @retval::0: execute successfully.
 *******************************************************************************************
 */
int crypto_hmac_sha256_starts(crypto_sha256_context *ctx, const uint8_t *key, size_t keylen);

/**
 *******************************************************************************************
 * @brief          This function feeds an input buffer into an ongoing
 *                 HMAC-SHA-256 calculation.
 *
 * @param[in]      ctx: The HMAC-SHA-256 context. This must be initialized
 *                 and have a hash operation started.
 * @param[in]      input: The buffer holding the input data. This must
 *                 be a readable buffer of length \p ilen Bytes.
 * @param[in]      ilen: The length of the input data in Bytes.
 *
 * @retval::-1:NULL input pointer.
 * @retval::0: execute successfully.
 *******************************************************************************************
 */
int crypto_hmac_sha256_update(crypto_sha256_context *ctx, const uint8_t *input, size_t ilen);

/**
 *******************************************************************************************
 * @brief          This function finishes the HMAC-SHA-256 operation, and
 *                 writes the result to the output buffer. This function is for
 *                 is forinternal use only.
 *
 * @param[in]      ctx: The HMAC-SHA-256 context. This must be initialized
 *                 and have a hash operation started.
 * @param[out]     output: HMAC-SHA-256 checksum result.
 *                 This must be a writable buffer of length \c 32 Bytes.
 *
 * @retval::-1:NULL input pointer.
 * @retval::0: execute successfully.
 *******************************************************************************************
 */
int crypto_hmac_sha256_finish(crypto_sha256_context *ctx, uint8_t output[32]);

/**
 *******************************************************************************************
 * @brief          This function calculates the HMAC-SHA-256 into a buffer.
 *
 *                 The function allocates the context, performs the
 *                 calculation, and frees the context.
 *
 *                 The HMAC-SHA-256 result is calculated as
 *                 output = HMAC-SHA-256(input buffer).
 *
 * @param[in]      key: The HMAC secret key.
 * @param[in]      keylen: The length of the HMAC key in Bytes.
 * @param[in]      input: The buffer holding the input data. This must be
 *                 a readable buffer of length \p ilen Bytes.
 * @param[in]      ilen: The length of the input data in Bytes.
 * @param[out]     output: HMAC-SHA-256 result.
 *                 This must be a writable buffer of length \c 32 Bytes.
 *
 * @retval::-1:NULL input pointer.
 * @retval::0: execute successfully.
 *******************************************************************************************
 */
int crypto_hmac_sha256(const uint8_t *key, size_t keylen, const uint8_t *input, size_t ilen, uint8_t output[32]);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __CRYPTO_SHA256_H__ */

/** @} */
/** @} */
/** @} */

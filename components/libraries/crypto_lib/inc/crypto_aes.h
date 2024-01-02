/**
 ****************************************************************************************
 *
 * @file    crypto_aes.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of crypto AES library.
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

/** @defgroup CRYPTO_AES AES
  * @brief AES CRYPTO driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRYPTO_AES_H__
#define __CRYPTO_AES_H__

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
/** @defgroup CRYPTO_AES_MACRO Defines
  * @{
  */

#define AES_BLOCK_SIZE  16   /**< AES BLOCK SIZE.   */
#define AES_ENCRYPT     1    /**< AES ENCRYPT MODE. */
#define AES_DECRYPT     0    /**< AES DECRYPT MODE. */
#define AES_MAX_KEY_SIZE 32  /**< AES MAX KEY SIZE. */
/** @} */

/** @addtogroup CRYPTO_AES_ENUM Enumerations
  * @{
  */

/**
  * @brief This defines the process state of aes padding type value.
  */
typedef enum {
    PADDING_NONE = 0,    /**< AES padding type none.  */
    PADDING_ZEROS,       /**< AES padding type zeros. */
    PADDING_PKCS7,       /**< AES padding type pkcs7. */
} crypto_aes_padding_t;
/** @} */

/** @addtogroup CRYPTO_AES_CONTEXT_STRUCTURES Structures
 * @{
 */

/** @defgroup AES Context Structurs Definition
 * @{
 */

/**
 * @brief This defines the structure of aes context.
 */
typedef struct
{
    uint8_t key[AES_MAX_KEY_SIZE];       /**< the key. */
    uint16_t keybits;                 /**< the keybits. */
    crypto_aes_padding_t   padding_mode;     /**< padding mode. */
    void *instance;                   /**< the aes instance. */
} crypto_aes_context;
/** @} */
/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CRYPTO_AES_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  crypto aes init.
 *
 * @param[in]  ctx: aes context.
 *
 ****************************************************************************************
 */
void crypto_aes_init(crypto_aes_context *ctx);

/**
 ****************************************************************************************
 * @brief  crypto aes free.
 *
 * @param[in]  ctx: aes context.
 *
 ****************************************************************************************
 */
void crypto_aes_free(crypto_aes_context *ctx);

/**
 ****************************************************************************************
 * @brief  crypto aes set the padding type.
 *
 * @param[in]  ctx: aes context.
 * @param[in]  padding_type: PADDING_NONE/PADDING_ZEROS/PADDING_PKCS7.
 *
 * @retval ::-1: The aes set paddings type error.
 * @retval ::0: The aes set paddings type successfully.
 ****************************************************************************************
 */
int crypto_aes_set_paddings(crypto_aes_context *ctx, crypto_aes_padding_t padding_type);

/**
 * ****************************************************************************************
 * @brief         AES key schedule (encryption)
 *
 * @param[in]     ctx: AES context to be initialized
 * @param[in]     key: encryption key
 * @param[in]     keybits: must be 128, 192 or 256
 *
 * @retval ::-1: The aes set key enc error.
 * @retval ::0:  The aes set key enc successfully.
 * ****************************************************************************************
 */
int crypto_aes_setkey_enc(crypto_aes_context *ctx, uint8_t *key, uint16_t keybits);

/**
 * ****************************************************************************************
 * @brief        AES key schedule (decryption)
 *
 * @param[in]     ctx: AES context to be initialized
 * @param[in]     key: encryption key
 * @param[in]     keybits: must be 128, 192 or 256
 *
 * @retval ::-1: The aes set key enc error.
 * @retval ::0:  The aes set key enc successfully.
 * ****************************************************************************************
 */
int crypto_aes_setkey_dec(crypto_aes_context *ctx, uint8_t *key, uint16_t keybits);

/**
 * ****************************************************************************************
 * @brief          pkcs7 padding algorithm
 *
 * @param[in]      input: input block
 * @param[in]      length: input length
 * @param[out]     output: output block
 *
 * @retval ::-1: The aes pkcs7 padding error.
 * @retval ::others:  The aes pkcs7 padding output length.
 * *****************************************************************************************
 */
int crypto_aes_pkcs7_padding(uint8_t *input, uint32_t length, uint8_t *output);

/**
 * ****************************************************************************************
 * @brief          zero padding algorithm
 *
 * @param[in]      input: input block
 * @param[in]      length: input length
 * @param[out]     output: output block
 *
 * @retval ::-1: The aes zero padding error.
 * @retval ::others:  The aes zero padding output length.
 * *****************************************************************************************
 */
int crypto_aes_zero_padding(uint8_t *input, uint32_t length, uint8_t *output);

/**
 * ****************************************************************************************
 * @brief          get output length
 *
 * @param[in]      length: input length
 * @param[in]      padding_type: padding type
 *
 * @retval ::len:  The output len.
 * *****************************************************************************************
 */
uint32_t crypto_aes_get_output_length(uint32_t length, crypto_aes_padding_t padding_type);

/**
 * ****************************************************************************************
 * @brief          AES-ECB block encryption/decryption
 *
 * @param[in]      ctx: AES context
 * @param[in]      mode: AES_ENCRYPT or AES_DECRYPT
 * @param[in]      input: input block
 * @param[in]      length: input length
 * @param[out]     output: output block
 *
 * @retval ::-1: The aes ecb crypt error.
 * @retval ::0:  The aes ecb crypt successfully.
 * *****************************************************************************************
 */
int crypto_aes_crypt_ecb(crypto_aes_context *ctx, uint8_t mode, uint8_t *input, uint32_t length, uint8_t *output);

/**
 * *****************************************************************************************
 * @brief         AES-CBC buffer encryption/decryption
 *                 Length should be a multiple of the block
 *                 size (16 bytes)
 *
 * @note           Upon exit, the content of the IV is updated so that you can
 *                 call the function same function again on the following
 *                 block(s) of data and get the same result as if it was
 *                 encrypted in one call. This allows a "streaming" usage.
 *                 If on the other hand you need to retain the contents of the
 *                 IV, you should either save it manually or use the cipher
 *                 module instead.
 *
 * @param[in]   ctx: AES context
 * @param[in]   mode: AES_ENCRYPT or AES_DECRYPT
 * @param[in]   iv:  initialization vector (updated after use)
 * @param[in]   input: buffer holding the input data
 * @param[in]   length: buffer holding the input data length
 * @param[out]  output: buffer holding the output data
 *
 * @retval ::-1: The aes cbc crypt error.
 * @retval ::0:  The aes cbc crypt successfully.
 * *****************************************************************************************
 */
int crypto_aes_crypt_cbc(crypto_aes_context *ctx, uint8_t mode, uint8_t iv[16], uint8_t *input, uint32_t length, uint8_t *output);

/**
 * *****************************************************************************************
 * @brief         AES-CTR buffer encryption/decryption
 *                 Length should be a multiple of the block
 *                 size (16 bytes)
 *
 * @note           Upon exit, the content of the IV is updated so that you can
 *                 call the function same function again on the following
 *                 block(s) of data and get the same result as if it was
 *                 encrypted in one call. This allows a "streaming" usage.
 *                 If on the other hand you need to retain the contents of the
 *                 IV, you should either save it manually or use the cipher
 *                 module instead.
 *
 * @param[in]   ctx: AES context
 * @param[in]   length: buffer holding the input data length
 * @param[in]   nc_off: The offset in the current stream_block, for resuming within the current cipher stream.
 *                      The offset pointer should be 0 at the start of a stream.
 * @param[in]   nonce_counter: The 128-bit nonce and counter.
 *                             It must be a readable-writeable buffer of 16 Bytes.
 * @param[in]   stream_block: The saved stream block for resuming.
 *                            This is overwritten by the function.
 *                            It must be a readable-writeable buffer of 16 Bytes.
 * @param[in]   input: buffer holding the input data
 * @param[out]  output: buffer holding the output data
 *
 * @retval ::-1: The aes ctr crypt error.
 * @retval ::0:  The aes ctr crypt successfully.
 * *****************************************************************************************
 */
int crypto_aes_crypt_ctr(crypto_aes_context *ctx, uint32_t length, uint32_t *nc_off,
                         uint8_t nonce_counter[16], uint8_t stream_block[16], const uint8_t *input, uint8_t *output);

/**
 * *****************************************************************************************
 * @brief         AES-CFB128 buffer encryption/decryption
 *                 Length should be a multiple of the block
 *                 size (16 bytes)
 *
 * @note           Upon exit, the content of the IV is updated so that you can
 *                 call the function same function again on the following
 *                 block(s) of data and get the same result as if it was
 *                 encrypted in one call. This allows a "streaming" usage.
 *                 If on the other hand you need to retain the contents of the
 *                 IV, you should either save it manually or use the cipher
 *                 module instead.
 *
 * @param[in]   ctx: AES context
 * @param[in]   mode: AES operation
 * @param[in]   length: buffer holding the input data length
 * @param[in]   iv_off: The offset in IV (updated after use)
 * @param[in]   iv: The initialization vector (updated after use)
 * @param[in]   input: buffer holding the input data
 * @param[out]  output: buffer holding the output data
 *
 * @retval ::-1: The aes ctr crypt error.
 * @retval ::0:  The aes ctr crypt successfully.
 * *****************************************************************************************
 */
int crypto_aes_crypt_cfb128(crypto_aes_context *ctx, uint8_t mode, uint32_t length, uint32_t *iv_off,
                            uint8_t iv[16], const uint8_t *input, uint8_t *output);

/**
 * *****************************************************************************************
 * @brief         AES-OFB buffer encryption/decryption
 *                 Length should be a multiple of the block
 *                 size (16 bytes)
 *
 * @note           Upon exit, the content of the IV is updated so that you can
 *                 call the function same function again on the following
 *                 block(s) of data and get the same result as if it was
 *                 encrypted in one call. This allows a "streaming" usage.
 *                 If on the other hand you need to retain the contents of the
 *                 IV, you should either save it manually or use the cipher
 *                 module instead.
 *
 * @param[in]   ctx: AES context
 * @param[in]   length: buffer holding the input data length
 * @param[in]   iv_off: The offset in IV (updated after use)
 * @param[in]   iv: The initialization vector (updated after use)
 * @param[in]   input: buffer holding the input data
 * @param[out]  output: buffer holding the output data
 *
 * @retval ::-1: The aes ctr crypt error.
 * @retval ::0:  The aes ctr crypt successfully.
 * *****************************************************************************************
 */
int crypto_aes_crypt_ofb(crypto_aes_context *ctx, uint32_t length, uint32_t *iv_off,
                         uint8_t iv[16], const uint8_t *input, uint8_t *output);
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __CRYPTO_AES_H__ */

/** @} */
/** @} */
/** @} */

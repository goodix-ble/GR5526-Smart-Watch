#include "crypto_aes.h"
#include "grx_hal.h"

#define crypto_malloc     malloc
#define crypto_free       free

typedef struct
{
    uint8_t seed[AES_BLOCK_SIZE];
    aes_handle_t aes_handle;
} aes_instance_t;

/*
 * AES context init
 */
void crypto_aes_init(crypto_aes_context *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    memset(ctx, 0x0, sizeof(crypto_aes_context));

    ctx->instance = crypto_malloc(sizeof(aes_instance_t));

    if (NULL == ctx->instance)
    {
        return;
    }

    memset(ctx->instance, 0, sizeof(aes_instance_t));
}

/*
 * AES context free
 */
void crypto_aes_free(crypto_aes_context *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    if (ctx->instance)
    {
        crypto_free(ctx->instance);
    }
}

/*
 * AES set paddings mode
 */
int crypto_aes_set_paddings(crypto_aes_context *ctx, crypto_aes_padding_t padding_type)
{
    if (ctx == NULL)
    {
        return -1;
    }
    ctx->padding_mode = padding_type;
    return 0;
}

static void aes_hardware_reset(void)
{
    CLEAR_BITS(MCU_SUB->SECURITY_RESET, MCU_SUB_SECURITY_RESET_AES);
    SET_BITS(MCU_SUB->SECURITY_RESET, MCU_SUB_SECURITY_RESET_AES);
}

static int crypto_set_aes_key(crypto_aes_context *ctx, uint8_t *key, uint16_t keybits)
{
    int8_t ret = 0;
    if (ctx == NULL || key == NULL || ctx->instance == NULL)
    {
        return -1;
    }

    memset(ctx->key, 0, AES_MAX_KEY_SIZE);
    memcpy(ctx->key, key, keybits >> 3);
    ctx->keybits = keybits;

    aes_handle_t *p_aes_handle = &((aes_instance_t *)ctx->instance)->aes_handle;

    switch (ctx->keybits)
    {
    case 128:
        p_aes_handle->init.key_size = AES_KEYSIZE_128BITS;
        break;
    case 192:
        p_aes_handle->init.key_size = AES_KEYSIZE_192BITS;
        break;
    case 256:
        p_aes_handle->init.key_size = AES_KEYSIZE_256BITS;
        break;
    default:
        ret = -1;
        break;
    }

    p_aes_handle->init.p_key = (uint32_t *)ctx->key;

    return ret;
}

/*
 * AES key schedule (encryption)
 */
int crypto_aes_setkey_enc(crypto_aes_context *ctx, uint8_t *key, uint16_t keybits)
{
    return crypto_set_aes_key(ctx, key, keybits);
}

/*
 * AES key schedule (decryption)
 */
int crypto_aes_setkey_dec(crypto_aes_context *ctx, uint8_t *key, uint16_t keybits)
{
    return crypto_set_aes_key(ctx, key, keybits);
}

/*
 * pkcs7 padding
 */
int crypto_aes_pkcs7_padding(uint8_t *input, uint32_t length, uint8_t *output)
{
    if (input == NULL || output == NULL)
    {
        return -1;
    }

    uint32_t out_len = ((length >> 4) + 1) * AES_BLOCK_SIZE;
    int padding_value = AES_BLOCK_SIZE - (length & 0xF);

    if (input != output)
    {
        for (uint32_t i = 0; i < length; i++)
        {
            output[i] = input[i];
        }
    }

    for (uint32_t i = length; i < out_len; i++)
    {
        output[i] = padding_value;
    }

    return out_len;
}

/*
 * zero padding
 */
int crypto_aes_zero_padding(uint8_t *input, uint32_t length, uint8_t *output)
{
    if (input == NULL || output == NULL)
    {
        return -1;
    }

    uint32_t out_len = ((length >> 4) + 1) * AES_BLOCK_SIZE;

    if (input != output)
    {
        for (uint32_t i = 0; i < length; i++)
        {
            output[i] = input[i];
        }
    }

    for (uint32_t i = length; i < out_len; i++)
    {
        output[i] = 0;
    }

    return out_len;
}

/*
 * get output length
 */
uint32_t crypto_aes_get_output_length(uint32_t length, crypto_aes_padding_t padding_type)
{
    if (PADDING_NONE == padding_type)
    {
        return length;
    }

    return ((length >> 4) + 1) * AES_BLOCK_SIZE;
}

/*
 * AES-ECB buffer encryption/decryption
 */
static int crypto_internal_aes_ecb_crypt(crypto_aes_context *ctx,
                                         uint8_t mode,
                                         uint8_t *input,
                                         uint32_t length,
                                         uint8_t *output)
{
    int ret = 0;
    aes_handle_t *p_aes_handle = &((aes_instance_t *)ctx->instance)->aes_handle;

    p_aes_handle->p_instance = AES;
    p_aes_handle->init.chaining_mode = AES_CHAININGMODE_ECB;
    p_aes_handle->init.p_init_vector = NULL;
    p_aes_handle->init.p_seed = (uint32_t *)(((aes_instance_t *)ctx->instance)->seed);
    p_aes_handle->init.dpa_mode = DISABLE;

    hal_aes_deinit(p_aes_handle);
    aes_hardware_reset();
    hal_aes_init(p_aes_handle);

    if (mode == AES_ENCRYPT)
    {
        if (HAL_OK != hal_aes_ecb_encrypt(p_aes_handle, (uint32_t *)input, length, (uint32_t *)output, 5000))
        {
            ret = -1;
            goto exit;
        }
    }
    else
    {
        if (HAL_OK != hal_aes_ecb_decrypt(p_aes_handle, (uint32_t *)input, length, (uint32_t *)output, 5000))
        {
            ret = -1;
            goto exit;
        }
    }

exit:
    hal_aes_deinit(p_aes_handle);
    aes_hardware_reset();
    return ret;
}

/*
 * AES-ECB block encryption/decryption
 */
int crypto_aes_crypt_ecb(crypto_aes_context *ctx, uint8_t mode, uint8_t *input, uint32_t length, uint8_t *output)
{
    int ret = 0;

    uint32_t compute_len = 0;
    uint8_t padding_len = 0;

    uint8_t padding_temp[AES_BLOCK_SIZE] = {0};

    uint8_t *p_input = input;
    uint8_t *p_output = output;

    if (ctx == NULL || input == NULL || output == NULL || mode > 1 || length == 0)
    {
        return -1;
    }

    compute_len = length & (~0xF);
    padding_len = length & 0xF;

    if ((ctx->padding_mode == PADDING_NONE) && padding_len)
    {
        return -1;
    }

    if (0 != crypto_internal_aes_ecb_crypt(ctx, mode, p_input, compute_len, p_output))
    {
        return -1;
    }

    if (PADDING_NONE == ctx->padding_mode)
    {
        return 0;
    }

    p_input += compute_len;
    p_output += compute_len;

    if (PADDING_ZEROS == ctx->padding_mode)
    {
        crypto_aes_zero_padding(p_input, padding_len, padding_temp);
    }
    else if (PADDING_PKCS7 == ctx->padding_mode)
    {
        crypto_aes_pkcs7_padding(p_input, padding_len, padding_temp);
    }
    else
    {
        ret = -1;
    }

    if (0 != ret)
    {
        return ret;
    }

    if (0 != crypto_internal_aes_ecb_crypt(ctx, mode, padding_temp, AES_BLOCK_SIZE, p_output))
    {
        return -1;
    }

    return ret;
}

/*
 * AES-CBC buffer encryption/decryption
 */
static int crypto_internal_aes_cbc_crypt(crypto_aes_context *ctx,
                                         uint8_t mode,
                                         uint8_t iv[AES_BLOCK_SIZE],
                                         const uint8_t *input,
                                         uint32_t length,
                                         uint8_t *output)
{
    int ret = 0;
    aes_handle_t *p_aes_handle = &((aes_instance_t *)ctx->instance)->aes_handle;

    p_aes_handle->p_instance = AES;
    p_aes_handle->init.chaining_mode = AES_CHAININGMODE_CBC;
    p_aes_handle->init.p_init_vector = (uint32_t *)iv;
    p_aes_handle->init.p_seed = (uint32_t *)(((aes_instance_t *)ctx->instance)->seed);
    p_aes_handle->init.dpa_mode = DISABLE;

    hal_aes_deinit(p_aes_handle);
    aes_hardware_reset();
    hal_aes_init(p_aes_handle);

    if (mode == AES_ENCRYPT)
    {
        if (HAL_OK != hal_aes_cbc_encrypt(p_aes_handle, (uint32_t *)input, length, (uint32_t *)output, 5000))
        {
            ret = -1;
            goto exit;
        }
    }
    else
    {
        if (HAL_OK != hal_aes_cbc_decrypt(p_aes_handle, (uint32_t *)input, length, (uint32_t *)output, 5000))
        {
            ret = -1;
            goto exit;
        }
    }

    if (mode == AES_ENCRYPT)
    {
        memcpy(iv, output + length - AES_BLOCK_SIZE, AES_BLOCK_SIZE);
    }
    else
    {
        memcpy(iv, input + length - AES_BLOCK_SIZE, AES_BLOCK_SIZE);
    }

exit:
    hal_aes_deinit(p_aes_handle);
    aes_hardware_reset();
    return ret;
}

/*
 * AES-CBC buffer encryption/decryption
 */
int crypto_aes_crypt_cbc(crypto_aes_context *ctx, uint8_t mode, uint8_t iv[16], uint8_t *input, uint32_t length, uint8_t *output)
{
    int ret = 0;

    uint32_t compute_len = 0;
    uint8_t padding_len = 0;

    uint8_t padding_temp[AES_BLOCK_SIZE] = {0};

    uint8_t *p_input = input;
    uint8_t *p_output = output;

    if (ctx == NULL || input == NULL || output == NULL || mode > 1 || length == 0)
    {
        return -1;
    }

    compute_len = length & (~0xF);
    padding_len = length & 0xF;

    if ((ctx->padding_mode == PADDING_NONE) && padding_len)
    {
        return -1;
    }

    if (0 != crypto_internal_aes_cbc_crypt(ctx, mode, iv, p_input, compute_len, p_output))
    {
        return -1;
    }

    if (PADDING_NONE == ctx->padding_mode)
    {
        return 0;
    }

    p_input += compute_len;
    p_output += compute_len;

    if (PADDING_ZEROS == ctx->padding_mode)
    {
        crypto_aes_zero_padding(p_input, padding_len, padding_temp);
    }
    else if (PADDING_PKCS7 == ctx->padding_mode)
    {
        crypto_aes_pkcs7_padding(p_input, padding_len, padding_temp);
    }
    else
    {
        ret = -1;
    }

    if (0 != ret)
    {
        return ret;
    }

    if (0 != crypto_internal_aes_cbc_crypt(ctx, mode, iv, padding_temp, AES_BLOCK_SIZE, p_output))
    {
        return -1;
    }

    return ret;
}

/*
 * AES-CTR buffer encryption/decryption
 */
int crypto_aes_crypt_ctr(crypto_aes_context *ctx, uint32_t length, uint32_t *nc_off,
                         uint8_t nonce_counter[16], uint8_t stream_block[16], const uint8_t *input, uint8_t *output)
{
    int c, i;
    size_t n;

    if (ctx == NULL || nc_off == NULL || nonce_counter == NULL || stream_block == NULL || input == NULL || output == NULL)
    {
        return ( -1 );
    }

    int ret = 0;
    aes_handle_t *p_aes_handle = &((aes_instance_t *)ctx->instance)->aes_handle;

    p_aes_handle->p_instance = AES;
    p_aes_handle->init.chaining_mode = AES_CHAININGMODE_ECB;
    p_aes_handle->init.p_init_vector = NULL;
    p_aes_handle->init.p_seed = (uint32_t *)(((aes_instance_t *)ctx->instance)->seed);
    p_aes_handle->init.dpa_mode = DISABLE;

    hal_aes_deinit(p_aes_handle);
    aes_hardware_reset();
    hal_aes_init(p_aes_handle);

    n = *nc_off;

    if ( n > 0x0F )
        return ( -1 );

    while( length-- )
    {
        if( n == 0 ) {
            if ( HAL_OK != hal_aes_ecb_encrypt(p_aes_handle, (uint32_t *)nonce_counter, 16, (uint32_t *)stream_block, 5000) )
            {
                ret = -1;
                goto exit;
            }

            for( i = 16; i > 0; i-- )
                if( ++nonce_counter[i - 1] != 0 )
                    break;
        }
        c = *input++;
        *output++ = (unsigned char)( c ^ stream_block[n] );

        n = ( n + 1 ) & 0x0F;
    }

    *nc_off = n;

exit:
    hal_aes_deinit(p_aes_handle);
    aes_hardware_reset();
    return( ret );
}

/*
 * AES-CFB128 buffer encryption/decryption
 */
int crypto_aes_crypt_cfb128(crypto_aes_context *ctx, uint8_t mode, uint32_t length, uint32_t *iv_off,
                            uint8_t iv[16], const uint8_t *input, uint8_t *output)
{
    int c;
    size_t n;

    if (ctx == NULL || mode > 1 || iv_off == NULL || iv == NULL || input == NULL || output == NULL)
    {
        return ( -1 );
    }

    int ret = 0;
    aes_handle_t *p_aes_handle = &((aes_instance_t *)ctx->instance)->aes_handle;

    p_aes_handle->p_instance = AES;
    p_aes_handle->init.chaining_mode = AES_CHAININGMODE_ECB;
    p_aes_handle->init.p_init_vector = NULL;
    p_aes_handle->init.p_seed = (uint32_t *)(((aes_instance_t *)ctx->instance)->seed);
    p_aes_handle->init.dpa_mode = DISABLE;

    hal_aes_deinit(p_aes_handle);
    aes_hardware_reset();
    hal_aes_init(p_aes_handle);

    n = *iv_off;

    if( n > 15 )
        return ( -1 );

    if( mode == AES_DECRYPT )
    {
        while( length-- )
        {
            if( n == 0 )
            {
                if ( HAL_OK != hal_aes_ecb_encrypt(p_aes_handle, (uint32_t *)iv, 16, (uint32_t *)iv, 5000) )
                {
                    ret = -1;
                    goto exit;
                }
            }

            c = *input++;
            *output++ = (unsigned char)( c ^ iv[n] );
            iv[n] = (unsigned char) c;

            n = ( n + 1 ) & 0x0F;
        }
    }
    else
    {
        while( length-- )
        {
            if( n == 0 )
            {
                if ( HAL_OK != hal_aes_ecb_encrypt(p_aes_handle, (uint32_t *)iv, 16, (uint32_t *)iv, 5000) )
                {
                    ret = -1;
                    goto exit;
                }
            }

            iv[n] = *output++ = (unsigned char)( iv[n] ^ *input++ );

            n = ( n + 1 ) & 0x0F;
        }
    }

    *iv_off = n;

exit:
    hal_aes_deinit(p_aes_handle);
    aes_hardware_reset();
    return( ret );
}

/*
 * AES-OFB (Output Feedback Mode) buffer encryption/decryption
 */
int crypto_aes_crypt_ofb(crypto_aes_context *ctx, uint32_t length, uint32_t *iv_off,
                         uint8_t iv[16], const uint8_t *input, uint8_t *output)
{
    int ret = 0;
    size_t n;

    if (ctx == NULL || iv_off == NULL || iv == NULL || input == NULL || output == NULL)
    {
        return ( -1 );
    }

    aes_handle_t *p_aes_handle = &((aes_instance_t *)ctx->instance)->aes_handle;

    p_aes_handle->p_instance = AES;
    p_aes_handle->init.chaining_mode = AES_CHAININGMODE_ECB;
    p_aes_handle->init.p_init_vector = NULL;
    p_aes_handle->init.p_seed = (uint32_t *)(((aes_instance_t *)ctx->instance)->seed);
    p_aes_handle->init.dpa_mode = DISABLE;

    hal_aes_deinit(p_aes_handle);
    aes_hardware_reset();
    hal_aes_init(p_aes_handle);

    n = *iv_off;

    if( n > 15 )
        return ( -1 );

    while( length-- )
    {
        if( n == 0 )
        {
            if ( HAL_OK != hal_aes_ecb_encrypt(p_aes_handle, (uint32_t *)iv, 16, (uint32_t *)iv, 5000) )
            {
                ret = -1;
                goto exit;
            }
        }
        *output++ =  *input++ ^ iv[n];

        n = ( n + 1 ) & 0x0F;
    }

    *iv_off = n;

exit:
    hal_aes_deinit(p_aes_handle);
    aes_hardware_reset();
    return( ret );
}

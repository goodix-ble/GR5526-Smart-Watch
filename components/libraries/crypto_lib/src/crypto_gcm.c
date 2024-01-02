#include "crypto_gcm.h"
#include "grx_hal.h"

#define GET_UINT32_BE(n,b,i)                            \
{                                                       \
    (n) = ( (uint32_t) (b)[(i)    ] << 24 )             \
        | ( (uint32_t) (b)[(i) + 1] << 16 )             \
        | ( (uint32_t) (b)[(i) + 2] <<  8 )             \
        | ( (uint32_t) (b)[(i) + 3]       );            \
}

#define PUT_UINT32_BE(n,b,i)                            \
{                                                       \
    (b)[(i)    ] = (uint8_t ) ( (n) >> 24 );            \
    (b)[(i) + 1] = (uint8_t) ( (n) >> 16 );             \
    (b)[(i) + 2] = (uint8_t) ( (n) >>  8 );             \
    (b)[(i) + 3] = (uint8_t) ( (n)       );             \
}

typedef struct
{
    uint8_t seed[AES_BLOCK_SIZE];
    aes_handle_t aes_handle;
} aes_instance_t;

/*
 * Shoup's method for multiplication use this table with
 *      last4[x] = x times P^128
 * where x and last4[x] are seen as elements of GF(2^128) as in [MGV]
 */
static const uint64_t last4[16] =
{
    0x0000, 0x1c20, 0x3840, 0x2460,
    0x7080, 0x6ca0, 0x48c0, 0x54e0,
    0xe100, 0xfd20, 0xd940, 0xc560,
    0x9180, 0x8da0, 0xa9c0, 0xb5e0
};

static void aes_hardware_reset(void)
{
    CLEAR_BITS(MCU_SUB->SECURITY_RESET, MCU_SUB_SECURITY_RESET_AES);
    SET_BITS(MCU_SUB->SECURITY_RESET, MCU_SUB_SECURITY_RESET_AES);
}

void crypto_gcm_init( crypto_gcm_context *ctx )
{
    if (ctx == NULL)
    {
        return;
    }

    memset( ctx, 0, sizeof( crypto_gcm_context ) );
    crypto_aes_init( &ctx->cipher_ctx );
}

/*
 * Precompute small multiples of H, that is set
 *      HH[i] || HL[i] = H times i,
 * where i is seen as a field element as in [MGV], ie high-order bits
 * correspond to low powers of P. The result is stored in the same way, that
 * is the high-order bit of HH corresponds to P^0 and the low-order bit of HL
 * corresponds to P^127.
 */
static int gcm_gen_table( crypto_gcm_context *ctx )
{
    int ret, i, j;
    uint64_t hi, lo;
    uint64_t vl, vh;
    unsigned char h[16];

    memset( h, 0, 16 );
    if( ( ret = crypto_aes_crypt_ecb(&ctx->cipher_ctx, AES_ENCRYPT, h, 16, h) ) != 0 )
        return( ret );

    /* pack h as two 64-bits ints, big-endian */
    GET_UINT32_BE( hi, h,  0  );
    GET_UINT32_BE( lo, h,  4  );
    vh = (uint64_t) hi << 32 | lo;

    GET_UINT32_BE( hi, h,  8  );
    GET_UINT32_BE( lo, h,  12 );
    vl = (uint64_t) hi << 32 | lo;

    /* 8 = 1000 corresponds to 1 in GF(2^128) */
    ctx->HL[8] = vl;
    ctx->HH[8] = vh;

    /* 0 corresponds to 0 in GF(2^128) */
    ctx->HH[0] = 0;
    ctx->HL[0] = 0;

    for( i = 4; i > 0; i >>= 1 )
    {
        uint32_t T = ( vl & 1 ) * 0xe1000000U;
        vl  = ( vh << 63 ) | ( vl >> 1 );
        vh  = ( vh >> 1 ) ^ ( (uint64_t) T << 32);

        ctx->HL[i] = vl;
        ctx->HH[i] = vh;
    }

    for( i = 2; i <= 8; i *= 2 )
    {
        uint64_t *HiL = ctx->HL + i, *HiH = ctx->HH + i;
        vh = *HiH;
        vl = *HiL;
        for( j = 1; j < i; j++ )
        {
            HiH[j] = vh ^ ctx->HH[j];
            HiL[j] = vl ^ ctx->HL[j];
        }
    }

    return( 0 );
}

int crypto_gcm_setkey( crypto_gcm_context *ctx, const uint8_t *key, uint32_t keybits )
{
    int ret = 0;

    if (ctx == NULL || key == NULL || ctx->cipher_ctx.instance == NULL)
    {
        return( -1 );
    }

    if( ( ret = crypto_aes_setkey_enc(&ctx->cipher_ctx, (uint8_t *)key, keybits) ) != 0 )
    {
        return( ret );
    }

    if( ( ret = gcm_gen_table( ctx ) ) != 0 )
        return( ret );

    return( 0 );
}

/*
 * Sets output to x times H using the precomputed tables.
 * x and output are seen as elements of GF(2^128) as in [MGV].
 */
static void gcm_mult( crypto_gcm_context *ctx, const uint8_t x[16], uint8_t output[16] )
{
    int i = 0;
    unsigned char lo, hi, rem;
    uint64_t zh, zl;

    lo = x[15] & 0xf;

    zh = ctx->HH[lo];
    zl = ctx->HL[lo];

    for( i = 15; i >= 0; i-- )
    {
        lo = x[i] & 0xf;
        hi = ( x[i] >> 4 ) & 0xf;

        if( i != 15 )
        {
            rem = (unsigned char) zl & 0xf;
            zl = ( zh << 60 ) | ( zl >> 4 );
            zh = ( zh >> 4 );
            zh ^= (uint64_t) last4[rem] << 48;
            zh ^= ctx->HH[lo];
            zl ^= ctx->HL[lo];

        }

        rem = (unsigned char) zl & 0xf;
        zl = ( zh << 60 ) | ( zl >> 4 );
        zh = ( zh >> 4 );
        zh ^= (uint64_t) last4[rem] << 48;
        zh ^= ctx->HH[hi];
        zl ^= ctx->HL[hi];
    }

    PUT_UINT32_BE( zh >> 32, output, 0 );
    PUT_UINT32_BE( zh, output, 4 );
    PUT_UINT32_BE( zl >> 32, output, 8 );
    PUT_UINT32_BE( zl, output, 12 );
}

int crypto_gcm_starts( crypto_gcm_context *ctx, int mode, const uint8_t *iv, uint32_t iv_len, const uint8_t *add, uint32_t add_len )
{
    int ret = 0;
    uint8_t work_buf[16];
    uint32_t i;
    const uint8_t *p;
    uint32_t use_len = 0;

    if (ctx == NULL || iv == NULL || (add == NULL && add_len != 0))
    {
        return( -1 );
    }

    /* IV and AD are limited to 2^64 bits, so 2^61 bytes */
    /* IV is not allowed to be zero length */
    if( iv_len == 0 ||
      ( (uint64_t) iv_len  ) >> 61 != 0 ||
      ( (uint64_t) add_len ) >> 61 != 0 )
    {
        return( -1 );
    }

    memset( ctx->y, 0x00, sizeof(ctx->y) );
    memset( ctx->buf, 0x00, sizeof(ctx->buf) );

    ctx->mode = mode;
    ctx->len = 0;
    ctx->add_len = 0;

    if( iv_len == 12 )
    {
        memcpy( ctx->y, iv, iv_len );
        ctx->y[15] = 1;
    }
    else
    {
        memset( work_buf, 0x00, 16 );
        PUT_UINT32_BE( iv_len * 8, work_buf, 12 );

        p = iv;
        while( iv_len > 0 )
        {
            use_len = ( iv_len < 16 ) ? iv_len : 16;

            for( i = 0; i < use_len; i++ )
                ctx->y[i] ^= p[i];

            gcm_mult( ctx, ctx->y, ctx->y );

            iv_len -= use_len;
            p += use_len;
        }

        for( i = 0; i < 16; i++ )
            ctx->y[i] ^= work_buf[i];

        gcm_mult( ctx, ctx->y, ctx->y );
    }

    if( ( ret = crypto_aes_crypt_ecb(&ctx->cipher_ctx, AES_ENCRYPT, ctx->y, 16, ctx->base_ectr) ) != 0 )
    {
        return( ret );
    }

    ctx->add_len = add_len;
    p = add;
    while( add_len > 0 )
    {
        use_len = ( add_len < 16 ) ? add_len : 16;

        for( i = 0; i < use_len; i++ )
            ctx->buf[i] ^= p[i];

        gcm_mult( ctx, ctx->buf, ctx->buf );

        add_len -= use_len;
        p += use_len;
    }

    return( 0 );
}

int crypto_gcm_update( crypto_gcm_context *ctx, uint32_t length, const uint8_t *input, uint8_t *output )
{
    int ret = 0;
    uint8_t ectr[16];
    uint32_t i;
    const uint8_t *p;
    uint8_t *out_p = output;
    uint32_t use_len = 0;

    if (ctx == NULL || input == NULL || output == NULL)
    {
        return( -1 );
    }

    if( output > input && (size_t) ( output - input ) < length )
        return( -1 );

    /* Total length is restricted to 2^39 - 256 bits, ie 2^36 - 2^5 bytes
     * Also check for possible overflow */
    if( ctx->len + length < ctx->len ||
        (uint64_t) ctx->len + length > 0xFFFFFFFE0ull )
    {
        return( -1 );
    }

    aes_handle_t *p_aes_handle = &((aes_instance_t *)ctx->cipher_ctx.instance)->aes_handle;

    p_aes_handle->p_instance = AES;
    p_aes_handle->init.chaining_mode = AES_CHAININGMODE_ECB;
    p_aes_handle->init.p_init_vector = NULL;
    p_aes_handle->init.p_seed = (uint32_t *)(((aes_instance_t *)ctx->cipher_ctx.instance)->seed);
    p_aes_handle->init.dpa_mode = DISABLE;

    hal_aes_deinit(p_aes_handle);
    aes_hardware_reset();
    hal_aes_init(p_aes_handle);

    ctx->len += length;

    p = input;
    while( length > 0 )
    {
        use_len = ( length < 16 ) ? length : 16;

        for( i = 16; i > 12; i-- )
            if( ++ctx->y[i - 1] != 0 )
                break;

        if ( HAL_OK != hal_aes_ecb_encrypt(p_aes_handle, (uint32_t *)ctx->y, 16, (uint32_t *)ectr, 5000) )
        {
            ret = -1;
            goto exit;
        }

        for( i = 0; i < use_len; i++ )
        {
            if( ctx->mode == AES_DECRYPT )
                ctx->buf[i] ^= p[i];
            out_p[i] = ectr[i] ^ p[i];
            if( ctx->mode == AES_ENCRYPT )
                ctx->buf[i] ^= out_p[i];
        }

        gcm_mult( ctx, ctx->buf, ctx->buf );

        length -= use_len;
        p += use_len;
        out_p += use_len;
    }

exit:
    hal_aes_deinit(p_aes_handle);
    aes_hardware_reset();
    return( ret );
}

int crypto_gcm_finish( crypto_gcm_context *ctx, uint8_t *tag, uint32_t tag_len )
{
    uint8_t work_buf[16];
    uint32_t i;
    uint64_t orig_len;
    uint64_t orig_add_len;

    if (ctx == NULL || tag == NULL)
    {
        return( -1 );
    }

    orig_len = ctx->len * 8;
    orig_add_len = ctx->add_len * 8;

    if( tag_len > 16 || tag_len < 4 )
        return( -1 );

    memcpy( tag, ctx->base_ectr, tag_len );

    if( orig_len || orig_add_len )
    {
        memset( work_buf, 0x00, 16 );

        PUT_UINT32_BE( ( orig_add_len >> 32 ), work_buf, 0  );
        PUT_UINT32_BE( ( orig_add_len       ), work_buf, 4  );
        PUT_UINT32_BE( ( orig_len     >> 32 ), work_buf, 8  );
        PUT_UINT32_BE( ( orig_len           ), work_buf, 12 );

        for( i = 0; i < 16; i++ )
            ctx->buf[i] ^= work_buf[i];

        gcm_mult( ctx, ctx->buf, ctx->buf );

        for( i = 0; i < tag_len; i++ )
            tag[i] ^= ctx->buf[i];
    }

    return( 0 );
}

void crypto_gcm_free( crypto_gcm_context *ctx )
{
    if( ctx == NULL )
        return;
    crypto_aes_free( &ctx->cipher_ctx );
    memset( ctx, 0, sizeof( crypto_gcm_context ) );
}

int crypto_gcm_crypt_and_tag( crypto_gcm_context *ctx, int mode, uint32_t length,
                              const uint8_t *iv, uint32_t iv_len, const uint8_t *add, uint32_t add_len,
                              const uint8_t *input, uint8_t *output, uint32_t tag_len, uint8_t *tag )
{
    int ret = 0;

    if (ctx == NULL || iv == NULL || input == NULL || output == NULL || tag == NULL || (add == NULL && add_len != 0))
    {
        return( -1 );
    }

    if( ( ret = crypto_gcm_starts( ctx, mode, iv, iv_len, add, add_len ) ) != 0 )
        return( ret );

    if( ( ret = crypto_gcm_update( ctx, length, input, output ) ) != 0 )
        return( ret );

    if( ( ret = crypto_gcm_finish( ctx, tag, tag_len ) ) != 0 )
        return( ret );

    return( 0 );
}

int crypto_gcm_auth_decrypt( crypto_gcm_context *ctx, uint32_t length,
                             const uint8_t *iv, uint32_t iv_len, const uint8_t *add, uint32_t add_len,
                             const uint8_t *tag, uint32_t tag_len, const uint8_t *input, uint8_t *output )
{
    int ret = 0;
    uint8_t check_tag[16];
    uint32_t i;
    int diff;

    if (ctx == NULL || iv == NULL || input == NULL || output == NULL || tag == NULL || (add == NULL && add_len != 0))
    {
        return( -1 );
    }

    if( ( ret = crypto_gcm_crypt_and_tag( ctx, AES_DECRYPT, length,
                                   iv, iv_len, add, add_len,
                                   input, output, tag_len, check_tag ) ) != 0 )
    {
        return( ret );
    }

    /* Check tag in "constant-time" */
    for( diff = 0, i = 0; i < tag_len; i++ )
        diff |= tag[i] ^ check_tag[i];

    if( diff != 0 )
    {
        memset( output, 0, length );
        return( -1 );
    }

    return( 0 );
}

#include "grx_sys.h"
#include "bootloader_config.h"

#ifndef SOC_GR5332
#if (BOOTLOADER_SIGN_ENABLE && !BOOTLOADER_BOOT_PORT_ENABLE)

#include "drv_common.h"

#include "rsa.h"
#include "pkc.h"



#define ciL    (sizeof(uint32_t))         /* chars in limb  */
#define biL    (ciL << 3)               /* bits  in limb  */
#define biH    (ciL << 2)               /* half limb size */

#define SHA256_SIZE             32
#define SIGNATURE_SIZE          256
#define ZERO_PADDING_SIZE       8

typedef struct
{
    uint8_t *user_hash;
    uint8_t  hmac_key_type_sel;
    uint8_t  operation_mode;
    uint8_t *hmac_key;
    uint8_t  dma_enable;
    uint8_t  no_padding;
    uint8_t  interrupt_enable;
    uint8_t  continue_input;
} gm_sha_config_t;

/*
 * Count leading zero bits in a given integer
 */
static uint32_t bl_clz( const uint32_t x )
{
    uint32_t j;
    uint32_t mask = (uint32_t) 1 << (biL - 1);

    for( j = 0; j < biL; j++ )
    {
        if( x & mask ) break;

        mask >>= 1;
    }

    return j;
}


/*
 * Return the number of bits
 */
static uint32_t bl_mpi_bitlen(int n, const uint32_t *p )
{
    uint32_t i, j;

    for( i = n - 1; i > 0; i-- )
        if( p[i] != 0 )
            break;

    j = biL - bl_clz( p[i] );

    return( ( i * biL ) + j );
}

extern gm_drv_ret_e gm_drv_sha_encrypt(const gm_sha_config_t *config, const uint8_t *in, uint32_t byte_length, uint8_t *out);

static int bl_rsa_decypt( bl_rsa_context *ctx, uint8_t *sig, uint8_t *output)
{
    int ret;
    int i, j;
    int n = SIGNATURE_SIZE;
    uint32_t tmp[SIGNATURE_SIZE/sizeof(uint32_t)];
    uint32_t word_bit_len = 2048;

    memcpy(tmp, sig, SIGNATURE_SIZE);

    if ((ret = bl_pkc_modular_exponet_65537(word_bit_len, (uint32_t*)sig, (uint32_t*)(ctx->pk->n), (uint32_t*)(ctx->pk->rr), ctx->pk->c,tmp)) < 0)
    {
        return ret;
    }

    //convert to EM format(string data, big endian)
    for( i = SIGNATURE_SIZE - 1, j = 0; n > 0; i--, j++, n-- )
        output[i] = (uint8_t)( tmp[j / ciL] >> ((j % ciL) << 3) );

    return GM_BL_OK;
}

/* Implementation that should never be optimized out by the compiler */
static inline void bl_zeroize( void *v, uint32_t n ) {
    volatile uint8_t *p = (uint8_t*)v; while( n-- ) *p++ = 0;
}

static int mgf_mask( uint8_t *dst, uint32_t dlen, uint8_t *src, uint32_t slen)
{
    int ret;
    uint32_t i, use_len;
    uint8_t counter[4];
    uint8_t *p = dst;
    uint32_t hlen = SHA256_SIZE;
    uint8_t mask[SHA256_SIZE];    
    uint8_t mhash[SHA256_SIZE+4];

    memset( mask, 0, SHA256_SIZE );
    memset( counter, 0, 4 );
    memcpy(mhash, src, SHA256_SIZE);

    /* Generate and apply dbMask */
    while( dlen > 0 )
    {
        use_len = hlen;
        if( dlen < hlen )
            use_len = dlen;

        gm_sha_config_t sha_config = {0};
        sha_config.user_hash = NULL;
        sha_config.dma_enable = 1;
        sha_config.interrupt_enable = 0;
        sha_config.no_padding = 0;

        memcpy((mhash + SHA256_SIZE), counter, 4);
        if ((ret = gm_drv_sha_encrypt(&sha_config, mhash, SHA256_SIZE+4, mask)) < 0)
        {
            return ret;
        }

        for( i = 0; i < use_len; ++i )
            *p++ ^= mask[i];

        counter[3]++;

        dlen -= use_len;
    }

    bl_zeroize( mask, sizeof( mask ) );

    return GM_BL_OK;
}

/*
 * Implementation of the PKCS#1 v2.1 RSASSA-PSS-VERIFY function
 */
int bl_rsa_rsassa_pss_verify(bl_rsa_context *ctx, const uint8_t *hash, const uint8_t *sig)
{
    int ret;
    uint32_t siglen;
    uint8_t *p;
    uint8_t result[SHA256_SIZE];
    uint32_t hlen;
    uint32_t slen, msb;
    uint8_t buf[SIGNATURE_SIZE];
    uint32_t expected_salt_len = SHA256_SIZE;

    siglen = ctx->len;

    if( siglen < 16 || siglen > sizeof( buf ) )
        return GM_BL_ERROR_RSA_BAD_INPUT_DATA;

    if ((ret = bl_rsa_decypt(ctx, (uint8_t*)sig, buf)) < 0)
    {
        return ret;
    }

    p = buf;

    if( buf[siglen - 1] != 0xBC )
    {
        return GM_BL_ERROR_RSA_INVALID_PADDING;
    }

    hlen = 32;
    slen = siglen - hlen - 1; /* Currently length of salt + padding */

    /*
     * Note: EMSA-PSS verification is over the length of N - 1 bits
     */
    msb = bl_mpi_bitlen( ctx->len>>2, (uint32_t*)ctx->pk->n ) - 1;

    /* Compensate for boundary condition when applying mask */
    if( msb % 8 == 0 )
    {
        p++;
        siglen -= 1;
    }

    if( buf[0] >> ( 8 - siglen * 8 + msb ) )
        return GM_BL_ERROR_RSA_BAD_INPUT_DATA;

    if ((ret = mgf_mask( p, siglen - hlen - 1, p + siglen - hlen - 1, hlen)) < 0)
    {
        return ret;
    }

    buf[0] &= 0xFF >> ( siglen * 8 - msb );

    while( p < buf + siglen && *p == 0 )
        p++;

    if((p == buf + siglen) || (*p++ != 0x01))
    {
        return GM_BL_ERROR_RSA_INVALID_PADDING;
    }

    /* Actual salt len */
    slen -= p - buf;

    if(slen != (uint32_t) expected_salt_len )
    {
        return GM_BL_ERROR_RSA_INVALID_PADDING;
    }

    /*
     * Generate H = Hash( M' )
     */

    {
        uint8_t zero_hash_p[ZERO_PADDING_SIZE + 2*SHA256_SIZE];
        gm_sha_config_t sha_config = {0};
        sha_config.user_hash = NULL;
        sha_config.dma_enable = 1;
        sha_config.interrupt_enable = 0;
        sha_config.no_padding = 0;

        memset(zero_hash_p, 0, ZERO_PADDING_SIZE);
        memcpy(zero_hash_p + ZERO_PADDING_SIZE, hash, SHA256_SIZE);
        memcpy(zero_hash_p + ZERO_PADDING_SIZE + SHA256_SIZE, p, SHA256_SIZE);
        if ((ret = gm_drv_sha_encrypt(&sha_config, zero_hash_p, (ZERO_PADDING_SIZE + 2*SHA256_SIZE), result)) < 0)
        {
            return ret;
        }
    }

    if( memcmp( p + slen, result, hlen ) == 0 )
        return GM_BL_OK;
    else
        return GM_BL_ERROR_RSA_VERIFY_FAILED;
}
#endif

#endif

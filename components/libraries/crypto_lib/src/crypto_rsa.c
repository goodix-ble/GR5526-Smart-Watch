#include "crypto_rsa.h"
#include "crypto_rsa_port.h"
#include "crypto_pkc_port.h"

#define ciL (sizeof(uint32_t)) /* chars in limb  */
#define biL (ciL << 3)         /* bits  in limb  */
#define biH (ciL << 2)         /* half limb size */
#define SHA256_SIZE 32
#define ZERO_PADDING_SIZE 8

typedef struct rsa_public_key
{
    uint32_t *n;
    uint32_t e;
    uint32_t *rr;
    uint32_t c;
} rsa_public_key_t;

typedef struct rsa_private_key
{
    uint32_t *d;
} rsa_private_key_t;

typedef struct
{
    algo_rsa_config_t rsa_config;
    uint32_t len; /*!<  size(N) in chars  */
    rsa_public_key_t pk;
    rsa_private_key_t sk;
} rsa_context_t;

static void swap_endian(uint32_t *in, uint32_t len, uint32_t *out)
{
    uint32_t i = 0;

    for (i = 0; i < len; i++)
    {
        out[i] = ((in[i] & 0xFF000000) >> 24) + ((in[i] & 0xFF0000) >> 8)
                + ((in[i] & 0xFF00) << 8) + ((in[i] & 0xFF) << 24);
    }
}

static void swap_order(uint8_t *in, uint32_t len)
{
    int i;
    uint8_t temp;
    for(i = 0; i < len/2; i++)
    {
        temp = in[i];
        in[i] = in[len-i-1];
        in[len-i-1] = temp;
    }
}

/*
 * Count leading zero bits in a given integer
 */
static uint32_t rsa_clz(const uint32_t x)
{
    uint32_t j = 0;
    uint32_t mask = (uint32_t)1 << (biL - 1);

    for (j = 0; j < biL; j++)
    {
        if (x & mask)
            break;

        mask >>= 1;
    }

    return j;
}

/*
 * Return the number of bits
 */
static uint32_t bl_mpi_bitlen(int n, const uint32_t *p)
{
    uint32_t i, j;

    for (i = n - 1; i > 0; i--)
    {
        if (p[i] != 0)
        {
            break;
        }
    }

    j = biL - rsa_clz(p[i]);

    return ((i * biL) + j);
}

static int mgf_mask(uint8_t *dst, uint32_t dlen, uint8_t *src, uint32_t slen)
{
    uint32_t i = 0;
    uint32_t use_len = 0;
    uint8_t counter[4] = { 0 };
    uint8_t *p = dst;
    uint32_t hlen = SHA256_SIZE;
    uint8_t mask[SHA256_SIZE];
    uint8_t mhash[SHA256_SIZE + 4];

    memset(mask, 0, SHA256_SIZE);
    memset(counter, 0, 4);
    memcpy(mhash, src, SHA256_SIZE);

    /* Generate and apply dbMask */
    while (dlen > 0)
    {
        use_len = hlen;
        if (dlen < hlen)
        {
            use_len = dlen;
        }
        memcpy((mhash + SHA256_SIZE), counter, 4);
        hw_rsa_sha(mhash, SHA256_SIZE + 4, mask);

        for (i = 0; i < use_len; ++i)
        {
            *p++ ^= mask[i];
        }

        counter[3]++;

        dlen -= use_len;
    }

    pkc_zeroize(mask, sizeof(mask) / 4);

    return 0;
}

static algo_rsa_ret_e rsa_short_modular_exponent(algo_rsa_config_t *rsa_config,
                                                 uint32_t bits_len,
                                                 uint32_t in_a[],
                                                 algo_rsa_public_exponent_e e,
                                                 uint32_t in_prime[],
                                                 uint32_t r_square[],
                                                 uint32_t constp,
                                                 uint32_t out_result[])
{
    algo_rsa_ret_e err = RSA_OK;
    uint32_t b[RSA_U32_LENGTH] = { 0 };
    uint32_t array_len = bits_len >> 5;

    if (NULL == rsa_config)
    {
        return RSA_ERROR_PARAMETER;
    }

    switch (e)
    {
        case RSA_E_65537:
            b[array_len - 1] = 65537;
            break;

        case RSA_E_17:
            b[array_len - 1] = 17;
            break;

        case RSA_E_3:
            b[array_len - 1] = 3;
            break;

        default:
            return RSA_ERROR_PARAMETER;
    }

    rsa_modular_exponent(rsa_config, bits_len, in_a, b, in_prime, r_square, constp, out_result);

    return err;
}

static algo_rsa_ret_e rsa_private_key_modular_exponent(algo_rsa_config_t *rsa_config,
                                                       uint32_t bits_len,
                                                       uint32_t in_a[],
                                                       uint32_t in_d[],
                                                       algo_rsa_public_exponent_e e,
                                                       uint32_t in_prime[],
                                                       uint32_t r_square[],
                                                       uint32_t constp,
                                                       uint32_t out_result[])
{
    uint32_t r[RSA_U32_LENGTH] = { 0 };
    uint32_t cx[RSA_U32_LENGTH] = { 0 };
    uint32_t r1[RSA_U32_LENGTH] = { 0 };
    uint32_t tmp[RSA_U32_LENGTH] = { 0 };
    uint32_t temp_a[RSA_U32_LENGTH] = { 0 };
    algo_rsa_ret_e err = RSA_OK;
    uint32_t i = 0;
    uint32_t u32_len = bits_len >> 5;

    if (NULL == rsa_config)
    {
        return RSA_ERROR_PARAMETER;
    }

    for (i = 0; i < u32_len; i++)
    {
        r[i] = hw_rsa_rng32();
    }

    // calc cx = in_a * (r ^(-1)) ^ e
    memcpy(r1, r, sizeof(uint32_t) * RSA_U32_LENGTH);
    rsa_modular_inverse(rsa_config, bits_len, r, in_prime, r_square, constp, cx);
    memcpy(r, r1, sizeof(uint32_t) * RSA_U32_LENGTH);

    rsa_short_modular_exponent(rsa_config, bits_len, cx, e, in_prime, r_square, constp, cx);

    hw_rsa_montgomery_mul(rsa_config, bits_len, in_a, cx, in_prime, constp, cx);
    hw_rsa_montgomery_mul(rsa_config, bits_len, r_square, cx, in_prime, constp, cx);

    // calc cx = cx ^ in_d = in_a ^ in_d * (r ^ (-1));
    rsa_modular_exponent(rsa_config, bits_len, cx, in_d, in_prime, r_square, constp, cx);

    // calc tmp = in_a ^ in_d = cx * r;
    hw_rsa_montgomery_mul(rsa_config, bits_len, cx, r, in_prime, constp, cx);
    hw_rsa_montgomery_mul(rsa_config, bits_len, cx, r_square, in_prime, constp, tmp);

    // cx = tmp ^ e
    rsa_short_modular_exponent(rsa_config, bits_len, tmp, e, in_prime, r_square, constp, cx);

    hw_rsa_modular_compare(rsa_config, bits_len, in_a, in_prime, temp_a);

    if (pkc_safe_compare(hw_rsa_rng32, cx, temp_a, u32_len) != 0)
    {
        pkc_zeroize(tmp, u32_len);
        return RSA_ERROR_SIGN_FUNCTION;
    }
    else
    {
        memcpy(out_result, tmp, u32_len * 4);
    }

    return err;
}

static algo_rsa_ret_e rsa_sign_encrypt_em(rsa_context_t *ctx, uint8_t *sig, uint32_t *output)
{
    algo_rsa_ret_e err = RSA_OK;
    uint32_t tmp[RSA_U32_LENGTH] = { 0 };
    uint32_t tmp2[RSA_U32_LENGTH] = { 0 };
    uint32_t tmp1[RSA_U32_LENGTH] = { 0 };
    uint32_t word_bit_len = ctx->len * 8;

    memcpy(tmp, sig, ctx->len);
    memcpy(tmp2, sig, ctx->len);
    hw_rsa_modular_compare(&ctx->rsa_config, word_bit_len, tmp, (uint32_t *)(ctx->pk.n), tmp);
    if (ctx->rsa_config.sign_pkcs_type == RSA_PKCS1_V21)
    {
        if (memcmp(tmp, tmp2, ctx->len) != 0)
        {
            return RSA_ERROR_RANDOM;
        }
    }

#ifdef RSA_OPENSSL_SEQ
    uint32_t i = 0;
    uint32_t j = 0;
    uint32_t n = ctx->len;
    for (i = (ctx->len) - 1, j = 0; n > 0; i--, j++, n--)
    {
        sig[i] = (uint8_t)(tmp[j / ciL] >> ((j % ciL) << 3));
    }
#else
    swap_endian((uint32_t *)sig, ctx->len/4, (uint32_t *)sig);
#endif

    if (ctx->rsa_config.sign_pkcs_type == RSA_PKCS1_V21)
    {
        memcpy(tmp1, sig, ctx->len);
        memcpy(tmp2, sig, ctx->len);
        hw_rsa_modular_compare(&ctx->rsa_config, word_bit_len, tmp1, (uint32_t *)(ctx->pk.n), tmp1);
        if (memcmp(tmp1, tmp2, ctx->len) != 0)
        {
            return RSA_ERROR_RANDOM;
        }
    }

    if ((err = rsa_private_key_modular_exponent(&ctx->rsa_config,
                                                word_bit_len,
                                                (uint32_t *)sig,
                                                ctx->sk.d,
                                                (algo_rsa_public_exponent_e)ctx->pk.e,
                                                (uint32_t *)(ctx->pk.n),
                                                (uint32_t *)(ctx->pk.rr),
                                                ctx->pk.c,
                                                (uint32_t *)output))
        != RSA_OK)
    {
        return err;
    }

    return RSA_OK;
}

static algo_rsa_ret_e rsa_rsassa_pss_sign(rsa_context_t *ctx, const uint8_t *hash, uint8_t *sig)
{
    uint32_t i = 0;
    algo_rsa_ret_e err = RSA_OK;
    uint32_t salt[8] = { 0 };
    unsigned char *p = NULL;
    uint32_t msb = 0, olen = 0, hlen = 32, slen = 32, offset = 0;
    uint8_t zero_hash_p[ZERO_PADDING_SIZE + 2 * SHA256_SIZE];

    do
    {
        err = RSA_OK;
        p = sig;
        msb = 0;
        olen = 0;
        hlen = 32;
        slen = 32;
        offset = 0;
        olen = ctx->len;
        if (olen < hlen + slen + 2)
        {
            err = RSA_ERROR_PARAMETER;
            break;
        }

        memset(sig, 0x0, olen);
        memset(salt, 0x0, 8);
        memset(zero_hash_p, 0, ZERO_PADDING_SIZE + 2 * SHA256_SIZE);
        for (i = 0; i < 8; i++)
        {
            salt[i] = hw_rsa_rng32();
        }

        /* Note: EMSA-PSS encoding is over the length of N - 1 bits */
#ifdef RSA_OPENSSL_SEQ
        msb = bl_mpi_bitlen(ctx->len >> 2, (const uint32_t *)ctx->pk.n) - 1;
#else
        swap_endian((uint32_t *)ctx->pk.n, ctx->len/4, (uint32_t *)ctx->pk.n);
        swap_order((uint8_t *)ctx->pk.n, ctx->len);
        msb = bl_mpi_bitlen(ctx->len >> 2, (const uint32_t *)ctx->pk.n) - 1;
        swap_endian((uint32_t *)ctx->pk.n, ctx->len/4, (uint32_t *)ctx->pk.n);
        swap_order((uint8_t *)ctx->pk.n, ctx->len);
#endif
        p += olen - hlen * 2 - 2;
        *p++ = 0x01;
        memcpy(p, salt, slen);
        p += slen;

        memset(zero_hash_p, 0, ZERO_PADDING_SIZE);
        memcpy(zero_hash_p + ZERO_PADDING_SIZE, hash, SHA256_SIZE);
        memcpy(zero_hash_p + ZERO_PADDING_SIZE + SHA256_SIZE, (uint8_t *)salt, slen);
        hw_rsa_sha(zero_hash_p, (ZERO_PADDING_SIZE + 2 * SHA256_SIZE), p);

        pkc_zeroize((uint32_t *)salt, slen / 4);

        /* Compensate for boundary condition when applying mask */
        if (msb % 8 == 0)
        {
            offset = 1;
        }

        /* maskedDB: Apply dbMask to DB */
        if (olen != hlen)
        {
            if (mgf_mask(sig + offset, olen - hlen - 1 - offset, p, hlen) < 0)
            {
                err = RSA_ERROR_PARAMETER;
                break;
            }
        }

#ifdef RSA_OPENSSL_SEQ
        msb = bl_mpi_bitlen(ctx->len >> 2, (const uint32_t *)ctx->pk.n) - 1;
#else
        swap_endian((uint32_t *)ctx->pk.n, ctx->len/4, (uint32_t *)ctx->pk.n);
        swap_order((uint8_t *)ctx->pk.n, ctx->len);
        msb = bl_mpi_bitlen(ctx->len >> 2, (const uint32_t *)ctx->pk.n) - 1;
        swap_endian((uint32_t *)ctx->pk.n, ctx->len/4, (uint32_t *)ctx->pk.n);
        swap_order((uint8_t *)ctx->pk.n, ctx->len);
#endif
        sig[0] &= 0xFF >> (olen * 8 - msb);

        p += hlen;
        *p++ = 0xBC;

        err = rsa_sign_encrypt_em(ctx, sig, (uint32_t *)sig);

    } while (err == RSA_ERROR_RANDOM);

    if (err != RSA_OK)
    {
        return err;
    }

#ifdef RSA_OPENSSL_SEQ
    // convert signature(number format) to oct string format.
    for (i = 0; i < (uint32_t)(ctx->len / 2); i++)
    {
        uint8_t temp = sig[i];
        sig[i] = sig[ctx->len - 1 - i];
        sig[ctx->len - 1 - i] = temp;
    }
#else
    swap_endian((uint32_t *)sig, ctx->len/4, (uint32_t *)sig);
#endif

    return RSA_OK;
}

static void rsa_compute_montgomery_constant(algo_rsa_config_t *rsa_config,
                                     uint32_t bits_len,
                                     uint32_t in_prime[],
                                     uint32_t r[],
                                     uint32_t r_square[],
                                     uint32_t *constp)
{
    uint32_t i = 0;
    uint32_t m0 = 0;
    uint32_t x = 0;
    if (NULL == rsa_config || NULL == in_prime || NULL == constp)
    {
        return;
    }

    // get R
    pkc_setbit(r, in_prime, bits_len, bits_len);

    // get R^2 mod p
    hw_rsa_modular_left_shift(rsa_config, bits_len, r, in_prime, bits_len, r_square);

    // get constp
    m0 = in_prime[(bits_len >> 5) - 1];
    x = m0;
    x += ((m0 + 2) & 4) << 1;
    for (i = 32; i >= 8; i /= 2)
    {
        x *= 2 - m0 * x;
    }

    *constp = (~x + 1);
}

#ifdef RSA_OPENSSL_SEQ
static void rsa_pkcs15_padding(const uint8_t *hash, uint32_t len, uint8_t *em)
{
    uint8_t sha256_oid[] = { 0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86, 0x48, 0x01,
                             0x65, 0x03, 0x04, 0x02, 0x01, 0x05, 0x00, 0x04, 0x20 };

    if (len < 32 || len > 256)
    {
        return;
    }
    if (NULL == em)
    {
        return;
    }

    memset(em, 0xff, len);
    em[0] = 0x0;
    em[1] = 0x01;
    em[len - 32 - sizeof(sha256_oid) - 1] = 0x0;

    memcpy(&em[len - 32 - sizeof(sha256_oid)], sha256_oid, sizeof(sha256_oid));
    memcpy(&em[len - 32], hash, 32);
}
#else
static void rsa_pkcs15_padding(const uint8_t *hash, uint32_t len, uint8_t *em)
{
    size_t nb_pad = len;
    uint8_t *p = em;
    size_t hashlen = 32;

    if( nb_pad < hashlen )
        return;

    nb_pad -= hashlen;

    /* Need space for signature header and padding delimiter (3 bytes),
     * and 8 bytes for the minimal padding */
    if( nb_pad < 3 + 8 )
        return;
    nb_pad -= 3;

    /* Now nb_pad is the amount of memory to be filled
     * with padding, and at least 8 bytes long. */

    /* Write signature header and padding */
    *p++ = 0;
    *p++ = 0x1;
    memset( p, 0xFF, nb_pad );
    p += nb_pad;
    *p++ = 0;
    memcpy( p, hash, hashlen );

    return;
}
#endif

static algo_rsa_ret_e rsa_rsassa_pkcs1v15_sign(rsa_context_t *ctx, const uint8_t *hash, uint8_t *sig)
{
    algo_rsa_ret_e err = RSA_OK;
    uint8_t *em = sig;

    rsa_pkcs15_padding(hash, ctx->len, em);

    if ((err = rsa_sign_encrypt_em(ctx, em, (uint32_t *)sig)) < 0)
    {
        return err;
    }

#ifdef RSA_OPENSSL_SEQ
    uint32_t i = 0;
    // convert signature(number format) to oct string format.
    for (i = 0; i < ctx->len / 2; i++)
    {

        uint8_t temp = sig[i];
        sig[i] = sig[ctx->len - 1 - i];
        sig[ctx->len - 1 - i] = temp;
    }
#else
    swap_endian((uint32_t *)sig, ctx->len/4, (uint32_t *)sig);
#endif

    return RSA_OK;
}

algo_rsa_ret_e crypto_rsa_pkcs1_sign(algo_rsa_config_t *rsa_config,
                                     uint32_t bits_len,
                                     uint8_t *message,
                                     uint32_t message_len,
                                     uint32_t private_key[],
                                     algo_rsa_public_exponent_e e,
                                     uint32_t in_prime[],
                                     uint32_t r_square[],
                                     uint32_t constp,
                                     uint8_t sig[],
                                     uint8_t is_hash)
{
    algo_rsa_ret_e ret = RSA_OK;
    rsa_context_t ctx = { 0 };
    uint32_t rr[RSA_U32_LENGTH] = { 0 };
    uint8_t hash[32] = { 0 };
    if (is_hash == 1)
    {
        memcpy(hash, message, message_len);
    }
    else
    {
        hw_rsa_sha(message, message_len, hash);
    }

    if (NULL == rsa_config || NULL == private_key || NULL == in_prime || bits_len < 1024 || bits_len > 2048)
    {
        return RSA_ERROR_PARAMETER;
    }

    ctx.len = bits_len / 8;
    ctx.sk.d = private_key;
    ctx.pk.n = in_prime;
    ctx.pk.rr = r_square;
    ctx.pk.c = constp;
    ctx.pk.e = e;
    memcpy(&ctx.rsa_config, rsa_config, sizeof(algo_rsa_config_t));

    // compute montgomery constants
    if (NULL == r_square)
    {
        uint32_t r[RSA_U32_LENGTH] = { 0 };
        ctx.pk.rr = &rr[0];
        rsa_compute_montgomery_constant(&ctx.rsa_config, bits_len, in_prime, r, (uint32_t *)ctx.pk.rr, &ctx.pk.c);
    }

    switch (e)
    {
        case RSA_E_65537:
        case RSA_E_17:
        case RSA_E_3:
            ctx.pk.e = e;
            break;

        default:
            ret = RSA_ERROR_PARAMETER;
            break;
    }

    switch (rsa_config->sign_pkcs_type)
    {
        case RSA_PKCS1_V21:
            ret = rsa_rsassa_pss_sign(&ctx, hash, sig);
            rsa_config->sign_pkcs_type = RSA_PKCS1_V21;
            break;

        case RSA_PKCS1_V15:
            ret = rsa_rsassa_pkcs1v15_sign(&ctx, hash, sig);
            rsa_config->sign_pkcs_type = RSA_PKCS1_V15;
            break;

        default:
            ret = RSA_ERROR_PARAMETER;
            break;
    }

    return ret;
}

static algo_rsa_ret_e rsa_verify_decrypt_em(rsa_context_t *ctx, uint8_t *sig, uint8_t *output)
{
#ifdef RSA_OPENSSL_SEQ
    algo_rsa_ret_e ret;
    int i, j;
    int n = ctx->len;
    uint32_t tmp[RSA_U32_LENGTH] = { 0 };
    uint32_t word_bit_len = ctx->len * 8;

    memcpy(tmp, sig, ctx->len);

    if ((ret = rsa_short_modular_exponent(&ctx->rsa_config,
                                          word_bit_len,
                                          (uint32_t *)sig,
                                          (algo_rsa_public_exponent_e)ctx->pk.e,
                                          (uint32_t *)(ctx->pk.n),
                                          (uint32_t *)(ctx->pk.rr),
                                          ctx->pk.c,
                                          tmp))
        != RSA_OK)
    {

        return ret;
    }

    // convert to EM format(string data, big endian)
    for (i = ctx->len - 1, j = 0; n > 0; i--, j++, n--)
    {
        output[i] = (uint8_t)(tmp[j / ciL] >> ((j % ciL) << 3));
    }
#else
    algo_rsa_ret_e ret;
    uint32_t word_bit_len = ctx->len * 8;

    if ((ret = rsa_short_modular_exponent(&ctx->rsa_config,
                                          word_bit_len,
                                          (uint32_t *)sig,
                                          (algo_rsa_public_exponent_e)ctx->pk.e,
                                          (uint32_t *)(ctx->pk.n),
                                          (uint32_t *)(ctx->pk.rr),
                                          ctx->pk.c,
                                          (uint32_t *)output))
        != RSA_OK)
    {

        return ret;
    }
    swap_endian((uint32_t *)output, ctx->len/4, (uint32_t *)output);
#endif

    return RSA_OK;
}

/*
 * Implementation of the PKCS#1 v2.1 RSASSA-PSS-VERIFY function
 */
static algo_rsa_ret_e rsa_rsassa_pss_verify(rsa_context_t *ctx, const uint8_t *hash, const uint8_t *sig)
{
    algo_rsa_ret_e ret = RSA_OK;
    uint32_t siglen = 32;
    uint8_t *p = NULL;
    uint8_t result[SHA256_SIZE] = { 0 };
    uint8_t zeros[8] = { 0 };
    uint32_t hlen = 0;
    uint32_t slen = 0, msb = 0;
    uint8_t buf[RSA_U32_LENGTH * 4] = { 0 };
    uint32_t expected_salt_len = SHA256_SIZE;
    uint32_t sig_number[RSA_U32_LENGTH] = { 0 };
    uint8_t zero_hash_p[ZERO_PADDING_SIZE + 2 * SHA256_SIZE] = { 0 };

    siglen = ctx->len;
    pkc_read_oct_string(sig_number, (uint8_t *)sig, siglen);

    if (siglen < 16 || siglen > sizeof(buf))
    {
        return RSA_ERROR_PARAMETER;
    }

    if ((ret = rsa_verify_decrypt_em(ctx, (uint8_t *)sig_number, buf)) < 0)
    {
        return ret;
    }

    p = buf;

    if (buf[siglen - 1] != 0xBC)
        return RSA_ERROR_VERIFY_FUNCTION;

    hlen = 32;
    slen = siglen - hlen - 1; /* Currently length of salt + padding */
    memset(zeros, 0, 8);

    /*
     * Note: EMSA-PSS verification is over the length of N - 1 bits
     */
#ifdef RSA_OPENSSL_SEQ
    msb = bl_mpi_bitlen(ctx->len >> 2, (const uint32_t *)ctx->pk.n) - 1;
#else
    swap_endian((uint32_t *)ctx->pk.n, ctx->len/4, (uint32_t *)ctx->pk.n);
    swap_order((uint8_t *)ctx->pk.n, ctx->len);
    msb = bl_mpi_bitlen(ctx->len >> 2, (const uint32_t *)ctx->pk.n) - 1;
    swap_endian((uint32_t *)ctx->pk.n, ctx->len/4, (uint32_t *)ctx->pk.n);
    swap_order((uint8_t *)ctx->pk.n, ctx->len);
#endif

    if (buf[0] >> (8 - siglen * 8 + msb))
    {
        return RSA_ERROR_VERIFY_FUNCTION;
    }

    /* Compensate for boundary condition when applying mask */
    if (msb % 8 == 0)
    {
        p++;
        siglen -= 1;
    }

    if (mgf_mask(p, siglen - hlen - 1, p + siglen - hlen - 1, hlen) < 0)
    {
        return RSA_ERROR_VERIFY_FUNCTION;
    }

    buf[0] &= 0xFF >> (siglen * 8 - msb);

    while (p < buf + siglen)
    {
        if (*p != 0)
        {
            break;
        }
        p++;
    }

    if ((p == buf + siglen) || (*p++ != 0x01))
    {
        return RSA_ERROR_VERIFY_FUNCTION;
    }

    /* Actual salt len */
    slen -= p - buf;

    if (slen != (uint32_t)expected_salt_len)
    {
        return RSA_ERROR_VERIFY_FUNCTION;
    }

    /*
     * Generate H = Hash( M' )
     */
    memset(zero_hash_p, 0, ZERO_PADDING_SIZE);
    memcpy(zero_hash_p + ZERO_PADDING_SIZE, hash, SHA256_SIZE);
    memcpy(zero_hash_p + ZERO_PADDING_SIZE + SHA256_SIZE, p, SHA256_SIZE);
    hw_rsa_sha(zero_hash_p, (ZERO_PADDING_SIZE + 2 * SHA256_SIZE), result);

#if 0
    if (memcmp(p + slen + 3, result, hlen - 4) == 0)
#else
    if( memcmp( p + slen, result, hlen ) == 0 )
#endif
    {
        return RSA_OK;
    }

    return RSA_ERROR_VERIFY_FUNCTION;
}

static algo_rsa_ret_e rsa_rsassa_pkcs1v15_verify(rsa_context_t *ctx, const uint8_t *hash, const uint8_t *sig)
{
    algo_rsa_ret_e err = RSA_OK;
    uint8_t em_expect[RSA_U32_LENGTH * 4] = { 0 };
    uint8_t em_act[RSA_U32_LENGTH * 4] = { 0 };
    uint32_t sig_number[RSA_U32_LENGTH] = { 0 };

    pkc_read_oct_string(sig_number, (uint8_t *)sig, ctx->len);

    rsa_pkcs15_padding(hash, ctx->len, em_expect);

#ifdef RSA_OPENSSL_SEQ
    uint32_t tmp[RSA_U32_LENGTH] = { 0 };
    uint32_t word_bit_len = ctx->len * 8;
    uint32_t i = 0;
    uint32_t j = 0;
    uint32_t n = ctx->len;
    uint32_t temp = 0;

    memcpy(tmp, em_expect, ctx->len);
    hw_rsa_modular_compare(&ctx->rsa_config, word_bit_len, tmp, (uint32_t *)(ctx->pk.n), tmp);

    for (i = (ctx->len) - 1, j = 0; n > 0; i--, j++, n--)
    {
        em_expect[i] = (uint8_t)(tmp[j / ciL] >> ((j % ciL) << 3));
    }
    memcpy(em_expect, tmp, ctx->len);
    swap_endian((uint32_t *)em_expect, ctx->len/4, (uint32_t *)em_expect);

    hw_rsa_modular_compare(&ctx->rsa_config, word_bit_len, (uint32_t *)em_expect,
                            (uint32_t *)(ctx->pk.n), (uint32_t *)em_expect);

    for (i = 0; i < ctx->len / 2; i++)
    {
        temp = em_expect[i];
        em_expect[i] = em_expect[ctx->len - 1 - i];
        em_expect[ctx->len - 1 - i] = temp;
    }
#endif

    if ((err = rsa_verify_decrypt_em(ctx, (uint8_t *)sig_number, em_act)) < 0)
    {
        return err;
    }

#ifdef RSA_OPENSSL_SEQ
    if (memcmp(em_expect, em_act, ctx->len) != 0)
    {
        return RSA_ERROR_VERIFY_FUNCTION;
    }
#else
    if( ( err = (algo_rsa_ret_e)rsa_safer_memcmp( em_expect, em_act, ctx->len ) ) != 0 )
    {
        return RSA_ERROR_VERIFY_FUNCTION;
    }
#endif

    return err;
}

algo_rsa_ret_e crypto_rsa_pkcs1_verify(algo_rsa_config_t *rsa_config,
                                       uint32_t bits_len,
                                       uint8_t *message,
                                       uint32_t message_len,
                                       algo_rsa_public_exponent_e e,
                                       uint32_t in_prime[],
                                       uint32_t r_square[],
                                       uint32_t constp,
                                       const uint8_t sig[],
                                       uint8_t is_hash)
{
    algo_rsa_ret_e ret = RSA_OK;
    rsa_context_t ctx = { 0 };
    uint32_t rr[RSA_U32_LENGTH] = { 0 };
    uint8_t hash[32] = { 0 };
    if (is_hash == 1)
    {
        memcpy(hash, message, message_len);
    }
    else
    {
        hw_rsa_sha(message, message_len, hash);
    }

    if (NULL == rsa_config || NULL == in_prime || bits_len < 1024 || bits_len > 2048)
    {
        return RSA_ERROR_PARAMETER;
    }

    ctx.len = bits_len / 8;
    ctx.pk.n = in_prime;
    ctx.pk.rr = r_square;
    ctx.pk.c = constp;
    memcpy(&ctx.rsa_config, rsa_config, sizeof(algo_rsa_config_t));

    // compute montgomery constants
    if (NULL == r_square)
    {
        uint32_t r[RSA_U32_LENGTH] = { 0 };
        ctx.pk.rr = &rr[0];
        rsa_compute_montgomery_constant(rsa_config, bits_len, in_prime, r, (uint32_t *)ctx.pk.rr, &ctx.pk.c);
    }

    switch (e)
    {
        case RSA_E_65537:
        case RSA_E_17:
        case RSA_E_3:
            ctx.pk.e = e;
            ret = RSA_OK;
            break;

        default:
            ret = RSA_ERROR_PARAMETER;
            break;
    }

    if (0 == ret)
    {

        if (rsa_config->sign_pkcs_type == RSA_PKCS1_V21)
        {
            ret = rsa_rsassa_pss_verify(&ctx, (const uint8_t *)hash, (const uint8_t *)sig);
        }
        else if (rsa_config->sign_pkcs_type == RSA_PKCS1_V15)
        {
            ret = rsa_rsassa_pkcs1v15_verify(&ctx, (const uint8_t *)hash, (const uint8_t *)sig);
        }
    }

    return ret;
}

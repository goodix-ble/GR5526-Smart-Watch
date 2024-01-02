#include "crypto_sha256.h"
#include "grx_hal.h"
//#include "mbedtls/sha256.h"

#define crypto_malloc     malloc
#define crypto_free       free

/**
 * @brief This defines the structure of sha256 hmac context.
 */
typedef struct crypto_sha256_hmac_context
{
//    uint8_t pad[2 * SHA256_BLOCK_SIZE];
//    mbedtls_sha256_context m_ctx;
    hmac_handle_t hmac_handle;
} crypto_sha256_hmac_context_t;

/*
 * SHA256 context init
 */
void crypto_sha256_init(crypto_sha256_context *ctx)
{
    if (ctx == NULL)
    {
        return;
    }
    memset(ctx, 0x0, sizeof(crypto_sha256_context));
    ctx->instance = (crypto_sha256_hmac_context_t *)crypto_malloc(sizeof(crypto_sha256_hmac_context_t));
    if (NULL == ctx->instance)
    {
        return;
    }
    memset(ctx->instance, 0x0, sizeof(crypto_sha256_hmac_context_t));
}

/*
 * SHA256 context free
 */
void crypto_sha256_free(crypto_sha256_context *ctx)
{
//    crypto_sha256_hmac_context_t *hmac_ctx = NULL;

    if (ctx == NULL)
    {
        return;
    }
    if (ctx->input != NULL)
    {
        crypto_free(ctx->input);
        ctx->input = NULL;
    }
//    hmac_ctx = (crypto_sha256_hmac_context_t *)ctx->hmac_ctx;
//    if (NULL != hmac_ctx->hmac_handle.init.p_key)
//    {
//        crypto_free(hmac_ctx->hmac_handle.init.p_key);
//        hmac_ctx->hmac_handle.init.p_key = NULL;
//    }
    if (NULL != ctx->instance)
    {
        crypto_free(ctx->instance);
        ctx->instance = NULL;
    }
    memset(ctx, 0x0, sizeof(crypto_sha256_context));
}

/*
 * SHA256 context clone
 */
void crypto_sha256_clone(crypto_sha256_context *dst, const crypto_sha256_context *src)
{
    if (dst == NULL || src == NULL)
    {
        return;
    }

    *dst = *src;
}

/*
 * SHA256 context setup
 */
int crypto_sha256_starts(crypto_sha256_context *ctx)
{
    crypto_sha256_hmac_context_t *hmac_ctx = NULL;

    if (ctx == NULL)
    {
        return -1;
    }

    if (ctx->input != NULL)
    {
        crypto_free(ctx->input);
        ctx->input = NULL;
        ctx->block_size = 0;
        ctx->total = 0;
    }

    hmac_ctx = (crypto_sha256_hmac_context_t *)ctx->instance;
    hmac_ctx->hmac_handle.p_instance = HMAC;
    hmac_ctx->hmac_handle.init.mode = HMAC_MODE_SHA;
    hmac_ctx->hmac_handle.init.p_key = NULL;
    hmac_ctx->hmac_handle.init.p_user_hash = NULL;
    hmac_ctx->hmac_handle.init.dpa_mode = DISABLE;

    return 0;
}

/*
 * SHA256 update
 */
int crypto_sha256_update(crypto_sha256_context *ctx, const uint8_t *input, size_t ilen)
{
    int total_len = ilen + ctx->total;
    int i = 0;
    uint8_t *new_input = NULL;

    if (ctx == NULL || input == NULL)
    {
        return -1;
    }

    if (ctx->input == NULL || total_len > ctx->block_size)
    {
        ctx->block_size = (total_len / SHA256_BLOCK_SIZE + 1) * SHA256_BLOCK_SIZE;
        new_input = crypto_malloc(ctx->block_size);
        if (new_input == NULL)
        {
            return -1;
        }
        memset(new_input, 0, ctx->block_size);
        for (i = 0; i < ctx->total; i++)
        {
            new_input[i] = ctx->input[i];
        }
        for (int j = 0; j < ilen; j++)
        {
            new_input[i] = input[j];
            i++;
        }
        if (ctx->input != NULL)
        {
            crypto_free(ctx->input);
        }
        ctx->input = new_input;
    }
    else
    {
        i = ctx->total;
        for (int j = 0; j < ilen; j++)
        {
            ctx->input[i] = input[j];
            i++;
        }
    }

    ctx->total = total_len;
    return 0;
}

static int crypto_internal_sha256_finish(crypto_sha256_context *ctx, uint8_t *output)
{
    crypto_sha256_hmac_context_t *hmac_ctx = NULL;
    int ret = 0;
    uint8_t buf[SHA256_SIZE] = {0};

    if (ctx == NULL || output == NULL)
    {
        return -1;
    }

    hmac_ctx = (crypto_sha256_hmac_context_t *)ctx->instance;
    hal_hmac_deinit(&hmac_ctx->hmac_handle);
    hal_hmac_init(&hmac_ctx->hmac_handle);

    ret = hal_hmac_sha256_digest(&hmac_ctx->hmac_handle, (uint32_t *)ctx->input, ctx->total, (uint32_t *)buf, 5000);
    if (0 != ret)
    {
        return -1;
    }

    memcpy(output, buf, SHA256_SIZE);
    hal_hmac_deinit(&hmac_ctx->hmac_handle);

    return ret;
}

/*
 * SHA256 finish
 */
int crypto_sha256_finish(crypto_sha256_context *ctx, uint8_t output[32])
{
    if (ctx == NULL || output == NULL)
    {
        return -1;
    }

    return crypto_internal_sha256_finish(ctx, output);
}

/*
 * SHA256 compute
 */
int crypto_sha256(const uint8_t *input, size_t ilen, uint8_t output[32])
{
    int ret = 0;
    crypto_sha256_context ctx = { 0 };

    if (NULL == input)
    {
        return (-1);
    }

    crypto_sha256_init(&ctx);

    if ((ret = crypto_sha256_starts(&ctx)) != 0)
        goto exit;

    if ((ret = crypto_sha256_update(&ctx, input, ilen)) != 0)
        goto exit;

    if ((ret = crypto_sha256_finish(&ctx, output)) != 0)
        goto exit;

exit:
    crypto_sha256_free(&ctx);

    return (ret);
}

/*********************************************************************/

int crypto_hmac_sha256_starts(crypto_sha256_context *ctx, const uint8_t *key, size_t keylen)
{
    crypto_sha256_hmac_context_t *hmac_ctx = NULL;

    if (ctx == NULL || key == NULL || keylen != HMAC_SHA256_KEY_SIZE)
    {
        return -1;
    }

    if (ctx->input != NULL)
    {
        crypto_free(ctx->input);
        ctx->input = NULL;
        ctx->block_size = 0;
        ctx->total = 0;
    }

    hmac_ctx = (crypto_sha256_hmac_context_t *)ctx->instance;
    hmac_ctx->hmac_handle.p_instance = HMAC;
    hmac_ctx->hmac_handle.init.mode = HMAC_MODE_HMAC;
    hmac_ctx->hmac_handle.init.key_fetch_type = HAL_HMAC_KEYTYPE_MCU;
    hmac_ctx->hmac_handle.init.p_key = (uint32_t *)key;
    hmac_ctx->hmac_handle.init.p_user_hash = NULL;
    hmac_ctx->hmac_handle.init.dpa_mode = DISABLE;

    return 0;
}

int crypto_hmac_sha256_update(crypto_sha256_context *ctx, const uint8_t *input, size_t ilen)
{
    return (crypto_sha256_update(ctx, input, ilen));
}

int crypto_hmac_sha256_finish(crypto_sha256_context *ctx, uint8_t output[32])
{
    if (ctx == NULL || output == NULL)
    {
        return -1;
    }

    return crypto_internal_sha256_finish(ctx, output);
}

int crypto_hmac_sha256(const uint8_t *key, size_t keylen, const uint8_t *input, size_t ilen, uint8_t output[32])
{
    int ret = 0;
    crypto_sha256_context ctx = { 0 };

    if (NULL == input)
    {
        return (-1);
    }

    crypto_sha256_init(&ctx);

    if ((ret = crypto_hmac_sha256_starts(&ctx, key, keylen)) != 0)
        goto exit;

    if ((ret = crypto_hmac_sha256_update(&ctx, input, ilen)) != 0)
        goto exit;

    if ((ret = crypto_hmac_sha256_finish(&ctx, output)) != 0)
        goto exit;

exit:
    crypto_sha256_free(&ctx);

    return (ret);
}

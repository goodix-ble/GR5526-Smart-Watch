#include "crypto_pkc_port.h"

static void pkc_random_delay(pkc_rng_func_ptr rng32_func)
{
    volatile uint32_t random = rng32_func();
    random &= 0x1F;

    while (random > 0)
    {
        random--;
    }
}

void pkc_setbit(uint32_t q[], uint32_t in_prime[], uint32_t k, uint32_t bits_len)
{
    uint32_t i = 0;
    uint32_t n_u32 = 0;                        // the n-th u32
    uint32_t m_bit = 0;                        // the m-th bit
    uint32_t prime_u32_length = bits_len >> 5; //(bits_len >> 5);

    if (k > bits_len)
    {
        return;
    }
    if (bits_len == k)
    {
        for (i = 0; i < prime_u32_length; i++)
        {
            q[i] = 0xFFFFFFFF ^ in_prime[i];
        }
        q[prime_u32_length - 1]++;
        return;
    }
    else
    {
        // 0 <= k < 2 ^ bits_len
        for (i = 0; i < prime_u32_length; i++)
        {
            q[i] = 0;
        }

        n_u32 = k >> 5;   // k / 32;
        m_bit = k & 0x1F; // k % 32
        q[n_u32] = 1 << m_bit;
    }

    return;
}

void pkc_zeroize(void *v, uint32_t u32_len)
{
    volatile uint32_t *p = (uint32_t *)v;
    uint32_t i = 0;
    for (i = 0; i < u32_len; i++)
    {
        p[i] = 0;
    }
    return;
}

// ret: a > b ret 1; a == b ret 0; a < b ret -1;
int32_t pkc_number_compare_to_const(uint32_t a[], uint32_t b, uint32_t bits_len)
{
    uint32_t i = 0;
    int32_t pkc_u32_len = bits_len >> 5; //(bits_len >> 5);

    if (a[0] > b)
    {
        return 1;
    }

    for (i = pkc_u32_len - 1; i >= 1; i--)
    {
        if (a[i] > 0)
        {
            return 1;
        }
    }

    if (a[0] == b)
    {
        return 0;
    }

    return -1;
}

int32_t pkc_number_compare(uint32_t a[], uint32_t b[], uint32_t bits_len)
{
    int32_t i = 0;
    uint32_t number_u32_len = bits_len >> 5; //(bits_len >> 5);

    for (i = number_u32_len - 1; i >= 0; i--)
    {
        if (a[i] > b[i])
        {
            return 1;
        }
        else if (a[i] < b[i])
        {
            return -1;
        }
    }

    return 0;
}

uint8_t pkc_safe_compare(pkc_rng_func_ptr rng32, uint32_t *src, uint32_t *dest, uint32_t u32_len)
{
    uint8_t ret = 1;

    ret = pkc_number_compare(src, dest, u32_len * 32);
    if (0 != ret)
    {
        return ret;
    }

    pkc_random_delay(rng32);

    ret = pkc_number_compare(src, dest, u32_len * 32);

    if (0 != ret)
    {
        return ret;
    }

    pkc_random_delay(rng32);

    return pkc_number_compare(src, dest, u32_len * 32);
}

void pkc_read_oct_string(uint32_t *number, uint8_t *buffer, uint32_t byte_size)
{
    uint32_t i = 0;
    uint8_t *x = (uint8_t *)number;

    for (i = 0; i < byte_size; i++)
    {
        x[i] = buffer[byte_size - 1 - i];
    }
}

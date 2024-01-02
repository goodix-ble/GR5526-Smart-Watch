#include "crypto_ecc_port.h"
#include "crypto_pkc.h"
#include "crypto_pkc_port.h"
#include "crypto_sha256.h"
#include "grx_hal.h"

uint32_t hw_ecc_rng32(void)
{
    uint32_t rng32;
    rng_handle_t g_rng_handle;

    g_rng_handle.p_instance = RNG;
    g_rng_handle.init.seed_mode  = RNG_SEED_FR0_S0;
    g_rng_handle.init.lfsr_mode  = RNG_LFSR_MODE_59BIT;
    g_rng_handle.init.out_mode   = RNG_OUTPUT_FR0_S0;
    g_rng_handle.init.post_mode  = RNG_POST_PRO_NOT;

    hal_rng_deinit(&g_rng_handle);
    hal_rng_init(&g_rng_handle);
    hal_rng_generate_random_number(&g_rng_handle, NULL, &rng32);
    hal_rng_deinit(&g_rng_handle);
    return rng32;
}

void hw_ecc_point_mul(algo_ecc_config_t *ecc_calc_options,
                      uint32_t k[ECC_U32_LENGTH],
                      algo_ecc_point_t *Q,
                      algo_ecc_point_t *result)
{

    hal_status_t err = HAL_OK;
    ecc_curve_init_t *ecc_curve = (ecc_curve_init_t *)ecc_calc_options->curve;

    hal_set_curve(ecc_curve);

    err = hal_pkc_ecc_point_mul_handle(k, (ecc_point_t *)Q, (ecc_point_t *)result);

    if (HAL_OK != err)
    {
//        printf("hw_ecc_point_mul error %d\n", __LINE__);
    }
}

void hw_ecc_sha(const uint8_t *message, uint32_t message_byte_length, uint8_t output[32])
{
    crypto_sha256_context config = { 0 };
    crypto_sha256_init(&config);
    crypto_sha256_starts(&config);
    crypto_sha256_update(&config, (uint8_t *)message, message_byte_length);
    crypto_sha256_finish(&config, output);
    crypto_sha256_free(&config);
}

void hw_ecc_modular_compare(algo_ecc_config_t *ecc_calc_options,
                            uint32_t in_a[],
                            uint32_t in_prime[],
                            uint32_t result[])
{

    hal_status_t err = HAL_OK;

    err = hal_pkc_modular_compare_handle(256, in_a, in_prime, result);

    if (HAL_OK != err)
    {
//        printf("hw_ecc_modular_compare error %d\n", __LINE__);
    }
}

// modular inverse
// output is a^(-1)
void ecc_modular_inverse(
    algo_ecc_config_t *ecc_config, uint32_t in_a[], uint32_t in_prime[], uint32_t r_square[], uint32_t constq, uint32_t out_a_inverse[])
{
    // check if input a = 0
    if (pkc_number_compare_to_const(in_a, 0, 256) == 0)
    {
        return;
    }

    if ((in_prime[0] & 1) == 0)
    {
        return;
    }

    hw_ecc_montgomery_inverse(ecc_config, in_a, in_prime, constq, out_a_inverse);
}

void hw_ecc_montgomery_inverse(
    algo_ecc_config_t *ecc_calc_options, uint32_t in_a[], uint32_t in_prime[], uint32_t constp, uint32_t out_x[])
{

    hal_status_t err = HAL_OK;

    err = hal_pkc_montgomery_inverse(256, in_a, in_prime, constp, out_x);

    if (HAL_OK != err)
    {
//        printf("hw_ecc_montgomery_inverse error %d\n", __LINE__);
    }
}

void hw_ecc_modular_sub(
    algo_ecc_config_t *ecc_calc_options, uint32_t in_a[], uint32_t in_b[], uint32_t in_prime[], uint32_t result[])
{
    hal_status_t err = HAL_OK;

    err = hal_pkc_modular_sub_handle(256, in_a, in_b, in_prime, result);

    if (HAL_OK != err)
    {
//        printf("hw_ecc_modular_sub error %d\n", __LINE__);
    }
}

void hw_ecc_montgomery_mul(algo_ecc_config_t *ecc_calc_options,
                           uint32_t in_a[],
                           uint32_t in_b[],
                           uint32_t in_prime[],
                           uint32_t constp,
                           uint32_t result[])
{
    hal_status_t err = HAL_OK;

    err = hal_pkc_montgomery_mul(256, in_a, in_b, in_prime, constp, result);

    if (HAL_OK != err)
    {
//        printf("hw_eccmontgomery_mul error %d\n", __LINE__);
    }
}

void hw_ecc_modular_add(
    algo_ecc_config_t *ecc_calc_options, uint32_t in_a[], uint32_t in_b[], uint32_t in_prime[], uint32_t result[])
{
    hal_status_t err = HAL_OK;

    err = hal_pkc_modular_add_handle(256, in_a, in_b, in_prime, result);

    if (HAL_OK != err)
    {
//        printf("hw_ecc_modular_add error %d\n", __LINE__);
    }
}

// c = a * b mod prime
void ecc_modular_multiply(algo_ecc_config_t *ecc_config,
                          uint32_t in_a[],
                          uint32_t in_b[],
                          uint32_t in_prime[],
                          uint32_t r_square[],
                          uint32_t constq,
                          uint32_t out_result[])
{
    //algo_ecc_ret_e err = ECC_OK;
    uint32_t tmp[64] = { 0 };

    hw_ecc_montgomery_mul(ecc_config, in_a, in_b, in_prime, constq, tmp);

    hw_ecc_montgomery_mul(ecc_config, tmp, r_square, in_prime, constq, out_result);
}

/**
 *  \brief check if point is an infinite point
 *
 *  \param[in] point  input point.
 *
 *  \return
 *      \li 0  Not inifinte point
 *      \li 1  is infinite point
 */
uint32_t ecc_is_infinite_point(algo_ecc_point_t *point)
{

    uint32_t i = 0;

    if (NULL == point)
    {
        return (uint32_t)ECC_ERROR_PARAMETER;
    }

    for (i = 0; i < ECC_U32_LENGTH; i++)
    {

        if (point->x[i] != 0 || point->y[i] != 0)
        {
            return 0;
        }
    }

    return 1;
}

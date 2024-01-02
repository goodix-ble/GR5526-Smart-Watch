#include "crypto_ecc.h"
#include "crypto_pkc.h"
#include "crypto_pkc_port.h"
#include "crypto_ecc_port.h"

static bool ecc_is_zero_value(uint32_t *value, uint32_t len)
{
    for (int i = 0; i < len; i++)
    {
        if (value[i])
        {
            return false;
        }
    }

    return true;
}

static void ecc_init_config(algo_ecc_config_t *ecc_config, algo_ecc_curve_type_e curve)
{
    if (NULL == ecc_config)
    {
        return;
    }

    ecc_config->curve = (algo_ecc_curve_parameter_t *)pkc_set_curve_p256_params(curve);
}

void crypto_ecc_ecdsa_init(algo_ecc_ecdsa_config_t *ecdsa_data, algo_ecc_curve_type_e curve)
{
    if (NULL == ecdsa_data)
    {
        return;
    }

    pkc_zeroize((void *)(&ecdsa_data->our_public_point), sizeof(algo_ecc_point_t) / 4);
    pkc_zeroize((void *)(&ecdsa_data->our_secret_value), ECC_U32_LENGTH);
    pkc_zeroize((void *)(&ecdsa_data->k), ECC_U32_LENGTH);

    ecc_init_config(&ecdsa_data->calc_options, curve);
}

static void ecc_gen_rng(algo_ecc_config_t *ecc_config, uint32_t out[])
{
    if (NULL == ecc_config)
    {
        return;
    }

    uint32_t i = 0;
    uint32_t *n = (uint32_t *)(ecc_config->curve->n);

    for (i = 0; i < ECC_U32_LENGTH; i++)
    {
        out[i] = hw_ecc_rng32();
    }

#if 0
    if (out[ECC_U32_LENGTH - 1] >= n[ECC_U32_LENGTH - 1])
    {
        out[ECC_U32_LENGTH - 1] = n[ECC_U32_LENGTH - 1] - 1;
    }

    out[0] |= 2;
#else
    // since the n defined as big endian
    if (out[0] >= n[0])
    {
        out[0] = n[0] - 1;
    }

    out[ECC_U32_LENGTH - 1] |= 2;
#endif
}

algo_ecc_ret_e crypto_ecc_ecdsa_gen_secret_and_public(algo_ecc_ecdsa_config_t *ecdsa_data)
{

    uint32_t i = 0;
    uint8_t ret = 0;

    if (NULL == ecdsa_data || NULL == ecdsa_data->calc_options.curve)
    {
        return ECC_ERROR_PARAMETER;
    }

    for (i = 0; i < ECC_U32_LENGTH; i++)
    {
        if (ecdsa_data->our_secret_value[i] == 0)
        {
            ecdsa_data->our_secret_value[i] = hw_ecc_rng32();
            ret = 1;
        }
    }

    // make sure 1 < our_secret_value < n
    if (ret == 1)
    {
        ecc_gen_rng(&ecdsa_data->calc_options, ecdsa_data->our_secret_value);
    }

    hw_ecc_point_mul(&ecdsa_data->calc_options, ecdsa_data->our_secret_value, NULL, &ecdsa_data->our_public_point);
    return ECC_OK;
}

// s = (e + r1*d1 + r2 * d2)* ke_inverse + (r1 * d2 + r2 * d1) * ke_inverse
static void secure_calc_signature_s(algo_ecc_config_t *ecc_config,
                                    uint32_t random_r1[ECC_U32_LENGTH],
                                    uint32_t random_d1[ECC_U32_LENGTH],
                                    uint32_t ke[ECC_U32_LENGTH],
                                    uint32_t private_key[ECC_U32_LENGTH],
                                    uint32_t e[ECC_U32_LENGTH],
                                    uint32_t r[ECC_U32_LENGTH],
                                    algo_ecc_curve_parameter_t *ecc_curve,
                                    uint32_t out_sig_s[ECC_U32_LENGTH])
{
    if (NULL == ecc_config || NULL == random_r1 || NULL == random_d1 || NULL == ke || NULL == private_key || NULL == e || NULL == r || NULL == out_sig_s || NULL == ecc_curve)
    {
        return;
    }

    uint32_t ke_inverse[ECC_U32_LENGTH] = { 0 };
    uint32_t r2[ECC_U32_LENGTH] = { 0 };
    uint32_t d2[ECC_U32_LENGTH] = { 0 };
    uint32_t tmp1[ECC_U32_LENGTH] = { 0 };
    uint32_t tmp2[ECC_U32_LENGTH] = { 0 };
    uint32_t tmp_rd1[ECC_U32_LENGTH] = { 0 };
    uint32_t tmp_rd2[ECC_U32_LENGTH] = { 0 };

    ecc_modular_inverse(ecc_config, ke, ecc_curve->n, ecc_curve->n_r_square, ecc_curve->constn, ke_inverse);

    // calc d2 = d - d1
    hw_ecc_modular_sub(ecc_config, private_key, random_d1, ecc_curve->n, d2);

    // calc r2 = r - r1
    hw_ecc_modular_sub(ecc_config, r, random_r1, ecc_curve->n, r2);

    // calc tmp_rd1 = r1 * d1
    hw_ecc_montgomery_mul(ecc_config, random_r1, random_d1, ecc_curve->n, ecc_curve->constn, tmp_rd1);
    hw_ecc_montgomery_mul(ecc_config, tmp_rd1, ecc_curve->n_r_square, ecc_curve->n, ecc_curve->constn, tmp_rd1);

    // calc tmp_rd2 = r2 * d2
    hw_ecc_montgomery_mul(ecc_config, r2, d2, ecc_curve->n, ecc_curve->constn, tmp_rd2);
    hw_ecc_montgomery_mul(ecc_config, tmp_rd2, ecc_curve->n_r_square, ecc_curve->n, ecc_curve->constn, tmp_rd2);

    // calc tmp1 = e + r1d1 + r2d2
    hw_ecc_modular_add(ecc_config, e, tmp_rd1, ecc_curve->n, tmp1);
    hw_ecc_modular_add(ecc_config, tmp1, tmp_rd2, ecc_curve->n, tmp1);

    // calc tmp1 = (e + r1d1 + r2d2) * ke_inverse
    hw_ecc_montgomery_mul(ecc_config, tmp1, ke_inverse, ecc_curve->n, ecc_curve->constn, tmp1);
    hw_ecc_montgomery_mul(ecc_config, tmp1, ecc_curve->n_r_square, ecc_curve->n, ecc_curve->constn, tmp1);

    ////Next Step.
    // calc tmp_rd1 = r1 * d2
    hw_ecc_montgomery_mul(ecc_config, random_r1, d2, ecc_curve->n, ecc_curve->constn, tmp_rd1);
    hw_ecc_montgomery_mul(ecc_config, tmp_rd1, ecc_curve->n_r_square, ecc_curve->n, ecc_curve->constn, tmp_rd1);

    // calc tmp_rd2 = r2 * d1
    hw_ecc_montgomery_mul(ecc_config, r2, random_d1, ecc_curve->n, ecc_curve->constn, tmp_rd2);
    hw_ecc_montgomery_mul(ecc_config, tmp_rd2, ecc_curve->n_r_square, ecc_curve->n, ecc_curve->constn, tmp_rd2);

    // calc tmp2 = r1d1 + r2d2
    hw_ecc_modular_add(ecc_config, tmp_rd1, tmp_rd2, ecc_curve->n, tmp2);

    // calc tmp2 = (r1d2 + r2d1) * ke_inverse
    hw_ecc_montgomery_mul(ecc_config, tmp2, ke_inverse, ecc_curve->n, ecc_curve->constn, tmp2);
    hw_ecc_montgomery_mul(ecc_config, tmp2, ecc_curve->n_r_square, ecc_curve->n, ecc_curve->constn, tmp2);

    ////Final Step.
    // calc s = tmp1 + tmp2
    hw_ecc_modular_add(ecc_config, tmp1, tmp2, ecc_curve->n, out_sig_s);
}

static algo_ecc_ret_e ecc_ecdsa_sign_with_hash(algo_ecc_ecdsa_config_t *ecdsa_calc_options,
                                        uint8_t *message_hash,
                                        uint32_t out_signiture_r[ECC_U32_LENGTH],
                                        uint32_t out_signiture_s[ECC_U32_LENGTH])
{
    algo_ecc_ret_e err = ECC_OK;
    uint32_t ke[ECC_U32_LENGTH] = { 0 };
    algo_ecc_point_t R = { { 0 }, { 0 } };
    uint32_t message_hash_bignumber[ECC_U32_LENGTH] = { 0 };
    uint32_t s1[ECC_U32_LENGTH] = { 0 };
    uint32_t s2[ECC_U32_LENGTH] = { 0 };
    uint32_t ke_backup[ECC_U32_LENGTH] = { 0 };
    algo_ecc_curve_parameter_t *ecc_curve = NULL;
    uint32_t temp = 0;

    if (NULL == ecdsa_calc_options)
    {
        return ECC_ERROR_PARAMETER;
    }

    ecc_curve = (ecdsa_calc_options->calc_options.curve);

    if (NULL == ecc_curve || NULL == message_hash || NULL == out_signiture_r || NULL == out_signiture_s)
    {
        return ECC_ERROR_PARAMETER;
    }

    pkc_read_oct_string((uint32_t *)message_hash_bignumber, message_hash, ECC_U32_LENGTH * 4);

    for (int i = 0; i < (ECC_U32_LENGTH/2); i++)
    {
        temp = message_hash_bignumber[i];
        message_hash_bignumber[i] = message_hash_bignumber[ECC_U32_LENGTH - 1 - i];
        message_hash_bignumber[ECC_U32_LENGTH - 1 - i] = temp;
    }

    do
    {
        uint32_t random_r1[ECC_U32_LENGTH] = { 0 };
        uint32_t random_d1[ECC_U32_LENGTH] = { 0 };
        uint32_t random_r2[ECC_U32_LENGTH] = { 0 };
        uint32_t random_d2[ECC_U32_LENGTH] = { 0 };

        if (ecc_is_zero_value(ecdsa_calc_options->k, ECC_U32_LENGTH))
        {
            // not indicate a k value, we random generate
            ecc_gen_rng(&ecdsa_calc_options->calc_options, ecdsa_calc_options->k);
            memcpy(ke, ecdsa_calc_options->k, ECC_U32_LENGTH * 4);
        }

        memcpy(ke_backup, ke, ECC_U32_LENGTH * 4);

        // Step 1 Gen ke = SHA(message_hash || private_key || "ECDSA_SIGN_KEY");

        ecc_gen_rng(&ecdsa_calc_options->calc_options, random_r1);
        ecc_gen_rng(&ecdsa_calc_options->calc_options, random_d1);
        ecc_gen_rng(&ecdsa_calc_options->calc_options, random_r2);
        ecc_gen_rng(&ecdsa_calc_options->calc_options, random_d2);

        // step 2. calc R = ke * G
        hw_ecc_point_mul(&ecdsa_calc_options->calc_options, ke, NULL, &R);

        hw_ecc_modular_compare(&ecdsa_calc_options->calc_options, R.x, ecc_curve->n, out_signiture_r); // print_ecc_data("R.x",R.x);

        hw_ecc_modular_compare(
            &ecdsa_calc_options->calc_options, message_hash_bignumber, ecc_curve->n, message_hash_bignumber);

        secure_calc_signature_s(
            &ecdsa_calc_options->calc_options, random_r1, random_d1, ke, ecdsa_calc_options->our_secret_value, message_hash_bignumber, out_signiture_r, ecc_curve, s1);

        memcpy(ke, ke_backup, ECC_U32_LENGTH * 4);
        secure_calc_signature_s(
            &ecdsa_calc_options->calc_options, random_r2, random_d2, ke, ecdsa_calc_options->our_secret_value, message_hash_bignumber, out_signiture_r, ecc_curve, s2);

        if (pkc_safe_compare(hw_ecc_rng32, s1, s2, ECC_U32_LENGTH) == 0)
        {
            err = ECC_OK;
        }
        else
        {
            err = ECC_ERROR_SIGN;
            break;
        }
        // compare s, 0;
        if (pkc_number_compare_to_const(s1, 0, 256) == 0)
        {
            err = ECC_ERROR_SIGN;
            break;
        }

        memcpy(out_signiture_s, s1, ECC_U32_LENGTH * 4);

    } while (0);

    if (ECC_OK != err)
    {
        pkc_zeroize((void *)out_signiture_s, ECC_U32_LENGTH);
        pkc_zeroize((void *)out_signiture_r, ECC_U32_LENGTH);
        pkc_zeroize((void *)ke, ECC_U32_LENGTH);
        pkc_zeroize((void *)ke_backup, ECC_U32_LENGTH);
    }

    pkc_zeroize((void *)s1, ECC_U32_LENGTH);
    pkc_zeroize((void *)s2, ECC_U32_LENGTH);

    return err;
}

algo_ecc_ret_e crypto_ecc_ecdsa_sign(algo_ecc_ecdsa_config_t *ecdsa_calc_options,
                              uint8_t hash_func,
                              uint8_t *message,
                              uint32_t message_byte_length,
                              uint32_t out_signiture_r[ECC_U32_LENGTH],
                              uint32_t out_signiture_s[ECC_U32_LENGTH])
{

    uint8_t hash[ECC_U32_LENGTH * 4] = { 0 };

    if (NULL == ecdsa_calc_options || NULL == message)
    {
        return ECC_ERROR_PARAMETER;
    }

    switch (hash_func)
    {
        case ECC_HASH_NONE:
            memcpy(hash, message, message_byte_length);
            break;
        case ECC_HASH_SHA_256:
            hw_ecc_sha(message, message_byte_length, (uint8_t *)hash);
            break;
        default:
            return ECC_ERROR_PARAMETER;
    }

    return ecc_ecdsa_sign_with_hash(ecdsa_calc_options, hash, out_signiture_r, out_signiture_s);
}

static algo_ecc_ret_e ecc_is_point_on_curve(algo_ecc_config_t *ecc_calc_options, algo_ecc_point_t *Q)
{

    uint32_t xr[ECC_U32_LENGTH] = { 0 };
    uint32_t yr[ECC_U32_LENGTH] = { 0 };
    uint32_t x3[ECC_U32_LENGTH] = { 0 };
    uint32_t ax[ECC_U32_LENGTH] = { 0 };
    uint32_t x3_ax_b[ECC_U32_LENGTH] = { 0 };
    uint32_t y2[ECC_U32_LENGTH] = { 0 };
    algo_ecc_curve_parameter_t *ecc_curve = NULL;

    if (NULL == ecc_calc_options && NULL == ecc_calc_options->curve && NULL == Q)
    {
        return ECC_ERROR_PARAMETER;
    }
    ecc_curve = ecc_calc_options->curve;

    // To montgomery field
    hw_ecc_montgomery_mul(ecc_calc_options, Q->x, ecc_curve->p_r_square, ecc_curve->p, ecc_curve->constp, xr);
    hw_ecc_montgomery_mul(ecc_calc_options, Q->y, ecc_curve->p_r_square, ecc_curve->p, ecc_curve->constp, yr);

    // calc x^3 + ax + b
    hw_ecc_montgomery_mul(ecc_calc_options, xr, xr, ecc_curve->p, ecc_curve->constp, x3);
    hw_ecc_montgomery_mul(ecc_calc_options, x3, xr, ecc_curve->p, ecc_curve->constp, x3);
    hw_ecc_montgomery_mul(ecc_calc_options, xr, ecc_curve->a, ecc_curve->p, ecc_curve->constp, ax);
    hw_ecc_modular_add(ecc_calc_options, x3, ax, ecc_curve->p, x3_ax_b);
    hw_ecc_modular_add(ecc_calc_options, x3_ax_b, ecc_curve->b, ecc_curve->p, x3_ax_b);

    // calc y^2
    hw_ecc_montgomery_mul(ecc_calc_options, yr, yr, ecc_curve->p, ecc_curve->constp, y2);

    if (pkc_number_compare(x3_ax_b, y2, 256) == 0)
    {
        return ECC_OK;
    }

    return ECC_ERROR_POINT_NOT_ON_CURVE;
}

static void ecc_point_addition(algo_ecc_config_t *ecc_config,
                        algo_ecc_point_t *pointA,
                        algo_ecc_point_t *pointB,
                        algo_ecc_point_t *out_result)
{

    uint32_t s[ECC_U32_LENGTH] = { 0 };
    uint32_t temp[ECC_U32_LENGTH] = { 0 };
    uint32_t temp2[ECC_U32_LENGTH] = { 0 };
    uint32_t temp3[ECC_U32_LENGTH] = { 0 };

    if (NULL == ecc_config)
    {
        return;
    }

    algo_ecc_curve_parameter_t *ecc_curve = (algo_ecc_curve_parameter_t *)(ecc_config->curve);

    // check input parameters
    if (NULL == pointA || NULL == pointB || NULL == out_result || NULL == ecc_curve)
    {
        return;
    }

    if (ecc_is_infinite_point(pointA))
    {
        memcpy(out_result, pointB, sizeof(algo_ecc_point_t));
        return;
    }

    if (ecc_is_infinite_point(pointB))
    {
        memcpy(out_result, pointA, sizeof(algo_ecc_point_t));
        return;
    }

    if (memcmp(pointA, pointB, sizeof(algo_ecc_point_t)) == 0)
    {
        if (pkc_number_compare_to_const(pointA->y, 0, 256) == 0)
        {
            // point of infinite
            memset(out_result, 0, sizeof(algo_ecc_point_t));
            return;
        }
        else
        {
            // s = (3*(x1^2) + a)/(2*y1) mod p
            uint32_t tmp[ECC_U32_LENGTH] = { 0 };
            uint32_t tmp_backup[ECC_U32_LENGTH] = { 0 };
            uint32_t tmp2[ECC_U32_LENGTH] = { 0 };
            uint32_t tmp3[ECC_U32_LENGTH] = { 0 };
            uint32_t number2[ECC_U32_LENGTH] = { 2, 0, 0, 0, 0, 0, 0, 0 };
            uint32_t number3[ECC_U32_LENGTH] = { 3, 0, 0, 0, 0, 0, 0, 0 };

            // calc tmp = x1 ^ 2
            ecc_modular_multiply(
                ecc_config, pointA->x, pointA->x, ecc_curve->p, ecc_curve->p_r_square, ecc_curve->constp, tmp);

            // calc tmp2 = 3 * tmp = 3*(x1^2)
            ecc_modular_multiply(
                ecc_config, number3, tmp, ecc_curve->p, ecc_curve->p_r_square, ecc_curve->constp, tmp2);

            // calc tmp3 = tmp2 + a = tmp2 - 3 = 3*(x1^2) - 3;
            hw_ecc_modular_sub(ecc_config, tmp2, number3, ecc_curve->p, tmp3);

            // calc tmp = 2* y1;
            ecc_modular_multiply(
                ecc_config, number2, pointA->y, ecc_curve->p, ecc_curve->p_r_square, ecc_curve->constp, tmp);

            // calc tmp2 = tmp ^(-1) = (2y1)^(-1);
            memcpy(tmp_backup, tmp, 32);
            ecc_modular_inverse(ecc_config, tmp, ecc_curve->p, ecc_curve->p_r_square, ecc_curve->constp, tmp2);
            memcpy(tmp, tmp_backup, 32);

            // calc s = tmp3 * tmp2;
            ecc_modular_multiply(ecc_config, tmp3, tmp2, ecc_curve->p, ecc_curve->p_r_square, ecc_curve->constp, s);
        }
    }
    else
    {
        // point addition: s = (y2-y1)/(x2-x1)
        if (memcmp(pointA->x, pointB->x, sizeof(algo_ecc_point_t)) == 0)
        {
            // return point of infinite
            memset(out_result, 0, sizeof(algo_ecc_point_t));
            return;
        }
        else
        {
            // s = (y2-y1)/(x2-x1) mod p
            uint32_t tmp[ECC_U32_LENGTH] = { 0 };
            uint32_t tmp2[ECC_U32_LENGTH] = { 0 };
            uint32_t tmp3[ECC_U32_LENGTH] = { 0 };

            // tmp = y2-y1;
            hw_ecc_modular_sub(ecc_config, pointB->y, pointA->y, ecc_curve->p, tmp);

            // tmp2 = x2-x1;
            hw_ecc_modular_sub(ecc_config, pointB->x, pointA->x, ecc_curve->p, tmp2);

            // tmp3 = tmp2 ^(-1) = (x2-x1)^(-1);
            ecc_modular_inverse(ecc_config, tmp2, ecc_curve->p, ecc_curve->p_r_square, ecc_curve->constp, tmp3);

            // s = tmp * tmp3 = (y2-y1)*((x2-x1)^(-1))
            ecc_modular_multiply(ecc_config, tmp, tmp3, ecc_curve->p, ecc_curve->p_r_square, ecc_curve->constp, s);
        }
    }

    // calc (x3,y3)
    // x3 = s^2 - x1 - x2;
    // y3 = s(x1-x3) - y1
    // temp = s^2
    ecc_modular_multiply(ecc_config, s, s, ecc_curve->p, ecc_curve->p_r_square, ecc_curve->constp, temp);

    // temp2 = temp - x1;
    hw_ecc_modular_sub(ecc_config, temp, pointA->x, ecc_curve->p, temp2);

    // x3 = temp2 - x2;
    hw_ecc_modular_sub(ecc_config, temp2, pointB->x, ecc_curve->p, out_result->x);

    // temp3 = x1 - x3;
    hw_ecc_modular_sub(ecc_config, pointA->x, out_result->x, ecc_curve->p, temp3);

    // temp2 = s * temp3 = s(x1-x3);
    ecc_modular_multiply(ecc_config, s, temp3, ecc_curve->p, ecc_curve->p_r_square, ecc_curve->constp, temp2);

    // y3 = temp2 - y1 = s(x1-x3) - y1;
    hw_ecc_modular_sub(ecc_config, temp2, pointA->y, ecc_curve->p, out_result->y);
}

static algo_ecc_ret_e ecc_ecdsa_verify_with_hash(algo_ecc_ecdsa_config_t *ecdsa_calc_options,
                                   uint8_t *message_hash,
                                   uint32_t in_signiture_r[ECC_U32_LENGTH],
                                   uint32_t in_signiture_s[ECC_U32_LENGTH])
{
    algo_ecc_ret_e err = ECC_OK;
    uint32_t t2[ECC_U32_LENGTH] = { 0 };
    uint32_t tmp_rd1[ECC_U32_LENGTH] = { 0 };
    uint32_t u1[ECC_U32_LENGTH] = { 0 };
    uint32_t u2[ECC_U32_LENGTH] = { 0 };
    uint32_t r[ECC_U32_LENGTH] = { 0 };
    uint32_t s[ECC_U32_LENGTH] = { 0 };
    uint32_t x[ECC_U32_LENGTH] = { 0 };
    uint32_t message_hash_bignumber[ECC_U32_LENGTH] = { 0 };
    algo_ecc_point_t A = { { 0 }, { 0 } };
    algo_ecc_point_t u1G = { { 0 }, { 0 } };
    algo_ecc_point_t u2Q = { { 0 }, { 0 } };
    uint32_t temp = 0;

    if (NULL == ecdsa_calc_options)
    {
        return ECC_ERROR_PARAMETER;
    }
    algo_ecc_curve_parameter_t *ecc_curve = ecdsa_calc_options->calc_options.curve;

    if (NULL == ecc_curve || NULL == in_signiture_r || NULL == in_signiture_s)
    {
        return ECC_ERROR_PARAMETER;
    }
    pkc_read_oct_string((uint32_t *)message_hash_bignumber, (uint8_t *)message_hash, ECC_U32_LENGTH * 4);
    for (int i = 0; i < (ECC_U32_LENGTH/2); i++)
    {
        temp = message_hash_bignumber[i];
        message_hash_bignumber[i] = message_hash_bignumber[ECC_U32_LENGTH - 1 - i];
        message_hash_bignumber[ECC_U32_LENGTH - 1 - i] = temp;
    }

    do
    {
        hw_ecc_modular_compare(&ecdsa_calc_options->calc_options, in_signiture_r, ecc_curve->n, r);
        hw_ecc_modular_compare(&ecdsa_calc_options->calc_options, in_signiture_s, ecc_curve->n, s);

        if (pkc_number_compare_to_const(in_signiture_r, 0, 256) == 0)
        {
            err = ECC_ERROR_VERIFY;
            break;
        }

        hw_ecc_modular_compare(
            &ecdsa_calc_options->calc_options, message_hash_bignumber, ecc_curve->n, tmp_rd1);

        ecc_modular_inverse(&ecdsa_calc_options->calc_options, s, ecc_curve->n, ecc_curve->n_r_square, ecc_curve->constn, t2);

        ecc_modular_multiply(
            &ecdsa_calc_options->calc_options, tmp_rd1, t2, ecc_curve->n, ecc_curve->n_r_square, ecc_curve->constn, u1);

        ecc_modular_multiply(&ecdsa_calc_options->calc_options, r, t2, ecc_curve->n, ecc_curve->n_r_square, ecc_curve->constn, u2);

        hw_ecc_point_mul(&ecdsa_calc_options->calc_options, u1, NULL, &u1G);

        if (ecc_is_point_on_curve(&ecdsa_calc_options->calc_options, &u1G) != ECC_OK)
        {

            return ECC_ERROR_POINT_NOT_ON_CURVE;
        }

        hw_ecc_point_mul(&ecdsa_calc_options->calc_options, u2, &ecdsa_calc_options->our_public_point, &u2Q);

        if (ecc_is_point_on_curve(&ecdsa_calc_options->calc_options, &u2Q) != ECC_OK)
        {
            return ECC_ERROR_POINT_NOT_ON_CURVE;
        }

        ecc_point_addition(&ecdsa_calc_options->calc_options, &u1G, &u2Q, &A);

        hw_ecc_modular_compare(&ecdsa_calc_options->calc_options, A.x, ecc_curve->n, x);

        if (ecc_is_point_on_curve(&ecdsa_calc_options->calc_options, &A) != ECC_OK)
        {
            return ECC_ERROR_POINT_NOT_ON_CURVE;
        }

        if (pkc_number_compare(x, r, 256) != 0)
        {
            return ECC_ERROR_VERIFY;
        }
    } while (0);

    return err;
}

algo_ecc_ret_e crypto_ecc_ecdsa_verify(algo_ecc_ecdsa_config_t *ecdsa_calc_options,
                         uint8_t hash_func,
                         uint8_t *message,
                         uint32_t message_byte_length,
                         uint32_t in_signiture_r[ECC_U32_LENGTH],
                         uint32_t in_signiture_s[ECC_U32_LENGTH])
{

    uint8_t e[ECC_U32_LENGTH * 4] = { 0 };
    if (NULL == ecdsa_calc_options || NULL == message)
    {
        return ECC_ERROR_PARAMETER;
    }

    switch (hash_func)
    {
        case ECC_HASH_NONE:
            memcpy(e, message, message_byte_length);
            break;
        case ECC_HASH_SHA_256:
            hw_ecc_sha(message, message_byte_length, (uint8_t *)e);
            break;
        default:
            return ECC_ERROR_PARAMETER;
    }

    return ecc_ecdsa_verify_with_hash(ecdsa_calc_options, e, in_signiture_r, in_signiture_s);
}

void crypto_ecc_ecdh_init(algo_ecc_ecdh_config_t *ecdh_data, algo_ecc_curve_type_e curve)
{
    if (NULL == ecdh_data)
    {
        return;
    }

    pkc_zeroize((void *)(&ecdh_data->our_public_point), sizeof(algo_ecc_point_t) / 4);
    pkc_zeroize((void *)(&ecdh_data->peer_public_point), sizeof(algo_ecc_point_t) / 4);
    pkc_zeroize((void *)(&ecdh_data->shared_point), sizeof(algo_ecc_point_t) / 4);
    pkc_zeroize((void *)(&ecdh_data->our_secret_value), ECC_U32_LENGTH);

    ecc_init_config(&ecdh_data->calc_options, curve);
}

algo_ecc_ret_e crypto_ecc_ecdh_gen_secret_and_public(algo_ecc_ecdh_config_t *ecdh_data)
{
    if (NULL == ecdh_data || NULL == ecdh_data->calc_options.curve)
    {
        return ECC_ERROR_PARAMETER;
    }

    // make sure 1 < our_secret_value < n
    ecc_gen_rng(&ecdh_data->calc_options, ecdh_data->our_secret_value);

    hw_ecc_point_mul(
        &ecdh_data->calc_options, ecdh_data->our_secret_value, NULL, &ecdh_data->our_public_point);

    return ECC_OK;
}

algo_ecc_ret_e crypto_ecc_ecdh_compute_shared(algo_ecc_ecdh_config_t *ecdh_data, algo_ecc_point_t *qb)
{

    if (NULL == ecdh_data || NULL == ecdh_data->calc_options.curve || NULL == qb)
    {
        return ECC_ERROR_PARAMETER;
    }

    if (ecc_is_point_on_curve(&ecdh_data->calc_options, qb) != ECC_OK)
    {
        return ECC_ERROR_POINT_NOT_ON_CURVE;
    }

    memcpy(&ecdh_data->peer_public_point, qb, sizeof(algo_ecc_point_t));

    hw_ecc_point_mul(
        &ecdh_data->calc_options, ecdh_data->our_secret_value, &ecdh_data->peer_public_point, &ecdh_data->shared_point);
    return ECC_OK;
}

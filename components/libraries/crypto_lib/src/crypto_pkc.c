#include "crypto_pkc.h"

static pkc_handle_t pkc_handle = { 0 };
static ecc_curve_init_t ECC_CurveInitStruct = LL_ECC_CURVE_DEFAULT_CONFIG;

static void pkc_set_curve_secp256r1(void)
{
    uint32_t A[ECC_U32_LENGTH] = {0xFFFFFFFC, 0x00000004, 0x00000000, 0x00000000, 0x00000003, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFC};
    uint32_t B[ECC_U32_LENGTH] = {0xDC30061D, 0x04874834, 0xE5A220AB, 0xF7212ED6, 0xACF005CD, 0x78843090, 0xD89CDF62, 0x29C4BDDF};
    uint32_t P[ECC_U32_LENGTH] = {0xFFFFFFFF, 0x00000001, 0x00000000, 0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
    uint32_t PRSquare[ECC_U32_LENGTH] = {0x00000004, 0xFFFFFFFD, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFB, 0xFFFFFFFF, 0x00000000, 0x00000003};
    uint32_t ConstP = 1;
    uint32_t N[ECC_U32_LENGTH] = {0xFFFFFFFF, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xBCE6FAAD, 0xA7179E84, 0xF3B9CAC2, 0xFC632551};
    uint32_t NRSquare[ECC_U32_LENGTH] = {0x66E12D94, 0xF3D95620, 0x2845B239, 0x2B6BEC59, 0x4699799C, 0x49BD6FA6, 0x83244C95, 0xBE79EEA2};
    uint32_t ConstN = 0xEE00BC4F;
    uint32_t H = 1;
    uint32_t GX[ECC_U32_LENGTH] = {0x6B17D1F2, 0xE12C4247, 0xF8BCE6E5, 0x63A440F2, 0x77037D81, 0x2DEB33A0, 0xF4A13945, 0xD898C296};
    uint32_t GY[ECC_U32_LENGTH] = {0x4FE342E2, 0xFE1A7F9B, 0x8EE7EB4A, 0x7C0F9E16, 0x2BCE3357, 0x6B315ECE, 0xCBB64068, 0x37BF51F5};

    memcpy(ECC_CurveInitStruct.A, A, sizeof(uint32_t) * ECC_U32_LENGTH);
    memcpy(ECC_CurveInitStruct.B, B, sizeof(uint32_t) * ECC_U32_LENGTH);
    memcpy(ECC_CurveInitStruct.P, P, sizeof(uint32_t) * ECC_U32_LENGTH);
    memcpy(ECC_CurveInitStruct.PRSquare, PRSquare, sizeof(uint32_t) * ECC_U32_LENGTH);
    ECC_CurveInitStruct.ConstP = ConstP;
    memcpy(ECC_CurveInitStruct.N, N, sizeof(uint32_t) * ECC_U32_LENGTH);
    memcpy(ECC_CurveInitStruct.NRSquare, NRSquare, sizeof(uint32_t) * ECC_U32_LENGTH);
    ECC_CurveInitStruct.ConstN = ConstN;
    ECC_CurveInitStruct.H = H;
    memcpy(ECC_CurveInitStruct.G.X, GX, sizeof(uint32_t) * ECC_U32_LENGTH);
    memcpy(ECC_CurveInitStruct.G.Y, GY, sizeof(uint32_t) * ECC_U32_LENGTH);
}

static void pkc_set_curve_secp256k1(void)
{
    uint32_t A[ECC_U32_LENGTH] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};
    uint32_t B[ECC_U32_LENGTH] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000007, 0x00001AB7};
    uint32_t P[ECC_U32_LENGTH] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFC2F};
    uint32_t PRSquare[ECC_U32_LENGTH] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000001, 0x000007A2, 0x000E90A1};
    uint32_t ConstP = 0xD2253531;
    uint32_t N[ECC_U32_LENGTH] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xBAAEDCE6, 0xAF48A03B, 0xBFD25E8C, 0xD0364141};
    uint32_t NRSquare[ECC_U32_LENGTH] = {0x9D671CD5, 0x81C69BC5, 0xE697F5E4, 0x5BCD07C6, 0x741496C2, 0x0E7CF878, 0x896CF214, 0x67D7D140};
    uint32_t ConstN = 0x5588B13F;
    uint32_t H = 1;
    uint32_t GX[ECC_U32_LENGTH] = {0x79BE667E, 0xF9DCBBAC, 0x55A06295, 0xCE870B07, 0x029BFCDB, 0x2DCE28D9, 0x59F2815B, 0x16F81798};
    uint32_t GY[ECC_U32_LENGTH] = {0x483ADA77, 0x26A3C465, 0x5DA4FBFC, 0x0E1108A8, 0xFD17B448, 0xA6855419, 0x9C47D08F, 0xFB10D4B8};

    memcpy(ECC_CurveInitStruct.A, A, sizeof(uint32_t) * ECC_U32_LENGTH);
    memcpy(ECC_CurveInitStruct.B, B, sizeof(uint32_t) * ECC_U32_LENGTH);
    memcpy(ECC_CurveInitStruct.P, P, sizeof(uint32_t) * ECC_U32_LENGTH);
    memcpy(ECC_CurveInitStruct.PRSquare, PRSquare, sizeof(uint32_t) * ECC_U32_LENGTH);
    ECC_CurveInitStruct.ConstP = ConstP;
    memcpy(ECC_CurveInitStruct.N, N, sizeof(uint32_t) * ECC_U32_LENGTH);
    memcpy(ECC_CurveInitStruct.NRSquare, NRSquare, sizeof(uint32_t) * ECC_U32_LENGTH);
    ECC_CurveInitStruct.ConstN = ConstN;
    ECC_CurveInitStruct.H = H;
    memcpy(ECC_CurveInitStruct.G.X, GX, sizeof(uint32_t) * ECC_U32_LENGTH);
    memcpy(ECC_CurveInitStruct.G.Y, GY, sizeof(uint32_t) * ECC_U32_LENGTH);
}

hal_status_t hal_pkc_modular_left_shift_handle(
    uint32_t bits_len, uint32_t in_a[], uint32_t in_prime[], uint32_t shift_bits, uint32_t result[])
{
    hal_status_t ret = HAL_OK;
    pkc_modular_shift_t pkc_left = {
        .p_A = in_a,
        .shift_bits = shift_bits,
        .p_P = in_prime,
    };
    pkc_handle.p_instance = PKC;
    pkc_handle.p_result = result;
    pkc_handle.init.p_ecc_curve = &ECC_CurveInitStruct;
    pkc_handle.init.data_bits = bits_len;
    pkc_handle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    pkc_handle.init.random_func = NULL;

    hal_pkc_init(&pkc_handle);
    ret = hal_pkc_modular_left_shift(&(pkc_handle), &pkc_left, 1000);
    hal_pkc_deinit(&pkc_handle);

    return ret;
}

hal_status_t hal_pkc_modular_compare_handle(uint32_t bits_len, uint32_t in_a[], uint32_t in_prime[], uint32_t result[])
{
    hal_status_t ret = HAL_OK;

    pkc_modular_compare_t pkc_comp = {
        .p_A = in_a,
        .p_P = in_prime,
    };
    pkc_handle.p_instance = PKC;
    pkc_handle.p_result = result;
    pkc_handle.init.p_ecc_curve = &ECC_CurveInitStruct;
    pkc_handle.init.data_bits = bits_len;
    pkc_handle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    pkc_handle.init.random_func = NULL;

    hal_pkc_init(&pkc_handle);
    ret = hal_pkc_modular_compare(&(pkc_handle), &pkc_comp, 1000);
    hal_pkc_deinit(&pkc_handle);

    return ret;
}

hal_status_t hal_pkc_montgomery_inverse(uint32_t bits_len, uint32_t in_a[], uint32_t in_prime[], uint32_t constp, uint32_t out_x[])
{
    hal_status_t ret = HAL_OK;

    pkc_montgomery_inversion_t pkc_inverse = {
        .p_A = in_a,
        .p_P = in_prime,
        .ConstP = constp,
    };
    pkc_handle.p_instance = PKC;
    pkc_handle.p_result = out_x;
    pkc_handle.init.p_ecc_curve = &ECC_CurveInitStruct;
    pkc_handle.init.data_bits = bits_len;
    pkc_handle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    pkc_handle.init.random_func = NULL;

    hal_pkc_init(&pkc_handle);
    ret = hal_pkc_montgomery_inversion(&pkc_handle, &pkc_inverse, 5000);
    hal_pkc_deinit(&pkc_handle);

    return ret;
}

hal_status_t hal_pkc_rsa_modular_exponent_handle(uint32_t bits_len,
                                                 uint32_t in_a[],
                                                 uint32_t in_b[],
                                                 uint32_t in_prime[],
                                                 uint32_t r_square[],
                                                 uint32_t in_constp,
                                                 uint32_t result[])
{
    hal_status_t ret = HAL_OK;
    pkc_rsa_modular_exponent_t pkc_rsa = {
        .p_A = in_a,
        .p_B = in_b,
        .p_P = in_prime,
        .p_P_R2 = r_square,
        .ConstP = in_constp
    };
    pkc_handle.p_instance = PKC;
    pkc_handle.p_result = result;
    pkc_handle.init.p_ecc_curve = &ECC_CurveInitStruct;
    pkc_handle.init.data_bits = bits_len;
    pkc_handle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    pkc_handle.init.random_func = NULL;

    hal_pkc_init(&pkc_handle);
    ret = hal_pkc_rsa_modular_exponent(&(pkc_handle), &pkc_rsa, 1000);
    hal_pkc_deinit(&pkc_handle);

    return ret;
}

hal_status_t hal_pkc_montgomery_mul(
    uint32_t bits_len, uint32_t in_a[], uint32_t in_b[], uint32_t in_prime[], uint32_t constq, uint32_t result[])
{
    hal_status_t ret = HAL_OK;
    pkc_montgomery_multi_t pkc_mul = {
        .p_A = in_a,
        .p_B = in_b,
        .p_P = in_prime,
        .ConstP = constq,
    };
    pkc_handle.p_instance = PKC;
    pkc_handle.p_result = result;
    pkc_handle.init.p_ecc_curve = &ECC_CurveInitStruct;
    pkc_handle.init.data_bits = bits_len;
    pkc_handle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    pkc_handle.init.random_func = NULL;

    hal_pkc_init(&pkc_handle);
    ret = hal_pkc_montgomery_multi(&(pkc_handle), &pkc_mul, 5000);
    hal_pkc_deinit(&pkc_handle);

    return ret;
}

ecc_curve_init_t *pkc_set_curve_p256_params(algo_ecc_curve_type_e curve)
{
    switch (curve)
    {
        case ECC_CURVE_SECP256R1:
            pkc_set_curve_secp256r1();
            break;
        case ECC_CURVE_SECP256K1:
            pkc_set_curve_secp256k1();
            break;
        default:
            break;
    }
    return &ECC_CurveInitStruct;
}

void hal_set_curve(ecc_curve_init_t *curve)
{
    if (NULL == curve)
    {
        pkc_handle.init.p_ecc_curve = (ecc_curve_init_t *)pkc_set_curve_p256_params(ECC_CURVE_SECP256R1);
    }
    else
    {
        pkc_handle.init.p_ecc_curve = curve;
    }
}

hal_status_t hal_pkc_ecc_point_mul_handle(uint32_t k[ECC_U32_LENGTH], ecc_point_t *Q, ecc_point_t *result)
{
    hal_status_t ret = HAL_OK;
    pkc_ecc_point_multi_t pkc_point_mul;

    pkc_handle.p_instance = PKC;
    pkc_handle.p_result = result;

    if (NULL == pkc_handle.init.p_ecc_curve)
    {
        pkc_handle.init.p_ecc_curve = &ECC_CurveInitStruct;
    }

    pkc_handle.init.data_bits = 256;
    pkc_handle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    pkc_handle.init.random_func = (uint32_t(*)(void))rand;

    hal_pkc_init(&pkc_handle);

    pkc_point_mul.p_K = k;
    pkc_point_mul.p_ecc_point = Q;

    ret = hal_pkc_ecc_point_multi(&pkc_handle, &pkc_point_mul, 5000);
    hal_pkc_deinit(&pkc_handle);

    return ret;
}

hal_status_t hal_pkc_modular_sub_handle(uint32_t bits_len, uint32_t in_a[], uint32_t in_b[], uint32_t in_prime[], uint32_t result[])
{
    hal_status_t ret = HAL_OK;
    pkc_modular_sub_t pkc_sub = {
        .p_A = in_a,
        .p_B = in_b,
        .p_P = in_prime,
    };
    pkc_handle.p_instance = PKC;
    pkc_handle.p_result = result;
    pkc_handle.init.p_ecc_curve = &ECC_CurveInitStruct;
    pkc_handle.init.data_bits = bits_len;
    pkc_handle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    pkc_handle.init.random_func = NULL;

    hal_pkc_init(&pkc_handle);
    ret = hal_pkc_modular_sub(&(pkc_handle), &pkc_sub, 1000);
    hal_pkc_deinit(&pkc_handle);

    return ret;
}

hal_status_t hal_pkc_modular_add_handle(uint32_t bits_len, uint32_t in_a[], uint32_t in_b[], uint32_t in_prime[], uint32_t result[])
{
    hal_status_t ret = HAL_OK;
    pkc_modular_add_t pkc_add = {
        .p_A = in_a,
        .p_B = in_b,
        .p_P = in_prime,
    };
    pkc_handle.p_instance = PKC;
    pkc_handle.p_result = result;
    pkc_handle.init.p_ecc_curve = &ECC_CurveInitStruct;
    pkc_handle.init.data_bits = bits_len;
    pkc_handle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    pkc_handle.init.random_func = NULL;

    hal_pkc_init(&pkc_handle);
    ret = hal_pkc_modular_add(&(pkc_handle), &pkc_add, 1000);
    hal_pkc_deinit(&pkc_handle);

    return ret;
}

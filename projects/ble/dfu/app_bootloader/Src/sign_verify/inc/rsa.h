/*
* Copyright (C) 2017, Shenzhen Goodix Technology Co., Ltd.
* All Rights Reserved.
*/

#ifndef __RSA_H__
#define __RSA_H__

#include "grx_sys.h"

typedef struct bl_rsa_public_key
{
    uint8_t n[256];
    uint32_t e;
    uint8_t rr[256];
    uint32_t c;
} bl_rsa_public_key_t;

typedef struct bl_rsa_private_key
{
    uint8_t s[256];
} bl_rsa_private_key_t;

typedef struct
{
    uint32_t len;                 /*!<  size(N) in chars  */
    bl_rsa_public_key_t  *pk;
    bl_rsa_private_key_t  *sk;
} bl_rsa_context;

int bl_rsa_rsassa_pss_verify(bl_rsa_context *ctx, const uint8_t *hash, const uint8_t *sig);

#endif // __RSA_H__


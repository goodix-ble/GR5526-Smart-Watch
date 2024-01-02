/**
 ****************************************************************************************
 *
 * @file utility.c
 *
 * @brief utility Implementation.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
* INCLUDE FILES
****************************************************************************************
*/
#include "utility.h"

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void htole16(void *buf, uint16_t x)
{
    uint8_t *u8ptr;
    u8ptr = buf;
    u8ptr[0] = (uint8_t) x;
    u8ptr[1] = (uint8_t)(x >> 8);
}

void htole32(void *buf, uint32_t x)
{
    uint8_t *u8ptr;
    u8ptr = buf;
    u8ptr[0] = (uint8_t) x;
    u8ptr[1] = (uint8_t)(x >> 8);
    u8ptr[2] = (uint8_t)(x >> 16);
    u8ptr[3] = (uint8_t)(x >> 24);
}

void htole64(void *buf, uint64_t x)
{
    uint8_t *u8ptr;
    u8ptr = buf;
    u8ptr[0] = (uint8_t) x;
    u8ptr[1] = (uint8_t)(x >> 8);
    u8ptr[2] = (uint8_t)(x >> 16);
    u8ptr[3] = (uint8_t)(x >> 24);
    u8ptr[4] = (uint8_t)(x >> 32);
    u8ptr[5] = (uint8_t)(x >> 40);
    u8ptr[6] = (uint8_t)(x >> 48);
    u8ptr[7] = (uint8_t)(x >> 56);
}

uint16_t le16toh(const void *buf)
{
    const uint8_t *u8ptr;
    uint16_t x;
    u8ptr = buf;
    x = u8ptr[0];
    x |= (uint16_t) u8ptr[1] << 8;
    return x;
}

uint32_t le32toh(const void *buf)
{
    const uint8_t *u8ptr;
    uint32_t x;
    u8ptr = buf;
    x = u8ptr[0];
    x |= (uint32_t) u8ptr[1] << 8;
    x |= (uint32_t) u8ptr[2] << 16;
    x |= (uint32_t) u8ptr[3] << 24;
    return x;
}

uint64_t le64toh(const void *buf)
{
    const uint8_t *u8ptr;
    uint64_t x;
    u8ptr = buf;
    x = u8ptr[0];
    x |= (uint64_t) u8ptr[1] << 8;
    x |= (uint64_t) u8ptr[2] << 16;
    x |= (uint64_t) u8ptr[3] << 24;
    x |= (uint64_t) u8ptr[4] << 32;
    x |= (uint64_t) u8ptr[5] << 40;
    x |= (uint64_t) u8ptr[6] << 48;
    x |= (uint64_t) u8ptr[7] << 56;
    return x;
}

void htobe16(void *buf, uint16_t x)
{
    uint8_t *u8ptr;
    u8ptr = buf;
    u8ptr[0] = (uint8_t)(x >> 8);
    u8ptr[1] = (uint8_t) x;
}

void htobe32(void *buf, uint32_t x)
{
    uint8_t *u8ptr;
    u8ptr = buf;
    u8ptr[0] = (uint8_t)(x >> 24);
    u8ptr[1] = (uint8_t)(x >> 16);
    u8ptr[2] = (uint8_t)(x >> 8);
    u8ptr[3] = (uint8_t) x;
}

void htobe64(void *buf, uint64_t x)
{
    uint8_t *u8ptr;
    u8ptr = buf;
    u8ptr[0] = (uint8_t)(x >> 56);
    u8ptr[1] = (uint8_t)(x >> 48);
    u8ptr[2] = (uint8_t)(x >> 40);
    u8ptr[3] = (uint8_t)(x >> 32);
    u8ptr[4] = (uint8_t)(x >> 24);
    u8ptr[5] = (uint8_t)(x >> 16);
    u8ptr[6] = (uint8_t)(x >> 8);
    u8ptr[7] = (uint8_t) x;
}

uint16_t be16toh(const void *buf)
{
    const uint8_t *u8ptr;
    uint16_t x;
    u8ptr = buf;
    x = (uint16_t) u8ptr[0] << 8;
    x |= u8ptr[1];
    return x;
}

uint32_t be32toh(const void *buf)
{
    const uint8_t *u8ptr;
    uint32_t x;
    u8ptr = buf;
    x = (uint32_t) u8ptr[0] << 24;
    x |= (uint32_t) u8ptr[1] << 16;
    x |= (uint32_t) u8ptr[2] << 8;
    x |= u8ptr[3];
    return x;
}

uint64_t be64toh(const void *buf)
{
    const uint8_t *u8ptr;
    uint64_t x;
    u8ptr = buf;
    x = (uint64_t) u8ptr[0] << 56;
    x |= (uint64_t) u8ptr[1] << 48;
    x |= (uint64_t) u8ptr[2] << 40;
    x |= (uint64_t) u8ptr[3] << 32;
    x |= (uint64_t) u8ptr[4] << 24;
    x |= (uint64_t) u8ptr[5] << 16;
    x |= (uint64_t) u8ptr[6] << 8;
    x |= u8ptr[7];
    return x;
}

uint8_t get_u8_inc(const uint8_t **pp_buf)
{
    const uint8_t *u8ptr;
    uint8_t x;
    u8ptr = *pp_buf;
    x = u8ptr[0];
    *pp_buf += 1;
    return x;
}

uint16_t get_u16_inc(const uint8_t **pp_buf)
{
    const uint8_t *u8ptr;
    uint16_t x;
    u8ptr = *pp_buf;
    x = u8ptr[0];
    x |= (uint16_t) u8ptr[1] << 8;
    *pp_buf += 2;
    return x;
}

uint32_t get_u32_inc(const uint8_t **pp_buf)
{
    const uint8_t *u8ptr;
    uint32_t x;
    u8ptr = *pp_buf;
    x = u8ptr[0];
    x |= (uint32_t) u8ptr[1] << 8;
    x |= (uint32_t) u8ptr[2] << 16;
    x |= (uint32_t) u8ptr[3] << 24;
    *pp_buf += 4;
    return x;
}

void put_u8_inc(uint8_t **pp_buf, uint8_t x)
{
    uint8_t *u8ptr;
    u8ptr = *pp_buf;
    u8ptr[0] = x;
    *pp_buf += 1;
}

void put_u16_inc(uint8_t **pp_buf, uint16_t x)
{
    uint8_t *u8ptr;
    u8ptr = *pp_buf;
    u8ptr[0] = (uint8_t) x;
    u8ptr[1] = (uint8_t)(x >> 8);
    *pp_buf += 2;
}

void put_u32_inc(uint8_t **pp_buf, uint32_t x)
{
    uint8_t *u8ptr;
    u8ptr = *pp_buf;
    u8ptr[0] = (uint8_t) x;
    u8ptr[1] = (uint8_t)(x >> 8);
    u8ptr[2] = (uint8_t)(x >> 16);
    u8ptr[3] = (uint8_t)(x >> 24);
    *pp_buf += 4;
}


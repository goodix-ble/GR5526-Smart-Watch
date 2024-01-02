/**
 ****************************************************************************************
 *
 * @file utility.c
 *
 * @brief Header file - utility
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

#ifndef __UTILITY_H__
#define __UTILITY_H__

#include <stdint.h>

/**
 * @defgroup UTILITY_MAROC Defines
 * @{
 */
#ifdef __cplusplus
extern "C" {
#endif

#define pbDATA(addr)           ((uint8_t   *) (addr))          /* byte pointer */
#define pwDATA(addr)           ((uint16_t  *) (addr))          /* word pointer */
#define pdwDATA(addr)          ((uint32_t  *) (addr))          /* double word pointer */

#define pcDATA(addr)           ((int8_t   *) (addr))           /* char pointer */
#define psDATA(addr)           ((int16_t  *) (addr))           /* short pointer */
#define plDATA(addr)           ((int32_t  *) (addr))           /* long pointer */

#define bDATA(addr)            (*(volatile uint8_t *) (addr))   /* byte */
#define wDATA(addr)            (*(volatile uint16_t *) (addr))  /* word */
#define dwDATA(addr)           (*(volatile uint32_t *) (addr))  /* double word */

#define cDATA(addr)            (*(volatile int8_t *) (addr))    /* char */
#define sDATA(addr)            (*(volatile int16_t *) (addr))   /* short */
#define lDATA(addr)            (*(volatile int32_t *) (addr))   /* ong */

#define W_SUCCESS    (uint16_t)0
#define W_FAIL       ((uint16_t)0xFFFF)

#define B_SUCCESS    (uint8_t)0
#define B_FAIL       ((uint8_t)0xFF)

#ifndef BV
#define BV(n)      (uint8_t)(1 << (n))
#endif

#ifndef BF
#define BF(x, b, s)  ((uint8_t)((x) & (b)) >> (s))
#endif

#ifndef MIN
#define MIN(n, m)   (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
#define MAX(n, m)   (((n) < (m)) ? (m) : (n))
#endif

#ifndef ABS
#define ABS(n)     (((n) < 0) ? -(n) : (n))
#endif

#ifndef ALIGN_NUM
#define ALIGN_NUM(align, num) (((num) - 1) + (align) - (((num) - 1) % (align)))
#endif

#define BIT_MASK(n)     (uint8_t)(((1) << n) - 1)

/*takes a byte out of a uint32:var -uint32, ByteNum - byte tao take out(0-3)*/
#define BREAK_U32(var, ByteNum) (uint8_t)((uint32_t)(((var) >> ((uint8_t)((ByteNum) * 8))) & 0x00FF))

#define BUILD_U32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32_t)((uint32_t)((Byte0) & 0x00FF) + \
          ((uint32_t)((Byte1) & 0x00FF) << 8) + \
          ((uint32_t)((Byte2) & 0x00FF) << 16) + \
          ((uint32_t)((Byte3) & 0x00FF) << 24)))

#define HI_UINT32_T(a) (((a) >> 24) & 0xFF)
#define L3_UINT32_T(a) (((a) >> 16) & 0xFF)
#define L2_UINT32_T(a) (((a) >> 8) & 0xFF)
#define LO_UINT32_T(a) ((a) & 0xFF)

#define BUILD_U16(loByte, hiByte)  ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_U16(a) (uint8_t)(((uint16_t)(a) >> 8) & 0xFF)
#define LO_U16(a) (uint8_t)((uint16_t)(a) & 0xFF)

#define BUILD_U8(hiByte, loByte) ((uint8_t)(((loByte) & 0x0F) + (((hiByte) & 0x0F) << 4)))

#ifndef HI_U8
#define HI_U8(a) (((uint8_t)(a) >> 4) & 0x0F)
#endif

#ifndef LO_U8
#define LO_U8(a) ((uint8_t)(a) & 0x0F)
#endif

#ifndef GET_BIT
#define GET_BIT(var, Idx)  (((uint8_t)(var) & BV((Idx) % 8)) ? 1 : 0)
#endif

#ifndef SET_BIT
#define SET_BIT(var, Idx)  ((uint8_t)(var) |= BV(((Idx) % 8)))
#endif

#ifndef CLR_BIT
#define CLR_BIT(var, Idx)  ((uint8_t)(var) &= ((BV((Idx) % 8) ^ 0xFF)))
#endif

#ifndef GET_BITFIELD
#define GET_BITFIELD(var, MSB, LSB)  ((uint8_t)((var) << (7 - MSB)) >> ((7 - MSB) + LSB))
#endif

#ifndef SET_BITFIELD
#define SET_BITFIELD(var, MSB, LSB, value)  ((uint8_t)(var) = \
        (uint8_t)((var & (~(BIT_MASK((MSB - LSB))<<LSB))) | ((value & BIT_MASK((MSB - LSB))) << LSB)))
#endif

#ifndef CLR_BITFIELD
#define CLR_BITFIELD(var, MSB, LSB)  ((uint8_t)(var) &=  (uint8_t)(~(BIT_MASK(MSB - LSB)<<LSB)))
#endif

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, field) \
    ((type *)(((char *)(ptr)) - offsetof(type, field)))
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) \
    ((unsigned long) ((sizeof(array) / sizeof((array)[0]))))
#endif

#define UNUSED_VARIABLE(x)  ((void)(x))
#define UNUSED_PARAMETER(x) UNUSED_VARIABLE(x)
#define UNUSED_RETURN_VALUE(x) UNUSED_VARIABLE(x)

#undef htole16
#undef htole32
#undef htole64
#undef le16toh
#undef le32toh
#undef le64toh
#undef htobe16
#undef htobe32
#undef htobe64
#undef be16toh
#undef be32toh
#undef be64toh
#undef get_u8_inc
#undef get_u16_inc
#undef get_u32_inc
#undef put_u8_inc
#undef put_u16_inc
#undef put_u32_inc
/** @} */

/**
 * @defgroup UTILITY_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Function for transforming a 16 digit number into a array according to Little-Endian.
 *
 * @param[out] p_buf: Pointer to a array.
 * @param[in]  x:     The 16 digit number need to be transformed.
 *****************************************************************************************
 */
void htole16(void *p_buf, uint16_t x);

/**
 *****************************************************************************************
 * @brief Function for transforming a 32 digit number into a array according to Little-Endian.
 *
 * @param[out] p_buf: Pointer to a array.
 * @param[in]  x:     The 32 digit number need to be transformed.
 *****************************************************************************************
 */
void htole32(void *p_buf, uint32_t x);

/**
 *****************************************************************************************
 * @brief Function for transforming a 64 digit number into a array according to Little-Endian.
 *
 * @param[out] p_buf: Pointer to a array.
 * @param[in]  x:     The 64 digit number need to be transformed.
 *****************************************************************************************
 */
void htole64(void *p_buf, uint64_t x);

/**
 *****************************************************************************************
 * @brief Function for transforming a array into a 16 digit unsigned number according to Little-Endian.
 *
 * @param[in] p_buf: Pointer to a array need to be transformed.
 * 
 * @retval ::The result of transforming
 *****************************************************************************************
 */
uint16_t le16toh(const void *p_buf);

/**
 *****************************************************************************************
 * @brief Function for transforming a array into a 32 digit unsigned number according to Little-Endian.
 *
 * @param[in] p_buf: Pointer to a array need to be transformed.
 * 
 * @retval ::The result of transforming
 *****************************************************************************************
 */
uint32_t le32toh(const void *p_buf);

/**
 *****************************************************************************************
 * @brief Function for transforming a array into a 64 digit unsigned number according to Little-Endian.
 *
 * @param[in] p_buf: Pointer to a array need to be transformed.
 * 
 * @retval ::The result of transforming
 *****************************************************************************************
 */
uint64_t le64toh(const void *p_buf);

/**
 *****************************************************************************************
 * @brief Function for transforming a 16 digit number into a array according to Big-Endian.
 *
 * @param[out] p_buf: Pointer to a array.
 * @param[in]  x:     The 16 digit number need to be transformed.
 *****************************************************************************************
 */
void htobe16(void *p_buf, uint16_t x);

/**
 *****************************************************************************************
 * @brief Function for transforming a 32 digit number into a array according to Big-Endian.
 *
 * @param[out] p_buf: Pointer to a array.
 * @param[in]  x:     The 16 digit number need to be transformed.
 *****************************************************************************************
 */
void htobe32(void *p_buf, uint32_t x);

/**
 *****************************************************************************************
 * @brief Function for transforming a 64 digit number into a array according to Big-Endian.
 *
 * @param[out] p_buf: Pointer to a array.
 * @param[in]  x:     The 16 digit number need to be transformed.
 *****************************************************************************************
 */
void htobe64(void *p_buf, uint64_t x);

/**
 *****************************************************************************************
 * @brief Function for transforming a array into a 16 digit unsigned number according to Big-Endian.
 *
 * @param[in] p_buf: Pointer to a array need to be transformed.
 * 
 * @retval ::The result of transforming
 *****************************************************************************************
 */
uint16_t be16toh(const void *buf);

/**
 *****************************************************************************************
 * @brief Function for transforming a array into a 32 digit unsigned number according to Big-Endian.
 *
 * @param[in] p_buf: Pointer to a array need to be transformed.
 * 
 * @retval ::The result of transforming
 *****************************************************************************************
 */
uint32_t be32toh(const void *buf);

/**
 *****************************************************************************************
 * @brief Function for transforming a array into a 64 digit unsigned number according to Big-Endian.
 *
 * @param[in] p_buf: Pointer to a array need to be transformed.
 * 
 * @retval ::The result of transforming
 *****************************************************************************************
 */
uint64_t be64toh(const void *buf);

/**
 *****************************************************************************************
 * @brief Function for getting the first 8 bits of an address.
 *
 * @param[in] pp_buf: Pointer to an address.
 * 
 * @retval ::The result of getting
 *****************************************************************************************
 */
uint8_t get_u8_inc(const uint8_t **pp_buf);

/**
 *****************************************************************************************
 * @brief Function for getting the first 16 bits of an address.
 *
 * @param[in] pp_buf: Pointer to an address.
 * 
 * @retval ::The result of getting
 *****************************************************************************************
 */
uint16_t get_u16_inc(const uint8_t **pp_buf);

/**
 *****************************************************************************************
 * @brief Function for getting the first 32 bits of an address.
 *
 * @param[in] pp_buf: Pointer to an address.
 * 
 * @retval ::The result of getting
 *****************************************************************************************
 */
uint32_t get_u32_inc(const uint8_t **pp_buf);

/**
 *****************************************************************************************
 * @brief Function for putting a 8 digit unsigned number to an address.
 *
 * @param[out] pp_buf: Pointer to an address.
 * @param[in]  x:      The 8 digit number need to be transformed.
 *****************************************************************************************
 */
void put_u8_inc(uint8_t **pp_buf, uint8_t x);

/**
 *****************************************************************************************
 * @brief Function for putting a 16 digit unsigned number to an address.
 *
 * @param[out] pp_buf: Pointer to an address.
 * @param[in]  x:      The 16 digit number need to be transformed.
 *****************************************************************************************
 */
void put_u16_inc(uint8_t **pp_buf, uint16_t x);

/**
 *****************************************************************************************
 * @brief Function for putting a 32 digit unsigned number to an address.
 *
 * @param[out] pp_buf: Pointer to an address.
 * @param[in]  x:      The 32 digit number need to be transformed.
 *****************************************************************************************
 */
void put_u32_inc(uint8_t **pp_buf, uint32_t x);
/** @} */
#ifdef __cplusplus
}
#endif
#endif    /* __UTILITY_H__ */


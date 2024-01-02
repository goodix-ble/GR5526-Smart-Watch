/**
 ****************************************************************************************
 *
 * @file    hal_gfx_utils.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of Graphics library.
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
 ****************************************************************************************
 */

/** @addtogroup GRAPHICS_SDK Graphics
 *  @{
 */

/** @addtogroup HAL_GFX HAL GFX
  * @{
  */

/** @defgroup HAL_GFX_UTILS GFX UTILS
  * @brief graphics utils  define
  * @{
  */


#ifndef HAL_GFX_UTILS_H_
#define HAL_GFX_UTILS_H_

#include "hal_gfx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup HAL_GFX_UTILS_STRUCT Structures
  * @{
  */

/**
  * @brief  Image object defination
  */

typedef struct _img_obj_ {
    hal_gfx_buffer_t bo;        /**< Texture image info */
    uint16_t w;                 /**< Texture image width */
    uint16_t h;                 /**< Texture image high */
    int      stride;            /**< Texture image stride */
    uint32_t color;             /**< Texture image color, default 0 */
    uint8_t  format;            /**< Texture image rgb format */
    uint8_t  sampling_mode;     /**< Texture image FILTER */
} img_obj_t;

/** @} */

/** @addtogroup HAL_GFX_UTILS_FUNCTION Functions
  * @{
  */

/**
 *****************************************************************************************
 * @brief Return system timestamp in second, need to porting
 *
 * @return timestamp in second
 *****************************************************************************************
 */
float           hal_gfx_get_time(void);

/**
 *****************************************************************************************
 * @brief Return system timestamp in millisecond, need to porting
 *
 * @return timestamp in millisecond
 *****************************************************************************************
 */
float           hal_gfx_get_wall_time(void);

/**
 *****************************************************************************************
 * @brief load file to buffer, need to porting
 *
 * @return the struct to save buffer
 *****************************************************************************************
 */
hal_gfx_buffer_t   hal_gfx_load_file(const char *filename, int length, void *buffer);

/**
 *****************************************************************************************
 * @brief save buffer to file, need to porting
 *
 * @return 0 - successful; other - fail
 *****************************************************************************************
 */
int             hal_gfx_save_file(const char *filename, int length, void *buffer);

/**
 *****************************************************************************************
 * @brief generate a random number, need to porting
 *
 * @return random number
 *****************************************************************************************
 */
unsigned int    hal_gfx_rand(void);

/**
 *****************************************************************************************
 * @brief calculate the fps, need to porting
 *
 * @return none
 *****************************************************************************************
 */
void            hal_gfx_calculate_fps(void);

/**
 *****************************************************************************************
 * @brief calculate the fps, need to porting
 *
 * @param[in] start_time: start timestamp in ms
 * @param[in] frame: frame count from start_time
 *
 * @return fps
 *****************************************************************************************
 */
float           hal_gfx_calculate_fps_ext(float start_time, uint32_t frame);

/**
 *****************************************************************************************
 * @brief memcpy function, need to porting
 *
 * @param[in] destination: destination address
 * @param[in] source: source address
 * @param[in] num: copy data in bytes
 *
 * @return destination address
 *****************************************************************************************
 */
void *          hal_gfx_memcpy ( void * destination, const void * source, size_t num );

/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */


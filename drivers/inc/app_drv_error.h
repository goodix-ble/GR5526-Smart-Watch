/**
 ****************************************************************************************
 *
 * @file    app_drv_error.h
 * @author  BLE Driver Team
 * @brief   Header file of app driver error code.
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

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_DRIVER_ERROR ERROR DEFINE
  * @brief APP ERROR DEFINE
  * @{
  */


#ifndef _APP_DRV_ERROR_H_
#define _APP_DRV_ERROR_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@addtogroup APP_DRV_ERR Defines
 * @{
 */
/**@addtogroup APP_DRV_ERR_CODE App Driver error codes
 * @{
 */
#define APP_DRV_SUCCESS                               0x0000   /**< Successful. */
#define APP_DRV_ERR_HAL                               0x00E1   /**< Hal internal error. */
#define APP_DRV_ERR_BUSY                              0x00E2   /**< Driver is busy. */
#define APP_DRV_ERR_TIMEOUT                           0x00E3   /**< Timeout occurred. */
#define APP_DRV_ERR_INVALID_PARAM                     0x00E4   /**< Invalid parameter. */
#define APP_DRV_ERR_POINTER_NULL                      0x00E5   /**< Invalid pointer. */
#define APP_DRV_ERR_INVALID_TYPE                      0x00E6   /**< Invalid type. */
#define APP_DRV_ERR_INVALID_MODE                      0x00E7   /**< Invalid mode. */
#define APP_DRV_ERR_INVALID_ID                        0x00E8   /**< Invalid ID. */
#define APP_DRV_ERR_NOT_INIT                          0x00E9   /**< Driver not init. */
#define APP_DRV_ERR_INVALID_INIT                      0x00EA   /**< Invalid init, deinit firstly. */
/** @} */

/**@addtogroup APP_DRV_ERR_CODE_CHECK App Driver error code check
 * @{
 */
/**@brief App Driver error code check. */
#ifndef APP_DRV_ERR_CODE_CHECK
#define APP_DRV_ERR_CODE_CHECK(err_code)            \
    do                                              \
    {                                               \
        if (APP_DRV_SUCCESS != err_code)            \
        {                                           \
            return err_code;                        \
        }                                           \
    } while(0)
#endif
/** @} */

/**@addtogroup HAL_DRV_ERR_CODE_CHECK Hal Driver error code check
 * @{
 */
 /**@brief Hal Driver error code check. */
#ifndef HAL_ERR_CODE_CHECK
#define HAL_ERR_CODE_CHECK(err_code)                \
    do                                              \
    {                                               \
        if (HAL_OK != err_code)                     \
        {                                           \
            return (uint16_t)err_code;              \
        }                                           \
    } while(0)
#endif
/** @} */
/** @} */

/**
 * @defgroup APP_DRV_ERROR_TYPEDEF Type definitions
 * @{
 */
/**@brief APP driver error type. */
typedef uint16_t   app_drv_err_t;
/**@} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */


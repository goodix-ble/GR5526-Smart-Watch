/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_bod.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of ADC HAL library.
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

/** @addtogroup HAL_DRIVER HAL Driver
  * @{
  */

/** @defgroup HAL_BOD BOD
  * @brief BOD HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_BOD_H__
#define __GR55xx_HAL_BOD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_bod.h"
#include "gr55xx_hal_def.h"
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/


/** @addtogroup HAL_BOD_TYPEDEFS Type definitions
  * @{
  */
/**
  * @brief BOD init structure definition
  */
typedef ll_bod_init_t       bod_init_t;
/** @} */

/** @addtogroup HAL_BOD_STRUCTURES Structures
  * @{
  */

/**
  * @brief BOD handle Structure definition
  */
typedef struct _bod_handle
{
    bod_init_t              init;           /**< BOD configuration parameters      */
    __IO hal_lock_t         lock;           /**< Locking object                    */
} bod_handle_t;

/**
  * @brief HAL BOD Callback function definition
  */
typedef struct _bod_callback
{
    void (*bod_msp_init)(bod_handle_t *p_bod);              /**< BOD init MSP callback            */
    void (*bod_msp_deinit)(bod_handle_t *p_bod);            /**< BOD de-init MSP callback         */
    void (*bod_fedge_callback)(bod_handle_t *p_bod);        /**< BOD conversion completed callback*/
    void (*bod_redge_callback)(bod_handle_t *p_bod);		/**< BOD conversion completed callback*/
} bod_callback_t;

/** @} */

/**
  * @defgroup  HAL_BOD_MACRO Defines
  * @{
  */
/* Exported constants --------------------------------------------------------*/
/** @defgroup HAL_BOD_Exported_Constants BOD Exported Constants
  * @{
  */

/** @defgroup HAL_BOD_ENABLE BOD ENABLE
  * @{
  */
#define HAL_BOD_ENABLE                  LL_BOD_ENABLE    /**< BOD enable  */
#define HAL_BOD_DISABLE                 LL_BOD_DISABLE   /**< BOD disable  */
/** @} */

/** @defgroup HAL_BOD_BOD2_ENABLE BOD2 ENABLE
  * @{
  */
#define HAL_BOD2_ENABLE                 LL_BOD2_ENABLE   /**< BOD2 enable  */
#define HAL_BOD2_DISABLE                LL_BOD2_DISABLE  /**< BOD2 disable  */
/** @} */

/** @defgroup BOD_HAL_STATIC_ENABLE BOD STATIC ENABLE
  * @{
  */
#define HAL_BOD_STATIC_ENABLE           LL_BOD_STATIC_ENABLE    /**< BOD STATIC enable  */
#define HAL_BOD_STATIC_DISABLE          LL_BOD_STATIC_DISABLE   /**< BOD STATIC disable  */
/** @} */

/** @defgroup HAL_BOD_BOD2_LEVEL BOD2 LVEVL
  * @{
  */
#define HAL_BOD2_LEVEL_0                LL_BOD2_LEVEL_0  /**< BOD2 Level 0  */
#define HAL_BOD2_LEVEL_1                LL_BOD2_LEVEL_1  /**< BOD2 Level 1  */
#define HAL_BOD2_LEVEL_2                LL_BOD2_LEVEL_2  /**< BOD2 Level 2  */
#define HAL_BOD2_LEVEL_3                LL_BOD2_LEVEL_3  /**< BOD2 Level 3  */
#define HAL_BOD2_LEVEL_4                LL_BOD2_LEVEL_4  /**< BOD2 Level 4  */
#define HAL_BOD2_LEVEL_5                LL_BOD2_LEVEL_5  /**< BOD2 Level 5  */
#define HAL_BOD2_LEVEL_6                LL_BOD2_LEVEL_6  /**< BOD2 Level 6  */
#define HAL_BOD2_LEVEL_7                LL_BOD2_LEVEL_7  /**< BOD2 Level 7  */
#define HAL_BOD2_LEVEL_8                LL_BOD2_LEVEL_8  /**< BOD2 Level 8  */
#define HAL_BOD2_LEVEL_9                LL_BOD2_LEVEL_9  /**< BOD2 Level 9  */
#define HAL_BOD2_LEVEL_10               LL_BOD2_LEVEL_10 /**< BOD2 Level 10  */
#define HAL_BOD2_LEVEL_11               LL_BOD2_LEVEL_11 /**< BOD2 Level 11  */
#define HAL_BOD2_LEVEL_12               LL_BOD2_LEVEL_12 /**< BOD2 Level 12  */
#define HAL_BOD2_LEVEL_13               LL_BOD2_LEVEL_13 /**< BOD2 Level 13  */
#define HAL_BOD2_LEVEL_14               LL_BOD2_LEVEL_14 /**< BOD2 Level 14  */
#define HAL_BOD2_LEVEL_15               LL_BOD2_LEVEL_15 /**< BOD2 Level 15  */
/** @} */
/** @} */
/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_BOD_DRIVER_FUNCTIONS Functions
  * @{
  */

/* Exported functions --------------------------------------------------------*/

/**
 ****************************************************************************************
 * @brief  Initialize the BOD according to the specified parameters
 *         in the bod_init_t and initialize the associated handle.
 *
 * @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
 *                    the specified BOD module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_bod_init(bod_handle_t *p_bod);

/**
 ****************************************************************************************
 * @brief  De-initialize the BOD peripheral.
 *
 * @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
 *                    the specified BOD module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_bod_deinit(bod_handle_t *p_bod);

/**
 ****************************************************************************************
 * @brief Initialize the BOD MSP.
 *
 * @note  This function should not be modified. When the callback is needed,
 *         the hal_bod_msp_init could be implemented in the user file
 *
 * @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
 *                    the specified BOD module.
 ****************************************************************************************
 */
void hal_bod_msp_init(bod_handle_t *p_bod);

/**
 ****************************************************************************************
 * @brief De-initialize the BOD MSP.
 *
 * @note  This function should not be modified. When the callback is needed,
 *         the hal_bod_msp_deinit could be implemented in the user file
 *
 * @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
 *                    the specified BOD module.
 ****************************************************************************************
 */
void hal_bod_msp_deinit(bod_handle_t *p_bod);

/**
 ****************************************************************************************
 * @brief  BOD fall edge callback.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_bod_fedge_callback can be implemented in the user file.
 *
 * @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
 *                    the specified BOD module.
 ****************************************************************************************
 */
void hal_bod_fedge_callback(bod_handle_t *p_bod);

/**
 ****************************************************************************************
 * @brief  BOD rise edge callback.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_bod_redge_callback can be implemented in the user file.
 *
 * @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
 *                    the specified BOD module.
 ****************************************************************************************
 */
void hal_bod_redge_callback(bod_handle_t *p_bod);

/**
 ****************************************************************************************
 * @brief Handle BOD fall edge interrupt request.
 *
 * @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
 *                    the specified BOD module.
 ****************************************************************************************
 */
void hal_bod_fedge_irq_handler(bod_handle_t *p_bod);

/**
 ****************************************************************************************
 * @brief Handle BOD rise edge interrupt request.
 *
 * @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
 *                    the specified BOD module.
 ****************************************************************************************
 */
void hal_bod_redge_irq_handler(bod_handle_t *p_bod);

/**
****************************************************************************************
* @brief  enable bod
*
* @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
*                    the specified BOD module.
* @param[in]  enable: bod enable flag. the value can be HAL_BOD_ENABLE or HAL_BOD_DISABLE
*
* @retval ::HAL_OK: Operation is OK.
* @retval ::HAL_ERROR: Parameter error or operation not supported.
* @retval ::HAL_BUSY: Driver is busy.
* @retval ::HAL_TIMEOUT: Timeout occurred.
****************************************************************************************
*/
hal_status_t hal_bod_enable(bod_handle_t *p_bod, uint8_t enable);

/**
****************************************************************************************
* @brief  enable bod2
*
* @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
*                    the specified BOD module.
* @param[in]  enable: bod2 enable flag. the value can be HAL_BOD2_ENABLE or HAL_BOD2_DISABLE
*
* @retval ::HAL_OK: Operation is OK.
* @retval ::HAL_ERROR: Parameter error or operation not supported.
* @retval ::HAL_BUSY: Driver is busy.
* @retval ::HAL_TIMEOUT: Timeout occurred.
****************************************************************************************
*/
hal_status_t hal_bod2_enable(bod_handle_t *p_bod, uint8_t enable);

/**
****************************************************************************************
* @brief  Set BOD2 control level.
*
* @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
*                    the specified BOD module.
* @param[in]  level: the level of bod2 control.the value range between 0x0 ~ 0xF
*
* @retval ::HAL_OK: Operation is OK.
* @retval ::HAL_ERROR: Parameter error or operation not supported.
* @retval ::HAL_BUSY: Driver is busy.
* @retval ::HAL_TIMEOUT: Timeout occurred.
****************************************************************************************
*/
hal_status_t hal_bod2_set_level(bod_handle_t *p_bod, uint8_t level);

/**
****************************************************************************************
* @brief  enable/disable static mode
*
* @param[in]  p_bod: Pointer to an BOD handle which contains the configuration information for
*                    the specified BOD module.
* @param[in]  enable: static enable flag. the value can be HAL_BOD_STATIC_ENABLE or HAL_BOD_STATIC_DISABLE
*
* @retval ::HAL_OK: Operation is OK.
* @retval ::HAL_ERROR: Parameter error or operation not supported.
* @retval ::HAL_BUSY: Driver is busy.
* @retval ::HAL_TIMEOUT: Timeout occurred.
****************************************************************************************
*/
hal_status_t hal_bod_static_mode_enable(bod_handle_t *p_bod, uint8_t enable);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_BOD_H__ */

/** @} */

/** @} */

/** @} */

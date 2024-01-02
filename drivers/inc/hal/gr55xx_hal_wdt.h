/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_wdt.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of WDT HAL library.
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

/** @defgroup HAL_WDT WDT
  * @brief WDT HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_WDT_H__
#define __GR55xx_HAL_WDT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_wdt.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_WDT_STRUCTURES Structures
  * @{
  */

/** @defgroup  WDT_Configuration WDT Configuration
  * @{
  */

/**
  * @brief  WDT init structure definition
  */
typedef struct _wdt_init
{
    uint32_t counter;     /**< Specifies the WDT free-running downcounter value.
                               This parameter can be a number ranging between 0x0U and 0xFFFFFFFFU. */

    uint32_t reset_mode ; /**< Specifies if WDT Reset output is enable or not.
                               When RESET Mode is enabled, WDT will generate an interrupt
                               on first timeout. If interrupt has not been cleared before the second
                               timeout, WDT will then request a SoC Reset.

                               This parameter can be a value of @ref WDT_RESET_Mode. */

} wdt_init_t;

/** @} */

/** @defgroup WDT_handle WDT handle
  * @{
  */

/**
  * @brief  WDT handle Structure definition
  */
typedef struct _wdt_handle
{
    wdt_regs_t      *p_instance;  /**< Register base address      */

    wdt_init_t      init;       /**< WDT required parameters    */

    hal_lock_t      lock;       /**< WDT locking object         */

} wdt_handle_t;

/** @} */

/** @} */

/** @addtogroup HAL_WDT_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_WDT_Callback Callback
  * @{
  */

/**
  * @brief HAL_WDT Callback function definition
  */

typedef struct _wdt_callback
{
    void (*wdt_msp_init)(wdt_handle_t *p_wdt);                  /**< WDT init MSP callback              */
    void (*wdt_msp_deinit)(wdt_handle_t *p_wdt);                /**< WDT de-init MSP callback           */
    void (*wdt_period_elapsed_callback)(wdt_handle_t *p_wdt);   /**< WDT count complete callback        */
} wdt_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_WDT_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup WDT_Exported_Constants WDT Exported Constants
  * @{
  */

/** @defgroup WDT_RESET_Mode WDT Reset Mode
  * @{
  */
#define WDT_RESET_DISABLE               (0x00000000U)       /**< Reset ouput disable */
#define WDT_RESET_ENABLE                (0x00000001U)       /**< Reset output enable */
/** @} */

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup WDT_Private_Macros WDT Private Macros
  * @{
  */

/**
  * @brief Check if the WDT reset mode is valid.
  * @param __MODE__ WDT reset mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_WDT_RESET_MODE(__MODE__)     (((__MODE__) == WDT_RESET_ENABLE) || \
                                         ((__MODE__) == WDT_RESET_DISABLE))
/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_WDT_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup WDT_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initialization functions.
 *
@verbatim
  ==============================================================================
          ##### Initialization and de-initialization functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
      (+) Initialize and start the WDT according to the specified parameters
          in the wdt_init_t of associated handle.
      (+) Initialize the WDT MSP.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the WDT according to the specified
 *         parameters in the wdt_init_t of  associated handle.
 * @param[in]  p_wdt: Pointer to a WDT handle which contains the configuration
 *               information for the specified WDT module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_wdt_init(wdt_handle_t *p_wdt);

/**
 ****************************************************************************************
 * @brief  De-initialize the WDT peripheral.
 * @param[in]  p_wdt: WDT handle.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_wdt_deinit(wdt_handle_t *p_wdt);

/**
 ****************************************************************************************
 * @brief  Initialize the WDT MSP.
 * @param[in]  p_wdt: Pointer to a WDT handle which contains the configuration
 *               information for the specified WDT module.
 * @note   When rewriting this function in a user file, this mechanism may be added
 *         to avoid multiple initialization when hal_wdt_init function is called
 *         again to change parameters.
 ****************************************************************************************
 */
void hal_wdt_msp_init(wdt_handle_t *p_wdt);

/**
 ****************************************************************************************
 * @brief  De-initialize the WDT MSP.
 * @param[in]  p_wdt: Pointer to a WDT handle which contains the configuration
 *               information for the specified WDT module.
 * @note   When rewriting this function in a user file, this mechanism may be added
 *         to avoid multiple initialization when hal_wdt_init function is called
 *         again to change parameters.
 ****************************************************************************************
 */
void hal_wdt_msp_deinit(wdt_handle_t *p_wdt);

/** @} */

/** @addtogroup WDT_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions
 *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Refresh the WDT.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Refresh the WDT.
 * @param[in]  p_wdt: Pointer to a WDT handle which contains the configuration
 *               information for the specified WDT module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_wdt_refresh(wdt_handle_t *p_wdt);

/** @} */


/** @addtogroup WDT_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
  *
@verbatim
  ==============================================================================
                 ##### IRQ Handler and Callbacks functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Handle WDT interrupt request and associated function callback.

@endverbatim
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle WDT interrupt request.
 * @note   The Count Complete can be used if specific safety operations
 *         or data logging must be performed before the actual reset is generated.
 *         When RESET Mode is enabled, WDT will generate an interrupt on first timeout.
 *         If interrupt has not been cleared before the second timeout, WDT will then
 *         request a SoC Reset.
 * @param[in]  p_wdt: Pointer to a WDT handle which contains the configuration
 *               information for the specified WDT module.
 ****************************************************************************************
 */
void hal_wdt_irq_handler(wdt_handle_t *p_wdt);

/**
 ****************************************************************************************
 * @brief  WDT count complete(counter reaches to 0) callback.
 * @note   In RESET mode, NVIC interrupt of WDT can be disabled in
 *         hal_wdt_period_elapsed_callback() to make sure this callback
 *         be called once only.
 *         This function should not be modified. When the callback is needed,
 *         the hal_wdt_count_cplt_callback can be implemented in the user file.
 * @param[in]  p_wdt: Pointer to a WDT handle which contains the configuration
 *               information for the specified WDT module.
 ****************************************************************************************
 */
void hal_wdt_period_elapsed_callback(wdt_handle_t *p_wdt);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_WDT_H__ */

/** @} */

/** @} */

/** @} */

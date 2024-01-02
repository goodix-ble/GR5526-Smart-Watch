/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_aon_wdt.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of AON WDT HAL library.
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

/** @defgroup HAL_AON_WDT AON_WDT
  * @brief WDT HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_AON_WDT_H__
#define __GR55xx_HAL_AON_WDT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_aon_wdt.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_AON_WDT_STRUCTURES Structures
  * @{
  */

/** @defgroup  AON_WDT_Configuration AON_WDT Configuration
  * @{
  */

/**
  * @brief  AON_WDT_Configuration init structure definition
  */
typedef struct _aon_wdt_init
{
    uint32_t counter;       /**< Specifies the AON_WDT free-running downcounter value. Unit (ms)
                                 This parameter can be a number ranging between 0x0U ~ 0xFFFFFFFFU / (SystemSlowClock / 1000). */

    uint32_t alarm_counter; /**< Specifies the AON_WDT downcounter alarm value before system reset.
                                 When counter counts down to the alarm value, AON_WDT will generate
                                 an interrupt. After counter counts down to 0, AON_WDT will then
                                 request a SoC Reset. Unit (ms)

                                 This parameter can be a number ranging between 0x0U ~ 0xFFFFU / (SystemSlowClock / 1000). */

} aon_wdt_init_t;

/** @} */

/** @defgroup AON_WDT_handle AON_WDT handle
  * @{
  */

/**
  * @brief  AON_WDT handle Structure definition
  */
typedef struct _aon_wdt_handle
{
    aon_wdt_init_t  init;                   /**< AON_WDT required parameters    */

    hal_lock_t      lock;                   /**< AON_WDT locking object         */

    uint32_t        *SystemCoreLowClock;    /**<  AON_WDT Specifies the number of low-speed clocks. */
} aon_wdt_handle_t;

/** @} */

/** @} */

/** @addtogroup HAL_AON_WDT_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_AON_WDT_Callback Callback
  * @{
  */

/**
  * @brief HAL_AON_WDT Callback function definition
  */

typedef struct _aon_wdt_callback
{
    void (*aon_wdt_alarm_callback)(aon_wdt_handle_t *p_aon_wdt);    /**< AON_WDT count complete callback */
} aon_wdt_callback_t;

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_AON_WDT_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup AON_WDT_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions.
 *
@verbatim
  ==============================================================================
          ##### Initialization and Configuration functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
      (+) Initialize and start the AON_WDT according to the specified parameters
          in the wdt_init_t of associated handle.
      (+) Initialize the AON_WDT MSP.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the AON_WDT according to the specified parameters in the wdt_init_t
 *         of associated handle.
 *
 * @param[in]  p_aon_wdt: Pointer to a AON_WDT handle which contains the configuration
 *                        information for the specified AON_WDT module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aon_wdt_init(aon_wdt_handle_t *p_aon_wdt);

/**
 ****************************************************************************************
 * @brief De-initialize the AON_WDT peripheral.
 *
 * @param[in]  p_aon_wdt: AON_WDT handle.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aon_wdt_deinit(aon_wdt_handle_t *p_aon_wdt);

/** @} */

/** @addtogroup AON_WDT_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions
 *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Refresh the AON_WDT.
    (+) Handle AON_WDT interrupt request and associated function callback.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Refresh the AON_WDT.
 *
 * @param[in]  p_aon_wdt: Pointer to a AON_WDT handle which contains the configuration
 *                        information for the specified AON_WDT module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aon_wdt_refresh(aon_wdt_handle_t *p_aon_wdt);

/** @} */


/** @addtogroup AON_WDT_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle AON_WDT interrupt request.
 *
 * @note   The count completed can be used if specific safety operations
 *         or data logging must be performed before the actual reset is generated.
 *         When RESET Mode is enabled, AON_WDT will generate an interrupt on first timeout.
 *         If interrupt has not been cleared before the second timeout, AON_WDT will then
 *         request a SoC Reset.
 *
 * @param[in]  p_aon_wdt: Pointer to a AON_WDT handle which contains the configuration
 *                        information for the specified AON_WDT module.
 ****************************************************************************************
 */
void hal_aon_wdt_irq_handler(aon_wdt_handle_t *p_aon_wdt);

/**
 ****************************************************************************************
 * @brief  AON_WDT count complete (counter reaches to 0) callback.
 *
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_wdt_count_cplt_callback can be implemented in the user file.
 *
 * @param[in]  p_aon_wdt: Pointer to a AON_WDT handle which contains the configuration
 *                        information for the specified AON_WDT module.
 ****************************************************************************************
 */
void hal_aon_wdt_alarm_callback(aon_wdt_handle_t *p_aon_wdt);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_AON_WDT_H__ */

/** @} */

/** @} */

/** @} */

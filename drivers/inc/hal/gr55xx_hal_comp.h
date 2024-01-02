/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_comp.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of COMP HAL library.
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

/** @defgroup HAL_COMP COMP
  * @brief COMP HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_COMP_H__
#define __GR55xx_HAL_COMP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_comp.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_COMP_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_COMP_state HAL COMP state
  * @{
  */

/**
  * @brief HAL COMP State Enumerations definition
  */
typedef enum
{
    HAL_COMP_STATE_RESET        = 0x00,    /**< Peripheral not initialized                          */
    HAL_COMP_STATE_READY        = 0x01,    /**< Peripheral initialized and ready for use            */
    HAL_COMP_STATE_BUSY         = 0x02,    /**< An internal process is ongoing                      */
    HAL_COMP_STATE_ERROR        = 0x04     /**< Peripheral in error                                 */
} hal_comp_state_t;

/** @} */

/** @} */

/** @addtogroup HAL_COMP_STRUCTURES Structures
  * @{
  */

/** @defgroup COMP_Configuration COMP Configuration
  * @{
  */

/**
  * @brief   COMP init structure definition
  */
typedef ll_comp_init_t       comp_init_t;
/** @} */

/** @defgroup COMP_handle COMP handle
  * @{
  */

/**
  * @brief COMP handle Structure definition
  */
typedef struct _comp_handle
{
    comp_init_t             init;           /**< COMP configuration parameters      */

    __IO hal_lock_t         lock;           /**< Locking object                    */

    __IO hal_comp_state_t    state;         /**< COMP communication state           */

    __IO uint32_t           error_code;     /**< COMP error code                    */

    uint32_t                retention[1];   /**< COMP important register information. */

} comp_handle_t;
/** @} */

/** @} */

/** @addtogroup HAL_COMP_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup COMP_Callback COMP Callback
  * @{
  */

/**
  * @brief HAL COMP Callback function definition
  */
typedef struct _comp_callback
{
    void (*comp_msp_init)(comp_handle_t *p_comp);           /**< COMP init MSP callback                  */
    void (*comp_msp_deinit)(comp_handle_t *p_comp);         /**< COMP de-init MSP callback               */
    void (*comp_rising_trigger_callback)(comp_handle_t *p_comp);   /**< COMP rising trigger callback                     */
    void (*comp_falling_trigger_callback)(comp_handle_t *p_comp);   /**< COMP falling trigger callback                     */
} comp_callback_t;

/** @} */

/** @} */


/**
  * @defgroup  HAL_COMP_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup COMP_Exported_Constants COMP Exported Constants
  * @{
  */

/** @defgroup COMP_Error_Code COMP Error Code
  * @{
  */
#define HAL_COMP_ERROR_NONE            ((uint32_t)0x00000000)  /**< No error           */
#define HAL_COMP_ERROR_TIMEOUT         ((uint32_t)0x00000001)  /**< Timeout error      */
#define HAL_COMP_ERROR_INVALID_PARAM   ((uint32_t)0x00000002)  /**< Invalid parameters error */
/** @} */


/** @defgroup COMP_INPUT_SOURCE COMP Input Channel Select
  * @{
  */
#define COMP_INPUT_SRC_IO0           LL_COMP_INPUT_SRC_IO0    /**< Set MSIO_0 as inputs for the comparator */
#define COMP_INPUT_SRC_IO1           LL_COMP_INPUT_SRC_IO1    /**< Set MSIO_1 as inputs for the comparator */
#define COMP_INPUT_SRC_IO2           LL_COMP_INPUT_SRC_IO2    /**< Set MSIO_2 as inputs for the comparator */
#define COMP_INPUT_SRC_IO3           LL_COMP_INPUT_SRC_IO3    /**< Set MSIO_3 as inputs for the comparator */
#define COMP_INPUT_SRC_IO4           LL_COMP_INPUT_SRC_IO4    /**< Set MSIO_4 as inputs for the comparator */
#define COMP_INPUT_SRC_IO5           LL_COMP_INPUT_SRC_IO5    /**< Set MSIO_5 as inputs for the comparator */
#define COMP_INPUT_SRC_IO6           LL_COMP_INPUT_SRC_IO6    /**< Set MSIO_6 as inputs for the comparator */
#define COMP_INPUT_SRC_IO7           LL_COMP_INPUT_SRC_IO7    /**< Set MSIO_7 as inputs for the comparator */
#define COMP_INPUT_SRC_VBAT          LL_COMP_INPUT_SRC_VBAT   /**< Set VBATT as inputs for the comparator  */
#define COMP_INPUT_SRC_VREF          LL_COMP_INPUT_SRC_VREF   /**< Set VREF as inputs for the comparator   */
/** @} */

/** @defgroup COMP_REFERENCE_SOURCE COMP Reference Source Select
  * @{
  */
#define COMP_REF_SRC_IO0             LL_COMP_REF_SRC_IO0      /**< Set MSIO_0 as references for the comparator */
#define COMP_REF_SRC_IO1             LL_COMP_REF_SRC_IO1      /**< Set MSIO_1 as references for the comparator */
#define COMP_REF_SRC_IO2             LL_COMP_REF_SRC_IO2      /**< Set MSIO_2 as references for the comparator */
#define COMP_REF_SRC_IO3             LL_COMP_REF_SRC_IO3      /**< Set MSIO_3 as references for the comparator */
#define COMP_REF_SRC_IO4             LL_COMP_REF_SRC_IO4      /**< Set MSIO_4 as references for the comparator */
#define COMP_REF_SRC_IO5             LL_COMP_REF_SRC_IO5      /**< Set MSIO_5 as references for the comparator */
#define COMP_REF_SRC_IO6             LL_COMP_REF_SRC_IO6      /**< Set MSIO_6 as references for the comparator */
#define COMP_REF_SRC_IO7             LL_COMP_REF_SRC_IO7      /**< Set MSIO_7 as references for the comparator */
#define COMP_REF_SRC_VBAT            LL_COMP_REF_SRC_VBAT     /**< Set VBATT as references for the comparator  */
#define COMP_REF_SRC_VREF            LL_COMP_REF_SRC_VREF     /**< Set VREF as references for the comparator   */
/** @} */

/** @defgroup COMP_HYSTERESIS COMP Hysteresis Select
  * @{
  */
#define COMP_HYST_POSITIVE           LL_COMP_HYST_POSITIVE    /**< Set positive side of hysteresis for the comparator */
#define COMP_HYST_NEGATIVE           LL_COMP_HYST_NEGATIVE    /**< Set negative side of hysteresis for the comparator */
/** @} */

/** @defgroup COMP_WAKEUP_EDGE COMP Wakeup Edge Select
  * @{
  */
#define COMP_WAKEUP_EDGE_BOTH        LL_COMP_WAKEUP_EDGE_BOTH    /**< Select both rising and falling edges for the comparator */
#define COMP_WAKEUP_EDGE_FALLING     LL_COMP_WAKEUP_EDGE_FALLING /**< Select only falling edges for the comparator */
#define COMP_WAKEUP_EDGE_RISING      LL_COMP_WAKEUP_EDGE_RISING  /**< Select only rising edges for the comparator */

/** @} */

/** @defgroup COMP_RES_DEGENERATION COMP Reset Degeneration Select
  * @{
  */
#define COMP_RES_DEGENERATION_POSITIVE             LL_COMP_RES_DEGENERATION_POSITIVE      /**< Set positive side of calibration for the comparator */
#define COMP_RES_DEGENERATION_NEGATIVE             LL_COMP_RES_DEGENERATION_NEGATIVE      /**< Set negative side of calibration for the comparator */
/** @} */

/**
  * @brief Default configuartion for initializing structure
  */
#define COMP_DEFAULT_CONFIG         LL_COMP_DEFAULT_CONFIG
/** @} */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup COMP_Private_Macros COMP Private Macros
  * @{
  */

/**
  * @brief Check if COMP input source is valid.
  * @param __INPUT__ COMP input source.
  * @retval SET (__INPUT__ is valid) or RESET (__INPUT__ is invalid)
  */
#define IS_COMP_INPUT(__INPUT__)            (((__INPUT__) == COMP_INPUT_SRC_IO0) || \
                                             ((__INPUT__) == COMP_INPUT_SRC_IO1) || \
                                             ((__INPUT__) == COMP_INPUT_SRC_IO2) || \
                                             ((__INPUT__) == COMP_INPUT_SRC_IO3) || \
                                             ((__INPUT__) == COMP_INPUT_SRC_IO4) || \
                                             ((__INPUT__) == COMP_INPUT_SRC_IO5) || \
                                             ((__INPUT__) == COMP_INPUT_SRC_IO6) || \
                                             ((__INPUT__) == COMP_INPUT_SRC_IO7) || \
                                             ((__INPUT__) == COMP_INPUT_SRC_VBAT)|| \
                                             ((__INPUT__) == COMP_INPUT_SRC_VREF))
/**
  * @brief Check if COMP reference source is valid.
  * @param __INPUT__ COMP reference source.
  * @retval SET (__INPUT__ is valid) or RESET (__INPUT__ is invalid)
  */

#define IS_COMP_REF(__INPUT__)              (((__INPUT__) == COMP_REF_SRC_IO0)   || \
                                             ((__INPUT__) == COMP_REF_SRC_IO1)   || \
                                             ((__INPUT__) == COMP_REF_SRC_IO2)   || \
                                             ((__INPUT__) == COMP_REF_SRC_IO3)   || \
                                             ((__INPUT__) == COMP_REF_SRC_IO4)   || \
                                             ((__INPUT__) == COMP_REF_SRC_IO5)   || \
                                             ((__INPUT__) == COMP_REF_SRC_IO6)   || \
                                             ((__INPUT__) == COMP_REF_SRC_IO7)   || \
                                             ((__INPUT__) == COMP_REF_SRC_VBAT)  || \
                                             ((__INPUT__) == COMP_REF_SRC_VREF))

/**
  * @brief Check if COMP reference source is valid.
  * @param __INPUT__ COMP hysteresis.
  * @retval SET (__INPUT__ is valid) or RESET (__INPUT__ is invalid)
  */
#define IS_COMP_HYST_POS(__INPUT__)              ((__INPUT__) == COMP_HYST_POSITIVE)


/**
  * @brief Check if COMP reference source is invalid.
  * @param __INPUT__ COMP hysteresis.
  */
#define IS_COMP_HYST_NEG(__INPUT__)               ((__INPUT__) == COMP_HYST_NEGATIVE )

/**
  * @brief Check if COMP reference source is valid.
  * @param __INPUT__ COMP res degeneration.
  * @retval SET (__INPUT__ is valid) or RESET (__INPUT__ is invalid)
  */
#define IS_COMP_RES_DEGENERATION_POS(__INPUT__)              ((__INPUT__) == COMP_RES_DEGENERATION_POSITIVE)


/**
  * @brief Check if COMP reference source is invalid.
  * @param __INPUT__ COMP res degeneration.
  */
#define IS_COMP_RES_DEGENERATION_NEG(__INPUT__)               ((__INPUT__) == COMP_RES_DEGENERATION_NEGATIVE )
/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_COMP_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup COMP_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the COMP according to the specified parameters
 *         in the comp_init_t and initialize the associated handle.
 *
 * @param[in]  p_comp: Pointer to a COMP handle which contains the configuration information for
 *                    the specified COMP module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_comp_init(comp_handle_t *p_comp);

/**
 ****************************************************************************************
 * @brief  De-initialize the COMP peripheral.
 *
 * @param[in]  p_comp: Pointer to a COMP handle which contains the configuration information for
 *                    the specified COMP module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_comp_deinit(comp_handle_t *p_comp);

/**
 ****************************************************************************************
 * @brief  Initialize the COMP MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_comp_msp_deinit can be implemented in the user file.
 *
 * @param[in]  p_comp: Pointer to a COMP handle which contains the configuration information for
 *                the specified COMP module.
 ****************************************************************************************
 */
void hal_comp_msp_init(comp_handle_t *p_comp);

/**
 ****************************************************************************************
 * @brief  De-initialize the COMP MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_comp_msp_deinit can be implemented in the user file.
 *
 * @param[in]  p_comp: Pointer to a COMP handle which contains the configuration information for
 *                    the specified COMP module.
 ****************************************************************************************
 */
void hal_comp_msp_deinit(comp_handle_t *p_comp);

/** @} */

/** @addtogroup COMP_Exported_Functions_Group2 IO operation functions
 *  @brief COMP polling and DMA conversion management functions.
 * @{
 */

 /**
 ****************************************************************************************
 * @brief  Start the comparator.
 *
 * @param[in]  p_comp: Pointer to a COMP handle which contains the configuration information for
 *                    the specified COMP module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_comp_start(comp_handle_t *p_comp);

 /**
 ****************************************************************************************
 * @brief  Stop the comparator.
 *
 * @param[in]  p_comp: Pointer to a COMP handle which contains the configuration information for
 *                    the specified COMP module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_comp_stop(comp_handle_t *p_comp);

/** @} */

/** @addtogroup COMP_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief Handle COMP interrupt request.
 * @param[in] p_comp: Pointer to a COMP handle which contains the configuration information
 *                 for the specified COMP module.
 ****************************************************************************************
 */
void hal_comp_irq_handler(comp_handle_t *p_comp);

/**
 ****************************************************************************************
 * @brief  comparator callback.
 *
 * @note  This function should not be modified. When the callback is needed,
 *          the hal_comp_trigger_callback can be implemented in the user file.
 *
 * @param[in]  p_comp: Pointer to a COMP handle which contains the configuration information for
 *                    the specified COMP module.
 ****************************************************************************************
 */
void hal_comp_rising_trigger_callback(comp_handle_t *p_comp);
void hal_comp_falling_trigger_callback(comp_handle_t *p_comp);

/** @} */

/** @defgroup COMP_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   COMP control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the COMP.
     (+) hal_comp_get_state() API can be helpful to check in run-time the state of the COMP peripheral.
     (+) hal_comp_get_error() check in run-time Errors occurring during communication.
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the COMP handle state.
 *
 * @param[in]  p_comp: Pointer to a COMP handle which contains the configuration information for
 *                    the specified COMP module.
 *
 * @retval ::HAL_COMP_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_COMP_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_COMP_STATE_BUSY: An internal process is ongoing.
 * @retval ::HAL_COMP_STATE_ERROR: Peripheral in error.
 ****************************************************************************************
 */
hal_comp_state_t hal_comp_get_state(comp_handle_t *p_comp);

/**
 ****************************************************************************************
 * @brief  Return the COMP error code.
 *
 * @param[in]  p_comp: Pointer to a COMP handle which contains the configuration information for
 *                    the specified COMP module.
 *
 * @return COMP error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_comp_get_error(comp_handle_t *p_comp);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to COMP configuration before sleep.
 * @param[in] p_comp: Pointer to a COMP handle which contains the configuration
 *                 information for the specified COMP module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_comp_suspend_reg(comp_handle_t *p_comp);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to COMP configuration after sleep.
 *         This function must be used in conjunction with the hal_comp_suspend_reg().
 * @param[in] p_comp: Pointer to a COMP handle which contains the configuration
 *                 information for the specified COMP module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_comp_resume_reg(comp_handle_t *p_comp);

/** @} */

/** @} */



#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_COMP_H__ */

/** @} */

/** @} */

/** @} */

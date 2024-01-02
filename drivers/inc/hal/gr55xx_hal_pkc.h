/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_pkc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PKC HAL library.
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

/** @defgroup HAL_PKC PKC
  * @brief PKC HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_PKC_H__
#define __GR55xx_HAL_PKC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_pkc.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_PKC_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_PKC_state HAL PKC state
  * @{
  */

/**
  * @brief HAL PKC State Enumerations definition
  */
typedef enum
{
    HAL_PKC_STATE_RESET             = 0x00,    /**< Peripheral not initialized                            */
    HAL_PKC_STATE_READY             = 0x01,    /**< Peripheral initialized and ready for use              */
    HAL_PKC_STATE_BUSY              = 0x02,    /**< Peripheral in indirect mode and busy                  */
    HAL_PKC_STATE_ERROR             = 0x04,    /**< Peripheral in error                                   */
    HAL_PKC_STATE_TIMEOUT           = 0x08,    /**< Peripheral in timeout                                 */

} hal_pkc_state_t;

/** @} */

/** @} */

/** @addtogroup HAL_PKC_STRUCTURES Structures
  * @{
  */

/** @defgroup PKC_Configuration PKC Configuration
  * @{
  */

/**
  * @brief PKC ECC Point Structure definition
  */
typedef struct _ll_ecc_point ecc_point_t;

/**
  * @brief PKC ECC P-256 Elliptic Curve Init Structure definition
  */
typedef struct _ll_ecc_curve_init ecc_curve_init_t;

/**
  * @brief PKC Init Structure definition
  */
typedef struct
{
    ecc_curve_init_t *p_ecc_curve;            /**< Specifies the pointer to elliptic curve description */

    uint32_t data_bits;                     /**< Specifies the Data size: 256 ~ 2048 bits            */

    uint32_t secure_mode;                   /**< Specifies the Secure Mode. It indicates that DPA-resistance software algorithm
                                                 and hardware measures are applied at a cost of about 35%- 50% performance loss.
                                                 This parameter can be a value of @ref PKC_Secure_Mode. */

    uint32_t (*random_func)(void);          /**< Specifies the function to generate random number.   */

} pkc_init_t;

/** @} */

/** @defgroup PKC_handle PKC handle
  * @{
  */

/**
  * @brief PKC handle Structure definition
  */
typedef struct _pkc_handle
{
    pkc_regs_t              *p_instance;      /**< PKC registers base address       */

    pkc_init_t              init;           /**< PKC operation parameters         */

    void                    *p_result;      /**< Pointer to PKC result buffer     */

    uint32_t                shift_count;    /**< Count to left shift              */

    uint32_t                *p_P;           /**< Prime number                     */

    __IO hal_lock_t         lock;           /**< Locking object                   */

    __IO hal_pkc_state_t    state;          /**< PKC operation state              */

    __IO uint32_t           error_code;     /**< PKC Error code                   */

    uint32_t                timeout;        /**< Timeout for the PKC operation    */

    uint32_t                retention[1];  /**< pkc important register information. */
} pkc_handle_t;
/** @} */

/** @defgroup PKC_Expression_Input PKC expression input
  * @{
  */

/**
  * @brief PKC ECC Point Multiplication expression input
  * @note  Result = K * Point
  */
typedef struct _pkc_ecc_point_multi
{
    uint32_t *p_K;                            /**< Pointer to operand K */
    ecc_point_t *p_ecc_point;                 /**< Pointer to ECC Point */
} pkc_ecc_point_multi_t;

/**
  * @brief PKC RSA Modular Exponentiation expression input
  * @note  Result = A^B mod P
  */
typedef struct _pkc_rsa_modular_exponent
{
    uint32_t *p_A;                            /**< Pointer to operand A */
    uint32_t *p_B;                            /**< Pointer to operand B */
    uint32_t *p_P;                            /**< Pointer to prime number P */
    uint32_t *p_P_R2;                         /**< P_R2 = R^2 mod P, where R = 2^DataBits  */
    uint32_t ConstP;                        /**< Montgomery multiplication constant of P */
} pkc_rsa_modular_exponent_t;

/**
  * @brief PKC Modular Addition expression input
  * @note  Result = (A + B) mod P
  */
typedef struct _pkc_modular_add
{
    uint32_t *p_A;                            /**< Pointer to operand A */
    uint32_t *p_B;                            /**< Pointer to operand B */
    uint32_t *p_P;                            /**< Pointer to prime number P */
} pkc_modular_add_t;

/**
  * @brief PKC Modular Subtraction expression input
  * @note  Result = (A - B) mod P
  */
typedef struct _pkc_modular_sub
{
    uint32_t *p_A;                            /**< Pointer to operand A */
    uint32_t *p_B;                            /**< Pointer to operand B */
    uint32_t *p_P;                            /**< Pointer to prime number P */
} pkc_modular_sub_t;

/**
  * @brief PKC Modular Left Shift expression input
  * @note  Result = (A << ShiftBits) mod P
  */
typedef struct _pkc_modular_shift
{
    uint32_t *p_A;                            /**< Pointer to operand A */
    uint32_t shift_bits;                    /**< Pointer to operand A */
    uint32_t *p_P;                            /**< Pointer to prime number P */
} pkc_modular_shift_t;

/**
  * @brief PKC Modular Comparison expression input
  * @note  Result = A mod P
  */
typedef struct _pkc_modular_compare
{
    uint32_t *p_A;                            /**< Pointer to operand A */
    uint32_t *p_P;                            /**< Pointer to prime number P */
} pkc_modular_compare_t;

/**
  * @brief PKC Montgomery Modular Multiplication expression input
  * @note  Result = A * B * R^(-1) mod P, where R = 2^DataBits
  */
typedef struct _pkc_montgomery_multi
{
    uint32_t *p_A;                            /**< Pointer to operand A */
    uint32_t *p_B;                            /**< Pointer to operand B */
    uint32_t *p_P;                            /**< Pointer to prime number P */
    uint32_t ConstP;                        /**< Montgomery multiplication constant for P,
                                                 where constp = (-P[0])^(-1) mod 2^32 */
} pkc_montgomery_multi_t;

/**
  * @brief PKC Montgomery Inversion expression input
  * @note  Result = A^(-1) * 2^(K) mod P
  */
typedef struct _pkc_montgomery_inversion
{
    uint32_t *p_A;                            /**< Pointer to operand A */
    uint32_t *p_P;                            /**< Pointer to prime number P */
    uint32_t ConstP;                          /**< Montgomery multiplication constant for P,
                                                   where ConstP = (-P[0])^(-1) mod 2^32 */
} pkc_montgomery_inversion_t;

/**
  * @brief PKC Big Number Multiplication expression input
  * @note  Result = A * B, up to 1024 bits
  */
typedef struct _pkc_big_number_multi
{
    uint32_t *p_A;                            /**< Pointer to operand A */
    uint32_t *p_B;                            /**< Pointer to operand B */
} pkc_big_number_multi_t;

/**
  * @brief PKC Big Number Addition expression input
  * @note  Result = A + B, up to 2048 bits
  */
typedef struct _pkc_big_number_add
{
    uint32_t *p_A;                            /**< Pointer to operand A */
    uint32_t *p_B;                            /**< Pointer to operand B */
} pkc_big_number_add_t;

/** @} */

/** @} */

/** @addtogroup HAL_PKC_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup PKC_Callback PKC Callback
  * @{
  */

/**
  * @brief HAL_PKC Callback function definition
  */

typedef struct _pkc_callback
{
    void (*pkc_msp_init)(pkc_handle_t *p_pkc);              /**< PKC init MSP callback                  */
    void (*pkc_msp_deinit)(pkc_handle_t *p_pkc);            /**< PKC de-init MSP callback               */
    void (*pkc_done_callback)(pkc_handle_t *p_pkc);         /**< PKC calculate done callback            */
    void (*pkc_error_callback)(pkc_handle_t *p_pkc);        /**< PKC error callback                     */
    void (*pkc_overflow_callback)(pkc_handle_t *p_pkc);     /**< PKC over flow callback                 */
} pkc_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_PKC_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PKC_Exported_Constants PKC Exported Constants
  * @{
  */

/** @defgroup PKC_Error_Code PKC Error Code
  * @{
  */
#define HAL_PKC_ERROR_NONE             ((uint32_t)0x00000000)   /**< No error                   */
#define HAL_PKC_ERROR_TIMEOUT          ((uint32_t)0x00000001)   /**< Timeout error              */
#define HAL_PKC_ERROR_TRANSFER         ((uint32_t)0x00000002)   /**< Transfer error             */
#define HAL_PKC_ERROR_OVERFLOW         ((uint32_t)0x00000004)   /**< Result overflow error      */
#define HAL_PKC_ERROR_INVALID_PARAM    ((uint32_t)0x00000008)   /**< Invalid parameters error   */
#define HAL_PKC_ERROR_INVERSE_K        ((uint32_t)0x00000010)   /**< Inverse K error            */
#define HAL_PKC_ERROR_IRREVERSIBLE     ((uint32_t)0x00000020)   /**< Irreversible error         */
/** @} */

/** @defgroup PKC_Secure_Mode PKC Secure Mode
  * @{
  */
#define PKC_SECURE_MODE_DISABLE        ((uint32_t)0x00000000)   /**< Secure mode disable */
#define PKC_SECURE_MODE_ENABLE         ((uint32_t)0x00000001)   /**< Secure mode enable  */
/** @} */

/** @defgroup PKC_Operation_Mode PKC Operation Mode
  * @{
  */
#define PKC_OPERATION_MODE_MULTI       LL_PKC_operation_mode_MULTIPLY            /**< Multiplication operation mode              */
#define PKC_OPERATION_MODE_INVER       LL_PKC_operation_mode_INVERTION           /**< Inversion operation mode                   */
#define PKC_OPERATION_MODE_ADD         LL_PKC_operation_mode_ADD                 /**< Addition operation mode                    */
#define PKC_OPERATION_MODE_SUB         LL_PKC_operation_mode_SUB                 /**< Subtraction operation mode                 */
#define PKC_OPERATION_MODE_CMP         LL_PKC_operation_mode_COMPARE             /**< Comparison operation mode                  */
#define PKC_OPERATION_MODE_LSHIFT      LL_PKC_operation_mode_LEFTSHIFT           /**< Left Shift operation mode                  */
#define PKC_OPERATION_MODE_BIGMULTI    LL_PKC_operation_mode_BIGINTEGERMULTIPLY  /**< Big Number Multiplication operation mode   */
#define PKC_OPERATION_MODE_BIGADD      LL_PKC_operation_mode_BIGINTEGERADD       /**< Big Number Addition operation mode         */
/** @} */

/** @defgroup PKC_Bits_Length PKC Bits Length
  * @{
  */
#define PKC_BITS_LENGTH_MIN            LL_PKC_BITS_LENGTH_MIN           /**< Min value of bits length  */
#define PKC_BITS_LENGTH_MAX            LL_PKC_BITS_LENGTH_MAX           /**< Max value of bits length  */
#define PKC_BIGMULTI_BITS_LENGTH_MAX   LL_PKC_BIGMULTI_BITS_LENGTH_MAX  /**< Max value of big number multiplication bits length */
/** @} */

/** @defgroup PKC_Flags PKC Flags
  * @{
  */
#define PKC_FLAG_BUSY                  LL_PKC_WORKSTAT_BUSY       /**< Busy flag */
/** @} */

/** @defgroup PKC_Interrupt_definition PKC Interrupt_definition
  * @{
  */
#define PKC_IT_DONE                    LL_PKC_INTEN_DONE          /**< Operation Done Interrupt source              */
#define PKC_IT_ERR                     LL_PKC_INTEN_ERR           /**< Operation Error Interrupt source             */
#define PKC_IT_OVF                     LL_PKC_INTEN_BAOVF         /**< Big Integer Result Overflow Interrupt source */
/** @} */

/** @defgroup PKC_Timeout_definition PKC Timeout_definition
  * @{
  */
#define HAL_PKC_TIMEOUT_DEFAULT_VALUE ((uint32_t)5000)            /**< The default value of PKC timeout is 5s */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup PKC_Exported_Macros PKC Exported Macros
  * @{
  */

/** @brief  Reset PKC handle states.
  * @param  __HANDLE__ PKC handle.
  * @retval None
  */
#define __HAL_PKC_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_PKC_STATE_RESET)

/** @brief  Reset the specified PKC peripheral.
  * @param  __HANDLE__ PKC handle.
  * @retval None
  */

#define __HAL_PKC_RESET(__HANDLE__)                            CLEAR_BITS((__HANDLE__)->p_instance->CTRL, PKC_CTRL_RST); \
                                                               SET_BITS((__HANDLE__)->p_instance->CTRL, PKC_CTRL_RST)

/** @brief  Enable the specified PKC peripheral.
  * @param  __HANDLE__ Specifies the PKC Handle.
  * @retval None
  */
#define __HAL_PKC_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->CTRL, PKC_CTRL_EN)

/** @brief  Disable the specified PKC peripheral.
  * @param  __HANDLE__ Specifies the PKC Handle.
  * @retval None
  */
#define __HAL_PKC_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->CTRL, PKC_CTRL_EN)

/** @brief  Enable the specified PKC interrupts.
  * @param  __HANDLE__ Specifies the PKC Handle.
  * @param  __INTERRUPT__ Specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg @ref PKC_IT_DONE Operation Done Interrupt source
  *            @arg @ref PKC_IT_ERR  Operation Error Interrupt source
  *            @arg @ref PKC_IT_OVF  Big Integer Result Overflow Interrupt source
  * @retval None
  */

#define __HAL_PKC_ENABLE_IT(__HANDLE__, __INTERRUPT__)         SET_BITS((__HANDLE__)->p_instance->INT_EN, (__INTERRUPT__))

/** @brief  Disable the specified PKC interrupts.
  * @param  __HANDLE__ Specifies the PKC Handle.
  * @param  __INTERRUPT__ Specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg @ref PKC_IT_DONE Operation Done Interrupt source
  *            @arg @ref PKC_IT_ERR  Operation Error Interrupt source
  *            @arg @ref PKC_IT_OVF  Big Integer Result Overflow Interrupt source
  * @retval None
  */

#define __HAL_PKC_DISABLE_IT(__HANDLE__, __INTERRUPT__)        CLEAR_BITS((__HANDLE__)->p_instance->INT_EN, (__INTERRUPT__))

/** @brief  Check whether the specified PKC interrupt flag is set or not.
  * @param  __HANDLE__ Specifies the PKC Handle.
  * @param  __FLAG__ Specifies the interrupt flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref PKC_IT_DONE Operation Done Interrupt source
  *            @arg @ref PKC_IT_ERR  Operation Error Interrupt source
  *            @arg @ref PKC_IT_OVF  Big Integer Result Overflow Interrupt source
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */

#define __HAL_PKC_GET_FLAG_IT(__HANDLE__, __FLAG__)            (READ_BITS((__HANDLE__)->p_instance->INT_STAT, (__FLAG__)) == (__FLAG__))

/** @brief  Clear the specified PKC interrupt flag.
  * @param  __HANDLE__ Specifies the PKC Handle.
  * @param  __FLAG__ Specifies the interrupt flag to clear.
  *         This parameter can be one of the following values:
  *            @arg @ref PKC_IT_DONE Operation Done Interrupt source
  *            @arg @ref PKC_IT_ERR  Operation Error Interrupt source
  *            @arg @ref PKC_IT_OVF  Big Integer Result Overflow Interrupt source
  * @retval None
  */

#define __HAL_PKC_CLEAR_FLAG_IT(__HANDLE__, __FLAG__)          SET_BITS((__HANDLE__)->p_instance->INT_STAT, (__FLAG__))

/** @brief  Check whether the specified PKC flag is set or not.
  * @param  __HANDLE__ Specifies the PKC Handle.
  * @param  __FLAG__ Specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref PKC_FLAG_BUSY Busy flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */

#define __HAL_PKC_GET_FLAG(__HANDLE__, __FLAG__)               ((READ_BITS((__HANDLE__)->p_instance->STAT, (__FLAG__)) != 0) ? SET : RESET)

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup PKC_Private_Macro PKC Private Macros
  * @{
  */

/** @brief  Check if PKC Bits Length is valid.
  * @param  __BITS__ PKC Bits Length.
  * @retval SET (__BITS__ is valid) or RESET (__BITS__ is invalid)
  */
#define IS_PKC_BITS_LENGTH(__BITS__)            (((__BITS__) >= PKC_BITS_LENGTH_MIN) && ((__BITS__) <= PKC_BITS_LENGTH_MAX))

/** @brief  Check if PKC Big Number Multiplication Bits Length is valid.
  * @param  __BITS__ PKC Big Number Multiplication Bits Length.
  * @retval SET (__BITS__ is valid) or RESET (__BITS__ is invalid)
  */
#define IS_PKC_BIGMULTI_BITS_LENGTH(__BITS__)   (((__BITS__) >= PKC_BITS_LENGTH_MIN) && ((__BITS__) <= PKC_BIGMULTI_BITS_LENGTH_MAX))

/** @brief  Check if PKC Secure Mode is valid.
  * @param  __MODE__ PKC Secure Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_PKC_SECURE_MODE(__MODE__)            (((__MODE__) == PKC_SECURE_MODE_DISABLE) || \
                                                 ((__MODE__) == PKC_SECURE_MODE_ENABLE))

/** @brief  Check if PKC Operation Mode is valid.
  * @param  __MODE__ PKC Operation Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_PKC_OPERATION_MODE(__MODE__)         (((__MODE__) == PKC_OPERATION_MODE_MULTI)    || \
                                                 ((__MODE__) == PKC_OPERATION_MODE_INVER)    || \
                                                 ((__MODE__) == PKC_OPERATION_MODE_ADD)      || \
                                                 ((__MODE__) == PKC_OPERATION_MODE_SUB)      || \
                                                 ((__MODE__) == PKC_OPERATION_MODE_CMP)      || \
                                                 ((__MODE__) == PKC_OPERATION_MODE_LSHIFT)   || \
                                                 ((__MODE__) == PKC_OPERATION_MODE_BIGMULTI) || \
                                                 ((__MODE__) == PKC_OPERATION_MODE_BIGADD))

/** @} */

/** @} */



/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_PKC_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup PKC_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the PKC peripheral:

      (+) User must implement hal_pkc_msp_init() function in which it configures
          all related peripherals resources (IT and NVIC ).

      (+) Call the function hal_pkc_init() to configure the selected device with
          the selected configuration:
        (++) pECC_Curve
        (++) DataBits
        (++) SecureMode
        (++) pRandomFunc

      (+) Call the function hal_pkc_deinit() to restore the default configuration
          of the selected PKC peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the PKC according to the specified parameters
 *         in the pkc_init_t and initialize the associated handle.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *               information for the specified PKC module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_init(pkc_handle_t *p_pkc);

/**
 ****************************************************************************************
 * @brief  De-initialize the PKC peripheral.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *               information for the specified PKC module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_deinit(pkc_handle_t *p_pkc);

/**
 ****************************************************************************************
 * @brief  Initialize the PKC MSP.
 * @note   This function should not be modified. When the callback is needed,
            the hal_pkc_msp_deinit can be implemented in the user file.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *               information for the specified PKC module.
 ****************************************************************************************
 */
void hal_pkc_msp_init(pkc_handle_t *p_pkc);

/**
 ****************************************************************************************
 * @brief  De-initialize the PKC MSP.
 * @note   This function should not be modified. When the callback is needed,
            the hal_pkc_msp_deinit can be implemented in the user file.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *               information for the specified PKC module.
 ****************************************************************************************
 */
void hal_pkc_msp_deinit(pkc_handle_t *p_pkc);

/** @} */

/** @defgroup PKC_Exported_Functions_Group2 IO Operation Functions
 *  @brief   Data transfers functions
 *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the PKC
    data transfers.

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts
            , These APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated PKC IRQ when using Interrupt mode.
            The hal_pkc_done_callback() user callbacks will be executed respectively at the end of the calculate process
            The hal_pkc_error_callback() user callback will be executed when a communication error is detected

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Execute RSA Modular Exponentiation in blocking mode.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_rsa_modular_exponent(pkc_handle_t *p_pkc, pkc_rsa_modular_exponent_t *p_input, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Execute ECC Point Multiplication in blocking mode.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_ecc_point_multi(pkc_handle_t *p_pkc, pkc_ecc_point_multi_t *p_input, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Execute ECC Point Multiplication in non-blocking mode with Interrupt.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_ecc_point_multi_it(pkc_handle_t *p_pkc, pkc_ecc_point_multi_t *p_input);

/**
 ****************************************************************************************
 * @brief  Execute Modular Addition in blocking mode.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_modular_add(pkc_handle_t *p_pkc, pkc_modular_add_t *p_input, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Execute Modular Addition in non-blocking mode with Interrupt.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_modular_add_it(pkc_handle_t *p_pkc, pkc_modular_add_t *p_input);

/**
 ****************************************************************************************
 * @brief  Execute Modular Subtraction in blocking mode.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_modular_sub(pkc_handle_t *p_pkc, pkc_modular_sub_t *p_input, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Execute Modular Subtraction in non-blocking mode with Interrupt.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_modular_sub_it(pkc_handle_t *p_pkc, pkc_modular_sub_t *p_input);

/**
 ****************************************************************************************
 * @brief  Execute Modular Left Shift in blocking mode.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_modular_left_shift(pkc_handle_t *p_pkc, pkc_modular_shift_t *p_input, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Execute Modular Left Shift in non-blocking mode with Interrupt.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_modular_left_shift_it(pkc_handle_t *p_pkc, pkc_modular_shift_t *p_input);

/**
 ****************************************************************************************
 * @brief  Execute Modular Comparison in blocking mode.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_modular_compare(pkc_handle_t *p_pkc, pkc_modular_compare_t *p_input, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Execute Modular Comparison in non-blocking mode with Interrupt.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_modular_compare_it(pkc_handle_t *p_pkc, pkc_modular_compare_t *p_input);

/**
 ****************************************************************************************
 * @brief  Execute Montgomery Modular Multiplication in blocking mode.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_montgomery_multi(pkc_handle_t *p_pkc, pkc_montgomery_multi_t *p_input, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Execute Montgomery Modular Multiplication in non-blocking mode with Interrupt.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_montgomery_multi_it(pkc_handle_t *p_pkc, pkc_montgomery_multi_t *p_input);

/**
 ****************************************************************************************
 * @brief  Execute Montgomery Inversion in blocking mode.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_montgomery_inversion(pkc_handle_t *p_pkc, pkc_montgomery_inversion_t *p_input, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Execute Montgomery Inversion in non-blocking mode with Interrupt.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_montgomery_inversion_it(pkc_handle_t *p_pkc, pkc_montgomery_inversion_t *p_input);

/**
 ****************************************************************************************
 * @brief  Execute Big Number Multiplication in blocking mode.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_big_number_multi(pkc_handle_t *p_pkc, pkc_big_number_multi_t *p_input, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Execute Big Number Multiplication in non-blocking mode with Interrupt.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_big_number_multi_it(pkc_handle_t *p_pkc, pkc_big_number_multi_t *p_input);

/**
 ****************************************************************************************
 * @brief  Execute Big Number Addition in blocking mode.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_big_number_add(pkc_handle_t *p_pkc, pkc_big_number_add_t *p_input, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Execute Big Number Addition in non-blocking mode with Interrupt.
 * @note   The computed result will be stored in the buffter pointed by p_pkc->pResult.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @param[in]  p_input: Pointer to an expression structure which contains the input computing parameters.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pkc_big_number_add_it(pkc_handle_t *p_pkc, pkc_big_number_add_t *p_input);

/** @} */

/** @addtogroup PKC_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle PKC interrupt request.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *               information for the specified PKC module.
 ****************************************************************************************
 */
void hal_pkc_irq_handler(pkc_handle_t *p_pkc);

/**
 ****************************************************************************************
 * @brief  PKC calculate done callback.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *               information for the specified PKC module.
 ****************************************************************************************
 */
void hal_pkc_done_callback(pkc_handle_t *p_pkc);

/**
 ****************************************************************************************
 * @brief  PKC error callback.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *               information for the specified PKC module.
 ****************************************************************************************
 */
void hal_pkc_error_callback(pkc_handle_t *p_pkc);

/**
 ****************************************************************************************
 * @brief  PKC over flow callback.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *               information for the specified PKC module.
 ****************************************************************************************
 */
void hal_pkc_overflow_callback(pkc_handle_t *p_pkc);

/** @} */

/** @addtogroup PKC_Exported_Functions_Group3 Peripheral Control and State functions
 *  @brief   PKC Peripheral State functions
 *
@verbatim
  ==============================================================================
            ##### Peripheral Control and State functions #####
  ==============================================================================
    [..]
    This subsection provides functions allowing to :
      (+) Return the PKC handle state.
      (+) Return the PKC handle error code.
      (+) Set the timeout during internal process.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the PKC handle state.
 * @param[in]  p_pkc: Pointer to a pkc_handle_t structure that contains
 *              the configuration information for the specified PKC.
 * @retval ::HAL_PKC_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_PKC_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_PKC_STATE_BUSY: Peripheral in indirect mode and busy.
 * @retval ::HAL_PKC_STATE_ERROR: Peripheral in error.
 * @retval ::HAL_PKC_STATE_TIMEOUT: Peripheral in timeout.
 ****************************************************************************************
 */
hal_pkc_state_t hal_pkc_get_state(pkc_handle_t *p_pkc);

/**
 ****************************************************************************************
 * @brief  Return the PKC error code.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *               information for the specified PKC module.
 * @return PKC error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_pkc_get_error(pkc_handle_t *p_pkc);

/**
 ****************************************************************************************
 * @brief  Set the PKC internal process timeout value.
 * @param[in]  p_pkc: Pointer to a PKC handle which contains the configuration
 *                  information for the specified PKC module.
 * @param[in]  timeout: Internal process timeout value.
 ****************************************************************************************
 */
void hal_pkc_set_timeout(pkc_handle_t *p_pkc, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to PKC configuration before sleep.
 * @param[in] p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */

hal_status_t hal_pkc_suspend_reg(pkc_handle_t *p_pkc);
/**
 ****************************************************************************************
 * @brief  Restore some registers related to PKC configuration after sleep.
 *         This function must be used in conjunction with the hal_hmac_suspend_reg().
 * @param[in] p_pkc: Pointer to a PKC handle which contains the configuration
 *                 information for the specified PKC module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */

hal_status_t hal_pkc_resume_reg(pkc_handle_t *p_pkc);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_PKC_H__ */

/** @} */

/** @} */

/** @} */

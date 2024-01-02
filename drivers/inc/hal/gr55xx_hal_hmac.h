/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_hmac.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of HMAC HAL library.
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

/** @defgroup HAL_HMAC HMAC
  * @brief HMAC HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_HMAC_H__
#define __GR55xx_HAL_HMAC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_hmac.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_HMAC_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_HMAC_state HAL HMAC state
  * @{
  */

/**
  * @brief HAL HMAC State enumerations definition
  * @note  HAL HMAC State value is a combination of 2 different substates: gState and RxState.
  */
typedef enum
{
    HAL_HMAC_STATE_RESET             = 0x00,    /**< Peripheral not initialized                            */
    HAL_HMAC_STATE_READY             = 0x01,    /**< Peripheral initialized and ready for use              */
    HAL_HMAC_STATE_BUSY              = 0x02,    /**< Peripheral in indirect mode and busy                  */
    HAL_HMAC_STATE_ERROR             = 0x03,    /**< Peripheral in error                                   */
    HAL_HMAC_STATE_TIMEOUT           = 0x04,    /**< Peripheral in timeout                                 */
    HAL_HMAC_STATE_SUSPENDED         = 0x05,    /**< Peripheral in suspended                               */

} hal_hmac_state_t;

/** @} */

/** @} */

/** @addtogroup HAL_HMAC_STRUCTURES Structures
  * @{
  */

/** @defgroup HMAC_Configuration HMAC Configuration
  * @{
  */

/**
  * @brief HMAC init Structure definition
  */
typedef struct _hmac_init
{
    uint32_t mode;                /**< Operating mode             */

    uint32_t *p_key;              /**< Encryption/Decryption Key  */

    uint32_t *p_user_hash;        /**< Initialization HASH value  */

    uint32_t key_fetch_type;      /**< Fetch key type.
                                       This parameter can be a value of @ref HMAC_HAL_KEY_TYPE */

    uint32_t dpa_mode;            /**< DPA Mode                   */

} hmac_init_t;
/** @} */

/** @defgroup HMAC_handle HMAC handle
  * @{
  */

/**
  * @brief HMAC handle Structure definition
  */
typedef struct _hmac_handle
{
    hmac_regs_t           *p_instance;        /**< HMAC registers base address              */

    hmac_init_t           init;             /**< HMAC operation parameters                */

    uint32_t              *p_message;         /**< Pointer to message input buffer          */

    uint32_t              *p_digest;          /**< Pointer to digest output buffer          */

    uint32_t              block_size;       /**< Data size in blocks (64 bytes per block)  */

    uint32_t              block_count;      /**< Blocks count                             */

    uint32_t              is_last_trans;    /**< Flag for last transfer                   */

    __IO hal_lock_t       lock;             /**< Locking object                           */

    __IO hal_hmac_state_t state;            /**< HMAC operation state                     */

    __IO uint32_t         error_code;       /**< HMAC Error code                          */

    uint32_t              timeout;          /**< Timeout for the HMAC operation           */

    uint32_t              retention[17];    /**< HMAC important register information. */

} hmac_handle_t;

/** @} */

/** @} */

/** @addtogroup HAL_HMAC_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HMAC_Callback HMAC Callback
  * @{
  */

/**
  * @brief HAL_HMAC Callback function definition
  */

typedef struct _hmac_callback
{
    void (*hmac_msp_init)(hmac_handle_t *p_hmac);               /**< HMAC init MSP callback             */
    void (*hmac_msp_deinit)(hmac_handle_t *p_hmac);             /**< HMAC de-init MSP callback          */
    void (*hmac_done_callback)(hmac_handle_t *p_hmac);          /**< HMAC digest done callback          */
    void (*hmac_error_callback)(hmac_handle_t *p_hmac);         /**< HMAC error callback                */
} hmac_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_HMAC_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup HMAC_Exported_Constants HMAC Exported Constants
  * @{
  */

/** @defgroup HMAC_Error_Code HMAC Error Code
  * @{
  */
#define HAL_HMAC_ERROR_NONE             ((uint32_t)0x00000000)          /**< No error           */
#define HAL_HMAC_ERROR_TIMEOUT          ((uint32_t)0x00000001)          /**< Timeout error      */
#define HAL_HMAC_ERROR_TRANSFER         ((uint32_t)0x00000002)          /**< Transfer error     */
#define HAL_HMAC_ERROR_INVALID_PARAM    ((uint32_t)0x00000004)          /**< Invalid parameters error */
/** @} */

/** @defgroup HMAC_Mode HMAC Mode
  * @{
  */
#define HMAC_MODE_SHA                   LL_HMAC_CALCULATETYPE_SHA       /**< SHA mode  */
#define HMAC_MODE_HMAC                  LL_HMAC_CALCULATETYPE_HMAC      /**< HMAC mode */
/** @} */

/** @defgroup HMAC_Block_Size HAMC Block Size
  * @{
  */
#define HMAC_BLOCK_MAX                  (512)                           /**< Block max size        */
#define HMAC_BLOCKSIZE_BITS             (512)                           /**< Block size in bits    */
#define HMAC_BLOCKSIZE_BYTES            (HMAC_BLOCKSIZE_BITS >> 3)      /**< Block size in bytes   */
#define HMAC_BLOCKSIZE_WORDS            (HMAC_BLOCKSIZE_BYTES >> 2)     /**< Block size in words   */
#define HMAC_DIGESTSIZE_BITS            (256)                           /**< Digest size in bits   */
#define HMAC_DIGESTSIZE_BYTES           (HMAC_DIGESTSIZE_BITS >> 3)     /**< Digest size in bytes  */
#define HMAC_DIGESTSIZE_WORDS           (HMAC_DIGESTSIZE_BYTES >> 2)    /**< Digest size in words  */
#define HMAC_DMA_BLOCK_MAX              (512)                           /**< DMA Block max size    */
/** @} */

/** @defgroup HMAC_Flags_definition HAMC Flags Definition
  * @{
  */
#define HMAC_FLAG_DATAREADY_SHA         LL_HMAC_FLAG_DATAREADY_SHA      /**< HMAC data ready (SHA mode)  */
#define HMAC_FLAG_DATAREADY_HMAC        LL_HMAC_FLAG_DATAREADY_HMAC     /**< HMAC data ready (HAMC mode) */
#define HMAC_FLAG_DMA_MESSAGEDONE       LL_HMAC_FLAG_DMA_MESSAGEDONE    /**< HMAC DMA message done       */
#define HMAC_FLAG_DMA_DONE              LL_HMAC_FLAG_DMA_DONE           /**< HMAC DMA transfer done      */
#define HMAC_FLAG_DMA_ERR               LL_HMAC_FLAG_DMA_ERR            /**< HMAC DMA transfer error     */
#define HMAC_FLAG_KEY_VALID             LL_HMAC_FLAG_KEY_VALID          /**< HMAC has fetched key        */
/** @} */

/** @defgroup HMAC_HAL_KEY_TYPE Key Type
  * @{
  */
#define HAL_HMAC_KEYTYPE_MCU            LL_HMAC_KEYTYPE_MCU             /**< Key from MCU        */
/* { Start_private */
#define HAL_HMAC_KEYTYPE_AHB            LL_HMAC_KEYTYPE_AHB             /**< Key from AHB master */
/* } End_private   */
#define HAL_HMAC_KEYTYPE_KRAM           LL_HMAC_KEYTYPE_KRAM            /**< Key from Key Port   */
/** @} */

/** @defgroup HMAC_Interrupt_definition HMAC Interrupt_definition
  * @{
  */
#define HMAC_IT_DONE                    ((uint32_t)0x00000001)          /**< Operation Done Interrupt source              */
/** @} */

/** @defgroup HMAC_Timeout_definition HMAC Timeout_definition
  * @{
  */
#define HAL_HMAC_TIMEOUT_DEFAULT_VALUE  ((uint32_t)5000)                /**< 5s */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup HMAC_Exported_Macros HMAC Exported Macros
  * @{
  */

/** @brief  Reset HMAC handle states.
  * @param  __HANDLE__ HMAC handle.
  * @retval None
  */
#define __HAL_HMAC_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_HMAC_STATE_RESET)

/** @brief  Enable the specified HMAC peripheral.
  * @param  __HANDLE__ Specifies the HMAC Handle.
  * @retval None
  */

#define __HAL_HMAC_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->CTRL, HMAC_CTRL_EN)
/** @brief  Disable the specified HMAC peripheral.
  * @param  __HANDLE__ Specifies the HMAC Handle.
  * @retval None
  */

#define __HAL_HMAC_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->CTRL, HMAC_CTRL_EN)

/** @brief  Enable the HMAC interrupt.
  * @param  __HANDLE__ Specifies the HMAC Handle.
  * @retval None
  */
#define __HAL_HMAC_ENABLE_IT(__HANDLE__)                        ll_hmac_enable_it_done((__HANDLE__)->p_instance)

/** @brief  Disable the HMAC interrupt.
  * @param  __HANDLE__ Specifies the HMAC Handle.
  * @retval None
  */
#define __HAL_HMAC_DISABLE_IT(__HANDLE__)                       ll_hmac_disable_it_done((__HANDLE__)->p_instance)

/** @brief  Check whether the specified HMAC interrupt flag is set or not.
  * @param  __HANDLE__  Specifies the HMAC Handle.
  * @param  __FLAG__    Specifies the interrupt flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref HMAC_IT_DONE Encrypted or Decrypted Data Done Interrupt
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */

#define __HAL_HMAC_GET_FLAG_IT(__HANDLE__, __FLAG__)            (READ_BITS((__HANDLE__)->p_instance->INT, (__FLAG__)) == (__FLAG__))

/** @brief  Clear the specified HMAC interrupt flag.
  * @param  __HANDLE__  Specifies the HMAC interrupt Handle.
  * @param  __FLAG__    Specifies the flag to clear.
  *         This parameter can be one of the following values:
  *            @arg @ref HMAC_IT_DONE Encrypted or Decrypted Data Done Interrupt
  * @retval None
  */

#define __HAL_HMAC_CLEAR_FLAG_IT(__HANDLE__, __FLAG__)          SET_BITS((__HANDLE__)->p_instance->INT, (__FLAG__))

/** @brief  Check whether the specified HMAC flag is set or not.
  * @param  __HANDLE__  Specifies the HMAC Handle.
  * @param  __FLAG__    Specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref HMAC_FLAG_DATAREADY_SHA Data ready (SHA mode) flag
  *            @arg @ref HMAC_FLAG_DATAREADY_HMAC Data ready (HMAC mode) flag
  *            @arg @ref HMAC_FLAG_DMA_DONE DMA transfer done flag
  *            @arg @ref HMAC_FLAG_DMA_ERR DMA transfer error flag
  *            @arg @ref HMAC_FLAG_KEY_VALID Key valid flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */

#define __HAL_HMAC_GET_FLAG(__HANDLE__, __FLAG__)               ((READ_BITS((__HANDLE__)->p_instance->STAT, (__FLAG__)) != 0) ? SET : RESET)

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_HMAC_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup HMAC_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the HMACx peripheral:

      (+) User must implement hal_hmac_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_hmac_init() to configure the selected device with
          the selected configuration:
        (++) mode
        (++) key
        (++) user_hash
        (++) dpa_mode

      (+) Call the function hal_hmac_deinit() to restore the default configuration
          of the selected HMACx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the HMAC according to the specified parameters
 *         in the hmac_init_t and initialize the associated handle.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_hmac_init(hmac_handle_t *p_hmac);

/**
 ****************************************************************************************
 * @brief  De-initialize the HMAC peripheral.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_hmac_deinit(hmac_handle_t *p_hmac);

/**
 ****************************************************************************************
 * @brief  Initialize the HMAC MSP.
 * @note   This function should not be modified. When the callback is needed,
            the hal_hmac_msp_deinit can be implemented in the user file.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 ****************************************************************************************
 */
void hal_hmac_msp_init(hmac_handle_t *p_hmac);

/**
 ****************************************************************************************
 * @brief  De-initialize the HMAC MSP.
 * @note   This function should not be modified. When the callback is needed,
            the HAL_HMAC_MspDeInit can be implemented in the user file.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 ****************************************************************************************
 */
void hal_hmac_msp_deinit(hmac_handle_t *p_hmac);

/** @} */

/** @addtogroup HMAC_Exported_Functions_Group2 IO operation functions
  * @brief HMAC Encrypt/Decrypt functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    This subsection provides a set of functions allowing to manage the HMAC encrypt or decrypt.

    (#) There are two mode of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
           The HAL status of all data processing is returned by the same function
           after finishing transfer.
       (++) Non-Blocking mode: The communication is performed using Interrupts
           or DMA, These API's return the HAL status.
           The end of the data processing will be indicated through the
           dedicated HMAC IRQ when using Interrupt mode or the DMA IRQ when
           using DMA mode.
           The hal_hmac_done_callback() user callbacks will be executed respectively
           at the end of the encrypt or decrypt process
           The hal_hmac_error_callback() user callback will be executed when a error is detected

    (#) Blocking mode API's are :
        (++) hal_hmac_sha256_digest()

    (#) Non-Blocking mode API's with Interrupt are :
        (++) hal_hmac_sha256_digest_it()

    (#) Non-Blocking mode API's with DMA are :
        (++) hal_hmac_sha256_digest_dma()

    (#) A set of encrypt or decrypt Callbacks are provided in Non_Blocking mode:
        (++) hal_hmac_done_callback()
        (++) hal_hmac_error_callback()

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  xxx in blocking mode in HMAC/SHA mode.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 * @param[in]  p_message: Pointer to message buffer
 * @param[in]  number: Amount of data
 * @param[out] p_digest: Pointer to digest buffer
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_hmac_sha256_digest(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t *p_digest, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  xxx in non-blocking mode with interrupt in HMAC/SHA mode.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 * @param[in]  p_message: Pointer to message buffer
 * @param[in]  number: Amount of data
 * @param[out] p_digest: Pointer to digest buffer
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_hmac_sha256_digest_it(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t *p_digest);

/**
 ****************************************************************************************
 * @brief  xxx in non-blocking mode with DMA in HMAC/SHA mode.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 * @param[in]  p_message: Pointer to message buffer
 * @param[in]  number: Amount of data
 * @param[out] p_digest: Pointer to digest buffer
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_hmac_sha256_digest_dma(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t *p_digest);

/** @} */

/** @addtogroup HMAC_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle HMAC interrupt request.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 ****************************************************************************************
 */
void hal_hmac_irq_handler(hmac_handle_t *p_hmac);

/**
 ****************************************************************************************
 * @brief  Digest Done callback.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 ****************************************************************************************
 */
void hal_hmac_done_callback(hmac_handle_t *p_hmac);

/**
 ****************************************************************************************
 * @brief  HMAC error callback.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 ****************************************************************************************
 */
void hal_hmac_error_callback(hmac_handle_t *p_hmac);

/** @} */

/** @defgroup HMAC_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   HMAC control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the HMAC.
     (+) hal_hmac_get_state() API can be helpful to check in run-time the state of the HMAC peripheral.
     (+) hal_hmac_get_error() check in run-time Errors occurring during communication.
     (+) hal_hmac_set_timeout() set the timeout during internal process.
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the HMAC handle state.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 * @retval ::HAL_HMAC_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_HMAC_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_HMAC_STATE_BUSY: Peripheral in indirect mode and busy.
 * @retval ::HAL_HMAC_STATE_ERROR: Peripheral in error.
 * @retval ::HAL_HMAC_STATE_TIMEOUT: Peripheral in timeout.
 * @retval ::HAL_HMAC_STATE_SUSPENDED: Peripheral in suspended.
 ****************************************************************************************
 */
hal_hmac_state_t hal_hmac_get_state(hmac_handle_t *p_hmac);

/**
 ****************************************************************************************
 * @brief  Return the HMAC error code.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 * @return HMAC error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_hmac_get_error(hmac_handle_t *p_hmac);

/**
 ****************************************************************************************
 * @brief  Set the HMAC internal process timeout value.
 * @param[in]  p_hmac: Pointer to a HMAC handle which contains the configuration information for the specified HMAC module.
 * @param[in]  timeout: Internal process timeout value.
 ****************************************************************************************
 */
void hal_hmac_set_timeout(hmac_handle_t *p_hmac, uint32_t timeout);


/**
 ****************************************************************************************
 * @brief  Suspend some registers related to HMAC configuration before sleep.
 * @param[in] p_hmac: Pointer to a HMAC handle which contains the configuration
 *                 information for the specified HMAC module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_hmac_suspend_reg(hmac_handle_t *p_hmac);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to HMAC configuration after sleep.
 *         This function must be used in conjunction with the hal_hmac_suspend_reg().
 * @param[in] p_hmac: Pointer to a HMAC handle which contains the configuration
 *                 information for the specified HMAC module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_hmac_resume_reg(hmac_handle_t *p_hmac);


/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_HMAC_H__ */

/** @} */

/** @} */

/** @} */

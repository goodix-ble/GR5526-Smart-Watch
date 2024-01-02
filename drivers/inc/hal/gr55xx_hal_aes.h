/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_aes.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of AES HAL library.
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

/** @defgroup HAL_AES AES
  * @brief AES HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_AES_H__
#define __GR55xx_HAL_AES_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_aes.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_AES_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_AES_state HAL AES State
  * @{
  */

/**
  * @brief HAL AES State Enumerations definition
  */
typedef enum
{
    HAL_AES_STATE_RESET         = 0x00,    /**< Peripheral not initialized                            */
    HAL_AES_STATE_READY         = 0x01,    /**< Peripheral initialized and ready for use              */
    HAL_AES_STATE_BUSY          = 0x02,    /**< Peripheral in indirect mode and busy                  */
    HAL_AES_STATE_ERROR         = 0x03,    /**< Peripheral in error                                   */
    HAL_AES_STATE_TIMEOUT       = 0x04,    /**< Peripheral in timeout                                 */
    HAL_AES_STATE_SUSPENDED     = 0x05,    /**< Peripheral in suspended                               */
} hal_aes_state_t;

/** @} */

/** @} */

/** @addtogroup HAL_AES_STRUCTURES Structures
  * @{
  */

/** @defgroup AES_Configuration AES Configuration
  * @{
  */

/**
  * @brief AES Init Structure definition
  */
typedef struct _aes_init
{
    uint32_t  key_size;         /**< 128, 192 or 256-bits key length.
                                     This parameter can be a value of @ref AES_Key_Size */

    uint32_t  operation_mode;   /**< AES operating mode.
                                     This parameter can be a value of @ref AES_OPERATION_MODE */

    uint32_t  chaining_mode;    /**< AES chaining mode.
                                     This parameter can be a value of @ref AES_CHAININGMODE */

    uint32_t *p_key;            /**< Encryption/Decryption Key */

    uint32_t *p_init_vector;    /**< Initialization Vector used for CBC modes */

    uint32_t  dpa_mode;         /**< DPA Mode */

    uint32_t *p_seed;           /**< Random seeds */

} aes_init_t;
/** @} */

/** @defgroup AES_handle AES Handle
  * @{
  */

/**
  * @brief AES handle Structure definition
  */
typedef struct _aes_handle
{
    aes_regs_t          *p_instance;            /**< AES registers base address       */

    aes_init_t           init;                  /**< AES operation parameters         */

    uint32_t            *p_cryp_input_buffer;   /**< Pointer to CRYP processing (encryption or decryption) input buffer  */

    uint32_t            *p_cryp_output_buffer;  /**< Pointer to CRYP processing (encryption or decryption) output buffer */

    uint32_t             block_size;            /**< Data size in blocks (16 bytes per block) */

    uint32_t             block_count;           /**< Blocks count                     */

    __IO hal_lock_t      lock;                  /**< Locking object                   */

    __IO hal_aes_state_t state;                 /**< AES operation state              */

    __IO uint32_t        error_code;            /**< AES Error code                   */

    uint32_t             timeout;               /**< Timeout for the AES operation    */

    uint32_t             retention[18];         /**< AES important register information. */

} aes_handle_t;
/** @} */

/** @} */

/** @addtogroup HAL_AES_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup AES_Callback AES Callback
  * @{
  */

/**
  * @brief HAL AES Callback function definition
  */
typedef struct _aes_callback
{
    void (*aes_msp_init)(aes_handle_t *p_aes);                  /**< AES init MSP callback                  */
    void (*aes_msp_deinit)(aes_handle_t *p_aes);                /**< AES de-init MSP callback               */
    void (*aes_error_callback)(aes_handle_t *p_aes);            /**< AES error callback                     */
    void (*aes_done_callback)(aes_handle_t *p_aes);             /**< AES encrypt or decrypt done callback   */
} aes_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_AES_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup AES_Exported_Constants AES Exported Constants
  * @{
  */

/** @defgroup AES_Error_Code AES Error Code
  * @{
  */
#define HAL_AES_ERROR_NONE             ((uint32_t)0x00000000)       /**< No error           */
#define HAL_AES_ERROR_TIMEOUT          ((uint32_t)0x00000001)       /**< Timeout error      */
#define HAL_AES_ERROR_TRANSFER         ((uint32_t)0x00000002)       /**< Transfer error     */
#define HAL_AES_ERROR_INVALID_PARAM    ((uint32_t)0x00000004)       /**< Invalid parameters error */
/** @} */

/** @defgroup AES_Key_Size AES Key Size
  * @{
  */
#define AES_KEYSIZE_128BITS            LL_AES_KEY_SIZE_128          /**< 128 bits */
#define AES_KEYSIZE_192BITS            LL_AES_KEY_SIZE_192          /**< 192 bits */
#define AES_KEYSIZE_256BITS            LL_AES_KEY_SIZE_256          /**< 256 bits */
/** @} */

/** @defgroup AES_Block_Size AES Block Size
  * @{
  */
#define AES_BLOCK_MAX                  (2048)                       /**< Block max size       */
#define AES_BLOCKSIZE_BITS             (128)                        /**< Block size in bits   */
#define AES_BLOCKSIZE_BYTES            (AES_BLOCKSIZE_BITS >> 3)    /**< Block size in bytes  */
#define AES_BLOCKSIZE_WORDS            (AES_BLOCKSIZE_BYTES >> 2)   /**< Block size in words  */
/** @} */

/** @defgroup AES_OPERATION_MODE AES Operation Mode
  * @{
  */
#define AES_OPERATION_MODE_ENCRYPT      (1)                         /**< Encrypt operation mode */
#define AES_OPERATION_MODE_DECRYPT      (0)                         /**< Decrypt operation mode */
/** @} */

/** @defgroup AES_CHAININGMODE AES Chaining Mode
  * @{
  */
#define AES_CHAININGMODE_ECB           LL_AES_OPERATION_MODE_ECB    /**< ECB chaining mode */
#define AES_CHAININGMODE_CBC           LL_AES_OPERATION_MODE_CBC    /**< CBC chaining mode */
/** @} */

/** @defgroup AES_Flags_definition AES Flags Definition
  * @{
  */
#define AES_FLAG_DATAREADY             LL_AES_FLAG_DATAREADY        /**< Data ready flag          */
#define AES_FLAG_DMA_DONE              LL_AES_FLAG_DMA_DONE         /**< DMA transfer done flag   */
#define AES_FLAG_DMA_ERR               LL_AES_FLAG_DMA_ERR          /**< DMA transfer error flag  */
#define AES_FLAG_KEY_VALID             LL_AES_FLAG_KEY_VALID        /**< Key valid flag           */
/** @} */

/** @defgroup AES_Interrupt_definition AES Interrupt definition
  * @{
  */
#define AES_IT_DONE                    ((uint32_t)0x00000001)       /**< AES Encrypted or Decrypted Data Done Interrupt source */
/** @} */

/** @defgroup AES_Timeout_definition AES Timeout definition
  * @{
  */
#define HAL_AES_TIMEOUT_DEFAULT_VALUE  ((uint32_t)5000)             /**< 5s */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup AES_Exported_Macros AES Exported Macros
  * @{
  */

/** @brief  Reset AES handle states.
  * @param  __HANDLE__ AES handle.
  * @retval None
  */
#define __HAL_AES_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_AES_STATE_RESET)

/** @brief  Enable the specified AES peripheral.
  * @param  __HANDLE__ Specifies the AES Handle.
  * @retval None
  */
#define __HAL_AES_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->CTRL, AES_CTRL_MODULE_EN)
/** @brief  Disable the specified AES peripheral.
  * @param  __HANDLE__ Specifies the AES Handle.
  * @retval None
  */
#define __HAL_AES_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->CTRL, AES_CTRL_MODULE_EN)
/** @brief  Enable the AES interrupt.
  * @param  __HANDLE__ Specifies the AES Handle.
  * @retval None
  */
#define __HAL_AES_ENABLE_IT(__HANDLE__)                        ll_aes_enable_it_done((__HANDLE__)->p_instance)

/** @brief  Disable the AES interrupt.
  * @param  __HANDLE__ Specifies the AES Handle.
  * @retval None
  */
#define __HAL_AES_DISABLE_IT(__HANDLE__)                       ll_aes_disable_it_done((__HANDLE__)->p_instance)

/** @brief  Check whether the specified AES interrupt flag is set or not.
  * @param  __HANDLE__ Specifies the AES Handle.
  * @param  __FLAG__ Specifies the interrupt flag to check.
  *         This parameter can be the following value:
  *            @arg @ref AES_IT_DONE Encrypted or Decrypted Data Done Interrupt
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_AES_GET_FLAG_IT(__HANDLE__, __FLAG__)            (READ_BITS((__HANDLE__)->p_instance->INT, (__FLAG__)) == (__FLAG__))
/** @brief  Clear the specified AES interrupt flag.
  * @param  __HANDLE__ Specifies the AES interrupt Handle.
  * @param  __FLAG__ Specifies the flag to clear.
  *         This parameter can be the following value:
  *            @arg @ref AES_IT_DONE Encrypted or Decrypted Data Done Interrupt
  * @retval None
  */
#define __HAL_AES_CLEAR_FLAG_IT(__HANDLE__, __FLAG__)          SET_BITS((__HANDLE__)->p_instance->INT, (__FLAG__))
/** @brief  Check whether the specified AES flag is set or not.
  * @param  __HANDLE__ Specifies the AES Handle.
  * @param  __FLAG__ Specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref AES_FLAG_DATAREADY Data ready flag
  *            @arg @ref AES_FLAG_DMA_DONE DMA transfer done flag
  *            @arg @ref AES_FLAG_DMA_ERR DMA transfer error flag
  *            @arg @ref AES_FLAG_KEY_VALID Key valid flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_AES_GET_FLAG(__HANDLE__, __FLAG__)               ((READ_BITS((__HANDLE__)->p_instance->STAT, (__FLAG__)) != 0) ? SET : RESET)
/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup AES_Private_Macro AES Private Macros
  * @{
  */

/** @brief  Check if AES Key Size is valid.
  * @param  __SIZE__ AES Key Size.
  * @retval SET (__SIZE__ is valid) or RESET (__SIZE__ is invalid)
  */
#define IS_AES_KEY_SIZE(__SIZE__)               (((__SIZE__) == AES_KEYSIZE_128BITS) || \
                                                 ((__SIZE__) == AES_KEYSIZE_192BITS) || \
                                                 ((__SIZE__) == AES_KEYSIZE_256BITS))

/** @brief  Check if AES Operation Mode is valid.
  * @param  __MODE__ AES Operation Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_AES_OPERATION_MODE(__MODE__)         (((__MODE__) == AES_OPERATION_MODE_ENCRYPT) || \
                                                 ((__MODE__) == AES_OPERATION_MODE_DECRYPT))

/** @brief  Check if AES Chaining Mode is valid.
  * @param  __MODE__ AES Chaining Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_AES_CHAININGMODE(__MODE__)          (((__MODE__) == AES_CHAININGMODE_ECB) || \
                                                 ((__MODE__) == AES_CHAININGMODE_CBC))

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_AES_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup AES_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the AESx peripheral:

      (+) User must implement hal_aes_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_aes_init() to configure the selected device with
          the selected configuration:
        (++) Key Size
        (++) operation_mode
        (++) ChainingMode
        (++) key
        (++) init_vector
        (++) DPAMode
        (++) Seed

      (+) Call the function hal_aes_deinit() to restore the default configuration
          of the selected AESx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the AES according to the specified parameters
 *         in the aes_init_t and initialize the associated handle.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration
 *                    information for the specified AES module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_init(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  De-initialize the AES peripheral.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration
 *                    information for the specified AES module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_deinit(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  Initialize the AES MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_aes_msp_deinit can be implemented in the user file.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration
 *                    information for the specified AES module.
 ****************************************************************************************
 */
void hal_aes_msp_init(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  De-initialize the AES MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_aes_msp_deinit can be implemented in the user file.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration
 *                    information for the specified AES module.
 ****************************************************************************************
 */
void hal_aes_msp_deinit(aes_handle_t *p_aes);

/** @} */

/** @addtogroup AES_Exported_Functions_Group2 IO operation functions
  * @brief AES Encrypt/Decrypt functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    This subsection provides a set of functions allowing to manage the AES encrypt or decrypt.

    (#) There are two mode of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
           The HAL status of all data processing are returned by the same function
           after finishing transfer.
       (++) Non-Blocking mode: The communication is performed using Interrupts
           or DMA. These API return the HAL status.
           The end of the data processing will be indicated through the
           dedicated AES IRQ when using Interrupt mode or the DMA IRQ when
           using DMA mode.
           The hal_aes_done_callback() user callbacks will be executed respectively
           at the end of the encrypt or decrypt process
           The hal_aes_error_callback() user callback will be executed when a error is detected

    (#) Blocking mode API's are :
        (++) hal_aes_ecb_encrypt()
        (++) hal_aes_ecb_decrypt()
        (++) hal_aes_cbc_encrypt()
        (++) hal_aes_cbc_decrypt()

    (#) Non-Blocking mode API's with Interrupt are :
        (++) hal_aes_ecb_encrypt_it()
        (++) hal_aes_ecb_decrypt_it()
        (++) hal_aes_cbc_encrypt_it()
        (++) hal_aes_cbc_decrypt_it()

    (#) Non-Blocking mode API's with DMA are :
        (++) hal_aes_ecb_encrypt_dma()
        (++) hal_aes_ecb_decrypt_dma()
        (++) hal_aes_cbc_encrypt_dma()
        (++) hal_aes_cbc_decrypt_dma()

    (#) A set of encrypt or decrypt Callbacks are provided in Non_Blocking mode:
        (++) hal_aes_done_callback()
        (++) hal_aes_error_callback()

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in blocking mode in ECB mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_plain_data: Pointer to plain data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_ecb_encrypt(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in blocking mode in ECB mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_cypher_data: Pointer to cypher data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_ecb_decrypt(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in blocking mode in CBC mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_plain_data: Pointer to plain data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_cbc_encrypt(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in blocking mode in CBC mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_cypher_data: Pointer to cypher data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_cbc_decrypt(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in non-blocking mode with Interrupt in ECB mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_plain_data: Pointer to plain data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_ecb_encrypt_it(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in non-blocking mode with Interrupt in ECB mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_cypher_data: Pointer to cypher data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_ecb_decrypt_it(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data);

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in non-blocking mode with Interrupt in CBC mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_plain_data: Pointer to plain data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_cbc_encrypt_it(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in non-blocking mode with Interrupt in CBC mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_cypher_data: Pointer to cypher data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_cbc_decrypt_it(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data);

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in non-blocking mode with DMA in ECB mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_plain_data: Pointer to plain data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_ecb_encrypt_dma(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in non-blocking mode with DMA in ECB mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_cypher_data: Pointer to cypher data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_ecb_decrypt_dma(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data);

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in non-blocking mode with DMA in CBC mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_plain_data: Pointer to plain data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_cbc_encrypt_dma(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in non-blocking mode with DMA in CBC mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_cypher_data: Pointer to cypher data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_cbc_decrypt_dma(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data);

/** @} */

/** @addtogroup AES_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Handle AES interrupt request.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                     the specified the specified AES module.
 ****************************************************************************************
 */
void hal_aes_irq_handler(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  Encrypt or decrypt Done callback.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 ****************************************************************************************
 */
void hal_aes_done_callback(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  AES error callback.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 ****************************************************************************************
 */
void hal_aes_error_callback(aes_handle_t *p_aes);

/** @} */

/** @defgroup AES_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   AES control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the AES.
     (+) hal_aes_get_state() API can be helpful to check in run-time the state of the AES peripheral.
     (+) hal_aes_get_error() check in run-time Errors occurring during communication.
     (+) hal_aes_set_timeout() set the timeout during internal process.
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the AES handle state.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 *
 * @retval ::HAL_AES_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_AES_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_AES_STATE_BUSY: Peripheral in indirect mode and busy.
 * @retval ::HAL_AES_STATE_ERROR: Peripheral in error.
 * @retval ::HAL_AES_STATE_TIMEOUT: Peripheral in timeout.
 * @retval ::HAL_AES_STATE_SUSPENDED: Peripheral in suspended.
 ****************************************************************************************
 */
hal_aes_state_t hal_aes_get_state(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  Return the AES error code.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 *
 * @return AES error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_aes_get_error(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  Set the AES internal process timeout value.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  timeout: Internal process timeout value.
 ****************************************************************************************
 */
void hal_aes_set_timeout(aes_handle_t *p_aes, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to AES configuration before sleep.
 * @param[in] p_aes: Pointer to a AES handle which contains the configuration
 *                 information for the specified AES module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_suspend_reg(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to AES configuration after sleep.
 *         This function must be used in conjunction with the hal_aes_suspend_reg().
 * @param[in] p_aes: Pointer to a AES handle which contains the configuration
 *                 information for the specified AES module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_resume_reg(aes_handle_t *p_aes);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_AES_H__ */

/** @} */

/** @} */

/** @} */

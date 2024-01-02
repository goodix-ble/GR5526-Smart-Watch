/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_rng.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of RNG HAL library.
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

/** @defgroup HAL_RNG RNG
  * @brief RNG HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_RNG_H__
#define __GR55xx_HAL_RNG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_rng.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_RNG_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_RNG_state HAL RNG state
  * @{
  */

/**
  * @brief HAL RNG State Enumerations definition
  */
typedef enum
{
    HAL_RNG_STATE_RESET     = 0x00,      /**< RNG not initialized or disabled yet */
    HAL_RNG_STATE_READY     = 0x01,      /**< RNG initialized and ready for use   */
    HAL_RNG_STATE_BUSY      = 0x02,      /**< RNG internal process is ongoing     */
    HAL_RNG_STATE_TIMEOUT   = 0x03,      /**< RNG timeout state                   */
    HAL_RNG_STATE_ERROR     = 0x04       /**< RNG error state                     */
} hal_rng_state_t;

/** @} */

/** @} */

/** @addtogroup HAL_RNG_STRUCTURES Structures
  * @{
  */

/** @defgroup  RNG_Configuration RNG Configuration
  * @{
  */

/**
  * @brief  RNG init structure definition
  */
typedef struct _rng_init
{
    uint32_t seed_mode;       /**< Specifies the seed source for the LFSR.
                                 This parameter can be a value of @ref RNG_SEED_SOURCE */

    uint32_t lfsr_mode;       /**< Specifies the configuration mode for the LFSR.
                                 This parameter can be a value of @ref RNG_LFSR_MODE */

    uint32_t out_mode;        /**< Specifies the Output mode for the RNG.
                                 This parameter can be a value of @ref RNG_OUTPUT_MODE */

    uint32_t post_mode;       /**< Specifies post-process configuration for the RNG.
                                 This parameter can be a value of @ref RNG_POST_PRO */

} rng_init_t;

/** @} */

/** @defgroup RNG_handle RNG handle
  * @{
  */

/**
  * @brief  RNG handle Structure definition
  */
typedef struct _rng_handle
{
    rng_regs_t           *p_instance;    /**< Register base address      */

    rng_init_t            init;          /**< RNG required parameters    */

    hal_lock_t            lock;          /**< RNG locking object         */

    __IO hal_rng_state_t  state;         /*!< RNG communication state      */

    uint32_t              random_number; /*!< Last-generated RNG Data      */

    uint32_t              retention[1];     /**< RNG important register information. */
} rng_handle_t;

/** @} */

/** @} */

/** @addtogroup HAL_RNG_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup RNG_Callback RNG Callback
  * @{
  */

/**
  * @brief HAL_RNG Callback function definition
  */

typedef struct _rng_callback
{
    void (*rng_msp_init)(rng_handle_t *p_rng);                                  /**< RNG init MSP callback      */
    void (*rng_msp_deinit)(rng_handle_t *p_rng);                                /**< RNG de-init MSP callback   */
    void (*rng_ready_data_callback)(rng_handle_t *p_rng, uint32_t random32bit); /**< RNG data ready callback    */
} rng_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_RNG_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RNG_Exported_Constants RNG Exported Constants
  * @{
  */

/** @defgroup RNG_SEED_SOURCE LFSR seed source
  * @{
  */
#define RNG_SEED_FR0_S0                 LL_RNG_SEED_FR0_S0            /**< LFSR seed is from the switching oscillator S0. */
#define RNG_SEED_USER                   LL_RNG_SEED_USER              /**< LFSR seed is configured by users. */
/** @} */


/** @defgroup RNG_LFSR_MODE LFSR configuration mode
  * @{
  */
#define RNG_LFSR_MODE_59BIT             LL_RNG_LFSR_MODE_59BIT        /**< 59-bit LFSR.  */
#define RNG_LFSR_MODE_128BIT            LL_RNG_LFSR_MODE_128BIT       /**< 128-bit LFSR. */
/** @} */

/** @defgroup RNG_POST_PRO Post-process mode
  * @{
  */
#define RNG_POST_PRO_NOT                LL_RNG_POST_PRO_NOT           /**< No post process. */
#define RNG_POST_PRO_SKIPPING           LL_RNG_POST_PRO_SKIPPING      /**< bit skipping.    */
#define RNG_POST_PRO_COUNTING           LL_RNG_POST_PRO_COUNTING      /**< bit counting.    */
#define RNG_POST_PRO_NEUMANN            LL_RNG_POST_PRO_NEUMANN       /**< Von-Neumann.     */
/** @} */

/** @defgroup RNG_OUTPUT_MODE RNG Output mode
  * @{
  */
#define RNG_OUTPUT_FR0_S0               LL_RNG_OUTPUT_FR0_S0          /**< Digital RNG direct output, ring oscillator S0. */
#define RNG_OUTPUT_CYCLIC_PARITY        LL_RNG_OUTPUT_CYCLIC_PARITY   /**< LFSR and RNG cyclic sampling and parity generation. */
#define RNG_OUTPUT_CYCLIC               LL_RNG_OUTPUT_CYCLIC          /**< LFSR and RNG cyclic sampling. */
#define RNG_OUTPUT_LFSR_RNG             LL_RNG_OUTPUT_LFSR_RNG        /**< LFSR ⊕ RNG. */
#define RNG_OUTPUT_LFSR                 LL_RNG_OUTPUT_LFSR            /**< LFSR direct output. */
/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_RNG_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup RNG_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initialization functions.
 *
@verbatim
  ==============================================================================
          ##### Initialization and de-initialization functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
      (+) Initialize and start the RNG according to the specified parameters
          in the rng_init_t of associated handle.
      (+) Initialize the RNG MSP.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the RNG according to the specified
 *         parameters in the rng_init_t of  associated handle.
 * @param[in]  p_rng: Pointer to a RNG handle which contains the configuration
 *               information for the specified RNG module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rng_init(rng_handle_t *p_rng);

/**
 ****************************************************************************************
 * @brief  De-initialize the RNG peripheral.
 * @param[in]  p_rng: RNG handle.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rng_deinit(rng_handle_t *p_rng);

/**
 ****************************************************************************************
 * @brief  Initialize the RNG MSP.
 * @param[in]  p_rng: Pointer to a RNG handle which contains the configuration
 *               information for the specified RNG module.
 * @note   When rewriting this function in user file, mechanism may be added
 *         to avoid multiple initialize when hal_rng_init function is called
 *         again to change parameters.
 ****************************************************************************************
 */
void hal_rng_msp_init(rng_handle_t *p_rng);

/**
 ****************************************************************************************
 * @brief  De-initialize the RNG MSP.
 * @param[in]  p_rng: Pointer to a RNG handle which contains the configuration
 *               information for the specified RNG module.
 * @note   When rewriting this function in user file, mechanism may be added
 *         to avoid multiple initialize when hal_rng_init function is called
 *         again to change parameters.
 ****************************************************************************************
 */
void hal_rng_msp_deinit(rng_handle_t *p_rng);

/** @} */


/** @addtogroup RNG_Exported_Functions_Group2 Peripheral Control functions
 *  @brief    Peripheral Control functions
 *
@verbatim
  ==============================================================================
                      ##### Peripheral Control functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Generate Random Number.
    (+) Handle RNG interrupt request and associated function callback.

@endverbatim
  * @{
  */


/**
 ****************************************************************************************
 * @brief  Generate a 32-bit random number.
 * @param[in]  p_rng: Pointer to a RNG handle which contains the configuration
 *               information for the specified RNG module.
 * @param[in]  p_seed: user configured seeds. the seed is valid when seed_mode member of
 *               rng_init_t is configured as RNG_SEED_USER. If 59-bit random number is
 *               selected, the seed need to provide [0~58] bit spaces. If 128-bit random
 *               number is selected, the seed need to provide [0~127] bit spaces.
 * @param[out] p_random32bit: Pointer to generated random number variable if successful.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rng_generate_random_number(rng_handle_t *p_rng, uint16_t *p_seed, uint32_t *p_random32bit);

/**
 ****************************************************************************************
 * @brief  Generate a 32-bit random number in interrupt mode.
 * @param[in]  p_rng: Pointer to a RNG handle which contains the configuration
 *               information for the specified RNG module.
 * @param[in]  p_seed: user configured seeds. the seed is valid when seed_mode member of
 *               rng_init_t is configured as RNG_SEED_USER. If 59-bit random number is
 *               selected, the seed need to provide [0~58] bit spaces. If 128-bit random
 *               number is selected, the seed need to provide [0~127] bit spaces.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rng_generate_random_number_it(rng_handle_t *p_rng, uint16_t *p_seed);

/**
 ****************************************************************************************
 * @brief  Read the latest generated random number.
 * @param[in]  p_rng: Pointer to a RNG handle which contains the configuration
 *               information for the specified RNG module.
 * @retval random value.
 ****************************************************************************************
 */
uint32_t hal_rng_read_last_random_number(rng_handle_t *p_rng);

/** @} */

/** @addtogroup RNG_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callback functions
 * @{
 */
/**
 ****************************************************************************************
 * @brief Handle RNG interrupt request.
 * @param[in] p_rng: RNG handle.
 ****************************************************************************************
 */
void hal_rng_irq_handler(rng_handle_t *p_rng);

/**
 ****************************************************************************************
 * @brief  Data Ready callback in non-blocking mode.
 * @note   This function should not be modified. When the callback is needed,
            the hal_rng_ready_data_callback can be implemented in the user file.
 * @param[in]  p_rng: Pointer to a RNG handle which contains the configuration
 *                 information for the specified RNG module.
 * @param  random32bit: generated random value
 * @retval None
 ****************************************************************************************
 */
__weak void hal_rng_ready_data_callback(rng_handle_t *p_rng, uint32_t random32bit);

/** @} */

/** @defgroup RNG_Exported_Functions_Group3 Peripheral State functions
  * @brief   RNG State functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the RNG.
     (+) hal_rng_get_state() API can be helpful to check in run-time the state of the RNG peripheral.
@endverbatim
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Return the RNG handle state.
 * @param[in]  p_rng: Pointer to a RNG handle which contains the configuration information for the specified HMAC module.
 * @retval ::HAL_RNG_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_RNG_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_RNG_STATE_BUSY: Peripheral in indirect mode and busy.
 * @retval ::HAL_RNG_STATE_ERROR: Peripheral in error.
 * @retval ::HAL_RNG_STATE_TIMEOUT: Peripheral in timeout.
 ****************************************************************************************
 */
hal_rng_state_t hal_rng_get_state(rng_handle_t *p_rng);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to RNG configuration before sleep.
 * @param[in] p_rng: Pointer to a RNG handle which contains the configuration
 *                 information for the specified RNG module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rng_suspend_reg(rng_handle_t *p_rng);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to RNG configuration after sleep.
 *         This function must be used in conjunction with the hal_rng_resume_reg().
 * @param[in] p_rng: Pointer to a RNG handle which contains the configuration
 *                 information for the specified RNG module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rng_resume_reg(rng_handle_t *p_rng);


/** @} */

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_RNG_H__ */

/** @} */

/** @} */

/** @} */

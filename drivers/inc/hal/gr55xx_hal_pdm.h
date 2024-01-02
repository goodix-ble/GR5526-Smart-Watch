/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_pdm.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PDM HAL library.
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

/** @defgroup HAL_PDM PDM
  * @brief PDM HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion ----------------------------------------------------------*/
#ifndef __GR55xx_HAL_PDM_H__
#define __GR55xx_HAL_PDM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ---------------------------------------------------------------------------------------*/
#include "gr55xx_ll_pdm.h"
#include "gr55xx_hal_def.h"
#include "gr55xx_hal_dma.h"

/* Exported types  --------------------------------------------------------------------------------*/
/** @addtogroup HAL_PDM_ENUMERATIONS Enumerations
  * @{
  */
/**
  * @brief HAL PDM State Enumerations definition
  */
typedef enum
{
    HAL_PDM_STATE_RESET        = 0x00, /**< Peripheral not initialized           */
    HAL_PDM_STATE_READY        = 0x01, /**< Peripheral initialized and ready for use */
    HAL_PDM_STATE_BUSY         = 0x02, /**< An internal process is ongoing       */
    HAL_PDM_STATE_ERROR        = 0x04  /**< Peripheral in error                  */
} hal_pdm_state_t;
/** @} */

/** @addtogroup HAL_PDM_STRUCTURES Structures
  * @{
  */
/**
  * @brief   PDM init structure definition
  */
typedef ll_pdm_init_t       pdm_init_t;

/**
  * @brief PDM handle Structure definition
  */
typedef struct _pdm_handle
{
    pdm_regs_t             *p_instance;     /**< PDM registers base address        */

    pdm_init_t              init;           /**< PDM configuration parameters      */

    dma_handle_t           *p_dma;          /**< PDM DMA Handle parameters         */

    __IO hal_lock_t         lock;           /**< Locking object                    */

    __IO hal_pdm_state_t    state;          /**< PDM communication state           */

    __IO uint32_t           error_code;     /**< PDM error code                    */

    uint32_t                retention[5];   /**< PDM important register information.  */
} pdm_handle_t;
/** @} */


/** @addtogroup HAL_PDM_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/**
  * @brief HAL PDM Callback function definition
  */
typedef struct _pdm_callback
{
    void (*pdm_msp_init)(pdm_handle_t *p_pdm);               /**< PDM init MSP callback      */
    void (*pdm_msp_deinit)(pdm_handle_t *p_pdm);             /**< PDM de-init MSP callback   */
    void (*pdm_left_overflow_callback)(pdm_handle_t *p_pdm); /**< PDM left channel overflow callback  */
    void (*pdm_right_overflow_callback)(pdm_handle_t *p_pdm);/**< PDM right channel overflow callback */
    void (*pdm_dma_cplt_callback)(pdm_handle_t *p_pdm);      /**< PDM dma completed callback  */
    void (*pdm_dma_blk_callback)(pdm_handle_t *p_pdm);       /**< PDM dma blocks completed callback   */
    void (*pdm_dma_error_callback)(pdm_handle_t *p_pdm);     /**< PDM error callback          */
} pdm_callback_t;
/** @} */

/** @defgroup  HAL_PDM_MACRO Defines
  * @{
  */
/* Exported constants -----------------------------------------------------------------------------*/
/** @defgroup PDM_Exported_Constants PDM Exported Constants
  * @{
  */
/** @defgroup PDM_Error_Code PDM Error Code
  * @{
  */
 /**
 * @brief PDM error code.
 */ 
#define HAL_PDM_ERROR_NONE              ((uint32_t)0x00000000)  /**< No error. */
#define HAL_PDM_ERROR_TIMEOUT           ((uint32_t)0x00000001)  /**< Timeout error. */
#define HAL_PDM_ERROR_TRANSFER          ((uint32_t)0x00000002)  /**< Transfer error. */
#define HAL_PDM_ERROR_DMA               ((uint32_t)0x00000004)  /**< DMA transfer error. */
#define HAL_PDM_ERROR_INVALID_PARAM     ((uint32_t)0x00000008)  /**< Invalid parameter error. */
/** @} */

/** @defgroup PDM_Sample_Rate PDM Sample Rate
  * @{
  */
/**
 * @brief PDM sample rate.
 */
#define PDM_SAMPLE_RATE_15_625K         LL_PDM_SAMPLE_RATE_15_625K /**< PDM sample rate 15.625K. */
#define PDM_SAMPLE_RATE_16K             LL_PDM_SAMPLE_RATE_16K     /**< PDM sample rate 16K. */
#define PDM_SAMPLE_RATE_8K              LL_PDM_SAMPLE_RATE_8K      /**< PDM sample rate 8K. */
/** @} */

/** @defgroup PDM_Operation_Mode PDM Operation Mode
  * @{
  */
/**
 * @brief PDM operation mode.
 */
#define PDM_MODE_LEFT                   LL_PDM_MODE_LEFT    /**< PDM left mono. */
#define PDM_MODE_RIGHT                  LL_PDM_MODE_RIGHT   /**< PDM right mono. */
#define PDM_MODE_STEREO                 LL_PDM_MODE_STEREO  /**< PDM stereo mono. */
/** @} */

// /**
//   * @brief PDM_default_config initStruct default configuartion
//   */
//#define PDM_DEFAULT_CONFIG          LL_PDM_DEFAULT_CONFIG

/** @} */

/* Exported macro ---------------------------------------------------------------------------------*/
/** @defgroup PDM_Exported_Macros PDM Exported Macros
  * @{
  */

/** @brief  Reset PDM handle states.
  * @param  __HANDLE__ PDM handle.
  * @retval None
  */
#define __HAL_PDM_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_PDM_STATE_RESET)

/** @brief  Enable the specified PDM clock.
  * @param  __HANDLE__ Specify the PDM Handle.
  * @retval None
  */
#define __HAL_PDM_ENABLE_CLOCK(__HANDLE__)                     ll_pdm_enable_clk((__HANDLE__)->p_instance)

/** @brief  Disable the specified ADC clock.
  * @param  __HANDLE__ Specify the ADC Handle.
  * @retval None
  */
#define __HAL_PDM_DISABLE_CLOCK(__HANDLE__)                    ll_pdm_disable_clk((__HANDLE__)->p_instance)
/** @} */

/** @} */

/* Exported functions -----------------------------------------------------------------------------*/
/** @addtogroup HAL_PDM_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup PDM_Exported_Functions_Group1 Initialization and de-initialization Functions
  *  @brief    Initialization and Configuration functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the PDM according to the specified parameters
 *         in the pdm_init_t and initialize the associated handle.
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                    the specified PDM module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_init(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  De-initialize the PDM peripheral.
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                    the specified PDM module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_deinit(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  Initialize the PDM MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_pdm_msp_init can be implemented in the user file.
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                the specified PDM module.
 ****************************************************************************************
 */
void hal_pdm_msp_init(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  De-initialize the PDM MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_pdm_msp_deinit can be implemented in the user file.
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                    the specified PDM module.
 ****************************************************************************************
 */
void hal_pdm_msp_deinit(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  Start the pdm left channel transfer with dma.
 *
 * @param[in]  p_pdm: Pointer to an PDM handle.
 * @param[in]  p_data: Point to the buffer that stores the data collected by PDM.
 * @param[in]  length: Length of data buffer, ranging between 0 and 4095.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_left_start_dma(pdm_handle_t *p_pdm, uint16_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Start the pdm right channel transfer with dma's sg and llp.
 *
 * @param[in]  p_pdm:         Pointer to an PDM handle.
 * @param[in]  p_data:        Point to the buffer that stores the data collected by PDM.
 * @param[in]  length:        Length of data buffer, ranging between 0 and 4095.
 * @param[in]  sg_llp_config: Config for dma llp and sg fuction.
 *
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_left_start_dma_sg_llp(pdm_handle_t *p_pdm, uint16_t *p_data, uint32_t length, dma_sg_llp_config_t *sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Start the pdm right channel transfer with dma.
 *
 * @param[in]  p_pdm: Pointer to an PDM handle.
 * @param[in]  p_data: Point to the buffer that stores the data collected by PDM.
 * @param[in]  length: Length of data buffer, ranging between 0 and 4095.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_right_start_dma(pdm_handle_t *p_pdm, uint16_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Start the pdm right channel transfer with dma's sg and llp.
 *
 * @param[in]  p_pdm:  Pointer to an PDM handle.
 * @param[in]  p_data: Point to the buffer that stores the data collected by PDM.
 * @param[in]  length: Length of data buffer, ranging between 0 and 4095.
 * @param[in]  sg_llp_config: config for dma llp and sg fuction.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_right_start_dma_sg_llp(pdm_handle_t *p_pdm, uint16_t *p_data, uint32_t length, dma_sg_llp_config_t *sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Start the pdm dual channel transfer with dma.
 *
 * @param[in]  p_pdm:  Pointer to an PDM handle.
 * @param[in]  p_data: Point to the buffer that stores the data collected by PDM.
 * @param[in]  length: Length of data buffer, ranging between 0 and 4095.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_stereo_start_dma(pdm_handle_t *p_pdm, uint32_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Start the pdm dual channel transfer with dma's sg and llp.
 *
 * @param[in]  p_pdm:  Pointer to an PDM handle.
 * @param[in]  p_data: Point to the buffer that stores the data collected by PDM.
 * @param[in]  length: Length of data buffer, ranging between 0 and 4095.
 * @param[in]  sg_llp_config: config for dma llp and sg fuction.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_stereo_start_dma_sg_llp(pdm_handle_t *p_pdm, uint32_t *p_data, uint32_t length, dma_sg_llp_config_t *sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Abort ongoing conversion (blocking mode).
 *
 * @note   This procedure could be only used for aborting transfer started in DMA mode.
 *         This procedure performs following operations:
 *           - Disable PDM clock, stop conversion
 *           - Abort DMA transfer by calling hal_dma_abort_it (in case of transfer in DMA mode)
 *           - Set handle State to READY.
 *         This procedure is executed in blocking mode: when exiting function, Abort is transfered as completed.
 *
 * @param[in]  p_pdm: PDM handle.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_abort(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  Handle PDM interrupt request.
 *
 * @param[in]  p_pdm: Pointer to a PDM handle which contains the configuration information for the specified PDM.
 ****************************************************************************************
 */
void hal_pdm_irq_handler(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  Left channel data overflow callback.
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                    the specified PDM module.
 ****************************************************************************************
 */
void hal_pdm_left_overflow_callback(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  Right channel data overflow callback.
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                    the specified PDM module.
 ****************************************************************************************
 */
void hal_pdm_right_overflow_callback(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  Dma transfer completed callback..
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                    the specified PDM module.
 ****************************************************************************************
*/
void hal_pdm_dma_cplt_callback(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  Block transfer completed callback..
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                    the specified PDM module.
 ****************************************************************************************
 */
void hal_pdm_dma_blk_callback(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  DMA transfer error callback..
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                    the specified PDM module.
 ****************************************************************************************
 */
void hal_pdm_dma_error_callback(pdm_handle_t *p_pdm);
/** @} */

/** @defgroup PDM_Exported_Functions_Group3 Peripheral State and Errors Functions
  * @brief   PDM control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the PDM.
     (+) hal_pdm_get_state() API can be helpful to check in run-time the state of the PDM peripheral.
     (+) hal_pdm_get_error() check in run-time Errors occurring during communication.
@endverbatim
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Return the PDM handle state.
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                    the specified PDM module.
 *
 * @retval ::HAL_ADC_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_ADC_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_ADC_STATE_BUSY: An internal process is ongoing.
 * @retval ::HAL_ADC_STATE_ERROR: Peripheral in error.
 ****************************************************************************************
 */
hal_pdm_state_t hal_pdm_get_state(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  Return the PDM error code.
 *
 * @param[in]  p_pdm: Pointer to an PDM handle which contains the configuration information for
 *                    the specified PDM module.
 *
 * @return PDM error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_pdm_get_error(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to PDM configuration before sleep.
 * @param[in] p_pdm: Pointer to a PDM handle which contains the configuration
 *                 information for the specified PDM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_suspend_reg(pdm_handle_t *p_pdm);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to PDM configuration after sleep.
 *         This function must be used in conjunction with the hal_adc_suspend_reg().
 * @param[in] p_pdm: Pointer to a PDM handle which contains the configuration
 *                 information for the specified PDM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pdm_resume_reg(pdm_handle_t *p_pdm);

/** @} */

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_PDM_H__ */

/** @} */

/** @} */

/** @} */

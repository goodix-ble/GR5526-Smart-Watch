/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_rtc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of RTC HAL library.
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

/** @defgroup HAL_RTC RTC
  * @brief RTC HAL module driver.
  * @{
  */

#ifndef __GR55XX_HAL_RTC_H__
#define __GR55XX_HAL_RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_rtc.h"
#include "gr55xx_hal_def.h"

/**
  * @defgroup  HAL_RTC_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RTC_Exported_Constants RTC Exported Constants
  * @{
  */

/** @defgroup RTC_INTERRUPT_STATUS RTC Interrupt Status
  * @{
  */
#define RTC_FLAG_ALARM                 RTC_INT_STAT_ALARM                      /**< Alarm interrupt flag   */
#define RTC_FLAG_WRAP                  RTC_INT_STAT_WRAP                       /**< Wrap interrupt flag    */
#define RTC_FLAG_TICK                  RTC_INT_STAT_TICK                       /**< Tick   interrupt flag  */
/** @} */

/** @defgroup RTC_INTERRUPT_ENABLE RTC Interrupt Enable
  * @{
  */
#define RTC_IT_ALARM                   RTC_INT_EN_ALARM                        /**< Alarm interrupt    */
#define RTC_IT_WRAP                    RTC_INT_EN_WRAP                         /**< Wrap interrupt     */
#define RTC_IT_TICK                    RTC_INT_EN_TICK                         /**< Tick   interrupt   */
/** @} */

/** @defgroup RTC_Error_Code_definition RTC Error Code definition
  * @{
  */
#define HAL_RTC_ERROR_NONE                   (0x00000000U)    /**< No error                        */
#define HAL_RTC_ERROR_INVALID_PARAM          (0x00000001U)    /**< Invalid parameter error         */
#define HAL_RTC_ERROR_TIMEOUT                (0x00000002U)    /**< Timeout error                   */
/** @} */

/** @defgroup RTC_CLOCK_DIV Clock divider
  * @{
  */
#define RTC_DIV_NONE                    LL_RTC_DIV_NONE           /**< Select SLP_CLK       */
#define RTC_DIV_2                       LL_RTC_DIV_2              /**< Select 1/32 divider  */
#define RTC_DIV_4                       LL_RTC_DIV_4              /**< Select 1/32 divider  */
#define RTC_DIV_8                       LL_RTC_DIV_8              /**< Select 1/32 divider  */
#define RTC_DIV_16                      LL_RTC_DIV_16             /**< Select 1/32 divider  */
#define RTC_DIV_32                      LL_RTC_DIV_32             /**< Select 1/64 divider  */
#define RTC_DIV_64                      LL_RTC_DIV_64             /**< Select 1/128 divider */
#define RTC_DIV_128                     LL_RTC_DIV_128            /**< Select 1/256 divider */
/** @} */

/** @defgroup RTC_RELOAD_MODE Relaod mode
  * @{
  */
#define ONE_TIME                        LL_RTC_TIMER_TICK_TYPE_SINGLE    /**< Select periodic alarm one-time     */
#define AUTO_RELOAD                     LL_RTC_TIMER_TICK_TYPE_AUTO      /**< Select periodic alarm auto-reload  */
/** @} */

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup RTC_Private_Macro RTC Private Macros
  * @{
  */

/** @brief  Disable RTC
  * @param  RTCx Specifies the RTC instance.
  * @retval None
  */
#define __HAL_RTC_DISABLE(RTCx)                            MODIFY_REG(RTCx->CFG0, 0xFFFFFFFF, RTC_CFG0_CFG);

/** @brief  Enable RTC
  * @param  RTCx Specifies the RTC instance.
  * @retval None
  */
#define __HAL_RTC_ENABLE(RTCx)                             SET_BITS(RTCx->CFG0, RTC_CFG0_CFG | RTC_CFG0_EN);

/** @brief  Check RTC busy state
  * @param  RTCx Specifies the RTC instance.
  * @retval TRUE or FALSE
  */
#define __HAL_RTC_BUSY_FLAG(RTCx)                          ((READ_BITS(RTCx->STAT, RTC_STAT_BUSY) == RTC_STAT_BUSY))

/** @brief  Clear RTC flag
  * @param  RTCx Specifies the RTC instance.
  * @param  __FLAG__ Specifies the RTC Interrupt Status.
  * @retval None
  */
#define __HAL_RTC_CLEAR_FLAG(RTCx, __FLAG__)               WRITE_REG(RTCx->INT_STAT, (__FLAG__))

/** @brief  Enable RTC interrupt
  * @param  RTCx Specifies the RTC instance.
  * @param  __INTERRUPT__ Specifies the RTC Interrupt Type.
  * @retval None
  */
#define __HAL_RTC_ENABLE_IT(RTCx, __INTERRUPT__)           SET_BITS(RTCx->INT_EN, (__INTERRUPT__))

/** @brief  Disable RTC interrupt
  * @param  RTCx Specifies the RTC instance.
  * @param  __INTERRUPT__ Specifies the RTC Interrupt Type.
  * @retval None
  */
#define __HAL_RTC_DISABLE_IT(RTCx, __INTERRUPT__)          CLEAR_BITS(RTCx->INT_EN, (__INTERRUPT__))

/** @brief  Get RTC interrupt source
  * @param  RTCx Specifies the RTC instance.
  * @param  __FLAG__ Specifies the RTC Interrupt Status.
  * @retval TRUE or FALSE
  */
#define __HAL_RTC_GET_IT_SOURCE(RTCx, __FLAG__)            (READ_BITS(RTCx->INT_STAT, (__FLAG__)) == (__FLAG__))

/** @brief  Set RTC config takes effect
  * @param  RTCx Specifies the RTC instance.
  * @retval None
  */
#define __HAL_RTC_CFG_EFFECT(RTCx)                         SET_BITS(RTCx->CFG0, RTC_CFG0_CFG)
/** @} */

/** @} */

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_RTC_ENUMERATIONS Enumerations
  * @{
  */

/**
  * @brief hal_rtc_state definition
  */
typedef enum
{
    HAL_RTC_READY    = 0x00U,
    HAL_RTC_RESET,
    HAL_RTC_RUNNING,
    HAL_RTC_ERROR
}hal_rtc_state_t;

/**
  * @brief rtc_tick_state definition
  */
typedef enum
{
    TICK_READY       = 0x00U,
    TICK_RUNNING,
    TICK_ERROR
}rtc_tick_state_t;

/**
  * @brief rtc_alarm_state definition
  */
typedef enum
{
    ALARM_READY       = 0x00U,
    ALARM_RUNNING,
    ALARM_ERROR
}rtc_alarm_state_t;

/**
  * @brief overflow config
  */
typedef enum
{
    CLOSED      = 0x00U,
    OPENED      = !CLOSED
}overflow_det_t;

/** @} */

/** @addtogroup HAL_RTC_STRUCTURES Structures
  * @{
  */

/**
  * @brief rtc init params
  * @attention count up from zero if not set start_value.
  */
typedef struct _rtc_init_t
{

    uint32_t                  start_value;

    uint32_t                  prescaler_div;

    overflow_det_t            overflow_det_state;

}rtc_init_t;

/**
  * @brief rtc tick handle definition
  */
typedef struct _tick_handle_t
{
    uint8_t                   mode;

    uint32_t                  value;

    __IO rtc_tick_state_t     state;

}tick_handle_t;

/**
  * @brief rtc alarm handle definition
  */
typedef struct _alarm_handle_t
{
    uint32_t                  value;

    __IO rtc_alarm_state_t    state;

}alarm_handle_t;


/**
  * @brief rtc handle definition
  */
typedef struct _rtc_handle_t
{
    rtc_regs_t                *p_instance;  /*!< Register base address    */

    rtc_init_t                init;         /*!< RTC required parameters  */

    tick_handle_t             tick;         /*!< tick_handle              */

    alarm_handle_t            alarm;        /*!< alarm_handle             */

    hal_lock_t                lock;         /*!< RTC locking object       */

    __IO hal_rtc_state_t      state;        /*!< Time communication state */

    __IO uint32_t             error_code;   /**< RTC Error code           */

}rtc_handle_t;

/** @} */

/** @addtogroup HAL_RTC_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup RTC_Callback RTC Callback
  * @{
  */

/**
  * @brief HAL_RTC Callback function definition
  */

typedef struct _rtc_callback
{
    void (*rtc_alarm_callback)(rtc_handle_t *p_rtc);        /**< RTC alarm count complete callback */
    void (*rtc_tick_callback)(rtc_handle_t *p_rtc);         /**< RTC tick count complete callback */
    void (*rtc_overflow_callback)(rtc_handle_t *p_rtc);     /**< RTC overflow callback */
} rtc_callback_t;

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_RTC_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup RTC_Exported_Functions_Group1 Initialization and de-initialization Functions
  *  @brief    Initialization and Configuration functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the RTC according to the specified parameters in the
 *         rtc_init_t of  associated handle.counter start after hal_rtc_init.
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rtc_init(rtc_handle_t *p_rtc);

/**
 ****************************************************************************************
 * @brief  DeInitialize the RTC according to the specified parameters in the
 *         rtc_init_t of  associated handle.counter stop after hal_rtc_deinit.
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rtc_deinit(rtc_handle_t *p_rtc);

/**
 ****************************************************************************************
 * @brief  Start counting down for tick module
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 * @param[in]  mode
 * @param[in]  value
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rtc_set_tick_and_start(rtc_handle_t *p_rtc, uint8_t mode,uint32_t value);

/**
 ****************************************************************************************
 * @brief  Stop counting down for tick module
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rtc_stop_tick(rtc_handle_t *p_rtc);

/**
 ****************************************************************************************
 * @brief  Restart tick module and continue run with the previous settings.
 *         This function need be called after hal_rtc_stop_tick
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rtc_restart_tick(rtc_handle_t *p_rtc);
/**
 ****************************************************************************************
 * @brief  Compare counter with alarm_value for alarm module
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 * @param[in]  value
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rtc_set_alarm(rtc_handle_t *p_rtc, uint32_t value);

/**
 ****************************************************************************************
 * @brief  Stop counting up and compare with alarm_value for alarm module
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rtc_stop_alarm(rtc_handle_t *p_rtc);

/**
 ****************************************************************************************
 * @brief  Clear wrap count
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_rtc_clear_wrap(rtc_handle_t* p_rtc);

/**
 ****************************************************************************************
 * @brief  Get the times of overflow
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @retval ::
 ****************************************************************************************
 */
uint32_t hal_rtc_get_wrap_count(rtc_handle_t *p_rtc);

/**
 ****************************************************************************************
 * @brief  Get_cur_count_value
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @retval ::
 ****************************************************************************************
 */
uint32_t hal_rtc_get_cur_count(rtc_handle_t* p_rtc);

/**
 ****************************************************************************************
 * @brief  Get_cur_tick_value
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @retval ::
 ****************************************************************************************
 */
uint32_t hal_rtc_get_cur_tick(rtc_handle_t* p_rtc);

/**
 ****************************************************************************************
 * @brief  Get_alarm_value
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @retval ::
 ****************************************************************************************
 */
uint32_t hal_rtc_get_alarm_value(rtc_handle_t *p_rtc);

/**
 ****************************************************************************************
 * @brief  Get_cur_rtc_state
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_rtc_state_t hal_rtc_get_state(rtc_handle_t *p_rtc);

/** @} */

/** @addtogroup RTC_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  RTC_IRQHandler
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 ****************************************************************************************
 */
void hal_rtc_irq_handler(rtc_handle_t *p_rtc);

/**
 ****************************************************************************************
 * @brief  alarm_callback
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 ****************************************************************************************
 */
void hal_rtc_alarm_callback(rtc_handle_t *p_rtc);

/**
 ****************************************************************************************
 * @brief  overflow_callback
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 ****************************************************************************************
 */
void hal_rtc_overflow_callback(rtc_handle_t *p_rtc);

/**
 ****************************************************************************************
 * @brief  tick_callback
 *
 * @param[in]  p_rtc: Pointer to a RTC handle which contains the configuration
 *               information for the specified RTC module.
 ****************************************************************************************
 */
void hal_rtc_tick_callback(rtc_handle_t *p_rtc);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */

/** @} */

/** @} */

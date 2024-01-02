/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_iso7816.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of ISO7816 HAL library.
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

/** @defgroup HAL_ISO7816 ISO7816
  * @brief ISO7816 HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_ISO7816_H__
#define __GR55xx_HAL_ISO7816_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_iso7816.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_ISO7816_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_ISO7816_state HAL ISO7816 State
  * @{
  */
/**
  * @brief HAL ISO7816 State Enumerations definition
  */
typedef enum
{
    HAL_ISO7816_STATE_RESET             = 0x00U,    /**< Peripheral not initialized                          */
    HAL_ISO7816_STATE_READY             = 0x20U,    /**< Peripheral initialized and ready for use            */
    HAL_ISO7816_STATE_BUSY              = 0x24U,    /**< An internal process is ongoing                      */
    HAL_ISO7816_STATE_BUSY_TX           = 0x21U,    /**< Data Transmission process is ongoing                */
    HAL_ISO7816_STATE_BUSY_RX           = 0x22U,    /**< Data Reception process is ongoing                   */
    HAL_ISO7816_STATE_BUSY_TX_RX        = 0x23U,    /**< Data Transmission and Reception process is ongoing  */
    HAL_ISO7816_STATE_ABORT             = 0x08U,    /**< Peripheral with abort request ongoing               */
    HAL_ISO7816_STATE_TIMEOUT           = 0xA0U,    /**< Timeout state                                       */
    HAL_ISO7816_STATE_ERROR             = 0xE0U     /**< Peripheral in error                                 */
} hal_iso7816_state_t;

/** @} */

/** @} */

/** @addtogroup HAL_ISO7816_STRUCTURES Structures
  * @{
  */

/** @defgroup ISO7816_Configuration ISO7816 Configuration
  * @{
  */
/**
  * @brief   ISO7816 init structure definition
  */
typedef struct
{
    uint32_t clk_div;                 /*!<  clk_div is used for dividing the system clock,
                                            and IOS7816 output clock is equal to (system clock)/(clk_div+1).*/
    uint32_t wait_time;               /*!< Specifies the guard time value in terms of number of baud clocks */
    uint16_t guard_time;              /*!< Specifies the maximum card response time (leading edge to leading edge) */
    uint8_t  detect_coding;           /*!< Specifies whether automatically detect coding convention during ATR receiption. */
} iso7816_init_t;

/** @} */


/** @defgroup ISO7816_handle  Handle
  * @{
  */

/**
  * @brief ISO7816 handle Structure definition
  */
typedef struct _iso7816_handle_t
{
    iso7816_regs_t              *p_instance;        /**< ISO7816 registers base address           */
    iso7816_init_t              init;               /**< ISO7816 configuration parameters         */
    uint8_t                     *p_tx_rx_buffer;    /**< Pointer to ISO7816 Tx&Rx transfer Buffer */
    uint16_t                    buffer_size;        /**< ISO7816 Buffer size                      */
    uint16_t                    tx_xfer_size;       /**< ISO7816 Tx Transfer size                 */
    __IO uint16_t               tx_xfer_count;      /**< ISO7816 Tx Transfer Counter              */
    uint16_t                    rx_xfer_size;       /**< ISO7816 Rx Transfer size                 */
    __IO uint16_t               rx_xfer_count;      /**< ISO7816 Rx Transfer Counter              */
    __IO hal_lock_t             lock;               /**< Locking object                           */
    __IO hal_iso7816_state_t    state;              /**< ISO7816 communication state              */
    __IO hal_iso7816_state_t    tx_state;           /**< ISO7816 state information related to Tx operations.  */
    __IO hal_iso7816_state_t    rx_state;           /**< ISO7816 state information related to Rx operations.  */
    __IO uint32_t               previous_action;    /**< ISO7816 previous_action                  */
    __IO uint32_t               error_code;         /**< ISO7816 error code                       */
    uint32_t                    retention[6];       /**< ISO7816 important register information.  */
} iso7816_handle_t;
/** @} */

/** @} */

/** @addtogroup HAL_ISO7816_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup ISO7816_Callback ISO7816 Callback
  * @{
  */

 /**
  * @brief HAL_ISO7816 Callback function definition
  */

typedef struct _iso7816_callback
{
    void (*iso7816_msp_init)(iso7816_handle_t *p_iso7816);                  /**< ISO7816 init MSP callback                      */
    void (*iso7816_msp_deinit)(iso7816_handle_t *p_iso7816);                /**< ISO7816 de-init MSP callback                   */
    void (*iso7816_error_callback)(iso7816_handle_t *p_iso7816);            /**< ISO7816 error callback                         */
    void (*iso7816_abort_cplt_callback)(iso7816_handle_t *p_iso7816);       /**< ISO7816 abort completed callback               */
    void (*iso7816_presence_callback)(iso7816_handle_t *p_iso7816);         /**< ISO7816 card presence state changed callback   */
    void (*iso7816_atr_cplt_callback)(iso7816_handle_t *p_iso7816);          /**< ISO7816 reseive atr completed callback        */
    void (*iso7816_tx_cplt_callback)(iso7816_handle_t *p_iso7816);          /**< ISO7816 rx transfer completed callback         */
    void (*iso7816_rx_cplt_callback)(iso7816_handle_t *p_iso7816);          /**< ISO7816 tx transfer completed callback         */
    void (*iso7816_tx_rx_cplt_callback)(iso7816_handle_t *p_iso7816);       /**< ISO7816 tx/rx transfer completed callback      */
} iso7816_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_ISO7816_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup ISO7816_Exported_Constants ISO7816 Exported Constants
  * @{
  */

/** @defgroup ISO7816_ACTION Action state
  * @{
  */
#define HAL_ISO7816_ACTION_NONE             LL_ISO7816_ACTION_NONE           /**< Do Nothing.                        */
#define HAL_ISO7816_ACTION_OFF              LL_ISO7816_ACTION_OFF            /**< Switch Off.                        */
#define HAL_ISO7816_ACTION_STOPCLK          LL_ISO7816_ACTION_STOPCLK        /**< Stop the clock.                    */
#define HAL_ISO7816_ACTION_ON               LL_ISO7816_ACTION_ON             /**< Switch on and receive ATR.         */
#define HAL_ISO7816_ACTION_WARMRST          LL_ISO7816_ACTION_WARMRST        /**< Trigger warm reset and receive ATR.*/
#define HAL_ISO7816_ACTION_RX               LL_ISO7816_ACTION_RX             /**< Receive.                           */
#define HAL_ISO7816_ACTION_TX               LL_ISO7816_ACTION_TX             /**< Transmit.                          */
#define HAL_ISO7816_ACTION_TXRX             LL_ISO7816_ACTION_TXRX           /**< Transmit, followed by RX.          */
/** @} */

/** @defgroup ISO7816_Interrupt_definition ISO7816 Interrupt Definition
  * @{
  */
#define HAL_ISO7816_INTR_TEST               LL_ISO7816_INTR_TEST       /**< Test interrupt                     */
#define HAL_ISO7816_INTR_PRESENCE           LL_ISO7816_INTR_PRESENCE   /**< Source presence interrupt          */
#define HAL_ISO7816_INTR_STATE_ERR          LL_ISO7816_INTR_STATE_ERR  /**< Source state error interrupt       */
#define HAL_ISO7816_INTR_DMA_ERR            LL_ISO7816_INTR_DMA_ERR    /**< Source dma error interrupt         */
#define HAL_ISO7816_INTR_RETRY_ERR          LL_ISO7816_INTR_RETRY_ERR  /**< Source retry error interrupt       */
#define HAL_ISO7816_INTR_RX_ERR             LL_ISO7816_INTR_RX_ERR     /**< Source rx error interrupt          */
#define HAL_ISO7816_INTR_DONE               LL_ISO7816_INTR_DONE       /**< Source done error interrupt        */
/** @} */


/** @defgroup ISO7816_HAL_CARD_PRESENCE Card Presence Defines
  * @{
  */
#define HAL_ISO7816_CARD_ABSENT            LL_ISO7816_CARD_ABSENT          /**< SIM Card is absent.   */
#define HAL_ISO7816_CARD_PRESENT           LL_ISO7816_CARD_PRESENT         /**< SIM Card is present.  */
/** @} */

/** @defgroup ISO7816_HAL_IO_STATES IO States Defines
  * @{
  */
#define HAL_ISO7816_IO_STATE_OFF             LL_ISO7816_IO_STATE_OFF         /**< Off                    */
#define HAL_ISO7816_IO_STATE_IDLE            LL_ISO7816_IO_STATE_IDLE        /**< Idle                   */
#define HAL_ISO7816_IO_STATE_RX_WAIT         LL_ISO7816_IO_STATE_RX_WAIT     /**< Receive Wait           */
#define HAL_ISO7816_IO_STATE_RX              LL_ISO7816_IO_STATE_RX          /**< Receive                */
#define HAL_ISO7816_IO_STATE_TX              LL_ISO7816_IO_STATE_TX          /**< Transmit               */
#define HAL_ISO7816_IO_STATE_TX_GUARD        LL_ISO7816_IO_STATE_TX_GUARD    /**< Transmit Guard         */
/** @} */

/** @defgroup ISO7816_HAL_PWR_STATES Power States Defines
  * @{
  */
#define HAL_ISO7816_PWR_STATE_OFF           LL_ISO7816_PWR_STATE_OFF         /**< Off                    */
#define HAL_ISO7816_PWR_STATE_PWRUP_VCC     LL_ISO7816_PWR_STATE_PWRUP_VCC   /**< Power up VCC           */
#define HAL_ISO7816_PWR_STATE_PWRUP_RST     LL_ISO7816_PWR_STATE_PWRUP_RST   /**< Power up reset         */
#define HAL_ISO7816_PWR_STATE_PWRDN_RST     LL_ISO7816_PWR_STATE_PWRDN_RST   /**< Power Down reset       */
#define HAL_ISO7816_PWR_STATE_PWRDN_VCC     LL_ISO7816_PWR_STATE_PWRDN_VCC   /**< Power Down VCC         */
#define HAL_ISO7816_PWR_STATE_STOP_PRE      LL_ISO7816_PWR_STATE_STOP_PRE    /**< Preparing Clock Stop   */
#define HAL_ISO7816_PWR_STATE_STOP          LL_ISO7816_PWR_STATE_STOP        /**< Clock Stopped          */
#define HAL_ISO7816_PWR_STATE_STOP_POST     LL_ISO7816_PWR_STATE_STOP_POST   /**< Exiting Clock Stop     */
#define HAL_ISO7816_PWR_STATE_IDLE          LL_ISO7816_PWR_STATE_IDLE        /**< Idle                   */
#define HAL_ISO7816_PWR_STATE_RX_TS0        LL_ISO7816_PWR_STATE_RX_TS0      /**< RX TS Character        */
#define HAL_ISO7816_PWR_STATE_RX_TS1        LL_ISO7816_PWR_STATE_RX_TS1      /**< RX TS Character        */
#define HAL_ISO7816_PWR_STATE_RX            LL_ISO7816_PWR_STATE_RX          /**< Receive                */
#define HAL_ISO7816_PWR_STATE_TX            LL_ISO7816_PWR_STATE_TX          /**< Transmit               */
#define HAL_ISO7816_PWR_STATE_TX_RX         LL_ISO7816_PWR_STATE_TX_RX       /**< Transmit and Receive   */
/** @} */

/** @defgroup ISO7816_Error_Code ISO7816 Error Code
  * @{
  */
#define HAL_ISO7816_ERROR_NONE              ((uint32_t)0x00000000)  /**< No error           */
#define HAL_ISO7816_ERROR_TIMEOUT           ((uint32_t)0x00000001)  /**< Timeout error      */
#define HAL_ISO7816_ERROR_TRANSFER          ((uint32_t)0x00000002)  /**< Transfer error     */
#define HAL_ISO7816_ERROR_INVALID_PARAM     ((uint32_t)0x00000008)  /**< Invalid parameters error */
/** @} */

/**
  * @brief ISO7816_default_config InitStruct default configuration
  */
#define ISO7816_DEFAULT_CONFIG                \
{                                             \
    .clk_div          = 0x2F,                 \
    .wait_time        = 0x6B,                 \
    .guard_time       = 0x00,                 \
    .detect_coding    = ENABLE,               \
}

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup ISO7816_Exported_Macros ISO7816 Exported Macros
  * @{
  */

/** @brief  Enable the specified ISO7816 interrupts.
  * @param  __HANDLE__      Specifies the ISO7816 Handle.
  * @param  __INTERRUPT__   Specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg @ref HAL_ISO7816_INTR_TEST Test interrupt
  *            @arg @ref HAL_ISO7816_INTR_PRESENCE Presence interrupt
  *            @arg @ref HAL_ISO7816_INTR_STATE_ERR State error interrupt
  *            @arg @ref HAL_ISO7816_INTR_DMA_ERR DMA error interrupt
  *            @arg @ref HAL_ISO7816_INTR_RETRY_ERR Retry error interrupt
  *            @arg @ref HAL_ISO7816_INTR_RX_ERR Rx error interrupt
  *            @arg @ref HAL_ISO7816_INTR_DONE Done error interrupt
  * @retval None
  */
#define __HAL_ISO7816_ENABLE_IT(__HANDLE__, __INTERRUPT__)         SET_BITS((__HANDLE__)->p_instance->INT_MASK, (__INTERRUPT__))

/** @brief  Disable the specified ISO7816 interrupts.
  * @param  __HANDLE__      Specifies the ISO7816 handle.
  * @param  __INTERRUPT__   Specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg @ref HAL_ISO7816_INTR_TEST Test interrupt
  *            @arg @ref HAL_ISO7816_INTR_PRESENCE Presence interrupt
  *            @arg @ref HAL_ISO7816_INTR_STATE_ERR State error interrupt
  *            @arg @ref HAL_ISO7816_INTR_DMA_ERR DMA error interrupt
  *            @arg @ref HAL_ISO7816_INTR_RETRY_ERR Retry error interrupt
  *            @arg @ref HAL_ISO7816_INTR_RX_ERR Rx error interrupt
  *            @arg @ref HAL_ISO7816_INTR_DONE Done error interrupt
  * @retval None
  */
#define __HAL_ISO7816_DISABLE_IT(__HANDLE__, __INTERRUPT__)        CLEAR_BITS((__HANDLE__)->p_instance->INT_MASK, (__INTERRUPT__))

/** @brief  Get the ISO7816 interrupt flags.
  * @param  __ISO7816_REGS      ISO7816 instance.
  * @retval ISO78116 Interrupt definitions.
  */
#define __HAL_ISO7816_IT_GET_IT_FLAG(__ISO7816_REGS)       ll_iso7816_get_it_flag(__ISO7816_REGS)

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_ISO7816_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 * @brief Transimit data in blocking mode
 *
 * @param p_iso7816 Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816.
 * @param tx_size Bytes of data to transmit
 * @param timeout Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 */
hal_status_t hal_iso7816_transmit(iso7816_handle_t *p_iso7816, uint16_t tx_size, uint32_t timeout);

/**
 * @brief Receive data in blocking mode
 *
 * @param p_iso7816 Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816.
 * @param rx_size Bytes of data to receive
 * @param timeout Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 */
hal_status_t hal_iso7816_receive(iso7816_handle_t *p_iso7816, uint16_t rx_size, uint32_t timeout);

/**
 * @brief Transimit and receive data in blocking mode
 *
 * @param p_iso7816 Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816.
 * @param tx_size Bytes of data to transmit
 * @param rx_size Bytes of data to receive
 * @param timeout Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 */
hal_status_t hal_iso7816_transmit_receive(iso7816_handle_t *p_iso7816, uint16_t tx_size,uint16_t rx_size, uint32_t timeout);

/**
 * @brief Transimit data in non-blocking mode with Interrupt
 *
 * @param p_iso7816  Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816.
 * @param tx_size Bytes of data to transmit
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 */
hal_status_t hal_iso7816_transmit_it(iso7816_handle_t *p_iso7816, uint16_t tx_size);

/**
 * @brief Receive data in non-blocking mode with Interrupt
 *
 * @param p_iso7816  Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816.
 * @param rx_size Bytes of data to receive
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 */
hal_status_t hal_iso7816_receive_it(iso7816_handle_t *p_iso7816, uint16_t rx_size);

/**
 * @brief Transimit and receive data in non-blocking mode with Interrupt
 *
 * @param p_iso7816: Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816.
 * @param tx_size:   Bytes of data to transfer.
 * @param rx_size:   Bytes of data to receive.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 */

hal_status_t hal_iso7816_transmit_receive_it(iso7816_handle_t *p_iso7816, uint16_t tx_size, uint16_t rx_size);

/**
 * @brief Transfer Abort functions
 *
 * @param p_iso7816 Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 */
hal_status_t hal_iso7816_abort(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  Initializes the ISO7816 according to the specified parameters
 *         in the iso7816_init_t and initialize the associated handle.
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration
 *                information for the specified ISO7816.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_iso7816_init(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  De-initializes the ISO7816 according to the specified parameters
 *         in the iso7816_init_t and initialize the associated handle.
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration
 *                information for the specified ISO7816.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_iso7816_deinit(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief Initialize the ISO7816 MSP.
 * @note  This function should not be modified. When the callback is needed,
 *            the hal_iso7816_msp_init could be implemented in the user file.
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816.
 ****************************************************************************************
 */
void hal_iso7816_msp_init(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief De-initialize the ISO7816 MSP.
 * @note  This function should not be modified. When the callback is needed,
 *            the hal_iso7816_msp_deinit could be implemented in the user file.
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816.
 ****************************************************************************************
 */
void hal_iso7816_msp_deinit(iso7816_handle_t *p_iso7816);
/**
 ****************************************************************************************
 * @brief  Handle ISO7816 interrupt request.
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816.
 ****************************************************************************************
 */
void hal_iso7816_irq_handler(iso7816_handle_t *p_iso7816);


/**
 ****************************************************************************************
 * @brief  Card presence state changed callback.
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816 module.
 ****************************************************************************************
 */
void hal_iso7816_presence_callback(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  Receive ART completed callback.
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816 module.
 ****************************************************************************************
 */
void hal_iso7816_atr_cplt_callback(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  Rx Transfer completed callback.
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816 module.
 ****************************************************************************************
 */
void hal_iso7816_rx_cplt_callback(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  Tx Transfer completed callback.
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816 module.
 ****************************************************************************************
 */
void hal_iso7816_tx_cplt_callback(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  Tx and Rx Transfer completed callback.
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816 module.
 ****************************************************************************************
 */
void hal_iso7816_tx_rx_cplt_callback(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  ISO7816 error callback.
 * @param[in]  p_iso7816: Pointer to an ISO7816 handle which contains the configuration information for the specified ISO7816 module.
 ****************************************************************************************
 */
void hal_iso7816_error_callback(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  ISO7816 Abort Completed callback.
 * @param[in]  p_iso7816: ISO7816 handle.
 ****************************************************************************************
 */
void hal_iso7816_abort_cplt_callback(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  Return the ISO7816 handle state.
 * @param[in]  p_iso7816: ISO7816 handle.
 * @retval ::HAL_ISO7816_STATE_RESET
 * @retval ::HAL_ISO7816_STATE_READY
 * @retval ::HAL_ISO7816_STATE_BUSY
 * @retval ::HAL_ISO7816_STATE_BUSY_TX
 * @retval ::HAL_ISO7816_STATE_BUSY_RX
 * @retval ::HAL_ISO7816_STATE_BUSY_TX_RX
 * @retval ::HAL_ISO7816_STATE_ABORT
 * @retval ::HAL_ISO7816_STATE_TIMEOUT
 * @retval ::HAL_ISO7816_STATE_ERROR
 ****************************************************************************************
 */
hal_iso7816_state_t hal_iso7816_get_state(iso7816_handle_t *p_iso7816);

/**
  * @brief  Request ISO7816 to go to the next action.
  * @param  p_iso7816: ISO7816 handle.
  * @param  action:    This parameter can be one of the following values:
  *         @arg @ref HAL_ISO7816_ACTION_NONE
  *         @arg @ref HAL_ISO7816_ACTION_OFF
  *         @arg @ref HAL_ISO7816_ACTION_STOPCLK
  *         @arg @ref HAL_ISO7816_ACTION_ON
  *         @arg @ref HAL_ISO7816_ACTION_WARMRST
  *         @arg @ref HAL_ISO7816_ACTION_RX
  *         @arg @ref HAL_ISO7816_ACTION_TX
  *         @arg @ref HAL_ISO7816_ACTION_TXRX
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
  */
hal_status_t hal_iso7816_set_action(iso7816_handle_t *p_iso7816, uint32_t action);

/**
  * @brief  Get ISO7816 Power States.
  * @param  p_iso7816: ISO7816 handle.
  * @retval Returned value can be one of the following values:
  *         @arg @ref HAL_ISO7816_PWR_STATE_OFF
  *         @arg @ref HAL_ISO7816_PWR_STATE_PWRUP_VCC
  *         @arg @ref HAL_ISO7816_PWR_STATE_PWRUP_RST
  *         @arg @ref HAL_ISO7816_PWR_STATE_PWRDN_RST
  *         @arg @ref HAL_ISO7816_PWR_STATE_PWRDN_VCC
  *         @arg @ref HAL_ISO7816_PWR_STATE_STOP_PRE
  *         @arg @ref HAL_ISO7816_PWR_STATE_STOP
  *         @arg @ref HAL_ISO7816_PWR_STATE_STOP_POST
  *         @arg @ref HAL_ISO7816_PWR_STATE_IDLE
  *         @arg @ref HAL_ISO7816_PWR_STATE_RX_TS0
  *         @arg @ref HAL_ISO7816_PWR_STATE_RX_TS1
  *         @arg @ref HAL_ISO7816_PWR_STATE_RX
  *         @arg @ref HAL_ISO7816_PWR_STATE_TX
  *         @arg @ref HAL_ISO7816_PWR_STATE_TX_RX
  */
uint32_t hal_iso7816_get_power_states(iso7816_handle_t *p_iso7816);

/**
  * @brief  Set divide ISO7816 clock.
  * @note   Divide SIM clock by this value+1 to define ETU length. The reset  value
  *         is the one, needed for theATR.
  * @param  p_iso7816: ISO7816 handle.
  * @param  divide This parameter should range between 0x0 and 0x3FF.
  * @retval ::HAL_OK: Operation is OK.
  * @retval ::HAL_ERROR: Parameter error or operation not supported.
  * @retval ::HAL_BUSY: Driver is busy.
  * @retval ::HAL_TIMEOUT: Timeout occurred.
  */
hal_status_t hal_iso7816_set_etudiv(iso7816_handle_t *p_iso7816, uint32_t divide);

/**
 ****************************************************************************************
 * @brief  Return the ISO7816 error code.
 * @param[in]  p_iso7816: ISO7816 handle.
 * @return ISO7816 Error Code
 ****************************************************************************************
 */
uint32_t hal_iso7816_get_error(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to ISO7816 configuration before sleep.
 * @param[in]  p_iso7816: ISO7816 handle.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_iso7816_suspend_reg(iso7816_handle_t *p_iso7816);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to ISO7816 configuration after sleep.
 *         This function must be used in conjunction with the hal_iso7816_suspend_reg().
 * @param[in]  p_iso7816: ISO7816 handle.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_iso7816_resume_reg(iso7816_handle_t *p_iso7816);


/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_ISO7816_H__ */

/** @} */

/** @} */

/** @} */

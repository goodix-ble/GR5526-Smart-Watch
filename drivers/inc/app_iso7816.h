/**
 ****************************************************************************************
 *
 * @file    app_iso7816.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of ISO7816 app library.
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

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_ISO7816 ISO7816
  * @brief ISO7816 APP module driver.
  * @{
  */


#ifndef _APP_ISO7816_H_
#define _APP_ISO7816_H_

#include "grx_hal.h"
#include "app_io.h"
#include "app_drv_error.h"
#include "app_drv_config.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_ISO7816_MODULE_ENABLED
/** @addtogroup APP_ISO7816_DEFINE Defines
  * @{
  */

/** @defgroup APP_ISO7816_ACTION Action state
  * @{
  */
#define APP_ISO7816_ACTION_NONE             HAL_ISO7816_ACTION_NONE          /**< Do Nothing                         */
#define APP_ISO7816_ACTION_OFF              HAL_ISO7816_ACTION_OFF           /**< Switch Off                         */
#define APP_ISO7816_ACTION_STOPCLK          HAL_ISO7816_ACTION_STOPCLK       /**< Stop the clock                     */
#define APP_ISO7816_ACTION_ON               HAL_ISO7816_ACTION_ON            /**< Switch on and receive ATR          */
#define APP_ISO7816_ACTION_WARMRST          HAL_ISO7816_ACTION_WARMRST       /**< Trigger warm reset and receive ATR */
#define APP_ISO7816_ACTION_RX               HAL_ISO7816_ACTION_RX            /**< Receive                            */
#define APP_ISO7816_ACTION_TX               HAL_ISO7816_ACTION_TX            /**< Transmit                           */
#define APP_ISO7816_ACTION_TXRX             HAL_ISO7816_ACTION_TXRX          /**< Transmit, followed by RX           */
/** @} */

/** @defgroup APP_ISO7816_Interrupt_definition ISO7816 Interrupt Definition
  * @{
  */
#define APP_ISO7816_INTR_PRESENCE           HAL_ISO7816_INTR_PRESENCE        /**< Source presence interrupt          */
#define APP_ISO7816_INTR_STATE_ERR          HAL_ISO7816_INTR_STATE_ERR       /**< Source state error interrupt       */
#define APP_ISO7816_INTR_DMA_ERR            HAL_ISO7816_INTR_DMA_ERR         /**< Source dma error interrupt         */
#define APP_ISO7816_INTR_RETRY_ERR          HAL_ISO7816_INTR_RETRY_ERR       /**< Source retry error interrupt       */
#define APP_ISO7816_INTR_RX_ERR             HAL_ISO7816_INTR_RX_ERR          /**< Source rx error interrupt          */
#define APP_ISO7816_INTR_DONE               HAL_ISO7816_INTR_DONE            /**< Source done error interrupt        */
/** @} */


/** @defgroup APP_ISO7816_CARD_PRESENCE Card Presence Defines
  * @{
  */
#define APP_ISO7816_CARD_ABSENT            HAL_ISO7816_CARD_ABSENT          /**< SIM Card is absent    */
#define APP_ISO7816_CARD_PRESENT           HAL_ISO7816_CARD_PRESENT         /**< SIM Card is present   */
/** @} */

/** @defgroup APP_ISO7816_IO_STATES IO States Defines
  * @{
  */
#define APP_ISO7816_IO_STATE_OFF            HAL_ISO7816_IO_STATE_OFF         /**< Off                    */
#define APP_ISO7816_IO_STATE_IDLE           HAL_ISO7816_IO_STATE_IDLE        /**< Idle                   */
#define APP_ISO7816_IO_STATE_RX_WAIT        HAL_ISO7816_IO_STATE_RX_WAIT     /**< Receive Wait           */
#define APP_ISO7816_IO_STATE_RX             HAL_ISO7816_IO_STATE_RX          /**< Receive                */
#define APP_ISO7816_IO_STATE_TX             HAL_ISO7816_IO_STATE_TX          /**< Transmit               */
#define APP_ISO7816_IO_STATE_TX_GUARD       HAL_ISO7816_IO_STATE_TX_GUARD    /**< Transmit Guard         */
/** @} */

/** @defgroup APP_ISO7816_PWR_STATES Power States Defines
  * @{
  */
#define APP_ISO7816_PWR_STATE_OFF           HAL_ISO7816_PWR_STATE_OFF         /**< Off                    */
#define APP_ISO7816_PWR_STATE_PWRUP_VCC     HAL_ISO7816_PWR_STATE_PWRUP_VCC   /**< Power up VCC           */
#define APP_ISO7816_PWR_STATE_PWRUP_RST     HAL_ISO7816_PWR_STATE_PWRUP_RST   /**< Power up reset         */
#define APP_ISO7816_PWR_STATE_PWRDN_RST     HAL_ISO7816_PWR_STATE_PWRDN_RST   /**< Power Down reset       */
#define APP_ISO7816_PWR_STATE_PWRDN_VCC     HAL_ISO7816_PWR_STATE_PWRDN_VCC   /**< Power Down VCC         */
#define APP_ISO7816_PWR_STATE_STOP_PRE      HAL_ISO7816_PWR_STATE_STOP_PRE    /**< Preparing Clock Stop   */
#define APP_ISO7816_PWR_STATE_STOP          HAL_ISO7816_PWR_STATE_STOP        /**< Clock Stopped          */
#define APP_ISO7816_PWR_STATE_STOP_POST     HAL_ISO7816_PWR_STATE_STOP_POST   /**< Exiting Clock Stop     */
#define APP_ISO7816_PWR_STATE_IDLE          HAL_ISO7816_PWR_STATE_IDLE        /**< Idle                   */
#define APP_ISO7816_PWR_STATE_RX_TS0        HAL_ISO7816_PWR_STATE_RX_TS0      /**< RX TS Character        */
#define APP_ISO7816_PWR_STATE_RX_TS1        HAL_ISO7816_PWR_STATE_RX_TS1      /**< RX TS Character        */
#define APP_ISO7816_PWR_STATE_RX            HAL_ISO7816_PWR_STATE_RX          /**< Receive                */
#define APP_ISO7816_PWR_STATE_TX            HAL_ISO7816_PWR_STATE_TX          /**< Transmit               */
#define APP_ISO7816_PWR_STATE_TX_RX         HAL_ISO7816_PWR_STATE_TX_RX       /**< Transmit and Receive   */
/** @} */

/** @} */

/** @addtogroup APP_ISO7816_ENUM Enumerations
  * @{
  */

/**
  * @brief ISO7816 operating mode Enumerations definition
  */
typedef enum
{
    APP_ISO7816_TYPE_INTERRUPT,        /**< Interrupt operation mode. */
    APP_ISO7816_TYPE_POLLING,          /**< Polling operation mode. */
    APP_ISO7816_TYPE_MAX,              /**< Only for check parameter, not used as input parameters. */
} app_iso7816_mode_t;

/**
  * @brief ISO7816 event Enumerations definition
  */
typedef enum
{
    APP_ISO7816_EVT_ERROR,             /**< Error reported by ISO7816 peripheral. */
    APP_ISO7816_EVT_ABORT,             /**< Error reported by ISO7816 peripheral. */
    APP_ISO7816_EVT_PRESENCE,          /**< Requested RX transfer completed. */
    APP_ISO7816_EVT_ATR_CPLT,          /**< Requested TX transfer completed. */
    APP_ISO7816_EVT_TX_CPLT,           /**< Requested TX transfer completed. */
    APP_ISO7816_EVT_RX_CPLT,           /**< Requested RX transfer completed. */
    APP_ISO7816_EVT_TX_RX_CPLT,        /**< Requested RX transfer completed. */
} app_iso7816_evt_type_t;
/** @} */

/** @addtogroup APP_ISO7816_STRUCTURES Structures
  * @{
  */

/**
  * @brief ISO7816 IO configuration Structures
  */
typedef struct
{
    app_io_type_t        type;     /**< Specifies the type of IO. */
    app_io_mux_t         mux;      /**< Specifies the Peripheral to be connected to the selected pins. */
    uint32_t             pin;      /**< Specifies the IO pins to be configured.
                                        This parameter can be any value of @ref GR5xxx_pins. */
    app_io_pull_t        pull;     /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
} app_iso7816_pin_t;

/**
  * @brief ISO7816 pin configure structure definition
  */
typedef struct
{
    app_iso7816_pin_t       clk;       /**< Set the configuration of ISO7816 clock pin. */
    app_iso7816_pin_t       rst;       /**< Set the configuration of ISO7816 reset pin. */
    app_iso7816_pin_t       io;        /**< Set the configuration of ISO7816 io pin. */
    app_iso7816_pin_t       presence;  /**< Set the configuration of ISO7816 presence pin. */
} app_iso7816_pin_cfg_t;

/**
  * @brief ISO7816 event structure definition
  */
typedef struct
{
    app_iso7816_evt_type_t type; /**< Type of event. */
    union
    {
        uint32_t error_code;     /**< ISO7816 Error code. */
        uint16_t size;           /**< ISO7816 transmitted/received counter. */
    } data;                      /**< ISO7816 union data. */
} app_iso7816_evt_t;

/** @} */

/** @addtogroup APP_ISO7816_TYPEDEFS Type definitions
  * @{
  */

/**
  * @brief ISO7816 event callback definition
  */
typedef void (*app_iso7816_evt_handler_t)(app_iso7816_evt_t *p_evt);

/** @} */

/** @addtogroup APP_ISO7816_ENUM Enumerations
  * @{
  */

/**@brief App iso7816 state types. */
typedef enum
{
    APP_ISO7816_INVALID = 0,
    APP_ISO7816_ACTIVITY,
#ifdef APP_DRIVER_WAKEUP_CALL_FUN
    APP_ISO7816_SLEEP,
#endif
} app_iso7816_state_t;

/** @} */

/** @addtogroup APP_ISO7816_STRUCTURES Structures
  * @{
  */
/**
  * @brief ISO7816 device structure definition
  */
typedef struct
{
    app_iso7816_evt_handler_t   evt_handler;           /**< ISO7816 event callback. */
    iso7816_handle_t            handle;                /**< ISO7816 handle Structure. */
    app_iso7816_mode_t          use_mode;              /**< ISO7816 operating mode Enumerations. */
    app_iso7816_pin_cfg_t       *p_pin_cfg;            /**< ISO7816 pin configure structure. */
    app_iso7816_state_t         iso7816_state;         /**< ISO7816 state configure structure. */
    bool                        start_flag;            /**< ISO7816 start_flage. */
}iso7816_env_t;

/**
  * @brief ISO7816 parameters structure definition
  */
typedef struct
{
    app_iso7816_mode_t      use_mode;        /**< Specifies the operation mode of ISO7816. */
    app_iso7816_pin_cfg_t   pin_cfg;         /**< The pin configuration information for the ISO7816. */
    iso7816_init_t          init;            /**< ISO7816 communication parameters. */
    iso7816_env_t           iso7816_env;     /**< ISO7816 device structure definition. */
} app_iso7816_params_t;

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_ISO7816_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP ISO7816 DRIVER according to the specified parameters
 *         in the app_iso7816_params_t and app_iso7816_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_iso7816_params_t parameter which contains the
 *                       configuration information for the specified ISO7816 module.
 * @param[in]  evt_handler: ISO7816 user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_iso7816_init(app_iso7816_params_t *p_params, app_iso7816_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP ISO7816 DRIVER peripheral.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_iso7816_deinit(void);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in blocking mode.
 *
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_iso7816_receive_sync(uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in non-blocking mode with Interrupt/DMA.
 *
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_iso7816_receive_async(uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmits an amount of data in blocking mode.
 *
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_iso7816_transmit_sync(uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmits an amount of data in non-blocking mode with Interrupt/DMA.
 *
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_iso7816_transmit_async(uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmit and receive in master mode an amount of data in blocking mode.
 *
 * @param[in]  tx_size: Amount of data to be sent
 * @param[in]  rx_size: Amount of data to be receive
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_iso7816_transmit_receive_sync(uint16_t tx_size, uint16_t rx_size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit and receive in master mode an amount of data in non-blocking mode.
 *
 * @param[in]  tx_size: Amount of data to be sent
 * @param[in]  rx_size: Amount of data to be receive
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_iso7816_transmit_receive_async(uint16_t tx_size, uint16_t rx_size);

/**
  * @brief  Request ISO7816 to go to the next action.
  * @param  action: This parameter can be one of the following values:
  *         @arg @ref APP_ISO7816_ACTION_NONE
  *         @arg @ref APP_ISO7816_ACTION_OFF
  *         @arg @ref APP_ISO7816_ACTION_STOPCLK
  *         @arg @ref APP_ISO7816_ACTION_ON
  *         @arg @ref APP_ISO7816_ACTION_WARMRST
  *         @arg @ref APP_ISO7816_ACTION_RX
  *         @arg @ref APP_ISO7816_ACTION_TX
  *         @arg @ref APP_ISO7816_ACTION_TXRX
  *
  * @return Result of operation.
  */
uint16_t app_iso7816_set_action(uint32_t action);

/**
  * @brief  Get ISO7816 Power States.
  * @return Returned value can be one of the following values:
  *         @arg @ref APP_ISO7816_PWR_STATE_OFF
  *         @arg @ref APP_ISO7816_PWR_STATE_PWRUP_VCC
  *         @arg @ref APP_ISO7816_PWR_STATE_PWRUP_RST
  *         @arg @ref APP_ISO7816_PWR_STATE_PWRDN_RST
  *         @arg @ref APP_ISO7816_PWR_STATE_PWRDN_VCC
  *         @arg @ref APP_ISO7816_PWR_STATE_STOP_PRE
  *         @arg @ref APP_ISO7816_PWR_STATE_STOP
  *         @arg @ref APP_ISO7816_PWR_STATE_STOP_POST
  *         @arg @ref APP_ISO7816_PWR_STATE_IDLE
  *         @arg @ref APP_ISO7816_PWR_STATE_RX_TS0
  *         @arg @ref APP_ISO7816_PWR_STATE_RX_TS1
  *         @arg @ref APP_ISO7816_PWR_STATE_RX
  *         @arg @ref APP_ISO7816_PWR_STATE_TX
  *         @arg @ref APP_ISO7816_PWR_STATE_TX_RX
  */
uint32_t app_iso7816_get_power_states(void);

/**
  * @brief  Set divide ISO7816 clock.
  * @note   Divide SIM clock by this value+1 to define ETU length. The reset  value
  *         is the one, needed for theATR.
  * @param  divide This parameter should range between 0x0 and 0x3FF.
  *
  * @return Result of operation.
  */
uint16_t app_iso7816_set_etudiv(uint32_t divide);

/**
 ****************************************************************************************
 * @brief  Return the ISO7816 handle.
 *
 * @return Pointer to the specified ID's ISO7816 handle.
 ****************************************************************************************
 */
iso7816_handle_t *app_iso7816_get_handle(void);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif  /* _APP_ISO7816_H_ */

/** @} */
/** @} */
/** @} */


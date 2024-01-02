/**
 ****************************************************************************************
 *
 * @file    app_bod.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of BOD app library.
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
#ifndef _APP_BOD_H_
#define _APP_BOD_H_

#include "grx_hal.h"
#include "app_drv_error.h"
#include "app_drv_config.h"

#ifdef HAL_BOD_MODULE_ENABLED
/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_BOD BOD
  * @brief BOD APP module driver.
  * @{
  */

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup APP_BOD_ENUM Enumerations
  * @{
  */
/**
  * @brief BOD event Enumerations definition
  */
typedef enum
{
    APP_BOD_EVT_TRIGGERED,              /**< BOD interrupt trigger event.  */
    APP_BOD_EVT_REMOVED,                /**< BOD interrupt condition removed event.  */
} app_bod_evt_type_t;

/**@brief App bod state types. */
typedef enum
{
    APP_BOD_INVALID = 0,
    APP_BOD_ACTIVITY,
} app_bod_state_t;

/** @} */

/**
  * @defgroup  APP_BOD_MACRO Defines
  * @{
  */

/** @defgroup APP_BOD_Exported_Constants BOD Exported Constants
  * @{
  */

/** @defgroup APP_BOD_ENABLE BOD enable defines
  * @{
  */
#define APP_BOD_ENABLE                      HAL_BOD_ENABLE    /**< BOD enable  */
#define APP_BOD_DISABLE                     HAL_BOD_DISABLE   /**< BOD disable  */
/** @} */

/** @defgroup APP_BOD_EVENT_ENABLE BOD event enable
  * @{
  */
#define APP_BOD_EVENT_ENABLE                HAL_BOD2_ENABLE   /**< BOD event enable  */
#define APP_BOD_EVENT_DISABLE               HAL_BOD2_DISABLE  /**< BOD event disable  */
/** @} */

/** @defgroup APP_BOD_STATIC_ENABLE BOD STATIC ENABLE
  * @{
  */
#define APP_BOD_STATIC_ENABLE               HAL_BOD_STATIC_ENABLE    /**< BOD STATIC enable  */
#define APP_BOD_STATIC_DISABLE              HAL_BOD_STATIC_DISABLE   /**< BOD STATIC disable  */
/** @} */

/** @defgroup APP_BOD_EVENT_LEVEL BOD event level
  * @{
  */
#define APP_BOD_EVENT_LEVEL_0               HAL_BOD2_LEVEL_0  /**< BOD event Level 0  */
#define APP_BOD_EVENT_LEVEL_1               HAL_BOD2_LEVEL_1  /**< BOD event Level 1  */
#define APP_BOD_EVENT_LEVEL_2               HAL_BOD2_LEVEL_2  /**< BOD event Level 2  */
#define APP_BOD_EVENT_LEVEL_3               HAL_BOD2_LEVEL_3  /**< BOD event Level 3  */
#define APP_BOD_EVENT_LEVEL_4               HAL_BOD2_LEVEL_4  /**< BOD event Level 4  */
#define APP_BOD_EVENT_LEVEL_5               HAL_BOD2_LEVEL_5  /**< BOD event Level 5  */
#define APP_BOD_EVENT_LEVEL_6               HAL_BOD2_LEVEL_6  /**< BOD event Level 6  */
#define APP_BOD_EVENT_LEVEL_7               HAL_BOD2_LEVEL_7  /**< BOD event Level 7  */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X)
#define APP_BOD_EVENT_LEVEL_8               HAL_BOD2_LEVEL_8  /**< BOD event Level 8  */
#define APP_BOD_EVENT_LEVEL_9               HAL_BOD2_LEVEL_9  /**< BOD event Level 9  */
#define APP_BOD_EVENT_LEVEL_10              HAL_BOD2_LEVEL_10 /**< BOD event Level 10  */
#define APP_BOD_EVENT_LEVEL_11              HAL_BOD2_LEVEL_11 /**< BOD event Level 11  */
#define APP_BOD_EVENT_LEVEL_12              HAL_BOD2_LEVEL_12 /**< BOD event Level 12  */
#define APP_BOD_EVENT_LEVEL_13              HAL_BOD2_LEVEL_13 /**< BOD event Level 13  */
#define APP_BOD_EVENT_LEVEL_14              HAL_BOD2_LEVEL_14 /**< BOD event Level 14  */
#define APP_BOD_EVENT_LEVEL_15              HAL_BOD2_LEVEL_15 /**< BOD event Level 15  */
#endif
/** @} */

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
/** @defgroup APP_BOD_EVENT_AUTO_POWER_BYPASS_ENABLE event enable
  * @{
  */
#define APP_BOD_EVENT_AUTO_POWER_BYPASS_ENABLE                HAL_BOD2_AUTO_POWER_BYPASS_ENABLE   /**< BOD event enable  */
#define APP_BOD_EVENT_AUTO_POWER_BYPASS_DISABLE               HAL_BOD2_AUTO_POWER_BYPASS_DISABLE  /**< BOD event disable  */
/** @} */


#endif

/** @} */
/** @} */

/** @addtogroup APP_BOD_STRUCTURES Structures
  * @{
  */

/**
  * @brief BOD event structure definition
  */
typedef struct
{
    app_bod_evt_type_t  type;           /**< Type of event. */
} app_bod_evt_t;

/** @} */

/** @addtogroup APP_BOD_TYPEDEFS Type definitions
  * @{
  */
/**
  * @brief BOD event callback definition
  */
typedef void (*app_bod_evt_handler_t)(app_bod_evt_t *p_evt);

/** @} */

/** @addtogroup APP_BOD_STRUCTURES Structures
  * @{
  */

/**
  * @brief BOD device structure definition
  */
typedef struct
{
    app_bod_evt_handler_t   evt_handler;             /**< BOD event callback. */
    bod_handle_t            handle;                  /**< BOD handle Structure. */
    app_bod_state_t         bod_state;               /**< App bod state types. */
}bod_env_t;

/**
  * @brief BOD parameters structure definition
  */
typedef struct
{
    bod_init_t          init;               /**< BOD init parameters. */
    bod_env_t           bod_env;            /**< BOD device structure. */
} app_bod_params_t;

/** @} */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_BOD_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP BOD DRIVER.
 *
 * @param[in]  p_params: Pointer to app_bod_params_t parameter which contains the
 *                       configuration information for the specified BOD module.
 * @param[in]  evt_handler: BOD user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_bod_init(app_bod_params_t *p_params, app_bod_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief De-initialize the APP BOD DRIVER.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_bod_deinit(void);

/**
 ****************************************************************************************
 * @brief  Enable or disable hardware BOD.
 *
 * @param[in]  enable: bod enable flag. the value can be APP_BOD_ENABLE or APP_BOD_DISABLE.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_bod_enable(uint8_t enable);

/**
 ****************************************************************************************
 * @brief  Enable or disable BOD Event.
 *
 * @param[in]  enable: bod event enable flag. the value can be APP_BOD_EVENT_ENABLE or APP_BOD_EVENT_DISABLE.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_bod_event_enable(uint8_t enable);

/**
 ****************************************************************************************
 * @brief  Set BOD EVENT control level..
 *
 * @param[in]  level: the level of bod event control.the value range between 0x0 ~ 0xF.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_bod_event_set_level(uint8_t level);

/**
 ****************************************************************************************
 * @brief  Enable or disable static mode.
 *
 * @param[in]  enable: static mode enable flag. the value can be APP_BOD_STATIC_ENABLE or APP_BOD_STATIC_DISABLE.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_bod_static_mode_enable(uint8_t enable);

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
/**
 ****************************************************************************************
 * @brief  Enable or disable BOD EVENT auto power bypass mode.
 *
 * @param[in]  enable: static mode enable flag. the value can be APP_BOD_EVENT_AUTO_POWER_BYPASS_ENABLE or APP_BOD_EVENT_AUTO_POWER_BYPASS_DISABLE.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_bod_event_auto_power_bypass_enable(uint8_t enable);

#endif

/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
/** @} */
/** @} */

#endif //END #ifdef HAL_BOD_MODULE_ENABLED
#endif //END #ifndef _APP_BOD_H_

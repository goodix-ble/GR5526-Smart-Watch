/**
 ****************************************************************************************
 *
 * @file gr55xx_sys_cfg.h
 *
 * @brief Define the chip configuration
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
 *****************************************************************************************
 */

 /**
 * @addtogroup SYSTEM
 * @{
 */
 
/**
 * @addtogroup SYS_CFG
 * @{
 * @brief Definitions and prototypes for SYS_CFG interface.
 */
 
#ifndef __GR55XX_SYS_CFG_H__
#define __GR55XX_SYS_CFG_H__

#include <cmsis_compiler.h>
#define __ARRAY_EMPTY

/**
 * @defgroup GR55XX_SYS_CFG_STRUCT Structures
 * @{
 */
/**@brief BLE Sleep configure defination. */
typedef struct
{
    uint8_t  sleep_enable;           /**< Sleep enable flag. */
    uint8_t  ext_wakeup_enable;      /**< External wake-up support. */
    uint16_t twosc;                  /**< Twosc delay. */
    uint16_t twext;                  /**< Twext delay. */
    uint16_t twrm;                   /**< Twrm delay. */
    uint16_t sleep_algo_dur;         /**< Duration of sleep and wake-up algorithm (depends on CPU speed) expressed in half us.. */
} ble_slp_config_t ;

/**@brief BLE Scheduler configure defination. */
typedef struct
{
    uint8_t prog_delay;             /**< Programme delay. */
} ble_sch_config_t;

/**@brief GR55XX Chip configure defination */
typedef struct
{
    ble_slp_config_t ble_slp_cfg;    /**< BLE Sleep configure. */
    ble_sch_config_t ble_sch_cfg;    /**< BLE Sch configure. */
} gr55xx_chip_config_t;
/** @} */

#endif
/** @} */
/** @} */

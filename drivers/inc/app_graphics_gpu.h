/**
 ****************************************************************************************
 *
 * @file    app_graphics_gpu.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of GPU app library.
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

/** @defgroup APP_GRAPHICS_GPU GPU
  * @brief GRAPHICS_GPU APP module driver.
  * @{
  */

#ifndef __APP_GRAPHICS_GPU_H__
#define __APP_GRAPHICS_GPU_H__

#include "gr55xx.h"
#include "app_io.h"
#include "hal_gfx_regs.h"
#include "hal_gfx_core.h"
#ifdef USE_OSAL
    #include "osal.h"
#else
    #include "app_rtos_cfg.h"
#endif
#include "graphics_sys_defs.h"

/**
  * @defgroup APP_GRAPHICS_GPU_DRIVER_MACRO Defines
  * @{
  */
/** @defgroup GRAPHICS_GPU_BASEADDR The GPU registers memory base address
  * @{
  */
#define GRAPHICS_GPU_BASEADDR                   0xA3FF0000 /**< GPU registers memory base address */
/** @} */

/** @defgroup GRAPHICS_GPU_IRQ_EVT IRQ callback events Define
  * @{
  */
#define GGPU_IRQ_EVT_CMD_LIST_END               0x01 /**< GPU CMD list end IRQ callback */
#define GGPU_IRQ_EVT_DRAW_CMD_END               0x02 /**< GPU draw CMD end IRQ callback */
#define GGPU_IRQ_EVT_IRQ_ID_CLEAR               0x03 /**< GPU IRQ ID clear callback */
#define GGPU_IRQ_EVT_OTHER                      0x04 /**< GPU other IRQ callback */
/** @} */

/** @} */

/** @addtogroup APP_GRAPHICS_GPU_DRIVER_ENUM Enumerations
  * @{
  */

/**
  * @brief GPU Power Mode Enumerations definition
  */
typedef enum {
    GPU_POWER_STATE_SLEEP = 0,        /* sleep  state */
    GPU_POWER_STATE_ACTIVE = 1,       /* active state */
} graphics_gpu_power_state_e;

/** @} */

/** @addtogroup APP_GRAPHICS_GPU_DRIVER_TYPEDEF Graphics GPU Type definitions
  * @{
  */
typedef void (* graphics_gpu_irq_event_notify )(uint32_t evt); /**< GPU IRQ callback definition */
/** @} */

/** @addtogroup APP_GRAPHICS_GPU_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 *****************************************************************************************
 * @brief GPU initialize.
 *
 * @param[in] evt_cb: Callback function
 *
 * @return 0 if no errors occurred
 *****************************************************************************************
 */
uint16_t graphics_gpu_init(graphics_gpu_irq_event_notify evt_cb);

/**
 *****************************************************************************************
 * @brief GPU de-initialize. just called when needed to reboot/reset
 *
 *****************************************************************************************
 */
void graphics_gpu_deinit(void);

/**
 *****************************************************************************************
 * @brief Switch power state for GPU module
 *
 * @param[in] state: power state to switch
 *
 * @return none
 *****************************************************************************************
 */
void app_graphics_gpu_set_power_state(graphics_gpu_power_state_e state);

/** @} */

#endif /* __APP_GRAPHICS_GPU_H__ */

/** @} */
/** @} */
/** @} */

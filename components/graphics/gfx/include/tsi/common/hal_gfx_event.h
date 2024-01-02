/**
 ****************************************************************************************
 *
 * @file    hal_gfx_event.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of Graphics library.
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

/** @addtogroup GRAPHICS_SDK Graphics
 *  @{
 */

/** @addtogroup HAL_GFX HAL GFX
  * @{
  */

/** @defgroup HAL_GFX_EVENT GFX EVENT
 * @brief graphics event. Deprecated, Not suggessted to use
 * @{
 */

#ifndef HAL_GFX_EVENT_H__
#define HAL_GFX_EVENT_H__

#include "hal_gfx_sys_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GFX_EVENT_MACRO Defines
 * @{
 */

#define HAL_GFX_EVENT_HIDE_CURSOR  (1 << 1)     /**< hide cursor event */
/** @} */


/** @addtogroup HAL_GFX_EVENT_ENUM Enumerations
  * @{
  */

/**
  * @brief  Mouse Event for Graphics
  */
typedef enum {
    MOUSE_EVENT_NONE = 0,               /**< none event */
    MOUSE_EVENT_LEFT_CLICK,             /**< left click event */
    MOUSE_EVENT_LEFT_RELEASE,           /**< left release event */
    MOUSE_EVENT_MIDDLE_CLICK,           /**< middle click event */
    MOUSE_EVENT_MIDDLE_RELEASE,         /**< middle release event */
    MOUSE_EVENT_RIGHT_CLICK,            /**< right click event */
    MOUSE_EVENT_RIGHT_RELEASE,          /**< right release event */
    MOUSE_EVENT_SCROLL_UP,              /**< scroll up event */
    MOUSE_EVENT_SCROLL_DOWN,            /**< scroll down event */
    MOUSE_EVENT_MAX                     /**< max event flag */
} hal_gfx_mouse_event_t;

/**
  * @brief  KeyBoard Event for Graphics
  */
typedef enum {
    KB_EVENT_NONE  = 0,
    KB_EVENT_PRESS,
    KB_EVENT_HOLD,
    KB_EVENT_RELEASE,
    KB_EVENT_MAX
} hal_gfx_kb_event_t;

/**
  * @brief  Mouse State for Graphics
  */
typedef enum {
    MOUSE_STATE_NONE           = 0,
    MOUSE_STATE_LEFT_CLICKED   = 1,
    MOUSE_STATE_MIDDLE_CLICKED = 1U<<1,
    MOUSE_STATE_RIGHT_CLICKED  = 1U<<2
} hal_gfx_mouse_state_t;

/** @} */

/** @addtogroup HAL_GFX_EVENT_STRUCT Structure
  * @{
  */

/**
  * @brief  Event Structure for Graphics
  */
typedef struct {
    int mouse_x;                    /**< x-coordinate for mouse */
    int mouse_y;                    /**< y-coordinate for mouse */
    int mouse_dx;                   /**< delta x-coordinate for mouse */
    int mouse_dy;                   /**< delta y-coordinate for mouse */
    int mouse_event;                /**< mouse event */
    int mouse_state;                /**< mouse state */
    int kb_event;                   /**< keyboard event */
    char kb_key;                    /**< keyboard key value */
    int timer_id;                   /**< timer id */
    uint32_t timer_expirations;     /**< expire time */
} hal_gfx_event_t;

/** @} */


/**
 * @defgroup HAL_GFX_EVENT_FUNCTION Functions
 * @{
 */

/**
 *****************************************************************************************
 * @brief event init function
 *
 * @param[in] flags: event flags
 * @param[in] mouse_init_x: init x-coord for mouse
 * @param[in] mouse_init_y: init y-coord for mouse
 * @param[in] mouse_max_x: max x-coord for mouse
 * @param[in] mouse_max_y: max y-coord for mouse
 *
 * @return init result
 *****************************************************************************************
 */
int  hal_gfx_event_init(int flags, int mouse_init_x, int mouse_init_y, int mouse_max_x, int mouse_max_y);

/**
 *****************************************************************************************
 * @brief wait event init function
 *
 * @param[in] event: point to event structure
 * @param[in] block_until_event: block time till event happens
 *
 * @return wait result
 *****************************************************************************************
 */
int  hal_gfx_event_wait(hal_gfx_event_t *event, int block_until_event);

/**
 *****************************************************************************************
 * @brief force setting cursy to [x,y]
 *
 * @param[in] x: x-coord to set
 * @param[in] y: y-coord to set
 *
 * @return none
 *****************************************************************************************
 */
void hal_gfx_event_force_cursor_xy(int x, int y);

/**
 *****************************************************************************************
 * @brief Init triple framebuffer (Not USED)
 *
 * @param[in] layer: graphics layer to set
 * @param[in] fb0_phys: layer0's phical address
 * @param[in] fb1_phys: layer1's phical address
 * @param[in] fb2_phys: layer2's phical address
 *
 * @return none
 *****************************************************************************************
 */
uintptr_t hal_gfx_init_triple_fb(int layer, uintptr_t fb0_phys, uintptr_t fb1_phys, uintptr_t fb2_phys);

/**
 *****************************************************************************************
 * @brief Swap layer to current layer
 *
 * @param[in] layer: graphics layer to set
 *
 * @return layer's phical address
 *****************************************************************************************
 */
uintptr_t hal_gfx_swap_fb(int layer);

/**
 *****************************************************************************************
 * @brief Create a timer, need to porting
 *
 * @return timer id
 *****************************************************************************************
 */
int hal_gfx_timer_create(void);

/**
 *****************************************************************************************
 * @brief Destroy timer, need to porting
 *
 * @param[in] timer_id: timer id
 *
 * @return none
 *****************************************************************************************
 */
void hal_gfx_timer_destroy(int timer_id);

/**
 *****************************************************************************************
 * @brief Set periodic timer, need to porting
 *
 * @param[in] timer_id: timer id
 * @param[in] timeout_milisecs: periodic time
 *
 * @return 1 - successful; 0 -fail
 *****************************************************************************************
 */
int  hal_gfx_timer_set_periodic(int timer_id, uint32_t timeout_milisecs);

/**
 *****************************************************************************************
 * @brief Set one-shot timer, need to porting
 *
 * @param[in] timer_id: timer id
 * @param[in] timeout_milisecs: timeout time
 *
 * @return 1 - successful; 0 -fail
 *****************************************************************************************
 */
int  hal_gfx_timer_set_oneshot(int timer_id, uint32_t timeout_milisecs);

/**
 *****************************************************************************************
 * @brief Stop timer, need to porting
 *
 * @param[in] timer_id: timer id
 *
 * @return none
 *****************************************************************************************
 */
void hal_gfx_timer_stop(int timer_id);
/** @} */

#ifdef __cplusplus
}
#endif

#endif //HAL_GFX_EVENT_H__
/** @} */
/** @} */
/** @} */


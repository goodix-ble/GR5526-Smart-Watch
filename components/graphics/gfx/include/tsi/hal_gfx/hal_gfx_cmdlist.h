/**
 ****************************************************************************************
 *
 * @file    hal_gfx_cmdlist.h
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

/** @defgroup HAL_GFX_CMDLIST GFX CMDLIST
  * @brief GPU management interfaces of command list.
  * @{
  */

#ifndef HAL_GFX_CMDLIST_H__
#define HAL_GFX_CMDLIST_H__

#include "hal_gfx_sys_defs.h"
#include "hal_gfx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GFX_CMDLIST_MACRO Defines
 * @{
 */
#define CL_NOP              0x010000U    /**< No operation. */
#define CL_PUSH             0x020000U    /**< Push command to currently command list. */
#define CL_RETURN           0x040000U    /**< Return from current command list. */
#define CL_ABORT            0x080000U    /**< Abort current command list. */
#define CL_BATCH_SHIFT      12           /**< TODO. */
#define CL_BATCH_LOOP       0x8000       /**< TODO. */
#define SUBMISSION_ID_MASK  0xffffff     /**< Mask. */
/** @} */

/**
 * @defgroup HAL_GFX_CMDLIST_STRUCT Structures
 * @{
 */
/**@brief Command list structure. */
typedef struct hal_gfx_cmdlist_t_
{
    hal_gfx_buffer_t bo;               /**< Buffer of command list*/
    int size;                          /**< Number of entries in the command list */
    int offset;                        /**< Points to the next address to write */
    uint32_t flags;                    /**< Flags */
    int32_t  submission_id;            /**< Command list id. */
    struct hal_gfx_cmdlist_t_ *next;   /**< Points to next command list */
    struct hal_gfx_cmdlist_t_ *root;   /**< Points to the head of the list */
} hal_gfx_cmdlist_t;
/** @} */

/**
 * @defgroup HAL_GFX_CMDLIST_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Create a new Command List into a preallocated space
 *
 * @param[in] bo:  Command List buffer (preallocated)
 *
 * @return The instance of the new Command List
 *****************************************************************************************
 */
hal_gfx_cmdlist_t hal_gfx_cl_create_prealloc(hal_gfx_buffer_t *bo);

/**
 *****************************************************************************************
 * @brief Create a new, non expandable Command List of specific size
 *
 * @param[in] size_bytes: Command List's size in bytes
 *
 * @return instance of the new Command List
 *****************************************************************************************
 */
hal_gfx_cmdlist_t hal_gfx_cl_create_sized(int size_bytes);

/**
 *****************************************************************************************
 * @brief Create a new expandable Command List
 *
 * @return instance of the new Command List
 *****************************************************************************************
 */
hal_gfx_cmdlist_t hal_gfx_cl_create(void);


/**
 *****************************************************************************************
 * @brief Create a new expandable Command List with power management mode
 *
 * @return instance of the new Command List
 *****************************************************************************************
 */
hal_gfx_cmdlist_t hal_gfx_cl_le_create(void);

/**
 *****************************************************************************************
 * @brief Destroy/Free a Command List
 *
 * @param[in] cl: Pointer to the Command List
 *****************************************************************************************
 */
void hal_gfx_cl_destroy(hal_gfx_cmdlist_t *cl);


/**
 *****************************************************************************************
 * @brief Destroy/Free a Command List with power management mode
 *
 * @param[in] cl: Pointer to the Command List
 *****************************************************************************************
 */
void hal_gfx_cl_le_destroy(hal_gfx_cmdlist_t *cl);

/**
 *****************************************************************************************
 * @brief Reset position of next command to be written to the beginning. Doesn't clear the List's contents.
 *
 * @param[in] cl: Pointer to the Command List
 *****************************************************************************************
 */
void hal_gfx_cl_rewind(hal_gfx_cmdlist_t *cl);

/**
 *****************************************************************************************
 * @brief Define in which Command List each subsequent commands are going to be inserted.
 *
 * @param[in] cl: Pointer to the Command List
 *****************************************************************************************
 */
void hal_gfx_cl_bind(hal_gfx_cmdlist_t *cl);

/**
 *****************************************************************************************
 * @brief Define in which Command List each subsequent commands are going to be inserted.
 * Bind this command list as Circular. It never gets full, it never expands,
 * it may get implicitly submitted, it cannot be reused.
 *
 * @param[in] cl: Pointer to the Command List
 *****************************************************************************************
 */
void hal_gfx_cl_bind_circular(hal_gfx_cmdlist_t *cl);

/**
 *****************************************************************************************
 * @brief  Unbind current bound Command List, if any.
 * @return None
 *****************************************************************************************
 */
void hal_gfx_cl_unbind(void);

/**
 *****************************************************************************************
 * @brief Get bound Command List
 *
 * @return Pointer to the bound Command List
 *****************************************************************************************
 */
hal_gfx_cmdlist_t *hal_gfx_cl_get_bound(void);

/**
 *****************************************************************************************
 * @brief Push command to command list, but do not trigger interrupt.
 *
 * @param[in] cl: command list
 *****************************************************************************************
 */
void hal_gfx_cl_submit_no_irq(hal_gfx_cmdlist_t *cl);

/**
 *****************************************************************************************
 * @brief Enqueue Command List to the Ring Buffer for execution
 *
 * @param[in] cl: Pointer to the Command List
 *****************************************************************************************
 */
void hal_gfx_cl_submit(hal_gfx_cmdlist_t *cl);

/**
 *****************************************************************************************
 * @brief Wait for Command List to finish
 *
 * @param[in] cl: Pointer to the Command List
 *
 * @return 0 if no error has occurred
 *****************************************************************************************
 */
int hal_gfx_cl_wait(hal_gfx_cmdlist_t *cl);

/**
 *****************************************************************************************
 * @brief Add a command to the bound Command List
 *
 * @param[in] reg:  Hardware register to be written
 * @param[in] data: Data to be written
 *****************************************************************************************
 */
void hal_gfx_cl_add_cmd(uint32_t reg, uint32_t data);

/**
 *****************************************************************************************
 * @brief Add multiple commands to the bound Command List
 *
 * @param[in] cmd_no: Numbers of commands to add
 * @param[in] cmd:    Pointer to the commands to be added
 *
 * @return 0 if no error has occurred
 *****************************************************************************************
 */
int hal_gfx_cl_add_multiple_cmds(int cmd_no, uint32_t *cmd);

/**
 *****************************************************************************************
 * @brief Request free space from command list.
 *
 * @param[in] cmd_no: Number of commands to write
 *
 * @return Return pointer in bound_cl to directly add commands
 *****************************************************************************************
 */
uint32_t * hal_gfx_cl_get_space(int cmd_no);

/**
 *****************************************************************************************
 * @brief Branch from the bound Command List to a different one. Return is implied.
 *
 * @param[in] cl: Pointer to the Command List to branch to
 *****************************************************************************************
 */
void hal_gfx_cl_branch(hal_gfx_cmdlist_t *cl);

/**
 *****************************************************************************************
 * @brief Jump from the bound Command List to a different one. No return is implied.
 *
 * @param[in] cl: Pointer to the Command List to jump to
 *****************************************************************************************
 */
void hal_gfx_cl_jump(hal_gfx_cmdlist_t *cl);

/**
 *****************************************************************************************
 * @brief Add an explicit return command to the bound Command List
 *****************************************************************************************
 */
void hal_gfx_cl_return(void);

/**
 *****************************************************************************************
 * @brief Returns positive number if the Command List is almost full, otherwise returns 0.
 *
 * @param[in] cl: Pointer to the Command List
 *****************************************************************************************
 */
int hal_gfx_cl_almost_full(hal_gfx_cmdlist_t *cl);

/**
 *****************************************************************************************
 * @brief Check if there is enough space or expansion can be performed for
 * required commands.
 *
 * @param[in] cmd_no: Numbers of commands to be checked if they fit
 *
 * @return  zero is commands fit or expansion xan be performed else return negative
 *****************************************************************************************
 */
int hal_gfx_cl_enough_space(int cmd_no);
/** @} */
#ifdef __cplusplus
}
#endif

#endif
/** @} */
/** @} */
/** @} */



#ifndef HAL_GFX_RINGBUFFER_H__
#define HAL_GFX_RINGBUFFER_H__

#include "hal_gfx_sys_defs.h"
#include "hal_gfx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------
//@function hal_gfx_rb_submit_cmdlist
//@brief Enqueue a Command List to the Ring Buffer for execution
//@param hal_gfx_buffer_t *bo desc: Pointer to the buffer object of the Command List
//@param uint32_t size desc: Size of the populated Command List
//@return int desc: Return submission id
//--------------------------------------------------
/** \private */
int32_t hal_gfx_rb_submit_cmdlist(hal_gfx_buffer_t *bo, int size);

int32_t hal_gfx_rb_submit_cmdlist2(uintptr_t base_phys, int size);

//--------------------------------------------------
//@function hal_gfx_rb_inline_cmd
//@brief Enqueue a Command to the Ring Buffer for execution
//@param uint32_t reg desc: Hardware Register to be written
//@param uint32_t data desc: Data to be written
//--------------------------------------------------
/** \private */
void hal_gfx_rb_inline_cmd(uint32_t reg, uint32_t data);

/** \private */
void hal_gfx_rb_force_flush(void);

/** \private */
void hal_gfx_rb_submit_cl_id(int cl_id);

#ifdef __cplusplus
}
#endif

#endif

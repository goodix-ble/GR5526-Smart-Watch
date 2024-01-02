
#ifndef HAL_GFX_ERROR_INTERN_H__
#define HAL_GFX_ERROR_INTERN_H__

#include "hal_gfx_sys_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

// -------------------------- ERROR HANDLING  -----------------------------------

/** \brief Set Error Code
 *
 * \param error    Error id
 *
 */
void hal_gfx_set_error(uint32_t error);

/** \brief Reset Error Code
 *
 *
 */
void hal_gfx_reset_error(void);


#ifdef __cplusplus
}
#endif

#endif // HAL_GFX_ERROR_INTERN_H__

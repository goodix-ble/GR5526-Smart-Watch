
/** @addtogroup GRAPHICS_SDK Graphics
 *  @{
 */

/** @defgroup HAL_GFX_RASTERIZER Hal gfx rasterizer TODO
  * @brief TODO.
  * @{
  */

#ifndef HAL_GFX_RASTERIZER_H__
#define HAL_GFX_RASTERIZER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_gfx_sys_defs.h"
#include "hal_gfx_rasterizer_intern.h"

/**
 * @defgroup HAL_GFX_RASTERIZER_ENUM Enumerations
 * @{
 */
 /**@brief TODO. */
typedef enum 
{
    VAR_INFO_TYPE_COLOR   = 0,  /**< Color */
    VAR_INFO_TYPE_TEXTURE = 1,  /**< Texture */
    VAR_INFO_TYPE_DEPTH   = 2   /**< Depth */
} var_info_type_e;
/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */


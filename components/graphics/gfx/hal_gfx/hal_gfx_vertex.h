
/** @addtogroup GRAPHICS_SDK Graphics
 *  @{
 */

/** @defgroup HAL_GFX_VERTEX Hal gfx vertex TODO
 * @brief TODO.
 * @{
 */

#ifndef HAL_GFX_VERTEX_H__
#define HAL_GFX_VERTEX_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GFX_VERTEX_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] addr_phys: TODO
 *****************************************************************************************
 */
void hal_gfx_bind_vertex_shader(uint32_t addr_phys);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] addr_phys: TODO
 *****************************************************************************************
 */
void hal_gfx_bind_vertex_assembler(uint32_t addr_phys);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] param: TODO
 *****************************************************************************************
 */
void hal_gfx_vertex_start(uint32_t param);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] reg: TODO
 * @param[in] value: TODO
 *****************************************************************************************
 */
void hal_gfx_vertex_set_reg(int reg, uint32_t value);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] reg: TODO
 * @param[in] value: TODO
 *****************************************************************************************
 */
void hal_gfx_vertex_set_const_reg(int reg, uint32_t value);
/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */


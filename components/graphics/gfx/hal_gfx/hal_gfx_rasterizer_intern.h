
/** @addtogroup GRAPHICS_SDK Graphics
 *  @{
 */

/** @defgroup HAL_GFX_RASTERIZER_INTERN Hal gfx rasterizer intern TODO
  * @brief TODO.
  * @{
  */

#ifndef HAL_GFX_RASTERIZER_INTERN_H__
#define HAL_GFX_RASTERIZER_INTERN_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HAL_GFX_RASTERIZER_INTERN_MACRO Defines
 * @{
 */
#define DRAW_LINE         (0x01U)   /**< TODO. */
#define DRAW_BOX          (0x02U)   /**< TODO. */
#define DRAW_TRIANGLE     (0x04U)   /**< TODO. */
#define DRAW_QUAD         (0x05U)   /**< TODO. */
#define GL_DRAW_LINE      (0x21U)   /**< TODO. */
#define GL_DRAW_BOX       (0x22U)   /**< TODO. */
#define GL_DRAW_PIXEL     (0x23U)   /**< TODO. */
#define GL_DRAW_TRIANGLE  (0x24U)   /**< TODO. */
#define GL_DRAW_QUAD      (0x25U)   /**< TODO. */

#define RAST_AA_E3          (0x00800000U)                                       /**< (1U<<23). */
#define RAST_AA_E2          (0x01000000U)                                       /**< (1U<<24). */
#define RAST_AA_E1          (0x02000000U)                                       /**< (1U<<25). */
#define RAST_AA_E0          (0x04000000U)                                       /**< (1U<<26). */
#define RAST_AA_MASK        (RAST_AA_E0 | RAST_AA_E1 | RAST_AA_E2 | RAST_AA_E3) /**< TODO. */
#define RAST_GRAD           (0x08000000U)                                       /**< (1U<<27). */
#define RAST_SETUP_CULL_CW  (0x10000000U)                                       /**< (1U<<28). */
#define RAST_SETUP_CULL_CCW (0x20000000U)                                       /**< (1U<<29). */
#define RAST_TILE           (0x40000000U)                                       /**< (1U<<30). */
/** @} */

/**
 * @defgroup HAL_GFX_FONT_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Enables MSAA per edge
 *
 * @param[in] aa: A combination of the flags RAST_AA_E0, RAST_AA_E1, RAST_AA_E2, RAST_AA_E3
 *
 * @return previous AA flags (may be ignored)
 *****************************************************************************************
 */
uint32_t hal_gfx_enable_aa_flags(uint32_t aa);
/** @} */

#ifdef __cplusplus
}
#endif

#endif //HAL_GFX_RASTERIZER_INTERN_H__

/** @} */
/** @} */


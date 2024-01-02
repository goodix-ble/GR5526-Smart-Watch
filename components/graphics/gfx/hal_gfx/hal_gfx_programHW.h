
/** @addtogroup GRAPHICS_SDK Graphics
 *  @{
 */

/** @defgroup HAL_GFX_PROGRAM_HW Hal gfx program hw TODO
  * @brief TODO.
  * @{
  */

#ifndef HAL_GFX_PROGRAM_HW_H__
#define HAL_GFX_PROGRAM_HW_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_gfx_sys_defs.h"
#include "hal_gfx_matrix3x3.h"

/**
 * @defgroup HAL_GFX_PROGRAM_HW_STRUCT Structures
 * @{
 */
/**@brief TODO. */
typedef struct 
{
    uint32_t      base;     /**< Base address as seen by the GPU. */
    int32_t       w;        /**< Width. */
    int32_t       h;        /**< Height. */
    uint32_t      fstride;  /**< Format and Stride. */
    int32_t       valid;    /**< 1 if valid. */
} tex_t;

/**@brief TODO. */
typedef struct hal_gfx_context_t_
{
    unsigned char en_tscFB;     /**< TODO. */
    unsigned char en_ZCompr;    /**< TODO. */
    unsigned char en_sw_depth;  /**< TODO. */
    uint32_t surface_tile;      /**< TODO. */
    uint32_t tri_cul;           /**< TODO. */
    uint32_t color_grad;        /**< TODO. */
    uint32_t draw_flags;        /**< TODO. */
    uint32_t aa;                /**< TODO. */
    uint32_t hal_gfx_error;     /**< TODO. */
    int16_t  breakpoint;        /**< TODO. */
    tex_t texs[8];              /**< TODO. */
} hal_gfx_context_t;

extern TLS_VAR hal_gfx_context_t hal_gfx_context;

/**@brief TODO. */
typedef union 
{
    float    f; /**< TODO. */
    uint32_t u; /**< TODO. */
    int32_t  i; /**< TODO. */
} hal_gfx_multi_union_t;
/** @} */

/**
 * @defgroup HAL_GFX_PROGRAM_HW_MACRO Defines
 * @{
 */
#define IS_HAL_GFX_T      ( (hal_gfx_readHwConfig() & (0x80000U)) != 0U )               /**< TODO. */

#define YX16TOREG32(y, x) ( (((unsigned)(y)) << 16) | (((unsigned)(x)) & 0xffffU))   /**< TODO. */

// MatMult
//-----------------------------------------------------------------------------------------------------------------------
#define MMUL_QUAD_BEZ   (0x02000000U) /**< (1U<<25). */
#define MMUL_BYPASS     (0x10000000U) /**< (1U<<28). */
#define MMUL_DONTPLUS05 (0x20000000U) /**< (1U<<29). */
#define MMUL_NONPERSP   (0x80000000U) /**< (1U<<31). */

// Z-Function
//-----------------------------------------------------------------------------------------------------------------------
#define HAL_GFX_Z_COMPARE_OP_NEVER         (0x0U)   /**< TODO. */
#define HAL_GFX_Z_COMPARE_OP_LESS          (0x1U)   /**< TODO. */
#define HAL_GFX_Z_COMPARE_OP_EQUAL         (0x2U)   /**< TODO. */
#define HAL_GFX_Z_COMPARE_OP_LESS_EQUAL    (0x3U)   /**< TODO. */
#define HAL_GFX_Z_COMPARE_OP_GREATER       (0x4U)   /**< TODO. */
#define HAL_GFX_Z_COMPARE_OP_NOT_EQUAL     (0x5U)   /**< TODO. */
#define HAL_GFX_Z_COMPARE_OP_GREATER_EQUAL (0x6U)   /**< TODO. */
#define HAL_GFX_Z_COMPARE_OP_ALWAYS        (0x7U)   /**< TODO. */

// Z-Control
//-----------------------------------------------------------------------------------------------------------------------
#define Z_LATE       (0x8U )    /**< TODO. */
#define Z_EARLY      (0x10U)    /**< TODO. */
#define Z_BOTH       (0x18U)    /**< TODO. */

#define BYPASS_COLOR         ( 1U)  /**< TODO. */
#define BYPASS_CODEPTR_FG    ( 2U)  /**< TODO. */
#define BYPASS_CODEPTR_BG    ( 3U)  /**< TODO. */
#define BYPASS_CLIP_MinX     ( 4U)  /**< TODO. */
#define BYPASS_CLIP_MinY     ( 5U)  /**< TODO. */
#define BYPASS_CLIP_MaxX     ( 6U)  /**< TODO. */
#define BYPASS_CLIP_MaxY     ( 7U)  /**< TODO. */
#define BYPASS_PT0_X         ( 8U)  /**< TODO. */
#define BYPASS_PT0_Y         ( 9U)  /**< TODO. */
#define BYPASS_PT1_X         (10U)  /**< TODO. */
#define BYPASS_PT1_Y         (11U)  /**< TODO. */
#define BYPASS_DEPTH_START_L (12U)  /**< TODO. */
#define BYPASS_DEPTH_START_H (13U)  /**< TODO. */
#define BYPASS_DEPTH_DX_L    (14U)  /**< TODO. */
#define BYPASS_DEPTH_DX_H    (15U)  /**< TODO. */
#define BYPASS_DEPTH_DY_L    (16U)  /**< TODO. */
#define BYPASS_DEPTH_DY_H    (17U)  /**< TODO. */
#define BYPASS_E0S           (18U)  /**< TODO. */
#define BYPASS_E0dx          (19U)  /**< TODO. */
#define BYPASS_E0dy          (20U)  /**< TODO. */
#define BYPASS_E1S           (21U)  /**< TODO. */
#define BYPASS_E1dx          (22U)  /**< TODO. */
#define BYPASS_E1dy          (23U)  /**< TODO. */
#define BYPASS_E2S           (24U)  /**< TODO. */
#define BYPASS_E2dx          (25U)  /**< TODO. */
#define BYPASS_E2dy          (26U)  /**< TODO. */
#define BYPASS_E3S           (27U)  /**< TODO. */
#define BYPASS_E3dx          (28U)  /**< TODO. */
#define BYPASS_E3dy          (29U)  /**< TODO. */
#define BYPASS_NEG_AREA      (30U)  /**< TODO. */
#define BYPASS_CMD           (31U)  /**< TODO. */

#define HAL_GFX_CONF_MASK_AXIM        (0x80000000U) /**< (0x1U  <<31). */
#define HAL_GFX_CONF_MASK_TEXFILTER   (0x40000000U) /**< (0x1U  <<30). */
#define HAL_GFX_CONF_MASK_TSC6        (0x20000000U) /**< (0x1U  <<29). */
#define HAL_GFX_CONF_MASK_ROP_BLENDER (0x10000000U) /**< (0x1U  <<28). */
#define HAL_GFX_CONF_MASK_ASYNC       (0x08000000U) /**< (0x1U  <<27). */
#define HAL_GFX_CONF_MASK_DIRTY       (0x04000000U) /**< (0x1U  <<26). */
#define HAL_GFX_CONF_MASK_TYPES       (0x03C00000U) /**< (0xfU  <<22). */
#define HAL_GFX_CONF_MASK_MMU         (0x00200000U) /**< (0x1U  <<21). */
#define HAL_GFX_CONF_MASK_ZCOMPR      (0x00100000U) /**< (0x1U  <<20). */
#define HAL_GFX_CONF_MASK_VRX         (0x00080000U) /**< (0x1U  <<19). */
#define HAL_GFX_CONF_MASK_ZBUF        (0x00040000U) /**< (0x1U  <<18). */
#define HAL_GFX_CONF_MASK_TSC         (0x00020000U) /**< (0x1U  <<17). */
#define HAL_GFX_CONF_MASK_CG          (0x00010000U) /**< (0x1U  <<16). */
#define HAL_GFX_CONF_MASK_VG          (0x00008000U) /**< (0x1U  <<15). */
#define HAL_GFX_CONF_MASK_CORES       (0x00000F00U) /**< (0xfU  <<8 ). */
#define HAL_GFX_CONF_MASK_THREADS     (0x000000FFU) /**< (0xffU <<0 ). */

#define HAL_GFX_CONF_MASK_AA          (0x00000001U) /**< (0x1U <<0). */
#define HAL_GFX_CONF_MASK_DEC         (0x00000002U) /**< (0x1U <<1). */
#define HAL_GFX_CONF_MASK_10BIT       (0x00000004U) /**< (0x1U <<1). */

#define HAL_GFX_RASTER_LINE_SIZE         (3U)   /**< TODO. */
#define HAL_GFX_RASTER_TRIANGLE_FX_SIZE  (7U)   /**< TODO. */
#define HAL_GFX_RASTER_QUAD_FX_SIZE      (9U)   /**< TODO. */
#define HAL_GFX_RASTER_RECT_SIZE         (3U)   /**< TODO. */
#define HAL_GFX_RASTER_PIXEL_SIZE        (3U)   /**< TODO. */
/** @} */


/**
 * @defgroup HAL_GFX_PROGRAM_HW_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief TODO
 *
 *****************************************************************************************
 */
uint32_t hal_gfx_readHwConfig(void);

/**
 *****************************************************************************************
 * @brief TODO
 *
 *****************************************************************************************
 */
uint32_t hal_gfx_readHwConfigH(void);

// -------------------------------- LOADCTRL -----------------------------------
/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] val: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_setLoadCtrlReg(uint32_t val);

// ------------------------------- MATMULT -------------------------------------
/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] enable: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_matmul_bypass(int enable);

/**
 *****************************************************************************************
 * @brief Load GPU's Matrix Multiplier with a given 3x3 matrix
 *
 * @param[in] m: Matrix to be loaded
 *
 *****************************************************************************************
 */
void hal_gfx_set_matrix(hal_gfx_matrix3x3_t m);

/**
 *****************************************************************************************
 * @brief Load GPU's Matrix Multiplier for scaling
 *
 * @param[in] dst_x:     X coordinate of upper-left vertex of the destination
 * @param[in] dst_y:     Y coordinate of upper-left vertex of the destination
 * @param[in] dst_xres:  Width of destination rectangular area
 * @param[in] dst_yres:  Height of destination rectangular area
 * @param[in] src_x:     X coordinate of upper-left vertex of the source
 * @param[in] src_y:     Y coordinate of upper-left vertex of the source
 * @param[in] src_xres:  Width of source rectangular area
 * @param[in] src_yres:  Height of source rectangular area
 *
 *****************************************************************************************
 */
void hal_gfx_set_matrix_scale( float dst_x, float dst_y, float dst_xres, float dst_yres,
                            float src_x, float src_y, float src_xres, float src_yres);

/**
 *****************************************************************************************
 * @brief Load GPU's Matrix Multiplier for a simple Blit (affine translation)
 *
 * @param[in] dst_x: X coordinate of upper-left vertex of the destination
 * @param[in] dst_y: Y coordinate of upper-left vertex of the destination
 *
 *****************************************************************************************
 */
void hal_gfx_set_matrix_translate(float dst_x, float dst_y);

// ------------------------------- BLENDER ------------------------------------
/**
 *****************************************************************************************
 * @brief Load a precompiled Shader to the GPU's internal memory
 *
 * @param[in] cmd:     Pointer to the shader
 * @param[in] count:   Number of commands
 * @param[in] codeptr: Internal Memory address to be written (default is 0)
 *
 *****************************************************************************************
 */
void hal_gfx_load_frag_shader(const uint32_t *cmd, uint32_t count, uint32_t codeptr);
/**
 *****************************************************************************************
 * @brief Set the Internal Memory address of the fragment shader to be executed
 *
 * @param[in] ptr: Internal Memory address of the fragment shader
 *
 *****************************************************************************************
 */
void hal_gfx_set_frag_ptr(uint32_t ptr);

/**
 *****************************************************************************************
 * @brief Load a precompiled Shader to the GPU's internal memory and set fragment pointer
 *
 * @param[in] cmd:     Pointer to the shader
 * @param[in] count:   Number of commands
 * @param[in] codeptr: Internal Memory address to be written (default is 0)
 * @param[in] ptr:     Internal Memory address of the fragment shader
 *
 *****************************************************************************************
 */
void hal_gfx_load_frag_shader_ptr(const uint32_t *cmd, uint32_t count, uint32_t codeptr, uint32_t ptr);

// ------------------------------- ROP BLENDER ------------------------------------
/**
 *****************************************************************************************
 * @brief Set ROP blending mode
 *
 * @param[in] bl_mode: Blending mode
 *
 *****************************************************************************************
 */
void hal_gfx_set_rop_blend_mode(uint32_t bl_mode);

/**
 *****************************************************************************************
 * @brief Set ROP destination color key
 *
 * @param[in] rgba: Destination Color Key
 *
 *****************************************************************************************
 */
void hal_gfx_set_rop_dst_color_key(uint32_t rgba);

/**
 *****************************************************************************************
 * @brief Set ROP constant color
 *
 * @param[in] rgba: Constant color
 *
 *****************************************************************************************
 */
void hal_gfx_set_rop_const_color(uint32_t rgba);

// ------------------------------- Z-BUFFER ------------------------------------
/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] val: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_set_depth_ctrl(uint32_t val);

// ------------------------------ INTERRUPT ------------------------------------
/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] val: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_set_interrupt_ctrl(uint32_t val);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] val: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_set_interrupt_ctrl_imm(uint32_t val);

// -------------------------------- UTILS --------------------------------------
/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] enable: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_enable_tiling(uint32_t enable);

/**
 *****************************************************************************************
 * @brief Set maximum and minimum values for depth buffer. Available ony for Nema|T
 *
 * @param[in] min_depth: Minimum value
 * @param[in] max_depth: Maximum value
 *
 *****************************************************************************************
 */
void hal_gfx_set_depth_range(float min_depth, float max_depth);

/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] val: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_set_matmul_ctrl(uint32_t val);
/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] m: TODO
 *
 *****************************************************************************************
 */

void hal_gfx_set_matrix_all(hal_gfx_matrix3x3_t m);
/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] addr: TODO
 * @param[in] data: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_set_bypass_reg( uint32_t addr, uint32_t data );

// -------------------------- RASTERIZER (GRAD) --------------------------------
/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] r_init: TODO
 * @param[in] g_init: TODO
 * @param[in] b_init: TODO
 * @param[in] a_init: TODO
 * @param[in] r_dx:   TODO
 * @param[in] r_dy:   TODO
 * @param[in] g_dx:   TODO
 * @param[in] g_dy:   TODO
 * @param[in] b_dx:   TODO
 * @param[in] b_dy:   TODO
 * @param[in] a_dx:   TODO
 * @param[in] a_dy:   TODO
 *
 *****************************************************************************************
 */
void hal_gfx_set_gradient_fx(int32_t r_init, int32_t g_init, int32_t b_init,
                          int32_t a_init, int32_t r_dx,   int32_t r_dy,
                          int32_t g_dx,   int32_t g_dy,   int32_t b_dx,
                          int32_t b_dy,   int32_t a_dx,   int32_t a_dy);

// -------------------------- RASTERIZER (DEPTH) -------------------------------
/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] startl: TODO
 * @param[in] starth: TODO
 * @param[in] dxl:    TODO
 * @param[in] dxh:    TODO
 * @param[in] dyl:    TODO
 * @param[in] dyh:    TODO
 *
 *****************************************************************************************
 */
void hal_gfx_set_depth_imm(uint32_t startl, uint32_t starth,
                        uint32_t dxl   , uint32_t dxh,
                        uint32_t dyl   , uint32_t dyh);


/**
 *****************************************************************************************
 * @brief Convert a float value to Internal representation of half float and write it to a Constant Register
 *
 * @param[in] reg:   Constant Register to be written
 * @param[in] v:     Register's Index to  be written
 * @param[in] value: Value to be written
 *
 *****************************************************************************************
 */
void hal_gfx_set_const_reg_half(uint32_t reg, uint32_t v, float value);

/**
 *****************************************************************************************
 * @brief Convert a float value to Internal representation of float and write it to a Constant Register
 *
 * @param[in] reg:   Constant Register to be written
 * @param[in] v:     Register's Index to  be written
 * @param[in] value: Value to be written
 *
 *****************************************************************************************
 */
void hal_gfx_set_const_reg_single(uint32_t reg, uint32_t v, float value);

// ------------------------------- FORKING  -------------------------------------
/**
 *****************************************************************************************
 * @brief TODO
 *
 * @param[in] codeptr: TODO
 * @param[in] dataptr: TODO
 *
 *****************************************************************************************
 */
void hal_gfx_fork(uint32_t codeptr, uint32_t dataptr);
/** @} */


#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */


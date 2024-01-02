
#ifndef HAL_GFX_MMU_H__
#define HAL_GFX_MMU_H__

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------------- MMU ------------------------------------
//--------------------------------------------------
//@function hal_gfx_build_translation_table
//@brief Build translation table
//--------------------------------------------------
void hal_gfx_build_translation_table(void);

//--------------------------------------------------
//@function hal_gfx_print_mmu_results
//@brief Print mmu results
//--------------------------------------------------
void hal_gfx_print_mmu_results(void);

#ifdef __cplusplus
}
#endif

#endif

#ifndef __LV_THEME_SIMPLIFY_H__
#define __LV_THEME_SIMPLIFY_H__

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include "../../../core/lv_obj.h"

#if LV_GDX_PATCH_USE_THEME_SIMPLIFY

/**
 * @brief Initialize the theme.
 *
 * @param disp Pointer to display to attach the theme
 * @return Pointer to the reference this theme later.
 */
lv_theme_t *lv_theme_simplify_init(lv_disp_t *disp);

/**
 * @brief Check if the theme is initialized.
 *
 * @return true The them is initialized.
 * @return false The theme is not initialized.
 */
bool lv_theme_simplify_is_inited(void);

#endif // LV_GDX_PATCH_USE_THEME_SIMPLIFY

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __LV_THEME_SIMPLIFY_H__

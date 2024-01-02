#include "../../../lvgl.h"

#if LV_GDX_PATCH_USE_THEME_SIMPLIFY

#include "lv_theme_simplify.h"

typedef struct
{
    lv_style_t label;
    lv_style_t arc_rounded;
#if LV_GDX_PATCH_USE_WMS_TILEVIEW
    lv_style_t wms_tile;
#endif // LV_GDX_PATCH_USE_WMS_TILEVIEW
} theme_styles_t;

static void styles_init(void);
static void style_init_reset(lv_style_t *style);
static void theme_apply(lv_theme_t *th, lv_obj_t *obj);

static theme_styles_t s_styles;
static lv_theme_t s_theme;
static bool s_inited = false;

lv_theme_t *lv_theme_simplify_init(lv_disp_t *disp)
{
    s_theme.disp = disp;
    s_theme.font_small = LV_FONT_DEFAULT;
    s_theme.font_normal = LV_FONT_DEFAULT;
    s_theme.font_large = LV_FONT_DEFAULT;
    s_theme.apply_cb = theme_apply;

    styles_init();

    if (disp == NULL || lv_disp_get_theme(disp) == &s_theme)
    {
        lv_obj_report_style_change(NULL);
    }

    s_inited = true;
    return &s_theme;
}

bool lv_theme_simplify_is_inited(void)
{
    return s_inited;
}

static void styles_init(void)
{
    // Label
    style_init_reset(&s_styles.label);
    lv_style_set_text_color(&s_styles.label, lv_color_white());
    lv_style_set_bg_color(&s_styles.label, lv_color_black());

    // Arc with rounded head
    style_init_reset(&s_styles.arc_rounded);
    lv_style_set_arc_rounded(&s_styles.arc_rounded, true);

#if LV_GDX_PATCH_USE_WMS_TILEVIEW
    // WMS Tile
    style_init_reset(&s_styles.wms_tile);
    lv_style_set_width(&s_styles.wms_tile, LV_HOR_RES);
    lv_style_set_height(&s_styles.wms_tile, LV_VER_RES);
    lv_style_set_bg_color(&s_styles.wms_tile, lv_color_black());
    lv_style_set_bg_opa(&s_styles.wms_tile, LV_OPA_COVER);
#endif // LV_GDX_PATCH_USE_WMS_TILEVIEW
}

static void style_init_reset(lv_style_t *style)
{
    if (s_inited)
    {
        lv_style_reset(style);
    }
    else
    {
        lv_style_init(style);
    }
}

static void theme_apply(lv_theme_t *th, lv_obj_t *obj)
{
    if (lv_obj_check_type(obj, &lv_label_class))
    {
        // Label
        lv_obj_add_style(obj, &s_styles.label, LV_STATE_DEFAULT);
    }
    else if (lv_obj_check_type(obj, &lv_arc_class))
    {
        lv_obj_add_style(obj, &s_styles.arc_rounded, LV_PART_MAIN);
        lv_obj_add_style(obj, &s_styles.arc_rounded, LV_PART_INDICATOR);
    }
#if LV_GDX_PATCH_USE_WMS_TILEVIEW
    else if (lv_obj_check_type(obj->parent, &lv_wms_tileview_class))
    {
        // WMS Tile
        lv_obj_add_style(obj, &s_styles.wms_tile, LV_STATE_DEFAULT);
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(obj, DISP_HOR_RES, DISP_VER_RES);
    }
#endif // LV_GDX_PATCH_USE_WMS_TILEVIEW
}

#endif // LV_GDX_PATCH_USE_THEME_SIMPLIFY

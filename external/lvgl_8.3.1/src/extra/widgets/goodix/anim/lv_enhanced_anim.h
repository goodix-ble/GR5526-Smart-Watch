#ifndef __LV_ENHANCED_ANIM_H__
#define __LV_ENHANCED_ANIM_H__


#include "lv_conf.h"
#include "lv_disp.h"
#include "../tileview/lv_wms_surface_flinger.h"


/*
 * new_scr :  The Object screen will goto.
 * transit_effect : transform effect. support following options:
 *       LV_TRANS_EFFECT_CUBE
 *       LV_TRANS_EFFECT_INNERCUBE
 *       LV_TRANS_EFFECT_STACK
 *       LV_TRANS_EFFECT_FADE
 *       LV_TRANS_EFFECT_FADE_ZOOM
 *       LV_TRANS_EFFECT_SPIN_H_R2L
 *       LV_TRANS_EFFECT_SPIN_H_L2R
 */
void lv_scr_load_anim_enhance(lv_obj_t * new_scr, lv_transit_effect_e transit_effect, uint32_t time, uint32_t delay, bool auto_del);



#endif /* __LV_ENHANCED_ANIM_H__ */

#ifndef LV_PORT_DISP_H
#define LV_PORT_DISP_H

#ifdef __cplusplusW
extern "C" {
#endif

#include "lvgl.h"

#if (LV_COLOR_DEPTH != 32) || (GR552X_GPU_RENDER_SUPPORT == 0)
    #warning "GR5526 Only supported LV_COLOR_DEPTH=32 and GR552X_GPU_RENDER_SUPPORT=1 configuration"
#endif

void lv_port_disp_init(void);

void lv_port_disp_flush_transition_fps(lv_disp_drv_t * disp_drv);

void lv_port_disp_debug_enable(bool enable);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif


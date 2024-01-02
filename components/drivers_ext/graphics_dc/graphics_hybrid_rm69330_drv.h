#ifndef __GRAPHICS_HYBRID_RM69330_DRV_H__
#define __GRAPHICS_HYBRID_RM69330_DRV_H__

#include <stdint.h>
#include <stdbool.h>

void graphics_hybrid_rm69330_init(uint16_t screen_w, uint16_t screen_h, graphics_dc_mipi_format_e mipi_format);
void graphics_hybrid_rm69330_deinit(void);
void graphics_hybrid_rm69330_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);
void graphics_hybrid_rm69330_flush(void *buf, uint32_t buf_format, uint16_t w, uint16_t h);
void graphics_hybrid_rm69330_set_on(bool on);
void graphics_hybrid_rm69330_wait_ready(void);
void graphics_hybrid_rm69330_sleep(void);
void graphics_hybrid_rm69330_wakeup(void);


#endif // __GRAPHICS_HYBRID_RM69330_DRV_H__

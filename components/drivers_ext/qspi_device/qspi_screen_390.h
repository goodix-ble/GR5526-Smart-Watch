#ifndef __QSPI_SCREEN_390_H__
#define __QSPI_SCREEN_390_H__

#include "app_qspi.h"


#define RM69330_LCD_RES_390         0x01        /* Resolution : 390x390x16bit */

#define RM69330_LCD_RES_454         0x02        /* Resolution : 454x454x16bit */

typedef uint32_t    rm69330_lcd_res_e;          /* optional :
                                                       @ref RM69330_LCD_RES_390
                                                       @ref RM69330_LCD_RES_454
                                                 */

void qspi_screen_init_basic(app_qspi_id_t id, uint32_t clock_prescaler, rm69330_lcd_res_e res, app_qspi_evt_handler_t evt_handler);
void qspi_screen_init(app_qspi_id_t id, uint32_t clock_prescaler, app_qspi_evt_handler_t evt_handler);
void qspi_screen_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);
void qspi_screen_deinit(app_qspi_id_t id);

#endif /*__QSPI_SCREEN_390_H__*/

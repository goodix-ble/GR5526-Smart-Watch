#ifndef __QSPI_AMO139_DISPLAY_H__
#define __QSPI_AMO139_DISPLAY_H__

#include "app_qspi.h"

/***********************************************************************
 * LCD Model : FLS-AMO139WV334 454x54
 * IC  Model : SH8601A, 480RGBx480 / 16.7M color, AMOLED Display Driver IC
 ***********************************************************************
 */

void amo139_screen_init_basic(app_qspi_id_t id, uint32_t clock_prescaler, uint32_t res, app_qspi_evt_handler_t evt_handler);

void amo139_screen_set_show_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);

void amo139_screen_deinit(app_qspi_id_t id);

#endif /*__QSPI_AMO139_DISPLAY_H__*/

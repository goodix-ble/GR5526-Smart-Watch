#ifndef __TOUCHPAD_DRIVER_H__
#define __TOUCHPAD_DRIVER_H__

#include "stdint.h"
#include "stdbool.h"

#define TOUCHPAD_SCRN_TYPE_454P     1u

#define TOUCHPAD_SCRN_TYPE_360P     2u

/********************************************************************************************
 *                       Public Declarations
 ********************************************************************************************/
void    touchpad_dev_init(uint32_t scrn_type, void(* _irq_notify)(void));
void    touchpad_dev_deinit(void);
bool    touchpad_dev_read_reg(uint8_t reg_addr, uint8_t *buffer, uint16_t len);
bool    touchpad_dev_write_reg(uint8_t reg_addr, uint8_t value);
bool    touchpad_dev_read_pointer(int16_t * x, int16_t * y);
bool    touchpad_dev_sleep(void);
bool    touchpad_dev_wakeup(void);

#endif /* __TOUCHPAD_DRIVER_H__ */

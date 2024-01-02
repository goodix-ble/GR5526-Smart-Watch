#ifndef __QSPI_PSRAM_H__
#define __QSPI_PSRAM_H__

#include "app_qspi.h"

/************************************************************
*        PSRAM Type:  APS6404L‐SQRH SPI/QPI PSRAM
 ************************************************************/

#define PSRAM_CMD_READ                                  0x03
#define PSRAM_CMD_QUAD_SLOWREAD                         0x0B
#define PSRAM_CMD_QUAD_FASTREAD                         0xEB
#define PSRAM_CMD_WRITE                                 0x02
#define PSRAM_CMD_QUAD_WRITE                            0x38
#define PSRAM_CMD_ENTER_QUAD_MODE                       0x35
#define PSRAM_CMD_EXIT_QUAD_MODE                        0xF5
#define PSRAM_CMD_RESET_ENABLE                          0x66
#define PSRAM_CMD_RESET                                 0x99
#define PSRAM_CMD_HALF_SLEEP                            0xC0
#define PSRAM_CMD_READID                                0x9F

#define PSRAM_DEVICE_ID                                 0x0D5D


uint32_t     qspi_psram_init(app_qspi_id_t id, uint32_t clock_prescaler, qspi_pins_group_e pin_group, uint32_t clock_mode);
uint32_t     qspi_psram_init_as_xip(app_qspi_id_t id, uint32_t clock_prescaler, qspi_pins_group_e pin_group);
void         qspi_psram_deinit(app_qspi_id_t id);
uint32_t     qspi_psram_read_id(app_qspi_id_t id);
void         qspi_psram_reset(app_qspi_id_t id);
void         qspi_psram_enable_quad_mode(app_qspi_id_t id);
void         qspi_psram_exit_quad_mode(app_qspi_id_t id);

#endif /*__QSPI_PSRAM_H__*/

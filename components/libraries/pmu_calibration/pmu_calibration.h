#ifndef _PMU_CALIB_H
#define _PMU_CALIB_H

#include <stdio.h>
#include <stdint.h>

#include "custom_config.h"
#include "app_timer.h"

void system_pmu_calibration_stop(void);

void system_pmu_calibration_init(uint32_t interval);

#ifdef ENV_USE_FREERTOS
void system_pmu_calibration_start(void);
#endif

#endif

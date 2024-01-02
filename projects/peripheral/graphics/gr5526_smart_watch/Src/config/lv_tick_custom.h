#ifndef __LV_TICK_CUSTOM__
#define __LV_TICK_CUSTOM__

#include "lvgl.h"

#ifdef USE_OSAL
#include "osal.h"
#else
#include "FreeRTOS.h"
#include "task.h"
#endif

#endif

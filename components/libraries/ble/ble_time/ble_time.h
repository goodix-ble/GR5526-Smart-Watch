#ifndef __APP_BLE_TIME_H__
#define __APP_BLE_TIME_H__
#include "grx_hal.h"

/*
 * MACRO DEFINITIONS
 *****************************************************************************************
 */
#define TICK_MS_IN_HUS                      (2000)
#define HALF_SLOT_SIZE                      (625)
#define RWIP_MAX_CLOCK_TIME                 (0xFFFFFFF)
#define SECOND_IN_HUS                       (2000000)
#define CLK_SUB(clock_a, clock_b)           ((uint32_t)(((clock_a) - (clock_b)) & RWIP_MAX_CLOCK_TIME))

/*
 * STRUCTURE DEFINITIONS
 *****************************************************************************************
 */
typedef struct
{
    uint32_t hs;
    uint16_t hus;
} ble_time_t;

/**
 ****************************************************************************************
 * @brief  This function get the ble time.
 *
 * @note   This function is supported only when ble stack is initiated
 *
 ****************************************************************************************
 */
ble_time_t ble_time_get(void);

#endif

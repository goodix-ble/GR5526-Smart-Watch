/**************************************************************************//**
 * @file     system_gr55xx.h
 * @brief    CMSIS Cortex-M# Device Peripheral Access Layer Header File for
 *           Device GR55xx
 * @version  V1.00
 * @date     12. June 2018
 ******************************************************************************/
/*
 * Copyright (c) 2016-2018, Shenzhen Huiding Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __SYSTEM_GR55xx_H__
#define __SYSTEM_GR55xx_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CLK_96M        96000000
#define CLK_64M        64000000
#define CLK_48M        48000000
#define CLK_32M        32000000
#define CLK_24M        24000000
#define CLK_16M        16000000

typedef enum
{
    XO_S16M_CLK   = 2,
    CPLL_S16M_CLK = 5,
    CPLL_T24M_CLK = 4,
    CPLL_T32M_CLK = 6,
    CPLL_F48M_CLK = 3,
    CPLL_S64M_CLK = 1,
    CPLL_S96M_CLK = 0,
    CLK_TYPE_NUM  = 7,
} mcu_clock_type_t;

typedef enum
{
    QSPI_64M_CLK  = 0,
    QSPI_48M_CLK  = 1,
    QSPI_32M_CLK  = 2,
    QSPI_24M_CLK  = 3,
    QSPI_16M_CLK  = 4,
    QSPI_CLK_TYPE_NUM  = 5,
} flash_clock_type_t;

typedef enum
{
    BLE_RNG_OSC_CLK = 0,
    BLE_RTC_OSC_CLK,
    BLE_RNG_OSC_CLK2,
} ble_clock_type_t;


typedef enum
{
    SERIAL_N96M_CLK     = 0,
    SERIAL_S64M_CLK     = 1,
    SERIAL_T32M_CLK     = 2,
    SERIAL_S16M_CLK     = 3,
    SERIAL_XO_S16M_CLK  = 4,
    SERIAL_CLK_TYPE_NUM = 5,
} serial_clock_type_t;

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock) */

/*!< System Slow Clock Frequency for RTC, SLEEP_TIMER and AON_WDT, default value is 32768*/
extern uint32_t SystemSlowClock;

/** @brief Setup the microcontroller system.

    Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit(void);

/** \brief  Update SystemCoreClock variable.

    Updates the SystemCoreClock with current core Clock
    retrieved from cpu registers.
 */
extern void SystemCoreSetClock(mcu_clock_type_t clock);

/** \brief  Get SystemCoreClock variable.

    Get the SystemCoreClock with current core Clock
    retrieved from cpu registers.
 */
extern void SystemCoreGetClock(mcu_clock_type_t *clock);


extern void SystemCoreUpdateClock(void);

/** \brief  Set Serial clock variable.

    Set the Serial with current Clock
 */
extern void SetSerialClock(serial_clock_type_t clock_type);

/** \brief  Get Serial clock variable.

    Get the Serial with current Clock
    retrieved from cpu registers.
 */
extern uint32_t GetSerialClock(void);

/**
 ****************************************************************************************
 * @brief  Handle Clock calibration interrupt request.
 * @retval :  void
 ****************************************************************************************
 */
extern void clock_calibration_irq_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_GR55xx_H__ */

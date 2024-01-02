/**
 *****************************************************************************************
 *
 * @file user_simu_key_init.h
 *
 * @brief Header file - User Function
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

#ifndef _UART_SIMU_KEY_INIT_H_
#define _UART_SIMU_KEY_INIT_H_

/*
 * DEFINES
 *****************************************************************************************
 */
#define VIR_KEY_UP_ID               0x00      /**< ID for Virtual UP KEY. */
#define VIR_KEY_DOWN_ID             0x01      /**< ID for Virtual DOWN KEY. */
#define VIR_KEY_LEFT_ID             0x02      /**< ID for Virtual LEFT KEY. */
#define VIR_KEY_RIGHT_ID            0x03      /**< ID for Virtual RIGHT KEY. */
#define VIR_KEY_OK_ID               0x04      /**< ID for Virtual OK KEY. */

#define VIR_KEY_UP_CMD              "up"      /**< Command for Virtual UP KEY. */
#define VIR_KEY_DOWN_CMD            "down"    /**< Command for Virtual DOWN KEY. */
#define VIR_KEY_LEFT_CMD            "left"    /**< Command for Virtual LEFT KEY. */
#define VIR_KEY_RIGHT_CMD           "right"   /**< Command for Virtual RIGHT KEY. */
#define VIR_KEY_OK_CMD              "ok"      /**< Command for Virtual OK KEY. */
#define VIR_KEY_DOUBLE_PRESS        "+"       /**< Double press click type. */
#define VIR_KEY_LONG_PRESS          "++"      /**< Long press click type. */
#define VIR_KEY_CONTINUE_PRESS      "-"       /**< Continue press click type. */
#define VIR_KEY_CONTINUE_RELEASE    "--"      /**< Continue release click type. */

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Function for ble stack init complete 
 *****************************************************************************************
 */
void uart_simu_key_init(void);

/** @} */

#endif


/**
 ****************************************************************************************
 *
 * @file    gr55xx_sim_card.h
 * @author  BLE Driver Team
 * @brief   sim card driver.
 *
 ****************************************************************************************
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
 ****************************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SIM_CARD_H
#define __SIM_CARD_H

/* Includes ------------------------------------------------------------------*/
#include "grx_hal.h"
#if APP_DRIVER_USE_ENABLE
#include "app_iso7816.h"
#endif

/* Exported constants --------------------------------------------------------*/
#define ATR_BUFFER_LEN   33    /* ATR buffer length */
#define T0_PROTOCOL      0x00  /* T0 protocol */
#define DIRECT           0x3B  /* Direct bit convention */
#define INDIRECT         0x3F  /* Indirect bit convention */
#define SETUP_LENGTH     20    /* Setup array length */
#define HIST_LENGTH      20    /* Historical array length*/
#define LC_MAX           20    /* Command parameters max length */
#define SC_TIMEOUT       200   /* Transimit and reseive timeout(ms) */

/* SIM Card ADPU Command: Operation Code -------------------------------------------*/
#define SC_CLA_GSM11     0xA0

/*-------------------- SIM Card Data Area Management Commands ---------------------*/
#define SC_SELECT_FILE   0xA4
#define SC_GET_RESPONCE  0xC0
#define SC_STATUS        0xF2
#define SC_UPDATE_BINARY 0xD6
#define SC_READ_BINARY   0xB0
#define SC_WRITE_BINARY  0xD0
#define SC_UPDATE_RECORD 0xDC
#define SC_READ_RECORD   0xB2

/*-------------------------- Administrative Commands -------------------------*/
#define SC_CREATE_FILE   0xE0

/*-------------------------- Safety Management Commands ----------------------*/
#define SC_VERIFY        0x20
#define SC_CHANGE        0x24
#define SC_DISABLE       0x26
#define SC_ENABLE        0x28
#define SC_UNBLOCK       0x2C
#define SC_EXTERNAL_AUTH 0x82
#define SC_GET_CHALLENGE 0x84

/* SC STATUS: Status Code ----------------------------------------------------*/
#define SC_EF_SELECTED   0x9F
#define SC_DF_SELECTED   0x9F
#define SC_OP_TERMINATED 0x9000

/* Exported types ------------------------------------------------------------*/

/* ATR structure - Answer To Reset -------------------------------------------*/
typedef struct
{
  uint8_t TS;              /* Bit Convention */
  uint8_t T0;              /* High nibble = Number of setup byte; low nibble = Number of historical byte */
  uint8_t T[SETUP_LENGTH]; /* Setup array */
  uint8_t H[HIST_LENGTH];  /* Historical array */
  uint8_t Tlength;         /* Setup array dimension */
  uint8_t Hlength;         /* Historical array dimension */
} ATR_t;

/* ADPU-Header command structure ---------------------------------------------*/
typedef struct
{
  uint8_t CLA; /* Command class */
  uint8_t INS; /* Operation code */
  uint8_t P1;  /* Selection Mode */
  uint8_t P2;  /* Selection Option */
} cmd_header_t;

/* ADPU-Body command structure -----------------------------------------------*/
typedef struct
{
  uint8_t LC;           /* Data field length */
  uint8_t Data[LC_MAX]; /* Command parameters */
  uint8_t LE;           /* Expected length of data to be returned */
} cmd_body_t;

/* ADPU Command structure ----------------------------------------------------*/
typedef struct
{
  cmd_header_t Header;
  cmd_body_t Body;
} ADPU_cmd_t;

/* SC response structure -----------------------------------------------------*/
typedef struct
{
  uint8_t Data[LC_MAX]; /* Data returned from the card */
  uint8_t SW1;          /* Command Processing status */
  uint8_t SW2;          /* Command Processing qualification */
} ADPU_response_t;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void sim_card_init(iso7816_handle_t *sim_card_handle);
void sim_card_get_ATR(iso7816_handle_t *sim_card_handle);
void sim_card_PTS(iso7816_handle_t *sim_card_handle);
void sim_card_send_cmd_get_response(iso7816_handle_t *sim_card_handle, ADPU_cmd_t *sim_ADPU, ADPU_response_t *APDU_respon);

void sim_card_select_MF(iso7816_handle_t *sim_card_handle);
void sim_card_read_ICCID(iso7816_handle_t *sim_card_handle);
void sim_card_read_IMSI(iso7816_handle_t *sim_card_handle);
#endif /* __SIM_CARD_H */

/************************ (C) COPYRIGHT GOODIX *****END OF FILE****/

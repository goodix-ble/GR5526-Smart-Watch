/**
 ****************************************************************************************
 *
 * @file    gr55xx_sim_card.c
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of sim card control.
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

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_sim_card.h"

#if !APP_DRIVER_USE_ENABLE
/* sim card command and response data buffer */
static __ALIGNED(4) uint8_t iso7816_buffer[ATR_BUFFER_LEN];
#endif

/* Answer To Reset (ATR) Structures */
ATR_t sim_ATR;

/* APDU Transport Structures */
ADPU_cmd_t sim_ADPU;
ADPU_response_t APDU_response;

/* F and D tabel which is used to calculate edudiv */
static uint32_t F_Table[16] = {372U, 372U, 558U, 744U, 1116U, 1488U, 1860U, 372U, 372U, 512U, 768U, 1024U, 1536U, 2048U, 372U, 372U};
static uint32_t D_Table[16] = {1U, 1U, 2U, 4U, 8U, 16U, 32U, 64U, 12U, 20U, 1U, 1U, 1U, 1U, 1U, 1U};

/* sim card notation (reference GSM11.11) */
const uint8_t MasterRoot[2] = {0x3F, 0x00};
const uint8_t GSMDir[2] = {0x7F, 0x20};
const uint8_t ICCID[2] = {0x2F, 0xE2};
const uint8_t IMSI[2] = {0x6F, 0x07};

const uint8_t CHV1[8] = {'0', '0', '0', '0', '0', '0', '0', '0'};
uint32_t CHV1_status = 0;
uint8_t ICCID_content[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t IMSI_content[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/**
  * @brief  Decodes the Answer to reset received from card.
  * @param  ATR: pointer to the buffer containing the ATR.
  * @retval sim card communication protocol
  */
static uint8_t sim_card_decode_ATR(uint8_t *ATR)
{
  uint8_t i = 0, flag = 0, buf = 0, protocol = 0;

  sim_ATR.TS = ATR[0]; /* Initial character */
  sim_ATR.T0 = ATR[1]; /* Format character */

  sim_ATR.Hlength = sim_ATR.T0 & (uint8_t)0x0F;

  if ((sim_ATR.T0 & (uint8_t)0x80) == 0x80)
  {
    flag = 1;
  }

  for (i = 0; i < 4; i++)
  {
    sim_ATR.Tlength = sim_ATR.Tlength + (((sim_ATR.T0 & (uint8_t)0xF0) >> (4 + i)) & (uint8_t)0x1);
  }

  for (i = 0; i < sim_ATR.Tlength; i++)
  {
    sim_ATR.T[i] = ATR[i + 2];
  }

  if ((sim_ATR.T0 & (uint8_t)0x80) == 0x00)
  {
    protocol = 0;
  }
  else
  {
    protocol = sim_ATR.T[sim_ATR.Tlength - 1] & (uint8_t)0x0F;
  }

  while (flag)
  {
    if ((sim_ATR.T[sim_ATR.Tlength - 1] & (uint8_t)0x80) == 0x80)
    {
      flag = 1;
    }
    else
    {
      flag = 0;
    }

    buf = sim_ATR.Tlength;
    sim_ATR.Tlength = 0;

    for (i = 0; i < 4; i++)
    {
      sim_ATR.Tlength = sim_ATR.Tlength + (((sim_ATR.T[buf - 1] & (uint8_t)0xF0) >> (4 + i)) & (uint8_t)0x1);
    }

    for (i = 0; i < sim_ATR.Tlength; i++)
    {
      sim_ATR.T[buf + i] = ATR[i + 2 + buf];
    }
    sim_ATR.Tlength += (uint8_t)buf;
  }

  for (i = 0; i < sim_ATR.Hlength; i++)
  {
    sim_ATR.H[i] = ATR[i + 2 + sim_ATR.Tlength];
  }

  return (uint8_t)protocol;
}

static void sim_card_response_decode(iso7816_handle_t *sim_card_handle, ADPU_cmd_t *ADPU_cmd, ADPU_response_t *APDU_respon)
{
  uint8_t answer_len = 33;
  uint8_t continue_flag = 1;
  uint32_t receive_time = 0;
  do
  {
    if (((sim_card_handle->p_tx_rx_buffer[0] & (uint8_t)0xF0) == 0x90))
    {
      /* SW1 received */
      APDU_respon->SW1 = sim_card_handle->p_tx_rx_buffer[0];
      APDU_respon->SW2 = sim_card_handle->p_tx_rx_buffer[1];
      continue_flag = 0;
    }
    else if (sim_card_handle->p_tx_rx_buffer[0] == ((uint8_t) ~(ADPU_cmd->Header.INS)) ||
            ((sim_card_handle->p_tx_rx_buffer[0]) == (ADPU_cmd->Header.INS)))
    {
      /* ACK received */
      APDU_respon->Data[0] = sim_card_handle->p_tx_rx_buffer[0];
      continue_flag=0;
    }

    if(continue_flag)
    {
#if APP_DRIVER_USE_ENABLE
      app_iso7816_receive_sync(answer_len, SC_TIMEOUT);
#else
      hal_iso7816_receive(sim_card_handle, answer_len, SC_TIMEOUT);
#endif
     receive_time++;
    }
    if(receive_time > 100)
    {
        continue_flag = 0;
    }
  }while(continue_flag);
}

/**
  * @brief  Initializes all peripheral used for Smartcard interface.
*   @param  sim_card_handle: pointer of iso7816_handle_t
  * @retval None
  */
void sim_card_init(iso7816_handle_t *sim_card_handle)
{
#if !APP_DRIVER_USE_ENABLE
  iso7816_init_t iso7816_config = ISO7816_DEFAULT_CONFIG;
  sim_card_handle->p_instance = ISO7816;
  sim_card_handle->tx_xfer_size = ATR_BUFFER_LEN;
  sim_card_handle->rx_xfer_size = ATR_BUFFER_LEN;
  sim_card_handle->p_tx_rx_buffer = iso7816_buffer;
  sim_card_handle->buffer_size = ATR_BUFFER_LEN;
  sim_card_handle->init = iso7816_config;
  hal_iso7816_init(sim_card_handle);
#endif
}

/**
  * @brief  Card warm reset
*   @param  sim_card_handle: pointer of iso7816_handle_t
  * @retval None
  */
void sim_card_warmreset(iso7816_handle_t *sim_card_handle)
{
#if APP_DRIVER_USE_ENABLE
    app_iso7816_set_action(APP_ISO7816_ACTION_WARMRST);
#else
    hal_iso7816_set_action(sim_card_handle, HAL_ISO7816_ACTION_WARMRST);
#endif
}

/**
  * @brief  Get Answer To Reset (ATR) information
  * @param  sim_card_handle: pointer of iso7816_handle_t
  * @retval None
  */
void sim_card_get_ATR(iso7816_handle_t *sim_card_handle)
{
#if APP_DRIVER_USE_ENABLE
  if (app_iso7816_get_power_states() == APP_ISO7816_PWR_STATE_IDLE) // warm reset
  {
      sim_card_warmreset(sim_card_handle);
    /* Wait until receiving the answer from the card */
      while(app_iso7816_get_power_states() != APP_ISO7816_PWR_STATE_IDLE);
  }
#else
  if (hal_iso7816_get_power_states(sim_card_handle) == HAL_ISO7816_PWR_STATE_IDLE) // warm reset
  {
    sim_card_warmreset(sim_card_handle);
    /* Wait until receiving the answer from the card */
    while (hal_iso7816_get_power_states(sim_card_handle) != HAL_ISO7816_PWR_STATE_IDLE);
  }
#endif
  else // cold reset
  {
#if APP_DRIVER_USE_ENABLE
      app_iso7816_set_action(APP_ISO7816_ACTION_ON);
    /* Wait until receiving the answer from the card */
      while(app_iso7816_get_power_states() != APP_ISO7816_PWR_STATE_IDLE);
#else
      hal_iso7816_set_action(sim_card_handle, HAL_ISO7816_ACTION_ON);
    /* Wait until receiving the answer from the card */
       while(hal_iso7816_get_power_states(sim_card_handle) != HAL_ISO7816_PWR_STATE_IDLE);
#endif
  }
  sim_card_decode_ATR(sim_card_handle->p_tx_rx_buffer);
}

/**
  * @brief  Protocol Type Select (response to the ATR) procedure and config communication speed
  * @param  sim_card_handle: pointer of iso7816_handle_t
  * @retval None
  */
void sim_card_PTS(iso7816_handle_t *sim_card_handle)
{
  uint8_t PPSConfirmStatus = 1;

  uint8_t cmd_len = 4;
  uint8_t answer_len = 4;

  if ((sim_ATR.T0 & (uint8_t)0x10) == 0x10)
  {
    if (sim_ATR.T[0] != 0x11) //0x11 means the card only supporting default speed (F=372, D=1)
    {

      /* PTSS identifies the PTS request or response and is equal to 0xFF */
      sim_card_handle->p_tx_rx_buffer[0] = 0xFF;

      /* PTS0 indicates by the bits b5, b6, b7 equal to 1 the presence of the optional
      bytes PTSI1, PTS2, PTS3 respectively */
      sim_card_handle->p_tx_rx_buffer[1] = 0x10;

      /* PTS1 allows the interface device to propose value of F and D to the card */
      sim_card_handle->p_tx_rx_buffer[2] = sim_ATR.T[0];

      /* PCK check character */
      uint8_t PCK = (sim_card_handle->p_tx_rx_buffer[0]) ^ (sim_card_handle->p_tx_rx_buffer[1]) ^ (sim_card_handle->p_tx_rx_buffer[2]);
      sim_card_handle->p_tx_rx_buffer[3] = PCK;
#if APP_DRIVER_USE_ENABLE
      app_iso7816_transmit_receive_sync(cmd_len, answer_len, SC_TIMEOUT);
#else
      hal_iso7816_transmit_receive(sim_card_handle, cmd_len, answer_len, SC_TIMEOUT);
#endif
      /* Check the command response (buffer data is changed after reseived response) */
      if ((sim_card_handle->p_tx_rx_buffer[0] != 0xFF) || (sim_card_handle->p_tx_rx_buffer[1] != 0x10) || (sim_card_handle->p_tx_rx_buffer[2] != sim_ATR.T[0]) || (sim_card_handle->p_tx_rx_buffer[3] != PCK))
      {
        /* PPSS, PPS0, PPS1 and PCK exchange unsuccessful */
        PPSConfirmStatus = 0x00;
      }
      /* PPS exchange successful */
      if (PPSConfirmStatus == 0x01)
      {
        uint32_t F = F_Table[((sim_ATR.T[0] >> 4) & (uint8_t)0x0F)];
        uint32_t D = D_Table[(sim_ATR.T[0] & (uint8_t)0x0F)];
#if APP_DRIVER_USE_ENABLE
        app_iso7816_set_etudiv(F / D - 1); // etudiv = F/D - 1
#else
        hal_iso7816_set_etudiv(sim_card_handle, F / D - 1); // etudiv = F/D - 1
#endif
      }
    }
  }
}

/**
  * @brief  Manages the sim card transport layer: send APDU commands and receives
  *         the APDU response.
  * @param  sim_ADPU: pointer to a ADPU_cmd_t structure which will be initialized.
  * @param  ADPU_response_t: pointer to a ADPU_response_t structure which will be initialized.
  * @retval None
  */
void sim_card_send_cmd_get_response(iso7816_handle_t *sim_card_handle, ADPU_cmd_t *ADPU_cmd, ADPU_response_t *APDU_respon)
{
  uint8_t cmd_len = 5;
  uint8_t answer_len = 33;

  /* Reset response buffer ---------------------------------------------------*/
  for (uint8_t i = 0; i < LC_MAX; i++)
  {
    APDU_respon->Data[i] = 0;
  }
  APDU_respon->SW1 = 0;
  APDU_respon->SW2 = 0;

  /* Send header -------------------------------------------------------------*/
  sim_card_handle->p_tx_rx_buffer[0] = ADPU_cmd->Header.CLA;
  sim_card_handle->p_tx_rx_buffer[1] = ADPU_cmd->Header.INS;
  sim_card_handle->p_tx_rx_buffer[2] = ADPU_cmd->Header.P1;
  sim_card_handle->p_tx_rx_buffer[3] = ADPU_cmd->Header.P2;

  /* Send body length to/from SC ---------------------------------------------*/
  if (ADPU_cmd->Body.LC)
  {
    sim_card_handle->p_tx_rx_buffer[4] = ADPU_cmd->Body.LC;
#if APP_DRIVER_USE_ENABLE
    app_iso7816_transmit_receive_sync(cmd_len, answer_len, SC_TIMEOUT);
#else
    hal_iso7816_transmit_receive(sim_card_handle, cmd_len, answer_len, SC_TIMEOUT);
#endif
  }
  else if (ADPU_cmd->Body.LE)
  {
    sim_card_handle->p_tx_rx_buffer[4] = ADPU_cmd->Body.LE;
#if APP_DRIVER_USE_ENABLE
    app_iso7816_transmit_receive_sync(cmd_len, answer_len, SC_TIMEOUT);
#else
    hal_iso7816_transmit_receive(sim_card_handle, cmd_len, answer_len, SC_TIMEOUT);
#endif
  }

  sim_card_response_decode(sim_card_handle, ADPU_cmd, APDU_respon);

  /* If no status bytes received ---------------------------------------------*/
  if (APDU_respon->SW1 == 0x00)
  {
    /* Send body data to SC --------------------------------------------------*/
    if (ADPU_cmd->Body.LC)
    {
      /* Send body data */
      for (uint8_t i = 0; i < ADPU_cmd->Body.LC; i++)
      {
        sim_card_handle->p_tx_rx_buffer[i] = ADPU_cmd->Body.Data[i];
      }
#if APP_DRIVER_USE_ENABLE
      app_iso7816_transmit_receive_sync(ADPU_cmd->Body.LC, 4, SC_TIMEOUT);
#else
      hal_iso7816_transmit_receive(sim_card_handle, ADPU_cmd->Body.LC, 4, SC_TIMEOUT);
#endif
     sim_card_response_decode(sim_card_handle, ADPU_cmd, APDU_respon);
    }
    /* Or receive body data from SC ------------------------------------------*/
    else if (ADPU_cmd->Body.LE)
    {
      /* Decode the body data */
      for (uint8_t i = 0; i < ADPU_cmd->Body.LE; i++)
      {
        APDU_respon->Data[i] = sim_card_handle->p_tx_rx_buffer[i];
      }

      /* Decode the SW1 */
      APDU_respon->SW1 = sim_card_handle->p_tx_rx_buffer[ADPU_cmd->Body.LE + 1];
      /* Decode the SW2 */
      APDU_respon->SW2 = sim_card_handle->p_tx_rx_buffer[ADPU_cmd->Body.LE + 2];
    }
  }
}

/**
  * @brief  select master root dirctory
  * @param  sim_card_handle: pointer to a iso7816_handle_t.
  * @retval None
  */
void sim_card_select_MF(iso7816_handle_t *sim_card_handle)
{
  uint8_t i = 0;
  /* Select MF */
  sim_ADPU.Header.CLA = SC_CLA_GSM11;
  sim_ADPU.Header.INS = SC_SELECT_FILE;
  sim_ADPU.Header.P1 = 0x00;
  sim_ADPU.Header.P2 = 0x00;
  sim_ADPU.Body.LC = 0x02;

  for (i = 0; i < sim_ADPU.Body.LC; i++)
  {
    sim_ADPU.Body.Data[i] = MasterRoot[i];
  }
  while (i < LC_MAX)
  {
    sim_ADPU.Body.Data[i++] = 0;
  }
  sim_ADPU.Body.LE = 0;

  sim_card_send_cmd_get_response(sim_card_handle, &sim_ADPU, &APDU_response);

  /* Get Response on MF */
  if (APDU_response.SW1 == SC_DF_SELECTED)
  {
    sim_ADPU.Header.CLA = SC_CLA_GSM11;
    sim_ADPU.Header.INS = SC_GET_RESPONCE;
    sim_ADPU.Header.P1 = 0x00;
    sim_ADPU.Header.P2 = 0x00;
    sim_ADPU.Body.LC = 0x00;
    sim_ADPU.Body.LE = APDU_response.SW2;

    sim_card_send_cmd_get_response(sim_card_handle, &sim_ADPU, &APDU_response);
  }
}


/**
  * @brief  read ICCID file
  * @param  sim_card_handle: pointer to a iso7816_handle_t.
  * @retval None
  */
void sim_card_read_ICCID(iso7816_handle_t *sim_card_handle)
{
  uint8_t i = 0;
  /* Select ICCID */
  if (((APDU_response.SW1 << 8) | (APDU_response.SW2)) == SC_OP_TERMINATED)
  {
    /* Check if the CHV1 is enabled */
    if ((APDU_response.Data[13] & 0x80) == 0x00)
    {
      CHV1_status = 0x01;
    }
    /* Send APDU Command for ICCID selection */
    sim_ADPU.Header.CLA = SC_CLA_GSM11;
    sim_ADPU.Header.INS = SC_SELECT_FILE;
    sim_ADPU.Header.P1 = 0x00;
    sim_ADPU.Header.P2 = 0x00;
    sim_ADPU.Body.LC = 0x02;

    for (i = 0; i < sim_ADPU.Body.LC; i++)
    {
      sim_ADPU.Body.Data[i] = ICCID[i];
    }
    while (i < LC_MAX)
    {
      sim_ADPU.Body.Data[i++] = 0;
    }
    sim_ADPU.Body.LE = 0;

    sim_card_send_cmd_get_response(sim_card_handle, &sim_ADPU, &APDU_response);
  }

  /* Read Binary in ICCID */
  if (APDU_response.SW1 == SC_EF_SELECTED)
  {
    sim_ADPU.Header.CLA = SC_CLA_GSM11;
    sim_ADPU.Header.INS = SC_READ_BINARY;
    sim_ADPU.Header.P1 = 0x00;
    sim_ADPU.Header.P2 = 0x00;
    sim_ADPU.Body.LC = 0x00;
    sim_ADPU.Body.LE = 10;

    sim_card_send_cmd_get_response(sim_card_handle, &sim_ADPU, &APDU_response);

    if (((APDU_response.SW1 << 8) | (APDU_response.SW2)) == SC_OP_TERMINATED)
    {
      /* Copy the ICCID File content into ICCID_content buffer */
      for (i = 0; i < sim_ADPU.Body.LE; i++)
      {
        ICCID_content[i] = sim_card_handle->p_tx_rx_buffer[i + 1];
      }
    }
  }
}

void sim_card_read_IMSI(iso7816_handle_t *sim_card_handle)
{
  uint8_t i = 0;
  /* Send APDU Command for GSMDir selection */
  sim_ADPU.Header.CLA = SC_CLA_GSM11;
  sim_ADPU.Header.INS = SC_SELECT_FILE;
  sim_ADPU.Header.P1 = 0x00;
  sim_ADPU.Header.P2 = 0x00;
  sim_ADPU.Body.LC = 0x02;

  for (i = 0; i < sim_ADPU.Body.LC; i++)
  {
    sim_ADPU.Body.Data[i] = GSMDir[i];
  }
  while (i < LC_MAX)
  {
    sim_ADPU.Body.Data[i++] = 0;
  }
  sim_ADPU.Body.LE = 0;

  sim_card_send_cmd_get_response(sim_card_handle, &sim_ADPU, &APDU_response);

  /* Select IMSI -----------------------------------------------------------*/
  if (APDU_response.SW1 == SC_DF_SELECTED)
  {
    sim_ADPU.Header.CLA = SC_CLA_GSM11;
    sim_ADPU.Header.INS = SC_SELECT_FILE;
    sim_ADPU.Header.P1 = 0x00;
    sim_ADPU.Header.P2 = 0x00;
    sim_ADPU.Body.LC = 0x02;

    for (i = 0; i < sim_ADPU.Body.LC; i++)
    {
      sim_ADPU.Body.Data[i] = IMSI[i];
    }
    while (i < LC_MAX)
    {
      sim_ADPU.Body.Data[i++] = 0;
    }
    sim_ADPU.Body.LE = 0;

    sim_card_send_cmd_get_response(sim_card_handle, &sim_ADPU, &APDU_response);
  }

  /* Read Binary in IMSI ---------------------------------------------------*/
  if (CHV1_status == 0x00)
  {
    if (((APDU_response.SW1 << 8) | (APDU_response.SW2)) == SC_OP_TERMINATED)
    {
      /* Enable CHV1 (PIN1) ------------------------------------------------*/
      sim_ADPU.Header.CLA = SC_CLA_GSM11;
      sim_ADPU.Header.INS = SC_ENABLE;
      sim_ADPU.Header.P1 = 0x00;
      sim_ADPU.Header.P2 = 0x01;
      sim_ADPU.Body.LC = 0x08;

      for (i = 0; i < sim_ADPU.Body.LC; i++)
      {
        sim_ADPU.Body.Data[i] = CHV1[i];
      }
      while (i < LC_MAX)
      {
        sim_ADPU.Body.Data[i++] = 0;
      }
      sim_ADPU.Body.LE = 0;

      sim_card_send_cmd_get_response(sim_card_handle, &sim_ADPU, &APDU_response);
    }
  }
  else
  {
    if (((APDU_response.SW1 << 8) | (APDU_response.SW2)) == SC_OP_TERMINATED)
    {
      /* Verify CHV1 (PIN1) ------------------------------------------------*/
      sim_ADPU.Header.CLA = SC_CLA_GSM11;
      sim_ADPU.Header.INS = SC_VERIFY;
      sim_ADPU.Header.P1 = 0x00;
      sim_ADPU.Header.P2 = 0x01;
      sim_ADPU.Body.LC = 0x08;

      for (i = 0; i < sim_ADPU.Body.LC; i++)
      {
        sim_ADPU.Body.Data[i] = CHV1[i];
      }
      while (i < LC_MAX)
      {
        sim_ADPU.Body.Data[i++] = 0;
      }
      sim_ADPU.Body.LE = 0;

      sim_card_send_cmd_get_response(sim_card_handle, &sim_ADPU, &APDU_response);
    }
  }

  /* Read Binary in IMSI ---------------------------------------------------*/
  if (APDU_response.SW1 == SC_EF_SELECTED)
  {
    sim_ADPU.Header.CLA = SC_CLA_GSM11;
    sim_ADPU.Header.INS = SC_READ_BINARY;
    sim_ADPU.Header.P1 = 0x00;
    sim_ADPU.Header.P2 = 0x00;
    sim_ADPU.Body.LC = 0x00;

    sim_ADPU.Body.LE = 9;

    sim_card_send_cmd_get_response(sim_card_handle, &sim_ADPU, &APDU_response);
  }

  if (((APDU_response.SW1 << 8) | (APDU_response.SW2)) == SC_OP_TERMINATED)
  {
    /* Copy the IMSI File content into IMSI_content buffer */
    for (i = 0; i < sim_ADPU.Body.LE; i++)
    {
      IMSI_content[i] = sim_card_handle->p_tx_rx_buffer[i + 1];
    }
  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT GOODIX *****END OF FILE****/

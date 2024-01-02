/**
 ****************************************************************************************
 *
 * @file nvds_tag.h
 *
 * @brief Nvds tag management file.
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
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */
/*
 * DEFINES
 *****************************************************************************************
 */
#ifndef __NVDS_TAG_MGR_H__
#define __NVDS_TAG_MGR_H__

#define APP_PARAM_ID_RF_LPCLK_DRIFT          0xC003
#define APP_PARAM_ID_ACTIVITY_MOVE_CONFIG    0xC005
#define APP_PARAM_ID_RF_XO_OFFSET            0xC016

/// List of parameters identifiers
enum PARAM_ID
{
    /// Definition of the tag associated to each parameters
    /// Local Bd Address
    PARAM_ID_BD_ADDRESS                 = 0x01,
    /// Device Name
    PARAM_ID_DEVICE_NAME                = 0x02,
    /// Low Power Clock Drift
    PARAM_ID_LPCLK_DRIFT                = 0x03,
    /// Active Clock Drift
    PARAM_ID_ACTCLK_DRIFT               = 0x04,

    /// Activity Move Configuration (enables/disables activity move for BLE connections and BT (e)SCO links)
    PARAM_ID_ACTIVITY_MOVE_CONFIG       = 0x05,

    /// Enable/disable scanning for extended advertising PDUs
    PARAM_ID_SCAN_EXT_ADV               = 0x06,

    /// Duration of the schedule reservation for long activities such as scan, inquiry, page, HDC advertising
    PARAM_ID_SCHED_SCAN_DUR             = 0x07,

    /// Enable/disable channel assessment for BT and/or BLE
    PARAM_ID_CH_ASS_EN                  = 0x08,

    /// Default MD bit used by slave when sending a data packet on a BLE connection
    PARAM_ID_DFT_SLAVE_MD               = 0x09,

    /// Tracer configuration
    PARAM_ID_TRACER_CONFIG              = 0x0A,

    /// Diagport configuration
    PARAM_ID_DIAG_BLE_HW                = 0x0B,
    PARAM_ID_DIAG_SW                    = 0x0C,
    PARAM_ID_DIAG_DM_HW                 = 0x0D,
    PARAM_ID_DIAG_PLF                   = 0x0E,

    /// RSSI threshold tags
    PARAM_ID_RSSI_HIGH_THR              = 0x0F,
    PARAM_ID_RSSI_LOW_THR               = 0x10,
    PARAM_ID_RSSI_INTERF_THR            = 0x11,

    /// RF BTIPT
    PARAM_ID_RF_BTIPT_VERSION           = 0x12,
    PARAM_ID_RF_BTIPT_XO_SETTING        = 0x13,
    PARAM_ID_RF_BTIPT_GAIN_SETTING      = 0x14,

    /// LE Coded PHY 500 Kbps selection
    PARAM_ID_LE_CODED_PHY_500           = 0x15,

    /// Add by Goodix: RF XO offset
    PARAM_ID_RF_XO_OFFSET               = 0x16,

    /// Add by Goodix: Advertising Packet Interval
    PARAM_ID_GDX_ADV_INT                = 0x17,
};

#endif //__NVDS_TAG_MGR_H__


/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_usb.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of usb LL library.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2020 GOODIX
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

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup LL_DRIVER LL Driver
  * @{
  */

/** @defgroup LL_USB Usb Module Driver
  * @brief USB LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_USB_H__
#define __GR55xx_LL_USB_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (USB)

/** @defgroup LL_USB_DRIVER_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup USB_LL_ES_INTT USB Exported init structure
  * @{
  */

/**
  * @brief LL USB init structures definition
  */
typedef struct _ll_usb_init_t
{
    uint32_t pwr_mode;              /**< Specifies power mode of usb modlue.
                                         This parameter can be a value of @ref USB_LL_EC_PWR_MODE */

    uint32_t speed;                 /**< Specifies the USB Transceiver speed.
                                         This parameter can be a value of @ref USB_LL_EC_XCRV_CTRL_SPEED.
                                         This feature can be modified afterwards using unitary function @ref ll_usb_set_xcrv_speed().*/

    uint32_t enum_type;             /**< Specifies the USB enumeration type( hardware enumeration  and MCU enumeration).
                                         This parameter can be a value of @ref USB_LL_EC_ENUM_TYPE.
                                         This feature can be modified afterwards using unitary function @ref ll_usb_set_usb_enum_type().*/
} ll_usb_init_t;

/** @} */

/** @} */

/**
  * @defgroup  USB_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup USB_LL_Exported_Constants USB Exported Constants
  * @{
  */

/** @defgroup USB_LL_EC_PWR_MODE USB poewr mode select
  * @{
  */
#define LL_USB_PWR_MODE_LP                     (0UL)  /**< USB runing in low power mode         */
#define LL_USB_PWR_MODE_NORMAL                 (1UL)  /**< USB runing in normal mode            */

/** @} */

/** @defgroup USB_LL_EC_ENUM_TYPE USB enumeration type
  * @{
  */
#define LL_USB_ENUM_TYPE_HW                     (0x00000000UL)                  /**< USB hardware enumeration type              */
#define LL_USB_ENUM_TYPE_MCU                    (1UL << USB_CTRL_MCU_ENUM_Pos)  /**< USB MCU enumeration type                   */

/** @} */

/** @defgroup USB_LL_EC_XCRV_LDO_BIAS_SEL Transceiver LDO_3.3V vout trimming signal select
  * @{
  */
#define LL_USB_TRX_LDO_BIAS_SEL0              (0x00000000UL)                                 /**< LDO_3.3V vout trimming signal select: 0      */
#define LL_USB_TRX_LDO_BIAS_SEL1              (1UL << AON_PMU_USB_TRX_LDO_BIAS_SEL_Pos)      /**< LDO_3.3V vout trimming signal select: 1      */
#define LL_USB_TRX_LDO_BIAS_SEL2              (2UL << AON_PMU_USB_TRX_LDO_BIAS_SEL_Pos)      /**< LDO_3.3V vout trimming signal select: 2      */
#define LL_USB_TRX_LDO_BIAS_SEL3              (3UL << AON_PMU_USB_TRX_LDO_BIAS_SEL_Pos)      /**< LDO_3.3V vout trimming signal select: 3      */
#define LL_USB_TRX_LDO_BIAS_SEL4              (4UL << AON_PMU_USB_TRX_LDO_BIAS_SEL_Pos)      /**< LDO_3.3V vout trimming signal select: 4      */
#define LL_USB_TRX_LDO_BIAS_SEL5              (5UL << AON_PMU_USB_TRX_LDO_BIAS_SEL_Pos)      /**< LDO_3.3V vout trimming signal select: 5      */
#define LL_USB_TRX_LDO_BIAS_SEL6              (6UL << AON_PMU_USB_TRX_LDO_BIAS_SEL_Pos)      /**< LDO_3.3V vout trimming signal select: 6      */
#define LL_USB_TRX_LDO_BIAS_SEL7              (7UL << AON_PMU_USB_TRX_LDO_BIAS_SEL_Pos)      /**< LDO_3.3V vout trimming signal select: 7      */
/** @} */


/** @defgroup USB_LL_EC_XCRV_LDO_VSEL Transceiver LDO_3.3V offer XCVR's current select
  * @{
  */
#define LL_USB_TRX_LDO_VSEL0                   (0x00000000UL)                                 /**< LDO_3.3V offer XCVR's select: 0      */
#define LL_USB_TRX_LDO_VSEL1                   (1UL << AON_PMU_USB_TRX_LDO_VSEL_Pos)          /**< LDO_3.3V offer XCVR's select: 1      */
#define LL_USB_TRX_LDO_VSEL2                   (2UL << AON_PMU_USB_TRX_LDO_VSEL_Pos)          /**< LDO_3.3V offer XCVR's select: 2      */
#define LL_USB_TRX_LDO_VSEL3                   (3UL << AON_PMU_USB_TRX_LDO_VSEL_Pos)          /**< LDO_3.3V offer XCVR's select: 3      */
#define LL_USB_TRX_LDO_VSEL4                   (4UL << AON_PMU_USB_TRX_LDO_VSEL_Pos)          /**< LDO_3.3V offer XCVR's select: 4      */
#define LL_USB_TRX_LDO_VSEL5                   (5UL << AON_PMU_USB_TRX_LDO_VSEL_Pos)          /**< LDO_3.3V offer XCVR's select: 5      */
#define LL_USB_TRX_LDO_VSEL6                   (6UL << AON_PMU_USB_TRX_LDO_VSEL_Pos)          /**< LDO_3.3V offer XCVR's select: 6      */
#define LL_USB_TRX_LDO_VSEL7                   (7UL << AON_PMU_USB_TRX_LDO_VSEL_Pos)          /**< LDO_3.3V offer XCVR's select: 7      */
/** @} */

/** @defgroup  USB_LL_EC_XCRV_CTRL_SPEED Transceiver speed select
  * @{
  */
#define LL_USB_XCRV_CTRL_SPEED_LOW              (0x00000000UL)                                  /**< USB Transceiver speed select: low speed            */
#define LL_USB_XCRV_CTRL_SPEED_FULL             (1UL << MCU_SUB_USB_XCRV_CTRL_SPEED_Pos)        /**< USB Transceiver speed select: full speed           */
/** @} */

/** @defgroup USB_LL_EC_XCRV_CTRL_RTRIMN Transceiver D- output impedance trim
  * @{
  */
#define LL_USB_XCRV_CTRL_RTRIMN0                (0x00000000UL)                                  /**< Transceiver D- output impedance trim: 0      */
#define LL_USB_XCRV_CTRL_RTRIMN1                (1UL << MCU_SUB_USB_XCRV_CTRL_RTRIMN_Pos)       /**< Transceiver D- output impedance trim: 1      */
#define LL_USB_XCRV_CTRL_RTRIMN2                (2UL << MCU_SUB_USB_XCRV_CTRL_RTRIMN_Pos)       /**< Transceiver D- output impedance trim: 2      */
#define LL_USB_XCRV_CTRL_RTRIMN3                (3UL << MCU_SUB_USB_XCRV_CTRL_RTRIMN_Pos)       /**< Transceiver D- output impedance trim: 3      */
#define LL_USB_XCRV_CTRL_RTRIMN4                (4UL << MCU_SUB_USB_XCRV_CTRL_RTRIMN_Pos)       /**< Transceiver D- output impedance trim: 4      */
#define LL_USB_XCRV_CTRL_RTRIMN5                (5UL << MCU_SUB_USB_XCRV_CTRL_RTRIMN_Pos)       /**< Transceiver D- output impedance trim: 5      */
#define LL_USB_XCRV_CTRL_RTRIMN6                (6UL << MCU_SUB_USB_XCRV_CTRL_RTRIMN_Pos)       /**< Transceiver D- output impedance trim: 6      */
#define LL_USB_XCRV_CTRL_RTRIMN7                (7UL << MCU_SUB_USB_XCRV_CTRL_RTRIMN_Pos)       /**< Transceiver D- output impedance trim: 7      */
/** @} */

/** @defgroup USB_LL_EC_XCRV_CTRL_RTRIMP Transceiver D+ output impedance trim
  * @{
  */
#define LL_USB_XCRV_CTRL_RTRIMP0                (0x00000000UL)                                  /**< Transceiver D+ output impedance trim: 0      */
#define LL_USB_XCRV_CTRL_RTRIMP1                (1UL << MCU_SUB_USB_XCRV_CTRL_RTRIMP_Pos)       /**< Transceiver D+ output impedance trim: 1      */
#define LL_USB_XCRV_CTRL_RTRIMP2                (2UL << MCU_SUB_USB_XCRV_CTRL_RTRIMP_Pos)       /**< Transceiver D+ output impedance trim: 2      */
#define LL_USB_XCRV_CTRL_RTRIMP3                (3UL << MCU_SUB_USB_XCRV_CTRL_RTRIMP_Pos)       /**< Transceiver D+ output impedance trim: 3      */
#define LL_USB_XCRV_CTRL_RTRIMP4                (4UL << MCU_SUB_USB_XCRV_CTRL_RTRIMP_Pos)       /**< Transceiver D+ output impedance trim: 4      */
#define LL_USB_XCRV_CTRL_RTRIMP5                (5UL << MCU_SUB_USB_XCRV_CTRL_RTRIMP_Pos)       /**< Transceiver D+ output impedance trim: 5      */
#define LL_USB_XCRV_CTRL_RTRIMP6                (6UL << MCU_SUB_USB_XCRV_CTRL_RTRIMP_Pos)       /**< Transceiver D+ output impedance trim: 6      */
#define LL_USB_XCRV_CTRL_RTRIMP7                (7UL << MCU_SUB_USB_XCRV_CTRL_RTRIMP_Pos)       /**< Transceiver D+ output impedance trim: 7      */
/** @} */

/** @defgroup USB_LL_EC_EP_ATTR_EP1 Endpoint 1 attribute setting
  * @{
  */
#define LL_USB_EP_ATTR_EP1_INT                  (0x00000000UL)                                  /**< Endpoint 1 attribute setting: interrupt        */
#define LL_USB_EP_ATTR_EP1_ISO                  (1UL << USB_EP_ATTR_EP1_Pos)                    /**< Endpoint 1 attribute setting: Isochronous      */
#define LL_USB_EP_ATTR_EP1_BULK                 (2UL << USB_EP_ATTR_EP1_Pos)                    /**< Endpoint 1 attribute setting: bulk             */
/** @} */

/** @defgroup USB_LL_EC_EP_ATTR_EP2 Endpoint 2 attribute setting
  * @{
  */
#define LL_USB_EP_ATTR_EP2_INT                  (0x00000000UL)                                  /**< Endpoint 2 attribute setting: interrupt        */
#define LL_USB_EP_ATTR_EP2_ISO                  (1UL << USB_EP_ATTR_EP2_Pos)                    /**< Endpoint 2 attribute setting: Isochronous      */
#define LL_USB_EP_ATTR_EP2_BULK                 (2UL << USB_EP_ATTR_EP2_Pos)                    /**< Endpoint 2 attribute setting: bulk             */
/** @} */

/** @defgroup USB_LL_EC_EP_ATTR_EP3 Endpoint 3 attribute setting
  * @{
  */
#define LL_USB_EP_ATTR_EP3_INT                  (0x00000000UL)                                  /**< Endpoint 3 attribute setting: interrupt        */
#define LL_USB_EP_ATTR_EP3_ISO                  (1UL << USB_EP_ATTR_EP3_Pos)                    /**< Endpoint 3 attribute setting: Isochronous      */
#define LL_USB_EP_ATTR_EP3_BULK                 (2UL << USB_EP_ATTR_EP3_Pos)                    /**< Endpoint 3 attribute setting: bulk             */
/** @} */

/** @defgroup USB_LL_EC_INT_STAT USB interrupt status
  * @{
  */
#define LL_USB_INT_STAT_ALL                     USB_INT_STAT_ALL                                /**< USB interrupt status all bit status                        */
#define LL_USB_INT_STAT_SUSPEND                 USB_INT_STAT_SUSPEND                            /**< USB suspend status interrupt active                        */
#define LL_USB_INT_STAT_EP0_OUT_READY           USB_INT_STAT_EP0_OUT_READY                      /**< USB ep0 data out ready interrupt active                    */
#define LL_USB_INT_STAT_EP1_OUT_READY           USB_INT_STAT_EP1_OUT_READY                      /**< USB ep1 data out ready interrupt active                    */
#define LL_USB_INT_STAT_CRC16_ERR               USB_INT_STAT_CRC16_ERR                          /**< USB receive CRC error data interrupt active                */
#define LL_USB_INT_STAT_UPID_ERR                USB_INT_STAT_UPID_ERR                           /**< USB receive unsupported PID interrupt active               */
#define LL_USB_INT_STAT_TIMEOUT_ERR             USB_INT_STAT_TIMEOUT_ERR                        /**< USB rx/tx timeout error interrupt active                   */
#define LL_USB_INT_STAT_SEQ_ERR                 USB_INT_STAT_SEQ_ERR                            /**< USB DATA0/DATA1 PID sequence error interrupt active        */
#define LL_USB_INT_STAT_PID_CKS_ERR             USB_INT_STAT_PID_CKS_ERR                        /**< USB PID checksum error interrupt active                    */
#define LL_USB_INT_STAT_PID_CRC_ERR             USB_INT_STAT_PID_CRC_ERR                        /**< USB PID CRC error interrupt active                         */
#define LL_USB_INT_STAT_HOST_RESET              USB_INT_STAT_HOST_RESET                         /**< USB host reset interrupt active                            */
#define LL_USB_INT_STAT_AHB_XFER_ERR            USB_INT_STAT_AHB_XFER_ERR                       /**< USB ep3 and ep4 AHB master receive error interrupt active  */
#define LL_USB_INT_STAT_NSE_ERR                 USB_INT_STAT_NSE_ERR                            /**< USB no such endpoint error interrupt active                */
#define LL_USB_INT_STAT_EP3_AHB_XFER_DONE       USB_INT_STAT_EP3_AHB_XFER_DONE                  /**< USB ep3 AHB master transfer done interrupt active          */
#define LL_USB_INT_STAT_SYNC_ERR                USB_INT_STAT_SYNC_ERR                           /**< USB SYNC error interrupt active                            */
#define LL_USB_INT_STAT_BIT_STUFF_ERR           USB_INT_STAT_BIT_STUFF_ERR                      /**< USB bit stuff error interrupt active                       */
#define LL_USB_INT_STAT_BYTE_ERR                USB_INT_STAT_BYTE_ERR                           /**< USB byte error interrupt active                            */
#define LL_USB_INT_STAT_SOF                     USB_INT_STAT_SOF                                /**< USB SOF interrupt active                                   */
#define LL_USB_INT_STAT_EP0_TX_DONE             USB_INT_STAT_EP0_TX_DONE                        /**< USB ep0 TX done interrupt active                           */
#define LL_USB_INT_STAT_EP2_TX_DONE             USB_INT_STAT_EP2_TX_DONE                        /**< USB ep2 TX done interrupt active                           */
#define LL_USB_INT_STAT_EP3_TX_DONE             USB_INT_STAT_EP3_TX_DONE                        /**< USB ep3 TX done interrupt active                           */
#define LL_USB_INT_STAT_INTO_CONFIG             USB_INT_STAT_INTO_CONFIG                        /**< USB into cofig status interrupt active                     */
#define LL_USB_INT_STAT_EP5_OUT_READY           USB_INT_STAT_EP5_OUT_READY                      /**< USB ep5 data out ready interrupt active                    */
#define LL_USB_INT_STAT_EP4_AHB_XFER_DONE       USB_INT_STAT_EP4_AHB_XFER_DONE                  /**< USB ep4 AHB master transfer done interrupt active          */
#define LL_USB_INT_STAT_EP4_TX_DONE             USB_INT_STAT_EP4_TX_DONE                        /**< USB ep4 TX done interrupt active                           */
#define LL_USB_INT_STAT_EP5_AHB_XFER_DONE       USB_INT_STAT_EP5_AHB_XFER_DONE                  /**< USB ep5 AHB master transfer done interrupt active          */
#define LL_USB_INT_STAT_EP5_TIMER_OUT_ERR       USB_INT_STAT_EP5_TIMER_OUT_ERR                  /**< USB ep5 timer out error interrupt active                   */
/** @} */

/** @defgroup USB_LL_EC_INT_EN USB interrupt enable
  * @{
  */
#define LL_USB_INT_EN_ALL                       USB_INT_EN_ALL                                  /**< USB all interrupt enable                                   */
#define LL_USB_INT_RESET_VAL                    USB_INT_EN_RESET_VAL                            /**< USB interrupt control reset value                          */
#define LL_USB_INT_EN_SUSPEND                   USB_INT_EN_SUSPEND                              /**< USB suspend status interrupt enable                        */
#define LL_USB_INT_EN_EP0_OUT_READY             USB_INT_EN_EP0_OUT_READY                        /**< USB ep0 data out ready interrupt enable                    */
#define LL_USB_INT_EN_EP1_OUT_READY             USB_INT_EN_EP1_OUT_READY                        /**< USB ep1 data out ready interrupt enable                    */
#define LL_USB_INT_EN_CRC16_ERR                 USB_INT_EN_CRC16_ERR                            /**< USB receive CRC error data interrupt enable                */
#define LL_USB_INT_EN_UPID_ERR                  USB_INT_EN_UPID_ERR                             /**< USB receive unsupported PID interrupt enable               */
#define LL_USB_INT_EN_TIMEOUT_ERR               USB_INT_EN_TIMEOUT_ERR                          /**< USB rx/tx timeout error interrupt enable                   */
#define LL_USB_INT_EN_SEQ_ERR                   USB_INT_EN_SEQ_ERR                              /**< USB DATA0/DATA1 PID sequence error interrupt enable        */
#define LL_USB_INT_EN_PID_CKS_ERR               USB_INT_EN_PID_CKS_ERR                          /**< USB PID checksum error interrupt enable                    */
#define LL_USB_INT_EN_PID_CRC_ERR               USB_INT_EN_PID_CRC_ERR                          /**< USB PID CRC error interrupt enable                         */
#define LL_USB_INT_EN_HOST_RESET                USB_INT_EN_HOST_RESET                           /**< USB host reset interrupt enable                            */
#define LL_USB_INT_EN_AHB_XFER_ERR              USB_INT_EN_AHB_XFER_ERR                         /**< USB ep3 and ep4 AHB master receive error interrupt enable  */
#define LL_USB_INT_EN_NSE_ERR                   USB_INT_EN_NSE_ERR                              /**< USB no such endpoint error interrupt enable                */
#define LL_USB_INT_EN_EP3_AHB_XFER_DONE         USB_INT_EN_EP3_AHB_XFER_DONE                    /**< USB ep3 AHB master transfer done interrupt enable          */
#define LL_USB_INT_EN_SYNC_ERR                  USB_INT_EN_SYNC_ERR                             /**< USB SYNC error interrupt enable                            */
#define LL_USB_INT_EN_BIT_STUFF_ERR             USB_INT_EN_BIT_STUFF_ERR                        /**< USB bit stuff error interrupt enable                       */
#define LL_USB_INT_EN_BYTE_ERR                  USB_INT_EN_BYTE_ERR                             /**< USB byte error interrupt enable                            */
#define LL_USB_INT_EN_SOF                       USB_INT_EN_SOF                                  /**< USB SOF interrupt enable                                   */
#define LL_USB_INT_EN_EP0_TX_DONE               USB_INT_EN_EP0_TX_DONE                          /**< USB ep0 TX done interrupt enable                           */
#define LL_USB_INT_EN_EP2_TX_DONE               USB_INT_EN_EP2_TX_DONE                          /**< USB ep2 TX done interrupt enable                           */
#define LL_USB_INT_EN_EP3_TX_DONE               USB_INT_EN_EP3_TX_DONE                          /**< USB ep3 TX done interrupt enable                           */
#define LL_USB_INT_EN_INTO_CONFIG               USB_INT_EN_INTO_CONFIG                          /**< USB into cofig status enable                               */
#define LL_USB_INT_EN_EP5_OUT_READY             USB_INT_EN_EP5_OUT_READY                        /**< USB ep5 data out ready interrupt enable                    */
#define LL_USB_INT_EN_EP4_AHB_XFER_DONE         USB_INT_EN_EP4_AHB_XFER_DONE                    /**< USB ep4 AHB master transfer done interrupt enable          */
#define LL_USB_INT_EN_EP4_TX_DONE               USB_INT_EN_EP4_TX_DONE                          /**< USB ep4 TX done interrupt enable                           */
#define LL_USB_INT_EN_EP5_AHB_XFER_DONE         USB_INT_EN_EP5_AHB_XFER_DONE                    /**< USB ep5 AHB master transfer done interrupt enable          */
#define LL_USB_INT_EN_EP5_TIMER_OUT_ERR         USB_INT_EN_EP5_TIMER_OUT_ERR                    /**< USB ep5 timer out error interrupt enable                   */
/** @} */

/** @defgroup USB_LL_EC_INT_CLR USB interrupt clear
  * @{
  */
#define LL_USB_INT_CLR_ALL                      USB_INT_CLR_ALL                                 /**< USB all interrupt clear                                    */
#define LL_USB_INT_CLR_SUSPEND                  USB_INT_CLR_SUSPEND                             /**< USB suspend status interrupt clear                         */
#define LL_USB_INT_CLR_EP0_OUT_READY            USB_INT_CLR_EP0_OUT_READY                       /**< USB ep0 data out ready interrupt clear                     */
#define LL_USB_INT_CLR_EP1_OUT_READY            USB_INT_CLR_EP1_OUT_READY                       /**< USB ep1 data out ready interrupt clear                     */
#define LL_USB_INT_CLR_CRC16_ERR                USB_INT_CLR_CRC16_ERR                           /**< USB receive CRC error data interrupt clear                 */
#define LL_USB_INT_CLR_UPID_ERR                 USB_INT_CLR_UPID_ERR                            /**< USB receive unsupported PID interrupt clear                */
#define LL_USB_INT_CLR_TIMEOUT_ERR              USB_INT_CLR_TIMEOUT_ERR                         /**< USB rx/tx timeout error interrupt clear                    */
#define LL_USB_INT_CLR_SEQ_ERR                  USB_INT_CLR_SEQ_ERR                             /**< USB DATA0/DATA1 PID sequence error interrupt clear         */
#define LL_USB_INT_CLR_PID_CKS_ERR              USB_INT_CLR_PID_CKS_ERR                         /**< USB PID checksum error interrupt clear                     */
#define LL_USB_INT_CLR_PID_CRC_ERR              USB_INT_CLR_PID_CRC_ERR                         /**< USB PID CRC error interrupt clear                          */
#define LL_USB_INT_CLR_HOST_RESET               USB_INT_CLR_HOST_RESET                          /**< USB host reset interrupt clear                             */
#define LL_USB_INT_CLR_AHB_XFER_ERR             USB_INT_CLR_AHB_XFER_ERR                        /**< USB ep3 and ep4 AHB master receive error interrupt clear   */
#define LL_USB_INT_CLR_NSE_ERR                  USB_INT_CLR_NSE_ERR                             /**< USB no such endpoint error interrupt clear                 */
#define LL_USB_INT_CLR_EP3_AHB_XFER_DONE        USB_INT_CLR_EP3_AHB_XFER_DONE                   /**< USB ep3 AHB master transfer done interrupt clear           */
#define LL_USB_INT_CLR_SYNC_ERR                 USB_INT_CLR_SYNC_ERR                            /**< USB SYNC error interrupt clear                             */
#define LL_USB_INT_CLR_BIT_STUFF_ERR            USB_INT_CLR_BIT_STUFF_ERR                       /**< USB bit stuff error interrupt clear                        */
#define LL_USB_INT_CLR_BYTE_ERR                 USB_INT_CLR_BYTE_ERR                            /**< USB byte error interrupt clear                             */
#define LL_USB_INT_CLR_SOF                      USB_INT_CLR_SOF                                 /**< USB SOF interrupt clear                                    */
#define LL_USB_INT_CLR_EP0_TX_DONE              USB_INT_CLR_EP0_TX_DONE                         /**< USB ep0 TX done interrupt clear                            */
#define LL_USB_INT_CLR_EP2_TX_DONE              USB_INT_CLR_EP2_TX_DONE                         /**< USB ep2 TX done interrupt clear                            */
#define LL_USB_INT_CLR_EP3_TX_DONE              USB_INT_CLR_EP3_TX_DONE                         /**< USB ep3 TX done interrupt clear                            */
#define LL_USB_INT_CLR_INTO_CONFIG              USB_INT_CLR_INTO_CONFIG                         /**< USB into cofig status interrupt clear                      */
#define LL_USB_INT_CLR_EP5_OUT_READY            USB_INT_CLR_EP5_OUT_READY                       /**< USB ep5 data out ready interrupt clear                     */
#define LL_USB_INT_CLR_EP4_AHB_XFER_DONE        USB_INT_CLR_EP4_AHB_XFER_DONE                   /**< USB ep4 AHB master transfer done interrupt clear           */
#define LL_USB_INT_CLR_EP4_TX_DONE              USB_INT_CLR_EP4_TX_DONE                         /**< USB ep4 TX done interrupt clear                            */
#define LL_USB_INT_CLR_EP5_AHB_XFER_DONE        USB_INT_CLR_EP5_AHB_XFER_DONE                   /**< USB ep5 AHB master transfer done interrupt clear           */
#define LL_USB_INT_CLR_EP5_TIMER_OUT_ERR        USB_INT_CLR_EP5_TIMER_OUT_ERR                   /**< USB ep5 timer out error interrupt clear                    */
/** @} */

/** @defgroup USB_LL_EC_CTRL0_OUTPUT_ENDIAN_CTRL USB output data endian
  * @{
  */
#define LL_USB_CTRL0_OUTPUT_ENDIAN_CTRL_SMALL                   (0x00000000UL)                                  /**< USB output data is small endian        */
#define LL_USB_CTRL0_OUTPUT_ENDIAN_CTRL_BIG                     (1UL << USB_CTRL0_OUTPUT_ENDIAN_CTRL_Pos)       /**< USB output data is big endian          */
/** @} */

/** @defgroup USB_LL_EC_CTRL0_INPUT_ENDIAN_CTRL USB input data endian
  * @{
  */
#define LL_USB_CTRL0_INPUT_ENDIAN_CTRL_SMALL                    (0x00000000UL)                                   /**< USB input data is small endian       */
#define LL_USB_CTRL0_INPUT_ENDIAN_CTRL_BIG                      (1UL << USB_CTRL0_INPUT_ENDIAN_CTRL_Pos)         /**< USB input data is big endian         */
/** @} */

/** @defgroup USB_LL_EC_CTRL0_PROBE_SEL USB probe select signal
  * @{
  */
#define LL_USB_CTRL0_PROBE_SEL_PROTOCAL_STAT                    (0x00000000UL)                                  /**< USB main protocal state machine        */
#define LL_USB_CTRL0_PROBE_SEL_RX_STAT                          (1UL << USB_CTRL0_PROBE_SEL_Pos)                /**< USB RX state machine                   */
#define LL_USB_CTRL0_PROBE_SEL_UTMI_SIGNALS                     (2UL << USB_CTRL0_PROBE_SEL_Pos)                /**< USB UTMI signals                       */
#define LL_USB_CTRL0_PROBE_SEL_SYNC_STAT                        (3UL << USB_CTRL0_PROBE_SEL_Pos)                /**< USB SYNC state machine, and RxError    */
#define LL_USB_CTRL0_PROBE_SEL_TX_STAT                          (4UL << USB_CTRL0_PROBE_SEL_Pos)                /**< USB TX state machine                   */
#define LL_USB_CTRL0_PROBE_SEL_DPLL_STAT                        (5UL << USB_CTRL0_PROBE_SEL_Pos)                /**< USB DPLL state machine                 */
/** @} */

/** @defgroup USB_LL_EC_USB_EP4_FIFO_WEN USB ep4 write fifo effective byte
  * @{
  */
#define LL_USB_EP4_FIFO_WEN_DEFAULT                             (15UL << USB_EP4_FIFO_WR_EN_Pos)                /**< USB ep4 write fifo enable default value     */
#define LL_USB_EP4_FIFO_WEN_1BYTE                               (1UL << USB_EP4_FIFO_WR_EN_Pos)                 /**< USB ep4 write fifo 1 byte effective         */
#define LL_USB_EP4_FIFO_WEN_2BYTE                               (3UL << USB_EP4_FIFO_WR_EN_Pos)                 /**< USB ep4 write fifo 2 byte effective         */
#define LL_USB_EP4_FIFO_WEN_3BYTE                               (7UL << USB_EP4_FIFO_WR_EN_Pos)                 /**< USB ep4 write fifo 3 byte effective         */
#define LL_USB_EP4_FIFO_WEN_4BYTE                               (15UL << USB_EP4_FIFO_WR_EN_Pos)                /**< USB ep4 write fifo 4 byte effective         */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup USB_LL_Exported_Macros USB Exported Macros
  * @{
  */

/** @defgroup USB_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in USB register
  * @param  __instance__ USB instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_USB_WriteReg(__instance__, __REG__, __VALUE__)   WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in USB register
  * @param  __instance__ USB instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_USB_ReadReg(__instance__, __REG__)               READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup USB_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup USB_LL_EF_Configuration Configuration functions
  * @{
  */


/**
  * @brief  Enable USB Transceiver LDO_3.3V low power mode
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_TRX_LDO | LP_EN
  *
  * @param  AON_PMUx AON_PMU instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_xcvr_ldo33_lp(aon_pmu_regs_t *AON_PMUx)
{
    SET_BITS(AON_PMUx->USB_TRX_LDO, AON_PMU_USB_TRX_LDO_LP_EN);
}

/**
  * @brief  Disable USB Transceiver LDO_3.3V low power mode
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_TRX_LDO | LP_EN
  *
  * @param  AON_PMUx AON_PMU instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_xcvr_ldo33_lp(aon_pmu_regs_t *AON_PMUx)
{
    CLEAR_BITS(AON_PMUx->USB_TRX_LDO, AON_PMU_USB_TRX_LDO_LP_EN);
}

/**
  * @brief  Check if USB Transceiver LDO_3.3V low power mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_TRX_LDO | LP_EN
  *
  * @param  AON_PMUx AON_PMU instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_xcvr_ldo33_lp(aon_pmu_regs_t *AON_PMUx)
{
    return (READ_BITS(AON_PMUx->USB_TRX_LDO, AON_PMU_USB_TRX_LDO_LP_EN) == (AON_PMU_USB_TRX_LDO_LP_EN));
}

/**
  * @brief  Enable USB Transceiver LDO_3.3V internal vref probe to test
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_LDO | VREF_TEST_EN
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_xcvr_ldo33_vref_test(mcu_sub_regs_t *MCU_SUBx)
{
    SET_BITS(MCU_SUBx->USB_XCRV_LDO, MCU_SUB_USB_XCRV_LDO_VREF_TEST_EN);
}

/**
  * @brief  Disable USB Transceiver LDO_3.3V internal vref probe to test
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_LDO | VREF_TEST_EN
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_xcvr_ldo33_vref_test(mcu_sub_regs_t *MCU_SUBx)
{
    CLEAR_BITS(MCU_SUBx->USB_XCRV_LDO, MCU_SUB_USB_XCRV_LDO_VREF_TEST_EN);
}

/**
  * @brief  Check if USB Transceiver LDO_3.3V internal vref probe to test is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_LDO | VREF_TEST_EN
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_xcvr_ldo33_vref_test(mcu_sub_regs_t *MCU_SUBx)
{
    return (READ_BITS(MCU_SUBx->USB_XCRV_LDO, MCU_SUB_USB_XCRV_LDO_VREF_TEST_EN) == (MCU_SUB_USB_XCRV_LDO_VREF_TEST_EN));
}

/**
  * @brief  Set USB Transceiver LDO_3.3V vout trimming signal
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_TRX_LDO | BIAS_SEL
  *
  * @param  AON_PMUx AON_PMU instance
  * @param  value This parameter can be one of the following values:
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL0
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL1
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL2
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL3
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL4
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL5
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL6
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL7
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_xcrv_ldo33_bias(aon_pmu_regs_t *AON_PMUx, uint32_t value)
{
    MODIFY_REG(AON_PMUx->USB_TRX_LDO, AON_PMU_USB_TRX_LDO_BIAS_SEL, value);
}

/**
  * @brief  Get USB Transceiver LDO_3.3V vout trimming signal
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_TRX_LDO | BIAS_SEL
  *
  * @param  AON_PMUx AON_PMU instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL0
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL1
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL2
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL3
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL4
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL5
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL6
  *         @arg @ref LL_USB_TRX_LDO_BIAS_SEL7
  */
__STATIC_INLINE uint32_t ll_usb_get_xcrv_ldo33_bias(aon_pmu_regs_t *AON_PMUx)
{
    return (uint32_t)(READ_BITS(AON_PMUx->USB_TRX_LDO, AON_PMU_USB_TRX_LDO_BIAS_SEL));
}

/**
  * @brief  Set USB Transceiver LDO_3.3V offer XCVR's current
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_TRX_LDO | VSEL
  *
  * @param  AON_PMUx AON_PMU instance
  * @param  value This parameter can be one of the following values:
  *         @arg @ref LL_USB_TRX_LDO_VSEL0
  *         @arg @ref LL_USB_TRX_LDO_VSEL1
  *         @arg @ref LL_USB_TRX_LDO_VSEL2
  *         @arg @ref LL_USB_TRX_LDO_VSEL3
  *         @arg @ref LL_USB_TRX_LDO_VSEL4
  *         @arg @ref LL_USB_TRX_LDO_VSEL5
  *         @arg @ref LL_USB_TRX_LDO_VSEL6
  *         @arg @ref LL_USB_TRX_LDO_VSEL7
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_xcrv_ldo33_VSEL(aon_pmu_regs_t *AON_PMUx, uint32_t value)
{
    MODIFY_REG(AON_PMUx->USB_TRX_LDO, AON_PMU_USB_TRX_LDO_BIAS_SEL, value);
}

/**
  * @brief  Get USB Transceiver LDO_3.3V offer XCVR's current
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_TRX_LDO | VSEL
  *
  * @param  AON_PMUx AON_PMU instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_USB_TRX_LDO_VSEL0
  *         @arg @ref LL_USB_TRX_LDO_VSEL1
  *         @arg @ref LL_USB_TRX_LDO_VSEL2
  *         @arg @ref LL_USB_TRX_LDO_VSEL3
  *         @arg @ref LL_USB_TRX_LDO_VSEL4
  *         @arg @ref LL_USB_TRX_LDO_VSEL5
  *         @arg @ref LL_USB_TRX_LDO_VSEL6
  *         @arg @ref LL_USB_TRX_LDO_VSEL7
  */
__STATIC_INLINE uint32_t ll_usb_get_xcrv_ldo33_VSEL(aon_pmu_regs_t *AON_PMUx)
{
    return (uint32_t)(READ_BITS(AON_PMUx->USB_TRX_LDO, AON_PMU_USB_TRX_LDO_BIAS_SEL));
}


/**
  * @brief  Enable Turn off USB clock during USB suspend
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_LP_CTRL | SUSPEND_CLK_OFF
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_suspend_clk_off(mcu_sub_regs_t *MCU_SUBx)
{
    SET_BITS(MCU_SUBx->USB_LP_CTRL, MCU_SUB_USB_LP_CTRL_SUSPEND_CLK_OFF);
}

/**
  * @brief  Disable Turn off USB clock during USB suspend
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_LP_CTRL | SUSPEND_CLK_OFF
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_suspend_clk_off(mcu_sub_regs_t *MCU_SUBx)
{
    CLEAR_BITS(MCU_SUBx->USB_LP_CTRL, MCU_SUB_USB_LP_CTRL_SUSPEND_CLK_OFF);
}

/**
  * @brief  Check if Turn off USB clock during USB suspend is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_LP_CTRL | SUSPEND_CLK_OFF
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_suspend_clk_off(mcu_sub_regs_t *MCU_SUBx)
{
    return (READ_BITS(MCU_SUBx->USB_LP_CTRL, MCU_SUB_USB_LP_CTRL_SUSPEND_CLK_OFF) == (MCU_SUB_USB_LP_CTRL_SUSPEND_CLK_OFF));
}

/**
  * @brief  Enable USB low power
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_LP_CTRL | PMU_LP_EN
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_pmu_lp(mcu_sub_regs_t *MCU_SUBx)
{
    SET_BITS(MCU_SUBx->USB_LP_CTRL, MCU_SUB_USB_LP_CTRL_PMU_LP_EN);
}

/**
  * @brief  Disable USB low power
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_LP_CTRL | PMU_LP_EN
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_pmu_lp(mcu_sub_regs_t *MCU_SUBx)
{
    CLEAR_BITS(MCU_SUBx->USB_LP_CTRL, MCU_SUB_USB_LP_CTRL_PMU_LP_EN);
}

/**
  * @brief  Check if USB low power is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_LP_CTRL | PMU_LP_EN
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_pmu_lp(mcu_sub_regs_t *MCU_SUBx)
{
    return (READ_BITS(MCU_SUBx->USB_LP_CTRL, MCU_SUB_USB_LP_CTRL_PMU_LP_EN) == (MCU_SUB_USB_LP_CTRL_PMU_LP_EN));
}


/**
  * @brief  Enable USB clock force off
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_LP_CTRL | CLK_FORCE_OFF
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_clk_force_off(mcu_sub_regs_t *MCU_SUBx)
{
    SET_BITS(MCU_SUBx->USB_LP_CTRL, MCU_SUB_USB_LP_CTRL_CLK_FORCE_OFF);
}

/**
  * @brief  Disable USB clock force off
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_LP_CTRL | CLK_FORCE_OFF
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_clk_force_off(mcu_sub_regs_t *MCU_SUBx)
{
    CLEAR_BITS(MCU_SUBx->USB_LP_CTRL, MCU_SUB_USB_LP_CTRL_CLK_FORCE_OFF);
}

/**
  * @brief  Check if USB clock force off is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_LP_CTRL | CLK_FORCE_OFF
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_enabled_clk_force_off_is(mcu_sub_regs_t *MCU_SUBx)
{
    return (READ_BITS(MCU_SUBx->USB_LP_CTRL, MCU_SUB_USB_LP_CTRL_CLK_FORCE_OFF) == (MCU_SUB_USB_LP_CTRL_CLK_FORCE_OFF));
}

/**
  * @brief  Enable USB Transceiver single end comparator power down
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | SECMP_PD
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_xcvr_secmp_pd(mcu_sub_regs_t *MCU_SUBx)
{
    SET_BITS(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_SECMP_PD);
}

/**
  * @brief  Disable USB Transceiver single end comparator power down
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | SECMP_PD
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_xcvr_secmp_pd(mcu_sub_regs_t *MCU_SUBx)
{
    CLEAR_BITS(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_SECMP_PD);
}

/**
  * @brief  Check if USB Transceiver single end comparator power down is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | SECMP_PD
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_xcvr_secmp_pd(mcu_sub_regs_t *MCU_SUBx)
{
    return (READ_BITS(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_SECMP_PD) == (MCU_SUB_USB_XCRV_CTRL_SECMP_PD));
}

/**
  * @brief  Set USB Transceiver speed
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | SPEED
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @param  speed This parameter can be one of the following values:
  *         @arg @ref LL_USB_XCRV_CTRL_SPEED_LOW
  *         @arg @ref LL_USB_XCRV_CTRL_SPEED_FULL
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_xcrv_speed(mcu_sub_regs_t *MCU_SUBx, uint32_t speed)
{
    MODIFY_REG(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_SPEED, speed);
}

/**
  * @brief  Get USB Transceiver speed
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | SPEED
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_USB_XCRV_CTRL_SPEED_LOW
  *         @arg @ref LL_USB_XCRV_CTRL_SPEED_FULL
  */
__STATIC_INLINE uint32_t ll_usb_get_xcrv_speed(mcu_sub_regs_t *MCU_SUBx)
{
    return (uint32_t)(READ_BITS(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_SPEED));
}

/**
  * @brief  Enable USB Transceiver suspend
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | SUSPEND
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_xcvr_suspend(mcu_sub_regs_t *MCU_SUBx)
{
    SET_BITS(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_SUSPEND);
}

/**
  * @brief  Disable USB Transceiver suspend
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | SUSPEND
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_xcvr_suspend(mcu_sub_regs_t *MCU_SUBx)
{
    CLEAR_BITS(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_SUSPEND);
}

/**
  * @brief  Check if USB Transceiver suspend is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | SUSPEND
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_xcvr_suspend(mcu_sub_regs_t *MCU_SUBx)
{
    return (READ_BITS(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_SUSPEND) == (MCU_SUB_USB_XCRV_CTRL_SUSPEND));
}

/**
  * @brief  Enable USB Transceiver bias
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | BIAS_EN
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_xcvr_bias(mcu_sub_regs_t *MCU_SUBx)
{
    SET_BITS(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_BIAS_EN);
}

/**
  * @brief  Disable USB Transceiver bias
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | BIAS_EN
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_xcvr_bias(mcu_sub_regs_t *MCU_SUBx)
{
    CLEAR_BITS(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_BIAS_EN);
}

/**
  * @brief  Check if USB Transceiver bias is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | BIAS_EN
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_xcvr_bias(mcu_sub_regs_t *MCU_SUBx)
{
    return (READ_BITS(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_BIAS_EN) == (MCU_SUB_USB_XCRV_CTRL_BIAS_EN));
}

/**
  * @brief  Set USB Transceiver D- output impedance trim
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | RTRIMN
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @param  value This parameter can be one of the following values:
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN0
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN1
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN2
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN3
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN4
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN5
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN6
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN7
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_xcrv_rtrimn(mcu_sub_regs_t *MCU_SUBx, uint32_t value)
{
    MODIFY_REG(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_RTRIMN, value);
}

/**
  * @brief  Get USB Transceiver D- output impedance trim
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | RTRIMN
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN0
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN1
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN2
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN3
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN4
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN5
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN6
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMN7
  */
__STATIC_INLINE uint32_t ll_usb_get_xcrv_rtrimn(mcu_sub_regs_t *MCU_SUBx)
{
    return (uint32_t)(READ_BITS(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_RTRIMN));
}

/**
  * @brief  Set USB Transceiver D+ output impedance trim
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | RTRIMP
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @param  value This parameter can be one of the following values:
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP0
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP1
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP2
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP3
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP4
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP5
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP6
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP7
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_xcrv_rtrimp(mcu_sub_regs_t *MCU_SUBx, uint32_t value)
{
    MODIFY_REG(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_RTRIMP, value);
}

/**
  * @brief  Get USB Transceiver D+ output impedance trim
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_XCRV_CTRL | RTRIMP
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP0
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP1
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP2
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP3
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP4
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP5
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP6
  *         @arg @ref LL_USB_XCRV_CTRL_RTRIMP7
  */
__STATIC_INLINE uint32_t ll_usb_get_xcrv_rtrimp(mcu_sub_regs_t *MCU_SUBx)
{
    return (uint32_t)(READ_BITS(MCU_SUBx->USB_XCRV_CTRL, MCU_SUB_USB_XCRV_CTRL_RTRIMP));
}

/**
  * @brief  Enable USB software reset
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_SW_RST | usb_software_reset
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_sw_reset(mcu_sub_regs_t *MCU_SUBx)
{
    SET_BITS(MCU_SUBx->USB_SW_RST, MCU_SUB_USB_SW_RST_EN);
}

/**
  * @brief  Disable USB software reset
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_SW_RST | usb_software_reset
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_sw_reset(mcu_sub_regs_t *MCU_SUBx)
{
    CLEAR_BITS(MCU_SUBx->USB_SW_RST, MCU_SUB_USB_SW_RST_EN);
}

/**
  * @brief  Check if USB software reset is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  USB_SW_RST | usb_software_reset
  *
  * @param  MCU_SUBx MCU_SUB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_sw_reset(mcu_sub_regs_t *MCU_SUBx)
{
    return (READ_BITS(MCU_SUBx->USB_SW_RST, MCU_SUB_USB_SW_RST_EN) == (MCU_SUB_USB_SW_RST_EN));
}

/**
  * @brief  Set USB enumeration type
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | MCU_ENUM
  *
  * @param  USBx USB instance
  * @param  type This parameter can be one of the following values:
  *         @arg @ref  LL_USB_ENUM_TYPE_HW
  *         @arg @ref  LL_USB_ENUM_TYPE_MCU
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_usb_enum_type(usb_regs_t *USBx, uint32_t type)
{
    MODIFY_REG(USBx->CTRL, USB_CTRL_MCU_ENUM, type);
}

/**
  * @brief  Get USB enumeration type
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | MCU_ENUM
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref  LL_USB_ENUM_TYPE_HW
  *         @arg @ref  LL_USB_ENUM_TYPE_MCU
  */
__STATIC_INLINE uint32_t ll_usb_get_enum_type(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->CTRL, USB_CTRL_MCU_ENUM));
}

/**
  * @brief  Enable USB OUT data in EP0 is ready
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | EP0_OUT_DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep0_out_dat_rdy(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL, USB_CTRL_EP0_OUT_DATA_RDY);
}

/**
  * @brief  Disable USB OUT data in EP0 is ready
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | EP0_OUT_DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep0_out_dat_rdy(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL, USB_CTRL_EP0_OUT_DATA_RDY);
}

/**
  * @brief  Check if USB OUT data in EP0 is ready is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | EP0_OUT_DATA_RDY
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep0_out_dat_rdy(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL, USB_CTRL_EP0_OUT_DATA_RDY) == (USB_CTRL_EP0_OUT_DATA_RDY));
}

/**
  * @brief  Enable USB MCU remote wakeup USB host
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | MCU_WAKEUP
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_mcu_wakeup(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL, USB_CTRL_MCU_WAKEUP);
}

/**
  * @brief  Disable USB MCU remote wakeup USB host
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | MCU_WAKEUP
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_mcu_wakeup(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL, USB_CTRL_MCU_WAKEUP);
}


/**
  * @brief  Enable USB REMOTE_WAKEUP feature
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | DEV_REMOTE_WAKEUP
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_dev_remote_wakeup(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL, USB_CTRL_DEV_REMOTE_WAKEUP);
}

/**
  * @brief  Disable USB REMOTE_WAKEUP feature
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | DEV_REMOTE_WAKEUP
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_dev_remote_wakeup(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL, USB_CTRL_DEV_REMOTE_WAKEUP);
}

/**
  * @brief  Check if USB REMOTE_WAKEUP feature is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | DEV_REMOTE_WAKEUP
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_dev_remote_wakeup(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL, USB_CTRL_DEV_REMOTE_WAKEUP) == (USB_CTRL_DEV_REMOTE_WAKEUP));
}

/**
  * @brief  Enable USB is in addressed status
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | ADDR_STAT
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_addr_stat(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL, USB_CTRL_ADDR_STAT);
}

/**
  * @brief  Disable USB is in addressed status
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | ADDR_STAT
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_addr_stat(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL, USB_CTRL_ADDR_STAT);
}

/**
  * @brief  Check if USB is in addressed status is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | ADDR_STAT
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_addr_stat(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL, USB_CTRL_ADDR_STAT) == (USB_CTRL_ADDR_STAT));
}

/**
  * @brief  Enable USB is in addressed status
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | CFG_STAT
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_cfg_stat(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL, USB_CTRL_CFG_STAT);
}

/**
  * @brief  Disable USB is in addressed status
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | CFG_STAT
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_cfg_stat(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL, USB_CTRL_CFG_STAT);
}

/**
  * @brief  Check if USB is in addressed status is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | CFG_STAT
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_cfg_stat(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL, USB_CTRL_CFG_STAT) == (USB_CTRL_CFG_STAT));
}

/**
  * @brief  Enable MCU interpret USB device ok
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | CMD_OK
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_cmd_ok(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL, USB_CTRL_CMD_OK);
}

/**
  * @brief  Disable MCU interpret USB device ok
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | CMD_OK
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_cmd_ok(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL, USB_CTRL_CMD_OK);
}

/**
  * @brief  Enable MCU interpret USB device error
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | CMD_ERR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_cmd_err(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL, USB_CTRL_CMD_ERR);
}

/**
  * @brief  Disable MCU interpret USB device error
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | CMD_ERR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_cmd_err(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL, USB_CTRL_CMD_ERR);
}

/**
  * @brief  Set USB function address
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | FUNC_ADDR
  *
  * @param  USBx USB instance
  * @param  addr This parameter can be one of the following values:
  *         1 ~127
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_func_addr(usb_regs_t *USBx, uint32_t addr)
{
    MODIFY_REG(USBx->CTRL, USB_CTRL_FUNC_ADDR, addr << USB_CTRL_FUNC_ADDR_Pos);
}

/**
  * @brief  Get USB function address
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | FUNC_ADDR
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         1 ~127
  */
__STATIC_INLINE uint32_t ll_usb_get_func_addr(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->CTRL, USB_CTRL_FUNC_ADDR) >> USB_CTRL_FUNC_ADDR_Pos);
}

/**
  * @brief  Enable USB OUT data in EP1 is ready
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | EP1_OUT_DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep1_out_dat_rdy(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL, USB_CTRL_EP1_OUT_DATA_RDY);
}

/**
  * @brief  Disable USB OUT data in EP1 is ready
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | EP1_OUT_DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep1_out_dat_rdy(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL, USB_CTRL_EP1_OUT_DATA_RDY);
}

/**
  * @brief  Check if USB OUT data in EP1 is ready is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | EP1_OUT_DATA_RDY
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep1_out_dat_rdy(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL, USB_CTRL_EP1_OUT_DATA_RDY) == (USB_CTRL_EP1_OUT_DATA_RDY));
}

/**
  * @brief  Enable USB OUT data in EP5 is ready
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | EP5_OUT_DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep5_out_dat_rdy(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL, USB_CTRL_EP5_OUT_DATA_RDY);
}

/**
  * @brief  Disable USB OUT data in EP1 is ready
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | EP5_OUT_DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep5_out_dat_rdy(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL, USB_CTRL_EP5_OUT_DATA_RDY);
}

/**
  * @brief  Check if USB OUT data in EP1 is ready is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | EP5_OUT_DATA_RDY
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep5_out_dat_rdy(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL, USB_CTRL_EP5_OUT_DATA_RDY) == (USB_CTRL_EP5_OUT_DATA_RDY));
}

/**
  * @brief  Enable USB EP0 FIFO switch
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | EP0_FIFO_SWITCH
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep0_fifo_switch(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL, USB_CTRL_EP0_FIFO_SWITCH);
}

/**
  * @brief  Disable USB EP0 FIFO switch
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | EP0_FIFO_SWITCH
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep0_fifo_switch(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL, USB_CTRL_EP0_FIFO_SWITCH);
}

/**
  * @brief  Check if USB EP0 FIFO switch is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL    | EP0_FIFO_SWITCH
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep0_fifo_switch(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL, USB_CTRL_EP0_FIFO_SWITCH) == (USB_CTRL_EP0_FIFO_SWITCH));
}


/**
  * @brief  Enable USB has sent all data to EP0 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP0_1_CTRL | EP0_DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep0_dat_rdy(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP0_1_CTRL, USB_EP0_CTRL_DATA_RDY);
}

/**
  * @brief  Disable USB has sent all data to EP0 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP0_1_CTRL | EP0_DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep0_dat_rdy(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP0_1_CTRL, USB_EP0_CTRL_DATA_RDY);
}

/**
  * @brief  Check if USB has sent all data to EP0 IN FIFO is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP0_1_CTRL | EP0_DATA_RDY
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep0_dat_rdy(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP0_1_CTRL, USB_EP0_CTRL_DATA_RDY) == (USB_EP0_CTRL_DATA_RDY));
}

/**
  * @brief  Enable Clear USB EP0 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP0_1_CTRL | EP0_IFIFO_CLR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_clr_ep0_fifo(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP0_1_CTRL, USB_EP0_CTRL_IFIFO_CLR);
}

/**
  * @brief  Disable Clear USB EP0 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP0_1_CTRL | EP0_IFIFO_CLR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_clr_ep0_fifo(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP0_1_CTRL, USB_EP0_CTRL_IFIFO_CLR);
}

/**
  * @brief  Enable Clear USB EP1 OUT FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP0_1_CTRL | EP1_IFIFO_CLR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_clr_ep1_fifo(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP0_1_CTRL, USB_EP1_CTRL_IFIFO_CLR);
}

/**
  * @brief  Disable Clear USB EP1 OUT FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP0_1_CTRL | EP1_IFIFO_CLR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_clr_ep1_fifo(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP0_1_CTRL, USB_EP1_CTRL_IFIFO_CLR);
}

/**
  * @brief  Enable USB has sent all data to EP2 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP2_CTRL | DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep2_dat_rdy(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP2_CTRL, USB_EP2_CTRL_DATA_RDY);
}

/**
  * @brief  Disable USB has sent all data to EP2 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP2_CTRL | DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep2_dat_rdy(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP2_CTRL, USB_EP2_CTRL_DATA_RDY);
}

/**
  * @brief  Check if USB has sent all data to EP2 IN FIFO is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP2_CTRL | DATA_RDY
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep2_dat_rdy(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP2_CTRL, USB_EP2_CTRL_DATA_RDY) == (USB_EP2_CTRL_DATA_RDY));
}

/**
  * @brief  Enable Clear USB EP2 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP2_CTRL | IFIFO_CLR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_clr_ep2_fifo(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP2_CTRL, USB_EP2_CTRL_IFIFO_CLR);
}

/**
  * @brief  Disable Clear USB EP2 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP2_CTRL | IFIFO_CLR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_clr_ep2_fifo(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP2_CTRL, USB_EP2_CTRL_IFIFO_CLR);
}

/**
  * @brief  Enable USB has sent all data to EP3 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP3_CTRL | DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep3_dat_rdy(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP3_CTRL, USB_EP3_CTRL_DATA_RDY);
}

/**
  * @brief  Disable USB has sent all data to EP3 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP3_CTRL | DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep3_dat_rdy(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP3_CTRL, USB_EP3_CTRL_DATA_RDY);
}

/**
  * @brief  Check if USB has sent all data to EP3 IN FIFO is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP3_CTRL | DATA_RDY
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep3_dat_rdy(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP3_CTRL, USB_EP3_CTRL_DATA_RDY) == (USB_EP3_CTRL_DATA_RDY));
}

/**
  * @brief  Enable Clear USB EP3 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP3_CTRL | IFIFO_CLR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_clr_ep3_fifo(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP3_CTRL, USB_EP3_CTRL_IFIFO_CLR);
}

/**
  * @brief  Disable Clear USB EP3 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP3_CTRL | IFIFO_CLR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_clr_ep3_fifo(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP3_CTRL, USB_EP3_CTRL_IFIFO_CLR);
}

/**
  * @brief  Enable USB has sent all data to EP4 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_CTRL | DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep4_dat_rdy(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP4_CTRL, USB_EP4_CTRL_DATA_RDY);
}

/**
  * @brief  Disable USB has sent all data to EP4 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_CTRL | DATA_RDY
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep4_dat_rdy(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP4_CTRL, USB_EP4_CTRL_DATA_RDY);
}

/**
  * @brief  Check if USB has sent all data to EP4 IN FIFO is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_CTRL | DATA_RDY
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep4_dat_rdy(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP4_CTRL, USB_EP4_CTRL_DATA_RDY) == (USB_EP4_CTRL_DATA_RDY));
}

/**
  * @brief  Enable Clear USB EP4 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_CTRL | IFIFO_CLR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_clr_ep4_fifo(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP4_CTRL, USB_EP4_CTRL_IFIFO_CLR);
}

/**
  * @brief  Disable Clear USB EP4 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_CTRL | IFIFO_CLR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_clr_ep4_fifo(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP4_CTRL, USB_EP4_CTRL_IFIFO_CLR);
}

/**
  * @brief  Enable USB EP4 empty packet
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_CTRL | EMPTY_PACKET_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep4_empty_packet(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP4_CTRL, USB_EP4_CTRL_EMPTY_PACKET_EN);
}

/**
  * @brief  Disable USB EP4 empty packet
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_CTRL | EMPTY_PACKET_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep4_empty_packet(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP4_CTRL, USB_EP4_CTRL_EMPTY_PACKET_EN);
}

/**
  * @brief  Check if USB EP4 empty packet is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_CTRL | EMPTY_PACKET_EN
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep4_empty_packet(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP4_CTRL, USB_EP4_CTRL_EMPTY_PACKET_EN) == (USB_EP4_CTRL_EMPTY_PACKET_EN));
}

/**
  * @brief  Enable Clear USB EP4 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_CTRL | IFIFO_CLR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_clr_ep5_fifo(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP5_CTRL, USB_EP5_CTRL_FIFO_CLR);
}

/**
  * @brief  Disable Clear USB EP4 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_CTRL | IFIFO_CLR
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_clr_ep5_fifo(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP5_CTRL, USB_EP5_CTRL_FIFO_CLR);
}
/**
  * @brief  Set USB Endpoint 1 attribute
  *
  *  Register|BitsName
  *  --------|--------
  *  EP_ATTR | EP1
  *
  * @param  USBx USB instance
  * @param  attr This parameter can be one of the following values:
  *         @arg @ref LL_USB_EP_ATTR_EP1_INT
  *         @arg @ref LL_USB_EP_ATTR_EP1_ISO
  *         @arg @ref LL_USB_EP_ATTR_EP1_BULK
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep1_attr(usb_regs_t *USBx, uint32_t attr)
{
    MODIFY_REG(USBx->EP_ATTR, USB_EP_ATTR_EP1, attr);
}

/**
  * @brief  Get USB Endpoint 1 attribute
  *
  *  Register|BitsName
  *  --------|--------
  *  EP_ATTR | EP1
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_USB_EP_ATTR_EP1_INT
  *         @arg @ref LL_USB_EP_ATTR_EP1_ISO
  *         @arg @ref LL_USB_EP_ATTR_EP1_BULK
  */
__STATIC_INLINE uint32_t ll_usb_get_ep1_attr(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP_ATTR, USB_EP_ATTR_EP1));
}

/**
  * @brief  Set USB Endpoint 2 attribute
  *
  *  Register|BitsName
  *  --------|--------
  *  EP_ATTR | EP2
  *
  * @param  USBx USB instance
  * @param  attr This parameter can be one of the following values:
  *         @arg @ref LL_USB_EP_ATTR_EP2_INT
  *         @arg @ref LL_USB_EP_ATTR_EP2_ISO
  *         @arg @ref LL_USB_EP_ATTR_EP2_BULK
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep2_attr(usb_regs_t *USBx, uint32_t attr)
{
    MODIFY_REG(USBx->EP_ATTR, USB_EP_ATTR_EP2, attr);
}

/**
  * @brief  Get USB Endpoint 2 attribute
  *
  *  Register|BitsName
  *  --------|--------
  *  EP_ATTR | EP2
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_USB_EP_ATTR_EP2_INT
  *         @arg @ref LL_USB_EP_ATTR_EP2_ISO
  *         @arg @ref LL_USB_EP_ATTR_EP2_BULK
  */
__STATIC_INLINE uint32_t ll_usb_get_ep2_attr(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP_ATTR, USB_EP_ATTR_EP2));
}

/**
  * @brief  Set USB Endpoint 3 attribute
  *
  *  Register|BitsName
  *  --------|--------
  *  EP_ATTR | EP3
  *
  * @param  USBx USB instance
  * @param  attr This parameter can be one of the following values:
  *         @arg @ref LL_USB_EP_ATTR_EP3_INT
  *         @arg @ref LL_USB_EP_ATTR_EP3_ISO
  *         @arg @ref LL_USB_EP_ATTR_EP3_BULK
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep3_attr(usb_regs_t *USBx, uint32_t attr)
{
    MODIFY_REG(USBx->EP_ATTR, USB_EP_ATTR_EP3, attr);
}

/**
  * @brief  Get USB Endpoint 1 attribute
  *
  *  Register|BitsName
  *  --------|--------
  *  EP_ATTR | EP3
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_USB_EP_ATTR_EP3_INT
  *         @arg @ref LL_USB_EP_ATTR_EP3_ISO
  *         @arg @ref LL_USB_EP_ATTR_EP3_BULK
  */
__STATIC_INLINE uint32_t ll_usb_get_ep3_attr(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP_ATTR, USB_EP_ATTR_EP3));
}

/**
  * @brief  Enable USB halt EP1
  *
  *  Register|BitsName
  *  --------|--------
  * EP_ATTR | EP1_HALT_MCU
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep1_halt_mcu(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP_ATTR, USB_EP_ATTR_EP1_HALT_MCU);
}

/**
  * @brief  Disable USB halt EP1
  *
  *  Register|BitsName
  *  --------|--------
  * EP_ATTR | EP1_HALT_MCU
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep1_halt_mcu(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP_ATTR, USB_EP_ATTR_EP1_HALT_MCU);
}

/**
  * @brief  Check if USB halt EP1 is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP_ATTR | EP1_HALT_MCU
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep1_halt_mcu(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP_ATTR, USB_EP_ATTR_EP1_HALT_MCU) == (USB_EP_ATTR_EP1_HALT_MCU));
}

/**
  * @brief  Enable USB halt EP2
  *
  *  Register|BitsName
  *  --------|--------
  * EP_ATTR | EP2_HALT_MCU
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep2_halt_mcu(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP_ATTR, USB_EP_ATTR_EP2_HALT_MCU);
}

/**
  * @brief  Disable USB halt EP2
  *
  *  Register|BitsName
  *  --------|--------
  * EP_ATTR | EP2_HALT_MCU
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep2_halt_mcu(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP_ATTR, USB_EP_ATTR_EP2_HALT_MCU);
}

/**
  * @brief  Check if USB halt EP2 is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP_ATTR | EP2_HALT_MCU
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep2_halt_mcu(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP_ATTR, USB_EP_ATTR_EP2_HALT_MCU) == (USB_EP_ATTR_EP2_HALT_MCU));
}

/**
  * @brief  Enable USB halt EP3
  *
  *  Register|BitsName
  *  --------|--------
  * EP_ATTR | EP3_HALT_MCU
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep3_halt_mcu(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP_ATTR, USB_EP_ATTR_EP3_HALT_MCU);
}

/**
  * @brief  Disable USB halt EP3
  *
  *  Register|BitsName
  *  --------|--------
  * EP_ATTR | EP3_HALT_MCU
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep3_halt_mcu(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP_ATTR, USB_EP_ATTR_EP3_HALT_MCU);
}

/**
  * @brief  Check if USB halt EP3 is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP_ATTR | EP3_HALT_MCU
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep3_halt_mcu(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP_ATTR, USB_EP_ATTR_EP3_HALT_MCU) == (USB_EP_ATTR_EP3_HALT_MCU));
}

/**
  * @brief  Check USB interrupt flag is actived
  *
  *  Register|BitsName
  *  --------|--------
  *  INT_STAT | SUSPEND
  *  INT_STAT | EP0_OUT_READY
  *  INT_STAT | EP1_OUT_READY
  *  INT_STAT | CRC16_ERR
  *  INT_STAT | UPID_ERR
  *  INT_STAT | TIMEOUT_ERR
  *  INT_STAT | SEQ_ERR
  *  INT_STAT | PID_CKS_ERR
  *  INT_STAT | PID_CRC_ERR
  *  INT_STAT | HOST_RESET
  *  INT_STAT | AHB_XFER_ERR
  *  INT_STAT | NSE_ERR
  *  INT_STAT | EP3 AHB_XFER_DONE
  *  INT_STAT | SYNC_ERR
  *  INT_STAT | BIT_STUFF_ERR
  *  INT_STAT | BYTE_ERR
  *  INT_STAT | SOF
  *  INT_STAT | EP0_TX_DONE
  *  INT_STAT | EP2_TX_DONE
  *  INT_STAT | EP3_TX_DONE
  *  INT_STAT | INTO_CONFIG
  *  INT_STAT | EP5_OUT_READY
  *  INT_STAT | EP4_AHB_XFER_DONE
  *  INT_STAT | EP4_TX_DONE
  *  INT_STAT | EP5_AHB_XFER_DONE
  *  INT_STAT | EP5_TIMER_OUT_ERR
  *
  * @param  USBx USB instance
  * @param flag This parameter can be one or more of the following values:
  *         @arg @ref LL_USB_INT_STAT_SUSPEND
  *         @arg @ref LL_USB_INT_STAT_EP0_OUT_READY
  *         @arg @ref LL_USB_INT_STAT_EP1_OUT_READY
  *         @arg @ref LL_USB_INT_STAT_CRC16_ERR
  *         @arg @ref LL_USB_INT_STAT_UPID_ERR
  *         @arg @ref LL_USB_INT_STAT_TIMEOUT_ERR
  *         @arg @ref LL_USB_INT_STAT_SEQ_ERR
  *         @arg @ref LL_USB_INT_STAT_PID_CKS_ERR
  *         @arg @ref LL_USB_INT_STAT_PID_CRC_ERR
  *         @arg @ref LL_USB_INT_STAT_HOST_RESET
  *         @arg @ref LL_USB_INT_STAT_AHB_XFER_ERR
  *         @arg @ref LL_USB_INT_STAT_NSE_ERR
  *         @arg @ref LL_USB_INT_STAT_EP3_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_STAT_SYNC_ERR
  *         @arg @ref LL_USB_INT_STAT_BIT_STUFF_ERR
  *         @arg @ref LL_USB_INT_STAT_BYTE_ERR
  *         @arg @ref LL_USB_INT_STAT_SOF
  *         @arg @ref LL_USB_INT_STAT_EP0_TX_DONE
  *         @arg @ref LL_USB_INT_STAT_EP2_TX_DONE
  *         @arg @ref LL_USB_INT_STAT_EP3_TX_DONE
  *         @arg @ref LL_USB_INT_STAT_INTO_CONFIG
  *         @arg @ref LL_USB_INT_STAT_EP5_OUT_READY
  *         @arg @ref LL_USB_INT_STAT_EP4_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_STAT_EP4_TX_DONE
  *         @arg @ref LL_USB_INT_STAT_EP5_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_STAT_EP5_TIMER_OUT_ERR
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_active_it_flag(usb_regs_t *USBx, uint32_t flag)
{
    return (uint32_t)(READ_BITS(USBx->INT_STAT, flag) == flag);
}

/**
  * @brief  get USB interrupt flag
  *
  *  Register|BitsName
  *  --------|--------
  *  INT_STAT | SUSPEND
  *  INT_STAT | EP0_OUT_READY
  *  INT_STAT | EP1_OUT_READY
  *  INT_STAT | CRC16_ERR
  *  INT_STAT | UPID_ERR
  *  INT_STAT | TIMEOUT_ERR
  *  INT_STAT | SEQ_ERR
  *  INT_STAT | PID_CKS_ERR
  *  INT_STAT | PID_CRC_ERR
  *  INT_STAT | HOST_RESET
  *  INT_STAT | AHB_XFER_ERR
  *  INT_STAT | NSE_ERR
  *  INT_STAT | EP3_AHB_XFER_DONE
  *  INT_STAT | SYNC_ERR
  *  INT_STAT | BIT_STUFF_ERR
  *  INT_STAT | BYTE_ERR
  *  INT_STAT | SOF
  *  INT_STAT | EP0_TX_DONE
  *  INT_STAT | EP2_TX_DONE
  *  INT_STAT | EP3_TX_DONE
  *  INT_STAT | INTO_CONFIG
  *  INT_STAT | EP5_OUT_READY
  *  INT_STAT | EP4_AHB_XFER_DONE
  *  INT_STAT | EP4_TX_DONE
  *  INT_STAT | EP5_AHB_XFER_DONE
  *  INT_STAT | EP5_TIMER_OUT_ERR
  *
  * @param  USBx USB instance
  * @retval intterupt status register value.
  */
__STATIC_INLINE uint32_t ll_usb_get_it_flag(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->INT_STAT, LL_USB_INT_STAT_ALL));
}

/**
  * @brief  Enable USB interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  INT_EN | SUSPEND
  *  INT_EN | EP0_OUT_READY
  *  INT_EN | EP1_OUT_READY
  *  INT_EN | CRC16_ERR
  *  INT_EN | UPID_ERR
  *  INT_EN | TIMEOUT_ERR
  *  INT_EN | SEQ_ERR
  *  INT_EN | PID_CKS_ERR
  *  INT_EN | PID_CRC_ERR
  *  INT_EN | HOST_RESET
  *  INT_EN | AHB_XFER_ERR
  *  INT_EN | NSE_ERR
  *  INT_EN | EP3_AHB_XFER_DONE
  *  INT_EN | SYNC_ERR
  *  INT_EN | BIT_STUFF_ERR
  *  INT_EN | BYTE_ERR
  *  INT_EN | SOF
  *  INT_EN | EP0_TX_DONE
  *  INT_EN | EP2_TX_DONE
  *  INT_EN | EP3_TX_DONE
  *  INT_EN | INTO_CONFIG
  *  INT_EN | EP5_OUT_READY
  *  INT_EN | EP4_AHB_XFER_DONE
  *  INT_EN | EP4_TX_DONE
  *  INT_EN | EP5_AHB_XFER_DONE
  *  INT_EN | EP5_TIMER_OUT_ERR
  *
  * @param  USBx USB instance
  * @param  mask This parameter can be one or more of the following values:
  *         @arg @ref LL_USB_INT_EN_ALL
  *         @arg @ref LL_USB_INT_RESET_VAL
  *         @arg @ref LL_USB_INT_EN_SUSPEND
  *         @arg @ref LL_USB_INT_EN_EP0_OUT_READY
  *         @arg @ref LL_USB_INT_EN_EP1_OUT_READY
  *         @arg @ref LL_USB_INT_EN_CRC16_ERR
  *         @arg @ref LL_USB_INT_EN_UPID_ERR
  *         @arg @ref LL_USB_INT_EN_TIMEOUT_ERR
  *         @arg @ref LL_USB_INT_EN_SEQ_ERR
  *         @arg @ref LL_USB_INT_EN_PID_CKS_ERR
  *         @arg @ref LL_USB_INT_EN_PID_CRC_ERR
  *         @arg @ref LL_USB_INT_EN_HOST_RESET
  *         @arg @ref LL_USB_INT_EN_AHB_XFER_ERR
  *         @arg @ref LL_USB_INT_EN_NSE_ERR
  *         @arg @ref LL_USB_INT_EN_EP3_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_EN_SYNC_ERR
  *         @arg @ref LL_USB_INT_EN_BIT_STUFF_ERR
  *         @arg @ref LL_USB_INT_EN_BYTE_ERR
  *         @arg @ref LL_USB_INT_EN_SOF
  *         @arg @ref LL_USB_INT_EN_EP0_TX_DONE
  *         @arg @ref LL_USB_INT_EN_EP2_TX_DONE
  *         @arg @ref LL_USB_INT_EN_EP3_TX_DONE
  *         @arg @ref LL_USB_INT_EN_INTO_CONFIG
  *         @arg @ref LL_USB_INT_EN_EP5_OUT_READY
  *         @arg @ref LL_USB_INT_EN_EP4_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_EN_EP4_TX_DONE
  *         @arg @ref LL_USB_INT_EN_EP5_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_EN_EP5_TIMER_OUT_ERR
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_it(usb_regs_t *USBx, uint32_t mask)
{
    SET_BITS(USBx->INT_EN, mask);
}

/**
  * @brief  Disable USB interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  INT_EN | SUSPEND
  *  INT_EN | EP0_OUT_READY
  *  INT_EN | EP1_OUT_READY
  *  INT_EN | CRC16_ERR
  *  INT_EN | UPID_ERR
  *  INT_EN | TIMEOUT_ERR
  *  INT_EN | SEQ_ERR
  *  INT_EN | PID_CKS_ERR
  *  INT_EN | PID_CRC_ERR
  *  INT_EN | HOST_RESET
  *  INT_EN | AHB_XFER_ERR
  *  INT_EN | NSE_ERR
  *  INT_EN | EP3_AHB_XFER_DONE
  *  INT_EN | SYNC_ERR
  *  INT_EN | BIT_STUFF_ERR
  *  INT_EN | BYTE_ERR
  *  INT_EN | SOF
  *  INT_EN | EP0_TX_DONE
  *  INT_EN | EP2_TX_DONE
  *  INT_EN | EP3_TX_DONE
  *  INT_EN | INTO_CONFIG
  *  INT_EN | EP5_OUT_READY
  *  INT_EN | EP4_AHB_XFER_DONE
  *  INT_EN | EP4_TX_DONE
  *  INT_EN | EP5_AHB_XFER_DONE
  *  INT_EN | EP5_TIMER_OUT_ERR
  *
  * @param  USBx USB instance
  * @param  mask This parameter can be one or more of the following values:
  *         @arg @ref LL_USB_INT_EN_ALL
  *         @arg @ref LL_USB_INT_RESET_VAL
  *         @arg @ref LL_USB_INT_EN_SUSPEND
  *         @arg @ref LL_USB_INT_EN_EP0_OUT_READY
  *         @arg @ref LL_USB_INT_EN_EP1_OUT_READY
  *         @arg @ref LL_USB_INT_EN_CRC16_ERR
  *         @arg @ref LL_USB_INT_EN_UPID_ERR
  *         @arg @ref LL_USB_INT_EN_TIMEOUT_ERR
  *         @arg @ref LL_USB_INT_EN_SEQ_ERR
  *         @arg @ref LL_USB_INT_EN_PID_CKS_ERR
  *         @arg @ref LL_USB_INT_EN_PID_CRC_ERR
  *         @arg @ref LL_USB_INT_EN_HOST_RESET
  *         @arg @ref LL_USB_INT_EN_AHB_XFER_ERR
  *         @arg @ref LL_USB_INT_EN_NSE_ERR
  *         @arg @ref LL_USB_INT_EN_EP3_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_EN_SYNC_ERR
  *         @arg @ref LL_USB_INT_EN_BIT_STUFF_ERR
  *         @arg @ref LL_USB_INT_EN_BYTE_ERR
  *         @arg @ref LL_USB_INT_EN_SOF
  *         @arg @ref LL_USB_INT_EN_EP0_TX_DONE
  *         @arg @ref LL_USB_INT_EN_EP2_TX_DONE
  *         @arg @ref LL_USB_INT_EN_EP3_TX_DONE
  *         @arg @ref LL_USB_INT_EN_INTO_CONFIG
  *         @arg @ref LL_USB_INT_EN_EP5_OUT_READY
  *         @arg @ref LL_USB_INT_EN_EP4_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_EN_EP4_TX_DONE
  *         @arg @ref LL_USB_INT_EN_EP5_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_EN_EP5_TIMER_OUT_ERR
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_it(usb_regs_t *USBx, uint32_t mask)
{
    CLEAR_BITS(USBx->INT_EN, mask);
}

/**
  * @brief  Check if USB interrupt is enabled
  *
  *  Register|BitsName
  *  --------|--------
  *  INT_EN | SUSPEND
  *  INT_EN | EP0_OUT_READY
  *  INT_EN | EP1_OUT_READY
  *  INT_EN | CRC16_ERR
  *  INT_EN | UPID_ERR
  *  INT_EN | TIMEOUT_ERR
  *  INT_EN | SEQ_ERR
  *  INT_EN | PID_CKS_ERR
  *  INT_EN | PID_CRC_ERR
  *  INT_EN | HOST_RESET
  *  INT_EN | AHB_XFER_ERR
  *  INT_EN | NSE_ERR
  *  INT_EN | EP3_AHB_XFER_DONE
  *  INT_EN | SYNC_ERR
  *  INT_EN | BIT_STUFF_ERR
  *  INT_EN | BYTE_ERR
  *  INT_EN | SOF
  *  INT_EN | EP0_TX_DONE
  *  INT_EN | EP2_TX_DONE
  *  INT_EN | EP3_TX_DONE
  *  INT_EN | INTO_CONFIG
  *  INT_EN | EP5_OUT_READY
  *  INT_EN | EP4_AHB_XFER_DONE
  *  INT_EN | EP4_TX_DONE
  *  INT_EN | EP5_AHB_XFER_DONE
  *  INT_EN | EP5_TIMER_OUT_ERR
  *
  * @param  USBx USB instance
  * @param  mask This parameter can be one or more of the following values:
  *         @arg @ref LL_USB_INT_EN_ALL
  *         @arg @ref LL_USB_INT_RESET_VAL
  *         @arg @ref LL_USB_INT_EN_SUSPEND
  *         @arg @ref LL_USB_INT_EN_EP0_OUT_READY
  *         @arg @ref LL_USB_INT_EN_EP1_OUT_READY
  *         @arg @ref LL_USB_INT_EN_CRC16_ERR
  *         @arg @ref LL_USB_INT_EN_UPID_ERR
  *         @arg @ref LL_USB_INT_EN_TIMEOUT_ERR
  *         @arg @ref LL_USB_INT_EN_SEQ_ERR
  *         @arg @ref LL_USB_INT_EN_PID_CKS_ERR
  *         @arg @ref LL_USB_INT_EN_PID_CRC_ERR
  *         @arg @ref LL_USB_INT_EN_HOST_RESET
  *         @arg @ref LL_USB_INT_EN_AHB_XFER_ERR
  *         @arg @ref LL_USB_INT_EN_NSE_ERR
  *         @arg @ref LL_USB_INT_EN_EP3_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_EN_SYNC_ERR
  *         @arg @ref LL_USB_INT_EN_BIT_STUFF_ERR
  *         @arg @ref LL_USB_INT_EN_BYTE_ERR
  *         @arg @ref LL_USB_INT_EN_SOF
  *         @arg @ref LL_USB_INT_EN_EP0_TX_DONE
  *         @arg @ref LL_USB_INT_EN_EP2_TX_DONE
  *         @arg @ref LL_USB_INT_EN_EP3_TX_DONE
  *         @arg @ref LL_USB_INT_EN_INTO_CONFIG
  *         @arg @ref LL_USB_INT_EN_EP5_OUT_READY
  *         @arg @ref LL_USB_INT_EN_EP4_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_EN_EP4_TX_DONE
  *         @arg @ref LL_USB_INT_EN_EP5_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_EN_EP5_TIMER_OUT_ERR
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_it(usb_regs_t *USBx, uint32_t mask)
{
    return (READ_BITS(USBx->INT_EN, mask) == (mask));
}

/**
  * @brief  Clear USB interrupt
  *
  *  Register|BitsName
  *  --------|--------
  *  INT_CLR | SUSPEND
  *  INT_CLR | EP0_OUT_READY
  *  INT_CLR | EP1_OUT_READY
  *  INT_CLR | CRC16_ERR
  *  INT_CLR | UPID_ERR
  *  INT_CLR | TIMEOUT_ERR
  *  INT_CLR | SEQ_ERR
  *  INT_CLR | PID_CKS_ERR
  *  INT_CLR | PID_CRC_ERR
  *  INT_CLR | HOST_RESET
  *  INT_CLR | AHB_XFER_ERR
  *  INT_CLR | NSE_ERR
  *  INT_CLR | EP3_AHB_XFER_DONE
  *  INT_CLR | SYNC_ERR
  *  INT_CLR | BIT_STUFF_ERR
  *  INT_CLR | BYTE_ERR
  *  INT_CLR | SOF
  *  INT_CLR | EP0_TX_DONE
  *  INT_CLR | EP2_TX_DONE
  *  INT_CLR | EP3_TX_DONE
  *  INT_CLR | INTO_CONFIG
  *  INT_CLR | EP5_OUT_READY
  *  INT_CLR | EP4_AHB_XFER_DONE
  *  INT_CLR | EP4_TX_DONE
  *  INT_CLR | EP5_AHB_XFER_DONE
  *  INT_CLR | EP5_TIMER_OUT_ERR
  *
  * @param   USBx USB instance
  * @param  mask This parameter can be one or more of the following values:
  *         @arg @ref LL_USB_INT_CLR_ALL
  *         @arg @ref LL_USB_INT_CLR_SUSPEND
  *         @arg @ref LL_USB_INT_CLR_EP0_OUT_READY
  *         @arg @ref LL_USB_INT_CLR_EP1_OUT_READY
  *         @arg @ref LL_USB_INT_CLR_CRC16_ERR
  *         @arg @ref LL_USB_INT_CLR_UPID_ERR
  *         @arg @ref LL_USB_INT_CLR_TIMEOUT_ERR
  *         @arg @ref LL_USB_INT_CLR_SEQ_ERR
  *         @arg @ref LL_USB_INT_CLR_PID_CKS_ERR
  *         @arg @ref LL_USB_INT_CLR_PID_CRC_ERR
  *         @arg @ref LL_USB_INT_CLR_HOST_RESET
  *         @arg @ref LL_USB_INT_CLR_AHB_XFER_ERR
  *         @arg @ref LL_USB_INT_CLR_NSE_ERR
  *         @arg @ref LL_USB_INT_CLR_EP3_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_CLR_SYNC_ERR
  *         @arg @ref LL_USB_INT_CLR_BIT_STUFF_ERR
  *         @arg @ref LL_USB_INT_CLR_BYTE_ERR
  *         @arg @ref LL_USB_INT_CLR_SOF
  *         @arg @ref LL_USB_INT_CLR_EP0_TX_DONE
  *         @arg @ref LL_USB_INT_CLR_EP2_TX_DONE
  *         @arg @ref LL_USB_INT_CLR_EP3_TX_DONE
  *         @arg @ref LL_USB_INT_CLR_INTO_CONFIG
  *         @arg @ref LL_USB_INT_CLR_EP5_OUT_READY
  *         @arg @ref LL_USB_INT_CLR_EP4_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_CLR_EP4_TX_DONE
  *         @arg @ref LL_USB_INT_CLR_EP5_AHB_XFER_DONE
  *         @arg @ref LL_USB_INT_CLR_EP5_TIMER_OUT_ERR
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE void ll_usb_clear_it(usb_regs_t *USBx, uint32_t mask)
{
    SET_BITS(USBx->INT_CLR, mask);
}

/**
  * @brief  Set USB ep3 AHB master read start address
  *
  *  Register|BitsName
  *  --------|--------
  *  EP3_AHBM_RADDR | RD_START_ADDR
  *
  * @param  USBx USB instance
  * @param  addr This parameter can be one of the following values:
  *         32 bit address
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep3_ahb_m_rd_start_addr(usb_regs_t *USBx, uint32_t addr)
{
    MODIFY_REG(USBx->EP3_AHBM_RADDR, USB_EP3_AHBM_RADDR_RD_START_ADDR, addr);
}

/**
  * @brief  Get USB ep3 AHB master read start address
  *
  *  Register|BitsName
  *  --------|--------
  *  EP3_AHBM_RADDR | RD_START_ADDR
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         32 bit address
  */
__STATIC_INLINE uint32_t ll_usb_get_ep3_ahb_m_rd_start_addr(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP3_AHBM_RADDR, USB_EP3_AHBM_RADDR_RD_START_ADDR));
}

/**
  * @brief  Enable USB ep3 AHB master
  *
  *  Register|BitsName
  *  --------|--------
  * EP3_AHBM_CTRL | EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep3_ahb_m(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP3_AHBM_CTRL, USB_EP3_AHBM_CTRL_EN);
}

/**
  * @brief  Disable USB ep3 AHB master
  *
  *  Register|BitsName
  *  --------|--------
  * EP3_AHBM_CTRL | EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep3_ahb_m(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP3_AHBM_CTRL, USB_EP3_AHBM_CTRL_EN);
}

/**
  * @brief  Check if USB ep3 AHB master is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP3_AHBM_CTRL | EN
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep3_ahb_m(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP3_AHBM_CTRL, USB_EP3_AHBM_CTRL_EN) == (USB_EP3_AHBM_CTRL_EN));
}

/**
  * @brief  Set USB ep3 AHB master burst length;(default 64)
  *
  *  Register|BitsName
  *  --------|--------
  *  EP3_AHBM_CTRL | BURST_SIZE
  *
  * @param  USBx USB instance
  * @param  size This parameter can be one of the following values:
  *         0x0 ~ 0xFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep3_ahb_m_burst_size(usb_regs_t *USBx, uint32_t size)
{
    MODIFY_REG(USBx->EP3_AHBM_CTRL, USB_EP3_AHBM_CTRL_BURST_SIZE, size << USB_EP3_AHBM_CTRL_BURST_SIZE_Pos);
}

/**
  * @brief  Get USB ep3 AHB master burst length;(default 64)
  *
  *  Register|BitsName
  *  --------|--------
  *  EP3_AHBM_CTRL | BURST_SIZE
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_ep3_ahb_m_burst_size(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP3_AHBM_CTRL, USB_EP3_AHBM_CTRL_BURST_SIZE) >> USB_EP3_AHBM_CTRL_BURST_SIZE_Pos);
}

/**
  * @brief  Set USB ep4 AHB master read start address
  *
  *  Register|BitsName
  *  --------|--------
  *  EP4_AHBM_RADDR | RD_START_ADDR
  *
  * @param  USBx USB instance
  * @param  addr This parameter can be one of the following values:
  *         32 bit address
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep4_ahb_m_rd_start_addr(usb_regs_t *USBx, uint32_t addr)
{
    MODIFY_REG(USBx->EP4_AHBM_RADDR, USB_EP4_AHBM_RADDR_RD_START_ADDR, addr);
}

/**
  * @brief  Get USB ep4 AHB master read start address
  *
  *  Register|BitsName
  *  --------|--------
  *  EP4_AHBM_RADDR | RD_START_ADDR
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         32 bit address
  */
__STATIC_INLINE uint32_t ll_usb_get_ep4_ahb_m_rd_start_addr(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP4_AHBM_RADDR, USB_EP4_AHBM_RADDR_RD_START_ADDR));
}

/**
  * @brief  Enable USB ep4 AHB master
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_AHBM_CTRL | EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep4_ahb_m(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP4_AHBM_CTRL, USB_EP4_AHBM_CTRL_EN);
}

/**
  * @brief  Disable USB ep4 AHB master
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_AHBM_CTRL | EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep4_ahb_m(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP4_AHBM_CTRL, USB_EP4_AHBM_CTRL_EN);
}

/**
  * @brief  Check if USB ep4 AHB master is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP4_AHBM_CTRL | EN
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep4_ahb_m(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP4_AHBM_CTRL, USB_EP4_AHBM_CTRL_EN) == (USB_EP4_AHBM_CTRL_EN));
}

/**
  * @brief  Set USB ep4 AHB master burst length;(default 64)
  *
  *  Register|BitsName
  *  --------|--------
  *  EP4_AHBM_CTRL | BURST_SIZE
  *
  * @param  USBx USB instance
  * @param  size This parameter can be one of the following values:
  *         32 ~ 1023
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep4_ahb_m_burst_size(usb_regs_t *USBx, uint32_t size)
{
    MODIFY_REG(USBx->EP4_AHBM_CTRL, USB_EP4_AHBM_CTRL_BURST_SIZE, size << USB_EP4_AHBM_CTRL_BURST_SIZE_Pos);
}

/**
  * @brief  Get USB ep4 AHB master burst length;(default 64)
  *
  *  Register|BitsName
  *  --------|--------
  *  EP4_AHBM_CTRL | BURST_SIZE
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         32 ~ 1023
  */
__STATIC_INLINE uint32_t ll_usb_get_ep4_ahb_m_burst_size(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP4_AHBM_CTRL, USB_EP4_AHBM_CTRL_BURST_SIZE) >> USB_EP4_AHBM_CTRL_BURST_SIZE_Pos);
}

/**
  * @brief  Set USB ep5 AHB master read start address
  *
  *  Register|BitsName
  *  --------|--------
  *  EP5_AHBM_RADDR | RD_START_ADDR
  *
  * @param  USBx USB instance
  * @param  addr This parameter can be one of the following values:
  *         32 bit address
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep5_ahb_m_rd_start_addr(usb_regs_t *USBx, uint32_t addr)
{
    MODIFY_REG(USBx->EP5_AHBM_RADDR, USB_EP5_AHBM_RADDR_RD_START_ADDR, addr);
}

/**
  * @brief  Get USB ep5 AHB master read start address
  *
  *  Register|BitsName
  *  --------|--------
  *  EP5_AHBM_RADDR | RD_START_ADDR
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         32 bit address
  */
__STATIC_INLINE uint32_t ll_usb_get_ep5_ahb_m_rd_start_addr(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP5_AHBM_RADDR, USB_EP5_AHBM_RADDR_RD_START_ADDR));
}

/**
  * @brief  Enable USB ep5 AHB master
  *
  *  Register|BitsName
  *  --------|--------
  * EP5_CTRL | AHBM_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep5_ahb_m(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP5_CTRL, USB_EP5_CTRL_AHBM_EN);
}

/**
  * @brief  Disable USB ep5 AHB master
  *
  *  Register|BitsName
  *  --------|--------
  * EP5_CTRL | AHBM_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep5_ahb_m(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP5_CTRL, USB_EP5_CTRL_AHBM_EN);
}

/**
  * @brief  Check if USB ep5 AHB master is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP5_CTRL | AHBM_EN
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep5_ahb_m(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP5_CTRL, USB_EP5_CTRL_AHBM_EN) == (USB_EP5_CTRL_AHBM_EN));
}

/**
  * @brief  Enable USB ep5 OUT FIFO clear
  *
  *  Register|BitsName
  *  --------|--------
  * EP5_CTRL | AHBM_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep5_fifo_clr(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP5_CTRL, USB_EP5_CTRL_FIFO_CLR);
}

/**
  * @brief  Disable USB ep5 OUT FIFO clear
  *
  *  Register|BitsName
  *  --------|--------
  * EP5_CTRL | AHBM_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep5_fifo_clr(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP5_CTRL, USB_EP5_CTRL_FIFO_CLR);
}

/**
  * @brief  Check if USB ep5 OUT FIFO clear is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP5_CTRL | AHBM_EN
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep5_fifo_clr(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP5_CTRL, USB_EP5_CTRL_FIFO_CLR) == (USB_EP5_CTRL_FIFO_CLR));
}

/**
  * @brief USB ep5 RX data sum can't be overwritten
  *
  *  Register|BitsName
  *  --------|--------
  * EP5_CTRL | RX_CNT_NO_OVERWRITE
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_ep5_rx_cnt_no_overwrite(usb_regs_t *USBx)
{
    SET_BITS(USBx->EP5_CTRL, USB_EP5_CTRL_RX_CNT_NO_OVERWRITE);
}

/**
  * @brief  USB ep5 RX data sum can be overwritten
  *
  *  Register|BitsName
  *  --------|--------
  * EP5_CTRL | RX_CNT_NO_OVERWRITE
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_ep5_rx_cnt_no_overwrite(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->EP5_CTRL, USB_EP5_CTRL_RX_CNT_NO_OVERWRITE);
}

/**
  * @brief  Check if USB ep5 RX data sum can't be overwritten is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * EP5_CTRL | RX_CNT_NO_OVERWRITE
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_ep5_rx_cnt_no_overwrite(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->EP5_CTRL, USB_EP5_CTRL_RX_CNT_NO_OVERWRITE) == (USB_EP5_CTRL_RX_CNT_NO_OVERWRITE));
}

/**
  * @brief  Enable USB test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_test_mode(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL0, USB_CTRL0_TEST_MODE);
}

/**
  * @brief  Disable USB test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_test_mode(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL0, USB_CTRL0_TEST_MODE);
}

/**
  * @brief  Check if USB test mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | EN
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_test_mode(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL0, USB_CTRL0_TEST_MODE) == (USB_CTRL0_TEST_MODE));
}

/**
  * @brief  Enable USB drive DP during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | DRIVE_DP
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_drive_dp(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL0, USB_CTRL0_DRIVE_DP);
}

/**
  * @brief  Disable USB drive DP during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | DRIVE_DP
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_drive_dp(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL0, USB_CTRL0_DRIVE_DP);
}

/**
  * @brief  Check if USB tdrive DP during transceiver test mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | DRIVE_DP
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_drive_dp(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL0, USB_CTRL0_DRIVE_DP) == (USB_CTRL0_DRIVE_DP));
}

/**
  * @brief  Enable USB drive DM during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | DRIVE_DM
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_drive_dm(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL0, USB_CTRL0_DRIVE_DM);
}

/**
  * @brief  Disable USB drive DM during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | DRIVE_DM
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_drive_dm(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL0, USB_CTRL0_DRIVE_DM);
}

/**
  * @brief  Check if USB tdrive DM during transceiver test mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | DRIVE_DP
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_drive_dm(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL0, USB_CTRL0_DRIVE_DM) == (USB_CTRL0_DRIVE_DM));
}

/**
  * @brief  Enable USB drive transceiver output enable bar during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_OEB
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_xcvr_oeb(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL0, USB_CTRL0_XCVR_OEB);
}

/**
  * @brief  Disable USB drive transceiver output enable bar during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_OEB
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_xcvr_oeb(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL0, USB_CTRL0_XCVR_OEB);
}

/**
  * @brief  Check if USB drive transceiver output enable bar during transceiver test mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_OEB
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_xcvr_oeb(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL0, USB_CTRL0_XCVR_OEB) == (USB_CTRL0_XCVR_OEB));
}

/**
  * @brief  Enable USB drive DP pullup during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DP_RPU_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_xcvr_dp_rpu(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DP_RPU_EN);
}

/**
  * @brief  Disable USB drive DP pullup during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DP_RPU_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_xcvr_dp_rpu(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DP_RPU_EN);
}

/**
  * @brief  Check if USB drive DP pullup during transceiver test mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DP_RPU_EN
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_xcvr_dp_rpu(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DP_RPU_EN) == (USB_CTRL0_XCVR_DP_RPU_EN));
}

/**
  * @brief  Enable USB drive DM pullup during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DM_RPU_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_xcvr_dm_rpu(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DM_RPU_EN);
}

/**
  * @brief  Disable USB drive DM pullup during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DM_RPU_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_xcvr_dm_rpu(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DM_RPU_EN);
}

/**
  * @brief  Check if USB drive DM pullup during transceiver test mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DM_RPU_EN
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_xcvr_dm_rpu(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DM_RPU_EN) == (USB_CTRL0_XCVR_DM_RPU_EN));
}

/**
  * @brief  Enable USB drive DP SW pullup during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DP_RPUSW_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_xcvr_dp_rpusw(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DP_RPUSW_EN);
}

/**
  * @brief  Disable USB drive DP SW pullup during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DP_RPUSW_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_xcvr_dp_rpusw(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DP_RPUSW_EN);
}

/**
  * @brief  Check if USB drive DP SW pullup during transceiver test mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DP_RPUSW_EN
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_xcvr_dp_rpusw(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DP_RPUSW_EN) == (USB_CTRL0_XCVR_DP_RPUSW_EN));
}

/**
  * @brief  Enable USB drive DM SW pullup during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DM_RPUSW_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_xcvr_dm_rpusw(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DM_RPUSW_EN);
}

/**
  * @brief  Disable USB drive DM SW pullup during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DM_RPUSW_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_xcvr_dm_rpusw(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DM_RPUSW_EN);
}

/**
  * @brief  Check if USB drive DM SW pullup during transceiver test mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DM_RPUSW_EN
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_xcvr_dm_rpusw(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DM_RPUSW_EN) == (USB_CTRL0_XCVR_DM_RPUSW_EN));
}

/**
  * @brief  Enable USB drive DP pull down during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DP_RPD_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_xcvr_dp_rpd(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DP_RPD_EN);
}

/**
  * @brief  Disable USB drive DP pull down during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DP_RPD_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_xcvr_dp_rpd(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DP_RPD_EN);
}

/**
  * @brief  Check if USB drive DP pull down during transceiver test mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DP_RPD_EN
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_xcvr_dp_rpd(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DP_RPD_EN) == (USB_CTRL0_XCVR_DP_RPD_EN));
}

/**
  * @brief  Enable USB drive DM pull down during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DM_RPD_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_enable_xcvr_dm_rpd(usb_regs_t *USBx)
{
    SET_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DM_RPD_EN);
}

/**
  * @brief  Disable USB drive DM pull down during transceiver test mode
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DM_RPD_EN
  *
  * @param  USBx USB instance
  * @retval None
  */
__STATIC_INLINE void ll_usb_disable_xcvr_dm_rpd(usb_regs_t *USBx)
{
    CLEAR_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DM_RPD_EN);
}

/**
  * @brief  Check if USB drive DM pull down during transceiver test mode is enabled
  *
  *  Register|BitsName
  *  --------|--------
  * CTRL0 | XCVR_DM_RPD_EN
  *
  * @param  USBx USB instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_usb_is_enabled_xcvr_dm_rpd(usb_regs_t *USBx)
{
    return (READ_BITS(USBx->CTRL0, USB_CTRL0_XCVR_DM_RPD_EN) == (USB_CTRL0_XCVR_DM_RPD_EN));
}

/**
  * @brief  Set USB output endian mode
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | OUTPUT_ENDIAN_CTRL
  *
  * @param  USBx USB instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_USB_CTRL0_OUTPUT_ENDIAN_CTRL_SMALL
  *         @arg @ref LL_USB_CTRL0_OUTPUT_ENDIAN_CTRL_BIG
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_output_endian_mode(usb_regs_t *USBx, uint32_t mode)
{
    MODIFY_REG(USBx->CTRL0, USB_CTRL0_OUTPUT_ENDIAN_CTRL, mode);
}

/**
  * @brief  Get USB output endian mode
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | OUTPUT_ENDIAN_CTRL
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_USB_CTRL0_OUTPUT_ENDIAN_CTRL_SMALL
  *         @arg @ref LL_USB_CTRL0_OUTPUT_ENDIAN_CTRL_BIG
  */
__STATIC_INLINE uint32_t ll_usb_get_output_endian_mode(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->CTRL0, USB_CTRL0_OUTPUT_ENDIAN_CTRL));
}

/**
  * @brief  Set USB input endian mode
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | INPUT_ENDIAN_CTRL
  *
  * @param  USBx USB instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_USB_CTRL0_INPUT_ENDIAN_CTRL_SMALL
  *         @arg @ref LL_USB_CTRL0_INPUT_ENDIAN_CTRL_BIG
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_input_endian_mode(usb_regs_t *USBx, uint32_t mode)
{
    MODIFY_REG(USBx->CTRL0, USB_CTRL0_INPUT_ENDIAN_CTRL, mode);
}

/**
  * @brief  Get USB input endian mode
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | INPUT_ENDIAN_CTRL
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_USB_CTRL0_INPUT_ENDIAN_CTRL_SMALL
  *         @arg @ref LL_USB_CTRL0_INPUT_ENDIAN_CTRL_BIG
  */
__STATIC_INLINE uint32_t ll_usb_get_input_endian_mode(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->CTRL0, USB_CTRL0_INPUT_ENDIAN_CTRL));
}

/**
  * @brief  Set USB probe select signal
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | INPUT_ENDIAN_CTRL
  *
  * @param  USBx USB instance
  * @param  sel This parameter can be one of the following values:
  *         @arg @ref LL_USB_CTRL0_PROBE_SEL_PROTOCAL_STAT
  *         @arg @ref LL_USB_CTRL0_PROBE_SEL_RX_STAT
  *         @arg @ref LL_USB_CTRL0_PROBE_SEL_UTMI_SIGNALS
  *         @arg @ref LL_USB_CTRL0_PROBE_SEL_SYNC_STAT
  *         @arg @ref LL_USB_CTRL0_PROBE_SEL_TX_STAT
  *         @arg @ref LL_USB_CTRL0_PROBE_SEL_DPLL_STAT
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_probe_sel(usb_regs_t *USBx, uint32_t sel)
{
    MODIFY_REG(USBx->CTRL0, USB_CTRL0_PROBE_SEL, sel);
}

/**
  * @brief  Get USB probe select signal
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL0 | INPUT_ENDIAN_CTRL
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_USB_CTRL0_PROBE_SEL_PROTOCAL_STAT
  *         @arg @ref LL_USB_CTRL0_PROBE_SEL_RX_STAT
  *         @arg @ref LL_USB_CTRL0_PROBE_SEL_UTMI_SIGNALS
  *         @arg @ref LL_USB_CTRL0_PROBE_SEL_SYNC_STAT
  *         @arg @ref LL_USB_CTRL0_PROBE_SEL_TX_STAT
  *         @arg @ref LL_USB_CTRL0_PROBE_SEL_DPLL_STAT
  */
__STATIC_INLINE uint32_t ll_usb_get_probe_sel(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->CTRL0, USB_CTRL0_PROBE_SEL));
}

/**
  * @brief  Set USB ep3 DMA total transfer length
  *
  *  Register|BitsName
  *  --------|--------
  *  EP3_XFER_LEN | EP3_XFER_LEN
  *
  * @param  USBx USB instance
  * @param  len This parameter can be one of the following values:
  *         0x0 ~ 0xFFFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep3_xfer_len(usb_regs_t *USBx, uint32_t len)
{
    MODIFY_REG(USBx->EP3_XFER_LEN, USB_EP3_XFER_LEN, len);
}

/**
  * @brief  Get USB ep3 DMA total transfer length
  *
  *  Register|BitsName
  *  --------|--------
  *  EP3_XFER_LEN | EP3_XFER_LEN
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFFFF
  */
__STATIC_INLINE uint32_t ll_usb_get_ep3_xfer_len(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP3_XFER_LEN, USB_EP3_XFER_LEN));
}

/**
  * @brief  Set USB ep4 DMA total transfer length
  *
  *  Register|BitsName
  *  --------|--------
  *  EP4_XFER_LEN | EP4_XFER_LEN
  *
  * @param  USBx USB instance
  * @param  len This parameter can be one of the following values:
  *         0x0 ~ 0xFFFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep4_xfer_len(usb_regs_t *USBx, uint32_t len)
{
    MODIFY_REG(USBx->EP4_XFER_LEN, USB_EP4_XFER_LEN, len);
}

/**
  * @brief  Get USB ep4 DMA total transfer length
  *
  *  Register|BitsName
  *  --------|--------
  *  EP4_XFER_LEN | EP4_XFER_LEN
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFFFF
  */
__STATIC_INLINE uint32_t ll_usb_get_ep4_xfer_len(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP4_XFER_LEN, USB_EP4_XFER_LEN));
}

/**
  * @brief  Set USB ep5 DMA total transfer length
  *
  *  Register|BitsName
  *  --------|--------
  *  EP5_XFER_LEN | EP5_XFER_LEN
  *
  * @param  USBx USB instance
  * @param  len This parameter can be one of the following values:
  *         0x0 ~ 0xFFFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep5_xfer_len(usb_regs_t *USBx, uint32_t len)
{
    MODIFY_REG(USBx->EP5_XFER_LEN, USB_EP5_XFER_LEN, len);
}

/**
  * @brief  Get USB ep5 DMA total transfer length
  *
  *  Register|BitsName
  *  --------|--------
  *  EP5_XFER_LEN | EP5_XFER_LEN
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFFFF
  */
__STATIC_INLINE uint32_t ll_usb_get_ep5_xfer_len(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP5_XFER_LEN, USB_EP5_XFER_LEN));
}

/**
  * @brief  Set USB ep5 DMA recieve data time out value
  *
  *  Register|BitsName
  *  --------|--------
  *  EP5_TIMER | EP5_TIMER
  *
  * @param  USBx USB instance
  * @param  len This parameter can be one of the following values:
  *         0x0 ~ 0x3FF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep5_timer_val(usb_regs_t *USBx, uint32_t len)
{
    MODIFY_REG(USBx->EP5_TIMER, USB_EP5_TIMER_VAL, len);
}

/**
  * @brief  Get USB ep5 DMA total transfer length
  *
  *  Register|BitsName
  *  --------|--------
  *  EP5_TIMER | EP5_TIMER
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFFFF
  */
__STATIC_INLINE uint32_t ll_usb_get_ep5_timer_val(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP5_TIMER, USB_EP5_TIMER_VAL));
}

/**
  * @brief  Get USB received data sum in the EP0 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  *  RX_CNT | EP0_RX_DATA_SUM
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_ep0_rx_data_sum(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->RX_CNT, USB_RX_CNT_EP0_RX_DATA_SUM));
}

/**
  * @brief  Get USB received data sum in the EP1 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  *  RX_CNT | EP1_RX_DATA_SUM
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_ep1_rx_data_sum(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->RX_CNT, USB_RX_CNT_EP1_RX_DATA_SUM) >> USB_RX_CNT_EP1_RX_DATA_SUM_Pos);
}

/**
  * @brief  Get USB received data sum in the EP5 IN FIFO
  *
  *  Register|BitsName
  *  --------|--------
  *  EP5_RX_CNT | EP5_RX_CNT
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFFFFFFFFUL
  */
__STATIC_INLINE uint32_t ll_usb_get_ep5_rx_data_sum(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP5_RX_CNT, USB_EP5_RX_CNT));
}

/**
  * @brief  Set USB config descriptor start address
  *
  *  Register|BitsName
  *  --------|--------
  *  CFG_DESC_CTRL | START
  *
  * @param  USBx USB instance
  * @param  addr This parameter can be one of the following values:
  *         0x0 ~ 0xFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_cfg_desc_ctrl_start(usb_regs_t *USBx, uint32_t addr)
{
    MODIFY_REG(USBx->CFG_DESC_CTRL, USB_CFG_DESC_CTRL_START, addr);
}

/**
  * @brief  Get USB config descriptor start address
  *
  *  Register|BitsName
  *  --------|--------
  *  CFG_DESC_CTRL | START
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_cfg_desc_ctrl_start(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->CFG_DESC_CTRL, USB_CFG_DESC_CTRL_START));
}

/**
  * @brief  Set USB config descriptor size
  *
  *  Register|BitsName
  *  --------|--------
  *  CFG_DESC_CTRL | SIZE
  *
  * @param  USBx USB instance
  * @param  size This parameter can be one of the following values:
  *         0x0 ~ 0xFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_cfg_desc_ctrl_size(usb_regs_t *USBx, uint32_t size)
{
    MODIFY_REG(USBx->CFG_DESC_CTRL, USB_CFG_DESC_CTRL_SIZE, size << USB_CFG_DESC_CTRL_SIZE_Pos);
}

/**
  * @brief  Get USB config descriptor size
  *
  *  Register|BitsName
  *  --------|--------
  *  CFG_DESC_CTRL | SIZE
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_cfg_desc_ctrl_size(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->CFG_DESC_CTRL, USB_CFG_DESC_CTRL_SIZE) >> USB_CFG_DESC_CTRL_SIZE_Pos);
}

/**
  * @brief  Set USB Language ID descriptor start address
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC0_CTRL | START
  *
  * @param  USBx USB instance
  * @param  addr This parameter can be one of the following values:
  *         0x0 ~ 0xFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_str_desc0_ctrl_start(usb_regs_t *USBx, uint32_t addr)
{
    MODIFY_REG(USBx->STR_DESC0_CTRL, USB_STR_DESC0_CTRL_START, addr);
}

/**
  * @brief  Get USB Language ID descriptor start address
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC0_CTRL | START
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_str_desc0_ctrl_start(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->STR_DESC0_CTRL, USB_STR_DESC0_CTRL_START));
}

/**
  * @brief  Set USB Language ID descriptor size
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC0_CTRL | SIZE
  *
  * @param  USBx USB instance
  * @param  size This parameter can be one of the following values:
  *         0x0 ~ 0xFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_str_desc0_ctrl_size(usb_regs_t *USBx, uint32_t size)
{
    MODIFY_REG(USBx->STR_DESC0_CTRL, USB_STR_DESC0_CTRL_SIZE, size << USB_STR_DESC0_CTRL_SIZE_Pos);
}

/**
  * @brief  Get USB Language ID descriptor size
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC0_CTRL | SIZE
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_str_desc0_ctrl_size(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->STR_DESC0_CTRL, USB_STR_DESC0_CTRL_SIZE) >> USB_STR_DESC0_CTRL_SIZE_Pos);
}

/**
  * @brief  Set USB string descriptor start address
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC1_CTRL | START
  *
  * @param  USBx USB instance
  * @param  addr This parameter can be one of the following values:
  *         0x0 ~ 0xFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_str_desc1_ctrl_start(usb_regs_t *USBx, uint32_t addr)
{
    MODIFY_REG(USBx->STR_DESC1_CTRL, USB_STR_DESC1_CTRL_START, addr);
}

/**
  * @brief  Get USB string descriptor start address
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC1_CTRL | START
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_str_desc1_ctrl_start(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->STR_DESC1_CTRL, USB_STR_DESC1_CTRL_START));
}

/**
  * @brief  Set USB string descriptor size
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC1_CTRL | SIZE
  *
  * @param  USBx USB instance
  * @param  size This parameter can be one of the following values:
  *         0x0 ~ 0xFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_str_desc1_ctrl_size(usb_regs_t *USBx, uint32_t size)
{
    MODIFY_REG(USBx->STR_DESC1_CTRL, USB_STR_DESC1_CTRL_SIZE, size << USB_STR_DESC1_CTRL_SIZE_Pos);
}

/**
  * @brief  Get USB string descriptor size
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC1_CTRL | SIZE
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_str_desc1_ctrl_size(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->STR_DESC1_CTRL, USB_STR_DESC1_CTRL_SIZE) >> USB_STR_DESC1_CTRL_SIZE_Pos);
}

/**
  * @brief  Set USB endpoint 0 FIFO address
  *
  *  Register|BitsName
  *  --------|--------
  *  EP0_FIFO_ADDR | EP0_FIFO_ADDR
  *
  * @param  USBx USB instance
  * @param  value This parameter can be one of the following values:
  *         32 bit address
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_usb_ep0_fifo(usb_regs_t *USBx, uint32_t value)
{
    MODIFY_REG(USBx->EP0_FIFO_ADDR, USB_EP0_FIFO_ADDR, value);
}

/**
  * @brief  Get USB endpoint 0 FIFO value
  *
  *  Register|BitsName
  *  --------|--------
  *  EP0_FIFO_ADDR | EP0_FIFO_ADDR
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         32 bit address
  */
__STATIC_INLINE uint32_t ll_usb_get_usb_ep0_fifo(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP0_FIFO_ADDR, USB_EP0_FIFO_ADDR));
}

/**
  * @brief  Get USB endpoint 1 FIFO value
  *
  *  Register|BitsName
  *  --------|--------
  *  EP1_FIFO_ADDR | EP1_FIFO_ADDR
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         32 bit value
  */
__STATIC_INLINE uint32_t ll_usb_get_usb_ep1_fifo(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP1_FIFO_ADDR, USB_EP1_FIFO_ADDR));
}

/**
  * @brief  Set USB endpoint 2 FIFO value
  *
  *  Register|BitsName
  *  --------|--------
  *  EP2_FIFO_ADDR | EP2_FIFO_ADDR
  *
  * @param  USBx USB instance
  * @param  value This parameter can be one of the following values:
  *         32 bit value
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_usb_ep2_fifo(usb_regs_t *USBx, uint32_t value)
{
    MODIFY_REG(USBx->EP2_FIFO_ADDR, USB_EP2_FIFO_ADDR, value);
}

/**
  * @brief  Set USB endpoint 3 FIFO value
  *
  *  Register|BitsName
  *  --------|--------
  *  EP3_FIFO_ADDR | EP3_FIFO_ADDR
  *
  * @param  USBx USB instance
  * @param  value This parameter can be one of the following values:
  *         32 bit value
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_usb_ep3_fifo(usb_regs_t *USBx, uint32_t value)
{
    MODIFY_REG(USBx->EP3_FIFO_ADDR, USB_EP3_FIFO_ADDR, value);
}

/**
  * @brief  Set USB endpoint 4 FIFO value
  *
  *  Register|BitsName
  *  --------|--------
  *  EP4_FIFO_ADDR | EP4_FIFO_ADDR
  *
  * @param  USBx USB instance
  * @param  value This parameter can be one of the following values:
  *         32 bit value
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_usb_ep4_fifo(usb_regs_t *USBx, uint32_t value)
{
    MODIFY_REG(USBx->EP4_FIFO_ADDR, USB_EP4_FIFO_ADDR, value);
}

/**
  * @brief  Get USB endpoint 5 FIFO value
  *
  *  Register|BitsName
  *  --------|--------
  *  EP5_FIFO_ADDR | EP5_FIFO_ADDR
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         32 bit value
  */
__STATIC_INLINE uint32_t ll_usb_get_usb_ep5_fifo(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP5_FIFO_ADDR, USB_EP5_FIFO_ADDR));
}

/**
  * @brief  Set USB ep4 FIFO write byte enable value
  *
  *  Register|BitsName
  *  --------|--------
  *  EP4_FIFO_WR_EN | EP4_FIFO_WR_EN
  *
  * @param  USBx USB instance
  * @param  len This parameter can be one of the following values:
  *         @arg @ref  USB_LL_EC_USB_EP4_FIFO_WEN
  *         @arg @ref  LL_USB_EP4_FIFO_WEN_DEFAULT
  *         @arg @ref  LL_USB_EP4_FIFO_WEN_1BYTE
  *         @arg @ref  LL_USB_EP4_FIFO_WEN_2BYTE
  *         @arg @ref  LL_USB_EP4_FIFO_WEN_3BYTE
  *         @arg @ref  LL_USB_EP4_FIFO_WEN_4BYTE
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_ep4_fifo_wr_en(usb_regs_t *USBx, uint32_t len)
{
    MODIFY_REG(USBx->EP4_FIFO_WR_EN, USB_EP4_FIFO_WR_EN, len);
}

/**
  * @brief  Get USB ep4 FIFO write byte enable value
  *
  *  Register|BitsName
  *  --------|--------
  *  EP4_FIFO_WR_EN | EP4_FIFO_WR_EN
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref  USB_LL_EC_USB_EP4_FIFO_WEN
  *         @arg @ref  LL_USB_EP4_FIFO_WEN_DEFAULT
  *         @arg @ref  LL_USB_EP4_FIFO_WEN_1BYTE
  *         @arg @ref  LL_USB_EP4_FIFO_WEN_2BYTE
  *         @arg @ref  LL_USB_EP4_FIFO_WEN_3BYTE
  *         @arg @ref  LL_USB_EP4_FIFO_WEN_4BYTE
  */
__STATIC_INLINE uint32_t ll_usb_get_ep4_fifo_wr_en(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->EP4_FIFO_WR_EN, USB_EP4_FIFO_WR_EN));
}

/**
  * @brief  Set USB descriptor SRAM value
  *
  *  Register|BitsName
  *  --------|--------
  *  SRAM_ADDR | DESC_SRAM
  *
  * @param  USBx USB instance
  * @param  addr This parameter can be one of the following values:
  *         32 bit value
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_usb_desc_sram_addr(usb_regs_t *USBx, uint32_t addr)
{
    MODIFY_REG(USBx->SRAM_ADDR, USB_SRAM_ADDR_DESC_SRAM, addr);
}

/**
  * @brief  Set USB string descriptor(302) start address
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC1_CTRL | START
  *
  * @param  USBx USB instance
  * @param  addr This parameter can be one of the following values:
  *         0x0 ~ 0xFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_str_desc2_ctrl_start(usb_regs_t *USBx, uint32_t addr)
{
    MODIFY_REG(USBx->STR_DESC2_CTRL, USB_STR_DESC2_CTRL_START, addr);
}

/**
  * @brief  Get USB string descriptor(302) start address
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC1_CTRL | START
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_str_desc2_ctrl_start(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->STR_DESC2_CTRL, USB_STR_DESC2_CTRL_START));
}

/**
  * @brief  Set USB string descriptor(302) size
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC1_CTRL | SIZE
  *
  * @param  USBx USB instance
  * @param  size This parameter can be one of the following values:
  *         0x0 ~ 0xFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_str_desc2_ctrl_size(usb_regs_t *USBx, uint32_t size)
{
    MODIFY_REG(USBx->STR_DESC2_CTRL, USB_STR_DESC2_CTRL_SIZE, size << USB_STR_DESC2_CTRL_SIZE_Pos);
}

/**
  * @brief  Get USB string descriptor(302) size
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC1_CTRL | SIZE
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_str_desc2_ctrl_size(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->STR_DESC2_CTRL, USB_STR_DESC1_CTRL_SIZE) >> USB_STR_DESC2_CTRL_SIZE_Pos);
}

/**
  * @brief  Set USB string descriptor(303) start address
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC1_CTRL | START
  *
  * @param  USBx USB instance
  * @param  addr This parameter can be one of the following values:
  *         0x0 ~ 0xFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_str_desc3_ctrl_start(usb_regs_t *USBx, uint32_t addr)
{
    MODIFY_REG(USBx->STR_DESC3_CTRL, USB_STR_DESC3_CTRL_START, addr);
}

/**
  * @brief  Get USB string descriptor(303) start address
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC1_CTRL | START
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_str_desc3_ctrl_start(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->STR_DESC3_CTRL, USB_STR_DESC3_CTRL_START));
}

/**
  * @brief  Set USB string descriptor(303) size
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC1_CTRL | SIZE
  *
  * @param  USBx USB instance
  * @param  size This parameter can be one of the following values:
  *         0x0 ~ 0xFF
  * @retval None
  */
__STATIC_INLINE void ll_usb_set_str_desc3_ctrl_size(usb_regs_t *USBx, uint32_t size)
{
    MODIFY_REG(USBx->STR_DESC3_CTRL, USB_STR_DESC3_CTRL_SIZE, size << USB_STR_DESC3_CTRL_SIZE_Pos);
}

/**
  * @brief  Get USB string descriptor(303) size
  *
  *  Register|BitsName
  *  --------|--------
  *  STR_DESC1_CTRL | SIZE
  *
  * @param  USBx USB instance
  * @retval Returned Value can be one of the following values:
  *         0x0 ~ 0xFF
  */
__STATIC_INLINE uint32_t ll_usb_get_str_desc3_ctrl_size(usb_regs_t *USBx)
{
    return (uint32_t)(READ_BITS(USBx->STR_DESC3_CTRL, USB_STR_DESC1_CTRL_SIZE) >> USB_STR_DESC3_CTRL_SIZE_Pos);
}

/**
  * @brief  Enable USB interrupt
  *
  * @retval None
  */
void ll_usb_enable(void);


/**
  * @brief  Disable USB interrupt
  *
  * @retval None
  */
void ll_usb_disable(void);

/** @} */

/** @defgroup USB_LL_EF_Init  Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize USB registers (Registers restored to their default values).
  * @param  USBx USB instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: USB registers are de-initialized
  *          - ERROR: USB registers are not de-initialized
  */
error_status_t ll_usb_deinit(usb_regs_t *USBx);

/**
  * @brief  Initialize USB registers according to the specified
  *         parameters in p_usb_init.
  * @param  USBx USB instance
  * @param  p_usb_init Pointer to a ll_usb_init_t structure that contains the configuration
  *                         information for the specified USB peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: USB registers are initialized according to p_usb_init content
  *          - ERROR: Problem occurred during US Registers initialization
  */
error_status_t ll_usb_init(usb_regs_t *USBx, ll_usb_init_t *p_usb_init);

/**
  * @brief Set each field of a @ref ll_usb_init_t type structure to default value.
  * @param p_usb_init  Pointer to a @ref ll_usb_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_usb_struct_init(ll_usb_init_t *p_usb_init);

/** @} */

/** @} */

#endif /* USB */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_USB_H__ */

/** @} */

/** @} */

/** @} */

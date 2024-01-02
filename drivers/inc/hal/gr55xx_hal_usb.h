/**
 ****************************************************************************************
 *
 * @file   gr55xx_hal_usb.h
 * @author BLE Driver Team
 * @brief  Header file containing functions prototypes of USB HAL library.
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

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup HAL_DRIVER HAL Driver
  * @{
  */

/** @defgroup HAL_USB USB
  * @brief USB HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_USB_H__
#define __GR55xx_HAL_USB_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_usb.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_USB_ENUMERATIONS Enumerations
  * @{
  */

/**
  * @brief HAL USB State Enumerations definition
  */
typedef enum
{
    HAL_USB_STATE_RESET                 = 0x00,    /**< Peripheral not initialized                          */
    HAL_USB_STATE_READY                 = 0x01,    /**< Peripheral initialized and ready for use            */
    HAL_USB_STATE_BUSY                  = 0x02,    /**< An internal process is ongoing                      */
    HAL_USB_STATE_BUSY_HOST_REST        = 0x12,    /**< USB in host rest state during enumeration           */
    HAL_USB_STATE_BUSY_ADDR             = 0x22,    /**< USB in adress state during enumeration              */
    HAL_USB_STATE_BUSY_CFG              = 0x32,    /**< USB in configure state during enumeration           */
    HAL_USB_STATE_ERROR                 = 0x04,    /**< Peripheral in error                                 */
    HAL_USB_STATE_ERROR_CRC16           = 0x14,    /**< USB receive CRC error data                          */
    HAL_USB_STATE_ERROR_UPID            = 0x24,    /**< USB receive unsupported PID error                   */
    HAL_USB_STATE_ERROR_TIMEOUT         = 0x34,    /**< USB rx/tx timeout error                             */
    HAL_USB_STATE_ERROR_SEQ             = 0x44,    /**< USB DATA0/DATA1 PID sequence error                  */
    HAL_USB_STATE_ERROR_PID_CKS         = 0x54,    /**< USB PID checksum error                              */
    HAL_USB_STATE_ERROR_PID_CRC         = 0x64,    /**< USB PID CRC error                                   */
    HAL_USB_STATE_ERROR_DMA_RX          = 0x74,    /**< USB ep3 or ep4 AHB master receive ERROR response    */
    HAL_USB_STATE_ERROR_NSE             = 0x84,    /**< USB no such endpoint error                          */
    HAL_USB_STATE_ERROR_SYNC            = 0x94,    /**< USB SYNC error                                      */
    HAL_USB_STATE_ERROR_BIT_STUFF       = 0xA4,    /**< USB bit stuff error                                 */
    HAL_USB_STATE_ERROR_BYTE            = 0xB4,    /**< USB byte error                                      */
    HAL_USB_STATE_ERROR_EP5_TIMER       = 0xC4,    /**< USB ep5 timer out error                             */
    HAL_USB_STATE_ABORT                 = 0x08,    /**< Peripheral with abort request ongoing               */
    HAL_USB_STATE_SUSPEND               = 0x10     /**< USB transceiver suspend                             */
} hal_usb_state_t;

/**
  * @brief HAL USB EP
  */
typedef enum
{
    HAL_USB_EP0                         = 0x00,     /**< USB EP0.                          */
    HAL_USB_EP1                         = 0x01,     /**< USB EP1.                          */
    HAL_USB_EP2                         = 0x02,     /**< USB EP2.                          */
    HAL_USB_EP3                         = 0x03,     /**< USB EP3.                          */
    HAL_USB_EP4                         = 0x04,     /**< USB EP4.                          */
    HAL_USB_EP5                         = 0x05      /**< USB EP4.                          */
}hal_usb_ep_t;

/** @} */

/** @addtogroup HAL_USB_STRUCTURES Structures
  * @{
  */

/** @defgroup USB_Configuration USB Configuration
  * @{
  */

/**
  * @brief USB hardware enumeration Structure definition
  */
typedef struct hw_enum_parm
{
    uint8_t cfg_desc_start;             /**< Specifies the USB config descriptor start address
                                         This parameter can be a value of 8 bit data of offset of config descriptor.(offset refer to sram_addr(device descriptor))*/

    uint8_t cfg_desc_size;              /**< Specifies the USB config descriptor size
                                         This parameter can be a value of 8 bit data size of config descriptor.*/

    uint8_t str_desc0_start;            /**< Specifies the USB Language ID descriptor start address
                                         This parameter can be a value of 8 bit data of offset of Language ID descriptor.(offset refer to cfg_desc_start(device descriptor))*/

    uint8_t str_desc0_size;             /**< Specifies the USB Language ID descriptor size
                                         This parameter can be a value of 8 bit data size of Language ID descriptor.*/

    uint8_t str_desc1_start;            /**< Specifies the USB string descriptor(Manufacturer string) start address
                                         This parameter can be a value of 8 bit data of offset of string descriptor.(offset refer to str_desc0_start(device descriptor)*/

    uint8_t str_desc1_size;             /**< Specifies the USB string descriptor(Manufacturer string) size
                                         This parameter can be a value of 8 bit data size of string descriptor.*/

    uint8_t str_desc2_start;            /**< Specifies the USB string descriptor(Product string) start address
                                         This parameter can be a value of 8 bit data of offset of string descriptor.(device descriptor)*/

    uint8_t str_desc2_size;             /**< Specifies the USB string descriptor(Product string) size
                                         This parameter can be a value of 8 bit data size of string descriptor.*/

    uint8_t str_desc3_start;            /**< Specifies the USB string descriptor(Serial Number string) start address
                                         This parameter can be a value of 8 bit data of offset of string descriptor.(device descriptor)*/

    uint8_t str_desc3_size;             /**< Specifies the USB string descriptor(Serial Number string) size
                                         This parameter can be a value of 8 bit data size of string descriptor.*/

    uint16_t sram_size;                 /**< Specifies the USB hardware enumeration SRAM address size
                                         This parameter can be a value of 0~256byte descriptor size.(the whole size of hardware enumeration is 256byte)*/

    uint32_t *p_sram_addr;                 /**< Specifies the USB hardware enumeration SRAM address
                                         This parameter can be a value of 32 bit descriptor SRAM address.(the whole size of hardware enumeration is 256byte)*/

}hw_enum_parm_t;

/**
  * @brief USB init Structure definition
  */
typedef struct _usb_init
{
    uint32_t speed;                     /**< Specifies the USB Transceiver speed.
                                         This parameter can be a value of @ref USB_Speed_Mode. */

    uint32_t enum_type;                 /**< Specifies the USB enumeration type( hardware enumeration  and MCU enumeration).
                                         This parameter can be a value of @ref USB_Enum_Type. */

    hw_enum_parm_t hw_enum;           /**< Specifies the USB hardware enumeration parameter(MCU enumeration:NULL;hardware enumeration: valid).
                                         This parameter can be a value of @ref hw_enum_parm_t. */
} usb_init_t;
/** @} */

/** @defgroup USB_handle USB handle
  * @{
  */

/**
  * @brief USB handle Structure definition
  */
typedef struct _usb_handle
{
    usb_regs_t              *p_instance;            /**< USB registers base address                         */

    usb_init_t              init;                   /**< USB communication parameters                       */

    __IO hal_lock_t         lock;                   /**< Locking object                                     */

    __IO hal_usb_state_t    state;                  /**< USB communication state                            */

    __IO uint32_t           error_code;             /**< USB Error code                                     */

    uint32_t                timeout;                /**< Timeout for the USB memory access                  */

    uint32_t                retention[12];          /**< USB important register information.                */
} usb_handle_t;
/** @} */

/** @} */

/** @addtogroup HAL_USB_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup  HAL_USB_Callback Callback
  * @{
  */

/**
  * @brief HAL_USB Callback function definition
  */

typedef struct _usb_callback
{
    void (*usb_msp_init)(usb_handle_t *p_usb);                                  /**< USB init MSP callback                                      */
    void (*usb_msp_deinit)(usb_handle_t *p_usb);                                /**< USB de-init MSP callback                                   */
    void (*usb_suspend_callback)(usb_handle_t *p_usb);                          /**< USB suspend status interrupt callback                      */
    void (*usb_ep0_out_ready_callback)(usb_handle_t *p_usb);                    /**< USB ep0 data out ready callback                            */
    void (*usb_ep1_out_ready_callback)(usb_handle_t *p_usb);                    /**< USB ep1 data out ready callback                            */
    void (*usb_crc16_err_callback)(usb_handle_t *p_usb);                        /**< USB receive CRC error data callback                        */
    void (*usb_upid_err_callback)(usb_handle_t *p_usb);                         /**< USB receive unsupported PID callbac                        */
    void (*usb_time_out_callback)(usb_handle_t *p_usb);                         /**< USB rx/tx timeout error callback                           */
    void (*usb_seq_err_callback)(usb_handle_t *p_usb);                          /**< USB DATA0/DATA1 PID sequence error callback                */
    void (*usb_pid_cks_err_callback)(usb_handle_t *p_usb);                      /**< USB PID checksum error callback                            */
    void (*usb_pid_crc_err_callback)(usb_handle_t *p_usb);                      /**< USB PID CRC error callback                                 */
    void (*usb_host_reset_callback)(usb_handle_t *p_usb);                       /**< USB host reset callback                                    */
    void (*usb_ahb_xfer_err_callback)(usb_handle_t *p_usb);                     /**< USB ep3 and ep4 AHB master receive error response callback */
    void (*usb_nse_err_callback)(usb_handle_t *p_usb);                          /**< USB no such endpoint error callback                        */
    void (*usb_ep3_ahb_xfer_done_callback)(usb_handle_t *p_usb);                /**< USB ep3 AHB master transfer done callback                  */
    void (*usb_sync_err_callback)(usb_handle_t *p_usb);                         /**< USB SYNC error callback                                    */
    void (*usb_bit_stuff_err_callback)(usb_handle_t *p_usb);                    /**< USB bit stuff error callback                               */
    void (*usb_byte_err_callback)(usb_handle_t *p_usb);                         /**< USB byte error callback                                    */
    void (*usb_sof_callback)(usb_handle_t *p_usb);                              /**< USB SOF callback                                           */
    void (*usb_ep0_tx_done_callback)(usb_handle_t *p_usb);                      /**< USB ep0 TX done callback                                   */
    void (*usb_ep2_tx_done_callback)(usb_handle_t *p_usb);                      /**< USB ep2 TX done callback                                   */
    void (*usb_ep3_tx_done_callback)(usb_handle_t *p_usb);                      /**< USB ep3 TX done callback                                   */
    void (*usb_into_config_callback)(usb_handle_t *p_usb);                      /**< USB into configcallback                                    */
    void (*usb_ep4_ahb_xfer_done_callback)(usb_handle_t *p_usb);                /**< USB ep4 AHB master transfer done callback                  */
    void (*usb_ep4_tx_done_callback)(usb_handle_t *p_usb);                      /**< USB ep4 TX done callback                                   */
    void (*usb_ep5_out_ready_callback)(usb_handle_t *p_usb);                    /**< USB ep5 RX done callback                                   */
    void (*usb_ep5_ahb_xfer_done_callback)(usb_handle_t *p_usb);                /**< USB ep5 AHB master transfer done callback                  */
    void (*usb_ep5_timer_out_err_callback)(usb_handle_t *p_usb);                /**< USB ep5 timer out error callback                           */
    void (*usb_attach_callback)(usb_handle_t *p_usb);                           /**< USB attatch callback                                       */
    void (*usb_detach_callback)(usb_handle_t *p_usb);                           /**< USB detatch callback                                       */
} usb_callback_t;
/** @} */

/** @} */

/**
  * @defgroup  HAL_USB_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup USB_Exported_Constants USB Exported Constants
  * @{
  */


/** @defgroup USB_Error_Code USB Error Code
  * @{
  */
#define HAL_USB_ERROR_NONE              ((uint32_t)0x00000000)  /**< No error           */
#define HAL_USB_ERROR_TIMEOUT           ((uint32_t)0x00000001)  /**< Timeout error      */
#define HAL_USB_ERROR_TRANSFER          ((uint32_t)0x00000002)  /**< Transfer error     */
#define HAL_USB_ERROR_DMA               ((uint32_t)0x00000004)  /**< DMA transfer error */
#define HAL_USB_ERROR_INVALID_PARAM     ((uint32_t)0x00000008)  /**< Invalid parameters error */
/** @} */

/** @defgroup USB_EP_FIFO_Address USB  EF FIFO address
  * @{
  */
#define HAL_USB_FIFO_EP0                (USB_BASE + 0x40)  /**< USB EP0 FIFO address. */
#define HAL_USB_FIFO_EP1                (USB_BASE + 0x44)  /**< USB EP1 FIFO address. */
#define HAL_USB_FIFO_EP2                (USB_BASE + 0x48)  /**< USB EP2 FIFO address. */
#define HAL_USB_FIFO_EP3                (USB_BASE + 0x4C)  /**< USB EP3 FIFO address. */
/** @} */

/** @defgroup USB_Pwr_Mode USB Power Mode
  * @{
  */
#define USB_PWR_MODE_LP                 LL_USB_PWR_MODE_LP       /**< usb working in low power mode.    */
#define USB_PWR_MODE_NORMAL             LL_USB_PWR_MODE_NORMAL   /**< usb working in normal mode.       */
/** @} */

/** @defgroup USB_Speed_Mode USB speed Mode
  * @{
  */
#define USB_HAL_SPEED_LOW                   LL_USB_XCRV_CTRL_SPEED_LOW    /**< USB Transceiver speed select: low speed            */
#define USB_HAL_SPEED_FULL                  LL_USB_XCRV_CTRL_SPEED_FULL   /**< USB Transceiver speed select: full speed           */
/** @} */


/** @defgroup USB_Enum_Type USB enumeration type
  * @{
  */
#define USB_ENUM_TYPE_HW                LL_USB_ENUM_TYPE_HW     /**< USB hardware enumeration type              */
#define USB_ENUM_TYPE_MCU               LL_USB_ENUM_TYPE_MCU    /**< USB MCU enumeration type                   */
/** @} */

/** @defgroup USB_EP_ATTR USB Endpoint attribute setting
  * @{
  */
#define USB_EP_ATTR_INT                 ((uint32_t)0x00000000UL)                                  /**< Endpoint attribute setting: interrupt        */
#define USB_EP_ATTR_ISO                 ((uint32_t)0x00000001UL)                                  /**< Endpoint attribute setting: Isochronous      */
#define USB_EP_ATTR_BULK                ((uint32_t)0x00000002UL)                                  /**< Endpoint attribute setting: bulk             */
/** @} */

/** @defgroup USB_EP4_FIFO_WEN USB ep4 write fifo effective byte
  * @{
  */
#define USB_EP4_FIFO_WEN_DEFAULT        LL_USB_EP4_FIFO_WEN_DEFAULT                             /**< USB ep4 write fifo enable default value        */
#define USB_EP4_FIFO_WEN_1BYTE          LL_USB_EP4_FIFO_WEN_1BYTE                               /**< USB ep4 write fifo 1 byte effective            */
#define USB_EP4_FIFO_WEN_2BYTE          LL_USB_EP4_FIFO_WEN_2BYTE                               /**< USB ep4 write fifo 2 byte effective            */
#define USB_EP4_FIFO_WEN_3BYTE          LL_USB_EP4_FIFO_WEN_3BYTE                               /**< USB ep4 write fifo 3 byte effective            */
#define USB_EP4_FIFO_WEN_4BYTE          LL_USB_EP4_FIFO_WEN_4BYTE                               /**< USB ep4 write fifo 4 byte effective            */
/** @} */

/** @defgroup USB_Timeout_definition USB Timeout_definition
  * @{
  */
#define HAL_USB_TIMEOUT_DEFAULT_VALUE   ((uint32_t)5000)        /**< 5s */
/** @} */

/** @defgroup USB_Flags_definition USB Flags Definition
  * @{
  */
#define USB_FLAG_SUSPEND                 LL_USB_INT_STAT_SUSPEND                            /**< USB suspend status flag                        */
#define USB_FLAG_EP0_OUT_READY           LL_USB_INT_STAT_EP0_OUT_READY                      /**< USB ep0 data out ready flag                    */
#define USB_FLAG_EP1_OUT_READY           LL_USB_INT_STAT_EP1_OUT_READY                      /**< USB ep1 data out ready flag                    */
#define USB_FLAG_CRC16_ERR               LL_USB_INT_STAT_CRC16_ERR                          /**< USB receive CRC error data flag                */
#define USB_FLAG_UPID_ERR                LL_USB_INT_STAT_UPID_ERR                           /**< USB receive unsupported PID flag               */
#define USB_FLAG_TIMEOUT_ERR             LL_USB_INT_STAT_TIMEOUT_ERR                        /**< USB rx/tx timeout error flag                   */
#define USB_FLAG_SEQ_ERR                 LL_USB_INT_STAT_SEQ_ERR                            /**< USB DATA0/DATA1 PID sequence error flag        */
#define USB_FLAG_PID_CKS_ERR             LL_USB_INT_STAT_PID_CKS_ERR                        /**< USB PID checksum error flag                    */
#define USB_FLAG_PID_CRC_ERR             LL_USB_INT_STAT_PID_CRC_ERR                        /**< USB PID CRC error flag                         */
#define USB_FLAG_HOST_RESET              LL_USB_INT_STAT_HOST_RESET                         /**< USB host reset flag                            */
#define USB_FLAG_AHB_XFER_ERR            LL_USB_INT_STAT_AHB_XFER_ERR                      /**< USB ep3 and ep4 AHB master receive error  flag  */
#define USB_FLAG_NSE_ERR                 LL_USB_INT_STAT_NSE_ERR                            /**< USB no such endpoint error flag                */
#define USB_FLAG_EP3_AHB_XFER_DONE       LL_USB_INT_STAT_EP3_AHB_XFER_DONE                  /**< USB ep3 AHB master transfer done flag          */
#define USB_FLAG_SYNC_ERR                LL_USB_INT_STAT_SYNC_ERR                           /**< USB SYNC error flag                            */
#define USB_FLAG_BIT_STUFF_ERR           LL_USB_INT_STAT_BIT_STUFF_ERR                      /**< USB bit stuff error flag                       */
#define USB_FLAG_BYTE_ERR                LL_USB_INT_STAT_BYTE_ERR                           /**< USB byte error flag                            */
#define USB_FLAG_SOF                     LL_USB_INT_STAT_SOF                                /**< USB SOF flag                                   */
#define USB_FLAG_EP0_TX_DONE             LL_USB_INT_STAT_EP0_TX_DONE                        /**< USB ep0 TX done flag                           */
#define USB_FLAG_EP2_TX_DONE             LL_USB_INT_STAT_EP2_TX_DONE                        /**< USB ep2 TX done flag                           */
#define USB_FLAG_EP3_TX_DONE             LL_USB_INT_STAT_EP3_TX_DONE                        /**< USB ep3 TX done flag                           */
#define USB_FLAG_INTO_CONFIG             LL_USB_INT_STAT_INTO_CONFIG                        /**< USB into cofig status flag                     */
#define USB_FLAG_EP5_OUT_READY           LL_USB_INT_STAT_EP5_OUT_READY                      /**< USB ep5 data out ready flag                    */
#define USB_FLAG_EP4_AHB_XFER_DONE       LL_USB_INT_STAT_EP4_AHB_XFER_DONE                  /**< USB ep4 AHB master transfer done flag          */
#define USB_FLAG_EP4_TX_DONE             LL_USB_INT_STAT_EP4_TX_DONE                        /**< USB ep4 TX done flag                           */
#define USB_FLAG_EP5_AHB_XFER_DONE       LL_USB_INT_STAT_EP5_AHB_XFER_DONE                  /**< USB ep5 AHB master transfer done flag          */
#define USB_FLAG_EP5_TIMER_OUT_ERR       LL_USB_INT_STAT_EP5_TIMER_OUT_ERR                  /**< USB ep5 timer out error flag                   */
/** @} */

/** @defgroup USB_Interrupt_definition USB Interrupt Definition
  * @{
  */
#define USB_IT_ALL                       LL_USB_INT_EN_ALL                                  /**< USB all interrupt                                    */
#define USB_IT_RESET_VAL                 LL_USB_INT_RESET_VAL                               /**< USB interrupt control reset value                    */
#define USB_IT_SUSPEND                   LL_USB_INT_EN_SUSPEND                              /**< USB suspend status interrupt                         */
#define USB_IT_EP0_OUT_READY             LL_USB_INT_EN_EP0_OUT_READY                        /**< USB ep0 data out ready interrupt                     */
#define USB_IT_EP1_OUT_READY             LL_USB_INT_EN_EP1_OUT_READY                        /**< USB ep1 data out ready interrupt                     */
#define USB_IT_CRC16_ERR                 LL_USB_INT_EN_CRC16_ERR                            /**< USB receive CRC error data interrupt                 */
#define USB_IT_UPID_ERR                  LL_USB_INT_EN_UPID_ERR                             /**< USB receive unsupported PID interrupt                */
#define USB_IT_TIMEOUT_ERR               LL_USB_INT_EN_TIMEOUT_ERR                          /**< USB rx/tx timeout error interrupt                    */
#define USB_IT_SEQ_ERR                   LL_USB_INT_EN_SEQ_ERR                              /**< USB DATA0/DATA1 PID sequence error interrupt         */
#define USB_IT_PID_CKS_ERR               LL_USB_INT_EN_PID_CKS_ERR                          /**< USB PID checksum error interrupt                     */
#define USB_IT_PID_CRC_ERR               LL_USB_INT_EN_PID_CRC_ERR                          /**< USB PID CRC error interrupt                          */
#define USB_IT_HOST_RESET                LL_USB_INT_EN_HOST_RESET                           /**< USB host reset interrupt                             */
#define USB_IT_AHB_XFER_ERR              LL_USB_INT_EN_AHB_XFER_ERR                         /**< USB ep3 and ep4 AHB master receive error interrupt   */
#define USB_IT_NSE_ERR                   LL_USB_INT_EN_NSE_ERR                              /**< USB no such endpoint error interrupt                 */
#define USB_IT_EP3_AHB_XFER_DONE         LL_USB_INT_EN_EP3_AHB_XFER_DONE                    /**< USB ep3 AHB master transfer done interrupt           */
#define USB_IT_SYNC_ERR                  LL_USB_INT_EN_SYNC_ERR                             /**< USB SYNC error interrupt                             */
#define USB_IT_BIT_STUFF_ERR             LL_USB_INT_EN_BIT_STUFF_ERR                        /**< USB bit stuff error interrupt                        */
#define USB_IT_BYTE_ERR                  LL_USB_INT_EN_BYTE_ERR                             /**< USB byte error interrupt                             */
#define USB_IT_SOF                       LL_USB_INT_EN_SOF                                  /**< USB SOF interrupt                                    */
#define USB_IT_EP0_TX_DONE               LL_USB_INT_EN_EP0_TX_DONE                          /**< USB ep0 TX done interrupt                            */
#define USB_IT_EP2_TX_DONE               LL_USB_INT_EN_EP2_TX_DONE                          /**< USB ep2 TX done interrupt                            */
#define USB_IT_EP3_TX_DONE               LL_USB_INT_EN_EP3_TX_DONE                          /**< USB ep3 TX done interrupt                            */
#define USB_IT_INTO_CONFIG               LL_USB_INT_EN_INTO_CONFIG                          /**< USB into cofig status interrupt                      */
#define USB_IT_EP5_OUT_READY             LL_USB_INT_EN_EP5_OUT_READY                        /**< USB ep5 data out ready interrupt                     */
#define USB_IT_EP4_AHB_XFER_DONE         LL_USB_INT_EN_EP4_AHB_XFER_DONE                    /**< USB ep4 AHB master transfer done interrupt           */
#define USB_IT_EP4_TX_DONE               LL_USB_INT_EN_EP4_TX_DONE                          /**< USB ep4 TX done interrupt                            */
#define USB_IT_EP5_AHB_XFER_DONE         LL_USB_INT_EN_EP5_AHB_XFER_DONE                    /**< USB ep5 AHB master transfer done interrupt           */
#define USB_IT_EP5_TIMER_OUT_ERR         LL_USB_INT_EN_EP5_TIMER_OUT_ERR                    /**< USB ep5 timer out error interrupt                    */
/** @} */

/** @defgroup USB_IT_CLR USB interrupt clear
  * @{
  */
#define USB_IT_CLR_ALL                  LL_USB_INT_CLR_ALL                                 /**< USB all interrupt clear                                    */
#define USB_IT_CLR_SUSPEND              LL_USB_INT_CLR_SUSPEND                             /**< USB suspend status interrupt clear                         */
#define USB_IT_CLR_EP0_OUT_READY        LL_USB_INT_CLR_EP0_OUT_READY                       /**< USB ep0 data out ready interrupt clear                     */
#define USB_IT_CLR_EP1_OUT_READY        LL_USB_INT_CLR_EP1_OUT_READY                       /**< USB ep1 data out ready interrupt clear                     */
#define USB_IT_CLR_CRC16_ERR            LL_USB_INT_CLR_CRC16_ERR                           /**< USB receive CRC error data interrupt clear                 */
#define USB_IT_CLR_UPID_ERR             LL_USB_INT_CLR_UPID_ERR                            /**< USB receive unsupported PID interrupt clear                */
#define USB_IT_CLR_TIMEOUT_ERR          LL_USB_INT_CLR_TIMEOUT_ERR                         /**< USB rx/tx timeout error interrupt clear                    */
#define USB_IT_CLR_SEQ_ERR              LL_USB_INT_CLR_SEQ_ERR                             /**< USB DATA0/DATA1 PID sequence error interrupt clear         */
#define USB_IT_CLR_PID_CKS_ERR          LL_USB_INT_CLR_PID_CKS_ERR                         /**< USB PID checksum error interrupt clear                     */
#define USB_IT_CLR_PID_CRC_ERR          LL_USB_INT_CLR_PID_CRC_ERR                         /**< USB PID CRC error interrupt clear                          */
#define USB_IT_CLR_HOST_RESET           LL_USB_INT_CLR_HOST_RESET                          /**< USB host reset interrupt clear                             */
#define USB_IT_CLR_AHB_XFER_ERR         LL_USB_INT_CLR_AHB_XFER_ERR                        /**< USB ep3 and ep4 AHB master receive error interrupt clear   */
#define USB_IT_CLR_NSE_ERR              LL_USB_INT_CLR_NSE_ERR                             /**< USB no such endpoint error interrupt clear                 */
#define USB_IT_CLR_EP3_AHB_XFER_DONE    LL_USB_INT_CLR_EP3_AHB_XFER_DONE                   /**< USB ep3 AHB master transfer done interrupt clear           */
#define USB_IT_CLR_SYNC_ERR             LL_USB_INT_CLR_SYNC_ERR                            /**< USB SYNC error interrupt clear                             */
#define USB_IT_CLR_BIT_STUFF_ERR        LL_USB_INT_CLR_BIT_STUFF_ERR                       /**< USB bit stuff error interrupt clear                        */
#define USB_IT_CLR_BYTE_ERR             LL_USB_INT_CLR_BYTE_ERR                            /**< USB byte error interrupt clear                             */
#define USB_IT_CLR_SOF                  LL_USB_INT_CLR_SOF                                 /**< USB SOF interrupt clear                                    */
#define USB_IT_CLR_EP0_TX_DONE          LL_USB_INT_CLR_EP0_TX_DONE                         /**< USB ep0 TX done interrupt clear                            */
#define USB_IT_CLR_EP2_TX_DONE          LL_USB_INT_CLR_EP2_TX_DONE                         /**< USB ep2 TX done interrupt clear                            */
#define USB_IT_CLR_EP3_TX_DONE          LL_USB_INT_CLR_EP3_TX_DONE                         /**< USB ep3 TX done interrupt clear                            */
#define USB_IT_CLR_INTO_CONFIG          LL_USB_INT_CLR_INTO_CONFIG                         /**< USB into cofig status interrupt clear                      */
#define USB_IT_CLR_EP5_OUT_READY        LL_USB_INT_CLR_EP5_OUT_READY                       /**< USB ep5 data out ready interrupt clear                     */
#define USB_IT_CLR_EP4_AHB_XFER_DONE    LL_USB_INT_CLR_EP4_AHB_XFER_DONE                   /**< USB ep4 AHB master transfer done interrupt clear           */
#define USB_IT_CLR_EP4_TX_DONE          LL_USB_INT_CLR_EP4_TX_DONE                         /**< USB ep4 TX done interrupt clear                            */
#define USB_IT_CLR_EP5_AHB_XFER_DONE    LL_USB_INT_CLR_EP5_AHB_XFER_DONE                   /**< USB ep5 AHB master transfer done interrupt clear           */
#define USB_IT_CLR_EP5_TIMER_OUT_ERR    LL_USB_INT_CLR_EP5_TIMER_OUT_ERR                   /**< USB ep5 timer out error interrupt clear                    */
/** @} */

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup USB_Private_Macro USB Private Macros
  * @{
  */

/** @brief  Check if USB PWR Mode is valid.
  * @param  __MODE__    USB power Mode.
  * @retval SET (__MODE__ is low power) or RESET (__MODE__ is normal)
  */
#define IS_USB_PWR_MODE(__MODE__)              (((__MODE__) == LL_USB_PWR_MODE_LP) || \
                                                 ((__MODE__) == USB_PWR_MODE_NORMAL))

/** @brief  Check if USB speed Mode is valid.
  * @param  __MODE__    USB speed Mode.
  * @retval SET (__MODE__ is low speed) or RESET (__MODE__ is full speed)
  */
#define IS_USB_SPEED(__MODE__)              (((__MODE__) == USB_HAL_SPEED_LOW) || \
                                                 ((__MODE__) == USB_HAL_SPEED_FULL))

/** @brief  Check if USB enumeration type is valid.
  * @param  __TYPE__    USB enumeration type.
  * @retval SET (__TYPE__ is HW enumeration) or RESET (__TYPE__ is MCU enumeration)
  */
#define IS_USB_ENUM_TYPE(__TYPE__)              (((__TYPE__) == USB_ENUM_TYPE_HW) || \
                                                 ((__TYPE__) == USB_ENUM_TYPE_MCU))

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup USB_Exported_Macros USB Exported Macros
  * @{
  */

/** @brief  Reset USB handle states.
  * @param  __HANDLE__ USB handle.
  * @retval None
  */
#define __HAL_USB_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_USB_STATE_RESET)

/** @brief  Enable the specified USB peripheral.
  * @retval None
  */
#define __HAL_USB_ENABLE()                                      ll_usb_enable()

/** @brief  Disable the specified USB peripheral.
  * @retval None
  */
#define __HAL_USB_DISABLE()                                     ll_usb_disable()

/** @brief  Enable the specified USB peripheral software reset.
  * @retval None
  */
#define __HAL_USB_ENABLE_SW_RST()                               SET_BITS(MCU_SUB->USB_SW_RST, MCU_SUB_USB_SW_RST_EN)

/** @brief  Disable the specified USB peripheral software reset.
  * @retval None
  */
#define __HAL_USB_DISABLE_SW_RST()                              CLEAR_BITS(MCU_SUB->USB_SW_RST, MCU_SUB_USB_SW_RST_EN)

/** @brief  Enable the specified USB interrupts.
  * @param  __HANDLE__      Specifies the USB Handle.
  * @param  __INTERRUPT__   Specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg @ref USB_IT_ALL USB all interrupt
  *            @arg @ref USB_IT_RESET_VAL interrupt control register rest value
  *            @arg @ref USB_IT_SUSPEND USB suspend status interrupt
  *            @arg @ref USB_IT_EP0_OUT_READY USB ep0 data out ready interrupt
  *            @arg @ref USB_IT_EP1_OUT_READY USB ep1 data out ready interrupt
  *            @arg @ref USB_IT_CRC16_ERR USB receive CRC error data interrupt
  *            @arg @ref USB_IT_UPID_ERR USB receive unsupported PID interrupt
  *            @arg @ref USB_IT_TIMEOUT_ERR USB rx/tx timeout error interrupt
  *            @arg @ref USB_IT_SEQ_ERR USB DATA0/DATA1 PID sequence error interrupt
  *            @arg @ref USB_IT_PID_CKS_ERR USB PID checksum error interrupt
  *            @arg @ref USB_IT_PID_CRC_ERR  USB PID CRC error interrupt
  *            @arg @ref USB_IT_HOST_RESET  USB host reset interrupt
  *            @arg @ref USB_IT_AHB_XFER_ERR USB ep3 and ep4 AHB master receive ERROR response interrupt
  *            @arg @ref USB_IT_NSE_ERR USB no such endpoint error interrupt
  *            @arg @ref USB_IT_EP3_AHB_XFER_DONE  USB ep3 AHB master transfer done interrupt
  *            @arg @ref USB_IT_SYNC_ERR  USB SYNC error interrupt
  *            @arg @ref USB_IT_BIT_STUFF_ERR  USB bit stuff error interrupt
  *            @arg @ref USB_IT_BYTE_ERR USB byte error interrupt
  *            @arg @ref USB_IT_SOF USB SOF interrupt
  *            @arg @ref USB_IT_EP0_TX_DONE USB ep0 TX done interrupt
  *            @arg @ref USB_IT_EP2_TX_DONE USB ep2 TX done interrupt
  *            @arg @ref USB_IT_EP3_TX_DONE USB ep3 TX done interrupt
  *            @arg @ref USB_IT_INTO_CONFIG USB into cofig status interrupt
  *            @arg @ref USB_IT_EP5_OUT_READY USB ep5 data out ready interrupt
  *            @arg @ref USB_IT_CLR_EP4_AHB_XFER_DONE USB ep4 AHB master transfer done interrupt
  *            @arg @ref USB_IT_EP4_TX_DONE USB ep4 TX done interrupt
  *            @arg @ref USB_IT_EP5_AHB_XFER_DONE USB ep5 AHB master transfer done interrupt
  *            @arg @ref USB_IT_EP5_TIMER_OUT_ERR USB ep5 timer out error interrupt
  * @retval None
  */
#define __HAL_USB_ENABLE_IT(__HANDLE__, __INTERRUPT__)         SET_BITS((__HANDLE__)->p_instance->INT_EN, (__INTERRUPT__))

/** @brief  Disable the specified USB interrupts.
  * @param  __HANDLE__      Specifies the USB handle.
  * @param  __INTERRUPT__   Specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg @ref USB_IT_ALL USB all interrupt
  *            @arg @ref USB_IT_RESET_VAL interrupt control register rest value
  *            @arg @ref USB_IT_SUSPEND USB suspend status interrupt
  *            @arg @ref USB_IT_EP0_OUT_READY USB ep0 data out ready interrupt
  *            @arg @ref USB_IT_EP1_OUT_READY USB ep1 data out ready interrupt
  *            @arg @ref USB_IT_CRC16_ERR USB receive CRC error data interrupt
  *            @arg @ref USB_IT_UPID_ERR USB receive unsupported PID interrupt
  *            @arg @ref USB_IT_TIMEOUT_ERR USB rx/tx timeout error interrupt
  *            @arg @ref USB_IT_SEQ_ERR USB DATA0/DATA1 PID sequence error interrupt
  *            @arg @ref USB_IT_PID_CKS_ERR USB PID checksum error interrupt
  *            @arg @ref USB_IT_PID_CRC_ERR  USB PID CRC error interrupt
  *            @arg @ref USB_IT_HOST_RESET  USB host reset interrupt
  *            @arg @ref USB_IT_AHB_XFER_ERR USB ep3 and ep4 AHB master receive ERROR response interrupt
  *            @arg @ref USB_IT_NSE_ERR USB no such endpoint error interrupt
  *            @arg @ref USB_IT_EP3_AHB_XFER_DONE  USB ep3 AHB master transfer done interrupt
  *            @arg @ref USB_IT_SYNC_ERR  USB SYNC error interrupt
  *            @arg @ref USB_IT_BIT_STUFF_ERR  USB bit stuff error interrupt
  *            @arg @ref USB_IT_BYTE_ERR USB byte error interrupt
  *            @arg @ref USB_IT_SOF USB SOF interrupt
  *            @arg @ref USB_IT_EP0_TX_DONE USB ep0 TX done interrupt
  *            @arg @ref USB_IT_EP2_TX_DONE USB ep2 TX done interrupt
  *            @arg @ref USB_IT_EP3_TX_DONE USB ep3 TX done interrupt
  *            @arg @ref USB_IT_EP3_TX_DONE USB ep3 TX done interrupt
  *            @arg @ref USB_IT_INTO_CONFIG USB into cofig status interrupt
  *            @arg @ref USB_IT_EP5_OUT_READY USB ep5 data out ready interrupt
  *            @arg @ref USB_IT_CLR_EP4_AHB_XFER_DONE USB ep4 AHB master transfer done interrupt
  *            @arg @ref USB_IT_EP4_TX_DONE USB ep4 TX done interrupt
  *            @arg @ref USB_IT_EP5_AHB_XFER_DONE USB ep5 AHB master transfer done interrupt
  *            @arg @ref USB_IT_EP5_TIMER_OUT_ERR USB ep5 timer out error interrupt
  * @retval None
  */
#define __HAL_USB_DISABLE_IT(__HANDLE__, __INTERRUPT__)        CLEAR_BITS((__HANDLE__)->p_instance->INT_EN, (__INTERRUPT__))

/** @brief  Check whether the specified USB flag is set or not.
  * @param  __HANDLE__  Specifies the USB Handle.
  * @param  __FLAG__    Specifies the flag to check.
  *         This parameter can be one of the following values:
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  *            @arg @ref USB_Flags_definition USB all flag
  *            @arg @ref USB_FLAG_SUSPEND USB suspend status flag
  *            @arg @ref USB_FLAG_EP0_OUT_READY USB ep0 data out ready flag
  *            @arg @ref USB_FLAG_EP1_OUT_READY USB ep1 data out ready flag
  *            @arg @ref USB_FLAG_CRC16_ERR USB receive CRC error data flag
  *            @arg @ref USB_FLAG_UPID_ERR USB receive unsupported PID flag
  *            @arg @ref USB_FLAG_TIMEOUT_ERR USB rx/tx timeout error flag
  *            @arg @ref USB_FLAG_SEQ_ERR USB DATA0/DATA1 PID sequence error flag
  *            @arg @ref USB_FLAG_PID_CKS_ERR USB PID checksum error flag
  *            @arg @ref USB_FLAG_PID_CRC_ERR  USB PID CRC error flag
  *            @arg @ref USB_FLAG_HOST_RESET  USB host reset flag
  *            @arg @ref USB_FLAG_AHB_XFER_ERR USB ep3 and ep4 AHB master receive ERROR response flag
  *            @arg @ref USB_FLAG_NSE_ERR USB no such endpoint error flag
  *            @arg @ref USB_FLAG_EP3_AHB_XFER_DONE  USB ep3 AHB master transfer done flag
  *            @arg @ref USB_FLAG_SYNC_ERR  USB SYNC error flag
  *            @arg @ref USB_FLAG_BIT_STUFF_ERR  USB bit stuff error flag
  *            @arg @ref USB_FLAG_BYTE_ERR USB byte error flag
  *            @arg @ref USB_FLAG_SOF USB SOF flag
  *            @arg @ref USB_FLAG_EP0_TX_DONE USB ep0 TX done flag
  *            @arg @ref USB_FLAG_EP2_TX_DONE USB ep2 TX done flag
  *            @arg @ref USB_FLAG_EP3_TX_DONE USB ep3 TX done flag
  *            @arg @ref USB_FLAG_INTO_CONFIG USB into cofig status flag
  *            @arg @ref USB_FLAG_EP5_OUT_READY USB ep5 data out ready flag
  *            @arg @ref USB_FLAG_EP4_AHB_XFER_DONE USB ep4 AHB master transfer done flag
  *            @arg @ref USB_FLAG_EP4_TX_DONE USB ep4 TX done flag
  *            @arg @ref USB_FLAG_EP5_AHB_XFER_DONE USB ep5 AHB master transfer done flag
  *            @arg @ref USB_FLAG_EP5_TIMER_OUT_ERR USB ep5 timer out error flag
  */
#define __HAL_USB_GET_FLAG(__HANDLE__, __FLAG__)               ((READ_BITS((__HANDLE__)->p_instance->INT_STAT, (__FLAG__)) != 0) ? SET : RESET)

/** @brief  Clear the specified USB flag.
  * @param  __HANDLE__  Specifies the USB Handle.
  * @param  __FLAG__    Specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref USB_IT_CLR_ALL USB all clear
  *            @arg @ref USB_IT_CLR_SUSPEND USB suspend status clear
  *            @arg @ref USB_IT_CLR_EP0_OUT_READY USB ep0 data out ready clear
  *            @arg @ref USB_IT_CLR_EP1_OUT_READY USB ep1 data out ready clear
  *            @arg @ref USB_IT_CLR_CRC16_ERR USB receive CRC error data clear
  *            @arg @ref USB_IT_CLR_UPID_ERR USB receive unsupported PID clear
  *            @arg @ref USB_IT_CLR_TIMEOUT_ERR USB rx/tx timeout error clear
  *            @arg @ref USB_IT_CLR_SEQ_ERR USB DATA0/DATA1 PID sequence error clear
  *            @arg @ref USB_IT_CLR_PID_CKS_ERR USB PID checksum error clear
  *            @arg @ref USB_IT_CLR_PID_CRC_ERR  USB PID CRC error clear
  *            @arg @ref USB_IT_CLR_HOST_RESET  USB host reset clear
  *            @arg @ref USB_IT_CLR_AHB_XFER_ERR USB ep3 and ep4 AHB master receive ERROR response clear
  *            @arg @ref USB_IT_CLR_NSE_ERR USB no such endpoint error clear
  *            @arg @ref USB_IT_CLR_EP3_AHB_XFER_DONE  USB ep3 AHB master transfer done clear
  *            @arg @ref USB_IT_CLR_SYNC_ERR  USB SYNC error flag
  *            @arg @ref USB_IT_CLR_BIT_STUFF_ERR  USB bit stuff error clear
  *            @arg @ref USB_IT_CLR_BYTE_ERR USB byte error clear
  *            @arg @ref USB_IT_CLR_SOF USB SOF clear
  *            @arg @ref USB_IT_CLR_EP0_TX_DONE USB ep0 TX done clear
  *            @arg @ref USB_IT_CLR_EP2_TX_DONE USB ep2 TX done clear
  *            @arg @ref USB_IT_CLR_EP3_TX_DONE USB ep3 TX done clear
  *            @arg @ref USB_IT_CLR_INTO_CONFIG USB into cofig status interrupt clear
  *            @arg @ref USB_IT_CLR_EP5_OUT_READY USB ep5 data out ready interrupt clear
  *            @arg @ref USB_IT_CLR_EP4_AHB_XFER_DONE USB ep4 AHB master transfer done interrupt clear
  *            @arg @ref USB_IT_CLR_EP4_TX_DONE USB ep4 TX done interrupt clear
  *            @arg @ref USB_IT_CLR_EP5_AHB_XFER_DONE USB ep5 AHB master transfer done interrupt clear
  *            @arg @ref USB_IT_CLR_EP5_TIMER_OUT_ERR USB ep5 timer out error interrupt clear
  * @retval None
  */
#define __HAL_USB_CLEAR_FLAG(__HANDLE__, __FLAG__)             SET_BITS((__HANDLE__)->p_instance->INT_CLR, (__FLAG__))

/** @brief  Check whether the specified USB flag is set or not.
  * @param  __HANDLE__  Specifies the USB Handle.
  * @retval The new state of TRUE or FALSE.
  */
#define __HAL_USB_GET_EP0_OUT_DAT_RDY(__HANDLE__)               ((READ_BITS((__HANDLE__)->p_instance->CTRL, USB_CTRL_EP0_OUT_DATA_RDY) != 0) ? SET : RESET)

/** @brief  Enable the specified USB in addressed status.
  * @param  __HANDLE__      Specifies the USB Handle.
  * @retval None
  */
#define __HAL_USB_ENABLE_ADDR_STAT(__HANDLE__)                  SET_BITS((__HANDLE__)->p_instance->CTRL,USB_CTRL_ADDR_STAT)

/** @brief  Enable the specified USB in configured status.
  * @param  __HANDLE__      Specifies the USB Handle.
  * @retval None
  */
#define __HAL_USB_ENABLE_CFG_STAT(__HANDLE__)                   SET_BITS((__HANDLE__)->p_instance->CTRL,USB_CTRL_CFG_STAT)

/** @brief  clear the specified USB EP0 IN FIFO.
  * @param  __HANDLE__      Specifies the USB Handle.
  * @retval None
  */
#define __HAL_USB_CLEAR_EP0_IFIFO(__HANDLE__)                   SET_BITS((__HANDLE__)->p_instance->EP0_1_CTRL,USB_EP0_CTRL_IFIFO_CLR)

/** @brief  clear the specified USB EP1 IN FIFO.
  * @param  __HANDLE__      Specifies the USB Handle.
  * @retval None
  */
#define __HAL_USB_CLEAR_EP1_IFIFO(__HANDLE__)                   SET_BITS((__HANDLE__)->p_instance->EP0_1_CTRL,USB_EP1_CTRL_IFIFO_CLR)

/** @brief  Enable the specified USB device remote_wakeup feature.
  * @param  __HANDLE__      Specifies the USB handle.
  * @retval None
  */
#define __HAL_USB_ENABLE_DEV_REMOTE_WAKEUP(__HANDLE__)          SET_BITS((__HANDLE__)->p_instance->CTRL,USB_CTRL_DEV_REMOTE_WAKEUP)

/** @brief  Disable the specified USB device remote_wakeup feature.
  * @param  __HANDLE__      Specifies the USB handle.
  * @retval None
  */
#define __HAL_USB_DISABLE_DEV_REMOTE_WAKEUP(__HANDLE__)         CLEAR_BITS((__HANDLE__)->p_instance->CTRL,USB_CTRL_DEV_REMOTE_WAKEUP)

/** @brief  Enable the specified USB EP5 DMA READ.
  * @param  __HANDLE__      Specifies the USB Handle.
  * @retval None
  */
#define __HAL_USB_ENABLE_EP5_DMA_READ(__HANDLE__)               SET_BITS((__HANDLE__)->p_instance->EP5_CTRL,USB_EP5_CTRL_AHBM_EN)

/** @brief  Disable the specified USB EP5 DMA READ.
  * @param  __HANDLE__      Specifies the USB Handle.
  * @retval None
  */
#define __HAL_USB_DISABLE_EP5_DMA_READ(__HANDLE__)              CLEAR_BITS((__HANDLE__)->p_instance->EP5_CTRL,USB_EP5_CTRL_AHBM_EN)


/** @brief  Set USB ep5 DMA recieve data time out value.
  * @param  __HANDLE__      Specifies the USB Handle.
  * @param  __VAL__    Specifies 0:never timeout 1~1000ms.
  * @retval None
  */
#define __HAL_USB_SET_EP5_TIMER_VAL(__HANDLE__, __VAL__)        MODIFY_REG((__HANDLE__)->p_instance->EP5_TIMER, USB_EP5_TIMER_VAL, (__VAL__));

/** @brief  Set USB ep4 DMA burst size value.
  * @param  __HANDLE__      Specifies the USB Handle.
  * @param  __VAL__    Specifies 0:ep4 DMA burst size value(32~1023).
  * @retval None
  */
#define __HAL_USB_SET_EP4_BURST_SIZE(__HANDLE__, __VAL__)        MODIFY_REG((__HANDLE__)->p_instance->EP4_AHBM_CTRL, USB_EP4_AHBM_CTRL_BURST_SIZE,\
                                                                            (__VAL__) << USB_EP4_AHBM_CTRL_BURST_SIZE_Pos);
/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_USB_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup USB_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initializations functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the USBx peripheral:

      (+) User must implement hal_usb_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_usb_init() to configure the selected device with
          the selected configuration:
        (++) Data Size
        (++) Clock Polarity
        (++) Audio Frequency

      (+) Call the function hal_usb_deinit() to restore the default configuration
          of the selected USBx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the USB according to the specified parameters
 *         in the usb_init_t and initialize the associated handle.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_init(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  De-initialize the USB peripheral.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_deinit(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  Initialize the USB MSP.
 * @note   This function should not be modified. When the callback is needed,
            the hal_usb_msp_deinit can be implemented in the user file.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_msp_init(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  De-initialize the USB MSP.
 * @note   This function should not be modified. When the callback is needed,
            the hal_usb_msp_deinit can be implemented in the user file.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_msp_deinit(usb_handle_t *p_usb);

/** @} */

/** @defgroup USB_Exported_Functions_Group2 USB operation functions
 *  @brief   Data transfers functions
 *
@verbatim
  ==============================================================================
                      ##### USB operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the USB
    data transfers.

    [..] The USB supports master and slave mode:

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts
            or DMA, These APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated USB IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The hal_usb_tx_cplt_callback(), hal_usb_rx_cplt_callback() and hal_usb_tx_rx_cplt_callback() user callbacks
            will be executed respectively at the end of the transmit or Receive process
            The hal_usb_error_callback() user callback will be executed when a communication error is detected.

    (#) APIs provided for these 2 transfer modes (Blocking mode or Non blocking mode using either Interrupt or DMA)
        exist for 1-Line (simplex) and 2-Line (full duplex) modes.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  usb modlue reset the basic configuration during host reset
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_reset(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb module gets whether the device is connected to the host.
 * @retval ::0: usb device is disconnected to the host.
 * @retval ::1: usb device is connected to the host.
 ****************************************************************************************
 */
uint8_t hal_usb_get_connect_state(void);

/**
 ****************************************************************************************
 * @brief  usb ep start transmit data.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:    USB endpoit number(only EP0 EP2 EP3 EP4 supported)
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_ep_write_start(usb_handle_t *p_usb,hal_usb_ep_t ep);

/**
 ****************************************************************************************
 * @brief  usbd inform host transmit data end.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:    USB endpoit number(only EP0 EP2 EP3 EP4 supported)
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_ep_write_end(usb_handle_t *p_usb,hal_usb_ep_t ep);

/**
 ****************************************************************************************
 * @brief  usb ep start receive data.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:    USB endpoit number(only EP0 EP1 EP5 supported)
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_ep_read_start(usb_handle_t *p_usb,hal_usb_ep_t ep);

/**
 ****************************************************************************************
 * @brief  usbd inform host receive data end.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:    USB endpoit number(only EP0 EP1 EP5 supported)
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_ep_read_end(usb_handle_t *p_usb,hal_usb_ep_t ep);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data to USB endpoint in dma mode.
 * @param[in]  p_usb:  Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:     USB endpoit number( EP3 or EP4 support dma transmit mode)
 * @param[in]  p_data: Pointer to ahbm buffer adderss
 * @param[in]  length: Amount of data transmit during dma mode.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_ep_transmit_dma(usb_handle_t *p_usb,hal_usb_ep_t ep,uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data to USB endpoint in dma mode.
 * @param[in]  p_usb:  Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:     USB endpoit number(EP5 support dma receive mode)
 * @param[in]  p_data: Pointer to AHB master buffer adderss
 * @param[in]  length: Amount of data receive during dma mode.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_ep_receive_dma(usb_handle_t *p_usb,hal_usb_ep_t ep,uint8_t *p_data, uint32_t length);
/**
 ****************************************************************************************
 * @brief  set USB devcice address.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  addr:  USB device address.
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_set_addr(usb_handle_t *p_usb,uint32_t addr);

/**
 ****************************************************************************************
 * @brief  get USB devcice address
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @return USB devcice address form host
 ****************************************************************************************
 */
uint32_t hal_usb_get_addr(usb_handle_t *p_usb);


/**
 ****************************************************************************************
 * @brief  write 32bit value to USB endpoit FIFO.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:    USB endpoit number(only EP0 EP2 EP3 EP4 supported)
 * @param[in]  value: USB endpoit FIFO 32bit data
 * @param[in]  ep4_wr_byte: USB ep4 write fifo effective byte
                            USB_EP4_FIFO_WEN_DEFAULT
                            USB_EP4_FIFO_WEN_1BYTE
                            USB_EP4_FIFO_WEN_2BYTE
                            USB_EP4_FIFO_WEN_3BYTE
                            USB_EP4_FIFO_WEN_4BYTE
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_write_ep_fifo(usb_handle_t *p_usb,hal_usb_ep_t ep,uint32_t value,uint32_t ep4_wr_byte);

/**
 ****************************************************************************************
 * @brief  read 32bit value from USB endpoit FIFO.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:    ep EP0 EP1 EP5
 * @retval ::  value USB endpoit FIFO 32bit data
 ****************************************************************************************
 */
uint32_t hal_usb_read_ep_fifo(usb_handle_t *p_usb,hal_usb_ep_t ep);


/**
 ****************************************************************************************
 * @brief  set USB EP attribute.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:    ep EP1 EP2 EP3
 * @param[in]  attr:  USB attribute setting. This parameter can be one of the following values:
 *            @arg @ref USB_EP_ATTR_INT Endpoint attribute setting: interrupt
 *            @arg @ref USB_EP_ATTR_ISO Endpoint attribute setting: Isochronous
 *            @arg @ref USB_EP_ATTR_BULK Endpoint attribute setting: bulk
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_set_ep_attr(usb_handle_t *p_usb,hal_usb_ep_t ep,uint32_t attr);

/**
 ****************************************************************************************
 * @brief  get USB EP attribute setting.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:    ep EP1 EP2 EP3
 * @return USB attribute setting This parameter can be one of the following values:
 *            @arg @ref USB_EP_ATTR_INT Endpoint attribute setting: interrupt
 *            @arg @ref USB_EP_ATTR_ISO Endpoint attribute setting: Isochronous
 *            @arg @ref USB_EP_ATTR_BULK Endpoint attribute setting: bulk
 ****************************************************************************************
 */
uint32_t hal_usb_get_ep_attr(usb_handle_t *p_usb,hal_usb_ep_t ep);

/**
 ****************************************************************************************
 * @brief  USB halt the endpoint.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:    ep EP1 EP2 EP3
 * @retval ::HAL_OK:      Operation is OK.
 * @retval ::HAL_ERROR:   Parameter error or operation not supported.
 * @retval ::HAL_BUSY:    Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_halt_ep(usb_handle_t *p_usb,hal_usb_ep_t ep);

/**
 ****************************************************************************************
 * @brief  USB un halt the endpoint.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:    ep EP1 EP2 EP3
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_un_halt_ep(usb_handle_t *p_usb,hal_usb_ep_t ep);

/**
 ****************************************************************************************
 * @brief  USB un halt the endpoint and set the endpoint attribute.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @param[in]  ep:    ep EP1 EP2 EP3
 * @param[in]  attr:  USB attribute setting.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_set_and_unhalt_ep(usb_handle_t *p_usb,hal_usb_ep_t ep,uint32_t attr);


/**
 ****************************************************************************************
 * @brief  get USB EP0 received data sum.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @return USB received data sum in the EP0 IN FIFO
 *         @arg 0x0 ~ 0xFF.
 ****************************************************************************************
 */
uint32_t hal_usb_get_ep0_rx_data_sum(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  get USB all interrupt status.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @return USB all interrupt flag
 ****************************************************************************************
 */
uint32_t hal_usb_get_it_flag(usb_handle_t *p_usb);
/**
 ****************************************************************************************
 * @brief  get USB EP1 received data sum.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @return USB received data sum in the EP IN FIFO
 *         @arg 0x0 ~ 0xFF.
 ****************************************************************************************
 */
uint32_t hal_usb_get_ep1_rx_data_sum(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  get USB EP5 received data sum.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @return USB received data sum in the EP IN FIFO
 *         @arg 0x0 ~ 0xFF.
 ****************************************************************************************
 */
uint32_t hal_usb_get_ep5_rx_data_sum(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  send ok command to USB host(EP0).
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
  */
hal_status_t hal_usb_set_cmd_ok(usb_handle_t *p_usb);


/**
 ****************************************************************************************
 * @brief  send error command to USB host(EP0).
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
  */
hal_status_t hal_usb_set_cmd_err(usb_handle_t *p_usb);

/** @} */

/** @addtogroup USB_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle USB interrupt request.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_irq_handler(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  Handle USB attach interrupt request.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_attach_irq_handler(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  Handle USB detach interrupt request.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_detach_irq_handler(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  USB attach callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_attach_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  USB detach callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_detach_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb suspend callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_suspend_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb EP0 output ready callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_ep0_out_ready_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb EP1 output ready callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_ep1_out_ready_callback(usb_handle_t *p_usb);


/**
 ****************************************************************************************
 * @brief  usb receive CRC error data callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_crc16_err_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb receive unsupported PID callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_upid_err_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb rx/tx timeout error callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_time_out_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb DATA0/DATA1 PID sequence error callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_seq_err_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb PID checksum error callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_pid_cks_err_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb PID CRC error callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_pid_crc_err_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb host reset callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_host_reset_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb EP3 and EP4 AHB master receive ERROR response callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_ahb_xfer_err_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb no such endpoint error callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_nse_err_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb ep3 AHB master transfer done callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_ep3_ahb_xfer_done_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb SYNC error callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_sync_err_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb bit stuff error callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_bit_stuff_err_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb byte error callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_byte_err_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb SOF interrupt callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_sof_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb EP0 IN FIFO data has sent to host callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_ep0_tx_done_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb EP2 IN FIFO data has sent to host callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_ep2_tx_done_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb EP3 IN FIFO data has sent to host callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_ep3_tx_done_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb hardware enumeration into config status callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_into_config_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb EP5 output ready callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_ep5_out_ready_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb ep4 AHB master transfer done callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_ep4_ahb_xfer_done_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb EP4 IN FIFO data has sent to host callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_ep4_tx_done_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb ep5 AHB master transfer done callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_ep5_ahb_xfer_done_callback(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  usb ep5 timer out error callback.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 ****************************************************************************************
 */
void hal_usb_ep5_timer_out_err_callback(usb_handle_t *p_usb);

/** @} */

/** @defgroup USB_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   USB control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the USB.
     (+) hal_usb_get_state() API can be helpful to check in run-time the state of the USB peripheral
     (+) hal_usb_get_error() check in run-time Errors occurring during communication
     (+) hal_usb_set_timeout() set the timeout during internal process
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the USB handle state.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @retval ::HAL_USB_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_USB_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_USB_STATE_BUSY: An internal process is ongoing.
 * @retval ::HAL_USB_STATE_BUSY_HOST_REST: USB in host rest state during enumeration
 * @retval ::HAL_USB_STATE_BUSY_ADDR: USB in adress state during enumeration
 * @retval ::HAL_USB_STATE_BUSY_CFG: USB in configure state during enumeration
 * @retval ::HAL_USB_STATE_ABORT: Peripheral with abort request ongoing.
 * @retval ::HAL_USB_STATE_ERROR: Peripheral in error.
 ****************************************************************************************
 */
hal_usb_state_t hal_usb_get_state(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  Return the USB error code.
 * @param[in]  p_usb: Pointer to an USB handle which contains the configuration information for the specified USB module.
 * @return USB error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_usb_get_error(usb_handle_t *p_usb);


/**
 ****************************************************************************************
 * @brief  Suspend some registers related to USB configuration before sleep.
 * @param[in] p_usb: Pointer to a USB handle which contains the configuration
 *                 information for the specified USB module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_suspend_reg(usb_handle_t *p_usb);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to USB configuration after sleep.
 *         This function must be used in conjunction with the hal_usb_suspend_reg().
 * @param[in] p_usb: Pointer to a USB handle which contains the configuration
 *                 information for the specified USB module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_usb_resume_reg(usb_handle_t *p_usb);


/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_USB_H__ */

/** @} */

/** @} */

/** @} */

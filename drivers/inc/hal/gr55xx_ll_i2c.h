/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_i2c.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of I2C LL library.
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

/** @addtogroup LL_DRIVER LL Driver
  * @{
  */

/** @defgroup LL_I2C I2C
  * @brief I2C LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_I2C_H__
#define __GR55xx_LL_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (I2C0) || defined (I2C1)

/** @defgroup I2C_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup I2C_LL_ES_INIT I2C Exported init structure
  * @{
  */

/**
  * @brief LL I2C init Structure definition
  */
typedef struct _ll_i2c_init
{
    uint32_t speed;               /**< Specifies the transfer speed. See @ref I2C_LL_EC_SPEED. */

    uint32_t own_address;         /**< Specifies the device own address.
                                     This parameter must be a value between Min_Data = 0x00 and Max_Data = 0x3FF

                                     This feature can be modified afterwards using unitary function @ref ll_i2c_set_own_address(). */

    uint32_t own_addr_size;       /**< Specifies the device own address 1 size (7-bit or 10-bit).
                                     This parameter can be a value of @ref I2C_LL_EC_OWNADDRESS

                                     This feature can be modified afterwards using unitary function @ref ll_i2c_set_own_address(). */
} ll_i2c_init_t;

/** @} */

/** @} */

/**
  * @defgroup  I2C_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup I2C_LL_Exported_Constants I2C Exported Constants
  * @{
  */

/** @defgroup I2C_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags definitions which can be used with LL_I2C_ReadReg function
  * @{
  */
#define LL_I2C_INTR_STAT_SCL_STUCK_AT_LOW   I2C_RAW_INT_STAT_SCL_STUCKLOW   /**< SCL_STUCK_AT_LOW interrupt flag */
#define LL_I2C_INTR_STAT_MST_ON_HOLD        I2C_RAW_INT_STAT_M_HOLD         /**< MST_ON_HOLD interrupt flag */
#define LL_I2C_INTR_STAT_RESTART_DET        I2C_RAW_INT_STAT_RESTART_DET    /**< RESTART_DET interrupt flag */
#define LL_I2C_INTR_STAT_GEN_CALL           I2C_RAW_INT_STAT_GEN_CALL       /**< GEN_CALL interrupt flag */
#define LL_I2C_INTR_STAT_START_DET          I2C_RAW_INT_STAT_START_DET      /**< START_DET interrupt flag */
#define LL_I2C_INTR_STAT_STOP_DET           I2C_RAW_INT_STAT_STOP_DET       /**< STOP_DET interrupt flag */
#define LL_I2C_INTR_STAT_ACTIVITY           I2C_RAW_INT_STAT_ACTIVITY       /**< ACTIVITY interrupt flag */
#define LL_I2C_INTR_STAT_RX_DONE            I2C_RAW_INT_STAT_RX_DONE        /**< RX_DONE interrupt flag */
#define LL_I2C_INTR_STAT_TX_ABRT            I2C_RAW_INT_STAT_TX_ABORT       /**< TX_ABRT interrupt flag */
#define LL_I2C_INTR_STAT_RD_REQ             I2C_RAW_INT_STAT_RD_REQ         /**< RD_REQ interrupt flag */
#define LL_I2C_INTR_STAT_TX_EMPTY           I2C_RAW_INT_STAT_TX_EMPTY       /**< TX_EMPTY interrupt flag */
#define LL_I2C_INTR_STAT_TX_OVER            I2C_RAW_INT_STAT_TX_OVER        /**< TX_OVER interrupt flag */
#define LL_I2C_INTR_STAT_RX_FULL            I2C_RAW_INT_STAT_RX_FULL        /**< RX_FULL interrupt flag */
#define LL_I2C_INTR_STAT_RX_OVER            I2C_RAW_INT_STAT_RX_OVER        /**< RX_OVER interrupt flag */
#define LL_I2C_INTR_STAT_RX_UNDER           I2C_RAW_INT_STAT_RX_UNDER       /**< RX_UNDER interrupt flag */

#define LL_I2C_ABRT_TX_FLUSH_CNT            I2C_TX_ABORT_SRC_TX_FLUSH_CNT          /**< Indcates the number of TX FOFO data command */
#define LL_I2C_ABRT_SDA_STUCK_AT_LOW        I2C_TX_ABORT_SRC_ABORT_SDA_STUCK       /**< The SDA stuck at low detected by master */
#define LL_I2C_ABRT_USER_ABRT               I2C_TX_ABORT_SRC_ABORT_USER_ABORT      /**< Transfer abort detected by master */
#define LL_I2C_ABRT_SLVRD_INTX              I2C_TX_ABORT_SRC_ABORT_SLVRD_INTX      /**< Slave trying to transmit to remote master in read mode */
#define LL_I2C_ABRT_SLV_ARBLOST             I2C_TX_ABORT_SRC_ABORT_S_ARBLOST       /**< Slave lost arbitration to remote master */
#define LL_I2C_ABRT_SLVFLUSH_TXFIFO         I2C_TX_ABORT_SRC_ABORT_SLVFLUSH_TXFIFO /**< Slave flushes existing data in TX-FIFO upon getting read command */
#define LL_I2C_ABRT_ARB_LOST                I2C_TX_ABORT_SRC_ABORT_LOST            /**< Master or Slave Transmitter lost arbitration */
#define LL_I2C_ABRT_MST_DIS                 I2C_TX_ABORT_SRC_ABORT_MASTER_DIS      /**< User intitating master operation when MASTER disabled */
#define LL_I2C_ABRT_10B_RD_NORSTRT          I2C_TX_ABORT_SRC_ABORT_10B_RD_NORSTR   /**< Master trying to read in 10-Bit addressing mode when RESTART disabled */
#define LL_I2C_ABRT_SBYTE_NORSTRT           I2C_TX_ABORT_SRC_ABORT_SBYTE_NORSTRT   /**< User trying to send START byte when RESTART disabled */
#define LL_I2C_ABRT_HS_NORSTRT              I2C_TX_ABORT_SRC_ABORT_HS_NORSTRT      /**< User trying to swidth Master to HS mode when RESTART disabled */
#define LL_I2C_ABRT_SBYTE_ACKDET            I2C_TX_ABORT_SRC_ABORT_SBYTE_ACKDET    /**< ACK detected for START byte */
#define LL_I2C_ABRT_HS_ACKDET               I2C_TX_ABORT_SRC_ABORT_HS_ACKDET       /**< HS Master code is ACKed in HS Mode */
#define LL_I2C_ABRT_GCALL_READ              I2C_TX_ABORT_SRC_ABORT_GCALL_RD        /**< GCALL is followed by read from bus */
#define LL_I2C_ABRT_GCALL_NOACK             I2C_TX_ABORT_SRC_ABORT_GCALL_NOACK     /**< GCALL is not ACKed by any slave */
#define LL_I2C_ABRT_TXDATA_NOACK            I2C_TX_ABORT_SRC_ABORT_TX_NOACK        /**< Transmitted data is not ACKed by addressed slave */
#define LL_I2C_ABRT_10ADDR2_NOACK           I2C_TX_ABORT_SRC_ABORT_10B2_NOACK      /**< Byte 2 of 10-Bit Address is not ACKed by any slave */
#define LL_I2C_ABRT_10ADDR1_NOACK           I2C_TX_ABORT_SRC_ABORT_10B1_NOACK      /**< Byte 1 of 10-Bit Address is not ACKed by any slave */
#define LL_I2C_ABRT_7B_ADDR_NOACK           I2C_TX_ABORT_SRC_ABORT_7B_NOACK        /**< 7Bit Address is not ACKed by any slave */
/** @} */

/** @defgroup I2C_LL_EC_IT IT Defines
  * @brief    Interrupt definitions which can be used with LL_I2C_ReadReg and  LL_I2C_WriteReg functions
  * @{
  */
#define LL_I2C_INTR_MASK_SCL_STUCK_AT_LOW   I2C_INT_MASK_MASK_SCL_STUCKLOW  /**< SCL_STUCK_AT_LOW interrupt flag */
#define LL_I2C_INTR_MASK_MST_ON_HOLD        I2C_INT_MASK_MASK_M_HOLD        /**< MST_ON_HOLD interrupt */
#define LL_I2C_INTR_MASK_RESTART_DET        I2C_INT_MASK_MASK_RESTART_DET   /**< RESTART_DET interrupt */
#define LL_I2C_INTR_MASK_GEN_CALL           I2C_INT_MASK_MASK_GEN_CALL      /**< GEN_CALL interrupt */
#define LL_I2C_INTR_MASK_START_DET          I2C_INT_MASK_MASK_START_DET     /**< START_DET interrupt */
#define LL_I2C_INTR_MASK_STOP_DET           I2C_INT_MASK_MASK_STOP_DET      /**< STOP_DET interrupt */
#define LL_I2C_INTR_MASK_ACTIVITY           I2C_INT_MASK_MASK_ACTIVITY      /**< ACTIVITY interrupt */
#define LL_I2C_INTR_MASK_RX_DONE            I2C_INT_MASK_MASK_RX_DONE       /**< RX_DONE interrupt */
#define LL_I2C_INTR_MASK_TX_ABRT            I2C_INT_MASK_MASK_TX_ABORT      /**< TX_ABRT interrupt */
#define LL_I2C_INTR_MASK_RD_REQ             I2C_INT_MASK_MASK_RD_REQ        /**< RD_REQ interrupt */
#define LL_I2C_INTR_MASK_TX_EMPTY           I2C_INT_MASK_MASK_TX_EMPTY      /**< TX_EMPTY interrupt */
#define LL_I2C_INTR_MASK_TX_OVER            I2C_INT_MASK_MASK_TX_OVER       /**< TX_OVER interrupt */
#define LL_I2C_INTR_MASK_RX_FULL            I2C_INT_MASK_MASK_RX_FULL       /**< RX_FULL interrupt */
#define LL_I2C_INTR_MASK_RX_OVER            I2C_INT_MASK_MASK_RX_OVER       /**< RX_OVER interrupt */
#define LL_I2C_INTR_MASK_RX_UNDER           I2C_INT_MASK_MASK_RX_UNDER      /**< RX_UNDER interrupt */

#define LL_I2C_INTR_MASK_ALL                0x00007FFFU                     /**< All interrupt */
/** @} */

/** @defgroup I2C_LL_EC_ADDRESSING_MODE Master Addressing Mode
  * @{
  */
#define LL_I2C_ADDRESSING_MODE_7BIT         0x00000000U             /**< Master operates in 7-bit addressing mode. */
#define LL_I2C_ADDRESSING_MODE_10BIT        I2C_CTRL_ADDR_BIT_M     /**< Master operates in 10-bit addressing mode.*/
/** @} */

/** @defgroup I2C_LL_EC_OWNADDRESS Own Address Length
  * @{
  */
#define LL_I2C_OWNADDRESS_7BIT              0x00000000U             /**< Own address 1 is a 7-bit address. */
#define LL_I2C_OWNADDRESS_10BIT             I2C_CTRL_ADDR_BIT_S     /**< Own address 1 is a 10-bit address.*/
/** @} */


/** @defgroup I2C_LL_EC_GENERATE Start And Stop Generation
  * @{
  */
#define LL_I2C_CMD_SLV_NONE                 0x00000000U             /**< Slave No command. */
#define LL_I2C_CMD_MST_WRITE                0x00000000U             /**< Master write command. */
#define LL_I2C_CMD_MST_READ                 I2C_DATA_CMD_CMD        /**< Master read command.  */
#define LL_I2C_CMD_MST_GEN_STOP             I2C_DATA_CMD_STOP       /**< Master issue STOP after this command.  */
#define LL_I2C_CMD_MST_GEN_RESTART          I2C_DATA_CMD_RESTART    /**< Master issue RESTART before this command.  */
/** @} */

/** @defgroup I2C_LL_EC_SPEED_MODE Transfer Speed Mode
  * @{
  */
#define LL_I2C_SPEED_MODE_STANDARD          (0x1U << I2C_CTRL_SPEED_POS)  /**< Standard Speed mode(0 to 100 Kb/s) of operation. */
#define LL_I2C_SPEED_MODE_FAST              (0x2U << I2C_CTRL_SPEED_POS)  /**< Fast (400 Kb/s) or Fast Plus mode (1000 Kb/s) of operation. */
#define LL_I2C_SPEED_MODE_HIGH              (0x3U << I2C_CTRL_SPEED_POS)  /**< High Speed mode (3.4 Mb/s) of operation. */
/** @} */

/** @defgroup I2C_LL_EC_SPEED Transfer Speed
  * @{
  */
#define LL_I2C_SPEED_100K                   (100000ul)              /**< Standard Speed.  */
#define LL_I2C_SPEED_400K                   (400000ul)              /**< Fast Speed.      */
#define LL_I2C_SPEED_1000K                  (1000000ul)             /**< Fast Plus Speed. */
#define LL_I2C_SPEED_2000K                  (2000000ul)             /**< High Speed.      */
/** @} */

/** @defgroup I2C_LL_EC_TX_FIFO_TH TX FIFO Threshold
  * @{
  */
#define LL_I2C_TX_FIFO_TH_EMPTY              0x00000000U      /**< TX FIFO empty */
#define LL_I2C_TX_FIFO_TH_CHAR_1             0x00000001U      /**< 1 character in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_2             0x00000002U      /**< 2 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_3             0x00000003U      /**< 3 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_4             0x00000004U      /**< 4 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_5             0x00000005U      /**< 5 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_6             0x00000006U      /**< 6 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_7             0x00000007U      /**< 7 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_8             0x00000008U      /**< 9 character in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_9             0x00000009U      /**< 9 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_10            0x0000000AU      /**< 10 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_11            0x0000000BU      /**< 11 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_12            0x0000000CU      /**< 12 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_13            0x0000000DU      /**< 13 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_14            0x0000000EU      /**< 14 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_15            0x0000000FU      /**< 15 character in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_16            0x00000010U      /**< 16 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_17            0x00000011U      /**< 17 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_18            0x00000012U      /**< 18 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_19            0x00000013U      /**< 19 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_20            0x00000014U      /**< 20 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_21            0x00000015U      /**< 21 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_22            0x00000016U      /**< 22 character in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_23            0x00000017U      /**< 23 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_24            0x00000018U      /**< 24 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_25            0x00000019U      /**< 25 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_26            0x0000001AU      /**< 26 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_27            0x0000001BU      /**< 27 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_28            0x0000001CU      /**< 28 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_29            0x0000001DU      /**< 29 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_30            0x0000001EU      /**< 30 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_31            0x0000001FU      /**< 31 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_32            0x00000020U      /**< 32 characters in TX FIFO */

/** @} */

/** @defgroup I2C_LL_EC_RX_FIFO_TH RX FIFO Threshold
  * @{
  */
#define LL_I2C_RX_FIFO_TH_CHAR_1             0x00000000U     /**< 1 character in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_2             0x00000001U     /**< 2 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_3             0x00000002U     /**< 3 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_4             0x00000003U     /**< 4 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_5             0x00000004U     /**< 5 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_6             0x00000005U     /**< 6 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_7             0x00000006U     /**< 7 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_8             0x00000007U     /**< 8 character in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_9             0x00000008U     /**< 9 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_10            0x00000009U     /**< 10 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_11            0x0000000AU     /**< 11 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_12            0x0000000BU     /**< 12 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_13            0x0000000CU     /**< 13 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_14            0x0000000DU     /**< 14 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_15            0x0000000EU     /**< 15 character in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_16            0x0000000FU     /**< 16 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_17            0x00000010U     /**< 17 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_18            0x00000011U     /**< 18 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_19            0x00000012U     /**< 19 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_20            0x00000013U     /**< 20 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_21            0x00000014U     /**< 21 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_22            0x00000015U     /**< 22 character in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_23            0x00000016U     /**< 23 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_24            0x00000017U     /**< 24 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_25            0x00000018U     /**< 25 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_26            0x00000019U     /**< 26 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_27            0x0000001AU     /**< 27 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_28            0x0000001BU     /**< 28 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_29            0x0000001CU     /**< 29 character in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_30            0x0000001DU     /**< 30 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_31            0x0000001EU     /**< 31 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_32            0x0000001FU     /**< 32 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_FULL               0x00000020U     /**< RX FIFO full */
/** @} */

/** @defgroup I2C_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */

/**
  * @brief LL I2C InitStrcut default configuartion
  */
#define LL_I2C_DEFAULT_CONFIG                          \
{                                                      \
    .speed           = LL_I2C_SPEED_400K,              \
    .own_address     = 0x55U,                          \
    .own_addr_size   = LL_I2C_OWNADDRESS_7BIT,         \
}
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup I2C_LL_Exported_Macros I2C Exported Macros
  * @{
  */

/** @defgroup I2C_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in I2C register
  * @param  __instance__ I2C instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None.
  */
#define LL_I2C_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in I2C register
  * @param  __instance__ I2C instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_I2C_ReadReg(__instance__, __REG__) READ_REG(__instance__->__REG__)
/** @} */

/** @defgroup I2C_LL_EM_Exported_Macros_Helper Exported_Macros_Helper
  * @{
  */

/**
  * @brief  Compute CLK_SSL_CNT value according to Peripheral Clock and expected Speed.
  * @param  __PERIPHCLK__ Peripheral Clock frequency used for I2C instance
  * @param  __SPEED__ Speed value to achieve
  * @retval CLK_SSL_CNT value to be used for XS_SCL_HCNT, XS_SCL_LCNT registers where X can be (S, F, H)
  */
#define __LL_I2C_CONVERT_CLK_SSL_CNT(__PERIPHCLK__, __SPEED__) ((__PERIPHCLK__) / 2 / (__SPEED__))

/**
  * @brief  Get Speed Mode according to expected Speed.
  * @param  __SPEED__ Speed value to achieve
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_SPEED_MODE_STANDARD
  *         @arg @ref LL_I2C_SPEED_MODE_FAST
  *         @arg @ref LL_I2C_SPEED_MODE_HIGH
  */
#define __LL_I2C_CONVERT_SPEED_MODE(__SPEED__)      ((__SPEED__ <= LL_I2C_SPEED_100K) ? LL_I2C_SPEED_MODE_STANDARD : \
                                                     ((__SPEED__ <= LL_I2C_SPEED_1000K) ? LL_I2C_SPEED_MODE_FAST : LL_I2C_SPEED_MODE_HIGH))
/** @} */

/** @} */

/** @} */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @defgroup I2C_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup I2C_LL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Enable I2C peripheral (ENABLE = 1).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  EN             | ACTIVITY
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->EN, I2C_EN_ACTIVITY);
}

/**
  * @brief  Disable I2C peripheral (ENABLE = 0).
  * @note   When ENABLE = 0, the TX FIFO and RX FIFO get flushed.
  *         Status bits in the IC_INTR_STAT register are still active until DW_apb_i2c goes into IDLE state.
  *         If the module is transmitting, it stops as well as deletes the contents of the transmit buffer
  *         after the current transfer is complete. If the module is receiving, the DW_apb_i2c stops
  *         the current transfer at the end of the current byte and does not acknowledge the transfer..
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  EN             | ACTIVITY
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->EN, I2C_EN_ACTIVITY);
}

/**
  * @brief  Check if the I2C peripheral is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | STAT_EN
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STAT, I2C_EN_STAT_EN) == (I2C_EN_STAT_EN));
}

/**
  * @brief  Enable I2C master mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | MASTER_ENABLE
  *  CTRL           | SLAVE_DISABLE
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_master_mode(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->CTRL, I2C_CTRL_M_MODE | I2C_CTRL_S_DIS);
}

/**
  * @brief  Disable I2C master mode and enable slave mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | MASTER_ENABLE
  *  CTRL           | SLAVE_DISABLE
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_master_mode(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->CTRL, I2C_CTRL_M_MODE | I2C_CTRL_S_DIS);
}

/**
  * @brief  Check if I2C master mode is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | MASTER_ENABLE
  *  CTRL           | SLAVE_DISABLE
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_master_mode(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->CTRL, I2C_CTRL_M_MODE | I2C_CTRL_S_DIS) == (I2C_CTRL_M_MODE | I2C_CTRL_S_DIS));
}

/**
  * @brief  Enable General Call(slave mode).
  * @note   When enabled, the Address 0x00 is ACKed.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  ACK_GEN_CALL   | ACK_GEN_CALL
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_general_call(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->ACK_GEN_CALL, I2C_ACK_GEN_CALL_ACK_GEN_CALL);
}

/**
  * @brief  Disable General Call(slave mode).
  * @note   When disabled, the Address 0x00 is NACKed.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  ACK_GEN_CALL   | ACK_GEN_CALL
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_general_call(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->ACK_GEN_CALL, I2C_ACK_GEN_CALL_ACK_GEN_CALL);
}

/**
  * @brief  Check if General Call is enabled or disabled(slave mode).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  ACK_GEN_CALL   | ACK_GEN_CALL
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_general_call(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->ACK_GEN_CALL, I2C_ACK_GEN_CALL_ACK_GEN_CALL) == (I2C_ACK_GEN_CALL_ACK_GEN_CALL));
}

/**
  * @brief  Enable Master Restart.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *         This bit determines whether RESTART conditions may be sent when acting as a master.
  *         Some older slaves do not support handling RESTART conditions.
  *         When RESTART is disabled, the master is prohibited from performing the following functions:
  *         - Performing any high-speed mode operation.
  *         - Performing direction changes in combined format mode.
  *         - Performing a read operation with a 10-bit address.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | RESTART_EN
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_master_restart(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->CTRL, I2C_CTRL_RESTART_EN);
}

/**
  * @brief  Disable Master Restart.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | RESTART_EN
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_master_restart(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->CTRL, I2C_CTRL_RESTART_EN);
}

/**
  * @brief  Check if Master Restart is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | RESTART_EN
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_master_restart(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->CTRL, I2C_CTRL_RESTART_EN) == (I2C_CTRL_RESTART_EN));
}

/**
  * @brief  Enable Slave issues STOP_DET interrupt only if addressed function.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *         During a general call address, the slave does not issue the STOP_DET interrupt if
  *         STOP_DET_IF_ADDRESSED = 1'b1, even if the slave responds to the general call address
  *         by generating ACK. The STOP_DET interrupt is generated only when the transmitted
  *         address matches the slave address (SAR).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | STOP_DET_INT
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_stop_det_if_addressed(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->CTRL, I2C_CTRL_STOP_DET_INT);
}

/**
  * @brief  Disable Slave issues STOP_DET interrupt only if addressed function.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | STOP_DET_INT
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_stop_det_if_addressed(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->CTRL, I2C_CTRL_STOP_DET_INT);
}

/**
  * @brief  Hold bus when the RX FIFO if full function.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | RXFIFO_FULL_HLD
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_hold_bus_if_rx_full(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->CTRL, I2C_CTRL_RXFIFO_FULL_HLD);
}

/**
  * @brief  overflow when the RX FIFO if full function.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | RXFIFO_FULL_HLD
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_overflow_if_rx_full(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->CTRL, I2C_CTRL_RXFIFO_FULL_HLD);
}

/**
  * @brief  enable bus clear feature function.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | BUS_CLR_FEATURE
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_bus_clear_feature(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->CTRL, I2C_CTRL_BUS_CLR_FEATURE);
}

/**
  * @brief  disable bus clear feature function.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | BUS_CLR_FEATURE
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_bus_clear_feature(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->CTRL, I2C_CTRL_BUS_CLR_FEATURE);
}

/**
  * @brief  check bus clear feature is enabled function.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | BUS_CLR_FEATURE
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_bus_clear_feature(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->CTRL, I2C_CTRL_BUS_CLR_FEATURE) == (I2C_CTRL_BUS_CLR_FEATURE));
}

/**
  * @brief  Check if Slave issues STOP_DET interrupt only if addressed function is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | STOP_DET_INT
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_stop_det_if_addressed(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->CTRL, I2C_CTRL_STOP_DET_INT) == (I2C_CTRL_STOP_DET_INT));
}

/**
  * @brief  Configure the Master to transfers in 7-bit or 10-bit addressing mode.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | ADDR_BIT_M
  *
  * @param  I2Cx I2C instance.
  * @param  addressing_mode This parameter can be one of the following values:
  *         @arg @ref LL_I2C_ADDRESSING_MODE_7BIT
  *         @arg @ref LL_I2C_ADDRESSING_MODE_10BIT
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_master_addressing_mode(i2c_regs_t *I2Cx, uint32_t addressing_mode)
{
    MODIFY_REG(I2Cx->CTRL, I2C_CTRL_ADDR_BIT_M, addressing_mode);
}

/**
  * @brief  Get the Master addressing mode.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | ADDR_BIT_M
  *
  * @param  I2Cx I2C instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_ADDRESSING_MODE_7BIT
  *         @arg @ref LL_I2C_ADDRESSING_MODE_10BIT
  */
__STATIC_INLINE uint32_t ll_i2c_get_master_addressing_mode(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->CTRL, I2C_CTRL_ADDR_BIT_M));
}

/**
  * @brief  Set the Own Address.
  * @note   The register IC_CON and IC_SAR can only be programmed when the I2C is disabled (IC_ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | ADDR_BIT_S
  *  S_ADDR         | SAR
  *
  * @param  I2Cx I2C instance.
  * @param  own_address  This parameter must be a value range between 0 ~ 0x3FF(10-bit mode) or 0 ~ 0x7F(7-bit mode).
  *         Reserved address 0x00 to 0x07, or 0x78 to 0x7f should not be configured.
  * @param  own_addr_size This parameter can be one of the following values:
  *         @arg @ref LL_I2C_OWNADDRESS_7BIT
  *         @arg @ref LL_I2C_OWNADDRESS_10BIT
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_own_address(i2c_regs_t *I2Cx, uint32_t own_address, uint32_t own_addr_size)
{
    MODIFY_REG(I2Cx->CTRL, I2C_CTRL_ADDR_BIT_S, own_addr_size);
    WRITE_REG(I2Cx->S_ADDR, own_address);
}

/**
  * @brief  Set the SCL clock high-period count for standard speed.
  * @note   The register IC_SS_SCL_HCNT can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  SS_CLK_HCOUNT  | SS_SCL_HCNT
  *
  * @param  I2Cx I2C instance.
  * @param  count This parameter must be a value range between 6 ~ 0xFFF5.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_clock_high_period_ss(i2c_regs_t *I2Cx, uint32_t count)
{
    WRITE_REG(I2Cx->SS_CLK_HCOUNT, count);
}

/**
  * @brief  Get the SCL clock high-period count for standard speed.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  SS_CLK_HCOUNT  | COUNT
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x6 and 0xFFF5.
  */
__STATIC_INLINE uint32_t ll_i2c_get_clock_high_period_ss(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->SS_CLK_HCOUNT, I2C_SS_CLK_HCOUNT_COUNT));
}

/**
  * @brief  Set the SCL clock low-period count for standard speed.
  * @note   The register IC_SS_SCL_LCNT can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  SS_CLK_LCOUNT  | COUNT
  *
  * @param  I2Cx I2C instance.
  * @param  count This parameter must be a value range between 0x8 and 0xFFFF.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_clock_low_period_ss(i2c_regs_t *I2Cx, uint32_t count)
{
    WRITE_REG(I2Cx->SS_CLK_LCOUNT, count);
}

/**
  * @brief  Get the SCL clock low-period count for standard speed.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  SS_CLK_LCOUNT  | COUNT
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x8 and 0xFFFF.
  */
__STATIC_INLINE uint32_t ll_i2c_get_clock_low_period_ss(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->SS_CLK_LCOUNT, I2C_SS_CLK_LCOUNT_COUNT));
}

/**
  * @brief  Set the SCL clock high-period count for fast speed.
  * @note   The register IC_FS_SCL_HCNT can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  FS_CLK_HCOUNT  | COUNT
  *
  * @param  I2Cx I2C instance.
  * @param  count range between 0x6 and 0xFFFF.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_clock_high_period_fs(i2c_regs_t *I2Cx, uint32_t count)
{
    WRITE_REG(I2Cx->FS_CLK_HCOUNT, count);
}

/**
  * @brief  Get the SCL clock high-period count for fast speed.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  FS_CLK_HCOUNT  | COUNT
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x6 and 0xFFFF.
  */
__STATIC_INLINE uint32_t ll_i2c_get_clock_high_period_fs(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->FS_CLK_HCOUNT, I2C_FS_CLK_HCOUNT_COUNT));
}

/**
  * @brief  Set the SCL clock low-period count for fast speed.
  * @note   The register IC_FS_SCL_LCNT can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  FS_CLK_LCOUNT  | COUNT
  *
  * @param  I2Cx I2C instance.
  * @param  count range between 0x8 and 0xFFFF
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_clock_low_period_fs(i2c_regs_t *I2Cx, uint32_t count)
{
    WRITE_REG(I2Cx->FS_CLK_LCOUNT, count);
}

/**
  * @brief  Get the SCL clock low-period count for fast speed.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  FS_CLK_LCOUNT  | COUNT
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x8 and 0xFFFF.
  */
__STATIC_INLINE uint32_t ll_i2c_get_clock_low_period_fs(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->FS_CLK_LCOUNT, I2C_FS_CLK_LCOUNT_COUNT));
}

/**
  * @brief  Get the SCL clock high-period count for high speed.
  * @note   The register IC_HS_SCL_HCNT can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  HS_CLK_HCOUNT  | COUNT
  *
  * @param  I2Cx I2C instance.
  * @param  count range between 0x6 and 0xFFFF, should be larger than IC_HS_SPKLEN + 5.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_clock_high_period_hs(i2c_regs_t *I2Cx, uint32_t count)
{
    WRITE_REG(I2Cx->HS_CLK_HCOUNT, count);
}

/**
  * @brief  Get the SCL clock high-period count for high speed.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  HS_CLK_HCOUNT  | COUNT
  *
  * @param  I2Cx I2C instance.
  * @retval range between 0x6 and 0xFFFF, should be larger than IC_HS_SPKLEN + 7.
  */
__STATIC_INLINE uint32_t ll_i2c_get_clock_high_period_hs(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->HS_CLK_HCOUNT, I2C_HS_CLK_HCOUNT_COUNT));
}

/**
  * @brief  Get the SCL clock low-period count for high speed.
  * @note   The register IC_HS_SCL_LCNT can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  HS_CLK_LCOUNT  | COUNT
  *
  * @param  I2Cx I2C instance.
  * @param  count range between 0x8 and 0xFFFF
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_clock_low_period_hs(i2c_regs_t *I2Cx, uint32_t count)
{
    WRITE_REG(I2Cx->HS_CLK_LCOUNT, count);
}

/**
  * @brief  Get the SCL clock low-period count for high speed.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  HS_CLK_LCOUNT  | COUNT
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x8 and 0xFFFF
  */
__STATIC_INLINE uint32_t ll_i2c_get_clock_low_period_hs(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->HS_CLK_LCOUNT, I2C_HS_CLK_LCOUNT_COUNT));
}

/**
  * @brief  Set the spike len in fast speed mode.
  * @note   The register FS_SPKLEN can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  FS_SPKLEN      | FS_SPKLEN
  *
  * @param  I2Cx I2C instance.
  * @param  length  Spike len.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_spike_len_fs(i2c_regs_t *I2Cx, uint32_t length)
{
    MODIFY_REG(I2Cx->FS_SPKLEN, I2C_FS_SPKLEN_FS_SPKLEN, length);
}

/**
  * @brief  Get the spike len in fast speed mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  FS_SPKLEN      | FS_SPKLEN
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x2 and 0xFF.
  */
__STATIC_INLINE uint32_t ll_i2c_get_spike_len_fs(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->FS_SPKLEN, I2C_FS_SPKLEN_FS_SPKLEN));
}

/**
  * @brief  Set the spike len in high speed mode.
  * @note   The register FS_SPKLEN can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  HS_SPKLEN      | HS_SPKLEN
  *
  * @param  I2Cx I2C instance.
  * @param  length  Spike len.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_spike_len_hs(i2c_regs_t *I2Cx, uint32_t length)
{
    MODIFY_REG(I2Cx->HS_SPKLEN, I2C_HS_SPKLEN_HS_SPKLEN, length);
}

/**
  * @brief  Get the spike len in high speed mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  HS_SPKLEN      | HS_SPKLEN
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x2 and 0xFF.
  */
__STATIC_INLINE uint32_t ll_i2c_get_spike_len_hs(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->HS_SPKLEN, I2C_HS_SPKLEN_HS_SPKLEN));
}

/**
  * @brief  Set I2C Speed mode.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | SPEED
  *
  * @param  I2Cx I2C instance.
  * @param  speed_mode This parameter can be one of the following values:
  *         @arg @ref LL_I2C_SPEED_MODE_STANDARD
  *         @arg @ref LL_I2C_SPEED_MODE_FAST
  *         @arg @ref LL_I2C_SPEED_MODE_HIGH
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_speed_mode(i2c_regs_t *I2Cx, uint32_t speed_mode)
{
    MODIFY_REG(I2Cx->CTRL, I2C_CTRL_SPEED, speed_mode);
}

/**
  * @brief  Get I2C Speed mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CTRL           | SPEED
  *
  * @param  I2Cx I2C instance.
  * @retval Value can be one of the following values:
  *         @arg @ref LL_I2C_SPEED_MODE_STANDARD
  *         @arg @ref LL_I2C_SPEED_MODE_FAST
  *         @arg @ref LL_I2C_SPEED_MODE_HIGH
  */
__STATIC_INLINE uint32_t ll_i2c_get_speed_mode(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->CTRL, I2C_CTRL_SPEED));
}

/**
  * @brief  Set I2C High Speed Master Code Address.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  M_HS_ADDR      | HS_MAR
  *
  * @param  I2Cx I2C instance.
  * @param  code HS mode master code, range between 0x00 and 0x07.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_high_speed_master_code(i2c_regs_t *I2Cx, uint32_t code)
{
    WRITE_REG(I2Cx->M_HS_ADDR, code);
}

/**
  * @brief  Get I2C Speed mode.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  M_HS_ADDR      | HS_MAR
  *
  * @param  I2Cx I2C instance.
  * @retval Returned value range between 0x00 and 0x07.
  */
__STATIC_INLINE uint32_t ll_i2c_get_high_speed_master_code(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->M_HS_ADDR, I2C_M_HS_ADDR_HS_ADDR));
}

/**
  * @brief  Set the required transmit SDA hold time in units of ic_clk period.
  * @note   The register IC_SDA_HOLD can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  SDA_HOLD       | TX_HOLD
  *
  * @param  I2Cx I2C instance.
  * @param  time SDA Tx hold time in units of ic_clk period.
  *         Time should range between 1 and (N_SCL_LOW - 2) in master mode or 7 and (N_SCL_LOW - 2) in slave mode.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_data_tx_hold_time(i2c_regs_t *I2Cx, uint32_t time)
{
    MODIFY_REG(I2Cx->SDA_HOLD, I2C_SDA_HOLD_TX_HOLD, time << I2C_SDA_HOLD_TX_HOLD_POS);
}

/**
  * @brief  Get the required transmit SDA hold time in units of ic_clk period.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  SDA_HOLD       | TX_HOLD
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 1 and (N_SCL_LOW - 2) in master mode or 7 and (N_SCL_LOW - 2) in slave mode
  */
__STATIC_INLINE uint32_t ll_i2c_get_data_tx_hold_time(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->SDA_HOLD, I2C_SDA_HOLD_TX_HOLD) >> I2C_SDA_HOLD_TX_HOLD_POS);
}

/**
  * @brief  Set the required receive SDA hold time in units of ic_clk period.
  * @note   The register IC_SDA_HOLD can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  SDA_HOLD       | RX_HOLD
  *
  * @param  I2Cx I2C instance.
  * @param  time SDA Tx hold time in units of ic_clk period.
  *         Time should range between 1 and (N_SCL_LOW - 2) in master mode or 7 and (N_SCL_LOW - 2) in slave mode.
  * @retval Value between Min_Data=0x0 and Max_Data=0xF
  */
__STATIC_INLINE void ll_i2c_set_data_rx_hold_time(i2c_regs_t *I2Cx, uint32_t time)
{
    MODIFY_REG(I2Cx->SDA_HOLD, I2C_SDA_HOLD_RX_HOLD, time << I2C_SDA_HOLD_RX_HOLD_POS);
}

/**
  * @brief  Get the required receive SDA hold time in units of ic_clk period.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  SDA_HOLD       | RX_HOLD
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 1 and (N_SCL_LOW - 2) in master mode or 7 and (N_SCL_LOW - 2) in slave mode
  */
__STATIC_INLINE uint32_t ll_i2c_get_data_rx_hold_time(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->SDA_HOLD, I2C_SDA_HOLD_RX_HOLD) >> I2C_SDA_HOLD_RX_HOLD_POS);
}

/**
  * @brief  Set the SDA setup time when operating as a slave transmitter.
  * @note   The register IC_SDA_SETUP can only be programmed when the I2C is disabled (ENABLE = 0).
  *         The length of setup time is calculated using [(IC_SDA_SETUP - 1) * (ic_clk_period)], so if the
  *         user requires 10 ic_clk periods of setup time, they should program a value of 11.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  SDA_SETUP      | SETUP
  *
  * @param  I2Cx I2C instance.
  * @param  time SDA data setup time in units of ic_clk period, range between 2 ~ 0xFF.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_data_setup_time(i2c_regs_t *I2Cx, uint32_t time)
{
    MODIFY_REG(I2Cx->SDA_SETUP, I2C_SDA_SETUP_SETUP, time);
}

/**
  * @brief  Get the SDA setup time when operating as a slave transmitter.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  SDA_SETUP      | SETUP
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x02 and 0xFF.
  */
__STATIC_INLINE uint32_t ll_i2c_get_data_setup_time(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->SDA_SETUP, I2C_SDA_SETUP_SETUP));
}

/**
  * @brief  Set threshold of entries (or below) that trigger the TX_EMPTY interrupt
  * @note   TX FIFO threshold only can be configured after FIFO was enabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  TX_FIFO_THD    | THD
  *
  * @param  I2Cx I2C instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_I2C_TX_FIFO_TH_EMPTY
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_1
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_2
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_3
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_4
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_5
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_6
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_7
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_8
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_9
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_10
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_11
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_12
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_13
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_14
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_15
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_16
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_17
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_18
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_19
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_20
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_21
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_22
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_23
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_24
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_25
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_26
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_27
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_28
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_29
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_30
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_31
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_32
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_tx_fifo_threshold(i2c_regs_t *I2Cx, uint32_t threshold)
{
    WRITE_REG(I2Cx->TX_FIFO_THD, threshold);
}

/**
  * @brief  Get threshold of TX FIFO that triggers an THRE interrupt
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  TX_FIFO_THD    | THD
  *
  * @param  I2Cx I2C instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_TX_FIFO_TH_EMPTY
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_1
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_2
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_3
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_4
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_5
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_6
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_7
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_8
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_9
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_10
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_11
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_12
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_13
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_14
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_15
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_16
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_17
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_18
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_19
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_20
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_21
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_22
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_23
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_24
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_25
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_26
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_27
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_28
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_29
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_30
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_31
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_32
  */
__STATIC_INLINE uint32_t ll_i2c_get_tx_fifo_threshold(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->TX_FIFO_THD, I2C_TX_FIFO_THD_THD));
}

/**
  * @brief  Set threshold of RX FIFO that triggers an RDA interrupt
  * @note   TX FIFO threshold only can be configured after FIFO was enabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RX_FIFO_THD    | THD
  *
  * @param  I2Cx I2C instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_1
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_2
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_3
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_4
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_5
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_6
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_7
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_8
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_9
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_10
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_11
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_12
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_13
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_14
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_15
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_16
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_17
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_18
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_19
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_20
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_21
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_22
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_23
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_24
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_25
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_26
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_27
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_28
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_29
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_30
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_31
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_32
  *         @arg @ref LL_I2C_RX_FIFO_TH_FULL
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_rx_fifo_threshold(i2c_regs_t *I2Cx, uint32_t threshold)
{
    WRITE_REG(I2Cx->RX_FIFO_THD, threshold);
}

/**
  * @brief  Get threshold of RX FIFO that triggers an RDA interrupt
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RX_FIFO_THD    | THD
  *
  * @param  I2Cx I2C instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_1
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_2
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_3
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_4
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_5
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_6
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_7
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_8
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_9
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_10
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_11
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_12
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_13
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_14
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_15
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_16
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_17
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_18
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_19
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_20
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_21
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_22
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_23
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_24
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_25
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_26
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_27
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_28
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_29
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_30
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_31
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_32
  *         @arg @ref LL_I2C_RX_FIFO_TH_FULL
  */
__STATIC_INLINE uint32_t ll_i2c_get_rx_fifo_threshold(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->RX_FIFO_THD, I2C_RX_FIFO_THD_THD));
}

/**
  * @brief  Get FIFO Transmission Level
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RX_FIFO_THD    | LEVEL
  *
  * @param  I2Cx I2C instance
  * @retval Value range between 0x0 and 0x1F.
  */
__STATIC_INLINE uint32_t ll_i2c_get_tx_fifo_level(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->TX_FIFO_LEVEL, I2C_TX_FIFO_LEVEL_LEVEL));
}

/**
  * @brief  Get FIFO reception Level
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RX_FIFO_THD    | LEVEL
  *
  * @param  I2Cx I2C instance
  * @retval Value range between 0x0 and 0x1F.
  */
__STATIC_INLINE uint32_t ll_i2c_get_rx_fifo_level(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->RX_FIFO_LEVEL, I2C_RX_FIFO_LEVEL_LEVEL));
}

/**
  * @brief  Master initates the SDA stuck at low recovery mechanism.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  EN             | SDA_STUCK_RECOVERY
  *
  * @param  I2Cx I2C instance
  */
__STATIC_INLINE void ll_i2c_enable_sda_stuck_recovery(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->EN, I2C_EN_SDA_STUCK_RECOVERY);
}

/**
  * @brief  Master disabled the SDA stuck at low recovery mechanism
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  EN             | SDA_STUCK_RECOVERY
  *
  * @param  I2Cx I2C instance
  */
__STATIC_INLINE void ll_i2c_disable_sda_stuck_recovery(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->EN, I2C_EN_SDA_STUCK_RECOVERY);
}

/**
  * @brief  the SDA stuck at low recovery mechanism is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  EN             | SDA_STUCK_RECOVERY
  *
  * @param  I2Cx I2C instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_sda_stuck_recovery(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->EN, I2C_EN_SDA_STUCK_RECOVERY) == (I2C_EN_SDA_STUCK_RECOVERY));
}

/**
  * @brief  Enable DMA reception requests.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  EN             | ABORT
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_transfer_abort(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->EN, I2C_EN_ABORT);
}

/**
  * @brief  Check if DMA reception requests are enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  EN             | ABORT
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_transfer_abort(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->EN, I2C_EN_ABORT) == (I2C_EN_ABORT));
}

/**
  * @brief  Get the transmit abort source.
  * @note   This can be used to retrieve source of TX_ABRT interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  TX_ABORT_SRC   | ABRT_USER_ABRT
  *  TX_ABORT_SRC   | ABRT_SLVRD_INTX
  *  TX_ABORT_SRC   | ABRT_SLV_ARBLOST
  *  TX_ABORT_SRC   | ABRT_SLVFLUSH_TXFIFO
  *  TX_ABORT_SRC   | ABRT_ARB_LOST
  *  TX_ABORT_SRC   | ABRT_MST_DIS
  *  TX_ABORT_SRC   | ABRT_10B_RD_NORSTRT
  *  TX_ABORT_SRC   | ABRT_SBYTE_NORSTRT
  *  TX_ABORT_SRC   | ABRT_HS_NORSTRT
  *  TX_ABORT_SRC   | ABRT_SBYTE_ACKDET
  *  TX_ABORT_SRC   | ABRT_HS_ACKDET
  *  TX_ABORT_SRC   | ABRT_GCALL_READ
  *  TX_ABORT_SRC   | ABRT_GCALL_NOACK
  *  TX_ABORT_SRC   | ABRT_TXDATA_NOACK
  *  TX_ABORT_SRC   | ABRT_10ADDR2_NOACK
  *  TX_ABORT_SRC   | ABRT_10ADDR1_NOACK
  *  TX_ABORT_SRC   | ABRT_7B_ADDR_NOACK
  *
  * @param  I2Cx I2C instance
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_I2C_ABRT_USER_ABRT
  *         @arg @ref LL_I2C_ABRT_SLVRD_INTX
  *         @arg @ref LL_I2C_ABRT_SLV_ARBLOST
  *         @arg @ref LL_I2C_ABRT_SLVFLUSH_TXFIFO
  *         @arg @ref LL_I2C_ABRT_ARB_LOST
  *         @arg @ref LL_I2C_ABRT_MST_DIS
  *         @arg @ref LL_I2C_ABRT_10B_RD_NORSTRT
  *         @arg @ref LL_I2C_ABRT_SBYTE_NORSTRT
  *         @arg @ref LL_I2C_ABRT_HS_NORSTRT
  *         @arg @ref LL_I2C_ABRT_SBYTE_ACKDET
  *         @arg @ref LL_I2C_ABRT_HS_ACKDET
  *         @arg @ref LL_I2C_ABRT_GCALL_READ
  *         @arg @ref LL_I2C_ABRT_GCALL_NOACK
  *         @arg @ref LL_I2C_ABRT_TXDATA_NOACK
  *         @arg @ref LL_I2C_ABRT_10ADDR2_NOACK
  *         @arg @ref LL_I2C_ABRT_10ADDR1_NOACK
  *         @arg @ref LL_I2C_ABRT_7B_ADDR_NOACK
  *
  * @note   @arg @ref LL_I2C_ABRT_TX_FLUSH_CNT can be used as a mask to get the
  *         number of Tx FIFO Data Commands which are flushed due to TX_ABRT
  *         interrupt.
  */
__STATIC_INLINE uint32_t ll_i2c_get_abort_source(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_REG(I2Cx->TX_ABORT_SRC) & (~I2C_TX_ABORT_SRC_TX_FLUSH_CNT));
}

/**
  * @brief  Get the number of Tx FIFO Data Commands which are flushed due to TX_ABRT interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  TX_ABORT_SRC   | TX_FLUSH_CNT
  *
  * @param  I2Cx I2C instance
  * @retval Tx flush count.
  */
__STATIC_INLINE uint32_t ll_i2c_get_tx_flush_count(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->TX_ABORT_SRC, I2C_TX_ABORT_SRC_TX_FLUSH_CNT) >> I2C_TX_ABORT_SRC_TX_FLUSH_CNT_POS);
}

/** @} */

/** @defgroup I2C_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable specified interrupts.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | MASK_GEN_CALL
  *  INT_MASK       | MASK_START_DET
  *  INT_MASK       | MASK_STOP_DET
  *  INT_MASK       | MASK_ACTIVITY
  *  INT_MASK       | MASK_RX_DONE
  *  INT_MASK       | MASK_TX_ABRT
  *  INT_MASK       | MASK_RD_REQ
  *  INT_MASK       | MASK_TX_EMPTY
  *  INT_MASK       | MASK_TX_OVER
  *  INT_MASK       | MASK_RX_FULL
  *  INT_MASK       | MASK_RX_OVER
  *  INT_MASK       | MASK_RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @param  mask This parameter can be a combination of the following values:
  *         @arg @ref LL_I2C_INTR_MASK_GEN_CALL
  *         @arg @ref LL_I2C_INTR_MASK_START_DET
  *         @arg @ref LL_I2C_INTR_MASK_STOP_DET
  *         @arg @ref LL_I2C_INTR_MASK_ACTIVITY
  *         @arg @ref LL_I2C_INTR_MASK_RX_DONE
  *         @arg @ref LL_I2C_INTR_MASK_TX_ABRT
  *         @arg @ref LL_I2C_INTR_MASK_RD_REQ
  *         @arg @ref LL_I2C_INTR_MASK_TX_EMPTY
  *         @arg @ref LL_I2C_INTR_MASK_TX_OVER
  *         @arg @ref LL_I2C_INTR_MASK_RX_FULL
  *         @arg @ref LL_I2C_INTR_MASK_RX_OVER
  *         @arg @ref LL_I2C_INTR_MASK_RX_UNDER
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it(i2c_regs_t *I2Cx, uint32_t mask)
{
    SET_BITS(I2Cx->INT_MASK, mask);
}

/**
  * @brief  Disable specified interrupts.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | MASK_GEN_CALL
  *  INT_MASK       | MASK_START_DET
  *  INT_MASK       | MASK_STOP_DET
  *  INT_MASK       | MASK_ACTIVITY
  *  INT_MASK       | MASK_RX_DONE
  *  INT_MASK       | MASK_TX_ABRT
  *  INT_MASK       | MASK_RD_REQ
  *  INT_MASK       | MASK_TX_EMPTY
  *  INT_MASK       | MASK_TX_OVER
  *  INT_MASK       | MASK_RX_FULL
  *  INT_MASK       | MASK_RX_OVER
  *  INT_MASK       | MASK_RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @param  mask This parameter can be a combination of the following values:
  *         @arg @ref LL_I2C_INTR_MASK_GEN_CALL
  *         @arg @ref LL_I2C_INTR_MASK_START_DET
  *         @arg @ref LL_I2C_INTR_MASK_STOP_DET
  *         @arg @ref LL_I2C_INTR_MASK_ACTIVITY
  *         @arg @ref LL_I2C_INTR_MASK_RX_DONE
  *         @arg @ref LL_I2C_INTR_MASK_TX_ABRT
  *         @arg @ref LL_I2C_INTR_MASK_RD_REQ
  *         @arg @ref LL_I2C_INTR_MASK_TX_EMPTY
  *         @arg @ref LL_I2C_INTR_MASK_TX_OVER
  *         @arg @ref LL_I2C_INTR_MASK_RX_FULL
  *         @arg @ref LL_I2C_INTR_MASK_RX_OVER
  *         @arg @ref LL_I2C_INTR_MASK_RX_UNDER
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it(i2c_regs_t *I2Cx, uint32_t mask)
{
    CLEAR_BITS(I2Cx->INT_MASK, mask);
}

/**
  * @brief  Check if the specified interrupts are enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | MASK_GEN_CALL
  *  INT_MASK       | MASK_START_DET
  *  INT_MASK       | MASK_STOP_DET
  *  INT_MASK       | MASK_ACTIVITY
  *  INT_MASK       | MASK_RX_DONE
  *  INT_MASK       | MASK_TX_ABRT
  *  INT_MASK       | MASK_RD_REQ
  *  INT_MASK       | MASK_TX_EMPTY
  *  INT_MASK       | MASK_TX_OVER
  *  INT_MASK       | MASK_RX_FULL
  *  INT_MASK       | MASK_RX_OVER
  *  INT_MASK       | MASK_RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @param  mask This parameter can be a combination of the following values:
  *         @arg @ref LL_I2C_INTR_MASK_GEN_CALL
  *         @arg @ref LL_I2C_INTR_MASK_START_DET
  *         @arg @ref LL_I2C_INTR_MASK_STOP_DET
  *         @arg @ref LL_I2C_INTR_MASK_ACTIVITY
  *         @arg @ref LL_I2C_INTR_MASK_RX_DONE
  *         @arg @ref LL_I2C_INTR_MASK_TX_ABRT
  *         @arg @ref LL_I2C_INTR_MASK_RD_REQ
  *         @arg @ref LL_I2C_INTR_MASK_TX_EMPTY
  *         @arg @ref LL_I2C_INTR_MASK_TX_OVER
  *         @arg @ref LL_I2C_INTR_MASK_RX_FULL
  *         @arg @ref LL_I2C_INTR_MASK_RX_OVER
  *         @arg @ref LL_I2C_INTR_MASK_RX_UNDER
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it(i2c_regs_t *I2Cx, uint32_t mask)
{
    return (READ_BITS(I2Cx->INT_MASK, mask) == (mask));
}

/**
  * @brief  Enable SCL_STUCK_AT_LOW interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | SCL_STUCK_AT_LOW
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_scl_stuck_at_low(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_SCL_STUCKLOW);
}
/**
  * @brief  Disable SCL_STUCK_AT_LOW interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | SCL_STUCK_AT_LOW
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_scl_stuck_at_low(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_SCL_STUCKLOW);
}
/**
  * @brief  Check if the SCL_STUCK_AT_LOW Interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | SCL_STUCK_AT_LOW
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_scl_stuck_at_low(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_SCL_STUCKLOW) == (I2C_INT_MASK_MASK_SCL_STUCKLOW));
}


/**
  * @brief  Enable MASTER_ON_HOLD interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | M_HOLD
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_master_on_hold(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_M_HOLD);
}

/**
  * @brief  Disable MASTER_ON_HOLD interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | M_HOLD
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_master_on_hold(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_M_HOLD);
}

/**
  * @brief  Check if the MASTER_ON_HOLD Interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | M_HOLD
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_master_on_hold(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_M_HOLD) == (I2C_INT_MASK_MASK_M_HOLD));
}

/**
  * @brief  Enable RESTART_DET interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RESTART_DET
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_restart_det(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RESTART_DET);
}

/**
  * @brief  Disable RESTART_DET interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RESTART_DET
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_restart_det(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RESTART_DET);
}

/**
  * @brief  Check if the RESTART_DET Interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RESTART_DET
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_restart_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RESTART_DET) == (I2C_INT_MASK_MASK_RESTART_DET));
}

/**
  * @brief  Enable GEN_CALL interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | GEN_CALL
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_gen_call(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_GEN_CALL);
}

/**
  * @brief  Disable GEN_CALL interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | GEN_CALL
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_gen_call(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_GEN_CALL);
}

/**
  * @brief  Check if GEN_CALL interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | GEN_CALL
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_gen_call(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_GEN_CALL) == (I2C_INT_MASK_MASK_GEN_CALL));
}

/**
  * @brief  Enable START_DET received interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | START_DET
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_start_det(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_START_DET);
}

/**
  * @brief  Disable START_DET received interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | START_DET
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_start_det(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_START_DET);
}

/**
  * @brief  Check if START_DET received interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | START_DET
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_start_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_START_DET) == (I2C_INT_MASK_MASK_START_DET));
}

/**
  * @brief  Enable STOP_DET interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | STOP_DET
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_stop_det(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_STOP_DET);
}

/**
  * @brief  Disable STOP_DET interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | STOP_DET
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_stop_det(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_STOP_DET);
}

/**
  * @brief  Check if STOP_DET interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | STOP_DET
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_stop_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_STOP_DET) == (I2C_INT_MASK_MASK_STOP_DET));
}

/**
  * @brief  Enable ACTIVITY interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | ACTIVITY
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_activity(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_ACTIVITY);
}

/**
  * @brief  Disable ACTIVITY interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | ACTIVITY
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_activity(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_ACTIVITY);
}

/**
  * @brief  Check if ACTIVITY interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | ACTIVITY
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_activity(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_ACTIVITY) == (I2C_INT_MASK_MASK_ACTIVITY));
}

/**
  * @brief  Enable RX_DONE interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RX_DONE
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE void ll_i2c_enable_it_rx_done(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RX_DONE);
}

/**
  * @brief  Disable RX_DONE interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RX_DONE
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_rx_done(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RX_DONE);
}

/**
  * @brief  Check if RX_DONE interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RX_DONE
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enable_it_rx_done(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RX_DONE) == (I2C_INT_MASK_MASK_RX_DONE));
}

/**
  * @brief  Enable TX_ABRT interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | TX_ABORT
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_rx_abort(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_TX_ABORT);
}

/**
  * @brief  Disable TX_ABRT interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | TX_ABORT
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_tx_abort(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_TX_ABORT);
}

/**
  * @brief  Check if TX_ABRT interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | TX_ABORT
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_tx_abort(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_TX_ABORT) == (I2C_INT_MASK_MASK_TX_ABORT));
}

/**
  * @brief  Enable RD_REQ interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RD_REQ
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_read_req(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RD_REQ);
}

/**
  * @brief  Disable RD_REQ interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RD_REQ
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_read_req(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RD_REQ);
}

/**
  * @brief  Check if RD_REQ interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RD_REQ
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_read_req(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RD_REQ) == (I2C_INT_MASK_MASK_RD_REQ));
}

/**
  * @brief  Enable TX_EMPTY interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | TX_EMPTY
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_tx_empty(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_TX_EMPTY);
}

/**
  * @brief  Disable TX_EMPTY interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | TX_EMPTY
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_tx_empty(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_TX_EMPTY);
}

/**
  * @brief  Check if TX_EMPTY interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | TX_EMPTY
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_tx_empty(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_TX_EMPTY) == (I2C_INT_MASK_MASK_TX_EMPTY));
}

/**
  * @brief  Enable TX_OVER interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | TX_OVER
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_tx_over(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_TX_OVER);
}

/**
  * @brief  Disable TX_OVER interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | TX_OVER
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_tx_over(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_TX_OVER);
}

/**
  * @brief  Check if TX_OVER interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | TX_OVER
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_tx_over(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_TX_OVER) == (I2C_INT_MASK_MASK_TX_OVER));
}

/**
  * @brief  Enable RX_FULL interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RX_FULL
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_rx_full(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RX_FULL);
}

/**
  * @brief  Disable RX_FULL interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RX_FULL
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disbale_it_rx_full(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RX_FULL);
}

/**
  * @brief  Check if RX_FULL interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RX_FULL
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_i2c_ls_enabled_it_rx_full(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RX_FULL) == (I2C_INT_MASK_MASK_RX_FULL));
}

/**
  * @brief  Enable RX_OVER interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RX_OVER
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_rx_over(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RX_OVER);
}

/**
  * @brief  Disable RX_OVER interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RX_OVER
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_rx_over(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RX_OVER);
}

/**
  * @brief  Check if RX_OVER interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RX_OVER
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_rx_over(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RX_OVER) == (I2C_INT_MASK_MASK_RX_OVER));
}

/**
  * @brief  Enable RX_UNDER interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_rx_under(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RX_UNDER);
}

/**
  * @brief  Disable RX_UNDER interrupt.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_rx_under(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RX_UNDER);
}

/**
  * @brief  Check if RX_UNDER interrupt is enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_MASK       | RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_rx_under(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_MASK, I2C_INT_MASK_MASK_RX_UNDER) == (I2C_INT_MASK_MASK_RX_UNDER));
}

/** @} */

/** @defgroup I2C_LL_EF_FLAG_management FLAG_management
  * @{
  */

/**
  * @brief  Get I2C interrupt flags
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | MST_ON_HOLD
  *  INT_STAT       | RESTART_DET
  *  INT_STAT       | GEN_CALL
  *  INT_STAT       | START_DET
  *  INT_STAT       | STOP_DET
  *  INT_STAT       | ACTIVITY
  *  INT_STAT       | RX_DONE
  *  INT_STAT       | TX_ABRT
  *  INT_STAT       | RD_REQ
  *  INT_STAT       | TX_EMPTY
  *  INT_STAT       | TX_OVER
  *  INT_STAT       | RX_FULL
  *  INT_STAT       | RX_OVER
  *  INT_STAT       | RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_I2C_INTR_STAT_MST_ON_HOLD
  *         @arg @ref LL_I2C_INTR_STAT_RESTART_DET
  *         @arg @ref LL_I2C_INTR_STAT_GEN_CALL
  *         @arg @ref LL_I2C_INTR_STAT_START_DET
  *         @arg @ref LL_I2C_INTR_STAT_STOP_DET
  *         @arg @ref LL_I2C_INTR_STAT_ACTIVITY
  *         @arg @ref LL_I2C_INTR_STAT_RX_DONE
  *         @arg @ref LL_I2C_INTR_STAT_TX_ABRT
  *         @arg @ref LL_I2C_INTR_STAT_RD_REQ
  *         @arg @ref LL_I2C_INTR_STAT_TX_EMPTY
  *         @arg @ref LL_I2C_INTR_STAT_TX_OVER
  *         @arg @ref LL_I2C_INTR_STAT_RX_FULL
  *         @arg @ref LL_I2C_INTR_STAT_RX_OVER
  *         @arg @ref LL_I2C_INTR_STAT_RX_UNDER
  */
__STATIC_INLINE uint32_t ll_i2c_get_it_flag(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_REG(I2Cx->INT_STAT));
}

/**
  * @brief  Get I2C RAW interrupt flags
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | MST_ON_HOLD
  *  RAW_INT_STAT   | RESTART_DET
  *  RAW_INT_STAT   | GEN_CALL
  *  RAW_INT_STAT   | START_DET
  *  RAW_INT_STAT   | STOP_DET
  *  RAW_INT_STAT   | ACTIVITY
  *  RAW_INT_STAT   | RX_DONE
  *  RAW_INT_STAT   | TX_ABRT
  *  RAW_INT_STAT   | RD_REQ
  *  RAW_INT_STAT   | TX_EMPTY
  *  RAW_INT_STAT   | TX_OVER
  *  RAW_INT_STAT   | RX_FULL
  *  RAW_INT_STAT   | RX_OVER
  *  RAW_INT_STAT   | RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_I2C_INTR_STAT_MST_ON_HOLD
  *         @arg @ref LL_I2C_INTR_STAT_RESTART_DET
  *         @arg @ref LL_I2C_INTR_STAT_GEN_CALL
  *         @arg @ref LL_I2C_INTR_STAT_START_DET
  *         @arg @ref LL_I2C_INTR_STAT_STOP_DET
  *         @arg @ref LL_I2C_INTR_STAT_ACTIVITY
  *         @arg @ref LL_I2C_INTR_STAT_RX_DONE
  *         @arg @ref LL_I2C_INTR_STAT_TX_ABRT
  *         @arg @ref LL_I2C_INTR_STAT_RD_REQ
  *         @arg @ref LL_I2C_INTR_STAT_TX_EMPTY
  *         @arg @ref LL_I2C_INTR_STAT_TX_OVER
  *         @arg @ref LL_I2C_INTR_STAT_RX_FULL
  *         @arg @ref LL_I2C_INTR_STAT_RX_OVER
  *         @arg @ref LL_I2C_INTR_STAT_RX_UNDER
  */
__STATIC_INLINE uint32_t ll_i2c_get_raw_it_flag(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_REG(I2Cx->RAW_INT_STAT));
}

__STATIC_INLINE uint32_t ll_i2c_is_active_flag_scl_stuck_at_low(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_SCL_STUCKLOW) == (I2C_INT_STAT_RAW_SCL_STUCKLOW));
}

__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_scl_stuck_at_low(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_SCL_STUCKLOW) == (I2C_RAW_INT_STAT_SCL_STUCKLOW));
}

/**
  * @brief  Indicate the status of MST_ON_HOLD flag.
  * @note   RESET: Clear default value.
  *         SET  : When MST_ON_HOLD interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | M_HOLD
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_master_on_hold(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_M_HOLD) == (I2C_INT_STAT_RAW_M_HOLD));
}

/**
  * @brief  Indicate the status of RAW_MST_ON_HOLD flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked MST_ON_HOLD interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | M_HOLD
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_master_on_hold(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_M_HOLD) == (I2C_RAW_INT_STAT_M_HOLD));
}

/**
  * @brief  Indicate the status of RESTART_DET flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked RESTART_DET interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | RESTART_DET
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_restart_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_RESTART_DET) == (I2C_INT_STAT_RAW_RESTART_DET));
}

/**
  * @brief  Indicate the status of RAW_RESTART_DET flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked RESTART_DET interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | RESTART_DET
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_restart_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_RESTART_DET) == (I2C_RAW_INT_STAT_RESTART_DET));
}

/**
  * @brief  Indicate the status of GEN_CALL flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked GEN_CALL interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | GEN_CALL
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_gen_call(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_GEN_CALL) == (I2C_INT_STAT_RAW_GEN_CALL));
}

/**
  * @brief  Indicate the status of RAW_GEN_CALL flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked GEN_CALL interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | GEN_CALL
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_gen_call(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_GEN_CALL) == (I2C_RAW_INT_STAT_GEN_CALL));
}

/**
  * @brief  Indicate the status of START_DET flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked START_DET interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | START_DET
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_start_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_START_DET) == (I2C_INT_STAT_RAW_START_DET));
}

/**
  * @brief  Indicate the status of RAW_START_DET flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked START_DET interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | START_DET
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_start_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_START_DET) == (I2C_RAW_INT_STAT_START_DET));
}

/**
  * @brief  Indicate the status of STOP_DET flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked STOP_DET interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | STOP_DET
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_stop_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_STOP_DET) == (I2C_INT_STAT_RAW_STOP_DET));
}

/**
  * @brief  Indicate the status of RAW_STOP_DET flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked STOP_DET interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | STOP_DET
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_stop_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_STOP_DET) == (I2C_RAW_INT_STAT_STOP_DET));
}

/**
  * @brief  Indicate the status of ACTIVITY flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked ACTIVITY interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | ACTIVITY
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_activity(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_ACTIVITY) == (I2C_INT_STAT_RAW_ACTIVITY));
}

/**
  * @brief  Indicate the status of RAW_ACTIVITY flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked ACTIVITY interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | ACTIVITY
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_activity(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_ACTIVITY) == (I2C_RAW_INT_STAT_ACTIVITY));
}

/**
  * @brief  Indicate the status of RX_DONE flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked RX_DONE interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | RX_DONE
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_rx_done(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_RX_DONE) == (I2C_INT_STAT_RAW_RX_DONE));
}

/**
  * @brief  Indicate the status of RAW_RX_DONE flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked RX_DONE interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | RX_DONE
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_rx_done(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_RX_DONE) == (I2C_RAW_INT_STAT_RX_DONE));
}

/**
  * @brief  Indicate the status of TX_ABRT flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked TX_ABRT interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | TX_ABORT
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_tx_abort(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_TX_ABORT) == (I2C_INT_STAT_RAW_TX_ABORT));
}

/**
  * @brief  Indicate the status of RAW_TX_ABRT flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked TX_ABRT interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | TX_ABORT
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_tx_abort(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_TX_ABORT) == (I2C_RAW_INT_STAT_TX_ABORT));
}

/**
  * @brief  Indicate the status of RD_REQ flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked RD_REQ interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | RD_REQ
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_read_req(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_RD_REQ) == (I2C_INT_STAT_RAW_RD_REQ));
}

/**
  * @brief  Indicate the status of RAW_RD_REQ flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked RD_REQ interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | RD_REQ
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_read_req(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_RD_REQ) == (I2C_RAW_INT_STAT_RD_REQ));
}

/**
  * @brief  Indicate the status of TX_EMPTY flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked TX_EMPTY interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | TX_EMPTY
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_tx_empty(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_TX_EMPTY) == (I2C_INT_STAT_RAW_TX_EMPTY));
}

/**
  * @brief  Indicate the status of RAW_TX_EMPTY flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked TX_EMPTY interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | TX_EMPTY
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_tx_empty(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_TX_EMPTY) == (I2C_RAW_INT_STAT_TX_EMPTY));
}

/**
  * @brief  Indicate the status of TX_OVER flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked TX_OVER interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | TX_OVER
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_tx_over(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_TX_OVER) == (I2C_INT_STAT_RAW_TX_OVER));
}

/**
  * @brief  Indicate the status of RAW_TX_OVER flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked TX_OVER interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | TX_OVER
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_tx_over(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_TX_OVER) == (I2C_RAW_INT_STAT_TX_OVER));
}

/**
  * @brief  Indicate the status of RX_FULL flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked RX_FULL interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | RX_FULL
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_rx_full(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_RX_FULL) == (I2C_INT_STAT_RAW_RX_FULL));
}

/**
  * @brief  Indicate the status of RAW_RX_FULL flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked RX_FULL interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | RX_FULL
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_rx_full(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_RX_FULL) == (I2C_RAW_INT_STAT_RX_FULL));
}

/**
  * @brief  Indicate the status of RX_OVER flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked RX_OVER interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | RX_OVER
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_rx_over(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_STAT_RAW_RX_OVER) == (I2C_INT_STAT_RAW_RX_OVER));
}

/**
  * @brief  Indicate the status of RAW_RX_OVER flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked RX_OVER interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | RX_OVER
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_rx_over(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_RX_OVER) == (I2C_RAW_INT_STAT_RX_OVER));
}

/**
  * @brief  Indicate the status of RX_UNDER flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked RX_UNDER interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  INT_STAT       | RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_rx_under(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INT_STAT, I2C_INT_MASK_MASK_RX_UNDER) == (I2C_INT_MASK_MASK_RX_UNDER));
}

/**
  * @brief  Indicate the status of RAW_RX_UNDER flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked RX_UNDER interrupt is actived.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  RAW_INT_STAT   | RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_rx_under(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INT_STAT, I2C_RAW_INT_STAT_RX_UNDER) == (I2C_RAW_INT_STAT_RX_UNDER));
}

/**
  * @brief  Clear the combined interrupt, all individual interrupts, and the IC_TX_ABRT_SOURCE register
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CLR_INT        | CLR_INTR
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_intr(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_INT);
    (void) tmpreg;
}

__STATIC_INLINE void ll_i2c_clear_flag_scl_stuck_det(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_SCL_STUCK_DET);
    (void) tmpreg;
}

/**
  * @brief  Clear GEN_CALL flag.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CLR_GEN_CALL   | CLR_GEN_CALL
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_gen_call(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_GEN_CALL);
    (void) tmpreg;
}

/**
  * @brief  Clear START_DET flag.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CLR_START_DET  | CLR_START_DET
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_start_det(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_START_DET);
    (void) tmpreg;
}

/**
  * @brief  Clear STOP_DET flag.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CLR_STOP_DET   | CLR_STOP_DET
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_stop_det(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_STOP_DET);
    (void) tmpreg;
}

/**
  * @brief  Clear ACTIVITY flag.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CLR_ACTIVITY   | CLR_ACTIVITY
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_activity(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_ACTIVITY);
    (void) tmpreg;
}

/**
  * @brief  Clear RX_DONE flag.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CLR_RX_DONE    | CLR_RX_DONE
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_rx_done(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_RX_DONE);
    (void) tmpreg;
}

/**
  * @brief  Clear TX_ABRT flag.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CLR_TX_ABORT   | CLR_TX_ABRT
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_tx_abort(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_TX_ABORT);
    (void) tmpreg;
}

/**
  * @brief  Clear RD_REQ flag.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CLR_RD_REQ     | CLR_RD_REQ
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_read_req(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_RD_REQ);
    (void) tmpreg;
}

/**
  * @brief  Clear TX_OVER flag.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CLR_TX_OVER    | CLR_TX_OVER
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_tx_over(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_TX_OVER);
    (void) tmpreg;
}

/**
  * @brief  Clear RX_OVER flag.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CLR_RX_OVER    | CLR_RX_OVER
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_rx_over(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_RX_OVER);
    (void) tmpreg;
}

/**
  * @brief  Clear RX_UNDER flag.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  CLR_RX_UNDER   | CLR_RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_rx_under(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_RX_UNDER);
    (void) tmpreg;
}

/**
  * @brief  Indicate the status of IC_STATUS SDA stuck at low is not recovered flag.
  * @note   RESET: stuck at low is recovered.
  *         SET  : stuck at low is not recovered.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | SDA_STUCK_RCVR
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_sda_stuck_not_recovered(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STAT, I2C_STAT_SDA_STUCK_RCVR) == (I2C_STAT_SDA_STUCK_RCVR));
}

/**
  * @brief  Indicate the status of IC_STATUS Slave FSM Activity Status flag.
  * @note   RESET: Slave FSM is in IDLE state.
  *         SET  : When Slave FSM is not in IDLE state.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | S_ACTIVITY
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_slave_activity(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STAT, I2C_STAT_S_ACTIVITY) == (I2C_STAT_S_ACTIVITY));
}

/**
  * @brief  Indicate the status of IC_STATUS Master FSM Activity Status flag.
  * @note   RESET: Master FSM is in IDLE state.
  *         SET  : When Master FSM is not in IDLE state.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | M_ACTIVITY
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_master_activity(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STAT, I2C_STAT_M_ACTIVITY) == (I2C_STAT_M_ACTIVITY));
}

/**
  * @brief  Indicate the status of IC_STATUS Receive FIFO Completely Full flag.
  * @note   RESET: Receive FIFO is not full.
  *         SET  : When Receive FIFO is full.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | RX_FIFO_CF
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_rff(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STAT, I2C_STAT_RX_FIFO_CF) == (I2C_STAT_RX_FIFO_CF));
}

/**
  * @brief  Indicate the status of IC_STATUS Receive FIFO Not Empty flag.
  * @note   RESET: Receive FIFO is empty.
  *         SET  : When Receive FIFO is not empty.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | RX_FIFO_NE
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_rfne(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STAT, I2C_STAT_RX_FIFO_NE) == (I2C_STAT_RX_FIFO_NE));
}

/**
  * @brief  Indicate the status of IC_STATUS Transmit FIFO Completely Empty flag.
  * @note   RESET: Transmit FIFO is not empty.
  *         SET  : When Transmit FIFO is empty.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | TX_FIFO_CE
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_tfe(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STAT, I2C_STAT_TX_FIFO_CE) == (I2C_STAT_TX_FIFO_CE));
}

/**
  * @brief  Indicate the status of IC_STATUS Transmit FIFO Not Full flag.
  * @note   RESET: Transmit FIFO is full.
  *         SET  : When Transmit FIFO is not full.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | TX_FIFO_NF
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_tfnf(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STAT, I2C_STAT_TX_FIFO_NF) == (I2C_STAT_TX_FIFO_NF));
}

/**
  * @brief  Indicate the status of IC_STATUS ACTIVITY flag.
  * @note   RESET:  I2C is idle.
  *         SET  :  When I2C is active.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  STAT           | ACTIVITY
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_activity(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STAT, I2C_STAT_ACTIVITY) == (I2C_STAT_ACTIVITY));
}

/**
  * @brief  Indicate the status of Slave Received Data Lost flag.
  * @note   RESET:  Slave RX Data is not lost.
  *         SET  :  Slave RX Data is lost.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  EN_STAT        | S_RX_DATA_LOST
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_slave_rx_data_lost(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->EN_STAT, I2C_EN_STAT_S_RX_DATA_LOST) == (I2C_EN_STAT_S_RX_DATA_LOST));
}

/**
  * @brief  Indicate the status of Slave Disabled While Busy flag.
  * @note   RESET:  Slave is disabled when it is idle.
  *         SET  :  Slave is disabled when it is active.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  EN_STAT        | S_DIS_BUSY
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_slave_dis_whl_busy(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->EN_STAT, I2C_EN_STAT_S_DIS_BUSY) == (I2C_EN_STAT_S_DIS_BUSY));
}
/** @} */

/** @defgroup I2C_LL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Enable DMA transmission requests.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DMA_CTRL       | TX_EN
  *
  * @retval Value range between 0 ~ 0x8.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_dma_req_tx(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->DMA_CTRL, I2C_DMA_CTRL_TX_EN);
}

/**
  * @brief  Disable DMA transmission requests.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DMA_CTRL       | TX_EN
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_dma_req_tx(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->DMA_CTRL, I2C_DMA_CTRL_TX_EN);
}

/**
  * @brief  Check if DMA transmission requests are enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DMA_CTRL       | TX_EN
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_dma_req_tx(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->DMA_CTRL, I2C_DMA_CTRL_TX_EN) == (I2C_DMA_CTRL_TX_EN));
}

/**
  * @brief  Enable DMA reception requests.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DMA_CTRL       | RX_EN
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_dma_req_rx(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->DMA_CTRL, I2C_DMA_CTRL_RX_EN);
}

/**
  * @brief  Disable DMA reception requests.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DMA_CTRL       | RX_EN
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_dma_req_rx(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->DMA_CTRL, I2C_DMA_CTRL_RX_EN);
}

/**
  * @brief  Check if DMA reception requests are enabled or disabled.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DMA_CTRL       | RX_EN
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_dma_req_rx(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->DMA_CTRL, I2C_DMA_CTRL_RX_EN) == (I2C_DMA_CTRL_RX_EN));
}

/**
  * @brief  Set level of TX FIFO that requests a DMA transmit.
  * @note   TX data level should equal to the watermark level, that is, the dma_tx_req
  *         signal is generated when the number of valid data entries in the transmit
  *         FIFO is equal to or below this field value, and TDMAE = 1.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DMA_TX_LEVEL   | LEVEL
  *
  * @param  I2Cx I2C instance
  * @param  level This parameter should range between 0x0 and 0x8.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_dma_tx_data_level(i2c_regs_t *I2Cx, uint32_t level)
{
    WRITE_REG(I2Cx->DMA_TX_LEVEL, level);
}

/**
  * @brief  Get level of TX FIFO that request a DMA transmit.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DMA_TX_LEVEL   | LEVEL
  *
  * @param  I2Cx I2C instance
  * @retval Returned value should range between 0x0 and 0x8.
  */
__STATIC_INLINE uint32_t ll_i2c_get_dma_tx_data_level(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->DMA_TX_LEVEL, I2C_DMA_TX_LEVEL_LEVEL));
}

/**
  * @brief  Set level of RX FIFO that requests a DMA receive.
  * @note   The watermark level = DMARDL + 1, that is, dma_rx_req is generated when
  *         the number of valid data entries in the receive FIFO is equal to or
  *         more than this field value + 1, and RDMAE = 1. For instance, when DMARDL
  *         is 0, then dma_rx_req is asserted when 1 or more data entries are present
  *         in the receive FIFO.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DMA_RX_LEVEL   | LEVEL
  *
  * @param  I2Cx I2C instance
  * @param  level This parameter should range between 0x0 and 0x8.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_dma_rx_data_level(i2c_regs_t *I2Cx, uint32_t level)
{
    WRITE_REG(I2Cx->DMA_RX_LEVEL, level);
}

/**
  * @brief  Get level of RX FIFO that request a DMA receive.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DMA_RX_LEVEL   | LEVEL
  *
  * @param  I2Cx I2C instance
  * @retval Returned value should range between 0x0 and 0x8.
  */
__STATIC_INLINE uint32_t ll_i2c_get_dma_rx_data_level(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->DMA_RX_LEVEL, I2C_DMA_RX_LEVEL_LEVEL));
}

/**
  * @brief  Get the data register address used for DMA transfer
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_CMD       | DAT
  *
  * @param  I2Cx I2C instance
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t ll_i2c_dma_get_register_address(i2c_regs_t *I2Cx)
{
    return ((uint32_t) & (I2Cx->DATA_CMD));
}



__STATIC_INLINE void ll_i2c_set_scl_stuck_at_low_timeout(i2c_regs_t *I2Cx, uint32_t timeout)
{
    WRITE_REG(I2Cx->SCL_STUCK_TIMEOUT, timeout);
}

__STATIC_INLINE void ll_i2c_set_sda_stuck_at_low_timeout(i2c_regs_t *I2Cx, uint32_t timeout)
{
    WRITE_REG(I2Cx->SDA_STUCK_TIMEOUT, timeout);
}


/** @} */

/** @defgroup I2C_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Configure the slave address for transfer (master mode).
  * @note   The register IC_TAR can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  TARGET_ADDR    | TAR_ADDR
  *
  * @param  I2Cx I2C instance.
  * @param  slave_addr This parameter must be a value between 0x00 and 0x3F.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_slave_address(i2c_regs_t *I2Cx, uint32_t slave_addr)
{
    MODIFY_REG(I2Cx->TARGET_ADDR, I2C_TARGET_ADDR_TARGET, slave_addr << I2C_TARGET_ADDR_TARGET_POS);
}

/**
  * @brief  Get the slave address programmed for transfer (master mode).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  TARGET_ADDR    | TAR_ADDR
  *
  * @param  I2Cx I2C instance.
  * @retval Value between 0x0 and0x3F
  */
__STATIC_INLINE uint32_t ll_i2c_get_slave_address(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->TARGET_ADDR, I2C_TARGET_ADDR_TARGET) >> I2C_TARGET_ADDR_TARGET_POS);
}

/**
  * @brief  Handles I2Cx communication when starting transfer or during transfer (TC or TCR flag are set).
  * @note   The register IC_CON and IC_TAR can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  TARGET_ADDR    | TAR_ADDR
  *  CTRL           | CON_10BITADDR_MST
  *
  * @param  I2Cx I2C instance.
  * @param  slave_addr      Specifies the slave address to be programmed.
  * @param  slave_addr_size This parameter can be one of the following values:
  *         @arg @ref LL_I2C_ADDRESSING_MODE_7BIT
  *         @arg @ref LL_I2C_ADDRESSING_MODE_10BIT
  * @note   SlaveAddrSize in IC_CON register can only be programmed when the I2C is disabled (IC_ENABLE = 0).
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_handle_transfer(i2c_regs_t *I2Cx, uint32_t slave_addr, uint32_t slave_addr_size)
{
    MODIFY_REG(I2Cx->TARGET_ADDR, I2C_TARGET_ADDR_TARGET, slave_addr << I2C_TARGET_ADDR_TARGET_POS);
    ll_i2c_set_master_addressing_mode(I2Cx, slave_addr_size);
}

/**
  * @brief  Read Receive Data register.
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_CMD       | DATA
  *
  * @param  I2Cx I2C instance.
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t ll_i2c_receive_data8(i2c_regs_t *I2Cx)
{
    return (uint8_t)(READ_BITS(I2Cx->DATA_CMD, I2C_DATA_CMD_DATA));
}

/**
  * @brief  Write in Transmit Data Register .
  *
  *  Register|BitsName
  *  ---------------|---------------
  *  DATA_CMD       | STOP
  *  DATA_CMD       | CMD
  *  DATA_CMD       | DATA
  *
  * @param  I2Cx I2C instance.
  * @param  data    Value range between 0x00 and 0xFF.
  * @param  cmd     This parameter can be one of the following values:
  *         @arg @ref LL_I2C_CMD_SLV_NONE
  *         @arg @ref LL_I2C_CMD_MST_WRITE
  *         @arg @ref LL_I2C_CMD_MST_READ
  *         @arg @ref LL_I2C_CMD_MST_GEN_STOP
  *         @arg @ref LL_I2C_CMD_MST_GEN_RESTART
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_transmit_data8(i2c_regs_t *I2Cx, uint8_t data, uint32_t cmd)
{
    WRITE_REG(I2Cx->DATA_CMD, data | cmd);
}

/** @} */

/** @defgroup I2C_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize I2C registers (Registers restored to their default values).
  * @param  I2Cx I2C instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: I2C registers are de-initialized
  *          - ERROR: I2C registers are not de-initialized
  */
error_status_t ll_i2c_deinit(i2c_regs_t *I2Cx);

/**
  * @brief  Initialize I2C registers according to the specified
  *         parameters in p_i2c_init.
  * @param  I2Cx I2C instance
  * @param  p_i2c_init  Pointer to a ll_i2c_init_t structure that contains the configuration
  *                         information for the specified I2C peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: I2C registers are initialized according to p_i2c_init content
  *          - ERROR: Problem occurred during I2C Registers initialization
  */
error_status_t ll_i2c_init(i2c_regs_t *I2Cx, ll_i2c_init_t *p_i2c_init);

/**
  * @brief Set each field of a @ref ll_i2c_init_t type structure to default value.
  * @param p_i2c_init   Pointer to a @ref ll_i2c_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_i2c_struct_init(ll_i2c_init_t *p_i2c_init);

/** @} */

/** @} */

#endif /* I2C0 || I2C1 */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_I2C_H__ */

/** @} */

/** @} */

/** @} */

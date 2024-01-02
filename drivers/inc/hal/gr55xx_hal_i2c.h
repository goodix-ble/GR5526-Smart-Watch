/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_i2c.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of I2C HAL library.
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

/** @defgroup HAL_I2C I2C
  * @brief I2C HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_I2C_H__
#define __GR55xx_HAL_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_i2c.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_I2C_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_I2C_state HAL I2C state
  * @{
  */

/**
  * @brief  HAL I2C State Enumerations definition
  * @note  HAL I2C State value coding follow below described bitmap :\n
  * @verbatim
    b7-b6  Error information
        00 : No Error
        01 : Abort (Abort user request on going)
        10 : Timeout
        11 : Error
    b5     IP initialisation status
        0  : Reset (IP not initialized)
        1  : init done (IP initialized and ready to use. HAL I2C init function called)
    b4     (not used)
        x  : Should be set to 0
    b3
        0  : Ready or Busy (No Listen mode ongoing)
        1  : Listen (IP in Address Listen Mode)
    b2     Intrinsic process state
        0  : Ready
        1  : Busy (IP busy with some configuration or internal operations)
    b1     Rx state
        0  : Ready (no Rx operation ongoing)
        1  : Busy (Rx operation ongoing)
    b0     Tx state
        0  : Ready (no Tx operation ongoing)
        1  : Busy (Tx operation ongoing)
  * @endverbatim
  */
typedef enum
{
    HAL_I2C_STATE_RESET             = 0x00U,   /**< Peripheral is not yet Initialized         */
    HAL_I2C_STATE_READY             = 0x20U,   /**< Peripheral Initialized and ready for use  */
    HAL_I2C_STATE_BUSY              = 0x24U,   /**< An internal process is ongoing            */
    HAL_I2C_STATE_BUSY_TX           = 0x21U,   /**< Data Transmission process is ongoing      */
    HAL_I2C_STATE_BUSY_RX           = 0x22U,   /**< Data Reception process is ongoing         */
    HAL_I2C_STATE_ABORT             = 0x60U,   /**< Abort user request ongoing                */
    HAL_I2C_STATE_TIMEOUT           = 0xA0U,   /**< Timeout state                             */
    HAL_I2C_STATE_ERROR             = 0xE0U    /**< Error                                     */

} hal_i2c_state_t;
/** @} */

/** @defgroup HAL_I2C_mode HAL I2C mode
  * @{
  */

/**
  * @brief  HAL I2C Mode Enumerations definition
  * @note  HAL I2C Mode value coding follow below described bitmap :\n
  * @verbatim
    b7     (not used)
       x  : Should be set to 0
    b6
       0  : None
       1  : Memory (HAL I2C communication is in Memory Mode)
    b5
       0  : None
       1  : Slave (HAL I2C communication is in Slave Mode)
    b4
       0  : None
       1  : Master (HAL I2C communication is in Master Mode)
    b3-b2-b1-b0  (not used)
       xxxx : Should be set to 0000
  * @endverbatim
  */
typedef enum
{
    HAL_I2C_MODE_NONE               = 0x00U,   /**< No I2C communication on going             */
    HAL_I2C_MODE_MASTER             = 0x10U,   /**< I2C communication is in Master Mode       */
    HAL_I2C_MODE_SLAVE              = 0x20U,   /**< I2C communication is in Slave Mode        */
} hal_i2c_mode_t;
/** @} */

/** @} */


/** @addtogroup HAL_I2C_STRUCTURES Structures
  * @{
  */

/** @defgroup I2C_Configuration I2C Configuration
  * @{
  */

/**
  * @brief  I2C Configuration Structure definition
  */
typedef struct _i2c_init
{
    uint32_t speed;               /**< Specifies the I2C transfer speed.
                                       This parameter can be a value of @ref I2C_Speed */

    uint32_t own_address;         /**< Specifies the device own address.
                                       This parameter can be a 7-bit or 10-bit address. */

    uint32_t addressing_mode;     /**< Specifies if 7-bit or 10-bit addressing mode is selected.
                                       This parameter can be a value of @ref I2C_Addressing_Mode */

    uint32_t general_call_mode;   /**< Specifies if general call mode is selected.
                                     This parameter can be a value of @ref I2C_General_Call_Addressing_Mode */

    uint32_t tx_hold_time;         /**< transmit SDA hold time. the unit is ns*/

    uint32_t rx_hold_time;         /**< receive SDA hold time. the unit is ns*/
} i2c_init_t;
/** @} */

/** @defgroup I2C_handle I2C handle
  * @{
  */

/**
  * @brief  I2C handle Structure definition
  */
typedef struct _i2c_handle
{
    i2c_regs_t             *p_instance;         /**< I2C registers base address                */

    i2c_init_t             init;              /**< I2C communication parameters              */

    uint8_t                *p_buffer;           /**< Pointer to I2C transfer buffer            */

    uint16_t               xfer_size;         /**< I2C transfer size                         */

    __IO uint16_t          xfer_count;        /**< I2C transfer counter                      */

    __IO uint16_t          master_ack_count;  /**< I2C master acknowledge counter in master receive progress */

    __IO uint32_t          xfer_options;      /**< I2C sequential transfer options, this parameter can
                                                   be a value of @ref I2C_XferOptions         */

    __IO uint32_t          previous_state;    /**< I2C communication Previous state           */

    hal_status_t(*xfer_isr)(struct _i2c_handle *p_i2c, uint32_t it_source, uint32_t abort_sources);
                                              /**< I2C transfer IRQ handler function pointer  */

    dma_handle_t          *p_dmatx;            /**< I2C Tx DMA handle parameters               */

    dma_handle_t          *p_dmarx;            /**< I2C Rx DMA handle parameters               */

    hal_lock_t            lock;               /**< I2C locking object                         */

    __IO hal_i2c_state_t  state;              /**< I2C communication state                    */

    __IO hal_i2c_mode_t   mode;               /**< I2C communication mode                     */

    __IO uint32_t         error_code;         /**< I2C Error code                             */

    uint32_t              retention[13];     /**< I2C important register information. */

    __IO flag_status_t    i2c_dma_req_tx_flag; /**< I2C DMA TX request flag. */
} i2c_handle_t;
/** @} */

/** @} */

/** @addtogroup HAL_I2C_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_I2C_Callback Callback
  * @{
  */

/**
  * @brief HAL_I2C Callback function definition
  */

typedef struct _i2c_callback
{
    void (*i2c_msp_init)(i2c_handle_t *p_i2c);                  /**< I2C init MSP callback                      */
    void (*i2c_msp_deinit)(i2c_handle_t *p_i2c);                /**< I2C de-init MSP callback                   */
    void (*i2c_master_tx_cplt_callback)(i2c_handle_t *p_i2c);   /**< I2C master tx transfer completed callback  */
    void (*i2c_master_rx_cplt_callback)(i2c_handle_t *p_i2c);   /**< I2C master rx transfer completed callback  */
    void (*i2c_slave_tx_cplt_callback)(i2c_handle_t *p_i2c);    /**< I2C slave tx transfer completed callback   */
    void (*i2c_slave_rx_cplt_callback)(i2c_handle_t *p_i2c);    /**< I2C slave rx transfer completed callback   */
    void (*i2c_error_callback)(i2c_handle_t *p_i2c);            /**< I2C error callback                         */
    void (*i2c_abort_cplt_callback)(i2c_handle_t *p_i2c);       /**< I2C abort completed callback               */
} i2c_callback_t;

/** @} */

/** @} */

/** @defgroup  HAL_I2C_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup I2C_Exported_Constants I2C Exported Constants
  * @{
  */

/** @defgroup I2C_Error_Code_definition I2C Error Code definition
  * @{
  */
#define HAL_I2C_ERROR_NONE                   (0x00000000U)    /**< No error                        */
#define HAL_I2C_ERROR_INVALID_PARAM          (0x00000001U)    /**< Invalid parameter error         */
#define HAL_I2C_ERROR_ARB_LOST               (0x00000002U)    /**< Arbitration lost error          */
#define HAL_I2C_ERROR_NOACK                  (0x00000004U)    /**< No acknowledge error            */
#define HAL_I2C_ERROR_OVER                   (0x00000008U)    /**< RX_OVER error                   */
#define HAL_I2C_ERROR_SCL_STUCK_AT_LOW       (0x00000010U)    /**< SCL_STUCK_AT_LOW error          */
#define HAL_I2C_ERROR_SDA_STUCK_AT_LOW       (0x00000020U)    /**< SCL_STUCK_AT_LOW error          */
#define HAL_I2C_ERROR_DMA                    (0x00000040U)    /**< DMA transfer error              */
#define HAL_I2C_ERROR_TIMEOUT                (0x00000080U)    /**< Timeout error                   */
#define HAL_I2C_ERROR_NOTE                   (0x00000100U)    /**< No TE error                     */

/** @} */

/** @defgroup I2C_Speed I2C Transfer Speed
  * @{
  */
#define I2C_SPEED_100K                  LL_I2C_SPEED_100K   /**< Standard speed. */
#define I2C_SPEED_400K                  LL_I2C_SPEED_400K   /**< Fast speed. */
#define I2C_SPEED_1000K                 LL_I2C_SPEED_1000K  /**< Fast Plus speed. */
#define I2C_SPEED_2000K                 LL_I2C_SPEED_2000K  /**< High speed. */
/** @} */

/** @defgroup I2C_Addressing_Mode I2C Addressing Mode
  * @{
  */
#define I2C_ADDRESSINGMODE_7BIT         (0x00000001U)       /**< 7-bit addressing mode. */
#define I2C_ADDRESSINGMODE_10BIT        (0x00000002U)       /**< 10-bit addressing mode. */
/** @} */

/** @defgroup I2C_General_Call_Addressing_Mode I2C General Call Addressing Mode
  * @{
  */
#define I2C_GENERALCALL_DISABLE         (0x00000000U)       /**< General call mode disable. */
#define I2C_GENERALCALL_ENABLE          (0x00000001U)       /**< General call mode enable. */
/** @} */

/** @defgroup I2C_Memory_Address_Size I2C Memory Address Size
  * @{
  */
#define I2C_MEMADD_SIZE_8BIT            (0x00000001U)       /**< 8-bit memory address. */
#define I2C_MEMADD_SIZE_16BIT           (0x00000002U)       /**< 16-bit memory address. */
/** @} */

/** @defgroup I2C_XferOptions  I2C Sequential Transfer Options
  * @{
  */
#define I2C_FIRST_FRAME                 (0x00000000U)     /**< First transfer frame. */
#define I2C_FIRST_AND_NEXT_FRAME        (0x00000001U)     /**< First and next transfer frames. */
#define I2C_NEXT_FRAME                  (0x00000002U)     /**< Next transfer frame. */
#define I2C_FIRST_AND_LAST_FRAME        (0x00000003U)     /**< First and last transfer frames. */
#define I2C_LAST_FRAME                  (0x00000004U)     /**< Last transfer frame. */
/** @} */

/** @defgroup I2C_Timing_type  I2C Timing type
  * @{
  */
#define  I2C_TIMING_SS_SCL_LOW          (0x00000000U)     /**< Standard speed(0, 100K] SCL low time. */
#define  I2C_TIMING_SS_SCL_HIGH         (0x00000001U)     /**< Standard speed(0, 100K] SCL high time. */
#define  I2C_TIMING_FS_SCL_LOW          (0x00000002U)     /**< Fast and fast puls speed(100K, 1000K] SCL low time. */
#define  I2C_TIMING_FS_SCL_HIGH         (0x00000003U)     /**< Fast and fast puls speed(100K, 1000K] SCL high time. */
#define  I2C_TIMING_HS_SCL_LOW          (0x00000004U)     /**< High speed(1000K, 3400K] SCL low time. */
#define  I2C_TIMING_HS_SCL_HIGH         (0x00000005U)     /**< High speed(1000K, 3400K] SCL high time. */
#define  I2C_TIMING_FS_SPK              (0x00000006U)     /**< Fast and fast puls speed(100K, 1000K] spike suppression time. */
#define  I2C_TIMING_HS_SPK              (0x00000007U)     /**< High speed(1000K, 3400K] spike suppression time. */
#define  I2C_TIMING_SDA_TX_HOLD         (0x00000008U)     /**< SDA hold time when TX. (Hold time: Time of master and slave exchange SDA control) */
#define  I2C_TIMING_SDA_RX_HOLD         (0x00000009U)     /**< SDA hold time when RX. (Hold time: Time of master and slave exchange SDA control) */
/** @} */

/**
  * @brief  I2C InitStruct default configuration
  */
#define I2C_DEFAULT_CONFIG                              \
{                                                       \
    .speed              = I2C_SPEED_400K,               \
    .own_address        = 0x55U,                        \
    .addressing_mode    = I2C_ADDRESSINGMODE_7BIT,      \
    .general_call_mode  = I2C_GENERALCALL_DISABLE,      \
    .tx_hold_time       = 1,                            \
    .rx_hold_time       = 0,                            \
}

/** @} */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup I2C_Exported_Macros I2C Exported Macros
  * @{
  */

/** @brief Reset I2C handle state.
  * @param  __HANDLE__ Specifies the I2C Handle.
  * @retval None
  */
#define __HAL_I2C_RESET_HANDLE_STATE(__HANDLE__)                ((__HANDLE__)->state = HAL_I2C_STATE_RESET)
/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup I2C_Private_Macro I2C Private Macros
  * @{
  */

/**
  * @brief Check if the I2C speed is valid.
  * @param __SPEED__ I2C transfer speed.
  * @retval SET (__SPEED__ is valid) or RESET (__SPEED__ is invalid)
  */
#define IS_I2C_SPEED(__SPEED__)             (((__SPEED__) == I2C_SPEED_100K)  || \
                                             ((__SPEED__) == I2C_SPEED_400K)  || \
                                             ((__SPEED__) == I2C_SPEED_1000K) || \
                                             ((__SPEED__) == I2C_SPEED_2000K))

/**
  * @brief Check if the I2C addressing mode is valid.
  * @param __MODE__ I2C addressing mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_I2C_ADDRESSING_MODE(__MODE__)    (((__MODE__) == I2C_ADDRESSINGMODE_7BIT) || \
                                             ((__MODE__) == I2C_ADDRESSINGMODE_10BIT))

/**
  * @brief Check if the I2C general call mode is valid.
  * @param __CALL__ I2C general call mode.
  * @retval SET (__CALL__ is valid) or RESET (__CALL__ is invalid)
  */
#define IS_I2C_GENERAL_CALL(__CALL__)       (((__CALL__) == I2C_GENERALCALL_DISABLE) || \
                                             ((__CALL__) == I2C_GENERALCALL_ENABLE))

/**
  * @brief Check if the I2C memory address size is valid.
  * @param __SIZE__ I2C memory address size.
  * @retval SET (__SIZE__ is valid) or RESET (__SIZE__ is invalid)
  */
#define IS_I2C_MEMADD_SIZE(__SIZE__)        (((__SIZE__) == I2C_MEMADD_SIZE_8BIT) || \
                                             ((__SIZE__) == I2C_MEMADD_SIZE_16BIT))

/**
  * @brief Check if the I2C transfer request command is valid.
  * @param __REQUEST__ I2C transfer request command.
  * @retval SET (__REQUEST__ is valid) or RESET (__REQUEST__ is invalid)
  */
#define IS_TRANSFER_REQUEST(__REQUEST__)    (((__REQUEST__) == I2C_CMD_SLV_NONE)     || \
                                             ((__REQUEST__) == I2C_CMD_MST_WRITE)    || \
                                             ((__REQUEST__) == I2C_CMD_MST_READ)     || \
                                             ((__REQUEST__) == I2C_CMD_MST_GEN_STOP) || \
                                             ((__REQUEST__) == I2C_CMD_MST_GEN_RESTART))

/**
  * @brief Check if the I2C transfer options request is valid.
  * @param __REQUEST__ I2C transfer options request.
  * @retval SET (__REQUEST__ is valid) or RESET (__REQUEST__ is invalid)
  */
#define IS_I2C_TRANSFER_OPTIONS_REQUEST(__REQUEST__)  (((__REQUEST__) == I2C_FIRST_FRAME)          || \
                                                       ((__REQUEST__) == I2C_FIRST_AND_NEXT_FRAME) || \
                                                       ((__REQUEST__) == I2C_NEXT_FRAME)           || \
                                                       ((__REQUEST__) == I2C_FIRST_AND_LAST_FRAME) || \
                                                       ((__REQUEST__) == I2C_LAST_FRAME))

/**
  * @brief Check if the I2C slave address is valid.
  * @param __ADDRESS__ I2C slave address.
  * @retval SET (__ADDRESS__ is valid) or RESET (__ADDRESS__ is invalid)
  */
#define IS_I2C_SLV_ADDRESS(__ADDRESS__)     ((__ADDRESS__) < 0x03FFU)

/**
  * @brief Check if the I2C own address is valid.
  * @param __ADDRESS__ I2C own address.
  * @retval SET (__ADDRESS__ is valid) or RESET (__ADDRESS__ is invalid)
  */
#define IS_I2C_OWN_ADDRESS(__ADDRESS__)     ((((__ADDRESS__) > 0x0007U) && ((__ADDRESS__) < 0x0078U)) || \
                                             (((__ADDRESS__) > 0x007FU) && ((__ADDRESS__) < 0x03FFU)))

/**
  * @brief Get the Most Significant 8 Bits of memory address.
  * @param __ADDRESS__ Memory address.
  * @retval SET (__ADDRESS__ is valid) or RESET (__ADDRESS__ is invalid)
  */
#define I2C_MEM_ADD_MSB(__ADDRESS__)              ((uint8_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)(0xFF00U))) >> 8U)))

/**
  * @brief Get the Least Significant 8 Bits of memory address.
  * @param __ADDRESS__ Memory address.
  * @retval SET (__ADDRESS__ is valid) or RESET (__ADDRESS__ is invalid)
  */
#define I2C_MEM_ADD_LSB(__ADDRESS__)              ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)(0x00FFU))))

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_I2C_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and de-initialization functions
  * @verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the I2Cx peripheral:

      (+) User must Implement hal_i2c_msp_init() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_i2c_init() to configure the selected device with
          the selected configuration:
        (++) Speed
        (++) Own Address
        (++) Addressing mode (Master, Slave)
        (++) General call mode

      (+) Call the function hal_i2c_deinit() to restore the default configuration
          of the selected I2Cx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initializes the I2C according to the specified parameters
 *         in the i2c_init_t and initialize the associated handle.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration
 *                information for the specified I2C.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_init(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  De-initialize the I2C peripheral.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_deinit(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief Initialize the I2C MSP.
 * @note  This function should not be modified. When the callback is needed,
 *            the hal_i2c_msp_init could be implemented in the user file.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 ****************************************************************************************
 */
void hal_i2c_msp_init(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief De-initialize the I2C MSP.
 * @note  This function should not be modified. When the callback is needed,
 *            the hal_i2c_msp_deinit could be implemented in the user file.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 ****************************************************************************************
 */
void hal_i2c_msp_deinit(i2c_handle_t *p_i2c);

/** @} */

/** @addtogroup I2C_Exported_Functions_Group2 IO operation functions
  * @brief   Data transfers functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the I2C data
    transfers.

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in the polling mode.
            The status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts
            or DMA. These functions return the status of the transfer startup.
            The end of the data processing will be indicated through the
            dedicated I2C IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.

    (#) Blocking mode functions are :
        (++) hal_i2c_master_transmit()
        (++) hal_i2c_master_receive()
        (++) hal_i2c_slave_transmit()
        (++) hal_i2c_slave_receive()
        (++) hal_i2c_mem_write()
        (++) hal_i2c_mem_read()
        (++) hal_i2c_is_device_ready()

    (#) No-Blocking mode functions with Interrupt are :
        (++) hal_i2c_master_transmit_it()
        (++) hal_i2c_master_receive_it()
        (++) hal_i2c_slave_transmit_it()
        (++) hal_i2c_slave_receive_it()
        (++) hal_i2c_mem_write_it()
        (++) hal_i2c_mem_read_it()

    (#) No-Blocking mode functions with DMA are :
        (++) hal_i2c_master_transmit_dma()
        (++) hal_i2c_master_receive_dma()
        (++) hal_i2c_slave_transmit_dma()
        (++) hal_i2c_slave_receive_dma()
        (++) hal_i2c_mem_write_dma()
        (++) hal_i2c_mem_read_dma()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) hal_i2c_mem_tx_cplt_callback()
        (++) hal_i2c_mem_rx_cplt_callback()
        (++) hal_i2c_master_tx_cplt_callback()
        (++) hal_i2c_master_rx_cplt_callback()
        (++) hal_i2c_slave_tx_cplt_callback()
        (++) hal_i2c_slave_rx_cplt_callback()()
        (++) hal_i2c_error_callback()

@endverbatim
  * @{
  */

/******* Blocking mode: Polling */

/**
 ****************************************************************************************
 * @brief  Transmits in master mode an amount of data in blocking mode.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_master_transmit(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receives in master mode an amount of data in blocking mode.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 * @note   This function will return HAL_OK even if the length of data sent by slave is
 *         less than the expected Size.
 ****************************************************************************************
 */
hal_status_t hal_i2c_master_receive(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmits in slave mode an amount of data in blocking mode.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_slave_transmit(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive in slave mode an amount of data in blocking mode
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_slave_receive(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Write an amount of data in blocking mode to a specific memory address
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_mem_write(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Read an amount of data in blocking mode from a specific memory address
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_mem_read(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout);

/******* Non-Blocking mode: Interrupt */

/**
 ****************************************************************************************
 * @brief  Transmit in master mode an amount of data in non-blocking mode with Interrupt
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_master_transmit_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Receive in master mode an amount of data in non-blocking mode with Interrupt
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_master_receive_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmit in slave mode an amount of data in non-blocking mode with Interrupt
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_slave_transmit_it(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Receive in slave mode an amount of data in non-blocking mode with Interrupt
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_slave_receive_it(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Write an amount of data in non-blocking mode with Interrupt to a specific memory address
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_mem_write_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Read an amount of data in non-blocking mode with Interrupt from a specific memory address
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_mem_read_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Abort a master I2C IT or DMA process communication with Interrupt.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_master_abort_it(i2c_handle_t *p_i2c);

/******* Non-Blocking mode: DMA */

/**
 ****************************************************************************************
 * @brief  Transmit in master mode an amount of data in non-blocking mode with DMA
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent,  ranging between 1 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_master_transmit_dma(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Receive in master mode an amount of data in non-blocking mode with DMA
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent,  ranging between 1 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_master_receive_dma(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmit in slave mode an amount of data in non-blocking mode with DMA
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent,  ranging between 1 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_slave_transmit_dma(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Receive in slave mode an amount of data in non-blocking mode with DMA
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent,  ranging between 1 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_slave_receive_dma(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Write an amount of data in non-blocking mode with DMA to a specific memory address
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent,  ranging between 1 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_mem_write_dma(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Reads an amount of data in non-blocking mode with DMA from a specific memory address.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent,  ranging between 1 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_mem_read_dma(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmit in master mode an amount of data in non-blocking mode with DMA
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent,  ranging between 1 and 4095.
 * @param[in] sg_llp_config: The config of source and destination's SG and LLP functions.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_master_transmit_dma_sg_llp(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, dma_sg_llp_config_t *sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Transmit in slave mode an amount of data in non-blocking mode with DMA
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent,  ranging between 1 and 4095.
 * @param[in] sg_llp_config: The config of source and destination's SG and LLP functions.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_slave_transmit_dma_sg_llp(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size, dma_sg_llp_config_t *sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Receive in slave mode an amount of data in non-blocking mode with DMA
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent,  ranging between 1 and 4095.
 * @param[in] sg_llp_config: The config of source and destination's SG and LLP functions.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_slave_receive_dma_sg_llp(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size, dma_sg_llp_config_t *sg_llp_config);

/**
 ****************************************************************************************
 * @brief  Write an amount of data in non-blocking mode with DMA to a specific memory address
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent,  ranging between 1 and 4095.
 * @param[in] sg_llp_config: The config of source and destination's SG and LLP functions.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_mem_write_dma_sg_llp(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, dma_sg_llp_config_t *sg_llp_config);

/** @} */

/** @addtogroup I2C_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  This function handles I2C event interrupt request.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 ****************************************************************************************
 */
void hal_i2c_irq_handler(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  Master Tx Transfer completed callback.
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_i2c_master_tx_cplt_callback can be implemented in the user file
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 ****************************************************************************************
 */
void hal_i2c_master_tx_cplt_callback(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  Master Rx Transfer completed callback.
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_i2c_master_rx_cplt_callback can be implemented in the user file
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 ****************************************************************************************
 */
void hal_i2c_master_rx_cplt_callback(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  Slave Tx Transfer completed callback.
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_i2c_slave_tx_cplt_callback can be implemented in the user file
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 ****************************************************************************************
 */
void hal_i2c_slave_tx_cplt_callback(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  Slave Rx Transfer completed callback.
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_i2c_slave_rx_cplt_callback can be implemented in the user file
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 ****************************************************************************************
 */
void hal_i2c_slave_rx_cplt_callback(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  I2C error callback.
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_i2c_error_callback can be implemented in the user file
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 ****************************************************************************************
 */
void hal_i2c_error_callback(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  I2C abort callback.
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_i2c_abort_cplt_callback can be implemented in the user file
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 ****************************************************************************************
 */
void hal_i2c_abort_cplt_callback(i2c_handle_t *p_i2c);

/** @} */

/** @addtogroup I2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
 *  @brief   Peripheral State, Mode and Error functions
 *
@verbatim
 ===============================================================================
            ##### Peripheral State, Mode and Error functions #####
 ===============================================================================
    [..]
    This subsection permit to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the I2C handle state.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @retval ::HAL_I2C_STATE_RESET: Peripheral is not yet Initialized.
 * @retval ::HAL_I2C_STATE_READY: Peripheral Initialized and ready for use.
 * @retval ::HAL_I2C_STATE_BUSY: An internal process is ongoing.
 * @retval ::HAL_I2C_STATE_BUSY_TX: Data Transmission process is ongoing.
 * @retval ::HAL_I2C_STATE_BUSY_RX: Data Reception process is ongoing.
 * @retval ::HAL_I2C_STATE_ABORT: Abort user request ongoing.
 * @retval ::HAL_I2C_STATE_TIMEOUT: Timeout state.
 * @retval ::HAL_I2C_STATE_ERROR: Error.
 ****************************************************************************************
 */
hal_i2c_state_t hal_i2c_get_state(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  Returns the I2C Master, Slave, Memory or no mode.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @retval ::HAL_I2C_MODE_NONE: No I2C communication on going.
 * @retval ::HAL_I2C_MODE_MASTER: I2C communication is in Master Mode.
 * @retval ::HAL_I2C_MODE_SLAVE: I2C communication is in Slave Mode.
 ****************************************************************************************
 */
hal_i2c_mode_t hal_i2c_get_mode(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  Return the I2C error code.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @return I2C Error Code
 ****************************************************************************************
 */
uint32_t hal_i2c_get_error(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  Return i2c sda at low is not recovered flag.
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @retval 1: i2c sda at low is not recovered.
 * @retval 0: i2c sda at low is recovered.
 ****************************************************************************************
 */
uint32_t hal_i2c_sda_at_low_is_not_recovered(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to I2C configuration before sleep.
 * @param[in] p_i2c: Pointer to a I2C handle which contains the configuration
 *                 information for the specified I2C module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_suspend_reg(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to I2C configuration after sleep.
 *         This function must be used in conjunction with the hal_i2c_suspend_reg().
 * @param[in] p_i2c: Pointer to a I2C handle which contains the configuration
 *                 information for the specified I2C module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_i2c_resume_reg(i2c_handle_t *p_i2c);

/**
 ****************************************************************************************
 * @brief  Configure the I2C transmit rate for I2C.
 * @param[in] p_i2c: Pointer to a I2C handle which contains the configuration
 *                 information for the specified I2C module.
 * @param[in] speed: The transmit rate of I2C.
 * @param[in] scl_fall_time: The fall time of scl.
 * @param[in] scl_rise_time: The rise time of scl.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 ****************************************************************************************
 */
hal_status_t hal_i2c_speed_config(i2c_handle_t *p_i2c, uint32_t speed, uint32_t scl_fall_time, uint32_t scl_rise_time);

/**
 ****************************************************************************************
 * @brief  Adjust I2C timing value to adapt to real load.
 * @param[in] p_i2c: Pointer to a I2C handle which contains the configuration
 *                 information for the specified I2C module.
 * @param[in] timing_type: Timing type. See I2C_Timing_type.
 * @param[in] delta: timing change value(unit: I2C work clock cycles).
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 ****************************************************************************************
 */
hal_status_t hal_i2c_timing_adjust(i2c_handle_t *p_i2c, uint32_t timing_type, int32_t delta);

/**
 ****************************************************************************************
 * @brief  Get the I2C timing value.
 * @param[in] p_i2c: Pointer to a I2C handle which contains the configuration
 *                 information for the specified I2C module.
 * @param[in] timing_type: Timing type. See I2C_Timing_type.
 * @param[in] p_timing_value: Pointer of I2C timing value(unit: I2C work clock cycles).
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error.
 ****************************************************************************************
 */
hal_status_t hal_i2c_timing_get(i2c_handle_t *p_i2c, uint32_t timing_type, uint32_t *p_timing_value);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_I2C_H__ */

/** @} */

/** @} */

/** @} */

/**
 *****************************************************************************************
 *
 * @file dis.h
 *
 * @brief Device Information Service API.
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_DIS Device Information Service (DIS)
 * @{
 * @brief Definitions and prototypes for the DIS interface.
 *
 * @details The Device Information Service exposes manufacturer and/or vendor information
 *          about a device.This module implements the Device Information Service with all
 *          optional characteristics.
 *
 *          After \ref dis_init_t variable is initialized, the application should call
 *          \ref dis_service_init() to add a Device Information Service and the characteristics
 *          which are selected by \ref dis_init_t.char_mask to the BLE Stack database.
 *          However the value of those characteristics locates in user space. The application
 *          should make sure the spaces for those values are available and those values will not be
 *          changed during the connection.
 *
 */

#ifndef __DIS_H__
#define __DIS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>

/**
 * @defgroup DIS_MACRO Defines
 * @{
 */
#define DIS_CONNECTION_MAX                  10              /**< Maximum number of DIS connections.The value is configurable. */
#define DIS_SYS_ID_LEN                      8               /**< System ID length. */
#define DIS_PNP_ID_LEN                      7               /**< PnP ID length. */
#define DIS_VAL_MAX_LEN                     128             /**< Maximal length for Characteristic values - 128 bytes. */
#define DIS_IEEE_CERTIF_MIN_LEN             6               /**< IEEE Certification length (min 6 bytes). */

/**
 * @defgroup DIS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref dis_init_t.char_mask.
 */
#define DIS_CHAR_SYSTEM_ID_SUP              0x00000006      /**< Bit mask of the System ID. */
#define DIS_CHAR_MODEL_NUMBER_SUP           0x00000018      /**< Bit mask of the Model Number. */
#define DIS_CHAR_SERIAL_NUMBER_SUP          0x00000060      /**< Bit mask of the Serial Number. */
#define DIS_CHAR_FIRMWARE_REV_SUP           0x00000180      /**< Bit mask of the Firmware Revision. */
#define DIS_CHAR_HARDWARE_REV_SUP           0x00000600      /**< Bit mask of the Hardware Revision. */
#define DIS_CHAR_SOFTWARE_REV_SUP           0x00001800      /**< Bit mask of the Software Revision. */
#define DIS_CHAR_MANUFACTURER_NAME_SUP      0x00006000      /**< Bit mask of the Manufacturer Name. */
#define DIS_CHAR_11073_CERT_DATA_SUP        0x00018000      /**< Bit mask of the IEEE 11073-20601 Regulatory Certification Data List. */
#define DIS_CHAR_PNP_ID_SUP                 0x00060000      /**< Bit mask of the PnP ID. */
#define DIS_CHAR_FULL                       0x0007ffff      /**< Bit mask of the full characteristic. */
/** @} */

/**
 * @defgroup DIS_IEEE_11073_BODY IEEE 11073-20601 Authoritative Body Type
 * @{
 */
#define DIS_11073_BODY_EMPTY                0              /**< Empty body type. */
#define DIS_11073_BODY_IEEE                 1              /**< IEEE body type. */
#define DIS_11073_BODY_CONTINUA             2              /**< Continua body type. */
#define DIS_11073_BODY_EXP                  254            /**< Exp body type. */
/** @} */
/** @} */

/**
 * @defgroup DIS_STRUCT Structures
 * @{
 */
/**@brief UTF-8 string data type. */
typedef struct
{
    uint8_t  length;              /**< String length. */
    char     *p_str;               /**< String data. */
} dis_string_t;

/**@brief System ID parameters. The first field is the LSOs and the second
 *        field contains the MSOs. */
typedef struct
{
    uint8_t manufacturer_id[5];   /**< Manufacturer-defined ID. */
    uint8_t org_unique_id[3];     /**< Organizationally unique ID (OUI) which is issued by IEEE. */
} dis_sys_id_t;

/**@brief IEEE 11073-20601 Regulatory Certification Data List Structure. */
typedef struct
{
    char     *p_list;              /**< Pointer to the list which contains the encoded opaque
                                   *   structure based on IEEE 11073-20601 specification. */
    uint8_t  list_len;            /**< Length of the list. */
} dis_reg_cert_data_list_t;

/**@brief PnP ID parameters */
typedef struct
{
    uint8_t  vendor_id_source;    /**< Vendor ID Source. */
    uint16_t vendor_id;           /**< Vendor ID. */
    uint16_t product_id;          /**< Product ID. */
    uint16_t product_version;     /**< Product Version. */
} dis_pnp_id_t;

/**@brief Device Information Service init structure. This contains all options
 *        and data needed for initialization of the service. */
typedef struct
{
    uint32_t                  char_mask;            /**< Initial mask of Supported characteristics, and configured with \ref DIS_CHAR_MASK. */
    dis_string_t              manufact_name_str;    /**< Initial manufacturer Name String. */
    dis_string_t              model_num_str;        /**< Initial model Number String. */
    dis_string_t              serial_num_str;       /**< Initial serial Number String. */
    dis_string_t              hw_rev_str;           /**< Initial hardware Revision String. */
    dis_string_t              fw_rev_str;           /**< Initial firmware Revision String. */
    dis_string_t              sw_rev_str;           /**< Initial software Revision String. */
    dis_sys_id_t             *p_sys_id;             /**< Initial system ID. */
    dis_reg_cert_data_list_t  reg_cert_data_list;   /**< Initial IEEE 11073-20601 Regulatory Certification Data List. */
    dis_pnp_id_t             *p_pnp_id;             /**< Initial PnP ID. */
} dis_init_t;
/** @} */

/**
 * @defgroup DIS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Device Information Service instance and add in the database.
 *
 * @param[in] p_dis_init: Pointer to Device Information Service initialization variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t dis_service_init(dis_init_t *p_dis_init);

/**
 *****************************************************************************************
 * @brief Provide the interface for other modules to obtain the dis service start handle .
 *
 * @return The dis service start handle.
 *****************************************************************************************
 */
uint16_t dis_service_start_handle_get(void);

/** @} */

#endif
/** @} */
/** @} */

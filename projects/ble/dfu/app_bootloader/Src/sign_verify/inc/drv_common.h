/*
* Copyright (C) 2017, Shenzhen Goodix Technology Co., Ltd.
* All Rights Reserved.
*/

#ifndef __DRV_COMMON_H__
#define __DRV_COMMON_H__

#include <stdint.h>


#undef NULL  /* others (e.g. <stdio.h>) also define */
#define NULL 0


/**
 * \brief General return codes
 */
typedef enum {
    GM_DRV_OK =                  0,                     ///< Operation succeeded

    /*common error code start*/
    GM_DRV_ERROR =              -10000,                 ///< Unspecified error
    GM_DRV_ERROR_BUSY =         -10001,                 ///< Driver is busy
    GM_DRV_ERROR_TIMEOUT =      -10002,                 ///< Timeout occurred
    GM_DRV_ERROR_UNSUPPORTED =  -10003,                 ///< Operation not supported
    GM_DRV_ERROR_PARAMETER  =   -10004,                 ///< Parameter error
    GM_DRV_ERROR_POWER_OFF  =   -10005,                 ///< Start of driver specific errors
    GM_DRV_ERROR_CONFIG = -10006,                       ///< Driver config error
    GM_DRV_ERROR_RETRY      =   -10007,                 ///< Retry operation
    GM_DRV_ERROR_NOT_READY  =   -10008,                 ///< Not ready
    /*common error code end*/

    /*SPIM error code start*/
    GM_DRV_SPIM_ERROR_MODE = -10101,                    ///< SPIM mode error, Master Mode supported only
    /*SPIM error code end*/

    /*MTP error code start*/
    GM_DRV_MTP_ERROR_ADDRESS_NOT_4BYTE_ALIGN = -10204,  ///< MTP address error, Address is not 4 bytes aligned
    GM_DRV_MTP_ERROR_ADDRESS_NOT_AVAILABLE = -10205,    ///< MTP address error, Address can not accessed
    /*MTP error code end*/

    /*AES error code start*/
    GM_DRV_AES_TRANSFER_ERROR  = -10300,                ///< AES Transfer error flag.
    GM_DRV_AES_ALIGN_ERROR     = -10301,                ///< Input/output data not aligned to 16bytes
    GM_DRV_AES_INPUT_ERROR     = -10302,                ///< Input parameters is error.
    /*AES error code end*/

    /*SHA error code start*/
    GM_DRV_SHA_ERROR_STATUS    = -10400,                ///< SHA execution error
    GM_DRV_SHA_ERROR_INPUT     = -10401,                ///< SHA parameters error
    /*SHA error code end*/

    /*UART error code start*/
    GM_DRV_UART_ERROR_PARITY  = -10500,                 ///< UART parity error
    GM_DRV_UART_ERROR_FRAMING = -10501,                 ///< UART frame error
    GM_DRV_UART_ERROR_PARITY_FRAMING = -10502,          ///< UART parity frame error in the same time
    /*UART error code end*/

    /*I2C error code start*/
    GM_DRV_I2C_ERROR_NACK     = -10600,                 ///< No ACK received error
    /*I2C error code end*/

    /*AES error code start*/
    GM_DRV_PRESENT_TRANSFER_ERROR  = -10300,                ///< AES Transfer error flag.
    GM_DRV_PRESENT_ALIGN_ERROR     = -10301,                ///< Input/output data not aligned to 16bytes
    GM_DRV_PRESENT_INPUT_ERROR     = -10302,                ///< Input parameters is error.
    /*AES error code end*/

    GM_DRV_ECC_PARAMETER_ERROR    = -10800,                ///< ECC ECDSA verify algorithm error
    GM_DRV_ECC_INTERRUPT_ERROR    = -10801,                ///< ECC interrupt error flag is raised
    GM_DRV_ECC_ECDSA_SIGN_ERROR   = -10802,                ///< ECC ECDSA sign algorithm error
    GM_DRV_ECC_ECDSA_VERIFY_ERROR = -10803,                ///< ECC ECDSA verify algorithm error
    GM_DRV_ECC_MONTGOMERY_INVERSE_K_ERROR = -10804,        ///< ECC Montgomery inverse output k is not in range
    GM_DRV_ECC_NOT_IRREVERSIBLE   = -10805,                ///< ECC Montgomery/Modular irreversible
    GM_DRV_ECC_CURVE_UNSUPPORTED  = -10806,                ///< ECC Curve a!=-3 mod p(NIST - P256)
    GM_DRV_ECC_RNG_ERROR  = -10807,                        ///< ECC RNG module not found or returns error
    GM_DRV_ECC_DEVICE_BUSY  = -10808,                      ///< ECC Device is busy, check if it is released
    GM_DRV_ECC_CURVE_CRC_ERROR  = -10809,                  ///< ECC Curve CRC Failed!
    GM_DRV_ECC_POINT_NOT_ON_CURVE =  -10810,               ///< ECC Input point is not on the curve

    /*EFUSE error code start*/
    GM_DRV_EFUSE_ERROR = -10900,
   /*EFUSE error code end*/


    /*Bootloader error code start*/
    GM_BL_OK = 0,
    GM_BL_ERROR_CRC32     =         -20001,
    GM_BL_ERROR_INFO       =            -20002,
    GM_BL_ERROR_KEY       =             -20003,
    GM_BL_ERROR_FW          =           -20004,
    GM_BL_ERROR_INVALID_FW_PUBLIC_KEY    =    -20005,
    GM_BL_ERROR_ROLLBACK      =         -20006,
    GM_BL_ERROR_SIGNATURE     =         -20007,
    GM_BL_ERROR_PRODUCT_ID  =           -20008,
    GM_BL_ERROR_INITIALIZED     =       -20009,
    GM_BL_ERROR_INVALID_UPGRADE_MODE  =   -20010,
    GM_BL_ERROR_TIMEOUT        =        -20011,
    GM_BL_ERROR_FW_HASH      =          -20012,
    GM_BL_ERROR_TRIM            =       -20013,
    GM_BL_ERROR_FLASH_INIT    =         -20014,
    GM_BL_ERROR_FLASH_READ    =         -20015,
    GM_BL_ERROR_FLASH_WRITE     =       -20016,
    GM_BL_ERROR_FLASH_ERASE       =     -20017,
    GM_BL_ERROR_ECIES           =       -20018,

    GM_BL_ERROR_ECIES_BAD_INPUT_DATA  = -20019,
    GM_BL_ERROR_ECIES_INVALID_PADDING = -20020,
    GM_BL_ERROR_ECIES_VERIFY_FAILED   = -20021,

    GM_BL_ERROR_RSA_BAD_INPUT_DATA  = -20022,
    GM_BL_ERROR_RSA_INVALID_PADDING = -20023,
    GM_BL_ERROR_RSA_VERIFY_FAILED   = -20024,

    GM_BL_ERROR_VBS          =          -20030,
    GM_BL_ERROR_VBS_WRITE_FLASH    =    -20031,

    GM_BL_ERROR_UPGRADE_INIT        =   -20040,
    GM_BL_ERROR_UPGRADE_OVERFLOW    =   -20041,
    GM_BL_ERROR_UPGRADE_CRC32       =   -20042,
    GM_BL_ERROR_UPGRADE_INVALID_CMD =   -20043,
    GM_BL_ERROR_UPGRADE_NOT_CONNECT =   -20044,
    GM_BL_UPGRADE_PROGRAM_FAILED    =   -20049,
    /*Bootloader error code end*/
    GM_BL_ERROR_INVALID_RANDOM            =   -20050,

}gm_drv_ret_e;


#endif /*__DRV_COMMON_H__*/

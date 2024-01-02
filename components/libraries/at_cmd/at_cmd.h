/**
 *****************************************************************************************
 *
 * @file AT_CMD.h
 *
 * @brief AT Command API
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

#ifndef __AT_CMD_H_
#define __AT_CMD_H_

#include <stdint.h>
#include <stdbool.h>


/**
 * @defgroup AT_CMD_MACRO Defines
 * @{
 */
#define AT_CMD_BUFFER_SIZE_MAX          256      /**< Maximum size of AT CMD buffer. */
#define AT_CMD_ARG_COUNT_MAX            28       /**< Maximum number of arguments input. */
/** @} */

/**
 * @defgroup AT_CMD_ENUM Enumerations
 * @{
 */
/**@brief AT CMD state. */
typedef enum
{
    AT_CMD_IN_READY_PARSE = 0x01,          /**< Ready for AT CMD parse state. */
    AT_CMD_IN_PARSING,                     /**< In parsing AT CMD state. */
    AT_CMD_IN_WAITE_EXECUTE,               /**< Waite for executing state. */
    AT_CMD_IN_EXECUTING,                   /**< In executing state. */
} at_cmd_state_t;

/**@brief AT CMD source type. */
typedef enum
{
    AT_CMD_SRC_INVALID,                   /**< Invalid AT CMD source. */
    AT_CMD_SRC_UART,                      /**< AT CMD source: Uart. */
    AT_CMD_SRC_BLE,                       /**< AT CMD source: BLE. */
} at_cmd_src_t;

/**@brief AT CMD response destination type. */
typedef enum
{
    AT_CMD_RSP_DEST_INVALID,             /**< Invalid AT CMD response destination. */
    AT_CMD_RSP_DEST_UART,                /**< AT CMD response destination: Uart. */
    AT_CMD_RSP_DEST_BLE,                 /**< AT CMD response destination: BLE.*/
} at_cmd_rsp_dest_t;

/**@brief AT CMD ID type. */
typedef enum
{
    AT_CMD_INVALID,            /**< Invalid AT CMD type. */
    AT_CMD_TEST,               /**< Test module AT CMD. */
    AT_CMD_VERSION_GET,        /**< Module version get AT CMD. */
    AT_CMD_RESET,              /**< System reset. */
    AT_CMD_BAUD_SET,           /**< Uart baud rate set AT CMD. */
    AT_CMD_ADDR_GET,           /**< Board address get AT CMD. */
    AT_CMD_GAP_ROLE_GET,       /**< GAP role get AT CMD. */
    AT_CMD_GAP_ROLE_SET,       /**< GAP role set AT CMD. */
    AT_CMD_GAP_NAME_GET,       /**< GAP name get AT CMD. */
    AT_CMD_GAP_NAME_SET,       /**< GAP name set AT CMD. */
    AT_CMD_ADV_PARAM_SET,      /**< Advertising parameters set AT CMD. */
    AT_CMD_ADV_DATA_SET,       /**< Advertising data set AT CMD. */
    AT_CMD_RSP_DATA_SET,       /**< Scan response data set AT CMD. */
    AT_CMD_ADV_START,          /**< Start advertising AT CMD. */
    AT_CMD_ADV_STOP,           /**< Stop advertising AT CMD. */
    AT_CMD_SCAN_PARAM_SET,     /**< Scan parameters set AT CMD. */
    AT_CMD_SCAN_START,         /**< Start scan AT CMD. */
    AT_CMD_SCAN_STOP,          /**< Stop scan AT CMD. */
    AT_CMD_CONN_PARAM_SET,     /**< Connect parameters set AT CMD. */
    AT_CMD_CONN_INIT,          /**< Initiate connection AT CMD. */
    AT_CMD_CONN_CANCEL,        /**< Cancel connection AT CMD. */
    AT_CMD_DISCONN,            /**< Disconnect AT CMD. */
    AT_CMD_MTU_EXCHANGE,       /**< Exchange MTU AT CMD. */
    AT_CMD_SRVC_DISC,          /**< Discovery service AT CMD. */
    AT_CMD_CONN_PARAM_UPDATE,  /**< Connect parameters update AT CMD. */
    AT_CMD_ATTR_READ,          /**< Read attr AT CMD. */
    AT_CMD_ATTR_WRITE,         /**< Write attr AT CMD. */
    AT_CMD_NB                  /**< Number of supported AT CMD. */
} at_cmd_id_t;

/**@brief AT CMD error code. */
typedef enum
{
    AT_CMD_ERR_NO_ERROR,            /**< No error. */
    AT_CMD_ERR_INVALID_INPUT,       /**< Invalid input. */
    AT_CMD_ERR_UNSUPPORTED_CMD,     /**< Unsupported AT CMD. */
    AT_CMD_ERR_PARSE_NOT_ALLOWED,   /**< No allowed parse state. */
    AT_CMD_ERR_CMD_REQ_ALLOWED,     /**< Command request is not allowed. */
    AT_CMD_ERR_NO_CMD_HANDLER,      /**< No AT CMD handler. */
    AT_CMD_ERR_INVALID_PARAM,       /**< Invalid parameters. */
    AT_CMD_ERR_HAL_ERROR,           /**< Hal error. */
    AT_CMD_ERR_TIMEOUT,             /**< AT CMD execute timeout. */

    AT_CMD_ERR_OTHER_ERROR,         /**< Other error code. */
} at_cmd_error_t;
/** @} */

/**
 * @defgroup AT_CMD_STRUCT Structures
 * @{
 */
/**@brief AT CMD parse result. */
typedef struct
{
    at_cmd_id_t    cmd_id;                              /**< AT CMD ID type. */
    uint8_t        cmd_idx;                             /**< AT CMD index in its table. */
    uint8_t        cmd_tag_idx;                         /**< AT CMD tag index. */
    uint8_t        cmd_tag_length;                      /**< Length of AT CMD tag. */
    uint8_t        arg_count;                           /**< Count of arguments. */
    uint8_t        arg_idx[AT_CMD_ARG_COUNT_MAX];       /**< Index of arguments. */
    uint8_t        arg_length[AT_CMD_ARG_COUNT_MAX];    /**< Length of arguments. */
    uint8_t       *p_buff;                              /**< Pointer to AT CMD buffer. */
    uint16_t       buff_length;                         /**< BUffer valid length. */
} at_cmd_parse_t;

/**@brief AT CMD response variables. */
typedef struct
{
    at_cmd_error_t  error_code;                       /**< Error code. */
    uint8_t         data[AT_CMD_BUFFER_SIZE_MAX];     /**< Response data. */
    uint16_t        length;                           /**< Length of response data. */
} at_cmd_rsp_t;
/** @} */

/**
 * @defgroup AT_CMD_TYPEDEF Typedefs
 * @{
 */
/**@brief AT CMD handler type. */
typedef void (*at_cmd_handler_t)(at_cmd_parse_t *p_cmd_parse);

/**@brief AT CMD execute timing callback type. */
typedef void (*at_cmd_time_callback_t)(void);

/**@brief AT CMD execute complete callback type. */
typedef void (*at_cmd_cplt_callback_t)(at_cmd_rsp_dest_t rsp_dest, const uint8_t *p_data, uint8_t length);
/** @} */

/**
 * @defgroup AT_CMD_STRUCT Structures
 * @{
 */
/**@brief AT CMD attribute. */
typedef struct
{
    at_cmd_id_t        cmd_id;                   /**< AT CMD ID type. */
    char              *cmd_tag_str;              /**< String of AT CMD tag. */
    uint8_t            cmd_tag_length;           /**< Length of AT CMD tag. */
    at_cmd_handler_t   cmd_handler;              /**< AT CMD handler. */
    char              *cmd_timeout_str;          /**< String of timeout AT CMD. */
} at_cmd_attr_t;

/**@brief AT CMD initialization variables. */
typedef struct
{
    at_cmd_attr_t            *p_cmd_attr;         /**< Pointer to AT CMD attribute table. */
    uint8_t                   cmd_num;            /**< Number of AT CMD register. */
    at_cmd_time_callback_t    cmd_time_cb;        /**< AT CMD timing callback. */
    at_cmd_cplt_callback_t    cmd_cplt_cb;        /**< AT CMD execute complete callback. */
} at_cmd_init_t;
/** @} */

/**
 * @defgroup AT_CMD_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize AT CMD module.
 *
 * @param[in] p_cmd_init: Pointer to p_cmd_init.
 *****************************************************************************************
 */
void at_cmd_init(at_cmd_init_t *p_cmd_init);

/**
 *****************************************************************************************
 * @brief Parse user input data.
 *
 * @param[in] cmd_src: Source of input.
 * @param[in] p_data:  Pointer to input data.
 * @param[in] length:  Length of input data.
 *****************************************************************************************
 */
void at_cmd_parse(at_cmd_src_t cmd_src, const uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Process AT CMD execute timing check.
 *****************************************************************************************
 */
void at_cmd_execute_timing_process(void);

/**
 *****************************************************************************************
 * @brief AT CMD execute complete.
 *
 * @param[in] p_cmd_rsp: Pointer to cmd response.
 *****************************************************************************************
 */
void at_cmd_execute_cplt(at_cmd_rsp_t *p_cmd_rsp);

/**
 *****************************************************************************************
 * @brief AT CMD schedule, called in main().
 *****************************************************************************************
 */
void at_cmd_schedule(void);
/** @} */

#endif



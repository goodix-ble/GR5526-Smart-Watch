/**
 ****************************************************************************************
 *
 * @file ams_c.h
 *
 * @brief Apple Media Service Client API.
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
 *****************************************************************************************
 */
/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 */

/**
 * @defgroup BLE_SDK_AMS_C Apple Media Service Client (AMS_C)
 * @{
 * @brief Apple Media Service Client module.
 *
 * @details The Apple Media Service Client contains the APIs and types, which can be used
 *          by the application to discover Apple Media Service of peer and interact with it.
 *
 *          The application must provide an event handler to be registered, then call \ref ams_c_client_init() to initialize the client.
 *          After the application discovers peer Apple Media Service by calling \ref ams_c_disc_srvc_start(),
 *          application can call \ref ams_c_cmd_send() to send romote command to peer and \ref ams_c_attr_focus_set to
 *          set the attribute focus. 
 *
 *          Secondly, use \ref ams_c_cmd_notify_set() and \ref ams_c_attr_update_notify_set() to enable the notification
 *          for CMD update and attribute update.
 *
 *          When the available CMDs or concerned attributes change, the module will receive notification from peer
 *          if notifications of them are enabled. If the notification containing the information of changed attribute
 *          is truncated, application can call \ref ams_c_attr_display_set to set the attribute that needs to be completely 
 *          displayed and \ref ams_c_cplt_attr_read to get the completely information of the attribute.

 */

#ifndef __AMS_C_H__
#define __AMS_C_H__

#include "gr_includes.h"
#include "ble_prf_types.h"
#include "custom_config.h"

/**
 * @defgroup AMS_C_MACRO Defines
 * @{
 */
#define AMS_C_CONNECTION_MAX                10                                                   /**< Maximum number of HRS Client connections. */         
#define AMS_C_ATTR_COUNT_MAX                256                                                  /**< The buffer size of command. */ 
#define AMS_C_TRUNCATED_FLAG                (0x01<<0)                                            /**< Bit of truncated. */
#define AMS_SRVC_UUID                       0xdc, 0xf8, 0x55, 0xad, 0x02, 0xc5, 0xf4, 0x8e,\
                                            0x3a, 0x43, 0x36, 0x0f, 0x2b, 0x50, 0xd3, 0x89       /**< UUID of Apple media service. */
#define AMS_CMD_UUID                        0xc2, 0x51, 0xca, 0xf7, 0x56, 0x0e, 0xdf, 0xb8,\
                                            0x8a, 0x4a, 0xb1, 0x57, 0xd8, 0x81, 0x3c, 0x9b       /**< UUID of remote command. */
#define AMS_ATTR_UPDATE_UUID                0x02, 0xC1, 0x96, 0xBA, 0x92, 0xBB, 0x0C, 0x9A,\
                                            0x1F, 0x41, 0x8D, 0x80, 0xCE, 0xAB, 0x7C, 0x2F       /**< UUID of attribute update. */
#define AMS_ATTR_DISPLAY_UUID               0xd7, 0xd5, 0xbb, 0x70, 0xa8, 0xa3, 0xab, 0xa6,\
                                            0xd8, 0x46, 0xab, 0x23, 0x8c, 0xf3, 0xb2, 0xc6       /**< UUID of attribute display. */
/** @} */

/**
 * @defgroup AMS_C_ENUM Enumerations
 * @{
 */

/**@brief Apple Media Service Command ID. */
typedef enum
{
    AMS_CMD_ID_PLAY,                                   /**< Command index of play. */
    AMS_CMD_ID_PAUSE,                                  /**< Command index of pause. */
    AMS_CMD_ID_TOGGLE_PLAY_PAUSE,                      /**< Command index of toggle. */
    AMS_CMD_ID_NEXT_TRACK,                             /**< Command index of next track. */
    AMS_CMD_ID_PREVIOUS_TRACK,                         /**< Command index of previous track. */
    AMS_CMD_ID_VOLUME_UP,                              /**< Command index of volume up. */
    AMS_CMD_ID_VOLUME_DOWN,                            /**< Command index of volume down. */
    AMS_CMD_ID_ADVANCE_REPEAT_MODE,                    /**< Command index of repeat mode. */
    AMS_CMD_ID_ADVANCE_SHUFFLE_MODE,                   /**< Command index of shuffle mode. */
    AMS_CMD_ID_SKIP_FORWARD,                           /**< Command index of skip forward. */
    AMS_CMD_ID_SKIP_BACKWARD,                          /**< Command index of skip backward. */
    AMS_CMD_ID_LIKE_TRACK,                             /**< Command index of like track. */
    AMS_CMD_ID_DISLIKE_TRACK,                          /**< Command index of dislike track. */
    AMS_CMD_ID_BOOK_MARK_TRACK,                        /**< ComMand index of book mark. */   
} ams_c_cmd_id_t;

/**@brief Apple Media Service entities index. */
typedef enum
{
    AMS_ETT_ID_PLAYER,                                 /**< Entity index of player. */
    AMS_ETT_ID_QUEUE,                                  /**< Entity index of queue. */
    AMS_ETT_ID_TRACK,                                  /**< Entity index of track. */
} ams_c_ett_id_t;

/**@brief Apple Media Service player attribute index. */
enum
{
    AMS_PLAYER_ATTR_ID_NAME,                           /**< Player attribute index of name. */
    AMS_PLAYER_ATTR_ID_PLAYBACK_INFO,                  /**< Player attribute index of playback information. */
    AMS_PLAYER_ATTR_ID_VOLUME,                         /**< Player attribute index of volume. */
};

/**@brief Apple Media Service queue attribute index. */
enum
{
    AMS_QUEUE_ATTR_ID_INDEX,                           /**< Queue attribute index of index. */
    AMS_QUEUE_ATTR_ID_COUNT,                           /**< Queue attribute index of count. */
    AMS_QUEUE_ATTR_ID_SHUFFLE_MODE,                    /**< Queue attribute index of shuffle mode. */
    AMS_QUEUE_ATTR_ID_REPEAT_MODE,                     /**< Queue attribute index of repeat mode. */
};

/**@brief Apple Media Service track attribute index. */
enum
{
    AMS_TRACK_ATTR_ID_ARTIST,                          /**< Track attribute index of artist. */
    AMS_TRACK_ATTR_ID_ALBUM,                           /**< Track attribute index of album. */
    AMS_TRACK_ATTR_ID_TITTLE,                          /**< Track attribute index of tittle. */
    AMS_TRACK_ATTR_ID_DURATION,                        /**< Track attribute index of duration. */
};

/**@brief Apple Media Service Client Event type. */
typedef enum
{
    AMS_C_EVT_INVALID,                                 /**< AMS Client invalid event type. */    
    AMS_C_EVT_DISCOVERY_CPLT,                          /**< AMS Client has found AMS service and its characteristics. */
    AMS_C_EVT_DISCOVERY_FAIL,                          /**< AMS Client found AMS service failed because of invalid operation or no found at the peer. */
    AMS_C_EVT_CMD_SEND_SUCCESS,                        /**< AMS Client has sent command. */
    AMS_C_EVT_CMD_UPDATE_RECEIVE,                      /**< AMS Client has recieved updated command list. */
    AMS_C_EVT_CMD_UPDATE_NTF_SET_SUCCESS,              /**< AMS Client has set command update notification. */
    AMS_C_EVT_ATTR_FOCUS_SET_SUCCESS,                  /**< AMS Client has set focus attribute */
    AMS_C_EVT_ATTR_UPDATE_RECEIVE,                     /**< AMS Client has received updated attribution data. */
    AMS_C_EVT_ATTR_UPDATE_NTF_SET_SUCCESS,             /**< AMS Client has set focus attribute notification. */
    AMS_C_EVT_CPLT_ATTR_DISPLAY_SET_SUCCESS,           /**< AMS Client has set the attribute which needs to be completely displayed. */
    AMS_C_EVT_CPLT_ATTR_READ_RSP,                      /**< AMS Client has received a read response. */
    AMS_C_EVT_WRITE_OP_ERR,                            /**< Write error. */
} ams_c_evt_type_t;
/** @} */

/**
 * @defgroup AMS_C_STRUCT Structures
 * @{
 */

/**@brief Structure that stores the attribute to be concerned or to display completely. */
typedef struct
{
    ams_c_ett_id_t      ett_id;                        /**< Entity index. */
    uint8_t             attr_id[AMS_C_ATTR_COUNT_MAX]; /**< Attribute indexs. */
    uint16_t            attr_count;                    /**< Count of attribute. */
} ams_c_ett_attr_id_t;

/**@brief Structure that stores new command list. */
typedef struct
{
    ams_c_cmd_id_t       *p_cmd;                       /**< Command list. */
    uint16_t              length;                      /**< Count of Command. */
} ams_c_cmd_list_t;

/**@brief Structure that stores attribute information. */
typedef struct
{
    ams_c_ett_id_t        ett_id;                      /**< Entity index. */
    uint8_t               attr_id;                     /**< Attribute index. */
    uint8_t               flag;                        /**< Flag about attribute. */
    uint8_t              *p_data;                      /**< Attribute data. */
    uint16_t              length;                      /**< Length of attribute data. */
} ams_c_attr_info_t;

/**@brief Complete attribution's value . */
typedef struct
{
    uint8_t              *p_data;                      /**< complete attribute data. */
    uint16_t              length;                      /**< Length of complete attribute data. */
} ams_c_cplt_attr_data_t;

/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t ams_srvc_start_handle;                    /**< AMS Service start handle. */
    uint16_t ams_srvc_end_handle;                      /**< AMS Service end handle. */
    uint16_t ams_cmd_handle;                           /**< AMS Service remote command handle. */
    uint16_t ams_cmd_cccd_handle;                      /**< AMS Service remote command cccd handle. */
    uint16_t ams_attr_update_handle;                   /**< AMS Service update attribute handle. */
    uint16_t ams_attr_update_cccd_handle;              /**< AMS Service update attribute cccd handle. */
    uint16_t ams_attr_display_handle;                  /**< AMS Service complete attribute display handle. */
} ams_c_handles_t;

/**@brief Apple Media Service Client event. */
typedef struct
{
    uint8_t          conn_idx;                         /**< The index of the connection. */
    ams_c_evt_type_t evt_type;                         /**< The AMS event type. */
    union
    {
        ams_c_cmd_list_t       cmd_list;               /**< Command list. */
        ams_c_attr_info_t      attr_info;              /**< Attribute information. */
        ams_c_cplt_attr_data_t cplt_attr_data;         /**< Complete attribute data. */
    } param;                                           /**< Data received. */
} ams_c_evt_t;
/** @} */

/**
 * @defgroup AMS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief Apple Media Service Client event handler type.*/
typedef void (*ams_c_evt_handler_t)(ams_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup AMS_FUNCTION Functions
 * @{
 */

/**
 *****************************************************************************************
 * @brief Register AMS Client event handler.
 *
 * @param[in] evt_handler: Apple Media Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t ams_c_client_init(ams_c_evt_handler_t evt_handler);;

/**
 *****************************************************************************************
 * @brief Discover Apple Media Service on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ams_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer new command list notify.
 *
 * @param[in] conn_idx:  Index of connection.
 * @param[in] is_enable: True for turn on, false for turn off.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ams_c_cmd_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Enable or disable peer updated attribute notify.
 *
 * @param[in] conn_idx:  Index of connection.
 * @param[in] is_enable: True for turn on, false for turn off.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t ams_c_attr_update_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Read complete value of updated attrbute .
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
*/
sdk_err_t ams_c_cplt_attr_read(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Send command to peer device.
 *
 * @param[in] conn_idx: Index of connection.
 * @param[in] cmd_id: Index of command. 
*
 * @return Operation result.
 *****************************************************************************************
*/
sdk_err_t ams_c_cmd_send(uint8_t conn_idx, uint8_t cmd_id);

/**
 *****************************************************************************************
 * @brief Set concerned attribute.
 *
 * @param[in] conn_idx: Index of connection.
 * @param[in] p_ett_attr_id: Point to the structure containing concerned attribute
*
 * @return Operation result.
 *****************************************************************************************
*/
sdk_err_t ams_c_attr_focus_set(uint8_t conn_idx, const ams_c_ett_attr_id_t *p_ett_attr_id);

/**
 *****************************************************************************************
 * @brief Set the attribute that needs to be completely displayed.
 *
 * @param[in] conn_idx:    Index of connection.
 * @param[in] p_attr_info: Point to the structure containing concerned attribute
*
 * @return Operation result.
 *****************************************************************************************
*/
sdk_err_t ams_c_attr_display_set(uint8_t conn_idx, const ams_c_attr_info_t *p_attr_info);

/**
 *****************************************************************************************
 * @brief Check if the command is available.
 *
 * @param[in] cmd_id: Index of command.
*
 * @return Operation result.
 *****************************************************************************************
*/
bool ams_c_cmd_enable_check(ams_c_cmd_id_t cmd_id);
/** @} */

#endif
/** @} */
/** @} */

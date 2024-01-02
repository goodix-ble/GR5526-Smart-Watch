/**
 ****************************************************************************************
 *
 * @file app_linked_list.h
 *
 * @brief App Linked List API
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

#ifndef __APP_LINKED_LIST_H__
#define __APP_LINKED_LIST_H__

#include "grx_hal.h"
#include "grx_sys.h"
#include "app_memory.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup APP_LINKED_LIST_STRUCT Structures
 * @{
 */
/**@brief App Singly Linked List Node */
typedef struct singly_list_node
{
    void                    *p_data;          /**< Pointer to data. */
    struct singly_list_node *p_next_node;     /**< Pointer to next node. */
} app_s_list_node_t;

/**@brief App Singly Linked List */
typedef struct
{
    app_s_list_node_t *p_head;     /**< Pointer to head node. */
    uint32_t           size;       /**< Size of list. */
} app_s_list_t;
/** @} */

/**
 * @defgroup APP_LINKED_LIST_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize one singly linked list instance.
 *
 * @param[in] p_s_list: Pointer to a singly linked list instance.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t app_s_list_init(app_s_list_t *p_s_list);

/**
 *****************************************************************************************
 * @brief Append one singly linked list node at tail.
 *
 * @param[in] p_s_list: Pointer to a singly linked list instance.
 *
 * @return Pointer to append node.
 *****************************************************************************************
 */
app_s_list_node_t *app_s_list_node_append(app_s_list_t *p_s_list);

/**
 *****************************************************************************************
 * @brief Insert one singly linked list node into designated location.
 *
 * @param[in] p_s_list:      Pointer to a singly linked list instance.
 * @param[in] p_dgt_node:    Pointer to a designated singly linked list node.
 * @param[in] is_ahead:      True: insert in ahead of designated node, False: insert in behind of designated node.
 *
 * @return Pointer to insert node.
 *****************************************************************************************
 */
app_s_list_node_t *app_s_list_node_insert(app_s_list_t *p_s_list, app_s_list_node_t *p_dgt_node, bool is_ahead);

/**
 *****************************************************************************************
 * @brief Delete one designated singly linked list node.
 *
 * @param[in] p_s_list:       Pointer to a singly linked list instance.
 * @param[in] p_s_list_node:  Pointer to a designated singly linked list node.
 * @param[in] free_data:      Free node data or not.
 *
 * @return Result of delete.
 *****************************************************************************************
 */
sdk_err_t app_s_list_node_delete(app_s_list_t *p_s_list, app_s_list_node_t *p_s_list_node, bool free_data);

/**
 *****************************************************************************************
 * @brief Clear one singly linked list instance.
 *
 * @param[in] p_s_list:    Pointer to a singly linked list instance.
 * @param[in] free_data:   Free node data or not.
 *
 * @return Result of clear.
 *****************************************************************************************
 */
sdk_err_t app_s_list_clear(app_s_list_t *p_s_list, bool free_data);



/** @} */

#endif



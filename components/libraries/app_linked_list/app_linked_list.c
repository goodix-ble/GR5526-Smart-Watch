/**
 *****************************************************************************************
 *
 * @file app_linked_list.c
 *
 * @brief App Linked List Implementation.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_linked_list.h"
#include <string.h>


/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t app_s_list_init(app_s_list_t *p_s_list)
{
    if (NULL == p_s_list)
    {
        return SDK_ERR_POINTER_NULL;
    }

    p_s_list->p_head = NULL;
    p_s_list->size   = 0;

    return SDK_SUCCESS;
}


app_s_list_node_t *app_s_list_node_append(app_s_list_t *p_s_list)
{
    app_s_list_node_t *p_iterator_node;
    app_s_list_node_t *p_s_insert_node;

    if (NULL == p_s_list)
    {
        return NULL;
    }

    p_s_insert_node = app_malloc(sizeof(app_s_list_node_t));

    if (NULL == p_s_insert_node)
    {
        return NULL;
    }

    memset(p_s_insert_node, 0, sizeof(app_s_list_node_t));

    if (NULL == p_s_list->p_head)
    {
        p_s_list->p_head = p_s_insert_node;
        p_s_list->size   = 1;

        return p_s_insert_node;
    }

    p_iterator_node = p_s_list->p_head;

    while(p_iterator_node->p_next_node)
    {
        p_iterator_node = p_iterator_node->p_next_node;
    }

    p_iterator_node->p_next_node = p_s_insert_node;
    p_s_insert_node->p_next_node   = NULL;

    p_s_list->size++;

    return p_s_insert_node;
}

app_s_list_node_t *app_s_list_node_insert(app_s_list_t *p_s_list, app_s_list_node_t *p_dgt_node, bool is_ahead)
{
    app_s_list_node_t *p_previous_node;
    app_s_list_node_t *p_iterator_node;
    app_s_list_node_t *p_s_insert_node;

    if (NULL == p_s_list || NULL == p_dgt_node)
    {
        return NULL;
    }

    p_s_insert_node = app_malloc(sizeof(app_s_list_node_t));

    if (NULL == p_s_insert_node)
    {
        return NULL;
    }


    p_previous_node = p_s_list->p_head;
    p_iterator_node = p_s_list->p_head;

    do
    {
        if (p_iterator_node == p_dgt_node)
        {
            break;
        }

        p_previous_node = p_iterator_node;
        p_iterator_node = p_iterator_node->p_next_node;

    } while (p_iterator_node);

    if (NULL == p_iterator_node)
    {
        return NULL;
    }

    if (is_ahead)
    {
        if (p_iterator_node == p_s_list->p_head)
        {
            p_s_insert_node->p_next_node = p_iterator_node;
            p_s_list->p_head             = p_s_insert_node;
        }
        else
        {
            p_s_insert_node->p_next_node   = p_iterator_node;
            p_previous_node->p_next_node   = p_s_insert_node;
        }
    }
    else
    {
        p_s_insert_node->p_next_node  = p_iterator_node->p_next_node;
        p_iterator_node->p_next_node  = p_s_insert_node;
    }

    p_s_list->size++;

    return p_s_insert_node;
}

sdk_err_t app_s_list_node_delete(app_s_list_t *p_s_list, app_s_list_node_t *p_s_list_node, bool free_data)
{
    app_s_list_node_t *p_previous_node;
    app_s_list_node_t *p_iterator_node;

    if (NULL == p_s_list || NULL == p_s_list_node)
    {
        return SDK_ERR_POINTER_NULL;
    }

    if (p_s_list_node == p_s_list->p_head)
    {
        p_s_list->p_head = p_s_list_node->p_next_node;
        if (free_data)
        {
            app_free(p_s_list_node->p_data);
        }
        app_free(p_s_list_node);

        return SDK_SUCCESS;
    }

    p_previous_node = p_s_list->p_head;
    p_iterator_node = p_s_list->p_head;

    do
    {
        if (p_iterator_node == p_s_list_node)
        {
            break;
        }

        p_previous_node = p_iterator_node;
        p_iterator_node = p_iterator_node->p_next_node;

    } while (p_iterator_node);

    if (NULL == p_iterator_node)
    {
        return SDK_ERR_LIST_ITEM_NOT_FOUND;
    }

    p_previous_node->p_next_node = p_iterator_node->p_next_node;
    if (free_data)
    {
        app_free(p_s_list_node->p_data);
    }
    app_free(p_s_list_node);

    p_s_list->size--;

    return SDK_SUCCESS;
}

sdk_err_t app_s_list_clear(app_s_list_t *p_s_list, bool free_data)
{
    app_s_list_node_t *p_iterator_node;

    if (NULL == p_s_list)
    {
        return SDK_ERR_POINTER_NULL;
    }

    if (NULL == p_s_list->p_head)
    {
        return SDK_SUCCESS;
    }

    p_iterator_node = p_s_list->p_head;

    while (p_iterator_node) 
    {
        if (free_data)
        {
            app_free(p_iterator_node->p_data);
        }
        p_s_list->p_head = p_iterator_node->p_next_node;
        app_free(p_iterator_node);
        p_iterator_node = p_s_list->p_head;
    };

    p_s_list->size = 0;

    return SDK_SUCCESS;
}


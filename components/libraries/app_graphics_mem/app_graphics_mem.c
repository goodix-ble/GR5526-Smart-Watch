/**
 *****************************************************************************************
 *
 * @file app_graphics_mem.c
 *
 * @brief app graphics memory API implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2022 GOODIX
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
#include "grx_hal.h"
#include "app_graphics_mem.h"
#include "app_graphics_ospi.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/// Enable the partially refresh low power mode
#define PSRAM_PARTIAL_REFR_ENABLE (0)

/// Heap size for graphics mem heap cur_node
#define MEM_INFO_NODE_BYTES (32)
#define MEM_INFO_POOL_SIZE (MEM_INFO_NODE_BYTES * GFX_MAX_MEM_MALLOC_NB)

/// Pattern used to check if memory block is not corrupted
#define MEM_FREE_STATE (0xA5A55A5A)    // mark the element is freed
#define MEM_USED_STATE (0x83833838)    // mark the element is allocated
#define MEM_FREE_ON_GOING (0xF0F00F0F) // mark the free action on-going

/// Align val on the multiple of 4 equal or nearest higher
#define CO_ALIGN4_HI(val)  (((val) + 3) & (~3))
#define CO_ALIGN16_HI(val) (((val) + 15) & (~15))

/// Define Block debug print info control macro
#define GMEM_DBG_INFO_ENABLE 0
#if GMEM_DBG_INFO_ENABLE
#define GMEM_DBG_PRINTF printf
#define GFX_LOG_FLUSH app_log_flush
#else
#define GMEM_DBG_PRINTF(fmt, ...)
#define GFX_LOG_FLUSH()
#endif

/// External assert functions for error and warning report
extern void assert_err(const char *condition, const char *file, int line);
extern void assert_param(int param0, int param1, const char *file, int line);
extern void assert_warn(int param0, int param1, const char *file, int line);
extern void app_log_flush(void);

/// Assertions showing a critical error that could require a full system reset
#define ASSERT_ERR(cond)                             \
    do                                               \
    {                                                \
        if (!(cond))                                 \
        {                                            \
            assert_err(#cond, __MODULE__, __LINE__); \
        }                                            \
    } while (0)

/// Assertions showing a critical error that could require a full system reset
#define ASSERT_INFO(cond, param0, param1)                                 \
    do                                                                    \
    {                                                                     \
        if (!(cond))                                                      \
        {                                                                 \
            assert_param((int)param0, (int)param1, __MODULE__, __LINE__); \
        }                                                                 \
    } while (0)

/// Assertions showing a non-critical problem that has to be fixed by the SW
#define ASSERT_WARN(cond, param0, param1)                                \
    do                                                                   \
    {                                                                    \
        if (!(cond))                                                     \
        {                                                                \
            assert_warn((int)param0, (int)param1, __MODULE__, __LINE__); \
        }                                                                \
    } while (0)

/// Free memory block structure (size must be 4bytes multiple)
/// Heap can be represented as a doubled linked list.
struct mem_info_free
{
    /// Used to check if memory block is allocated or free
    uint32_t state;
    /// Size of the current free block (including descriptor space)
    uint32_t free_size;
    /// Next free block pointer
    struct mem_info_free *next;
    /// Previous free block pointer
    struct mem_info_free *previous;
};

/// Used memory block descriptor space structure (size must be 4bytes multiple)
struct mem_info_used
{
    /// Used to check if memory block is allocated or free
    uint32_t state;
    /// Size of the current used block (including descriptor space)
    uint32_t size;
};

/// Free memory block descriptor space structure (size must be 4bytes multiple)
/// Heap can be represented as a doubled linked list.
struct mblock_desc
{
    /// Used to check if memory block is allocated or free
    uint32_t address;
    /// Size of the current memory block
    uint32_t size;
    /// Next free block pointer
    struct mblock_desc *next;
    /// Previous free block pointer
    struct mblock_desc *previous;
};

/**@brief app graphics cur_node heap management variable. */
/// Total size of heaps when initialized
static uint32_t gfx_mem_info_pool_init_size;
/// Mem Info Heap Pool definition
static uint8_t gfx_mem_info_pool[MEM_INFO_POOL_SIZE] __attribute__((aligned(32)));
/// The Root pointer = pointer to first element of heap linked lists
static struct mem_info_free *gfx_free_mem_info_list;
/// Total size representing dynamic used heaps
static uint32_t gfx_mem_info_used_size;

/**@brief app graphics mem heap management variable. */
/// Total size of heaps when initialized
static uint32_t gfx_mem_buffer_pool_init_size;
/// Mem Root pointer = pointer to first element of heap linked lists
static struct mblock_desc *gfx_free_mem_buffer_list;
/// Mem Root pointer = pointer to first element of used heap linked lists
static struct mblock_desc *gfx_used_mem_buffer_list;
/// Total size representing dynamic used heaps
static uint32_t gfx_used_mem_buffer_total_size;

/*
 * EXPORTED MEM cur_node MANAGEMENT FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void gfx_mem_info_mem_init(void)
{
    // 4bytes align first free descriptor to word boundary
    gfx_free_mem_info_list = (struct mem_info_free *)CO_ALIGN4_HI((uint32_t)gfx_mem_info_pool);

    // protect accesses to descriptors
    GLOBAL_EXCEPTION_DISABLE();

    // initialize the first block
    // 1) compute the size from the last aligned word before heap_end
    gfx_free_mem_info_list->free_size = ((uint32_t)&gfx_mem_info_pool[MEM_INFO_POOL_SIZE] & (~3)) - (uint32_t)(gfx_free_mem_info_list);
    GMEM_DBG_PRINTF("mem_info_free_size: %d\n\r\n\r", gfx_free_mem_info_list->free_size);
    GFX_LOG_FLUSH(); // polling dump the log information

    // 2) initialize the state pattern and list
    gfx_free_mem_info_list->state = MEM_FREE_STATE;
    gfx_free_mem_info_list->next = NULL;
    gfx_free_mem_info_list->previous = NULL;

    gfx_mem_info_pool_init_size = MEM_INFO_POOL_SIZE;

    GLOBAL_EXCEPTION_ENABLE();
}

void *gfx_mem_info_malloc(uint32_t size)
{
    struct mem_info_free *cur_node = NULL, *select_node = NULL;
    struct mem_info_used *alloc = NULL;
    uint32_t total_free_size = 0;

    // compute overall block size (including requested size PLUS descriptor size)
    uint32_t totalsize = CO_ALIGN4_HI(size) + sizeof(struct mem_info_used);
    // ensure the allocated memory could be restored as a free node
    if (totalsize < sizeof(struct mem_info_free))
    {
        totalsize = sizeof(struct mem_info_free);
    }

    // protect accesses to descriptors
    GLOBAL_EXCEPTION_DISABLE();

    // 1) find a smallest node for memory allocation
    {
        // point the current node to the free mem list(root node)
        cur_node = gfx_free_mem_info_list;
        ASSERT_ERR(cur_node != NULL);

        // go through free memory list and select the smallest match
        while (cur_node != NULL)
        {
            ASSERT_ERR(cur_node->state == MEM_FREE_STATE);

            total_free_size += cur_node->free_size;

            // ensure the cur_node has enough space for mblok_free structure after allocated
            if (cur_node->free_size >= (totalsize + sizeof(struct mem_info_free)))
            {
                // select_node update strategy to seek smallest one
                // 1) update the select_node if the select_node is NULL
                // 2) update the select_node if the current cur_node is a smaller one
                if ((NULL == select_node) || (select_node->free_size > cur_node->free_size))
                {
                    select_node = cur_node; // update the select_node to current cur_node
                }
            }
            // move to next block
            cur_node = cur_node->next;
        }
    }

    ASSERT_INFO(select_node != NULL, size, total_free_size);

    // 2) allocate a memory and update the free link list
    {
        // select_node is completely reused when size is equal
        if (select_node->free_size == totalsize)
        {
            // select_node is not the root node
            if (select_node->previous != NULL)
            {
                // remove the select_node from the free list
                // point the previous node to the next node
                select_node->previous->next = select_node->next;
                if (select_node->next != NULL)
                {
                    select_node->next->previous = select_node->previous;
                }
            }
            else // select_node is the root node
            {
                // it is not allowed when the root node is fully reused
                ASSERT_ERR(NULL != select_node->next);
                gfx_free_mem_info_list = select_node->next;
                gfx_free_mem_info_list->previous = NULL;
            }

            // completely reuse the selected free node
            alloc = (struct mem_info_used *)((uint32_t)select_node);
        }
        else // the selected node free size is bigger than the totalsize
        {
            // decrease the free_size in the select_node cur_node for allocation
            select_node->free_size -= totalsize;

            // compute the pointer to the beginning of the free space
            alloc = (struct mem_info_used *)((uint32_t)select_node + select_node->free_size);
        }

        // save the size of the allocated block
        alloc->size = totalsize;
        alloc->state = MEM_USED_STATE;

        // update the mem memory profiling info
        gfx_mem_info_used_size = gfx_mem_info_pool_init_size + alloc->size - total_free_size;
    }

    // notice shall PLUS move to the user memory space
    alloc++;

    GMEM_DBG_PRINTF("mem_info_alloc_addr: %08X, size: %08X, end_addr: %08X, mem_info_used_size: %08X\n\r\n\r",
                   (uint32_t)alloc, totalsize, (((uint32_t)alloc) + totalsize), gfx_mem_info_used_size);
    GFX_LOG_FLUSH(); // polling dump the log information

    // end of protection (as early as possible)
    GLOBAL_EXCEPTION_ENABLE();

    return (void *)(alloc);
}

static bool gfx_mem_info_is_in_heap(void *mem_ptr)
{
    bool ret = false;
    uint8_t *block = (uint8_t *)gfx_free_mem_info_list;
    uint32_t size = gfx_mem_info_pool_init_size;
    if ((((uint32_t)mem_ptr) >= ((uint32_t)block)) && (((uint32_t)mem_ptr) <= (((uint32_t)block) + size)))
    {
        ret = true;
    }
    return ret;
}

void gfx_mem_info_free(void *mem_ptr)
{
    // The address shall be ascending in the free memory list and not NULL
    ASSERT_INFO(mem_ptr != NULL, mem_ptr, 0);

    struct mem_info_free *freed;
    struct mem_info_used *bfreed;
    // record next_node and prev_node for convinient link operation
    struct mem_info_free *cur_node = NULL, *next_node = NULL, *prev_node = NULL;

    // point to the block descriptor (before user memory so decrement)
    bfreed = ((struct mem_info_used *)mem_ptr) - 1;

    // protect accesses to descriptors
    GLOBAL_EXCEPTION_DISABLE();

    // check if memory block is allocated or free
    ASSERT_INFO(bfreed->state == MEM_USED_STATE, bfreed->state, mem_ptr);
    // change corruption token in order to know if buffer has been already freed.
    bfreed->state = MEM_FREE_ON_GOING;

    // record the free node size
    uint32_t bfreed_size = bfreed->size;
    // point to the first cur_node of the free elements linked list
    freed = ((struct mem_info_free *)bfreed);

    // 1) Retrieve the root cur_node (where memory block comes from)
    {
        if (gfx_mem_info_is_in_heap(mem_ptr))
        {
            // point the cur_node to the root of the free list
            cur_node = gfx_free_mem_info_list;
        }
        else
        {
            // the memory to be freed is not in the heap
            ASSERT_ERR(0);
        }
    }

    // sanity checks
    ASSERT_ERR(cur_node != NULL);
    // notice the root node has the lowest address
    // the node address in the free list shall be increased from the root node
    ASSERT_ERR(((uint32_t)mem_ptr > (uint32_t)cur_node));

    while (cur_node != NULL)
    {
        // ensure the root cur_node and select_node cur_node is in free state
        ASSERT_ERR(cur_node->state == MEM_FREE_STATE);

        // 1) the cur_node is right after the last(lowest address) free block
        uint32_t node_tail = ((uint32_t)cur_node + cur_node->free_size);
        if ((uint32_t)freed == node_tail)
        {
            // append the freed block to the current one
            cur_node->free_size += bfreed_size;

            // check if this merge made the link between free blocks
            if ((NULL != cur_node->next) &&
                (((uint32_t)cur_node->next) == (((uint32_t)cur_node) + cur_node->free_size)))
            {
                // merge cur_node with next blocks when the merge generate a new merge
                next_node = cur_node->next;
                // add the size of the next cur_node to the current cur_node
                cur_node->free_size += next_node->free_size;
                // merge the next node with current node
                cur_node->next = next_node->next;
                // update linked list.
                if (next_node->next != NULL)
                {
                    next_node->next->previous = cur_node;
                }
            }
            break;
        } // 2) the cur_node is before the current free block
        else if ((uint32_t)freed < (uint32_t)cur_node)
        {
            freed->state = MEM_FREE_STATE;
            // if the current node is not the root node
            // note only the root node previous is NULL
            if (prev_node != NULL)
            {
                // put the freed node before cur_node
                prev_node->next = freed;
                freed->previous = prev_node;
            }
            else // if the current node is the root node
            {
                // update the root node and put the freed before cur_node
                gfx_free_mem_info_list = freed;
                freed->previous = NULL;
            }

            // decide whether to merge the freed node and update the free_size
            if (((uint32_t)freed + bfreed_size) == (uint32_t)cur_node)
            {
                // 2.1) the released cur_node is right before the current cur_node block
                // merge the two nodes
                freed->next = cur_node->next;
                if (cur_node->next != NULL)
                {
                    cur_node->next->previous = freed;
                }
                freed->free_size = cur_node->free_size + bfreed_size;
            }
            else
            {
                // 2.2) the release cur_node is before the current cur_node block and not continuous
                // insert the release cur_node
                freed->next = cur_node;
                cur_node->previous = freed;
                freed->free_size = bfreed_size;
            }
            break;
        }

        // move to the next free block cur_node
        prev_node = cur_node;
        cur_node = cur_node->next;
    }

    // The freed node has the highest address and shall be in the last of the free list
    if (NULL == cur_node)
    {
        prev_node->next = (struct mem_info_free *)freed;
        freed->next = NULL;
        freed->previous = prev_node;
        freed->free_size = bfreed_size;
        freed->state = MEM_FREE_STATE;
    }

    // update the mem memory profiling info
    gfx_mem_info_used_size -= bfreed_size;

    GMEM_DBG_PRINTF("mem_info_free_addr: %08X, size: %08X, end_addr: %08X, mem_info_used_size: %08X\n\r\n\r",
                   (uint32_t)mem_ptr, bfreed_size, (((uint32_t)mem_ptr) + bfreed_size), gfx_mem_info_used_size);
    GFX_LOG_FLUSH(); // polling dump the log information

    // end of protection
    GLOBAL_EXCEPTION_ENABLE();
}

bool gfx_mem_info_is_free(void *mem_ptr)
{
    bool result;
    struct mem_info_used *p_mem_info = ((struct mem_info_used *)mem_ptr) - 1;
    // use corrupt check info in order to know if pointer already free.
    result = (p_mem_info->state != MEM_USED_STATE);
    return (result);
}

uint32_t gfx_mem_info_pool_used_size_get(void)
{
    return gfx_mem_info_used_size;
}

/*
 * MEM MANAGEMENT FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_graphics_mem_init(uint8_t *heap, uint32_t heap_size)
{
    // prepare mem info pool for mem management
    gfx_mem_info_mem_init();

    GLOBAL_EXCEPTION_DISABLE();

    // 4bytes align first free descriptor to word boundary
    // 1) malloc a memory block descriptor as an root node for free memory buffer list
    gfx_free_mem_buffer_list = (struct mblock_desc *)gfx_mem_info_malloc(sizeof(struct mblock_desc));

    // record the heap init size
    gfx_mem_buffer_pool_init_size = heap_size;
    GMEM_DBG_PRINTF("gfx_free_mem_buffer_init_size: %d\n\r\n\r", heap_size);

    // initialize the free list list
    gfx_free_mem_buffer_list->address = CO_ALIGN16_HI((uint32_t)heap);
    GMEM_DBG_PRINTF("gfx_root_free_mem_buffer_address: %08X\n\r\n\r", gfx_free_mem_buffer_list->address);

    gfx_free_mem_buffer_list->size = heap_size;
    GMEM_DBG_PRINTF("gfx_root_free_mem_buffer_size: %d\n\r\n\r", gfx_free_mem_buffer_list->size);
    gfx_free_mem_buffer_list->next = NULL;
    gfx_free_mem_buffer_list->previous = NULL;

    // 2) malloc a memory block descriptor as an root node for used memory buffer list
    gfx_used_mem_buffer_list = (struct mblock_desc *)gfx_mem_info_malloc(sizeof(struct mblock_desc));
    // initialize the used list list
    gfx_used_mem_buffer_list->address = NULL;
    gfx_used_mem_buffer_list->size = 0;
    gfx_used_mem_buffer_list->next = NULL;
    gfx_used_mem_buffer_list->previous = NULL;
    gfx_used_mem_buffer_total_size = 0;

    GLOBAL_EXCEPTION_ENABLE();
}

void *app_graphics_mem_malloc(uint32_t size)
{
    struct mblock_desc *cur_node = NULL, *select_node = NULL;
    uint32_t alloc_address = NULL;
    uint32_t total_free_size = 0;

    // compute the aligned requested memory block size
    uint32_t totalsize = CO_ALIGN16_HI(size);

    // protect accesses to descriptors
    GLOBAL_EXCEPTION_DISABLE();

    // Step 1) find a smallest node for memory allocation
    {
        // point the current node to the free mem list(root node)
        cur_node = gfx_free_mem_buffer_list;
        ASSERT_ERR(cur_node != NULL);

        // go through free memory list and select the smallest match
        while (cur_node != NULL)
        {
            total_free_size += cur_node->size;

            // ensure the cur_node free memory block has enough space for allocation
            // the node info is seperated, so no need to reserve size for node info
            if (cur_node->size >= (totalsize))
            {
                // if a match was already select_node, check if this one is smaller
                if ((select_node == NULL) || (select_node->size > cur_node->size))
                {
                    select_node = cur_node;
                }
            }

            // move to next block
            cur_node = cur_node->next;
        }
    }

    ASSERT_INFO(select_node != NULL, size, total_free_size);

    // Step 2) update the free list and used list
    {
        // sublist completely reused
        if (select_node->size == totalsize)
        {
            // select_node is not the root node
            if (select_node->previous != NULL)
            {
                // remove the select_node from free list
                // point the previous node to the next node
                select_node->previous->next = select_node->next;
                if (select_node->next != NULL)
                {
                    select_node->next->previous = select_node->previous;
                }
            }
            else // select_node is the root node
            {
                // it is not allowed when the root node is fully reused
                // ASSERT_ERR(NULL != select_node->next);
                gfx_free_mem_buffer_list = select_node->next;
                gfx_free_mem_buffer_list->previous = NULL;
            }
            // no need to generate a new node
            alloc_address = select_node->address;
            // add the new node to the memory buffer used list
            struct mblock_desc *root_node = gfx_used_mem_buffer_list;
            select_node->next = root_node->next;
            select_node->previous = root_node;
            root_node->next = select_node;
        }
        else
        {
            // decrease the free_size in the select_node cur_node for allocation
            select_node->size -= totalsize;
            // compute the pointer to the beginning of the allocated space
            alloc_address = select_node->address;
            // first allocate the SRAM address for faster access speed
            select_node->address += totalsize;

            // generate an allocated node for memory free usage
            struct mblock_desc *alloc_node = (struct mblock_desc *)gfx_mem_info_malloc(sizeof(struct mblock_desc));
            alloc_node->address = alloc_address;
            alloc_node->size = totalsize;
            alloc_node->previous = NULL;
            alloc_node->next = NULL;
            // add the new node to the memory buffer used list
            struct mblock_desc *root_node = gfx_used_mem_buffer_list;
            alloc_node->next = root_node->next;
            alloc_node->previous = root_node;
            root_node->next = alloc_node;
        }

        // update the mem memory used size info and free size info
        gfx_used_mem_buffer_total_size = gfx_mem_buffer_pool_init_size + totalsize - total_free_size;
    }

    GMEM_DBG_PRINTF("mem_alloc_addr: %08X, size: %08X, end_addr: %08X, mem_used_size: %08X\n\r\n\r",
                   alloc_address, totalsize, (alloc_address + totalsize), gfx_used_mem_buffer_total_size);
    GFX_LOG_FLUSH(); // polling dump the log information

    // end of protection (as early as possible)
    GLOBAL_EXCEPTION_ENABLE();

    // support the psram partially refresh to save power
#if PSRAM_PARTIAL_REFR_ENABLE
    uint32_t psram_used_addr = app_graphics_mem_max_alloc_addr_get();
    app_graphics_ospi_pasr_update(psram_used_addr);
#endif

    return (void *)alloc_address;
}

static struct mblock_desc *gfx_mem_info_node_find(uint32_t address)
{
    struct mblock_desc *cur_node = gfx_used_mem_buffer_list;
    struct mblock_desc *prev_node = gfx_used_mem_buffer_list;
    while ((cur_node != NULL) && (address != cur_node->address))
    {
        prev_node = cur_node;
        cur_node = cur_node->next;
    }
    ASSERT_ERR(cur_node != NULL);
    // remove the current node from the gfx_used_mem_buffer_list
    // it will be totally freed or put in the gfx_free_mem_buffer_list
    prev_node->next = cur_node->next;
    if (cur_node->next != NULL)
    {
        cur_node->next->previous = prev_node;
    }
    cur_node->previous = NULL;
    cur_node->next = NULL;
    return cur_node;
}

uint32_t app_graphics_mem_max_alloc_addr_get(void)
{
    uint32_t max_address = 0;
    struct mblock_desc *cur_node = gfx_used_mem_buffer_list;
    while (cur_node != NULL)
    {
        uint32_t cur_end_address = cur_node->address + cur_node->size;
        if(cur_end_address > max_address)
        {
            max_address = cur_end_address;
        }
        cur_node = cur_node->next;
    }
    return max_address;
}

void app_graphics_mem_free(void *mem_ptr)
{
    // The address shall be ascending in the free memory list and not NULL
    ASSERT_INFO(mem_ptr != NULL, mem_ptr, 0);

    // record next_node and prev_node for convinient link operation
    struct mblock_desc *cur_node = NULL, *next_node = NULL, *prev_node = NULL;

    // protect accesses to descriptors
    GLOBAL_EXCEPTION_DISABLE();

    // Retrieve the node to be freed in used mem list
    struct mblock_desc *freed = gfx_mem_info_node_find((uint32_t)mem_ptr);
    uint32_t free_size = freed->size;

    // Try to add the freed node into the free mem list
    cur_node = gfx_free_mem_buffer_list;

    // sanity checks
    ASSERT_ERR(cur_node != NULL);

    while (cur_node != NULL)
    {
        // check if the freed block is right after the current block
        uint32_t node_tail_address = cur_node->address + cur_node->size;
        if (freed->address == node_tail_address)
        {
            // append the freed block to the current one
            cur_node->size += freed->size;

            // free the freed node because of node merge
            gfx_mem_info_free(freed);

            // check if this merge made the link between free blocks
            if ((NULL != cur_node->next) &&
                (cur_node->next->address == (cur_node->address + cur_node->size)))
            {
                struct mblock_desc *merge_bfreed = cur_node->next;
                // merge cur_node with next blocks when the merge generate a new merge
                next_node = cur_node->next;
                // add the size of the next cur_node to the current cur_node
                cur_node->size += next_node->size;
                // merge the next node with current node
                cur_node->next = next_node->next;
                if (next_node->next != NULL)
                {
                    next_node->next->previous = cur_node;
                }
                // free the freed node because of node merge
                gfx_mem_info_free(merge_bfreed);
            }
            break;
        }
        else if (freed->address < cur_node->address)
        {
            // if the current node is not the root node
            // note only the root node previous is NULL
            if (prev_node != NULL)
            {
                // put the freed node before cur_node
                prev_node->next = freed;
                freed->previous = prev_node;
            }
            else // if the current node is the root node
            {
                // update the root node and put the freed before cur_node
                gfx_free_mem_buffer_list = freed;
                freed->previous = NULL;
            }

            // check if the released cur_node is right before the free block
            if ((freed->address + freed->size) == cur_node->address)
            {
                // merge the two nodes
                freed->next = cur_node->next;
                if (cur_node->next != NULL)
                {
                    cur_node->next->previous = freed;
                }
                freed->size = cur_node->size + freed->size;
                // the current node shall be freed because of node merge
                gfx_mem_info_free(cur_node);
            }
            else
            {
                // move the current node next to the new freed node
                freed->next = cur_node;
                cur_node->previous = freed;
            }
            break;
        }
        // move to the next free block cur_node
        prev_node = cur_node;
        cur_node = cur_node->next;
    }

    // The freed node has the highest address and shall be in the last of the free list
    if (NULL == cur_node)
    {
        prev_node->next = freed;
        freed->next = NULL;
        freed->previous = prev_node;
    }

    // update the mem memory used and free info
    gfx_used_mem_buffer_total_size -= free_size;

    GMEM_DBG_PRINTF("mem_free_addr: %08X, size: %08X, end_addr: %08X, mem_used_size: %08X\n\r\n\r",
                   (uint32_t)mem_ptr, free_size, ((uint32_t)mem_ptr + free_size), gfx_used_mem_buffer_total_size);
    GFX_LOG_FLUSH(); // polling dump the log information

    // end of protection
    GLOBAL_EXCEPTION_ENABLE();

    // support the psram partially refresh to save power
#if PSRAM_PARTIAL_REFR_ENABLE
    uint32_t psram_used_addr = app_graphics_mem_max_alloc_addr_get();
    app_graphics_ospi_pasr_update(psram_used_addr);
#endif
}

uint32_t app_graphics_mem_used_size_get(void)
{
    return gfx_used_mem_buffer_total_size;
}

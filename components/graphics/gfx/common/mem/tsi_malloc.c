// -----------------------------------------------------------------------------
// Copyright (c) 2019 Think Silicon S.A.
// Think Silicon S.A. Confidential Proprietary
// -----------------------------------------------------------------------------
//     All Rights reserved - Unpublished -rights reserved under
//         the Copyright laws of the European Union
//
//  This file includes the Confidential information of Think Silicon S.A.
//  The receiver of this Confidential Information shall not disclose
//  it to any third party and shall protect its confidentiality by
//  using the same degree of care, but not less than a reasonable
//  degree of care, as the receiver uses to protect receiver's own
//  Confidential Information. The entire notice must be reproduced on all
//  authorised copies and copies may only be made to the extent permitted
//  by a licensing agreement from Think Silicon S.A..
//
//  The software is provided 'as is', without warranty of any kind, express or
//  implied, including but not limited to the warranties of merchantability,
//  fitness for a particular purpose and noninfringement. In no event shall
//  Think Silicon S.A. be liable for any claim, damages or other liability, whether
//  in an action of contract, tort or otherwise, arising from, out of or in
//  connection with the software or the use or other dealings in the software.
//
//
//                    Think Silicon S.A.
//                    http://www.think-silicon.com
//                    Patras Science Park
//                    Rion Achaias 26504
//                    Greece
// -----------------------------------------------------------------------------

#include "hal_gfx_sys_defs.h"
#include "tsi_malloc.h"
#include "tsi_malloc_intern.h"

static pool_t pools[MAX_MEM_POOLS] = {{0}};
#define HEAD (pools[pool].head_of_empty_list)

#define IF_POOL_IS_INVALID \
    if ( pool < 0 || pool >= MAX_MEM_POOLS || HEAD == NULL )

static inline int
pool_from_virt_addr(void *addr) {
    uintptr_t addr_u = (uintptr_t)addr;
    for (int pool = 0; pool < MAX_MEM_POOLS; ++pool) {
        if ( HEAD != NULL ) {
            if (addr_u >= pools[pool].base_virt && addr_u < pools[pool].end_virt) {
                return pool;
            }
        }
    }

    return -1;
}

//pointer to next cell
static inline cell_t *
next_cell(int pool, cell_t *cur_cell) {
    uintptr_t cur_cell_u = (uintptr_t)pools[pool].head_of_empty_list + cur_cell->next_offset;
    return (cell_t *)cur_cell_u;
}

//cell to pointer
static inline uintptr_t
c2p(const cell_t * const c) {
    return (uintptr_t)(c) + (uintptr_t)cell_t_size;
}

//pointer to cell
static inline cell_t *
p2c(void *p) {
    uintptr_t ptr = (uintptr_t)(p) - (uintptr_t)cell_t_size;
    return (cell_t *)ptr;
}

//c1 is adjascent to c2
static inline bool
isadj(const cell_t *c1, const cell_t *c2) {
    uintptr_t c1_p_ptr = c2p(c1);
    uintptr_t c1_next_c_ptr = c1_p_ptr + (uintptr_t)(int)(c1)->size;
    uintptr_t c2_c_ptr = (uintptr_t)c2;
    return c1_next_c_ptr == c2_c_ptr;
}

#if 0
void
print_pool(int pool) {
    cell_t *c = HEAD;

    printf("EMPTY LIST\n");
    printf("---\n");
    while ( 1 ) {

        printf("%p - %8d - next by offset: %08x, %s\n", (void *)c, c->size, (uintptr_t)c + c->next_offset, c->flags == FLAG_EMPTY ? "EMPTY" : "NOT EMPTY");


        if ( IS_LAST(c) ) {
            break;
        }

        c = next_cell(pool, c);
    }


    printf("---\n");
    printf("POOL ARENA\n");
    printf("---\n");


    // Get root cell
    c = (cell_t *)pools[pool].base_virt;
    // Skip root cell
    c = (cell_t *)((uintptr_t)c + cell_t_size);
    while ( 1 ) {
        if ( c->size == 0 ) {
            break;
        }

        printf("%p - %8d - next by size: %08x, %s\n", (void *)c, c->size, (uintptr_t)c + c->size, c->flags == FLAG_EMPTY ? "EMPTY" : "NOT EMPTY");

        c = (cell_t *)((uintptr_t)c + c->size + cell_t_size);
    }

}
#endif

int
tsi_malloc_init_pool(int pool,
                     void *base_virt,
                     uintptr_t base_phys,
                     int size,
                     int reset)
{
    if (pool >= MAX_MEM_POOLS || pool < 0) {
        return -1;
    }

    HEAD = (cell_t *)base_virt;
    pools[pool].end_virt = (uintptr_t)base_virt+(uintptr_t)(int)size;

    pools[pool].base_phys = base_phys;
    pools[pool].base_virt = (uintptr_t)base_virt;
    pools[pool].size      = size;

    if (reset != 0) {
        HEAD->size  = 0;
        HEAD->next_offset = cell_t_size;
        HEAD->flags = FLAG_EMPTY;

        cell_t *next = next_cell(pool, HEAD);
        next->size = size-2*cell_t_size;
        next->next_offset = 0;
        next->flags = FLAG_EMPTY;
    }

    return 0;
}

void *
tsi_malloc_pool(int pool, int size)
{
    IF_POOL_IS_INVALID {
        return NULL;
    }

    cell_t *c1, *c_prev;

    if ( size < cell_t_size ) {
        size = cell_t_size;
    }

    size = ALIGN(size);

    c_prev = HEAD;
    c1 = HEAD;

    //find a big enough cell (c1->next)
    while (c1->size < size) {
        if ( IS_LAST(c1) /*|| next_cell(pool, c1)->flags == FLAG_NONEMPTY*/) {
            //returned to head_of_empty_list without finding space
            return NULL;
        }

        //try next cell
        c_prev = c1;
        c1 = next_cell(pool, c1);
    }

    //c1 is a big enough empty cell
    if (c1->size > (cell_t_size + size)) {
        //c1 is bigger than needed
        //split it
        uintptr_t next_ptr = c2p(c1) + (uintptr_t)(int)size;
        cell_t *c_next = (cell_t *)(next_ptr);
        c_next->size  = c1->size - (size + cell_t_size);
        c_next->flags = FLAG_EMPTY;
        c_next->next_offset  = c1->next_offset;

        c_prev->next_offset = OFFSET(c_next);

        c1->size = size;
    } else {
        c_prev->next_offset = c1->next_offset;
    }

    c1->flags = FLAG_NONEMPTY;

    uintptr_t ptr = c2p(c1);
    return (void *)(char *)ptr;
}

void
tsi_free(void *ptr)
{
    if (ptr == NULL) {
        return;
    }

    int pool = pool_from_virt_addr(ptr);

    IF_POOL_IS_INVALID {
        return;
    }

    cell_t *c1, *c2, *c3;
    bool j1, j2;

    //find cell from pointer
    char *_ptr = ptr; //misra stuff...
    c2 = p2c(_ptr);
    if (c2->flags != FLAG_NONEMPTY) {
        return;
    }

    c1 = HEAD;

    c2->flags = FLAG_EMPTY;

    uintptr_t next_ptr = (uintptr_t)next_cell(pool, c1);
    uintptr_t c2_ptr = (uintptr_t)c2;

    while ( next_ptr < c2_ptr && !IS_LAST(c1)) { /* find insertion point */
        c1 = next_cell(pool, c1);
        next_ptr = (uintptr_t)next_cell(pool, c1);
    }

    c3 = next_cell(pool, c1);

    if (c1 == HEAD) {
        j1 = false;
    }
    else {
        j1 = isadj(c1,c2); /* c1 and c2 need to be joined */
    }

    j2 = isadj(c2,c3); /* c2 and c3 need to be joined */

    //make c1 point to c2
    c1->next_offset = OFFSET(c2);
    //make c2 point to c3
    c2->next_offset = OFFSET(c3);

    if (j1) {
        //join c1 with c2
        c1->size += cell_t_size + c2->size;
        c1->next_offset = OFFSET(c3);
        c2 = c1;
    }

    if (j2) {
        //join c2 with c3
        c2->size += cell_t_size + c3->size;;
        c2->next_offset = c3->next_offset;
    }
}

uintptr_t
tsi_virt2phys(void *addr) {
#ifdef USE_GLOBAL_MMU
    return (uintptr_t)addr;
#else
    int pool = pool_from_virt_addr(addr);
    IF_POOL_IS_INVALID {
        return 0U;
    }

    char *_addr = addr; //misra stuff...

    uintptr_t offset = OFFSET(_addr);
    uintptr_t phys_addr = offset+pools[pool].base_phys;
    return phys_addr;
#endif
}


#ifdef UNIT_TEST

//LCOV_EXCL_START

static int allocated_bytes[MAX_MEM_POOLS] = {0};

void *test_malloc(int pool, int size) {
    void *ptr = tsi_malloc_pool(pool, size);
    if (ptr) {
        cell_t *cell = p2c(ptr);
        allocated_bytes[pool] += cell->size;
    }

    return ptr;
}

void test_free(void *ptr) {

    int pool = pool_from_virt_addr(ptr);

    if ( pool >= 0 && pool < MAX_MEM_POOLS ) {
        cell_t *cell = p2c(ptr);
        // printf("#\t0x%08x:%d\n", cell->flags, cell->size);
        if (cell->flags == FLAG_NONEMPTY)
            allocated_bytes[pool] -= cell->size;
    }

    tsi_free(ptr);
}

int test_get_allocated_bytes(int pool) {
    if (pool >= 0 && pool < MAX_MEM_POOLS) {
        // printf("%d: %d bytes allocated\n", pool, allocated_bytes[pool]);
        return(allocated_bytes[pool]);
    }
    return 0;
}

int free_entire_pool(int pool) {
    if (pool < 0 || pool >= MAX_MEM_POOLS)
        return 0;

    cell_t *c1 = HEAD;

    while ( (int)OFFSET(c1) < pools[pool].size ) {
        test_free( (void *)c2p(c1) );
        c1 = (cell_t *)((char *)c1+c1->size+cell_t_size);
    }

    return validate_final_state(pool);
}

int validate_final_state(int pool) {
    if (pool < 0 || pool >= MAX_MEM_POOLS)
        return 0;

    cell_t  init_cell     = *HEAD;
    cell_t *init_cell_ptr =  HEAD;
    cell_t  init_cell_next = *next_cell(pool, HEAD);
    cell_t *init_cell_next_ptr = next_cell(pool, HEAD);

    int err = 0;

    if (init_cell_ptr != HEAD) {
        err |= (1U<<0);
      //  printf("Validation error: head_of_empty_list-mem pointer missmatch\n");
    }

    if (HEAD->flags != FLAG_EMPTY) {
        err |= (1U<<1);
      //  printf("Validation error: head_of_empty_list is not empty\n");
    }

    if (HEAD->size != init_cell.size || HEAD->next_offset != init_cell.next_offset) {
        err |= (1U<<2);
      //  printf("Validation error: head_of_empty_list/init_cell missmatch\n");
    }

    if (init_cell_next_ptr != next_cell(pool, HEAD) ) {
        err |= (1U<<3);
      //  printf("Validation error: HEAD-mem pointer missmatch\n");
    }

    if (init_cell_next_ptr->size != init_cell_next.size || next_cell(pool, HEAD)->next_offset != init_cell_next.next_offset) {
        err |= (1U<<4);
      //  printf("Validation error: head_of_empty_list/init_cell_next missmatch\n");
    }


    int alloc_bytes = test_get_allocated_bytes(pool);
    // printf("%d bytes allocated\n", alloc_bytes);

    if (alloc_bytes != 0) {
        err |= (1U<<5);
      //  printf("This should be 0\n");
    }
    return err;
}

//LCOV_EXCL_STOP

#endif


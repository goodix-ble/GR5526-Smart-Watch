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

// #include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdint.h>
#include "hal_gfx_devmem.h"

int hal_gfx_devmem_open( void ) {
    /* Open /dev/mem file for mmap*/
    int fd = open ("/dev/mem", O_RDWR);
    if (fd < 0) {
        // printf("can't open /dev/mem");
        return -1;
    }

    return fd;
}


int hal_gfx_devmem_close( int fd ) {
    return close(fd);
}

int hal_gfx_devmem_mmap( void **ptr, uintptr_t mem_addr, unsigned size, const char *comment ) {
    int fd = hal_gfx_devmem_open();

    if ( fd < 0 ) {
        return -1;
    }

    unsigned flags = (unsigned)PROT_READ|(unsigned)PROT_WRITE;

    *ptr = mmap(NULL, (size_t)size, (int)flags, MAP_SHARED, fd, (off_t)mem_addr);

   // printf("mmapping phys addr 0x%08x to virt %p for %s\n", mem_addr, *ptr, comment);

    (void)hal_gfx_devmem_close(fd);
    return 0;
}


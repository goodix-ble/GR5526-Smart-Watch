#ifndef __HAL_GFX_SYS_DEFS_H__
#define __HAL_GFX_SYS_DEFS_H__

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define TLS_VAR

#define HAL_GFX_BASEADDR               0xA3FF0000            /* the GPU registers memory base address */

#define HAL_GDC_BASEADDR             0xA3FF4000            /* the DC registers memory base address */

#define USE_TSI_MALLOC

#endif /*__HAL_GFX_SYS_DEFS_H__*/

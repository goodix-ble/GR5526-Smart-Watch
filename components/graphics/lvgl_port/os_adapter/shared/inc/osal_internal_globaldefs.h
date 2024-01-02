#ifndef __OSAL_INTERNAL_GLOBALDEFS_H__
#define __OSAL_INTERNAL_GLOBALDEFS_H__

#include "osal_config.h"
#include "common_types.h"
#include "osal_error.h"
#include "osal_macros.h"

#define OSAL_CHECK_POINTER(ptr) ARGCHECK((ptr) != NULL, OSAL_INVALID_POINTER)

#define OSAL_CHECK_SIZE(val) ARGCHECK((val) > 0 && (val) < (UINT32_MAX / 2), OSAL_ERR_INVALID_SIZE)

#define OSAL_CHECK_STRING(str, maxlen, errcode) \
    do                                        \
    {                                         \
        OSAL_CHECK_POINTER(str);                \
        LENGTHCHECK(str, maxlen, errcode);    \
    } while (0)


#endif // __OSAL_INTERNAL_GLOBALDEFS_H__

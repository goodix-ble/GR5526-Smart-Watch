#ifndef __OSAL_MACROS_H__
#define __OSAL_MACROS_H__

//#include <stdlib.h>
#include <string.h>

/**
 * @brief Generic argument checking macro for non-critical values
 *
 * This macro checks a conditional that is expected to be true, and return a value
 * if it evaluates false.
 *
 * ARGCHECK can be used to check for out of range or other invalid argument conditions
 * which may (validly) occur at runtime and do not necessarily indicate bugs in the
 * application.
 *
 * These argument checks are NOT considered fatal errors.  The application
 * continues to run normally.  This does not report the error on the console.
 *
 * As such, ARGCHECK actions are always compiled in - not selectable at compile-time.
 *
 * @sa BUGCHECK for checking critical values that indicate bugs
 */
#define ARGCHECK(cond, errcode) \
    if (!(cond))                \
    {                           \
        return errcode;         \
    }

/**
 * @brief String length limit check macro
 *
 * This macro is a specialized version of ARGCHECK that confirms a string will fit
 * into a buffer of the specified length, and return an error code if it will not.
 *
 * @note this uses ARGCHECK, thus treating a string too long as a normal runtime
 * (i.e. non-bug) error condition with a typical error return to the caller.
 */
#define LENGTHCHECK(str, len, errcode) ARGCHECK(memchr(str, '\0', len), errcode)

#define OSAL_IS_IN_ISR() (__get_IPSR() != 0U)


#endif // __OSAL_MACROS_H__

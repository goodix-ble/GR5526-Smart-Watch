#ifndef __DTM_FCC_TEST_H__
#define __DTM_FCC_TEST_H__

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>

/**
 *****************************************************************************************
 * @brief Initialize the FCC test environment.
 *
 * @note This function is used to prepare the FCC test environment.
 *****************************************************************************************
 */
void fcc_test_init(void);

/**
 *****************************************************************************************
 * @brief Save system context.
 *
 * @note This function is used to prepare the DTM test environment.
 *****************************************************************************************
 */
void dtm_test_init(void);

#endif

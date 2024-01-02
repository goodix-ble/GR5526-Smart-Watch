/**
 ****************************************************************************************
 *
 * @file scatter_common.h
 *
 * @brief decalare the symbols in scatter_common.sct.
 *
 *
 ****************************************************************************************
 */

#ifndef __BLE_CFG_H__
#define __BLE_CFG_H__

#include <stdint.h>
#include "custom_config.h"

/* ************************************************************************
 * developer must define CFG_MAX_CONNECTIONS in custom_config.h .
 * Max value for GR551X: 10 which must be same with CFG_CON
 * in ROM's configs.opt
 */
#ifndef CFG_MAX_CONNECTIONS
    #error "CFG_MAX_CONNECTIONS is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_CONNECTIONS <= 10)
    #define USER_MAX_CONNECTIONS CFG_MAX_CONNECTIONS
#else
    #define USER_MAX_CONNECTIONS (1)
#endif

#ifndef CFG_MAX_ADVS
    #error "CFG_MAX_ADVS is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_ADVS <= 5)
    #define USER_MAX_ADVS CFG_MAX_ADVS
#else
    #define USER_MAX_ADVS (1)
#endif

#ifndef CFG_MAX_PER_ADVS
    #error "CFG_MAX_PER_ADVS is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_PER_ADVS <= 5)
    #define USER_MAX_PER_ADVS CFG_MAX_PER_ADVS
#else
    #define USER_MAX_PER_ADVS (0)
#endif

#if ((USER_MAX_ADVS+USER_MAX_PER_ADVS) > 5)
    #error "The number of BLE Legacy/Extended/Periodic Advertising exceeds the limit."
#endif

#ifndef CFG_MAX_SCAN
    #error "CFG_MAX_SCAN is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_SCAN <= 1)
    #define USER_MAX_SCAN CFG_MAX_SCAN
#else
    #define USER_MAX_SCAN (1)
#endif

#ifndef CFG_MAX_SYNCS
    #error "CFG_MAX_SYNCS is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_SYNCS <= 5)
    #define USER_MAX_SYNCS CFG_MAX_SYNCS
#else
    #define USER_MAX_SYNCS (0)
#endif

#if (CFG_MAX_EATT_CHANNELS <= 10)
    #define USER_MAX_EATT_CHANNELS CFG_MAX_EATT_CHANNELS
#else
    #define USER_MAX_EATT_CHANNELS (1)
#endif

#if ((USER_MAX_CONNECTIONS+USER_MAX_ADVS+2*USER_MAX_PER_ADVS+USER_MAX_SCAN+USER_MAX_SYNCS) > 12)
    #error "The number of BLE Activities exceeds the limit."
#endif

#ifndef CFG_MAX_BOND_DEVS
    #error "CFG_MAX_BOND_DEVS is not defined in app's custom_config.h ."
#endif

#define USER_MAX_BOND_DEVS CFG_MAX_BOND_DEVS

#ifndef CFG_MAX_PRFS
    #error "CFG_MAX_PRFS is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_PRFS <= 64)
    #define USER_MAX_PRFS CFG_MAX_PRFS
#else
    #define USER_MAX_PRFS (1)
#endif

#if (CFG_RANGING_ALLOCATE)
// iq sample size for master(20 channel) 1922 * 20 * 2
#define RANGING_SAMPLE_SIZE       (81920UL)
#else
#define RANGING_SAMPLE_SIZE       (0)
#endif

#if (CFG_ISO_SUPPORT)
//ISO MAX (TX+RX) buffer NUMBER is 30
#define ISO_DATA_PATH_BUFFER    (251 * 30)
#else
#define ISO_DATA_PATH_BUFFER    (0)
#endif

/* The macro is used to compute size of the heap block in bytes. */
#define MEM_HEAP_HEADER                     (16 / sizeof(uint32_t))
#define MEM_CALC_HEAP_LEN(len)              ((((len) + (sizeof(uint32_t) - 1)) / sizeof(uint32_t)) + MEM_HEAP_HEADER)
#define MEM_CALC_HEAP_LEN_IN_BYTES(len)     (MEM_CALC_HEAP_LEN(len) * sizeof(uint32_t))

#if (CFG_CONTROLLER_ONLY == 0)
#define ENV_HEAP_SIZE             MEM_CALC_HEAP_LEN_IN_BYTES(412 * USER_MAX_CONNECTIONS + 172 * USER_MAX_EATT_CHANNELS \
                                                           + 426 * (USER_MAX_CONNECTIONS+USER_MAX_ADVS+2*USER_MAX_PER_ADVS+USER_MAX_SCAN+USER_MAX_SYNCS) \
                                                           + 600  \
                                                           + RANGING_SAMPLE_SIZE \
                                                           + ISO_DATA_PATH_BUFFER)
#else
#define ENV_HEAP_SIZE             MEM_CALC_HEAP_LEN_IN_BYTES(230 * (USER_MAX_CONNECTIONS+USER_MAX_ADVS+2*USER_MAX_PER_ADVS+USER_MAX_SCAN+USER_MAX_SYNCS) \
                                                           + 600 \
                                                           + ISO_DATA_PATH_BUFFER)
#endif


/* The size of heap for ATT database depends on the number of attributes in
 * profiles. The value can be tuned based on supported profiles. */
#if (CFG_CONTROLLER_ONLY == 0)
#define ATT_DB_HEAP_SIZE          MEM_CALC_HEAP_LEN_IN_BYTES(1024)
#else
#define ATT_DB_HEAP_SIZE          MEM_CALC_HEAP_LEN_IN_BYTES(100)
#endif

#if (CFG_CONTROLLER_ONLY == 0)
#define KE_MSG_HEAP_SIZE          MEM_CALC_HEAP_LEN_IN_BYTES(1650 * (USER_MAX_SCAN+USER_MAX_SYNCS) \
                                                           + 112 *(USER_MAX_CONNECTIONS+USER_MAX_ADVS+2*USER_MAX_PER_ADVS) \
                                                           + 408 *(USER_MAX_CONNECTIONS+USER_MAX_ADVS+2*USER_MAX_PER_ADVS+USER_MAX_SCAN+USER_MAX_SYNCS) \
                                                           + 3072)
#else
#define KE_MSG_HEAP_SIZE          MEM_CALC_HEAP_LEN_IN_BYTES(1650 * (USER_MAX_SCAN+USER_MAX_SYNCS) \
                                                           + 420 * (USER_MAX_CONNECTIONS+USER_MAX_ADVS+2*USER_MAX_PER_ADVS+USER_MAX_SCAN+USER_MAX_SYNCS) \
                                                           + 1020 * (USER_MAX_SCAN+USER_MAX_SYNCS) \
                                                           + 1259)
#endif
/* The size of non-retention heap is customized. This heap will used by BLE
 * stack only when other three heaps are full. */
#define NON_RET_HEAP_SIZE         MEM_CALC_HEAP_LEN_IN_BYTES(328 * 2)

#if (CFG_CONTROLLER_ONLY == 0)
#define PRF_BUF_SIZE              (96*USER_MAX_PRFS + 4)
#define BOND_BUF_SIZE             (48*USER_MAX_BOND_DEVS + 4)
#define CONN_BUF_SIZE             (372*USER_MAX_CONNECTIONS + 4)
#else
#define PRF_BUF_SIZE              (4)
#define BOND_BUF_SIZE             (4)
#define CONN_BUF_SIZE             (4)
#endif



#define STACK_HEAP_INIT(heaps_table)    uint8_t prf_buf[PRF_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t bond_buf[BOND_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t conn_buf[CONN_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t env_heap_buf[ENV_HEAP_SIZE] __attribute__((aligned (32)))= {0};\
                                        uint8_t att_db_heap_buf[ATT_DB_HEAP_SIZE] __attribute__((aligned (32)))= {0};\
                                        uint8_t ke_msg_heap_buf[KE_MSG_HEAP_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t non_ret_heap_buf[NON_RET_HEAP_SIZE]__attribute__((aligned (32))) = {0};\
stack_heaps_table_t heaps_table = { (uint32_t *)env_heap_buf,\
                                        (uint32_t *)att_db_heap_buf,\
                                        (uint32_t *)ke_msg_heap_buf,\
                                        (uint32_t *)non_ret_heap_buf,\
                                        ENV_HEAP_SIZE,\
                                        ATT_DB_HEAP_SIZE,\
                                        KE_MSG_HEAP_SIZE,\
                                        NON_RET_HEAP_SIZE,\
                                        (uint8_t *)prf_buf,\
                                        PRF_BUF_SIZE,\
                                        (uint8_t *)bond_buf,\
                                        BOND_BUF_SIZE,\
                                        (uint8_t *)conn_buf,\
                                        CONN_BUF_SIZE}


#endif // __SCATTER_COMMON_H__

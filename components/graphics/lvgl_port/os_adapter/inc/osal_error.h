#ifndef __OSAL_ERROR_H__
#define __OSAL_ERROR_H__

#include "common_types.h"

#define OSAL_ERROR_NAME_LENGTH (35)

#define OSAL_SUCCESS                     (0)   /**< @brief Successful execution */
#define OSAL_ERROR                       (1)  /**< @brief Failed execution */
#define OSAL_INVALID_POINTER             (2)  /**< @brief Invalid pointer */
#define OSAL_ERROR_ADDRESS_MISALIGNED    (3)  /**< @brief Address misalignment */
#define OSAL_ERROR_TIMEOUT               (4)  /**< @brief Error timeout */
#define OSAL_INVALID_INT_NUM             (5)  /**< @brief Invalid Interrupt number */
#define OSAL_SEM_FAILURE                 (6)  /**< @brief Semaphore failure */
#define OSAL_SEM_TIMEOUT                 (7)  /**< @brief Semaphore timeout */
#define OSAL_QUEUE_EMPTY                 (8)  /**< @brief Queue empty */
#define OSAL_QUEUE_FULL                  (9)  /**< @brief Queue full */
#define OSAL_QUEUE_TIMEOUT               (10) /**< @brief Queue timeout */
#define OSAL_QUEUE_INVALID_SIZE          (11) /**< @brief Queue invalid size */
#define OSAL_QUEUE_ID_ERROR              (12) /**< @brief Queue ID error */
#define OSAL_ERR_NAME_TOO_LONG           (13) /**< @brief name length including null terminator greater than #OSAL_MAX_API_NAME */
#define OSAL_ERR_NO_FREE_IDS             (14) /**< @brief No free IDs */
#define OSAL_ERR_NAME_TAKEN              (15) /**< @brief Name taken */
#define OSAL_ERR_INVALID_ID              (16) /**< @brief Invalid ID */
#define OSAL_ERR_NAME_NOT_FOUND          (17) /**< @brief Name not found */
#define OSAL_ERR_SEM_NOT_FULL            (18) /**< @brief Semaphore not full */
#define OSAL_ERR_INVALID_PRIORITY        (19) /**< @brief Invalid priority */
#define OSAL_INVALID_SEM_VALUE           (20) /**< @brief Invalid semaphore value */
#define OSAL_ERR_FILE                    (27) /**< @brief File error */
#define OSAL_ERR_NOT_IMPLEMENTED         (28) /**< @brief Not implemented */
#define OSAL_TIMER_ERR_INVALID_ARGS      (29) /**< @brief Timer invalid arguments */
#define OSAL_TIMER_ERR_TIMER_ID          (30) /**< @brief Timer ID error */
#define OSAL_TIMER_ERR_UNAVAILABLE       (31) /**< @brief Timer unavailable */
#define OSAL_TIMER_ERR_INTERNAL          (32) /**< @brief Timer internal error */
#define OSAL_ERR_OBJECT_IN_USE           (33) /**< @brief Object in use */
#define OSAL_ERR_BAD_ADDRESS             (34) /**< @brief Bad address */
#define OSAL_ERR_INCORRECT_OBJ_STATE     (35) /**< @brief Incorrect object state */
#define OSAL_ERR_INCORRECT_OBJ_TYPE      (36) /**< @brief Incorrect object type */
#define OSAL_ERR_STREAM_DISCONNECTED     (37) /**< @brief Stream disconnected */
#define OSAL_ERR_OPERATION_NOT_SUPPORTED (38) /**< @brief Requested operation not support on supplied object(s) */
#define OSAL_ERR_INVALID_SIZE            (40) /**< @brief Invalid Size */
#define OSAL_ERR_OUTPUT_TOO_LARGE        (41) /**< @brief Size of output exceeds limit  */
#define OSAL_ERR_INVALID_ARGUMENT        (42) /**< @brief Invalid argument value (other than ID or size) */
#define OSAL_ERR_IN_ISR                  (43) /**< @brief Currently in interrupt context */

#endif // __OSAL_ERROR_H__

#ifndef __DRV_ADAPTER_TOUCH_PAD_H__
#define __DRV_ADAPTER_TOUCH_PAD_H__

#include "stdint.h"
#include "stdbool.h"

/** @addtogroup TOUCHPAD_DEV_STRUCTURES Structures
  * @{
  */

/**
  * @brief NorFlash Driver Device Structures
  */
typedef struct _touchpad_drv_t {

    uint32_t id;                                                                                                /**< Record the device id. */

    bool (* touchpad_drv_init)(struct _touchpad_drv_t * dev);                                                   /**< function pointer to device init */

    bool (* touchpad_drv_deinit)(struct _touchpad_drv_t * dev);                                                 /**< function pointer to device deinit */

    bool (* touchpad_drv_read_pointer)(struct _touchpad_drv_t * dev, int16_t * x, int16_t * y);                 /**< function pointer to read pointer cordinate */

    bool (* touchpad_drv_sleep)(struct _touchpad_drv_t * dev);                                                  /**< function pointer to device sleep  */

    bool (* touchpad_drv_wakeup)(struct _touchpad_drv_t * dev);                                                 /**< function pointer to device wakeup  */

} touchpad_drv_t;

/** @} */

/**
 ****************************************************************************************
 * @brief  Register the device data & operation functions
 *
 * @param[in]  index : norf device index. 0 in default
 * @param[in]  dev   : pointer to the norf device
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_touchpad_reg(touchpad_drv_t * dev) ;

/**
 ****************************************************************************************
 * @brief  init the touchpad device
 *
 * @param[in]  None
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_touchpad_init(void);

/**
 ****************************************************************************************
 * @brief  deinit the touchpad device
 *
 * @param[in]  None
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_touchpad_deinit(void);

/**
 ****************************************************************************************
 * @brief  write data into the norf device, user should erase corresponding region firstly
 *
 * @param[in]  x : x-coordinate for touching
 * @param[in]  y : y-coordinate for touching
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_touchpad_read_pointer(int16_t * x, int16_t *y);

/**
 ****************************************************************************************
 * @brief  make the touchpad device to sleep
 *
 * @param[in]  None
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_touchpad_sleep(void);

/**
 ****************************************************************************************
 * @brief  wakeup the touchpad device
 *
 * @param[in]  None
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_touchpad_wakeup(void);

#endif /* __DRV_ADAPTER_TOUCH_PAD_H__ */

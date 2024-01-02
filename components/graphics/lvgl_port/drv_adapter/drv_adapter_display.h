#ifndef __DRV_ADAPTER_DISPLAY_H__
#define __DRV_ADAPTER_DISPLAY_H__

#include "stdint.h"
#include "stdbool.h"

#define DISPLAY_DEV_MAX         2u          /* max display device to support */

/** @addtogroup DISP_DEV_STRUCTURES Structures
  * @{
  */

/**
  * @brief Display Driver Device Structures
  */
typedef struct _disp_drv_t {

    uint32_t idx;                                                                                                    /**< Record the instance index. */

    uint32_t dev_id;                                                                                                 /**< Record the device id. */

    void * user_data;                                                                                                /**< Record the user data */

    void (* disp_drv_init)(struct _disp_drv_t * dev);                                                                /**< function pointer to device init */

    void (* disp_drv_deinit)(struct _disp_drv_t * dev);                                                              /**< function pointer to device deinit */

    void (* disp_drv_set_show_area)(struct _disp_drv_t * dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);   /**< function pointer to set show area */

    void (* disp_drv_wait_to_flush)(struct _disp_drv_t * dev);                                                       /**< function pointer to wait flush done */

    void (* disp_drv_flush)(struct _disp_drv_t * dev, void * buff, uint32_t buff_format, uint16_t w, uint16_t h);    /**< function pointer to flush framebuffer data */

    void (* disp_drv_wait_te)(struct _disp_drv_t * dev);                                                             /**< function pointer to wait tearing-effect signal */

    void (* disp_drv_on)(struct _disp_drv_t * dev, bool on);                                                         /**< function pointer to set screen on/off */

    void (* disp_drv_set_brightness)(struct _disp_drv_t * dev, uint32_t percent);                                    /**< function pointer to set brightness */

    void (* disp_drv_sleep)(struct _disp_drv_t * dev);                                                               /**< function pointer to set sleep */

    void (* disp_drv_wakeup)(struct _disp_drv_t * dev);                                                              /**< function pointer to wakeup davice */

} disp_drv_t;
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
bool drv_adapter_disp_reg(uint32_t index, disp_drv_t * dev) ;

/**
 ****************************************************************************************
 * @brief  init the display device
 *
 * @param[in]  None
 *
 * @return None
 ****************************************************************************************
 */
void drv_adapter_disp_init(void);

/**
 ****************************************************************************************
 * @brief  deinit the display device
 *
 * @param[in]  None
 *
 * @return None
 ****************************************************************************************
 */
void drv_adapter_disp_deinit(void);

/**
 ****************************************************************************************
 * @brief  Set the show area for the display device
 *
 * @param[in]  x1 : left-up x coordinate
 * @param[in]  y1 : left-up y coordinate
 * @param[in]  x2 : right-down x coordinate
 * @param[in]  y2 : right-down y coordinate
 *
 * @return None
 ****************************************************************************************
 */
void drv_adapter_disp_set_show_area(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

/**
 ****************************************************************************************
 * @brief  wait last flush to finish, usually wait in semaphore mode
 *
 * @param[in]  None
 *
 * @return None
 ****************************************************************************************
 */
void drv_adapter_disp_wait_to_flush(void);

/**
 ****************************************************************************************
 * @brief  Flush the framebuffer data to display device
 *
 * @param[in]  buff : pointer to framebuffer to flush
 * @param[in]  buff_format : @ref graphics_dc_data_format_e
 * @param[in]  w : pixel width of the framebuffer
 * @param[in]  h : pixel height of the framebuffer
 *
 * @return None
 ****************************************************************************************
 */
void drv_adapter_disp_flush(void * buff, uint32_t buff_format, uint16_t w, uint16_t h);

/**
 ****************************************************************************************
 * @brief  wait the te signal
 *
 * @param[in]  None
 *
 * @return None
 ****************************************************************************************
 */
void drv_adapter_disp_wait_te(void);


/**
 ****************************************************************************************
 * @brief  set the display device to display on/off
 *
 * @param[in]  is_on - true: set display on; false: set display off
 *
 * @return None
 ****************************************************************************************
 */
void drv_adapter_disp_on(bool is_on);


/**
 ****************************************************************************************
 * @brief  set the brightness of display device
 *
 * @param[in]  percent - valid value - [0, 100]
 *
 * @return None
 ****************************************************************************************
 */
void drv_adapter_disp_set_brightness(uint32_t percent);


/**
 ****************************************************************************************
 * @brief  make the display device to sleep
 *
 * @param[in]  None
 *
 * @return None
 ****************************************************************************************
 */
void drv_adapter_disp_sleep(void);

/**
 ****************************************************************************************
 * @brief  wakeup the display device
 *
 * @param[in]  None
 *
 * @return None
 ****************************************************************************************
 */
void drv_adapter_disp_wakeup(void);

#endif /* __DRV_ADAPTER_DISPLAY_H__ */

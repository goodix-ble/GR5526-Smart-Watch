#ifndef __DRV_ADAPTER_NOR_FLASH_H__
#define __DRV_ADAPTER_NOR_FLASH_H__

#include "stdint.h"
#include "stdbool.h"
//#include "app_qspi.h"

#define ADAPTER_NORFLASH_DEV_MAX            2u          /**< max display device to support */

/** @addtogroup NorFlash erase mode defines
  * @{
  */
#define ADAPTER_NORFFLASH_ERASE_PAGE         0u          /**< Erase the Norf dev by Page   */
#define ADAPTER_NORFFLASH_ERASE_SECTOR       1u          /**< Erase the Norf dev by Sector */
#define ADAPTER_NORFFLASH_ERASE_BLOCK        2u          /**< Erase the Norf dev by Block  */
#define ADAPTER_NORFFLASH_ERASE_CHIP         3u          /**< Erase the Norf dev by Chip   */
/** @} */

/** @addtogroup NORFLASH_DEV_STRUCTURES Structures
  * @{
  */

/**
  * @brief NorFlash Driver Device Structures
  */
typedef struct _norflash_drv_t {

    uint32_t idx;                                                                                               /**< Record the instance index. */

    uint32_t sector_size;                                                                                       /**< Specify the Sector size. */

    void * sector_buff;                                                                                         /**< Specify the pointer to Sector buffer. */

    void * user_data;                                                                                           /**< Specify the pointer to user data */

    bool (* norflash_drv_init)(struct _norflash_drv_t * dev);                                                   /**< function pointer to device init */

    bool (* norflash_drv_deinit)(struct _norflash_drv_t * dev);                                                 /**< function pointer to device deinit */

    bool (* norflash_drv_write)(struct _norflash_drv_t * dev, uint32_t addr, uint8_t * buff, uint32_t len);     /**< function pointer to device write */

    bool (* norflash_drv_read)(struct _norflash_drv_t * dev, uint32_t addr, uint8_t * buff, uint32_t len);      /**< function pointer to device read */

    bool (* norflash_drv_update)(struct _norflash_drv_t * dev, uint32_t addr, uint8_t * buff, uint32_t len);    /**< function pointer to device update */

    bool (* norflash_drv_erase)(struct _norflash_drv_t * dev, uint32_t addr, uint32_t erase_mode);              /**< function pointer to device erase */

    bool (* norflash_drv_set_mmap_mode)(struct _norflash_drv_t * dev, bool mmap);                               /**< function pointer to device mmap mode */

    bool (* norflash_drv_sleep)(struct _norflash_drv_t * dev);                                                  /**< function pointer to device sleep  */

    bool (* norflash_drv_wakeup)(struct _norflash_drv_t * dev);                                                 /**< function pointer to device wakeup  */

} norflash_drv_t;
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
bool drv_adapter_norflash_reg(uint32_t index, norflash_drv_t * dev) ;

/**
 ****************************************************************************************
 * @brief  set the sector info, the sector is used as cache when updating the device space
 *
 * @param[in]  sec_buff : pointer to sector buffer
 * @param[in]  sec_size : size of sector buffer, in bytes
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_norflash_set_sector_buff(uint32_t index, void * sec_buff, uint32_t sec_size) ;

/**
 ****************************************************************************************
 * @brief  init the norf device
 *
 * @param[in]  None
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_norflash_init(void);

/**
 ****************************************************************************************
 * @brief  deinit the norf device
 *
 * @param[in]  None
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_norflash_deinit(void);

/**
 ****************************************************************************************
 * @brief  write data into the norf device, user should erase corresponding region firstly
 *
 * @param[in]  addr : start address to write, should be aligned by data width
 * @param[in]  buff : buffer pointer to write, should be aligned by data width
 * @param[in]  len  : data length to write
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_norflash_write(uint32_t addr, uint8_t * buff, uint32_t len);

/**
 ****************************************************************************************
 * @brief  read data from the norf device
 *
 * @param[in]  addr : start address to read, should be aligned by data width
 * @param[in]  buff : buffer pointer to read, should be aligned by data width
 * @param[in]  len  : data length to read
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_norflash_read(uint32_t addr, uint8_t * buff, uint32_t len);

/**
 ****************************************************************************************
 * @brief  update data into the norf device,
 *         the function can help to backup & rewrite the un-expected modified data
 *
 * @param[in]  addr : start address to update, should be aligned by data width
 * @param[in]  buff : buffer pointer to update, should be aligned by data width
 * @param[in]  len  : data length to update
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_norflash_update(uint32_t addr, uint8_t * buff, uint32_t len);

/**
 ****************************************************************************************
 * @brief  erase the norf device with specified mode
 *
 * @param[in]  addr : start address to erase, should be aligned by mode
 * @param[in]  mode : @ref ADAPTER_NORFFLASH_ERASE_PAGE
 *                    @ref ADAPTER_NORFFLASH_ERASE_SECTOR
 *                    @ref ADAPTER_NORFFLASH_ERASE_BLOCK
 *                    @ref ADAPTER_NORFFLASH_ERASE_CHIP
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_norflash_erase(uint32_t addr, uint32_t mode);

/**
 ****************************************************************************************
 * @brief  set the access mode for norf device
 *
 * @param[in]  mmap : true - mmap mode; false - register mode
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_norflash_set_mmap_mode(bool mmap);

/**
 ****************************************************************************************
 * @brief  make the norf device to sleep
 *
 * @param[in]  None
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_norflash_sleep(void);

/**
 ****************************************************************************************
 * @brief  wakeup the norf device
 *
 * @param[in]  None
 *
 * @return true/false
 ****************************************************************************************
 */
bool drv_adapter_norflash_wakeup(void);

#endif /* __DRV_ADAPTER_NOR_FLASH_H__ */

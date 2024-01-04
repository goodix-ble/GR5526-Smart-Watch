#ifndef __LV_HAL_MEM_H__
#define __LV_HAL_MEM_H__

#include "gr55xx.h"
#include "lvgl.h"

#define LV_PORT_MEM_NONE            0u
#define LV_PORT_MEM_SET             1u
#define LV_PORT_MEM_CPY             2u
#define LV_PORT_MEM_BLIT            3u

typedef uint8_t lv_hal_mem_op_e;

#define LV_DMA_XFER_WIDTH_BYTE      0u
#define LV_DMA_XFER_WIDTH_HALFWORD  1u
#define LV_DMA_XFER_WIDTH_WORD      2u

typedef uint8_t lv_dma_xfer_width_e;

#define LVGL_PORT_DMA_INSTANCE                  DMA0
#define LVGL_PORT_DMA_CHANNEL                   DMA_Channel0
#define LVGL_PORT_DMA_XFER_BLOCK_SIZE           4000u             /* by beat */
#define LVGL_PORT_MEMSET_SMALL_SIZE             1024u             /* by byte */
#define LVGL_PORT_MEMCPY_SMALL_SIZE             512               /* by byte */


/**
  * @brief Blit Image structure definition
  */
typedef struct {
    uint32_t src_img_address;           /**< start address for source */
    uint32_t src_img_w;                 /**< pixel width of source image */
    uint32_t src_img_h;                 /**< pixel height of source image */
    uint32_t src_img_x;                 /**< x-coordinate of source image, left-top point is Coordinate origin */
    uint32_t src_img_x_delta;           /**< blit image width in pixel, do not multiple pixel depth */
    uint32_t src_img_y;                 /**< y-coordinate of source image, left-top point is Coordinate origin */
    uint32_t src_img_y_delta;           /**< blit image height in pixel, do not multiple pexel depth */
    uint32_t dst_buff_address;          /**< destination buffer address */
    uint32_t dst_buff_width;            /**< pixel width of destination buffer */
    uint32_t dst_buff_height;           /**< pixel height of destination buffer */
    uint32_t dst_buff_x;                /**< x-coordinate of destination buffer, left-top point is Coordinate origin */
    uint32_t dst_buff_y;                /**< y-coordinate of destination buffer, left-top point is Coordinate origin */
    uint32_t pixel_depth;               /**< pixel depth in byte */
} mem_rect_blit_config_t;

bool lv_hal_mem_init(void) ;

/* not thread-safe now ! */
void lv_hal_memset(void * dst, uint8_t v, uint32_t len);

/* not thread-safe now ! */
void * lv_hal_memcpy(void * dst, void * src, uint32_t len);


bool lv_hal_mem_rect_blit(mem_rect_blit_config_t * p_blit_config, lv_dma_xfer_width_e xfer_width);

#endif /* __LV_HAL_MEM_H__ */

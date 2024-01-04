#include "lv_hal_mem.h"
#include "app_dma.h"
#include "app_graphics_ospi.h"

#define LV_PORT_ALIGN_MASK          0x03

typedef struct {
    dma_id_t                dma_id;
    volatile bool           is_dma_err;
    volatile bool           is_dma_done;
    volatile bool           is_busy;
    volatile lv_dma_xfer_width_e    dma_xfer_width;
    volatile lv_hal_mem_op_e        mem_op;
    volatile uint32_t       xfered_size;         // in byte
    volatile uint32_t       once_xfer_max_size;  // max size in one xfer (in byte)
    volatile uint32_t       total_size;          // in byte

    volatile uint32_t       src_address;         // in byte
    volatile uint32_t       dst_address;         // in byte

    volatile uint32_t       sg_line_length;
    volatile uint32_t       sg_src_delta_width;
    volatile uint32_t       sg_dst_delta_width;
    dma_sg_llp_config_t     sgc;
} lv_hal_mem_env_t;

static lv_hal_mem_env_t s_mem_env = { -1, false, false, false, LV_DMA_XFER_WIDTH_BYTE, LV_PORT_MEM_NONE, 0, 0, 0, 0, 0, 0, 0, 0};

static void _config_dma_xfer_inc_mode(dma_regs_t * p_instance, dma_channel_t channel, uint32_t src_inc_mode, uint32_t dst_inc_mode);
static void _config_dma_xfer_width(dma_regs_t * p_instance, dma_channel_t channel, uint32_t src_width, uint32_t dst_width);
static void _config_dma_xfer_mszie(dma_regs_t * p_instance, dma_channel_t channel, uint32_t src_mszie, uint32_t dst_msize);
static void _lv_hal_dma_handler(app_dma_evt_type_t type);

/**************************************************************************************************
 *               Porting Public memory functions with DMA
 **************************************************************************************************/

bool lv_hal_mem_init(void) {

    app_dma_params_t dma_params = {0};

    dma_params.p_instance                 = LVGL_PORT_DMA_INSTANCE;
    dma_params.channel_number             = LVGL_PORT_DMA_CHANNEL;
    dma_params.init.src_request           = DMA1_REQUEST_MEM;
    dma_params.init.dst_request           = DMA1_REQUEST_MEM;
    dma_params.init.direction             = DMA_MEMORY_TO_MEMORY;
    dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    dma_params.init.dst_increment         = DMA_DST_INCREMENT;
    dma_params.init.src_data_alignment    = DMA_SDATAALIGN_WORD;
    dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_WORD;
    dma_params.init.mode                  = DMA_NORMAL;
    dma_params.init.priority              = DMA_PRIORITY_HIGH;

    s_mem_env.dma_id = app_dma_init(&dma_params, _lv_hal_dma_handler);
    if (s_mem_env.dma_id < 0)
    {
        return true;
    }

    return false;
}

/* not thread-safe now ! */
SECTION_RAM_CODE void lv_hal_memset(void * dst, uint8_t v, uint32_t len) {

    if((len < LVGL_PORT_MEMSET_SMALL_SIZE) ||
       !(app_graphics_is_ospi_address((uint32_t)dst) || app_graphics_is_ospi_address((uint32_t)dst + len))
    ) {
        memset(dst, v, len);
    } else {
        uint8_t * d = (uint8_t*) dst;

        if(s_mem_env.is_busy) {
            printf("ERROR: Pls check Thread-Safe !\r\n");
            return;
        }

        s_mem_env.is_busy     = true;

        uint8_t d_align  = ((uint32_t) dst) & LV_PORT_ALIGN_MASK;
        uint8_t d_align_1  = 0;
        uint8_t d_left   = 0;
        uint32_t new_len = len - (d_align + d_left);

        if(d_align) {
            d_align_1 = d_align = LV_PORT_ALIGN_MASK + 1 - d_align;
            d_left = (len - d_align) & LV_PORT_ALIGN_MASK;
            new_len = len - (d_align + d_left);
            while(d_align && len) {
                *d++ = v;
                d_align--;
                len--;
            }
        } else {
            d_left = len & LV_PORT_ALIGN_MASK;
            new_len = len - d_left;
        }

        if(d_left) {
            d = (uint8_t*)((uint32_t) dst + d_align_1 + new_len);

            while(d_left) {
                *d++ = v;
                d_left--;
            }
        }

        if(new_len) {
            uint16_t ret;
            uint32_t this_sent = 0;
            __align(4) uint32_t val = 0;

            val = ((uint32_t) v << 24) | ((uint32_t) v << 16) | ((uint32_t) v << 8) | (uint32_t) v ;
            _config_dma_xfer_inc_mode(LVGL_PORT_DMA_INSTANCE, LVGL_PORT_DMA_CHANNEL, DMA_SRC_NO_CHANGE, DMA_DST_INCREMENT);
            _config_dma_xfer_width(LVGL_PORT_DMA_INSTANCE, LVGL_PORT_DMA_CHANNEL, DMA_SDATAALIGN_WORD, DMA_DDATAALIGN_WORD);

            s_mem_env.dma_xfer_width     = LV_DMA_XFER_WIDTH_WORD;
            s_mem_env.once_xfer_max_size = LVGL_PORT_DMA_XFER_BLOCK_SIZE * 4;
            this_sent = s_mem_env.once_xfer_max_size < new_len ? s_mem_env.once_xfer_max_size : new_len;

            s_mem_env.is_dma_done = false;
            s_mem_env.is_dma_err  = false;
            s_mem_env.xfered_size = this_sent;
            s_mem_env.total_size  = new_len;

            s_mem_env.mem_op      = LV_PORT_MEM_SET;
            s_mem_env.src_address = (uint32_t) &val;
            s_mem_env.dst_address = (uint32_t) dst + d_align_1 ;

            ret = app_dma_start(s_mem_env.dma_id, s_mem_env.src_address, s_mem_env.dst_address, s_mem_env.xfered_size >> 2);
            if(APP_DRV_SUCCESS == ret) {
                while(!s_mem_env.is_dma_err && !s_mem_env.is_dma_done) {}
            } else {
                s_mem_env.is_dma_err = 1;
            }

            if(s_mem_env.is_dma_err) {
                printf("Pls Check DMA Error!!!\r\n");
            }

            s_mem_env.is_dma_done = false;
            s_mem_env.is_dma_err  = false;
            s_mem_env.xfered_size = 0;
            s_mem_env.total_size  = 0;
            s_mem_env.once_xfer_max_size = 0;
            s_mem_env.mem_op      = LV_PORT_MEM_NONE;
            s_mem_env.is_busy     = false;
        }
    }
}

SECTION_RAM_CODE void * lv_hal_memcpy(void * dst, void * src, uint32_t len) {
    if(len <= LVGL_PORT_MEMCPY_SMALL_SIZE) {
        memcpy(dst, src, len);
    } else {

        uint16_t ret;
        uint32_t this_sent = 0;
        bool prefetch = false;

        if(s_mem_env.is_busy) {
            printf("ERROR: Pls check Thread-Safe !\r\n");
            return dst;
        }

        s_mem_env.is_busy     = true;

        _config_dma_xfer_inc_mode(LVGL_PORT_DMA_INSTANCE, LVGL_PORT_DMA_CHANNEL, DMA_SRC_INCREMENT, DMA_DST_INCREMENT);
        _config_dma_xfer_width(LVGL_PORT_DMA_INSTANCE, LVGL_PORT_DMA_CHANNEL, DMA_SDATAALIGN_BYTE, DMA_DDATAALIGN_BYTE);

        s_mem_env.dma_xfer_width     = LV_DMA_XFER_WIDTH_BYTE;
        s_mem_env.once_xfer_max_size = LVGL_PORT_DMA_XFER_BLOCK_SIZE;

        this_sent = s_mem_env.once_xfer_max_size < len ? s_mem_env.once_xfer_max_size : len;

        s_mem_env.is_dma_done = false;
        s_mem_env.is_dma_err  = false;
        s_mem_env.xfered_size = this_sent;
        s_mem_env.total_size  = len;

        s_mem_env.mem_op      = LV_PORT_MEM_CPY;
        s_mem_env.src_address = (uint32_t) src;
        s_mem_env.dst_address = (uint32_t) dst;

        if(app_graphics_is_ospi_address(s_mem_env.src_address) ||
           app_graphics_is_ospi_address(s_mem_env.src_address + len)) {
            if(!app_graphics_ospi_get_read_prefetch()) {
                app_graphics_ospi_set_read_prefetch(true);
                prefetch = true;
            }
        }

        ret = app_dma_start(s_mem_env.dma_id, s_mem_env.src_address, s_mem_env.dst_address, s_mem_env.xfered_size >> 0);
        if(APP_DRV_SUCCESS == ret) {
            while(!s_mem_env.is_dma_err && !s_mem_env.is_dma_done) {}
        } else {
            s_mem_env.is_dma_err = 1;
        }

        if(s_mem_env.is_dma_err) {
            printf("Pls Check DMA Error!!!\r\n");
        }

        if(prefetch) {
            app_graphics_ospi_set_read_prefetch(false);
        }
        s_mem_env.is_dma_done = false;
        s_mem_env.is_dma_err  = false;
        s_mem_env.xfered_size = 0;
        s_mem_env.total_size  = 0;
        s_mem_env.once_xfer_max_size = 0;
        s_mem_env.mem_op      = LV_PORT_MEM_NONE;
        s_mem_env.is_busy     = false;
    }

    return dst;
}

bool lv_hal_mem_rect_blit(mem_rect_blit_config_t * p_blit_config, lv_dma_xfer_width_e xfer_width) {

    bool ret                 = true;
    uint16_t status          = HAL_OK;
    uint32_t this_sent_lines = 0;
    dma_sg_llp_config_t     sg_llp_config;

    if(p_blit_config == NULL) {
        return false;
    }

    if(p_blit_config->src_img_x + p_blit_config->src_img_x_delta > p_blit_config->src_img_w) {
        return false;
    }

    if(p_blit_config->src_img_y + p_blit_config->src_img_y_delta > p_blit_config->src_img_h) {
        return false;
    }

    const uint32_t src_start_address = p_blit_config->src_img_address + (p_blit_config->src_img_y * p_blit_config->src_img_w + p_blit_config->src_img_x) * p_blit_config->pixel_depth;
    const uint32_t dst_start_address = p_blit_config->dst_buff_address + (p_blit_config->dst_buff_y * p_blit_config->dst_buff_width + p_blit_config->dst_buff_x) * p_blit_config->pixel_depth;
    const uint32_t line_length       = p_blit_config->src_img_x_delta * p_blit_config->pixel_depth;
    const uint32_t total_lines       = p_blit_config->src_img_y_delta;

    {
        if(LV_DMA_XFER_WIDTH_BYTE == xfer_width) {
            _config_dma_xfer_inc_mode(LVGL_PORT_DMA_INSTANCE, LVGL_PORT_DMA_CHANNEL, DMA_SRC_INCREMENT, DMA_DST_INCREMENT);
            _config_dma_xfer_width(LVGL_PORT_DMA_INSTANCE, LVGL_PORT_DMA_CHANNEL, DMA_SDATAALIGN_BYTE, DMA_DDATAALIGN_BYTE);
            s_mem_env.once_xfer_max_size = LVGL_PORT_DMA_XFER_BLOCK_SIZE/(line_length);
            s_mem_env.dma_xfer_width     = LV_DMA_XFER_WIDTH_BYTE;
        } else if(LV_DMA_XFER_WIDTH_HALFWORD == xfer_width) {
            _config_dma_xfer_inc_mode(LVGL_PORT_DMA_INSTANCE, LVGL_PORT_DMA_CHANNEL, DMA_SRC_INCREMENT, DMA_DST_INCREMENT);
            _config_dma_xfer_width(LVGL_PORT_DMA_INSTANCE, LVGL_PORT_DMA_CHANNEL, DMA_SDATAALIGN_HALFWORD, DMA_DDATAALIGN_HALFWORD);
            s_mem_env.once_xfer_max_size = (LVGL_PORT_DMA_XFER_BLOCK_SIZE*2)/(line_length);
            s_mem_env.dma_xfer_width     = LV_DMA_XFER_WIDTH_HALFWORD;
        }  else if(LV_DMA_XFER_WIDTH_WORD == xfer_width) {
            _config_dma_xfer_inc_mode(LVGL_PORT_DMA_INSTANCE, LVGL_PORT_DMA_CHANNEL, DMA_SRC_INCREMENT, DMA_DST_INCREMENT);
            _config_dma_xfer_width(LVGL_PORT_DMA_INSTANCE, LVGL_PORT_DMA_CHANNEL, DMA_SDATAALIGN_WORD, DMA_DDATAALIGN_WORD);
            s_mem_env.once_xfer_max_size = (LVGL_PORT_DMA_XFER_BLOCK_SIZE*4)/(line_length);
            s_mem_env.dma_xfer_width     = LV_DMA_XFER_WIDTH_WORD;
        }

        /* config S&G */
        {
            /* source - gather */
            sg_llp_config.gather_config.src_sgc         = (p_blit_config->src_img_x_delta * p_blit_config->pixel_depth) >> s_mem_env.dma_xfer_width;
            sg_llp_config.gather_config.src_sgi         = ((p_blit_config->src_img_w - p_blit_config->src_img_x_delta) * p_blit_config->pixel_depth) >> s_mem_env.dma_xfer_width;
            sg_llp_config.gather_config.src_gather_en   = DMA_SRC_GATHER_ENABLE;

            /* dest - scatter */
            sg_llp_config.scatter_config.dst_dsc        = (p_blit_config->src_img_x_delta * p_blit_config->pixel_depth) >> s_mem_env.dma_xfer_width;
            sg_llp_config.scatter_config.dst_dsi        = ((p_blit_config->dst_buff_width - p_blit_config->src_img_x_delta) * p_blit_config->pixel_depth) >> s_mem_env.dma_xfer_width;
            sg_llp_config.scatter_config.dst_scatter_en = DMA_DST_SCATTER_ENABLE;

            sg_llp_config.llp_config.head_lli           = NULL;
            sg_llp_config.llp_config.llp_src_en         = DMA_LLP_SRC_DISABLE;
            sg_llp_config.llp_config.llp_dst_en         = DMA_LLP_DST_DISABLE;
            sg_llp_config.llp_config.llp_src_writeback  = 0;
            sg_llp_config.llp_config.llp_dst_writeback  = 0;
        }

        ret        = true;

        this_sent_lines = total_lines < s_mem_env.once_xfer_max_size ? total_lines : s_mem_env.once_xfer_max_size;

        s_mem_env.is_dma_done = false;
        s_mem_env.is_dma_err  = false;

        s_mem_env.xfered_size = this_sent_lines;
        s_mem_env.total_size  = total_lines;        /* record the total lines as size */

        s_mem_env.mem_op      = LV_PORT_MEM_BLIT;
        s_mem_env.src_address = (uint32_t) src_start_address;
        s_mem_env.dst_address = (uint32_t) dst_start_address;
        s_mem_env.sg_src_delta_width = p_blit_config->src_img_w*p_blit_config->pixel_depth;
        s_mem_env.sg_dst_delta_width = p_blit_config->dst_buff_width*p_blit_config->pixel_depth;

        s_mem_env.is_dma_done = 0;
        s_mem_env.is_dma_err  = 0;
        s_mem_env.sg_line_length = line_length;
        memcpy(&s_mem_env.sgc, &sg_llp_config, sizeof(dma_sg_llp_config_t));

        status = app_dma_start_sg_llp(s_mem_env.dma_id, s_mem_env.src_address, s_mem_env.dst_address, (this_sent_lines*line_length) >> xfer_width, &sg_llp_config);

        if(APP_DRV_SUCCESS == status) {
            while(!s_mem_env.is_dma_err && !s_mem_env.is_dma_done) {}
        } else {
            s_mem_env.is_dma_err = 1;
        }

        if(s_mem_env.is_dma_err) {
            printf("Pls Check DMA Error!!!\r\n");
            ret        = false;
        }
    }

    s_mem_env.is_dma_done = false;
    s_mem_env.is_dma_err  = false;
    s_mem_env.xfered_size = 0;
    s_mem_env.total_size  = 0;
    s_mem_env.once_xfer_max_size = 0;
    s_mem_env.sg_line_length = 0;
    s_mem_env.sg_src_delta_width = 0;
    s_mem_env.sg_dst_delta_width = 0;
    s_mem_env.mem_op      = LV_PORT_MEM_NONE;
    s_mem_env.is_busy     = false;

    return ret;
}


/**************************************************************************************************
 *               Local memory functions with DMA
 **************************************************************************************************/

SECTION_RAM_CODE static void _config_dma_xfer_inc_mode(dma_regs_t * p_instance, dma_channel_t channel, uint32_t src_inc_mode, uint32_t dst_inc_mode)
{
    ll_dma_set_source_increment_mode(p_instance, channel, src_inc_mode);
    ll_dma_set_destination_increment_mode(p_instance, channel, dst_inc_mode);
}

SECTION_RAM_CODE static void _config_dma_xfer_width(dma_regs_t * p_instance, dma_channel_t channel, uint32_t src_width, uint32_t dst_width)
{
    ll_dma_set_source_width(p_instance, channel, src_width);
    ll_dma_set_destination_width(p_instance, channel, dst_width);
}

SECTION_RAM_CODE static void _config_dma_xfer_mszie(dma_regs_t * p_instance, dma_channel_t channel, uint32_t src_mszie, uint32_t dst_msize)
{
    ll_dma_set_source_burst_length(p_instance, channel, src_mszie);
    ll_dma_set_destination_burst_length(p_instance, channel, dst_msize);
}

SECTION_RAM_CODE static void _lv_hal_dma_handler(app_dma_evt_type_t type) {
    uint16_t ret;
    uint32_t offset = 0;
    switch(type) {
        case APP_DMA_EVT_TFR:
        case APP_DMA_EVT_BLK:
        {
            if(s_mem_env.xfered_size < s_mem_env.total_size) {
                uint32_t this_sent = ((s_mem_env.total_size - s_mem_env.xfered_size) > s_mem_env.once_xfer_max_size) ? s_mem_env.once_xfer_max_size : (s_mem_env.total_size - s_mem_env.xfered_size);

                switch(s_mem_env.mem_op) {
                    case LV_PORT_MEM_SET:
                    {
                        offset = s_mem_env.xfered_size;
                        s_mem_env.xfered_size += this_sent;
                        ret = app_dma_start(s_mem_env.dma_id, s_mem_env.src_address, s_mem_env.dst_address + offset, this_sent >> (uint8_t)s_mem_env.dma_xfer_width);

                        if(APP_DRV_SUCCESS != ret) {
                            s_mem_env.is_dma_err = 1;
                            s_mem_env.is_dma_done = 1;
                        }
                    }
                    break;

                    case LV_PORT_MEM_CPY:
                    {
                        offset = s_mem_env.xfered_size;
                        s_mem_env.xfered_size += this_sent;
                        ret = app_dma_start(s_mem_env.dma_id, s_mem_env.src_address + offset, s_mem_env.dst_address + offset, this_sent >> (uint8_t)s_mem_env.dma_xfer_width);

                        if(APP_DRV_SUCCESS != ret) {
                            s_mem_env.is_dma_err = 1;
                            s_mem_env.is_dma_done = 1;
                        }
                    }
                    break;

                    case LV_PORT_MEM_BLIT:
                    {
                        offset = s_mem_env.xfered_size;
                        s_mem_env.xfered_size += this_sent;
                        ret = app_dma_start_sg_llp(s_mem_env.dma_id, s_mem_env.src_address + offset*s_mem_env.sg_src_delta_width,
                                                   s_mem_env.dst_address + offset*s_mem_env.sg_dst_delta_width,
                                                   (this_sent*s_mem_env.sg_line_length) >> (uint8_t)s_mem_env.dma_xfer_width,
                                                   &s_mem_env.sgc);

                        if(APP_DRV_SUCCESS != ret) {
                            s_mem_env.is_dma_err = 1;
                            s_mem_env.is_dma_done = 1;
                        }
                    }
                    break;

                    default:
                    {
                        printf("INVALID Switch!!!\r\n");
                    }
                    break;
                }
            } else {
                s_mem_env.is_dma_err = 0;
                s_mem_env.is_dma_done = 1;
            }
        }
        break;

        default:
        case APP_DMA_EVT_ERROR:
        {
            s_mem_env.is_dma_err = 1;
            s_mem_env.is_dma_done = 1;
        }
        break;
    }

    return;
}

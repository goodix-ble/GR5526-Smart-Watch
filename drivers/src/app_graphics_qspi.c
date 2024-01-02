/**
  ****************************************************************************************
  * @file    app_graphics_qspi.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2019 GOODIX
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of GOODIX nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
  ****************************************************************************************
  */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_assert.h"
#include "app_qspi.h"
#include "app_qspi_dma.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include <string.h>
#include "platform_sdk.h"
#include "app_drv.h"
#include "gr_soc.h"

#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X))
#include "app_graphics_qspi.h"
#include "gr55xx_ll_qspi.h"

#ifdef HAL_QSPI_MODULE_ENABLED
/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_QSPI_EXCEPT_DEBUG_EN            1u
#define APP_QSPI_IN_DEBUG_MODE              0
/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X))
static bool         app_qspi_switch_dma_mode(app_qspi_id_t id, bool is_m2m_mode);
#if (QSPI_DMA_LLP_FEATUTE_SUPPORT > 0u)
static bool         app_qspi_cmd_llp_transmit(app_qspi_id_t screen_id, app_qspi_command_t * p_cmd, dma_llp_config_t * p_llp_config, bool is_sync);
static bool         app_qspi_llp_transmit(app_qspi_id_t screen_id, dma_llp_config_t * p_llp_config, uint32_t data_mode, uint32_t data_len, bool is_sync);
#endif
static void         app_qspi_dma_evt_handler_0(app_dma_evt_type_t type);
static void         app_qspi_dma_evt_handler_1(app_dma_evt_type_t type);
static void         app_qspi_dma_evt_handler_2(app_dma_evt_type_t type);
extern bool         app_qspi_mmap_set_prefetch(app_qspi_id_t id, bool prefetch_en);
extern void         app_qspi_force_cs(app_qspi_id_t screen_id, bool low_level);
#endif
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const app_dma_evt_handler_t  s_dma_evt_handler[APP_QSPI_ID_MAX]  = {app_qspi_dma_evt_handler_0, app_qspi_dma_evt_handler_1, app_qspi_dma_evt_handler_2};
extern const uint32_t s_qspi_instance[APP_QSPI_ID_MAX];
extern qspi_env_t *p_qspi_env[APP_QSPI_ID_MAX];

#if (QSPI_DMA_LLP_FEATUTE_SUPPORT > 0u)
    static dma_block_config_t           s_dma_llp_block[DMA_LLP_BLOCKS_FOR_WRITE*2 + 1];
    app_qspi_async_draw_screen_info_t    s_async_write_screen_info;
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X))
bool app_qspi_dma_mmap_read_block(app_qspi_id_t id, uint32_t address, uint8_t * buffer, uint32_t length) {
    hal_status_t status;
    bool ret = true;
    APP_ASSERT_CHECK(p_qspi_env[id]->is_mmap_inited);

    app_qspi_mmap_set_endian_mode(id, APP_QSPI_MMAP_ENDIAN_MODE_0);
    APP_ASSERT_CHECK(p_qspi_env[id]->is_used_dma);
    app_qspi_mmap_set_prefetch(id, true);
    ret = app_qspi_switch_dma_mode(id, true);
    if(ret) {
        p_qspi_env[id]->is_dma_done = 0;
        p_qspi_env[id]->is_xfer_err = 0;
        status = hal_dma_start_it(p_qspi_env[id]->handle.p_dma, ll_qspi_get_xip_base_address((qspi_regs_t*)s_qspi_instance[id]) + address, (uint32_t)buffer, length);
        if(HAL_OK == status) {
            while(!p_qspi_env[id]->is_dma_done && !p_qspi_env[id]->is_xfer_err);
            if(p_qspi_env[id]->is_xfer_err) {
                ret = false;
            }
        } else {
            ret = false;
        }
    }

    app_qspi_mmap_set_prefetch(id, false);
    return ret;
}

bool app_qspi_dma_llp_scroll_read(app_qspi_id_t id, uint8_t data_size, uint32_t llp_src_en, uint32_t llp_dst_en, app_qspi_scroll_read_info_t* p_link_scroll_read, uint32_t link_len) {
#if (QSPI_ASYNC_VERI_LINK_DRAW_SCREEN_SUPPORT > 0u)

    hal_status_t status;
    bool ret = true;
    dma_sg_llp_config_t sg_llp_config;
    uint32_t j;

    APP_ASSERT_CHECK(p_qspi_env[id]->is_mmap_inited);
    APP_ASSERT_CHECK(p_qspi_env[id]->is_used_dma);
    app_qspi_mmap_set_prefetch(id, true);
    ret = app_qspi_switch_dma_mode(id, true);
    if(ret) {
        sg_llp_config.scatter_config.dst_scatter_en = DMA_DST_SCATTER_DISABLE;
        sg_llp_config.scatter_config.dst_dsi = 0x2;
        sg_llp_config.scatter_config.dst_dsc = 0x1;
        sg_llp_config.gather_config.src_gather_en = DMA_SRC_GATHER_DISABLE;
        sg_llp_config.gather_config.src_sgi = 0x2;
        sg_llp_config.gather_config.src_sgc = 0x1;
        sg_llp_config.llp_config.head_lli = &s_dma_llp_block[0];
        sg_llp_config.llp_config.llp_src_writeback = 0XAA;
        sg_llp_config.llp_config.llp_dst_writeback = 0XBB;

        uint32_t dma_sdata_align,dma_ddata_align;
        if(data_size == QSPI_DATASIZE_32_BITS)
        {
            dma_sdata_align = DMA_SDATAALIGN_WORD;
            dma_ddata_align = DMA_DDATAALIGN_WORD;
            app_qspi_mmap_set_endian_mode(id, APP_QSPI_MMAP_ENDIAN_MODE_2);
        }
        else if(data_size == QSPI_DATASIZE_16_BITS)
        {
            dma_sdata_align = DMA_SDATAALIGN_HALFWORD;
            dma_ddata_align = DMA_DDATAALIGN_HALFWORD;
            app_qspi_mmap_set_endian_mode(id, APP_QSPI_MMAP_ENDIAN_MODE_1);
        }
        else
        {
            dma_sdata_align = DMA_SDATAALIGN_BYTE;
            dma_ddata_align = DMA_DDATAALIGN_BYTE;
            app_qspi_mmap_set_endian_mode(id, APP_QSPI_MMAP_ENDIAN_MODE_0);
        }
        uint32_t isrc_llp_en;
        if(llp_src_en > 0)
        {
            sg_llp_config.llp_config.llp_src_en = DMA_LLP_SRC_ENABLE;
            isrc_llp_en = DMA_LLP_SRC_ENABLE;
        }
        else
        {
            sg_llp_config.llp_config.llp_src_en = DMA_LLP_SRC_DISABLE;
            isrc_llp_en = DMA_LLP_SRC_DISABLE;
        }
        uint32_t idst_llp_en;
        if(llp_dst_en > 0)
        {
            sg_llp_config.llp_config.llp_dst_en = DMA_LLP_DST_ENABLE;
            idst_llp_en = DMA_LLP_DST_ENABLE;
        }
        else
        {
            sg_llp_config.llp_config.llp_dst_en = DMA_LLP_DST_DISABLE;
            idst_llp_en = DMA_LLP_DST_DISABLE;
        }
        uint32_t ctrl_low = DMA_CTLL_INI_EN | DMA_DST_INCREMENT | DMA_SRC_INCREMENT | dma_sdata_align | dma_ddata_align | DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_DISABLE | isrc_llp_en | idst_llp_en | LL_DMA_SRC_BURST_LENGTH_1 | LL_DMA_DST_BURST_LENGTH_1 | DMA_MEMORY_TO_MEMORY;

        memset(&s_dma_llp_block[0], 0, sizeof(dma_block_config_t) * (link_len +1));

        app_qspi_scroll_read_info_t* p_current_scroll_read_info = p_link_scroll_read;
        for (j = 0; j < link_len; j++) {
            s_dma_llp_block[j].src_address = p_current_scroll_read_info->src_start_address;
            s_dma_llp_block[j].dst_address = p_current_scroll_read_info->dst_start_address;
            s_dma_llp_block[j].p_lli = &s_dma_llp_block[j +1];
            s_dma_llp_block[j].CTL_L = ctrl_low;
            s_dma_llp_block[j].CTL_H = p_current_scroll_read_info->length;
            s_dma_llp_block[j].src_status = 0x0;
            s_dma_llp_block[j].dst_status = 0x0;

            p_current_scroll_read_info = p_current_scroll_read_info->next;
        }
        s_dma_llp_block[j - 1].p_lli = NULL;

        p_qspi_env[id]->is_dma_done = 0;
        p_qspi_env[id]->is_xfer_err = 0;
        status = hal_dma_start_sg_llp_it(p_qspi_env[id]->handle.p_dma, p_link_scroll_read->src_start_address, p_link_scroll_read->dst_start_address, p_link_scroll_read->length, &sg_llp_config);
        if (HAL_OK == status)
        {
            while(!p_qspi_env[id]->is_dma_done && !p_qspi_env[id]->is_xfer_err);
            if(p_qspi_env[id]->is_xfer_err) {
                ret = false;
            }
        }
        else
        {
            ret = false;
        }
    }

    app_qspi_mmap_set_prefetch(id, false);
    return ret;
#else
    return false;
#endif
}

#if APP_QSPI_IN_DEBUG_MODE
bool app_qspi_sync_draw_screen(app_qspi_id_t screen_id, app_qspi_id_t storage_id, const app_qspi_screen_command_t * const p_screen_cmd, const app_qspi_screen_info_t * const p_screen_info, app_qspi_screen_scroll_t * p_scroll_config) {

#if (QSPI_SYNC_SCROLL_DRAW_SCREEN_SUPPORT > 0u)

    app_qspi_command_t          app_scrn_cmd;
    dma_llp_config_t            scrn_llp_config;
    app_qspi_screen_scroll_t    s_scroll_config;
    uint32_t this_send_lines         = 0;
    uint32_t sent_lines              = 0;
    uint32_t image_start_address     = 0;
    uint32_t j                       = 0;
    uint32_t llp_cfg_right_shift_bit = 0;
    uint32_t llp_cfg_ctrl_low        = 0;
    uint32_t sent_line_order         = 0;
    bool ret                         = false;
    bool is_lead_addr                = true;
    bool is_stored_in_qspi_storage   = false;

    const uint32_t scrn_pixel_height        = p_screen_info->scrn_pixel_height;
    const uint32_t scrn_pixel_width         = p_screen_info->scrn_pixel_width;
    const uint32_t scrn_pixel_depth         = p_screen_info->scrn_pixel_depth;
    const uint32_t scrn_line_size           = (scrn_pixel_width * scrn_pixel_depth);
    const uint32_t scrn_refresh_lines_once  = (QSPI_MAX_XFER_SIZE_ONCE/scrn_line_size);
    const uint32_t send_lines_once          = DMA_LLP_BLOCKS_FOR_WRITE < scrn_refresh_lines_once ? DMA_LLP_BLOCKS_FOR_WRITE : scrn_refresh_lines_once;

    if((p_screen_cmd == NULL) || (p_screen_info == NULL) || (p_scroll_config == NULL)) {
        return false;
    }

    if(storage_id <= APP_QSPI_ID_2) {
        is_stored_in_qspi_storage   = true;
    } else if (storage_id == APP_STORAGE_RAM_ID) {
        is_stored_in_qspi_storage   = false;
    } else {
        return false;
    }

    if(screen_id == storage_id) {
        return false;
    }

    if ((p_qspi_env[screen_id] == NULL) || (p_qspi_env[screen_id]->qspi_state == APP_QSPI_INVALID) ||
        (p_qspi_env[storage_id] == NULL) || (p_qspi_env[storage_id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    /* check screen height/width/depth */
    if((scrn_pixel_width  % 2 != 0) ||
       (scrn_pixel_height % 2 != 0) ||
       (scrn_pixel_depth      != 2)) {
        return false;
    }

    if (p_qspi_env[screen_id]->start_flag || (is_stored_in_qspi_storage && p_qspi_env[storage_id]->start_flag)) {
        return false;
    }

    p_qspi_env[screen_id]->start_flag = true;
    if(is_stored_in_qspi_storage) p_qspi_env[storage_id]->start_flag = true;

    if(is_stored_in_qspi_storage)
    {
        app_qspi_mmap_set_prefetch(storage_id, true);
    }

    APP_ASSERT_CHECK(p_qspi_env[screen_id]->is_used_dma);
    app_qspi_switch_dma_mode(screen_id,    false);

    if(QSPI_DATASIZE_08_BITS == p_screen_cmd->data_size) {
        llp_cfg_right_shift_bit = 0;
        llp_cfg_ctrl_low        = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_BYTE, DMA_DDATAALIGN_BYTE, LL_DMA_SRC_BURST_LENGTH_8, LL_DMA_DST_BURST_LENGTH_8, DMA_SRC_GATHER_DISABLE);
        if(is_stored_in_qspi_storage) app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_0);
    } else if (QSPI_DATASIZE_16_BITS == p_screen_cmd->data_size) {
        llp_cfg_right_shift_bit = 1;
        llp_cfg_ctrl_low        = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_HALFWORD, DMA_DDATAALIGN_HALFWORD, LL_DMA_SRC_BURST_LENGTH_8, LL_DMA_DST_BURST_LENGTH_8, DMA_SRC_GATHER_DISABLE);
        if(is_stored_in_qspi_storage) app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_1);
    } else if (QSPI_DATASIZE_32_BITS == p_screen_cmd->data_size) {
        llp_cfg_right_shift_bit = 2;
        llp_cfg_ctrl_low        = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_WORD, DMA_DDATAALIGN_WORD, LL_DMA_SRC_BURST_LENGTH_8, LL_DMA_DST_BURST_LENGTH_8, DMA_SRC_GATHER_DISABLE);
        if(is_stored_in_qspi_storage) app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_2);
    } else {
        p_qspi_env[screen_id]->start_flag = false;
        if(is_stored_in_qspi_storage) p_qspi_env[storage_id]->start_flag = false;

        return false;
    }

    memcpy(&s_scroll_config, p_scroll_config, sizeof(app_qspi_screen_scroll_t));

    /* check and adjust start & end coordinate */
    if(s_scroll_config.is_horizontal_scroll) {
        if(s_scroll_config.scroll_coordinate > scrn_pixel_width) {
           s_scroll_config.scroll_coordinate = scrn_pixel_width;
        }

    } else {
        if(s_scroll_config.scroll_coordinate > scrn_pixel_height) {
           s_scroll_config.scroll_coordinate = scrn_pixel_height;
        }
    }

    /* adjust scroll_coordinate to even number */
    s_scroll_config.scroll_coordinate = (s_scroll_config.scroll_coordinate >> 1) << 1;

    const uint32_t image_1_ahb_address = p_scroll_config->first_frame_start_address;
    const uint32_t image_2_ahb_address = p_scroll_config->second_frame_start_address;

    APP_ASSERT_CHECK(p_qspi_env[screen_id]->is_used_dma);
    app_qspi_mmap_set_prefetch(storage_id, true);

    /* prepare llp config */
    scrn_llp_config.llp_src_writeback       = 1;
    scrn_llp_config.llp_dst_writeback       = 1;
    scrn_llp_config.llp_src_en              = DMA_LLP_SRC_ENABLE;
    scrn_llp_config.llp_dst_en              = DMA_LLP_DST_DISABLE;
    scrn_llp_config.head_lli                = &s_dma_llp_block[0];

    app_scrn_cmd.instruction                = p_screen_cmd->instruction;
    app_scrn_cmd.instruction_size           = p_screen_cmd->instruction_size;
    app_scrn_cmd.address                    = p_screen_cmd->leading_address;
    app_scrn_cmd.address_size               = p_screen_cmd->address_size;
    app_scrn_cmd.dummy_cycles               = p_screen_cmd->dummy_cycles;
    app_scrn_cmd.data_size                  = p_screen_cmd->data_size;
    app_scrn_cmd.instruction_address_mode   = p_screen_cmd->instruction_address_mode;
    app_scrn_cmd.data_mode                  = p_screen_cmd->data_mode;
    app_scrn_cmd.length                     = 0x00;
    app_scrn_cmd.clock_stretch_en           = 1;

    is_lead_addr = true;

    if(s_scroll_config.is_horizontal_scroll) {

        if((s_scroll_config.scroll_coordinate == 0) ||                   /* Only left image */
           (s_scroll_config.scroll_coordinate == scrn_pixel_width)) {    /* Only right image */
            this_send_lines = 0;
            sent_lines = 0;

            if(0 == s_scroll_config.scroll_coordinate) {
                image_start_address = image_1_ahb_address;
            } else {
                image_start_address = image_2_ahb_address;
            }

            while(sent_lines < scrn_pixel_height) {
                this_send_lines = (scrn_pixel_height - sent_lines) < send_lines_once ? (scrn_pixel_height - sent_lines) : send_lines_once;

                memset(&s_dma_llp_block[0], 0, sizeof(dma_block_config_t) * (this_send_lines*2 + 1));

                for(j = 0; j < this_send_lines; j++) {
                    s_dma_llp_block[j].src_address    = image_start_address + (sent_lines + j) * scrn_line_size ;
                    s_dma_llp_block[j].dst_address    = 0;
                    s_dma_llp_block[j].p_lli          = &s_dma_llp_block[j + 1];
                    s_dma_llp_block[j].CTL_L          = llp_cfg_ctrl_low;
                    s_dma_llp_block[j].CTL_H          = (uint32_t)(scrn_line_size >> llp_cfg_right_shift_bit);
                    s_dma_llp_block[j].src_status     = 0x0;
                    s_dma_llp_block[j].dst_status     = 0x0;
                }
                s_dma_llp_block[j - 1].p_lli          = NULL;

                if(is_lead_addr) {
                    app_scrn_cmd.address = p_screen_cmd->leading_address;
                    is_lead_addr = false;
                } else {
                    app_scrn_cmd.address = p_screen_cmd->ongoing_address;
                }

                app_scrn_cmd.length = this_send_lines * scrn_line_size;
                ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, true);

                if(!ret) {
                    goto __fail;
                }

                sent_lines += this_send_lines;
            }
        } else {    /* part of left image, part of right image */
            this_send_lines = 0;
            sent_lines = 0;

            while(sent_lines < scrn_pixel_height) {
                this_send_lines = (scrn_pixel_height - sent_lines) < send_lines_once ? (scrn_pixel_height - sent_lines) : send_lines_once;

                memset(&s_dma_llp_block[0], 0, sizeof(dma_block_config_t) * (this_send_lines*2 + 1));

                for(j = 0; j < this_send_lines; j++) {
                    /* first image */
                    s_dma_llp_block[2*j].src_address    = image_1_ahb_address + (sent_lines + j) * scrn_line_size + s_scroll_config.scroll_coordinate * scrn_pixel_depth;
                    s_dma_llp_block[2*j].dst_address    = 0;
                    s_dma_llp_block[2*j].p_lli          = &s_dma_llp_block[2*j + 1];
                    s_dma_llp_block[2*j].CTL_L          = llp_cfg_ctrl_low;
                    s_dma_llp_block[2*j].CTL_H          = (uint32_t)(((scrn_pixel_width - s_scroll_config.scroll_coordinate)*scrn_pixel_depth) >> llp_cfg_right_shift_bit);
                    s_dma_llp_block[2*j].src_status     = 0x0;
                    s_dma_llp_block[2*j].dst_status     = 0x0;

                    /* second image */
                    s_dma_llp_block[2*j + 1].src_address    = image_2_ahb_address + (sent_lines + j) * scrn_line_size ;
                    s_dma_llp_block[2*j + 1].dst_address    = 0;
                    s_dma_llp_block[2*j + 1].p_lli          = &s_dma_llp_block[2*j + 2];
                    s_dma_llp_block[2*j + 1].CTL_L          = llp_cfg_ctrl_low;
                    s_dma_llp_block[2*j + 1].CTL_H          = (uint32_t)((s_scroll_config.scroll_coordinate * scrn_pixel_depth) >> llp_cfg_right_shift_bit);
                    s_dma_llp_block[2*j + 1].src_status     = 0x0;
                    s_dma_llp_block[2*j + 1].dst_status     = 0x0;

                }
                s_dma_llp_block[2*j - 1].p_lli          = NULL;

                if(is_lead_addr) {
                    app_scrn_cmd.address = p_screen_cmd->leading_address;
                    is_lead_addr = false;
                } else {
                    app_scrn_cmd.address = p_screen_cmd->ongoing_address;
                }
                app_scrn_cmd.length = this_send_lines * scrn_line_size;
                ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, true);

                if(!ret) {
                    goto __fail;
                }

                sent_lines += this_send_lines;
            }
        }
    } else { /* Scroll vertically */
        sent_line_order = s_scroll_config.scroll_coordinate;
        this_send_lines = 0;
        sent_lines      = 0;

        while(sent_lines < scrn_pixel_height) {

            this_send_lines     = (scrn_pixel_height - sent_lines) < send_lines_once ? (scrn_pixel_height - sent_lines) : send_lines_once;
            memset(&s_dma_llp_block[0], 0, sizeof(dma_block_config_t) * (this_send_lines*2 + 1));

            for(j = 0; j < this_send_lines; j++) {

                image_start_address = sent_line_order < scrn_pixel_height ? image_1_ahb_address : image_2_ahb_address;

                s_dma_llp_block[j].src_address    = image_start_address + (sent_line_order % scrn_pixel_height) * scrn_line_size ;
                s_dma_llp_block[j].dst_address    = 0;
                s_dma_llp_block[j].p_lli          = &s_dma_llp_block[j + 1];
                s_dma_llp_block[j].CTL_L          = llp_cfg_ctrl_low;
                s_dma_llp_block[j].CTL_H          = (uint32_t)(scrn_line_size >> llp_cfg_right_shift_bit);
                s_dma_llp_block[j].src_status     = 0x0;
                s_dma_llp_block[j].dst_status     = 0x0;

                sent_line_order ++;
            }
            s_dma_llp_block[j - 1].p_lli          = NULL;

            if(is_lead_addr) {
                app_scrn_cmd.address = p_screen_cmd->leading_address;
                is_lead_addr = false;
            } else {
                app_scrn_cmd.address = p_screen_cmd->ongoing_address;
            }

            app_scrn_cmd.length = this_send_lines * scrn_line_size;
            ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, true);

            if(!ret) {
                goto __fail;
            }

            sent_lines += this_send_lines;
        }
    }

    p_qspi_env[screen_id]->start_flag = false;
    if(is_stored_in_qspi_storage) p_qspi_env[storage_id]->start_flag = false;
    return true;

__fail:
    p_qspi_env[screen_id]->start_flag = false;
    if(is_stored_in_qspi_storage) p_qspi_env[storage_id]->start_flag = false;
    return false;

#else
    return false;
#endif
}
#endif

bool app_qspi_async_draw_screen(app_qspi_id_t screen_id, app_qspi_id_t storage_id, const app_qspi_screen_command_t * const p_screen_cmd, const app_qspi_screen_info_t * const p_screen_info, app_qspi_screen_scroll_t * p_scroll_config, bool is_first_call) {

#if (QSPI_ASYNC_SCROLL_DRAW_SCREEN_SUPPORT > 0u)

    app_qspi_command_t            app_scrn_cmd;
    dma_llp_config_t              scrn_llp_config;
    uint32_t image_start_address   = 0;
    uint32_t j                     = 0;
    bool ret                       = false;
    bool is_stored_in_qspi_storage = false;

    if((p_screen_cmd == NULL) || (p_screen_info == NULL) || (p_scroll_config == NULL)) {
        return false;
    }

    if(screen_id > APP_QSPI_ID_2) {
        return false;
    }

    if((storage_id > APP_QSPI_ID_2) && (storage_id != APP_STORAGE_RAM_ID)) {
        return false;
    }

    if(screen_id == storage_id) {
        return false;
    }

    if(storage_id <= APP_QSPI_ID_2) {
        is_stored_in_qspi_storage   = true;
    } else if (storage_id == APP_STORAGE_RAM_ID) {
        is_stored_in_qspi_storage   = false;
    } else {
        return false;
    }

    if ((p_qspi_env[screen_id] == NULL) || (p_qspi_env[screen_id]->qspi_state == APP_QSPI_INVALID) ||
        (is_stored_in_qspi_storage && ((p_qspi_env[storage_id] == NULL) || (p_qspi_env[storage_id]->qspi_state == APP_QSPI_INVALID)))
        )
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if(p_qspi_env[screen_id]->is_async_write_screen && is_first_call) {
        printf("RE-CALLED !\r\n");
        return false;
    }

    if(is_first_call) {
        if (p_qspi_env[screen_id]->start_flag || (is_stored_in_qspi_storage && p_qspi_env[storage_id]->start_flag)) {
            printf("BUSY... !\r\n");
            return false;
        }

        p_qspi_env[screen_id]->start_flag = true;
        if(is_stored_in_qspi_storage){
            p_qspi_env[storage_id]->start_flag = true;
        }

        memset(&s_async_write_screen_info, 0, sizeof(app_qspi_async_draw_screen_info_t));

        s_async_write_screen_info.if_type    = DRAW_TYPE_IF_DUAL_SCREEN;
        s_async_write_screen_info.screen_id  = screen_id;
        s_async_write_screen_info.storage_id = storage_id;

        memcpy(&s_async_write_screen_info.qspi_screen_command,      p_screen_cmd,    sizeof(app_qspi_screen_command_t));
        memcpy(&s_async_write_screen_info.screen_info,              p_screen_info,   sizeof(app_qspi_screen_info_t));
        memcpy(&s_async_write_screen_info.ss.dual_ss.scroll_config, p_scroll_config, sizeof(app_qspi_screen_scroll_t));

        /* check and adjust start & end coordinate */
        if(s_async_write_screen_info.ss.dual_ss.scroll_config.is_horizontal_scroll) {
            if(s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate > p_screen_info->scrn_pixel_width) {
               s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate = p_screen_info->scrn_pixel_width;
            }
        } else {
            if(s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate > p_screen_info->scrn_pixel_height) {
               s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate = p_screen_info->scrn_pixel_height;
            }
        }

        /* adjust scroll_coordinate to even number */
        s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate = (s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate >> 1) << 1;

        if(is_stored_in_qspi_storage) {
            app_qspi_mmap_set_prefetch(storage_id, true);
        }
        APP_ASSERT_CHECK(p_qspi_env[screen_id]->is_used_dma);
        app_qspi_switch_dma_mode(screen_id,    false);

        if(QSPI_DATASIZE_08_BITS == p_screen_cmd->data_size) {
            s_async_write_screen_info.llp_cfg_right_shift_bit = 0;
            s_async_write_screen_info.llp_cfg_ctrl_low        = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_BYTE, DMA_DDATAALIGN_BYTE, LL_DMA_SRC_BURST_LENGTH_8, LL_DMA_DST_BURST_LENGTH_8, DMA_SRC_GATHER_DISABLE);
            if(is_stored_in_qspi_storage) app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_0);
        } else if (QSPI_DATASIZE_16_BITS == p_screen_cmd->data_size) {
            s_async_write_screen_info.llp_cfg_right_shift_bit = 1;
            s_async_write_screen_info.llp_cfg_ctrl_low        = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_HALFWORD, DMA_DDATAALIGN_HALFWORD, LL_DMA_SRC_BURST_LENGTH_8, LL_DMA_DST_BURST_LENGTH_8, DMA_SRC_GATHER_DISABLE);
            if(is_stored_in_qspi_storage) app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_1);
        } else if (QSPI_DATASIZE_32_BITS == p_screen_cmd->data_size) {
            s_async_write_screen_info.llp_cfg_right_shift_bit = 2;
            if(is_stored_in_qspi_storage)
            {
                s_async_write_screen_info.llp_cfg_ctrl_low    = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_WORD, DMA_DDATAALIGN_WORD, LL_DMA_SRC_BURST_LENGTH_4, LL_DMA_DST_BURST_LENGTH_4, DMA_SRC_GATHER_DISABLE);//BALIPRO-226:LL_DMA_SRC_BURST_LENGTH_8 will cause exception for qspi-psram data
            }
            else
            {
                s_async_write_screen_info.llp_cfg_ctrl_low    = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_WORD, DMA_DDATAALIGN_WORD, LL_DMA_SRC_BURST_LENGTH_8, LL_DMA_DST_BURST_LENGTH_8, DMA_SRC_GATHER_DISABLE);
            }
            if(is_stored_in_qspi_storage) app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_2);
        } else {
            p_qspi_env[screen_id]->start_flag = false;
            if(storage_id <= APP_QSPI_ID_2) {
                p_qspi_env[storage_id]->start_flag = false;
            }
            s_async_write_screen_info.if_type = DRAW_TYPE_IF_NONE;
            return false;
        }

        s_async_write_screen_info.ss.dual_ss.image_1_ahb_address = p_scroll_config->first_frame_start_address;
        s_async_write_screen_info.ss.dual_ss.image_2_ahb_address = p_scroll_config->second_frame_start_address;
        s_async_write_screen_info.ss.dual_ss.total_sent_lines    = 0;
        s_async_write_screen_info.ss.dual_ss.this_send_lines     = 0;
        s_async_write_screen_info.ss.dual_ss.sent_line_order     = s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate;
    }

    const uint32_t scrn_pixel_height        = s_async_write_screen_info.screen_info.scrn_pixel_height;
    const uint32_t scrn_pixel_width         = s_async_write_screen_info.screen_info.scrn_pixel_width;
    const uint32_t scrn_pixel_depth         = s_async_write_screen_info.screen_info.scrn_pixel_depth;
    const uint32_t scrn_line_size           = (scrn_pixel_width * scrn_pixel_depth);
    const uint32_t scrn_refresh_lines_once  = (QSPI_MAX_XFER_SIZE_ONCE/scrn_line_size);
    const uint32_t send_lines_once          = DMA_LLP_BLOCKS_FOR_WRITE < scrn_refresh_lines_once ? DMA_LLP_BLOCKS_FOR_WRITE : scrn_refresh_lines_once;

    /* check screen height/width/depth */
    if((scrn_pixel_width % 2 != 0) ||
       (scrn_pixel_height % 2 != 0) ||
       (scrn_pixel_depth != 2) ) {
        p_qspi_env[screen_id]->start_flag = false;
        if(storage_id <= APP_QSPI_ID_2) {
            p_qspi_env[storage_id]->start_flag = false;
        }
        s_async_write_screen_info.if_type = DRAW_TYPE_IF_NONE;
        return false;
    }

    /* prepare llp config */
    scrn_llp_config.llp_src_writeback       = 1;
    scrn_llp_config.llp_dst_writeback       = 1;
    scrn_llp_config.llp_src_en              = DMA_LLP_SRC_ENABLE;
    scrn_llp_config.llp_dst_en              = DMA_LLP_DST_DISABLE;
    scrn_llp_config.head_lli                = &s_dma_llp_block[0];

    app_scrn_cmd.instruction                = s_async_write_screen_info.qspi_screen_command.instruction;
    app_scrn_cmd.instruction_size           = s_async_write_screen_info.qspi_screen_command.instruction_size;
    app_scrn_cmd.address                    = is_first_call ? s_async_write_screen_info.qspi_screen_command.leading_address : s_async_write_screen_info.qspi_screen_command.ongoing_address;
    app_scrn_cmd.address_size               = s_async_write_screen_info.qspi_screen_command.address_size;
    app_scrn_cmd.dummy_cycles               = s_async_write_screen_info.qspi_screen_command.dummy_cycles;
    app_scrn_cmd.data_size                  = s_async_write_screen_info.qspi_screen_command.data_size;
    app_scrn_cmd.instruction_address_mode   = s_async_write_screen_info.qspi_screen_command.instruction_address_mode;
    app_scrn_cmd.data_mode                  = s_async_write_screen_info.qspi_screen_command.data_mode;
    app_scrn_cmd.length                     = 0x00;
    app_scrn_cmd.clock_stretch_en           = 1;

    if(s_async_write_screen_info.ss.dual_ss.scroll_config.is_horizontal_scroll) {

        if((s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate == 0) ||                   /* Only left image */
           (s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate == scrn_pixel_width)) {    /* Only right image */

            if(0 == s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate) {
                image_start_address = s_async_write_screen_info.ss.dual_ss.image_1_ahb_address;
            } else {
                image_start_address = s_async_write_screen_info.ss.dual_ss.image_2_ahb_address;
            }

            if(s_async_write_screen_info.ss.dual_ss.total_sent_lines  < scrn_pixel_height) {
                s_async_write_screen_info.ss.dual_ss.this_send_lines = (scrn_pixel_height - s_async_write_screen_info.ss.dual_ss.total_sent_lines) < send_lines_once ? (scrn_pixel_height - s_async_write_screen_info.ss.dual_ss.total_sent_lines) : send_lines_once;

                memset(&s_dma_llp_block[0], 0, sizeof(dma_block_config_t) * (s_async_write_screen_info.ss.dual_ss.this_send_lines*2 + 1));

                for(j = 0; j < s_async_write_screen_info.ss.dual_ss.this_send_lines; j++) {
                    s_dma_llp_block[j].src_address    = image_start_address + (s_async_write_screen_info.ss.dual_ss.total_sent_lines + j) * scrn_line_size ;
                    s_dma_llp_block[j].dst_address    = 0;
                    s_dma_llp_block[j].p_lli          = &s_dma_llp_block[j + 1];
                    s_dma_llp_block[j].CTL_L          = s_async_write_screen_info.llp_cfg_ctrl_low;
                    s_dma_llp_block[j].CTL_H          = (uint32_t)(scrn_line_size >> s_async_write_screen_info.llp_cfg_right_shift_bit);
                    s_dma_llp_block[j].src_status     = 0x0;
                    s_dma_llp_block[j].dst_status     = 0x0;
                }
                s_dma_llp_block[j - 1].p_lli          = NULL;

                app_scrn_cmd.length = s_async_write_screen_info.ss.dual_ss.this_send_lines * scrn_line_size;

                if(s_async_write_screen_info.qspi_screen_command.is_one_take_cs) {
                    if(is_first_call) {
                        app_qspi_force_cs(screen_id, true);
                        ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, false);
                    } else {
                        ret = app_qspi_llp_transmit(screen_id, &scrn_llp_config, app_scrn_cmd.data_mode, app_scrn_cmd.length, false);
                    }
                } else {
                    ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, false);
                }

                if(!ret) {
                    goto __fail;
                }
            }
        } else {    /* part of left image, part of right image */

            if(s_async_write_screen_info.ss.dual_ss.total_sent_lines < scrn_pixel_height) {
                s_async_write_screen_info.ss.dual_ss.this_send_lines = (scrn_pixel_height - s_async_write_screen_info.ss.dual_ss.total_sent_lines) < send_lines_once ? (scrn_pixel_height - s_async_write_screen_info.ss.dual_ss.total_sent_lines) : send_lines_once;

                memset(&s_dma_llp_block[0], 0, sizeof(dma_block_config_t) * (s_async_write_screen_info.ss.dual_ss.this_send_lines*2 + 1));

                for(j = 0; j < s_async_write_screen_info.ss.dual_ss.this_send_lines; j++) {
                    /* first image */
                    s_dma_llp_block[2*j].src_address    = s_async_write_screen_info.ss.dual_ss.image_1_ahb_address + (s_async_write_screen_info.ss.dual_ss.total_sent_lines + j) * scrn_line_size + s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate * scrn_pixel_depth;
                    s_dma_llp_block[2*j].dst_address    = 0;
                    s_dma_llp_block[2*j].p_lli          = &s_dma_llp_block[2*j + 1];
                    s_dma_llp_block[2*j].CTL_L          = s_async_write_screen_info.llp_cfg_ctrl_low;
                    s_dma_llp_block[2*j].CTL_H          = (uint32_t)(((scrn_pixel_width - s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate)*scrn_pixel_depth) >> s_async_write_screen_info.llp_cfg_right_shift_bit);
                    s_dma_llp_block[2*j].src_status     = 0x0;
                    s_dma_llp_block[2*j].dst_status     = 0x0;

                    /* second image */
                    s_dma_llp_block[2*j + 1].src_address    = s_async_write_screen_info.ss.dual_ss.image_2_ahb_address + (s_async_write_screen_info.ss.dual_ss.total_sent_lines + j) * scrn_line_size ;
                    s_dma_llp_block[2*j + 1].dst_address    = 0;
                    s_dma_llp_block[2*j + 1].p_lli          = &s_dma_llp_block[2*j + 2];
                    s_dma_llp_block[2*j + 1].CTL_L          = s_async_write_screen_info.llp_cfg_ctrl_low;
                    s_dma_llp_block[2*j + 1].CTL_H          = (uint32_t)((s_async_write_screen_info.ss.dual_ss.scroll_config.scroll_coordinate * scrn_pixel_depth) >> s_async_write_screen_info.llp_cfg_right_shift_bit);
                    s_dma_llp_block[2*j + 1].src_status     = 0x0;
                    s_dma_llp_block[2*j + 1].dst_status     = 0x0;

                }
                s_dma_llp_block[2*j - 1].p_lli          = NULL;

                app_scrn_cmd.length = s_async_write_screen_info.ss.dual_ss.this_send_lines * scrn_line_size;

                if(s_async_write_screen_info.qspi_screen_command.is_one_take_cs) {
                    if(is_first_call) {
                        app_qspi_force_cs(screen_id, true);
                        ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, false);
                    } else {
                        ret = app_qspi_llp_transmit(screen_id, &scrn_llp_config, app_scrn_cmd.data_mode, app_scrn_cmd.length, false);
                    }
                } else {
                    ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, false);
                }

                if(!ret) {
                    goto __fail;
                }
            }
        }
    } else { /* Scroll vertically */

        if(s_async_write_screen_info.ss.dual_ss.total_sent_lines < scrn_pixel_height) {

            s_async_write_screen_info.ss.dual_ss.this_send_lines     = (scrn_pixel_height - s_async_write_screen_info.ss.dual_ss.total_sent_lines) < send_lines_once ? (scrn_pixel_height - s_async_write_screen_info.ss.dual_ss.total_sent_lines) : send_lines_once;

            memset(&s_dma_llp_block[0], 0, sizeof(dma_block_config_t) * (s_async_write_screen_info.ss.dual_ss.this_send_lines*2 + 1));

            for(j = 0; j < s_async_write_screen_info.ss.dual_ss.this_send_lines; j++) {

                image_start_address = s_async_write_screen_info.ss.dual_ss.sent_line_order < scrn_pixel_height ? s_async_write_screen_info.ss.dual_ss.image_1_ahb_address : s_async_write_screen_info.ss.dual_ss.image_2_ahb_address;

                s_dma_llp_block[j].src_address    = image_start_address + (s_async_write_screen_info.ss.dual_ss.sent_line_order % scrn_pixel_height) * scrn_line_size ;
                s_dma_llp_block[j].dst_address    = 0;
                s_dma_llp_block[j].p_lli          = &s_dma_llp_block[j + 1];
                s_dma_llp_block[j].CTL_L          = s_async_write_screen_info.llp_cfg_ctrl_low;
                s_dma_llp_block[j].CTL_H          = (uint32_t)(scrn_line_size >> s_async_write_screen_info.llp_cfg_right_shift_bit);
                s_dma_llp_block[j].src_status     = 0x0;
                s_dma_llp_block[j].dst_status     = 0x0;

                s_async_write_screen_info.ss.dual_ss.sent_line_order ++;
            }
            s_dma_llp_block[j - 1].p_lli          = NULL;

            app_scrn_cmd.length = s_async_write_screen_info.ss.dual_ss.this_send_lines * scrn_line_size;

            if(s_async_write_screen_info.qspi_screen_command.is_one_take_cs) {
                if(is_first_call) {
                    app_qspi_force_cs(screen_id, true);
                    ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, false);
                } else {
                    ret = app_qspi_llp_transmit(screen_id, &scrn_llp_config, app_scrn_cmd.data_mode, app_scrn_cmd.length, false);
                }
            } else {
                ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, false);
            }

            if(!ret) {
                goto __fail;
            }
        }
    }
    return true;

__fail:
    if(s_async_write_screen_info.qspi_screen_command.is_one_take_cs) {
        app_qspi_force_cs(s_async_write_screen_info.screen_id, false);
    }
    p_qspi_env[screen_id]->start_flag = false;
    if(storage_id <= APP_QSPI_ID_2) {
        p_qspi_env[storage_id]->start_flag = false;
    }
    s_async_write_screen_info.if_type = DRAW_TYPE_IF_NONE;
    return false;

#else
    return false;
#endif
}

bool app_qspi_async_veri_draw_screen(app_qspi_id_t screen_id,
                                          app_qspi_id_t storage_id,
                                          const app_qspi_screen_command_t * const p_screen_cmd,
                                          const app_qspi_screen_info_t * const p_screen_info,
                                          app_qspi_screen_veri_link_scroll_t * p_link_scroll,
                                          bool is_first_call) {

#if (QSPI_ASYNC_VERI_LINK_DRAW_SCREEN_SUPPORT > 0u)

    app_qspi_command_t          app_scrn_cmd;
    dma_llp_config_t            scrn_llp_config;
    bool ret                       = false;
    uint32_t j                     = 0;
    bool is_stored_in_qspi_storage = false;
    app_qspi_screen_veri_link_scroll_t * p_cur_scroll = NULL;

    if((p_screen_cmd == NULL) ||  (p_screen_info == NULL) || (p_link_scroll == NULL)) {
        return false;
    }

    if(screen_id > APP_QSPI_ID_2) {
        return false;
    }

    if((storage_id > APP_QSPI_ID_2) && (storage_id != APP_STORAGE_RAM_ID)) {
        return false;
    }

    if(screen_id == storage_id) {
        return false;
    }

    if ((p_qspi_env[screen_id] == NULL) || (p_qspi_env[screen_id]->qspi_state == APP_QSPI_INVALID) ||
        (p_qspi_env[storage_id] == NULL) || (p_qspi_env[storage_id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if(p_qspi_env[screen_id]->is_async_write_screen && is_first_call) {
        printf("RE-CALLED !\r\n");
        return false;
    }

    if(is_first_call) {

        if (p_qspi_env[screen_id]->start_flag) {
            printf("BUSY... !\r\n");
            return false;
        }
        p_qspi_env[screen_id]->start_flag = true;
        if(storage_id <= APP_QSPI_ID_2) {
            p_qspi_env[storage_id]->start_flag = true;
        }

        memset(&s_async_write_screen_info, 0, sizeof(app_qspi_async_draw_screen_info_t));

        is_stored_in_qspi_storage            = (storage_id <= APP_QSPI_ID_2) ? true : false;
        s_async_write_screen_info.if_type    = DRAW_TYPE_IF_VERI_LINKED_SCREEN;
        s_async_write_screen_info.screen_id  = screen_id;
        s_async_write_screen_info.storage_id = storage_id;

        memcpy(&s_async_write_screen_info.screen_info,  p_screen_info, sizeof(app_qspi_screen_info_t));
        memcpy(&s_async_write_screen_info.qspi_screen_command, p_screen_cmd , sizeof(app_qspi_command_t));
        memcpy(&s_async_write_screen_info.ss.veri_linked_ss.vl_scroll, p_link_scroll, sizeof(app_qspi_screen_veri_link_scroll_t));

        s_async_write_screen_info.ss.veri_linked_ss.total_sent_lines = 0;
        s_async_write_screen_info.ss.veri_linked_ss.p_cur_scroll     = p_link_scroll;

        if(is_stored_in_qspi_storage) {
            app_qspi_mmap_set_prefetch(storage_id, true);
        }
        APP_ASSERT_CHECK(p_qspi_env[screen_id]->is_used_dma);
        app_qspi_switch_dma_mode(screen_id,    false);

        if(QSPI_DATASIZE_08_BITS == p_screen_cmd->data_size) {
            s_async_write_screen_info.llp_cfg_right_shift_bit = 0;
            s_async_write_screen_info.llp_cfg_ctrl_low        = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_BYTE, DMA_DDATAALIGN_BYTE, LL_DMA_SRC_BURST_LENGTH_8, LL_DMA_DST_BURST_LENGTH_8, DMA_SRC_GATHER_DISABLE);
            if(is_stored_in_qspi_storage) app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_0);
        } else if (QSPI_DATASIZE_16_BITS == p_screen_cmd->data_size) {
            s_async_write_screen_info.llp_cfg_right_shift_bit = 1;
            s_async_write_screen_info.llp_cfg_ctrl_low        = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_HALFWORD, DMA_DDATAALIGN_HALFWORD, LL_DMA_SRC_BURST_LENGTH_8, LL_DMA_DST_BURST_LENGTH_8, DMA_SRC_GATHER_DISABLE);
            if(is_stored_in_qspi_storage) app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_1);
        } else if (QSPI_DATASIZE_32_BITS == p_screen_cmd->data_size) {
            s_async_write_screen_info.llp_cfg_right_shift_bit = 2;
            s_async_write_screen_info.llp_cfg_ctrl_low        = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_WORD, DMA_DDATAALIGN_WORD, LL_DMA_SRC_BURST_LENGTH_8, LL_DMA_DST_BURST_LENGTH_8, DMA_SRC_GATHER_DISABLE);
            if(is_stored_in_qspi_storage) app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_2);
        } else {
            p_qspi_env[screen_id]->start_flag = false;
            if(storage_id <= APP_QSPI_ID_2) {
                p_qspi_env[storage_id]->start_flag = false;
            }
            s_async_write_screen_info.if_type = DRAW_TYPE_IF_NONE;
            return false;
        }
    }

    const uint32_t scrn_pixel_height        = s_async_write_screen_info.screen_info.scrn_pixel_height;
    const uint32_t scrn_pixel_width         = s_async_write_screen_info.screen_info.scrn_pixel_width;
    const uint32_t scrn_pixel_depth         = s_async_write_screen_info.screen_info.scrn_pixel_depth;
    const uint32_t scrn_line_size           = (scrn_pixel_width * scrn_pixel_depth);
    const uint32_t scrn_refresh_lines_once  = (QSPI_MAX_XFER_SIZE_ONCE/scrn_line_size);
    const uint32_t max_lines                = (sizeof(s_dma_llp_block)/sizeof(dma_block_config_t) - 2);
    const uint32_t send_lines_once          = max_lines < scrn_refresh_lines_once ? max_lines : scrn_refresh_lines_once;

    p_cur_scroll = s_async_write_screen_info.ss.veri_linked_ss.p_cur_scroll;

    /* check screen height/width/depth */
    if((p_cur_scroll == NULL) ||
       (scrn_pixel_width % 2 != 0) ||
       (scrn_pixel_height % 2 != 0) ||
       (scrn_pixel_depth != 2) ) {
        p_qspi_env[screen_id]->start_flag = false;
        if(storage_id <= APP_QSPI_ID_2) {
            p_qspi_env[storage_id]->start_flag = false;
        }
        s_async_write_screen_info.if_type = DRAW_TYPE_IF_NONE;
        return false;
    }

    if(p_cur_scroll->frame_draw_lines > send_lines_once) {

        if(APP_QSPI_EXCEPT_DEBUG_EN) printf("+++ ERR: Set psram(frame_draw_lines) <= %d in each Link Point\r\n", send_lines_once);
        p_qspi_env[screen_id]->start_flag = false;
        if(storage_id <= APP_QSPI_ID_2) {
            p_qspi_env[storage_id]->start_flag = false;
        }
        s_async_write_screen_info.if_type = DRAW_TYPE_IF_NONE;
        return false;
    }

    /* prepare llp config */
    scrn_llp_config.llp_src_writeback       = 1;
    scrn_llp_config.llp_dst_writeback       = 1;
    scrn_llp_config.llp_src_en              = DMA_LLP_SRC_ENABLE;
    scrn_llp_config.llp_dst_en              = DMA_LLP_DST_DISABLE;
    scrn_llp_config.head_lli                = &s_dma_llp_block[0];

    app_scrn_cmd.instruction                = s_async_write_screen_info.qspi_screen_command.instruction;
    app_scrn_cmd.instruction_size           = s_async_write_screen_info.qspi_screen_command.instruction_size;
    app_scrn_cmd.address                    = s_async_write_screen_info.qspi_screen_command.leading_address;
    app_scrn_cmd.address_size               = s_async_write_screen_info.qspi_screen_command.address_size;
    app_scrn_cmd.dummy_cycles               = s_async_write_screen_info.qspi_screen_command.dummy_cycles;
    app_scrn_cmd.data_size                  = s_async_write_screen_info.qspi_screen_command.data_size;
    app_scrn_cmd.instruction_address_mode   = s_async_write_screen_info.qspi_screen_command.instruction_address_mode;
    app_scrn_cmd.data_mode                  = s_async_write_screen_info.qspi_screen_command.data_mode;
    app_scrn_cmd.length                     = 0x00;
    app_scrn_cmd.clock_stretch_en           = 1;

    memset(&s_dma_llp_block[0], 0, sizeof(dma_block_config_t) * (p_cur_scroll->frame_draw_lines + 1));

    for(j = 0; j < p_cur_scroll->frame_draw_lines; j++) {
        s_dma_llp_block[j].src_address    = p_cur_scroll->frame_ahb_start_address + (p_cur_scroll->frame_offset_lines + j) * scrn_line_size ;
        s_dma_llp_block[j].dst_address    = 0;
        s_dma_llp_block[j].p_lli          = &s_dma_llp_block[j + 1];
        s_dma_llp_block[j].CTL_L          = s_async_write_screen_info.llp_cfg_ctrl_low;
        s_dma_llp_block[j].CTL_H          = (uint32_t)(scrn_line_size >> s_async_write_screen_info.llp_cfg_right_shift_bit);
        s_dma_llp_block[j].src_status     = 0x0;
        s_dma_llp_block[j].dst_status     = 0x0;
    }
    s_dma_llp_block[j - 1].p_lli          = NULL;

    if(is_first_call) {
        app_scrn_cmd.address = s_async_write_screen_info.qspi_screen_command.leading_address;
    } else {
        app_scrn_cmd.address = s_async_write_screen_info.qspi_screen_command.ongoing_address;
    }

    app_scrn_cmd.length = p_cur_scroll->frame_draw_lines * scrn_line_size;

    if(s_async_write_screen_info.qspi_screen_command.is_one_take_cs) {
        if(is_first_call) {
            app_qspi_force_cs(screen_id, true);
            ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, false);
        } else {
            ret = app_qspi_llp_transmit(screen_id, &scrn_llp_config, app_scrn_cmd.data_mode, app_scrn_cmd.length, false);
        }
    } else {
        ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, false);
    }

    if(ret) {
        return true;
    } else {
        if(s_async_write_screen_info.qspi_screen_command.is_one_take_cs) {
            app_qspi_force_cs(s_async_write_screen_info.screen_id, false);
        }
        p_qspi_env[screen_id]->start_flag = false;
        if(storage_id <= APP_QSPI_ID_2) {
            p_qspi_env[storage_id]->start_flag = false;
        }
        s_async_write_screen_info.if_type = DRAW_TYPE_IF_NONE;
        return false;
    }
#else
    return false;
#endif
}

bool app_qspi_async_llp_draw_block(app_qspi_id_t screen_id,
                                         app_qspi_id_t storage_id,
                                         const app_qspi_screen_command_t *const p_screen_cmd,
                                         const app_qspi_screen_info_t *const p_screen_info,
                                         app_qspi_screen_block_t *p_block_info,
                                         bool is_first_call) {

#if (QSPI_ASYNC_VERI_LINK_DRAW_SCREEN_SUPPORT > 0u)

    app_qspi_command_t app_scrn_cmd;
    dma_llp_config_t scrn_llp_config;
    bool ret = false;
    uint32_t j = 0;
    bool is_stored_in_qspi_storage = false;
    app_qspi_screen_block_t *p_cur_block = NULL;
    if ((p_screen_cmd == NULL) || (p_screen_info == NULL) || (p_block_info == NULL)) {
        return false;
    }

    if (screen_id > APP_QSPI_ID_2) {
        return false;
    }

    if ((storage_id > APP_QSPI_ID_2) && (storage_id != APP_STORAGE_RAM_ID)) {
        return false;
    }

    if (screen_id == storage_id) {
        return false;
    }

    if ((p_qspi_env[screen_id] == NULL) || (p_qspi_env[screen_id]->qspi_state == APP_QSPI_INVALID) || (p_qspi_env[storage_id] == NULL) || (p_qspi_env[storage_id]->qspi_state == APP_QSPI_INVALID)) {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_qspi_env[screen_id]->start_flag) {
        printf("BUSY... !\r\n");
        return false;
    }

    p_qspi_env[screen_id]->start_flag = true;

    if (storage_id <= APP_QSPI_ID_2) {
        p_qspi_env[storage_id]->start_flag = true;
    }

    memset(&s_async_write_screen_info, 0, sizeof(app_qspi_async_draw_screen_info_t));

    is_stored_in_qspi_storage = (storage_id <= APP_QSPI_ID_2) ? true : false;

    s_async_write_screen_info.screen_id = screen_id;

    s_async_write_screen_info.storage_id = storage_id;

    memcpy(&s_async_write_screen_info.screen_info, p_screen_info, sizeof(app_qspi_screen_info_t));

    memcpy(&s_async_write_screen_info.qspi_screen_command, p_screen_cmd, sizeof(app_qspi_command_t));

    memcpy(&s_async_write_screen_info.ss.veri_linked_ss.vl_scroll, p_block_info, sizeof(app_qspi_screen_block_t));

    s_async_write_screen_info.ss.veri_linked_ss.total_sent_lines = 0;

    s_async_write_screen_info.ss.veri_linked_ss.p_cur_scroll = (app_qspi_screen_veri_link_scroll_t*)p_block_info;

    s_async_write_screen_info.if_type = DRAW_TYPE_IF_NONE;

    if (is_stored_in_qspi_storage) {
        app_qspi_mmap_set_prefetch(storage_id, true);
    }

    APP_ASSERT_CHECK(p_qspi_env[screen_id]->is_used_dma);
    app_qspi_switch_dma_mode(screen_id, false);

    if (QSPI_DATASIZE_08_BITS == p_screen_cmd->data_size) {
        s_async_write_screen_info.llp_cfg_right_shift_bit = 0;
        s_async_write_screen_info.llp_cfg_ctrl_low = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_BYTE, DMA_DDATAALIGN_BYTE, LL_DMA_SRC_BURST_LENGTH_8, LL_DMA_DST_BURST_LENGTH_8, DMA_SRC_GATHER_DISABLE);
        if (is_stored_in_qspi_storage)
            app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_0);
    }
    else if (QSPI_DATASIZE_16_BITS == p_screen_cmd->data_size) {
        s_async_write_screen_info.llp_cfg_right_shift_bit = 1;
        s_async_write_screen_info.llp_cfg_ctrl_low = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_HALFWORD, DMA_DDATAALIGN_HALFWORD, LL_DMA_SRC_BURST_LENGTH_8, LL_DMA_DST_BURST_LENGTH_8, DMA_SRC_GATHER_DISABLE);
        if (is_stored_in_qspi_storage)
            app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_1);
    }
    else if (QSPI_DATASIZE_32_BITS == p_screen_cmd->data_size) {
        s_async_write_screen_info.llp_cfg_right_shift_bit = 2;
        if(is_stored_in_qspi_storage)
        {
            s_async_write_screen_info.llp_cfg_ctrl_low    = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_WORD, DMA_DDATAALIGN_WORD, LL_DMA_SRC_BURST_LENGTH_4, LL_DMA_DST_BURST_LENGTH_4, DMA_SRC_GATHER_DISABLE);//BALIPRO-226:LL_DMA_SRC_BURST_LENGTH_8 will cause exception for qspi-psram data
        }
        else
        {
            s_async_write_screen_info.llp_cfg_ctrl_low    = QSPI_DMA_CRTL_LOW_REGISTER_CFG(DMA_SRC_INCREMENT, DMA_SDATAALIGN_WORD, DMA_DDATAALIGN_WORD, LL_DMA_SRC_BURST_LENGTH_8, LL_DMA_DST_BURST_LENGTH_8, DMA_SRC_GATHER_DISABLE);
        }
        if (is_stored_in_qspi_storage)
            app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_2);
    }
    else {
        p_qspi_env[screen_id]->start_flag = false;
        if (storage_id <= APP_QSPI_ID_2) {
            p_qspi_env[storage_id]->start_flag = false;
        }
        return false;
    }

    const uint32_t scrn_pixel_height = s_async_write_screen_info.screen_info.scrn_pixel_height;
    const uint32_t scrn_pixel_width = s_async_write_screen_info.screen_info.scrn_pixel_width;
    const uint32_t scrn_pixel_depth = s_async_write_screen_info.screen_info.scrn_pixel_depth;
    const uint32_t scrn_line_size = (scrn_pixel_width * scrn_pixel_depth);
    const uint32_t scrn_refresh_lines_once = (QSPI_MAX_XFER_SIZE_ONCE / scrn_line_size);
    const uint32_t max_lines = (sizeof(s_dma_llp_block) / sizeof(dma_block_config_t) -2);
    const uint32_t send_lines_once = max_lines < scrn_refresh_lines_once ? max_lines : scrn_refresh_lines_once;
    p_cur_block = p_block_info;

    /* check screen height/width/depth */
    if ((p_cur_block == NULL) || (scrn_pixel_width % 2 != 0) || (scrn_pixel_height % 2 != 0) || (scrn_pixel_depth != 2)) {
        p_qspi_env[screen_id]->start_flag = false;
        if (storage_id <= APP_QSPI_ID_2)
        {
            p_qspi_env[storage_id]->start_flag = false;
        }
        return false;
    }

    if (p_cur_block->frame_draw_lines > send_lines_once) {
        if (APP_QSPI_EXCEPT_DEBUG_EN)
            printf("+++ ERR: Set psram(frame_draw_lines) <= %d in each Link Point\r\n", send_lines_once);

        p_qspi_env[screen_id]->start_flag = false;

        if (storage_id <= APP_QSPI_ID_2) {
            p_qspi_env[storage_id]->start_flag = false;
        }

        return false;
    }

    /* prepare llp config */
    scrn_llp_config.llp_src_writeback = 1;
    scrn_llp_config.llp_dst_writeback = 1;
    scrn_llp_config.llp_src_en = DMA_LLP_SRC_ENABLE;
    scrn_llp_config.llp_dst_en = DMA_LLP_DST_DISABLE;
    scrn_llp_config.head_lli = &s_dma_llp_block[0];

    app_scrn_cmd.instruction = s_async_write_screen_info.qspi_screen_command.instruction;
    app_scrn_cmd.instruction_size = s_async_write_screen_info.qspi_screen_command.instruction_size;
    app_scrn_cmd.address = s_async_write_screen_info.qspi_screen_command.leading_address;
    app_scrn_cmd.address_size = s_async_write_screen_info.qspi_screen_command.address_size;
    app_scrn_cmd.dummy_cycles = s_async_write_screen_info.qspi_screen_command.dummy_cycles;
    app_scrn_cmd.data_size = s_async_write_screen_info.qspi_screen_command.data_size;
    app_scrn_cmd.instruction_address_mode = s_async_write_screen_info.qspi_screen_command.instruction_address_mode;
    app_scrn_cmd.data_mode = s_async_write_screen_info.qspi_screen_command.data_mode;
    app_scrn_cmd.length = 0x00;
    app_scrn_cmd.clock_stretch_en = 1;

    memset(&s_dma_llp_block[0],0, sizeof(dma_block_config_t) * (p_cur_block->frame_draw_lines +1));

    for (j = 0; j < p_cur_block->frame_draw_lines; j++) {
        s_dma_llp_block[j].src_address = p_cur_block->frame_ahb_start_address + (p_cur_block->frame_offset_lines + j) * scrn_line_size;
        s_dma_llp_block[j].dst_address = 0;
        s_dma_llp_block[j].p_lli = &s_dma_llp_block[j +1];
        s_dma_llp_block[j].CTL_L = s_async_write_screen_info.llp_cfg_ctrl_low;
        s_dma_llp_block[j].CTL_H = (uint32_t)(scrn_line_size >> s_async_write_screen_info.llp_cfg_right_shift_bit);
        s_dma_llp_block[j].src_status = 0x0;
        s_dma_llp_block[j].dst_status = 0x0;
    }
    s_dma_llp_block[j - 1].p_lli = NULL;

    if (is_first_call) {
        app_scrn_cmd.address = s_async_write_screen_info.qspi_screen_command.leading_address;
    }
    else {
        app_scrn_cmd.address = s_async_write_screen_info.qspi_screen_command.ongoing_address;
    }

    app_scrn_cmd.length = p_cur_block->frame_draw_lines * scrn_line_size;

    if (s_async_write_screen_info.qspi_screen_command.is_one_take_cs) {
        if (is_first_call) {
            ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, false);
        }
        else {
            ret = app_qspi_llp_transmit(screen_id, &scrn_llp_config, app_scrn_cmd.data_mode, app_scrn_cmd.length, false);
        }
    }
    else {
        ret = app_qspi_cmd_llp_transmit(screen_id, &app_scrn_cmd, &scrn_llp_config, false);
    }

    if (ret) {
        return true;
    }
    else {
        p_qspi_env[screen_id]->start_flag = false;

        if (storage_id <= APP_QSPI_ID_2) {
            p_qspi_env[storage_id]->start_flag = false;
        }

        return false;
    }
#else
    return false;
#endif
}


__weak void * _q_malloc(uint32_t size) {
    // TODO: override this function
    return NULL;
}

__weak void _q_free(void * ptr) {
    // TODO: override this function
    return;
}

static dma_block_config_t * p_llp = NULL;
static app_qspi_id_t s_screen_id ;

void _free_qspi_dma_llp_resource(void) {
    if (p_llp != NULL)
    {
        dma_block_config_t * p = p_llp;
        dma_block_config_t * q = NULL;
        while (p)
        {
            q = p->p_lli;
            _q_free(p);
            p = q;
        }

        p_qspi_env[s_screen_id]->start_flag = false;
        app_qspi_force_cs(s_screen_id, false);
    }
    p_llp = NULL;
}

/****************************************************************
 * IF Flush Area <= Frame Buffer :
 *
 *      ---------------------[stride]------------------
 *      |    [p_buff]                                 |
 *      |    ++++++++++++++[width]++++++++++++        |
 *      |    +                               +        |
 *      |    +                               +        |
 *      |    +          Flush Area        [height]    |
 *      |    +                               +        |
 *      |    +                               +        |
 *      |    +++++++++++++++++++++++++++++++++        |
 *      |                                             |
 *      -----------------------------------------------
 *                      Frame Buffer
 ****************************************************************/
uint16_t app_qspi_send_display_frame(app_qspi_id_t screen_id,
                                     const app_qspi_screen_command_t *const p_screen_cmd,
                                     const app_qspi_screen_info_t *const p_screen_info, const uint8_t * p_buff) {

    uint32_t shift_bit   = 0;
    uint32_t xfer_width  = 0;
    uint32_t i           = 0;
    uint32_t block_count = 0;
    uint32_t block_left  = 0;
    uint32_t block_beat  = 0;
    hal_status_t status  = HAL_ERROR;

    if ((p_screen_cmd == NULL) || (p_screen_info == NULL) || (p_buff == NULL)) {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if(p_screen_info->scrn_pixel_width > p_screen_info->scrn_pixel_stride) {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    dma_block_config_t * p_llp_block      = NULL;
    dma_block_config_t * p_llp_block_prev = NULL;

    dma_llp_config_t  s_llp_config = {
        .llp_src_en = DMA_LLP_SRC_ENABLE,
        .llp_dst_en = DMA_LLP_DST_DISABLE,
        .head_lli = NULL,
    };

    qspi_command_t s_cmd = {
        .instruction              = p_screen_cmd->instruction,
        .address                  = p_screen_cmd->leading_address,
        .instruction_size         = p_screen_cmd->instruction_size,
        .address_size             = p_screen_cmd->address_size,
        .dummy_cycles             = p_screen_cmd->dummy_cycles,
        .data_size                = p_screen_cmd->data_size,
        .instruction_address_mode = p_screen_cmd->instruction_address_mode,
        .data_mode                = p_screen_cmd->data_mode,
        .length                   = p_screen_info->scrn_pixel_width * p_screen_info->scrn_pixel_height * p_screen_info->scrn_pixel_depth,
        .clock_stretch_en         = false,
    };

    if(p_screen_cmd->data_size == QSPI_DATASIZE_08_BITS) {
        xfer_width = DMA_SDATAALIGN_BYTE | DMA_DDATAALIGN_BYTE ;
        shift_bit  = 0;
    } else if(p_screen_cmd->data_size == QSPI_DATASIZE_16_BITS) {
        xfer_width = DMA_SDATAALIGN_HALFWORD | DMA_DDATAALIGN_HALFWORD ;
        shift_bit  = 1;
    } else if(p_screen_cmd->data_size == QSPI_DATASIZE_32_BITS) {
        if(s_cmd.length % 4 != 0) {
            printf("Not Support length: %d\r\n", s_cmd.length);
            return APP_DRV_ERR_INVALID_PARAM;
        }
        xfer_width = DMA_SDATAALIGN_WORD | DMA_DDATAALIGN_WORD ;
        shift_bit  = 2;
    } else {
        printf("Not Support data_size: %d\r\n", p_screen_cmd->data_size);
        return APP_DRV_ERR_INVALID_PARAM;
    }

    uint32_t stride_size = 0;
    const uint32_t total_beats = (p_screen_info->scrn_pixel_width * p_screen_info->scrn_pixel_height * p_screen_info->scrn_pixel_depth) >> shift_bit;

    if(p_screen_info->scrn_pixel_width == p_screen_info->scrn_pixel_stride)
    {
        block_count = total_beats / 4092;
        block_left  = total_beats % 4092;
        block_beat  = 4092;
        stride_size = block_beat << shift_bit;
    } else {
        block_count = p_screen_info->scrn_pixel_height;
        block_left  = 0;
        block_beat  = (p_screen_info->scrn_pixel_width * p_screen_info->scrn_pixel_depth) >> shift_bit;
        stride_size = p_screen_info->scrn_pixel_stride * p_screen_info->scrn_pixel_depth;
    }

    uint32_t src_addr = (uint32_t) p_buff;

    for(i = 0; i < block_count; i++) {
        p_llp_block = _q_malloc(sizeof(dma_block_config_t));

        p_llp_block->src_address = src_addr;
        p_llp_block->dst_address = 0;
        p_llp_block->src_status  = 0x00;
        p_llp_block->dst_status  = 0x00;

        /* memset in word mode */
        p_llp_block->CTL_L       = DMA_CTLL_INI_EN
                                    | DMA_SRC_INCREMENT
                                    | DMA_DST_NO_CHANGE
                                    | DMA_SRC_GATHER_DISABLE
                                    | DMA_DST_SCATTER_DISABLE
                                    | DMA_LLP_SRC_ENABLE
                                    | DMA_LLP_DST_DISABLE
                                    | DMA_MEMORY_TO_PERIPH
                                    | LL_DMA_SRC_BURST_LENGTH_8 | LL_DMA_DST_BURST_LENGTH_8
                                    | xfer_width;

        p_llp_block->CTL_H       = block_beat;
        p_llp_block->p_lli       = NULL;

        if(p_llp_block_prev == NULL) {
            s_llp_config.head_lli = p_llp_block;
        } else {
            p_llp_block_prev->p_lli = p_llp_block;
        }
        p_llp_block_prev = p_llp_block;

        src_addr += stride_size;
    }

    if(block_left > 0) {
        p_llp_block = _q_malloc(sizeof(dma_block_config_t));

        p_llp_block->src_address = src_addr;
        p_llp_block->dst_address = 0;
        p_llp_block->src_status  = 0x00;
        p_llp_block->dst_status  = 0x00;

        /* memset in word mode */
        p_llp_block->CTL_L       = DMA_CTLL_INI_EN
                                    | DMA_SRC_INCREMENT
                                    | DMA_DST_NO_CHANGE
                                    | DMA_SRC_GATHER_DISABLE
                                    | DMA_DST_SCATTER_DISABLE
                                    | DMA_LLP_SRC_ENABLE
                                    | DMA_LLP_DST_DISABLE
                                    | DMA_MEMORY_TO_PERIPH
                                    | LL_DMA_SRC_BURST_LENGTH_8 | LL_DMA_DST_BURST_LENGTH_8
                                    | xfer_width;

        p_llp_block->CTL_H       = block_left;
        p_llp_block->p_lli       = NULL;

        if(p_llp_block_prev == NULL) {
            s_llp_config.head_lli = p_llp_block;
        } else {
            p_llp_block_prev->p_lli = p_llp_block;
        }
        p_llp_block_prev = p_llp_block;
    }

    p_llp = s_llp_config.head_lli;
    s_screen_id = screen_id;
    if (p_qspi_env[screen_id]->start_flag == false)
    {
        p_qspi_env[screen_id]->start_flag = true;
        app_qspi_force_cs(screen_id, true);
        status = hal_qspi_command_transmit_dma_llp(&(p_qspi_env[screen_id]->handle), &s_cmd, &s_llp_config);
        if (status != HAL_OK)
        {
            _free_qspi_dma_llp_resource();
            p_qspi_env[screen_id]->start_flag = false;
            return (uint16_t)status;
        }
    }
    else
    {
        _free_qspi_dma_llp_resource();
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}

/*
 * if p_screen_info->scrn_pixel_width == p_screen_info->scrn_pixel_stride, Please use this API.
 * This API costs less time than app_qspi_send_display_frame
 */
SECTION_RAM_CODE uint16_t app_qspi_send_display_frame_simp(app_qspi_id_t screen_id,
                                     const app_qspi_screen_command_t *const p_screen_cmd,
                                     const app_qspi_screen_info_t *const p_screen_info, const uint8_t * p_buff) {
    uint32_t i           = 0;
    hal_status_t status  = HAL_ERROR;
    uint32_t beat_bytes  = 1;
    uint32_t xfer_width  = DMA_SDATAALIGN_BYTE | DMA_DDATAALIGN_BYTE;

    dma_llp_config_t  s_llp_config = {
        .llp_src_en = DMA_LLP_SRC_ENABLE,
        .llp_dst_en = DMA_LLP_DST_DISABLE,
        .head_lli = NULL,
    };

    if ((p_screen_cmd == NULL) || (p_screen_info == NULL) || (p_buff == NULL)) {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if(p_screen_cmd->data_size == QSPI_DATASIZE_08_BITS) {
        beat_bytes  = 1;
        xfer_width  = DMA_SDATAALIGN_BYTE | DMA_DDATAALIGN_BYTE | LL_DMA_SRC_BURST_LENGTH_8 | LL_DMA_DST_BURST_LENGTH_8;
    } else if(p_screen_cmd->data_size == QSPI_DATASIZE_16_BITS) {
        beat_bytes  = 2;
        xfer_width  = DMA_SDATAALIGN_HALFWORD | DMA_DDATAALIGN_HALFWORD | LL_DMA_SRC_BURST_LENGTH_8 | LL_DMA_DST_BURST_LENGTH_8;
    } else if(p_screen_cmd->data_size == QSPI_DATASIZE_32_BITS) {
        beat_bytes  = 4;
        xfer_width  = DMA_SDATAALIGN_WORD | DMA_DDATAALIGN_WORD | LL_DMA_SRC_BURST_LENGTH_4 | LL_DMA_DST_BURST_LENGTH_4;
    }

    const uint32_t _xfer_lines_once  = (beat_bytes*4092)/(p_screen_info->scrn_pixel_width * p_screen_info->scrn_pixel_depth);
    const uint32_t _xfer_times_once  = p_screen_info->scrn_pixel_height/_xfer_lines_once + ((p_screen_info->scrn_pixel_height%_xfer_lines_once !=0) ? 1 : 0);
    const uint32_t _xfer_bytes_once  = _xfer_lines_once * p_screen_info->scrn_pixel_width * p_screen_info->scrn_pixel_depth;
    const uint32_t _xfer_bytes_total = p_screen_info->scrn_pixel_width * p_screen_info->scrn_pixel_depth * p_screen_info->scrn_pixel_height;
          uint32_t _xfer_bytes_left  = _xfer_bytes_total;
          uint32_t _xfer_bytes_this  = 0;
    memset(&s_dma_llp_block[0], 0, sizeof(dma_block_config_t)*_xfer_times_once);

    for(i = 0; i < _xfer_times_once; i++) {
        _xfer_bytes_this = (_xfer_bytes_left > _xfer_bytes_once) ? _xfer_bytes_once : _xfer_bytes_left;
        s_dma_llp_block[i].src_address = (uint32_t) p_buff + i * _xfer_bytes_once;
        s_dma_llp_block[i].dst_address = 0;
        s_dma_llp_block[i].src_status  = 0x00;
        s_dma_llp_block[i].dst_status  = 0x00;

        /* memset in word mode */
        s_dma_llp_block[i].CTL_L = DMA_CTLL_INI_EN
                                 | DMA_SRC_INCREMENT
                                 | DMA_DST_NO_CHANGE
                                 | DMA_SRC_GATHER_DISABLE
                                 | DMA_DST_SCATTER_DISABLE
                                 | DMA_LLP_SRC_ENABLE
                                 | DMA_LLP_DST_DISABLE
                                 | DMA_MEMORY_TO_PERIPH
                                 | xfer_width;
        s_dma_llp_block[i].CTL_H       = _xfer_bytes_this/beat_bytes;
        s_dma_llp_block[i].p_lli       = &s_dma_llp_block[i+1];
        _xfer_bytes_left -= _xfer_bytes_this;
    }
    s_dma_llp_block[i-1].p_lli       = NULL;

    qspi_command_t s_cmd = {
        .instruction              = p_screen_cmd->instruction,
        .address                  = p_screen_cmd->leading_address,
        .instruction_size         = p_screen_cmd->instruction_size,
        .address_size             = p_screen_cmd->address_size,
        .dummy_cycles             = p_screen_cmd->dummy_cycles,
        .data_size                = p_screen_cmd->data_size,
        .instruction_address_mode = p_screen_cmd->instruction_address_mode,
        .data_mode                = p_screen_cmd->data_mode,
        .length                   = _xfer_bytes_total,
        .clock_stretch_en         = true,
    };

    s_screen_id = screen_id;
    s_llp_config.head_lli = &s_dma_llp_block[0];
    if (p_qspi_env[screen_id]->start_flag == false)
    {
        p_qspi_env[screen_id]->start_flag = true;

        status = hal_qspi_command_transmit_dma_llp(&(p_qspi_env[screen_id]->handle), &s_cmd, &s_llp_config);
        if (status != HAL_OK)
        {
            p_qspi_env[screen_id]->start_flag = false;
            return (uint16_t)status;
        }
    }
    else
    {
        return APP_DRV_ERR_BUSY;
    }

    return APP_DRV_SUCCESS;
}



bool app_qspi_mmap_blit_image(app_qspi_id_t storage_id, blit_image_config_t * p_blit_config, blit_xfer_type_e xfer_type) {
#if QSPI_BLIT_RECT_IMAGE_SUPPORT > 0u
    bool ret                 = true;
    bool is_dma_sg_xfer      = false;
    hal_status_t status      = HAL_OK;
    uint32_t sent_lines      = 0;
    uint32_t this_sent_lines = 0;
    uint32_t src_addr        = 0;
    uint32_t dst_addr        = 0;
    dma_sg_llp_config_t     sg_llp_config;

    if ((p_qspi_env[storage_id] == NULL) || (p_qspi_env[storage_id]->qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if(p_blit_config == NULL) {
        return false;
    }

    if(p_blit_config->src_img_x + p_blit_config->src_img_x_delta > p_blit_config->src_img_w) {
        return false;
    }

    if(p_blit_config->src_img_y + p_blit_config->src_img_y_delta > p_blit_config->src_img_h) {
        return false;
    }

    const uint32_t src_image_address = ll_qspi_get_xip_base_address((qspi_regs_t*)s_qspi_instance[storage_id]) + p_blit_config->src_img_address;
    const uint32_t src_start_address = src_image_address + (p_blit_config->src_img_y * p_blit_config->src_img_w + p_blit_config->src_img_x) * p_blit_config->pixel_depth;
    const uint32_t dst_start_address = p_blit_config->dst_buff_address + (p_blit_config->dst_buff_y * p_blit_config->dst_buff_width + p_blit_config->dst_buff_x) * p_blit_config->pixel_depth;
    const uint32_t line_length       = p_blit_config->src_img_x_delta * p_blit_config->pixel_depth;
    const uint32_t send_lines_once   = DMA_MAX_XFER_SIZE_ONCE/(line_length);
    const uint32_t total_lines       = p_blit_config->src_img_y_delta;

    APP_ASSERT_CHECK(p_qspi_env[storage_id]->is_used_dma);
    app_qspi_mmap_set_prefetch(storage_id, true);
    app_qspi_switch_dma_mode(storage_id,   true);

    if(BLIT_BY_DMA_SG == xfer_type) {

        is_dma_sg_xfer = true;

        if(is_dma_sg_xfer) {
            /* source - gather */
            sg_llp_config.gather_config.src_sgc         = p_blit_config->src_img_x_delta * p_blit_config->pixel_depth;
            sg_llp_config.gather_config.src_sgi         = (p_blit_config->src_img_w - p_blit_config->src_img_x_delta) * p_blit_config->pixel_depth;
            sg_llp_config.gather_config.src_gather_en   = DMA_SRC_GATHER_ENABLE;

            /* dest - scatter */
            sg_llp_config.scatter_config.dst_dsc        = p_blit_config->src_img_x_delta * p_blit_config->pixel_depth;
            sg_llp_config.scatter_config.dst_dsi        = (p_blit_config->dst_buff_width - p_blit_config->src_img_x_delta) * p_blit_config->pixel_depth;
            sg_llp_config.scatter_config.dst_scatter_en = DMA_DST_SCATTER_ENABLE;

            sg_llp_config.llp_config.head_lli           = NULL;
            sg_llp_config.llp_config.llp_src_en         = DMA_LLP_SRC_DISABLE;
            sg_llp_config.llp_config.llp_dst_en         = DMA_LLP_DST_DISABLE;
            sg_llp_config.llp_config.llp_src_writeback  = 0;
            sg_llp_config.llp_config.llp_dst_writeback  = 0;
        }

        ret        = true;
        sent_lines = 0;
        src_addr   = src_start_address;
        dst_addr   = dst_start_address;
        while(sent_lines < total_lines) {

            this_sent_lines = (total_lines - sent_lines) < send_lines_once ? (total_lines - sent_lines) : send_lines_once;

            p_qspi_env[storage_id]->is_dma_done = 0;
            p_qspi_env[storage_id]->is_xfer_err = 0;

            status = hal_dma_start_sg_llp_it(p_qspi_env[storage_id]->handle.p_dma, src_addr, dst_addr, this_sent_lines*line_length, &sg_llp_config);

            if(status == HAL_OK) {
                while(!p_qspi_env[storage_id]->is_dma_done && !p_qspi_env[storage_id]->is_xfer_err);
                if(p_qspi_env[storage_id]->is_xfer_err) {
                    ret = false;
                }
            } else {
                ret = false;
            }

            if(!ret) {
                break;
            }

            sent_lines += this_sent_lines;
            src_addr   += this_sent_lines*p_blit_config->src_img_w*p_blit_config->pixel_depth;
            dst_addr   += this_sent_lines*p_blit_config->dst_buff_width*p_blit_config->pixel_depth;
        }
    } else if(BLIT_BY_DMA_LLP == xfer_type) {

#if (QSPI_DMA_LLP_FEATUTE_SUPPORT > 0u)

        static dma_block_config_t  dma_llp_block[DMA_LLP_BLOCKS_FOR_BLIT + 1];

        const uint32_t sent_blocks_once = DMA_LLP_BLOCKS_FOR_BLIT;
        uint32_t i                      = 0;

        app_qspi_mmap_set_endian_mode(storage_id, APP_QSPI_MMAP_ENDIAN_MODE_0);

        sg_llp_config.gather_config.src_gather_en   = DMA_SRC_GATHER_DISABLE;
        sg_llp_config.scatter_config.dst_scatter_en = DMA_DST_SCATTER_DISABLE;
        sg_llp_config.llp_config.llp_src_en         = DMA_LLP_SRC_ENABLE;
        sg_llp_config.llp_config.llp_dst_en         = DMA_LLP_DST_ENABLE;
        sg_llp_config.llp_config.llp_src_writeback  = 1;
        sg_llp_config.llp_config.llp_dst_writeback  = 1;
        sg_llp_config.llp_config.head_lli           = &dma_llp_block[0];

        sent_lines = 0;
        while(sent_lines < total_lines) {
            this_sent_lines = (total_lines - sent_lines) < sent_blocks_once ? (total_lines - sent_lines) : sent_blocks_once;

            memset(&dma_llp_block[0], 0, sizeof(dma_block_config_t) * (sent_blocks_once + 1));
            for(i = 0; i < this_sent_lines; i++) {
                dma_llp_block[i].src_address    = src_start_address + (sent_lines + i) * p_blit_config->src_img_w*p_blit_config->pixel_depth;
                dma_llp_block[i].dst_address    = dst_start_address + (sent_lines + i) * p_blit_config->dst_buff_width*p_blit_config->pixel_depth;
                dma_llp_block[i].p_lli          = &dma_llp_block[i + 1];
                dma_llp_block[i].CTL_L          = DMA_CTLL_INI_EN | DMA_MEMORY_TO_MEMORY | DMA_LLP_SRC_ENABLE | \
                                                  DMA_LLP_DST_ENABLE | DMA_SRC_GATHER_DISABLE | DMA_DST_SCATTER_DISABLE | \
                                                  DMA_DST_INCREMENT | DMA_SRC_INCREMENT | DMA_SDATAALIGN_BYTE | \
                                                  DMA_DDATAALIGN_BYTE | LL_DMA_SRC_BURST_LENGTH_8 | LL_DMA_DST_BURST_LENGTH_8 ;
                dma_llp_block[i].CTL_H          = (uint32_t)(line_length);
                dma_llp_block[i].src_status     = 0x0;
                dma_llp_block[i].dst_status     = 0x0;
            }

            dma_llp_block[i - 1].p_lli = NULL;

            p_qspi_env[storage_id]->is_dma_done = 0;
            p_qspi_env[storage_id]->is_xfer_err = 0;

            status = hal_dma_start_sg_llp_it(p_qspi_env[storage_id]->handle.p_dma, src_start_address, dst_start_address, this_sent_lines*line_length, &sg_llp_config);

            if(status == HAL_OK) {
                while(!p_qspi_env[storage_id]->is_dma_done && !p_qspi_env[storage_id]->is_xfer_err);
                if(p_qspi_env[storage_id]->is_xfer_err) {
                    ret = false;
                }
            } else {
                ret = false;
            }

            if(!ret) {
                break;
            }

            sent_lines += this_sent_lines;
        }
#else
        ret = false;
#endif
    }

    return ret;

#else /* QSPI_BLIT_RECT_IMAGE_SUPPORT */

    return false;

#endif /* QSPI_BLIT_RECT_IMAGE_SUPPORT */
}

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

static bool app_qspi_switch_dma_mode(app_qspi_id_t id, bool is_m2m_mode) {
    app_dma_params_t dma_params = {0};

    if(p_qspi_env[id]->is_dma_mode_m2m == is_m2m_mode) {
        return true;
    } else {
        if(is_m2m_mode) {
            dma_params.p_instance                 = p_qspi_env[id]->dma_cfg.dma_instance;
            dma_params.channel_number             = p_qspi_env[id]->dma_cfg.dma_channel;
            dma_params.init.direction             = DMA_MEMORY_TO_MEMORY;
            dma_params.init.src_increment         = DMA_SRC_INCREMENT;
            dma_params.init.dst_increment         = DMA_DST_INCREMENT;
            dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
            dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
            dma_params.init.mode                  = DMA_NORMAL;
            dma_params.init.priority              = DMA_PRIORITY_LOW;
        } else {
            dma_params.p_instance                 = p_qspi_env[id]->dma_cfg.dma_instance;
            dma_params.channel_number             = p_qspi_env[id]->dma_cfg.dma_channel;
            dma_params.init.direction             = DMA_MEMORY_TO_PERIPH;
            dma_params.init.src_increment         = DMA_SRC_INCREMENT;
            dma_params.init.dst_increment         = DMA_DST_NO_CHANGE;
            dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
            dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
            dma_params.init.mode                  = DMA_NORMAL;
            dma_params.init.priority              = DMA_PRIORITY_LOW;
        }

        p_qspi_env[id]->dma_id = app_dma_init(&dma_params, s_dma_evt_handler[id]);
        if (p_qspi_env[id]->dma_id < 0)
        {
            return false;
        }
        p_qspi_env[id]->handle.p_dma = app_dma_get_handle(p_qspi_env[id]->dma_id);
        p_qspi_env[id]->handle.p_dma->p_parent = (void*)&p_qspi_env[id]->handle;
    }

    p_qspi_env[id]->is_dma_mode_m2m = is_m2m_mode;
    return true;
}

#if (QSPI_DMA_LLP_FEATUTE_SUPPORT > 0u)

static bool app_qspi_cmd_llp_transmit(app_qspi_id_t screen_id, app_qspi_command_t * p_cmd, dma_llp_config_t * p_llp_config, bool is_sync) {
    hal_status_t status = HAL_OK;

    if(is_sync) {   /* sync mode  */
        p_qspi_env[screen_id]->is_async_write_screen = false;
        p_qspi_env[screen_id]->is_tx_done  = 0;
        p_qspi_env[screen_id]->is_xfer_err = 0;
        status = hal_qspi_command_transmit_dma_llp(&(p_qspi_env[screen_id]->handle), p_cmd, p_llp_config);

        if(HAL_OK == status) {
            while(!p_qspi_env[screen_id]->is_tx_done && !p_qspi_env[screen_id]->is_xfer_err);

            if(p_qspi_env[screen_id]->is_xfer_err) {
                return false;
            }
            return true;
        }
    } else {   /* async mode  */
        p_qspi_env[screen_id]->is_async_write_screen = true;
        status = hal_qspi_command_transmit_dma_llp(&(p_qspi_env[screen_id]->handle), p_cmd, p_llp_config);

        if(HAL_OK == status) {
            return true;
        } else {
            p_qspi_env[screen_id]->is_async_write_screen = false;
        }
    }

    return false;
}
static bool app_qspi_llp_transmit(app_qspi_id_t screen_id, dma_llp_config_t * p_llp_config, uint32_t data_mode, uint32_t data_len, bool is_sync) {
    hal_status_t status = HAL_OK;

    if(is_sync) {   /* sync mode  */
        p_qspi_env[screen_id]->is_async_write_screen = false;
        p_qspi_env[screen_id]->is_tx_done  = 0;
        p_qspi_env[screen_id]->is_xfer_err = 0;
        status = hal_qspi_transmit_dma_llp(&(p_qspi_env[screen_id]->handle), p_llp_config, data_mode, data_len, true);

        if(HAL_OK == status) {
            while(!p_qspi_env[screen_id]->is_tx_done && !p_qspi_env[screen_id]->is_xfer_err);

            if(p_qspi_env[screen_id]->is_xfer_err) {
                return false;
            }
            return true;
        }
    } else {   /* async mode  */
        p_qspi_env[screen_id]->is_async_write_screen = true;
        status = hal_qspi_transmit_dma_llp(&(p_qspi_env[screen_id]->handle), p_llp_config, data_mode, data_len, true);

        if(HAL_OK == status) {
            return true;
        } else {
            p_qspi_env[screen_id]->is_async_write_screen = false;
        }
    }

    return false;
}

bool app_graphics_qspi_draw_screen_continue(app_qspi_id_t id, app_qspi_evt_t qspi_evt)
//if return true, you immediately should return from app_qspi_event_call too.
{
    if(p_qspi_env[id]->is_async_write_screen) {
        if(p_qspi_env[id]->is_tx_done && !p_qspi_env[id]->is_xfer_err) {

            if(DRAW_TYPE_IF_DUAL_SCREEN == s_async_write_screen_info.if_type) {

                s_async_write_screen_info.ss.dual_ss.total_sent_lines += s_async_write_screen_info.ss.dual_ss.this_send_lines;

                if(s_async_write_screen_info.ss.dual_ss.total_sent_lines >= s_async_write_screen_info.screen_info.scrn_pixel_height) {
                    p_qspi_env[id]->is_async_write_screen = false;
                    qspi_evt.type = APP_QSPI_EVT_ASYNC_WR_SCRN_CPLT;
                    goto Label_async_scrn_notify;
                } else {
                    bool ret = app_qspi_async_draw_screen(s_async_write_screen_info.screen_id,
                                                          s_async_write_screen_info.storage_id,
                                                          &s_async_write_screen_info.qspi_screen_command,
                                                          &s_async_write_screen_info.screen_info,
                                                          &s_async_write_screen_info.ss.dual_ss.scroll_config,
                                                          false);
                    if(!ret) {
                        p_qspi_env[id]->is_async_write_screen = false;
                        qspi_evt.type = APP_QSPI_EVT_ASYNC_WR_SCRN_FAIL;
                        goto Label_async_scrn_notify;
                    } else {
                        return true;
                    }
                }
            } else if(DRAW_TYPE_IF_VERI_LINKED_SCREEN == s_async_write_screen_info.if_type) {
                s_async_write_screen_info.ss.veri_linked_ss.total_sent_lines += s_async_write_screen_info.ss.veri_linked_ss.p_cur_scroll->frame_draw_lines;
                if(s_async_write_screen_info.ss.veri_linked_ss.p_cur_scroll->next != NULL) {
                    s_async_write_screen_info.ss.veri_linked_ss.p_cur_scroll = s_async_write_screen_info.ss.veri_linked_ss.p_cur_scroll->next;
                    bool ret = app_qspi_async_veri_draw_screen(s_async_write_screen_info.screen_id,
                                                               s_async_write_screen_info.storage_id,
                                                               &s_async_write_screen_info.qspi_screen_command,
                                                               &s_async_write_screen_info.screen_info,
                                                               s_async_write_screen_info.ss.veri_linked_ss.p_cur_scroll,
                                                               false);
                    if(!ret) {
                        p_qspi_env[id]->is_async_write_screen = false;
                        qspi_evt.type = APP_QSPI_EVT_ASYNC_WR_SCRN_FAIL;
                        goto Label_async_scrn_notify;
                    } else {
                        return true;
                    }
                } else {
                    p_qspi_env[id]->is_async_write_screen = false;
                    qspi_evt.type = APP_QSPI_EVT_ASYNC_WR_SCRN_CPLT;
                    goto Label_async_scrn_notify;
                }
            } else {
                p_qspi_env[id]->is_async_write_screen = false;
                p_qspi_env[id]->start_flag = false;
                if(s_async_write_screen_info.storage_id <= APP_QSPI_ID_2) {
                    p_qspi_env[s_async_write_screen_info.storage_id]->start_flag = false;
                }
                return false;
            }
        } else {
            p_qspi_env[id]->is_async_write_screen = false;
            qspi_evt.type = APP_QSPI_EVT_ASYNC_WR_SCRN_FAIL;
        }
Label_async_scrn_notify:
        if(s_async_write_screen_info.qspi_screen_command.is_one_take_cs) {
            app_qspi_force_cs(s_async_write_screen_info.screen_id, false);
        }
        p_qspi_env[id]->start_flag = false;
        if(s_async_write_screen_info.storage_id <= APP_QSPI_ID_2) {
            p_qspi_env[s_async_write_screen_info.storage_id]->start_flag = false;
        }
        s_async_write_screen_info.if_type = DRAW_TYPE_IF_NONE;
        if(p_qspi_env[id]->evt_handler != NULL)
        {
            p_qspi_env[id]->evt_handler(&qspi_evt);
        }
        return true;
    }
    else {
        return false;
    }
}

#endif

static void app_qspi_dma_evt_handler_0(app_dma_evt_type_t type) {
    switch(type) {
        case APP_DMA_EVT_TFR:
        {
            p_qspi_env[APP_QSPI_ID_0]->is_xfer_err = 0;
            p_qspi_env[APP_QSPI_ID_0]->is_dma_done = 1;
        }
        break;

        case APP_DMA_EVT_BLK:
        break;

        default:
        case APP_DMA_EVT_ERROR:
        {
            p_qspi_env[APP_QSPI_ID_0]->is_xfer_err = 1;
            p_qspi_env[APP_QSPI_ID_0]->is_dma_done = 1;
        }
        break;
    }
}

static void app_qspi_dma_evt_handler_1(app_dma_evt_type_t type) {
    switch(type) {
        case APP_DMA_EVT_TFR:
        {
            p_qspi_env[APP_QSPI_ID_1]->is_xfer_err = 0;
            p_qspi_env[APP_QSPI_ID_1]->is_dma_done = 1;
        }
        break;

        case APP_DMA_EVT_BLK:
        break;

        default:
        case APP_DMA_EVT_ERROR:
        {
            p_qspi_env[APP_QSPI_ID_1]->is_xfer_err = 1;
            p_qspi_env[APP_QSPI_ID_1]->is_dma_done = 1;
        }
        break;
    }
}

static void app_qspi_dma_evt_handler_2(app_dma_evt_type_t type) {
    switch(type) {
        case APP_DMA_EVT_TFR:
        {
            p_qspi_env[APP_QSPI_ID_2]->is_xfer_err = 0;
            p_qspi_env[APP_QSPI_ID_2]->is_dma_done = 1;
        }
        break;

        case APP_DMA_EVT_BLK:
        break;

        default:
        case APP_DMA_EVT_ERROR:
        {
            p_qspi_env[APP_QSPI_ID_2]->is_xfer_err = 1;
            p_qspi_env[APP_QSPI_ID_2]->is_dma_done = 1;
        }
        break;
    }
}

#endif
#endif
#endif

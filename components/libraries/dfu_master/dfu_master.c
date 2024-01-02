/**
 *****************************************************************************************
 *
 * @file dfu_master.c
 *
 * @brief  DFU master Implementation.
 *
 *****************************************************************************************
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
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "dfu_master.h"
#include "flash_scatter_config.h"
#include <string.h>

/*
 * DEFINES
 ****************************************************************************************
 */
#define ONCE_SEND_LEN                1024     /**< DFU master once send length. */
#define RECEIVE_MAX_LEN              2048     /**< DFU master receive max length. */
#define CMD_FRAME_HEADER_L           0x44     /**< CMD header low byte. */
#define CMD_FRAME_HEADER_H           0x47     /**< CMD header high byte. */
#define GET_INFO                     0x01     /**< Get info cmd. */
#define PROGRAM_START                0x23     /**< Program start cmd. */
#define PROGRAME_FLASH               0x24     /**< Program flash cmd. */
#define PROGRAME_END                 0x25     /**< Program end cmd. */
#define SYSTEM_INFO                  0x27     /**< System information cmd. */
#define DFU_MODE_SET                 0x41     /**< Dfu mode set cmd. */
#define DFU_FW_INFO_GET              0x42     /**< Dfu fw info get cmd. */
#define ACK_SUCCESS                  0x01     /**< CMD ack success. */
#define ACK_ERROR                    0x02     /**< CMD ack error. */

#define DFU_ERASE_REGION_NOT_ALIGNED 0x00     /**< FW erase region not aligned. */
#define DFU_ERASE_START_SUCCESS      0x01     /**< FW erase start sucess event. */
#define DFU_ERASEING_SUCCESS         0x02     /**< FW erase flash sucess event. */
#define DFU_ERASE_END_SUCCESS        0x03     /**< FW erase end sucess event. */
#define DFU_ERASE_REGIONS_OVERLAP    0x04     /**< FW erase regions overlap. */
#define DFU_ERASEING_FAIL            0x05     /**< FW erase flash fail event. */
#define DFU_ERASE_REGIONS_NOT_EXIS   0x06     /**< FW erase regions not exist. */
#define FAST_DFU_FLASH_SUCCESS       0xFF     /**< FW write flash success. */

#define FLASH_OP_PAGE_SIZE           0x1000   /**< Flash page size. */
#define PATTERN_VALUE                (0x4744) /**< Pattern value. */

#define FW_SIGN_FLAG_OFFSET          72       /**< Firmware sign flag offset. */
#define SIGN_FW_TYPE                 0x10     /**< Sign Firmware Type. */
#define NORMAL_FW_TYPE               0x00     /**< Normal Firmaware Type. */


/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/**@brief DFU master submachine state. */
typedef enum
{
    CHECK_FRAME_L_STATE = 0x00,
    CHECK_FRAME_H_STATE,
    RECEIVE_CMD_TYPE_L_STATE,
    RECEIVE_CMD_TYPE_H_STATE,
    RECEIVE_LEN_L_STATE,
    RECEIVE_LEN_H_STATE,
    RECEIVE_DATA_STATE,
    RECEIVE_CHECK_SUM_L_STATE,
    RECEIVE_CHECK_SUM_H_STATE,
} cmd_parse_state_t;

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief DFU master receive frame structure define. */
typedef struct
{
    uint16_t cmd_type;
    uint16_t data_len;
    uint8_t  data[RECEIVE_MAX_LEN];
    uint16_t check_sum;
} receive_frame_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
static receive_frame_t   s_receive_frame;
static boot_info_t       s_bootloader_boot_info;
static bool              s_cmd_receive_flag;
static uint16_t          s_receive_data_count;
static uint32_t          s_receive_check_sum;

static dfu_img_info_t    s_now_img_info;
static dfu_img_info_t    s_app_info;
static uint32_t          s_page_start_addr;
static uint32_t          s_all_check_sum;
static uint32_t          s_file_size;
static bool              s_run_fw_flag;
static dfu_m_func_cfg_t *s_p_func_cfg;

static uint16_t          s_sended_len;
static uint16_t          s_all_send_len;
static uint16_t          s_once_size = 350;
static uint8_t*          s_send_data_buffer;

static cmd_parse_state_t s_parse_state = CHECK_FRAME_L_STATE;

static bool              s_sec_flag = false;
static bool              s_version_flag = false;

static uint16_t          s_erase_all_count = 0;
static uint32_t          s_dfu_save_addr = 0;

uint8_t                  fast_dfu_mode = 0;
uint32_t                 program_size;
uint8_t                  ble_send_cplt_flag = 0;
/**
 *****************************************************************************************
 * @brief Function for getting updated firmware information.
 *
 * @param[in]  img_info: Pointer of firmware information
 *****************************************************************************************
 */
static void dfu_m_get_img_info(dfu_img_info_t *img_info)
{
    if(s_p_func_cfg -> dfu_m_get_img_info != NULL)
    {
        s_p_func_cfg -> dfu_m_get_img_info(img_info);
    }
}

/**
 *****************************************************************************************
 * @brief Function for get updated firmware data..
 *
 * @param[in]  addr: Get data address.
 * @param[in]  data: Pointer of get data.
 * @param[in]  len: Get data length
 *****************************************************************************************
 */
static void dfu_m_get_img_data(uint32_t addr, uint8_t *data, uint16_t len)
{
    if(s_p_func_cfg -> dfu_m_get_img_data != NULL)
    {
        s_p_func_cfg -> dfu_m_get_img_data(addr, data, len);
    }
}

/**
 *****************************************************************************************
 * @brief Function for start update firmware.
 *
 * @param[in]  security: Upgrade firmware is encrypted?.
 * @param[in]  run_fw: Whether to run the firmware immediately after the upgrade.
 *****************************************************************************************
 */
static void dfu_m_send_data(uint8_t *data, uint16_t len)
{
    if(s_p_func_cfg -> dfu_m_send_data != NULL)
    {
        s_p_func_cfg -> dfu_m_send_data(data, len);
    }
}

/**
 *****************************************************************************************
 * @brief Function for read firmware information.
 *
 * @param[in]  addr: The address of firmware stored.
 * @param[in]  p_buf:The buffer stored firmware data.
 * @param[in]  size: The read size.
 *****************************************************************************************
 */
static uint32_t dfu_m_fw_read(const uint32_t addr, uint8_t *p_buf, const uint32_t size)
{
    if (s_p_func_cfg->dfu_m_fw_read != NULL)
    {
        return s_p_func_cfg->dfu_m_fw_read(addr, p_buf, size);
    }
    else
    {
        return 0;
    }
}

/**
 *****************************************************************************************
 * @brief Function for start update firmware.
 *
 * @param[in]  security: Upgrade firmware is encrypted?.
 * @param[in]  run_fw: Whether to run the firmware immediately after the upgrade.
 *****************************************************************************************
 */
static void dfu_m_event_handler(dfu_m_event_t event, uint8_t pre)
{
    if(s_p_func_cfg -> dfu_m_event_handler != NULL)
    {
        s_p_func_cfg -> dfu_m_event_handler(event, pre);
    }
}


/**
 *****************************************************************************************
 * @brief Function for start update firmware.
 *
 * @param[in]  security: Upgrade firmware is encrypted?.
 * @param[in]  run_fw: Whether to run the firmware immediately after the upgrade.
 *****************************************************************************************
 */
static void dfu_m_cmd_check(void)
{
    uint16_t i = 0;
    for(i=0; i<s_receive_frame.data_len; i++)
    {
        s_receive_check_sum += s_receive_frame.data[i];
    }

    if((s_receive_check_sum & 0xffff) == s_receive_frame.check_sum)
    {
        s_cmd_receive_flag = true;
    }
    else
    {
        s_cmd_receive_flag = false;
        dfu_m_event_handler(FRAM_CHECK_ERROR, 0);
    }
}

/**
 *****************************************************************************************
 * @brief Function for start update firmware.
 *
 * @param[in]  security: Upgrade firmware is encrypted?.
 * @param[in]  run_fw: Whether to run the firmware immediately after the upgrade.
 *****************************************************************************************
 */
static void dfu_m_send(uint8_t *data, uint16_t len)
{
    s_send_data_buffer = s_receive_frame.data;
    memcpy(s_send_data_buffer,data,len);
    s_all_send_len = len;
    if(len >= s_once_size)
    {
        s_sended_len = s_once_size;
    }
    else
    {
        s_sended_len = len;
    }

    dfu_m_send_data(s_send_data_buffer,s_sended_len);
}

/**
 *****************************************************************************************
 * @brief Function for start update firmware.
 *
 * @param[in]  security: Upgrade firmware is encrypted?.
 * @param[in]  run_fw: Whether to run the firmware immediately after the upgrade.
 *****************************************************************************************
 */
static void dfu_m_send_frame(uint8_t *data,uint16_t len,uint16_t cmd_type)
{
    uint8_t send_data[RECEIVE_MAX_LEN + 8];
    uint16_t i = 0;
    uint32_t check_sum = 0;
    send_data[0] = CMD_FRAME_HEADER_L;
    send_data[1] = CMD_FRAME_HEADER_H;
    send_data[2] = cmd_type;
    send_data[3] = cmd_type >> 8;
    send_data[4] = len;
    send_data[5] = len >> 8;

    for(i=2; i<6; i++)
    {
        check_sum += send_data[i];
    }

    for(i=0; i<len; i++)
    {
        send_data[6+i] = *(data+i);
        check_sum += *(data+i);
    }
    send_data[6+len] = check_sum;
    send_data[7+len] = check_sum >> 8;
    dfu_m_send(send_data,len+8);
}

/**
 *****************************************************************************************
 * @brief Function for start update firmware.
 *
 * @param[in]  security: Upgrade firmware is encrypted?.
 * @param[in]  run_fw: Whether to run the firmware immediately after the upgrade.
 *****************************************************************************************
 */
static void dfu_m_program_flash(uint16_t len)
{
    uint16_t i=0;
    program_size += len;

    dfu_m_get_img_data(s_page_start_addr, &s_receive_frame.data[7], len);
    for(i=0; i<len; i++)
    {
        s_all_check_sum += s_receive_frame.data[i+7];
    }
    s_receive_frame.data[0] = 0x01;

    s_receive_frame.data[1] = s_dfu_save_addr;
    s_receive_frame.data[2] = s_dfu_save_addr>>8;
    s_receive_frame.data[3] = s_dfu_save_addr>>16;
    s_receive_frame.data[4] = s_dfu_save_addr>>24;

    s_receive_frame.data[5] = len;
    s_receive_frame.data[6] = len>>8;

    dfu_m_send_frame(s_receive_frame.data, len+7, PROGRAME_FLASH);
    s_dfu_save_addr += len;
    s_page_start_addr += len;
}

static void dfu_m_fast_program_flash(void)
{
    uint16_t remain;
    uint16_t i = 0;
    uint8_t pre = 0;

    while (program_size != s_file_size)
    {
        if (ble_send_cplt_flag || program_size == 0)
        {
            ble_send_cplt_flag = 0;

            dfu_m_get_img_data(s_page_start_addr, &s_receive_frame.data[0], s_once_size);

            if (program_size + s_once_size > s_file_size)
            {
                remain = s_file_size - program_size;
                dfu_m_send(&s_receive_frame.data[0], remain);
                for (i = 0; i < remain; i++)
                {
                    s_all_check_sum += s_receive_frame.data[i];
                }
                program_size += remain;
            }
            else
            {
                program_size += s_once_size;
                dfu_m_send(&s_receive_frame.data[0], s_once_size);
                for (i = 0; i < s_once_size; i++)
                {
                    s_all_check_sum += s_receive_frame.data[i];
                }
            }

            pre = (program_size * 100) / s_file_size;
            dfu_m_event_handler(FAST_DFU_PRO_FLASH_SUCCESS, pre);
            s_page_start_addr += s_once_size;
        }
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void dfu_m_send_data_cmpl_process(void)
{
    int remain = s_all_send_len - s_sended_len;

    if(remain >= s_once_size)
    {
        dfu_m_send_data(&s_send_data_buffer[s_sended_len], s_once_size);
        s_sended_len += s_once_size;
    }
    else if(remain > 0)
    {
        dfu_m_send_data(&s_send_data_buffer[s_sended_len], remain);
        s_sended_len += remain;
    }
}


void dfu_m_cmd_prase(uint8_t* data,uint16_t len)
{
    uint16_t i = 0;

    if(s_cmd_receive_flag == 0)
    {
        for(i=0; i<len; i++)
        {
            switch(s_parse_state)
            {
                case CHECK_FRAME_L_STATE:
                {
                    s_receive_check_sum = 0;
                    if(data[i] == CMD_FRAME_HEADER_L)
                    {
                        s_parse_state = CHECK_FRAME_H_STATE;
                    }
                }
                break;

                case CHECK_FRAME_H_STATE:
                {
                    if(data[i] == CMD_FRAME_HEADER_H)
                    {
                        s_parse_state = RECEIVE_CMD_TYPE_L_STATE;
                    } else if(data[i] == CMD_FRAME_HEADER_L) {
                        s_parse_state = CHECK_FRAME_H_STATE;
                    } else {
                        s_parse_state = CHECK_FRAME_L_STATE;
                    }
                }
                break;

                case RECEIVE_CMD_TYPE_L_STATE:
                {
                    s_receive_frame.cmd_type = data[i];
                    s_receive_check_sum += data[i];
                    s_parse_state = RECEIVE_CMD_TYPE_H_STATE;
                }
                break;

                case RECEIVE_CMD_TYPE_H_STATE:
                {
                    s_receive_frame.cmd_type |= (data[i] << 8);
                    s_receive_check_sum += data[i];
                    s_parse_state = RECEIVE_LEN_L_STATE;
                }
                break;

                case RECEIVE_LEN_L_STATE:
                {
                    s_receive_frame.data_len = data[i];
                    s_receive_check_sum += data[i];
                    s_parse_state = RECEIVE_LEN_H_STATE;
                }
                break;

                case RECEIVE_LEN_H_STATE:
                {
                    s_receive_frame.data_len |= (data[i] << 8);
                    s_receive_check_sum += data[i];
                    if(s_receive_frame.data_len == 0)
                    {
                        s_parse_state = RECEIVE_CHECK_SUM_L_STATE;
                    }
                    else if(s_receive_frame.data_len >= RECEIVE_MAX_LEN)
                    {
                        s_parse_state = CHECK_FRAME_L_STATE;
                    }
                    else
                    {
                        s_receive_data_count = 0;
                        s_parse_state = RECEIVE_DATA_STATE;
                    }
                }
                break;

                case RECEIVE_DATA_STATE:
                {
                    s_receive_frame.data[s_receive_data_count] = data[i];
                    if(++s_receive_data_count == s_receive_frame.data_len)
                    {
                        s_parse_state = RECEIVE_CHECK_SUM_L_STATE;
                    }
                }
                break;

                case RECEIVE_CHECK_SUM_L_STATE:
                {
                    s_receive_frame.check_sum = data[i];
                    s_parse_state = RECEIVE_CHECK_SUM_H_STATE;
                }
                break;

                case RECEIVE_CHECK_SUM_H_STATE:
                {
                    s_receive_frame.check_sum |= (data[i] << 8);
                    s_parse_state = CHECK_FRAME_L_STATE;
                    dfu_m_cmd_check();
                }
                break;

                default:{s_parse_state=CHECK_FRAME_L_STATE;}break;
            }
        }
    }
}

void dfu_m_init(dfu_m_func_cfg_t *dfu_m_func_cfg, uint16_t once_send_size)
{
    if(once_send_size != 0)
    {
      s_once_size = once_send_size;
    }

    if(dfu_m_func_cfg != NULL)
    {
        s_p_func_cfg = dfu_m_func_cfg;
    }
}


void dfu_m_program_start(bool security, bool run_fw)
{
    uint16_t img_len = sizeof(dfu_img_info_t);
    uint32_t fw_sign_flag_addr = 0;
    uint8_t fw_sign_flag[4] = {0};

    s_run_fw_flag = run_fw;
    s_page_start_addr = 0;
    s_all_check_sum = 0;
    s_file_size = 0;
    program_size = 0;
    s_receive_frame.data[0] = 0;

    dfu_m_get_img_info(&s_now_img_info);

    if (((s_now_img_info.boot_info.load_addr >= s_bootloader_boot_info.load_addr) && \
        (s_now_img_info.boot_info.load_addr <=  s_bootloader_boot_info.load_addr + s_bootloader_boot_info.bin_size + 48 + 856)) || \
         (s_now_img_info.boot_info.load_addr >= s_dfu_save_addr && s_now_img_info.boot_info.load_addr <= s_dfu_save_addr + s_now_img_info.boot_info.bin_size + 48 + 856))
    {
        dfu_m_event_handler(IMG_INFO_LOAD_ADDR_ERROR, 0);
    }

    if((s_now_img_info.pattern != PATTERN_VALUE) || \
       (s_now_img_info.boot_info.load_addr % FLASH_OP_PAGE_SIZE != 0))
    {
        dfu_m_event_handler(IMG_INFO_CHECK_FAIL, 0);
    }

    s_page_start_addr = (s_now_img_info.boot_info.load_addr & 0xfffff000);

    s_now_img_info.boot_info.load_addr = s_dfu_save_addr;

    if(security)//security mode
    {
        s_file_size = (s_now_img_info.boot_info.bin_size + 48 + 856);
    }
    else
    {
        fw_sign_flag_addr = s_page_start_addr + s_now_img_info.boot_info.bin_size + 48 + FW_SIGN_FLAG_OFFSET;
        dfu_m_fw_read(fw_sign_flag_addr, fw_sign_flag, sizeof(fw_sign_flag));

        if (fw_sign_flag[0] == 0x53 && fw_sign_flag[1] == 0x49 && fw_sign_flag[2] == 0x47 && fw_sign_flag[3] == 0x4E)
        {
            s_file_size = (s_now_img_info.boot_info.bin_size + 48 + 856);
            s_receive_frame.data[0] |= SIGN_FW_TYPE;
        }
        else
        {
            s_file_size = (s_now_img_info.boot_info.bin_size + 48);
            s_receive_frame.data[0] |= NORMAL_FW_TYPE;
        }
    }

    s_receive_frame.data[0] |= fast_dfu_mode;

    memcpy(&s_receive_frame.data[1], &s_now_img_info, img_len);

    dfu_m_send_frame(s_receive_frame.data, img_len+1, PROGRAM_START);
}

void dfu_m_parse_state_reset(void)
{
    s_parse_state = CHECK_FRAME_L_STATE;
    s_cmd_receive_flag   = false;
    s_receive_data_count = 0;
    s_receive_check_sum  = 0;
}

void dfu_m_get_info(void)
{
    dfu_m_send_frame(s_receive_frame.data, 0, GET_INFO);
}

void dfu_m_dfu_mode_set(uint8_t dfu_mode)
{
    s_receive_frame.data[0] = dfu_mode;
    dfu_m_send_frame(s_receive_frame.data, 1, DFU_MODE_SET);
}

void dfu_m_dfu_fw_info_get(void)
{
    dfu_m_send_frame(s_receive_frame.data, 0, DFU_FW_INFO_GET);
}

void dfu_m_system_info_get(void)
{
    // read
    uint32_t addr = FLASH_START_ADDR;
    s_receive_frame.data[0] = 0x00;
    // address
    s_receive_frame.data[1] = addr;
    s_receive_frame.data[2] = addr >> 8;
    s_receive_frame.data[3] = addr >> 16;
    s_receive_frame.data[4] = addr >> 24;
    //length
    s_receive_frame.data[5] = 0x30;
    s_receive_frame.data[6] = 0;

    dfu_m_send_frame(s_receive_frame.data, 7, SYSTEM_INFO);
}

bool dfu_m_get_sec_flag(void)
{
    return s_sec_flag;
}

void  dfu_m_schedule(dfu_m_rev_cmd_cb_t rev_cmd_cb)
{
    uint8_t pre = 0;
    uint16_t erase_count = 0;

    if(s_cmd_receive_flag)
    {
        if (rev_cmd_cb)
        {
            rev_cmd_cb();
        }

        switch (s_receive_frame.cmd_type)
        {
            case PROGRAM_START:
                if(s_receive_frame.data[0] == ACK_SUCCESS)
                {
                    if (FAST_DFU_MODE_DISABLE == fast_dfu_mode)
                    {
                        dfu_m_program_flash(ONCE_SEND_LEN);
                        dfu_m_event_handler(PRO_START_SUCCESS, 0);
                    }
                    else if (FAST_DFU_MODE_ENABLE == fast_dfu_mode)
                    {
                        switch (s_receive_frame.data[1])
                        {
                            case DFU_ERASE_START_SUCCESS:
                                s_erase_all_count = 0;
                                s_erase_all_count |= (s_receive_frame.data[2] & 0xff);
                                s_erase_all_count |= ((s_receive_frame.data[3] << 8) & 0xff00);
                                dfu_m_event_handler(ERASE_START_SUCCESS, 0);
                            break;

                            case DFU_ERASEING_SUCCESS:
                                erase_count |= (s_receive_frame.data[2] & 0xff);
                                erase_count |= ((s_receive_frame.data[3] << 8) & 0xff00);
                                pre = (erase_count * 100) / s_erase_all_count;
                                dfu_m_event_handler(ERASEING_SUCCESS, pre);
                            break;

                            case DFU_ERASE_END_SUCCESS:
                                dfu_m_event_handler(ERASE_END_SUCCESS, 0);
                                dfu_m_fast_program_flash();
                            break;

                            case DFU_ERASE_REGION_NOT_ALIGNED:
                                dfu_m_event_handler(ERASE_REGION_NOT_ALIGNED, 0);
                            break;

                            case DFU_ERASE_REGIONS_OVERLAP:
                                dfu_m_event_handler(ERASE_REGION_OVERLAP, 0);
                                break;

                            case DFU_ERASEING_FAIL:
                                dfu_m_event_handler(ERASE_FLASH_FAIL, 0);
                                break;

                            case DFU_ERASE_REGIONS_NOT_EXIS:
                                dfu_m_event_handler(ERASE_REGION_NOT_EXIST, 0);
                                break;

                            default:
                                break;
                        }
                    }
                }
                else
                {
                    dfu_m_event_handler(PRO_START_ERROR, 0);
                }
                break;

            case PROGRAME_FLASH:
                if(s_receive_frame.data[0] == ACK_SUCCESS)
                {
                    pre = (program_size * 100) / s_file_size;
                    dfu_m_event_handler(PRO_FLASH_SUCCESS, pre);
                    //pro success, precent
                    if(program_size == s_file_size)
                    {
                        s_receive_frame.data[0] = s_run_fw_flag;
                        s_receive_frame.data[1] = s_all_check_sum;
                        s_receive_frame.data[2] = s_all_check_sum>>8;
                        s_receive_frame.data[3] = s_all_check_sum>>16;
                        s_receive_frame.data[4] = s_all_check_sum>>24;
                        dfu_m_send_frame(s_receive_frame.data, 5, PROGRAME_END);//progem end
                    }
                    else if(program_size + ONCE_SEND_LEN > s_file_size)
                    {
                        dfu_m_program_flash(s_file_size - program_size);
                    }
                    else
                    {
                        dfu_m_program_flash(ONCE_SEND_LEN);
                    }
                }
                else
                {
                    dfu_m_event_handler(PRO_FLASH_FAIL, pre);
                }
                break;

            case PROGRAME_END:
                if(s_receive_frame.data[0] == ACK_SUCCESS)
                {
                    if (fast_dfu_mode == FAST_DFU_MODE_ENABLE)
                    {
                        uint32_t check_sum = 0;
                        check_sum |= (s_receive_frame.data[1] & 0xff);
                        check_sum |= ((s_receive_frame.data[2] << 8) & 0xff00);
                        check_sum |= ((s_receive_frame.data[3] << 16) & 0xff0000);
                        check_sum |= ((s_receive_frame.data[4] << 24) & 0xff000000);

                        if (check_sum == s_all_check_sum)
                        {
                            dfu_m_event_handler(PRO_END_SUCCESS, 0);
                        }
                        else
                        {
                            dfu_m_event_handler(PRO_END_FAIL, 0);
                        }
                    }
                    else if (fast_dfu_mode == FAST_DFU_MODE_DISABLE)
                    {
                        dfu_m_event_handler(PRO_END_SUCCESS, 0);
                    }
                }
                else
                {
                    dfu_m_event_handler(PRO_END_FAIL, 0);
                }
                break;

            case GET_INFO:
                s_dfu_save_addr = 0;
                if(s_receive_frame.data[0] == ACK_SUCCESS)
                {
                    // dfu version
                    if (s_receive_frame.data[17] == DFU_VERSION)
                    {
                        s_version_flag = 1; // new version
                    }
                    else
                    {
                        s_version_flag = 0; // old version
                    }
                    dfu_m_system_info_get();
                }
                else
                {
                    dfu_m_event_handler(GET_INFO_FAIL, 0);
                }
                break;

            case DFU_FW_INFO_GET:
                if (s_receive_frame.data[0] == ACK_SUCCESS)
                {
                    s_dfu_save_addr = 0;
                    s_dfu_save_addr |= (s_receive_frame.data[1] & 0xff);
                    s_dfu_save_addr |= ((s_receive_frame.data[2] << 8) & 0xff00);
                    s_dfu_save_addr |= ((s_receive_frame.data[3] << 16) & 0xff0000);
                    s_dfu_save_addr |= ((s_receive_frame.data[4] << 24) & 0xff000000);

                    memcpy(&s_app_info, &s_receive_frame.data[6], sizeof(dfu_img_info_t));

                    if (((s_dfu_save_addr >= s_app_info.boot_info.load_addr) && \
                        (s_dfu_save_addr <= s_app_info.boot_info.load_addr + s_app_info.boot_info.bin_size + 48 + 856)) || \
                         ((s_dfu_save_addr >= s_bootloader_boot_info.load_addr) && (s_dfu_save_addr <= s_bootloader_boot_info.load_addr + 48 + 856))) 
                    {
                        dfu_m_event_handler(DFU_FW_SAVE_ADDR_CONFLICT, 0);
                    }
                    else
                    {
                        dfu_m_dfu_mode_set(0x01);
                    }
                }
                else
                {
                    // error
                }
                break;

            case SYSTEM_INFO:
                if(s_receive_frame.data[0] == ACK_SUCCESS)
                {
                    // security mode
                    if (s_receive_frame.data[1])
                    {
                        s_sec_flag = true;
                    }
                    else
                    {
                        s_sec_flag = false;
                    }

                    memcpy(&s_bootloader_boot_info, &s_receive_frame.data[8], sizeof(boot_info_t));

                    if (s_version_flag)
                    {
                        dfu_m_dfu_fw_info_get();
                    }
                }
                break;

            case FAST_DFU_FLASH_SUCCESS:
                if (s_receive_frame.data[0] == ACK_SUCCESS)
                {
                    s_receive_frame.data[0] = s_run_fw_flag;
                    s_receive_frame.data[1] = s_all_check_sum;
                    s_receive_frame.data[2] = s_all_check_sum>>8;
                    s_receive_frame.data[3] = s_all_check_sum>>16;
                    s_receive_frame.data[4] = s_all_check_sum>>24;
                    dfu_m_send_frame(s_receive_frame.data, 5, PROGRAME_END);//progem end
                }
                else
                {
                    dfu_m_event_handler(FAST_DFU_FLASH_FAIL, 0);
                }
            break;

            default:
                break;
        }

        s_cmd_receive_flag = 0;
    }
}





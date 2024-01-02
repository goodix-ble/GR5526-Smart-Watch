/**
  ****************************************************************************************
  * @file    app_graphics_ospi.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2021 GOODIX All rights reserved.

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
#include "gr_soc.h"
#include "string.h"
#include "app_pwr_mgmt.h"
#include "app_drv_error.h"
#include "app_graphics_ospi.h"
#include "grx_hal.h"

#pragma  diag_suppress      177

#define GRAPHICS_OSPI_MODULE_ENABLED
#ifdef  GRAPHICS_OSPI_MODULE_ENABLED

#define REG(reg)            *((volatile uint32_t *) (reg))

#define OSPI_IO_NOPULL          0
#define OSPI_IO_PULLUP          1
#define OSPI_IO_PULLDOWN        2


#define GRAPHICS_DCDC_VOLTAGE           1150
#define GRAPHICS_DCORE_VOLTAGE          1150

#define MCU_BLE_DCDC_VOLTAGE            1150
#define MCU_BLE_DCORE_VOLTAGE           1100


/// Define Block debug print info control macro
#define OSPI_DBG_INFO_ENABLE  0
#if OSPI_DBG_INFO_ENABLE
    #define OSPI_DBG_PRINTF printf
#else
    #define OSPI_DBG_PRINTF(fmt, ...)
#endif

typedef struct {
    ospi_x_regs_t *                 p_instance;
    app_graphics_ospi_params_t      init_params;
} app_graphics_ospi_t;

typedef struct {
    uint32_t mem_base_address;                  /**< memory base address */
    uint32_t mem_top_address;                   /**< memory top address, max 0x1FFFFFF */
    uint32_t mem_page_size;                     /**< memory page size      */
    uint32_t timing_max_tCEM_cnt;               /**< max tCEM time in us   */
    uint32_t timing_min_tRST_cnt;               /**< min tRST time in us   */
    uint32_t timing_min_tCPH_cnt;               /**< min tCPH time in ns   */
    uint32_t timing_min_tRC_cnt;                /**< min tRC time in ns    */
    uint32_t timing_min_tXPDPD_cnt;             /**< min tXPDPD time in ns */
    uint32_t timing_min_tXDPD_cnt;              /**< min tXDPD time in us  */
    uint32_t timing_min_tXPHS_cnt;              /**< min tXPHS time in ns  */
    uint32_t timing_min_tXHS_cnt;               /**< min tXHS time in us   */
    uint32_t dqs_timeout_cnt;                   /**< DQS timeout in ns */
    uint32_t phy_delay_tap;                     /**< DQS timeout in ns */
    uint8_t is_tcem_ignore;                     /**< let controller ignore the tCEM or not  */
    uint8_t is_txdpd_ignore;                    /**< let controller ignore the txdpd or not */
    uint8_t is_txhs_ignore;                     /**< let controller ignore the txhs or not  */
    uint8_t is_read_prefetch;                   /**< enable read prefetch or not */
} app_graphics_ospi_config_t;

void OSPI_IRQHandler(void);

extern void gpu_psram_pmu_init(void);
extern void gpu_psram_pmu_deinit(void);

static bool                 ospi_config_init(app_graphics_ospi_config_t * p_ospi_init);
static void                 ospi_set_access_mode(app_ospi_access_mode_e mode);
static uint8_t              ospi_register_mode_read(uint32_t register_addr);
static void                 ospi_register_mode_write(uint32_t register_addr, uint8_t val);
static void                 ospi_set_reg_00h(app_ospi_psram_rd_latency_e rd_lc, app_ospi_psram_drv_strength_e drv);
static void                 ospi_set_reg_04h(app_ospi_psram_wr_latency_e wr_lc);
static uint8_t              ospi_read_reg_02h(void);
static void                 ospi_power_on(void);
static void                 ospi_power_off(void);
static hal_status_t         ospi_regs_resume(void * p_ospi_handle);
static void                 ospi_clock_on(app_ospi_clock_freq_e freq);
static void                 ospi_clock_off(void);
static void                 ospi_set_io_mode(uint32_t io_pull);
static void                 ospi_register_isr(ospi_irq_handler cb);
static void                 ospi_interrupr_handler(void);
static void                 ospi_init_light(void);
static void                 ospi_adjust_dig_core_voltage(void);
static uint16_t             ospi_psram_reinit(void);

typedef struct
{
    uint16_t    dcdc_mv;     /**< dcdc vout mv. */
    uint16_t    dcore_mv;    /**< dcore vout mv. */
    uint16_t    dcdc_vout;   /**< dcdc vout code. */
    uint16_t    dcore_vout;  /**< dcore vout code. */
    bool        dcore_bypass_en; /**< dcore bypass enable. */
} dcdc_dcore_info_t;

extern void     sys_pmu_dcdc_set(uint32_t dcdc_96m_mv, uint32_t dcdc_64m_mv);
extern void     sys_pmu_digcore_set(uint32_t dcore_96m_mv, uint32_t dcore_64m_mv);
extern void     pmu_dcdc_dcore_info_record(dcdc_dcore_info_t* p_dcdc_dcore);
extern          dcdc_dcore_info_t g_dcdc_dcore_set_for_gpu;
extern          dcdc_dcore_info_t g_dcdc_dcore_set_for_mcu;
extern          dcdc_dcore_info_t g_dcdc_dcore_set_for_ble;
extern          bool g_gpu_is_working;

static psram_reload_func_t          s_psram_reload_func;
static app_ospi_work_state_e        s_app_ospi_sleep_state = OSPI_STATE_HALF_SLEEP;
static app_graphics_ospi_t          s_graphics_ospi_env;
static ospi_irq_handler             s_ospi_handler   = NULL;
const app_graphics_ospi_config_t    s_ospi_config[4] = {
/**************************************************************
 *** PLEASE DO NOT MODIFY !
 **************************************************************/
    {   /* Config for 48MHz OSPI Clock */
        .mem_base_address       = 0x00000000,
        .mem_top_address        = 0x007FFFFF,       /* SIP : 8MB                */
        .mem_page_size          = LL_OSPI_X_MEM_PAGE_SIZE_1024Bytes,
        .timing_max_tCEM_cnt    = 180,              /* 48*4 - 12 = 180          */
        .timing_min_tRST_cnt    = 97,               /* 48*2 + 1                 */
        .timing_min_tCPH_cnt    = 1,                /* 1 48MHz clock : 20.5ns   */
        .timing_min_tRC_cnt     = 3,                /* 3 48MHz clock : 61.5ns   */
        .timing_min_tXPDPD_cnt  = 3,                /* 3 48MHz clock : 61.5ns   */
        .timing_min_tXDPD_cnt   = 7248,             /* (150+1)*48 = 7248        */
        .timing_min_tXPHS_cnt   = 3,                /* 3 48MHz clock : 61.5ns   */
        .timing_min_tXHS_cnt    = 7248,             /* (150+1)*48 = 7248        */
        .dqs_timeout_cnt        = 0x1F,
        .phy_delay_tap          = 0x02,
        .is_tcem_ignore         = 0x00,
        .is_txdpd_ignore        = 0x00,
        .is_txhs_ignore         = 0x00,
        .is_read_prefetch       = 0x00,
    },

    {   /* Config for 32MHz OSPI Clock */
        .mem_base_address       = 0x00000000,
        .mem_top_address        = 0x007FFFFF,       /* SIP : 8MB                */
        .mem_page_size          = LL_OSPI_X_MEM_PAGE_SIZE_1024Bytes,
        .timing_max_tCEM_cnt    = 116,              /* 32*4 - 12 = 116          */
        .timing_min_tRST_cnt    = 65,               /* 32*2 + 1                 */
        .timing_min_tCPH_cnt    = 1,                /* 1 32MHz clock : 31.2ns   */
        .timing_min_tRC_cnt     = 2,                /* 2 32MHz clock : 62.5ns   */
        .timing_min_tXPDPD_cnt  = 2,                /* 2 32MHz clock : 62.5ns   */
        .timing_min_tXDPD_cnt   = 4832,             /* (150+1)*32 = 4832        */
        .timing_min_tXPHS_cnt   = 2,                /* 2 32MHz clock : 62.5ns   */
        .timing_min_tXHS_cnt    = 4832,             /* (150+1)*32 = 4832        */
        .dqs_timeout_cnt        = 0x1F,
        .phy_delay_tap          = 0x02,
        .is_tcem_ignore         = 0x00,
        .is_txdpd_ignore        = 0x00,
        .is_txhs_ignore         = 0x00,
        .is_read_prefetch       = 0x00,
    },

    {   /* Config for 24MHz OSPI Clock */
        .mem_base_address       = 0x00000000,
        .mem_top_address        = 0x007FFFFF,       /* SIP : 8MB                */
        .mem_page_size          = LL_OSPI_X_MEM_PAGE_SIZE_1024Bytes,
        .timing_max_tCEM_cnt    = 84,               /* 24*4 - 12 = 84           */
        .timing_min_tRST_cnt    = 49,               /* 24*2 + 1                 */
        .timing_min_tCPH_cnt    = 1,                /* 1 24MHz clock : 41.6ns   */
        .timing_min_tRC_cnt     = 2,                /* 2 24MHz clock : 83.2ns   */
        .timing_min_tXPDPD_cnt  = 2,                /* 2 24MHz clock : 83.2ns   */
        .timing_min_tXDPD_cnt   = 3624,             /* (150+1)*24 = 3624        */
        .timing_min_tXPHS_cnt   = 2,                /* 2 24MHz clock : 83.2ns   */
        .timing_min_tXHS_cnt    = 3624,             /* (150+1)*24 = 3624        */
        .dqs_timeout_cnt        = 0x1F,
        .phy_delay_tap          = 0x02,
        .is_tcem_ignore         = 0x00,
        .is_txdpd_ignore        = 0x00,
        .is_txhs_ignore         = 0x00,
        .is_read_prefetch       = 0x00,
    },

    {   /* Config for 16MHz OSPI Clock */
        .mem_base_address       = 0x00000000,
        .mem_top_address        = 0x007FFFFF,        /* SIP : 8MB               */
        .mem_page_size          = LL_OSPI_X_MEM_PAGE_SIZE_1024Bytes,
        .timing_max_tCEM_cnt    = 52,               /* 16*4 - 12 = 52           */
        .timing_min_tRST_cnt    = 33,               /* 16*2 + 1                 */
        .timing_min_tCPH_cnt    = 1,                /* 1 16MHz clock : 62.5ns   */
        .timing_min_tRC_cnt     = 1,                /* 1 16MHz clock : 62.5ns   */
        .timing_min_tXPDPD_cnt  = 1,                /* 1 16MHz clock : 62.5ns   */
        .timing_min_tXDPD_cnt   = 2416,             /* (150+1)*16 = 2416        */
        .timing_min_tXPHS_cnt   = 1,                /* 1 16MHz clock : 62.5ns   */
        .timing_min_tXHS_cnt    = 2416,             /* (150+1)*16 = 2416        */
        .dqs_timeout_cnt        = 0x1F,
        .phy_delay_tap          = 0x02,
        .is_tcem_ignore         = 0x00,
        .is_txdpd_ignore        = 0x00,
        .is_txhs_ignore         = 0x00,
        .is_read_prefetch       = 0x00,
    }
};


void app_graphics_ospi_set_sleep_state(app_ospi_work_state_e state)
{
    s_app_ospi_sleep_state = state;
}

app_ospi_work_state_e app_graphics_ospi_get_sleep_state(void)
{
    return s_app_ospi_sleep_state;
}

uint16_t app_graphics_ospi_init(app_graphics_ospi_params_t * p_params)
{
    app_graphics_ospi_config_t  ospi_config;
    ll_ospi_x_init_t            ospi_init;
    uint8_t                     dev_id  = 0x00;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if(true != g_gpu_is_working) {
        OSPI_DBG_PRINTF("Adjust DCore Policy Firstly (call app_graphics_adjust_dcore_policy())! \r\n");
        ospi_adjust_dig_core_voltage();
    }

    ospi_power_on();
    ospi_clock_on(p_params->ospi_freq);
    ospi_set_io_mode(OSPI_IO_NOPULL);

    ospi_config                   = s_ospi_config[(uint32_t)p_params->ospi_freq];
    ospi_config.phy_delay_tap     = p_params->phy_delay;
    ospi_config.is_read_prefetch  = p_params->is_read_prefetch;
    ospi_config_init(&ospi_config);
    app_graphics_ospi_reset();
    delay_us(500);
    ospi_set_reg_00h(p_params->rd_lc,   p_params->drv_strength);
    delay_us(100);
    ospi_set_reg_04h(p_params->wr_lc);
    delay_us(100);
    dev_id = ospi_read_reg_02h() & 0x1F;

    if(dev_id != OSPI_PSRAM_DEVICE_ID) {
        OSPI_DBG_PRINTF("Invalid O.PSRAM ID: 0x%02x \r\n", dev_id);
        return APP_DRV_ERR_INVALID_ID;
    }

    soc_register_nvic(OSPI_IRQn, (uint32_t)OSPI_IRQHandler);
    ospi_set_access_mode(OSPI_ACCESS_MEMORY);
    ospi_register_isr(ospi_interrupr_handler);
    ll_ospi_x_set_dqs_timeout_interrupt(OSPI0,  LL_OSPI_X_INTERRUPT_ENABLE);
    NVIC_ClearPendingIRQ(OSPI_IRQn);
    NVIC_EnableIRQ(OSPI_IRQn);

    s_graphics_ospi_env.p_instance = OSPI0;
    hal_pwr_mgmt_extra_device_suspend_register(EXTRA_DEVICE_NUM_OSPI, NULL, NULL);
    hal_pwr_mgmt_extra_device_resume_register(EXTRA_DEVICE_NUM_OSPI,  ospi_regs_resume, NULL);

    memcpy((void*)&s_graphics_ospi_env.init_params, (void*)p_params,    sizeof(app_graphics_ospi_params_t));
    return APP_DRV_SUCCESS;
}

void app_graphics_ospi_deinit(void) {
    NVIC_DisableIRQ(OSPI_IRQn);
    NVIC_ClearPendingIRQ(OSPI_IRQn);
    ospi_clock_off();
    ospi_power_off();
    gpu_psram_pmu_deinit();
    return;
}

void app_graphics_ospi_reset(void)
{
    uint8_t ret = 0;
    ll_ospi_x_enable_global_reset(OSPI0);

    do {
        delay_us(1);
        ret = ll_ospi_x_is_global_rst_done(OSPI0);
    } while(ret !=0);

    return;
}

/**
 ****************************************************************************************
 * @brief  Set the OSPI work state.
 * @note   Currently the OSPI state is synchronized with the MCU state
 * @note   Currently shall not call hal_pwr_mgmt_set_extra_device_state to prevent sleep
 ****************************************************************************************
 */
void app_graphics_ospi_set_power_state(app_ospi_work_state_e state) {
    static app_ospi_work_state_e last_state = OSPI_STATE_ACTIVE;
    if(state == last_state){
        return;
    }
    switch(state) {
        case OSPI_STATE_DEEP_SLEEP:
        {
            ospi_clock_off();
            ospi_power_off();
            last_state = OSPI_STATE_DEEP_SLEEP;
        }
        break;

        case OSPI_STATE_HALF_SLEEP:
        {
            ll_ospi_x_entry_half_sleep(OSPI0);
            last_state = OSPI_STATE_HALF_SLEEP;
        }
        break;

        case OSPI_STATE_ACTIVE:
        {
            ospi_psram_reinit();
            last_state = OSPI_STATE_ACTIVE;
        }
        break;

        default:break;
    }
}


void app_graphics_ospi_set_pasr(app_ospi_pasr_e area) {
    if(area >= OSPI_PASR_MAX) {
        return;
    }

    uint8_t pasr = (uint8_t) area;
    uint8_t tmp  = 0;

    ospi_set_access_mode(OSPI_ACCESS_REGISTER);
    tmp = ospi_register_mode_read(0x04);
    ospi_register_mode_write(0x04, (tmp & 0xF8) | pasr);
    ospi_set_access_mode(OSPI_ACCESS_MEMORY);
}

void app_graphics_ospi_pasr_update(uint32_t psram_addr){
    uint32_t relative_addr = psram_addr - OSPI_PSRAM_MIN_XIP_ADDRESS;
    app_ospi_pasr_e refr_area = OSPI_PASR_FULL;
    if(relative_addr > 0x3FFFFF){
        refr_area = OSPI_PASR_FULL;
    }else if(relative_addr > 0x1FFFFF){
        refr_area = OSPI_PASR_BOTTOM_4MB;
    }else if(relative_addr > 0x0FFFFF){
        refr_area = OSPI_PASR_BOTTOM_2MB;
    }else{
        refr_area = OSPI_PASR_BOTTOM_1MB;
    }

    app_graphics_ospi_set_pasr(refr_area);
}

uint32_t app_graphics_ospi_get_base_address(void) {
    return OSPI_PSRAM_MIN_XIP_ADDRESS;
}

uint32_t app_graphics_ospi_get_byte_size(void) {
    return OSPI_PSRAM_BYTE_SIZE;
}

/**
 ****************************************************************************************
 * @brief  Register the OSPI reload function for PSRAM recovery after deep sleep.
 *
 * @param[in]  psram_reload_func_t: Reload the resources to PSRAM after PSRAM deep sleep.
 ****************************************************************************************
 */
void app_graphics_ospi_register_psram_reload_func(psram_reload_func_t psram_reload_func)
{
    s_psram_reload_func = psram_reload_func;
}

/**
 ****************************************************************************************
 * @brief  app graphics ospi sleep implementation
 *
 ****************************************************************************************
 */
void app_graphics_ospi_sleep(void)
{
    app_graphics_ospi_set_power_state(s_app_ospi_sleep_state);
}

/**
 ****************************************************************************************
 * @brief   app graphics ospi wakeup implementation
 *
 ****************************************************************************************
 */
void app_graphics_ospi_wakeup(void)
{
    app_graphics_ospi_set_power_state(OSPI_STATE_ACTIVE);
    if(OSPI_STATE_DEEP_SLEEP == s_app_ospi_sleep_state){
        if(s_psram_reload_func) s_psram_reload_func();
    }
}

void app_graphics_adjust_dcore_policy(void) {
    sys_pmu_dcdc_set(MCU_BLE_DCDC_VOLTAGE    /* 96MHz */,   MCU_BLE_DCDC_VOLTAGE  /* 64MHz */);
    sys_pmu_digcore_set(MCU_BLE_DCORE_VOLTAGE/* 96MHz */,   MCU_BLE_DCORE_VOLTAGE /* 64MHz */);
    pmu_dcdc_dcore_info_record(&g_dcdc_dcore_set_for_mcu);
    pmu_dcdc_dcore_info_record(&g_dcdc_dcore_set_for_ble);

    sys_pmu_dcdc_set(GRAPHICS_DCDC_VOLTAGE    /* 96MHz */,   GRAPHICS_DCDC_VOLTAGE  /* 64MHz */);
    sys_pmu_digcore_set(GRAPHICS_DCORE_VOLTAGE/* 96MHz */,   GRAPHICS_DCORE_VOLTAGE /* 64MHz */);
    pmu_dcdc_dcore_info_record(&g_dcdc_dcore_set_for_gpu);

    gpu_psram_pmu_init();
}

/*********************************************************************************************************
 **********************************************     STATIC METHODS    ************************************
 *********************************************************************************************************/

/*
 * Just Resume The OSPI Controller, when PSRAM NOT Sleep!
 */
static hal_status_t ospi_regs_resume(void * p_ospi_handle)
{
    ospi_init_light();
    return HAL_OK;
}


static uint16_t ospi_psram_reinit(void)
{
    app_graphics_ospi_config_t  ospi_config;
    ll_ospi_x_init_t            ospi_init;
    uint8_t                     dev_id  = 0x00;

    app_graphics_ospi_params_t * p_params = &s_graphics_ospi_env.init_params;
    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    ospi_adjust_dig_core_voltage();
    ospi_power_on();
    ospi_clock_on(p_params->ospi_freq);
    ospi_set_io_mode(OSPI_IO_NOPULL);

    ospi_config                   = s_ospi_config[(uint32_t)p_params->ospi_freq];
    ospi_config.phy_delay_tap     = p_params->phy_delay;
    ospi_config.is_read_prefetch  = p_params->is_read_prefetch;
    ospi_config_init(&ospi_config);
    app_graphics_ospi_reset();
    delay_us(100);
    ospi_set_reg_00h(p_params->rd_lc,   p_params->drv_strength);
    delay_us(50);
    ospi_set_reg_04h(p_params->wr_lc);
    delay_us(50);
    dev_id = ospi_read_reg_02h() & 0x1F;

    if(dev_id != OSPI_PSRAM_DEVICE_ID) {
        OSPI_DBG_PRINTF("Invalid O.PSRAM ID: 0x%02x \r\n", dev_id);
        return APP_DRV_ERR_INVALID_ID;
    }

    soc_register_nvic(OSPI_IRQn, (uint32_t)OSPI_IRQHandler);
    ospi_set_access_mode(OSPI_ACCESS_MEMORY);
    ospi_register_isr(ospi_interrupr_handler);
    ll_ospi_x_set_dqs_timeout_interrupt(OSPI0,  LL_OSPI_X_INTERRUPT_ENABLE);
    NVIC_ClearPendingIRQ(OSPI_IRQn);
    NVIC_EnableIRQ(OSPI_IRQn);

    return APP_DRV_SUCCESS;
}

static SECTION_RAM_CODE void ospi_init_light(void)
{
    app_graphics_ospi_config_t  ospi_config;
    ospi_clock_on(s_graphics_ospi_env.init_params.ospi_freq);

    ospi_config                   = s_ospi_config[(uint32_t)s_graphics_ospi_env.init_params.ospi_freq];
    ospi_config.phy_delay_tap     = s_graphics_ospi_env.init_params.phy_delay;
    ospi_config.is_read_prefetch  = s_graphics_ospi_env.init_params.is_read_prefetch;
    ospi_config_init(&ospi_config);
    //app_graphics_ospi_reset();
    ospi_set_reg_00h(s_graphics_ospi_env.init_params.rd_lc,   s_graphics_ospi_env.init_params.drv_strength);
    ospi_set_reg_04h(s_graphics_ospi_env.init_params.wr_lc);

    ospi_set_access_mode(OSPI_ACCESS_MEMORY);
    //ospi_register_isr(ospi_interrupr_handler);
    //ll_ospi_x_set_dqs_timeout_interrupt(OSPI0,  LL_OSPI_X_INTERRUPT_ENABLE);
    //NVIC_ClearPendingIRQ(OSPI_IRQn);
    //NVIC_EnableIRQ(OSPI_IRQn);
}

SECTION_RAM_CODE static bool ospi_config_init(app_graphics_ospi_config_t * p_ospi_init)
{
    /* Check the parameters */
    if(p_ospi_init == (void*)0) {
        return false;
    }

    {
        /* config device characterstics */
        ll_ospi_x_set_mem_base_address(OSPI0,       p_ospi_init->mem_base_address);
        ll_ospi_x_set_mem_top_address(OSPI0,        p_ospi_init->mem_top_address);
        ll_ospi_x_set_mem_page_size(OSPI0,          p_ospi_init->mem_page_size);
        ll_ospi_x_set_tcem_count(OSPI0,             p_ospi_init->timing_max_tCEM_cnt);
        ll_ospi_x_set_trst_count(OSPI0,             p_ospi_init->timing_min_tRST_cnt);
        ll_ospi_x_set_tcph_count(OSPI0,             p_ospi_init->timing_min_tCPH_cnt);
        ll_ospi_x_set_trc_count(OSPI0,              p_ospi_init->timing_min_tRC_cnt);
        ll_ospi_x_set_txpdpd_count(OSPI0,           p_ospi_init->timing_min_tXPDPD_cnt);
        ll_ospi_x_set_txdpd_count(OSPI0,            p_ospi_init->timing_min_tXDPD_cnt);
        ll_ospi_x_set_txphs_count(OSPI0,            p_ospi_init->timing_min_tXPHS_cnt);
        ll_ospi_x_set_txhs_count(OSPI0,             p_ospi_init->timing_min_tXHS_cnt);
        ll_ospi_x_set_dqs_timeout(OSPI0,            p_ospi_init->dqs_timeout_cnt);
        ll_ospi_x_set_phy_delay(OSPI0,              p_ospi_init->phy_delay_tap);
        ll_ospi_x_set_read_prefetch(OSPI0,          p_ospi_init->is_read_prefetch);
        /* config controller ignore */
        ll_ospi_x_set_tcem_ignore(OSPI0,            p_ospi_init->is_tcem_ignore  ? LL_OSPI_X_TCEM_TIME_IGNORE_ENABLE  : LL_OSPI_X_TCEM_TIME_IGNORE_DISABLE );
        ll_ospi_x_set_txdpd_ignore(OSPI0,           p_ospi_init->is_txdpd_ignore ? LL_OSPI_X_TXDPD_TIME_IGNORE_ENABLE : LL_OSPI_X_TXDPD_TIME_IGNORE_DISABLE);
        ll_ospi_x_set_txhs_ignore(OSPI0,            p_ospi_init->is_txhs_ignore  ? LL_OSPI_X_TXHS_TIME_IGNORE_ENABLE  : LL_OSPI_X_TXHS_TIME_IGNORE_DISABLE );
    }

    return true;
}

static void ospi_set_access_mode(app_ospi_access_mode_e mode) {
    switch(mode) {
        case OSPI_ACCESS_REGISTER : ll_ospi_x_set_access_type(OSPI0, LL_OSPI_X_ACCESS_TYPE_MODE_REGISTER);      break;
        case OSPI_ACCESS_MEMORY   : ll_ospi_x_set_access_type(OSPI0, LL_OSPI_X_ACCESS_TYPE_MEMORY_ARRAY);       break;
        default: break;
    }
    return;
}

static uint8_t ospi_register_mode_read(uint32_t register_addr)
{
    return  *((volatile uint8_t *) (OSPI0_XIP_BASE + register_addr)) ;
}

static void ospi_register_mode_write(uint32_t register_addr, uint8_t val){
    *((volatile uint8_t *) (OSPI0_XIP_BASE + register_addr)) = val ;
}

static void ospi_set_reg_00h(app_ospi_psram_rd_latency_e rd_lc, app_ospi_psram_drv_strength_e drv) {
    uint32_t val  = (((uint32_t)rd_lc) << 2 ) | ((uint32_t)drv);
    ospi_set_access_mode(OSPI_ACCESS_REGISTER);
    ospi_register_mode_write(0x00, val);
}

static void ospi_set_reg_04h(app_ospi_psram_wr_latency_e wr_lc) {
    uint32_t val  = ((uint32_t)wr_lc) << 5 ;
    ospi_set_access_mode(OSPI_ACCESS_REGISTER);
    ospi_register_mode_write(0x04, val);
}

static uint8_t ospi_read_reg_02h(void) {
    ospi_set_access_mode(OSPI_ACCESS_REGISTER);
    return ospi_register_mode_read(0x02);
}

void ospi_enable_slow_refresh(void) {

    ospi_set_access_mode(OSPI_ACCESS_REGISTER);
    uint8_t val = ospi_register_mode_read(0x04);
    ospi_register_mode_write(0x04, val | 0x08);
    ospi_set_access_mode(OSPI_ACCESS_MEMORY);
}

static void ospi_power_on(void) {
    REG(0xA000A010) = (REG(0xA000A010)) | 0x0002;
}

static void ospi_power_off(void) {
    REG(0xA000A010) = (REG(0xA000A010)) & (~0x0002);
}

static void ospi_clock_on(app_ospi_clock_freq_e freq) {
    //REG(0xA000EE18) = (REG(0xA000EE18)) | 0x07FF;
    REG(0xA000EE8C) = (REG(0xA000EE8C)) & (~(1u << 31));
    REG(0xA000EE94) = (REG(0xA000EE94)) & (~(1u << 25));
    REG(0xA000E050) = 0x0001 | (((uint32_t) freq) << 8);
}

static void ospi_clock_off(void) {
    REG(0xA000EE8C) = (REG(0xA000EE8C)) | (1u << 31);
    REG(0xA000EE94) = (REG(0xA000EE94)) | (1u << 25);
    REG(0xA000E050) = 0x0000;
}


static void ospi_set_io_mode(uint32_t io_pull) {
    if(io_pull == OSPI_IO_NOPULL) {
        REG(0xA000EE18) = (REG(0xA000EE18)) & (~0x07FF);
    } else if (io_pull == OSPI_IO_PULLUP) {
        REG(0xA000EE18) = (REG(0xA000EE18)) | 0x07FF;
        REG(0xA000EE1C) = (REG(0xA000EE1C)) | 0x07FF;
    } else if (io_pull == OSPI_IO_PULLDOWN) {
        REG(0xA000EE18) = (REG(0xA000EE18)) | 0x07FF;
        REG(0xA000EE1C) = (REG(0xA000EE1C)) & (~0x07FF);
    }
}

static void ospi_register_isr(ospi_irq_handler cb) {
    s_ospi_handler = cb;
}

static void ospi_interrupr_handler(void) {
    ll_ospi_x_set_dqs_timeout_interrupt(OSPI0,  LL_OSPI_X_INTERRUPT_DISABLE);
    uint32_t status = OSPI0->XFER_STATUS;
    status = status;
    *((volatile uint32_t *)0xA000E194) = 0x0000;
    *((volatile uint32_t *)0xA000E194) = 0x0101;
    app_graphics_ospi_init(&s_graphics_ospi_env.init_params);
    OSPI_DBG_PRINTF("<");
}

static void ospi_adjust_dig_core_voltage(void) {
    sys_pmu_dcdc_set(GRAPHICS_DCDC_VOLTAGE    /* 96MHz */,   GRAPHICS_DCDC_VOLTAGE  /* 64MHz */);
    sys_pmu_digcore_set(GRAPHICS_DCORE_VOLTAGE/* 96MHz */,   GRAPHICS_DCORE_VOLTAGE /* 64MHz */);
    pmu_dcdc_dcore_info_record(&g_dcdc_dcore_set_for_gpu);

    gpu_psram_pmu_init();
}

void OSPI_IRQHandler(void) {
    if(s_ospi_handler != NULL)
        s_ospi_handler();
}


#endif /* GRAPHICS_OSPI_MODULE_ENABLED */

#include "gr_soc.h"

#ifndef DRIVER_TEST
#include "gr_includes.h"
#endif

#include "hal_flash.h"
#include "platform_sdk.h"
#include "pmu_calibration.h"
#include "patch_tab.h"
#include "app_pwr_mgmt.h"

#define PUYA_FLASH_HP_CMD              (0xA3)
#define PUYA_FLASH_HP_END_DUMMY        (2)
#define FALSH_HP_MODE                  XQSPI_HP_MODE_DIS
#define FLASH_HP_CMD                   PUYA_FLASH_HP_CMD
#define FLASH_HP_END_DUMMY             PUYA_FLASH_HP_END_DUMMY
#define SOFTWARE_REG_WAKEUP_FLAG_POS   (8)

extern uint8_t g_short_digcore_aon_flag;


/******************************************************************************/

#if (CFG_LCP_SUPPORT)
static uint8_t lcp_buf[280] __attribute__((section (".ARM.__at_0x20070000"), zero_init));
#endif

__ALIGNED(0x400) FuncVector_t FuncVector_table[MAX_NUMS_IRQn + NVIC_USER_IRQ_OFFSET] = {
    0,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
};

void soc_register_nvic(IRQn_Type indx, uint32_t func)
{
    FuncVector_table[indx + NVIC_USER_IRQ_OFFSET] = (FuncVector_t)func;
}

static fun_t svc_user_func = NULL;

void svc_func_register(uint8_t svc_num, uint32_t user_func)
{
    svc_user_func = (fun_t)user_func;
}

void svc_user_handler(uint8_t svc_num)
{
    if (svc_user_func)
        svc_user_func();
}

static void nvds_setup(void)
{
#ifndef ATE_TEST_ENABLE
    #ifndef DRIVER_TEST
    nvds_retention_size(CFG_MAX_BOND_DEVS);
    #endif
#ifdef NVDS_START_ADDR
    uint8_t err_code = nvds_init(NVDS_START_ADDR, NVDS_NUM_SECTOR);
#else
    uint8_t err_code = nvds_init(0, NVDS_NUM_SECTOR);
#endif

    switch(err_code)
    {
        case NVDS_FAIL:
        case NVDS_STORAGE_ACCESS_FAILED:
            {
                uint32_t start_addr  = nvds_get_start_addr();
                uint32_t sector_size = hal_flash_sector_size();
                if (hal_flash_erase(start_addr, NVDS_NUM_SECTOR * sector_size))
                {
                    err_code = nvds_init(start_addr, NVDS_NUM_SECTOR);
                    if (NVDS_SUCCESS == err_code)
                    {
                        break;
                    }
                }
                /* Flash fault, cannot startup.
                 * TODO: Output log via UART or Dump an error code to flash. */
                while(1);
            }
        case NVDS_SUCCESS:
            break;
        default:
            /* Illegal NVDS Parameters.
             * Please check the start address and number of sectors. */
            while(1);
    }
    #endif //ATE_TEST_ENABLE
}

bool platform_flash_init(void)
{
    platform_xqspi_env_init();
    g_xqspi_handle.init.cache_flush = LL_XQSPI_CACHE_FLUSH_EN;
    g_xqspi_handle.init.cache_direct_map_en = 0;
    bool status = hal_flash_init();
    g_xqspi_handle.init.cache_flush = LL_XQSPI_CACHE_FLUSH_DIS;
    return status;
}

void efuse_init(void)
{
    if (SDK_SUCCESS != sys_efuse_info_sync())
    {
        /* do nothing for not calibrated chips, ate shall return here */
        while(1);
    }
}

void first_class_task(void)
{
    // set sleep voltage level 0, wakeup voltage to level 4
    ll_aon_pmu_set_lpd_sleep(0);
    ll_aon_pmu_set_lpd_active(4);
    if(g_short_digcore_aon_flag)
    {
        ll_aon_pmu_enable_short_aon_digcore();
        ll_aon_pmu_set_lpd_active(0);
    }

    /* set sram power state. */
    mem_pwr_mgmt_mode_set(MEM_POWER_AUTO_MODE);
    /* set rng 2M to 1.3hz to save sleep power,enable pwer save */
    aon_pmu_set_rng_freq(AON_PMU_RF_REG_0_RNG_FREQ_CONT_1P3MHz);

    exflash_hp_init_t hp_init;

    if (!platform_flash_init())
    {
        /* Flash fault, cannot startup.
         * TODO: Output log via UART or Dump an error code to flash. */
        while(1);
    }

    hp_init.xqspi_hp_enable    = FALSH_HP_MODE;
    hp_init.xqspi_hp_cmd       = FLASH_HP_CMD;
    hp_init.xqspi_hp_end_dummy = FLASH_HP_END_DUMMY;
    platform_flash_enable_quad(&hp_init);

    /* nvds module init process. */
    nvds_setup();

    /* platform init process. */
    platform_sdk_init();
}

void second_class_task(void)
{
    /* To choose the System clock source and set the accuracy of OSC. */
#if CFG_LPCLK_INTERNAL_EN
    platform_clock_init_with_rng2((mcu_clock_type_t)SYSTEM_CLOCK, CFG_LF_ACCURACY_PPM, 0);
#else
    platform_clock_init_with_rtc((mcu_clock_type_t)SYSTEM_CLOCK,  CFG_LF_ACCURACY_PPM, 0);
#endif

    #if !defined(DRIVER_TEST)
    /* Enable auto pmu calibration function. */
    if(!CHECK_IS_ON_FPGA())
    {
        system_pmu_calibration_init(30000);
    }
    #endif

    system_pmu_init((mcu_clock_type_t)SYSTEM_CLOCK);
    SystemCoreSetClock((mcu_clock_type_t)SYSTEM_CLOCK);
    SetSerialClock(SERIAL_N96M_CLK);

    #if (CFG_LCP_SUPPORT)
    gdx_lcp_buf_init((uint32_t)lcp_buf);
    #endif

    // recover the default setting by temperature, should be called in the end
    if(!CHECK_IS_ON_FPGA())
    {
        pmu_calibration_handler(NULL);
    }
    /* Init peripheral sleep management */
    app_pwr_mgmt_init();
}

static void patch_init(void)
{
    gr5xx_fpb_init(FPB_MODE_PATCH_AND_DEBUG);
}

const FUNC_t high_priority_func_list[] =
{
    patch_init,
    efuse_init,
    first_class_task,
};

const FUNC_t low_priority_func_list[] =
{
    second_class_task,
};

void platform_init(void)
{
    for (uint32_t idx = 0; idx < (sizeof (high_priority_func_list) / sizeof(uint32_t)); idx ++ )
    {
        if (high_priority_func_list[idx])
        {
            high_priority_func_list[idx]();
        }
    }

#ifndef BYPASS_LOW_PRIO_TASK_EXEC
    for (uint32_t idx = 0; idx < (sizeof (low_priority_func_list) / sizeof(uint32_t)); idx ++ )
    {
        if (low_priority_func_list[idx])
        {
            low_priority_func_list[idx]();
        }
    }
#endif
    return ;
}

uint32_t get_wakeup_flag(void)
{
    return (AON_CTL->SOFTWARE_REG0 & (1 << SOFTWARE_REG_WAKEUP_FLAG_POS));
}

void warm_boot_process(void)
{
    vector_table_init();
    pwr_mgmt_warm_boot();
}

void vector_table_init(void)
{
    __DMB(); // Data Memory Barrier
    FuncVector_table[0] = *(FuncVector_t *)(SCB->VTOR);
    SCB->VTOR = (uint32_t)FuncVector_table; // Set VTOR to the new vector table location
    __DSB(); // Data Synchronization Barrier to ensure all
}

void soc_init(void)
{
    platform_init();
}

void ble_sdk_patch_env_init(void)
{
    // register the ke msg handler for patch
    uint16_t ke_msg_cnt = sizeof(ke_msg_tab) / sizeof(ke_msg_tab_item_t);
    reg_ke_msg_patch_tab(ke_msg_tab, ke_msg_cnt);

    // register the llm hci cmd handler for patch
    uint16_t llm_hci_cmd_cnt = sizeof(llm_hci_cmd_tab) / sizeof(llm_hci_cmd_tab_item_t);
    reg_llm_hci_cmd_patch_tab(llm_hci_cmd_tab, llm_hci_cmd_cnt);

    // register the gapm hci evt handler for patch
    uint16_t gapm_hci_evt_cnt = sizeof(gapm_hci_evt_tab) / sizeof(gapm_hci_evt_tab_item_t);
    reg_gapm_hci_evt_patch_tab(gapm_hci_evt_tab, gapm_hci_evt_cnt);

    #if CFG_ISO_SUPPORT
    uint16_t lli_hci_cmd_cnt = sizeof(lli_hci_cmd_tab) / sizeof(lli_hci_cmd_tab_item_t);
    reg_lli_hci_cmd_patch_tab(lli_hci_cmd_tab, lli_hci_cmd_cnt);
    #endif

    ble_common_env_init();

    #if CFG_MAX_CONNECTIONS
    ble_con_env_init();
    //ble_adv_param_init(CFG_MAX_CONNECTIONS);
    #endif

    #if CFG_MAX_SCAN
    ble_scan_env_init();
    #endif

    #if CFG_MAX_ADVS
    ble_adv_env_init();
    #endif

    #if CFG_ISO_SUPPORT
    ble_iso_env_init();
    #endif

    #if CFG_EATT_SUPPORT
    ble_eatt_evn_init();
    #endif

    #if CFG_MUL_LINK_WITH_SAME_DEV
    ble_mul_link_env_init();
    #endif

    #if CFG_CAR_KEY_SUPPORT
    ble_car_key_env_init();
    #endif

    #if CFG_BT_BREDR
    ble_bt_bredr_env_init();
    #endif
}

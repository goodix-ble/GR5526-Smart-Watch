#ifndef GR_SOC_H
#define GR_SOC_H

#include "grx_sys.h"

extern void Reset_Handler(void);
extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void MemManage_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);
extern void SVC_Handler(void);
extern void DebugMon_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

extern void platform_xqspi_env_init(void);
extern void vector_table_init(void);
extern void soc_init(void);
extern void warm_boot_process(void);
extern void platform_init(void);
extern void sdk_init(void);
extern void soc_register_nvic(IRQn_Type indx, uint32_t func);
extern void aon_pmu_set_rng_freq(uint32_t value);
extern uint16_t sys_efuse_info_sync(void);
extern uint16_t gdx_lcp_buf_init(uint32_t buf_addr);
extern uint32_t nvds_get_start_addr(void);
extern uint32_t get_wakeup_flag(void);
extern xqspi_handle_t g_xqspi_handle;

typedef void (*FuncVector_t)(void);
typedef void (*FUNC_t)(void);

#endif

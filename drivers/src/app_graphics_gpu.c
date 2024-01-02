/**
  ****************************************************************************************
  * @file    app_graphics_gpu.c
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
#include "app_io.h"
#include "app_graphics_gpu.h"

#include "grx_hal.h"
#include "hal_gfx_regs.h"
#include "hal_gfx_graphics.h"
#include "hal_gfx_ringbuffer.h"
#include "hal_gfx_error.h"
#include "hal_gfx_error_intern.h"
#include "hal_gfx_hal.h"


#ifdef USE_TSI_MALLOC
    #include "tsi_malloc.h"
#else
    #include "FreeRTOS.h"
#endif

#ifdef ENV_USE_FREERTOS
    #ifdef USE_OSAL
        #include "osal.h"
    #else // USE_OSAL
        #include "FreeRTOS.h"
        #include "task.h"
        #include "app_rtos_cfg.h"
    #endif //  USE_OSAL
#elif 1==CONFIG_ZEPHYR_OS
    #include <zephyr/kernel.h>
#endif

/*
 * DEFINES
 *****************************************************************************************
 */
#if CONFIG_ZEPHYR_OS
#define GPU_WAIT_IRQ_USING_SEMAPHORE            (0)
#else
#define GPU_WAIT_IRQ_USING_SEMAPHORE            (1u)
#endif
#define GPU_WAIT_TIMEOUT_MS                     (1000)
#define REG(reg)                                *((volatile uint32_t *) (reg))
#define GPU_REG(reg)                            *((uint32_t volatile *) (GRAPHICS_GPU_BASEADDR + reg))

#define GPU_IRQ_CMD_LIST_END_BIT                (1u << 1)
#define GPU_IRQ_DRAW_CMD_END_BIT                (1u << 2)
#define GPU_IRQ_CLEAR_IRQ_ID_BIT                (1u << 3)
#define GPU_IRQ_OTHER_IRQ_BITS                  0xFFFFFFF1U
#define RINGSTOP_NOFLUSH                        (0x1U)
#define RINGSTOP_NOTRIGGER                      (0x2U)
#define RINGSTOP_ENABLE                         (0x4U)

#define GRAPHICS_MCU_CONF_ENABLE                1u

/// Define Block debug print info control macro
#define GPU_DBG_INFO_ENABLE 0
#if GPU_DBG_INFO_ENABLE
#define GPU_DBG_PRINTF printf
#else
#define GPU_DBG_PRINTF(fmt, ...)
#endif

#if GPU_WAIT_IRQ_USING_SEMAPHORE > 0u
#ifdef USE_OSAL
    static osal_sema_handle_t s_gpu_irq_sem;
#else
    APP_DRV_SEM_STATIC(s_gpu_irq_sem);
#endif
#endif

void TSI_GPU_IRQHandler(void);
void TSI_GPU_ERR_IRQHandler(void);

static uint8_t  s_graphics_memory_buffer[VMEM_SIZE] __attribute__((aligned(0x10))) ;

typedef struct {
    graphics_gpu_irq_event_notify   irq_evt_cb;
    volatile uint8_t                is_inited;
    volatile uint8_t                is_cmd_cplt;
    volatile uint8_t                is_async_mark;
    uint32_t                        rb_offset;
} app_graphics_gpu_t;

extern void gpu_psram_pmu_init(void);

/*
 * STATIC DECLRATION
 *****************************************************************************************
 */
static void  _gpu_clock_on(void);
static void  _gpu_clock_off(void);

#if GPU_WAIT_IRQ_USING_SEMAPHORE > 0u
    static int   _gpu_irq_sem_init(void);
    static int   _gpu_irq_sem_take(void);
    static int   _gpu_irq_sem_give(void);
#endif

static volatile app_graphics_gpu_t  s_graphics_gpu_env;
static volatile int                 g_last_cl_id = -1;
static hal_gfx_ringbuffer_t         g_nema_rb = {{0}};
static volatile int                 g_cl_create_cnt = 0;
#if GPU_WAIT_IRQ_USING_SEMAPHORE > 0u
static volatile int                 s_gpu_sem_give_cnt = 0;
static volatile int                 s_gpu_sem_take_cnt = 0;
#endif

/*
 * PUBLIC METHODS
 *****************************************************************************************
 */
uint16_t graphics_gpu_init(graphics_gpu_irq_event_notify evt_cb) {

    _gpu_clock_off();
    delay_us(10);
    _gpu_clock_on();

    if ( hal_gfx_init() != 0 ) {
        return APP_DRV_ERR_HAL;
    }
#if GPU_WAIT_IRQ_USING_SEMAPHORE > 0u
    _gpu_irq_sem_init();
#endif
    gpu_psram_pmu_init();

    s_graphics_gpu_env.irq_evt_cb       = evt_cb;
    s_graphics_gpu_env.is_cmd_cplt      = 0;
    s_graphics_gpu_env.is_async_mark    = 0;
    s_graphics_gpu_env.is_inited        = 1;
    s_graphics_gpu_env.rb_offset        = g_nema_rb.offset;

#if GRAPHICS_MCU_CONF_ENABLE > 0u
    // Enable MCU clock firstly for safety(To Be Optimized)
    MCU_RET->MCU_SUBSYS_CG_CTRL[0] = 0; // MCU clock related
    MCU_RET->MCU_SUBSYS_CG_CTRL[1] = 0; // MCU clock related
    // Prevent WFI turn off mcu register clock
    MCU_RET->MCU_SUBSYS_CG_CTRL[2] = 0;
    MCU_RET->MCU_PERIPH_PCLK_OFF = 0; // XQSPI related
    MCU_RET->MCU_PERIPH_CG_LP_EN = 0; // QSPI related
    MCU_RET->MCU_PERIPH_CLK_SLP_OFF = 0; // PSRAM related
    // Prevent WFI turn off GPU register clock
    GPU_REG(0x94) = 0xFFFFFFFF;         /* fix blur issue in TSC4 transition */
#endif

    return APP_DRV_SUCCESS;
}

void graphics_gpu_deinit(void) {
    _gpu_clock_off();
    NVIC_DisableIRQ(TSI_GPU_IRQn);
    NVIC_DisableIRQ(TSI_GPU_ERR_IRQn);
    NVIC_ClearPendingIRQ(TSI_GPU_IRQn);
    NVIC_ClearPendingIRQ(TSI_GPU_ERR_IRQn);
}


void app_graphics_gpu_sleep(void)
{
    _gpu_clock_off();
}

void app_graphics_gpu_set_power_state(graphics_gpu_power_state_e state) {
    switch(state) {

        case GPU_POWER_STATE_SLEEP:
#if !CONFIG_ZEPHYR_OS
        {
            hal_pwr_mgmt_set_extra_device_state(EXTRA_DEVICE_NUM_GPU, IDLE);
        }
#endif
        break;

        case GPU_POWER_STATE_ACTIVE:
#if !CONFIG_ZEPHYR_OS
        {
            //Keep it simple to reset the GPU state before rendering every time for system robust
            _gpu_clock_on();
            GPU_REG(HAL_GFX_CMDSTATUS) = 0x0;//resets CL processor
            GPU_REG(HAL_GFX_STATUS) = 0x0;//resets the GPU
            GPU_REG(HAL_GFX_INTERRUPT) = 0x0;//set interrupt ctrl
            GPU_REG(HAL_GFX_CMDRINGSTOP) = g_nema_rb.bo.base_phys | RINGSTOP_NOTRIGGER | RINGSTOP_ENABLE;
            GPU_REG(HAL_GFX_CMDADDR) = g_nema_rb.bo.base_phys;
            GPU_REG(HAL_GFX_CMDSIZE) = g_nema_rb.bo.size;
            g_nema_rb.offset = s_graphics_gpu_env.rb_offset;
            NVIC_ClearPendingIRQ(TSI_GPU_IRQn);
            NVIC_ClearPendingIRQ(TSI_GPU_ERR_IRQn);
            NVIC_EnableIRQ(TSI_GPU_IRQn);
            NVIC_EnableIRQ(TSI_GPU_ERR_IRQn);
            GPU_REG(0x94) = 0xFFFFFFFF;         /* fix blur issue in TSC4 transition */
            /* WORK-AROUND: QSPI0 XIP mode is overrided, forced to change to Auto-Mode */
            *((volatile uint32_t *) 0xA000E180) = *((volatile uint32_t *) 0xA000E180) & 0xFFFFFFFE; //QSPI0 XIP - auto mode
            hal_pwr_mgmt_set_extra_device_state(EXTRA_DEVICE_NUM_GPU, ACTIVE);
        }
#endif
        break;

        default:break;
    }
}

/**
 *****************************************************************************************
 * @brief Create a new expandable Command List with power management
 *
 * @return instance of the new Command List
 *****************************************************************************************
 */
hal_gfx_cmdlist_t hal_gfx_cl_le_create(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    if(0 == g_cl_create_cnt)
    {
        app_graphics_gpu_set_power_state(GPU_POWER_STATE_ACTIVE);
    }
    g_cl_create_cnt++;
    GLOBAL_EXCEPTION_ENABLE();
    return hal_gfx_cl_create();
}

/**
 *****************************************************************************************
 * @brief Destroy/Free a Command List with power management
 *
 * @param[in] cl: Pointer to the Command List
 *****************************************************************************************
 */
void hal_gfx_cl_le_destroy(hal_gfx_cmdlist_t *cl)
{
    hal_gfx_cl_destroy(cl);
    GLOBAL_EXCEPTION_DISABLE();
    g_cl_create_cnt--;
    if(0 == g_cl_create_cnt)
    {
        app_graphics_gpu_set_power_state(GPU_POWER_STATE_SLEEP);
    }
    GLOBAL_EXCEPTION_ENABLE();
}

/*
 * Static Methods
 *****************************************************************************************
 */
static void _gpu_clock_on(void) {
    MCU_RET->MCU_MISC_CLK = (MCU_RET->MCU_MISC_CLK) & 0x3D;
}

static void _gpu_clock_off(void) {
    MCU_RET->MCU_MISC_CLK = (MCU_RET->MCU_MISC_CLK) | (1u << 1);
}

static void gpu_irq_handler( void *pvUnused )
{
    uint32_t irq = hal_gfx_reg_read(HAL_GFX_INTERRUPT);
    g_last_cl_id = (int)hal_gfx_reg_read(HAL_GFX_CLID);

    /* Clear the interrupt */
    hal_gfx_reg_write(HAL_GFX_INTERRUPT, 0);

    s_graphics_gpu_env.is_cmd_cplt = 1;

#if GPU_WAIT_IRQ_USING_SEMAPHORE > 0u
    _gpu_irq_sem_give();
#endif

    if(s_graphics_gpu_env.irq_evt_cb != NULL) {
        s_graphics_gpu_env.irq_evt_cb(irq);
    }

    return;
}

/*
 * Porting APIs
 *****************************************************************************************
 */

int hal_gfx_wait_irq (void)
{
#if GPU_WAIT_IRQ_USING_SEMAPHORE > 0u
    _gpu_irq_sem_take();
#else
    while( hal_gfx_reg_read(HAL_GFX_INTERRUPT) == 0U ) {
        if(1 == s_graphics_gpu_env.is_cmd_cplt) {
            break;
        } else {
            delay_us(1);
        }
    }
#endif

#if 1
    s_graphics_gpu_env.is_cmd_cplt = 0;     //TO BE CHECKED
#endif
    return 0;
}

int hal_gfx_wait_irq_cl (int cl_id)
{
    while ( g_last_cl_id < cl_id) {
        int ret = hal_gfx_wait_irq();
        g_last_cl_id = (int)hal_gfx_reg_read(HAL_GFX_CLID);
        if ( ret == -1 ) {
            return 0;
        }
        (void)ret;
    }
    return 0;
}

int32_t hal_gfx_sys_init (void)
{
    int32_t ret = 0;

    soc_register_nvic(TSI_GPU_ERR_IRQn, (uint32_t)TSI_GPU_ERR_IRQHandler);
    soc_register_nvic(TSI_GPU_IRQn, (uint32_t)TSI_GPU_IRQHandler);
    hal_gfx_reg_write(HAL_GFX_INTERRUPT, 0);

    NVIC_ClearPendingIRQ(TSI_GPU_IRQn);
    NVIC_ClearPendingIRQ(TSI_GPU_ERR_IRQn);
    NVIC_EnableIRQ(TSI_GPU_IRQn);
    NVIC_EnableIRQ(TSI_GPU_ERR_IRQn);

#ifdef USE_TSI_MALLOC
    // Map and initialize Graphics Memory
    tsi_malloc_init((void *)VMEM_BASEADDR, VMEM_BASEADDR, VMEM_SIZE, 1);
#endif

    g_nema_rb.bo = hal_gfx_buffer_create(HAL_GFX_RING_BUFFER_SIZE);
    hal_gfx_buffer_map(&g_nema_rb.bo);

    //Initialize Ring BUffer
    ret = hal_gfx_rb_init(&g_nema_rb, 1);

    if (ret) {
        return ret;
    }

    g_last_cl_id = -1;

    return 0;
}

uint32_t hal_gfx_reg_read (uint32_t reg)
{
    return *((volatile uint32_t *)(GRAPHICS_GPU_BASEADDR + reg));
}

void hal_gfx_reg_write (uint32_t reg,uint32_t value)
{
    *((volatile uint32_t *)(GRAPHICS_GPU_BASEADDR + reg)) = value;
}

void hal_gfx_reset_irq_marks(void) {
    s_graphics_gpu_env.is_cmd_cplt = 0;
}

hal_gfx_buffer_t hal_gfx_buffer_create (int size)
{
    hal_gfx_mutex_lock(MUTEX_MALLOC);

    hal_gfx_buffer_t bo;
#ifdef USE_TSI_MALLOC
    bo.base_virt = tsi_malloc((size_t)size);
#else
    bo.base_virt = pvPortMalloc((size_t)size);
#endif

    bo.base_phys = (uintptr_t) (bo.base_virt);
    bo.size      = size;
    bo.fd        = 0;

    hal_gfx_mutex_unlock(MUTEX_MALLOC);
    return bo;
}

hal_gfx_buffer_t hal_gfx_buffer_create_pool (int pool, int size)
{
    return hal_gfx_buffer_create(size);
}

void *hal_gfx_buffer_map (hal_gfx_buffer_t * bo)
{
    if(bo == NULL){
        hal_gfx_set_error(HAL_GFX_ERR_INVALID_BO);
        return NULL;
    }
    return bo->base_virt;
}

void hal_gfx_buffer_unmap (hal_gfx_buffer_t * bo)
{
    /* Nothing TODO */
    return;
}

void hal_gfx_buffer_destroy (hal_gfx_buffer_t * bo)
{
    if(bo == NULL){
        hal_gfx_set_error(HAL_GFX_ERR_INVALID_BO);
        return;
    }
    hal_gfx_mutex_lock(MUTEX_MALLOC);

#ifdef USE_TSI_MALLOC
    tsi_free(bo->base_virt);
#else
    vPortFree(bo->base_virt);
#endif

    bo->base_virt = (void *)NULL;
    bo->base_phys = 0;
    bo->size      = 0;
    bo->fd        = -1;

    hal_gfx_mutex_unlock(MUTEX_MALLOC);
}

uintptr_t hal_gfx_buffer_phys (hal_gfx_buffer_t * bo)
{
    if(bo == NULL){
        hal_gfx_set_error(HAL_GFX_ERR_INVALID_BO);
        return (uintptr_t)-1;
    }
    return bo->base_phys;
}

void hal_gfx_buffer_flush(hal_gfx_buffer_t * bo)
{
    /* Nothing TODO */
    return;
}

void * hal_gfx_host_malloc (size_t size)
{
    hal_gfx_mutex_lock(MUTEX_MALLOC);

#ifdef USE_TSI_MALLOC
    void *ptr = tsi_malloc(size);
#else
    void *ptr = pvPortMalloc(size);
#endif

    hal_gfx_mutex_unlock(MUTEX_MALLOC);
    return ptr;
}

void hal_gfx_host_free (void * ptr)
{
    hal_gfx_mutex_lock(MUTEX_MALLOC);

#ifdef USE_TSI_MALLOC
    tsi_free(ptr);
#else
    vPortFree(ptr);
#endif

    hal_gfx_mutex_unlock(MUTEX_MALLOC);
}

int hal_gfx_mutex_lock (int mutex_id)
{
#if (defined(HAL_GFX_MULTI_PROCESS) || defined(HAL_GFX_MULTI_THREAD))
    if ((enable_mutices == 1) && (mutex_id >= 0) && (mutex_id <= MUTEX_MAX)) {
        xSemaphoreTake( xMutex[mutex_id], portMAX_DELAY );
    }
#endif
    return 0;
}

int hal_gfx_mutex_unlock (int mutex_id)
{
#if (defined(HAL_GFX_MULTI_PROCESS) || defined(HAL_GFX_MULTI_THREAD))
    if ((enable_mutices == 1) && (mutex_id >= 0) && (mutex_id <= MUTEX_MAX)) {
        xSemaphoreGive( xMutex[mutex_id] );
    }
#endif
    return 0;
}

#if GPU_WAIT_IRQ_USING_SEMAPHORE > 0u
static int _gpu_irq_sem_init(void) {
#ifdef USE_OSAL
    if (OSAL_SUCCESS == osal_sema_binary_create(&s_gpu_irq_sem))
    {
        return 1;
    }
#else
    if(APP_DRV_SUCCESS == app_driver_sem_init(&s_gpu_irq_sem)) {
        return 1;
    }
#endif
    return 0;
}

static int _gpu_irq_sem_take(void) {
    s_gpu_sem_take_cnt++;
#ifdef USE_OSAL
    osal_sema_take(s_gpu_irq_sem, GPU_WAIT_TIMEOUT_MS);
#else
    app_driver_sem_pend(s_gpu_irq_sem, GPU_WAIT_TIMEOUT_MS);
#endif
    return 1;
}

static int _gpu_irq_sem_give(void) {
    s_gpu_sem_give_cnt++;
#ifdef USE_OSAL
    osal_sema_give(s_gpu_irq_sem);
#else
    app_driver_sem_post_from_isr(s_gpu_irq_sem);
#endif
    return 1;
}
#endif

/*
 * IRQ Service
 *****************************************************************************************
 */
void TSI_GPU_IRQHandler(void) {
    gpu_irq_handler(NULL);
}

void TSI_GPU_ERR_IRQHandler(void) {
    GPU_DBG_PRINTF("TSI_GPU_ERR_IRQHandler \r\n");
}

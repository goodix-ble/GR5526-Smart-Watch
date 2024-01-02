/**
 ****************************************************************************************
 *
 * @file watcher.c
 *
 * @brief CPU total calulate function implementation.
 *
 * Copyright(C) 2016-2018, Shenzhen Goodix  Technology Co., Ltd
 * All Rights Reserved
 *
 ****************************************************************************************
 */
 
 /**
 ****************************************************************************************
 * @addtogroup WATCHER
 * @{
 ****************************************************************************************
 */

 /*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "grx_hal.h"
#include "watcher.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern char *get_current_tcb_name(void);
TASK_TIME_t global_task_list[TOTAL_TASK_COUNT];
static unsigned char static_number;
static dual_timer_handle_t DUALTIM0Handle = {0};

 /*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
 
__WEAK uint32_t dual_timer_read_value(void)
{
    return ll_dual_timer_get_counter(DUALTIM0Handle.p_instance);
}

void watcher_timer_init()
{
    DUALTIM0Handle.p_instance = DUAL_TIMER0;
    DUALTIM0Handle.init.prescaler   = DUAL_TIMER_PRESCALER_DIV16;
    DUALTIM0Handle.init.counter_mode = DUAL_TIMER_COUNTERMODE_LOOP;
    DUALTIM0Handle.init.auto_reload  =  (unsigned int)-1;
    hal_dual_timer_base_init(&DUALTIM0Handle);
    hal_dual_timer_base_start_it(&DUALTIM0Handle);
}

int get_handle_by_name(const char *p_name)
{
    int idx = 0;
    for (idx=0; idx<TOTAL_TASK_COUNT; idx ++)
    {
        if (strcmp(p_name, global_task_list[idx].watcher_task_name) == 0)
        {
            return idx; 
        }
    }
    return -1;
}

void create_task_counter(char *pName)
{
    if (static_number < TOTAL_TASK_COUNT)
    {
        memcpy(global_task_list[static_number].watcher_task_name, pName, strlen(pName));
        global_task_list[static_number].task_main_time = 0x0;
        global_task_list[static_number].task_isr_time = 0x0;
        static_number ++;
    }
}

void clear_all()
{
    int idx = 0;
    for (idx=0; idx<TOTAL_TASK_COUNT; idx ++)
    {
        global_task_list[idx].task_main_time = 0x0;
        global_task_list[idx].task_isr_time = 0x0;	
    }
}

void add_isr_time(unsigned int time)
{
	int handle = get_handle_by_name(get_current_tcb_name()); 
    global_task_list[handle].task_isr_time += time;
}

void add_main_time(int handle, unsigned int time)
{
    global_task_list[handle].task_main_time += time;
}

void get_time_by_handle(int handle, unsigned int *isr_time, unsigned int *main_time)
{
    *isr_time = global_task_list[handle].task_isr_time;
    *main_time = global_task_list[handle].task_main_time;
}

void get_info_by_handle(int handle, char *pName)
{
    strcpy(pName, global_task_list[handle].watcher_task_name );
}

void get_all_time(unsigned int *total_time, unsigned int *total_isr_time)
{
    int idx = 0;
    *total_time = 0;
    *total_isr_time = 0;
    for (idx=0; idx<TOTAL_TASK_COUNT; idx ++)
    {
            *total_time += global_task_list[idx].task_main_time;
            *total_isr_time += global_task_list[idx].task_isr_time;
    }
}

void Goodix_Watcher_Task(void *arg)
{
    volatile unsigned int a;
    volatile unsigned int b;
    unsigned int total_isr_time, total_main_time;
    int index = 0;
    float time;
    unsigned int isr_used_time;
    unsigned int task_used_time;
    unsigned int task_finaly_used_time;

    uint32_t TotalRunTime;
    UBaseType_t ArraySize,x;
    TaskStatus_t *StatusArray;

    ArraySize=uxTaskGetNumberOfTasks();
    StatusArray=pvPortMalloc(ArraySize*sizeof(TaskStatus_t));
    if (StatusArray != NULL)
    {
        ArraySize = uxTaskGetSystemState((TaskStatus_t* )StatusArray, (UBaseType_t)ArraySize, (uint32_t*)&TotalRunTime);
        for (x=0;x<ArraySize;x++)
        {
            global_task_list[x].watcher_task_used = 0x1;
            memcpy(global_task_list[x].watcher_task_name, StatusArray[x].pcTaskName, strlen(StatusArray[x].pcTaskName));
        }
    }
    vPortFree(StatusArray);

    while (1)
    {
        __disable_irq();

        get_all_time(&total_main_time, &total_isr_time);
        time = (float)(total_main_time);

        printf("\f");
        for (index=0; index<TOTAL_TASK_COUNT; index ++)
        {
            if (global_task_list[index].watcher_task_used)
            {
                get_time_by_handle(index, &isr_used_time, &task_used_time);
                task_finaly_used_time = task_used_time-isr_used_time;
                printf("[%s]  cpu: %0.2f%% time:%f \r\n", global_task_list[index].watcher_task_name, (double)(task_finaly_used_time)/time*100.0, (double)task_finaly_used_time * TIMER_DIV_CONTANT);
                global_task_list[index].watcher_cpu_ratio = (double)(task_finaly_used_time)/time*100.0;
            }
        }
        clear_all();
        __enable_irq();
        vTaskDelay(1000);
    }
}


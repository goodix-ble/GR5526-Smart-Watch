#ifndef _WATCHER_H
#define _WATCHER_H

#include "FreeRTOS.h"

#define TOTAL_TASK_COUNT 10
typedef struct TASK_TIME
{
   uint8_t watcher_task_used;
//	 char mark[configMAX_TASK_NAME_LEN];
   char watcher_task_name[configMAX_TASK_NAME_LEN];
   unsigned int task_main_time;
   unsigned int task_isr_time;
   uint32_t watcher_task_main_time;
   uint32_t watcher_irs_time;
   double  watcher_cpu_ratio;
	
}TASK_TIME_t;


#define TIMER_DIV_CONTANT 0.00000025

extern void Goodix_Watcher_Task(void *arg);

void watcher_timer_init(void);

void create_task_counter(char *pName);

void clear_all(void);

extern int get_handle_by_name(const char *p_name);

//void add_isr_time(int handle, unsigned int time);

void add_main_time(int handle, unsigned int time);

void get_time_by_handle(int handle, unsigned int *isr_time, unsigned int *main_time);

void get_info_by_handle(int handle, char *pName);

void get_all_time(unsigned int *total_time, unsigned int *total_isr_time);

//extern unsigned int dual_timer_read_value(void);

#endif

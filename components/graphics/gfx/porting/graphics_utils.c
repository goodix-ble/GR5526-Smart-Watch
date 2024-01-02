#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "hal_gfx_utils.h"
#include "stdio.h"

extern uint32_t get_rtc_tick(void);

//returns time in seconds
__WEAK float hal_gfx_get_time(void)
{
    float sec = 1.0;
#ifdef ENV_USE_FREERTOS
    TickType_t ticks = xTaskGetTickCount();
    sec = (float)ticks/(float) configTICK_RATE_HZ;
#endif
    return sec;
}

__WEAK uint32_t hal_gfx_get_time_ms(void) {
#ifdef ENV_USE_FREERTOS
    return xTaskGetTickCount();
#else
    return 0;
#endif
}

static unsigned long s[] = {123456789, 362436069};
unsigned int hal_gfx_rand()
{
    uint64_t x = s[0];
    uint64_t y = s[1];
    s[0] = y;
    x ^= x << 23; // a
    s[1] = x ^ y ^ (x >> 17) ^ (y >> 26); // b, c

    return s[1] + y;
}

hal_gfx_buffer_t hal_gfx_load_file(const char *filename, int length, void *buffer)
{
    hal_gfx_buffer_t bo = {0};
    return bo;
}

int hal_gfx_save_file(const char *filename, int length, void *buffer)
{
    return -1;
}

// returns wall time in secconds
float hal_gfx_get_wall_time(void)
{
    return hal_gfx_get_time();
}

float hal_gfx_calculate_fps_ext(float start_time, uint32_t frame) {
    uint32_t cur_time;
    static float fps = 0.0;
    static uint32_t last_time = 0;

    if(frame % 5 == 0)
    {
        cur_time = hal_gfx_get_time_ms();
        fps = (frame*1000.0)/(cur_time - start_time);
        printf("fps = %.2f , gap:%d \r\n", fps, (uint32_t)cur_time - last_time);
        last_time = cur_time;
    }

    return fps;
}


void *hal_gfx_memcpy ( void * destination, const void * source, size_t num )
{
    return memcpy(destination, source, num);
}

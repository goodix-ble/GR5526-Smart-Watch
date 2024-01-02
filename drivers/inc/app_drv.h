#ifndef __APP_DRV_H__
#define __APP_DRV_H__

#include "app_drv_config.h"
#include "app_drv_error.h"

#ifndef APP_DRV_DEPRECATED
#if defined ( __CC_ARM ) || defined ( __GNUC__ )
    #define APP_DRV_DEPRECATED  __attribute__ ((deprecated))
#else
    #define APP_DRV_DEPRECATED
#endif
#endif

#ifndef APP_DRV_LOG_INFO
#if APP_DRV_LOG_LEVEL >= APP_DVR_LOG_LVL_INFO
    #define APP_DRV_LOG_INFO(format, ...) APP_DRV_LOG_INTERFACE(format, ##__VA_ARGS__)
#else
    #define APP_DRV_LOG_INFO(...)
#endif
#endif

#ifndef APP_DRV_LOG_WARN
#if APP_DRV_LOG_LEVEL >= APP_DVR_LOG_LVL_WARN
    #define APP_DRV_LOG_WARN(format, ...) APP_DRV_LOG_INTERFACE(format, ##__VA_ARGS__)
#else
    #define APP_DRV_LOG_WARN(...)
#endif
#endif

#ifndef APP_DRV_LOG_ERR
#if APP_DRV_LOG_LEVEL >= APP_DVR_LOG_LVL_ERR
    #define APP_DRV_LOG_ERR(format, ...) APP_DRV_LOG_INTERFACE(format, ##__VA_ARGS__)
#else
    #define APP_DRV_LOG_ERR(...)
#endif
#endif

#ifndef APP_DRV_ASSERT
#if APP_DRV_ASSERT_ENABLE
    #define APP_DRV_ASSERT(x)  if(!(x)) { __set_PRIMASK(1); while(1); }
#else
    #define APP_DRV_ASSERT(ignore)  ((void)0)
#endif
#endif

#endif /* __APP_DRV_H__ */
